/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Device-timed replay source: timed_burst without hardware.
 */
#ifndef GR_UWB_TIMED_REPLAY_SOURCE_H
#define GR_UWB_TIMED_REPLAY_SOURCE_H

#include "live_rx_config.h"

#include <algorithm>
#include <complex>
#include <cstdint>
#include <fstream>
#include <mutex>
#include <vector>

#include <gnuradio/uwb/uwb_timed_rx_plan.h>
#include <gnuradio/uwb/uwb_uhd_sc16_pipeline.h>

namespace openuwb {
namespace live {

// ---------------------------------------------------------------------------
// Device-timed replay source (timed_burst without hardware).
//
// The file is a continuous timeline; this source only delivers the planned
// windows and marks the space between them as a planned gap.  It is the
// timestamped fake source the plan requires for exercising the plan/assembly
// and error paths without claiming a timed UHD board run.
// ---------------------------------------------------------------------------
class TimedReplaySc16Source : public pipeline::ISc16StreamSource {
public:
    TimedReplaySc16Source(const std::string& file, double rate,
                          const core::TimedRxPlanConfig& cfg, bool paced = true)
        : file_(file), rate_(rate), plan_(cfg), paced_(paced) {}
    bool open() override {
        f_.open(file_, std::ios::binary | std::ios::ate);
        if (!f_) return false;
        const auto bytes = f_.tellg();
        if (bytes <= 0) return false;
        total_ = size_t(bytes) / sizeof(std::complex<int16_t>);
        eof_ = false;
        t0_ = Clock::now();
        return true;
    }
    void close() override { if (f_.is_open()) f_.close(); eof_ = true; }
    double actual_rx_rate() const override { return rate_; }
    bool is_eof() const override { return eof_; }
    // The scheduler thread queues windows while the decorator's reader thread
    // assembles them, so every access to the plan is serialised.  Holding the
    // lock across the file read is deliberate: a planned window is far larger
    // than one recv block, and the queue side must never observe a half-built
    // plan.
    bool queue_window(int64_t start, size_t samples, uint64_t generation) {
        core::TimedRxRequest r;
        r.logical_start = start;
        r.logical_samples = samples;
        r.generation = generation;
        std::lock_guard<std::mutex> l(plan_mutex_);
        return plan_.queue_request(r);
    }
    core::TimedRxPlanStats plan_stats() {
        std::lock_guard<std::mutex> l(plan_mutex_);
        return plan_.stats();
    }
    uint64_t delivered_samples() const { return delivered_.load(); }
    size_t read_samples(std::complex<int16_t>* dest, size_t max_samples,
                        pipeline::StreamReadMeta& meta) override {
        meta = pipeline::StreamReadMeta{};
        if (!f_.is_open() || max_samples == 0) { eof_ = true; meta.is_eof = true; return 0; }
        std::lock_guard<std::mutex> l(plan_mutex_);
        // Serve the tail of an assembled window first: a planned window is far
        // larger than one recv block.
        if (out_valid_) {
            const size_t n = std::min(max_samples, out_buf_.size() - out_off_);
            std::copy_n(out_buf_.begin() + static_cast<long>(out_off_), n, dest);
            meta.start_sample_abs = out_start_ + int64_t(out_off_);
            meta.has_time_spec = true;
            meta.time_spec_s = double(meta.start_sample_abs) / rate_;
            meta.is_gap = (out_off_ == 0) && out_gap_;
            meta.gap_samples = (out_off_ == 0) ? out_gap_samples_ : 0;
            meta.error_code = out_err_;
            out_off_ += n;
            if (out_off_ >= out_buf_.size())
                out_valid_ = false;
            delivered_.fetch_add(n);
            return n;
        }
        std::vector<std::complex<int16_t>> chunk(65536);
        // Issue every request whose window is still in the future.  The replay
        // has no real-time constraint, so "now" is placed just before the
        // requested start and the lead time is always satisfied.
        for (;;) {
            core::TimedRxWindowSpec spec;
            if (!plan_.next_command(next_issue_s_ - 1.0, spec)) break;
            // Read the planned window from the file and deliver it in chunks.
            size_t remaining = spec.device_samples;
            int64_t pos = spec.device_start;
            while (remaining > 0) {
                const size_t n = std::min(remaining, chunk.size());
                const size_t got = read_file(pos, chunk.data(), n);
                core::TimedRxChunk c;
                c.device_start = pos;
                c.count = got;
                c.data = chunk.data();
                c.has_time_spec = true;
                c.time_spec_s = double(pos) / rate_;
                plan_.on_chunk(c);
                if (got < n) { plan_.flush_partial(); break; }
                pos += int64_t(got);
                remaining -= got;
            }
            next_issue_s_ = double(spec.device_start + int64_t(spec.device_samples)) / rate_;
        }
        core::TimedRxAssembled w;
        if (!plan_.take_assembled(w)) {
            if (at_file_end_) { eof_ = true; meta.is_eof = true; }
            return 0;
        }
        // Paced replay: the fake device delivers a window no earlier than its
        // device time, so queue depth and drops reflect the plan rather than
        // the speed of a file read.  Unpaced mode is for plan-only QA.
        if (paced_) {
            const auto due = t0_ + std::chrono::duration_cast<Clock::duration>(
                std::chrono::duration<double>(w.spec.device_time_s));
            std::this_thread::sleep_until(due);
        }
        out_buf_ = std::move(w.samples);
        out_start_ = w.spec.device_start;
        out_gap_ = w.planned_gap_samples > 0;
        out_gap_samples_ = w.planned_gap_samples;
        out_err_ = int(w.terminal);
        out_off_ = 0;
        out_valid_ = !out_buf_.empty();
        if (!out_valid_)
            return 0;
        const size_t n = std::min(max_samples, out_buf_.size());
        std::copy_n(out_buf_.begin(), n, dest);
        meta.start_sample_abs = out_start_;
        meta.has_time_spec = true;
        meta.time_spec_s = w.spec.device_time_s;
        meta.is_gap = out_gap_;
        meta.gap_samples = out_gap_samples_;
        meta.error_code = out_err_;
        out_off_ = n;
        if (out_off_ >= out_buf_.size())
            out_valid_ = false;
        delivered_.fetch_add(n);
        return n;
    }
private:
    size_t read_file(int64_t start, std::complex<int16_t>* dst, size_t n) {
        if (start < 0 || size_t(start) >= total_) { at_file_end_ = true; return 0; }
        const size_t avail = total_ - size_t(start);
        n = std::min(n, avail);
        f_.clear();
        f_.seekg(std::streamoff(start) * std::streamoff(sizeof(std::complex<int16_t>)));
        f_.read(reinterpret_cast<char*>(dst),
                std::streamsize(n * sizeof(std::complex<int16_t>)));
        const size_t got = size_t(f_.gcount()) / sizeof(std::complex<int16_t>);
        if (got < n || size_t(start) + got >= total_) at_file_end_ = true;
        return got;
    }
    std::string file_;
    double rate_;
    std::mutex plan_mutex_;
    core::TimedRxPlanCore plan_;
    std::ifstream f_;
    size_t total_ = 0;
    bool eof_ = false;
    bool at_file_end_ = false;
    double next_issue_s_ = 0.0;
    std::vector<std::complex<int16_t>> out_buf_;
    int64_t out_start_ = 0;
    size_t out_off_ = 0;
    bool out_valid_ = false;
    bool out_gap_ = false;
    int64_t out_gap_samples_ = 0;
    int out_err_ = 0;
    std::atomic<uint64_t> delivered_{0};
    bool paced_ = true;
    Clock::time_point t0_{};
};

} // namespace live
} // namespace openuwb

#endif /* GR_UWB_TIMED_REPLAY_SOURCE_H */
