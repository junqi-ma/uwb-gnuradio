/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * GNU Radio-independent device-timed RX plan for the native X410 live receiver.
 *
 * Design contract (docs/performance/X410_目标周期锁定与随机干扰双路径修改规划_2026-11-15.md §7/§8):
 *
 *   - Exactly one RX owner issues stream commands and calls recv.  DSP workers
 *     never touch the USRP.
 *   - Windows are queued in the future with an explicit device time and a
 *     finite sample count (STREAM_MODE_NUM_SAMPS_AND_DONE semantics).  Host
 *     timers are never used as a substitute for device-timed capture.
 *   - The lookahead is bounded.  A window already issued to the device is
 *     immutable: late feedback may only change windows not yet issued.
 *   - Start/length are expanded outward to the hardware alignment (SPC)
 *     multiple, and the EXPANDED start is what the demod coordinate mapping
 *     uses.
 *   - recv may return in several pieces; they are assembled by device time and
 *     sample count.  Partial recv, timeout, late command, overflow, device time
 *     jump and complete windows are distinct terminal states.
 *   - A gap between planned windows is `planned_gap`, not a UHD error and not
 *     fabricated continuous samples.  Missing samples inside a planned window
 *     are `unexpected_gap` and invalidate detection statistics / the target
 *     lock.
 *   - "Fewer samples uploaded" is never described as the ADC/RF stopping; only
 *     the host-visible data window and host load change.
 *
 * Coordinates: absolute 64-bit device sample indices at `sample_rate`.  The
 * device sample index is derived from the UHD time spec, never from a running
 * "how many samples did I receive" counter, so a planned stop does not shift
 * the timeline.
 */

#pragma once

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdint>
#include <deque>
#include <limits>
#include <mutex>
#include <vector>

namespace gr {
namespace uwb {
namespace core {

// ---------------------------------------------------------------------------
// Terminal states
// ---------------------------------------------------------------------------
enum class TimedRxTerminal : int {
    Complete = 0,      // full planned window received
    Partial = 1,       // burst ended early / short read at EOS
    Timeout = 2,       // recv timed out
    LateCommand = 3,   // device rejected the command time
    Overflow = 4,      // device overflow
    DeviceTimeJump = 5,// time spec went backwards or jumped
    PlannedGap = 6,    // gap between two planned windows (not an error)
    UnexpectedGap = 7, // missing samples inside a planned window
    Cancelled = 8      // dropped before issue (queue full / superseded)
};

inline const char* timed_rx_terminal_name(TimedRxTerminal t)
{
    switch (t) {
    case TimedRxTerminal::Complete:
        return "complete";
    case TimedRxTerminal::Partial:
        return "partial";
    case TimedRxTerminal::Timeout:
        return "timeout";
    case TimedRxTerminal::LateCommand:
        return "late_command";
    case TimedRxTerminal::Overflow:
        return "overflow";
    case TimedRxTerminal::DeviceTimeJump:
        return "device_time_jump";
    case TimedRxTerminal::PlannedGap:
        return "planned_gap";
    case TimedRxTerminal::UnexpectedGap:
        return "unexpected_gap";
    case TimedRxTerminal::Cancelled:
        return "cancelled";
    }
    return "unknown";
}

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------
struct TimedRxPlanConfig {
    double sample_rate = 737280000.0;
    // Hardware samples-per-clock / alignment granularity.  Start and length are
    // expanded outward to this multiple.
    size_t alignment = 4;
    // Maximum number of windows issued but not yet assembled.
    size_t max_lookahead = 4;
    // Minimum lead between command issue and the window start.
    double command_lead_s = 200e-6;
    // Device per-command sample cap (0 = unlimited).
    size_t max_samples_per_command = 0;
    // Bounded request queue (scheduler -> issuer).
    size_t request_capacity = 32;
};

// ---------------------------------------------------------------------------
// Window specification
// ---------------------------------------------------------------------------
struct TimedRxWindowSpec {
    uint64_t window_id = 0;
    uint64_t generation = 0;
    // Logical (unexpanded) request.
    int64_t logical_start = 0;
    size_t logical_samples = 0;
    // Expanded, hardware-aligned request; this is the demod coordinate origin.
    int64_t device_start = 0;
    size_t device_samples = 0;
    double device_time_s = 0.0; // device_start / sample_rate
};

struct TimedRxRequest {
    int64_t logical_start = 0;
    size_t logical_samples = 0;
    uint64_t generation = 0;
};

struct TimedRxChunk {
    int64_t device_start = 0;
    size_t count = 0;
    // Samples for this chunk (count entries).  May be null only for
    // error/timeout chunks with count == 0.
    const std::complex<int16_t>* data = nullptr;
    bool has_time_spec = false;
    double time_spec_s = 0.0;
    int error_code = 0; // 0 = none; caller maps UHD codes
    bool overflow = false;
    bool late_command = false;
    bool timeout = false;
};

struct TimedRxAssembled {
    TimedRxWindowSpec spec;
    TimedRxTerminal terminal = TimedRxTerminal::Complete;
    // Samples actually assembled (may be shorter than spec.device_samples).
    std::vector<std::complex<int16_t>> samples;
    size_t received = 0;
    // First missing sample offset inside the window, or -1.
    int64_t first_missing = -1;
    // Gap between the previous window end and this window start.
    int64_t planned_gap_samples = 0;
    uint64_t issue_delay_us = 0;
};

struct TimedRxPlanStats {
    uint64_t requested = 0;
    uint64_t issued = 0;
    uint64_t completed = 0;
    uint64_t partial = 0;
    uint64_t timeouts = 0;
    uint64_t late_commands = 0;
    uint64_t overflows = 0;
    uint64_t device_time_jumps = 0;
    uint64_t planned_gaps = 0;
    uint64_t unexpected_gaps = 0;
    uint64_t cancelled = 0;
    uint64_t request_queue_full = 0;
    uint64_t lookahead_blocked = 0;
    uint64_t alignment_expanded_samples = 0;
    int64_t planned_gap_samples = 0;
    int64_t unexpected_gap_samples = 0;
};

// ---------------------------------------------------------------------------
// Plan core
// ---------------------------------------------------------------------------
/**
 * Scheduler-side timed RX plan.
 *
 * Typical loop (single RX thread):
 *   queue_request(...)            // from the target slot scheduler
 *   while (next_command(now_s, spec)) issue_stream_cmd(spec)   // device
 *   chunk = recv(...); on_chunk(chunk)
 *   while (take_assembled(w)) deliver(w)
 */
class TimedRxPlanCore {
public:
    explicit TimedRxPlanCore(const TimedRxPlanConfig& cfg = TimedRxPlanConfig())
        : cfg_(cfg)
    {
        sanitize();
    }

    void configure(const TimedRxPlanConfig& cfg)
    {
        cfg_ = cfg;
        sanitize();
        reset();
    }

    const TimedRxPlanConfig& config() const { return cfg_; }

    void reset()
    {
        requests_.clear();
        outstanding_.clear();
        assembled_.clear();
        stats_ = TimedRxPlanStats{};
        next_window_id_ = 1;
        last_device_end_ = -1;
        last_time_s_ = -1.0;
    }

    TimedRxPlanStats stats() const { return stats_; }
    size_t outstanding() const { return outstanding_.size(); }
    size_t queued_requests() const { return requests_.size(); }

    /** Expand a logical request to the hardware-aligned device window. */
    TimedRxWindowSpec expand(const TimedRxRequest& r, uint64_t window_id) const
    {
        TimedRxWindowSpec s;
        s.window_id = window_id;
        s.generation = r.generation;
        s.logical_start = r.logical_start;
        s.logical_samples = r.logical_samples;
        const int64_t a = static_cast<int64_t>(cfg_.alignment);
        int64_t start = r.logical_start;
        int64_t end = r.logical_start + static_cast<int64_t>(r.logical_samples);
        if (a > 1) {
            start = (start >= 0) ? (start / a) * a
                                 : -(((-start) + a - 1) / a) * a;
            end = ((end + a - 1) / a) * a;
        }
        size_t n = static_cast<size_t>(std::max<int64_t>(0, end - start));
        if (cfg_.max_samples_per_command && n > cfg_.max_samples_per_command)
            n = cfg_.max_samples_per_command;
        s.device_start = start;
        s.device_samples = n;
        s.device_time_s =
            cfg_.sample_rate > 0.0
                ? static_cast<double>(start) / cfg_.sample_rate
                : 0.0;
        return s;
    }

    /**
     * Queue a future logical window.  Returns false (counted) when the bounded
     * request queue is full, so a target slot is never silently lost.
     */
    bool queue_request(const TimedRxRequest& r)
    {
        if (r.logical_samples == 0)
            return false;
        ++stats_.requested;
        if (requests_.size() >= cfg_.request_capacity) {
            ++stats_.request_queue_full;
            return false;
        }
        requests_.push_back(r);
        return true;
    }

    /**
     * Pop the next request that can be issued now.
     *
     * `device_now_s` is the current device time.  A request is issuable when
     * its expanded start is at least `command_lead_s` in the future and the
     * lookahead is not exhausted.  Requests whose window has already passed are
     * dropped as Cancelled (counted) rather than issued late.
     */
    bool next_command(double device_now_s, TimedRxWindowSpec& out)
    {
        while (!requests_.empty()) {
            if (outstanding_.size() >= cfg_.max_lookahead) {
                ++stats_.lookahead_blocked;
                return false;
            }
            TimedRxRequest r = requests_.front();
            TimedRxWindowSpec spec = expand(r, next_window_id_);
            const double start_s = spec.device_time_s;
            if (start_s < device_now_s + cfg_.command_lead_s) {
                // Too late to schedule this window; never issue a late command.
                requests_.pop_front();
                TimedRxAssembled a;
                a.spec = spec;
                a.terminal = TimedRxTerminal::Cancelled;
                a.first_missing = 0;
                push_assembled(std::move(a));
                continue;
            }
            requests_.pop_front();
            ++next_window_id_;
            ++stats_.issued;
            if (spec.device_samples >= r.logical_samples)
                stats_.alignment_expanded_samples +=
                    spec.device_samples - r.logical_samples;
            IssuedWindow iw;
            iw.spec = spec;
            iw.next_expected = spec.device_start;
            iw.samples.assign(spec.device_samples, std::complex<int16_t>{ 0, 0 });
            outstanding_.push_back(std::move(iw));
            out = spec;
            return true;
        }
        return false;
    }

    /**
     * Feed one recv result.  Chunks may be partial and out of order; they are
     * matched to the outstanding window that contains them.
     */
    void on_chunk(const TimedRxChunk& chunk)
    {
        if (chunk.device_start < 0) {
            fail_all(TimedRxTerminal::DeviceTimeJump);
            return;
        }
        if (chunk.has_time_spec) {
            if (last_time_s_ >= 0.0 &&
                chunk.time_spec_s + 1e-9 < last_time_s_) {
                fail_all(TimedRxTerminal::DeviceTimeJump);
                last_time_s_ = chunk.time_spec_s;
                return;
            }
            last_time_s_ = chunk.time_spec_s;
        }
        if (chunk.overflow) {
            fail_window_at(chunk.device_start, TimedRxTerminal::Overflow);
            return;
        }
        if (chunk.late_command) {
            fail_window_at(chunk.device_start, TimedRxTerminal::LateCommand);
            return;
        }
        if (chunk.timeout) {
            fail_window_at(chunk.device_start, TimedRxTerminal::Timeout);
            return;
        }
        if (chunk.count == 0)
            return;

        // Find the outstanding window containing this chunk start.
        for (auto& iw : outstanding_) {
            const int64_t ws = iw.spec.device_start;
            const int64_t we = ws + static_cast<int64_t>(iw.spec.device_samples);
            if (chunk.device_start + static_cast<int64_t>(chunk.count) <= ws)
                continue;
            if (chunk.device_start >= we)
                continue;
            const int64_t copy_from =
                std::max<int64_t>(chunk.device_start, ws);
            const int64_t copy_to =
                std::min<int64_t>(chunk.device_start +
                                      static_cast<int64_t>(chunk.count),
                                  we);
            if (copy_to <= copy_from)
                continue;
            const size_t dst = static_cast<size_t>(copy_from - ws);
            const size_t n = static_cast<size_t>(copy_to - copy_from);
            // Fill the hole in front of this chunk with zeros and record it.
            if (copy_from > iw.next_expected) {
                if (iw.first_missing < 0)
                    iw.first_missing = iw.next_expected - ws;
                iw.gap = true;
            }
            if (chunk.data && dst + n <= iw.samples.size()) {
                const size_t src =
                    static_cast<size_t>(copy_from - chunk.device_start);
                std::copy_n(chunk.data + src, n, iw.samples.begin() + dst);
            }
            iw.next_expected = std::max(iw.next_expected, copy_to);
            // Contiguous coverage from the window start; overlapping chunks are
            // not counted twice.
            iw.filled = static_cast<size_t>(iw.next_expected - ws);
            if (iw.next_expected >= we)
                finish(iw, iw.gap ? TimedRxTerminal::UnexpectedGap
                                  : TimedRxTerminal::Complete);
            return;
        }
        // A chunk that belongs to no outstanding window is an unexpected gap
        // relative to the plan.
        ++stats_.unexpected_gaps;
        stats_.unexpected_gap_samples += static_cast<int64_t>(chunk.count);
    }

    /**
     * End of stream / explicit burst end for the oldest outstanding window.
     * A window that is not full becomes Partial.
     */
    void flush_partial()
    {
        if (outstanding_.empty())
            return;
        auto& iw = outstanding_.front();
        finish(iw, iw.filled >= iw.spec.device_samples
                       ? TimedRxTerminal::Complete
                       : TimedRxTerminal::Partial);
    }

    /** Pop one assembled window.  Returns false when none is ready. */
    bool take_assembled(TimedRxAssembled& out)
    {
        if (assembled_.empty())
            return false;
        out = std::move(assembled_.front());
        assembled_.pop_front();
        return true;
    }

    size_t assembled_ready() const { return assembled_.size(); }

private:
    struct IssuedWindow {
        TimedRxWindowSpec spec;
        std::vector<std::complex<int16_t>> samples;
        size_t filled = 0;
        int64_t next_expected = 0;
        int64_t first_missing = -1;
        bool gap = false;
    };

    void sanitize()
    {
        if (cfg_.sample_rate <= 0.0)
            cfg_.sample_rate = 737280000.0;
        if (cfg_.alignment == 0)
            cfg_.alignment = 1;
        if (cfg_.max_lookahead == 0)
            cfg_.max_lookahead = 1;
        if (cfg_.command_lead_s < 0.0)
            cfg_.command_lead_s = 0.0;
        if (cfg_.request_capacity == 0)
            cfg_.request_capacity = 1;
    }

    void push_assembled(TimedRxAssembled&& a)
    {
        if (a.spec.device_start >= 0) {
            if (last_device_end_ >= 0 && a.spec.device_start > last_device_end_) {
                a.planned_gap_samples = a.spec.device_start - last_device_end_;
                ++stats_.planned_gaps;
                stats_.planned_gap_samples += a.planned_gap_samples;
            }
            last_device_end_ =
                a.spec.device_start +
                static_cast<int64_t>(a.spec.device_samples);
        }
        switch (a.terminal) {
        case TimedRxTerminal::Complete:
            ++stats_.completed;
            break;
        case TimedRxTerminal::Partial:
            ++stats_.partial;
            break;
        case TimedRxTerminal::Timeout:
            ++stats_.timeouts;
            break;
        case TimedRxTerminal::LateCommand:
            ++stats_.late_commands;
            break;
        case TimedRxTerminal::Overflow:
            ++stats_.overflows;
            break;
        case TimedRxTerminal::UnexpectedGap:
            ++stats_.unexpected_gaps;
            if (a.first_missing >= 0)
                stats_.unexpected_gap_samples += a.first_missing;
            break;
        case TimedRxTerminal::DeviceTimeJump:
            ++stats_.device_time_jumps;
            break;
        case TimedRxTerminal::Cancelled:
            ++stats_.cancelled;
            break;
        default:
            break;
        }
        assembled_.push_back(std::move(a));
    }

    void finish(IssuedWindow& iw, TimedRxTerminal terminal)
    {
        TimedRxAssembled a;
        a.spec = iw.spec;
        a.terminal = terminal;
        a.received = iw.filled;
        a.first_missing = iw.first_missing;
        a.samples = std::move(iw.samples);
        a.samples.resize(iw.filled);
        for (auto it = outstanding_.begin(); it != outstanding_.end(); ++it) {
            if (it->spec.window_id == iw.spec.window_id) {
                outstanding_.erase(it);
                break;
            }
        }
        push_assembled(std::move(a));
    }

    void fail_window_at(int64_t device_start, TimedRxTerminal terminal)
    {
        for (auto& iw : outstanding_) {
            const int64_t ws = iw.spec.device_start;
            const int64_t we = ws + static_cast<int64_t>(iw.spec.device_samples);
            if (device_start >= ws && device_start < we) {
                finish(iw, terminal);
                return;
            }
        }
        // No matching outstanding window: treat as a global device fault.
        fail_all(terminal);
    }

    void fail_all(TimedRxTerminal terminal)
    {
        while (!outstanding_.empty()) {
            auto& iw = outstanding_.front();
            finish(iw, terminal);
        }
    }

    TimedRxPlanConfig cfg_{};
    std::deque<TimedRxRequest> requests_;
    std::deque<IssuedWindow> outstanding_;
    std::deque<TimedRxAssembled> assembled_;
    TimedRxPlanStats stats_{};
    uint64_t next_window_id_ = 1;
    int64_t last_device_end_ = -1;
    double last_time_s_ = -1.0;
};

// ---------------------------------------------------------------------------
// Fixed-tone schedule compatibility (§8)
// ---------------------------------------------------------------------------
/**
 * One physical RX interval that may serve several logical windows.
 *
 * The fixed-tone module needs a 0.5 ms training window every 20 ms anchored to
 * device time.  When a target window overlaps a training window the physical
 * capture is merged once and each logical window references its slice, so the
 * tone estimate is never computed twice and the same samples are never
 * subtracted twice.
 */
struct PhysicalRxInterval {
    int64_t start = 0;
    size_t samples = 0;
    bool has_tone_training = false;
    bool has_target = false;
    // Logical slices inside this interval (relative offsets).
    struct Slice {
        int64_t offset = 0;
        size_t samples = 0;
        bool tone_training = false;
        bool target = false;
        uint64_t slot_index = 0;
    };
    std::vector<Slice> slices;

    int64_t end() const { return start + static_cast<int64_t>(samples); }
};

/**
 * Merge tone-training and target windows into physical RX intervals.
 *
 * `tone_period_samples` / `tone_window_samples` describe the periodic training
 * grid; `targets` are (start, samples, slot) triples.  Overlapping windows are
 * merged and keep one slice per logical purpose.  Intervals are returned sorted
 * by start and never overlap.
 */
inline std::vector<PhysicalRxInterval> merge_physical_rx_intervals(
    const std::vector<std::pair<int64_t, size_t>>& targets,
    const std::vector<uint64_t>& target_slots,
    int64_t tone_origin,
    int64_t tone_period_samples,
    size_t tone_window_samples,
    int64_t range_start,
    int64_t range_end)
{
    struct Raw {
        int64_t start;
        size_t samples;
        bool tone;
        bool target;
        uint64_t slot;
    };
    std::vector<Raw> raw;
    if (tone_period_samples > 0 && tone_window_samples > 0) {
        // First tone window at or after range_start, using independent
        // rounding per index so the grid never drifts.
        const double k0 = std::ceil(
            static_cast<double>(range_start - tone_origin) /
            static_cast<double>(tone_period_samples));
        for (int64_t k = static_cast<int64_t>(std::max(0.0, k0));; ++k) {
            const int64_t s =
                tone_origin + static_cast<int64_t>(k) * tone_period_samples;
            if (s >= range_end)
                break;
            if (s + static_cast<int64_t>(tone_window_samples) <= range_start)
                continue;
            raw.push_back({ s, tone_window_samples, true, false, 0 });
        }
    }
    for (size_t i = 0; i < targets.size(); ++i) {
        const uint64_t slot = i < target_slots.size() ? target_slots[i] : 0;
        raw.push_back({ targets[i].first, targets[i].second, false, true,
                        slot });
    }
    std::sort(raw.begin(), raw.end(),
              [](const Raw& a, const Raw& b) { return a.start < b.start; });

    std::vector<PhysicalRxInterval> out;
    for (const auto& r : raw) {
        if (r.samples == 0)
            continue;
        if (!out.empty()) {
            PhysicalRxInterval& cur = out.back();
            const int64_t r_end = r.start + static_cast<int64_t>(r.samples);
            if (r.start <= cur.end()) {
                // Overlap or adjacency: merge.
                if (r_end > cur.end())
                    cur.samples = static_cast<size_t>(r_end - cur.start);
                cur.has_tone_training = cur.has_tone_training || r.tone;
                cur.has_target = cur.has_target || r.target;
                PhysicalRxInterval::Slice sl;
                sl.offset = r.start - cur.start;
                sl.samples = r.samples;
                sl.tone_training = r.tone;
                sl.target = r.target;
                sl.slot_index = r.slot;
                cur.slices.push_back(sl);
                continue;
            }
        }
        PhysicalRxInterval iv;
        iv.start = r.start;
        iv.samples = r.samples;
        iv.has_tone_training = r.tone;
        iv.has_target = r.target;
        PhysicalRxInterval::Slice sl;
        sl.offset = 0;
        sl.samples = r.samples;
        sl.tone_training = r.tone;
        sl.target = r.target;
        sl.slot_index = r.slot;
        iv.slices.push_back(sl);
        out.push_back(std::move(iv));
    }
    return out;
}

} // namespace core
} // namespace uwb
} // namespace gr
