/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Structured console telemetry for uwb_dw3000_live.
 *
 * ConsoleReporter is the ONLY writer of the receiver's stdout protocol
 * (START / STREAM_STARTED / PACKET / SUMMARY / FINAL / COST_TOTAL /
 * LOCK_STATS / ...).  The line formats, key order and bounded queue semantics
 * are a downstream contract pinned by tools/rx/qa_live_dw3000_console.py:
 * do not add prefixes, timestamps or blank lines here.
 */
#ifndef GR_UWB_CONSOLE_REPORTER_H
#define GR_UWB_CONSOLE_REPORTER_H

#include "live_rx_config.h"

#include <algorithm>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <deque>
#include <iomanip>
#include <iostream>
#include <map>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <gnuradio/uwb/uwb_demod_pipeline.h>
#include <gnuradio/uwb/uwb_fixed_tone_cancel_source.h>
#include <gnuradio/uwb/uwb_live_packet_processor.h>

namespace openuwb {
namespace live {

struct PacketCost {
    double queue_us=0, resample_us=0, demod_us=0, retry_us=0, compute_us=0, cpu_us=0;
    size_t worker=0;
};

inline bool crc_ok(const std::vector<uint8_t>& b) {
    return live::verify_packet_crc(b);
}

inline const char* status_name(demod::DemodStatus s) {
    static const char* names[] = {"OK", "INVALID_INPUT", "TIMING_FAILED", "CFO_FAILED",
        "SFD_FAILED", "PHR_FAILED", "PAYLOAD_FAILED", "FCS_FAILED", "QUEUE_FULL",
        "INTERNAL_ERROR", "CIR_FAILED"};
    const int i = int(s); return i >= 0 && i <= 10 ? names[i] : "UNKNOWN";
}

// Every completed attempt (including queue/gap drops) contributes to a batch.
// Printing is isolated and bounded: a slow terminal cannot block the RX thread.
struct ConsoleReporter {
    struct SubmissionSummary {
        uint64_t acq_seeds = 0;
        uint64_t submit_attempts = 0;
        uint64_t admitted = 0;
        uint64_t dropped_queue = 0;
    };
    std::mutex mutex;
    std::condition_variable cv;
    std::deque<std::string> lines;
    uint64_t count = 0, pass = 0, batch_pass = 0, omitted = 0;
    uint64_t demod_completed = 0;
    uint64_t full_retries = 0, batch_retries = 0;
    uint64_t received = 0, source_errors = 0, raw_drops = 0, gaps = 0, models = 0;
    size_t queued = 0, queue_peak = 0, interval;
    std::map<std::string, size_t> batch_fail;
    // Per-batch timing, so `--summary N` alone is enough to watch the pipeline:
    // at a 1 ms packet period N=500 is one report per 0.5 s.
    std::vector<float> batch_compute, batch_resample, batch_demod;
    bool done = false;
    std::thread thread;
    bool print_packets = true;
    explicit ConsoleReporter(size_t n, bool packets = true)
        : interval(n), print_packets(packets)
    { thread = std::thread([this]{ loop(); }); }
    ~ConsoleReporter() { finish(); }
    void line_locked(std::string s) {
        if (lines.size() < 4096) lines.push_back(std::move(s)); else ++omitted;
        cv.notify_one();
    }
    void info(std::string s) { std::lock_guard<std::mutex> l(mutex); line_locked(std::move(s)); }
    static void stat(std::vector<float>& v, float& mean, float& p95)
    {
        if (v.empty()) { mean = 0; p95 = 0; return; }
        std::sort(v.begin(), v.end());
        double sum = 0;
        for (float x : v) sum += x;
        mean = float(sum / double(v.size()));
        p95 = v[std::min(v.size() - 1, size_t(0.95 * double(v.size())))];
    }
    void batch_locked(const std::string& prefix, size_t n) {
        std::ostringstream s;
        s << prefix << " completed=" << count << " batch=" << n
          << " batch_fcs=" << batch_pass << '/' << n
          << " batch_pct=" << std::fixed << std::setprecision(1) << 100.0*batch_pass/n
          << " total_fcs=" << pass << '/' << count
          << " total_pct=" << 100.0*pass/count
          << " full_retry=" << batch_retries
          << " raw_drop_samples=" << raw_drops << " source_errors=" << source_errors
          << " gaps=" << gaps << " models=" << models
          << " queued=" << queued << " queue_peak=" << queue_peak
          << " output_omitted=" << omitted;
        float cm, cp, rm, rp, dm, dp;
        stat(batch_compute, cm, cp);
        stat(batch_resample, rm, rp);
        stat(batch_demod, dm, dp);
        s << std::fixed << std::setprecision(1)
          << " compute_mean=" << cm << " compute_p95=" << cp
          << " resample_mean=" << rm << " resample_p95=" << rp
          << " demod_mean=" << dm << " demod_p95=" << dp;
        for (const auto& v : batch_fail) s << ' ' << v.first << '=' << v.second;
        line_locked(s.str()); batch_pass = 0; batch_retries = 0; batch_fail.clear();
        batch_compute.clear(); batch_resample.clear(); batch_demod.clear();
    }
    void result(uint64_t id, int64_t seed, const std::string& state,
                const demod::DemodResult* r = nullptr, double latency_ms = 0,
                bool full_retry = false, const PacketCost* cost = nullptr,
                const std::string& tag = std::string()) {
        const bool good = r && r->status == demod::DemodStatus::Success
            && r->payload.fcs_pass && crc_ok(r->payload.bytes);
        std::lock_guard<std::mutex> l(mutex);
        ++count; pass += good; batch_pass += good;
        if (r) ++demod_completed;
        full_retries += full_retry; batch_retries += full_retry;
        if (!good) ++batch_fail[state == "OK" ? "CRC_CHECK_FAILED" : state];
        std::ostringstream s;
        s << std::fixed << std::setprecision(3) << "PACKET id=" << id
          << " time_ms=" << seed / (fs / 1000) << " status="
          << (state == "OK" && !good ? "CRC_CHECK_FAILED" : state);
        if (!tag.empty()) s << ' ' << tag;
        if (r) s << " sync=" << r->timing.detected_peaks
                 << " sync_folded=" << r->timing.sync_peaks_folded
                 << " sync_grid=" << r->timing.cir_grid_repetitions
                 << " sfd=" << r->sfd.ok << " phr=" << r->phr.ok
                 << " len=" << r->phr.psdu_length << " cfo_khz=" << r->cfo.cfo_hz / 1000
                 << " fit=" << r->cfo.peaks_used << " fcs=" << r->payload.fcs_pass
                 << " crc=" << crc_ok(r->payload.bytes) << " latency_ms=" << latency_ms;
        if (r) s << " full_retry=" << full_retry;
        if (r) s << " sync_strategy=" << demod::sync_strategy_name(r->sync_strategy)
                 << " roi=" << r->timing.bounded_roi_samples
                 << " bounded_ok=" << r->timing.bounded_success
                 << " bounded_fail=" << r->timing.bounded_failures
                 << " legacy_scans=" << r->timing.bounded_legacy_scans
                 << " backtrack=" << r->timing.backtrack_symbols
                 << " grid_anch=" << r->timing.grid_anchor_hits
                 << " grid_anch_try=" << r->timing.grid_anchor_attempts
                 << " grid_pts=" << r->timing.grid_single_points
                 << " grid_valid=" << r->timing.grid_single_valid
                 << " grid_fb=" << r->timing.grid_fallbacks
                 << " tail_probe=" << r->timing.tail_probe_attempts
                 << " tail_hit=" << r->timing.tail_probe_hits
                 << " tail_k=" << r->timing.tail_probe_k
                 << std::fixed << std::setprecision(3)
                 << " tail_best=" << r->timing.tail_probe_best_metric
                 << " tail_raw=" << r->timing.tail_probe_raw_max
                 << " inner_raw=" << r->timing.inner_probe_raw_max;
        // Per-stage decomposition of the demod.  Parents and children are never
        // summed together; `stage_total_us` is the authoritative total.
        if (r) s << " st_total=" << r->stage_total_us
                 << " st_timing=" << r->stage_timing_us
                 << " timing_coarse=" << r->timing.coarse_search_us
                 << " timing_fine=" << r->timing.fine_track_us
                 << " timing_seed=" << r->timing.seed_search_us
                 << " timing0_coarse=" << r->timing.initial_coarse_search_us
                 << " timing0_fine=" << r->timing.initial_fine_track_us
                 << " st_cfo=" << r->stage_cfo_us
                 << " st_sfd=" << r->stage_sfd_us
                 << " st_cir=" << r->stage_cir_us
                 << " st_nssfd=" << r->stage_ns_sfd_us
                 << " st_phr=" << r->stage_phr_us
                 << " st_payload=" << r->stage_payload_us
                 << " st_fallback=" << r->stage_softchip_fallback_us
                 << " st_fb_count=" << r->stage_softchip_fallbacks
                 << " ondemand_ext=" << r->cir.on_demand_extended
                 << " ondemand_fb=" << r->cir.on_demand_fallbacks
                 << " chips=" << r->cir.soft_chip_count
                 // Measured chip-grid offset in ppm. The fused gate limits
                 // the accumulated drift across a frame.
                 << " spc_ppm=" << (r->cir.samples_per_chip > 0.0
                                        ? (r->cir.samples_per_chip / 2.0 - 1.0) * 1e6
                                        : 0.0)
                 << " taps=" << r->cir.cir_filter_taps.size()
                 << " grid_reject=" << r->stage_grid_rejects
                 << " st_fused=" << r->stage_fused_attempts
                 // Payload children (already inside st_payload; never added to
                 // st_total).  soft+viterbi+rs+crc+other equals the in-function
                 // payload interval, so st_payload minus their sum is only the
                 // call/lap overhead.  pay_fused_calls / pay_legacy_calls show
                 // which payload backend ran and therefore how much of
                 // st_payload the children actually cover.
                 << " pay_soft=" << r->stage_payload_soft_us
                 << " pay_vit=" << r->stage_payload_viterbi_us
                 << " pay_rs=" << r->stage_payload_rs_us
                 << " pay_crc=" << r->stage_payload_crc_us
                 << " pay_other=" << r->stage_payload_other_us
                 << " pay_derot=" << r->stage_payload_derotate_us
                 << " pay_fused_calls=" << r->stage_payload_fused_calls
                 << " pay_legacy_calls=" << r->stage_payload_legacy_calls
                 // SFD children (already inside st_sfd; never added again).
                 << " sfd_boot=" << r->stage_sfd_bootstrap_us
                 << " sfd_tail=" << r->stage_sfd_tail_retrack_us
                 << " sfd_cforef=" << r->stage_sfd_cfo_refit_us
                 << " sfd_final=" << r->stage_sfd_final_us
                 << " sfd_other=" << r->stage_sfd_anchor_other_us
                 << " tail_fb=" << r->stage_tail_retrack_fallbacks
                 // SFD-anchored CFO refit diagnostics: the two estimates, the
                 // span the second rotation covered, and why it was skipped.
                 // cfo_act: 0=legacy refit+rotate, 1=auto skip, 2=forced skip,
                 //          3=requested skip but fell back.
                 << std::fixed << std::setprecision(1)
                 << " cfo_init_hz=" << r->cfo_initial_hz
                 << " cfo_refit_hz=" << r->cfo_refit_hz
                 << " cfo_init_pk=" << r->cfo_initial_peaks
                 << " cfo_refit_pk=" << r->cfo_refit_peaks
                 << " cfo_refit_span=" << r->cfo_refit_span_samples
                 << " cfo_rot2_us=" << r->cfo_refit_rotate_us
                 << " cfo_drift_rad=" << r->cfo_refit_phase_drift_rad
                 << " cfo_rot_passes=" << r->stage_cfo_rotation_passes
                 << " cfo_rot_samples=" << r->stage_cfo_rotated_samples
                 // In-situ cost of the derotation kernel itself: total rotation
                 // microseconds over all passes divided by the samples rotated.
                 // Diagnostic only (no logic depends on it); it is the number to
                 // compare when the kernel implementation changes.
                 << std::setprecision(3)
                 << " cfo_rot_ns_per_sample="
                 << (r->stage_cfo_rotated_samples > 0
                         ? 1000.0 * static_cast<double>(r->stage_cfo_rotate_us) /
                               static_cast<double>(r->stage_cfo_rotated_samples)
                         : 0.0)
                 << " cfo_act=" << r->cfo_refit_action
                 << " sfd_win=" << r->sfd.search_windows
                 << " sfd_coarse=" << r->sfd.coarse_correlations
                 << " sfd_fine=" << r->sfd.fine_correlations
                 // Link quality proxies: correlation metrics, not energies.
                 << std::fixed << std::setprecision(4)
                 << " tmetric=" << r->timing.metric
                 << " sfdmetric=" << r->sfd.metric
                 << " cirmetric=" << r->cir.cir_peak_metric
                 << " phr_corr=" << r->phr.secded_corrected
                 << " phr_unc=" << r->phr.secded_uncorrectable
                 << " psdu=";
        if (r) {
            char hex[3 * 12 + 1];
            const size_t m = std::min<size_t>(r->payload.bytes.size(), 12);
            for (size_t q = 0; q < m; ++q)
                std::snprintf(hex + 3 * q, 4, "%02x ", r->payload.bytes[q]);
            hex[3 * m] = 0;
            s << hex;
        }
        if (cost) s << " worker=" << cost->worker << " queue_us=" << cost->queue_us
                    << " resample_us=" << cost->resample_us << " demod_us=" << cost->demod_us
                    << " retry_us=" << cost->retry_us << " compute_us=" << cost->compute_us
                    << " cpu_us=" << cost->cpu_us;
        if (cost) {
            batch_compute.push_back(float(cost->compute_us));
            batch_resample.push_back(float(cost->resample_us));
            batch_demod.push_back(float(cost->demod_us));
        }
        // Printing one line per packet is ~1000 lines/s at a 1 ms period; that
        // I/O is measurable.  `--print summary` keeps only the periodic report.
        if (print_packets) line_locked(s.str());
        if (count % interval == 0) batch_locked("SUMMARY", interval);
    }
    void snapshot(const pipeline::FixedToneCancelStats& st, uint64_t gap_count, size_t q) {
        std::lock_guard<std::mutex> l(mutex);
        received = st.source_samples; source_errors = st.source_error_reads;
        raw_drops = st.dropped_samples; models = st.model_updates; gaps = gap_count;
        queued = q; queue_peak = std::max(queue_peak, q);
    }
    void finish() { finish(SubmissionSummary{}); }
    void finish(const SubmissionSummary& sub) {
        {
            std::lock_guard<std::mutex> l(mutex);
            if (done) return;
            if (count % interval) batch_locked("SUMMARY_PARTIAL", count % interval);
            std::ostringstream s;
            s << "FINAL completed=" << count << " fcs=" << pass << '/' << count
              << " raw_samples=" << received << " raw_drop_samples=" << raw_drops
              << " source_errors=" << source_errors << " gaps=" << gaps
              << " models=" << models << " output_omitted=" << omitted
              << " full_retry=" << full_retries
              << " acq_seeds=" << sub.acq_seeds
              << " submit_attempts=" << sub.submit_attempts
              << " admitted=" << sub.admitted
              << " dropped_queue=" << sub.dropped_queue
              << " demod_completed=" << demod_completed
              << " fcs_ok=" << pass;
            // Workers and RX are stopped here. Reserve one final line even if
            // normal packet output filled the bounded console queue.
            lines.push_back(s.str()); done = true;
        }
        cv.notify_all(); if (thread.joinable()) thread.join();
    }
    void loop() {
        auto next = Clock::now() + std::chrono::seconds(1);
        std::unique_lock<std::mutex> l(mutex);
        for (;;) {
            cv.wait_until(l, next, [&]{ return done || !lines.empty(); });
            while (!lines.empty()) {
                auto s = std::move(lines.front()); lines.pop_front();
                l.unlock(); std::cout << s << std::endl; l.lock();
            }
            if (done) return;
            if (Clock::now() >= next) {
                std::ostringstream s;
                s << "HEARTBEAT completed=" << count << " fcs=" << pass
                  << " raw_samples=" << received << " queued=" << queued
                  << " raw_drop_samples=" << raw_drops << " source_errors=" << source_errors
                  << " gaps=" << gaps << " output_omitted=" << omitted;
                l.unlock(); std::cout << s.str() << std::endl; l.lock();
                next = Clock::now() + std::chrono::seconds(1);
            }
        }
    }
};

inline std::string format_request_tag(const live::PacketRequest& req,
                                      bool include_residual = false,
                                      int64_t residual = 0) {
    std::ostringstream tag;
    if (req.target_slot) {
        tag << "path=target slot=" << req.slot_index
            << " gen=" << req.generation
            << " pred=" << req.seed
            << " win=" << req.window_start
            << " guard=" << req.guard;
        if (include_residual)
            tag << " residual=" << residual;
    } else if (req.slot_index == std::numeric_limits<uint64_t>::max()) {
        tag << "path=acquire guard=" << req.guard;
    } else {
        tag << "path=interference slot=" << req.slot_index
            << " guard=" << req.guard
            << " offset=" << (req.seed - req.window_start);
    }
    return tag.str();
}

/// The --help text.  The only direct stdout literal in the receiver.
void print_live_help();

} // namespace live
} // namespace openuwb

#endif /* GR_UWB_CONSOLE_REPORTER_H */
