/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * UwbRadarCirEstimator — message-PDU monostatic radar CIR block (Radar Step 7).
 *
 * Block type: gr::block, zero stream ports, message PDU only (same pattern as
 * UwbRealtimeDemodulator).  The "rx" handler only validates the PDU, predicts
 * the SFD position from the radar metadata contract and enqueues an immutable
 * job into a bounded FIFO.  A single worker thread calls the header-only
 * radar core (radar_cir_one: SFD search → SYNC backtrack → CIR) with a
 * per-block scratch prepared at make(); the hot path performs no scratch
 * allocation.  A single worker keeps pulse ordering.
 *
 * Input (port "rx"): cons(meta, c32vector) at the 998.4 MS/s work rate.
 * Required metadata: pulse_id, sample_rate (work rate), pre_guard_samples,
 * sync_repetitions / sfd_mode / code_index (must match the prepared profile),
 * and a calibration delay: calibration_delay_work_samples when present,
 * otherwise calibration_delay_native_samples is used as-is (direct 998.4
 * PDUs without resampler mapping).  The predicted SFD position is
 *
 *   predicted_sfd = window_start_sample (0 if absent, 65/48 mapped)
 *                   + pre_guard + round(calibration_delay_work)
 *                   + sync_repetitions * kQm35SamplesPerSymbol
 *
 * computed with checked integer math; any overflow is an invalid PDU.
 * window_start_sample is part of the unified 65/48 coordinate convention:
 * the resampler emits the group-delay-centered window start
 * (map(window_start_native)) while pre_guard_samples is mapped relative to
 * it, so the prediction adds the window start back.  zero_delay_tap is
 * cir_pre: the core's CIR axis origin is the predicted SYNC origin (which
 * includes the calibration delay), so a path at exactly the calibrated
 * delay lands at cir_pre and the calibrated leakage peak reads
 * peak_tap - zero_delay_tap = +2 (the known pulse-shape offset).
 *
 * Output: every enqueued job produces exactly one PDU on "cir":
 *   cons(meta, c32vector raw taps) — empty vector for failed frames.
 *   meta carries status ("ok"/"sfd_failed"/"timing_failed"/"cir_failed"/
 *   "internal_error"), full lineage and, when emit_normalized is set and the
 *   frame is ok, the L2-normalized taps under "normalized_taps" (c32vector).
 * Failure frames never contain taps.  Invalid PDUs (bad dtype/rate/profile,
 * missing metadata, empty payload) are never enqueued; they only produce a
 * "status" message and bump invalid_inputs.
 */

#ifndef INCLUDED_GNURADIO_UWB_UWB_RADAR_CIR_ESTIMATOR_BLOCK_H
#define INCLUDED_GNURADIO_UWB_UWB_RADAR_CIR_ESTIMATOR_BLOCK_H

#include <gnuradio/block.h>
#include <gnuradio/gr_complex.h>
#include <gnuradio/uwb/api.h>
#include <gnuradio/uwb/uwb_radar_cir_core.h>
#include <pmt/pmt.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace gr {
namespace uwb {

class UWB_API UwbRadarCirEstimator : public gr::block
{
public:
    using sptr = std::shared_ptr<UwbRadarCirEstimator>;

    /**
     * \param template_path   CF32 file with one pulse-shaped SYNC symbol
     *                        (kQm35SamplesPerSymbol samples at 998.4 MS/s),
     *                        e.g. the first 1016 samples of the 998.4 TX
     *                        packet golden.
     * \param sync_repetitions SYNC repetitions of the TX profile (32/64/128).
     * \param sfd_mode         SFD mode of the TX profile ("4z2", ...).
     * \param code_index       HRP preamble code of the TX profile (9..12).
     * \param cir_pre / cir_post  CIR tap window (pre + post taps at 998.4).
     * \param cir_skip_initial SYNC repetitions skipped before CIR averaging.
     * \param cir_repetitions  max repetitions averaged; 0 = sync_reps - skip.
     * \param sfd_search_margin / sync_refine_margin  bounded search margins.
     * \param sfd_threshold / sync_refine_threshold  detection thresholds.
     * \param emit_normalized  attach L2-normalized taps to ok frames.
     * \param queue_capacity   bounded job queue depth (must be > 0).
     */
    static sptr make(const std::string& template_path,
                     size_t sync_repetitions = 64,
                     const std::string& sfd_mode = "4z2",
                     size_t code_index = 9,
                     size_t cir_pre = 16,
                     size_t cir_post = 100,
                     size_t cir_skip_initial = 10,
                     size_t cir_repetitions = 0,
                     int64_t sfd_search_margin = 64,
                     int64_t sync_refine_margin = 8,
                     float sfd_threshold = 0.3f,
                     float sync_refine_threshold = 0.3f,
                     bool emit_normalized = true,
                     size_t queue_capacity = 64);

    ~UwbRadarCirEstimator() override;

    const std::string& template_path() const { return d_template_path_; }
    size_t sync_repetitions() const { return d_cfg_.sync_repetitions; }
    const std::string& sfd_mode() const { return d_sfd_mode_; }
    size_t code_index() const { return d_cfg_.code_index; }
    size_t cir_pre() const { return d_cfg_.cir_pre; }
    size_t cir_post() const { return d_cfg_.cir_post; }
    size_t cir_skip_initial() const { return d_cfg_.cir_skip_initial; }
    size_t cir_repetitions() const { return d_cfg_.cir_repetitions; }
    int64_t sfd_search_margin() const { return d_cfg_.sfd_search_margin; }
    int64_t sync_refine_margin() const { return d_cfg_.sync_refine_margin; }
    bool emit_normalized() const { return d_emit_normalized_; }
    size_t queue_capacity() const { return d_queue_capacity_; }

    uint64_t pdus_received() const;
    uint64_t pdus_enqueued() const;
    uint64_t pdus_completed() const;   // core status ok
    uint64_t pdus_failed() const;      // core status not ok (still published)
    uint64_t pdus_dropped() const;     // queue full
    uint64_t invalid_inputs() const;   // rejected in handler, not enqueued
    uint64_t worker_exceptions() const;
    size_t queue_depth() const;
    size_t queue_high_watermark() const;

    // Service time (radar_cir_one call) statistics in µs.
    uint64_t service_mean_us() const;
    uint64_t service_p95_us() const;
    uint64_t service_p99_us() const;
    uint64_t service_max_us() const;
    void service_histogram(uint64_t buckets[64], uint64_t* overflow) const;

    bool drained() const;
    void drain();
    void reset_stats();

    bool start() override;
    bool stop() override;

    // Public for gnuradio::make_block_sptr; use make().
    UwbRadarCirEstimator(const std::string& template_path,
                         size_t sync_repetitions,
                         const std::string& sfd_mode,
                         size_t code_index,
                         size_t cir_pre,
                         size_t cir_post,
                         size_t cir_skip_initial,
                         size_t cir_repetitions,
                         int64_t sfd_search_margin,
                         int64_t sync_refine_margin,
                         float sfd_threshold,
                         float sync_refine_threshold,
                         bool emit_normalized,
                         size_t queue_capacity);

private:
    struct Job {
        uint64_t pulse_id = 0;
        uint64_t schedule_index = 0;
        int64_t predicted_sfd = -1;
        int64_t zero_delay_tap = -1;
        double calibration_delay_work = 0.0;
        pmt::pmt_t meta = pmt::PMT_NIL;    // input meta dict (immutable)
        pmt::pmt_t samples = pmt::PMT_NIL; // c32vector (immutable shared ref)
        std::chrono::steady_clock::time_point enqueued_at;
    };

    void handle_rx(pmt::pmt_t msg);
    bool enqueue(Job&& job);
    void worker_loop();
    void publish_frame(const Job& job,
                       const radar::RadarCirResult& r,
                       uint64_t queue_us,
                       uint64_t service_us);
    void publish_status(const std::string& event, pmt::pmt_t extra = pmt::PMT_NIL);
    void snapshot_stats(pmt::pmt_t& meta);
    void record_service_time(uint64_t us);
    uint64_t service_percentile_us(double pct) const;

    static std::vector<gr_complex> load_cf32_file(const std::string& path);
    static const char* status_to_string(radar::RadarCirStatus s);

    // Immutable after construction (worker only reads).
    std::string d_template_path_;
    std::string d_sfd_mode_; // owned copy; d_cfg_.sfd_mode points into it
    radar::RadarCirConfig d_cfg_;
    radar::RadarCirCoreScratch d_scratch_;
    bool d_emit_normalized_ = true;
    size_t d_queue_capacity_ = 0;

    // Job queue.
    mutable std::mutex d_queue_mutex_;
    std::condition_variable d_queue_cv_;
    std::deque<Job> d_queue_;
    std::atomic<bool> d_stop_{ false };
    std::thread d_worker_;
    std::atomic<bool> d_worker_busy_{ false };

    // Counters.
    std::atomic<uint64_t> d_received_{ 0 };
    std::atomic<uint64_t> d_enqueued_{ 0 };
    std::atomic<uint64_t> d_completed_{ 0 };
    std::atomic<uint64_t> d_failed_{ 0 };
    std::atomic<uint64_t> d_dropped_{ 0 };
    std::atomic<uint64_t> d_invalid_{ 0 };
    std::atomic<uint64_t> d_exceptions_{ 0 };
    std::atomic<size_t> d_queue_depth_{ 0 };
    std::atomic<size_t> d_queue_high_watermark_{ 0 };

    // Service-time histogram: 64 bins × 64 µs + overflow (own mutex).
    mutable std::mutex d_hist_mutex_;
    uint64_t d_service_buckets_[64] = {};
    uint64_t d_service_overflow_ = 0;
    uint64_t d_service_sum_us_ = 0;
    std::atomic<uint64_t> d_service_max_us_{ 0 };
};

} // namespace uwb
} // namespace gr

#endif /* INCLUDED_GNURADIO_UWB_UWB_RADAR_CIR_ESTIMATOR_BLOCK_H */
