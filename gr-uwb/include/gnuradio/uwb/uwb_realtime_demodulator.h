/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * UwbRealtimeDemodulator — asynchronous message-only UWB demodulator (R5).
 *
 * Accepts window PDUs on "samples", runs the frozen header-only demod core
 * (demodulate_one) on a bounded worker pool, and publishes one result PDU
 * (meta + u8vector payload bytes) per accepted job on "result".  Status /
 * events and a stats snapshot go out on "status".
 *
 * Samples port payload conventions (R7 SC16):
 *   1. pmt::cons(meta, c32vector)     — primary CF32 path (zero-copy into job)
 *   2. pmt::cons(meta, s16vector)     — interleaved int16 I/Q (I0,Q0,I1,Q1,…)
 *                                      converted to CF32 with scale 1/32767
 *   3. plain pmt::s16vector           — same interleaved I/Q, empty meta dict
 * Length of an s16vector must be even (2 * n_complex).  After conversion the
 * job always holds a c32vector so the worker hot path is unchanged.
 *
 * Block type: gr::block, zero stream ports.  Message handlers only validate
 * and enqueue; the hot demod path runs on worker threads with per-worker
 * DemodScratch (no per-packet allocation).  Queue-full drops never block.
 * stop() drains queued jobs before joining workers.
 */

#ifndef INCLUDED_GNURADIO_UWB_UWB_REALTIME_DEMODULATOR_H
#define INCLUDED_GNURADIO_UWB_UWB_REALTIME_DEMODULATOR_H

#include <gnuradio/block.h>
#include <gnuradio/gr_complex.h>
#include <gnuradio/uwb/api.h>
#include <gnuradio/uwb/uwb_demod_core.h>
#include <gnuradio/uwb/uwb_demod_result.h>
#include <gnuradio/uwb/uwb_phy_profile.h>
#include <pmt/pmt.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

namespace gr {
namespace uwb {

class UWB_API UwbRealtimeDemodulator : public gr::block
{
public:
    using sptr = std::shared_ptr<UwbRealtimeDemodulator>;

    ~UwbRealtimeDemodulator() override;

    /**
     * \param template_path   CF32 file: code-9 SYNC matched-filter template
     *                        (typically 1016 samples).
     * \param num_workers     worker-pool size (must be > 0).
     * \param queue_capacity  bounded job queue depth (must be > 0).
     * \param sfd_mode        SFD sequence mode used by stages 3/5
     *                        ("4z2" for QM35825, "ieee" for the MATLAB lrwpan
     *                        golden generator).
     * \param cir_rake_top_k  0 for the full CIR filter; otherwise use the K
     *                        strongest CIR taps for sparse RAKE combining.
     * \param cir_filter_mode "auto", "full", "rake", or "bypass";
     *                        bypass is the realtime default.
     * \param code_index      preamble code index (9 = QM35825; 10–12 =
     *                        supported DW1000 HRP codes).
     * \param preamble_repetitions number of preamble SYNC symbols the
     *                        transmitter sends (drives NS-SFD search position
     *                        and the CIR/phase-align window).
     * \param timing_coarse_stride full-rate samples between coarse timing
     *                        probes; 1 disables coarse decimation.
     */
    static sptr make(const std::string& template_path,
                     size_t num_workers = 2,
                     size_t queue_capacity = 64,
                     const std::string& sfd_mode = "4z2",
                     size_t cir_rake_top_k = 0,
                     const std::string& cir_filter_mode = "bypass",
                     size_t code_index = 9,
                     size_t preamble_repetitions = 64,
                     size_t timing_coarse_stride = 14);

    /**
     * Same as make(), but takes an in-memory CF32 template waveform.
     */
    static sptr make_from_template(const std::vector<gr_complex>& template_wf,
                                   size_t num_workers = 2,
                                   size_t queue_capacity = 64,
                                   const std::string& sfd_mode = "4z2",
                                   size_t cir_rake_top_k = 0,
                                   const std::string& cir_filter_mode = "bypass",
                                   size_t code_index = 9,
                                   size_t preamble_repetitions = 64,
                                   size_t timing_coarse_stride = 14);

    // Counters / stats (thread-safe snapshots).
    uint64_t jobs_received() const;
    uint64_t jobs_completed() const;
    uint64_t jobs_failed() const;
    uint64_t jobs_dropped() const;
    uint64_t invalid_inputs() const;
    uint64_t worker_exceptions() const;
    size_t queue_depth() const;
    size_t queue_high_watermark() const;
    size_t num_workers() const;
    size_t timing_coarse_stride() const;
    uint64_t latency_p50_us() const;
    uint64_t latency_p95_us() const;
    uint64_t latency_p99_us() const;
    uint64_t latency_max_us() const;
    void latency_histogram(uint64_t buckets[64], uint64_t* overflow) const;
    double worker_utilization_pct() const;
    bool drained() const;
    void drain();
    void reset_stats();

    // Per-stage mean demod time (µs) over jobs published so far.
    // stage index: 0..6 = timing/cfo/sfd/cir/ns_sfd/phr/payload, 7 = total.
    // Returns 0 when no job has been published for that stage yet.
    uint64_t stage_mean_us(size_t stage) const;
    uint64_t stage_mean_total_us() const;

protected:
    UwbRealtimeDemodulator(const std::vector<gr_complex>& template_wf,
                           size_t num_workers,
                           size_t queue_capacity,
                           const std::string& sfd_mode,
                           size_t cir_rake_top_k,
                           const std::string& cir_filter_mode,
                           size_t code_index = 9,
                           size_t preamble_repetitions = 64,
                           size_t timing_coarse_stride = 14);

    bool start() override;
    bool stop() override;

private:
    struct Job {
        uint64_t packet_id = 0;
        uint64_t schedule_index = 0;
        int64_t predicted_start_sample = -1;
        int64_t window_start_sample = 0;
        int64_t sample_count = 0;
        double sample_rate = 0.0;          // demod domain (usually 998.4e6)
        double native_sample_rate = 0.0;   // extractor domain if resampled
        double resample_filter_delay = 0.0; // group delay at demod domain
        uint64_t resample_us = 0;          // upstream PDU FIR process+flush
        uint64_t acquisition_epoch = 0;
        uint64_t schedule_generation = 0;
        bool has_acquisition_epoch = false;
        bool has_schedule_generation = false;
        pmt::pmt_t samples; // c32vector (immutable shared ref)
        std::chrono::steady_clock::time_point enqueued_at;
    };

    void handle_samples(pmt::pmt_t msg);
    void handle_control(pmt::pmt_t msg);
    bool enqueue(Job&& job);
    void worker_loop(size_t wid);
    void publish_result(const Job& job, const gr::uwb::demod::DemodResult& r);
    void publish_schedule_feedback(const Job& job,
                                   const gr::uwb::demod::DemodResult& r);
    void publish_status(const std::string& event,
                        pmt::pmt_t extra = pmt::PMT_NIL);
    void record_latency(uint64_t us);
    void snapshot_stats(pmt::pmt_t& meta);
    uint64_t latency_percentile_us(double pct) const;

    static std::vector<gr_complex> load_cf32_file(const std::string& path);
    static const char* status_to_string(gr::uwb::demod::DemodStatus s);

    // Immutable after construction (workers only read).
    std::vector<gr_complex> d_template_wf_;
    std::string d_sfd_mode_; // owned copy; d_profile_.sfd_mode points into it
    gr::uwb::demod::Qm35825Profile d_profile_;
    size_t d_num_workers_ = 0;
    size_t d_queue_capacity_ = 0;

    // Per-worker demod scratch (index = worker id; touched only by that worker).
    std::vector<gr::uwb::demod::core::DemodScratch> d_scratch_;

    // Job queue.
    mutable std::mutex d_queue_mutex_;
    std::condition_variable d_queue_cv_;
    std::deque<Job> d_queue_;
    std::atomic<bool> d_stop_{ false };
    std::vector<std::thread> d_workers_;
    std::atomic<size_t> d_idle_workers_{ 0 };

    // Counters.
    std::atomic<uint64_t> d_jobs_received_{ 0 };
    std::atomic<uint64_t> d_jobs_completed_{ 0 };
    std::atomic<uint64_t> d_jobs_failed_{ 0 };
    std::atomic<uint64_t> d_jobs_dropped_{ 0 };
    std::atomic<uint64_t> d_invalid_inputs_{ 0 };
    std::atomic<uint64_t> d_worker_exceptions_{ 0 };
    std::atomic<size_t> d_queue_depth_{ 0 };
    std::atomic<size_t> d_queue_high_watermark_{ 0 };

    // Latency histogram: 64 bins × 64 µs + overflow (own mutex).
    mutable std::mutex d_hist_mutex_;
    uint64_t d_latency_buckets_[64] = {};
    uint64_t d_latency_overflow_ = 0;
    std::atomic<uint64_t> d_latency_max_us_{ 0 };

    // Per-worker busy/total µs for utilization (own mutex).
    mutable std::mutex d_worker_stats_mutex_;
    std::vector<uint64_t> d_worker_busy_us_;
    std::vector<uint64_t> d_worker_total_us_;

    // Per-stage timing accumulation (own mutex): stage 0..6 + total (index 7).
    mutable std::mutex d_stage_mutex_;
    uint64_t d_stage_sums_[8] = {};
    uint64_t d_stage_n_ = 0;

    // TEST-ONLY fault injection: throw in worker for matching packet_id.
    mutable std::mutex d_fail_mutex_;
    std::optional<uint64_t> d_fail_packet_id_;
};

} // namespace uwb
} // namespace gr

#endif /* INCLUDED_GNURADIO_UWB_UWB_REALTIME_DEMODULATOR_H */
