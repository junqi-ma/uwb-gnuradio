/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * UwbScheduledExtractor — fixed-interval radar-slot IQ capture.
 *
 * Given continuous CF32 IQ, sample rate, radar seed t0 and period T, emits one
 * fixed-length window PDU per predicted radar slot:
 *
 *   predicted_start(k) = round(t0 + k*T)   // independent per slot
 *   window = [predicted - pre_guard, predicted + capture + post_guard)
 *
 * Default emit_policy = every_slot: always emit even when radar verification
 * fails or communication energy collides.  Window count tracks the radar
 * schedule only — dense communication traffic does not create extra windows.
 *
 * Block type: gr::sync_block, 1 stream in, 0 stream out, PDU on "packet",
 * status on "status", control on "schedule".
 *
 * Scheduler: work() bulk-skips outside windows and bulk-copies inside; completed
 * handles go to a bounded queue.  A background worker owns optional template
 * verification, PMT construction, and message_port_pub.  Finite streams leave
 * a one-sample sentinel so the worker drains before EOS (same idiom as
 * UwbDetector).
 */

#pragma once

#include <gnuradio/gr_complex.h>
#include <gnuradio/io_signature.h>
#include <gnuradio/sync_block.h>
#include <gnuradio/uwb/api.h>
#include <gnuradio/uwb/uwb_defaults.h>
#include <gnuradio/uwb/uwb_scheduled_extractor_core.h>
#include <pmt/pmt.h>

#include <array>
#include <atomic>
#include <condition_variable>
#include <complex>
#include <cstdint>
#include <mutex>
#include <thread>
#include <vector>

namespace gr {
namespace uwb {

class UWB_API UwbScheduledExtractor : public gr::sync_block
{
public:
    using sptr = std::shared_ptr<UwbScheduledExtractor>;
    using EmitPolicy = core::EmitPolicy;
    using PartialEosPolicy = core::PartialEosPolicy;

    ~UwbScheduledExtractor() override;

    /**
     * \param sample_rate           input sample rate (Hz)
     * \param packet_interval_s     radar period T (s)
     * \param first_packet_sample   seed t0 as absolute sample (0-based)
     * \param pre_guard_samples     samples before predicted start
     * \param capture_samples       samples after predicted start (packet body)
     * \param post_guard_samples    trailing guard
     * \param pool_size             preallocated window slots
     * \param emit_policy           every_slot (default) or verified_only
     * \param verification_enabled  run code-9 style template check in worker
     * \param radar_template        optional radar SYNC template for verification
     * \param radar_threshold       verification metric threshold
     * \param comm_template         optional comm template for collision flag
     * \param comm_threshold        comm metric threshold
     */
    static sptr make(double sample_rate,
                     double packet_interval_s,
                     uint64_t first_packet_sample,
                     size_t pre_guard_samples = defaults::kScheduledPreGuard,
                     size_t capture_samples = defaults::kScheduledCapture,
                     size_t post_guard_samples = defaults::kScheduledPostGuard,
                     size_t pool_size = defaults::kScheduledPoolSize,
                     EmitPolicy emit_policy = EmitPolicy::EverySlot,
                     bool verification_enabled = false,
                     const std::vector<std::complex<float>>& radar_template =
                         std::vector<std::complex<float>>(),
                     float radar_threshold = 0.5f,
                     const std::vector<std::complex<float>>& comm_template =
                         std::vector<std::complex<float>>(),
                     float comm_threshold = 0.5f);

    // Runtime schedule control (also available via "schedule" message port).
    void set_schedule(uint64_t first_packet_sample, double packet_interval_s);
    void set_schedule_time(double first_packet_time_s, double packet_interval_s);
    void pause_schedule();
    void resume_schedule();
    void reset_schedule();

    void set_emit_policy(EmitPolicy p);
    EmitPolicy emit_policy() const;
    void set_partial_eos_policy(PartialEosPolicy p);
    void set_verification_enabled(bool en);

    /**
     * Schedule lock (learn-then-freeze):
     *   Searching → Learning (b, δ from SYNC observations) → Hold (freeze).
     *
     * Nominal T is typically 5000 us; true period is T_nom + δ with nearly
     * fixed δ (SFO).  Wide demod search covers residual during Learning;
     * after converge, (t0, T) stay frozen until sustained residual outliers
     * re-enter Learning.  Not a continuous PLL / not an FCS fast loop.
     *
     * Preferred observation: SYNC/preamble timing via "lock_obs" (demod
     * schedule_feedback).  Optional radar verification peaks are secondary.
     * Default OFF until set_schedule_lock_enabled(true).
     *
     * Message port "lock_obs" accepts a dict:
     *   command="observe" (default),
     *   schedule_index (uint),
     *   detected_start_sample (absolute, observation domain),
     *   sample_rate (Hz of detected_start; optional),
     *   native_sample_rate / input_sample_rate (extractor domain; optional),
     *   resample_filter_delay (group delay samples at obs rate; for 65/48).
     *   timing_ok (optional; if present and false, observation is ignored),
     *   fcs_pass (optional; ignored for lock — FCS is not the fast sensor).
     */
    void set_schedule_lock_enabled(bool en);
    bool schedule_lock_enabled() const;
    int schedule_lock_state() const; // 0=Searching, 1=Learning, 2=Hold
    uint64_t schedule_lock_updates() const;
    double locked_packet_interval_s() const;
    double locked_first_packet_sample() const;
    /** Learned period deviation in samples (period - nominal). 0 if not Hold. */
    double locked_delta_period_samples() const;
    /** Learned t0 bias in samples (t0 - seed). 0 if not Hold. */
    double locked_bias_t0_samples() const;

    /** Direct observation (same domain as the extractor sample_rate). */
    void observe_detection(uint64_t schedule_index, int64_t detected_start_sample);

    // Counters (thread-safe snapshots via core stats + block emit counters).
    uint64_t scheduled_windows() const;
    uint64_t completed_windows() const;
    uint64_t emitted_windows() const;
    uint64_t dropped_windows() const;
    uint64_t partial_windows() const;
    uint64_t queue_high_watermark() const;
    uint64_t verification_failures() const;
    uint64_t collisions() const;

    // work-chunk instrumentation (same shape as UwbDetector).
    uint64_t work_calls() const;
    uint64_t work_items_total() const;
    int work_min_noutput_items() const;
    int work_max_noutput_items() const;
    double work_mean_noutput_items() const;
    void work_noutput_histogram(uint64_t out[5]) const;
    void reset_work_stats();

protected:
    UwbScheduledExtractor(
        double sample_rate,
        double packet_interval_s,
        uint64_t first_packet_sample,
        size_t pre_guard_samples,
        size_t capture_samples,
        size_t post_guard_samples,
        size_t pool_size,
        EmitPolicy emit_policy,
        bool verification_enabled,
        const std::vector<std::complex<float>>& radar_template,
        float radar_threshold,
        const std::vector<std::complex<float>>& comm_template,
        float comm_threshold);

    int work(int noutput_items,
             gr_vector_const_void_star& input_items,
             gr_vector_void_star& output_items) override;

    bool start() override;
    bool stop() override;

private:
    void handle_schedule_msg(pmt::pmt_t msg);
    void handle_lock_obs_msg(pmt::pmt_t msg);
    void apply_pending_config();
    void enqueue_ready_windows();
    void worker_loop();
    void shutdown_worker();
    void wait_for_worker_idle();
    void publish_window(core::ScheduledWindowCore::WindowHandle handle);
    void publish_status(const std::string& event, pmt::pmt_t extra = pmt::PMT_NIL);
    // Map an absolute sample index from obs_rate → native extractor rate.
    // Handles the 998.4↔737.28 (65/48) group-delay map when rates match.
    int64_t map_obs_sample_to_native(int64_t obs_sample,
                                     double obs_rate,
                                     double filter_delay) const;
    // Thread-safe observe; queues a soft t0/T update for the work() path.
    bool note_lock_observation(uint64_t schedule_index,
                               int64_t detected_native);

    core::ScheduledWindowCore core_;
    uint64_t d_current_sample_ = 0;

    // Pending config from message port (swapped on chunk boundary).
    mutable std::mutex d_cfg_mutex_;
    bool d_pending_cfg_ = false;
    core::ScheduleConfig d_pending_;
    bool d_pending_pause_ = false;
    bool d_pending_resume_ = false;
    bool d_pending_reset_ = false;
    bool d_pending_lock_ = false;
    bool d_pending_lock_force_t0_ = false;
    double d_pending_lock_t0_ = 0.0;
    double d_pending_lock_period_s_ = 0.0;

    EmitPolicy d_emit_policy_;
    bool d_verification_enabled_ = false;
    float d_radar_threshold_ = 0.5f;
    float d_comm_threshold_ = 0.5f;
    std::vector<std::complex<float>> d_radar_tmpl_;
    std::vector<std::complex<float>> d_comm_tmpl_;

    // Schedule lock (guarded by d_cfg_mutex_).
    core::ScheduleLockTracker d_lock_;

    std::thread d_worker_;
    std::mutex d_job_mutex_;
    std::condition_variable d_job_cv_;
    static constexpr size_t kJobQueueSize = core::ScheduledWindowCore::kMaxPoolSize;
    std::array<core::ScheduledWindowCore::WindowHandle, kJobQueueSize> d_job_queue_{};
    size_t d_job_head_ = 0;
    size_t d_job_tail_ = 0;
    size_t d_job_count_ = 0;
    size_t d_jobs_in_flight_ = 0;
    bool d_worker_stop_ = false;
    std::atomic<uint64_t> d_dropped_jobs_{ 0 };
    std::atomic<uint64_t> d_emitted_{ 0 };

    uint64_t d_work_calls_ = 0;
    uint64_t d_work_items_total_ = 0;
    int d_work_min_n_ = 0;
    int d_work_max_n_ = 0;
    uint64_t d_work_hist_[5] = {};
};

} // namespace uwb
} // namespace gr
