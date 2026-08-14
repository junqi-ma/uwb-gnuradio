/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * UwbAutoScheduledExtractorSc16 — blind-acquire the first confirmable QM35
 * packet from a native 737.28 MS/s SC16 stream, then lock a 5 ms scheduled
 * capture.  No user-supplied first_packet_sample.
 *
 * Block type: gr::sync_block, 1 × packed complex<int16_t> in, 0 stream out.
 *   message out : packet, status
 *   message in  : lock_obs, control
 *
 * Scheduler (searched before implementation):
 *   GNU Radio null_sink / tag_debug are zero-output sync_blocks; work()
 *   returns the number of input items consumed.  Existing UwbDetectorSc16
 *   and UwbScheduledExtractor use the same idiom plus a one-sample EOS
 *   sentinel so the worker drains before the flowgraph tears down.
 *   Message handlers only write pending observations/control; work()
 *   applies mode switches on chunk boundaries from d_current_sample and
 *   never creates past windows.
 *
 * UHD (4.6, gr-uhd usrp_source_impl):
 *   rx_time is a stream tag (uint64 seconds, double frac) at start() and
 *   after overflows.  Overflow itself is also published as an async
 *   message ("overflows") and does not always attach a dedicated overflow
 *   stream tag — the X410 app must forward async overflow on "control".
 *
 * Hot path: acquisition energy/core XOR scheduled bulk-skip/copy.  Never
 * nest another GNU Radio block's work().
 */

#pragma once

#include <gnuradio/io_signature.h>
#include <gnuradio/sync_block.h>
#include <gnuradio/uwb/api.h>
#include <gnuradio/uwb/uwb_defaults.h>
#include <gnuradio/uwb/uwb_detector_core.h>
#include <gnuradio/uwb/uwb_preamble_verifier_sc16.h>
#include <gnuradio/uwb/uwb_qm35_acquisition_tracker.h>
#include <gnuradio/uwb/uwb_scheduled_extractor_core.h>
#include <pmt/pmt.h>

#include <array>
#include <atomic>
#include <condition_variable>
#include <complex>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace gr {
namespace uwb {

class UWB_API UwbAutoScheduledExtractorSc16 : public gr::sync_block
{
public:
    using sptr = std::shared_ptr<UwbAutoScheduledExtractorSc16>;
    using EmitPolicy = core::EmitPolicy;
    using LockState = core::Qm35LockState;

    ~UwbAutoScheduledExtractorSc16() override;

    /**
     * \param known_preamble     native 737.28 MS/s one-SYNC template
     * \param sample_rate        native stream rate (default 737.28e6)
     * \param packet_interval_s  known QM35 period (default 0.005)
     * \param pre_guard_samples  scheduled window pre-guard
     * \param capture_samples    scheduled window body
     * \param post_guard_samples scheduled window post-guard
     * \param energy_threshold   acquire energy gate
     * \param energy_gate_decimation
     * \param coarse_decimation / repetitions / margin
     * \param lock_observations  timing-ok count to enter LOCKED
     * \param holdover_miss_count
     * \param reacquire_miss_count
     * \param provisional_guard_us extra guard while provisional/holdover
     * \param acquire_pre_trigger / acquire_capture  acquisition PDU geometry
     */
    static sptr make(
        const std::vector<std::complex<float>>& known_preamble,
        double sample_rate = defaults::kNativeSampleRateHz,
        double packet_interval_s = defaults::kQm35PacketIntervalS,
        size_t pre_guard_samples = defaults::kNativeScheduledPreGuard,
        size_t capture_samples = defaults::kNativeScheduledCapture,
        size_t post_guard_samples = defaults::kNativeScheduledPostGuard,
        float energy_threshold = defaults::kDetectorEnergyThreshold,
        size_t energy_gate_decimation = defaults::kDetectorEnergyGateDecimation,
        size_t coarse_decimation = defaults::kDetectorCoarseDecimation,
        size_t coarse_repetitions = defaults::kDetectorCoarseRepetitions,
        size_t coarse_margin = defaults::kDetectorCoarseMargin,
        size_t lock_observations = defaults::kLockObservations,
        size_t holdover_miss_count = defaults::kHoldoverMissCount,
        size_t reacquire_miss_count = defaults::kReacquireMissCount,
        double provisional_guard_us = defaults::kProvisionalGuardUs,
        size_t acquire_pre_trigger = defaults::kDetectorPreTrigger,
        size_t acquire_capture = defaults::kDetectorCapture,
        size_t scheduled_pool_size = defaults::kAutoScheduledPoolSize);

    static sptr make_from_file(
        const std::string& template_file,
        double sample_rate = defaults::kNativeSampleRateHz,
        double packet_interval_s = defaults::kQm35PacketIntervalS,
        size_t pre_guard_samples = defaults::kNativeScheduledPreGuard,
        size_t capture_samples = defaults::kNativeScheduledCapture,
        size_t post_guard_samples = defaults::kNativeScheduledPostGuard,
        float energy_threshold = defaults::kDetectorEnergyThreshold,
        size_t energy_gate_decimation = defaults::kDetectorEnergyGateDecimation,
        size_t coarse_decimation = defaults::kDetectorCoarseDecimation,
        size_t coarse_repetitions = defaults::kDetectorCoarseRepetitions,
        size_t coarse_margin = defaults::kDetectorCoarseMargin,
        size_t lock_observations = defaults::kLockObservations,
        size_t holdover_miss_count = defaults::kHoldoverMissCount,
        size_t reacquire_miss_count = defaults::kReacquireMissCount,
        double provisional_guard_us = defaults::kProvisionalGuardUs,
        size_t acquire_pre_trigger = defaults::kDetectorPreTrigger,
        size_t acquire_capture = defaults::kDetectorCapture,
        size_t scheduled_pool_size = defaults::kAutoScheduledPoolSize);

    int lock_state() const;
    const char* lock_state_name() const;
    uint64_t schedule_generation() const;
    uint64_t acquisition_epoch() const;
    bool identity_confirmed() const;
    double locked_t0() const;
    double locked_period_s() const;
    uint64_t energy_regions() const;
    uint64_t energy_regions_after_lock() const;
    uint64_t candidates_emitted() const;
    uint64_t candidates_rejected() const;
    uint64_t scheduled_windows() const;
    uint64_t emitted_windows() const;
    uint64_t dropped_windows() const;
    uint64_t pool_drops() const;
    uint64_t queue_full_drops() const;
    uint64_t stale_feedback() const;
    uint64_t unmapped_feedback() const;
    uint64_t discontinuities() const;
    uint64_t current_sample() const;

    // Test / app helper: same pending path as the lock_obs message port.
    void post_lock_obs(pmt::pmt_t msg);
    void post_control(pmt::pmt_t msg);

protected:
    UwbAutoScheduledExtractorSc16(
        const std::vector<std::complex<float>>& known_preamble,
        double sample_rate,
        double packet_interval_s,
        size_t pre_guard_samples,
        size_t capture_samples,
        size_t post_guard_samples,
        float energy_threshold,
        size_t energy_gate_decimation,
        size_t coarse_decimation,
        size_t coarse_repetitions,
        size_t coarse_margin,
        size_t lock_observations,
        size_t holdover_miss_count,
        size_t reacquire_miss_count,
        double provisional_guard_us,
        size_t acquire_pre_trigger,
        size_t acquire_capture,
        size_t scheduled_pool_size);

    int work(int noutput_items,
             gr_vector_const_void_star& input_items,
             gr_vector_void_star& output_items) override;
    bool start() override;
    bool stop() override;

private:
    enum class JobKind : uint8_t { Acquisition = 0, Scheduled = 1 };
    struct Job {
        JobKind kind = JobKind::Acquisition;
        size_t handle = 0;
    };

    void handle_lock_obs_msg(pmt::pmt_t msg);
    void handle_control_msg(pmt::pmt_t msg);
    void apply_pending();
    void apply_one_obs(const core::Qm35LockObservation& obs);
    void apply_identity_schedule();
    void apply_guard_for_state();
    void scan_rx_time_tags(int nitems);
    void handle_discontinuity(const char* reason);
    void enqueue_ready_jobs();
    void worker_loop();
    void shutdown_worker();
    void wait_for_worker_idle();
    void publish_acquisition(UwbDetectorStateMachineSc16::RegionHandle handle);
    void publish_scheduled(core::ScheduledWindowCore::WindowHandle handle);
    void publish_status(const std::string& event, pmt::pmt_t extra = pmt::PMT_NIL);
    static std::vector<std::complex<float>> load_template(const std::string& path);

    core::Qm35AcquisitionTracker tracker_;
    UwbDetectorStateMachineSc16 sm_;
    core::ScheduledWindowCore sched_;
    core::UwbPreambleVerifierSc16 verifier_;

    double d_sample_rate_ = defaults::kNativeSampleRateHz;
    double d_interval_s_ = defaults::kQm35PacketIntervalS;
    size_t d_pre_ = defaults::kNativeScheduledPreGuard;
    size_t d_cap_ = defaults::kNativeScheduledCapture;
    size_t d_post_ = defaults::kNativeScheduledPostGuard;
    size_t d_acq_pre_ = defaults::kDetectorPreTrigger;
    size_t d_acq_cap_ = defaults::kDetectorCapture;
    double d_prov_guard_us_ = defaults::kProvisionalGuardUs;

    uint64_t d_current_sample_ = 0;
    uint64_t d_packet_id_ = 0;
    uint64_t d_energy_regions_ = 0;
    uint64_t d_energy_regions_at_lock_ = 0;
    std::atomic<uint64_t> d_energy_after_lock_{ 0 };
    std::atomic<uint64_t> d_candidates_{ 0 };
    std::atomic<uint64_t> d_rejected_{ 0 };
    std::atomic<uint64_t> d_emitted_{ 0 };
    std::atomic<uint64_t> d_pool_drops_{ 0 };
    std::atomic<uint64_t> d_queue_full_{ 0 };
    std::atomic<uint64_t> d_discontinuities_{ 0 };

    bool d_have_rx_time_ = false;
    uint64_t d_rx_time_abs_ = 0;
    double d_rx_time_s_ = 0.0;
    double d_rx_rate_ = 0.0;

    mutable std::mutex d_cfg_mutex_;
    static constexpr size_t kPendingObsCap = 16;
    std::array<core::Qm35LockObservation, kPendingObsCap> d_pending_obs_{};
    size_t d_pending_obs_n_ = 0;
    bool d_pending_disc_ = false;
    std::string d_pending_disc_reason_;
    bool d_pending_reset_ = false;
    LockState d_last_applied_state_ = LockState::UnlockedAcquire;

    std::thread d_worker_;
    std::mutex d_job_mutex_;
    std::condition_variable d_job_cv_;
    static constexpr size_t kJobQueueSize = 16;
    std::array<Job, kJobQueueSize> d_job_queue_{};
    size_t d_job_head_ = 0;
    size_t d_job_tail_ = 0;
    size_t d_job_count_ = 0;
    size_t d_jobs_in_flight_ = 0;
    bool d_worker_stop_ = false;
};

} // namespace uwb
} // namespace gr
