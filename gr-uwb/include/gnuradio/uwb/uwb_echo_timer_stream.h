/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * UwbEchoTimerStream — tagged-stream monostatic UWB echo timer (Radar
 * Stage 2).  Replaces the Python TimedUhdEcho RX path with a
 * gr-radar-style tagged-stream block while reusing the existing EchoTimer
 * scheduling core and IRadioBurstBackend instead of reimplementing UHD.
 *
 * Block type: gr::tagged_stream_block, 1 complex64 stream in, 1 complex64
 * stream out.  One input "packet_len"-delimited packet = one TX burst;
 * one output window of fixed rx_samples = the timed RX capture for that
 * burst.
 *
 * Scheduler semantics (identical burst execution to
 * UwbRealtimeEchoTimer::run_one_burst):
 *   1. First work() lazily arms an EchoGrid at t0_ticks (or, when
 *      t0_ticks == 0, at device_time_ticks() + arm_margin_s * rate).
 *   2. Each work() takes the next future grid slot (expired slots are
 *      skipped, never caught up) in exact integer device ticks.
 *   3. The RX command is issued BEFORE the TX burst; on any failure the
 *      stream still emits rx_samples zero-filled complex64 samples plus a
 *      "burst_status" tag so the flowgraph keeps running.
 *   4. work() blocks in the backend's collect_result wait — exactly ONE
 *      burst per call, like gr-radar usrp_echotimer_cc::work.  It returns
 *      WORK_DONE after max_frames bursts (0 = unlimited) or on stop.
 *
 * The TX payload is converted complex64 -> interleaved SC16 (scale 32768,
 * clipped) into a fixed member buffer sized 2*tx_samples; the RX scratch
 * is a fixed member sized 2*max_rx_samples allocated once at construction
 * and never resized (no allocation on the work hot path).
 *
 * Output tags (added at the output window start; packet_len is added
 * automatically by tagged_stream_block): pulse_id, schedule_index,
 * sample_rate, window_start_sample, pre_guard_samples, capture_samples,
 * post_guard_samples, sample_count, calibration_delay_native_samples,
 * sync_repetitions, sfd_mode, code_index, and (on success) rx_time as a
 * (full, frac) tuple.  Failed windows additionally carry burst_status.
 */

#ifndef INCLUDED_GNURADIO_UWB_UWB_ECHO_TIMER_STREAM_H
#define INCLUDED_GNURADIO_UWB_UWB_ECHO_TIMER_STREAM_H

#include <gnuradio/gr_complex.h>
#include <gnuradio/tagged_stream_block.h>
#include <gnuradio/uwb/api.h>
#include <gnuradio/uwb/uwb_echo_burst_backend.h>
#include <gnuradio/uwb/uwb_echo_scheduler_core.h>

#ifdef UWB_HAVE_UHD
#include <gnuradio/uwb/uwb_uhd_backend_config.h>
#endif

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace gr {
namespace uwb {

class UWB_API UwbEchoTimerStream : public gr::tagged_stream_block
{
public:
    using sptr = std::shared_ptr<UwbEchoTimerStream>;

    /**
     * \param sched_cfg          device-time grid configuration (exact PRI
     *                           rational, pre-guard ticks, catch-up cap,
     *                           burst fragment chunk); validated once here
     *                           by prepare_echo_scheduler() and frozen.
     * \param backend            injected radio burst backend (never null).
     * \param tx_samples         fixed TX burst length in complex64 samples;
     *                           each input packet length must match.
     * \param rx_samples         fixed RX window length (output window).
     * \param sample_rate_hz     device sample rate, used for rx_time tags.
     * \param pre_guard_samples  reported window geometry (tags only).
     * \param capture_samples    reported window geometry (tags only).
     * \param post_guard_samples reported window geometry (tags only).
     * \param sync_repetitions   reported radar metadata (tags only).
     * \param sfd_mode           reported radar metadata (tags only).
     * \param code_index         reported radar metadata (tags only).
     * \param calibration_delay_native_samples  initial cal delay (tags).
     * \param t0_ticks           first grid slot device tick; 0 = arm at
     *                           device_time + arm_margin_s * sample_rate.
     * \param arm_margin_s       lazy-arm margin when t0_ticks == 0.
     * \param max_frames         stop after this many windows (0 = never).
     * \param collect_wait_ms    RX completion wait budget per burst.
     * \param max_tx_samples     fixed TX scratch cap (>= tx_samples).
     * \param max_rx_samples     fixed RX scratch cap (>= rx_samples).
     * \param lengthtagname      input/output length tag key.
     */
    static sptr make(const echo::EchoSchedulerConfig& sched_cfg,
                     std::shared_ptr<echo::IRadioBurstBackend> backend,
                     uint64_t tx_samples,
                     uint64_t rx_samples,
                     double sample_rate_hz,
                     uint64_t pre_guard_samples,
                     uint64_t capture_samples,
                     uint64_t post_guard_samples,
                     size_t sync_repetitions,
                     const std::string& sfd_mode,
                     size_t code_index,
                     double calibration_delay_native_samples,
                     int64_t t0_ticks,
                     double arm_margin_s,
                     uint64_t max_frames,
                     uint64_t collect_wait_ms,
                     size_t max_tx_samples,
                     size_t max_rx_samples,
                     const std::string& lengthtagname = "packet_len");

#ifdef UWB_HAVE_UHD
    /**
     * UHD convenience factory: builds a UhdBurstBackend from uhd_cfg and
     * forwards to make().  Same geometry arguments as make().
     */
    static sptr make_uhd(const uhd::UhdBurstBackendConfig& uhd_cfg,
                         const echo::EchoSchedulerConfig& sched_cfg,
                         uint64_t tx_samples,
                         uint64_t rx_samples,
                         double sample_rate_hz,
                         uint64_t pre_guard_samples,
                         uint64_t capture_samples,
                         uint64_t post_guard_samples,
                         size_t sync_repetitions,
                         const std::string& sfd_mode,
                         size_t code_index,
                         double calibration_delay_native_samples,
                         int64_t t0_ticks,
                         double arm_margin_s,
                         uint64_t max_frames,
                         uint64_t collect_wait_ms,
                         size_t max_tx_samples,
                         size_t max_rx_samples,
                         const std::string& lengthtagname = "packet_len");
#endif

    ~UwbEchoTimerStream() override;

    int calculate_output_stream_length(
        const gr_vector_int& ninput_items) override;

    int work(int noutput_items,
             gr_vector_int& ninput_items,
             gr_vector_const_void_star& input_items,
             gr_vector_void_star& output_items) override;

    // Lifecycle: start() prepares the backend; stop() aborts in-flight I/O.
    bool start() override;
    bool stop() override;

    // Thread-safe tuning/calibration (called from Python/message threads
    // while work() runs).  cal is read at the tag-emission point.
    void set_freq(double hz);
    double freq() const;
    void set_cal_delay_native(double v);
    double cal_delay_native() const;

    const echo::EchoSchedulerPrepared& sched_config() const
    {
        return d_sched_;
    }
    const std::shared_ptr<echo::IRadioBurstBackend>& backend() const
    {
        return d_backend_;
    }
    uint64_t tx_samples() const
    {
        return d_tx_samples_total_.load(std::memory_order_relaxed);
    }
    uint64_t rx_samples() const
    {
        return d_rx_samples_total_.load(std::memory_order_relaxed);
    }
    uint64_t bursts_ok() const
    {
        return d_bursts_ok_.load(std::memory_order_relaxed);
    }
    uint64_t bursts_failed() const
    {
        return d_bursts_failed_.load(std::memory_order_relaxed);
    }
    uint64_t late_slot_skips() const
    {
        return d_late_slot_skips_.load(std::memory_order_relaxed);
    }
    uint64_t grid_errors() const
    {
        return d_grid_errors_.load(std::memory_order_relaxed);
    }
    uint64_t frames() const
    {
        return d_frames_.load(std::memory_order_relaxed);
    }
    /** Last per-burst backend error string (empty before/after an ok burst). */
    std::string last_error() const
    {
        std::lock_guard<std::mutex> lk(d_err_mutex_);
        return d_last_error_;
    }

    // Fixed scratch introspection (QA): capacity never changes.
    size_t tx_scratch_capacity() const { return d_tx_sc16_.size(); }
    size_t rx_scratch_capacity() const { return d_rx_buf_.size(); }

    // Public for gnuradio::make_block_sptr; use make().
    UwbEchoTimerStream(const echo::EchoSchedulerConfig& sched_cfg,
                       std::shared_ptr<echo::IRadioBurstBackend> backend,
                       uint64_t tx_samples,
                       uint64_t rx_samples,
                       double sample_rate_hz,
                       uint64_t pre_guard_samples,
                       uint64_t capture_samples,
                       uint64_t post_guard_samples,
                       size_t sync_repetitions,
                       const std::string& sfd_mode,
                       size_t code_index,
                       double calibration_delay_native_samples,
                       int64_t t0_ticks,
                       double arm_margin_s,
                       uint64_t max_frames,
                       uint64_t collect_wait_ms,
                       size_t max_tx_samples,
                       size_t max_rx_samples,
                       const std::string& lengthtagname);

private:
    // Convert the TX input packet to interleaved SC16 into d_tx_sc16_.
    void convert_tx(const gr_complex* in, int nin);

    // Emit the output window tags and counters for one burst.
    void finish_burst(gr_complex* out, const echo::BurstResult& r, bool ok);

    // RC-only configuration (immutable after construction; work() reads).
    echo::EchoSchedulerPrepared d_sched_;
    std::shared_ptr<echo::IRadioBurstBackend> d_backend_;
    uint64_t d_tx_samples_ = 0;
    uint64_t d_rx_samples_ = 0;
    double d_sample_rate_hz_ = 0.0;
    uint64_t d_pre_guard_samples_ = 0;
    uint64_t d_capture_samples_ = 0;
    uint64_t d_post_guard_samples_ = 0;
    size_t d_sync_repetitions_ = 0;
    std::string d_sfd_mode_;
    size_t d_code_index_ = 0;
    int64_t d_t0_ticks_ = 0;
    double d_arm_margin_s_ = 0.0;
    uint64_t d_max_frames_ = 0;
    uint64_t d_collect_wait_ms_ = 0;
    size_t d_max_tx_samples_ = 0;
    size_t d_max_rx_samples_ = 0;

    // Mutable config (atomics: set from Python threads, read in work()).
    std::atomic<double> d_cal_delay_native_{ 0.0 };
    std::atomic<double> d_freq_hz_{ 0.0 };

    // Fixed scratch: allocated ONCE at construction, never resized.
    std::vector<int16_t> d_tx_sc16_; // 2 * tx_samples
    std::vector<int16_t> d_rx_buf_;  // 2 * max_rx_samples
    echo::BurstFragment d_txf_[echo::kEchoMaxFragmentsPerBurst] = {};
    echo::BurstFragment d_rxf_[echo::kEchoMaxFragmentsPerBurst] = {};

    // Work()-only scheduler state.
    echo::EchoGrid d_grid_;
    bool d_grid_armed_ = false;
    int64_t d_armed_t0_ = 0;
    std::atomic<bool> d_stop_{ false };

    // Counters (atomic).
    std::atomic<uint64_t> d_frames_{ 0 };
    std::atomic<uint64_t> d_bursts_ok_{ 0 };
    std::atomic<uint64_t> d_bursts_failed_{ 0 };
    std::atomic<uint64_t> d_late_slot_skips_{ 0 };
    std::atomic<uint64_t> d_grid_errors_{ 0 };
    std::atomic<uint64_t> d_rx_samples_total_{ 0 };
    std::atomic<uint64_t> d_tx_samples_total_{ 0 };

    // Last per-burst backend error string (diagnostics only).
    mutable std::mutex d_err_mutex_;
    std::string d_last_error_;
};

} // namespace uwb
} // namespace gr

#endif /* INCLUDED_GNURADIO_UWB_UWB_ECHO_TIMER_STREAM_H */
