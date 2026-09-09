/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * UwbRealtimeEchoTimer ("EchoTimer") — message-only timed burst radio
 * scheduler (Radar Step 10).  Monostatic UWB radar: one TX burst plus one
 * timed RX capture per PRI on a device-time tick grid.
 *
 * Block type: gr::block, zero stream ports, message PDU only (same
 * lifecycle/worker conventions as UwbRadarCirEstimator / UwbCirWriter).
 * The "schedule" handler only validates the PDU and enqueues it into a
 * bounded FIFO (overflow → drop + status event, never blocking).  A
 * single dedicated worker thread owns the EchoGrid device-time scheduler
 * core and the injected IRadioBurstBackend: per schedule index it
 * computes t_tx/t_rx in integer device ticks, skips expired slots (no
 * catch-up), issues the RX command BEFORE the TX burst, then blocks in
 * the backend's asynchronous completion wait — never busy-waiting and
 * never doing I/O in a GNU Radio handler.
 *
 * Radio I/O is dependency-injected: QA injects FakeBurstBackend (this
 * step); Step 11 injects the UhdBurstBackend.
 *
 * Input port "schedule": cons(meta_dict, s16vector TX payload).
 * The payload is one reusable TX burst of tx_samples SC16 sample pairs.
 * Dict keys (all lengths in device sample ticks / SC16 element pairs):
 *   t0_ticks          (i64/u64, required) device tick of the first TX slot
 *   tx_samples        (u64, required)     TX burst length, > 0; must equal
 *                                         the payload length
 *   rx_samples        (u64, required)     RX capture length, > 0
 *   schedule_index    (u64, optional)     grid index of the first slot
 *                                         (default 0)
 *   pulse_id          (u64, optional)     passthrough id on burst results
 *                                         (default schedule_index)
 *   burst_count       (u64, optional)     produce this many bursts, then
 *                                         publish "grid_complete"
 *                                         (default 0 = unlimited)
 *   max_fragment_size (u64, optional)     override the prepared planning
 *                                         chunk for this schedule (default
 *                                         block config, must be > 0)
 *   sample_rate       (real, optional)    device sample rate, used only to
 *                                         add tx_time_full/tx_time_frac and
 *                                         rx_time_full/rx_time_frac metadata
 *
 * Bounded-input contract: tx_samples/rx_samples are rejected unless
 * 0 < value <= max_tx_samples / max_rx_samples (fixed make() caps).  The
 * RX scratch buffer is preallocated once at construction for
 * max_rx_samples*2 int16 and never resized; t0_ticks must be
 * int64-representable and >= pre_guard_ticks.
 *
 * Output port "burst": cons(meta, s16vector) — exactly ONE result message
 * per schedule index, in index order.  ok/partial_handled results carry
 * the stitched RX capture in the s16vector; every failure result carries
 * an empty vector plus a UHD-like "uhd_error" string.  Meta keys:
 *   status ("ok"/"partial_handled"/"late_command"/"timeout"/"overflow"/
 *          "broken_chain"/"stop_during_io"/"backend_error"),
 *   status_code, schedule_index, pulse_id, tx_ticks, rx_ticks,
 *   rx_time_ticks, skipped_slots, tx_samples_requested/sent,
 *   rx_samples_requested/received, tx_reissues, rx_reissues, uhd_error,
 *   sample_format ("sc16"), and (when sample_rate is set) tx_time_full/
 *   tx_time_frac/rx_time_full/rx_time_frac.
 *
 * Output port "status": lifecycle/drop events ("started", "stopped",
 * "schedule_armed", "grid_complete", "grid_error", "late_slot_skip",
 * "queue_full", "invalid_schedule", "backend_prepare_failed",
 * "schedule_dropped_on_stop") with a full counter snapshot, mirroring the
 * radar estimator block.
 *
 * Grid-level failures (skip-limit exceeded / tick-grid overflow) have no
 * valid schedule index: they are reported as "grid_error" status events
 * and disarm the schedule; every *valid* index still yields exactly one
 * burst message.
 */

#ifndef INCLUDED_GNURADIO_UWB_UWB_REALTIME_ECHO_TIMER_H
#define INCLUDED_GNURADIO_UWB_UWB_REALTIME_ECHO_TIMER_H

#include <gnuradio/block.h>
#include <gnuradio/uwb/api.h>
#include <gnuradio/uwb/uwb_echo_burst_backend.h>
#include <gnuradio/uwb/uwb_echo_scheduler_core.h>
#include <pmt/pmt.h>

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace gr {
namespace uwb {

class UWB_API UwbRealtimeEchoTimer : public gr::block
{
public:
    using sptr = std::shared_ptr<UwbRealtimeEchoTimer>;

    /**
     * \param sched_cfg          device-time grid configuration (PRI
     *                           rational, pre-guard, catch-up cap, default
     *                           fragment chunk); validated once here via
     *                           prepare_echo_scheduler() and frozen.
     * \param backend            injected radio burst backend (never null;
     *                           FakeBurstBackend for CI, UhdBurstBackend
     *                           in Step 11).
     * \param queue_capacity     bounded schedule-queue depth (> 0).
     * \param rx_collect_wait_ms how long the worker waits for one RX
     *                           completion before synthesizing Timeout.
     * \param max_tx_samples     hard cap on tx_samples per schedule PDU
     *                           (must be > 0 and safely representable as
     *                           2 * value in size_t).
     * \param max_rx_samples     hard cap on rx_samples per schedule PDU;
     *                           the RX scratch buffer is preallocated ONCE
     *                           at construction for this size and never
     *                           grows (bounded input, fixed scratch).
     */
    static sptr make(const echo::EchoSchedulerConfig& sched_cfg,
                     std::shared_ptr<echo::IRadioBurstBackend> backend,
                     size_t queue_capacity = 64,
                     uint64_t rx_collect_wait_ms = 1000,
                     uint64_t max_tx_samples = 1u << 21,
                     uint64_t max_rx_samples = 1u << 21);

    ~UwbRealtimeEchoTimer() override;

    const echo::EchoSchedulerPrepared& sched_config() const
    {
        return d_sched_;
    }
    const std::shared_ptr<echo::IRadioBurstBackend>& backend() const
    {
        return d_backend_;
    }
    size_t queue_capacity() const { return d_queue_capacity_; }
    uint64_t rx_collect_wait_ms() const { return d_collect_wait_ms_; }
    uint64_t max_tx_samples() const { return d_max_tx_samples_; }
    uint64_t max_rx_samples() const { return d_max_rx_samples_; }

    // Fixed RX scratch introspection (QA): the buffer is allocated once at
    // construction; capacity and data address never change afterwards.
    size_t rx_scratch_capacity() const { return d_rx_buf_.size(); }
    const int16_t* rx_scratch_data() const { return d_rx_buf_.data(); }

    uint64_t schedules_received() const;
    uint64_t schedules_enqueued() const;
    uint64_t schedules_dropped() const;      // queue full
    uint64_t schedules_invalid() const;      // rejected in the handler
    uint64_t schedules_dropped_on_stop() const;
    uint64_t bursts_published() const;       // exactly one per index
    uint64_t bursts_ok() const;              // ok / partial_handled
    uint64_t bursts_failed() const;
    uint64_t late_slot_skips() const;        // expired grid slots skipped
    uint64_t tx_reissues() const;            // partial sends handled
    uint64_t rx_reissues() const;            // partial recvs handled
    uint64_t grid_errors() const;            // overflow / skip-limit events
    size_t queue_depth() const;
    size_t queue_high_watermark() const;

    bool drained() const;
    void drain();

    bool start() override;
    bool stop() override;

    // Public for gnuradio::make_block_sptr; use make().
    UwbRealtimeEchoTimer(const echo::EchoSchedulerConfig& sched_cfg,
                         std::shared_ptr<echo::IRadioBurstBackend> backend,
                         size_t queue_capacity,
                         uint64_t rx_collect_wait_ms,
                         uint64_t max_tx_samples,
                         uint64_t max_rx_samples);

private:
    struct Job {
        int64_t t0_ticks = 0;
        uint64_t schedule_index = 0;
        uint64_t pulse_id = 0;
        uint64_t tx_samples = 0;
        uint64_t rx_samples = 0;
        uint64_t max_fragment_size = 0; // 0 = prepared default
        uint64_t burst_count = 0;       // 0 = unlimited
        double sample_rate = 0.0;
        pmt::pmt_t meta = pmt::PMT_NIL;    // input dict (passthrough)
        pmt::pmt_t payload = pmt::PMT_NIL; // s16vector TX burst (immutable)
    };

    void handle_schedule(pmt::pmt_t msg);
    bool enqueue(Job&& job);
    void worker_loop();
    void apply_schedule(const Job& job);
    void run_one_burst();
    void finish_grid();
    void publish_burst(const echo::BurstResult& r, bool with_samples);
    void publish_status(const std::string& event, pmt::pmt_t extra = pmt::PMT_NIL);
    void snapshot_stats(pmt::pmt_t& meta) const;
    void stop_worker_and_join();

    // Immutable after construction (worker only reads).
    echo::EchoSchedulerPrepared d_sched_;
    std::shared_ptr<echo::IRadioBurstBackend> d_backend_;
    size_t d_queue_capacity_ = 0;
    uint64_t d_collect_wait_ms_ = 0;
    uint64_t d_max_tx_samples_ = 0; // fixed caps, validated at make()
    uint64_t d_max_rx_samples_ = 0;

    // Worker-owned schedule state (single worker; never touched by the
    // handler).  The RX scratch is preallocated at construction for
    // max_rx_samples and is NEVER resized on the worker: arming a
    // schedule only zero-fills the used prefix (std::fill, no realloc).
    echo::EchoGrid d_grid_;
    uint64_t d_tx_samples_ = 0;
    uint64_t d_rx_samples_ = 0;
    uint64_t d_max_frag_ = 0;
    uint64_t d_burst_budget_ = 0; // 0 = unlimited
    uint64_t d_pulse_id_ = 0;
    uint64_t d_bursts_this_grid_ = 0;
    double d_sample_rate_ = 0.0;
    pmt::pmt_t d_tx_payload_ = pmt::PMT_NIL;
    pmt::pmt_t d_sched_meta_ = pmt::PMT_NIL;
    const int16_t* d_tx_ptr_ = nullptr;
    std::vector<int16_t> d_rx_buf_; // fixed: max_rx_samples * 2 int16
    echo::BurstFragment d_txf_[echo::kEchoMaxFragmentsPerBurst] = {};
    echo::BurstFragment d_rxf_[echo::kEchoMaxFragmentsPerBurst] = {};
    std::atomic<bool> d_armed_{ false };

    // Bounded schedule queue (handler enqueues, worker drains).
    mutable std::mutex d_queue_mutex_;
    std::condition_variable d_queue_cv_;
    std::deque<Job> d_queue_;
    std::atomic<bool> d_stop_{ false };
    std::thread d_worker_;
    std::atomic<bool> d_worker_busy_{ false };

    // Counters.
    std::atomic<uint64_t> d_received_{ 0 };
    std::atomic<uint64_t> d_enqueued_{ 0 };
    std::atomic<uint64_t> d_dropped_{ 0 };
    std::atomic<uint64_t> d_invalid_{ 0 };
    std::atomic<uint64_t> d_dropped_on_stop_{ 0 };
    std::atomic<uint64_t> d_bursts_published_{ 0 };
    std::atomic<uint64_t> d_bursts_ok_{ 0 };
    std::atomic<uint64_t> d_bursts_failed_{ 0 };
    std::atomic<uint64_t> d_late_slot_skips_{ 0 };
    std::atomic<uint64_t> d_tx_reissues_{ 0 };
    std::atomic<uint64_t> d_rx_reissues_{ 0 };
    std::atomic<uint64_t> d_grid_errors_{ 0 };
    std::atomic<size_t> d_queue_depth_{ 0 };
    std::atomic<size_t> d_queue_high_watermark_{ 0 };
};

} // namespace uwb
} // namespace gr

#endif /* INCLUDED_GNURADIO_UWB_UWB_REALTIME_ECHO_TIMER_H */
