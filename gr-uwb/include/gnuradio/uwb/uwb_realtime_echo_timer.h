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
 *   pulse_id_increment(u64, optional)     when != 0, emit
 *                                         pulse_id + (schedule_index -
 *                                         schedule_index_of_this_PDU) per
 *                                         burst so a single multi-burst
 *                                         schedule yields unique ids (0 =
 *                                         constant passthrough, default)
 *   burst_count       (u64, optional)     legacy: attempted bursts; active
 *                                         jammer scan: successful RX target,
 *                                         required to equal points*dwell
 *                                         (default 0 = unlimited otherwise)
 *   max_fragment_size (u64, optional)     override the prepared planning
 *                                         chunk for this schedule (default
 *                                         block config, must be > 0)
 *   sample_rate       (real, optional)    device sample rate, used only to
 *                                         add tx_time_full/tx_time_frac and
 *                                         rx_time_full/rx_time_frac metadata
 *
 * Multi-TX input (dual-channel jammer, planning §5.3; M3/M4/M5):
 * cons(meta_dict, pmt_vector[s16vector_ch0, ...]).  The payload holds one
 * *effective waveform* per TX channel (never a per-pulse (2, L) composite).
 * Vector length must be 1..kEchoMaxTxChannels (4); a 1-element vector is
 * exactly equivalent to the legacy single-TX form above.  Each element must
 * be an s16vector with an even element count (SC16 pairs); channel 0 is the
 * sensing waveform.  Additional metadata keys (all integer ticks floats are
 * converted by the Python/control layer; the worker does no float geometry):
 *   tx_channel_count      (u64/int, optional) must equal the vector length
 *                                             (default = vector length)
 *   tx_samples            (u64, required)     physical burst length L per
 *                                             channel, > 0, <= max cap
 *   tx_waveform_samples   (u64vector, optional) per-channel pair counts;
 *                                             must match the payload lengths
 *                                             (default = payload lengths)
 *   tx_base_offsets_native(u64vector, optional) per-channel base start in
 *                                             [0, L) (default all 0)
 *   jam_logical_channel   (u64/int, default 1) retuned TX channel, in
 *                                             [1, channel_count)
 *   jam_delay_mode        ("fixed"/"uniform" symbol, or 0/1; default fixed)
 *   jam_delay_lo_native   (i64, default 0)    fixed: the per-pulse delay;
 *   jam_delay_hi_native   (i64, default 0)    uniform: draw range [lo, hi]
 *   jam_delay_seed        (u64, default 0)    uniform PRNG seed (frozen arm)
 *   jam_freq_offsets_hz   (f64vector, optional) dwell scan plan (empty = off)
 *   jam_dwell             (u64, default 0)    successful full RX captures per
 *                                             scan step (0 = off)
 *   jam_freq_settle_ticks (u64, default 0)    re-anchor delay after retune
 *
 * Geometry convention (§5.4, contiguous-window remediation): D =
 * tx_base_offsets_native[0] (sense parked at D).  Uniform mode: per-pulse
 * jammer begin = D + delay with delay in [-D, +D] drawn from the frozen
 * JamDelayRng; L must equal prepare_multitx_geometry(sense, jam, D).
 * Fixed mode (lo == hi): jammer begin = D + lo must equal
 * tx_base_offsets_native[jam], and L must equal max over channels of
 * (base + waveform length).  The handler materializes an immutable
 * MultiTxWindowBank (dense rows of length L, jam backing of length L+2D
 * in uniform mode) and rejects max_fragment_size < L so each burst is
 * exactly one data fragment.  Delay never changes TX0 pointer, length,
 * or sample positions, and never changes send boundaries.
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
 * Multi-TX bursts additionally carry (§5.2/§5.6, present on every burst):
 *   tx_channel_count, tx_wire_samples_requested/sent (channels x per-ch),
 *   tx_async_underflow/seq_error/time_error/unmatched/dropped/ack, and
 *   (multi-TX bursts only) jam_delay_native/us, jam_freq_plan/actual/offset
 *   _hz, jam_retune_seq.  Single-TX output semantics are otherwise unchanged
 *   (same PDU per index, same samples, same legacy keys/values).
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

#ifdef UWB_HAVE_UHD
#include <gnuradio/uwb/uwb_uhd_backend_config.h>
#endif

#include <pmt/pmt.h>

#include <atomic>
#include <array>
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

#ifdef UWB_HAVE_UHD
    /**
     * UHD convenience factory: builds a UhdBurstBackend from uhd_cfg and
     * forwards to make().  Block type stays message-only gr::block.
     */
    static sptr make_uhd(const uhd::UhdBurstBackendConfig& uhd_cfg,
                         const echo::EchoSchedulerConfig& sched_cfg,
                         size_t queue_capacity = 64,
                         uint64_t rx_collect_wait_ms = 1000,
                         uint64_t max_tx_samples = 1u << 21,
                         uint64_t max_rx_samples = 1u << 21);
#endif

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

    // --- runtime control (thread-safe; applied by the radio worker) ---
    // set_freq stores a pending tune; the worker calls backend->tune() at the
    // next burst boundary (serialized with all UHD I/O).  freq() returns the
    // last successfully applied centre frequency (0 before the first apply).
    void set_freq(double hz);
    double freq() const;
    // Pending calibration delay (native samples) read when a burst PDU is
    // published; never touched from the worker's I/O path.
    void set_cal_delay_native(double native_samples);
    double cal_delay_native() const;

    // --- publish ROI (M2) ---
    // 0 = publish the whole physical RX window (golden/fallback);
    // >0 = publish only this many leading native SC16 sample pairs.
    // Resolution of "-1 auto" is done by the caller (Python), which passes an
    // explicit >0 value here.  Values >= rx_samples_received are a no-op.
    void set_publish_native(uint64_t n);
    uint64_t publish_native() const;
    uint64_t published_samples_total() const;   // sum of published sample pairs

    // --- radio-deadline timing (M0/M1) ---
    int64_t device_time_ticks() const;          // backend clock, 0 if unarmed
    uint64_t last_worker_us() const;            // last burst worker wall time
    uint64_t max_worker_us() const;
    uint64_t mean_worker_us() const;            // total_worker_us / bursts_published

    // Per-burst worker segment timing (D; diagnostics only — never changes
    // scheduling or publishing).  Means are total / bursts_published();
    // every counter below is reset in start().
    uint64_t slot_lead_us_last() const;    // planned t_tx - now at burst start
    uint64_t issue_rx_us_mean() const;     // backend issue_rx() wall
    uint64_t tx_send_us_mean() const;      // backend issue_tx() wall
    uint64_t rx_collect_us_mean() const;   // backend collect_result() wall
    uint64_t pdu_build_us_mean() const;    // meta+PMT build in publish_burst
    uint64_t pdu_publish_us_mean() const;  // message_port_pub in publish_burst
    uint64_t worker_total_us_mean() const; // alias of mean_worker_us()

    // --- multi-TX introspection (M3/M4/M5; thread-safe atomics) ---
    // Active TX channel count (1 after a legacy arm; N after a multi arm).
    size_t tx_channel_count() const;
    // Fixed zero-scratch introspection (QA): allocated once at construction
    // for max_tx_samples, capacity and data address never change afterwards.
    size_t zero_scratch_capacity() const;
    const int16_t* zero_scratch_data() const;
    // Armed contiguous window bank (nullptr when single-TX / unarmed).
    // Worker-owned; callers must only read after the grid has been armed
    // and not concurrently with a re-arm.
    const echo::MultiTxWindowBank* armed_window_bank() const;
    // Jammer channel/plan state frozen at arm.
    size_t jam_logical_channel() const;
    // Last multi-TX burst values (stale when the armed grid is single-TX).
    int64_t jam_delay_native_last() const; // relative delay, may be < 0
    double jam_delay_us_last() const;
    double jam_freq_plan_hz_last() const;   // absolute target of last burst
    double jam_freq_actual_hz_last() const; // last NCO readback
    double jam_freq_offset_hz_last() const; // last applied plan offset
    uint64_t jam_retunes() const;           // successful jam retunes, grid
    uint64_t jam_retune_failures() const;   // failed jam retune attempts
    // Live TX async event counters (§5.6) as a PMT dict with u64 keys:
    // underflow, seq_error, time_error, unmatched, dropped, ack.
    pmt::pmt_t tx_async_counts() const;

    // Last per-burst backend error string (diagnostics).
    std::string last_error() const;

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
        uint64_t burst_count = 0; // attempts, or successful RX target for scan
        uint64_t publish_native = 0;    // 0 = block-level setting (M2 ROI)
        uint64_t pulse_id_increment = 0; // != 0 → per-burst unique ids
        double sample_rate = 0.0;
        pmt::pmt_t meta = pmt::PMT_NIL;    // input dict (passthrough)
        pmt::pmt_t payload = pmt::PMT_NIL; // s16vector TX burst (immutable)
        // --- multi-TX (M3/M4/M5; only meaningful when tx_channel_count>1)
        size_t tx_channel_count = 1; // 1 = legacy single-TX path
        // Per-channel waveforms (each an s16vector; holds PMT refs so the
        // underlying buffers stay alive for the whole grid).
        std::vector<pmt::pmt_t> tx_payloads;
        // Per-channel effective waveform lengths (pairs) and base offsets.
        std::array<uint64_t, echo::kEchoMaxTxChannels> tx_waveform_samples{};
        std::array<uint64_t, echo::kEchoMaxTxChannels> tx_base_offsets{};
        size_t jam_logical_channel = 1;
        echo::JamDelayMode jam_delay_mode = echo::JamDelayMode::Fixed;
        int64_t jam_delay_lo_native = 0;
        int64_t jam_delay_hi_native = 0;
        uint64_t jam_delay_seed = 0; // frozen PRNG seed (uniform mode)
        std::vector<double> jam_freq_offsets_hz; // dwell scan plan (maybe empty)
        uint64_t jam_dwell = 0; // successful full RX captures/step (0 = off)
        uint64_t jam_freq_settle_ticks = 0;      // re-anchor delay, ticks
        // Immutable contiguous window bank, built in handle_schedule.
        std::shared_ptr<const echo::MultiTxWindowBank> window_bank;
    };

    // Per-burst multi-TX publish extras (worker stack only, never shared).
    // multi == false reproduces the exact legacy metadata (plus the §5.2 /
    // §5.6 accounting keys that are present on every burst).
    struct PubExtra {
        bool multi = false;
        int64_t jam_delay_native = 0; // relative delay in native samples
        double jam_delay_us = 0.0;
        double jam_freq_plan_hz = 0.0;   // absolute NCO target this burst
        double jam_freq_actual_hz = 0.0; // last NCO readback
        double jam_freq_offset_hz = 0.0; // plan offset this burst
        uint64_t jam_retune_seq = 0;     // successful retunes so far, grid
        uint64_t jam_scan_step = 0;
        uint64_t jam_dwell_target = 0;
        uint64_t jam_dwell_successes_before = 0;
        echo::JamDelayMode jam_delay_mode = echo::JamDelayMode::Fixed;
        uint64_t jam_delay_seed = 0;
        uint64_t sense_offset_native = 0;
        uint64_t tx_fragment_count = 0;
        int64_t sense_tx_ticks = 0;
        int64_t jam_tx_ticks = 0;
    };

    void handle_schedule(pmt::pmt_t msg);
    bool enqueue(Job&& job);
    void worker_loop();
    void apply_schedule(const Job& job);
    void run_one_burst();
    void finish_grid();
    void publish_burst(const echo::BurstResult& r,
                       bool with_samples,
                       const PubExtra& px);
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
    // Per-burst publish ROI override from the schedule PDU (0 = use the
    // block-level d_publish_native_).  Worker-owned; read in publish_burst.
    uint64_t d_burst_publish_native_ = 0;
    // Per-burst pulse_id: when the schedule requests incrementing, the
    // emitted id is d_pulse_id_ + (schedule_index - d_sched_index_start_).
    uint64_t d_pulse_id_increment_ = 0;
    uint64_t d_sched_index_start_ = 0;
    std::vector<int16_t> d_rx_buf_; // fixed: max_rx_samples * 2 int16
    echo::BurstFragment d_txf_[echo::kEchoMaxFragmentsPerBurst] = {};
    echo::BurstFragment d_rxf_[echo::kEchoMaxFragmentsPerBurst] = {};
    // --- multi-TX frozen state (M3/M4/M5; worker-owned, set in
    // apply_schedule, read-only on the per-burst hot path) ---
    bool d_mtx_armed_ = false; // true when the armed grid uses N > 1
    size_t d_mtx_N_ = 1;
    echo::MultiTxGeometry d_mtx_geom_;
    // Per-channel effective lengths / base offsets (copies of the Job).
    std::array<uint64_t, echo::kEchoMaxTxChannels> d_mtx_wave_len_{};
    std::array<uint64_t, echo::kEchoMaxTxChannels> d_mtx_base_{};
    size_t d_mtx_jam_ch_ = 1;
    echo::JamDelayMode d_mtx_delay_mode_ = echo::JamDelayMode::Fixed;
    int64_t d_mtx_delay_lo_ = 0;
    int64_t d_mtx_delay_hi_ = 0;
    // PMT lifetime holders for the per-channel waveforms (reserved for
    // kEchoMaxTxChannels at construction: re-arming never allocates).
    std::vector<pmt::pmt_t> d_mtx_payloads_;
    // Frozen per-channel waveform bases ("UHD buffer pointer array": the
    // per-fragment send pointers live in d_mtxf_[i].tx_data, resolved from
    // these bases or d_zero_scratch_ on every burst without allocation).
    const int16_t* d_mtx_wave_ptrs_[echo::kEchoMaxTxChannels] = {};
    // All-zero TX scratch, fixed max_tx_samples * 2 int16 from construction;
    // capacity and address never change; the worker only reads it.
    std::vector<int16_t> d_zero_scratch_;
    // Fixed multi-TX fragment array (hot path writes only, no allocation).
    echo::TxBurstFragment d_mtxf_[echo::kEchoMaxFragmentsPerBurst] = {};
    // Dwell/scan plan (reserved 2048 at construction; larger plans allocate
    // once here in apply_schedule, never on the per-burst path).
    std::vector<double> d_mtx_freqs_;
    uint64_t d_mtx_dwell_ = 0;
    uint64_t d_mtx_settle_ticks_ = 0;
    double d_mtx_base_hz_ = 0.0; // frozen absolute base for scan targets
    size_t d_mtx_step_ = 0;      // current scan step (SIZE_MAX = unknown)
    // Scan dwell is counted from successfully published RX captures, never
    // from schedule_index/pulse_id.  A completed dwell requests one forward
    // retune; the final step completes the grid without modulo wraparound.
    uint64_t d_mtx_ok_in_step_ = 0;
    bool d_mtx_advance_pending_ = false;
    double d_mtx_jam_offset_hz_ = 0.0; // assumed/applied jammer offset state
    double d_mtx_jam_actual_hz_ = 0.0; // last jammer NCO readback
    echo::JamDelayRng d_mtx_rng_;      // seeded at arm; one draw per burst
    // Contiguous window bank frozen at apply_schedule; hot path read-only.
    std::shared_ptr<const echo::MultiTxWindowBank> d_mtx_bank_;
    uint64_t d_mtx_delay_seed_ = 0;
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

    // Runtime control / ROI / worker timing (atomics: set from Python or
    // message threads, read in the worker / publish path).
    std::atomic<double> d_freq_hz_{ 0.0 };            // last applied centre freq
    std::atomic<double> d_cal_delay_native_{ 0.0 };
    std::atomic<uint64_t> d_publish_native_{ 0 };     // 0 = full window
    std::atomic<uint64_t> d_published_samples_{ 0 };
    std::atomic<uint64_t> d_worker_us_total_{ 0 };
    std::atomic<uint64_t> d_worker_us_max_{ 0 };
    std::atomic<uint64_t> d_worker_us_last_{ 0 };
    // Per-burst segment timing (D): atomic totals + maxima, reset in
    // start().  Diagnostics only (relaxed atomics, no print/allocation);
    // they never feed back into the scheduler or the publish path.
    std::atomic<uint64_t> d_slot_lead_us_last_{ 0 };
    std::atomic<uint64_t> d_slot_lead_us_total_{ 0 };
    std::atomic<uint64_t> d_slot_lead_us_max_{ 0 };
    std::atomic<uint64_t> d_issue_rx_us_total_{ 0 };
    std::atomic<uint64_t> d_issue_rx_us_max_{ 0 };
    std::atomic<uint64_t> d_tx_send_us_total_{ 0 };
    std::atomic<uint64_t> d_tx_send_us_max_{ 0 };
    std::atomic<uint64_t> d_rx_collect_us_total_{ 0 };
    std::atomic<uint64_t> d_rx_collect_us_max_{ 0 };
    std::atomic<uint64_t> d_pdu_build_us_total_{ 0 };
    std::atomic<uint64_t> d_pdu_build_us_max_{ 0 };
    std::atomic<uint64_t> d_pdu_publish_us_total_{ 0 };
    std::atomic<uint64_t> d_pdu_publish_us_max_{ 0 };
    // pending tune request (worker consumes at burst boundary)
    std::atomic<bool> d_tune_pending_{ false };
    std::atomic<double> d_tune_hz_{ 0.0 };
    // --- multi-TX atomic mirrors (M3/M4/M5; worker writes, anyone reads)
    std::atomic<size_t> d_atomic_tx_channels_{ 1 };
    std::atomic<size_t> d_atomic_jam_ch_{ 1 };
    std::atomic<int64_t> d_jam_delay_last_{ 0 };
    std::atomic<double> d_jam_delay_us_last_{ 0.0 };
    std::atomic<double> d_jam_plan_last_{ 0.0 };
    std::atomic<double> d_jam_actual_last_{ 0.0 };
    std::atomic<double> d_jam_offset_last_{ 0.0 };
    std::atomic<uint64_t> d_jam_retunes_{ 0 };
    std::atomic<uint64_t> d_jam_retune_fails_{ 0 };
    mutable std::mutex d_err_mutex_;
    std::string d_last_error_;
};

} // namespace uwb
} // namespace gr

#endif /* INCLUDED_GNURADIO_UWB_UWB_REALTIME_ECHO_TIMER_H */
