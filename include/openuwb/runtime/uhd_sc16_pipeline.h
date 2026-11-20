/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * UHD SC16 Direct RX and Multi-Core Stream Pipeline.
 * Bypasses GNU Radio scheduler and PMT/PDU messaging overhead.
 */

#pragma once

#include <openuwb/support/api.h>
#include <openuwb/sensing/radar_cir_core.h>
#include <openuwb/runtime/scheduled_extractor_core.h>
#include <openuwb/radio/sc16_stream_source.h>
#include <openuwb/sic/sic_unified.h>

#include <atomic>
#include <chrono>
#include <complex>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace openuwb {
namespace core {
using openuwb::radar::RadarCirConfig;
}
}

namespace openuwb {
namespace pipeline {

// Defined in uwb_fixed_tone_cancel_source.h.  The pipeline owns the stage
// through source_ (as an ISc16StreamSource) and exposes a non-owning pointer
// for diagnostics only.
class FixedToneCancelSc16Source;

enum class StreamSourceType {
    Replay,
    UhdDevice
};

enum class ConsumerMode {
    SicOnly,
    SensingCir,
    Both
};

enum class WindowTerminalState {
    Completed,
    ProcessingFailed,
    DroppedAdmission,
    InvalidGap,
    Cancelled
};

enum class SlotState : uint32_t {
    Free = 0,
    Filling = 1,
    Ready = 2,
    Reading = 3
};

enum class Sc16ScaleMode {
    UnitRange, // 1.0f / 32768.0f (default)
    RawInteger // 1.0f
};

// ---------------------------------------------------------------------------
// DSP worker readiness / failure propagation (frozen contract §3)
// ---------------------------------------------------------------------------
// A worker starts Pending, publishes Ready after affinity + every allocation /
// template load / scratch preparation succeeded, and publishes Failed (with a
// human readable reason) instead of dying when that preparation throws.
enum class WorkerReadyState {
    Pending,
    Ready,
    Failed
};

struct WorkerReadyInfo {
    WorkerReadyState state = WorkerReadyState::Pending;
    std::string error; // non-empty when state == Failed
    bool affinity_requested = false;
    bool affinity_applied = false;
    int affinity_core = -1;
};

// RX-side counters.  Only accumulated when
// PipelineConfig::enable_rx_diagnostics is true; `enabled` records that fact so
// a caller can tell "zero because nothing happened" from "not measured".
struct RxDiagnostics {
    bool enabled = false;
    uint64_t read_calls = 0;
    uint64_t samples_read = 0; // complex samples actually returned by the source
    uint64_t bytes_read = 0;   // samples_read * sizeof(complex<int16_t>)
    uint64_t windows_published = 0;
    uint64_t windows_dropped = 0;
    uint64_t slot_copy_bytes = 0;
    uint64_t stream_buf_resize_count = 0;    // resizes that changed the length
    uint64_t stream_buf_zero_init_bytes = 0; // value-initialised tail bytes
    uint64_t compact_move_bytes = 0;         // bytes moved by the existing erase()
    uint64_t max_buffered_samples = 0;       // peak valid samples in stream_buf
    double read_us = 0.0;                    // source_->read_samples
    double assemble_us = 0.0;                // stream_buf compaction / erase
    double slot_copy_us = 0.0;               // slot memcpy
    double dispatch_us = 0.0;                // pool probing + publish
    double ledger_wait_us = 0.0;             // RX-side ledger_mutex_ wait
};

// Commit-path counters.  Only accumulated when
// PipelineConfig::enable_commit_diagnostics is true.
struct CommitDiagnostics {
    bool enabled = false;
    uint64_t commit_calls = 0;
    uint64_t results_emitted = 0;
    uint64_t max_out_of_order_span = 0; // max(window_id - next_commit_id_) seen
    uint64_t max_pending_results = 0;   // peak buffered out-of-order WindowResult count
    uint64_t reorder_backpressure_waits = 0; // RX producer waits on reorder capacity
    uint64_t reorder_overflow_drops = 0;     // RX producer drops due to reorder capacity
    double lock_wait_us = 0.0;          // waiting for commit_mutex_
    double reorder_us = 0.0;            // insertion + in-order flush, no callback
    double callback_us = 0.0;           // user result callback
};

// StreamReadMeta and ISc16StreamSource live in uwb_sc16_stream_source.h so
// the tone-cancel decorator can consume the source contract without this
// whole pipeline header.

struct WindowTimingStats {
    double slice_us = 0.0;
    double queue_us = 0.0;
    double convert_us = 0.0;
    double resample_us = 0.0;
    double sic_us = 0.0;
    double sensing_us = 0.0;
    double e2e_us = 0.0;
};

struct WindowResult {
    uint64_t window_id = 0;
    // Burst-level multi-core contract: index of the DSP worker that produced
    // this window, in [0, dsp_workers).  Always 0 when dsp_workers == 1.
    // Windows that never reached a worker (DroppedAdmission) carry -1.
    int worker_id = 0;
    WindowTerminalState state = WindowTerminalState::Completed;
    int64_t native_start_abs = 0;
    size_t native_sample_count = 0;
    WindowTimingStats timing;

    // Monotonic (steady_clock) stage boundaries, used by the runner to delimit
    // the steady-state measurement interval (frozen contract §2).
    //   rx_push_time     : sampled just before publish_ready_slot(); identical
    //                      value to Sc16WindowSlot::push_time, so queue_us keeps
    //                      its historical meaning.
    //   worker_start_time: worker picked the slot up.
    //   worker_end_time  : DSP finished, slot about to be released.
    //   commit_time      : commit_result finished the in-order emit (callback
    //                      included).
    // DroppedAdmission results only carry rx_push_time.
    std::chrono::steady_clock::time_point rx_push_time{};
    std::chrono::steady_clock::time_point worker_start_time{};
    std::chrono::steady_clock::time_point worker_end_time{};
    std::chrono::steady_clock::time_point commit_time{};

    // SIC result
    bool sic_executed = false;
    bool sic_committed = false;
    float actual_suppression_db = 0.0f;
    std::string sic_status;

    // Sensing result
    bool radar_executed = false;
    bool radar_sfd_found = false;
    float radar_peak_metric = 0.0f;
    int64_t radar_detected_start = -1;
};

struct UWB_API PipelineLedger {
    uint64_t offered_windows = 0;
    uint64_t admitted_windows = 0;
    uint64_t completed_windows = 0;
    uint64_t processing_failed = 0;
    uint64_t dropped_admission = 0;
    uint64_t invalid_gap_before_admission = 0;
    uint64_t invalid_gap_after_admission = 0;
    uint64_t cancelled_windows = 0;

    uint64_t total_samples_received = 0;
    uint64_t short_reads = 0;
    uint64_t read_timeouts = 0;
    uint64_t read_overflows = 0;

    bool verify_invariants(std::string* err = nullptr) const;
};

struct PipelineConfig {
    StreamSourceType source_type = StreamSourceType::Replay;
    std::string replay_file;
    bool paced_replay = false;

    double native_rate = 998.4e6;
    double work_rate = 998.4e6;
    double schedule_t0 = 0.0;
    double period_s = 0.001; // 1 ms default
    size_t pre_guard_samples = 400;
    size_t capture_samples = 3200;
    size_t post_guard_samples = 400;

    size_t pool_capacity = 8;
    ConsumerMode consumer_mode = ConsumerMode::SicOnly;
    int cpu_core_rx = -1;
    int cpu_core_dsp = -1;
    Sc16ScaleMode scale_mode = Sc16ScaleMode::UnitRange;

    // ------------------------------------------------------------------
    // Burst-level multi-core consumer contract (run-to-completion)
    // ------------------------------------------------------------------
    // Number of independent DSP consumer threads.  Each worker owns a
    // private SPSC ring pool, a private resampler core and private SIC /
    // radar scratch, and processes one whole burst on a single core using
    // the flat AVX2 4-way macroblock kernel.  1 = legacy single-consumer.
    int dsp_workers = 1;
    // Optional explicit affinity list for the DSP workers.  When non-empty
    // it overrides cpu_core_dsp and must have exactly dsp_workers entries.
    std::vector<int> cpu_cores_dsp;

    // Resampler configuration
    std::string resampler_profile = "quality_minorder";
    // NOTE: the former `resampler_workers` field was removed in the
    // 2026-11-15 native cleanup: every DSP worker already forces its private
    // resampler core to num_workers == 1 (burst run-to-completion), so the
    // entry was dead and misleading.

    // Phase-C kernel experiments (resampler core).  Each non-"none" value is
    // BIT-IDENTICAL in output to "none"; these are performance experiments.
    //
    // "fixed_phase" is the measured default for this pipeline: 5 ABBA pairs on
    // the burst pipeline gave -9.1% (56k realtime N=1), -5.2% (56k quality
    // N=1), -5.7% (4k realtime N=1) and -3.7% (4k quality N=1) steady
    // us/window, resample mean -11.6%/-6.1%/-4.6%/-3.9%, and P95/P99 worse in
    // no cell.  The core's own default stays off, so other users of
    // RationalResamplerLmCore are unaffected; `none` opts back out here.
    //
    // Each DSP worker applies the switch to its own private resampler core, so
    // the choice is per-worker state and never shared.  Accepted values:
    //   "none"          default kernel (no setter is called at all)
    //   "fixed_phase"   core.set_fixed_phase_schedule(true)
    // "convert_cache"/"both" were measured, NOT retained and REMOVED in the
    // 2026-11-15 native cleanup.  Any other non-empty value (including the
    // removed ones) is a configuration error: initialize() prints one
    // diagnostic line and fails closed to "none" (it is never silently
    // accepted as an unknown experiment).
    std::string resampler_experiment = "fixed_phase"; // none|fixed_phase

    // ------------------------------------------------------------------
    // Phase 0 diagnostics (frozen contract §1)
    // ------------------------------------------------------------------
    // Both default to false.  While false the only cost on the hot path is a
    // single boolean test; no clock reads, no extra synchronisation.
    bool enable_rx_diagnostics = false;
    bool enable_commit_diagnostics = false;

    // QA-only injection hook, called on each DSP worker thread after the
    // affinity is applied and before any allocation.  Throwing moves that
    // worker to WorkerReadyState::Failed and propagates the message; the
    // production path leaves it empty (then it costs nothing, not even a
    // synchronisation).
    std::function<void(int worker_id)> test_worker_init_hook;

    // ------------------------------------------------------------------
    // Diagnostic per-window IQ dump (default off)
    // ------------------------------------------------------------------
    // Dumps the exact CF32 samples handed to SIC/radar: the post-resample
    // buffer (or the direct SC16->CF32 buffer when no resampling is needed)
    // with exactly dsp_n samples, for the first `iq_dump_windows` COMPLETED
    // windows whose window_id is < iq_dump_window_limit.
    //
    // Files: <iq_dump_dir>/win_<window_id>_<tag>.cf32, raw little-endian
    // complex<float> (real/imag float32 pairs).  An empty tag is written as
    // "dump" so the name always parses as win_<id>_<tag>.cf32.
    // Selection uses an atomic completion counter, so with dsp_workers > 1 the
    // chosen set follows completion order; dsp_workers == 1 is deterministic.
    //
    // The write happens on the worker thread after the resampler/flush produced
    // dsp_n and before the SIC call.  Its duration is excluded from resample_us,
    // convert_us and e2e_us, and it costs one boolean test per window when
    // iq_dump_dir is empty.
    std::string iq_dump_dir;         // empty = disabled
    std::string iq_dump_tag;         // filename suffix, e.g. "control" / "B"
    size_t iq_dump_windows = 0;      // 0 = disabled
    size_t iq_dump_window_limit = 0; // only window_id < this (0 = no limit)

    uint64_t max_windows = 0; // 0 = unlimited
    uint64_t max_samples = 0; // 0 = unlimited

    // Result retention & ordered commit bounds:
    // - retain_results = true (default) keeps every emitted WindowResult in
    //   results() for post-run inspection.  false enables callback-only mode
    //   so results() stays empty and retention memory stays flat on long runs.
    // - max_reorder_pending_windows == 0 disables the explicit reorder limit
    //   (only pool_capacity bounds in-flight windows).  > 0 caps both the
    //   live out-of-order span (window_id - next_commit_id_) and the number of
    //   uncommitted buffered results before admitting a new window.
    // - backpressure_on_reorder_limit = true makes rx_producer_loop() wait for
    //   next_commit_id_ to advance; false explicitly drops the new window as
    //   DroppedAdmission so worker/pool capacity remains reserved for the
    //   earliest unfinished window.
    bool retain_results = true;
    size_t max_reorder_pending_windows = 0;
    bool backpressure_on_reorder_limit = false;

    // ------------------------------------------------------------------
    // Fixed-frequency CW (tone) cancellation, applied to the raw SC16 stream
    // BEFORE the window assembler (see uwb_fixed_tone_cancel_source.h).
    //
    // Explicit opt-in: default false keeps the original source path
    // instruction-for-instruction identical (the plain source is used directly,
    // no decorator, no extra thread, no extra copy).
    // ------------------------------------------------------------------
    bool fixed_tone_cancel_enable = false;
    // Receive baseband frequency in Hz (f_RF - f_LO), NOT the RF centre
    // frequency.  The validated default is -232.96 MHz at 737.28 MS/s, where
    // f0/fs == -91/288 and the complex exponential is exactly 288-periodic.
    double fixed_tone_cancel_frequency_hz = -232.96e6;
    double fixed_tone_cancel_period_s = 20.0e-3;
    double fixed_tone_cancel_training_s = 0.5e-3;
    size_t fixed_tone_cancel_crossfade_samples = 256;
    // Bounded raw ring: whole blocks are dropped (never stalling recv) when it
    // is full.  It must hold at least one training window.
    size_t fixed_tone_cancel_block_samples = 65536;
    size_t fixed_tone_cancel_queue_blocks = 32;

    // UHD specific
    std::string uhd_args;
    std::string uhd_subdev;
    std::string uhd_channel;
    std::string uhd_antenna; // empty keeps the device default
    double uhd_center_freq = 3993.6e6;
    double uhd_gain = 30.0;
    std::string uhd_clock_source;
    std::string uhd_time_source;

    // DSP configs
    openuwb::sic::SicUnifiedConfig sic_cfg;
    openuwb::core::RadarCirConfig radar_cfg;
    // Optional override for the public SYNC template used by SIC (one shaped
    // SYNC symbol, 1016 CF32 samples at 998.4 MS/s).  Empty keeps the
    // historical `testdata/reference_preamble.bin` (sensing code 9).  A SIC
    // run against a comm packet of a different code MUST set this, otherwise
    // the comm demodulator correlates against the wrong SYNC waveform.
    std::string sic_sync_template_file;

    size_t native_window_capacity() const {
        return pre_guard_samples + capture_samples + post_guard_samples;
    }
};

struct Sc16WindowSlot {
    alignas(64) std::atomic<SlotState> state{SlotState::Free};
    uint64_t window_id = 0;
    int64_t native_start_abs = 0;
    size_t native_count = 0;
    double native_rate = 0.0;
    double work_rate = 0.0;
    std::chrono::steady_clock::time_point push_time;
    std::vector<std::complex<int16_t>> sc16_samples;
};

// ---------------------------------------------------------------------------
// Conversion kernels
// ---------------------------------------------------------------------------
UWB_API void sc16_to_cf32_scalar(const std::complex<int16_t>* in,
                                 std::complex<float>* out,
                                 size_t n,
                                 float scale = 1.0f / 32768.0f);

UWB_API void sc16_to_cf32_avx2(const std::complex<int16_t>* in,
                               std::complex<float>* out,
                               size_t n,
                               float scale = 1.0f / 32768.0f);

inline void sc16_to_cf32(const std::complex<int16_t>* in,
                         std::complex<float>* out,
                         size_t n,
                         float scale = 1.0f / 32768.0f)
{
#if defined(__AVX2__) && defined(__FMA__)
    sc16_to_cf32_avx2(in, out, n, scale);
#else
    sc16_to_cf32_scalar(in, out, n, scale);
#endif
}

// ---------------------------------------------------------------------------
// Lock-free SPSC Ring Pool
// ---------------------------------------------------------------------------
class UWB_API Sc16WindowRingPool {
public:
    explicit Sc16WindowRingPool(size_t capacity, size_t slot_sample_capacity);
    ~Sc16WindowRingPool();

    size_t capacity() const { return capacity_; }

    // Producer methods
    Sc16WindowSlot* acquire_filling_slot();
    bool publish_ready_slot(Sc16WindowSlot* slot);
    void abort_filling_slot(Sc16WindowSlot* slot);

    // Consumer methods
    Sc16WindowSlot* acquire_ready_slot();
    void release_reading_slot(Sc16WindowSlot* slot);

private:
    const size_t capacity_;
    const size_t slot_sample_capacity_;
    std::vector<std::unique_ptr<Sc16WindowSlot>> slots_;

    alignas(64) std::atomic<size_t> write_idx_{0};
    alignas(64) std::atomic<size_t> read_idx_{0};
};

// ---------------------------------------------------------------------------
// Data Sources
// ---------------------------------------------------------------------------
class UWB_API ReplaySc16Source : public ISc16StreamSource {
public:
    ReplaySc16Source(const std::string& filepath,
                     double sample_rate,
                     bool paced = false);
    ~ReplaySc16Source() override;

    bool open() override;
    // Re-anchors the paced-replay time base.  open() must not start acquisition,
    // and the paced target must be measured from the moment streaming actually
    // begins (the pipeline opens the source long before its ready barrier), so
    // a stale reference time would make the source burst at full speed.
    bool start_stream() override;
    void close() override;
    size_t read_samples(std::complex<int16_t>* dest,
                        size_t max_samples,
                        StreamReadMeta& meta) override;
    double actual_rx_rate() const override { return sample_rate_; }
    bool is_eof() const override { return eof_; }

private:
    std::string filepath_;
    double sample_rate_;
    bool paced_;
    bool eof_{false};
    int64_t current_sample_abs_{0};
    int fd_{-1};
    std::chrono::steady_clock::time_point start_time_;
};

class UWB_API UhdDeviceSc16Source : public ISc16StreamSource {
public:
    explicit UhdDeviceSc16Source(const PipelineConfig& cfg);
    ~UhdDeviceSc16Source() override;

    bool open() override;
    void close() override;
    // Issues STREAM_MODE_START_CONTINUOUS.  Split out of open() so the device
    // does not start streaming while the DSP workers are still initialising.
    bool start_stream() override;
    size_t read_samples(std::complex<int16_t>* dest,
                        size_t max_samples,
                        StreamReadMeta& meta) override;
    double actual_rx_rate() const override;
    bool is_eof() const override { return eof_; }
    static bool is_supported();

private:
    PipelineConfig cfg_;
    bool eof_{false};
    double actual_rate_{0.0};
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

// ---------------------------------------------------------------------------
// Pipeline Manager
// ---------------------------------------------------------------------------
class UWB_API UhdSc16Pipeline {
public:
    using ResultCallback = std::function<void(const WindowResult&)>;

    explicit UhdSc16Pipeline(const PipelineConfig& cfg);
    ~UhdSc16Pipeline();

    bool initialize(std::unique_ptr<ISc16StreamSource> custom_source = nullptr);
    void start();
    void stop();
    void wait();

    // ------------------------------------------------------------------
    // Worker readiness / failure (frozen contract §3, A2)
    // ------------------------------------------------------------------
    // Blocks until every DSP worker published Ready, any worker published
    // Failed, or stop() was called.  timeout_ms == 0 waits indefinitely (still
    // interruptible by stop()).  Returns true only when all workers are Ready;
    // Failed / timeout / stop return false.  Safe to call after wait(): it then
    // simply reports the current state.
    bool wait_ready(unsigned timeout_ms = 0);
    // WORKER readiness only: every DSP worker published Ready and none
    // published Failed.  This deliberately says nothing about the stream
    // source -- a run whose start_stream() failed can still report
    // ready_ok() == true.  Combine it with source_started()/source_failed()
    // (below) when deciding whether a run actually succeeded.
    bool ready_ok() const;
    // Overall failure state: any worker Failed, OR the stream source failed
    // (start_stream() returned false, or the RX thread recorded a fatal
    // source error).  Read it after wait().
    bool failed() const;
    // Publish-ordered snapshot of every worker.  Every state and every
    // affinity_* field is written before that worker becomes Ready/Failed, so
    // reading it after wait_ready() / wait() needs no further synchronisation.
    const std::vector<WorkerReadyInfo>& worker_ready_info() const { return worker_ready_; }
    // First worker failure reason; when no worker failed this falls back to
    // source_error().  Empty == no failure at all.
    std::string init_error() const;

    // ------------------------------------------------------------------
    // Source lifecycle (round-2 review R2).
    //
    // start_stream() runs on the RX thread only after the worker-ready
    // barrier, so "the source never started" and "the source started and
    // then failed" are different states and must not be conflated:
    //   source_started() == false && source_failed()  -> start failed
    //   source_started() == true  && source_failed()  -> fatal RX error
    // The two booleans are acquire/release atomics; source_error() takes the
    // internal mutex, so it is safe to call while the RX thread runs.  The
    // canonical read point is after wait(), once the RX thread has joined.
    // ------------------------------------------------------------------
    bool source_started() const;      // start_stream() returned true
    bool source_failed() const;       // start_stream() failed, or a fatal RX error
    std::string source_error() const; // empty unless source_failed()

    // Experiment string applied by worker w (empty before start / unknown worker).
    //
    // Reports the canonical PipelineConfig::resampler_experiment value that the
    // worker actually pushed into its own resampler core ("none" when it has no
    // resampler core, e.g. when native_rate == work_rate, and "none" when the
    // configured value was rejected).  Written by the worker before it publishes
    // Ready, so it is stable once wait_ready() / wait() returned.
    std::string applied_resampler_experiment(int worker_id) const;

    // ------------------------------------------------------------------
    // Diagnostics (frozen contract §3).  Counters are written by the RX and
    // worker threads; read them after wait() only, and call
    // reset_diagnostics() either before start() or after wait().
    // ------------------------------------------------------------------
    const RxDiagnostics& rx_diagnostics() const;
    const CommitDiagnostics& commit_diagnostics() const;
    void reset_diagnostics();

    // Number of per-window IQ dump files actually written by this pipeline
    // (0 when PipelineConfig::iq_dump_dir / iq_dump_windows is empty).  Reset by
    // reset_diagnostics().  Read it after wait().
    uint64_t iq_dumps_written() const
    {
        return iq_dump_written_.load(std::memory_order_relaxed);
    }

    void set_result_callback(ResultCallback cb) { result_cb_ = std::move(cb); }

    // Non-owning access to the fixed-tone cancellation stage.  nullptr when
    // cfg.fixed_tone_cancel_enable is false (the plain source is used) or
    // before initialize() wrapped the source.  Read the stage's stats after
    // wait() only.
    FixedToneCancelSc16Source* fixed_tone_source() const { return fixed_tone_source_; }

    const PipelineLedger& ledger() const { return ledger_; }
    const std::vector<WindowResult>& results() const { return results_; }

private:
    void rx_producer_loop();
    // Each DSP worker runs this loop over its own private ring pool.
    void dsp_consumer_loop(int worker_id);
    // Effective worker count actually instantiated (>= 1).
    int effective_dsp_workers() const;
    // In-order commit of a finished window result: buffers out-of-order
    // completions and flushes them strictly by window_id.
    void commit_result(int worker_id, const WindowResult& res);

    // --- ready barrier helpers (all *locked variants need ready_mutex_) ---
    bool all_ready_locked() const;
    bool any_failed_locked() const;
    // RX-side block until the barrier resolves; false == abort acquisition.
    bool wait_for_workers_ready();
    void publish_worker_state(int worker_id, WorkerReadyState state, const std::string& err);
    // Post-join safety net: settle any slot still sitting in a ring pool so
    // every admitted window reaches a terminal state.
    void recover_orphaned_windows();
    // Diagnostic per-window CF32 dump (worker thread, never in the DSP timers).
    bool write_iq_dump(uint64_t window_id,
                       const std::complex<float>* data,
                       size_t n) const;

    PipelineConfig cfg_;
    std::unique_ptr<ISc16StreamSource> source_;
    // Non-owning: points at source_ when the fixed-tone stage wrapped it.
    FixedToneCancelSc16Source* fixed_tone_source_ = nullptr;
    // One lock-free SPSC ring pool per DSP worker.  The RX producer thread is
    // the only producer and never touches a pool concurrently with itself, so
    // each pool stays a strict 1P/1C SPSC structure.
    std::vector<std::unique_ptr<Sc16WindowRingPool>> pools_;

    std::atomic<bool> running_{false};
    std::atomic<bool> stop_requested_{false};
    std::atomic<bool> rx_finished_{false};

    // --- source lifecycle (round-2 review R2) ------------------------------
    // Source-only state, kept out of the worker-ready ledger on purpose: the
    // ready barrier describes DSP workers, and conflating the two would make
    // ready_ok() report a source failure as "workers not ready".
    // The RX thread is the only writer (message under source_error_mutex_,
    // then source_failed_ with release); the caller reads after wait().
    // Both flags and the message are reset by start() (new run epoch) and
    // after a successful source_->open() in initialize().
    std::atomic<bool> source_started_{ false }; // start_stream() returned true
    std::atomic<bool> source_failed_{ false };  // start failure or fatal RX error
    mutable std::mutex source_error_mutex_;
    std::string source_error_; // first fatal source message, empty == healthy
    // Records a fatal source error and raises source_failed_.  The first
    // message wins so a later symptom cannot hide the root cause.
    void set_source_error(const std::string& msg);

    // --- ready barrier (A2) ------------------------------------------------
    // Guards worker_ready_ and the ready-epoch handshake with stop().  Workers
    // publish under it and notify; wait_ready() / the RX thread wait on it.
    // Never held while running DSP work.
    mutable std::mutex ready_mutex_;
    std::condition_variable ready_cv_;
    std::vector<WorkerReadyInfo> worker_ready_;
    // Per-worker record of the resampler experiment actually applied, guarded
    // by ready_mutex_ (written by the worker before it publishes Ready, read
    // only after wait()); empty entries mean "not started yet".
    std::vector<std::string> applied_resampler_experiment_;

    // --- Phase 0 diagnostics accumulators ----------------------------------
    // Relaxed atomics: the RX counters have a single writer (the RX thread),
    // the commit counters are shared by all DSP workers.  Read them after
    // wait() only.  Deliberately NOT under ledger_mutex_, so instrumentation can
    // never extend ledger lock hold time.
    struct RxDiagAcc {
        std::atomic<uint64_t> read_calls{ 0 };
        std::atomic<uint64_t> samples_read{ 0 };
        std::atomic<uint64_t> bytes_read{ 0 };
        std::atomic<uint64_t> windows_published{ 0 };
        std::atomic<uint64_t> windows_dropped{ 0 };
        std::atomic<uint64_t> slot_copy_bytes{ 0 };
        std::atomic<uint64_t> stream_buf_resize_count{ 0 };
        std::atomic<uint64_t> stream_buf_zero_init_bytes{ 0 };
        std::atomic<uint64_t> compact_move_bytes{ 0 };
        std::atomic<uint64_t> max_buffered_samples{ 0 };
        std::atomic<uint64_t> read_ns{ 0 };
        std::atomic<uint64_t> assemble_ns{ 0 };
        std::atomic<uint64_t> slot_copy_ns{ 0 };
        std::atomic<uint64_t> dispatch_ns{ 0 };
        std::atomic<uint64_t> ledger_wait_ns{ 0 };
    };

    struct CommitDiagAcc {
        std::atomic<uint64_t> commit_calls{ 0 };
        std::atomic<uint64_t> results_emitted{ 0 };
        std::atomic<uint64_t> max_out_of_order_span{ 0 };
        std::atomic<uint64_t> max_pending_results{ 0 };
        std::atomic<uint64_t> reorder_backpressure_waits{ 0 };
        std::atomic<uint64_t> reorder_overflow_drops{ 0 };
        std::atomic<uint64_t> lock_wait_ns{ 0 };
        std::atomic<uint64_t> reorder_ns{ 0 };
        std::atomic<uint64_t> callback_ns{ 0 };
    };

    RxDiagAcc rx_acc_{};
    CommitDiagAcc commit_acc_{};
    // Diagnostic IQ dump bookkeeping (worker threads, relaxed atomics).
    //   iq_dump_started_ : eligible windows that consumed a dump slot
    //   iq_dump_written_ : files actually written (exposed to the runner)
    std::atomic<uint64_t> iq_dump_started_{ 0 };
    std::atomic<uint64_t> iq_dump_written_{ 0 };
    // Snapshots handed out by rx_diagnostics() / commit_diagnostics().  They
    // are refilled on every call; the frozen contract only allows reading them
    // after wait(), so no extra locking is required.
    mutable RxDiagnostics rx_diag_view_{};
    mutable CommitDiagnostics commit_diag_view_{};

    std::thread rx_thread_;
    std::vector<std::thread> dsp_threads_;

    PipelineLedger ledger_;
    std::vector<WindowResult> results_;
    ResultCallback result_cb_;
    // Guards multi-threaded ledger mutations (DSP workers run concurrently).
    std::mutex ledger_mutex_;
    // Guards the reorder buffer, results_ and single-publisher handoff so that
    // external observers always see strictly increasing window_id order while
    // result_cb_ runs outside commit_mutex_.
    std::mutex commit_mutex_;
    std::condition_variable commit_cv_;
    std::vector<std::unique_ptr<WindowResult>> pending_results_;
    uint64_t next_commit_id_ = 0;
    size_t buffered_result_count_ = 0;
    bool publish_in_progress_ = false;
    void emit_batch_unlocked(
        std::vector<std::unique_ptr<WindowResult>>& batch,
        bool diag,
        std::atomic<uint64_t>* lock_wait_ns);
    // Drain any still-buffered results after all threads have joined.
    void flush_pending_results();
};

} // namespace pipeline
} // namespace openuwb
