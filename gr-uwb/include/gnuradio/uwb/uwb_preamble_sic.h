/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * UwbPreambleSic — thin GNU Radio PDU adapter for the S0 preamble-only SIC
 * math core (gr::uwb::sic, uwb_preamble_sic_core.h).  Frozen contract:
 * docs/sic/S0_sic_adapter_api.md.
 *
 * ============================ Scheduler semantics ==========================
 * Block type: zero-stream gr::block (0 input / 0 output stream ports), message
 * PDUs only.  There is NO forecast() and NO general_work().
 *
 *   - The `in` message handler runs on the GNU Radio block thread.  It only
 *     validates the PDU, takes an immutable shared reference to the input IQ
 *     c32vector, applies admission control, and enqueues a FIFO entry.  It
 *     performs no SIC math.
 *   - A SINGLE worker thread (owned by this block, started in start()) calls
 *     sic_one() for each admitted window.  Because exactly one thread
 *     publishes `out`, publication order is the offered order.
 *   - Scratch (SicScratch) is prepared once with prepare_sic_core(); the
 *     output buffers form a pool sized either from `max_window_samples` at
 *     construction (>0) or from the first window (==0).  A later window larger
 *     than the pool triggers AT MOST ONE counted grow (`window_grow_count`);
 *     after that a larger window is fail-closed as `pool_capacity_exceeded`
 *     rather than growing per window.  The steady-state hot path allocates
 *     nothing and does no file I/O.  PMT meta/vector construction DOES
 *     allocate; that cost is charged to the `output_wrap_publish_us` adapter
 *     timer and never to the core.
 *   - stop() drains every queued window to a terminal state and joins the
 *     worker; no thread is left behind.
 *
 * ================================== Ports ==================================
 *   in    : pmt::cons(meta, c32vector).  meta carries the SENSING description
 *           (code_index / sync_repetitions / sfd_mode / pulse_id / window ids).
 *           Those fields ALWAYS describe sensing and are copied through
 *           untouched; the comm profile is block configuration only.
 *           Optional `sensing_roi_lo` / `sensing_roi_hi` (int, absolute window
 *           samples) declare the SENSING processing ROI = the subtraction /
 *           commit region.  It is kept SEPARATE from the wide search ROI in the
 *           block config: the core only commits when the detected comm support
 *           actually OVERLAPS this region (a real comm entirely outside it is
 *           bypassed, out == y).  Optional `sensing_min_overlap` (uint) sets the
 *           minimum required overlap in samples (default 0 => require > 0).  If
 *           the keys are absent the gate is disabled (subtraction = support ∩
 *           search ROI, the legacy behaviour).
 *   truth : ORACLE-ONLY dict (see below).  Registered/read only when
 *           mode == "oracle"; in estimate/off the port does not exist and no
 *           truth object is stored on the block.
 *   out   : pmt::cons(meta', c32vector) — exactly one per accepted window, in
 *           order.  meta' = input meta + the SIC result keys below.  On every
 *           bypass path the IQ is the ORIGINAL c32vector (bit-identical).
 *   status: small dict per window (state, reason, counters, adapter timers).
 *   i_hat : pmt::cons(meta', c32vector) — reconstructed interference, emitted
 *           ONLY when publish_reconstruction=true (diagnostic NMSE vs truth).
 *
 * ============================== Admission ==================================
 * in_flight = admitted windows not yet terminated.  Admit only if
 *   in_flight < in_flight_max
 *   AND pending_bytes + window_bytes <= iq_budget_bytes
 *   AND queued_admitted < queue_capacity
 * Otherwise the window is BYPASSED IN ORDER (FIFO) with the original IQ
 * unchanged and counted as missed_admission; a status event records the
 * reason.  The pending-bypass FIFO is bounded by `pending_bypass_max`
 * (default in_flight_max + queue_capacity): once it is at the cap, admission
 * stops and a rejected window is terminated as `queue_overflow` (recorded via
 * the counter and a status event, never a silent drop) rather than parked.
 * Upstream is expected to honor the in-flight credit so that
 * offered == admitted + missed_admission holds under normal operation.
 *
 * ===================== Per-window `out` meta keys ==========================
 * (in addition to the input meta)
 *   sic_state, sic_detected, sic_committed, sic_partial, sic_ambiguous,
 *   sic_bypass_reason, sic_uses_truth,
 *   sic_detected_start, sic_cfo_hz, sic_n_paths, sic_support_lo,
 *   sic_support_hi, sic_sub_overlap, sic_residual_corr,
 *   handler_us, queue_wait_us, core_total_us, output_wrap_publish_us,
 *   completion_latency_us,
 *   sic_detect_coarse_us, sic_detect_fine_us, sic_cfo_us, sic_cir_us,
 *   sic_path_select_us, sic_fractional_refine_us, sic_matrix_build_us,
 *   sic_gain_fit_us, sic_reconstruct_us, sic_validate_commit_us
 * Additive adapter diagnostics: sic_missed_admission, sic_pool_capacity_exceeded.
 * `output_wrap_publish_us` / `completion_latency_us` in the out meta are
 * measured through PMT construction; the pub-inclusive mirrors are on `status`
 * (a message cannot carry the duration of its own enqueue).
 *
 * ================= S1 frozen 8-SYNC model metadata (additive) ===============
 * Every `out` meta also carries the frozen-model publication produced by the S1
 * core (`estimate_model` + `apply_model`, composed exactly like `sic_one`):
 *   sic_model_mode            "estimate8_frozen" | "confirmed_legacy"
 *   sic_model_id              FNV-1a-64 of the frozen model fields
 *   sic_model_fingerprint     lower-case hex ("0x%016llx") of sic_model_id
 *   sic_observed_end_sample   DECLARED estimation visibility limit (0 = whole
 *                             window); see sic_observed_end_source
 *   sic_observed_end_source   "observed_end_sample" | "visible_end_sample" |
 *                             "whole_window_default"
 *   sic_visible_end_sample    RESOLVED visibility limit actually used by the
 *                             core (== window length for whole-window mode)
 *   sic_training_lo/hi        training span the fit actually read
 *   sic_estimation_ready_sample  last sample the estimator required (== hi)
 *   sic_support_source        "configured_nominal_prediction" | ... | "none"
 *   sic_support_coverage      (support_hi-support_lo)/(sync_reps*sps+post_tail)
 *   sic_sub_lo/sub_hi         permitted cancellation region [lo, hi)
 *   sic_phase_reference       CFO phase anchor (== origin)
 *   sic_fit_residual          normalized training residual
 *   sic_cond2                 cond2(A) of the final gain-fit design
 *   sic_apply_refit_count     ALWAYS 0 (apply never re-fits); asserted by QA
 *   sic_apply_param_deltas    ALWAYS 0 (no param update in apply); asserted
 *   sic_nominal_sync_reps / sic_samples_per_symbol / sic_sample_rate
 * The essentials are mirrored on the `status` event dict.
 *
 * ==================== S1 per-path frozen-model tables (additive) ============
 * The frozen model's per-path tables are ALWAYS present on `out` (and on the
 * diagnostic `i_hat` meta), so an absent key never has to be guessed at:
 *   sic_tau        f64vector, length n_paths: ABSOLUTE work-sample delays
 *                  (SicModel::tau_abs); index 0..n_paths-1 follows the model's
 *                  own (magnitude-sorted) path selection order
 *   sic_gain_re    f64vector, length n_paths: real part of the complex gains
 *   sic_gain_im    f64vector, length n_paths: imag part of the complex gains
 *                  (SAME index order as sic_tau: gain[k] belongs to tau[k])
 *   sic_path_count size_t: number of published per-path entries
 *                  (== sic_tau.size()); on a committed window it also equals
 *                  sic_n_paths, the core's detected selection count
 *   sic_gain_cond  double: cond2(A) of the final gain-fit design
 *                  (SicModel::gain_condition; == sic_cond2 when a model exists)
 * The values come from the SicModel returned by estimate_model (authoritative),
 * not from SicResult::taus.  On every window where no frozen model was APPLIED
 * (off/oracle, bypass, failure) the three vectors are published PRESENT but
 * EMPTY (length 0) and sic_path_count is 0, so "no paths" is distinguishable
 * from "not published".  sic_n_paths keeps its earlier meaning (paths selected
 * by detection) and is therefore only equal to sic_path_count when a model was
 * frozen and committed.  sic_path_count / sic_gain_cond are mirrored on
 * `status` (together with sic_n_paths).
 *
 * ===================== observed_end_sample plumbing ========================
 * The per-window estimation visibility limit is taken FROM THE PDU META:
 * `observed_end_sample` (preferred) or `visible_end_sample`, both int64.  When
 * neither key is present the adapter passes 0 = "the whole window is visible"
 * and records sic_observed_end_sample=0 with
 * sic_observed_end_source="whole_window_default" (the legacy-visible contract).
 * A declared positive prefix is passed through unchanged and is NEVER widened to
 * the whole window.  Estimation still only ever reads that prefix; `apply`
 * always cancels over the whole window (S1 API doc §2/§6.2).
 *
 * =========================== Oracle truth dict =============================
 * The `truth` port exists ONLY when mode == "oracle".  In estimate/off it is
 * not registered and no truth object is stored on the block.  Posted keyed by
 * `window_id` (post before the window):
 *   window_id (int), start_sample (double), cfo_hz (double, optional),
 *   n_paths (int, optional; default = length of tau),
 *   tau (f64vector), gain_re (f64vector), gain_im (f64vector)
 * It is consumed only for a matching window_id.  Estimate mode never touches
 * it, so `sic_uses_truth` stays false.
 */

#ifndef INCLUDED_GNURADIO_UWB_UWB_PREAMBLE_SIC_H
#define INCLUDED_GNURADIO_UWB_UWB_PREAMBLE_SIC_H

#include <gnuradio/block.h>
#include <gnuradio/gr_complex.h>
#include <gnuradio/uwb/api.h>
#include <gnuradio/uwb/uwb_preamble_sic_core.h>
#include <pmt/pmt.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace gr {
namespace uwb {

class UWB_API UwbPreambleSic : public gr::block
{
public:
    using sptr = std::shared_ptr<UwbPreambleSic>;

    /**
     * \param sync_template_path     CF32 file: ONE shaped comm SYNC waveform
     *                               (length == samples_per_symbol).
     * \param preamble_template_path CF32 file: the FULL comm preamble waveform.
     * \param comm_code              comm preamble code (block config; it never
     *                               overwrites the sensing fields in the PDU).
     * \param sync_reps              nominal comm SYNC repetitions.
     * \param samples_per_symbol     work samples per SYNC symbol (1016).
     * \param sample_rate            work sample rate (998.4e6).
     * \param k_paths                1/2/4/8/16 interference paths.
     * \param fit_reps               estimation window in SYNC symbols (8/16/32).
     * \param validation_reps        disjoint holdout symbols (>= 8).
     * \param roi_lo, roi_hi         public candidate search ROI [lo, hi).
     * \param coarse_stride          detection coarse stride (samples).
     * \param mode                   "off" | "estimate" | "oracle".
     * \param detect_threshold       fixed on the calibration set.
     * \param validate_threshold     fixed on the calibration set.
     * \param in_flight_max          admission cap on un-terminated windows.
     * \param iq_budget_bytes        admission cap on pending IQ bytes.
     * \param queue_capacity         admission cap on queued work items.
     * \param publish_reconstruction emit the diagnostic i_hat port.
     * \param enable_fractional      core fractional-delay refine.
     * \param enable_cfo             core CFO estimation.
     * \param path_min_spacing       core path min spacing (0 = autocorr).
     * \param post_tail              core defined reconstruction tail.
     * \param max_window_samples     pre-size the output/scratch pool (0 = size
     *                               from the first window).
     * \param pending_bypass_max     bound on the pending-bypass FIFO
     *                               (0 = in_flight_max + queue_capacity).
     * \param support_mode           S1 cancellation-support mode:
     *                               "configured_nominal" (default, the frozen
     *                               8-SYNC estimate cancels the whole nominal
     *                               preamble) or "confirmed_legacy" (explicit
     *                               S0-comparison support + holdout gate).
     * \param online_validation_reps 0 (default) = cancel as soon as the 8-SYNC
     *                               model is frozen; > 0 = explicit conservative
     *                               holdout wait (must be <= validation_reps).
     */
    static sptr make(const std::string& sync_template_path,
                     const std::string& preamble_template_path,
                     size_t comm_code,
                     size_t sync_reps,
                     size_t samples_per_symbol,
                     double sample_rate,
                     size_t k_paths,
                     size_t fit_reps,
                     size_t validation_reps,
                     int64_t roi_lo,
                     int64_t roi_hi,
                     size_t coarse_stride,
                     const std::string& mode,
                     double detect_threshold,
                     double validate_threshold,
                     size_t in_flight_max = 8,
                     size_t iq_budget_bytes = (size_t(64) << 20),
                     size_t queue_capacity = 8,
                     bool publish_reconstruction = false,
                     bool enable_fractional = true,
                     bool enable_cfo = true,
                     size_t path_min_spacing = 0,
                     size_t post_tail = 100,
                     size_t max_window_samples = 0,
                     size_t pending_bypass_max = 0,
                     const std::string& support_mode = "configured_nominal",
                     size_t online_validation_reps = 0);

    /**
     * Python / in-memory path: same as make() but the two comm templates are
     * supplied directly as CF32 waveforms (std::vector<gr_complex>).
     */
    static sptr make_from_templates(const std::vector<gr_complex>& sync,
                                    const std::vector<gr_complex>& preamble,
                                    size_t comm_code,
                                    size_t sync_reps,
                                    size_t samples_per_symbol,
                                    double sample_rate,
                                    size_t k_paths,
                                    size_t fit_reps,
                                    size_t validation_reps,
                                    int64_t roi_lo,
                                    int64_t roi_hi,
                                    size_t coarse_stride,
                                    const std::string& mode,
                                    double detect_threshold,
                                    double validate_threshold,
                                    size_t in_flight_max = 8,
                                    size_t iq_budget_bytes = (size_t(64) << 20),
                                    size_t queue_capacity = 8,
                                    bool publish_reconstruction = false,
                                    bool enable_fractional = true,
                                    bool enable_cfo = true,
                                    size_t path_min_spacing = 0,
                                    size_t post_tail = 100,
                                    size_t max_window_samples = 0,
                                    size_t pending_bypass_max = 0,
                                    const std::string& support_mode =
                                        "configured_nominal",
                                    size_t online_validation_reps = 0);

    ~UwbPreambleSic() override;

    // ---- counters / lifecycle -------------------------------------------
    uint64_t windows_offered() const
    {
        return d_offered_.load(std::memory_order_relaxed);
    }
    uint64_t windows_admitted() const
    {
        return d_admitted_.load(std::memory_order_relaxed);
    }
    uint64_t windows_completed() const
    {
        return d_completed_.load(std::memory_order_relaxed);
    }
    uint64_t windows_failed() const
    {
        return d_failed_.load(std::memory_order_relaxed);
    }
    uint64_t windows_bypassed() const
    {
        return d_bypassed_.load(std::memory_order_relaxed);
    }
    uint64_t windows_dropped() const
    {
        return d_dropped_.load(std::memory_order_relaxed);
    }
    uint64_t windows_invalid() const
    {
        return d_invalid_.load(std::memory_order_relaxed);
    }
    uint64_t windows_missed_admission() const
    {
        return d_missed_admission_.load(std::memory_order_relaxed);
    }
    uint64_t windows_queue_overflow() const
    {
        return d_queue_overflow_.load(std::memory_order_relaxed);
    }
    uint64_t windows_pool_capacity_exceeded() const
    {
        return d_pool_capacity_exceeded_.load(std::memory_order_relaxed);
    }
    uint64_t window_grow_count() const
    {
        return d_window_grow_count_.load(std::memory_order_relaxed);
    }
    size_t inflight_high_watermark() const
    {
        std::lock_guard<std::mutex> lock(d_queue_mutex_);
        return d_inflight_hwm_;
    }
    size_t queue_high_watermark() const
    {
        std::lock_guard<std::mutex> lock(d_queue_mutex_);
        return d_queue_hwm_;
    }
    size_t bypass_fifo_high_watermark() const
    {
        std::lock_guard<std::mutex> lock(d_queue_mutex_);
        return d_bypass_fifo_hwm_;
    }
    size_t queue_depth() const;

    //! True when the queue is empty and the worker is idle (all terminal).
    bool drained() const;
    //! Block until every queued window reaches a terminal state.
    void drain();

    // ---- frozen configuration introspection ------------------------------
    const std::string& mode() const { return d_mode_name_; }
    size_t in_flight_max() const { return d_in_flight_max_; }
    size_t iq_budget_bytes() const { return d_iq_budget_bytes_; }
    size_t queue_capacity() const { return d_queue_capacity_; }
    bool publish_reconstruction() const { return d_publish_reconstruction_; }
    bool enable_fractional() const { return d_cfg_.enable_fractional; }
    bool enable_cfo() const { return d_cfg_.enable_cfo; }
    size_t path_min_spacing() const { return d_cfg_.path_min_spacing; }
    size_t post_tail() const { return d_cfg_.post_tail; }
    size_t max_window_samples() const { return d_max_window_samples_; }
    size_t pending_bypass_max() const { return d_pending_bypass_max_; }
    //! True only in oracle mode (the truth port exists only then).
    bool truth_port_enabled() const { return d_truth_oracle_; }
    //! Configured S1 support mode ("configured_nominal" | "confirmed_legacy").
    const std::string& support_mode() const { return d_support_mode_name_; }
    //! Published frozen-model mode ("estimate8_frozen" | "confirmed_legacy").
    const std::string& model_mode() const { return d_model_mode_name_; }
    //! Explicit conservative holdout repetitions (0 = cancel with no holdout).
    size_t online_validation_reps() const
    {
        return d_cfg_.online_validation_reps;
    }

protected:
    UwbPreambleSic(const std::vector<gr_complex>& sync,
                   const std::vector<gr_complex>& preamble,
                   const std::string& mode_name,
                   size_t comm_code,
                   size_t sync_reps,
                   size_t samples_per_symbol,
                   double sample_rate,
                   size_t k_paths,
                   size_t fit_reps,
                   size_t validation_reps,
                   int64_t roi_lo,
                   int64_t roi_hi,
                   size_t coarse_stride,
                   double detect_threshold,
                   double validate_threshold,
                   size_t in_flight_max,
                   size_t iq_budget_bytes,
                   size_t queue_capacity,
                   bool publish_reconstruction,
                   bool enable_fractional,
                   bool enable_cfo,
                   size_t path_min_spacing,
                   size_t post_tail,
                   size_t max_window_samples,
                   size_t pending_bypass_max,
                   const std::string& support_mode,
                   size_t online_validation_reps);

    bool start() override;
    bool stop() override;

private:
    // One offered window, kept in FIFO order.  `iq_in` is an immutable shared
    // reference to the producer's c32vector; bypass publications reuse it so
    // they are bit-identical by construction.
    struct Pending {
        pmt::pmt_t meta_in;      // input meta dict (immutable)
        pmt::pmt_t iq_in;        // c32vector (immutable)
        size_t n = 0;            // samples
        size_t bytes = 0;        // n * sizeof(gr_complex)
        bool admitted = false;   // false => admission reject (bypass in order)
        const char* bypass_reason = nullptr;
        uint64_t handler_us = 0;
        std::chrono::steady_clock::time_point offered_at;  // latency origin
        std::chrono::steady_clock::time_point enqueued_at;
    };

    void handle_in(pmt::pmt_t msg);
    void handle_truth(pmt::pmt_t msg);
    void worker_loop();
    void process(Pending& p);

    bool ensure_pool(size_t n);
    void publish_status(const std::string& event,
                        pmt::pmt_t extra = pmt::PMT_NIL);

    static std::vector<gr_complex> load_cf32_file(const std::string& path);
    static const char* state_name(sic::SicState s);
    static bool parse_mode(const std::string& mode, sic::SicMode& out);

    // Immutable after construction (workers only read).
    std::vector<gr_complex> d_sync_;
    std::vector<gr_complex> d_preamble_;
    sic::SicConfig d_cfg_;
    sic::SicScratch d_scratch_;
    std::string d_mode_name_;
    //! Configured S1 support-mode label (ctor argument, verbatim).
    std::string d_support_mode_name_ = "configured_nominal";
    //! Canonical model label published as sic_model_mode.
    std::string d_model_mode_name_ = "estimate8_frozen";
    size_t d_in_flight_max_ = 8;
    size_t d_iq_budget_bytes_ = (size_t(64) << 20);
    size_t d_queue_capacity_ = 8;
    bool d_publish_reconstruction_ = false;
    bool d_truth_oracle_ = false;       // truth port exists only in oracle mode
    size_t d_max_window_samples_ = 0;   // 0 => size from the first window
    size_t d_pending_bypass_max_ = 0;   // resolved bound on the bypass FIFO

    // Worker-owned output pool (single worker): reused across windows.
    std::vector<gr_complex> d_out_;
    std::vector<gr_complex> d_ihat_;

    // Ordered work FIFO + admission state.  All guarded by d_queue_mutex_.
    mutable std::mutex d_queue_mutex_;
    std::condition_variable d_queue_cv_;
    std::deque<Pending> d_queue_;
    bool d_worker_idle_ = true;
    bool d_stop_ = false;
    std::thread d_worker_;
    size_t d_queue_admitted_ = 0;   // admitted items still queued
    size_t d_in_flight_ = 0;        // admitted, not yet terminated
    uint64_t d_pending_bytes_ = 0;  // bytes of admitted, not yet terminated
    size_t d_pending_bypass_ = 0;   // bypass entries parked in the FIFO
    size_t d_queue_depth_ = 0;
    size_t d_inflight_hwm_ = 0;
    size_t d_queue_hwm_ = 0;
    size_t d_bypass_fifo_hwm_ = 0;
    size_t d_pool_capacity_ = 0;    // current output-pool capacity (samples)

    // Oracle truth sidecar (window_id -> truth).  Used ONLY in oracle mode.
    mutable std::mutex d_truth_mutex_;
    std::unordered_map<uint64_t, sic::SicTruth> d_truth_;

    // Counters (atomic snapshots).
    std::atomic<uint64_t> d_offered_{ 0 };
    std::atomic<uint64_t> d_admitted_{ 0 };
    std::atomic<uint64_t> d_completed_{ 0 };
    std::atomic<uint64_t> d_failed_{ 0 };
    std::atomic<uint64_t> d_bypassed_{ 0 };
    std::atomic<uint64_t> d_dropped_{ 0 };
    std::atomic<uint64_t> d_invalid_{ 0 };
    std::atomic<uint64_t> d_missed_admission_{ 0 };
    std::atomic<uint64_t> d_queue_overflow_{ 0 };
    std::atomic<uint64_t> d_pool_capacity_exceeded_{ 0 };
    std::atomic<uint64_t> d_window_grow_count_{ 0 };
};

} // namespace uwb
} // namespace gr

#endif /* INCLUDED_GNURADIO_UWB_UWB_PREAMBLE_SIC_H */
