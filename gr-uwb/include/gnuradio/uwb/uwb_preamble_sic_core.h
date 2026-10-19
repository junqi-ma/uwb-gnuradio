/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * S0/S1 preamble-only SIC math core (S1 contract:
 * docs/sic/S1_八符号模型API.md; S0 contract: docs/sic/S0_sic_core_api.md).
 * Namespace gr::uwb::sic.
 *
 * This is the PURE math core of the detection-triggered preamble SIC branch.
 * It has NO GNU Radio types, NO scheduler, NO file I/O and does NOT allocate
 * or grow a vector inside sic_one() / estimate_model() / apply_model(): every
 * buffer is sized once by prepare_sic_core().  The PDU adapter
 * (uwb_preamble_sic.*) is a separate, thinner layer.
 *
 * S1 pipeline (contract §Pipeline):
 *   estimate_model: detect_coarse / detect_fine  -> detect_threshold gate
 *                   repetition confirmation      -> training-span confirmations
 *                   cfo (training reps)          -> rotate the reconstruction basis
 *                   cir / path_select            -> <= K distinct integer delays
 *                   fractional_refine (optional) -> polyphase-table bandlimited delay
 *                   matrix_build / gain_fit      -> complex LS via Householder QR
 *                   freeze SicModel              -> immutable + model_id
 *                   commit gate                  -> fit_residual (default) and/or
 *                                                   the explicit online holdout
 *   apply_model:    reconstruct on a fixed ABSOLUTE block grid, subtract only
 *                   inside the model's permitted region, bit-identical outside.
 *   sic_one:        estimate over min(n, observed_end) (0 => n), then apply the
 *                   frozen model to the whole [0,n) window (backfill + forward).
 *
 * STRICT PREFIX: estimation reads only y[0, min(observed_end, n)); every
 * correlation row / LS row / CIR tap / CFO pair lies fully inside it (see the
 * S1 API doc §2.1).  The injected comm start is never an input.
 *
 * Support: ConfiguredNominal PREDICTS origin + sync_reps*sps + post_tail from the
 * public profile and is NOT clamped to the confirmed repetition count;
 * ConfirmedLegacy reproduces S0.  Only the frozen model may cancel, and only
 * inside model.[sub_lo, sub_hi): out == y byte-for-byte everywhere else, and
 * EVERY failure / bypass path leaves out bit-identical to y.
 *
 * Output convention:
 *   i_hat[n] = exp(j*w*(n - phase_reference_sample))
 *              * sum_k gain[k] * preamble(n - tau_abs[k])
 *   out[n]   = y[n] - i_hat[n]   (inside the permitted region only)
 * i_hat is the reconstruction over the model support; it is NOT confined to the
 * subtraction mask.
 *
 * TRUTH ISOLATION: Estimate mode reads only y, the comm templates and the
 * config.  It never receives injected delay/CFO/paths/ISR/interferer IQ.
 * Oracle mode consumes an explicit SicTruth and sets uses_truth=true; it is a
 * diagnostic upper bound and is never valid for effect/real-time acceptance.
 */

#pragma once

#include <gnuradio/uwb/api.h>

#include <complex>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace gr {
namespace uwb {
namespace sic {

// ---------------------------------------------------------------------------
// Modes and states (contract §Config, §Result)
// ---------------------------------------------------------------------------
enum class SicMode : uint8_t { Off = 0, Estimate = 1, Oracle = 2 };

enum class SicState : uint8_t {
    Bypassed = 0,
    Committed = 1,
    Failed = 2,
    Ambiguous = 3
};

// S1: how the CANCELLATION support is derived.  This is deliberately decoupled
// from how many repetitions were confirmed / validated (S1-01), and it never
// re-fits anything: it only decides WHICH samples the frozen model is allowed to
// modify.
//
//   ConfiguredNominal (default) : the support is PREDICTED from the public
//       profile -- detected origin + configured `sync_reps * samples_per_symbol`
//       + template/post tail -- and is NOT clamped to the confirmed/validated
//       repetition count.  The estimate itself still uses only the first
//       `fit_reps` SYNC symbols (8 by default) of the VISIBLE PREFIX, so the
//       full nominal preamble can be cancelled while the parameters remain
//       strictly 8-symbol.  This is the S1 "estimate 8 symbols, then subtract
//       the whole known preamble" path.
//   ConfirmedLegacy (1)         : S0 behaviour kept as an EXPLICIT legacy
//       comparison: support = confirmed repetitions (counted over
//       `fit_reps + online_validation_reps`) * samples_per_symbol + post_tail,
//       and the commit gate is the old disjoint-holdout residual test.  It is
//       NOT the default.  Strict-prefix estimation still applies to it (CFO is
//       training-only), so it is bit-identical to S0 only when the whole window
//       is visible and the estimator's changed stages are not exercised.
enum class SicSupportMode : uint8_t { ConfiguredNominal = 0, ConfirmedLegacy = 1 };

// Maximum number of interference paths the core will ever fit.  The config
// permits k_paths in {1,2,4,8,16}; larger values are rejected by prepare.
constexpr size_t kSicMaxPaths = 16;

// Round-trip notes: no gnuradio headers, no Eigen.  Detection is
// VOLK-vectorized (volk_32fc_x2_conjugate_dot_prod_32fc_u for the template
// correlation); the segment self-energy is a precomputed double-precision
// prefix sum of |y|^2 (one indexed difference per lag) rather than a second
// per-lag VOLK dot.  The reconstruction combine is VOLK 32fc
// (volk_64f_convert_32f to narrow the double accumulation, then
// volk_32fc_x2_multiply_32fc with a block-decomposed float32 CFO ramp).  The
// final gain fit solves the M x K least squares by Householder QR of A (NO
// normal equations, NO explicit inverse) and reports cond(A) computed from the
// singular values of the K x K factor R (A = QR, Q orthogonal => cond2(A) ==
// cond2(R) exactly) via a self-contained one-sided complex Jacobi SVD.  The old
// max|R_kk|/min|R_kk| diagonal ratio was NOT cond(A) and has been removed; see
// `SicResult::gain_condition`.  VOLK is already a link dependency of
// gnuradio-uwb, so this adds no new wiring.  The fractional-refine seed solve
// uses the same Householder QR in the .cc.
struct SicConfig {
    SicMode mode = SicMode::Off;
    size_t comm_code = 10;
    size_t sync_reps = 64;            // nominal; the true start is unknown
    size_t samples_per_symbol = 1016;
    double sample_rate = 998.4e6;
    size_t k_paths = 4;               // 1,2,4,8,16
    size_t fit_reps = 8;              // estimation window in SYNC (8/16/32)
    size_t validation_reps = 8;       // holdout, disjoint from fit_reps, >=8
    int64_t roi_lo = 0, roi_hi = 0;   // public candidate search ROI [lo,hi)
    // Distinct SUBTRACTION / COMMIT ROI [lo, hi): the SENSING processing region
    // (the sensing preamble / CIR window) the SIC is allowed to modify and that
    // a detected interferer must actually OVERLAP to be actionable.  It is NOT
    // the candidate search range: `roi_lo/roi_hi` above stay deliberately wide
    // so the natural preamble/multipath tail of a real comm is never clipped by
    // the candidate range (R02).  A real comm detected entirely OUTSIDE this
    // region is not actionable -- committing there would change the output
    // without removing anything from the sensing signal, which the negative
    // contract counts as a false commit -- so the core BYPASSES (out == y).
    // Disabled when `sub_roi_hi <= sub_roi_lo` (legacy: subtraction = support).
    // Absolute sample coordinates in the sic_one window.
    int64_t sub_roi_lo = 0, sub_roi_hi = 0;
    size_t sub_min_overlap = 0;       // required support∩sub_roi overlap (0 => >0)
    size_t coarse_stride = 14;        // detection coarse stride (samples)
    size_t path_min_spacing = 0;      // 0 => derive from template autocorr mainlobe
    size_t post_tail = 100;           // defined reconstruction tail (samples)
    bool enable_fractional = true;
    bool enable_cfo = true;
    double detect_threshold = 0.0;    // fixed from the CALIBRATION set
    double validate_threshold = 0.0;  // fixed from the CALIBRATION set

    // ---------------------------------------------------------------------
    // S1: strict 8-SYNC estimation, frozen model, full-nominal cancellation.
    // ---------------------------------------------------------------------
    // Support derivation (see SicSupportMode).  Default = the S1 path.
    SicSupportMode support_mode = SicSupportMode::ConfiguredNominal;
    // VISIBILITY LIMIT of the estimation stage: only y[0, min(observed_end, n))
    // may be read by ANY estimation stage (detection, repetition confirmation,
    // CFO, CIR, fractional refine, gain fit).  0 (default) => the whole sic_one
    // window is visible, which is the legacy caller contract.  Negative is
    // invalid.  This does NOT limit `apply`: the frozen model is applied to the
    // whole [0,n) window (see apply_model / sic_one).
    int64_t observed_end_sample = 0;
    // Optional EXPLICIT conservative mode (0 = default, no holdout wait): when
    // > 0 the legacy disjoint-holdout residual gate is evaluated over
    //   [origin + fit_reps*sps, origin + (fit_reps+online_validation_reps)*sps)
    // and the cancellation still covers the whole predicted support.  The extra
    // observation latency is therefore explicit.  Must be <= validation_reps
    // (the prepared holdout capacity) and fit_reps+online_validation_reps <=
    // kMaxReps.  For the exact S0 gate set combine with
    // support_mode=ConfirmedLegacy and online_validation_reps=validation_reps.
    size_t online_validation_reps = 0;
    // Default-mode commit ceiling on the NORMALIZED TRAINING residual
    // ||y - i_hat||^2 / ||y||^2 over the (prefix-clipped) training span.  This
    // is a model-quality sanity ceiling, NOT an effect threshold: a misaligned
    // or wrong-template fit leaves ~1.0, while a perfect cancellation of an
    // ISR=0 dB interferer still leaves ~0.5 (the sensing signal is in the
    // residual).  Calibration-fixed at 0.8 per the S0/S1 profile review; it must
    // be re-calibrated per profile, never tuned on acceptance windows.
    double fit_residual_max = 0.8;
};

// Both templates are supplied by the caller and live in the INDEPENDENT comm
// namespace (they never alias the sensing code/profile).
//   sync      : ONE shaped SYNC waveform (length == samples_per_symbol).
//   preamble  : the FULL comm preamble waveform (real repetition pattern,
//               leading edge transient, pulse/multipath tail).  Detection
//               correlates `sync`; reconstruction delays `preamble`.
struct SicTemplates {
    const std::complex<float>* sync = nullptr;
    size_t sync_len = 0;
    const std::complex<float>* preamble = nullptr;
    size_t preamble_len = 0;
};

// ---------------------------------------------------------------------------
// SicModel: the FROZEN estimate produced by estimate_model() and consumed by
// apply_model().  Every field is fixed at freeze time; apply NEVER updates CFO,
// delays, gains or the path count from the data it sees.  Two consequences used
// by the S1 QA:
//   * the whole struct (including tau[], gain[], support and the terminal state)
//     is a deterministic function of y[0, observed_end) + templates + config --
//     nothing after the prefix can change it;
//   * model_id is an FNV-1a-64 hash over exactly those frozen fields plus the
//     template/config hashes, so equality of model_id is a strong (but not
//     sufficient) equality check; the QA compares every field explicitly too.
// ---------------------------------------------------------------------------
struct SicModel {
    bool valid = false;                  // true <=> state == Committed (may cancel)
    SicState state = SicState::Failed;   // terminal estimation state
    const char* status = "not_estimated"; // stable literal: model_ready|bypassed|
                                          // ambiguous|failed|not_estimated
    const char* reason = nullptr;        // stable literal or nullptr
    uint64_t model_id = 0;               // hash of all frozen fields below
    uint64_t template_hash = 0;          // hash of the prepared sync+preamble bytes
    uint64_t config_hash = 0;            // hash of the estimation-relevant config
    int64_t origin = -1;                 // comm SYNC origin (integer work sample)
    double cfo_hz = 0.0;
    size_t n_paths = 0;
    // Absolute work-sample delays and COMPLEX gains (per path).  Gains are the
    // fitted least-squares coefficients in the phase_reference_sample frame:
    //   i_hat[n] = exp(j*w*(n - phase_reference_sample))
    //              * sum_k gain[k] * preamble(n - tau_abs[k])
    // where preamble(m) is the full frozen comm preamble template (finite, with
    // its natural pulse/multipath tail) and w = 2*pi*cfo_hz/sample_rate.
    double tau_abs[kSicMaxPaths] = {};
    std::complex<float> gain[kSicMaxPaths] = {};
    double gain_condition = 0.0;         // cond2(A) of the final gain-fit design
    double fit_residual = 0.0;           // ||y-i_hat||^2/||y||^2 over training
    int64_t training_lo = -1;            // samples the fit actually used
    int64_t training_hi = -1;            // (clipped to the visible prefix)
    int64_t observed_end_sample = 0;     // visibility limit that was used
    int64_t estimation_ready_sample = -1; // == training_hi
    int64_t predicted_support_lo = -1;   // permitted cancellation support
    int64_t predicted_support_hi = -1;
    const char* support_source = "none"; // stable literal
    int64_t sub_lo = -1;                 // permitted cancellation region
    int64_t sub_hi = -1;                 // (= support ∩ sub_roi, else legacy mask)
    size_t training_reps_confirmed = 0;  // reps confirmed INSIDE the training span
    size_t nominal_sync_reps = 0;        // public profile: the preamble length
    size_t samples_per_symbol = 0;
    double sample_rate = 0.0;
    int64_t phase_reference_sample = 0;  // CFO phase anchor (== origin)
};

// Oracle-only injected truth.  NEVER passed to Estimate mode.  `tau` and
// `gain` describe the interferer relative to `start_sample` (the comm SYNC
// origin in work samples).
struct SicTruth {
    bool valid = false;
    double start_sample = 0.0;
    double cfo_hz = 0.0;
    size_t n_paths = 0;
    double tau[kSicMaxPaths] = {};
    std::complex<float> gain[kSicMaxPaths] = {};
};

// Every field is measured.  Failed stages are timed; not-run stages stay 0
// (with a reason in the result).  core_total_us is the outer steady_clock
// total and is the authority; children are sequential and never summed into it.
//
// S1 decomposition (also sequential, NOT additive with the stage children):
//   estimate_total_us  acquisition + estimation + model freeze (the stages
//                      above are its children; it is NOT a new child of itself)
//   model_prepare_us   the fit-residual reconstruction + model finalisation
//   apply_us           apply_model total (reconstruct + masked subtract)
//   backfill_us        apply over [0, observed_end)  (retro-clean the prefix)
//   apply_forward_us   apply over [observed_end, n)  (forward clean)
//   apply_us == backfill_us + apply_forward_us (both measured in sic_one).
// None of these may be added to the stage children; core_total_us stays the
// single authority.
struct SicTimings {
    uint64_t detect_coarse_us = 0;
    uint64_t detect_fine_us = 0;
    uint64_t cfo_us = 0;
    uint64_t cir_us = 0;
    uint64_t path_select_us = 0;
    uint64_t fractional_refine_us = 0;
    uint64_t matrix_build_us = 0;
    uint64_t gain_fit_us = 0;
    uint64_t reconstruct_us = 0;
    uint64_t validate_commit_us = 0;
    uint64_t core_total_us = 0;
    // S1 decomposition (see above).
    uint64_t estimate_total_us = 0;
    uint64_t model_prepare_us = 0;
    uint64_t apply_us = 0;
    uint64_t backfill_us = 0;
    uint64_t apply_forward_us = 0;
    // Children of fractional_refine_us only. They are NOT stage children of
    // core_total_us and must not be added to fractional_refine_us, to the
    // stage-child sum, or to core_total_us. Clocks stay zero when
    // SicScratch::refine_diag is false; the counts below are still filled.
    uint64_t refine_init_rot_us = 0;
    uint64_t refine_init_columns_us = 0;
    uint64_t refine_init_qr_us = 0;
    uint64_t refine_resid_us = 0;
    uint64_t refine_derot_us = 0;
    uint64_t refine_candidate_us = 0;
    uint64_t refine_project_us = 0;
    uint64_t refine_self_energy_us = 0;
    uint32_t refine_k = 0;
    uint32_t refine_m = 0;
    uint32_t refine_candidates = 0;
    uint32_t refine_rebuilds = 0;
    uint32_t refine_cache_hits = 0;
    uint32_t refine_cache_misses = 0;
    uint32_t refine_self_energy_hits = 0;
    uint32_t refine_self_energy_misses = 0;
    uint32_t apply_block_builds = 0;
    uint32_t apply_block_hits = 0;
    uint32_t apply_rot_base_hits = 0;
    uint32_t apply_rot_base_misses = 0;
};

struct SicResult {
    SicState state = SicState::Bypassed;
    bool detected = false;
    bool committed = false;
    bool partial = false;
    bool ambiguous = false;
    const char* bypass_reason = nullptr;   // stable string literal
    bool uses_truth = false;
    int64_t detected_start = -1;           // estimated comm SYNC origin (work samples)
    int64_t support_lo = -1;
    int64_t support_hi = -1;               // PREDICTED support [lo,hi); see
                                           // support_source (S1: not "confirmed")
    double cfo_hz = 0.0;
    size_t n_paths = 0;
    size_t n_path_candidates = 0;
    // Selected fit delays, ABSOLUTE work samples: tau_abs[k] = detected_start +
    // (comm-relative delay_k), matching sic_reference.py's result["taus"].  This
    // is a debug/inspection field: it makes the fractional-refine selection
    // directly checkable against the NumPy reference instead of only implied by
    // an NMSE tolerance.  Estimate and Oracle both fill it; `n_taus` is the
    // number of valid entries.
    size_t n_taus = 0;
    double taus[kSicMaxPaths] = {};
    double gain_condition = 0.0;           // cond2(A) of the final gain-fit design
    double residual_corr = 0.0;            // holdout comm residual correlation
    // |support ∩ sub_roi| in samples: the actionable overlap used by the
    // subtraction/commit gate.  -1 when no sub_roi was configured (disabled).
    int64_t sub_overlap = -1;
    uint64_t coarse_candidates = 0;
    uint64_t fine_candidates = 0;
    uint64_t correlations = 0;
    size_t roi_samples = 0;
    const char* reject_reason = nullptr;   // when not committed
    SicTimings timings;

    // ---------------------------------------------------------------------
    // S1 diagnostics.  support_lo/support_hi now carry the PREDICTED support
    // (``support_source`` says which derivation produced it):
    //   "configured_nominal_prediction" : detected origin + configured
    //       sync_reps*sps + post_tail, clipped to [0,n).  This is NOT a measured
    //       64-SYNC boundary and is NOT clamped to the confirmed repetition count.
    //   "confirmed_repetition_legacy"   : S0 confirmed*sps + post_tail.
    // ---------------------------------------------------------------------
    const char* support_source = nullptr;   // stable string literal
    double fit_residual = 0.0;              // training-region ||y-i_hat||^2/||y||^2
    size_t training_reps_confirmed = 0;     // confirmed reps INSIDE the training span
    int64_t observed_end_sample = 0;        // visibility limit actually used
    uint64_t model_id = 0;                  // hash of the frozen model fields

    // ---------------------------------------------------------------------
    // I/O buffers: CALLER-OWNED, both of length `n` (the sic_one window).
    // Set them before calling sic_one.  They are preserved across the call
    // (sic_one never resets these two pointers).
    //
    //   out   : the cleaned window, written by sic_one.  On bypass/failure it
    //           is a byte-for-byte copy of y.
    //   i_hat : optional; the reconstruction over the union of the delayed
    //           preamble supports.  It is NOT zero outside the subtraction mask
    //           (only `out` is guaranteed to equal y outside the mask).
    // If a pointer is null the corresponding buffer is simply not written.
    // ---------------------------------------------------------------------
    std::complex<float>* out = nullptr;
    std::complex<float>* i_hat = nullptr;
};

// ---------------------------------------------------------------------------
// Preallocated scratch.  Allocated / prepared ONCE.  sic_one never grows it.
//
// Peak scratch bytes for the main work config (sps=1016, k=4, fit_reps=8,
// preamble_len~65024, post_tail=100) is reported in `peak_bytes` after
// prepare_sic_core(); it is dominated by the polyphase-upsampled preamble
// (kPreUpPhases rows), followed by the M x K double-precision design matrix,
// the length-`cap_region` double reconstruction accumulator / CFO ramp and the
// float32 `cap_region` reconstruction ramp (rot_f).
// ---------------------------------------------------------------------------
struct SicScratch {
    bool prepared = false;
    const void* orig_sync = nullptr;      // identity captured at prepare
    const void* orig_preamble = nullptr;
    size_t sync_len = 0;
    size_t preamble_len = 0;
    size_t sps = 0;
    size_t k_paths = 0;
    size_t fit_reps = 0;
    size_t validation_reps = 0;
    size_t post_tail = 0;
    size_t autocorr_half = 1;             // template autocorr mainlobe half-width
    size_t min_spacing = 1;               // derived path min spacing (samples)
    size_t fit_len = 0;                   // M = fit_reps * sps
    size_t work_len = 0;                  // max(fit_reps, validation_reps) * sps
    size_t cap_region = 0;                // max reconstruction-mask length
    size_t peak_bytes = 0;                // peak scratch footprint (bytes)
    uint64_t template_hash = 0;           // FNV-1a-64 of the prepared template bytes

    std::vector<std::complex<float>> sync_norm;  // L2-normalized `sync`
    std::vector<std::complex<float>> preamble;   // raw `preamble` copy
    // Phase-major polyphase-upsampled `preamble`: kPreUpPhases rows of
    // `preamble_len`, row g holding the template advanced by g/kPreUpPhases of
    // a sample (row 0 is the exact integer copy).  This is the ONE
    // fractional-delay source: matrix_build / fractional_refine / reconstruct
    // read at most two neighbouring rows and blend them (LINEARLY in the
    // sub-sample phase) instead of running a 33-tap windowed-sinc dot product
    // per output sample.  Each row is the same windowed-sinc kernel evaluated at
    // its own phase; the two-row blend is a piecewise-linear approximation of
    // that kernel in the phase variable, not the kernel itself.  Size =
    // kPreUpPhases * preamble_len * 8 bytes (see peak_bytes).
    std::vector<std::complex<float>> pre_up;
    std::vector<std::complex<float>> cir;        // local matched-filter CIR
    std::vector<float> cir_mag;
    std::vector<std::complex<double>> A;         // M x K design matrix
    std::vector<std::complex<double>> hh_v;      // Householder vector (M)
    std::vector<std::complex<double>> hh_b;      // RHS / transformed RHS (M)
    std::vector<std::complex<double>> col;       // column builder (M)
    std::vector<std::complex<float>> col_f;      // float shifted-template column (M)
    std::vector<std::complex<float>> q_f;        // float rotated residual (M)
    std::vector<std::complex<double>> resid;     // greedy residual (M)
    std::vector<std::complex<double>> rot;       // CFO phasor (double), max(M, cap_region)
    // Float32 CFO ramp for the reconstruction combine and its fixed block base
    // ramp: rot_f[b*B + j] = rot_base[j] * blockphase_b.  Building the ramp this
    // way replaces the previous serial per-sample double recurrence (which cost
    // ~0.39 ms over the ~65k-sample reconstruction window) with one VOLK 32fc
    // scalar multiply per block, re-anchored absolutely every block so the phase
    // error stays at float32 rounding (~1.4e-7 max vs the double ramp) instead
    // of accumulating over the whole window.
    std::vector<std::complex<float>> rot_f;      // float32 CFO ramp, cap_region
    std::vector<std::complex<float>> rot_base;   // one block of exp(j*w*j)
    std::vector<std::complex<double>> acc;       // double reconstruction acc, cap_region
    std::vector<std::complex<float>> ihat;       // reconstruction over mask
    std::vector<int64_t> cand_delay;             // CIR local-peak delays
    std::vector<double> cand_mag;                // CIR local-peak magnitudes
    // Sliding-window self-energy prefix of |y|^2 (absolute index, double
    // precision): energy_prefix[i] = sum_{k<i} |y[k]|^2.  The detection
    // normalization ||y[d..d+Ls)||^2 = energy_prefix[d+Ls] - energy_prefix[d]
    // is then ONE indexed difference per lag instead of a second per-lag VOLK
    // conjugate dot <seg,seg>, halving the detection work with (double-exact)
    // identical normalization.  Sized by prepare_sic_core to cover every lag
    // the prepared ROI + repetition geometry can query; any query outside it
    // falls back to the exact legacy VOLK self-dot, so a runtime ROI/window
    // change stays correct.  Size is O(ROI + reps*sps) doubles.
    std::vector<double> energy_prefix;

    // ---------------------------------------------------------------------
    // feature/sic-perf.  These are NOT part of the frozen model hash.  They
    // are fixed when prepare_sic_core() runs, from the environment below, so
    // a paired benchmark can toggle one item without a second binary.
    //   UWB_SIC_REFINE_DIAG     default 1  (sub-clocks inside fractional_refine)
    //   UWB_SIC_COL_CACHE       default 1  (E0 path-column cache)
    //   UWB_SIC_ROT_BASE_CACHE  default 1  (E0 CFO base-table cache)
    //   UWB_SIC_RECON_CACHE     default 1  (E0 absolute reconstruction block)
    //   UWB_SIC_SELF_ENERGY     default 1  (E0 geometry self-energy table)
    //   UWB_SIC_INCR_RESID      default 0  (E1 incremental residual)
    //   UWB_SIC_FUSED_SCORE     default 0  (E1 fused candidate score)
    // A value starting with 0/n/N/f/F disables the item.
    // ---------------------------------------------------------------------
    bool refine_diag = true;
    bool opt_column_cache = true;
    bool opt_rot_base_cache = true;
    bool opt_recon_cache = true;
    bool opt_self_energy = true;
    bool opt_incr_resid = false;
    bool opt_fused_score = false;

    std::vector<std::complex<double>> col_cache;    // k_paths * work_len
    std::vector<std::complex<double>> sync_conj_d;  // conj(sync) in double
    bool train_rot_ready = false;
    bool train_columns_ready = false;
    int64_t train_rot_idx0 = 0;
    size_t train_rot_count = 0;
    double train_rot_w = 0.0;
    size_t train_col_m = 0;
    size_t train_col_k = 0;

    bool rot_base_valid = false;
    double rot_base_w = 0.0;

    // Last absolute reconstruction block written into ihat[0, recon_len).
    // The key is the model parameters themselves, not model_id alone.
    bool recon_valid = false;
    int64_t recon_b0 = 0;
    size_t recon_len = 0;
    uint64_t recon_template_hash = 0;
    double recon_cfo = 0.0;
    double recon_fs = 0.0;
    int64_t recon_phase_ref = 0;
    size_t recon_n_paths = 0;
    double recon_tau[kSicMaxPaths] = {};
    std::complex<float> recon_gain[kSicMaxPaths] = {};

    // ||u(relative delay, 0.05-grid delta)||^2 for the prepared training
    // length.  NaN until filled.  A clipped training window does not use it.
    std::vector<double> self_energy;
    bool self_energy_ready = false;
    int64_t self_energy_rel_min = 0;
    size_t self_energy_rel_span = 0;
    size_t self_energy_m = 0;
};

// ---------------------------------------------------------------------------
// detail:: internal helpers, exposed ONLY so the QA can drive them directly
// (e.g. at a declared condition number the signal-level path cannot synthesize).
// They are NOT part of the frozen contract and must not be used by production
// code; sic_one() calls them internally.
// ---------------------------------------------------------------------------
namespace detail {

// Complex least squares A h ~ y for a column-major M x K matrix (K in
// [1, kSicMaxPaths]) via Householder QR of A (NO normal equations, NO explicit
// inverse): factor A = Q R, transform the RHS with Q^H, back-substitute
// R h = Q^H y.  A is overwritten with the factor R; `work` is caller scratch of
// length >= M used for the Householder vector.
//
// The reported `*cond` is cond2(A) -- the ratio of the largest to smallest
// singular value, the same quantity as NumPy's `numpy.linalg.cond`.  It is
// computed from the singular values of the K x K factor R: A = Q R with Q
// orthogonal implies A^H A = R^H R, so cond2(A) == cond2(R) exactly.  The
// singular values of R come from a self-contained one-sided complex Jacobi SVD
// (K <= 16, no LAPACK); operating on R and never on R^H R avoids the
// normal-equation kappa^2 amplification.  This is NOT the ratio of the |R_kk|
// diagonal entries, which underestimates cond(A).
//
// Returns false when the factor is rank-deficient / non-finite or a computed
// singular value is not strictly positive (fail-closed); on success writes
// h[0..K) and, when non-null, *cond = cond2(A) >= 1.
UWB_API bool qr_cond_solve(std::complex<double>* A,
                           size_t M,
                           size_t K,
                           std::complex<double>* y,
                           std::complex<double>* h,
                           std::complex<double>* work,
                           double* cond);

} // namespace detail

// ---------------------------------------------------------------------------
// prepare_sic_core: may allocate.  Call at init / when the comm template
// changes.  Captures the template identity and pre-computes derived quantities
// (normalized sync, autocorr mainlobe width, capacity).  `err`, when non-null,
// receives a short diagnostic on failure.  Safe to call with mode Off (the
// template/identity still have to be valid to be usable later).
//
// Return value: true on success.
// ---------------------------------------------------------------------------
UWB_API bool prepare_sic_core(const SicConfig& cfg,
                              const std::complex<float>* sync,
                              size_t sync_len,
                              const std::complex<float>* preamble,
                              size_t preamble_len,
                              SicScratch& scratch,
                              std::string* err = nullptr);

// ---------------------------------------------------------------------------
// estimate_model: S1 estimation stage.  May NOT allocate (scratch is prepared).
//
// Reads ONLY y[0, min(observed_end, n)) and the templates/config:
//   * every correlation row / LS row / CIR tap / CFO pair lies fully inside the
//     visible prefix (a row that would cross it is dropped, never read past);
//   * the training span is [origin, origin + fit_reps*samples_per_symbol), also
//     clipped to the visible prefix;
//   * the injected comm start is never an input (detection runs from the public
//     coarse ROI only).
// `observed_end <= 0` means "the whole window is visible" (legacy callers).
//
// On success `model` is a FROZEN SicModel (valid or not; see model.state/status/
// reason) and apply_model() is the only consumer.  Returns false only when no
// defined model could be produced at all (null y / n==0 / unprepared scratch /
// template or geometry mismatch); in that case model.reason says why.
//
// `timings` (optional) receives the stage timings.  `diag` (optional) receives
// the SicResult diagnostics (detected flags, stage counters, support, overlap);
// its caller-owned out/i_hat pointers are preserved.
// ---------------------------------------------------------------------------
UWB_API bool estimate_model(const std::complex<float>* y,
                            size_t n,
                            int64_t observed_end,
                            const SicTemplates& templates,
                            const SicConfig& cfg,
                            SicScratch& scratch,
                            SicModel& model,
                            SicTimings* timings = nullptr,
                            SicResult* diag = nullptr);

// ---------------------------------------------------------------------------
// apply_model: apply a FROZEN model to an arbitrary chunk [offset, offset+n).
//
//   y / out / i_hat : chunk-length buffers; `y` is the window starting at
//                     ABSOLUTE sample `offset`.  `out` and `i_hat` may differ.
//   i_hat[n] = exp(j*w*(n - phase_reference_sample))
//              * sum_k gain[k] * preamble(n - tau_abs[k])
//   out[n]   = y[n] - i_hat[n]   ONLY inside model.[sub_lo, sub_hi)
//   out[n]   = y[n]              bit-identically everywhere else.
//
// The CFO phase is anchored ABSOLUTELY at phase_reference_sample and the phase
// block grid is aligned to absolute sample multiples, so a given absolute sample
// is reconstructed with the SAME arithmetic in every chunking: whole-window,
// fixed chunks and random chunks agree BIT-IDENTICALLY (no cross-chunk state,
// no re-fit).  Preallocated scratch only: no heap growth, no I/O, and no
// per-sample sin/cos (one exp per kRotBlock samples plus one per chunk edge).
//
// Fail-closed: a null `out`, an unprepared scratch, a template/geometry
// mismatch, an invalid model, or a reconstruction region larger than the
// prepared capacity all leave out == y byte-for-byte.  Returns false only when
// `out` is null or `n > 0` with `y == nullptr` (no defined output possible).
//
// `timings` (optional) accumulates reconstruct_us and apply_us.
//
// `out_preinitialized`: the caller has already copied `y` into `out` and, when
// `i_hat` is non-null, filled that chunk with zeros.  apply then skips those
// two writes.  The public default is false, so a caller that does not
// preinitialize still gets the fail-closed `out == y` copy.  Passing true
// without having done both initializations is a caller bug.
// ---------------------------------------------------------------------------
UWB_API bool apply_model(const SicModel& model,
                         const std::complex<float>* y,
                         size_t n,
                         int64_t offset,
                         const SicTemplates& templates,
                         const SicConfig& cfg,
                         SicScratch& scratch,
                         std::complex<float>* out,
                         std::complex<float>* i_hat,
                         SicTimings* timings = nullptr,
                         bool out_preinitialized = false);

// ---------------------------------------------------------------------------
// sic_one: the hot path.  No allocation, no vector growth, no file I/O.
//
// S1 composition: estimation over min(n, observed_end) (0 => n), then the
// frozen model is applied to the WHOLE [0,n) window in two passes -- the
// backfill of the already-observed prefix [0, observed_end) and the forward
// clean [observed_end, n) -- so the cross-chunk phase/index continuity is the
// same code path the hardware loop would use.
//
//   y / n     : input IQ window (n samples).
//   templates : the same sync/preamble handed to prepare_sic_core (lengths and
//               pointers are checked against the prepared identity).
//   cfg       : runtime config.  Template geometry (samples_per_symbol,
//               sync_reps, k_paths, fit_reps, validation_reps, post_tail) must
//               agree with prepare; roi / stride / thresholds / support mode /
//               visibility / online validation may change.
//   scratch   : prepared once.
//   out       : result + caller-owned out/i_hat buffers.
//   truth     : Oracle only.  Pass nullptr in Estimate mode.
//
// Returns false only when the call cannot write a defined `out` (null y, n==0,
// unprepared, template mismatch, invalid config).  For every other path it
// returns true, sets a terminal SicState, and guarantees out == y unless the
// state is Committed.
// ---------------------------------------------------------------------------
UWB_API bool sic_one(const std::complex<float>* y,
                     size_t n,
                     const SicTemplates& templates,
                     const SicConfig& cfg,
                     SicScratch& scratch,
                     SicResult& out,
                     const SicTruth* truth = nullptr);

} // namespace sic
} // namespace uwb
} // namespace gr
