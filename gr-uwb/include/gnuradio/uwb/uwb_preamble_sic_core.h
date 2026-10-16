/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * S0 preamble-only SIC math core (frozen contract:
 * docs/sic/S0_sic_core_api.md).  Namespace gr::uwb::sic.
 *
 * This is the PURE math core of the detection-triggered preamble SIC branch.
 * It has NO GNU Radio types, NO scheduler, NO file I/O and does NOT allocate
 * or grow a vector inside sic_one(): every buffer is sized once by
 * prepare_sic_core().  The PDU adapter (uwb_preamble_sic.*) is a separate,
 * thinner layer.
 *
 * Pipeline (contract §Algorithm):
 *   detect_coarse / detect_fine  -> detect_threshold gate
 *   repetition confirmation       -> confirmed origin + support
 *   cfo from repeated-peak phases -> rotate the RECONSTRUCTION basis (never y)
 *   cir / path_select             -> <= K distinct integer delays
 *   fractional_refine (optional)  -> core-owned bandlimited interpolation
 *   matrix_build / gain_fit       -> complex least squares via the K x K Gram
 *                                    matrix + Cholesky (double; no inverse)
 *   reconstruct                   -> i_hat over the union support window
 *   validate_commit               -> DISJOINT holdout reps; fail-closed
 *
 * Output convention (matches UWB_Preamble_SIC_GNU_Radio_Design.md):
 *   i_hat[n] = sum_k h_k * preamble[n - origin - tau_k] * exp(j 2*pi*f*n/Fs)
 *   out[n]   = y[n] - i_hat[n]
 * i_hat is the reconstruction over the union of the delayed-preamble supports
 * (the whole `plen`-sample tail); it is NOT confined to the subtraction mask.
 * Only `out` equals `y` byte-for-byte outside the subtraction mask, and EVERY
 * failure / bypass path leaves out bit-identical to y.
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
// final gain fit forms the K x K Gram matrix and RHS in DOUBLE precision (a
// flat, unrolled complex conjugate dot per entry) and solves them with a
// self-contained double Cholesky -- no explicit inverse, and NOT a float32
// normal-equation factor, so `gain_condition` is the honest cond(A) up to the
// 1e6 commit ceiling.  VOLK is already a link dependency of gnuradio-uwb, so
// this adds no new wiring.  The fractional-refine seed solve remains a
// Householder QR in the .cc.
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
    size_t coarse_stride = 14;        // detection coarse stride (samples)
    size_t path_min_spacing = 0;      // 0 => derive from template autocorr mainlobe
    size_t post_tail = 100;           // defined reconstruction tail (samples)
    bool enable_fractional = true;
    bool enable_cfo = true;
    double detect_threshold = 0.0;    // fixed from the CALIBRATION set
    double validate_threshold = 0.0;  // fixed from the CALIBRATION set
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
    int64_t support_hi = -1;               // confirmed interferer support [lo,hi)
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
    double gain_condition = 0.0;           // rank/conditioning indicator
    double residual_corr = 0.0;            // holdout comm residual correlation
    uint64_t coarse_candidates = 0;
    uint64_t fine_candidates = 0;
    uint64_t correlations = 0;
    size_t roi_samples = 0;
    const char* reject_reason = nullptr;   // when not committed
    SicTimings timings;

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
// Peak scratch bytes for the S0 main config (sps=1016, k=4, fit_reps=8,
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
};

// ---------------------------------------------------------------------------
// detail:: internal helpers, exposed ONLY so the QA can drive them directly
// (e.g. at a declared condition number the signal-level path cannot synthesize).
// They are NOT part of the frozen contract and must not be used by production
// code; sic_one() calls them internally.
// ---------------------------------------------------------------------------
namespace detail {

// Complex least squares A h ~ y for a column-major M x K matrix (K in
// [1, kSicMaxPaths]) via the K x K Gram matrix G = A^H A and a double
// Cholesky factorization G = L L^H.  The Gram and RHS are accumulated in DOUBLE
// (not float32) so that cond(G) = cond(A)^2 cannot destroy the conditioning
// indicator at the 1e6 commit ceiling.  No explicit matrix inverse is formed.
// Returns false when a Cholesky pivot is non-positive / non-finite (rank or
// conditioning collapse); on success writes h[0..K) and, when non-null,
// *cond = max|R_kk| / min|R_kk| = cond(A).
UWB_API bool gram_cholesky_solve(const std::complex<double>* A,
                                 size_t M,
                                 size_t K,
                                 const std::complex<double>* y,
                                 std::complex<double>* h,
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
// sic_one: the hot path.  No allocation, no vector growth, no file I/O.
//
//   y / n     : input IQ window (n samples).
//   templates : the same sync/preamble handed to prepare_sic_core (lengths and
//               pointers are checked against the prepared identity).
//   cfg       : runtime config.  Template geometry (samples_per_symbol,
//               sync_reps, k_paths, fit_reps, validation_reps, post_tail) must
//               agree with prepare; roi / stride / thresholds may change.
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
