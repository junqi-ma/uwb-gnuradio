/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Full-packet SIC: direct discrete equivalent CIR estimation.
 *
 * This is an EXPLICIT experiment backend for the full-packet path.  It does
 * not change the frozen preamble-only core, adapter or Python reference.
 *
 * Math (absolute work-grid samples, packet origin q, CFO w, phase anchor r=q):
 *
 *   y_hat[n] = exp(j*w*(n-r)) * sum_l h[l] * x[n-q-l]
 *
 * x is the COMPLETE pulse-shaped waveform produced by the HRP modulator, not
 * the chip sequence and not a correlator output.  h is a finite integer-lag
 * FIR estimated ONLY from the first fit_reps SYNC symbols visible before
 * observed_end:
 *
 *   z[m]   = y[q+m] * exp(-j*w*m)
 *   X[m,i] = train[m-(lag_lo+i)]          (zero before the packet start)
 *   h      = argmin ||X h - z||^2 + lambda ||h||^2
 *
 * The support is a contiguous finite lag interval selected from a coarse
 * matched-filter CIR over the same prefix.  Candidate length supports are
 * pre-factorised once per (template, lag, training rows, lambda) key; the hot
 * call only projects z onto the cached support and does two triangular solves.
 *
 * Normalisation: h is fitted against the PUBLIC shaped SYNC train.  At apply
 * time the remodulated packet is scaled ONCE to that public reference before
 * the FIR, so h already contains the packet amplitude and must not be scaled
 * again.  This matches the existing full-packet path's waveform_scale check.
 *
 * Fail-closed: every estimate failure leaves model.valid == false.  Every
 * apply failure copies out == y byte-for-byte and zeroes ihat.
 */
#pragma once

#include <gnuradio/uwb/api.h>
#include <gnuradio/uwb/uwb_hrp_mod_core.h>

#include <array>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace gr {
namespace uwb {
namespace sic {

// Maximum finite lag support the estimator will ever prepare.  A support must
// also stay strictly shorter than one SYNC period (samples_per_symbol).
constexpr size_t kDiscreteCirMaxTaps = 256;
constexpr size_t kDiscreteCirMaxActive = 64;

struct DiscreteCirConfig {
    // Public profile.  The estimator never receives injected delay/CFO/paths.
    size_t comm_code = 10;
    size_t sync_reps = 64;
    size_t samples_per_symbol = 1016;
    double sample_rate = 998.4e6;
    size_t fit_reps = 8;             // training symbols, strictly visible prefix
    int64_t roi_lo = 0, roi_hi = 0;  // public coarse candidate search ROI

    // Finite discrete support.  lag_lo is the LOWEST integer lag.  tap_count
    // is the MAXIMUM continuous hull width.  The estimator selects local
    // coarse-CIR peaks above `peak_floor` (relative to the strongest peak),
    // enforces `peak_min_spacing` samples between peaks, and activates a
    // +/-`active_radius` group around each selected peak.  The continuous
    // hull spans [min(active),max(active)]; taps outside the active groups are
    // exactly zero.  The hull must stay strictly shorter than one SYNC period.
    int64_t lag_lo = -8;
    size_t tap_count = 64;
    double peak_floor = 0.15;
    size_t peak_min_spacing = 5;
    size_t active_radius = 1;

    // Ridge coefficient RELATIVE to the mean diagonal of X^H X.  Calibration
    // fixed; never selected on acceptance windows.
    double lambda = 1e-6;

    // Acquisition gates copied from the frozen experiment profile.  They are
    // calibrated public constants, not injected truth.
    size_t coarse_stride = 1;
    double detect_threshold = 0.40;
    double fit_residual_max = 0.8;
    double gain_condition_max = 1e8;

    // Prepared capacities.  max_window_samples is the largest y/out window
    // (the benchmark passes public.n); max_packet_samples bounds the
    // remodulated waveform and is derived when zero.
    size_t max_window_samples = 0;
    size_t max_packet_samples = 0;
};

// Both templates are public.  `sync` is ONE shaped SYNC symbol.  `train` is
// the first fit_reps shaped SYNC symbols of the public comm preamble; it is
// NOT tiled from `sync` because the real finite train keeps the start
// transient and inter-symbol pulse tail.
struct DiscreteCirTemplates {
    const std::complex<float>* sync = nullptr;
    size_t sync_len = 0;
    const std::complex<float>* train = nullptr;
    size_t train_len = 0;
};

// Cached factorization for one ACTIVE lag set (sorted absolute lags).  `chol`
// is the row-major lower Cholesky factor of X^H X + lambda I over the active
// columns.  The cache is pre-sized in prepare; a miss fills a round-robin slot
// without allocating on the hot path.
struct DiscreteCirActiveFactor {
    uint64_t key = 0;
    size_t count = 0;
    std::array<int16_t, kDiscreteCirMaxActive> lags{};
    double condition = 0.0;
    size_t effective_rank = 0;
    double lambda_abs = 0.0;
    std::vector<std::complex<double>> chol; // count*count, row-major
};

struct DiscreteCirScratch {
    bool prepared = false;
    uint64_t template_hash = 0;
    uint64_t config_hash = 0;
    const void* orig_sync = nullptr;
    const void* orig_train = nullptr;
    size_t sync_len = 0;
    size_t train_len = 0;
    size_t fit_reps = 0;
    size_t sps = 0;
    size_t max_taps = 0;
    size_t max_active = 0;
    size_t max_window_samples = 0;
    size_t max_packet_samples = 0;
    int64_t lag_lo = 0;
    int64_t lag_hi = 0;
    size_t training_rows = 0;        // M = fit_reps*sps + min(0, lag_lo)
    double lambda_rel = 0.0;
    double peak_floor = 0.15;
    size_t peak_min_spacing = 5;
    size_t active_radius = 1;

    std::vector<std::complex<float>> sync_norm;
    std::vector<std::complex<float>> train;
    std::vector<std::complex<double>> gram;   // full max-support Gram
    std::vector<DiscreteCirActiveFactor> factor_cache;
    size_t factor_cache_next = 0;
    std::vector<double> energy_prefix;        // visible-prefix |y|^2
    std::vector<std::complex<double>> z;      // training target
    std::vector<std::complex<double>> b;      // X^H z (active set)
    std::vector<std::complex<double>> work;   // triangular solve / condition
    std::vector<std::complex<double>> factor_work; // K*K cache-miss eigen temp
    std::vector<double> factor_eigen;               // K eigen values
    std::vector<std::complex<float>> waveform;       // remodulated packet
    std::vector<std::complex<float>> scaled_waveform; // waveform scaled once by apply
    std::vector<std::complex<float>> accumulator;     // tap-major FIR accumulator
    std::vector<std::complex<float>> fir_tmp;          // CFO scalar-multiply temp
    std::vector<int8_t> expected_phr;
    mod::HrpModScratch modulator;

    UWB_API void reserve(size_t max_window_samples);
};

struct DiscreteCirModel {
    bool valid = false;
    const char* status = "not_estimated";
    const char* reason = nullptr;
    uint64_t model_id = 0;
    uint64_t template_hash = 0;
    uint64_t config_hash = 0;
    int64_t q = -1;                    // integer packet origin
    double cfo_hz = 0.0;
    int64_t phase_reference_sample = 0; // r == q
    int64_t observed_end_sample = 0;
    int64_t lag_lo = 0;                // continuous hull lowest lag
    size_t tap_count = 0;              // continuous hull width
    size_t active_count = 0;           // nonzero active taps inside hull
    std::array<int16_t, kDiscreteCirMaxActive> active_lags{};
    std::array<std::complex<float>, kDiscreteCirMaxTaps> taps{}; // dense hull, zeros inactive
    double lambda = 0.0;               // absolute ridge used
    double gain_condition = 0.0;       // cond2(X), from cached factor
    size_t effective_rank = 0;
    double fit_residual = 0.0;         // normalized training residual (float taps)
    int64_t training_lo = -1;
    int64_t training_hi = -1;
    size_t repeats_confirmed = 0;
    double detection_metric = 0.0;     // peak normalized correlation
    double coarse_peak = 0.0;
    const char* support_source = "none"; // "coarse_cir_peak_groups"
};

struct DiscreteCirTimings {
    double detect_us = 0, cfo_us = 0, support_us = 0;
    double projection_us = 0, solve_us = 0, fit_check_us = 0;
    double estimate_total_us = 0;
};

// Apply diagnostics.  `channel_us` is the parent of scale/convolution/CFO/
// finite checks and is never added again to a parent total by callers.
struct DiscreteCirResult {
    bool committed = false;
    const char* reason = "not_run";
    size_t packet_samples = 0;
    int64_t support_lo = 0, support_hi = 0;
    int64_t sub_lo = 0, sub_hi = 0;
    double waveform_scale = 0.0;
    double validate_us = 0, tx_us = 0;
    double scale_us = 0, convolution_us = 0, cfo_us = 0;
    double channel_us = 0, subtract_us = 0, apply_total_us = 0;
    uint64_t model_id = 0;
    int64_t q = -1;
    double cfo_hz = 0.0;
    size_t tap_count = 0;
    double fit_residual = 0.0, gain_condition = 0.0;
};

UWB_API bool prepare_discrete_cir(const DiscreteCirConfig& cfg,
                                  const DiscreteCirTemplates& templates,
                                  DiscreteCirScratch& scratch,
                                  std::string* err = nullptr);

// Strict-prefix estimation.  Reads only y[0, min(observed_end, n)); every
// detection row, coarse CIR row and training row lies fully inside it.
UWB_API bool estimate_discrete_cir(const std::complex<float>* y,
                                   size_t n,
                                   int64_t observed_end,
                                   const DiscreteCirTemplates& templates,
                                   const DiscreteCirConfig& cfg,
                                   DiscreteCirScratch& scratch,
                                   DiscreteCirModel& model,
                                   DiscreteCirTimings* timings = nullptr);

// Low-level apply.  `waveform` is the complete remodulated packet (length
// waveform_n).  `reference_sync` is the public one-symbol shaped SYNC used to
// compute the single waveform scale.  out == y and ihat == 0 on every failure.
UWB_API bool apply_discrete_cir(const std::complex<float>* y,
                                size_t n,
                                const std::complex<float>* waveform,
                                size_t waveform_n,
                                const std::complex<float>* reference_sync,
                                size_t sync_n,
                                const DiscreteCirModel& model,
                                int64_t roi_lo,
                                int64_t roi_hi,
                                DiscreteCirScratch& scratch,
                                std::complex<float>* out,
                                std::complex<float>* ihat,
                                DiscreteCirResult* result = nullptr);

// Transactional full-packet wrapper: validates FCS/PHR/profile, remodulates
// the decoded wire PSDU once, applies the frozen discrete model and subtracts
// only inside [roi_lo, roi_hi).  No truth or sidecar input.
UWB_API DiscreteCirResult fullpacket_cancel_discrete(
    const std::complex<float>* y,
    size_t n,
    const std::vector<uint8_t>& phr_bits,
    const std::vector<uint8_t>& wire_psdu,
    bool fcs_pass,
    const mod::HrpModConfig& profile,
    const DiscreteCirModel& model,
    const std::complex<float>* reference_sync,
    size_t sync_n,
    int64_t roi_lo,
    int64_t roi_hi,
    DiscreteCirScratch& scratch,
    std::complex<float>* out,
    std::complex<float>* ihat);

} // namespace sic
} // namespace uwb
} // namespace gr
