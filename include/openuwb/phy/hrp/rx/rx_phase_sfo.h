/* -*- c++ -*- */
/* SPDX-License-Identifier: GPL-3.0-or-later */
/*
 * Pure-DSP helpers for residual phase/CFO refinement from the 21-symbol HRP
 * PHR pilot and for a fractional-delay CIR template bank used to track
 * sampling-frequency offset (SFO) on the payload.
 *
 * This header has no GNU Radio, UHD or other in-tree dependency: it only uses
 * the C++ standard library, so it can be unit tested standalone and compiled
 * in scalar-only builds.  Nothing here allocates, touches global state or
 * silently falls back to a different grid.
 *
 * ---------------------------------------------------------------------------
 * 1) PHR phase / residual CFO refinement
 * ---------------------------------------------------------------------------
 *
 * Input contract for `phr_z_win` (IMPORTANT):
 *
 *   phr_z_win[k] is the complex correlation of the WINNING BPM half of PHR
 *   symbol k, exactly as produced by `rxfusion::correlate_phr()` /
 *   `correlate_payload_fused_cfo()` in `Metrics::complex_value`:
 *
 *     - NOT multiplied by `rot` (the frozen preamble phase rotation),
 *     - NOT multiplied by `sfd_polarity`,
 *     - the initial `cfo_w_rad` phase ramp has ALREADY been removed, so the
 *       phase reference is the absolute sample index used by the CFO
 *       derotation (sample 0 of the derotated buffer).
 *
 *   With BPRF polarity p (+1/-1) and PHR data bit g1[k] (0/1) the winning
 *   half carries the BPSK sign `p * (1 - 2*g1[k])`, hence
 *
 *     tilde[k] = (1 - 2*phr_g1[k]) * sfd_polarity * phr_z_win[k]
 *              ~ A_k * exp(+j * theta_res(k)),
 *
 *   where theta_res is the residual signal phase AFTER the preamble rotation
 *   and the initial CFO were removed.  theta_res therefore still contains the
 *   static phase bias and any residual CFO drift.
 *
 * Returned rotation contract:
 *
 *   `rot_at_payload_start` is the rotation in the SAME convention as the
 *   `rot` argument of `correlate_payload()` / `correlate_phr()`, i.e. the
 *   production metric is
 *
 *     signed = polarity * Re(rot * complex_value)
 *
 *   so a positive BPSK symbol is derotated by `rot = exp(-j*theta_res)`.
 *   Consequently
 *
 *     rot_at_payload_start = rot_init * exp(-j * theta_payload)
 *
 *   where theta_payload is the extrapolated residual signal phase at the
 *   payload window start (see below).  Multiplying by `exp(+j*...)` would
 *   rotate the constellation away from the decision axis and double the
 *   static bias angle.  The result is normalised: |rot_at_payload_start| = 1
 *   (to float rounding), so chaining it as the next `rot` never changes the
 *   decision scale.  On rejection the returned rotation is `rot_init`.
 *
 * `delta_cfo_w_rad` is the residual CFO the correlation kernel still has to
 * compensate, in rad/sample.  Sign convention: it is the per-sample phase
 * increment of tilde, so a positive residual CFO (phase of the derotated
 * signal advancing with sample index) yields a POSITIVE value.  Wave 2 adds
 * it to the next kernel's `cfo_w_rad` (the kernel derotates with
 * exp(-j*cfo_w_rad*i)).  To convert to Hz the caller supplies its own sample
 * rate:  delta_cfo_hz = delta_cfo_w_rad * fs / (2*pi).
 *
 * Estimator (21 symbols, nsym >= 4):
 *   coherence   C = |sum tilde| / (sum |tilde| + eps);  C < min_coherence ->
 *               reject (applied = false, rot = rot_init, delta = 0).
 *   symbol step dtheta = arg(sum_{k=0..nsym-2} tilde[k+1]*conj(tilde[k])),
 *               delta = clamp(dtheta / phr_symbol_samples, +-max_delta_w_rad).
 *   the phases are re-referenced to the centre symbol
 *               kc = (nsym-1)/2  (double; for even nsym this is the exact
 *               half-integer midpoint between the two centre symbols),
 *               theta_c = arg(sum_k u[k] * exp(-j*delta*(k-kc)*S)),
 *               u[k] = rot_init_normalised * tilde[k]  (the rot_init frame),
 *               theta_payload = theta_c + delta*d,
 *   with
 *               d = phr_last_to_payload_samples + (nsym-1-kc)*phr_symbol_samples
 *   the exact sample distance from the centre symbol's correlation WINDOW
 *   START to the payload s=0 correlation window start.  `phr_last_to_payload_samples`
 *   is therefore the distance from the LAST PHR symbol window start to the
 *   payload s=0 window start (1024 samples for the 512-chip PHR symbol at
 *   2 samples/chip, with no inter-field gap), not a chip/symbol count.
 *
 * The estimator is deliberately the plain lag-1 phase-difference average from
 * the task specification.  Extrapolating its per-sample slope over d samples
 * multiplies the slope error by d, so the residual CFO accuracy is limited by
 * the pilot SNR: with d ~ 11k samples, a 200 Hz requirement needs a
 * post-despread pilot SNR of roughly 50 dB per symbol.  A least-squares /
 * unwrapped phase regression would reach the same accuracy ~20 dB earlier and
 * is the recommended follow-up for Wave 2; it is not used here so that the
 * branch matches the specified algorithm bit-for-bit in behaviour.
 *
 * Defensive behaviour (all return applied = false, rot = rot_init and never
 * read or write out of bounds): null pointers, nsym < 4, non-finite or
 * non-positive phr_symbol_samples, non-finite phr_last_to_payload_samples,
 * non-finite min_coherence / max_delta_w_rad (or max_delta_w_rad <= 0),
 * non-finite rot_init with zero magnitude, and any non-finite input sample.
 *
 * ---------------------------------------------------------------------------
 * 2) Fractional-delay CIR template bank (8 phase bins)
 * ---------------------------------------------------------------------------
 *
 * `build_fractional_cir_bank()` resamples `fir_taps` (the matched-filter CIR,
 * h[q] = conj(CIR) in the existing convention) onto 8 sub-sample phases.
 * Phase convention: bin b corresponds to a fractional delay delta = b/phase_bins
 * and the output is
 *
 *     out_bank[b][m] = h(m - delta),   delta in [0, 1),
 *
 * i.e. the bank entry is DELAYED by +delta samples, matching the positive
 * offset convention of `decompose_sfo_position()` (int_pos + bin/phase_bins).
 * A received payload whose chips arrive at sample position q + delta is then
 * correlated with window origin q and taps out_bank[bin].
 *
 * bin 0 is a bit-exact copy of `fir_taps` (std::copy, no arithmetic), so the
 * integer grid keeps the verified zero-distortion behaviour.  Every bin is
 * L2-normalised to the same norm as `fir_taps` (skipped when that norm is 0),
 * so switching tabs never changes the soft-metric scale.
 *
 * Interpolation is a 4-tap Catmull-Rom cubic with zero padding outside
 * [0, tap_count); it is a pure function of the taps (no phase of its own).
 * The bank is contiguous: out_bank[b*tap_count + m].
 *
 * `decompose_sfo_position()` splits a fractional sample position into the
 * nearest sub-sample grid point int_pos + bin/phase_bins, rounding to NEAREST
 * (never truncating toward zero).  Negative positions floor correctly, e.g.
 * pos = -0.3 with 8 bins gives int_pos = -1, bin = 6 (-0.25, which is the
 * nearest grid point; -1 + 7/8 = -0.125 would be 0.175 samples away).
 * Non-finite input and phase_bins == 0 return int_pos = 0, bin = 0.
 */

#pragma once

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace openuwb {
namespace demod {
namespace rxopt {

struct PhrPhaseRefineResult {
    bool applied = false;
    std::complex<float> rot_at_payload_start{ 1.0f, 0.0f };
    double delta_cfo_w_rad = 0.0;
    float coherence = 0.0f;
};

PhrPhaseRefineResult refine_phase_cfo_from_phr(
    const std::complex<float>* phr_z_win,
    size_t nsym,
    const int8_t* phr_g1,
    int8_t sfd_polarity,
    std::complex<float> rot_init,
    double phr_symbol_samples,
    double phr_last_to_payload_samples,
    float min_coherence = 0.65f,
    double max_delta_w_rad = 2.5e-5);

// Number of sub-sample phases in the fractional CIR template bank.
inline constexpr size_t kSfoPhaseBins = 8;

// Build `phase_bins` CIR templates from `fir_taps`; the output is contiguous,
// out_bank[b*tap_count + m].  bin 0 is a bit-exact copy (integer grid
// untouched); bin b is delayed by +b/phase_bins samples using 4-tap
// Catmull-Rom interpolation with zero padding, then L2-normalised to the
// norm of `fir_taps` (skipped when that norm is zero).  Returns false for a
// null pointer, tap_count == 0, phase_bins == 0 (or an absurd bin count).
bool build_fractional_cir_bank(const std::complex<float>* fir_taps,
                               size_t tap_count,
                               size_t phase_bins,
                               std::complex<float>* out_bank);

// Nearest-sub-sample decomposition of a (possibly negative) fractional sample
// position: int_pos + bin/phase_bins rounds to the nearest 1/phase_bins grid
// point, with correct floor semantics for negative values.  Non-finite input
// or phase_bins == 0 yields (0, 0).
inline void decompose_sfo_position(double pos_samples,
                                   size_t phase_bins,
                                   int64_t& int_pos,
                                   size_t& bin)
{
    int_pos = 0;
    bin = 0;
    if (phase_bins == 0)
        return;
    if (!std::isfinite(pos_samples))
        return;
    const double p = static_cast<double>(phase_bins);
    // Keep pos*phase_bins inside int64 range (well inside 2^53 as well, so
    // the nearest-grid arithmetic below stays exact).
    if (std::abs(pos_samples) > 9.0e15 / p)
        return;

    const double q = std::floor(pos_samples * p + 0.5); // nearest grid index
    const double base = std::floor(q / p);              // floor, also for q<0
    const double frac = q - base * p;
    int_pos = static_cast<int64_t>(base);
    const double clamped = std::max(0.0, std::min(p - 1.0, frac));
    bin = static_cast<size_t>(clamped);
}

} // namespace rxopt
} // namespace demod
} // namespace openuwb
