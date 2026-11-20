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

inline PhrPhaseRefineResult refine_phase_cfo_from_phr(
    const std::complex<float>* phr_z_win,
    size_t nsym,
    const int8_t* phr_g1,
    int8_t sfd_polarity,
    std::complex<float> rot_init,
    double phr_symbol_samples,
    double phr_last_to_payload_samples,
    float min_coherence = 0.65f,
    double max_delta_w_rad = 2.5e-5)
{
    PhrPhaseRefineResult out;

    // --- Argument / value guards -------------------------------------------
    const bool rot_finite =
        std::isfinite(rot_init.real()) && std::isfinite(rot_init.imag());
    const double rot_mag = rot_finite ? static_cast<double>(std::abs(rot_init))
                                      : 0.0;
    const bool rot_usable = rot_finite && std::isfinite(rot_mag) &&
                            rot_mag > 1e-12;
    if (rot_usable)
        out.rot_at_payload_start = rot_init;
    else
        out.rot_at_payload_start = std::complex<float>(1.0f, 0.0f);

    if (phr_z_win == nullptr || phr_g1 == nullptr)
        return out;
    if (nsym < 4)
        return out;
    if (!std::isfinite(phr_symbol_samples) || phr_symbol_samples <= 0.0)
        return out;
    if (!std::isfinite(phr_last_to_payload_samples))
        return out;
    if (!std::isfinite(min_coherence))
        return out;
    if (!std::isfinite(max_delta_w_rad) || max_delta_w_rad <= 0.0)
        return out;
    if (!rot_usable)
        return out;

    const double rr = static_cast<double>(rot_init.real()) / rot_mag;
    const double ri = static_cast<double>(rot_init.imag()) / rot_mag;

    // tilde[k] = (1-2*g1)*pol*z, expressed in the rot_init frame:
    //   u[k] = rot_init_normalised * tilde[k].
    // Recomputing it per pass avoids any allocation in this per-packet helper.
    auto tilde_at = [&](size_t k, std::complex<double>& u) -> bool {
        const double zr = static_cast<double>(phr_z_win[k].real());
        const double zi = static_cast<double>(phr_z_win[k].imag());
        if (!std::isfinite(zr) || !std::isfinite(zi))
            return false;
        const int sign_bit = 1 - 2 * static_cast<int>(phr_g1[k]);
        const double sgn = static_cast<double>(sign_bit) *
                           static_cast<double>(sfd_polarity);
        const double tr = sgn * zr;
        const double ti = sgn * zi;
        if (!std::isfinite(tr) || !std::isfinite(ti))
            return false;
        u = std::complex<double>(rr * tr - ri * ti, ri * tr + rr * ti);
        return std::isfinite(u.real()) && std::isfinite(u.imag());
    };

    // --- Pass 1: coherence and lag-1 residual CFO --------------------------
    constexpr double kEps = 1e-30;
    std::complex<double> sum(0.0, 0.0);
    std::complex<double> lag(0.0, 0.0);
    double abs_sum = 0.0;
    std::complex<double> prev(0.0, 0.0);
    for (size_t k = 0; k < nsym; ++k) {
        std::complex<double> u;
        if (!tilde_at(k, u))
            return out;
        // |u| == |tilde| because the frame rotation has unit modulus.
        sum += u;
        abs_sum += std::abs(u);
        if (k > 0)
            lag += u * std::conj(prev);
        prev = u;
    }
    const double coherence = std::abs(sum) / (abs_sum + kEps);
    out.coherence = static_cast<float>(coherence);
    if (coherence < static_cast<double>(min_coherence))
        return out; // applied = false, rot = rot_init, delta = 0

    const double dtheta = std::atan2(lag.imag(), lag.real());
    double delta = dtheta / phr_symbol_samples;
    delta = std::max(-max_delta_w_rad, std::min(max_delta_w_rad, delta));
    if (!std::isfinite(delta))
        return out;

    // --- Pass 2: mean phase at the centre symbol, extrapolated to payload ---
    const double kc = 0.5 * static_cast<double>(nsym - 1);
    std::complex<double> acc(0.0, 0.0);
    for (size_t k = 0; k < nsym; ++k) {
        std::complex<double> u;
        if (!tilde_at(k, u))
            return out;
        const double ang = -delta *
                           (static_cast<double>(k) - kc) *
                           phr_symbol_samples;
        acc += u * std::complex<double>(std::cos(ang), std::sin(ang));
    }
    const double theta_c = std::atan2(acc.imag(), acc.real());
    const double d =
        phr_last_to_payload_samples +
        (static_cast<double>(nsym - 1) - kc) * phr_symbol_samples;
    const double theta_payload = theta_c + delta * d;
    if (!std::isfinite(theta_c) || !std::isfinite(theta_payload))
        return out;

    // rot = rot_init * exp(-j*theta_payload) in the unit-modulus frame.
    const double cr = std::cos(-theta_payload);
    const double ci = std::sin(-theta_payload);
    out.rot_at_payload_start = std::complex<float>(
        static_cast<float>(rr * cr - ri * ci),
        static_cast<float>(ri * cr + rr * ci));
    out.delta_cfo_w_rad = delta;
    out.applied = true;
    return out;
}

// Number of sub-sample phases in the fractional CIR template bank.
inline constexpr size_t kSfoPhaseBins = 8;

// Build `phase_bins` CIR templates from `fir_taps`; the output is contiguous,
// out_bank[b*tap_count + m].  bin 0 is a bit-exact copy (integer grid
// untouched); bin b is delayed by +b/phase_bins samples using 4-tap
// Catmull-Rom interpolation with zero padding, then L2-normalised to the
// norm of `fir_taps` (skipped when that norm is zero).  Returns false for a
// null pointer, tap_count == 0, phase_bins == 0 (or an absurd bin count).
inline bool build_fractional_cir_bank(const std::complex<float>* fir_taps,
                                      size_t tap_count,
                                      size_t phase_bins,
                                      std::complex<float>* out_bank)
{
    if (fir_taps == nullptr || out_bank == nullptr)
        return false;
    if (tap_count == 0 || phase_bins == 0)
        return false;
    // Sanity bound: keeps tap_count*phase_bins far away from size_t overflow.
    if (phase_bins > (static_cast<size_t>(1) << 16))
        return false;

    // bin 0: bit-exact integer grid.
    std::copy(fir_taps, fir_taps + tap_count, out_bank);
    if (phase_bins == 1)
        return true;

    double ref_norm_sq = 0.0;
    for (size_t m = 0; m < tap_count; ++m) {
        const double re = static_cast<double>(fir_taps[m].real());
        const double im = static_cast<double>(fir_taps[m].imag());
        ref_norm_sq += re * re + im * im;
    }
    const double ref_norm = std::sqrt(ref_norm_sq);

    auto tap_value = [&](long long idx) -> std::complex<double> {
        if (idx < 0 || idx >= static_cast<long long>(tap_count))
            return std::complex<double>(0.0, 0.0);
        const std::complex<float>& v = fir_taps[static_cast<size_t>(idx)];
        return std::complex<double>(static_cast<double>(v.real()),
                                    static_cast<double>(v.imag()));
    };

    for (size_t b = 1; b < phase_bins; ++b) {
        const double delta =
            static_cast<double>(b) / static_cast<double>(phase_bins);
        std::complex<float>* dst = out_bank + b * tap_count;
        double norm_sq = 0.0;
        for (size_t m = 0; m < tap_count; ++m) {
            // Catmull-Rom interpolation of h at x = m - delta: the template
            // peak therefore moves by +delta samples.
            const double x = static_cast<double>(m) - delta;
            const double fl = std::floor(x);
            const long long i = static_cast<long long>(fl);
            const double t = x - fl;
            const double t2 = t * t;
            const double t3 = t2 * t;
            const double w0 = -0.5 * t3 + t2 - 0.5 * t;
            const double w1 = 1.5 * t3 - 2.5 * t2 + 1.0;
            const double w2 = -1.5 * t3 + 2.0 * t2 + 0.5 * t;
            const double w3 = 0.5 * t3 - 0.5 * t2;
            const std::complex<double> v =
                w0 * tap_value(i - 1) + w1 * tap_value(i) +
                w2 * tap_value(i + 1) + w3 * tap_value(i + 2);
            dst[m] = std::complex<float>(static_cast<float>(v.real()),
                                         static_cast<float>(v.imag()));
            const double re = static_cast<double>(dst[m].real());
            const double im = static_cast<double>(dst[m].imag());
            norm_sq += re * re + im * im;
        }
        const double norm = std::sqrt(norm_sq);
        if (ref_norm > 0.0 && norm > 0.0) {
            const float scale = static_cast<float>(ref_norm / norm);
            for (size_t m = 0; m < tap_count; ++m)
                dst[m] *= scale;
        }
    }
    return true;
}

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
