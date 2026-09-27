/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Header-only monostatic-radar CIR estimator at 998.4 MS/s.
 *
 * Ports MATLAB +uwbdecoder/estimateCir.m and testdata/uwb_radar
 * local_estimate_cir (0-based).  Input is a known preamble origin; output
 * is complex raw CIR plus L2-normalized CIR.  Not a GNU Radio block.
 */

#pragma once

#include <gnuradio/uwb/uwb_phy_profile.h>
#include <gnuradio/uwb/uwb_detector_core.h>
#include <gnuradio/uwb/uwb_radar_timing_core.h>

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdint>
#include <limits>
#include <vector>

// AVX2/FMA codegen policy (phase 4a): the AVX2 kernels live in this
// header and are compiled via per-function __attribute__((target)), so
// even a baseline (no -mavx2/-mfma flags) build emits the accelerated
// code path; every call site gates at runtime via UWB_RADAR_CIR_CPU_AVX2()
// (scalar fallback for hosts lacking AVX2/FMA).  The module is never
// built with -march=native.
// The AVX2 kernels below need the intrinsic declarations in scope in
// every build (baseline too): uwb_detector_core.h includes <immintrin.h>
// only when __AVX2__ is predefined, so include it locally here.
#if (defined(__x86_64__) || defined(__i386__)) && \
    !defined(_IMMINTRIN_H_INCLUDED)
#include <immintrin.h>
#endif

// Runtime AVX2/FMA check for the per-function-target-attribute kernels.
// x86 only: __builtin_cpu_supports is an x86 GCC/clang builtin; other
// architectures take the scalar mainline unconditionally.
#if defined(__x86_64__) || defined(__i386__)
#define UWB_RADAR_CIR_CPU_AVX2() \
    (__builtin_cpu_supports("avx2") && __builtin_cpu_supports("fma"))
#else
#define UWB_RADAR_CIR_CPU_AVX2() 0
#endif

namespace gr {
namespace uwb {
namespace radar {

enum class CirStatus : uint8_t { Ok = 0, CirFailed = 1, InvalidInput = 2 };

struct RadarCirEstimate {
    CirStatus status = CirStatus::CirFailed;
    size_t tap_count = 0;
    size_t peak_tap = 0;     // 0-based on CIR grid
    float peak_abs = 0.f;    // |raw| at peak
    float raw_l2_norm = 0.f;
    size_t valid_repetitions = 0;
    size_t first_repetition = 0;
};

struct RadarCirScratch {
    std::vector<std::complex<float>> sampled_code; // 1016
    // Non-zero sampled-code entries in ascending sample order.  HRP codes
    // contain one non-zero chip sample followed by seven exact zeros, so
    // per-repetition CIR must not rescan all 1016 entries for every tap.
    // Prepared once; the hot path only reads these vectors.
    std::vector<size_t> active_code_indices;
    std::vector<std::complex<float>> active_code_values;
    float code_energy = 0.f;
    std::vector<std::complex<float>> avg;          // capacity >= wlen
    // Double-precision accumulator for estimate_radar_cir_from_repetitions
    // (adds each valid repetition window, then scales by 1/N once).
    std::vector<std::complex<double>> avg_rep_sum; // capacity >= wlen
    std::vector<std::complex<float>> raw_taps;     // capacity >= tap_count
    std::vector<std::complex<float>> norm_taps;    // capacity >= tap_count

    // AVX2 kernel plan (fixed per prepare; hot path is read-only): the
    // broadcast float weights of the active code chips in
    // active_code_indices order.  The HRP code is real {-1,0,+1}, so the
    // per-chip weight is one plain float broadcast, no mask lanes.
    std::vector<float> wt_active;                  // reserve(code_len)

    // May allocate. Call from prepare, never from estimate_radar_cir.
    void reserve(size_t code_len, size_t max_taps)
    {
        sampled_code.reserve(code_len);
        active_code_indices.reserve(code_len);
        active_code_values.reserve(code_len);
        wt_active.reserve(code_len);
        const size_t max_wlen = code_len + max_taps - 1;
        avg.reserve(max_wlen);
        avg_rep_sum.reserve(max_wlen);
        raw_taps.reserve(max_taps);
        norm_taps.reserve(max_taps);
    }
};

namespace detail {

inline bool cir_add_overflow(size_t a, size_t b)
{
    return b > std::numeric_limits<size_t>::max() - a;
}

inline void cir_fail(RadarCirEstimate& out, CirStatus status)
{
    out = RadarCirEstimate{};
    out.status = status;
}

inline bool cir_fail_status(RadarCirEstimate& out, CirStatus status)
{
    cir_fail(out, status);
    return false;
}

// ---------------------------------------------------------------------------
// Detail: AVX2/FMA sparse CIR kernels
//
// Lane plan (floats): a std::complex<float> is 2 packed floats [re,im], so
// one __m256 holds 4 complex samples; the correlation processes 4 adjacent
// taps per 256-bit window load over the active (non-zero) code chips.
// ---------------------------------------------------------------------------

// AVX2 kernels are compiled into EVERY x86 build (the per-function
// target("avx2,fma") attribute overrides the command-line ISA baseline);
// callers gate them at runtime via UWB_RADAR_CIR_CPU_AVX2().  Non-x86
// targets keep the scalar mainline only.
#if (defined(__x86_64__) || defined(__i386__)) && defined(__GNUC__)
#define UWB_RADAR_CIR_HAVE_AVX2 1
#else
#define UWB_RADAR_CIR_HAVE_AVX2 0
#endif

#if UWB_RADAR_CIR_HAVE_AVX2

// Accumulate one repetition window into the float average buffer: 4 complex
// (16 floats) per step.  Per-element add order equals the scalar
// "avg[m] += src[m]" loop (lanes are independent, no reduction reorder),
// so the averaged window is bit-identical to the scalar mainline.
__attribute__((target("avx2,fma"))) inline void
cir_avg_window_avx2(std::complex<float>* avg,
                    const std::complex<float>* src,
                    size_t wlen)
{
    float* dst = reinterpret_cast<float*>(avg);
    const float* s = reinterpret_cast<const float*>(src);
    size_t m = 0;
    for (; m + 4 <= wlen; m += 4)
        _mm256_storeu_ps(
            dst + 2 * m,
            _mm256_add_ps(_mm256_loadu_ps(dst + 2 * m),
                          _mm256_loadu_ps(s + 2 * m)));
    for (; m < wlen; ++m) {
        avg[m].real(avg[m].real() + src[m].real());
        avg[m].imag(avg[m].imag() + src[m].imag());
    }
}

// In-place scale of the float average window by 1/valid: 4 complex per step.
__attribute__((target("avx2,fma"))) inline void
cir_scale_window_avx2(std::complex<float>* avg, size_t wlen, float inv)
{
    float* dst = reinterpret_cast<float*>(avg);
    const __m256 iv = _mm256_set1_ps(inv);
    size_t m = 0;
    for (; m + 4 <= wlen; m += 4)
        _mm256_storeu_ps(dst + 2 * m,
                         _mm256_mul_ps(_mm256_loadu_ps(dst + 2 * m), iv));
    for (; m < wlen; ++m)
        avg[m] *= inv;
}

// One 4-tap group of the sparse active-chip correlation.  tar0 points at
// the tap-0 window sample base (tap row n loads tar0 + n).  The packed
// result is [re0,im0,re1,im1,re2,im2,re3,im3] for taps n0..n0+3.
__attribute__((target("avx2,fma"))) inline void
cir_corr4_kernel_avx2(const std::complex<float>* tar0,
                      const size_t* m_active,
                      const float* wt_active,
                      size_t n_active,
                      float* out,
                      float inv_energy)
{
    __m256 acc = _mm256_setzero_ps();
    for (size_t j = 0; j < n_active; ++j) {
        const __m256 x = _mm256_loadu_ps(
            reinterpret_cast<const float*>(tar0 + m_active[j]));
        acc = _mm256_fmadd_ps(x, _mm256_set1_ps(wt_active[j]), acc);
    }
    _mm256_storeu_ps(out, _mm256_mul_ps(acc, _mm256_set1_ps(inv_energy)));
}

// Per-repetition double avoidance: accumulate the float window complex
// sample (float)->(double) into the double rep-sum buffer (2 doubles per
// complex).  Lane plan: row0 = re[m..m+3] (pd), row1 = im (pd).  Two
// 4-complex (4-pd) rows per step.
__attribute__((target("avx2,fma"))) inline void
cir_avg_window_rep_sum_avx2(std::complex<double>* rep_sum,
                            const std::complex<float>* src,
                            size_t wlen)
{
    double* dst = reinterpret_cast<double*>(rep_sum);
    size_t m = 0;
    for (; m + 2 <= wlen; m += 2) {
        // Two complex samples per step: src float pair [re0,im0,re1,im1]
        // converts straight to the dst pair's double layout
        // [re0,im0,re1,im1] (cvtps2pd widens each float lane in order).
        // dst complex index 2m..2m+1 = doubles [4m .. 4m+3].
        const __m128 x = _mm_loadu_ps(
            reinterpret_cast<const float*>(src + m));
        const __m256d r = _mm256_loadu_pd(dst + 2 * m);
        const __m256d conv = _mm256_cvtps_pd(x);
        _mm256_storeu_pd(dst + 2 * m, _mm256_add_pd(r, conv));
    }
    for (; m < wlen; ++m) {
        rep_sum[m].real(rep_sum[m].real() + double(src[m].real()));
        rep_sum[m].imag(rep_sum[m].imag() + double(src[m].imag()));
    }
}

// Convert the (already-summed) double rep-sum window to a float window
// with the 1/valid scale: avg_f[m] = (float)(rep_sum[m] * inv_valid).
// Used by the AVX2 from_repetitions path, which computes correlation on
// the reduced float window with corr4 (float FMA).  2 complex per step.
__attribute__((target("avx2,fma"))) inline void
cir_rep_sum_to_float_window_avx2(const std::complex<double>* rep_sum,
                                 std::complex<float>* avg_f,
                                 size_t wlen,
                                 double inv_valid)
{
    const float inv = static_cast<float>(inv_valid);
    size_t m = 0;
    for (; m + 2 <= wlen; m += 2) {
        const __m256d r = _mm256_loadu_pd(
            reinterpret_cast<const double*>(rep_sum + m));
        // even lanes = re, odd = im (lo 4 doubles = 2 complex)
        const __m256d rf = _mm256_mul_pd(r, _mm256_set1_pd(inv));
        const __m128 f = _mm256_cvtpd_ps(rf); // [re0,im0,re1,im1]
        _mm_storeu_ps(reinterpret_cast<float*>(avg_f + m), f);
    }
    for (; m < wlen; ++m) {
        avg_f[m] = std::complex<float>(
            static_cast<float>(rep_sum[m].real()) * inv,
            static_cast<float>(rep_sum[m].imag()) * inv);
    }
}

#endif

} // namespace detail

// May allocate. Builds 1016 sampled_code + code_energy; sizes avg/raw/norm for max_pre+max_post.
inline bool prepare_radar_cir_code(const int8_t* hrp_code,
                                   size_t code_len,
                                   size_t max_pre,
                                   size_t max_post,
                                   RadarCirScratch& scratch)
{
    scratch.sampled_code.clear();
    scratch.active_code_indices.clear();
    scratch.active_code_values.clear();
    scratch.code_energy = 0.f;

    if (!hrp_code || code_len == 0)
        return false;
    if (demod::kQm35SpreadingFactor > 0 &&
        code_len > std::numeric_limits<size_t>::max() / demod::kQm35SpreadingFactor)
        return false;
    if (detail::cir_add_overflow(max_pre, max_post))
        return false;

    const std::vector<int8_t> spread =
        demod::BuildSampledCode(hrp_code, code_len);
    if (spread.empty())
        return false;
    if (demod::kQm35SamplesPerChip > 0 &&
        spread.size() >
            std::numeric_limits<size_t>::max() / demod::kQm35SamplesPerChip)
        return false;

    const size_t sampled_len = spread.size() * demod::kQm35SamplesPerChip;
    const size_t max_taps = max_pre + max_post;
    size_t max_wlen = 0;
    if (max_taps == 0) {
        if (sampled_len == 0)
            return false;
        max_wlen = sampled_len - 1;
    } else {
        if (sampled_len > std::numeric_limits<size_t>::max() - (max_taps - 1))
            return false;
        max_wlen = sampled_len + max_taps - 1;
    }

    scratch.reserve(sampled_len, max_taps);
    scratch.sampled_code.assign(sampled_len, std::complex<float>(0.f, 0.f));
    for (size_t j = 0; j < spread.size(); ++j) {
        const float v = static_cast<float>(spread[j]);
        if (!std::isfinite(v)) {
            scratch.sampled_code.clear();
            scratch.code_energy = 0.f;
            return false;
        }
        scratch.sampled_code[j * demod::kQm35SamplesPerChip] =
            std::complex<float>(v, 0.f);
    }

    float energy = 0.f;
    for (size_t m = 0; m < sampled_len; ++m) {
        energy += std::norm(scratch.sampled_code[m]);
        if (scratch.sampled_code[m] != std::complex<float>(0.f, 0.f)) {
            scratch.active_code_indices.push_back(m);
            scratch.active_code_values.push_back(scratch.sampled_code[m]);
        }
    }
    if (!(energy > 0.f) || !std::isfinite(energy)) {
        scratch.sampled_code.clear();
        scratch.active_code_indices.clear();
        scratch.active_code_values.clear();
        scratch.avg_rep_sum.clear();
        scratch.code_energy = 0.f;
        return false;
    }
    scratch.code_energy = energy;

    // AVX2 kernel plan: extract the broadcast weights of the active chips
    // once (descending |weight| not required; index order is ascending).
    scratch.wt_active.clear();
    scratch.wt_active.reserve(scratch.active_code_values.size());
    for (const auto& cv : scratch.active_code_values)
        scratch.wt_active.push_back(cv.real());

    scratch.avg.assign(max_wlen, std::complex<float>(0.f, 0.f));
    scratch.avg_rep_sum.assign(max_wlen, std::complex<double>(0.0, 0.0));
    scratch.raw_taps.assign(max_taps, std::complex<float>(0.f, 0.f));
    scratch.norm_taps.assign(max_taps, std::complex<float>(0.f, 0.f));
    return true;
}

// HOT PATH: no alloc. Taps written to scratch.raw_taps/norm_taps [0, tap_count).
// On failure: out.tap_count=0; do not present leftover taps as a result.
inline bool estimate_radar_cir(const std::complex<float>* rx,
                               size_t n,
                               int64_t preamble_start,
                               size_t samples_per_symbol,
                               size_t pre,
                               size_t post,
                               size_t skip_initial,
                               size_t max_repetitions,
                               size_t preamble_repetitions,
                               RadarCirEstimate& out,
                               RadarCirScratch& scratch)
{
    detail::cir_fail(out, CirStatus::InvalidInput);

    if (!rx || n == 0 || preamble_start < 0 || samples_per_symbol == 0 ||
        scratch.sampled_code.empty() || !(scratch.code_energy > 0.f) ||
        !std::isfinite(scratch.code_energy))
        return false;
    if (detail::cir_add_overflow(pre, post))
        return false;

    const size_t tap_count = pre + post;
    if (tap_count == 0)
        return false;
    if (tap_count > scratch.raw_taps.size() ||
        tap_count > scratch.norm_taps.size() ||
        tap_count > scratch.raw_taps.capacity() ||
        tap_count > scratch.norm_taps.capacity())
        return false;

    const size_t code_len = scratch.sampled_code.size();
    if (code_len > std::numeric_limits<size_t>::max() - (tap_count - 1))
        return false;
    const size_t wlen = code_len + tap_count - 1;
    if (wlen > scratch.avg.size() || wlen > scratch.avg.capacity())
        return false;

    const size_t available = preamble_repetitions;
    if (skip_initial >= available)
        return false;

    const size_t count =
        std::min(max_repetitions, available - skip_initial);

    std::fill(scratch.avg.begin(),
              scratch.avg.begin() + static_cast<std::ptrdiff_t>(wlen),
              std::complex<float>(0.f, 0.f));

    int64_t n64 = 0;
    int64_t pre64 = 0;
    int64_t wlen64 = 0;
    int64_t period64 = 0;
    int64_t last_ok = 0;
    if (!radar_i64_from_size(n, n64) || !radar_i64_from_size(pre, pre64) ||
        !radar_i64_from_size(wlen, wlen64) ||
        !radar_i64_from_size(samples_per_symbol, period64) ||
        !radar_i64_sub(n64, wlen64, last_ok))
        return false;

    size_t valid = 0;
    for (size_t k = skip_initial; k < skip_initial + count; ++k) {
        int64_t k64 = 0;
        int64_t offset = 0;
        int64_t rs = 0;
        int64_t lo = 0;
        if (!radar_i64_from_size(k, k64) ||
            !radar_i64_mul(k64, period64, offset) ||
            !radar_i64_add(preamble_start, offset, rs) ||
            !radar_i64_sub(rs, pre64, lo))
            return false;
        if (lo < 0 || lo > last_ok)
            continue;
        const std::complex<float>* src = rx + static_cast<size_t>(lo);
#if UWB_RADAR_CIR_HAVE_AVX2
        if (UWB_RADAR_CIR_CPU_AVX2())
            detail::cir_avg_window_avx2(scratch.avg.data(), src, wlen);
        else
#endif
        {
            for (size_t m = 0; m < wlen; ++m)
                scratch.avg[m] += src[m];
        }
        ++valid;
    }
    if (valid == 0)
        return detail::cir_fail_status(out, CirStatus::CirFailed);

    const float inv_valid = 1.f / static_cast<float>(valid);
#if UWB_RADAR_CIR_HAVE_AVX2
    if (UWB_RADAR_CIR_CPU_AVX2())
        detail::cir_scale_window_avx2(scratch.avg.data(), wlen, inv_valid);
    else
#endif
        for (size_t m = 0; m < wlen; ++m)
            scratch.avg[m] *= inv_valid;

    // Correlate on the prepared non-zero code chips (active_code_indices/
    // values) instead of scanning all code_len sampled-code entries.  HRP
    // sampled codes hold one non-zero chip per 8 samples (64 active of
    // 1016), so the dropped entries are exact-zero terms whose removal
    // does not change the mathematical value; the double-summation order
    // changes, so results may differ from the dense scan by float rounding
    // only (golden tolerance covers this; see Phase-1 point 3).
    const double energy_d = static_cast<double>(scratch.code_energy);
    size_t peak_tap = 0;
    float peak_abs = -1.f;
    double nrm2 = 0.0;
    const size_t n_active = scratch.active_code_indices.size();
#if UWB_RADAR_CIR_HAVE_AVX2
    if (UWB_RADAR_CIR_CPU_AVX2()) {
        // AVX2: 4 adjacent taps share one 256-bit unaligned window load.
        // Accumulation is float FMA (error ~1e-8, harmless under the
        // golden L2 budget and smaller than the float tap precision).
        const size_t* const ci = scratch.active_code_indices.data();
        const float* const cw = scratch.wt_active.data();
        const size_t groups = tap_count / 4;
        const float inv_energy = static_cast<float>(1.0 / energy_d);
        for (size_t g = 0; g < groups; ++g) {
            detail::cir_corr4_kernel_avx2(
                scratch.avg.data() + static_cast<std::ptrdiff_t>(4 * g), ci,
                cw, n_active,
                reinterpret_cast<float*>(scratch.raw_taps.data() + 4 * g),
                inv_energy);
        }
        const size_t rem = tap_count - 4 * groups;
        for (size_t t = 0; t < rem; ++t) {
            const size_t tap0 = 4 * groups + t;
            double acc_re = 0.0;
            double acc_im = 0.0;
            for (size_t j = 0; j < n_active; ++j) {
                const size_t m = ci[j];
                const auto c = scratch.active_code_values[j];
                const auto a = scratch.avg[tap0 + m];
                const double cr = static_cast<double>(c.real());
                const double ci2 = static_cast<double>(c.imag());
                acc_re += static_cast<double>(a.real()) * cr +
                          static_cast<double>(a.imag()) * ci2;
                acc_im += static_cast<double>(a.imag()) * cr -
                          static_cast<double>(a.real()) * ci2;
            }
            scratch.raw_taps[tap0] = std::complex<float>(
                static_cast<float>(acc_re / energy_d),
                static_cast<float>(acc_im / energy_d));
        }
    } else
#endif
    {
        // Scalar mainline (phase-2 mainline): kept byte-for-byte so the
        // fallback path is bit-identical to the pre-phase-4a results.
        for (size_t nn = 0; nn < tap_count; ++nn) {
            double acc_re = 0.0;
            double acc_im = 0.0;
            for (size_t j = 0; j < n_active; ++j) {
                const size_t m = scratch.active_code_indices[j];
                const auto c = scratch.active_code_values[j];
                const auto a = scratch.avg[nn + m];
                const double cr = static_cast<double>(c.real());
                const double ci = static_cast<double>(c.imag());
                acc_re += static_cast<double>(a.real()) * cr +
                          static_cast<double>(a.imag()) * ci;
                acc_im += static_cast<double>(a.imag()) * cr -
                          static_cast<double>(a.real()) * ci;
            }
            const std::complex<float> raw(
                static_cast<float>(acc_re / energy_d),
                static_cast<float>(acc_im / energy_d));
            scratch.raw_taps[nn] = raw;
        }
    }
    // Norm + peak metric generation: scalar, double accumulation (shared
    // by both kernels so peak_tap / raw_l2_norm are bit-identical).
    for (size_t nn = 0; nn < tap_count; ++nn) {
        nrm2 += static_cast<double>(std::norm(scratch.raw_taps[nn]));
        const float mag = std::abs(scratch.raw_taps[nn]);
        if (mag > peak_abs) {
            peak_abs = mag;
            peak_tap = nn;
        }
    }

    const float nrm = static_cast<float>(std::sqrt(nrm2));
    if (!std::isfinite(nrm) || !(nrm > 0.f) || !std::isfinite(peak_abs))
        return detail::cir_fail_status(out, CirStatus::CirFailed);

    const float inv_n = 1.f / (nrm + 1e-12f);
    for (size_t i = 0; i < tap_count; ++i)
        scratch.norm_taps[i] = scratch.raw_taps[i] * inv_n;

    out.status = CirStatus::Ok;
    out.tap_count = tap_count;
    out.peak_tap = peak_tap;
    out.peak_abs = peak_abs;
    out.raw_l2_norm = nrm;
    out.valid_repetitions = valid;
    out.first_repetition = skip_initial;
    return true;
}

// HOT PATH: average-mode CIR assembled from the per-repetition kernel.
// Numerically this is mean(rep taps): the coherent sum over the same
// repetitions is only scaled by 1/N at the end, and the correlation already
// runs on the sparse non-zero code chips (active_code_indices/values).
// Compared with estimate_radar_cir the floating-point order differs (N
// per-repetition accumulations instead of one correlated accumulation over
// the coherent mean window); exact equality with estimate_radar_cir is NOT
// required or provided.
//
// Repetition loop is identical to estimate_radar_cir: absolute repetition
// k = skip..skip+count-1, out-of-window / negative repetitions are skipped,
// and out.valid_repetitions counts the ones used.  raw_taps of the sum stay
// in double precision; scratch.raw_taps is not touched until the final
// result (single-rep callers must copy their taps before calling this).
inline bool estimate_radar_cir_from_repetitions(
    const std::complex<float>* rx,
    size_t n,
    int64_t preamble_start,
    size_t samples_per_symbol,
    size_t pre,
    size_t post,
    size_t skip_initial,
    size_t max_repetitions,
    size_t preamble_repetitions,
    RadarCirEstimate& out,
    RadarCirScratch& scratch)
{
    detail::cir_fail(out, CirStatus::InvalidInput);
    if (!rx || n == 0 || preamble_start < 0 || samples_per_symbol == 0 ||
        scratch.sampled_code.empty() || !(scratch.code_energy > 0.f) ||
        !std::isfinite(scratch.code_energy) ||
        scratch.avg_rep_sum.empty() ||
        scratch.active_code_indices.empty() ||
        scratch.active_code_indices.size() !=
            scratch.active_code_values.size() ||
        detail::cir_add_overflow(pre, post))
        return false;

    const size_t tap_count = pre + post;
    if (tap_count == 0 || tap_count > scratch.raw_taps.size() ||
        tap_count > scratch.norm_taps.size())
        return false;
    const size_t code_len = scratch.sampled_code.size();
    if (code_len > std::numeric_limits<size_t>::max() - (tap_count - 1))
        return false;
    const size_t wlen = code_len + tap_count - 1;
    if (wlen > scratch.avg_rep_sum.size() ||
        wlen > scratch.avg_rep_sum.capacity())
        return false;

    const size_t available = preamble_repetitions;
    if (skip_initial >= available)
        return false;
    const size_t count = std::min(max_repetitions, available - skip_initial);

    int64_t n64 = 0;
    int64_t pre64 = 0;
    int64_t wlen64 = 0;
    int64_t period64 = 0;
    int64_t last_ok = 0;
    if (!radar_i64_from_size(n, n64) || !radar_i64_from_size(pre, pre64) ||
        !radar_i64_from_size(wlen, wlen64) ||
        !radar_i64_from_size(samples_per_symbol, period64) ||
        !radar_i64_sub(n64, wlen64, last_ok))
        return false;

    // One fixed accumulation buffer, zeroed per call (fill only wlen).
    std::fill(scratch.avg_rep_sum.begin(),
              scratch.avg_rep_sum.begin() + static_cast<std::ptrdiff_t>(wlen),
              std::complex<double>(0.0, 0.0));

    // Add each repetition's window into the double accumulator.  The
    // ordering matches estimate_radar_cir (repetitions ascending, samples
    // ascending); sparse code chips are applied in the final correlation.
    size_t valid = 0;
    for (size_t k = skip_initial; k < skip_initial + count; ++k) {
        int64_t k64 = 0;
        int64_t offset = 0;
        int64_t rs = 0;
        int64_t lo = 0;
        if (!radar_i64_from_size(k, k64) ||
            !radar_i64_mul(k64, period64, offset) ||
            !radar_i64_add(preamble_start, offset, rs) ||
            !radar_i64_sub(rs, pre64, lo))
            return false;
        if (lo < 0 || lo > last_ok)
            continue;
        const std::complex<float>* src = rx + static_cast<size_t>(lo);
#if UWB_RADAR_CIR_HAVE_AVX2
        if (UWB_RADAR_CIR_CPU_AVX2()) {
            detail::cir_avg_window_rep_sum_avx2(scratch.avg_rep_sum.data(),
                                                src, wlen);
        } else
#endif
        {
            for (size_t m = 0; m < wlen; ++m) {
                const std::complex<float> s = src[m];
                scratch.avg_rep_sum[m] +=
                    std::complex<double>(static_cast<double>(s.real()),
                                         static_cast<double>(s.imag()));
            }
        }
        ++valid;
    }
    if (valid == 0)
        return detail::cir_fail_status(out, CirStatus::CirFailed);

    const double inv_valid = 1.0 / static_cast<double>(valid);
    const double energy = static_cast<double>(scratch.code_energy);
    size_t peak_tap = 0;
    float peak_abs = -1.f;
    double nrm2 = 0.0;
#if UWB_RADAR_CIR_HAVE_AVX2
    if (UWB_RADAR_CIR_CPU_AVX2()) {
        // AVX2: convert the double sum to float in-place (with the SAME
        // 1/valid scaling as the scalar path), then use the 4-tap corr4
        // float FMA kernel (average-mode correction: taps = avg - re-pivoted
        // code weights).  The conversion kernel is exactly: avg_f[m] =
        // (float)(avg_rep_sum[m] * inv_valid) elementwise.
        detail::cir_rep_sum_to_float_window_avx2(scratch.avg_rep_sum.data(),
                                                 scratch.avg.data(), wlen,
                                                 inv_valid);
        const size_t* const ci = scratch.active_code_indices.data();
        const float* const cw = scratch.wt_active.data();
        const size_t groups = tap_count / 4;
        const float inv_energy_f = static_cast<float>(1.0 / energy);
        for (size_t g = 0; g < groups; ++g) {
            detail::cir_corr4_kernel_avx2(
                scratch.avg.data() + static_cast<std::ptrdiff_t>(4 * g),
                ci, cw, scratch.active_code_indices.size(),
                reinterpret_cast<float*>(scratch.raw_taps.data() + 4 * g),
                inv_energy_f);
        }
        const size_t rem = tap_count - 4 * groups;
        for (size_t t = 0; t < rem; ++t) {
            const size_t tap0 = 4 * groups + t;
            double acc_re = 0.0;
            double acc_im = 0.0;
            for (size_t j = 0; j < scratch.active_code_indices.size(); ++j) {
                const size_t m = scratch.active_code_indices[j];
                const auto c = scratch.active_code_values[j];
                const auto a = scratch.avg[tap0 + m];
                const double cr = static_cast<double>(c.real());
                const double ci2 = static_cast<double>(c.imag());
                acc_re += (static_cast<double>(a.real()) * cr +
                           static_cast<double>(a.imag()) * ci2);
                acc_im += (static_cast<double>(a.imag()) * cr -
                           static_cast<double>(a.real()) * ci2);
            }
            scratch.raw_taps[tap0] = std::complex<float>(
                static_cast<float>(acc_re * inv_valid / energy),
                static_cast<float>(acc_im * inv_valid / energy));
        }
    } else
#endif
    {
        for (size_t nn = 0; nn < tap_count; ++nn) {
            double acc_re = 0.0;
            double acc_im = 0.0;
            for (size_t j = 0; j < scratch.active_code_indices.size(); ++j) {
                const size_t m = scratch.active_code_indices[j];
                const auto c = scratch.active_code_values[j];
                const auto a = scratch.avg_rep_sum[nn + m];
                const double cr = static_cast<double>(c.real());
                const double ci = static_cast<double>(c.imag());
                acc_re += (static_cast<double>(a.real()) * cr +
                           static_cast<double>(a.imag()) * ci) * inv_valid;
                acc_im += (static_cast<double>(a.imag()) * cr -
                           static_cast<double>(a.real()) * ci) * inv_valid;
            }
            const std::complex<float> raw(
                static_cast<float>(acc_re / energy),
                static_cast<float>(acc_im / energy));
            scratch.raw_taps[nn] = raw;
        }
    }
    // Norm + peak metrics: shared scalar double pass (matches the
    // estimate_radar_cir edit; keeps peak_tap/l2 bit-identical).
    for (size_t nn = 0; nn < tap_count; ++nn) {
        nrm2 += static_cast<double>(std::norm(scratch.raw_taps[nn]));
        const float mag = std::abs(scratch.raw_taps[nn]);
        if (mag > peak_abs) {
            peak_abs = mag;
            peak_tap = nn;
        }
    }

    const float nrm = static_cast<float>(std::sqrt(nrm2));
    if (!std::isfinite(nrm) || !(nrm > 0.f) || !std::isfinite(peak_abs))
        return detail::cir_fail_status(out, CirStatus::CirFailed);

    const float inv_n = 1.f / (nrm + 1e-12f);
    for (size_t i = 0; i < tap_count; ++i)
        scratch.norm_taps[i] = scratch.raw_taps[i] * inv_n;

    out.status = CirStatus::Ok;
    out.tap_count = tap_count;
    out.peak_tap = peak_tap;
    out.peak_abs = peak_abs;
    out.raw_l2_norm = nrm;
    out.valid_repetitions = valid;
    out.first_repetition = skip_initial;
    return true;
}

// HOT PATH: estimate one absolute SYNC repetition without averaging.  This
// shares the prepared code and fixed scratch buffers with estimate_radar_cir;
// callers must consume/copy the taps before the next call.  No allocation or
// vector growth occurs here.
inline bool estimate_radar_cir_repetition(
    const std::complex<float>* rx,
    size_t n,
    int64_t preamble_start,
    size_t samples_per_symbol,
    size_t pre,
    size_t post,
    size_t repetition_index,
    size_t preamble_repetitions,
    RadarCirEstimate& out,
    RadarCirScratch& scratch)
{
    detail::cir_fail(out, CirStatus::InvalidInput);
    if (!rx || n == 0 || preamble_start < 0 || samples_per_symbol == 0 ||
        repetition_index >= preamble_repetitions ||
        scratch.sampled_code.empty() || !(scratch.code_energy > 0.f) ||
        !std::isfinite(scratch.code_energy) ||
        scratch.active_code_indices.empty() ||
        scratch.active_code_indices.size() != scratch.active_code_values.size() ||
        detail::cir_add_overflow(pre, post))
        return false;

    const size_t tap_count = pre + post;
    const size_t code_len = scratch.sampled_code.size();
    if (tap_count == 0 || tap_count > scratch.raw_taps.size() ||
        tap_count > scratch.norm_taps.size() ||
        code_len > std::numeric_limits<size_t>::max() - (tap_count - 1))
        return false;
    const size_t wlen = code_len + tap_count - 1;

    int64_t rep64 = 0, period64 = 0, offset = 0, rep_start = 0, pre64 = 0;
    int64_t lo = 0, n64 = 0, wlen64 = 0, last_ok = 0;
    if (!radar_i64_from_size(repetition_index, rep64) ||
        !radar_i64_from_size(samples_per_symbol, period64) ||
        !radar_i64_mul(rep64, period64, offset) ||
        !radar_i64_add(preamble_start, offset, rep_start) ||
        !radar_i64_from_size(pre, pre64) ||
        !radar_i64_sub(rep_start, pre64, lo) ||
        !radar_i64_from_size(n, n64) || !radar_i64_from_size(wlen, wlen64) ||
        !radar_i64_sub(n64, wlen64, last_ok) || lo < 0 || lo > last_ok)
        return detail::cir_fail_status(out, CirStatus::CirFailed);

    const std::complex<float>* src = rx + static_cast<size_t>(lo);
    const double energy = static_cast<double>(scratch.code_energy);
    size_t peak_tap = 0;
    float peak_abs = -1.f;
    double nrm2 = 0.0;
#if UWB_RADAR_CIR_HAVE_AVX2
    if (UWB_RADAR_CIR_CPU_AVX2()) {
        // AVX2: same 4-tap corr4 kernel on the raw repetition window.
        const size_t* const ci = scratch.active_code_indices.data();
        const float* const cw = scratch.wt_active.data();
        const size_t groups = tap_count / 4;
        const float inv_energy = static_cast<float>(1.0 / energy);
        for (size_t g = 0; g < groups; ++g) {
            detail::cir_corr4_kernel_avx2(
                src + static_cast<std::ptrdiff_t>(4 * g), ci, cw,
                    scratch.active_code_indices.size(),
                reinterpret_cast<float*>(scratch.raw_taps.data() + 4 * g),
                inv_energy);
        }
        const size_t rem = tap_count - 4 * groups;
        for (size_t t = 0; t < rem; ++t) {
            const size_t tap0 = 4 * groups + t;
            double acc_re = 0.0;
            double acc_im = 0.0;
            for (size_t j = 0; j < scratch.active_code_indices.size(); ++j) {
                const size_t m = scratch.active_code_indices[j];
                const auto c = scratch.active_code_values[j];
                const auto a = src[tap0 + m];
                const double cr = static_cast<double>(c.real());
                const double ci2 = static_cast<double>(c.imag());
                acc_re += static_cast<double>(a.real()) * cr +
                          static_cast<double>(a.imag()) * ci2;
                acc_im += static_cast<double>(a.imag()) * cr -
                          static_cast<double>(a.real()) * ci2;
            }
            scratch.raw_taps[tap0] = std::complex<float>(
                static_cast<float>(acc_re / energy),
                static_cast<float>(acc_im / energy));
        }
        // Norm + peak metric pass (shared scalar, double accumulation).
        for (size_t nn = 0; nn < tap_count; ++nn) {
            nrm2 += static_cast<double>(std::norm(scratch.raw_taps[nn]));
            const float mag = std::abs(scratch.raw_taps[nn]);
            if (mag > peak_abs) {
                peak_abs = mag;
                peak_tap = nn;
            }
        }
    } else
#endif
    {
        // Scalar mainline (phase-2 mainline): kept byte-for-byte so the
        // fallback path is bit-identical to the pre-phase-4a results.
        for (size_t nn = 0; nn < tap_count; ++nn) {
            double acc_re = 0.0;
            double acc_im = 0.0;
            for (size_t j = 0; j < scratch.active_code_indices.size(); ++j) {
                const size_t m = scratch.active_code_indices[j];
                const auto c = scratch.active_code_values[j];
                const auto a = src[nn + m];
                // HRP sampled_code is real {-1,0,+1}; keep the general
                // complex form for correctness, but avoid constructing
                // complex<double> temporaries in the 64 x tap_count loop.
                const double cr = static_cast<double>(c.real());
                const double ci = static_cast<double>(c.imag());
                acc_re += static_cast<double>(a.real()) * cr +
                          static_cast<double>(a.imag()) * ci;
                acc_im += static_cast<double>(a.imag()) * cr -
                          static_cast<double>(a.real()) * ci;
            }
            const std::complex<float> raw(
                static_cast<float>(acc_re / energy),
                static_cast<float>(acc_im / energy));
            scratch.raw_taps[nn] = raw;
            nrm2 += static_cast<double>(std::norm(raw));
            const float mag = std::abs(raw);
            if (mag > peak_abs) {
                peak_abs = mag;
                peak_tap = nn;
            }
        }
    }
    const float nrm = static_cast<float>(std::sqrt(nrm2));
    if (!std::isfinite(nrm) || !(nrm > 0.f) || !std::isfinite(peak_abs))
        return detail::cir_fail_status(out, CirStatus::CirFailed);
    const float inv_n = 1.f / (nrm + 1e-12f);
    for (size_t i = 0; i < tap_count; ++i)
        scratch.norm_taps[i] = scratch.raw_taps[i] * inv_n;

    out.status = CirStatus::Ok;
    out.tap_count = tap_count;
    out.peak_tap = peak_tap;
    out.peak_abs = peak_abs;
    out.raw_l2_norm = nrm;
    out.valid_repetitions = 1;
    out.first_repetition = repetition_index;
    return true;
}

} // namespace radar
} // namespace uwb
} // namespace gr
