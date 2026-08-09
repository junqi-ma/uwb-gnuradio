/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Fixed-length soft-chip matched-filter FIR kernels for stage_cir_softchips.
 *
 * Math (equivalent forms, no -ffast-math):
 *   chip = sum_k conj(values[T-1-k]) * rx[base - k]
 *        = sum_q conj(values[q])     * rx[base - (T-1) + q]   // contiguous forward
 *
 * Pre-prepare h[q] = conj(values[q]) once outside the chip loop, then:
 *   chip = sum_q h[q] * rx[win + q],  win = base - (T-1)
 *
 * Candidate kernels (P2):
 *   (a) MultiAcc8  — 8-way scalar accumulators (breaks serial reduction chain)
 *   (b) Volk       — volk_32fc_x2_conjugate_dot_prod_32fc (runtime dispatch)
 *   (c) Avx2Fixed  — fixed-38-tap AVX2/FMA with portable scalar fallback
 *
 * VOLK note (local install: /usr/include/volk, volk 2.5):
 *   volk_32fc_x2_conjugate_dot_prod_32fc(result, input, taps, N)
 *   computes  result = sum_i input[i] * conj(taps[i])
 *   so for our form we pass taps = values (natural order, NOT pre-conjugated)
 *   and input = &rx[base-(T-1)].  Dispatch picks AVX/SSE/generic once per call;
 *   ~154k short (38-pt) calls can dominate over the arithmetic.
 *
 * Prefer (c) when __AVX2__ && __FMA__; fallback (a).  Tail of 38 taps is
 * scalar (2 remaining after 9×4 SIMD lanes); no out-of-bounds loads.
 */

#pragma once

#include <complex>
#include <cstddef>
#include <cstdint>

#include <volk/volk.h>

#if defined(__AVX2__) && defined(__FMA__)
#include <immintrin.h>
#define UWB_CIR_FIR_HAVE_AVX2 1
#else
#define UWB_CIR_FIR_HAVE_AVX2 0
#endif

namespace gr {
namespace uwb {
namespace demod {
namespace cir_fir {

// Default max CIR taps used by QM35825 (pre=8 + post=30).  Kernels that
// specialise on 38 still accept a runtime tap_count and fall back when it
// differs.
static constexpr size_t kDefaultTapCount = 38;

// Sparse Top-K RAKE. Indices refer to the natural-order CIR tap array and
// weights are pre-conjugated. Eight independent accumulators avoid one long
// dependency chain; K=0 is not a valid call.
inline std::complex<float>
dot_topk(const std::complex<float>* rx_win,
         const uint8_t* indices,
         const std::complex<float>* weights,
         size_t k)
{
    std::complex<float> a0(0.0f, 0.0f), a1(0.0f, 0.0f),
        a2(0.0f, 0.0f), a3(0.0f, 0.0f), a4(0.0f, 0.0f),
        a5(0.0f, 0.0f), a6(0.0f, 0.0f), a7(0.0f, 0.0f);
    size_t i = 0;
    for (; i + 8 <= k; i += 8) {
        a0 += weights[i + 0] * rx_win[indices[i + 0]];
        a1 += weights[i + 1] * rx_win[indices[i + 1]];
        a2 += weights[i + 2] * rx_win[indices[i + 2]];
        a3 += weights[i + 3] * rx_win[indices[i + 3]];
        a4 += weights[i + 4] * rx_win[indices[i + 4]];
        a5 += weights[i + 5] * rx_win[indices[i + 5]];
        a6 += weights[i + 6] * rx_win[indices[i + 6]];
        a7 += weights[i + 7] * rx_win[indices[i + 7]];
    }
    for (; i + 4 <= k; i += 4) {
        a0 += weights[i + 0] * rx_win[indices[i + 0]];
        a1 += weights[i + 1] * rx_win[indices[i + 1]];
        a2 += weights[i + 2] * rx_win[indices[i + 2]];
        a3 += weights[i + 3] * rx_win[indices[i + 3]];
    }
    std::complex<float> acc =
        (a0 + a1) + (a2 + a3) + (a4 + a5) + (a6 + a7);
    for (; i < k; ++i)
        acc += weights[i] * rx_win[indices[i]];
    return acc;
}

// Fixed Top-4/Top-8 sparse RAKE across four adjacent decimated outputs.
// For one tap q, outputs use complex samples q+{0,2,4,6}. Two contiguous
// AVX2 loads plus shuffles form that vector without a slow hardware gather.
// The caller guarantees that q+7 is readable for every selected tap.
#if UWB_CIR_FIR_HAVE_AVX2
inline __m256 load_decim2_complex4(const std::complex<float>* rx_win, uint8_t q)
{
    const float* p = reinterpret_cast<const float*>(rx_win + q);
    const __m256 a = _mm256_loadu_ps(p);     // q .. q+3
    const __m256 b = _mm256_loadu_ps(p + 8); // q+4 .. q+7
    const __m256i pick = _mm256_setr_epi32(0, 1, 4, 5, 0, 1, 4, 5);
    const __m256 ae = _mm256_permutevar8x32_ps(a, pick);
    const __m256 be = _mm256_permutevar8x32_ps(b, pick);
    return _mm256_permute2f128_ps(ae, be, 0x20);
}

template <size_t K>
inline void dot_topk_x4_avx2(const std::complex<float>* rx_win,
                             const uint8_t* indices,
                             const std::complex<float>* weights,
                             std::complex<float>* out)
{
    static_assert(K == 4 || K == 8, "only fixed Top-4/Top-8 are supported");
    __m256 acc = _mm256_setzero_ps();
    for (size_t i = 0; i < K; ++i) {
        const __m256 x = load_decim2_complex4(rx_win, indices[i]);
        const float hr = weights[i].real();
        const float hi = weights[i].imag();
        const __m256 h = _mm256_setr_ps(hr, hi, hr, hi, hr, hi, hr, hi);
        const __m256 xre = _mm256_moveldup_ps(x);
        const __m256 xim = _mm256_movehdup_ps(x);
        const __m256 hsw = _mm256_permute_ps(h, 0xB1);
        const __m256 prod =
            _mm256_fmaddsub_ps(xre, h, _mm256_mul_ps(xim, hsw));
        acc = _mm256_add_ps(acc, prod);
    }
    _mm256_storeu_ps(reinterpret_cast<float*>(out), acc);
}
#endif

enum class Kernel : int {
    MultiAcc8 = 0, // (a)
    Volk = 1,      // (b)
    Avx2Fixed = 2, // (c)
};

// ---------------------------------------------------------------------------
// (a) 8-way scalar multi-accumulator.
// Pre-conjugated taps h[q] = conj(values[q]); window length = tap_count.
// ---------------------------------------------------------------------------
inline std::complex<float>
dot_multi_acc8(const std::complex<float>* rx_win,
               const std::complex<float>* h,
               size_t tap_count)
{
    std::complex<float> a0(0.0f, 0.0f), a1(0.0f, 0.0f), a2(0.0f, 0.0f),
        a3(0.0f, 0.0f), a4(0.0f, 0.0f), a5(0.0f, 0.0f), a6(0.0f, 0.0f),
        a7(0.0f, 0.0f);
    size_t q = 0;
    for (; q + 8 <= tap_count; q += 8) {
        a0 += h[q + 0] * rx_win[q + 0];
        a1 += h[q + 1] * rx_win[q + 1];
        a2 += h[q + 2] * rx_win[q + 2];
        a3 += h[q + 3] * rx_win[q + 3];
        a4 += h[q + 4] * rx_win[q + 4];
        a5 += h[q + 5] * rx_win[q + 5];
        a6 += h[q + 6] * rx_win[q + 6];
        a7 += h[q + 7] * rx_win[q + 7];
    }
    std::complex<float> acc =
        (a0 + a1) + (a2 + a3) + (a4 + a5) + (a6 + a7);
    for (; q < tap_count; ++q)
        acc += h[q] * rx_win[q];
    return acc;
}

// ---------------------------------------------------------------------------
// (b) VOLK conjugate dot product.
// Uses natural-order values (NOT pre-conjugated): VOLK conjugates the taps.
// ---------------------------------------------------------------------------
inline std::complex<float>
dot_volk(const std::complex<float>* rx_win,
         const std::complex<float>* values_natural,
         size_t tap_count)
{
    std::complex<float> acc(0.0f, 0.0f);
    volk_32fc_x2_conjugate_dot_prod_32fc(
        &acc,
        reinterpret_cast<const lv_32fc_t*>(rx_win),
        reinterpret_cast<const lv_32fc_t*>(values_natural),
        static_cast<unsigned int>(tap_count));
    return acc;
}

// ---------------------------------------------------------------------------
// (c) Fixed-38-tap AVX2/FMA kernel.  Portable scalar fallback otherwise.
// Pre-conjugated taps; processes 36 complex samples as 9×4 SIMD, then 2-tap
// scalar tail.  Uses unaligned loads (chip base is not 32-byte aligned).
// ---------------------------------------------------------------------------
// One 4-complex complex-MAC group → interleaved product vector.
#if UWB_CIR_FIR_HAVE_AVX2
inline __m256 cir_fir_avx2_group4(const float* xp, const float* hp, int off)
{
    const __m256 x = _mm256_loadu_ps(xp + off);
    const __m256 ht = _mm256_loadu_ps(hp + off);
    // xre = [xr0,xr0,xr1,xr1,xr2,xr2,xr3,xr3]
    // xim = [xi0,xi0,xi1,xi1,xi2,xi2,xi3,xi3]
    const __m256 xre = _mm256_moveldup_ps(x);
    const __m256 xim = _mm256_movehdup_ps(x);
    // hsw = [hi0,hr0,hi1,hr1,...]  (swap re/im within each complex pair)
    const __m256 hsw = _mm256_permute_ps(ht, 0xB1);
    // Complex mul via FMA3 fmaddsub:
    //   [xr*hr - xi*hi,  xr*hi + xi*hr, ...]
    const __m256 t = _mm256_mul_ps(xim, hsw);
    return _mm256_fmaddsub_ps(xre, ht, t);
}
#endif

inline std::complex<float>
dot_avx2_fixed38(const std::complex<float>* rx_win,
                 const std::complex<float>* h_conj,
                 size_t tap_count)
{
#if UWB_CIR_FIR_HAVE_AVX2
    if (tap_count == kDefaultTapCount) {
        // Two independent accumulators hide the add-chain latency across the
        // 9 groups of 4 complex taps (36 taps) + 2-tap scalar tail.
        const float* xp = reinterpret_cast<const float*>(rx_win);
        const float* hp = reinterpret_cast<const float*>(h_conj);

        __m256 acc0 = cir_fir_avx2_group4(xp, hp, 0);
        __m256 acc1 = cir_fir_avx2_group4(xp, hp, 8);
        acc0 = _mm256_add_ps(acc0, cir_fir_avx2_group4(xp, hp, 16));
        acc1 = _mm256_add_ps(acc1, cir_fir_avx2_group4(xp, hp, 24));
        acc0 = _mm256_add_ps(acc0, cir_fir_avx2_group4(xp, hp, 32));
        acc1 = _mm256_add_ps(acc1, cir_fir_avx2_group4(xp, hp, 40));
        acc0 = _mm256_add_ps(acc0, cir_fir_avx2_group4(xp, hp, 48));
        acc1 = _mm256_add_ps(acc1, cir_fir_avx2_group4(xp, hp, 56));
        acc0 = _mm256_add_ps(acc0, cir_fir_avx2_group4(xp, hp, 64));
        const __m256 acc = _mm256_add_ps(acc0, acc1);

        // Horizontal reduce 4 complex partials → one complex.
        const __m128 lo = _mm256_castps256_ps128(acc);
        const __m128 hi = _mm256_extractf128_ps(acc, 1);
        const __m128 s = _mm_add_ps(lo, hi); // [re0+re2, im0+im2, re1+re3, im1+im3]
        const __m128 s2 = _mm_add_ps(s, _mm_movehl_ps(s, s));
        float re = _mm_cvtss_f32(s2);
        float im = _mm_cvtss_f32(_mm_shuffle_ps(s2, s2, 1));

        // Scalar tail: taps 36 and 37 (no OOB beyond tap_count).
        const float* xpt = xp + 72; // 36 complex * 2 floats
        const float* hpt = hp + 72;
        // Manual complex mul for the two tail taps (avoids std::complex overhead).
        re += xpt[0] * hpt[0] - xpt[1] * hpt[1];
        im += xpt[0] * hpt[1] + xpt[1] * hpt[0];
        re += xpt[2] * hpt[2] - xpt[3] * hpt[3];
        im += xpt[2] * hpt[3] + xpt[3] * hpt[2];
        return std::complex<float>(re, im);
    }
#endif
    // Portable fallback (or non-38 tap_count): multi-acc scalar.
    return dot_multi_acc8(rx_win, h_conj, tap_count);
}

// Dispatch one chip FIR evaluation.
inline std::complex<float>
dot(Kernel k,
    const std::complex<float>* rx_win,
    const std::complex<float>* h_conj,
    const std::complex<float>* values_natural,
    size_t tap_count)
{
    switch (k) {
    case Kernel::Volk:
        return dot_volk(rx_win, values_natural, tap_count);
    case Kernel::Avx2Fixed:
        return dot_avx2_fixed38(rx_win, h_conj, tap_count);
    case Kernel::MultiAcc8:
    default:
        return dot_multi_acc8(rx_win, h_conj, tap_count);
    }
}

// Production default: fixed-38 AVX2/FMA when available, else multi-acc.
// Overridden at compile time for candidate benchmarking via
// -DUWB_CIR_FIR_KERNEL=0|1|2 .
#if defined(UWB_CIR_FIR_KERNEL)
static constexpr Kernel kDefaultKernel =
    static_cast<Kernel>(UWB_CIR_FIR_KERNEL);
#elif UWB_CIR_FIR_HAVE_AVX2
static constexpr Kernel kDefaultKernel = Kernel::Avx2Fixed;
#else
static constexpr Kernel kDefaultKernel = Kernel::MultiAcc8;
#endif

} // namespace cir_fir
} // namespace demod
} // namespace uwb
} // namespace gr
