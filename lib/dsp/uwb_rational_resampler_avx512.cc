/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * AVX-512 SC16 FIR kernels for the rational resampler core.
 *
 * See uwb_rational_resampler_avx512.h for the numerical contract: the
 * per-lane accumulation order and the reduction tree are identical to the
 * AVX2/FMA kernel, so every shipped H_pad produces bit-identical output.
 *
 * This file is compiled with the AVX2/FMA baseline only; the wide code is
 * isolated behind function target attributes.  That is deliberate: a global
 * -mavx512 flag would let the compiler auto-vectorise unrelated code, and an
 * unsupported machine could then die on SIGILL before the runtime check.
 */

#include <openuwb/dsp/rational_resampler_avx512.h>

#include <immintrin.h>

namespace gr {
namespace uwb {
namespace core {
namespace simd {
namespace avx512 {

namespace {

using sc16 = std::complex<int16_t>;
using gr_complex = std::complex<float>;

#define UWB_AVX512_TARGET \
    __attribute__((target("avx512f,avx512bw,avx512dq,avx512vl")))

/** Load 8 complex SC16 samples (16 int16) and widen to 16 floats. */
UWB_AVX512_TARGET inline __m512 load_sc16_x8_as_ps(const sc16* p) noexcept
{
    const __m256i v16 =
        _mm256_loadu_si256(reinterpret_cast<const __m256i*>(p));
    const __m512i v32 = _mm512_cvtepi16_epi32(v16);
    return _mm512_cvtepi32_ps(v32);
}

/**
 * Reduce a 16-float accumulator (8 complex lanes) to the 4-float
 * [re, im, re2, im2] form the AVX2 kernel holds after its acc_lo + acc_hi
 * step.  lo256 + hi256 reproduces that step exactly because lane j (j<4)
 * accumulated the AVX2 acc_lo taps and lane j+4 the acc_hi taps.
 */
UWB_AVX512_TARGET inline __m128 reduce_to_128(__m512 acc) noexcept
{
    const __m256 lo = _mm512_castps512_ps256(acc);
    const __m256 hi = _mm512_extractf32x8_ps(acc, 1);
    const __m256 s256 = _mm256_add_ps(lo, hi);
    return _mm_add_ps(_mm256_castps256_ps128(s256),
                      _mm256_extractf128_ps(s256, 1));
}

/** Full scalar reduction, identical to the AVX2 fir_avx2_sc16() tail. */
UWB_AVX512_TARGET inline gr_complex reduce_to_complex(__m512 acc) noexcept
{
    const __m128 s = reduce_to_128(acc);
    const __m128 sh = _mm_movehdup_ps(s);
    const __m128 re2 = _mm_add_ps(s, _mm_movehl_ps(s, s));
    const __m128 im2 = _mm_add_ps(sh, _mm_movehl_ps(sh, sh));
    return gr_complex(_mm_cvtss_f32(re2), _mm_cvtss_f32(im2));
}

/**
 * Four-output SC16 FIR.
 *
 * H_PAD != 0 gives a compile-time tap bound (fully unrolled); H_PAD == 0
 * keeps the runtime bound for any other profile.  The body mirrors
 * fir_avx2_sc16_4x_hp<H_PAD>() instruction for instruction, with one 16-float
 * accumulator per output instead of two 8-float accumulators.
 *
 * ALIGNED selects _mm512_load_ps for the tap vectors (RS-3).  The core's
 * coefficient table is 64-byte aligned and every arm stride is a multiple of
 * 16 floats, so the batched production entry points can use the aligned form;
 * the pointer-form public entries dispatch on the actual pointer and keep the
 * unaligned fallback.
 */
template <size_t H_PAD, bool ALIGNED = false>
UWB_AVX512_TARGET inline void
fir_sc16_4x_impl(const float* h0, const float* h1,
                 const float* h2, const float* h3,
                 const sc16* win0, const sc16* win1,
                 const sc16* win2, const sc16* win3,
                 size_t hpad_rt, uint8_t reuse, gr_complex* dest4) noexcept
{
    const size_t HP = (H_PAD != 0) ? H_PAD : hpad_rt;
    const bool w1_same = (reuse & 2u) != 0u;
    const bool w2_same = (reuse & 4u) != 0u;
    const bool w3_same = (reuse & 8u) != 0u;

    __m512 acc0 = _mm512_setzero_ps();
    __m512 acc1 = _mm512_setzero_ps();
    __m512 acc2 = _mm512_setzero_ps();
    __m512 acc3 = _mm512_setzero_ps();

    for (size_t q = 0; q < HP; q += 8) {
        const __m512 x0 = load_sc16_x8_as_ps(win0 + q);
        const __m512 h0v = ALIGNED ? _mm512_load_ps(h0 + 2 * q)
                                   : _mm512_loadu_ps(h0 + 2 * q);
        acc0 = _mm512_fmadd_ps(x0, h0v, acc0);

        const __m512 x1 = w1_same ? x0 : load_sc16_x8_as_ps(win1 + q);
        const __m512 h1v = ALIGNED ? _mm512_load_ps(h1 + 2 * q)
                                   : _mm512_loadu_ps(h1 + 2 * q);
        acc1 = _mm512_fmadd_ps(x1, h1v, acc1);

        const __m512 x2 = w2_same ? x1 : load_sc16_x8_as_ps(win2 + q);
        const __m512 h2v = ALIGNED ? _mm512_load_ps(h2 + 2 * q)
                                   : _mm512_loadu_ps(h2 + 2 * q);
        acc2 = _mm512_fmadd_ps(x2, h2v, acc2);

        const __m512 x3 = w3_same ? x2 : load_sc16_x8_as_ps(win3 + q);
        const __m512 h3v = ALIGNED ? _mm512_load_ps(h3 + 2 * q)
                                   : _mm512_loadu_ps(h3 + 2 * q);
        acc3 = _mm512_fmadd_ps(x3, h3v, acc3);
    }

    const __m128 s0 = reduce_to_128(acc0);
    const __m128 s1 = reduce_to_128(acc1);
    const __m128 s2 = reduce_to_128(acc2);
    const __m128 s3 = reduce_to_128(acc3);

    // Same two-output packing as the AVX2 4x kernel: the hadd step for two
    // outputs at once, so the stored floats are bit-identical.
    const __m128 out01 = _mm_add_ps(
        _mm_castpd_ps(_mm_unpacklo_pd(_mm_castps_pd(s0), _mm_castps_pd(s1))),
        _mm_castpd_ps(_mm_unpackhi_pd(_mm_castps_pd(s0), _mm_castps_pd(s1))));
    const __m128 out23 = _mm_add_ps(
        _mm_castpd_ps(_mm_unpacklo_pd(_mm_castps_pd(s2), _mm_castps_pd(s3))),
        _mm_castpd_ps(_mm_unpackhi_pd(_mm_castps_pd(s2), _mm_castps_pd(s3))));

    _mm_storeu_ps(reinterpret_cast<float*>(dest4), out01);
    _mm_storeu_ps(reinterpret_cast<float*>(dest4 + 2), out23);
}

/**
 * RS-4 register-sliding four-output SC16 FIR.
 *
 * The C1 descriptor and the production schedule both guarantee a short
 * monotone window advance inside one 4-output group: d_j = win[j] - win[0]
 * is small (<= 7 complex samples for every shipped direction).  Instead of
 * reloading and re-widening all four windows (and branching on the reuse
 * mask), load only win0 and win0 + 8 once per 8-tap step and rebuild the
 * shifted vectors with vpermi2ps (the runtime index form of
 * _mm512_alignr_epi32, whose immediate cannot be a runtime offset).
 *
 * This changes code generation only: the selected samples, tap vectors,
 * FMA order and reduction tree are identical to fir_sc16_4x_impl(), so the
 * output is bit-identical.
 */
template <size_t H_PAD, bool ALIGNED = true>
UWB_AVX512_TARGET inline void
fir_sc16_4x_slide_impl(const float* h0, const float* h1,
                       const float* h2, const float* h3,
                       const sc16* win0, int d1, int d2, int d3,
                       size_t hpad_rt, gr_complex* dest4) noexcept
{
    const size_t HP = (H_PAD != 0) ? H_PAD : hpad_rt;

    const __m512i lane = _mm512_setr_epi32(0, 1, 2, 3, 4, 5, 6, 7,
                                           8, 9, 10, 11, 12, 13, 14, 15);
    const __m512i idx1 = _mm512_add_epi32(lane, _mm512_set1_epi32(2 * d1));
    const __m512i idx2 = _mm512_add_epi32(lane, _mm512_set1_epi32(2 * d2));
    const __m512i idx3 = _mm512_add_epi32(lane, _mm512_set1_epi32(2 * d3));

    __m512 acc0 = _mm512_setzero_ps();
    __m512 acc1 = _mm512_setzero_ps();
    __m512 acc2 = _mm512_setzero_ps();
    __m512 acc3 = _mm512_setzero_ps();

    for (size_t q = 0; q < HP; q += 8) {
        const __m512 v0 = load_sc16_x8_as_ps(win0 + q);
        const __m512 v1 = load_sc16_x8_as_ps(win0 + q + 8);

        const __m512 x1 = (d1 == 0) ? v0
                                    : _mm512_permutex2var_ps(v0, idx1, v1);
        const __m512 x2 = (d2 == 0) ? v0
                                    : _mm512_permutex2var_ps(v0, idx2, v1);
        const __m512 x3 = (d3 == 0) ? v0
                                    : _mm512_permutex2var_ps(v0, idx3, v1);

        const __m512 h0v = ALIGNED ? _mm512_load_ps(h0 + 2 * q)
                                   : _mm512_loadu_ps(h0 + 2 * q);
        const __m512 h1v = ALIGNED ? _mm512_load_ps(h1 + 2 * q)
                                   : _mm512_loadu_ps(h1 + 2 * q);
        const __m512 h2v = ALIGNED ? _mm512_load_ps(h2 + 2 * q)
                                   : _mm512_loadu_ps(h2 + 2 * q);
        const __m512 h3v = ALIGNED ? _mm512_load_ps(h3 + 2 * q)
                                   : _mm512_loadu_ps(h3 + 2 * q);

        acc0 = _mm512_fmadd_ps(v0, h0v, acc0);
        acc1 = _mm512_fmadd_ps(x1, h1v, acc1);
        acc2 = _mm512_fmadd_ps(x2, h2v, acc2);
        acc3 = _mm512_fmadd_ps(x3, h3v, acc3);
    }

    const __m128 s0 = reduce_to_128(acc0);
    const __m128 s1 = reduce_to_128(acc1);
    const __m128 s2 = reduce_to_128(acc2);
    const __m128 s3 = reduce_to_128(acc3);

    const __m128 out01 = _mm_add_ps(
        _mm_castpd_ps(_mm_unpacklo_pd(_mm_castps_pd(s0), _mm_castps_pd(s1))),
        _mm_castpd_ps(_mm_unpackhi_pd(_mm_castps_pd(s0), _mm_castps_pd(s1))));
    const __m128 out23 = _mm_add_ps(
        _mm_castpd_ps(_mm_unpacklo_pd(_mm_castps_pd(s2), _mm_castps_pd(s3))),
        _mm_castpd_ps(_mm_unpackhi_pd(_mm_castps_pd(s2), _mm_castps_pd(s3))));

    _mm_storeu_ps(reinterpret_cast<float*>(dest4), out01);
    _mm_storeu_ps(reinterpret_cast<float*>(dest4 + 2), out23);
}

} // namespace

UWB_AVX512_TARGET
gr_complex fir_sc16(const float* taps_scaled, const sc16* win, size_t hpad)
{
    __m512 acc = _mm512_setzero_ps();
    for (size_t q = 0; q < hpad; q += 8) {
        acc = _mm512_fmadd_ps(load_sc16_x8_as_ps(win + q),
                              _mm512_loadu_ps(taps_scaled + 2 * q), acc);
    }
    return reduce_to_complex(acc);
}

UWB_AVX512_TARGET
void fir_sc16_4x(const float* h_base,
                 size_t h_stride,
                 uint8_t arm0, const sc16* win0,
                 uint8_t arm1, const sc16* win1,
                 uint8_t arm2, const sc16* win2,
                 uint8_t arm3, const sc16* win3,
                 size_t hpad,
                 gr_complex* dest4)
{
    const float* h0 = h_base + static_cast<size_t>(arm0) * h_stride;
    const float* h1 = h_base + static_cast<size_t>(arm1) * h_stride;
    const float* h2 = h_base + static_cast<size_t>(arm2) * h_stride;
    const float* h3 = h_base + static_cast<size_t>(arm3) * h_stride;

    uint8_t reuse = 0;
    if (win1 == win0)
        reuse |= 2u;
    if (win2 == win1)
        reuse |= 4u;
    if (win3 == win2)
        reuse |= 8u;

    fir_sc16_4x_impl<0>(h0, h1, h2, h3, win0, win1, win2, win3, hpad, reuse,
                        dest4);
}

namespace {

/** FC32 four-output kernel; same tree as fir_avx2_4x(). */
template <size_t H_PAD>
UWB_AVX512_TARGET inline void
fir_cf32_4x_impl(const float* h0, const float* h1,
                 const float* h2, const float* h3,
                 const gr_complex* win0, const gr_complex* win1,
                 const gr_complex* win2, const gr_complex* win3,
                 size_t hpad_rt, uint8_t reuse, gr_complex* dest4) noexcept
{
    const size_t HP = (H_PAD != 0) ? H_PAD : hpad_rt;
    const bool w1_same = (reuse & 2u) != 0u;
    const bool w2_same = (reuse & 4u) != 0u;
    const bool w3_same = (reuse & 8u) != 0u;

    const float* const xp0 = reinterpret_cast<const float*>(win0);
    const float* const xp1 = reinterpret_cast<const float*>(win1);
    const float* const xp2 = reinterpret_cast<const float*>(win2);
    const float* const xp3 = reinterpret_cast<const float*>(win3);

    __m512 acc0 = _mm512_setzero_ps();
    __m512 acc1 = _mm512_setzero_ps();
    __m512 acc2 = _mm512_setzero_ps();
    __m512 acc3 = _mm512_setzero_ps();

    for (size_t q = 0; q < HP; q += 8) {
        const __m512 x0 = _mm512_loadu_ps(xp0 + 2 * q);
        acc0 = _mm512_fmadd_ps(x0, _mm512_loadu_ps(h0 + 2 * q), acc0);

        const __m512 x1 = w1_same ? x0 : _mm512_loadu_ps(xp1 + 2 * q);
        acc1 = _mm512_fmadd_ps(x1, _mm512_loadu_ps(h1 + 2 * q), acc1);

        const __m512 x2 = w2_same ? x1 : _mm512_loadu_ps(xp2 + 2 * q);
        acc2 = _mm512_fmadd_ps(x2, _mm512_loadu_ps(h2 + 2 * q), acc2);

        const __m512 x3 = w3_same ? x2 : _mm512_loadu_ps(xp3 + 2 * q);
        acc3 = _mm512_fmadd_ps(x3, _mm512_loadu_ps(h3 + 2 * q), acc3);
    }

    const __m128 s0 = reduce_to_128(acc0);
    const __m128 s1 = reduce_to_128(acc1);
    const __m128 s2 = reduce_to_128(acc2);
    const __m128 s3 = reduce_to_128(acc3);

    const __m128 out01 = _mm_add_ps(
        _mm_castpd_ps(_mm_unpacklo_pd(_mm_castps_pd(s0), _mm_castps_pd(s1))),
        _mm_castpd_ps(_mm_unpackhi_pd(_mm_castps_pd(s0), _mm_castps_pd(s1))));
    const __m128 out23 = _mm_add_ps(
        _mm_castpd_ps(_mm_unpacklo_pd(_mm_castps_pd(s2), _mm_castps_pd(s3))),
        _mm_castpd_ps(_mm_unpackhi_pd(_mm_castps_pd(s2), _mm_castps_pd(s3))));

    _mm_storeu_ps(reinterpret_cast<float*>(dest4), out01);
    _mm_storeu_ps(reinterpret_cast<float*>(dest4 + 2), out23);
}

} // namespace

UWB_AVX512_TARGET
gr_complex fir_cf32(const float* taps_dup, const gr_complex* win, size_t hpad)
{
    __m512 acc = _mm512_setzero_ps();
    const float* const xp = reinterpret_cast<const float*>(win);
    for (size_t q = 0; q < hpad; q += 8) {
        acc = _mm512_fmadd_ps(_mm512_loadu_ps(xp + 2 * q),
                              _mm512_loadu_ps(taps_dup + 2 * q), acc);
    }
    return reduce_to_complex(acc);
}

UWB_AVX512_TARGET
void fir_cf32_4x(const float* h_base,
                 size_t h_stride,
                 uint8_t arm0, const gr_complex* win0,
                 uint8_t arm1, const gr_complex* win1,
                 uint8_t arm2, const gr_complex* win2,
                 uint8_t arm3, const gr_complex* win3,
                 size_t hpad,
                 gr_complex* dest4)
{
    const float* h0 = h_base + static_cast<size_t>(arm0) * h_stride;
    const float* h1 = h_base + static_cast<size_t>(arm1) * h_stride;
    const float* h2 = h_base + static_cast<size_t>(arm2) * h_stride;
    const float* h3 = h_base + static_cast<size_t>(arm3) * h_stride;

    uint8_t reuse = 0;
    if (win1 == win0)
        reuse |= 2u;
    if (win2 == win1)
        reuse |= 4u;
    if (win3 == win2)
        reuse |= 8u;

    fir_cf32_4x_impl<0>(h0, h1, h2, h3, win0, win1, win2, win3, hpad, reuse,
                        dest4);
}

UWB_AVX512_TARGET
void fir_sc16_4x_hp16(const float* h0, const float* h1,
                      const float* h2, const float* h3,
                      const sc16* win0, const sc16* win1,
                      const sc16* win2, const sc16* win3,
                      size_t hpad_rt, uint8_t reuse, gr_complex* dest4)
{
    fir_sc16_4x_impl<16, true>(h0, h1, h2, h3, win0, win1, win2, win3, hpad_rt,
                         reuse, dest4);
}

UWB_AVX512_TARGET
void fir_sc16_4x_hp24(const float* h0, const float* h1,
                      const float* h2, const float* h3,
                      const sc16* win0, const sc16* win1,
                      const sc16* win2, const sc16* win3,
                      size_t hpad_rt, uint8_t reuse, gr_complex* dest4)
{
    fir_sc16_4x_impl<24, true>(h0, h1, h2, h3, win0, win1, win2, win3, hpad_rt,
                         reuse, dest4);
}

UWB_AVX512_TARGET
void fir_sc16_4x_hp32(const float* h0, const float* h1,
                      const float* h2, const float* h3,
                      const sc16* win0, const sc16* win1,
                      const sc16* win2, const sc16* win3,
                      size_t hpad_rt, uint8_t reuse, gr_complex* dest4)
{
    fir_sc16_4x_impl<32, true>(h0, h1, h2, h3, win0, win1, win2, win3, hpad_rt,
                         reuse, dest4);
}

UWB_AVX512_TARGET
void fir_sc16_4x_hp48(const float* h0, const float* h1,
                      const float* h2, const float* h3,
                      const sc16* win0, const sc16* win1,
                      const sc16* win2, const sc16* win3,
                      size_t hpad_rt, uint8_t reuse, gr_complex* dest4)
{
    fir_sc16_4x_impl<48, true>(h0, h1, h2, h3, win0, win1, win2, win3, hpad_rt,
                         reuse, dest4);
}

UWB_AVX512_TARGET
void fir_sc16_4x_generic(const float* h0, const float* h1,
                         const float* h2, const float* h3,
                         const sc16* win0, const sc16* win1,
                         const sc16* win2, const sc16* win3,
                         size_t hpad_rt, uint8_t reuse, gr_complex* dest4)
{
    fir_sc16_4x_impl<0, true>(h0, h1, h2, h3, win0, win1, win2, win3, hpad_rt,
                        reuse, dest4);
}

namespace {

template <size_t H_PAD>
UWB_AVX512_TARGET inline void
fir_sc16_macroblock_c1_impl(const float* h_base,
                            const C1GroupDesc* groups, size_t ngroups,
                            const sc16* base_win, size_t hpad_rt,
                            gr_complex* dest,
                            const C1TailDesc* tails, size_t ntails) noexcept
{
    for (size_t g = 0; g < ngroups; ++g) {
        const C1GroupDesc& d = groups[g];
        const int d1 = static_cast<int>(d.win[1]) - static_cast<int>(d.win[0]);
        const int d2 = static_cast<int>(d.win[2]) - static_cast<int>(d.win[0]);
        const int d3 = static_cast<int>(d.win[3]) - static_cast<int>(d.win[0]);
        if (d3 <= 7) {
            fir_sc16_4x_slide_impl<H_PAD, true>(
                h_base + d.tap[0], h_base + d.tap[1],
                h_base + d.tap[2], h_base + d.tap[3],
                base_win + d.win[0], d1, d2, d3, hpad_rt, dest + 4 * g);
        } else {
            // Non-shipped direction with a wider per-group advance: keep the
            // pointer form (still bit-identical, just without the sliding
            // register reuse).
            fir_sc16_4x_impl<H_PAD, true>(
                h_base + d.tap[0], h_base + d.tap[1],
                h_base + d.tap[2], h_base + d.tap[3],
                base_win + d.win[0], base_win + d.win[1],
                base_win + d.win[2], base_win + d.win[3],
                hpad_rt, d.reuse, dest + 4 * g);
        }
    }
    for (size_t t = 0; t < ntails; ++t) {
        const size_t o = 4 * ngroups + t;
        dest[o] = fir_sc16(h_base + tails[t].tap, base_win + tails[t].win,
                           (H_PAD != 0) ? H_PAD : hpad_rt);
    }
}

/**
 * RS-2 batched macroblock body: the same per-macroblock work as
 * fir_sc16_macroblock_c1_impl() in one call; every macroblock shares the
 * group/tail descriptors and advances by `decim` inputs / `nout` outputs.
 */
template <size_t H_PAD>
UWB_AVX512_TARGET inline void
fir_sc16_macroblocks_c1_impl(const float* h_base,
                             const C1GroupDesc* groups, size_t ngroups,
                             size_t n_macroblocks, size_t decim, size_t nout,
                             const C1TailDesc* tails, size_t ntails,
                             const sc16* base_win, size_t hpad_rt,
                             gr_complex* dest) noexcept
{
    for (size_t mb = 0; mb < n_macroblocks; ++mb) {
        fir_sc16_macroblock_c1_impl<H_PAD>(
            h_base, groups, ngroups, base_win + mb * decim, hpad_rt,
            dest + mb * nout, tails, ntails);
    }
}

} // namespace

UWB_AVX512_TARGET
void fir_sc16_macroblock_c1_hp16(const float* h_base,
                                 const C1GroupDesc* groups, size_t ngroups,
                                 const sc16* base_win, gr_complex* dest)
{
    // Single-macroblock form has no leftover-output descriptors available;
    // callers that need them use the batched entry point.
    fir_sc16_macroblock_c1_impl<16>(h_base, groups, ngroups, base_win, 0,
                                    dest, nullptr, 0);
}

UWB_AVX512_TARGET
void fir_sc16_macroblock_c1_hp24(const float* h_base,
                                 const C1GroupDesc* groups, size_t ngroups,
                                 const sc16* base_win, gr_complex* dest)
{
    fir_sc16_macroblock_c1_impl<24>(h_base, groups, ngroups, base_win, 0,
                                    dest, nullptr, 0);
}

UWB_AVX512_TARGET
void fir_sc16_macroblock_c1_hp32(const float* h_base,
                                 const C1GroupDesc* groups, size_t ngroups,
                                 const sc16* base_win, gr_complex* dest)
{
    fir_sc16_macroblock_c1_impl<32>(h_base, groups, ngroups, base_win, 0,
                                    dest, nullptr, 0);
}

UWB_AVX512_TARGET
void fir_sc16_macroblock_c1_hp48(const float* h_base,
                                 const C1GroupDesc* groups, size_t ngroups,
                                 const sc16* base_win, gr_complex* dest)
{
    fir_sc16_macroblock_c1_impl<48>(h_base, groups, ngroups, base_win, 0,
                                    dest, nullptr, 0);
}

UWB_AVX512_TARGET
void fir_sc16_macroblock_c1_generic(const float* h_base,
                                    const C1GroupDesc* groups, size_t ngroups,
                                    const sc16* base_win, size_t hpad,
                                    gr_complex* dest)
{
    fir_sc16_macroblock_c1_impl<0>(h_base, groups, ngroups, base_win, hpad,
                                   dest, nullptr, 0);
}

UWB_AVX512_TARGET
void fir_sc16_macroblocks_c1_hp16(const float* h_base,
                                  const C1GroupDesc* groups, size_t ngroups,
                                  size_t n_macroblocks, size_t decim, size_t nout,
                                  const C1TailDesc* tails, size_t ntails,
                                  const sc16* base_win, gr_complex* dest)
{
    fir_sc16_macroblocks_c1_impl<16>(h_base, groups, ngroups, n_macroblocks,
                                     decim, nout, tails, ntails, base_win, 0,
                                     dest);
}

UWB_AVX512_TARGET
void fir_sc16_macroblocks_c1_hp24(const float* h_base,
                                  const C1GroupDesc* groups, size_t ngroups,
                                  size_t n_macroblocks, size_t decim, size_t nout,
                                  const C1TailDesc* tails, size_t ntails,
                                  const sc16* base_win, gr_complex* dest)
{
    fir_sc16_macroblocks_c1_impl<24>(h_base, groups, ngroups, n_macroblocks,
                                     decim, nout, tails, ntails, base_win, 0,
                                     dest);
}

UWB_AVX512_TARGET
void fir_sc16_macroblocks_c1_hp32(const float* h_base,
                                  const C1GroupDesc* groups, size_t ngroups,
                                  size_t n_macroblocks, size_t decim, size_t nout,
                                  const C1TailDesc* tails, size_t ntails,
                                  const sc16* base_win, gr_complex* dest)
{
    fir_sc16_macroblocks_c1_impl<32>(h_base, groups, ngroups, n_macroblocks,
                                     decim, nout, tails, ntails, base_win, 0,
                                     dest);
}

UWB_AVX512_TARGET
void fir_sc16_macroblocks_c1_hp48(const float* h_base,
                                  const C1GroupDesc* groups, size_t ngroups,
                                  size_t n_macroblocks, size_t decim, size_t nout,
                                  const C1TailDesc* tails, size_t ntails,
                                  const sc16* base_win, gr_complex* dest)
{
    fir_sc16_macroblocks_c1_impl<48>(h_base, groups, ngroups, n_macroblocks,
                                     decim, nout, tails, ntails, base_win, 0,
                                     dest);
}

UWB_AVX512_TARGET
void fir_sc16_macroblocks_c1_generic(const float* h_base,
                                     const C1GroupDesc* groups, size_t ngroups,
                                     size_t n_macroblocks, size_t decim,
                                     size_t nout, const C1TailDesc* tails,
                                     size_t ntails, const sc16* base_win,
                                     size_t hpad, gr_complex* dest)
{
    fir_sc16_macroblocks_c1_impl<0>(h_base, groups, ngroups, n_macroblocks,
                                    decim, nout, tails, ntails, base_win, hpad,
                                    dest);
}

UWB_AVX512_TARGET
void fir_sc16_macroblock(const float* h_base, size_t h_stride,
                         const uint8_t* arms, const uint8_t* advs,
                         const sc16* base_win, size_t decim,
                         size_t n_macroblocks, size_t nout, size_t hpad,
                         gr_complex* dest)
{
    for (size_t mb = 0; mb < n_macroblocks; ++mb) {
        const sc16* const win = base_win + mb * decim;
        gr_complex* const d = dest + mb * nout;
        size_t cur = 0;
        size_t o = 0;
        for (; o + 4 <= nout; o += 4) {
            const sc16* const w0 = win + cur;
            cur += advs[o];
            const sc16* const w1 = win + cur;
            cur += advs[o + 1];
            const sc16* const w2 = win + cur;
            cur += advs[o + 2];
            const sc16* const w3 = win + cur;
            cur += advs[o + 3];

            const int d1 = static_cast<int>(w1 - w0);
            const int d2 = static_cast<int>(w2 - w0);
            const int d3 = static_cast<int>(w3 - w0);

            const float* const h0 =
                h_base + static_cast<size_t>(arms[o]) * h_stride;
            const float* const h1 =
                h_base + static_cast<size_t>(arms[o + 1]) * h_stride;
            const float* const h2 =
                h_base + static_cast<size_t>(arms[o + 2]) * h_stride;
            const float* const h3 =
                h_base + static_cast<size_t>(arms[o + 3]) * h_stride;

            if (d3 <= 7) {
                fir_sc16_4x_slide_impl<0, true>(
                    h0, h1, h2, h3, w0, d1, d2, d3, hpad, d + o);
            } else {
                uint8_t reuse = 0;
                if (w1 == w0)
                    reuse |= 2u;
                if (w2 == w1)
                    reuse |= 4u;
                if (w3 == w2)
                    reuse |= 8u;
                fir_sc16_4x_impl<0, true>(h0, h1, h2, h3, w0, w1, w2, w3,
                                          hpad, reuse, d + o);
            }
        }
        for (; o < nout; ++o) {
            d[o] = fir_sc16(
                h_base + static_cast<size_t>(arms[o]) * h_stride,
                win + cur, hpad);
            cur += advs[o];
        }
    }
}

UWB_AVX512_TARGET
void fir_cf32_macroblock(const float* h_base, size_t h_stride,
                         const uint8_t* arms, const uint8_t* advs,
                         const gr_complex* base_win, size_t decim,
                         size_t n_macroblocks, size_t nout, size_t hpad,
                         gr_complex* dest)
{
    for (size_t mb = 0; mb < n_macroblocks; ++mb) {
        const gr_complex* const win = base_win + mb * decim;
        gr_complex* const d = dest + mb * nout;
        size_t cur = 0;
        size_t o = 0;
        for (; o + 4 <= nout; o += 4) {
            const gr_complex* const w0 = win + cur;
            cur += advs[o];
            const gr_complex* const w1 = win + cur;
            cur += advs[o + 1];
            const gr_complex* const w2 = win + cur;
            cur += advs[o + 2];
            const gr_complex* const w3 = win + cur;
            cur += advs[o + 3];

            uint8_t reuse = 0;
            if (w1 == w0)
                reuse |= 2u;
            if (w2 == w1)
                reuse |= 4u;
            if (w3 == w2)
                reuse |= 8u;

            fir_cf32_4x_impl<0>(
                h_base + static_cast<size_t>(arms[o]) * h_stride,
                h_base + static_cast<size_t>(arms[o + 1]) * h_stride,
                h_base + static_cast<size_t>(arms[o + 2]) * h_stride,
                h_base + static_cast<size_t>(arms[o + 3]) * h_stride,
                w0, w1, w2, w3, hpad, reuse, d + o);
        }
        for (; o < nout; ++o) {
            d[o] = fir_cf32(
                h_base + static_cast<size_t>(arms[o]) * h_stride,
                win + cur, hpad);
            cur += advs[o];
        }
    }
}

} // namespace avx512
} // namespace simd
} // namespace core
} // namespace uwb
} // namespace gr
