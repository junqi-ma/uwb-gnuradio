/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * RS-5 AVX-512 8-macroblock transposed microkernel.
 *
 * This is the ONLY translation unit for the transposed kernel that may contain
 * AVX-512 instructions.  Like uwb_rational_resampler_avx512.cc it is compiled
 * on the AVX2/FMA baseline and every function carries the explicit target
 * attribute, so no wide instruction can execute before the runtime check in
 * TransposedResampler / RationalResamplerLmCore.
 *
 * Numerical contract: see uwb_resampler_transposed.h.  The accumulation is
 * one vector accumulator per output phase over the tap axis, validated against
 * an independent float64 upfirdn model rather than against the bit-exact
 * macroblock kernels.
 */

#include <gnuradio/uwb/uwb_resampler_transposed.h>

#if UWB_SIMD_HAVE_AVX512

#include <immintrin.h>

namespace gr {
namespace uwb {
namespace core {
namespace simd {
namespace transposed {
namespace avx512 {

namespace {

#define UWB_TRANSPOSED_AVX512_TARGET \
    __attribute__((target("avx512f,avx512bw,avx512dq,avx512vl")))

UWB_TRANSPOSED_AVX512_TARGET inline int32_t load_sc16_bits(const sc16* p)
{
    int32_t v;
    __builtin_memcpy(&v, p, sizeof(v));
    return v;
}

/** Build X_tile[u] for 8 macroblocks: [re0,im0,...,re7,im7] as 16 floats. */
UWB_TRANSPOSED_AVX512_TARGET inline void
load_tile_slot(const sc16* in, size_t ninput, size_t mb0, size_t u, size_t H,
               size_t decim, float* tile_slot)
{
    int32_t v[8];
    for (size_t m = 0; m < 8; ++m) {
        const int64_t idx = static_cast<int64_t>(mb0 + m) *
                                static_cast<int64_t>(decim) +
                            static_cast<int64_t>(u) -
                            static_cast<int64_t>(H - 1);
        v[m] = (idx >= 0 && idx < static_cast<int64_t>(ninput))
                   ? load_sc16_bits(in + idx)
                   : 0;
    }
    const __m256i q = _mm256_setr_epi32(v[0], v[1], v[2], v[3], v[4], v[5],
                                        v[6], v[7]);
    const __m512i i32 = _mm512_cvtepi16_epi32(q);
    _mm512_store_ps(tile_slot, _mm512_cvtepi32_ps(i32));
}

UWB_TRANSPOSED_AVX512_TARGET inline __m512
phase_accum(const float* tp, const float* tile, size_t H)
{
    __m512 acc = _mm512_setzero_ps();
    for (size_t k = 0; k < H; ++k) {
        acc = _mm512_fmadd_ps(_mm512_set1_ps(tp[k]),
                              _mm512_load_ps(tile + k * 16), acc);
    }
    return acc;
}

UWB_TRANSPOSED_AVX512_TARGET inline void
load_tile_4slots_interior(const sc16* const* p8, size_t u, float* tile_base)
{
    const __m128i r0 = _mm_loadu_si128(reinterpret_cast<const __m128i*>(p8[0] + u));
    const __m128i r1 = _mm_loadu_si128(reinterpret_cast<const __m128i*>(p8[1] + u));
    const __m128i r2 = _mm_loadu_si128(reinterpret_cast<const __m128i*>(p8[2] + u));
    const __m128i r3 = _mm_loadu_si128(reinterpret_cast<const __m128i*>(p8[3] + u));
    const __m128i r4 = _mm_loadu_si128(reinterpret_cast<const __m128i*>(p8[4] + u));
    const __m128i r5 = _mm_loadu_si128(reinterpret_cast<const __m128i*>(p8[5] + u));
    const __m128i r6 = _mm_loadu_si128(reinterpret_cast<const __m128i*>(p8[6] + u));
    const __m128i r7 = _mm_loadu_si128(reinterpret_cast<const __m128i*>(p8[7] + u));

    const __m128i a01 = _mm_unpacklo_epi32(r0, r1);
    const __m128i a23 = _mm_unpacklo_epi32(r2, r3);
    const __m128i a45 = _mm_unpacklo_epi32(r4, r5);
    const __m128i a67 = _mm_unpacklo_epi32(r6, r7);
    const __m128i b01 = _mm_unpackhi_epi32(r0, r1);
    const __m128i b23 = _mm_unpackhi_epi32(r2, r3);
    const __m128i b45 = _mm_unpackhi_epi32(r4, r5);
    const __m128i b67 = _mm_unpackhi_epi32(r6, r7);

    const __m256i q0 = _mm256_set_m128i(_mm_unpacklo_epi64(a45, a67), _mm_unpacklo_epi64(a01, a23));
    const __m256i q1 = _mm256_set_m128i(_mm_unpackhi_epi64(a45, a67), _mm_unpackhi_epi64(a01, a23));
    const __m256i q2 = _mm256_set_m128i(_mm_unpacklo_epi64(b45, b67), _mm_unpacklo_epi64(b01, b23));
    const __m256i q3 = _mm256_set_m128i(_mm_unpackhi_epi64(b45, b67), _mm_unpackhi_epi64(b01, b23));

    _mm512_store_ps(tile_base + (u + 0) * 16, _mm512_cvtepi32_ps(_mm512_cvtepi16_epi32(q0)));
    _mm512_store_ps(tile_base + (u + 1) * 16, _mm512_cvtepi32_ps(_mm512_cvtepi16_epi32(q1)));
    _mm512_store_ps(tile_base + (u + 2) * 16, _mm512_cvtepi32_ps(_mm512_cvtepi16_epi32(q2)));
    _mm512_store_ps(tile_base + (u + 3) * 16, _mm512_cvtepi32_ps(_mm512_cvtepi16_epi32(q3)));
}

template <size_t H_VAL>
UWB_TRANSPOSED_AVX512_TARGET inline void
phase_accum_8x(const float* const* tp8, const float* const* x8, size_t h_rt,
               __m512* out8)
{
    const size_t H = (H_VAL != 0) ? H_VAL : h_rt;
    __m512 a0 = _mm512_setzero_ps(), a1 = _mm512_setzero_ps();
    __m512 a2 = _mm512_setzero_ps(), a3 = _mm512_setzero_ps();
    __m512 a4 = _mm512_setzero_ps(), a5 = _mm512_setzero_ps();
    __m512 a6 = _mm512_setzero_ps(), a7 = _mm512_setzero_ps();
    const float* const tp0 = tp8[0]; const float* const tp1 = tp8[1];
    const float* const tp2 = tp8[2]; const float* const tp3 = tp8[3];
    const float* const tp4 = tp8[4]; const float* const tp5 = tp8[5];
    const float* const tp6 = tp8[6]; const float* const tp7 = tp8[7];
    const float* const x0 = x8[0]; const float* const x1 = x8[1];
    const float* const x2 = x8[2]; const float* const x3 = x8[3];
    const float* const x4 = x8[4]; const float* const x5 = x8[5];
    const float* const x6 = x8[6]; const float* const x7 = x8[7];
    for (size_t k = 0; k < H; ++k) {
        const size_t off = k * 16;
        a0 = _mm512_fmadd_ps(_mm512_set1_ps(tp0[k]), _mm512_load_ps(x0 + off), a0);
        a1 = _mm512_fmadd_ps(_mm512_set1_ps(tp1[k]), _mm512_load_ps(x1 + off), a1);
        a2 = _mm512_fmadd_ps(_mm512_set1_ps(tp2[k]), _mm512_load_ps(x2 + off), a2);
        a3 = _mm512_fmadd_ps(_mm512_set1_ps(tp3[k]), _mm512_load_ps(x3 + off), a3);
        a4 = _mm512_fmadd_ps(_mm512_set1_ps(tp4[k]), _mm512_load_ps(x4 + off), a4);
        a5 = _mm512_fmadd_ps(_mm512_set1_ps(tp5[k]), _mm512_load_ps(x5 + off), a5);
        a6 = _mm512_fmadd_ps(_mm512_set1_ps(tp6[k]), _mm512_load_ps(x6 + off), a6);
        a7 = _mm512_fmadd_ps(_mm512_set1_ps(tp7[k]), _mm512_load_ps(x7 + off), a7);
    }
    out8[0] = a0; out8[1] = a1; out8[2] = a2; out8[3] = a3;
    out8[4] = a4; out8[5] = a5; out8[6] = a6; out8[7] = a7;
}

UWB_TRANSPOSED_AVX512_TARGET inline void
transpose_8x8_store(const __m512* out8, gr_complex* out, size_t mb0,
                    size_t Interp, size_t j)
{
    const __m512d r0 = _mm512_castps_pd(out8[0]);
    const __m512d r1 = _mm512_castps_pd(out8[1]);
    const __m512d r2 = _mm512_castps_pd(out8[2]);
    const __m512d r3 = _mm512_castps_pd(out8[3]);
    const __m512d r4 = _mm512_castps_pd(out8[4]);
    const __m512d r5 = _mm512_castps_pd(out8[5]);
    const __m512d r6 = _mm512_castps_pd(out8[6]);
    const __m512d r7 = _mm512_castps_pd(out8[7]);

    const __m512d t0 = _mm512_unpacklo_pd(r0, r1);
    const __m512d t1 = _mm512_unpackhi_pd(r0, r1);
    const __m512d t2 = _mm512_unpacklo_pd(r2, r3);
    const __m512d t3 = _mm512_unpackhi_pd(r2, r3);
    const __m512d t4 = _mm512_unpacklo_pd(r4, r5);
    const __m512d t5 = _mm512_unpackhi_pd(r4, r5);
    const __m512d t6 = _mm512_unpacklo_pd(r6, r7);
    const __m512d t7 = _mm512_unpackhi_pd(r6, r7);

    const __m512d u0 = _mm512_shuffle_f64x2(t0, t2, 0x44);
    const __m512d u2 = _mm512_shuffle_f64x2(t0, t2, 0xEE);
    const __m512d u4 = _mm512_shuffle_f64x2(t4, t6, 0x44);
    const __m512d u6 = _mm512_shuffle_f64x2(t4, t6, 0xEE);

    const __m512d u1 = _mm512_shuffle_f64x2(t1, t3, 0x44);
    const __m512d u3 = _mm512_shuffle_f64x2(t1, t3, 0xEE);
    const __m512d u5 = _mm512_shuffle_f64x2(t5, t7, 0x44);
    const __m512d u7 = _mm512_shuffle_f64x2(t5, t7, 0xEE);

    const __m512d row0 = _mm512_shuffle_f64x2(u0, u4, 0x88);
    const __m512d row2 = _mm512_shuffle_f64x2(u0, u4, 0xDD);
    const __m512d row4 = _mm512_shuffle_f64x2(u2, u6, 0x88);
    const __m512d row6 = _mm512_shuffle_f64x2(u2, u6, 0xDD);

    const __m512d row1 = _mm512_shuffle_f64x2(u1, u5, 0x88);
    const __m512d row3 = _mm512_shuffle_f64x2(u1, u5, 0xDD);
    const __m512d row5 = _mm512_shuffle_f64x2(u3, u7, 0x88);
    const __m512d row7 = _mm512_shuffle_f64x2(u3, u7, 0xDD);

    _mm512_storeu_ps(reinterpret_cast<float*>(out + (mb0 + 0) * Interp + j), _mm512_castpd_ps(row0));
    _mm512_storeu_ps(reinterpret_cast<float*>(out + (mb0 + 1) * Interp + j), _mm512_castpd_ps(row1));
    _mm512_storeu_ps(reinterpret_cast<float*>(out + (mb0 + 2) * Interp + j), _mm512_castpd_ps(row2));
    _mm512_storeu_ps(reinterpret_cast<float*>(out + (mb0 + 3) * Interp + j), _mm512_castpd_ps(row3));
    _mm512_storeu_ps(reinterpret_cast<float*>(out + (mb0 + 4) * Interp + j), _mm512_castpd_ps(row4));
    _mm512_storeu_ps(reinterpret_cast<float*>(out + (mb0 + 5) * Interp + j), _mm512_castpd_ps(row5));
    _mm512_storeu_ps(reinterpret_cast<float*>(out + (mb0 + 6) * Interp + j), _mm512_castpd_ps(row6));
    _mm512_storeu_ps(reinterpret_cast<float*>(out + (mb0 + 7) * Interp + j), _mm512_castpd_ps(row7));
}

template <size_t H_VAL>
UWB_TRANSPOSED_AVX512_TARGET inline void
transposed_process_window_hp(const float* taps, size_t h_rt, size_t Interp,
                             size_t Decim, const sc16* in, size_t ninput,
                             size_t n_mb_full, float* tile, gr_complex* out)
{
    const size_t H = (H_VAL != 0) ? H_VAL : h_rt;
    const size_t slots = Decim + H - 1;

    for (size_t mb0 = 0; mb0 + 8 <= n_mb_full; mb0 += 8) {
        const int64_t base0 = static_cast<int64_t>(mb0) * Decim -
                              static_cast<int64_t>(H - 1);
        const int64_t max_idx = base0 + 7 * static_cast<int64_t>(Decim) +
                                static_cast<int64_t>(slots);
        if (base0 >= 0 && max_idx <= static_cast<int64_t>(ninput)) {
            const sc16* p8[8];
            for (size_t m = 0; m < 8; ++m)
                p8[m] = in + base0 + m * Decim;
            const size_t slots_vec = slots & ~size_t(3);
            for (size_t u = 0; u < slots_vec; u += 4)
                load_tile_4slots_interior(p8, u, tile);
            for (size_t u = slots_vec; u < slots; ++u) {
                const __m256i q = _mm256_setr_epi32(
                    load_sc16_bits(p8[0] + u), load_sc16_bits(p8[1] + u),
                    load_sc16_bits(p8[2] + u), load_sc16_bits(p8[3] + u),
                    load_sc16_bits(p8[4] + u), load_sc16_bits(p8[5] + u),
                    load_sc16_bits(p8[6] + u), load_sc16_bits(p8[7] + u));
                _mm512_store_ps(tile + u * 16,
                                _mm512_cvtepi32_ps(_mm512_cvtepi16_epi32(q)));
            }
        } else {
            for (size_t u = 0; u < slots; ++u)
                load_tile_slot(in, ninput, mb0, u, H, Decim, tile + u * 16);
        }

        size_t j = 0;
        for (; j + 8 <= Interp; j += 8) {
            const float* tp8[8];
            const float* x8[8];
            for (size_t p = 0; p < 8; ++p) {
                const size_t prod = (j + p) * Decim;
                tp8[p] = taps + (prod % Interp) * H;
                x8[p] = tile + (prod / Interp) * 16;
            }
            __m512 out8[8];
            phase_accum_8x<H_VAL>(tp8, x8, H, out8);
            transpose_8x8_store(out8, out, mb0, Interp, j);
        }
        for (; j < Interp; ++j) {
            const size_t arm = (j * Decim) % Interp;
            const size_t w = (j * Decim) / Interp;
            const __m512 acc = phase_accum(taps + arm * H, tile + w * 16, H);
            alignas(64) gr_complex tmp[8];
            _mm512_store_ps(reinterpret_cast<float*>(tmp), acc);
            for (size_t m = 0; m < 8; ++m)
                out[(mb0 + m) * Interp + j] = tmp[m];
        }
    }
}

} // namespace

UWB_TRANSPOSED_AVX512_TARGET
void transposed_process_window(const float* taps, size_t H, size_t Interp,
                               size_t Decim, const sc16* in, size_t ninput,
                               size_t n_mb_full, float* tile, gr_complex* out)
{
    if (H == 16) transposed_process_window_hp<16>(taps, H, Interp, Decim, in, ninput, n_mb_full, tile, out);
    else if (H == 21) transposed_process_window_hp<21>(taps, H, Interp, Decim, in, ninput, n_mb_full, tile, out);
    else if (H == 32) transposed_process_window_hp<32>(taps, H, Interp, Decim, in, ninput, n_mb_full, tile, out);
    else if (H == 42) transposed_process_window_hp<42>(taps, H, Interp, Decim, in, ninput, n_mb_full, tile, out);
    else transposed_process_window_hp<0>(taps, H, Interp, Decim, in, ninput, n_mb_full, tile, out);
}

} // namespace avx512
} // namespace transposed
} // namespace simd
} // namespace core
} // namespace uwb
} // namespace gr

#endif // UWB_SIMD_HAVE_AVX512
