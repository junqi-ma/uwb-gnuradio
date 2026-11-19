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

} // namespace

UWB_TRANSPOSED_AVX512_TARGET
void transposed_process_window(const float* taps, size_t H, size_t Interp,
                               size_t Decim, const sc16* in, size_t ninput,
                               size_t n_mb_full, float* tile, gr_complex* out)
{
    const size_t slots = Decim + H - 1;
    const __m512i idx = _mm512_setr_epi64(
        0 * static_cast<long long>(Interp), 1 * static_cast<long long>(Interp),
        2 * static_cast<long long>(Interp), 3 * static_cast<long long>(Interp),
        4 * static_cast<long long>(Interp), 5 * static_cast<long long>(Interp),
        6 * static_cast<long long>(Interp), 7 * static_cast<long long>(Interp));

    for (size_t mb0 = 0; mb0 + 8 <= n_mb_full; mb0 += 8) {
        for (size_t u = 0; u < slots; ++u)
            load_tile_slot(in, ninput, mb0, u, H, Decim, tile + u * 16);

        for (size_t j = 0; j < Interp; ++j) {
            const size_t arm = (j * Decim) % Interp;
            const size_t w = (j * Decim) / Interp;
            const __m512 acc = phase_accum(taps + arm * H, tile + w * 16, H);
            // out[(mb0+m)*Interp + j] for m = 0..7, one complex each; the
            // scatter writes the 64-bit complex pairs without materialising an
            // 8x8 transpose in registers.
            _mm512_i64scatter_pd(
                reinterpret_cast<double*>(out + mb0 * Interp + j), idx,
                _mm512_castps_pd(acc), 8);
        }
    }
}

} // namespace avx512
} // namespace transposed
} // namespace simd
} // namespace core
} // namespace uwb
} // namespace gr

#endif // UWB_SIMD_HAVE_AVX512
