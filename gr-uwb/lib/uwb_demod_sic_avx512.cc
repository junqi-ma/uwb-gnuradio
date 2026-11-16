/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Phase-2 AVX-512 kernels for the demodulator and the SIC front end.
 *
 * See uwb_demod_sic_avx512.h for the numerical contract: every kernel keeps
 * the AVX2 lane mapping, FMA use, accumulation order and reduction tree, so
 * the output is bit-identical to the AVX2 reference.
 *
 * This file is compiled on the AVX2/FMA baseline only; the wide code is
 * isolated behind function target attributes.  A global -mavx512 flag would
 * let the compiler auto-vectorise unrelated code, and an unsupported machine
 * could then die on SIGILL before the runtime capability check.
 */

#include <gnuradio/uwb/uwb_demod_sic_avx512.h>

#include <cstring>

#include <immintrin.h>

namespace gr {
namespace uwb {
namespace core {
namespace simd {
namespace avx512 {

namespace {

#define UWB_DS_AVX512_TARGET \
    __attribute__((target("avx512f,avx512bw,avx512dq,avx512vl")))

/** AVX2 scalar reduction tree for one 256-bit accumulator (4 complex). */
inline gr_complex reduce_256(__m256 acc) noexcept
{
    alignas(32) float tmp[8];
    _mm256_storeu_ps(tmp, acc);
    return gr_complex(tmp[0] + tmp[2] + tmp[4] + tmp[6],
                      tmp[1] + tmp[3] + tmp[5] + tmp[7]);
}

UWB_DS_AVX512_TARGET inline void accumulate_ps(float* d, const float* s,
                                               size_t nf) noexcept
{
    size_t i = 0;
    const size_t limit = nf & ~size_t(15);
    for (; i < limit; i += 16)
        _mm512_storeu_ps(
            d + i, _mm512_add_ps(_mm512_loadu_ps(d + i), _mm512_loadu_ps(s + i)));
    for (; i < nf; ++i)
        d[i] += s[i];
}

UWB_DS_AVX512_TARGET inline void sub_ps(float* d, const float* s,
                                        size_t nf) noexcept
{
    size_t i = 0;
    const size_t limit = nf & ~size_t(15);
    for (; i < limit; i += 16)
        _mm512_storeu_ps(
            d + i, _mm512_sub_ps(_mm512_loadu_ps(d + i), _mm512_loadu_ps(s + i)));
    for (; i < nf; ++i)
        d[i] -= s[i];
}

UWB_DS_AVX512_TARGET inline void sub3_ps(float* d, const float* a,
                                         const float* b, size_t nf) noexcept
{
    size_t i = 0;
    const size_t limit = nf & ~size_t(15);
    for (; i < limit; i += 16)
        _mm512_storeu_ps(
            d + i, _mm512_sub_ps(_mm512_loadu_ps(a + i), _mm512_loadu_ps(b + i)));
    for (; i < nf; ++i)
        d[i] = a[i] - b[i];
}

} // namespace

bool available() noexcept
{
    return true;
}

UWB_DS_AVX512_TARGET void real_template_dot_8lag(const gr_complex* rx,
                                                 const float* taps_dup,
                                                 size_t L, gr_complex out[8])
{
    if (L == 0 || L > 65536) {
        for (int m = 0; m < 8; ++m)
            out[m] = gr_complex(0.f, 0.f);
        return;
    }
    const float* rx0 = reinterpret_cast<const float*>(rx);
    __m512 a0 = _mm512_setzero_ps();
    __m512 a1 = _mm512_setzero_ps();
    __m512 a2 = _mm512_setzero_ps();
    __m512 a3 = _mm512_setzero_ps();

    size_t k = 0;
    const size_t limit = L & ~size_t(3);
    for (; k < limit; k += 4) {
        // One 256-bit tap load broadcast into both halves: lanes 0..7 serve
        // lag m, lanes 8..15 serve lag m+4.
        const __m256 t = _mm256_loadu_ps(taps_dup + 2 * k);
        const __m512 tb = _mm512_broadcast_f32x8(t);
        a0 = _mm512_fmadd_ps(_mm512_loadu_ps(rx0 + 2 * (k + 0)), tb, a0);
        a1 = _mm512_fmadd_ps(_mm512_loadu_ps(rx0 + 2 * (k + 1)), tb, a1);
        a2 = _mm512_fmadd_ps(_mm512_loadu_ps(rx0 + 2 * (k + 2)), tb, a2);
        a3 = _mm512_fmadd_ps(_mm512_loadu_ps(rx0 + 2 * (k + 3)), tb, a3);
    }

    // Reduce each half with the AVX2 tree.  Low half = lag r, high half =
    // lag r+4.
    const __m512 acc[4] = { a0, a1, a2, a3 };
    gr_complex res[8];
    for (int r = 0; r < 4; ++r) {
        res[r] = reduce_256(_mm512_castps512_ps256(acc[r]));
        res[r + 4] = reduce_256(_mm512_extractf32x8_ps(acc[r], 1));
    }
    // Scalar tail for L % 4, same association as the AVX2 kernel.
    for (; k < L; ++k) {
        const float t = taps_dup[2 * k];
        for (int m = 0; m < 8; ++m) {
            float re = res[m].real() + rx0[2 * (k + m)] * t;
            float im = res[m].imag() + rx0[2 * (k + m) + 1] * t;
            res[m] = gr_complex(re, im);
        }
    }
    for (int m = 0; m < 8; ++m)
        out[m] = res[m];
}

UWB_DS_AVX512_TARGET void accumulate_cf32(gr_complex* dst,
                                          const gr_complex* src, size_t count)
{
    if (count == 0)
        return;
    accumulate_ps(reinterpret_cast<float*>(dst),
                  reinterpret_cast<const float*>(src), 2 * count);
}

UWB_DS_AVX512_TARGET void sub_cf32(gr_complex* dst, const gr_complex* src,
                                   size_t count)
{
    if (count == 0)
        return;
    sub_ps(reinterpret_cast<float*>(dst), reinterpret_cast<const float*>(src),
           2 * count);
}

UWB_DS_AVX512_TARGET void sub3_cf32(gr_complex* dst, const gr_complex* a,
                                    const gr_complex* b, size_t count)
{
    if (count == 0)
        return;
    sub3_ps(reinterpret_cast<float*>(dst), reinterpret_cast<const float*>(a),
            reinterpret_cast<const float*>(b), 2 * count);
}

UWB_DS_AVX512_TARGET void scale_cf32(gr_complex* dst, float scale,
                                     size_t count)
{
    if (count == 0)
        return;
    float* d = reinterpret_cast<float*>(dst);
    const size_t nf = 2 * count;
    const __m512 vs = _mm512_set1_ps(scale);
    size_t i = 0;
    const size_t limit = nf & ~size_t(15);
    for (; i < limit; i += 16)
        _mm512_storeu_ps(d + i, _mm512_mul_ps(_mm512_loadu_ps(d + i), vs));
    for (; i < nf; ++i)
        d[i] *= scale;
}

UWB_DS_AVX512_TARGET void fold_sfd_window(const gr_complex* rx_base,
                                          size_t sym, const int8_t* sfd_seq,
                                          size_t sfd_len, size_t fold_len,
                                          gr_complex* out)
{
    if (!rx_base || !sfd_seq || !out || sym == 0 || sfd_len == 0 ||
        fold_len == 0)
        return;
    size_t i = 0;
    while (i < sfd_len && sfd_seq[i] == 0)
        ++i;
    if (i == sfd_len) {
        std::memset(out, 0, fold_len * sizeof(gr_complex));
        return;
    }
    float* of = reinterpret_cast<float*>(out);
    const size_t nf = 2 * fold_len;
    if (sfd_seq[i] == 1) {
        std::memcpy(out, rx_base + i * sym, fold_len * sizeof(gr_complex));
    } else {
        // 0 - src, element-wise; matches the AVX2 fill + sub sequence.
        std::memset(out, 0, fold_len * sizeof(gr_complex));
        sub_ps(of, reinterpret_cast<const float*>(rx_base + i * sym), nf);
    }
    ++i;
    for (; i < sfd_len; ++i) {
        const int8_t s = sfd_seq[i];
        if (s == 1)
            accumulate_ps(of, reinterpret_cast<const float*>(rx_base + i * sym),
                          nf);
        else if (s == -1)
            sub_ps(of, reinterpret_cast<const float*>(rx_base + i * sym), nf);
    }
}

} // namespace avx512
} // namespace simd
} // namespace core
} // namespace uwb
} // namespace gr
