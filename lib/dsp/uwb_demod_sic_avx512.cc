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

#include <openuwb/dsp/demod_sic_avx512.h>

#include <cstring>

#include <immintrin.h>

namespace openuwb {
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

UWB_DS_AVX512_TARGET void sparse_ternary_dot_16lag(
    const gr_complex* rx, const uint16_t* pos_offsets, size_t pos_count,
    const uint16_t* neg_offsets, size_t neg_count, gr_complex out[16])
{
    const float* rx_f = reinterpret_cast<const float*>(rx);
    __m512 p0 = _mm512_setzero_ps();
    __m512 p1 = _mm512_setzero_ps();
    for (size_t i = 0; i < pos_count; ++i) {
        const size_t off = pos_offsets[i];
        p0 = _mm512_add_ps(p0, _mm512_loadu_ps(rx_f + off));
        p1 = _mm512_add_ps(p1, _mm512_loadu_ps(rx_f + off + 16));
    }
    __m512 n0 = _mm512_setzero_ps();
    __m512 n1 = _mm512_setzero_ps();
    for (size_t i = 0; i < neg_count; ++i) {
        const size_t off = neg_offsets[i];
        n0 = _mm512_add_ps(n0, _mm512_loadu_ps(rx_f + off));
        n1 = _mm512_add_ps(n1, _mm512_loadu_ps(rx_f + off + 16));
    }
    _mm512_storeu_ps(reinterpret_cast<float*>(out), _mm512_sub_ps(p0, n0));
    _mm512_storeu_ps(reinterpret_cast<float*>(out + 8), _mm512_sub_ps(p1, n1));
}

UWB_DS_AVX512_TARGET void sparse_ternary_dot_32lag(
    const gr_complex* rx, const uint16_t* pos_offsets, size_t pos_count,
    const uint16_t* neg_offsets, size_t neg_count, gr_complex out[32])
{
    const float* rx_f = reinterpret_cast<const float*>(rx);
    // Four independent accumulator pairs.  Thirty-two lags = four 512-bit
    // registers (eight complex samples each), so each lane is one lag and its
    // accumulation order is identical to the 8-lag kernel for that lag.
    __m512 p0 = _mm512_setzero_ps();
    __m512 p1 = _mm512_setzero_ps();
    __m512 p2 = _mm512_setzero_ps();
    __m512 p3 = _mm512_setzero_ps();
    for (size_t i = 0; i < pos_count; ++i) {
        const size_t off = pos_offsets[i];
        p0 = _mm512_add_ps(p0, _mm512_loadu_ps(rx_f + off));
        p1 = _mm512_add_ps(p1, _mm512_loadu_ps(rx_f + off + 16));
        p2 = _mm512_add_ps(p2, _mm512_loadu_ps(rx_f + off + 32));
        p3 = _mm512_add_ps(p3, _mm512_loadu_ps(rx_f + off + 48));
    }
    __m512 n0 = _mm512_setzero_ps();
    __m512 n1 = _mm512_setzero_ps();
    __m512 n2 = _mm512_setzero_ps();
    __m512 n3 = _mm512_setzero_ps();
    for (size_t i = 0; i < neg_count; ++i) {
        const size_t off = neg_offsets[i];
        n0 = _mm512_add_ps(n0, _mm512_loadu_ps(rx_f + off));
        n1 = _mm512_add_ps(n1, _mm512_loadu_ps(rx_f + off + 16));
        n2 = _mm512_add_ps(n2, _mm512_loadu_ps(rx_f + off + 32));
        n3 = _mm512_add_ps(n3, _mm512_loadu_ps(rx_f + off + 48));
    }
    float* o = reinterpret_cast<float*>(out);
    _mm512_storeu_ps(o + 0, _mm512_sub_ps(p0, n0));
    _mm512_storeu_ps(o + 16, _mm512_sub_ps(p1, n1));
    _mm512_storeu_ps(o + 32, _mm512_sub_ps(p2, n2));
    _mm512_storeu_ps(o + 48, _mm512_sub_ps(p3, n3));
}

UWB_DS_AVX512_TARGET void pack_even_complex(const gr_complex* src,
                                            size_t even_count,
                                            gr_complex* dst)
{
    const __m512i idx = _mm512_set_epi64(14, 12, 10, 8, 6, 4, 2, 0);
    size_t i = 0;
    for (; i + 8 <= even_count; i += 8) {
        const __m512i v0 =
            _mm512_loadu_si512(reinterpret_cast<const void*>(src + 2 * i));
        const __m512i v1 =
            _mm512_loadu_si512(reinterpret_cast<const void*>(src + 2 * i + 8));
        const __m512i ev = _mm512_permutex2var_epi64(v0, idx, v1);
        _mm512_storeu_si512(reinterpret_cast<void*>(dst + i), ev);
    }
    for (; i < even_count; ++i)
        dst[i] = src[2 * i];
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

    // Single-pass fold.  The legacy implementation rewrote `out` once per SFD
    // symbol (one memcpy/sub, then seven read-modify-write passes).  Here every
    // output element is produced by loading all non-zero symbols for the
    // current chunk, accumulating them in symbol order in registers, and
    // storing once.  The per-element association is unchanged, so the result
    // stays bit-identical to the symbol-by-symbol reference.
    bool have_out = false;
    size_t idx[8];
    int8_t sg[8];
    for (size_t chunk = 0; chunk < sfd_len; chunk += 8) {
        const size_t chunk_end = std::min(chunk + 8, sfd_len);
        size_t cnt = 0;
        for (size_t i = chunk; i < chunk_end; ++i) {
            if (sfd_seq[i] == 0)
                continue;
            idx[cnt] = i;
            sg[cnt] = sfd_seq[i];
            ++cnt;
        }
        if (cnt == 0)
            continue;
        const bool first_chunk = !have_out;
        size_t k = 0;
        const size_t limit = fold_len & ~size_t(7);
        for (; k < limit; k += 8) {
            size_t j = 0;
            __m512 acc;
            if (first_chunk) {
                const __m512 v = _mm512_loadu_ps(reinterpret_cast<const float*>(
                    rx_base + idx[0] * sym + k));
                acc = (sg[0] == 1) ? v : _mm512_sub_ps(_mm512_setzero_ps(), v);
                j = 1;
            } else {
                acc = _mm512_loadu_ps(reinterpret_cast<const float*>(out + k));
            }
            for (; j < cnt; ++j) {
                const __m512 v = _mm512_loadu_ps(reinterpret_cast<const float*>(
                    rx_base + idx[j] * sym + k));
                acc = (sg[j] == 1) ? _mm512_add_ps(acc, v)
                                   : _mm512_sub_ps(acc, v);
            }
            _mm512_storeu_ps(reinterpret_cast<float*>(out + k), acc);
        }
        for (; k < fold_len; ++k) {
            size_t j = 0;
            gr_complex acc;
            if (first_chunk) {
                const gr_complex v = rx_base[idx[0] * sym + k];
                acc = (sg[0] == 1) ? v : gr_complex(0.f, 0.f) - v;
                j = 1;
            } else {
                acc = out[k];
            }
            for (; j < cnt; ++j) {
                const gr_complex v = rx_base[idx[j] * sym + k];
                acc = (sg[j] == 1) ? acc + v : acc - v;
            }
            out[k] = acc;
        }
        have_out = true;
    }
    if (!have_out)
        std::memset(out, 0, fold_len * sizeof(gr_complex));
}

} // namespace avx512
} // namespace simd
} // namespace core
} // namespace openuwb
