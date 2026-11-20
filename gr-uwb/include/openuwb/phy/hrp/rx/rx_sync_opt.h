/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Experimental RX fast-path helpers: SFD template caching and the
 * SFD-anchored first-SYNC refine radius.  Header-only and GNU-Radio-runtime
 * independent, mirroring uwb_demod_core.h.
 *
 * Every helper is opt-in through Qm35825Profile.  With the experimental flags
 * disabled the callers keep the legacy algorithms unchanged.
 *
 * Threading: the cache lives in DemodScratch, which is per worker.  Nothing
 * here is shared between workers or allocated per candidate.
 */

#pragma once

#include <openuwb/dsp/detector_core.h> // gr::uwb::core::uwb_l2_normalize
#include <openuwb/sic/demod_sic_avx512.h> // phase-2 wide kernels
#include <openuwb/support/kernel_backend.h>   // per-group backend policy
#include <openuwb/phy/hrp/common/profile.h>      // kQm35SamplesPerSymbol
#include <gnuradio/uwb/uwb_simd_config.h>      // UWB_SIMD_HAVE_AVX2

#include <volk/volk.h>
#if defined(__x86_64__) || defined(_M_X64)
#include <immintrin.h>
#endif

#include <algorithm>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <vector>

// The hand-written AVX2 kernels in this file are gated by the compile-time
// backend, so UWB_SIMD=SCALAR really is a scalar build for the kernels this
// file owns.  (UWB_SIMD=AVX512 keeps the AVX2 reference: the two must coexist
// in one binary for the paired A/B runs.)
#if (defined(__x86_64__) || defined(_M_X64)) && \
    (defined(__GNUC__) || defined(__clang__)) && UWB_SIMD_HAVE_AVX2
#define UWB_RXOPT_HAVE_AVX2 1
#else
#define UWB_RXOPT_HAVE_AVX2 0
#endif

namespace gr {
namespace uwb {
namespace demod {
namespace rxopt {

using gr::uwb::core::conjugate_dot;

// ---------------------------------------------------------------------------
// Pre-duplicated real-valued SYNC template for AVX2 fast dot product.
// In IEEE 802.15.4z baseband, the pulse-shaped SYNC sequence is purely real.
// Storing [t0, t0, t1, t1, t2, t2, ...] allows direct FMA with interleaved
// complex rx [re0, im0, re1, im1, ...] without unpacking or cross-multiplies.
// ---------------------------------------------------------------------------
struct RealSyncTemplate {
    uint64_t key = 0;
    bool is_real = false;
    size_t length = 0;
    std::vector<float> taps_dup;
    float energy = 0.0f;

    void reset()
    {
        key = 0;
        is_real = false;
        length = 0;
        taps_dup.clear();
        energy = 0.0f;
    }
};

inline void build_real_sync_template(const std::vector<std::complex<float>>& template_wf,
                                     RealSyncTemplate& out)
{
    out.reset();
    if (template_wf.empty())
        return;
    const size_t L = template_wf.size();
    bool real_only = true;
    for (size_t k = 0; k < L; ++k) {
        if (std::abs(template_wf[k].imag()) > 1e-7f) {
            real_only = false;
            break;
        }
    }
    out.is_real = real_only;
    out.length = L;
    if (real_only) {
        out.taps_dup.resize(2 * L);
        float e = 0.0f;
        for (size_t k = 0; k < L; ++k) {
            const float r = template_wf[k].real();
            out.taps_dup[2 * k] = r;
            out.taps_dup[2 * k + 1] = r;
            e += r * r;
        }
        out.energy = e;
    }
}

// FNV-1a over the template's raw float words.  Only a content key is needed,
// so hashing one 32-bit word per real/imag component (2 per complex) is
// sufficient and much cheaper than a byte-wise hash on the hot path.  Letting
// the caller skip a rebuild also avoids the L2 normalize + duplicate copy on
// the sparse path entirely.
inline uint64_t real_sync_template_key(
    const std::vector<std::complex<float>>& template_wf)
{
    static_assert(sizeof(float) == 4, "template key assumes 32-bit float");
    uint64_t h = 14695981039346656037ULL;
    for (const std::complex<float>& v : template_wf) {
        const float parts[2] = { v.real(), v.imag() };
        uint32_t bits[2];
        std::memcpy(bits, parts, sizeof(bits));
        h = (h ^ static_cast<uint64_t>(bits[0])) * 1099511628211ULL;
        h = (h ^ static_cast<uint64_t>(bits[1])) * 1099511628211ULL;
    }
    return h;
}

inline void ensure_real_sync_template(
    const std::vector<std::complex<float>>& template_wf,
    RealSyncTemplate& out)
{
    const uint64_t key = real_sync_template_key(template_wf);
    if (key == out.key && out.length == template_wf.size())
        return;
    build_real_sync_template(template_wf, out);
    out.key = key;
}

#if UWB_RXOPT_HAVE_AVX2
inline bool has_avx2_fma()
{
    static const bool s_has =
        __builtin_cpu_supports("avx2") && __builtin_cpu_supports("fma");
    return s_has;
}

__attribute__((target("avx2,fma")))
inline void real_template_dot_4lag_avx2(const std::complex<float>* rx,
                                        const float* taps_dup,
                                        size_t L,
                                        std::complex<float> out[4])
{
    if (L == 0 || L > 65536) {
        out[0] = out[1] = out[2] = out[3] = std::complex<float>(0.f, 0.f);
        return;
    }
    const float* rx0 = reinterpret_cast<const float*>(rx);
    const float* rx1 = reinterpret_cast<const float*>(rx + 1);
    const float* rx2 = reinterpret_cast<const float*>(rx + 2);
    const float* rx3 = reinterpret_cast<const float*>(rx + 3);
    __m256 a0 = _mm256_setzero_ps();
    __m256 a1 = _mm256_setzero_ps();
    __m256 a2 = _mm256_setzero_ps();
    __m256 a3 = _mm256_setzero_ps();

    size_t k = 0;
    const size_t limit = L & ~size_t(3);
    for (; k < limit; k += 4) {
        __m256 t0 = _mm256_loadu_ps(taps_dup + 2 * k);
        __m256 r0 = _mm256_loadu_ps(rx0 + 2 * k);
        __m256 r1 = _mm256_loadu_ps(rx1 + 2 * k);
        __m256 r2 = _mm256_loadu_ps(rx2 + 2 * k);
        __m256 r3 = _mm256_loadu_ps(rx3 + 2 * k);

        a0 = _mm256_fmadd_ps(r0, t0, a0);
        a1 = _mm256_fmadd_ps(r1, t0, a1);
        a2 = _mm256_fmadd_ps(r2, t0, a2);
        a3 = _mm256_fmadd_ps(r3, t0, a3);
    }

    alignas(32) float tmp0[8], tmp1[8], tmp2[8], tmp3[8];
    _mm256_storeu_ps(tmp0, a0);
    _mm256_storeu_ps(tmp1, a1);
    _mm256_storeu_ps(tmp2, a2);
    _mm256_storeu_ps(tmp3, a3);

    float s0_re = tmp0[0] + tmp0[2] + tmp0[4] + tmp0[6];
    float s0_im = tmp0[1] + tmp0[3] + tmp0[5] + tmp0[7];
    float s1_re = tmp1[0] + tmp1[2] + tmp1[4] + tmp1[6];
    float s1_im = tmp1[1] + tmp1[3] + tmp1[5] + tmp1[7];
    float s2_re = tmp2[0] + tmp2[2] + tmp2[4] + tmp2[6];
    float s2_im = tmp2[1] + tmp2[3] + tmp2[5] + tmp2[7];
    float s3_re = tmp3[0] + tmp3[2] + tmp3[4] + tmp3[6];
    float s3_im = tmp3[1] + tmp3[3] + tmp3[5] + tmp3[7];

    for (; k < L; ++k) {
        float t = taps_dup[2 * k];
        s0_re += rx0[2 * k] * t; s0_im += rx0[2 * k + 1] * t;
        s1_re += rx1[2 * k] * t; s1_im += rx1[2 * k + 1] * t;
        s2_re += rx2[2 * k] * t; s2_im += rx2[2 * k + 1] * t;
        s3_re += rx3[2 * k] * t; s3_im += rx3[2 * k + 1] * t;
    }
    out[0] = std::complex<float>(s0_re, s0_im);
    out[1] = std::complex<float>(s1_re, s1_im);
    out[2] = std::complex<float>(s2_re, s2_im);
    out[3] = std::complex<float>(s3_re, s3_im);
}

__attribute__((target("avx2,fma")))
inline std::complex<float> real_template_dot_single_avx2(const std::complex<float>* rx,
                                                         const float* taps_dup,
                                                         size_t L)
{
    if (L == 0 || L > 65536)
        return std::complex<float>(0.f, 0.f);
    const float* rx0 = reinterpret_cast<const float*>(rx);
    __m256 a0 = _mm256_setzero_ps();
    size_t k = 0;
    const size_t limit = L & ~size_t(3);
    for (; k < limit; k += 4) {
        __m256 t0 = _mm256_loadu_ps(taps_dup + 2 * k);
        __m256 r0 = _mm256_loadu_ps(rx0 + 2 * k);
        a0 = _mm256_fmadd_ps(r0, t0, a0);
    }
    alignas(32) float tmp0[8];
    _mm256_storeu_ps(tmp0, a0);
    float s0_re = tmp0[0] + tmp0[2] + tmp0[4] + tmp0[6];
    float s0_im = tmp0[1] + tmp0[3] + tmp0[5] + tmp0[7];
    for (; k < L; ++k) {
        float t = taps_dup[2 * k];
        s0_re += rx0[2 * k] * t;
        s0_im += rx0[2 * k + 1] * t;
    }
    return std::complex<float>(s0_re, s0_im);
}
#endif

inline void real_template_dot_4lag(const std::complex<float>* rx,
                                   const float* taps_dup,
                                   size_t L,
                                   std::complex<float> out[4])
{
#if UWB_RXOPT_HAVE_AVX2
    if (has_avx2_fma()) {
        real_template_dot_4lag_avx2(rx, taps_dup, L, out);
        return;
    }
#endif
    out[0] = out[1] = out[2] = out[3] = std::complex<float>(0.f, 0.f);
    // Same length contract as the AVX2 kernel: an out-of-range L is a caller
    // bug, and the reference must not read past the buffer to report it.
    if (L == 0 || L > 65536)
        return;
    for (size_t k = 0; k < L; ++k) {
        float t = taps_dup[2 * k];
        out[0] += rx[k + 0] * t;
        out[1] += rx[k + 1] * t;
        out[2] += rx[k + 2] * t;
        out[3] += rx[k + 3] * t;
    }
}

inline std::complex<float> real_template_dot_single(const std::complex<float>* rx,
                                                    const float* taps_dup,
                                                    size_t L)
{
#if UWB_RXOPT_HAVE_AVX2
    if (has_avx2_fma()) {
        return real_template_dot_single_avx2(rx, taps_dup, L);
    }
#endif
    std::complex<float> acc(0.f, 0.f);
    if (L == 0 || L > 65536)
        return acc;
    for (size_t k = 0; k < L; ++k) {
        acc += rx[k] * taps_dup[2 * k];
    }
    return acc;
}

#if UWB_RXOPT_HAVE_AVX2
__attribute__((target("avx2,fma")))
inline void sfd_real_dot_4lag_avx2(const std::complex<float>* rx,
                                   const float* taps_dup,
                                   size_t sym,
                                   const int8_t* sfd_seq,
                                   size_t sfd_len,
                                   float sfd_scale,
                                   std::complex<float> out[4])
{
    out[0] = out[1] = out[2] = out[3] = std::complex<float>(0.0f, 0.0f);
    if (!rx || !taps_dup || !sfd_seq || sym == 0 || sfd_len == 0)
        return;

    for (size_t i = 0; i < sfd_len; ++i) {
        const int8_t s = sfd_seq[i];
        if (s == 0)
            continue;
        std::complex<float> sym_dots[4];
        real_template_dot_4lag_avx2(rx + i * sym, taps_dup, sym, sym_dots);
        if (s > 0) {
            out[0] += sym_dots[0];
            out[1] += sym_dots[1];
            out[2] += sym_dots[2];
            out[3] += sym_dots[3];
        } else {
            out[0] -= sym_dots[0];
            out[1] -= sym_dots[1];
            out[2] -= sym_dots[2];
            out[3] -= sym_dots[3];
        }
    }
    out[0] *= sfd_scale;
    out[1] *= sfd_scale;
    out[2] *= sfd_scale;
    out[3] *= sfd_scale;
}
#endif

inline void sfd_real_dot_4lag(const std::complex<float>* rx,
                              const float* taps_dup,
                              size_t sym,
                              const int8_t* sfd_seq,
                              size_t sfd_len,
                              float sfd_scale,
                              std::complex<float> out[4])
{
#if UWB_RXOPT_HAVE_AVX2
    if (has_avx2_fma()) {
        sfd_real_dot_4lag_avx2(rx, taps_dup, sym, sfd_seq, sfd_len, sfd_scale, out);
        return;
    }
#endif
    out[0] = out[1] = out[2] = out[3] = std::complex<float>(0.0f, 0.0f);
    if (!rx || !taps_dup || !sfd_seq || sym == 0 || sfd_len == 0)
        return;

    for (size_t i = 0; i < sfd_len; ++i) {
        const int8_t s = sfd_seq[i];
        if (s == 0)
            continue;
        std::complex<float> sym_dots[4];
        real_template_dot_4lag(rx + i * sym, taps_dup, sym, sym_dots);
        if (s > 0) {
            out[0] += sym_dots[0];
            out[1] += sym_dots[1];
            out[2] += sym_dots[2];
            out[3] += sym_dots[3];
        } else {
            out[0] -= sym_dots[0];
            out[1] -= sym_dots[1];
            out[2] -= sym_dots[2];
            out[3] -= sym_dots[3];
        }
    }
    out[0] *= sfd_scale;
    out[1] *= sfd_scale;
    out[2] *= sfd_scale;
    out[3] *= sfd_scale;
}

inline std::complex<float> sfd_real_dot_single(const std::complex<float>* rx,
                                               const float* taps_dup,
                                               size_t sym,
                                               const int8_t* sfd_seq,
                                               size_t sfd_len,
                                               float sfd_scale)
{
    std::complex<float> acc(0.0f, 0.0f);
    if (!rx || !taps_dup || !sfd_seq || sym == 0 || sfd_len == 0)
        return acc;

    for (size_t i = 0; i < sfd_len; ++i) {
        const int8_t s = sfd_seq[i];
        if (s == 0)
            continue;
        const std::complex<float> sym_dot =
            real_template_dot_single(rx + i * sym, taps_dup, sym);
        acc += (s > 0 ? sym_dot : -sym_dot);
    }
    return acc * sfd_scale;
}

#if UWB_RXOPT_HAVE_AVX2
__attribute__((target("avx2")))
inline void vector_accumulate_complex_avx2(std::complex<float>* dst,
                                           const std::complex<float>* src,
                                           size_t count)
{
    float* d = reinterpret_cast<float*>(dst);
    const float* s = reinterpret_cast<const float*>(src);
    const size_t num_floats = 2 * count;
    size_t i = 0;
    const size_t limit = num_floats & ~size_t(7);
    for (; i < limit; i += 8) {
        __m256 vd = _mm256_loadu_ps(d + i);
        __m256 vs = _mm256_loadu_ps(s + i);
        _mm256_storeu_ps(d + i, _mm256_add_ps(vd, vs));
    }
    for (; i < num_floats; ++i) {
        d[i] += s[i];
    }
}

__attribute__((target("avx2")))
inline void vector_sub_complex_avx2(std::complex<float>* dst,
                                    const std::complex<float>* src,
                                    size_t count)
{
    float* d = reinterpret_cast<float*>(dst);
    const float* s = reinterpret_cast<const float*>(src);
    const size_t num_floats = 2 * count;
    size_t i = 0;
    const size_t limit = num_floats & ~size_t(7);
    for (; i < limit; i += 8) {
        __m256 vd = _mm256_loadu_ps(d + i);
        __m256 vs = _mm256_loadu_ps(s + i);
        _mm256_storeu_ps(d + i, _mm256_sub_ps(vd, vs));
    }
    for (; i < num_floats; ++i) {
        d[i] -= s[i];
    }
}

__attribute__((target("avx2")))
inline void vector_scale_complex_avx2(std::complex<float>* dst,
                                      float scale,
                                      size_t count)
{
    float* d = reinterpret_cast<float*>(dst);
    const size_t num_floats = 2 * count;
    size_t i = 0;
    const size_t limit = num_floats & ~size_t(7);
    __m256 vs = _mm256_set1_ps(scale);
    for (; i < limit; i += 8) {
        __m256 vd = _mm256_loadu_ps(d + i);
        _mm256_storeu_ps(d + i, _mm256_mul_ps(vd, vs));
    }
    for (; i < num_floats; ++i) {
        d[i] *= scale;
    }
}
#endif

// ---------------------------------------------------------------------------
// Phase-2 backend dispatch.
//
// resolve_backend() consults the diagnostic override first, then the measured
// per-group default policy, then what the build/CPU can actually run.  It is a
// plain out-of-line call (one relaxed atomic load), so it is only used at
// batch boundaries: once per 8-lag dot, once per whole fold, once per whole
// vector op -- never per tap, chip or sample.
// ---------------------------------------------------------------------------
inline bool use_avx512(const gr::uwb::core::simd::KernelGroup group)
{
#if UWB_SIMD_HAVE_AVX512
    // The wide translation unit only exists in an AVX-512 build, so the call
    // sites below are compiled out otherwise (no dangling link reference).
    return gr::uwb::core::simd::resolve_backend(group) ==
           gr::uwb::core::simd::Backend::Avx512;
#else
    (void)group;
    return false;
#endif
}

// Eight consecutive lags in one call.  With the AVX-512 backend this is a
// single wide kernel (-37% per lag, bit-identical); otherwise it is two
// unchanged AVX2 4-lag calls, so the fallback is exactly the legacy math.
inline void real_template_dot_8lag(const std::complex<float>* rx,
                                   const float* taps_dup,
                                   size_t L,
                                   std::complex<float> out[8])
{
#if UWB_SIMD_HAVE_AVX512
    if (use_avx512(gr::uwb::core::simd::KernelGroup::Sync)) {
        gr::uwb::core::simd::avx512::real_template_dot_8lag(rx, taps_dup, L,
                                                            out);
        return;
    }
#endif
    real_template_dot_4lag(rx, taps_dup, L, out);
    real_template_dot_4lag(rx + 4, taps_dup, L, out + 4);
}

inline void vector_accumulate_complex(std::complex<float>* dst,
                                      const std::complex<float>* src,
                                      size_t count)
{
    if (count == 0)
        return;
#if UWB_SIMD_HAVE_AVX512
    if (use_avx512(gr::uwb::core::simd::KernelGroup::Sync)) {
        gr::uwb::core::simd::avx512::accumulate_cf32(dst, src, count);
        return;
    }
#endif
#if UWB_RXOPT_HAVE_AVX2
    if (has_avx2_fma()) {
        vector_accumulate_complex_avx2(dst, src, count);
        return;
    }
#endif
    for (size_t m = 0; m < count; ++m)
        dst[m] += src[m];
}

inline void vector_sub_complex(std::complex<float>* dst,
                               const std::complex<float>* src,
                               size_t count)
{
    if (count == 0)
        return;
#if UWB_SIMD_HAVE_AVX512
    if (use_avx512(gr::uwb::core::simd::KernelGroup::Sync)) {
        gr::uwb::core::simd::avx512::sub_cf32(dst, src, count);
        return;
    }
#endif
#if UWB_RXOPT_HAVE_AVX2
    if (has_avx2_fma()) {
        vector_sub_complex_avx2(dst, src, count);
        return;
    }
#endif
    for (size_t m = 0; m < count; ++m)
        dst[m] -= src[m];
}

// dst = a - b (three-operand form).  Used by the SIC transactional
// subtraction, which cannot use the in-place kernel because out may not alias
// the received buffer.  Element-wise, so widening is transparent.
inline void vector_sub3_complex(std::complex<float>* dst,
                                const std::complex<float>* a,
                                const std::complex<float>* b,
                                size_t count)
{
    if (count == 0)
        return;
#if UWB_SIMD_HAVE_AVX512
    if (use_avx512(gr::uwb::core::simd::KernelGroup::Sync)) {
        gr::uwb::core::simd::avx512::sub3_cf32(dst, a, b, count);
        return;
    }
#endif
    for (size_t m = 0; m < count; ++m)
        dst[m] = a[m] - b[m];
}

#if UWB_RXOPT_HAVE_AVX2
__attribute__((target("avx2")))
inline void fold_sfd_window_avx2(const std::complex<float>* rx_base,
                                 size_t sym, const int8_t* sfd_seq,
                                 size_t sfd_len, size_t fold_len,
                                 std::complex<float>* out)
{
    // Single pass: load every non-zero SFD symbol for the current chunk, add
    // them into one register pair in symbol order, store once.  The per-element
    // association is the legacy one, hence bit-identical output.
    size_t idx[8];
    int8_t sg[8];
    bool have_out = false;
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
        const size_t limit = fold_len & ~size_t(3);
        for (; k < limit; k += 4) {
            size_t j = 0;
            __m256 acc;
            if (first_chunk) {
                const __m256 v = _mm256_loadu_ps(reinterpret_cast<const float*>(
                    rx_base + idx[0] * sym + k));
                acc = (sg[0] == 1) ? v : _mm256_sub_ps(_mm256_setzero_ps(), v);
                j = 1;
            } else {
                acc = _mm256_loadu_ps(reinterpret_cast<const float*>(out + k));
            }
            for (; j < cnt; ++j) {
                const __m256 v = _mm256_loadu_ps(reinterpret_cast<const float*>(
                    rx_base + idx[j] * sym + k));
                acc = (sg[j] == 1) ? _mm256_add_ps(acc, v)
                                   : _mm256_sub_ps(acc, v);
            }
            _mm256_storeu_ps(reinterpret_cast<float*>(out + k), acc);
        }
        for (; k < fold_len; ++k) {
            size_t j = 0;
            std::complex<float> acc;
            if (first_chunk) {
                const std::complex<float> v = rx_base[idx[0] * sym + k];
                acc = (sg[0] == 1) ? v : std::complex<float>(0.f, 0.f) - v;
                j = 1;
            } else {
                acc = out[k];
            }
            for (; j < cnt; ++j) {
                const std::complex<float> v = rx_base[idx[j] * sym + k];
                acc = (sg[j] == 1) ? acc + v : acc - v;
            }
            out[k] = acc;
        }
        have_out = true;
    }
    if (!have_out)
        std::fill(out, out + fold_len, std::complex<float>(0.0f, 0.0f));
}
#endif

inline void fold_sfd_window(const std::complex<float>* rx_base,
                            size_t sym,
                            const int8_t* sfd_seq,
                            size_t sfd_len,
                            size_t fold_len,
                            std::complex<float>* out)
{
    if (!rx_base || !sfd_seq || !out || sym == 0 || sfd_len == 0 || fold_len == 0)
        return;
#if UWB_SIMD_HAVE_AVX512
    if (use_avx512(gr::uwb::core::simd::KernelGroup::Sync)) {
        // Batched: the whole symbol loop runs inside the wide translation
        // unit, so the fold pays one call instead of one per symbol.
        gr::uwb::core::simd::avx512::fold_sfd_window(rx_base, sym, sfd_seq,
                                                     sfd_len, fold_len, out);
        return;
    }
#endif
#if UWB_RXOPT_HAVE_AVX2
    if (has_avx2_fma()) {
        fold_sfd_window_avx2(rx_base, sym, sfd_seq, sfd_len, fold_len, out);
        return;
    }
#endif
    size_t idx[8];
    int8_t sg[8];
    bool have_out = false;
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
        for (size_t k = 0; k < fold_len; ++k) {
            size_t j = 0;
            std::complex<float> acc;
            if (first_chunk) {
                const std::complex<float> v = rx_base[idx[0] * sym + k];
                acc = (sg[0] == 1) ? v : std::complex<float>(0.0f, 0.0f) - v;
                j = 1;
            } else {
                acc = out[k];
            }
            for (; j < cnt; ++j) {
                const std::complex<float> v = rx_base[idx[j] * sym + k];
                acc = (sg[j] == 1) ? acc + v : acc - v;
            }
            out[k] = acc;
        }
        have_out = true;
    }
    if (!have_out)
        std::fill(out, out + fold_len, std::complex<float>(0.0f, 0.0f));
}

#if UWB_RXOPT_HAVE_AVX2
__attribute__((target("avx2")))
inline void vector_accumulate_scaled_complex_avx2(std::complex<float>* dst,
                                                  const std::complex<float>* src,
                                                  float c, float s, size_t count)
{
    const __m256 vc = _mm256_set1_ps(c);
    const __m256 vs = _mm256_set1_ps(s);
    size_t j = 0;
    const size_t limit = count & ~size_t(3);
    for (; j < limit; j += 4) {
        const __m256 v =
            _mm256_loadu_ps(reinterpret_cast<const float*>(src + j));
        const __m256 sw = _mm256_permute_ps(v, 0xB1);
        const __m256 rot = _mm256_addsub_ps(_mm256_mul_ps(v, vc),
                                            _mm256_mul_ps(sw, vs));
        const __m256 d =
            _mm256_loadu_ps(reinterpret_cast<const float*>(dst + j));
        _mm256_storeu_ps(reinterpret_cast<float*>(dst + j),
                         _mm256_add_ps(d, rot));
    }
    for (; j < count; ++j) {
        const auto v = src[j];
        dst[j] += std::complex<float>(c * v.real() - s * v.imag(),
                                      s * v.real() + c * v.imag());
    }
}

__attribute__((target("avx2")))
inline void vector_mul_complex_avx2(std::complex<float>* dst,
                                    const std::complex<float>* mul,
                                    size_t count)
{
    size_t j = 0;
    const size_t limit = count & ~size_t(3);
    for (; j < limit; j += 4) {
        const __m256 v =
            _mm256_loadu_ps(reinterpret_cast<const float*>(dst + j));
        const __m256 b =
            _mm256_loadu_ps(reinterpret_cast<const float*>(mul + j));
        const __m256 br = _mm256_moveldup_ps(b);
        const __m256 bi = _mm256_movehdup_ps(b);
        const __m256 prod = _mm256_addsub_ps(
            _mm256_mul_ps(v, br), _mm256_mul_ps(_mm256_permute_ps(v, 0xB1), bi));
        _mm256_storeu_ps(reinterpret_cast<float*>(dst + j), prod);
    }
    for (; j < count; ++j)
        dst[j] *= mul[j];
}
#endif

inline void vector_accumulate_scaled_complex(std::complex<float>* dst,
                                             const std::complex<float>* src,
                                             std::complex<float> alpha,
                                             size_t count)
{
    if (count == 0)
        return;
#if UWB_RXOPT_HAVE_AVX2
    if (has_avx2_fma()) {
        vector_accumulate_scaled_complex_avx2(dst, src, alpha.real(),
                                              alpha.imag(), count);
        return;
    }
#endif
    for (size_t j = 0; j < count; ++j)
        dst[j] += alpha * src[j];
}

inline void vector_mul_complex(std::complex<float>* dst,
                               const std::complex<float>* mul,
                               size_t count)
{
    if (count == 0)
        return;
#if UWB_RXOPT_HAVE_AVX2
    if (has_avx2_fma()) {
        vector_mul_complex_avx2(dst, mul, count);
        return;
    }
#endif
    for (size_t j = 0; j < count; ++j)
        dst[j] *= mul[j];
}

// dst[i] = exp(-j*w*i), i < count, using the same 4-chain / 1024-sample
// re-anchoring as derotate_range_recurrence().  Used by the fused CIR
// estimator, which applies the per-sample phase once outside the SYNC loop.
inline void build_rotation_ramp(double w, size_t count,
                                std::complex<float>* table)
{
    if (count == 0)
        return;
    constexpr size_t kB = 1024;
    const float s4r = static_cast<float>(std::cos(-4.0 * w));
    const float s4i = static_cast<float>(std::sin(-4.0 * w));
    for (size_t b0 = 0; b0 < count; b0 += kB) {
        const size_t b1 = std::min(count, b0 + kB);
        float cr[4], ci[4];
        for (int c = 0; c < 4; ++c) {
            const double ph = -w * static_cast<double>(b0 + c);
            cr[c] = static_cast<float>(std::cos(ph));
            ci[c] = static_cast<float>(std::sin(ph));
        }
        size_t i = b0;
        for (; i + 4 <= b1; i += 4) {
            for (int c = 0; c < 4; ++c) {
                table[i + c] = std::complex<float>(cr[c], ci[c]);
                const float nr = cr[c] * s4r - ci[c] * s4i;
                const float ni = cr[c] * s4i + ci[c] * s4r;
                cr[c] = nr;
                ci[c] = ni;
            }
        }
        for (; i < b1; ++i) {
            const double ph = -w * static_cast<double>(i);
            table[i] = std::complex<float>(static_cast<float>(std::cos(ph)),
                                           static_cast<float>(std::sin(ph)));
        }
    }
}

inline void vector_scale_complex(std::complex<float>* dst,
                                 float scale,
                                 size_t count)
{
    if (count == 0)
        return;
#if UWB_SIMD_HAVE_AVX512
    if (use_avx512(gr::uwb::core::simd::KernelGroup::Sync)) {
        gr::uwb::core::simd::avx512::scale_cf32(dst, scale, count);
        return;
    }
#endif
#if UWB_RXOPT_HAVE_AVX2
    if (has_avx2_fma()) {
        vector_scale_complex_avx2(dst, scale, count);
        return;
    }
#endif
    for (size_t m = 0; m < count; ++m)
        dst[m] *= scale;
}

namespace detail {

// FNV-1a (64-bit).  Multi-byte values are fed little-endian so the key is
// reproducible across hosts and does not depend on struct padding.
inline constexpr uint64_t kFnvOffsetBasis = 14695981039346656037ULL;
inline constexpr uint64_t kFnvPrime = 1099511628211ULL;

inline uint64_t fnv1a_bytes(uint64_t h, const void* data, size_t count)
{
    const uint8_t* p = static_cast<const uint8_t*>(data);
    for (size_t i = 0; i < count; ++i) {
        h ^= static_cast<uint64_t>(p[i]);
        h *= kFnvPrime;
    }
    return h;
}

inline uint64_t fnv1a_u32(uint64_t h, uint32_t v)
{
    uint8_t buf[4];
    for (size_t i = 0; i < sizeof(buf); ++i)
        buf[i] = static_cast<uint8_t>((v >> (8 * i)) & 0xffu);
    return fnv1a_bytes(h, buf, sizeof(buf));
}

inline uint64_t fnv1a_u64(uint64_t h, uint64_t v)
{
    uint8_t buf[8];
    for (size_t i = 0; i < sizeof(buf); ++i)
        buf[i] = static_cast<uint8_t>((v >> (8 * i)) & 0xffu);
    return fnv1a_bytes(h, buf, sizeof(buf));
}

} // namespace detail

// ---------------------------------------------------------------------------
// Cache key for the normalized SFD template.
//
// Covers, in order: template length, every template sample's raw float bit
// pattern, SFD sequence length, and the int8 SFD values.  It deliberately does
// NOT look at DemodScratch work/corr contents or at any pointer identity, so a
// cached template stays valid across jobs and scratch reallocations.
//
// Callers must also require a non-empty cached template before reuse: a key of
// 0 is also the "never built" state.
// ---------------------------------------------------------------------------
inline uint64_t
sfd_template_key(const std::vector<std::complex<float>>& template_wf,
                 const std::vector<int8_t>& sfd_sequence)
{
    static_assert(sizeof(float) == 4, "SFD cache key assumes 32-bit float");
    uint64_t h = detail::kFnvOffsetBasis;
    h = detail::fnv1a_u64(h, static_cast<uint64_t>(template_wf.size()));
    for (const std::complex<float>& v : template_wf) {
        uint32_t re_bits = 0;
        uint32_t im_bits = 0;
        const float re = v.real();
        const float im = v.imag();
        std::memcpy(&re_bits, &re, sizeof(re_bits));
        std::memcpy(&im_bits, &im, sizeof(im_bits));
        h = detail::fnv1a_u32(h, re_bits);
        h = detail::fnv1a_u32(h, im_bits);
    }
    h = detail::fnv1a_u64(h, static_cast<uint64_t>(sfd_sequence.size()));
    if (!sfd_sequence.empty())
        h = detail::fnv1a_bytes(h, sfd_sequence.data(), sfd_sequence.size());
    return h;
}

// ---------------------------------------------------------------------------
// Build the normalized SFD waveform: kron(sfd_sequence, template_wf), then
// L2-normalize.  This reproduces the legacy stage_sfd construction exactly,
// including its per-symbol length of kQm35SamplesPerSymbol samples (the legacy
// loop reads template_wf[0..kQm35SamplesPerSymbol), not template_wf.size()).
//
// May allocate (a cache miss only).  On invalid input the output is cleared so
// callers can fail closed; the cache key must only be committed when the
// output is non-empty.
// ---------------------------------------------------------------------------
inline void
build_sfd_template(const std::vector<std::complex<float>>& template_wf,
                   const std::vector<int8_t>& sfd_sequence,
                   std::vector<std::complex<float>>& out)
{
    out.clear();
    const size_t sym = kQm35SamplesPerSymbol;
    if (sfd_sequence.empty() || template_wf.size() < sym)
        return; // the legacy loop would read past the template: fail closed
    if (sfd_sequence.size() > std::numeric_limits<size_t>::max() / sym)
        return;
    out.resize(sfd_sequence.size() * sym);
    for (size_t i = 0; i < sfd_sequence.size(); ++i) {
        const int8_t s = sfd_sequence[i];
        for (size_t k = 0; k < sym; ++k)
            out[i * sym + k] =
                std::complex<float>(static_cast<float>(s) * template_wf[k].real(),
                                    static_cast<float>(s) * template_wf[k].imag());
    }
    gr::uwb::core::uwb_l2_normalize(out);
}

// ---------------------------------------------------------------------------
// Half-width of the first seeded SYNC refine.
//
// configured_radius == 0 keeps the legacy max(256, timing_coarse_stride)
// behaviour; a nonzero configured radius is the opt-in narrow search and is
// clamped to at least 1 sample.  Mirrors the legacy integer promotion exactly
// so the fallback radius is unchanged.
// ---------------------------------------------------------------------------
inline int64_t seed_refine_radius(size_t configured_radius, size_t stride)
{
    if (configured_radius != 0)
        return std::max<int64_t>(1, static_cast<int64_t>(configured_radius));
    return std::max<int64_t>(256, static_cast<int64_t>(stride));
}

} // namespace rxopt
} // namespace demod
} // namespace uwb
} // namespace gr
