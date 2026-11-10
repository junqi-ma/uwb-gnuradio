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

#include <gnuradio/uwb/uwb_detector_core.h> // gr::uwb::core::uwb_l2_normalize
#include <gnuradio/uwb/uwb_phy_profile.h>   // kQm35SamplesPerSymbol

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

namespace gr {
namespace uwb {
namespace demod {
namespace rxopt {

// ---------------------------------------------------------------------------
// Long complex conjugate dot product, identical math to
// demod::core::detail::timing_conjugate_dot:
//
//   returns sum_{k < count} input[k] * conj(taps[k])
//
// VOLK runtime-dispatches to the fastest kernel on the host.  The SFD search
// evaluates 8128-point dots, so dispatch overhead is amortized.  Reduction
// order differs from the scalar loop, so the fast path is only enabled
// explicitly and its output is verified against the legacy search.
// ---------------------------------------------------------------------------
inline std::complex<float> conjugate_dot(const std::complex<float>* input,
                                         const std::complex<float>* taps,
                                         size_t count)
{
    std::complex<float> acc(0.0f, 0.0f);
    if (count == 0)
        return acc;
    volk_32fc_x2_conjugate_dot_prod_32fc(
        &acc,
        reinterpret_cast<const lv_32fc_t*>(input),
        reinterpret_cast<const lv_32fc_t*>(taps),
        static_cast<unsigned int>(count));
    return acc;
}

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

#if (defined(__x86_64__) || defined(_M_X64)) && (defined(__GNUC__) || defined(__clang__))
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
#if (defined(__x86_64__) || defined(_M_X64)) && (defined(__GNUC__) || defined(__clang__))
    if (has_avx2_fma()) {
        real_template_dot_4lag_avx2(rx, taps_dup, L, out);
        return;
    }
#endif
    out[0] = out[1] = out[2] = out[3] = std::complex<float>(0.f, 0.f);
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
#if (defined(__x86_64__) || defined(_M_X64)) && (defined(__GNUC__) || defined(__clang__))
    if (has_avx2_fma()) {
        return real_template_dot_single_avx2(rx, taps_dup, L);
    }
#endif
    std::complex<float> acc(0.f, 0.f);
    for (size_t k = 0; k < L; ++k) {
        acc += rx[k] * taps_dup[2 * k];
    }
    return acc;
}

#if (defined(__x86_64__) || defined(_M_X64)) && (defined(__GNUC__) || defined(__clang__))
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
#if (defined(__x86_64__) || defined(_M_X64)) && (defined(__GNUC__) || defined(__clang__))
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

#if (defined(__x86_64__) || defined(_M_X64)) && (defined(__GNUC__) || defined(__clang__))
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

inline void vector_accumulate_complex(std::complex<float>* dst,
                                      const std::complex<float>* src,
                                      size_t count)
{
#if (defined(__x86_64__) || defined(_M_X64)) && (defined(__GNUC__) || defined(__clang__))
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
#if (defined(__x86_64__) || defined(_M_X64)) && (defined(__GNUC__) || defined(__clang__))
    if (has_avx2_fma()) {
        vector_sub_complex_avx2(dst, src, count);
        return;
    }
#endif
    for (size_t m = 0; m < count; ++m)
        dst[m] -= src[m];
}

inline void fold_sfd_window(const std::complex<float>* rx_base,
                            size_t sym,
                            const int8_t* sfd_seq,
                            size_t sfd_len,
                            size_t fold_len,
                            std::complex<float>* out)
{
    if (!rx_base || !sfd_seq || !out || sym == 0 || sfd_len == 0 || fold_len == 0)
        return;
    size_t i = 0;
    while (i < sfd_len && sfd_seq[i] == 0)
        ++i;
    if (i == sfd_len) {
        std::fill(out, out + fold_len, std::complex<float>(0.0f, 0.0f));
        return;
    }
    const int8_t first_s = sfd_seq[i];
    if (first_s == 1) {
        std::memcpy(out, rx_base + i * sym, fold_len * sizeof(std::complex<float>));
    } else {
        std::fill(out, out + fold_len, std::complex<float>(0.0f, 0.0f));
        vector_sub_complex(out, rx_base + i * sym, fold_len);
    }
    ++i;
    for (; i < sfd_len; ++i) {
        const int8_t s = sfd_seq[i];
        if (s == 1) {
            vector_accumulate_complex(out, rx_base + i * sym, fold_len);
        } else if (s == -1) {
            vector_sub_complex(out, rx_base + i * sym, fold_len);
        }
    }
}

inline void vector_scale_complex(std::complex<float>* dst,
                                 float scale,
                                 size_t count)
{
#if (defined(__x86_64__) || defined(_M_X64)) && (defined(__GNUC__) || defined(__clang__))
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
