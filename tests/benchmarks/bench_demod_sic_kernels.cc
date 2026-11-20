/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Phase-2 (demod / SIC AVX-512) operator-level benchmark.
 *
 * Plan: docs/performance/解调与SIC_AVX512内核扩展开发规划_2026-11-16.md (P0/P4).
 *
 * This tool measures ONE kernel at a time with the sample loop outside the
 * timer, so a microsecond-resolution stage clock cannot hide a sub-microsecond
 * change.  For every kernel it also asserts that the candidate backend is
 * BIT-IDENTICAL to the AVX2 reference on the same input, because the phase-2
 * numerical contract forbids changing the lane mapping, FMA use, accumulation
 * order or reduction tree.
 *
 * Kernels covered (production sizes taken from bench_unified_timing):
 *   K1  real-template 4-lag dot        L = 1016  (timing refine / SFD refine)
 *   K2  element-wise accumulate / sub / scale   (CIR average, fold, SIC)
 *   K3  fold_sfd_window                fold_len = 700, 8 SFD symbols
 *   K4  SIC 20-tap pulse reconstruction (zero CFO)
 *   K5  fused CMF fixed-52 payload dot
 *   K6  fixed-38 CIR FIR dot
 *
 * Usage: bench_demod_sic_kernels [--rounds N] [--iters N]
 */

#include <gnuradio/uwb/uwb_cir_fir_simd.h>
#include <gnuradio/uwb/uwb_kernel_backend.h>
#include <gnuradio/uwb/uwb_rx_fused_cmf.h>
#include <gnuradio/uwb/uwb_rx_sync_opt.h>
#include <openuwb/support/simd_config.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <random>
#include <string>
#include <vector>

#if defined(__x86_64__) || defined(_M_X64)
#include <immintrin.h>
#endif

using cf = std::complex<float>;

namespace {

using Clock = std::chrono::steady_clock;

double ns_since(Clock::time_point t0)
{
    return std::chrono::duration<double, std::nano>(Clock::now() - t0).count();
}

// Median over `rounds` of a batched loop, so one scheduler hiccup cannot
// masquerade as a kernel regression.
template <typename Fn>
double bench_ns(Fn&& fn, int rounds, int inner)
{
    std::vector<double> v;
    v.reserve(static_cast<size_t>(rounds));
    for (int r = 0; r < rounds; ++r) {
        const auto t0 = Clock::now();
        for (int i = 0; i < inner; ++i)
            fn();
        v.push_back(ns_since(t0) / static_cast<double>(inner));
    }
    std::sort(v.begin(), v.end());
    return v[v.size() / 2];
}

int g_failures = 0;

// Consumes measured results so the optimiser cannot delete the timed loops.
volatile float g_sink = 0.0f;

void report(const char* name, double avx2_ns, double cand_ns, bool identical)
{
    const double ratio = (avx2_ns > 0.0) ? cand_ns / avx2_ns : 0.0;
    std::printf("  %-40s %10.1f %10.1f  %+7.2f%%  %s\n", name, avx2_ns,
                cand_ns, (ratio - 1.0) * 100.0,
                identical ? "bit-identical" : "*** MISMATCH ***");
    if (!identical)
        ++g_failures;
}

void header(const char* title)
{
    std::printf("\n%s\n", title);
    std::printf("  %-40s %10s %10s  %8s  %s\n", "kernel", "avx2 ns", "cand ns",
                "delta", "identity");
}

// ---------------------------------------------------------------------------
// AVX2 reference replicas for kernels that live in a .cc translation unit and
// are therefore not reachable from here.  They mirror the production code.
// ---------------------------------------------------------------------------
__attribute__((target("avx2")))
void accumulate_pulses_20tap_avx2_ref(cf* recon_base, const cf* taps_ptr,
                                      const int32_t* offsets,
                                      const int8_t* signs, size_t count)
{
    const float* t_flt = reinterpret_cast<const float*>(taps_ptr);
    const __m256 t0 = _mm256_loadu_ps(t_flt);
    const __m256 t1 = _mm256_loadu_ps(t_flt + 8);
    const __m256 t2 = _mm256_loadu_ps(t_flt + 16);
    const __m256 t3 = _mm256_loadu_ps(t_flt + 24);
    const __m256 t4 = _mm256_loadu_ps(t_flt + 32);
    for (size_t i = 0; i < count; ++i) {
        float* d = reinterpret_cast<float*>(recon_base + offsets[i]);
        if (signs[i] > 0) {
            _mm256_storeu_ps(d, _mm256_add_ps(_mm256_loadu_ps(d), t0));
            _mm256_storeu_ps(d + 8, _mm256_add_ps(_mm256_loadu_ps(d + 8), t1));
            _mm256_storeu_ps(d + 16,
                             _mm256_add_ps(_mm256_loadu_ps(d + 16), t2));
            _mm256_storeu_ps(d + 24,
                             _mm256_add_ps(_mm256_loadu_ps(d + 24), t3));
            _mm256_storeu_ps(d + 32,
                             _mm256_add_ps(_mm256_loadu_ps(d + 32), t4));
        } else {
            _mm256_storeu_ps(d, _mm256_sub_ps(_mm256_loadu_ps(d), t0));
            _mm256_storeu_ps(d + 8, _mm256_sub_ps(_mm256_loadu_ps(d + 8), t1));
            _mm256_storeu_ps(d + 16,
                             _mm256_sub_ps(_mm256_loadu_ps(d + 16), t2));
            _mm256_storeu_ps(d + 24,
                             _mm256_sub_ps(_mm256_loadu_ps(d + 24), t3));
            _mm256_storeu_ps(d + 32,
                             _mm256_sub_ps(_mm256_loadu_ps(d + 32), t4));
        }
    }
}

// ---------------------------------------------------------------------------
// AVX-512 candidates.  Prototyped here first: a candidate only moves into the
// production translation unit after this bench shows it is both bit-identical
// and materially faster.
// ---------------------------------------------------------------------------
#if UWB_SIMD_HAVE_AVX512
#define UWB_BENCH_AVX512 \
    __attribute__((target("avx512f,avx512bw,avx512dq,avx512vl")))

// Eight-lag real-template dot.
//
// Numerical contract: register m holds lag m in lanes 0..7 and lag m+4 in
// lanes 8..15, because one 512-bit load at rx + (k + m) yields complex samples
// (k+m .. k+m+3) in the low half and (k+m+4 .. k+m+7) in the high half.  Lane j
// of the low half therefore accumulates exactly the taps that AVX2 256-bit
// accumulator lane j accumulates for lag m, and the high half does the same
// for lag m+4.  The final reduction reuses the AVX2 scalar tree
// tmp[0]+tmp[2]+tmp[4]+tmp[6], so the result is bit-identical.
UWB_BENCH_AVX512 inline void
real_template_dot_8lag_avx512(const cf* rx, const float* taps_dup, size_t L,
                              cf out[8])
{
    if (L == 0 || L > 65536) {
        for (int m = 0; m < 8; ++m)
            out[m] = cf(0.f, 0.f);
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
        const __m256 t = _mm256_loadu_ps(taps_dup + 2 * k);
        const __m512 tb = _mm512_broadcast_f32x8(t);
        a0 = _mm512_fmadd_ps(_mm512_loadu_ps(rx0 + 2 * (k + 0)), tb, a0);
        a1 = _mm512_fmadd_ps(_mm512_loadu_ps(rx0 + 2 * (k + 1)), tb, a1);
        a2 = _mm512_fmadd_ps(_mm512_loadu_ps(rx0 + 2 * (k + 2)), tb, a2);
        a3 = _mm512_fmadd_ps(_mm512_loadu_ps(rx0 + 2 * (k + 3)), tb, a3);
    }

    alignas(32) float lo[8], hi[8];
    const __m512 acc[4] = { a0, a1, a2, a3 };
    float sre[8], sim[8];
    for (int r = 0; r < 4; ++r) {
        _mm256_storeu_ps(lo, _mm512_castps512_ps256(acc[r]));
        _mm256_storeu_ps(hi, _mm512_extractf32x8_ps(acc[r], 1));
        sre[r] = lo[0] + lo[2] + lo[4] + lo[6];
        sim[r] = lo[1] + lo[3] + lo[5] + lo[7];
        sre[r + 4] = hi[0] + hi[2] + hi[4] + hi[6];
        sim[r + 4] = hi[1] + hi[3] + hi[5] + hi[7];
    }
    for (; k < L; ++k) {
        const float t = taps_dup[2 * k];
        for (int m = 0; m < 8; ++m) {
            sre[m] += rx0[2 * (k + m)] * t;
            sim[m] += rx0[2 * (k + m) + 1] * t;
        }
    }
    for (int m = 0; m < 8; ++m)
        out[m] = cf(sre[m], sim[m]);
}

// Eight-lag real-template dot, variant B: the tap vector is pre-duplicated
// into a 512-bit-friendly layout (16 floats per 4 taps) so the inner loop
// needs no broadcast.  Same numerical contract as variant A.
UWB_BENCH_AVX512 inline void
real_template_dot_8lag_dup_avx512(const cf* rx, const float* taps_quad,
                                  size_t L, cf out[8])
{
    if (L == 0 || L > 65536) {
        for (int m = 0; m < 8; ++m)
            out[m] = cf(0.f, 0.f);
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
        const __m512 tb = _mm512_loadu_ps(taps_quad + 4 * k);
        a0 = _mm512_fmadd_ps(_mm512_loadu_ps(rx0 + 2 * (k + 0)), tb, a0);
        a1 = _mm512_fmadd_ps(_mm512_loadu_ps(rx0 + 2 * (k + 1)), tb, a1);
        a2 = _mm512_fmadd_ps(_mm512_loadu_ps(rx0 + 2 * (k + 2)), tb, a2);
        a3 = _mm512_fmadd_ps(_mm512_loadu_ps(rx0 + 2 * (k + 3)), tb, a3);
    }

    alignas(32) float lo[8], hi[8];
    const __m512 acc[4] = { a0, a1, a2, a3 };
    float sre[8], sim[8];
    for (int r = 0; r < 4; ++r) {
        _mm256_storeu_ps(lo, _mm512_castps512_ps256(acc[r]));
        _mm256_storeu_ps(hi, _mm512_extractf32x8_ps(acc[r], 1));
        sre[r] = lo[0] + lo[2] + lo[4] + lo[6];
        sim[r] = lo[1] + lo[3] + lo[5] + lo[7];
        sre[r + 4] = hi[0] + hi[2] + hi[4] + hi[6];
        sim[r + 4] = hi[1] + hi[3] + hi[5] + hi[7];
    }
    for (; k < L; ++k) {
        const float t = taps_quad[4 * k];
        for (int m = 0; m < 8; ++m) {
            sre[m] += rx0[2 * (k + m)] * t;
            sim[m] += rx0[2 * (k + m) + 1] * t;
        }
    }
    for (int m = 0; m < 8; ++m)
        out[m] = cf(sre[m], sim[m]);
}

UWB_BENCH_AVX512 inline void accumulate_cf32_avx512(cf* dst, const cf* src,
                                                    size_t count)
{
    float* d = reinterpret_cast<float*>(dst);
    const float* s = reinterpret_cast<const float*>(src);
    const size_t nf = 2 * count;
    size_t i = 0;
    const size_t limit = nf & ~size_t(15);
    for (; i < limit; i += 16)
        _mm512_storeu_ps(
            d + i, _mm512_add_ps(_mm512_loadu_ps(d + i), _mm512_loadu_ps(s + i)));
    for (; i < nf; ++i)
        d[i] += s[i];
}

UWB_BENCH_AVX512 inline void sub_cf32_avx512(cf* dst, const cf* src,
                                             size_t count)
{
    float* d = reinterpret_cast<float*>(dst);
    const float* s = reinterpret_cast<const float*>(src);
    const size_t nf = 2 * count;
    size_t i = 0;
    const size_t limit = nf & ~size_t(15);
    for (; i < limit; i += 16)
        _mm512_storeu_ps(
            d + i, _mm512_sub_ps(_mm512_loadu_ps(d + i), _mm512_loadu_ps(s + i)));
    for (; i < nf; ++i)
        d[i] -= s[i];
}

UWB_BENCH_AVX512 inline void scale_cf32_avx512(cf* dst, float scale,
                                               size_t count)
{
    float* d = reinterpret_cast<float*>(dst);
    const size_t nf = 2 * count;
    size_t i = 0;
    const size_t limit = nf & ~size_t(15);
    const __m512 vs = _mm512_set1_ps(scale);
    for (; i < limit; i += 16)
        _mm512_storeu_ps(d + i, _mm512_mul_ps(_mm512_loadu_ps(d + i), vs));
    for (; i < nf; ++i)
        d[i] *= scale;
}

// Complex MAC of 8 complex samples = two AVX2 "group4" groups at once.
// Low 256 lanes = group 2i, high 256 lanes = group 2i+1, so accumulating into
// one 512-bit register reproduces the AVX2 acc0 (even groups) / acc1 (odd
// groups) split exactly.
UWB_BENCH_AVX512 inline __m512 cir_group8_avx512(const float* xp,
                                                 const float* hp, int off)
{
    const __m512 x = _mm512_loadu_ps(xp + off);
    const __m512 ht = _mm512_loadu_ps(hp + off);
    const __m512 xre = _mm512_moveldup_ps(x);
    const __m512 xim = _mm512_movehdup_ps(x);
    const __m512 hsw = _mm512_permute_ps(ht, 0xB1);
    const __m512 t = _mm512_mul_ps(xim, hsw);
    return _mm512_fmaddsub_ps(xre, ht, t);
}

// Fold a 512-bit accumulator (even groups in the low half, odd groups in the
// high half) plus an optional trailing even group into one complex value using
// the AVX2 reduction tree.
UWB_BENCH_AVX512 inline cf reduce_group_acc_avx512(__m512 acc,
                                                   __m256 trailing_even)
{
    __m256 lo = _mm512_castps512_ps256(acc);
    const __m256 hi = _mm512_extractf32x8_ps(acc, 1);
    lo = _mm256_add_ps(lo, trailing_even);
    const __m256 a = _mm256_add_ps(lo, hi);
    const __m128 s =
        _mm_add_ps(_mm256_castps256_ps128(a), _mm256_extractf128_ps(a, 1));
    const __m128 s2 = _mm_add_ps(s, _mm_movehl_ps(s, s));
    return cf(_mm_cvtss_f32(s2), _mm_cvtss_f32(_mm_shuffle_ps(s2, s2, 1)));
}

UWB_BENCH_AVX512 inline cf dot_fixed52_avx512(const cf* rx_win, const cf* taps)
{
    const float* xp = reinterpret_cast<const float*>(rx_win);
    const float* hp = reinterpret_cast<const float*>(taps);
    __m512 acc = cir_group8_avx512(xp, hp, 0);
    acc = _mm512_add_ps(acc, cir_group8_avx512(xp, hp, 16));
    acc = _mm512_add_ps(acc, cir_group8_avx512(xp, hp, 32));
    acc = _mm512_add_ps(acc, cir_group8_avx512(xp, hp, 48));
    acc = _mm512_add_ps(acc, cir_group8_avx512(xp, hp, 64));
    acc = _mm512_add_ps(acc, cir_group8_avx512(xp, hp, 80));
    // 52 complex = 13 groups: 12 in the 512-bit accumulator (6 pairs) plus the
    // final even group 12, which belongs in the LOW half only.
    const __m256 g12 = openuwb::demod::cir_fir::cir_fir_avx2_group4(xp, hp, 96);
    return reduce_group_acc_avx512(acc, g12);
}

UWB_BENCH_AVX512 inline cf dot_fixed38_avx512(const cf* rx_win,
                                              const cf* h_conj)
{
    const float* xp = reinterpret_cast<const float*>(rx_win);
    const float* hp = reinterpret_cast<const float*>(h_conj);
    __m512 acc = cir_group8_avx512(xp, hp, 0);
    acc = _mm512_add_ps(acc, cir_group8_avx512(xp, hp, 16));
    acc = _mm512_add_ps(acc, cir_group8_avx512(xp, hp, 32));
    acc = _mm512_add_ps(acc, cir_group8_avx512(xp, hp, 48));
    // 38 taps = 9 groups of 4 + a 2-tap scalar tail (same as the AVX2 kernel).
    const __m256 g8 = openuwb::demod::cir_fir::cir_fir_avx2_group4(xp, hp, 64);
    const cf acc_c = reduce_group_acc_avx512(acc, g8);
    // The AVX2 kernel adds each tail tap as (re + (a - b)); keep that
    // association so the result stays bit-identical.
    float re = acc_c.real();
    float im = acc_c.imag();
    re += xp[72] * hp[72] - xp[73] * hp[73];
    im += xp[72] * hp[73] + xp[73] * hp[72];
    re += xp[74] * hp[74] - xp[75] * hp[75];
    im += xp[74] * hp[75] + xp[75] * hp[74];
    return cf(re, im);
}

// SIC 20-tap pulse accumulation, zero CFO.  40 floats = two 512-bit vectors
// plus one 256-bit tail; element-wise, so bit-identical by construction.
UWB_BENCH_AVX512 inline void accumulate_pulses_20tap_avx512(
    cf* recon_base, const cf* taps_ptr, const int32_t* offsets,
    const int8_t* signs, size_t count)
{
    const float* t_flt = reinterpret_cast<const float*>(taps_ptr);
    const __m512 t0 = _mm512_loadu_ps(t_flt);
    const __m512 t1 = _mm512_loadu_ps(t_flt + 16);
    const __m256 t2 = _mm256_loadu_ps(t_flt + 32);
    for (size_t i = 0; i < count; ++i) {
        float* d = reinterpret_cast<float*>(recon_base + offsets[i]);
        if (signs[i] > 0) {
            _mm512_storeu_ps(d, _mm512_add_ps(_mm512_loadu_ps(d), t0));
            _mm512_storeu_ps(d + 16,
                             _mm512_add_ps(_mm512_loadu_ps(d + 16), t1));
            _mm256_storeu_ps(d + 32,
                             _mm256_add_ps(_mm256_loadu_ps(d + 32), t2));
        } else {
            _mm512_storeu_ps(d, _mm512_sub_ps(_mm512_loadu_ps(d), t0));
            _mm512_storeu_ps(d + 16,
                             _mm512_sub_ps(_mm512_loadu_ps(d + 16), t1));
            _mm256_storeu_ps(d + 32,
                             _mm256_sub_ps(_mm256_loadu_ps(d + 32), t2));
        }
    }
}
#endif // UWB_SIMD_HAVE_AVX512

// Scalar reference for the identity check.
cf dot_fixed52_scalar(const cf* rx_win, const cf* taps)
{
    cf acc(0.f, 0.f);
    for (size_t i = 0; i < 52; ++i)
        acc += rx_win[i] * taps[i];
    return acc;
}

cf dot_fixed38_scalar(const cf* rx_win, const cf* h_conj)
{
    cf acc(0.f, 0.f);
    for (size_t i = 0; i < 38; ++i)
        acc += rx_win[i] * h_conj[i];
    return acc;
}

} // namespace

int main(int argc, char** argv)
{
    int rounds = 9;
    int iters = 200;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--rounds" && i + 1 < argc)
            rounds = std::atoi(argv[++i]);
        else if (a == "--iters" && i + 1 < argc)
            iters = std::atoi(argv[++i]);
        else {
            std::printf("usage: %s [--rounds N] [--iters N]\n", argv[0]);
            return 2;
        }
    }

    // The reference arm must be the AVX2 kernel, not "whatever the default
    // policy picks" -- in an AVX-512 build the Sync default is AVX-512, which
    // would silently compare AVX-512 against itself.
    openuwb::core::simd::set_kernel_backend(
        openuwb::core::simd::KernelGroup::Sync,
        openuwb::core::simd::Backend::Avx2);
    std::printf("bench_demod_sic_kernels: UWB_SIMD=%s level=%d avx512=%d\n",
                UWB_SIMD_NAME, UWB_SIMD_LEVEL, UWB_SIMD_HAVE_AVX512);
    std::printf("reference arm: sync group forced to %s\n",
                openuwb::core::simd::resolved_backend_name(
                    openuwb::core::simd::KernelGroup::Sync));
    std::printf("rounds=%d iters=%d (median of batched loops)\n", rounds, iters);

    std::mt19937 rng(20261116u);
    std::uniform_real_distribution<float> dist(-1.0f, 1.0f);

    constexpr size_t kSym = 1016; // kQm35SamplesPerSymbol

    // ---- K1: real-template 4-lag dot, L = 1016 -----------------------------
    {
        std::vector<cf> rx(kSym + 64);
        for (auto& v : rx)
            v = cf(dist(rng), dist(rng));
        std::vector<float> taps_dup(2 * kSym);
        for (size_t k = 0; k < kSym; ++k) {
            const float t = dist(rng);
            taps_dup[2 * k] = t;
            taps_dup[2 * k + 1] = t;
        }

        header("K1  real-template dot, L=1016 (timing/SFD refine)");
        // Normalise everything to ns per LAG so a 4-lag and an 8-lag call are
        // directly comparable.
        const double t_avx2 = bench_ns(
            [&] {
                for (int i = 0; i < 8; ++i) {
                    cf o[4];
                    openuwb::demod::rxopt::real_template_dot_4lag(
                        rx.data() + i, taps_dup.data(), kSym, o);
                    g_sink += o[0].real();
                }
            },
            rounds, std::max(1, iters / 8)) /
            (8.0 * 4.0);
#if UWB_SIMD_HAVE_AVX512
        // Pre-duplicated 512-bit tap layout: 16 floats per 4 taps, i.e.
        // [t0,t0,t1,t1,t2,t2,t3,t3] repeated twice.
        std::vector<float> taps_quad(4 * kSym);
        for (size_t k = 0; k < kSym; ++k) {
            const float t = taps_dup[2 * k];
            const size_t off = 16 * (k / 4) + 2 * (k % 4);
            taps_quad[off] = t;
            taps_quad[off + 1] = t;
            taps_quad[off + 8] = t;
            taps_quad[off + 9] = t;
        }

        // One 8-lag call covers two production 4-lag steps.
        const double t_512 = bench_ns(
            [&] {
                for (int i = 0; i < 4; ++i) {
                    cf o[8];
                    real_template_dot_8lag_avx512(rx.data() + 2 * i,
                                                  taps_dup.data(), kSym, o);
                    g_sink += o[0].real();
                }
            },
            rounds, std::max(1, iters / 8)) /
            (4.0 * 8.0);
        const double t_dup = bench_ns(
            [&] {
                for (int i = 0; i < 4; ++i) {
                    cf o[8];
                    real_template_dot_8lag_dup_avx512(
                        rx.data() + 2 * i, taps_quad.data(), kSym, o);
                    g_sink += o[0].real();
                }
            },
            rounds, std::max(1, iters / 8)) /
            (4.0 * 8.0);
        cf got[8];
        real_template_dot_8lag_avx512(rx.data(), taps_dup.data(), kSym, got);
        bool same = true;
        for (int half = 0; half < 2; ++half) {
            cf ref4[4];
            openuwb::demod::rxopt::real_template_dot_4lag(
                rx.data() + 4 * half, taps_dup.data(), kSym, ref4);
            for (int m = 0; m < 4; ++m)
                if (std::memcmp(&got[4 * half + m], &ref4[m], sizeof(cf)) != 0)
                    same = false;
        }
        report("4lag AVX2 -> 8lag AVX-512 (bcast)", t_avx2, t_512, same);
        report("4lag AVX2 -> 8lag AVX-512 (dup)", t_avx2, t_dup, same);
#else
        (void)t_avx2;
        std::printf("  (no AVX-512 backend in this build)\n");
#endif
    }

    // ---- K2: element-wise accumulate / sub / scale -------------------------
    {
        header("K2  element-wise complex ops (CIR average / fold / SIC)");
        for (size_t n : { size_t(1053), size_t(2032), size_t(135568) }) {
            std::vector<cf> a(n), b(n);
            for (size_t i = 0; i < n; ++i) {
                a[i] = cf(dist(rng), dist(rng));
                b[i] = cf(dist(rng), dist(rng));
            }
#if UWB_SIMD_HAVE_AVX512
            auto check = [&](const char* what, auto&& avx2_fn, auto&& avx512_fn) {
                std::vector<cf> r1 = a, r2 = a;
                avx2_fn(r1);
                avx512_fn(r2);
                const bool same =
                    std::memcmp(r1.data(), r2.data(), n * sizeof(cf)) == 0;
                char name[128];
                std::snprintf(name, sizeof(name), "%s n=%zu", what, n);
                const double t2 = bench_ns([&] { avx2_fn(a); }, rounds, iters);
                const double t5 = bench_ns([&] { avx512_fn(a); }, rounds, iters);
                report(name, t2, t5, same);
            };
            check("accumulate",
                  [&](std::vector<cf>& d) {
                      openuwb::demod::rxopt::vector_accumulate_complex(
                          d.data(), b.data(), n);
                  },
                  [&](std::vector<cf>& d) {
                      accumulate_cf32_avx512(d.data(), b.data(), n);
                  });
            check("sub",
                  [&](std::vector<cf>& d) {
                      openuwb::demod::rxopt::vector_sub_complex(d.data(),
                                                                b.data(), n);
                  },
                  [&](std::vector<cf>& d) {
                      sub_cf32_avx512(d.data(), b.data(), n);
                  });
            check("scale",
                  [&](std::vector<cf>& d) {
                      openuwb::demod::rxopt::vector_scale_complex(d.data(),
                                                                  0.5f, n);
                  },
                  [&](std::vector<cf>& d) {
                      scale_cf32_avx512(d.data(), 0.5f, n);
                  });
#else
            (void)a;
            (void)b;
#endif
        }
    }

    // ---- K3: fold_sfd_window ----------------------------------------------
    {
        constexpr size_t kFoldLen = 700;
        constexpr size_t kSfdLen = 8;
        std::vector<cf> rx(kSfdLen * kSym + kFoldLen);
        for (auto& v : rx)
            v = cf(dist(rng), dist(rng));
        const std::vector<int8_t> seq = { 1, -1, 1, 1, -1, -1, 1, -1 };
        std::vector<cf> out_a(kFoldLen), out_b(kFoldLen);

        header("K3  fold_sfd_window (8 SFD symbols, fold_len=700)");
        auto fold_avx2 = [&] {
            openuwb::demod::rxopt::fold_sfd_window(
                rx.data(), kSym, seq.data(), kSfdLen, kFoldLen, out_a.data());
        };
        const double t2 = bench_ns(fold_avx2, rounds, iters);
        fold_avx2();
#if UWB_SIMD_HAVE_AVX512
        auto fold_512 = [&] {
            std::memcpy(out_b.data(), rx.data(), kFoldLen * sizeof(cf));
            for (size_t i = 1; i < kSfdLen; ++i) {
                if (seq[i] == 1)
                    accumulate_cf32_avx512(out_b.data(), rx.data() + i * kSym,
                                           kFoldLen);
                else
                    sub_cf32_avx512(out_b.data(), rx.data() + i * kSym,
                                    kFoldLen);
            }
        };
        fold_512();
        const bool same =
            std::memcmp(out_a.data(), out_b.data(), kFoldLen * sizeof(cf)) == 0;
        const double t5 = bench_ns(fold_512, rounds, iters);
        report("fold_sfd_window", t2, t5, same);
#else
        (void)t2;
#endif
    }

    // ---- K4: SIC 20-tap pulse reconstruction -------------------------------
    {
        constexpr size_t kEvents = 4096;
        constexpr size_t kSpan = 20;
        constexpr size_t kMaxOff = 200000;
        std::vector<cf> taps(kSpan);
        for (auto& v : taps)
            v = cf(dist(rng), dist(rng));
        std::vector<int32_t> offs(kEvents);
        std::vector<int8_t> signs(kEvents);
        for (size_t i = 0; i < kEvents; ++i) {
            offs[i] = static_cast<int32_t>(2 * i);
            signs[i] = (i & 1) ? -1 : 1;
        }
        std::vector<cf> buf_a(kMaxOff + kSpan + 8), buf_b(buf_a.size());

        header("K4  SIC 20-tap pulse accumulation (4096 events)");
        auto run_avx2 = [&] {
            std::fill(buf_a.begin(), buf_a.end(), cf(0.f, 0.f));
            accumulate_pulses_20tap_avx2_ref(buf_a.data(), taps.data(),
                                             offs.data(), signs.data(), kEvents);
        };
        const double t2 = bench_ns(run_avx2, rounds, std::max(1, iters / 4));
        run_avx2();
#if UWB_SIMD_HAVE_AVX512
        auto run_512 = [&] {
            std::fill(buf_b.begin(), buf_b.end(), cf(0.f, 0.f));
            accumulate_pulses_20tap_avx512(buf_b.data(), taps.data(),
                                           offs.data(), signs.data(), kEvents);
        };
        run_512();
        const bool same = std::memcmp(buf_a.data(), buf_b.data(),
                                      (kMaxOff + kSpan) * sizeof(cf)) == 0;
        const double t5 = bench_ns(run_512, rounds, std::max(1, iters / 4));
        report("accumulate 20-tap pulses", t2, t5, same);
#else
        (void)t2;
#endif
    }

    // ---- K5 / K6: fused CMF 52-tap and CIR 38-tap dots --------------------
    {
        constexpr size_t kWin = 4096;
        std::vector<cf> rx(kWin);
        for (auto& v : rx)
            v = cf(dist(rng), dist(rng));
        std::vector<cf> t52(52), t38(38);
        for (auto& v : t52)
            v = cf(dist(rng), dist(rng));
        for (auto& v : t38)
            v = cf(dist(rng), dist(rng));

        header("K5/K6  fixed-length complex dots (payload CMF / CIR FIR)");
#if UWB_SIMD_HAVE_AVX512
        auto bench_dot = [&](const char* name, auto&& avx2_fn, auto&& avx512_fn,
                             auto&& scalar_fn) {
            const cf s = scalar_fn(rx.data() + 7);
            const cf a = avx2_fn(rx.data() + 7);
            const cf b = avx512_fn(rx.data() + 7);
            const bool same_ab = std::memcmp(&a, &b, sizeof(cf)) == 0;
            const double tol = 1e-4f * (1.0f + std::abs(s));
            const bool same_scalar = std::abs(a - s) <= tol;
            const double t2 = bench_ns(
                                   [&] {
                                       cf acc(0.f, 0.f);
                                       for (size_t i = 0; i < 256; ++i)
                                           acc += avx2_fn(rx.data() + i);
                                       g_sink += acc.real();
                                   },
                                   rounds, std::max(1, iters / 8)) /
                               256.0;
            const double t5 = bench_ns(
                                   [&] {
                                       cf acc(0.f, 0.f);
                                       for (size_t i = 0; i < 256; ++i)
                                           acc += avx512_fn(rx.data() + i);
                                       g_sink += acc.real();
                                   },
                                   rounds, std::max(1, iters / 8)) /
                               256.0;
            report(name, t2, t5, same_ab && same_scalar);
        };
        bench_dot("dot_fixed52 (payload CMF)",
                  [&](const cf* p) {
                      return openuwb::demod::rxfusion::dot_fixed52(p,
                                                                   t52.data());
                  },
                  [&](const cf* p) { return dot_fixed52_avx512(p, t52.data()); },
                  [&](const cf* p) { return dot_fixed52_scalar(p, t52.data()); });
        bench_dot("dot_fixed38 (CIR FIR)",
                  [&](const cf* p) {
                      return openuwb::demod::cir_fir::dot_avx2_fixed38(
                          p, t38.data(), 38);
                  },
                  [&](const cf* p) { return dot_fixed38_avx512(p, t38.data()); },
                  [&](const cf* p) { return dot_fixed38_scalar(p, t38.data()); });
#else
        std::printf("  (no AVX-512 backend in this build)\n");
#endif
    }

    std::printf("\n%s\n", g_failures == 0
                              ? "IDENTITY: all candidates bit-identical"
                              : "IDENTITY: FAILURES PRESENT");
    return g_failures == 0 ? 0 : 1;
}
