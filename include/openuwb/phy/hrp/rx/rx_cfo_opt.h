/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Experimental communication-RX CFO derotation kernel.
 *
 * `stage_cfo` in uwb_demod_core.h used to derotate the entire RX window with
 * four scalar float rotation chains plus one exact sin/cos anchor per
 * 1024-sample block.  std::complex<float> multiply is not vectorized by GCC,
 * so the two full-window rotations (initial fit + SFD-anchored refit) dominate
 * the CFO stage.
 *
 * `derotate_block` keeps that block decomposition and sign convention but
 * replaces the per-sample scalar work with one fused pass per block:
 *
 *     dst[b0 + k] = src[b0 + k] * (table[k] * e^{-j w b0})
 *
 * The block anchor e^{-j w b0} is folded into the table entry **in registers**,
 * so the data array is read once and written once.  The earlier two-pass form
 * (`dst = src * table`, then `dst *= anchor`) touched dst three times per
 * sample; that round trip stayed in L1 because the block is only 8 KiB, but it
 * still cost a second full-width multiply plus a load and a store per sample.
 * Folding the anchor into the table instead of into the data removes that pass
 * without changing the phase origin, the block decomposition or the accuracy
 * budget (see the QA error contract in qa_uwb_cfo_derotate.cc).
 *
 * The 1024-entry table is built once per call (w changes with the frame) from
 * four independent float chains advanced by step^4 = e^{-j 4 w}; each chain is
 * re-anchored from the exact double-precision phase every
 * kCfoTableReanchorHops steps.  The per-block anchor is evaluated from the
 * exact double absolute phase -w*b0 (one sin/cos pair per 1024 samples), so no
 * anchor drift accumulates and the table is never mutated between blocks.
 *
 * Both complex multiplies are elementwise, so the lane mapping cannot change
 * the result: the AVX2 kernel below and an AVX-512 build (which compiles this
 * header with the same -mavx2 baseline) produce bit-identical output.
 *
 * No heap allocation occurs in the block loop (`dst.resize(n)` runs once per
 * call; the table is stack storage).  `n` does not need to be a multiple of
 * 1024: the final block and the n < 1024 case use the same table prefix.
 *
 * Sign convention (identical to the legacy loop): w is
 * 2*pi*cfo_hz/sample_rate [rad/sample] and
 *
 *     dst[i] = src[i] * exp(-j * w * i).
 */

#pragma once

#include <openuwb/phy/hrp/rx/demod_result.h>
#include <openuwb/phy/hrp/common/profile.h>
#include <openuwb/phy/hrp/rx/rx_matched_filter.h> // legacy sparse CIR API
#include <volk/volk.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

#if defined(__AVX2__) && defined(__FMA__)
#include <immintrin.h>
#define UWB_RX_CFO_HAVE_AVX2 1
#else
#define UWB_RX_CFO_HAVE_AVX2 0
#endif

namespace openuwb {
namespace demod {
namespace rxopt {

// Samples per absolute-phase anchor block (same as the legacy rotation).
constexpr std::size_t kCfoDerotateBlock = 1024;
// Four independent float chains advance by step^4; each chain segment is
// re-anchored from the exact double phase every this many chain steps.
constexpr std::size_t kCfoTableReanchorHops = 16;
constexpr std::size_t kCfoChainCount = 4;
// Double-precision block anchor recurrence re-anchor period.  64 blocks is
// 65536 samples; 64 double multiplications drift by < 1e-14.
constexpr std::size_t kCfoAnchorReanchorBlocks = 64;

namespace detail {

// Build table[k] = exp(-j*w*k) for k in [0, kCfoDerotateBlock).
// Four float chains interleave so the compiler gets 4 independent dependency
// chains (the reason the legacy loop also used 4 chains), but each chain is
// restarted from an exact double sin/cos every kCfoTableReanchorHops steps.
inline void build_cfo_table(std::array<std::complex<float>, kCfoDerotateBlock>& table,
                            double w)
{
    const float step4_r = static_cast<float>(std::cos(-4.0 * w));
    const float step4_i = static_cast<float>(std::sin(-4.0 * w));
    for (std::size_t c = 0; c < kCfoChainCount; ++c) {
        std::size_t k = c;
        while (k < kCfoDerotateBlock) {
            const double phase = -w * static_cast<double>(k);
            float cr = static_cast<float>(std::cos(phase));
            float ci = static_cast<float>(std::sin(phase));
            const std::size_t kend =
                std::min(kCfoDerotateBlock,
                         k + kCfoChainCount * kCfoTableReanchorHops);
            for (; k < kend; k += kCfoChainCount) {
                table[k] = std::complex<float>(cr, ci);
                const float nr = cr * step4_r - ci * step4_i;
                const float ni = cr * step4_i + ci * step4_r;
                cr = nr;
                ci = ni;
            }
        }
    }
}

// One block of the fused derotation:
//
//     dst[k] = src[k] * (table[k] * anchor),  anchor = (ar, ai)
//
// for k in [0, len).  `table` already points at the block's first entry, so the
// caller passes `base_table + k_start` and the absolute phase origin stays
// sample 0.
//
// The anchor is folded into the table entry in registers, so `src` is read once
// and `dst` is written once.  Both multiplies are elementwise (no reduction, no
// cross-lane combine), therefore the AVX2 and the scalar form compute each
// output with the same sequence of float operations and an AVX-512 build -- which
// compiles this header with the same -mavx2 baseline -- is bit-identical.
inline void derotate_block_anchored(std::complex<float>* dst,
                                    const std::complex<float>* src,
                                    const std::complex<float>* table,
                                    std::size_t len,
                                    float ar,
                                    float ai)
{
    std::size_t k = 0;
#if UWB_RX_CFO_HAVE_AVX2
    // Complex multiply, interleaved [re, im, re, im, ...] lanes:
    //   permute(x, 0xB1) swaps re/im, addsub(a, b) = (a_even - b_even,
    //   a_odd + b_odd), which yields re = xr*yr - xi*yi, im = xi*yr + xr*yi.
    const __m256 are = _mm256_set1_ps(ar);
    const __m256 aim = _mm256_set1_ps(ai);
    for (; k + 4 <= len; k += 4) {
        const __m256 t = _mm256_loadu_ps(reinterpret_cast<const float*>(table + k));
        const __m256 ts = _mm256_permute_ps(t, 0xB1);
        const __m256 a =
            _mm256_addsub_ps(_mm256_mul_ps(t, are), _mm256_mul_ps(ts, aim));
        const __m256 s = _mm256_loadu_ps(reinterpret_cast<const float*>(src + k));
        const __m256 ss = _mm256_permute_ps(s, 0xB1);
        const __m256 r =
            _mm256_addsub_ps(_mm256_mul_ps(s, _mm256_moveldup_ps(a)),
                             _mm256_mul_ps(ss, _mm256_movehdup_ps(a)));
        _mm256_storeu_ps(reinterpret_cast<float*>(dst + k), r);
    }
#endif
    for (; k < len; ++k) {
        const std::complex<float> t = table[k];
        const float tr = t.real();
        const float ti = t.imag();
        const float yr = tr * ar - ti * ai;
        const float yi = ti * ar + tr * ai;
        const float sr = src[k].real();
        const float si = src[k].imag();
        dst[k] = std::complex<float>(sr * yr - si * yi, si * yr + sr * yi);
    }
}

} // namespace detail

// dst[i] = src[i] * exp(-j*w*i) for absolute i in [lo, hi), w in rad/sample.
// Phase origin is strictly sample 0 (not lo). dst may not alias src.
inline void derotate_range_block(std::complex<float>* dst,
                                 const std::complex<float>* src,
                                 std::size_t n,
                                 std::size_t lo,
                                 std::size_t hi,
                                 double w)
{
    if (n == 0 || src == nullptr || dst == nullptr)
        return;
    hi = std::min(hi, n);
    if (lo >= hi)
        return;

    std::array<std::complex<float>, kCfoDerotateBlock> table;
    detail::build_cfo_table(table, w);

    const std::size_t b_start = (lo / kCfoDerotateBlock) * kCfoDerotateBlock;
    for (std::size_t b0 = b_start; b0 < hi; b0 += kCfoDerotateBlock) {
        const std::size_t k_start = (b0 < lo) ? (lo - b0) : 0;
        const std::size_t k_end =
            std::min(hi - b0, std::min(kCfoDerotateBlock, n - b0));
        if (k_start >= k_end)
            continue;
        const std::size_t len = k_end - k_start;

        // Exact double block anchor; b0 == 0 gives (1, 0) and the fold below is
        // then bit-exactly the identity, matching the legacy "no second pass
        // for the first block" behaviour.
        const double phase = -w * static_cast<double>(b0);
        const float ar = static_cast<float>(std::cos(phase));
        const float ai = static_cast<float>(std::sin(phase));

        detail::derotate_block_anchored(dst + b0 + k_start,
                                        src + b0 + k_start,
                                        table.data() + k_start,
                                        len,
                                        ar,
                                        ai);
    }
}

// 4-chain recurrence with 1024-sample block re-anchoring.
// Matches the legacy scalar stage_cfo rotation speed and accuracy, generalized
// to operate on an arbitrary absolute range [lo, hi) where phase origin is sample 0.
inline void derotate_range_recurrence(std::complex<float>* dst,
                                      const std::complex<float>* src,
                                      std::size_t n,
                                      std::size_t lo,
                                      std::size_t hi,
                                      double w)
{
    if (n == 0 || src == nullptr || dst == nullptr)
        return;
    hi = std::min(hi, n);
    if (lo >= hi)
        return;

    constexpr std::size_t kB = 1024;
    const double cw = w;
    const double s4_phase = -4.0 * cw;
    const float s4r = static_cast<float>(std::cos(s4_phase));
    const float s4i = static_cast<float>(std::sin(s4_phase));

    for (std::size_t b0 = lo; b0 < hi; b0 += kB) {
        const std::size_t b1 = std::min(hi, b0 + kB);
        const double ph0 = -cw * static_cast<double>(b0);
        float cr[4], ci[4];
        for (int c = 0; c < 4; ++c) {
            const double ph = ph0 - cw * static_cast<double>(c);
            cr[c] = static_cast<float>(std::cos(ph));
            ci[c] = static_cast<float>(std::sin(ph));
        }
        std::size_t i = b0;
        for (; i + 4 <= b1; i += 4) {
            for (int c = 0; c < 4; ++c) {
                const float sr = src[i + c].real();
                const float si = src[i + c].imag();
                dst[i + c] = std::complex<float>(sr * cr[c] - si * ci[c],
                                                 sr * ci[c] + si * cr[c]);
                const float nr = cr[c] * s4r - ci[c] * s4i;
                const float ni = cr[c] * s4i + ci[c] * s4r;
                cr[c] = nr;
                ci[c] = ni;
            }
        }
        for (; i < b1; ++i) {
            const double ph = -cw * static_cast<double>(i);
            const float c = static_cast<float>(std::cos(ph));
            const float s = static_cast<float>(std::sin(ph));
            const float sr = src[i].real();
            const float si = src[i].imag();
            dst[i] = std::complex<float>(sr * c - si * s, sr * s + si * c);
        }
    }
}

// Scalar reference range derotation.  Phase origin is strictly
// sample 0 (i.e. base sample index has phase -w * i, not treating lo as 0).
// Kept for QA and numerical verification; not used on hot demod paths.
inline void derotate_range_scalar(std::complex<float>* dst,
                                  const std::complex<float>* src,
                                  std::size_t n,
                                  std::size_t lo,
                                  std::size_t hi,
                                  double w)
{
    if (n == 0 || src == nullptr || dst == nullptr)
        return;
    hi = std::min(hi, n);
    if (lo >= hi)
        return;
    for (std::size_t i = lo; i < hi; ++i) {
        const double phase = -w * static_cast<double>(i);
        const float c = static_cast<float>(std::cos(phase));
        const float s = static_cast<float>(std::sin(phase));
        const float re = src[i].real();
        const float im = src[i].imag();
        dst[i] = std::complex<float>(re * c - im * s, re * s + im * c);
    }
}

// dst[i] = src[i] * exp(-j*w*i) for absolute i, w in rad/sample.
// Replaces the legacy scalar four-chain rotation in stage_cfo when
// profile.rx_fast_cfo_rotation is enabled.  dst may not alias src.
inline void derotate_block(std::vector<std::complex<float>>& dst,
                           const std::complex<float>* src,
                           std::size_t n,
                           double w)
{
    dst.resize(n);
    derotate_range_block(dst.data(), src, n, 0, n, w);
}

// QA-only independent reference: per-sample double-precision cos/sin and
// double complex multiply, rounded once to float.  Not used on any hot path.
inline void derotate_reference(std::complex<float>* dst,
                               const std::complex<float>* src,
                               std::size_t n,
                               double w)
{
    if (n == 0 || src == nullptr || dst == nullptr)
        return;
    for (std::size_t i = 0; i < n; ++i) {
        const double phase = -w * static_cast<double>(i);
        const double c = std::cos(phase);
        const double s = std::sin(phase);
        const double re = static_cast<double>(src[i].real());
        const double im = static_cast<double>(src[i].imag());
        dst[i] = std::complex<float>(static_cast<float>(re * c - im * s),
                                     static_cast<float>(re * s + im * c));
    }
}

// QA-only independent range reference: exact double precision cos/sin over [lo, hi).
inline void derotate_range_reference(std::complex<float>* dst,
                                     const std::complex<float>* src,
                                     std::size_t n,
                                     std::size_t lo,
                                     std::size_t hi,
                                     double w)
{
    if (n == 0 || src == nullptr || dst == nullptr)
        return;
    hi = std::min(hi, n);
    if (lo >= hi)
        return;
    for (std::size_t i = lo; i < hi; ++i) {
        const double phase = -w * static_cast<double>(i);
        const double c = std::cos(phase);
        const double s = std::sin(phase);
        const double re = static_cast<double>(src[i].real());
        const double im = static_cast<double>(src[i].imag());
        dst[i] = std::complex<float>(static_cast<float>(re * c - im * s),
                                     static_cast<float>(re * s + im * c));
    }
}

inline constexpr int64_t kSfdNarrowHalf = 64;
inline constexpr int kSfdSymbolOffsets = 4;
inline constexpr int64_t kSfdGridHalf = 36;

// Computes the absolute sample range [lo, hi) covering the SFD candidates
// and template. The wider discrete grid is only needed on a search miss.
inline std::pair<size_t, size_t> compute_sfd_search_range(
    size_t n,
    const TimingResult& timing,
    size_t preamble_repetitions,
    size_t sfd_len,
    size_t sym = kQm35SamplesPerSymbol,
    bool include_grid = false)
{
    const double period = timing.measured_period > 0.0
        ? timing.measured_period : static_cast<double>(sym);
    const int64_t expected = timing.preamble_start_sample +
        std::llround(static_cast<double>(preamble_repetitions) * period);
    const int64_t half_span = include_grid
        ? std::llround(kSfdGridHalf * period) + kSfdNarrowHalf
        : std::llround(kSfdSymbolOffsets * period) + kSfdNarrowHalf;
    const int64_t lo = std::max<int64_t>(0, expected - half_span);
    const int64_t hi = std::min<int64_t>(
        static_cast<int64_t>(n),
        expected + half_span + static_cast<int64_t>(sfd_len));
    if (hi <= lo)
        return { 0, 0 };
    return { static_cast<size_t>(lo), static_cast<size_t>(hi) };
}

} // namespace rxopt
} // namespace demod
} // namespace openuwb
