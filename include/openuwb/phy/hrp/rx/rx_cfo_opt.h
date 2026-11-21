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
void derotate_block_anchored(std::complex<float>* dst,
                             const std::complex<float>* src,
                             const std::complex<float>* table,
                             std::size_t len,
                             float ar,
                             float ai);

} // namespace detail

// dst[i] = src[i] * exp(-j*w*i) for absolute i in [lo, hi), w in rad/sample.
// Phase origin is strictly sample 0 (not lo). dst may not alias src.
void derotate_range_block(std::complex<float>* dst,
                          const std::complex<float>* src,
                          std::size_t n,
                          std::size_t lo,
                          std::size_t hi,
                          double w);

// 4-chain recurrence with 1024-sample block re-anchoring.
// Matches the legacy scalar stage_cfo rotation speed and accuracy, generalized
// to operate on an arbitrary absolute range [lo, hi) where phase origin is sample 0.
void derotate_range_recurrence(std::complex<float>* dst,
                               const std::complex<float>* src,
                               std::size_t n,
                               std::size_t lo,
                               std::size_t hi,
                               double w);

// Scalar reference range derotation.  Phase origin is strictly
// sample 0 (i.e. base sample index has phase -w * i, not treating lo as 0).
// Kept for QA and numerical verification; not used on hot demod paths.
void derotate_range_scalar(std::complex<float>* dst,
                           const std::complex<float>* src,
                           std::size_t n,
                           std::size_t lo,
                           std::size_t hi,
                           double w);

// dst[i] = src[i] * exp(-j*w*i) for absolute i, w in rad/sample.
// Replaces the legacy scalar four-chain rotation in stage_cfo when
// profile.rx_fast_cfo_rotation is enabled.  dst may not alias src.
void derotate_block(std::vector<std::complex<float>>& dst,
                    const std::complex<float>* src,
                    std::size_t n,
                    double w);

// QA-only independent reference: per-sample double-precision cos/sin and
// double complex multiply, rounded once to float.  Not used on any hot path.
void derotate_reference(std::complex<float>* dst,
                        const std::complex<float>* src,
                        std::size_t n,
                        double w);

// QA-only independent range reference: exact double precision cos/sin over [lo, hi).
void derotate_range_reference(std::complex<float>* dst,
                              const std::complex<float>* src,
                              std::size_t n,
                              std::size_t lo,
                              std::size_t hi,
                              double w);

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
