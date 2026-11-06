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
 * replaces the per-sample scalar work with two runtime-dispatched VOLK kernels
 * per block:
 *
 *     dst[b0 + k] = src[b0 + k] * table[k]          (table[k] = e^{-j w k})
 *     dst[b0 + k] *= e^{-j w b0}                    (one anchor per block)
 *
 * The 1024-entry table is built once per call (w changes with the frame) from
 * four independent float chains advanced by step^4 = e^{-j 4 w}; each chain is
 * re-anchored from the exact double-precision phase every
 * kCfoTableReanchorHops steps.  The block anchor is advanced in double by one
 * complex multiply per block and re-anchored from the exact absolute phase
 * every kCfoAnchorReanchorBlocks blocks, so neither table drift nor anchor
 * drift accumulates.
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

#include <volk/volk.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <cstddef>
#include <vector>

namespace gr {
namespace uwb {
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

} // namespace detail

// dst[i] = src[i] * exp(-j*w*i) for absolute i, w in rad/sample.
// Replaces the legacy scalar four-chain rotation in stage_cfo when
// profile.rx_fast_cfo_rotation is enabled.  dst may not alias src.
inline void derotate_block(std::vector<std::complex<float>>& dst,
                           const std::complex<float>* src,
                           std::size_t n,
                           double w)
{
    dst.resize(n);
    if (n == 0 || src == nullptr)
        return;

    std::array<std::complex<float>, kCfoDerotateBlock> table;
    detail::build_cfo_table(table, w);

    // Double-precision block anchor exp(-j*w*b0).  One complex multiply per
    // block, with an exact sin/cos re-anchor every kCfoAnchorReanchorBlocks.
    const double block_step_r =
        std::cos(-w * static_cast<double>(kCfoDerotateBlock));
    const double block_step_i =
        std::sin(-w * static_cast<double>(kCfoDerotateBlock));
    double ar = 1.0;
    double ai = 0.0;
    std::size_t block = 0;

    for (std::size_t b0 = 0; b0 < n; b0 += kCfoDerotateBlock, ++block) {
        if (block != 0) {
            if ((block % kCfoAnchorReanchorBlocks) == 0) {
                const double phase = -w * static_cast<double>(b0);
                ar = std::cos(phase);
                ai = std::sin(phase);
            } else {
                const double nr = ar * block_step_r - ai * block_step_i;
                const double ni = ar * block_step_i + ai * block_step_r;
                ar = nr;
                ai = ni;
            }
        }
        const std::size_t len = std::min(kCfoDerotateBlock, n - b0);
        volk_32fc_x2_multiply_32fc(
            reinterpret_cast<lv_32fc_t*>(dst.data() + b0),
            reinterpret_cast<const lv_32fc_t*>(src + b0),
            reinterpret_cast<const lv_32fc_t*>(table.data()),
            static_cast<unsigned>(len));
        if (block != 0) {
            // Fold the absolute block phase into the same pass result.  In
            // place is safe: VOLK loads before it stores each element.
            volk_32fc_s32fc_multiply_32fc(
                reinterpret_cast<lv_32fc_t*>(dst.data() + b0),
                reinterpret_cast<const lv_32fc_t*>(dst.data() + b0),
                lv_cmake(static_cast<float>(ar), static_cast<float>(ai)),
                static_cast<unsigned>(len));
        }
    }
}

// QA-only independent reference: per-sample double-precision cos/sin and
// double complex multiply, rounded once to float.  Not used on any hot path.
inline void derotate_reference(std::complex<float>* dst,
                               const std::complex<float>* src,
                               std::size_t n,
                               double w)
{
    if (n == 0)
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

} // namespace rxopt
} // namespace demod
} // namespace uwb
} // namespace gr
