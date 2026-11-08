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

#include <gnuradio/uwb/uwb_demod_result.h>
#include <gnuradio/uwb/uwb_phy_profile.h>
#include <volk/volk.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <utility>
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

        const std::complex<float>* src_ptr = src + b0 + k_start;
        std::complex<float>* dst_ptr = dst + b0 + k_start;
        const std::complex<float>* table_ptr = table.data() + k_start;

        volk_32fc_x2_multiply_32fc(
            reinterpret_cast<lv_32fc_t*>(dst_ptr),
            reinterpret_cast<const lv_32fc_t*>(src_ptr),
            reinterpret_cast<const lv_32fc_t*>(table_ptr),
            static_cast<unsigned>(len));

        if (b0 != 0) {
            const double phase = -w * static_cast<double>(b0);
            const float ar = static_cast<float>(std::cos(phase));
            const float ai = static_cast<float>(std::sin(phase));
            volk_32fc_s32fc_multiply_32fc(
                reinterpret_cast<lv_32fc_t*>(dst_ptr),
                reinterpret_cast<const lv_32fc_t*>(dst_ptr),
                lv_cmake(ar, ai),
                static_cast<unsigned>(len));
        }
    }
}

// Scalar reference / fallback range derotation.  Phase origin is strictly
// sample 0 (i.e. base sample index has phase -w * i, not treating lo as 0).
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

// Computes the absolute sample range [lo, hi) covering all SFD search candidates,
// ±4 symbol backtrack, ±64 search halo, and the full SFD template length.
inline std::pair<size_t, size_t> compute_sfd_search_range(
    size_t n,
    const TimingResult& timing,
    size_t preamble_repetitions,
    size_t sfd_len,
    size_t sym = kQm35SamplesPerSymbol)
{
    constexpr int64_t kSfdNarrowHalf = 64;
    constexpr int kSfdSymbolOffsets = 4;
    const int64_t period =
        (timing.measured_period > 0.0)
            ? static_cast<int64_t>(std::llround(timing.measured_period))
            : static_cast<int64_t>(sym);
    const int64_t expected = timing.preamble_start_sample +
        static_cast<int64_t>(preamble_repetitions) * period;

    const int64_t center_min = expected -
        static_cast<int64_t>(kSfdSymbolOffsets) * static_cast<int64_t>(sym);
    const int64_t center_max = expected +
        static_cast<int64_t>(kSfdSymbolOffsets) * static_cast<int64_t>(sym);

    const int64_t lo = std::max<int64_t>(0, center_min - kSfdNarrowHalf);
    const int64_t hi = std::min<int64_t>(
        static_cast<int64_t>(n),
        center_max + kSfdNarrowHalf + static_cast<int64_t>(sfd_len));
    if (hi <= lo)
        return { 0, 0 };
    return { static_cast<size_t>(lo), static_cast<size_t>(hi) };
}

// Sparse ternary code template for CIR estimation (Task A2).
// Iterates only non-zero positions (+-1) against accumulated IQ, preserving
// tap order, position, and energy normalization.
struct SparseCirEntry {
    size_t pos = 0;      // sample index in [0, 1016), always even: 2 * j
    int8_t sign = 0;     // +1 or -1
};

struct SparseCirTemplate {
    uint64_t key = 0;
    std::vector<SparseCirEntry> entries;
    float code_energy = 0.0f;

    void reset()
    {
        key = 0;
        entries.clear();
        code_energy = 0.0f;
    }
};

inline uint64_t cir_template_key(const int8_t* code, size_t code_len)
{
    uint64_t h = 14695981039346656037ULL;
    for (size_t i = 0; i < code_len; ++i) {
        h ^= static_cast<uint8_t>(code[i]);
        h *= 1099511628211ULL;
    }
    return h;
}

inline void build_sparse_cir_template(const int8_t* code,
                                      size_t code_len,
                                      SparseCirTemplate& tmpl)
{
    tmpl.reset();
    tmpl.key = cir_template_key(code, code_len);
    const std::vector<int8_t> spread = BuildSampledCode(code, code_len);
    tmpl.entries.reserve(code_len);
    for (size_t j = 0; j < spread.size(); ++j) {
        if (spread[j] != 0) {
            tmpl.entries.push_back({ 2 * j, spread[j] });
        }
    }
    tmpl.code_energy = static_cast<float>(tmpl.entries.size());
}

inline void correlate_cir_sparse(const std::complex<float>* corr,
                                 const SparseCirTemplate& tmpl,
                                 size_t tap_count,
                                 std::vector<std::complex<float>>& values)
{
    values.resize(tap_count);
    if (tmpl.code_energy <= 0.0f)
        return;
    for (size_t nn = 0; nn < tap_count; ++nn) {
        std::complex<float> acc(0.0f, 0.0f);
        const std::complex<float>* pcorr = corr + nn;
        for (const auto& e : tmpl.entries) {
            const std::complex<float>& s = pcorr[e.pos];
            if (e.sign > 0)
                acc += s;
            else
                acc -= s;
        }
        values[nn] = acc / tmpl.code_energy;
    }
}

} // namespace rxopt
} // namespace demod
} // namespace uwb
} // namespace gr
