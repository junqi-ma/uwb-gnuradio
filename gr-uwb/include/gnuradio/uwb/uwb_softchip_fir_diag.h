/* -*- c++ -*- */
/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Diagnostic-only replay of the legacy soft-chip FIR, segmented at decoded
 * field boundaries.  The production stage_cir_softchips() times the complete
 * FIR as one block.  Once the PHR/payload boundaries are known, this helper
 * replays the same kernels over disjoint OUTPUT-chip ranges:
 *
 *   preamble [0, sfd_start)
 *   SFD      [sfd_start, phr_start)
 *   PHR      [phr_start, payload_start)
 *   payload  [payload_start, payload_end)
 *   extra    [payload_end, total_chips)
 *
 * Each output chip belongs to exactly one segment; overlapping input halos are
 * not counted twice.  This is an independent diagnostic measurement, not a
 * production-accurate per-field attribution, and it must be run outside formal
 * timing comparisons.  No per-chip clock calls are made.
 */

#pragma once

#include <gnuradio/uwb/uwb_cir_fir_simd.h>
#include <gnuradio/uwb/uwb_demod_result.h>
#include <gnuradio/uwb/uwb_rx_fused_cmf.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace gr {
namespace uwb {
namespace demod {
namespace core {
namespace detail {

struct SoftFirSegments {
    bool valid = false;
    uint64_t preamble_us = 0;
    uint64_t sfd_us = 0;
    uint64_t phr_us = 0;
    uint64_t payload_us = 0;
    uint64_t extra_us = 0;
    size_t preamble_chips = 0;
    size_t sfd_chips = 0;
    size_t phr_chips = 0;
    size_t payload_chips = 0;
    size_t extra_chips = 0;
};

inline bool diagnose_softchip_fir_segments(
    const std::complex<float>* rx,
    size_t n,
    const std::complex<float>* fir_taps,
    size_t tap_count,
    int64_t chip_start,
    size_t total_chips,
    double samples_per_chip,
    const NsSfdResult& ns_sfd,
    const PhrResult& phr,
    size_t payload_symbols,
    std::vector<std::complex<float>>& work,
    SoftFirSegments& out)
{
    out = SoftFirSegments{};
    if (!rx || !fir_taps || tap_count == 0 || tap_count > 64 ||
        chip_start < 0 || total_chips == 0 ||
        !(samples_per_chip > 0.0) || !std::isfinite(samples_per_chip) ||
        !ns_sfd.ok || ns_sfd.sfd_start_chip < 0 ||
        ns_sfd.sfd_end_chip < 0 || !phr.ok)
        return false;

    const int64_t phr_start = ns_sfd.sfd_end_chip + 1;
    const int64_t payload_start =
        phr_start + static_cast<int64_t>(rxfusion::kPhrSymbols) *
                        static_cast<int64_t>(rxfusion::kPhrChipsPerSymbol);
    if (phr_start < 0 || payload_start < 0)
        return false;
    const int64_t payload_end =
        payload_start + static_cast<int64_t>(payload_symbols) *
                            static_cast<int64_t>(rxfusion::kChipsPerSymbol);
    if (payload_end < payload_start)
        return false;

    const int64_t total = static_cast<int64_t>(total_chips);
    auto clip = [&](int64_t value) {
        return std::min<int64_t>(total, std::max<int64_t>(0, value));
    };
    const int64_t b1 = clip(ns_sfd.sfd_start_chip);
    const int64_t b2 = std::max(b1, clip(phr_start));
    const int64_t b3 = std::max(b2, clip(payload_start));
    const int64_t b4 = std::max(b3, clip(payload_end));

    {
        const size_t sum = static_cast<size_t>(b1) +
                           static_cast<size_t>(b2 - b1) +
                           static_cast<size_t>(b3 - b2) +
                           static_cast<size_t>(b4 - b3) +
                           static_cast<size_t>(total - b4);
        if (sum != total_chips)
            return false;
    }

    // The replay mirrors the production edge/main split and kernel choice.
    const int64_t fwd = static_cast<int64_t>(tap_count) - 1;
    const int64_t spc_i = std::max<int64_t>(1, std::llround(samples_per_chip));
    const bool fractional =
        std::abs(samples_per_chip - static_cast<double>(spc_i)) > 1e-9;
    alignas(32) std::complex<float> values_nat[64];
    for (size_t q = 0; q < tap_count; ++q)
        values_nat[q] = std::conj(fir_taps[q]);
    const cir_fir::Kernel fir_k = cir_fir::kDefaultKernel;

    work.resize(total_chips);
    auto scalar_dot = [&](int64_t p) {
        std::complex<float> acc(0.0f, 0.0f);
        for (size_t q = 0; q < tap_count; ++q) {
            const int64_t idx = p - fwd + static_cast<int64_t>(q);
            if (idx >= 0 && idx < static_cast<int64_t>(n))
                acc += fir_taps[q] * rx[static_cast<size_t>(idx)];
        }
        return acc;
    };

    auto run = [&](int64_t lo, int64_t hi, uint64_t& elapsed_us,
                   size_t& chips) {
        chips = static_cast<size_t>(std::max<int64_t>(0, hi - lo));
        if (hi <= lo) {
            elapsed_us = 0;
            return;
        }
        volatile double sink = 0.0;
        const auto t0 = std::chrono::steady_clock::now();
        for (int64_t i = lo; i < hi; ++i) {
            const std::complex<float> value = [&]() {
                if (fractional) {
                    const double pd =
                        static_cast<double>(chip_start) +
                        static_cast<double>(i) * samples_per_chip;
                    const int64_t p0 = static_cast<int64_t>(std::floor(pd));
                    const float frac = static_cast<float>(pd - std::floor(pd));
                    const std::complex<float> y0 = scalar_dot(p0);
                    const std::complex<float> y1 = scalar_dot(p0 + 1);
                    return y0 + frac * (y1 - y0);
                }
                // Guard arithmetic overflow on p
                if (spc_i > 0 &&
                    i > (std::numeric_limits<int64_t>::max() - 1000 - chip_start) /
                            static_cast<int64_t>(spc_i))
                    return scalar_dot(chip_start);
                const int64_t p =
                    chip_start + i * static_cast<int64_t>(spc_i);
                if (p < fwd)
                    return scalar_dot(p);
                const int64_t p_sub_fwd = p - fwd;
                if (static_cast<uint64_t>(p_sub_fwd) >
                    std::numeric_limits<size_t>::max() - tap_count)
                    return scalar_dot(p);
                const size_t win = static_cast<size_t>(p_sub_fwd);
                // Bounded edge calculation: if the FIR window extends past the right
                // IQ bound (win + tap_count > n), safely fall back to scalar_dot()
                // which only accumulates valid in-bounds samples.
                if (win + tap_count > n)
                    return scalar_dot(p);
                if (fir_k == cir_fir::Kernel::Avx2Fixed &&
                    tap_count == cir_fir::kDefaultTapCount)
                    return cir_fir::dot_avx2_fixed38(
                        rx + win, fir_taps, tap_count);
                if (fir_k == cir_fir::Kernel::Volk)
                    return cir_fir::dot_volk(rx + win, values_nat, tap_count);
                return cir_fir::dot_multi_acc8(rx + win, fir_taps, tap_count);
            }();
            work[static_cast<size_t>(i)] = value;
            sink += value.real();
        }
        elapsed_us = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::steady_clock::now() - t0)
                .count());
        if (!std::isfinite(static_cast<double>(sink)))
            elapsed_us = 0; // keep the sink observable; unreachable in practice
    };

    run(0, b1, out.preamble_us, out.preamble_chips);
    run(b1, b2, out.sfd_us, out.sfd_chips);
    run(b2, b3, out.phr_us, out.phr_chips);
    run(b3, b4, out.payload_us, out.payload_chips);
    run(b4, total, out.extra_us, out.extra_chips);
    out.valid = true;
    return true;
}

} // namespace detail
} // namespace core
} // namespace demod
} // namespace uwb
} // namespace gr
