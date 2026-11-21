// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <openuwb/phy/hrp/rx/demod_grid.h>
#include <openuwb/phy/hrp/rx/rx_cfo_opt.h>
#include <openuwb/phy/hrp/rx/rx_fused_cmf.h>
#include <openuwb/phy/hrp/rx/rx_softchip_opt.h>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <utility>

namespace openuwb::demod::core {
// Half-open [lo, hi) window-relative sample interval.
struct SampleRange {
    size_t lo = 0;
    size_t hi = 0;

    constexpr bool empty() const { return hi <= lo; }
    constexpr size_t size() const { return hi > lo ? (hi - lo) : 0; }
    constexpr bool covers(const SampleRange& other) const
    {
        return !other.empty() && !empty() && lo <= other.lo && other.hi <= hi;
    }
    friend constexpr bool operator==(const SampleRange& a, const SampleRange& b)
    {
        return a.lo == b.lo && a.hi == b.hi;
    }
};

// Tracks which window-relative interval of scratch.derotated is valid, together
// with the carrier-frequency estimate and phase origin used to produce it.
struct DerotatedSpan {
    SampleRange range{};
    double cfo_hz = 0.0;
    int64_t phase_origin_sample = 0;
    bool valid = false;

    constexpr bool covers(const SampleRange& need) const
    {
        return valid && range.covers(need);
    }
    constexpr bool covers_with_cfo(const SampleRange& need,
                                   double expected_cfo_hz,
                                   int64_t expected_origin = 0) const
    {
        return covers(need) && cfo_hz == expected_cfo_hz &&
               phase_origin_sample == expected_origin;
    }
};

inline SampleRange clamp_sample_range(int64_t lo, int64_t hi, size_t n)
{
    const int64_t clamped_lo = std::max<int64_t>(0, lo);
    const int64_t clamped_hi = std::min<int64_t>(static_cast<int64_t>(n), hi);
    if (clamped_hi <= clamped_lo)
        return SampleRange{};
    return SampleRange{static_cast<size_t>(clamped_lo),
                       static_cast<size_t>(clamped_hi)};
}

inline SampleRange compute_sfd_window_range(size_t n,
                                            const TimingResult& timing,
                                            size_t preamble_repetitions,
                                            size_t sfd_len,
                                            bool include_wide_grid = false)
{
    const auto [lo, hi] = rxopt::compute_sfd_search_range(
        n, timing, preamble_repetitions, sfd_len, kQm35SamplesPerSymbol,
        include_wide_grid);
    return SampleRange{lo, hi};
}

// Unclamped [req_lo, req_hi) bounds covering SFD, CIR, phase estimation,
// soft-chip prefix and optional fused PHR reads from a frame origin.
std::pair<int64_t, int64_t> compute_post_sfd_prefix_bounds(
    const Qm35825Profile& profile,
    int64_t frame_start,
    double measured_period,
    const SampleRange& sfd_range);

SampleRange compute_predicted_single_pass_range(
    size_t n,
    const Qm35825Profile& profile,
    const TimingResult& initial_timing,
    const SampleRange& sfd1_range);

SampleRange compute_cir_only_rotation_range(
    size_t n,
    const Qm35825Profile& profile,
    const TimingResult& cfo_timing,
    const SampleRange& final_sfd_range);

SampleRange compute_anchored_prefix_range(
    size_t n,
    const Qm35825Profile& profile,
    bool integer_grid_ok,
    const TimingResult& cfo_timing,
    const SampleRange& final_sfd_range);

SampleRange compute_fused_payload_rotation_range(
    size_t n,
    const Qm35825Profile& profile,
    const TimingResult& timing,
    size_t cir_tap_count,
    const NsSfdResult& ns_sfd,
    const PhrResult& phr,
    size_t already_rotated_hi);

SampleRange compute_ondemand_extension_rotation_range(
    size_t n,
    const Qm35825Profile& profile,
    const TimingResult& timing,
    size_t cir_tap_count,
    size_t final_budget,
    size_t already_rotated_hi);

// One admission decision shared by rotation, prefix generation and fused
// decoding. Fractional-grid packets keep the requested CIR mode.
struct PacketExecutionPlan {
    bool integer_grid_ok = true;
    Qm35825Profile effective_profile;
};
PacketExecutionPlan plan_packet_execution(
    const Qm35825Profile& profile, const TimingResult& timing);
} // namespace openuwb::demod::core
