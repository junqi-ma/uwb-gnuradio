// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <gnuradio/uwb/uwb_demod_grid.h>
#include <gnuradio/uwb/uwb_rx_cfo_opt.h>
#include <gnuradio/uwb/uwb_rx_fused_cmf.h>
#include <gnuradio/uwb/uwb_rx_softchip_opt.h>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <utility>

namespace gr::uwb::demod::core {
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
inline std::pair<int64_t, int64_t> compute_post_sfd_prefix_bounds(
    const Qm35825Profile& profile,
    int64_t frame_start,
    double measured_period,
    const SampleRange& sfd_range)
{
    const size_t pre = profile.cir_pre_samples;
    const size_t post = profile.cir_post_samples;
    const size_t tap_count = pre + post;
    const size_t first_rep = profile.cir_skip_initial_repetitions;
    const size_t full_reps = profile.preamble_repetitions;
    const size_t rep_count = (profile.cir_repetitions > 0)
        ? std::min(profile.cir_repetitions,
                   full_reps > first_rep ? full_reps - first_rep : 0)
        : 0;
    const double period = (measured_period > 0.0)
        ? measured_period
        : static_cast<double>(kQm35SamplesPerSymbol);
    const int64_t cir_lo = (rep_count > 0)
        ? frame_start + static_cast<int64_t>(std::llround(
                            static_cast<double>(first_rep) * period)) -
              static_cast<int64_t>(pre)
        : frame_start;

    const bool skip_sync_fir =
        profile.rx_fused_phr_cmf && profile.rx_fused_payload_cmf;
    int64_t req_lo = cir_lo;
    if (!skip_sync_fir)
        req_lo = std::min<int64_t>(req_lo,
                                   frame_start - static_cast<int64_t>(pre));
    req_lo = std::min<int64_t>(req_lo, static_cast<int64_t>(sfd_range.lo));

    const bool phr_fusion = profile.rx_fused_phr_cmf;
    const size_t chip_prefix = phr_fusion
        ? rxopt::fused_phr_prefix_budget(profile)
        : rxopt::initial_chip_budget(profile);
    const int64_t chip_start = frame_start + static_cast<int64_t>(post) - 1;
    int64_t req_hi = chip_start + static_cast<int64_t>(chip_prefix) * 2 +
                     static_cast<int64_t>(tap_count) + 128;
    if (phr_fusion) {
        const int64_t sfd_end_chip_est =
            static_cast<int64_t>(profile.preamble_repetitions +
                                 rxopt::kSfdSymbols) *
                static_cast<int64_t>(kQm35ChipsPerSymbol) + 8;
        const int64_t phr_start_est = sfd_end_chip_est + 1;
        const int64_t phr_len =
            static_cast<int64_t>(tap_count) +
            2 * static_cast<int64_t>(rxfusion::kPhrChipsPerBurst - 1);
        const int64_t phr_hi = chip_start +
            (phr_start_est +
             static_cast<int64_t>((rxfusion::kPhrSymbols - 1) *
                                  rxfusion::kPhrChipsPerSymbol) +
             static_cast<int64_t>(rxfusion::kPhrCandidateChipOffset[3])) * 2 +
            phr_len + 64;
        req_hi = std::max(req_hi, phr_hi);
    }
    req_hi = std::max<int64_t>(req_hi, static_cast<int64_t>(sfd_range.hi));
    return {req_lo, req_hi};
}

inline SampleRange compute_predicted_single_pass_range(
    size_t n,
    const Qm35825Profile& profile,
    const TimingResult& initial_timing,
    const SampleRange& sfd1_range)
{
    if (profile.cfo_refit_mode == 0)
        return SampleRange{};
    const bool on_demand = profile.rx_softchip_on_demand ||
                           profile.rx_fused_payload_cmf ||
                           profile.rx_fused_phr_cmf;
    if (!profile.rx_stop_after_sfd && !on_demand)
        return SampleRange{0, n};
    const auto [u_lo, u_hi] = compute_post_sfd_prefix_bounds(
        profile, initial_timing.preamble_start_sample,
        initial_timing.measured_period, sfd1_range);
    const int64_t slack =
        static_cast<int64_t>(4 * kQm35SamplesPerSymbol + 64 + 128);
    return clamp_sample_range(u_lo - slack, u_hi + slack, n);
}

inline SampleRange compute_anchored_prefix_range(
    size_t n,
    const Qm35825Profile& profile,
    bool integer_grid_ok,
    const TimingResult& cfo_timing,
    const SampleRange& final_sfd_range)
{
    if (profile.rx_stop_after_sfd) {
        return clamp_sample_range(
            static_cast<int64_t>(final_sfd_range.lo),
            static_cast<int64_t>(final_sfd_range.hi), n);
    }
    const bool on_demand = profile.rx_softchip_on_demand ||
                           profile.rx_fused_payload_cmf ||
                           profile.rx_fused_phr_cmf;
    const double nominal_spc = profile.sample_rate / 499.2e6;
    const bool is_integer_grid = (std::abs(nominal_spc - 2.0) < 1e-6);
    if (on_demand && is_integer_grid && integer_grid_ok) {
        const auto [req_lo, req_hi] = compute_post_sfd_prefix_bounds(
            profile, cfo_timing.preamble_start_sample,
            cfo_timing.measured_period, final_sfd_range);
        return clamp_sample_range(req_lo, req_hi, n);
    }
    return SampleRange{0, n};
}

inline SampleRange compute_fused_payload_rotation_range(
    size_t n,
    const Qm35825Profile& profile,
    const TimingResult& timing,
    size_t cir_tap_count,
    const NsSfdResult& ns_sfd,
    const PhrResult& phr,
    size_t already_rotated_hi)
{
    const size_t psdu_bits = static_cast<size_t>(phr.psdu_length) * 8;
    const size_t num_blocks = (psdu_bits + 329) / 330;
    const size_t nsym = psdu_bits + 48 * num_blocks;
    const int64_t phr_start = ns_sfd.sfd_end_chip + 1;
    const int64_t payload_start = phr_start + 21 * 512;
    const int64_t chip_start = timing.preamble_start_sample +
        static_cast<int64_t>(cir_tap_count) -
        static_cast<int64_t>(profile.cir_pre_samples) - 1;
    const int64_t base0 = chip_start + payload_start * 2 -
        static_cast<int64_t>(cir_tap_count) + 1;
    const int64_t last_chip = static_cast<int64_t>(nsym - 1) * 64 + 40;
    const size_t len = cir_tap_count + 2 * (8 - 1);
    const int64_t last_sample =
        base0 + last_chip * 2 + static_cast<int64_t>(len);
    const size_t pay_lo = std::min(
        already_rotated_hi,
        static_cast<size_t>(std::max<int64_t>(0, base0)));
    const size_t pay_hi = static_cast<size_t>(
        std::min<int64_t>(static_cast<int64_t>(n), last_sample + 64));
    if (pay_hi <= pay_lo)
        return SampleRange{};
    return SampleRange{pay_lo, pay_hi};
}

inline SampleRange compute_ondemand_extension_rotation_range(
    size_t n,
    const Qm35825Profile& profile,
    const TimingResult& timing,
    size_t cir_tap_count,
    size_t final_budget,
    size_t already_rotated_hi)
{
    const int64_t chip_start = timing.preamble_start_sample +
        static_cast<int64_t>(cir_tap_count) -
        static_cast<int64_t>(profile.cir_pre_samples) - 1;
    const int64_t last_pos =
        chip_start + static_cast<int64_t>(final_budget) * 2 +
        static_cast<int64_t>(cir_tap_count) + 64;
    const size_t pay_lo = already_rotated_hi;
    const size_t pay_hi = static_cast<size_t>(
        std::min<int64_t>(static_cast<int64_t>(n), last_pos));
    if (pay_hi <= pay_lo)
        return SampleRange{};
    return SampleRange{pay_lo, pay_hi};
}

// One admission decision shared by rotation, prefix generation and fused
// decoding. Fractional-grid packets keep the requested CIR mode.
struct PacketExecutionPlan {
    bool integer_grid_ok = true;
    Qm35825Profile effective_profile;
};
inline PacketExecutionPlan plan_packet_execution(
    const Qm35825Profile& profile, const TimingResult& timing)
{
    PacketExecutionPlan plan;
    plan.effective_profile = profile;
    if (profile.rx_softchip_on_demand || profile.rx_fused_payload_cmf ||
        profile.rx_fused_phr_cmf) {
        const double spc_hat = (timing.measured_period > 0.0)
            ? timing.measured_period / static_cast<double>(kQm35ChipsPerSymbol)
            : 0.0;
        plan.integer_grid_ok = integer_chip_grid(spc_hat, profile.fused_gate_ppm * 1e-6);
        if (!plan.integer_grid_ok) {
            plan.effective_profile.rx_softchip_on_demand = false;
            plan.effective_profile.rx_fused_payload_cmf = false;
            plan.effective_profile.rx_fused_phr_cmf = false;
        }
    }
    return plan;
}
} // namespace gr::uwb::demod::core
