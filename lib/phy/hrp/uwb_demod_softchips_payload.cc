// SPDX-License-Identifier: GPL-3.0-or-later


// Implementations for the RX soft-chip, PHR, payload and fused-CMF stages.
// The headers keep only declarations; see include/openuwb/phy/hrp/rx/.
#include <openuwb/phy/hrp/rx/demod_softchips.h>
#include <openuwb/phy/hrp/rx/demod_phr.h>
#include <openuwb/phy/hrp/rx/demod_payload.h>
#include <openuwb/phy/hrp/rx/demod_execution.h>
#include <openuwb/phy/hrp/rx/rx_fused_cmf.h>
#include <openuwb/phy/hrp/rx/rx_softchip_opt.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <utility>
#include <vector>


// ---- rx_softchip_opt.h ----

namespace openuwb::demod::rxopt {
bool sync_phase_gain(const std::complex<float>* rx, size_t n,
                     int64_t base, const std::complex<float>* h,
                     size_t taps, const std::vector<int8_t>& spread,
                     size_t reps, std::vector<std::complex<float>>& sum,
                     std::complex<float>& gain)
{
    gain = {};
    if (!rx || !h || taps == 0 || taps > 64 || spread.empty() ||
        spread.size() > 508 || reps == 0 || reps > 32 || base < -64)
        return false;
    const size_t stride = 2 * spread.size();
    const size_t span = 2 * (spread.size() - 1) + taps;
    const size_t extent = (reps - 1) * stride + span;
    // Avoid additions to a caller-controlled signed coordinate.
    if (base >= 0 && (static_cast<uint64_t>(base) > n ||
        extent > n - static_cast<size_t>(base)))
        return false;
    if (base < 0 && extent > n + static_cast<size_t>(-base))
        return false;
    sum.assign(span, std::complex<float>{});
    for (size_t r = 0; r < reps; ++r) {
        const int64_t first = base + static_cast<int64_t>(r * stride);
        const size_t lo = first < 0 ? static_cast<size_t>(-first) : 0;
        if (lo == 0 && static_cast<uint64_t>(first + span) <= n) {
            vector_accumulate_complex(sum.data(), rx + first, span);
        } else {
            for (size_t m = lo; m < span; ++m)
                sum[m] += rx[static_cast<size_t>(first + static_cast<int64_t>(m))];
        }
    }
    for (size_t c = 0; c < spread.size(); ++c) {
        if (!spread[c])
            continue;
        std::complex<float> v{};
        for (size_t q = 0; q < taps; ++q)
            v += h[q] * sum[2 * c + q];
        gain += static_cast<float>(spread[c]) * v;
    }
    return std::isfinite(gain.real()) && std::isfinite(gain.imag());
}

bool sync_phase_gain_from_tail(const std::complex<float>* tail,
                               const std::complex<float>* h,
                               size_t taps,
                               const std::vector<int8_t>& spread,
                               std::complex<float>& gain)
{
    gain = {};
    if (!tail || !h || taps == 0 || taps > 64 || spread.empty() ||
        spread.size() > 508)
        return false;
    for (size_t c = 0; c < spread.size(); ++c) {
        if (!spread[c])
            continue;
        std::complex<float> v{};
        for (size_t q = 0; q < taps; ++q)
            v += h[q] * tail[2 * c + q];
        gain += static_cast<float>(spread[c]) * v;
    }
    return std::isfinite(gain.real()) && std::isfinite(gain.imag());
}

size_t payload_chips_needed(uint32_t psdu_len)
{
    const size_t psdu_bits = static_cast<size_t>(psdu_len) * 8;
    const size_t num_blocks = (psdu_bits + 329) / 330;
    const size_t nsym = psdu_bits + 48 * num_blocks;
    return kPhrSymbols * kPhrChipsPerSymbol + nsym * kPayloadChipsPerSymbol;
}

size_t legacy_chip_budget(const Qm35825Profile& profile)
{
    const size_t n_shr =
        (profile.preamble_repetitions + kSfdSymbols) * kQm35ChipsPerSymbol;
    const size_t n_phr_head = kLegacyPhrHeadSymbols * kQm35ChipsPerSymbol;
    const size_t extra_bytes =
        (profile.max_psdu_bytes > 12) ? (profile.max_psdu_bytes - 12) : 0;
    const size_t n_extra = static_cast<size_t>(
        std::ceil(static_cast<double>(extra_bytes) *
                  kLegacyExtraChipsPerByte));
    const size_t n_guard = kLegacyGuardSymbols * kQm35ChipsPerSymbol;
    size_t n_chips = n_shr + n_phr_head + n_extra + n_guard;
    n_chips = static_cast<size_t>(
        std::ceil(static_cast<double>(n_chips) * kLegacyBudgetFactor));
    return n_chips;
}

size_t initial_chip_budget(const Qm35825Profile& profile)
{
    const size_t n_preamble =
        (profile.preamble_repetitions + kSfdSymbols) * kQm35ChipsPerSymbol;
    const size_t n_phr_head = kLegacyPhrHeadSymbols * kQm35ChipsPerSymbol;
    size_t budget = static_cast<size_t>(
        std::ceil(static_cast<double>(n_preamble + n_phr_head) *
                  kLegacyBudgetFactor));
    const size_t cap = legacy_chip_budget(profile);
    return std::min(budget, cap);
}

size_t fused_phr_prefix_budget(const Qm35825Profile& profile)
{
    const size_t expected =
        profile.preamble_repetitions * kQm35ChipsPerSymbol;
    const size_t ns_sfd_positive_shift = 8;
    const size_t sfd_len = kSfdSymbols * kQm35ChipsPerSymbol;
    size_t budget = expected + ns_sfd_positive_shift + sfd_len + 2;
    const size_t cap = legacy_chip_budget(profile);
    return std::min(budget, cap);
}

size_t final_chip_budget(const Qm35825Profile& profile,
                         int64_t sfd_end_chip,
                         uint32_t psdu_length)
{
    if (sfd_end_chip < 0)
        return 0;
    const uint64_t neighbourhood =
        static_cast<uint64_t>(psdu_length) + 2u;
    const uint64_t capped_len =
        std::min<uint64_t>(neighbourhood,
                           static_cast<uint64_t>(profile.max_psdu_bytes));
    uint64_t wanted = static_cast<uint64_t>(sfd_end_chip) + 1u;
    wanted += static_cast<uint64_t>(
        payload_chips_needed(static_cast<uint32_t>(capped_len)));
    wanted += static_cast<uint64_t>(kFinalGuardChips);
    const size_t cap = legacy_chip_budget(profile);
    return static_cast<size_t>(std::min<uint64_t>(
        wanted, static_cast<uint64_t>(cap)));
}

} // namespace openuwb::demod::rxopt

// ---- demod_execution.h ----

namespace openuwb::demod::core {
std::pair<int64_t, int64_t> compute_post_sfd_prefix_bounds(
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
    if (!skip_sync_fir) {
        req_lo = std::min<int64_t>(req_lo,
                                   frame_start - static_cast<int64_t>(pre));
        req_lo = std::min<int64_t>(req_lo, static_cast<int64_t>(sfd_range.lo));
    } else {
        // `--fused on` keeps the CIR estimator on the raw frame (per-SYNC CFO
        // scalar) and reuses its tail average for the SYNC phase.  Nothing
        // below the SFD search window reads the preamble from `derotated`, so
        // the whole ~250k-sample preamble no longer has to be rotated.
        req_lo = static_cast<int64_t>(sfd_range.lo);
    }

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

SampleRange compute_predicted_single_pass_range(
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

SampleRange compute_cir_only_rotation_range(
    size_t n,
    const Qm35825Profile& profile,
    const TimingResult& cfo_timing,
    const SampleRange& final_sfd_range)
{
    // `--stop-after-cir on` calls estimate_cir() directly on the raw frame with
    // the CFO applied per SYNC, so the CIR windows never read `derotated`.
    // Only the SFD search window has to be rotated.
    (void)profile;
    (void)cfo_timing;
    return clamp_sample_range(static_cast<int64_t>(final_sfd_range.lo),
                              static_cast<int64_t>(final_sfd_range.hi), n);
}

SampleRange compute_anchored_prefix_range(
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
    if (profile.rx_stop_after_cir) {
        return compute_cir_only_rotation_range(
            n, profile, cfo_timing, final_sfd_range);
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

SampleRange compute_fused_payload_rotation_range(
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

SampleRange compute_ondemand_extension_rotation_range(
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

PacketExecutionPlan plan_packet_execution(
    const Qm35825Profile& profile, const TimingResult& timing)
{
    PacketExecutionPlan plan;
    plan.effective_profile = profile;
    if (profile.rx_softchip_on_demand || profile.rx_fused_payload_cmf ||
        profile.rx_fused_phr_cmf) {
        const double spc_hat = (timing.measured_period > 0.0)
            ? timing.measured_period / static_cast<double>(kQm35ChipsPerSymbol)
            : 0.0;
        const bool int_ok =
            integer_chip_grid(spc_hat, profile.fused_gate_ppm * 1e-6);
        if (profile.rx_fused_sfo_track) {
            // The fused SFO path keeps the on-demand prefix and tracks the
            // fractional payload phase with the 8-bin CIR bank.  Accept the
            // full +-65 ppm the bank is sized for; a larger offset still falls
            // back to the legacy fractional full stream.  On an integer grid
            // the SFO machinery is pure overhead, so it is switched off.
            const double lo = 2.0 * (1.0 - 65e-6);
            const double hi = 2.0 * (1.0 + 65e-6);
            const bool sfo_ok = spc_hat >= lo && spc_hat <= hi;
            plan.integer_grid_ok = int_ok || sfo_ok;
            if (plan.integer_grid_ok && int_ok)
                plan.effective_profile.rx_fused_sfo_track = false;
        } else {
            plan.integer_grid_ok = int_ok;
        }
        if (!plan.integer_grid_ok) {
            plan.effective_profile.rx_softchip_on_demand = false;
            plan.effective_profile.rx_fused_payload_cmf = false;
            plan.effective_profile.rx_fused_phr_cmf = false;
            plan.effective_profile.rx_fused_sfo_track = false;
        }
    }
    return plan;
}

} // namespace openuwb::demod::core

// ---- demod_softchips.h ----

namespace openuwb::demod::core {
bool estimate_cir(const std::complex<float>* rx,
                  size_t n,
                  const Qm35825Profile& profile,
                  const TimingResult& timing,
                  const std::vector<int8_t>& preamble_code,
                  CirResult& out,
                  DemodScratch& scratch,
                  double cfo_hz)
{
    out = CirResult{};
    auto t_cir0 = std::chrono::steady_clock::now();
    const size_t pre = profile.cir_pre_samples;
    const size_t post = profile.cir_post_samples;
    const size_t tap_count = pre + post;
    const double period = timing.measured_period;
    const int64_t start = timing.preamble_start_sample;
    if (period <= 0.0 || start < 0 || preamble_code.empty() || tap_count == 0)
        return false;

    // ---- Build (or reuse) the sparse sampled code (508 spread x2 -> 1016, zeros on the
    //      odd samples), matching MATLAB buildUwbReference.sampled_code. ----
    if (scratch.cached_seq.code_index != profile.code_index ||
        scratch.cached_seq.sampled_pcode.empty()) {
        scratch.cached_seq.update(profile.code_index, profile.sfd_mode);
    }
    rxopt::ensure_sparse_cir_template(
        preamble_code.data(), preamble_code.size(), scratch.cir_template);
    if (scratch.cir_template.code_energy <= 0.0f)
        return false;
    const size_t code_len = 2 * kQm35ChipsPerSymbol; // 1016

    // ---- Coherently average the aligned SYNC windows. ----
    const size_t available = std::min(
        timing.cir_grid_repetitions ? timing.cir_grid_repetitions : timing.detected_peaks,
        profile.preamble_repetitions);
    if (available == 0 || profile.cir_skip_initial_repetitions >= available)
        return false;
    const size_t first_rep = profile.cir_skip_initial_repetitions;
    const size_t rep_count = std::min(profile.cir_repetitions,
                                      available - first_rep);
    if (rep_count == 0)
        return false;
    // Match acceleration/estimateCir.m's explicit-skip branch: begin after
    // cir_skip_initial_repetitions and consume at most cir_repetitions.
    // The runtime profile sets count=preamble-skip when all remaining SYNCs
    // should participate.

    const size_t wlen = code_len + tap_count - 1; // 1053
    // A few zero-padded complex samples let the widest sparse kernel finish
    // without a scalar tail.
    scratch.corr.assign(wlen + 8, std::complex<float>(0.0f, 0.0f));
    scratch.cir_avg_tail_valid = false;

    // ---- Tail average for `sync_phase_gain` reuse. ----
    // Mirrors its exact integer grid (stride 2*chips_per_symbol) and span so
    // the phase statistic is the same one the soft-chip stage used to
    // recompute from the frame.  Filled for cfo_hz == 0 too: when the caller
    // already passes a derotated frame the windows are simply summed directly,
    // and the soft-chip stage can always consume the result.
    const size_t tail_reps = profile.preamble_repetitions;
    const size_t tail_n = std::min<size_t>(32, tail_reps);
    const size_t tail_span = 2 * (kQm35ChipsPerSymbol - 1) + tap_count;
    const size_t tail_k0 = tail_reps - tail_n;
    const int64_t tail_stride =
        static_cast<int64_t>(2 * kQm35ChipsPerSymbol);
    // The tail can ride on the main SYNC loop only when every one of its
    // repetitions is inside the averaged range, lands exactly on the integer
    // grid, and keeps the full wlen window in bounds.  Otherwise the
    // standalone tail loop below re-reads the frame (unchanged behaviour).
    bool fused_tail = tail_n > 0 && tail_span <= wlen &&
        tail_k0 >= first_rep && tail_k0 + tail_n <= first_rep + rep_count;
    if (fused_tail) {
        for (size_t k = tail_k0; k < tail_reps; ++k) {
            const int64_t rs = start + static_cast<int64_t>(
                std::llround(static_cast<double>(k) * period));
            const int64_t lo = rs - static_cast<int64_t>(pre);
            if (rs != start + static_cast<int64_t>(k) * tail_stride ||
                lo < 0 || lo + static_cast<int64_t>(wlen) >
                              static_cast<int64_t>(n)) {
                fused_tail = false;
                break;
            }
        }
    }
    if (fused_tail)
        scratch.cir_avg_tail.assign(tail_span, std::complex<float>(0.0f, 0.0f));

    const bool fused_rotation = cfo_hz != 0.0;
    const double w = fused_rotation
        ? 2.0 * M_PI * cfo_hz / profile.sample_rate
        : 0.0;
    size_t valid = 0;
    for (size_t k = first_rep; k < first_rep + rep_count; ++k) {
        const int64_t rs = start + static_cast<int64_t>(std::llround((double)k * period));
        const int64_t lo = rs - static_cast<int64_t>(pre);
        const int64_t hi = lo + static_cast<int64_t>(wlen);
        if (lo < 0 || hi > static_cast<int64_t>(n))
            continue; // drop SYNCs clipped by the buffer edge (like MATLAB)
        if (fused_rotation) {
            const double phase = -w * static_cast<double>(lo);
            const std::complex<float> alpha(
                static_cast<float>(std::cos(phase)),
                static_cast<float>(std::sin(phase)));
            rxopt::vector_accumulate_scaled_complex(
                scratch.corr.data(), rx + static_cast<size_t>(lo), alpha,
                wlen);
            // Same window, same per-SYNC scalar, same ascending SYNC order as
            // the standalone tail loop: the tail average is bit-identical.
            if (fused_tail && k >= tail_k0) {
                rxopt::vector_accumulate_scaled_complex(
                    scratch.cir_avg_tail.data(), rx + static_cast<size_t>(lo),
                    alpha, tail_span);
            }
        } else {
            rxopt::vector_accumulate_complex(
                scratch.corr.data(),
                rx + static_cast<size_t>(lo),
                wlen);
            if (fused_tail && k >= tail_k0) {
                rxopt::vector_accumulate_complex(
                    scratch.cir_avg_tail.data(), rx + static_cast<size_t>(lo),
                    tail_span);
            }
        }
        ++valid;
    }
    if (valid == 0)
        return false;
    if (!fused_rotation) {
        // The 1/valid scale cancels in the L2 normalization below; keep it only
        // on the legacy path so callers comparing raw averages still see it.
        const float inv = 1.0f / static_cast<float>(valid);
        rxopt::vector_scale_complex(scratch.corr.data(), inv, wlen);
    } else {
        // One per-sample ramp for the whole coherent average.
        scratch.cir_phase_rot.resize(wlen);
        rxopt::build_rotation_ramp(w, wlen, scratch.cir_phase_rot.data());
        rxopt::vector_mul_complex(scratch.corr.data(),
                                  scratch.cir_phase_rot.data(), wlen);
    }

    if (fused_tail) {
        // The first tail_span entries of the main ramp are bit-identical to a
        // fresh tail_span ramp (same 1024-block re-anchoring), so reuse it.
        if (fused_rotation)
            rxopt::vector_mul_complex(scratch.cir_avg_tail.data(),
                                      scratch.cir_phase_rot.data(), tail_span);
        scratch.cir_avg_tail_valid = true;
    } else if (tail_n > 0 && tail_span <= wlen) {
        // Fallback: standalone tail loop (non-integer grid, tail outside the
        // averaged range, or a tail window clipped by the buffer edge).
        scratch.cir_avg_tail.assign(tail_span, std::complex<float>(0.0f, 0.0f));
        bool tail_ok = true;
        for (size_t r = 0; r < tail_n; ++r) {
            const size_t k = tail_reps - tail_n + r;
            const int64_t lo = start - static_cast<int64_t>(pre) +
                static_cast<int64_t>(k) * tail_stride;
            if (lo < 0 || lo + static_cast<int64_t>(tail_span) >
                              static_cast<int64_t>(n)) {
                tail_ok = false;
                break;
            }
            if (fused_rotation) {
                const double phase = -w * static_cast<double>(lo);
                const std::complex<float> alpha(
                    static_cast<float>(std::cos(phase)),
                    static_cast<float>(std::sin(phase)));
                rxopt::vector_accumulate_scaled_complex(
                    scratch.cir_avg_tail.data(),
                    rx + static_cast<size_t>(lo), alpha, tail_span);
            } else {
                rxopt::vector_accumulate_complex(
                    scratch.cir_avg_tail.data(),
                    rx + static_cast<size_t>(lo), tail_span);
            }
        }
        if (tail_ok) {
            if (fused_rotation) {
                rxopt::build_rotation_ramp(w, tail_span,
                                           scratch.cir_phase_rot.data());
                rxopt::vector_mul_complex(scratch.cir_avg_tail.data(),
                                          scratch.cir_phase_rot.data(),
                                          tail_span);
            }
            scratch.cir_avg_tail_valid = true;
        }
    }

    // ---- CIR taps: sparse ternary template correlation (Task A2). ----
    std::array<std::complex<float>, 64> stack_values{};
    std::vector<std::complex<float>> heap_values;
    std::complex<float>* values = nullptr;
    if (tap_count <= stack_values.size()) {
        values = stack_values.data();
        rxopt::correlate_cir_sparse_raw(scratch.corr.data(),
                                        scratch.cir_template, tap_count,
                                        values);
    } else {
        heap_values.resize(tap_count);
        values = heap_values.data();
        rxopt::correlate_cir_sparse(scratch.corr.data(), scratch.cir_template,
                                    tap_count, heap_values);
    }
    const float nrm = std::sqrt(
        openuwb::core::sample_power_sum(values, tap_count));
    if (!(nrm > 0.0f))
        return false;
    for (size_t i = 0; i < tap_count; ++i)
        values[i] /= (nrm + 1e-12f);

    size_t peak_idx = 0;
    float peak_mag = -1.0f;
    for (size_t i = 0; i < tap_count; ++i) {
        const float m = std::abs(values[i]);
        if (m > peak_mag) {
            peak_mag = m;
            peak_idx = i;
        }
    }
    out.first_path_sample =
        static_cast<size_t>(std::max<int64_t>(0, start + (int64_t)peak_idx - (int64_t)pre));
    out.pre_samples = pre;
    out.post_samples = post;
    out.cir_peak_metric = peak_mag;
    out.cir_values.resize(tap_count);
    out.cir_filter_taps.resize(tap_count);
    for (size_t i = 0; i < tap_count; ++i) {
        out.cir_values[i] = values[i].real(); // golden CIR is real (Q=0 channel)
        out.cir_filter_taps[i] = std::conj(values[i]);
    }
    out.cir_estimate_us = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - t_cir0)
            .count());
    out.ok = true;
    return true;
}

bool stage_softchips_from_cir(const std::complex<float>* rx,
                              size_t n,
                              const Qm35825Profile& profile,
                              const TimingResult& timing,
                              CirResult& out,
                              DemodScratch& scratch,
                              size_t max_chips,
                              size_t begin_chip)
{
    const bool on_demand =
        (profile.rx_softchip_on_demand || profile.rx_fused_payload_cmf ||
         profile.rx_fused_phr_cmf) &&
        max_chips != 0;
    const bool extending = on_demand && begin_chip != 0;
    if (extending && !scratch.softchip.prepared)
        return false;
    if (on_demand && !extending)
        scratch.softchip.reset();

    const size_t pre = out.pre_samples;
    const size_t post = out.post_samples;
    const size_t tap_count = out.cir_filter_taps.size();
    const double period = timing.measured_period;
    const int64_t start = timing.preamble_start_sample;
    if (!out.ok || period <= 0.0 || start < 0 || tap_count == 0 ||
        tap_count != pre + post)
        return false;

    auto t_fir0 = std::chrono::steady_clock::now();

    // ---- Soft-chip matched filter on the chip grid. ----
    const double spc = period / static_cast<double>(kQm35ChipsPerSymbol);
    size_t n_chips = rxopt::legacy_chip_budget(profile);
    if (on_demand) {
        if (max_chips < 2)
            return false;
        if (n_chips > max_chips)
            n_chips = max_chips;
    }

    const int64_t chip_start = start + static_cast<int64_t>(tap_count) -
                               static_cast<int64_t>(pre) - 1; // = start + post - 1
    const int64_t last_by_budget =
        chip_start + static_cast<int64_t>(std::ceil((double)(n_chips - 1) * spc));
    const int64_t last_chip_sample = std::min<int64_t>(static_cast<int64_t>(n), last_by_budget);
    const int64_t last_pos = last_chip_sample - 1; // inclusive, 0-based
    if (last_pos < chip_start)
        return false;
    const int64_t spc_i = std::max<int64_t>(1, std::llround(spc));
    // Integer grid unless the clock offset would drift the grid past the
    // correlation tolerance across the longest frame (see integer_chip_grid).
    // With fused SFO tracking the on-demand prefix may run on the fractional
    // grid: only the payload needs the fractional-phase CMF, but the prefix
    // path already supports interpolation, so admit it instead of falling
    // back to the full stream.  The extension path reuses the prefix and must
    // not be rejected for starting inside the fractional stream.
    const bool fractional_grid =
        !integer_chip_grid(spc, profile.fused_gate_ppm * 1e-6);
    // Only a genuinely fractional grid needs the SFO-specific phase path.
    const bool sfo_active = profile.rx_fused_sfo_track && fractional_grid;
    const size_t num_chips =
        static_cast<size_t>(std::floor(
            static_cast<double>(last_pos - chip_start) / spc)) + 1;
    if (on_demand &&
        num_chips > static_cast<size_t>(std::numeric_limits<uint32_t>::max()))
        return false; // result chip counters are uint32_t

    if (extending && num_chips <= begin_chip) {
        // The PHR-derived range already fits in the prefix.  Nothing to do;
        // the existing normalized prefix remains the final stream.
        out.soft_chip_count = begin_chip;
        out.initial_chip_count = static_cast<uint32_t>(begin_chip);
        out.extended_chip_count = static_cast<uint32_t>(begin_chip);
        out.on_demand_extended = 0;
        out.samples_per_chip = spc;
        out.ok = true;
        return true;
    }

    // Causal FIR on the chip grid, rewritten for contiguous forward access:
    //   chip = sum_k conj(values[T-1-k]) * rx[base-k]
    //        = sum_q conj(values[q])     * rx[base-(T-1)+q]
    // Pre-conjugated taps h[q]=conj(values[q]) once outside the chip loop.
    // Kernels: multi-acc / VOLK / fixed-38 AVX2 (see uwb_cir_fir_simd.h).
    //
    // On-demand uses a separate output buffer: the prefix must survive the
    // extension call's CIR re-estimate (which temporarily uses scratch.corr).
    std::vector<std::complex<float>>& fir_out =
        on_demand ? scratch.softchip_corr : scratch.corr;
    fir_out.resize(num_chips);
    const int64_t fwd = static_cast<int64_t>(tap_count) - 1;

    // Stack buffers — no heap alloc in the FIR hot path.
    // h_conj[q] = conj(values[q]) for multi-acc / AVX2; values_nat for VOLK.
    alignas(32) std::complex<float> h_conj[64];
    alignas(32) std::complex<float> values_nat[64];
    if (tap_count > 64)
        return false;
    for (size_t q = 0; q < tap_count; ++q) {
        h_conj[q] = out.cir_filter_taps[q];
        values_nat[q] = std::conj(h_conj[q]);
    }
    size_t peak_idx = 0;
    float peak_mag = -1.0f;
    for (size_t q = 0; q < tap_count; ++q) {
        const float m = std::abs(values_nat[q]);
        if (m > peak_mag) {
            peak_mag = m;
            peak_idx = q;
        }
    }
    if (scratch.cached_seq.code_index != profile.code_index ||
        scratch.cached_seq.sampled_pcode.empty()) {
        scratch.cached_seq.update(profile.code_index, profile.sfd_mode);
    }
    const std::vector<int8_t>& spread = scratch.cached_seq.sampled_pcode;

    // Optional sparse RAKE: choose the strongest complex CIR taps once per
    // packet. Keep indices in ascending delay order for predictable loads;
    // weights retain their estimated complex phases for coherent combining.
    alignas(32) uint8_t rake_indices[64];
    alignas(32) std::complex<float> rake_weights[64];
    const bool bypass_filter =
        profile.cir_soft_chip_mode == CirSoftChipMode::Bypass;
    const bool use_rake =
        profile.cir_soft_chip_mode == CirSoftChipMode::Rake ||
        (profile.cir_soft_chip_mode == CirSoftChipMode::Auto &&
         profile.cir_rake_top_k > 0);
    const size_t rake_k = use_rake
        ? std::min(profile.cir_rake_top_k, tap_count)
        : size_t(0);
    if (rake_k > 0 && rake_k < tap_count) {
        std::array<uint8_t, 64> order{};
        for (size_t q = 0; q < tap_count; ++q)
            order[q] = static_cast<uint8_t>(q);
        std::partial_sort(order.begin(), order.begin() + rake_k,
                          order.begin() + tap_count,
                          [&](uint8_t a, uint8_t b) {
                              return std::norm(values_nat[a]) > std::norm(values_nat[b]);
                          });
        std::sort(order.begin(), order.begin() + rake_k);
        for (size_t i = 0; i < rake_k; ++i) {
            rake_indices[i] = order[i];
            rake_weights[i] = h_conj[order[i]];
        }
    }
    // The on-demand experiment is limited to the production integer-grid,
    // full-tap kernel.  The fused SFO path explicitly admits the fractional
    // grid because its payload decoder has a fractional-phase CIR bank.
    if (on_demand &&
        ((fractional_grid && !sfo_active) ||
         bypass_filter || rake_k > 0))
        return false;

    // Only NS-SFD still consumes a soft-chip stream on the fused PHR path.
    // Keep its -8-chip search halo, not a filtered copy of every SYNC.
    const bool skip_sync_fir = on_demand && !extending &&
        profile.rx_fused_phr_cmf && profile.rx_fused_payload_cmf && spc_i == 2;
    const size_t sync_end = profile.preamble_repetitions * kQm35ChipsPerSymbol;
    // Phase alignment reads the last min(32, preamble) SYNCs.  The plain
    // integer path can restore that gain from estimate_cir's tail average;
    // under SFO the tail average is smeared by the fractional drift, so the
    // SFO path instead keeps those SYNC chips in `fir_out` and reuses the
    // normal gain estimator on the fractional grid.
    const size_t phase_sync_chips =
        (profile.preamble_repetitions -
         std::min<size_t>(32, profile.preamble_repetitions)) *
        kQm35ChipsPerSymbol;
    const size_t fir_first =
        skip_sync_fir
            ? (sfo_active
                   ? std::min(sync_end > 8 ? sync_end - 8 : 0, phase_sync_chips)
                   : (sync_end > 8 ? sync_end - 8 : 0))
            : 0;
    if (fir_first >= num_chips)
        return false;
    const bool sparse_rake = rake_k > 0 && rake_k < tap_count;
    auto clipped_chip = [&](int64_t p) {
        const int64_t window_start = p - fwd;
        if (bypass_filter) {
            const int64_t idx = window_start + static_cast<int64_t>(peak_idx);
            return idx >= 0 && idx < static_cast<int64_t>(n)
                ? rx[static_cast<size_t>(idx)] : std::complex<float>{};
        }
        return cir_fir::dot_clipped(rx, n, window_start, h_conj,
                                    sparse_rake ? rake_k : tap_count,
                                    sparse_rake ? rake_indices : nullptr);
    };
    size_t checked = 0;
    if (fractional_grid) {
        // MATLAB keeps measured_period/chips_per_symbol as a fractional
        // position and linearly interpolates the CIR-filtered stream.  Keep
        // the integer positions as FIR inputs and interpolate the two outputs.
        auto dot_at = [&](int64_t p) {
            if (bypass_filter)
                return clipped_chip(p);
            if (p >= fwd && p < static_cast<int64_t>(n)) {
                const auto* window = rx + (p - fwd);
                if (sparse_rake)
                    return cir_fir::dot_topk(window, rake_indices, rake_weights, rake_k);
                return cir_fir::dot(cir_fir::kDefaultKernel, window, h_conj,
                                    values_nat, tap_count);
            }
            return clipped_chip(p);
        };
        for (size_t i = skip_sync_fir ? fir_first : 0; i < num_chips; ++i) {
            const double pd = static_cast<double>(chip_start) +
                              static_cast<double>(i) * spc;
            const int64_t p0 = static_cast<int64_t>(std::floor(pd));
            const float frac = static_cast<float>(pd - static_cast<double>(p0));
            const std::complex<float> y0 = dot_at(p0);
            const std::complex<float> y1 = dot_at(p0 + 1);
            fir_out[i] = y0 + frac * (y1 - y0);
        }
        checked = extending ? std::min(num_chips, begin_chip) : num_chips;
        // The fractional loop writes the whole requested range; when this is
        // an extension the prefix is already normalized and only the new tail
        // is consumed below.
    } else if (chip_start < fwd) {
        const int64_t gap = fwd - chip_start;
        checked = std::min(num_chips,
                           static_cast<size_t>((gap + spc_i - 1) / spc_i));
    }
    if (extending && begin_chip < checked)
        return false; // [begin_chip, checked) would never be generated
    if (extending)
        checked = std::min(num_chips, begin_chip);
    // Edge chips: FIR window may clip below sample 0 (bounds-checked scalar).
    // A prefix call handles them once; an extension starts after them.
    for (size_t i = extending ? checked : 0;
         !fractional_grid && !skip_sync_fir && i < checked; ++i) {
        const int64_t p = chip_start + static_cast<int64_t>(i) * spc_i;
        fir_out[i] = clipped_chip(p);
    }
    if (skip_sync_fir)
        checked = std::max(checked, fir_first);
    out.soft_fir_first_chip = static_cast<uint32_t>(checked);
    out.soft_fir_computed_chips = static_cast<uint32_t>(
        num_chips - (skip_sync_fir ? fir_first : (extending ? begin_chip : 0)));
    // Main body: full tap window is in-bounds.  Call the selected kernel
    // directly (no per-chip switch) so the fixed-38 AVX2 path fully inlines.
    {
        const cir_fir::Kernel fir_k = cir_fir::kDefaultKernel;
        const std::complex<float>* rx0 = rx;
        if (bypass_filter) {
            for (size_t i = checked; i < num_chips; ++i) {
                const int64_t p =
                    chip_start + static_cast<int64_t>(i) * spc_i;
                const size_t idx = static_cast<size_t>(
                    p - fwd + static_cast<int64_t>(peak_idx));
                fir_out[i] = rx0[idx];
            }
        } else if (rake_k > 0 && rake_k < tap_count) {
            size_t i = checked;
#if UWB_CIR_FIR_HAVE_AVX2
            // Fixed Top-4/Top-8 fast path: vectorize across four outputs. The
            // load helper reads through max_q+7; keep the final boundary on
            // the scalar path when that speculative contiguous load is unsafe.
            if ((rake_k == 4 || rake_k == 8) && spc_i == 2) {
                const size_t max_q = rake_indices[rake_k - 1];
                for (; i + 4 <= num_chips; i += 4) {
                    const int64_t p =
                        chip_start + static_cast<int64_t>(i) * spc_i;
                    const size_t win = static_cast<size_t>(p - fwd);
                    if (win + max_q + 7 >= n)
                        break;
                    if (rake_k == 4)
                        cir_fir::dot_topk_x4_avx2<4>(
                            rx0 + win, rake_indices, rake_weights,
                            fir_out.data() + i);
                    else
                        cir_fir::dot_topk_x4_avx2<8>(
                            rx0 + win, rake_indices, rake_weights,
                            fir_out.data() + i);
                }
            }
#endif
            for (; i < num_chips; ++i) {
                const int64_t p = chip_start + static_cast<int64_t>(i) * spc_i;
                const size_t win = static_cast<size_t>(p - fwd);
                fir_out[i] = cir_fir::dot_topk(
                    rx0 + win, rake_indices, rake_weights, rake_k);
            }
        } else if (fir_k == cir_fir::Kernel::Avx2Fixed &&
            tap_count == cir_fir::kDefaultTapCount) {
            size_t i = checked;
            // Four adjacent outputs share one h_conj traversal on the integer
            // grid (spc_i == 2).  The batch is only taken while the LAST
            // window (+6) stays inside the buffer; the remaining <4 chips use
            // the single-output kernel.
            if (spc_i == 2) {
                for (; i + 4 <= num_chips; i += 4) {
                    const int64_t p =
                        chip_start + static_cast<int64_t>(i) * spc_i;
                    const size_t win = static_cast<size_t>(p - fwd);
                    if (win + cir_fir::kDefaultTapCount + 6 > n)
                        break;
                    cir_fir::dot_avx2_fixed38_x4(rx0 + win, h_conj,
                                                 fir_out.data() + i);
                }
            }
            for (; i < num_chips; ++i) {
                const int64_t p = chip_start + static_cast<int64_t>(i) * spc_i;
                const size_t win = static_cast<size_t>(p - fwd);
                fir_out[i] =
                    cir_fir::dot_avx2_fixed38(rx0 + win, h_conj, tap_count);
            }
        } else if (fir_k == cir_fir::Kernel::Volk) {
            for (size_t i = checked; i < num_chips; ++i) {
                const int64_t p = chip_start + static_cast<int64_t>(i) * spc_i;
                const size_t win = static_cast<size_t>(p - fwd);
                fir_out[i] =
                    cir_fir::dot_volk(rx0 + win, values_nat, tap_count);
            }
        } else {
            for (size_t i = checked; i < num_chips; ++i) {
                const int64_t p = chip_start + static_cast<int64_t>(i) * spc_i;
                const size_t win = static_cast<size_t>(p - fwd);
                fir_out[i] =
                    cir_fir::dot_multi_acc8(rx0 + win, h_conj, tap_count);
            }
        }
    }
    out.soft_fir_us = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - t_fir0)
            .count());
    auto t_post0 = std::chrono::steady_clock::now();

    // ---- Phase-align against the last min(32, preamble) SYNCs' spread code. ----
    const size_t phase_reps = std::min<size_t>(32, profile.preamble_repetitions);
    const size_t phase_first =
        (profile.preamble_repetitions - phase_reps) * kQm35ChipsPerSymbol;
    const size_t phase_len = phase_reps * kQm35ChipsPerSymbol;
    if (phase_first + phase_len > num_chips)
        return false; // chip stream shorter than the configured preamble
    // spread[] is real ±1/0, so conj(spread)*corr = spread*corr.  Avoid % by
    // walking the 508-chip period with a running index.
    std::complex<float> rot(1.0f, 0.0f);
    if (extending) {
        // Frozen by the prefix pass: phase alignment runs once per packet.
        rot = scratch.softchip.rot;
    } else if (skip_sync_fir && !sfo_active) {
        const auto phase_begin = std::chrono::steady_clock::now();
        std::complex<float> gain{};
        bool gain_ok = false;
        // The fused CIR estimator already produced the coherent tail average
        // (last 32 SYNCs, CFO-compensated) from the raw frame.  Contract it
        // with the CIR taps instead of re-reading 32 x 1052 derotated samples.
        if (scratch.cir_avg_tail_valid &&
            scratch.cir_avg_tail.size() >= 2 * (spread.size() - 1) + tap_count) {
            gain_ok = rxopt::sync_phase_gain_from_tail(
                scratch.cir_avg_tail.data(), h_conj, tap_count, spread, gain);
        }
        if (!gain_ok) {
            gain_ok = rxopt::sync_phase_gain(
                rx, n,
                chip_start - fwd + static_cast<int64_t>(2 * phase_first),
                h_conj, tap_count, spread, phase_reps, scratch.work, gain);
        }
        if (!gain_ok)
            return false;
        const float ang = std::arg(gain);
        rot = std::complex<float>(std::cos(-ang), std::sin(-ang));
        scratch.softchip.rot = rot;
        out.sync_phase_us = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::steady_clock::now() - phase_begin).count());
    } else {
        std::complex<float> gain(0.0f, 0.0f);
        size_t s = 0;
        for (size_t i = 0; i < phase_len; ++i) {
            gain += static_cast<float>(spread[s]) *
                    fir_out[phase_first + i];
            if (++s == kQm35ChipsPerSymbol)
                s = 0;
        }
        const float ang = std::arg(gain);
        rot = std::complex<float>(std::cos(-ang), std::sin(-ang));
        if (on_demand)
            scratch.softchip.rot = rot;
    }

    // Merged postprocess: compute only the rotated real component and track
    // max|real|, then normalize into soft_chips. No later stage consumes the
    // rotated complex corr stream, so writing it back wastes ~1.2 MB/packet.
    scratch.soft_chips.resize(num_chips);
    if (!extending) {
        // Preserve absolute chip coordinates; the skipped prefix is not data.
        // NS-SFD uses normalized correlation, invariant to this common scale.
        const size_t post_first = skip_sync_fir ? fir_first : 0;
        std::fill(scratch.soft_chips.begin(),
                  scratch.soft_chips.begin() + post_first, 0.0f);
        const float mx = rxopt::project_soft_chips(
            fir_out.data() + post_first, scratch.soft_chips.data() + post_first,
            num_chips - post_first, rot);
        const float inv_denom = 1.0f / (mx + 1e-12f);
        for (size_t i = post_first; i < num_chips; ++i)
            scratch.soft_chips[i] *= inv_denom;
        if (on_demand) {
            scratch.softchip.prefix_max = mx;
            scratch.softchip.generated = num_chips;
            scratch.softchip.prepared = true;
            out.initial_chip_count = static_cast<uint32_t>(num_chips);
            out.extended_chip_count = 0;
            out.on_demand_extended = 0;
        }
    } else {
        // Extension: rotate only the new chips, then put the whole stream on
        // one denominator.  rescale_prefix() multiplies the already-normalized
        // prefix by (prefix_max+eps)/(final_max+eps); the new tail is scaled by
        // 1/(final_max+eps).  This equals the one-shot raw/final_max result up
        // to one float rounding on the prefix.
        const float ext_mx = rxopt::project_soft_chips(
            fir_out.data() + begin_chip, scratch.soft_chips.data() + begin_chip,
            num_chips - begin_chip, rot);
        const float final_max = std::max(scratch.softchip.prefix_max, ext_mx);
        rxopt::rescale_prefix(scratch.soft_chips, begin_chip,
                              scratch.softchip.prefix_max, final_max);
        const float inv_denom = 1.0f / (final_max + 1e-12f);
        for (size_t i = begin_chip; i < num_chips; ++i)
            scratch.soft_chips[i] *= inv_denom;
        scratch.softchip.prefix_max = final_max;
        scratch.softchip.generated = num_chips;
        out.initial_chip_count = static_cast<uint32_t>(begin_chip);
        out.extended_chip_count = static_cast<uint32_t>(num_chips);
        out.on_demand_extended = 1;
    }
    out.postprocess_us = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - t_post0)
            .count());

    out.soft_chip_count = num_chips;
    out.samples_per_chip = spc;
    out.ok = true;
    return true;
}

bool stage_cir_softchips(const std::complex<float>* rx,
                         size_t n,
                         const Qm35825Profile& profile,
                         const TimingResult& timing,
                         const std::vector<int8_t>& preamble_code,
                         CirResult& out,
                         DemodScratch& scratch,
                         size_t max_chips,
                         size_t begin_chip)
{
    if (!estimate_cir(rx, n, profile, timing, preamble_code, out, scratch))
        return false;
    return stage_softchips_from_cir(rx, n, profile, timing, out, scratch,
                                    max_chips, begin_chip);
}

} // namespace openuwb::demod::core

// ---- demod_phr.h ----

namespace openuwb::demod::core {
bool stage_ns_sfd(const std::vector<float>& soft_chips,
                  const Qm35825Profile& profile,
                  const std::vector<int8_t>& sfd_sequence,
                  size_t chips_per_symbol,
                  NsSfdResult& out,
                  DemodScratch& scratch)
{
    out = NsSfdResult{};
    if (soft_chips.empty() || sfd_sequence.empty() || chips_per_symbol == 0)
        return false;

    // spread = kron(sfd_sequence, spread_code); both real ternary -> real.
    const char* cur_mode = profile.sfd_mode ? profile.sfd_mode : "";
    if (scratch.cached_seq.code_index != profile.code_index ||
        scratch.cached_seq.sfd_mode != cur_mode ||
        scratch.cached_seq.sampled_pcode.empty()) {
        scratch.cached_seq.update(profile.code_index, profile.sfd_mode);
    }
    if (scratch.cached_seq.sfd_seq != sfd_sequence ||
        scratch.cached_seq.ns_sfd_sparse.empty()) {
        scratch.cached_seq.sfd_seq = sfd_sequence;
        scratch.cached_seq.ns_sfd_sparse.clear();
        const std::vector<int8_t>& sc = scratch.cached_seq.sampled_pcode;
        scratch.cached_seq.ns_sfd_total_chips = sfd_sequence.size() * sc.size();
        for (size_t i = 0; i < sfd_sequence.size(); ++i) {
            const int8_t si = sfd_sequence[i];
            if (si == 0) continue;
            for (size_t j = 0; j < sc.size(); ++j) {
                const int8_t pj = sc[j];
                if (pj == 0) continue;
                const int8_t val = static_cast<int8_t>(si * pj);
                scratch.cached_seq.ns_sfd_sparse.push_back({ static_cast<uint32_t>(i * sc.size() + j), val });
            }
        }
        scratch.cached_seq.ns_sfd_norm = std::sqrt(static_cast<float>(scratch.cached_seq.ns_sfd_sparse.size()));
    }

    const size_t sfd_len = sfd_sequence.size() * chips_per_symbol;
    const int64_t expected =
        static_cast<int64_t>(profile.preamble_repetitions) *
        static_cast<int64_t>(chips_per_symbol);
    const int64_t half = 8;
    const int64_t lo = std::max<int64_t>(0, expected - half);
    const int64_t hi = std::min<int64_t>(
        static_cast<int64_t>(soft_chips.size()) - static_cast<int64_t>(sfd_len),
        expected + half);
    if (hi < lo)
        return false;

    float best = -1.0f;
    int64_t best_k = lo;
    int pol = 1;

    if (chips_per_symbol == scratch.cached_seq.sampled_pcode.size() &&
        scratch.cached_seq.ns_sfd_total_chips == sfd_len &&
        !scratch.cached_seq.ns_sfd_sparse.empty()) {
        const float spread_norm = scratch.cached_seq.ns_sfd_norm;
        double win_energy = 0.0;
        for (size_t i = 0; i < sfd_len; ++i) {
            const double v = static_cast<double>(
                soft_chips[static_cast<size_t>(lo + static_cast<int64_t>(i))]);
            win_energy += v * v;
        }
        if (!std::isfinite(win_energy) || win_energy < 0.0)
            return false;

        for (int64_t k = lo; k <= hi; ++k) {
            if (k > lo) {
                const double v_in = static_cast<double>(
                    soft_chips[static_cast<size_t>(k + static_cast<int64_t>(sfd_len) - 1)]);
                const double v_out = static_cast<double>(
                    soft_chips[static_cast<size_t>(k - 1)]);
                win_energy += v_in * v_in - v_out * v_out;
                if (!std::isfinite(win_energy))
                    return false;
            }
            const float seg_norm = std::sqrt(std::max(0.0, win_energy));
            float corr = 0.0f;
            for (const auto& entry : scratch.cached_seq.ns_sfd_sparse) {
                const float v = soft_chips[static_cast<size_t>(k + static_cast<int64_t>(entry.offset))];
                // The sparse sign pattern is fixed across the 17 lag trials, so
                // the branch predictor learns it; add/sub is cheaper than the
                // branchless `sign * v` multiply.
                if (entry.sign > 0)
                    corr += v;
                else
                    corr -= v;
            }
            const float m = std::abs(corr) / (spread_norm * seg_norm + 1e-12f);
            if (m > best) {
                best = m;
                best_k = k;
                pol = (corr >= 0.0f) ? 1 : -1;
            }
        }
    } else {
        const std::vector<int8_t>& spread_code = scratch.cached_seq.sampled_pcode;
        scratch.work.assign(sfd_len, std::complex<float>(0.0f, 0.0f));
        for (size_t i = 0; i < sfd_sequence.size(); ++i)
            for (size_t j = 0; j < spread_code.size(); ++j)
                scratch.work[i * spread_code.size() + j] = std::complex<float>(
                    static_cast<float>(sfd_sequence[i] * spread_code[j]), 0.0f);
        float spread_norm = 0.0f;
        for (size_t i = 0; i < sfd_len; ++i)
            spread_norm += std::norm(scratch.work[i]);
        spread_norm = std::sqrt(spread_norm);

        for (int64_t k = lo; k <= hi; ++k) {
            float seg_norm = 0.0f, corr = 0.0f;
            for (size_t i = 0; i < sfd_len; ++i) {
                const float v = soft_chips[static_cast<size_t>(k + (int64_t)i)];
                seg_norm += v * v;
                corr += scratch.work[i].real() * v;
            }
            seg_norm = std::sqrt(seg_norm);
            const float m = std::abs(corr) / (spread_norm * seg_norm + 1e-12f);
            if (m > best) {
                best = m;
                best_k = k;
                pol = (corr >= 0.0f) ? 1 : -1;
            }
        }
    }

    if (best < profile.sfd_detection_threshold)
        return false;

    out.ok = true;
    out.sfd_start_chip = best_k;
    out.sfd_end_chip = best_k + static_cast<int64_t>(sfd_len) - 1;
    out.polarity = pol;
    out.metric = best;
    return true;
}

} // namespace openuwb::demod::core

namespace openuwb::demod::core::detail {
bool phr_secded_parse(const std::vector<int8_t>& decoded,
                      const std::vector<float>& soft_chips,
                      const NsSfdResult& ns_sfd,
                      PhrResult& out)
{
    out = PhrResult{};
    if (decoded.size() < 19)
        return false;

    std::vector<int8_t> sys13(decoded.begin(), decoded.begin() + 13);
    std::vector<int8_t> recv_par(decoded.begin() + 13, decoded.begin() + 19);
    std::vector<int8_t> phr19;
    detail::hrp_secded(sys13, phr19);
    int8_t synd[6];
    for (int i = 0; i < 6; ++i)
        synd[i] = static_cast<int8_t>(recv_par[i] ^ phr19[13 + i]);

    bool secded_pass = true;
    int idx = 0;
    for (int i = 1; i < 6; ++i)
        idx |= (synd[i] << (i - 1));
    if (idx > 0) {
        static const int kAddresses[13] = { 3, 5, 6, 7, 9, 10, 11,
                                            12, 13, 14, 15, 17, 18 };
        for (int j = 0; j < 13; ++j) {
            if (kAddresses[j] == idx) {
                phr19[j] ^= 1;
                out.secded_corrected = true;
                break;
            }
        }
        if (!synd[0])
            secded_pass = false; // DED: second error cannot be corrected
    }
    out.secded_uncorrectable = !secded_pass;

    // MATLAB bit2int default is MSB-first (bit2int(x,n)); phr(1:2) -> data
    // rate, phr(3:9) -> PSDU length.
    //
    // Guard against SECDED mis-correction on real RF.  A false single-bit
    // flip can inflate PSDU length (e.g. 13 -> 45 on DW1000 captures) past
    // the RF energy while the soft-chip *buffer* is still long enough.  If
    // the corrected length needs a long tail whose soft-chip energy is near
    // the noise floor relative to the uncorrected body, keep uncorrected.
    auto length_of = [](const std::vector<int8_t>& b) -> uint32_t {
        uint32_t psdu = 0;
        for (int i = 0; i < 7; ++i)
            psdu |= (static_cast<uint32_t>(b[2 + i]) << (6 - i));
        return psdu;
    };
    const uint32_t len_uncorr = length_of(sys13); // sys13 is never flipped
    const uint32_t len_corr = length_of(phr19);
    if (out.secded_corrected && len_corr > len_uncorr &&
        ns_sfd.sfd_end_chip >= 0) {
        const int64_t phr_start = ns_sfd.sfd_end_chip + 1;
        const size_t need_c = rxopt::payload_chips_needed(len_corr);
        const size_t need_u = rxopt::payload_chips_needed(len_uncorr);
        if (phr_start >= 0 && need_c > need_u + 64) {
            const size_t base = static_cast<size_t>(phr_start);
            const size_t end_u = base + need_u;
            const size_t end_c = base + need_c;
            if (end_c <= soft_chips.size() && end_u < soft_chips.size()) {
                double e_body = 0.0, e_tail = 0.0;
                for (size_t i = base; i < end_u; ++i) {
                    const double v = soft_chips[i];
                    e_body += v * v;
                }
                for (size_t i = end_u; i < end_c; ++i) {
                    const double v = soft_chips[i];
                    e_tail += v * v;
                }
                const double n_body = static_cast<double>(end_u - base);
                const double n_tail = static_cast<double>(end_c - end_u);
                const double mean_body = e_body / (n_body + 1e-12);
                const double mean_tail = e_tail / (n_tail + 1e-12);
                // Tail is noise-like if its mean energy is << body.
                if (mean_tail < 0.15 * mean_body) {
                    for (int i = 0; i < 13; ++i)
                        phr19[i] = sys13[i];
                    out.secded_corrected = false;
                }
            }
        }
    }

    const int dr = (phr19[0] << 1) | phr19[1];
    static const float kDataRates[4] = { 0.11f, 0.85f, 6.81f, 27.24f };
    out.data_rate_mbps = kDataRates[dr & 3];
    out.psdu_length = length_of(phr19);
    out.phr_bits.reserve(19);
    for (int i = 0; i < 19; ++i)
        out.phr_bits.push_back(static_cast<uint8_t>(phr19[i]));
    out.ok = true;
    return true;
}

} // namespace openuwb::demod::core::detail

namespace openuwb::demod::core {
bool decode_phr_decisions(const std::vector<int8_t>& g0,
                          const std::vector<int8_t>& g1,
                          const std::vector<float>& soft_chips,
                          const NsSfdResult& ns_sfd,
                          PhrResult& out)
{
    out = PhrResult{};
    const size_t nsym = 21;
    if (g0.size() != nsym || g1.size() != nsym)
        return false;

    std::vector<int8_t> rx(2 * nsym);
    for (size_t s = 0; s < nsym; ++s) {
        rx[2 * s] = g0[s];
        rx[2 * s + 1] = g1[s];
    }
    std::vector<int8_t> decoded;
    detail::vitdec_cl3(rx.data(), nsym, decoded);
    return detail::phr_secded_parse(decoded, soft_chips, ns_sfd, out);
}

bool decode_phr_decisions_soft(const std::vector<float>& llr0,
                               const std::vector<float>& llr1,
                               const std::vector<float>& soft_chips,
                               const NsSfdResult& ns_sfd,
                               PhrResult& out)
{
    out = PhrResult{};
    const size_t nsym = 21;
    if (llr0.size() != nsym || llr1.size() != nsym)
        return false;

    std::vector<int8_t> decoded;
    detail::vitdec_cl3_soft(llr0.data(), llr1.data(), nsym, decoded);
    return detail::phr_secded_parse(decoded, soft_chips, ns_sfd, out);
}

bool stage_phr(const std::vector<float>& soft_chips,
               const Qm35825Profile& profile,
               const NsSfdResult& ns_sfd,
               size_t chips_per_symbol,
               PhrResult& out,
               DemodScratch& scratch)
{
    out = PhrResult{};
    if (soft_chips.empty() || !ns_sfd.ok || ns_sfd.sfd_end_chip < 0)
        return false;
    const size_t cpb = 64, cps = 512, nsym = 21;
    const int64_t phr_start = ns_sfd.sfd_end_chip + 1;
    if (phr_start < 0 ||
        static_cast<size_t>(phr_start + static_cast<int64_t>(cps * nsym)) >
            soft_chips.size())
        return false;
    const int8_t pol = (ns_sfd.polarity < 0) ? -1 : 1;

    // Soft-decision PHR: bprf_demod_soft() produces the same hard decisions as
    // bprf_demod() plus the per-symbol LLRs, and the soft CL-3 Viterbi is
    // strictly better informed than the hard one.  The SECDED/parse tail is
    // shared, so the PHR bit/length/rate contract is unchanged.
    std::vector<int8_t> g0, g1;
    std::vector<float> llr0, llr1;
    if (!detail::bprf_demod_soft(soft_chips, phr_start, profile.code_index,
                                 nsym, cpb, cps, 0, pol, g0, g1, llr0, llr1))
        return false;
    return decode_phr_decisions_soft(llr0, llr1, soft_chips, ns_sfd, out);
}

bool prepare_fused_phr_pn(DemodScratch& scratch, size_t code_index)
{
    const size_t nbits =
        rxfusion::kPhrSymbols * rxfusion::kPhrChipsPerBurst;
    if (scratch.fused_phr.code_index == code_index &&
        scratch.fused_phr.pn.size() == nbits)
        return true;
    if (!detail::bprf_spreading(scratch.fused_phr.pn, code_index,
                                rxfusion::kPhrScramOffset, nbits)) {
        scratch.fused_phr.code_index = 0;
        scratch.fused_phr.pn.clear();
        scratch.fused_phr.reset_decisions();
        return false;
    }
    scratch.fused_phr.code_index = code_index;
    scratch.fused_phr.reset_decisions();
    return true;
}

bool stage_phr_fused_cmf(const std::complex<float>* rx,
                         size_t n,
                         const Qm35825Profile& profile,
                         const TimingResult& timing,
                         const CirResult& cir,
                         const std::vector<float>& prefix_soft_chips,
                         const NsSfdResult& ns_sfd,
                         PhrResult& out,
                         DemodScratch& scratch,
                         uint64_t& template_us,
                         uint64_t& correlate_us,
                         uint64_t& decode_us,
                         uint64_t& decode_ns,
                         size_t& symbols_out)
{
    out = PhrResult{};
    template_us = 0;
    correlate_us = 0;
    decode_us = 0;
    decode_ns = 0;
    symbols_out = 0;
    if (!rx || !ns_sfd.ok || ns_sfd.sfd_end_chip < 0)
        return false;
    const int64_t phr_start = ns_sfd.sfd_end_chip + 1;
    if (phr_start < 0 ||
        static_cast<size_t>(phr_start) > prefix_soft_chips.size())
        return false;

    const size_t tap_count = cir.cir_filter_taps.size();
    if (tap_count == 0 || tap_count > rxfusion::kMaxTapCount)
        return false;
    if (tap_count != profile.cir_pre_samples + profile.cir_post_samples)
        return false;
    if (!profile.rx_fused_sfo_track &&
        !integer_chip_grid_at(cir.samples_per_chip,
                              rxfusion::kSamplesPerChip,
                              profile.fused_gate_ppm * 1e-6))
        return false;
    if (profile.cir_pre_samples > tap_count)
        return false;
    if (!scratch.softchip.prepared || !std::isfinite(scratch.softchip.rot.real()) ||
        !std::isfinite(scratch.softchip.rot.imag()))
        return false;

    const int64_t chip_start =
        timing.preamble_start_sample + static_cast<int64_t>(tap_count) -
        static_cast<int64_t>(profile.cir_pre_samples) - 1;
    // Under SFO the chip grid accumulates (spc - 2) samples per chip, so the
    // PHR window origin computed on the nominal 2-sample grid is off by
    // phr_start*(spc-2) samples.  Shift the integer origin by the rounded
    // drift; the residual (< 0.5 sample) is well inside the 64-chip PHR burst
    // tolerance.  The payload uses the exact fractional bank instead.
    int64_t phr_chip_start = chip_start;
    if (profile.rx_fused_sfo_track && cir.samples_per_chip > 0.0) {
        const double drift =
            static_cast<double>(phr_start) * (cir.samples_per_chip - 2.0);
        phr_chip_start += static_cast<int64_t>(std::llround(drift));
    }
    if (chip_start < 0 || static_cast<int64_t>(n) <= chip_start)
        return false;

    const auto t_plan0 = std::chrono::steady_clock::now();
    if (!prepare_fused_phr_pn(scratch, profile.code_index))
        return false;
    const auto t_plan1 = std::chrono::steady_clock::now();
    const uint64_t plan_us = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(t_plan1 - t_plan0)
            .count());

    const int8_t pol = (ns_sfd.polarity < 0) ? -1 : 1;
    const double spc_phr =
        (cir.samples_per_chip > 0.0) ? cir.samples_per_chip
                                     : static_cast<double>(rxfusion::kSamplesPerChip);
    rxfusion::Timings timings;
    rxfusion::Metrics2& pm = scratch.fused_phr.metrics2;
    if (profile.rx_bpm_known_hop) {
        // Known-hop 2-candidate PHR: the hop comes from pn[64*s], so only the
        // two BPM halves are correlated.  z0/z1 are raw complex values on the
        // same phase basis as Metrics::complex_value, ready for the pilot
        // refinement below.
        if (!rxfusion::correlate_phr_known_hop(
                scratch.derotated.data(), n, cir.cir_filter_taps.data(),
                tap_count, phr_chip_start, rxfusion::kSamplesPerChip, phr_start,
                scratch.fused_phr.pn.data(), pol, scratch.softchip.rot, pm,
                scratch.fused_phr.kernel, &timings))
            return false;
        scratch.fused_phr.metrics2_valid = true;
    } else {
        if (!rxfusion::correlate_phr(
                scratch.derotated.data(), n, cir.cir_filter_taps.data(),
                tap_count, phr_chip_start, rxfusion::kSamplesPerChip, phr_start,
                scratch.fused_phr.pn.data(), pol, scratch.softchip.rot,
                scratch.fused_phr.metrics, scratch.fused_phr.kernel,
                &timings))
            return false;
        for (float v : scratch.fused_phr.metrics.signed_value) {
            if (!std::isfinite(v))
                return false;
        }
        // Project the four candidates onto the two known-hop metrics by
        // picking the hop with the larger half energy.  The 21 raw complex
        // values are kept for the pilot refinement.
        pm.reset(rxfusion::kPhrSymbols, true);
        const auto& sv = scratch.fused_phr.metrics.signed_value;
        const auto& cv = scratch.fused_phr.metrics.complex_value;
        for (size_t s = 0; s < rxfusion::kPhrSymbols; ++s) {
            const float h0 = std::max(std::abs(sv[4 * s + 0]),
                                      std::abs(sv[4 * s + 1]));
            const float h1 = std::max(std::abs(sv[4 * s + 2]),
                                      std::abs(sv[4 * s + 3]));
            const size_t h = (h1 > h0) ? 1 : 0;
            pm.m0[s] = sv[4 * s + 2 * h + 0];
            pm.m1[s] = sv[4 * s + 2 * h + 1];
            pm.z0[s] = cv[4 * s + 2 * h + 0];
            pm.z1[s] = cv[4 * s + 2 * h + 1];
        }
        scratch.fused_phr.metrics2_valid = true;
    }
    template_us = plan_us + timings.template_update_us;
    correlate_us = timings.correlate_us;

    // Decisions and LLRs from the two known-hop metrics.
    auto& pg0 = scratch.fused_phr.g0;
    auto& pg1 = scratch.fused_phr.g1;
    auto& pll0 = scratch.fused_phr.llr0;
    auto& pll1 = scratch.fused_phr.llr1;
    pg0.resize(rxfusion::kPhrSymbols);
    pg1.resize(rxfusion::kPhrSymbols);
    pll0.resize(rxfusion::kPhrSymbols);
    pll1.resize(rxfusion::kPhrSymbols);
    for (size_t s = 0; s < rxfusion::kPhrSymbols; ++s) {
        const float a = pm.m0[s];
        const float b = pm.m1[s];
        pg0[s] = (std::abs(a) >= std::abs(b)) ? 0 : 1;
        const float best = (pg0[s] == 0) ? a : b;
        pg1[s] = (best < 0.0f) ? 1 : 0;
        pll0[s] = std::abs(a) - std::abs(b);
        pll1[s] = std::max(a, b) - std::max(-a, -b);
    }

    const auto t_decode0 = std::chrono::steady_clock::now();
    const bool decoded =
        profile.rx_bpm_joint_ml
            ? detail::decode_phr_biorth_soft(
                  pm.m0.data(), pm.m1.data(), prefix_soft_chips, ns_sfd, out)
            : decode_phr_decisions_soft(pll0, pll1, prefix_soft_chips, ns_sfd,
                                        out);

    // PHR pilot refinement: only after a valid PHR decode, and only when the
    // 21 winning complex pilot values are available.  A failed coherence gate
    // leaves `applied=false` and the payload keeps the original rotation.
    if (decoded && profile.rx_phr_phase_refine &&
        scratch.fused_phr.metrics2_valid) {
        auto& z0 = pm.z0;
        auto& z1 = pm.z1;
        for (size_t s = 0; s < rxfusion::kPhrSymbols; ++s) {
            const std::complex<float> win =
                (pg0[s] == 0) ? z0[s] : z1[s];
            z0[s] = win; // alias the winning half into the pilot slot
        }
        const double phr_symbol_samples =
            static_cast<double>(rxfusion::kPhrChipsPerSymbol) * spc_phr;
        // Last PHR symbol window start -> payload s=0 window start is one PHR
        // symbol; the refine helper adds the centre-symbol offset itself.
        scratch.phr_refine = rxopt::refine_phase_cfo_from_phr(
            z0.data(), rxfusion::kPhrSymbols, pg1.data(), pol,
            scratch.softchip.rot, phr_symbol_samples, phr_symbol_samples);
        // Reliability gate: the lag-1 phase-difference slope is extrapolated
        // over ~11 PHR symbols, so a noisy estimate hurts marginal packets.
        // Only accept the correction when the polarity-stripped adjacent
        // products are nearly collinear, i.e. the pilot really follows one
        // phase ramp.  UWB_BPM_REFINE_PROD overrides the 0.997 default.
        double prod_re = 0.0, prod_abs = 0.0;
        for (size_t k = 0; k + 1 < rxfusion::kPhrSymbols; ++k) {
            const float s0 = static_cast<float>(
                (1 - 2 * static_cast<int>(pg1[k])) *
                static_cast<int>(pol));
            const float s1 = static_cast<float>(
                (1 - 2 * static_cast<int>(pg1[k + 1])) *
                static_cast<int>(pol));
            const std::complex<float> p = (s1 * z0[k + 1]) *
                                          std::conj(s0 * z0[k]);
            prod_re += p.real();
            prod_abs += std::abs(p);
        }
        const double prod = (prod_abs > 0.0) ? prod_re / prod_abs : 0.0;
        double prod_gate = 0.997;
        if (const char* g = std::getenv("UWB_BPM_REFINE_PROD"))
            prod_gate = std::atof(g);
        if (prod < prod_gate) {
            scratch.phr_refine.applied = false;
            scratch.phr_refine.rot_at_payload_start = scratch.softchip.rot;
            scratch.phr_refine.delta_cfo_w_rad = 0.0;
        }
        if (scratch.phr_refine.applied) {
            // Re-express the 21 PHR metrics in the refined payload phase so
            // the joint ML Viterbi sees one consistent frame.
            const std::complex<float> corr =
                std::conj(scratch.softchip.rot) *
                scratch.phr_refine.rot_at_payload_start;
            for (size_t s = 0; s < rxfusion::kPhrSymbols; ++s) {
                const std::complex<float> y0 =
                    scratch.softchip.rot * pm.z0[s];
                const std::complex<float> y1 =
                    scratch.softchip.rot * pm.z1[s];
                const float r0 = (corr * y0).real();
                const float r1 = (corr * y1).real();
                pm.m0[s] = (pol < 0) ? -r0 : r0;
                pm.m1[s] = (pol < 0) ? -r1 : r1;
            }
        }
    }

    // Only publish the cache once the decode succeeded: a failed PHR must not
    // leave decisions/LLRs that a later stage could mistake for this packet's.
    scratch.fused_phr.decisions_valid = decoded;
    decode_ns = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now() - t_decode0)
            .count());
    decode_us = decode_ns / 1000;
    symbols_out = rxfusion::kPhrSymbols;
    if (!decoded) {
        scratch.fused_phr.reset_decisions();
        return false;
    }
    return true;
}

} // namespace openuwb::demod::core

// ---- demod_payload.h ----

namespace openuwb::demod::core {
bool stage_payload_fcs(const std::vector<float>& soft_chips,
                       const Qm35825Profile& profile,
                       const PhrResult& phr,
                       const NsSfdResult& ns_sfd,
                       size_t chips_per_symbol,
                       PayloadResult& out,
                       DemodScratch& scratch,
                       PayloadSubTimers* sub)
{
    // Diagnostic child timing of `stage_payload_us`.  `sub_guard` attributes the
    // remainder of this function to `other_us`; the named buckets are filled
    // around the exact calls that produce them.  Passing `sub == nullptr`
    // (e.g. the full-stream fallback retry, which is accounted in its own
    // bucket) disables all of it.
    detail::PayloadSubTimer sub_guard(sub);
    if (sub)
        ++sub->legacy_calls;
    out = PayloadResult{};
    if (soft_chips.empty() || !phr.ok || !ns_sfd.ok || phr.psdu_length == 0)
        return false;
    const int64_t phr_start = ns_sfd.sfd_end_chip + 1;
    if (phr_start < 0)
        return false;
    // The payload field begins right after the 21 PHR symbols (512 chips each).
    const int64_t payload_start = phr_start + static_cast<int64_t>(512 * 21);

    bool any_ok = false;

    // First run the MATLAB-equivalent production path: reported PHR length,
    // NS-SFD polarity, hard BPM decisions and hard truncated Viterbi.  The
    // RF experiments below (length neighborhood, opposite polarity and soft
    // Viterbi) are fallbacks; they must not hide a valid fixed-length decode
    // or replace it with a shorter accidental RS codeword.
    {
        const int8_t pol = (ns_sfd.polarity < 0) ? -1 : 1;
        const size_t psdu_bits = static_cast<size_t>(phr.psdu_length) * 8;
        const size_t num_blocks = (psdu_bits + 329) / 330;
        const size_t nsym = psdu_bits + 48 * num_blocks;
        std::vector<int8_t> g0p, g1p, g0, g1;
        const auto t_soft = std::chrono::steady_clock::now();
        const bool demod_ok =
            detail::bprf_demod(soft_chips, phr_start, profile.code_index, 21,
                               64, 512, 0, pol, g0p, g1p) &&
            detail::bprf_demod(soft_chips, payload_start, profile.code_index,
                               nsym, 8, 64, 1344, pol, g0, g1);
        if (sub)
            sub->soft_us += detail::payload_elapsed_us(t_soft);
        if (demod_ok) {
            if (detail::decode_payload_hard(g0p, g1p, g0, g1, psdu_bits, out,
                                            sub)) {
                any_ok = true;
                if (out.fcs_pass)
                    return true;
            }
        }
    }

    // Candidate PSDU lengths: reported PHR length, then a small neighborhood
    // (±2) that covers common SECDED single-bit length slips on real RF.
    uint32_t lens[5];
    size_t n_lens = 0;
    auto add_len = [&](uint32_t L) {
        if (L == 0 || L > profile.max_psdu_bytes)
            return;
        for (size_t i = 0; i < n_lens; ++i)
            if (lens[i] == L)
                return;
        if (n_lens < 5)
            lens[n_lens++] = L;
    };
    add_len(phr.psdu_length);
    if (phr.psdu_length > 2)
        add_len(phr.psdu_length - 1);
    add_len(phr.psdu_length + 1);
    if (phr.psdu_length > 3)
        add_len(phr.psdu_length - 2);
    add_len(phr.psdu_length + 2);

    // Try reported SFD polarity first, then the opposite.  On real RF the
    // NS-SFD sign can flip relative to the payload field even when the
    // magnitude metric is high; one polarity yields RS+FCS, the other fails.
    const int8_t pol0 = (ns_sfd.polarity < 0) ? -1 : 1;
    const int8_t pols[2] = { pol0, static_cast<int8_t>(-pol0) };
    for (size_t li = 0; li < n_lens; ++li) {
        const uint32_t L = lens[li];
        const size_t psdu_bits = static_cast<size_t>(L) * 8;
        const size_t num_blocks = (psdu_bits + 329) / 330;
        const size_t nsym = psdu_bits + 48 * num_blocks;
        for (int8_t pol : pols) {
            std::vector<int8_t> g0p, g1p, g0, g1;
            std::vector<float> llr0p, llr1p, llr0, llr1;
            const auto t_soft = std::chrono::steady_clock::now();
            const bool soft_ok =
                detail::bprf_demod_soft(soft_chips, phr_start,
                                        profile.code_index, 21, 64, 512, 0,
                                        pol, g0p, g1p, llr0p, llr1p) &&
                detail::bprf_demod_soft(soft_chips, payload_start,
                                        profile.code_index, nsym, 8, 64,
                                        1344, pol, g0, g1, llr0, llr1);
            if (sub)
                sub->soft_us += detail::payload_elapsed_us(t_soft);
            if (!soft_ok)
                continue;

            const size_t total = 21 + nsym;
            std::vector<float> llr0j(total), llr1j(total);
            for (size_t s = 0; s < 21; ++s) {
                llr0j[s] = llr0p[s];
                llr1j[s] = llr1p[s];
            }
            for (size_t s = 0; s < nsym; ++s) {
                llr0j[s + 21] = llr0[s];
                llr1j[s + 21] = llr1[s];
            }
            std::vector<int8_t> decoded;
            const auto t_vit = std::chrono::steady_clock::now();
            detail::vitdec_cl3_soft(llr0j.data(), llr1j.data(), total, decoded);
            if (sub)
                sub->viterbi_us += detail::payload_elapsed_us(t_vit);
            if (decoded.size() < total)
                continue;

            std::vector<int8_t> rs_cw(decoded.begin() + 19,
                                      decoded.begin() + (total - 2));
            if (rs_cw.size() != nsym)
                continue;

            std::vector<int8_t> psdu_bits_out;
            const auto t_rs = std::chrono::steady_clock::now();
            const bool rs_ok =
                detail::rs_decode_stream(rs_cw, psdu_bits, psdu_bits_out);
            if (sub)
                sub->rs_us += detail::payload_elapsed_us(t_rs);
            if (!rs_ok)
                continue;

            // PSDU assembly + FCS: bit packing, CRC-16 and the comparison.
            const auto t_crc = std::chrono::steady_clock::now();
            const size_t nbytes = psdu_bits_out.size() / 8;
            PayloadResult cand;
            cand.bytes.resize(nbytes);
            for (size_t i = 0; i < nbytes; ++i) {
                uint8_t b = 0;
                for (int bit = 0; bit < 8; ++bit)
                    b |= static_cast<uint8_t>(psdu_bits_out[8 * i + bit])
                         << bit;
                cand.bytes[i] = b;
            }
            cand.bits.assign(psdu_bits_out.begin(), psdu_bits_out.end());
            if (nbytes >= 2) {
                cand.calculated_fcs =
                    detail::crc16_802154(cand.bytes.data(), nbytes - 2);
                cand.received_fcs =
                    static_cast<uint16_t>(cand.bytes[nbytes - 2]) |
                    (static_cast<uint16_t>(cand.bytes[nbytes - 1]) << 8);
                cand.fcs_pass =
                    (cand.received_fcs == cand.calculated_fcs);
            }
            cand.ok = true;
            if (sub)
                sub->crc_us += detail::payload_elapsed_us(t_crc);
            if (!any_ok || cand.fcs_pass) {
                out = cand;
                any_ok = true;
            }
            if (out.fcs_pass)
                return true;
        }
    }
    return any_ok;
}

bool prepare_fused_payload_pn(DemodScratch& scratch,
                              size_t code_index,
                              size_t symbols)
{
    const size_t nbits = symbols * rxfusion::kChipsPerBurst;
    if (scratch.fused.code_index == code_index &&
        scratch.fused.symbols == symbols &&
        scratch.fused.pn.size() == nbits)
        return true;
    if (!detail::bprf_spreading(scratch.fused.pn, code_index, 1344, nbits)) {
        scratch.fused.code_index = 0;
        scratch.fused.symbols = 0;
        scratch.fused.pn.clear();
        return false;
    }
    scratch.fused.code_index = code_index;
    scratch.fused.symbols = symbols;
    return true;
}

bool stage_payload_fused_cmf(const std::complex<float>* rx,
                             size_t n,
                             const Qm35825Profile& profile,
                             const TimingResult& timing,
                             const CirResult& cir,
                             const std::vector<float>& prefix_soft_chips,
                             const NsSfdResult& ns_sfd,
                             const PhrResult& phr,
                             PayloadResult& out,
                             DemodScratch& scratch,
                             uint64_t& template_us,
                             uint64_t& correlate_us,
                             uint64_t& decode_us,
                             uint64_t& decode_ns,
                             size_t& symbols_out,
                             const std::vector<int8_t>* phr_g0,
                             const std::vector<int8_t>* phr_g1,
                             PayloadSubTimers* sub,
                             const std::vector<float>* phr_llr0,
                             const std::vector<float>* phr_llr1,
                             double cfo_w_rad)
{
    // Diagnostic child timing of `stage_payload_us`; see stage_payload_fcs().
    detail::PayloadSubTimer sub_guard(sub);
    if (sub)
        ++sub->fused_calls;
    out = PayloadResult{};
    template_us = 0;
    correlate_us = 0;
    decode_us = 0;
    decode_ns = 0;
    symbols_out = 0;
    if (!rx || !phr.ok || !ns_sfd.ok || phr.psdu_length == 0 ||
        phr.psdu_length > profile.max_psdu_bytes)
        return false;

    const int64_t phr_start = ns_sfd.sfd_end_chip + 1;
    const int64_t payload_start =
        phr_start + static_cast<int64_t>(rxopt::kPhrSymbols) *
                       static_cast<int64_t>(rxopt::kPhrChipsPerSymbol);
    if (phr_start < 0 || payload_start < 0)
        return false;
    // When the fused PHR stage supplies the raw 21 g0/g1 decisions, the
    // payload path does not need the PHR soft chips in the prefix.  Otherwise
    // it must reproduce the legacy bprf_demod() admission exactly.
    const bool have_phr_decisions = phr_g0 && phr_g1;
    if (!have_phr_decisions &&
        static_cast<size_t>(phr_start) +
                rxopt::kPhrSymbols * rxopt::kPhrChipsPerSymbol >
            prefix_soft_chips.size())
        return false;

    const size_t psdu_bits = static_cast<size_t>(phr.psdu_length) * 8;
    const size_t num_blocks = (psdu_bits + 329) / 330;
    const size_t symbols = psdu_bits + 48 * num_blocks;
    if (symbols == 0)
        return false;
    symbols_out = symbols;

    const size_t tap_count = cir.cir_filter_taps.size();
    if (tap_count == 0 || tap_count > rxfusion::kMaxTapCount)
        return false;
    if (tap_count != profile.cir_pre_samples + profile.cir_post_samples)
        return false;
    const bool known_hop = profile.rx_bpm_known_hop;
    const bool joint_ml = profile.rx_bpm_joint_ml;
    const bool terminated = profile.rx_bpm_terminated_viterbi;
    const bool sfo_track =
        profile.rx_fused_sfo_track &&
        !integer_chip_grid_at(cir.samples_per_chip,
                              rxfusion::kSamplesPerChip,
                              profile.fused_gate_ppm * 1e-6);
    if (!sfo_track &&
        !integer_chip_grid_at(cir.samples_per_chip,
                              rxfusion::kSamplesPerChip,
                              profile.fused_gate_ppm * 1e-6))
        return false;
    // The fractional-phase path is implemented by the 2-candidate kernel only.
    if (sfo_track && !known_hop)
        return false;
    if (profile.cir_pre_samples > tap_count)
        return false;
    if (!scratch.softchip.prepared || std::norm(scratch.softchip.rot) <= 0.0f ||
        !std::isfinite(scratch.softchip.rot.real()) ||
        !std::isfinite(scratch.softchip.rot.imag()))
        return false;

    const int64_t chip_start =
        timing.preamble_start_sample + static_cast<int64_t>(tap_count) -
        static_cast<int64_t>(profile.cir_pre_samples) - 1;
    if (chip_start < 0 || static_cast<int64_t>(n) <= chip_start)
        return false;
    const int64_t max_chip =
        (static_cast<int64_t>(n) - 1 - chip_start) /
        static_cast<int64_t>(rxfusion::kSamplesPerChip);
    if (max_chip < 0)
        return false;

    // Reproduce the exact length of the soft-chip stream that the legacy
    // on-demand path would have produced (prefix + optional extension).  The
    // legacy bprf_demod() admission is `payload_start + 64*nsym <= stream`,
    // which is 16 chips stricter than the last actually used chip.  Using only
    // the raw-IQ length here would be a different contract if the chip budget
    // ever became the limiting factor.
    const size_t prefix_chips = cir.soft_chip_count;
    size_t stream_chips = prefix_chips;
    const size_t final_budget = rxopt::final_chip_budget(
        profile, ns_sfd.sfd_end_chip, phr.psdu_length);
    if (final_budget > prefix_chips) {
        const double spc = cir.samples_per_chip;
        const int64_t last_by_budget =
            chip_start + static_cast<int64_t>(std::ceil(
                             static_cast<double>(final_budget - 1) * spc));
        const int64_t last_chip_sample =
            std::min<int64_t>(static_cast<int64_t>(n), last_by_budget);
        const int64_t last_pos = last_chip_sample - 1;
        if (last_pos < chip_start)
            return false;
        stream_chips = static_cast<size_t>(
                           std::floor(static_cast<double>(last_pos - chip_start) /
                                      spc)) +
                       1;
    }
    if (payload_start + static_cast<int64_t>(symbols) *
                            static_cast<int64_t>(rxopt::kPayloadChipsPerSymbol) >
        static_cast<int64_t>(stream_chips))
        return false;

    const int8_t pol = (ns_sfd.polarity < 0) ? -1 : 1;
    // PHR pilot refinement (M3): the refined rotation already includes the
    // frozen preamble rotation, and its residual CFO is added to the kernel's
    // exact-CFO ramp.
    std::complex<float> rot = scratch.softchip.rot;
    double cfo_eff = cfo_w_rad;
    if (profile.rx_phr_phase_refine && scratch.phr_refine.applied) {
        rot = scratch.phr_refine.rot_at_payload_start;
        cfo_eff += scratch.phr_refine.delta_cfo_w_rad;
    }
    rxfusion::Timings timings;
    {
        // PN build + four-candidate correlation is the fused "soft bit"
        // generation; it is the soft bucket of this path.
        detail::PayloadSubBucket soft_bucket(sub, &PayloadSubTimers::soft_us);
        const auto t_plan0 = std::chrono::steady_clock::now();
        if (!prepare_fused_payload_pn(scratch, profile.code_index, symbols))
            return false;
        const auto t_plan1 = std::chrono::steady_clock::now();
        const uint64_t plan_us = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::microseconds>(t_plan1 -
                                                                  t_plan0)
                .count());

        if (known_hop) {
            const std::complex<float>* bank = nullptr;
            size_t bins = 0;
            if (sfo_track) {
                bins = rxopt::kSfoPhaseBins;
                scratch.frac_cir_bank.resize(bins * tap_count);
                if (!rxopt::build_fractional_cir_bank(
                        cir.cir_filter_taps.data(), tap_count, bins,
                        scratch.frac_cir_bank.data()))
                    return false;
                bank = scratch.frac_cir_bank.data();
            }
            rxfusion::DdPllConfig pll;
            pll.enable = profile.rx_bpm_ddpll;
            if (!rxfusion::correlate_payload_known_hop_fused_cfo(
                    rx, n, cir.cir_filter_taps.data(), tap_count, chip_start,
                    sfo_track
                        ? cir.samples_per_chip
                        : static_cast<double>(rxfusion::kSamplesPerChip),
                    payload_start, scratch.fused.pn.data(), symbols, pol, rot,
                    cfo_eff, pll, scratch.fused.metrics2,
                    scratch.fused.kernel, &timings, /*write_complex=*/false,
                    bank, bins))
                return false;
            scratch.fused.metrics2_valid = true;
            for (float v : scratch.fused.metrics2.m0)
                if (!std::isfinite(v))
                    return false;
            for (float v : scratch.fused.metrics2.m1)
                if (!std::isfinite(v))
                    return false;
        } else {
            if (!rxfusion::correlate_payload_fused_cfo(
                    rx, n, cir.cir_filter_taps.data(), tap_count, chip_start,
                    rxfusion::kSamplesPerChip, payload_start,
                    scratch.fused.pn.data(), symbols, pol, rot, cfo_eff,
                    scratch.fused.metrics, scratch.fused.kernel, &timings,
                    /*write_complex=*/false))
                return false;
            scratch.fused.metrics2_valid = false;
            for (float v : scratch.fused.metrics.signed_value) {
                if (!std::isfinite(v))
                    return false;
            }
        }
        template_us = plan_us + timings.template_update_us;
        correlate_us = timings.correlate_us;
    }

    const auto t_decode0 = std::chrono::steady_clock::now();
    auto stamp_decode = [&]() {
        decode_ns = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now() - t_decode0)
                .count());
        decode_us = decode_ns / 1000;
    };
    std::vector<int8_t> local_g0p, local_g1p;
    std::vector<int8_t>& g0 = scratch.fused.g0;
    std::vector<int8_t>& g1 = scratch.fused.g1;
    const std::vector<int8_t>* g0p = phr_g0;
    const std::vector<int8_t>* g1p = phr_g1;
    {
        // PHR re-demod (only when the fused PHR stage supplied no decisions) and
        // the BPM decisions belong to the metric-generation bucket.
        detail::PayloadSubBucket soft_bucket(sub, &PayloadSubTimers::soft_us);
        if (g0p || g1p) {
            if (!g0p || !g1p || g0p->size() != 21 || g1p->size() != 21) {
                stamp_decode();
                return false;
            }
        } else {
            if (!detail::bprf_demod(prefix_soft_chips, phr_start,
                                    profile.code_index, 21, 64, 512, 0, pol,
                                    local_g0p, local_g1p)) {
                stamp_decode();
                return false;
            }
            g0p = &local_g0p;
            g1p = &local_g1p;
        }
        if (known_hop) {
            const auto& a = scratch.fused.metrics2.m0;
            const auto& b = scratch.fused.metrics2.m1;
            g0.resize(symbols);
            g1.resize(symbols);
            for (size_t s = 0; s < symbols; ++s) {
                g0[s] = (std::abs(a[s]) >= std::abs(b[s])) ? 0 : 1;
                const float best = (g0[s] == 0) ? a[s] : b[s];
                g1[s] = (best < 0.0f) ? 1 : 0;
            }
        } else {
            rxfusion::decide_bpm(scratch.fused.metrics, g0, g1);
        }
    }

    // Attempt 1 — the unchanged hard-decision path.  On success this is the
    // whole cost of a high-SNR packet: nothing below runs.
    const bool decoded = detail::decode_payload_hard(
        *g0p, *g1p, g0, g1, psdu_bits, out, sub, &scratch.fused.decode);
    if (decoded && out.fcs_pass) {
        stamp_decode();
        return true;
    }

    // Attempt 2 (M2) — joint bi-orthogonal ML Viterbi on the same metrics.
    // The PHR metrics come from the fused PHR stage (already in the same
    // rotation frame when the pilot refinement ran).  If the ML attempt fails
    // its FCS the legacy split-LLR soft attempt below still runs, so the
    // switch can never lose a packet the old path could decode.
    if (joint_ml && scratch.fused_phr.metrics2_valid &&
        scratch.fused_phr.metrics2.m0.size() == 21) {
        const float* bm0 = nullptr;
        const float* bm1 = nullptr;
        if (known_hop) {
            bm0 = scratch.fused.metrics2.m0.data();
            bm1 = scratch.fused.metrics2.m1.data();
        } else {
            auto& v0 = scratch.fused.biorth_m0;
            auto& v1 = scratch.fused.biorth_m1;
            v0.resize(symbols);
            v1.resize(symbols);
            const auto& sv = scratch.fused.metrics.signed_value;
            for (size_t s = 0; s < symbols; ++s) {
                const float h0 = std::max(std::abs(sv[4 * s + 0]),
                                          std::abs(sv[4 * s + 1]));
                const float h1 = std::max(std::abs(sv[4 * s + 2]),
                                          std::abs(sv[4 * s + 3]));
                const size_t h = (h1 > h0) ? 1 : 0;
                v0[s] = sv[4 * s + 2 * h + 0];
                v1[s] = sv[4 * s + 2 * h + 1];
            }
            bm0 = v0.data();
            bm1 = v1.data();
        }
        const bool ml_decoded = detail::decode_payload_biorth_soft(
            scratch.fused_phr.metrics2.m0.data(),
            scratch.fused_phr.metrics2.m1.data(), bm0, bm1, symbols,
            psdu_bits, out, sub, &scratch.fused.decode, terminated,
            /*enable_clip=*/true);
        if (ml_decoded && out.fcs_pass) {
            stamp_decode();
            return true;
        }
    }

    // Attempt 3 — in-place soft-decision retry on the SAME four signed metrics.
    // No extra correlation and no extra derotation: the LLRs are a by-product
    // of the metrics the fused kernel already produced.  The PHR LLRs come from
    // the fused PHR cache when it is valid, otherwise from one bprf_demod_soft()
    // pass over the legacy prefix.  This is what keeps a marginal packet out of
    // the ~15 ms full-stream stage_softchip_fallbacks retry.
    std::vector<int8_t>& soft_g0 = scratch.fused.soft_g0;
    std::vector<int8_t>& soft_g1 = scratch.fused.soft_g1;
    std::vector<float>& llr0 = scratch.fused.llr0;
    std::vector<float>& llr1 = scratch.fused.llr1;
    std::vector<int8_t> local_soft_g0p, local_soft_g1p;
    std::vector<float> local_llr0p, local_llr1p;
    const std::vector<float>* llr0p = phr_llr0;
    const std::vector<float>* llr1p = phr_llr1;
    {
        detail::PayloadSubBucket soft_bucket(sub, &PayloadSubTimers::soft_us);
        if (known_hop) {
            const auto& a = scratch.fused.metrics2.m0;
            const auto& b = scratch.fused.metrics2.m1;
            llr0.resize(symbols);
            llr1.resize(symbols);
            for (size_t s = 0; s < symbols; ++s) {
                llr0[s] = std::abs(a[s]) - std::abs(b[s]);
                llr1[s] = std::max(a[s], b[s]) - std::max(-a[s], -b[s]);
            }
        } else {
            rxfusion::decide_bpm_soft(scratch.fused.metrics, soft_g0, soft_g1,
                                      llr0, llr1);
        }
        if (!llr0p || !llr1p || llr0p->size() != 21 || llr1p->size() != 21) {
            llr0p = nullptr;
            llr1p = nullptr;
            if (detail::bprf_demod_soft(prefix_soft_chips, phr_start,
                                        profile.code_index, 21, 64, 512, 0,
                                        pol, local_soft_g0p, local_soft_g1p,
                                        local_llr0p, local_llr1p)) {
                llr0p = &local_llr0p;
                llr1p = &local_llr1p;
            }
        }
    }
    if (llr0p && llr1p) {
        const bool soft_decoded = detail::decode_payload_soft(
            *llr0p, *llr1p, llr0, llr1, psdu_bits, out, sub,
            &scratch.fused.decode);
        if (soft_decoded && out.fcs_pass) {
            stamp_decode();
            return true;
        }
    }

    // Safety net: the known-hop / joint-ML path failed.  Before paying for the
    // ~10 ms full-stream retry, replay the previous-round four-candidate fused
    // decode on the same raw IQ (integer grid only).  This is exactly the M0
    // fused path, so an edge case in the new metric/decoder combination can
    // never cost a packet the old fused path could decode.
    if (known_hop && !sfo_track) {
        detail::PayloadSubBucket soft_bucket(sub, &PayloadSubTimers::soft_us);
        rxfusion::Timings t4;
        if (rxfusion::correlate_payload_fused_cfo(
                rx, n, cir.cir_filter_taps.data(), tap_count, chip_start,
                rxfusion::kSamplesPerChip, payload_start,
                scratch.fused.pn.data(), symbols, pol, rot, cfo_eff,
                scratch.fused.metrics, scratch.fused.kernel, &t4,
                /*write_complex=*/false)) {
            std::vector<int8_t> g0_4, g1_4;
            rxfusion::decide_bpm(scratch.fused.metrics, g0_4, g1_4);
            if (detail::decode_payload_hard(*g0p, *g1p, g0_4, g1_4, psdu_bits,
                                            out, sub, &scratch.fused.decode) &&
                out.fcs_pass) {
                stamp_decode();
                return true;
            }
            std::vector<int8_t> sg0, sg1;
            std::vector<float> l0, l1;
            rxfusion::decide_bpm_soft(scratch.fused.metrics, sg0, sg1, l0, l1);
            if (llr0p && llr1p &&
                detail::decode_payload_soft(*llr0p, *llr1p, l0, l1, psdu_bits,
                                            out, sub,
                                            &scratch.fused.decode) &&
                out.fcs_pass) {
                stamp_decode();
                return true;
            }
        }
    }
    stamp_decode();
    return false;
}

} // namespace openuwb::demod::core

// ---- rx_fused_cmf.h ----

namespace openuwb::demod::rxfusion {
bool correlate_field(const std::complex<float>* rx,
                     size_t n,
                     const std::complex<float>* fir_taps,
                     size_t tap_count,
                     int64_t chip_start,
                     size_t samples_per_chip,
                     int64_t field_start,
                     const int8_t* pn,
                     size_t symbols,
                     const Geometry& geometry,
                     int8_t polarity,
                     std::complex<float> rot,
                     Metrics& out,
                     Scratch& scratch,
                     Timings* timings)
{
    if (!rx || !fir_taps || !pn || symbols == 0 || tap_count == 0 ||
        tap_count > kMaxTapCount)
        return false;
    if (samples_per_chip != kSamplesPerChip)
        return false;
    if (chip_start < 0 || field_start < 0)
        return false;
    if (polarity != 1 && polarity != -1)
        return false;
    if (geometry.candidate_count != kCandidateCount ||
        !geometry.candidate_chip_offset)
        return false;
    if (geometry.chips_per_burst == 0 || geometry.chips_per_symbol == 0)
        return false;
    // Every candidate burst must lie inside its symbol (offset + chips_per_burst <= chips_per_symbol).
    // Compare with subtraction so the addition cannot wrap.
    if (geometry.chips_per_burst > geometry.chips_per_symbol)
        return false;
    const size_t max_allowed_offset =
        geometry.chips_per_symbol - geometry.chips_per_burst;
    size_t max_candidate_offset = 0;
    for (size_t p = 0; p < kCandidateCount; ++p) {
        const size_t off = geometry.candidate_chip_offset[p];
        if (off > max_allowed_offset)
            return false;
        if (off > max_candidate_offset)
            max_candidate_offset = off;
    }
    // Bound the template length without overflowing size_t.  The support
    // limit is the 64-chip PHR burst: 38 taps give 164 complex samples and
    // the absolute maximum is 64 + 2*63 = 190.
    const size_t max_len =
        kMaxTapCount + kSamplesPerChip * (kPhrChipsPerBurst - 1);
    if (geometry.chips_per_burst - 1 >
        (max_len - tap_count) / samples_per_chip)
        return false;
    if (geometry.chips_per_symbol > (static_cast<size_t>(1) << 30))
        return false;
    if (symbols > (static_cast<size_t>(1) << 20))
        return false;
    // The PN contract is a +/-1 sequence.  A zero would silently be treated
    // as +1 by build_template_geometry(), so reject it before producing
    // metrics.
    if (!pn_stream_is_pm1(pn, symbols * geometry.chips_per_burst))
        return false;

    const size_t len =
        template_length(tap_count, samples_per_chip, geometry.chips_per_burst);
    const int64_t spc = static_cast<int64_t>(samples_per_chip);
    // Guard the sample-coordinate product before it enters int64.  Real
    // fields use small chip indices; this only rejects absurd offsets.
    if (field_start >
        (std::numeric_limits<int64_t>::max() - chip_start) / spc)
        return false;
    // Window for symbol 0, hop 0, half 0, chip c=0.  The furthest candidate is
    // symbol symbols-1 with the maximum geometry offset across all candidates;
    // its template then spans len samples.
    const int64_t base0 = chip_start + field_start * spc -
                          static_cast<int64_t>(tap_count) + 1;
    if (base0 < 0)
        return false;
    const int64_t last_start_chip =
        static_cast<int64_t>(symbols - 1) *
            static_cast<int64_t>(geometry.chips_per_symbol) +
        static_cast<int64_t>(max_candidate_offset);
    // Compare as unsigned int64/u64 indices; base0 is non-negative and the
    // geometry bounds above keep this addition far from wrapping.
    const uint64_t add = static_cast<uint64_t>(last_start_chip) *
                             static_cast<uint64_t>(spc) +
                         static_cast<uint64_t>(len) - 1;
    const uint64_t last = static_cast<uint64_t>(base0) + add;
    if (last < static_cast<uint64_t>(base0) ||
        last >= static_cast<uint64_t>(n))
        return false;

    out.reset(symbols);
    const size_t block_symbols = std::min(kBlockSymbols, symbols);
    scratch.ensure(block_symbols, len);

    auto t_template = std::chrono::steady_clock::duration::zero();
    auto t_correlate = std::chrono::steady_clock::duration::zero();
    // No clock calls at all when the caller does not ask for timings.
    const bool want_timings = (timings != nullptr);
    std::chrono::steady_clock::time_point t0, t1, t2;

    for (size_t start = 0; start < symbols; start += kBlockSymbols) {
        const size_t count = std::min(kBlockSymbols, symbols - start);
        if (want_timings)
            t0 = std::chrono::steady_clock::now();
        for (size_t i = 0; i < count; ++i) {
            build_template_geometry(
                scratch.templates.data() + i * len, fir_taps, tap_count,
                pn + (start + i) * geometry.chips_per_burst,
                geometry.chips_per_burst, samples_per_chip);
        }
        if (want_timings) {
            t1 = std::chrono::steady_clock::now();
            t_template += t1 - t0;
        }

        for (size_t i = 0; i < count; ++i) {
            const size_t s = start + i;
            const std::complex<float>* taps =
                scratch.templates.data() + i * len;
            const int64_t win0 =
                base0 + static_cast<int64_t>(s) *
                            static_cast<int64_t>(geometry.chips_per_symbol) *
                            spc;
            for (size_t p = 0; p < kCandidateCount; ++p) {
                const int64_t off =
                    static_cast<int64_t>(
                        geometry.candidate_chip_offset[p]) *
                    spc;
                const std::complex<float> m =
                    dot_template(rx + win0 + off, taps, len);
                out.complex_value[kCandidateCount * s + p] = m;
                const float re = rot.real() * m.real() -
                                 rot.imag() * m.imag();
                out.signed_value[kCandidateCount * s + p] =
                    (polarity < 0) ? -re : re;
            }
        }
        if (want_timings) {
            t2 = std::chrono::steady_clock::now();
            t_correlate += t2 - t1;
        }
    }

    if (timings) {
        timings->template_update_us = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::microseconds>(t_template)
                .count());
        timings->correlate_us = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::microseconds>(t_correlate)
                .count());
    }
    return true;
}

bool correlate_payload(const std::complex<float>* rx,
                       size_t n,
                       const std::complex<float>* fir_taps,
                       size_t tap_count,
                       int64_t chip_start,
                       size_t samples_per_chip,
                       int64_t payload_start,
                       const int8_t* pn,
                       size_t symbols,
                       int8_t polarity,
                       std::complex<float> rot,
                       Metrics& out,
                       Scratch& scratch,
                       Timings* timings)
{
    return correlate_field(rx, n, fir_taps, tap_count, chip_start,
                           samples_per_chip, payload_start, pn, symbols,
                           kPayloadGeometry, polarity, rot, out, scratch,
                           timings);
}

bool correlate_phr(const std::complex<float>* rx,
                   size_t n,
                   const std::complex<float>* fir_taps,
                   size_t tap_count,
                   int64_t chip_start,
                   size_t samples_per_chip,
                   int64_t phr_start,
                   const int8_t* pn,
                   int8_t polarity,
                   std::complex<float> rot,
                   Metrics& out,
                   Scratch& scratch,
                   Timings* timings)
{
    return correlate_field(rx, n, fir_taps, tap_count, chip_start,
                           samples_per_chip, phr_start, pn, kPhrSymbols,
                           kPhrGeometry, polarity, rot, out, scratch, timings);
}

bool correlate_payload_fused_cfo_impl(
    const std::complex<float>* rx_raw,
    size_t n,
    const std::complex<float>* fir_taps,
    size_t tap_count,
    int64_t chip_start,
    size_t samples_per_chip,
    int64_t payload_start,
    const int8_t* pn,
    size_t symbols,
    int8_t polarity,
    std::complex<float> rot,
    double cfo_w_rad,
    Metrics& out,
    Scratch& scratch,
    Timings* timings,
    bool write_complex,
    bool validate_pn)
{
    if (!rx_raw || !fir_taps || !pn)
        return false;
    if (polarity != 1 && polarity != -1)
        return false;
    if (!std::isfinite(cfo_w_rad) || !std::isfinite(rot.real()) ||
        !std::isfinite(rot.imag()))
        return false;
    int64_t base0 = 0;
    size_t len = 0;
    if (!payload_window_bounds(n, tap_count, chip_start, samples_per_chip,
                               payload_start, symbols, base0, len))
        return false;
    if (validate_pn && !pn_stream_is_pm1(pn, symbols * kChipsPerBurst))
        return false;

    const double w = cfo_w_rad;
    const bool has_cfo = (w != 0.0);
    const bool fast = (tap_count == kFusedFastTaps);

    scratch.ensure_fused_cfo(tap_count);
    std::complex<float>* h_w = scratch.fused_h_w.data();
    std::complex<float>* tpl = scratch.fused_tpl.data();

    using clock = std::chrono::steady_clock;
    clock::time_point t_begin;
    if (timings)
        t_begin = clock::now();

    std::complex<float> chip_phase[kChipsPerBurst];
    build_fused_tap_ramp(h_w, fir_taps, tap_count, w, chip_phase);
    if (fast) {
        build_fused_half_lut(scratch.fused_lut_lo.data(),
                             scratch.fused_lut_hi.data(), h_w, chip_phase);
    }
    clock::time_point t_templ;
    if (timings)
        t_templ = clock::now();

    // Per-packet phase of each candidate window offset (chips -> samples).
    std::complex<float> cand_phase[kCandidateCount];
    for (size_t p = 0; p < kCandidateCount; ++p) {
        if (!has_cfo) {
            cand_phase[p] = std::complex<float>(1.0f, 0.0f);
            continue;
        }
        const double ang = -w * static_cast<double>(
                                   kCandidateChipOffset[p] * samples_per_chip);
        cand_phase[p] = std::complex<float>(static_cast<float>(std::cos(ang)),
                                           static_cast<float>(std::sin(ang)));
    }

    out.reset(symbols, write_complex);

    const int64_t symbol_stride =
        static_cast<int64_t>(kChipsPerSymbol) *
        static_cast<int64_t>(samples_per_chip);
    const double dphi = -w * static_cast<double>(symbol_stride);
    const std::complex<double> step(std::cos(dphi), std::sin(dphi));
    std::complex<double> ramp(1.0, 0.0);

    const std::complex<float>* lut_lo = scratch.fused_lut_lo.data();
    const std::complex<float>* lut_hi = scratch.fused_lut_hi.data();
    const int64_t off_sample[kCandidateCount] = {
        static_cast<int64_t>(kCandidateChipOffset[0]) *
            static_cast<int64_t>(samples_per_chip),
        static_cast<int64_t>(kCandidateChipOffset[1]) *
            static_cast<int64_t>(samples_per_chip),
        static_cast<int64_t>(kCandidateChipOffset[2]) *
            static_cast<int64_t>(samples_per_chip),
        static_cast<int64_t>(kCandidateChipOffset[3]) *
            static_cast<int64_t>(samples_per_chip)
    };

    for (size_t s = 0; s < symbols; ++s) {
        std::complex<float> phase(1.0f, 0.0f);
        if (has_cfo) {
            if ((s % kFusedPhaseAnchor) == 0) {
                const double ang =
                    -w * static_cast<double>(
                             base0 + static_cast<int64_t>(s) * symbol_stride);
                ramp = std::complex<double>(std::cos(ang), std::sin(ang));
            }
            phase = std::complex<float>(static_cast<float>(ramp.real()),
                                        static_cast<float>(ramp.imag()));
            ramp *= step;
        }

        if (fast) {
            const int8_t* ps = pn + s * kChipsPerBurst;
            const float sign_lo = (ps[0] < 0) ? -1.0f : 1.0f;
            const float sign_hi = (ps[4] < 0) ? -1.0f : 1.0f;
            size_t idx_lo = 0;
            size_t idx_hi = 0;
            for (size_t c = 1; c < 4; ++c) {
                if (ps[c] != ps[0])
                    idx_lo |= (static_cast<size_t>(1) << (c - 1));
                if (ps[c + 4] != ps[4])
                    idx_hi |= (static_cast<size_t>(1) << (c - 1));
            }
            build_fused_symbol_template(tpl, lut_lo, lut_hi, idx_lo, sign_lo,
                                        idx_hi, sign_hi);
        } else {
            std::fill(tpl, tpl + len, std::complex<float>(0.0f, 0.0f));
            for (size_t c = 0; c < kChipsPerBurst; ++c) {
                const float sgn =
                    (pn[s * kChipsPerBurst + c] < 0) ? -1.0f : 1.0f;
                accumulate_scaled_complex(tpl + samples_per_chip * c, h_w,
                                          tap_count,
                                          sgn * chip_phase[c].real(),
                                          sgn * chip_phase[c].imag());
            }
        }

        const int64_t win0 =
            base0 + static_cast<int64_t>(s) * symbol_stride;
        std::complex<float> m[kCandidateCount];
        if (fast) {
            dot4_shared_fixed52(rx_raw + win0 + off_sample[0],
                                rx_raw + win0 + off_sample[1],
                                rx_raw + win0 + off_sample[2],
                                rx_raw + win0 + off_sample[3], tpl, m);
        } else {
            for (size_t p = 0; p < kCandidateCount; ++p)
                m[p] = dot_generic(rx_raw + win0 + off_sample[p], tpl, len);
        }

        for (size_t p = 0; p < kCandidateCount; ++p) {
            std::complex<float> value = m[p];
            if (has_cfo)
                value = (phase * cand_phase[p]) * m[p];
            if (write_complex)
                out.complex_value[kCandidateCount * s + p] = value;
            const float re = rot.real() * value.real() -
                             rot.imag() * value.imag();
            out.signed_value[kCandidateCount * s + p] =
                (polarity < 0) ? -re : re;
        }
    }

    if (timings) {
        const clock::time_point t_end = clock::now();
        timings->template_update_us = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::microseconds>(t_templ -
                                                                 t_begin)
                .count());
        timings->correlate_us = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::microseconds>(t_end -
                                                                 t_templ)
                .count());
    }
    return true;
}

bool correlate_payload_fused_cfo(
    const std::complex<float>* rx_raw,
    size_t n,
    const std::complex<float>* fir_taps,
    size_t tap_count,
    int64_t chip_start,
    size_t samples_per_chip,
    int64_t payload_start,
    const int8_t* pn,
    size_t symbols,
    int8_t polarity,
    std::complex<float> rot,
    double cfo_w_rad,
    Metrics& out,
    Scratch& scratch,
    Timings* timings,
    bool write_complex)
{
    return correlate_payload_fused_cfo_impl(
        rx_raw, n, fir_taps, tap_count, chip_start, samples_per_chip,
        payload_start, pn, symbols, polarity, rot, cfo_w_rad, out, scratch,
        timings, write_complex, true);
}

void decide_bpm_impl(const Metrics& metrics,
                     std::vector<int8_t>& g0, std::vector<int8_t>& g1,
                     std::vector<float>* llr0, std::vector<float>* llr1)
{
    g0.resize(metrics.symbols);
    g1.resize(metrics.symbols);
    if (llr0) {
        llr0->resize(metrics.symbols);
        llr1->resize(metrics.symbols);
    }
    for (size_t s = 0; s < metrics.symbols; ++s) {
        const float* m = &metrics.signed_value[kCandidateCount * s];
        const auto decision = rxopt::decide_bpm_symbol(m, false);
        g0[s] = decision.position;
        g1[s] = decision.polarity;
        if (llr0) {
            (*llr0)[s] = decision.position_llr;
            (*llr1)[s] = rxopt::bpm_polarity_llr(m);
        }
    }
}

void decide_bpm(const Metrics& metrics,
                std::vector<int8_t>& g0,
                std::vector<int8_t>& g1)
{
    decide_bpm_impl(metrics, g0, g1, nullptr, nullptr);
}

void decide_bpm_soft(const Metrics& metrics,
                     std::vector<int8_t>& g0, std::vector<int8_t>& g1,
                     std::vector<float>& llr0, std::vector<float>& llr1)
{
    decide_bpm_impl(metrics, g0, g1, &llr0, &llr1);
}

bool correlate_payload_known_hop_fused_cfo(
    const std::complex<float>* rx_raw,
    size_t n,
    const std::complex<float>* fir_taps,
    size_t tap_count,
    int64_t chip_start,
    double samples_per_chip_actual,
    int64_t payload_start,
    const int8_t* pn,
    size_t symbols,
    int8_t polarity,
    std::complex<float> rot,
    double cfo_w_rad,
    const DdPllConfig& pll_cfg,
    Metrics2& out,
    Scratch& scratch,
    Timings* timings,
    bool write_complex,
    const std::complex<float>* frac_cir_bank,
    size_t frac_phase_bins)
{
    if (!rx_raw || !fir_taps || !pn)
        return false;
    if (polarity != 1 && polarity != -1)
        return false;
    if (!std::isfinite(cfo_w_rad) || !std::isfinite(rot.real()) ||
        !std::isfinite(rot.imag()))
        return false;
    if (tap_count == 0 || tap_count > kMaxTapCount)
        return false;
    if (symbols == 0 || symbols > (static_cast<size_t>(1) << 20))
        return false;
    if (chip_start < 0 || payload_start < 0)
        return false;
    if (!std::isfinite(samples_per_chip_actual) ||
        !(samples_per_chip_actual > 0.0))
        return false;
    if (pll_cfg.enable) {
        if (!std::isfinite(pll_cfg.alpha) || !std::isfinite(pll_cfg.beta) ||
            !std::isfinite(pll_cfg.max_freq_step) ||
            !std::isfinite(pll_cfg.min_mag_ratio) || pll_cfg.beta < 0.0f ||
            pll_cfg.max_freq_step < 0.0f || pll_cfg.min_mag_ratio < 0.0f)
            return false;
    }

    const bool sfo = (frac_cir_bank != nullptr);
    if (sfo) {
        if (frac_phase_bins == 0 || frac_phase_bins > kKhMaxPhaseBins)
            return false;
    } else if (samples_per_chip_actual !=
               static_cast<double>(kSamplesPerChip)) {
        // Non-integer spacing without a phase bank is unsupported.
        return false;
    }

    // The hop is taken from the same chip as the first half template sign, so
    // the +/-1 scan also protects the hop rule from a zero chip.
    if (!pn_stream_is_pm1(pn, symbols * kChipsPerBurst))
        return false;

    const size_t len =
        template_length(tap_count, kSamplesPerChip, kChipsPerBurst);
    const double w = cfo_w_rad;
    const bool has_cfo = (w != 0.0);
    const bool fast = (tap_count == kFusedFastTaps);
    const bool sfo_path = sfo;
    const double spc_d =
        sfo_path ? samples_per_chip_actual
                 : static_cast<double>(kSamplesPerChip);
    const size_t bins = sfo_path ? frac_phase_bins : 0;

    int64_t base0 = 0;
    double base_d = 0.0;
    const int64_t stride_i =
        static_cast<int64_t>(kChipsPerSymbol) *
        static_cast<int64_t>(kSamplesPerChip); // 128
    const double stride_d = static_cast<double>(kChipsPerSymbol) * spc_d;

    if (!sfo_path) {
        const int64_t spc = static_cast<int64_t>(kSamplesPerChip);
        if (payload_start >
            (std::numeric_limits<int64_t>::max() - chip_start) / spc)
            return false;
        const int64_t window0 = chip_start + payload_start * spc;
        base0 = window0 - static_cast<int64_t>(tap_count) + 1;
        if (base0 < 0)
            return false;
        // Furthest window: known hop 1, BPM half 1 = 16 + 64 samples.
        const int64_t max_off = 16 + 64;
        const uint64_t add =
            static_cast<uint64_t>(symbols - 1) *
                static_cast<uint64_t>(stride_i) +
            static_cast<uint64_t>(max_off) + static_cast<uint64_t>(len) - 1;
        const uint64_t last = static_cast<uint64_t>(base0) + add;
        if (last < static_cast<uint64_t>(base0) ||
            last >= static_cast<uint64_t>(n))
            return false;
        base_d = static_cast<double>(base0);
    } else {
        base_d = static_cast<double>(chip_start) +
                 static_cast<double>(payload_start) * spc_d -
                 static_cast<double>(tap_count) + 1.0;
        if (base_d < 0.0 || !std::isfinite(base_d))
            return false;
        const double last_d = base_d +
                              static_cast<double>(symbols - 1) * stride_d +
                              40.0 * spc_d; // hop 1 + half 1 = 40 chips
        if (!std::isfinite(last_d) || last_d > 9.0e15)
            return false;
        int64_t last_int = 0;
        size_t last_bin = 0;
        if (!decompose_frac_position(last_d, bins, last_int, last_bin))
            return false;
        if (last_int < 0)
            return false;
        const uint64_t last =
            static_cast<uint64_t>(last_int) + static_cast<uint64_t>(len) - 1;
        if (last >= static_cast<uint64_t>(n))
            return false;
        (void)last_bin;
    }

    scratch.ensure_known_hop(tap_count);
    if (sfo_path)
        scratch.ensure_known_hop_sfo(tap_count, bins);

    using clock = std::chrono::steady_clock;
    clock::time_point t_begin{};
    clock::time_point t_templ{};
    if (timings)
        t_begin = clock::now();

    std::complex<float> chip_phase[kChipsPerBurst];
    std::complex<float>* tpl = scratch.fused_tpl.data();
    if (!sfo_path) {
        build_fused_tap_ramp(scratch.fused_h_w.data(), fir_taps, tap_count, w,
                             chip_phase);
        if (fast) {
            build_fused_half_lut(scratch.fused_lut_lo.data(),
                                 scratch.fused_lut_hi.data(),
                                 scratch.fused_h_w.data(), chip_phase);
        }
    } else {
        for (size_t b = 0; b < bins; ++b) {
            build_fused_tap_ramp(scratch.kh_h_w.data() + b * tap_count,
                                 frac_cir_bank + b * tap_count, tap_count, w,
                                 chip_phase);
        }
        if (fast) {
            for (size_t b = 0; b < bins; ++b) {
                std::complex<float>* lo =
                    scratch.kh_lut.data() + b * kKhLutPerBin;
                std::complex<float>* hi =
                    lo + kFusedHalfTemplates * kFusedHalfSpan;
                build_fused_half_lut(lo, hi,
                                     scratch.kh_h_w.data() + b * tap_count,
                                     chip_phase);
            }
        }
    }
    if (timings)
        t_templ = clock::now();

    // Per-packet constants.  hop_phase[1] and half_phase are the same
    // single-rounded values as the 4-candidate cand_phase[] entries for chip
    // offsets 8 and 32, so the integer path stays close to bit-identical.
    std::complex<float> hop_phase[2] = {
        std::complex<float>(1.0f, 0.0f), std::complex<float>(1.0f, 0.0f)
    };
    std::complex<float> half_phase(1.0f, 0.0f);
    if (!sfo_path && has_cfo) {
        hop_phase[1] = std::complex<float>(
            static_cast<float>(std::cos(-w * 16.0)),
            static_cast<float>(std::sin(-w * 16.0)));
        half_phase = std::complex<float>(
            static_cast<float>(std::cos(-w * 64.0)),
            static_cast<float>(std::sin(-w * 64.0)));
    }
    const double dphi = -w * static_cast<double>(stride_i);
    const std::complex<double> ramp_step(std::cos(dphi), std::sin(dphi));
    std::complex<double> ramp(1.0, 0.0);

    out.reset(symbols, write_complex);

    const bool pll_on = pll_cfg.enable;
    // rot_cur is the per-symbol tracking rotation.  Polarity is applied only
    // to the final real metric, never folded into z0/z1 or the phase detector.
    std::complex<float> rot_cur = rot;
    float pll_freq = 0.0f;
    float pll_theta = 0.0f;
    float mag_ema = 0.0f;
    bool mag_init = false;
    size_t pll_updates = 0;

    for (size_t s = 0; s < symbols; ++s) {
        const int8_t* ps = pn + s * kChipsPerBurst;
        const int hop = (ps[0] < 0) ? 1 : 0;
        const float sign_lo = (ps[0] < 0) ? -1.0f : 1.0f;
        const float sign_hi = (ps[4] < 0) ? -1.0f : 1.0f;

        int64_t win0 = 0;
        int64_t win1 = 0;
        size_t bin = 0;
        std::complex<float> phase0(1.0f, 0.0f);
        std::complex<float> phase1(1.0f, 0.0f);

        if (!sfo_path) {
            if (has_cfo) {
                if ((s % kFusedPhaseAnchor) == 0) {
                    const double ang =
                        -w * static_cast<double>(
                                 base0 + static_cast<int64_t>(s) * stride_i);
                    ramp = std::complex<double>(std::cos(ang), std::sin(ang));
                }
                // Anchor at win0 + hop*16, i.e. the true half-0 window.
                phase0 = std::complex<float>(static_cast<float>(ramp.real()),
                                             static_cast<float>(ramp.imag())) *
                         hop_phase[hop];
                phase1 = phase0 * half_phase;
                ramp *= ramp_step;
            }
            win0 = base0 + static_cast<int64_t>(s) * stride_i + hop * 16;
            win1 = win0 + 64;
        } else {
            // Half 0 starts at the known-hop offset (hop*8 chips); half 1 is
            // another 32 chips later, matching the integer path.
            const double pos0 =
                base_d + static_cast<double>(s) * stride_d +
                static_cast<double>(hop * 8) * spc_d;
            int64_t int0 = 0;
            int64_t int1 = 0;
            size_t bin0 = 0;
            size_t bin1 = 0;
            if (!decompose_frac_position(pos0, bins, int0, bin0))
                return false;
            if (!decompose_frac_position(
                    pos0 + static_cast<double>(kChipsPerSymbol / 2) * spc_d,
                    bins, int1, bin1))
                return false;
            win0 = int0;
            win1 = int1;
            bin = bin0;
            (void)bin1;
            if (has_cfo) {
                const double ang0 = -w * static_cast<double>(int0);
                const double ang1 = -w * static_cast<double>(int1);
                phase0 = std::complex<float>(
                    static_cast<float>(std::cos(ang0)),
                    static_cast<float>(std::sin(ang0)));
                phase1 = std::complex<float>(
                    static_cast<float>(std::cos(ang1)),
                    static_cast<float>(std::sin(ang1)));
            }
        }

        if (fast) {
            size_t idx_lo = 0;
            size_t idx_hi = 0;
            for (size_t c = 1; c < 4; ++c) {
                if (ps[c] != ps[0])
                    idx_lo |= (static_cast<size_t>(1) << (c - 1));
                if (ps[c + 4] != ps[4])
                    idx_hi |= (static_cast<size_t>(1) << (c - 1));
            }
            const std::complex<float>* lut_lo;
            const std::complex<float>* lut_hi;
            if (sfo_path) {
                lut_lo = scratch.kh_lut.data() + bin * kKhLutPerBin;
                lut_hi = lut_lo + kFusedHalfTemplates * kFusedHalfSpan;
            } else {
                lut_lo = scratch.fused_lut_lo.data();
                lut_hi = scratch.fused_lut_hi.data();
            }
            build_fused_symbol_template(tpl, lut_lo, lut_hi, idx_lo, sign_lo,
                                        idx_hi, sign_hi);
        } else {
            const std::complex<float>* hw =
                sfo_path ? scratch.kh_h_w.data() + bin * tap_count
                         : scratch.fused_h_w.data();
            std::fill(tpl, tpl + len, std::complex<float>(0.0f, 0.0f));
            for (size_t c = 0; c < kChipsPerBurst; ++c) {
                const float sgn = (ps[c] < 0) ? -1.0f : 1.0f;
                accumulate_scaled_complex(
                    tpl + kSamplesPerChip * c, hw, tap_count,
                    sgn * chip_phase[c].real(), sgn * chip_phase[c].imag());
            }
        }

        std::complex<float> m[2];
        if (fast) {
            dot2_shared_fixed52(rx_raw + win0, rx_raw + win1, tpl, m);
        } else {
            m[0] = dot_generic(rx_raw + win0, tpl, len);
            m[1] = dot_generic(rx_raw + win1, tpl, len);
        }
        // Raw complex correlation on the same phase basis as
        // Metrics::complex_value: CFO fusion applied, rot/polarity/PLL not.
        const std::complex<float> v0 = has_cfo ? phase0 * m[0] : m[0];
        const std::complex<float> v1 = has_cfo ? phase1 * m[1] : m[1];
        if (write_complex) {
            out.z0[s] = v0;
            out.z1[s] = v1;
        }

        if (pll_on) {
            // rot_cur is the per-symbol tracked rotation (starts at rot).
            const std::complex<float> r0 = rot_cur * v0;
            const std::complex<float> r1 = rot_cur * v1;
            const float re0 = r0.real();
            const float re1 = r1.real();
            out.m0[s] = (polarity < 0) ? -re0 : re0;
            out.m1[s] = (polarity < 0) ? -re1 : re1;

            const float norm0 = std::norm(r0);
            const float norm1 = std::norm(r1);
            const std::complex<float> z_win =
                (norm0 >= norm1) ? r0 : r1;
            const float mag = std::sqrt(std::norm(z_win));
            if (!std::isfinite(mag)) {
                // Never propagate a non-finite phase error into the loop.
                continue;
            }
            if (!mag_init) {
                mag_ema = mag;
                mag_init = true;
            } else {
                mag_ema += 0.05f * (mag - mag_ema);
            }
            if (!(mag_ema > 0.0f) ||
                mag < pll_cfg.min_mag_ratio * mag_ema)
                continue;
            const float sgn = (z_win.real() >= 0.0f) ? 1.0f : -1.0f;
            const float err = (sgn * z_win.imag()) / (mag + 1e-12f);
            pll_freq += pll_cfg.beta * err;
            pll_freq = std::max(-pll_cfg.max_freq_step,
                                std::min(pll_cfg.max_freq_step, pll_freq));
            const float dtheta = pll_cfg.alpha * err + pll_freq;
            pll_theta -= dtheta;
            rot_cur *= ddpll_rot_neg(dtheta);
            ++pll_updates;
            if ((pll_updates % 64) == 0) {
                const float rmag = std::sqrt(std::norm(rot_cur));
                if (rmag > 0.0f && std::isfinite(rmag))
                    rot_cur /= rmag;
            }
        } else {
            // PLL disabled: rot_cur == rot, and the arithmetic matches the
            // 4-candidate signed_value expression exactly.
            const float re0 = rot.real() * v0.real() -
                              rot.imag() * v0.imag();
            const float re1 = rot.real() * v1.real() -
                              rot.imag() * v1.imag();
            out.m0[s] = (polarity < 0) ? -re0 : re0;
            out.m1[s] = (polarity < 0) ? -re1 : re1;
        }
    }

    out.pll_freq_rad = pll_freq;
    out.pll_theta = pll_theta;
    out.pll_updates = pll_updates;

    if (timings) {
        const clock::time_point t_end = clock::now();
        timings->template_update_us = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::microseconds>(t_templ -
                                                                 t_begin)
                .count());
        timings->correlate_us = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::microseconds>(t_end -
                                                                 t_templ)
                .count());
    }
    return true;
}

bool correlate_phr_known_hop(const std::complex<float>* rx,
                             size_t n,
                             const std::complex<float>* fir_taps,
                             size_t tap_count,
                             int64_t chip_start,
                             size_t samples_per_chip,
                             int64_t phr_start,
                             const int8_t* pn,
                             int8_t polarity,
                             std::complex<float> rot,
                             Metrics2& out,
                             Scratch& scratch,
                             Timings* timings)
{
    if (!rx || !fir_taps || !pn)
        return false;
    if (polarity != 1 && polarity != -1)
        return false;
    if (tap_count == 0 || tap_count > kMaxTapCount)
        return false;
    if (samples_per_chip != kSamplesPerChip)
        return false;
    if (chip_start < 0 || phr_start < 0)
        return false;
    if (!std::isfinite(rot.real()) || !std::isfinite(rot.imag()))
        return false;
    if (!pn_stream_is_pm1(pn, kPhrSymbols * kPhrChipsPerBurst))
        return false;

    const int64_t spc = static_cast<int64_t>(samples_per_chip);
    if (phr_start > (std::numeric_limits<int64_t>::max() - chip_start) / spc)
        return false;
    const size_t len = template_length(tap_count, kSamplesPerChip,
                                       kPhrChipsPerBurst);
    const int64_t base0 = chip_start + phr_start * spc -
                          static_cast<int64_t>(tap_count) + 1;
    if (base0 < 0)
        return false;
    // Furthest window: known hop 1, BPM half 1 = (64 + 256) chips.
    const int64_t max_off =
        static_cast<int64_t>(kPhrChipsPerBurst + kPhrChipsPerSymbol / 2) * spc;
    const int64_t stride_i =
        static_cast<int64_t>(kPhrChipsPerSymbol) * spc;
    const uint64_t add = static_cast<uint64_t>(kPhrSymbols - 1) *
                             static_cast<uint64_t>(stride_i) +
                         static_cast<uint64_t>(max_off) +
                         static_cast<uint64_t>(len) - 1;
    const uint64_t last = static_cast<uint64_t>(base0) + add;
    if (last < static_cast<uint64_t>(base0) || last >= static_cast<uint64_t>(n))
        return false;

    using clock = std::chrono::steady_clock;
    clock::time_point t_begin{};
    clock::time_point t_templ{};
    if (timings)
        t_begin = clock::now();

    scratch.ensure(kPhrSymbols, len);
    std::complex<float>* templates = scratch.templates.data();
    for (size_t s = 0; s < kPhrSymbols; ++s) {
        build_template_geometry(templates + s * len, fir_taps, tap_count,
                                pn + s * kPhrChipsPerBurst,
                                kPhrChipsPerBurst, kSamplesPerChip);
    }
    if (timings)
        t_templ = clock::now();

    out.reset(kPhrSymbols, true);
    for (size_t s = 0; s < kPhrSymbols; ++s) {
        const int hop =
            (pn[s * kPhrChipsPerBurst] < 0) ? 1 : 0;
        const int64_t win0 = base0 + static_cast<int64_t>(s) * stride_i +
                             static_cast<int64_t>(hop * kPhrChipsPerBurst) *
                                 spc;
        const int64_t win1 =
            win0 + static_cast<int64_t>(kPhrChipsPerSymbol / 2) * spc;
        const std::complex<float>* tpl = templates + s * len;
        // Raw complex correlation on the Metrics::complex_value phase basis:
        // no rot, no polarity.  Wave 2 consumes these 21 pilot values
        // directly, so they are always written.
        const std::complex<float> z0 = dot_generic(rx + win0, tpl, len);
        const std::complex<float> z1 = dot_generic(rx + win1, tpl, len);
        out.z0[s] = z0;
        out.z1[s] = z1;
        const float re0 = rot.real() * z0.real() -
                          rot.imag() * z0.imag();
        const float re1 = rot.real() * z1.real() -
                          rot.imag() * z1.imag();
        out.m0[s] = (polarity < 0) ? -re0 : re0;
        out.m1[s] = (polarity < 0) ? -re1 : re1;
    }

    if (timings) {
        const clock::time_point t_end = clock::now();
        timings->template_update_us = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::microseconds>(t_templ -
                                                                 t_begin)
                .count());
        timings->correlate_us = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::microseconds>(t_end -
                                                                 t_templ)
                .count());
    }
    return true;
}

} // namespace openuwb::demod::rxfusion
