/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Range planning helpers for the experimental on-demand soft-chip path
 * (profile.rx_softchip_on_demand).
 *
 * Production stage_cir_softchips() budgets the chip stream from
 * profile.max_psdu_bytes (127 by default): about 184758 chips for the default
 * profile, truncated only by the input length n.  A 32-byte PSDU occupies
 * about 67872 chips including the 64-chip tail guard, so roughly 40% of the
 * matched-filter work can be outside the actual packet on a long mixed window.
 *
 * The helpers below are pure arithmetic and allocate nothing.  They do not
 * touch the legacy stage; they only describe how many chips are needed:
 *
 *   * payload_chips_needed()   -- exactly stage_phr::chips_needed()
 *   * legacy_chip_budget()     -- exactly the current n_chips formula
 *   * initial_chip_budget()    -- prefix covering SYNC alignment, NS-SFD + PHR
 *   * final_chip_budget()      -- PHR-derived payload range (plus guard)
 *   * rescale_prefix()         -- two-phase normalization helper
 *
 * The default profile numbers (preamble_repetitions=64, max_psdu_bytes=127)
 * are 184758 legacy chips, 59741 initial chips, and 67872 final chips for a
 * 32-byte PSDU.  The prefix must cover chip 47327 (NS-SFD + PHR end), so the
 * 59741-chip prefix has 12413 chips of headroom for the +/-8-chip NS-SFD
 * search and the phase-alignment SYNC block.
 *
 * Coordinate convention: every chip index is an absolute index into the
 * soft-chip stream produced by stage_cir_softchips*(), i.e. index 0 is
 * chip_start = preamble_start + post - 1.  SFD/PHR/payload coordinates used
 * by final_chip_budget() come from the existing NsSfdResult/PhrResult fields.
 *
 * These functions are diagnostics/planning only: they never read the
 * transmitter's PSDU length.  final_chip_budget() must be called with the
 * length decoded and validated by stage_phr().
 */

#pragma once

#include <openuwb/phy/hrp/common/profile.h>
#include <openuwb/phy/hrp/rx/rx_sync_opt.h>

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace gr {
namespace uwb {
namespace demod {
namespace rxopt {

// Compute the legacy SYNC phase statistic without generating SYNC soft chips.
// Sum corresponding IQ samples across repetitions first, then contract the
// one-symbol sum with the sparse spreading code and CMF. Linearity preserves
// the old statistic, apart from floating-point summation order.
inline bool sync_phase_gain(const std::complex<float>* rx, size_t n,
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

// Contract a pre-averaged SYNC tail with the CIR taps and the spreading code,
// producing exactly the statistic sync_phase_gain() returns after its own
// repetition sum.  The tail is the coherent average of the last
// min(32, preamble_repetitions) SYNCs on the 2*508 integer grid, already
// CFO-compensated, so the 32 symbol windows no longer have to be re-read.
inline bool sync_phase_gain_from_tail(const std::complex<float>* tail,
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

// Project a chip range onto the recovered SYNC phase and return its peak
// magnitude so the caller can normalize a prefix or update the full stream.
inline float project_soft_chips(const std::complex<float>* fir,
                                float* chips,
                                size_t count,
                                std::complex<float> rotation)
{
    const float rr = rotation.real();
    const float ri = rotation.imag();
    float peak = 0.0f;
    for (size_t i = 0; i < count; ++i) {
        const float chip = fir[i].real() * rr - fir[i].imag() * ri;
        chips[i] = chip;
        const float mag = chip >= 0.0f ? chip : -chip;
        if (mag > peak)
            peak = mag;
    }
    return peak;
}

// Geometry used by stage_phr()/stage_payload_fcs() (6.81 Mb/s payload).
inline constexpr size_t kPhrSymbols = 21;
inline constexpr size_t kPhrChipsPerSymbol = 512;
inline constexpr size_t kPayloadChipsPerSymbol = 64;
// 802.15.4z NS-SFD is 8 symbols in every profile the RX supports today
// (stage_cir_softchips() hardcodes sfd_symbols = 8).
inline constexpr size_t kSfdSymbols = 8;
// Conservatively large PHR head used by the production budget formula
// (40 symbols x 508 chips; the actual PHR is 21 x 512 chips).
inline constexpr size_t kLegacyPhrHeadSymbols = 40;
// Matches n_guard in stage_cir_softchips().
inline constexpr size_t kLegacyGuardSymbols = 8;
// Matches n_extra in stage_cir_softchips(): 1000 chips per byte of PSDU
// budget above the 12-byte PHR/header allowance.
inline constexpr double kLegacyExtraChipsPerByte = 1000.0;
// Legacy budget safety factor (1.05).
inline constexpr double kLegacyBudgetFactor = 1.05;
// Tail guard added after the PHR-derived payload range.  It absorbs the
// PSDU-length neighbourhood tried by stage_payload_fcs() (the caller already
// budgets psdu_length+2; this is an extra cushion for the final symbol's
// pulse/matched-filter tail).
inline constexpr size_t kFinalGuardChips = 64;
// Same epsilon as stage_cir_softchips(): inv_denom = 1/(max + eps).
inline constexpr float kNormalizationEps = 1e-12f;

// ---------------------------------------------------------------------------
// Chips from the PHR start (SFD end + 1) through the last payload symbol.
// Used by PHR length validation and the final soft-chip budget:
//   psdu_bits = psdu_len*8
//   num_blocks = (psdu_bits + 329) / 330   // RS(63,55) blocks of 330 bits
//   nsym = psdu_bits + 48*num_blocks
//   chips = 21*512 (PHR) + nsym*64 (6.81 Mb/s payload)
// ---------------------------------------------------------------------------
inline size_t payload_chips_needed(uint32_t psdu_len)
{
    const size_t psdu_bits = static_cast<size_t>(psdu_len) * 8;
    const size_t num_blocks = (psdu_bits + 329) / 330;
    const size_t nsym = psdu_bits + 48 * num_blocks;
    return kPhrSymbols * kPhrChipsPerSymbol + nsym * kPayloadChipsPerSymbol;
}

// ---------------------------------------------------------------------------
// Full-stream chip budget. It also caps the initial and final on-demand
// budgets:
//
//   n_shr     = (preamble_repetitions + 8) * 508
//   n_phr_head= 40 * 508
//   n_extra   = ceil((max_psdu_bytes > 12 ? max_psdu_bytes - 12 : 0) * 1000)
//   n_guard   = 8 * 508
//   n_chips   = ceil((n_shr + n_phr_head + n_extra + n_guard) * 1.05)
//
// The double-precision multiplication and ceil are intentional: the floating
// point result is part of the current behaviour (e.g. 159154.8 for a 16-byte
// budget becomes 159155).
// ---------------------------------------------------------------------------
inline size_t legacy_chip_budget(const Qm35825Profile& profile)
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

// ---------------------------------------------------------------------------
// Prefix budget: enough chips for
//   * phase alignment against the last min(32, preamble_repetitions) SYNCs
//     (ends at preamble_repetitions*508),
//   * the 8-symbol NS-SFD (expected at preamble_repetitions*508),
//   * the 21-symbol PHR (starts at SFD end + 1).
// It deliberately reuses the production head/guard geometry (SFD + 40
// "PHR-head" symbols) so the prefix cannot be tighter than what the decoder
// is already assumed to see, then caps at legacy_chip_budget().
//
// Default profile: ceil(1.05 * ((64+8)*508 + 40*508)) = 59741 chips, which
// covers the required 32512 + 4064 + 21*512 = 47328 chips with 12413 spare.
// The formula is >= the required chip index for every preamble_repetitions.
// ---------------------------------------------------------------------------
inline size_t initial_chip_budget(const Qm35825Profile& profile)
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

// ---------------------------------------------------------------------------
// PHR-fusion prefix budget.  The fused PHR kernel consumes raw CFO-compensated
// IQ, so the soft-chip prefix only has to cover the dependencies that still
// read chips:
//   * phase alignment ends at preamble_repetitions * 508 chips;
//   * NS-SFD searches +/-8 chips around that point and spans 8 SFD symbols.
// The worst-case NS-SFD shift is +8 chips, so the last required chip is at
//   expected + 8 + 8*508 - 1.
// The 21 PHR symbols are NOT generated on this path; the fused kernel reads
// them from the same raw IQ.  stage_cir_softchips() maps a chip budget B to
// B-1 generated chips for the production integer grid, so add two entries.
// This is still capped by the legacy budget and is only used when the explicit
// PHR-fusion option is on.
// Default profile: 32512 + 8 + 4064 + 2 = 36586 chips.
// ---------------------------------------------------------------------------
inline size_t fused_phr_prefix_budget(const Qm35825Profile& profile)
{
    const size_t expected =
        profile.preamble_repetitions * kQm35ChipsPerSymbol;
    const size_t ns_sfd_positive_shift = 8;
    const size_t sfd_len = kSfdSymbols * kQm35ChipsPerSymbol;
    size_t budget = expected + ns_sfd_positive_shift + sfd_len + 2;
    const size_t cap = legacy_chip_budget(profile);
    return std::min(budget, cap);
}

// ---------------------------------------------------------------------------
// Total chips needed once the PHR has been decoded:
//   sfd_end_chip + 1                      (PHR start)
//   + payload_chips_needed(psdu_length+2) (includes PHR + longest length
//                                          neighbourhood tried by
//                                          stage_payload_fcs)
//   + kFinalGuardChips
// capped by legacy_chip_budget().  sfd_end_chip < 0 returns 0, which always
// triggers the full legacy fallback (the PHR cannot be valid in that case).
//
// The +2 mirrors stage_payload_fcs(), which tries psdu_length, -1, +1, -2, +2
// and rejects candidates above profile.max_psdu_bytes.  Using the validated
// PHR length (never an injected/sidecar length) is mandatory.
//
// Default profile, 32-byte PHR: 36576 + 31232 + 64 = 67872 chips.
// Default profile, 16-byte PHR: 36576 + 23040 + 64 = 59680 chips (smaller
// than the 59741-chip prefix, so the extension is a no-op).
// Default profile, 127-byte PHR: 36576 + 88064 + 64 = 124704 chips.
// ---------------------------------------------------------------------------
inline size_t final_chip_budget(const Qm35825Profile& profile,
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

// ---------------------------------------------------------------------------
// Two-phase normalization.
//
// stage_cir_softchips() normalizes the real rotated chips by 1/(max|r|+eps)
// over the whole generated stream.  The on-demand path must decode NS-SFD and
// PHR before the payload range exists, so the prefix is normalized by its own
// visible maximum.  Once the extension has been generated, every chip must
// share one denominator: multiply the already-normalized prefix by
//
//     (prefix_max + eps) / (final_max + eps)
//
// and the extension chunk by 1/(final_max + eps).  This equals the one-shot
// raw/final_max normalization up to one float rounding per prefix sample.
//
// Contract: final_max >= prefix_max (the caller computes
// final_max = max(prefix_max, extension_max)).  If prefix_max == final_max
// the ratio is exactly 1.0f, so the prefix is left bit-identical; if the
// contract is violated the function is a no-op rather than an amplifier.
// The epsilon matches stage_cir_softchips() and is what makes the two-phase
// result equal the one-shot result including the +1e-12 denominator offset.
// ---------------------------------------------------------------------------
inline void rescale_prefix(std::vector<float>& chips,
                           size_t prefix_count,
                           float prefix_max,
                           float final_max)
{
    if (prefix_count == 0 || !(final_max > prefix_max))
        return;
    const float ratio = (prefix_max + kNormalizationEps) /
                        (final_max + kNormalizationEps);
    for (size_t i = 0; i < prefix_count; ++i)
        chips[i] *= ratio;
}

} // namespace rxopt
} // namespace demod
} // namespace uwb
} // namespace gr
