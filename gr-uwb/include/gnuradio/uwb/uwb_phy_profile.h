/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Default QM35825 PHY profile for the first-stage realtime demodulator.
 * The realtime path uses the same 998.4 MHz / 62.4 MHz BPRF geometry for
 * selectable HRP code indices 9–12; the defaults remain code 9, 64 SYNC
 * repetitions, SFD mode 4z2, and 6.81 Mb/s.
 *
 * Matches UWB_demodulation/run_decode_uwb_radar.m + defaultOptions + the
 * verified golden reference (testdata/realtime_demod_golden/manifest.csv):
 *   preamble_repetitions = 64, cir_skip_initial_repetitions = 10
 *   cir_repetitions = preamble - skip = 54
 *   samples_per_symbol = 1016, samples_per_chip = 2
 *   chips_per_symbol = 508   (= 127-chip Ipatov code x 4 spreading)
 *   HRPCodes(9) is the 127-long ternary IEEE 802.15.4a code-9 sequence.
 */

#pragma once

#include <array>
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace gr {
namespace uwb {
namespace demod {

enum class CirSoftChipMode : uint8_t {
    Auto = 0,   // full for K=0, sparse RAKE for K>0 (legacy behavior)
    Full = 1,   // complete CIR matched filter
    Rake = 2,   // sparse Top-K coherent combining
    Bypass = 3, // strongest-path chip sampling, no channel matched filter
};

// ---------------------------------------------------------------------------
// IEEE 802.15.4a/z HRP constants (QM35825 radar, BPRF 62.4 MHz)
// ---------------------------------------------------------------------------
inline constexpr double kQm35SampleRate = 998400000.0;
inline constexpr double kQm35MeanPrfMHz = 62.4;
inline constexpr size_t kQm35SamplesPerSymbol = 1016;
inline constexpr size_t kQm35SamplesPerChip = 2;   // SamplesPerPulse = 2
inline constexpr size_t kQm35ChipsPerSymbol = 508; // 1016 / 2
inline constexpr size_t kQm35SpreadingFactor = 4;  // PreambleSpreadingFactor
inline constexpr size_t kQm35CodeLength = 127;     // HRP code-9 Ipatov length

// ---------------------------------------------------------------------------
// Fixed QM35825 radar profile (Phase-1 only).
// ---------------------------------------------------------------------------
struct Qm35825Profile {
    double sample_rate = kQm35SampleRate;
    double mean_prf_mhz = kQm35MeanPrfMHz;
    size_t code_index = 9;
    size_t preamble_repetitions = 64;
    const char* sfd_mode = "4z2";
    double data_rate_mbps = 6.81;
    size_t max_psdu_bytes = 127;

    // CIR estimation window (from defaultOptions)
    size_t cir_pre_samples = 8;
    size_t cir_post_samples = 30;
    size_t cir_skip_initial_repetitions = 10; // skip the first N SYNC for CIR
    size_t cir_repetitions = 54;              // preamble - skip
    // Match MATLAB compensateCarrierOffset: discard the startup transient in
    // at most the first 24 SYNCs, while retaining at least 32 phase samples.
    // This is intentionally separate from the CIR skip policy above.
    size_t cfo_skip_initial_repetitions = 24;
    size_t cfo_min_fit_repetitions = 32;
    // Sparse RAKE policy: 0 keeps the full CIR matched filter; otherwise use
    // only the K strongest complex CIR taps (clamped to the tap count).
    size_t cir_rake_top_k = 0;
    CirSoftChipMode cir_soft_chip_mode = CirSoftChipMode::Auto;

    // timing validation
    double period_tolerance_pct = 2.0; // SYNC period deviation tolerance
    size_t min_valid_peaks = 8;        // min SYNC peaks to accept timing

    // soft-chip / CIR threshold
    float cir_detection_threshold = 0.3f;
    float sfd_detection_threshold = 0.3f;
    float radar_verification_threshold = 0.25f;

    // timing
    // Seed (schedule t0+kT or detector) can sit hundreds–thousands of samples
    // off the true SYNC grid when the nominal period has ~ppm-level SFO.  A
    // ±10 us margin is not enough for a 0.5 s capture; ~40 us (~40k samples)
    // covers the residual after schedule lock and still fits typical pre_guard.
    size_t timing_search_margin = 40960; // ~41 us @ 998.4e6
    // Coarse preamble acquisition stride (samples).  Fine refine uses ±stride.
    // QM35 SC16 stride sweep: 14 was the fastest tested zero-miss setting;
    // 16 missed 13/99 scheduled packets due to phase aliasing.
    size_t timing_coarse_stride = 14;
    // Backtrack limit also bounds the SFD early-window scan (stage_sfd), so a
    // seed landing anywhere within the first N preamble SYNCs still decodes.
    // Real QM35825 captures seed from an energy/schedule detector that can land
    // several SYNCs into the preamble (5-8 common); the previous 3 was too tight
    // and made otherwise-valid packets fail SFD.
    size_t timing_max_backtrack_symbols = 40;
    // ±sample half-width of the per-SYNC local refine in the track loop.
    size_t timing_track_radius = 8;
    size_t post_guard_samples = 4096;

    static Qm35825Profile Default() { return Qm35825Profile{}; }
};

// ---------------------------------------------------------------------------
// SFD sequences (from +uwbdecoder/defaultOptions.m). Each is a ternary vector
// in {-1, 0, +1}. Phase-1 uses kSfd4z2; the others are kept for completeness.
// ---------------------------------------------------------------------------
inline const std::array<int8_t, 8> kSfdDecawave = { -1, -1, -1, -1, 1, -1, 0, 0 };
inline const std::array<int8_t, 8> kSfdIeee = { 0, 1, 0, -1, 1, 0, 0, -1 };
inline const std::array<int8_t, 4> kSfd4z1 = { -1, -1, 1, -1 };
inline const std::array<int8_t, 8> kSfd4z2 = { -1, -1, -1, 1, -1, -1, 1, -1 };
inline const std::array<int8_t, 16> kSfd4z3 = { -1, -1, -1, -1, -1, 1, 1, -1,
                                                -1, 1, -1, 1, -1, -1, 1, -1 };
inline const std::array<int8_t, 32> kSfd4z4 = { -1, -1, -1, -1, -1, -1, -1, 1,
                                                -1, -1, 1, -1, -1, 1, -1, 1,
                                                -1, 1, -1, -1, -1, 1, 1, -1,
                                                -1, -1, 1, -1, 1, 1, -1, -1 };

inline std::vector<int8_t> GetSfdSequence(const char* sfd_mode)
{
    if (!sfd_mode)
        return {};
    if (std::string(sfd_mode) == "decawave")
        return { kSfdDecawave.begin(), kSfdDecawave.end() };
    if (std::string(sfd_mode) == "ieee")
        return { kSfdIeee.begin(), kSfdIeee.end() };
    if (std::string(sfd_mode) == "4z1")
        return { kSfd4z1.begin(), kSfd4z1.end() };
    if (std::string(sfd_mode) == "4z2")
        return { kSfd4z2.begin(), kSfd4z2.end() };
    if (std::string(sfd_mode) == "4z3")
        return { kSfd4z3.begin(), kSfd4z3.end() };
    if (std::string(sfd_mode) == "4z4")
        return { kSfd4z4.begin(), kSfd4z4.end() };
    return {};
}

// ---------------------------------------------------------------------------
// IEEE 802.15.4a HRP code-9 spreading sequence (Ipatov 127, from MATLAB
// lrwpan.internal.HRPCodes(9)).  Ternary {-1,0,+1}.  The de-spread chip
// stream is built by spreading this code by kQm35SpreadingFactor=4 then
// kQm35SamplesPerChip=2, giving kQm35ChipsPerSymbol=508 sampled chips and
// kQm35SamplesPerSymbol=1016 waveform samples.
// ---------------------------------------------------------------------------
inline constexpr std::array<int8_t, kQm35CodeLength> kPreambleCode9 = { {
    1, 0, 0, 1, 0, 0, 0, -1, 0, -1, -1, 0, 0, -1, -1, 1, 0, 1, 0, 1, 0, 0,
    -1, 1, -1, 1, 1, 0, 1, 0, 0, 0, 0, 1, 1, -1, 0, 0, 0, 1, 0, 0, -1, 0,
    0, -1, -1, 0, -1, 1, 0, 1, 0, -1, -1, 0, -1, 1, 1, 1, 0, 1, 1, 0, 0, 0,
    1, -1, 0, 1, 0, 0, -1, 0, 1, 1, -1, 0, 1, 1, 1, 0, 0, -1, 1, 0, 0, 1,
    0, 1, 0, -1, 0, 1, 1, -1, 1, -1, -1, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
    0, 0, -1, 1, 0, 0, 0, 0, -1, 0, -1, 0, 0, 0, -1, -1, 1
} };

// ---------------------------------------------------------------------------
// IEEE 802.15.4a-2007 HRP code index 10 (Ipatov length 127, ternary {-1,0,+1}).
// Source: IEEE 802.15 contrib 15-05-0737-01-004a (length-127 preamble set for
// 500 MHz bands; sequence S10), cyclic origin phase-aligned to MATLAB
// lrwpan.internal.HRPCodes(10) / testdata/uwb_code10_preamble16_payload8.cfile.
//
// History: an earlier draft used the same published-table shift that makes
// code-9 match HRPCodes(9), but code-10 required an additional +23 cyclic
// roll to match HRPCodes(10) and the MATLAB-generated code-10 waveform
// (sparse-grid correlation ≈0.999 at sample phase 2; uncorrected origin
// collapsed CIR/soft-chip NS-SFD to noise on real DW1000 captures).
// Non-zero count = 64, energy = 64 — same as code-9.
// ---------------------------------------------------------------------------
inline constexpr std::array<int8_t, kQm35CodeLength> kPreambleCode10 = { {
    1, 1, 0, 0, 1, 0, -1, 1, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, -1, 0,
    0, 0, -1, 0, 0, -1, -1, 0, 0, 0, -1, 0, 1, -1, 1, 0, -1, 0, 1, -1, 0, -1,
    1, 0, 0, 0, 0, 0, 1, -1, 0, 0, 1, 1, 0, -1, 0, 1, 0, 0, -1, -1, 1, 0,
    0, 1, 1, -1, 1, 0, 1, -1, 0, 1, 0, 0, 0, 0, -1, 0, -1, 0, -1, 0, -1, 1,
    1, -1, 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 0, -1, 1, 0, 1, 1, 1, 0, 0, 0,
    -1, -1, -1, -1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 0, -1, -1
} };

// MATLAB R2025b lrwpan.internal.HRPCodes(11), exported on 2026-08-12.
inline constexpr std::array<int8_t, kQm35CodeLength> kPreambleCode11 = { {
    -1, 1, -1, 0, 0, 0, 0, 1, 0, 0, -1, -1, 0, 0, 0, 0,
    0, -1, 0, 1, 0, 1, 0, 1, -1, 0, 1, 0, 0, 1, 0, 0,
    1, 0, -1, 0, 0, -1, 1, 1, 1, 0, 0, 1, 0, 0, 0, -1,
    1, 0, 1, 0, -1, 0, 0, 0, 0, 1, 1, 1, 1, 1, -1, 1,
    0, 1, -1, -1, 0, 1, -1, 0, 1, 1, -1, -1, 0, -1, 0, 0,
    0, 1, 0, -1, 1, 0, 0, 1, 0, 1, -1, -1, -1, -1, 0, 0,
    0, -1, 0, 0, 0, 0, 0, 0, -1, 1, 0, 0, 1, -1, 0, 1,
    1, 0, 0, 0, 1, 1, -1, 0, 0, 1, 1, -1, 0, -1, 0
} };

// MATLAB R2025b lrwpan.internal.HRPCodes(12), exported on 2026-08-12.
inline constexpr std::array<int8_t, kQm35CodeLength> kPreambleCode12 = { {
    -1, 1, 0, 1, 1, 0, 0, 0, 0, 0, 0, -1, 0, 1, 0, -1,
    1, 0, -1, -1, -1, 1, -1, 1, 1, 0, 0, -1, 1, 0, 1, 1,
    0, 1, 0, 1, 0, 1, 0, 0, 0, -1, 0, 0, -1, 0, 0, -1,
    1, 0, 0, 1, -1, 1, 1, 0, 0, 0, -1, 1, -1, 0, -1, 1,
    1, 0, -1, 0, 1, 1, 1, 1, 0, -1, 0, 0, -1, 0, 1, 1,
    0, 0, 1, 0, 1, 0, 0, 1, 1, -1, 0, 0, 1, 0, 0, 0,
    1, -1, 0, 0, 0, -1, 0, -1, -1, 1, 0, 0, 0, 0, -1, 0,
    0, 0, 0, -1, -1, 0, 1, 0, 0, 0, 0, 0, 1, -1, -1
} };

// DW1000-mode BPRF profile (default code-10, 64 SYNC).  Same sample-rate /
// timing geometry as Qm35825Profile::Default(); only code_index and
// documentation differ. Reuses BuildSampledCode / GetSfdSequence.
struct Dw1000Profile {
    double sample_rate = kQm35SampleRate;
    double mean_prf_mhz = kQm35MeanPrfMHz;
    size_t code_index = 10;
    size_t preamble_repetitions = 64;
    const char* sfd_mode = "4z2";
    double data_rate_mbps = 6.81;
    size_t max_psdu_bytes = 127;

    size_t cir_pre_samples = 8;
    size_t cir_post_samples = 30;
    size_t cir_skip_initial_repetitions = 10;
    size_t cir_repetitions = 54;
    size_t cfo_skip_initial_repetitions = 24;
    size_t cfo_min_fit_repetitions = 32;
    size_t cir_rake_top_k = 0;
    CirSoftChipMode cir_soft_chip_mode = CirSoftChipMode::Auto;

    double period_tolerance_pct = 2.0;
    size_t min_valid_peaks = 8;
    float cir_detection_threshold = 0.3f;
    float sfd_detection_threshold = 0.3f;
    float radar_verification_threshold = 0.25f;

    size_t timing_search_margin = 40960;
    size_t timing_coarse_stride = 16;
    size_t timing_max_backtrack_symbols = 40;
    // ±sample half-width of the per-SYNC local refine in the track loop.
    size_t timing_track_radius = 8;
    size_t post_guard_samples = 4096;

    static Dw1000Profile Default() { return Dw1000Profile{}; }

    // Convert to the Qm35825Profile layout used by demodulate_one / stages.
    Qm35825Profile as_qm35825() const
    {
        Qm35825Profile p;
        p.sample_rate = sample_rate;
        p.mean_prf_mhz = mean_prf_mhz;
        p.code_index = code_index;
        p.preamble_repetitions = preamble_repetitions;
        p.sfd_mode = sfd_mode;
        p.data_rate_mbps = data_rate_mbps;
        p.max_psdu_bytes = max_psdu_bytes;
        p.cir_pre_samples = cir_pre_samples;
        p.cir_post_samples = cir_post_samples;
        p.cir_skip_initial_repetitions = cir_skip_initial_repetitions;
        p.cir_repetitions = cir_repetitions;
        p.cfo_skip_initial_repetitions = cfo_skip_initial_repetitions;
        p.cfo_min_fit_repetitions = cfo_min_fit_repetitions;
        p.period_tolerance_pct = period_tolerance_pct;
        p.min_valid_peaks = min_valid_peaks;
        p.cir_detection_threshold = cir_detection_threshold;
        p.sfd_detection_threshold = sfd_detection_threshold;
        p.radar_verification_threshold = radar_verification_threshold;
        p.timing_search_margin = timing_search_margin;
        p.timing_coarse_stride = timing_coarse_stride;
        p.timing_max_backtrack_symbols = timing_max_backtrack_symbols;
        p.timing_track_radius = timing_track_radius;
        p.post_guard_samples = post_guard_samples;
        return p;
    }
};

// Returns the ternary Ipatov code for the given HRP code index.
// Supported: 9 (QM35825), 10–12 (common DW1000 HRP codes). Unknown indices
// fall back to code-9 with a stable pointer (public construction validates).
inline const int8_t* GetPreambleCode(size_t code_index)
{
    switch (code_index) {
    case 10: return kPreambleCode10.data();
    case 11: return kPreambleCode11.data();
    case 12: return kPreambleCode12.data();
    default: return kPreambleCode9.data();
    }
}

// ---------------------------------------------------------------------------
// Build the sampled chip stream for a preamble code: spread by
// kQm35SpreadingFactor, then upsample by kQm35SamplesPerChip.  Returns
// kQm35ChipsPerSymbol (508) entries, matching MATLAB buildUwbReference's
// `sampled_code`.  Used by CIR estimation / de-spreading.
// ---------------------------------------------------------------------------
inline std::vector<int8_t> BuildSampledCode(const int8_t* code, size_t code_len)
{
    std::vector<int8_t> sampled(kQm35ChipsPerSymbol, 0);
    // spread code: non-zero every kQm35SpreadingFactor-th chip
    // sampled: place each spread chip at kQm35SamplesPerChip grid
    for (size_t c = 0; c < code_len; ++c) {
        const size_t spread_pos = c * kQm35SpreadingFactor; // 0..507
        if (spread_pos < kQm35ChipsPerSymbol)
            sampled[spread_pos] = code[c];
    }
    return sampled;
}

} // namespace demod
} // namespace uwb
} // namespace gr
