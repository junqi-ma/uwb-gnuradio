/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Fixed QM35825 PHY profile for the first-stage realtime demodulator.
 * Phase-1 supports ONLY this profile: fs 998.4 MHz, Mean PRF 62.4 MHz BPRF,
 * code index 9, 64 SYNC repetitions, SFD mode 4z2, 6.81 Mb/s.
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
    size_t cir_skip_initial_repetitions = 10; // skip the first N SYNC for CFO/CIR
    size_t cir_repetitions = 54;              // preamble - skip

    // timing validation
    double period_tolerance_pct = 2.0; // SYNC period deviation tolerance
    size_t min_valid_peaks = 8;        // min SYNC peaks to accept timing

    // SFD search window (half-width in chips around expected start)
    size_t sfd_search_half_width = 8;

    // soft-chip / CIR threshold
    float cir_detection_threshold = 0.3f;
    float sfd_detection_threshold = 0.3f;
    float radar_verification_threshold = 0.3f;

    // timing
    size_t timing_search_margin = 9984; // ~10 us @ 998.4e6
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
// 500 MHz bands; sequence S10), phase-aligned to the same cyclic origin that
// MATLAB lrwpan.internal.HRPCodes(9) / kPreambleCode9 use (shift 73 on the
// published table so code-9 matches bit-for-bit).  Non-zero count = 64,
// energy = 64 — same as code-9.
//
// STATUS: profile + self-consistency only.  Full golden cross-check against
// MATLAB UWB_demodulation requires exporting a code-10 window the same way
// as code-9 (see testdata/generate_and_export_golden.m with CodeIndex=10 on
// Windows MATLAB F:\MATLAB\bin\matlab.exe).  Do NOT claim code-10 is
// golden-verified until that export + stage-by-stage QA lands.
// ---------------------------------------------------------------------------
inline constexpr std::array<int8_t, kQm35CodeLength> kPreambleCode10 = { {
    0, -1, 0, 0, -1, -1, 0, 0, 0, -1, 0, 1, -1, 1, 0, -1, 0, 1, -1, 0, -1, 1,
    0, 0, 0, 0, 0, 1, -1, 0, 0, 1, 1, 0, -1, 0, 1, 0, 0, -1, -1, 1, 0, 0, 1,
    1, -1, 1, 0, 1, -1, 0, 1, 0, 0, 0, 0, -1, 0, -1, 0, -1, 0, -1, 1, 1, -1,
    1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 0, -1, 1, 0, 1, 1, 1, 0, 0, 0, -1, -1, -1,
    -1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 0, -1, -1, 1, 1, 0, 0, 1, 0, -1, 1, 0,
    0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, -1, 0, 0
} };

// Production DW1000 HRP code index 11, exported directly from MATLAB
// Communications Toolbox lrwpan.internal.HRPCodes(11).  The versioned source
// fixture is testdata/sic_profile_golden/dw1000_code11.csv.  Ternary length
// 127, non-zero count 64, energy 64.
inline constexpr std::array<int8_t, kQm35CodeLength> kPreambleCode11 = { {
    -1, 1, -1, 0, 0, 0, 0, 1, 0, 0, -1, -1, 0, 0, 0, 0, 0, -1, 0, 1, 0, 1,
    0, 1, -1, 0, 1, 0, 0, 1, 0, 0, 1, 0, -1, 0, 0, -1, 1, 1, 1, 0, 0, 1, 0,
    0, 0, -1, 1, 0, 1, 0, -1, 0, 0, 0, 0, 1, 1, 1, 1, 1, -1, 1, 0, 1, -1,
    -1, 0, 1, -1, 0, 1, 1, -1, -1, 0, -1, 0, 0, 0, 1, 0, -1, 1, 0, 0, 1, 0,
    1, -1, -1, -1, -1, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, -1, 1, 0, 0, 1, -1,
    0, 1, 1, 0, 0, 0, 1, 1, -1, 0, 0, 1, 1, -1, 0, -1, 0
} };

// Historical code-10/64-SYNC/4z2 communication fixture.  This is useful for
// synthetic collision QA only and must never be selected as production
// DW1000 SIC configuration.
struct SyntheticCode10Profile {
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

    double period_tolerance_pct = 2.0;
    size_t min_valid_peaks = 8;
    size_t sfd_search_half_width = 8;

    float cir_detection_threshold = 0.3f;
    float sfd_detection_threshold = 0.3f;
    float radar_verification_threshold = 0.3f;

    size_t timing_search_margin = 9984;
    size_t post_guard_samples = 4096;

    static SyntheticCode10Profile Default() { return SyntheticCode10Profile{}; }

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
        p.period_tolerance_pct = period_tolerance_pct;
        p.min_valid_peaks = min_valid_peaks;
        p.sfd_search_half_width = sfd_search_half_width;
        p.cir_detection_threshold = cir_detection_threshold;
        p.sfd_detection_threshold = sfd_detection_threshold;
        p.radar_verification_threshold = radar_verification_threshold;
        p.timing_search_margin = timing_search_margin;
        p.post_guard_samples = post_guard_samples;
        return p;
    }
};

// Production DW1000 BPRF profile used by Phase-2 SIC.  It shares the BPRF
// geometry/configuration adapter with the synthetic fixture, but replaces the
// PHY identity with the MATLAB-verified code-11 / 128-SYNC / Decawave DW-8
// values.  CIR repetitions begin at SYNC 11, matching the MATLAB pipeline.
struct Dw1000Profile : SyntheticCode10Profile {
    Dw1000Profile()
    {
        code_index = 11;
        preamble_repetitions = 128;
        sfd_mode = "decawave";
        cir_skip_initial_repetitions = 10;
        cir_repetitions = 118;
    }

    static Dw1000Profile Default() { return Dw1000Profile{}; }
};

// Returns the ternary Ipatov code for the given HRP code index.
// Supported: 9 (QM35825 golden), 10 (synthetic fixture), 11 (production
// DW1000).  Unknown indices fall
// back to code-9 with a stable pointer (callers that care must check).
inline const int8_t* GetPreambleCode(size_t code_index)
{
    if (code_index == 10)
        return kPreambleCode10.data();
    if (code_index == 11)
        return kPreambleCode11.data();
    return kPreambleCode9.data();
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
