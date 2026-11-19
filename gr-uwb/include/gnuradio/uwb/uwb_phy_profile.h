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
// First-SYNC search strategy (see Qm35825Profile::sync_strategy).
// ---------------------------------------------------------------------------
enum class SyncStrategy : int {
    AcquireLegacy = 0, // narrow seed refine -> stride coarse scan -> grid
                       // probes -> per-symbol backtrack (blind acquisition)
    LockedBounded = 1  // bounded ROI around the predicted origin only; an ROI
                       // miss is a tracking failure, never a wide scan
};

inline const char* sync_strategy_name(SyncStrategy s)
{
    switch (s) {
    case SyncStrategy::AcquireLegacy:
        return "acquire_legacy";
    case SyncStrategy::LockedBounded:
        return "locked_bounded";
    }
    return "unknown";
}

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
    // Applies to the initial acquisition fit.  The SFD-anchored refit uses
    // all measured tail peaks directly, independent of preamble length.
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
    // Experimental raw correlation PAR gates for the dual-metric searches.
    bool rx_sync_par_only = false;
    bool rx_sfd_par_only = false;
    // Validate short PAR-acquired SYNC trains and discrete-grid SFD candidates.
    bool rx_par_confirm = false;
    // Integrate the peak and its two adjacent sample lags for raw PAR.
    bool rx_sync_par_three_samples = false;
    bool rx_sfd_par_three_samples = false;

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
    // Timing backtrack limit (SYNC grid).  SFD itself now probes ±4 symbols
    // around preamble_start + 64×period with a ±64-sample narrow window.
    // Real QM35825 captures seed from an energy/schedule detector that can land
    // several SYNCs into the preamble (5-8 common); the previous 3 was too tight
    // and made otherwise-valid packets fail SFD.
    size_t timing_max_backtrack_symbols = 40;
    // ±sample half-width of the per-SYNC local refine in the track loop.
    size_t timing_track_radius = 8;
    size_t post_guard_samples = 4096;

    // ---- Explicit synchronization strategy -------------------------------
    // AcquireLegacy keeps the full acquisition behaviour: narrow seed refine,
    // then a stride coarse scan, whole-SYNC grid probes, and per-symbol
    // backtracking.  That is what a blind autocorrelation seed needs.
    //
    // LockedBounded is the locked-tracking contract: the packet origin is
    // already predicted by the target lock, so the first SYNC is searched only
    // inside ±locked_roi_samples of the seed.  It NEVER escalates to a wide
    // scan or a long backtrack; an ROI miss returns a tracking failure so the
    // lock state machine can widen the next window or reacquire.  The full
    // per-SYNC tracking train still runs, because CFO and CIR need valid
    // peaks.
    //
    // The default stays AcquireLegacy so every existing caller is unchanged.
    SyncStrategy sync_strategy = SyncStrategy::AcquireLegacy;
    // ± half-width (samples) of the locked ROI around the predicted origin.
    // 256 is the conservative first value; 64/128 are only valid after the
    // residual distribution has been measured on real captures.
    size_t locked_roi_samples = 256;
    // LockedBounded only: number of consecutive SYNC positions to probe inside
    // the ROI (seed + m*symbol, m = 0..n-1).  A real QM35825 has a startup
    // transient that can put SYNC #0 below the verification threshold; probing
    // a few inner SYNCs and converting back on the known grid keeps the packet
    // without ever running a wide scan.  0 disables the inner probes.
    size_t locked_inner_symbols = 4;
    // LockedBounded only: allow the per-symbol backtrack walk.  Default false:
    // backtracking is exactly how a lock can silently drift onto SYNC #2/#3.
    bool locked_allow_backtrack = false;
    // LockedBounded only: SYNC-train tracking strategy.
    //   true  (grid)   -- measure a sparse set of SYNC anchors with the normal
    //                     +/-track_radius search, fit the period from the
    //                     anchors, then evaluate every remaining SYNC with ONE
    //                     correlation at start + round(k*period).  The legacy
    //                     per-symbol search is 17 lags x 1016 taps per SYNC;
    //                     this is 1 lag x 1016 taps plus a local power sum.
    //   false (legacy) -- the original per-SYNC +/-track_radius search.
    // measured_period always comes from the anchor least-squares fit (never
    // from the nominal grid), because it feeds integer_chip_grid()/spc_ppm and
    // the fused-CMF admission.  A degenerate fit falls back to the legacy path.
    bool locked_grid_timing = true;
    // Anchor count for the grid fit (clamped to 4..32).  Anchors are spread
    // from the first confirmed SYNC to the last SYNC of the train.
    size_t locked_grid_anchor_count = 16;
    // Minimum anchors that must be found before the grid fit is trusted.
    size_t locked_grid_min_anchors = 4;
    // LockedBounded failure path only: when every inner-SYNC probe (m =
    // 0..locked_inner_symbols) misses, keep probing sparse SYNC anchors on the
    // known symbol grid before declaring a tracking failure.  A burst
    // interferer can sit on the first few SYNCs while the tail is clean; the
    // old code failed the packet without ever looking at the tail.  Normal
    // packets never execute this path.
    bool locked_tail_probe = true;
    // Symbol stride between tail probes (8 -> 32 probes over 256 SYNCs).
    size_t locked_tail_probe_step = 8;
    // A tail-probe hit must be confirmed by at least this many neighbouring
    // grid symbols (m-1, m+1, m+2) at the same threshold, otherwise it is a
    // +/-locked_roi_samples false alarm and probing continues.
    size_t locked_tail_probe_confirm = 2;

    // ---- Experimental RX fast paths (explicit opt-in) --------------------
    // Defaults keep the legacy algorithms so existing callers are unchanged.
    // The full-packet experiment wrapper selects these per run, allowing the
    // old and new paths to be timed from the same binary.
    bool rx_fast_sfd = false;            // cached SFD template + VOLK dot
    bool rx_fast_cfo_rotation = false;   // block-anchored vector derotation
    bool rx_seed_refine_tail = false;    // narrow first tail-SYNC search
    bool rx_softchip_on_demand = false;  // prefix chips + PHR-length extend
    // Experimental fused payload CMF + PN despread.  Requires the on-demand
    // prefix (SYNC/SFD/PHR) and the full integer-grid CMF; unsupported grids
    // keep the legacy path.  Default off.
    bool rx_fused_payload_cmf = false;
    // Experimental fused PHR CMF + PN despread.  Requires the payload fusion
    // and the on-demand prefix; it shortens the prefix to the SYNC/SFD/PHR
    // range.  Default off.
    bool rx_fused_phr_cmf = false;
    // Admission tolerance for the integer chip grid, in ppm of the sample
    // clock.  Both the AVX2 soft-chip kernel and the fused CMF index samples
    // on an exact kSamplesPerChip grid, so a clock offset that accumulates more
    // than this much drift across the longest frame (127 B = 154,624 samples)
    // is refused and the fractional/legacy path is used instead.
    //
    // The default is the physical ceiling, 1 sample of drift = 6.468 ppm
    // (demod::core::kIntegerGridCeilingPpm).  It is deliberately NOT tighter:
    // the 256-peak period estimator quantises samples_per_chip in ~2 ppm steps
    // (measured stdev 1.5-2.2 ppm), so a tight gate rejects packets whose true
    // clock is fine and pushes them onto the scalar fractional soft-chip
    // kernel, which is ~10x slower.  Measured on the X410 capture: a 1.5 ppm
    // gate refused 16% of packets and cost 15.3 ms/packet on the refused ones
    // (demod 1171 -> 3554 us).  Tighten it only when a better clock estimate
    // than the period fit is available.
    double fused_gate_ppm = 6.468;
    // NOTE: the diagnostic-only segmented soft-chip FIR replay
    // (rx_diagnose_fir_segments) is NOT part of this public production
    // profile any more; it moved to demod::core::DemodScratch and is driven
    // only by the experiment wrappers (apps/fullpacket_rx_pipeline.h).
    // First seeded SYNC refine half-width.  0 keeps the legacy
    // max(256, timing_coarse_stride) radius.  A nonzero value is used only
    // when the caller explicitly supplies it (the SFD-anchored tail profile)
    // and always falls back to the legacy radius if the narrow search misses.
    size_t timing_seed_refine_radius = 0;

    // ---- SFD-anchored CFO refit policy -----------------------------------
    // `demodulate_one` estimates the carrier offset twice: once from the
    // Stage-1 SYNC peak train (all repetitions, minus the startup exclusion)
    // and once from the 40-SYNC tail that the SFD anchor re-tracks.  The second
    // fit is followed by a full-frame derotation of the whole downstream span,
    // which is one of the largest single costs in the demodulator (measured
    // ~200 us/packet on the X410 1 ms captures).
    //   0 = Legacy: always refit and always run the second rotation.  This is
    //       the default and is byte-identical to the pre-existing behaviour.
    //   1 = Auto:   keep the Stage-2 estimate and skip the second rotation when
    //       (a) the two fits disagree by less than `cfo_refit_skip_rad` of
    //       accumulated phase over the downstream span, (b) the Stage-2 fit
    //       used at least `cfo_refit_min_peaks` peaks, and (c) the predicted
    //       single-pass rotation range provably covers the actual one.  Any
    //       failed condition falls back to the legacy path (fail-closed).
    //   2 = Skip:   same as Auto without (a)/(b); the range containment check
    //       is still enforced.
    int cfo_refit_mode = 0;
    // Auto only: maximum |2*pi*delta_f*span/fs| (rad) accepted for the skip.
    double cfo_refit_skip_rad = 0.10;
    // Auto only: minimum Stage-2 peaks for the skip.
    size_t cfo_refit_min_peaks = 64;

    // Early termination: stop receiver processing right after Final SFD is verified
    // and preamble_start_sample / CFO are anchored. Skips CIR, PHR, and Payload.
    bool rx_stop_after_sfd = false;

    // Early termination: stop receiver processing right after standalone CIR
    // estimation completes (after Final SFD). Skips soft-chip CMF, PHR, and Payload.
    bool rx_stop_after_cir = false;

    // Use the 64-entry sparse ternary preamble code (same template as CIR
    // estimation) for timing/SFD/fold matched filtering instead of the dense
    // pulse-shaped template (~7x faster demod on wide ROIs).
    // TODO(matlab-verify): UWB_demodulation/+uwbdecoder/detectRepeatedPreamble.m
    // uses the pulse-shaped dense preamble_waveform for low-SNR (-12/-14 dB)
    // detection and MATLAB golden QA expects dense metric >= 0.95 and its
    // 2-sample group-delay alignment, so Qm35825Profile keeps false by default
    // while uwb_dw3000_live defaults --timing-sparse on. Cross-check sparse
    // vs dense SNR threshold and group-delay offset in MATLAB before any
    // global switch or dense-path removal.
    bool rx_timing_sparse = false;

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
