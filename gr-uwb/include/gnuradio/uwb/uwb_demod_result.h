/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Result schema for the UWB realtime demodulator.  Mirrors the per-stage
 * outputs of UWB_demodulation/decode_uwb.m so the C++ and MATLAB paths can be
 * compared field-by-field.
 *
 * Coordinate convention: all sample indices are 0-based absolute (matching the
 * scheduled extractor / detector PDU metadata).  Where a stage reports a
 * position relative to a cropped window, both the cropped-relative and the
 * absolute value are stored.
 */

#pragma once

#include <gnuradio/uwb/uwb_phy_profile.h>

#include <complex>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace gr {
namespace uwb {
namespace demod {

// ---------------------------------------------------------------------------
// Per-stage status.  Each job emits exactly one of these in its result.
// ---------------------------------------------------------------------------
enum class DemodStatus : int {
    Success = 0,
    InvalidInput = 1,   // bad PDU / missing metadata / too short
    TimingFailed = 2,   // preamble detection / SYNC tracking failed
    CfoFailed = 3,      // CFO estimation diverged
    SfdFailed = 4,      // SFD not found in soft-chip stream
    PhrFailed = 5,      // PHR decode / SECDED / length check failed
    PayloadFailed = 6,  // payload decode error
    FcsFailed = 7,      // CRC mismatch (valid decode, bad FCS)
    QueueFull = 8,      // job dropped: demod queue saturated
    InternalError = 9,  // unexpected exception in worker
    CirFailed = 10      // CIR / soft-chip generation failed
};

// ---------------------------------------------------------------------------
// Stage 1 — timing: seeded / adaptive preamble detection + SYNC tracking.
// ---------------------------------------------------------------------------
struct TimingResult {
    bool ok = false;
    int64_t preamble_start_sample = -1; // absolute, 0-based
    int64_t preamble_start_uncropped = -1; // before cropToFrame shift
    double measured_period = 0.0;          // mean SYNC period in samples
    float metric = 0.0f;                   // correlation metric at start
    size_t detected_peaks = 0;             // SYNC repetitions actually found
    size_t expected_peaks = 0;             // from profile
    std::vector<int64_t> peak_samples;    // absolute position of each SYNC peak
    std::vector<float> peak_metrics;       // per-peak correlation metric
    // Per-peak SYNC matched-filter complex value (phase = CFO-bearing carrier
    // phase at that peak).  Same length as peak_samples; the CFO stage uses
    // arg() of these rather than the raw rx[peak] sample.
    std::vector<std::complex<float>> peak_corr;
    // Initial seeded acquisition profile: stride-S coarse scan vs all other
    // timing work (setup, local refine, grid probes and SYNC tracking).
    uint64_t coarse_search_us = 0;
    uint64_t fine_track_us = 0;
    // Experimental narrow seed search (timing_seed_refine_radius != 0).
    // seed_search_us is the first seeded refine (child of fine_track_us, not
    // included in coarse_search_us).  Attempts/success/fallbacks count one
    // seeded call each, not SYNC symbols.
    uint64_t seed_search_us = 0;
    uint32_t seed_refine_narrow_attempts = 0;
    uint32_t seed_refine_narrow_success = 0;
    uint32_t seed_refine_fallbacks = 0;
    // LockedBounded strategy diagnostics.  `bounded_legacy_scans` counts any
    // escalation to the stride coarse scan / whole-SYNC grid probe / long
    // backtrack; it MUST stay 0 under SyncStrategy::LockedBounded, and a
    // nonzero value is a contract violation, not a tuning result.
    uint32_t bounded_attempts = 0;
    uint32_t bounded_success = 0;
    uint32_t bounded_failures = 0;
    uint32_t bounded_legacy_scans = 0;
    uint32_t bounded_inner_used = 0; // recovered from an inner SYNC, not SYNC #0
    size_t bounded_roi_samples = 0;
    uint32_t backtrack_symbols = 0;
};

// ---------------------------------------------------------------------------
// Stage 2 — CFO estimation + compensation.
// ---------------------------------------------------------------------------
struct CfoResult {
    bool ok = false;
    double cfo_hz = 0.0;            // estimated carrier frequency offset
    double residual_phase = 0.0;    // constant phase after derotation
    size_t peaks_used = 0;          // stable peaks used in the linear fit
    size_t skipped_peaks = 0;       // unstable leading SYNCs excluded
    // Peak coordinates used by the CFO fit.  These are retained separately
    // from TimingResult because SFD refinement later updates the public
    // preamble origin, whereas CFO must be diagnosable against its original
    // matched-filter peak train.
    int64_t fit_first_peak_sample = -1;
    int64_t fit_last_peak_sample = -1;
    float fit_residual = 0.0f;      // residual of the phase linear fit
    // Rotation-kernel diagnostics: fit vs frame derotation are independent.
    uint64_t fit_us = 0;
    uint64_t rotate_us = 0;
    uint32_t rotation_kernel = 0;   // 0=legacy four-chain, 1=block vector
};

// ---------------------------------------------------------------------------
// Stage 3 — SFD template selection + timing refinement.
// ---------------------------------------------------------------------------
struct SfdResult {
    bool ok = false;
    const char* sfd_mode = nullptr;     // selected SFD mode (e.g. "4z2")
    int64_t sfd_start_sample = -1;      // absolute, full-rate refined
    int64_t sfd_end_sample = -1;        // absolute
    // SFD start predicted directly from the timing stage's final tracked SYNC:
    // last_sync_start + measured_period, before any SFD correlation/refinement.
    int64_t expected_start_sample = -1;
    int64_t sfd_start_chip = -1;        // chip-index in soft-chip stream
    int polarity = 0;                   // +/-1 soft-chip polarity
    float metric = 0.0f;                // SFD correlation metric
    // Exact count of normalized complex SFD correlations evaluated.  Coarse
    // and fine count candidate positions (including any deliberate overlap).
    uint64_t search_windows = 0;
    uint64_t coarse_correlations = 0;
    uint64_t fine_correlations = 0;
    // First candidate window (0=initial prediction, 1=one-symbol backtrack)
    // whose local maximum passes the SFD threshold. Diagnostic only: the
    // current algorithm still evaluates all candidates and selects global max.
    int64_t first_threshold_backtrack_symbols = -1;
};

// ---------------------------------------------------------------------------
// Stage 4 — CIR estimation + soft-chip generation.
// ---------------------------------------------------------------------------
struct CirResult {
    bool ok = false;
    size_t first_path_sample = 0;       // CIR peak position (samples)
    size_t pre_samples = 0;             // CIR window before first path
    size_t post_samples = 0;            // CIR window after first path
    float cir_peak_metric = 0.0f;       // peak CIR magnitude
    std::vector<float> cir_values;      // CIR tap magnitudes (diagnostics only)
    // Pre-conjugated CMF coefficients h[q] = conj(values[q]) exactly as used by
    // the soft-chip FIR.  Populated for the experimental fused payload path and
    // for diagnostics; the default path does not read it.
    std::vector<std::complex<float>> cir_filter_taps;
    size_t soft_chip_count = 0;         // number of soft chips produced
    double samples_per_chip = 0.0;      // measured chip period

    // Sub-stage wall-clock timing (µs), so CIR estimate vs soft-chip FIR vs
    // postprocess can be ranked separately (P0 diagnostics).
    uint64_t cir_estimate_us = 0;       // spread build + SYNC average + taps
    uint32_t soft_fir_first_chip = 0;   // absolute coordinate; skipped prefix is zero
    uint32_t soft_fir_computed_chips = 0;
    uint64_t sync_phase_us = 0;         // child of postprocess_us
    uint64_t soft_fir_us = 0;           // chip-grid matched-filter FIR
    uint64_t postprocess_us = 0;        // phase align + rotate + real/max/norm
    // On-demand soft-chip experiment: initial prefix vs extension.  Zero
    // values mean the legacy full-stream path was used.
    uint64_t soft_extend_us = 0;        // extension FIR + renormalization
    uint32_t initial_chip_count = 0;    // prefix chips before PHR decode
    uint32_t extended_chip_count = 0;   // total chips after PHR-driven extend
    uint32_t on_demand_extended = 0;    // 1 if the stream was extended
    uint32_t on_demand_fallbacks = 0;   // full-path retries for this packet
};

// ---------------------------------------------------------------------------
// Stage 5 — NS-SFD location in soft-chip stream.
// ---------------------------------------------------------------------------
struct NsSfdResult {
    bool ok = false;
    int64_t sfd_start_chip = -1;        // chip index of SFD start
    int64_t sfd_end_chip = -1;          // chip index of SFD end
    int polarity = 0;
    float metric = 0.0f;
};

// ---------------------------------------------------------------------------
// Stage 6 — BPRF PHR decode.
// ---------------------------------------------------------------------------
struct PhrResult {
    bool ok = false;
    bool secded_corrected = false;      // single-bit error corrected
    bool secded_uncorrectable = false;  // double-bit error detected
    uint32_t psdu_length = 0;           // decoded PSDU length in bytes
    float data_rate_mbps = 0.0f;        // decoded data rate
    std::vector<uint8_t> phr_bits;      // 19 decoded PHR bits (diagnostics)
};

// ---------------------------------------------------------------------------
// Stage 7 — payload + FCS.
// ---------------------------------------------------------------------------
struct PayloadResult {
    bool ok = false;
    bool fcs_pass = false;
    uint16_t received_fcs = 0;
    uint16_t calculated_fcs = 0;
    std::vector<uint8_t> bytes;         // decoded PSDU bytes
    std::vector<uint8_t> bits;          // raw payload bits (diagnostics)
};

// ---------------------------------------------------------------------------
// Diagnostic CHILD sub-timers of `stage_payload_us`.
//
// The five microsecond buckets are mutually exclusive and additive: they
// partition the interval measured *inside* stage_payload_fcs() /
// stage_payload_fused_cmf(), and `other_us` is computed as the exact remainder
// after the four named buckets have been subtracted.  Therefore
//
//     soft_us + viterbi_us + rs_us + crc_us + other_us
//         == in-function payload interval
//         <= stage_payload_us
//
// and the only difference to `stage_payload_us` is the call/lap overhead
// (a few µs at most).  They are published for diagnostics only and must NEVER
// be added to `stage_total_us`: the parent lap already contains them.
// ---------------------------------------------------------------------------
struct PayloadSubTimers {
    uint64_t soft_us = 0;      // BPM metric generation: bprf_demod[_soft] and
                               // the fused CMF PN build + four-candidate
                               // correlation
    uint64_t viterbi_us = 0;   // vitdec_cl3 / vitdec_cl3_soft (joint PHR+payload)
    uint64_t rs_us = 0;        // rs_decode_stream
    uint64_t crc_us = 0;       // PSDU bit packing + crc16_802154 + FCS compare
    uint64_t other_us = 0;     // remainder: admission checks, buffer assembly
    // Diagnostic CHILD of `other_us` (NOT a sixth additive bucket): the
    // payload-region CFO derotation that runs inside the stage_payload_us lap
    // but outside the payload decoder.  `other_us - other_derotate_us` is the
    // true "everything else" remainder.
    uint64_t other_derotate_us = 0;
    uint32_t fused_calls = 0;  // stage_payload_fused_cmf invocations
    uint32_t legacy_calls = 0; // stage_payload_fcs invocations
};

// ---------------------------------------------------------------------------
// Per-job result: one per input PDU.  Any stage failure still emits a full
// result with ok=false up to the failing stage.
// ---------------------------------------------------------------------------
struct DemodResult {
    DemodStatus status = DemodStatus::Success;

    // input metadata (copied from the source PDU)
    uint64_t packet_id = 0;
    uint64_t schedule_index = 0;
    int64_t predicted_start_sample = -1;
    int64_t detected_start_sample = -1;
    int64_t window_start_sample = -1;
    size_t input_sample_count = 0;
    // Synchronization strategy actually used for this packet.
    SyncStrategy sync_strategy = SyncStrategy::AcquireLegacy;

    // per-stage outputs
    TimingResult timing;
    CfoResult cfo;
    SfdResult sfd;
    CirResult cir;
    NsSfdResult ns_sfd;
    PhrResult phr;
    PayloadResult payload;

    // worker / queue diagnostics
    uint32_t worker_id = 0;
    uint64_t queue_delay_us = 0;        // time spent waiting in job queue
    uint64_t demod_latency_us = 0;      // total worker processing time
    uint64_t wall_latency_us = 0;       // end-to-end from PDU arrival

    // per-stage wall-clock timings (µs), recorded by demodulate_one.
    // Only stages actually run are non-zero; on early failure the failing
    // stage gets the elapsed time since the previous completed stage.
    uint64_t stage_timing_us = 0;
    uint64_t stage_timing_coarse_us = 0;
    uint64_t stage_timing_fine_track_us = 0;
    uint64_t stage_cfo_us = 0;
    uint64_t stage_sfd_us = 0;
    uint64_t stage_cir_us = 0;
    uint64_t stage_ns_sfd_us = 0;
    uint64_t stage_phr_us = 0;
    uint64_t stage_payload_us = 0;
    uint64_t stage_total_us = 0;        // sum of parents + fallback bucket
    // Diagnostic child sub-timers for the CFO and soft-chip stages.
    // stage_cfo_fit_us/stage_cfo_rotate_us are already included in their parent
    // stage above and must never be added to stage_total_us.  They sum BOTH CFO
    // calls (initial + SFD-refit) unless noted otherwise.
    // stage_softchip_fallback_us is different: it is NOT part of stage_cir_us;
    // it is a separate eighth bucket that is included in stage_total_us.
    uint64_t stage_cfo_fit_us = 0;
    uint64_t stage_cfo_rotate_us = 0;
    uint32_t stage_cfo_rotation_kernels = 0; // bit0=initial block, bit1=refit block
    uint64_t stage_cfo_rotated_samples = 0; // total samples derotated across passes
    uint32_t stage_cfo_rotation_passes = 0;  // count of derotate_cfo_range invocations
    uint32_t stage_softchip_fallbacks = 0;   // full-path retries (diagnostic)
    uint64_t stage_softchip_fallback_us = 0; // separate total bucket (not a child)
    // Experimental fused payload CMF/despread path.  The two sub-timers are
    // CHILDREN of `stage_payload_us` and must never be added to the total.
    // `stage_fused_fallbacks` counts a fused attempt that failed and then ran
    // the full legacy stream once; that retry's wall time is recorded in the
    // existing `stage_softchip_fallback_us` bucket (also not a child of CIR).
    // Integer chip-grid admission refused this packet (see
    // Qm35825Profile::fused_gate_ppm).  When the fused path was requested the
    // soft-chip mode is downgraded Full->Bypass for this packet: the full
    // 38-tap FIR over the whole payload only exists to feed the fused CMF, so
    // running it after the fusion was refused costs ~15 ms for nothing.
    uint32_t stage_grid_rejects = 0;
    uint32_t stage_fused_attempts = 0;   // kernels actually run
    uint32_t stage_fused_fallbacks = 0;  // fused attempt -> full legacy retry
    uint32_t stage_fused_symbols = 0;    // BPRF payload symbols correlated
    uint64_t stage_fused_template_us = 0; // per-packet template build
    uint64_t stage_fused_correlate_us = 0; // four-candidate correlations
    uint64_t stage_fused_decode_us = 0;  // PHR cache + BPM/FEC/FCS decode
    uint64_t stage_fused_decode_ns = 0;  // same interval, nanosecond resolution
    // Diagnostic CHILD sub-timers of `stage_payload_us` (see PayloadSubTimers).
    // `stage_payload_other_us` is the exact remainder, so
    // soft+viterbi+rs+crc+other equals the in-function payload interval and the
    // gap to `stage_payload_us` is only call overhead.  These are NEVER added to
    // `stage_total_us`: the parent lap already contains them.
    // `stage_payload_fused_calls` / `stage_payload_legacy_calls` count how many
    // times each payload backend ran, i.e. the coverage of the sub-timers.
    uint64_t stage_payload_soft_us = 0;
    uint64_t stage_payload_viterbi_us = 0;
    uint64_t stage_payload_rs_us = 0;
    uint64_t stage_payload_crc_us = 0;
    uint64_t stage_payload_other_us = 0;
    // CHILD of `stage_payload_other_us` (not a sixth additive bucket): the
    // payload-region CFO derotation.  `stage_payload_other_us -
    // stage_payload_derotate_us` is the true "everything else" remainder.
    uint64_t stage_payload_derotate_us = 0;
    uint32_t stage_payload_fused_calls = 0;
    uint32_t stage_payload_legacy_calls = 0;
    // Experimental fused PHR CMF/despread path.  These sub-timers are
    // CHILDREN of `stage_phr_us`, like the payload ones above.
    uint32_t stage_phr_fused_attempts = 0;
    uint32_t stage_phr_fused_fallbacks = 0;
    uint32_t stage_phr_fused_symbols = 0;
    uint64_t stage_phr_fused_template_us = 0;
    uint64_t stage_phr_fused_correlate_us = 0;
    uint64_t stage_phr_fused_decode_us = 0;
    uint64_t stage_phr_fused_decode_ns = 0; // nanosecond resolution
    // Diagnostic-only FIR segmentation from the replay helper.  `valid=1`
    // means the ranges below partition `cir.soft_chip_count` at the decoded
    // boundaries.  These values are never production exact attribution.
    uint32_t diag_fir_valid = 0;
    uint64_t diag_fir_preamble_us = 0;
    uint64_t diag_fir_sfd_us = 0;
    uint64_t diag_fir_phr_us = 0;
    uint64_t diag_fir_payload_us = 0;
    uint64_t diag_fir_extra_us = 0;
    uint32_t diag_fir_preamble_chips = 0;
    uint32_t diag_fir_sfd_chips = 0;
    uint32_t diag_fir_phr_chips = 0;
    uint32_t diag_fir_payload_chips = 0;
    uint32_t diag_fir_extra_chips = 0;
    uint32_t stage_tail_retrack_fallbacks = 0; // legacy-radius tail retries
    uint64_t stage_sfd_tail_retrack_fallback_us = 0; // included in tail timer
    // `stage_sfd` runs once for bootstrap CFO anchoring and once after the
    // MATLAB-aligned CFO refit.  Preserve both actual search counts.
    uint64_t sfd_bootstrap_windows = 0;
    uint64_t sfd_bootstrap_coarse_correlations = 0;
    uint64_t sfd_bootstrap_fine_correlations = 0;
    uint64_t sfd_final_windows = 0;
    uint64_t sfd_final_coarse_correlations = 0;
    uint64_t sfd_final_fine_correlations = 0;
    // Diagnostic CHILD sub-timers of `stage_sfd_us`.  They break the parent
    // stage — already measured end-to-end by the single `lap(stage_sfd_us)`
    // on the success path — into its internal sub-blocks:
    //   bootstrap stage_sfd()  -> stage_sfd_bootstrap_us
    //   tail stage_timing()    -> stage_sfd_tail_retrack_us
    //   second stage_cfo()     -> stage_sfd_cfo_refit_us
    //   final stage_sfd()      -> stage_sfd_final_us
    //   remainder of the stage -> stage_sfd_anchor_other_us (checks, grid
    //                             rebuild, clamp at 0)
    // IMPORTANT: the parent `stage_sfd_us` ALREADY INCLUDES these children.
    // They must NEVER be added to `stage_sfd_us` or `stage_total_us` (that
    // would double-count).  They are published for diagnostics only.
    uint64_t stage_sfd_bootstrap_us = 0;
    uint64_t stage_sfd_tail_retrack_us = 0;
    uint64_t stage_sfd_cfo_refit_us = 0;
    uint64_t stage_sfd_final_us = 0;
    uint64_t stage_sfd_anchor_other_us = 0;
    // SFD-anchored CFO refit diagnostics (read-only).  The demodulator fits the
    // carrier offset twice; these fields publish both estimates and the exact
    // span the second rotation covered, so the "is the second pass redundant?"
    // question can be answered from the log instead of from a model.
    //   cfo_initial_hz   Stage-2 fit over the Stage-1 SYNC peak train
    //   cfo_refit_hz     SFD-anchored 40-SYNC tail refit (0 when not run)
    //   cfo_refit_span_samples  sec_hi - sec_lo of the second rotation
    //   cfo_refit_rotate_us     second rotation only (child of st_sfd)
    //   cfo_refit_phase_drift_rad  |2*pi*(refit-initial)*span/fs|
    //   cfo_refit_action  0=legacy refit+rotate, 1=auto skip, 2=forced skip,
    //                     3=requested skip but fell back to the legacy path
    double cfo_initial_hz = 0.0;
    double cfo_refit_hz = 0.0;
    uint32_t cfo_initial_peaks = 0;
    uint32_t cfo_refit_peaks = 0;
    uint64_t cfo_refit_span_samples = 0;
    uint64_t cfo_refit_rotate_us = 0;
    uint64_t cfo_refit_fit_us = 0;
    double cfo_refit_phase_drift_rad = 0.0;
    uint32_t cfo_refit_action = 0;
    // Bootstrap preamble prediction vs final CFO-aligned SFD coordinates.
    // Both are absolute after demodulate_one() rebases the cropped PDU.
    int64_t sfd_initial_predicted_sample = -1;
    int64_t sfd_bootstrap_detected_sample = -1;
    int64_t sfd_bootstrap_first_threshold_backtrack_symbols = -1;
    // 65/48 PDU resampler compute time, copied from upstream metadata.  This
    // precedes the worker and is excluded from stage_total_us.
    uint64_t resample_us = 0;
};

// ---------------------------------------------------------------------------
// Tolerances for MATLAB <-> C++ comparison (Phase-1).
// ---------------------------------------------------------------------------
struct DemodTolerance {
    double sample_tolerance = 2.0;      // sample index tolerance (samples)
    double cfo_hz_tolerance = 50.0;     // CFO within 50 Hz
    float metric_tolerance = 0.05f;     // correlation metric within 0.05
    float cir_tolerance = 0.1f;         // normalized CIR L2 error
    size_t soft_chip_symbol_tol = 1;   // soft-chip alignment tolerance
};

} // namespace demod
} // namespace uwb
} // namespace gr
