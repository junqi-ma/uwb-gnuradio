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
    size_t soft_chip_count = 0;         // number of soft chips produced
    double samples_per_chip = 0.0;      // measured chip period

    // Sub-stage wall-clock timing (µs), so CIR estimate vs soft-chip FIR vs
    // postprocess can be ranked separately (P0 diagnostics).
    uint64_t cir_estimate_us = 0;       // spread build + SYNC average + taps
    uint64_t soft_fir_us = 0;           // chip-grid matched-filter FIR
    uint64_t postprocess_us = 0;        // phase align + rotate + real/max/norm
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
    uint64_t stage_total_us = 0;        // sum of the above (== demod_latency_us)
    // `stage_sfd` runs once for bootstrap CFO anchoring and once after the
    // MATLAB-aligned CFO refit.  Preserve both actual search counts.
    uint64_t sfd_bootstrap_windows = 0;
    uint64_t sfd_bootstrap_coarse_correlations = 0;
    uint64_t sfd_bootstrap_fine_correlations = 0;
    uint64_t sfd_final_windows = 0;
    uint64_t sfd_final_coarse_correlations = 0;
    uint64_t sfd_final_fine_correlations = 0;
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
