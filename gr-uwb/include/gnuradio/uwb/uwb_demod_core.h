/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * GNU Radio-independent UWB demodulation core.  Each stage is a pure function
 * that can be unit-tested and benchmarked without linking against GNU Radio.
 *
 * This header is the Phase-1 SKELETON: it declares the per-stage function
 * signatures and the worker's scratch state.  No algorithm is implemented
 * here until the MATLAB golden reference (R0) is frozen.
 *
 * Coordinate convention: all sample indices are 0-based absolute.  Each stage
 * takes the cropped frame + the previous stage's output and fills the next.
 */

#pragma once

#include <gnuradio/uwb/uwb_demod_result.h>
#include <gnuradio/uwb/uwb_detector_core.h>
#include <gnuradio/uwb/uwb_phy_profile.h>

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdint>
#include <limits>
#include <vector>

namespace gr {
namespace uwb {
namespace demod {
namespace core {

// ---------------------------------------------------------------------------
// Worker scratch: one per worker thread.  Pre-allocated at construction and
// reused across jobs to avoid per-packet allocation in the hot path.
// ---------------------------------------------------------------------------
struct DemodScratch {
    std::vector<std::complex<float>> work;     // cropped frame workspace
    std::vector<std::complex<float>> derotated; // CFO-derotated frame
    std::vector<float> energy;                  // per-sample |x|^2
    std::vector<float> metric;                  // correlation metric buffer
    std::vector<std::complex<float>> corr;      // matched-filter output
    std::vector<float> soft_chips;              // soft-chip stream
    std::vector<float> cir_taps;                // CIR estimate
    std::vector<int64_t> peaks;                 // SYNC peak positions
    std::vector<float> peak_values;             // per-peak complex correlation

    void reserve(size_t n)
    {
        work.reserve(n);
        derotated.reserve(n);
        energy.reserve(n);
        metric.reserve(n);
        corr.reserve(n);
        soft_chips.reserve(n);
        cir_taps.reserve(64);
        peaks.reserve(256);
        peak_values.reserve(256);
    }
};

// ---------------------------------------------------------------------------
// Stage 1 — seeded timing: detect + track SYNC preamble repetitions.
// Returns false if fewer than min_valid_peaks SYNC peaks are confirmed.
// ---------------------------------------------------------------------------
bool stage_timing(const std::complex<float>* rx,
                  size_t n,
                  const Qm35825Profile& profile,
                  const std::vector<std::complex<float>>& template_wf,
                  int64_t seed_start, // absolute predicted start, or -1
                  TimingResult& out,
                  DemodScratch& scratch);

// ---------------------------------------------------------------------------
// Stage 2 — CFO estimation + compensation.
// Uses stable SYNC peaks (skipping the first cir_skip_initial_repetitions)
// to fit phase vs time, then derotates the frame.
// ---------------------------------------------------------------------------
bool stage_cfo(const std::complex<float>* rx,
               size_t n,
               const Qm35825Profile& profile,
               const TimingResult& timing,
               CfoResult& out,
               DemodScratch& scratch);

// ---------------------------------------------------------------------------
// Stage 3 — SFD template selection + full-rate timing refinement.
// Picks the best-matching SFD template (4z2 for QM35825) and refines the
// preamble start at full sample rate.
// ---------------------------------------------------------------------------
bool stage_sfd(const std::complex<float>* rx,
               size_t n,
               const Qm35825Profile& profile,
               const TimingResult& timing,
               const std::vector<int8_t>& sfd_sequence,
               const std::vector<std::complex<float>>& template_wf,
               SfdResult& out,
               DemodScratch& scratch);

// ---------------------------------------------------------------------------
// Stage 4 — CIR estimation + soft-chip generation.
// Estimates CIR from the last cir_repetitions SYNC symbols, then matched-
// filters the frame to produce a soft-chip stream.
// ---------------------------------------------------------------------------
bool stage_cir_softchips(const std::complex<float>* rx,
                         size_t n,
                         const Qm35825Profile& profile,
                         const TimingResult& timing,
                         const std::vector<int8_t>& preamble_code,
                         CirResult& out,
                         DemodScratch& scratch);

// ---------------------------------------------------------------------------
// Stage 5 — NS-SFD location in the soft-chip stream.
// ---------------------------------------------------------------------------
bool stage_ns_sfd(const std::vector<float>& soft_chips,
                  const Qm35825Profile& profile,
                  const std::vector<int8_t>& sfd_sequence,
                  size_t chips_per_symbol,
                  NsSfdResult& out,
                  DemodScratch& scratch);

// ---------------------------------------------------------------------------
// Stage 6 — BPRF PHR demod + convolutional decode + SECDED.
// ---------------------------------------------------------------------------
bool stage_phr(const std::vector<float>& soft_chips,
               const Qm35825Profile& profile,
               const NsSfdResult& ns_sfd,
               size_t chips_per_symbol,
               PhrResult& out,
               DemodScratch& scratch);

// ---------------------------------------------------------------------------
// Stage 7 — payload BPM-BPSK demod + convolutional + RS + FCS.
// ---------------------------------------------------------------------------
bool stage_payload_fcs(const std::vector<float>& soft_chips,
                       const Qm35825Profile& profile,
                       const PhrResult& phr,
                       const NsSfdResult& ns_sfd,
                       size_t chips_per_symbol,
                       PayloadResult& out,
                       DemodScratch& scratch);

// ---------------------------------------------------------------------------
// Full pipeline: runs all stages in order, stopping at the first failure.
// Used by the worker thread for each job.
// ---------------------------------------------------------------------------
DemodResult demodulate_one(const std::complex<float>* rx,
                           size_t n,
                           const Qm35825Profile& profile,
                           uint64_t packet_id,
                           int64_t predicted_start,
                           int64_t window_start,
                           const std::vector<std::complex<float>>& template_wf,
                           DemodScratch& scratch);

} // namespace core
} // namespace demod
} // namespace uwb
} // namespace gr
