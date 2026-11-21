// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <openuwb/dsp/cir_fir_simd.h>
#include <openuwb/phy/hrp/rx/demod_scratch.h>
#include <openuwb/phy/hrp/rx/demod_grid.h>
#include <openuwb/phy/hrp/rx/rx_softchip_opt.h>

namespace openuwb::demod::core {
// ---------------------------------------------------------------------------
// Stage 4 — CIR estimation + soft-chip generation (R2).
// Mirrors MATLAB estimateCir + estimateCirAndSoftChips:
//
//   * CIR: coherently average the aligned raw SYNC windows of the LAST
//     cir_repetitions repetitions (skipping the first
//     cir_skip_initial_repetitions), then correlate each of the
//     (pre+post) taps against the code in NATURAL order:
//       values[n] = sum_m avg[n+m] * conj(sampled_code[m]) / code_energy
//     and L2-normalize.  (The template is the sparse sampled code, NOT the
//     pulse-shaped preamble waveform — forward order, not time-reversed.)
//
//   * Soft chips: causal FIR with cirMf = conj(flip(values)) evaluated on the
//     chip grid chipStart = start + (pre+post) - pre - 1, spaced
//     measured_period/chips_per_symbol apart, up to the frame-span budget
//     bounded by the buffer length.  The complex chips are then phase-aligned
//     against the last min(32, preamble_repetitions) SYNCs' spread code, and
//     the real part is normalized by max|real|.
//
// Coordinates are relative to rx, the supplied scheduled/captured window.
// The complete packet entry point rebases the published CIR first-path.  Verified against the MATLAB golden reference: CIR max diff
// ~6e-8, soft-chip stream max diff ~6e-7 (stage_cir.mat / stage_softchips.mat).
// ---------------------------------------------------------------------------

// Standalone CIR estimation stage (runs after SFD, independent of CMF/despreading).
//
// `cfo_hz` is the carrier offset that is STILL PRESENT in `rx`.  When it is
// non-zero, the caller passes the raw, unrotated frame and the rotation is
// folded into the SYNC averaging:
//
//   sum_k r[lo_k + m] e^{-j w (lo_k + m)}
//     = e^{-j w m} * sum_k ( e^{-j w lo_k} * r[lo_k + m] )
//
// The per-SYNC factor is one complex scalar; the per-sample ramp is applied
// once to the ~1053-sample average instead of to the whole preamble.  The
// default (cfo_hz == 0) keeps the legacy path exactly, so callers that already
// hold a derotated frame are unchanged.
bool estimate_cir(const std::complex<float>* rx,
                  size_t n,
                  const Qm35825Profile& profile,
                  const TimingResult& timing,
                  const std::vector<int8_t>& preamble_code,
                  CirResult& out,
                  DemodScratch& scratch,
                  double cfo_hz = 0.0);

// Soft-chip matched-filter FIR stage consuming a precomputed CIR.
bool stage_softchips_from_cir(const std::complex<float>* rx,
                              size_t n,
                              const Qm35825Profile& profile,
                              const TimingResult& timing,
                              CirResult& out,
                              DemodScratch& scratch,
                              size_t max_chips = 0,
                              size_t begin_chip = 0);

bool stage_cir_softchips(const std::complex<float>* rx,
                         size_t n,
                         const Qm35825Profile& profile,
                         const TimingResult& timing,
                         const std::vector<int8_t>& preamble_code,
                         CirResult& out,
                         DemodScratch& scratch,
                         size_t max_chips = 0,
                         size_t begin_chip = 0);


} // namespace openuwb::demod::core
