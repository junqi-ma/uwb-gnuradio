// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <openuwb/phy/hrp/rx/demod_sync.h>
#include <openuwb/phy/hrp/rx/rx_cfo_opt.h>

namespace openuwb::demod::core {
// ---------------------------------------------------------------------------
// Stage 2 — CFO estimation + compensation (R1).
// Uses the stable SYNC peaks after the startup transient to fit phase vs time
// linearly; the slope / 2pi is the CFO in Hz.  MATLAB skips up to its first
// 24 peaks while retaining at least 32, controlled here by the separate CFO
// profile fields (not the CIR skip policy). The frame is then derotated by
// exp(-j2pi*f*n/fs) and phase-resolved against the preamble waveform. Golden
// reference: CFO = 0 Hz for the clean baseband signal.
// ---------------------------------------------------------------------------
// Decoupled CFO estimation (Task A1).
// Fits the CFO frequency from SYNC peaks without derotating samples.

bool fit_cfo(const std::complex<float>* rx,
             size_t n,
             const Qm35825Profile& profile,
             const TimingResult& timing,
             CfoResult& out,
             DemodScratch& scratch);

// Decoupled CFO range rotation (Task A1).
// Compensates [lo, hi) where phase origin is strictly sample 0.
void derotate_cfo_range(const std::complex<float>* rx,
                        size_t n,
                        const Qm35825Profile& profile,
                        double cfo_hz,
                        size_t lo,
                        size_t hi,
                        DemodScratch& scratch,
                        uint64_t& rotate_us,
                        uint32_t& rotation_kernel);

bool stage_cfo(const std::complex<float>* rx,
               size_t n,
               const Qm35825Profile& profile,
               const TimingResult& timing,
               CfoResult& out,
               DemodScratch& scratch);

} // namespace openuwb::demod::core
