// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <cmath>
#include <algorithm>
#include <openuwb/phy/hrp/rx/demod_result.h>
#include <openuwb/phy/hrp/common/profile.h>

namespace gr::uwb::demod::core {


// Low-SNR policy gates: normalized correlation metrics (not dB).
// Validated with code10 / 256 SYNC / 4z2, 0 ppm AWGN replay. See
// docs/performance/低SNR解调计算耗时优化开发报告_2026-11-17.md
// and 低SNR解调与进锁优化开发报告_2026-11-17.md for QA and limits.
inline constexpr float kLockedInnerNarrowRawGate = 0.045f;
inline constexpr float kSfdGridPeakFloor = 0.008f;
inline constexpr double kSfdGridParFloor = 1.5;

// ---------------------------------------------------------------------------
// Integer chip-grid admission
//
// The AVX2 soft-chip kernel and the fused CMF both index samples on an exact
// kSamplesPerChip grid, so the real question is not "is samples_per_chip
// exactly 2" but "does the grid drift accumulated across the frame stay inside
// the correlation tolerance".
//
// Physical ceiling.  The longest supported frame is 127 B, whose BPRF payload
// spans 154,624 samples.  One sample of drift over that span is a sample-clock
// offset of 1/154624 = 6.5 ppm; past that the integer grid walks off the
// correlation peak inside a single frame.
//
// Default admission is deliberately tighter: 1.5 ppm = 0.23 samples of drift,
// with 2.3x margin over the best clock measured on hardware (0.66 ppm).  The
// 256-peak period estimator quantises samples_per_chip in ~2 ppm steps, so a
// gate at the physical ceiling would admit a clock whose *measured* offset is
// 3-6 ppm away from zero without any real evidence that the clock is good.
// Callers can widen or tighten it per run through Qm35825Profile::fused_gate_ppm
// without a rebuild.
//
// Measured on hardware: a 27 ppm clock drifts 4.2 samples across the payload
// and is rejected at every setting; a 0.66 ppm clock drifts 0.10 samples and is
// accepted.  The previous 1e-9 (0.0005 ppm) threshold was three orders of
// magnitude BELOW the estimator noise, so real data could never satisfy it and
// the fused path was unreachable on hardware.
// ---------------------------------------------------------------------------
inline constexpr double kMaxFramePayloadSamples = 154624.0; // 127 B BPRF
// Hard ceiling: one sample of drift across the longest frame.
inline constexpr double kIntegerGridCeilingPpm =
    1.0 / kMaxFramePayloadSamples * 1e6;
// Default admission (deliberately tighter than the ceiling).
inline constexpr double kIntegerGridMaxClockPpm = 1.5;
inline constexpr double kIntegerGridRelTolerance =
    kIntegerGridMaxClockPpm * 1e-6;
inline constexpr double kIntegerGridMaxDriftSamples =
    kIntegerGridRelTolerance * kMaxFramePayloadSamples;

/** True when `samples_per_chip` sits within `rel_tolerance` of `target`. */
inline bool integer_chip_grid_at(double samples_per_chip, size_t target,
                                 double rel_tolerance)
{
    if (!(samples_per_chip > 0.0) || target == 0 ||
        !(rel_tolerance > 0.0) || !std::isfinite(rel_tolerance))
        return false;
    return std::abs(samples_per_chip - static_cast<double>(target)) /
               samples_per_chip <=
           rel_tolerance;
}

/** Default-tolerance overload (see kIntegerGridMaxClockPpm). */
inline bool integer_chip_grid_at(double samples_per_chip, size_t target)
{
    return integer_chip_grid_at(samples_per_chip, target,
                                kIntegerGridRelTolerance);
}

/** True when `samples_per_chip` sits within tolerance of its nearest integer. */
inline bool integer_chip_grid(double samples_per_chip, double rel_tolerance)
{
    if (!(samples_per_chip > 0.0))
        return false;
    const double nearest = std::round(samples_per_chip);
    if (nearest <= 0.0)
        return false;
    return integer_chip_grid_at(samples_per_chip,
                                static_cast<size_t>(nearest), rel_tolerance);
}

/** Default-tolerance overload (see kIntegerGridMaxClockPpm). */
inline bool integer_chip_grid(double samples_per_chip)
{
    return integer_chip_grid(samples_per_chip, kIntegerGridRelTolerance);
}


} // namespace gr::uwb::demod::core
