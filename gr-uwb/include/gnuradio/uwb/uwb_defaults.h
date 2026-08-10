/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Single source of production default geometry and sample-rate values used by
 * block factory defaults.  GRC YAML defaults must match these literals
 * (see qa_uwb_defaults / defaults_sources checks).
 *
 * Host rate 998.4e6 is the post-RFNoC (65/48) rate after X410 radio 737.28e6.
 */

#pragma once

#include <cstddef>

namespace gr {
namespace uwb {
namespace defaults {

// Host-side CF32 / SC16 stream rate used by production detector metadata and
// scheduled-extractor defaults.
inline constexpr double kSampleRateHz = 998400000.0;

// UwbDetector / UwbDetectorSc16 production capture geometry.
inline constexpr size_t kDetectorPreTrigger = 2032;
inline constexpr size_t kDetectorCapture = 200000;
inline constexpr float kDetectorEnergyThreshold = 1e-3f;
inline constexpr size_t kDetectorEnergyGateDecimation = 100;
inline constexpr size_t kDetectorCoarseDecimation = 4;
inline constexpr size_t kDetectorCoarseRepetitions = 1;
inline constexpr size_t kDetectorCoarseMargin = 16;

// UwbScheduledExtractor production radar-slot geometry (QM35825).
inline constexpr size_t kScheduledPreGuard = 9984;   // ~10 us @ 998.4e6
inline constexpr size_t kScheduledCapture = 189696;  // ~190 us
inline constexpr size_t kScheduledPostGuard = 4096;
inline constexpr size_t kScheduledPoolSize = 8;
inline constexpr double kScheduledPacketIntervalS = 0.01; // 100 radar/s

} // namespace defaults
} // namespace uwb
} // namespace gr
