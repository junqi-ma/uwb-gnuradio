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
// A strong peak establishes preamble existence.  QM35 startup can attenuate
// the first few SYNCs, so refine the start backwards on the known symbol grid
// with a lower per-symbol threshold; stop at the first missing grid point.
inline constexpr float kDetectorFineThreshold = 0.5f;
inline constexpr float kDetectorBacktrackThreshold = 0.2f;
inline constexpr size_t kDetectorBacktrackRadius = 8;
inline constexpr size_t kDetectorMaxBacktrackSymbols = 3;

// UwbScheduledExtractor production radar-slot geometry (QM35825).
inline constexpr size_t kScheduledPreGuard = 9984;   // ~10 us @ 998.4e6
inline constexpr size_t kScheduledCapture = 189696;  // ~190 us
inline constexpr size_t kScheduledPostGuard = 4096;
inline constexpr size_t kScheduledPoolSize = 8;
inline constexpr double kScheduledPacketIntervalS = 0.01; // 100 radar/s

// X410 native SC16 rate (no RFNoC / host 65/48).
inline constexpr double kNativeSampleRateHz = 737280000.0;
// QM35 radar slot period used by the auto-scheduled native capture path.
inline constexpr double kQm35PacketIntervalS = 0.005; // 200 slot/s
// Time-derived window geometry at 737.28 MS/s.  Do NOT copy the 998.4
// sample counts 9984/189696/4096 without converting by 48/65.
//   pre  = llround(10e-6  * 737.28e6) = 7373
//   body = llround(190e-6 * 737.28e6) = 140083
//   post = llround(4.1e-6 * 737.28e6) = 3023
inline constexpr size_t kNativeScheduledPreGuard = 7373;
inline constexpr size_t kNativeScheduledCapture = 140083;
inline constexpr size_t kNativeScheduledPostGuard = 3023;
// Native dump geometry.  CIR for QM35 is taken from the preamble
// (~64 SYNC ≈ 65 µs), so a DW1000 that starts before / at predicted t0 is
// the interferer that matters.  Keep a full DW1000 airtime in front so that
// packet's preamble origin is in-window; the tail only has to finish that
// same packet after the 190 µs QM35 body.
//   256 SYNC + DW-8 + 12 B @ 6.81 Mbps ≈ 270–290 µs
//   pre  = llround(300e-6 * 737.28e6) = 221184
//   post = llround(100e-6 * 737.28e6) = 73728
//     (covers a ≤290 µs DW1000 that starts at t0: 290 − 190 = 100 µs)
// Do NOT replace the 10/190/4.1 µs demod defaults.
inline constexpr double kNativeInterferencePreGuardS = 300e-6;
inline constexpr double kNativeInterferencePostGuardS = 100e-6;
inline constexpr size_t kNativeInterferencePreGuard = 221184;
inline constexpr size_t kNativeInterferencePostGuard = 73728;
inline constexpr size_t kAutoAcquirePoolSize = 8;
inline constexpr size_t kAutoScheduledPoolSize = 8;
inline constexpr size_t kLockObservations = 3;
inline constexpr size_t kHoldoverMissCount = 3;
inline constexpr size_t kReacquireMissCount = 8;
inline constexpr double kProvisionalGuardUs = 25.0;

} // namespace defaults
} // namespace uwb
} // namespace gr
