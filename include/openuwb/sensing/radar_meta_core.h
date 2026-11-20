/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Pure C++ radar/PDU metadata constants and validation.
 *
 * Split out of uwb_radar_pdu_meta.h so that core code (e.g.
 * uwb_hrp_mod_core.h, the TWR contract layer and native QA) can use the
 * rates, spans and whitelist key names WITHOUT pulling in <pmt/pmt.h>.
 * The PMT dictionary mapping helpers stay in uwb_radar_pdu_meta.h, which
 * includes this header; no constant is duplicated between the two.
 */

#pragma once

#include <openuwb/support/checked_math.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstddef>

namespace gr {
namespace uwb {
namespace radar_meta {

inline constexpr double kRateTolRel = 1e-6;
inline constexpr double kWorkRateHz = 998400000.0;
inline constexpr double kNativeRateHz = 737280000.0;       // UC200
inline constexpr double kCg400NativeRateHz = 491520000.0;  // CG400
inline constexpr uint32_t kResampleInterp = 65;
inline constexpr uint32_t kUc200NativeDecim = 48;
inline constexpr uint32_t kCg400NativeDecim = 32;

inline bool rates_close(double a, double b)
{
    if (!(a > 0.0) || !(b > 0.0))
        return false;
    return std::abs(a - b) <= kRateTolRel * std::max(std::abs(a), std::abs(b));
}

inline bool is_work_rate(double fs)
{
    return rates_close(fs, kWorkRateHz) || rates_close(fs, 998.4e6);
}

inline bool is_uc200_native_rate(double fs)
{
    return rates_close(fs, kNativeRateHz) || rates_close(fs, 737.28e6);
}

inline bool is_cg400_native_rate(double fs)
{
    return rates_close(fs, kCg400NativeRateHz) || rates_close(fs, 491.52e6);
}

inline bool is_native_rate(double fs)
{
    return is_uc200_native_rate(fs) || is_cg400_native_rate(fs);
}

inline uint32_t native_decim(double fs)
{
    return is_cg400_native_rate(fs) ? kCg400NativeDecim : kUc200NativeDecim;
}

// ceil(work_samples * decim / 65).  64-SYNC native spans: 48018 @737.28
// (M=48) and 32012 @491.52 (M=32).
inline int64_t native_span(int64_t work_samples, uint32_t decim)
{
    if (work_samples <= 0 || decim == 0)
        return 0;
    const int64_t scaled = work_samples * static_cast<int64_t>(decim);
    return (scaled + static_cast<int64_t>(kResampleInterp) - 1) /
           static_cast<int64_t>(kResampleInterp);
}

// Radar TX SYNC lengths.  IEEE BPRF native PreambleDuration is
// 16/64/1024/4096; 32/128/256/512/2048 are custom profiles built by
// cropping/tiling a pulse-shaped SYNC field (not 751-sample repeats).
inline constexpr size_t kRadarSyncRepetitions[] = {
    1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024, 2048
};
inline constexpr size_t kMaxRadarBurstSamples = 2097152; // 1<<21, 2048-SYNC RX
inline constexpr size_t kMaxPsduBytes = 127;

inline bool sync_reps_supported(size_t n)
{
    for (size_t v : kRadarSyncRepetitions) {
        if (n == v)
            return true;
    }
    return false;
}

inline const char* sync_reps_supported_list()
{
    return "1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024 or 2048";
}

inline bool code_index_supported(size_t n)
{
    return n >= 9 && n <= 12;
}

inline constexpr const char* kPulseId = "pulse_id";
inline constexpr const char* kScheduleIndex = "schedule_index";
inline constexpr const char* kTxTimeFull = "tx_time_full";
inline constexpr const char* kTxTimeFrac = "tx_time_frac";
inline constexpr const char* kRxTimeFull = "rx_time_full";
inline constexpr const char* kRxTimeFrac = "rx_time_frac";
inline constexpr const char* kNumDelaySamps = "num_delay_samps";
inline constexpr const char* kCalDelayNative = "calibration_delay_native_samples";
inline constexpr const char* kCalDelayWork = "calibration_delay_work_samples";
inline constexpr const char* kCalibrationId = "calibration_id";
inline constexpr const char* kSyncRepetitions = "sync_repetitions";
inline constexpr const char* kSfdMode = "sfd_mode";
inline constexpr const char* kCodeIndex = "code_index";
inline constexpr const char* kUhdError = "uhd_error";
inline constexpr const char* kSource = "source";
inline constexpr const char* kSyncSamples = "sync_samples";
inline constexpr const char* kSfdSamples = "sfd_samples";
inline constexpr const char* kRangeGuardSamples = "range_guard_samples";
inline constexpr const char* kTxPacketSamples = "tx_packet_samples";
inline constexpr const char* kRxCaptureSamples = "rx_capture_samples";
inline constexpr const char* kPredictedSfdStart = "predicted_sfd_start_sample";

// Copied unchanged. sample_format is owned by the resampler output path.
inline constexpr const char* kPassthrough[] = {
    "pulse_id",
    "schedule_index",
    "schedule_generation",
    "acquisition_epoch",
    "packet_id",
    "tx_time_full",
    "tx_time_frac",
    "rx_time_full",
    "rx_time_frac",
    "num_delay_samps",
    "calibration_delay_native_samples",
    "calibration_id",
    "sync_repetitions",
    "sfd_mode",
    "code_index",
    "uhd_error",
    "source",
    // Sense-RF sweep coordinates, used by non-jamming CIR captures.
    "freq_hz",
    "freq_offset_hz",
    // Dual-TX jammer diagnostics (native control quantities: do not scale
    // jam_delay_native through 65/48).
    "jam_delay_native",
    "jam_delay_us",
    "jam_delay_mode",
    "jam_delay_seed",
    "jam_freq_plan_hz",
    "jam_freq_actual_hz",
    "jam_freq_offset_hz",
    "jam_scan_step",
    "jam_dwell_target",
    "jam_dwell_successes_before",
    "jam_retune_seq",
    "sense_offset_native",
    "tx_fragment_count",
    "sense_tx_ticks",
    "jam_tx_ticks",
    // Preamble-overlap real packet collision (§6). Native/control
    // quantities: never scaled through 65/48.
    "jam_overlap_reps",
    "jam_overlap_side",
    "sense_preamble_reps",
    "jam_preamble_reps",
    "jam_overlap_seed",
};

// Absolute native indices → map(p). Native value stored as key+"_native".
inline constexpr const char* kIndexKeys[] = {
    "predicted_sfd_start_sample",
    "sfd_start_sample",
    "preamble_start_sample",
};

// Native lengths → map(n)-map(0). Native value stored as key+"_native".
inline constexpr const char* kLengthKeys[] = {
    "sync_samples",
    "sfd_samples",
    "range_guard_samples",
    "tx_packet_samples",
    "rx_capture_samples",
};

} // namespace radar_meta
} // namespace uwb
} // namespace gr
