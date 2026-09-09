/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Radar PDU metadata keys and copy/map helpers for the 65/48 adapter.
 *
 * Identity/time/profile fields are copied as-is. Native sample *indices*
 * and *lengths* used on the 998.4 work grid are mapped with the caller's
 * group-delay-centered map(p). Existing scheduled-capture keys keep their
 * current semantics; this header only adds the radar whitelist.
 */

#pragma once

#include <gnuradio/uwb/uwb_radar_checked_math.h>
#include <pmt/pmt.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>

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
    32, 64, 128, 256, 512, 1024, 2048
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
    return "32, 64, 128, 256, 512, 1024 or 2048";
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

inline bool dict_has(pmt::pmt_t dict, const char* key)
{
    return pmt::is_dict(dict) && pmt::dict_has_key(dict, pmt::mp(key));
}

inline void copy_if_present(pmt::pmt_t& dst, pmt::pmt_t src, const char* key)
{
    if (!dict_has(src, key) || dict_has(dst, key))
        return;
    dst = pmt::dict_add(dst, pmt::mp(key),
                        pmt::dict_ref(src, pmt::mp(key), pmt::PMT_NIL));
}

inline bool try_to_i64(pmt::pmt_t v, int64_t& out)
{
    if (pmt::is_uint64(v))
        return radar::radar_i64_from_u64(pmt::to_uint64(v), out);
    if (pmt::is_integer(v)) {
        out = static_cast<int64_t>(pmt::to_long(v));
        return true;
    }
    if (pmt::is_real(v)) {
        const double d = pmt::to_double(v);
        if (!std::isfinite(d) ||
            d > static_cast<double>(std::numeric_limits<int64_t>::max()) ||
            d < static_cast<double>(std::numeric_limits<int64_t>::min()))
            return false;
        out = static_cast<int64_t>(std::llround(d));
        return true;
    }
    return false;
}

inline int64_t to_i64(pmt::pmt_t v, int64_t def)
{
    int64_t out = def;
    if (!try_to_i64(v, out))
        return def;
    return out;
}

inline double to_f64(pmt::pmt_t v, double def)
{
    if (pmt::is_real(v) || pmt::is_integer(v) || pmt::is_uint64(v))
        return pmt::to_double(v);
    return def;
}

// MapFn: bool(int64_t native_index, int64_t& mapped). Returns false if any
// present Radar index/length field cannot be converted or mapped.
template <typename MapFn>
inline bool apply_radar_whitelist(pmt::pmt_t& dst,
                                  pmt::pmt_t src,
                                  MapFn map_index,
                                  uint32_t interp = kResampleInterp,
                                  uint32_t decim = kUc200NativeDecim)
{
    if (!pmt::is_dict(src))
        return true;

    for (const char* key : kPassthrough)
        copy_if_present(dst, src, key);

    if (dict_has(src, kCalDelayNative) && !dict_has(dst, kCalDelayWork)) {
        const double native = to_f64(
            pmt::dict_ref(src, pmt::mp(kCalDelayNative), pmt::from_double(0)),
            0.0);
        if (!std::isfinite(native))
            return false;
        if (interp == 0 || decim == 0)
            return false;
        dst = pmt::dict_add(dst, pmt::mp(kCalDelayWork),
                            pmt::from_double(native * static_cast<double>(interp) /
                                             static_cast<double>(decim)));
    }

    auto add_native_alias = [&](const char* key, pmt::pmt_t value) {
        std::string nk = std::string(key) + "_native";
        if (!dict_has(dst, nk.c_str()))
            dst = pmt::dict_add(dst, pmt::mp(nk.c_str()), value);
    };

    for (const char* key : kIndexKeys) {
        if (!dict_has(src, key))
            continue;
        const pmt::pmt_t raw =
            pmt::dict_ref(src, pmt::mp(key), pmt::from_long(-1));
        int64_t p = 0;
        if (!try_to_i64(raw, p))
            return false;
        add_native_alias(key, raw);
        if (p < 0)
            return false;
        int64_t mapped = 0;
        if (!map_index(p, mapped))
            return false;
        if (!dict_has(dst, key))
            dst = pmt::dict_add(dst, pmt::mp(key), pmt::from_long(mapped));
    }

    for (const char* key : kLengthKeys) {
        if (!dict_has(src, key))
            continue;
        const pmt::pmt_t raw =
            pmt::dict_ref(src, pmt::mp(key), pmt::from_long(0));
        int64_t n = 0;
        if (!try_to_i64(raw, n))
            return false;
        add_native_alias(key, raw);
        if (n < 0)
            return false;
        int64_t mapped_n = 0;
        int64_t mapped_0 = 0;
        if (!map_index(n, mapped_n) || !map_index(0, mapped_0))
            return false;
        int64_t mapped = 0;
        if (!radar::radar_i64_sub(mapped_n, mapped_0, mapped))
            return false;
        if (mapped < 0)
            mapped = 0;
        if (!dict_has(dst, key))
            dst = pmt::dict_add(dst, pmt::mp(key), pmt::from_long(mapped));
    }
    return true;
}

} // namespace radar_meta
} // namespace uwb
} // namespace gr
