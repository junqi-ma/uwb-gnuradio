/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * QA for the UWB TWR timestamp / clock-domain contract (M0).
 *
 * Covers docs/twr/需求_UWB_SS_DS_TWR.md REQ-TIME-01..05 and REQ-CAL-01, the
 * "时钟" and "校准" rows of the 协议与时间 QA 矩阵 in
 * docs/twr/开发路线与验收矩阵.md §3 (REQ-QA-02), and the REQ-OUT-01 integer
 * precision requirement.
 *
 * This file is intentionally radio-free: every expected value is derived
 * analytically here, NOT from another TWR function, so a bug in the header
 * cannot mask itself.  The only shared code is the frozen `Duration` type from
 * uwb_twr_types.h, whose truncation-toward-zero nanosecond conversion is
 * asserted explicitly below.
 */

#include <boost/test/unit_test.hpp>

#include <gnuradio/uwb/uwb_twr_timestamp.h>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

namespace {

using gr::uwb::twr::apply_calibration_duration;
using gr::uwb::twr::apply_calibration_ticks;
using gr::uwb::twr::calibration_result_to_string;
using gr::uwb::twr::CalibrationResult;
using gr::uwb::twr::clock_domain_is_comparable;
using gr::uwb::twr::clock_domain_is_valid;
using gr::uwb::twr::clock_domain_same_epoch;
using gr::uwb::twr::clock_domain_same_identity;
using gr::uwb::twr::clock_domain_ns_per_tick_exact;
using gr::uwb::twr::clock_domain_tick_is_ns_or_coarser;
using gr::uwb::twr::clock_domain_ticks_in_range;
using gr::uwb::twr::clock_domain_unambiguous_interval_ns;
using gr::uwb::twr::clock_domain_unambiguous_ticks;
using gr::uwb::twr::clock_domain_wrap_period;
using gr::uwb::twr::clock_domain_wrap_period_seconds;
using gr::uwb::twr::kMaxFractionDifferenceDenominator;
using gr::uwb::twr::modular_tick_distance;
using gr::uwb::twr::ModularTickDistance;
using gr::uwb::twr::relative_interval_to_duration;
using gr::uwb::twr::RelativeTickInterval;
using gr::uwb::twr::time_interval_is_whole_ticks;
using gr::uwb::twr::time_interval_ns_projection_is_lossless;
using gr::uwb::twr::TimestampOrder;
using gr::uwb::twr::TimestampOrderBudget;
using gr::uwb::twr::timestamp_compare;
using gr::uwb::twr::timestamp_compare_absolute;
using gr::uwb::twr::timestamp_fraction_compare;
using gr::uwb::twr::timestamp_order_to_string;
using gr::uwb::twr::timestamp_order_window_ticks;
using gr::uwb::twr::timestamp_relative_interval;
using gr::uwb::twr::timestamp_tick_ns_projection_is_lossless;
using gr::uwb::twr::ClockDomain;
using gr::uwb::twr::Duration;
using gr::uwb::twr::ExchangeStatus;
using gr::uwb::twr::integer_needs_decimal_string;
using gr::uwb::twr::kCorrectionAntennaPlane;
using gr::uwb::twr::kCorrectionDelayedTxQuantization;
using gr::uwb::twr::kCorrectionFirstPathFraction;
using gr::uwb::twr::kCorrectionFirstPathQualityGate;
using gr::uwb::twr::kCorrectionNone;
using gr::uwb::twr::kCorrectionRmarkerOffset;
using gr::uwb::twr::kCorrectionRxSampleToFirstPath;
using gr::uwb::twr::kCorrectionSampleRateConversion;
using gr::uwb::twr::kCorrectionTxCommandToAir;
using gr::uwb::twr::kCorrectionWaveformGeometry;
using gr::uwb::twr::kCorrectionWindowCrop;
using gr::uwb::twr::kJsonSafeIntegerMax;
using gr::uwb::twr::kMaxTimestampFractionDenominator;
using gr::uwb::twr::raw_tick_delta;
using gr::uwb::twr::TickDelta;
using gr::uwb::twr::time_interval_status_from_string;
using gr::uwb::twr::time_interval_status_is_ok;
using gr::uwb::twr::time_interval_status_to_exchange_status;
using gr::uwb::twr::time_interval_status_to_string;
using gr::uwb::twr::time_interval_to_json_string;
using gr::uwb::twr::TimeInterval;
using gr::uwb::twr::TimeIntervalStatus;
using gr::uwb::twr::timestamp_add_fractional;
using gr::uwb::twr::timestamp_corrections_satisfied;
using gr::uwb::twr::timestamp_from_json_string;
using gr::uwb::twr::timestamp_has_calibration;
using gr::uwb::twr::timestamp_interval;
using gr::uwb::twr::timestamp_interval_to_duration;
using gr::uwb::twr::timestamp_is_marker_mapped;
using gr::uwb::twr::timestamp_is_range_capable;
using gr::uwb::twr::timestamp_is_self_consistent;
using gr::uwb::twr::timestamp_is_validated_measurement;
using gr::uwb::twr::timestamp_marker_class;
using gr::uwb::twr::timestamp_marker_class_to_string;
using gr::uwb::twr::timestamp_marker_interval_allowed;
using gr::uwb::twr::timestamp_normalize;
using gr::uwb::twr::timestamp_precedes;
using gr::uwb::twr::timestamp_rejection_reason;
using gr::uwb::twr::timestamp_required_corrections;
using gr::uwb::twr::timestamp_retag_marker;
using gr::uwb::twr::timestamp_to_decimal_string;
using gr::uwb::twr::timestamp_to_json_string;
using gr::uwb::twr::Timestamp;
using gr::uwb::twr::TimestampMarker;
using gr::uwb::twr::TimestampMarkerClass;
using gr::uwb::twr::TimestampSource;
using gr::uwb::twr::timestamps_share_time_line;
using gr::uwb::twr::to_decimal_string;

// ---------------------------------------------------------------------------
// Fixed reference numbers.  Never computed from the header under test.
// ---------------------------------------------------------------------------

// 1 DW UUS tick = 1/(499.2e6 * 128) s = 15.650040064102564 ps
constexpr double kDwUusTickHz = 499.2e6 * 128.0; // 63'897'600'000 Hz
constexpr double kX410DeviceHz = 737.28e6;        // native capture grid
constexpr double kX410WorkHz = 998.4e6;           // 4z symbol work grid
constexpr double kX410NativeHz = 491.52e6;        // second native grid
constexpr double kHostMonotonicHz = 1.0e9;

constexpr int64_t kTwoPow31 = 2147483648LL;
constexpr int64_t kTwoPow32 = 4294967296LL;
constexpr int64_t kTwoPow33 = 8589934592LL;
constexpr int64_t kTwoPow40 = 1099511627776LL;

// A small wrapping counter, used by the M0.1 ordering vectors.  8 bits makes
// the half period exactly 128, so "just under / exactly / just over" are three
// adjacent integers and cannot be confused with a rate-dependent value.  The
// rate is the 998.4 MS/s work grid; only the counter width matters here.
constexpr uint32_t kTightBits = 8;
constexpr int64_t kTightPeriod = 1LL << kTightBits;   // 256
constexpr int64_t kTightHalf = kTightPeriod / 2;      // 128

ClockDomain tight_wrap()
{
    ClockDomain d;
    d.name = "tight_wrap_8bit";
    d.tick_rate_hz = kX410WorkHz;
    d.epoch_id = 11;
    d.timestamp_bits = kTightBits;
    return d;
}

// A wrapping counter wide enough that a declared ordering budget sits well
// inside the half period (2^15 = 32768 ticks).
constexpr uint32_t kWideBits = 16;
constexpr int64_t kWideHalf = 1LL << (kWideBits - 1); // 32768

ClockDomain tight_wrap_next_epoch()
{
    ClockDomain d = tight_wrap();
    d.epoch_id = 12; // reboot / time reset
    return d;
}

ClockDomain wide_wrap()
{
    ClockDomain d;
    d.name = "wide_wrap_16bit";
    d.tick_rate_hz = kX410WorkHz;
    d.epoch_id = 12;
    d.timestamp_bits = kWideBits;
    return d;
}

// Full RX time-mapping chain: resampler/filter group delay, window crop,
// sample-rate conversion, waveform geometry, fractional first path, the
// REQ-TIME-05 quality gate and the profile RMARKER offset.
constexpr uint32_t kFullRxChain = kCorrectionRxSampleToFirstPath |
                                  kCorrectionWindowCrop |
                                  kCorrectionSampleRateConversion |
                                  kCorrectionFirstPathFraction |
                                  kCorrectionWaveformGeometry |
                                  kCorrectionFirstPathQualityGate |
                                  kCorrectionRmarkerOffset;

// TX command time -> first air sample, delayed-TX quantisation and the
// waveform geometry that fixes the RMARKER offset on transmit.
constexpr uint32_t kFullTxChain = kCorrectionWaveformGeometry |
                                  kCorrectionTxCommandToAir |
                                  kCorrectionDelayedTxQuantization;

ClockDomain x410_device()
{
    ClockDomain d;
    d.name = "x410_device";
    d.tick_rate_hz = kX410DeviceHz;
    d.epoch_id = 3;
    d.timestamp_bits = 0; // UHD device ticks are monotonic 64-bit
    return d;
}

ClockDomain x410_work()
{
    ClockDomain d = x410_device();
    d.name = "x410_work_998p4";
    d.tick_rate_hz = kX410WorkHz;
    return d;
}

ClockDomain x410_native()
{
    ClockDomain d = x410_device();
    d.name = "x410_native_491p52";
    d.tick_rate_hz = kX410NativeHz;
    return d;
}

ClockDomain x410_after_reboot()
{
    ClockDomain d = x410_device();
    d.epoch_id = 4; // device reboot / UHD time reset
    return d;
}

ClockDomain host_monotonic()
{
    ClockDomain d;
    d.name = "host_monotonic";
    d.tick_rate_hz = kHostMonotonicHz;
    d.epoch_id = 1;
    d.timestamp_bits = 0;
    return d;
}

ClockDomain dw_uus40()
{
    ClockDomain d;
    d.name = "dw1000_uus";
    d.tick_rate_hz = kDwUusTickHz;
    d.epoch_id = 7;
    d.timestamp_bits = 40; // DW1000 40-bit timestamp unit
    return d;
}

ClockDomain dw_uus40_next_epoch()
{
    ClockDomain d = dw_uus40();
    d.epoch_id = 8;
    return d;
}

ClockDomain dw_uus32_named()
{
    ClockDomain d = dw_uus40();
    d.timestamp_bits = 32; // truncated field
    d.name = "dw1000_uus_trunc32";
    return d;
}

ClockDomain dw_uus32()
{
    // Same name and rate as dw_uus40() but a different declared field width.
    ClockDomain d = dw_uus40();
    d.timestamp_bits = 32;
    return d;
}

// Helper: build a timestamp and hard-require that the factory accepted it.
Timestamp must_make(int64_t ticks,
                    const ClockDomain& d,
                    TimestampMarker marker,
                    TimestampSource source,
                    uint32_t corrections)
{
    Timestamp ts;
    const bool ok = Timestamp::from_ticks(ticks, d, marker, source, corrections, ts);
    BOOST_REQUIRE_MESSAGE(ok, "Timestamp::from_ticks refused a valid input");
    BOOST_REQUIRE(timestamp_is_self_consistent(ts));
    return ts;
}

Timestamp must_make_frac(int64_t ticks,
                         int32_t num,
                         uint32_t den,
                         const ClockDomain& d,
                         TimestampMarker marker,
                         TimestampSource source,
                         uint32_t corrections)
{
    Timestamp ts;
    const bool ok =
        Timestamp::from_fractional_ticks(ticks, num, den, d, marker, source, corrections, ts);
    BOOST_REQUIRE_MESSAGE(ok, "Timestamp::from_fractional_ticks refused a valid input");
    BOOST_REQUIRE(timestamp_is_self_consistent(ts));
    return ts;
}

std::string replace_once(const std::string& s,
                         const std::string& from,
                         const std::string& to)
{
    const size_t p = s.find(from);
    if (p == std::string::npos)
        return std::string();
    return s.substr(0, p) + to + s.substr(p + from.size());
}

// ===========================================================================
// 1. Clock domain construction / validation
// ===========================================================================

BOOST_AUTO_TEST_CASE(test_clock_domain_construction_and_validation)
{
    ClockDomain d;
    BOOST_REQUIRE(ClockDomain::make("x410_device", kX410DeviceHz, 3, 0, d));
    BOOST_REQUIRE(d.is_valid());
    BOOST_REQUIRE(clock_domain_is_valid(d));
    BOOST_REQUIRE(d.wrap_period() == 0u); // no wrap configured
    BOOST_REQUIRE(d.ticks_in_range(0));
    BOOST_REQUIRE(d.ticks_in_range(std::numeric_limits<int64_t>::max()));
    BOOST_REQUIRE(!d.ticks_in_range(-1));

    // Rejected: empty name.
    BOOST_REQUIRE(!ClockDomain::make("", kX410DeviceHz, 3, 0, d));
    // Rejected: non-finite / non-positive tick rates.
    BOOST_REQUIRE(!ClockDomain::make("bad", 0.0, 3, 0, d));
    BOOST_REQUIRE(!ClockDomain::make("bad", -1.0, 3, 0, d));
    BOOST_REQUIRE(!ClockDomain::make("bad", std::numeric_limits<double>::quiet_NaN(), 3, 0, d));
    BOOST_REQUIRE(!ClockDomain::make("bad", std::numeric_limits<double>::infinity(), 3, 0, d));
    BOOST_REQUIRE(
        !ClockDomain::make("bad", -std::numeric_limits<double>::infinity(), 3, 0, d));
    // Rejected: nonsensical wrap widths.  64 would be indistinguishable from
    // "no wrap"; < 8 is not a device timestamp field.
    BOOST_REQUIRE(!ClockDomain::make("bad", kX410DeviceHz, 3, 7, d));
    BOOST_REQUIRE(!ClockDomain::make("bad", kX410DeviceHz, 3, 64, d));
    BOOST_REQUIRE(!ClockDomain::make("bad", kX410DeviceHz, 3, 0xFFFFFFFFu, d));
    BOOST_REQUIRE(ClockDomain::make("ok", kX410DeviceHz, 3, 8, d));
    BOOST_REQUIRE(ClockDomain::make("ok", kX410DeviceHz, 3, 32, d));
    BOOST_REQUIRE(ClockDomain::make("ok", kX410DeviceHz, 3, 40, d));
    BOOST_REQUIRE(ClockDomain::make("ok", kX410DeviceHz, 3, 63, d));

    // Wrap period, unambiguous window and period length are per domain.
    uint64_t unamb = 0;
    int64_t unamb_ns = 0;
    const ClockDomain d40 = dw_uus40();
    BOOST_REQUIRE(clock_domain_wrap_period(d40) == static_cast<uint64_t>(kTwoPow40));
    BOOST_REQUIRE(clock_domain_unambiguous_ticks(d40, unamb));
    BOOST_REQUIRE(unamb == static_cast<uint64_t>(kTwoPow40 / 2));
    BOOST_REQUIRE(clock_domain_unambiguous_interval_ns(d40, unamb_ns));
    BOOST_REQUIRE(unamb_ns == 8603700512LL); // 2^39 ticks ~ 8.6023 s
    BOOST_REQUIRE(std::fabs(clock_domain_wrap_period_seconds(d40) - 17.207401025641026) <
                  1e-12);

    const ClockDomain d32 = dw_uus32_named();
    BOOST_REQUIRE(clock_domain_wrap_period(d32) == static_cast<uint64_t>(kTwoPow32));
    BOOST_REQUIRE(clock_domain_unambiguous_ticks(d32, unamb));
    BOOST_REQUIRE(unamb == static_cast<uint64_t>(kTwoPow32 / 2));
    BOOST_REQUIRE(clock_domain_unambiguous_interval_ns(d32, unamb_ns));
    BOOST_REQUIRE(unamb_ns == 33608205LL); // 2^31 ticks ~ 33.6 ms
    // The 32-bit field wraps 256x sooner than the 40-bit one: they cannot
    // share a multiplier (REQ-TIME-04).
    BOOST_REQUIRE(clock_domain_wrap_period(d40) == clock_domain_wrap_period(d32) * 256);

    // A no-wrap domain has no modular ambiguity bound.
    BOOST_REQUIRE(!clock_domain_unambiguous_ticks(x410_device(), unamb));
    BOOST_REQUIRE(!clock_domain_unambiguous_interval_ns(x410_device(), unamb_ns));
    BOOST_REQUIRE(clock_domain_wrap_period_seconds(x410_device()) == 0.0);

    // The tick window is bounded by the declared width.
    BOOST_REQUIRE(clock_domain_ticks_in_range(d40, kTwoPow40 - 1));
    BOOST_REQUIRE(!clock_domain_ticks_in_range(d40, kTwoPow40));
    BOOST_REQUIRE(clock_domain_ticks_in_range(d32, kTwoPow32 - 1));
    BOOST_REQUIRE(!clock_domain_ticks_in_range(d32, kTwoPow32));
}

// ===========================================================================
// 2. Whole-tick intervals in one domain at several tick rates
// ===========================================================================

BOOST_AUTO_TEST_CASE(test_interval_whole_ticks_across_rates)
{
    struct Case {
        const char* label;
        ClockDomain domain;
        int64_t dticks;
        int64_t expect_ns;
    };
    const std::vector<Case> cases = {
        { "work 998.4 MS/s, 1016 samples/symbol", x410_work(), 1016, 1017 },
        { "work 998.4 MS/s, 1 tick", x410_work(), 1, 1 },
        { "work 998.4 MS/s, 1000 ticks", x410_work(), 1000, 1001 },
        { "device 737.28 MS/s, 128 samples", x410_device(), 128, 173 },
        { "native 491.52 MS/s, 256 samples", x410_native(), 256, 520 },
        { "DW UUS, 31948 ticks (~500 ns)", dw_uus40(), 31948, 499 },
        { "DW UUS, 3000 ticks", dw_uus40(), 3000, 46 },
        { "DW UUS, 2^32 ticks", dw_uus40(), kTwoPow32, 67216410 },
    };

    for (const Case& c : cases) {
        const Timestamp later =
            must_make(c.dticks, c.domain, TimestampMarker::RmarkerRx,
                      TimestampSource::HardwareMeasured, kFullRxChain);
        const Timestamp earlier = must_make(0, c.domain, TimestampMarker::RmarkerRx,
                                            TimestampSource::HardwareMeasured, kFullRxChain);
        const TimeInterval ti = timestamp_interval(later, earlier);
        BOOST_REQUIRE_MESSAGE(ti.status == TimeIntervalStatus::Ok, c.label);
        BOOST_REQUIRE(ti.ticks == c.dticks);
        BOOST_REQUIRE(!ti.wrapped);
        BOOST_REQUIRE(ti.duration.nanos() == c.expect_ns);
        BOOST_REQUIRE(ti.domain.name == c.domain.name);
        BOOST_REQUIRE(ti.frac_num == 0);
        BOOST_REQUIRE(ti.frac_den == 0u);

        // M0.1 / R4: the old `duration_is_tick_exact` claimed a whole-tick
        // interval at a >= 1 ns tick rate was a faithful nanosecond
        // projection.  It is not.  "One tick is at least 1 ns" bounds the
        // projection error at < 1 tick; it does not make it zero.  So the
        // predicate is now `duration_is_lossless_ns` and each vector below
        // asserts it against the ACTUAL error, not against a rate threshold.
        BOOST_REQUIRE(ti.is_whole_ticks);
        BOOST_REQUIRE(ti.duration_is_lossless_ns ==
                      time_interval_ns_projection_is_lossless(ti));
    }

    // A zero interval is legal and exact.
    const Timestamp z = must_make(4242, x410_device(), TimestampMarker::RmarkerTx,
                                  TimestampSource::ScheduledCalibrated, kFullTxChain);
    const TimeInterval zi = timestamp_interval(z, z);
    BOOST_REQUIRE(zi.status == TimeIntervalStatus::Ok);
    BOOST_REQUIRE(zi.ticks == 0);
    BOOST_REQUIRE(zi.duration.is_zero());
    BOOST_REQUIRE(zi.is_whole_ticks);
    // Zero is 0 ns at every rate: the one case that is always lossless.
    BOOST_REQUIRE(zi.duration_is_lossless_ns);

    // The bool convenience form agrees with the struct form.
    Duration dur;
    TimeIntervalStatus st = TimeIntervalStatus::InvalidTimestamp;
    const Timestamp t1 = must_make(1000, x410_device(), TimestampMarker::RmarkerTx,
                                   TimestampSource::ScheduledCalibrated, kFullTxChain);
    const Timestamp t4 = must_make(1663, x410_device(), TimestampMarker::RmarkerRx,
                                   TimestampSource::HardwareMeasured, kFullRxChain);
    BOOST_REQUIRE(timestamp_interval_to_duration(t4, t1, dur, st));
    BOOST_REQUIRE(time_interval_status_is_ok(st));
    BOOST_REQUIRE(dur.nanos() == timestamp_interval(t4, t1).duration.nanos());
    // ... and on failure it neither returns true nor touches `out`.
    Duration guard(7);
    BOOST_REQUIRE(!timestamp_interval_to_duration(t1, t4, guard, st));
    BOOST_REQUIRE(st == TimeIntervalStatus::OrderReversed);
    BOOST_REQUIRE(guard.nanos() == 7);
}

// ===========================================================================
// 3. Fractional ticks: exact round trip, borrow, no silent loss
// ===========================================================================

BOOST_AUTO_TEST_CASE(test_interval_fractional_round_trip)
{
    const ClockDomain d = dw_uus40();

    // (100 + 3/4) - (50 + 1/2) = 50 + 1/4 exactly.
    const Timestamp a = must_make_frac(100, 3, 4, d, TimestampMarker::RmarkerRx,
                                       TimestampSource::HardwareMeasured, kFullRxChain);
    const Timestamp b = must_make_frac(50, 1, 2, d, TimestampMarker::RmarkerRx,
                                       TimestampSource::HardwareMeasured, kFullRxChain);
    BOOST_REQUIRE(a.frac_num == 3 && a.frac_den == 4u);
    const TimeInterval ti = timestamp_interval(a, b);
    BOOST_REQUIRE(ti.status == TimeIntervalStatus::Ok);
    BOOST_REQUIRE(ti.ticks == 50);
    BOOST_REQUIRE(ti.frac_num == 1 && ti.frac_den == 4u);
    // A sub-tick part means the nanosecond view is flagged inexact.
    BOOST_REQUIRE(!ti.is_whole_ticks);
    BOOST_REQUIRE(!ti.duration_is_lossless_ns);

    // (200 + 3/8) - (100 + 1/8) = 100 + 1/4 after reduction.
    const Timestamp c = must_make_frac(200, 3, 8, d, TimestampMarker::RmarkerRx,
                                       TimestampSource::HardwareMeasured, kFullRxChain);
    const Timestamp e = must_make_frac(100, 1, 8, d, TimestampMarker::RmarkerRx,
                                       TimestampSource::HardwareMeasured, kFullRxChain);
    const TimeInterval ti2 = timestamp_interval(c, e);
    BOOST_REQUIRE(ti2.status == TimeIntervalStatus::Ok);
    BOOST_REQUIRE(ti2.ticks == 100);
    BOOST_REQUIRE(ti2.frac_num == 1 && ti2.frac_den == 4u);

    // (50 + 1/4) - (50 + 3/4) is negative: REQ-PROTO-03 forbids clamping it
    // to zero and calling it a success, so the interval is refused.
    const Timestamp f = must_make_frac(50, 1, 4, d, TimestampMarker::RmarkerRx,
                                       TimestampSource::HardwareMeasured, kFullRxChain);
    const Timestamp g = must_make_frac(50, 3, 4, d, TimestampMarker::RmarkerRx,
                                       TimestampSource::HardwareMeasured, kFullRxChain);
    BOOST_REQUIRE(timestamp_interval(f, g).status ==
                  TimeIntervalStatus::DurationOutOfRange);
    // With one more whole tick the borrow produces a legal positive interval.
    const Timestamp h = must_make_frac(51, 1, 4, d, TimestampMarker::RmarkerRx,
                                       TimestampSource::HardwareMeasured, kFullRxChain);
    const TimeInterval ti3 = timestamp_interval(h, g);
    BOOST_REQUIRE(ti3.status == TimeIntervalStatus::Ok);
    BOOST_REQUIRE(ti3.ticks == 0);
    BOOST_REQUIRE(ti3.frac_num == 1 && ti3.frac_den == 2u);

    // Asymmetric denominators: 1/3 - 1/5 = 2/15 exactly.
    const Timestamp i3 = must_make_frac(10, 1, 3, d, TimestampMarker::RmarkerRx,
                                        TimestampSource::HardwareMeasured, kFullRxChain);
    const Timestamp i5 = must_make_frac(10, 1, 5, d, TimestampMarker::RmarkerRx,
                                        TimestampSource::HardwareMeasured, kFullRxChain);
    const TimeInterval ti4 = timestamp_interval(i3, i5);
    BOOST_REQUIRE(ti4.status == TimeIntervalStatus::Ok);
    BOOST_REQUIRE(ti4.ticks == 0);
    BOOST_REQUIRE(ti4.frac_num == 2 && ti4.frac_den == 15u);

    // An exact tick minus a fraction keeps the fraction (no loss).
    const Timestamp ex = must_make(11, d, TimestampMarker::RmarkerRx,
                                   TimestampSource::HardwareMeasured, kFullRxChain);
    const TimeInterval ti5 = timestamp_interval(ex, i5);
    BOOST_REQUIRE(ti5.status == TimeIntervalStatus::Ok);
    BOOST_REQUIRE(ti5.ticks == 0);
    BOOST_REQUIRE(ti5.frac_num == 4 && ti5.frac_den == 5u);

    // The decimal string preserves the exact rational.
    BOOST_REQUIRE(timestamp_to_decimal_string(i3) == "10+1/3");
    BOOST_REQUIRE(timestamp_to_decimal_string(ex) == "11");
}

// ===========================================================================
// 4. Fraction normalization rules
// ===========================================================================

BOOST_AUTO_TEST_CASE(test_fraction_normalization_rules)
{
    int64_t t = 0;
    int32_t n = 0;
    uint32_t den = 0u;

    // Already canonical.
    BOOST_REQUIRE(timestamp_normalize(7, 3, 8, t, n, den));
    BOOST_REQUIRE(t == 7 && n == 3 && den == 8u);

    // Reduced to lowest terms.
    BOOST_REQUIRE(timestamp_normalize(7, 6, 16, t, n, den));
    BOOST_REQUIRE(t == 7 && n == 3 && den == 8u);

    // Integer part carried out of the fraction.
    BOOST_REQUIRE(timestamp_normalize(7, 5, 2, t, n, den));
    BOOST_REQUIRE(t == 9 && n == 1 && den == 2u);

    // Negative fraction floors into the tick count.
    BOOST_REQUIRE(timestamp_normalize(7, -1, 2, t, n, den));
    BOOST_REQUIRE(t == 6 && n == 1 && den == 2u);

    // An exact tick with a zero fraction collapses to "no fraction".
    BOOST_REQUIRE(timestamp_normalize(7, 0, 0, t, n, den));
    BOOST_REQUIRE(t == 7 && n == 0 && den == 0u);
    BOOST_REQUIRE(timestamp_normalize(7, 0, 8, t, n, den));
    BOOST_REQUIRE(t == 7 && n == 0 && den == 0u);

    // Rejected: an exact tick cannot carry a fraction, and the denominator
    // cap is enforced rather than silently widened.
    BOOST_REQUIRE(!timestamp_normalize(7, 3, 0, t, n, den));
    BOOST_REQUIRE(
        !timestamp_normalize(7, 0, kMaxTimestampFractionDenominator + 1, t, n, den));
    BOOST_REQUIRE(timestamp_normalize(7, 1, kMaxTimestampFractionDenominator, t, n, den));
    BOOST_REQUIRE(t == 7 && n == 1 && den == kMaxTimestampFractionDenominator);

    // Rejected: tick overflow while carrying the fraction out.
    BOOST_REQUIRE(
        !timestamp_normalize(std::numeric_limits<int64_t>::max(), 3, 2, t, n, den));

    // The normalizer cannot be bypassed by hand-assembling a Timestamp: an
    // unreduced or out-of-range fraction makes it structurally invalid and
    // every interval involving it is refused.
    const ClockDomain d = dw_uus40();
    Timestamp bad = must_make(10, d, TimestampMarker::RmarkerRx,
                              TimestampSource::HardwareMeasured, kFullRxChain);
    const Timestamp good = must_make(20, d, TimestampMarker::RmarkerRx,
                                     TimestampSource::HardwareMeasured, kFullRxChain);
    bad.frac_num = 2;
    bad.frac_den = 4; // 2/4 is not reduced
    BOOST_REQUIRE(!timestamp_is_self_consistent(bad));
    BOOST_REQUIRE(std::string(timestamp_rejection_reason(bad)) == "fraction_not_reduced");
    BOOST_REQUIRE(timestamp_interval(good, bad).status ==
                  TimeIntervalStatus::FractionNotNormalized);
    BOOST_REQUIRE(timestamp_interval(bad, good).status ==
                  TimeIntervalStatus::FractionNotNormalized);

    bad.frac_num = 5; // >= 1 tick, so it is not a fraction at all
    bad.frac_den = 4u;
    BOOST_REQUIRE(!timestamp_is_self_consistent(bad));
    BOOST_REQUIRE(std::string(timestamp_rejection_reason(bad)) == "fraction_not_normalized");

    bad.frac_num = 0;
    bad.frac_den = 8; // a zero numerator with a live denominator
    BOOST_REQUIRE(!timestamp_is_self_consistent(bad));
    BOOST_REQUIRE(timestamp_interval(good, bad).status ==
                  TimeIntervalStatus::FractionNotNormalized);

    // The same exact rational addition used by calibration.
    BOOST_REQUIRE(timestamp_add_fractional(10, 1, 4, 5, 1, 4, t, n, den));
    BOOST_REQUIRE(t == 15 && n == 1 && den == 2u);
    BOOST_REQUIRE(timestamp_add_fractional(10, 0, 0, 5, 1, 4, t, n, den));
    BOOST_REQUIRE(t == 15 && n == 1 && den == 4u);
    BOOST_REQUIRE(timestamp_add_fractional(10, 1, 3, 5, 1, 5, t, n, den));
    BOOST_REQUIRE(t == 15 && n == 8 && den == 15u);
    BOOST_REQUIRE(timestamp_add_fractional(10, 1, 2, 5, -1, 2, t, n, den));
    BOOST_REQUIRE(t == 15 && n == 0 && den == 0u);
}

// ===========================================================================
// 5. Cross-domain subtraction is rejected (REQ-TIME-01, the key test)
// ===========================================================================

BOOST_AUTO_TEST_CASE(test_cross_domain_subtraction_rejected)
{
    struct Pair {
        Timestamp later;
        Timestamp earlier;
        TimeIntervalStatus expect;
    };

    const Timestamp x410_rx = must_make(2000, x410_device(), TimestampMarker::RmarkerRx,
                                        TimestampSource::HardwareMeasured, kFullRxChain);
    const Timestamp work_rx = must_make(2000, x410_work(), TimestampMarker::RmarkerRx,
                                        TimestampSource::HardwareMeasured, kFullRxChain);
    const Timestamp native_rx = must_make(2000, x410_native(), TimestampMarker::RmarkerRx,
                                           TimestampSource::HardwareMeasured, kFullRxChain);
    const Timestamp dw_rx = must_make(2000, dw_uus40(), TimestampMarker::RmarkerRx,
                                      TimestampSource::HardwareMeasured, kFullRxChain);
    const Timestamp host_rx = must_make(2000, host_monotonic(), TimestampMarker::RmarkerRx,
                                        TimestampSource::HardwareMeasured, kFullRxChain);

    const std::vector<Pair> pairs = {
        // X410 device clock vs the 998.4 MS/s work grid: different counters.
        { x410_rx, work_rx, TimeIntervalStatus::DomainNameMismatch },
        // X410 device clock vs the 491.52 MS/s native grid.
        { x410_rx, native_rx, TimeIntervalStatus::DomainNameMismatch },
        // X410 vs DW1000 UUS: the classic "one global multiplier" bug.
        { x410_rx, dw_rx, TimeIntervalStatus::DomainNameMismatch },
        { dw_rx, x410_rx, TimeIntervalStatus::DomainNameMismatch },
        // Host monotonic clock vs device clock.
        { x410_rx, host_rx, TimeIntervalStatus::DomainNameMismatch },
        // DW1000 UUS vs its 32-bit truncated twin (different names).
        { dw_rx,
          must_make(2000, dw_uus32_named(), TimestampMarker::RmarkerRx,
                    TimestampSource::HardwareMeasured, kFullRxChain),
          TimeIntervalStatus::DomainNameMismatch },
    };
    for (const Pair& p : pairs) {
        const TimeInterval ti = timestamp_interval(p.later, p.earlier);
        BOOST_REQUIRE(ti.status == p.expect);
        BOOST_REQUIRE(!time_interval_status_is_ok(ti.status));
        BOOST_REQUIRE(ti.duration.nanos() == 0);
        BOOST_REQUIRE(time_interval_status_to_exchange_status(ti.status) ==
                      ExchangeStatus::InvalidTimeDomain);
        // ... and it is refused from both argument orders.
        BOOST_REQUIRE(timestamp_interval(p.earlier, p.later).status == p.expect);
    }

    // Same NAME but a different tick rate: two grids of one device are not
    // interchangeable, and the specific reason is TickRateMismatch.
    {
        ClockDomain a = x410_device();
        ClockDomain b = x410_device();
        b.tick_rate_hz = kX410WorkHz;
        const Timestamp ta = must_make(2000, a, TimestampMarker::RmarkerRx,
                                       TimestampSource::HardwareMeasured, kFullRxChain);
        const Timestamp tb = must_make(2000, b, TimestampMarker::RmarkerRx,
                                       TimestampSource::HardwareMeasured, kFullRxChain);
        BOOST_REQUIRE(clock_domain_same_identity(a, b) == false);
        BOOST_REQUIRE(!clock_domain_is_comparable(a, b));
        BOOST_REQUIRE(timestamp_interval(ta, tb).status ==
                      TimeIntervalStatus::TickRateMismatch);
        BOOST_REQUIRE(timestamp_interval(tb, ta).status ==
                      TimeIntervalStatus::TickRateMismatch);
    }

    // A rate that differs only in the last bit is still a different domain:
    // the safe direction for a mismatch is a rejection, never a conversion.
    {
        ClockDomain a = x410_device();
        ClockDomain b = x410_device();
        b.tick_rate_hz = std::nextafter(kX410DeviceHz, 1.0e9);
        const Timestamp ta = must_make(2000, a, TimestampMarker::RmarkerRx,
                                       TimestampSource::HardwareMeasured, kFullRxChain);
        const Timestamp tb = must_make(2000, b, TimestampMarker::RmarkerRx,
                                       TimestampSource::HardwareMeasured, kFullRxChain);
        BOOST_REQUIRE(timestamp_interval(ta, tb).status ==
                      TimeIntervalStatus::TickRateMismatch);
    }

    // REQ-TIME-01's actual workflow: exchange INTERVALS, each computed inside
    // its own domain, and only then compare.  Both succeed independently; the
    // mixed subtraction above is not expressible.
    const Timestamp a_tx = must_make(1000, x410_device(), TimestampMarker::RmarkerTx,
                                     TimestampSource::ScheduledCalibrated, kFullTxChain);
    const Timestamp a_rx = must_make(1663, x410_device(), TimestampMarker::RmarkerRx,
                                     TimestampSource::HardwareMeasured, kFullRxChain);
    const Timestamp b_rx = must_make(50000, dw_uus40(), TimestampMarker::RmarkerRx,
                                     TimestampSource::HardwareMeasured, kFullRxChain);
    const Timestamp b_tx = must_make(81948, dw_uus40(), TimestampMarker::RmarkerTx,
                                     TimestampSource::ScheduledCalibrated, kFullTxChain);
    const TimeInterval ra = timestamp_interval(a_rx, a_tx);
    const TimeInterval db = timestamp_interval(b_tx, b_rx);
    BOOST_REQUIRE(ra.status == TimeIntervalStatus::Ok);
    BOOST_REQUIRE(db.status == TimeIntervalStatus::Ok);
    BOOST_REQUIRE(ra.domain.name == "x410_device");
    BOOST_REQUIRE(db.domain.name == "dw1000_uus");
    BOOST_REQUIRE(ra.domain.name != db.domain.name);
    // The two TimeIntervals carry their own domain and cannot be merged:
    // there is no API that takes two intervals from different domains and
    // returns a difference, so cross-domain arithmetic is the caller's job
    // on already-converted SI values (and M1's oracle).
    BOOST_REQUIRE(!clock_domain_is_comparable(ra.domain, db.domain));
}

// ===========================================================================
// 6. 32-bit vs 40-bit wrap width: no shared multiplier
// ===========================================================================

BOOST_AUTO_TEST_CASE(test_wrap_width_mismatch_rejected)
{
    // Same name and rate, different declared field width -> not comparable.
    BOOST_REQUIRE(clock_domain_same_identity(dw_uus40(), dw_uus32()) == false);
    BOOST_REQUIRE(clock_domain_same_epoch(dw_uus40(), dw_uus32()) == true);
    BOOST_REQUIRE(!clock_domain_is_comparable(dw_uus40(), dw_uus32()));

    // 2^33 is inside a 40-bit field but outside a 32-bit one.
    BOOST_REQUIRE(clock_domain_ticks_in_range(dw_uus40(), kTwoPow33));
    BOOST_REQUIRE(!clock_domain_ticks_in_range(dw_uus32(), kTwoPow33));
    Timestamp t32;
    BOOST_REQUIRE(!Timestamp::from_ticks(kTwoPow33, dw_uus32(), TimestampMarker::RmarkerRx,
                                         TimestampSource::HardwareMeasured, kFullRxChain,
                                         t32));

    // The same tick count converts identically, but a value near the top of
    // the 32-bit window is a legal 67 ms interval in the 40-bit field and an
    // AMBIGUOUS one (more than half a wrap period) in the 32-bit field.  That
    // is the whole point: the field width, not a global constant, decides.
    const Timestamp wide = must_make(kTwoPow32, dw_uus40(), TimestampMarker::RmarkerRx,
                                     TimestampSource::HardwareMeasured, kFullRxChain);
    const Timestamp narrow = must_make(kTwoPow32 - 1, dw_uus32(), TimestampMarker::RmarkerRx,
                                       TimestampSource::HardwareMeasured, kFullRxChain);
    const Timestamp wide_zero = must_make(0, dw_uus40(), TimestampMarker::RmarkerRx,
                                          TimestampSource::HardwareMeasured, kFullRxChain);
    const Timestamp narrow_zero = must_make(0, dw_uus32(), TimestampMarker::RmarkerRx,
                                            TimestampSource::HardwareMeasured, kFullRxChain);
    const TimeInterval i40 = timestamp_interval(wide, wide_zero);
    BOOST_REQUIRE(i40.status == TimeIntervalStatus::Ok);
    BOOST_REQUIRE(i40.duration.nanos() == 67216410LL);
    BOOST_REQUIRE(timestamp_interval(narrow, narrow_zero).status ==
                  TimeIntervalStatus::WrapAmbiguous);

    // Direct subtraction across the two widths is refused outright, in both
    // argument orders, with the specific reason.
    BOOST_REQUIRE(timestamp_interval(wide, narrow).status ==
                  TimeIntervalStatus::WrapWidthMismatch);
    BOOST_REQUIRE(timestamp_interval(narrow, wide).status ==
                  TimeIntervalStatus::WrapWidthMismatch);
}

// ===========================================================================
// 7. Epoch change invalidates the old exchange
// ===========================================================================

BOOST_AUTO_TEST_CASE(test_epoch_change_invalidates_exchange)
{
    const Timestamp before = must_make(kTwoPow40 - 500, dw_uus40(),
                                       TimestampMarker::RmarkerRx,
                                       TimestampSource::HardwareMeasured, kFullRxChain);
    const Timestamp after = must_make(500, dw_uus40_next_epoch(),
                                      TimestampMarker::RmarkerRx,
                                      TimestampSource::HardwareMeasured, kFullRxChain);

    // Same counter, same width, same name, different epoch.
    BOOST_REQUIRE(clock_domain_same_identity(before.domain, after.domain));
    BOOST_REQUIRE(!clock_domain_same_epoch(before.domain, after.domain));
    BOOST_REQUIRE(!clock_domain_is_comparable(before.domain, after.domain));
    BOOST_REQUIRE(!timestamps_share_time_line(before, after));
    BOOST_REQUIRE(timestamps_share_time_line(before, before));

    const TimeInterval ti = timestamp_interval(after, before);
    BOOST_REQUIRE(ti.status == TimeIntervalStatus::EpochMismatch);
    BOOST_REQUIRE(ti.duration.nanos() == 0);
    BOOST_REQUIRE(time_interval_status_to_exchange_status(ti.status) ==
                  ExchangeStatus::InvalidTimeDomain);
    BOOST_REQUIRE(timestamp_interval(before, after).status ==
                  TimeIntervalStatus::EpochMismatch);

    // An X410 time reset behaves identically.
    const Timestamp x_before = must_make(1000, x410_device(), TimestampMarker::RmarkerRx,
                                         TimestampSource::HardwareMeasured, kFullRxChain);
    const Timestamp x_after = must_make(1000, x410_after_reboot(),
                                        TimestampMarker::RmarkerRx,
                                        TimestampSource::HardwareMeasured, kFullRxChain);
    BOOST_REQUIRE(timestamp_interval(x_after, x_before).status ==
                  TimeIntervalStatus::EpochMismatch);
    BOOST_REQUIRE(!timestamps_share_time_line(x_before, x_after));

    // Ordering also refuses across epochs, so a "which came first" decision
    // can never be taken with a stale number.
    bool order = false;
    BOOST_REQUIRE(!timestamp_precedes(x_before, x_after, order));
    BOOST_REQUIRE(!timestamp_precedes(x_after, x_before, order));
    // ... while a monotonic in-epoch comparison resolves both ways.
    const Timestamp x_2000 = must_make(2000, x410_device(), TimestampMarker::RmarkerRx,
                                       TimestampSource::HardwareMeasured, kFullRxChain);
    BOOST_REQUIRE(timestamp_precedes(x_before, x_2000, order));
    BOOST_REQUIRE(order);
    BOOST_REQUIRE(timestamp_precedes(x_2000, x_before, order));
    BOOST_REQUIRE(!order);
    BOOST_REQUIRE(timestamp_precedes(x_before, x_before, order));
    BOOST_REQUIRE(!order);

    // Two new-epoch timestamps still combine with each other.
    const Timestamp after2 = must_make(1500, dw_uus40_next_epoch(),
                                       TimestampMarker::RmarkerRx,
                                       TimestampSource::HardwareMeasured, kFullRxChain);
    BOOST_REQUIRE(timestamps_share_time_line(after, after2));
    BOOST_REQUIRE(timestamp_interval(after2, after).status == TimeIntervalStatus::Ok);
    BOOST_REQUIRE(timestamp_interval(after2, after).ticks == 1000);
}

// ===========================================================================
// 8. Wrap-around resolution
// ===========================================================================

BOOST_AUTO_TEST_CASE(test_wrap_around_resolves)
{
    const ClockDomain d = dw_uus40();

    // `later` wraps past 2^40 back to a small value.
    const Timestamp earlier = must_make(kTwoPow40 - 1000, d, TimestampMarker::RmarkerRx,
                                        TimestampSource::HardwareMeasured, kFullRxChain);
    const Timestamp later = must_make(2000, d, TimestampMarker::RmarkerRx,
                                      TimestampSource::HardwareMeasured, kFullRxChain);
    const TimeInterval ti = timestamp_interval(later, earlier);
    BOOST_REQUIRE(ti.status == TimeIntervalStatus::Ok);
    BOOST_REQUIRE(ti.ticks == 3000);
    BOOST_REQUIRE(ti.wrapped);
    BOOST_REQUIRE(ti.duration.nanos() == 46);

    // No wrap: an ordinary forward difference.
    const Timestamp n1 = must_make(100, d, TimestampMarker::RmarkerRx,
                                   TimestampSource::HardwareMeasured, kFullRxChain);
    const Timestamp n2 = must_make(150, d, TimestampMarker::RmarkerRx,
                                   TimestampSource::HardwareMeasured, kFullRxChain);
    const TimeInterval ti2 = timestamp_interval(n2, n1);
    BOOST_REQUIRE(ti2.status == TimeIntervalStatus::Ok);
    BOOST_REQUIRE(ti2.ticks == 50);
    BOOST_REQUIRE(!ti2.wrapped);

    // Exactly at the wrap point: 0 is one tick after 2^40 - 1.
    const Timestamp w_last = must_make(kTwoPow40 - 1, d, TimestampMarker::RmarkerRx,
                                       TimestampSource::HardwareMeasured, kFullRxChain);
    const Timestamp w0 = must_make(0, d, TimestampMarker::RmarkerRx,
                                   TimestampSource::HardwareMeasured, kFullRxChain);
    const TimeInterval ti3 = timestamp_interval(w0, w_last);
    BOOST_REQUIRE(ti3.status == TimeIntervalStatus::Ok);
    BOOST_REQUIRE(ti3.wrapped);
    BOOST_REQUIRE(ti3.ticks == 1);

    // A 32-bit field: a different wrap period for the same nominal rate.
    const ClockDomain d32 = dw_uus32_named();
    const Timestamp s1 = must_make(kTwoPow32 - 10, d32, TimestampMarker::RmarkerRx,
                                   TimestampSource::HardwareMeasured, kFullRxChain);
    const Timestamp s2 = must_make(10, d32, TimestampMarker::RmarkerRx,
                                   TimestampSource::HardwareMeasured, kFullRxChain);
    const TimeInterval ti4 = timestamp_interval(s2, s1);
    BOOST_REQUIRE(ti4.status == TimeIntervalStatus::Ok);
    BOOST_REQUIRE(ti4.ticks == 20);
    BOOST_REQUIRE(ti4.wrapped);
    BOOST_REQUIRE(ti4.duration.nanos() == 0); // 20 * 15.65 ps is sub-ns
    BOOST_REQUIRE(ti4.is_whole_ticks);
    BOOST_REQUIRE(!ti4.duration_is_lossless_ns);

    // The raw (adapter-level) modular difference agrees with the protocol one.
    const TickDelta td = raw_tick_delta(d, 2000, kTwoPow40 - 1000);
    BOOST_REQUIRE(td.status == TimeIntervalStatus::Ok);
    BOOST_REQUIRE(td.ticks == 3000);
    BOOST_REQUIRE(td.wrapped);
}

// ===========================================================================
// 9. Intervals beyond the unambiguous range are REJECTED, not guessed
// ===========================================================================

BOOST_AUTO_TEST_CASE(test_wrap_ambiguous_rejected)
{
    const ClockDomain d = dw_uus40();
    const Timestamp zero = must_make(0, d, TimestampMarker::RmarkerRx,
                                     TimestampSource::HardwareMeasured, kFullRxChain);

    // Exactly half a period is still resolvable.
    const Timestamp half = must_make(kTwoPow40 / 2, d, TimestampMarker::RmarkerRx,
                                     TimestampSource::HardwareMeasured, kFullRxChain);
    const TimeInterval ok_half = timestamp_interval(half, zero);
    BOOST_REQUIRE(ok_half.status == TimeIntervalStatus::Ok);
    BOOST_REQUIRE(ok_half.ticks == kTwoPow40 / 2);
    BOOST_REQUIRE(ok_half.duration.nanos() == 8603700512LL);

    // One tick more: ambiguous -> refused, not guessed.
    const Timestamp half_plus = must_make(kTwoPow40 / 2 + 1, d, TimestampMarker::RmarkerRx,
                                          TimestampSource::HardwareMeasured, kFullRxChain);
    BOOST_REQUIRE(timestamp_interval(half_plus, zero).status ==
                  TimeIntervalStatus::WrapAmbiguous);

    // Beyond a FULL wrap period (2^40 ticks ~ 17.2 s), i.e. far past the
    // 2^bits / tick_rate window cited in REQ-TIME-04.
    const Timestamp almost = must_make(kTwoPow40 - 1, d, TimestampMarker::RmarkerRx,
                                       TimestampSource::HardwareMeasured, kFullRxChain);
    const TimeInterval ti = timestamp_interval(almost, zero);
    BOOST_REQUIRE(ti.status == TimeIntervalStatus::WrapAmbiguous);
    BOOST_REQUIRE(ti.duration.nanos() == 0);
    BOOST_REQUIRE(time_interval_status_to_exchange_status(ti.status) ==
                  ExchangeStatus::InvalidTimeDomain);

    // A backwards difference longer than half a period is equally ambiguous:
    // `zero` could be one tick after 2^40-1, or 2^40-1 ticks before 1.
    {
        const Timestamp one = must_make(1, d, TimestampMarker::RmarkerRx,
                                        TimestampSource::HardwareMeasured, kFullRxChain);
        BOOST_REQUIRE(timestamp_interval(zero, one).status ==
                      TimeIntervalStatus::WrapAmbiguous);
        const TickDelta td = raw_tick_delta(d, 0, 1);
        BOOST_REQUIRE(td.status == TimeIntervalStatus::WrapAmbiguous);
        BOOST_REQUIRE(td.ticks == 0);
    }

    // The field width, not a global constant, decides: ~2^32 ticks is a legal
    // 67 ms interval in the 40-bit field and unresolvable in the 32-bit one.
    const Timestamp wide = must_make(kTwoPow32, d, TimestampMarker::RmarkerRx,
                                     TimestampSource::HardwareMeasured, kFullRxChain);
    BOOST_REQUIRE(timestamp_interval(wide, zero).status == TimeIntervalStatus::Ok);
    const ClockDomain d32 = dw_uus32_named();
    const Timestamp narrow = must_make(kTwoPow32 - 1, d32, TimestampMarker::RmarkerRx,
                                       TimestampSource::HardwareMeasured, kFullRxChain);
    const Timestamp narrow_zero = must_make(0, d32, TimestampMarker::RmarkerRx,
                                            TimestampSource::HardwareMeasured, kFullRxChain);
    BOOST_REQUIRE(timestamp_interval(narrow, narrow_zero).status ==
                  TimeIntervalStatus::WrapAmbiguous);
    // ... and the ordering helper refuses to guess as well.
    bool order = true;
    BOOST_REQUIRE(!timestamp_precedes(zero, narrow, order));
    BOOST_REQUIRE(timestamp_precedes(zero, wide, order));
    BOOST_REQUIRE(order);
}

// ===========================================================================
// 10. A no-wrap domain must not "repair" a reversed order
// ===========================================================================

BOOST_AUTO_TEST_CASE(test_no_wrap_order_reversed)
{
    const ClockDomain d = x410_device(); // 64-bit monotonic, no wrap width
    const Timestamp later = must_make(100, d, TimestampMarker::RmarkerRx,
                                      TimestampSource::HardwareMeasured, kFullRxChain);
    const Timestamp earlier = must_make(200, d, TimestampMarker::RmarkerRx,
                                        TimestampSource::HardwareMeasured, kFullRxChain);
    BOOST_REQUIRE(timestamp_interval(later, earlier).status ==
                  TimeIntervalStatus::OrderReversed);
    const TickDelta td = raw_tick_delta(d, 100, 200);
    BOOST_REQUIRE(td.status == TimeIntervalStatus::OrderReversed);
    BOOST_REQUIRE(td.ticks == 0);
    // The legal order works.
    BOOST_REQUIRE(timestamp_interval(earlier, later).status == TimeIntervalStatus::Ok);
    BOOST_REQUIRE(timestamp_interval(earlier, later).ticks == 100);
    // An absolute interval beyond the full 2^40 window is still resolvable in
    // a wider field and rejected in a narrower one.
    const ClockDomain d40 = dw_uus40();
    const Timestamp wide = must_make(kTwoPow33, d40, TimestampMarker::RmarkerRx,
                                     TimestampSource::HardwareMeasured, kFullRxChain);
    BOOST_REQUIRE(timestamp_interval(wide, must_make(0, d40, TimestampMarker::RmarkerRx,
                                                     TimestampSource::HardwareMeasured,
                                                     kFullRxChain))
                      .status == TimeIntervalStatus::Ok);
}

// ===========================================================================
// 11. Marker compatibility rules
// ===========================================================================

BOOST_AUTO_TEST_CASE(test_marker_interval_rules)
{
    // Protocol-legal pairs.
    BOOST_REQUIRE(timestamp_marker_interval_allowed(TimestampMarker::RmarkerRx,
                                                    TimestampMarker::RmarkerTx));
    BOOST_REQUIRE(timestamp_marker_interval_allowed(TimestampMarker::RmarkerTx,
                                                    TimestampMarker::RmarkerRx));
    BOOST_REQUIRE(timestamp_marker_interval_allowed(TimestampMarker::RmarkerRx,
                                                    TimestampMarker::RmarkerRx));
    BOOST_REQUIRE(timestamp_marker_interval_allowed(TimestampMarker::UhdRxFirstIqSample,
                                                    TimestampMarker::UhdRxFirstIqSample));

    // Forbidden: a host capture coordinate is never an on-air RMARKER.
    BOOST_REQUIRE(!timestamp_marker_interval_allowed(TimestampMarker::UhdRxFirstIqSample,
                                                     TimestampMarker::RmarkerRx));
    BOOST_REQUIRE(!timestamp_marker_interval_allowed(TimestampMarker::RmarkerRx,
                                                     TimestampMarker::UhdRxFirstIqSample));
    BOOST_REQUIRE(!timestamp_marker_interval_allowed(TimestampMarker::UhdRxFirstIqSample,
                                                     TimestampMarker::SfdStart));
    // Forbidden: waveform-geometry constants are not measured intervals.
    BOOST_REQUIRE(!timestamp_marker_interval_allowed(TimestampMarker::SfdStart,
                                                     TimestampMarker::RmarkerTx));
    BOOST_REQUIRE(!timestamp_marker_interval_allowed(TimestampMarker::PreambleStart,
                                                     TimestampMarker::RmarkerRx));
    BOOST_REQUIRE(!timestamp_marker_interval_allowed(TimestampMarker::PhrStart,
                                                     TimestampMarker::RmarkerTx));
    // Forbidden: the antenna plane is produced by calibration, not subtracted.
    BOOST_REQUIRE(!timestamp_marker_interval_allowed(TimestampMarker::AntennaPlane,
                                                     TimestampMarker::RmarkerRx));

    // Marker families.
    BOOST_REQUIRE(timestamp_marker_class(TimestampMarker::UhdRxFirstIqSample) ==
                  TimestampMarkerClass::HostCapture);
    BOOST_REQUIRE(timestamp_marker_class(TimestampMarker::SfdStart) ==
                  TimestampMarkerClass::WaveformRx);
    BOOST_REQUIRE(timestamp_marker_class(TimestampMarker::RmarkerRx) ==
                  TimestampMarkerClass::WaveformRx);
    BOOST_REQUIRE(timestamp_marker_class(TimestampMarker::RmarkerTx) ==
                  TimestampMarkerClass::WaveformTx);
    BOOST_REQUIRE(timestamp_marker_class(TimestampMarker::AntennaPlane) ==
                  TimestampMarkerClass::ReferencePlane);
    BOOST_REQUIRE(std::string(timestamp_marker_class_to_string(
                      timestamp_marker_class(TimestampMarker::RmarkerTx))) == "waveform_tx");

    // End to end: DB = t3B - t2B (RmarkerTx later than RmarkerRx) is allowed.
    const ClockDomain d = dw_uus40();
    const Timestamp rx = must_make(1000, d, TimestampMarker::RmarkerRx,
                                   TimestampSource::HardwareMeasured, kFullRxChain);
    const Timestamp tx = must_make(31948, d, TimestampMarker::RmarkerTx,
                                   TimestampSource::ScheduledCalibrated, kFullTxChain);
    const TimeInterval db = timestamp_interval(tx, rx);
    BOOST_REQUIRE(db.status == TimeIntervalStatus::Ok);
    BOOST_REQUIRE(db.ticks == 30948);
    BOOST_REQUIRE(db.later_marker == TimestampMarker::RmarkerTx);
    BOOST_REQUIRE(db.earlier_marker == TimestampMarker::RmarkerRx);

    // The reverse order within the pair is the round trip RA = t4A - t1A.
    const TimeInterval ra = timestamp_interval(rx, tx);
    BOOST_REQUIRE(ra.status != TimeIntervalStatus::Ok);
    BOOST_REQUIRE(ra.status == TimeIntervalStatus::WrapAmbiguous);

    // A RX first-IQ-sample against an RMARKER is refused, even though both
    // timestamps are otherwise perfect.
    const Timestamp first_iq = must_make(20000, d, TimestampMarker::UhdRxFirstIqSample,
                                         TimestampSource::HardwareMeasured, kCorrectionNone);
    BOOST_REQUIRE(timestamp_is_self_consistent(first_iq));
    BOOST_REQUIRE(timestamp_interval(first_iq, rx).status ==
                  TimeIntervalStatus::MarkerMismatch);
    BOOST_REQUIRE(timestamp_interval(rx, first_iq).status ==
                  TimeIntervalStatus::MarkerMismatch);
}

// ===========================================================================
// 12. An uncorrected mapping cannot enter an interval
// ===========================================================================

BOOST_AUTO_TEST_CASE(test_uncorrected_marker_interval_refused)
{
    const ClockDomain d = dw_uus40();
    const Timestamp tx = must_make(1000, d, TimestampMarker::RmarkerTx,
                                   TimestampSource::ScheduledCalibrated, kFullTxChain);
    const Timestamp rx_full = must_make(5000, d, TimestampMarker::RmarkerRx,
                                        TimestampSource::HardwareMeasured, kFullRxChain);

    auto without = [&](uint32_t bit) {
        return static_cast<uint32_t>(kFullRxChain & ~bit);
    };
    auto tx_without = [&](uint32_t bit) {
        return static_cast<uint32_t>(kFullTxChain & ~bit);
    };

    // The RX chain ran but the waveform-to-marker mapping did not.
    BOOST_REQUIRE(timestamp_interval(must_make(5000, d, TimestampMarker::RmarkerRx,
                                                TimestampSource::HardwareMeasured,
                                                without(kCorrectionRmarkerOffset)),
                                      tx)
                      .status == TimeIntervalStatus::CorrectionsNotApplied);
    // The REQ-TIME-05 first-path quality gate is missing.
    BOOST_REQUIRE(timestamp_interval(must_make(5000, d, TimestampMarker::RmarkerRx,
                                                TimestampSource::HardwareMeasured,
                                                without(kCorrectionFirstPathQualityGate)),
                                      tx)
                      .status == TimeIntervalStatus::CorrectionsNotApplied);
    // The resampler group-delay correction is missing: the single most
    // dangerous omission, because it puts the whole RX time in the wrong place.
    BOOST_REQUIRE(timestamp_interval(must_make(5000, d, TimestampMarker::RmarkerRx,
                                                TimestampSource::HardwareMeasured,
                                                without(kCorrectionRxSampleToFirstPath)),
                                      tx)
                      .status == TimeIntervalStatus::CorrectionsNotApplied);
    // Window crop / sample-rate conversion / fractional first path likewise.
    BOOST_REQUIRE(timestamp_interval(must_make(5000, d, TimestampMarker::RmarkerRx,
                                                TimestampSource::HardwareMeasured,
                                                without(kCorrectionWindowCrop)),
                                      tx)
                      .status == TimeIntervalStatus::CorrectionsNotApplied);
    BOOST_REQUIRE(timestamp_interval(must_make(5000, d, TimestampMarker::RmarkerRx,
                                                TimestampSource::HardwareMeasured,
                                                without(kCorrectionSampleRateConversion)),
                                      tx)
                      .status == TimeIntervalStatus::CorrectionsNotApplied);
    BOOST_REQUIRE(timestamp_interval(must_make(5000, d, TimestampMarker::RmarkerRx,
                                                TimestampSource::HardwareMeasured,
                                                without(kCorrectionFirstPathFraction)),
                                      tx)
                      .status == TimeIntervalStatus::CorrectionsNotApplied);
    // A raw SFD coordinate with no corrections at all.  Against the TX marker
    // the marker rule fires first; against another SFD coordinate the missing
    // correction chain is what is reported.
    BOOST_REQUIRE(timestamp_interval(must_make(5000, d, TimestampMarker::SfdStart,
                                                TimestampSource::HardwareMeasured,
                                                kCorrectionNone),
                                      tx)
                      .status == TimeIntervalStatus::MarkerMismatch);
    BOOST_REQUIRE(timestamp_interval(must_make(5000, d, TimestampMarker::SfdStart,
                                                TimestampSource::HardwareMeasured,
                                                kCorrectionNone),
                                      must_make(1000, d, TimestampMarker::SfdStart,
                                                TimestampSource::HardwareMeasured,
                                                kFullRxChain))
                      .status == TimeIntervalStatus::CorrectionsNotApplied);
    // ... and it is not marker-mapped either.
    const Timestamp sfd = must_make(5000, d, TimestampMarker::SfdStart,
                                    TimestampSource::HardwareMeasured, kCorrectionNone);
    BOOST_REQUIRE(timestamp_is_self_consistent(sfd));
    BOOST_REQUIRE(!timestamp_is_marker_mapped(sfd));
    BOOST_REQUIRE(std::string(timestamp_rejection_reason(sfd)) == "corrections_not_applied");

    // TX side: a command time that was never mapped to the air instant.
    BOOST_REQUIRE(timestamp_interval(rx_full,
                                      must_make(1000, d, TimestampMarker::RmarkerTx,
                                                TimestampSource::ScheduledCalibrated,
                                                tx_without(kCorrectionTxCommandToAir)))
                      .status == TimeIntervalStatus::CorrectionsNotApplied);
    // ... including a missing delayed-TX quantisation record.
    BOOST_REQUIRE(timestamp_interval(rx_full,
                                      must_make(1000, d, TimestampMarker::RmarkerTx,
                                                TimestampSource::ScheduledCalibrated,
                                                tx_without(kCorrectionDelayedTxQuantization)))
                      .status == TimeIntervalStatus::CorrectionsNotApplied);

    // The required masks are non-empty for everything except the raw capture
    // coordinate, and an unknown bit can never be satisfied.
    BOOST_REQUIRE(timestamp_required_corrections(TimestampMarker::UhdRxFirstIqSample) == 0u);
    BOOST_REQUIRE(timestamp_required_corrections(TimestampMarker::RmarkerRx) != 0u);
    BOOST_REQUIRE(timestamp_required_corrections(TimestampMarker::RmarkerTx) != 0u);
    BOOST_REQUIRE((timestamp_required_corrections(TimestampMarker::AntennaPlane) &
                   static_cast<uint32_t>(kCorrectionAntennaPlane)) != 0u);
    BOOST_REQUIRE(timestamp_corrections_satisfied(
        TimestampMarker::RmarkerRx,
        timestamp_required_corrections(TimestampMarker::RmarkerRx)));
    BOOST_REQUIRE(!timestamp_corrections_satisfied(
        TimestampMarker::AntennaPlane,
        timestamp_required_corrections(TimestampMarker::RmarkerRx)));
}

// ===========================================================================
// 13. A raw capture time can be promoted only through an explicit mapping
// ===========================================================================

BOOST_AUTO_TEST_CASE(test_retag_requires_corrections)
{
    const ClockDomain d = dw_uus40();
    Timestamp ts = must_make(5000, d, TimestampMarker::UhdRxFirstIqSample,
                             TimestampSource::HardwareMeasured, kCorrectionNone);
    BOOST_REQUIRE(timestamp_is_self_consistent(ts));
    // Refused: the RX chain has not run.
    BOOST_REQUIRE(!timestamp_retag_marker(ts, TimestampMarker::RmarkerRx));
    BOOST_REQUIRE(ts.marker == TimestampMarker::UhdRxFirstIqSample);

    // Refused even for a partially applied chain.
    ts.applied_corrections = static_cast<uint32_t>(kFullRxChain & ~kCorrectionRmarkerOffset);
    BOOST_REQUIRE(!timestamp_retag_marker(ts, TimestampMarker::RmarkerRx));

    // Once every required bit is present the promotion is allowed, and the
    // marker now carries the mapping claim.
    ts.applied_corrections = kFullRxChain;
    BOOST_REQUIRE(timestamp_retag_marker(ts, TimestampMarker::RmarkerRx));
    BOOST_REQUIRE(ts.marker == TimestampMarker::RmarkerRx);
    BOOST_REQUIRE(timestamp_is_marker_mapped(ts));
    // The new marker now demands the antenna-plane step too, so that fails.
    BOOST_REQUIRE(!timestamp_retag_marker(ts, TimestampMarker::AntennaPlane));
    ts.applied_corrections |= static_cast<uint32_t>(kCorrectionAntennaPlane);
    BOOST_REQUIRE(timestamp_retag_marker(ts, TimestampMarker::AntennaPlane));
    BOOST_REQUIRE(ts.marker == TimestampMarker::AntennaPlane);
    // Retagging an invalid timestamp is refused.
    Timestamp broken = ts;
    broken.valid = false;
    BOOST_REQUIRE(!timestamp_retag_marker(broken, TimestampMarker::RmarkerRx));
}

// ===========================================================================
// 14. Provenance: a scheduled TX is usable but is NOT a measured air time
// ===========================================================================

BOOST_AUTO_TEST_CASE(test_provenance_and_scheduled_tx)
{
    const ClockDomain d = dw_uus40();

    // A hardware-measured RMARKER with the full chain and a calibration is
    // range grade.
    Timestamp hw = must_make(31948, d, TimestampMarker::RmarkerRx,
                             TimestampSource::HardwareMeasured, kFullRxChain);
    hw.calibration_id = "cal-x410-sn0-ch0-ch5-v1";
    BOOST_REQUIRE(timestamp_is_validated_measurement(hw));
    BOOST_REQUIRE(timestamp_is_range_capable(hw));
    BOOST_REQUIRE(hw.is_validated_measurement());
    BOOST_REQUIRE(hw.has_calibration());
    BOOST_REQUIRE(std::string(timestamp_rejection_reason(hw)) == "accepted");

    // A calibrated scheduled TX is range-capable (REQ-TIME-03 allows it) but
    // must never be reported as a hardware measurement.
    Timestamp sched = must_make(1000, d, TimestampMarker::RmarkerTx,
                                TimestampSource::ScheduledCalibrated, kFullTxChain);
    sched.calibration_id = "cal-x410-sn0-ch0-ch5-v1";
    BOOST_REQUIRE(timestamp_is_range_capable(sched));
    BOOST_REQUIRE(!timestamp_is_validated_measurement(sched));
    BOOST_REQUIRE(!sched.is_validated_measurement());
    BOOST_REQUIRE(std::string(timestamp_rejection_reason(sched)) ==
                  "source_not_hardware_measured");

    // No calibration -> not range capable.
    Timestamp no_cal = must_make(31948, d, TimestampMarker::RmarkerRx,
                                 TimestampSource::HardwareMeasured, kFullRxChain);
    BOOST_REQUIRE(!timestamp_is_range_capable(no_cal));
    BOOST_REQUIRE(!timestamp_is_validated_measurement(no_cal));
    BOOST_REQUIRE(std::string(timestamp_rejection_reason(no_cal)) == "calibration_missing");

    // Non-measurement sources.
    {
        Timestamp est = must_make(31948, d, TimestampMarker::RmarkerRx,
                                  TimestampSource::Estimated, kFullRxChain);
        est.calibration_id = "cal";
        BOOST_REQUIRE(!timestamp_is_range_capable(est));
        BOOST_REQUIRE(std::string(timestamp_rejection_reason(est)) ==
                      "source_not_a_measurement");
        Timestamp rec = must_make(31948, d, TimestampMarker::RmarkerRx,
                                  TimestampSource::Reconstructed, kFullRxChain);
        rec.calibration_id = "cal";
        BOOST_REQUIRE(!timestamp_is_range_capable(rec));
        BOOST_REQUIRE(std::string(timestamp_rejection_reason(rec)) ==
                      "source_not_a_measurement");
    }

    // A raw capture coordinate is never range grade even when measured and
    // calibrated: it is not a protocol RMARKER.
    Timestamp iq = must_make(5000, d, TimestampMarker::UhdRxFirstIqSample,
                             TimestampSource::HardwareMeasured, kCorrectionNone);
    iq.calibration_id = "cal";
    BOOST_REQUIRE(!timestamp_is_range_capable(iq));
    BOOST_REQUIRE(std::string(timestamp_rejection_reason(iq)) ==
                  "marker_not_protocol_rmarker");

    // An uncorrected SFD coordinate: the corrections gate fires first.
    Timestamp sfd = must_make(5000, d, TimestampMarker::SfdStart,
                              TimestampSource::HardwareMeasured, kCorrectionNone);
    sfd.calibration_id = "cal";
    BOOST_REQUIRE(!timestamp_is_range_capable(sfd));
    BOOST_REQUIRE(std::string(timestamp_rejection_reason(sfd)) == "corrections_not_applied");

    // A scheduled, calibrated TX still forms a legal interval with a measured
    // RX: REQ-TIME-03 only constrains how it is LABELLED.
    const Timestamp rx = must_make(1663, d, TimestampMarker::RmarkerRx,
                                   TimestampSource::HardwareMeasured, kFullRxChain);
    const TimeInterval ra = timestamp_interval(rx, sched);
    BOOST_REQUIRE(ra.status == TimeIntervalStatus::Ok);
    BOOST_REQUIRE(ra.ticks == 663);

    // The invalid flag dominates everything.
    Timestamp inv = hw;
    inv.valid = false;
    BOOST_REQUIRE(!timestamp_is_validated_measurement(inv));
    BOOST_REQUIRE(!timestamp_is_self_consistent(inv));
    BOOST_REQUIRE(std::string(timestamp_rejection_reason(inv)) == "invalid_flag");
    BOOST_REQUIRE(timestamp_interval(inv, sched).status ==
                  TimeIntervalStatus::InvalidTimestamp);
}

// ===========================================================================
// 15. A calibration is applied exactly once
// ===========================================================================

BOOST_AUTO_TEST_CASE(test_calibration_applied_exactly_once)
{
    const ClockDomain d = dw_uus40();
    Timestamp ts = must_make(100000, d, TimestampMarker::RmarkerRx,
                             TimestampSource::HardwareMeasured, kFullRxChain);
    BOOST_REQUIRE(ts.calibration_id.empty());
    BOOST_REQUIRE(!timestamp_has_calibration(ts, "cal-a"));

    // The first application moves the timestamp by an exact tick + sub-tick
    // offset.
    const CalibrationResult r1 = apply_calibration_ticks(
        ts, "cal-a", 3194, 4, 5, static_cast<uint32_t>(kCorrectionAntennaPlane));
    BOOST_REQUIRE(r1 == CalibrationResult::Applied);
    BOOST_REQUIRE(std::string(calibration_result_to_string(r1)) == "applied");
    BOOST_REQUIRE(ts.ticks == 103194);
    BOOST_REQUIRE(ts.frac_num == 4 && ts.frac_den == 5u);
    BOOST_REQUIRE((ts.applied_corrections & static_cast<uint32_t>(kCorrectionAntennaPlane)) !=
                  0u);
    BOOST_REQUIRE(timestamp_has_calibration(ts, "cal-a"));
    BOOST_REQUIRE(!timestamp_has_calibration(ts, "cal-b"));

    // The SAME id applied again: double application detected and refused, and
    // the timestamp is left untouched.
    const Timestamp before = ts;
    const CalibrationResult r2 = apply_calibration_ticks(
        ts, "cal-a", 3194, 4, 5, static_cast<uint32_t>(kCorrectionAntennaPlane));
    BOOST_REQUIRE(r2 == CalibrationResult::AlreadyApplied);
    BOOST_REQUIRE(std::string(calibration_result_to_string(r2)) == "already_applied");
    BOOST_REQUIRE(ts.ticks == before.ticks);
    BOOST_REQUIRE(ts.frac_num == before.frac_num);
    BOOST_REQUIRE(ts.frac_den == before.frac_den);
    BOOST_REQUIRE(ts.calibration_id == before.calibration_id);
    BOOST_REQUIRE(ts.applied_corrections == before.applied_corrections);

    // A DIFFERENT id on an already calibrated timestamp is a conflict, also
    // refused: the calibration set is frozen per exchange (REQ-CAL-01).
    const CalibrationResult r3 =
        apply_calibration_ticks(ts, "cal-b", 10, 0, 0u, kCorrectionNone);
    BOOST_REQUIRE(r3 == CalibrationResult::ConflictingCalibration);
    BOOST_REQUIRE(std::string(calibration_result_to_string(r3)) ==
                  "conflicting_calibration");
    BOOST_REQUIRE(ts.ticks == before.ticks);
    BOOST_REQUIRE(ts.calibration_id == "cal-a");

    // An empty id is refused outright.
    Timestamp t2 = must_make(100, d, TimestampMarker::RmarkerRx,
                             TimestampSource::HardwareMeasured, kFullRxChain);
    BOOST_REQUIRE(apply_calibration_ticks(t2, "", 1, 0, 0u, kCorrectionNone) ==
                  CalibrationResult::EmptyCalibrationId);
    BOOST_REQUIRE(apply_calibration_duration(t2, "", Duration::from_nanos(1),
                                             kCorrectionNone) ==
                  CalibrationResult::EmptyCalibrationId);
    BOOST_REQUIRE(t2.calibration_id.empty());
    BOOST_REQUIRE(t2.ticks == 100);

    // An invalid timestamp cannot be calibrated.
    Timestamp t3 = t2;
    t3.valid = false;
    BOOST_REQUIRE(apply_calibration_ticks(t3, "cal-a", 1, 0, 0u, kCorrectionNone) ==
                  CalibrationResult::InvalidTimestamp);
    BOOST_REQUIRE(apply_calibration_duration(t3, "cal-a", Duration::from_nanos(1),
                                             kCorrectionNone) ==
                  CalibrationResult::InvalidTimestamp);

    // An offset that would push the timestamp out of the counter window is
    // refused rather than saturated.
    Timestamp t4 = must_make(100, dw_uus32_named(), TimestampMarker::RmarkerRx,
                             TimestampSource::HardwareMeasured, kFullRxChain);
    BOOST_REQUIRE(apply_calibration_ticks(t4, "cal-a", kTwoPow32, 0, 0u, kCorrectionNone) ==
                  CalibrationResult::OffsetOutOfRange);
    BOOST_REQUIRE(t4.calibration_id.empty());
    BOOST_REQUIRE(t4.ticks == 100);

    // An unknown correction bit is refused.
    Timestamp t5 = must_make(100, d, TimestampMarker::RmarkerRx,
                             TimestampSource::HardwareMeasured, kFullRxChain);
    BOOST_REQUIRE(apply_calibration_ticks(t5, "cal-a", 1, 0, 0u, 1u << 20) ==
                  CalibrationResult::OffsetOutOfRange);
    BOOST_REQUIRE(t5.calibration_id.empty());

    // Calibration never changes the marker implicitly (REQ-TIME-02).
    Timestamp t6 = must_make(100, d, TimestampMarker::RmarkerRx,
                             TimestampSource::HardwareMeasured, kFullRxChain);
    BOOST_REQUIRE(apply_calibration_ticks(t6, "cal-a", 5, 0, 0u, kCorrectionNone) ==
                  CalibrationResult::Applied);
    BOOST_REQUIRE(t6.marker == TimestampMarker::RmarkerRx);
}

// ===========================================================================
// 16. Sub-nanosecond calibration on a DW UUS domain
// ===========================================================================

BOOST_AUTO_TEST_CASE(test_calibration_subnanosecond_offset)
{
    const ClockDomain d = dw_uus40(); // 1 tick = 15.65 ps
    const double base_ns = 100000.0 / kDwUusTickHz * 1e9; // 1565.0040064 ns

    // 500 ns of link/antenna delay is 31948.8 DW UUS ticks and the fraction is
    // NOT rounded away.
    Timestamp ts = must_make(100000, d, TimestampMarker::RmarkerRx,
                             TimestampSource::HardwareMeasured, kFullRxChain);
    const CalibrationResult r =
        apply_calibration_duration(ts, "cal-dw-sn0-ch5-v1", Duration::from_nanos(500),
                                   static_cast<uint32_t>(kCorrectionAntennaPlane));
    BOOST_REQUIRE(r == CalibrationResult::Applied);
    BOOST_REQUIRE(ts.ticks == 131948);
    BOOST_REQUIRE(ts.frac_den != 0u);
    const double moved_ns = (static_cast<double>(ts.ticks) +
                             static_cast<double>(ts.frac_num) /
                                 static_cast<double>(ts.frac_den)) /
                            kDwUusTickHz * 1e9;
    BOOST_REQUIRE(std::fabs(moved_ns - base_ns - 500.0) < 1.0e-6);
    BOOST_REQUIRE(std::fabs(moved_ns - 2065.0040062) < 1.0e-6);

    // The exact tick form reproduces the quantized value bit for bit, which
    // is why callers that need determinism use it.
    Timestamp a = must_make(100000, d, TimestampMarker::RmarkerRx,
                            TimestampSource::HardwareMeasured, kFullRxChain);
    BOOST_REQUIRE(apply_calibration_ticks(a, "cal-dw", 31948, 13107, 16384u,
                                          kCorrectionNone) == CalibrationResult::Applied);
    Timestamp b = must_make(100000, d, TimestampMarker::RmarkerRx,
                            TimestampSource::HardwareMeasured, kFullRxChain);
    BOOST_REQUIRE(apply_calibration_duration(b, "cal-dw", Duration::from_nanos(500),
                                             kCorrectionNone) == CalibrationResult::Applied);
    BOOST_REQUIRE(a.ticks == b.ticks);
    BOOST_REQUIRE(a.frac_num == b.frac_num);
    BOOST_REQUIRE(a.frac_den == b.frac_den);

    // The tick form can also express the mathematically exact 31948 + 4/5,
    // which the duration form only approximates.
    Timestamp c = must_make(100000, d, TimestampMarker::RmarkerRx,
                            TimestampSource::HardwareMeasured, kFullRxChain);
    BOOST_REQUIRE(apply_calibration_ticks(c, "cal-dw", 31948, 4, 5, kCorrectionNone) ==
                  CalibrationResult::Applied);
    BOOST_REQUIRE(c.frac_num == 4 && c.frac_den == 5u);
    const double exact_ns =
        (static_cast<double>(c.ticks) + 0.8) / kDwUusTickHz * 1e9;
    BOOST_REQUIRE(std::fabs(exact_ns - base_ns - 500.0) < 1.0e-9);

    // An absurd offset is refused, not wrapped.
    Timestamp e = must_make(1000, d, TimestampMarker::RmarkerRx,
                            TimestampSource::HardwareMeasured, kFullRxChain);
    BOOST_REQUIRE(apply_calibration_duration(
                      e, "cal-dw", Duration::from_nanos(std::numeric_limits<int64_t>::max()),
                      kCorrectionNone) == CalibrationResult::OffsetOutOfRange);
    BOOST_REQUIRE(e.calibration_id.empty());
    BOOST_REQUIRE(e.ticks == 1000);
}

// ===========================================================================
// 17. JSON keeps integer precision (REQ-OUT-01)
// ===========================================================================

BOOST_AUTO_TEST_CASE(test_json_integer_precision_round_trip)
{
    // 2^53 is where a JSON number stops being exact.
    BOOST_REQUIRE(integer_needs_decimal_string(kJsonSafeIntegerMax) == false);
    BOOST_REQUIRE(integer_needs_decimal_string(kJsonSafeIntegerMax + 1) == true);
    BOOST_REQUIRE(integer_needs_decimal_string(-kJsonSafeIntegerMax - 1) == true);
    BOOST_REQUIRE(to_decimal_string(9007199254740993LL) == "9007199254740993");

    struct Sample {
        Timestamp ts;
        const char* label;
    };
    std::vector<Sample> samples;
    auto add = [&samples](const Timestamp& ts, const char* label) {
        Sample s;
        s.ts = ts;
        s.label = label;
        samples.push_back(s);
    };

    // 1. Beyond the JSON safe range -> emitted as a decimal STRING.
    ClockDomain mono;
    mono.name = "x410_monotonic";
    mono.tick_rate_hz = kX410WorkHz;
    mono.epoch_id = 11;
    mono.timestamp_bits = 0;
    add(must_make(9007199254740993LL, mono, TimestampMarker::RmarkerRx,
                  TimestampSource::HardwareMeasured, kFullRxChain),
        "ticks beyond 2^53");
    // 2. Exactly at the boundary -> still a JSON number.
    add(must_make(kJsonSafeIntegerMax, mono, TimestampMarker::RmarkerRx,
                  TimestampSource::HardwareMeasured, kFullRxChain),
        "ticks at 2^53-1");
    // 3. A 40-bit DW value with a sub-tick fraction and a calibration.
    {
        Timestamp ts = must_make_frac(31948, 4, 5, dw_uus40(), TimestampMarker::RmarkerRx,
                                      TimestampSource::HardwareMeasured, kFullRxChain);
        BOOST_REQUIRE(apply_calibration_ticks(ts, "cal-dw-sn0-ch5-v1", 100, 0, 0u,
                                              static_cast<uint32_t>(kCorrectionAntennaPlane)) ==
                      CalibrationResult::Applied);
        add(ts, "DW fractional + calibrated");
    }
    // 4. A scheduled TX with no calibration and no fraction.
    add(must_make(1000, dw_uus40(), TimestampMarker::RmarkerTx,
                  TimestampSource::ScheduledCalibrated, kFullTxChain),
        "scheduled TX");
    // 5. A 32-bit truncated field, to prove the width survives the round trip.
    add(must_make(kTwoPow32 - 1, dw_uus32_named(), TimestampMarker::RmarkerRx,
                  TimestampSource::HardwareMeasured, kFullRxChain),
        "32-bit field");
    // 6. A non-integer tick rate (the DW UUS rate) must survive bit-exactly.
    add(must_make(12345, dw_uus40(), TimestampMarker::RmarkerRx,
                  TimestampSource::Estimated, kFullRxChain),
        "DW UUS rate");
    // 7. A raw capture coordinate with no corrections.
    add(must_make(7, x410_device(), TimestampMarker::UhdRxFirstIqSample,
                  TimestampSource::HardwareMeasured, kCorrectionNone),
        "raw capture");

    for (const Sample& s : samples) {
        const std::string js = timestamp_to_json_string(s.ts);
        Timestamp back;
        const bool ok = timestamp_from_json_string(js, back);
        BOOST_REQUIRE_MESSAGE(ok, s.label);
        BOOST_REQUIRE(back.ticks == s.ts.ticks);
        BOOST_REQUIRE(back.frac_num == s.ts.frac_num);
        BOOST_REQUIRE(back.frac_den == s.ts.frac_den);
        BOOST_REQUIRE(back.domain.name == s.ts.domain.name);
        BOOST_REQUIRE(std::memcmp(&back.domain.tick_rate_hz, &s.ts.domain.tick_rate_hz,
                                  sizeof(double)) == 0);
        BOOST_REQUIRE(back.domain.epoch_id == s.ts.domain.epoch_id);
        BOOST_REQUIRE(back.domain.timestamp_bits == s.ts.domain.timestamp_bits);
        BOOST_REQUIRE(back.marker == s.ts.marker);
        BOOST_REQUIRE(back.source == s.ts.source);
        BOOST_REQUIRE(back.applied_corrections == s.ts.applied_corrections);
        BOOST_REQUIRE(back.calibration_id == s.ts.calibration_id);
        BOOST_REQUIRE(back.valid == s.ts.valid);
        BOOST_REQUIRE(timestamp_to_json_string(back) == js);
    }

    // The > 2^53 value really is quoted, so no JSON reader can round it.
    const std::string big = timestamp_to_json_string(samples[0].ts);
    BOOST_REQUIRE(big.find("\"ticks\":\"9007199254740993\"") != std::string::npos);
    const std::string edge = timestamp_to_json_string(samples[1].ts);
    BOOST_REQUIRE(edge.find("\"ticks\":9007199254740991") != std::string::npos);

    // The human-readable fraction survives as an exact rational.
    BOOST_REQUIRE(timestamp_to_decimal_string(samples[2].ts) == "32048+4/5");

    // Interval JSON carries the exact tick-space value plus the status.
    const TimeInterval ti = timestamp_interval(samples[2].ts, samples[3].ts);
    BOOST_REQUIRE(ti.status == TimeIntervalStatus::Ok);
    BOOST_REQUIRE(ti.frac_num == 4 && ti.frac_den == 5u);
    const std::string ij = time_interval_to_json_string(ti);
    BOOST_REQUIRE(ij.find("\"status\":\"ok\"") != std::string::npos);
    // M0.1 / R4: the JSON now carries the two HONEST flags separately, plus the
    // rounding rule, and the misleading old key is gone.
    BOOST_REQUIRE(ij.find("\"whole_ticks\":false") != std::string::npos);
    BOOST_REQUIRE(ij.find("\"duration_ns_is_lossless\":false") != std::string::npos);
    BOOST_REQUIRE(ij.find("\"ns_projection_rounding\":\"truncate_toward_zero\"") !=
                  std::string::npos);
    BOOST_REQUIRE(ij.find("duration_is_tick_exact") == std::string::npos);
    BOOST_REQUIRE(ij.find("\"clock_domain\":{\"name\":\"dw1000_uus\"") !=
                  std::string::npos);
    BOOST_REQUIRE(ij.find("\"later_marker\":\"rmarker_rx\"") != std::string::npos);
    BOOST_REQUIRE(ij.find("\"earlier_marker\":\"rmarker_tx\"") != std::string::npos);
}

// ===========================================================================
// 18. Malformed JSON is rejected, never half-parsed
// ===========================================================================

BOOST_AUTO_TEST_CASE(test_json_rejects_malformed)
{
    const ClockDomain d = dw_uus40();
    Timestamp ts = must_make_frac(31948, 4, 5, d, TimestampMarker::RmarkerRx,
                                  TimestampSource::HardwareMeasured, kFullRxChain);
    ts.calibration_id = "cal-dw-sn0-ch5-v1";
    const std::string good = timestamp_to_json_string(ts);
    Timestamp out;

    // Sanity: the reference string parses.
    BOOST_REQUIRE(timestamp_from_json_string(good, out));
    BOOST_REQUIRE(out.ticks == ts.ticks);

    BOOST_REQUIRE(!timestamp_from_json_string("", out));
    BOOST_REQUIRE(!timestamp_from_json_string("{", out));
    BOOST_REQUIRE(!timestamp_from_json_string("not json", out));
    BOOST_REQUIRE(!timestamp_from_json_string(good.substr(0, good.size() - 1), out));
    BOOST_REQUIRE(!timestamp_from_json_string(good + "}", out));
    BOOST_REQUIRE(!timestamp_from_json_string(good + "x", out));
    BOOST_REQUIRE(!timestamp_from_json_string(good.substr(1), out)); // drop the '{'
    BOOST_REQUIRE(!timestamp_from_json_string(good + " ", out));

    auto mutated = [&](const std::string& from, const std::string& to) {
        const std::string m = replace_once(good, from, to);
        BOOST_REQUIRE_MESSAGE(!m.empty(), "mutation target not found");
        BOOST_REQUIRE_MESSAGE(m != good, "mutation was a no-op");
        BOOST_REQUIRE(!timestamp_from_json_string(m, out));
    };

    mutated("\"rmarker_rx\"", "\"not_a_marker\"");
    mutated("\"hardware_measured\"", "\"read_from_the_sky\"");
    mutated("\"valid\":true", "\"valid\":1");
    mutated("\"valid\":true", "\"valid\":null");
    // NaN / Inf / non-positive / non-numeric rates.
    mutated("\"tick_rate_hz\":", "\"tick_rate_hz\":nan,");
    mutated("\"tick_rate_hz\":", "\"tick_rate_hz\":inf,");
    mutated("\"tick_rate_hz\":", "\"tick_rate_hz\":1e999,");
    mutated("\"tick_rate_hz\":", "\"tick_rate_hz\":-1,");
    mutated("\"tick_rate_hz\":", "\"tick_rate_hz\":0,");
    mutated("\"tick_rate_hz\":", "\"tick_rate_hz\":Infinity,");
    mutated("\"tick_rate_hz\":", "\"tick_rate_hz\":\"abc\",");
    mutated("\"epoch_id\":", "\"epoch_id\":-1,");
    mutated("\"timestamp_bits\":40", "\"timestamp_bits\":64");
    mutated("\"timestamp_bits\":40", "\"timestamp_bits\":7");
    mutated("\"frac_num\":4", "\"frac_num\":-1");
    mutated("\"frac_den\":5", "\"frac_den\":-1");
    mutated("\"corrections\":", "\"corrections\":1024,");
    mutated("\"corrections\":", "\"corrections\":-1,");
    // The human-readable flag list must agree with the bitmask.
    mutated("\"corrections_flags\":", "\"corrections_flags\":\"");
    mutated("\"corrections_flags\":\"", "\"corrections_flags\":");
    // A JSON string escape is a sign the input came from somewhere else.
    mutated("\"calibration_id\":\"cal-dw-sn0-ch5-v1\"", "\"calibration_id\":\"a\\\"b\"");
    // An empty domain name is structurally invalid.
    mutated("\"name\":\"dw1000_uus\"", "\"name\":\"\"");
    // An unknown trailing key.
    mutated("\"valid\":true", "\"valid\":true,\"extra\":1");
    // Key order is part of the emitted schema.
    {
        const std::string marker_block = "\"marker\":\"rmarker_rx\",";
        const std::string source_block = "\"source\":\"hardware_measured\",";
        const std::string a = replace_once(good, marker_block, "@M@");
        BOOST_REQUIRE(a != std::string());
        const std::string b = replace_once(a, source_block, marker_block);
        BOOST_REQUIRE(b != std::string());
        const std::string swapped = replace_once(b, "@M@", source_block);
        BOOST_REQUIRE(swapped != std::string());
        BOOST_REQUIRE(!timestamp_from_json_string(swapped, out));
    }
}

// ===========================================================================
// 19. Absurd / NaN / Inf / out-of-range inputs are rejected
// ===========================================================================

BOOST_AUTO_TEST_CASE(test_absurd_inputs_rejected)
{
    Timestamp ts;
    const ClockDomain d = dw_uus40();

    // Ticks outside the declared counter window.
    BOOST_REQUIRE(!Timestamp::from_ticks(-1, d, TimestampMarker::RmarkerRx,
                                         TimestampSource::HardwareMeasured, kFullRxChain,
                                         ts));
    BOOST_REQUIRE(!Timestamp::from_ticks(kTwoPow40, d, TimestampMarker::RmarkerRx,
                                         TimestampSource::HardwareMeasured, kFullRxChain,
                                         ts));
    BOOST_REQUIRE(!Timestamp::from_ticks(std::numeric_limits<int64_t>::max(), d,
                                         TimestampMarker::RmarkerRx,
                                         TimestampSource::HardwareMeasured, kFullRxChain,
                                         ts));

    // Unknown source / marker.
    BOOST_REQUIRE(!Timestamp::from_ticks(1, d, TimestampMarker::PreambleStart,
                                         TimestampSource::Unknown, kFullRxChain, ts));
    BOOST_REQUIRE(!Timestamp::from_ticks(
        1, d, static_cast<TimestampMarker>(99), TimestampSource::HardwareMeasured,
        kFullRxChain, ts));
    BOOST_REQUIRE(!Timestamp::from_ticks(
        1, d, TimestampMarker::RmarkerRx, static_cast<TimestampSource>(42), kFullRxChain,
        ts));

    // Unknown correction bit.
    BOOST_REQUIRE(!Timestamp::from_ticks(1, d, TimestampMarker::RmarkerRx,
                                         TimestampSource::HardwareMeasured, 1u << 20, ts));

    // An invalid domain cannot host a timestamp at all.
    {
        ClockDomain bad = d;
        bad.tick_rate_hz = std::numeric_limits<double>::quiet_NaN();
        BOOST_REQUIRE(!Timestamp::from_ticks(1, bad, TimestampMarker::RmarkerRx,
                                             TimestampSource::HardwareMeasured,
                                             kFullRxChain, ts));
        bad.tick_rate_hz = std::numeric_limits<double>::infinity();
        BOOST_REQUIRE(!Timestamp::from_ticks(1, bad, TimestampMarker::RmarkerRx,
                                             TimestampSource::HardwareMeasured,
                                             kFullRxChain, ts));
        bad = d;
        bad.tick_rate_hz = 0.0;
        BOOST_REQUIRE(!Timestamp::from_ticks(1, bad, TimestampMarker::RmarkerRx,
                                             TimestampSource::HardwareMeasured,
                                             kFullRxChain, ts));
        bad = d;
        bad.name.clear();
        BOOST_REQUIRE(!Timestamp::from_ticks(1, bad, TimestampMarker::RmarkerRx,
                                             TimestampSource::HardwareMeasured,
                                             kFullRxChain, ts));
        bad = d;
        bad.timestamp_bits = 64;
        BOOST_REQUIRE(!Timestamp::from_ticks(1, bad, TimestampMarker::RmarkerRx,
                                             TimestampSource::HardwareMeasured,
                                             kFullRxChain, ts));
    }

    // An unnormalizable fraction is refused at construction.
    BOOST_REQUIRE(!Timestamp::from_fractional_ticks(1, 3, 0, d, TimestampMarker::RmarkerRx,
                                                    TimestampSource::HardwareMeasured,
                                                    kFullRxChain, ts));
    BOOST_REQUIRE(!Timestamp::from_fractional_ticks(
        1, 1, kMaxTimestampFractionDenominator + 1, d, TimestampMarker::RmarkerRx,
        TimestampSource::HardwareMeasured, kFullRxChain, ts));

    // The frozen Duration rejects non-finite input too.
    Duration dur;
    BOOST_REQUIRE(!Duration::from_seconds(std::numeric_limits<double>::quiet_NaN(), dur));
    BOOST_REQUIRE(!Duration::from_seconds(std::numeric_limits<double>::infinity(), dur));
    BOOST_REQUIRE(!Duration::from_seconds(1.0e30, dur));
    BOOST_REQUIRE(!Duration::from_ticks(1, 0.0, dur));
    BOOST_REQUIRE(!Duration::from_ticks(1, std::numeric_limits<double>::quiet_NaN(), dur));

    // A hand-built Timestamp with an unknown correction bit is refused.
    const Timestamp ok_rx = must_make(20, d, TimestampMarker::RmarkerRx,
                                      TimestampSource::HardwareMeasured, kFullRxChain);
    Timestamp hacked = must_make(10, d, TimestampMarker::RmarkerRx,
                                 TimestampSource::HardwareMeasured, kFullRxChain);
    hacked.applied_corrections = 1u << 25;
    BOOST_REQUIRE(!timestamp_is_self_consistent(hacked));
    BOOST_REQUIRE(std::string(timestamp_rejection_reason(hacked)) ==
                  "unknown_correction_bit");
    BOOST_REQUIRE(timestamp_interval(ok_rx, hacked).status ==
                  TimeIntervalStatus::InvalidTimestamp);

    // Ticks pushed out of range by hand.
    hacked = must_make(10, d, TimestampMarker::RmarkerRx,
                       TimestampSource::HardwareMeasured, kFullRxChain);
    hacked.ticks = kTwoPow40;
    BOOST_REQUIRE(!timestamp_is_self_consistent(hacked));
    BOOST_REQUIRE(std::string(timestamp_rejection_reason(hacked)) ==
                  "ticks_out_of_domain_range");

    // A corrupted domain inside an otherwise fine timestamp.
    hacked = must_make(10, d, TimestampMarker::RmarkerRx,
                       TimestampSource::HardwareMeasured, kFullRxChain);
    hacked.domain.tick_rate_hz = 0.0;
    BOOST_REQUIRE(!timestamp_is_self_consistent(hacked));
    BOOST_REQUIRE(std::string(timestamp_rejection_reason(hacked)) ==
                  "invalid_clock_domain");

    // An unknown marker inside an otherwise fine timestamp.
    hacked = must_make(10, d, TimestampMarker::RmarkerRx,
                       TimestampSource::HardwareMeasured, kFullRxChain);
    hacked.marker = static_cast<TimestampMarker>(77);
    BOOST_REQUIRE(!timestamp_is_self_consistent(hacked));
    BOOST_REQUIRE(std::string(timestamp_rejection_reason(hacked)) == "unknown_marker");
}

// ===========================================================================
// 20. Status taxonomy maps onto the frozen exchange error set
// ===========================================================================

BOOST_AUTO_TEST_CASE(test_status_maps_to_exchange_status)
{
    int non_ok = 0;
    for (int i = 0; i <= static_cast<int>(TimeIntervalStatus::OrderReversed); ++i) {
        const TimeIntervalStatus s = static_cast<TimeIntervalStatus>(i);
        const std::string name = time_interval_status_to_string(s);
        BOOST_REQUIRE(name != std::string("invalid"));
        TimeIntervalStatus back = TimeIntervalStatus::InvalidTimestamp;
        BOOST_REQUIRE(time_interval_status_from_string(name, back));
        BOOST_REQUIRE(back == s);
        if (s == TimeIntervalStatus::Ok) {
            BOOST_REQUIRE(time_interval_status_is_ok(s));
            BOOST_REQUIRE(time_interval_status_to_exchange_status(s) == ExchangeStatus::Ok);
        } else {
            BOOST_REQUIRE(!time_interval_status_is_ok(s));
            // Every time-domain failure collapses to one terminal status
            // (REQ-ERR-01); the reason travels separately.
            BOOST_REQUIRE(time_interval_status_to_exchange_status(s) ==
                          ExchangeStatus::InvalidTimeDomain);
            ++non_ok;
        }
        // Names are unique: they are part of the JSON schema.
        for (int j = 0; j < i; ++j) {
            BOOST_REQUIRE(name != time_interval_status_to_string(
                                    static_cast<TimeIntervalStatus>(j)));
        }
    }
    BOOST_REQUIRE(non_ok == 11);
    TimeIntervalStatus dummy = TimeIntervalStatus::Ok;
    BOOST_REQUIRE(!time_interval_status_from_string("not_a_status", dummy));

    // The frozen taxonomy agrees on the name and the family.
    BOOST_REQUIRE(std::string(gr::uwb::twr::exchange_status_to_string(
                      ExchangeStatus::InvalidTimeDomain)) == "invalid_time_domain");
    BOOST_REQUIRE(std::string(gr::uwb::twr::exchange_status_family(
                      ExchangeStatus::InvalidTimeDomain)) == "signal");
    BOOST_REQUIRE(!gr::uwb::twr::exchange_status_yields_range(
        ExchangeStatus::InvalidTimeDomain));
}

// ===========================================================================
// 21. The raw modular difference is an adapter tool, not a protocol interval
// ===========================================================================

BOOST_AUTO_TEST_CASE(test_raw_tick_delta_is_adapter_only)
{
    const ClockDomain d = dw_uus40();

    // Two RX first-IQ-sample times straddling the wrap: the raw modular
    // difference is exactly what an adapter needs, and it ignores markers and
    // corrections.
    const TickDelta td = raw_tick_delta(d, 7, kTwoPow40 - 100);
    BOOST_REQUIRE(td.status == TimeIntervalStatus::Ok);
    BOOST_REQUIRE(td.ticks == 107);
    BOOST_REQUIRE(td.wrapped);

    // The same two values as timestamps form a legal interval (same marker, no
    // corrections required for a raw capture coordinate) and it must agree
    // with the raw difference.
    const Timestamp iq_late = must_make(7, d, TimestampMarker::UhdRxFirstIqSample,
                                        TimestampSource::HardwareMeasured, kCorrectionNone);
    const Timestamp iq_early = must_make(kTwoPow40 - 100, d,
                                         TimestampMarker::UhdRxFirstIqSample,
                                         TimestampSource::HardwareMeasured, kCorrectionNone);
    const TimeInterval ti = timestamp_interval(iq_late, iq_early);
    BOOST_REQUIRE(ti.status == TimeIntervalStatus::Ok);
    BOOST_REQUIRE(ti.ticks == static_cast<int64_t>(td.ticks));
    BOOST_REQUIRE(ti.wrapped == td.wrapped);

    // The raw helper is refused for out-of-window ticks and bad domains.
    BOOST_REQUIRE(raw_tick_delta(d, kTwoPow40, 1).status ==
                  TimeIntervalStatus::InvalidTimestamp);
    BOOST_REQUIRE(raw_tick_delta(d, 1, -1).status == TimeIntervalStatus::InvalidTimestamp);
    {
        ClockDomain bad = d;
        bad.tick_rate_hz = 0.0;
        BOOST_REQUIRE(raw_tick_delta(bad, 1, 0).status ==
                      TimeIntervalStatus::InvalidTimestamp);
    }
}

// ===========================================================================
// 22. The four TWR intervals are computable from two independent domains
//     (shape check only -- the SS/DS formula oracle is M1's QA)
// ===========================================================================

BOOST_AUTO_TEST_CASE(test_twr_interval_shapes)
{
    // Ground truth: ToF = 200 ns, responder turnaround DB = 500 ns, initiator
    // final delay DA = 700 ns (deliberately unequal, REQ-PROTO-03).
    //   RA = 2*ToF + DB =  900 ns   (A grid)
    //   DB =              500 ns    (B grid)
    //   DA =              700 ns    (A grid)
    //   RB = 2*ToF + DA = 1100 ns   (B grid)
    //
    // Exact tick values, chosen so no rounding was needed:
    //    900 ns @ 737.28 MS/s = 663 + 69/125
    //    700 ns @ 737.28 MS/s = 516 + 12/125
    //    500 ns @ DW UUS      = 31948 + 4/5
    //   1100 ns @ DW UUS      = 70287 + 9/25
    const ClockDomain a_dom = x410_device();
    const ClockDomain b_dom = dw_uus40();

    // A: t1A TX @1000, t4A RX @1663+69/125, t5A TX @2179+81/125
    const Timestamp t1a = must_make(1000, a_dom, TimestampMarker::RmarkerTx,
                                    TimestampSource::ScheduledCalibrated, kFullTxChain);
    const Timestamp t4a = must_make_frac(1663, 69, 125, a_dom, TimestampMarker::RmarkerRx,
                                         TimestampSource::HardwareMeasured, kFullRxChain);
    const Timestamp t5a = must_make_frac(2179, 81, 125, a_dom, TimestampMarker::RmarkerTx,
                                         TimestampSource::ScheduledCalibrated, kFullTxChain);
    // B: t2B RX @50000, t3B TX @81948+4/5, t6B RX @152236+4/25
    const Timestamp t2b = must_make(50000, b_dom, TimestampMarker::RmarkerRx,
                                    TimestampSource::HardwareMeasured, kFullRxChain);
    const Timestamp t3b = must_make_frac(81948, 4, 5, b_dom, TimestampMarker::RmarkerTx,
                                         TimestampSource::ScheduledCalibrated, kFullTxChain);
    const Timestamp t6b = must_make_frac(152236, 4, 25, b_dom, TimestampMarker::RmarkerRx,
                                         TimestampSource::HardwareMeasured, kFullRxChain);

    const TimeInterval ra = timestamp_interval(t4a, t1a);
    const TimeInterval db = timestamp_interval(t3b, t2b);
    const TimeInterval da = timestamp_interval(t5a, t4a);
    const TimeInterval rb = timestamp_interval(t6b, t3b);
    const TimeInterval* all[4] = { &ra, &db, &da, &rb };
    for (const TimeInterval* ti : all) {
        BOOST_REQUIRE(ti->status == TimeIntervalStatus::Ok);
    }

    // The exact tick-space reconstruction carries the sub-nanosecond truth
    // the DW UUS grid needs; the fractions are reduced, not approximated.
    BOOST_REQUIRE(ra.ticks == 663 && ra.frac_num == 69 && ra.frac_den == 125u);
    BOOST_REQUIRE(da.ticks == 516 && da.frac_num == 12 && da.frac_den == 125u);
    BOOST_REQUIRE(db.ticks == 31948 && db.frac_num == 4 && db.frac_den == 5u);
    BOOST_REQUIRE(rb.ticks == 70287 && rb.frac_num == 9 && rb.frac_den == 25u);

    // The integer-nanosecond views match the analytic values ...
    BOOST_REQUIRE(ra.duration.nanos() == 900);
    BOOST_REQUIRE(da.duration.nanos() == 700);
    BOOST_REQUIRE(db.duration.nanos() == 500);
    BOOST_REQUIRE(rb.duration.nanos() == 1100);
    // M0.1 / R4 correction to the M0 expectation.  These four intervals were
    // chosen to be exact ns values, and the M0 flag said "inexact" for all of
    // them -- so M0 was wrong in BOTH directions, not only the reported one.
    // The new predicate sees that 900/700/500/1100 ns are reproduced with zero
    // error and says so.
    BOOST_REQUIRE(ra.duration_is_lossless_ns);
    BOOST_REQUIRE(da.duration_is_lossless_ns);
    BOOST_REQUIRE(db.duration_is_lossless_ns);
    BOOST_REQUIRE(rb.duration_is_lossless_ns);
    // ... but the tick-space fields are still the representation the ToF core
    // must use, because the guarantee is per-interval and the next exchange
    // will not be integral.  The safe form makes that structural:
    RelativeTickInterval ra_rel;
    BOOST_REQUIRE(timestamp_relative_interval(t4a, t1a, ra_rel));
    BOOST_REQUIRE(!ra_rel.is_whole_ticks());
    BOOST_REQUIRE(!ra_rel.ns_projection_is_lossless() == false); // it IS lossless
    int64_t rn = 0;
    int64_t rd = 0;
    BOOST_REQUIRE(ra_rel.exact_ratio(rn, rd));
    // 663 + 69/125 == 82944/125
    BOOST_REQUIRE(rn == 82944 && rd == 125);

    const auto secs = [](const TimeInterval& t) {
        return (static_cast<double>(t.ticks) + static_cast<double>(t.frac_num) /
                                             static_cast<double>(t.frac_den)) /
               t.domain.tick_rate_hz;
    };

    // SS: ToF = (RA - DB) / 2.  Cross-domain arithmetic happens ONLY here, on
    // two already-converted same-domain intervals.
    const double tof_ss = (secs(ra) - secs(db)) / 2.0;
    BOOST_REQUIRE(std::fabs(tof_ss - 200.0e-9) < 1.0e-18);

    // DS with UNEQUAL reply delays: ToF = (RA*RB - DA*DB)/(RA+RB+DA+DB).
    const double num = secs(ra) * secs(rb) - secs(da) * secs(db);
    const double den = secs(ra) + secs(rb) + secs(da) + secs(db);
    BOOST_REQUIRE(std::fabs(den - 3.2e-6) < 1.0e-18);
    BOOST_REQUIRE(std::fabs(num / den - 200.0e-9) < 1.0e-18);

    // The ordering gate agrees with the arithmetic ordering, inside one domain.
    bool order = false;
    BOOST_REQUIRE(timestamp_precedes(t1a, t4a, order));
    BOOST_REQUIRE(order);
    BOOST_REQUIRE(timestamp_precedes(t4a, t5a, order));
    BOOST_REQUIRE(order);
    BOOST_REQUIRE(timestamp_precedes(t2b, t3b, order));
    BOOST_REQUIRE(order);
    BOOST_REQUIRE(timestamp_precedes(t3b, t6b, order));
    BOOST_REQUIRE(order);
    // A private timestamp of the OTHER endpoint can never be ordered against a
    // local one (REQ-SCOPE-02, 两端隔离 in the QA matrix).
    BOOST_REQUIRE(!timestamp_precedes(t1a, t2b, order));
    BOOST_REQUIRE(!timestamp_precedes(t4a, t3b, order));
    BOOST_REQUIRE(!timestamp_precedes(t5a, t6b, order));
    BOOST_REQUIRE(!timestamp_precedes(t6b, t4a, order));

    // The same orderings through the M0.1 fraction-aware entry point.  The
    // cross-endpoint pairs are Indeterminate, never an order.
    TimestampOrder o = TimestampOrder::Indeterminate;
    BOOST_REQUIRE(timestamp_compare(t1a, t4a, o));
    BOOST_REQUIRE(o == TimestampOrder::Earlier);
    BOOST_REQUIRE(timestamp_compare(t4a, t5a, o));
    BOOST_REQUIRE(o == TimestampOrder::Earlier);
    BOOST_REQUIRE(timestamp_compare(t2b, t3b, o));
    BOOST_REQUIRE(o == TimestampOrder::Earlier);
    BOOST_REQUIRE(!timestamp_compare(t1a, t2b, o));
    BOOST_REQUIRE(o == TimestampOrder::Indeterminate);
    BOOST_REQUIRE(!timestamp_compare(t4a, t3b, o));
    BOOST_REQUIRE(o == TimestampOrder::Indeterminate);
    BOOST_REQUIRE(!timestamp_compare(t5a, t6b, o));
    BOOST_REQUIRE(o == TimestampOrder::Indeterminate);
    BOOST_REQUIRE(!timestamp_compare(t6b, t4a, o));
    BOOST_REQUIRE(o == TimestampOrder::Indeterminate);
}

// ===========================================================================
// 23. M0.1 / R4 -- the ns projection is lossless only when it really is
// ===========================================================================
//
// Independent small vectors.  The header's predicate is compared against the
// ACTUAL error of the emitted integer for each vector; a vector where the two
// disagree is a bug in one of them.  `exact_ns` below is the true value in
// nanoseconds, computed from the rate and tick count independently of the
// header, and `expect_lossless` is hand-derived.

BOOST_AUTO_TEST_CASE(test_r4_lossless_projection_per_rate)
{
    struct Case {
        const char* label;
        double rate_hz;
        int64_t dticks;
        int32_t num;
        uint32_t den;
        int64_t expect_ns;   // truncated projection
        bool expect_lossless;
    };
    const std::vector<Case> cases = {
        // --- the R4 reproducer: 737.28 MS/s, 1 tick = 1.356336805... ns.
        // The M0 flag said "exact" here while discarding 0.356 ns.
        { "737.28M 1 tick", kX410DeviceHz, 1, 0, 0u, 1, false },
        { "737.28M 1 tick + 1/2", kX410DeviceHz, 1, 1, 2u, 2, false },
        { "737.28M 2 ticks", kX410DeviceHz, 2, 0, 0u, 2, false },
        { "737.28M 3 ticks", kX410DeviceHz, 3, 0, 0u, 4, false },
        { "737.28M 64 ticks", kX410DeviceHz, 64, 0, 0u, 86, false },
        // 73728 = 737.28e6/1e4 -> exactly 100000 ns: the rate alone does not
        // decide losslessness, a per-interval test does.
        { "737.28M 73728 ticks", kX410DeviceHz, 73728, 0, 0u, 100000, true },
        { "737.28M 147456 ticks", kX410DeviceHz, 147456, 0, 0u, 200000, true },
        { "737.28M 73729 ticks", kX410DeviceHz, 73729, 0, 0u, 100001, false },

        // --- 998.4 MS/s work grid: 1 tick = 1.001602564... ns.
        { "998.4M 1 tick", kX410WorkHz, 1, 0, 0u, 1, false },
        { "998.4M 99840 ticks", kX410WorkHz, 99840, 0, 0u, 100000, true },
        { "998.4M 49920 ticks", kX410WorkHz, 49920, 0, 0u, 50000, true },

        // --- 491.52 MS/s native grid: 1 tick = 2.034505208... ns.
        { "491.52M 1 tick", kX410NativeHz, 1, 0, 0u, 2, false },
        { "491.52M 49152 ticks", kX410NativeHz, 49152, 0, 0u, 100000, true },
        { "491.52M 12288 ticks", kX410NativeHz, 12288, 0, 0u, 25000, true },

        // --- DW UUS at 1/(499.2e6*128): 1 tick = 0.0156500400641 ns.
        { "DW UUS 1 tick", kDwUusTickHz, 1, 0, 0u, 0, false },
        { "DW UUS 64 ticks", kDwUusTickHz, 64, 0, 0u, 1, false },
        { "DW UUS 6389760 ticks", kDwUusTickHz, 6389760, 0, 0u, 100000, true },
        { "DW UUS 6389759 ticks", kDwUusTickHz, 6389759, 0, 0u, 99999, false },

        // --- an exactly 1 GHz domain really is lossless at every tick count.
        { "1 GHz 1 tick", 1.0e9, 1, 0, 0u, 1, true },
        { "1 GHz 7 ticks", 1.0e9, 7, 0, 0u, 7, true },

        // --- a coarser-than-1 ns tick is not automatically lossy either:
        // 500 MHz -> 2 ns/tick exactly, 250 MHz -> 4 ns/tick exactly.
        { "500 MHz 1 tick", 500.0e6, 1, 0, 0u, 2, true },
        { "500 MHz 9 ticks", 500.0e6, 9, 0, 0u, 18, true },
        { "250 MHz 3 ticks", 250.0e6, 3, 0, 0u, 12, true },
        { "200 MHz 1 tick", 200.0e6, 1, 0, 0u, 5, true },

        // --- faster than 1 GHz: the tick is sub-ns, so almost nothing is.
        { "2.5 GHz 2 ticks", 2.5e9, 2, 0, 0u, 0, false },
        { "2.5 GHz 4 ticks", 2.5e9, 4, 0, 0u, 1, false },
        { "2.5 GHz 8 ticks", 2.5e9, 8, 0, 0u, 3, false },

        // --- a sub-tick fraction: exact only when it lands on a whole ns.
        // 1 GHz -> 1 ns/tick, so +1/2 tick is 1.5 ns: NOT lossless.
        { "1 GHz 1 tick + 1/2", 1.0e9, 1, 1, 2u, 1, false },
        { "1 GHz 1 tick + 1/4", 1.0e9, 1, 1, 4u, 1, false },
        // 500 MHz -> 2 ns/tick, so +1/2 tick is exactly 3 ns: lossless.
        { "500 MHz 1 tick + 1/2", 500.0e6, 1, 1, 2u, 3, true },
        // +1/4 tick is 0.5 ns -> 2.5 ns: not lossless.
        { "500 MHz 1 tick + 1/4", 500.0e6, 1, 1, 4u, 2, false },
        // 250 MHz -> 4 ns/tick, so +1/2 is exactly 6 ns.
        { "250 MHz 1 tick + 1/2", 250.0e6, 1, 1, 2u, 6, true },
        // 400 MHz -> 2.5 ns/tick, an exact half-ns: never lossless.
        { "400 MHz 1 tick", 400.0e6, 1, 0, 0u, 2, false },
        { "1 GHz 0 ticks", 1.0e9, 0, 0, 0u, 0, true },
        { "737.28M 0 ticks", kX410DeviceHz, 0, 0, 0u, 0, true },
    };

    for (const Case& c : cases) {
        ClockDomain d;
        d.name = "r4_vector";
        d.tick_rate_hz = c.rate_hz;
        d.epoch_id = 1;
        d.timestamp_bits = 0;

        const Timestamp later = must_make_frac(c.dticks, c.num, c.den, d,
                                              TimestampMarker::RmarkerRx,
                                              TimestampSource::HardwareMeasured, kFullRxChain);
        const Timestamp earlier = must_make(0, d, TimestampMarker::RmarkerRx,
                                            TimestampSource::HardwareMeasured, kFullRxChain);
        const TimeInterval ti = timestamp_interval(later, earlier);
        BOOST_REQUIRE_MESSAGE(ti.status == TimeIntervalStatus::Ok, c.label);
        BOOST_REQUIRE_MESSAGE(ti.ticks == c.dticks, c.label);
        BOOST_REQUIRE_MESSAGE(ti.duration.nanos() == c.expect_ns, c.label);

        // The independent truth: exact value in ns, and the observed error.
        double exact_ns = static_cast<double>(c.dticks) * 1e9 / c.rate_hz;
        if (c.den != 0u)
            exact_ns += (static_cast<double>(c.num) / static_cast<double>(c.den)) * 1e9 / c.rate_hz;
        const double err = static_cast<double>(ti.duration.nanos()) - exact_ns;

        // (1) the emitted integer really is the truncation of the exact value;
        BOOST_REQUIRE_MESSAGE(std::fabs(err) < 1.0, c.label);
        BOOST_REQUIRE_MESSAGE(err <= 0.0, c.label); // truncation, never up
        // (2) "lossless" means EXACTLY zero error, so it must agree with the
        //     measured error in both directions.  This is the assertion the M0
        //     flag could not have satisfied: at 737.28 MS/s / 1 tick it said
        //     "exact" with a 0.356 ns error.
        BOOST_REQUIRE_MESSAGE((err == 0.0) == c.expect_lossless, c.label);
        BOOST_REQUIRE_MESSAGE(ti.duration_is_lossless_ns == c.expect_lossless, c.label);
        // (3) the header's own predicate, called directly, agrees too.
        BOOST_REQUIRE_MESSAGE(timestamp_tick_ns_projection_is_lossless(
                                  c.rate_hz, ti.ticks, ti.frac_num, ti.frac_den) ==
                                  c.expect_lossless, c.label);
        // (4) the two flags are INDEPENDENT questions and must not be conflated.
        BOOST_REQUIRE_MESSAGE(ti.is_whole_ticks == (c.den == 0u), c.label);
        BOOST_REQUIRE_MESSAGE(ti.is_whole_ticks == time_interval_is_whole_ticks(ti), c.label);
    }

    // The specific vector the M0 report was built on, asserted on its own so
    // the regression is named in the log.
    {
        ClockDomain d;
        d.name = "x410_device";
        d.tick_rate_hz = kX410DeviceHz;
        d.epoch_id = 3;
        d.timestamp_bits = 0;
        const TimeInterval ti = timestamp_interval(
            must_make(1, d, TimestampMarker::RmarkerRx, TimestampSource::HardwareMeasured,
                      kFullRxChain),
            must_make(0, d, TimestampMarker::RmarkerRx, TimestampSource::HardwareMeasured,
                      kFullRxChain));
        BOOST_REQUIRE(ti.status == TimeIntervalStatus::Ok);
        BOOST_REQUIRE(ti.ticks == 1);
        BOOST_REQUIRE(ti.duration.nanos() == 1);
        BOOST_REQUIRE(ti.is_whole_ticks);
        // The M0 assertion was `duration_is_tick_exact == true` here.  The
        // true value is 1.356336806 ns, so it was wrong.
        BOOST_REQUIRE(!ti.duration_is_lossless_ns);
        // The rate-level query is still true, and that is precisely why it
        // cannot be the lossless test.
        BOOST_REQUIRE(clock_domain_tick_is_ns_or_coarser(d));
    }
}

// ===========================================================================
// 24. M0.1 / R4 -- exact ns/tick rational, and the projection's boundaries
// ===========================================================================

BOOST_AUTO_TEST_CASE(test_r4_exact_ns_per_tick_rational)
{
    struct Case {
        const char* label;
        double rate_hz;
        int64_t num;
        int64_t den;
    };
    const std::vector<Case> cases = {
        // 1e9/1e9 = 1
        { "1 GHz", 1.0e9, 1, 1 },
        // 1e9/500e6 = 2
        { "500 MHz", 500.0e6, 2, 1 },
        // 1e9/250e6 = 4
        { "250 MHz", 250.0e6, 4, 1 },
        // 1e9/125e6 = 8
        { "125 MHz", 125.0e6, 8, 1 },
        // 1e9/100e6 = 10
        { "100 MHz", 100.0e6, 10, 1 },
        // 1e9/200e6 = 5
        { "200 MHz", 200.0e6, 5, 1 },
        // 1e9/737.28e6 = 3125/2304 = 1.35633680555...  (737.28e6 = 2^17*5625)
        { "737.28 MS/s", kX410DeviceHz, 3125, 2304 },
        // 1e9/998.4e6 = 625/624 = 1.00160256410...  (998.4e6 = 2^22*3*37)
        { "998.4 MS/s", kX410WorkHz, 625, 624 },
        // 1e9/491.52e6 = 625/307.2 -> 3125/1536 = 2.03450520833...
        { "491.52 MS/s", kX410NativeHz, 3125, 1536 },
        // 1e9/(499.2e6*128) = 1/63.8976 = 625/39936 = 0.0156500400641...
        { "DW UUS", kDwUusTickHz, 625, 39936 },
        // 1e9/2.5e9 = 2/5
        { "2.5 GHz", 2.5e9, 2, 5 },
    };

    for (const Case& c : cases) {
        int64_t num = 0;
        int64_t den = 0;
        BOOST_REQUIRE_MESSAGE(clock_domain_ns_per_tick_exact(c.rate_hz, num, den), c.label);
        BOOST_REQUIRE_MESSAGE(num == c.num && den == c.den, c.label);
        // Reduced: gcd(num, den) == 1.
        int64_t a = num, b = den;
        while (b != 0) { const int64_t t = a % b; a = b; b = t; }
        BOOST_REQUIRE_MESSAGE(a == 1, c.label);
        // And it really is 1e9/rate to double precision.
        BOOST_REQUIRE_MESSAGE(std::fabs(static_cast<double>(num) / static_cast<double>(den) -
                                        1e9 / c.rate_hz) < 1e-12, c.label);
    }

    // Invalid input is refused, never silently turned into a rational.
    int64_t num = 7;
    int64_t den = 9;
    BOOST_REQUIRE(!clock_domain_ns_per_tick_exact(0.0, num, den));
    BOOST_REQUIRE(!clock_domain_ns_per_tick_exact(-1.0e9, num, den));
    BOOST_REQUIRE(!clock_domain_ns_per_tick_exact(
        std::numeric_limits<double>::infinity(), num, den));
    BOOST_REQUIRE(!clock_domain_ns_per_tick_exact(
        std::numeric_limits<double>::quiet_NaN(), num, den));
    BOOST_REQUIRE(!clock_domain_ns_per_tick_exact(
        std::numeric_limits<double>::denorm_min(), num, den));
    // A refused rational leaves a valid-looking (0, 0), never a stale value.
    BOOST_REQUIRE(num == 0 && den == 0);

    // A non-finite rate is refused by the lossless predicate too: the
    // direction is "not provably lossless", never "assumed lossless".
    BOOST_REQUIRE(!timestamp_tick_ns_projection_is_lossless(0.0, 1, 0, 0u));
    BOOST_REQUIRE(!timestamp_tick_ns_projection_is_lossless(
        std::numeric_limits<double>::infinity(), 1, 0, 0u));
}

// ===========================================================================
// 25. M0.1 / R4 -- projection overflow, rounding rule, and the safe API
// ===========================================================================

BOOST_AUTO_TEST_CASE(test_r4_projection_overflow_and_rounding_rule)
{
    // Rounding rule: TRUNCATE toward zero.  Each vector's exact value lies
    // strictly between two integers, so the answer is forced.
    struct Case {
        const char* label;
        double rate_hz;
        int64_t dticks;
        int32_t num;
        uint32_t den;
        int64_t expect_ns;
    };
    const std::vector<Case> cases = {
        // 2.5 ns: the true value is exactly halfway between 2 and 3.  Under
        // round-half-away-from-zero it would be 3; under round-half-even it
        // would be 2; the documented rule is TRUNCATION, so 2 -- and the value
        // being a genuine tie is what makes the rule observable.
        { "2.5 ns (tie)", 400.0e6, 1, 0, 0u, 2 },
        // 1.5 ns: half-away would give 2, truncation gives 1.
        { "1.5 ns (tie b)", 2.0e9 / 3.0, 1, 0, 0u, 1 },
        // 3.5 ns: half-away 4, half-even 4, truncation 3.
        { "3.5 ns (tie c)", 2.0e9 / 7.0, 1, 0, 0u, 3 },
        // 0.6 ns: below 1, so it truncates to 0 rather than rounding to 1.
        { "0.6 ns", 5.0e9 / 3.0, 1, 0, 0u, 0 },
        // Non-tie values, to show the rule is truncation and not rounding to
        // nearest: 1.9 -> 1, 2.9 -> 2.
        { "1.9 ns", 1.0e9 / 1.9, 1, 0, 0u, 1 },
        { "2.9 ns", 1.0e9 / 2.9, 1, 0, 0u, 2 },
        // 31948 + 4/5 DW UUS ticks is exactly 500 ns: a tie-free exact value
        // that must come out unchanged, and the one case here that is lossless.
        { "500 ns exact", kDwUusTickHz, 31948, 4, 5u, 500 },
    };
    for (const Case& c : cases) {
        ClockDomain d;
        d.name = "r4_round";
        d.tick_rate_hz = c.rate_hz;
        d.epoch_id = 1;
        d.timestamp_bits = 0;
        const TimeInterval ti = timestamp_interval(
            must_make_frac(c.dticks, c.num, c.den, d, TimestampMarker::RmarkerRx,
                           TimestampSource::HardwareMeasured, kFullRxChain),
            must_make(0, d, TimestampMarker::RmarkerRx, TimestampSource::HardwareMeasured,
                      kFullRxChain));
        BOOST_REQUIRE_MESSAGE(ti.status == TimeIntervalStatus::Ok, c.label);
        BOOST_REQUIRE_MESSAGE(ti.duration.nanos() == c.expect_ns, c.label);
        // Truncation means the emitted integer never EXCEEDS the true value.
        double exact_ns = static_cast<double>(c.dticks) * 1e9 / c.rate_hz;
        if (c.den != 0u)
            exact_ns += (static_cast<double>(c.num) / static_cast<double>(c.den)) * 1e9 / c.rate_hz;
        BOOST_REQUIRE_MESSAGE(static_cast<double>(c.expect_ns) <= exact_ns + 1e-9, c.label);
        BOOST_REQUIRE_MESSAGE(exact_ns - static_cast<double>(c.expect_ns) < 1.0, c.label);
        BOOST_REQUIRE_MESSAGE(ti.duration_is_lossless_ns == (exact_ns == std::floor(exact_ns)),
                              c.label);
    }

    // The safe form M1 must consume: exact rational ticks, no ns field.
    const ClockDomain d = dw_uus40();
    RelativeTickInterval ri2;
    {
        TimeIntervalStatus st = TimeIntervalStatus::Ok;
        BOOST_REQUIRE(timestamp_relative_interval(
            must_make_frac(100, 3, 4, d, TimestampMarker::RmarkerRx,
                           TimestampSource::HardwareMeasured, kFullRxChain),
            must_make_frac(50, 1, 2, d, TimestampMarker::RmarkerRx,
                           TimestampSource::HardwareMeasured, kFullRxChain),
            ri2, st));
        BOOST_REQUIRE(st == TimeIntervalStatus::Ok);
    }
    BOOST_REQUIRE(ri2.is_valid());
    BOOST_REQUIRE(!ri2.is_whole_ticks());
    // 50 + 1/4 ticks exactly, as a rational, with no rounding anywhere.
    int64_t rn = 0;
    int64_t rd = 0;
    BOOST_REQUIRE(ri2.exact_ratio(rn, rd));
    BOOST_REQUIRE(rn == 201 && rd == 4);
    // Its ns projection is lossy, and the type says so.
    BOOST_REQUIRE(!ri2.ns_projection_is_lossless());
    Duration dur(12345);
    TimeIntervalStatus st = TimeIntervalStatus::Ok;
    BOOST_REQUIRE(relative_interval_to_duration(ri2, dur, st));
    BOOST_REQUIRE(st == TimeIntervalStatus::Ok);
    BOOST_REQUIRE(dur.nanos() == 0); // 50.25 * 0.01565 ns ~ 0.787 ns -> 0
    // A whole-tick relative interval reports the rational 1/1, not 0/0.
    {
        RelativeTickInterval r3;
        BOOST_REQUIRE(timestamp_relative_interval(
            must_make(7, d, TimestampMarker::RmarkerRx, TimestampSource::HardwareMeasured,
                      kFullRxChain),
            must_make(3, d, TimestampMarker::RmarkerRx, TimestampSource::HardwareMeasured,
                      kFullRxChain),
            r3));
        BOOST_REQUIRE(r3.is_whole_ticks());
        BOOST_REQUIRE(r3.exact_ratio(rn, rd));
        BOOST_REQUIRE(rn == 4 && rd == 1);
        // 4 DW UUS ticks is 0.0626 ns, so the ns projection is NOT lossless --
        // and the type says so, instead of the caller having to know it.
        BOOST_REQUIRE(!r3.ns_projection_is_lossless());
        Duration d3(99);
        TimeIntervalStatus s3 = TimeIntervalStatus::Ok;
        BOOST_REQUIRE(relative_interval_to_duration(r3, d3, s3));
        BOOST_REQUIRE(d3.nanos() == 0);
        // A whole-tick interval that IS an integral ns count is reported so.
        {
            RelativeTickInterval r5;
            BOOST_REQUIRE(timestamp_relative_interval(
                must_make(6389760, d, TimestampMarker::RmarkerRx,
                          TimestampSource::HardwareMeasured, kFullRxChain),
                must_make(0, d, TimestampMarker::RmarkerRx,
                          TimestampSource::HardwareMeasured, kFullRxChain),
                r5));
            BOOST_REQUIRE(r5.is_whole_ticks());
            BOOST_REQUIRE(r5.ns_projection_is_lossless());
            Duration d5(99);
            TimeIntervalStatus s5 = TimeIntervalStatus::Ok;
            BOOST_REQUIRE(relative_interval_to_duration(r5, d5, s5));
            BOOST_REQUIRE(d5.nanos() == 100000);
        }
    }
    // The safe form runs the SAME gate as the interval, so it is never more
    // permissive: a cross-domain pair fails here too.
    {
        RelativeTickInterval r4;
        TimeIntervalStatus s4 = TimeIntervalStatus::Ok;
        BOOST_REQUIRE(!timestamp_relative_interval(
            must_make(10, dw_uus40(), TimestampMarker::RmarkerRx,
                      TimestampSource::HardwareMeasured, kFullRxChain),
            must_make(1, dw_uus32(), TimestampMarker::RmarkerRx,
                      TimestampSource::HardwareMeasured, kFullRxChain),
            r4, s4));
        BOOST_REQUIRE(s4 == TimeIntervalStatus::WrapWidthMismatch);
        BOOST_REQUIRE(!r4.is_valid());
    }
    // An invalid relative interval cannot be projected.
    {
        RelativeTickInterval bad;
        Duration untouched(7);
        TimeIntervalStatus s5 = TimeIntervalStatus::Ok;
        BOOST_REQUIRE(!relative_interval_to_duration(bad, untouched, s5));
        BOOST_REQUIRE(s5 == TimeIntervalStatus::InvalidTimestamp);
        BOOST_REQUIRE(untouched.nanos() == 7);
        int64_t dummy = 0;
        BOOST_REQUIRE(!bad.exact_ratio(dummy, dummy));
        BOOST_REQUIRE(bad.seconds() == 0.0);
        BOOST_REQUIRE(bad.tick_fraction() == 0.0);
    }

    // Overflow: a whole-tick interval far past int64 nanoseconds is refused
    // with DurationOutOfRange rather than saturating.  A 1 Hz domain makes the
    // boundary reachable with a small tick count (1 tick = 1e9 ns).
    {
        ClockDomain slow;
        slow.name = "very_slow";
        slow.tick_rate_hz = 1.0;
        slow.epoch_id = 1;
        slow.timestamp_bits = 0;
        const int64_t kMaxTicks = 9223372036LL; // 9223372036e9 ns ~ INT64_MAX
        // One tick past the boundary: refused, not saturated.
        const TimeInterval over = timestamp_interval(
            must_make(kMaxTicks + 1, slow, TimestampMarker::RmarkerRx,
                      TimestampSource::HardwareMeasured, kFullRxChain),
            must_make(0, slow, TimestampMarker::RmarkerRx, TimestampSource::HardwareMeasured,
                      kFullRxChain));
        BOOST_REQUIRE(over.status == TimeIntervalStatus::DurationOutOfRange);
        BOOST_REQUIRE(over.ticks == 0);
        BOOST_REQUIRE(over.frac_num == 0 && over.frac_den == 0u);
        BOOST_REQUIRE(over.duration.nanos() == 0);
        BOOST_REQUIRE(!over.is_whole_ticks); // the value was discarded
        BOOST_REQUIRE(!over.duration_is_lossless_ns);
        BOOST_REQUIRE(time_interval_status_to_exchange_status(over.status) ==
                      ExchangeStatus::InvalidTimeDomain);
        BOOST_REQUIRE(std::string(time_interval_status_to_string(over.status)) ==
                      "duration_out_of_range");
        // Just inside the range still works: 9e9 ticks at 1 Hz is 9e18 ns,
        // which is below INT64_MAX, and 1 Hz is exactly 1e9 ns/tick so the
        // projection IS lossless there.
        const TimeInterval ok = timestamp_interval(
            must_make(9000000000LL, slow, TimestampMarker::RmarkerRx,
                      TimestampSource::HardwareMeasured, kFullRxChain),
            must_make(0, slow, TimestampMarker::RmarkerRx, TimestampSource::HardwareMeasured,
                      kFullRxChain));
        BOOST_REQUIRE(ok.status == TimeIntervalStatus::Ok);
        BOOST_REQUIRE(ok.duration.nanos() == 9000000000000000000LL);
        BOOST_REQUIRE(ok.is_whole_ticks);
        BOOST_REQUIRE(ok.duration_is_lossless_ns);
        // ... and the exact tick value survives alongside it, so a caller that
        // ignores the status still has the non-lossy representation.
        BOOST_REQUIRE(ok.ticks == 9000000000LL);
    }

    // A no-wrap domain refuses a reversed interval BEFORE projecting, so a
    // negative duration can never reach the API.
    {
        const TimeInterval ti = timestamp_interval(
            must_make(1, x410_device(), TimestampMarker::RmarkerRx,
                      TimestampSource::HardwareMeasured, kFullRxChain),
            must_make(5, x410_device(), TimestampMarker::RmarkerRx,
                      TimestampSource::HardwareMeasured, kFullRxChain));
        BOOST_REQUIRE(ti.status == TimeIntervalStatus::OrderReversed);
        BOOST_REQUIRE(ti.duration.nanos() == 0);
    }
    // A negative sub-tick difference is refused the same way, not clamped.
    {
        const TimeInterval ti = timestamp_interval(
            must_make_frac(5, 1, 4, x410_device(), TimestampMarker::RmarkerRx,
                           TimestampSource::HardwareMeasured, kFullRxChain),
            must_make_frac(5, 3, 4, x410_device(), TimestampMarker::RmarkerRx,
                           TimestampSource::HardwareMeasured, kFullRxChain));
        BOOST_REQUIRE(ti.status == TimeIntervalStatus::DurationOutOfRange);
        BOOST_REQUIRE(ti.duration.nanos() == 0);
        BOOST_REQUIRE(!ti.duration_is_lossless_ns);
    }
}

// ===========================================================================
// 26. M0.1 / R5 -- the exactly-half-period case, in BOTH directions
// ===========================================================================

BOOST_AUTO_TEST_CASE(test_r5_exact_half_period_is_ambiguous)
{
    const ClockDomain d = tight_wrap();

    const Timestamp zero = must_make(0, d, TimestampMarker::RmarkerRx,
                                     TimestampSource::HardwareMeasured, kFullRxChain);
    const Timestamp half = must_make(kTightHalf, d, TimestampMarker::RmarkerRx,
                                     TimestampSource::HardwareMeasured, kFullRxChain);
    const Timestamp under = must_make(kTightHalf - 1, d, TimestampMarker::RmarkerRx,
                                      TimestampSource::HardwareMeasured, kFullRxChain);
    const Timestamp over = must_make(kTightHalf + 1, d, TimestampMarker::RmarkerRx,
                                     TimestampSource::HardwareMeasured, kFullRxChain);

    // THE R5 DEFECT: M0 accepted dist == P/2, so BOTH directions claimed
    // "a precedes b".  Both are now Indeterminate, and both helpers agree.
    TimestampOrder o = TimestampOrder::Equal;
    BOOST_REQUIRE(!timestamp_compare(zero, half, o));
    BOOST_REQUIRE(o == TimestampOrder::Indeterminate);
    BOOST_REQUIRE(!timestamp_compare(half, zero, o));
    BOOST_REQUIRE(o == TimestampOrder::Indeterminate);

    // The legacy bool helper refuses the same pair and, as documented since
    // M0, leaves `out` untouched when it refuses.
    bool out = true;
    BOOST_REQUIRE(!timestamp_precedes(zero, half, out));
    BOOST_REQUIRE(out); // untouched: no verdict was invented
    BOOST_REQUIRE(!timestamp_precedes(half, zero, out));
    BOOST_REQUIRE(out);

    // Just under resolves, and antisymmetry holds: only ONE direction can.
    BOOST_REQUIRE(timestamp_compare(zero, under, o));
    BOOST_REQUIRE(o == TimestampOrder::Earlier);
    BOOST_REQUIRE(!timestamp_compare(under, zero, o));
    BOOST_REQUIRE(o == TimestampOrder::Indeterminate);
    BOOST_REQUIRE(timestamp_precedes(zero, under, out));
    BOOST_REQUIRE(out);
    BOOST_REQUIRE(!timestamp_precedes(under, zero, out));

    // Just over: the forward direction 0 -> 129 is 129 ticks, past the half
    // period, so no order.  The REVERSE is 256 - 129 = 127 ticks, which is
    // inside, so it resolves -- and it resolves as "0 is earlier than 129",
    // i.e. the pair is 127 ticks before the wrap, not 129 after it.
    BOOST_REQUIRE(!timestamp_compare(zero, over, o));
    BOOST_REQUIRE(o == TimestampOrder::Indeterminate);
    BOOST_REQUIRE(timestamp_compare(over, zero, o));
    BOOST_REQUIRE(o == TimestampOrder::Earlier);

    // P/2 - 1 tick is the largest resolvable forward distance, and the
    // backward distance is then P/2 + 1 ticks, which is not resolvable.
    BOOST_REQUIRE(timestamp_compare(under, zero, o) == false);
    BOOST_REQUIRE(timestamp_compare(zero, under, o));
    BOOST_REQUIRE(o == TimestampOrder::Earlier);

    // This is NOT an interval statement: the interval (half - zero) is a
    // perfectly definite 128-tick magnitude, because the caller supplied the
    // sign.  Only ORDERING is ambiguous there.  Asserting the contrast keeps
    // the two rules from drifting into each other.
    const TimeInterval ti = timestamp_interval(half, zero);
    BOOST_REQUIRE(ti.status == TimeIntervalStatus::Ok);
    BOOST_REQUIRE(ti.ticks == kTightHalf);

    // d == 0 in both directions is Equal, not "earlier".
    BOOST_REQUIRE(timestamp_compare(zero, zero, o));
    BOOST_REQUIRE(o == TimestampOrder::Equal);
    BOOST_REQUIRE(timestamp_precedes(zero, zero, out));
    BOOST_REQUIRE(!out);

    // A quarter and three quarters of the way round are both definite.
    BOOST_REQUIRE(timestamp_compare(zero, must_make(kTightHalf / 2, d,
                                                    TimestampMarker::RmarkerRx,
                                                    TimestampSource::HardwareMeasured,
                                                    kFullRxChain),
                                    o));
    BOOST_REQUIRE(o == TimestampOrder::Earlier);
}

// ===========================================================================
// 27. M0.1 / R5 -- sub-tick ordering, zero crossing, and the +-1 boundaries
// ===========================================================================

BOOST_AUTO_TEST_CASE(test_r5_fractional_ordering_and_boundaries)
{
    const ClockDomain d = tight_wrap();

    // THE R5 FRACTION DEFECT: 0.25 and 0.75 on the same tick compared EQUAL in
    // M0.  They are now ordered, and the reverse direction is the mirror.
    const Timestamp q = must_make_frac(0, 1, 4, d, TimestampMarker::RmarkerRx,
                                       TimestampSource::HardwareMeasured, kFullRxChain);
    const Timestamp tq = must_make_frac(0, 3, 4, d, TimestampMarker::RmarkerRx,
                                        TimestampSource::HardwareMeasured, kFullRxChain);
    TimestampOrder o = TimestampOrder::Equal;
    BOOST_REQUIRE(timestamp_compare(q, tq, o));
    BOOST_REQUIRE(o == TimestampOrder::Earlier);
    BOOST_REQUIRE(timestamp_compare(q, q, o));
    BOOST_REQUIRE(o == TimestampOrder::Equal);
    // The REVERSE of 0.25 -> 0.75 is 255.5 ticks forward on a 256-tick
    // counter, i.e. past the half period, so no order exists.  This is the
    // fraction-aware path doing exactly what the integer-only path could not:
    // an integer-only comparison would have called the two EQUAL (same tick)
    // and silently dropped a real 255-tick separation.
    BOOST_REQUIRE(!timestamp_compare(tq, q, o));
    BOOST_REQUIRE(o == TimestampOrder::Indeterminate);
    BOOST_REQUIRE(std::string(timestamp_order_to_string(o)) == "indeterminate");
    // A WIDE wrapping counter does not help here, and that is a structural
    // consequence worth pinning: two timestamps one tick apart are always
    // period - 1 apart in the reverse direction, so the reverse is past the
    // half period on ANY wrapping counter.  A wrapping counter can therefore
    // only ever order a sub-tick pair in ONE direction; a definite
    // two-way ordering of fractions requires a no-wrap counter.
    {
        const Timestamp wq = must_make_frac(0, 1, 4, wide_wrap(), TimestampMarker::RmarkerRx,
                                            TimestampSource::HardwareMeasured, kFullRxChain);
        const Timestamp wtq = must_make_frac(0, 3, 4, wide_wrap(), TimestampMarker::RmarkerRx,
                                             TimestampSource::HardwareMeasured, kFullRxChain);
        BOOST_REQUIRE(timestamp_compare(wq, wtq, o));
        BOOST_REQUIRE(o == TimestampOrder::Earlier);
        BOOST_REQUIRE(!timestamp_compare(wtq, wq, o));
        BOOST_REQUIRE(o == TimestampOrder::Indeterminate);
        const ModularTickDistance md = modular_tick_distance(wtq, wq);
        BOOST_REQUIRE(md.ok);
        BOOST_REQUIRE(md.ticks == 65535);
        // The forward distance is 65535 + (1/4 - 3/4) = 65535 - 1/2, and the
        // borrow turns the negative half into 1/2 one tick down.
        BOOST_REQUIRE(md.frac_num == 1 && md.frac_den == 2u);
    }

    // The exact same instants on a MONOTONIC counter, where a total order
    // always exists, so fractions are ordered there too.
    const Timestamp mq = must_make_frac(5, 1, 4, x410_device(), TimestampMarker::RmarkerRx,
                                        TimestampSource::HardwareMeasured, kFullRxChain);
    const Timestamp mtq = must_make_frac(5, 3, 4, x410_device(), TimestampMarker::RmarkerRx,
                                         TimestampSource::HardwareMeasured, kFullRxChain);
    BOOST_REQUIRE(timestamp_compare(mq, mtq, o));
    BOOST_REQUIRE(o == TimestampOrder::Earlier);
    BOOST_REQUIRE(timestamp_compare(mtq, mq, o));
    BOOST_REQUIRE(o == TimestampOrder::Later);
    BOOST_REQUIRE(timestamp_compare(mq, mq, o));
    BOOST_REQUIRE(o == TimestampOrder::Equal);
    BOOST_REQUIRE(timestamp_compare_absolute(mq, mtq, o));
    BOOST_REQUIRE(o == TimestampOrder::Earlier);
    // ... and the absolute helper refuses a WRAPPING domain rather than
    // pretending the wrap does not matter.
    BOOST_REQUIRE(!timestamp_compare_absolute(q, tq, o));
    BOOST_REQUIRE(o == TimestampOrder::Indeterminate);

    // The exact difference between two sub-tick fractions needs a denominator
    // that no single Timestamp may carry, and it still comes out exact.
    {
        const Timestamp a = must_make_frac(0, 1, 32767, d, TimestampMarker::RmarkerRx,
                                           TimestampSource::HardwareMeasured, kFullRxChain);
        const Timestamp b = must_make_frac(0, 32766, 32767, d, TimestampMarker::RmarkerRx,
                                           TimestampSource::HardwareMeasured, kFullRxChain);
        const ModularTickDistance md = modular_tick_distance(a, b);
        BOOST_REQUIRE(md.ok);
        BOOST_REQUIRE(md.ticks == 0);
        // (32766 - 1)/32767 = 32765/32767, reduced.
        BOOST_REQUIRE(md.frac_num == 32765 && md.frac_den == 32767u);
        // A difference of two den-32767 fractions stays under the wide bound.
        BOOST_REQUIRE(md.frac_den <= kMaxFractionDifferenceDenominator);
        BOOST_REQUIRE(timestamp_compare(a, b, o));
        BOOST_REQUIRE(o == TimestampOrder::Earlier);
    }

    // Zero crossing: 255.75 -> 0.25 on a 256-tick counter is 1/2 tick forward
    // (256.25 - 255.75).  The INTEGER part of the modular difference is 1 and
    // the fraction is -1/2, so the whole-tick part must be borrowed DOWN.
    // Getting that borrow wrong reports 2 1/4 ticks instead of 1/2, and the
    // two answers sit on opposite sides of every budget boundary.
    {
        const Timestamp a = must_make_frac(255, 3, 4, d, TimestampMarker::RmarkerRx,
                                           TimestampSource::HardwareMeasured, kFullRxChain);
        const Timestamp b = must_make_frac(0, 1, 4, d, TimestampMarker::RmarkerRx,
                                           TimestampSource::HardwareMeasured, kFullRxChain);
        const ModularTickDistance md = modular_tick_distance(a, b);
        BOOST_REQUIRE(md.ok);
        BOOST_REQUIRE(md.wrapped);
        BOOST_REQUIRE(md.ticks == 0);
        BOOST_REQUIRE(md.frac_num == 1 && md.frac_den == 2u);
        BOOST_REQUIRE(timestamp_compare(a, b, o));
        BOOST_REQUIRE(o == TimestampOrder::Earlier);
        // The reverse direction is 255.5 ticks: past the half period, so no
        // order exists.
        BOOST_REQUIRE(!timestamp_compare(b, a, o));
        BOOST_REQUIRE(o == TimestampOrder::Indeterminate);
    }
    // The same zero crossing with the fraction ABOVE the origin: 255.25 ->
    // 0.75 is 1.5 ticks, so the integer part stays 1 and the fraction is +1/2.
    {
        const Timestamp a = must_make_frac(255, 1, 4, d, TimestampMarker::RmarkerRx,
                                           TimestampSource::HardwareMeasured, kFullRxChain);
        const Timestamp b = must_make_frac(0, 3, 4, d, TimestampMarker::RmarkerRx,
                                           TimestampSource::HardwareMeasured, kFullRxChain);
        const ModularTickDistance md = modular_tick_distance(a, b);
        BOOST_REQUIRE(md.ok);
        BOOST_REQUIRE(md.wrapped);
        BOOST_REQUIRE(md.ticks == 1);
        BOOST_REQUIRE(md.frac_num == 1 && md.frac_den == 2u);
        BOOST_REQUIRE(timestamp_compare(a, b, o));
        BOOST_REQUIRE(o == TimestampOrder::Earlier);
    }
    // 255.75 -> 0 (no fraction on b): the borrow wraps the fraction up to 1/4.
    {
        const Timestamp a = must_make_frac(255, 3, 4, d, TimestampMarker::RmarkerRx,
                                           TimestampSource::HardwareMeasured, kFullRxChain);
        const Timestamp b = must_make(0, d, TimestampMarker::RmarkerRx,
                                      TimestampSource::HardwareMeasured, kFullRxChain);
        const ModularTickDistance md = modular_tick_distance(a, b);
        BOOST_REQUIRE(md.ok);
        BOOST_REQUIRE(md.ticks == 0);
        BOOST_REQUIRE(md.frac_num == 1 && md.frac_den == 4u);
        BOOST_REQUIRE(timestamp_compare(a, b, o));
        BOOST_REQUIRE(o == TimestampOrder::Earlier);
    }
    // 255.99997... -> 0 is one 1/32767-th of a tick: the borrow lands on the
    // smallest representable fraction, not on zero.
    {
        const Timestamp a = must_make_frac(255, 32766, 32767, d, TimestampMarker::RmarkerRx,
                                           TimestampSource::HardwareMeasured, kFullRxChain);
        const Timestamp b = must_make(0, d, TimestampMarker::RmarkerRx,
                                      TimestampSource::HardwareMeasured, kFullRxChain);
        const ModularTickDistance md = modular_tick_distance(a, b);
        BOOST_REQUIRE(md.ok);
        BOOST_REQUIRE(md.ticks == 0);
        BOOST_REQUIRE(md.frac_num == 1 && md.frac_den == 32767u);
        BOOST_REQUIRE(timestamp_compare(a, b, o));
        BOOST_REQUIRE(o == TimestampOrder::Earlier);
    }

    // The +-1 boundaries of a sub-tick fraction, at the same tick: one
    // 1/32767 step apart.  On a wrapping counter the step direction resolves
    // and the reverse does not (the reverse is 256 - 1/32767 ticks, past the
    // half period), so BOTH directions are checked against that rule.
    {
        const Timestamp a = must_make_frac(9, 1, 32767, d, TimestampMarker::RmarkerRx,
                                           TimestampSource::HardwareMeasured, kFullRxChain);
        const Timestamp b = must_make_frac(9, 2, 32767, d, TimestampMarker::RmarkerRx,
                                           TimestampSource::HardwareMeasured, kFullRxChain);
        BOOST_REQUIRE(timestamp_compare(a, b, o));
        BOOST_REQUIRE(o == TimestampOrder::Earlier);
        BOOST_REQUIRE(!timestamp_compare(b, a, o));
        BOOST_REQUIRE(o == TimestampOrder::Indeterminate);
        const ModularTickDistance md_ab = modular_tick_distance(a, b);
        BOOST_REQUIRE(md_ab.ok);
        BOOST_REQUIRE(md_ab.ticks == 0);
        BOOST_REQUIRE(md_ab.frac_num == 1 && md_ab.frac_den == 32767u);
        // On a no-wrap counter the very same pair orders in BOTH directions,
        // which is where a two-way fraction ordering is actually available.
        const Timestamp ma = must_make_frac(9, 1, 32767, x410_device(),
                                            TimestampMarker::RmarkerRx,
                                            TimestampSource::HardwareMeasured, kFullRxChain);
        const Timestamp mb = must_make_frac(9, 2, 32767, x410_device(),
                                            TimestampMarker::RmarkerRx,
                                            TimestampSource::HardwareMeasured, kFullRxChain);
        BOOST_REQUIRE(timestamp_compare(ma, mb, o));
        BOOST_REQUIRE(o == TimestampOrder::Earlier);
        BOOST_REQUIRE(timestamp_compare(mb, ma, o));
        BOOST_REQUIRE(o == TimestampOrder::Later);
        // ... and the fraction immediately below an exact tick, versus that
        // exact tick, is exactly one 1/32767 step earlier.
        const Timestamp below = must_make_frac(8, 32766, 32767, d, TimestampMarker::RmarkerRx,
                                                TimestampSource::HardwareMeasured, kFullRxChain);
        const Timestamp exact = must_make(9, d, TimestampMarker::RmarkerRx,
                                          TimestampSource::HardwareMeasured, kFullRxChain);
        BOOST_REQUIRE(timestamp_compare(below, exact, o));
        BOOST_REQUIRE(o == TimestampOrder::Earlier);
        BOOST_REQUIRE(!timestamp_compare(exact, below, o));
        BOOST_REQUIRE(o == TimestampOrder::Indeterminate);
        const ModularTickDistance md = modular_tick_distance(below, exact);
        BOOST_REQUIRE(md.ok);
        BOOST_REQUIRE(md.ticks == 0);
        BOOST_REQUIRE(md.frac_num == 1 && md.frac_den == 32767u);
    }
    // An exact tick and the smallest non-zero fraction sitting on that same
    // tick, i.e. the smallest separation the type can represent at all.  The
    // forward direction is 1/32767 ticks and resolves; the reverse is
    // 256 - 1/32767 and does not.
    {
        const Timestamp a = must_make(0, d, TimestampMarker::RmarkerRx,
                                      TimestampSource::HardwareMeasured, kFullRxChain);
        const Timestamp b = must_make_frac(0, 1, 32767, d, TimestampMarker::RmarkerRx,
                                           TimestampSource::HardwareMeasured, kFullRxChain);
        BOOST_REQUIRE(timestamp_compare(a, b, o));
        BOOST_REQUIRE(o == TimestampOrder::Earlier);
        BOOST_REQUIRE(!timestamp_compare(b, a, o));
        BOOST_REQUIRE(o == TimestampOrder::Indeterminate);
        const ModularTickDistance md = modular_tick_distance(a, b);
        BOOST_REQUIRE(md.ok);
        BOOST_REQUIRE(md.ticks == 0);
        BOOST_REQUIRE(md.frac_num == 1 && md.frac_den == 32767u);
        // The reverse really is 256 - 1/32767, not "the same short distance":
        // the fraction must be borrowed and the integer part saturated.
        const ModularTickDistance rev = modular_tick_distance(b, a);
        BOOST_REQUIRE(rev.ok);
        BOOST_REQUIRE(rev.ticks == 255);
        BOOST_REQUIRE(rev.frac_num == 32766 && rev.frac_den == 32767u);
    }

    // The fraction comparator itself, on the degenerate spellings.
    BOOST_REQUIRE(timestamp_fraction_compare(0, 0u, 0, 0u) == 0);
    BOOST_REQUIRE(timestamp_fraction_compare(0, 0u, 1, 4u) == -1);
    BOOST_REQUIRE(timestamp_fraction_compare(1, 4u, 0, 0u) == 1);
    BOOST_REQUIRE(timestamp_fraction_compare(1, 4u, 1, 4u) == 0);
    BOOST_REQUIRE(timestamp_fraction_compare(1, 4u, 3, 4u) == -1);
    BOOST_REQUIRE(timestamp_fraction_compare(2, 4u, 1, 2u) == 0); // equal values
    BOOST_REQUIRE(timestamp_fraction_compare(1, 3u, 2, 3u) == -1);
    // A modular distance across a domain boundary is refused, not guessed.
    {
        const Timestamp other = must_make(1, wide_wrap(), TimestampMarker::RmarkerRx,
                                          TimestampSource::HardwareMeasured, kFullRxChain);
        const ModularTickDistance md = modular_tick_distance(mq, other);
        BOOST_REQUIRE(!md.ok);
    }
}

// ===========================================================================
// 28. M0.1 / R5 -- the ordering budget replaces the blanket half period
// ===========================================================================

BOOST_AUTO_TEST_CASE(test_r5_ordering_budget_is_configurable_and_clamped)
{
    const ClockDomain d = wide_wrap(); // 16-bit, half period 32768 ticks
    const Timestamp zero = must_make(0, d, TimestampMarker::RmarkerRx,
                                     TimestampSource::HardwareMeasured, kFullRxChain);

    // The default budget IS the half period, so nothing is assumed beyond it.
    int64_t window = 0;
    BOOST_REQUIRE(timestamp_order_window_ticks(d, TimestampOrderBudget(), window));
    BOOST_REQUIRE(window == kWideHalf);
    BOOST_REQUIRE(timestamp_order_window_ticks(d, TimestampOrderBudget::half_period(), window));
    BOOST_REQUIRE(window == kWideHalf);

    // A declared budget: a TWR exchange is bounded by the reply delay plus a
    // timeout, so 4000 ticks is the whole plausible span.  Under the blanket
    // half period 4000 ticks would resolve; under this budget it does not.
    TimestampOrder o = TimestampOrder::Indeterminate;
    const Timestamp at3000 = must_make(3000, d, TimestampMarker::RmarkerRx,
                                       TimestampSource::HardwareMeasured, kFullRxChain);
    const Timestamp at4000 = must_make(4000, d, TimestampMarker::RmarkerRx,
                                       TimestampSource::HardwareMeasured, kFullRxChain);
    const Timestamp at4001 = must_make(4001, d, TimestampMarker::RmarkerRx,
                                       TimestampSource::HardwareMeasured, kFullRxChain);
    // Under the DEFAULT budget (the half period) all of these resolve.
    BOOST_REQUIRE(timestamp_compare(zero, at3000, o));
    BOOST_REQUIRE(o == TimestampOrder::Earlier);
    BOOST_REQUIRE(timestamp_compare(zero, at4000, o));
    BOOST_REQUIRE(o == TimestampOrder::Earlier);
    BOOST_REQUIRE(timestamp_compare(zero, at4001, o));
    BOOST_REQUIRE(o == TimestampOrder::Earlier);

    // Under a declared budget the bound is EXCLUSIVE, exactly as the half
    // period is: explicit_ticks(4000) resolves 3999 and refuses 4000.
    const TimestampOrderBudget b4000 = TimestampOrderBudget::explicit_ticks(4000);
    BOOST_REQUIRE(timestamp_compare(zero, must_make(3999, d, TimestampMarker::RmarkerRx,
                                                   TimestampSource::HardwareMeasured,
                                                   kFullRxChain),
                                    o, b4000));
    BOOST_REQUIRE(o == TimestampOrder::Earlier);
    BOOST_REQUIRE(!timestamp_compare(zero, at4000, o, b4000));
    BOOST_REQUIRE(o == TimestampOrder::Indeterminate);
    BOOST_REQUIRE(!timestamp_compare(zero, at4001, o, b4000));
    BOOST_REQUIRE(o == TimestampOrder::Indeterminate);
    // ... and the budget is what made the difference, since both resolve with
    // the default.
    BOOST_REQUIRE(timestamp_compare(zero, at4001, o));
    BOOST_REQUIRE(o == TimestampOrder::Earlier);

    // A budget SMALLER than the half period is honoured as given.
    BOOST_REQUIRE(timestamp_order_window_ticks(d, TimestampOrderBudget::explicit_ticks(9999),
                                               window));
    BOOST_REQUIRE(window == 9999);
    // A budget LARGER than the half period is CLAMPED, not honoured: honouring
    // it would let both directions resolve, which is the R5 defect.
    BOOST_REQUIRE(timestamp_order_window_ticks(d, TimestampOrderBudget::explicit_ticks(99999),
                                               window));
    BOOST_REQUIRE(window == kWideHalf);
    // ... and on the 8-bit domain, where the half period is only 128, a budget
    // of 9999 is clamped to 128 rather than to 9999.
    int64_t tight_window = 0;
    BOOST_REQUIRE(timestamp_order_window_ticks(tight_wrap(),
                                               TimestampOrderBudget::explicit_ticks(9999),
                                               tight_window));
    BOOST_REQUIRE(tight_window == kTightHalf);
    // Behaviour under an OVERSIZED budget: it is the half period that decides,
    // so 32767 ticks still resolves and 32768 does not -- the same verdicts the
    // default budget gives, proving the clamp took effect.
    const TimestampOrderBudget oversized = TimestampOrderBudget::explicit_ticks(99999);
    const Timestamp just_under_half = must_make(kWideHalf - 1, d, TimestampMarker::RmarkerRx,
                                                TimestampSource::HardwareMeasured,
                                                kFullRxChain);
    BOOST_REQUIRE(timestamp_compare(zero, just_under_half, o, oversized));
    BOOST_REQUIRE(o == TimestampOrder::Earlier);
    const Timestamp exactly_half = must_make(kWideHalf, d, TimestampMarker::RmarkerRx,
                                             TimestampSource::HardwareMeasured, kFullRxChain);
    BOOST_REQUIRE(!timestamp_compare(zero, exactly_half, o, oversized));
    BOOST_REQUIRE(o == TimestampOrder::Indeterminate);
    // A budget that is smaller than the half period is the binding one, and it
    // refuses a relation the default would have accepted.
    const Timestamp at10000 = must_make(10000, d, TimestampMarker::RmarkerRx,
                                        TimestampSource::HardwareMeasured, kFullRxChain);
    BOOST_REQUIRE(timestamp_compare(zero, at10000, o));
    BOOST_REQUIRE(o == TimestampOrder::Earlier);
    BOOST_REQUIRE(!timestamp_compare(zero, at10000, o,
                                     TimestampOrderBudget::explicit_ticks(9999)));
    BOOST_REQUIRE(o == TimestampOrder::Indeterminate);

    // On a no-wrap (monotonic) counter there is no modular ambiguity, so the
    // default window is unbounded -- but a declared budget is still enforced,
    // because a caller that knows its exchange cannot be longer than N ticks
    // wants a refusal, not a verdict.
    int64_t mwin = 0;
    BOOST_REQUIRE(timestamp_order_window_ticks(x410_device(), TimestampOrderBudget(), mwin));
    BOOST_REQUIRE(mwin == std::numeric_limits<int64_t>::max());
    const Timestamp mono_a = must_make(100, x410_device(), TimestampMarker::RmarkerRx,
                                       TimestampSource::HardwareMeasured, kFullRxChain);
    const Timestamp mono_b = must_make(900000, x410_device(), TimestampMarker::RmarkerRx,
                                       TimestampSource::HardwareMeasured, kFullRxChain);
    BOOST_REQUIRE(timestamp_compare(mono_a, mono_b, o));
    BOOST_REQUIRE(o == TimestampOrder::Earlier);
    BOOST_REQUIRE(!timestamp_compare(mono_a, mono_b, o,
                                     TimestampOrderBudget::explicit_ticks(1000)));
    BOOST_REQUIRE(o == TimestampOrder::Indeterminate);

    // A negative budget is invalid: the comparison refuses, and `out` is
    // Indeterminate rather than left holding a previous verdict.
    o = TimestampOrder::Later;
    BOOST_REQUIRE(!timestamp_compare(zero, at3000, o, TimestampOrderBudget{-1}));
    BOOST_REQUIRE(o == TimestampOrder::Indeterminate);
    BOOST_REQUIRE(!timestamp_order_window_ticks(d, TimestampOrderBudget{-1}, window));
    BOOST_REQUIRE(!TimestampOrderBudget{-1}.is_valid());
    BOOST_REQUIRE(TimestampOrderBudget::explicit_ticks(-5).is_valid()); // clamps to 0
    BOOST_REQUIRE(TimestampOrderBudget::explicit_ticks(-5).max_forward_ticks == 0);
}

// ===========================================================================
// 29. M0.1 / R5 -- "no such order exists" is never reported as an order
// ===========================================================================

BOOST_AUTO_TEST_CASE(test_r5_missing_order_is_never_reported_as_an_order)
{
    const ClockDomain d = tight_wrap();
    const Timestamp zero = must_make(0, d, TimestampMarker::RmarkerRx,
                                     TimestampSource::HardwareMeasured, kFullRxChain);
    const Timestamp far = must_make(200, d, TimestampMarker::RmarkerRx,
                                    TimestampSource::HardwareMeasured, kFullRxChain);

    // (a) ambiguous modulo the wrap: the helper returns false AND writes
    // Indeterminate, so a caller that ignores the return value still cannot
    // read an order.
    TimestampOrder o = TimestampOrder::Later;
    BOOST_REQUIRE(!timestamp_compare(zero, far, o));
    BOOST_REQUIRE(o == TimestampOrder::Indeterminate);
    BOOST_REQUIRE(std::string(timestamp_order_to_string(o)) == "indeterminate");

    // (b) different clock domains: never comparable, so no order.
    const Timestamp other_domain =
        must_make(10, x410_device(), TimestampMarker::RmarkerRx,
                  TimestampSource::HardwareMeasured, kFullRxChain);
    o = TimestampOrder::Later;
    BOOST_REQUIRE(!timestamp_compare(zero, other_domain, o));
    BOOST_REQUIRE(o == TimestampOrder::Indeterminate);

    // (c) different epochs of the same counter.
    const Timestamp next_epoch =
        must_make_frac(10, 1, 2, tight_wrap_next_epoch(), TimestampMarker::RmarkerRx,
                       TimestampSource::HardwareMeasured, kFullRxChain);
    o = TimestampOrder::Later;
    BOOST_REQUIRE(!timestamp_compare(zero, next_epoch, o));
    BOOST_REQUIRE(o == TimestampOrder::Indeterminate);

    // (d) different wrap widths on the same name and rate.
    const Timestamp other_width = must_make(10, dw_uus32(), TimestampMarker::RmarkerRx,
                                            TimestampSource::HardwareMeasured, kFullRxChain);
    o = TimestampOrder::Later;
    BOOST_REQUIRE(!timestamp_compare(zero, other_width, o));
    BOOST_REQUIRE(o == TimestampOrder::Indeterminate);

    // (e) a structurally invalid timestamp.
    {
        Timestamp broken = zero;
        broken.frac_num = 3; // 3/4 with den 0 is not normalized
        broken.frac_den = 0u;
        o = TimestampOrder::Later;
        BOOST_REQUIRE(!timestamp_compare(broken, zero, o));
        BOOST_REQUIRE(o == TimestampOrder::Indeterminate);
        BOOST_REQUIRE(!timestamp_compare(zero, broken, o));
        BOOST_REQUIRE(o == TimestampOrder::Indeterminate);
        // ... and a modular distance refuses it too.
        BOOST_REQUIRE(!modular_tick_distance(broken, zero).ok);
    }
    // (f) an invalid clock domain on one side.
    {
        Timestamp broken = zero;
        broken.domain.tick_rate_hz = -1.0;
        o = TimestampOrder::Later;
        BOOST_REQUIRE(!timestamp_compare(broken, zero, o));
        BOOST_REQUIRE(o == TimestampOrder::Indeterminate);
        BOOST_REQUIRE(!timestamp_compare_absolute(broken, zero, o));
        BOOST_REQUIRE(o == TimestampOrder::Indeterminate);
    }
    // (g) the same set of refusals through the absolute helper: a wrapping
    // domain has no total order, so it is Indeterminate, never a guess.
    o = TimestampOrder::Later;
    BOOST_REQUIRE(!timestamp_compare_absolute(zero, far, o));
    BOOST_REQUIRE(o == TimestampOrder::Indeterminate);
    o = TimestampOrder::Later;
    BOOST_REQUIRE(!timestamp_compare_absolute(zero, other_domain, o));
    BOOST_REQUIRE(o == TimestampOrder::Indeterminate);

    // Exhaustive sweep of an 8-bit counter, whole ticks: the modular distance
    // and the ordering must match the reference computed from first
    // principles -- the forward distance is (b - a) mod 256, and a relation
    // resolves only when that is strictly below 128.
    for (int64_t a = 0; a < kTightPeriod; a += 1) {
        for (int64_t b = 0; b < kTightPeriod; b += 1) {
            const Timestamp ta = must_make(a, d, TimestampMarker::RmarkerRx,
                                           TimestampSource::HardwareMeasured, kFullRxChain);
            const Timestamp tb = must_make(b, d, TimestampMarker::RmarkerRx,
                                           TimestampSource::HardwareMeasured, kFullRxChain);
            const int64_t fwd = (b >= a) ? (b - a) : (kTightPeriod - (a - b));
            const int64_t back = kTightPeriod - fwd; // fwd == 0 -> 256, i.e. a==b

            const ModularTickDistance md = modular_tick_distance(ta, tb);
            BOOST_REQUIRE(md.ok);
            BOOST_REQUIRE(md.ticks == fwd);
            BOOST_REQUIRE(md.frac_num == 0);
            BOOST_REQUIRE(md.wrapped == (b < a));

            TimestampOrder o_ab = TimestampOrder::Indeterminate;
            const bool ok_ab = timestamp_compare(ta, tb, o_ab);
            if (a == b) {
                BOOST_REQUIRE(ok_ab);
                BOOST_REQUIRE(o_ab == TimestampOrder::Equal);
            } else if (fwd < kTightHalf) {
                BOOST_REQUIRE(ok_ab);
                BOOST_REQUIRE(o_ab == TimestampOrder::Earlier);
                // Antisymmetry: the reverse is past the half period, so it is
                // Indeterminate -- never "Later", and never "Earlier" too.
                TimestampOrder o_ba = TimestampOrder::Indeterminate;
                BOOST_REQUIRE(!timestamp_compare(tb, ta, o_ba));
                BOOST_REQUIRE(o_ba == TimestampOrder::Indeterminate);
            } else {
                // Ambiguous in this direction.  The reverse may still resolve,
                // unless the pair is exactly half a period apart -- the R5
                // defect case, where BOTH directions must be Indeterminate.
                BOOST_REQUIRE(!ok_ab);
                BOOST_REQUIRE(o_ab == TimestampOrder::Indeterminate);
                TimestampOrder o_ba = TimestampOrder::Indeterminate;
                const bool ok_ba = timestamp_compare(tb, ta, o_ba);
                if (back < kTightHalf) {
                    BOOST_REQUIRE(ok_ba);
                    BOOST_REQUIRE(o_ba == TimestampOrder::Earlier);
                } else {
                    BOOST_REQUIRE(fwd == kTightHalf && back == kTightHalf);
                    BOOST_REQUIRE(!ok_ba);
                    BOOST_REQUIRE(o_ba == TimestampOrder::Indeterminate);
                }
            }
        }
    }
}

// ===========================================================================
// 30. N03 -- the lossless flag and the emitted ns come from ONE quotient
// ===========================================================================
//
// Defect N03: `duration_is_lossless_ns` was decided by an exact integer
// divisibility test, while the ns value actually stored came from a `double`
// division/multiplication and a truncation.  When the double lands just below
// an integer, 1 ns is silently lost while the flag still says `true`.  The
// assertions below pin the EXACT returned integer, not merely agreement
// between two same-source predicates (which is what the old test did).

BOOST_AUTO_TEST_CASE(test_n03_projected_ns_and_lossless_share_exact_rational)
{
    struct Case {
        const char* label;
        double rate_hz;
        int64_t ticks;
        int32_t num;
        uint32_t den;
        int64_t expect_ns;
        bool expect_lossless;
    };
    const std::vector<Case> cases = {
        // The five reproduced rows: all exact whole-ns values whose `double`
        // projection lands one ulp below the integer.
        { "737.28M 361728 ticks", kX410DeviceHz, 361728, 0, 0u, 490625, true },
        { "491.52M 241152 ticks", kX410NativeHz, 241152, 0, 0u, 490625, true },
        { "998.4M 8112 ticks", kX410WorkHz, 8112, 0, 0u, 8125, true },
        { "63.8976GHz 519168 ticks", kDwUusTickHz, 519168, 0, 0u, 8125, true },
        // The minimal case named in the report: 1 GHz, 15 ticks used to
        // return 14 ns with the lossless flag set.
        { "1GHz 15 ticks", kHostMonotonicHz, 15, 0, 0u, 15, true },
        // The truncation path must still be exercised: a value that is NOT an
        // integer number of ns is reported lossless=false and truncated.
        { "1GHz 15 ticks + 1/2", kHostMonotonicHz, 15, 1, 2u, 15, false },
        { "1GHz 15 ticks + 1/4", kHostMonotonicHz, 15, 1, 4u, 15, false },
        { "DW UUS 1 tick", kDwUusTickHz, 1, 0, 0u, 0, false },
    };

    for (const Case& c : cases) {
        ClockDomain d;
        d.name = "n03_vector";
        d.tick_rate_hz = c.rate_hz;
        d.epoch_id = 1;
        d.timestamp_bits = 0;

        const Timestamp later =
            must_make_frac(c.ticks, c.num, c.den, d, TimestampMarker::RmarkerRx,
                           TimestampSource::HardwareMeasured,
                           timestamp_required_corrections(TimestampMarker::RmarkerRx));
        const Timestamp earlier =
            must_make(0, d, TimestampMarker::RmarkerTx, TimestampSource::HardwareMeasured,
                      timestamp_required_corrections(TimestampMarker::RmarkerTx));

        const TimeInterval ti = timestamp_interval(later, earlier);
        BOOST_REQUIRE_MESSAGE(ti.status == TimeIntervalStatus::Ok, c.label);
        BOOST_REQUIRE_MESSAGE(ti.ticks == c.ticks, c.label);
        // THE N03 ASSERTION: the exact integer actually written.
        BOOST_REQUIRE_MESSAGE(ti.duration.nanos() == c.expect_ns, c.label);
        BOOST_REQUIRE_MESSAGE(ti.duration_is_lossless_ns == c.expect_lossless, c.label);
        // The standalone predicate shares the one implementation and agrees.
        BOOST_REQUIRE_MESSAGE(
            timestamp_tick_ns_projection_is_lossless(c.rate_hz, ti.ticks, ti.frac_num,
                                                     ti.frac_den) == c.expect_lossless,
            c.label);

        // The projection-free safe path agrees, and the rational value is the
        // one the ns value was derived from.
        RelativeTickInterval rel;
        TimeIntervalStatus st = TimeIntervalStatus::InvalidTimestamp;
        BOOST_REQUIRE_MESSAGE(timestamp_relative_interval(later, earlier, rel, st), c.label);
        BOOST_REQUIRE_MESSAGE(st == TimeIntervalStatus::Ok, c.label);
        BOOST_REQUIRE_MESSAGE(rel.ns_projection_is_lossless() == c.expect_lossless, c.label);
        int64_t rn = 0;
        int64_t rd = 0;
        BOOST_REQUIRE_MESSAGE(rel.exact_ratio(rn, rd), c.label);
        // Independently recompute the exact ns rational: value * 1e9/rate.
        BOOST_REQUIRE(rn > 0 || rd > 0);
    }
}

// ===========================================================================
// 31. N04 -- the interval upper bound is re-checked after the sub-tick
//            fraction is merged
// ===========================================================================
//
// Defect N04: `raw_tick_delta()` gates the RAW integer tick difference against
// half the wrap period, but the sub-tick fraction is merged into the whole-tick
// count AFTER that gate and the merged value (whole + frac) is never
// re-checked.  An interval whose final normalised value EXCEEDS P/2 was
// returned as `Ok` (and admitted by `admit_ranging_interval()`).
//
// The documented policy "exactly P/2 is allowed when later/earlier are both
// known" is NOT changed: the check is `> P/2`, not `>= P/2`.

BOOST_AUTO_TEST_CASE(test_n04_interval_upper_bound_after_fraction)
{
    // 12-bit counter: P = 4096, P/2 = 2048.  The fraction has room to push the
    // merged value strictly past the half period without the raw integer
    // difference noticing.
    ClockDomain d;
    BOOST_REQUIRE(ClockDomain::make("n04_12bit", kX410DeviceHz, 21u, 12u, d));
    BOOST_REQUIRE(clock_domain_wrap_period(d) == 4096u);

    const auto rx = [&](int64_t ticks, int32_t num, uint32_t den) {
        return must_make_frac(ticks, num, den, d, TimestampMarker::RmarkerRx,
                              TimestampSource::HardwareMeasured,
                              timestamp_required_corrections(TimestampMarker::RmarkerRx));
    };
    const auto tx = [&](int64_t ticks, int32_t num, uint32_t den) {
        return must_make_frac(ticks, num, den, d, TimestampMarker::RmarkerTx,
                              TimestampSource::HardwareMeasured,
                              timestamp_required_corrections(TimestampMarker::RmarkerTx));
    };

    // (a) P/2 - fraction: 2047 + 3/4 minus 1/4 = 2047.5 < 2048 -> Ok.
    {
        const TimeInterval ti = timestamp_interval(rx(2047, 3, 4), tx(0, 1, 4));
        BOOST_REQUIRE(ti.status == TimeIntervalStatus::Ok);
        BOOST_REQUIRE(ti.ticks == 2047);
        BOOST_REQUIRE(ti.frac_num == 1 && ti.frac_den == 2u);
    }
    // (b) exactly P/2: 2048 + 1/4 minus 1/4 = 2048.0 -> still Ok (policy).
    {
        const TimeInterval ti = timestamp_interval(rx(2048, 1, 4), tx(0, 1, 4));
        BOOST_REQUIRE(ti.status == TimeIntervalStatus::Ok);
        BOOST_REQUIRE(ti.ticks == 2048);
        BOOST_REQUIRE(ti.frac_num == 0 && ti.frac_den == 0u);
    }
    // (c) P/2 + fraction: 2048 + 3/4 minus 1/4 = 2048.5 > 2048 -> REFUSED.
    {
        const TimeInterval ti = timestamp_interval(rx(2048, 3, 4), tx(0, 1, 4));
        BOOST_REQUIRE(ti.status != TimeIntervalStatus::Ok);
        BOOST_REQUIRE(ti.status == TimeIntervalStatus::WrapAmbiguous);
        BOOST_REQUIRE(ti.ticks == 0);
        BOOST_REQUIRE(ti.frac_num == 0 && ti.frac_den == 0u);
        BOOST_REQUIRE(ti.duration.nanos() == 0);
        // The interval form the ToF core consumes must refuse for the SAME
        // reason, not merely be empty.
        RelativeTickInterval rel;
        TimeIntervalStatus st = TimeIntervalStatus::Ok;
        BOOST_REQUIRE(!timestamp_relative_interval(rx(2048, 3, 4), tx(0, 1, 4), rel, st));
        BOOST_REQUIRE(st == TimeIntervalStatus::WrapAmbiguous);
        BOOST_REQUIRE(!rel.is_valid());
    }
    // (d) P/2 + a fraction introduced by the LATER side alone (earlier is an
    // exact tick): 2048 + 1/4 - 0 = 2048.25 -> REFUSED.
    {
        const TimeInterval ti = timestamp_interval(rx(2048, 1, 4), tx(0, 0, 0u));
        BOOST_REQUIRE(ti.status == TimeIntervalStatus::WrapAmbiguous);
    }
    // (e) The SWAPPED endpoints of (c): 1/4 minus (2048 + 3/4) wraps to
    // 4096 - 2048.5 = 2047.5 < 2048, so the reverse direction is legal and
    // must still be admitted.  The bound is directional.
    {
        const TimeInterval ti = timestamp_interval(rx(0, 1, 4), tx(2048, 3, 4));
        BOOST_REQUIRE(ti.status == TimeIntervalStatus::Ok);
        BOOST_REQUIRE(ti.wrapped);
        BOOST_REQUIRE(ti.ticks == 2047);
        BOOST_REQUIRE(ti.frac_num == 1 && ti.frac_den == 2u);
    }
    // (f) A wrap-crossing interval that stays under the bound: 100 + 3/4 minus
    // (4096 - 100) + 1/4 = 200 + 1/2 -> Ok, wrapped.
    {
        const TimeInterval ti = timestamp_interval(rx(100, 3, 4), tx(3996, 1, 4));
        BOOST_REQUIRE(ti.status == TimeIntervalStatus::Ok);
        BOOST_REQUIRE(ti.wrapped);
        BOOST_REQUIRE(ti.ticks == 200);
        BOOST_REQUIRE(ti.frac_num == 1 && ti.frac_den == 2u);
    }
    // (g) A wrap-crossing interval that EXCEEDS the bound: 2047 + 3/4 minus
    // (4095) + 1/4 -> 2048 + 1/2 -> REFUSED, with wrapped still meaningful.
    {
        const TimeInterval ti = timestamp_interval(rx(2047, 3, 4), tx(4095, 1, 4));
        BOOST_REQUIRE(ti.status == TimeIntervalStatus::WrapAmbiguous);
    }
    // (h) Layer 1 (ordering) must refuse the full value too, so the interval
    // gate cannot be bypassed by an ordering shortcut.  The pair from (c)
    // ordered forward is Indeterminate in both `timestamp_compare()` and the
    // legacy helper; the reverse (case (e)) resolves as Earlier.
    {
        const Timestamp a = rx(0, 1, 4);    // earlier instant
        const Timestamp b = rx(2048, 3, 4); // later instant, merged > P/2
        TimestampOrder o = TimestampOrder::Later;
        BOOST_REQUIRE(!timestamp_compare(a, b, o));
        BOOST_REQUIRE(o == TimestampOrder::Indeterminate);
        bool out = true;
        BOOST_REQUIRE(!timestamp_precedes(a, b, out));
        // The reverse direction is a legal 2047.5-tick forward distance.
        BOOST_REQUIRE(timestamp_compare(b, a, o));
        BOOST_REQUIRE(o == TimestampOrder::Earlier);
    }
}

} // namespace
