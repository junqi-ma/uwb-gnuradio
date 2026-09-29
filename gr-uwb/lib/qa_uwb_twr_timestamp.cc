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
using gr::uwb::twr::clock_domain_tick_is_ns_or_coarser;
using gr::uwb::twr::clock_domain_ticks_in_range;
using gr::uwb::twr::clock_domain_unambiguous_interval_ns;
using gr::uwb::twr::clock_domain_unambiguous_ticks;
using gr::uwb::twr::clock_domain_wrap_period;
using gr::uwb::twr::clock_domain_wrap_period_seconds;
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

        // A whole-tick interval in a sub-nanosecond domain is NOT a faithful
        // nanosecond projection; the tick-space field is the exact one.
        const bool coarse = clock_domain_tick_is_ns_or_coarser(c.domain);
        BOOST_REQUIRE(ti.duration_is_tick_exact == coarse);
        BOOST_REQUIRE((c.domain.tick_rate_hz > 1.0e9) != coarse);
    }

    // A zero interval is legal and exact.
    const Timestamp z = must_make(4242, x410_device(), TimestampMarker::RmarkerTx,
                                  TimestampSource::ScheduledCalibrated, kFullTxChain);
    const TimeInterval zi = timestamp_interval(z, z);
    BOOST_REQUIRE(zi.status == TimeIntervalStatus::Ok);
    BOOST_REQUIRE(zi.ticks == 0);
    BOOST_REQUIRE(zi.duration.is_zero());
    BOOST_REQUIRE(zi.duration_is_tick_exact);

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
    BOOST_REQUIRE(!ti.duration_is_tick_exact);

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
    BOOST_REQUIRE(!ti4.duration_is_tick_exact);

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
    BOOST_REQUIRE(ij.find("\"duration_is_tick_exact\":false") != std::string::npos);
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
    // ... but three of the four carry a sub-tick part and one uses a
    // sub-nanosecond tick, so only the exactness flag tells the ToF core
    // which representation it must use.
    BOOST_REQUIRE(!ra.duration_is_tick_exact);
    BOOST_REQUIRE(!da.duration_is_tick_exact);
    BOOST_REQUIRE(!db.duration_is_tick_exact);
    BOOST_REQUIRE(!rb.duration_is_tick_exact);

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
}

} // namespace
