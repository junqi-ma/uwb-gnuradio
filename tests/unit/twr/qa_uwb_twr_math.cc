/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * QA for the M1-A SS/DS ToF mathematics (uwb_twr_math.h).
 *
 * WHAT IS INDEPENDENT HERE
 * ------------------------
 * The golden values come from `testdata/twr/tof_oracle_vectors.json`, which is
 * produced from a PHYSICAL model (see gen_tof_oracle.py / generate_tof_oracle.m)
 * and never from this C++.  This QA reads that file, rebuilds each vector's
 * admitted intervals through the REAL ranging gate, calls `compute_*`, and
 * compares the exact rational results.  The failure matrix below uses
 * hand-written expectations, not a second call to the same predicate (N03).
 *
 * The MATLAB oracle of record could NOT be run on this machine (no MATLAB
 * installation); the JSON is from the Python reference of the same model.  That
 * is stated in the JSON provenance and in the M1-A report.  It is NOT MATLAB
 * verification, and this file does not claim otherwise.
 *
 * Everything is radio-free: no GNU Radio, no UHD, no device.  The 40-bit DW
 * UUS grid is used because its 15.645 ps tick makes every integer-nanosecond
 * projection lossy, which the math must never rely on.
 */

#include <boost/test/unit_test.hpp>

#include <gnuradio/uwb/uwb_twr_math.h>
// TEST-ONLY: the shared JSON reader.  The math HEADER deliberately does not
// include the config layer; a test may, and reusing this parser avoids a
// second, unverified JSON implementation.
#include <gnuradio/uwb/uwb_twr_config.h>

#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>
#include <type_traits>
#include <vector>

#ifndef UWB_TESTDATA_DIR
#define UWB_TESTDATA_DIR "../../../testdata"
#endif

namespace boost {
namespace test_tools {
namespace tt_detail {

template <>
struct print_log_value<gr::uwb::twr::TofStatus> {
    void operator()(std::ostream& os, gr::uwb::twr::TofStatus const& s) const
    {
        os << gr::uwb::twr::tof_status_to_string(s);
    }
};

template <>
struct print_log_value<gr::uwb::twr::ClockRatioSource> {
    void operator()(std::ostream& os, gr::uwb::twr::ClockRatioSource const& s) const
    {
        os << gr::uwb::twr::clock_ratio_source_to_string(s);
    }
};

template <>
struct print_log_value<gr::uwb::twr::ComputedAt> {
    void operator()(std::ostream& os, gr::uwb::twr::ComputedAt const& c) const
    {
        os << gr::uwb::twr::computed_at_to_string(c);
    }
};

template <>
struct print_log_value<gr::uwb::twr::ExchangeStatus> {
    void operator()(std::ostream& os, gr::uwb::twr::ExchangeStatus const& s) const
    {
        os << gr::uwb::twr::exchange_status_to_string(s);
    }
};

} // namespace tt_detail
} // namespace test_tools
} // namespace boost

BOOST_AUTO_TEST_SUITE(twr_math)

namespace {

using namespace gr::uwb::twr;
using gr::uwb::twr::json::Type;
using gr::uwb::twr::json::Value;

// ---------------------------------------------------------------------------
// Structural detectors (COM-2 / COM-3).  Every negative claim is paired with
// a positive control below, so a detector that never fires is caught.
// ---------------------------------------------------------------------------
template <typename T, typename = void>
struct has_nanos_member : std::false_type {};
template <typename T>
struct has_nanos_member<T, std::void_t<decltype(std::declval<const T&>().nanos())>>
    : std::true_type {};

template <typename T, typename = void>
struct has_seconds_member : std::false_type {};
template <typename T>
struct has_seconds_member<T, std::void_t<decltype(std::declval<const T&>().seconds())>>
    : std::true_type {};

// ---------------------------------------------------------------------------
// Reading the oracle
// ---------------------------------------------------------------------------
const char* const kOracleRelative = "twr/tof_oracle_vectors.json";

Value read_oracle()
{
    const std::string path = std::string(UWB_TESTDATA_DIR) + "/" + kOracleRelative;
    std::ifstream in(path, std::ios::binary);
    if (!in)
        BOOST_FAIL("cannot open the ToF oracle at " << path
                                                    << " (is UWB_TESTDATA_DIR set?)");
    std::stringstream ss;
    ss << in.rdbuf();
    Value root;
    std::string err;
    if (!json::parse(ss.str(), root, err))
        BOOST_FAIL("cannot parse the ToF oracle at " << path << ": " << err);
    return root;
}

const Value& member(const Value& v, const std::string& key)
{
    const Value* m = v.find(key);
    BOOST_REQUIRE_MESSAGE(m != nullptr, "missing JSON member '" << key << "'");
    return *m;
}

int64_t as_i64(const Value& v)
{
    if (v.type == Type::Int)
        return v.integer;
    if (v.type == Type::Double)
        return static_cast<int64_t>(v.number);
    BOOST_FAIL("expected an integer, got " << v.type_name());
    return 0;
}

// ---------------------------------------------------------------------------
// Fixtures
// ---------------------------------------------------------------------------

struct Endpoint {
    int64_t ticks = 0;
    int32_t num = 0;
    uint32_t den = 0;
    TimestampMarker marker = TimestampMarker::RmarkerRx;
};

Endpoint endpoint_from_json(const Value& interval, const std::string& side,
                            const std::string& marker_key)
{
    const Value& v = member(interval, side);
    Endpoint e;
    e.ticks = as_i64(member(v, "ticks"));
    e.num = static_cast<int32_t>(as_i64(member(v, "num")));
    e.den = static_cast<uint32_t>(as_i64(member(v, "den")));
    const std::string ms = member(interval, marker_key).text;
    BOOST_REQUIRE_MESSAGE(timestamp_marker_from_string(ms, e.marker),
                          "unknown marker '" << ms << "'");
    return e;
}

bool domain_from_json(const Value& v, ClockDomain& out)
{
    return ClockDomain::make(member(v, "name").text, member(v, "tick_rate_hz").as_double(),
                             static_cast<uint64_t>(as_i64(member(v, "epoch"))),
                             static_cast<uint32_t>(as_i64(member(v, "timestamp_bits"))),
                             out);
}

// A calibration that is in force across the whole exchange.  Mirrors the
// tof_input QA fixture, but the window is deliberately wider than any vector.
CalibrationStamp calibration_for(const ClockDomain& d, const std::string& id)
{
    CalibrationStamp c;
    c.id = id;
    c.calibrated_epoch = d.epoch_id;
    c.valid_from_ticks = 1000;
    c.valid_until_ticks = 2000000;
    CalibrationApplication a;
    a.calibration_id = id;
    a.result = CalibrationResult::Applied;
    c.applications.push_back(a);
    return c;
}

// Build ONE admitted interval from two endpoint specifications, through the
// real gate.  No test backdoor into RangeCapableTime.
bool admit(const ClockDomain& d, const Endpoint& later, const Endpoint& earlier,
           RangingIntervalAdmission& out)
{
    const std::string cal_id = "cal-m1a-" + d.name;
    const CalibrationStamp cal = calibration_for(d, cal_id);

    Timestamp lts;
    Timestamp ets;
    if (!Timestamp::from_fractional_ticks(later.ticks, later.num, later.den, d, later.marker,
                                          TimestampSource::HardwareMeasured, kCorrectionNone,
                                          lts))
        return false;
    if (!Timestamp::from_fractional_ticks(earlier.ticks, earlier.num, earlier.den, d,
                                          earlier.marker, TimestampSource::HardwareMeasured,
                                          kCorrectionNone, ets))
        return false;

    // The full required correction set for each marker, then the calibration.
    const uint32_t lneed = timestamp_required_corrections(later.marker);
    const uint32_t eneed = timestamp_required_corrections(earlier.marker);
    if (apply_calibration_ticks(lts, cal_id, 0, 0, 0u, lneed) != CalibrationResult::Applied)
        return false;
    if (apply_calibration_ticks(ets, cal_id, 0, 0, 0u, eneed) != CalibrationResult::Applied)
        return false;

    RangeAdmissionContext ctx;
    ctx.calibration = &cal;
    ctx.reference_ticks = 1500;
    ctx.reference_ticks_recorded = true;
    ctx.rx_first_path = FirstPathQuality::passed(18.5, 9.0, 0.82);

    out = admit_ranging_interval(lts, ets, ctx);
    return out.admitted;
}

bool ratio_from_json(const Value& v, const ClockDomain& a, const ClockDomain& b,
                     ClockRatio& out)
{
    const Value& kr = member(v, "k");
    const int64_t kn = as_i64(member(kr, "num"));
    const int64_t kd = as_i64(member(kr, "den"));
    ClockRatioSource s = ClockRatioSource::NominalSameClock;
    BOOST_REQUIRE_MESSAGE(clock_ratio_source_from_string(member(v, "source").text, s),
                          "unknown ratio source");
    if (s == ClockRatioSource::NominalSameClock)
        return ClockRatio::unity_same_clock(a, out);
    return ClockRatio::make(a, b, kn, kd, s, out);
}

TofRationalTicks ticks_from_json(const Value& v)
{
    TofRationalTicks t;
    t.num = as_i64(member(v, "num"));
    t.den = as_i64(member(v, "den"));
    t.valid = true;
    return t;
}

// Convenience for the hand-written cases: one whole-tick interval in `d`.
// `rx_later` picks the marker pair (Rx later / Tx earlier, or the reverse).
bool admit_whole(const ClockDomain& d, int64_t diff, bool rx_later,
                 RangingIntervalAdmission& out)
{
    Endpoint earlier;
    earlier.ticks = 0;
    earlier.marker = rx_later ? TimestampMarker::RmarkerTx : TimestampMarker::RmarkerRx;
    Endpoint later;
    later.ticks = diff;
    later.marker = rx_later ? TimestampMarker::RmarkerRx : TimestampMarker::RmarkerTx;
    return admit(d, later, earlier, out);
}

} // namespace

// ===========================================================================
// The oracle: every vector from testdata/twr/tof_oracle_vectors.json
// ===========================================================================
BOOST_AUTO_TEST_CASE(math_oracle_vectors_match)
{
    const Value root = read_oracle();
    const Value& vectors = member(root, "vectors");
    BOOST_REQUIRE_MESSAGE(vectors.type == Type::Array, "vectors must be an array");
    BOOST_REQUIRE_MESSAGE(vectors.items.size() >= 10,
                          "the oracle must carry the full vector set");

    for (const Value& v : vectors.items) {
        const std::string id = member(v, "id").text;
        const std::string proto = member(v, "protocol").text;

        ClockDomain a;
        ClockDomain b;
        BOOST_REQUIRE_MESSAGE(domain_from_json(member(v, "domain_a"), a),
                              id << ": bad A domain");
        BOOST_REQUIRE_MESSAGE(domain_from_json(member(v, "domain_b"), b),
                              id << ": bad B domain");

        ClockRatio k;
        BOOST_REQUIRE_MESSAGE(ratio_from_json(member(v, "clock_ratio"), a, b, k),
                              id << ": ratio could not be built");

        const Value& ivs = member(v, "intervals");
        const Value& ra_j = member(ivs, "ra");
        const Value& db_j = member(ivs, "db");

        RangingIntervalAdmission ra;
        RangingIntervalAdmission db;
        BOOST_REQUIRE_MESSAGE(
            admit(a, endpoint_from_json(ra_j, "later", "later_marker"),
                  endpoint_from_json(ra_j, "earlier", "earlier_marker"), ra),
            id << ": ra did not pass the ranging gate");
        BOOST_REQUIRE_MESSAGE(
            admit(b, endpoint_from_json(db_j, "later", "later_marker"),
                  endpoint_from_json(db_j, "earlier", "earlier_marker"), db),
            id << ": db did not pass the ranging gate");
        BOOST_REQUIRE_MESSAGE(ra.value.has_value() && db.value.has_value(),
                              id << ": admitted but no value");

        TofResult r;
        if (proto == "ss") {
            r = compute_ss_tof(*ra.value, *db.value, k);
        } else {
            const Value& rb_j = member(ivs, "rb");
            const Value& da_j = member(ivs, "da");
            RangingIntervalAdmission rb;
            RangingIntervalAdmission da;
            BOOST_REQUIRE_MESSAGE(
                admit(b, endpoint_from_json(rb_j, "later", "later_marker"),
                      endpoint_from_json(rb_j, "earlier", "earlier_marker"), rb),
                id << ": rb did not pass the ranging gate");
            BOOST_REQUIRE_MESSAGE(
                admit(a, endpoint_from_json(da_j, "later", "later_marker"),
                      endpoint_from_json(da_j, "earlier", "earlier_marker"), da),
                id << ": da did not pass the ranging gate");
            BOOST_REQUIRE_MESSAGE(rb.value.has_value() && da.value.has_value(),
                                  id << ": admitted but no value");
            r = compute_ds_tof(*ra.value, *rb.value, *da.value, *db.value, k);
        }

        const Value& exp = member(v, "expected");
        const std::string estatus = member(exp, "status").text;
        const TofRationalTicks expected = ticks_from_json(member(exp, "tof"));

        // The result is always expressed in A ticks.
        BOOST_REQUIRE_MESSAGE(clock_domain_same_identity(r.domain, a),
                              id << ": result domain is not A");

        if (estatus == "ok") {
            BOOST_REQUIRE_MESSAGE(r.ok, id << ": expected Ok, got " << r.to_string());
            BOOST_REQUIRE_MESSAGE(r.status == TofStatus::Ok, id);
            BOOST_REQUIRE_MESSAGE(r.exchange_status == ExchangeStatus::Ok, id);
            BOOST_REQUIRE_MESSAGE(
                tof_rational_equals(r.tof, expected),
                id << ": ToF " << r.tof.to_string() << " != oracle "
                   << expected.to_string());
        } else if (estatus == "negative_tof") {
            BOOST_REQUIRE_MESSAGE(!r.ok, id << ": a negative ToF must not be Ok");
            BOOST_REQUIRE_MESSAGE(r.status == TofStatus::NegativeTof, id);
            BOOST_REQUIRE_MESSAGE(r.exchange_status == ExchangeStatus::NegativeTof,
                                  id << ": negative ToF must terminate as negative_tof");
            BOOST_CHECK_MESSAGE(r.tof.negative(), id << ": sign was lost");
            BOOST_REQUIRE_MESSAGE(
                tof_rational_equals(r.tof, expected),
                id << ": signed ToF " << r.tof.to_string() << " != oracle "
                   << expected.to_string());
        } else {
            BOOST_FAIL(id << ": unknown expected status '" << estatus << "'");
        }

        // A negative ToF must never become a distance.
        if (!r.ok) {
            double d = 0.0;
            BOOST_TEST_MESSAGE(id << ": " << r.to_string());
            BOOST_TEST(!tof_distance_m(r, kPropagationSpeedVacuumMps, d));
        }
    }
}

// ===========================================================================
// Hand-calculated cases (numbers written out, not read from a predicate)
// ===========================================================================
BOOST_AUTO_TEST_CASE(math_ss_common_clock_hand_check)
{
    // The install-consumer example.  Common clock (k = 1 BY STATEMENT),
    // RA = 2000 ticks, DB = 1000 ticks -> ToF = (2000 - 1000)/2 = 500 ticks.
    ClockDomain a;
    BOOST_REQUIRE(ClockDomain::make("dw_uus_a", 499.2e6 * 128.0, 7, 40, a));
    ClockRatio k;
    BOOST_REQUIRE(ClockRatio::unity_same_clock(a, k));
    BOOST_REQUIRE(k.check() == TofStatus::Ok);
    BOOST_REQUIRE(k.source() == ClockRatioSource::NominalSameClock);
    BOOST_REQUIRE(k.k_num() == 1 && k.k_den() == 1);

    RangingIntervalAdmission ra;
    RangingIntervalAdmission db;
    BOOST_REQUIRE(admit_whole(a, 2000, true, ra));
    BOOST_REQUIRE(admit_whole(a, 1000, false, db));

    const TofResult r = compute_ss_tof(*ra.value, *db.value, k);
    BOOST_REQUIRE_MESSAGE(r.ok, r.to_string());
    BOOST_REQUIRE_EQUAL(r.tof.num, 500);
    BOOST_REQUIRE_EQUAL(r.tof.den, 1);
    BOOST_REQUIRE(r.computed_at == ComputedAt::InitiatorA); // REQ-PROTO-05
    BOOST_REQUIRE(r.protocol == Protocol::Ss);
    // The display projection is there but is not the truth.
    BOOST_REQUIRE_CLOSE(r.display_seconds, 500.0 / (499.2e6 * 128.0), 1e-9);

    // The optional distance uses the same exact ToF and a named speed.
    double d = 0.0;
    BOOST_REQUIRE(tof_distance_m(r, kPropagationSpeedVacuumMps, d));
    BOOST_REQUIRE_CLOSE(d, kPropagationSpeedVacuumMps * 500.0 / (499.2e6 * 128.0), 1e-9);
    // A test can inject a slower (cable) velocity instead of the vacuum one.
    double cable = 0.0;
    BOOST_REQUIRE(tof_distance_m(r, 200000000.0, cable));
    BOOST_REQUIRE_CLOSE(cable, 200000000.0 * 500.0 / (499.2e6 * 128.0), 1e-9);
    BOOST_REQUIRE(cable < d);
}

BOOST_AUTO_TEST_CASE(math_ds_asymmetric_hand_check)
{
    // RA=2000, RB=4000, DA=3000, DB=1000 (asymmetric replies), k=1:
    //   N = 2000*4000 - 3000*1000 = 8e6 - 3e6 = 5e6
    //   D = 2000 + 4000 + 3000 + 1000 = 10000
    //   ToF = 5e6/1e4 = 500 ticks   (the same physical ToF as the SS case)
    ClockDomain a;
    BOOST_REQUIRE(ClockDomain::make("dw_uus_a", 499.2e6 * 128.0, 7, 40, a));
    ClockRatio k;
    BOOST_REQUIRE(ClockRatio::unity_same_clock(a, k));

    RangingIntervalAdmission ra;
    RangingIntervalAdmission rb;
    RangingIntervalAdmission da;
    RangingIntervalAdmission db;
    BOOST_REQUIRE(admit_whole(a, 2000, true, ra));
    BOOST_REQUIRE(admit_whole(a, 4000, true, rb));
    BOOST_REQUIRE(admit_whole(a, 3000, false, da));
    BOOST_REQUIRE(admit_whole(a, 1000, false, db));

    const TofResult r = compute_ds_tof(*ra.value, *rb.value, *da.value, *db.value, k);
    BOOST_REQUIRE_MESSAGE(r.ok, r.to_string());
    BOOST_REQUIRE_EQUAL(r.tof.num, 500);
    BOOST_REQUIRE_EQUAL(r.tof.den, 1);
    BOOST_REQUIRE(r.computed_at == ComputedAt::ResponderB); // REQ-PROTO-05
    BOOST_REQUIRE(r.protocol == Protocol::Ds);
}

BOOST_AUTO_TEST_CASE(math_ss_nominal_rate_ratio_from_factory)
{
    // Two nominal rates 1.0 GHz and 998.4 MHz: k = 1000000000/998400000
    // reduces to 625/624.  With RA = 2000 A ticks and DB = 4992/5 B ticks,
    // k*DB = (625/624)*(4992/5) = 1000 A ticks, so ToF = (2000-1000)/2 = 500.
    ClockDomain a;
    ClockDomain b;
    BOOST_REQUIRE(ClockDomain::make("dw_a", 1.0e9, 7, 40, a));
    BOOST_REQUIRE(ClockDomain::make("dw_b", 998.4e6, 7, 40, b));
    ClockRatio k;
    BOOST_REQUIRE(ClockRatio::from_nominal_rates(a, b, 1000000000LL, 998400000LL, k));
    BOOST_REQUIRE_EQUAL(k.k_num(), 625);
    BOOST_REQUIRE_EQUAL(k.k_den(), 624);
    BOOST_REQUIRE(k.source() == ClockRatioSource::NominalRateRatio);

    RangingIntervalAdmission ra;
    BOOST_REQUIRE(admit_whole(a, 2000, true, ra));

    // DB = 4992/5 B ticks, built through the gate.
    Endpoint be;
    be.ticks = 0;
    be.marker = TimestampMarker::RmarkerRx;
    Endpoint bl;
    bl.ticks = 998;
    bl.num = 2;
    bl.den = 5;
    bl.marker = TimestampMarker::RmarkerTx;
    RangingIntervalAdmission db;
    BOOST_REQUIRE(admit(b, bl, be, db));

    const TofResult r = compute_ss_tof(*ra.value, *db.value, k);
    BOOST_REQUIRE_MESSAGE(r.ok, r.to_string());
    BOOST_REQUIRE_EQUAL(r.tof.num, 500);
    BOOST_REQUIRE_EQUAL(r.tof.den, 1);
    // The B interval as consumed is in A ticks, not B ticks.
    BOOST_REQUIRE_EQUAL(r.db.num, 1000);
    BOOST_REQUIRE_EQUAL(r.db.den, 1);
    BOOST_REQUIRE_EQUAL(r.db_raw.num, 4992);
    BOOST_REQUIRE_EQUAL(r.db_raw.den, 5);
}

// ===========================================================================
// Failure matrix (SS-4..7, DS-4..6, COM-1..3)
// ===========================================================================
namespace {

ClockDomain uus(const std::string& name, uint64_t epoch = 7u)
{
    ClockDomain d;
    BOOST_REQUIRE_MESSAGE(ClockDomain::make(name, 499.2e6 * 128.0, epoch, 40, d),
                          "domain " << name << " did not construct");
    return d;
}

} // namespace

BOOST_AUTO_TEST_CASE(math_ss_independent_clocks_need_an_explicit_ratio)
{
    // SS-4: two INDEPENDENT clocks with no ratio supplied.  k = 1 must never
    // be assumed, so a default-constructed ClockRatio is refused as
    // `clock_ratio_missing` -> ExchangeStatus::ClockEstimateInvalid.
    const ClockDomain a = uus("dw_a");
    const ClockDomain b = uus("dw_b");

    RangingIntervalAdmission ra;
    RangingIntervalAdmission db;
    BOOST_REQUIRE(admit_whole(a, 2000, true, ra));
    BOOST_REQUIRE(admit_whole(b, 1000, false, db));

    const ClockRatio missing; // never constructed
    BOOST_REQUIRE(!missing.provided());
    BOOST_REQUIRE(missing.check() == TofStatus::ClockRatioMissing);

    const TofResult r = compute_ss_tof(*ra.value, *db.value, missing);
    BOOST_REQUIRE(!r.ok);
    BOOST_REQUIRE(r.status == TofStatus::ClockRatioMissing);
    BOOST_REQUIRE(r.exchange_status == ExchangeStatus::ClockEstimateInvalid);
    BOOST_REQUIRE(!r.tof.valid);
}

BOOST_AUTO_TEST_CASE(math_ss_expired_ratio_is_refused)
{
    // SS-5: k is valid on [0,100] but the exchange's RA spans [0,2000].
    const ClockDomain a = uus("dw_a");
    ClockRatio k;
    BOOST_REQUIRE(ClockRatio::unity_same_clock(a, k));
    BOOST_REQUIRE(k.set_validity_window(0, 100));
    BOOST_REQUIRE(k.covers_ticks(50));
    BOOST_REQUIRE(!k.covers_ticks(2000));

    RangingIntervalAdmission ra;
    RangingIntervalAdmission db;
    BOOST_REQUIRE(admit_whole(a, 2000, true, ra));
    BOOST_REQUIRE(admit_whole(a, 1000, false, db));

    const TofResult r = compute_ss_tof(*ra.value, *db.value, k);
    BOOST_REQUIRE(!r.ok);
    BOOST_REQUIRE(r.status == TofStatus::ClockRatioNotValidAtTime);
    BOOST_REQUIRE(r.exchange_status == ExchangeStatus::ClockEstimateInvalid);
    BOOST_REQUIRE(r.detail.find("validity window") != std::string::npos);
}

BOOST_AUTO_TEST_CASE(math_ss_epoch_and_domain_mismatch_are_distinct)
{
    // SS-6: same counter identity, different epoch -> a DOMAIN failure named
    // as an epoch problem, not as a missing ratio.
    const ClockDomain a7 = uus("dw_a", 7u);
    const ClockDomain a8 = uus("dw_a", 8u);
    ClockRatio k;
    BOOST_REQUIRE(ClockRatio::unity_same_clock(a8, k));

    RangingIntervalAdmission ra;
    RangingIntervalAdmission db;
    BOOST_REQUIRE(admit_whole(a7, 2000, true, ra));
    BOOST_REQUIRE(admit_whole(a7, 1000, false, db));

    const TofResult r = compute_ss_tof(*ra.value, *db.value, k);
    BOOST_REQUIRE(!r.ok);
    BOOST_REQUIRE(r.status == TofStatus::ClockRatioEpochMismatch);
    BOOST_REQUIRE(r.exchange_status == ExchangeStatus::InvalidTimeDomain);

    // A genuinely different counter identity is reported as a domain
    // mismatch instead.
    const ClockDomain other = uus("dw_other", 7u);
    ClockRatio k2;
    BOOST_REQUIRE(ClockRatio::unity_same_clock(other, k2));
    const TofResult r2 = compute_ss_tof(*ra.value, *db.value, k2);
    BOOST_REQUIRE(!r2.ok);
    BOOST_REQUIRE(r2.status == TofStatus::ClockRatioDomainMismatch);
    BOOST_REQUIRE(r2.detail.find("dw_other") != std::string::npos);
}

BOOST_AUTO_TEST_CASE(math_ds_zero_denominator_is_refused)
{
    // DS-4: all four intervals zero -> the denominator is not positive.
    const ClockDomain a = uus("dw_a");
    ClockRatio k;
    BOOST_REQUIRE(ClockRatio::unity_same_clock(a, k));

    RangingIntervalAdmission ra;
    RangingIntervalAdmission rb;
    RangingIntervalAdmission da;
    RangingIntervalAdmission db;
    BOOST_REQUIRE_MESSAGE(admit_whole(a, 0, true, ra), "a zero interval must be formable");
    BOOST_REQUIRE(admit_whole(a, 0, true, rb));
    BOOST_REQUIRE(admit_whole(a, 0, false, da));
    BOOST_REQUIRE(admit_whole(a, 0, false, db));

    const TofResult r = compute_ds_tof(*ra.value, *rb.value, *da.value, *db.value, k);
    BOOST_REQUIRE(!r.ok);
    BOOST_REQUIRE(r.status == TofStatus::ZeroDenominator);
    BOOST_REQUIRE(r.exchange_status == ExchangeStatus::InvalidTimeDomain);
}

BOOST_AUTO_TEST_CASE(math_ds_overflow_is_refused_not_wrapped)
{
    // DS-5: a huge clock ratio drives RA*k*RB past 128 bits.  The answer must
    // be an explicit Overflow, never a wrapped value and never a double.
    const ClockDomain a = uus("dw_a");
    const ClockDomain b = uus("dw_b");
    ClockRatio k;
    BOOST_REQUIRE(ClockRatio::make(a, b, (1LL << 62), 1, ClockRatioSource::NominalRateRatio,
                                   k));

    // Each interval is 2^38 + 1/32767 ticks, comfortably inside the 40-bit
    // unambiguous window.
    const int64_t big = (1LL << 38);
    Endpoint e;
    e.ticks = 0;
    e.marker = TimestampMarker::RmarkerRx;
    Endpoint l;
    l.ticks = big;
    l.num = 1;
    l.den = 32767;
    l.marker = TimestampMarker::RmarkerTx;

    RangingIntervalAdmission ra;
    RangingIntervalAdmission rb;
    RangingIntervalAdmission da;
    RangingIntervalAdmission db;
    BOOST_REQUIRE_MESSAGE(admit(a, l, e, ra), "ra must admit");
    BOOST_REQUIRE_MESSAGE(admit(b, l, e, rb), "rb must admit");
    BOOST_REQUIRE_MESSAGE(admit(a, l, e, da), "da must admit");
    BOOST_REQUIRE_MESSAGE(admit(b, l, e, db), "db must admit");

    const TofResult r = compute_ds_tof(*ra.value, *rb.value, *da.value, *db.value, k);
    BOOST_REQUIRE(!r.ok);
    BOOST_REQUIRE(r.status == TofStatus::Overflow);
    BOOST_REQUIRE(r.exchange_status == ExchangeStatus::InternalError);
}

BOOST_AUTO_TEST_CASE(math_clock_ratio_enum_domain_is_fail_closed)
{
    // COM-1: an out-of-domain source is refused at CONSTRUCTION, and the
    // standalone predicate is fail-closed.  (`ClockRatio::check()` re-tests it
    // as defence in depth, but a valid object cannot hold it.)
    const ClockDomain a = uus("dw_a");
    const ClockDomain b = uus("dw_b");
    const ClockRatioSource bogus = static_cast<ClockRatioSource>(250);
    BOOST_TEST(!clock_ratio_source_is_known(bogus));
    BOOST_TEST(clock_ratio_source_is_known(ClockRatioSource::NominalSameClock));
    BOOST_TEST(std::string(clock_ratio_source_to_string(bogus)) == "invalid");

    ClockRatio out;
    BOOST_TEST(!ClockRatio::make(a, b, 1, 1, bogus, out));
    BOOST_TEST(!out.provided());

    ClockRatioSource back = ClockRatioSource::NominalSameClock;
    BOOST_TEST(!clock_ratio_source_from_string("not_a_source", back));
    BOOST_TEST(clock_ratio_source_from_string("estimated", back));
    BOOST_TEST(back == ClockRatioSource::Estimated);

    // Negative k and a zero denominator are refused too, never used to flip a
    // formula.
    BOOST_TEST(!ClockRatio::make(a, b, -1, 1, ClockRatioSource::Calibrated, out));
    BOOST_TEST(!ClockRatio::make(a, b, 1, 0, ClockRatioSource::Calibrated, out));
    BOOST_TEST(!ClockRatio::make(a, b, 1, -1, ClockRatioSource::Calibrated, out));
}

BOOST_AUTO_TEST_CASE(math_clock_ratio_same_clock_statement_is_enforced)
{
    // "NominalSameClock" MEANS A and B are one physical clock.  It must not be
    // used to smuggle k=1 onto two different counters, and a rate ratio over
    // one clock is a contradiction.
    const ClockDomain a = uus("dw_a");
    const ClockDomain b = uus("dw_b");
    ClockRatio out;

    // Positive control: one clock, k=1, is fine.
    BOOST_TEST(ClockRatio::make(a, a, 1, 1, ClockRatioSource::NominalSameClock, out));
    BOOST_TEST(out.provided());

    // Same clock with k != 1 contradicts the source.
    BOOST_TEST(!ClockRatio::make(a, a, 2, 1, ClockRatioSource::NominalSameClock, out));
    // Different counters cannot be "the same clock".
    BOOST_TEST(!ClockRatio::make(a, b, 1, 1, ClockRatioSource::NominalSameClock, out));
    // A rate ratio over one clock requires the rates to agree.
    BOOST_TEST(!ClockRatio::from_nominal_rates(a, a, 1000000000LL, 998400000LL, out));
    BOOST_TEST(ClockRatio::from_nominal_rates(a, a, 1000000000LL, 1000000000LL, out));
}

BOOST_AUTO_TEST_CASE(math_optional_ratio_fields_reject_bad_values)
{
    // §8.4: the OPTIONAL window/uncertainty are absent by default and must be
    // validated when present.
    const ClockDomain a = uus("dw_a");
    ClockRatio k;
    BOOST_REQUIRE(ClockRatio::unity_same_clock(a, k));
    BOOST_REQUIRE(!k.has_validity_window());
    BOOST_REQUIRE(!k.has_uncertainty());
    BOOST_REQUIRE(k.covers_ticks(std::numeric_limits<int64_t>::max())); // no window

    BOOST_TEST(!k.set_validity_window(10, 5)); // inverted
    BOOST_TEST(!k.set_uncertainty_ppm(-1, 1));
    BOOST_TEST(!k.set_uncertainty_ppm(1, 0));

    BOOST_REQUIRE(k.set_validity_window(0, 100));
    BOOST_REQUIRE(k.set_uncertainty_ppm(3, 2));
    BOOST_REQUIRE(k.has_validity_window());
    BOOST_REQUIRE(k.has_uncertainty());
    BOOST_REQUIRE_EQUAL(k.uncertainty_ppm_num(), 3);
    BOOST_REQUIRE_EQUAL(k.uncertainty_ppm_den(), 2);
    BOOST_REQUIRE(k.check() == TofStatus::Ok);
}

BOOST_AUTO_TEST_CASE(math_tof_status_taxonomy_and_mapping)
{
    // Every defined status round trips, and an out-of-domain status is
    // refused rather than mapped (N07).
    const TofStatus defined[] = {
        TofStatus::Ok,                       TofStatus::InvalidInput,
        TofStatus::InvalidClockDomain,       TofStatus::ClockRatioMissing,
        TofStatus::ClockRatioInvalid,        TofStatus::ClockRatioNotValidAtTime,
        TofStatus::ClockRatioDomainMismatch, TofStatus::ClockRatioEpochMismatch,
        TofStatus::NegativeTof,              TofStatus::ZeroDenominator,
        TofStatus::Overflow,                 TofStatus::InvalidResult
    };
    for (const TofStatus s : defined) {
        BOOST_REQUIRE(tof_status_is_known(s));
        TofStatus back = TofStatus::Ok;
        BOOST_REQUIRE(tof_status_from_string(tof_status_to_string(s), back));
        BOOST_REQUIRE(back == s);
        BOOST_REQUIRE(tof_status_to_exchange_status(s) != ExchangeStatus::ConfigRejected);
    }
    const TofStatus bogus = static_cast<TofStatus>(250);
    BOOST_TEST(!tof_status_is_known(bogus));
    BOOST_TEST(std::string(tof_status_to_string(bogus)) == "invalid");
    // Fail-closed: an unknown status maps to an ERROR, never to Ok.
    BOOST_TEST(tof_status_to_exchange_status(bogus) == ExchangeStatus::InternalError);
    // Only Ok yields an Ok exchange status.
    BOOST_TEST(tof_status_to_exchange_status(TofStatus::Ok) == ExchangeStatus::Ok);
    BOOST_TEST(tof_status_to_exchange_status(TofStatus::NegativeTof) ==
               ExchangeStatus::NegativeTof);
    BOOST_TEST(exchange_status_to_string(ExchangeStatus::NegativeTof) ==
               std::string("negative_tof"));
    BOOST_TEST(exchange_status_family(ExchangeStatus::NegativeTof) == std::string("signal"));
}

BOOST_AUTO_TEST_CASE(math_result_is_projection_free_and_intervals_are_exact)
{
    // COM-2 / COM-3: the exact result is a rational and cannot be fed back as
    // an integer-nanosecond value; the formula entry points take admitted
    // intervals, so a bare Timestamp is not even accepted.
    static_assert(!has_nanos_member<TofRationalTicks>::value,
                  "TofRationalTicks must not expose an integer-nanosecond member");
    static_assert(has_nanos_member<Duration>::value,
                  "control: the detector must fire for a type that DOES have nanos()");
    static_assert(std::is_same<std::decay_t<decltype(std::declval<const TofResult&>().tof)>,
                               TofRationalTicks>::value,
                  "TofResult::tof must be the exact rational, not a double");

    static_assert(std::is_invocable_v<decltype(&compute_ss_tof),
                                      const AdmittedRangingInterval&,
                                      const AdmittedRangingInterval&, const ClockRatio&>,
                  "compute_ss_tof must take admitted intervals");
    static_assert(!std::is_invocable_v<decltype(&compute_ss_tof), const Timestamp&,
                                       const Timestamp&, const ClockRatio&>,
                  "a bare Timestamp must not be a formula input");
    static_assert(!std::is_convertible_v<Timestamp, AdmittedRangingInterval>,
                  "there must be no way to turn a Timestamp into an admitted interval");
    static_assert(!std::is_default_constructible_v<AdmittedRangingInterval>,
                  "an admitted interval can only come from the gate");

    BOOST_TEST(true); // the assertions above are the test
}

BOOST_AUTO_TEST_SUITE_END()
