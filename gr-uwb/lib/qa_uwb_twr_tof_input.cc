/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * QA for the TWR ranging-input admission layer (M0.1, review defect R6/P1).
 *
 * R6: two `UhdRxFirstIqSample + HardwareMeasured + no corrections` timestamps
 * still produce `timestamp_interval().status == Ok`.  That is a legitimate
 * general-purpose "time between two samples" tool -- but M0's report claimed
 * "裸设备时间戳无法产出协议 interval", and M1 is going to read an Ok status as
 * authorisation to compute a distance.  This QA pins the boundary that R6
 * asked for: a *validated ranging input* is a distinct type, produced only by
 * a gate that checks the real conditions of REQ-TIME-02 / 03 / 05 and
 * REQ-CAL-01.  See gr-uwb/include/gnuradio/uwb/uwb_twr_tof_input.h.
 *
 * Every expected value here is written out analytically.  Where the header
 * offers a predicate (e.g. `timestamp_required_corrections()`) the test
 * deliberately re-states the expected bit list itself, so a change to the
 * requirement cannot quietly change what the QA demands.
 *
 * Radio-free: no GNU Radio, no UHD, no device.  The 40-bit DW UUS grid is
 * used because its 15.645 ps tick makes every integer-nanosecond projection
 * lossy, which is exactly the condition the admitted value must hide.
 */

#include <boost/test/unit_test.hpp>

#include <gnuradio/uwb/uwb_twr_tof_input.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <ostream>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

// Boost.Test needs to print both operands of every BOOST_TEST expression, and
// these enums have no stream operator.  `print_log_value` is Boost's own
// documented customization point, so it is specialized here rather than
// bolting an `operator<<` onto the frozen headers.  Each specialization
// prints the SAME stable string the production code uses, so a failing
// assertion names the contract rather than a raw integer.
namespace boost {
namespace test_tools {
namespace tt_detail {

template <>
struct print_log_value<gr::uwb::twr::ExchangeStatus> {
    void operator()(std::ostream& os, gr::uwb::twr::ExchangeStatus const& s) const
    {
        os << gr::uwb::twr::exchange_status_to_string(s);
    }
};

template <>
struct print_log_value<gr::uwb::twr::TimeIntervalStatus> {
    void operator()(std::ostream& os, gr::uwb::twr::TimeIntervalStatus const& s) const
    {
        os << gr::uwb::twr::time_interval_status_to_string(s);
    }
};

template <>
struct print_log_value<gr::uwb::twr::TimestampMarker> {
    void operator()(std::ostream& os, gr::uwb::twr::TimestampMarker const& m) const
    {
        os << gr::uwb::twr::timestamp_marker_to_string(m);
    }
};

template <>
struct print_log_value<gr::uwb::twr::RangeAdmissionReason> {
    void operator()(std::ostream& os, gr::uwb::twr::RangeAdmissionReason const& r) const
    {
        os << gr::uwb::twr::range_admission_reason_to_string(r);
    }
};

template <>
struct print_log_value<gr::uwb::twr::RangeTimeSide> {
    void operator()(std::ostream& os, gr::uwb::twr::RangeTimeSide const& s) const
    {
        os << gr::uwb::twr::range_time_side_to_string(s);
    }
};

template <>
struct print_log_value<gr::uwb::twr::FirstPathDecision> {
    void operator()(std::ostream& os, gr::uwb::twr::FirstPathDecision const& d) const
    {
        os << gr::uwb::twr::first_path_decision_to_string(d);
    }
};

template <>
struct print_log_value<gr::uwb::twr::TimestampSource> {
    void operator()(std::ostream& os, gr::uwb::twr::TimestampSource const& s) const
    {
        os << gr::uwb::twr::timestamp_source_to_string(s);
    }
};

template <>
struct print_log_value<gr::uwb::twr::CalibrationResult> {
    void operator()(std::ostream& os, gr::uwb::twr::CalibrationResult const& r) const
    {
        os << gr::uwb::twr::calibration_result_to_string(r);
    }
};

} // namespace tt_detail
} // namespace test_tools
} // namespace boost


namespace {

using gr::uwb::twr::AdmittedRangingInterval;
using gr::uwb::twr::admit_ranging_interval;
using gr::uwb::twr::admit_range_capable_time;
using gr::uwb::twr::CalibrationApplication;
using gr::uwb::twr::CalibrationResult;
using gr::uwb::twr::CalibrationStamp;
using gr::uwb::twr::ExchangeStatus;
using gr::uwb::twr::FirstPathDecision;
using gr::uwb::twr::FirstPathQuality;
using gr::uwb::twr::Duration;
using gr::uwb::twr::kRangeAdmissionReasonMax;
using gr::uwb::twr::RangeAdmission;
using gr::uwb::twr::RangeAdmissionContext;
using gr::uwb::twr::RangeAdmissionReason;
using gr::uwb::twr::range_admission_reason_from_string;
using gr::uwb::twr::range_admission_reason_to_exchange_status;
using gr::uwb::twr::range_admission_reason_to_string;
using gr::uwb::twr::RangeCapableTime;
using gr::uwb::twr::RangingIntervalAdmission;
using gr::uwb::twr::RangeTimeSide;
using gr::uwb::twr::RelativeTickInterval;
using gr::uwb::twr::TimeInterval;
using gr::uwb::twr::TimeIntervalStatus;
using gr::uwb::twr::timestamp_interval;
using gr::uwb::twr::Timestamp;
using gr::uwb::twr::TimestampMarker;
using gr::uwb::twr::TimestampSource;
using gr::uwb::twr::tx_outcome_is_known;
using gr::uwb::twr::TxOutcome;
using gr::uwb::twr::TxSendEvidence;

// ---------------------------------------------------------------------------
// Expected correction masks, written out by hand from the requirement text
// (REQ-TIME-02 / 03) rather than read back from the header.
// ---------------------------------------------------------------------------
constexpr uint32_t kBitRxSampleToFirstPath = 1u << 0;
constexpr uint32_t kBitWindowCrop = 1u << 1;
constexpr uint32_t kBitSampleRateConversion = 1u << 2;
constexpr uint32_t kBitFirstPathFraction = 1u << 3;
constexpr uint32_t kBitWaveformGeometry = 1u << 4;
constexpr uint32_t kBitRmarkerOffset = 1u << 5;
constexpr uint32_t kBitTxCommandToAir = 1u << 6;
constexpr uint32_t kBitDelayedTxQuantization = 1u << 7;
constexpr uint32_t kBitAntennaPlane = 1u << 8;
constexpr uint32_t kBitFirstPathQualityGate = 1u << 9;

const std::vector<uint32_t>& rx_chain_bits()
{
    static const std::vector<uint32_t> v{kBitRxSampleToFirstPath, kBitWindowCrop,
                                         kBitSampleRateConversion, kBitFirstPathFraction,
                                         kBitWaveformGeometry, kBitFirstPathQualityGate};
    return v;
}

const std::vector<uint32_t>& rmarker_rx_bits()
{
    static const std::vector<uint32_t> v{
        kBitRxSampleToFirstPath, kBitWindowCrop, kBitSampleRateConversion,
        kBitFirstPathFraction,   kBitWaveformGeometry, kBitFirstPathQualityGate,
        kBitRmarkerOffset};
    return v;
}

const std::vector<uint32_t>& rmarker_tx_bits()
{
    static const std::vector<uint32_t> v{kBitWaveformGeometry, kBitTxCommandToAir,
                                         kBitDelayedTxQuantization};
    return v;
}

const std::vector<uint32_t>& antenna_plane_bits()
{
    static const std::vector<uint32_t> v{
        kBitRxSampleToFirstPath, kBitWindowCrop, kBitSampleRateConversion,
        kBitFirstPathFraction,   kBitWaveformGeometry, kBitFirstPathQualityGate,
        kBitRmarkerOffset,       kBitAntennaPlane};
    return v;
}

uint32_t mask_of(const std::vector<uint32_t>& bits)
{
    uint32_t m = 0u;
    for (uint32_t b : bits)
        m |= b;
    return m;
}

// ---------------------------------------------------------------------------
// Structural detectors.  A detection idiom that never fires is worthless, so
// every negative structural claim is paired with a positive control below.
// ---------------------------------------------------------------------------
template <typename T, typename = void>
struct has_nanos_member : std::false_type {};
template <typename T>
struct has_nanos_member<T, std::void_t<decltype(std::declval<const T&>().nanos())>>
    : std::true_type {};

template <typename T, typename = void>
struct has_duration_member : std::false_type {};
template <typename T>
struct has_duration_member<T, std::void_t<decltype(std::declval<const T&>().duration)>>
    : std::true_type {};

template <typename T, typename = void>
struct has_seconds_member : std::false_type {};
template <typename T>
struct has_seconds_member<T, std::void_t<decltype(std::declval<const T&>().seconds())>>
    : std::true_type {};

// ---------------------------------------------------------------------------
// Fixtures
// ---------------------------------------------------------------------------

// DW UUS grid: 1/(499.2e6*128) s per tick ~ 15.645 ps, 40-bit wire field.
bool make_uus_domain(gr::uwb::twr::ClockDomain& out,
                     const std::string& name = "dw1000_sn0001_ch5_uus",
                     uint64_t epoch = 7u,
                     uint32_t bits = 40u)
{
    return gr::uwb::twr::ClockDomain::make(name, 499.2e6 * 128.0, epoch, bits, out);
}

bool make_raw_iq_sample(int64_t ticks,
                        const gr::uwb::twr::ClockDomain& domain,
                        Timestamp& out)
{
    // EXACTLY the reviewer's reproducer inputs.
    return Timestamp::from_ticks(ticks, domain, TimestampMarker::UhdRxFirstIqSample,
                                 TimestampSource::HardwareMeasured,
                                 gr::uwb::twr::kCorrectionNone, out);
}

bool make_mapped(int64_t ticks,
                 TimestampMarker marker,
                 uint32_t corrections,
                 const gr::uwb::twr::ClockDomain& domain,
                 Timestamp& out)
{
    Timestamp ts;
    if (!Timestamp::from_ticks(ticks, domain, marker, TimestampSource::HardwareMeasured,
                               corrections, ts))
        return false;
    if (gr::uwb::twr::apply_calibration_ticks(ts, "cal-dw1000-sn0001-ch5-uus-r1", 0, 0, 0u,
                                               0u) != CalibrationResult::Applied)
        return false;
    out = ts;
    return true;
}

// A calibration that is in force: one Applied record, current epoch, a
// non-degenerate window that contains reference_ticks == 1500.
CalibrationStamp good_calibration(uint64_t epoch = 7u,
                                  const std::string& id = "cal-dw1000-sn0001-ch5-uus-r1")
{
    CalibrationStamp c;
    c.id = id;
    c.calibrated_epoch = epoch;
    c.valid_from_ticks = 1000;
    c.valid_until_ticks = 2000;
    CalibrationApplication a;
    a.calibration_id = id;
    a.result = CalibrationResult::Applied;
    c.applications.push_back(a);
    return c;
}

} // namespace

// ===========================================================================
// R6 -- the raw IQ sample pair that produced an Ok interval
// ===========================================================================

BOOST_AUTO_TEST_CASE(tof_r6_raw_iq_sample_pair_is_refused_as_ranging_input)
{
    gr::uwb::twr::ClockDomain d;
    BOOST_REQUIRE(make_uus_domain(d));

    Timestamp a;
    Timestamp b;
    BOOST_REQUIRE(make_raw_iq_sample(1000, d, a));
    BOOST_REQUIRE(make_raw_iq_sample(2000, d, b));

    // (1) The general-purpose tool still works, and that is fine: "time
    //     between two sample times" is a real question.
    const TimeInterval ti = timestamp_interval(b, a);
    BOOST_TEST(ti.status == TimeIntervalStatus::Ok);
    BOOST_TEST(ti.ticks == 1000);

    // (2) It is NOT a ranging input.  The marker is not range-capable, so the
    //     pair is refused and M1 has nothing to read.
    const CalibrationStamp cal = good_calibration();
    RangeAdmissionContext ctx;
    ctx.calibration = &cal;
    ctx.reference_ticks = 1500;
    ctx.reference_ticks_recorded = true;
    ctx.rx_first_path = FirstPathQuality::passed(18.5, 9.0, 0.82);

    const RangingIntervalAdmission r = admit_ranging_interval(b, a, ctx);
    BOOST_TEST(!r.admitted);
    BOOST_TEST(!r.value.has_value());
    BOOST_TEST(r.reason == RangeAdmissionReason::NotRangeCapableMarker);
    BOOST_TEST(r.side == RangeTimeSide::Later);
    BOOST_TEST(r.marker == TimestampMarker::UhdRxFirstIqSample);
    BOOST_TEST(r.status == ExchangeStatus::InvalidTimeDomain);

    // (3) Each side is refused on its own too, with the same reason.
    const RangeAdmission ra = admit_range_capable_time(a, ctx);
    BOOST_TEST(!ra.admitted);
    BOOST_TEST(!ra.value.has_value());
    BOOST_TEST(ra.reason == RangeAdmissionReason::NotRangeCapableMarker);
    BOOST_TEST(ra.side == RangeTimeSide::Subject);

    // (4) Setting the correction bits is NOT an acceptable fix: a raw sample
    //     time stays non-range-capable no matter which bits are set, because
    //     the RMARKER <-> waveform mapping was never performed.  A timestamp
    //     merely *claiming* the RX chain bits must still be refused.
    Timestamp forged;
    BOOST_REQUIRE(Timestamp::from_ticks(1000, d, TimestampMarker::UhdRxFirstIqSample,
                                        TimestampSource::HardwareMeasured,
                                        mask_of(antenna_plane_bits()), forged));
    const RangeAdmission rf = admit_range_capable_time(forged, ctx);
    BOOST_TEST(!rf.admitted);
    BOOST_TEST(rf.reason == RangeAdmissionReason::NotRangeCapableMarker);
    BOOST_TEST(!rf.value.has_value());
}

BOOST_AUTO_TEST_CASE(tof_rmarker_without_the_rmarker_offset_is_refused)
{
    // A corrected RX chain whose RMARKER <-> waveform mapping was never
    // applied: the corrections for `RmarkerRx` are one bit short, and the
    // refusal must NAME that bit rather than report a generic failure.
    gr::uwb::twr::ClockDomain d;
    BOOST_REQUIRE(make_uus_domain(d));

    const uint32_t missing_one = kBitRmarkerOffset;
    const uint32_t applied = mask_of(rmarker_rx_bits()) & ~missing_one;
    BOOST_TEST(applied == (mask_of(rmarker_rx_bits()) ^ missing_one));

    Timestamp ts;
    BOOST_REQUIRE(make_mapped(1000, TimestampMarker::RmarkerRx, applied, d, ts));

    const CalibrationStamp cal = good_calibration();
    RangeAdmissionContext ctx;
    ctx.calibration = &cal;
    ctx.reference_ticks = 1500;
    ctx.reference_ticks_recorded = true;
    ctx.rx_first_path = FirstPathQuality::passed(18.5, 9.0, 0.82);

    const RangeAdmission r = admit_range_capable_time(ts, ctx);
    BOOST_TEST(!r.admitted);
    BOOST_TEST(!r.value.has_value());
    BOOST_TEST(r.reason == RangeAdmissionReason::MissingCorrection);
    BOOST_TEST(r.missing_correction == missing_one);
    // The detail string must name the missing stage, not just say "no".
    BOOST_TEST(r.detail.find("rmarker_offset") != std::string::npos);
    BOOST_TEST(r.status == ExchangeStatus::InvalidTimeDomain);
}

// ===========================================================================
// The admitted path
// ===========================================================================

BOOST_AUTO_TEST_CASE(tof_fully_evidenced_rmarker_pair_is_admitted)
{
    gr::uwb::twr::ClockDomain d;
    BOOST_REQUIRE(make_uus_domain(d));

    Timestamp rx;
    Timestamp tx;
    BOOST_REQUIRE(
        make_mapped(9000, TimestampMarker::RmarkerRx, mask_of(rmarker_rx_bits()), d, rx));
    BOOST_REQUIRE(
        make_mapped(5000, TimestampMarker::RmarkerTx, mask_of(rmarker_tx_bits()), d, tx));

    const CalibrationStamp cal = good_calibration();
    RangeAdmissionContext ctx;
    ctx.calibration = &cal;
    ctx.reference_ticks = 1500;
    ctx.reference_ticks_recorded = true;
    ctx.rx_first_path = FirstPathQuality::passed(18.5, 9.0, 0.82);

    // Individual admission, RX side: needs the first-path decision.
    const RangeAdmission a = admit_range_capable_time(rx, ctx);
    BOOST_TEST(a.admitted);
    BOOST_TEST(a.reason == RangeAdmissionReason::Admitted);
    BOOST_TEST(a.status == ExchangeStatus::Ok);
    BOOST_REQUIRE(a.value.has_value());
    BOOST_TEST(a.value->marker() == TimestampMarker::RmarkerRx);
    BOOST_TEST(a.value->source() == TimestampSource::HardwareMeasured);
    BOOST_TEST(a.value->calibration_id() == cal.id);
    BOOST_TEST(a.value->first_path_quality().has_passed());

    // Individual admission, TX side: REQ-TIME-05 is a *receive* first-path
    // gate, so a TX RMARKER must not need one.  A caller that never ran a
    // first-path estimator at all still gets a valid TX time.
    RangeAdmissionContext tx_ctx = ctx;
    tx_ctx.rx_first_path = FirstPathQuality::not_recorded();
    const RangeAdmission b = admit_range_capable_time(tx, tx_ctx);
    BOOST_TEST(b.admitted);
    BOOST_REQUIRE(b.value.has_value());
    BOOST_TEST(!b.value->first_path_quality().has_passed());

    // The pair.  The RX side is in it, so the pair context is the one that
    // carries the first-path decision -- a transmit-only context would be
    // refused, which is the point of condition 6.
    const RangingIntervalAdmission r = admit_ranging_interval(rx, tx, ctx);
    BOOST_TEST(r.reason == RangeAdmissionReason::Admitted);
    BOOST_TEST(r.status == ExchangeStatus::Ok);
    BOOST_REQUIRE(r.value.has_value());
    BOOST_TEST(r.value->interval().ticks == 4000);
    BOOST_TEST(r.value->interval().is_whole_ticks());
    BOOST_TEST(r.value->later().marker() == TimestampMarker::RmarkerRx);
    BOOST_TEST(r.value->earlier().marker() == TimestampMarker::RmarkerTx);
}

BOOST_AUTO_TEST_CASE(tof_antenna_plane_marker_is_range_capable)
{
    // REQ-TIME-02 puts the calibrated antenna reference plane in the same
    // family as the protocol RMARKER: both denote an on-air instant.
    gr::uwb::twr::ClockDomain d;
    BOOST_REQUIRE(make_uus_domain(d));

    Timestamp plane;
    BOOST_REQUIRE(make_mapped(7000, TimestampMarker::AntennaPlane,
                              mask_of(antenna_plane_bits()), d, plane));

    const CalibrationStamp cal = good_calibration();
    RangeAdmissionContext ctx;
    ctx.calibration = &cal;
    ctx.reference_ticks = 1500;
    ctx.reference_ticks_recorded = true;
    ctx.rx_first_path = FirstPathQuality::passed(20.0, 11.0, 0.9);

    const RangeAdmission a = admit_range_capable_time(plane, ctx);
    BOOST_TEST(a.admitted);
    BOOST_REQUIRE(a.value.has_value());
    BOOST_TEST(a.value->marker() == TimestampMarker::AntennaPlane);
}

BOOST_AUTO_TEST_CASE(tof_sub_mid_preamble_and_sfd_are_not_range_capable)
{
    // Those are waveform-geometry coordinates, not protocol instants.  Even
    // with the full RX chain they must not become ranging inputs.
    gr::uwb::twr::ClockDomain d;
    BOOST_REQUIRE(make_uus_domain(d));
    const CalibrationStamp cal = good_calibration();
    RangeAdmissionContext ctx;
    ctx.calibration = &cal;
    ctx.reference_ticks = 1500;
    ctx.reference_ticks_recorded = true;
    ctx.rx_first_path = FirstPathQuality::passed(20.0, 11.0, 0.9);

    const TimestampMarker mid[] = {TimestampMarker::PreambleStart, TimestampMarker::SfdStart,
                                   TimestampMarker::PhrStart};
    for (TimestampMarker m : mid) {
        Timestamp ts;
        BOOST_REQUIRE(make_mapped(7000, m, mask_of(rx_chain_bits()), d, ts));
        const RangeAdmission a = admit_range_capable_time(ts, ctx);
        BOOST_TEST(!a.admitted);
        BOOST_TEST(a.reason == RangeAdmissionReason::NotRangeCapableMarker);
        BOOST_TEST(a.marker == m);
    }
}

// ===========================================================================
// REQ-TIME-03 -- provenance
// ===========================================================================

BOOST_AUTO_TEST_CASE(tof_scheduled_calibrated_source_is_permitted_only_with_send_evidence)
{
    // REQ-TIME-03 permits a deterministic timed TX ("可通过确定性时序加标定取得
    // 合格 TX timestamp") as long as it is labelled scheduled_calibrated and
    // carries the evidence that the burst happened; it forbids passing a
    // SCHEDULE off as a measurement.  This test used to assert the opposite --
    // that a scheduled TX must be refused -- which would have blocked the
    // planned X410 timed-TX route and pushed the pipeline toward mislabelling
    // its own timestamps as HardwareMeasured.
    gr::uwb::twr::ClockDomain d;
    BOOST_REQUIRE(make_uus_domain(d));

    // (1) A scheduled RECEIVE time is still refused: it is a PREDICTION.
    Timestamp ts;
    BOOST_REQUIRE(Timestamp::from_ticks(9000, d, TimestampMarker::RmarkerRx,
                                        TimestampSource::ScheduledCalibrated,
                                        mask_of(rmarker_rx_bits()), ts));
    BOOST_TEST(gr::uwb::twr::apply_calibration_ticks(
                   ts, "cal-dw1000-sn0001-ch5-uus-r1", 0, 0, 0u, 0u) ==
               CalibrationResult::Applied);

    const CalibrationStamp cal = good_calibration();
    RangeAdmissionContext ctx;
    ctx.calibration = &cal;
    ctx.reference_ticks = 1500;
    ctx.reference_ticks_recorded = true;
    ctx.rx_first_path = FirstPathQuality::passed(18.5, 9.0, 0.82);

    const RangeAdmission r = admit_range_capable_time(ts, ctx);
    BOOST_TEST(!r.admitted);
    BOOST_TEST(r.reason == RangeAdmissionReason::NotMeasured);
    BOOST_TEST(r.status == ExchangeStatus::InvalidTimeDomain);
    BOOST_TEST(!r.value.has_value());

    // (2) A scheduled TX RMARKER WITHOUT send evidence is refused, and the
    //     refusal names the missing evidence rather than the source.
    Timestamp txs;
    BOOST_REQUIRE(Timestamp::from_ticks(5000, d, TimestampMarker::RmarkerTx,
                                        TimestampSource::ScheduledCalibrated,
                                        mask_of(rmarker_tx_bits()), txs));
    BOOST_TEST(gr::uwb::twr::apply_calibration_ticks(
                   txs, "cal-dw1000-sn0001-ch5-uus-r1", 0, 0, 0u, 0u) ==
               CalibrationResult::Applied);

    const RangeAdmission no_ev = admit_range_capable_time(txs, ctx);
    BOOST_TEST(!no_ev.admitted);
    BOOST_TEST(no_ev.reason == RangeAdmissionReason::ScheduledTxEvidenceMissing);
    BOOST_TEST(no_ev.status == ExchangeStatus::InvalidTimeDomain);

    // (3) `send_accepted` ALONE is not enough: the outcome must be known to be
    //     Completed.  This is the REQ-TIME-03 sentence "send 返回成功或 burst
    //     ACK 不等价于芯片提供的精细空口 timestamp".
    TxSendEvidence send_only;
    send_only.command_time_recorded = true;
    send_only.quantised_instant_recorded = true;
    send_only.marker_offset_recorded = true;
    send_only.calibrated_air_time_recorded = true;
    send_only.send_accepted = true;
    send_only.outcome = TxOutcome::Unknown;
    ctx.tx_evidence = &send_only;
    const RangeAdmission only_send = admit_range_capable_time(txs, ctx);
    BOOST_TEST(!only_send.admitted);
    BOOST_TEST(only_send.reason == RangeAdmissionReason::ScheduledTxEvidenceMissing);
    BOOST_TEST(only_send.detail.find("send_accepted") != std::string::npos);

    // (4) An incomplete PLAN is refused, and names the record that is missing.
    TxSendEvidence no_plan;
    no_plan.outcome = TxOutcome::Completed;
    no_plan.send_accepted = true;
    ctx.tx_evidence = &no_plan;
    const RangeAdmission plan = admit_range_capable_time(txs, ctx);
    BOOST_TEST(!plan.admitted);
    BOOST_TEST(plan.reason == RangeAdmissionReason::ScheduledTxEvidenceMissing);
    BOOST_TEST(plan.detail.find("command_time") != std::string::npos);

    // (5) Late / underflow / cancelled each invalidate the exchange, with their
    //     OWN terminal status -- a soak report must be able to tell them apart
    //     without parsing prose (REQ-ERR-01).
    const TxOutcome bad[] = {TxOutcome::Late, TxOutcome::Underflow,
                             TxOutcome::Cancelled};
    const RangeAdmissionReason bad_reason[] = {
        RangeAdmissionReason::TxOutcomeLate,
        RangeAdmissionReason::TxOutcomeUnderflow,
        RangeAdmissionReason::TxOutcomeCancelled};
    const ExchangeStatus bad_status[] = {ExchangeStatus::TxLate,
                                         ExchangeStatus::TxUnderflow,
                                         ExchangeStatus::Cancelled};
    for (size_t i = 0; i < sizeof(bad) / sizeof(bad[0]); ++i) {
        TxSendEvidence ev;
        ev.command_time_recorded = true;
        ev.quantised_instant_recorded = true;
        ev.marker_offset_recorded = true;
        ev.calibrated_air_time_recorded = true;
        ev.send_accepted = true;
        ev.outcome = bad[i];
        ctx.tx_evidence = &ev;
        const RangeAdmission rb = admit_range_capable_time(txs, ctx);
        BOOST_TEST(!rb.admitted);
        BOOST_TEST(rb.reason == bad_reason[i]);
        BOOST_TEST(rb.status == bad_status[i]);
        BOOST_TEST(!rb.value.has_value());
    }

    // (6) A COMPLETE, completed plan is admitted -- and the admitted value is
    //     still labelled ScheduledCalibrated, never promoted to a hardware
    //     measurement.  Reaching the ranging math did not relabel the source.
    TxSendEvidence good;
    good.command_time_recorded = true;
    good.quantised_instant_recorded = true;
    good.marker_offset_recorded = true;
    good.calibrated_air_time_recorded = true;
    good.send_accepted = true;
    good.outcome = TxOutcome::Completed;
    ctx.tx_evidence = &good;
    const RangeAdmission ok = admit_range_capable_time(txs, ctx);
    BOOST_REQUIRE_MESSAGE(ok.admitted, ok.detail);
    BOOST_TEST(ok.reason == RangeAdmissionReason::Admitted);
    BOOST_TEST(ok.value.has_value());
    BOOST_TEST(ok.value->source() == TimestampSource::ScheduledCalibrated);
    BOOST_TEST(ok.value->source() != TimestampSource::HardwareMeasured);
}


// ===========================================================================
// N07: an out-of-domain TxOutcome must not reach the admitted path
// ===========================================================================
//
// The admission switch covers all five enumerators with NO `default`, so a
// value the enum does not have -- `static_cast<TxOutcome>(5)`, which a caller,
// a deserialiser or an async-event mapping can produce -- matched no case, fell
// THROUGH the switch and continued to the admitted path.  That is fail-open in
// the gate that decides whether a distance may be published, and it is the
// exact opposite of what REQ-TIME-03 requires: only a COMPLETED burst may
// publish a ranging instant.
BOOST_AUTO_TEST_CASE(tof_out_of_domain_tx_outcome_is_refused)
{
    gr::uwb::twr::ClockDomain d;
    BOOST_REQUIRE(make_uus_domain(d));
    Timestamp txs;
    BOOST_REQUIRE(Timestamp::from_ticks(5000, d, TimestampMarker::RmarkerTx,
                                        TimestampSource::ScheduledCalibrated,
                                        mask_of(rmarker_tx_bits()), txs));
    BOOST_TEST(gr::uwb::twr::apply_calibration_ticks(
                   txs, "cal-dw1000-sn0001-ch5-uus-r1", 0, 0, 0u, 0u) ==
               CalibrationResult::Applied);

    const CalibrationStamp cal = good_calibration();
    RangeAdmissionContext ctx;
    ctx.calibration = &cal;
    ctx.reference_ticks = 1500;
    ctx.reference_ticks_recorded = true;

    // The domain test itself, so a regression is attributed to the right half.
    BOOST_TEST(tx_outcome_is_known(TxOutcome::Completed));
    BOOST_TEST(tx_outcome_is_known(TxOutcome::Unknown));
    BOOST_TEST(!tx_outcome_is_known(static_cast<TxOutcome>(5)));
    BOOST_TEST(!tx_outcome_is_known(static_cast<TxOutcome>(255)));

    // A complete, otherwise-admissible plan: the ONLY thing wrong is the
    // outcome, so a rejection here is the outcome's doing.
    for (const int v : { 5, 6, 99, 255 }) {
        TxSendEvidence ev;
        ev.command_time_recorded = true;
        ev.quantised_instant_recorded = true;
        ev.marker_offset_recorded = true;
        ev.calibrated_air_time_recorded = true;
        ev.send_accepted = true;
        ev.outcome = static_cast<TxOutcome>(v);
        ctx.tx_evidence = &ev;
        const RangeAdmission r = admit_range_capable_time(txs, ctx);
        BOOST_TEST(!r.admitted);
        BOOST_TEST(r.reason == RangeAdmissionReason::ScheduledTxEvidenceMissing);
        BOOST_TEST(r.status == ExchangeStatus::InvalidTimeDomain);
        BOOST_TEST(!r.value.has_value());
    }

    // Control: Completed with the same plan IS admitted, so the loop above is
    // failing for the outcome and not because the fixture never admits.
    {
        TxSendEvidence ev;
        ev.command_time_recorded = true;
        ev.quantised_instant_recorded = true;
        ev.marker_offset_recorded = true;
        ev.calibrated_air_time_recorded = true;
        ev.send_accepted = true;
        ev.outcome = TxOutcome::Completed;
        ctx.tx_evidence = &ev;
        const RangeAdmission ok = admit_range_capable_time(txs, ctx);
        BOOST_REQUIRE_MESSAGE(ok.admitted, ok.detail);
    }
}

BOOST_AUTO_TEST_CASE(tof_estimated_and_reconstructed_sources_are_refused)
{
    gr::uwb::twr::ClockDomain d;
    BOOST_REQUIRE(make_uus_domain(d));
    const CalibrationStamp cal = good_calibration();
    RangeAdmissionContext ctx;
    ctx.calibration = &cal;
    ctx.reference_ticks = 1500;
    ctx.reference_ticks_recorded = true;
    ctx.rx_first_path = FirstPathQuality::passed(18.5, 9.0, 0.82);

    const TimestampSource bad[] = {TimestampSource::Estimated, TimestampSource::Reconstructed};
    for (TimestampSource s : bad) {
        Timestamp ts;
        BOOST_REQUIRE(Timestamp::from_ticks(9000, d, TimestampMarker::RmarkerRx, s,
                                            mask_of(rmarker_rx_bits()), ts));
        BOOST_TEST(gr::uwb::twr::apply_calibration_ticks(
                       ts, "cal-dw1000-sn0001-ch5-uus-r1", 0, 0, 0u, 0u) ==
                   CalibrationResult::Applied);
        const RangeAdmission r = admit_range_capable_time(ts, ctx);
        BOOST_TEST(!r.admitted);
        BOOST_TEST(r.reason == RangeAdmissionReason::NotMeasured);
    }
}

// ===========================================================================
// REQ-TIME-02 -- every required correction, one at a time
// ===========================================================================

BOOST_AUTO_TEST_CASE(tof_every_missing_rx_correction_is_named)
{
    gr::uwb::twr::ClockDomain d;
    BOOST_REQUIRE(make_uus_domain(d));
    const CalibrationStamp cal = good_calibration();
    RangeAdmissionContext ctx;
    ctx.calibration = &cal;
    ctx.reference_ticks = 1500;
    ctx.reference_ticks_recorded = true;
    ctx.rx_first_path = FirstPathQuality::passed(18.5, 9.0, 0.82);

    const uint32_t full = mask_of(rmarker_rx_bits());
    for (uint32_t bit : rmarker_rx_bits()) {
        Timestamp ts;
        BOOST_REQUIRE(
            make_mapped(9000, TimestampMarker::RmarkerRx, full & ~bit, d, ts));
        const RangeAdmission r = admit_range_capable_time(ts, ctx);
        BOOST_TEST(!r.admitted);
        BOOST_TEST(r.reason == RangeAdmissionReason::MissingCorrection);
        BOOST_TEST(r.missing_correction == bit);
        BOOST_TEST(r.detail.find(gr::uwb::twr::timestamp_correction_name(bit)) !=
                   std::string::npos);
        BOOST_TEST(!r.value.has_value());
    }
}

BOOST_AUTO_TEST_CASE(tof_every_missing_tx_correction_is_named)
{
    gr::uwb::twr::ClockDomain d;
    BOOST_REQUIRE(make_uus_domain(d));
    const CalibrationStamp cal = good_calibration();
    RangeAdmissionContext ctx;
    ctx.calibration = &cal;
    ctx.reference_ticks = 1500;
    ctx.reference_ticks_recorded = true;

    const uint32_t full = mask_of(rmarker_tx_bits());
    for (uint32_t bit : rmarker_tx_bits()) {
        Timestamp ts;
        BOOST_REQUIRE(
            make_mapped(5000, TimestampMarker::RmarkerTx, full & ~bit, d, ts));
        const RangeAdmission r = admit_range_capable_time(ts, ctx);
        BOOST_TEST(!r.admitted);
        BOOST_TEST(r.reason == RangeAdmissionReason::MissingCorrection);
        BOOST_TEST(r.missing_correction == bit);
    }
}

BOOST_AUTO_TEST_CASE(tof_every_missing_antenna_plane_correction_is_named)
{
    gr::uwb::twr::ClockDomain d;
    BOOST_REQUIRE(make_uus_domain(d));
    const CalibrationStamp cal = good_calibration();
    RangeAdmissionContext ctx;
    ctx.calibration = &cal;
    ctx.reference_ticks = 1500;
    ctx.reference_ticks_recorded = true;
    ctx.rx_first_path = FirstPathQuality::passed(20.0, 11.0, 0.9);

    const uint32_t full = mask_of(antenna_plane_bits());
    for (uint32_t bit : antenna_plane_bits()) {
        Timestamp ts;
        BOOST_REQUIRE(
            make_mapped(7000, TimestampMarker::AntennaPlane, full & ~bit, d, ts));
        const RangeAdmission r = admit_range_capable_time(ts, ctx);
        BOOST_TEST(!r.admitted);
        BOOST_TEST(r.reason == RangeAdmissionReason::MissingCorrection);
        BOOST_TEST(r.missing_correction == bit);
    }
}

// ===========================================================================
// REQ-CAL-01 -- present, once, current
// ===========================================================================

BOOST_AUTO_TEST_CASE(tof_missing_calibration_is_refused)
{
    gr::uwb::twr::ClockDomain d;
    BOOST_REQUIRE(make_uus_domain(d));

    // Fully corrected and measured, but never calibrated.
    Timestamp ts;
    BOOST_REQUIRE(Timestamp::from_ticks(9000, d, TimestampMarker::RmarkerRx,
                                        TimestampSource::HardwareMeasured,
                                        mask_of(rmarker_rx_bits()), ts));
    const CalibrationStamp cal = good_calibration();
    RangeAdmissionContext ctx;
    ctx.calibration = &cal;
    ctx.reference_ticks = 1500;
    ctx.reference_ticks_recorded = true;
    ctx.rx_first_path = FirstPathQuality::passed(18.5, 9.0, 0.82);

    const RangeAdmission r = admit_range_capable_time(ts, ctx);
    BOOST_TEST(!r.admitted);
    BOOST_TEST(r.reason == RangeAdmissionReason::NoCalibration);
    BOOST_TEST(r.status == ExchangeStatus::CalibrationMissing);
    BOOST_TEST(!r.value.has_value());
}

BOOST_AUTO_TEST_CASE(tof_absent_or_empty_calibration_context_is_refused)
{
    gr::uwb::twr::ClockDomain d;
    BOOST_REQUIRE(make_uus_domain(d));
    Timestamp ts;
    BOOST_REQUIRE(make_mapped(9000, TimestampMarker::RmarkerRx, mask_of(rmarker_rx_bits()),
                              d, ts));

    // No calibration set supplied at all.
    RangeAdmissionContext none;
    none.reference_ticks = 1500;
    none.reference_ticks_recorded = true;
    none.rx_first_path = FirstPathQuality::passed(18.5, 9.0, 0.82);
    const RangeAdmission r0 = admit_range_capable_time(ts, none);
    BOOST_TEST(!r0.admitted);
    BOOST_TEST(r0.reason == RangeAdmissionReason::NoCalibration);

    // A calibration with an empty id is not a calibration.
    CalibrationStamp empty = good_calibration();
    empty.id.clear();
    empty.applications.clear();
    RangeAdmissionContext ctx;
    ctx.calibration = &empty;
    ctx.reference_ticks = 1500;
    ctx.reference_ticks_recorded = true;
    ctx.rx_first_path = FirstPathQuality::passed(18.5, 9.0, 0.82);
    const RangeAdmission r1 = admit_range_capable_time(ts, ctx);
    BOOST_TEST(!r1.admitted);
    BOOST_TEST(r1.reason == RangeAdmissionReason::NoCalibration);
    BOOST_TEST(r1.status == ExchangeStatus::CalibrationMissing);

    // A reference instant that was never recorded cannot be used to decide
    // the calibration is current: fail closed, do not assume tick 0 is fine.
    const CalibrationStamp cal = good_calibration();
    RangeAdmissionContext no_ref;
    no_ref.calibration = &cal;
    no_ref.rx_first_path = FirstPathQuality::passed(18.5, 9.0, 0.82);
    const RangeAdmission r2 = admit_range_capable_time(ts, no_ref);
    BOOST_TEST(!r2.admitted);
    BOOST_TEST(r2.reason == RangeAdmissionReason::NoCalibration);
}

BOOST_AUTO_TEST_CASE(tof_calibration_applied_twice_is_refused)
{
    gr::uwb::twr::ClockDomain d;
    BOOST_REQUIRE(make_uus_domain(d));
    Timestamp ts;
    BOOST_REQUIRE(make_mapped(9000, TimestampMarker::RmarkerRx, mask_of(rmarker_rx_bits()),
                              d, ts));
    const std::string id = "cal-dw1000-sn0001-ch5-uus-r1";

    // (a) The stage was run twice: the evidence carries two records and the
    //     second one is the header's own AlreadyApplied refusal.
    {
        CalibrationStamp cal = good_calibration();
        CalibrationApplication second;
        second.calibration_id = id;
        second.result = CalibrationResult::AlreadyApplied;
        cal.applications.push_back(second);

        RangeAdmissionContext ctx;
        ctx.calibration = &cal;
        ctx.reference_ticks = 1500;
        ctx.reference_ticks_recorded = true;
        ctx.rx_first_path = FirstPathQuality::passed(18.5, 9.0, 0.82);
        const RangeAdmission r = admit_range_capable_time(ts, ctx);
        BOOST_TEST(!r.admitted);
        BOOST_TEST(r.reason == RangeAdmissionReason::CalibrationAppliedTwice);
        BOOST_TEST(r.status == ExchangeStatus::CalibrationMissing);
        BOOST_TEST(!r.value.has_value());
    }

    // (b) A single record that reports AlreadyApplied means the only attempt
    //     was a repeat: nothing was applied to this timestamp by that stage.
    {
        CalibrationStamp cal = good_calibration();
        cal.applications.clear();
        CalibrationApplication only;
        only.calibration_id = id;
        only.result = CalibrationResult::AlreadyApplied;
        cal.applications.push_back(only);

        RangeAdmissionContext ctx;
        ctx.calibration = &cal;
        ctx.reference_ticks = 1500;
        ctx.reference_ticks_recorded = true;
        ctx.rx_first_path = FirstPathQuality::passed(18.5, 9.0, 0.82);
        const RangeAdmission r = admit_range_capable_time(ts, ctx);
        BOOST_TEST(!r.admitted);
        BOOST_TEST(r.reason == RangeAdmissionReason::CalibrationAppliedTwice);
    }

    // (c) Two different calibrations claimed: also refused, as a mismatch.
    {
        CalibrationStamp cal = good_calibration();
        CalibrationApplication other;
        other.calibration_id = "cal-dw1000-sn0001-ch5-uus-r2";
        other.result = CalibrationResult::Applied;
        cal.applications.push_back(other);

        RangeAdmissionContext ctx;
        ctx.calibration = &cal;
        ctx.reference_ticks = 1500;
        ctx.reference_ticks_recorded = true;
        ctx.rx_first_path = FirstPathQuality::passed(18.5, 9.0, 0.82);
        const RangeAdmission r = admit_range_capable_time(ts, ctx);
        BOOST_TEST(!r.admitted);
        BOOST_TEST(r.reason == RangeAdmissionReason::CalibrationAppliedTwice);
    }
}

BOOST_AUTO_TEST_CASE(tof_calibration_id_mismatch_is_refused)
{
    gr::uwb::twr::ClockDomain d;
    BOOST_REQUIRE(make_uus_domain(d));
    Timestamp ts;
    BOOST_REQUIRE(make_mapped(9000, TimestampMarker::RmarkerRx, mask_of(rmarker_rx_bits()),
                              d, ts));

    // The timestamp carries revision r1; the exchange is running on r2.
    const CalibrationStamp cal = good_calibration(7u, "cal-dw1000-sn0001-ch5-uus-r2");
    RangeAdmissionContext ctx;
    ctx.calibration = &cal;
    ctx.reference_ticks = 1500;
    ctx.reference_ticks_recorded = true;
    ctx.rx_first_path = FirstPathQuality::passed(18.5, 9.0, 0.82);

    const RangeAdmission r = admit_range_capable_time(ts, ctx);
    BOOST_TEST(!r.admitted);
    BOOST_TEST(r.reason == RangeAdmissionReason::CalibrationMismatch);
    BOOST_TEST(r.status == ExchangeStatus::CalibrationMissing);
    BOOST_TEST(r.detail.find("r1") != std::string::npos);
    BOOST_TEST(r.detail.find("r2") != std::string::npos);
}

BOOST_AUTO_TEST_CASE(tof_stale_calibration_is_refused)
{
    gr::uwb::twr::ClockDomain d;
    BOOST_REQUIRE(make_uus_domain(d));
    Timestamp ts;
    BOOST_REQUIRE(make_mapped(9000, TimestampMarker::RmarkerRx, mask_of(rmarker_rx_bits()),
                              d, ts));

    // (a) The exchange instant falls outside the validity window.
    {
        const CalibrationStamp cal = good_calibration();
        RangeAdmissionContext ctx;
        ctx.calibration = &cal;
        ctx.reference_ticks = 5000; // after valid_until_ticks == 2000
        ctx.reference_ticks_recorded = true;
        ctx.rx_first_path = FirstPathQuality::passed(18.5, 9.0, 0.82);
        const RangeAdmission r = admit_range_capable_time(ts, ctx);
        BOOST_TEST(!r.admitted);
        BOOST_TEST(r.reason == RangeAdmissionReason::CalibrationStale);
        BOOST_TEST(r.status == ExchangeStatus::CalibrationExpired);
    }

    // (b) The exclusive upper bound is exclusive.
    {
        const CalibrationStamp cal = good_calibration();
        RangeAdmissionContext ctx;
        ctx.calibration = &cal;
        ctx.reference_ticks = 2000;
        ctx.reference_ticks_recorded = true;
        ctx.rx_first_path = FirstPathQuality::passed(18.5, 9.0, 0.82);
        const RangeAdmission r = admit_range_capable_time(ts, ctx);
        BOOST_TEST(!r.admitted);
        BOOST_TEST(r.reason == RangeAdmissionReason::CalibrationStale);
    }

    // (c) The calibration was established before a reboot / time reset, so
    //     it belongs to a different epoch and must not be reused.
    {
        const CalibrationStamp cal = good_calibration(6u);
        RangeAdmissionContext ctx;
        ctx.calibration = &cal;
        ctx.reference_ticks = 1500;
        ctx.reference_ticks_recorded = true;
        ctx.rx_first_path = FirstPathQuality::passed(18.5, 9.0, 0.82);
        const RangeAdmission r = admit_range_capable_time(ts, ctx);
        BOOST_TEST(!r.admitted);
        BOOST_TEST(r.reason == RangeAdmissionReason::CalibrationStale);
        BOOST_TEST(r.status == ExchangeStatus::CalibrationExpired);
    }

    // (d) A degenerate window (zero length, or inverted) is not a window.
    {
        CalibrationStamp cal = good_calibration();
        cal.valid_until_ticks = cal.valid_from_ticks;
        RangeAdmissionContext ctx;
        ctx.calibration = &cal;
        ctx.reference_ticks = 1500;
        ctx.reference_ticks_recorded = true;
        ctx.rx_first_path = FirstPathQuality::passed(18.5, 9.0, 0.82);
        const RangeAdmission r = admit_range_capable_time(ts, ctx);
        BOOST_TEST(!r.admitted);
        BOOST_TEST(r.reason == RangeAdmissionReason::CalibrationStale);
    }

    // (e) No application record at all: "the calibration exists" is not
    //     "this timestamp was calibrated by it".
    {
        CalibrationStamp cal = good_calibration();
        cal.applications.clear();
        RangeAdmissionContext ctx;
        ctx.calibration = &cal;
        ctx.reference_ticks = 1500;
        ctx.reference_ticks_recorded = true;
        ctx.rx_first_path = FirstPathQuality::passed(18.5, 9.0, 0.82);
        const RangeAdmission r = admit_range_capable_time(ts, ctx);
        BOOST_TEST(!r.admitted);
        BOOST_TEST(r.reason == RangeAdmissionReason::NoCalibration);
    }
}

// ===========================================================================
// REQ-TIME-05 -- the first-path quality decision
// ===========================================================================

BOOST_AUTO_TEST_CASE(tof_first_path_quality_gate)
{
    gr::uwb::twr::ClockDomain d;
    BOOST_REQUIRE(make_uus_domain(d));
    Timestamp rx;
    BOOST_REQUIRE(make_mapped(9000, TimestampMarker::RmarkerRx, mask_of(rmarker_rx_bits()),
                              d, rx));
    const CalibrationStamp cal = good_calibration();

    // (a) Gate failed -> refuse.  Never "recover" a distance from it.
    {
        RangeAdmissionContext ctx;
        ctx.calibration = &cal;
        ctx.reference_ticks = 1500;
        ctx.reference_ticks_recorded = true;
        ctx.rx_first_path = FirstPathQuality::failed(2.1, 0.0, 0.05);
        const RangeAdmission r = admit_range_capable_time(rx, ctx);
        BOOST_TEST(!r.admitted);
        BOOST_TEST(r.reason == RangeAdmissionReason::FirstPathUnreliable);
        BOOST_TEST(r.status == ExchangeStatus::FirstPathUnreliable);
        BOOST_TEST(!r.value.has_value());
    }

    // (b) Nothing recorded at all -> refuse, and specifically NOT as a
    //     defaulted "fine".  A default-constructed context must fail.
    {
        RangeAdmissionContext ctx;
        ctx.calibration = &cal;
        ctx.reference_ticks = 1500;
        ctx.reference_ticks_recorded = true;
        const RangeAdmission r = admit_range_capable_time(rx, ctx);
        BOOST_TEST(!r.admitted);
        BOOST_TEST(r.reason == RangeAdmissionReason::FirstPathQualityNotRecorded);
        BOOST_TEST(r.status == ExchangeStatus::FirstPathUnreliable);
        BOOST_TEST(!r.value.has_value());
    }
    {
        RangeAdmissionContext ctx;
        ctx.calibration = &cal;
        ctx.reference_ticks = 1500;
        ctx.reference_ticks_recorded = true;
        ctx.rx_first_path = FirstPathQuality::not_recorded();
        BOOST_TEST(!admit_range_capable_time(rx, ctx).admitted);
    }

    // (c) A "passed" decision carrying impossible numbers is not a decision.
    {
        RangeAdmissionContext ctx;
        ctx.calibration = &cal;
        ctx.reference_ticks = 1500;
        ctx.reference_ticks_recorded = true;
        ctx.rx_first_path = FirstPathQuality::passed(
            std::numeric_limits<double>::quiet_NaN(), 9.0, 0.82);
        const RangeAdmission r = admit_range_capable_time(rx, ctx);
        BOOST_TEST(!r.admitted);
        BOOST_TEST(r.reason == RangeAdmissionReason::FirstPathQualityInvalid);
    }
    {
        RangeAdmissionContext ctx;
        ctx.calibration = &cal;
        ctx.reference_ticks = 1500;
        ctx.reference_ticks_recorded = true;
        ctx.rx_first_path = FirstPathQuality::passed(18.5, 9.0, 1.5); // confidence > 1
        const RangeAdmission r = admit_range_capable_time(rx, ctx);
        BOOST_TEST(!r.admitted);
        BOOST_TEST(r.reason == RangeAdmissionReason::FirstPathQualityInvalid);
    }

    // (d) Passed -> admitted, and the decision is carried through for the
    //     result JSON rather than being thrown away.
    {
        RangeAdmissionContext ctx;
        ctx.calibration = &cal;
        ctx.reference_ticks = 1500;
        ctx.reference_ticks_recorded = true;
        ctx.rx_first_path = FirstPathQuality::passed(18.5, 9.0, 0.82);
        const RangeAdmission r = admit_range_capable_time(rx, ctx);
        BOOST_TEST(r.admitted);
        BOOST_REQUIRE(r.value.has_value());
        BOOST_TEST(r.value->first_path_quality().decision == FirstPathDecision::Passed);
        BOOST_TEST(r.value->first_path_quality().confidence == 0.82);
    }
}

// ===========================================================================
// Epoch / cross-domain discipline
// ===========================================================================

BOOST_AUTO_TEST_CASE(tof_cross_epoch_pair_is_refused)
{
    gr::uwb::twr::ClockDomain d7;
    gr::uwb::twr::ClockDomain d8;
    BOOST_REQUIRE(make_uus_domain(d7, "dw1000_sn0001_ch5_uus", 7u, 40u));
    BOOST_REQUIRE(make_uus_domain(d8, "dw1000_sn0001_ch5_uus", 8u, 40u));

    Timestamp old_ts;
    Timestamp new_ts;
    BOOST_REQUIRE(
        make_mapped(9000, TimestampMarker::RmarkerRx, mask_of(rmarker_rx_bits()), d7,
                    old_ts));
    BOOST_REQUIRE(
        make_mapped(9000, TimestampMarker::RmarkerTx, mask_of(rmarker_tx_bits()), d8, new_ts));

    // One calibration per epoch, both otherwise perfect.
    const CalibrationStamp cal7 = good_calibration(7u);
    const CalibrationStamp cal8 = good_calibration(8u);

    RangeAdmissionContext ctx_old;
    ctx_old.calibration = &cal7;
    ctx_old.reference_ticks = 1500;
    ctx_old.reference_ticks_recorded = true;
    ctx_old.rx_first_path = FirstPathQuality::passed(18.5, 9.0, 0.82);
    RangeAdmissionContext ctx_new = ctx_old;
    ctx_new.calibration = &cal8;

    BOOST_TEST(admit_range_capable_time(old_ts, ctx_old).admitted);
    BOOST_TEST(admit_range_capable_time(new_ts, ctx_new).admitted);

    // Individually valid, together on two timelines: refused.
    const RangingIntervalAdmission r = admit_ranging_interval(old_ts, new_ts, ctx_old);
    BOOST_TEST(!r.admitted);
    BOOST_TEST(!r.value.has_value());
    BOOST_TEST(r.reason == RangeAdmissionReason::WrongEpoch);
    BOOST_TEST(r.status == ExchangeStatus::InvalidTimeDomain);
    BOOST_TEST(r.interval_status == TimeIntervalStatus::EpochMismatch);
}

BOOST_AUTO_TEST_CASE(tof_cross_domain_pair_is_refused)
{
    const CalibrationStamp cal = good_calibration();

    struct Case {
        const char* other_name;
        double other_rate;
        uint32_t other_bits;
    };
    const Case cases[] = {
        {"dw1000_sn0002_ch5_uus", 499.2e6 * 128.0, 40u}, // a second board
        {"dw1000_sn0001_ch5_uus", 998.4e6, 40u},          // a different grid
        {"dw1000_sn0001_ch5_uus", 499.2e6 * 128.0, 32u},  // truncated wire field
    };

    for (const Case& c : cases) {
        gr::uwb::twr::ClockDomain a;
        gr::uwb::twr::ClockDomain b;
        BOOST_REQUIRE(make_uus_domain(a));
        BOOST_REQUIRE(gr::uwb::twr::ClockDomain::make(c.other_name, c.other_rate, 7u,
                                                     c.other_bits, b));

        Timestamp ta;
        Timestamp tb;
        BOOST_REQUIRE(
            make_mapped(9000, TimestampMarker::RmarkerRx, mask_of(rmarker_rx_bits()), a,
                        ta));
        BOOST_REQUIRE(
            make_mapped(5000, TimestampMarker::RmarkerTx, mask_of(rmarker_tx_bits()), b, tb));

        RangeAdmissionContext ctx;
        ctx.calibration = &cal;
        ctx.reference_ticks = 1500;
        ctx.reference_ticks_recorded = true;
        ctx.rx_first_path = FirstPathQuality::passed(18.5, 9.0, 0.82);

        // Each side is fine on its own timeline.
        BOOST_TEST(admit_range_capable_time(ta, ctx).admitted);
        BOOST_TEST(admit_range_capable_time(tb, ctx).admitted);

        // Together they are refused, and never silently converted.
        const RangingIntervalAdmission r = admit_ranging_interval(ta, tb, ctx);
        BOOST_TEST(!r.admitted);
        BOOST_TEST(!r.value.has_value());
        BOOST_TEST(r.reason == RangeAdmissionReason::CrossDomain);
        BOOST_TEST(r.status == ExchangeStatus::InvalidTimeDomain);
    }
}

BOOST_AUTO_TEST_CASE(tof_disallowed_marker_pair_is_refused)
{
    gr::uwb::twr::ClockDomain d;
    BOOST_REQUIRE(make_uus_domain(d));
    const CalibrationStamp cal = good_calibration();
    RangeAdmissionContext ctx;
    ctx.calibration = &cal;
    ctx.reference_ticks = 1500;
    ctx.reference_ticks_recorded = true;
    ctx.rx_first_path = FirstPathQuality::passed(18.5, 9.0, 0.82);

    // The antenna-plane instant minus the protocol RMARKER is a bookkeeping
    // offset, not a propagation time.  Both sides are range-capable; the PAIR
    // is not.
    Timestamp plane;
    Timestamp rmarker;
    BOOST_REQUIRE(
        make_mapped(7000, TimestampMarker::AntennaPlane, mask_of(antenna_plane_bits()), d,
                    plane));
    BOOST_REQUIRE(
        make_mapped(5000, TimestampMarker::RmarkerRx, mask_of(rmarker_rx_bits()), d, rmarker));

    BOOST_TEST(admit_range_capable_time(plane, ctx).admitted);
    BOOST_TEST(admit_range_capable_time(rmarker, ctx).admitted);

    const RangingIntervalAdmission r = admit_ranging_interval(plane, rmarker, ctx);
    BOOST_TEST(!r.admitted);
    BOOST_TEST(r.reason == RangeAdmissionReason::MarkerPairNotAllowed);
    BOOST_TEST(r.status == ExchangeStatus::InvalidTimeDomain);
    BOOST_TEST(r.interval_status == TimeIntervalStatus::MarkerMismatch);
}

BOOST_AUTO_TEST_CASE(tof_residual_interval_failure_is_surfaced)
{
    // Two individually perfect RMARKERs that are more than half a wrap period
    // apart: the modular difference is ambiguous, so the pair is refused with
    // the underlying interval status attached rather than guessed.
    gr::uwb::twr::ClockDomain d;
    BOOST_REQUIRE(make_uus_domain(d));
    const CalibrationStamp cal = good_calibration();
    RangeAdmissionContext ctx;
    ctx.calibration = &cal;
    ctx.reference_ticks = 1500;
    ctx.reference_ticks_recorded = true;
    ctx.rx_first_path = FirstPathQuality::passed(18.5, 9.0, 0.82);

    // Exactly P/2 apart is ACCEPTED on purpose (`raw_tick_delta()` is handed an
    // explicit later/earlier pair, so the magnitude at d == P/2 is unique), so
    // the ambiguous case has to be pushed just past it.
    const int64_t quarter = int64_t{1} << 38; // P/4
    Timestamp a;
    Timestamp b;
    BOOST_REQUIRE(
        make_mapped(quarter, TimestampMarker::RmarkerRx, mask_of(rmarker_rx_bits()), d, a));
    BOOST_REQUIRE(
        make_mapped(3 * quarter + 1000, TimestampMarker::RmarkerRx,
                    mask_of(rmarker_rx_bits()), d, b));

    const RangingIntervalAdmission r = admit_ranging_interval(b, a, ctx);
    BOOST_TEST(!r.admitted);
    BOOST_TEST(r.reason == RangeAdmissionReason::IntervalNotFormable);
    BOOST_TEST(r.interval_status == TimeIntervalStatus::WrapAmbiguous);
    BOOST_TEST(r.status == ExchangeStatus::InvalidTimeDomain);
    BOOST_TEST(!r.value.has_value());
}

// ===========================================================================
// The safe value M1 gets
// ===========================================================================

BOOST_AUTO_TEST_CASE(tof_admitted_interval_is_lossless_and_carries_no_nanoseconds)
{
    gr::uwb::twr::ClockDomain d;
    BOOST_REQUIRE(make_uus_domain(d));
    const CalibrationStamp cal = good_calibration();
    RangeAdmissionContext ctx;
    ctx.calibration = &cal;
    ctx.reference_ticks = 1500;
    ctx.reference_ticks_recorded = true;
    ctx.rx_first_path = FirstPathQuality::passed(18.5, 9.0, 0.82);

    // 1000 - (500 + 1/2) = 499 + 1/2 = 999/2 ticks at the UUS rate.
    Timestamp a;
    Timestamp b;
    {
        Timestamp tmp;
        BOOST_REQUIRE(Timestamp::from_ticks(1000, d, TimestampMarker::RmarkerRx,
                                            TimestampSource::HardwareMeasured,
                                            mask_of(rmarker_rx_bits()), tmp));
        BOOST_TEST(gr::uwb::twr::apply_calibration_ticks(
                       tmp, cal.id, 0, 0, 0u, 0u) == CalibrationResult::Applied);
        a = tmp;
    }
    {
        Timestamp tmp;
        BOOST_REQUIRE(Timestamp::from_fractional_ticks(
            500, 1, 2, d, TimestampMarker::RmarkerRx, TimestampSource::HardwareMeasured,
            mask_of(rmarker_rx_bits()), tmp));
        BOOST_TEST(gr::uwb::twr::apply_calibration_ticks(
                       tmp, cal.id, 0, 0, 0u, 0u) == CalibrationResult::Applied);
        b = tmp;
    }

    const RangingIntervalAdmission r = admit_ranging_interval(a, b, ctx);
    BOOST_TEST(r.admitted);
    BOOST_REQUIRE(r.value.has_value());

    // The exact rational tick value survives untouched.
    int64_t num = 0;
    int64_t den = 0;
    BOOST_REQUIRE(r.value->exact_ratio(num, den));
    BOOST_TEST(num == 999);
    BOOST_TEST(den == 2);
    BOOST_TEST(!r.value->is_whole_ticks());

    // And the nanosecond view of the same number is a LOSSY truncation, which
    // is exactly why the admitted type must not hand M1 one.
    const TimeInterval ti = timestamp_interval(a, b);
    BOOST_REQUIRE(ti.status == TimeIntervalStatus::Ok);
    BOOST_TEST(!ti.duration_is_lossless_ns);
    // 499.5 ticks * 1/(499.2e6*128) s = 7.8146e-9 s = 7.8146... ns -> 7 ns:
    // 0.81 ns is discarded, which is larger than any plausible UWB range step.
    BOOST_TEST(ti.duration.nanos() == 7);
    BOOST_TEST(r.value->ns_projection_is_lossless() == false);
    // The admitted interval is the exact rational, not that integer.
    BOOST_TEST(ti.duration.nanos() != 8);

    // Structural guarantee: neither the admitted time nor the admitted
    // interval exposes a nanosecond accessor at all.  The positive controls
    // prove the detectors are not vacuous.
    BOOST_TEST((has_nanos_member<Duration>::value));
    BOOST_TEST((has_duration_member<TimeInterval>::value));
    BOOST_TEST(!(has_nanos_member<TimeInterval>::value));
    BOOST_TEST((has_seconds_member<RelativeTickInterval>::value));
    BOOST_TEST(!(has_nanos_member<RelativeTickInterval>::value));
    BOOST_TEST(!(has_duration_member<RelativeTickInterval>::value));
    BOOST_TEST(!(has_nanos_member<RangeCapableTime>::value));
    BOOST_TEST(!(has_duration_member<RangeCapableTime>::value));
    BOOST_TEST(!(has_seconds_member<RangeCapableTime>::value));
    BOOST_TEST(!(has_nanos_member<AdmittedRangingInterval>::value));
    BOOST_TEST(!(has_duration_member<AdmittedRangingInterval>::value));
    BOOST_TEST(!(has_seconds_member<AdmittedRangingInterval>::value));
}

BOOST_AUTO_TEST_CASE(tof_wrap_crossing_pair_is_admitted_with_the_wrap_flag)
{
    // A reply that crosses the counter wrap is a normal TWR event, not an
    // error.  The admitted value must say so, because the caller cannot see
    // the wrap anywhere else.
    gr::uwb::twr::ClockDomain d;
    BOOST_REQUIRE(make_uus_domain(d));
    const CalibrationStamp cal = good_calibration();
    RangeAdmissionContext ctx;
    ctx.calibration = &cal;
    ctx.reference_ticks = 1500;
    ctx.reference_ticks_recorded = true;
    ctx.rx_first_path = FirstPathQuality::passed(18.5, 9.0, 0.82);

    const int64_t p = int64_t{1} << 40;
    Timestamp a; // just AFTER the rollover
    Timestamp b; // just BEFORE it
    BOOST_REQUIRE(
        make_mapped(100, TimestampMarker::RmarkerRx, mask_of(rmarker_rx_bits()), d, a));
    BOOST_REQUIRE(
        make_mapped(p - 100, TimestampMarker::RmarkerRx, mask_of(rmarker_rx_bits()), d, b));

    int64_t num = 0;
    int64_t den = 0;
    const RangingIntervalAdmission r = admit_ranging_interval(a, b, ctx);
    BOOST_TEST(r.admitted);
    BOOST_REQUIRE(r.value.has_value());
    BOOST_TEST(r.value->wrapped());
    BOOST_TEST(r.value->interval().ticks == 200);
    BOOST_TEST(r.value->interval().wrapped);
    // The modular resolution is exact: 100 - (P-100) mod P == 200.
    BOOST_REQUIRE(r.value->exact_ratio(num, den));
    BOOST_TEST(num == 200);
    BOOST_TEST(den == 1);
}

// ===========================================================================
// The error taxonomy
// ===========================================================================

BOOST_AUTO_TEST_CASE(tof_every_reason_round_trips_and_maps_to_a_defined_status)
{
    // The exact mapping is pinned, so a future edit that quietly widens or
    // narrows the taxonomy is a test failure.
    const struct {
        RangeAdmissionReason reason;
        const char* text;
        ExchangeStatus status;
    } expected[] = {
        {RangeAdmissionReason::Admitted, "admitted", ExchangeStatus::Ok},
        {RangeAdmissionReason::InvalidTimestamp, "invalid_timestamp",
         ExchangeStatus::InvalidTimeDomain},
        {RangeAdmissionReason::NotRangeCapableMarker, "not_range_capable_marker",
         ExchangeStatus::InvalidTimeDomain},
        {RangeAdmissionReason::NotMeasured, "not_measured",
         ExchangeStatus::InvalidTimeDomain},
        {RangeAdmissionReason::MissingCorrection, "missing_correction",
         ExchangeStatus::InvalidTimeDomain},
        {RangeAdmissionReason::NoCalibration, "no_calibration",
         ExchangeStatus::CalibrationMissing},
        {RangeAdmissionReason::CalibrationMismatch, "calibration_mismatch",
         ExchangeStatus::CalibrationMissing},
        {RangeAdmissionReason::CalibrationAppliedTwice, "calibration_applied_twice",
         ExchangeStatus::CalibrationMissing},
        {RangeAdmissionReason::CalibrationStale, "calibration_stale",
         ExchangeStatus::CalibrationExpired},
        {RangeAdmissionReason::FirstPathQualityNotRecorded, "first_path_quality_not_recorded",
         ExchangeStatus::FirstPathUnreliable},
        {RangeAdmissionReason::FirstPathQualityInvalid, "first_path_quality_invalid",
         ExchangeStatus::FirstPathUnreliable},
        {RangeAdmissionReason::FirstPathUnreliable, "first_path_unreliable",
         ExchangeStatus::FirstPathUnreliable},
        {RangeAdmissionReason::MarkerPairNotAllowed, "marker_pair_not_allowed",
         ExchangeStatus::InvalidTimeDomain},
        {RangeAdmissionReason::WrongEpoch, "wrong_epoch", ExchangeStatus::InvalidTimeDomain},
        {RangeAdmissionReason::CrossDomain, "cross_domain",
         ExchangeStatus::InvalidTimeDomain},
        {RangeAdmissionReason::IntervalNotFormable, "interval_not_formable",
         ExchangeStatus::InvalidTimeDomain},
        {RangeAdmissionReason::ScheduledTxEvidenceMissing,
         "scheduled_tx_evidence_missing", ExchangeStatus::InvalidTimeDomain},
        {RangeAdmissionReason::TxOutcomeLate, "tx_outcome_late",
         ExchangeStatus::TxLate},
        {RangeAdmissionReason::TxOutcomeUnderflow, "tx_outcome_underflow",
         ExchangeStatus::TxUnderflow},
        {RangeAdmissionReason::TxOutcomeCancelled, "tx_outcome_cancelled",
         ExchangeStatus::Cancelled},
    };
    const int n = static_cast<int>(sizeof(expected) / sizeof(expected[0]));
    BOOST_TEST(n == static_cast<int>(kRangeAdmissionReasonMax) + 1);

    for (int i = 0; i < n; ++i) {
        const std::string s = range_admission_reason_to_string(expected[i].reason);
        BOOST_TEST(s == std::string(expected[i].text));
        RangeAdmissionReason back = RangeAdmissionReason::Admitted;
        BOOST_REQUIRE(range_admission_reason_from_string(s, back));
        BOOST_TEST(back == expected[i].reason);
        BOOST_TEST(range_admission_reason_to_exchange_status(expected[i].reason) ==
                   expected[i].status);
    }

    // A garbage code round-trips to nothing, and an out-of-range value is
    // never silently treated as Admitted.
    RangeAdmissionReason junk = RangeAdmissionReason::Admitted;
    BOOST_TEST(!range_admission_reason_from_string("no_such_reason", junk));
    BOOST_TEST(!range_admission_reason_from_string("", junk));
    BOOST_TEST(range_admission_reason_to_string(
                   static_cast<RangeAdmissionReason>(kRangeAdmissionReasonMax + 1)) ==
               std::string("invalid"));

    // The taxonomy must keep the three physical failure classes apart: a
    // timing-domain problem, a calibration problem and a first-path problem
    // are never collapsed onto one code.
    std::vector<ExchangeStatus> distinct;
    for (int i = 0; i < n; ++i) {
        const ExchangeStatus s = range_admission_reason_to_exchange_status(expected[i].reason);
        if (std::find(distinct.begin(), distinct.end(), s) == distinct.end())
            distinct.push_back(s);
    }
    const auto reached = [&distinct](ExchangeStatus s) {
        return std::find(distinct.begin(), distinct.end(), s) != distinct.end();
    };
    BOOST_TEST(distinct.size() == 8u);
    BOOST_TEST(reached(ExchangeStatus::Ok));
    BOOST_TEST(reached(ExchangeStatus::InvalidTimeDomain));
    BOOST_TEST(reached(ExchangeStatus::CalibrationMissing));
    BOOST_TEST(reached(ExchangeStatus::CalibrationExpired));
    BOOST_TEST(reached(ExchangeStatus::FirstPathUnreliable));
    // N02: a timed TX that did not complete is not a time-domain problem.  The
    // three TX failure modes keep their own terminal codes so a soak report can
    // separate them without parsing prose.
    BOOST_TEST(reached(ExchangeStatus::TxLate));
    BOOST_TEST(reached(ExchangeStatus::TxUnderflow));
    BOOST_TEST(reached(ExchangeStatus::Cancelled));

    // A refusal can never yield a range (REQ-PROTO-03 / types.h).
    for (int i = 1; i < n; ++i) {
        BOOST_TEST(!gr::uwb::twr::exchange_status_yields_range(expected[i].status));
    }
}

BOOST_AUTO_TEST_CASE(tof_detail_strings_name_the_evidence)
{
    // A machine-readable reason is not enough for an operator reading a log:
    // the detail must say which calibration, which epoch, which marker.
    gr::uwb::twr::ClockDomain d;
    BOOST_REQUIRE(make_uus_domain(d));
    Timestamp ts;
    BOOST_REQUIRE(make_mapped(9000, TimestampMarker::RmarkerRx, mask_of(rmarker_rx_bits()),
                              d, ts));

    // (a) A calibration from the wrong epoch: the detail names the set AND
    //     the epoch, so an operator can tell which constant to re-measure.
    const CalibrationStamp stale = good_calibration(6u);
    RangeAdmissionContext ctx;
    ctx.calibration = &stale;
    ctx.reference_ticks = 1500;
    ctx.reference_ticks_recorded = true;
    ctx.rx_first_path = FirstPathQuality::passed(18.5, 9.0, 0.82);

    const RangeAdmission r = admit_range_capable_time(ts, ctx);
    BOOST_TEST(!r.admitted);
    BOOST_TEST(r.reason == RangeAdmissionReason::CalibrationStale);
    BOOST_TEST(!r.detail.empty());
    BOOST_TEST(r.detail.find("cal-dw1000-sn0001-ch5-uus-r1") != std::string::npos);
    BOOST_TEST(r.detail.find("epoch") != std::string::npos);

    // (b) A different calibration set in force: the detail names both ids.
    const CalibrationStamp other = good_calibration(7u, "cal-dw1000-sn0001-ch5-uus-r7");
    RangeAdmissionContext ctx2;
    ctx2.calibration = &other;
    ctx2.reference_ticks = 1500;
    ctx2.reference_ticks_recorded = true;
    ctx2.rx_first_path = FirstPathQuality::passed(18.5, 9.0, 0.82);
    const RangeAdmission r2 = admit_range_capable_time(ts, ctx2);
    BOOST_TEST(!r2.admitted);
    BOOST_TEST(r2.reason == RangeAdmissionReason::CalibrationMismatch);
    BOOST_TEST(r2.detail.find("cal-dw1000-sn0001-ch5-uus-r1") != std::string::npos);
    BOOST_TEST(r2.detail.find("cal-dw1000-sn0001-ch5-uus-r7") != std::string::npos);
}

// ===========================================================================
// The escape hatch is closed
// ===========================================================================

BOOST_AUTO_TEST_CASE(tof_admission_result_json_names_the_reason)
{
    // REQ-OUT-01: the result record has to survive into the exchange result,
    // and a machine must be able to read the terminal status and the specific
    // reason out of it without parsing prose.
    gr::uwb::twr::ClockDomain d;
    BOOST_REQUIRE(make_uus_domain(d));
    Timestamp ts;
    BOOST_REQUIRE(make_mapped(9000, TimestampMarker::RmarkerRx,
                              mask_of(rmarker_rx_bits()) & ~kBitRmarkerOffset, d, ts));
    const CalibrationStamp cal = good_calibration();
    RangeAdmissionContext ctx;
    ctx.calibration = &cal;
    ctx.reference_ticks = 1500;
    ctx.reference_ticks_recorded = true;
    ctx.rx_first_path = FirstPathQuality::passed(18.5, 9.0, 0.82);

    const RangeAdmission single = admit_range_capable_time(ts, ctx);
    BOOST_TEST(!single.admitted);
    const std::string js = single.to_json_string();
    BOOST_TEST(js.find("\"admitted\":false") != std::string::npos);
    BOOST_TEST(js.find("\"reason\":\"missing_correction\"") != std::string::npos);
    BOOST_TEST(js.find("\"exchange_status\":\"invalid_time_domain\"") != std::string::npos);
    BOOST_TEST(js.find("\"missing_correction\":\"rmarker_offset\"") != std::string::npos);
    BOOST_TEST(js.find("\"marker\":\"rmarker_rx\"") != std::string::npos);

    const RangingIntervalAdmission pair = admit_ranging_interval(ts, ts, ctx);
    BOOST_TEST(!pair.admitted);
    BOOST_TEST(pair.side == RangeTimeSide::Later);
}

BOOST_AUTO_TEST_CASE(tof_admitted_type_cannot_exist_without_the_factory)
{
    // A caller cannot default-construct or hand-assemble an admitted value.
    BOOST_TEST(!(std::is_default_constructible<RangeCapableTime>::value));
    BOOST_TEST(!(std::is_constructible<RangeCapableTime, const Timestamp&>::value));
    BOOST_TEST(!(std::is_constructible<RangeCapableTime, const Timestamp&,
                                       const FirstPathQuality&>::value));
    BOOST_TEST(!(std::is_aggregate<RangeCapableTime>::value));
    BOOST_TEST(!(std::is_default_constructible<AdmittedRangingInterval>::value));
    BOOST_TEST(!(std::is_constructible<AdmittedRangingInterval,
                                       const RelativeTickInterval&>::value));
    BOOST_TEST(!(std::is_aggregate<AdmittedRangingInterval>::value));

    // It is a value: copies and moves are allowed, so it can be stored and
    // handed to M1.
    BOOST_TEST((std::is_copy_constructible<RangeCapableTime>::value));
    BOOST_TEST((std::is_copy_assignable<RangeCapableTime>::value));
    BOOST_TEST((std::is_move_constructible<RangeCapableTime>::value));
    BOOST_TEST((std::is_copy_constructible<AdmittedRangingInterval>::value));

    // And the admitted value is not a Timestamp in disguise: it does not
    // convert implicitly to one either.
    BOOST_TEST(!(std::is_convertible<RangeCapableTime, Timestamp>::value));
    BOOST_TEST(!(std::is_convertible<RangeCapableTime, int64_t>::value));
}

BOOST_AUTO_TEST_CASE(tof_value_is_engaged_exactly_when_admitted)
{
    // The invariant that makes the type worth having: there is no path that
    // yields a populated value together with a refusal, or an empty value
    // together with an acceptance.
    gr::uwb::twr::ClockDomain d;
    BOOST_REQUIRE(make_uus_domain(d));
    const CalibrationStamp cal = good_calibration();

    std::vector<Timestamp> pool;
    Timestamp t;
    BOOST_REQUIRE(make_raw_iq_sample(1, d, t));
    pool.push_back(t);
    BOOST_REQUIRE(make_mapped(9000, TimestampMarker::RmarkerRx, mask_of(rmarker_rx_bits()),
                              d, t));
    pool.push_back(t);
    BOOST_REQUIRE(make_mapped(9000, TimestampMarker::RmarkerRx,
                              mask_of(rmarker_rx_bits()) & ~kBitWindowCrop, d, t));
    pool.push_back(t);
    BOOST_REQUIRE(make_mapped(9000, TimestampMarker::RmarkerTx, mask_of(rmarker_tx_bits()),
                              d, t));
    pool.push_back(t);

    int admitted_count = 0;
    int rejected_count = 0;
    for (const Timestamp& ts : pool) {
        for (int qv = 0; qv < 3; ++qv) {
            RangeAdmissionContext ctx;
            ctx.calibration = &cal;
            ctx.reference_ticks = 1500;
            ctx.reference_ticks_recorded = true;
            ctx.rx_first_path = (qv == 0)   ? FirstPathQuality::passed(18.5, 9.0, 0.82)
                                : (qv == 1) ? FirstPathQuality::failed(1.0, 0.0, 0.0)
                                            : FirstPathQuality::not_recorded();
            const RangeAdmission r = admit_range_capable_time(ts, ctx);
            BOOST_TEST(r.admitted == (r.reason == RangeAdmissionReason::Admitted));
            BOOST_TEST(r.value.has_value() == r.admitted);
            BOOST_TEST((r.status == ExchangeStatus::Ok) == r.admitted);
            if (r.admitted)
                ++admitted_count;
            else
                ++rejected_count;
        }
    }
    BOOST_TEST(admitted_count > 0);
    BOOST_TEST(rejected_count > 0);

    for (const Timestamp& later : pool) {
        for (const Timestamp& earlier : pool) {
            RangeAdmissionContext ctx;
            ctx.calibration = &cal;
            ctx.reference_ticks = 1500;
            ctx.reference_ticks_recorded = true;
            ctx.rx_first_path = FirstPathQuality::passed(18.5, 9.0, 0.82);
            const RangingIntervalAdmission r = admit_ranging_interval(later, earlier, ctx);
            BOOST_TEST(r.admitted == (r.reason == RangeAdmissionReason::Admitted));
            BOOST_TEST(r.value.has_value() == r.admitted);
            if (!r.admitted) {
                BOOST_TEST(!gr::uwb::twr::exchange_status_yields_range(r.status));
            }
        }
    }
}

BOOST_AUTO_TEST_CASE(tof_invalid_timestamp_is_refused_before_anything_else)
{
    gr::uwb::twr::ClockDomain d;
    BOOST_REQUIRE(make_uus_domain(d));
    const CalibrationStamp cal = good_calibration();
    RangeAdmissionContext ctx;
    ctx.calibration = &cal;
    ctx.reference_ticks = 1500;
    ctx.reference_ticks_recorded = true;
    ctx.rx_first_path = FirstPathQuality::passed(18.5, 9.0, 0.82);

    Timestamp broken;
    BOOST_REQUIRE(make_mapped(9000, TimestampMarker::RmarkerRx, mask_of(rmarker_rx_bits()),
                              d, broken));
    broken.valid = false;
    const RangeAdmission r = admit_range_capable_time(broken, ctx);
    BOOST_TEST(!r.admitted);
    BOOST_TEST(r.reason == RangeAdmissionReason::InvalidTimestamp);
    BOOST_TEST(!r.value.has_value());

    // A default-constructed Timestamp (never went through timestamp_make) is
    // refused the same way.
    const RangeAdmission r2 = admit_range_capable_time(Timestamp(), ctx);
    BOOST_TEST(!r2.admitted);
    BOOST_TEST(r2.reason == RangeAdmissionReason::InvalidTimestamp);
}

// ===========================================================================
// N04 -- the admission layer must not admit an interval whose merged
//        tick+fraction value exceeds half the wrap period
// ===========================================================================
//
// The interval formability gate is reached through `timestamp_relative_interval()`
// -> `timestamp_interval()`, so once the interval re-checks the merged value the
// admission verdict follows.  A 12-bit domain is used (P = 4096, P/2 = 2048)
// because the calibration window must fit inside one wrap period.

BOOST_AUTO_TEST_CASE(tof_interval_upper_bound_after_fraction_is_refused)
{
    gr::uwb::twr::ClockDomain d;
    BOOST_REQUIRE(gr::uwb::twr::ClockDomain::make("dw1000_n04_12bit", 737.28e6, 7u, 12u, d));
    BOOST_TEST(gr::uwb::twr::clock_domain_wrap_period(d) == 4096u);

    const CalibrationStamp cal = good_calibration();
    RangeAdmissionContext ctx;
    ctx.calibration = &cal;
    ctx.reference_ticks = 1500;
    ctx.reference_ticks_recorded = true;
    ctx.rx_first_path = FirstPathQuality::passed(18.5, 9.0, 0.82);

    const auto mapped = [&](int64_t ticks, int32_t num, uint32_t den, TimestampMarker marker,
                            Timestamp& out) {
        Timestamp ts;
        const uint32_t corrections = (marker == TimestampMarker::RmarkerRx)
                                         ? mask_of(rmarker_rx_bits())
                                         : mask_of(rmarker_tx_bits());
        if (!Timestamp::from_fractional_ticks(ticks, num, den, d, marker,
                                              TimestampSource::HardwareMeasured, corrections, ts))
            return false;
        if (gr::uwb::twr::apply_calibration_ticks(ts, cal.id, 0, 0, 0u, 0u) !=
            CalibrationResult::Applied)
            return false;
        out = ts;
        return true;
    };

    Timestamp later;
    Timestamp earlier;
    BOOST_REQUIRE(mapped(2048, 3, 4, TimestampMarker::RmarkerRx, later)); // 2048 + 3/4
    BOOST_REQUIRE(mapped(0, 1, 4, TimestampMarker::RmarkerTx, earlier));   // 0 + 1/4

    // The merged value is 2048 + 1/2 = 2048.5 > 2048, so the pair must be
    // refused as IntervalNotFormable rather than admitted.
    const RangingIntervalAdmission r = admit_ranging_interval(later, earlier, ctx);
    BOOST_TEST(!r.admitted);
    BOOST_TEST(r.reason == RangeAdmissionReason::IntervalNotFormable);
    BOOST_TEST(r.interval_status == TimeIntervalStatus::WrapAmbiguous);
    BOOST_TEST(r.status == ExchangeStatus::InvalidTimeDomain);
    BOOST_TEST(!r.value.has_value());

    // Exactly P/2 (2048 + 1/4 minus 1/4) is still admitted: the documented
    // "exactly half period is allowed when later/earlier are known" policy is
    // unchanged.
    Timestamp half_later;
    Timestamp half_earlier;
    BOOST_REQUIRE(mapped(2048, 1, 4, TimestampMarker::RmarkerRx, half_later));
    BOOST_REQUIRE(mapped(0, 1, 4, TimestampMarker::RmarkerTx, half_earlier));
    const RangingIntervalAdmission rh = admit_ranging_interval(half_later, half_earlier, ctx);
    BOOST_TEST(rh.admitted);
    BOOST_REQUIRE(rh.value.has_value());
    int64_t hn = 0;
    int64_t hd = 0;
    BOOST_REQUIRE(rh.value->exact_ratio(hn, hd));
    BOOST_TEST(hn == 2048);
    BOOST_TEST(hd == 1);
}
