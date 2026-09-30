/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * QA for the M1-B wire-claim / protocol-time boundary
 * (`uwb_twr_protocol_time.h`, G0 gate "wire cannot be forged into a strict
 * measurement input").
 *
 * WHAT IS INDEPENDENT HERE
 * ------------------------
 * Every expected ToF in this file is derived from a PHYSICAL model with
 * Python `fractions.Fraction` and hard-coded as an exact num/den; none of it is
 * produced by calling `compute_protocol_*`.  The physical model is
 *
 *   SS: RA = 2*tau_A + k*DB,  ToF_A = (RA - k*DB)/2 = tau_A
 *   DS: k*RB = 2*tau_A + DA,  ToF_A = (RA*k*RB - DA*k*DB)/(RA+k*RB+DA+k*DB) = tau_A
 *
 * with A = 1.0 GHz, B = 998.4 MHz, k = 625/624, tau_A = 1001/2, DB = 624,
 * RB = 1248, DA = 249, RA = 1626 -- all four wire intervals are WHOLE ticks,
 * while the ToF is the exact fraction 1001/2.  The M1-A report §9 flag is
 * honoured: the protocol path must not need a fraction to be representable on
 * the wire in order to return an exact fractional ToF.
 *
 * No radio, no GNU Radio, no UHD: the protocol-time header depends only on the
 * frame codec and the ToF math headers, which are themselves dependency-free.
 */

#include <boost/test/unit_test.hpp>

#include <gnuradio/uwb/uwb_twr_protocol_time.h>

#include <cstdint>
#include <string>
#include <type_traits>

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
struct print_log_value<gr::uwb::twr::PeerClaimError> {
    void operator()(std::ostream& os, gr::uwb::twr::PeerClaimError const& e) const
    {
        os << gr::uwb::twr::peer_claim_error_to_string(e);
    }
};

template <>
struct print_log_value<gr::uwb::twr::ProtocolCompletionStatus> {
    void operator()(std::ostream& os, gr::uwb::twr::ProtocolCompletionStatus const& s) const
    {
        os << gr::uwb::twr::protocol_completion_status_to_string(s);
    }
};

template <>
struct print_log_value<gr::uwb::twr::ProtocolIntervalOrigin> {
    void operator()(std::ostream& os,
                    gr::uwb::twr::ProtocolIntervalOrigin const& o) const
    {
        os << gr::uwb::twr::protocol_interval_origin_to_string(o);
    }
};

template <>
struct print_log_value<gr::uwb::twr::ExchangeStatus> {
    void operator()(std::ostream& os, gr::uwb::twr::ExchangeStatus const& s) const
    {
        os << gr::uwb::twr::exchange_status_to_string(s);
    }
};

template <>
struct print_log_value<gr::uwb::twr::Protocol> {
    void operator()(std::ostream& os, gr::uwb::twr::Protocol const& p) const
    {
        os << gr::uwb::twr::protocol_to_string(p);
    }
};

template <>
struct print_log_value<gr::uwb::twr::ComputedAt> {
    void operator()(std::ostream& os, gr::uwb::twr::ComputedAt const& c) const
    {
        os << gr::uwb::twr::computed_at_to_string(c);
    }
};

} // namespace tt_detail
} // namespace test_tools
} // namespace boost

BOOST_AUTO_TEST_SUITE(twr_protocol_time)

namespace {

using namespace gr::uwb::twr;

// ---------------------------------------------------------------------------
// Hand-computed constants (Python fractions.Fraction, physical model above).
//
//   k = 1000000000 / 998400000 = 625/624
//   k*DB = (625/624)*624       = 625        -> RA = 2*(1001/2) + 625 = 1626
//   k*RB = (625/624)*1248      = 1250       -> DA = k*RB - 2*(1001/2) = 249
//   SS   ToF = (1626 - 625)/2              = 1001/2
//   DS   N   = 1626*1250 - 249*625         = 1876875
//        D   = 1626 + 1250 + 249 + 625     = 3750
//        ToF = 1876875/3750                 = 1001/2
// ---------------------------------------------------------------------------
constexpr int64_t kRA = 1626; // A ticks, whole
constexpr int64_t kDB = 624;  // B ticks, whole
constexpr int64_t kRB = 1248; // B ticks, whole (never on the wire: t6B is local)
constexpr int64_t kDA = 249;  // A ticks, whole
constexpr int64_t kTofNum = 1001;
constexpr int64_t kTofDen = 2;

constexpr double kRateA = 1.0e9;
constexpr double kRateB = 998.4e6;
constexpr uint64_t kEpoch = 7u;

// ---------------------------------------------------------------------------
// Fixtures
// ---------------------------------------------------------------------------

ClockDomain domain(const std::string& name,
                   double rate,
                   uint64_t epoch = kEpoch,
                   uint32_t bits = 40u)
{
    ClockDomain d;
    BOOST_REQUIRE_MESSAGE(ClockDomain::make(name, rate, epoch, bits, d),
                          "domain " << name << " did not construct");
    return d;
}

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

// Build ONE admitted LOCAL interval through the real M0.1 gate.  No test
// backdoor into RangeCapableTime.
bool admit_interval(const ClockDomain& d,
                    int64_t later_ticks,
                    TimestampMarker later_marker,
                    int64_t earlier_ticks,
                    TimestampMarker earlier_marker,
                    RangingIntervalAdmission& out)
{
    const std::string cal_id = "cal-m1b-" + d.name;
    const CalibrationStamp cal = calibration_for(d, cal_id);

    Timestamp lts;
    Timestamp ets;
    if (!Timestamp::from_ticks(later_ticks, d, later_marker,
                               TimestampSource::HardwareMeasured, kCorrectionNone, lts))
        return false;
    if (!Timestamp::from_ticks(earlier_ticks, d, earlier_marker,
                               TimestampSource::HardwareMeasured, kCorrectionNone, ets))
        return false;
    if (apply_calibration_ticks(lts, cal_id, 0, 0, 0u,
                                timestamp_required_corrections(later_marker)) !=
        CalibrationResult::Applied)
        return false;
    if (apply_calibration_ticks(ets, cal_id, 0, 0, 0u,
                                timestamp_required_corrections(earlier_marker)) !=
        CalibrationResult::Applied)
        return false;

    RangeAdmissionContext ctx;
    ctx.calibration = &cal;
    ctx.reference_ticks = 1500;
    ctx.reference_ticks_recorded = true;
    ctx.rx_first_path = FirstPathQuality::passed(18.5, 9.0, 0.82);

    out = admit_ranging_interval(lts, ets, ctx);
    return out.admitted;
}

// Whole-tick local interval.  `rx_later` picks the marker pair.
bool admit_whole(const ClockDomain& d, int64_t diff, bool rx_later,
                 RangingIntervalAdmission& out)
{
    if (rx_later)
        return admit_interval(d, diff, TimestampMarker::RmarkerRx, 0,
                              TimestampMarker::RmarkerTx, out);
    return admit_interval(d, diff, TimestampMarker::RmarkerTx, 0,
                          TimestampMarker::RmarkerRx, out);
}

ProtocolInterval local_interval(const ClockDomain& d, int64_t diff, bool rx_later)
{
    RangingIntervalAdmission adm;
    BOOST_REQUIRE_MESSAGE(admit_whole(d, diff, rx_later, adm),
                          "local interval of " << diff << " ticks did not pass the gate");
    BOOST_REQUIRE(adm.value.has_value());
    ProtocolInterval p = ProtocolInterval::from_local(*adm.value);
    BOOST_REQUIRE(p.is_local_admitted());
    return p;
}

Frame final_frame(uint16_t src, uint16_t session, uint16_t seq, uint64_t t1,
                  uint64_t t4, uint64_t t5)
{
    Frame f;
    f.function_code = FrameType::Final;
    f.pan_id = 0x1234;
    f.src_addr = src;
    f.dst_addr = 0x0001;
    f.session_id = session;
    f.seq = seq;
    f.flags = make_flags(true, false);
    BOOST_REQUIRE(f.set(TimestampField::T1A, t1));
    BOOST_REQUIRE(f.set(TimestampField::T4A, t4));
    BOOST_REQUIRE(f.set(TimestampField::T5A, t5));
    return f;
}

Frame response_frame(uint16_t src, uint16_t session, uint16_t seq, uint64_t t2,
                     uint64_t t3)
{
    Frame f;
    f.function_code = FrameType::Response;
    f.pan_id = 0x1234;
    f.src_addr = src;
    f.dst_addr = 0x0001;
    f.session_id = session;
    f.seq = seq;
    f.flags = make_flags(true, false);
    BOOST_REQUIRE(f.set(TimestampField::T2B, t2));
    BOOST_REQUIRE(f.set(TimestampField::T3B, t3));
    return f;
}

PeerTimestampClaim claim_from(const Frame& f, TimestampField field)
{
    PeerTimestampClaim c;
    BOOST_REQUIRE_MESSAGE(make_peer_claim(f, field, c),
                          "frame does not carry " << timestamp_field_name(field));
    return c;
}

WireTimestampBinding binding_for(const ClockDomain& d,
                                 uint64_t session_gen = 1,
                                 uint64_t binding_gen = 1,
                                 uint64_t max_ticks = (1ull << 30),
                                 uint16_t modulus = 4,
                                 TimestampMarker marker = TimestampMarker::RmarkerRx)
{
    WireTimestampBinding b;
    b.peer_domain = d;
    b.session_generation = session_gen;
    b.binding_generation = binding_gen;
    b.peer_marker = marker;
    b.unit_convention = "peer_device_ticks";
    b.calibration_convention_id = "twr-link-cal-ch5-v1";
    b.max_interval_ticks = max_ticks;
    b.sequence_modulus = modulus;
    return b;
}

// A hand-built claim with an explicit message identity.  Used only to reach
// the refusals that the factory is designed to make hard to reach; the
// "bound" bit is set on purpose where a bound claim is required.
PeerTimestampClaim hand_claim(TimestampField field,
                              uint64_t raw,
                              uint64_t msg_id,
                              bool bound = true,
                              uint16_t src = 0xAAAA)
{
    PeerTimestampClaim c;
    c.field = field;
    c.raw_ticks = raw;
    c.frame_type = FrameType::Final;
    c.frame_session = 1;
    c.frame_seq = 1;
    c.frame_src = src;
    c.message_identity = msg_id;
    c.bound = bound;
    return c;
}

} // namespace

// ===========================================================================
// 1. The peer wire-claim path produces a valid protocol interval and an exact
//    hand-computed SS estimate, with every range entry false.
// ===========================================================================
BOOST_AUTO_TEST_CASE(protocol_time_ss_peer_path_matches_hand_math)
{
    const ClockDomain a = domain("dw_a_1e9", kRateA);
    const ClockDomain b = domain("dw_b_998p4", kRateB);

    ClockRatio k;
    BOOST_REQUIRE(ClockRatio::from_nominal_rates(a, b, 1000000000LL, 998400000LL, k));
    BOOST_REQUIRE_EQUAL(k.k_num(), 625);
    BOOST_REQUIRE_EQUAL(k.k_den(), 624);
    BOOST_REQUIRE(k.check() == TofStatus::Ok);

    const WireTimestampBinding ba =
        binding_for(a, 1, 1, (1ull << 30), 4, TimestampMarker::RmarkerRx);
    const WireTimestampBinding bb =
        binding_for(b, 1, 1, (1ull << 30), 4, TimestampMarker::RmarkerTx);

    // A's Final carries t1A/t4A/t5A; RA = t4A - t1A = 1626, DA = t5A - t4A = 249.
    const Frame fin = final_frame(0xAAAA, 0x1234, 9, 100000, 100000 + kRA,
                                  100000 + kRA + kDA);
    const PeerTimestampClaim t1 = claim_from(fin, TimestampField::T1A);
    const PeerTimestampClaim t4 = claim_from(fin, TimestampField::T4A);

    ProtocolInterval ra;
    BOOST_REQUIRE(ProtocolInterval::make_peer(t4, t1, ba, ra) == PeerClaimError::Ok);
    BOOST_REQUIRE(ra.is_valid());
    BOOST_REQUIRE(ra.is_peer_wire_claim());
    BOOST_REQUIRE(ra.origin() == ProtocolIntervalOrigin::PeerWireClaim);
    BOOST_REQUIRE(ra.is_whole_ticks());
    {
        int64_t n = 0;
        int64_t d = 0;
        BOOST_REQUIRE(ra.exact_ratio(n, d));
        BOOST_REQUIRE_EQUAL(n, kRA);
        BOOST_REQUIRE_EQUAL(d, 1);
    }

    // B's Response carries t2B/t3B; DB = t3B - t2B = 624.
    const Frame rsp = response_frame(0xBBBB, 0x1234, 9, 500000, 500000 + kDB);
    const PeerTimestampClaim t2 = claim_from(rsp, TimestampField::T2B);
    const PeerTimestampClaim t3 = claim_from(rsp, TimestampField::T3B);

    ProtocolInterval db;
    BOOST_REQUIRE(ProtocolInterval::make_peer(t3, t2, bb, db) == PeerClaimError::Ok);
    BOOST_REQUIRE(db.is_peer_wire_claim());
    {
        int64_t n = 0;
        int64_t d = 0;
        BOOST_REQUIRE(db.exact_ratio(n, d));
        BOOST_REQUIRE_EQUAL(n, kDB);
        BOOST_REQUIRE_EQUAL(d, 1);
    }

    const ProtocolTofEstimate est = compute_protocol_ss_tof(ra, db, k);
    BOOST_REQUIRE_MESSAGE(est.completion == ProtocolCompletionStatus::Complete,
                          est.to_string());
    BOOST_REQUIRE(est.estimate_available);
    BOOST_REQUIRE(est.local_evidence_complete);
    BOOST_REQUIRE(est.peer_evidence_is_wire_claim);
    BOOST_REQUIRE(est.math_status == TofStatus::Ok);
    BOOST_REQUIRE(est.failure_reason == ExchangeStatus::Ok);
    BOOST_REQUIRE(est.protocol == Protocol::Ss);
    BOOST_REQUIRE(est.computed_at == ComputedAt::InitiatorA);
    BOOST_REQUIRE(clock_domain_same_identity(est.domain, a));

    // The hand-computed exact ToF: 1001/2 A ticks.
    BOOST_REQUIRE_EQUAL(est.tof.num, kTofNum);
    BOOST_REQUIRE_EQUAL(est.tof.den, kTofDen);
    BOOST_REQUIRE(est.tof.valid);

    // The range-validity entries are hard false even on this "success".
    BOOST_TEST(!est.measurement_valid);
    BOOST_TEST(!est.yields_range());
    BOOST_TEST(!est.is_hardware_measurement());
    BOOST_TEST(!est.is_validated_measurement());
    BOOST_TEST(est.execution_mode_is_simulation);

    BOOST_TEST_MESSAGE("protocol SS peer path: " << est.to_string());
}

// ===========================================================================
// 2. DS with A-side wire claims (Final) and B-side LOCAL intervals (t6B is
//    never on the wire) gives the same exact hand-computed ToF.
// ===========================================================================
BOOST_AUTO_TEST_CASE(protocol_time_ds_peer_and_local_paths_match)
{
    const ClockDomain a = domain("dw_a_1e9", kRateA);
    const ClockDomain b = domain("dw_b_998p4", kRateB);

    ClockRatio k;
    BOOST_REQUIRE(ClockRatio::from_nominal_rates(a, b, 1000000000LL, 998400000LL, k));

    const WireTimestampBinding ba =
        binding_for(a, 1, 1, (1ull << 30), 4, TimestampMarker::RmarkerRx);

    const Frame fin = final_frame(0xAAAA, 0x1234, 9, 100000, 100000 + kRA,
                                  100000 + kRA + kDA);
    const PeerTimestampClaim t4 = claim_from(fin, TimestampField::T4A);
    const PeerTimestampClaim t5 = claim_from(fin, TimestampField::T5A);

    ProtocolInterval ra;
    ProtocolInterval da;
    BOOST_REQUIRE(ProtocolInterval::make_peer(t4, claim_from(fin, TimestampField::T1A),
                                              ba, ra) == PeerClaimError::Ok);
    BOOST_REQUIRE(ProtocolInterval::make_peer(t5, t4, ba, da) == PeerClaimError::Ok);

    // B-side local intervals: RB = 1248, DB = 624.
    const ProtocolInterval rb = local_interval(b, kRB, true);
    const ProtocolInterval db = local_interval(b, kDB, false);

    const ProtocolTofEstimate est = compute_protocol_ds_tof(ra, rb, da, db, k);
    BOOST_REQUIRE_MESSAGE(est.completion == ProtocolCompletionStatus::Complete,
                          est.to_string());
    BOOST_REQUIRE(est.estimate_available);
    BOOST_REQUIRE(est.peer_evidence_is_wire_claim);
    BOOST_REQUIRE(est.math_status == TofStatus::Ok);
    BOOST_REQUIRE(est.protocol == Protocol::Ds);
    BOOST_REQUIRE(est.computed_at == ComputedAt::ResponderB);
    BOOST_REQUIRE_EQUAL(est.tof.num, kTofNum);
    BOOST_REQUIRE_EQUAL(est.tof.den, kTofDen);
    BOOST_TEST(!est.measurement_valid);
    BOOST_TEST(!est.yields_range());
    BOOST_TEST(!est.is_hardware_measurement());
    BOOST_TEST(!est.is_validated_measurement());

    // All-LOCAL control for the same physical scenario: the wire path adds
    // provenance, not a different number.
    const ProtocolInterval ra_l = local_interval(a, kRA, true);
    const ProtocolInterval rb_l = local_interval(b, kRB, true);
    const ProtocolInterval da_l = local_interval(a, kDA, false);
    const ProtocolInterval db_l = local_interval(b, kDB, false);
    const ProtocolTofEstimate est_l =
        compute_protocol_ds_tof(ra_l, rb_l, da_l, db_l, k);
    BOOST_REQUIRE(est_l.completion == ProtocolCompletionStatus::Complete);
    BOOST_REQUIRE(est_l.estimate_available);
    BOOST_TEST(!est_l.peer_evidence_is_wire_claim);
    BOOST_REQUIRE_EQUAL(est_l.tof.num, kTofNum);
    BOOST_REQUIRE_EQUAL(est_l.tof.den, kTofDen);
    BOOST_REQUIRE(tof_rational_equals(est.tof, est_l.tof));

    BOOST_TEST_MESSAGE("protocol DS peer/local: " << est.to_string());
}

// ===========================================================================
// 3. `ProtocolInterval::from_local` and the peer path give the SAME exact ToF
//    for the same physical scenario (SS and DS).
// ===========================================================================
BOOST_AUTO_TEST_CASE(protocol_time_local_and_peer_agree_exactly)
{
    const ClockDomain a = domain("dw_a_1e9", kRateA);
    const ClockDomain b = domain("dw_b_998p4", kRateB);

    ClockRatio k;
    BOOST_REQUIRE(ClockRatio::from_nominal_rates(a, b, 1000000000LL, 998400000LL, k));

    // Local SS: RA = 1626 (A), DB = 624 (B).
    const ProtocolTofEstimate ss_local = compute_protocol_ss_tof(
        local_interval(a, kRA, true), local_interval(b, kDB, false), k);
    BOOST_REQUIRE(ss_local.completion == ProtocolCompletionStatus::Complete);
    BOOST_REQUIRE(ss_local.estimate_available);
    BOOST_TEST(!ss_local.peer_evidence_is_wire_claim);
    BOOST_REQUIRE_EQUAL(ss_local.tof.num, kTofNum);
    BOOST_REQUIRE_EQUAL(ss_local.tof.den, kTofDen);

    // The peer path with the same physical numbers (see test 1).
    const WireTimestampBinding ba = binding_for(a);
    const WireTimestampBinding bb = binding_for(b);
    const Frame fin = final_frame(0xAAAA, 0x1234, 9, 100000, 100000 + kRA,
                                  100000 + kRA + kDA);
    ProtocolInterval ra;
    BOOST_REQUIRE(ProtocolInterval::make_peer(claim_from(fin, TimestampField::T4A),
                                              claim_from(fin, TimestampField::T1A), ba,
                                              ra) == PeerClaimError::Ok);
    const Frame rsp = response_frame(0xBBBB, 0x1234, 9, 500000, 500000 + kDB);
    ProtocolInterval db;
    BOOST_REQUIRE(ProtocolInterval::make_peer(claim_from(rsp, TimestampField::T3B),
                                              claim_from(rsp, TimestampField::T2B), bb,
                                              db) == PeerClaimError::Ok);

    const ProtocolTofEstimate ss_peer =
        compute_protocol_ss_tof(ra, db, k);
    BOOST_REQUIRE(ss_peer.completion == ProtocolCompletionStatus::Complete);
    BOOST_REQUIRE(ss_peer.estimate_available);
    BOOST_TEST(ss_peer.peer_evidence_is_wire_claim);

    // Exact rational equality: 1001/2 in both.
    BOOST_REQUIRE(tof_rational_equals(ss_peer.tof, ss_local.tof));
    BOOST_REQUIRE_EQUAL(ss_peer.tof.num, ss_local.tof.num);
    BOOST_REQUIRE_EQUAL(ss_peer.tof.den, ss_local.tof.den);
    BOOST_REQUIRE_EQUAL(ss_peer.tof.num, kTofNum);
    BOOST_REQUIRE_EQUAL(ss_peer.tof.den, kTofDen);
}

// ===========================================================================
// 4. The TYPE-LEVEL separation: the wire interval and the protocol estimate
//    cannot reach the strict M1-A path, and no range entry is ever true.
// ===========================================================================
BOOST_AUTO_TEST_CASE(protocol_time_wire_cannot_become_strict_input)
{
    // A wire protocol interval is NOT an admitted ranging interval...
    static_assert(!std::is_convertible_v<ProtocolInterval, AdmittedRangingInterval>,
                  "ProtocolInterval must not convert to AdmittedRangingInterval");
    static_assert(!std::is_constructible_v<AdmittedRangingInterval, ProtocolInterval>,
                  "there must be no constructor from a wire interval to admitted");
    // ... and the protocol estimate is NOT a strict TofResult.
    static_assert(!std::is_convertible_v<ProtocolTofEstimate, TofResult>,
                  "ProtocolTofEstimate must not convert to TofResult");
    static_assert(!std::is_constructible_v<TofResult, ProtocolTofEstimate>,
                  "there must be no constructor from a protocol estimate to TofResult");

    // The strict entry points do not accept protocol intervals at all.
    static_assert(!std::is_invocable_v<decltype(&compute_ss_tof), const ProtocolInterval&,
                                       const ProtocolInterval&, const ClockRatio&>,
                  "compute_ss_tof must not accept a ProtocolInterval");
    static_assert(!std::is_invocable_v<decltype(&compute_ds_tof), const ProtocolInterval&,
                                       const ProtocolInterval&, const ProtocolInterval&,
                                       const ProtocolInterval&, const ClockRatio&>,
                  "compute_ds_tof must not accept a ProtocolInterval");

    // Positive control: the strict entry points DO accept admitted intervals.
    static_assert(std::is_invocable_v<decltype(&compute_ss_tof),
                                      const AdmittedRangingInterval&,
                                      const AdmittedRangingInterval&, const ClockRatio&>,
                  "control: compute_ss_tof takes admitted intervals");

    // The result type has no range-validating entry that can be true, and a
    // default-constructed estimate is already false everywhere.
    const ProtocolTofEstimate def;
    BOOST_TEST(!def.yields_range());
    BOOST_TEST(!def.measurement_valid);
    BOOST_TEST(!def.is_hardware_measurement());
    BOOST_TEST(!def.is_validated_measurement());
    BOOST_TEST(def.execution_mode_is_simulation);

    // The origin enum is fail-closed (N07).
    BOOST_TEST(protocol_interval_origin_is_known(ProtocolIntervalOrigin::LocalAdmitted));
    BOOST_TEST(protocol_interval_origin_is_known(ProtocolIntervalOrigin::PeerWireClaim));
    BOOST_TEST(!protocol_interval_origin_is_known(
        static_cast<ProtocolIntervalOrigin>(250)));
    BOOST_TEST(std::string(protocol_interval_origin_to_string(
                   static_cast<ProtocolIntervalOrigin>(250))) == "invalid");

    BOOST_TEST(true); // the static assertions are the test
}

// ===========================================================================
// 5. Binding refusals, each reported as BindingNotWellFormed.
// ===========================================================================
BOOST_AUTO_TEST_CASE(protocol_time_binding_refusals)
{
    const ClockDomain a = domain("dw_a_1e9", kRateA);
    const WireTimestampBinding good = binding_for(a);
    std::string why;

    // Positive control.
    BOOST_REQUIRE(wire_timestamp_binding_is_well_formed(good, why));
    BOOST_TEST(sequence_modulus_is_supported(4));
    BOOST_TEST(sequence_modulus_is_supported(256));
    BOOST_TEST(!sequence_modulus_is_supported(5));

    const PeerTimestampClaim later = hand_claim(TimestampField::T4A, 5000, 0x55);
    const PeerTimestampClaim earlier = hand_claim(TimestampField::T1A, 3000, 0x55);

    struct Case {
        const char* what;
        WireTimestampBinding b;
    };
    Case cases[] = {
        { "empty unit_convention", [&] { WireTimestampBinding x = good; x.unit_convention.clear(); return x; }() },
        { "empty calibration_convention_id", [&] { WireTimestampBinding x = good; x.calibration_convention_id.clear(); return x; }() },
        { "max_interval_ticks == 0", [&] { WireTimestampBinding x = good; x.max_interval_ticks = 0; return x; }() },
        { "session_generation == 0", [&] { WireTimestampBinding x = good; x.session_generation = 0; return x; }() },
        { "binding_generation == 0", [&] { WireTimestampBinding x = good; x.binding_generation = 0; return x; }() },
        { "unsupported sequence_modulus", [&] { WireTimestampBinding x = good; x.sequence_modulus = 5; return x; }() },
        { "unknown peer_marker", [&] { WireTimestampBinding x = good; x.peer_marker = static_cast<TimestampMarker>(250); return x; }() },
        { "invalid peer_domain", [&] { WireTimestampBinding x = good; x.peer_domain.name.clear(); return x; }() },
        // 40-bit domain: half period is 2^39; anything above is ambiguous.
        { "max_interval_ticks above half period",
          [&] { WireTimestampBinding x = good; x.max_interval_ticks = (1ull << 39) + 1ull; return x; }() },
    };

    for (const Case& c : cases) {
        std::string w;
        BOOST_TEST_MESSAGE("binding case: " << c.what);
        BOOST_TEST(!wire_timestamp_binding_is_well_formed(c.b, w));
        BOOST_TEST(!w.empty());
        ProtocolInterval out;
        BOOST_TEST(ProtocolInterval::make_peer(later, earlier, c.b, out) ==
                   PeerClaimError::BindingNotWellFormed);
    }

    // Exactly half the period IS a legal bound (the interval policy accepts
    // d == P/2), so the binding must be accepted.
    WireTimestampBinding at_half = good;
    at_half.max_interval_ticks = (1ull << 39);
    BOOST_TEST(wire_timestamp_binding_is_well_formed(at_half, why));
}

// ===========================================================================
// 6. Claim refusals, each reported with its specific PeerClaimError.
// ===========================================================================
BOOST_AUTO_TEST_CASE(protocol_time_claim_refusals)
{
    const ClockDomain a = domain("dw_a_1e9", kRateA);
    const WireTimestampBinding ba = binding_for(a);
    ProtocolInterval out;

    // Not bound: a hand-built claim that never said where it came from.
    {
        PeerTimestampClaim later = hand_claim(TimestampField::T4A, 5000, 0x55, false);
        PeerTimestampClaim earlier = hand_claim(TimestampField::T1A, 3000, 0x55, false);
        BOOST_TEST(ProtocolInterval::make_peer(later, earlier, ba, out) ==
                   PeerClaimError::ClaimNotBound);
    }

    // Unknown field (out of the enum's domain).
    {
        PeerTimestampClaim later = hand_claim(TimestampField::Count, 5000, 0x55);
        PeerTimestampClaim earlier = hand_claim(TimestampField::T1A, 3000, 0x55);
        BOOST_TEST(ProtocolInterval::make_peer(later, earlier, ba, out) ==
                   PeerClaimError::UnknownField);
    }

    // Claims from different messages.
    {
        PeerTimestampClaim later = hand_claim(TimestampField::T4A, 5000, 0x55);
        PeerTimestampClaim earlier = hand_claim(TimestampField::T1A, 3000, 0x56);
        BOOST_TEST(ProtocolInterval::make_peer(later, earlier, ba, out) ==
                   PeerClaimError::NotSameMessage);
    }

    // Raw ticks outside the declared 40-bit domain.
    {
        PeerTimestampClaim later = hand_claim(TimestampField::T4A, (1ull << 40), 0x55);
        PeerTimestampClaim earlier = hand_claim(TimestampField::T1A, (1ull << 40) - 1ull, 0x55);
        BOOST_TEST(ProtocolInterval::make_peer(later, earlier, ba, out) ==
                   PeerClaimError::RawOutOfDomain);
        BOOST_TEST(!a.ticks_in_range(static_cast<int64_t>(1ull << 40)));
    }

    // Reversed interval on a NON-wrapping (monotonic) domain is not formable.
    {
        const ClockDomain mono = domain("dw_mono", kRateA, kEpoch, 0u);
        const WireTimestampBinding bm = binding_for(mono);
        PeerTimestampClaim later = hand_claim(TimestampField::T4A, 100, 0x55);
        PeerTimestampClaim earlier = hand_claim(TimestampField::T1A, 200, 0x55);
        BOOST_TEST(ProtocolInterval::make_peer(later, earlier, bm, out) ==
                   PeerClaimError::IntervalNotFormable);
    }

    // A formable interval longer than the declared session bound.
    {
        WireTimestampBinding small = binding_for(a);
        small.max_interval_ticks = 999;
        PeerTimestampClaim later = hand_claim(TimestampField::T4A, 1000, 0x55);
        PeerTimestampClaim earlier = hand_claim(TimestampField::T1A, 0, 0x55);
        BOOST_TEST(ProtocolInterval::make_peer(later, earlier, small, out) ==
                   PeerClaimError::IntervalTooLong);
    }

    // Out-of-domain sequence modulus is refused before any arithmetic.
    {
        WireTimestampBinding bad_mod = binding_for(a);
        bad_mod.sequence_modulus = 7;
        PeerTimestampClaim later = hand_claim(TimestampField::T4A, 5000, 0x55);
        PeerTimestampClaim earlier = hand_claim(TimestampField::T1A, 3000, 0x55);
        BOOST_TEST(ProtocolInterval::make_peer(later, earlier, bad_mod, out) ==
                   PeerClaimError::BindingNotWellFormed);
    }
}

// ===========================================================================
// 7. An interval EXACTLY half a period is accepted (existing policy).
// ===========================================================================
BOOST_AUTO_TEST_CASE(protocol_time_half_period_is_accepted)
{
    const ClockDomain a = domain("dw_a_40", kRateA, kEpoch, 40u);
    WireTimestampBinding b = binding_for(a);
    b.max_interval_ticks = (1ull << 39); // == P/2

    // later = 0, earlier = 2^39: the modular difference wraps and is exactly
    // P/2.  raw_tick_delta accepts d == P/2 by design, so make_peer must too.
    PeerTimestampClaim later = hand_claim(TimestampField::T4A, 0, 0x77);
    PeerTimestampClaim earlier = hand_claim(TimestampField::T1A, (1ull << 39), 0x77);

    ProtocolInterval out;
    BOOST_REQUIRE(ProtocolInterval::make_peer(later, earlier, b, out) == PeerClaimError::Ok);
    BOOST_REQUIRE(out.is_valid());
    BOOST_REQUIRE(out.is_peer_wire_claim());
    BOOST_REQUIRE(out.wrapped());
    int64_t n = 0;
    int64_t d = 0;
    BOOST_REQUIRE(out.exact_ratio(n, d));
    BOOST_REQUIRE_EQUAL(n, static_cast<int64_t>(1ull << 39));
    BOOST_REQUIRE_EQUAL(d, 1);

    // One tick beyond the half period is ambiguous and refused as not formable
    // (the modular difference is the thing that is ambiguous; the binding's max
    // is itself capped at P/2, so it is raw_tick_delta that must reject this).
    PeerTimestampClaim later2 = hand_claim(TimestampField::T4A, (1ull << 39) + 1ull, 0x78);
    PeerTimestampClaim earlier2 = hand_claim(TimestampField::T1A, 0, 0x78);
    BOOST_TEST(ProtocolInterval::make_peer(later2, earlier2, b, out) ==
               PeerClaimError::IntervalNotFormable);
}

// ===========================================================================
// 8. Empty / garbage inputs produce a FAILED estimate and never throw.
// ===========================================================================
BOOST_AUTO_TEST_CASE(protocol_time_empty_inputs_fail_closed)
{
    const ClockDomain a = domain("dw_a_1e9", kRateA);
    const ClockDomain b = domain("dw_b_998p4", kRateB);
    ClockRatio k;
    BOOST_REQUIRE(ClockRatio::from_nominal_rates(a, b, 1000000000LL, 998400000LL, k));

    const ProtocolInterval invalid; // default-constructed: never valid

    const ProtocolTofEstimate ss = compute_protocol_ss_tof(invalid, invalid, k);
    BOOST_REQUIRE(ss.completion == ProtocolCompletionStatus::Failed);
    BOOST_TEST(!ss.estimate_available);
    BOOST_TEST(ss.math_status == TofStatus::InvalidInput);
    BOOST_TEST(ss.failure_reason == ExchangeStatus::InvalidTimeDomain);
    BOOST_TEST(!ss.yields_range());

    const ProtocolTofEstimate ds =
        compute_protocol_ds_tof(invalid, invalid, invalid, invalid, k);
    BOOST_REQUIRE(ds.completion == ProtocolCompletionStatus::Failed);
    BOOST_TEST(!ds.estimate_available);
    BOOST_TEST(ds.math_status == TofStatus::InvalidInput);

    // A ratio that was never supplied is refused, not defaulted to k = 1.
    const ClockRatio missing;
    const ProtocolInterval ra_l = local_interval(a, kRA, true);
    const ProtocolInterval db_l = local_interval(b, kDB, false);
    const ProtocolTofEstimate no_ratio = compute_protocol_ss_tof(ra_l, db_l, missing);
    BOOST_REQUIRE(no_ratio.completion == ProtocolCompletionStatus::Failed);
    BOOST_TEST(!no_ratio.estimate_available);
    BOOST_TEST(no_ratio.math_status == TofStatus::ClockRatioMissing);
    BOOST_TEST(no_ratio.failure_reason == ExchangeStatus::ClockEstimateInvalid);

    // The completion enum is fail-closed too (N07).
    BOOST_TEST(protocol_completion_status_is_known(ProtocolCompletionStatus::Complete));
    BOOST_TEST(protocol_completion_status_is_known(ProtocolCompletionStatus::Failed));
    BOOST_TEST(!protocol_completion_status_is_known(
        static_cast<ProtocolCompletionStatus>(250)));
}

// ===========================================================================
// 9. A negative protocol ToF keeps its sign and its exact value; it is a
//    failure, never clamped to zero.
// ===========================================================================
BOOST_AUTO_TEST_CASE(protocol_time_negative_tof_is_retained)
{
    const ClockDomain a = domain("dw_a_1e9", kRateA);
    const ClockDomain b = domain("dw_b_998p4", kRateB);
    ClockRatio k;
    BOOST_REQUIRE(ClockRatio::from_nominal_rates(a, b, 1000000000LL, 998400000LL, k));

    const WireTimestampBinding ba = binding_for(a);
    const WireTimestampBinding bb = binding_for(b);

    // RA = 1000 A, DB = 1248 B -> k*DB = 1250 -> (1000-1250)/2 = -125 exactly.
    PeerTimestampClaim ra_later = hand_claim(TimestampField::T4A, 5000 + 1000, 0x99);
    PeerTimestampClaim ra_earlier = hand_claim(TimestampField::T1A, 5000, 0x99);
    ProtocolInterval ra;
    BOOST_REQUIRE(ProtocolInterval::make_peer(ra_later, ra_earlier, ba, ra) ==
                  PeerClaimError::Ok);

    PeerTimestampClaim db_later = hand_claim(TimestampField::T3B, 7000 + 1248, 0x9A);
    PeerTimestampClaim db_earlier = hand_claim(TimestampField::T2B, 7000, 0x9A);
    ProtocolInterval db;
    BOOST_REQUIRE(ProtocolInterval::make_peer(db_later, db_earlier, bb, db) ==
                  PeerClaimError::Ok);

    const ProtocolTofEstimate est = compute_protocol_ss_tof(ra, db, k);
    BOOST_REQUIRE(est.completion == ProtocolCompletionStatus::Failed);
    BOOST_TEST(!est.estimate_available);
    BOOST_TEST(est.math_status == TofStatus::NegativeTof);
    BOOST_TEST(est.failure_reason == ExchangeStatus::NegativeTof);
    BOOST_REQUIRE(est.tof.valid);
    BOOST_REQUIRE_EQUAL(est.tof.num, -125);
    BOOST_REQUIRE_EQUAL(est.tof.den, 1);
    BOOST_TEST(est.tof.negative());
    BOOST_TEST(!est.yields_range());
    BOOST_TEST(!est.measurement_valid);

    // Python check: (1000 - (625/624)*1248)/2 = (1000 - 1250)/2 = -125.
    BOOST_TEST_MESSAGE("protocol SS negative: " << est.to_string());
}

// ===========================================================================
// 10. The protocol path still performs domain / epoch / validity-window checks
//     even though the inputs are wire claims.
// ===========================================================================
BOOST_AUTO_TEST_CASE(protocol_time_domain_and_window_checks_still_apply)
{
    const ClockDomain a = domain("dw_a_1e9", kRateA, kEpoch);
    const ClockDomain a_next = domain("dw_a_1e9", kRateA, kEpoch + 1u);
    const ClockDomain b = domain("dw_b_998p4", kRateB, kEpoch);
    const ClockDomain other = domain("dw_other", kRateA, kEpoch);

    ClockRatio k;
    BOOST_REQUIRE(ClockRatio::from_nominal_rates(a, b, 1000000000LL, 998400000LL, k));

    // An A-side interval in a later epoch is an epoch mismatch.
    {
        const ProtocolInterval ra = local_interval(a_next, kRA, true);
        const ProtocolInterval db = local_interval(b, kDB, false);
        const ProtocolTofEstimate est = compute_protocol_ss_tof(ra, db, k);
        BOOST_REQUIRE(est.completion == ProtocolCompletionStatus::Failed);
        BOOST_TEST(est.math_status == TofStatus::ClockRatioEpochMismatch);
        BOOST_TEST(est.failure_reason == ExchangeStatus::InvalidTimeDomain);
    }

    // A differently-named A counter is a domain mismatch.
    {
        const ProtocolInterval ra = local_interval(other, kRA, true);
        const ProtocolInterval db = local_interval(b, kDB, false);
        const ProtocolTofEstimate est = compute_protocol_ss_tof(ra, db, k);
        BOOST_REQUIRE(est.completion == ProtocolCompletionStatus::Failed);
        BOOST_TEST(est.math_status == TofStatus::ClockRatioDomainMismatch);
    }

    // A validity window that does not cover the A-domain interval is refused.
    {
        ClockRatio windowed;
        BOOST_REQUIRE(ClockRatio::from_nominal_rates(a, b, 1000000000LL, 998400000LL,
                                                     windowed));
        BOOST_REQUIRE(windowed.set_validity_window(0, 100));
        const ProtocolInterval ra = local_interval(a, kRA, true);
        const ProtocolInterval db = local_interval(b, kDB, false);
        const ProtocolTofEstimate est = compute_protocol_ss_tof(ra, db, windowed);
        BOOST_REQUIRE(est.completion == ProtocolCompletionStatus::Failed);
        BOOST_TEST(est.math_status == TofStatus::ClockRatioNotValidAtTime);
        BOOST_TEST(est.failure_reason == ExchangeStatus::ClockEstimateInvalid);
    }
}

BOOST_AUTO_TEST_SUITE_END()
