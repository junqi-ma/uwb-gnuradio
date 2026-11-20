/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * M1-B end-to-end QA (Agent D): two real `EndpointCore`s driven through one
 * real `FakeTwrLink` by `TwoEndpointDriver`.
 *
 * This is where the causal, receive-driven contract is actually exercised: a
 * frame is only produced because a valid frame was received, and the estimate
 * is compared against an independently (Python `fractions`) derived value
 * recorded in testdata/twr/m1b/.
 *
 * B-ID coverage that lives in this file: B01 (four direction cells), B02
 * (k>1 / k<1 and unequal DS replies, with the exact frame fields), B03 (each
 * frame dropped), the causal half of B16, and the determinism half of B18.
 */

#include <gnuradio/uwb/uwb_twr_core.h>
#include <gnuradio/uwb/uwb_twr_fake_link.h>
#include <gnuradio/uwb/uwb_twr_frame.h>

#include "twr_m1b_test_support.h"

#include <boost/test/unit_test.hpp>

#include <cstdint>
#include <ostream>
#include <string>

using namespace gr::uwb::twr;
using namespace gr::uwb::twr::m1b_test;

// Printable enums for Boost.Test failure reports (ADL, no production change).
namespace gr {
namespace uwb {
namespace twr {
inline std::ostream& operator<<(std::ostream& os, ExchangeStatus s)
{
    return os << exchange_status_to_string(s);
}
inline std::ostream& operator<<(std::ostream& os, ProtocolCompletionStatus s)
{
    return os << protocol_completion_status_to_string(s);
}
inline std::ostream& operator<<(std::ostream& os, Protocol p)
{
    return os << protocol_to_string(p);
}
inline std::ostream& operator<<(std::ostream& os, FrameType t)
{
    return os << frame_type_to_string(t);
}
inline std::ostream& operator<<(std::ostream& os, Role r) { return os << role_to_string(r); }
} // namespace twr
} // namespace uwb
} // namespace gr

namespace {

// ---------------------------------------------------------------------------
// Scenario construction
// ---------------------------------------------------------------------------

struct FixedEnv {
    ClockDomain dom;
    FrameProfile prof;
    CalibrationStamp cal;
};

FixedEnv make_fixed_env()
{
    FixedEnv e;
    e.dom = make_domain("x410_dev", 1.0e9, 1, 40);
    e.prof = make_profile(1.0e9);
    e.cal = make_calibration("cal1", 1, 0, (1LL << 39));
    return e;
}

CoreConfig core_cfg(const std::string& id,
                    Role role,
                    Protocol proto,
                    const ClockDomain& local,
                    const ClockDomain& peer,
                    double unit_hz,
                    const CalibrationStamp& cal,
                    uint16_t self_addr,
                    uint16_t peer_addr,
                    const ClockRatio& k,
                    uint16_t session_id = 7,
                    uint16_t modulus = 4,
                    int64_t exchange_timeout = 100000)
{
    CoreConfig c;
    c.endpoint_id = id;
    c.protocol = proto;
    c.role = role;
    c.pan_id = 0x1234;
    c.local_address = self_addr;
    c.peer_address = peer_addr;
    c.session_id = session_id;
    c.session_generation = 1;
    c.sequence_modulus = modulus;
    c.initial_sequence = 0;
    c.local_domain = local;
    c.peer_binding = make_binding(peer, 1, 1, 1000000, modulus);
    c.ratio = k;
    c.frame_profile = make_profile(unit_hz);
    c.local_calibration = cal;
    c.exchange_timeout_ticks = exchange_timeout;
    c.evidence_wait_ticks = exchange_timeout;
    c.result_queue_capacity = 8;
    c.max_in_flight = 1;
    return c;
}

// One physical clock shared by both logical endpoints (the one-X410 case).
DriverConfig same_clock_driver(Protocol proto,
                               uint8_t initiator,
                               double distance_m,
                               int64_t respond_turn,
                               int64_t final_turn,
                               int64_t poll_offset)
{
    const FixedEnv env = make_fixed_env();
    ClockRatio k;
    ClockRatio::unity_same_clock(env.dom, k);
    const Role role_a = (initiator == kEndpointA) ? Role::Initiator : Role::Responder;
    const Role role_b = (initiator == kEndpointA) ? Role::Responder : Role::Initiator;

    DriverConfig d;
    d.cfg_a = core_cfg("A", role_a, proto, env.dom, env.dom, 1.0e9, env.cal, 0x0101, 0x0202, k);
    d.cfg_b = core_cfg("B", role_b, proto, env.dom, env.dom, 1.0e9, env.cal, 0x0202, 0x0101, k);
    d.link.domain_a = env.dom;
    d.link.domain_b = env.dom;
    d.link.distance_m = distance_m;
    d.link.rx_queue_capacity = 16;
    d.link.seed = 1;
    d.poll_offset_ticks = poll_offset;
    d.respond_turnaround_ticks = respond_turn;
    d.final_turnaround_ticks = final_turn;
    return d;
}

// Two independent nominal clock domains.  A is always the physical/initiator
// side in the cross-clock cases; `prop_a_ticks` fixes the ground-truth distance
// so both propagation delays are whole numbers.
DriverConfig diff_rate_driver(Protocol proto,
                              double fA,
                              double fB,
                              int64_t prop_a_ticks,
                              int64_t respond_turn,
                              int64_t final_turn,
                              int64_t poll_offset)
{
    const ClockDomain da = make_domain("devA", fA, 1, 40);
    const ClockDomain db = make_domain("devB", fB, 1, 40);
    const CalibrationStamp cal = make_calibration("cal1", 1, 0, (1LL << 39));
    ClockRatio k;
    ClockRatio::from_nominal_rates(da, db, static_cast<int64_t>(fA), static_cast<int64_t>(fB), k);

    DriverConfig d;
    d.cfg_a = core_cfg("A", Role::Initiator, proto, da, db, fA, cal, 0x0101, 0x0202, k, 7, 4,
                       1000000);
    d.cfg_b = core_cfg("B", Role::Responder, proto, db, da, fB, cal, 0x0202, 0x0101, k, 7, 4,
                       1000000);
    d.link.domain_a = da;
    d.link.domain_b = db;
    d.link.distance_m = static_cast<double>(prop_a_ticks) * kFakeLinkSpeedOfLightMps / fA;
    d.link.rx_queue_capacity = 16;
    d.link.seed = 1;
    d.poll_offset_ticks = poll_offset;
    d.respond_turnaround_ticks = respond_turn;
    d.final_turnaround_ticks = final_turn;
    return d;
}

const TerminalRecord* last_terminal(const TwoEndpointDriver& d, uint8_t p)
{
    const std::vector<TerminalRecord>& v = d.terminals(p);
    return v.empty() ? nullptr : &v.back();
}

void assert_conservation(const TwoEndpointDriver& d, uint8_t p)
{
    const CoreCounters& c = d.core(p).counters();
    BOOST_TEST(c.accepted_exchanges == c.terminal_results + d.core(p).in_flight());
}

void assert_not_a_range(const ProtocolTofEstimate& e)
{
    BOOST_TEST(e.measurement_valid == false);
    BOOST_TEST(e.execution_mode_is_simulation == true);
    BOOST_TEST(!e.yields_range());
    BOOST_TEST(!e.is_hardware_measurement());
    BOOST_TEST(!e.is_validated_measurement());
}

} // namespace

// ===========================================================================
// B01: four cells -- SS/DS x both initiator directions
// ===========================================================================

BOOST_AUTO_TEST_CASE(b01_four_cells_ss_ds_both_initiator_directions)
{
    for (Protocol proto : { Protocol::Ss, Protocol::Ds }) {
        for (uint8_t initiator : { kEndpointA, kEndpointB }) {
            // distance 15 m -> 50 A ticks at 1 GHz; turnarounds 500 / 300.
            DriverConfig dc = same_clock_driver(proto, initiator, 15.0, 500, 300, 1000);
            TwoEndpointDriver drv;
            std::string why;
            BOOST_TEST(drv.configure(dc, why));
            BOOST_TEST(drv.begin(initiator, 1000, 1));
            BOOST_TEST(drv.run_until_idle(2000, why));

            const uint8_t responder = static_cast<uint8_t>(1 - initiator);
            const uint8_t est_phys =
                (proto == Protocol::Ss) ? initiator : responder;

            // Both FSMs actually ran.
            BOOST_TEST(drv.core(kEndpointA).counters().accepted_exchanges == 1u);
            BOOST_TEST(drv.core(kEndpointB).counters().accepted_exchanges == 1u);
            BOOST_TEST(drv.core(kEndpointA).counters().terminal_results == 1u);
            BOOST_TEST(drv.core(kEndpointB).counters().terminal_results == 1u);

            const TerminalRecord* t_init = last_terminal(drv, initiator);
            const TerminalRecord* t_resp = last_terminal(drv, responder);
            BOOST_TEST(t_init != nullptr);
            BOOST_TEST(t_resp != nullptr);
            if (t_init == nullptr || t_resp == nullptr)
                continue;
            BOOST_TEST(t_init->status == ExchangeStatus::Ok);
            BOOST_TEST(t_resp->status == ExchangeStatus::Ok);

            // SS lands ONLY at the initiator; DS ONLY at the responder.
            const ProtocolTofEstimate& est = (est_phys == kEndpointA)
                                                 ? drv.terminals(kEndpointA).back().result
                                                 : drv.terminals(kEndpointB).back().result;
            const ProtocolTofEstimate& other = (est_phys == kEndpointA)
                                                   ? drv.terminals(kEndpointB).back().result
                                                   : drv.terminals(kEndpointA).back().result;

            BOOST_TEST(est.estimate_available);
            BOOST_TEST(est.tof.num == 50);
            BOOST_TEST(est.tof.den == 1);
            assert_not_a_range(est);

            BOOST_TEST(!other.estimate_available);
            BOOST_TEST(!other.tof.valid); // the estimate is never copied across
            BOOST_TEST(other.completion == ProtocolCompletionStatus::Complete);

            // Transmit multiplicity: the initiator sends Poll (+ Final for DS).
            const unsigned expect_initiator_tx = (proto == Protocol::Ds) ? 2u : 1u;
            BOOST_TEST(drv.core(initiator).counters().tx_submitted == expect_initiator_tx);
            BOOST_TEST(drv.core(responder).counters().tx_submitted == 1u);

            assert_conservation(drv, kEndpointA);
            assert_conservation(drv, kEndpointB);
        }
    }
}

// ===========================================================================
// B02: independent clock domains -- k > 1 and k < 1
// ===========================================================================

BOOST_AUTO_TEST_CASE(b02_ss_k_greater_than_one)
{
    // fA = 1e9, fB = 0.8e9 -> k = 5/4.  propA = 250, propB = 200.
    // t1=100000 t2=80200 t3=84200 t4=105500 t5=106200 t6=85160.
    DriverConfig dc = diff_rate_driver(Protocol::Ss, 1.0e9, 0.8e9, 250, 4000, 700, 100000);
    TwoEndpointDriver drv;
    std::string why;
    BOOST_TEST(drv.configure(dc, why));
    BOOST_TEST(drv.begin(kEndpointA, 0, 1));
    BOOST_TEST(drv.run_until_idle(2000, why));

    const TerminalRecord* t = last_terminal(drv, kEndpointA);
    BOOST_TEST(t != nullptr);
    if (t != nullptr) {
        BOOST_TEST(t->status == ExchangeStatus::Ok);
        BOOST_TEST(t->result.estimate_available);
        BOOST_TEST(t->result.tof.num == 250);
        BOOST_TEST(t->result.tof.den == 1);
        assert_not_a_range(t->result);
    }
    const TerminalRecord* tb = last_terminal(drv, kEndpointB);
    BOOST_TEST(tb != nullptr);
    if (tb != nullptr)
        BOOST_TEST(!tb->result.estimate_available);

    // Exact frame fields, independently recomputed in Python.
    BOOST_TEST(drv.sent_frames(kEndpointB).size() == 1u);
    if (!drv.sent_frames(kEndpointB).empty()) {
        const Frame& resp = drv.sent_frames(kEndpointB)[0];
        BOOST_TEST(resp.t2B() == 80200u);
        BOOST_TEST(resp.t3B() == 84200u);
    }
    assert_conservation(drv, kEndpointA);
    assert_conservation(drv, kEndpointB);
}

BOOST_AUTO_TEST_CASE(b02_ds_k_greater_than_one)
{
    // Same k = 5/4, but DS: the responder owns the estimate.
    DriverConfig dc = diff_rate_driver(Protocol::Ds, 1.0e9, 0.8e9, 250, 4000, 700, 100000);
    TwoEndpointDriver drv;
    std::string why;
    BOOST_TEST(drv.configure(dc, why));
    BOOST_TEST(drv.begin(kEndpointA, 0, 1));
    BOOST_TEST(drv.run_until_idle(2000, why));

    const TerminalRecord* t = last_terminal(drv, kEndpointB);
    BOOST_TEST(t != nullptr);
    if (t != nullptr) {
        BOOST_TEST(t->status == ExchangeStatus::Ok);
        BOOST_TEST(t->result.estimate_available);
        BOOST_TEST(t->result.tof.num == 250);
        BOOST_TEST(t->result.tof.den == 1);
        assert_not_a_range(t->result);
    }
    const TerminalRecord* ta = last_terminal(drv, kEndpointA);
    BOOST_TEST(ta != nullptr);
    if (ta != nullptr)
        BOOST_TEST(!ta->result.estimate_available);

    // B's Response carries t2B/t3B; A's Final carries t1A/t4A/t5A.
    BOOST_TEST(drv.sent_frames(kEndpointB).size() == 1u);
    if (!drv.sent_frames(kEndpointB).empty()) {
        const Frame& resp = drv.sent_frames(kEndpointB)[0];
        BOOST_TEST(resp.t2B() == 80200u);
        BOOST_TEST(resp.t3B() == 84200u);
    }
    BOOST_TEST(drv.sent_frames(kEndpointA).size() == 2u);
    if (drv.sent_frames(kEndpointA).size() == 2u) {
        const Frame& fin = drv.sent_frames(kEndpointA)[1];
        BOOST_TEST(fin.function_code == FrameType::Final);
        BOOST_TEST(fin.t1A() == 100000u);
        BOOST_TEST(fin.t4A() == 105500u);
        BOOST_TEST(fin.t5A() == 106200u);
    }
    assert_conservation(drv, kEndpointA);
    assert_conservation(drv, kEndpointB);
}

BOOST_AUTO_TEST_CASE(b02_ss_k_less_than_one)
{
    // fA = 0.8e9, fB = 1e9 -> k = 4/5.  propA = 200, propB = 250.
    // t1=100000 t2=125250 t3=130250 t4=104400 t5=105200 t6=131750.
    DriverConfig dc = diff_rate_driver(Protocol::Ss, 0.8e9, 1.0e9, 200, 5000, 800, 100000);
    TwoEndpointDriver drv;
    std::string why;
    BOOST_TEST(drv.configure(dc, why));
    BOOST_TEST(drv.begin(kEndpointA, 0, 1));
    BOOST_TEST(drv.run_until_idle(2000, why));

    const TerminalRecord* t = last_terminal(drv, kEndpointA);
    BOOST_TEST(t != nullptr);
    if (t != nullptr) {
        BOOST_TEST(t->result.estimate_available);
        BOOST_TEST(t->result.tof.num == 200);
        BOOST_TEST(t->result.tof.den == 1);
    }
    BOOST_TEST(drv.sent_frames(kEndpointB).size() == 1u);
    if (!drv.sent_frames(kEndpointB).empty()) {
        const Frame& resp = drv.sent_frames(kEndpointB)[0];
        BOOST_TEST(resp.t2B() == 125250u);
        BOOST_TEST(resp.t3B() == 130250u);
    }
    assert_conservation(drv, kEndpointA);
    assert_conservation(drv, kEndpointB);
}

BOOST_AUTO_TEST_CASE(b02_ds_unequal_reply_intervals_still_recover_the_same_tof)
{
    // Same clock, DS, with DA != DB: respond = 500 (DB), final = 1300 (DA).
    // t1=2000 t2=2050 t3=2550 t4=2600 t5=3900 t6=3950
    // RA=600 DA=1300 RB=1400 DB=500 -> ToF = (600*1400 - 1300*500)/(600+1400+1300+500) = 50.
    DriverConfig dc = same_clock_driver(Protocol::Ds, kEndpointA, 15.0, 500, 1300, 1000);
    TwoEndpointDriver drv;
    std::string why;
    BOOST_TEST(drv.configure(dc, why));
    BOOST_TEST(drv.begin(kEndpointA, 1000, 1));
    BOOST_TEST(drv.run_until_idle(2000, why));

    BOOST_TEST(drv.sent_frames(kEndpointA).size() == 2u);
    BOOST_TEST(drv.sent_frames(kEndpointB).size() == 1u);
    if (drv.sent_frames(kEndpointA).size() == 2u) {
        const Frame& fin = drv.sent_frames(kEndpointA)[1];
        BOOST_TEST(fin.t1A() == 2000u);
        BOOST_TEST(fin.t4A() == 2600u);
        BOOST_TEST(fin.t5A() == 3900u);
    }
    if (!drv.sent_frames(kEndpointB).empty()) {
        const Frame& resp = drv.sent_frames(kEndpointB)[0];
        BOOST_TEST(resp.t2B() == 2050u);
        BOOST_TEST(resp.t3B() == 2550u);
    }

    const TerminalRecord* t = last_terminal(drv, kEndpointB);
    BOOST_TEST(t != nullptr);
    if (t != nullptr) {
        BOOST_TEST(t->result.estimate_available);
        BOOST_TEST(t->result.tof.num == 50);
        BOOST_TEST(t->result.tof.den == 1);
    }
    assert_conservation(drv, kEndpointA);
    assert_conservation(drv, kEndpointB);
}

// ===========================================================================
// B03: drop each frame in turn
// ===========================================================================

BOOST_AUTO_TEST_CASE(b03_dropped_poll_leaves_the_responder_silent)
{
    DriverConfig dc = same_clock_driver(Protocol::Ss, kEndpointA, 15.0, 500, 300, 1000);
    dc.drop_submits.push_back({ kEndpointA, 1u }); // A's Poll never arrives
    TwoEndpointDriver drv;
    std::string why;
    BOOST_TEST(drv.configure(dc, why));
    BOOST_TEST(drv.begin(kEndpointA, 1000, 1));
    BOOST_TEST(drv.run_until_idle(2000, why));

    const TerminalRecord* ta = last_terminal(drv, kEndpointA);
    BOOST_TEST(ta != nullptr);
    if (ta != nullptr)
        BOOST_TEST(ta->status == ExchangeStatus::ProtocolTimeout);

    // The passive end never fabricates a request or a reply.
    BOOST_TEST(drv.core(kEndpointB).counters().accepted_exchanges == 0u);
    BOOST_TEST(drv.core(kEndpointB).counters().tx_prepared == 0u);
    BOOST_TEST(drv.core(kEndpointB).counters().tx_submitted == 0u);
    BOOST_TEST(drv.sent_frames(kEndpointB).empty());
    assert_conservation(drv, kEndpointA);
    assert_conservation(drv, kEndpointB);
}

BOOST_AUTO_TEST_CASE(b03_dropped_response_ss_responder_completes_initiator_times_out)
{
    DriverConfig dc = same_clock_driver(Protocol::Ss, kEndpointA, 15.0, 500, 300, 1000);
    dc.drop_submits.push_back({ kEndpointB, 1u }); // B's Response never arrives

    TwoEndpointDriver drv;
    std::string why;
    BOOST_TEST(drv.configure(dc, why));
    BOOST_TEST(drv.begin(kEndpointA, 1000, 1));
    BOOST_TEST(drv.run_until_idle(2000, why));

    const TerminalRecord* ta = last_terminal(drv, kEndpointA);
    const TerminalRecord* tb = last_terminal(drv, kEndpointB);
    BOOST_TEST(ta != nullptr);
    BOOST_TEST(tb != nullptr);
    if (ta != nullptr)
        BOOST_TEST(ta->status == ExchangeStatus::ProtocolTimeout);
    if (tb != nullptr) {
        BOOST_TEST(tb->status == ExchangeStatus::Ok); // local send succeeded
        BOOST_TEST(!tb->result.estimate_available);
    }
    assert_conservation(drv, kEndpointA);
    assert_conservation(drv, kEndpointB);
}

BOOST_AUTO_TEST_CASE(b03_dropped_response_ds_never_produces_a_final)
{
    DriverConfig dc = same_clock_driver(Protocol::Ds, kEndpointA, 15.0, 500, 300, 1000);
    dc.drop_submits.push_back({ kEndpointB, 1u });
    TwoEndpointDriver drv;
    std::string why;
    BOOST_TEST(drv.configure(dc, why));
    BOOST_TEST(drv.begin(kEndpointA, 1000, 1));
    BOOST_TEST(drv.run_until_idle(2000, why));

    // The initiator never received a Response, so the dependent Final was never built.
    BOOST_TEST(drv.core(kEndpointA).counters().tx_submitted == 1u);
    BOOST_TEST(drv.sent_frames(kEndpointA).size() == 1u);
    const TerminalRecord* ta = last_terminal(drv, kEndpointA);
    const TerminalRecord* tb = last_terminal(drv, kEndpointB);
    if (ta != nullptr)
        BOOST_TEST(ta->status == ExchangeStatus::ProtocolTimeout);
    if (tb != nullptr)
        BOOST_TEST(tb->status == ExchangeStatus::ProtocolTimeout);
    BOOST_TEST(ta != nullptr);
    BOOST_TEST(tb != nullptr);
    assert_conservation(drv, kEndpointA);
    assert_conservation(drv, kEndpointB);
}

BOOST_AUTO_TEST_CASE(b03_dropped_final_ds_responder_times_out)
{
    DriverConfig dc = same_clock_driver(Protocol::Ds, kEndpointA, 15.0, 500, 300, 1000);
    dc.drop_submits.push_back({ kEndpointA, 2u }); // A's Final never arrives
    TwoEndpointDriver drv;
    std::string why;
    BOOST_TEST(drv.configure(dc, why));
    BOOST_TEST(drv.begin(kEndpointA, 1000, 1));
    BOOST_TEST(drv.run_until_idle(2000, why));

    const TerminalRecord* ta = last_terminal(drv, kEndpointA);
    const TerminalRecord* tb = last_terminal(drv, kEndpointB);
    BOOST_TEST(ta != nullptr);
    BOOST_TEST(tb != nullptr);
    if (ta != nullptr) {
        BOOST_TEST(ta->status == ExchangeStatus::Ok); // initiator's local Final "sent"
        BOOST_TEST(!ta->result.estimate_available);
    }
    if (tb != nullptr)
        BOOST_TEST(tb->status == ExchangeStatus::ProtocolTimeout);
    // The responder did not fabricate a second Response.
    BOOST_TEST(drv.core(kEndpointB).counters().tx_submitted == 1u);
    assert_conservation(drv, kEndpointA);
    assert_conservation(drv, kEndpointB);
}

// ===========================================================================
// B16 (causal): removing a frame changes the outcome
// ===========================================================================

BOOST_AUTO_TEST_CASE(b16_deleting_the_response_frame_changes_the_outcome)
{
    // Path 1: the Response arrives -> a completed estimate at the initiator.
    {
        DriverConfig dc = same_clock_driver(Protocol::Ss, kEndpointA, 15.0, 500, 300, 1000);
        TwoEndpointDriver drv;
        std::string why;
        BOOST_TEST(drv.configure(dc, why));
        BOOST_TEST(drv.begin(kEndpointA, 1000, 1));
        BOOST_TEST(drv.run_until_idle(2000, why));
        const TerminalRecord* t = last_terminal(drv, kEndpointA);
        BOOST_TEST(t != nullptr);
        if (t != nullptr) {
            BOOST_TEST(t->status == ExchangeStatus::Ok);
            BOOST_TEST(t->result.estimate_available);
        }
    }
    // Path 2: the same exchange with the Response deleted -> timeout.
    {
        DriverConfig dc = same_clock_driver(Protocol::Ss, kEndpointA, 15.0, 500, 300, 1000);
        dc.drop_submits.push_back({ kEndpointB, 1u });
        TwoEndpointDriver drv;
        std::string why;
        BOOST_TEST(drv.configure(dc, why));
        BOOST_TEST(drv.begin(kEndpointA, 1000, 1));
        BOOST_TEST(drv.run_until_idle(2000, why));
        const TerminalRecord* t = last_terminal(drv, kEndpointA);
        BOOST_TEST(t != nullptr);
        if (t != nullptr) {
            BOOST_TEST(t->status == ExchangeStatus::ProtocolTimeout);
            BOOST_TEST(!t->result.estimate_available);
        }
    }
}

// ===========================================================================
// B18: same seed / same scenario -> identical results
// ===========================================================================

BOOST_AUTO_TEST_CASE(b18_same_scenario_is_bit_for_bit_reproducible)
{
    auto run = [](const DriverConfig& dc, std::string& why) {
        // Returned by value: only the observable numbers, not the driver.
        struct Snapshot {
            ExchangeStatus a_status = ExchangeStatus::InternalError;
            int64_t a_num = 0, a_den = 0;
            bool a_est = false;
            ExchangeStatus b_status = ExchangeStatus::InternalError;
            int64_t b_num = 0, b_den = 0;
            bool b_est = false;
            uint64_t a_submitted = 0, b_submitted = 0;
            uint64_t a_final_t1 = 0, a_final_t5 = 0, b_resp_t2 = 0, b_resp_t3 = 0;
            bool a_has_final = false;
        };
        Snapshot s;
        TwoEndpointDriver drv;
        if (!drv.configure(dc, why))
            return s;
        if (!drv.begin(kEndpointA, 1000, 1)) {
            why = "begin failed";
            return s;
        }
        if (!drv.run_until_idle(2000, why))
            return s;
        const TerminalRecord* ta = last_terminal(drv, kEndpointA);
        const TerminalRecord* tb = last_terminal(drv, kEndpointB);
        if (ta != nullptr) {
            s.a_status = ta->status;
            s.a_num = ta->result.tof.num;
            s.a_den = ta->result.tof.den;
            s.a_est = ta->result.estimate_available;
        }
        if (tb != nullptr) {
            s.b_status = tb->status;
            s.b_num = tb->result.tof.num;
            s.b_den = tb->result.tof.den;
            s.b_est = tb->result.estimate_available;
        }
        s.a_submitted = drv.core(kEndpointA).counters().tx_submitted;
        s.b_submitted = drv.core(kEndpointB).counters().tx_submitted;
        if (drv.sent_frames(kEndpointA).size() == 2u) {
            s.a_has_final = true;
            s.a_final_t1 = drv.sent_frames(kEndpointA)[1].t1A();
            s.a_final_t5 = drv.sent_frames(kEndpointA)[1].t5A();
        }
        if (!drv.sent_frames(kEndpointB).empty()) {
            s.b_resp_t2 = drv.sent_frames(kEndpointB)[0].t2B();
            s.b_resp_t3 = drv.sent_frames(kEndpointB)[0].t3B();
        }
        return s;
    };

    const DriverConfig dc = same_clock_driver(Protocol::Ds, kEndpointA, 15.0, 500, 300, 1000);
    std::string why1, why2;
    const auto s1 = run(dc, why1);
    const auto s2 = run(dc, why2);
    BOOST_TEST(why1.empty());
    BOOST_TEST(why2.empty());
    BOOST_TEST(s1.a_status == s2.a_status);
    BOOST_TEST(s1.a_num == s2.a_num);
    BOOST_TEST(s1.a_est == s2.a_est);
    BOOST_TEST(s1.b_status == s2.b_status);
    BOOST_TEST(s1.b_num == s2.b_num);
    BOOST_TEST(s1.b_den == s2.b_den);
    BOOST_TEST(s1.b_est == s2.b_est);
    BOOST_TEST(s1.a_submitted == s2.a_submitted);
    BOOST_TEST(s1.b_submitted == s2.b_submitted);
    BOOST_TEST(s1.a_has_final == s2.a_has_final);
    BOOST_TEST(s1.a_final_t1 == s2.a_final_t1);
    BOOST_TEST(s1.a_final_t5 == s2.a_final_t5);
    BOOST_TEST(s1.b_resp_t2 == s2.b_resp_t2);
    BOOST_TEST(s1.b_resp_t3 == s2.b_resp_t3);
}

// ===========================================================================
// R02/R04/R05 end-to-end additions (m1b_fix_qa)
// ===========================================================================

namespace {

// A raw RxFrame event for `cfg`'s endpoint, built without the link so a
// scenario can replay a specific wire Poll / session.
CoreEvent e2e_poll_rx(const CoreConfig& cfg,
                      const ClockDomain& d,
                      uint16_t session,
                      uint16_t seq,
                      int64_t ticks)
{
    Frame f;
    f.version = cfg.frame_profile.version;
    f.function_code = FrameType::Poll;
    f.session_id = session;
    f.seq = seq;
    f.pan_id = cfg.pan_id;
    f.src_addr = cfg.peer_address;
    f.dst_addr = cfg.local_address;
    f.flags = make_flags(true, false);
    CoreEvent e;
    e.kind = CoreEventKind::RxFrame;
    e.fcs_passed = true;
    e.decode_ok = true;
    e.frame = f;
    e.rx_time = make_rx_ts(ticks, d, cfg.local_calibration.id);
    e.rx_first_path = FirstPathQuality::passed(20.0, 6.0, 0.9);
    return e;
}

} // namespace

BOOST_AUTO_TEST_CASE(r02_e2e_ds_initiator_submits_poll_and_final)
{
    // R02: over the real two-endpoint link a DS initiator owes BOTH a Poll and
    // a Final; both must close locally before its own (estimate-less) terminal,
    // while the responder owns the estimate.
    DriverConfig dc = same_clock_driver(Protocol::Ds, kEndpointA, 15.0, 500, 300, 1000);
    TwoEndpointDriver drv;
    std::string why;
    BOOST_TEST(drv.configure(dc, why));
    BOOST_TEST(drv.begin(kEndpointA, 1000, 1));
    BOOST_TEST(drv.run_until_idle(2000, why));

    BOOST_TEST(drv.core(kEndpointA).counters().tx_submitted == 2u);
    BOOST_TEST(drv.core(kEndpointA).counters().tx_outcome_completed == 2u);
    BOOST_TEST(drv.core(kEndpointA).counters().tx_outcome_failed == 0u);

    const TerminalRecord* ta = last_terminal(drv, kEndpointA);
    BOOST_TEST(ta != nullptr);
    if (ta != nullptr) {
        BOOST_TEST(ta->status == ExchangeStatus::Ok);
        BOOST_TEST(!ta->result.estimate_available);
    }
    const TerminalRecord* tb = last_terminal(drv, kEndpointB);
    BOOST_TEST(tb != nullptr);
    if (tb != nullptr) {
        BOOST_TEST(tb->status == ExchangeStatus::Ok);
        BOOST_TEST(tb->result.estimate_available);
        BOOST_TEST(tb->result.tof.num == 50);
        BOOST_TEST(tb->result.tof.den == 1);
    }
    assert_conservation(drv, kEndpointA);
    assert_conservation(drv, kEndpointB);
}

BOOST_AUTO_TEST_CASE(r04_e2e_new_wire_session_clears_the_replay_barrier)
{
    // R04: over the link, the SAME wire Poll after only a LOCAL generation bump
    // must not open a second exchange; a declared NEW wire session may.
    DriverConfig dc = same_clock_driver(Protocol::Ss, kEndpointA, 15.0, 500, 300, 1000);
    TwoEndpointDriver drv;
    std::string why;
    BOOST_TEST(drv.configure(dc, why));
    BOOST_TEST(drv.begin(kEndpointA, 1000, 1));
    BOOST_TEST(drv.run_until_idle(2000, why));
    BOOST_TEST(drv.core(kEndpointB).counters().accepted_exchanges == 1u);

    const uint64_t accepted_b = drv.core(kEndpointB).counters().accepted_exchanges;

    // (a) local generation bump only: the replay is refused.
    drv.post(kEndpointB, make_reset(2));
    drv.post(kEndpointB,
             e2e_poll_rx(dc.cfg_b, dc.link.domain_b, dc.cfg_b.session_id, 0, 5000));
    BOOST_TEST(drv.core(kEndpointB).counters().accepted_exchanges == accepted_b);
    BOOST_TEST(drv.core(kEndpointB).counters().stale_events >= 1u);

    // (b) a declared new wire session clears the barrier.
    drv.post(kEndpointB, make_reset(3, 0, /*has_new_wire_session*/ true,
                                    /*new_wire_session_id*/ 9, /*generation*/ 2));
    drv.post(kEndpointB, e2e_poll_rx(dc.cfg_b, dc.link.domain_b, 9, 0, 6000));
    BOOST_TEST(drv.core(kEndpointB).counters().accepted_exchanges == accepted_b + 1u);
    assert_conservation(drv, kEndpointB);
}

BOOST_AUTO_TEST_CASE(r05_e2e_responder_capacity_refuses_until_drained)
{
    // R05: with result capacity 1 the responder refuses a second exchange
    // BEFORE accepting it (no dropped result); after draining it accepts again.
    DriverConfig dc = same_clock_driver(Protocol::Ss, kEndpointA, 15.0, 500, 300, 1000);
    dc.cfg_b.result_queue_capacity = 1;
    TwoEndpointDriver drv;
    std::string why;
    BOOST_TEST(drv.configure(dc, why));

    BOOST_TEST(drv.begin(kEndpointA, 1000, 1));
    BOOST_TEST(drv.run_until_idle(2000, why));
    BOOST_TEST(drv.core(kEndpointB).counters().accepted_exchanges == 1u);
    BOOST_TEST(drv.core(kEndpointB).counters().terminal_results == 1u);
    BOOST_TEST(drv.core(kEndpointB).counters().results_dropped == 0u);

    BOOST_TEST(drv.begin(kEndpointA, 5000, 2));
    BOOST_TEST(drv.run_until_idle(2000, why));
    BOOST_TEST(drv.core(kEndpointB).counters().accepted_exchanges == 1u);
    BOOST_TEST(drv.core(kEndpointB).counters().requests_rejected_queue_full == 1u);
    BOOST_TEST(drv.core(kEndpointB).counters().results_dropped == 0u);
    assert_conservation(drv, kEndpointB);

    ProtocolTofEstimate est;
    BOOST_TEST(drv.core(kEndpointB).pop_result(est));
    BOOST_TEST(drv.begin(kEndpointA, 9000, 3));
    BOOST_TEST(drv.run_until_idle(2000, why));
    BOOST_TEST(drv.core(kEndpointB).counters().accepted_exchanges == 2u);
    BOOST_TEST(drv.core(kEndpointB).counters().results_dropped == 0u);
}
