/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * QA for the deterministic fake TWR transport (uwb_twr_fake_link.h).
 *
 * WHAT IS INDEPENDENT HERE
 * ------------------------
 * Every arrival expectation below is written out BY HAND from the documented
 * arithmetic (the propagation number, the queue order, the tie-break), not by
 * calling the transport and copying its output.  Where a rate conversion is
 * involved the test states the expected integer explicitly.  The ground-truth
 * isolation is checked STRUCTURALLY: `FakeRx` must not carry a distance /
 * interval / ToF member, with a positive control proving the detector fires.
 *
 * Radio-free: no GNU Radio, no UHD, no thread, no clock, no I/O.
 */

#include <boost/test/unit_test.hpp>

#include <gnuradio/uwb/uwb_twr_fake_link.h>

#include <cstdint>
#include <limits>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

namespace boost {
namespace test_tools {
namespace tt_detail {

template <>
struct print_log_value<gr::uwb::twr::FakeLinkError> {
    void operator()(std::ostream& os, gr::uwb::twr::FakeLinkError const& e) const
    {
        os << gr::uwb::twr::fake_link_error_to_string(e);
    }
};

} // namespace tt_detail
} // namespace test_tools
} // namespace boost

BOOST_AUTO_TEST_SUITE(twr_fake_link)

namespace {

using namespace gr::uwb::twr;

constexpr double kRate1G = 1.0e9;
constexpr double kRateX410 = 998.4e6;

ClockDomain domain(const std::string& name, double rate = kRate1G, uint64_t epoch = 7u)
{
    ClockDomain d;
    const bool ok = ClockDomain::make(name, rate, epoch, 0u, d);
    BOOST_REQUIRE_MESSAGE(ok, "domain " << name << " did not construct");
    return d;
}

FakeLinkConfig base_config(double rate_a = kRate1G, double rate_b = kRate1G)
{
    FakeLinkConfig c;
    c.domain_a = domain("ep_a", rate_a);
    c.domain_b = domain("ep_b", rate_b);
    return c;
}

FakeTx tx_from(uint8_t source, uint64_t token, int64_t air_ticks)
{
    FakeTx t;
    t.source = source;
    t.token = token;
    t.nbytes = 4;
    for (size_t i = 0; i < t.nbytes; ++i)
        t.bytes[i] = static_cast<uint8_t>(token + i);
    t.air_ticks = air_ticks;
    return t;
}

// ---------------------------------------------------------------------------
// Ground-truth isolation, by member name (positive control + negative claims)
// ---------------------------------------------------------------------------
template <typename T, typename = void>
struct has_distance_member : std::false_type {};
template <typename T>
struct has_distance_member<T, std::void_t<decltype(std::declval<const T&>().distance_m)>>
    : std::true_type {};

template <typename T, typename = void>
struct has_interval_member : std::false_type {};
template <typename T>
struct has_interval_member<T, std::void_t<decltype(std::declval<const T&>().interval)>>
    : std::true_type {};

template <typename T, typename = void>
struct has_tof_member : std::false_type {};
template <typename T>
struct has_tof_member<T, std::void_t<decltype(std::declval<const T&>().tof)>>
    : std::true_type {};

template <typename T, typename = void>
struct has_truth_member : std::false_type {};
template <typename T>
struct has_truth_member<T, std::void_t<decltype(std::declval<const T&>().truth)>>
    : std::true_type {};

} // namespace

// ===========================================================================
// Error taxonomy: domain test first, formatting switch with no default
// ===========================================================================
BOOST_AUTO_TEST_CASE(fake_link_error_domain_is_fail_closed)
{
    const FakeLinkError defined[] = {
        FakeLinkError::Ok,          FakeLinkError::NotConfigured,
        FakeLinkError::UnknownEndpoint, FakeLinkError::QueueFull,
        FakeLinkError::BadBytes,    FakeLinkError::BadTime
    };
    for (const FakeLinkError e : defined) {
        BOOST_TEST(fake_link_error_is_known(e));
        BOOST_TEST(std::string(fake_link_error_to_string(e)) != "invalid");
    }
    const FakeLinkError bogus = static_cast<FakeLinkError>(200);
    BOOST_TEST(!fake_link_error_is_known(bogus));
    BOOST_TEST(std::string(fake_link_error_to_string(bogus)) == "invalid");
    BOOST_TEST(std::string(fake_link_error_to_string(FakeLinkError::QueueFull)) ==
               "queue_full");
}

// ===========================================================================
// Empty / unconfigured link
// ===========================================================================
BOOST_AUTO_TEST_CASE(fake_link_empty_and_unconfigured)
{
    FakeTwrLink link;
    BOOST_TEST(!link.is_configured());

    std::string why = "stale";
    FakeTx t = tx_from(kEndpointA, 1, 100);
    BOOST_TEST(link.submit(t, why) == FakeLinkError::NotConfigured);
    BOOST_TEST(!why.empty());
    BOOST_TEST(link.counters().rejected == 1);

    int64_t ticks = 12345;
    BOOST_TEST(!link.next_arrival(kEndpointA, ticks));
    BOOST_TEST(ticks == 12345); // refusal leaves the output untouched
    BOOST_TEST(link.pending(kEndpointA) == 0);
    BOOST_TEST(link.pending(kEndpointB) == 0);

    FakeRx rx;
    BOOST_TEST(!link.pop_rx(kEndpointA, rx));
    BOOST_TEST(!link.pop_rx(kEndpointB, rx));
    BOOST_TEST(link.propagation_ticks(kEndpointA) == 0);
}

// ===========================================================================
// configure(): round trip and every refusal
// ===========================================================================
BOOST_AUTO_TEST_CASE(fake_link_config_roundtrip_and_refusals)
{
    FakeLinkConfig good = base_config();
    good.distance_m = 12.5;
    good.tx_air_latency_a_ticks = 11;
    good.tx_air_latency_b_ticks = 13;
    good.rx_window_ticks = 4096;
    good.rx_queue_capacity = 3;
    good.seed = 99;
    good.drop_every_n_tx = 5;
    good.bad_fcs_every_n_rx = 7;

    FakeTwrLink link;
    std::string why = "stale";
    BOOST_REQUIRE_MESSAGE(link.configure(good, why), why);
    BOOST_TEST(link.is_configured());
    BOOST_TEST(link.config().distance_m == 12.5);
    BOOST_TEST(link.config().tx_air_latency_a_ticks == 11);
    BOOST_TEST(link.config().tx_air_latency_b_ticks == 13);
    BOOST_TEST(link.config().rx_window_ticks == 4096);
    BOOST_TEST(link.config().rx_queue_capacity == 3u);
    BOOST_TEST(link.config().seed == 99u);
    BOOST_TEST(link.config().drop_every_n_tx == 5u);
    BOOST_TEST(link.config().bad_fcs_every_n_rx == 7u);

    // Refusals.  Each fresh link must keep `why` non-empty and stay
    // unconfigured.
    auto refused = [](const FakeLinkConfig& c) {
        FakeTwrLink l;
        std::string w;
        const bool ok = l.configure(c, w);
        return !ok && !w.empty() && !l.is_configured();
    };

    FakeLinkConfig bad;
    BOOST_TEST(refused(bad)); // both domains invalid (empty names)

    FakeLinkConfig bad_a = base_config();
    bad_a.domain_a.name.clear();
    BOOST_TEST(refused(bad_a));

    FakeLinkConfig bad_b = base_config();
    bad_b.domain_b.name.clear();
    BOOST_TEST(refused(bad_b));

    FakeLinkConfig bad_lat_a = base_config();
    bad_lat_a.tx_air_latency_a_ticks = -1;
    BOOST_TEST(refused(bad_lat_a));

    FakeLinkConfig bad_lat_b = base_config();
    bad_lat_b.tx_air_latency_b_ticks = -1;
    BOOST_TEST(refused(bad_lat_b));

    FakeLinkConfig bad_win = base_config();
    bad_win.rx_window_ticks = -5;
    BOOST_TEST(refused(bad_win));

    FakeLinkConfig bad_cap = base_config();
    bad_cap.rx_queue_capacity = 0;
    BOOST_TEST(refused(bad_cap));

    FakeLinkConfig bad_nan = base_config();
    bad_nan.distance_m = std::numeric_limits<double>::quiet_NaN();
    BOOST_TEST(refused(bad_nan));

    FakeLinkConfig bad_dist = base_config();
    bad_dist.distance_m = -0.25;
    BOOST_TEST(refused(bad_dist));
}

// ===========================================================================
// Arrival arithmetic, same rate: arrival == air + latency + propagation
// ===========================================================================
BOOST_AUTO_TEST_CASE(fake_link_arrival_same_rate_is_exact)
{
    FakeLinkConfig c = base_config();
    // 299792.458 m / c == exactly 1e-3 s; at 1 GHz that is 1,000,000 ticks.
    c.distance_m = 299792.458;
    c.tx_air_latency_a_ticks = 250;
    c.tx_air_latency_b_ticks = 75;

    FakeTwrLink link;
    std::string why;
    BOOST_REQUIRE(link.configure(c, why));

    BOOST_TEST(link.propagation_ticks(kEndpointA) == 1000000);
    BOOST_TEST(link.propagation_ticks(kEndpointB) == 1000000);

    FakeTx t = tx_from(kEndpointA, 1, 5000);
    BOOST_TEST(link.submit(t, why) == FakeLinkError::Ok);
    int64_t arr = -1;
    BOOST_REQUIRE(link.next_arrival(kEndpointB, arr));
    BOOST_TEST(arr == 5000 + 250 + 1000000);

    FakeRx rx;
    BOOST_REQUIRE(link.pop_rx(kEndpointB, rx));
    BOOST_TEST(rx.rx_marker_ticks == arr);
    BOOST_TEST(rx.source == kEndpointA);
    BOOST_TEST(rx.destination == kEndpointB);
    BOOST_TEST(rx.nbytes == 4u);
    BOOST_TEST(link.pending(kEndpointB) == 0u);
}

// ===========================================================================
// Arrival ordering and the submit-order tie-break
// ===========================================================================
BOOST_AUTO_TEST_CASE(fake_link_pops_in_arrival_order_with_submit_tiebreak)
{
    FakeLinkConfig c = base_config(); // distance 0, latency 0
    FakeTwrLink link;
    std::string why;
    BOOST_REQUIRE(link.configure(c, why));

    // Submitted 500, 100, 500 -> arrivals must order as token2, token1, token3.
    BOOST_REQUIRE(link.submit(tx_from(kEndpointA, 1, 500), why) == FakeLinkError::Ok);
    BOOST_REQUIRE(link.submit(tx_from(kEndpointA, 2, 100), why) == FakeLinkError::Ok);
    BOOST_REQUIRE(link.submit(tx_from(kEndpointA, 3, 500), why) == FakeLinkError::Ok);

    BOOST_TEST(link.pending(kEndpointB) == 3u);
    int64_t arr = -1;
    BOOST_REQUIRE(link.next_arrival(kEndpointB, arr));
    BOOST_TEST(arr == 100);

    FakeRx rx;
    BOOST_REQUIRE(link.pop_rx(kEndpointB, rx));
    BOOST_TEST(rx.token == 2u);
    BOOST_REQUIRE(link.pop_rx(kEndpointB, rx));
    BOOST_TEST(rx.token == 1u); // the earlier submit wins the 500-tick tie
    BOOST_REQUIRE(link.pop_rx(kEndpointB, rx));
    BOOST_TEST(rx.token == 3u);
    BOOST_TEST(!link.pop_rx(kEndpointB, rx));
}

// ===========================================================================
// Bounded queue: QueueFull is reported and counted, never silent
// ===========================================================================
BOOST_AUTO_TEST_CASE(fake_link_queue_full_is_rejected_and_counted)
{
    FakeLinkConfig c = base_config();
    c.rx_queue_capacity = 2;
    FakeTwrLink link;
    std::string why;
    BOOST_REQUIRE(link.configure(c, why));

    BOOST_TEST(link.submit(tx_from(kEndpointA, 1, 10), why) == FakeLinkError::Ok);
    BOOST_TEST(link.submit(tx_from(kEndpointA, 2, 20), why) == FakeLinkError::Ok);
    why.clear();
    BOOST_TEST(link.submit(tx_from(kEndpointA, 3, 30), why) == FakeLinkError::QueueFull);
    BOOST_TEST(!why.empty());

    BOOST_TEST(link.counters().queue_full == 1u);
    BOOST_TEST(link.counters().delivered == 2u);
    BOOST_TEST(link.counters().submitted == 3u);
    BOOST_TEST(link.counters().rejected == 0u);
    BOOST_TEST(link.pending(kEndpointB) == 2u);
}

// ===========================================================================
// Drop injection: per source, counted, payload-free
// ===========================================================================
BOOST_AUTO_TEST_CASE(fake_link_drop_injection_is_per_source_and_counted)
{
    FakeLinkConfig c = base_config();
    c.drop_every_n_tx = 2;
    FakeTwrLink link;
    std::string why;
    BOOST_REQUIRE(link.configure(c, why));

    // 1st A healthy, 1st B healthy, 2nd A drop, 2nd B drop.
    BOOST_REQUIRE(link.submit(tx_from(kEndpointA, 11, 100), why) == FakeLinkError::Ok);
    BOOST_REQUIRE(link.submit(tx_from(kEndpointB, 22, 200), why) == FakeLinkError::Ok);
    BOOST_REQUIRE(link.submit(tx_from(kEndpointA, 33, 300), why) == FakeLinkError::Ok);
    BOOST_REQUIRE(link.submit(tx_from(kEndpointB, 44, 400), why) == FakeLinkError::Ok);

    FakeRx a;
    BOOST_REQUIRE(link.pop_rx(kEndpointB, a)); // A's first frame
    BOOST_TEST(a.token == 11u);
    BOOST_TEST(!a.injected_drop);
    BOOST_TEST(a.decode_ok);
    FakeRx b;
    BOOST_REQUIRE(link.pop_rx(kEndpointB, b)); // A's second frame, dropped
    BOOST_TEST(b.token == 33u);
    BOOST_TEST(b.injected_drop);
    BOOST_TEST(!b.fcs_passed);
    BOOST_TEST(!b.decode_ok);
    BOOST_TEST(b.nbytes == 0u);
    BOOST_TEST(!b.first_path.is_recorded());

    FakeRx c1;
    BOOST_REQUIRE(link.pop_rx(kEndpointA, c1));
    BOOST_TEST(c1.token == 22u);
    BOOST_TEST(!c1.injected_drop);
    FakeRx c2;
    BOOST_REQUIRE(link.pop_rx(kEndpointA, c2));
    BOOST_TEST(c2.token == 44u);
    BOOST_TEST(c2.injected_drop);

    BOOST_TEST(link.counters().injected_drops == 2u);
    BOOST_TEST(link.counters().delivered == 4u);
    BOOST_TEST(link.counters().submitted == 4u);
}

// ===========================================================================
// Bad-FCS injection preserves the bytes and keeps the frame parseable
// ===========================================================================
BOOST_AUTO_TEST_CASE(fake_link_bad_fcs_preserves_bytes)
{
    FakeLinkConfig c = base_config();
    c.bad_fcs_every_n_rx = 2;
    FakeTwrLink link;
    std::string why;
    BOOST_REQUIRE(link.configure(c, why));

    FakeTx t1 = tx_from(kEndpointA, 1, 100);
    t1.bytes[0] = 0xAA;
    t1.bytes[1] = 0xBB;
    t1.bytes[2] = 0xCC;
    t1.bytes[3] = 0xDD;
    FakeTx t2 = tx_from(kEndpointA, 2, 200);
    t2.bytes[0] = 0x01;
    t2.bytes[1] = 0x02;
    t2.bytes[2] = 0x03;
    t2.bytes[3] = 0x04;

    BOOST_REQUIRE(link.submit(t1, why) == FakeLinkError::Ok);
    BOOST_REQUIRE(link.submit(t2, why) == FakeLinkError::Ok);

    FakeRx rx;
    BOOST_REQUIRE(link.pop_rx(kEndpointB, rx));
    BOOST_TEST(rx.fcs_passed);
    BOOST_TEST(rx.bytes[0] == 0xAA);
    BOOST_TEST(rx.bytes[3] == 0xDD);

    BOOST_REQUIRE(link.pop_rx(kEndpointB, rx));
    BOOST_TEST(!rx.fcs_passed);  // the Nth (2nd) delivery fails FCS
    BOOST_TEST(rx.decode_ok);    // still a decodable frame; only the FCS verdict
    BOOST_TEST(rx.nbytes == 4u);
    BOOST_TEST(rx.bytes[0] == 0x01);
    BOOST_TEST(rx.bytes[3] == 0x04);
    BOOST_TEST(link.counters().injected_bad_fcs == 1u);
}

// ===========================================================================
// First-path defaults and the named helper
// ===========================================================================
BOOST_AUTO_TEST_CASE(fake_link_first_path_defaults)
{
    FakeLinkConfig c = base_config();
    FakeTwrLink link;
    std::string why;
    BOOST_REQUIRE(link.configure(c, why));
    BOOST_REQUIRE(link.submit(tx_from(kEndpointA, 1, 10), why) == FakeLinkError::Ok);

    FakeRx rx;
    BOOST_REQUIRE(link.pop_rx(kEndpointB, rx));
    BOOST_TEST(rx.first_path.has_passed());
    BOOST_TEST(rx.first_path.first_path_snr_db == 20.0);
    BOOST_TEST(rx.first_path.peak_to_first_path_db == 6.0);
    BOOST_TEST(rx.first_path.confidence == 0.9);

    const FirstPathQuality q = fake_first_path_passed(11.5, 2.5, 0.5);
    BOOST_TEST(q.has_passed());
    BOOST_TEST(q.first_path_snr_db == 11.5);
    BOOST_TEST(q.peak_to_first_path_db == 2.5);
    BOOST_TEST(q.confidence == 0.5);
}

// ===========================================================================
// Invalid input: unknown endpoint, bad bytes, bad time
// ===========================================================================
BOOST_AUTO_TEST_CASE(fake_link_rejects_unknown_endpoint_and_bad_input)
{
    FakeTwrLink link;
    std::string why;
    FakeLinkConfig c = base_config();
    BOOST_REQUIRE(link.configure(c, why));

    FakeTx bad_src = tx_from(9, 1, 100);
    why.clear();
    BOOST_TEST(link.submit(bad_src, why) == FakeLinkError::UnknownEndpoint);
    BOOST_TEST(!why.empty());

    FakeTx empty = tx_from(kEndpointA, 2, 100);
    empty.nbytes = 0;
    why.clear();
    BOOST_TEST(link.submit(empty, why) == FakeLinkError::BadBytes);

    FakeTx oversized = tx_from(kEndpointA, 3, 100);
    oversized.nbytes = kMaxFrameBytes + 1;
    why.clear();
    BOOST_TEST(link.submit(oversized, why) == FakeLinkError::BadBytes);

    FakeTx negative_time = tx_from(kEndpointA, 4, -1);
    why.clear();
    BOOST_TEST(link.submit(negative_time, why) == FakeLinkError::BadTime);

    BOOST_TEST(link.counters().rejected == 4u);
    BOOST_TEST(link.counters().submitted == 0u);
    BOOST_TEST(link.pending(kEndpointB) == 0u);

    int64_t ticks = 7;
    BOOST_TEST(!link.next_arrival(9, ticks));
    BOOST_TEST(ticks == 7);
    BOOST_TEST(link.pending(9) == 0u);
    FakeRx rx;
    BOOST_TEST(!link.pop_rx(9, rx));
}

// ===========================================================================
// reset_counters(): zeroes counters AND restarts the injection phase
// ===========================================================================
BOOST_AUTO_TEST_CASE(fake_link_reset_counters_restarts_injection_phase)
{
    FakeLinkConfig c = base_config();
    c.drop_every_n_tx = 2;
    FakeTwrLink link;
    std::string why;
    BOOST_REQUIRE(link.configure(c, why));

    BOOST_REQUIRE(link.submit(tx_from(kEndpointA, 1, 10), why) == FakeLinkError::Ok);
    BOOST_REQUIRE(link.submit(tx_from(kEndpointA, 2, 20), why) == FakeLinkError::Ok);
    BOOST_TEST(link.counters().injected_drops == 1u);

    link.reset_counters();
    BOOST_TEST(link.counters().submitted == 0u);
    BOOST_TEST(link.counters().delivered == 0u);
    BOOST_TEST(link.counters().injected_drops == 0u);

    // The Nth-submit phase restarts: the first submit after the reset is
    // healthy again, the second drops.
    BOOST_REQUIRE(link.submit(tx_from(kEndpointA, 3, 30), why) == FakeLinkError::Ok);
    BOOST_REQUIRE(link.submit(tx_from(kEndpointA, 4, 40), why) == FakeLinkError::Ok);
    FakeRx rx;
    BOOST_REQUIRE(link.pop_rx(kEndpointB, rx));
    BOOST_TEST(rx.token == 1u);
    BOOST_REQUIRE(link.pop_rx(kEndpointB, rx));
    BOOST_TEST(rx.token == 2u);
    BOOST_TEST(rx.injected_drop);
    BOOST_REQUIRE(link.pop_rx(kEndpointB, rx));
    BOOST_TEST(rx.token == 3u);
    BOOST_TEST(!rx.injected_drop);
    BOOST_REQUIRE(link.pop_rx(kEndpointB, rx));
    BOOST_TEST(rx.token == 4u);
    BOOST_TEST(rx.injected_drop);
}

// ===========================================================================
// Cross-rate conversion (independent clocks on the two endpoints)
// ===========================================================================
BOOST_AUTO_TEST_CASE(fake_link_cross_rate_conversion)
{
    FakeLinkConfig c = base_config(kRate1G, kRateX410); // A=1 GHz, B=998.4 MHz
    c.distance_m = 299792.458; // 1e-3 s -> 998400 ticks at B
    c.tx_air_latency_a_ticks = 1000;

    FakeTwrLink link;
    std::string why;
    BOOST_REQUIRE(link.configure(c, why));

    // propagation into B: round(1e-3 * 998.4e6) = 998400
    BOOST_TEST(link.propagation_ticks(kEndpointA) == 998400);
    // propagation into A: round(1e-3 * 1e9) = 1000000
    BOOST_TEST(link.propagation_ticks(kEndpointB) == 1000000);

    // air 0 + latency 1000 in A ticks -> round(1000/1e9*998.4e6) = round(998.4)
    // = 998 B ticks, plus 998400 propagation = 999398.
    BOOST_REQUIRE(link.submit(tx_from(kEndpointA, 1, 0), why) == FakeLinkError::Ok);
    int64_t arr = -1;
    BOOST_REQUIRE(link.next_arrival(kEndpointB, arr));
    BOOST_TEST(arr == 998 + 998400);

    // B -> A: propagation 1000000, no latency.
    BOOST_REQUIRE(link.submit(tx_from(kEndpointB, 2, 0), why) == FakeLinkError::Ok);
    BOOST_REQUIRE(link.next_arrival(kEndpointA, arr));
    BOOST_TEST(arr == 1000000);
}

// ===========================================================================
// Ground-truth isolation: FakeRx has no distance / interval / ToF / truth
// ===========================================================================
BOOST_AUTO_TEST_CASE(fake_link_rx_has_no_ground_truth)
{
    // Positive controls: the detector fires where it must, so a silent
    // no-op detector cannot make the negative claims below pass.
    static_assert(has_distance_member<FakeLinkConfig>::value,
                  "control: FakeLinkConfig must carry the ground-truth distance");
    static_assert(!has_interval_member<FakeLinkConfig>::value,
                  "control: the detector must be discriminating");

    // The endpoint-visible value must not carry the answer.
    static_assert(!has_distance_member<FakeRx>::value,
                  "FakeRx must not expose the ground-truth distance");
    static_assert(!has_interval_member<FakeRx>::value,
                  "FakeRx must not expose a propagation interval");
    static_assert(!has_tof_member<FakeRx>::value,
                  "FakeRx must not expose a ToF");
    static_assert(!has_truth_member<FakeRx>::value,
                  "FakeRx must not expose a truth member");

    BOOST_TEST(true); // the static_asserts are the test
}

// ===========================================================================
// Determinism: identical scenario twice -> identical event stream
// ===========================================================================
BOOST_AUTO_TEST_CASE(fake_link_is_deterministic)
{
    auto run = [](std::vector<FakeRx>& out) {
        FakeLinkConfig c = base_config();
        c.distance_m = 100.0;
        c.tx_air_latency_a_ticks = 37;
        c.drop_every_n_tx = 3;
        c.bad_fcs_every_n_rx = 4;
        c.seed = 1234;
        FakeTwrLink link;
        std::string why;
        BOOST_REQUIRE(link.configure(c, why));
        for (int i = 0; i < 6; ++i)
            BOOST_REQUIRE(link.submit(tx_from(kEndpointA, static_cast<uint64_t>(i), 1000 + i * 7),
                                      why) == FakeLinkError::Ok);
        FakeRx rx;
        while (link.pop_rx(kEndpointB, rx))
            out.push_back(rx);
    };

    std::vector<FakeRx> first;
    std::vector<FakeRx> second;
    run(first);
    run(second);

    BOOST_REQUIRE_EQUAL(first.size(), 6u);
    BOOST_REQUIRE_EQUAL(first.size(), second.size());
    for (size_t i = 0; i < first.size(); ++i) {
        BOOST_TEST(first[i].token == second[i].token);
        BOOST_TEST(first[i].rx_marker_ticks == second[i].rx_marker_ticks);
        BOOST_TEST(first[i].injected_drop == second[i].injected_drop);
        BOOST_TEST(first[i].fcs_passed == second[i].fcs_passed);
    }
}

BOOST_AUTO_TEST_SUITE_END()
