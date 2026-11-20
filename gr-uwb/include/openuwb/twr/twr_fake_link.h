/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Deterministic fake TWR transport for M1-B tests and the fake demo.
 *
 * WHAT THIS IS
 * ------------
 * A radio-free, clock-free, thread-free transport between exactly two
 * logical TWR endpoints (A and B).  It models the four things an adapter
 * needs from a link:
 *
 *   * propagation: the ground-truth `distance_m` becomes an arrival delay in
 *     the DESTINATION endpoint's ticks (both endpoints each own a ClockDomain,
 *     so the conversion is explicit and never a shared multiplier);
 *   * per-endpoint TX air latency (the delay between a planned TX instant and
 *     the frame actually leaving the antenna);
 *   * a BOUNDED, arrival-ordered RX queue per endpoint;
 *   * deterministic fault injection (drops and bad FCS), counted, never
 *     silently swallowed.
 *
 * WHAT THIS IS NOT, AND WHY THAT MATTERS
 * --------------------------------------
 * It is NOT an extension of `uwb_fake_burst_backend.h`.  That backend is
 * radar-schedule oriented (`schedule_index` drives an arm-RX / echo-TX pair);
 * TWR needs a receive-driven, message-ordered transport, so this is a separate
 * type with a separate queue discipline.
 *
 * It is NOT installed as public ABI.  M1-B's public contract is
 * `uwb_twr_core.h` / `uwb_twr_protocol_time.h`; this header exists only as
 * QA/demo support and is deliberately excluded from the install set.
 *
 * GROUND-TRUTH ISOLATION (the point of the whole file)
 * ----------------------------------------------------
 * `distance_m` is GROUND TRUTH.  It lives in `FakeLinkConfig`, is consumed
 * only inside `compute_arrival()`, and is never copied into a `FakeRx` or any
 * endpoint-visible object: a `FakeRx` carries bytes, a token, the RX marker
 * instant in the receiver's ticks, the FCS/decode verdict and the first-path
 * quality.  There is deliberately NO `distance`, NO `interval` and NO
 * `tof`/`truth` field on `FakeRx`; `qa_uwb_twr_fake_link.cc` asserts that by
 * name with a detection trait.  An endpoint therefore cannot read the answer
 * out of the transport -- it can only see what a real receiver would see.
 *
 * DETERMINISM
 * -----------
 * The transport never reads a clock, never sleeps and never starts a thread.
 * Every arrival is a pure function of the submitted `air_ticks`, the two
 * `ClockDomain`s, the configured latencies and `distance_m`.  `seed` is
 * recorded so a scenario can state it, but there is no randomness in this
 * type: drop/bad-FCS injection is periodic and counted, so two runs with the
 * same configuration produce the same arrivals byte-for-byte.
 *
 * ARRIVAL ARITHMETIC (one line, stated exactly)
 * ---------------------------------------------
 *   src_ticks   = tx.air_ticks + (source A ? tx_air_latency_a
 *                                          : tx_air_latency_b)   [source grid]
 *   dest_ticks  = src_ticks                     if src.rate == dst.rate
 *               = llround(src_ticks/src.rate*hz * dst.rate)     otherwise
 *   arrival     = dest_ticks + llround((distance_m/kSpeedOfLightMps)
 *                                      * dst.rate)
 *
 * `tx_air_latency_*` is a TX-chain property of the SOURCE, so it is expressed
 * in SOURCE ticks and travels through the same grid conversion as
 * `air_ticks`.  `propagation_ticks()` is the last term, in DESTINATION ticks.
 * At equal rates (the one-X410-two-channel case) the conversion is the
 * identity, so arrival == air_ticks + latency + propagation exactly.
 */

#ifndef INCLUDED_GNURADIO_UWB_UWB_TWR_FAKE_LINK_H
#define INCLUDED_GNURADIO_UWB_UWB_TWR_FAKE_LINK_H

#include <gnuradio/uwb/uwb_twr_frame.h>     // kMaxFrameBytes
#include <gnuradio/uwb/uwb_twr_tof_input.h> // FirstPathQuality, ClockDomain

#include <cstddef>
#include <cstdint>
#include <deque>
#include <string>

namespace gr {
namespace uwb {
namespace twr {

// ---------------------------------------------------------------------------
// Endpoint identity
// ---------------------------------------------------------------------------
//
// Two logical endpoints, mirroring the one-X410-two-channel phase-1 setup.
// These are PHYSICAL endpoints, not protocol roles: which one is the
// initiator is a separate binding decided by the exchange, so role reversal
// does not require a new transport.
inline constexpr uint8_t kEndpointA = 0;
inline constexpr uint8_t kEndpointB = 1;
inline constexpr uint8_t kEndpointCount = 2;

// Speed of light in vacuum, used ONLY here to turn the ground-truth distance
// into ticks.  Spelled locally so the transport header does not depend on the
// ToF math header; the value matches kPropagationSpeedVacuumMps there.
inline constexpr double kFakeLinkSpeedOfLightMps = 299792458.0;

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

struct FakeLinkConfig {
    // One clock domain per physical endpoint.  Both endpoints on one X410
    // legitimately share one domain; independent clocks use different names.
    ClockDomain domain_a;
    ClockDomain domain_b;

    // GROUND TRUTH.  Metric metres.  Read only by compute_arrival(); never
    // exposed through an endpoint-visible object.
    double distance_m = 0.0;

    // TX-chain air latency of each source endpoint, in that endpoint's ticks.
    int64_t tx_air_latency_a_ticks = 0;
    int64_t tx_air_latency_b_ticks = 0;

    // The receiver's declared listen window, in the receiver's ticks.  Recorded
    // for the adapter and validated for sanity; this transport does not arm or
    // enforce a window (arrival is a deterministic function of the submit, and
    // there is no shared arm instant to measure it against).
    int64_t rx_window_ticks = 0;

    // Bounded RX queue per endpoint.  Capacity 0 is refused.
    uint32_t rx_queue_capacity = 16;

    // Recorded for scenario repeatability; there is no randomness in this type.
    uint64_t seed = 1;

    // Periodic fault injection.  0 disables the injector.
    //   drop_every_n_tx    : the Nth submit (1-based, PER SOURCE endpoint)
    //                        queues a drop event instead of the frame.
    //   bad_fcs_every_n_rx : the Nth non-drop delivery (1-based, GLOBAL across
    //                        both queues) has fcs_passed=false, bytes intact.
    uint32_t drop_every_n_tx = 0;
    uint32_t bad_fcs_every_n_rx = 0;
};

// ---------------------------------------------------------------------------
// Error taxonomy
// ---------------------------------------------------------------------------

enum class FakeLinkError : uint8_t {
    Ok = 0,
    NotConfigured = 1,
    UnknownEndpoint = 2,
    QueueFull = 3,
    BadBytes = 4,
    BadTime = 5
};

inline const char* fake_link_error_to_string(FakeLinkError e)
{
    // No `default`: adding an enumerator without handling it here is a
    // -Wswitch warning, which is the point.
    switch (e) {
    case FakeLinkError::Ok:
        return "ok";
    case FakeLinkError::NotConfigured:
        return "not_configured";
    case FakeLinkError::UnknownEndpoint:
        return "unknown_endpoint";
    case FakeLinkError::QueueFull:
        return "queue_full";
    case FakeLinkError::BadBytes:
        return "bad_bytes";
    case FakeLinkError::BadTime:
        return "bad_time";
    }
    return "invalid";
}

// Domain test for the public enum, separate from the formatting switch.
// A value cast in from an integer is not a member and must be caught BEFORE
// any consumer switches on it (the M0.1 N07 lesson).  No `default`.
inline bool fake_link_error_is_known(FakeLinkError e)
{
    switch (e) {
    case FakeLinkError::Ok:
    case FakeLinkError::NotConfigured:
    case FakeLinkError::UnknownEndpoint:
    case FakeLinkError::QueueFull:
    case FakeLinkError::BadBytes:
    case FakeLinkError::BadTime:
        return true;
    }
    return false;
}

// ---------------------------------------------------------------------------
// Wire values
// ---------------------------------------------------------------------------

// One scheduled transmit handed to the transport.  `air_ticks` is the planned
// on-air instant in the SOURCE endpoint's ticks.  This is the transport's
// INPUT, not an endpoint's RX event.
struct FakeTx {
    uint8_t source = 0;
    uint64_t token = 0;
    uint8_t bytes[kMaxFrameBytes] = {};
    size_t nbytes = 0;
    int64_t air_ticks = 0;
};

// One receive event handed to an endpoint.  The receiver sees bytes, an FCS
// verdict, a decode verdict, its own marker instant and the first-path
// quality -- exactly what a physical receiver could observe.  It does NOT see
// the ground-truth distance, a ToF, or a propagation interval.
struct FakeRx {
    uint8_t source = 0;      // which endpoint transmitted this
    uint8_t destination = 0; // which endpoint's queue it is in
    uint64_t token = 0;
    uint8_t bytes[kMaxFrameBytes] = {};
    size_t nbytes = 0;
    int64_t rx_marker_ticks = 0; // arrival, in the destination's ticks
    bool fcs_passed = true;
    bool decode_ok = true;
    FirstPathQuality first_path;
    bool injected_drop = false;
    std::string note;
};

// The default first-path verdict the transport attaches to a healthy frame.
inline FirstPathQuality fake_first_path_default()
{
    return FirstPathQuality::passed(20.0, 6.0, 0.9);
}

// The named helper the interface exposes; kept distinct from the default so a
// scenario can state its own SNR/peak/confidence.
inline FirstPathQuality fake_first_path_passed(double snr_db, double peak_db, double conf)
{
    return FirstPathQuality::passed(snr_db, peak_db, conf);
}

// ---------------------------------------------------------------------------
// FakeTwrLink
// ---------------------------------------------------------------------------

class FakeTwrLink
{
public:
    FakeTwrLink();
    ~FakeTwrLink();

    FakeTwrLink(const FakeTwrLink&) = delete;
    FakeTwrLink& operator=(const FakeTwrLink&) = delete;

    // Validates and installs the configuration, then clears all queues and
    // counters so a scenario starts from a known state.  Refuses, with a
    // reason, when:
    //   * either domain is not a valid ClockDomain;
    //   * either TX air latency is negative;
    //   * rx_window_ticks is negative;
    //   * rx_queue_capacity is zero;
    //   * distance_m is not finite or is negative.
    bool configure(const FakeLinkConfig& cfg, std::string& why);

    bool is_configured() const { return d_configured; }
    const FakeLinkConfig& config() const { return d_cfg; }

    // Propagation delay from `source` to the other endpoint, in the
    // DESTINATION's ticks: llround((distance_m / c) * dest.tick_rate_hz).
    // Returns 0 for an unconfigured link or an unknown endpoint.
    int64_t propagation_ticks(uint8_t source) const;

    // Queue one transmit.  On success the event is inserted into the
    // destination's bounded queue in arrival order.  On QueueFull the event is
    // NOT kept and the counter records it; nothing is silently dropped.
    // Rejected (with `why`) for an unknown source, empty/oversized bytes, a
    // negative air instant, or an unconfigured link.
    FakeLinkError submit(const FakeTx& tx, std::string& why);

    // The arrival instant at the head of `endpoint`'s queue, or false when the
    // queue is empty (or the endpoint is unknown).
    bool next_arrival(uint8_t endpoint, int64_t& ticks) const;

    // Pop the oldest arrival (ties broken by submit order) into `out`.
    bool pop_rx(uint8_t endpoint, FakeRx& out);

    // Number of queued receive events for `endpoint` (0 for an unknown one).
    size_t pending(uint8_t endpoint) const;

    struct Counters {
        uint64_t submitted = 0;       // valid submits, including later QueueFull
        uint64_t delivered = 0;       // events actually enqueued (drops included)
        uint64_t injected_drops = 0;  // drop events enqueued
        uint64_t injected_bad_fcs = 0; // deliveries marked fcs_passed=false
        uint64_t queue_full = 0;      // submits refused by a full queue
        uint64_t rejected = 0;        // submits refused by input validation
    };

    const Counters& counters() const { return d_counters; }
    // Clears the public counters AND the periodic-injection phase, so the
    // drop/bad-FCS pattern restarts deterministically.
    void reset_counters();

private:
    struct Entry {
        int64_t arrival = 0;
        uint64_t seq = 0; // submit order, the arrival-order tie-break
        FakeRx rx;
    };

    const ClockDomain& domain_for(uint8_t endpoint) const;
    int64_t compute_arrival(const FakeTx& tx) const;
    void insert_sorted(uint8_t endpoint, Entry&& e);
    void clear_state();

    FakeLinkConfig d_cfg;
    bool d_configured = false;

    std::deque<Entry> d_queues[kEndpointCount];

    // Periodic-injection phase, separate from the public counters.
    uint64_t d_submit_count[kEndpointCount] = { 0, 0 };
    uint64_t d_delivery_count = 0; // non-drop enqueues, for bad-FCS phase
    uint64_t d_seq = 0;

    Counters d_counters;
};

} // namespace twr
} // namespace uwb
} // namespace gr

#endif /* INCLUDED_GNURADIO_UWB_UWB_TWR_FAKE_LINK_H */
