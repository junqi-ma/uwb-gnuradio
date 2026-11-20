/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Implementation of the deterministic fake TWR transport.  See
 * uwb_twr_fake_link.h for the contract; this file contains no clock, no
 * sleep, no thread and no randomness.
 */

#include <gnuradio/uwb/uwb_twr_fake_link.h>

#include <algorithm>
#include <cmath>
#include <utility>

namespace gr {
namespace uwb {
namespace twr {

FakeTwrLink::FakeTwrLink() = default;
FakeTwrLink::~FakeTwrLink() = default;

const ClockDomain& FakeTwrLink::domain_for(uint8_t endpoint) const
{
    return endpoint == kEndpointA ? d_cfg.domain_a : d_cfg.domain_b;
}

void FakeTwrLink::clear_state()
{
    for (size_t i = 0; i < kEndpointCount; ++i)
        d_queues[i].clear();
    d_submit_count[0] = 0;
    d_submit_count[1] = 0;
    d_delivery_count = 0;
    d_seq = 0;
    d_counters = Counters{};
}

void FakeTwrLink::reset_counters()
{
    d_submit_count[0] = 0;
    d_submit_count[1] = 0;
    d_delivery_count = 0;
    d_counters = Counters{};
}

bool FakeTwrLink::configure(const FakeLinkConfig& cfg, std::string& why)
{
    why.clear();
    if (!clock_domain_is_valid(cfg.domain_a)) {
        why = "domain_a is not a valid ClockDomain";
        return false;
    }
    if (!clock_domain_is_valid(cfg.domain_b)) {
        why = "domain_b is not a valid ClockDomain";
        return false;
    }
    if (cfg.tx_air_latency_a_ticks < 0) {
        why = "tx_air_latency_a_ticks must be >= 0";
        return false;
    }
    if (cfg.tx_air_latency_b_ticks < 0) {
        why = "tx_air_latency_b_ticks must be >= 0";
        return false;
    }
    if (cfg.rx_window_ticks < 0) {
        why = "rx_window_ticks must be >= 0";
        return false;
    }
    if (cfg.rx_queue_capacity == 0) {
        why = "rx_queue_capacity must be > 0";
        return false;
    }
    if (!std::isfinite(cfg.distance_m) || cfg.distance_m < 0.0) {
        why = "distance_m must be finite and >= 0";
        return false;
    }

    d_cfg = cfg;
    d_configured = true;
    clear_state();
    return true;
}

int64_t FakeTwrLink::propagation_ticks(uint8_t source) const
{
    if (!d_configured || source >= kEndpointCount)
        return 0;
    const ClockDomain& dest = domain_for(static_cast<uint8_t>(1u - source));
    const double seconds = d_cfg.distance_m / kFakeLinkSpeedOfLightMps;
    return static_cast<int64_t>(std::llround(seconds * dest.tick_rate_hz));
}

int64_t FakeTwrLink::compute_arrival(const FakeTx& tx) const
{
    const ClockDomain& src = domain_for(tx.source);
    const ClockDomain& dst = domain_for(static_cast<uint8_t>(1u - tx.source));
    const int64_t latency = tx.source == kEndpointA ? d_cfg.tx_air_latency_a_ticks
                                                    : d_cfg.tx_air_latency_b_ticks;
    const int64_t src_ticks = tx.air_ticks + latency;

    int64_t dest_ticks = 0;
    if (src.tick_rate_hz == dst.tick_rate_hz) {
        // Same-rate fast path: the grids are identical, so the planned instant
        // (plus source latency) is already a destination-grid coordinate.
        dest_ticks = src_ticks;
    } else {
        const double seconds = static_cast<double>(src_ticks) / src.tick_rate_hz;
        dest_ticks = static_cast<int64_t>(std::llround(seconds * dst.tick_rate_hz));
    }
    return dest_ticks + propagation_ticks(tx.source);
}

void FakeTwrLink::insert_sorted(uint8_t endpoint, Entry&& e)
{
    std::deque<Entry>& q = d_queues[endpoint];
    auto it = std::upper_bound(q.begin(), q.end(), e,
                               [](const Entry& a, const Entry& b) {
                                   if (a.arrival != b.arrival)
                                       return a.arrival < b.arrival;
                                   return a.seq < b.seq;
                               });
    q.insert(it, std::move(e));
}

FakeLinkError FakeTwrLink::submit(const FakeTx& tx, std::string& why)
{
    why.clear();

    if (!d_configured) {
        ++d_counters.rejected;
        why = "link is not configured";
        return FakeLinkError::NotConfigured;
    }
    if (tx.source >= kEndpointCount) {
        ++d_counters.rejected;
        why = "unknown source endpoint " + std::to_string(static_cast<unsigned>(tx.source));
        return FakeLinkError::UnknownEndpoint;
    }
    if (tx.nbytes == 0) {
        ++d_counters.rejected;
        why = "frame has zero bytes";
        return FakeLinkError::BadBytes;
    }
    if (tx.nbytes > kMaxFrameBytes) {
        ++d_counters.rejected;
        why = "frame of " + std::to_string(tx.nbytes) + " bytes exceeds kMaxFrameBytes=" +
              std::to_string(kMaxFrameBytes);
        return FakeLinkError::BadBytes;
    }
    if (tx.air_ticks < 0) {
        ++d_counters.rejected;
        why = "air_ticks must be >= 0";
        return FakeLinkError::BadTime;
    }

    const uint8_t dest = static_cast<uint8_t>(1u - tx.source);
    ++d_counters.submitted;
    ++d_submit_count[tx.source];

    const bool drop = d_cfg.drop_every_n_tx != 0 &&
                      (d_submit_count[tx.source] % d_cfg.drop_every_n_tx) == 0;
    const int64_t arrival = compute_arrival(tx);

    // Capacity is checked BEFORE any injection counter moves, so a refused
    // submit changes no observable injection state.
    if (d_queues[dest].size() >= d_cfg.rx_queue_capacity) {
        ++d_counters.queue_full;
        why = "destination RX queue is full (capacity " +
              std::to_string(d_cfg.rx_queue_capacity) + ")";
        return FakeLinkError::QueueFull;
    }

    Entry e;
    e.arrival = arrival;
    e.seq = d_seq++;
    e.rx.source = tx.source;
    e.rx.destination = dest;
    e.rx.token = tx.token;
    e.rx.rx_marker_ticks = arrival;

    if (drop) {
        // A dropped frame arrives as a decode event with no payload: the
        // receiver sees "something was not received", not a valid frame.
        e.rx.fcs_passed = false;
        e.rx.decode_ok = false;
        e.rx.nbytes = 0;
        e.rx.injected_drop = true;
        e.rx.first_path = FirstPathQuality::not_recorded();
        e.rx.note = "injected drop (drop_every_n_tx)";
        ++d_counters.injected_drops;
    } else {
        for (size_t i = 0; i < tx.nbytes; ++i)
            e.rx.bytes[i] = tx.bytes[i];
        e.rx.nbytes = tx.nbytes;
        e.rx.fcs_passed = true;
        e.rx.decode_ok = true;
        e.rx.first_path = fake_first_path_default();
        ++d_delivery_count;
        if (d_cfg.bad_fcs_every_n_rx != 0 &&
            (d_delivery_count % d_cfg.bad_fcs_every_n_rx) == 0) {
            // Bytes are preserved: a bad-FCS frame is still a parseable frame
            // for the codec's error path, it is the FCS verdict that fails.
            e.rx.fcs_passed = false;
            e.rx.note = "injected bad FCS (bad_fcs_every_n_rx)";
            ++d_counters.injected_bad_fcs;
        }
    }

    insert_sorted(dest, std::move(e));
    ++d_counters.delivered;
    return FakeLinkError::Ok;
}

bool FakeTwrLink::next_arrival(uint8_t endpoint, int64_t& ticks) const
{
    if (endpoint >= kEndpointCount || d_queues[endpoint].empty())
        return false;
    ticks = d_queues[endpoint].front().arrival;
    return true;
}

bool FakeTwrLink::pop_rx(uint8_t endpoint, FakeRx& out)
{
    if (endpoint >= kEndpointCount || d_queues[endpoint].empty())
        return false;
    out = std::move(d_queues[endpoint].front().rx);
    d_queues[endpoint].pop_front();
    return true;
}

size_t FakeTwrLink::pending(uint8_t endpoint) const
{
    if (endpoint >= kEndpointCount)
        return 0;
    return d_queues[endpoint].size();
}

} // namespace twr
} // namespace uwb
} // namespace gr
