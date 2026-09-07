/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * EchoTimer device-time scheduling core (Radar Step 10).  Header-only,
 * not a GNU Radio block.
 *
 * All schedule geometry lives in integer device sample ticks (one tick =
 * one native sample, e.g. 1/737.28e6 s).  No doubles participate: a PRI
 * with a fractional tick part is represented exactly as the rational
 *
 *     ticks_per_pri = pri_num / pri_den          (pri_den > 0)
 *
 * and the grid advances slot-by-slot keeping
 *
 *     t_tx(k) = whole(k) + rem(k) / pri_den      [device ticks]
 *
 * with 0 <= rem < pri_den maintained in integers (quotient/remainder
 * update).  The grid is therefore exact for arbitrarily large schedule
 * indices — bounded only by the int64 whole-tick range (centuries at any
 * realistic sample rate) — and no cumulative drift can accumulate.
 *
 *     t_tx(k) = t0 + (k - index_start) * ticks_per_pri
 *     t_rx(k) = t_tx(k) - pre_guard_ticks        (RX commands EARLIER than TX)
 *
 * Expired slots are skipped, never caught up: next_schedule() returns the
 * first index whose t_tx is strictly in the future with respect to the
 * backend device time (`now`).
 *
 * Checked integer arithmetic mirrors uwb_radar_checked_math.h; any
 * overflow is an explicit failure status, never undefined behaviour.
 * Fragment planning mirrors the scratch-freeze pattern of
 * uwb_radar_cir_estimator.h: all configuration is validated once in
 * prepare_echo_scheduler() and frozen into EchoSchedulerPrepared; the
 * per-burst hot path performs no allocation.
 */

#ifndef INCLUDED_GNURADIO_UWB_UWB_ECHO_SCHEDULER_CORE_H
#define INCLUDED_GNURADIO_UWB_UWB_ECHO_SCHEDULER_CORE_H

#include <gnuradio/uwb/uwb_radar_checked_math.h>

#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>

namespace gr {
namespace uwb {
namespace echo {

// Hard cap shared by the scheduler core and backends so burst fragment
// lists can live in fixed-size (allocation-free) arrays.
inline constexpr size_t kEchoMaxFragmentsPerBurst = 64;

enum class EchoScheduleStatus : uint8_t {
    Ok = 0,                // slot returned, t_tx > now
    ExpiredSkipped = 1,    // slot returned after skipping expired slots
    NotArmed = 2,          // grid has no t0
    SkipLimitExceeded = 3, // more than max_catchup_slots slots expired
    GridOverflow = 4,      // checked-integer failure advancing the grid
};

// One planned burst fragment (offset/count inside the burst).  Flag
// assignment (time spec + SOB on the first fragment, EOB on the last)
// belongs to the caller; see uwb_echo_burst_backend.h for the flag
// contract.
struct EchoFragmentSpan {
    uint64_t offset = 0;
    uint64_t count = 0;
};

struct EchoSlot {
    uint64_t index = 0;
    uint64_t skipped = 0;   // expired slots skipped to reach this slot
    int64_t t_tx_whole = 0; // t_tx = t_tx_whole + t_tx_rem_num / t_tick_den
    int64_t t_tx_rem_num = 0;
    int64_t t_tick_den = 1;
    int64_t t_rx_whole = 0; // t_rx = t_tx - pre_guard_ticks (same fraction)
};

struct EchoSchedulerConfig {
    // PRI on the device tick grid as an exact rational (defaults to the
    // 5 ms radar PRI: 0.005 * 737.28e6 = 3686400 ticks).
    int64_t pri_num = 3686400;
    int64_t pri_den = 1;
    // t_rx = t_tx - pre_guard_ticks (defaults to 2 µs at 737.28 MS/s).
    int64_t pre_guard_ticks = 1475;
    // Upper bound on expired slots skipped in a single next_schedule()
    // call (protects the worker after an arbitrarily long stall).
    uint64_t max_catchup_slots = 1u << 20;
    // Default burst fragment planning chunk (elements per TX/RX fragment).
    uint64_t max_fragment_size = 65536;
};

// Frozen identity produced once by prepare_echo_scheduler().  Mirrors the
// prepared-identity pattern of the radar cores: validation happens at
// make/config time, the struct is read-only afterwards, and the per-burst
// path performs no allocation and no re-validation.
struct EchoSchedulerPrepared {
    int64_t pri_num = 0;
    int64_t pri_den = 0;
    int64_t pre_guard_ticks = 0;
    int64_t pri_whole = 0; // pri = pri_whole + pri_rem / pri_den
    int64_t pri_rem = 0;
    uint64_t max_catchup_slots = 0;
    uint64_t max_fragment_size = 0;
};

inline bool prepare_echo_scheduler(const EchoSchedulerConfig& cfg,
                                   EchoSchedulerPrepared& out,
                                   std::string* error = nullptr)
{
    auto fail = [&](const char* what) {
        if (error)
            *error = what;
        return false;
    };
    out = EchoSchedulerPrepared{};
    if (cfg.pri_num <= 0 || cfg.pri_den <= 0)
        return fail("pri_num and pri_den must be > 0");
    // Keep the fractional remainder well inside int64 so remainder
    // updates cannot overflow.
    if (cfg.pri_den > (int64_t(1) << 62))
        return fail("pri_den too large (must be <= 2^62)");
    if (cfg.pre_guard_ticks < 0)
        return fail("pre_guard_ticks must be >= 0");
    // RX windows must not overlap: pre_guard < pri (exact rational compare).
    int64_t pg_scaled = 0;
    if (!radar::radar_i64_mul(cfg.pre_guard_ticks, cfg.pri_den, pg_scaled))
        return fail("pre_guard_ticks overflow");
    if (pg_scaled >= cfg.pri_num)
        return fail("pre_guard_ticks must be smaller than the PRI");
    if (cfg.max_catchup_slots == 0)
        return fail("max_catchup_slots must be > 0");
    if (cfg.max_fragment_size == 0)
        return fail("max_fragment_size must be > 0");

    out.pri_num = cfg.pri_num;
    out.pri_den = cfg.pri_den;
    out.pre_guard_ticks = cfg.pre_guard_ticks;
    out.pri_whole = cfg.pri_num / cfg.pri_den;
    out.pri_rem = cfg.pri_num % cfg.pri_den;
    out.max_catchup_slots = cfg.max_catchup_slots;
    out.max_fragment_size = cfg.max_fragment_size;
    return true;
}

// Exact reference evaluation used by QA: the offset of schedule index k
// from t0 is k * num / den ticks, computed as an integer quotient and
// remainder.  Fails on checked-mul overflow (k * num not representable in
// int64) — the documented representation limit of the tick grid.
inline bool echo_slot_offset(uint64_t k,
                             int64_t num,
                             int64_t den,
                             int64_t& whole,
                             int64_t& rem)
{
    int64_t k64 = 0;
    if (!radar::radar_i64_from_u64(k, k64))
        return false;
    int64_t prod = 0;
    if (!radar::radar_i64_mul(k64, num, prod))
        return false;
    whole = prod / den;
    rem = prod % den;
    return true;
}

// Stateful device-time tick grid (owned by the scheduler worker thread;
// never touched from a GNU Radio handler).  `prepared` must outlive the
// grid — the block stores the prepared config as a member so this holds.
class EchoGrid
{
public:
    EchoGrid() = default;
    explicit EchoGrid(const EchoSchedulerPrepared& prepared)
        : d_p(&prepared)
    {
    }

    // (Re)arm at device tick t0.  t0 must leave room for the RX guard
    // (t_rx >= 0, checked); index_start is the schedule index of the
    // first slot.
    bool arm(int64_t t0_ticks, uint64_t index_start)
    {
        if (!d_p)
            return false;
        int64_t t_rx = 0;
        if (!radar::radar_i64_sub(t0_ticks, d_p->pre_guard_ticks, t_rx) ||
            t_rx < 0)
            return false;
        d_whole = t0_ticks;
        d_rem = 0;
        d_index = index_start;
        d_armed = true;
        return true;
    }

    bool armed() const { return d_armed; }
    uint64_t current_index() const { return d_index; }

    // Return the next schedule slot whose t_tx is strictly in the future
    // (t_tx > now).  Expired slots are skipped (no catch-up) up to
    // max_catchup_slots.  On success the grid has already advanced past
    // the returned slot, so repeated calls produce each index exactly
    // once, in order.
    EchoScheduleStatus next_schedule(int64_t now, EchoSlot& out)
    {
        out = EchoSlot{};
        if (!d_armed)
            return EchoScheduleStatus::NotArmed;

        // Skip expired slots (t_tx <= now); never catch up.
        uint64_t skipped = 0;
        while (!future(now)) {
            if (skipped >= d_p->max_catchup_slots)
                return EchoScheduleStatus::SkipLimitExceeded;
            if (!advance())
                return EchoScheduleStatus::GridOverflow;
            ++skipped;
        }

        out.index = d_index;
        out.skipped = skipped;
        out.t_tx_whole = d_whole;
        out.t_tx_rem_num = d_rem;
        out.t_tick_den = d_p->pri_den;
        if (!radar::radar_i64_sub(d_whole, d_p->pre_guard_ticks,
                                  out.t_rx_whole))
            return EchoScheduleStatus::GridOverflow;

        // Consume the slot so the next call yields index+1.
        if (!advance())
            return EchoScheduleStatus::GridOverflow;

        return skipped == 0 ? EchoScheduleStatus::Ok
                            : EchoScheduleStatus::ExpiredSkipped;
    }

private:
    // t_tx > now  ⟺  whole > now || (whole == now && rem > 0)
    bool future(int64_t now) const
    {
        return d_whole > now || (d_whole == now && d_rem > 0);
    }

    // t += pri (exact quotient/remainder update, checked).  The remainder
    // invariant 0 <= rem < pri_den <= 2^62 means the remainder add cannot
    // overflow; whole ticks are added through radar_i64_add.
    bool advance()
    {
        int64_t rem = d_rem + d_p->pri_rem;
        int64_t whole = d_whole;
        if (!radar::radar_i64_add(whole, d_p->pri_whole, whole))
            return false;
        if (rem >= d_p->pri_den) {
            if (!radar::radar_i64_add(whole, int64_t(1), whole))
                return false;
            rem -= d_p->pri_den;
        }
        d_whole = whole;
        d_rem = rem;
        if (d_index == std::numeric_limits<uint64_t>::max())
            return false; // schedule index space exhausted
        ++d_index;
        return true;
    }

    const EchoSchedulerPrepared* d_p = nullptr;
    uint64_t d_index = 0;
    int64_t d_whole = 0; // t_tx whole ticks for d_index
    int64_t d_rem = 0;   // fractional part, 0 <= d_rem < pri_den
    bool d_armed = false;
};

// Checked fragment planning: split `total` elements into spans of at most
// `max_fragment_size` elements.  Writes at most max_spans entries; fails
// when the burst would need more than kEchoMaxFragmentsPerBurst fragments
// (fixed-size fragment arrays).  Spans are contiguous and cover [0, total).
inline bool plan_fragments(uint64_t total,
                           uint64_t max_fragment_size,
                           EchoFragmentSpan* spans,
                           size_t max_spans,
                           size_t& span_count)
{
    span_count = 0;
    if (total == 0 || max_fragment_size == 0 || spans == nullptr)
        return false;
    uint64_t n = total / max_fragment_size;
    if (total % max_fragment_size != 0)
        ++n;
    if (n == 0 || n > static_cast<uint64_t>(max_spans) ||
        n > static_cast<uint64_t>(kEchoMaxFragmentsPerBurst))
        return false;
    uint64_t off = 0;
    for (uint64_t i = 0; i < n; ++i) {
        const uint64_t remaining = total - off;
        const uint64_t len =
            remaining > max_fragment_size ? max_fragment_size : remaining;
        spans[i].offset = off;
        spans[i].count = len;
        off += len; // bounded by total: no overflow
    }
    span_count = static_cast<size_t>(n);
    return true;
}

} // namespace echo
} // namespace uwb
} // namespace gr

#endif /* INCLUDED_GNURADIO_UWB_UWB_ECHO_SCHEDULER_CORE_H */
