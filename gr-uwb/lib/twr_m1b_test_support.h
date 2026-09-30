/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * M1-B test support (Agent D / m1b_fix_qa): header-only drivers shared by the
 * endpoint-core unit QA and the two-endpoint end-to-end QA.
 *
 * WHAT THIS IS
 * ------------
 *   * small, explicit builders for the clock/timestamp/calibration/binding
 *     values the QA needs, so every test states its own numbers rather than
 *     inheriting whatever the production code happens to do;
 *   * the R08 numeric `TxPlan` builder plus deliberately-broken variants, so a
 *     test can inject a plan whose internal mapping is wrong and assert that
 *     the core refuses it;
 *   * `TwoEndpointDriver`: wires two real `EndpointCore`s to one real
 *     `FakeTwrLink`.  It translates `SubmitTx` actions into `FakeTx`, turns
 *     `FakeRx` arrivals into `RxFrame` events, and services `Deadline` events
 *     from the most recent `ArmRx` deadline.  TX outcomes are resolved
 *     deterministically (documented below).
 *
 * WHAT THIS IS NOT
 * ----------------
 *   * It is NOT a second FSM and holds no expected values; the QA files hold
 *     the hand-computed / Fraction-derived expectations.
 *   * The ground-truth `distance_m` lives only in the link configuration; the
 *     driver never reads it into an endpoint event.  An inbound event carries
 *     exactly what a receiver could observe (bytes, FCS/decode verdict, the
 *     local marker instant and a first-path verdict).
 *
 * DETERMINISTIC TX PLANNING MODEL (stated once, used by both QA files)
 * -------------------------------------------------------------------
 * The "adapter" here is a deterministic stand-in for M2's real PHY adapter:
 *
 *   Poll     air = begin_now_ticks + poll_offset_ticks
 *   Response air = <local Poll RX instant>      + respond_turnaround_ticks
 *   Final    air = <local Response RX instant>  + final_turnaround_ticks
 *
 * The planned instant is what the core writes into the frame, so a symmetric
 * two-way propagation makes both SS and DS return exactly the one-way
 * propagation delay regardless of the turnarounds (see testdata/twr/m1b/
 * README.md for the derivation).  The link itself is the authority for the
 * arrival instants; the driver only forwards what the link computed.
 *
 * The R08 plan is a NUMERIC record.  This driver builds it with a real internal
 * mapping -- quantised_instant = air - marker_offset, command_time =
 * quantised - command_lead, calibrated_air = quantised + marker_offset -- so a
 * regression that drops the mapping or the identity check is caught here.
 */

#ifndef INCLUDED_GNURADIO_UWB_TWR_M1B_TEST_SUPPORT_H
#define INCLUDED_GNURADIO_UWB_TWR_M1B_TEST_SUPPORT_H

#include <gnuradio/uwb/uwb_twr_core.h>
#include <gnuradio/uwb/uwb_twr_fake_link.h>
#include <gnuradio/uwb/uwb_twr_frame.h>

#include <cstddef>
#include <cstdint>
#include <limits>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace gr {
namespace uwb {
namespace twr {
namespace m1b_test {

// ===========================================================================
// Builders
// ===========================================================================

// A TWR clock domain.  `bits` is the counter width; 40 is the phase-1 profile.
inline ClockDomain make_domain(const std::string& name,
                               double rate_hz,
                               uint64_t epoch = 1,
                               uint32_t bits = 40)
{
    ClockDomain d;
    ClockDomain::make(name, rate_hz, epoch, bits, d);
    return d;
}

inline FrameProfile make_profile(double unit_hz, uint8_t bits = 40)
{
    FrameProfile p;
    p.version = kFrameVersion;
    p.timestamp_bits = bits;
    p.timestamp_unit_hz = unit_hz;
    p.max_psdu_bytes = kPhyMaxPsduBytes;
    p.fcs_appended_by_modulation_layer = true;
    return p;
}

inline CalibrationStamp make_calibration(const std::string& id,
                                         uint64_t epoch,
                                         int64_t from_ticks,
                                         int64_t until_ticks)
{
    CalibrationStamp c;
    c.id = id;
    c.calibrated_epoch = epoch;
    c.valid_from_ticks = from_ticks;
    c.valid_until_ticks = until_ticks;
    c.applications.push_back(CalibrationApplication{ id, CalibrationResult::Applied });
    return c;
}

// A local RX instant: marker RMARKER_RX, HardwareMeasured, the full correction
// chain for that marker and a calibration id.
inline Timestamp make_rx_ts(int64_t ticks,
                            const ClockDomain& d,
                            const std::string& calibration_id)
{
    Timestamp ts;
    Timestamp::from_ticks(ticks, d, TimestampMarker::RmarkerRx,
                          TimestampSource::HardwareMeasured,
                          timestamp_required_corrections(TimestampMarker::RmarkerRx), ts);
    ts.calibration_id = calibration_id;
    return ts;
}

// A planned TX instant: marker RMARKER_TX, ScheduledCalibrated and the full TX
// correction chain.
inline Timestamp make_tx_ts(int64_t ticks,
                            const ClockDomain& d,
                            const std::string& calibration_id)
{
    Timestamp ts;
    Timestamp::from_ticks(ticks, d, TimestampMarker::RmarkerTx,
                          TimestampSource::ScheduledCalibrated,
                          timestamp_required_corrections(TimestampMarker::RmarkerTx), ts);
    ts.calibration_id = calibration_id;
    return ts;
}

// ===========================================================================
// R08: the numeric TxPlan and its deliberate defects
// ===========================================================================

// A fully-consistent plan whose RMARKER air instant is `calibrated_air_ticks`.
// The internal identity is calibrated = quantised + marker_offset.
inline TxPlan make_tx_plan(int64_t calibrated_air_ticks,
                           const ClockDomain& d,
                           const std::string& calibration_id,
                           int64_t marker_offset_ticks = 16,
                           int64_t command_lead_ticks = 32)
{
    TxPlan p;
    p.domain = d;
    p.source = TimestampSource::ScheduledCalibrated;
    p.applied_corrections = timestamp_required_corrections(TimestampMarker::RmarkerTx);
    p.calibration_id = calibration_id;
    p.marker_offset_ticks = marker_offset_ticks;
    p.quantised_instant_ticks = calibrated_air_ticks - marker_offset_ticks;
    p.calibrated_air_ticks = p.quantised_instant_ticks + marker_offset_ticks;
    p.command_time_ticks = p.quantised_instant_ticks - command_lead_ticks;
    p.valid = true;
    return p;
}

// Deliberately-broken plan variants.  Each names ONE way the R08 mapping can be
// wrong, so a negative case never masks another.
enum class TxPlanDefect {
    None = 0,
    NotValid,             // valid == false
    InvalidDomain,        // a default-constructed (invalid) ClockDomain
    SourceOutOfDomain,    // a source the enum does not have
    EmptyCalibrationId,   // no calibration id
    MissingCorrections,   // kCorrectionNone (RMARKER_TX requires three bits)
    MappingMismatch,      // calibrated != quantised + offset
    ReversedMapping,      // calibrated == quantised - offset (sign error)
    QuantisedNegative,    // the first quantised instant is negative
    TickOutOfDomainRange, // a tick at/above the domain's wrap period
    SumOverflow,          // quantised + offset overflows int64
    WrongEpoch,           // same counter identity, different epoch
    WrongDomainRate       // same name, different tick rate (different domain)
};

inline TxPlan make_tx_plan_defect(int64_t calibrated_air_ticks,
                                  const ClockDomain& d,
                                  const std::string& calibration_id,
                                  TxPlanDefect defect,
                                  int64_t marker_offset_ticks = 16,
                                  int64_t command_lead_ticks = 32)
{
    TxPlan p = make_tx_plan(calibrated_air_ticks, d, calibration_id, marker_offset_ticks,
                            command_lead_ticks);
    switch (defect) {
    case TxPlanDefect::None:
        break;
    case TxPlanDefect::NotValid:
        p.valid = false;
        break;
    case TxPlanDefect::InvalidDomain:
        p.domain = ClockDomain{};
        break;
    case TxPlanDefect::SourceOutOfDomain:
        p.source = static_cast<TimestampSource>(99);
        break;
    case TxPlanDefect::EmptyCalibrationId:
        p.calibration_id.clear();
        break;
    case TxPlanDefect::MissingCorrections:
        p.applied_corrections = kCorrectionNone;
        break;
    case TxPlanDefect::MappingMismatch:
        p.calibrated_air_ticks += 1;
        break;
    case TxPlanDefect::ReversedMapping:
        p.calibrated_air_ticks =
            p.quantised_instant_ticks - p.marker_offset_ticks;
        break;
    case TxPlanDefect::QuantisedNegative:
        p.quantised_instant_ticks = -5;
        p.marker_offset_ticks = 5;
        p.calibrated_air_ticks = 0;
        break;
    case TxPlanDefect::TickOutOfDomainRange: {
        const uint64_t period = d.wrap_period();
        if (period != 0u) {
            p.quantised_instant_ticks = static_cast<int64_t>(period);
            p.marker_offset_ticks = 16;
            p.calibrated_air_ticks = static_cast<int64_t>(period) + 16;
        } else {
            p.quantised_instant_ticks = (std::numeric_limits<int64_t>::max)() - 8;
            p.marker_offset_ticks = 8;
            p.calibrated_air_ticks = (std::numeric_limits<int64_t>::max)();
        }
        break;
    }
    case TxPlanDefect::SumOverflow:
        p.quantised_instant_ticks = 10;
        p.marker_offset_ticks = (std::numeric_limits<int64_t>::max)();
        p.calibrated_air_ticks = 10;
        break;
    case TxPlanDefect::WrongEpoch:
        p.domain.epoch_id = d.epoch_id + 1;
        break;
    case TxPlanDefect::WrongDomainRate:
        p.domain.tick_rate_hz = d.tick_rate_hz * 2.0;
        break;
    }
    return p;
}

// ---- CoreEvent builders --------------------------------------------------

inline CoreEvent make_begin(uint64_t exchange_value,
                            int64_t now_ticks,
                            uint64_t event_id = 0,
                            uint64_t generation = 1)
{
    CoreEvent e;
    e.kind = CoreEventKind::Begin;
    e.event_id = event_id;
    e.generation = generation;
    e.exchange.valid = true;
    e.exchange.value = exchange_value;
    e.now_ticks = now_ticks;
    return e;
}

inline CoreEvent make_deadline(int64_t now_ticks, uint64_t event_id = 0,
                               uint64_t generation = 1)
{
    CoreEvent e;
    e.kind = CoreEventKind::Deadline;
    e.now_ticks = now_ticks;
    e.event_id = event_id;
    e.generation = generation;
    return e;
}

inline CoreEvent make_cancel(ExchangeStatus reason = ExchangeStatus::Cancelled,
                             uint64_t event_id = 0,
                             uint64_t generation = 1)
{
    CoreEvent e;
    e.kind = CoreEventKind::Cancel;
    e.cancel_reason = reason;
    e.event_id = event_id;
    e.generation = generation;
    return e;
}

inline CoreEvent make_stop(uint64_t event_id = 0, uint64_t generation = 1)
{
    CoreEvent e;
    e.kind = CoreEventKind::Stop;
    e.event_id = event_id;
    e.generation = generation;
    return e;
}

// A local Reset EVENT.  `has_new_wire_session` is what actually clears the wire
// replay barrier (R04); a pure local generation bump must NOT.
inline CoreEvent make_reset(uint64_t new_generation,
                            uint64_t event_id = 0,
                            bool has_new_wire_session = false,
                            uint16_t new_wire_session_id = 0,
                            uint64_t generation = 1)
{
    CoreEvent e;
    e.kind = CoreEventKind::Reset;
    e.new_generation = new_generation;
    e.event_id = event_id;
    e.has_new_wire_session_id = has_new_wire_session;
    e.new_wire_session_id = new_wire_session_id;
    e.generation = generation;
    return e;
}

inline CoreEvent make_accepted(const TxToken& t, uint64_t event_id = 0,
                               uint64_t generation = 1)
{
    CoreEvent e;
    e.kind = CoreEventKind::TxAccepted;
    e.event_id = event_id;
    e.token = t;
    e.generation = generation;
    return e;
}

inline CoreEvent make_outcome(const TxToken& t,
                              TxOutcome outcome,
                              AdapterFault fault = AdapterFault::None,
                              uint64_t generation = 1,
                              uint64_t event_id = 0)
{
    CoreEvent e;
    e.kind = CoreEventKind::TxOutcomeResolved;
    e.event_id = event_id;
    e.token = t;
    e.tx_outcome = outcome;
    e.adapter_fault = fault;
    e.generation = generation;
    return e;
}

// A TxPlanned event carrying an ARBITRARY plan (the injection point for the R08
// negative cases).
inline CoreEvent make_tx_planned_raw(const TxToken& token,
                                     FrameType intent,
                                     const TxPlan& plan,
                                     bool deadline_verdict_feasible = true,
                                     int64_t deadline_slack_ticks = 0,
                                     uint64_t event_id = 0,
                                     uint64_t generation = 1)
{
    CoreEvent e;
    e.kind = CoreEventKind::TxPlanned;
    e.event_id = event_id;
    e.generation = generation;
    e.token = token;
    e.tx_intent = intent;
    e.tx_plan = plan;
    e.deadline_verdict_feasible = deadline_verdict_feasible;
    e.deadline_slack_ticks = deadline_slack_ticks;
    return e;
}

// The common case: a consistent plan for `calibrated_air_ticks`, optionally
// broken by `defect`.  The plan carries NO sent/completed state (R01); only a
// later `TxOutcomeResolved` can complete the transmit.
inline CoreEvent make_tx_planned(const TxToken& token,
                                 FrameType intent,
                                 int64_t calibrated_air_ticks,
                                 const ClockDomain& d,
                                 const std::string& calibration_id,
                                 TxPlanDefect defect = TxPlanDefect::None,
                                 bool deadline_verdict_feasible = true,
                                 int64_t deadline_slack_ticks = 0,
                                 uint64_t event_id = 0,
                                 uint64_t generation = 1,
                                 int64_t marker_offset_ticks = 16,
                                 int64_t command_lead_ticks = 32)
{
    const TxPlan plan = make_tx_plan_defect(calibrated_air_ticks, d, calibration_id,
                                             defect, marker_offset_ticks,
                                             command_lead_ticks);
    return make_tx_planned_raw(token, intent, plan, deadline_verdict_feasible,
                               deadline_slack_ticks, event_id, generation);
}

// Build an RxFrame event from already-decoded bytes.  `decode_ok`/`fcs_passed`
// are the PHY verdicts; the frame is decoded here with the endpoint's own
// profile so the caller states bytes exactly as a receiver would see them.
inline CoreEvent make_rx_event(const uint8_t* bytes,
                               size_t nbytes,
                               const FrameProfile& profile,
                               int64_t rx_ticks,
                               const ClockDomain& d,
                               const std::string& calibration_id,
                               const FirstPathQuality& first_path,
                               bool fcs_passed = true,
                               bool decode_ok = true,
                               uint64_t event_id = 0,
                               uint64_t generation = 1)
{
    CoreEvent e;
    e.kind = CoreEventKind::RxFrame;
    e.event_id = event_id;
    e.generation = generation;
    e.fcs_passed = fcs_passed;
    e.decode_ok = decode_ok;
    e.rx_time = make_rx_ts(rx_ticks, d, calibration_id);
    e.rx_first_path = first_path;
    if (decode_ok && nbytes == 0) {
        e.decode_ok = false;
    }
    if (e.decode_ok) {
        std::string err;
        if (!decode(bytes, nbytes, profile, e.frame, err)) {
            e.decode_ok = false;
        }
    }
    return e;
}

// Stamp the endpoint's CURRENT session generation into an event and post it.
// The core requires every event to name the live generation (R04), so a test
// that resets must post subsequent events through this helper rather than
// hard-coding a generation.
inline CoreActionBatch post_now(EndpointCore& c, CoreEvent e)
{
    e.generation = c.config().session_generation;
    return c.post(e);
}

inline WireTimestampBinding make_binding(const ClockDomain& peer_domain,
                                         uint64_t session_generation = 1,
                                         uint64_t binding_generation = 1,
                                         uint64_t max_interval_ticks = 1000000,
                                         uint16_t sequence_modulus = 4,
                                         TimestampMarker peer_marker = TimestampMarker::RmarkerTx)
{
    WireTimestampBinding b;
    b.peer_domain = peer_domain;
    b.session_generation = session_generation;
    b.binding_generation = binding_generation;
    b.peer_marker = peer_marker;
    b.unit_convention = "peer_device_ticks";
    b.calibration_convention_id = "twr-link-cal-v1";
    b.max_interval_ticks = max_interval_ticks;
    b.sequence_modulus = sequence_modulus;
    return b;
}

// ===========================================================================
// Two-endpoint driver
// ===========================================================================

struct DriverConfig {
    CoreConfig cfg_a;
    CoreConfig cfg_b;
    FakeLinkConfig link;

    int64_t poll_offset_ticks = 1000;
    int64_t respond_turnaround_ticks = 500;
    int64_t final_turnaround_ticks = 300;

    // Resolve every accepted TX as Completed immediately after SubmitTx with a
    // modelled completion event (never on accepted/ACK alone).
    bool auto_resolve_completed = true;

    // Precise single-frame drop injection: (physical endpoint, 1-based submit
    // index for that source).  A dropped frame is handed to the core's adapter
    // bookkeeping but never reaches the link, i.e. it is "lost on the air".
    std::vector<std::pair<uint8_t, uint32_t>> drop_submits;
};

struct TerminalRecord {
    ExchangeStatus status = ExchangeStatus::InternalError;
    ProtocolTofEstimate result;
    std::string detail;
    FrameType last_intent = FrameType::Poll;
};

class TwoEndpointDriver
{
public:
    TwoEndpointDriver()
    {
        cores_[kEndpointA] = &core_a_;
        cores_[kEndpointB] = &core_b_;
    }

    bool configure(const DriverConfig& cfg, std::string& why)
    {
        cfg_ = cfg;
        why.clear();
        if (!link_.configure(cfg.link, why))
            return false;
        if (!core_a_.configure(cfg.cfg_a, why))
            return false;
        if (!core_b_.configure(cfg.cfg_b, why))
            return false;
        return true;
    }

    EndpointCore& core(uint8_t p) { return *cores_[p]; }
    const EndpointCore& core(uint8_t p) const { return *cores_[p]; }
    FakeTwrLink& link() { return link_; }
    const DriverConfig& config() const { return cfg_; }

    const std::vector<Frame>& sent_frames(uint8_t p) const { return sent_[p]; }
    const std::vector<TerminalRecord>& terminals(uint8_t p) const { return terminals_[p]; }
    uint32_t submit_count(uint8_t p) const { return submit_count_[p]; }
    uint32_t dropped_count(uint8_t p) const { return dropped_[p]; }
    int64_t last_local_ticks(uint8_t p) const { return last_local_[p]; }

    // Post an event and fully process the resulting action cascade.
    CoreActionBatch post(uint8_t p, const CoreEvent& ev)
    {
        // R04: the core requires every event to name the live generation.  A
        // driver-level event with generation 0 means "unspecified" and is
        // stamped here; an explicit generation is preserved so a stale-event
        // test still works.
        CoreEvent stamped = ev;
        if (stamped.generation == 0)
            stamped.generation = core(p).config().session_generation;
        const CoreActionBatch b = core(p).post(stamped);
        process(p, b);
        return b;
    }

    // Start an exchange at the endpoint configured as initiator.
    bool begin(uint8_t initiator, int64_t now_ticks, uint64_t exchange_value)
    {
        if (initiator >= kEndpointCount)
            return false;
        last_local_[initiator] = now_ticks;
        post(initiator, make_begin(exchange_value, now_ticks));
        return true;
    }

    // Pop one queued arrival into its destination endpoint and process it.
    bool deliver_one_rx(uint8_t dest)
    {
        FakeRx rx;
        if (!link_.pop_rx(dest, rx))
            return false;
        last_local_[dest] = rx.rx_marker_ticks;
        post(dest, rx_to_event(dest, rx));
        return true;
    }

    // Service every queued RX and every outstanding ArmRx deadline in time
    // order until the link and the deadline table are empty.  A stalled
    // in-flight exchange with no registered deadline is reported, not hidden.
    bool run_until_idle(uint32_t max_steps, std::string& why)
    {
        for (uint32_t i = 0; i < max_steps; ++i) {
            if (!dispatch_one(why))
                return true;
        }
        why = "run_until_idle exceeded the step budget";
        return false;
    }

    void clear_pending_deadline(uint8_t p) { deadline_[p] = 0; }

private:
    bool dispatch_one(std::string& why)
    {
        bool has_rx[kEndpointCount] = { false, false };
        int64_t rx_t[kEndpointCount] = { 0, 0 };
        bool has_dl[kEndpointCount] = { false, false };
        int64_t dl_t[kEndpointCount] = { 0, 0 };
        for (uint8_t p = 0; p < kEndpointCount; ++p) {
            has_rx[p] = link_.next_arrival(p, rx_t[p]);
            if (deadline_[p] != 0) {
                has_dl[p] = true;
                dl_t[p] = deadline_[p];
            }
        }
        bool any = false;
        for (uint8_t p = 0; p < kEndpointCount; ++p)
            any = any || has_rx[p] || has_dl[p];
        if (!any) {
            for (uint8_t p = 0; p < kEndpointCount; ++p) {
                if (core(p).in_flight() != 0) {
                    why = std::string("endpoint ") + (p == kEndpointA ? "A" : "B") +
                          " is in flight but has no RX or deadline pending";
                    return false;
                }
            }
            return false; // quiescent
        }

        // Earliest external event wins; a deadline at exactly the same tick
        // fires first (the core's own test is `now >= deadline`).
        uint8_t pick = 0;
        bool pick_is_deadline = false;
        bool have_pick = false;
        for (uint8_t p = 0; p < kEndpointCount; ++p) {
            if (has_dl[p] && (!have_pick || dl_t[p] <= (pick_is_deadline ? dl_t[pick] : rx_t[pick]))) {
                pick = p;
                pick_is_deadline = true;
                have_pick = true;
            }
            if (has_rx[p] && (!have_pick || rx_t[p] < (pick_is_deadline ? dl_t[pick] : rx_t[pick]))) {
                pick = p;
                pick_is_deadline = false;
                have_pick = true;
            }
        }
        if (!have_pick)
            return false;

        if (pick_is_deadline) {
            const int64_t now = dl_t[pick];
            deadline_[pick] = 0;
            post(pick, make_deadline(now));
            return true;
        }
        return deliver_one_rx(pick);
    }

    CoreEvent rx_to_event(uint8_t dest, const FakeRx& rx)
    {
        const CoreConfig& cfg = (dest == kEndpointA) ? cfg_.cfg_a : cfg_.cfg_b;
        CoreEvent e;
        e.kind = CoreEventKind::RxFrame;
        e.generation = cfg.session_generation;
        e.fcs_passed = rx.fcs_passed;
        e.decode_ok = rx.decode_ok;
        e.rx_time = make_rx_ts(rx.rx_marker_ticks, cfg.local_domain,
                               cfg.local_calibration.id);
        e.rx_first_path = rx.first_path;
        if (rx.decode_ok && rx.nbytes > 0) {
            std::string err;
            if (!decode(rx.bytes, rx.nbytes, cfg.frame_profile, e.frame, err)) {
                e.decode_ok = false;
            }
        } else {
            e.decode_ok = false;
        }
        return e;
    }

    void process(uint8_t p, const CoreActionBatch& batch)
    {
        for (size_t i = 0; i < batch.count; ++i) {
            const CoreAction& a = batch.actions[i];
            switch (a.kind) {
            case CoreActionKind::ArmRx:
                if (a.deadline_ticks != 0)
                    deadline_[p] = a.deadline_ticks;
                break;
            case CoreActionKind::PrepareTx:
                process_prepare(p, a);
                break;
            case CoreActionKind::SubmitTx:
                process_submit(p, a);
                break;
            case CoreActionKind::AbortPending:
                break;
            case CoreActionKind::TerminalResult: {
                TerminalRecord r;
                // The product action now carries a strong `ProtocolTerminalStatus`.
                // This TEST helper maps it back to the exchange vocabulary it
                // asserts on; the product API itself no longer exposes a
                // range-compatible status (F's finding B-1).
                ExchangeStatus mapped = ExchangeStatus::InternalError;
                if (a.status.failure_reason(mapped)) {
                    r.status = mapped;
                } else if (a.status.completion == ProtocolCompletionStatus::Complete) {
                    r.status = ExchangeStatus::Ok;
                } else {
                    r.status = ExchangeStatus::InternalError;
                }
                r.result = a.result;
                r.detail = a.detail;
                terminals_[p].push_back(r);
                deadline_[p] = 0;
                break;
            }
            }
        }
    }

    void process_prepare(uint8_t p, const CoreAction& a)
    {
        const CoreConfig& cfg = (p == kEndpointA) ? cfg_.cfg_a : cfg_.cfg_b;
        int64_t air = 0;
        switch (a.tx_intent) {
        case FrameType::Poll:
            air = last_local_[p] + cfg_.poll_offset_ticks;
            break;
        case FrameType::Response:
            air = last_local_[p] + cfg_.respond_turnaround_ticks;
            break;
        case FrameType::Final:
            air = last_local_[p] + cfg_.final_turnaround_ticks;
            break;
        case FrameType::Report:
            air = last_local_[p];
            break;
        }
        plan_air_[p][a.token.value] = air;
        // R08: the adapter returns a NUMERIC plan with a real internal mapping;
        // it carries no sent/completed state (R01).
        const CoreEvent planned = make_tx_planned(a.token, a.tx_intent, air,
                                                  cfg.local_domain,
                                                  cfg.local_calibration.id);
        post(p, planned);
    }

    void process_submit(uint8_t p, const CoreAction& a)
    {
        const CoreConfig& cfg = (p == kEndpointA) ? cfg_.cfg_a : cfg_.cfg_b;
        // The planned air instant was recorded when the plan was posted.
        int64_t air = 0;
        {
            const auto it = plan_air_[p].find(a.token.value);
            if (it != plan_air_[p].end())
                air = it->second;
        }
        FakeTx tx;
        tx.source = p;
        tx.token = a.token.value;
        tx.nbytes = a.nbytes;
        for (size_t i = 0; i < a.nbytes; ++i)
            tx.bytes[i] = a.bytes[i];
        tx.air_ticks = air;
        submitted_.push_back(std::make_pair(p, tx));

        // Observer copy (decode again for field assertions).
        Frame f;
        std::string err;
        if (decode(a.bytes, a.nbytes, cfg.frame_profile, f, err))
            sent_[p].push_back(f);

        ++submit_count_[p];
        bool drop = false;
        for (const auto& d : cfg_.drop_submits) {
            if (d.first == p && d.second == submit_count_[p])
                drop = true;
        }
        if (drop) {
            ++dropped_[p];
        } else {
            std::string why;
            link_.submit(tx, why);
        }

        if (cfg_.auto_resolve_completed) {
            post(p, make_accepted(a.token));
            post(p, make_outcome(a.token, TxOutcome::Completed));
        }
    }

    DriverConfig cfg_;
    EndpointCore core_a_;
    EndpointCore core_b_;
    EndpointCore* cores_[kEndpointCount] = { nullptr, nullptr };
    FakeTwrLink link_;

    std::map<uint64_t, int64_t> plan_air_[kEndpointCount];
    int64_t last_local_[kEndpointCount] = { 0, 0 };
    int64_t deadline_[kEndpointCount] = { 0, 0 };
    uint32_t submit_count_[kEndpointCount] = { 0, 0 };
    uint32_t dropped_[kEndpointCount] = { 0, 0 };
    std::vector<Frame> sent_[kEndpointCount];
    std::vector<std::pair<uint8_t, FakeTx>> submitted_;
    std::vector<TerminalRecord> terminals_[kEndpointCount];
};

} // namespace m1b_test
} // namespace twr
} // namespace uwb
} // namespace gr

#endif /* INCLUDED_GNURADIO_UWB_TWR_M1B_TEST_SUPPORT_H */
