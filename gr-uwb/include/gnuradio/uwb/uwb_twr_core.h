/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * M1-B: the SS-TWR / DS-TWR per-endpoint protocol core.
 *
 * ---------------------------------------------------------------------------
 * What this is
 * ---------------------------------------------------------------------------
 *
 * A pure C++ object -- NOT a GNU Radio block.  It includes no GNU Radio, no
 * PMT and no UHD; it never reads a system clock, never sleeps and never does
 * I/O.  Time advances only because an input EVENT says so, which is what makes
 * the two-endpoint simulation deterministic and reproducible (M1-B §4.1).
 *
 * Each endpoint owns exactly one `EndpointCore`.  An exchange advances ONLY
 * when a frame that passed FCS AND matched address / session / sequence /
 * message type / current state arrives; the endpoint never uses a shared
 * schedule, the true distance, or any private peer event (REQ-PROTO-01/03).
 *
 * The state machine is receive-driven: the TX of a Response/Final exists
 * because a VALID RX frame was accepted, not because a timer fired.
 *
 * ---------------------------------------------------------------------------
 * The causal TX-token state table (G0, M1-B §4.2)
 * ---------------------------------------------------------------------------
 *
 *   core -> adapter : PrepareTx(token, intent)
 *   adapter -> core : TxPlanned(token, plan)        // quantised instant,
 *                                                    // command time, marker
 *                                                    // offset, calibrated air
 *                                                    // time, deadline verdict
 *   core  -> adapter: SubmitTx(token)               // exactly once
 *   adapter -> core : TxAccepted(token)             // send() returned
 *   adapter -> core : TxOutcomeResolved(token, o)   // Completed / Late / ...
 *
 * A token is created by `PrepareTx` and is the ONLY key that can resolve it.
 * Out-of-order, duplicate, unknown or stale tokens NEVER rewrite the plan,
 * never produce a second Submit, and never advance the exchange; they are
 * counted as contract violations (or attributed to the stated reason).
 *
 * `TxAccepted` (and a burst ACK) is NOT completion.  A frame may be BUILT from
 * an unresolved plan -- delayed TX needs the planned instant in the frame --
 * but a successful terminal RESULT waits until the local TX evidence has
 * converged (`Completed`), or fails when the evidence deadline expires.
 *
 * ---------------------------------------------------------------------------
 * Local evidence vs peer wire claims
 * ---------------------------------------------------------------------------
 *
 * Local RX instants go through the strict M0.1 ranging gate
 * (`admit_ranging_interval`) before any formula sees them.  Peer instants can
 * only ever be `PeerTimestampClaim`s interpreted through a
 * `WireTimestampBinding`, and the protocol formulas consume them via
 * `ProtocolInterval` / `ProtocolTofEstimate` (see uwb_twr_protocol_time.h).
 * The two never mix: a wire claim cannot become an `AdmittedRangingInterval`,
 * and a `ProtocolTofEstimate` cannot become a validated range.
 *
 * ---------------------------------------------------------------------------
 * N07
 * ---------------------------------------------------------------------------
 *
 * Every enum here has an `is_known()` switch with NO `default`, tested BEFORE
 * the value is consumed.  An out-of-domain event/action/fault is REPORTED as a
 * failed action/terminal result, never thrown and never reasoned about.
 */

#ifndef INCLUDED_GNURADIO_UWB_UWB_TWR_CORE_H
#define INCLUDED_GNURADIO_UWB_UWB_TWR_CORE_H

#include <gnuradio/uwb/uwb_twr_config.h>
#include <gnuradio/uwb/uwb_twr_frame.h>
#include <gnuradio/uwb/uwb_twr_protocol_time.h>

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace gr {
namespace uwb {
namespace twr {

// ===========================================================================
// Identifiers
// ===========================================================================

// A runtime exchange identity, separate from the short wire `seq`.  `value`
// is reserved by the caller and must be non-zero; a retry allocates a NEW
// identity and never reuses the failed attempt's timestamps.
struct ExchangeId {
    uint64_t value = 0;
    bool valid = false;

    bool operator==(const ExchangeId& o) const
    {
        return valid == o.valid && value == o.value;
    }
};

// A causal transmit token.  Created by `PrepareTx`, consumed by exactly one
// `SubmitTx` and resolved by `TxAccepted` / `TxOutcomeResolved`.
struct TxToken {
    uint64_t value = 0;
    bool valid = false;

    bool operator==(const TxToken& o) const { return valid == o.valid && value == o.value; }
};

// ===========================================================================
// Input events
// ===========================================================================

enum class CoreEventKind : uint8_t {
    Begin = 0,             // initiator asks to start one exchange
    RxFrame = 1,           // a decoded frame + the local RX instant
    TxPlanned = 2,         // adapter returns the quantised plan for a token
    TxAccepted = 3,        // send() returned / burst ACKed (NOT completion)
    TxOutcomeResolved = 4, // the async outcome of one transmit
    Deadline = 5,          // "now is this many local ticks"
    Cancel = 6,            // cancel one exchange (or the listener)
    Stop = 7,              // stop the endpoint: drain in-flight as cancelled
    Overflow = 8,          // the adapter dropped an event / overflowed
    Reset = 9              // a local reset: new session generation
};

inline const char* core_event_kind_to_string(CoreEventKind k)
{
    switch (k) {
    case CoreEventKind::Begin:
        return "begin";
    case CoreEventKind::RxFrame:
        return "rx_frame";
    case CoreEventKind::TxPlanned:
        return "tx_planned";
    case CoreEventKind::TxAccepted:
        return "tx_accepted";
    case CoreEventKind::TxOutcomeResolved:
        return "tx_outcome_resolved";
    case CoreEventKind::Deadline:
        return "deadline";
    case CoreEventKind::Cancel:
        return "cancel";
    case CoreEventKind::Stop:
        return "stop";
    case CoreEventKind::Overflow:
        return "overflow";
    case CoreEventKind::Reset:
        return "reset";
    }
    return "invalid";
}

inline bool core_event_kind_is_known(CoreEventKind k)
{
    switch (k) {
    case CoreEventKind::Begin:
    case CoreEventKind::RxFrame:
    case CoreEventKind::TxPlanned:
    case CoreEventKind::TxAccepted:
    case CoreEventKind::TxOutcomeResolved:
    case CoreEventKind::Deadline:
    case CoreEventKind::Cancel:
    case CoreEventKind::Stop:
    case CoreEventKind::Overflow:
    case CoreEventKind::Reset:
        return true;
    }
    return false;
}

// Async adapter faults that are NOT members of `TxOutcome` (M1-B §4.2).  They
// are mapped onto their own `ExchangeStatus` instead of being cast into an
// out-of-domain `TxOutcome`.
enum class AdapterFault : uint8_t {
    None = 0,
    Late = 1,
    Underflow = 2,
    SeqError = 3,
    ChainBroken = 4,
    Cancelled = 5,
    RxOverflow = 6,
    Unattributable = 7
};

inline const char* adapter_fault_to_string(AdapterFault f)
{
    switch (f) {
    case AdapterFault::None:
        return "none";
    case AdapterFault::Late:
        return "late";
    case AdapterFault::Underflow:
        return "underflow";
    case AdapterFault::SeqError:
        return "seq_error";
    case AdapterFault::ChainBroken:
        return "chain_broken";
    case AdapterFault::Cancelled:
        return "cancelled";
    case AdapterFault::RxOverflow:
        return "rx_overflow";
    case AdapterFault::Unattributable:
        return "unattributable";
    }
    return "invalid";
}

inline bool adapter_fault_is_known(AdapterFault f)
{
    switch (f) {
    case AdapterFault::None:
    case AdapterFault::Late:
    case AdapterFault::Underflow:
    case AdapterFault::SeqError:
    case AdapterFault::ChainBroken:
    case AdapterFault::Cancelled:
    case AdapterFault::RxOverflow:
    case AdapterFault::Unattributable:
        return true;
    }
    return false;
}

inline ExchangeStatus adapter_fault_to_exchange_status(AdapterFault f)
{
    if (!adapter_fault_is_known(f))
        return ExchangeStatus::InternalError;
    switch (f) {
    case AdapterFault::None:
        return ExchangeStatus::Ok;
    case AdapterFault::Late:
        return ExchangeStatus::TxLate;
    case AdapterFault::Underflow:
        return ExchangeStatus::TxUnderflow;
    case AdapterFault::SeqError:
        return ExchangeStatus::TxSeqError;
    case AdapterFault::ChainBroken:
        return ExchangeStatus::TxChainBroken;
    case AdapterFault::Cancelled:
        return ExchangeStatus::Cancelled;
    case AdapterFault::RxOverflow:
        return ExchangeStatus::RxOverflow;
    case AdapterFault::Unattributable:
        return ExchangeStatus::InternalError;
    }
    return ExchangeStatus::InternalError;
}

// One input event.  A tagged record rather than a union: every field is
// explicit, and a field that is not meaningful for the active kind is simply
// ignored.  `event_id` is caller bookkeeping and is echoed back in the action
// batch so a driver can correlate.
struct CoreEvent {
    CoreEventKind kind = CoreEventKind::Begin;
    uint64_t event_id = 0;

    // ---- Begin -----------------------------------------------------------
    ExchangeId exchange;

    // ---- RxFrame ---------------------------------------------------------
    bool fcs_passed = false;
    bool decode_ok = false;
    Frame frame;
    // The LOCAL receive instant: marker RMARKER_RX, source HardwareMeasured,
    // full correction chain and calibration id.  The adapter owns this; the
    // core never invents one.
    Timestamp rx_time;
    // The local first-path DECISION for this reception.  Default is
    // NotRecorded, which the strict gate refuses.
    FirstPathQuality rx_first_path;

    // ---- TxPlanned -------------------------------------------------------
    TxToken token;
    FrameType tx_intent = FrameType::Poll;
    // The planned transmit instant: marker RMARKER_TX, source
    // ScheduledCalibrated, the full TX correction chain, the SAME
    // calibration_id as `local_calibration`, and the four planned records
    // present.  The core builds the frame from THIS value, never from "now".
    Timestamp planned_tx_time;
    TxSendEvidence tx_evidence;
    // The adapter's verdict about the remaining host deadline budget.  The
    // core only consumes it; it does not read a clock.
    bool deadline_verdict_feasible = false;
    int64_t deadline_slack_ticks = 0;

    // ---- TxOutcomeResolved ----------------------------------------------
    TxOutcome tx_outcome = TxOutcome::Unknown;
    AdapterFault adapter_fault = AdapterFault::None;

    // ---- Deadline / Reset -------------------------------------------------
    int64_t now_ticks = 0;
    uint64_t new_generation = 0;

    // ---- Cancel ----------------------------------------------------------
    ExchangeStatus cancel_reason = ExchangeStatus::Cancelled;
};

// ===========================================================================
// Output actions
// ===========================================================================

enum class CoreActionKind : uint8_t {
    ArmRx = 0,         // open the RX window / (re)state the driver's deadline
    PrepareTx = 1,     // ask the adapter to quantise a transmit plan
    SubmitTx = 2,      // hand the encoded frame to the adapter (exactly once)
    AbortPending = 3,  // drop the pending token/plan without submitting
    TerminalResult = 4 // one exchange ended; carries the protocol estimate
};

inline const char* core_action_kind_to_string(CoreActionKind k)
{
    switch (k) {
    case CoreActionKind::ArmRx:
        return "arm_rx";
    case CoreActionKind::PrepareTx:
        return "prepare_tx";
    case CoreActionKind::SubmitTx:
        return "submit_tx";
    case CoreActionKind::AbortPending:
        return "abort_pending";
    case CoreActionKind::TerminalResult:
        return "terminal_result";
    }
    return "invalid";
}

inline bool core_action_kind_is_known(CoreActionKind k)
{
    switch (k) {
    case CoreActionKind::ArmRx:
    case CoreActionKind::PrepareTx:
    case CoreActionKind::SubmitTx:
    case CoreActionKind::AbortPending:
    case CoreActionKind::TerminalResult:
        return true;
    }
    return false;
}

struct CoreAction {
    CoreActionKind kind = CoreActionKind::ArmRx;
    uint64_t event_id = 0;
    ExchangeId exchange;
    TxToken token;

    // ArmRx: the message this window expects.
    //
    // ArmRx is ALSO the driver's deadline channel: `deadline_ticks` is the
    // local tick at which this endpoint expects a Deadline event, and the
    // core may emit a fresh ArmRx to TIGHTEN it (e.g. when a result is owed
    // but a local transmit outcome has not converged, the evidence deadline
    // from `CoreConfig::evidence_wait_ticks` replaces the looser
    // whole-exchange deadline).  A compliant driver posts a Deadline at the
    // most recent `deadline_ticks` it was given.
    FrameType expect_type = FrameType::Response;
    // The local tick at which this wait must be given up (0 == no deadline).
    int64_t deadline_ticks = 0;

    // PrepareTx: the message the adapter must plan.
    FrameType tx_intent = FrameType::Poll;

    // SubmitTx: the encoded MAC payload (no FCS; the PHY layer appends it).
    uint8_t bytes[kMaxFrameBytes] = {};
    size_t nbytes = 0;

    // TerminalResult: the protocol outcome.  A separate strong type, so a
    // protocol completion can never be read as a validated range.
    ProtocolTofEstimate result;

    // TerminalResult: the exchange's terminal status.  A strong type whose
    // every range entry is false -- deliberately NOT an `ExchangeStatus`, so
    // `exchange_status_yields_range(action.status)` does not even compile
    // (F's review finding B-1).
    ProtocolTerminalStatus status;
    std::string detail;
};

// A bounded batch returned by `post()`.  The core allocates no queue per
// event; this fixed array IS the output.
inline constexpr size_t kCoreMaxActionsPerEvent = 4;

struct CoreActionBatch {
    CoreAction actions[kCoreMaxActionsPerEvent];
    size_t count = 0;

    void push(const CoreAction& a)
    {
        if (count < kCoreMaxActionsPerEvent)
            actions[count++] = a;
    }
};

// ===========================================================================
// Observability
// ===========================================================================

// Diagnostic counters.  `frames_*` count REJECTIONS separately so a soak
// report can tell noise from a wrong peer from a stale session without
// parsing prose (M1-B §4.3).  A noise frame does NOT create a measurement
// terminal and does NOT unconditionally cancel a correct in-flight request.
struct CoreCounters {
    uint64_t accepted_exchanges = 0;
    uint64_t terminal_results = 0;
    uint64_t in_flight_peak = 0;
    uint64_t requests_rejected_queue_full = 0;
    uint64_t frames_accepted = 0;
    uint64_t frames_rejected_fcs = 0;
    uint64_t frames_rejected_decode = 0;
    uint64_t frames_rejected_wrong_peer = 0;
    uint64_t frames_rejected_wrong_type = 0;
    uint64_t frames_rejected_session = 0;
    uint64_t frames_rejected_seq = 0;
    uint64_t frames_rejected_self = 0;
    uint64_t duplicate_frames = 0;
    uint64_t noise_frames = 0;
    uint64_t tx_prepared = 0;
    uint64_t tx_submitted = 0;
    uint64_t tx_submit_rejected_deadline = 0;
    uint64_t tx_accepted = 0;
    uint64_t tx_outcome_completed = 0;
    uint64_t tx_outcome_failed = 0;
    uint64_t tx_token_mismatch = 0;
    uint64_t deadline_events = 0;
    uint64_t timeouts = 0;
    uint64_t cancellations = 0;
    uint64_t resets = 0;
    uint64_t overflow_events = 0;
    uint64_t stale_events = 0;
    uint64_t events_rejected_out_of_domain = 0;
    uint64_t contract_violations = 0;
    uint64_t results_dropped = 0;

    std::string to_json_string() const;
};

// ===========================================================================
// Configuration snapshot
// ===========================================================================
//
// An immutable value captured at `configure()`.  It is the endpoint's whole
// execution authority: no live config object is consulted later.
struct CoreConfig {
    std::string endpoint_id; // "A" / "B"
    Protocol protocol = Protocol::Ss;
    Role role = Role::Initiator;

    uint16_t pan_id = 0;
    uint16_t local_address = 0;
    uint16_t peer_address = 0;

    uint16_t session_id = 0;       // short wire session
    uint64_t session_generation = 1; // local runtime generation (non-zero)

    uint16_t sequence_modulus = 4;
    uint16_t initial_sequence = 0;

    // This endpoint's own tick domain.  Local RX TX instants live here.
    ClockDomain local_domain;
    // The peer's declared wire domain / conventions.
    WireTimestampBinding peer_binding;
    // k = fA/fB, required for a cross-clock estimate.
    ClockRatio ratio;

    // The wire profile used to encode this endpoint's own timestamps.  Its
    // `timestamp_unit_hz` MUST equal `local_domain.tick_rate_hz`: a lossy
    // local->wire unit conversion is not silently performed in M1-B.
    FrameProfile frame_profile;

    // The calibration set in force for this endpoint's local instants.
    CalibrationStamp local_calibration;

    // Timing budget, in LOCAL ticks.  All three default to "not stated"
    // (0 == disabled) and are enforced only when non-zero.
    int64_t reply_deadline_ticks = 0;   // per-message reply budget
    int64_t exchange_timeout_ticks = 0; // whole-exchange deadline
    int64_t evidence_wait_ticks = 0;    // how long to wait for local TX evidence

    // Bounded completed-result storage.  When it is full a NEW Begin is
    // refused with QueueFull; an already-accepted exchange's terminal result
    // is never dropped (M1-B §4.3 / B13).
    uint32_t result_queue_capacity = 4;
    // Deeper than the queue so a refused Begin and counters can still be
    // reported.
    uint32_t max_in_flight = 1;
};

// ===========================================================================
// The endpoint core
// ===========================================================================

class EndpointCore
{
public:
    EndpointCore();
    ~EndpointCore();

    EndpointCore(const EndpointCore&) = delete;
    EndpointCore& operator=(const EndpointCore&) = delete;

    // Validate and freeze a configuration snapshot.  Returns false and leaves
    // `why` explaining the FIRST problem; a refused configure leaves the core
    // unconfigured and unable to advance.
    bool configure(const CoreConfig& cfg, std::string& why);

    // Start a new session generation on the same configuration.  REFUSES
    // while an exchange is in flight (post a `Reset` or `Stop` EVENT first so
    // its terminal result is not lost), requires the supplied configuration to
    // carry a peer binding for the new generation, does NOT clear the
    // counters, and RETAINS undrained terminal results.  Old events cannot
    // revive an exchange because their token/exchange identities are gone.
    bool reset(const CoreConfig& cfg, uint64_t new_generation, std::string& why);

    bool is_configured() const;
    const CoreConfig& config() const;

    Protocol protocol() const;
    Role role() const;
    EndpointState state() const;

    // Post one event.  Domain-checked first: an out-of-domain kind is counted
    // and produces NO action and NO terminal result.  Never throws.
    CoreActionBatch post(const CoreEvent& ev);

    // Drain one completed terminal result.  Returns false when the queue is
    // empty.  Terminal results are ALSO delivered as TerminalResult actions,
    // so a driver that drains actions never needs this; a driver that does not
    // can hold them here up to `result_queue_capacity`.
    bool pop_result(ProtocolTofEstimate& out);

    const CoreCounters& counters() const;

    // The conservation identity the acceptance suite asserts:
    //     accepted_exchanges == terminal_results + in_flight
    uint64_t in_flight() const;

    // Human-readable state dump (cold path).
    std::string to_string() const;

private:
    struct Impl;
    Impl* d_impl;
};

} // namespace twr
} // namespace uwb
} // namespace gr

#endif /* INCLUDED_GNURADIO_UWB_UWB_TWR_CORE_H */
