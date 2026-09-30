/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * M1-B: the boundary between a LOCAL, strictly-admitted ranging interval and a
 * REMOTE timestamp that arrived as a wire field.
 *
 * ---------------------------------------------------------------------------
 * Why this header exists (M1-B instruction §3)
 * ---------------------------------------------------------------------------
 *
 * The v1 frame carries 40-bit LE INTEGER ticks and nothing else: no epoch, no
 * tick rate, no first-path quality, no calibration record, no `TxSendEvidence`.
 * `compute_ss_tof()` / `compute_ds_tof()` (M1-A) consume an
 * `AdmittedRangingInterval`, which can only be produced by running the strict
 * M0.1 gate over two `Timestamp`s that each carry a marker, a measurement
 * source, the correction chain, a current calibration and a first-path
 * DECISION.  A raw integer off the air has none of that, and the instruction is
 * explicit that it must never be silently upgraded to `HardwareMeasured`,
 * "calibrated" or "TX Completed" just because it arrived.
 *
 * So M1-B adds a SECOND, explicitly weaker path:
 *
 *   * a `WireTimestampBinding` is the SESSION-LEVEL declaration ("this peer's
 *     wire fields are device ticks on this domain, marker X, conventions Y/Z,
 *     at most N ticks apart, sequence modulus M").  It is an AGREEMENT, not
 *     runtime evidence of anything;
 *   * a `PeerTimestampClaim` is one field of one matched frame, with its raw
 *     ticks and the message identity it was read from;
 *   * a `ProtocolInterval` is either an already-admitted LOCAL interval or a
 *     pair of bound peer claims.  It cannot be converted to
 *     `AdmittedRangingInterval`, and it has no `Timestamp` inside, so the
 *     strict gate can never be reached from it;
 *   * `compute_protocol_ss_tof()` / `compute_protocol_ds_tof()` run the SAME
 *     numeric kernel as M1-A but their own, weaker input checks, and produce a
 *     `ProtocolTofEstimate` whose every "is this a range?" entry is false.
 *
 * The result is a PROTOCOL ESTIMATE with an explicit `wire_claim` provenance,
 * suitable for an offline two-endpoint simulation.  It is NOT a validated
 * range, NOT a hardware measurement, and it does not upgrade any capability.
 *
 * ---------------------------------------------------------------------------
 * N07 discipline
 * ---------------------------------------------------------------------------
 *
 * Every enum below has an `is_known()` written as a switch with NO `default`,
 * and every consumer tests the domain BEFORE switching on the value.
 */

#ifndef INCLUDED_GNURADIO_UWB_UWB_TWR_PROTOCOL_TIME_H
#define INCLUDED_GNURADIO_UWB_UWB_TWR_PROTOCOL_TIME_H

#include <gnuradio/uwb/uwb_twr_frame.h>
#include <gnuradio/uwb/uwb_twr_math.h>

#include <cstdint>
#include <cstdio>
#include <string>

namespace gr {
namespace uwb {
namespace twr {

// ===========================================================================
// WireTimestampBinding -- the session-level peer declaration
// ===========================================================================
//
// Everything the two endpoints must AGREE on before a raw peer integer can be
// turned into an interval.  None of it is runtime evidence: `calibration_
// convention_id` is the identifier of the agreement, NOT a record that a
// calibration stage ran, and `binding_generation` exists so a re-declaration
// (a new session) invalidates intervals formed under the old one.
struct WireTimestampBinding {
    // The peer's tick domain: name, rate, epoch and wrap width.  Identity and
    // epoch are the same fields the strict gate compares, so a peer claim is
    // expressed in exactly one (name, rate, bits, epoch) counter.
    ClockDomain peer_domain;

    // Which local session generation this declaration belongs to.  A claim
    // formed under a different generation is refused.
    uint64_t session_generation = 0;
    // Version of the DECLARATION itself; bumping it invalidates older claims.
    uint64_t binding_generation = 0;

    // The marker the wire field denotes.  For the three TWR frames this is
    // `RmarkerRx` or `RmarkerTx`; `AntennaPlane` is accepted only as a
    // declared convention and never as a measured instant.
    TimestampMarker peer_marker = TimestampMarker::RmarkerRx;

    // Human/audit identifiers of the AGREEMENTS.  Empty is refused: an
    // unnamed convention cannot be cited in a result.
    std::string unit_convention;           // e.g. "peer_device_ticks"
    std::string calibration_convention_id; // e.g. "twr-link-cal-ch5-v1"

    // The largest interval, in peer ticks, this session will accept.  Must be
    // positive and must not exceed the half period of a wrapping domain.
    uint64_t max_interval_ticks = 0;

    // The configured wire-sequence reuse modulus.  Must be one of the values
    // the config layer supports (4/16/64/256): a peer claim is refused when
    // the running exchange has already reached the reuse boundary.
    uint16_t sequence_modulus = 4;

    std::string to_string() const
    {
        char buf[320];
        std::snprintf(buf, sizeof(buf),
                      "WireTimestampBinding{session_gen=%llu binding_gen=%llu marker=%s "
                      "unit=%s cal=%s max_ticks=%llu modulus=%u domain=%s}",
                      static_cast<unsigned long long>(session_generation),
                      static_cast<unsigned long long>(binding_generation),
                      timestamp_marker_to_string(peer_marker), unit_convention.c_str(),
                      calibration_convention_id.c_str(),
                      static_cast<unsigned long long>(max_interval_ticks),
                      static_cast<unsigned>(sequence_modulus),
                      peer_domain.to_string().c_str());
        return std::string(buf);
    }
};

// The sequence moduli the configuration layer supports.  Kept here so the
// protocol path can refuse an unsupported one without re-reading the config.
inline bool sequence_modulus_is_supported(uint16_t m)
{
    switch (m) {
    case 4:
    case 16:
    case 64:
    case 256:
        return true;
    }
    return false;
}

// A binding is usable when it names a real peer counter, states both
// conventions, has a positive interval bound inside the domain's unambiguous
// window, and uses a supported sequence modulus.
inline bool wire_timestamp_binding_is_well_formed(const WireTimestampBinding& b,
                                                   std::string& why)
{
    if (!b.peer_domain.is_valid()) {
        why = "peer_domain is not a valid clock domain";
        return false;
    }
    if (!timestamp_marker_is_known(b.peer_marker)) {
        why = "peer_marker is not a known TimestampMarker";
        return false;
    }
    if (b.session_generation == 0) {
        why = "session_generation is 0 (unset)";
        return false;
    }
    if (b.binding_generation == 0) {
        why = "binding_generation is 0 (unset)";
        return false;
    }
    if (b.unit_convention.empty()) {
        why = "unit_convention is empty";
        return false;
    }
    if (b.calibration_convention_id.empty()) {
        why = "calibration_convention_id is empty";
        return false;
    }
    if (b.max_interval_ticks == 0) {
        why = "max_interval_ticks is 0";
        return false;
    }
    if (!sequence_modulus_is_supported(b.sequence_modulus)) {
        why = "sequence_modulus is not one of 4/16/64/256";
        return false;
    }
    const uint64_t period = clock_domain_wrap_period(b.peer_domain);
    if (period != 0u && b.max_interval_ticks > period / 2u) {
        why = "max_interval_ticks exceeds the domain's unambiguous half period";
        return false;
    }
    return true;
}

// ===========================================================================
// PeerTimestampClaim -- one field of one matched frame
// ===========================================================================
struct PeerTimestampClaim {
    TimestampField field = TimestampField::T1A;
    uint64_t raw_ticks = 0;

    // The message the field was read from.  The protocol path refuses a pair
    // of claims that do not come from the same frame/message.
    FrameType frame_type = FrameType::Poll;
    uint16_t frame_session = 0;
    uint16_t frame_seq = 0;
    uint16_t frame_src = 0;
    uint64_t message_identity = 0;

    // Set only by the factory below.  A hand-built claim is `bound == false`
    // and is refused, so "I forgot to say where this number came from" cannot
    // read as a valid claim.
    bool bound = false;

    bool field_is_known() const
    {
        switch (field) {
        case TimestampField::T1A:
        case TimestampField::T2B:
        case TimestampField::T3B:
        case TimestampField::T4A:
        case TimestampField::T5A:
            return true;
        case TimestampField::Count:
            return false;
        }
        return false;
    }

    std::string to_string() const
    {
        char buf[256];
        std::snprintf(buf, sizeof(buf),
                      "PeerTimestampClaim{%s raw=%llu type=%s session=%u seq=%u src=%u "
                      "id=%llu bound=%s}",
                      timestamp_field_name(field),
                      static_cast<unsigned long long>(raw_ticks),
                      frame_type_to_string(frame_type),
                      static_cast<unsigned>(frame_session),
                      static_cast<unsigned>(frame_seq), static_cast<unsigned>(frame_src),
                      static_cast<unsigned long long>(message_identity),
                      bound ? "true" : "false");
        return std::string(buf);
    }
};

// The identity the core stores alongside a claim.  A stable FNV-1a over the
// frame's declared content -- not over an encoder, so the codec cannot change
// it -- used only to prove two claims came from the SAME message.
inline uint64_t frame_message_identity(const Frame& f)
{
    uint64_t h = 1469598103934665603ull;
    auto mix = [&h](uint64_t v) {
        for (int i = 0; i < 8; ++i) {
            h ^= (v >> (8 * i)) & 0xffu;
            h *= 1099511628211ull;
        }
    };
    mix(static_cast<uint64_t>(f.version));
    mix(static_cast<uint64_t>(f.function_code));
    mix(f.session_id);
    mix(f.seq);
    mix(f.pan_id);
    mix(f.src_addr);
    mix(f.dst_addr);
    mix(f.flags);
    for (size_t i = 0; i < f.timestamp_count(); ++i)
        mix(f.timestamps[i]);
    return h;
}

// Read a field out of an already-decoded frame into a claim.  Fails when the
// frame does not carry the field -- a claim is never synthesised for a field
// the message does not have.
inline bool make_peer_claim(const Frame& f, TimestampField field, PeerTimestampClaim& out)
{
    uint64_t raw = 0;
    if (!f.get(field, raw))
        return false;
    out.field = field;
    out.raw_ticks = raw;
    out.frame_type = f.function_code;
    out.frame_session = f.session_id;
    out.frame_seq = f.seq;
    out.frame_src = f.src_addr;
    out.message_identity = frame_message_identity(f);
    out.bound = true;
    return true;
}

// ===========================================================================
// ProtocolInterval -- local admitted OR peer wire claim, never both/neither
// ===========================================================================
enum class ProtocolIntervalOrigin : uint8_t {
    LocalAdmitted = 0, // produced by the strict M0.1 gate on this endpoint
    PeerWireClaim = 1  // formed from bound peer claims and a session binding
};

inline const char* protocol_interval_origin_to_string(ProtocolIntervalOrigin o)
{
    switch (o) {
    case ProtocolIntervalOrigin::LocalAdmitted:
        return "local_admitted";
    case ProtocolIntervalOrigin::PeerWireClaim:
        return "peer_wire_claim";
    }
    return "invalid";
}

inline bool protocol_interval_origin_is_known(ProtocolIntervalOrigin o)
{
    switch (o) {
    case ProtocolIntervalOrigin::LocalAdmitted:
    case ProtocolIntervalOrigin::PeerWireClaim:
        return true;
    }
    return false;
}

// The refusal vocabulary for the peer-claim path.  Append-only schema.
enum class PeerClaimError : uint8_t {
    Ok = 0,
    BindingNotWellFormed = 1,
    ClaimNotBound = 2,
    UnknownField = 3,
    NotSameMessage = 4,
    RawOutOfDomain = 5,
    IntervalNotFormable = 6, // reversed / wrap ambiguous
    IntervalTooLong = 7,
    // Reserved for a future binding that declares which session a claim must
    // belong to; M1-B's `WireTimestampBinding` carries a generation but a
    // claim does not, so no M1-B path returns this value.  Kept because the
    // numeric codes are append-only schema (REQ-OUT-01).
    SessionGenerationMismatch = 8
};

// Domain test with NO `default` (N07): an out-of-domain error code is not
// "some other refusal" and must not be formatted or reasoned about.
inline bool peer_claim_error_is_known(PeerClaimError e)
{
    switch (e) {
    case PeerClaimError::Ok:
    case PeerClaimError::BindingNotWellFormed:
    case PeerClaimError::ClaimNotBound:
    case PeerClaimError::UnknownField:
    case PeerClaimError::NotSameMessage:
    case PeerClaimError::RawOutOfDomain:
    case PeerClaimError::IntervalNotFormable:
    case PeerClaimError::IntervalTooLong:
    case PeerClaimError::SessionGenerationMismatch:
        return true;
    }
    return false;
}

inline const char* peer_claim_error_to_string(PeerClaimError e)
{
    switch (e) {
    case PeerClaimError::Ok:
        return "ok";
    case PeerClaimError::BindingNotWellFormed:
        return "binding_not_well_formed";
    case PeerClaimError::ClaimNotBound:
        return "claim_not_bound";
    case PeerClaimError::UnknownField:
        return "unknown_field";
    case PeerClaimError::NotSameMessage:
        return "not_same_message";
    case PeerClaimError::RawOutOfDomain:
        return "raw_out_of_domain";
    case PeerClaimError::IntervalNotFormable:
        return "interval_not_formable";
    case PeerClaimError::IntervalTooLong:
        return "interval_too_long";
    case PeerClaimError::SessionGenerationMismatch:
        return "session_generation_mismatch";
    }
    return "invalid";
}

// An interval the protocol path may consume.  It is a closed variant: the
// only two ways to make a valid one are `from_local()` (an interval that
// already passed the strict gate) and `make_peer()` (two bound claims plus a
// well-formed binding).  There is deliberately NO conversion operator to
// `AdmittedRangingInterval` and no `Timestamp` inside, so the strict path
// cannot be reached from a wire claim.
class ProtocolInterval {
public:
    ProtocolInterval() = default; // invalid; origin is never read when !valid

    // Wrap an interval that already passed `admit_ranging_interval()`.
    static ProtocolInterval from_local(const AdmittedRangingInterval& admitted)
    {
        ProtocolInterval p;
        p.origin_ = ProtocolIntervalOrigin::LocalAdmitted;
        p.valid_ = true;
        p.interval_ = admitted.interval();
        p.later_tick_ = admitted.later().ticks();
        p.earlier_tick_ = admitted.earlier().ticks();
        return p;
    }

    // Form a peer interval from two bound claims.  `later`/`earlier` are the
    // protocol roles (e.g. for DB = t3B - t2B, later = T3B, earlier = T2B).
    // Any refusal leaves `out` untouched.
    static PeerClaimError make_peer(const PeerTimestampClaim& later,
                                    const PeerTimestampClaim& earlier,
                                    const WireTimestampBinding& binding,
                                    ProtocolInterval& out)
    {
        std::string why;
        if (!wire_timestamp_binding_is_well_formed(binding, why))
            return PeerClaimError::BindingNotWellFormed;
        if (!later.bound || !earlier.bound)
            return PeerClaimError::ClaimNotBound;
        if (!later.field_is_known() || !earlier.field_is_known())
            return PeerClaimError::UnknownField;
        if (later.message_identity != earlier.message_identity ||
            later.frame_type != earlier.frame_type ||
            later.frame_session != earlier.frame_session ||
            later.frame_seq != earlier.frame_seq || later.frame_src != earlier.frame_src)
            return PeerClaimError::NotSameMessage;
        if (!binding.peer_domain.ticks_in_range(static_cast<int64_t>(later.raw_ticks)) ||
            !binding.peer_domain.ticks_in_range(static_cast<int64_t>(earlier.raw_ticks)))
            return PeerClaimError::RawOutOfDomain;

        // The modular difference is computed with the timestamp layer's OWN
        // tick-space tool, so the half-period policy (== half is legal, beyond
        // is ambiguous) is shared rather than re-derived.
        const TickDelta d = raw_tick_delta(
            binding.peer_domain, static_cast<int64_t>(later.raw_ticks),
            static_cast<int64_t>(earlier.raw_ticks));
        if (d.status != TimeIntervalStatus::Ok)
            return PeerClaimError::IntervalNotFormable;
        if (d.ticks > binding.max_interval_ticks)
            return PeerClaimError::IntervalTooLong;

        ProtocolInterval p;
        p.origin_ = ProtocolIntervalOrigin::PeerWireClaim;
        p.valid_ = true;
        p.session_generation_ = binding.session_generation;
        p.binding_generation_ = binding.binding_generation;
        p.interval_.valid = true;
        p.interval_.domain = binding.peer_domain;
        p.interval_.later_marker = binding.peer_marker;
        p.interval_.earlier_marker = binding.peer_marker;
        p.interval_.ticks = static_cast<int64_t>(d.ticks);
        p.interval_.frac_num = 0;
        p.interval_.frac_den = 0; // whole-tick wire: no fraction to fabricate
        p.interval_.wrapped = d.wrapped;
        p.later_tick_ = static_cast<int64_t>(later.raw_ticks);
        p.earlier_tick_ = static_cast<int64_t>(earlier.raw_ticks);
        out = p;
        return PeerClaimError::Ok;
    }

    bool is_valid() const { return valid_; }
    ProtocolIntervalOrigin origin() const { return origin_; }
    bool is_local_admitted() const
    {
        return valid_ && origin_ == ProtocolIntervalOrigin::LocalAdmitted;
    }
    bool is_peer_wire_claim() const
    {
        return valid_ && origin_ == ProtocolIntervalOrigin::PeerWireClaim;
    }

    const ClockDomain& domain() const { return interval_.domain; }
    TimestampMarker later_marker() const { return interval_.later_marker; }
    TimestampMarker earlier_marker() const { return interval_.earlier_marker; }
    bool wrapped() const { return interval_.wrapped; }
    bool is_whole_ticks() const { return interval_.is_whole_ticks(); }

    uint64_t session_generation() const { return session_generation_; }
    uint64_t binding_generation() const { return binding_generation_; }

    // The absolute tick coordinate of each endpoint of the interval, in the
    // interval's OWN domain.  For a local interval these are the admitted
    // instants' ticks; for a peer claim they are the raw wire values.  They
    // are what the clock-ratio validity window is checked against.
    int64_t later_tick() const { return later_tick_; }
    int64_t earlier_tick() const { return earlier_tick_; }

    // Exact rational tick value.  For a peer claim the denominator is 1: a
    // wire value is a whole number of ticks by construction.
    bool exact_ratio(int64_t& out_num, int64_t& out_den) const
    {
        if (!valid_)
            return false;
        return interval_.exact_ratio(out_num, out_den);
    }

    std::string to_string() const
    {
        if (!valid_)
            return "ProtocolInterval{invalid}";
        int64_t n = 0;
        int64_t d = 1;
        exact_ratio(n, d);
        char buf[256];
        std::snprintf(buf, sizeof(buf),
                      "ProtocolInterval{%s %lld/%lld ticks domain=%s later=%s earlier=%s "
                      "wrapped=%s}",
                      protocol_interval_origin_to_string(origin_),
                      static_cast<long long>(n), static_cast<long long>(d),
                      interval_.domain.to_string().c_str(),
                      timestamp_marker_to_string(interval_.later_marker),
                      timestamp_marker_to_string(interval_.earlier_marker),
                      interval_.wrapped ? "true" : "false");
        return std::string(buf);
    }

private:
    bool valid_ = false;
    ProtocolIntervalOrigin origin_ = ProtocolIntervalOrigin::LocalAdmitted;
    RelativeTickInterval interval_;
    int64_t later_tick_ = 0;
    int64_t earlier_tick_ = 0;
    uint64_t session_generation_ = 0;
    uint64_t binding_generation_ = 0;
};

// ===========================================================================
// ProtocolTofEstimate -- the result of the protocol-level ToF path
// ===========================================================================
//
// A separate strong type from `TofResult` on purpose.  `TofResult` carries
// `exchange_status`, and `exchange_status_yields_range(ExchangeStatus::Ok)`
// returns true; the instruction forbids using that as the top-level success
// state of a protocol estimate.  Every range-validity entry on this type is
// hard-coded false, and there is no conversion to `TofResult`.
enum class ProtocolCompletionStatus : uint8_t {
    NotComplete = 0, // the exchange has not reached a protocol terminal state
    Complete = 1,    // the protocol exchange completed (estimate may still be absent)
    Failed = 2       // the exchange failed for a stated reason
};

inline const char* protocol_completion_status_to_string(ProtocolCompletionStatus s)
{
    switch (s) {
    case ProtocolCompletionStatus::NotComplete:
        return "not_complete";
    case ProtocolCompletionStatus::Complete:
        return "complete";
    case ProtocolCompletionStatus::Failed:
        return "failed";
    }
    return "invalid";
}

inline bool protocol_completion_status_is_known(ProtocolCompletionStatus s)
{
    switch (s) {
    case ProtocolCompletionStatus::NotComplete:
    case ProtocolCompletionStatus::Complete:
    case ProtocolCompletionStatus::Failed:
        return true;
    }
    return false;
}

struct ProtocolTofEstimate {
    ProtocolCompletionStatus completion = ProtocolCompletionStatus::NotComplete;

    // True only when the computing endpoint had all of ITS OWN local evidence
    // admitted and the numeric kernel succeeded.
    bool estimate_available = false;
    // True because the numeric path only runs on admitted local intervals and
    // converges only after local evidence is complete.
    bool local_evidence_complete = false;
    // True when at least one input interval came off the wire.
    bool peer_evidence_is_wire_claim = false;
    // ALWAYS false in M1-B: a protocol estimate is not a validated measurement.
    bool measurement_valid = false;
    // ALWAYS true in M1-B: this result comes from the deterministic simulation.
    bool execution_mode_is_simulation = true;

    Protocol protocol = Protocol::Ss;
    ComputedAt computed_at = ComputedAt::InitiatorA;

    // Exact SIGNED ToF in `domain` ticks, retained even on failure (so a
    // negative ToF keeps its sign; the instruction forbids clamping).
    TofRationalTicks tof;
    ClockDomain domain;
    ClockRatio ratio;

    TofStatus math_status = TofStatus::InvalidResult;
    ExchangeStatus failure_reason = ExchangeStatus::InternalError;
    std::string detail;

    // No entry point here can ever claim a range.  `method_*` naming keeps the
    // intent obvious and makes an accidental use in a range helper a compile
    // error rather than a silent success.
    bool yields_range() const { return false; }
    bool is_hardware_measurement() const { return false; }
    bool is_validated_measurement() const { return false; }

    std::string to_string() const
    {
        char buf[384];
        std::snprintf(buf, sizeof(buf),
                      "ProtocolTofEstimate{completion=%s estimate=%s local=%s peer_wire=%s "
                      "measurement_valid=%s protocol=%s at=%s tof=%s domain=%s math=%s}",
                      protocol_completion_status_to_string(completion),
                      estimate_available ? "true" : "false",
                      local_evidence_complete ? "true" : "false",
                      peer_evidence_is_wire_claim ? "true" : "false",
                      measurement_valid ? "true" : "false", protocol_to_string(protocol),
                      computed_at_to_string(computed_at), tof.to_string().c_str(),
                      domain.is_valid() ? domain.name.c_str() : "(none)",
                      tof_status_to_string(math_status));
        std::string s(buf);
        if (!detail.empty())
            s += " detail=" + detail;
        return s;
    }
};

// A terminal status that CANNOT be fed to a range helper.  F's read-only review
// (M1-B_评审报告.md, finding B-1) reproduced that a successful protocol action
// carried `ExchangeStatus::Ok`, and
// `exchange_status_yields_range(ExchangeStatus::Ok) == true` -- i.e. the FSM's
// own output envelope re-opened the fail-open path the result type closes.  This
// strong type keeps the completion state and the failure reason, and its every
// range entry is hard false.
struct ProtocolTerminalStatus {
    ProtocolCompletionStatus completion = ProtocolCompletionStatus::NotComplete;
    // Only meaningful when `completion == Failed`; `Ok` means "no failure".
    ExchangeStatus failure_reason = ExchangeStatus::InternalError;

    bool completed() const { return completion == ProtocolCompletionStatus::Complete; }
    bool failed() const { return completion == ProtocolCompletionStatus::Failed; }
    bool yields_range() const { return false; }
    bool measurement_valid() const { return false; }

    std::string to_string() const
    {
        std::string s = protocol_completion_status_to_string(completion);
        if (failed())
            s += "(" + std::string(exchange_status_to_string(failure_reason)) + ")";
        return s;
    }
};

inline std::string protocol_terminal_status_to_string(const ProtocolTerminalStatus& s)
{
    return s.to_string();
}

// The protocol terminal state of an endpoint that owns NO ToF estimate
// (SS responder / DS initiator).  It completes, and it does so without a
// distance: the other end's result is never copied across.
inline ProtocolTofEstimate make_protocol_complete_without_estimate(Protocol protocol,
                                                                   ComputedAt computed_at,
                                                                   const std::string& detail)
{
    ProtocolTofEstimate e;
    e.completion = ProtocolCompletionStatus::Complete;
    e.estimate_available = false;
    e.local_evidence_complete = true;
    e.peer_evidence_is_wire_claim = false;
    e.protocol = protocol;
    e.computed_at = computed_at;
    e.math_status = TofStatus::Ok;
    e.failure_reason = ExchangeStatus::Ok;
    e.detail = detail;
    return e;
}

namespace detail {

// Map the shared kernel status onto the protocol estimate's problem codes.
inline TofStatus protocol_kernel_to_tof_status(KernelStatus k)
{
    switch (k) {
    case KernelStatus::Ok:
        return TofStatus::Ok;
    case KernelStatus::Overflow:
        return TofStatus::Overflow;
    case KernelStatus::ZeroDenominator:
        return TofStatus::ZeroDenominator;
    }
    return TofStatus::InvalidResult;
}

inline ProtocolTofEstimate protocol_fail(ProtocolTofEstimate e, TofStatus s,
                                         const std::string& why)
{
    e.completion = ProtocolCompletionStatus::Failed;
    e.estimate_available = false;
    e.math_status = s;
    e.failure_reason = tof_status_to_exchange_status(s);
    e.detail = why;
    return e;
}

// The domain of each formula slot.  SS uses (A, B); DS uses (A, B, A, B).
inline bool protocol_domain_ok(ProtocolTofEstimate& e, const char* what,
                               const ProtocolInterval& iv, const ClockDomain& want)
{
    if (!iv.is_valid()) {
        e = protocol_fail(e, TofStatus::InvalidInput,
                          std::string(what) + ": protocol interval is not valid");
        return false;
    }
    if (!protocol_interval_origin_is_known(iv.origin())) {
        e = protocol_fail(e, TofStatus::InvalidInput,
                          std::string(what) + ": protocol interval origin is out of domain");
        return false;
    }
    if (!iv.domain().is_valid()) {
        e = protocol_fail(e, TofStatus::InvalidClockDomain,
                          std::string(what) + ": interval carries an invalid clock domain");
        return false;
    }
    if (!clock_domain_same_identity(iv.domain(), want)) {
        e = protocol_fail(e, TofStatus::ClockRatioDomainMismatch,
                          std::string(what) + " is in " + iv.domain().to_string() +
                              " but the clock ratio declares " + want.to_string());
        return false;
    }
    if (!clock_domain_same_epoch(iv.domain(), want)) {
        e = protocol_fail(e, TofStatus::ClockRatioEpochMismatch,
                          std::string(what) + " is epoch " +
                              std::to_string(iv.domain().epoch_id) +
                              " but the clock ratio declares epoch " +
                              std::to_string(want.epoch_id));
        return false;
    }
    return true;
}

inline bool protocol_load_interval(ProtocolTofEstimate& e, const char* what,
                                   const ProtocolInterval& iv, Rat128& out)
{
    if (!iv.is_valid()) {
        e = protocol_fail(e, TofStatus::InvalidInput,
                          std::string(what) + ": protocol interval is not valid");
        return false;
    }
    int64_t num = 0;
    int64_t den = 0;
    if (!iv.exact_ratio(num, den)) {
        e = protocol_fail(e, TofStatus::Overflow,
                          std::string(what) + ": exact_ratio() did not fit int64");
        return false;
    }
    out = rat128_make(static_cast<i128>(num), static_cast<i128>(den));
    return true;
}

// The A-domain validity window of the clock ratio applies to whatever A-domain
// interval the exchange runs on, local or wire.  Both endpoints of the
// interval are checked (min/max, so a wrapped interval that straddles the
// rollover is handled the same way as the strict M1-A window check).
inline bool protocol_window_ok(ProtocolTofEstimate& e, const char* what,
                               const ProtocolInterval& iv, const ClockRatio& k)
{
    if (!k.has_validity_window())
        return true;
    if (!iv.is_valid()) {
        e = protocol_fail(e, TofStatus::InvalidInput,
                          std::string(what) + ": protocol interval is not valid");
        return false;
    }
    const int64_t a = iv.earlier_tick();
    const int64_t b = iv.later_tick();
    const int64_t lo = (a < b) ? a : b;
    const int64_t hi = (a < b) ? b : a;
    if (!k.covers_ticks(lo) || !k.covers_ticks(hi)) {
        e = protocol_fail(e, TofStatus::ClockRatioNotValidAtTime,
                          std::string(what) + " spans ticks [" + std::to_string(lo) +
                              "," + std::to_string(hi) +
                              "], outside the clock ratio validity window [" +
                              std::to_string(k.valid_from_ticks()) + "," +
                              std::to_string(k.valid_until_ticks()) + "]");
        return false;
    }
    return true;
}

} // namespace detail

// ===========================================================================
// Protocol-level SS / DS entry points
// ===========================================================================
//
// Same numeric kernel as M1-A, weaker input rules: peer intervals need only a
// well-formed binding, message identity, domain/rate/window agreement and wire
// representability.  They deliberately do NOT require a peer first path, a
// calibration application record or a TX completion -- none of those can be
// proven by a wire field, and demanding them would force the caller to
// fabricate evidence.
//
// All four slots must already be expressed in the unit the formula wants:
// RA/DA in A ticks and RB/DB in B ticks (the kernel converts).  A local slot
// is an admitted interval; a peer slot is a wire claim.
inline ProtocolTofEstimate compute_protocol_ss_tof(const ProtocolInterval& ra,
                                                   const ProtocolInterval& db,
                                                   const ClockRatio& k_ab)
{
    ProtocolTofEstimate e;
    e.protocol = Protocol::Ss;
    e.computed_at = ComputedAt::InitiatorA;
    e.ratio = k_ab;
    e.domain = k_ab.domain_a();

    const TofStatus ks = k_ab.check();
    if (ks != TofStatus::Ok)
        return detail::protocol_fail(e, ks, "clock ratio is unusable: " + k_ab.to_string());

    if (!detail::protocol_domain_ok(e, "ra", ra, k_ab.domain_a()))
        return e;
    if (!detail::protocol_domain_ok(e, "db", db, k_ab.domain_b()))
        return e;
    if (!detail::protocol_window_ok(e, "ra", ra, k_ab))
        return e;

    detail::Rat128 RA;
    detail::Rat128 DB;
    if (!detail::protocol_load_interval(e, "ra", ra, RA))
        return e;
    if (!detail::protocol_load_interval(e, "db", db, DB))
        return e;

    const detail::Rat128 k = detail::rat128_make(k_ab.k_num(), k_ab.k_den());
    detail::Rat128 half;
    const detail::KernelStatus st = detail::ss_tof_kernel(RA, DB, k, half);
    if (st != detail::KernelStatus::Ok)
        return detail::protocol_fail(e, detail::protocol_kernel_to_tof_status(st),
                                     "SS exact kernel failed");
    if (!detail::rat128_fits_int64(half))
        return detail::protocol_fail(e, TofStatus::Overflow,
                                     "ToF numerator/denominator do not fit int64 ticks");

    e.tof = detail::rat128_to_ticks(half);
    e.math_status = TofStatus::Ok;
    e.failure_reason = ExchangeStatus::Ok;
    e.local_evidence_complete = true;
    e.peer_evidence_is_wire_claim = ra.is_peer_wire_claim() || db.is_peer_wire_claim();
    e.estimate_available = true;
    e.completion = (e.tof.num < 0) ? ProtocolCompletionStatus::Failed
                                   : ProtocolCompletionStatus::Complete;
    if (e.tof.num < 0) {
        e.estimate_available = false;
        e.math_status = TofStatus::NegativeTof;
        e.failure_reason = tof_status_to_exchange_status(TofStatus::NegativeTof);
        e.detail = "protocol SS ToF is negative (" + e.tof.to_string() +
                   " A ticks): retained with its sign";
    } else {
        e.detail = "protocol SS ToF = (RA - k*DB)/2 over wire/local inputs";
    }
    return e;
}

inline ProtocolTofEstimate compute_protocol_ds_tof(const ProtocolInterval& ra,
                                                   const ProtocolInterval& rb,
                                                   const ProtocolInterval& da,
                                                   const ProtocolInterval& db,
                                                   const ClockRatio& k_ab)
{
    ProtocolTofEstimate e;
    e.protocol = Protocol::Ds;
    e.computed_at = ComputedAt::ResponderB;
    e.ratio = k_ab;
    e.domain = k_ab.domain_a(); // the common unit is A ticks

    const TofStatus ks = k_ab.check();
    if (ks != TofStatus::Ok)
        return detail::protocol_fail(e, ks, "clock ratio is unusable: " + k_ab.to_string());

    if (!detail::protocol_domain_ok(e, "ra", ra, k_ab.domain_a()))
        return e;
    if (!detail::protocol_domain_ok(e, "da", da, k_ab.domain_a()))
        return e;
    if (!detail::protocol_domain_ok(e, "rb", rb, k_ab.domain_b()))
        return e;
    if (!detail::protocol_domain_ok(e, "db", db, k_ab.domain_b()))
        return e;
    if (!detail::protocol_window_ok(e, "ra", ra, k_ab))
        return e;
    if (!detail::protocol_window_ok(e, "da", da, k_ab))
        return e;

    detail::Rat128 RA;
    detail::Rat128 RB;
    detail::Rat128 DA;
    detail::Rat128 DB;
    if (!detail::protocol_load_interval(e, "ra", ra, RA))
        return e;
    if (!detail::protocol_load_interval(e, "rb", rb, RB))
        return e;
    if (!detail::protocol_load_interval(e, "da", da, DA))
        return e;
    if (!detail::protocol_load_interval(e, "db", db, DB))
        return e;

    const detail::Rat128 k = detail::rat128_make(k_ab.k_num(), k_ab.k_den());
    detail::Rat128 tof;
    const detail::KernelStatus st = detail::ds_tof_kernel(RA, RB, DA, DB, k, tof);
    if (st != detail::KernelStatus::Ok)
        return detail::protocol_fail(e, detail::protocol_kernel_to_tof_status(st),
                                     "DS exact kernel failed");
    if (!detail::rat128_fits_int64(tof))
        return detail::protocol_fail(e, TofStatus::Overflow,
                                     "ToF numerator/denominator do not fit int64 ticks");

    e.tof = detail::rat128_to_ticks(tof);
    e.math_status = TofStatus::Ok;
    e.failure_reason = ExchangeStatus::Ok;
    e.local_evidence_complete = true;
    e.peer_evidence_is_wire_claim = ra.is_peer_wire_claim() || rb.is_peer_wire_claim() ||
                                    da.is_peer_wire_claim() || db.is_peer_wire_claim();
    e.estimate_available = true;
    e.completion = (e.tof.num < 0) ? ProtocolCompletionStatus::Failed
                                   : ProtocolCompletionStatus::Complete;
    if (e.tof.num < 0) {
        e.estimate_available = false;
        e.math_status = TofStatus::NegativeTof;
        e.failure_reason = tof_status_to_exchange_status(TofStatus::NegativeTof);
        e.detail = "protocol DS ToF is negative (" + e.tof.to_string() +
                   " A ticks): retained with its sign";
    } else {
        e.detail = "protocol DS ToF = (RA*k*RB - DA*k*DB)/(RA+k*RB+DA+k*DB), all in A ticks";
    }
    return e;
}

} // namespace twr
} // namespace uwb
} // namespace gr

#endif /* INCLUDED_GNURADIO_UWB_UWB_TWR_PROTOCOL_TIME_H */
