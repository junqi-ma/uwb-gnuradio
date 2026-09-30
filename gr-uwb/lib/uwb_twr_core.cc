/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * M1-B: the SS-TWR / DS-TWR per-endpoint protocol core (implementation).
 *
 * Pure C++: no GNU Radio, no UHD, no PMT, no system clock.  Time advances only
 * because an input event says so.  See uwb_twr_core.h for the contract.
 */

#include <gnuradio/uwb/uwb_twr_core.h>

#include <array>
#include <cstdio>
#include <cstring>
#include <string>

namespace gr {
namespace uwb {
namespace twr {

namespace {

// Copy a local tick value into a wire timestamp field.  Refuses anything the
// wire cannot carry: a sub-tick fraction, a negative value, a value outside
// the profile field width, or a profile whose timestamp unit is not this
// endpoint's own domain rate (a lossy local->wire conversion is not silently
// performed in M1-B).
bool local_ts_to_wire(const Timestamp& ts, const FrameProfile& p, uint64_t& out,
                      std::string& why)
{
    if (!ts.is_self_consistent()) {
        why = "local timestamp is not self-consistent";
        return false;
    }
    if (ts.frac_den != 0u || ts.frac_num != 0) {
        why = "local timestamp carries a sub-tick fraction; the v1 wire holds "
              "whole ticks only";
        return false;
    }
    if (ts.ticks < 0) {
        why = "local timestamp is negative";
        return false;
    }
    if (!(p.timestamp_unit_hz == ts.domain.tick_rate_hz)) {
        why = "frame profile timestamp unit does not equal the local domain rate";
        return false;
    }
    if (!timestamp_fits(p, static_cast<uint64_t>(ts.ticks))) {
        why = "local timestamp does not fit the wire field width";
        return false;
    }
    out = static_cast<uint64_t>(ts.ticks);
    return true;
}

// The single configuration validator used by both `configure()` and
// `reset()`.  An out-of-domain enum is refused up front (N07).
bool validate_core_config(const CoreConfig& cfg, std::string& why)
{
    if (cfg.endpoint_id.empty()) {
        why = "endpoint_id is empty";
        return false;
    }
    if (!protocol_is_known(cfg.protocol)) {
        why = "protocol is out of domain";
        return false;
    }
    if (!role_is_known(cfg.role)) {
        why = "role is out of domain";
        return false;
    }
    if (!cfg.local_domain.is_valid()) {
        why = "local_domain is not a valid clock domain";
        return false;
    }
    if (!wire_timestamp_binding_is_well_formed(cfg.peer_binding, why))
        return false;
    if (cfg.peer_binding.session_generation != cfg.session_generation) {
        why = "peer_binding session_generation does not match the endpoint session";
        return false;
    }
    if (!sequence_modulus_is_supported(cfg.sequence_modulus)) {
        why = "sequence_modulus is not one of 4/16/64/256";
        return false;
    }
    if (cfg.initial_sequence >= cfg.sequence_modulus) {
        why = "initial_sequence is outside the sequence modulus";
        return false;
    }
    if (cfg.result_queue_capacity == 0) {
        why = "result_queue_capacity is 0";
        return false;
    }
    if (cfg.max_in_flight != 1) {
        why = "M1-B supports exactly one in-flight exchange per endpoint";
        return false;
    }
    if (!cfg.local_calibration.is_well_formed()) {
        why = "local_calibration is not well formed";
        return false;
    }
    if (!frame_profile_validate(cfg.frame_profile, why, nullptr))
        return false;
    if (!(cfg.frame_profile.timestamp_unit_hz == cfg.local_domain.tick_rate_hz)) {
        why = "frame profile timestamp unit must equal local_domain.tick_rate_hz "
              "(a lossy local->wire unit conversion is not performed in M1-B)";
        return false;
    }
    if (const TofStatus ks = cfg.ratio.check(); ks != TofStatus::Ok) {
        why = std::string("clock ratio is unusable: ") + tof_status_to_string(ks);
        return false;
    }
    // The formula's A/B domains: for BOTH protocols the initiator is "A" and
    // the responder is "B".  The local domain must match the side this
    // endpoint occupies, so a role swap re-binds the formula domain rather
    // than silently reusing the names.
    if (cfg.role == Role::Initiator) {
        if (!clock_domain_same_identity(cfg.local_domain, cfg.ratio.domain_a())) {
            why = "initiator local_domain does not match the clock ratio's A domain";
            return false;
        }
    } else {
        if (!clock_domain_same_identity(cfg.local_domain, cfg.ratio.domain_b())) {
            why = "responder local_domain does not match the clock ratio's B domain";
            return false;
        }
    }
    return true;
}

} // namespace

// ===========================================================================
// Counters
// ===========================================================================

std::string CoreCounters::to_json_string() const
{
    char buf[2048];
    std::snprintf(buf, sizeof(buf),
                  "{\"accepted_exchanges\":%llu,\"terminal_results\":%llu,"
                  "\"in_flight_peak\":%llu,\"requests_rejected_queue_full\":%llu,"
                  "\"frames_accepted\":%llu,\"frames_rejected_fcs\":%llu,"
                  "\"frames_rejected_decode\":%llu,\"frames_rejected_wrong_peer\":%llu,"
                  "\"frames_rejected_wrong_type\":%llu,\"frames_rejected_session\":%llu,"
                  "\"frames_rejected_seq\":%llu,\"frames_rejected_self\":%llu,"
                  "\"duplicate_frames\":%llu,\"noise_frames\":%llu,"
                  "\"tx_prepared\":%llu,\"tx_submitted\":%llu,"
                  "\"tx_submit_rejected_deadline\":%llu,\"tx_accepted\":%llu,"
                  "\"tx_outcome_completed\":%llu,\"tx_outcome_failed\":%llu,"
                  "\"tx_token_mismatch\":%llu,\"deadline_events\":%llu,"
                  "\"timeouts\":%llu,\"cancellations\":%llu,\"resets\":%llu,"
                  "\"overflow_events\":%llu,\"stale_events\":%llu,"
                  "\"events_rejected_out_of_domain\":%llu,\"contract_violations\":%llu,"
                  "\"results_dropped\":%llu}",
                  static_cast<unsigned long long>(accepted_exchanges),
                  static_cast<unsigned long long>(terminal_results),
                  static_cast<unsigned long long>(in_flight_peak),
                  static_cast<unsigned long long>(requests_rejected_queue_full),
                  static_cast<unsigned long long>(frames_accepted),
                  static_cast<unsigned long long>(frames_rejected_fcs),
                  static_cast<unsigned long long>(frames_rejected_decode),
                  static_cast<unsigned long long>(frames_rejected_wrong_peer),
                  static_cast<unsigned long long>(frames_rejected_wrong_type),
                  static_cast<unsigned long long>(frames_rejected_session),
                  static_cast<unsigned long long>(frames_rejected_seq),
                  static_cast<unsigned long long>(frames_rejected_self),
                  static_cast<unsigned long long>(duplicate_frames),
                  static_cast<unsigned long long>(noise_frames),
                  static_cast<unsigned long long>(tx_prepared),
                  static_cast<unsigned long long>(tx_submitted),
                  static_cast<unsigned long long>(tx_submit_rejected_deadline),
                  static_cast<unsigned long long>(tx_accepted),
                  static_cast<unsigned long long>(tx_outcome_completed),
                  static_cast<unsigned long long>(tx_outcome_failed),
                  static_cast<unsigned long long>(tx_token_mismatch),
                  static_cast<unsigned long long>(deadline_events),
                  static_cast<unsigned long long>(timeouts),
                  static_cast<unsigned long long>(cancellations),
                  static_cast<unsigned long long>(resets),
                  static_cast<unsigned long long>(overflow_events),
                  static_cast<unsigned long long>(stale_events),
                  static_cast<unsigned long long>(events_rejected_out_of_domain),
                  static_cast<unsigned long long>(contract_violations),
                  static_cast<unsigned long long>(results_dropped));
    return std::string(buf);
}

// ===========================================================================
// Implementation
// ===========================================================================

struct EndpointCore::Impl {
    bool configured = false;
    CoreConfig cfg;
    mutable CoreCounters counters;
    EndpointState state = EndpointState::Idle;

    bool in_flight = false;
    ExchangeId exchange;
    uint16_t current_seq = 0;
    uint64_t seq_issued = 0;
    // Bounded per-session-generation reuse barrier for the RESPONDER side.
    // A Poll whose sequence was already consumed in this generation is a
    // replay of a possibly-live identity and is refused rather than opening a
    // second exchange with the same seq (instruction §4.1; F's finding N-1).
    // `sequence_modulus` is at most 256, so this is a fixed-size array.
    std::array<bool, 256> seq_used{};

    // ---- causal TX records -------------------------------------------------
    struct TxRecord {
        bool present = false;
        TxToken token;
        FrameType intent = FrameType::Poll;
        Timestamp planned_tx_time;
        TxSendEvidence evidence;
        bool accepted = false;
        bool submitted = false;
        TxOutcome outcome = TxOutcome::Unknown;
        Frame frame;

        bool plan_ready() const { return present && evidence.records_the_plan(); }
        bool completed() const
        {
            return plan_ready() && tx_outcome_is_known(outcome) &&
                   outcome == TxOutcome::Completed;
        }
    };

    TxRecord tx_poll;    // initiator: Poll
    TxRecord tx_response;// responder: Response
    TxRecord tx_final;   // initiator (DS): Final

    bool pending_prepare = false;
    TxToken pending_token;
    FrameType pending_intent = FrameType::Poll;

    // ---- local RX instants + quality --------------------------------------
    bool have_rx_poll = false;
    Timestamp rx_poll_time;
    FirstPathQuality rx_poll_fp;

    bool have_rx_response = false;
    Timestamp rx_response_time;
    FirstPathQuality rx_response_fp;

    bool have_rx_final = false;
    Timestamp rx_final_time;
    FirstPathQuality rx_final_fp;

    // ---- peer wire claims --------------------------------------------------
    bool have_peer_response = false;
    PeerTimestampClaim c_t2B, c_t3B;

    bool have_peer_final = false;
    PeerTimestampClaim c_t1A, c_t4A, c_t5A;

    int64_t deadline_ticks = 0;
    // Set when the exchange has reached a point where a RESULT is owed but a
    // local transmit outcome has not converged.  The evidence deadline is a
    // stricter bound than the whole-exchange deadline (M1-B §4.2): without it
    // an unresolved TX would only be caught by the exchange timeout.
    bool awaiting_evidence = false;
    int64_t evidence_deadline_ticks = 0;

    std::vector<ProtocolTofEstimate> results;
    uint64_t token_counter = 0;

    // ---- helpers -----------------------------------------------------------

    void clear_exchange()
    {
        in_flight = false;
        exchange = ExchangeId{};
        state = EndpointState::Idle;
        tx_poll = TxRecord{};
        tx_response = TxRecord{};
        tx_final = TxRecord{};
        pending_prepare = false;
        pending_token = TxToken{};
        have_rx_poll = have_rx_response = have_rx_final = false;
        have_peer_response = have_peer_final = false;
        deadline_ticks = 0;
        awaiting_evidence = false;
        evidence_deadline_ticks = 0;
    }

    void terminal(CoreActionBatch& out, ProtocolTofEstimate est, ExchangeStatus st,
                  const std::string& detail, uint64_t event_id)
    {
        CoreAction a;
        a.kind = CoreActionKind::TerminalResult;
        a.event_id = event_id;
        a.exchange = exchange;
        // The action's status is a strong type: it carries the completion
        // state, and its range entries are false (F's finding B-1).
        a.status.completion = est.completion;
        if (est.completion == ProtocolCompletionStatus::Failed)
            a.status.set_failure_reason(st);
        a.detail = detail;
        a.result = est;
        out.push(a);

        if (results.size() < cfg.result_queue_capacity) {
            results.push_back(std::move(est));
        } else {
            counters.results_dropped++;
        }
        counters.terminal_results++;
        clear_exchange();
    }

    // Fail the in-flight exchange with a stated reason.
    void fail(CoreActionBatch& out, ExchangeStatus st, const std::string& detail,
              uint64_t event_id)
    {
        ProtocolTofEstimate est;
        est.completion = ProtocolCompletionStatus::Failed;
        est.estimate_available = false;
        est.local_evidence_complete = false;
        est.peer_evidence_is_wire_claim = true;
        est.measurement_valid = false;
        est.protocol = cfg.protocol;
        est.failure_reason = st;
        est.detail = detail;
        terminal(out, std::move(est), st, detail, event_id);
    }

    void emit_prepare(CoreActionBatch& out, const TxToken& token, FrameType intent,
                      uint64_t event_id)
    {
        CoreAction a;
        a.kind = CoreActionKind::PrepareTx;
        a.event_id = event_id;
        a.exchange = exchange;
        a.token = token;
        a.tx_intent = intent;
        out.push(a);
        counters.tx_prepared++;
    }

    void emit_abort(CoreActionBatch& out, const TxToken& token, uint64_t event_id)
    {
        CoreAction a;
        a.kind = CoreActionKind::AbortPending;
        a.event_id = event_id;
        a.exchange = exchange;
        a.token = token;
        out.push(a);
        pending_prepare = false;
    }

    void arm_rx(CoreActionBatch& out, FrameType expect, uint64_t event_id)
    {
        CoreAction a;
        a.kind = CoreActionKind::ArmRx;
        a.event_id = event_id;
        a.exchange = exchange;
        a.expect_type = expect;
        a.deadline_ticks = deadline_ticks;
        out.push(a);
    }

    TxToken next_token()
    {
        TxToken t;
        t.valid = true;
        t.value = ++token_counter;
        return t;
    }

    bool validate_frame(const Frame& f)
    {
        // Source address: frame_match() checks dst + self, not that src is the
        // CONFIGURED peer (M1-B §3.1).  Do it here, explicitly.  A frame whose
        // source is OUR OWN address is a loopback of our own transmission, not
        // a peer measurement, and is counted separately (REQ-PROTO-01/03).
        if (f.src_addr == cfg.local_address) {
            counters.frames_rejected_self++;
            return false;
        }
        if (f.src_addr != cfg.peer_address) {
            counters.frames_rejected_wrong_peer++;
            return false;
        }
        return true;
    }

    // Build a frame of `intent` with the current exchange's fixed fields.
    Frame base_frame(FrameType intent) const
    {
        Frame f;
        f.version = cfg.frame_profile.version;
        f.function_code = intent;
        f.session_id = cfg.session_id;
        f.seq = current_seq;
        f.pan_id = cfg.pan_id;
        f.src_addr = cfg.local_address;
        f.dst_addr = cfg.peer_address;
        f.flags = make_flags(true, false);
        return f;
    }

    bool build_frame(FrameType intent, const TxRecord& plan_rec, Frame& out,
                     std::string& why) const
    {
        Frame f = base_frame(intent);
        uint64_t v = 0;
        switch (intent) {
        case FrameType::Poll:
            break;
        case FrameType::Response:
            if (!local_ts_to_wire(rx_poll_time, cfg.frame_profile, v, why))
                return false;
            f.set(TimestampField::T2B, v);
            if (!local_ts_to_wire(plan_rec.planned_tx_time, cfg.frame_profile, v, why))
                return false;
            f.set(TimestampField::T3B, v);
            break;
        case FrameType::Final:
            if (!local_ts_to_wire(tx_poll.planned_tx_time, cfg.frame_profile, v, why))
                return false;
            f.set(TimestampField::T1A, v);
            if (!local_ts_to_wire(rx_response_time, cfg.frame_profile, v, why))
                return false;
            f.set(TimestampField::T4A, v);
            if (!local_ts_to_wire(plan_rec.planned_tx_time, cfg.frame_profile, v, why))
                return false;
            f.set(TimestampField::T5A, v);
            break;
        case FrameType::Report:
            why = "report frames are not implemented";
            return false;
        }
        out = f;
        return true;
    }

    RangeAdmissionContext admission_ctx(const Timestamp& ref_rx,
                                        const FirstPathQuality& rx_fp,
                                        const TxSendEvidence* tx_ev) const
    {
        RangeAdmissionContext ctx;
        ctx.calibration = &cfg.local_calibration;
        ctx.reference_ticks = ref_rx.ticks;
        ctx.reference_ticks_recorded = true;
        ctx.rx_first_path = rx_fp;
        ctx.tx_evidence = tx_ev;
        return ctx;
    }

    // Local interval admission through the REAL strict gate.
    bool admit_local(const Timestamp& later, const Timestamp& earlier,
                     const Timestamp& ref_rx, const FirstPathQuality& rx_fp,
                     const TxSendEvidence* tx_ev, std::optional<AdmittedRangingInterval>& out,
                     ExchangeStatus& status, std::string& why) const
    {
        const RangeAdmissionContext ctx = admission_ctx(ref_rx, rx_fp, tx_ev);
        const RangingIntervalAdmission a = admit_ranging_interval(later, earlier, ctx);
        if (!a.admitted) {
            status = a.status;
            why = std::string("local admission refused: ") +
                  range_admission_reason_to_string(a.reason) + " (" + a.detail + ")";
            return false;
        }
        out = *a.value;
        return true;
    }

    bool make_peer_interval(const PeerTimestampClaim& later,
                            const PeerTimestampClaim& earlier, ProtocolInterval& out,
                            std::string& why) const
    {
        // Runtime enforcement of the session binding (F's finding B-3).  The
        // wire bytes alone cannot prove they belong to the current session, so
        // a binding whose generation no longer matches this endpoint's session
        // is refused instead of being used.  This is the producer for
        // `PeerClaimError::SessionGenerationMismatch`.
        if (cfg.peer_binding.session_generation != cfg.session_generation) {
            why = peer_claim_error_to_string(PeerClaimError::SessionGenerationMismatch);
            return false;
        }
        const PeerClaimError e =
            ProtocolInterval::make_peer(later, earlier, cfg.peer_binding, out);
        if (e != PeerClaimError::Ok) {
            why = peer_claim_error_to_string(e);
            return false;
        }
        return true;
    }

    // ---- completion evaluation --------------------------------------------
    //
    // `maybe_finish` is the single place an exchange decides whether it can
    // reach a terminal result.  It first checks that the LOCAL transmit
    // evidence the result depends on has converged; if it has not, it records
    // an EVIDENCE deadline (M1-B §4.2) so the wait is finite even when the
    // whole-exchange timeout is longer or unset, and re-arms the driver's
    // deadline hint.  A pending result is never published as a success.
    void maybe_finish(CoreActionBatch& out, uint64_t event_id)
    {
        if (!in_flight)
            return;

        const TxRecord* need = nullptr;
        int64_t base = 0;
        FrameType expect = FrameType::Response;
        if (cfg.role == Role::Initiator && cfg.protocol == Protocol::Ss) {
            if (state != EndpointState::ResponseReceived)
                return;
            need = &tx_poll;
            base = rx_response_time.ticks;
            expect = FrameType::Response;
        } else if (cfg.role == Role::Initiator && cfg.protocol == Protocol::Ds) {
            if (state != EndpointState::FinalSent)
                return;
            need = &tx_final;
            base = rx_response_time.ticks;
            expect = FrameType::Final;
        } else if (cfg.role == Role::Responder && cfg.protocol == Protocol::Ss) {
            if (state != EndpointState::PollReceived)
                return;
            need = &tx_response;
            base = rx_poll_time.ticks;
            expect = FrameType::Response;
        } else {
            if (state != EndpointState::FinalReceived)
                return;
            need = &tx_response;
            base = rx_final_time.ticks;
            expect = FrameType::Final;
        }

        if (!need->completed()) {
            if (cfg.evidence_wait_ticks > 0) {
                if (!awaiting_evidence) {
                    awaiting_evidence = true;
                    evidence_deadline_ticks = base + cfg.evidence_wait_ticks;
                    // Tighten the driver's deadline hint so it can post a
                    // Deadline at the evidence bound; the action also states
                    // the frame this endpoint is still waiting for (or would
                    // be, for an endpoint with no air wait).
                    deadline_ticks = evidence_deadline_ticks;
                    arm_rx(out, expect, event_id);
                }
            }
            return;
        }
        awaiting_evidence = false;

        if (cfg.role == Role::Initiator && cfg.protocol == Protocol::Ss) {
            finish_ss_initiator(out, event_id);
            return;
        }
        if (cfg.role == Role::Initiator && cfg.protocol == Protocol::Ds) {
            ProtocolTofEstimate est = make_protocol_complete_without_estimate(
                cfg.protocol, ComputedAt::InitiatorA,
                "DS initiator: Final transmitted; the responder computes the estimate");
            terminal(out, std::move(est), ExchangeStatus::Ok,
                     "DS initiator exchange complete", event_id);
            return;
        }
        if (cfg.role == Role::Responder && cfg.protocol == Protocol::Ss) {
            ProtocolTofEstimate est = make_protocol_complete_without_estimate(
                cfg.protocol, ComputedAt::ResponderB,
                "SS responder: Response transmitted; the initiator computes the estimate");
            terminal(out, std::move(est), ExchangeStatus::Ok,
                     "SS responder exchange complete", event_id);
            return;
        }
        finish_ds_responder(out, event_id);
    }

    void finish_ss_initiator(CoreActionBatch& out, uint64_t event_id)
    {
        // RA = t4A - t1A  (local, admitted)
        std::optional<AdmittedRangingInterval> ra;
        ExchangeStatus st = ExchangeStatus::InternalError;
        std::string why;
        if (!admit_local(rx_response_time, tx_poll.planned_tx_time, rx_response_time,
                         rx_response_fp, &tx_poll.evidence, ra, st, why)) {
            fail(out, st, "SS initiator: " + why, event_id);
            return;
        }
        // DB = t3B - t2B  (peer wire claim)
        ProtocolInterval db_iv;
        if (!make_peer_interval(c_t3B, c_t2B, db_iv, why)) {
            fail(out, ExchangeStatus::InvalidTimeDomain,
                 "SS initiator: peer interval refused: " + why, event_id);
            return;
        }
        const ProtocolInterval ra_iv = ProtocolInterval::from_local(*ra);
        ProtocolTofEstimate est = compute_protocol_ss_tof(ra_iv, db_iv, cfg.ratio);
        const ExchangeStatus tst = est.estimate_available ? ExchangeStatus::Ok
                                                          : est.failure_reason;
        terminal(out, std::move(est), tst, "SS initiator protocol estimate", event_id);
    }

    void finish_ds_responder(CoreActionBatch& out, uint64_t event_id)
    {
        std::optional<AdmittedRangingInterval> db;
        std::optional<AdmittedRangingInterval> rb;
        ExchangeStatus st = ExchangeStatus::InternalError;
        std::string why;
        // DB = t3B - t2B  (local: B's own Response plan minus Poll RX)
        if (!admit_local(tx_response.planned_tx_time, rx_poll_time, rx_poll_time,
                         rx_poll_fp, &tx_response.evidence, db, st, why)) {
            fail(out, st, "DS responder: " + why, event_id);
            return;
        }
        // RB = t6B - t3B  (local: Final RX minus Response plan)
        if (!admit_local(rx_final_time, tx_response.planned_tx_time, rx_final_time,
                         rx_final_fp, &tx_response.evidence, rb, st, why)) {
            fail(out, st, "DS responder: " + why, event_id);
            return;
        }
        // RA = t4A - t1A, DA = t5A - t4A  (peer wire claims from Final)
        ProtocolInterval ra_iv;
        ProtocolInterval da_iv;
        if (!make_peer_interval(c_t4A, c_t1A, ra_iv, why)) {
            fail(out, ExchangeStatus::InvalidTimeDomain,
                 "DS responder: peer RA refused: " + why, event_id);
            return;
        }
        if (!make_peer_interval(c_t5A, c_t4A, da_iv, why)) {
            fail(out, ExchangeStatus::InvalidTimeDomain,
                 "DS responder: peer DA refused: " + why, event_id);
            return;
        }
        ProtocolTofEstimate est = compute_protocol_ds_tof(
            ra_iv, ProtocolInterval::from_local(*rb), da_iv,
            ProtocolInterval::from_local(*db), cfg.ratio);
        const ExchangeStatus tst = est.estimate_available ? ExchangeStatus::Ok
                                                          : est.failure_reason;
        terminal(out, std::move(est), tst, "DS responder protocol estimate", event_id);
    }

    // ---- event handlers ----------------------------------------------------

    void on_begin(const CoreEvent& ev, CoreActionBatch& out)
    {
        if (in_flight) {
            counters.requests_rejected_queue_full++;
            return;
        }
        if (cfg.role != Role::Initiator) {
            // A responder does not originate exchanges; a Begin is a no-op.
            counters.contract_violations++;
            return;
        }
        if (results.size() >= cfg.result_queue_capacity) {
            counters.requests_rejected_queue_full++;
            return;
        }
        if (seq_issued >= cfg.sequence_modulus) {
            // The wire sequence space of this session generation is exhausted.
            // A new exchange must wait for a NEW session (or a defined reuse
            // barrier); reusing an identity that may still be alive is refused.
            counters.stale_events++;
            return;
        }
        if (!ev.exchange.valid || ev.exchange.value == 0) {
            counters.contract_violations++;
            return;
        }

        in_flight = true;
        exchange = ev.exchange;
        current_seq = static_cast<uint16_t>(
            (static_cast<uint64_t>(cfg.initial_sequence) + seq_issued) %
            static_cast<uint64_t>(cfg.sequence_modulus));
        seq_issued++;
        state = EndpointState::PollSent;
        counters.accepted_exchanges++;
        if (counters.in_flight_peak < 1)
            counters.in_flight_peak = 1;

        deadline_ticks = (cfg.exchange_timeout_ticks > 0)
                             ? ev.now_ticks + cfg.exchange_timeout_ticks
                             : 0;
        arm_rx(out, FrameType::Response, ev.event_id);
        const TxToken t = next_token();
        pending_prepare = true;
        pending_token = t;
        pending_intent = FrameType::Poll;
        emit_prepare(out, t, FrameType::Poll, ev.event_id);
    }

    void on_rx_frame(const CoreEvent& ev, CoreActionBatch& out)
    {
        // PHY verdicts first: a bad FCS or a failed decode is counted and
        // never triggers a reply (M1-B §4.3).
        if (!ev.decode_ok) {
            counters.frames_rejected_decode++;
            counters.noise_frames++;
            return;
        }
        if (!ev.fcs_passed) {
            counters.frames_rejected_fcs++;
            counters.noise_frames++;
            return;
        }
        if (!validate_frame(ev.frame))
            return;

        if (cfg.role == Role::Responder) {
            if (!in_flight) {
                if (ev.frame.function_code != FrameType::Poll) {
                    // A response/final with no exchange: unrelated noise; it
                    // does NOT create a false exchange.
                    counters.noise_frames++;
                    return;
                }
                if (ev.frame.session_id != cfg.session_id) {
                    counters.frames_rejected_session++;
                    return;
                }
                if (ev.frame.pan_id != cfg.pan_id ||
                    ev.frame.dst_addr != cfg.local_address) {
                    counters.frames_rejected_wrong_peer++;
                    return;
                }
                if (ev.frame.seq >= cfg.sequence_modulus) {
                    counters.frames_rejected_seq++;
                    return;
                }
                if (seq_used[ev.frame.seq]) {
                    // A replay of an already-consumed wire sequence: the old
                    // exchange's identity may still be alive, so it is refused
                    // rather than reused (requires a fresh session).
                    counters.frames_rejected_seq++;
                    counters.stale_events++;
                    return;
                }
                // Adopt the Poll as a real exchange.
                seq_used[ev.frame.seq] = true;
                in_flight = true;
                exchange.valid = true;
                exchange.value = ev.event_id;
                current_seq = ev.frame.seq;
                state = EndpointState::PollReceived;
                counters.accepted_exchanges++;
                if (counters.in_flight_peak < 1)
                    counters.in_flight_peak = 1;
                rx_poll_time = ev.rx_time;
                rx_poll_fp = ev.rx_first_path;
                have_rx_poll = true;
                deadline_ticks = (cfg.exchange_timeout_ticks > 0)
                                     ? ev.rx_time.ticks + cfg.exchange_timeout_ticks
                                     : 0;
                counters.frames_accepted++;
                const TxToken t = next_token();
                pending_prepare = true;
                pending_token = t;
                pending_intent = FrameType::Response;
                emit_prepare(out, t, FrameType::Response, ev.event_id);
                return;
            }
            // In flight (Poll already accepted; DS awaiting Final).
            if (cfg.protocol == Protocol::Ds && state == EndpointState::PollReceived &&
                ev.frame.function_code == FrameType::Final) {
                if (ev.frame.session_id != cfg.session_id) {
                    counters.frames_rejected_session++;
                    return;
                }
                if (!seq_matches(current_seq, ev.frame.seq)) {
                    counters.frames_rejected_seq++;
                    return;
                }
                if (ev.frame.pan_id != cfg.pan_id ||
                    ev.frame.dst_addr != cfg.local_address) {
                    counters.frames_rejected_wrong_peer++;
                    return;
                }
                rx_final_time = ev.rx_time;
                rx_final_fp = ev.rx_first_path;
                have_rx_final = true;
                PeerTimestampClaim a1, a4, a5;
                if (!make_peer_claim(ev.frame, TimestampField::T1A, a1) ||
                    !make_peer_claim(ev.frame, TimestampField::T4A, a4) ||
                    !make_peer_claim(ev.frame, TimestampField::T5A, a5)) {
                    counters.frames_rejected_decode++;
                    return;
                }
                c_t1A = a1;
                c_t4A = a4;
                c_t5A = a5;
                have_peer_final = true;
                counters.frames_accepted++;
                state = EndpointState::FinalReceived;
                maybe_finish(out, ev.event_id);
                return;
            }
            // Anything else while in flight: a duplicate/late frame, NOT a
            // reason to cancel the correct in-flight request.
            counters.duplicate_frames++;
            return;
        }

        // ---- initiator -----------------------------------------------------
        if (!in_flight) {
            counters.noise_frames++;
            return;
        }
        if (ev.frame.function_code != FrameType::Response) {
            counters.frames_rejected_wrong_type++;
            return;
        }
        if (ev.frame.session_id != cfg.session_id) {
            counters.frames_rejected_session++;
            return;
        }
        if (!seq_matches(current_seq, ev.frame.seq)) {
            counters.frames_rejected_seq++;
            return;
        }
        if (ev.frame.pan_id != cfg.pan_id || ev.frame.dst_addr != cfg.local_address) {
            counters.frames_rejected_wrong_peer++;
            return;
        }
        if (state != EndpointState::PollSent) {
            counters.duplicate_frames++;
            return;
        }
        PeerTimestampClaim b2, b3;
        if (!make_peer_claim(ev.frame, TimestampField::T2B, b2) ||
            !make_peer_claim(ev.frame, TimestampField::T3B, b3)) {
            counters.frames_rejected_decode++;
            return;
        }
        c_t2B = b2;
        c_t3B = b3;
        have_peer_response = true;
        rx_response_time = ev.rx_time;
        rx_response_fp = ev.rx_first_path;
        have_rx_response = true;
        counters.frames_accepted++;
        state = EndpointState::ResponseReceived;

        if (cfg.protocol == Protocol::Ds) {
            // The initiator owes a Final and owns no estimate.
            const TxToken t = next_token();
            pending_prepare = true;
            pending_token = t;
            pending_intent = FrameType::Final;
            emit_prepare(out, t, FrameType::Final, ev.event_id);
            return;
        }
        maybe_finish(out, ev.event_id);
    }

    void on_tx_planned(const CoreEvent& ev, CoreActionBatch& out)
    {
        if (!pending_prepare || !ev.token.valid || !(ev.token == pending_token)) {
            counters.tx_token_mismatch++;
            return;
        }
        if (ev.tx_intent != pending_intent) {
            counters.contract_violations++;
            emit_abort(out, pending_token, ev.event_id);
            fail(out, ExchangeStatus::InternalError,
                 "TxPlanned intent does not match the outstanding PrepareTx", ev.event_id);
            return;
        }
        // Domain check first (N07).
        if (!tx_outcome_is_known(ev.tx_evidence.outcome) ||
            !timestamp_source_is_known(ev.planned_tx_time.source)) {
            counters.events_rejected_out_of_domain++;
            emit_abort(out, pending_token, ev.event_id);
            fail(out, ExchangeStatus::InternalError,
                 "TxPlanned carries an out-of-domain enum value", ev.event_id);
            return;
        }
        if (!ev.tx_evidence.records_the_plan()) {
            emit_abort(out, pending_token, ev.event_id);
            fail(out, ExchangeStatus::InvalidTimeDomain,
                 "TxPlanned did not record the full transmit plan", ev.event_id);
            return;
        }
        if (!ev.deadline_verdict_feasible) {
            counters.tx_submit_rejected_deadline++;
            emit_abort(out, pending_token, ev.event_id);
            fail(out, ExchangeStatus::DeadlineMissed,
                 "the adapter's deadline verdict says the plan cannot be submitted",
                 ev.event_id);
            return;
        }

        TxRecord* rec = nullptr;
        switch (pending_intent) {
        case FrameType::Poll:
            rec = &tx_poll;
            break;
        case FrameType::Response:
            rec = &tx_response;
            break;
        case FrameType::Final:
            rec = &tx_final;
            break;
        case FrameType::Report:
            emit_abort(out, pending_token, ev.event_id);
            fail(out, ExchangeStatus::Unsupported, "report frames are not implemented",
                 ev.event_id);
            return;
        }
        rec->present = true;
        rec->token = ev.token;
        rec->intent = pending_intent;
        rec->planned_tx_time = ev.planned_tx_time;
        rec->evidence = ev.tx_evidence;
        rec->outcome = ev.tx_evidence.outcome;

        Frame f;
        std::string why;
        if (!build_frame(pending_intent, *rec, f, why)) {
            emit_abort(out, pending_token, ev.event_id);
            fail(out, ExchangeStatus::InvalidTimeDomain,
                 "could not build the frame from the plan: " + why, ev.event_id);
            return;
        }
        rec->frame = f;

        CoreAction a;
        a.kind = CoreActionKind::SubmitTx;
        a.event_id = ev.event_id;
        a.exchange = exchange;
        a.token = ev.token;
        a.tx_intent = pending_intent;
        std::string err;
        FrameError code = FrameError::None;
        size_t written = 0;
        if (!encode_into(f, cfg.frame_profile, a.bytes, sizeof(a.bytes), written, err,
                         &code)) {
            emit_abort(out, pending_token, ev.event_id);
            fail(out, ExchangeStatus::InvalidTimeDomain,
                 "frame encode failed: " + err, ev.event_id);
            return;
        }
        a.nbytes = written;
        out.push(a);
        rec->submitted = true;
        counters.tx_submitted++;

        if (pending_intent == FrameType::Response) {
            state = EndpointState::PollReceived;
            if (cfg.protocol == Protocol::Ds) {
                deadline_ticks = (cfg.exchange_timeout_ticks > 0)
                                     ? ev.planned_tx_time.ticks + cfg.exchange_timeout_ticks
                                     : 0;
                arm_rx(out, FrameType::Final, ev.event_id);
            }
        } else if (pending_intent == FrameType::Final) {
            state = EndpointState::FinalSent;
        }
        pending_prepare = false;
        maybe_finish(out, ev.event_id);
    }

    void on_tx_accepted(const CoreEvent& ev, CoreActionBatch& out)
    {
        (void)out;
        TxRecord* rec = find_record(ev.token);
        if (rec == nullptr) {
            counters.tx_token_mismatch++;
            return;
        }
        rec->accepted = true;
        counters.tx_accepted++;
    }

    void on_tx_outcome(const CoreEvent& ev, CoreActionBatch& out)
    {
        if (!tx_outcome_is_known(ev.tx_outcome) || !adapter_fault_is_known(ev.adapter_fault)) {
            counters.events_rejected_out_of_domain++;
            fail(out, ExchangeStatus::InternalError,
                 "TxOutcomeResolved carries an out-of-domain enum value", ev.event_id);
            return;
        }
        TxRecord* rec = find_record(ev.token);
        if (rec == nullptr) {
            counters.tx_token_mismatch++;
            return;
        }
        if (ev.adapter_fault != AdapterFault::None) {
            const ExchangeStatus st = adapter_fault_to_exchange_status(ev.adapter_fault);
            counters.tx_outcome_failed++;
            fail(out, st,
                 std::string("adapter fault on a transmit: ") +
                     adapter_fault_to_string(ev.adapter_fault),
                 ev.event_id);
            return;
        }
        rec->outcome = ev.tx_outcome;
        rec->evidence.outcome = ev.tx_outcome;
        if (ev.tx_outcome == TxOutcome::Completed) {
            counters.tx_outcome_completed++;
            maybe_finish(out, ev.event_id);
        } else {
            counters.tx_outcome_failed++;
            ExchangeStatus st = ExchangeStatus::TxLate;
            switch (ev.tx_outcome) {
            case TxOutcome::Completed:
                break;
            case TxOutcome::Late:
                st = ExchangeStatus::TxLate;
                break;
            case TxOutcome::Underflow:
                st = ExchangeStatus::TxUnderflow;
                break;
            case TxOutcome::Cancelled:
                st = ExchangeStatus::Cancelled;
                break;
            case TxOutcome::Unknown:
                st = ExchangeStatus::InternalError;
                break;
            }
            fail(out, st, std::string("transmit outcome ") +
                              tx_outcome_to_string(ev.tx_outcome),
                 ev.event_id);
        }
    }

    TxRecord* find_record(const TxToken& t)
    {
        if (!t.valid)
            return nullptr;
        if (tx_poll.present && tx_poll.token == t)
            return &tx_poll;
        if (tx_response.present && tx_response.token == t)
            return &tx_response;
        if (tx_final.present && tx_final.token == t)
            return &tx_final;
        return nullptr;
    }

    void on_deadline(const CoreEvent& ev, CoreActionBatch& out)
    {
        counters.deadline_events++;
        if (!in_flight)
            return;
        // The evidence bound is stricter than the whole-exchange bound and is
        // checked first so a pending result that never converges is reported
        // as an evidence timeout rather than a generic protocol timeout.
        if (awaiting_evidence && cfg.evidence_wait_ticks > 0 &&
            ev.now_ticks >= evidence_deadline_ticks) {
            counters.timeouts++;
            fail(out, ExchangeStatus::ProtocolTimeout,
                 "the local transmit evidence did not converge before the evidence deadline",
                 ev.event_id);
            return;
        }
        if (deadline_ticks != 0 && ev.now_ticks >= deadline_ticks) {
            counters.timeouts++;
            fail(out, ExchangeStatus::ProtocolTimeout,
                 "the exchange deadline expired before a terminal result", ev.event_id);
        }
    }

    void on_cancel(const CoreEvent& ev, CoreActionBatch& out)
    {
        if (!in_flight)
            return;
        // Domain test FIRST (N07 / F's finding N-2): `cancel_reason` is a
        // public ExchangeStatus the caller supplies, so it can hold a value
        // the enum does not have.  An out-of-domain reason is replaced by the
        // default cancellation reason rather than stored as-is.
        ExchangeStatus st = ev.cancel_reason;
        if (static_cast<int>(st) > static_cast<int>(ExchangeStatus::NegativeTof) ||
            st == ExchangeStatus::Ok)
            st = ExchangeStatus::Cancelled;
        counters.cancellations++;
        if (pending_prepare)
            emit_abort(out, pending_token, ev.event_id);
        fail(out, st, "the exchange was cancelled", ev.event_id);
    }

    void on_stop(const CoreEvent& ev, CoreActionBatch& out)
    {
        if (!in_flight)
            return;
        counters.cancellations++;
        if (pending_prepare)
            emit_abort(out, pending_token, ev.event_id);
        fail(out, ExchangeStatus::Cancelled, "the endpoint was stopped", ev.event_id);
    }

    void on_overflow(const CoreEvent& ev, CoreActionBatch& out)
    {
        counters.overflow_events++;
        if (!in_flight)
            return;
        ExchangeStatus st = ExchangeStatus::QueueFull;
        if (adapter_fault_is_known(ev.adapter_fault) &&
            ev.adapter_fault == AdapterFault::RxOverflow)
            st = ExchangeStatus::RxOverflow;
        if (pending_prepare)
            emit_abort(out, pending_token, ev.event_id);
        fail(out, st, "the adapter reported an overflow/backpressure event", ev.event_id);
    }

    void on_reset(const CoreEvent& ev, CoreActionBatch& out)
    {
        counters.resets++;
        cfg.session_generation = (ev.new_generation != 0) ? ev.new_generation
                                                          : cfg.session_generation + 1;
        seq_issued = 0;
        if (in_flight) {
            if (pending_prepare)
                emit_abort(out, pending_token, ev.event_id);
            fail(out, ExchangeStatus::StaleSession,
                 "a local reset invalidated the in-flight exchange", ev.event_id);
        }
        counters.stale_events++;
    }
};

// ===========================================================================
// Public API
// ===========================================================================

EndpointCore::EndpointCore() : d_impl(new Impl()) {}
EndpointCore::~EndpointCore() { delete d_impl; }

bool EndpointCore::configure(const CoreConfig& cfg, std::string& why)
{
    Impl& d = *d_impl;

    if (!validate_core_config(cfg, why))
        return false;

    d.cfg = cfg;
    d.configured = true;
    d.counters = CoreCounters{};
    d.clear_exchange();
    d.state = EndpointState::Idle;
    d.seq_issued = 0;
    d.token_counter = 0;
    d.results.clear();
    d.results.reserve(cfg.result_queue_capacity);
    d.seq_used.fill(false);
    return true;
}

// Start a new session generation.  F's review (finding B-2) reproduced that
// the old implementation called `configure()`, which silently ERASED an
// in-flight exchange's terminal and every counter.  The instruction's
// conservation rule (`accepted == terminal + in_flight`, REQ-LIFE-01) forbids
// losing an accepted exchange's terminal, so a reset with an exchange in
// flight is REFUSED: the caller must post a `Reset` event (which fails the
// in-flight exchange with a terminal result) or a `Stop` event first.
//
// Counters are observability and are NOT cleared here.
bool EndpointCore::reset(const CoreConfig& cfg, uint64_t new_generation, std::string& why)
{
    Impl& d = *d_impl;
    if (d.in_flight) {
        why = "cannot reset while an exchange is in flight; post a Reset/Stop "
              "event first so its terminal result is not lost";
        return false;
    }
    if (!validate_core_config(cfg, why))
        return false;

    d.cfg = cfg;
    // A reset MUST declare a fresh peer binding: reusing a binding from the
    // previous generation would let same wire bytes look like a new session
    // (M1-B §4.1 / finding B-3).  The requested generation is the one the
    // binding must name.
    d.cfg.session_generation =
        (new_generation != 0) ? new_generation : cfg.session_generation + 1;
    if (d.cfg.peer_binding.session_generation != d.cfg.session_generation) {
        why = "reset requires a fresh peer_binding for the new session generation";
        return false;
    }
    d.configured = true;
    d.state = EndpointState::Idle;
    d.seq_issued = 0;
    d.token_counter = 0;
    d.results.clear();
    d.results.reserve(cfg.result_queue_capacity);
    d.seq_used.fill(false);
    return true;
}

bool EndpointCore::is_configured() const { return d_impl->configured; }
const CoreConfig& EndpointCore::config() const { return d_impl->cfg; }
Protocol EndpointCore::protocol() const { return d_impl->cfg.protocol; }
Role EndpointCore::role() const { return d_impl->cfg.role; }
EndpointState EndpointCore::state() const { return d_impl->state; }
const CoreCounters& EndpointCore::counters() const { return d_impl->counters; }

uint64_t EndpointCore::in_flight() const { return d_impl->in_flight ? 1u : 0u; }

CoreActionBatch EndpointCore::post(const CoreEvent& ev)
{
    CoreActionBatch out;
    Impl& d = *d_impl;
    if (!d.configured) {
        d.counters.events_rejected_out_of_domain++;
        return out;
    }
    // Domain test FIRST (N07): an out-of-domain kind is counted and produces
    // no action and no terminal result.
    if (!core_event_kind_is_known(ev.kind)) {
        d.counters.events_rejected_out_of_domain++;
        return out;
    }
    switch (ev.kind) {
    case CoreEventKind::Begin:
        d.on_begin(ev, out);
        break;
    case CoreEventKind::RxFrame:
        d.on_rx_frame(ev, out);
        break;
    case CoreEventKind::TxPlanned:
        d.on_tx_planned(ev, out);
        break;
    case CoreEventKind::TxAccepted:
        d.on_tx_accepted(ev, out);
        break;
    case CoreEventKind::TxOutcomeResolved:
        d.on_tx_outcome(ev, out);
        break;
    case CoreEventKind::Deadline:
        d.on_deadline(ev, out);
        break;
    case CoreEventKind::Cancel:
        d.on_cancel(ev, out);
        break;
    case CoreEventKind::Stop:
        d.on_stop(ev, out);
        break;
    case CoreEventKind::Overflow:
        d.on_overflow(ev, out);
        break;
    case CoreEventKind::Reset:
        d.on_reset(ev, out);
        break;
    }
    return out;
}

bool EndpointCore::pop_result(ProtocolTofEstimate& out)
{
    Impl& d = *d_impl;
    if (d.results.empty())
        return false;
    out = d.results.front();
    d.results.erase(d.results.begin());
    return true;
}

std::string EndpointCore::to_string() const
{
    const Impl& d = *d_impl;
    char buf[512];
    std::snprintf(buf, sizeof(buf),
                  "EndpointCore{%s role=%s protocol=%s state=%s configured=%s "
                  "in_flight=%llu results=%zu}",
                  d.cfg.endpoint_id.c_str(), role_to_string(d.cfg.role),
                  protocol_to_string(d.cfg.protocol), endpoint_state_to_string(d.state),
                  d.configured ? "true" : "false",
                  static_cast<unsigned long long>(d.in_flight ? 1 : 0),
                  static_cast<size_t>(d.results.size()));
    return std::string(buf);
}

} // namespace twr
} // namespace uwb
} // namespace gr
