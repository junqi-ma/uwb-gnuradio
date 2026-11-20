/* -*- c++ -*- */
/*
 * M1-B installed-consumer example: drive the PUBLIC protocol core by hand.
 *
 * This program includes ONLY the installed M1-B / M0 headers and links ONLY
 * the standalone `uwb_twr_core` archive.  It deliberately does NOT use the
 * fake link (which is not installed) and does NOT touch GNU Radio or UHD, so
 * running it from an isolated prefix proves the public core is radio-free
 * (M1-B instruction §8).
 *
 * It runs one SS and one DS exchange between two hand-driven endpoints.  The
 * events (planned TX instants, RX instants, outcomes) are written out by hand
 * -- there is no scheduler and no clock: time advances because the program
 * says so.  The expected ToF is a physical constant of the scenario
 * (propagation = 100 ticks one way), NOT derived from the core.
 *
 * Exit 0 only when both exchanges produce the expected exact estimate.
 */

#include <gnuradio/uwb/uwb_twr_core.h>

#include <cstdio>
#include <cstring>
#include <string>

using namespace gr::uwb::twr;

namespace {

ClockDomain g_dom;
FrameProfile g_prof;
CalibrationStamp g_cal;

Timestamp rx_ts(int64_t t)
{
    Timestamp ts;
    Timestamp::from_ticks(t, g_dom, TimestampMarker::RmarkerRx,
                          TimestampSource::HardwareMeasured,
                          timestamp_required_corrections(TimestampMarker::RmarkerRx), ts);
    ts.calibration_id = g_cal.id;
    return ts;
}

TxPlan good_plan(int64_t air)
{
    TxPlan p;
    p.valid = true;
    p.domain = g_dom;
    p.source = TimestampSource::ScheduledCalibrated;
    p.applied_corrections = timestamp_required_corrections(TimestampMarker::RmarkerTx);
    p.calibration_id = g_cal.id;
    p.marker_offset_ticks = 10;
    p.quantised_instant_ticks = air - 10;
    p.command_time_ticks = air - 20;
    p.calibrated_air_ticks = air;
    return p;
}

WireTimestampBinding binding()
{
    WireTimestampBinding b;
    b.peer_domain = g_dom;
    b.session_generation = 1;
    b.binding_generation = 1;
    b.peer_marker = TimestampMarker::RmarkerTx;
    b.unit_convention = "device_ticks";
    b.calibration_convention_id = g_cal.id;
    b.max_interval_ticks = 1000000;
    b.sequence_modulus = 4;
    return b;
}

CoreConfig cfg_of(Role role, Protocol proto)
{
    CoreConfig c;
    c.endpoint_id = (role == Role::Initiator) ? "A" : "B";
    c.protocol = proto;
    c.role = role;
    c.pan_id = 0x1234;
    c.local_address = (role == Role::Initiator) ? 0x0101 : 0x0202;
    c.peer_address = (role == Role::Initiator) ? 0x0202 : 0x0101;
    c.session_id = 7;
    c.session_generation = 1;
    c.sequence_modulus = 4;
    c.initial_sequence = 0;
    c.local_domain = g_dom;
    c.peer_binding = binding();
    ClockRatio::unity_same_clock(g_dom, c.ratio);
    c.frame_profile = g_prof;
    c.local_calibration = g_cal;
    c.exchange_timeout_ticks = 100000;
    c.result_queue_capacity = 8;
    c.max_in_flight = 1;
    return c;
}

// Last parsed action state, kept simple because this is a linear driver.
struct Parse {
    bool have_prepare = false;
    TxToken token;
    FrameType submit_type = FrameType::Poll;
    uint8_t bytes[kMaxFrameBytes] = {};
    size_t nbytes = 0;
    bool terminal = false;
    bool estimate = false;
    std::string tof;
};

void scan(const CoreActionBatch& b, Parse& p)
{
    for (size_t i = 0; i < b.count; ++i) {
        const CoreAction& a = b.actions[i];
        if (a.kind == CoreActionKind::PrepareTx) {
            p.have_prepare = true;
            p.token = a.token;
        } else if (a.kind == CoreActionKind::SubmitTx) {
            p.submit_type = a.tx_intent;
            std::memcpy(p.bytes, a.bytes, sizeof(p.bytes));
            p.nbytes = a.nbytes;
        } else if (a.kind == CoreActionKind::TerminalResult) {
            p.terminal = true;
            p.estimate = a.result.estimate_available;
            p.tof = a.result.tof.to_string();
            // The result type's every range entry must be false.
            if (a.result.yields_range() || a.result.measurement_valid ||
                a.result.is_hardware_measurement()) {
                std::printf("FAIL: a protocol result claimed to be a measurement\n");
            }
        }
    }
}

CoreEvent ev_begin(uint64_t id, int64_t now)
{
    CoreEvent e;
    e.kind = CoreEventKind::Begin;
    e.generation = 1;
    e.exchange.valid = true;
    e.exchange.value = id;
    e.now_ticks = now;
    return e;
}
CoreEvent ev_planned(const TxToken& t, FrameType intent, int64_t air)
{
    CoreEvent e;
    e.kind = CoreEventKind::TxPlanned;
    e.generation = 1;
    e.token = t;
    e.tx_intent = intent;
    e.tx_plan = good_plan(air);
    e.deadline_verdict_feasible = true;
    return e;
}
CoreEvent ev_outcome(const TxToken& t, TxOutcome o)
{
    CoreEvent e;
    e.kind = CoreEventKind::TxOutcomeResolved;
    e.generation = 1;
    e.token = t;
    e.tx_outcome = o;
    e.adapter_fault = AdapterFault::None;
    return e;
}
CoreEvent ev_rx(const uint8_t* bytes, size_t n, int64_t t)
{
    CoreEvent e;
    e.kind = CoreEventKind::RxFrame;
    e.generation = 1;
    e.fcs_passed = true;
    e.decode_ok = true;
    e.rx_time = rx_ts(t);
    e.rx_first_path = FirstPathQuality::passed(20.0, 6.0, 0.9);
    std::string err;
    if (!decode(bytes, n, g_prof, e.frame, err)) {
        std::printf("FAIL: decode: %s\n", err.c_str());
        e.decode_ok = false;
    }
    return e;
}

bool run(Protocol proto, const char* expect_endpoint, int64_t expect_tof)
{
    EndpointCore A;
    EndpointCore B;
    std::string why;
    if (!A.configure(cfg_of(Role::Initiator, proto), why)) {
        std::printf("FAIL: A configure: %s\n", why.c_str());
        return false;
    }
    if (!B.configure(cfg_of(Role::Responder, proto), why)) {
        std::printf("FAIL: B configure: %s\n", why.c_str());
        return false;
    }

    const int64_t prop = 100;
    const int64_t t1 = 2000;
    const int64_t t2 = t1 + prop;
    const int64_t t3 = t2 + 500;
    const int64_t t4 = t3 + prop;
    const int64_t t5 = t4 + 300;
    const int64_t t6 = t5 + prop;

    Parse pa;
    pa.have_prepare = false;
    scan(A.post(ev_begin(1, 1000)), pa);
    const TxToken tokA = pa.token;
    scan(A.post(ev_planned(tokA, FrameType::Poll, t1)), pa);
    scan(A.post(ev_outcome(tokA, TxOutcome::Completed)), pa);

    uint8_t poll[kMaxFrameBytes];
    std::memcpy(poll, pa.bytes, sizeof(poll));
    const size_t poll_n = pa.nbytes;

    Parse pb;
    pb.have_prepare = false;
    scan(B.post(ev_rx(poll, poll_n, t2)), pb);
    const TxToken tokB = pb.token;
    scan(B.post(ev_planned(tokB, FrameType::Response, t3)), pb);
    uint8_t resp[kMaxFrameBytes];
    std::memcpy(resp, pb.bytes, sizeof(resp));
    const size_t resp_n = pb.nbytes;
    scan(B.post(ev_outcome(tokB, TxOutcome::Completed)), pb);

    scan(A.post(ev_rx(resp, resp_n, t4)), pa);

    if (proto == Protocol::Ds) {
        const TxToken tokF = pa.token;
        scan(A.post(ev_planned(tokF, FrameType::Final, t5)), pa);
        uint8_t fin[kMaxFrameBytes];
        std::memcpy(fin, pa.bytes, sizeof(fin));
        const size_t fin_n = pa.nbytes;
        scan(A.post(ev_outcome(tokF, TxOutcome::Completed)), pa);
        scan(B.post(ev_rx(fin, fin_n, t6)), pb);
    }

    // Drain the bounded result queues.
    ProtocolTofEstimate ea;
    ProtocolTofEstimate eb;
    const bool ra = A.pop_result(ea);
    const bool rb = B.pop_result(eb);

    const bool initiator = (std::strcmp(expect_endpoint, "A") == 0);
    const bool got_estimate = initiator ? (ra && ea.estimate_available)
                                        : (rb && eb.estimate_available);
    const bool other_none = initiator ? (rb && !eb.estimate_available)
                                      : (ra && !ea.estimate_available);
    // Hold the string: `to_string()` returns a temporary, so taking `.c_str()`
    // from it would dangle (F's finding N-4).
    const std::string tof = initiator ? ea.tof.to_string() : eb.tof.to_string();
    const int64_t expected_num = expect_tof;

    std::printf("%s: estimate at %s = %s (expected %lld/1), other end none=%s\n",
                protocol_to_string(proto), expect_endpoint, tof.c_str(),
                static_cast<long long>(expected_num), other_none ? "yes" : "no");

    // Conservation on both endpoints.
    const CoreCounters& ca = A.counters();
    const CoreCounters& cb = B.counters();
    const bool cons = (ca.accepted_exchanges == ca.terminal_results + A.in_flight()) &&
                      (cb.accepted_exchanges == cb.terminal_results + B.in_flight());

    bool ok = got_estimate && other_none && cons;
    if (got_estimate) {
        const ProtocolTofEstimate& e = initiator ? ea : eb;
        ok = ok && e.tof.num == expected_num && e.tof.den == 1 &&
             e.measurement_valid == false && e.peer_evidence_is_wire_claim &&
             e.completion == ProtocolCompletionStatus::Complete;
    }
    if (!ok)
        std::printf("FAIL: %s exchange did not match the expectation\n",
                    protocol_to_string(proto));
    return ok;
}

} // namespace

int main()
{
    ClockDomain::make("x410_dev_ticks", 1.0e9, 1, 0, g_dom);
    g_prof.version = 1;
    g_prof.timestamp_bits = 40;
    g_prof.timestamp_unit_hz = 1.0e9;
    g_prof.max_psdu_bytes = 127;
    g_cal.id = "cal1";
    g_cal.calibrated_epoch = 1;
    g_cal.valid_from_ticks = 0;
    g_cal.valid_until_ticks = 1000000000LL;
    g_cal.applications.push_back(CalibrationApplication{ "cal1", CalibrationResult::Applied });

    std::printf("M1-B installed core consumer\n");
    const bool ss = run(Protocol::Ss, "A", 100);
    const bool ds = run(Protocol::Ds, "B", 100);
    if (ss && ds) {
        std::printf("ALL OK\n");
        return 0;
    }
    return 1;
}
