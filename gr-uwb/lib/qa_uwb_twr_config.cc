/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * QA for uwb_twr_config.h (TWR M0, REQ-API-01..05).
 *
 * What this suite is FOR:
 *   1. prove a config that satisfies the whitelist validates and yields a
 *      correct frozen effective_config() snapshot;
 *   2. prove DEFAULT DENY -- the central requirement.  Every combination that
 *      is not in the capability whitelist is rejected with a specific,
 *      machine-readable reason, and nothing is ever silently defaulted to the
 *      radar / QM35 behaviour (REQ-SCOPE-01);
 *   3. prove the mirrored constants (native rate, code index, max PSDU)
 *      still agree with the code they mirror, so the duplication in this
 *      UHD-free header cannot drift;
 *   4. prove NaN/Inf, index range, channel/frequency conflict, resource
 *      conflict, over-capacity, timing-precision and calibration rules each
 *      produce their own reason;
 *   5. prove the JSON round trip is stable and keeps integer precision
 *      (REQ-OUT-01).
 *
 * The MEASURED-PENDING PHY matrix (sync repetitions / SFD mode / data rate) is
 * NOT invented here.  This suite instead exercises the mechanism: it installs
 * a synthetic row through the public `CapabilitiesBuilder` and shows the same
 * config is REJECTED against the shipped default-deny whitelist.  When
 * testdata/twr/phy_matrix_*.csv lands, the real rows replace the synthetic one
 * and nothing else in the suite changes.
 */

#include <gnuradio/uwb/uwb_twr_config.h>
// The frame codec is the geometry authority the config layer validates
// against (M0.1 / R3).  Both headers are stdlib-only, so co-including them
// here is exactly what a consumer does.  qa_uwb_twr_frame.cc owns the other
// half of this contract.
#include <gnuradio/uwb/uwb_twr_frame.h>

#include <boost/test/unit_test.hpp>

// Cross-checks against the code this header mirrors.  These headers pull in
// GNU Radio (and, for the UHD backend config, the echo backend), so they are
// included HERE and never in uwb_twr_config.h itself, which must stay
// buildable without a radio.
#include <gnuradio/uwb/uwb_radar_pdu_meta.h>
#include <gnuradio/uwb/uwb_uhd_backend_config.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace twr = gr::uwb::twr;
namespace radar_meta = gr::uwb::radar_meta;
namespace uhd_cfg = gr::uwb::uhd;

using namespace gr::uwb::twr;

BOOST_AUTO_TEST_SUITE(twr_config)

namespace {

// ---------------------------------------------------------------------------
// Synthetic measured capability row.
//
// NOT a capability claim: the real matrix (testdata/twr/phy_matrix_*.csv) is
// still MEASURED-PENDING.  This row exists only so the default-deny mechanism
// can be exercised end to end without pretending the measurement is done.
// ---------------------------------------------------------------------------
Capabilities measured_caps()
{
    PhyCapabilityRow row;
    row.native_rate_hz = kTwrNativeRateUc200Hz;
    row.code_index = 9;
    row.sync_repetitions = 128;
    row.sfd_mode = SfdMode::R4z2;
    row.max_psdu_bytes = kTwrMaxPsduBytes;
    row.ranging = true;
    row.status = CapabilityStatus::Measured;
    row.reason = "SYNTHETIC qa_uwb_twr_config row; not a measurement";

    return CapabilitiesBuilder(build_default_capabilities())
        .add_data_rate(DataRate::R6p8M)
        .add_sync_repetitions(128)
        .add_sfd_mode(SfdMode::R4z2)
        .add_phy_row(row)
        .build();
}

TimedField device_field(int64_t ns, TimeReferenceEvent ref, TimestampMarker marker)
{
    TimedField t;
    t.value = Duration::from_nanos(ns);
    t.domain = TimeDomain::DeviceTicks;
    t.reference = ref;
    t.marker = marker;
    t.required_quantisation_hz = kTwrNativeRateUc200Hz;
    return t;
}

TimedField host_field(int64_t ns)
{
    TimedField t;
    t.value = Duration::from_nanos(ns);
    t.domain = TimeDomain::MonotonicHost;
    t.reference = TimeReferenceEvent::HostMonotonic;
    t.required_quantisation_hz = 1.0e9;
    return t;
}

// A minimal but COMPLETE config: every field is stated, nothing is assumed.
// The frame geometry is written out explicitly rather than taken from a
// helper, so a reader can see the exact byte budget being asserted.
TwrConfig minimal(Protocol protocol = Protocol::Ss, Role role = Role::Initiator)
{
    const bool ds = protocol == Protocol::Ds;
    const bool initiator = role == Role::Initiator;
    const uint8_t ch = initiator ? 0u : 1u;

    TwrConfig c;
    c.meta.schema_version = capabilities().schema_version;
    c.meta.profile_version = "qa-synthetic-frame/0";
    c.meta.calibration_version = "qa-synthetic-cal/0";
    c.meta.label = "qa_uwb_twr_config minimal";

    // 会话
    c.session.protocol = protocol;
    c.session.role = role;
    c.session.local_address = initiator ? 0x0001 : 0x0002;
    c.session.peer_address = initiator ? 0x0002 : 0x0001;
    c.session.pan_id = 0xcafe;
    c.session.session_id = 0x1234u;
    c.session.exchange_id = 1u;
    c.session.sequence = 7u;
    c.session.sequence_modulus = 256u;
    c.session.measurement_count = 1u;
    c.session.measurement_interval = Duration::from_nanos(20'000'000);
    c.session.max_attempts_per_exchange = 1u;
    c.session.retry_backoff = Duration::from_nanos(0);
    c.session.max_in_flight_exchanges = kTwrMaxInFlightExchanges;
    c.session.require_pan_match = true;
    c.session.require_address_match = true;

    // PHY
    c.phy.channel = 5u;
    c.phy.center_frequency_hz = 6489.6e6;
    c.phy.tx_preamble_code = 9u;
    c.phy.rx_preamble_code = 9u;
    // 64 SYNC is the MEASURED phase-1 profile (testdata/twr/
    // phy_matrix_737280000.csv).  128/256/512/2048 are measured-unsupported.
    c.phy.preamble_symbols = preamble_length_symbols(PreambleLength::Sym64);
    c.phy.prf_class = PrfClass::Bprf64;
    c.phy.data_rate = DataRate::R6p8M;
    // 0.85 Mb/s PHR + 6.81 Mb/s payload: the profile uwb_hrp_mod_core.h
    // actually modulates.  The two rates are separate fields.
    c.phy.phr_rate = PhrRate::Standard850k;

    // 帧格式
    c.frame.sfd_mode = SfdMode::R4z2;
    c.frame.sfd_symbols = sfd_mode_symbols(SfdMode::R4z2);
    c.frame.sfd_timeout =
        device_field(20'000, TimeReferenceEvent::RxEnable,
            TimestampMarker::UhdRxFirstIqSample);
    c.frame.phr_mode = PhrMode::Standard;
    c.frame.ranging_bit = true;
    // The geometry is a CLAIM about the layout, taken from the codec that
    // OWNS the layout: frame_geometry_of_profile() reads
    // frame_geometry_for() / frame_profile_for().  No number in this fixture
    // is written by hand, so a change to the codec cannot leave a stale copy
    // here (M0 review R3).
    BOOST_REQUIRE_MESSAGE(
        frame_geometry_of_profile(c.frame.frame_profile, c.frame.geometry),
        "the fixture could not read the frame geometry from the codec authority");
    // Final is the largest frozen TWR frame: 14 + 3*5 = 29 B of MAC PSDU.
    // mac_psdu_bytes EXCLUDES the FCS because fcs_append == PhyLayer, so the
    // 31 B on-air size is 29 + 2 and the 127-byte IEEE limit is applied to
    // that on-air length, not to this field.  It is read from the codec too.
    {
        FrameProfileGeometry auth;
        FrameProfile profile;
        BOOST_REQUIRE(frame_geometry_for(c.frame.frame_profile, auth));
        BOOST_REQUIRE(frame_profile_for(c.frame.frame_profile, profile));
        c.frame.mac_psdu_bytes = static_cast<uint16_t>(
            auth.mac_payload_bytes(FrameType::Final, profile));
    }
    c.frame.mac_psdu_includes_fcs = false;
    c.frame.fcs_append = FcsAppender::PhyLayer;
    c.frame.fcs_bytes = 2u;
    c.frame.application_payload_bytes = 8u;
    c.frame.sts_mode = StsMode::Off;
    c.frame.sts_length_symbols = 0u;

    // 发射: gain dB is the authority; IQ amplitude and dBm stay absent.
    c.tx.port = ch;
    c.tx.gain_db = Opt<double>(20.0);
    c.tx.power_policy = TxPowerPolicy::ManualGainDb;
    c.tx.pulse_shaping = PulseShaping::ExistingHrP;

    // 接收
    c.rx.port = ch;
    c.rx.gain_db = Opt<double>(30.0);
    c.rx.agc = AgcMode::Manual;
    c.rx.bandwidth_hz = Opt<double>(500.0e6);
    c.rx.detection_threshold = 0.30;
    c.rx.correlation_threshold = 0.35;
    c.rx.first_path_threshold = 0.25;
    c.rx.first_path_index = 8u;
    c.rx.first_path_window = 30u;

    // 无线设备
    c.radio.device_args = "addr=192.168.10.2";
    c.radio.tx_channel = ch;
    c.radio.rx_channel = ch;
    c.radio.native_sample_rate_hz = kTwrNativeRateUc200Hz;
    c.radio.clock_source = "internal";
    c.radio.time_source = "internal";
    c.radio.fpga_image = "";
    c.radio.dpdk_config = "";
    {
        EndpointBinding peer;
        peer.id = initiator ? "B" : "A";
        peer.role = initiator ? Role::Responder : Role::Initiator;
        peer.tx_channel = initiator ? 1u : 0u;
        peer.rx_channel = peer.tx_channel;
        peer.native_sample_rate_hz = kTwrNativeRateUc200Hz;
        peer.occupies_resources = true;
        c.radio.peers.push_back(peer);
    }
    c.radio.readback.present = true;
    c.radio.readback.sample_rate_hz = kTwrNativeRateUc200Hz;
    c.radio.readback.center_freq_hz = 6489.6e6;
    c.radio.readback.tx_channel = ch;
    c.radio.readback.rx_channel = ch;
    c.radio.readback.mpm_string = "X410";
    c.radio.readback.fpga_image = "";
    c.radio.readback.uhd_version = "4.x";
    c.radio.readback.clock_source = "internal";
    c.radio.readback.time_source = "internal";
    c.radio.require_readback = true;

    // 每包时间
    c.timing.poll_start =
        device_field(initiator ? 40'000'000'000LL : 0,
                     TimeReferenceEvent::PollTransmitRmarker,
                     TimestampMarker::RmarkerTx);
    if (initiator) {
        // The initiator owns NO reply delay.
        c.timing.poll_to_response = device_field(
            0, TimeReferenceEvent::PollReceiveRmarker, TimestampMarker::RmarkerRx);
        c.timing.response_to_final = ds
                                        ? device_field(600'000,
                                                       TimeReferenceEvent::ResponseReceiveRmarker,
                                                       TimestampMarker::RmarkerRx)
                                        : device_field(0,
                                                       TimeReferenceEvent::FinalTransmitRmarker,
                                                       TimestampMarker::RmarkerRx);
    } else {
        c.timing.poll_to_response = device_field(
            600'000, TimeReferenceEvent::PollReceiveRmarker, TimestampMarker::RmarkerRx);
        c.timing.response_to_final = device_field(
            0, TimeReferenceEvent::FinalTransmitRmarker, TimestampMarker::RmarkerRx);
    }
    c.timing.final_to_report =
        device_field(0, TimeReferenceEvent::ReportTransmitRmarker,
            TimestampMarker::RmarkerTx);
    // A third, unrelated quantity: arm RX after the end of our own frame.
    c.timing.post_tx_rx_enable =
        device_field(2'000, TimeReferenceEvent::FrameTail,
            TimestampMarker::PreambleStart);
    c.timing.min_tx_lead_time = device_field(
        20'000, TimeReferenceEvent::ResponseTransmitRmarker, TimestampMarker::RmarkerTx);

    // 超时
    c.timeouts.poll_rx_window = device_field(initiator ? 0 : 20'000,
                                             TimeReferenceEvent::RxEnable,
                                             TimestampMarker::UhdRxFirstIqSample);
    c.timeouts.response_rx_window = device_field(initiator ? 20'000 : 0,
                                                 TimeReferenceEvent::RxEnable,
                                                 TimestampMarker::UhdRxFirstIqSample);
    c.timeouts.final_rx_window = device_field((ds && !initiator) ? 20'000 : 0,
                                              TimeReferenceEvent::RxEnable,
                                              TimestampMarker::UhdRxFirstIqSample);
    c.timeouts.report_rx_window = device_field(0,
                                               TimeReferenceEvent::RxEnable,
                                               TimestampMarker::UhdRxFirstIqSample);
    c.timeouts.rx_timeout =
        device_field(40'000, TimeReferenceEvent::RxEnable,
            TimestampMarker::UhdRxFirstIqSample);
    c.timeouts.exchange_timeout = host_field(50'000'000);
    c.timeouts.retry_interval = host_field(0);

    // 时戳与校准
    c.calibration.tx_link_delay = Duration::from_nanos(120);
    c.calibration.rx_link_delay = Duration::from_nanos(180);
    c.calibration.antenna_delay = Duration::from_nanos(35);
    c.calibration.cable_delay = Duration::from_nanos(90);
    c.calibration.tx_link_delay_native_ticks = 0;
    c.calibration.rx_link_delay_native_ticks = 0;
    c.calibration.native_sample_rate_hz = kTwrNativeRateUc200Hz;
    c.calibration.link_delay_unit = TimeUnit::Nanoseconds;
    c.calibration.first_path_algorithm = FirstPathAlgorithm::LeadingEdge;
    c.calibration.cfo_compensation = CompensationFlag::Required;
    c.calibration.sfo_compensation = CompensationFlag::On;
    c.calibration.calibration_id = "qa-synthetic-cal/0";
    c.calibration.record.calibration_id = "qa-synthetic-cal/0";
    c.calibration.record.device_serial = "X410-QA";
    c.calibration.record.channel = 5u;
    c.calibration.record.native_sample_rate_hz = kTwrNativeRateUc200Hz;
    c.calibration.record.profile_version = "qa-synthetic-frame/0";
    c.calibration.record.gain_db = Opt<double>(20.0);
    c.calibration.record.valid_until_monotonic_ns = 0;
    c.calibration.applied_count = 0u;
    c.calibration.calibration_required = true;

    // 诊断
    c.diagnostics.cir_capture_enabled = false;
    c.diagnostics.cir_capture_max_bytes = 0u;
    c.diagnostics.cir_capture_stride = 0u;
    c.diagnostics.short_iq_enabled = false;
    c.diagnostics.short_iq_max_bytes = 0u;
    c.diagnostics.short_iq_stride = 0u;
    c.diagnostics.raw_frame_dump = false;
    c.diagnostics.raw_frame_max_bytes = 0u;
    c.diagnostics.result_output_path = "";
    c.diagnostics.result_queue_capacity = 256u;
    c.diagnostics.event_queue_capacity = 512u;
    c.diagnostics.stats_cadence = host_field(1'000'000'000);
    c.diagnostics.io_on_realtime_thread = false;
    return c;
}

bool has(const ValidationReport& r, const char* field, ConfigReason reason)
{
    return r.find(field, reason) != nullptr;
}

// Every rejection must be actionable: a field path, a reason, a status and a
// message.  A bare bool is never an acceptable outcome.
void require_machine_readable(const ValidationReport& r)
{
    BOOST_REQUIRE(!r.ok());
    BOOST_REQUIRE(!r.violations.empty());
    for (const auto& v : r.violations) {
        BOOST_REQUIRE(!v.message.empty());
        BOOST_REQUIRE(v.reason != ConfigReason::None);
        BOOST_REQUIRE(std::string(config_reason_to_string(v.reason)) != "invalid");
        BOOST_REQUIRE(std::string(exchange_status_to_string(v.status)) != "invalid");
        BOOST_REQUIRE(!v.requirement.empty());
        BOOST_REQUIRE(std::string(exchange_status_family(v.status)) != "invalid");
    }
}

} // namespace

// ===========================================================================
// A. Frozen types and mirrored constants
// ===========================================================================

BOOST_AUTO_TEST_CASE(frozen_types_unchanged)
{
    // The frozen contract this header builds on.
    BOOST_REQUIRE_EQUAL(protocol_to_string(Protocol::Ss), "ss");
    BOOST_REQUIRE_EQUAL(protocol_to_string(Protocol::Ds), "ds");
    BOOST_REQUIRE_EQUAL(role_to_string(Role::Initiator), "initiator");
    BOOST_REQUIRE_EQUAL(role_to_string(Role::Responder), "responder");
    BOOST_REQUIRE_EQUAL(static_cast<int>(FrameType::Poll), 0x00);
    BOOST_REQUIRE_EQUAL(static_cast<int>(FrameType::Response), 0x01);
    BOOST_REQUIRE_EQUAL(static_cast<int>(FrameType::Final), 0x02);
    BOOST_REQUIRE_EQUAL(static_cast<int>(FrameType::Report), 0x03);
    BOOST_REQUIRE_EQUAL(frame_type_timestamp_count(FrameType::Poll), 0u);
    BOOST_REQUIRE_EQUAL(frame_type_timestamp_count(FrameType::Response), 2u);
    BOOST_REQUIRE_EQUAL(frame_type_timestamp_count(FrameType::Final), 3u);
    BOOST_REQUIRE_CLOSE(twr::kTwrWorkSampleRateHz, 998.4e6, 1e-9);
    BOOST_REQUIRE_EQUAL(twr::kTwrWorkSamplesPerSymbol, 1016);
    // Only the DEFINED status values round trip; the gaps in the taxonomy are
    // reserved and must not acquire a name.
    const ExchangeStatus defined[] = {
        ExchangeStatus::Ok,          ExchangeStatus::ConfigRejected,
        ExchangeStatus::Unsupported, ExchangeStatus::Cancelled,
        ExchangeStatus::QueueFull,   ExchangeStatus::InternalError,
        ExchangeStatus::PhyFcsFailed, ExchangeStatus::PhyDecodeFailed,
        ExchangeStatus::WrongPeer,   ExchangeStatus::UnexpectedFrameType,
        ExchangeStatus::StaleSession, ExchangeStatus::RxTimeout,
        ExchangeStatus::RxOverflow,  ExchangeStatus::RxChainBroken,
        ExchangeStatus::TxLate,      ExchangeStatus::TxUnderflow,
        ExchangeStatus::TxSeqError,  ExchangeStatus::TxChainBroken,
        ExchangeStatus::InvalidTimeDomain, ExchangeStatus::ClockEstimateInvalid,
        ExchangeStatus::FirstPathUnreliable, ExchangeStatus::CalibrationMissing,
        ExchangeStatus::CalibrationExpired, ExchangeStatus::ProtocolTimeout,
        ExchangeStatus::DeadlineMissed
    };
    for (const ExchangeStatus s : defined) {
        ExchangeStatus back = ExchangeStatus::Ok;
        BOOST_REQUIRE(exchange_status_from_string(exchange_status_to_string(s), back));
        BOOST_REQUIRE(back == s);
    }
    for (int i = 0; i <= static_cast<int>(ExchangeStatus::DeadlineMissed); ++i) {
        const ExchangeStatus s = static_cast<ExchangeStatus>(i);
        if (std::string(exchange_status_to_string(s)) == "invalid")
            continue; // a reserved code point
        bool listed = false;
        for (const ExchangeStatus d : defined)
            if (d == s)
                listed = true;
        BOOST_REQUIRE_MESSAGE(listed, "a named status is not in the QA list");
    }
    for (int i = 0; i <= static_cast<int>(ConfigReason::IntegerPrecisionLoss); ++i) {
        const ConfigReason r = static_cast<ConfigReason>(i);
        if (std::string(config_reason_to_string(r)) == "invalid")
            continue; // a reserved code point
        ConfigReason back = ConfigReason::None;
        BOOST_REQUIRE_MESSAGE(config_reason_from_string(config_reason_to_string(r),
            back),
                              twr_int_to_text(i));
        BOOST_REQUIRE(back == r);
    }
}

BOOST_AUTO_TEST_CASE(native_rate_mirrors_uhd_backend_config)
{
    // The TWR header is UHD-free, so it re-implements the rule.  Prove the two
    // implementations agree, including at the tolerance edge.
    const double probes[] = { 0.0,
                              1.0,
                              200.0e6,
                              998.4e6,
                              491.52e6,
                              491520000.0,
                              491520001.0,
                              491520010.0,
                              737280000.0,
                              737280001.0,
                              737280010.0,
                              1.0e9,
                              -std::numeric_limits<double>::infinity(),
                              std::numeric_limits<double>::quiet_NaN() };
    for (double hz : probes) {
        BOOST_REQUIRE_EQUAL(is_allowed_native_rate(hz),
                            uhd_cfg::is_allowed_uhd_native_rate(hz));
    }
    // A +Inf sample rate used to be ACCEPTED by the mirrored UHD helper:
    // with a finite `requested`, scale became +Inf, |diff| became +Inf and
    // tol*scale became +Inf, so the comparison degenerated to
    // (+Inf <= +Inf) == true.  A device reporting +Inf would then have been
    // accepted as a matching 737.28 MS/s, defeating the "silent UHD rate
    // coercion is a hard prepare failure" contract.  The orchestrator fixed
    // uwb_uhd_backend_config.h:rate_matches_strict with an explicit
    // non-finite guard, so parity is now asserted rather than a divergence.
    BOOST_REQUIRE(!uhd_cfg::is_allowed_uhd_native_rate(
        std::numeric_limits<double>::infinity()));
    BOOST_REQUIRE(!uhd_cfg::is_allowed_uhd_native_rate(
        -std::numeric_limits<double>::infinity()));
    BOOST_REQUIRE(!uhd_cfg::is_allowed_uhd_native_rate(
        std::numeric_limits<double>::quiet_NaN()));
    BOOST_REQUIRE(!is_allowed_native_rate(std::numeric_limits<double>::infinity()));
    BOOST_REQUIRE(is_allowed_native_rate(737280000.0));
    BOOST_REQUIRE(is_allowed_native_rate(491520000.0));
    BOOST_REQUIRE(!is_allowed_native_rate(998.4e6)); // work rate != native rate
    BOOST_REQUIRE(!is_allowed_native_rate(64.0e6));  // mean PRF != native rate
}

BOOST_AUTO_TEST_CASE(code_index_and_psdu_mirror_radar_pdu_meta)
{
    for (int i = 0; i <= 16; ++i) {
        const uint8_t code = static_cast<uint8_t>(i);
        BOOST_REQUIRE_EQUAL(code_index_supported(code),
                            radar_meta::code_index_supported(static_cast<size_t>(i)));
    }
    BOOST_REQUIRE_EQUAL(kTwrMaxPsduBytes,
                        static_cast<uint16_t>(radar_meta::kMaxPsduBytes));
    BOOST_REQUIRE(code_index_supported(9));
    BOOST_REQUIRE(code_index_supported(12));
    BOOST_REQUIRE(!code_index_supported(8));
    BOOST_REQUIRE(!code_index_supported(13));
}

BOOST_AUTO_TEST_CASE(shipped_capabilities_are_the_measured_subset)
{
    const Capabilities& c = capabilities();
    // MEASURED (from existing code)
    BOOST_REQUIRE(c.native_rate_supported(737280000.0));
    BOOST_REQUIRE(c.native_rate_supported(491520000.0));
    BOOST_REQUIRE(!c.native_rate_supported(998.4e6));
    for (uint8_t code = 0; code < 20; ++code)
        BOOST_REQUIRE_EQUAL(c.code_index_supported(code), code_index_supported(code));
    BOOST_REQUIRE_EQUAL(c.max_psdu_bytes(), kTwrMaxPsduBytes);
    BOOST_REQUIRE_CLOSE(c.work_sample_rate_hz, kTwrWorkSampleRateHz, 1e-9);
    BOOST_REQUIRE_EQUAL(c.work_samples_per_symbol, kTwrWorkSamplesPerSymbol);
    BOOST_REQUIRE_CLOSE(c.mean_prf_hz, kTwrExistingMeanPrfHz, 1e-9);

    // MEASURED: populated from testdata/twr/phy_matrix_737280000.csv
    // (gr-uwb/lib/qa_uwb_twr_phy_matrix.cc).  64 SYNC x {9,10,11,12} x
    // {4z1,4z2,4z3,4z4,decawave,ieee} = 24 measured round-trip rows.
    // 2 measured SYNC lengths (16, 64) x 4 codes x 6 SFD modes = 48 rows.
    BOOST_REQUIRE_EQUAL(c.phy_matrix.size(), 48u);
    BOOST_REQUIRE_EQUAL(c.data_rates.size(), 1u);
    BOOST_REQUIRE_EQUAL(std::string(data_rate_to_string(c.data_rates.front())),
                        std::string("6p8m"));
    // 64 SYNC only: 16 decodes but its first-path ToA accuracy is
    // unverified; 128/256/512/2048 fail on the CFO-fit zero-phase leak AND
    // on PHR self-description; 32 advertises 64.  See the config header.
    // {16, 64}, both measured round trips.  See the header for the per-length
    // measured reasons; 16 additionally carries the ToA-accuracy caveat.
    BOOST_REQUIRE_EQUAL(c.sync_repetitions.size(), 2u);
    BOOST_REQUIRE_EQUAL(c.sync_repetitions.front(), 16);
    BOOST_REQUIRE_EQUAL(c.sync_repetitions.back(), 64);
    BOOST_REQUIRE(c.sync_repetitions_supported(16));
    BOOST_REQUIRE(c.sync_repetitions_supported(64));
    BOOST_REQUIRE(!c.sync_repetitions_supported(1));   // stage_cfo needs >=4
    BOOST_REQUIRE(!c.sync_repetitions_supported(4));   // CIR skip exceeds avail
    BOOST_REQUIRE(!c.sync_repetitions_supported(32));  // advertises 64
    BOOST_REQUIRE(!c.sync_repetitions_supported(128)); // CFO leak + PHR
    BOOST_REQUIRE(!c.sync_repetitions_supported(1024));// PHR-legal, CFO-biased
    BOOST_REQUIRE(!c.sync_repetitions_supported(2048));
    BOOST_REQUIRE_EQUAL(c.sfd_modes.size(), 6u);
    BOOST_REQUIRE(c.sfd_mode_supported(SfdMode::R4z2)); // phase-1 profile
    BOOST_REQUIRE(!c.pending_reason.empty());

    // Out of scope for phase 1.
    BOOST_REQUIRE(!c.sts_supported);
    BOOST_REQUIRE(c.sts_modes.empty());
    BOOST_REQUIRE(c.unsupported_sts_reason.find("phase 1") != std::string::npos);

    // By definition.
    BOOST_REQUIRE(c.ranging_bit_required);
    BOOST_REQUIRE_EQUAL(c.channels.size(), 1u);
    BOOST_REQUIRE_EQUAL(static_cast<int>(c.channels.front()), 5);
    BOOST_REQUIRE(c.prf_class_supported(PrfClass::Bprf64));
    BOOST_REQUIRE(!c.prf_class_supported(PrfClass::Hprf400));

    // The dump must be printable.  The measured matrix has landed, so it must
    // now report the MEASURED state and the measured row counts rather than
    // MEASURED-PENDING.  The only axes still pending are the ones with no
    // measurement at all (STS is out of scope, not pending).
    const std::string dump = c.to_string();
    BOOST_REQUIRE(dump.find("MEASURED-PENDING") == std::string::npos);
    BOOST_REQUIRE(dump.find("sts_supported=false") != std::string::npos);
    BOOST_REQUIRE(dump.find("phy_matrix") != std::string::npos);
}

BOOST_AUTO_TEST_CASE(channel_plan_table_is_ieee_and_channel5_is_6489p6)
{
    BOOST_REQUIRE_CLOSE(uwb_channel_center_frequency_hz(5), 6489.6e6, 1e-6);
    BOOST_REQUIRE_CLOSE(uwb_channel_center_frequency_hz(0), 4835.2e6, 1e-6);
    BOOST_REQUIRE_CLOSE(uwb_channel_center_frequency_hz(9), 6988.8e6, 1e-6);
    BOOST_REQUIRE(uwb_channel_center_frequency_hz(16) > 0.0);
    BOOST_REQUIRE_CLOSE(uwb_channel_center_frequency_hz(17), 0.0, 1e-12);
}

// ===========================================================================
// B. Valid configurations and the effective snapshot
// ===========================================================================

BOOST_AUTO_TEST_CASE(valid_minimal_ss_initiator)
{
    const TwrConfig c = minimal(Protocol::Ss, Role::Initiator);
    const ValidationReport r = validate(c, measured_caps());
    BOOST_REQUIRE_MESSAGE(r.ok(), r.to_string());
    BOOST_REQUIRE(r.violations.empty());
    BOOST_REQUIRE(r.first_status() == ExchangeStatus::Ok);
}

BOOST_AUTO_TEST_CASE(valid_minimal_ss_responder)
{
    const TwrConfig c = minimal(Protocol::Ss, Role::Responder);
    const ValidationReport r = validate(c, measured_caps());
    BOOST_REQUIRE_MESSAGE(r.ok(), r.to_string());
}

BOOST_AUTO_TEST_CASE(valid_minimal_ds_initiator)
{
    const TwrConfig c = minimal(Protocol::Ds, Role::Initiator);
    const ValidationReport r = validate(c, measured_caps());
    BOOST_REQUIRE_MESSAGE(r.ok(), r.to_string());
}

BOOST_AUTO_TEST_CASE(valid_minimal_ds_responder)
{
    const TwrConfig c = minimal(Protocol::Ds, Role::Responder);
    const ValidationReport r = validate(c, measured_caps());
    BOOST_REQUIRE_MESSAGE(r.ok(), r.to_string());
}

BOOST_AUTO_TEST_CASE(effective_config_snapshot_is_correct)
{
    const TwrConfig c = minimal(Protocol::Ds, Role::Initiator);
    const EffectiveConfig e = effective_config(c, measured_caps());
    BOOST_REQUIRE_MESSAGE(e.ok, e.validation.to_string());

    // versions (REQ-API-01)
    BOOST_REQUIRE_EQUAL(e.schema_version, capabilities().schema_version);
    BOOST_REQUIRE_EQUAL(e.profile_version, c.meta.profile_version);
    BOOST_REQUIRE_EQUAL(e.calibration_version, c.meta.calibration_version);
    BOOST_REQUIRE(!e.config_hash.empty());
    BOOST_REQUIRE(e.config_hash.find("fnv1a64:") == 0u);

    // requested vs effective
    BOOST_REQUIRE_EQUAL(e.requested.calibration.tx_link_delay.nanos(), 120);
    BOOST_REQUIRE_EQUAL(e.effective.calibration.tx_link_delay.nanos(), 120);
    BOOST_REQUIRE_EQUAL(config_hash(e.requested), config_hash(e.effective));
    // Nothing is invented: a validated config changes no field except the
    // recorded tick quantisation, which stays within the requested nanosecond.
    for (const auto& ch : e.changes) {
        if (!ch.adjusted())
            continue;
        BOOST_REQUIRE(ch.path == "timing.quantised_delay");
        BOOST_REQUIRE(ch.effective.find("quantised at") == std::string::npos);
    }

    // readback is preserved verbatim
    BOOST_REQUIRE(e.readback.present);
    BOOST_REQUIRE_CLOSE(e.readback.sample_rate_hz, kTwrNativeRateUc200Hz, 1e-9);
    BOOST_REQUIRE_EQUAL(e.readback.mpm_string, std::string("X410"));

    // quantised timing: 600 us at 737.28 MS/s
    BOOST_REQUIRE_CLOSE(e.tick_rate_hz, kTwrNativeRateUc200Hz, 1e-9);
    const int64_t expect_ticks = 442368; // round(600e-6 * 737.28e6)
    BOOST_REQUIRE_EQUAL(e.response_to_final_ticks, expect_ticks);
    BOOST_REQUIRE_EQUAL(e.poll_to_response_ticks, 0);
    BOOST_REQUIRE_EQUAL(e.poll_to_response_effective.nanos(), 0);
    // post-TX RX enable is a SEPARATE quantity and is quantised on its own.
    BOOST_REQUIRE_EQUAL(e.post_tx_rx_enable_ticks, 1475);
    BOOST_REQUIRE(e.poll_to_response_ticks != e.post_tx_rx_enable_ticks);

    // frame budget: the FROZEN 14-byte header + n*5 timestamps + footer 0
    // + the 2 B FCS the PHY appends.  Poll 16, Response 26, Final 31 on air.
    BOOST_REQUIRE_EQUAL(e.poll_bytes, 14 + 0 + 2);
    BOOST_REQUIRE_EQUAL(e.response_bytes, 14 + 2 * 5 + 2);
    BOOST_REQUIRE_EQUAL(e.final_bytes, 14 + 3 * 5 + 2);
    BOOST_REQUIRE_EQUAL(e.max_psdu_bytes, kTwrMaxPsduBytes);
    BOOST_REQUIRE_EQUAL(e.max_timestamp_count, 3u);
}

BOOST_AUTO_TEST_CASE(effective_config_records_quantisation_difference)
{
    TwrConfig c = minimal(Protocol::Ds, Role::Initiator);
    // 600001 ns is not an exact number of 737.28 MS/s device ticks.
    c.timing.response_to_final.value = Duration::from_nanos(600'002);
    const EffectiveConfig e = effective_config(c, measured_caps());
    BOOST_REQUIRE_MESSAGE(e.ok, e.validation.to_string());
    bool found = false;
    for (const auto& ch : e.changes) {
        if (ch.path != "timing.quantised_delay")
            continue;
        found = true;
        BOOST_REQUIRE(ch.adjusted());
        BOOST_REQUIRE(ch.requested == "600002ns");
        BOOST_REQUIRE(ch.effective != ch.requested);
        BOOST_REQUIRE(ch.note.find("quantised at") != std::string::npos);
        BOOST_REQUIRE(ch.note.find("ticks=442369") != std::string::npos);
    }
    BOOST_REQUIRE(found);
}

BOOST_AUTO_TEST_CASE(rejected_config_yields_no_effective_values)
{
    const TwrConfig c = minimal(Protocol::Ds, Role::Initiator);
    // Break it: unsupported STS.
    TwrConfig bad = c;
    bad.frame.sts_mode = StsMode::Sp128;
    bad.frame.sts_length_symbols = 128u;
    const EffectiveConfig e = effective_config(bad, measured_caps());
    BOOST_REQUIRE(!e.ok);
    BOOST_REQUIRE(!e.validation.ok());
    // The hash of the REQUESTED config is recorded so a failure is traceable.
    BOOST_REQUIRE(e.config_hash.find("fnv1a64:") == 0u);
    BOOST_REQUIRE_EQUAL(e.effective.meta.profile_version, std::string());
    BOOST_REQUIRE_EQUAL(static_cast<int>(e.effective.frame.sts_mode),
                        static_cast<int>(StsMode::Off));
    BOOST_REQUIRE_EQUAL(e.effective.frame.sts_length_symbols, 0);
    BOOST_REQUIRE(e.effective.meta.profile_version.empty());
    BOOST_REQUIRE(e.effective.calibration.calibration_id.empty());
    BOOST_REQUIRE_EQUAL(e.effective.timing.response_to_final.value.nanos(), 0);
    BOOST_REQUIRE(e.changes.empty());
}

BOOST_AUTO_TEST_CASE(config_hash_tracks_every_field)
{
    const TwrConfig a = minimal();
    TwrConfig b = minimal();
    BOOST_REQUIRE_EQUAL(config_hash(a), config_hash(b));
    b.session.pan_id = 0xcbff;
    BOOST_REQUIRE(config_hash(a) != config_hash(b));
    const std::vector<FieldChange> d = diff_fields(a, b);
    BOOST_REQUIRE_EQUAL(d.size(), 1u);
    BOOST_REQUIRE_EQUAL(d.front().path, std::string("session.pan_id"));
    BOOST_REQUIRE(d.front().adjusted());
    // The canonical text covers every group, so a field-list omission would
    // show up as a missing path rather than a silent hash collision.
    const std::string canon = canonical_text(flatten_fields(a));
    BOOST_REQUIRE(canon.find("session.protocol=ss\n") != std::string::npos);
    BOOST_REQUIRE(canon.find("phy.channel=5\n") != std::string::npos);
    BOOST_REQUIRE(canon.find("timing.poll_to_response.ns=") != std::string::npos);
    BOOST_REQUIRE(canon.find("timeouts.rx_timeout.reference=rx_enable\n") != std::string::npos);
    BOOST_REQUIRE(canon.find("diagnostics.io_on_realtime_thread=false\n") != std::string::npos);
    BOOST_REQUIRE(canon.find("calibration.record.calibration_id=") != std::string::npos);
}

// ===========================================================================
// C. DEFAULT DENY: the capability whitelist
// ===========================================================================

BOOST_AUTO_TEST_CASE(shipped_whitelist_is_measured_not_default_deny_forever)
{
    // The measured matrix has landed (testdata/twr/phy_matrix_737280000.csv),
    // so the shipped whitelist is no longer empty and the minimal 6.81 Mb/s /
    // 64 SYNC / 4z2 / ch5 profile must now VALIDATE.  The default-deny
    // property is preserved by rejecting everything OUTSIDE the measured set.
    const TwrConfig c = minimal();
    const ValidationReport r = validate(c, capabilities());
    BOOST_REQUIRE_MESSAGE(r.ok(), r.to_string());
    // The accepted joint lookup is the measured row, and it is all-or-nothing.
    BOOST_REQUIRE(capabilities()
                      .lookup_phy(kTwrNativeRateUc200Hz, 9, 64, SfdMode::R4z2, 127,
                          true)
                      .allowed);
    BOOST_REQUIRE(!capabilities()
                       .lookup_phy(kTwrNativeRateCg400Hz, 9, 64, SfdMode::R4z2, 127,
                           true)
                       .allowed);
    // Every unmeasured axis still produces a specific, machine-readable
    // rejection naming the measured matrix -- nothing is defaulted to the
    // radar/QM35 behaviour (REQ-SCOPE-01).
    struct Probe {
        void (*mutate)(TwrConfig&);
        const char* field;
    };
    const Probe probes[] = {
        { [](TwrConfig& x) { x.phy.data_rate = DataRate::R850k; }, "phy.data_rate" },
        { [](TwrConfig& x) { x.phy.preamble_symbols = 128; }, "phy.preamble_symbols" },
        // 16 SYNC is measured-supported and must NOT appear here; 32 is the
        // shortest length that is still rejected (PHR advertises 64).
        { [](TwrConfig& x) { x.phy.preamble_symbols = 32; }, "phy.preamble_symbols" },
        // All six enum SFD modes are measured, so the remaining capability
        // probe is an out-of-enum value, which must be rejected rather than
        // coerced to the nearest known mode.
        { [](TwrConfig& x) {
              x.frame.sfd_mode = static_cast<SfdMode>(77);
              x.frame.sfd_symbols = 0;
          },
          "frame.sfd_mode" },
        { [](TwrConfig& x) { x.phy.channel = 9; }, "phy.channel" },
        // STS is out of scope for phase 1 (explicit user decision, REQ-SCOPE-04).
        { [](TwrConfig& x) { x.frame.sts_mode = StsMode::Sp128; }, "frame.sts_mode" },
    };
    for (const Probe& pr : probes) {
        TwrConfig x = minimal();
        pr.mutate(x);
        const ValidationReport v = validate(x, capabilities());
        require_machine_readable(v);
        // Every rejection must point at the whitelist that refused it -- the
        // measured PHY matrix for the data-rate / SYNC / SFD axes, and the
        // capability whitelist for channel.  What matters is that the message
        // is actionable, not that every axis uses one wording.
        bool named = false;
        for (const auto& vi : v.violations)
            if ((vi.reason == ConfigReason::Unsupported ||
                 vi.reason == ConfigReason::OutOfScope) &&
                (vi.message.find("whitelist") != std::string::npos ||
                 vi.message.find("phy_matrix") != std::string::npos ||
                 vi.message.find("phase 1") != std::string::npos))
                named = true;
        BOOST_REQUIRE_MESSAGE(named, std::string("rejection for ") + pr.field
                                         + " did not name the refusing whitelist: "
                                         + v.to_string());
    }
}

BOOST_AUTO_TEST_CASE(shipped_whitelist_accepts_measured_data_rate_only)
{
    // 6.81 Mb/s is the ONLY measured PAYLOAD rate: uwb_hrp_mod_core.h hardcodes
    // the payload geometry (64 chips/burst, 64 chips/symbol, scrambler offset
    // 1344) and uwb_demod_core.h:stage_payload_fcs always decodes the 6.81
    // geometry.  The PHR is a SEPARATE axis at its own 0.85 Mb/s rate, which
    // is why the fixture sets phr_rate independently.
    {
        TwrConfig c = minimal();
        c.phy.data_rate = DataRate::R6p8M;
        c.phy.phr_rate = PhrRate::Standard850k;
        BOOST_REQUIRE_MESSAGE(validate(c, capabilities()).ok(),
                              "measured 6.81 Mb/s payload was rejected");
    }
    const char* unmeasured[] = { "850k", "27m", "7p8m", "27p2m", "6p8m_hprf" };
    for (const char* n : unmeasured) {
        DataRate r = DataRate::R850k;
        BOOST_REQUIRE(data_rate_from_string(n, r));
        TwrConfig c = minimal();
        c.phy.data_rate = r;
        const ValidationReport v = validate(c, capabilities());
        require_machine_readable(v);
        BOOST_REQUIRE_MESSAGE(has(v, "phy.data_rate", ConfigReason::Unsupported),
                              std::string("unmeasured data rate ") + n + " was accepted");
    }
}

BOOST_AUTO_TEST_CASE(shipped_whitelist_accepts_measured_sync_repetition_count)
{
    // 64 SYNC is the measured phase-1 profile and 16 SYNC is measured-supported
    // (its tail covers the whole preamble, so there is no zero-phase leak, and
    // encode_phr19 advertises the legal duration 16).  Both must be ACCEPTED.
    for (uint16_t reps : { 16, 64 }) {
        TwrConfig c = minimal();
        c.phy.preamble_symbols = reps;
        BOOST_REQUIRE_MESSAGE(validate(c, capabilities()).ok(),
                              std::string("measured ") + twr_int_to_text(reps) +
                                  " SYNC was rejected");
    }
    // 16 SYNC is decode-verified but its first-path / ToA accuracy is NOT
    // measured, and ranging accuracy depends on exactly that.  The caveat is
    // queryable so M2 cannot use 16 as a ranging profile by accident.
    BOOST_REQUIRE(twr_sync_reps_needs_toa_validation(16));
    BOOST_REQUIRE(!twr_sync_reps_needs_toa_validation(64));
    // Still rejected, each for a MEASURED reason recorded in the config
    // header: 1/2 need >= 4 CFO peaks, 4/8 exceed the CIR skip, 32/128/256/
    // 512/2048 fail on PHR self-description and the CFO-fit zero-phase leak,
    // and 1024 is PHR-legal but CFO-biased.
    for (uint16_t reps : { 1, 2, 4, 8, 32, 128, 256, 512, 1024, 2048 }) {
        TwrConfig c = minimal();
        c.phy.preamble_symbols = reps;
        const ValidationReport v = validate(c, capabilities());
        require_machine_readable(v);
        BOOST_REQUIRE_MESSAGE(has(v, "phy.preamble_symbols", ConfigReason::Unsupported),
                              twr_int_to_text(reps) + " was accepted");
    }
}

BOOST_AUTO_TEST_CASE(shipped_whitelist_accepts_every_measured_sfd_mode)
{
    // All six SFD modes round-tripped and must now be accepted.  4z2 is the
    // phase-1 common profile; 4z1 (4 symbols) and 4z4 (32 symbols) are NOT
    // IEEE 802.15.4a HRP lengths and must be re-confirmed against the target
    // module before M5.
    for (int i = 0; i <= static_cast<int>(SfdMode::Ieee8); ++i) {
        const SfdMode m = static_cast<SfdMode>(i);
        TwrConfig c = minimal();
        c.frame.sfd_mode = m;
        c.frame.sfd_symbols = sfd_mode_symbols(m);
        BOOST_REQUIRE_MESSAGE(validate(c, capabilities()).ok(),
                              std::string("measured sfd mode ") + sfd_mode_to_string(m)
                                  + " was rejected");
    }
    // Values outside the enum are still rejected rather than coerced.
    for (int i = static_cast<int>(SfdMode::Ieee8) + 1; i < 256; i += 37) {
        TwrConfig c = minimal();
        c.frame.sfd_mode = static_cast<SfdMode>(i);
        const ValidationReport v = validate(c, capabilities());
        require_machine_readable(v);
        BOOST_REQUIRE(has(v, "frame.sfd_mode", ConfigReason::Unsupported));
    }
}

BOOST_AUTO_TEST_CASE(measured_whitelist_accepts_only_listed_combination)
{
    const Capabilities caps = measured_caps();
    // The joint lookup is all-or-nothing.
    BOOST_REQUIRE(caps.lookup_phy(kTwrNativeRateUc200Hz, 9, 128, SfdMode::R4z2, 127,
        true)
                      .allowed);
    BOOST_REQUIRE(!caps.lookup_phy(kTwrNativeRateCg400Hz, 9, 128, SfdMode::R4z2, 127,
        true)
                       .allowed);
    BOOST_REQUIRE(!caps.lookup_phy(kTwrNativeRateUc200Hz, 10, 128, SfdMode::R4z2, 127,
        true)
                       .allowed);
    // The shipped whitelist now also contains all 24 measured
    // 64-SYNC x {9,10,11,12} x 6-SFD rows, so 64 SYNC at 4z2 is allowed for
    // every code.  Use combinations that are genuinely absent from BOTH the
    // synthetic row and the shipped matrix: the CG400 native rate (the matrix
    // was measured at 737.28 only) and a non-measured SFD.
    BOOST_REQUIRE(caps.lookup_phy(kTwrNativeRateUc200Hz, 9, 64, SfdMode::R4z2, 127,
        true).allowed);
    BOOST_REQUIRE(caps.lookup_phy(kTwrNativeRateUc200Hz, 12, 64, SfdMode::Ieee8, 127,
        true).allowed);
    BOOST_REQUIRE(!caps.lookup_phy(kTwrNativeRateCg400Hz, 9, 64, SfdMode::R4z2, 127,
        true).allowed);
    BOOST_REQUIRE(
        !caps.lookup_phy(kTwrNativeRateUc200Hz, 9, 128, SfdMode::R4z3, 127,
            true).allowed);
    BOOST_REQUIRE(!caps.lookup_phy(kTwrNativeRateUc200Hz, 9, 32, SfdMode::R4z2, 127,
        true).allowed);
    BOOST_REQUIRE(!caps.lookup_phy(kTwrNativeRateUc200Hz, 9, 128, SfdMode::R4z2, 127,
        false)
                       .allowed);
    // A preamble length that is inside the code range but absent from every
    // measured matrix row is still rejected.  32 SYNC is refused by TWO gates
    // for two different reasons, and the report keeps them apart: the
    // PreambleLength enumeration (this decoder's CFO fit and PHR
    // self-description) and the capability whitelist (the measurement).
    TwrConfig c = minimal();
    c.phy.tx_preamble_code = 10u;
    c.phy.rx_preamble_code = 10u;
    c.phy.preamble_symbols = 32;
    const ValidationReport v = validate(c, caps);
    require_machine_readable(v);
    BOOST_REQUIRE(has(v, "phy.preamble_symbols", ConfigReason::Unsupported));
    // Both gates are visible: the software limit and the measurement.
    bool named_implementation = false;
    bool named_whitelist = false;
    for (const auto& vi : v.violations) {
        if (vi.field != "phy.preamble_symbols" || vi.reason != ConfigReason::Unsupported)
            continue;
        if (vi.message == preamble_length_unsupported_reason(32))
            named_implementation = true;
        if (vi.message.find("capability whitelist") != std::string::npos ||
            vi.message.find("no measured row") != std::string::npos)
            named_whitelist = true;
    }
    BOOST_REQUIRE_MESSAGE(named_implementation,
                          "the implementation limit was not reported: " + v.to_string());
    BOOST_REQUIRE_MESSAGE(named_whitelist,
                          "the measurement gate was not reported: " + v.to_string());
}

BOOST_AUTO_TEST_CASE(sts_is_rejected_for_every_non_off_mode)
{
    for (int i = 1; i <= static_cast<int>(StsMode::Sp1024); ++i) {
        const StsMode m = static_cast<StsMode>(i);
        TwrConfig c = minimal();
        c.frame.sts_mode = m;
        c.frame.sts_length_symbols = 128u;
        const ValidationReport v = validate(c, measured_caps());
        require_machine_readable(v);
        const ConfigViolation* viol = v.find("frame.sts_mode",
            ConfigReason::OutOfScope);
        BOOST_REQUIRE_MESSAGE(viol != nullptr,
                              std::string("STS mode ") + sts_mode_to_string(m) + " was accepted");
        BOOST_REQUIRE(viol->status == ExchangeStatus::Unsupported);
        BOOST_REQUIRE(viol->message.find("phase 1") != std::string::npos);
        BOOST_REQUIRE(viol->requirement == "REQ-SCOPE-04");
        // The length is a separate conflict, not silently ignored either.
        BOOST_REQUIRE(has(v, "frame.sts_length_symbols", ConfigReason::FieldConflict));
    }
    // Off is fine.
    TwrConfig ok = minimal();
    ok.frame.sts_mode = StsMode::Off;
    BOOST_REQUIRE(validate(ok, measured_caps()).ok());
}

BOOST_AUTO_TEST_CASE(ranging_bit_false_is_rejected)
{
    TwrConfig c = minimal();
    c.frame.ranging_bit = false;
    const ValidationReport v = validate(c, measured_caps());
    require_machine_readable(v);
    const ConfigViolation* viol = v.find("frame.ranging_bit",
        ConfigReason::FieldConflict);
    BOOST_REQUIRE(viol != nullptr);
    BOOST_REQUIRE(viol->requirement == "REQ-PROTO-05");
    // The joint capability lookup refuses it too, since no measured row has
    // ranging = false.
    BOOST_REQUIRE(has(v, "phy.preamble_symbols", ConfigReason::Unsupported));
}

BOOST_AUTO_TEST_CASE(unsupported_channel_is_rejected_not_defaulted)
{
    for (uint8_t ch : { 0, 4, 6, 8, 9, 16 }) {
        TwrConfig c = minimal();
        c.phy.channel = ch;
        c.phy.center_frequency_hz = uwb_channel_center_frequency_hz(ch);
        c.calibration.record.channel = ch;
        const ValidationReport v = validate(c, measured_caps());
        require_machine_readable(v);
        BOOST_REQUIRE_MESSAGE(has(v, "phy.channel", ConfigReason::Unsupported),
                              "channel " + twr_int_to_text(ch) + " was accepted");
    }
}

BOOST_AUTO_TEST_CASE(prf_class_hprf400_is_rejected)
{
    TwrConfig c = minimal();
    c.phy.prf_class = PrfClass::Hprf400;
    const ValidationReport v = validate(c, measured_caps());
    require_machine_readable(v);
    BOOST_REQUIRE(has(v, "phy.prf_class", ConfigReason::Unsupported));
}

BOOST_AUTO_TEST_CASE(schema_version_mismatch_is_rejected)
{
    TwrConfig c = minimal();
    c.meta.schema_version = "twr-config/99";
    const ValidationReport v = validate(c, measured_caps());
    require_machine_readable(v);
    BOOST_REQUIRE(has(v, "meta.schema_version", ConfigReason::Unsupported));
}

// ===========================================================================
// D. NaN / Inf in every float field
// ===========================================================================

BOOST_AUTO_TEST_CASE(non_finite_rejected_in_every_float_field)
{
    const double bad[] = { std::numeric_limits<double>::quiet_NaN(),
                           std::numeric_limits<double>::infinity(),
                           -std::numeric_limits<double>::infinity() };
    // Each entry: a name plus a mutator.
    struct Mut {
        const char* name;
        void (*apply)(TwrConfig&, double);
    };
    const Mut muts[] = {
        { "phy.center_frequency_hz", [](TwrConfig& c,
            double v) { c.phy.center_frequency_hz = v; } },
        { "radio.native_sample_rate_hz",
          [](TwrConfig& c, double v) { c.radio.native_sample_rate_hz = v; } },
        { "calibration.native_sample_rate_hz",
          [](TwrConfig& c, double v) { c.calibration.native_sample_rate_hz = v; } },
        { "tx.gain_db", [](TwrConfig& c, double v) { c.tx.gain_db = Opt<double>(v); } },
        { "tx.iq_amplitude", [](TwrConfig& c,
            double v) { c.tx.iq_amplitude = Opt<double>(v); } },
        { "tx.calibrated_tx_power_dbm",
          [](TwrConfig& c,
              double v) { c.tx.calibrated_tx_power_dbm = Opt<double>(v); } },
        { "rx.gain_db", [](TwrConfig& c, double v) { c.rx.gain_db = Opt<double>(v); } },
        { "rx.bandwidth_hz", [](TwrConfig& c,
            double v) { c.rx.bandwidth_hz = Opt<double>(v); } },
        { "rx.detection_threshold",
          [](TwrConfig& c, double v) { c.rx.detection_threshold = v; } },
        { "rx.correlation_threshold",
          [](TwrConfig& c, double v) { c.rx.correlation_threshold = v; } },
        { "rx.first_path_threshold",
          [](TwrConfig& c, double v) { c.rx.first_path_threshold = v; } },
        { "rx.vendor_pac_applied_step",
          [](TwrConfig& c,
              double v) { c.rx.vendor_pac_applied_step = Opt<double>(v); } },
        { "calibration.record.gain_db",
          [](TwrConfig& c,
              double v) { c.calibration.record.gain_db = Opt<double>(v); } },
        { "timing.poll_to_response.quantisation_hz",
          [](TwrConfig& c,
              double v) { c.timing.poll_to_response.required_quantisation_hz = v; } },
        { "timeouts.rx_timeout.quantisation_hz",
          [](TwrConfig& c,
              double v) { c.timeouts.rx_timeout.required_quantisation_hz = v; } },
        { "timeouts.exchange_timeout.quantisation_hz",
          [](TwrConfig& c,
              double v) { c.timeouts.exchange_timeout.required_quantisation_hz = v; } },
        { "diagnostics.stats_cadence.quantisation_hz",
          [](TwrConfig& c,
              double v) { c.diagnostics.stats_cadence.required_quantisation_hz = v; } },
        { "radio.readback.sample_rate_hz",
          [](TwrConfig& c, double v) { c.radio.readback.sample_rate_hz = v; } },
    };
    for (const auto& m : muts) {
        for (double v : bad) {
            TwrConfig c = minimal();
            m.apply(c, v);
            const ValidationReport r = validate(c, measured_caps());
            require_machine_readable(r);
            bool not_finite = false;
            for (const auto& viol : r.violations)
                if (viol.reason == ConfigReason::NotFinite)
                    not_finite = true;
            BOOST_REQUIRE_MESSAGE(not_finite,
                                  std::string(m.name) + " accepted a non-finite value");
        }
    }
}

BOOST_AUTO_TEST_CASE(non_finite_cannot_be_exported_to_json)
{
    TwrConfig c = minimal();
    c.rx.detection_threshold = std::numeric_limits<double>::quiet_NaN();
    std::string out;
    std::string err;
    BOOST_REQUIRE(!to_json_string(c, out, err));
    BOOST_REQUIRE(!err.empty());
    BOOST_REQUIRE(err.find("not finite") != std::string::npos);
    // And it never emits a bare `NaN` token.
    BOOST_REQUIRE(out.find("NaN") == std::string::npos);
    BOOST_REQUIRE(out.find("nan") == std::string::npos);
}

BOOST_AUTO_TEST_CASE(threshold_ordering_is_checked)
{
    TwrConfig c = minimal();
    c.rx.correlation_threshold = 0.1; // looser than the coarse gate
    const ValidationReport v = validate(c, measured_caps());
    require_machine_readable(v);
    BOOST_REQUIRE(has(v, "rx.correlation_threshold", ConfigReason::FieldConflict));
    // A threshold above 1 is a range error, not a silent clamp.
    TwrConfig d = minimal();
    d.rx.first_path_threshold = 1.5;
    const ValidationReport w = validate(d, measured_caps());
    require_machine_readable(w);
    BOOST_REQUIRE(has(w, "rx.first_path_threshold", ConfigReason::OutOfRange));
}

// ===========================================================================
// E. Indices, ranges, conflicts
// ===========================================================================

BOOST_AUTO_TEST_CASE(channel_versus_center_frequency_conflict)
{
    TwrConfig c = minimal();
    c.phy.center_frequency_hz = 6489.6e6 + 5000.0; // 5 kHz off
    const ValidationReport v = validate(c, measured_caps());
    require_machine_readable(v);
    BOOST_REQUIRE(has(v, "phy.center_frequency_hz",
        ConfigReason::ChannelFrequencyMismatch));
    BOOST_REQUIRE(has(v, "radio.readback.center_freq_hz",
                      ConfigReason::ChannelFrequencyMismatch));
    // A different channel's frequency for the same channel number.
    TwrConfig d = minimal();
    d.phy.center_frequency_hz = uwb_channel_center_frequency_hz(9);
    const ValidationReport w = validate(d, measured_caps());
    require_machine_readable(w);
    BOOST_REQUIRE(has(w, "phy.center_frequency_hz",
        ConfigReason::ChannelFrequencyMismatch));
    // Within tolerance: accepted.
    TwrConfig e = minimal();
    e.phy.center_frequency_hz = 6489.6e6 + 10.0;
    BOOST_REQUIRE(validate(e, measured_caps()).ok());
}

BOOST_AUTO_TEST_CASE(channel_index_out_of_range)
{
    TwrConfig c = minimal();
    c.phy.channel = 17u;
    const ValidationReport v = validate(c, measured_caps());
    require_machine_readable(v);
    BOOST_REQUIRE(has(v, "phy.channel", ConfigReason::IndexOutOfRange));
    c.phy.channel = 200u;
    BOOST_REQUIRE(has(validate(c, measured_caps()), "phy.channel",
                      ConfigReason::IndexOutOfRange));
}

BOOST_AUTO_TEST_CASE(code_index_outside_9_to_12)
{
    for (uint8_t code : { 1u, 8u, 13u, 32u, 255u }) {
        TwrConfig c = minimal();
        c.phy.tx_preamble_code = code;
        c.phy.rx_preamble_code = code;
        const ValidationReport v = validate(c, measured_caps());
        require_machine_readable(v);
        BOOST_REQUIRE_MESSAGE(has(v, "phy.tx_preamble_code", ConfigReason::Unsupported),
                              "code " + twr_int_to_text(code) + " was accepted");
        BOOST_REQUIRE(has(v, "phy.rx_preamble_code", ConfigReason::Unsupported));
    }
    // 0 is "not stated", which is a missing value, not an unsupported index.
    TwrConfig zero = minimal();
    zero.phy.tx_preamble_code = 0u;
    zero.phy.rx_preamble_code = 0u;
    const ValidationReport z = validate(zero, measured_caps());
    require_machine_readable(z);
    BOOST_REQUIRE(has(z, "phy.tx_preamble_code", ConfigReason::ZeroValue));
    // TX/RX codes must agree: one profile uses one code.
    TwrConfig d = minimal();
    d.phy.tx_preamble_code = 9u;
    d.phy.rx_preamble_code = 10u;
    const ValidationReport w = validate(d, measured_caps());
    require_machine_readable(w);
    BOOST_REQUIRE(has(w, "phy.rx_preamble_code", ConfigReason::FieldConflict));
}

BOOST_AUTO_TEST_CASE(native_rate_other_than_737p28_or_491p52)
{
    const double rates[] = { 1.0e6, 64.0e6, 100.0e6, 500.0e6, 998.4e6, 1.0e9, 2.0e9 };
    for (double r : rates) {
        TwrConfig c = minimal();
        c.radio.native_sample_rate_hz = r;
        const ValidationReport v = validate(c, measured_caps());
        require_machine_readable(v);
        BOOST_REQUIRE_MESSAGE(has(v, "radio.native_sample_rate_hz",
            ConfigReason::Unsupported),
                              twr_double_to_text(r) + " was accepted");
    }
    {
        TwrConfig c = minimal();
        BOOST_REQUIRE_MESSAGE(validate(c, measured_caps()).ok(),
            "737.28 MS/s was rejected");
    }
    // 491.52 MS/s IS a supported native rate, but the only measured PHY row
    // is for 737.28 MS/s, so the combination is still refused -- and the
    // refusal says so instead of quietly re-running the other rate.
    {
        TwrConfig c = minimal();
        c.radio.native_sample_rate_hz = kTwrNativeRateCg400Hz;
        c.radio.readback.sample_rate_hz = kTwrNativeRateCg400Hz;
        c.radio.peers.front().native_sample_rate_hz = kTwrNativeRateCg400Hz;
        c.calibration.native_sample_rate_hz = kTwrNativeRateCg400Hz;
        c.calibration.record.native_sample_rate_hz = kTwrNativeRateCg400Hz;
        const ValidationReport v = validate(c, measured_caps());
        require_machine_readable(v);
        BOOST_REQUIRE(!v.has("radio.native_sample_rate_hz", ConfigReason::Unsupported));
        const ConfigViolation* viol = v.find("phy.preamble_symbols",
            ConfigReason::Unsupported);
        BOOST_REQUIRE(viol != nullptr);
        BOOST_REQUIRE(viol->message.find("native_rate=491520000") != std::string::npos);
    }
}

BOOST_AUTO_TEST_CASE(psdu_over_127_bytes)
{
    TwrConfig c = minimal();
    c.frame.mac_psdu_bytes = 128u;
    const ValidationReport v = validate(c, measured_caps());
    require_machine_readable(v);
    BOOST_REQUIRE(has(v, "frame.mac_psdu_bytes", ConfigReason::FrameLengthOverflow));
    BOOST_REQUIRE(has(v, "phy.preamble_symbols", ConfigReason::Unsupported));

    c.frame.mac_psdu_bytes = 1000u;
    const ValidationReport w = validate(c, measured_caps());
    require_machine_readable(w);
    BOOST_REQUIRE(has(w, "frame.mac_psdu_bytes", ConfigReason::FrameLengthOverflow));

    // 127 is exactly the maximum and is fine.
    TwrConfig ok = minimal();
    ok.frame.mac_psdu_bytes = 127u;
    BOOST_REQUIRE(validate(ok, measured_caps()).ok());
}

BOOST_AUTO_TEST_CASE(frame_length_for_poll_response_final_must_fit)
{
    // With the FROZEN 14-byte header: Poll = 14, Response = 14 + 2*5 = 24,
    // Final = 14 + 3*5 = 29 MAC bytes.  Negotiating 24 therefore fits Poll
    // and Response exactly and overflows only Final.  (The 7-byte header that
    // stood here before made the old 23-byte probe ambiguous.)
    TwrConfig tight = minimal();
    tight.frame.mac_psdu_bytes = 24u;
    const ValidationReport v = validate(tight, measured_caps());
    require_machine_readable(v);
    BOOST_REQUIRE(has(v, "frame.mac_psdu_bytes[final]",
        ConfigReason::FrameLengthOverflow));
    BOOST_REQUIRE(!v.find("frame.mac_psdu_bytes[poll]",
        ConfigReason::FrameLengthOverflow));
    BOOST_REQUIRE(!v.find("frame.mac_psdu_bytes[response]",
        ConfigReason::FrameLengthOverflow));
    const std::string msg =
        v.find("frame.mac_psdu_bytes[final]",
            ConfigReason::FrameLengthOverflow)->message;
    BOOST_REQUIRE(msg.find("29") != std::string::npos);
    BOOST_REQUIRE(msg.find("24") == std::string::npos ||
                  msg.find("only 24") != std::string::npos);

    // The Poll alone is 9 bytes; below that even the Poll does not fit.
    TwrConfig too_small = minimal();
    too_small.frame.mac_psdu_bytes = 8u;
    const ValidationReport w = validate(too_small, measured_caps());
    require_machine_readable(w);
    BOOST_REQUIRE(has(w, "frame.mac_psdu_bytes[poll]",
        ConfigReason::FrameLengthOverflow));
    BOOST_REQUIRE(has(w, "frame.mac_psdu_bytes[response]",
        ConfigReason::FrameLengthOverflow));
    BOOST_REQUIRE(has(w, "frame.mac_psdu_bytes[final]",
        ConfigReason::FrameLengthOverflow));

    // 29 exactly fits all three: it is the frozen Final MAC PSDU
    // (14 + 3*5).  The earlier value of 24 only fit because the header was
    // still the 7-byte placeholder.
    TwrConfig exact = minimal();
    exact.frame.mac_psdu_bytes = 29u;
    BOOST_REQUIRE_MESSAGE(validate(exact, measured_caps()).ok(),
                          "negotiating the exact Final size 29 was rejected: " +
                              validate(exact, measured_caps()).to_string());
}

BOOST_AUTO_TEST_CASE(frame_psdu_bytes_helper_respects_single_fcs_layer)
{
    // The helper is a DELEGATION to the codec now, so it is asserted against
    // the codec and never against a private 7-byte geometry (M0 review R3).
    FrameProfileGeometry auth;
    FrameProfile profile;
    BOOST_REQUIRE(frame_geometry_for(FrameProfileId::TwrV1, auth));
    BOOST_REQUIRE(frame_profile_for(FrameProfileId::TwrV1, profile));

    // MAC appends the FCS: it is inside the MAC PSDU, so the MAC-appends
    // variant of the authority is the one to compare against.
    const FrameProfileGeometry mac_auth = frame_geometry_mac_appends_fcs();
    BOOST_REQUIRE_EQUAL(frame_psdu_bytes(FrameType::Poll, FcsAppender::MacLayer),
        mac_auth.mac_payload_bytes(FrameType::Poll, profile));
    BOOST_REQUIRE_EQUAL(frame_psdu_bytes(FrameType::Response, FcsAppender::MacLayer),
        mac_auth.mac_payload_bytes(FrameType::Response, profile));
    BOOST_REQUIRE_EQUAL(frame_psdu_bytes(FrameType::Final, FcsAppender::MacLayer),
        mac_auth.mac_payload_bytes(FrameType::Final, profile));
    // PHY appends it: the MAC PSDU stops before the CRC.
    BOOST_REQUIRE_EQUAL(frame_psdu_bytes(FrameType::Poll, FcsAppender::PhyLayer),
        auth.mac_payload_bytes(FrameType::Poll, profile));
    BOOST_REQUIRE_EQUAL(frame_psdu_bytes(FrameType::Final, FcsAppender::PhyLayer),
        auth.mac_payload_bytes(FrameType::Final, profile));
    // The MAC-appends form is exactly the on-air length (14/24/29 + 2 FCS);
    // the PHY-appends form is the on-air length minus the FCS.
    BOOST_REQUIRE_EQUAL(frame_psdu_bytes(FrameType::Final, FcsAppender::MacLayer), 31u);
    BOOST_REQUIRE_EQUAL(frame_psdu_bytes(FrameType::Final, FcsAppender::PhyLayer), 29u);
    // Either way the ON-AIR length is the same, which is the property the
    // frame-duration budget depends on.
    BOOST_REQUIRE_EQUAL(auth.on_air_bytes(FrameType::Final, profile), 31u);
    BOOST_REQUIRE_EQUAL(mac_auth.on_air_bytes(FrameType::Final, profile), 31u);

    // The retained three-argument form ignores its geometry argument, on
    // purpose: a claim cannot decide a length.
    FrameGeometry wrong;
    wrong.mac_header_bytes = 7;
    wrong.timestamp_bytes = 5;
    wrong.mac_fcs_bytes = 2;
    wrong.phr_bytes = 12;
    BOOST_REQUIRE_EQUAL(frame_psdu_bytes(wrong, FrameType::Poll, FcsAppender::PhyLayer),
        frame_psdu_bytes(FrameType::Poll, FcsAppender::PhyLayer));
}

BOOST_AUTO_TEST_CASE(exactly_one_layer_appends_the_fcs)
{
    // Declared "MAC includes FCS" while the PHY is the appender.
    TwrConfig c = minimal();
    c.frame.fcs_append = FcsAppender::PhyLayer;
    c.frame.mac_psdu_includes_fcs = true;
    const ValidationReport v = validate(c, measured_caps());
    require_machine_readable(v);
    BOOST_REQUIRE(has(v, "frame.mac_psdu_includes_fcs", ConfigReason::FieldConflict));

    // No layer at all.
    TwrConfig d = minimal();
    d.frame.fcs_append = FcsAppender::None;
    const ValidationReport w = validate(d, measured_caps());
    require_machine_readable(w);
    BOOST_REQUIRE(has(w, "frame.fcs_append", ConfigReason::FieldConflict));

    // MAC appends it, but the geometry still reserves none: the claim is
    // checked against the MAC-appends variant of the authority, which is the
    // layout this config just named.
    TwrConfig e = minimal();
    e.frame.fcs_append = FcsAppender::MacLayer;
    e.frame.mac_psdu_includes_fcs = true;
    const ValidationReport x = validate(e, measured_caps());
    require_machine_readable(x);
    BOOST_REQUIRE(has(x, "frame.geometry.mac_fcs_bytes", ConfigReason::FieldConflict));

    // Self-consistent as a description: the MAC appends, the claim reserves the
    // 2 FCS bytes and the negotiated mac_psdu_bytes includes them (29 + 2 =
    // 31).  It is still REJECTED, because this codec cannot ENCODE that form:
    // frame_geometry_mac_appends_fcs().executable() is false.  M0 accepted it,
    // and a config that validates but cannot be encoded is exactly the R3
    // defect one layer down.
    TwrConfig described = minimal();
    described.frame.fcs_append = FcsAppender::MacLayer;
    described.frame.mac_psdu_includes_fcs = true;
    described.frame.geometry.mac_fcs_bytes = 2u;
    described.frame.mac_psdu_bytes = 31u;
    const ValidationReport y = validate(described, measured_caps());
    require_machine_readable(y);
    const ConfigViolation* viol = y.first_for_field("frame.fcs_append");
    BOOST_REQUIRE_MESSAGE(viol != nullptr,
                          "a MAC-appends-FCS profile this codec cannot encode was accepted: " +
                              y.to_string());
    BOOST_REQUIRE(viol->reason == ConfigReason::Unsupported);
    BOOST_REQUIRE(viol->status == ExchangeStatus::Unsupported);
    BOOST_REQUIRE(viol->message.find("executable") != std::string::npos);
    // The geometry itself is NOT the thing being refused: the claim now agrees
    // with the MAC-appends authority.
    BOOST_REQUIRE(!y.has_field("frame.geometry.mac_fcs_bytes"));
    // And the on-air length is unchanged by who appends the FCS.
    FrameProfileGeometry auth;
    FrameProfile profile;
    BOOST_REQUIRE(frame_geometry_for(FrameProfileId::TwrV1, auth));
    BOOST_REQUIRE(frame_profile_for(FrameProfileId::TwrV1, profile));
    const FrameProfileGeometry mac_auth = frame_geometry_mac_appends_fcs();
    BOOST_REQUIRE_EQUAL(auth.on_air_bytes(FrameType::Final, profile),
                        mac_auth.on_air_bytes(FrameType::Final, profile));
    BOOST_REQUIRE(mac_auth.mac_fcs_bytes() == 2u);
    BOOST_REQUIRE(!mac_auth.executable());
}

BOOST_AUTO_TEST_CASE(application_payload_macsdu_and_fcs_are_distinct)
{
    // Payload is a MAC-PSDU sub-field: an oversized payload is its own reason.
    TwrConfig c = minimal();
    c.frame.mac_psdu_bytes = 32u;
    c.frame.application_payload_bytes = 64u;
    const ValidationReport v = validate(c, measured_caps());
    require_machine_readable(v);
    BOOST_REQUIRE(has(v, "frame.application_payload_bytes",
        ConfigReason::FrameLengthOverflow));
    // A payload that fits is not required: the frame profile needs no payload.
    TwrConfig ok = minimal();
    ok.frame.application_payload_bytes = 0u;
    BOOST_REQUIRE(validate(ok, measured_caps()).ok());
    // mac_psdu_bytes is independent of the application payload -- but NOT of
    // the geometry, which is a claim the codec owns.  M0 asserted a 2-byte
    // header validated; it is refused now, per field, because the header is 14
    // and claiming 2 understates the buffer and frame-duration budget.
    TwrConfig g = minimal();
    g.frame.geometry.mac_header_bytes = 2u;
    g.frame.application_payload_bytes = 20u;
    const ValidationReport w = validate(g, measured_caps());
    require_machine_readable(w);
    const ConfigViolation* viol = w.first_for_field("frame.geometry.mac_header_bytes");
    BOOST_REQUIRE_MESSAGE(viol != nullptr, "a 2-byte header claim was accepted");
    BOOST_REQUIRE(viol->message.find("geometry authority") != std::string::npos);
}

BOOST_AUTO_TEST_CASE(sfd_length_must_match_the_mode)
{
    TwrConfig c = minimal();
    c.frame.sfd_symbols = 4u; // 4z2 is 8 symbols
    const ValidationReport v = validate(c, measured_caps());
    require_machine_readable(v);
    BOOST_REQUIRE(has(v, "frame.sfd_symbols", ConfigReason::FieldConflict));
    // Symbol counts agree with the reference sequence lengths.
    BOOST_REQUIRE_EQUAL(sfd_mode_symbols(SfdMode::R4z1), 4);
    BOOST_REQUIRE_EQUAL(sfd_mode_symbols(SfdMode::R4z2), 8);
    BOOST_REQUIRE_EQUAL(sfd_mode_symbols(SfdMode::R4z3), 16);
    BOOST_REQUIRE_EQUAL(sfd_mode_symbols(SfdMode::R4z4), 32);
    BOOST_REQUIRE_EQUAL(sfd_mode_symbols(SfdMode::Dwt8), 8);
    BOOST_REQUIRE_EQUAL(sfd_mode_symbols(SfdMode::Ieee8), 8);
}

// ---------------------------------------------------------------------------
// M0.1 / R2: the PHR rate is its OWN quantity, not the payload rate.
//
// The modulator's frozen profile (uwb_hrp_mod_core.h:8-12) is "0.85 Mb/s PHR
// + 6.81 Mb/s payload", and the constants back it: the PHR is 21 symbols of
// 512 chips (kPhrSymbols / kPhrChipsPerSymbol) while the payload is 8 chips
// per burst of 64 per symbol.  The PHR is a different modulation of a
// different rate.  The data-rate field that lives INSIDE the PHR describes the
// PAYLOAD, so it says nothing about how the PHR itself is transmitted.
//
// The M0 rule `phr_rate == data_rate` therefore had it exactly backwards: it
// accepted the 6.81 Mb/s PHR this modulator cannot produce and rejected the
// 0.85 Mb/s PHR it actually produces.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(phr_rate_is_independent_of_the_payload_rate)
{
    // 0.85 Mb/s PHR + 6.81 Mb/s payload: the profile that is really modulated.
    // This configuration was REJECTED by M0's `phr_rate == data_rate` rule.
    TwrConfig c = minimal();
    c.phy.phr_rate = PhrRate::Standard850k;
    c.phy.data_rate = DataRate::R6p8M;
    const ValidationReport v = validate(c, measured_caps());
    BOOST_REQUIRE_MESSAGE(v.ok(),
                          "the modulator's own 0.85 Mb/s PHR was rejected: " + v.to_string());
    BOOST_REQUIRE(!v.has_field("phy.phr_rate"));

    // The two axes are genuinely independent: an unsupported PAYLOAD rate is
    // refused by the payload whitelist while the PHR, at its own rate, is not
    // implicated at all.
    TwrConfig d = minimal();
    d.phy.phr_rate = PhrRate::Standard850k;
    d.phy.data_rate = DataRate::R27M;
    const ValidationReport w = validate(d, measured_caps());
    require_machine_readable(w);
    BOOST_REQUIRE(has(w, "phy.data_rate", ConfigReason::Unsupported));
    BOOST_REQUIRE(!w.has_field("phy.phr_rate"));
}

BOOST_AUTO_TEST_CASE(every_other_phr_rate_is_rejected_with_its_own_reason)
{
    // Exactly one member is implemented.  Every other member -- including the
    // out-of-enum values a cast can produce -- is refused, and each refusal
    // names its own member and its own reason.  The obsolete "must equal the
    // data rate" wording is gone for good.
    for (size_t i = 0; i < kPhrRateCount; ++i) {
        const PhrRate r = kAllPhrRates[i];
        if (phr_rate_is_implemented(r)) {
            TwrConfig c = minimal();
            c.phy.phr_rate = r;
            BOOST_REQUIRE_MESSAGE(validate(c, measured_caps()).ok(),
                                  std::string("implemented PHR rate ") +
                                      phr_rate_to_string(r) + " was rejected");
            continue;
        }
        TwrConfig c = minimal();
        c.phy.phr_rate = r;
        const ValidationReport v = validate(c, measured_caps());
        require_machine_readable(v);
        const ConfigViolation* viol = v.first_for_field("phy.phr_rate");
        BOOST_REQUIRE_MESSAGE(viol != nullptr,
                              std::string("PHR rate ") + phr_rate_to_string(r) +
                                  " was accepted");
        BOOST_REQUIRE(viol->reason == ConfigReason::Unsupported);
        BOOST_REQUIRE(viol->status == ExchangeStatus::Unsupported);
        BOOST_REQUIRE(viol->requirement == "REQ-PHY-02");
        // The message names the member, carries its specific reason, and never
        // claims the two rates must be equal.
        BOOST_REQUIRE(viol->message.find(phr_rate_to_string(r)) != std::string::npos);
        BOOST_REQUIRE(viol->message.find("must equal") == std::string::npos);
        BOOST_REQUIRE(viol->message.find(phr_rate_unsupported_reason(r).substr(0, 40)) !=
                      std::string::npos);
    }
    // `same_as_data` in particular is refused as a limit of THIS software, and
    // says so rather than claiming the option does not exist.
    {
        TwrConfig c = minimal();
        c.phy.phr_rate = PhrRate::SameAsData;
        const ValidationReport v = validate(c, measured_caps());
        const ConfigViolation* viol = v.first_for_field("phy.phr_rate");
        BOOST_REQUIRE(viol != nullptr);
        BOOST_REQUIRE(viol->message.find("Qorvo") != std::string::npos);
        BOOST_REQUIRE(viol->message.find("21 symbols") != std::string::npos);
    }
    // Values outside the enumeration are refused as OUT-OF-DOMAIN.  Note this
    // is a different answer from "a real member this build does not implement"
    // (PhrRate::SameAsData above, which stays Unsupported): N07 made the
    // domain question explicit instead of leaving it to be caught incidentally
    // by the support check.
    for (unsigned r = 2; r < 8; ++r) {
        TwrConfig c = minimal();
        c.phy.phr_rate = static_cast<PhrRate>(r);
        const ValidationReport v = validate(c, measured_caps());
        require_machine_readable(v);
        const ConfigViolation* viol = v.first_for_field("phy.phr_rate");
        BOOST_REQUIRE_MESSAGE(viol != nullptr, "an out-of-enum PHR rate was accepted");
        BOOST_REQUIRE(viol->reason == ConfigReason::UnknownEnumValue);
    }
    // The string form round-trips, and a PAYLOAD-rate name is not a PHR-rate
    // name any more: the two axes no longer share a value domain.
    {
        PhrRate r = PhrRate::Standard850k;
        for (size_t i = 0; i < kPhrRateCount; ++i) {
            const PhrRate m = kAllPhrRates[i];
            BOOST_REQUIRE(phr_rate_from_string(phr_rate_to_string(m), r));
            BOOST_REQUIRE(r == m);
            BOOST_REQUIRE(!phr_rate_from_string("nope", r));
            // "850k" is a legal spelling of BOTH enums: it is the PHR rate and
            // also a payload-rate class.  The DOMAIN is what separates them.
            BOOST_REQUIRE(phr_rate_from_string("850k", r));
            BOOST_REQUIRE(r == PhrRate::Standard850k);
        }
        BOOST_REQUIRE(!phr_rate_from_string("6p8m", r));
        BOOST_REQUIRE(!phr_rate_from_string("same as data", r));
        BOOST_REQUIRE(!phr_rate_from_string("SameAsData", r));
        BOOST_REQUIRE_EQUAL(std::string(phr_rate_to_string(PhrRate::SameAsData)),
                            std::string("same_as_data"));
    }
    // A PHR rate with no implementation has 0 symbols rather than a plausible
    // budget, so it can never be mistaken for a real one.
    BOOST_REQUIRE_EQUAL(phr_rate_symbols(PhrRate::Standard850k), 21u);
    BOOST_REQUIRE_EQUAL(phr_rate_symbols(PhrRate::SameAsData), 0u);
}

BOOST_AUTO_TEST_CASE(phr_mode_members_each_carry_their_own_reason)
{
    // PHR presence/form is a different axis from the PHR rate.  M0 rejected
    // only `none` and let `extended` through as if it were a valid profile.
    for (int i = 0; i <= static_cast<int>(PhrMode::None); ++i) {
        const PhrMode m = static_cast<PhrMode>(i);
        TwrConfig c = minimal();
        c.frame.phr_mode = m;
        const ValidationReport v = validate(c, measured_caps());
        const ConfigViolation* viol = v.first_for_field("frame.phr_mode");
        if (m == PhrMode::Standard) {
            BOOST_REQUIRE_MESSAGE(viol == nullptr, "the standard PHR was rejected");
            continue;
        }
        require_machine_readable(v);
        BOOST_REQUIRE_MESSAGE(viol != nullptr, "an unimplemented PHR mode was accepted");
        BOOST_REQUIRE(viol->status == ExchangeStatus::Unsupported);
        // Each member's own words, not one shared sentence.
        BOOST_REQUIRE(viol->message.find(phr_mode_unsupported_reason(m).substr(0, 40)) !=
                      std::string::npos);
        if (m == PhrMode::None)
            BOOST_REQUIRE(viol->reason == ConfigReason::OutOfScope);
        else
            BOOST_REQUIRE(viol->reason == ConfigReason::Unsupported);
    }
    // `extended` is a vendor PHR layout with no encoder here; the reason says
    // so instead of leaving it silently accepted.
    {
        TwrConfig c = minimal();
        c.frame.phr_mode = PhrMode::Extended;
        const ConfigViolation* viol =
            validate(c, measured_caps()).first_for_field("frame.phr_mode");
        BOOST_REQUIRE(viol != nullptr);
        BOOST_REQUIRE(viol->message.find("extended") != std::string::npos);
        BOOST_REQUIRE(viol->message.find("Qorvo") != std::string::npos);
    }
    // Out-of-enum values are refused, not coerced to Standard.
    for (unsigned m = 3; m < 8; ++m) {
        TwrConfig c = minimal();
        c.frame.phr_mode = static_cast<PhrMode>(m);
        const ValidationReport v = validate(c, measured_caps());
        require_machine_readable(v);
        BOOST_REQUIRE(v.first_for_field("frame.phr_mode") != nullptr);
    }
}

// ---------------------------------------------------------------------------
// M0.1 / R3: the config may CLAIM a geometry, it may not DECIDE one.
//
// uwb_twr_frame.h is the authority for frame v1's byte geometry.  The config
// states what it believes the layout is; frame_geometry_check() reports every
// field that disagrees, and the validator turns each one into its own
// violation naming that exact field.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(the_codec_is_the_only_frame_geometry_authority)
{
    FrameProfileGeometry auth;
    FrameProfile profile;
    BOOST_REQUIRE(frame_geometry_for(FrameProfileId::TwrV1, auth));
    BOOST_REQUIRE(frame_profile_for(FrameProfileId::TwrV1, profile));

    // The shipped/default geometry, read from the authority and not from a
    // literal in this file.
    BOOST_REQUIRE_EQUAL(auth.mac_header_bytes(), 14u);
    BOOST_REQUIRE_EQUAL(auth.timestamp_bytes(profile), 5u);
    BOOST_REQUIRE_EQUAL(auth.mac_footer_bytes(), 0u);
    BOOST_REQUIRE_EQUAL(auth.mac_fcs_bytes(), 0u);
    BOOST_REQUIRE_EQUAL(auth.phr_bytes(), 2u);
    BOOST_REQUIRE_EQUAL(auth.phr_coded_bits(), kPhrStandardCodedBits);
    BOOST_REQUIRE(auth.executable());

    // On the air, with the FCS the modulation layer appends exactly once.
    BOOST_REQUIRE_EQUAL(auth.on_air_bytes(FrameType::Poll, profile), 16u);
    BOOST_REQUIRE_EQUAL(auth.on_air_bytes(FrameType::Response, profile), 26u);
    BOOST_REQUIRE_EQUAL(auth.on_air_bytes(FrameType::Final, profile), 31u);

    // The claim the authority itself produces checks back with no mismatch:
    // this is what makes "one place the numbers come from" testable.
    const GeometryCheckResult self =
        frame_geometry_check(auth.claim(profile), auth, profile);
    BOOST_REQUIRE_MESSAGE(self.ok(), self.summary());
    BOOST_REQUIRE(self.first_mismatch() == nullptr);
}

BOOST_AUTO_TEST_CASE(frame_geometry_claim_must_match_the_codec_authority)
{
    // The R3 regression: the pre-codec 7-byte header that used to validate.
    TwrConfig c = minimal();
    c.frame.geometry.mac_header_bytes = 7u;
    const ValidationReport v = validate(c, measured_caps());
    require_machine_readable(v);
    const ConfigViolation* viol = v.first_for_field("frame.geometry.mac_header_bytes");
    BOOST_REQUIRE_MESSAGE(viol != nullptr,
                          "a 7-byte header claim was accepted: " + v.to_string());
    BOOST_REQUIRE(viol->reason == ConfigReason::FieldConflict);
    // The message names the field and both numbers, so the operator can see
    // which of the two geometries is the real one.
    BOOST_REQUIRE(viol->message.find("mac_header_bytes") != std::string::npos);
    BOOST_REQUIRE(viol->message.find("14") != std::string::npos);
    BOOST_REQUIRE(viol->message.find("7") != std::string::npos);

    // Every one of the five comparable fields is checked on its own, and a
    // claim that is wrong in several of them produces one violation each.
    struct Probe {
        const char* what;
        void (*mutate)(FrameGeometry&);
        const char* field;
    };
    const Probe probes[] = {
        { "mac_header_bytes", [](FrameGeometry& g) { g.mac_header_bytes = 7; },
            "frame.geometry.mac_header_bytes" },
        { "timestamp_bytes", [](FrameGeometry& g) { g.timestamp_bytes = 4; },
            "frame.geometry.timestamp_bytes" },
        { "mac_footer_bytes", [](FrameGeometry& g) { g.mac_footer_bytes = 3; },
            "frame.geometry.mac_footer_bytes" },
        { "mac_fcs_bytes", [](FrameGeometry& g) { g.mac_fcs_bytes = 2; },
            "frame.geometry.mac_fcs_bytes" },
        { "phr_bytes", [](FrameGeometry& g) { g.phr_bytes = 12; },
            "frame.geometry.phr_bytes" },
    };
    for (const Probe& pr : probes) {
        TwrConfig x = minimal();
        pr.mutate(x.frame.geometry);
        const ValidationReport r = validate(x, measured_caps());
        require_machine_readable(r);
        BOOST_REQUIRE_MESSAGE(r.first_for_field(pr.field) != nullptr,
                              std::string("a wrong ") + pr.what + " claim was accepted: " +
                                  r.to_string());
    }
    // A claim that is wrong in two fields names both, not just the first.
    TwrConfig multi = minimal();
    multi.frame.geometry.mac_header_bytes = 7u;
    multi.frame.geometry.phr_bytes = 12u;
    const ValidationReport m = validate(multi, measured_caps());
    require_machine_readable(m);
    BOOST_REQUIRE(m.first_for_field("frame.geometry.mac_header_bytes") != nullptr);
    BOOST_REQUIRE(m.first_for_field("frame.geometry.phr_bytes") != nullptr);

    // The authority's own claim validates, so the fixture is not carrying a
    // private copy of the numbers.
    TwrConfig good = minimal();
    BOOST_REQUIRE_MESSAGE(validate(good, measured_caps()).ok(),
                          "the authority's own geometry was rejected: " +
                              validate(good, measured_caps()).to_string());
}

BOOST_AUTO_TEST_CASE(effective_frame_budget_is_the_codecs_own)
{
    // The effective budget must be the codec's, not a re-summation of a
    // claim: 16 / 26 / 31 on air for Poll / Response / Final.
    FrameProfileGeometry auth;
    FrameProfile profile;
    BOOST_REQUIRE(frame_geometry_for(FrameProfileId::TwrV1, auth));
    BOOST_REQUIRE(frame_profile_for(FrameProfileId::TwrV1, profile));

    const EffectiveConfig e = effective_config(minimal(Protocol::Ds, Role::Initiator),
        measured_caps());
    BOOST_REQUIRE_MESSAGE(e.ok, e.validation.to_string());
    BOOST_REQUIRE_EQUAL(e.poll_bytes, auth.on_air_bytes(FrameType::Poll, profile));
    BOOST_REQUIRE_EQUAL(e.response_bytes, auth.on_air_bytes(FrameType::Response, profile));
    BOOST_REQUIRE_EQUAL(e.final_bytes, auth.on_air_bytes(FrameType::Final, profile));
    // And not the 9 / 19 / 24 the pre-codec 7-byte claim produced.
    BOOST_REQUIRE(e.poll_bytes != 9u);
    BOOST_REQUIRE(e.final_bytes != 24u);
    // The PHR of the profile is recorded, so no consumer recomputes it: 2
    // information octets, SEC-DED coded to 19 bits over 21 symbols.  The
    // fixture's `phr_bytes = 12` from M0 was neither the information size nor
    // anything else the standard PHR has.
    BOOST_REQUIRE_EQUAL(e.phr_bytes, auth.phr_bytes());
    BOOST_REQUIRE_EQUAL(e.phr_bytes, 2u);
    BOOST_REQUIRE_EQUAL(e.phr_coded_bits, auth.phr_coded_bits());
    BOOST_REQUIRE_EQUAL(e.phr_coded_bits, kPhrStandardCodedBits);
    // ... and the PHR rate agrees with the modulator's own description of the
    // frozen profile: one implemented rate, 21 symbols, and nothing else.
    BOOST_REQUIRE_EQUAL(phr_rate_symbols(PhrRate::Standard850k), 21u);
    BOOST_REQUIRE_EQUAL(phr_rate_symbols(PhrRate::SameAsData), 0u);
}

// ---------------------------------------------------------------------------
// M0.1: the SYNC repetition count is a MEMBER of an enumeration, and every
// refusal names a limit of THIS software rather than of a chip.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(preamble_length_enum_bounds_the_sync_repetitions)
{
    // Both members are accepted: 64 is the phase-1 profile, 16 is
    // decode-verified (its CFO-fit tail covers the whole preamble and
    // encode_phr19 advertises the legal duration 16).
    for (size_t i = 0; i < kPreambleLengthCount; ++i) {
        const PreambleLength p = kAllPreambleLengths[i];
        TwrConfig c = minimal();
        c.phy.preamble_symbols = preamble_length_symbols(p);
        BOOST_REQUIRE_MESSAGE(validate(c, capabilities()).ok(),
                              std::string("SYNC length ") + preamble_length_to_string(p) +
                                  " was rejected");
    }
    BOOST_REQUIRE_EQUAL(preamble_length_symbols(PreambleLength::Sym16), 16u);
    BOOST_REQUIRE_EQUAL(preamble_length_symbols(PreambleLength::Sym64), 64u);
    BOOST_REQUIRE_EQUAL(kPreambleLengthCount, 2u);

    // 128 SYNC and every other non-member are refused.
    const uint16_t refused[] = { 0,    1,    2,    4,    8,     32,    128,
                                 256,  512,  1024, 2048, 100,   4096,  0xffff };
    for (uint16_t reps : refused) {
        TwrConfig c = minimal();
        c.phy.preamble_symbols = reps;
        const ValidationReport v = validate(c, capabilities());
        require_machine_readable(v);
        const ConfigViolation* viol = v.first_for_field("phy.preamble_symbols");
        BOOST_REQUIRE_MESSAGE(viol != nullptr, twr_int_to_text(reps) + " SYNC was accepted");
        if (reps == 0) {
            // 0 is "not stated", not "an unsupported length".
            BOOST_REQUIRE(viol->reason == ConfigReason::ZeroValue);
            continue;
        }
        BOOST_REQUIRE(viol->reason == ConfigReason::Unsupported);
        BOOST_REQUIRE(viol->status == ExchangeStatus::Unsupported);
        BOOST_REQUIRE(viol->requirement == "REQ-PHY-01");
        BOOST_REQUIRE(viol->message == preamble_length_unsupported_reason(reps));
    }
    // 128 specifically: the reason names the demodulator and the measurement,
    // and explicitly says the Qorvo parts' API accepts it.
    {
        TwrConfig c = minimal();
        c.phy.preamble_symbols = 128;
        const ConfigViolation* viol =
            validate(c, capabilities()).first_for_field("phy.preamble_symbols");
        BOOST_REQUIRE(viol != nullptr);
        BOOST_REQUIRE(viol->message.find("CURRENT DEMODULATOR") != std::string::npos);
        BOOST_REQUIRE(viol->message.find("not by any hardware") != std::string::npos);
        BOOST_REQUIRE(viol->message.find("Qorvo") != std::string::npos);
        BOOST_REQUIRE(viol->message.find("max(0, reps - 64)") != std::string::npos);
        // It must never say the hardware is incapable of 128.
        BOOST_REQUIRE(viol->message.find("cannot do 128") == std::string::npos);
        BOOST_REQUIRE(viol->message.find("not supported by the chip") ==
                      std::string::npos);
    }
    // The reason helper is total, and every length it is asked about gets a
    // message that names the length itself.
    for (uint16_t reps = 1; reps < 300; ++reps) {
        const std::string why = preamble_length_unsupported_reason(reps);
        BOOST_REQUIRE(!why.empty());
        BOOST_REQUIRE_MESSAGE(why.find(twr_int_to_text(reps)) != std::string::npos,
                              "the reason for " + twr_int_to_text(reps) +
                                  " does not name the length");
    }
    // The string / symbols round trips, and anything else is refused rather
    // than rounded to the nearest member.
    {
        PreambleLength p = PreambleLength::Sym64;
        for (size_t i = 0; i < kPreambleLengthCount; ++i) {
            const PreambleLength m = kAllPreambleLengths[i];
            BOOST_REQUIRE(preamble_length_from_string(preamble_length_to_string(m), p));
            BOOST_REQUIRE(p == m);
            BOOST_REQUIRE(preamble_length_from_symbols(preamble_length_symbols(m), p));
            BOOST_REQUIRE(p == m);
        }
        BOOST_REQUIRE(!preamble_length_from_string("128", p));
        BOOST_REQUIRE(!preamble_length_from_string("nope", p));
        BOOST_REQUIRE(!preamble_length_from_string("0x40", p));
        BOOST_REQUIRE(!preamble_length_from_symbols(128, p));
        BOOST_REQUIRE(!preamble_length_from_symbols(32, p));
        BOOST_REQUIRE(!preamble_length_from_symbols(0, p));
        BOOST_REQUIRE_EQUAL(std::string(preamble_length_to_string(PreambleLength::Sym16)),
                            std::string("16"));
        BOOST_REQUIRE_EQUAL(std::string(preamble_length_to_string(PreambleLength::Sym64)),
                            std::string("64"));
        // An out-of-enum value has no name and no length, so it cannot be
        // silently rendered as a real one.
        BOOST_REQUIRE_EQUAL(
            std::string(preamble_length_to_string(static_cast<PreambleLength>(9))),
            std::string("invalid"));
        BOOST_REQUIRE_EQUAL(preamble_length_symbols(static_cast<PreambleLength>(9)), 0u);
    }
    // The capability whitelist is a SEPARATE, second gate.
    BOOST_REQUIRE(capabilities().sync_repetitions_supported(16));
    BOOST_REQUIRE(capabilities().sync_repetitions_supported(64));
    BOOST_REQUIRE(!capabilities().sync_repetitions_supported(128));
    BOOST_REQUIRE(!capabilities().sync_repetitions_supported(32));
    // 16 is decode-verified but its first-path / ToA accuracy is NOT measured,
    // so it is accepted as a config and still refused as a ranging profile.
    BOOST_REQUIRE(twr_sync_reps_needs_toa_validation(
        preamble_length_symbols(PreambleLength::Sym16)));
    BOOST_REQUIRE(!twr_sync_reps_needs_toa_validation(
        preamble_length_symbols(PreambleLength::Sym64)));
}

// ---------------------------------------------------------------------------
// M0.1 / R3: the frame profile is an ENUM, so an arbitrary profile NAME
// cannot act as an executable geometry.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(unknown_frame_profile_is_rejected_not_defaulted)
{
    for (unsigned i = 0; i < 8; ++i) {
        const FrameProfileId id = static_cast<FrameProfileId>(i);
        TwrConfig c = minimal();
        c.frame.frame_profile = id;
        const ValidationReport v = validate(c, measured_caps());
        if (frame_profile_id_is_supported(id)) {
            BOOST_REQUIRE_MESSAGE(v.ok(), "an implemented frame profile was rejected");
            continue;
        }
        require_machine_readable(v);
        const ConfigViolation* viol = v.first_for_field("frame.frame_profile");
        BOOST_REQUIRE_MESSAGE(viol != nullptr, "an unknown frame profile was accepted");
        // N07: the DOMAIN answer comes first now -- a cast-in id is not a value
        // the enum has, which is a different (and more precise) statement than
        // "this build does not implement it".
        BOOST_REQUIRE(viol->reason == ConfigReason::UnknownEnumValue);
        // ... and the support answer is still given, so a caller that only
        // reads that one is not left without the codec explanation.
        const ConfigViolation* unsup_viol =
            v.find("frame.frame_profile", ConfigReason::Unsupported);
        BOOST_REQUIRE(unsup_viol != nullptr);
        BOOST_REQUIRE(unsup_viol->message.find("free-form") != std::string::npos);
    }
    // The strings round trip and nothing else is accepted, so a document
    // cannot name a profile this build has no codec for.
    {
        FrameProfileId id = FrameProfileId::TwrV1;
        for (size_t i = 0; i < kFrameProfileIdCount; ++i) {
            const FrameProfileId m = kAllFrameProfileIds[i];
            BOOST_REQUIRE(frame_profile_id_from_string(frame_profile_id_to_string(m), id));
            BOOST_REQUIRE(id == m);
        }
        BOOST_REQUIRE(!frame_profile_id_from_string("frame_v2", id));
        BOOST_REQUIRE(!frame_profile_id_from_string("", id));
        BOOST_REQUIRE(!frame_profile_id_from_string("qa-synthetic-frame/0", id));
    }
    // The fixture's profile is the one the authority serves, and the geometry
    // came from that authority rather than from a literal.
    {
        const TwrConfig c = minimal();
        FrameGeometry from_codec;
        BOOST_REQUIRE(frame_geometry_of_profile(c.frame.frame_profile, from_codec));
        BOOST_REQUIRE(from_codec == c.frame.geometry);
        FrameGeometry none;
        BOOST_REQUIRE(!frame_geometry_of_profile(static_cast<FrameProfileId>(9), none));
    }
}

// ---------------------------------------------------------------------------
// M0.1 / R3: the session id is narrowed by REFUSAL, never by truncation.
//
// kOffSessionId is 2 bytes.  session_id_to_wire() is identity-or-refuse: in
// range -> exact, out of range -> explicit error.  A fold, a mask or a hash
// would make two different local sessions produce byte-identical session
// fields, and frame_match() would then accept the wrong session's reply.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(session_id_must_fit_the_codec_wire_field)
{
    // In range: accepted, and the wire value is the local value, bit for bit.
    const uint32_t in_range[] = { 1u, 0x1234u, 0x8000u, kSessionIdWireMax };
    for (uint32_t id : in_range) {
        TwrConfig c = minimal();
        c.session.session_id = id;
        const ValidationReport v = validate(c, measured_caps());
        BOOST_REQUIRE_MESSAGE(v.ok(),
                              twr_int_to_text(id) + " was rejected: " + v.to_string());
        uint16_t wire = 0xffffu;
        std::string err;
        BOOST_REQUIRE_MESSAGE(session_id_to_wire(id, wire, err), err);
        BOOST_REQUIRE_EQUAL(static_cast<uint32_t>(wire), id);
    }

    // Out of range: REFUSED, naming the width and the value.
    const uint32_t out_of_range[] = { 0x00010000u, 0x0001ffffu, 0xffffffffu };
    for (uint32_t id : out_of_range) {
        TwrConfig c = minimal();
        c.session.session_id = id;
        const ValidationReport v = validate(c, measured_caps());
        require_machine_readable(v);
        const ConfigViolation* viol = v.first_for_field("session.session_id");
        BOOST_REQUIRE_MESSAGE(viol != nullptr,
                              twr_int_to_text(id) + " was accepted, it does not fit 16 bits");
        BOOST_REQUIRE(viol->reason == ConfigReason::OutOfRange);
        BOOST_REQUIRE(viol->status == ExchangeStatus::ConfigRejected);
        BOOST_REQUIRE(viol->message.find("16") != std::string::npos);
        BOOST_REQUIRE(viol->message.find(twr_int_to_text(id)) != std::string::npos);

        // The mapping itself agrees with the validator, and it leaves nothing
        // plausible-looking behind on failure.
        uint16_t wire = 0x1234u;
        std::string err;
        SessionIdError code = SessionIdError::None;
        BOOST_REQUIRE(!session_id_to_wire(id, wire, err, &code));
        BOOST_REQUIRE(code == SessionIdError::OutOfWireRange);
        BOOST_REQUIRE_EQUAL(static_cast<int>(wire), 0);
        BOOST_REQUIRE(err.find("not truncated") != std::string::npos ||
                      err.find("refused") != std::string::npos);
    }

    // The consequence of a 16-bit session field, stated as a number rather
    // than as prose: ~256 concurrently live sessions at a 50% chance of one
    // collision.  It is a property of the field, not of this config.
    BOOST_REQUIRE_EQUAL(kSessionIdWireBits, 16u);
    BOOST_REQUIRE_EQUAL(kSessionIdWireMax, 0xffffu);
    BOOST_REQUIRE_CLOSE(kSessionIdBirthdaySessions50pct, 256.0, 1e-9);
    // Two accepted ids are indistinguishable on the wire iff they are the same
    // integer; the predicate is what makes the injectivity claim testable.
    BOOST_REQUIRE(wire_session_id_collides(0x0001u, 0x0001u));
    BOOST_REQUIRE(!wire_session_id_collides(0x0001u, 0x0002u));
    BOOST_REQUIRE(!wire_session_id_collides(0x00010000u, 0x00010000u));
}

BOOST_AUTO_TEST_CASE(addresses_and_pan_must_be_sane)
{
    TwrConfig c = minimal();
    c.session.peer_address = c.session.local_address;
    BOOST_REQUIRE(has(validate(c, measured_caps()), "session.peer_address",
                      ConfigReason::FieldConflict));
    TwrConfig d = minimal();
    d.session.local_address = 0xffff;
    BOOST_REQUIRE(has(validate(d, measured_caps()), "session.local_address",
                      ConfigReason::FieldConflict));
    TwrConfig e = minimal();
    e.session.pan_id = 0x0000;
    BOOST_REQUIRE(has(validate(e, measured_caps()), "session.pan_id",
        ConfigReason::EmptyValue));
    TwrConfig f = minimal();
    f.session.session_id = 0u;
    BOOST_REQUIRE(has(validate(f, measured_caps()), "session.session_id",
                      ConfigReason::ZeroValue));
}

BOOST_AUTO_TEST_CASE(sequence_wrap_is_independent_of_the_session_id)
{
    TwrConfig c = minimal();
    c.session.sequence = 200u;
    c.session.sequence_modulus = 256u;
    BOOST_REQUIRE(validate(c, measured_caps()).ok());
    // Modulus 4 with sequence 3 is fine; 4 is out of range.
    c.session.sequence_modulus = 4u;
    c.session.sequence = 3u;
    BOOST_REQUIRE(validate(c, measured_caps()).ok());
    c.session.sequence = 4u;
    BOOST_REQUIRE(has(validate(c, measured_caps()), "session.sequence",
                      ConfigReason::IndexOutOfRange));
    // A modulus that is not 4/16/64/256 is rejected, not rounded.
    TwrConfig d = minimal();
    d.session.sequence_modulus = 100u;
    BOOST_REQUIRE(has(validate(d, measured_caps()), "session.sequence_modulus",
                      ConfigReason::OutOfRange));
}

BOOST_AUTO_TEST_CASE(retry_policy_is_consistent)
{
    TwrConfig c = minimal();
    c.session.max_attempts_per_exchange = 3u;
    const ValidationReport v = validate(c, measured_caps());
    require_machine_readable(v);
    BOOST_REQUIRE(has(v, "session.retry_backoff", ConfigReason::FieldConflict));
    c.session.retry_backoff = Duration::from_nanos(1'000'000);
    BOOST_REQUIRE(validate(c, measured_caps()).ok());
    // A backoff with a single attempt is a conflict, not a no-op.
    TwrConfig d = minimal();
    d.session.retry_backoff = Duration::from_nanos(1'000'000);
    BOOST_REQUIRE(has(validate(d, measured_caps()), "session.retry_backoff",
                      ConfigReason::FieldConflict));
}

// ===========================================================================
// F. Resource conflicts on the same physical channel
// ===========================================================================

BOOST_AUTO_TEST_CASE(same_physical_channel_for_two_conflicting_roles)
{
    // The peer claims the local endpoint's TX channel.
    TwrConfig c = minimal(Protocol::Ss, Role::Initiator);
    c.radio.peers.front().tx_channel = c.radio.tx_channel;
    const ValidationReport v = validate(c, measured_caps());
    require_machine_readable(v);
    const ConfigViolation* viol =
        v.find("radio.peers[0].tx_channel", ConfigReason::DuplicateResource);
    BOOST_REQUIRE(viol != nullptr);
    BOOST_REQUIRE(viol->status == ExchangeStatus::ConfigRejected);
    BOOST_REQUIRE(viol->requirement == "REQ-BASE-01");
    BOOST_REQUIRE(viol->message.find("one owner") != std::string::npos);

    // ... and the same for the RX channel.
    TwrConfig d = minimal();
    d.radio.peers.front().rx_channel = d.radio.rx_channel;
    const ValidationReport w = validate(d, measured_caps());
    require_machine_readable(w);
    BOOST_REQUIRE(w.find("radio.peers[0].rx_channel",
        ConfigReason::DuplicateResource) !=
                  nullptr);

    // A peer that claims the SAME role is a protocol conflict.
    TwrConfig e = minimal();
    e.radio.peers.front().role = Role::Initiator;
    const ValidationReport x = validate(e, measured_caps());
    require_machine_readable(x);
    BOOST_REQUIRE(x.find("radio.peers[0].role",
        ConfigReason::FieldConflict) != nullptr);

    // Two peers colliding with each other.
    TwrConfig f = minimal();
    EndpointBinding second = f.radio.peers.front(); // same id, same channels
    f.radio.peers.push_back(second);
    const ValidationReport y = validate(f, measured_caps());
    require_machine_readable(y);
    BOOST_REQUIRE(y.find("radio.peers[1].tx_channel",
        ConfigReason::DuplicateResource) !=
                  nullptr);
    BOOST_REQUIRE(y.find("radio.peers[1].rx_channel",
        ConfigReason::DuplicateResource) !=
                  nullptr);
    BOOST_REQUIRE(y.find("radio.peers[1].id",
        ConfigReason::DuplicateResource) != nullptr);
}

BOOST_AUTO_TEST_CASE(endpoints_sharing_a_device_share_the_native_rate)
{
    TwrConfig c = minimal();
    c.radio.peers.front().native_sample_rate_hz = kTwrNativeRateCg400Hz;
    const ValidationReport v = validate(c, measured_caps());
    require_machine_readable(v);
    BOOST_REQUIRE(v.find("radio.peers[0].native_sample_rate_hz",
        ConfigReason::FieldConflict) !=
                  nullptr);
}

BOOST_AUTO_TEST_CASE(peer_that_does_not_occupy_resources_is_not_a_conflict)
{
    TwrConfig c = minimal();
    c.radio.peers.front().occupies_resources = false;
    c.radio.peers.front().tx_channel = c.radio.tx_channel;
    c.radio.peers.front().rx_channel = c.radio.rx_channel;
    BOOST_REQUIRE(validate(c, measured_caps()).ok());
}

BOOST_AUTO_TEST_CASE(readback_mismatch_is_a_startup_failure)
{
    TwrConfig c = minimal();
    c.radio.readback.sample_rate_hz = 491520000.0;
    const ValidationReport v = validate(c, measured_caps());
    require_machine_readable(v);
    BOOST_REQUIRE(has(v, "radio.readback.sample_rate_hz", ConfigReason::FieldConflict));

    TwrConfig d = minimal();
    d.radio.readback.tx_channel = static_cast<uint8_t>(d.radio.tx_channel + 1);
    BOOST_REQUIRE(has(validate(d, measured_caps()), "radio.readback.tx_channel",
                      ConfigReason::FieldConflict));

    TwrConfig e = minimal();
    e.radio.readback.clock_source = "gpsdo";
    BOOST_REQUIRE(has(validate(e, measured_caps()), "radio.readback.clock_source",
                      ConfigReason::FieldConflict));

    // require_readback without a readback is a CalibrationMissing-class failure.
    TwrConfig f = minimal();
    f.radio.readback.present = false;
    const ValidationReport w = validate(f, measured_caps());
    require_machine_readable(w);
    const ConfigViolation* viol = w.find("radio.readback.present",
        ConfigReason::CalibrationMissing);
    BOOST_REQUIRE(viol != nullptr);
    BOOST_REQUIRE(viol->status == ExchangeStatus::CalibrationMissing);

    // An offline dry run may opt out explicitly.
    TwrConfig g = minimal();
    g.radio.readback.present = false;
    g.radio.require_readback = false;
    BOOST_REQUIRE(validate(g, measured_caps()).ok());
}

// ===========================================================================
// G. At most one in-flight exchange
// ===========================================================================

BOOST_AUTO_TEST_CASE(more_than_one_in_flight_exchange_is_unsupported)
{
    TwrConfig c = minimal();
    c.session.max_in_flight_exchanges = 2u;
    const ValidationReport v = validate(c, measured_caps());
    require_machine_readable(v);
    const ConfigViolation* viol =
        v.find("session.max_in_flight_exchanges", ConfigReason::InFlightNotSupported);
    BOOST_REQUIRE(viol != nullptr);
    BOOST_REQUIRE(viol->status == ExchangeStatus::Unsupported);
    BOOST_REQUIRE(viol->message.find("at most 1") != std::string::npos);
    BOOST_REQUIRE_EQUAL(kTwrMaxInFlightExchanges, 1u);
}

BOOST_AUTO_TEST_CASE(exchange_gate_rejects_a_second_concurrent_exchange)
{
    ExchangeGate gate(kTwrMaxInFlightExchanges);
    BOOST_REQUIRE_EQUAL(gate.in_flight(), 0u);
    const ValidationReport first = gate.try_begin();
    BOOST_REQUIRE(first.ok());
    BOOST_REQUIRE_EQUAL(gate.in_flight(), 1u);
    const ValidationReport second = gate.try_begin();
    require_machine_readable(second);
    const ConfigViolation* viol = &second.violations.front();
    BOOST_REQUIRE(viol->reason == ConfigReason::ExchangeAlreadyInFlight);
    BOOST_REQUIRE(viol->status == ExchangeStatus::QueueFull);
    BOOST_REQUIRE(viol->requirement == "REQ-API-01");
    gate.end();
    BOOST_REQUIRE(gate.try_begin().ok());
}

BOOST_AUTO_TEST_CASE(measurement_interval_must_cover_one_whole_exchange)
{
    TwrConfig c = minimal();
    c.session.measurement_count = 10u;
    c.session.measurement_interval = Duration::from_nanos(1'000'000);
    const ValidationReport v = validate(c, measured_caps());
    require_machine_readable(v);
    BOOST_REQUIRE(has(v, "session.measurement_interval.ns",
        ConfigReason::TimingOrderViolation));
    c.session.measurement_interval = Duration::from_nanos(60'000'000);
    BOOST_REQUIRE(validate(c, measured_caps()).ok());
}

// ===========================================================================
// H. Timing: the three distinct quantities, precision and quantisation
// ===========================================================================

BOOST_AUTO_TEST_CASE(reply_delay_post_tx_rx_enable_and_rx_timeout_are_distinct)
{
    TwrConfig c = minimal(Protocol::Ds, Role::Initiator);
    const int64_t reply = c.timing.response_to_final.value.nanos();
    const int64_t post = c.timing.post_tx_rx_enable.value.nanos();
    const int64_t timeout = c.timeouts.rx_timeout.value.nanos();
    BOOST_REQUIRE(reply != post);
    BOOST_REQUIRE(reply != timeout);
    BOOST_REQUIRE(post != timeout);
    // Each carries a different reference event and a different marker.
    BOOST_REQUIRE(c.timing.response_to_final.reference ==
                  TimeReferenceEvent::ResponseReceiveRmarker);
    BOOST_REQUIRE(c.timing.post_tx_rx_enable.reference == TimeReferenceEvent::FrameTail);
    BOOST_REQUIRE(c.timeouts.rx_timeout.reference == TimeReferenceEvent::RxEnable);
    BOOST_REQUIRE(c.timing.response_to_final.marker.value() == TimestampMarker::RmarkerRx);
    BOOST_REQUIRE(c.timing.post_tx_rx_enable.marker.value() == TimestampMarker::PreambleStart);
    BOOST_REQUIRE(c.timeouts.rx_timeout.marker.value() == TimestampMarker::UhdRxFirstIqSample);
    // Changing one does not change the others.
    TwrConfig d = c;
    d.timing.post_tx_rx_enable.value = Duration::from_nanos(post + 500);
    BOOST_REQUIRE_EQUAL(d.timing.response_to_final.value.nanos(), reply);
    BOOST_REQUIRE_EQUAL(d.timeouts.rx_timeout.value.nanos(), timeout);
}

BOOST_AUTO_TEST_CASE(insufficient_timing_precision_is_rejected)
{
    // A device-tick rate finer than the allowed set cannot express ns.
    TwrConfig c = minimal(Protocol::Ds, Role::Initiator);
    c.timing.response_to_final.required_quantisation_hz = 4.0e9; // 0.25 ns period
    const ValidationReport v = validate(c, measured_caps());
    require_machine_readable(v);
    BOOST_REQUIRE(has(v, "timing.response_to_final.quantisation_hz",
                      ConfigReason::QuantisationUnsupported));

    // An ALLOWED rate, but the caller demands an exact nanosecond.
    TwrConfig d = minimal(Protocol::Ds, Role::Initiator);
    d.timing.response_to_final.max_quantisation_error_ns = 0;
    const ValidationReport w = validate(d, measured_caps());
    require_machine_readable(w);
    BOOST_REQUIRE(has(w, "timing.response_to_final.ns",
                      ConfigReason::TimingPrecisionInsufficient));
    // The default tolerance of one nanosecond passes at every allowed rate.
    BOOST_REQUIRE(Duration::from_nanos(600'000)
                      .representable_at(kTwrNativeRateUc200Hz, 1));
    BOOST_REQUIRE(!Duration::from_nanos(600'000)
                       .representable_at(kTwrNativeRateUc200Hz, 0));
    BOOST_REQUIRE(Duration::from_nanos(600'000).representable_at(kTwrWorkSampleRateHz,
        1));

    // The 1 ns error budget is exactly what the 1 GHz, work and 737.28 MS/s
    // rates offer; nothing finer than that is claimed anywhere.
    for (double r : { 1.0e9, kTwrWorkSampleRateHz, kTwrNativeRateUc200Hz }) {
        BOOST_REQUIRE_MESSAGE(Duration::from_nanos(600'001).representable_at(r, 1),
                              twr_double_to_text(r));
        BOOST_REQUIRE_MESSAGE(!Duration::from_nanos(600'001).representable_at(r, 0),
                              twr_double_to_text(r));
    }

    // A REAL insufficient-precision case, not a contrived one: the CG400
    // native rate has a 2.0345 ns device tick, so a 1 ns error budget cannot
    // be met there and the config is rejected instead of being silently
    // rounded.
    BOOST_REQUIRE(!Duration::from_nanos(600'000).representable_at(kTwrNativeRateCg400Hz,
        1));
    BOOST_REQUIRE(Duration::from_nanos(600'000).representable_at(kTwrNativeRateCg400Hz,
        3));
    {
        TwrConfig e = minimal();
        e.timeouts.rx_timeout.required_quantisation_hz = kTwrNativeRateCg400Hz;
        const ValidationReport x = validate(e, measured_caps());
        require_machine_readable(x);
        BOOST_REQUIRE(has(x, "timeouts.rx_timeout.ns",
                          ConfigReason::TimingPrecisionInsufficient));
        // Asking for the tick's true resolution instead is accepted.
        e.timeouts.rx_timeout.max_quantisation_error_ns = 3;
        BOOST_REQUIRE(validate(e, measured_caps()).ok());
    }
}

BOOST_AUTO_TEST_CASE(quantisation_rate_must_be_a_rate_this_build_expresses)
{
    const double rates[] = { 737280000.0, 491520000.0, 998400000.0, 1.0e9 };
    for (double r : rates)
        BOOST_REQUIRE_MESSAGE(is_allowed_quantisation_hz(r), twr_double_to_text(r));
    const double bad[] = { 0.0,   -1.0,      1.0,      2.0e9,
                           1e18,  std::numeric_limits<double>::quiet_NaN(),
                           std::numeric_limits<double>::infinity(),
                           737.28e6 + 5000.0 };
    for (double r : bad)
        BOOST_REQUIRE_MESSAGE(!is_allowed_quantisation_hz(r), twr_double_to_text(r));
}

BOOST_AUTO_TEST_CASE(timed_field_must_state_its_domain_marker_and_reference)
{
    // No domain.
    TwrConfig c = minimal();
    c.timing.post_tx_rx_enable.domain = TimeDomain::Unspecified;
    BOOST_REQUIRE(has(validate(c, measured_caps()), "timing.post_tx_rx_enable.domain",
                      ConfigReason::EmptyValue));
    // Device-tick field with no RF marker.
    TwrConfig d = minimal();
    d.timing.post_tx_rx_enable.marker.reset();
    BOOST_REQUIRE(has(validate(d, measured_caps()), "timing.post_tx_rx_enable.marker",
                      ConfigReason::EmptyValue));
    // Host-monotonic field carrying an RF marker: an invitation to a
    // cross-domain subtraction.
    TwrConfig e = minimal();
    e.timeouts.exchange_timeout.marker = TimestampMarker::RmarkerTx;
    BOOST_REQUIRE(has(validate(e, measured_caps()), "timeouts.exchange_timeout.marker",
                      ConfigReason::FieldConflict));
    // Host-monotonic field not in integer nanoseconds.
    TwrConfig f = minimal();
    f.timeouts.exchange_timeout.required_quantisation_hz = 1.0e6;
    BOOST_REQUIRE(has(validate(f, measured_caps()),
                      "timeouts.exchange_timeout.quantisation_hz",
                      ConfigReason::QuantisationUnsupported));
    // Wrong reference for the field's role.
    TwrConfig g = minimal();
    g.timeouts.rx_timeout.reference = TimeReferenceEvent::PollTransmitRmarker;
    BOOST_REQUIRE(has(validate(g, measured_caps()), "timeouts.rx_timeout.reference",
                      ConfigReason::FieldConflict));
    // A negative delay is never a delay.
    TwrConfig h = minimal();
    h.timing.post_tx_rx_enable.value = Duration::from_nanos(-1);
    const ValidationReport v = validate(h, measured_caps());
    require_machine_readable(v);
    BOOST_REQUIRE(has(v, "timing.post_tx_rx_enable.ns", ConfigReason::NegativeValue));
}

BOOST_AUTO_TEST_CASE(roles_own_different_delays)
{
    // The initiator never sends a Response.
    TwrConfig c = minimal(Protocol::Ss, Role::Initiator);
    c.timing.poll_to_response.value = Duration::from_nanos(600'000);
    const ValidationReport v = validate(c, measured_caps());
    require_machine_readable(v);
    BOOST_REQUIRE(has(v, "timing.poll_to_response", ConfigReason::FieldConflict));

    // SS has no Final at all: neither the delay nor the RX window may exist.
    TwrConfig d = minimal(Protocol::Ss, Role::Initiator);
    d.timing.response_to_final.value = Duration::from_nanos(600'000);
    d.timeouts.final_rx_window.value = Duration::from_nanos(20'000);
    const ValidationReport w = validate(d, measured_caps());
    require_machine_readable(w);
    BOOST_REQUIRE(has(w, "timing.response_to_final", ConfigReason::FieldConflict));
    BOOST_REQUIRE(has(w, "timeouts.final_rx_window.ns", ConfigReason::FieldConflict));
    // The same Final RX window is legal for a DS responder, which does wait
    // for it.
    TwrConfig ds = minimal(Protocol::Ds, Role::Responder);
    BOOST_REQUIRE(ds.timeouts.final_rx_window.value.nanos() == 20'000);
    BOOST_REQUIRE(validate(ds, measured_caps()).ok());

    // The responder never sends a Final.
    TwrConfig e = minimal(Protocol::Ds, Role::Responder);
    e.timing.response_to_final.value = Duration::from_nanos(600'000);
    BOOST_REQUIRE(has(validate(e, measured_caps()), "timing.response_to_final",
                      ConfigReason::FieldConflict));

    // A Report message is out of scope in phase 1.
    TwrConfig f = minimal();
    f.timing.final_to_report.value = Duration::from_nanos(600'000);
    f.timeouts.report_rx_window.value = Duration::from_nanos(20'000);
    const ValidationReport x = validate(f, measured_caps());
    require_machine_readable(x);
    BOOST_REQUIRE(has(x, "timing.final_to_report", ConfigReason::OutOfScope));
    BOOST_REQUIRE(has(x, "timeouts.report_rx_window", ConfigReason::OutOfScope));
}

BOOST_AUTO_TEST_CASE(rx_window_must_fit_inside_the_rx_timeout)
{
    TwrConfig c = minimal(Protocol::Ds, Role::Initiator);
    c.timeouts.response_rx_window.value = Duration::from_nanos(50'000);
    c.timeouts.rx_timeout.value = Duration::from_nanos(40'000);
    const ValidationReport v = validate(c, measured_caps());
    require_machine_readable(v);
    BOOST_REQUIRE(has(v, "timeouts.response_rx_window.ns",
        ConfigReason::TimingOrderViolation));
}

BOOST_AUTO_TEST_CASE(whole_exchange_timeout_must_cover_the_whole_chain)
{
    TwrConfig c = minimal(Protocol::Ds, Role::Responder);
    c.timeouts.exchange_timeout.value = Duration::from_nanos(1'000); // far too short
    const ValidationReport v = validate(c, measured_caps());
    require_machine_readable(v);
    BOOST_REQUIRE(has(v, "timeouts.exchange_timeout.ns",
        ConfigReason::TimingOrderViolation));
}

BOOST_AUTO_TEST_CASE(reply_delay_must_cover_the_measured_tx_lead_time)
{
    TwrConfig c = minimal(Protocol::Ds, Role::Initiator);
    c.timing.min_tx_lead_time.value = Duration::from_nanos(600'000);
    const ValidationReport v = validate(c, measured_caps());
    require_machine_readable(v);
    const ConfigViolation* viol =
        v.find("timing.response_to_final.ns", ConfigReason::TimingBudgetInfeasible);
    BOOST_REQUIRE(viol != nullptr);
    // A schedulable deadline that cannot be met is a DEADLINE, not a soft
    // warning and not a reason to transmit late.
    BOOST_REQUIRE(viol->status == ExchangeStatus::DeadlineMissed);
    BOOST_REQUIRE(viol->requirement == "REQ-GR-04");
    // A zero lead time (unstated) is itself a rejection, not an assumption.
    TwrConfig d = minimal(Protocol::Ds, Role::Initiator);
    d.timing.min_tx_lead_time.value = Duration::from_nanos(0);
    const ValidationReport w = validate(d, measured_caps());
    require_machine_readable(w);
    BOOST_REQUIRE(has(w, "timing.min_tx_lead_time.ns", ConfigReason::ZeroValue));
}

BOOST_AUTO_TEST_CASE(quantise_duration_is_checked)
{
    bool ok = false;
    BOOST_REQUIRE_EQUAL(quantise_duration(Duration::from_nanos(600'000),
                                           kTwrNativeRateUc200Hz, ok),
                        442368);
    BOOST_REQUIRE(ok);
    (void)quantise_duration(Duration::from_nanos(1), 0.0, ok);
    BOOST_REQUIRE(!ok);
    (void)quantise_duration(Duration::from_nanos(1), -1.0, ok);
    BOOST_REQUIRE(!ok);
    // Duration::ticks_at() itself reports a bad tick rate.
    (void)Duration::from_nanos(1).ticks_at(0.0, ok);
    BOOST_REQUIRE(!ok);
    (void)Duration::from_nanos(1).ticks_at(-1.0, ok);
    BOOST_REQUIRE(!ok);
    (void)Duration::from_nanos(1)
        .ticks_at(std::numeric_limits<double>::quiet_NaN(), ok);
    BOOST_REQUIRE(!ok);
    BOOST_REQUIRE_EQUAL(Duration::from_nanos(1).ticks_at(1.0e9, ok), 1);
    BOOST_REQUIRE(ok);
}

// ===========================================================================
// I. Overrides on an immutable snapshot (REQ-API-03)
// ===========================================================================

BOOST_AUTO_TEST_CASE(overrides_apply_at_an_exchange_boundary)
{
    const Capabilities caps = measured_caps();
    const EffectiveConfig e = effective_config(minimal(Protocol::Ds, Role::Initiator),
        caps);
    TwrConfigSnapshot snap(e, caps);
    BOOST_REQUIRE(snap.ok());
    BOOST_REQUIRE_EQUAL(snap.in_flight(), 0u);

    PerMessageOverrides o;
    o.measurement_count = 5;
    o.measurement_interval_ns = 80'000'000;
    TwrConfigSnapshot next;
    const ValidationReport r = snap.set_overrides(o, next);
    BOOST_REQUIRE_MESSAGE(r.ok(), r.to_string());
    BOOST_REQUIRE(next.ok());
    BOOST_REQUIRE_EQUAL(next.get().effective.session.measurement_count, 5u);
    BOOST_REQUIRE_EQUAL(next.get().requested.session.measurement_count, 1u);
    // The original is untouched: a snapshot is a value, not a handle.
    BOOST_REQUIRE_EQUAL(snap.get().effective.session.measurement_count, 1u);
    // The requested side of the new snapshot is still the original request.
    BOOST_REQUIRE_EQUAL(next.get().effective.timing.response_to_final.value.nanos(),
        600'000);
}

BOOST_AUTO_TEST_CASE(overrides_are_validated_not_trusted)
{
    const Capabilities caps = measured_caps();
    TwrConfigSnapshot snap(effective_config(minimal(Protocol::Ds, Role::Initiator),
        caps), caps);
    PerMessageOverrides o;
    o.rx_timeout_ns = 10; // smaller than the 20 us RX window it must contain
    TwrConfigSnapshot next;
    const ValidationReport r = snap.set_overrides(o, next);
    require_machine_readable(r);
    BOOST_REQUIRE(has(r, "timeouts.response_rx_window.ns",
        ConfigReason::TimingOrderViolation));
    BOOST_REQUIRE(!next.ok());
}

BOOST_AUTO_TEST_CASE(mid_exchange_mutation_is_rejected)
{
    const Capabilities caps = measured_caps();
    TwrConfigSnapshot snap(effective_config(minimal(Protocol::Ds, Role::Initiator),
        caps), caps);
    const ValidationReport begin = snap.begin_exchange();
    BOOST_REQUIRE_MESSAGE(begin.ok(), begin.to_string());
    BOOST_REQUIRE_EQUAL(snap.in_flight(), 1u);

    PerMessageOverrides o;
    o.measurement_count = 99;
    o.measurement_interval_ns = 80'000'000; // >= one whole-exchange timeout
    TwrConfigSnapshot next;
    const ValidationReport r = snap.set_overrides(o, next);
    require_machine_readable(r);
    const ConfigViolation* viol = &r.violations.front();
    BOOST_REQUIRE(viol->reason == ConfigReason::MidExchangeMutation);
    BOOST_REQUIRE(viol->status == ExchangeStatus::ConfigRejected);
    BOOST_REQUIRE(viol->requirement == "REQ-API-03");
    BOOST_REQUIRE(!next.ok());

    // Synchronisation-affecting values are NOT overridable at all: the struct
    // simply has no field for them.
    BOOST_REQUIRE_EQUAL(sizeof(PerMessageOverrides), 7u * sizeof(int64_t));

    snap.end_exchange();
    BOOST_REQUIRE(snap.set_overrides(o, next).ok());
    BOOST_REQUIRE(next.ok());
}

BOOST_AUTO_TEST_CASE(second_concurrent_exchange_is_queue_full_through_the_snapshot)
{
    const Capabilities caps = measured_caps();
    TwrConfigSnapshot snap(effective_config(minimal(), caps), caps);
    BOOST_REQUIRE(snap.begin_exchange().ok());
    const ValidationReport second = snap.begin_exchange();
    require_machine_readable(second);
    BOOST_REQUIRE(second.violations.front().status == ExchangeStatus::QueueFull);
    snap.end_exchange();
    BOOST_REQUIRE(snap.begin_exchange().ok());
}

// ===========================================================================
// J. Calibration (REQ-CAL-01)
// ===========================================================================

BOOST_AUTO_TEST_CASE(calibration_is_applicable_exactly_once)
{
    TimestampCalibrationConfig cal = minimal().calibration;
    std::string why;
    BOOST_REQUIRE(apply_calibration_once(cal, 0, &why) == ExchangeStatus::Ok);
    BOOST_REQUIRE_EQUAL(cal.applied_count, 1u);
    BOOST_REQUIRE(apply_calibration_once(cal, 0,
        &why) == ExchangeStatus::CalibrationMissing);
    BOOST_REQUIRE(why.find("exactly once") != std::string::npos);
    // A config that already carries an applied calibration is not valid input.
    TwrConfig c = minimal();
    c.calibration = cal;
    const ValidationReport v = validate(c, measured_caps());
    require_machine_readable(v);
    const ConfigViolation* viol =
        v.find("calibration.applied_count", ConfigReason::CalibrationAlreadyApplied);
    BOOST_REQUIRE(viol != nullptr);
    BOOST_REQUIRE(viol->status == ExchangeStatus::CalibrationMissing);
}

BOOST_AUTO_TEST_CASE(calibration_missing_expired_and_mismatched)
{
    TimestampCalibrationConfig none = minimal().calibration;
    none.calibration_id.clear();
    std::string why;
    BOOST_REQUIRE(apply_calibration_once(none, 0,
        &why) == ExchangeStatus::CalibrationMissing);

    TimestampCalibrationConfig stale = minimal().calibration;
    stale.record.valid_until_monotonic_ns = 1000;
    BOOST_REQUIRE(apply_calibration_once(stale, 2000,
        &why) == ExchangeStatus::CalibrationExpired);
    BOOST_REQUIRE(why.find("expired") != std::string::npos);
    BOOST_REQUIRE(apply_calibration_once(stale, 500, &why) == ExchangeStatus::Ok);

    // An uncalibrated absolute range may not be claimed.
    TwrConfig a = minimal();
    a.calibration.calibration_id.clear();
    a.calibration.record.calibration_id.clear();
    const ValidationReport va = validate(a, measured_caps());
    require_machine_readable(va);
    BOOST_REQUIRE(va.find("calibration.calibration_id",
        ConfigReason::CalibrationMissing) !=
                  nullptr);

    // Record must describe THIS channel / rate / profile / gain.
    TwrConfig b = minimal();
    b.calibration.record.channel = 9u;
    BOOST_REQUIRE(has(validate(b, measured_caps()), "calibration.record.channel",
                      ConfigReason::CalibrationMismatch));
    TwrConfig d = minimal();
    d.calibration.record.native_sample_rate_hz = kTwrNativeRateCg400Hz;
    BOOST_REQUIRE(has(validate(d, measured_caps()),
                      "calibration.record.native_sample_rate_hz",
                      ConfigReason::CalibrationMismatch));
    TwrConfig e = minimal();
    e.calibration.record.profile_version = "other-profile/9";
    BOOST_REQUIRE(has(validate(e, measured_caps()),
        "calibration.record.profile_version",
                      ConfigReason::CalibrationMismatch));
    TwrConfig f = minimal();
    f.calibration.record.gain_db = Opt<double>(10.0);
    BOOST_REQUIRE(has(validate(f, measured_caps()), "calibration.record.gain_db",
                      ConfigReason::CalibrationMismatch));
    TwrConfig g = minimal();
    g.meta.calibration_version = "different-cal/0";
    BOOST_REQUIRE(has(validate(g, measured_caps()), "meta.calibration_version",
                      ConfigReason::CalibrationMismatch));
}

BOOST_AUTO_TEST_CASE(nanoseconds_seconds_and_native_ticks_stay_separate)
{
    TwrConfig c = minimal();
    c.calibration.link_delay_unit = TimeUnit::Seconds;
    BOOST_REQUIRE(has(validate(c, measured_caps()), "calibration.link_delay_unit",
                      ConfigReason::UnitMismatch));
    TwrConfig d = minimal();
    d.calibration.link_delay_unit = TimeUnit::NativeTicks;
    BOOST_REQUIRE(has(validate(d, measured_caps()), "calibration.link_delay_unit",
                      ConfigReason::FieldConflict));
    TwrConfig e = minimal();
    e.calibration.link_delay_unit = TimeUnit::NativeTicks;
    e.calibration.tx_link_delay_native_ticks = 88;
    e.calibration.rx_link_delay_native_ticks = 133;
    BOOST_REQUIRE(validate(e, measured_caps()).ok());
    // The native-tick fields do not change the nanosecond fields.
    BOOST_REQUIRE_EQUAL(e.calibration.tx_link_delay.nanos(), 120);
    // A negative delay in either unit is rejected.
    TwrConfig f = minimal();
    f.calibration.rx_link_delay_native_ticks = -1;
    BOOST_REQUIRE(has(validate(f, measured_caps()),
                      "calibration.tx_link_delay_native_ticks", ConfigReason::NegativeValue));
}

// ===========================================================================
// K. TX/RX power concepts and vendor items
// ===========================================================================

BOOST_AUTO_TEST_CASE(tx_gain_db_iq_amplitude_and_dbm_are_three_distinct_fields)
{
    // gain dB is the authority.
    TwrConfig a = minimal();
    BOOST_REQUIRE(validate(a, measured_caps()).ok());

    // IQ amplitude as the authority: gain must not be silently reused.
    TwrConfig b = minimal();
    b.tx.gain_db.reset();
    b.tx.iq_amplitude = Opt<double>(0.5);
    b.tx.power_policy = TxPowerPolicy::IqAmplitude;
    BOOST_REQUIRE(validate(b, measured_caps()).ok());

    // Calibrated dBm as the authority, with the gain it was measured at.
    TwrConfig c = minimal();
    c.tx.calibrated_tx_power_dbm = Opt<double>(-12.5);
    c.tx.power_policy = TxPowerPolicy::CalibratedDbm;
    BOOST_REQUIRE(validate(c, measured_caps()).ok());

    // Policy names a field that is absent: rejected, not defaulted.
    TwrConfig d = minimal();
    d.tx.power_policy = TxPowerPolicy::IqAmplitude;
    BOOST_REQUIRE(has(validate(d, measured_caps()), "tx.power_policy",
                      ConfigReason::FieldConflict));
    // leave_untouched with a field set: also a conflict.
    TwrConfig e = minimal();
    e.tx.power_policy = TxPowerPolicy::LeaveUntouched;
    BOOST_REQUIRE(has(validate(e, measured_caps()), "tx.power_policy",
                      ConfigReason::FieldConflict));
    // dBm without the gain it was measured at is meaningless.
    TwrConfig f = minimal();
    f.tx.gain_db.reset();
    f.tx.calibrated_tx_power_dbm = Opt<double>(-12.5);
    f.tx.power_policy = TxPowerPolicy::CalibratedDbm;
    BOOST_REQUIRE(has(validate(f, measured_caps()), "tx.gain_db",
        ConfigReason::FieldConflict));
    // Ranges are ranges, not clamps.
    TwrConfig g = minimal();
    g.tx.iq_amplitude = Opt<double>(2.0);
    g.tx.gain_db.reset();
    g.tx.power_policy = TxPowerPolicy::IqAmplitude;
    BOOST_REQUIRE(has(validate(g, measured_caps()), "tx.iq_amplitude",
        ConfigReason::OutOfRange));
}

BOOST_AUTO_TEST_CASE(vendor_power_word_is_a_separate_named_field)
{
    TwrConfig a = minimal();
    a.tx.vendor_power_word = Opt<uint32_t>(0x1234u);
    const ValidationReport v = validate(a, measured_caps());
    require_machine_readable(v);
    const ConfigViolation* viol = v.find("tx.vendor_power_word",
        ConfigReason::CalibrationMismatch);
    BOOST_REQUIRE(viol != nullptr);
    BOOST_REQUIRE(viol->message.find("backend capability id") != std::string::npos);

    TwrConfig b = minimal();
    b.tx.vendor_power_word = Opt<uint32_t>(0x1234u);
    b.tx.vendor_power_word_backend = "synthetic-backend/0";
    BOOST_REQUIRE(validate(b, measured_caps()).ok());
    // A backend id without a word is a conflict.
    TwrConfig c = minimal();
    c.tx.vendor_power_word_backend = "synthetic-backend/0";
    BOOST_REQUIRE(has(validate(c, measured_caps()), "tx.vendor_power_word_backend",
                      ConfigReason::FieldConflict));
}

BOOST_AUTO_TEST_CASE(vendor_pac_needs_a_backend_capability_and_a_real_step)
{
    TwrConfig a = minimal();
    a.rx.vendor_pac_value = Opt<uint32_t>(5u);
    const ValidationReport v = validate(a, measured_caps());
    require_machine_readable(v);
    BOOST_REQUIRE(v.find("rx.vendor_pac_value",
        ConfigReason::CalibrationMismatch) != nullptr);

    TwrConfig b = minimal();
    b.rx.vendor_pac_value = Opt<uint32_t>(5u);
    b.rx.vendor_pac_backend = "synthetic-backend/0";
    const ValidationReport w = validate(b, measured_caps());
    require_machine_readable(w);
    // A PAC value may not be mapped onto an arbitrary software detection step.
    BOOST_REQUIRE(w.find("rx.vendor_pac_applied_step",
        ConfigReason::CalibrationMismatch) !=
                  nullptr);

    TwrConfig c = minimal();
    c.rx.vendor_pac_value = Opt<uint32_t>(5u);
    c.rx.vendor_pac_backend = "synthetic-backend/0";
    c.rx.vendor_pac_applied_step = Opt<double>(0.25);
    BOOST_REQUIRE(validate(c, measured_caps()).ok());
    // A zero step is not a step.
    TwrConfig d = c;
    d.rx.vendor_pac_applied_step = Opt<double>(0.0);
    BOOST_REQUIRE(has(validate(d, measured_caps()), "rx.vendor_pac_applied_step",
                      ConfigReason::OutOfRange));
    // A step without a PAC value is a conflict.
    TwrConfig e = minimal();
    e.rx.vendor_pac_applied_step = Opt<double>(0.25);
    BOOST_REQUIRE(has(validate(e, measured_caps()), "rx.vendor_pac_applied_step",
                      ConfigReason::FieldConflict));
}

BOOST_AUTO_TEST_CASE(rx_agc_and_gain_and_bandwidth_are_checked)
{
    TwrConfig a = minimal();
    a.rx.agc = AgcMode::Manual;
    a.rx.gain_db.reset();
    BOOST_REQUIRE(has(validate(a, measured_caps()), "rx.gain_db",
        ConfigReason::FieldConflict));
    TwrConfig b = minimal();
    b.rx.bandwidth_hz = Opt<double>(0.0);
    BOOST_REQUIRE(has(validate(b, measured_caps()), "rx.bandwidth_hz",
        ConfigReason::OutOfRange));
    TwrConfig c = minimal();
    c.rx.first_path_index = 30u;
    c.rx.first_path_window = 30u;
    BOOST_REQUIRE(has(validate(c, measured_caps()), "rx.first_path_index",
                      ConfigReason::IndexOutOfRange));
    TwrConfig d = minimal();
    d.rx.first_path_window = 0u;
    BOOST_REQUIRE(has(validate(d, measured_caps()), "rx.first_path_window",
        ConfigReason::ZeroValue));
}

// ===========================================================================
// L. Diagnostics bounds
// ===========================================================================

BOOST_AUTO_TEST_CASE(diagnostics_are_bounded)
{
    TwrConfig a = minimal();
    a.diagnostics.cir_capture_enabled = true;
    a.diagnostics.cir_capture_max_bytes = 1ull << 30; // 1 GiB
    a.diagnostics.cir_capture_stride = 16u;
    BOOST_REQUIRE(has(validate(a, measured_caps()), "diagnostics.cir_capture_max_bytes",
                      ConfigReason::OverCapacity));

    TwrConfig b = minimal();
    b.diagnostics.short_iq_enabled = true;
    BOOST_REQUIRE(has(validate(b, measured_caps()), "diagnostics.short_iq_max_bytes",
                      ConfigReason::ZeroValue));
    TwrConfig c = minimal();
    c.diagnostics.raw_frame_dump = true;
    c.diagnostics.raw_frame_max_bytes = 1u << 20;
    BOOST_REQUIRE(validate(c, measured_caps()).ok());
    // A budget for a disabled diagnostic is a conflict, not dead state.
    TwrConfig d = minimal();
    d.diagnostics.raw_frame_max_bytes = 1u << 20;
    BOOST_REQUIRE(has(validate(d, measured_caps()), "diagnostics.raw_frame_max_bytes",
                      ConfigReason::FieldConflict));
    TwrConfig e = minimal();
    e.diagnostics.result_queue_capacity = 0u;
    BOOST_REQUIRE(has(validate(e, measured_caps()), "diagnostics.result_queue_capacity",
                      ConfigReason::ZeroValue));
    TwrConfig f = minimal();
    f.diagnostics.event_queue_capacity = kTwrMaxQueueEntries + 1u;
    BOOST_REQUIRE(has(validate(f, measured_caps()), "diagnostics.event_queue_capacity",
                      ConfigReason::OverCapacity));
}

BOOST_AUTO_TEST_CASE(diagnostic_io_may_not_run_on_the_realtime_thread)
{
    TwrConfig c = minimal();
    c.diagnostics.io_on_realtime_thread = true;
    const ValidationReport v = validate(c, measured_caps());
    require_machine_readable(v);
    const ConfigViolation* viol =
        v.find("diagnostics.io_on_realtime_thread", ConfigReason::FieldConflict);
    BOOST_REQUIRE(viol != nullptr);
    BOOST_REQUIRE(viol->message.find("realtime") != std::string::npos);
}

// ===========================================================================
// M. JSON import / export
// ===========================================================================

BOOST_AUTO_TEST_CASE(json_round_trip_is_stable)
{
    for (int p = 0; p < 2; ++p) {
        for (int r = 0; r < 2; ++r) {
            const Protocol proto = p == 0 ? Protocol::Ss : Protocol::Ds;
            const Role role = r == 0 ? Role::Initiator : Role::Responder;
            const TwrConfig c = minimal(proto, role);

            std::string js;
            std::string err;
            BOOST_REQUIRE_MESSAGE(to_json_string(c, js, err), err);
            BOOST_REQUIRE(!js.empty());

            TwrConfig back;
            const ValidationReport rep = from_json_string(js, back);
            BOOST_REQUIRE_MESSAGE(rep.ok(), rep.to_string());

            std::string js2;
            std::string err2;
            BOOST_REQUIRE_MESSAGE(to_json_string(back, js2, err2), err2);
            BOOST_REQUIRE_MESSAGE(js == js2, "the second write differs from the first");
            BOOST_REQUIRE_EQUAL(config_hash(c), config_hash(back));

            // The re-imported config still validates against the whitelist.
            const Capabilities caps = measured_caps();
            BOOST_REQUIRE_MESSAGE(validate(back, caps).ok(),
                                  validate(back, caps).to_string());
        }
    }
}

BOOST_AUTO_TEST_CASE(json_preserves_64bit_integer_precision)
{
    TwrConfig c = minimal();
    // Beyond 2^53, so a naive double round trip would corrupt this.
    c.timeouts.exchange_timeout.value = Duration::from_nanos(4'000'000'000'000'000'001LL);
    c.calibration.tx_link_delay = Duration::from_nanos(9'007'199'254'740'993LL);
    c.calibration.record.valid_until_monotonic_ns = 9'223'372'036'854'775'000LL;
    c.session.session_id = 0xffffffffu;

    std::string js;
    std::string err;
    BOOST_REQUIRE_MESSAGE(to_json_string(c, js, err), err);
    // Large integers are written as decimal STRINGS, so no JSON reader can
    // silently round them through a double.
    BOOST_REQUIRE(js.find("\"4000000000000000001\"") != std::string::npos);
    BOOST_REQUIRE(js.find("\"9007199254740993\"") != std::string::npos);
    BOOST_REQUIRE(js.find("\"9223372036854775000\"") != std::string::npos);
    // Small integers stay unquoted numbers.
    BOOST_REQUIRE(js.find("\"session_id\": 4294967295") != std::string::npos);

    TwrConfig back;
    const ValidationReport rep = from_json_string(js, back);
    BOOST_REQUIRE_MESSAGE(rep.ok(), rep.to_string());
    BOOST_REQUIRE_EQUAL(back.timeouts.exchange_timeout.value.nanos(),
                        4'000'000'000'000'000'001LL);
    BOOST_REQUIRE_EQUAL(back.calibration.tx_link_delay.nanos(),
        9'007'199'254'740'993LL);
    BOOST_REQUIRE_EQUAL(back.calibration.record.valid_until_monotonic_ns,
                        9'223'372'036'854'775'000LL);
    BOOST_REQUIRE_EQUAL(back.session.session_id, 0xffffffffu);
    BOOST_REQUIRE_EQUAL(config_hash(c), config_hash(back));

    // Both the quoted and the unquoted form are accepted for an int field.
    int64_t parsed = 0;
    BOOST_REQUIRE(ConfigJsonReader::parse_int64_strict("4000000000000000001", parsed));
    BOOST_REQUIRE_EQUAL(parsed, 4'000'000'000'000'000'001LL);
    BOOST_REQUIRE(ConfigJsonReader::parse_int64_strict("-42", parsed));
    BOOST_REQUIRE_EQUAL(parsed, -42);
    BOOST_REQUIRE(!ConfigJsonReader::parse_int64_strict("42.0", parsed));
    BOOST_REQUIRE(!ConfigJsonReader::parse_int64_strict("4e9", parsed));
    BOOST_REQUIRE(!ConfigJsonReader::parse_int64_strict("", parsed));
    BOOST_REQUIRE(!ConfigJsonReader::parse_int64_strict("0x10", parsed));
    BOOST_REQUIRE(!ConfigJsonReader::parse_int64_strict(" 42", parsed));
    BOOST_REQUIRE(!ConfigJsonReader::parse_int64_strict("42 ", parsed));
    // An integer literal outside int64 is not silently wrapped: the parser
    // keeps it as a number and the integer reader rejects it.
    {
        json::Value v;
        std::string perr;
        BOOST_REQUIRE(json::parse("99999999999999999999999", v, perr));
        BOOST_REQUIRE(v.type == json::Type::Double);
    }
}

BOOST_AUTO_TEST_CASE(json_doubles_round_trip_exactly)
{
    TwrConfig c = minimal();
    c.phy.center_frequency_hz = 6489.6e6;
    c.radio.native_sample_rate_hz = 491520000.0;
    c.rx.detection_threshold = 0.1 + 0.2; // 0.30000000000000004
    c.calibration.first_path_algorithm = FirstPathAlgorithm::LeadingEdge;
    std::string js;
    std::string err;
    BOOST_REQUIRE_MESSAGE(to_json_string(c, js, err), err);
    TwrConfig back;
    BOOST_REQUIRE(from_json_string(js, back).ok());
    BOOST_REQUIRE_EQUAL(back.rx.detection_threshold, 0.1 + 0.2);
    BOOST_REQUIRE_EQUAL(config_hash(c), config_hash(back));
}

BOOST_AUTO_TEST_CASE(json_unknown_key_is_rejected)
{
    TwrConfig c = minimal();
    std::string js;
    std::string err;
    BOOST_REQUIRE(to_json_string(c, js, err));
    // Splice an unknown key into the phy group.
    const std::string needle = "\"phy\": {";
    const size_t at = js.find(needle);
    BOOST_REQUIRE(at != std::string::npos);
    const std::string broken =
        js.substr(0, at + needle.size()) + "\n    \"turbo_mode\": 1," + js.substr(at +
            needle.size());
    TwrConfig back;
    const ValidationReport rep = from_json_string(broken, back);
    require_machine_readable(rep);
    BOOST_REQUIRE(rep.find("phy.turbo_mode", ConfigReason::UnknownKey) != nullptr);
}

BOOST_AUTO_TEST_CASE(json_missing_key_is_rejected)
{
    TwrConfig c = minimal();
    std::string js;
    std::string err;
    BOOST_REQUIRE(to_json_string(c, js, err));
    const size_t at = js.find("\"channel\": 5");
    BOOST_REQUIRE(at != std::string::npos);
    const std::string broken = js.substr(0, at) + js.substr(js.find(',', at) + 1);
    TwrConfig back;
    const ValidationReport rep = from_json_string(broken, back);
    require_machine_readable(rep);
    BOOST_REQUIRE(rep.find("phy.channel", ConfigReason::MissingKey) != nullptr);
}

BOOST_AUTO_TEST_CASE(json_type_mismatch_is_rejected_not_coerced)
{
    TwrConfig c = minimal();
    std::string js;
    std::string err;
    BOOST_REQUIRE(to_json_string(c, js, err));
    // A string where a number belongs.
    TwrConfig back;
    {
        std::string bad = js;
        const size_t at = bad.find("\"detection_threshold\": ");
        BOOST_REQUIRE(at != std::string::npos);
        const size_t val = at + std::string("\"detection_threshold\": ").size();
        const size_t end = bad.find(',', val);
        bad = bad.substr(0, val) + "\"0.3\"" + bad.substr(end);
        const ValidationReport rep = from_json_string(bad, back);
        require_machine_readable(rep);
        BOOST_REQUIRE(rep.find("rx.detection_threshold",
            ConfigReason::TypeMismatch) != nullptr);
    }
    // A float where an integer belongs is NEVER truncated silently.
    {
        std::string bad = js;
        const size_t at = bad.find("\"session_id\": ");
        BOOST_REQUIRE(at != std::string::npos);
        const size_t val = at + std::string("\"session_id\": ").size();
        const size_t end = bad.find(',', val);
        bad = bad.substr(0, val) + "2863311530.7" + bad.substr(end);
        TwrConfig b2;
        const ValidationReport rep = from_json_string(bad, b2);
        require_machine_readable(rep);
        BOOST_REQUIRE(rep.find("session.session_id",
            ConfigReason::TypeMismatch) != nullptr);
    }
    // A missing group entirely.
    {
        TwrConfig b3;
        const ValidationReport rep = from_json_string("{\"meta\": {}}", b3);
        require_machine_readable(rep);
        BOOST_REQUIRE(rep.find("session", ConfigReason::MissingKey) != nullptr);
        BOOST_REQUIRE(rep.find("phy", ConfigReason::MissingKey) != nullptr);
    }
}

BOOST_AUTO_TEST_CASE(json_unknown_enum_value_is_rejected)
{
    TwrConfig c = minimal();
    std::string js;
    std::string err;
    BOOST_REQUIRE(to_json_string(c, js, err));
    std::string bad = js;
    const size_t at = bad.find("\"protocol\": \"ss\"");
    BOOST_REQUIRE(at != std::string::npos);
    const size_t val = at + std::string("\"protocol\": ").size();
    bad = bad.substr(0, val) + "\"sideways\"" + bad.substr(val +
        std::string("\"ss\"").size());
    TwrConfig back;
    const ValidationReport rep = from_json_string(bad, back);
    require_machine_readable(rep);
    BOOST_REQUIRE(rep.find("session.protocol",
        ConfigReason::UnknownEnumValue) != nullptr);
}

namespace {

// Replaces the first `"key": <old>` literal with `"key": <new>`.
std::string json_set(const std::string& js, const std::string& key,
                     const std::string& old_value, const std::string& new_value)
{
    const std::string needle = "\"" + key + "\": " + old_value;
    const size_t at = js.find(needle);
    if (at == std::string::npos)
        return std::string();
    return js.substr(0, at) + "\"" + key + "\": " + new_value +
           js.substr(at + needle.size());
}

} // namespace

// N01: the reader assigned through `static_cast<uintN_t>(get_i64(...))`, which
// WRAPS an out-of-range JSON number instead of refusing it: channel 261 became
// channel 5, session 2^32+1 became session 1, local_address -1 became the
// reserved address 0xffff.  The validator then saw the wrapped value, so two
// different documents produced one runtime configuration and the value the
// document actually carried was lost.  A cast is not a specification.
BOOST_AUTO_TEST_CASE(json_unsigned_fields_refuse_out_of_range_instead_of_wrapping)
{
    TwrConfig c = minimal();
    std::string js;
    std::string err;
    BOOST_REQUIRE_MESSAGE(to_json_string(c, js, err), err);

    struct Row {
        const char* key;
        long long from;
        const char* to;
        const char* field;
    };
    // The second entry of each uint8/16/32 group is one MODULUS past a legal
    // value, which a mask would accept by landing back INSIDE the range -- the
    // case that makes "one past the top" insufficient on its own.
    const Row rows[] = {
        {"channel", c.phy.channel, "256", "phy.channel"},
        {"channel", c.phy.channel, "261", "phy.channel"},
        {"session_id", c.session.session_id, "4294967297", "session.session_id"},
        {"local_address", c.session.local_address, "65537", "session.local_address"},
        {"local_address", c.session.local_address, "-1", "session.local_address"},
        {"pan_id", c.session.pan_id, "65536", "session.pan_id"},
        {"sequence", c.session.sequence, "256", "session.sequence"},
        {"preamble_symbols", c.phy.preamble_symbols, "65536", "phy.preamble_symbols"},
        {"mac_header_bytes", c.frame.geometry.mac_header_bytes, "65550",
         "frame.geometry.mac_header_bytes"},
        {"port", c.tx.port, "256", "tx.port"},
        {"port", c.tx.port, "-1", "tx.port"},
    };
    for (const Row& r : rows) {
        const std::string doc = json_set(js, r.key, std::to_string(r.from), r.to);
        BOOST_REQUIRE_MESSAGE(
            !doc.empty(), std::string("the document has no \"") + r.key +
                              "\": " + std::to_string(r.from));
        TwrConfig back;
        const ValidationReport rep = from_json_string(doc, back);
        BOOST_REQUIRE_MESSAGE(!rep.ok(),
            std::string(r.field) + " = " + r.to + " was ACCEPTED");
        BOOST_REQUIRE_MESSAGE(
            rep.find(r.field, ConfigReason::OutOfRange) != nullptr,
            std::string(r.field) + " = " + r.to + ": " + rep.to_string());
    }

    // The BOUNDARY is inclusive: the width maximum is still read as itself. A
    // range check that rejected the maximum would be as wrong as one that
    // wrapped it.
    const std::string at_max =
        json_set(js, "sequence", std::to_string(c.session.sequence), "255");
    BOOST_REQUIRE(!at_max.empty());
    TwrConfig back;
    const ValidationReport rep = from_json_string(at_max, back);
    BOOST_REQUIRE_MESSAGE(rep.ok(), rep.to_string());
    BOOST_REQUIRE_EQUAL(back.session.sequence, 255u);
}

BOOST_AUTO_TEST_CASE(json_carries_the_phr_rate_and_frame_profile_contract)
{
    const TwrConfig c = minimal();
    std::string js;
    std::string err;
    BOOST_REQUIRE_MESSAGE(to_json_string(c, js, err), err);

    // The PHR rate is written in the PHR-rate domain, and the frame profile
    // is a named id.  A document that used a payload-rate spelling for the
    // PHR was describing a 6.81 Mb/s PHR, which this modulator cannot
    // produce.
    BOOST_REQUIRE(js.find("\"phr_rate\": \"850k\"") != std::string::npos);
    BOOST_REQUIRE(js.find("\"data_rate\": \"6p8m\"") != std::string::npos);
    BOOST_REQUIRE(js.find("\"frame_profile\": \"frame_v1\"") != std::string::npos);

    // A payload-rate spelling in the PHR-rate field is now an
    // UnknownEnumValue, not a silently reinterpreted 850k.
    {
        const std::string bad = json_set(js, "phr_rate", "\"850k\"", "\"6p8m\"");
        BOOST_REQUIRE(!bad.empty());
        TwrConfig back;
        const ValidationReport rep = from_json_string(bad, back);
        require_machine_readable(rep);
        BOOST_REQUIRE(rep.find("phy.phr_rate", ConfigReason::UnknownEnumValue) !=
                      nullptr);
    }
    // Same for a frame profile this build has no codec for.
    {
        const std::string bad =
            json_set(js, "frame_profile", "\"frame_v1\"", "\"frame_v2\"");
        BOOST_REQUIRE(!bad.empty());
        TwrConfig back;
        const ValidationReport rep = from_json_string(bad, back);
        require_machine_readable(rep);
        BOOST_REQUIRE(rep.find("frame.frame_profile", ConfigReason::UnknownEnumValue) !=
                      nullptr);
    }
    // A geometry that disagrees with the codec survives the IMPORT (the
    // reader is not the authority) and is refused by the validator, naming
    // the field.  That split is deliberate: the JSON says what was claimed,
    // the validator says whether it is true.
    {
        const std::string bad = json_set(js, "mac_header_bytes", "14", "7");
        BOOST_REQUIRE(!bad.empty());
        TwrConfig back;
        BOOST_REQUIRE(from_json_string(bad, back).ok());
        BOOST_REQUIRE_EQUAL(back.frame.geometry.mac_header_bytes, 7u);
        const ValidationReport v = validate(back, measured_caps());
        require_machine_readable(v);
        BOOST_REQUIRE(v.first_for_field("frame.geometry.mac_header_bytes") != nullptr);
    }
    // An out-of-wire-range session id survives the import as the 32-bit value
    // the operator wrote and is then refused -- never narrowed on the way in.
    {
        const std::string bad = json_set(js, "session_id", "4660", "4294967295");
        BOOST_REQUIRE(!bad.empty());
        TwrConfig back;
        BOOST_REQUIRE(from_json_string(bad, back).ok());
        BOOST_REQUIRE_EQUAL(back.session.session_id, 0xffffffffu);
        const ValidationReport v = validate(back, measured_caps());
        require_machine_readable(v);
        BOOST_REQUIRE(v.find("session.session_id", ConfigReason::OutOfRange) != nullptr);
    }
    // frame_profile is part of the schema, so omitting it is a MissingKey and
    // not a silent default.
    {
        const size_t at = js.find("\"frame_profile\": \"frame_v1\"");
        BOOST_REQUIRE(at != std::string::npos);
        const size_t comma = js.find(',', at);
        const std::string bad = js.substr(0, at) + js.substr(comma + 1);
        TwrConfig back;
        const ValidationReport rep = from_json_string(bad, back);
        require_machine_readable(rep);
        BOOST_REQUIRE(rep.find("frame.frame_profile", ConfigReason::MissingKey) != nullptr);
    }
}

BOOST_AUTO_TEST_CASE(json_malformed_documents_are_rejected)
{
    const char* bad[] = { "",       "{",       "[]",     "null",     "{\"a\":}",
                          "{\"a\": 1,}",  "{'a': 1}", "{\"a\": 01}",
                          "{\"a\": 1.}",   "{\"a\": 1e}", "{\"a\": \"unterminated" };
    for (const char* text : bad) {
        TwrConfig back;
        const ValidationReport rep = from_json_string(text, back);
        require_machine_readable(rep);
        BOOST_REQUIRE_MESSAGE(rep.has(ConfigReason::MalformedJson) ||
                                   rep.has(ConfigReason::TypeMismatch),
                              std::string("accepted a malformed document: ") + text);
    }
    // Deep nesting is refused rather than blowing the stack.
    std::string deep;
    for (int i = 0; i < 200; ++i)
        deep += "[";
    for (int i = 0; i < 200; ++i)
        deep += "]";
    TwrConfig back;
    BOOST_REQUIRE(!from_json_string(deep, back).ok());
}

BOOST_AUTO_TEST_CASE(json_effective_config_export_carries_requested_effective_and_reasons)
{
    const Capabilities caps = measured_caps();
    const EffectiveConfig e = effective_config(minimal(Protocol::Ds, Role::Initiator),
        caps);
    std::string js;
    std::string err;
    BOOST_REQUIRE_MESSAGE(to_json_string(e, js, err), err);
    BOOST_REQUIRE(js.find("\"config_hash\": \"fnv1a64:") != std::string::npos);
    BOOST_REQUIRE(js.find("\"profile_version\": \"qa-synthetic-frame/0\"") != std::string::npos);
    BOOST_REQUIRE(js.find("\"calibration_version\": \"qa-synthetic-cal/0\"") !=
                  std::string::npos);
    BOOST_REQUIRE(js.find("\"changes\"") != std::string::npos);
    BOOST_REQUIRE(js.find("\"violations\"") != std::string::npos);
    BOOST_REQUIRE(js.find("\"quantised\"") != std::string::npos);
    BOOST_REQUIRE(js.find("\"response_to_final_ticks\": 442368") != std::string::npos);
    BOOST_REQUIRE(js.find("\"frame_budget\"") != std::string::npos);
    // It parses as JSON.
    json::Value root;
    std::string perr;
    BOOST_REQUIRE_MESSAGE(json::parse(js, root, perr), perr);
    BOOST_REQUIRE(root.type == json::Type::Object);
    // A rejected snapshot lists its reasons.
    TwrConfig bad = minimal();
    bad.frame.sts_mode = StsMode::Sp256;
    const EffectiveConfig re = effective_config(bad, caps);
    std::string rjs;
    std::string rerr;
    BOOST_REQUIRE_MESSAGE(to_json_string(re, rjs, rerr), rerr);
    BOOST_REQUIRE(rjs.find("\"reason\": \"out_of_scope\"") != std::string::npos);
    BOOST_REQUIRE(rjs.find("\"status\": \"unsupported\"") != std::string::npos);
    BOOST_REQUIRE(rjs.find("\"requirement\": \"REQ-SCOPE-04\"") != std::string::npos);
}

// ===========================================================================
// N. Report quality
// ===========================================================================

BOOST_AUTO_TEST_CASE(multiple_violations_are_reported_together)
{
    TwrConfig c = minimal();
    c.frame.sts_mode = StsMode::Sp128;
    c.frame.sts_length_symbols = 128u;
    c.radio.native_sample_rate_hz = 998.4e6; // the WORK rate, not a native rate
    c.phy.tx_preamble_code = 7u;
    c.phy.rx_preamble_code = 7u;
    c.session.max_in_flight_exchanges = 4u;
    c.rx.detection_threshold = 5.0;
    c.diagnostics.io_on_realtime_thread = true;

    const ValidationReport v = validate(c, measured_caps());
    require_machine_readable(v);
    BOOST_REQUIRE_MESSAGE(v.violations.size() >= 6,
                          "expected several violations, got: " + v.to_string());
    BOOST_REQUIRE(has(v, "frame.sts_mode", ConfigReason::OutOfScope));
    BOOST_REQUIRE(has(v, "radio.native_sample_rate_hz", ConfigReason::Unsupported));
    BOOST_REQUIRE(has(v, "phy.tx_preamble_code", ConfigReason::Unsupported));
    BOOST_REQUIRE(has(v, "session.max_in_flight_exchanges",
                      ConfigReason::InFlightNotSupported));
    BOOST_REQUIRE(has(v, "rx.detection_threshold", ConfigReason::OutOfRange));
    BOOST_REQUIRE(has(v, "diagnostics.io_on_realtime_thread",
        ConfigReason::FieldConflict));
    // Each is a distinct (field, reason) pair -- no silent collapse.
    std::vector<std::string> seen;
    for (const auto& viol : v.violations)
        seen.push_back(viol.field + "|" + config_reason_to_string(viol.reason));
    std::sort(seen.begin(), seen.end());
    BOOST_REQUIRE(std::unique(seen.begin(), seen.end()) == seen.end());
    // And every message is printable.
    const std::vector<std::string> msgs = v.messages();
    BOOST_REQUIRE_EQUAL(msgs.size(), v.violations.size());
    for (const auto& m : msgs)
        BOOST_REQUIRE(m.find("[") != std::string::npos);
}

BOOST_AUTO_TEST_CASE(report_lookup_helpers)
{
    TwrConfig c = minimal();
    c.frame.sts_mode = StsMode::Sp512;
    c.frame.sts_length_symbols = 512u;
    const ValidationReport v = validate(c, measured_caps());
    require_machine_readable(v);
    BOOST_REQUIRE(v.has(ConfigReason::OutOfScope));
    BOOST_REQUIRE(v.has_field("frame.sts_mode"));
    BOOST_REQUIRE(!v.has_field("frame.nothing"));
    BOOST_REQUIRE(v.find("frame.sts_mode", ConfigReason::FieldConflict) == nullptr);
    BOOST_REQUIRE(v.first_for_field("frame.sts_mode") != nullptr);
    BOOST_REQUIRE(v.first_for_field("nope") == nullptr);
    // The report is ordered by section, so the first reason of this config is
    // the STS one.
    BOOST_REQUIRE(v.first_status() == ExchangeStatus::Unsupported);
    BOOST_REQUIRE(v.to_string() != "ok");
    // A clean report is printable as "ok".
    BOOST_REQUIRE_EQUAL(validate(minimal(), measured_caps()).to_string(),
        std::string("ok"));
}

BOOST_AUTO_TEST_CASE(every_default_constructed_config_is_rejected)
{
    // A value-initialised config must not be silently accepted: the default is
    // a starting point for the operator, never a valid profile.
    const TwrConfig c;
    const ValidationReport v = validate(c, measured_caps());
    require_machine_readable(v);
    BOOST_REQUIRE_MESSAGE(v.violations.size() >= 8, v.to_string());
    BOOST_REQUIRE(has(v, "session.local_address", ConfigReason::FieldConflict));
    BOOST_REQUIRE(has(v, "session.pan_id", ConfigReason::EmptyValue));
    BOOST_REQUIRE(has(v, "phy.channel", ConfigReason::Unsupported) ||
                      has(v, "phy.tx_preamble_code", ConfigReason::ZeroValue));
    BOOST_REQUIRE(has(v, "frame.mac_psdu_bytes", ConfigReason::ZeroValue) ||
                      has(v, "frame.geometry", ConfigReason::EmptyValue));
    BOOST_REQUIRE(has(v, "timing.post_tx_rx_enable.domain", ConfigReason::EmptyValue));
    BOOST_REQUIRE(has(v, "calibration.calibration_id",
        ConfigReason::CalibrationMissing));
    BOOST_REQUIRE(has(v, "diagnostics.result_queue_capacity", ConfigReason::ZeroValue));
    // And it produces no effective config.
    const EffectiveConfig e = effective_config(c, measured_caps());
    BOOST_REQUIRE(!e.ok);
    BOOST_REQUIRE(e.changes.empty());
    BOOST_REQUIRE(e.tick_rate_hz == 0.0);
}

BOOST_AUTO_TEST_CASE(enum_helpers_round_trip_for_every_new_enum)
{
    PrfClass p = PrfClass::Bprf64;
    BOOST_REQUIRE(prf_class_from_string(prf_class_to_string(PrfClass::Hprf400), p));
    BOOST_REQUIRE(p == PrfClass::Hprf400);
    BOOST_REQUIRE(!prf_class_from_string("nope", p));
    DataRate d = DataRate::R6p8M;
    BOOST_REQUIRE(data_rate_from_string(data_rate_to_string(DataRate::R27M), d));
    BOOST_REQUIRE(d == DataRate::R27M);
    BOOST_REQUIRE(!data_rate_from_string("nope", d));
    SfdMode s = SfdMode::R4z2;
    BOOST_REQUIRE(sfd_mode_from_string(sfd_mode_to_string(SfdMode::Ieee8), s));
    BOOST_REQUIRE(!sfd_mode_from_string("nope", s));
    PhrMode pm = PhrMode::Standard;
    BOOST_REQUIRE(phr_mode_from_string(phr_mode_to_string(PhrMode::None), pm));
    BOOST_REQUIRE(!phr_mode_from_string("nope", pm));
    // The two NEW enumerations round-trip for every member, and the PHR rate
    // deliberately REJECTS the payload-rate spellings: the two axes no longer
    // share a value domain, which is the whole point of M0.1 / R2.
    for (size_t i = 0; i < kPhrRateCount; ++i) {
        const PhrRate r = kAllPhrRates[i];
        PhrRate back = PhrRate::Standard850k;
        BOOST_REQUIRE(phr_rate_from_string(phr_rate_to_string(r), back));
        BOOST_REQUIRE(back == r);
        BOOST_REQUIRE(!phr_rate_from_string("6p8m", back));
        BOOST_REQUIRE(!phr_rate_from_string("27m", back));
        BOOST_REQUIRE(!phr_rate_from_string("nope", back));
    }
    for (size_t i = 0; i < kPreambleLengthCount; ++i) {
        const PreambleLength p = kAllPreambleLengths[i];
        PreambleLength back = PreambleLength::Sym64;
        BOOST_REQUIRE(preamble_length_from_string(preamble_length_to_string(p), back));
        BOOST_REQUIRE(back == p);
        BOOST_REQUIRE(preamble_length_from_symbols(preamble_length_symbols(p), back));
        BOOST_REQUIRE(back == p);
        BOOST_REQUIRE(!preamble_length_from_string("128", back));
        BOOST_REQUIRE(!preamble_length_from_string("nope", back));
    }
    for (size_t i = 0; i < kFrameProfileIdCount; ++i) {
        const FrameProfileId m = kAllFrameProfileIds[i];
        FrameProfileId back = FrameProfileId::TwrV1;
        BOOST_REQUIRE(frame_profile_id_from_string(frame_profile_id_to_string(m), back));
        BOOST_REQUIRE(back == m);
        BOOST_REQUIRE(!frame_profile_id_from_string("nope", back));
    }
    StsMode sm = StsMode::Off;
    BOOST_REQUIRE(sts_mode_from_string(sts_mode_to_string(StsMode::Sp1024), sm));
    BOOST_REQUIRE(!sts_mode_from_string("nope", sm));
    FcsAppender fa = FcsAppender::None;
    BOOST_REQUIRE(fcs_appender_from_string(fcs_appender_to_string(FcsAppender::MacLayer), fa));
    BOOST_REQUIRE(!fcs_appender_from_string("nope", fa));
    TxPowerPolicy tp = TxPowerPolicy::LeaveUntouched;
    BOOST_REQUIRE(
        tx_power_policy_from_string(tx_power_policy_to_string(TxPowerPolicy::IqAmplitude), tp));
    BOOST_REQUIRE(!tx_power_policy_from_string("nope", tp));
    PulseShaping ps = PulseShaping::ExistingHrP;
    BOOST_REQUIRE(pulse_shaping_from_string(pulse_shaping_to_string(PulseShaping::Rectangular),
                                            ps));
    BOOST_REQUIRE(!pulse_shaping_from_string("nope", ps));
    AgcMode am = AgcMode::Manual;
    BOOST_REQUIRE(agc_mode_from_string(agc_mode_to_string(AgcMode::VendorDefault), am));
    BOOST_REQUIRE(!agc_mode_from_string("nope", am));
    FirstPathAlgorithm fp = FirstPathAlgorithm::Peak;
    BOOST_REQUIRE(first_path_algorithm_from_string(
        first_path_algorithm_to_string(FirstPathAlgorithm::StrongestCluster), fp));
    BOOST_REQUIRE(!first_path_algorithm_from_string("nope", fp));
    CompensationFlag cf = CompensationFlag::Off;
    BOOST_REQUIRE(compensation_flag_from_string(compensation_flag_to_string(
                      CompensationFlag::Required),
                                                  cf));
    BOOST_REQUIRE(!compensation_flag_from_string("nope", cf));
    TimeReferenceEvent ev = TimeReferenceEvent::HostMonotonic;
    BOOST_REQUIRE(time_reference_event_from_string(
        time_reference_event_to_string(TimeReferenceEvent::RxEnable), ev));
    BOOST_REQUIRE(ev == TimeReferenceEvent::RxEnable);
    BOOST_REQUIRE(!time_reference_event_from_string("nope", ev));
    TimeDomain td = TimeDomain::Unspecified;
    BOOST_REQUIRE(time_domain_from_string(time_domain_to_string(TimeDomain::MonotonicHost), td));
    BOOST_REQUIRE(!time_domain_from_string("nope", td));
    TimeUnit tu = TimeUnit::Nanoseconds;
    BOOST_REQUIRE(time_unit_from_string(time_unit_to_string(TimeUnit::NativeTicks),
        tu));
    BOOST_REQUIRE(!time_unit_from_string("nope", tu));
    CapabilityStatus cs = CapabilityStatus::Measured;
    BOOST_REQUIRE_EQUAL(std::string(capability_status_to_string(CapabilityStatus::Measured)),
                        std::string("measured"));
    BOOST_REQUIRE_EQUAL(
        std::string(capability_status_to_string(CapabilityStatus::MeasuredPending)),
        std::string("measured_pending"));
    BOOST_REQUIRE_EQUAL(std::string(capability_status_to_string(CapabilityStatus::OutOfScope)),
                        std::string("out_of_scope"));
    (void)cs;
}

BOOST_AUTO_TEST_CASE(opt_presence_is_explicit)
{
    Opt<double> a;
    BOOST_REQUIRE(!a.has());
    BOOST_REQUIRE(a.value_or(7.0) == 7.0);
    a = 1.5;
    BOOST_REQUIRE(a.has());
    BOOST_REQUIRE(static_cast<bool>(a));
    BOOST_REQUIRE(a.value() == 1.5);
    BOOST_REQUIRE(a.value_or(7.0) == 1.5);
    a.reset();
    BOOST_REQUIRE(!a.has());
    BOOST_REQUIRE(Opt<double>::some(2.0) == Opt<double>(2.0));
    BOOST_REQUIRE(Opt<double>::none() == Opt<double>());
    // Reading an absent value is a programming error, not a silent default.
    BOOST_REQUIRE_THROW(a.value(), std::logic_error);
    TimedField f;
    BOOST_REQUIRE(!f.marker.has());
    f.marker = TimestampMarker::RmarkerTx;
    BOOST_REQUIRE(f.marker.has());
    BOOST_REQUIRE(f == f);
    TimedField g = f;
    g.marker = TimestampMarker::AntennaPlane;
    BOOST_REQUIRE(f != g);
    BOOST_REQUIRE(g.to_text().find("antenna_plane") != std::string::npos);
    BOOST_REQUIRE(f.to_text().find("rmarker_tx") != std::string::npos);
    BOOST_REQUIRE(g.to_text().find("rmarker_tx") == std::string::npos);
}


// ===========================================================================
// N07: an enum value the enum does not have must be REFUSED, and NAMED.
// ===========================================================================
//
// Enum values are validated only at the JSON reader.  A value that reaches a
// TwrConfig by DIRECT CONSTRUCTION went through validate() unchecked, and the
// fields gated by an equality test rather than an allow-list were ACCEPTED:
// `rx.agc = 250` passed, serialised as "invalid", and was then refused by this
// layer's own reader -- the contract contradicted itself.  A few more were
// refused only incidentally, with the violation naming an UNRELATED field
// (`session.protocol = 2` was reported as a problem with
// timing.response_to_final.ns).
//
// This test is exhaustive over the enum sites rather than a sample: a new enum
// field that forgets its domain check should fail here, not ship.
BOOST_AUTO_TEST_CASE(every_config_enum_refuses_an_out_of_domain_value)
{
    using Setter = void (*)(TwrConfig&, int);
    struct Site {
        const char* field;
        Setter set;
    };
    // 99 and 250 are outside every one of these enums.  NOT 7: that is a real
    // `TimeReferenceEvent` member (which has eleven), and a real member must be
    // judged on merit by the semantic checks rather than flagged as unknown --
    // the last block below pins exactly that distinction.
    const Site sites[] = {
        {"session.protocol",
         [](TwrConfig& c, int v) { c.session.protocol = static_cast<Protocol>(v); }},
        {"session.role",
         [](TwrConfig& c, int v) { c.session.role = static_cast<Role>(v); }},
        {"phy.prf_class",
         [](TwrConfig& c, int v) { c.phy.prf_class = static_cast<PrfClass>(v); }},
        {"phy.data_rate",
         [](TwrConfig& c, int v) { c.phy.data_rate = static_cast<DataRate>(v); }},
        {"phy.phr_rate",
         [](TwrConfig& c, int v) { c.phy.phr_rate = static_cast<PhrRate>(v); }},
        {"frame.frame_profile",
         [](TwrConfig& c, int v) { c.frame.frame_profile = static_cast<FrameProfileId>(v); }},
        {"frame.sfd_mode",
         [](TwrConfig& c, int v) { c.frame.sfd_mode = static_cast<SfdMode>(v); }},
        {"frame.phr_mode",
         [](TwrConfig& c, int v) { c.frame.phr_mode = static_cast<PhrMode>(v); }},
        {"frame.fcs_append",
         [](TwrConfig& c, int v) { c.frame.fcs_append = static_cast<FcsAppender>(v); }},
        {"frame.sts_mode",
         [](TwrConfig& c, int v) { c.frame.sts_mode = static_cast<StsMode>(v); }},
        {"tx.power_policy",
         [](TwrConfig& c, int v) { c.tx.power_policy = static_cast<TxPowerPolicy>(v); }},
        {"tx.pulse_shaping",
         [](TwrConfig& c, int v) { c.tx.pulse_shaping = static_cast<PulseShaping>(v); }},
        {"rx.agc",
         [](TwrConfig& c, int v) { c.rx.agc = static_cast<AgcMode>(v); }},
        {"calibration.link_delay_unit",
         [](TwrConfig& c, int v) { c.calibration.link_delay_unit = static_cast<TimeUnit>(v); }},
        {"calibration.first_path_algorithm",
         [](TwrConfig& c, int v) {
             c.calibration.first_path_algorithm = static_cast<FirstPathAlgorithm>(v); }},
        {"calibration.cfo_compensation",
         [](TwrConfig& c, int v) {
             c.calibration.cfo_compensation = static_cast<CompensationFlag>(v); }},
        {"calibration.sfo_compensation",
         [](TwrConfig& c, int v) {
             c.calibration.sfo_compensation = static_cast<CompensationFlag>(v); }},
        {"frame.sfd_timeout.domain",
         [](TwrConfig& c, int v) { c.frame.sfd_timeout.domain = static_cast<TimeDomain>(v); }},
        {"frame.sfd_timeout.reference",
         [](TwrConfig& c, int v) {
             c.frame.sfd_timeout.reference = static_cast<TimeReferenceEvent>(v); }},
        {"timing.poll_start.domain",
         [](TwrConfig& c, int v) { c.timing.poll_start.domain = static_cast<TimeDomain>(v); }},
        {"timing.poll_to_response.reference",
         [](TwrConfig& c, int v) {
             c.timing.poll_to_response.reference = static_cast<TimeReferenceEvent>(v); }},
        {"timeouts.rx_timeout.domain",
         [](TwrConfig& c, int v) { c.timeouts.rx_timeout.domain = static_cast<TimeDomain>(v); }},
        {"timeouts.rx_timeout.reference",
         [](TwrConfig& c, int v) {
             c.timeouts.rx_timeout.reference = static_cast<TimeReferenceEvent>(v); }},
        {"diagnostics.stats_cadence.domain",
         [](TwrConfig& c, int v) {
             c.diagnostics.stats_cadence.domain = static_cast<TimeDomain>(v); }},
        // The third enum on every TimedField.  It is OPTIONAL (absent is legal
        // on a host-monotonic clock), and because it is optional it is easy to
        // forget: covering domain+reference and assuming the pair was complete
        // left twelve more configurations accepted.  The exhaustive sweep is
        // what found that, so the sweep is what is pinned here.
        {"frame.sfd_timeout.marker",
         [](TwrConfig& c, int v) {
             c.frame.sfd_timeout.marker = Opt<TimestampMarker>(static_cast<TimestampMarker>(v)); }},
        {"timing.poll_start.marker",
         [](TwrConfig& c, int v) {
             c.timing.poll_start.marker = Opt<TimestampMarker>(static_cast<TimestampMarker>(v)); }},
        {"timing.poll_to_response.marker",
         [](TwrConfig& c, int v) {
             c.timing.poll_to_response.marker = Opt<TimestampMarker>(static_cast<TimestampMarker>(v)); }},
        {"timing.response_to_final.marker",
         [](TwrConfig& c, int v) {
             c.timing.response_to_final.marker = Opt<TimestampMarker>(static_cast<TimestampMarker>(v)); }},
        {"timing.final_to_report.marker",
         [](TwrConfig& c, int v) {
             c.timing.final_to_report.marker = Opt<TimestampMarker>(static_cast<TimestampMarker>(v)); }},
        {"timing.post_tx_rx_enable.marker",
         [](TwrConfig& c, int v) {
             c.timing.post_tx_rx_enable.marker = Opt<TimestampMarker>(static_cast<TimestampMarker>(v)); }},
        {"timing.min_tx_lead_time.marker",
         [](TwrConfig& c, int v) {
             c.timing.min_tx_lead_time.marker = Opt<TimestampMarker>(static_cast<TimestampMarker>(v)); }},
        {"timeouts.poll_rx_window.marker",
         [](TwrConfig& c, int v) {
             c.timeouts.poll_rx_window.marker = Opt<TimestampMarker>(static_cast<TimestampMarker>(v)); }},
        {"timeouts.response_rx_window.marker",
         [](TwrConfig& c, int v) {
             c.timeouts.response_rx_window.marker = Opt<TimestampMarker>(static_cast<TimestampMarker>(v)); }},
        {"timeouts.final_rx_window.marker",
         [](TwrConfig& c, int v) {
             c.timeouts.final_rx_window.marker = Opt<TimestampMarker>(static_cast<TimestampMarker>(v)); }},
        {"timeouts.report_rx_window.marker",
         [](TwrConfig& c, int v) {
             c.timeouts.report_rx_window.marker = Opt<TimestampMarker>(static_cast<TimestampMarker>(v)); }},
        {"timeouts.rx_timeout.marker",
         [](TwrConfig& c, int v) {
             c.timeouts.rx_timeout.marker = Opt<TimestampMarker>(static_cast<TimestampMarker>(v)); }},
        {"timeouts.exchange_timeout.marker",
         [](TwrConfig& c, int v) {
             c.timeouts.exchange_timeout.marker = Opt<TimestampMarker>(static_cast<TimestampMarker>(v)); }},
        {"timeouts.retry_interval.marker",
         [](TwrConfig& c, int v) {
             c.timeouts.retry_interval.marker = Opt<TimestampMarker>(static_cast<TimestampMarker>(v)); }},
        {"diagnostics.stats_cadence.marker",
         [](TwrConfig& c, int v) {
             c.diagnostics.stats_cadence.marker = Opt<TimestampMarker>(static_cast<TimestampMarker>(v)); }},
    };
    for (const Site& s : sites) {
        for (const int v : { 99, 250 }) {
            TwrConfig c = minimal();
            s.set(c, v);
            const ValidationReport rep = validate(c, measured_caps());
            require_machine_readable(rep);
            BOOST_REQUIRE_MESSAGE(!rep.ok(), std::string(s.field) + " = " +
                std::to_string(v) + " was ACCEPTED (fail-open)");
            const ConfigViolation* viol =
                rep.find(s.field, ConfigReason::UnknownEnumValue);
            BOOST_REQUIRE_MESSAGE(viol != nullptr,
                std::string(s.field) + " = " + std::to_string(v) +
                " was refused, but not as an out-of-domain enum, and the "
                "violation does not name the field: " + rep.to_string());
        }
    }

    // The peer binding repeats Role once per endpoint.
    {
        TwrConfig c = minimal();
        c.radio.peers.clear();
        EndpointBinding p;
        p.id = "peer";
        c.radio.peers.push_back(p);
        c.radio.peers[0].role = static_cast<Role>(99);
        const ValidationReport rep = validate(c, measured_caps());
        require_machine_readable(rep);
        BOOST_REQUIRE(!rep.ok());
        BOOST_REQUIRE(rep.find("radio.peers[0].role",
                               ConfigReason::UnknownEnumValue) != nullptr);
    }

    // ... and a value that IS a member is not touched by this check: the
    // domain answer and the supported/implemented answer are different
    // questions, and a real member survives to be judged on merit.
    {
        TwrConfig c = minimal();
        c.calibration.cfo_compensation = CompensationFlag::Required;
        const ValidationReport rep = validate(c, measured_caps());
        BOOST_REQUIRE(rep.find("calibration.cfo_compensation",
                               ConfigReason::UnknownEnumValue) == nullptr);
    }
}

} // BOOST_AUTO_TEST_SUITE(twr_config)
