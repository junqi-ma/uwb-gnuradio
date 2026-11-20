/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * TWR configuration, split by responsibility (Phase 2).
 * The aggregate entry point is <openuwb/twr/twr_config.h>; these pieces are
 * ordered, not independent: each one includes its predecessor.
 */
#ifndef INCLUDED_OPENUWB_TWR_CONFIG_TYPES_H
#define INCLUDED_OPENUWB_TWR_CONFIG_TYPES_H

#include <openuwb/twr/twr_config_capabilities.h>
namespace gr {
namespace uwb {
namespace twr {

// ===========================================================================
// 4. The typed configuration
// ===========================================================================

// --- 帧几何 (frame geometry) ------------------------------------------------

// A CLAIM about the versioned TWR MAC frame layout (REQ-PROTO-06), not a
// second source of truth for it.
//
// M0 review R3: this struct used to carry a pre-codec "7-byte header" geometry
// (FCF(2) + sequence(1) + PAN(2) + address(2)) with 9/19/24-byte budgets while
// uwb_twr_frame.h emitted a 14-byte header and 16/26/31-byte on-air frames.
// Both suites passed because nothing compared the two.
//
// Now the codec owns every one of these numbers (`FrameProfileGeometry`) and
// this struct is only what a caller BELIEVES.  `frame_geometry_claim()` turns
// it into the codec's `FrameGeometryClaim`, `frame_geometry_check()` reports
// every field that disagrees, and the validator turns each disagreement into
// its own rejection naming that exact field.  The fields stay here so the
// claim is visible in the canonical field listing, in the config hash and in
// the JSON -- an operator must be able to see what was claimed -- but they
// never decide anything.
struct FrameGeometry {
    // MAC header bytes the caller believes the frame has.  frame v1: 14
    // (version / function code / 16-bit session / seq / PAN / src / dst /
    // flags), from kFrameHeaderBytes.
    uint16_t mac_header_bytes = 0;
    // bytes per timestamp field; 5 for the 40-bit frame-v1 timestamp.
    uint16_t timestamp_bytes = 0;
    // trailing address / profile trailer; frame v1 has none.
    uint16_t mac_footer_bytes = 0;
    // FCS bytes reserved INSIDE the MAC PSDU; 0 when the PHY appends it.
    uint16_t mac_fcs_bytes = 0;
    // PHR information bytes the caller believes are carried; frame v1: 2
    // (13 information bits, SEC-DED coded to 19 bits over 21 symbols).  This is
    // the PHR's INFORMATION size, not its on-air duration.
    uint16_t phr_bytes = 0;

    bool operator==(const FrameGeometry& o) const
    {
        return mac_header_bytes == o.mac_header_bytes &&
               timestamp_bytes == o.timestamp_bytes &&
               mac_footer_bytes == o.mac_footer_bytes &&
                   mac_fcs_bytes == o.mac_fcs_bytes &&
               phr_bytes == o.phr_bytes;
    }
    bool operator!=(const FrameGeometry& o) const { return !(*this == o); }
    bool any_set() const
    {
        return mac_header_bytes != 0 || timestamp_bytes != 0 || mac_footer_bytes != 0 ||
               mac_fcs_bytes != 0 || phr_bytes != 0;
    }
    // One line for a rejection or a diff, with the field order of the codec's
    // own report so the two can be read side by side.
    std::string to_text() const
    {
        return "mac_header_bytes=" + std::to_string(mac_header_bytes) +
               " timestamp_bytes=" + std::to_string(timestamp_bytes) +
               " mac_footer_bytes=" + std::to_string(mac_footer_bytes) +
               " mac_fcs_bytes=" + std::to_string(mac_fcs_bytes) +
               " phr_bytes=" + std::to_string(phr_bytes);
    }
};

// The claim, in the codec's own claim type.  Same field names on purpose: the
// config states what it believes and the codec checks it.  A DISTINCT type
// from FrameProfileGeometry is also deliberate, so an authority and a claim can
// never be swapped for one another by accident.
inline FrameGeometryClaim frame_geometry_claim(const FrameGeometry& g)
{
    FrameGeometryClaim c;
    c.mac_header_bytes = g.mac_header_bytes;
    c.timestamp_bytes = g.timestamp_bytes;
    c.mac_footer_bytes = g.mac_footer_bytes;
    c.mac_fcs_bytes = g.mac_fcs_bytes;
    c.phr_bytes = g.phr_bytes;
    return c;
}

// The geometry the CODEC says, in the config's own struct.  This is how a
// caller (and every test fixture) states a claim without keeping a private
// copy of the numbers: fill the config from the authority, then let the
// validator prove the two agree.
inline FrameGeometry frame_geometry_from_authority(const FrameProfileGeometry& g,
                                                   const FrameProfile& p)
{
    FrameGeometry out;
    out.mac_header_bytes = static_cast<uint16_t>(g.mac_header_bytes());
    out.timestamp_bytes = static_cast<uint16_t>(g.timestamp_bytes(p));
    out.mac_footer_bytes = static_cast<uint16_t>(g.mac_footer_bytes());
    out.mac_fcs_bytes = static_cast<uint16_t>(g.mac_fcs_bytes());
    out.phr_bytes = static_cast<uint16_t>(g.phr_bytes());
    return out;
}

// The claim of the executable authority for a profile id, or false when the id
// names no implemented layout.  Never a silent fallback to frame v1.
inline bool frame_geometry_of_profile(FrameProfileId id, FrameGeometry& out)
{
    FrameProfileGeometry g;
    if (!frame_geometry_for(id, g))
        return false;
    FrameProfile p;
    if (!frame_profile_for(id, p))
        return false;
    out = frame_geometry_from_authority(g, p);
    return true;
}

// Number of MAC PSDU bytes a frame of `type` occupies for a profile, INCLUDING
// the FCS when the MAC appends it and EXCLUDING it when the PHY does (exactly
// one layer appends the FCS, REQ-API-01 帧格式).
//
// This is a DELEGATION to the codec, which is the only place a frame length is
// computed: `FrameProfileGeometry::mac_payload_bytes()` calls
// `frame_length_for()`.  M0 re-summed `header + n * timestamp + footer` here,
// which is how a 7-byte header produced a 9-byte Poll and understated the
// frame-duration budget.
inline uint32_t frame_psdu_bytes(FrameType type, FcsAppender fcs)
{
    const FcsOwner owner = (fcs == FcsAppender::MacLayer) ? FcsOwner::MacLayer
                                                          : FcsOwner::PhyLayer;
    FrameProfileGeometry g;
    g.id = FrameProfileId::TwrV1;
    g.fcs_owner = owner;
    FrameProfile p;
    if (!frame_profile_for(FrameProfileId::TwrV1, p))
        return 0;
    return static_cast<uint32_t>(g.mac_payload_bytes(type, p));
}

// Retained call shape.  `g` is DELIBERATELY IGNORED: a claim does not decide
// a length, and keeping the parameter means an existing call site keeps
// compiling while losing the ability to pass a number of its own.  New code
// should use the two-argument form above.
inline uint32_t frame_psdu_bytes(const FrameGeometry& g, FrameType type, FcsAppender fcs)
{
    (void)g;
    return frame_psdu_bytes(type, fcs);
}

struct SessionConfig {
    Protocol protocol = Protocol::Ss;
    Role role = Role::Initiator;
    uint16_t local_address = 0;
    uint16_t peer_address = 0;
    uint16_t pan_id = 0;
    // LOCAL session id.  The wire field at kOffSessionId is 16 bits, so the
    // mapping is identity-or-refuse (session_id_to_wire()), never a fold or a
    // mask.  The type stays uint32_t DELIBERATELY rather than narrowing to
    // uint16_t: narrowing would make the local id space and the wire id space
    // indistinguishable in the type system, so an operator could no longer
    // express the id they intended and would never learn it does not fit.
    // Keeping the wider type makes the refusal a VISIBLE, TESTABLE, ATTRIBUTABLE
    // configuration error instead of an unreachable code path.
    //
    // The cost is stated here rather than left to be discovered: the wire
    // session space is 2^16, so ~256 concurrently live sessions reach a 50%
    // chance of one collision (kSessionIdBirthdaySessions50pct).  No local ->
    // wire mapping can raise that; only a different frame version with a wider
    // or keyed session field could, and the peer would have to understand it.
    // 0 is the frame profile's "no session" marker.
    uint32_t session_id = 0;
    uint32_t exchange_id = 0;
    // Frame sequence number and its own wrap modulus, INDEPENDENT of the
    // session id (REQ-API-01 会话: 帧序号回绕与会话 ID 独立).
    uint8_t sequence = 0;
    uint16_t sequence_modulus = 256;
    uint32_t measurement_count = 1;
    Duration measurement_interval{0};
    uint32_t max_attempts_per_exchange = 1;
    Duration retry_backoff{0};
    // Phase 1 allows exactly one in-flight exchange per endpoint.
    uint32_t max_in_flight_exchanges = kTwrMaxInFlightExchanges;
    // A peer frame with a mismatching PAN/address is WrongPeer, not a match.
    bool require_pan_match = true;
    bool require_address_match = true;
};

// --- PHY -------------------------------------------------------------------

struct PhyConfig {
    uint8_t channel = 5;
    double center_frequency_hz = 0.0; // 0 = not stated; the plan decides
    uint8_t tx_preamble_code = 0;     // 0 = "not stated" -> rejected
    uint8_t rx_preamble_code = 0;
    // SYNC repetitions.  The raw count is kept because it is what the
    // capability whitelist and the measured PHY matrix key on, but it must be
    // a member of `PreambleLength` -- a length this decoder/CFO path can
    // actually run -- and the validator separately consults the whitelist.
    uint16_t preamble_symbols = 0;
    PrfClass prf_class = PrfClass::Bprf64;
    // PAYLOAD data rate, measured: 6.81 Mb/s is the only rate that round
    // trips through this modulator/demodulator pair.
    DataRate data_rate = DataRate::R6p8M;
    // PHR rate -- a SEPARATE quantity from `data_rate` (M0.1 / R2).  It used
    // to be a DataRate required to EQUAL `data_rate`, which had the rule
    // exactly backwards: this modulator sends a 0.85 Mb/s PHR (21 symbols of
    // 512 chips) ahead of a 6.81 Mb/s payload (8 chips per burst of 64 per
    // symbol).  See the PhrRate comment for the full argument.
    PhrRate phr_rate = PhrRate::Standard850k;
};

// --- 帧格式 (frame format) --------------------------------------------------

struct FrameFormatConfig {
    // WHICH MAC LAYOUT.  A FrameProfileId, not a free-form string: an
    // arbitrary non-empty profile name must not be able to act as an
    // executable geometry (M0 review R3).  It selects the authority that the
    // `geometry` claim below is checked against.
    FrameProfileId frame_profile = FrameProfileId::TwrV1;
    SfdMode sfd_mode = SfdMode::R4z2;
    // Raw SFD length.  It must equal sfd_mode_symbols(sfd_mode): the mode
    // names the sequence, the count is a restatement of it, and a caller may
    // not say "4z2" while stating 4.
    uint16_t sfd_symbols = 0;
    // Timeout after which a received frame is declared SFD-missing.
    // Zero means "disabled"; a non-zero value must be in a device-tick domain.
    TimedField sfd_timeout;
    // PHR presence/form.  The PHR RATE is phy.phr_rate; the two are separate
    // axes and neither is derived from the other.
    PhrMode phr_mode = PhrMode::Standard;
    bool ranging_bit = true;

    // THREE DISTINCT CONCEPTS, never conflated:
    //  (1) upper-layer bytes handed to the MAC, stated as a CLAIM about the
    //      layout that the codec then checks field by field,
    FrameGeometry geometry;
    //  (2) bytes the MAC places in the PSDU field,
    uint16_t mac_psdu_bytes = 0;
    //  (3) whether (2) already contains the FCS,
    bool mac_psdu_includes_fcs = false;
    FcsAppender fcs_append = FcsAppender::PhyLayer;
    // 16-bit CRC.  Checked against the profile's own FCS size; the default
    // value here is only a starting point the validator will refuse.
    uint16_t fcs_bytes = 2;
    // 1 = the application payload the caller wants to carry; the MAC header,
    // the timestamps and the FCS are NOT counted here.
    uint16_t application_payload_bytes = 0;

    // STS: phase-2 only.  Present so the interface can grow, rejected now.
    StsMode sts_mode = StsMode::Off;
    uint16_t sts_length_symbols = 0;
};

// --- 发射 (transmit) -------------------------------------------------------

struct TransmitConfig {
    uint8_t port = 0; // physical TX channel index inside the device
    // The THREE distinct power concepts (REQ-API-01 发射, AGENTS rule 7).
    // `gain_db`  : the UHD/USRP TX gain setting, in dB.
    Opt<double> gain_db;
    // `iq_amplitude` : digital scaling of the SC16 waveform, dimensionless,
    //                  (0, 1] for a full-scale signed 16-bit stream.
    Opt<double> iq_amplitude;
    // `calibrated_tx_power_dbm` : the power MEASURED at the antenna plane for
    //                  this gain/frequency/bandwidth, in dBm.
    Opt<double> calibrated_tx_power_dbm;
    TxPowerPolicy power_policy = TxPowerPolicy::LeaveUntouched;
    PulseShaping pulse_shaping = PulseShaping::ExistingHrP;
    // A vendor "power word" is a FOURTH, separately named field.  It may only
    // be set together with the backend capability id that defines its bits;
    // there is no software-side default for it.
    Opt<uint32_t> vendor_power_word;
    std::string vendor_power_word_backend;
};

// --- 接收 (receive) --------------------------------------------------------

struct ReceiveConfig {
    uint8_t port = 0; // physical RX channel index inside the device
    Opt<double> gain_db; // RX gain in dB
    AgcMode agc = AgcMode::Manual;
    Opt<double> bandwidth_hz; // RF bandwidth, distinct from the sample rate
    double detection_threshold = 0.0;    // energy/coarse gate, linear
    double correlation_threshold = 0.0;  // fine preamble correlation gate
    double first_path_threshold = 0.0;   // first-path quality gate
    uint16_t first_path_index = 0;       // first usable CIR index
    uint16_t first_path_window = 0;      // CIR taps kept for the search
    // A vendor PAC-like value is ONLY meaningful together with the backend
    // capability that defines it AND the software step the backend actually
    // applies.  Mapping a PAC value onto an arbitrary software detection step
    // is forbidden (REQ-API-01 接收).
    Opt<uint32_t> vendor_pac_value;
    std::string vendor_pac_backend;
    Opt<double> vendor_pac_applied_step;
};

// --- 无线设备 (radio) ------------------------------------------------------

struct RadioReadback {
    bool present = false;
    double sample_rate_hz = 0.0;
    double center_freq_hz = 0.0;
    uint8_t tx_channel = 0;
    uint8_t rx_channel = 0;
    std::string mpm_string;    // e.g. "CG400" / "X410"
    std::string fpga_image;    // e.g. "CG400-...-R1"
    std::string uhd_version;
    std::string clock_source;
    std::string time_source;
};

// One logical endpoint sharing the physical device.  Phase 1: two of these
// (A = TX0/RX0, B = TX1/RX1) on one X410.
struct EndpointBinding {
    std::string id; // "A" / "B"
    Role role = Role::Initiator;
    uint8_t tx_channel = 0;
    uint8_t rx_channel = 0;
    double native_sample_rate_hz = 0.0;
    bool occupies_resources = true;
};

struct RadioConfig {
    std::string device_args;
    uint8_t tx_channel = 0;
    uint8_t rx_channel = 0;
    double native_sample_rate_hz = 0.0;
    std::string clock_source; // "" = leave untouched
    std::string time_source;  // "" = leave untouched
    std::string fpga_image;   // "" = build default
    std::string dpdk_config;  // "" = no DPDK
    // The other endpoint(s) of the same session, so the config is
    // self-describing and the validator can reject channel conflicts.
    std::vector<EndpointBinding> peers;
    // Filled at startup.  A readback that differs from the request is a
    // STARTUP FAILURE (REQ-API-01 无线设备: 读回不符启动失败).
    RadioReadback readback;
    // Require the readback block to be filled in.  A dry-run/offline session
    // may set it false; a real radio must not.
    bool require_readback = true;
};

// --- 每包时间 (per-message timing) -----------------------------------------

struct PerMessageTiming {
    // When the first Poll of a measurement is transmitted.  Absolute in the
    // device clock domain when domain == DeviceTicks.
    TimedField poll_start;
    // Responder: RX Poll RMARKER -> Response TX RMARKER.  THE PROTOCOL REPLY
    // DELAY.
    TimedField poll_to_response;
    // Initiator: RX Response RMARKER -> Final TX RMARKER (DS only).  A
    // different thing from poll_to_response.
    TimedField response_to_final;
    // Report: phase 1 has no Report frame (FrameType::Report is reserved),
    // so this must stay zero.
    TimedField final_to_report;
    // After the END OF OUR OWN TRANSMITTED FRAME, delay before RX is armed.
    // THE POST-TX RX-ENABLE DELAY -- a completely different number from the
    // reply delay and from the RX timeout.
    TimedField post_tx_rx_enable;
    // Measured minimum UHD timed-command lead time.  The operator must supply
    // it; there is no default (REQ-GR-04: 未经验证的短时序不得承诺).
    TimedField min_tx_lead_time;
};

// --- 超时 (timeouts) -------------------------------------------------------

struct TimeoutConfig {
    // Per-frame RX windows.  Each is armed at RX ENABLE and measured on the
    // device clock.
    TimedField poll_rx_window;
    TimedField response_rx_window;
    TimedField final_rx_window;
    TimedField report_rx_window; // must stay zero in phase 1
    // How long to wait for the expected RX frame, measured FROM RX ENABLE.
    TimedField rx_timeout;
    // Whole-exchange deadline, measured on the HOST MONOTONIC CLOCK.
    TimedField exchange_timeout;
    // Gap between a failed attempt and its retry, host monotonic.
    TimedField retry_interval;
};

// --- 时戳与校准 (timestamps and calibration) -------------------------------

// A calibration record, versioned by device / channel / rate / gain /
// profile.  `valid_until_monotonic_ns` is 0 = no expiry.
struct CalibrationRecord {
    std::string calibration_id;
    std::string device_serial;
    uint8_t channel = 0;
    double native_sample_rate_hz = 0.0;
    std::string profile_version;
    Opt<double> gain_db; // the gain the constants were taken at
    int64_t valid_until_monotonic_ns = 0;
};

struct TimestampCalibrationConfig {
    // The link / antenna / cable delays, in integer NANOSECONDS (named type,
    // not a vendor tick).
    Duration tx_link_delay{0};
    Duration rx_link_delay{0};
    Duration antenna_delay{0};
    Duration cable_delay{0};
    // The SAME quantity expressed in DEVICE TICKS -- a separate field, never
    // added to the nanosecond fields above.
    int64_t tx_link_delay_native_ticks = 0;
    int64_t rx_link_delay_native_ticks = 0;
    double native_sample_rate_hz = 0.0;
    // Which unit the *_native_ticks fields and the Durations refer to.
    TimeUnit link_delay_unit = TimeUnit::Nanoseconds;
    FirstPathAlgorithm first_path_algorithm = FirstPathAlgorithm::LeadingEdge;
    CompensationFlag cfo_compensation = CompensationFlag::Off;
    CompensationFlag sfo_compensation = CompensationFlag::Off;
    std::string calibration_id;
    CalibrationRecord record;
    // REQ-CAL-01: a calibration is applicable EXACTLY ONCE.  `applied_count`
    // is bumped by apply_calibration_once(); a second call is rejected.
    uint32_t applied_count = 0;
    // A calibration is mandatory for a validated absolute range claim.
    bool calibration_required = true;
};

// --- 诊断 (diagnostics) ----------------------------------------------------

struct DiagnosticsConfig {
    bool cir_capture_enabled = false;
    uint64_t cir_capture_max_bytes = 0;
    uint32_t cir_capture_stride = 0;
    bool short_iq_enabled = false;
    uint64_t short_iq_max_bytes = 0;
    uint32_t short_iq_stride = 0;
    bool raw_frame_dump = false;
    uint64_t raw_frame_max_bytes = 0;
    std::string result_output_path; // "" = no file output
    uint32_t result_queue_capacity = 0;
    uint32_t event_queue_capacity = 0;
    TimedField stats_cadence; // host monotonic
    // Diagnostic I/O must never run on the realtime thread (REQ-API-01
    // 诊断).  This flag exists so that violation is a CONFIG error, not a
    // surprise discovered during a soak.
    bool io_on_realtime_thread = false;
};

// --- meta ------------------------------------------------------------------

struct ConfigMeta {
    std::string schema_version = "twr-config/2";
    std::string profile_version;
    std::string calibration_version;
    // Optional operator note carried into every result.
    std::string label;
};

// --- the config ------------------------------------------------------------

struct TwrConfig {
    ConfigMeta meta;
    SessionConfig session;
    PhyConfig phy;
    FrameFormatConfig frame;
    TransmitConfig tx;
    ReceiveConfig rx;
    RadioConfig radio;
    PerMessageTiming timing;
    TimeoutConfig timeouts;
    TimestampCalibrationConfig calibration;
    DiagnosticsConfig diagnostics;
};

} // namespace twr
} // namespace uwb
} // namespace gr

#endif /* INCLUDED_OPENUWB_TWR_CONFIG_TYPES_H */
