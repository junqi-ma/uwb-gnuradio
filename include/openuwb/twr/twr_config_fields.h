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
#ifndef INCLUDED_OPENUWB_TWR_CONFIG_FIELDS_H
#define INCLUDED_OPENUWB_TWR_CONFIG_FIELDS_H

#include <openuwb/twr/twr_config_types.h>
namespace openuwb {
namespace twr {

// ===========================================================================
// 5. Canonical field listing (requested vs effective diff + config hash)
// ===========================================================================

// A flattened "path=value" record.  Used to diff requested against effective
// and to compute a stable config hash (REQ-OUT-01: config hash).
struct FieldRecord {
    std::string path;
    std::string value;
};

class FieldSink
{
public:
    explicit FieldSink(std::vector<FieldRecord>& out) : d_out(out) {}

    void add(const std::string& path, const std::string& v) { d_out.push_back({ path,
        v }); }
    void add_i64(const std::string& path, int64_t v) { add(path, twr_int_to_text(v)); }
    void add_u64(const std::string& path, uint64_t v)
    {
        add(path, twr_int_to_text(static_cast<int64_t>(v)));
    }
    void add_u32(const std::string& path, uint32_t v)
    {
        add(path, twr_int_to_text(static_cast<int64_t>(v)));
    }
    void add_u16(const std::string& path, uint16_t v)
    {
        add(path, twr_int_to_text(static_cast<int64_t>(v)));
    }
    void add_u8(const std::string& path, uint8_t v)
    {
        add(path, twr_int_to_text(static_cast<int64_t>(v)));
    }
    void add_bool(const std::string& path, bool v) { add(path, twr_bool_to_text(v)); }
    void add_double(const std::string& path, double v) { add(path,
        twr_double_to_text(v)); }
    void add_dur(const std::string& path, const Duration& d) { add(path,
        twr_duration_to_text(d)); }
    void add_enum(const std::string& path, const char* name) { add(path,
        name ? name : "invalid"); }

    void add_opt_i64(const std::string& path, const Opt<int64_t>& o)
    {
        add(path, o.has() ? twr_int_to_text(o.value()) : std::string("null"));
    }
    void add_opt_u32(const std::string& path, const Opt<uint32_t>& o)
    {
        add(path, o.has() ? twr_int_to_text(static_cast<int64_t>(o.value()))
                          : std::string("null"));
    }
    void add_opt_double(const std::string& path, const Opt<double>& o)
    {
        add(path, o.has() ? twr_double_to_text(o.value()) : std::string("null"));
    }
    void add_opt_str(const std::string& path, const Opt<std::string>& o)
    {
        add(path, o.has() ? o.value() : std::string("null"));
    }
    void add_opt_marker(const std::string& path, const Opt<TimestampMarker>& o)
    {
        add(path,
            o.has() ? std::string(timestamp_marker_to_string(o.value())) : std::string("null"));
    }
    void add_timed(const std::string& path, const TimedField& f) { add(path,
        f.to_text()); }
    void add_record(const std::string& path, const std::string& v) { add(path, v); }

private:
    std::vector<FieldRecord>& d_out;
};

inline void collect_timed_fields(FieldSink& s, const std::string& p,
    const TimedField& f)
{
    s.add(p + ".ns", twr_int_to_text(f.value.nanos()));
    s.add_enum(p + ".domain", time_domain_to_string(f.domain));
    s.add_enum(p + ".reference", time_reference_event_to_string(f.reference));
    s.add_opt_marker(p + ".marker", f.marker);
    s.add_double(p + ".quantisation_hz", f.required_quantisation_hz);
    s.add_i64(p + ".max_quantisation_error_ns", f.max_quantisation_error_ns);
    s.add(p + ".note", f.note);
}

inline void collect_fields(FieldSink& s, const ConfigMeta& v, const std::string& p)
{
    s.add(p + ".schema_version", v.schema_version);
    s.add(p + ".profile_version", v.profile_version);
    s.add(p + ".calibration_version", v.calibration_version);
    s.add(p + ".label", v.label);
}

inline void collect_fields(FieldSink& s, const SessionConfig& v, const std::string& p)
{
    s.add_enum(p + ".protocol", protocol_to_string(v.protocol));
    s.add_enum(p + ".role", role_to_string(v.role));
    s.add_u16(p + ".local_address", v.local_address);
    s.add_u16(p + ".peer_address", v.peer_address);
    s.add_u16(p + ".pan_id", v.pan_id);
    s.add_u32(p + ".session_id", v.session_id);
    s.add_u32(p + ".exchange_id", v.exchange_id);
    s.add_u8(p + ".sequence", v.sequence);
    s.add_u16(p + ".sequence_modulus", v.sequence_modulus);
    s.add_u32(p + ".measurement_count", v.measurement_count);
    s.add_dur(p + ".measurement_interval", v.measurement_interval);
    s.add_u32(p + ".max_attempts_per_exchange", v.max_attempts_per_exchange);
    s.add_dur(p + ".retry_backoff", v.retry_backoff);
    s.add_u32(p + ".max_in_flight_exchanges", v.max_in_flight_exchanges);
    s.add_bool(p + ".require_pan_match", v.require_pan_match);
    s.add_bool(p + ".require_address_match", v.require_address_match);
}

inline void collect_fields(FieldSink& s, const PhyConfig& v, const std::string& p)
{
    s.add_u8(p + ".channel", v.channel);
    s.add_double(p + ".center_frequency_hz", v.center_frequency_hz);
    s.add_u8(p + ".tx_preamble_code", v.tx_preamble_code);
    s.add_u8(p + ".rx_preamble_code", v.rx_preamble_code);
    s.add_u16(p + ".preamble_symbols", v.preamble_symbols);
    s.add_enum(p + ".prf_class", prf_class_to_string(v.prf_class));
    s.add_enum(p + ".data_rate", data_rate_to_string(v.data_rate));
    s.add_enum(p + ".phr_rate", phr_rate_to_string(v.phr_rate));
}

inline void collect_fields(FieldSink& s, const FrameGeometry& v, const std::string& p)
{
    s.add_u16(p + ".mac_header_bytes", v.mac_header_bytes);
    s.add_u16(p + ".timestamp_bytes", v.timestamp_bytes);
    s.add_u16(p + ".mac_footer_bytes", v.mac_footer_bytes);
    s.add_u16(p + ".mac_fcs_bytes", v.mac_fcs_bytes);
    s.add_u16(p + ".phr_bytes", v.phr_bytes);
}

inline void collect_fields(FieldSink& s, const FrameFormatConfig& v,
    const std::string& p)
{
    s.add_enum(p + ".frame_profile", frame_profile_id_to_string(v.frame_profile));
    s.add_enum(p + ".sfd_mode", sfd_mode_to_string(v.sfd_mode));
    s.add_u16(p + ".sfd_symbols", v.sfd_symbols);
    collect_timed_fields(s, p + ".sfd_timeout", v.sfd_timeout);
    s.add_enum(p + ".phr_mode", phr_mode_to_string(v.phr_mode));
    s.add_bool(p + ".ranging_bit", v.ranging_bit);
    // The geometry CLAIM goes into the hash verbatim: two configs that claim
    // different layouts are different configs, even though the codec will
    // reject one of them.
    collect_fields(s, v.geometry, p + ".geometry");
    s.add_u16(p + ".mac_psdu_bytes", v.mac_psdu_bytes);
    s.add_bool(p + ".mac_psdu_includes_fcs", v.mac_psdu_includes_fcs);
    s.add_enum(p + ".fcs_append", fcs_appender_to_string(v.fcs_append));
    s.add_u16(p + ".fcs_bytes", v.fcs_bytes);
    s.add_u16(p + ".application_payload_bytes", v.application_payload_bytes);
    s.add_enum(p + ".sts_mode", sts_mode_to_string(v.sts_mode));
    s.add_u16(p + ".sts_length_symbols", v.sts_length_symbols);
}

inline void collect_fields(FieldSink& s, const TransmitConfig& v, const std::string& p)
{
    s.add_u8(p + ".port", v.port);
    s.add_opt_double(p + ".gain_db", v.gain_db);
    s.add_opt_double(p + ".iq_amplitude", v.iq_amplitude);
    s.add_opt_double(p + ".calibrated_tx_power_dbm", v.calibrated_tx_power_dbm);
    s.add_enum(p + ".power_policy", tx_power_policy_to_string(v.power_policy));
    s.add_enum(p + ".pulse_shaping", pulse_shaping_to_string(v.pulse_shaping));
    s.add_opt_u32(p + ".vendor_power_word", v.vendor_power_word);
    s.add(p + ".vendor_power_word_backend", v.vendor_power_word_backend);
}

inline void collect_fields(FieldSink& s, const ReceiveConfig& v, const std::string& p)
{
    s.add_u8(p + ".port", v.port);
    s.add_opt_double(p + ".gain_db", v.gain_db);
    s.add_enum(p + ".agc", agc_mode_to_string(v.agc));
    s.add_opt_double(p + ".bandwidth_hz", v.bandwidth_hz);
    s.add_double(p + ".detection_threshold", v.detection_threshold);
    s.add_double(p + ".correlation_threshold", v.correlation_threshold);
    s.add_double(p + ".first_path_threshold", v.first_path_threshold);
    s.add_u16(p + ".first_path_index", v.first_path_index);
    s.add_u16(p + ".first_path_window", v.first_path_window);
    s.add_opt_u32(p + ".vendor_pac_value", v.vendor_pac_value);
    s.add(p + ".vendor_pac_backend", v.vendor_pac_backend);
    s.add_opt_double(p + ".vendor_pac_applied_step", v.vendor_pac_applied_step);
}

inline void collect_fields(FieldSink& s, const EndpointBinding& v, const std::string& p)
{
    s.add(p + ".id", v.id);
    s.add_enum(p + ".role", role_to_string(v.role));
    s.add_u8(p + ".tx_channel", v.tx_channel);
    s.add_u8(p + ".rx_channel", v.rx_channel);
    s.add_double(p + ".native_sample_rate_hz", v.native_sample_rate_hz);
    s.add_bool(p + ".occupies_resources", v.occupies_resources);
}

inline void collect_fields(FieldSink& s, const RadioReadback& v, const std::string& p)
{
    s.add_bool(p + ".present", v.present);
    s.add_double(p + ".sample_rate_hz", v.sample_rate_hz);
    s.add_double(p + ".center_freq_hz", v.center_freq_hz);
    s.add_u8(p + ".tx_channel", v.tx_channel);
    s.add_u8(p + ".rx_channel", v.rx_channel);
    s.add(p + ".mpm_string", v.mpm_string);
    s.add(p + ".fpga_image", v.fpga_image);
    s.add(p + ".uhd_version", v.uhd_version);
    s.add(p + ".clock_source", v.clock_source);
    s.add(p + ".time_source", v.time_source);
}

inline void collect_fields(FieldSink& s, const RadioConfig& v, const std::string& p)
{
    s.add(p + ".device_args", v.device_args);
    s.add_u8(p + ".tx_channel", v.tx_channel);
    s.add_u8(p + ".rx_channel", v.rx_channel);
    s.add_double(p + ".native_sample_rate_hz", v.native_sample_rate_hz);
    s.add(p + ".clock_source", v.clock_source);
    s.add(p + ".time_source", v.time_source);
    s.add(p + ".fpga_image", v.fpga_image);
    s.add(p + ".dpdk_config", v.dpdk_config);
    for (size_t i = 0; i < v.peers.size(); ++i)
        collect_fields(s, v.peers[i], p + ".peers[" +
            twr_int_to_text(static_cast<int64_t>(i)) + "]");
    collect_fields(s, v.readback, p + ".readback");
    s.add_bool(p + ".require_readback", v.require_readback);
}

inline void collect_fields(FieldSink& s, const PerMessageTiming& v,
    const std::string& p)
{
    collect_timed_fields(s, p + ".poll_start", v.poll_start);
    collect_timed_fields(s, p + ".poll_to_response", v.poll_to_response);
    collect_timed_fields(s, p + ".response_to_final", v.response_to_final);
    collect_timed_fields(s, p + ".final_to_report", v.final_to_report);
    collect_timed_fields(s, p + ".post_tx_rx_enable", v.post_tx_rx_enable);
    collect_timed_fields(s, p + ".min_tx_lead_time", v.min_tx_lead_time);
}

inline void collect_fields(FieldSink& s, const TimeoutConfig& v, const std::string& p)
{
    collect_timed_fields(s, p + ".poll_rx_window", v.poll_rx_window);
    collect_timed_fields(s, p + ".response_rx_window", v.response_rx_window);
    collect_timed_fields(s, p + ".final_rx_window", v.final_rx_window);
    collect_timed_fields(s, p + ".report_rx_window", v.report_rx_window);
    collect_timed_fields(s, p + ".rx_timeout", v.rx_timeout);
    collect_timed_fields(s, p + ".exchange_timeout", v.exchange_timeout);
    collect_timed_fields(s, p + ".retry_interval", v.retry_interval);
}

inline void collect_fields(FieldSink& s, const CalibrationRecord& v,
    const std::string& p)
{
    s.add(p + ".calibration_id", v.calibration_id);
    s.add(p + ".device_serial", v.device_serial);
    s.add_u8(p + ".channel", v.channel);
    s.add_double(p + ".native_sample_rate_hz", v.native_sample_rate_hz);
    s.add(p + ".profile_version", v.profile_version);
    s.add_opt_double(p + ".gain_db", v.gain_db);
    s.add_i64(p + ".valid_until_monotonic_ns", v.valid_until_monotonic_ns);
}

inline void collect_fields(FieldSink& s, const TimestampCalibrationConfig& v,
    const std::string& p)
{
    s.add_dur(p + ".tx_link_delay", v.tx_link_delay);
    s.add_dur(p + ".rx_link_delay", v.rx_link_delay);
    s.add_dur(p + ".antenna_delay", v.antenna_delay);
    s.add_dur(p + ".cable_delay", v.cable_delay);
    s.add_i64(p + ".tx_link_delay_native_ticks", v.tx_link_delay_native_ticks);
    s.add_i64(p + ".rx_link_delay_native_ticks", v.rx_link_delay_native_ticks);
    s.add_double(p + ".native_sample_rate_hz", v.native_sample_rate_hz);
    s.add_enum(p + ".link_delay_unit", time_unit_to_string(v.link_delay_unit));
    s.add_enum(p + ".first_path_algorithm",
        first_path_algorithm_to_string(v.first_path_algorithm));
    s.add_enum(p + ".cfo_compensation",
        compensation_flag_to_string(v.cfo_compensation));
    s.add_enum(p + ".sfo_compensation",
        compensation_flag_to_string(v.sfo_compensation));
    s.add(p + ".calibration_id", v.calibration_id);
    collect_fields(s, v.record, p + ".record");
    s.add_u32(p + ".applied_count", v.applied_count);
    s.add_bool(p + ".calibration_required", v.calibration_required);
}

inline void collect_fields(FieldSink& s, const DiagnosticsConfig& v,
    const std::string& p)
{
    s.add_bool(p + ".cir_capture_enabled", v.cir_capture_enabled);
    s.add_u64(p + ".cir_capture_max_bytes", v.cir_capture_max_bytes);
    s.add_u32(p + ".cir_capture_stride", v.cir_capture_stride);
    s.add_bool(p + ".short_iq_enabled", v.short_iq_enabled);
    s.add_u64(p + ".short_iq_max_bytes", v.short_iq_max_bytes);
    s.add_u32(p + ".short_iq_stride", v.short_iq_stride);
    s.add_bool(p + ".raw_frame_dump", v.raw_frame_dump);
    s.add_u64(p + ".raw_frame_max_bytes", v.raw_frame_max_bytes);
    s.add(p + ".result_output_path", v.result_output_path);
    s.add_u32(p + ".result_queue_capacity", v.result_queue_capacity);
    s.add_u32(p + ".event_queue_capacity", v.event_queue_capacity);
    collect_timed_fields(s, p + ".stats_cadence", v.stats_cadence);
    s.add_bool(p + ".io_on_realtime_thread", v.io_on_realtime_thread);
}

inline void collect_fields(FieldSink& s, const TwrConfig& v, const std::string& p)
{
    collect_fields(s, v.meta, p + "meta");
    collect_fields(s, v.session, p + "session");
    collect_fields(s, v.phy, p + "phy");
    collect_fields(s, v.frame, p + "frame");
    collect_fields(s, v.tx, p + "tx");
    collect_fields(s, v.rx, p + "rx");
    collect_fields(s, v.radio, p + "radio");
    collect_fields(s, v.timing, p + "timing");
    collect_fields(s, v.timeouts, p + "timeouts");
    collect_fields(s, v.calibration, p + "calibration");
    collect_fields(s, v.diagnostics, p + "diagnostics");
}

// Ordered "path=value" listing.  Stable across runs and across builds, so it
// can be hashed and diffed.
inline std::vector<FieldRecord> flatten_fields(const TwrConfig& cfg)
{
    std::vector<FieldRecord> out;
    out.reserve(256);
    FieldSink s(out);
    collect_fields(s, cfg, std::string());
    return out;
}

inline std::string canonical_text(const std::vector<FieldRecord>& fields)
{
    std::string out;
    out.reserve(fields.size() * 24);
    for (const auto& f : fields) {
        out += f.path;
        out += '=';
        out += f.value;
        out += '\n';
    }
    return out;
}

// FNV-1a 64 over the canonical text.  Not a security hash: an identifier that
// appears in every result so a config can be tied to the run that used it.
inline std::string config_hash(const TwrConfig& cfg)
{
    const std::string text = canonical_text(flatten_fields(cfg));
    uint64_t h = 1469598103934665603ull;
    for (unsigned char c : text) {
        h ^= static_cast<uint64_t>(c);
        h *= 1099511628211ull;
    }
    char buf[32];
    std::snprintf(buf, sizeof(buf), "fnv1a64:%016llx",
        static_cast<unsigned long long>(h));
    return std::string(buf);
}

// One requested-vs-effective difference.
struct FieldChange {
    std::string path;
    std::string requested;
    std::string effective;
    std::string note;
    bool adjusted() const { return requested != effective; }
};

inline std::vector<FieldChange>
diff_fields(const TwrConfig& requested, const TwrConfig& effective)
{
    const std::vector<FieldRecord> a = flatten_fields(requested);
    const std::vector<FieldRecord> b = flatten_fields(effective);
    std::vector<FieldChange> out;
    const size_t n = a.size() < b.size() ? a.size() : b.size();
    for (size_t i = 0; i < n; ++i) {
        if (a[i].path != b[i].path || a[i].value == b[i].value)
            continue;
        FieldChange c;
        c.path = a[i].path;
        c.requested = a[i].value;
        c.effective = b[i].value;
        out.push_back(c);
    }
    return out;
}

// ===========================================================================
// 5b. Enum-domain checks (M0.1, review finding N07)
// ===========================================================================
//
// Why this section exists
// -----------------------
// Every enum below is validated ONLY at the JSON reader (`get_enum` ->
// `from_string`).  A value that reaches a `TwrConfig` by DIRECT CONSTRUCTION --
// `c.rx.agc = static_cast<AgcMode>(250)`, or a bad async-event / status mapping
// in M1 -- went through `validate()` with no domain check at all.  The fields
// whose gates happen to be allow-lists were refused, but the ones gated by an
// equality test (`if (r.agc == AgcMode::Manual)`) or not gated at all were
// ACCEPTED.  That is fail-open in the single gate that admits a configuration,
// and it contradicted the layer's own contract: such a config serialises as
// "invalid" and is then REFUSED by this layer's own reader.
//
// Same root cause as the ranging gate's `switch (ev->outcome)` with no
// `default`: an enum's DOMAIN was treated as if its enumerators were exhaustive
// of what a variable can hold.  They are not.
//
// Why a `switch` and not `<= kXxxMax`
// -----------------------------------
// These switches have NO `default`, exactly like `timestamp_marker_is_known` in
// uwb_twr_timestamp.h.  Adding an enumerator to one of these enums therefore
// makes -Wswitch fire instead of silently widening the accepted domain.  A
// `<= last` form would need a second constant kept in sync, and a `default:`
// would silence the warning that makes this safe.
//
// `frame_profile_id_is_supported()` in uwb_twr_frame.h already does this for
// FrameProfileId, so there is no duplicate here.

inline bool prf_class_is_known(PrfClass v)
{
    switch (v) {
    case PrfClass::Bprf64: case PrfClass::Hprf64: return true;
    case PrfClass::Hprf400: return true;
    }
    return false;
}

inline bool data_rate_is_known(DataRate v)
{
    switch (v) {
    case DataRate::R850k: case DataRate::R6p8M: return true;
    case DataRate::R27M: case DataRate::R7p8M: return true;
    case DataRate::R27p2M: case DataRate::R6p8M_hprf: return true;
    }
    return false;
}

inline bool phr_rate_is_known(PhrRate v)
{
    switch (v) {
    case PhrRate::Standard850k: case PhrRate::SameAsData: return true;
    }
    return false;
}

inline bool sfd_mode_is_known(SfdMode v)
{
    switch (v) {
    case SfdMode::R4z1: case SfdMode::R4z2: return true;
    case SfdMode::R4z3: case SfdMode::R4z4: return true;
    case SfdMode::Dwt8: case SfdMode::Ieee8: return true;
    }
    return false;
}

inline bool phr_mode_is_known(PhrMode v)
{
    switch (v) {
    case PhrMode::Standard: case PhrMode::Extended: return true;
    case PhrMode::None: return true;
    }
    return false;
}

inline bool fcs_appender_is_known(FcsAppender v)
{
    switch (v) {
    case FcsAppender::None: case FcsAppender::MacLayer: return true;
    case FcsAppender::PhyLayer: return true;
    }
    return false;
}

inline bool sts_mode_is_known(StsMode v)
{
    switch (v) {
    case StsMode::Off: case StsMode::Sp64: return true;
    case StsMode::Sp128: case StsMode::Sp256: return true;
    case StsMode::Sp512: case StsMode::Sp1024: return true;
    }
    return false;
}

inline bool tx_power_policy_is_known(TxPowerPolicy v)
{
    switch (v) {
    case TxPowerPolicy::LeaveUntouched: case TxPowerPolicy::ManualGainDb: return true;
    case TxPowerPolicy::IqAmplitude: case TxPowerPolicy::CalibratedDbm: return true;
    }
    return false;
}

inline bool pulse_shaping_is_known(PulseShaping v)
{
    switch (v) {
    case PulseShaping::ExistingHrP: case PulseShaping::Rectangular: return true;
    case PulseShaping::RootRaisedCosine: return true;
    }
    return false;
}

inline bool agc_mode_is_known(AgcMode v)
{
    switch (v) {
    case AgcMode::Manual: case AgcMode::Disabled: return true;
    case AgcMode::VendorDefault: return true;
    }
    return false;
}

inline bool first_path_algorithm_is_known(FirstPathAlgorithm v)
{
    switch (v) {
    case FirstPathAlgorithm::LeadingEdge: case FirstPathAlgorithm::Peak: return true;
    case FirstPathAlgorithm::EnergyCentroid: case FirstPathAlgorithm::InterpolatedPeak: return true;
    case FirstPathAlgorithm::StrongestCluster: return true;
    }
    return false;
}

inline bool compensation_flag_is_known(CompensationFlag v)
{
    switch (v) {
    case CompensationFlag::Off: case CompensationFlag::On: return true;
    case CompensationFlag::Required: return true;
    }
    return false;
}

inline bool time_unit_is_known(TimeUnit v)
{
    switch (v) {
    case TimeUnit::Nanoseconds: case TimeUnit::Seconds: return true;
    case TimeUnit::NativeTicks: return true;
    }
    return false;
}

inline bool time_reference_event_is_known(TimeReferenceEvent v)
{
    switch (v) {
    case TimeReferenceEvent::PollTransmitRmarker: case TimeReferenceEvent::PollReceiveRmarker: return true;
    case TimeReferenceEvent::ResponseTransmitRmarker: case TimeReferenceEvent::ResponseReceiveRmarker: return true;
    case TimeReferenceEvent::FinalTransmitRmarker: case TimeReferenceEvent::FinalReceiveRmarker: return true;
    case TimeReferenceEvent::ReportTransmitRmarker: case TimeReferenceEvent::ReportReceiveRmarker: return true;
    case TimeReferenceEvent::FrameTail: case TimeReferenceEvent::RxEnable: return true;
    case TimeReferenceEvent::HostMonotonic: return true;
    }
    return false;
}

inline bool time_domain_is_known(TimeDomain v)
{
    switch (v) {
    case TimeDomain::Unspecified: case TimeDomain::DeviceTicks: return true;
    case TimeDomain::MonotonicHost: return true;
    }
    return false;
}

inline bool protocol_is_known(Protocol v)
{
    switch (v) {
    case Protocol::Ss: case Protocol::Ds: return true;
    }
    return false;
}

inline bool role_is_known(Role v)
{
    switch (v) {
    case Role::Initiator: case Role::Responder: return true;
    }
    return false;
}

} // namespace twr
} // namespace openuwb

#endif /* INCLUDED_OPENUWB_TWR_CONFIG_FIELDS_H */
