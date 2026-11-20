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
#ifndef INCLUDED_OPENUWB_TWR_CONFIG_VALIDATOR_H
#define INCLUDED_OPENUWB_TWR_CONFIG_VALIDATOR_H

#include <openuwb/twr/twr_config_fields.h>
namespace gr {
namespace uwb {
namespace twr {

// ===========================================================================
// 6. The validator
// ===========================================================================

// Every check appends a specific violation.  There is no early return: a
// config that is wrong in many places is reported in full so the operator
// fixes them in one pass.
class ConfigValidator
{
public:
    explicit ConfigValidator(const Capabilities& caps) : d_caps(caps) {}

    ValidationReport run(const TwrConfig& cfg) const
    {
        d_report = ValidationReport{};
        // FIRST: an out-of-domain enum makes every later check meaningless --
        // a value the enum does not have cannot be reasoned about.  Catching it
        // here also means the refusal NAMES the enum field instead of surfacing
        // as a confusing complaint about some field that depends on it (an
        // unknown `session.protocol`, for instance, used to be reported as a
        // problem with `timing.response_to_final.ns`).
        check_enum_domains(cfg);
        check_meta(cfg);
        check_session(cfg);
        check_phy(cfg);
        check_frame(cfg);
        check_tx(cfg);
        check_rx(cfg);
        check_radio(cfg);
        check_timing(cfg);
        check_timeouts(cfg);
        check_calibration(cfg);
        check_diagnostics(cfg);
        check_frame_lengths(cfg);
        return d_report;
    }

private:
    // -- enum domains (N07) ----------------------------------------------
    //
    // One place that refuses an enum value the enum does not have, for EVERY
    // enum field in the config.  Fourteen of these were verified to be
    // accepted outright before this pass existed, and a further few were
    // refused only incidentally, with the violation naming an unrelated field.
    //
    // The check is deliberately unconditional: it does not matter whether the
    // value came from a cast, a deserialiser, an async-event mapping or a
    // future enum member whose gate was not updated.  `UnknownEnumValue` is the
    // same reason the JSON reader uses, so a caller sees one vocabulary
    // whichever path produced the bad value.
    void check_enum(const std::string& f, bool known, const char* what) const
    {
        if (!known)
            cfg_rej(f, ConfigReason::UnknownEnumValue,
                    std::string("value is not a member of ") + what +
                        "; an out-of-domain enum is refused rather than reasoned "
                        "about (it would serialise as \"invalid\" and be refused "
                        "by the reader, so accepting it would contradict this "
                        "layer's own contract)",
                    "REQ-API-01");
    }

    void check_enum_domains(const TwrConfig& c) const
    {
        check_enum("session.protocol", protocol_is_known(c.session.protocol),
                   "Protocol");
        check_enum("session.role", role_is_known(c.session.role), "Role");
        check_enum("phy.prf_class", prf_class_is_known(c.phy.prf_class),
                   "PrfClass");
        check_enum("phy.data_rate", data_rate_is_known(c.phy.data_rate),
                   "DataRate");
        check_enum("phy.phr_rate", phr_rate_is_known(c.phy.phr_rate),
                   "PhrRate");
        check_enum("frame.frame_profile",
                   frame_profile_id_is_supported(c.frame.frame_profile),
                   "FrameProfileId");
        check_enum("frame.sfd_mode", sfd_mode_is_known(c.frame.sfd_mode),
                   "SfdMode");
        check_enum("frame.phr_mode", phr_mode_is_known(c.frame.phr_mode),
                   "PhrMode");
        check_enum("frame.fcs_append",
                   fcs_appender_is_known(c.frame.fcs_append), "FcsAppender");
        check_enum("frame.sts_mode", sts_mode_is_known(c.frame.sts_mode),
                   "StsMode");
        check_enum("tx.power_policy",
                   tx_power_policy_is_known(c.tx.power_policy),
                   "TxPowerPolicy");
        check_enum("tx.pulse_shaping",
                   pulse_shaping_is_known(c.tx.pulse_shaping), "PulseShaping");
        check_enum("rx.agc", agc_mode_is_known(c.rx.agc), "AgcMode");
        check_enum("calibration.link_delay_unit",
                   time_unit_is_known(c.calibration.link_delay_unit),
                   "TimeUnit");
        check_enum("calibration.first_path_algorithm",
                   first_path_algorithm_is_known(
                       c.calibration.first_path_algorithm),
                   "FirstPathAlgorithm");
        check_enum("calibration.cfo_compensation",
                   compensation_flag_is_known(c.calibration.cfo_compensation),
                   "CompensationFlag");
        check_enum("calibration.sfo_compensation",
                   compensation_flag_is_known(c.calibration.sfo_compensation),
                   "CompensationFlag");

        // Every TimedField carries a (domain, reference) pair; both are enums,
        // and both are reachable by direct construction.
        check_timed_enums("frame.sfd_timeout", c.frame.sfd_timeout);
        check_timed_enums("timing.poll_start", c.timing.poll_start);
        check_timed_enums("timing.poll_to_response", c.timing.poll_to_response);
        check_timed_enums("timing.response_to_final", c.timing.response_to_final);
        check_timed_enums("timing.final_to_report", c.timing.final_to_report);
        check_timed_enums("timing.post_tx_rx_enable",
                          c.timing.post_tx_rx_enable);
        check_timed_enums("timing.min_tx_lead_time", c.timing.min_tx_lead_time);
        check_timed_enums("timeouts.poll_rx_window",
                          c.timeouts.poll_rx_window);
        check_timed_enums("timeouts.response_rx_window",
                          c.timeouts.response_rx_window);
        check_timed_enums("timeouts.final_rx_window",
                          c.timeouts.final_rx_window);
        check_timed_enums("timeouts.report_rx_window",
                          c.timeouts.report_rx_window);
        check_timed_enums("timeouts.rx_timeout", c.timeouts.rx_timeout);
        check_timed_enums("timeouts.exchange_timeout",
                          c.timeouts.exchange_timeout);
        check_timed_enums("timeouts.retry_interval", c.timeouts.retry_interval);
        check_timed_enums("diagnostics.stats_cadence",
                          c.diagnostics.stats_cadence);

        // The peer bindings repeat one of the same enums, once per endpoint.
        for (size_t i = 0; i < c.radio.peers.size(); ++i) {
            const std::string p = "radio.peers[" + twr_int_to_text(
                                      static_cast<int64_t>(i)) + "]";
            check_enum(p + ".role", role_is_known(c.radio.peers[i].role),
                       "Role");
        }
    }

    void check_timed_enums(const std::string& f, const TimedField& t) const
    {
        check_enum(f + ".domain", time_domain_is_known(t.domain), "TimeDomain");
        check_enum(f + ".reference",
                   time_reference_event_is_known(t.reference),
                   "TimeReferenceEvent");
        // `marker` is OPTIONAL -- absent is a legal state (there is no RF marker
        // on a host-monotonic clock) -- but when it is present it must be a
        // value the enum has.  Missing this third enum was worth twelve more
        // accepted configurations (an exhaustive sweep over every enum-typed
        // field is what found it; covering domain+reference and assuming the
        // pair was complete was the mistake).
        if (t.marker.has())
            check_enum(f + ".marker",
                       timestamp_marker_is_known(t.marker.value()),
                       "TimestampMarker");
    }

    // -- helpers ---------------------------------------------------------
    void rej(const std::string& f,
             ConfigReason r,
             ExchangeStatus s,
             const std::string& m,
             const char* req) const
    {
        d_report.add(f, r, s, m, req);
    }
    void cfg_rej(const std::string& f,
                 ConfigReason r,
                 const std::string& m,
                 const char* req = "REQ-API-01") const
    {
        rej(f, r, ExchangeStatus::ConfigRejected, m, req);
    }
    void unsup(const std::string& f,
               const std::string& m,
               ConfigReason r = ConfigReason::Unsupported,
               const char* req = "REQ-SCOPE-01") const
    {
        rej(f, r, ExchangeStatus::Unsupported, m, req);
    }
    // A float field that must be finite.
    void finite(const std::string& f, double v, const char* req = "REQ-API-01") const
    {
        if (!std::isfinite(v))
            cfg_rej(f, ConfigReason::NotFinite, "must be finite (NaN/Inf rejected)",
                req);
    }
    void check_finite_opt(const std::string& f, const Opt<double>& o) const
    {
        if (o.has() && !std::isfinite(o.value()))
            cfg_rej(f, ConfigReason::NotFinite,
                "must be finite when present (NaN/Inf rejected)");
    }
    void check_finite_timed(const std::string& f, const TimedField& t) const
    {
        finite(f + ".quantisation_hz", t.required_quantisation_hz);
        if (t.value.negative())
            cfg_rej(f, ConfigReason::NegativeValue,
                "a delay or window may not be negative");
    }

    // -- sections --------------------------------------------------------
    void check_meta(const TwrConfig& c) const
    {
        if (c.meta.schema_version.empty())
            cfg_rej("meta.schema_version", ConfigReason::EmptyValue,
                "schema_version is required");
        if (c.meta.schema_version != d_caps.schema_version) {
            // M0.1: twr-config/2 is a BREAKING change -- frame.frame_profile
            // became required and phy.phr_rate changed value domain.  A v1
            // document is refused with a migration message, never silently
            // misread as a v2 one.
            if (c.meta.schema_version == "twr-config/1")
                unsup("meta.schema_version",
                      "twr-config/1 is superseded by twr-config/2: "
                      "frame.frame_profile is now REQUIRED (frame_v1) and "
                      "phy.phr_rate now takes 850k (the PHR rate the modulator "
                      "actually uses) rather than the payload rate; the frame "
                      "geometry moved from the old 7-byte header to the frozen "
                      "14-byte header, so on-air sizes are 16/26/31 B and "
                      "session.session_id must fit 16 bits. This document is "
                      "refused rather than reinterpreted");
            else
                unsup("meta.schema_version",
                      "this build implements " + d_caps.schema_version +
                          ", config asks for '" + c.meta.schema_version + "'");
        }
        if (c.meta.profile_version.empty())
            cfg_rej("meta.profile_version",
                    ConfigReason::EmptyValue,
                    "profile_version is required (REQ-OUT-01 calibration/profile/config hash)");
    }

    void check_session(const TwrConfig& c) const
    {
        const SessionConfig& s = c.session;
        if (s.local_address == s.peer_address)
            cfg_rej("session.peer_address",
                    ConfigReason::FieldConflict,
                    "local and peer address are identical (would make every frame a match)");
        if (s.local_address == 0x0000 || s.local_address == 0xffff)
            cfg_rej("session.local_address",
                    ConfigReason::FieldConflict,
                    "0x0000 and 0xffff are reserved/invalid local addresses");
        if (s.peer_address == 0x0000 || s.peer_address == 0xffff)
            cfg_rej("session.peer_address",
                    ConfigReason::FieldConflict,
                    "0x0000 and 0xffff are reserved/invalid peer addresses");
        if (s.pan_id == 0x0000 || s.pan_id == 0xffff)
            cfg_rej("session.pan_id", ConfigReason::EmptyValue,
                "PAN 0x0000/0xffff is not usable");
        if (s.session_id == 0)
            cfg_rej("session.session_id",
                    ConfigReason::ZeroValue,
                    "0 is the frame profile's 'no session' marker");
        else {
            // The wire field at kOffSessionId is 16 bits.  session_id_to_wire()
            // is identity-or-refuse: in range -> exact, out of range -> an
            // explicit error.  It is NOT folded, masked or hashed, because a
            // lossy map makes two different local sessions produce a
            // byte-identical session field and frame_match() would then accept
            // the wrong session's reply as this one's.
            uint16_t wire = 0;
            std::string sid_err_text;
            SessionIdError sid_err = SessionIdError::None;
            if (!session_id_to_wire(s.session_id, wire, sid_err_text, &sid_err)) {
                rej("session.session_id",
                    ConfigReason::OutOfRange,
                    session_id_error_to_exchange_status(sid_err),
                    sid_err_text,
                    "REQ-PROTO-01");
            }
        }
        if (s.sequence_modulus == 0)
            cfg_rej("session.sequence_modulus", ConfigReason::ZeroValue,
                "wrap modulus is zero");
        else if (s.sequence_modulus != 4 && s.sequence_modulus != 16 &&
                 s.sequence_modulus != 64 && s.sequence_modulus != 256)
            cfg_rej("session.sequence_modulus",
                    ConfigReason::OutOfRange,
                    "must be 4, 16, 64 or 256 (2/4/6/8 sequence bits)");
        if (static_cast<uint16_t>(s.sequence) >= s.sequence_modulus)
            cfg_rej("session.sequence",
                    ConfigReason::IndexOutOfRange,
                    "sequence does not fit the wrap modulus");
        if (s.measurement_count == 0 || s.measurement_count > kTwrMaxMeasurementCount)
            cfg_rej("session.measurement_count",
                    ConfigReason::OutOfRange,
                    "must be 1.." + twr_int_to_text(kTwrMaxMeasurementCount));
        if (s.measurement_interval.negative())
            cfg_rej("session.measurement_interval", ConfigReason::NegativeValue,
                "negative interval");
        if (s.max_attempts_per_exchange == 0 ||
            s.max_attempts_per_exchange > kTwrMaxAttemptsPerExchange)
            cfg_rej("session.max_attempts_per_exchange",
                    ConfigReason::OutOfRange,
                    "must be 1.." + twr_int_to_text(kTwrMaxAttemptsPerExchange));
        if (s.retry_backoff.negative())
            cfg_rej("session.retry_backoff", ConfigReason::NegativeValue,
                "negative backoff");
        if (s.max_attempts_per_exchange > 1) {
            if (s.retry_backoff.is_zero())
                cfg_rej("session.retry_backoff",
                        ConfigReason::FieldConflict,
                        "a retry policy with more than one attempt needs a non-zero backoff");
        } else if (!s.retry_backoff.is_zero()) {
            cfg_rej("session.retry_backoff",
                    ConfigReason::FieldConflict,
                    "retry_backoff is set but max_attempts_per_exchange is 1");
        }
        if (s.max_in_flight_exchanges != kTwrMaxInFlightExchanges)
            unsup("session.max_in_flight_exchanges",
                  "phase 1 allows at most " + twr_int_to_text(kTwrMaxInFlightExchanges) +
                      " in-flight exchange per endpoint; requested " +
                      twr_int_to_text(s.max_in_flight_exchanges),
                  ConfigReason::InFlightNotSupported,
                  "REQ-API-01");
    }

    void check_phy(const TwrConfig& c) const
    {
        const PhyConfig& p = c.phy;
        if (p.channel > 16u)
            cfg_rej("phy.channel", ConfigReason::IndexOutOfRange,
                "channel must be 0..16");
        else if (!d_caps.channel_supported(p.channel))
            unsup("phy.channel",
                  "channel " + twr_int_to_text(p.channel) +
                      " is not in this build's capability whitelist; the phase-1 common profile "
                      "is channel 5 (REQ-PHY-01/03) and other channels are per-channel "
                      "acceptance items");

        // Channel vs centre frequency: a conflict is rejected, never ignored.
        finite("phy.center_frequency_hz", p.center_frequency_hz, "REQ-PHY-01");
        if (std::isfinite(p.center_frequency_hz) && p.center_frequency_hz > 0.0) {
            const double plan = uwb_channel_center_frequency_hz(p.channel);
            if (plan <= 0.0) {
                cfg_rej("phy.center_frequency_hz",
                        ConfigReason::IndexOutOfRange,
                        "no channel plan entry for this channel");
            } else if (std::fabs(p.center_frequency_hz - plan) > kTwrChannelFrequencyToleranceHz) {
                cfg_rej("phy.center_frequency_hz",
                        ConfigReason::ChannelFrequencyMismatch,
                        "channel " + twr_int_to_text(p.channel) + " is " +
                            twr_double_to_text(plan) + " Hz, config states " +
                            twr_double_to_text(p.center_frequency_hz) + " Hz",
                        "REQ-PHY-01");
            }
        }

        // code / PRF / channel / profile are cross-validated together.
        for (int dir = 0; dir < 2; ++dir) {
            const std::string f =
                dir == 0 ? "phy.tx_preamble_code" : "phy.rx_preamble_code";
            const uint8_t code = dir == 0 ? p.tx_preamble_code : p.rx_preamble_code;
            if (code == 0)
                cfg_rej(f, ConfigReason::ZeroValue, "preamble code is required");
            else if (!d_caps.code_index_supported(code))
                unsup(f,
                      "preamble code " + twr_int_to_text(code) +
                          " is not in the capability whitelist (" +
                          twr_int_to_text(kTwrCodeIndexMin) + ".." +
                          twr_int_to_text(kTwrCodeIndexMax) + ")");
        }
        if (p.tx_preamble_code != 0 && p.rx_preamble_code != 0 &&
            p.tx_preamble_code != p.rx_preamble_code)
            cfg_rej("phy.rx_preamble_code",
                    ConfigReason::FieldConflict,
                    "TX and RX preamble codes differ (" + twr_int_to_text(p.tx_preamble_code) +
                        " vs " + twr_int_to_text(p.rx_preamble_code) +
                        "); one profile uses one code",
                    "REQ-PHY-01");
        if (!d_caps.prf_class_supported(p.prf_class))
            unsup("phy.prf_class",
                  std::string("PRF class ") + prf_class_to_string(p.prf_class) +
                      " is not in the capability whitelist");
        // The PHR rate is validated on its OWN axis.  It is NOT compared to
        // the payload data rate: the PHR is a different modulation at a
        // different rate, and the data-rate field INSIDE the PHR describes
        // the payload, not the PHR's own transmission.  M0's `phr_rate ==
        // data_rate` rule therefore accepted the 6.81 Mb/s PHR this modulator
        // cannot produce and rejected the 0.85 Mb/s PHR it does produce.
        if (!phr_rate_is_implemented(p.phr_rate)) {
            unsup("phy.phr_rate",
                  "PHR rate " + std::string(phr_rate_to_string(p.phr_rate)) +
                      " is not implemented: " + phr_rate_unsupported_reason(p.phr_rate) +
                      " It is NOT compared with phy.data_rate (" +
                      data_rate_to_string(p.data_rate) +
                      "): the PHR and the payload are separate modulations, and the "
                      "data-rate field carried inside the PHR describes the payload",
                  ConfigReason::Unsupported,
                  "REQ-PHY-02");
        }

        // MEASURED-PENDING: sync repetitions / SFD mode / data rate.
        if (!d_caps.data_rate_supported(p.data_rate))
            unsup("phy.data_rate",
                  std::string("data rate ") + data_rate_to_string(p.data_rate) +
                      " is not in the capability whitelist: " + d_caps.pending_reason,
                  ConfigReason::Unsupported,
                  "REQ-PHY-01");
        if (p.preamble_symbols == 0)
            cfg_rej("phy.preamble_symbols", ConfigReason::ZeroValue,
                "preamble length is required");
        else {
            // Gate 1: the length must be a member of PreambleLength, i.e. one
            // this repository's demodulator / CFO path can actually run.  The
            // reason names THAT limit and never a chip's capability: the
            // Qorvo parts' own API accepts 64 through 2048, so "the hardware
            // cannot do 128" would be a false statement about hardware this
            // project has never spoken to.
            PreambleLength plen = PreambleLength::Sym64;
            if (!preamble_length_from_symbols(p.preamble_symbols, plen)) {
                unsup("phy.preamble_symbols",
                      preamble_length_unsupported_reason(p.preamble_symbols),
                      ConfigReason::Unsupported,
                      "REQ-PHY-01");
            }
            // Gate 2: the length must also be MEASURED for this (native rate,
            // code, SFD, PSDU, ranging) combination.  Separate from gate 1 on
            // purpose: a length can be implemented and still unmeasured.
            if (!d_caps.sync_repetitions_supported(p.preamble_symbols)) {
                // M0.1 R7: state the SCOPE of the refusal.  For the lengths
                // Qorvo parts do support, this build's decoder / CFO path is
                // the limit -- that is not a hardware claim, and the note
                // carries the vendor citation so C++ and the CSV agree.
                std::string note = evidence::preamble_length_supported_by_vendor(
                                       p.preamble_symbols)
                                       ? std::string(" [") +
                                             evidence::preamble_length_reject_note(
                                                 p.preamble_symbols) +
                                             "]"
                                       : std::string();
                unsup("phy.preamble_symbols",
                      "preamble length " + twr_int_to_text(p.preamble_symbols) +
                          " is not in the capability whitelist: " +
                          d_caps.pending_reason + note,
                      ConfigReason::Unsupported,
                      "REQ-PHY-01");
            }
        }
    }

    void check_frame(const TwrConfig& c) const
    {
        const FrameFormatConfig& f = c.frame;

        // ---- which layout? -------------------------------------------------
        // The profile is an ENUM, so "an arbitrary non-empty profile string"
        // is not expressible.  An id this build has no implementation for is
        // refused with its name, never defaulted to frame v1.
        if (!frame_profile_id_is_supported(f.frame_profile)) {
            unsup("frame.frame_profile",
                  "frame profile " + std::string(frame_profile_id_to_string(f.frame_profile)) +
                      " is not implemented: only the ids named by "
                      "frame_profile_id_to_string() have a codec, a geometry authority and an "
                      "encoder in this build. A free-form profile name is never an executable "
                      "geometry",
                  ConfigReason::Unsupported,
                  "REQ-PROTO-06");
        }
        // The authority this config is checked against, and the codec profile
        // whose timestamp width it reports.  Both come from the enum; neither
        // is read out of the config.
        FrameProfileGeometry authority;
        FrameProfile profile;
        const bool have_profile =
            frame_profile_id_is_supported(f.frame_profile) &&
            frame_geometry_for(f.frame_profile, authority) &&
            frame_profile_for(f.frame_profile, profile);

        if (!d_caps.sfd_mode_supported(f.sfd_mode))
            unsup("frame.sfd_mode",
                  std::string("SFD mode ") + sfd_mode_to_string(f.sfd_mode) +
                      " is not in the capability whitelist: " + d_caps.pending_reason,
                  ConfigReason::Unsupported,
                  "REQ-PHY-01");
        // The raw SFD length is a RESTATEMENT of the mode, never an
        // independent number: "4z2" and 4 symbols cannot both be stated.
        if (f.sfd_symbols == 0)
            cfg_rej("frame.sfd_symbols", ConfigReason::ZeroValue,
                "SFD length is required");
        else if (f.sfd_symbols != sfd_mode_symbols(f.sfd_mode))
            cfg_rej("frame.sfd_symbols",
                    ConfigReason::FieldConflict,
                    std::string("SFD mode ") + sfd_mode_to_string(f.sfd_mode) + " is " +
                        twr_int_to_text(sfd_mode_symbols(f.sfd_mode)) + " symbols, config states " +
                        twr_int_to_text(f.sfd_symbols) +
                        "; the length is derived from the mode, not chosen beside it",
                    "REQ-PHY-01");

        // PHR presence/form.  Each unimplemented member carries its own reason
        // (phr_mode_unsupported_reason); M0 caught only `none` and let
        // `extended` through as a valid profile.
        if (f.phr_mode != PhrMode::Standard) {
            unsup("frame.phr_mode",
                  std::string("PHR mode ") + phr_mode_to_string(f.phr_mode) +
                      " is not implemented: " + phr_mode_unsupported_reason(f.phr_mode),
                  f.phr_mode == PhrMode::None ? ConfigReason::OutOfScope
                                              : ConfigReason::Unsupported,
                  "REQ-PHY-01");
        }
        if (d_caps.ranging_bit_required && !f.ranging_bit)
            cfg_rej("frame.ranging_bit",
                    ConfigReason::FieldConflict,
                    "a TWR MAC frame must set the ranging bit; ranging_bit=false is not a valid "
                    "TWR profile",
                    "REQ-PROTO-05");
        if (!d_caps.ranging_bit_required && f.ranging_bit)
            cfg_rej("frame.ranging_bit",
                    ConfigReason::FieldConflict,
                    "this build's whitelist carries no ranging frame",
                    "REQ-PROTO-05");

        check_finite_timed("frame.sfd_timeout", f.sfd_timeout);
        validate_timed_field("frame.sfd_timeout", f.sfd_timeout, /*required=*/false,
                             TimeReferenceEvent::RxEnable, /*require_marker=*/true);

        // Exactly one layer appends the FCS, and WHICH one is not free.
        // A config that names the MAC as the appender is DESCRIBABLE -- the
        // geometry authority has a variant for it, frame_geometry_check()
        // reports the mac_fcs_bytes that layout requires, and the on-air
        // length does not change -- but it is not ENCODABLE here:
        // FrameProfile::fcs_appended_by_modulation_layer is true, so
        // frame_profile_validate() rejects it, and
        // FrameProfileGeometry::executable() is false for it.  M0 accepted
        // such a configuration and then reported a "MAC PSDU" no encoder can
        // produce, which is the R3 class of defect one layer down.
        if (f.fcs_append == FcsAppender::None)
            cfg_rej("frame.fcs_append",
                    ConfigReason::FieldConflict,
                    "no layer appends the FCS; exactly one of mac/phy must");
        else if (f.fcs_append == FcsAppender::MacLayer && have_profile) {
            unsup("frame.fcs_append",
                  "fcs_append=mac asks for a MAC that builds and appends the FCS itself. The "
                  "geometry is DESCRIBABLE and the on-air length is unchanged, but this codec "
                  "only encodes the PHY-appends form: FrameProfileGeometry::executable() is false "
                  "for the MAC-appends variant and frame_profile_validate() rejects "
                  "fcs_appended_by_modulation_layer=false",
                  ConfigReason::Unsupported,
                  "REQ-API-01");
        }
        if (f.fcs_bytes == 0)
            cfg_rej("frame.fcs_bytes", ConfigReason::ZeroValue,
                "FCS length is required");
        else if (have_profile &&
                 f.fcs_bytes != static_cast<uint16_t>(authority.fcs_bytes_on_air()))
            unsup("frame.fcs_bytes",
                  "frame profile " + std::string(frame_profile_id_to_string(f.frame_profile)) +
                      " has a " + twr_int_to_text(authority.fcs_bytes_on_air()) +
                      "-byte FCS (IEEE 802.15.4), the config states " +
                      twr_int_to_text(f.fcs_bytes) + " bytes",
                  ConfigReason::Unsupported,
                  "REQ-API-01");
        const bool declared_includes = (f.fcs_append == FcsAppender::MacLayer);
        if (f.mac_psdu_includes_fcs != declared_includes)
            cfg_rej("frame.mac_psdu_includes_fcs",
                    ConfigReason::FieldConflict,
                    std::string("fcs_append=") + fcs_appender_to_string(f.fcs_append) +
                        " but mac_psdu_includes_fcs=" + twr_bool_to_text(f.mac_psdu_includes_fcs) +
                        "; exactly one layer appends the FCS",
                    "REQ-API-01");

        // ---- the geometry CLAIM, field by field, against the codec ---------
        if (!f.geometry.any_set()) {
            cfg_rej("frame.geometry",
                    ConfigReason::EmptyValue,
                    "the frame profile geometry must be stated explicitly; it is never assumed. "
                    "Fill it with frame_geometry_of_profile(frame.frame_profile) so the claim "
                    "comes from the codec rather than from a literal");
        } else if (have_profile) {
            // A claim that disagrees with the codec is a REJECTION PER FIELD,
            // not a silently-corrected value: an operator who believes the
            // header is 7 bytes must be told it is 14, not handed a working
            // config that says something else.
            //
            // The authority the claim is checked against follows the FCS owner
            // the config NAMES, not the one it should name: a MAC-appends
            // layout is described by frame_geometry_mac_appends_fcs(), which
            // expects the 2 FCS bytes inside the MAC PSDU.  That way a
            // MAC-appends claim that forgets to reserve them is caught as the
            // field disagreement it is, and the separate `fcs_append`
            // rejection above says the layout is not encodable here at all.
            const FrameProfileGeometry& named_authority =
                (f.fcs_append == FcsAppender::MacLayer)
                    ? frame_geometry_mac_appends_fcs(f.frame_profile)
                    : authority;
            const GeometryCheckResult geo = frame_geometry_check(
                frame_geometry_claim(f.geometry), named_authority, profile);
            for (size_t i = 0; i < geo.fields.count; ++i) {
                const GeometryMismatch& m = geo.fields.items[i];
                cfg_rej(std::string("frame.geometry.") + geometry_field_to_string(m.field),
                        ConfigReason::FieldConflict,
                        std::string("the frame codec is the geometry authority: profile ") +
                            frame_profile_id_to_string(f.frame_profile) + " with FCS appended by " +
                            fcs_owner_to_string(named_authority.fcs_owner) + " has " +
                            geometry_field_to_string(m.field) + " = " +
                            twr_int_to_text(static_cast<int64_t>(m.expected)) +
                            ", the config claims " +
                            twr_int_to_text(static_cast<int64_t>(m.actual)) +
                            ". A claim does not decide the layout: fill the geometry from "
                            "frame_geometry_of_profile() so there is one source",
                        "REQ-PROTO-06");
            }
        }

        // STS: out of scope for phase 1, explicitly.
        if (f.sts_mode != StsMode::Off)
            unsup("frame.sts_mode",
                  std::string("STS mode ") + sts_mode_to_string(f.sts_mode) + ": " +
                      d_caps.unsupported_sts_reason,
                  ConfigReason::OutOfScope,
                  "REQ-SCOPE-04");
        if (f.sts_length_symbols != 0)
            cfg_rej("frame.sts_length_symbols",
                    ConfigReason::FieldConflict,
                    "sts_mode is off but an STS length is requested");
    }

    void check_tx(const TwrConfig& c) const
    {
        const TransmitConfig& t = c.tx;
        if (static_cast<uint32_t>(t.port) >= kTwrMaxPhysicalChannels)
            cfg_rej("tx.port", ConfigReason::IndexOutOfRange,
                "physical TX channel index too large");
        check_finite_opt("tx.gain_db", t.gain_db);
        check_finite_opt("tx.iq_amplitude", t.iq_amplitude);
        check_finite_opt("tx.calibrated_tx_power_dbm", t.calibrated_tx_power_dbm);

        if (t.gain_db.has() && (t.gain_db.value() < 0.0 || t.gain_db.value() > 120.0))
            cfg_rej("tx.gain_db", ConfigReason::OutOfRange,
                "TX gain must be within 0..120 dB");
        if (t.iq_amplitude.has() && (t.iq_amplitude.value() <= 0.0 || t.iq_amplitude.value() > 1.0))
            cfg_rej("tx.iq_amplitude",
                    ConfigReason::OutOfRange,
                    "digital IQ amplitude must be within (0, 1] for a full-scale SC16 stream");
        if (t.calibrated_tx_power_dbm.has() &&
            (t.calibrated_tx_power_dbm.value() < -30.0 || t.calibrated_tx_power_dbm.value() > 30.0))
            cfg_rej("tx.calibrated_tx_power_dbm",
                    ConfigReason::OutOfRange,
                    "calibrated antenna-plane power must be within -30..30 dBm");

        // Exactly one power authority, and the authority must be the field that
        // is actually present.  The three quantities are never conflated.
        int present = 0;
        if (t.gain_db.has())
            present++;
        if (t.iq_amplitude.has())
            present++;
        if (t.calibrated_tx_power_dbm.has())
            present++;
        const bool authority_present =
            (t.power_policy == TxPowerPolicy::LeaveUntouched) ||
            (t.power_policy == TxPowerPolicy::ManualGainDb && t.gain_db.has()) ||
            (t.power_policy == TxPowerPolicy::IqAmplitude && t.iq_amplitude.has()) ||
            (t.power_policy == TxPowerPolicy::CalibratedDbm && t.calibrated_tx_power_dbm.has());
        if (!authority_present)
            cfg_rej("tx.power_policy",
                    ConfigReason::FieldConflict,
                    std::string("power_policy=") + tx_power_policy_to_string(t.power_policy) +
                        " but the field it names is not set (gain dB, IQ amplitude and calibrated "
                        "dBm are three separate fields)");
        if (t.power_policy == TxPowerPolicy::LeaveUntouched && present != 0)
            cfg_rej("tx.power_policy",
                    ConfigReason::FieldConflict,
                    "power_policy is leave_untouched but " + twr_int_to_text(present) +
                        " power field(s) are set");
        if (t.power_policy == TxPowerPolicy::CalibratedDbm && t.calibrated_tx_power_dbm.has() &&
            !t.gain_db.has())
            cfg_rej("tx.gain_db",
                    ConfigReason::FieldConflict,
                    "a calibrated dBm is only meaningful together with the gain it was "
                    "measured at");

        // A vendor power word is a fourth, separately named field and may only
        // exist with the backend capability that defines its bits.
        if (t.vendor_power_word.has()) {
            if (t.vendor_power_word_backend.empty())
                cfg_rej("tx.vendor_power_word",
                        ConfigReason::CalibrationMismatch,
                        "a vendor power word is meaningless without the backend capability id that "
                        "defines its bits");
        } else if (!t.vendor_power_word_backend.empty()) {
            cfg_rej("tx.vendor_power_word_backend",
                    ConfigReason::FieldConflict,
                    "a backend capability id is set but no vendor power word is");
        }
    }

    void check_rx(const TwrConfig& c) const
    {
        const ReceiveConfig& r = c.rx;
        if (static_cast<uint32_t>(r.port) >= kTwrMaxPhysicalChannels)
            cfg_rej("rx.port", ConfigReason::IndexOutOfRange,
                "physical RX channel index too large");
        check_finite_opt("rx.gain_db", r.gain_db);
        check_finite_opt("rx.bandwidth_hz", r.bandwidth_hz);
        finite("rx.detection_threshold", r.detection_threshold);
        finite("rx.correlation_threshold", r.correlation_threshold);
        finite("rx.first_path_threshold", r.first_path_threshold);
        check_finite_opt("rx.vendor_pac_applied_step", r.vendor_pac_applied_step);

        if (r.gain_db.has() && (r.gain_db.value() < 0.0 || r.gain_db.value() > 120.0))
            cfg_rej("rx.gain_db", ConfigReason::OutOfRange,
                "RX gain must be within 0..120 dB");
        if (r.agc == AgcMode::Manual && !r.gain_db.has())
            cfg_rej("rx.gain_db",
                    ConfigReason::FieldConflict,
                    "agc=manual needs an explicit RX gain in dB");
        if (r.bandwidth_hz.has() && r.bandwidth_hz.value() <= 0.0)
            cfg_rej("rx.bandwidth_hz",
                    ConfigReason::OutOfRange,
                    "RF bandwidth must be > 0; it is a different quantity from the sample rate "
                    "(REQ-PHY-02)");
        if (r.bandwidth_hz.has() && r.bandwidth_hz.value() > 1.0e9)
            cfg_rej("rx.bandwidth_hz", ConfigReason::OutOfRange,
                "RF bandwidth is implausible");
        if (r.detection_threshold < 0.0 || r.detection_threshold > 1.0)
            cfg_rej("rx.detection_threshold", ConfigReason::OutOfRange,
                "must be within 0..1");
        if (r.correlation_threshold < 0.0 || r.correlation_threshold > 1.0)
            cfg_rej("rx.correlation_threshold", ConfigReason::OutOfRange,
                "must be within 0..1");
        if (r.first_path_threshold < 0.0 || r.first_path_threshold > 1.0)
            cfg_rej("rx.first_path_threshold", ConfigReason::OutOfRange,
                "must be within 0..1");
        if (r.correlation_threshold < r.detection_threshold)
            cfg_rej("rx.correlation_threshold",
                    ConfigReason::FieldConflict,
                    "the fine correlation gate must not be looser than the coarse detection gate");
        if (r.first_path_window == 0)
            cfg_rej("rx.first_path_window",
                    ConfigReason::ZeroValue,
                    "the first-path search window must be at least one tap");
        else if (r.first_path_index >= r.first_path_window)
            cfg_rej("rx.first_path_index",
                    ConfigReason::IndexOutOfRange,
                    "first usable CIR index lies outside the search window");

        // A vendor PAC value must come from a backend capability query AND
        // carry the software step that is actually applied.  Mapping a PAC
        // value onto an arbitrary software detection step is forbidden.
        if (r.vendor_pac_value.has()) {
            if (r.vendor_pac_backend.empty())
                cfg_rej("rx.vendor_pac_value",
                        ConfigReason::CalibrationMismatch,
                        "a vendor PAC value is meaningless without the backend capability id that "
                        "defines it");
            if (!r.vendor_pac_applied_step.has())
                cfg_rej("rx.vendor_pac_applied_step",
                        ConfigReason::CalibrationMismatch,
                        "a vendor PAC value may not be mapped onto an arbitrary software "
                        "detection step; the backend must state the step it applies");
            else if (r.vendor_pac_applied_step.value() <= 0.0)
                cfg_rej("rx.vendor_pac_applied_step",
                        ConfigReason::OutOfRange,
                        "the applied software step must be > 0");
        } else {
            if (!r.vendor_pac_backend.empty())
                cfg_rej("rx.vendor_pac_backend",
                        ConfigReason::FieldConflict,
                        "a backend capability id is set but no vendor PAC value is");
            if (r.vendor_pac_applied_step.has())
                cfg_rej("rx.vendor_pac_applied_step",
                        ConfigReason::FieldConflict,
                        "a software step is declared without a vendor PAC value");
        }
    }

    void check_radio(const TwrConfig& c) const
    {
        const RadioConfig& r = c.radio;
        if (static_cast<uint32_t>(r.tx_channel) >= kTwrMaxPhysicalChannels)
            cfg_rej("radio.tx_channel", ConfigReason::IndexOutOfRange,
                "TX channel index too large");
        if (static_cast<uint32_t>(r.rx_channel) >= kTwrMaxPhysicalChannels)
            cfg_rej("radio.rx_channel", ConfigReason::IndexOutOfRange,
                "RX channel index too large");
        finite("radio.native_sample_rate_hz", r.native_sample_rate_hz, "REQ-PHY-02");
        if (std::isfinite(r.native_sample_rate_hz) && r.native_sample_rate_hz > 0.0 &&
            !d_caps.native_rate_supported(r.native_sample_rate_hz))
            unsup("radio.native_sample_rate_hz",
                  "native sample rate " + twr_double_to_text(r.native_sample_rate_hz) +
                      " is not supported; this build accepts 737280000 or 491520000",
                  ConfigReason::Unsupported,
                  "REQ-PHY-02");

        if (r.peers.size() > kTwrMaxPeersPerConfig)
            cfg_rej("radio.peers",
                    ConfigReason::OverCapacity,
                    "at most " + twr_int_to_text(static_cast<int64_t>(kTwrMaxPeersPerConfig)) +
                        " peer endpoints per config");

        // Resource conflicts on the same physical channel.
        for (size_t i = 0; i < r.peers.size(); ++i) {
            const EndpointBinding& p = r.peers[i];
            const std::string pf = "radio.peers[" + twr_int_to_text(static_cast<int64_t>(i)) + "]";
            if (p.id.empty())
                cfg_rej(pf + ".id", ConfigReason::EmptyValue,
                    "endpoint id is required");
            if (static_cast<uint32_t>(p.tx_channel) >= kTwrMaxPhysicalChannels)
                cfg_rej(pf + ".tx_channel", ConfigReason::IndexOutOfRange,
                    "TX channel too large");
            if (static_cast<uint32_t>(p.rx_channel) >= kTwrMaxPhysicalChannels)
                cfg_rej(pf + ".rx_channel", ConfigReason::IndexOutOfRange,
                    "RX channel too large");
            finite(pf + ".native_sample_rate_hz", p.native_sample_rate_hz);
            if (!p.occupies_resources)
                continue;
            if (p.tx_channel == r.tx_channel)
                cfg_rej(pf + ".tx_channel",
                        ConfigReason::DuplicateResource,
                        "physical TX channel " + twr_int_to_text(p.tx_channel) +
                            " is already claimed by this endpoint; one device resource, one owner",
                        "REQ-BASE-01");
            if (p.rx_channel == r.rx_channel)
                cfg_rej(pf + ".rx_channel",
                        ConfigReason::DuplicateResource,
                        "physical RX channel " + twr_int_to_text(p.rx_channel) +
                            " is already claimed by this endpoint; one device resource, one owner",
                        "REQ-BASE-01");
            if (p.role == c.session.role)
                cfg_rej(pf + ".role",
                        ConfigReason::FieldConflict,
                        std::string("both endpoints claim role ") + role_to_string(p.role) +
                            "; a two-endpoint session needs one initiator and one responder",
                        "REQ-PROTO-01");
            if (p.native_sample_rate_hz > 0.0 && r.native_sample_rate_hz > 0.0 &&
                !twr_rate_matches_strict(p.native_sample_rate_hz,
                    r.native_sample_rate_hz, 1e-9))
                cfg_rej(pf + ".native_sample_rate_hz",
                        ConfigReason::FieldConflict,
                        "endpoints sharing one device must share the native sample rate (" +
                            twr_double_to_text(r.native_sample_rate_hz) + " vs " +
                            twr_double_to_text(p.native_sample_rate_hz) + ")",
                        "REQ-PHY-02");
            for (size_t j = 0; j < i; ++j) {
                const EndpointBinding& q = r.peers[j];
                if (!q.occupies_resources)
                    continue;
                if (q.id == p.id)
                    cfg_rej(pf + ".id", ConfigReason::DuplicateResource,
                        "duplicate endpoint id");
                if (q.tx_channel == p.tx_channel)
                    cfg_rej(pf + ".tx_channel",
                            ConfigReason::DuplicateResource,
                            "physical TX channel " + twr_int_to_text(p.tx_channel) +
                                " claimed by two endpoints",
                            "REQ-BASE-01");
                if (q.rx_channel == p.rx_channel)
                    cfg_rej(pf + ".rx_channel",
                            ConfigReason::DuplicateResource,
                            "physical RX channel " + twr_int_to_text(p.rx_channel) +
                                " claimed by two endpoints",
                            "REQ-BASE-01");
                if (q.role == p.role)
                    cfg_rej(pf + ".role",
                            ConfigReason::FieldConflict,
                            std::string("two peers claim the same role ") + role_to_string(p.role),
                            "REQ-PROTO-01");
            }
        }

        // Readback: a mismatch is a STARTUP FAILURE, never a soft warning.
        if (r.require_readback && !r.readback.present)
            rej("radio.readback.present",
                ConfigReason::CalibrationMissing,
                ExchangeStatus::CalibrationMissing,
                "require_readback is set but no hardware readback was recorded",
                "REQ-PHY-02");
        if (r.readback.present) {
            finite("radio.readback.sample_rate_hz", r.readback.sample_rate_hz,
                "REQ-PHY-02");
            finite("radio.readback.center_freq_hz", r.readback.center_freq_hz,
                "REQ-PHY-01");
            if (!d_caps.native_rate_supported(r.readback.sample_rate_hz))
                cfg_rej("radio.readback.sample_rate_hz",
                        ConfigReason::OutOfRange,
                        "readback native rate " + twr_double_to_text(r.readback.sample_rate_hz) +
                            " Hz is not a supported rate");
            else if (r.native_sample_rate_hz > 0.0 &&
                     !twr_rate_matches_strict(r.native_sample_rate_hz,
                                              r.readback.sample_rate_hz,
                                              1e-9))
                cfg_rej("radio.readback.sample_rate_hz",
                        ConfigReason::FieldConflict,
                        "readback native rate " + twr_double_to_text(r.readback.sample_rate_hz) +
                            " Hz differs from the requested " +
                            twr_double_to_text(r.native_sample_rate_hz) + " Hz",
                        "REQ-PHY-02");
            if (r.readback.tx_channel != r.tx_channel)
                cfg_rej("radio.readback.tx_channel",
                        ConfigReason::FieldConflict,
                        "readback TX channel differs from the requested one",
                        "REQ-PHY-02");
            if (r.readback.rx_channel != r.rx_channel)
                cfg_rej("radio.readback.rx_channel",
                        ConfigReason::FieldConflict,
                        "readback RX channel differs from the requested one",
                        "REQ-PHY-02");
            if (c.phy.center_frequency_hz > 0.0 && r.readback.center_freq_hz > 0.0 &&
                std::fabs(c.phy.center_frequency_hz - r.readback.center_freq_hz) >
                    kTwrChannelFrequencyToleranceHz)
                cfg_rej("radio.readback.center_freq_hz",
                        ConfigReason::ChannelFrequencyMismatch,
                        "readback centre frequency " +
                        twr_double_to_text(r.readback.center_freq_hz) +
                            " Hz differs from the requested " +
                            twr_double_to_text(c.phy.center_frequency_hz) + " Hz",
                        "REQ-PHY-01");
            if (!r.clock_source.empty() && !r.readback.clock_source.empty() &&
                r.clock_source != r.readback.clock_source)
                cfg_rej("radio.readback.clock_source",
                        ConfigReason::FieldConflict,
                        "readback clock source differs from the requested one",
                        "REQ-PHY-02");
            if (!r.time_source.empty() && !r.readback.time_source.empty() &&
                r.time_source != r.readback.time_source)
                cfg_rej("radio.readback.time_source",
                        ConfigReason::FieldConflict,
                        "readback time source differs from the requested one",
                        "REQ-PHY-02");
        }
    }

    // The generic timed-field contract.  Returns nothing; appends violations.
    void validate_timed_field(const std::string& f,
                              const TimedField& t,
                              bool required,
                              TimeReferenceEvent expect_ref,
                              bool require_marker) const
    {
        if (t.domain == TimeDomain::Unspecified) {
            cfg_rej(f + ".domain",
                    ConfigReason::EmptyValue,
                    "the clock domain must be stated (device_ticks or monotonic_host)",
                    "REQ-API-02");
            return;
        }
        if (t.value.negative())
            cfg_rej(f + ".ns", ConfigReason::NegativeValue, "may not be negative",
                "REQ-API-02");
        if (required && t.value.is_zero())
            cfg_rej(f + ".ns", ConfigReason::ZeroValue, "must be > 0", "REQ-API-02");
        if (expect_ref != TimeReferenceEvent::HostMonotonic && t.reference != expect_ref)
            cfg_rej(f + ".reference",
                    ConfigReason::FieldConflict,
                    std::string("must be ") + time_reference_event_to_string(expect_ref) +
                        std::string(", is ") + time_reference_event_to_string(t.reference),
                    "REQ-API-02");
        if (t.domain == TimeDomain::DeviceTicks) {
            if (require_marker && !t.marker.has())
                cfg_rej(f + ".marker",
                        ConfigReason::EmptyValue,
                        "a device-tick field must name the RF marker it is measured at "
                        "(REQ-TIME-02)",
                        "REQ-API-02");
            if (t.marker.has() && t.marker.value() == TimestampMarker::UhdRxFirstIqSample &&
                expect_ref == TimeReferenceEvent::FrameTail)
                cfg_rej(f + ".marker",
                        ConfigReason::FieldConflict,
                        "the first RX IQ sample is not the end of a frame",
                        "REQ-TIME-02");
        } else { // MonotonicHost
            if (t.marker.has())
                cfg_rej(f + ".marker",
                        ConfigReason::FieldConflict,
                        "a host monotonic field has no RF marker; a marker here would invite a "
                        "cross-domain subtraction",
                        "REQ-TIME-01");
            if (t.required_quantisation_hz != 1.0e9)
                cfg_rej(f + ".quantisation_hz",
                        ConfigReason::QuantisationUnsupported,
                        "a host monotonic field must be quantised at 1e9 (integer nanoseconds)",
                        "REQ-TIME-01");
        }
        if (!std::isfinite(t.required_quantisation_hz) || t.required_quantisation_hz <= 0.0) {
            cfg_rej(f + ".quantisation_hz",
                    ConfigReason::QuantisationUnsupported,
                    "the required quantisation rate must be finite and > 0",
                    "REQ-API-01");
            return;
        }
        if (!d_caps.quantisation_supported(t.required_quantisation_hz)) {
            cfg_rej(f + ".quantisation_hz",
                    ConfigReason::QuantisationUnsupported,
                    "quantisation " + twr_double_to_text(t.required_quantisation_hz) +
                        " Hz is not a rate this build can express (allowed: 1e9 ns, " +
                        twr_double_to_text(d_caps.work_sample_rate_hz) + " work rate, the native "
                        "device rates)",
                    "REQ-API-01");
            return;
        }
        if (t.max_quantisation_error_ns < 0)
            cfg_rej(f + ".max_quantisation_error_ns",
                    ConfigReason::OutOfRange,
                    "the tolerated quantisation error may not be negative");
        // The two distinct precision gates.
        if (!t.value.representable_at(t.required_quantisation_hz,
            t.max_quantisation_error_ns))
            cfg_rej(f + ".ns",
                    ConfigReason::TimingPrecisionInsufficient,
                    "a device tick at " + twr_double_to_text(t.required_quantisation_hz) +
                        " Hz cannot express this value with <= " +
                        twr_int_to_text(t.max_quantisation_error_ns) + " ns of error",
                    "REQ-API-01");
        bool ok = false;
        (void)quantise_duration(t.value, t.required_quantisation_hz, ok);
        if (!ok)
            cfg_rej(f + ".ns",
                    ConfigReason::TimingPrecisionInsufficient,
                    "the value does not fit the device tick range at " +
                        twr_double_to_text(t.required_quantisation_hz) + " Hz",
                    "REQ-API-01");
    }

    void check_timing(const TwrConfig& c) const
    {
        const PerMessageTiming& t = c.timing;
        const bool ss = c.session.protocol == Protocol::Ss;
        const bool initiator = c.session.role == Role::Initiator;

        check_finite_timed("timing.poll_start", t.poll_start);
        check_finite_timed("timing.poll_to_response", t.poll_to_response);
        check_finite_timed("timing.response_to_final", t.response_to_final);
        check_finite_timed("timing.final_to_report", t.final_to_report);
        check_finite_timed("timing.post_tx_rx_enable", t.post_tx_rx_enable);
        check_finite_timed("timing.min_tx_lead_time", t.min_tx_lead_time);

        // poll_start: absolute, on the device clock, at the Poll TX RMARKER.
        // Only the initiator transmits a Poll, so a responder's poll_start must
        // be zero rather than silently reusing the initiator's value.
        validate_timed_field("timing.poll_start",
                             t.poll_start,
                             initiator,
                             TimeReferenceEvent::PollTransmitRmarker,
                             true);
        if (!initiator && !t.poll_start.value.is_zero())
            cfg_rej("timing.poll_start.ns",
                    ConfigReason::FieldConflict,
                    "the responder transmits no Poll, so the Poll start time must be zero",
                    "REQ-API-02");

        // The three distinct quantities.
        if (initiator) {
            // The initiator does not send a Response, so it owns no reply delay.
            if (!t.poll_to_response.value.is_zero())
                cfg_rej("timing.poll_to_response",
                        ConfigReason::FieldConflict,
                        "the initiator never sends a Response, so this reply delay must be zero",
                        "REQ-API-02");
            if (t.poll_to_response.reference != TimeReferenceEvent::PollReceiveRmarker)
                cfg_rej("timing.poll_to_response.reference",
                        ConfigReason::FieldConflict,
                        "must be poll_rx_rmarker",
                        "REQ-API-02");
            if (ss) {
                if (!t.response_to_final.value.is_zero())
                    cfg_rej("timing.response_to_final",
                            ConfigReason::FieldConflict,
                            "SS-TWR has no Final message, so this delay must be zero",
                            "REQ-API-02");
            } else {
                validate_timed_field("timing.response_to_final",
                                     t.response_to_final,
                                     true,
                                     TimeReferenceEvent::ResponseReceiveRmarker,
                                     true);
            }
        } else {
            if (!t.response_to_final.value.is_zero())
                cfg_rej("timing.response_to_final",
                        ConfigReason::FieldConflict,
                        "the responder never sends a Final, so this delay must be zero",
                        "REQ-API-02");
            if (t.response_to_final.reference != TimeReferenceEvent::FinalTransmitRmarker)
                cfg_rej("timing.response_to_final.reference",
                        ConfigReason::FieldConflict,
                        "must be final_tx_rmarker",
                        "REQ-API-02");
            validate_timed_field("timing.poll_to_response",
                                 t.poll_to_response,
                                 true,
                                 TimeReferenceEvent::PollReceiveRmarker,
                                 true);
        }

        // post-TX RX enable: a device-tick delay from the end of OUR OWN frame.
        validate_timed_field("timing.post_tx_rx_enable",
                             t.post_tx_rx_enable,
                             true,
                             TimeReferenceEvent::FrameTail,
                             true);
        validate_timed_field("timing.min_tx_lead_time",
                             t.min_tx_lead_time,
                             true,
                             TimeReferenceEvent::ResponseTransmitRmarker,
                             true);

        // Report is reserved in phase 1.
        if (!t.final_to_report.value.is_zero())
            unsup("timing.final_to_report",
                  "FrameType::Report is reserved in phase 1; a Report message would invalidate the "
                  "three-message interop claim (REQ-PROTO-05)",
                  ConfigReason::OutOfScope,
                  "REQ-PROTO-05");

        // REQ-GR-04 feasibility: the reply delay must cover the device lead
        // time.  There is no default lead time -- the operator must supply the
        // measured value, and a delay shorter than it is rejected instead of
        // being sent late.
        const TimedField& reply = initiator ? t.response_to_final : t.poll_to_response;
        if (reply.value.negative() || t.min_tx_lead_time.value.negative() || reply.value.is_zero())
            return; // the structural violation was already reported above
        if (reply.value > t.min_tx_lead_time.value)
            return; // the budget is feasible
        rej(initiator ? "timing.response_to_final.ns" : "timing.poll_to_response.ns",
            ConfigReason::TimingBudgetInfeasible,
            ExchangeStatus::DeadlineMissed,
            "reply delay " + twr_duration_to_text(reply.value) +
                " does not cover the measured minimum TX lead time " +
                twr_duration_to_text(t.min_tx_lead_time.value) +
                " (a scheduled TX whose deadline cannot be met must fail, not be sent late)",
            "REQ-GR-04");
    }

    void check_timeouts(const TwrConfig& c) const
    {
        const TimeoutConfig& t = c.timeouts;
        const bool ss = c.session.protocol == Protocol::Ss;
        const bool initiator = c.session.role == Role::Initiator;

        check_finite_timed("timeouts.poll_rx_window", t.poll_rx_window);
        check_finite_timed("timeouts.response_rx_window", t.response_rx_window);
        check_finite_timed("timeouts.final_rx_window", t.final_rx_window);
        check_finite_timed("timeouts.report_rx_window", t.report_rx_window);
        check_finite_timed("timeouts.rx_timeout", t.rx_timeout);
        check_finite_timed("timeouts.exchange_timeout", t.exchange_timeout);
        check_finite_timed("timeouts.retry_interval", t.retry_interval);

        // The RX timeout is measured FROM RX ENABLE, on the device clock.
        validate_timed_field("timeouts.rx_timeout", t.rx_timeout, true,
                             TimeReferenceEvent::RxEnable, true);

        // The whole-exchange deadline and the retry gap live on the host
        // monotonic clock and must carry NO RF marker (REQ-TIME-01).
        validate_timed_field("timeouts.exchange_timeout", t.exchange_timeout, true,
                             TimeReferenceEvent::HostMonotonic, false);
        validate_timed_field("timeouts.retry_interval", t.retry_interval, false,
                             TimeReferenceEvent::HostMonotonic, false);

        // Per-frame RX windows, armed at RX enable.  Only the frames this role
        // actually waits for may be non-zero.
        validate_timed_field("timeouts.poll_rx_window", t.poll_rx_window, !initiator,
                             TimeReferenceEvent::RxEnable, true);
        validate_timed_field("timeouts.response_rx_window", t.response_rx_window,
            initiator,
                             TimeReferenceEvent::RxEnable, true);
        if (initiator && !t.poll_rx_window.value.is_zero())
            cfg_rej("timeouts.poll_rx_window.ns",
                    ConfigReason::FieldConflict,
                    "the initiator never receives a Poll, so its Poll RX window must be zero",
                    "REQ-API-02");
        if (!initiator && !t.response_rx_window.value.is_zero())
            cfg_rej("timeouts.response_rx_window.ns",
                    ConfigReason::FieldConflict,
                    "the responder never receives a Response, so its Response RX window must be "
                    "zero",
                    "REQ-API-02");
        if (ss) {
            if (!t.final_rx_window.value.is_zero())
                cfg_rej("timeouts.final_rx_window.ns",
                        ConfigReason::FieldConflict,
                        "SS-TWR has no Final message, so the Final RX window must be zero",
                        "REQ-API-02");
        } else {
            validate_timed_field("timeouts.final_rx_window", t.final_rx_window,
                !initiator,
                                 TimeReferenceEvent::RxEnable, true);
        }
        if (!t.report_rx_window.value.is_zero())
            unsup("timeouts.report_rx_window",
                  "FrameType::Report is reserved in phase 1 (REQ-PROTO-05)",
                  ConfigReason::OutOfScope,
                  "REQ-PROTO-05");

        // Every active window must fit inside the RX timeout measured from the
        // same RX enable event, otherwise the timeout can fire before the
        // window closes.
        const struct {
            const char* path;
            const TimedField& f;
        } windows[] = {
            { "timeouts.poll_rx_window", t.poll_rx_window },
            { "timeouts.response_rx_window", t.response_rx_window },
            { "timeouts.final_rx_window", t.final_rx_window },
        };
        for (const auto& w : windows) {
            if (w.f.value.is_zero())
                continue;
            if (w.f.value > t.rx_timeout.value)
                cfg_rej(std::string(w.path) + ".ns",
                        ConfigReason::TimingOrderViolation,
                        "the RX window " + twr_duration_to_text(w.f.value) +
                            " is longer than the RX timeout " +
                            twr_duration_to_text(t.rx_timeout.value) +
                            " measured from the same RX enable",
                        "REQ-API-02");
        }

        // The whole-exchange deadline must cover the chain this endpoint runs.
        int64_t chain_ns = t.rx_timeout.value.nanos();
        if (!initiator)
            chain_ns += c.timing.poll_to_response.value.nanos();
        if (!ss && initiator)
            chain_ns += c.timing.response_to_final.value.nanos();
        if (chain_ns > 0 && t.exchange_timeout.value.nanos() < chain_ns)
            cfg_rej("timeouts.exchange_timeout.ns",
                    ConfigReason::TimingOrderViolation,
                    "the whole-exchange deadline " +
                        twr_duration_to_text(t.exchange_timeout.value) +
                        " is shorter than the chain this endpoint must run (" +
                        twr_duration_to_text(Duration::from_nanos(chain_ns)) + ")",
                    "REQ-API-02");

        // Consecutive measurements may not overlap: at most one exchange is in
        // flight, so the interval must be at least one whole exchange.
        if (c.session.measurement_count > 1 &&
            c.session.measurement_interval.nanos() < t.exchange_timeout.value.nanos())
            cfg_rej("session.measurement_interval.ns",
                    ConfigReason::TimingOrderViolation,
                    "at most one exchange is in flight, so the measurement interval must be at "
                    "least one whole-exchange timeout",
                    "REQ-API-01");
    }

    void check_calibration(const TwrConfig& c) const
    {
        const TimestampCalibrationConfig& k = c.calibration;
        if (k.tx_link_delay.negative() || k.rx_link_delay.negative() ||
            k.antenna_delay.negative() ||
            k.cable_delay.negative())
            cfg_rej("calibration.tx_link_delay",
                    ConfigReason::NegativeValue,
                    "a link/antenna/cable delay may not be negative",
                    "REQ-CAL-01");
        finite("calibration.native_sample_rate_hz", k.native_sample_rate_hz,
            "REQ-PHY-02");
        check_finite_opt("calibration.record.gain_db", k.record.gain_db);
        if (k.native_sample_rate_hz > 0.0 && !is_allowed_native_rate(k.native_sample_rate_hz))
            cfg_rej("calibration.native_sample_rate_hz",
                    ConfigReason::OutOfRange,
                    "calibration is versioned by sample rate; this build only calibrates "
                    "at 737280000 or 491520000",
                    "REQ-CAL-01");
        if (k.tx_link_delay_native_ticks < 0 || k.rx_link_delay_native_ticks < 0)
            cfg_rej("calibration.tx_link_delay_native_ticks",
                    ConfigReason::NegativeValue,
                    "a native-tick delay may not be negative",
                    "REQ-CAL-01");
        if (k.link_delay_unit == TimeUnit::Seconds)
            cfg_rej("calibration.link_delay_unit",
                    ConfigReason::UnitMismatch,
                    "the delay fields are integer nanoseconds; use link_delay_unit=ns or "
                    "native_ticks and state the seconds value elsewhere",
                    "REQ-TIME-01");
        if (k.link_delay_unit == TimeUnit::NativeTicks && k.native_sample_rate_hz <= 0.0)
            cfg_rej("calibration.native_sample_rate_hz",
                    ConfigReason::UnitMismatch,
                    "native_ticks needs the native sample rate it refers to",
                    "REQ-TIME-01");
        if (k.link_delay_unit == TimeUnit::NativeTicks &&
            (k.tx_link_delay_native_ticks == 0 && k.rx_link_delay_native_ticks == 0))
            cfg_rej("calibration.link_delay_unit",
                    ConfigReason::FieldConflict,
                    "link_delay_unit=native_ticks but no native-tick delay is stated",
                    "REQ-TIME-01");

        if (k.applied_count != 0)
            rej("calibration.applied_count",
                ConfigReason::CalibrationAlreadyApplied,
                ExchangeStatus::CalibrationMissing,
                "a calibration is applicable exactly once; this one has already been applied " +
                    twr_int_to_text(k.applied_count) + " time(s)",
                "REQ-CAL-01");

        if (!k.calibration_required)
            return;
        if (k.calibration_id.empty()) {
            rej("calibration.calibration_id",
                ConfigReason::CalibrationMissing,
                ExchangeStatus::CalibrationMissing,
                "a calibration id is required; an uncalibrated absolute range may not be claimed",
                "REQ-CAL-01");
            return;
        }
        if (k.record.calibration_id != k.calibration_id)
            cfg_rej("calibration.record.calibration_id",
                    ConfigReason::FieldConflict,
                    "the applicability record names a different calibration id",
                    "REQ-CAL-01");
        if (k.record.channel != c.phy.channel)
            cfg_rej("calibration.record.channel",
                    ConfigReason::CalibrationMismatch,
                    "calibration channel " + twr_int_to_text(k.record.channel) +
                        " does not match phy.channel " + twr_int_to_text(c.phy.channel),
                    "REQ-CAL-01");
        // M0.1: the applicability rule below is guarded by `> 0.0`, and a NaN
        // fails that comparison, so a NaN here used to sail through the whole
        // validator.  The record is compared against a rate, so it must be
        // finite whether or not the match ends up being attempted.
        // (record.gain_db is already covered by check_finite_opt above.)
        finite("calibration.record.native_sample_rate_hz",
               k.record.native_sample_rate_hz, "REQ-CAL-01");
        if (k.record.native_sample_rate_hz > 0.0 && c.radio.native_sample_rate_hz > 0.0 &&
            !twr_rate_matches_strict(k.record.native_sample_rate_hz,
                                     c.radio.native_sample_rate_hz,
                                     1e-9))
            cfg_rej("calibration.record.native_sample_rate_hz",
                    ConfigReason::CalibrationMismatch,
                    "calibration sample rate does not match the radio native rate",
                    "REQ-CAL-01");
        if (!k.record.profile_version.empty() && !c.meta.profile_version.empty() &&
            k.record.profile_version != c.meta.profile_version)
            cfg_rej("calibration.record.profile_version",
                    ConfigReason::CalibrationMismatch,
                    "calibration profile_version '" + k.record.profile_version +
                        "' does not match the session profile_version '" + c.meta.profile_version +
                        "'",
                    "REQ-CAL-01");
        if (!c.meta.calibration_version.empty() && k.calibration_id != c.meta.calibration_version)
            cfg_rej("meta.calibration_version",
                    ConfigReason::CalibrationMismatch,
                    "meta.calibration_version does not name the calibration that is loaded",
                    "REQ-OUT-01");
        if (c.tx.gain_db.has() && k.record.gain_db.has() &&
            k.record.gain_db.value() != c.tx.gain_db.value())
            cfg_rej("calibration.record.gain_db",
                    ConfigReason::CalibrationMismatch,
                    "the calibration was taken at a different TX gain",
                    "REQ-CAL-01");
    }

    void check_diagnostics(const TwrConfig& c) const
    {
        const DiagnosticsConfig& d = c.diagnostics;
        const struct {
            const char* on;
            const char* bytes;
            const char* stride;
            bool enabled;
            uint64_t bytes_v;
            uint32_t stride_v;
        } buffers[] = {
            { "diagnostics.cir_capture_enabled", "diagnostics.cir_capture_max_bytes",
              "diagnostics.cir_capture_stride", d.cir_capture_enabled, d.cir_capture_max_bytes,
              d.cir_capture_stride },
            { "diagnostics.short_iq_enabled", "diagnostics.short_iq_max_bytes",
              "diagnostics.short_iq_stride", d.short_iq_enabled, d.short_iq_max_bytes,
              d.short_iq_stride },
            { "diagnostics.raw_frame_dump", "diagnostics.raw_frame_max_bytes", nullptr,
              d.raw_frame_dump, d.raw_frame_max_bytes, 0u },
        };
        for (const auto& b : buffers) {
            if (b.bytes_v > kTwrMaxDiagnosticBytes)
                cfg_rej(b.bytes, ConfigReason::OverCapacity,
                        "diagnostic buffer above the " +
                            twr_int_to_text(static_cast<int64_t>(kTwrMaxDiagnosticBytes)) +
                            " byte bound (bounded memory, REQ-GR-03)");
            if (b.enabled && b.bytes_v == 0)
                cfg_rej(b.bytes, ConfigReason::ZeroValue,
                        "the diagnostic is enabled but no byte budget was stated");
            if (!b.enabled && b.bytes_v != 0)
                cfg_rej(b.bytes, ConfigReason::FieldConflict,
                        "a byte budget is stated but the diagnostic is disabled");
            if (b.stride != nullptr && b.enabled && b.stride_v == 0)
                cfg_rej(b.stride, ConfigReason::ZeroValue,
                        "a diagnostic stride of 0 would capture nothing");
            if (b.stride != nullptr && !b.enabled && b.stride_v != 0)
                cfg_rej(b.stride, ConfigReason::FieldConflict,
                        "a diagnostic stride is stated but the diagnostic is disabled");
        }
        if (d.result_queue_capacity == 0)
            cfg_rej("diagnostics.result_queue_capacity", ConfigReason::ZeroValue,
                    "the result queue must hold at least one entry");
        else if (d.result_queue_capacity > kTwrMaxQueueEntries)
            cfg_rej("diagnostics.result_queue_capacity", ConfigReason::OverCapacity,
                    "above the bounded queue limit");
        if (d.event_queue_capacity == 0)
            cfg_rej("diagnostics.event_queue_capacity", ConfigReason::ZeroValue,
                    "the event queue must hold at least one entry");
        else if (d.event_queue_capacity > kTwrMaxQueueEntries)
            cfg_rej("diagnostics.event_queue_capacity", ConfigReason::OverCapacity,
                    "above the bounded queue limit");
        if (!d.result_output_path.empty() && d.result_output_path.size() > 4096)
            cfg_rej("diagnostics.result_output_path", ConfigReason::OverCapacity,
                    "absurdly long output path");
        if (d.io_on_realtime_thread)
            cfg_rej("diagnostics.io_on_realtime_thread",
                    ConfigReason::FieldConflict,
                    "diagnostic I/O must not run on the realtime processing thread",
                    "REQ-GR-02");
        check_finite_timed("diagnostics.stats_cadence", d.stats_cadence);
        validate_timed_field("diagnostics.stats_cadence", d.stats_cadence, false,
                             TimeReferenceEvent::HostMonotonic, false);
    }

    // Frame lengths: Poll / Response / Final must each fit the negotiated PSDU
    // (REQ-API-01 帧格式; REQ-PHY-01 standard-length PSDU).
    void check_frame_lengths(const TwrConfig& c) const
    {
        const FrameFormatConfig& f = c.frame;
        if (f.mac_psdu_bytes == 0)
            return; // already reported as a missing value elsewhere
        const uint16_t cap = d_caps.max_psdu_bytes();
        if (cap != 0 && f.mac_psdu_bytes > cap)
            rej("frame.mac_psdu_bytes",
                ConfigReason::FrameLengthOverflow,
                ExchangeStatus::Unsupported,
                "negotiated PSDU " + twr_int_to_text(f.mac_psdu_bytes) +
                    " bytes exceeds the capability maximum " + twr_int_to_text(cap) + " bytes",
                "REQ-PHY-01");

        // Every length below comes from the codec, through
        // frame_psdu_bytes() -> FrameProfileGeometry::mac_payload_bytes() ->
        // frame_length_for().  The config's geometry is a claim, already
        // checked field by field in check_frame(), and is used here ONLY to
        // describe the arithmetic back to the operator.
        FrameProfileGeometry authority;
        FrameProfile profile;
        const bool have_profile =
            frame_profile_id_is_supported(f.frame_profile) &&
            frame_geometry_for(f.frame_profile, authority) &&
            frame_profile_for(f.frame_profile, profile);

        const FrameType types[] = { FrameType::Poll, FrameType::Response,
            FrameType::Final };
        for (FrameType t : types) {
            // `mac_psdu_bytes` is the negotiated MAC PSDU and, when the PHY
            // appends the FCS, EXCLUDES it (see the fcs_append /
            // mac_psdu_includes_fcs cross-check above).  So the comparison
            // below must be against the MAC bytes only.  The 127-byte IEEE
            // limit is separately checked against the on-air length.
            const uint32_t need = frame_psdu_bytes(t, f.fcs_append);
            const uint32_t on_air = have_profile
                                        ? static_cast<uint32_t>(
                                              authority.on_air_bytes(t, profile))
                                        : need + ((f.fcs_append == FcsAppender::PhyLayer)
                                                      ? f.fcs_bytes
                                                      : 0u);
            const std::string path =
                std::string("frame.mac_psdu_bytes[") + frame_type_to_string(t) + "]";
            if (need == 0)
                continue; // the profile itself was already reported as unknown
            if (need > static_cast<uint32_t>(f.mac_psdu_bytes))
                rej(path,
                    ConfigReason::FrameLengthOverflow,
                    ExchangeStatus::Unsupported,
                    std::string("the ") + frame_type_to_string(t) + " frame needs " +
                        twr_int_to_text(static_cast<int64_t>(need)) +
                        " MAC PSDU bytes (as frame profile " +
                        frame_profile_id_to_string(f.frame_profile) + " builds it: header " +
                        twr_int_to_text(authority.mac_header_bytes()) + " + " +
                        twr_int_to_text(frame_type_timestamp_count(t)) + "x" +
                        twr_int_to_text(authority.timestamp_bytes(profile)) +
                        " timestamps + footer " +
                        twr_int_to_text(authority.mac_footer_bytes()) +
                        (f.fcs_append == FcsAppender::MacLayer
                             ? std::string(" + FCS ") + twr_int_to_text(f.fcs_bytes)
                             : std::string("; the PHY appends the FCS separately")) +
                        ") but only " +
                        twr_int_to_text(f.mac_psdu_bytes) + " are negotiated",
                    "REQ-PHY-01");
            // The IEEE 127-byte limit applies to the ON-AIR PSDU, i.e. with
            // the FCS the PHY appends (UwbHrpPacketSource appends and then
            // checks 127), so this is the check that uses `on_air`.
            if (cap != 0 && on_air > static_cast<uint32_t>(cap))
                rej(path,
                    ConfigReason::FrameLengthOverflow,
                    ExchangeStatus::Unsupported,
                    std::string("the ") + frame_type_to_string(t) +
                        " frame needs " + twr_int_to_text(static_cast<int64_t>(on_air)) +
                        " on-air PSDU bytes (MAC " + twr_int_to_text(static_cast<int64_t>(need)) +
                        " + FCS " + twr_int_to_text(authority.fcs_bytes_on_air()) +
                        ") and does not fit the " + twr_int_to_text(cap) +
                        "-byte PSDU maximum",
                    "REQ-PHY-01");
        }
        // The application payload is a separate concept and must fit inside
        // the MAC PSDU after the header/timestamps/FCS are removed.
        if (f.geometry.any_set() && f.application_payload_bytes > f.mac_psdu_bytes)
            rej("frame.application_payload_bytes",
                ConfigReason::FrameLengthOverflow,
                ExchangeStatus::Unsupported,
                "the application payload (" + twr_int_to_text(f.application_payload_bytes) +
                    " bytes) does not fit the MAC PSDU (" +
                    twr_int_to_text(f.mac_psdu_bytes) + " bytes)",
                "REQ-API-01");

        // The joint PHY capability lookup, now that every axis is known.
        const CapabilityLookup look = d_caps.lookup_phy(c.radio.native_sample_rate_hz,
                                                       c.phy.tx_preamble_code,
                                                       c.phy.preamble_symbols,
                                                       f.sfd_mode,
                                                       f.mac_psdu_bytes,
                                                       f.ranging_bit);
        if (!look.allowed) {
            rej("phy.preamble_symbols",
                ConfigReason::Unsupported,
                ExchangeStatus::Unsupported,
                std::string("the (native_rate, code, sync, sfd, psdu, ranging) combination is not "
                            "in the capability whitelist [") +
                    capability_status_to_string(look.status) + "]: " + look.reason,
                "REQ-PHY-01");
        }
    }

    const Capabilities& d_caps;
    mutable ValidationReport d_report;
};

inline ValidationReport validate(const TwrConfig& cfg, const Capabilities& caps)
{
    ConfigValidator v(caps);
    return v.run(cfg);
}

inline ValidationReport validate(const TwrConfig& cfg) { return validate(cfg,
    capabilities()); }

} // namespace twr
} // namespace uwb
} // namespace gr

#endif /* INCLUDED_OPENUWB_TWR_CONFIG_VALIDATOR_H */
