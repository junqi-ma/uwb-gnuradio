/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Core vocabulary types for the UWB SS-TWR / DS-TWR protocol core (M0).
 *
 * This header is the frozen contract that every later TWR header, the
 * Python schema and the CLI depend on.  It is deliberately:
 *
 *   - header-only and dependency-free (no GNU Radio, no UHD), so the
 *     protocol core can be unit tested and fuzzed without a radio;
 *   - free of any time-unit conversion for vendor parts.  Per REQ-API-02
 *     only SI units or this named `Duration` cross the public boundary;
 *     DW UUS, device ticks and USRP ticks are converted inside an adapter
 *     and never appear here.
 *
 * Requirements traceability (docs/twr/需求_UWB_SS_DS_TWR.md):
 *   REQ-PROTO-01  per-endpoint state, only matching frames advance the FSM
 *   REQ-PROTO-05  SS result lands at the initiator, DS at the responder
 *   REQ-ERR-01    the error taxonomy below is the complete set
 *   REQ-LIFE-01   exactly one terminal result per exchange
 *   REQ-TIME-01   clock domains / epochs are explicit (see timestamp.h)
 *   REQ-TIME-03   a scheduled TX is not a hardware-measured TX
 */

#ifndef INCLUDED_GNURADIO_UWB_UWB_TWR_TYPES_H
#define INCLUDED_GNURADIO_UWB_UWB_TWR_TYPES_H

#include <cmath>
#include <cstdint>
#include <limits>
#include <string>

namespace gr {
namespace uwb {
namespace twr {

// ---------------------------------------------------------------------------
// Protocol / role
// ---------------------------------------------------------------------------

// Two-Way Rounding algorithm.  SS computes at the initiator using the
// responder's (t2B, t3B); DS computes at the responder using its own
// (t2B, t3B, t6B) plus the initiator's times carried in Final.
enum class Protocol : uint8_t {
    Ss = 0, // Single-Sided:   Poll + Response
    Ds = 1  // Double-Sided:  Poll + Response + Final (asymmetric DS-TWR)
};

inline const char* protocol_to_string(Protocol p)
{
    switch (p) {
    case Protocol::Ss:
        return "ss";
    case Protocol::Ds:
        return "ds";
    }
    return "invalid";
}

inline bool protocol_from_string(const std::string& s, Protocol& out)
{
    if (s == "ss" || s == "SS" || s == "single") {
        out = Protocol::Ss;
        return true;
    }
    if (s == "ds" || s == "DS" || s == "double") {
        out = Protocol::Ds;
        return true;
    }
    return false;
}

// Phase-1 endpoints are logical roles on one X410; the physical channel map
// is a separate config axis (endpoint_channels), never encoded here.
enum class Role : uint8_t {
    Initiator = 0, // sends Poll, receives Response (and Final for DS)
    Responder = 1 // receives Poll, sends Response (and Final for DS)
};

inline const char* role_to_string(Role r)
{
    switch (r) {
    case Role::Initiator:
        return "initiator";
    case Role::Responder:
        return "responder";
    }
    return "invalid";
}

inline bool role_from_string(const std::string& s, Role& out)
{
    if (s == "initiator") {
        out = Role::Initiator;
        return true;
    }
    if (s == "responder") {
        out = Role::Responder;
        return true;
    }
    return false;
}

// ---------------------------------------------------------------------------
// Frame types
// ---------------------------------------------------------------------------

// Wire function codes.  The numeric values are part of the versioned frame
// profile (see uwb_twr_frame.h) and must never be renumbered.
enum class FrameType : uint8_t {
    Poll = 0x00,     // no timestamps carried
    Response = 0x01, // carries (t2B, t3B)
    Final = 0x02,    // DS only; carries (t1A, t4A, t5A)
    // Reserved.  Phase 1 deliberately has no Result/Report message: the
    // off-the-shelf three-message reference firmware does not emit one and
    // adding a fourth message would invalidate the interop claim
    // (REQ-PROTO-05).  The code point is kept so a future profile can claim
    // it, but the codec rejects it as NotImplemented in phase 1.
    Report = 0x03
};

inline const char* frame_type_to_string(FrameType t)
{
    switch (t) {
    case FrameType::Poll:
        return "poll";
    case FrameType::Response:
        return "response";
    case FrameType::Final:
        return "final";
    case FrameType::Report:
        return "report";
    }
    return "invalid";
}

inline bool frame_type_from_string(const std::string& s, FrameType& out)
{
    if (s == "poll") {
        out = FrameType::Poll;
        return true;
    }
    if (s == "response") {
        out = FrameType::Response;
        return true;
    }
    if (s == "final") {
        out = FrameType::Final;
        return true;
    }
    if (s == "report") {
        out = FrameType::Report;
        return true;
    }
    return false;
}

// Number of 40-bit timestamps a given frame type carries.  Fixed by the
// frame profile; a decoder that sees a different count must reject the frame
// rather than reinterpret it.
inline unsigned frame_type_timestamp_count(FrameType t)
{
    switch (t) {
    case FrameType::Poll:
        return 0;
    case FrameType::Response:
        return 2; // t2B, t3B
    case FrameType::Final:
        return 3; // t1A, t4A, t5A
    case FrameType::Report:
        return 0; // reserved / not implemented
    }
    return 0;
}

// ---------------------------------------------------------------------------
// Timestamp markers
// ---------------------------------------------------------------------------

// REQ-TIME-02 forbids collapsing these into one number.  They are four
// physically different instants and each carries a different correction:
//   - UhdRxFirstIqSample needs resampler group delay, filter delay, window
//     crop and sample-rate conversion before it means anything on air.
//   - PreambleStart / SfdStart are waveform coordinates inside a frame.
//   - Rmarker* are the protocol markers exchanged between endpoints.
//   - AntennaPlane is the calibrated reference plane.
enum class TimestampMarker : uint8_t {
    UhdRxFirstIqSample = 0, // device-reported time of received sample 0
    PreambleStart = 1,      // first sample of the preamble
    SfdStart = 2,           // start of the SFD, i.e. end of the preamble
    PhrStart = 3,           // start of the PHR
    RmarkerTx = 4,          // protocol RMARKER for a transmit
    RmarkerRx = 5,          // protocol RMARKER for a receive
    AntennaPlane = 6        // calibrated antenna reference plane
};

inline const char* timestamp_marker_to_string(TimestampMarker m)
{
    switch (m) {
    case TimestampMarker::UhdRxFirstIqSample:
        return "uhd_rx_first_iq_sample";
    case TimestampMarker::PreambleStart:
        return "preamble_start";
    case TimestampMarker::SfdStart:
        return "sfd_start";
    case TimestampMarker::PhrStart:
        return "phr_start";
    case TimestampMarker::RmarkerTx:
        return "rmarker_tx";
    case TimestampMarker::RmarkerRx:
        return "rmarker_rx";
    case TimestampMarker::AntennaPlane:
        return "antenna_plane";
    }
    return "invalid";
}

// REQ-TIME-03: a UHD timed command that returns success is NOT a
// hardware-measured air-interface timestamp.  The provenance must survive
// into every result (REQ-OUT-01) so a scheduled time is never presented as a
// measured one.
enum class TimestampSource : uint8_t {
    Unknown = 0,
    HardwareMeasured = 1,     // read back from hardware
    ScheduledCalibrated = 2,  // deterministic schedule + calibration
    Estimated = 3,            // e.g. fractional first-path interpolation
    Reconstructed = 4         // derived from another timestamp, no new data
};

inline const char* timestamp_source_to_string(TimestampSource s)
{
    switch (s) {
    case TimestampSource::Unknown:
        return "unknown";
    case TimestampSource::HardwareMeasured:
        return "hardware_measured";
    case TimestampSource::ScheduledCalibrated:
        return "scheduled_calibrated";
    case TimestampSource::Estimated:
        return "estimated";
    case TimestampSource::Reconstructed:
        return "reconstructed";
    }
    return "invalid";
}

// True when the timestamp may be used to produce a validated range.
inline bool timestamp_source_is_measurement(TimestampSource s)
{
    return s == TimestampSource::HardwareMeasured ||
           s == TimestampSource::ScheduledCalibrated;
}

// ---------------------------------------------------------------------------
// Endpoint FSM state (REQ-PROTO-01)
// ---------------------------------------------------------------------------

// Each endpoint owns exactly one of these and advances it only on a frame
// that passed FCS AND matched address / sequence / frame type / current
// state.  Late, duplicate, stale-session and out-of-order frames are counted
// separately and never advance the machine.
enum class EndpointState : uint8_t {
    Idle = 0,
    PollSent = 1,        // initiator: Poll transmitted, waiting for Response
    ResponseReceived = 2,// initiator: got Response, SS result computable
    FinalSent = 3,       // initiator: Final transmitted (DS), awaiting nothing
    PollReceived = 4,    // responder: got Poll, Response owed
    FinalReceived = 5    // responder: got Final, DS result computable
};

inline const char* endpoint_state_to_string(EndpointState s)
{
    switch (s) {
    case EndpointState::Idle:
        return "idle";
    case EndpointState::PollSent:
        return "poll_sent";
    case EndpointState::ResponseReceived:
        return "response_received";
    case EndpointState::FinalSent:
        return "final_sent";
    case EndpointState::PollReceived:
        return "poll_received";
    case EndpointState::FinalReceived:
        return "final_received";
    }
    return "invalid";
}

// ---------------------------------------------------------------------------
// Terminal exchange status (REQ-ERR-01, REQ-LIFE-01)
// ---------------------------------------------------------------------------

// Every exchange terminates with exactly ONE of these.  There is no
// "recovered" path: a failed measurement is reported as a failure, never
// patched up by skipping a slot and continuing (the radar skip-and-carry-on
// behaviour is explicitly forbidden for TWR).
//
// Numeric values are part of the JSON/CSV schema (REQ-OUT-01); append only.
enum class ExchangeStatus : uint8_t {
    Ok = 0,

    // --- configuration / lifecycle -----------------------------------------
    ConfigRejected = 1,     // validate() refused before the radio started
    Unsupported = 2,        // known-but-not-enabled capability (e.g. STS)
    Cancelled = 3,          // caller cancelled / stop() during the exchange
    QueueFull = 4,          // bounded event or result queue saturated
    InternalError = 5,      // unexpected internal failure

    // --- PHY / frame --------------------------------------------------------
    PhyFcsFailed = 10,      // decoded cleanly but FCS mismatch
    PhyDecodeFailed = 11,   // preamble/SFD/PHR/payload stage failed
    WrongPeer = 12,         // address, PAN or session did not match
    UnexpectedFrameType = 13,// well-formed frame illegal in current state
    StaleSession = 14,      // session id / sequence from an expired exchange

    // --- receive ------------------------------------------------------------
    RxTimeout = 20,         // expected frame did not arrive in its window
    RxOverflow = 21,        // device reported an RX overflow
    RxChainBroken = 22,     // SOB/EOB/time-spec fragment chain corrupted

    // --- transmit -----------------------------------------------------------
    TxLate = 30,            // delayed TX deadline could not be met
    TxUnderflow = 31,
    TxSeqError = 32,
    TxChainBroken = 33,

    // --- timing / signal quality -------------------------------------------
    InvalidTimeDomain = 40, // cross-domain subtraction, bad epoch, bad wrap
    ClockEstimateInvalid = 41, // clock ratio unavailable or out of validity
    FirstPathUnreliable = 42,  // first-path estimator failed its quality gate

    // --- configuration state at runtime ------------------------------------
    CalibrationMissing = 50,  // no calibration for device/channel/profile
    CalibrationExpired = 51,

    // --- protocol -----------------------------------------------------------
    ProtocolTimeout = 60,  // whole-exchange deadline expired
    DeadlineMissed = 61    // a per-message reply deadline was not schedulable
};

inline const char* exchange_status_to_string(ExchangeStatus s)
{
    switch (s) {
    case ExchangeStatus::Ok:
        return "ok";
    case ExchangeStatus::ConfigRejected:
        return "config_rejected";
    case ExchangeStatus::Unsupported:
        return "unsupported";
    case ExchangeStatus::Cancelled:
        return "cancelled";
    case ExchangeStatus::QueueFull:
        return "queue_full";
    case ExchangeStatus::InternalError:
        return "internal_error";
    case ExchangeStatus::PhyFcsFailed:
        return "phy_fcs_failed";
    case ExchangeStatus::PhyDecodeFailed:
        return "phy_decode_failed";
    case ExchangeStatus::WrongPeer:
        return "wrong_peer";
    case ExchangeStatus::UnexpectedFrameType:
        return "unexpected_frame_type";
    case ExchangeStatus::StaleSession:
        return "stale_session";
    case ExchangeStatus::RxTimeout:
        return "rx_timeout";
    case ExchangeStatus::RxOverflow:
        return "rx_overflow";
    case ExchangeStatus::RxChainBroken:
        return "rx_chain_broken";
    case ExchangeStatus::TxLate:
        return "tx_late";
    case ExchangeStatus::TxUnderflow:
        return "tx_underflow";
    case ExchangeStatus::TxSeqError:
        return "tx_seq_error";
    case ExchangeStatus::TxChainBroken:
        return "tx_chain_broken";
    case ExchangeStatus::InvalidTimeDomain:
        return "invalid_time_domain";
    case ExchangeStatus::ClockEstimateInvalid:
        return "clock_estimate_invalid";
    case ExchangeStatus::FirstPathUnreliable:
        return "first_path_unreliable";
    case ExchangeStatus::CalibrationMissing:
        return "calibration_missing";
    case ExchangeStatus::CalibrationExpired:
        return "calibration_expired";
    case ExchangeStatus::ProtocolTimeout:
        return "protocol_timeout";
    case ExchangeStatus::DeadlineMissed:
        return "deadline_missed";
    }
    return "invalid";
}

inline bool exchange_status_from_string(const std::string& s, ExchangeStatus& out)
{
    // Reverse of exchange_status_to_string; used by config/CLI round trips.
    for (int i = 0; i <= static_cast<int>(ExchangeStatus::DeadlineMissed); ++i) {
        ExchangeStatus c = static_cast<ExchangeStatus>(i);
        if (s == exchange_status_to_string(c)) {
            out = c;
            return true;
        }
    }
    return false;
}

inline bool exchange_status_is_ok(ExchangeStatus s) { return s == ExchangeStatus::Ok; }

// A validated range is only defined for a fully successful exchange.  First
// path and clock problems are reported as failures, never as a distance
// number with a bad quality flag attached (REQ-TIME-05, REQ-ERR-01).
inline bool exchange_status_yields_range(ExchangeStatus s)
{
    return s == ExchangeStatus::Ok;
}

// Coarse grouping for statistics and soak reports.
inline const char* exchange_status_family(ExchangeStatus s)
{
    switch (s) {
    case ExchangeStatus::Ok:
        return "ok";
    case ExchangeStatus::ConfigRejected:
    case ExchangeStatus::Unsupported:
        return "config";
    case ExchangeStatus::Cancelled:
        return "lifecycle";
    case ExchangeStatus::QueueFull:
    case ExchangeStatus::InternalError:
        return "host";
    case ExchangeStatus::PhyFcsFailed:
    case ExchangeStatus::PhyDecodeFailed:
        return "phy";
    case ExchangeStatus::WrongPeer:
    case ExchangeStatus::UnexpectedFrameType:
    case ExchangeStatus::StaleSession:
        return "protocol";
    case ExchangeStatus::RxTimeout:
    case ExchangeStatus::RxOverflow:
    case ExchangeStatus::RxChainBroken:
        return "rx";
    case ExchangeStatus::TxLate:
    case ExchangeStatus::TxUnderflow:
    case ExchangeStatus::TxSeqError:
    case ExchangeStatus::TxChainBroken:
        return "tx";
    case ExchangeStatus::InvalidTimeDomain:
    case ExchangeStatus::ClockEstimateInvalid:
    case ExchangeStatus::FirstPathUnreliable:
        return "signal";
    case ExchangeStatus::CalibrationMissing:
    case ExchangeStatus::CalibrationExpired:
        return "calibration";
    case ExchangeStatus::ProtocolTimeout:
    case ExchangeStatus::DeadlineMissed:
        return "deadline";
    }
    return "invalid";
}

// ---------------------------------------------------------------------------
// Duration — the only time type that crosses the public API
// ---------------------------------------------------------------------------

// REQ-API-02: the API speaks SI seconds or this named type.  Vendor units
// (DW UUS at 1/(499.2e6*128) s, USRP device ticks, host monotonic clock) are
// adapter-internal.  Integer nanoseconds keep round trips exact and keep
// JSON output free of binary floating point drift; the underlying tick
// quantisation is a separate, explicit conversion performed by
// `ticks_at()` / `from_ticks()` with a checked tick rate.
//
// Construction from a floating point value REJECTS non-finite input and
// values outside the representable range instead of silently saturating.
class Duration
{
public:
    constexpr Duration() = default;
    explicit constexpr Duration(int64_t ns) : d_ns(ns) {}

    static constexpr Duration from_nanos(int64_t ns) { return Duration(ns); }

    // Returns false and leaves `out` untouched for NaN / +-Inf / overflow.
    static bool from_seconds(double seconds, Duration& out)
    {
        if (!std::isfinite(seconds))
            return false;
        const double ns = seconds * 1e9;
        if (ns > 9.2233720368547758e18 || ns < -9.2233720368547758e18)
            return false;
        out = Duration(static_cast<int64_t>(ns));
        return true;
    }

    // Vendor-tick conversion.  `tick_rate_hz` must be finite and > 0.
    // Rounding is half-away-from-zero, matching the repo's llround usage.
    static bool from_ticks(int64_t ticks, double tick_rate_hz, Duration& out)
    {
        if (!std::isfinite(tick_rate_hz) || tick_rate_hz <= 0.0)
            return false;
        if (!from_seconds(static_cast<double>(ticks) / tick_rate_hz, out))
            return false;
        return true;
    }

    constexpr int64_t nanos() const { return d_ns; }
    constexpr double seconds() const { return static_cast<double>(d_ns) * 1e-9; }
    constexpr bool is_zero() const { return d_ns == 0; }
    constexpr bool negative() const { return d_ns < 0; }

    // Integer device ticks at `tick_rate_hz`, with an explicit overflow /
    // precision signal.  `ok == false` means the requested tick resolution is
    // finer than one nanosecond, i.e. the value would silently lose
    // information (REQ-API-01 rejects insufficient timing precision).
    int64_t ticks_at(double tick_rate_hz, bool& ok) const
    {
        ok = true;
        if (!std::isfinite(tick_rate_hz) || tick_rate_hz <= 0.0) {
            ok = false;
            return 0;
        }
        const double t = seconds() * tick_rate_hz;
        if (t > 9.2233720368547758e18 || t < -9.2233720368547758e18) {
            ok = false;
            return 0;
        }
        return static_cast<int64_t>(t < 0.0 ? t - 0.5 : t + 0.5);
    }

    // True when this duration survives a round trip through `tick_rate_hz`
    // ticks with at most `max_error_ns` of quantisation error.
    bool representable_at(double tick_rate_hz, int64_t max_error_ns) const
    {
        if (!std::isfinite(tick_rate_hz) || tick_rate_hz <= 0.0)
            return false;
        const double period_ns = 1e9 / tick_rate_hz;
        return period_ns <= static_cast<double>(max_error_ns) * 2.0;
    }

    constexpr Duration operator+(const Duration& o) const
    {
        return Duration(d_ns + o.d_ns);
    }
    constexpr Duration operator-(const Duration& o) const
    {
        return Duration(d_ns - o.d_ns);
    }
    constexpr bool operator<(const Duration& o) const { return d_ns < o.d_ns; }
    constexpr bool operator>(const Duration& o) const { return d_ns > o.d_ns; }
    constexpr bool operator<=(const Duration& o) const { return d_ns <= o.d_ns; }
    constexpr bool operator>=(const Duration& o) const { return d_ns >= o.d_ns; }
    constexpr bool operator==(const Duration& o) const { return d_ns == o.d_ns; }
    constexpr bool operator!=(const Duration& o) const { return d_ns != o.d_ns; }

private:
    int64_t d_ns = 0;
};

inline constexpr int64_t kMaxDurationNanos = std::numeric_limits<int64_t>::max();

// ---------------------------------------------------------------------------
// Small shared constants
// ---------------------------------------------------------------------------

// Nominal HRP mean PRF family used by the existing PHY geometry.  62.4 MHz is
// the measured BPRF value of the current code base (uwb_phy_profile.h); the
// 64 MHz figure is the standard HRP nominal and is kept for documentation so
// nobody silently conflates the two (REQ-PHY-02).
inline constexpr double kTwrNominalMeanPrfHz = 64.0e6;
inline constexpr double kTwrExistingMeanPrfHz = 62.4e6;

// Work (symbol) sample rate of the existing demodulation chain.
inline constexpr double kTwrWorkSampleRateHz = 998.4e6;
inline constexpr int64_t kTwrWorkSamplesPerSymbol = 1016;

} // namespace twr
} // namespace uwb
} // namespace gr

#endif /* INCLUDED_GNURADIO_UWB_UWB_TWR_TYPES_H */
