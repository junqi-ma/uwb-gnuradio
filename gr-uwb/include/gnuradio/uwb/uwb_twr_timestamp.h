/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Clock domains, epochs, markers and integer/fractional device timestamps for
 * the UWB SS-TWR / DS-TWR protocol core (M0).
 *
 * This header is the *type system* that makes REQ-TIME-01..05 unrepresentable
 * when violated.  It is header-only and dependency-free (no GNU Radio, no
 * UHD) so the timing core can be unit tested without a radio.  It contains
 * NO time-mapping arithmetic: resampler group delay, window-crop offset,
 * sample-rate conversion, first-path interpolation and the RMARKER <-> SFD
 * geometry are M2's job.  What this header does is *record* whether those
 * corrections were applied, and refuse the operations that would otherwise
 * quietly assume them.
 *
 * Requirements traceability (docs/twr/需求_UWB_SS_DS_TWR.md):
 *   REQ-TIME-01  integer device ticks + fractional sample; clock_domain,
 *                tick_rate, epoch_id, valid bits, marker, calibration id and
 *                source are all recorded.  Absolute timestamps from different
 *                clock domains CANNOT be subtracted -- `timestamp_interval()`
 *                rejects them with TimeIntervalStatus::DomainNameMismatch /
 *                TickRateMismatch / WrapWidthMismatch.  Only same-domain
 *                intervals are exchanged and converted.  A device reboot or
 *                time reset allocates a new epoch_id and every old-epoch
 *                timestamp is rejected with EpochMismatch.
 *   REQ-TIME-02  RX first IQ sample / SFD / RMARKER / antenna-plane instants
 *                are distinct TimestampMarker values.  An uncorrected
 *                capture-side timestamp cannot enter an interval
 *                (CorrectionsNotApplied) because the mapping bits are absent.
 *   REQ-TIME-03  a UHD timed-TX command that returned success is
 *                TimestampSource::ScheduledCalibrated and therefore is NOT a
 *                hardware-measured air time; the source survives into the JSON.
 *   REQ-TIME-04  the wrap width is a property of the *domain* (DW1000 40-bit,
 *                truncated 32-bit, USRP 64-bit ...).  There is deliberately no
 *                project-wide ticks-to-seconds multiplier anywhere in this
 *                header: every conversion goes through
 *                domain.tick_rate_hz + domain.timestamp_bits.
 *   REQ-TIME-05  a first-path estimate must carry the quality gate and the
 *                fractional-position bits before it can be used.
 *   REQ-CAL-01   a calibration is applied at most once per timestamp; the id
 *                is carried so a second application is detectable and refused
 *                (CalibrationResult::AlreadyApplied / ConflictingCalibration).
 *   REQ-OUT-01   JSON output keeps integer precision: ticks outside the
 *                2^53-1 safe range are emitted (and parsed) as decimal
 *                strings, never as a lossy double.
 *   REQ-ERR-01   every rejected operation carries a distinguishable status;
 *                all of them map to ExchangeStatus::InvalidTimeDomain.
 *
 * Intentionally NOT here (M2 owns these):
 *   - the actual resampler / filter / window-crop group-delay correction;
 *   - the RMARKER-to-SFD sample offset table (that is the PHY profile);
 *   - fractional first-path estimation and its SNR/sign gate arithmetic;
 *   - the SS clock-ratio (SFO) estimate and its validity window.
 * This header only models and validates the *contract* those stages fill in.
 */

#ifndef INCLUDED_GNURADIO_UWB_UWB_TWR_TIMESTAMP_H
#define INCLUDED_GNURADIO_UWB_UWB_TWR_TIMESTAMP_H

#include <gnuradio/uwb/uwb_twr_types.h>

#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <string>

namespace gr {
namespace uwb {
namespace twr {

// ===========================================================================
// Integer helpers
// ===========================================================================

namespace detail {

// Largest sub-tick fraction denominator accepted anywhere in this header.
// Bounded so that the sum or difference of two arbitrary fractions
// (den_a * den_b <= 32767^2 = 1.07e9, |num| < 2.15e9) is computed exactly in
// int64 and then, after gcd reduction, always fits the stored int32/uint32
// fields.  1/32767 of a tick is ~0.5 fs at the DW UUS rate, i.e. far finer
// than any first-path position this project can resolve.
inline constexpr uint32_t kFractionDenominatorMax = 32767u;

inline int64_t ts_gcd(int64_t a, int64_t b)
{
    a = a < 0 ? -a : a;
    b = b < 0 ? -b : b;
    while (b != 0) {
        const int64_t t = a % b;
        a = b;
        b = t;
    }
    return a;
}

inline bool ts_add_i64(int64_t a, int64_t b, int64_t& out)
{
    if (b > 0 && a > std::numeric_limits<int64_t>::max() - b)
        return false;
    if (b < 0 && a < std::numeric_limits<int64_t>::min() - b)
        return false;
    out = a + b;
    return true;
}

inline bool ts_sub_i64(int64_t a, int64_t b, int64_t& out)
{
    if (b < 0 && a > std::numeric_limits<int64_t>::max() + b)
        return false;
    if (b > 0 && a < std::numeric_limits<int64_t>::min() + b)
        return false;
    out = a - b;
    return true;
}

// ticks + num/den, floor-normalized and gcd-reduced.  `num` and `den` are
// int64 on entry so a product of two supported denominators never overflows;
// the result is refused if the reduced denominator is still above the
// supported cap (which would mean the value cannot be stored without loss).
inline bool ts_normalize_rational(int64_t ticks,
                                  int64_t num,
                                  int64_t den,
                                  int64_t& out_ticks,
                                  int32_t& out_num,
                                  uint32_t& out_den)
{
    if (den == 0) {
        if (num != 0)
            return false; // an exact tick cannot carry a fraction
        out_ticks = ticks;
        out_num = 0;
        out_den = 0u;
        return true;
    }
    if (den < 0) {
        den = -den;
        num = -num;
    }
    if (num == 0) {
        out_ticks = ticks;
        out_num = 0;
        out_den = 0u;
        return true;
    }
    if (den > static_cast<int64_t>(kFractionDenominatorMax))
        return false;

    int64_t carry = num / den;
    if ((num % den) != 0 && num < 0)
        carry -= 1; // floor semantics so the remainder lands in [0, den)
    num -= carry * den;

    int64_t nt = 0;
    if (!ts_add_i64(ticks, carry, nt))
        return false;

    const int64_t g = ts_gcd(num, den);
    num /= g;
    den /= g;

    out_ticks = nt;
    if (num == 0) {
        out_num = 0;
        out_den = 0u;
        return true;
    }
    out_num = static_cast<int32_t>(num);   // 0 < num < den <= 32767
    out_den = static_cast<uint32_t>(den);
    return true;
}

} // namespace detail

// Public alias of the sub-tick fraction denominator cap.
inline constexpr uint32_t kMaxTimestampFractionDenominator =
    detail::kFractionDenominatorMax;

// Largest JSON integer that survives a double round trip (2^53 - 1).  Ticks
// outside this window must be emitted as decimal strings (REQ-OUT-01).
inline constexpr int64_t kJsonSafeIntegerMax = 9007199254740991LL;

// ===========================================================================
// Forward declarations (the free-function API)
// ===========================================================================

struct ClockDomain;
struct Timestamp;

inline bool clock_domain_is_valid(const ClockDomain& d);
inline uint64_t clock_domain_wrap_period(const ClockDomain& d);
inline bool clock_domain_ticks_in_range(const ClockDomain& d, int64_t ticks);
inline bool clock_domain_tick_is_ns_or_coarser(const ClockDomain& d);
inline std::string clock_domain_to_string(const ClockDomain& d);
inline std::string clock_domain_to_json_string(const ClockDomain& d);

inline bool timestamp_normalize(int64_t ticks,
                                int32_t frac_num,
                                uint32_t frac_den,
                                int64_t& out_ticks,
                                int32_t& out_num,
                                uint32_t& out_den);
inline bool timestamp_marker_is_known(TimestampMarker m);
inline bool timestamp_source_is_known(TimestampSource s);
inline bool timestamp_make(int64_t ticks,
                           int32_t frac_num,
                           uint32_t frac_den,
                           const ClockDomain& domain,
                           TimestampMarker marker,
                           TimestampSource source,
                           uint32_t corrections,
                           Timestamp& out);
inline bool timestamp_is_self_consistent(const Timestamp& ts);
inline bool timestamp_fraction_is_normalized(const Timestamp& ts);
inline bool timestamp_is_marker_mapped(const Timestamp& ts);
inline bool timestamp_is_range_capable(const Timestamp& ts);
inline bool timestamp_is_validated_measurement(const Timestamp& ts);
inline const char* timestamp_rejection_reason(const Timestamp& ts);
inline std::string timestamp_to_json_string(const Timestamp& ts);
inline std::string timestamp_to_decimal_string(const Timestamp& ts);

// ===========================================================================
// Marker classification
// ===========================================================================

// REQ-TIME-02.  The four families are physically different reference planes;
// an interval is only meaningful inside the family pair the protocol actually
// uses.  A raw host capture coordinate is NEVER interchangeable with an
// on-air RMARKER: the two are connected by the M2 RX time-mapping
// correction, and taking an interval across that boundary would fabricate a
// propagation time out of a bookkeeping offset.
enum class TimestampMarkerClass : uint8_t {
    HostCapture = 0,   // UhdRxFirstIqSample: host / sample-clock coordinate
    WaveformRx = 1,    // received-waveform coordinates (preamble..RMARKER)
    WaveformTx = 2,    // transmitted RMARKER / command-time mapping
    ReferencePlane = 3 // calibrated antenna reference plane
};

inline TimestampMarkerClass timestamp_marker_class(TimestampMarker m)
{
    switch (m) {
    case TimestampMarker::UhdRxFirstIqSample:
        return TimestampMarkerClass::HostCapture;
    case TimestampMarker::PreambleStart:
    case TimestampMarker::SfdStart:
    case TimestampMarker::PhrStart:
    case TimestampMarker::RmarkerRx:
        return TimestampMarkerClass::WaveformRx;
    case TimestampMarker::RmarkerTx:
        return TimestampMarkerClass::WaveformTx;
    case TimestampMarker::AntennaPlane:
        return TimestampMarkerClass::ReferencePlane;
    }
    return TimestampMarkerClass::HostCapture;
}

inline const char* timestamp_marker_class_to_string(TimestampMarkerClass c)
{
    switch (c) {
    case TimestampMarkerClass::HostCapture:
        return "host_capture";
    case TimestampMarkerClass::WaveformRx:
        return "waveform_rx";
    case TimestampMarkerClass::WaveformTx:
        return "waveform_tx";
    case TimestampMarkerClass::ReferencePlane:
        return "reference_plane";
    }
    return "invalid";
}

// Which marker pairs may legally be subtracted to form a protocol interval.
//
// Allowed:
//   * identical marker on both sides  (two captures of the same kind, or a
//     corrected/uncorrected pair used to measure a link delay);
//   * {RmarkerRx, RmarkerTx} in either order.  This is exactly the TWR set:
//     RA = t4A - t1A, DB = t3B - t2B, DA = t5A - t4A, RB = t6B - t3B.
//
// Rejected on purpose:
//   * {UhdRxFirstIqSample, RmarkerRx} and friends -- the difference is a
//     pipeline bookkeeping offset, not a propagation time;
//   * {PreambleStart, RmarkerRx}, {SfdStart, RmarkerTx}, ... -- those are
//     fixed waveform-geometry constants that belong to the PHY profile, not
//     to two independent timestamps.  Taking them from two timestamps would
//     silently import a profile error into the ToF;
//   * anything involving AntennaPlane paired with a non-plane marker: the
//     antenna delay is applied by `apply_calibration*()`, which produces the
//     plane timestamp; it is not something to subtract after the fact.
inline bool timestamp_marker_interval_allowed(TimestampMarker later_marker,
                                             TimestampMarker earlier_marker)
{
    if (later_marker == earlier_marker)
        return true;
    return (later_marker == TimestampMarker::RmarkerRx &&
            earlier_marker == TimestampMarker::RmarkerTx) ||
           (later_marker == TimestampMarker::RmarkerTx &&
            earlier_marker == TimestampMarker::RmarkerRx);
}

// ===========================================================================
// Applied-correction bitmask
// ===========================================================================

// Each bit records that one specific M1/M2 stage actually ran.  A timestamp
// whose marker requires a bit that is not set cannot be used in a protocol
// interval -- this is how "the correction has not been applied yet" is
// *expressed* rather than assumed.
enum TimestampCorrection : uint32_t {
    kCorrectionNone = 0u,
    kCorrectionRxSampleToFirstPath = 1u << 0,   // resampler + filter group delay
    kCorrectionWindowCrop = 1u << 1,            // ROI start / window crop offset
    kCorrectionSampleRateConversion = 1u << 2,  // native <-> work sample grid
    kCorrectionFirstPathFraction = 1u << 3,     // sub-tick first-path estimate
    kCorrectionWaveformGeometry = 1u << 4,      // sample -> preamble/SFD/PHR coord
    kCorrectionRmarkerOffset = 1u << 5,         // waveform coord -> protocol marker
    kCorrectionTxCommandToAir = 1u << 6,        // TX command time -> first air sample
    kCorrectionDelayedTxQuantization = 1u << 7, // delayed-TX deadline rounding
    kCorrectionAntennaPlane = 1u << 8,          // link/antenna/cable -> plane
    kCorrectionFirstPathQualityGate = 1u << 9,  // REQ-TIME-05 sign / SNR gate
    kCorrectionAll = (1u << 10) - 1u
};

// Corrections that MUST be present for a timestamp to be treated as its own
// marker.  UhdRxFirstIqSample requires nothing: it is by definition the raw
// device-reported sample-0 coordinate, which is exactly why it may never be
// silently promoted to an RMARKER.
inline uint32_t timestamp_required_corrections(TimestampMarker m)
{
    constexpr uint32_t rx_chain = kCorrectionRxSampleToFirstPath |
                                  kCorrectionWindowCrop |
                                  kCorrectionSampleRateConversion |
                                  kCorrectionFirstPathFraction |
                                  kCorrectionWaveformGeometry |
                                  kCorrectionFirstPathQualityGate;
    switch (m) {
    case TimestampMarker::UhdRxFirstIqSample:
        return kCorrectionNone;
    case TimestampMarker::PreambleStart:
    case TimestampMarker::SfdStart:
    case TimestampMarker::PhrStart:
        return rx_chain;
    case TimestampMarker::RmarkerRx:
        return rx_chain | kCorrectionRmarkerOffset;
    case TimestampMarker::RmarkerTx:
        return kCorrectionWaveformGeometry | kCorrectionTxCommandToAir |
               kCorrectionDelayedTxQuantization;
    case TimestampMarker::AntennaPlane:
        return rx_chain | kCorrectionRmarkerOffset | kCorrectionAntennaPlane;
    }
    return kCorrectionAll + 1u; // unknown marker: nothing can satisfy it
}

inline bool timestamp_corrections_satisfied(TimestampMarker m, uint32_t applied)
{
    const uint32_t need = timestamp_required_corrections(m);
    return (applied & need) == need;
}

inline const char* timestamp_correction_name(uint32_t bit)
{
    switch (bit) {
    case kCorrectionRxSampleToFirstPath:
        return "rx_sample_to_first_path";
    case kCorrectionWindowCrop:
        return "window_crop";
    case kCorrectionSampleRateConversion:
        return "sample_rate_conversion";
    case kCorrectionFirstPathFraction:
        return "first_path_fraction";
    case kCorrectionWaveformGeometry:
        return "waveform_geometry";
    case kCorrectionRmarkerOffset:
        return "rmarker_offset";
    case kCorrectionTxCommandToAir:
        return "tx_command_to_air";
    case kCorrectionDelayedTxQuantization:
        return "delayed_tx_quantization";
    case kCorrectionAntennaPlane:
        return "antenna_plane";
    case kCorrectionFirstPathQualityGate:
        return "first_path_quality_gate";
    }
    return "unknown";
}

// "rx_sample_to_first_path|window_crop|..." (low bit first), or "none".
inline std::string timestamp_corrections_to_string(uint32_t applied)
{
    if (applied == kCorrectionNone)
        return "none";
    const uint32_t last_bit = static_cast<uint32_t>(kCorrectionFirstPathQualityGate);
    std::string s;
    for (uint32_t bit = 1u; bit != 0u && bit <= last_bit; bit <<= 1) {
        if (applied & bit) {
            if (!s.empty())
                s += "|";
            s += timestamp_correction_name(bit);
        }
    }
    return s.empty() ? std::string("unknown") : s;
}

// ===========================================================================
// TimeIntervalStatus
// ===========================================================================

// Why an interval was refused.  Every non-Ok value maps to
// ExchangeStatus::InvalidTimeDomain; the distinct codes exist so QA and the
// result JSON can say WHICH contract was broken (REQ-ERR-01, REQ-OUT-01).
// Numeric values are part of the JSON schema: append only.
enum class TimeIntervalStatus : uint8_t {
    Ok = 0,
    InvalidTimestamp = 1,
    DomainNameMismatch = 2,
    TickRateMismatch = 3,
    WrapWidthMismatch = 4,
    EpochMismatch = 5,
    MarkerMismatch = 6,
    CorrectionsNotApplied = 7,
    WrapAmbiguous = 8,
    FractionNotNormalized = 9,
    DurationOutOfRange = 10,
    OrderReversed = 11
};

inline const char* time_interval_status_to_string(TimeIntervalStatus s)
{
    switch (s) {
    case TimeIntervalStatus::Ok:
        return "ok";
    case TimeIntervalStatus::InvalidTimestamp:
        return "invalid_timestamp";
    case TimeIntervalStatus::DomainNameMismatch:
        return "domain_name_mismatch";
    case TimeIntervalStatus::TickRateMismatch:
        return "tick_rate_mismatch";
    case TimeIntervalStatus::WrapWidthMismatch:
        return "wrap_width_mismatch";
    case TimeIntervalStatus::EpochMismatch:
        return "epoch_mismatch";
    case TimeIntervalStatus::MarkerMismatch:
        return "marker_mismatch";
    case TimeIntervalStatus::CorrectionsNotApplied:
        return "corrections_not_applied";
    case TimeIntervalStatus::WrapAmbiguous:
        return "wrap_ambiguous";
    case TimeIntervalStatus::FractionNotNormalized:
        return "fraction_not_normalized";
    case TimeIntervalStatus::DurationOutOfRange:
        return "duration_out_of_range";
    case TimeIntervalStatus::OrderReversed:
        return "order_reversed";
    }
    return "invalid";
}

inline bool time_interval_status_from_string(const std::string& s, TimeIntervalStatus& out)
{
    for (int i = 0; i <= static_cast<int>(TimeIntervalStatus::OrderReversed); ++i) {
        const TimeIntervalStatus c = static_cast<TimeIntervalStatus>(i);
        if (s == time_interval_status_to_string(c)) {
            out = c;
            return true;
        }
    }
    return false;
}

inline bool time_interval_status_is_ok(TimeIntervalStatus s)
{
    return s == TimeIntervalStatus::Ok;
}

// REQ-ERR-01: every time-domain failure collapses to the same terminal
// status, with the specific reason carried separately in the result's
// `interval_status` field.
inline ExchangeStatus time_interval_status_to_exchange_status(TimeIntervalStatus s)
{
    return s == TimeIntervalStatus::Ok ? ExchangeStatus::Ok
                                       : ExchangeStatus::InvalidTimeDomain;
}

// ===========================================================================
// ClockDomain
// ===========================================================================

// REQ-TIME-01 + REQ-TIME-04.  A ClockDomain answers "whose counter is this
// number?".  Two fields matter for arithmetic:
//
//   tick_rate_hz    seconds per tick.  There is NO global default: 998.4 MS/s
//                   (work grid), 737.28 MS/s, 491.52 MS/s (native) and
//                   1/(499.2e6*128) Hz (DW UUS) are four different domains.
//   timestamp_bits  width of the counter / wire field, hence the wrap period
//                   2^bits.  0 means "no wrap configured" (a monotonic 64-bit
//                   counter).  DW1000 uses 40, a truncated 32-bit field is a
//                   DIFFERENT domain, and 32 vs 40 must never share a
//                   multiplier -- hence bits is part of the identity.
//
// `name` must uniquely identify ONE counter on ONE device.  Two X410 channels
// legitimately share one device counter and therefore one domain; two DW1000
// boards must use different names (e.g. "dw1000_sn0001_ch5_uus").
//
// `epoch_id` is part of the IDENTITY, not decoration: a device reboot, a UHD
// time reset or a register clear restarts the counter, so ticks from two
// epochs are unrelated numbers even though they look like integers.
struct ClockDomain {
    std::string name;
    double tick_rate_hz = 0.0;
    uint64_t epoch_id = 0;
    uint32_t timestamp_bits = 0; // 0 == no wrap

    static bool make(const std::string& name,
                     double tick_rate_hz,
                     uint64_t epoch_id,
                     uint32_t timestamp_bits,
                     ClockDomain& out)
    {
        ClockDomain d;
        d.name = name;
        d.tick_rate_hz = tick_rate_hz;
        d.epoch_id = epoch_id;
        d.timestamp_bits = timestamp_bits;
        if (!clock_domain_is_valid(d))
            return false;
        out = d;
        return true;
    }

    bool is_valid() const { return clock_domain_is_valid(*this); }
    uint64_t wrap_period() const { return clock_domain_wrap_period(*this); }
    bool ticks_in_range(int64_t ticks) const
    {
        return clock_domain_ticks_in_range(*this, ticks);
    }
    std::string to_string() const { return clock_domain_to_string(*this); }
    std::string to_json_string() const { return clock_domain_to_json_string(*this); }
};

inline bool clock_domain_is_valid(const ClockDomain& d)
{
    if (d.name.empty())
        return false;
    if (!std::isfinite(d.tick_rate_hz) || d.tick_rate_hz <= 0.0)
        return false;
    if (d.timestamp_bits != 0) {
        // A real timestamp field is at least 8 bits wide; 64 would be
        // indistinguishable from "no wrap" and is refused so the intent of the
        // domain has to be stated explicitly.
        if (d.timestamp_bits < 8u || d.timestamp_bits > 63u)
            return false;
    }
    return true;
}

inline uint64_t clock_domain_wrap_period(const ClockDomain& d)
{
    if (d.timestamp_bits == 0u || d.timestamp_bits > 63u)
        return 0u;
    return 1ULL << d.timestamp_bits;
}

inline bool clock_domain_ticks_in_range(const ClockDomain& d, int64_t ticks)
{
    if (ticks < 0)
        return false;
    const uint64_t p = clock_domain_wrap_period(d);
    if (p == 0u)
        return true;
    return static_cast<uint64_t>(ticks) < p;
}

// Largest interval that is unambiguous modulo the wrap: HALF the period,
// i.e. 2^(bits-1) ticks.  Beyond that the modular difference is genuinely
// ambiguous (a backwards interval longer than half a period is
// indistinguishable from a short forwards one) and MUST be rejected rather
// than guessed.  Returns false when the domain declares no wrap width.
inline bool clock_domain_unambiguous_ticks(const ClockDomain& d, uint64_t& out_ticks)
{
    const uint64_t p = clock_domain_wrap_period(d);
    if (p == 0u)
        return false;
    out_ticks = p / 2u;
    return true;
}

// The same unambiguous window as a Duration (2^(bits-1) / tick_rate).
inline bool clock_domain_unambiguous_interval_ns(const ClockDomain& d, int64_t& out_ns)
{
    uint64_t max_ticks = 0;
    if (!clock_domain_unambiguous_ticks(d, max_ticks))
        return false;
    if (max_ticks > static_cast<uint64_t>(kMaxDurationNanos))
        return false;
    Duration dur;
    if (!Duration::from_ticks(static_cast<int64_t>(max_ticks), d.tick_rate_hz, dur))
        return false;
    out_ns = dur.nanos();
    return true;
}

// Wrap period in seconds (0 when no wrap width is configured).
inline double clock_domain_wrap_period_seconds(const ClockDomain& d)
{
    const uint64_t p = clock_domain_wrap_period(d);
    if (p == 0u)
        return 0.0;
    return static_cast<double>(p) / d.tick_rate_hz;
}

// True when one tick is at least one nanosecond, i.e. the integer-ns
// `Duration` view of an interval loses nothing.  DW UUS ticks
// (1/(499.2e6*128) s ~ 15.65 ps) do NOT satisfy this.
inline bool clock_domain_tick_is_ns_or_coarser(const ClockDomain& d)
{
    return std::isfinite(d.tick_rate_hz) && d.tick_rate_hz <= 1.0e9;
}

// Same counter, ignoring the epoch: same name, same tick rate, same width.
inline bool clock_domain_same_identity(const ClockDomain& a, const ClockDomain& b)
{
    // Exact double comparison on purpose (REQ-TIME-04): a rate that differs in
    // the last bit is a different domain, and the safe direction for a
    // mismatch is a rejection.  Construct the domain ONCE and pass the same
    // ClockDomain by reference; never re-derive the rate per timestamp.
    return a.name == b.name && a.timestamp_bits == b.timestamp_bits &&
           std::memcmp(&a.tick_rate_hz, &b.tick_rate_hz, sizeof(double)) == 0;
}

inline bool clock_domain_same_epoch(const ClockDomain& a, const ClockDomain& b)
{
    return a.epoch_id == b.epoch_id;
}

// The single gate for "these two numbers live on the same timeline".
inline bool clock_domain_is_comparable(const ClockDomain& a, const ClockDomain& b)
{
    return clock_domain_same_identity(a, b) && clock_domain_same_epoch(a, b);
}

inline std::string clock_domain_to_string(const ClockDomain& d)
{
    char buf[192];
    std::snprintf(buf, sizeof(buf), "%s@%.17gHz,epoch=%llu,bits=%u", d.name.c_str(),
                  d.tick_rate_hz, static_cast<unsigned long long>(d.epoch_id),
                  d.timestamp_bits);
    return std::string(buf);
}

// ===========================================================================
// Timestamp
// ===========================================================================

// A single instant in one endpoint's own clock, with everything REQ-TIME-01
// requires recorded and nothing inferred.
//
//   ticks / frac_num / frac_den
//       ticks are integer device ticks of `domain`.  The optional sub-tick
//       fraction carries the fractional first-path position (REQ-TIME-02,
//       REQ-TIME-05).  It is kept NORMALIZED: frac_num == 0 <=> frac_den ==
//       0, otherwise 0 < frac_num < frac_den <= kMaxTimestampFractionDenominator
//       and gcd(frac_num, frac_den) == 1.  A non-normalizable value is
//       rejected, never silently rounded, because rounding here loses the
//       sub-tick information fractional first-path estimation exists to keep.
//   marker             which physical instant this is (REQ-TIME-02)
//   source             where the number came from (REQ-TIME-03)
//   applied_corrections which M1/M2 stages ran (see TimestampCorrection)
//   calibration_id     REQ-CAL-01; empty means "no calibration applied"
//   valid              an explicit flag; an invalid timestamp is refused
//                      everywhere.
struct Timestamp {
    int64_t ticks = 0;
    int32_t frac_num = 0;
    uint32_t frac_den = 0; // 0 == exact, no fraction

    ClockDomain domain;

    TimestampMarker marker = TimestampMarker::RmarkerTx;
    TimestampSource source = TimestampSource::Unknown;

    uint32_t applied_corrections = kCorrectionNone;
    std::string calibration_id;
    bool valid = false;

    // Whole ticks only.
    static bool from_ticks(int64_t ticks,
                           const ClockDomain& domain,
                           TimestampMarker marker,
                           TimestampSource source,
                           uint32_t corrections,
                           Timestamp& out)
    {
        return timestamp_make(ticks, 0, 0u, domain, marker, source, corrections, out);
    }

    // Ticks + an explicit sub-tick fraction.  A fraction >= 1 tick or a
    // negative fraction is carried into `ticks` exactly (floor semantics); it
    // is never dropped.
    static bool from_fractional_ticks(int64_t ticks,
                                      int32_t frac_num,
                                      uint32_t frac_den,
                                      const ClockDomain& domain,
                                      TimestampMarker marker,
                                      TimestampSource source,
                                      uint32_t corrections,
                                      Timestamp& out)
    {
        return timestamp_make(
            ticks, frac_num, frac_den, domain, marker, source, corrections, out);
    }

    bool is_self_consistent() const { return timestamp_is_self_consistent(*this); }
    bool is_marker_mapped() const { return timestamp_is_marker_mapped(*this); }
    bool is_range_capable() const { return timestamp_is_range_capable(*this); }
    bool is_validated_measurement() const
    {
        return timestamp_is_validated_measurement(*this);
    }
    bool has_calibration() const { return !calibration_id.empty(); }
    const char* rejection_reason() const { return timestamp_rejection_reason(*this); }
    std::string to_json_string() const { return timestamp_to_json_string(*this); }
    std::string to_decimal_string() const { return timestamp_to_decimal_string(*this); }
};

// Normalize a (ticks, num, den) triple.  Rejects a non-positive or oversized
// denominator; carries the integer part of num/den into `out_ticks` with floor
// semantics; reduces by gcd.  Returns false on overflow.
inline bool timestamp_normalize(int64_t ticks,
                                int32_t frac_num,
                                uint32_t frac_den,
                                int64_t& out_ticks,
                                int32_t& out_num,
                                uint32_t& out_den)
{
    if (frac_den == 0u) {
        if (frac_num != 0)
            return false; // an exact tick cannot carry a fraction
        return detail::ts_normalize_rational(ticks, 0, 0, out_ticks, out_num, out_den);
    }
    if (frac_den > detail::kFractionDenominatorMax)
        return false;
    return detail::ts_normalize_rational(ticks, static_cast<int64_t>(frac_num),
                                         static_cast<int64_t>(frac_den), out_ticks,
                                         out_num, out_den);
}

// out = a + b, both exact rationals, normalized.  Used by calibration.
inline bool timestamp_add_fractional(int64_t a_ticks,
                                     int32_t a_num,
                                     uint32_t a_den,
                                     int64_t b_ticks,
                                     int32_t b_num,
                                     uint32_t b_den,
                                     int64_t& out_ticks,
                                     int32_t& out_num,
                                     uint32_t& out_den)
{
    int64_t ticks = 0;
    if (!detail::ts_add_i64(a_ticks, b_ticks, ticks))
        return false;
    const int64_t ad = static_cast<int64_t>(a_den);
    const int64_t bd = static_cast<int64_t>(b_den);
    if (ad == 0 && bd == 0)
        return detail::ts_normalize_rational(ticks, 0, 0, out_ticks, out_num, out_den);
    if (ad == 0)
        return detail::ts_normalize_rational(
            ticks, static_cast<int64_t>(b_num), bd, out_ticks, out_num, out_den);
    if (bd == 0)
        return detail::ts_normalize_rational(
            ticks, static_cast<int64_t>(a_num), ad, out_ticks, out_num, out_den);
    return detail::ts_normalize_rational(
        ticks, static_cast<int64_t>(a_num) * bd + static_cast<int64_t>(b_num) * ad,
        ad * bd, out_ticks, out_num, out_den);
}

inline bool timestamp_marker_is_known(TimestampMarker m)
{
    switch (m) {
    case TimestampMarker::UhdRxFirstIqSample:
    case TimestampMarker::PreambleStart:
    case TimestampMarker::SfdStart:
    case TimestampMarker::PhrStart:
    case TimestampMarker::RmarkerTx:
    case TimestampMarker::RmarkerRx:
    case TimestampMarker::AntennaPlane:
        return true;
    }
    return false;
}

inline bool timestamp_source_is_known(TimestampSource s)
{
    switch (s) {
    case TimestampSource::Unknown:
    case TimestampSource::HardwareMeasured:
    case TimestampSource::ScheduledCalibrated:
    case TimestampSource::Estimated:
    case TimestampSource::Reconstructed:
        return true;
    }
    return false;
}

inline bool timestamp_from_string(const std::string& s, TimestampSource& out)
{
    for (int i = 0; i <= static_cast<int>(TimestampSource::Reconstructed); ++i) {
        const TimestampSource c = static_cast<TimestampSource>(i);
        if (s == timestamp_source_to_string(c)) {
            out = c;
            return true;
        }
    }
    return false;
}

inline bool timestamp_marker_from_string(const std::string& s, TimestampMarker& out)
{
    for (int i = 0; i <= static_cast<int>(TimestampMarker::AntennaPlane); ++i) {
        const TimestampMarker c = static_cast<TimestampMarker>(i);
        if (s == timestamp_marker_to_string(c)) {
            out = c;
            return true;
        }
    }
    return false;
}

// Single construction path for both Timestamp factories.  Rejects an invalid
// domain, an unknown marker/source, `TimestampSource::Unknown`, ticks outside
// the counter's window, an unknown correction bit, and any fraction that
// cannot be represented exactly.
inline bool timestamp_make(int64_t ticks,
                           int32_t frac_num,
                           uint32_t frac_den,
                           const ClockDomain& domain,
                           TimestampMarker marker,
                           TimestampSource source,
                           uint32_t corrections,
                           Timestamp& out)
{
    if (!clock_domain_is_valid(domain))
        return false;
    if (!timestamp_marker_is_known(marker))
        return false;
    if (!timestamp_source_is_known(source) || source == TimestampSource::Unknown)
        return false;
    if (corrections > kCorrectionAll)
        return false;

    int64_t nt = 0;
    int32_t nn = 0;
    uint32_t nd = 0u;
    if (!timestamp_normalize(ticks, frac_num, frac_den, nt, nn, nd))
        return false;
    if (!clock_domain_ticks_in_range(domain, nt))
        return false;

    Timestamp ts;
    ts.ticks = nt;
    ts.frac_num = nn;
    ts.frac_den = nd;
    ts.domain = domain;
    ts.marker = marker;
    ts.source = source;
    ts.applied_corrections = corrections;
    ts.calibration_id.clear();
    ts.valid = true;
    out = ts;
    return true;
}

// True when the sub-tick fraction is either absent or a canonical reduced
// rational strictly inside [0, 1) of one tick.
inline bool timestamp_fraction_is_normalized(const Timestamp& ts)
{
    if (ts.frac_num == 0)
        return ts.frac_den == 0u;
    if (ts.frac_den == 0u)
        return false;
    if (ts.frac_den > detail::kFractionDenominatorMax)
        return false;
    if (ts.frac_num < 0)
        return false;
    if (static_cast<int64_t>(ts.frac_num) >= static_cast<int64_t>(ts.frac_den))
        return false;
    return detail::ts_gcd(static_cast<int64_t>(ts.frac_num),
                          static_cast<int64_t>(ts.frac_den)) == 1;
}

// Structural validity: everything that can be checked without any other
// timestamp.  A self-inconsistent timestamp is refused by every operation.
inline bool timestamp_is_self_consistent(const Timestamp& ts)
{
    if (!ts.valid)
        return false;
    if (!clock_domain_is_valid(ts.domain))
        return false;
    if (!timestamp_marker_is_known(ts.marker))
        return false;
    if (!timestamp_source_is_known(ts.source) || ts.source == TimestampSource::Unknown)
        return false;
    if (!clock_domain_ticks_in_range(ts.domain, ts.ticks))
        return false;
    if (ts.applied_corrections > kCorrectionAll)
        return false;
    return timestamp_fraction_is_normalized(ts);
}

// The timestamp is a faithful instance of its own marker: every correction
// that marker requires has actually been applied.
inline bool timestamp_is_marker_mapped(const Timestamp& ts)
{
    return timestamp_is_self_consistent(ts) &&
           timestamp_corrections_satisfied(ts.marker, ts.applied_corrections);
}

// May this timestamp take part in a validated ToF?  Requires a measurement
// source (HardwareMeasured or ScheduledCalibrated per the frozen
// types.h helper), a protocol RMARKER, a fully applied correction chain and a
// calibration id.  A raw UhdRxFirstIqSample or an uncorrected SFD coordinate
// cannot reach this.
inline bool timestamp_is_range_capable(const Timestamp& ts)
{
    if (!timestamp_is_marker_mapped(ts))
        return false;
    if (!timestamp_source_is_measurement(ts.source))
        return false;
    if (ts.marker != TimestampMarker::RmarkerRx && ts.marker != TimestampMarker::RmarkerTx)
        return false;
    return !ts.calibration_id.empty();
}

// Strictest provenance gate (REQ-TIME-03).  A ScheduledCalibrated timestamp
// is NOT a validated measurement and must never be reported as one; only
// HardwareMeasured plus a complete correction chain plus a calibration id
// qualifies.
inline bool timestamp_is_validated_measurement(const Timestamp& ts)
{
    if (!timestamp_is_range_capable(ts))
        return false;
    return ts.source == TimestampSource::HardwareMeasured;
}

// First failing check, as a stable machine-readable tag for QA / JSON / logs.
inline const char* timestamp_rejection_reason(const Timestamp& ts)
{
    if (!ts.valid)
        return "invalid_flag";
    if (!clock_domain_is_valid(ts.domain))
        return "invalid_clock_domain";
    if (!timestamp_marker_is_known(ts.marker))
        return "unknown_marker";
    if (!timestamp_source_is_known(ts.source) || ts.source == TimestampSource::Unknown)
        return "unknown_source";
    if (!clock_domain_ticks_in_range(ts.domain, ts.ticks))
        return "ticks_out_of_domain_range";
    if (ts.applied_corrections > kCorrectionAll)
        return "unknown_correction_bit";
    if (ts.frac_num == 0) {
        if (ts.frac_den != 0u)
            return "fraction_not_normalized";
    } else if (ts.frac_den == 0u) {
        return "fraction_not_normalized";
    } else if (ts.frac_den > detail::kFractionDenominatorMax) {
        return "fraction_denominator_too_large";
    } else if (ts.frac_num < 0 ||
               static_cast<int64_t>(ts.frac_num) >= static_cast<int64_t>(ts.frac_den)) {
        return "fraction_not_normalized";
    } else if (detail::ts_gcd(static_cast<int64_t>(ts.frac_num),
                              static_cast<int64_t>(ts.frac_den)) != 1) {
        return "fraction_not_reduced";
    }

    if (!timestamp_corrections_satisfied(ts.marker, ts.applied_corrections))
        return "corrections_not_applied";
    if (ts.marker != TimestampMarker::RmarkerRx && ts.marker != TimestampMarker::RmarkerTx)
        return "marker_not_protocol_rmarker";
    if (!timestamp_source_is_measurement(ts.source))
        return "source_not_a_measurement";
    if (ts.calibration_id.empty())
        return "calibration_missing";
    if (ts.source != TimestampSource::HardwareMeasured)
        return "source_not_hardware_measured";
    return "accepted";
}

// Two timestamps may be combined into one exchange only when they share a
// single timeline (same counter, same width, same epoch).  This is the
// REQ-TIME-01 "a device restart invalidates the old exchange" rule at the
// type level.
inline bool timestamps_share_time_line(const Timestamp& a, const Timestamp& b)
{
    return timestamp_is_self_consistent(a) && timestamp_is_self_consistent(b) &&
           clock_domain_is_comparable(a.domain, b.domain);
}

// ===========================================================================
// Raw (adapter-internal) modular tick difference
// ===========================================================================

// NOT a protocol interval.  This is the plain modular counter difference an
// adapter needs to unwrap a device's wire timestamp; it deliberately ignores
// markers, sources and corrections.  `timestamp_interval()` is the only
// function that may produce an interval used by the ToF math.
struct TickDelta {
    TimeIntervalStatus status = TimeIntervalStatus::InvalidTimestamp;
    uint64_t ticks = 0;
    bool wrapped = false; // the modular difference crossed the wrap point
};

inline TickDelta raw_tick_delta(const ClockDomain& d, int64_t later_ticks, int64_t earlier_ticks)
{
    TickDelta r;
    if (!clock_domain_is_valid(d))
        return r;
    if (!clock_domain_ticks_in_range(d, later_ticks) ||
        !clock_domain_ticks_in_range(d, earlier_ticks))
        return r;

    const uint64_t later = static_cast<uint64_t>(later_ticks);
    const uint64_t earlier = static_cast<uint64_t>(earlier_ticks);
    const uint64_t p = clock_domain_wrap_period(d);

    if (p == 0u) {
        // No wrap width: the counter is monotonic, so a decrease is a bug and
        // must not be "repaired" by adding a period.
        if (later < earlier) {
            r.status = TimeIntervalStatus::OrderReversed;
            return r;
        }
        r.ticks = later - earlier;
    } else {
        r.wrapped = later < earlier;
        const uint64_t d_ticks = r.wrapped ? (later + p - earlier) : (later - earlier);
        // Half the period is the unambiguous window; beyond it the modular
        // difference cannot be resolved without guessing (REQ-TIME-04).
        if (d_ticks > p / 2u) {
            r.status = TimeIntervalStatus::WrapAmbiguous;
            r.ticks = 0;
            return r;
        }
        r.ticks = d_ticks;
    }
    r.status = TimeIntervalStatus::Ok;
    return r;
}

// ===========================================================================
// TimeInterval
// ===========================================================================

// A validated, same-domain interval.
//
// `ticks` + `frac_num`/`frac_den` are the EXACT tick-space value, including
// the sub-tick part.  `duration` is the integer-nanosecond view required by
// REQ-API-02, converted from the full tick+fraction value; for domains whose
// tick is finer than a nanosecond (DW UUS, ~15.65 ps) that projection is
// necessarily a coarse rounding, which is exactly what `duration_is_tick_exact`
// reports.  The ToF core must use the tick-space fields, never the nanosecond
// projection, whenever `duration_is_tick_exact` is false.
struct TimeInterval {
    TimeIntervalStatus status = TimeIntervalStatus::InvalidTimestamp;

    ClockDomain domain;
    TimestampMarker later_marker = TimestampMarker::RmarkerRx;
    TimestampMarker earlier_marker = TimestampMarker::RmarkerTx;

    int64_t ticks = 0;
    int32_t frac_num = 0;
    uint32_t frac_den = 0;
    bool wrapped = false;

    Duration duration = Duration();
    bool duration_is_tick_exact = false;
};

// The central safety function (REQ-TIME-01).
//
// `later - earlier` is computed ONLY when both timestamps live on the same
// timeline: same clock name, same tick rate, same wrap width AND same epoch.
// Every other case returns a status and leaves the interval unusable.  There
// is deliberately no overload taking two domains plus a conversion factor,
// because REQ-TIME-01 forbids mixing absolute timestamps across domains and
// REQ-TIME-04 forbids a global ticks multiplier.
//
// Check order (first failure wins):
//   1. structural validity / fraction normalization  -> InvalidTimestamp,
//      FractionNotNormalized
//   2. clock name                                    -> DomainNameMismatch
//   3. tick rate                                     -> TickRateMismatch
//   4. wrap width                                    -> WrapWidthMismatch
//   5. epoch                                         -> EpochMismatch
//   6. marker pair legality                          -> MarkerMismatch
//   7. applied correction chain on both sides        -> CorrectionsNotApplied
//   8. modular resolution / ambiguity                -> WrapAmbiguous,
//                                                       OrderReversed
//   9. exact sub-tick combination                    -> FractionNotNormalized
//  10. nanosecond projection overflow                -> DurationOutOfRange
inline TimeInterval timestamp_interval(const Timestamp& later, const Timestamp& earlier)
{
    TimeInterval r;
    r.later_marker = later.marker;
    r.earlier_marker = earlier.marker;

    // 1. structural validity, including fraction normalization
    const bool frac_bad =
        !timestamp_fraction_is_normalized(later) ||
        !timestamp_fraction_is_normalized(earlier);
    if (!timestamp_is_self_consistent(later) || !timestamp_is_self_consistent(earlier)) {
        r.status = frac_bad ? TimeIntervalStatus::FractionNotNormalized
                            : TimeIntervalStatus::InvalidTimestamp;
        return r;
    }

    // 2-4. same counter?  name, tick rate and wrap width are the identity.
    if (later.domain.name != earlier.domain.name) {
        r.status = TimeIntervalStatus::DomainNameMismatch;
        return r;
    }
    if (std::memcmp(&later.domain.tick_rate_hz, &earlier.domain.tick_rate_hz,
                    sizeof(double)) != 0) {
        r.status = TimeIntervalStatus::TickRateMismatch;
        return r;
    }
    if (later.domain.timestamp_bits != earlier.domain.timestamp_bits) {
        r.status = TimeIntervalStatus::WrapWidthMismatch;
        return r;
    }

    // 5. same epoch?  a reboot / time reset makes older ticks meaningless.
    if (!clock_domain_same_epoch(later.domain, earlier.domain)) {
        r.status = TimeIntervalStatus::EpochMismatch;
        return r;
    }
    r.domain = later.domain;

    // 6. marker semantics: the two markers must denote a difference the
    //    protocol actually defines.  A UhdRxFirstIqSample is never silently
    //    equated with an RmarkerRx.
    if (!timestamp_marker_interval_allowed(later.marker, earlier.marker)) {
        r.status = TimeIntervalStatus::MarkerMismatch;
        return r;
    }

    // 7. both timestamps must be faithful instances of their marker
    //    (REQ-TIME-02): no interval across an unapplied mapping.
    if (!timestamp_corrections_satisfied(later.marker, later.applied_corrections) ||
        !timestamp_corrections_satisfied(earlier.marker, earlier.applied_corrections)) {
        r.status = TimeIntervalStatus::CorrectionsNotApplied;
        return r;
    }

    // 8. modular tick difference with wrap resolution / ambiguity refusal
    const TickDelta d = raw_tick_delta(r.domain, later.ticks, earlier.ticks);
    if (d.status != TimeIntervalStatus::Ok) {
        r.status = d.status;
        return r;
    }
    r.wrapped = d.wrapped;

    // 9. exact sub-tick part: frac_later - frac_earlier, carried into the
    //    whole-tick count with floor semantics.
    int64_t whole = 0;
    if (!detail::ts_sub_i64(static_cast<int64_t>(d.ticks), 0, whole)) {
        r.status = TimeIntervalStatus::DurationOutOfRange;
        return r;
    }
    if (whole > static_cast<int64_t>(std::numeric_limits<int64_t>::max())) {
        r.status = TimeIntervalStatus::DurationOutOfRange;
        return r;
    }

    if (later.frac_den != 0u || earlier.frac_den != 0u) {
        int64_t num = 0;
        int64_t den = 0;
        if (later.frac_den == 0u) {
            num = -static_cast<int64_t>(earlier.frac_num);
            den = static_cast<int64_t>(earlier.frac_den);
        } else if (earlier.frac_den == 0u) {
            num = static_cast<int64_t>(later.frac_num);
            den = static_cast<int64_t>(later.frac_den);
        } else {
            // den = db*de <= 32767^2 < 2^31 and |num| < 2^32, so both are
            // exact in int64.
            const int64_t a = static_cast<int64_t>(later.frac_num);
            const int64_t b = static_cast<int64_t>(later.frac_den);
            const int64_t c = static_cast<int64_t>(earlier.frac_num);
            const int64_t e = static_cast<int64_t>(earlier.frac_den);
            num = a * e - c * b;
            den = b * e;
        }
        if (num < 0) {
            if (!detail::ts_sub_i64(whole, 1, whole)) {
                r.status = TimeIntervalStatus::DurationOutOfRange;
                return r;
            }
            num += den;
        }
        if (num < 0 || den <= 0) {
            r.status = TimeIntervalStatus::FractionNotNormalized;
            return r;
        }
        const int64_t g = detail::ts_gcd(num, den);
        num /= g;
        den /= g;
        if (den > static_cast<int64_t>(detail::kFractionDenominatorMax)) {
            // Cannot store the exact value without losing it -> refuse.
            r.status = TimeIntervalStatus::FractionNotNormalized;
            return r;
        }
        if (num == 0) {
            r.frac_num = 0;
            r.frac_den = 0u;
        } else {
            r.frac_num = static_cast<int32_t>(num);
            r.frac_den = static_cast<uint32_t>(den);
        }
        if (whole < 0) {
            // A negative interval is never a valid TWR interval; REQ-PROTO-03
            // forbids clamping it to zero and calling it a success.
            r.status = TimeIntervalStatus::DurationOutOfRange;
            r.frac_num = 0;
            r.frac_den = 0u;
            return r;
        }
    }

    r.ticks = whole;

    // 10. nanosecond projection.  Req-API-02 wants SI time at the API
    //     boundary; the tick-space fields above stay exact for sub-nanosecond
    //     domains such as DW UUS.  `duration_is_tick_exact` is true only when
    //     the interval is a whole number of ticks AND one tick spans at least
    //     a nanosecond -- then `duration` is within one tick period (< 1 ns)
    //     of the exact value and nothing finer was discarded.
    r.duration_is_tick_exact = (r.frac_den == 0u) &&
                               clock_domain_tick_is_ns_or_coarser(r.domain);
    double secs = static_cast<double>(whole) / r.domain.tick_rate_hz;
    if (r.frac_den != 0u) {
        secs += (static_cast<double>(r.frac_num) / static_cast<double>(r.frac_den)) /
                r.domain.tick_rate_hz;
    }
    if (!Duration::from_seconds(secs, r.duration)) {
        r.status = TimeIntervalStatus::DurationOutOfRange;
        r.ticks = 0;
        r.frac_num = 0;
        r.frac_den = 0u;
        return r;
    }

    r.status = TimeIntervalStatus::Ok;
    return r;
}

// Bool-returning convenience form for call sites that only want the Duration.
inline bool timestamp_interval_to_duration(const Timestamp& later,
                                           const Timestamp& earlier,
                                           Duration& out,
                                           TimeIntervalStatus& status)
{
    const TimeInterval r = timestamp_interval(later, earlier);
    status = r.status;
    if (r.status != TimeIntervalStatus::Ok)
        return false;
    out = r.duration;
    return true;
}

// Ordering helper with the same domain/epoch gate as `timestamp_interval()`.
// `out` is set to true when `a` is strictly earlier than `b`, false when the
// two denote the same instant.  Returns false (leaving `out` untouched) when
// the two are not on one timeline, when either is structurally invalid, or
// when the relation is ambiguous modulo the wrap.  The sub-tick fraction is
// not consulted: two timestamps one fraction apart compare equal here, which
// is harmless for ordering and keeps the check purely modular.
inline bool timestamp_precedes(const Timestamp& a, const Timestamp& b, bool& out)
{
    if (!timestamp_is_self_consistent(a) || !timestamp_is_self_consistent(b))
        return false;
    if (!clock_domain_is_comparable(a.domain, b.domain))
        return false;
    const uint64_t p = clock_domain_wrap_period(a.domain);
    const uint64_t from = static_cast<uint64_t>(a.ticks);
    const uint64_t to = static_cast<uint64_t>(b.ticks);
    uint64_t dist = 0;
    if (p == 0u) {
        // Monotonic counter: a decrease is a definite ordering, not an error.
        if (to < from) {
            out = false;
            return true;
        }
        dist = to - from;
    } else {
        dist = (to >= from) ? (to - from) : (to + p - from);
        if (dist > p / 2u)
            return false; // ambiguous modulo the wrap
    }
    out = dist != 0;
    return true;
}

// ===========================================================================
// Calibration (REQ-CAL-01)
// ===========================================================================

enum class CalibrationResult : uint8_t {
    Applied = 0,
    InvalidTimestamp = 1,
    EmptyCalibrationId = 2,
    AlreadyApplied = 3,         // same id already present: double application
    ConflictingCalibration = 4, // a DIFFERENT id is already present
    OffsetOutOfRange = 5
};

inline const char* calibration_result_to_string(CalibrationResult r)
{
    switch (r) {
    case CalibrationResult::Applied:
        return "applied";
    case CalibrationResult::InvalidTimestamp:
        return "invalid_timestamp";
    case CalibrationResult::EmptyCalibrationId:
        return "empty_calibration_id";
    case CalibrationResult::AlreadyApplied:
        return "already_applied";
    case CalibrationResult::ConflictingCalibration:
        return "conflicting_calibration";
    case CalibrationResult::OffsetOutOfRange:
        return "offset_out_of_range";
    }
    return "invalid";
}

// A calibration may be applied EXACTLY ONCE per timestamp.  The id is stored
// on the timestamp, so:
//
//   * applying the same id twice    -> CalibrationResult::AlreadyApplied;
//   * applying a different id to an already calibrated timestamp
//                                   -> CalibrationResult::ConflictingCalibration;
//   * an empty id                   -> refused.
//
// The offset is given in TICKS of the timestamp's own domain (the exact path);
// `apply_calibration_duration()` converts a SI Duration for call sites that
// already hold a calibrated link delay.  Neither variant changes `marker`
// implicitly: REQ-TIME-02 forbids collapsing markers, and the antenna-plane
// step is an explicit, separately audited operation
// (`timestamp_retag_marker()`).
inline CalibrationResult apply_calibration_ticks(Timestamp& ts,
                                                 const std::string& calibration_id,
                                                 int64_t offset_ticks,
                                                 int32_t offset_frac_num,
                                                 uint32_t offset_frac_den,
                                                 uint32_t additional_corrections)
{
    if (!timestamp_is_self_consistent(ts))
        return CalibrationResult::InvalidTimestamp;
    if (calibration_id.empty())
        return CalibrationResult::EmptyCalibrationId;
    if (!ts.calibration_id.empty()) {
        return ts.calibration_id == calibration_id
                   ? CalibrationResult::AlreadyApplied
                   : CalibrationResult::ConflictingCalibration;
    }
    if ((ts.applied_corrections | additional_corrections) > kCorrectionAll)
        return CalibrationResult::OffsetOutOfRange;

    int64_t nt = 0;
    int32_t nn = 0;
    uint32_t nd = 0u;
    if (!timestamp_add_fractional(ts.ticks, ts.frac_num, ts.frac_den, offset_ticks,
                                  offset_frac_num, offset_frac_den, nt, nn, nd))
        return CalibrationResult::OffsetOutOfRange;
    if (!clock_domain_ticks_in_range(ts.domain, nt))
        return CalibrationResult::OffsetOutOfRange;

    ts.ticks = nt;
    ts.frac_num = nn;
    ts.frac_den = nd;
    ts.applied_corrections |= additional_corrections;
    ts.calibration_id = calibration_id;
    return CalibrationResult::Applied;
}

// SI-duration form of `apply_calibration_ticks()`.  Sub-tick precision is
// kept when the domain's tick is finer than a nanosecond (DW UUS), which is
// exactly the antenna-delay case.  The offset is quantized to 1/16384 of a
// tick, which is far below any meaningful first-path resolution.  A caller
// that needs bit-exact behaviour must use the tick form; the double round
// trip here is documented, not silent.  If the exact sum cannot be stored
// under the fraction-denominator cap, the call is REFUSED with
// OffsetOutOfRange rather than rounded.
inline CalibrationResult apply_calibration_duration(Timestamp& ts,
                                                    const std::string& calibration_id,
                                                    Duration offset,
                                                    uint32_t additional_corrections)
{
    if (!timestamp_is_self_consistent(ts))
        return CalibrationResult::InvalidTimestamp;
    if (calibration_id.empty())
        return CalibrationResult::EmptyCalibrationId;
    if (!std::isfinite(ts.domain.tick_rate_hz) || ts.domain.tick_rate_hz <= 0.0)
        return CalibrationResult::OffsetOutOfRange;

    constexpr double kMaxExactDouble = 9.007199254740992e15;
    const double t = offset.seconds() * ts.domain.tick_rate_hz;
    if (!std::isfinite(t) || std::fabs(t) > kMaxExactDouble)
        return CalibrationResult::OffsetOutOfRange;
    const double whole_f = std::floor(t);
    const double frac_f = t - whole_f;
    const int64_t whole = static_cast<int64_t>(whole_f);
    int64_t frac_num = std::llround(frac_f * 16384.0);
    if (frac_num < 0)
        frac_num = 0;
    if (frac_num > 16384)
        frac_num = 16384; // == den: normalizer carries it into the ticks

    return apply_calibration_ticks(ts, calibration_id, whole,
                                   static_cast<int32_t>(frac_num), 16384u,
                                   additional_corrections);
}

inline bool timestamp_has_calibration(const Timestamp& ts, const std::string& calibration_id)
{
    return !calibration_id.empty() && ts.calibration_id == calibration_id;
}

// Retag a timestamp's marker once the corresponding mapping has been applied.
// Refused when the new marker's required corrections are not all present, so a
// caller cannot promote a raw sample-0 time into an RMARKER by fiat.
inline bool timestamp_retag_marker(Timestamp& ts, TimestampMarker new_marker)
{
    if (!timestamp_is_self_consistent(ts))
        return false;
    if (!timestamp_marker_is_known(new_marker))
        return false;
    if (!timestamp_corrections_satisfied(new_marker, ts.applied_corrections))
        return false;
    ts.marker = new_marker;
    return true;
}

// ===========================================================================
// REQ-OUT-01: integer-exact JSON / decimal formatting
// ===========================================================================

inline std::string to_decimal_string(int64_t v) { return std::to_string(v); }

// 2^53-1.  Beyond this a JSON number cannot be read back without loss.
inline bool integer_needs_decimal_string(int64_t v)
{
    return v > kJsonSafeIntegerMax || v < -kJsonSafeIntegerMax;
}

inline std::string clock_domain_to_json_string(const ClockDomain& d)
{
    char buf[320];
    std::snprintf(buf, sizeof(buf),
                  "{\"name\":\"%s\",\"tick_rate_hz\":%.17g,\"epoch_id\":%llu,"
                  "\"timestamp_bits\":%u}",
                  d.name.c_str(), d.tick_rate_hz,
                  static_cast<unsigned long long>(d.epoch_id), d.timestamp_bits);
    return std::string(buf);
}

// "<ticks>" for an exact tick, "<ticks>+<num>/<den>" when a sub-tick fraction
// is present.  Never lossy: the fraction is printed as an exact rational.
inline std::string timestamp_to_decimal_string(const Timestamp& ts)
{
    char buf[64];
    if (ts.frac_num == 0 || ts.frac_den == 0u) {
        std::snprintf(buf, sizeof(buf), "%lld", static_cast<long long>(ts.ticks));
        return std::string(buf);
    }
    std::snprintf(buf, sizeof(buf), "%lld+%d/%u", static_cast<long long>(ts.ticks),
                  static_cast<int>(ts.frac_num), static_cast<unsigned>(ts.frac_den));
    return std::string(buf);
}

// Canonical single-line JSON.  `ticks` becomes a quoted decimal string when
// it is outside the 2^53-1 safe range; every other integer is small enough to
// stay numeric.  `timestamp_from_json_string()` is the strict inverse.
inline std::string timestamp_to_json_string(const Timestamp& ts)
{
    char buf[1400];
    std::string ticks;
    if (integer_needs_decimal_string(ts.ticks)) {
        ticks = "\"" + to_decimal_string(ts.ticks) + "\"";
    } else {
        char tb[32];
        std::snprintf(tb, sizeof(tb), "%lld", static_cast<long long>(ts.ticks));
        ticks = std::string(tb);
    }
    std::snprintf(
        buf, sizeof(buf),
        "{\"ticks\":%s,\"frac_num\":%d,\"frac_den\":%u,\"clock_domain\":%s,"
        "\"marker\":\"%s\",\"source\":\"%s\",\"corrections\":%u,"
        "\"corrections_flags\":\"%s\",\"calibration_id\":\"%s\",\"valid\":%s}",
        ticks.c_str(), static_cast<int>(ts.frac_num), static_cast<unsigned>(ts.frac_den),
        clock_domain_to_json_string(ts.domain).c_str(), timestamp_marker_to_string(ts.marker),
        timestamp_source_to_string(ts.source), ts.applied_corrections,
        timestamp_corrections_to_string(ts.applied_corrections).c_str(),
        ts.calibration_id.c_str(), ts.valid ? "true" : "false");
    return std::string(buf);
}

inline std::string time_interval_to_json_string(const TimeInterval& ti)
{
    char buf[1024];
    std::snprintf(buf, sizeof(buf),
                  "{\"status\":\"%s\",\"ticks\":%lld,\"frac_num\":%d,\"frac_den\":%u,"
                  "\"wrapped\":%s,\"duration_ns\":%lld,\"duration_is_tick_exact\":%s,"
                  "\"clock_domain\":%s,\"later_marker\":\"%s\","
                  "\"earlier_marker\":\"%s\"}",
                  time_interval_status_to_string(ti.status),
                  static_cast<long long>(ti.ticks), static_cast<int>(ti.frac_num),
                  static_cast<unsigned>(ti.frac_den), ti.wrapped ? "true" : "false",
                  static_cast<long long>(ti.duration.nanos()),
                  ti.duration_is_tick_exact ? "true" : "false",
                  clock_domain_to_json_string(ti.domain).c_str(),
                  timestamp_marker_to_string(ti.later_marker),
                  timestamp_marker_to_string(ti.earlier_marker));
    return std::string(buf);
}

namespace detail {

// Strict, allocation-light scanner for exactly the shape emitted by
// `timestamp_to_json_string()`.  Anything else -- reordered keys, unknown
// keys, trailing garbage, escapes, NaN/Inf -- is rejected rather than
// guessed, because a silently mis-parsed result timestamp is worse than a
// missing one.
struct JsonCursor {
    const char* p = nullptr;
    const char* end = nullptr;

    bool eat(char c)
    {
        if (p >= end || *p != c)
            return false;
        ++p;
        return true;
    }
    bool peek(char c) const { return p < end && *p == c; }
    bool lit(const char* s)
    {
        const size_t n = std::strlen(s);
        if (static_cast<size_t>(end - p) < n || std::memcmp(p, s, n) != 0)
            return false;
        p += n;
        return true;
    }
    bool key(const char* name)
    {
        if (!eat('"') || !lit(name) || !eat('"'))
            return false;
        return eat(':');
    }
    bool boolean(bool& out)
    {
        if (lit("true")) {
            out = true;
            return true;
        }
        if (lit("false")) {
            out = false;
            return true;
        }
        return false;
    }
    // Accepts a bare JSON integer or a quoted decimal integer string.
    bool integer(int64_t& out)
    {
        const bool quoted = peek('"');
        if (quoted)
            ++p;
        const char* s = p;
        if (p < end && *p == '-')
            ++p;
        const char* digits = p;
        while (p < end && *p >= '0' && *p <= '9')
            ++p;
        if (p == digits)
            return false;
        const std::string text(s, static_cast<size_t>(p - s));
        if (quoted && !eat('"'))
            return false;
        if (text.size() > 19)
            return false;
        char* stop = nullptr;
        errno = 0;
        const long long v = std::strtoll(text.c_str(), &stop, 10);
        if (stop == nullptr || *stop != '\0')
            return false;
        out = static_cast<int64_t>(v);
        return true;
    }
    // Accepts a bare JSON number or a quoted decimal real.  The scan already
    // excludes "nan"/"inf" spellings, and the isfinite() check catches the
    // overflow form.
    bool real(double& out)
    {
        const bool quoted = peek('"');
        if (quoted)
            ++p;
        const char* s = p;
        if (p < end && *p == '-')
            ++p;
        const char* digits = p;
        while (p < end && *p >= '0' && *p <= '9')
            ++p;
        if (p == digits)
            return false;
        if (p < end && *p == '.') {
            ++p;
            while (p < end && *p >= '0' && *p <= '9')
                ++p;
        }
        if (p < end && (*p == 'e' || *p == 'E')) {
            ++p;
            if (p < end && (*p == '-' || *p == '+'))
                ++p;
            const char* e = p;
            while (p < end && *p >= '0' && *p <= '9')
                ++p;
            if (p == e)
                return false;
        }
        const std::string text(s, static_cast<size_t>(p - s));
        if (quoted && !eat('"'))
            return false;
        char* stop = nullptr;
        errno = 0;
        const double v = std::strtod(text.c_str(), &stop);
        if (stop == nullptr || *stop != '\0' || !std::isfinite(v))
            return false;
        out = v;
        return true;
    }
    // JSON string with no escapes; the emitted form never contains any, so an
    // escape is a sign the input was not produced by this header.
    bool string(std::string& out)
    {
        if (!eat('"'))
            return false;
        const char* s = p;
        while (p < end && *p != '"') {
            if (*p == '\\')
                return false;
            ++p;
        }
        if (p >= end)
            return false;
        out.assign(s, static_cast<size_t>(p - s));
        return eat('"');
    }
    // A nested object; used for the inline clock_domain so the whole
    // timestamp stays on one line.  `out` receives the braces too, so the
    // result can be handed to a fresh JsonCursor.
    bool raw_object(std::string& out)
    {
        if (!eat('{'))
            return false;
        const char* s = p; // just after the '{'
        int depth = 1;
        while (p < end) {
            if (*p == '{') {
                ++depth;
                ++p;
            } else if (*p == '}') {
                --depth;
                if (depth == 0)
                    break;
                ++p;
            } else if (*p == '"') {
                ++p;
                while (p < end && *p != '"') {
                    if (*p == '\\')
                        return false;
                    ++p;
                }
                if (p >= end)
                    return false;
                ++p; // past the closing quote
            } else {
                ++p;
            }
        }
        if (p >= end)
            return false;
        out.assign(s - 1, static_cast<size_t>(p - s + 2)); // '{' .. '}'
        return eat('}');
    }
};

} // namespace detail

// Strict inverse of `timestamp_to_json_string()`.  Rejects a truncated or
// reordered object, a bad fraction, an unknown marker/source, a non-positive
// or non-finite tick rate and a corrupt correction bitmask.  Structural
// validity only: `timestamp_is_self_consistent()` remains the semantic gate
// and stays the caller's decision.
inline bool timestamp_from_json_string(const std::string& json, Timestamp& out)
{
    detail::JsonCursor c{json.c_str(), json.c_str() + json.size()};
    Timestamp ts;
    int64_t frac_num_tmp = 0;
    int64_t frac_den_tmp = 0;
    if (!c.eat('{'))
        return false;

    if (!c.key("ticks") || !c.integer(ts.ticks))
        return false;
    if (!c.eat(','))
        return false;
    if (!c.key("frac_num") || !c.integer(frac_num_tmp))
        return false;
    if (!c.eat(','))
        return false;
    if (!c.key("frac_den") || !c.integer(frac_den_tmp))
        return false;
    if (frac_num_tmp < 0 || frac_num_tmp > std::numeric_limits<int32_t>::max())
        return false;
    if (frac_den_tmp < 0 || frac_den_tmp > std::numeric_limits<uint32_t>::max())
        return false;
    ts.frac_num = static_cast<int32_t>(frac_num_tmp);
    ts.frac_den = static_cast<uint32_t>(frac_den_tmp);
    if (!c.eat(','))
        return false;

    if (!c.key("clock_domain"))
        return false;
    std::string domain_raw;
    if (!c.raw_object(domain_raw))
        return false;
    {
        detail::JsonCursor d{domain_raw.c_str(), domain_raw.c_str() + domain_raw.size()};
        if (!d.eat('{'))
            return false;
        if (!d.key("name") || !d.string(ts.domain.name))
            return false;
        if (!d.eat(','))
            return false;
        if (!d.key("tick_rate_hz") || !d.real(ts.domain.tick_rate_hz))
            return false;
        if (!d.eat(','))
            return false;
        int64_t epoch = 0;
        if (!d.key("epoch_id") || !d.integer(epoch))
            return false;
        if (epoch < 0)
            return false;
        ts.domain.epoch_id = static_cast<uint64_t>(epoch);
        if (!d.eat(','))
            return false;
        int64_t bits = 0;
        if (!d.key("timestamp_bits") || !d.integer(bits))
            return false;
        if (bits < 0 || bits > 63)
            return false;
        ts.domain.timestamp_bits = static_cast<uint32_t>(bits);
        if (!d.eat('}'))
            return false;
    }
    if (!c.eat(','))
        return false;

    {
        std::string m;
        if (!c.key("marker") || !c.string(m))
            return false;
        if (!timestamp_marker_from_string(m, ts.marker))
            return false;
    }
    if (!c.eat(','))
        return false;
    {
        std::string s;
        if (!c.key("source") || !c.string(s))
            return false;
        if (!timestamp_from_string(s, ts.source))
            return false;
    }
    if (!c.eat(','))
        return false;
    {
        int64_t bits = 0;
        if (!c.key("corrections") || !c.integer(bits))
            return false;
        if (bits < 0 || bits > static_cast<int64_t>(kCorrectionAll))
            return false;
        ts.applied_corrections = static_cast<uint32_t>(bits);
    }
    if (!c.eat(','))
        return false;
    {
        std::string flags;
        if (!c.key("corrections_flags") || !c.string(flags))
            return false;
        if (flags != timestamp_corrections_to_string(ts.applied_corrections))
            return false;
    }
    if (!c.eat(','))
        return false;
    if (!c.key("calibration_id") || !c.string(ts.calibration_id))
        return false;
    if (!c.eat(','))
        return false;
    if (!c.key("valid") || !c.boolean(ts.valid))
        return false;
    if (!c.eat('}'))
        return false;
    if (c.p != c.end)
        return false; // trailing garbage

    if (!clock_domain_is_valid(ts.domain))
        return false;
    out = ts;
    return true;
}

} // namespace twr
} // namespace uwb
} // namespace gr

#endif /* INCLUDED_GNURADIO_UWB_UWB_TWR_TIMESTAMP_H */
