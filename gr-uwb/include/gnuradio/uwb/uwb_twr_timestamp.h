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
 *
 * ---------------------------------------------------------------------------
 * M0.1 corrections to the M0 time contract (defects R4 and R5)
 * ---------------------------------------------------------------------------
 *
 * R4 -- the "exact" flag was not exact.  M0 shipped one boolean,
 * `TimeInterval::duration_is_tick_exact`, meant as "use ticks, not ns".  It was
 * computed as `frac == 0 && tick_rate <= 1e9`, i.e. "the interval is whole
 * ticks AND one tick is at least 1 ns".  The second clause bounds the ns
 * projection error at less than one tick; it does not make it zero.  At
 * 737.28 MS/s a 1-tick interval was reported `nanos() == 1` with the flag SET
 * while the true value is 1.356336806 ns: 0.356 ns silently discarded.  A
 * report that tells M1 "use ticks when the flag is set" is therefore unsafe.
 *
 *   * The flag is split into two honest, independent questions:
 *       `TimeInterval::is_whole_ticks`         -- is the VALUE a whole number
 *                                                 of device ticks?
 *       `TimeInterval::duration_is_lossless_ns` -- does the integer-ns
 *                                                 projection reproduce the
 *                                                 value with NO error?
 *     The second is decided per INTERVAL by an exact rational test
 *     (`timestamp_tick_ns_projection_is_lossless`), never by a tick-rate
 *     threshold.  `clock_domain_tick_is_ns_or_coarser()` survives only as a
 *     RATE-LEVEL QUERY and is documented as never being the lossless test.
 *     `duration_is_tick_exact` is GONE; the misleading name must not survive.
 *   * The nanosecond projection has a stated rounding rule -- TRUNCATE toward
 *     zero, error strictly inside the interval and always in the safe
 *     direction for a reply-delay budget.  It is applied explicitly, not left
 *     as an accident of `Duration::from_seconds`.
 *   * `RelativeTickInterval` + `timestamp_relative_interval()` is the SAFE form
 *     for M1: the exact rational tick value in a type that has NO nanosecond
 *     field, so a lossy projection cannot be read even by accident.
 *
 * R5 -- modular ordering accepted the ambiguous case.  M0's
 * `timestamp_precedes()` refused only `dist > P/2`, so a pair exactly half a
 * period apart was reported "a precedes b" AND "b precedes a", and the
 * sub-tick fraction was ignored, so 0.25 and 0.75 on the same tick compared
 * equal.
 *
 *   * The half-period bound is now STRICT (`dist < P/2`) and is applied
 *     consistently in both directions, in `timestamp_precedes()` and in the new
 *     `timestamp_compare()` alike.
 *   * `timestamp_compare()` orders sub-tick fractions exactly (with the
 *     borrow that a modular difference needs), is three-valued, and returns
 *     `TimestampOrder::Indeterminate` -- never an order -- when no definite
 *     relation exists.  It ALWAYS writes `out`, so a caller that ignores the
 *     return value still cannot read an invented order.
 *   * The half-period rule is documented as the strategy it is: a sound bound
 *     that needs no extra history, sound only up to P/2, and NOT a hardware
 *     law.  `TimestampOrderBudget` lets a caller declare the real
 *     maximum-exchange-duration bound; the declaration is clamped to P/2
 *     because exceeding it would destroy antisymmetry.
 *   * `raw_tick_delta()` still ACCEPTS d == P/2, deliberately and for a
 *     different reason: `timestamp_interval()` is given the (later, earlier)
 *     pair, so the sign is supplied and only the magnitude is derived, and at
 *     d == P/2 the magnitude is unique.  It is only ORDERING that has to
 *     derive the direction itself.
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

// RATE-LEVEL QUERY ONLY.  True when one tick is at least one nanosecond long.
//
// M0.1 (defect R4): this predicate does NOT say that an integer-nanosecond
// projection of a tick-space value is lossless, and it never may be used that
// way.  At 998.4 MS/s one tick is 1.0016 ns and at 737.28 MS/s it is
// 1.3563 ns, so a *whole* number of ticks still projects onto a NON-integer
// number of nanoseconds: a 1-tick interval at 737.28 MS/s is 1.356336806... ns
// and the integer-ns view is 1 ns, i.e. 0.356 ns of discarded precision.  A
// tick being "at least 1 ns" bounds the projection error at < 1 tick; it does
// not make that error zero.  For the lossless question, which is per-INTERVAL
// and not per-domain, use `timestamp_tick_ns_projection_is_lossless()` (or the
// `is_whole_ticks` / `duration_is_lossless_ns` pair on a TimeInterval).
//
// It remains useful as a cheap coarse/fine screen -- a sub-nanosecond tick
// means essentially every interval is lossy -- and as a documented statement
// about the grid, which is why it is kept at all.
//
// DW UUS ticks (1/(499.2e6*128) s ~ 15.65 ps) do NOT satisfy this at all.
inline bool clock_domain_tick_is_ns_or_coarser(const ClockDomain& d)
{
    return std::isfinite(d.tick_rate_hz) && d.tick_rate_hz <= 1.0e9;
}

// ---------------------------------------------------------------------------
// R4: exact nanosecond projection
// ---------------------------------------------------------------------------
//
// The nanosecond field of a `Duration` is a *projection*: the interval's exact
// value is `ticks/rate + (num/den)/rate` seconds, and a `Duration` can only
// hold an integer number of nanoseconds.  Two questions must not be confused:
//
//   Q1 "is the tick-space value a whole number of device ticks?"  -- a
//       property of the VALUE.  Answer: `frac_num == 0` (see
//       `TimeInterval::is_whole_ticks` / `time_interval_is_whole_ticks()`).
//   Q2 "does the integer-nanosecond projection lose anything?" -- a property
//       of the VALUE *AND* the tick rate.  Answer:
//       `timestamp_tick_ns_projection_is_lossless()`.
//
// The old single flag conflated them, and answered Q2 with the rate-level test
// `tick_rate <= 1e9`.  That is the R4 defect: at 737.28 MS/s a 1-tick interval
// was reported `duration_is_tick_exact = true` with `nanos() == 1` while the
// true value is 1.356336806 ns, i.e. 0.356 ns had silently been discarded.
//
// The tests below are EXACT, not floating-point tolerances.  A finite positive
// double is exactly `m * 2^e` with `m` a 53-bit integer, so
// `1e9 / tick_rate_hz` is exactly a rational and can be computed with integer
// arithmetic only.

namespace detail {

// 1e9 == 2^9 * 1953125, and 1953125 is odd.
inline constexpr int64_t kNsPerSecondOdd = 1953125LL;
inline constexpr int kNsPerSecondPow2 = 9;

} // namespace detail

// EXACT nanoseconds in one tick of `rate_hz`, as a reduced rational
// `num/den` (num >= 1, den >= 1).  Returns false only when the exact value does
// not fit the bounded rational, which needs an absurd tick rate; the caller
// must then treat every nanosecond projection as lossy.
inline bool clock_domain_ns_per_tick_exact(double tick_rate_hz,
                                           int64_t& out_num,
                                           int64_t& out_den)
{
    out_num = 0;
    out_den = 0;
    if (!std::isfinite(tick_rate_hz) || tick_rate_hz <= 0.0)
        return false;

    // rate == mant * 2^exp exactly (frexp/ldexp are exact for binary FP).
    int be = 0;
    const double mant = std::ldexp(std::frexp(tick_rate_hz, &be), 53);
    if (!(mant >= 4503599627370496.0 && mant < 9007199254740992.0))
        return false; // not the expected normal form
    uint64_t m = static_cast<uint64_t>(mant);
    int e = be - 53; // rate == m * 2^e
    while ((m & 1u) == 0u) { // absorb the powers of two into e; m ends up odd
        m >>= 1;
        ++e;
    }

    // ns_per_tick = 1e9 / (m * 2^e) = (2^9 * 1953125) / (m * 2^e)
    const int k = detail::kNsPerSecondPow2 - e;
    int64_t num = 0;
    int64_t den = 0;
    if (k >= 0) {
        if (k > 42) // 1953125 * 2^43 > INT64_MAX
            return false;
        num = detail::kNsPerSecondOdd * (static_cast<int64_t>(1) << k);
        den = static_cast<int64_t>(m);
    } else {
        const int s = -k;
        if (s >= 62 || m > (static_cast<uint64_t>(INT64_MAX) >> s))
            return false;
        num = detail::kNsPerSecondOdd;
        den = static_cast<int64_t>(m) << s;
    }
    const int64_t g = detail::ts_gcd(num, den);
    out_num = num / g;
    out_den = den / g;
    return true;
}

namespace detail {

// EXACT test: is `ticks + frac_num/frac_den` ticks an integer number of
// nanoseconds at `rate_hz`?  The exact value is
//
//     (ticks*den + num) * Np / (den * Dp)      with Np/Dp = ns per tick
//
// and the integrality question is answered WITHOUT ever forming that product.
// With `g = gcd(Dp*den, Np)`, `Dp*den | X*Np` is equivalent to
// `(Dp*den)/g | X` for `X = ticks*den + num`, because `gcd(D/g, Np/g) == 1`.
// Only `X` and `Dp*den` are formed, both of which are checked for overflow; a
// value that cannot be proven exact is reported NOT lossless, never guessed.
inline bool ts_tick_ns_projection_is_lossless(double tick_rate_hz,
                                              int64_t ticks,
                                              int32_t frac_num,
                                              uint32_t frac_den)
{
    int64_t np = 0;
    int64_t dp = 0;
    if (!clock_domain_ns_per_tick_exact(tick_rate_hz, np, dp))
        return false;
    if (ticks < 0 || frac_num < 0 || (frac_den == 0u && frac_num != 0))
        return false;

    int64_t x = ticks; // X
    int64_t dd = dp;   // Dp*den
    if (frac_den != 0u) {
        const int64_t fden = static_cast<int64_t>(frac_den);
        if (dp > (std::numeric_limits<int64_t>::max() / fden))
            return false;
        dd = dp * fden;
        if (ticks > (std::numeric_limits<int64_t>::max() - frac_num) / fden)
            return false;
        x = ticks * fden + frac_num;
    }
    const int64_t g = ts_gcd(dd, np);
    return (x % (dd / g)) == 0;
}

// ---------------------------------------------------------------------------
// R4: the nanosecond projection rounding rule, made explicit
// ---------------------------------------------------------------------------
//
// `Duration` holds INTEGER nanoseconds, so a real-valued interval must be
// rounded.  The rule in this header is **truncate toward zero**:
//
//   ns = trunc(exact_seconds * 1e9)
//
// Why truncation, and not round-half-away-from-zero or round-half-even:
//
//   * Direction.  A TWR reply-delay budget is an upper bound: over-reporting
//     the elapsed time is how a late Response/Final becomes a "successful"
//     exchange that was really late.  Truncation can only under-report, i.e.
//     err toward "we still have time", which is the direction a human reader
//     also assumes when they see a smaller number.
//   * Stability.  Half-way cases are exactly where a double lands on either
//     side depending on the accumulated double, so a tie rule makes the
//     emitted integer depend on the path taken to compute the seconds (this
//     header computes `ticks/rate` and `frac/rate` separately).  Truncation is
//     a single monotone map, so the same tick value always yields the same
//     integer regardless of how the sum was formed.
//   * The error is bounded by exactly 1 ns, always strictly inside the
//     interval, and is always in the same direction; `half-away` would make the
//     error two-sided and up to 0.5 ns, and would still not be exact.
//
// The consequence, stated once so no caller re-derives it: for a tick rate
// finer than 1 GHz, or for any interval with a sub-tick fraction, the
// nanosecond field is a LOSSY projection.  It is fine for JSON, logs and
// API display.  It is NOT fine for ToF.  For ToF use
// `timestamp_relative_interval()`, whose type has no nanosecond field at all.
inline bool ts_project_seconds_to_nanos_trunc(double seconds, int64_t& out_ns)
{
    out_ns = 0;
    if (!std::isfinite(seconds))
        return false;
    const double ns = seconds * 1e9;
    const double limit = 9223372036854775808.0; // |INT64_MIN| exactly
    if (!(ns > -limit) || !(ns < limit))
        return false;
    const double t = std::trunc(ns);
    if (!(t >= -9223372036854774784.0)) // largest double < |INT64_MIN| + 1
        return false;
    out_ns = static_cast<int64_t>(t);
    return true;
}

} // namespace detail

// Public spelling of the R4 rules above.
inline bool timestamp_tick_ns_projection_is_lossless(double tick_rate_hz,
                                                     int64_t ticks,
                                                     int32_t frac_num,
                                                     uint32_t frac_den)
{
    return detail::ts_tick_ns_projection_is_lossless(tick_rate_hz, ticks, frac_num, frac_den);
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
// the sub-tick part.  They are the only fields the ToF core may consume.
//
// `duration` is the REQ-API-02 integer-nanosecond view of the SAME value.  It
// is a *projection* and `duration_is_lossless_ns` says whether it is a faithful
// one.  M0.1 (defect R4) split the old, misleading `duration_is_tick_exact`
// into two honest flags:
//
//   `is_whole_ticks`         the tick-space value is a whole number of device
//                            ticks (there is no sub-tick fraction).  A
//                            statement about the VALUE only.
//   `duration_is_lossless_ns` `duration` reproduces the exact interval with no
//                            error at all, i.e. the exact value is an integer
//                            number of nanoseconds.  A statement about the
//                            VALUE *AND* the tick rate; it is false whenever
//                            the rate is not an exact sub-multiple of 1 GHz
//                            for this particular tick count, and false for
//                            every sub-tick fraction that is not itself a
//                            whole number of nanoseconds.
//
// When `duration_is_lossless_ns` is false, `duration` is a TRUNCATION whose
// absolute error is < 1 ns (see `detail::ts_project_seconds_to_nanos_trunc`).
// That is acceptable for JSON / logs / the public API.  It is NOT acceptable as
// an input to the ToF math: feeding it in would inject a quantisation error
// that is comparable to the whole measurement.  Use
// `timestamp_relative_interval()` instead -- its type deliberately has no
// nanosecond field.
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

    // R4 (a): the tick-space value is a whole number of device ticks.
    bool is_whole_ticks = false;

    // R4 (b): `duration` is a LOSSLESS record of the interval.  When false,
    // `duration` is a truncation with < 1 ns error and must not drive ToF.
    bool duration_is_lossless_ns = false;
};

// The R4 whole-tick predicate, stated once.  True when the tick-space value
// carries no sub-tick fraction.
inline bool time_interval_is_whole_ticks(const TimeInterval& ti)
{
    return ti.frac_num == 0;
}

// The R4 lossless-projection predicate, stated once, for an interval's tick
// fields.  See `timestamp_tick_ns_projection_is_lossless()`.
inline bool time_interval_ns_projection_is_lossless(const TimeInterval& ti)
{
    return detail::ts_tick_ns_projection_is_lossless(
        ti.domain.tick_rate_hz, ti.ticks, ti.frac_num, ti.frac_den);
}

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

    // 10. nanosecond projection.  REQ-API-02 wants SI time at the API
    //     boundary; the tick-space fields above stay exact for sub-nanosecond
    //     domains such as DW UUS.  The projection is TRUNCATED toward zero and
    //     its exactness is decided here by actually testing the conversion
    //     (R4) -- never by the tick rate alone.
    r.is_whole_ticks = (r.frac_den == 0u);
    r.duration_is_lossless_ns = detail::ts_tick_ns_projection_is_lossless(
        r.domain.tick_rate_hz, r.ticks, r.frac_num, r.frac_den);

    double secs = static_cast<double>(whole) / r.domain.tick_rate_hz;
    if (r.frac_den != 0u) {
        secs += (static_cast<double>(r.frac_num) / static_cast<double>(r.frac_den)) /
                r.domain.tick_rate_hz;
    }
    int64_t ns = 0;
    if (!detail::ts_project_seconds_to_nanos_trunc(secs, ns)) {
        // A rejected interval is EMPTY, not partially populated: no tick count
        // and no "whole ticks" claim survive a failed projection, so a caller
        // that ignores `status` sees a zeroed value rather than a tick count it
        // might mistake for a measurement.
        r.status = TimeIntervalStatus::DurationOutOfRange;
        r.ticks = 0;
        r.frac_num = 0;
        r.frac_den = 0u;
        r.is_whole_ticks = false;
        r.duration_is_lossless_ns = false;
        return r;
    }
    r.duration = Duration::from_nanos(ns);

    r.status = TimeIntervalStatus::Ok;
    return r;
}

// Bool-returning convenience form for call sites that only want the Duration.
//
// M0.1 (R4) WARNING: this is the *projected* form.  It is right for JSON,
// logs and the public API, and wrong as a ToF input unless the interval's
// `duration_is_lossless_ns` is true.  The ToF core must use
// `timestamp_relative_interval()`.
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

// ===========================================================================
// RelativeTickInterval -- the ONLY interval type the ToF core may consume
// ===========================================================================
//
// M0.1 (R4).  The review's finding was that a report telling M1 "use ticks
// when the exact flag is set" is unsafe, because the flag itself was wrong.
// So the safe form is made structurally safe instead: `RelativeTickInterval`
// has NO nanosecond field at all.  There is nothing lossy to reach for, no
// predicate to get wrong, and no way for a future caller to read a truncated
// projection into a distance.
//
// Everything here is the exact rational tick value of the interval, and
// `tick_rate_hz` is the domain's.  Cross-domain conversion is still M1's job
// and is only allowed between two of these that have already been validated.
struct RelativeTickInterval {
    bool valid = false;

    ClockDomain domain;
    TimestampMarker later_marker = TimestampMarker::RmarkerRx;
    TimestampMarker earlier_marker = TimestampMarker::RmarkerTx;

    int64_t ticks = 0;    // >= 0 whole device ticks
    int32_t frac_num = 0; // 0 when exact, else 0 < num < den <= 32767
    uint32_t frac_den = 0;
    bool wrapped = false;

    bool is_valid() const { return valid; }

    // R4 (a): whole number of device ticks, no sub-tick fraction.
    bool is_whole_ticks() const { return valid && frac_num == 0; }

    // R4 (b): would an integer-nanosecond `Duration` be a lossless record of
    // this value?  The ToF core does not need the answer -- it must use the
    // tick fields regardless -- but callers that must emit SI time need it in
    // order to label the projection honestly.
    bool ns_projection_is_lossless() const
    {
        return valid && detail::ts_tick_ns_projection_is_lossless(
                            domain.tick_rate_hz, ticks, frac_num, frac_den);
    }

    // EXACT: the interval as a rational number of TICKS,
    //     value == out_num / out_den,
    // from which seconds follow as value/tick_rate_hz.  This is the form M1's
    // SS clock-ratio and DS non-symmetric formulas should consume, so that no
    // floating point enters before the final division.  Returns false only on
    // an int64 overflow of `ticks*den + num` (never on a valid, small
    // interval); it is refused, never rounded.
    bool exact_ratio(int64_t& out_num, int64_t& out_den) const
    {
        if (!valid)
            return false;
        if (frac_den == 0u) {
            out_num = ticks;
            out_den = 1;
            return true;
        }
        const int64_t den = static_cast<int64_t>(frac_den);
        if (ticks > (std::numeric_limits<int64_t>::max() - frac_num) / den)
            return false;
        out_num = ticks * den + frac_num;
        out_den = den;
        return true;
    }

    // The sub-tick fraction as a double in [0, 1).  CONVENIENCE ONLY: for
    // frac_den > 2^53 a double cannot represent every rational, so the ToF
    // math must use `exact_ratio()` rather than this.
    double tick_fraction() const
    {
        if (!valid || frac_den == 0u)
            return 0.0;
        return static_cast<double>(frac_num) / static_cast<double>(frac_den);
    }

    // Seconds as a double.  DISPLAY / LOG / ORACLE COMPARISON ONLY: this is the
    // projected number, subject to the truncating nanosecond rule and to
    // double rounding, and it is NOT what a distance should be computed from.
    double seconds() const
    {
        if (!valid)
            return 0.0;
        double t = static_cast<double>(ticks);
        if (frac_den != 0u)
            t += tick_fraction();
        return t / domain.tick_rate_hz;
    }
};

// Project a validated interval into the exact, projection-free form.  Runs
// exactly the same gate sequence as `timestamp_interval()` (domain, epoch,
// marker pair, correction chain, wrap resolution, exact sub-tick combination),
// so it cannot be more permissive than the interval it is derived from.
inline bool timestamp_relative_interval(const Timestamp& later,
                                        const Timestamp& earlier,
                                        RelativeTickInterval& out,
                                        TimeIntervalStatus& status)
{
    const TimeInterval ti = timestamp_interval(later, earlier);
    status = ti.status;
    if (ti.status != TimeIntervalStatus::Ok)
        return false;

    RelativeTickInterval r;
    r.valid = true;
    r.domain = ti.domain;
    r.later_marker = ti.later_marker;
    r.earlier_marker = ti.earlier_marker;
    r.ticks = ti.ticks;
    r.frac_num = ti.frac_num;
    r.frac_den = ti.frac_den;
    r.wrapped = ti.wrapped;
    out = r;
    return true;
}

inline bool timestamp_relative_interval(const Timestamp& later,
                                        const Timestamp& earlier,
                                        RelativeTickInterval& out)
{
    TimeIntervalStatus ignored = TimeIntervalStatus::Ok;
    return timestamp_relative_interval(later, earlier, out, ignored);
}

// An explicit SI projection of the exact form, for the API/JSON boundary.  The
// caller is told whether that projection lost anything, so a truncated integer
// can never be mistaken for a measurement.
inline bool relative_interval_to_duration(const RelativeTickInterval& ri,
                                          Duration& out,
                                          TimeIntervalStatus& status)
{
    if (!ri.valid) {
        status = TimeIntervalStatus::InvalidTimestamp;
        return false;
    }
    int64_t ns = 0;
    if (!detail::ts_project_seconds_to_nanos_trunc(ri.seconds(), ns)) {
        status = TimeIntervalStatus::DurationOutOfRange;
        return false;
    }
    out = Duration::from_nanos(ns);
    status = TimeIntervalStatus::Ok;
    return true;
}

// ===========================================================================
// Ordering (M0.1 / R5)
// ===========================================================================
//
// The half-period rule and what it actually rests on
// -------------------------------------------------
//
// A wrapping counter stores `ticks mod 2^bits`.  Given only two readings, the
// forward distance from `a` to `b` is known only modulo the period: it is
// `d`, or `d + P`, or `d + 2P`, ...  If the caller can state an upper bound B
// on the true forward distance, then `d` is the unique candidate in (0, B)
// provided B <= P.  The textbook choice B = P/2 ("half period") needs NO
// extra information at all: it is the largest bound that is guaranteed sound
// for arbitrary serial numbers, and it makes the comparison antisymmetric,
// because the two directions sum to exactly P and so at most one of them can
// be < P/2.
//
// Consequences, all of which this header now enforces:
//
//   * EXACTLY P/2 IS AMBIGUOUS.  At d == P/2 the forward and backward
//     distances are equal, so nothing in the two numbers says which one a and
//     b are.  M0 accepted it (`d > P/2` refused only what is strictly greater)
//     and so reported "a precedes b" AND "b precedes a" for the same pair.
//     The rule is now strict: `d < P/2`, so d == P/2 is REFUSED in both
//     directions.
//   * HALF PERIOD IS A STRATEGY, NOT A HARDWARE LAW.  It is the right default
//     only when no additional history exists.  A TWR exchange does have such a
//     bound: an SS/DS exchange at 499.2e6*128 UUS is bounded by the reply
//     delay plus a timeout, a few hundred microseconds at most, i.e. far below
//     2^39 ticks on a 40-bit DW field.  A caller that knows its real budget
//     passes it in a `TimestampOrderBudget`, which is both stricter (it refuses
//     stale or spoofed readings earlier) and honest about what it assumes.
//   * A REQUESTED BUDGET IS CLAMPED TO P/2.  Asking for more than P/2 would
//     let both directions resolve, which is exactly the R5 defect; the
//     request is therefore reduced, not honoured, and the effective window is
//     readable through `timestamp_order_window_ticks()`.
//   * THIS DOES NOT APPLY TO INTERVALS.  `raw_tick_delta()` still accepts
//     d == P/2, and that is not an inconsistency: `timestamp_interval()` is
//     handed an explicit (later, earlier) pair, so the SIGN is supplied by the
//     caller and only the magnitude is derived, and at d == P/2 the magnitude
//     is unique.  It is only ORDERING -- deriving which of two numbers came
//     first -- that has no such external information, and that is where the
//     strict bound applies.
//
// WHICH ENTRY POINT MAY THE ToF CORE USE?
//
//   `timestamp_compare()`      YES.  Exact sub-tick ordering, budget-checked,
//                              three-valued, and it can return "no order
//                              exists" instead of guessing.
//   `timestamp_precedes()`     NO for anything that decides a distance, a
//                              deadline or a frame field.  It is a
//                              whole-tick, fraction-agnostic convenience kept
//                              for adapter bookkeeping and tests; M0 already
//                              documented that it ignores the sub-tick
//                              fraction, and this header keeps that
//                              restriction explicit.

// Tri-state ordering.  `Indeterminate` is the "does not exist" answer: it is
// never reported as an order, and `timestamp_compare()` returns false whenever
// it writes it.
enum class TimestampOrder : uint8_t {
    Indeterminate = 0, // no definite order: different timeline, invalid input,
                       // or the relation is ambiguous under the budget
    Equal = 1,         // the same instant
    Earlier = 2,       // `a` is strictly before `b`
    Later = 3          // `a` is strictly after `b`
};

inline const char* timestamp_order_to_string(TimestampOrder o)
{
    switch (o) {
    case TimestampOrder::Indeterminate:
        return "indeterminate";
    case TimestampOrder::Equal:
        return "equal";
    case TimestampOrder::Earlier:
        return "earlier";
    case TimestampOrder::Later:
        return "later";
    }
    return "invalid";
}

// Declared EXCLUSIVE upper bound on the forward distance of any ordering
// relation, in WHOLE TICKS of the domain.  "Exclusive" and "whole ticks" are
// both deliberate:
//
//   * whole ticks, because the resolvability test is then
//     `floor(distance) < budget`.  A sub-tick fraction can therefore never
//     change the outcome, and no cross-multiplication is needed.  Callers that
//     want a fractional bound express it by flooring it themselves.
//   * exclusive, so that the budget has EXACTLY the same meaning as the half
//     period it defaults to: a relation resolves only when its forward
//     distance is strictly below the window.  `explicit_ticks(4000)` therefore
//     resolves up to 3999 ticks and refuses 4000, matching `4000 < 4000` being
//     false.  Using `<=` here would make the budget and the half period mean
//     different things for the same number.
//
// Use 0 (the default) to mean "the domain's half period", i.e. assume nothing
// beyond what the counter width guarantees.
struct TimestampOrderBudget {
    int64_t max_forward_ticks = 0;

    bool is_valid() const { return max_forward_ticks >= 0; }

    static TimestampOrderBudget half_period() { return TimestampOrderBudget{0}; }
    static TimestampOrderBudget explicit_ticks(int64_t ticks)
    {
        return TimestampOrderBudget{ticks < 0 ? 0 : ticks};
    }
};

// The window actually used, after clamping an explicit budget to the half
// period.  False for a negative budget.  A no-wrap (monotonic) domain has no
// modular ambiguity, so the window is unbounded there unless the caller
// declared one -- in which case the declaration is still enforced, because a
// caller that knows its exchange cannot be longer than N ticks wants a
// refusal, not a verdict, when it is.
inline bool timestamp_order_window_ticks(const ClockDomain& d,
                                          const TimestampOrderBudget& budget,
                                          int64_t& out_window)
{
    out_window = 0;
    if (!clock_domain_is_valid(d) || !budget.is_valid())
        return false;
    const uint64_t p = clock_domain_wrap_period(d);
    const int64_t half = (p == 0u) ? 0
                                   : static_cast<int64_t>(p / 2u);
    if (budget.max_forward_ticks == 0) {
        out_window = (p == 0u) ? std::numeric_limits<int64_t>::max() : half;
        return true;
    }
    if (p == 0u) {
        out_window = budget.max_forward_ticks;
        return true;
    }
    out_window = budget.max_forward_ticks < half ? budget.max_forward_ticks : half;
    return true;
}

// EXACT comparison of two normalized sub-tick fractions; den == 0 means 0/1.
// Returns -1, 0 or +1.  num < 32767 and den <= 32767, so num*den < 2^30 and
// int64 cannot overflow.
inline int timestamp_fraction_compare(int32_t a_num, uint32_t a_den,
                                      int32_t b_num, uint32_t b_den)
{
    if (a_num == 0)
        return b_num == 0 ? 0 : -1;
    if (b_num == 0)
        return 1;
    const int64_t l = static_cast<int64_t>(a_num) * static_cast<int64_t>(b_den);
    const int64_t r = static_cast<int64_t>(b_num) * static_cast<int64_t>(a_den);
    if (l < r)
        return -1;
    if (l > r)
        return 1;
    return 0;
}

// The denominator a DIFFERENCE of two sub-tick fractions may need.
//
// `Timestamp::frac_den` is capped at kMaxTimestampFractionDenominator (32767)
// so that any single stored value stays small.  The difference of two such
// values has denominator lcm(den_a, den_b), which can be as large as
// 32767*32766 ~ 1.07e9 even when both inputs are perfectly legal and reduced.
// Capping a difference at 32767 would therefore refuse ordinary inputs such
// as 1/32767 - (-1/32766), so the modular-distance helper carries its own,
// wider, exact bound.  It is deliberately a DIFFERENT constant from the
// timestamp one, and it is not a stored-timestamp constraint.
inline constexpr uint32_t kMaxFractionDifferenceDenominator = 32767u * 32767u;

// Exact forward distance from `a` to `b` on a wrapping counter, in ticks plus
// a normalized sub-tick fraction, always in [0, period).
//
//   m = (b.ticks - a.ticks) mod P            integer part from the tick grid
//   D = m + (b.frac - a.frac)                exact forward distance
//
// When b's fraction is below a's, `m + (b.frac - a.frac)` lands in (m-1, m),
// so the whole-tick part DECREASES by one and the fraction wraps up to
// `1 - a.frac`.  Getting that sign wrong is not cosmetic: 255.75 -> 0.25 on an
// 8-bit counter is a distance of 1/2 tick, not of 2 1/4 ticks, and the two
// answers sit on opposite sides of a budget boundary.
struct ModularTickDistance {
    bool ok = false;
    int64_t ticks = 0;     // in [0, period-1]
    int32_t frac_num = 0;  // 0, or 0 < num < den <= 32767
    uint32_t frac_den = 0;
    bool wrapped = false; // the integer tick difference itself crossed the wrap
};

inline ModularTickDistance modular_tick_distance(const Timestamp& a, const Timestamp& b)
{
    ModularTickDistance r;
    if (!timestamp_is_self_consistent(a) || !timestamp_is_self_consistent(b))
        return r;
    if (!clock_domain_is_comparable(a.domain, b.domain))
        return r;
    const uint64_t p = clock_domain_wrap_period(a.domain);
    if (p == 0u)
        return r; // a monotonic counter has no modular distance

    const uint64_t from = static_cast<uint64_t>(a.ticks);
    const uint64_t to = static_cast<uint64_t>(b.ticks);
    r.wrapped = to < from;
    r.ticks = r.wrapped ? (to + p - from) : (to - from);

    // Exact sub-tick part: b.frac - a.frac, as a common-denominator rational.
    // den == 0 means 0/1.  den_a*den_b <= 32767^2 fits in uint32, and the
    // numerator is bounded by the same, so int64 cannot overflow.
    const uint32_t ad = a.frac_den == 0u ? 1u : a.frac_den;
    const uint32_t bd = b.frac_den == 0u ? 1u : b.frac_den;
    if (ad > kMaxFractionDifferenceDenominator / bd)
        return r; // unreachable for legal Timestamps; refused, never rounded
    const uint32_t den = ad * bd;
    const int64_t an = static_cast<int64_t>(a.frac_num) * static_cast<int64_t>(bd);
    const int64_t bn = static_cast<int64_t>(b.frac_num) * static_cast<int64_t>(ad);
    int64_t num = bn - an; // in (-den, den)
    if (num < 0) {
        // A fraction below `a`'s makes the distance one whole tick SHORTER
        // plus the wrapped-up remainder.  Skipping this borrow is the M0 bug
        // that made 255.25 -> 0.75 look like a full tick instead of 1.5, and
        // the two answers sit on opposite sides of a budget boundary.
        // When the integer part is already 0, the downward borrow would go
        // negative: a whole period elapsed minus (a.frac - b.frac), which is
        // outside every budget by construction.
        if (r.ticks == 0) {
            r.ticks = static_cast<int64_t>(p) - 1;
        } else {
            --r.ticks;
        }
        num += den;
    }
    const int64_t g = detail::ts_gcd(num, den);
    num /= g;
    if (num == 0) {
        r.frac_num = 0;
        r.frac_den = 0u;
    } else {
        r.frac_num = static_cast<int32_t>(num);
        r.frac_den = static_cast<uint32_t>(den / g);
    }
    r.ok = true;
    return r;
}

// The R5 ordering entry point.  `out` is ALWAYS written, including on failure,
// so a caller that ignores the return value still cannot read an invented
// order: the failure value is `Indeterminate`.
//
// Returns false and writes `Indeterminate` when
//   - either timestamp is structurally invalid;
//   - the two are not on one timeline (different name / rate / width / epoch);
//   - the budget is invalid; or
//   - the relation is not resolvable: the forward distance is not strictly
//     below the effective window.  Exactly half a period is in this class, in
//     BOTH directions, which is the R5 fix.
inline bool timestamp_compare(const Timestamp& a,
                              const Timestamp& b,
                              TimestampOrder& out,
                              const TimestampOrderBudget& budget = TimestampOrderBudget())
{
    out = TimestampOrder::Indeterminate;
    if (!timestamp_is_self_consistent(a) || !timestamp_is_self_consistent(b))
        return false;
    if (!clock_domain_is_comparable(a.domain, b.domain))
        return false;
    int64_t window = 0;
    if (!timestamp_order_window_ticks(a.domain, budget, window))
        return false;

    const uint64_t p = clock_domain_wrap_period(a.domain);
    if (p == 0u) {
        // Monotonic counter: a total order, no ambiguity, but a declared
        // budget is still enforced as a sanity bound.
        if (a.ticks == b.ticks) {
            const int c =
                timestamp_fraction_compare(a.frac_num, a.frac_den, b.frac_num, b.frac_den);
            out = (c < 0) ? TimestampOrder::Earlier
                          : (c > 0 ? TimestampOrder::Later : TimestampOrder::Equal);
            return true;
        }
        const int64_t dist = a.ticks < b.ticks ? b.ticks - a.ticks : a.ticks - b.ticks;
        if (dist >= window)
            return false;
        out = a.ticks < b.ticks ? TimestampOrder::Earlier : TimestampOrder::Later;
        return true;
    }

    const ModularTickDistance ab = modular_tick_distance(a, b);
    if (!ab.ok)
        return false;
    if (ab.ticks == 0 && ab.frac_num == 0) {
        out = TimestampOrder::Equal;
        return true;
    }
    // STRICT: `>= window` is refused, so the exactly-half-period pair is
    // Indeterminate in both directions instead of "earlier" in both.
    if (ab.ticks >= window)
        return false;
    out = TimestampOrder::Earlier;
    return true;
}

// Absolute (non-modular) ordering, for a no-wrap counter where a total order
// always exists.  Refuses a wrapping domain rather than pretending the wrap
// does not matter.
inline bool timestamp_compare_absolute(const Timestamp& a,
                                       const Timestamp& b,
                                       TimestampOrder& out)
{
    out = TimestampOrder::Indeterminate;
    if (!timestamp_is_self_consistent(a) || !timestamp_is_self_consistent(b))
        return false;
    if (!clock_domain_is_comparable(a.domain, b.domain))
        return false;
    if (clock_domain_wrap_period(a.domain) != 0u)
        return false;
    if (a.ticks == b.ticks) {
        const int c = timestamp_fraction_compare(a.frac_num, a.frac_den, b.frac_num, b.frac_den);
        out = (c < 0) ? TimestampOrder::Earlier
                      : (c > 0 ? TimestampOrder::Later : TimestampOrder::Equal);
        return true;
    }
    out = a.ticks < b.ticks ? TimestampOrder::Earlier : TimestampOrder::Later;
    return true;
}

// Whole-tick, fraction-agnostic modular ordering, kept for adapter
// bookkeeping and for the M0 test suite.  Same domain/epoch gate as
// `timestamp_interval()`, and M0.1 tightened the boundary: the forward
// distance must be STRICTLY below half the period, so the exactly-half-period
// pair is refused in both directions instead of claiming "earlier" twice.
//
// Returns false (leaving `out` untouched, as documented since M0) when the two
// are not on one timeline, when either is structurally invalid, or when the
// relation is ambiguous modulo the wrap.  The sub-tick fraction is NOT
// consulted: two timestamps less than a tick apart compare equal here.
//
// M1 MUST NOT use this to decide a distance, a deadline or a frame field; use
// `timestamp_compare()`.
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
        // R5: STRICT.  `dist == p/2` is the ambiguous case and is refused here
        // exactly as it is in `timestamp_compare()`.
        if (dist >= p / 2u)
            return false;
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
    char buf[1200];
    std::snprintf(
        buf, sizeof(buf),
        "{\"status\":\"%s\",\"ticks\":%lld,\"frac_num\":%d,\"frac_den\":%u,"
        "\"wrapped\":%s,\"duration_ns\":%lld,\"whole_ticks\":%s,"
        "\"duration_ns_is_lossless\":%s,\"ns_projection_rounding\":\"truncate_toward_zero\","
        "\"clock_domain\":%s,\"later_marker\":\"%s\","
        "\"earlier_marker\":\"%s\"}",
        time_interval_status_to_string(ti.status),
        static_cast<long long>(ti.ticks), static_cast<int>(ti.frac_num),
        static_cast<unsigned>(ti.frac_den), ti.wrapped ? "true" : "false",
        static_cast<long long>(ti.duration.nanos()),
        ti.is_whole_ticks ? "true" : "false",
        ti.duration_is_lossless_ns ? "true" : "false",
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
