/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * M1-A: the SS-TWR / DS-TWR ToF mathematics, and nothing else.
 *
 * ---------------------------------------------------------------------------
 * What this file is, and what it deliberately is not
 * ---------------------------------------------------------------------------
 *
 * This is a HEADER-ONLY, UHD-free, GNU-Radio-free, config-free core.  It
 * consumes exactly two things:
 *
 *   * an `AdmittedRangingInterval` -- an interval that already passed the
 *     M0.1 ranging gate (`admit_ranging_interval()` in
 *     uwb_twr_tof_input.h).  The formulas do NOT admit anything themselves
 *     and never see a bare `Timestamp`; "which instants may measure a
 *     distance" is decided once, at the gate, and is not re-litigated here.
 *   * an explicit `ClockRatio` -- how many A ticks one B tick is worth.
 *
 * It does NOT contain a protocol state machine, a fake endpoint, a GNU Radio
 * block, a UHD adapter, or a CFO/SFO ESTIMATOR.  M1-A only CONSUMES a clock
 * ratio that a caller (M2) constructs; how to estimate one is a later
 * milestone.  See docs/twr/OpenCode开发指示_M1-A_ToF数学.md.
 *
 * ---------------------------------------------------------------------------
 * The formulas (REQ-PROTO-02 / 03)
 * ---------------------------------------------------------------------------
 *
 * `k = fA / fB` is A ticks per B tick.  Its direction is fixed: it converts a
 * B-domain interval INTO the A domain.  With
 *
 *   RA = t4A - t1A   (A domain)        DB = t3B - t2B   (B domain)
 *   DA = t5A - t4A   (A domain)        RB = t6B - t3B   (B domain)
 *
 *   SS:  ToF_A = (RA - k*DB) / 2
 *   DS:  ToF_A = (RA*k*RB - DA*k*DB) / (RA*k*RB + DA*k*DB + ... )
 *        i.e. (RA*k*RB - DA*k*DB) / (RA + k*RB + DA + k*DB)
 *
 * THE COMMON UNIT FOR DS IS **A TICKS**.  REQ-PROTO-03 forbids plugging the
 * four raw intervals into the formula while they live in two different
 * devices' tick units: RB and DB are first multiplied by k, and only then is
 * the formula applied.  The result is therefore in A ticks for both SS and
 * DS.  (Expressed in seconds it is the SAME physical ToF; A ticks is simply
 * the unit the exact arithmetic is done in, so that no floating point enters
 * before the final display division.)
 *
 * Why the identities come out exactly:
 *
 *   RA = 2*tau + dB, DB = dB/k, DA = dA, RB = (2*tau + dA)/k, so
 *   SS: (RA - k*DB)/2 = (2tau + dB - dB)/2 = tau.
 *   DS: with kRB = 2tau + dA and kDB = dB,
 *       (RA*kRB - DA*kDB) = (2tau+dB)(2tau+dA) - dA*dB = 2tau(2tau+dA+dB)
 *       (RA+kRB+DA+kDB)   = (2tau+dB)+(2tau+dA)+dA+dB = 2(2tau+dA+dB)
 *       quotient = tau.
 *
 * ---------------------------------------------------------------------------
 * Exactness rules (REQ-TIME-01 / 04, and the R4 review defect)
 * ---------------------------------------------------------------------------
 *
 *   * The arithmetic is exact rational over device ticks.  NO nanosecond
 *     projection, NO `double` seconds, and NO `relative_interval_to_duration()`
 *     ever appears as an intermediate: an interval that is 15.645 ps wide is
 *     not representable in integer nanoseconds, so projecting first would
 *     discard the very precision the formula exists to keep.  A `double` is
 *     produced ONCE, in the `display_seconds` field, for a human or for the
 *     oracle comparison, and is labelled as a projection.
 *   * `RelativeTickInterval::exact_ratio()` is the only interval accessor
 *     used.  `tick_fraction()` / `seconds()` are forbidden inputs.
 *   * All intermediate products go through 128-bit integers and every
 *     multiply/add is overflow-checked; an overflow is a REPORTED failure
 *     (`TofStatus::Overflow`), never a silent wrap and never a fallback to
 *     double.
 *   * A negative ToF is RETAINED with its sign.  Clamping it to 0 and calling
 *     that "success" is exactly the failure REQ-ERR-01 forbids.
 *
 * ---------------------------------------------------------------------------
 * Enum handling (N07)
 * ---------------------------------------------------------------------------
 *
 * Every enum this file introduces has a `xxx_is_known()` written as a switch
 * with NO `default`, so adding an enumerator makes `-Wswitch` fire rather
 * than silently widening a domain.  Domain tests run BEFORE the switch that
 * consumes the value.  An out-of-domain value is refused, never reasoned
 * about.
 */

#ifndef INCLUDED_GNURADIO_UWB_UWB_TWR_MATH_H
#define INCLUDED_GNURADIO_UWB_UWB_TWR_MATH_H

#include <gnuradio/uwb/uwb_twr_tof_input.h>

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <string>

namespace gr {
namespace uwb {
namespace twr {

// ===========================================================================
// ClockRatioSource -- where k came from
// ===========================================================================
//
// REQ-API-01 wants the provenance of a clock ratio recorded, and the review
// wants "same clock" to be an explicit statement rather than a default.  A
// single X410 with two channels on one physical clock is `NominalSameClock`
// (k == 1); two independent devices are `NominalRateRatio` (k from the two
// nominal rates) or, later, `Calibrated` / `Estimated`.
enum class ClockRatioSource : uint8_t {
    NominalSameClock = 0, // A and B are one physical clock; k == 1 BY STATEMENT
    NominalRateRatio = 1, // k = fA/fB from the two nominal rates
    Calibrated = 2,       // k from a measured/declared calibration record
    Estimated = 3         // k from a runtime estimator (M2)
};

inline const char* clock_ratio_source_to_string(ClockRatioSource s)
{
    switch (s) {
    case ClockRatioSource::NominalSameClock:
        return "nominal_same_clock";
    case ClockRatioSource::NominalRateRatio:
        return "nominal_rate_ratio";
    case ClockRatioSource::Calibrated:
        return "calibrated";
    case ClockRatioSource::Estimated:
        return "estimated";
    }
    return "invalid";
}

// No `default`: adding an enumerator must be a compile-time prompt, not a
// silent acceptance (N07).  An out-of-domain value is NOT a ratio source.
inline bool clock_ratio_source_is_known(ClockRatioSource s)
{
    switch (s) {
    case ClockRatioSource::NominalSameClock:
    case ClockRatioSource::NominalRateRatio:
    case ClockRatioSource::Calibrated:
    case ClockRatioSource::Estimated:
        return true;
    }
    return false;
}

inline bool clock_ratio_source_from_string(const std::string& s, ClockRatioSource& out)
{
    for (int i = 0; i <= static_cast<int>(ClockRatioSource::Estimated); ++i) {
        const ClockRatioSource c = static_cast<ClockRatioSource>(i);
        if (s == clock_ratio_source_to_string(c)) {
            out = c;
            return true;
        }
    }
    return false;
}

// ===========================================================================
// TofStatus -- the math layer's OWN failure vocabulary
// ===========================================================================
//
// "Why did the formula refuse" is a different question from "what is the
// exchange's terminal state", and collapsing the two would lose the reason a
// caller needs.  Each value maps to an `ExchangeStatus` (see
// `tof_status_to_exchange_status()`), but the mapping is one-way and lossy.
//
// Numeric values are append-only schema (REQ-OUT-01).
enum class TofStatus : uint8_t {
    Ok = 0,

    InvalidInput = 1,             // an interval object is not structurally usable
    InvalidClockDomain = 2,       // a required ClockDomain is malformed

    ClockRatioMissing = 3,        // no ratio supplied where one is REQUIRED
    ClockRatioInvalid = 4,        // source unknown / k <= 0 / den <= 0 / bad window
    ClockRatioNotValidAtTime = 5, // the interval lies outside k's validity window
    ClockRatioDomainMismatch = 6, // k's A/B identity != the interval's domain
    ClockRatioEpochMismatch = 7,  // same identity, different epoch

    NegativeTof = 8,              // signed value RETAINED, exchange not Ok
    ZeroDenominator = 9,          // DS denominator <= 0
    Overflow = 10,                // an intermediate left the exact 128-bit range

    InvalidResult = 11            // internal invariant (should not occur)
};

inline const char* tof_status_to_string(TofStatus s)
{
    switch (s) {
    case TofStatus::Ok:
        return "ok";
    case TofStatus::InvalidInput:
        return "invalid_input";
    case TofStatus::InvalidClockDomain:
        return "invalid_clock_domain";
    case TofStatus::ClockRatioMissing:
        return "clock_ratio_missing";
    case TofStatus::ClockRatioInvalid:
        return "clock_ratio_invalid";
    case TofStatus::ClockRatioNotValidAtTime:
        return "clock_ratio_not_valid_at_time";
    case TofStatus::ClockRatioDomainMismatch:
        return "clock_ratio_domain_mismatch";
    case TofStatus::ClockRatioEpochMismatch:
        return "clock_ratio_epoch_mismatch";
    case TofStatus::NegativeTof:
        return "negative_tof";
    case TofStatus::ZeroDenominator:
        return "zero_denominator";
    case TofStatus::Overflow:
        return "overflow";
    case TofStatus::InvalidResult:
        return "invalid_result";
    }
    return "invalid";
}

// No `default` (N07).  A value the enum does not have is not a status.
inline bool tof_status_is_known(TofStatus s)
{
    switch (s) {
    case TofStatus::Ok:
    case TofStatus::InvalidInput:
    case TofStatus::InvalidClockDomain:
    case TofStatus::ClockRatioMissing:
    case TofStatus::ClockRatioInvalid:
    case TofStatus::ClockRatioNotValidAtTime:
    case TofStatus::ClockRatioDomainMismatch:
    case TofStatus::ClockRatioEpochMismatch:
    case TofStatus::NegativeTof:
    case TofStatus::ZeroDenominator:
    case TofStatus::Overflow:
    case TofStatus::InvalidResult:
        return true;
    }
    return false;
}

inline bool tof_status_from_string(const std::string& s, TofStatus& out)
{
    for (int i = 0; i <= static_cast<int>(TofStatus::InvalidResult); ++i) {
        const TofStatus c = static_cast<TofStatus>(i);
        if (s == tof_status_to_string(c)) {
            out = c;
            return true;
        }
    }
    return false;
}

// Which terminal `ExchangeStatus` a math failure becomes (REQ-ERR-01).
//
// The domain test runs FIRST so an out-of-domain value cannot pick a mapping
// by falling through a switch (N07); it is reported as InternalError.
inline ExchangeStatus tof_status_to_exchange_status(TofStatus s)
{
    if (!tof_status_is_known(s))
        return ExchangeStatus::InternalError;
    switch (s) {
    case TofStatus::Ok:
        return ExchangeStatus::Ok;
    case TofStatus::InvalidInput:
    case TofStatus::InvalidClockDomain:
    case TofStatus::ClockRatioDomainMismatch:
    case TofStatus::ClockRatioEpochMismatch:
    case TofStatus::ZeroDenominator:
        return ExchangeStatus::InvalidTimeDomain;
    case TofStatus::ClockRatioMissing:
    case TofStatus::ClockRatioInvalid:
    case TofStatus::ClockRatioNotValidAtTime:
        return ExchangeStatus::ClockEstimateInvalid;
    case TofStatus::NegativeTof:
        return ExchangeStatus::NegativeTof;
    case TofStatus::Overflow:
    case TofStatus::InvalidResult:
        return ExchangeStatus::InternalError;
    }
    return ExchangeStatus::InternalError;
}

// ===========================================================================
// ComputedAt -- which endpoint the baseline result is defined at
// ===========================================================================
//
// REQ-PROTO-05: the SS baseline result is available at A, the DS baseline at
// B.  That is recorded EXPLICITLY so a reader cannot confuse "the formula was
// evaluated at A" with "the units happen to be A ticks" -- for DS the units
// are A ticks by construction while the result belongs to B.
enum class ComputedAt : uint8_t {
    InitiatorA = 0,
    ResponderB = 1
};

inline const char* computed_at_to_string(ComputedAt c)
{
    switch (c) {
    case ComputedAt::InitiatorA:
        return "initiator_a";
    case ComputedAt::ResponderB:
        return "responder_b";
    }
    return "invalid";
}

inline bool computed_at_is_known(ComputedAt c)
{
    switch (c) {
    case ComputedAt::InitiatorA:
    case ComputedAt::ResponderB:
        return true;
    }
    return false;
}

// ===========================================================================
// TofRationalTicks -- a signed exact rational number of ticks
// ===========================================================================
//
// `num/den` is in lowest terms with `den > 0`, so the SIGN lives in `num`
// alone.  This is the exact ToF and the exact intervals; it is never a
// floating-point approximation.
struct TofRationalTicks {
    int64_t num = 0;
    int64_t den = 1; // > 0 when `valid`
    bool valid = false;

    bool negative() const { return valid && num < 0; }
    bool is_zero() const { return valid && num == 0; }

    // DISPLAY / LOG / ORACLE COMPARISON ONLY.  A double projection, taken once
    // at the end, of (num/den)/tick_rate_hz.  It is NOT an input to anything.
    double to_seconds(double tick_rate_hz) const
    {
        if (!valid || !(tick_rate_hz > 0.0))
            return 0.0;
        return (static_cast<double>(num) / static_cast<double>(den)) / tick_rate_hz;
    }

    std::string to_string() const
    {
        if (!valid)
            return "invalid";
        char buf[64];
        std::snprintf(buf, sizeof(buf), "%lld/%lld", static_cast<long long>(num),
                      static_cast<long long>(den));
        return std::string(buf);
    }
};

// Exact rational equality, cross-multiplied (no floating point, no reduction
// requirement on the caller).
inline bool tof_rational_equals(const TofRationalTicks& a, const TofRationalTicks& b)
{
    if (!a.valid || !b.valid)
        return false;
    using i128 = __int128;
    return static_cast<i128>(a.num) * static_cast<i128>(b.den) ==
           static_cast<i128>(b.num) * static_cast<i128>(a.den);
}

// ===========================================================================
// ClockRatio -- k = fA/fB, exactly, with provenance and a validity window
// ===========================================================================
//
// DIRECTION IS FIXED: `k` multiplies a B-domain interval to express it in A
// ticks.  `unity_same_clock()` is the ONLY way to say "one physical clock",
// and it must be called on purpose -- an independent-clock configuration may
// not silently default to k == 1 (SS-4).  A default-constructed `ClockRatio`
// is `provided() == false` and is refused as `ClockRatioMissing`.
//
// `k` is an exact positive rational.  A negative k is refused, never used to
// "flip" the formula; the direction is the caller's job to get right, and
// getting it wrong shows up as a refusal rather than a sign that cancels.
class ClockRatio {
public:
    ClockRatio()
        : provided_(false), k_num_(0), k_den_(1),
          source_(ClockRatioSource::NominalSameClock), has_window_(false),
          window_from_(0), window_until_(0), has_uncertainty_(false),
          uncertainty_ppm_num_(0), uncertainty_ppm_den_(1)
    {
    }

    // ---- factories -----------------------------------------------------
    //
    // All factories leave `out` untouched when they return false, so a failed
    // construction cannot leave a half-valid ratio behind.

    // A and B are the SAME physical clock (the single-X410 two-channel case).
    // k == 1 is a STATEMENT the caller makes, not a default.
    static bool unity_same_clock(const ClockDomain& a, ClockRatio& out)
    {
        if (!a.is_valid())
            return false;
        ClockRatio r;
        r.provided_ = true;
        r.domain_a_ = a;
        r.domain_b_ = a;
        r.k_num_ = 1;
        r.k_den_ = 1;
        r.source_ = ClockRatioSource::NominalSameClock;
        out = r;
        return true;
    }

    // k = fA/fB from two NOMINAL rates in Hz.  Integer Hz on purpose: a
    // nominal UWB rate is an exact integer (e.g. 63897600000), and accepting
    // a double here would import a rounding error into the ratio.
    static bool from_nominal_rates(const ClockDomain& a, const ClockDomain& b,
                                   int64_t fA_hz, int64_t fB_hz, ClockRatio& out)
    {
        if (!a.is_valid() || !b.is_valid())
            return false;
        if (fA_hz <= 0 || fB_hz <= 0)
            return false;
        // If the two domains are literally the same counter, the rates must
        // agree with that: a "rate ratio" over one clock is a contradiction.
        if (clock_domain_same_identity(a, b) && fA_hz != fB_hz)
            return false;
        ClockRatio r;
        r.provided_ = true;
        r.domain_a_ = a;
        r.domain_b_ = b;
        r.k_num_ = fA_hz;
        r.k_den_ = fB_hz;
        r.source_ = ClockRatioSource::NominalRateRatio;
        r.reduce();
        out = r;
        return true;
    }

    // Fully explicit.  `source` must be a value the enum has (N07): an
    // out-of-domain source is refused here, not carried into the formula.
    static bool make(const ClockDomain& a, const ClockDomain& b, int64_t k_num,
                     int64_t k_den, ClockRatioSource source, ClockRatio& out)
    {
        if (!a.is_valid() || !b.is_valid())
            return false;
        if (k_num <= 0 || k_den <= 0)
            return false;
        if (!clock_ratio_source_is_known(source))
            return false;
        // `NominalSameClock` is a STATEMENT that A and B are one physical
        // clock, so it requires one comparable domain AND k == 1.  It must
        // not be used to smuggle k=1 onto two different counters, nor a
        // non-unit ratio onto one.  `unity_same_clock()` is the intended
        // spelling.
        if (source == ClockRatioSource::NominalSameClock &&
            (!clock_domain_is_comparable(a, b) || k_num != k_den))
            return false;
        ClockRatio r;
        r.provided_ = true;
        r.domain_a_ = a;
        r.domain_b_ = b;
        r.k_num_ = k_num;
        r.k_den_ = k_den;
        r.source_ = source;
        r.reduce();
        out = r;
        return true;
    }

    // ---- optional validity window (A-domain ticks) ---------------------
    //
    // ABSENT BY DEFAULT, and absent means "no declared window" (valid
    // whenever used).  When PRESENT it must be well formed: `from <= until`.
    // The window is in A-domain ticks, because k is applied to B intervals but
    // validated against the A-domain interval the exchange runs on.
    bool set_validity_window(int64_t from_ticks, int64_t until_ticks)
    {
        if (from_ticks > until_ticks)
            return false;
        has_window_ = true;
        window_from_ = from_ticks;
        window_until_ = until_ticks;
        return true;
    }

    // ---- optional uncertainty, an exact rational in ppm -----------------
    //
    // ABSENT BY DEFAULT ("not declared").  When present: num >= 0, den > 0.
    // M1-A only CARRIES this; it does not propagate it into a result tolerance
    // (that needs an error model M1-A does not have).
    bool set_uncertainty_ppm(int64_t num, int64_t den)
    {
        if (den <= 0 || num < 0)
            return false;
        has_uncertainty_ = true;
        uncertainty_ppm_num_ = num;
        uncertainty_ppm_den_ = den;
        return true;
    }

    // ---- accessors -----------------------------------------------------
    bool provided() const { return provided_; }
    bool is_valid() const { return check() == TofStatus::Ok; }
    const ClockDomain& domain_a() const { return domain_a_; }
    const ClockDomain& domain_b() const { return domain_b_; }
    int64_t k_num() const { return k_num_; }
    int64_t k_den() const { return k_den_; }
    ClockRatioSource source() const { return source_; }

    bool has_validity_window() const { return has_window_; }
    int64_t valid_from_ticks() const { return window_from_; }
    int64_t valid_until_ticks() const { return window_until_; }

    bool has_uncertainty() const { return has_uncertainty_; }
    int64_t uncertainty_ppm_num() const { return uncertainty_ppm_num_; }
    int64_t uncertainty_ppm_den() const { return uncertainty_ppm_den_; }

    // Is `t` (an A-domain tick) inside the declared window?  Inclusive on both
    // ends; an absent window covers everything.
    bool covers_ticks(int64_t t) const
    {
        if (!has_window_)
            return true;
        return t >= window_from_ && t <= window_until_;
    }

    // ---- the ONE validity question -------------------------------------
    //
    // Returns `TofStatus::Ok` when the object is usable, otherwise the exact
    // reason.  Domain/identity checks against the intervals are NOT here --
    // they need the interval and live in `compute_*`.
    TofStatus check() const
    {
        if (!provided_)
            return TofStatus::ClockRatioMissing;
        // Domain test FIRST (N07): an unknown source must not reach the switch
        // that consumes it.
        if (!clock_ratio_source_is_known(source_))
            return TofStatus::ClockRatioInvalid;
        if (!domain_a_.is_valid() || !domain_b_.is_valid())
            return TofStatus::InvalidClockDomain;
        if (k_num_ <= 0 || k_den_ <= 0)
            return TofStatus::ClockRatioInvalid;
        if (has_window_ && window_from_ > window_until_)
            return TofStatus::ClockRatioInvalid;
        if (has_uncertainty_ && (uncertainty_ppm_den_ <= 0 || uncertainty_ppm_num_ < 0))
            return TofStatus::ClockRatioInvalid;
        return TofStatus::Ok;
    }

    std::string to_string() const
    {
        char buf[320];
        std::snprintf(buf, sizeof(buf),
                      "k=%lld/%lld src=%s A=%s B=%s window=%s uncertainty_ppm=%s",
                      static_cast<long long>(k_num_), static_cast<long long>(k_den_),
                      clock_ratio_source_to_string(source_),
                      domain_a_.to_string().c_str(), domain_b_.to_string().c_str(),
                      has_window_ ? "declared" : "absent",
                      has_uncertainty_ ? "declared" : "absent");
        return std::string(buf);
    }

private:
    // Reduce k to lowest terms so the later 128-bit products stay small.
    void reduce()
    {
        const int64_t g = detail_gcd64(k_num_, k_den_);
        if (g > 1) {
            k_num_ /= g;
            k_den_ /= g;
        }
    }

    static int64_t detail_gcd64(int64_t a, int64_t b)
    {
        if (a < 0)
            a = -a;
        if (b < 0)
            b = -b;
        while (b != 0) {
            const int64_t t = a % b;
            a = b;
            b = t;
        }
        return a == 0 ? 1 : a;
    }

    bool provided_;
    ClockDomain domain_a_;
    ClockDomain domain_b_;
    int64_t k_num_;
    int64_t k_den_;

    ClockRatioSource source_;

    bool has_window_;
    int64_t window_from_;
    int64_t window_until_;

    bool has_uncertainty_;
    int64_t uncertainty_ppm_num_;
    int64_t uncertainty_ppm_den_;
};

// ===========================================================================
// TofResult -- the complete, self-describing outcome
// ===========================================================================
//
// Carries the exact signed ToF, the exact intervals it combined, the clock
// ratio that was consumed, and a human-readable detail.  REQ-OUT-01 wants the
// inputs recorded, so the native-domain intervals are kept alongside the
// converted ones.
struct TofResult {
    bool ok = false;
    TofStatus status = TofStatus::Ok;
    // The math status mapped onto the exchange's terminal vocabulary.
    ExchangeStatus exchange_status = ExchangeStatus::Ok;

    Protocol protocol = Protocol::Ss;
    ComputedAt computed_at = ComputedAt::InitiatorA;

    // Exact SIGNED ToF, in `domain` ticks.
    TofRationalTicks tof;
    // The unit the ToF is expressed in: A for SS, and A for DS too (the four
    // intervals were converted to A ticks before the formula ran).
    ClockDomain domain;

    // The four intervals exactly as admitted, in their own native domains.
    TofRationalTicks ra_raw, rb_raw, da_raw, db_raw;
    // The same four converted into `domain` (A ticks): what the formula used.
    TofRationalTicks ra, rb, da, db;

    // The ratio actually consumed, by value (a later change to the caller's
    // object cannot reach this result).
    ClockRatio ratio;

    // DISPLAY / LOG / ORACLE COMPARISON ONLY.  `tof` projected to seconds once.
    double display_seconds = 0.0;

    // Optional derived distance, filled only by `tof_distance_m()`.
    bool distance_valid = false;
    double distance_m = 0.0;

    std::string detail;

    std::string to_string() const
    {
        char buf[512];
        std::snprintf(buf, sizeof(buf),
                      "ok=%s status=%s exchange=%s protocol=%s at=%s tof=%s ticks "
                      "@ %s display=%.12g s",
                      ok ? "true" : "false", tof_status_to_string(status),
                      exchange_status_to_string(exchange_status),
                      protocol_to_string(protocol), computed_at_to_string(computed_at),
                      tof.to_string().c_str(),
                      domain.is_valid() ? domain.name.c_str() : "(none)",
                      display_seconds);
        std::string s(buf);
        if (!detail.empty())
            s += " detail=" + detail;
        return s;
    }
};

// Vacuum speed of light.  Named on purpose: a distance is only ever a derived,
// optional field, and a test may inject a slower cable velocity instead.
inline constexpr double kPropagationSpeedVacuumMps = 299792458.0;

// ===========================================================================
// detail: exact 128-bit rational arithmetic
// ===========================================================================
//
// A rational is `n/d` with `d > 0`, kept in lowest terms so the products stay
// small.  Every product/sum is overflow-checked; an overflow is reported, not
// wrapped and not retried in double.  `__int128` is the same width the
// timestamp layer already uses (`uwb_twr_timestamp.h`).
namespace detail {

using i128 = __int128;

inline i128 i128_abs(i128 v) { return v < 0 ? -v : v; }

inline i128 i128_gcd(i128 a, i128 b)
{
    a = i128_abs(a);
    b = i128_abs(b);
    while (b != 0) {
        const i128 t = a % b;
        a = b;
        b = t;
    }
    return a;
}

struct Rat128 {
    i128 n;
    i128 d; // > 0
};

inline Rat128 rat128_make(i128 n, i128 d)
{
    if (d < 0) {
        n = -n;
        d = -d;
    }
    const i128 g = i128_gcd(n, d);
    if (g > 1) {
        n /= g;
        d /= g;
    }
    Rat128 r;
    r.n = n;
    r.d = d;
    return r;
}

inline bool rat128_mul(const Rat128& a, const Rat128& b, Rat128& out)
{
    i128 n = 0;
    i128 d = 0;
    if (__builtin_mul_overflow(a.n, b.n, &n))
        return false;
    if (__builtin_mul_overflow(a.d, b.d, &d))
        return false;
    out = rat128_make(n, d);
    return true;
}

inline bool rat128_add(const Rat128& a, const Rat128& b, Rat128& out)
{
    i128 t1 = 0;
    i128 t2 = 0;
    i128 n = 0;
    i128 d = 0;
    if (__builtin_mul_overflow(a.n, b.d, &t1))
        return false;
    if (__builtin_mul_overflow(b.n, a.d, &t2))
        return false;
    if (__builtin_add_overflow(t1, t2, &n))
        return false;
    if (__builtin_mul_overflow(a.d, b.d, &d))
        return false;
    out = rat128_make(n, d);
    return true;
}

inline bool rat128_sub(const Rat128& a, const Rat128& b, Rat128& out)
{
    Rat128 neg;
    neg.n = -b.n;
    neg.d = b.d;
    return rat128_add(a, neg, out);
}

inline bool rat128_div(const Rat128& a, const Rat128& b, Rat128& out)
{
    if (b.n == 0)
        return false;
    i128 n = 0;
    i128 d = 0;
    if (__builtin_mul_overflow(a.n, b.d, &n))
        return false;
    if (__builtin_mul_overflow(a.d, b.n, &d))
        return false;
    out = rat128_make(n, d);
    return true;
}

// The exact interval, via the ONLY accessor the ToF path may use.
inline bool rat128_from_interval(const RelativeTickInterval& ri, Rat128& out)
{
    int64_t num = 0;
    int64_t den = 0;
    if (!ri.exact_ratio(num, den))
        return false;
    out = rat128_make(static_cast<i128>(num), static_cast<i128>(den));
    return true;
}

inline TofRationalTicks rat128_to_ticks(const Rat128& r)
{
    TofRationalTicks t;
    t.num = static_cast<int64_t>(r.n);
    t.den = static_cast<int64_t>(r.d);
    t.valid = true;
    return t;
}

// Fits only when BOTH the numerator and the denominator fit in int64.  A
// result that does not fit is an Overflow, not a truncation.
inline bool rat128_fits_int64(const Rat128& r)
{
    const i128 lo = static_cast<i128>(std::numeric_limits<int64_t>::min());
    const i128 hi = static_cast<i128>(std::numeric_limits<int64_t>::max());
    return r.n >= lo && r.n <= hi && r.d >= 1 && r.d <= hi;
}

inline Rat128 rat128_one()
{
    Rat128 r;
    r.n = 1;
    r.d = 1;
    return r;
}

// ---- shared refusal plumbing -------------------------------------------
inline TofResult tof_fail(TofResult r, TofStatus s, const std::string& detail)
{
    r.ok = false;
    r.status = s;
    r.exchange_status = tof_status_to_exchange_status(s);
    r.detail = detail;
    return r;
}

// One timeline, stated once.  Identity (name+rate+bits) and epoch are kept
// distinct so the caller is told WHICH of the two failed.
inline bool tof_domain_check(TofResult& r, const char* what, const ClockDomain& got,
                             const ClockDomain& want)
{
    if (!got.is_valid()) {
        r = tof_fail(r, TofStatus::InvalidClockDomain,
                     std::string(what) + ": interval carries an invalid clock domain");
        return false;
    }
    if (!clock_domain_same_identity(got, want)) {
        r = tof_fail(r, TofStatus::ClockRatioDomainMismatch,
                     std::string(what) + " is in " + got.to_string() +
                         " but the clock ratio declares " + want.to_string());
        return false;
    }
    if (!clock_domain_same_epoch(got, want)) {
        r = tof_fail(r, TofStatus::ClockRatioEpochMismatch,
                     std::string(what) + " is epoch " + std::to_string(got.epoch_id) +
                         " but the clock ratio declares epoch " +
                         std::to_string(want.epoch_id));
        return false;
    }
    return true;
}

// k must be valid across the WHOLE A-domain interval the exchange runs on.
// For a wrapped interval the two raw readings straddle the wrap, so the span
// is [min, max]; requiring both ends inside the window is the safe reading.
inline bool tof_window_check(TofResult& r, const char* what,
                             const AdmittedRangingInterval& iv, const ClockRatio& k)
{
    if (!k.has_validity_window())
        return true;
    const int64_t a = iv.earlier().ticks();
    const int64_t b = iv.later().ticks();
    const int64_t lo = (a < b) ? a : b;
    const int64_t hi = (a < b) ? b : a;
    if (!k.covers_ticks(lo) || !k.covers_ticks(hi)) {
        r = tof_fail(r, TofStatus::ClockRatioNotValidAtTime,
                     std::string(what) + " spans A ticks [" + std::to_string(lo) + "," +
                         std::to_string(hi) + "] which is outside the clock ratio's " +
                         "validity window [" + std::to_string(k.valid_from_ticks()) + "," +
                         std::to_string(k.valid_until_ticks()) + "]");
        return false;
    }
    return true;
}

inline bool tof_load_interval(TofResult& r, const char* what,
                              const AdmittedRangingInterval& iv, Rat128& out)
{
    if (!iv.interval().is_valid()) {
        r = tof_fail(r, TofStatus::InvalidInput,
                     std::string(what) + ": admitted interval is not structurally valid");
        return false;
    }
    if (!rat128_from_interval(iv.interval(), out)) {
        r = tof_fail(r, TofStatus::Overflow,
                     std::string(what) + ": exact_ratio() did not fit int64");
        return false;
    }
    return true;
}

} // namespace detail

// ===========================================================================
// SS-TWR (REQ-PROTO-02)
// ===========================================================================
//
//   RA = t4A - t1A   (A ticks, admitted)
//   DB = t3B - t2B   (B ticks, admitted)
//   ToF_A = (RA - k*DB) / 2          in A ticks, SIGNED
//
// Hand check (the install-consumer example): a common clock, RA = 2000,
// DB = 1000 -> ToF = 500 A ticks.
//
// This function does NOT admit anything.  The caller (the future FSM) runs
// `admit_ranging_interval()` first; a bare `Timestamp` cannot even be passed.
inline TofResult compute_ss_tof(const AdmittedRangingInterval& ra,
                                const AdmittedRangingInterval& db,
                                const ClockRatio& k_ab)
{
    TofResult r;
    r.protocol = Protocol::Ss;
    r.computed_at = ComputedAt::InitiatorA; // REQ-PROTO-05: baseline at A
    r.ratio = k_ab;
    r.domain = k_ab.domain_a();

    const TofStatus ks = k_ab.check();
    if (ks != TofStatus::Ok)
        return detail::tof_fail(r, ks, "clock ratio is unusable: " + k_ab.to_string());

    if (!detail::tof_domain_check(r, "ra", ra.domain(), k_ab.domain_a()))
        return r;
    if (!detail::tof_domain_check(r, "db", db.domain(), k_ab.domain_b()))
        return r;
    // REQ-TIME-04: the ratio must be in force across the A-domain interval.
    if (!detail::tof_window_check(r, "ra", ra, k_ab))
        return r;

    detail::Rat128 RA;
    detail::Rat128 DB;
    if (!detail::tof_load_interval(r, "ra", ra, RA))
        return r;
    if (!detail::tof_load_interval(r, "db", db, DB))
        return r;

    const detail::Rat128 k = detail::rat128_make(k_ab.k_num(), k_ab.k_den());
    detail::Rat128 kdb;
    detail::Rat128 diff;
    detail::Rat128 half;
    if (!detail::rat128_mul(k, DB, kdb))
        return detail::tof_fail(r, TofStatus::Overflow, "k*DB left 128 bits");
    if (!detail::rat128_sub(RA, kdb, diff))
        return detail::tof_fail(r, TofStatus::Overflow, "RA - k*DB left 128 bits");
    if (!detail::rat128_div(diff, detail::rat128_make(2, 1), half))
        return detail::tof_fail(r, TofStatus::Overflow, "(RA - k*DB)/2 left 128 bits");
    if (!detail::rat128_fits_int64(half))
        return detail::tof_fail(r, TofStatus::Overflow,
                                "ToF numerator/denominator do not fit int64 ticks");

    r.ra_raw = detail::rat128_to_ticks(RA);
    r.db_raw = detail::rat128_to_ticks(DB);
    r.ra = r.ra_raw;                    // already A ticks
    r.db = detail::rat128_to_ticks(kdb); // k*DB, converted to A ticks
    r.tof = detail::rat128_to_ticks(half);
    r.display_seconds = r.tof.to_seconds(r.domain.tick_rate_hz);

    if (r.tof.num < 0) {
        r.ok = false;
        r.status = TofStatus::NegativeTof;
        r.exchange_status = tof_status_to_exchange_status(TofStatus::NegativeTof);
        r.detail = "SS ToF is negative (" + r.tof.to_string() +
                   " A ticks): retained with its sign, never clamped to 0";
        return r;
    }
    r.ok = true;
    r.status = TofStatus::Ok;
    r.exchange_status = ExchangeStatus::Ok;
    r.detail = "SS ToF = (RA - k*DB)/2";
    return r;
}

// ===========================================================================
// DS-TWR (REQ-PROTO-03)
// ===========================================================================
//
//   RA = t4A - t1A   DA = t5A - t4A   (A ticks, admitted)
//   DB = t3B - t2B   RB = t6B - t3B   (B ticks, admitted)
//   ToF_A = (RA*k*RB - DA*k*DB) / (RA + k*RB + DA + k*DB)
//
// COMMON UNIT = A TICKS.  `rb` and `db` are multiplied by k BEFORE the formula
// is applied; plugging the native B ticks straight into the quotient would mix
// two devices' units (REQ-PROTO-03 forbids exactly that).  Asymmetric reply
// delays (`DA != DB`) are legal and required to work.
inline TofResult compute_ds_tof(const AdmittedRangingInterval& ra,
                                const AdmittedRangingInterval& rb,
                                const AdmittedRangingInterval& da,
                                const AdmittedRangingInterval& db,
                                const ClockRatio& k_ab)
{
    TofResult r;
    r.protocol = Protocol::Ds;
    r.computed_at = ComputedAt::ResponderB; // REQ-PROTO-05: baseline at B
    r.ratio = k_ab;
    r.domain = k_ab.domain_a(); // the common unit is A ticks

    const TofStatus ks = k_ab.check();
    if (ks != TofStatus::Ok)
        return detail::tof_fail(r, ks, "clock ratio is unusable: " + k_ab.to_string());

    if (!detail::tof_domain_check(r, "ra", ra.domain(), k_ab.domain_a()))
        return r;
    if (!detail::tof_domain_check(r, "da", da.domain(), k_ab.domain_a()))
        return r;
    if (!detail::tof_domain_check(r, "rb", rb.domain(), k_ab.domain_b()))
        return r;
    if (!detail::tof_domain_check(r, "db", db.domain(), k_ab.domain_b()))
        return r;
    if (!detail::tof_window_check(r, "ra", ra, k_ab))
        return r;
    if (!detail::tof_window_check(r, "da", da, k_ab))
        return r;

    detail::Rat128 RA;
    detail::Rat128 RB;
    detail::Rat128 DA;
    detail::Rat128 DB;
    if (!detail::tof_load_interval(r, "ra", ra, RA))
        return r;
    if (!detail::tof_load_interval(r, "rb", rb, RB))
        return r;
    if (!detail::tof_load_interval(r, "da", da, DA))
        return r;
    if (!detail::tof_load_interval(r, "db", db, DB))
        return r;

    const detail::Rat128 k = detail::rat128_make(k_ab.k_num(), k_ab.k_den());
    detail::Rat128 rba;
    detail::Rat128 dba;
    if (!detail::rat128_mul(k, RB, rba))
        return detail::tof_fail(r, TofStatus::Overflow, "k*RB left 128 bits");
    if (!detail::rat128_mul(k, DB, dba))
        return detail::tof_fail(r, TofStatus::Overflow, "k*DB left 128 bits");

    detail::Rat128 p1;
    detail::Rat128 p2;
    detail::Rat128 numer;
    if (!detail::rat128_mul(RA, rba, p1))
        return detail::tof_fail(r, TofStatus::Overflow, "RA*k*RB left 128 bits");
    if (!detail::rat128_mul(DA, dba, p2))
        return detail::tof_fail(r, TofStatus::Overflow, "DA*k*DB left 128 bits");
    if (!detail::rat128_sub(p1, p2, numer))
        return detail::tof_fail(r, TofStatus::Overflow, "DS numerator left 128 bits");

    detail::Rat128 s1;
    detail::Rat128 s2;
    detail::Rat128 denom;
    if (!detail::rat128_add(RA, rba, s1))
        return detail::tof_fail(r, TofStatus::Overflow, "RA + k*RB left 128 bits");
    if (!detail::rat128_add(DA, dba, s2))
        return detail::tof_fail(r, TofStatus::Overflow, "DA + k*DB left 128 bits");
    if (!detail::rat128_add(s1, s2, denom))
        return detail::tof_fail(r, TofStatus::Overflow, "DS denominator left 128 bits");

    if (denom.n <= 0) {
        // Four non-negative intervals sum to something non-positive: only the
        // all-zero case is reachable, and the quotient is undefined.
        r.ok = false;
        r.status = TofStatus::ZeroDenominator;
        r.exchange_status = tof_status_to_exchange_status(TofStatus::ZeroDenominator);
        r.detail = "DS denominator is not positive: RA+k*RB+DA+k*DB = " +
                   detail::rat128_to_ticks(denom).to_string();
        return r;
    }

    detail::Rat128 tof;
    if (!detail::rat128_div(numer, denom, tof))
        return detail::tof_fail(r, TofStatus::Overflow, "DS quotient left 128 bits");
    if (!detail::rat128_fits_int64(tof))
        return detail::tof_fail(r, TofStatus::Overflow,
                                "ToF numerator/denominator do not fit int64 ticks");

    r.ra_raw = detail::rat128_to_ticks(RA);
    r.rb_raw = detail::rat128_to_ticks(RB);
    r.da_raw = detail::rat128_to_ticks(DA);
    r.db_raw = detail::rat128_to_ticks(DB);
    r.ra = r.ra_raw;                      // A ticks already
    r.da = r.da_raw;                      // A ticks already
    r.rb = detail::rat128_to_ticks(rba);  // converted to A ticks
    r.db = detail::rat128_to_ticks(dba);  // converted to A ticks
    r.tof = detail::rat128_to_ticks(tof);
    r.display_seconds = r.tof.to_seconds(r.domain.tick_rate_hz);

    if (r.tof.num < 0) {
        r.ok = false;
        r.status = TofStatus::NegativeTof;
        r.exchange_status = tof_status_to_exchange_status(TofStatus::NegativeTof);
        r.detail = "DS ToF is negative (" + r.tof.to_string() +
                   " A ticks): retained with its sign, never clamped to 0";
        return r;
    }
    r.ok = true;
    r.status = TofStatus::Ok;
    r.exchange_status = ExchangeStatus::Ok;
    r.detail = "DS ToF = (RA*k*RB - DA*k*DB)/(RA+k*RB+DA+k*DB), all in A ticks";
    return r;
}

// ===========================================================================
// Optional derived distance
// ===========================================================================
//
// Only defined for a successful (non-negative) ToF.  The speed is a parameter
// so a test can inject a cable velocity; the vacuum constant is named.  This
// is NOT an M1-A acceptance criterion and no centimetre claim is made.
inline bool tof_distance_m(const TofResult& r, double propagation_speed_mps, double& out_m)
{
    if (!r.ok || !r.tof.valid)
        return false;
    if (!(propagation_speed_mps > 0.0) || !std::isfinite(propagation_speed_mps))
        return false;
    const double seconds = r.tof.to_seconds(r.domain.tick_rate_hz);
    const double distance = propagation_speed_mps * seconds;
    if (!std::isfinite(distance))
        return false;
    out_m = distance;
    return true;
}

} // namespace twr
} // namespace uwb
} // namespace gr

#endif /* INCLUDED_GNURADIO_UWB_UWB_TWR_MATH_H */
