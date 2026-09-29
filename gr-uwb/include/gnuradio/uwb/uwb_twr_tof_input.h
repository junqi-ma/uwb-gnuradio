/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Ranging-input admission for the UWB SS-TWR / DS-TWR protocol core (M0.1).
 *
 * ---------------------------------------------------------------------------
 * Why this file exists: review defect R6 (P1)
 * ---------------------------------------------------------------------------
 *
 * M0 shipped `timestamp_interval()`, and it was reproduced that two
 * `UhdRxFirstIqSample + HardwareMeasured + kCorrectionNone` timestamps return
 * `TimeIntervalStatus::Ok`.  That is NOT a bug in `timestamp_interval()`:
 * "how far apart are these two sample times" is a perfectly reasonable
 * general-purpose question, and an adapter needs it.  The bug was in the
 * contract AROUND it:
 *
 *   * M0's own report claimed "裸设备时间戳无法产出协议 interval", which the
 *     reproducer contradicts; and
 *   * nothing distinguished "an Ok sample-to-sample difference" from "a
 *     validated ranging input", so M1 reading `interval.status == Ok` would
 *     have been reading a distance from a raw host capture coordinate.
 *
 * The review's warning is honoured here: the fix is NOT "set the correction
 * bits".  A bit in `applied_corrections` is an ASSERTION by the pipeline; if
 * the gate only checked bits, it would be checking an assertion rather than a
 * condition, and the R6 reproducer would pass by fiat.  This file therefore
 * checks conditions that cannot be produced by merely flipping a bit:
 * a real first-path DECISION with real numbers behind it, a calibration
 * record that says it was applied exactly once, and a calibration that is
 * current for this exchange's epoch and instant.
 *
 * ---------------------------------------------------------------------------
 * The shape of the fix
 * ---------------------------------------------------------------------------
 *
 * `RangeCapableTime` is a DISTINCT TYPE, not a flag on `Timestamp`.  It has a
 * private constructor whose only caller is `admit_range_capable_time()`, so
 * there is no way to write `RangeCapableTime t(raw_timestamp);` and no way to
 * reach a distance computation with an unvalidated raw time.  A
 * `Timestamp` converts to nothing; the only conversion that exists runs
 * through the gate.
 *
 * `admit_ranging_interval()` is the entry point M1-A will use.  On success it
 * yields an `AdmittedRangingInterval`, whose `interval()` is a
 * `RelativeTickInterval` -- B3's type with NO nanosecond field, so there is
 * no lossy projection to reach for -- together with the two admitted
 * instants, so the DS formula has both sides it needs without ever touching a
 * raw timestamp again.
 *
 * ---------------------------------------------------------------------------
 * Admission conditions, and what each one traces to
 * ---------------------------------------------------------------------------
 *
 *   1. self-consistent Timestamp                  REQ-TIME-01
 *   2. marker is range-capable for ToF             REQ-TIME-02
 *        {RmarkerRx, RmarkerTx, AntennaPlane}.
 *        `UhdRxFirstIqSample` is NEVER one, whatever bits are set.
 *   3. source is a hardware measurement            REQ-TIME-03
 *        `HardwareMeasured` only.  `ScheduledCalibrated` is a schedule, not
 *        an air time; `Estimated` / `Reconstructed` are neither.
 *   4. every correction the marker REQUIRES is recorded
 *        REQ-TIME-02 / REQ-TIME-03.  The refusal names the missing bit.
 *   5. a calibration is present, applied EXACTLY once, and current
 *        REQ-CAL-01.  Present / once / current are three separate failures.
 *   6. first-path quality: recorded AND passed     REQ-TIME-05
 *        Required for the receive families (WaveformRx, ReferencePlane).
 *        A transmit RMARKER has no first path, so it needs no gate.
 *   7. the pair is on ONE timeline                 REQ-TIME-01
 *        Existing `ClockDomain` comparability rules only; no second notion
 *        of domain identity is introduced here.
 *   8. the marker PAIR denotes a protocol interval REQ-TIME-02
 *   9. the interval itself is formable              REQ-TIME-04
 *        (wrap ambiguity etc. surfaces as `IntervalNotFormable` with the
 *        underlying `TimeIntervalStatus` attached.)
 *
 * ---------------------------------------------------------------------------
 * What is deliberately NOT here
 * ---------------------------------------------------------------------------
 *
 * The SS and DS ToF formulas (M1-A), the clock-ratio estimate, and the
 * first-path gate ARITHMETIC.  On that last point, stated plainly because it
 * would be easy to overclaim: `FirstPathQuality` models the *condition* that
 * REQ-TIME-05 requires -- an explicit decision backed by the numbers the
 * decision was made from -- and this file checks that the decision exists,
 * is self-consistent, and says "passed".  It does NOT compute an SNR
 * threshold, a peak-vs-first-path discrimination margin, or a confidence from
 * a CIR.  Choosing those numbers is M2's job, and no default in this header
 * can produce a "passed": a default-constructed `FirstPathQuality` is
 * `NotRecorded`, which is refused.
 */

#ifndef INCLUDED_GNURADIO_UWB_UWB_TWR_TOF_INPUT_H
#define INCLUDED_GNURADIO_UWB_UWB_TWR_TOF_INPUT_H

#include <gnuradio/uwb/uwb_twr_types.h>
#include <gnuradio/uwb/uwb_twr_timestamp.h>

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace gr {
namespace uwb {
namespace twr {

struct RangeAdmission;
struct RangingIntervalAdmission;

// ===========================================================================
// First-path quality -- the DECISION, not the raw peak
// ===========================================================================
//
// REQ-TIME-05: "首径定位必须有质量判决" -- first-path location must carry a
// quality decision, and "首径不可靠时输出低质量/失败状态，不制造有效距离".
// REQ-TIME-05 also says the largest correlation peak may be a later multipath
// component, so the peak alone is not the answer.
//
// The three numbers below are stored so the decision can be audited after the
// fact (they belong in the result JSON, REQ-OUT-01).  The gate arithmetic --
// what SNR counts as enough, how peak-to-first-path discrimination is judged,
// how a confidence is derived -- is M2's job.  This header requires the
// DECISION to exist and to be internally consistent; it never invents one,
// and it has no default that reads as "fine".

enum class FirstPathDecision : uint8_t {
    NotRecorded = 0, // nothing was evaluated: must never be read as "fine"
    Passed = 1,      // an explicit, positive quality verdict
    Failed = 2       // an explicit, negative verdict
};

inline const char* first_path_decision_to_string(FirstPathDecision d)
{
    switch (d) {
    case FirstPathDecision::NotRecorded:
        return "not_recorded";
    case FirstPathDecision::Passed:
        return "passed";
    case FirstPathDecision::Failed:
        return "failed";
    }
    return "invalid";
}

inline bool first_path_decision_from_string(const std::string& s, FirstPathDecision& out)
{
    for (int i = 0; i <= static_cast<int>(FirstPathDecision::Failed); ++i) {
        const FirstPathDecision c = static_cast<FirstPathDecision>(i);
        if (s == first_path_decision_to_string(c)) {
            out = c;
            return true;
        }
    }
    return false;
}

struct FirstPathQuality {
    FirstPathDecision decision = FirstPathDecision::NotRecorded;
    double first_path_snr_db = 0.0;      // M2's estimate
    double peak_to_first_path_db = 0.0;  // peak dominance over the first path
    double confidence = 0.0;              // [0, 1]

    static FirstPathQuality not_recorded() { return FirstPathQuality(); }

    static FirstPathQuality
    passed(double first_path_snr_db, double peak_to_first_path_db, double confidence)
    {
        FirstPathQuality q;
        q.decision = FirstPathDecision::Passed;
        q.first_path_snr_db = first_path_snr_db;
        q.peak_to_first_path_db = peak_to_first_path_db;
        q.confidence = confidence;
        return q;
    }

    static FirstPathQuality
    failed(double first_path_snr_db, double peak_to_first_path_db, double confidence)
    {
        FirstPathQuality q = passed(first_path_snr_db, peak_to_first_path_db, confidence);
        q.decision = FirstPathDecision::Failed;
        return q;
    }

    bool is_recorded() const { return decision != FirstPathDecision::NotRecorded; }
    bool has_passed() const { return decision == FirstPathDecision::Passed; }

    // A verdict is only believable if the numbers behind it are finite and
    // the confidence is a fraction.  NaN sneaking in from a failed CIR
    // estimate is exactly the kind of thing this catches.
    bool measurements_are_sane() const
    {
        if (!std::isfinite(first_path_snr_db) || !std::isfinite(peak_to_first_path_db) ||
            !std::isfinite(confidence))
            return false;
        if (confidence < 0.0 || confidence > 1.0)
            return false;
        return true;
    }

    std::string to_string() const
    {
        char buf[192];
        std::snprintf(buf, sizeof(buf),
                      "{\"decision\":\"%s\",\"first_path_snr_db\":%.17g,"
                      "\"peak_to_first_path_db\":%.17g,\"confidence\":%.17g}",
                      first_path_decision_to_string(decision), first_path_snr_db,
                      peak_to_first_path_db, confidence);
        return std::string(buf);
    }
};

// ===========================================================================
// Calibration evidence -- REQ-CAL-01
// ===========================================================================
//
// `Timestamp::calibration_id` records WHICH calibration a timestamp claims.
// That alone cannot tell "calibrated once by r1" from "calibrated twice, once
// by r1 and once by r2", because the second application of a different id is
// refused by the header and leaves the timestamp untouched.
//
// So the gate also requires the RECORD of the calibration stage: the
// `CalibrationResult` values the stage actually returned.  Exactly one
// `Applied` is required; `AlreadyApplied` means the stage was run twice;
// `ConflictingCalibration` means two sets were tried.  This is real evidence
// produced by the real stage, not a flag -- but it is evidence the CALLER
// supplies, and that limitation is stated rather than hidden: a pipeline that
// fabricates a record can still lie to this gate.  What the gate guarantees
// is that "I forgot to check" and "I never ran the stage" cannot be
// expressed, and that a stage that returned anything but a single `Applied`
// is refused.

struct CalibrationApplication {
    std::string calibration_id;
    CalibrationResult result = CalibrationResult::InvalidTimestamp;
};

struct CalibrationStamp {
    // The calibration set identity: device/channel/profile/rate/gain, per
    // REQ-CAL-01's versioned-calibration requirement.
    std::string id;
    // The clock epoch the calibration was established in.  A reboot or UHD
    // time reset allocates a new epoch, and a calibration from a previous
    // epoch cannot be reused: the counter it was measured against restarted.
    uint64_t calibrated_epoch = 0;
    // Validity window on the SAME counter, `[from, until)`.  A wrapping
    // domain requires both bounds inside one wrap period, so the window can
    // never straddle the rollover and silently become a second window.
    int64_t valid_from_ticks = 0;
    int64_t valid_until_ticks = 0;
    // What the calibration stage returned.  Must be exactly one `Applied`.
    std::vector<CalibrationApplication> applications;

    bool is_well_formed() const
    {
        return !id.empty() && valid_from_ticks < valid_until_ticks;
    }
};

namespace detail {

// A validity window is only meaningful if both bounds live in the SAME wrap
// period of the domain it is expressed on.  A window that crosses the
// rollover would be satisfied by two disjoint instants and is refused.
inline bool calibration_window_fits_domain(const ClockDomain& d, const CalibrationStamp& c)
{
    const uint64_t p = clock_domain_wrap_period(d);
    if (p == 0u)
        return c.valid_from_ticks >= 0 && c.valid_until_ticks >= 0;
    return c.valid_from_ticks >= 0 && c.valid_until_ticks >= 0 &&
           static_cast<uint64_t>(c.valid_until_ticks) < p;
}

} // namespace detail

// ===========================================================================
// REQ-TIME-03: the evidence a SCHEDULED transmit must carry
// ===========================================================================
//
// The requirement is explicit:
//
//   "UHD timed TX 的目标是第一样点；send 返回成功或 burst ACK 不等价于芯片
//    提供的精细空口 timestamp。可通过确定性时序加标定取得合格 TX timestamp，
//    但必须标注 `scheduled_calibrated` 来源，关联 async error，不能冒充
//    `hardware_measured`。"
//
// So a deterministic timed TX IS a legitimate ranging input -- the X410 route
// depends on it -- provided the planned instant is recorded and `send()`
// returning is not mistaken for the burst having happened.  Which of the two
// it was cannot be told from `send()`: UHD reports late/underflow
// asynchronously, after the call has already returned.
enum class TxOutcome : uint8_t {
    Unknown = 0,    // nothing observed yet -- NOT "fine"
    Completed = 1,  // the burst completed with no late/underflow
    Late = 2,       // ERR_TX_LATE_DATA: the deadline could not be met
    Underflow = 3,  // the TX stream underran
    Cancelled = 4   // stop()/cancel during the burst
};

inline const char* tx_outcome_to_string(TxOutcome o)
{
    switch (o) {
    case TxOutcome::Unknown:
        return "unknown";
    case TxOutcome::Completed:
        return "completed";
    case TxOutcome::Late:
        return "late";
    case TxOutcome::Underflow:
        return "underflow";
    case TxOutcome::Cancelled:
        return "cancelled";
    }
    return "invalid";
}

// Which of the planned-instant records is missing, as a bit mask, so a
// refusal names the one stage that did not run instead of saying "incomplete".
enum class TxPlanRecord : uint32_t {
    CommandTime = 1u << 0,          // the UHD command time
    QuantisedInstant = 1u << 1,     // the first quantised sample instant
    MarkerOffset = 1u << 2,         // the waveform marker offset
    CalibratedAirTime = 1u << 3     // the calibrated expected air time
};

inline const char* tx_plan_record_name(uint32_t bit)
{
    switch (static_cast<TxPlanRecord>(bit)) {
    case TxPlanRecord::CommandTime:
        return "command_time";
    case TxPlanRecord::QuantisedInstant:
        return "quantised_instant";
    case TxPlanRecord::MarkerOffset:
        return "marker_offset";
    case TxPlanRecord::CalibratedAirTime:
        return "calibrated_air_time";
    }
    return "unknown";
}

// The send evidence for ONE scheduled transmit.  Every field is explicit and
// defaults to "not recorded", so a default-constructed value records nothing
// and admits nothing.
struct TxSendEvidence {
    bool command_time_recorded = false;
    bool quantised_instant_recorded = false;
    bool marker_offset_recorded = false;
    bool calibrated_air_time_recorded = false;

    // `send()` returned success or the burst was ACKed.  Recorded because it is
    // a real observation, but deliberately NOT sufficient on its own: no check
    // below admits on this bit alone.
    bool send_accepted = false;

    TxOutcome outcome = TxOutcome::Unknown;

    // The planned instant, fully recorded.  Says nothing about whether the
    // burst happened; `completed()` is the other half.
    bool records_the_plan() const
    {
        return command_time_recorded && quantised_instant_recorded &&
               marker_offset_recorded && calibrated_air_time_recorded;
    }
    bool completed() const { return outcome == TxOutcome::Completed; }

    // The first missing plan record, or 0 when all four were recorded.
    uint32_t missing_plan_record() const
    {
        uint32_t missing = 0u;
        if (!command_time_recorded)
            missing |= static_cast<uint32_t>(TxPlanRecord::CommandTime);
        if (!quantised_instant_recorded)
            missing |= static_cast<uint32_t>(TxPlanRecord::QuantisedInstant);
        if (!marker_offset_recorded)
            missing |= static_cast<uint32_t>(TxPlanRecord::MarkerOffset);
        if (!calibrated_air_time_recorded)
            missing |= static_cast<uint32_t>(TxPlanRecord::CalibratedAirTime);
        return missing;
    }

    std::string to_string() const
    {
        return std::string("TxSendEvidence{plan=") +
               (records_the_plan() ? "recorded" : "incomplete") +
               ", send_accepted=" + (send_accepted ? "true" : "false") +
               ", outcome=" + tx_outcome_to_string(outcome) + "}";
    }
};

// ===========================================================================
// The admission context -- what the pipeline hands the gate
// ===========================================================================
//
// Every field is explicit and none has a permissive default:
//   * `calibration == nullptr`  -> refused (no calibration set in force)
//   * `reference_ticks_recorded == false` -> refused (cannot decide currency)
//   * `rx_first_path` default-constructed -> refused for receive markers
//   * `tx_evidence == nullptr` -> refused for a scheduled TRANSMIT
//
// A default-constructed `RangeAdmissionContext` therefore admits NOTHING,
// which is the whole point: forgetting to fill one in is a compile-clean
// runtime rejection, not a silent success.
struct RangeAdmissionContext {
    const CalibrationStamp* calibration = nullptr;

    // The exchange's own instant on the same counter, used to decide whether
    // the calibration is current.  It is a pointer-free value plus an
    // explicit "was it supplied" flag because tick 0 is a legal instant and
    // must not double as "unset".
    int64_t reference_ticks = 0;
    bool reference_ticks_recorded = false;

    // REQ-TIME-05.  Applies to the receive marker families only; a transmit
    // RMARKER has no first path to judge.
    FirstPathQuality rx_first_path;

    // REQ-TIME-03.  Required for a `ScheduledCalibrated` TRANSMIT RMARKER and
    // ignored for everything else.  `nullptr` means "no send evidence was
    // supplied", which is a REFUSAL -- not a permissive default.
    const TxSendEvidence* tx_evidence = nullptr;
};

// ===========================================================================
// Rejection reasons
// ===========================================================================
//
// Numeric values are part of the result JSON / CSV schema: append only.
// Every value maps to a specific `ExchangeStatus` (see
// `range_admission_reason_to_exchange_status()`), and the mapping keeps the
// three physical failure classes -- timing domain, calibration, first path --
// on distinct terminal codes, so a soak report can separate them without
// parsing a detail string.

enum class RangeAdmissionReason : uint8_t {
    Admitted = 0,
    InvalidTimestamp = 1,          // REQ-TIME-01
    NotRangeCapableMarker = 2,     // REQ-TIME-02
    NotMeasured = 3,               // REQ-TIME-03
    MissingCorrection = 4,         // REQ-TIME-02 / 03, `missing_correction` names it
    NoCalibration = 5,             // REQ-CAL-01
    CalibrationMismatch = 6,       // REQ-CAL-01
    CalibrationAppliedTwice = 7,   // REQ-CAL-01
    CalibrationStale = 8,          // REQ-CAL-01
    FirstPathQualityNotRecorded = 9, // REQ-TIME-05
    FirstPathQualityInvalid = 10,    // REQ-TIME-05
    FirstPathUnreliable = 11,        // REQ-TIME-05
    MarkerPairNotAllowed = 12,       // REQ-TIME-02
    WrongEpoch = 13,                 // REQ-TIME-01
    CrossDomain = 14,                // REQ-TIME-01 / 04
    IntervalNotFormable = 15,        // REQ-TIME-04
    // REQ-TIME-03: a deterministic timed TX must carry the evidence that the
    // burst happened as planned.  `send()` returning success is NOT it.
    ScheduledTxEvidenceMissing = 16,
    TxOutcomeLate = 17,              // the timed TX missed its deadline
    TxOutcomeUnderflow = 18,         // the TX stream underran
    TxOutcomeCancelled = 19          // the burst was cancelled
};

inline constexpr int kRangeAdmissionReasonMax =
    static_cast<int>(RangeAdmissionReason::TxOutcomeCancelled);

inline const char* range_admission_reason_to_string(RangeAdmissionReason r)
{
    switch (r) {
    case RangeAdmissionReason::Admitted:
        return "admitted";
    case RangeAdmissionReason::InvalidTimestamp:
        return "invalid_timestamp";
    case RangeAdmissionReason::NotRangeCapableMarker:
        return "not_range_capable_marker";
    case RangeAdmissionReason::NotMeasured:
        return "not_measured";
    case RangeAdmissionReason::MissingCorrection:
        return "missing_correction";
    case RangeAdmissionReason::NoCalibration:
        return "no_calibration";
    case RangeAdmissionReason::CalibrationMismatch:
        return "calibration_mismatch";
    case RangeAdmissionReason::CalibrationAppliedTwice:
        return "calibration_applied_twice";
    case RangeAdmissionReason::CalibrationStale:
        return "calibration_stale";
    case RangeAdmissionReason::FirstPathQualityNotRecorded:
        return "first_path_quality_not_recorded";
    case RangeAdmissionReason::FirstPathQualityInvalid:
        return "first_path_quality_invalid";
    case RangeAdmissionReason::FirstPathUnreliable:
        return "first_path_unreliable";
    case RangeAdmissionReason::MarkerPairNotAllowed:
        return "marker_pair_not_allowed";
    case RangeAdmissionReason::WrongEpoch:
        return "wrong_epoch";
    case RangeAdmissionReason::CrossDomain:
        return "cross_domain";
    case RangeAdmissionReason::IntervalNotFormable:
        return "interval_not_formable";
    case RangeAdmissionReason::ScheduledTxEvidenceMissing:
        return "scheduled_tx_evidence_missing";
    case RangeAdmissionReason::TxOutcomeLate:
        return "tx_outcome_late";
    case RangeAdmissionReason::TxOutcomeUnderflow:
        return "tx_outcome_underflow";
    case RangeAdmissionReason::TxOutcomeCancelled:
        return "tx_outcome_cancelled";
    }
    return "invalid";
}

inline bool range_admission_reason_from_string(const std::string& s, RangeAdmissionReason& out)
{
    for (int i = 0; i <= kRangeAdmissionReasonMax; ++i) {
        const RangeAdmissionReason c = static_cast<RangeAdmissionReason>(i);
        if (s == range_admission_reason_to_string(c)) {
            out = c;
            return true;
        }
    }
    return false;
}

// REQ-ERR-01: one terminal status per exchange, with the specific cause kept
// in the reason.  The calibration and first-path classes get their own codes
// so a failure report says WHICH contract broke without reading prose.
inline ExchangeStatus range_admission_reason_to_exchange_status(RangeAdmissionReason r)
{
    switch (r) {
    case RangeAdmissionReason::Admitted:
        return ExchangeStatus::Ok;
    case RangeAdmissionReason::NoCalibration:
    case RangeAdmissionReason::CalibrationMismatch:
    case RangeAdmissionReason::CalibrationAppliedTwice:
        return ExchangeStatus::CalibrationMissing;
    case RangeAdmissionReason::CalibrationStale:
        return ExchangeStatus::CalibrationExpired;
    case RangeAdmissionReason::FirstPathQualityNotRecorded:
    case RangeAdmissionReason::FirstPathQualityInvalid:
    case RangeAdmissionReason::FirstPathUnreliable:
        return ExchangeStatus::FirstPathUnreliable;
    case RangeAdmissionReason::InvalidTimestamp:
    case RangeAdmissionReason::NotRangeCapableMarker:
    case RangeAdmissionReason::NotMeasured:
    case RangeAdmissionReason::MissingCorrection:
    case RangeAdmissionReason::MarkerPairNotAllowed:
    case RangeAdmissionReason::WrongEpoch:
    case RangeAdmissionReason::CrossDomain:
    case RangeAdmissionReason::IntervalNotFormable:
    case RangeAdmissionReason::ScheduledTxEvidenceMissing:
        return ExchangeStatus::InvalidTimeDomain;
    // A timed TX that did not complete is NOT an invalid time domain: the
    // domain is fine, the burst is the problem.  Each of these maps to its own
    // terminal status so a soak report can separate them without parsing a
    // detail string (REQ-ERR-01).
    case RangeAdmissionReason::TxOutcomeLate:
        return ExchangeStatus::TxLate;
    case RangeAdmissionReason::TxOutcomeUnderflow:
        return ExchangeStatus::TxUnderflow;
    case RangeAdmissionReason::TxOutcomeCancelled:
        return ExchangeStatus::Cancelled;
    }
    return ExchangeStatus::InternalError;
}

// Which side of a pair the refusal is about, so the caller knows whether to
// blame the transmit or the receive time without re-deriving it.
enum class RangeTimeSide : uint8_t {
    Unknown = 0,
    Subject = 1, // single-timestamp admission
    Later = 2,
    Earlier = 3
};

inline const char* range_time_side_to_string(RangeTimeSide s)
{
    switch (s) {
    case RangeTimeSide::Unknown:
        return "unknown";
    case RangeTimeSide::Subject:
        return "subject";
    case RangeTimeSide::Later:
        return "later";
    case RangeTimeSide::Earlier:
        return "earlier";
    }
    return "invalid";
}

// ===========================================================================
// RangeCapableTime -- a validated ranging input
// ===========================================================================
//
// A single instant that has passed the gate.  Private constructor, public
// copy/move: it is a value that can be stored and passed on, and there is no
// public way to create one.  `timestamp()` is the only way back to the raw
// `Timestamp`, and it is read-only.
class RangeCapableTime {
public:
    RangeCapableTime(const RangeCapableTime&) = default;
    RangeCapableTime(RangeCapableTime&&) = default;
    RangeCapableTime& operator=(const RangeCapableTime&) = default;
    RangeCapableTime& operator=(RangeCapableTime&&) = default;
    ~RangeCapableTime() = default;

    // Trivially true: reaching one of these at all means the gate ran.  Kept
    // so a call site can log the fact without holding a separate flag.
    bool is_range_capable() const { return true; }

    const Timestamp& timestamp() const { return ts_; }
    const ClockDomain& domain() const { return ts_.domain; }
    TimestampMarker marker() const { return ts_.marker; }
    TimestampSource source() const { return ts_.source; }
    const std::string& calibration_id() const { return ts_.calibration_id; }
    uint32_t applied_corrections() const { return ts_.applied_corrections; }

    // `NotRecorded` for a transmit RMARKER, which has no first path.
    const FirstPathQuality& first_path_quality() const { return first_path_; }

    int64_t ticks() const { return ts_.ticks; }
    int32_t frac_num() const { return ts_.frac_num; }
    uint32_t frac_den() const { return ts_.frac_den; }
    bool is_whole_ticks() const { return ts_.frac_num == 0; }

    // The exact instant as a rational number of TICKS, for the M1 formulas
    // that need an absolute value rather than a difference.  No nanosecond
    // accessor is offered on purpose: see the file header.
    bool exact_ratio(int64_t& out_num, int64_t& out_den) const
    {
        if (ts_.frac_den == 0u) {
            out_num = ts_.ticks;
            out_den = 1;
            return true;
        }
        const int64_t den = static_cast<int64_t>(ts_.frac_den);
        if (ts_.ticks > (std::numeric_limits<int64_t>::max() - ts_.frac_num) / den)
            return false;
        out_num = ts_.ticks * den + ts_.frac_num;
        out_den = den;
        return true;
    }

    std::string to_decimal_string() const { return timestamp_to_decimal_string(ts_); }
    std::string to_json_string() const { return timestamp_to_json_string(ts_); }

private:
    RangeCapableTime(const Timestamp& ts, const FirstPathQuality& quality)
        : ts_(ts), first_path_(quality)
    {
    }

    // The ONLY construction path.  Declared as a friend, not as a public
    // static, so the gate cannot be bypassed by naming a key.
    friend RangeAdmission admit_range_capable_time(const Timestamp& ts,
                                                   const RangeAdmissionContext& ctx);

    Timestamp ts_;
    FirstPathQuality first_path_;
};

// ===========================================================================
// The single-timestamp admission result
// ===========================================================================

struct RangeAdmission {
    bool admitted = false;
    RangeAdmissionReason reason = RangeAdmissionReason::InvalidTimestamp;
    ExchangeStatus status = ExchangeStatus::InvalidTimeDomain;

    // Engaged if and only if `admitted`.  The invariant is asserted by the
    // QA suite: there is no state in which a refusal still carries a value.
    std::optional<RangeCapableTime> value;

    RangeTimeSide side = RangeTimeSide::Subject;
    TimestampMarker marker = TimestampMarker::RmarkerRx;
    // The exact missing correction bit, for `MissingCorrection`; 0 otherwise.
    uint32_t missing_correction = 0u;
    // The underlying interval status, for `IntervalNotFormable`; unused by
    // the single-timestamp form.
    TimeIntervalStatus interval_status = TimeIntervalStatus::InvalidTimestamp;
    // Human-readable detail for logs.  Never load-bearing: the reason code
    // and the fields above are what a machine reads.
    std::string detail;

    std::string to_json_string() const
    {
        char buf[768];
        std::snprintf(buf, sizeof(buf),
                      "{\"admitted\":%s,\"reason\":\"%s\",\"exchange_status\":\"%s\","
                      "\"side\":\"%s\",\"marker\":\"%s\",\"missing_correction\":\"%s\","
                      "\"detail\":\"%s\"}",
                      admitted ? "true" : "false",
                      range_admission_reason_to_string(reason),
                      exchange_status_to_string(status), range_time_side_to_string(side),
                      timestamp_marker_to_string(marker),
                      timestamp_correction_name(missing_correction), detail.c_str());
        return std::string(buf);
    }
};

// ===========================================================================
// AdmittedRangingInterval -- the value M1-A consumes
// ===========================================================================
//
// `interval()` is a `RelativeTickInterval`, which has no nanosecond field, so
// the lossy integer-ns projection that R4 identified cannot be read off this
// object even by accident.  `ns_projection_is_lossless()` exists only to let
// a caller LABEL a projection it makes for display, and is documented as not
// being an input to any formula.

class AdmittedRangingInterval {
public:
    AdmittedRangingInterval(const AdmittedRangingInterval&) = default;
    AdmittedRangingInterval(AdmittedRangingInterval&&) = default;
    AdmittedRangingInterval& operator=(const AdmittedRangingInterval&) = default;
    AdmittedRangingInterval& operator=(AdmittedRangingInterval&&) = default;
    ~AdmittedRangingInterval() = default;

    // The exact, projection-free interval.  This is the value ToF consumes.
    const RelativeTickInterval& interval() const { return interval_; }
    // The two admitted instants, for formulas that need the absolute ticks
    // (the DS numerator is built from four intervals, but the sanity checks
    // want the endpoints).
    const RangeCapableTime& later() const { return later_; }
    const RangeCapableTime& earlier() const { return earlier_; }

    bool is_whole_ticks() const { return interval_.is_whole_ticks(); }
    bool wrapped() const { return interval_.wrapped; }
    const ClockDomain& domain() const { return interval_.domain; }
    TimestampMarker later_marker() const { return interval_.later_marker; }
    TimestampMarker earlier_marker() const { return interval_.earlier_marker; }

    // EXACT rational tick value: num/den.  No floating point, no truncation;
    // seconds follow as (num/den)/tick_rate_hz, evaluated once, at the end.
    bool exact_ratio(int64_t& out_num, int64_t& out_den) const
    {
        return interval_.exact_ratio(out_num, out_den);
    }

    // DISPLAY / LOG ONLY.  Tells a caller whether an integer-nanosecond
    // projection of this interval would be faithful; it is never an input to
    // a distance.
    bool ns_projection_is_lossless() const { return interval_.ns_projection_is_lossless(); }

    std::string to_string() const
    {
        int64_t num = 0;
        int64_t den = 0;
        const bool exact = exact_ratio(num, den);
        char buf[384];
        std::snprintf(buf, sizeof(buf),
                      "ticks=%lld+%d/%u later=%s earlier=%s wrapped=%s exact=%s",
                      static_cast<long long>(interval_.ticks),
                      static_cast<int>(interval_.frac_num),
                      static_cast<unsigned>(interval_.frac_den),
                      timestamp_marker_to_string(later_marker()),
                      timestamp_marker_to_string(earlier_marker()),
                      wrapped() ? "true" : "false", exact ? "true" : "false");
        std::string s(buf);
        if (exact) {
            char ex[64];
            std::snprintf(ex, sizeof(ex), " ratio=%lld/%lld", static_cast<long long>(num),
                          static_cast<long long>(den));
            s += ex;
        }
        return s;
    }

private:
    AdmittedRangingInterval(const RelativeTickInterval& interval,
                            const RangeCapableTime& later,
                            const RangeCapableTime& earlier)
        : interval_(interval), later_(later), earlier_(earlier)
    {
    }

    friend RangingIntervalAdmission admit_ranging_interval(
        const Timestamp& later, const Timestamp& earlier, const RangeAdmissionContext& ctx);

    RelativeTickInterval interval_;
    RangeCapableTime later_;
    RangeCapableTime earlier_;
};

// ===========================================================================
// The pair admission result
// ===========================================================================

struct RangingIntervalAdmission {
    bool admitted = false;
    RangeAdmissionReason reason = RangeAdmissionReason::InvalidTimestamp;
    ExchangeStatus status = ExchangeStatus::InvalidTimeDomain;

    // Engaged if and only if `admitted`.
    std::optional<AdmittedRangingInterval> value;

    RangeTimeSide side = RangeTimeSide::Unknown;
    TimestampMarker marker = TimestampMarker::RmarkerRx;
    uint32_t missing_correction = 0u;
    // Populated for `CrossDomain` / `WrongEpoch` / `MarkerPairNotAllowed` /
    // `IntervalNotFormable`: the specific `TimeIntervalStatus` the shared
    // interval layer would have produced.  It is recorded so a caller can
    // tell "these are not on one timeline" from "the modular difference is
    // ambiguous" without re-running the check.
    TimeIntervalStatus interval_status = TimeIntervalStatus::InvalidTimestamp;
    std::string detail;
};

// ===========================================================================
// The gate
// ===========================================================================

namespace detail {

// Which marker families must clear the first-path gate (REQ-TIME-05).  The
// transmit RMARKER family is excluded: REQ-TIME-05 is about locating an
// incoming first path, and a TX time has none to locate.
inline bool tof_marker_requires_first_path(TimestampMarker m)
{
    const TimestampMarkerClass c = timestamp_marker_class(m);
    return c == TimestampMarkerClass::WaveformRx || c == TimestampMarkerClass::ReferencePlane;
}

// The ToF-admissible marker set.  Deliberately wider than the timestamp
// header's `timestamp_is_range_capable()`, which is the *interval* gate and
// admits only the two protocol RMARKERs.  REQ-TIME-02 puts the calibrated
// antenna reference plane in the same family -- it is an on-air instant, and
// a link-delay calibration is expressed there -- so the reference plane is
// admitted here.  What is excluded is everything that is a bookkeeping
// coordinate rather than an on-air instant: the host capture sample time, and
// the preamble/SFD/PHR positions, which are fixed waveform geometry that
// belongs to the PHY profile and is never two independent timestamps.
inline bool tof_marker_is_range_capable(TimestampMarker m)
{
    return m == TimestampMarker::RmarkerRx || m == TimestampMarker::RmarkerTx ||
           m == TimestampMarker::AntennaPlane;
}

// No default arguments on purpose: every refusal must state WHICH side and
// WHICH marker it is about, so a caller reading only `marker` cannot be left
// guessing at RmarkerRx by accident.
inline void reject(RangeAdmission& r,
                   RangeAdmissionReason reason,
                   std::string detail,
                   RangeTimeSide side,
                   TimestampMarker marker,
                   uint32_t missing = 0u)
{
    r.admitted = false;
    r.reason = reason;
    r.status = range_admission_reason_to_exchange_status(reason);
    r.value.reset();
    r.side = side;
    r.marker = marker;
    r.missing_correction = missing;
    r.detail = std::move(detail);
}

} // namespace detail

// The single-timestamp gate.  `out.value` is engaged if and only if
// `out.admitted`; every rejection leaves it empty, so a caller that ignores
// `admitted` still cannot read a distance.
inline RangeAdmission admit_range_capable_time(const Timestamp& ts,
                                               const RangeAdmissionContext& ctx)
{
    RangeAdmission r;

    // --- 1. REQ-TIME-01: the timestamp must be a well-formed instant. -----
    if (!timestamp_is_self_consistent(ts)) {
        detail::reject(r, RangeAdmissionReason::InvalidTimestamp,
                       std::string("timestamp_rejection_reason=") +
                           timestamp_rejection_reason(ts),
                       RangeTimeSide::Subject, ts.marker);
        return r;
    }

    // --- 2. REQ-TIME-02: the marker must denote an on-air instant. --------
    // Checked BEFORE the correction bits, and independently of them: a raw
    // capture coordinate is not an RMARKER however many stages claim to have
    // run, which is the R6 point.
    if (!detail::tof_marker_is_range_capable(ts.marker)) {
        detail::reject(r, RangeAdmissionReason::NotRangeCapableMarker,
                       std::string("marker=") + timestamp_marker_to_string(ts.marker) +
                           " class=" + timestamp_marker_class_to_string(
                                           timestamp_marker_class(ts.marker)) +
                           " is not an on-air instant",
                       RangeTimeSide::Subject, ts.marker);
        return r;
    }

    // --- 3. REQ-TIME-03: a real measurement, or an EVIDENCED schedule. ----
    //
    // This step used to refuse every source but `HardwareMeasured`, which also
    // refused the deterministic timed TX that REQ-TIME-03 explicitly PERMITS
    // ("可通过确定性时序加标定取得合格 TX timestamp，但必须标注
    // scheduled_calibrated 来源...不能冒充 hardware_measured").  The forbidden
    // thing is presenting a schedule AS a measurement, not using a calibrated
    // schedule -- and the planned X410 timed-TX route depends on the permitted
    // case, so refusing it would have forced the pipeline to mislabel its own
    // timestamps to make progress.
    //
    // The rule is therefore split by marker class:
    //   * a RECEIVE instant must be hardware-measured.  A scheduled receive
    //     time is a PREDICTION, which is the R6 point and stays refused;
    //   * a TRANSMIT RMARKER may be `ScheduledCalibrated`, but only with the
    //     send evidence REQ-TIME-03 lists.  It is never relabelled: the
    //     admitted value keeps source == ScheduledCalibrated, so a caller can
    //     never mistake a planned instant for a chip timestamp.
    //
    // A planned instant may be STORED before the exchange converges, but it
    // must not become a valid distance until the outcome is known; that is why
    // `Unknown` is a refusal here rather than a "probably fine".
    if (ts.source == TimestampSource::ScheduledCalibrated &&
        timestamp_marker_class(ts.marker) == TimestampMarkerClass::WaveformTx) {
        const TxSendEvidence* ev = ctx.tx_evidence;
        if (ev == nullptr) {
            detail::reject(
                r, RangeAdmissionReason::ScheduledTxEvidenceMissing,
                "a scheduled TX needs TxSendEvidence in the admission context: "
                "send() returning is not evidence that the burst happened as planned",
                RangeTimeSide::Subject, ts.marker);
            return r;
        }
        const uint32_t missing = ev->missing_plan_record();
        if (missing != 0u) {
            uint32_t first = 0u;
            for (uint32_t bit = 1u; bit != 0u; bit <<= 1) {
                if ((missing & bit) != 0u) {
                    first = bit;
                    break;
                }
            }
            detail::reject(
                r, RangeAdmissionReason::ScheduledTxEvidenceMissing,
                std::string("the scheduled TX plan is not fully recorded: missing=") +
                    tx_plan_record_name(first) + " (mask=" +
                    std::to_string(missing) + ", " + ev->to_string() +
                    "); send_accepted alone is never sufficient",
                RangeTimeSide::Subject, ts.marker);
            return r;
        }
        switch (ev->outcome) {
        case TxOutcome::Completed:
            break; // the burst completed as planned
        case TxOutcome::Late:
            detail::reject(r, RangeAdmissionReason::TxOutcomeLate,
                           "the timed TX missed its deadline (ERR_TX_LATE_DATA): the "
                           "frame was not sent at the planned instant",
                           RangeTimeSide::Subject, ts.marker);
            return r;
        case TxOutcome::Underflow:
            detail::reject(r, RangeAdmissionReason::TxOutcomeUnderflow,
                           "the TX stream underran: the frame is not on the air",
                           RangeTimeSide::Subject, ts.marker);
            return r;
        case TxOutcome::Cancelled:
            detail::reject(r, RangeAdmissionReason::TxOutcomeCancelled,
                           "the TX burst was cancelled before it completed",
                           RangeTimeSide::Subject, ts.marker);
            return r;
        case TxOutcome::Unknown:
            detail::reject(r, RangeAdmissionReason::ScheduledTxEvidenceMissing,
                           std::string("the TX outcome is Unknown (") + ev->to_string() +
                               "): send_accepted does not establish that the burst "
                               "completed, and a planned instant is not a measurement "
                               "until it has",
                           RangeTimeSide::Subject, ts.marker);
            return r;
        }
    } else if (ts.source != TimestampSource::HardwareMeasured) {
        const bool scheduled_rx =
            ts.source == TimestampSource::ScheduledCalibrated;
        detail::reject(
            r, RangeAdmissionReason::NotMeasured,
            std::string("source=") + timestamp_source_to_string(ts.source) +
                " marker=" + timestamp_marker_to_string(ts.marker) +
                (scheduled_rx
                     ? " is a scheduled RECEIVE time: a prediction, not a measured "
                       "receive instant"
                     : " is not a hardware measurement"),
            RangeTimeSide::Subject, ts.marker);
        return r;
    }

    // --- 4. REQ-TIME-02/03: every correction the marker requires. --------
    // The refusal names the FIRST missing stage, so the pipeline is told what
    // to run rather than just that something is wrong.
    if (!timestamp_corrections_satisfied(ts.marker, ts.applied_corrections)) {
        const uint32_t need = timestamp_required_corrections(ts.marker);
        uint32_t missing = 0u;
        for (uint32_t bit = 1u; bit != 0u; bit <<= 1) {
            if ((need & bit) != 0u && (ts.applied_corrections & bit) == 0u) {
                missing = bit;
                break;
            }
        }
        detail::reject(r, RangeAdmissionReason::MissingCorrection,
                       std::string("missing=") + timestamp_correction_name(missing) +
                           " marker=" + timestamp_marker_to_string(ts.marker) +
                           " applied=" + timestamp_corrections_to_string(
                                            ts.applied_corrections),
                       RangeTimeSide::Subject, ts.marker, missing);
        return r;
    }

    // --- 5. REQ-CAL-01: present, exactly once, current. ------------------
    {
        if (ts.calibration_id.empty()) {
            detail::reject(r, RangeAdmissionReason::NoCalibration,
                           "timestamp carries no calibration_id",
                           RangeTimeSide::Subject, ts.marker);
            return r;
        }
        if (ctx.calibration == nullptr) {
            detail::reject(r, RangeAdmissionReason::NoCalibration,
                           "no calibration set supplied in the admission context",
                           RangeTimeSide::Subject, ts.marker);
            return r;
        }
        const CalibrationStamp& cal = *ctx.calibration;
        if (cal.id.empty()) {
            detail::reject(r, RangeAdmissionReason::NoCalibration,
                           "the calibration set in force has an empty id",
                           RangeTimeSide::Subject, ts.marker);
            return r;
        }
        // A zero-length or inverted window cannot show the calibration to be
        // current for any instant, so it is reported as STALENESS rather than
        // as an absent calibration: the set exists, it just does not cover
        // this exchange.
        if (!cal.is_well_formed() || !detail::calibration_window_fits_domain(ts.domain, cal)) {
            detail::reject(r, RangeAdmissionReason::CalibrationStale,
                           "calibration validity window is degenerate or does not fit the "
                           "clock domain",
                           RangeTimeSide::Subject, ts.marker);
            return r;
        }
        if (!ctx.reference_ticks_recorded) {
            detail::reject(r, RangeAdmissionReason::NoCalibration,
                           "no reference instant recorded: calibration currency cannot be decided",
                           RangeTimeSide::Subject, ts.marker);
            return r;
        }
        // Applied exactly once, by this set, with no conflict.
        if (cal.applications.size() != 1u) {
            detail::reject(
                r, cal.applications.empty() ? RangeAdmissionReason::NoCalibration
                                           : RangeAdmissionReason::CalibrationAppliedTwice,
                "calibration stage records " + std::to_string(cal.applications.size()) +
                    " applications; exactly one Applied is required",
                RangeTimeSide::Subject, ts.marker);
            return r;
        }
        const CalibrationApplication& app = cal.applications.front();
        if (app.result != CalibrationResult::Applied) {
            detail::reject(r, RangeAdmissionReason::CalibrationAppliedTwice,
                           std::string("calibration stage returned ") +
                               calibration_result_to_string(app.result) +
                               "; the calibration was not applied exactly once",
                           RangeTimeSide::Subject, ts.marker);
            return r;
        }
        if (app.calibration_id != cal.id) {
            detail::reject(r, RangeAdmissionReason::CalibrationAppliedTwice,
                           "calibration stage record names a different set: " +
                               app.calibration_id + " vs " + cal.id,
                           RangeTimeSide::Subject, ts.marker);
            return r;
        }
        // The timestamp must be the one THIS set was applied to.
        if (ts.calibration_id != cal.id) {
            detail::reject(r, RangeAdmissionReason::CalibrationMismatch,
                           "timestamp calibration_id=" + ts.calibration_id +
                               " but the exchange runs on " + cal.id,
                           RangeTimeSide::Subject, ts.marker);
            return r;
        }
        // Currency: same epoch (a reboot invalidates the old constant) and
        // the exchange's own instant inside the window.
        if (cal.calibrated_epoch != ts.domain.epoch_id) {
            char buf[192];
            std::snprintf(buf, sizeof(buf),
                          "calibration %s was established in epoch %llu, timestamp is in "
                          "epoch %llu",
                          cal.id.c_str(),
                          static_cast<unsigned long long>(cal.calibrated_epoch),
                          static_cast<unsigned long long>(ts.domain.epoch_id));
            detail::reject(r, RangeAdmissionReason::CalibrationStale, std::string(buf),
                           RangeTimeSide::Subject, ts.marker);
            return r;
        }
        if (ctx.reference_ticks < cal.valid_from_ticks ||
            ctx.reference_ticks >= cal.valid_until_ticks) {
            char buf[256];
            std::snprintf(buf, sizeof(buf),
                          "calibration %s valid on [%lld,%lld) but the exchange instant is "
                          "%lld",
                          cal.id.c_str(), static_cast<long long>(cal.valid_from_ticks),
                          static_cast<long long>(cal.valid_until_ticks),
                          static_cast<long long>(ctx.reference_ticks));
            detail::reject(r, RangeAdmissionReason::CalibrationStale, std::string(buf),
                           RangeTimeSide::Subject, ts.marker);
            return r;
        }
    }

    // --- 6. REQ-TIME-05: the first-path quality DECISION. ----------------
    // A default-constructed `FirstPathQuality` is `NotRecorded` and is
    // refused, so "we never ran the gate" cannot be read as "the gate passed".
    if (detail::tof_marker_requires_first_path(ts.marker)) {
        if (!ctx.rx_first_path.is_recorded()) {
            detail::reject(r, RangeAdmissionReason::FirstPathQualityNotRecorded,
                           "no first-path quality decision recorded for marker " +
                               std::string(timestamp_marker_to_string(ts.marker)),
                           RangeTimeSide::Subject, ts.marker);
            return r;
        }
        if (!ctx.rx_first_path.measurements_are_sane()) {
            detail::reject(r, RangeAdmissionReason::FirstPathQualityInvalid,
                           "first-path quality decision carries unusable measurements",
                           RangeTimeSide::Subject, ts.marker);
            return r;
        }
        if (!ctx.rx_first_path.has_passed()) {
            detail::reject(r, RangeAdmissionReason::FirstPathUnreliable,
                           "first-path quality gate failed: " + ctx.rx_first_path.to_string(),
                           RangeTimeSide::Subject, ts.marker);
            return r;
        }
    }

    // --- admitted --------------------------------------------------------
    r.admitted = true;
    r.reason = RangeAdmissionReason::Admitted;
    r.status = ExchangeStatus::Ok;
    r.side = RangeTimeSide::Subject;
    r.marker = ts.marker;
    r.missing_correction = 0u;
    r.detail.clear();
    r.value = RangeCapableTime(ts, detail::tof_marker_requires_first_path(ts.marker)
                                     ? ctx.rx_first_path
                                     : FirstPathQuality::not_recorded());
    return r;
}

// The pair gate, and M1-A's entry point.
//
// Check order, and why:
//   1. both numbers are structurally valid        -- a property of the
//      timestamps alone;
//   2. they are on ONE timeline                   -- likewise, and it is
//      checked BEFORE the evidence-dependent steps so that a reboot or a
//      second device is reported as exactly that, instead of surfacing as a
//      calibration or first-path complaint that is only a CONSEQUENCE of it;
//   3. each side is admitted on its own           -- marker, source,
//      corrections, calibration, first path;
//   4. the marker PAIR denotes a protocol interval -- REQ-TIME-02;
//   5. the interval itself is formable            -- REQ-TIME-04; wrap
//      ambiguity and friends surface as
//      `IntervalNotFormable` with the underlying `TimeIntervalStatus`
//      attached.
//
// The value handed back is an `AdmittedRangingInterval` whose `interval()` is
// a `RelativeTickInterval`: exact tick-space rationals with no nanosecond
// field, so M1-A cannot consume a truncated projection even by mistake.
inline RangingIntervalAdmission admit_ranging_interval(const Timestamp& later,
                                                      const Timestamp& earlier,
                                                      const RangeAdmissionContext& ctx)
{
    RangingIntervalAdmission r;

    // --- 1. structural validity of both numbers --------------------------
    if (!timestamp_is_self_consistent(later) || !timestamp_is_self_consistent(earlier)) {
        const bool later_ok = timestamp_is_self_consistent(later);
        const Timestamp& bad = later_ok ? earlier : later;
        r.admitted = false;
        r.reason = RangeAdmissionReason::InvalidTimestamp;
        r.status = range_admission_reason_to_exchange_status(r.reason);
        r.side = later_ok ? RangeTimeSide::Earlier : RangeTimeSide::Later;
        r.marker = bad.marker;
        r.detail = std::string("timestamp_rejection_reason=") +
                   timestamp_rejection_reason(bad);
        return r;
    }

    // --- 2. REQ-TIME-01: one timeline. ----------------------------------
    // Reuses the header's comparability rule (`ClockDomain` identity plus
    // epoch) -- there is deliberately no second notion of domain identity in
    // this file, and no cross-domain conversion is offered.
    if (!clock_domain_is_comparable(later.domain, earlier.domain)) {
        const bool same_epoch = clock_domain_same_epoch(later.domain, earlier.domain);
        r.admitted = false;
        r.side = RangeTimeSide::Earlier;
        r.marker = earlier.marker;
        r.reason = same_epoch ? RangeAdmissionReason::CrossDomain
                              : RangeAdmissionReason::WrongEpoch;
        r.status = range_admission_reason_to_exchange_status(r.reason);
        r.interval_status = same_epoch ? TimeIntervalStatus::DomainNameMismatch
                                       : TimeIntervalStatus::EpochMismatch;
        r.detail = same_epoch ? std::string("not one clock domain: ") +
                                    later.domain.to_string() + " vs " +
                                    earlier.domain.to_string()
                              : std::string("different epoch: ") +
                                    std::to_string(later.domain.epoch_id) + " vs " +
                                    std::to_string(earlier.domain.epoch_id);
        return r;
    }

    // --- 3. each side on its own ----------------------------------------
    const RangeAdmission a_later = admit_range_capable_time(later, ctx);
    if (!a_later.admitted) {
        r.admitted = false;
        r.reason = a_later.reason;
        r.status = a_later.status;
        r.side = RangeTimeSide::Later;
        r.marker = a_later.marker;
        r.missing_correction = a_later.missing_correction;
        r.detail = std::string("later: ") + a_later.detail;
        return r;
    }
    const RangeAdmission a_earlier = admit_range_capable_time(earlier, ctx);
    if (!a_earlier.admitted) {
        r.admitted = false;
        r.reason = a_earlier.reason;
        r.status = a_earlier.status;
        r.side = RangeTimeSide::Earlier;
        r.marker = a_earlier.marker;
        r.missing_correction = a_earlier.missing_correction;
        r.detail = std::string("earlier: ") + a_earlier.detail;
        return r;
    }

    // REQ-TIME-01: one timeline.  Reuses the header's comparability rule
    // (`ClockDomain` identity + epoch) -- there is deliberately no second
    // notion of domain identity in this file.
    if (!clock_domain_is_comparable(later.domain, earlier.domain)) {
        const bool same_epoch = clock_domain_same_epoch(later.domain, earlier.domain);
        r.admitted = false;
        r.side = RangeTimeSide::Earlier;
        r.marker = earlier.marker;
        r.reason = same_epoch ? RangeAdmissionReason::CrossDomain
                              : RangeAdmissionReason::WrongEpoch;
        r.status = range_admission_reason_to_exchange_status(r.reason);
        r.interval_status =
            same_epoch ? TimeIntervalStatus::DomainNameMismatch
                       : TimeIntervalStatus::EpochMismatch;
        r.detail = same_epoch
                       ? std::string("not one clock domain: ") + later.domain.to_string() +
                             " vs " + earlier.domain.to_string()
                       : std::string("different epoch: ") +
                             std::to_string(later.domain.epoch_id) + " vs " +
                             std::to_string(earlier.domain.epoch_id);
        return r;
    }

    // --- 4. REQ-TIME-02: the marker PAIR must denote a protocol interval. -
    // Both sides can individually be on-air instants and still not be
    // subtractable (e.g. the antenna plane minus the protocol RMARKER is a
    // bookkeeping offset, not a propagation time).
    if (!timestamp_marker_interval_allowed(later.marker, earlier.marker)) {
        r.admitted = false;
        r.reason = RangeAdmissionReason::MarkerPairNotAllowed;
        r.status = range_admission_reason_to_exchange_status(r.reason);
        r.side = RangeTimeSide::Later;
        r.marker = later.marker;
        r.interval_status = TimeIntervalStatus::MarkerMismatch;
        r.detail = std::string("marker pair (") + timestamp_marker_to_string(later.marker) +
                   ", " + timestamp_marker_to_string(earlier.marker) +
                   ") does not denote a protocol interval";
        return r;
    }

    // --- 5. REQ-TIME-04: the interval must actually be formable (wrap
    // ambiguity, modular order, exact sub-tick combination).
    // `timestamp_relative_interval()` applies the same gate as
    // `timestamp_interval()` and returns the exact tick-space value with no
    // nanosecond projection.
    RelativeTickInterval ri;
    TimeIntervalStatus istatus = TimeIntervalStatus::InvalidTimestamp;
    if (!timestamp_relative_interval(later, earlier, ri, istatus)) {
        r.admitted = false;
        r.reason = RangeAdmissionReason::IntervalNotFormable;
        r.status = range_admission_reason_to_exchange_status(r.reason);
        r.side = RangeTimeSide::Unknown;
        r.marker = later.marker;
        r.interval_status = istatus;
        r.detail = std::string("interval not formable: ") +
                   time_interval_status_to_string(istatus);
        return r;
    }

    r.admitted = true;
    r.reason = RangeAdmissionReason::Admitted;
    r.status = ExchangeStatus::Ok;
    r.side = RangeTimeSide::Unknown;
    r.marker = later.marker;
    r.missing_correction = 0u;
    r.interval_status = TimeIntervalStatus::Ok;
    r.detail.clear();
    r.value = AdmittedRangingInterval(ri, *a_later.value, *a_earlier.value);
    return r;
}

} // namespace twr
} // namespace uwb
} // namespace gr

#endif /* INCLUDED_GNURADIO_UWB_UWB_TWR_TOF_INPUT_H */
