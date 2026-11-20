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
#ifndef INCLUDED_OPENUWB_TWR_CONFIG_REASONS_H
#define INCLUDED_OPENUWB_TWR_CONFIG_REASONS_H

#include <openuwb/twr/twr_config_util.h>
namespace gr {
namespace uwb {
namespace twr {

// ===========================================================================
// 1. Machine-readable rejection reasons and the validation report
// ===========================================================================

// Every rejection carries one of these.  Values are appended-only (they are
// part of the JSON/CSV schema, REQ-OUT-01).
enum class ConfigReason : uint16_t {
    None = 0,

    // --- value level ------------------------------------------------------
    NotFinite = 1,          // NaN / +Inf / -Inf in a float field
    OutOfRange = 2,         // outside the documented numeric range
    IndexOutOfRange = 3,    // array / code / channel index out of range
    NegativeValue = 4,      // a delay or window may not be negative
    EmptyValue = 5,         // required string / identifier is empty
    OverCapacity = 6,       // queue or diagnostic buffer above its bound
    ZeroValue = 7,          // a field that must be non-zero is zero

    // --- cross-field ------------------------------------------------------
    FieldConflict = 20,     // two fields cannot both be true
    ChannelFrequencyMismatch = 21,
    DuplicateResource = 22, // two endpoints claim one physical channel
    TimingOrderViolation = 23,
    TimingBudgetInfeasible = 24,
    TimingPrecisionInsufficient = 25,
    QuantisationUnsupported = 26,
    UnitMismatch = 27,
    FrameLengthOverflow = 28,

    // --- capability -------------------------------------------------------
    Unsupported = 40,      // not in the capability whitelist
    OutOfScope = 41,       // real, but deliberately out of phase-1 scope
    InFlightNotSupported = 42,

    // --- calibration ------------------------------------------------------
    CalibrationMissing = 60,
    CalibrationExpired = 61,
    CalibrationAlreadyApplied = 62,
    CalibrationMismatch = 63,

    // --- lifecycle / mutation ---------------------------------------------
    MidExchangeMutation = 80,
    ExchangeAlreadyInFlight = 81,

    // --- JSON import ------------------------------------------------------
    MalformedJson = 100,
    UnknownKey = 101,
    MissingKey = 102,
    TypeMismatch = 103,
    UnknownEnumValue = 104,
    IntegerPrecisionLoss = 105
};

inline const char* config_reason_to_string(ConfigReason r)
{
    switch (r) {
    case ConfigReason::None:
        return "none";
    case ConfigReason::NotFinite:
        return "not_finite";
    case ConfigReason::OutOfRange:
        return "out_of_range";
    case ConfigReason::IndexOutOfRange:
        return "index_out_of_range";
    case ConfigReason::NegativeValue:
        return "negative_value";
    case ConfigReason::EmptyValue:
        return "empty_value";
    case ConfigReason::OverCapacity:
        return "over_capacity";
    case ConfigReason::ZeroValue:
        return "zero_value";
    case ConfigReason::FieldConflict:
        return "field_conflict";
    case ConfigReason::ChannelFrequencyMismatch:
        return "channel_frequency_mismatch";
    case ConfigReason::DuplicateResource:
        return "duplicate_resource";
    case ConfigReason::TimingOrderViolation:
        return "timing_order_violation";
    case ConfigReason::TimingBudgetInfeasible:
        return "timing_budget_infeasible";
    case ConfigReason::TimingPrecisionInsufficient:
        return "timing_precision_insufficient";
    case ConfigReason::QuantisationUnsupported:
        return "quantisation_unsupported";
    case ConfigReason::UnitMismatch:
        return "unit_mismatch";
    case ConfigReason::FrameLengthOverflow:
        return "frame_length_overflow";
    case ConfigReason::Unsupported:
        return "unsupported";
    case ConfigReason::OutOfScope:
        return "out_of_scope";
    case ConfigReason::InFlightNotSupported:
        return "in_flight_not_supported";
    case ConfigReason::CalibrationMissing:
        return "calibration_missing";
    case ConfigReason::CalibrationExpired:
        return "calibration_expired";
    case ConfigReason::CalibrationAlreadyApplied:
        return "calibration_already_applied";
    case ConfigReason::CalibrationMismatch:
        return "calibration_mismatch";
    case ConfigReason::MidExchangeMutation:
        return "mid_exchange_mutation";
    case ConfigReason::ExchangeAlreadyInFlight:
        return "exchange_already_in_flight";
    case ConfigReason::MalformedJson:
        return "malformed_json";
    case ConfigReason::UnknownKey:
        return "unknown_key";
    case ConfigReason::MissingKey:
        return "missing_key";
    case ConfigReason::TypeMismatch:
        return "type_mismatch";
    case ConfigReason::UnknownEnumValue:
        return "unknown_enum_value";
    case ConfigReason::IntegerPrecisionLoss:
        return "integer_precision_loss";
    }
    return "invalid";
}

inline bool config_reason_from_string(const std::string& s, ConfigReason& out)
{
    for (int i = 0; i <= static_cast<int>(ConfigReason::IntegerPrecisionLoss); ++i) {
        ConfigReason c = static_cast<ConfigReason>(i);
        if (s == config_reason_to_string(c)) {
            out = c;
            return true;
        }
    }
    return false;
}

// One rejection.  `status` is the frozen ExchangeStatus the caller will see,
// so a configuration failure and a runtime failure of the same family are
// reported with the same machine-readable code (REQ-ERR-01).
struct ConfigViolation {
    std::string field;   // dotted path, e.g. "phy.code_index"
    ConfigReason reason = ConfigReason::None;
    ExchangeStatus status = ExchangeStatus::ConfigRejected;
    std::string message;  // human-readable detail
    std::string requirement; // "REQ-API-01", ...

    bool operator==(const ConfigViolation& o) const
    {
        return field == o.field && reason == o.reason && status == o.status &&
               message == o.message && requirement == o.requirement;
    }
};

inline std::string config_violation_to_string(const ConfigViolation& v)
{
    std::string s = v.field.empty() ? std::string("<config>") : v.field;
    s += ": ";
    s += config_reason_to_string(v.reason);
    s += " [";
    s += exchange_status_to_string(v.status);
    s += "]";
    if (!v.message.empty()) {
        s += " ";
        s += v.message;
    }
    if (!v.requirement.empty()) {
        s += " (";
        s += v.requirement;
        s += ")";
    }
    return s;
}

// The full report.  `ok()` is never the only information a caller gets.
struct ValidationReport {
    bool valid = true;
    std::vector<ConfigViolation> violations;

    bool ok() const { return valid; }
    bool has(ConfigReason r) const
    {
        for (const auto& v : violations)
            if (v.reason == r)
                return true;
        return false;
    }
    bool has(const std::string& field, ConfigReason r) const
    {
        return find(field, r) != nullptr;
    }
    bool has_field(const std::string& path) const
    {
        for (const auto& v : violations)
            if (v.field == path)
                return true;
        return false;
    }
    const ConfigViolation* find(const std::string& path, ConfigReason r) const
    {
        for (const auto& v : violations)
            if (v.field == path && v.reason == r)
                return &v;
        return nullptr;
    }
    const ConfigViolation* first_for_field(const std::string& path) const
    {
        for (const auto& v : violations)
            if (v.field == path)
                return &v;
        return nullptr;
    }
    // First non-Ok status, in report order.  Never a bare bool.
    ExchangeStatus first_status() const
    {
        if (violations.empty())
            return ExchangeStatus::Ok;
        return violations.front().status;
    }
    std::vector<std::string> messages() const
    {
        std::vector<std::string> out;
        out.reserve(violations.size());
        for (const auto& v : violations)
            out.push_back(config_violation_to_string(v));
        return out;
    }
    std::string to_string() const
    {
        if (violations.empty())
            return "ok";
        std::string s;
        for (size_t i = 0; i < violations.size(); ++i) {
            if (i)
                s += "; ";
            s += config_violation_to_string(violations[i]);
        }
        return s;
    }
    void add(const ConfigViolation& v)
    {
        valid = false;
        violations.push_back(v);
    }
    void add(const std::string& field,
             ConfigReason reason,
             ExchangeStatus status,
             const std::string& message,
             const char* requirement)
    {
        ConfigViolation v;
        v.field = field;
        v.reason = reason;
        v.status = status;
        v.message = message;
        v.requirement = requirement;
        add(v);
    }
};

} // namespace twr
} // namespace uwb
} // namespace gr

#endif /* INCLUDED_OPENUWB_TWR_CONFIG_REASONS_H */
