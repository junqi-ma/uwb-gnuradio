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
#ifndef INCLUDED_OPENUWB_TWR_CONFIG_EFFECTIVE_H
#define INCLUDED_OPENUWB_TWR_CONFIG_EFFECTIVE_H

#include <openuwb/twr/twr_config_validator.h>
namespace gr {
namespace uwb {
namespace twr {

// ===========================================================================
// 7. The frozen effective config
// ===========================================================================

// A frozen, self-describing snapshot (REQ-API-01: 完整保存 requested、effective、
// 硬件 readback).  It is a VALUE: an exchange in flight holds a copy, so a
// later change to the requested config cannot reach it (REQ-API-03).
struct EffectiveConfig {
    bool ok = false;
    ValidationReport validation;

    std::string schema_version;
    std::string profile_version;
    std::string calibration_version;
    std::string config_hash;

    TwrConfig requested;
    TwrConfig effective;
    std::vector<FieldChange> changes; // requested vs effective, per field

    // The hardware readback exactly as recorded at startup.
    RadioReadback readback;

    // Quantised timing actually handed to the device.  The REQUESTED delay and
    // the EFFECTIVE (tick-quantised) delay are both available so REQ-GR-04 can
    // report the margin.
    int64_t poll_to_response_ticks = 0;
    int64_t response_to_final_ticks = 0;
    int64_t post_tx_rx_enable_ticks = 0;
    int64_t poll_start_ticks = 0;
    Duration poll_to_response_effective{0};
    Duration response_to_final_effective{0};
    Duration post_tx_rx_enable_effective{0};
    Duration poll_start_effective{0};
    double tick_rate_hz = 0.0;

    // Frame budget, per frame type, as transmitted (including the FCS exactly
    // once).  These are the CODEC's numbers, not a re-summation of the
    // config's geometry claim: see effective_config().
    uint16_t poll_bytes = 0;
    uint16_t response_bytes = 0;
    uint16_t final_bytes = 0;
    uint16_t max_psdu_bytes = 0;
    uint32_t max_timestamp_count = 0;
    // The PHR of this profile, recorded so no consumer recomputes it: the
    // number of INFORMATION bytes and the number of transmitted coded bits.
    uint16_t phr_bytes = 0;
    uint16_t phr_coded_bits = 0;
};

// Build the frozen snapshot.  On failure `ok == false`, `validation` holds
// every reason, and `effective` is a value-initialised (NOT a default
// substituted) config: there is no way to accidentally use it.
inline EffectiveConfig
effective_config(const TwrConfig& cfg, const Capabilities& caps)
{
    EffectiveConfig out;
    out.requested = cfg;
    out.validation = validate(cfg, caps);
    out.ok = out.validation.ok();
    out.schema_version = caps.schema_version;
    out.profile_version = cfg.meta.profile_version;
    out.calibration_version = cfg.meta.calibration_version;
    if (!out.ok) {
        // A rejected config produces no effective values at all.  The hash of
        // the REQUESTED config is still recorded so a failure can be tied to
        // the exact input that caused it (REQ-OUT-01).
        out.effective = TwrConfig{};
        out.config_hash = config_hash(cfg);
        out.changes.clear();
        return out;
    }

    // Effective == requested plus the materialisations the validator proved
    // legal: nothing is invented, everything is recorded.
    out.effective = cfg;
    out.config_hash = config_hash(out.effective);
    out.readback = cfg.radio.readback;
    out.changes = diff_fields(out.requested, out.effective);
    out.max_psdu_bytes = caps.max_psdu_bytes();
    out.max_timestamp_count = frame_type_timestamp_count(FrameType::Final);

    const double rate = caps.native_rate_supported(cfg.radio.native_sample_rate_hz)
                            ? cfg.radio.native_sample_rate_hz
                            : caps.native_rates_hz.front();
    out.tick_rate_hz = rate;

    struct Q {
        const TimedField* in;
        int64_t* ticks;
        Duration* eff;
    } q[] = {
        { &cfg.timing.poll_start, &out.poll_start_ticks, &out.poll_start_effective },
        { &cfg.timing.poll_to_response, &out.poll_to_response_ticks,
          &out.poll_to_response_effective },
        { &cfg.timing.response_to_final, &out.response_to_final_ticks,
          &out.response_to_final_effective },
        { &cfg.timing.post_tx_rx_enable, &out.post_tx_rx_enable_ticks,
          &out.post_tx_rx_enable_effective },
    };
    for (const auto& item : q) {
        if (item.in->domain != TimeDomain::DeviceTicks)
            continue;
        bool ok = false;
        const int64_t ticks =
            quantise_duration(item.in->value, item.in->required_quantisation_hz, ok);
        *item.ticks = ok ? ticks : 0;
        Duration back;
        *item.eff = (ok && Duration::from_ticks(ticks, item.in->required_quantisation_hz, back))
                        ? back
                        : Duration{0};
        if (back != item.in->value) {
            // Record the quantisation as an explicit requested/effective
            // difference; the device gets the tick value, the operator sees both.
            FieldChange ch;
            ch.path = item.in->value.nanos() != 0 ? "timing.quantised_delay" : "timing.delay";
            ch.requested = twr_duration_to_text(item.in->value);
            ch.effective = twr_duration_to_text(back);
            ch.note = "quantised at " + twr_double_to_text(item.in->required_quantisation_hz) +
                      " Hz (ticks=" + twr_int_to_text(ticks) + ")";
            out.changes.push_back(ch);
        }
    }

    // The frame budget is the CODEC's, computed once through the authority.
    // M0 re-summed the config's own geometry claim here, which is how a 7-byte
    // header produced a 9-byte Poll and understated the on-air length; and it
    // added the FCS from a raw config field, so a wrong fcs_bytes silently
    // changed the answer.  `on_air_bytes()` is invariant under which layer
    // appends the FCS, which is exactly the property the budget should have.
    FrameProfileGeometry authority;
    FrameProfile profile;
    if (frame_geometry_for(cfg.frame.frame_profile, authority) &&
        frame_profile_for(cfg.frame.frame_profile, profile)) {
        out.poll_bytes = static_cast<uint16_t>(authority.on_air_bytes(FrameType::Poll, profile));
        out.response_bytes =
            static_cast<uint16_t>(authority.on_air_bytes(FrameType::Response, profile));
        out.final_bytes =
            static_cast<uint16_t>(authority.on_air_bytes(FrameType::Final, profile));
        // The PHR is present in every frame of this profile; recording its
        // information size here means a consumer never has to recompute it.
        out.phr_bytes = static_cast<uint16_t>(authority.phr_bytes());
        out.phr_coded_bits = authority.phr_coded_bits();
    }
    return out;
}

inline EffectiveConfig effective_config(const TwrConfig& cfg)
{
    return effective_config(cfg, capabilities());
}

// ---------------------------------------------------------------------------
// In-flight gate (phase 1: at most one exchange per endpoint)
// ---------------------------------------------------------------------------

// Enforces "同端初版一次仅一个在途 exchange" at RUN time, next to the config
// that promised it.  A second concurrent request is QueueFull, never a
// silently dropped or queued-forever exchange.
class ExchangeGate
{
public:
    explicit ExchangeGate(uint32_t max_in_flight = kTwrMaxInFlightExchanges)
        : d_max(max_in_flight == 0 ? 1u : max_in_flight)
    {
    }

    ValidationReport try_begin()
    {
        ValidationReport r;
        if (d_in_flight >= d_max) {
            ConfigViolation v;
            v.field = "session.max_in_flight_exchanges";
            v.reason = ConfigReason::ExchangeAlreadyInFlight;
            v.status = ExchangeStatus::QueueFull;
            v.message = "an exchange is already in flight; at most " + twr_int_to_text(d_max) +
                        " may be outstanding";
            v.requirement = "REQ-API-01";
            r.add(v);
            // A REFUSED request must not consume a slot, or a run of refusals
            // would deadlock the endpoint.
            return r;
        }
        ++d_in_flight;
        return r;
    }
    void end()
    {
        if (d_in_flight > 0)
            --d_in_flight;
    }
    uint32_t in_flight() const { return d_in_flight; }

private:
    uint32_t d_max;
    uint32_t d_in_flight = 0;
};

// ---------------------------------------------------------------------------
// REQ-API-03: per-message / per-exchange overrides on an immutable snapshot
// ---------------------------------------------------------------------------

// Only the values a single exchange may re-negotiate.  Anything that affects
// synchronisation (PHY, frequency, gain, addresses, calibration) is
// deliberately absent: it may only change at a session boundary, and the
// snapshot an exchange already holds is immutable (REQ-API-03).
struct PerMessageOverrides {
    // 0 = "not overridden"; the validator then requires the base config value.
    int64_t measurement_count = 0;
    int64_t measurement_interval_ns = 0;
    int64_t poll_to_response_ns = 0;
    int64_t response_to_final_ns = 0;
    int64_t post_tx_rx_enable_ns = 0;
    int64_t rx_timeout_ns = 0;
    int64_t exchange_timeout_ns = 0;

    bool any() const
    {
        return measurement_count != 0 || measurement_interval_ns != 0 ||
               poll_to_response_ns != 0 || response_to_final_ns != 0 ||
               post_tx_rx_enable_ns != 0 || rx_timeout_ns != 0 ||
                   exchange_timeout_ns != 0;
    }
};

// The frozen snapshot plus the exchange gate.  Copying it is the only way to
// hand an immutable config to a worker; there is no mutable accessor.
class TwrConfigSnapshot
{
public:
    TwrConfigSnapshot() : d_effective(), d_caps(&capabilities()) {}
    TwrConfigSnapshot(const EffectiveConfig& e, const Capabilities& caps)
        : d_effective(e), d_caps(&caps)
    {
    }

    const EffectiveConfig& get() const { return d_effective; }
    bool ok() const { return d_effective.ok; }

    uint32_t in_flight() const { return d_gate.in_flight(); }

    // Begin an exchange.  Returns Ok when the gate admitted it.
    ValidationReport begin_exchange() { return d_gate.try_begin(); }
    void end_exchange() { d_gate.end(); }

    // REQ-API-03: overrides take effect at an exchange BOUNDARY.  A call
    // while an exchange is in flight is rejected -- the running exchange keeps
    // the snapshot it started with.
    ValidationReport
    set_overrides(const PerMessageOverrides& o, TwrConfigSnapshot& out) const
    {
        ValidationReport r;
        if (d_gate.in_flight() > 0) {
            ConfigViolation v;
            v.field = "session";
            v.reason = ConfigReason::MidExchangeMutation;
            v.status = ExchangeStatus::ConfigRejected;
            v.message = "an exchange is in flight; overrides may only be applied at an exchange "
                        "boundary and the running exchange keeps its immutable snapshot";
            v.requirement = "REQ-API-03";
            r.add(v);
            return r;
        }
        if (!d_effective.ok) {
            ConfigViolation v;
            v.field = "";
            v.reason = ConfigReason::OutOfScope;
            v.status = ExchangeStatus::ConfigRejected;
            v.message = "the base config is not valid; overrides cannot be applied to it";
            v.requirement = "REQ-API-03";
            r.add(v);
            return r;
        }
        TwrConfig next = d_effective.effective;
        if (o.measurement_count != 0) {
            if (o.measurement_count < 0)
                next.session.measurement_count = 0; // let the validator reject
            else
                next.session.measurement_count = static_cast<uint32_t>(o.measurement_count);
        }
        if (o.measurement_interval_ns != 0)
            next.session.measurement_interval = Duration::from_nanos(o.measurement_interval_ns);
        if (o.poll_to_response_ns != 0)
            next.timing.poll_to_response.value = Duration::from_nanos(o.poll_to_response_ns);
        if (o.response_to_final_ns != 0)
            next.timing.response_to_final.value = Duration::from_nanos(o.response_to_final_ns);
        if (o.post_tx_rx_enable_ns != 0)
            next.timing.post_tx_rx_enable.value = Duration::from_nanos(o.post_tx_rx_enable_ns);
        if (o.rx_timeout_ns != 0)
            next.timeouts.rx_timeout.value = Duration::from_nanos(o.rx_timeout_ns);
        if (o.exchange_timeout_ns != 0)
            next.timeouts.exchange_timeout.value = Duration::from_nanos(o.exchange_timeout_ns);

        out = TwrConfigSnapshot(effective_config(next, *d_caps), *d_caps);
        if (!out.d_effective.ok) {
            r = out.d_effective.validation;
            return r;
        }
        out.d_effective.requested = d_effective.requested; // keep the original
        out.d_effective.changes = diff_fields(d_effective.requested,
            out.d_effective.effective);
        return r;
    }

private:
    EffectiveConfig d_effective;
    const Capabilities* d_caps;
    ExchangeGate d_gate;
};

// ---------------------------------------------------------------------------
// REQ-CAL-01: a calibration is applicable exactly once
// ---------------------------------------------------------------------------

// Apply the configured calibration to a measurement.  Idempotence is
// enforced, not assumed: a second application is rejected so a range can never
// be corrected twice.  `now_monotonic_ns` comes from the host monotonic clock
// and is only used for the expiry check.
inline ExchangeStatus
apply_calibration_once(TimestampCalibrationConfig& cal, int64_t now_monotonic_ns,
    std::string* why)
{
    if (cal.calibration_id.empty()) {
        if (why)
            *why = "no calibration id: an uncalibrated absolute range may not be claimed";
        return ExchangeStatus::CalibrationMissing;
    }
    if (cal.applied_count != 0) {
        if (why)
            *why = "calibration already applied " + twr_int_to_text(cal.applied_count) +
                   " time(s); a calibration is applicable exactly once";
        return ExchangeStatus::CalibrationMissing;
    }
    if (cal.record.valid_until_monotonic_ns != 0 && now_monotonic_ns != 0 &&
        now_monotonic_ns > cal.record.valid_until_monotonic_ns) {
        if (why)
            *why = "calibration " + cal.calibration_id + " expired at " +
                   twr_int_to_text(cal.record.valid_until_monotonic_ns);
        return ExchangeStatus::CalibrationExpired;
    }
    cal.applied_count = 1;
    if (why)
        *why = "calibration " + cal.calibration_id + " applied once";
    return ExchangeStatus::Ok;
}

// ===========================================================================
// 8. JSON import / export (dependency-free)
// ===========================================================================
//
// The repository has NO JSON library dependency (verified: nothing under
// gr-uwb/include or gr-uwb/lib provides one) and this milestone must not add
// one, so a minimal reader/writer for exactly this schema lives here.
//
// SCHEMA NOTES (REQ-OUT-01)
//   * Every value is emitted, including `null` for an absent optional field,
//     so `from_json(to_json(c)) == c` is exact.
//   * 64-bit integers are emitted UNQUOTED when |v| <= 2^53-1 and as a
//     QUOTED DECIMAL STRING otherwise.  The reader accepts both forms for any
//     integer field, so a timestamp beyond the JSON safe range survives a
//     round trip.  A decimal fraction or exponent on an integer field is
//     REJECTED (`type_mismatch`) rather than silently truncated.
//   * Doubles are emitted with %.17g, which round-trips every finite IEEE 754
//     double.  NaN / +Inf / -Inf have no JSON representation and are
//     REJECTED on export, so a corrupt config cannot be serialised into a
//     file that a later run would parse as a valid one.
//   * Durations are emitted as a plain integer number of NANOSECONDS under a
//     `_ns` key.  Native-tick fields carry a `_native_ticks` key and the
//     calibration additionally carries an explicit `link_delay_unit`, so ns,
//     seconds and native ticks can never be confused.
//   * Number parsing uses strtod and therefore expects the C numeric locale,
//     which is what GNU Radio and the Python host run with.
namespace json {

enum class Type { Null, Bool, Int, Double, String, Array, Object };

class Value
{
public:
    Type type = Type::Null;
    bool boolean = false;
    int64_t integer = 0;
    double number = 0.0;
    std::string text;
    std::vector<Value> items;                            // Array
    std::vector<std::pair<std::string, Value>> members;  // Object, ordered

    static Value make_null() { return Value(); }
    static Value make_bool(bool v)
    {
        Value x;
        x.type = Type::Bool;
        x.boolean = v;
        return x;
    }
    static Value make_int(int64_t v)
    {
        Value x;
        x.type = Type::Int;
        x.integer = v;
        return x;
    }
    static Value make_double(double v)
    {
        Value x;
        x.type = Type::Double;
        x.number = v;
        return x;
    }
    static Value make_string(std::string v)
    {
        Value x;
        x.type = Type::String;
        x.text = std::move(v);
        return x;
    }
    static Value make_array()
    {
        Value x;
        x.type = Type::Array;
        return x;
    }
    static Value make_object()
    {
        Value x;
        x.type = Type::Object;
        return x;
    }

    const Value* find(const std::string& key) const
    {
        for (const auto& m : members)
            if (m.first == key)
                return &m.second;
        return nullptr;
    }
    Value* find(const std::string& key)
    {
        for (auto& m : members)
            if (m.first == key)
                return &m.second;
        return nullptr;
    }
    void set(const std::string& key, Value v)
    {
        if (type == Type::Null)
            type = Type::Object;
        for (auto& m : members) {
            if (m.first == key) {
                m.second = std::move(v);
                return;
            }
        }
        members.emplace_back(key, std::move(v));
    }
    void push(Value v)
    {
        if (type == Type::Null)
            type = Type::Array;
        items.push_back(std::move(v));
    }

    bool is_number() const { return type == Type::Int || type == Type::Double; }
    double as_double() const
    {
        return type == Type::Int ? static_cast<double>(integer) : number;
    }
    const char* type_name() const
    {
        switch (type) {
        case Type::Null:
            return "null";
        case Type::Bool:
            return "bool";
        case Type::Int:
            return "integer";
        case Type::Double:
            return "number";
        case Type::String:
            return "string";
        case Type::Array:
            return "array";
        case Type::Object:
            return "object";
        }
        return "invalid";
    }
};

inline constexpr size_t kJsonMaxDepth = 32;
inline constexpr size_t kJsonMaxInputBytes = 4u * 1024u * 1024u;

class Parser
{
public:
    explicit Parser(const std::string& text) : s(text) {}

    bool parse(Value& out, std::string& err)
    {
        if (s.size() > kJsonMaxInputBytes) {
            err = "json input above " + twr_int_to_text(static_cast<int64_t>(kJsonMaxInputBytes)) +
                  " bytes";
            return false;
        }
        skip_ws();
        if (!parse_value(out, err))
            return false;
        skip_ws();
        if (p != s.size()) {
            err = "trailing characters at offset " + twr_int_to_text(static_cast<int64_t>(p));
            return false;
        }
        return true;
    }

private:
    bool fail(const std::string& m, std::string& err)
    {
        err = m + " at offset " + twr_int_to_text(static_cast<int64_t>(p));
        return false;
    }
    void skip_ws()
    {
        while (p < s.size() &&
               (s[p] == ' ' || s[p] == '\t' || s[p] == '\n' || s[p] == '\r'))
            ++p;
    }
    bool parse_value(Value& out, std::string& err)
    {
        if (d_depth >= kJsonMaxDepth)
            return fail("json nesting too deep", err);
        if (p >= s.size())
            return fail("unexpected end of input", err);
        switch (s[p]) {
        case '{':
            return parse_object(out, err);
        case '[':
            return parse_array(out, err);
        case '"': {
            std::string t;
            if (!parse_string(t, err))
                return false;
            out = Value::make_string(std::move(t));
            return true;
        }
        case 't':
            if (s.compare(p, 4, "true") != 0)
                return fail("bad literal", err);
            p += 4;
            out = Value::make_bool(true);
            return true;
        case 'f':
            if (s.compare(p, 5, "false") != 0)
                return fail("bad literal", err);
            p += 5;
            out = Value::make_bool(false);
            return true;
        case 'n':
            if (s.compare(p, 4, "null") != 0)
                return fail("bad literal", err);
            p += 4;
            out = Value::make_null();
            return true;
        default:
            return parse_number(out, err);
        }
    }
    bool parse_object(Value& out, std::string& err)
    {
        ++p; // '{'
        ++d_depth;
        out = Value::make_object();
        skip_ws();
        if (p < s.size() && s[p] == '}') {
            ++p;
            --d_depth;
            return true;
        }
        while (true) {
            skip_ws();
            if (p >= s.size() || s[p] != '"')
                return fail("expected object key", err);
            std::string key;
            if (!parse_string(key, err))
                return false;
            skip_ws();
            if (p >= s.size() || s[p] != ':')
                return fail("expected ':'", err);
            ++p;
            skip_ws();
            Value v;
            if (!parse_value(v, err))
                return false;
            out.set(key, std::move(v));
            skip_ws();
            if (p < s.size() && s[p] == ',') {
                ++p;
                continue;
            }
            if (p < s.size() && s[p] == '}') {
                ++p;
                --d_depth;
                return true;
            }
            return fail("expected ',' or '}'", err);
        }
    }
    bool parse_array(Value& out, std::string& err)
    {
        ++p; // '['
        ++d_depth;
        out = Value::make_array();
        skip_ws();
        if (p < s.size() && s[p] == ']') {
            ++p;
            --d_depth;
            return true;
        }
        while (true) {
            skip_ws();
            Value v;
            if (!parse_value(v, err))
                return false;
            out.push(std::move(v));
            skip_ws();
            if (p < s.size() && s[p] == ',') {
                ++p;
                continue;
            }
            if (p < s.size() && s[p] == ']') {
                ++p;
                --d_depth;
                return true;
            }
            return fail("expected ',' or ']'", err);
        }
    }
    bool parse_string(std::string& out, std::string& err)
    {
        ++p; // opening quote
        out.clear();
        while (p < s.size()) {
            const char c = s[p++];
            if (c == '"')
                return true;
            if (c == '\\') {
                if (p >= s.size())
                    return fail("unterminated escape", err);
                const char e = s[p++];
                switch (e) {
                case '"':
                    out.push_back('"');
                    break;
                case '\\':
                    out.push_back('\\');
                    break;
                case '/':
                    out.push_back('/');
                    break;
                case 'b':
                    out.push_back('\b');
                    break;
                case 'f':
                    out.push_back('\f');
                    break;
                case 'n':
                    out.push_back('\n');
                    break;
                case 'r':
                    out.push_back('\r');
                    break;
                case 't':
                    out.push_back('\t');
                    break;
                case 'u': {
                    if (p + 4 > s.size())
                        return fail("short \\u escape", err);
                    unsigned cp = 0;
                    for (int i = 0; i < 4; ++i) {
                        const char h = s[p + static_cast<size_t>(i)];
                        cp <<= 4;
                        if (h >= '0' && h <= '9')
                            cp |= static_cast<unsigned>(h - '0');
                        else if (h >= 'a' && h <= 'f')
                            cp |= static_cast<unsigned>(h - 'a' + 10);
                        else if (h >= 'A' && h <= 'F')
                            cp |= static_cast<unsigned>(h - 'A' + 10);
                        else
                            return fail("bad \\u escape", err);
                    }
                    p += 4;
                    // UTF-8 encode (surrogate pairs are passed through as-is;
                    // this config only ever carries ASCII identifiers).
                    if (cp < 0x80) {
                        out.push_back(static_cast<char>(cp));
                    } else if (cp < 0x800) {
                        out.push_back(static_cast<char>(0xC0 | (cp >> 6)));
                        out.push_back(static_cast<char>(0x80 | (cp & 0x3F)));
                    } else {
                        out.push_back(static_cast<char>(0xE0 | (cp >> 12)));
                        out.push_back(static_cast<char>(0x80 | ((cp >> 6) & 0x3F)));
                        out.push_back(static_cast<char>(0x80 | (cp & 0x3F)));
                    }
                    break;
                }
                default:
                    return fail("unknown escape", err);
                }
                continue;
            }
            out.push_back(c);
        }
        return fail("unterminated string", err);
    }
    bool parse_number(Value& out, std::string& err)
    {
        const size_t start = p;
        if (p < s.size() && s[p] == '-')
            ++p;
        if (p >= s.size() || s[p] < '0' || s[p] > '9')
            return fail("expected a number", err);
        if (s[p] == '0') {
            ++p;
        } else {
            while (p < s.size() && s[p] >= '0' && s[p] <= '9')
                ++p;
        }
        bool integral = true;
        if (p < s.size() && s[p] == '.') {
            integral = false;
            ++p;
            if (p >= s.size() || s[p] < '0' || s[p] > '9')
                return fail("expected digits after '.'", err);
            while (p < s.size() && s[p] >= '0' && s[p] <= '9')
                ++p;
        }
        if (p < s.size() && (s[p] == 'e' || s[p] == 'E')) {
            integral = false;
            ++p;
            if (p < s.size() && (s[p] == '+' || s[p] == '-'))
                ++p;
            if (p >= s.size() || s[p] < '0' || s[p] > '9')
                return fail("expected digits in exponent", err);
            while (p < s.size() && s[p] >= '0' && s[p] <= '9')
                ++p;
        }
        const std::string tok = s.substr(start, p - start);
        if (integral) {
            errno = 0;
            char* end = nullptr;
            const long long v = std::strtoll(tok.c_str(), &end, 10);
            if (errno == 0 && end != nullptr && *end == '\0') {
                out = Value::make_int(static_cast<int64_t>(v));
                return true;
            }
            // An integer literal outside int64 is kept as a Double so the
            // reader for an integer field can reject it explicitly.
        }
        errno = 0;
        char* end = nullptr;
        const double d = std::strtod(tok.c_str(), &end);
        if (end == nullptr || *end != '\0')
            return fail("malformed number", err);
        out = Value::make_double(d);
        return true;
    }

    const std::string& s;
    size_t p = 0;
    size_t d_depth = 0;
};

inline bool parse(const std::string& text, Value& out, std::string& err)
{
    Parser parser(text);
    return parser.parse(out, err);
}

inline void escape_into(const std::string& in, std::string& out)
{
    out.push_back('"');
    for (unsigned char c : in) {
        switch (c) {
        case '"':
            out += "\\\"";
            break;
        case '\\':
            out += "\\\\";
            break;
        case '\n':
            out += "\\n";
            break;
        case '\r':
            out += "\\r";
            break;
        case '\t':
            out += "\\t";
            break;
        default:
            if (c < 0x20) {
                char buf[8];
                std::snprintf(buf, sizeof(buf), "\\u%04x", static_cast<unsigned>(c));
                out += buf;
            } else {
                out.push_back(static_cast<char>(c));
            }
        }
    }
    out.push_back('"');
}

// 64-bit integer as a JSON number when safe, else as a decimal string.
inline void put_int(int64_t v, std::string& out)
{
    if (v <= kTwrJsonMaxSafeInteger && v >= -kTwrJsonMaxSafeInteger) {
        out += twr_int_to_text(v);
        return;
    }
    escape_into(twr_int_to_text(v), out);
}

// -- dump ------------------------------------------------------------------

class Dumper
{
public:
    Dumper(const Value& v, bool pretty) : d_v(v), d_pretty(pretty) {}

    bool run(std::string& out, std::string& err)
    {
        m_buf.clear();
        if (!dump(d_v, 0, err))
            return false;
        out = m_buf;
        if (d_pretty)
            out += "\n";
        return true;
    }

private:
    void indent(int depth)
    {
        if (!d_pretty)
            return;
        m_buf += "\n";
        m_buf.append(static_cast<size_t>(depth) * 2, ' ');
    }
    bool dump(const Value& v, int depth, std::string& err)
    {
        switch (v.type) {
        case Type::Null:
            m_buf += "null";
            return true;
        case Type::Bool:
            m_buf += v.boolean ? "true" : "false";
            return true;
        case Type::Int:
            put_int(v.integer, m_buf);
            return true;
        case Type::Double:
            if (!std::isfinite(v.number)) {
                err = "non-finite double cannot be represented in JSON";
                return false;
            }
            m_buf += twr_double_to_text(v.number);
            return true;
        case Type::String:
            escape_into(v.text, m_buf);
            return true;
        case Type::Array: {
            if (v.items.empty()) {
                m_buf += "[]";
                return true;
            }
            m_buf += "[";
            for (size_t i = 0; i < v.items.size(); ++i) {
                if (i)
                    m_buf += ",";
                indent(depth + 1);
                if (!dump(v.items[i], depth + 1, err))
                    return false;
            }
            indent(depth);
            m_buf += "]";
            return true;
        }
        case Type::Object: {
            if (v.members.empty()) {
                m_buf += "{}";
                return true;
            }
            m_buf += "{";
            for (size_t i = 0; i < v.members.size(); ++i) {
                if (i)
                    m_buf += ",";
                indent(depth + 1);
                escape_into(v.members[i].first, m_buf);
                m_buf += ":";
                if (d_pretty)
                    m_buf += " ";
                if (!dump(v.members[i].second, depth + 1, err))
                    return false;
            }
            indent(depth);
            m_buf += "}";
            return true;
        }
        }
        err = "unreachable json type";
        return false;
    }

    const Value& d_v;
    bool d_pretty;
    std::string m_buf;
};

inline bool dump(const Value& v, std::string& out, std::string& err, bool pretty = true)
{
    Dumper d(v, pretty);
    return d.run(out, err);
}

} // namespace json

// ===========================================================================
// 9. TwrConfig <-> JSON
// ===========================================================================

// Small ordered object writer.  Emits every field, so the round trip is
// exact and a config file documents itself.
class ConfigJsonWriter
{
public:
    ConfigJsonWriter() { m_stack.push_back(&m_root); }

    bool ok() const { return m_err.empty(); }
    const std::string& error() const { return m_err; }
    json::Value value() const { return m_root; }
    std::string to_string(bool pretty = true) const
    {
        std::string out;
        std::string err;
        if (!json::dump(m_root, out, err, pretty)) {
            return std::string();
        }
        return out;
    }

    // The root is already an object; this exists so the document shape reads
    // the same as the reader.
    void current_object() {}

    void begin_group(const std::string& key)
    {
        m_stack.back()->set(key, json::Value::make_object());
        m_stack.push_back(m_stack.back()->find(key));
    }
    void begin_array(const std::string& key)
    {
        m_stack.back()->set(key, json::Value::make_array());
        m_stack.push_back(m_stack.back()->find(key));
    }
    void end() { m_stack.pop_back(); }

    void push_object()
    {
        json::Value* parent = m_stack.back();
        parent->push(json::Value::make_object());
        m_stack.push_back(&parent->items.back());
    }
    void pop_object() { m_stack.pop_back(); }

    void put(const std::string& key, const std::string& v)
    {
        current()->set(key, json::Value::make_string(v));
    }
    void put(const std::string& key, int64_t v)
    {
        current()->set(key, json::Value::make_int(v));
    }
    void put(const std::string& key, uint32_t v)
    {
        current()->set(key, json::Value::make_int(static_cast<int64_t>(v)));
    }
    void put(const std::string& key, uint16_t v)
    {
        current()->set(key, json::Value::make_int(static_cast<int64_t>(v)));
    }
    void put(const std::string& key, uint8_t v)
    {
        current()->set(key, json::Value::make_int(static_cast<int64_t>(v)));
    }
    void put(const std::string& key, int v)
    {
        current()->set(key, json::Value::make_int(static_cast<int64_t>(v)));
    }
    void put(const std::string& key, uint64_t v)
    {
        // uint64 above int64 max is not representable; the schema has no such
        // field, but keep the check explicit rather than wrapping silently.
        if (v > static_cast<uint64_t>(INT64_MAX)) {
            fail("field " + key + " exceeds the int64 range of the JSON schema");
            current()->set(key, json::Value::make_null());
            return;
        }
        current()->set(key, json::Value::make_int(static_cast<int64_t>(v)));
    }
    void put(const std::string& key, bool v)
    {
        current()->set(key, json::Value::make_bool(v));
    }
    void put(const std::string& key, double v)
    {
        if (!std::isfinite(v)) {
            fail("field " + key + " is not finite and cannot be written as JSON");
            current()->set(key, json::Value::make_null());
            return;
        }
        current()->set(key, json::Value::make_double(v));
    }
    void put(const std::string& key, const char* v)
    {
        current()->set(key, json::Value::make_string(v ? v : ""));
    }
    void put_ns(const std::string& key, const Duration& d)
    {
        current()->set(key, json::Value::make_int(d.nanos()));
    }
    void put_opt(const std::string& key, const Opt<int64_t>& o)
    {
        current()->set(key,
                       o.has() ? json::Value::make_int(o.value()) : json::Value::make_null());
    }
    void put_opt(const std::string& key, const Opt<uint32_t>& o)
    {
        current()->set(key,
                       o.has() ? json::Value::make_int(static_cast<int64_t>(o.value()))
                               : json::Value::make_null());
    }
    void put_opt(const std::string& key, const Opt<double>& o)
    {
        if (!o.has()) {
            current()->set(key, json::Value::make_null());
            return;
        }
        if (!std::isfinite(o.value())) {
            fail("field " + key + " is not finite and cannot be written as JSON");
            current()->set(key, json::Value::make_null());
            return;
        }
        current()->set(key, json::Value::make_double(o.value()));
    }
    void put_timed(const std::string& key, const TimedField& t)
    {
        json::Value o = json::Value::make_object();
        o.set("ns", json::Value::make_int(t.value.nanos()));
        o.set("domain", json::Value::make_string(time_domain_to_string(t.domain)));
        o.set("reference",
            json::Value::make_string(time_reference_event_to_string(t.reference)));
        if (t.marker.has())
            o.set("marker",
                json::Value::make_string(timestamp_marker_to_string(t.marker.value())));
        else
            o.set("marker", json::Value::make_null());
        o.set("quantisation_hz", json::Value::make_double(t.required_quantisation_hz));
        o.set("max_quantisation_error_ns",
              json::Value::make_int(t.max_quantisation_error_ns));
        o.set("note", json::Value::make_string(t.note));
        current()->set(key, std::move(o));
    }

private:
    json::Value* current() { return m_stack.back(); }
    void fail(const std::string& m)
    {
        if (m_err.empty())
            m_err = m;
    }

    json::Value m_root;
    std::vector<json::Value*> m_stack;
    std::string m_err;
};

inline void write_config_json(const TwrConfig& c, ConfigJsonWriter& w)
{
    w.current_object();
    w.begin_group("meta");
    w.put("schema_version", c.meta.schema_version);
    w.put("profile_version", c.meta.profile_version);
    w.put("calibration_version", c.meta.calibration_version);
    w.put("label", c.meta.label);
    w.end();

    w.begin_group("session");
    w.put("protocol", protocol_to_string(c.session.protocol));
    w.put("role", role_to_string(c.session.role));
    w.put("local_address", c.session.local_address);
    w.put("peer_address", c.session.peer_address);
    w.put("pan_id", c.session.pan_id);
    w.put("session_id", c.session.session_id);
    w.put("exchange_id", c.session.exchange_id);
    w.put("sequence", c.session.sequence);
    w.put("sequence_modulus", c.session.sequence_modulus);
    w.put("measurement_count", c.session.measurement_count);
    w.put_ns("measurement_interval_ns", c.session.measurement_interval);
    w.put("max_attempts_per_exchange", c.session.max_attempts_per_exchange);
    w.put_ns("retry_backoff_ns", c.session.retry_backoff);
    w.put("max_in_flight_exchanges", c.session.max_in_flight_exchanges);
    w.put("require_pan_match", c.session.require_pan_match);
    w.put("require_address_match", c.session.require_address_match);
    w.end();

    w.begin_group("phy");
    w.put("channel", c.phy.channel);
    w.put("center_frequency_hz", c.phy.center_frequency_hz);
    w.put("tx_preamble_code", c.phy.tx_preamble_code);
    w.put("rx_preamble_code", c.phy.rx_preamble_code);
    w.put("preamble_symbols", c.phy.preamble_symbols);
    w.put("prf_class", prf_class_to_string(c.phy.prf_class));
    w.put("data_rate", data_rate_to_string(c.phy.data_rate));
    // PHR rate, in the PHR-rate spelling ("850k" / "same_as_data"), NOT the
    // payload-rate spelling: the two axes have separate value domains and a
    // document that used "6p8m" here was describing a PHR this build cannot
    // transmit.
    w.put("phr_rate", phr_rate_to_string(c.phy.phr_rate));
    w.end();

    w.begin_group("frame");
    w.put("frame_profile", frame_profile_id_to_string(c.frame.frame_profile));
    w.put("sfd_mode", sfd_mode_to_string(c.frame.sfd_mode));
    w.put("sfd_symbols", c.frame.sfd_symbols);
    w.put_timed("sfd_timeout", c.frame.sfd_timeout);
    w.put("phr_mode", phr_mode_to_string(c.frame.phr_mode));
    w.put("ranging_bit", c.frame.ranging_bit);
    // The geometry CLAIM is written out verbatim, so a reader can see what the
    // operator asserted; the validator is what decides whether it agrees with
    // the codec.
    w.begin_group("geometry");
    w.put("mac_header_bytes", c.frame.geometry.mac_header_bytes);
    w.put("timestamp_bytes", c.frame.geometry.timestamp_bytes);
    w.put("mac_footer_bytes", c.frame.geometry.mac_footer_bytes);
    w.put("mac_fcs_bytes", c.frame.geometry.mac_fcs_bytes);
    w.put("phr_bytes", c.frame.geometry.phr_bytes);
    w.end();
    w.put("mac_psdu_bytes", c.frame.mac_psdu_bytes);
    w.put("mac_psdu_includes_fcs", c.frame.mac_psdu_includes_fcs);
    w.put("fcs_append", fcs_appender_to_string(c.frame.fcs_append));
    w.put("fcs_bytes", c.frame.fcs_bytes);
    w.put("application_payload_bytes", c.frame.application_payload_bytes);
    w.put("sts_mode", sts_mode_to_string(c.frame.sts_mode));
    w.put("sts_length_symbols", c.frame.sts_length_symbols);
    w.end();

    w.begin_group("tx");
    w.put("port", c.tx.port);
    w.put_opt("gain_db", c.tx.gain_db);
    w.put_opt("iq_amplitude", c.tx.iq_amplitude);
    w.put_opt("calibrated_tx_power_dbm", c.tx.calibrated_tx_power_dbm);
    w.put("power_policy", tx_power_policy_to_string(c.tx.power_policy));
    w.put("pulse_shaping", pulse_shaping_to_string(c.tx.pulse_shaping));
    w.put_opt("vendor_power_word", c.tx.vendor_power_word);
    w.put("vendor_power_word_backend", c.tx.vendor_power_word_backend);
    w.end();

    w.begin_group("rx");
    w.put("port", c.rx.port);
    w.put_opt("gain_db", c.rx.gain_db);
    w.put("agc", agc_mode_to_string(c.rx.agc));
    w.put_opt("bandwidth_hz", c.rx.bandwidth_hz);
    w.put("detection_threshold", c.rx.detection_threshold);
    w.put("correlation_threshold", c.rx.correlation_threshold);
    w.put("first_path_threshold", c.rx.first_path_threshold);
    w.put("first_path_index", c.rx.first_path_index);
    w.put("first_path_window", c.rx.first_path_window);
    w.put_opt("vendor_pac_value", c.rx.vendor_pac_value);
    w.put("vendor_pac_backend", c.rx.vendor_pac_backend);
    w.put_opt("vendor_pac_applied_step", c.rx.vendor_pac_applied_step);
    w.end();

    w.begin_group("radio");
    w.put("device_args", c.radio.device_args);
    w.put("tx_channel", c.radio.tx_channel);
    w.put("rx_channel", c.radio.rx_channel);
    w.put("native_sample_rate_hz", c.radio.native_sample_rate_hz);
    w.put("clock_source", c.radio.clock_source);
    w.put("time_source", c.radio.time_source);
    w.put("fpga_image", c.radio.fpga_image);
    w.put("dpdk_config", c.radio.dpdk_config);
    w.begin_array("peers");
    for (const auto& p : c.radio.peers) {
        w.push_object();
        w.put("id", p.id);
        w.put("role", role_to_string(p.role));
        w.put("tx_channel", p.tx_channel);
        w.put("rx_channel", p.rx_channel);
        w.put("native_sample_rate_hz", p.native_sample_rate_hz);
        w.put("occupies_resources", p.occupies_resources);
        w.pop_object();
    }
    w.end();
    w.begin_group("readback");
    w.put("present", c.radio.readback.present);
    w.put("sample_rate_hz", c.radio.readback.sample_rate_hz);
    w.put("center_freq_hz", c.radio.readback.center_freq_hz);
    w.put("tx_channel", c.radio.readback.tx_channel);
    w.put("rx_channel", c.radio.readback.rx_channel);
    w.put("mpm_string", c.radio.readback.mpm_string);
    w.put("fpga_image", c.radio.readback.fpga_image);
    w.put("uhd_version", c.radio.readback.uhd_version);
    w.put("clock_source", c.radio.readback.clock_source);
    w.put("time_source", c.radio.readback.time_source);
    w.end();
    w.put("require_readback", c.radio.require_readback);
    w.end();

    w.begin_group("timing");
    w.put_timed("poll_start", c.timing.poll_start);
    w.put_timed("poll_to_response", c.timing.poll_to_response);
    w.put_timed("response_to_final", c.timing.response_to_final);
    w.put_timed("final_to_report", c.timing.final_to_report);
    w.put_timed("post_tx_rx_enable", c.timing.post_tx_rx_enable);
    w.put_timed("min_tx_lead_time", c.timing.min_tx_lead_time);
    w.end();

    w.begin_group("timeouts");
    w.put_timed("poll_rx_window", c.timeouts.poll_rx_window);
    w.put_timed("response_rx_window", c.timeouts.response_rx_window);
    w.put_timed("final_rx_window", c.timeouts.final_rx_window);
    w.put_timed("report_rx_window", c.timeouts.report_rx_window);
    w.put_timed("rx_timeout", c.timeouts.rx_timeout);
    w.put_timed("exchange_timeout", c.timeouts.exchange_timeout);
    w.put_timed("retry_interval", c.timeouts.retry_interval);
    w.end();

    w.begin_group("calibration");
    w.put_ns("tx_link_delay_ns", c.calibration.tx_link_delay);
    w.put_ns("rx_link_delay_ns", c.calibration.rx_link_delay);
    w.put_ns("antenna_delay_ns", c.calibration.antenna_delay);
    w.put_ns("cable_delay_ns", c.calibration.cable_delay);
    w.put("tx_link_delay_native_ticks", c.calibration.tx_link_delay_native_ticks);
    w.put("rx_link_delay_native_ticks", c.calibration.rx_link_delay_native_ticks);
    w.put("native_sample_rate_hz", c.calibration.native_sample_rate_hz);
    w.put("link_delay_unit", time_unit_to_string(c.calibration.link_delay_unit));
    w.put("first_path_algorithm",
          first_path_algorithm_to_string(c.calibration.first_path_algorithm));
    w.put("cfo_compensation",
        compensation_flag_to_string(c.calibration.cfo_compensation));
    w.put("sfo_compensation",
        compensation_flag_to_string(c.calibration.sfo_compensation));
    w.put("calibration_id", c.calibration.calibration_id);
    w.begin_group("record");
    w.put("calibration_id", c.calibration.record.calibration_id);
    w.put("device_serial", c.calibration.record.device_serial);
    w.put("channel", c.calibration.record.channel);
    w.put("native_sample_rate_hz", c.calibration.record.native_sample_rate_hz);
    w.put("profile_version", c.calibration.record.profile_version);
    w.put_opt("gain_db", c.calibration.record.gain_db);
    w.put("valid_until_monotonic_ns", c.calibration.record.valid_until_monotonic_ns);
    w.end();
    w.put("applied_count", c.calibration.applied_count);
    w.put("calibration_required", c.calibration.calibration_required);
    w.end();

    w.begin_group("diagnostics");
    w.put("cir_capture_enabled", c.diagnostics.cir_capture_enabled);
    w.put("cir_capture_max_bytes", c.diagnostics.cir_capture_max_bytes);
    w.put("cir_capture_stride", c.diagnostics.cir_capture_stride);
    w.put("short_iq_enabled", c.diagnostics.short_iq_enabled);
    w.put("short_iq_max_bytes", c.diagnostics.short_iq_max_bytes);
    w.put("short_iq_stride", c.diagnostics.short_iq_stride);
    w.put("raw_frame_dump", c.diagnostics.raw_frame_dump);
    w.put("raw_frame_max_bytes", c.diagnostics.raw_frame_max_bytes);
    w.put("result_output_path", c.diagnostics.result_output_path);
    w.put("result_queue_capacity", c.diagnostics.result_queue_capacity);
    w.put("event_queue_capacity", c.diagnostics.event_queue_capacity);
    w.put_timed("stats_cadence", c.diagnostics.stats_cadence);
    w.put("io_on_realtime_thread", c.diagnostics.io_on_realtime_thread);
    w.end();
}

// Strict reader: every failure is a machine-readable violation, never a
// silently skipped key and never a default.
class ConfigJsonReader
{
public:
    void bad(const std::string& field,
             ConfigReason r,
             const std::string& msg,
             ExchangeStatus st = ExchangeStatus::ConfigRejected)
    {
        m_report.add(field, r, st, msg, "REQ-API-01");
    }

    ValidationReport report() const { return m_report; }
    bool ok() const { return m_report.ok(); }

    // -- object navigation ------------------------------------------------
    const json::Value* group(const std::string& path, const json::Value* parent)
    {
        const json::Value* g = parent ? parent->find(path) : nullptr;
        if (!g) {
            bad(path, ConfigReason::MissingKey, "missing required group");
            return nullptr;
        }
        if (g->type != json::Type::Object) {
            bad(path, ConfigReason::TypeMismatch, "expected an object, got " +
                                                         std::string(g->type_name()));
            return nullptr;
        }
        return g;
    }

    // -- typed getters ----------------------------------------------------
    std::string get_str(const json::Value* o, const std::string& path, const char* key)
    {
        const json::Value* v = key_of(o, path, key);
        if (!v)
            return std::string();
        if (v->type == json::Type::Null)
            return std::string();
        if (v->type != json::Type::String) {
            bad(path + "." + key, ConfigReason::TypeMismatch,
                "expected a string, got " + std::string(v->type_name()));
            return std::string();
        }
        return v->text;
    }
    bool get_bool(const json::Value* o, const std::string& path, const char* key)
    {
        const json::Value* v = key_of(o, path, key);
        if (!v)
            return false;
        if (v->type != json::Type::Bool) {
            bad(path + "." + key, ConfigReason::TypeMismatch,
                "expected a bool, got " + std::string(v->type_name()));
            return false;
        }
        return v->boolean;
    }
    int64_t get_i64(const json::Value* o, const std::string& path, const char* key)
    {
        const json::Value* v = key_of(o, path, key);
        if (!v)
            return 0;
        if (v->type == json::Type::String) {
            // The decimal-string form used for values beyond 2^53.
            int64_t out = 0;
            if (parse_int64_strict(v->text, out))
                return out;
            bad(path + "." + key, ConfigReason::IntegerPrecisionLoss,
                "string is not a decimal int64: '" + v->text + "'");
            return 0;
        }
        if (v->type != json::Type::Int) {
            bad(path + "." + key, ConfigReason::TypeMismatch,
                "expected an integer, got " + std::string(v->type_name()) +
                    " (a decimal fraction or exponent is never truncated silently)");
            return 0;
        }
        return v->integer;
    }

    // -- range-checked unsigned narrowing (M0.1 / N01) --------------------
    //
    // The reader used to assign through `static_cast<uint16_t>(get_i64(...))`,
    // which WRAPS an out-of-range JSON number instead of refusing it: channel
    // 261 became channel 5, session 2^32+1 became session 1, local_address
    // 65537 became 1.  The validator then never saw the value the document
    // actually carried, and two different documents could produce the same
    // runtime configuration -- exactly the silent fallback REQ-CFG forbids and
    // the reason `session_id_to_wire()` exists.
    //
    // A C++ cast is not a specification.  These getters are: the value must be
    // in [0, max] or it is refused with `out_of_range` and stored as 0.  They
    // keep `get_i64`'s shape (same arguments) so the call sites read the same
    // way, but the range check happens BEFORE any narrowing.
    bool check_unsigned(const std::string& path, const char* key, int64_t v,
                        uint64_t max_value, uint64_t& out)
    {
        // Formatted unsigned: `twr_int_to_text` takes an int64_t, so passing a
        // uint64_t max would print a negative bound.
        const std::string bound = std::to_string(
            static_cast<unsigned long long>(max_value));
        if (v < 0) {
            bad(path + "." + key, ConfigReason::OutOfRange,
                "must be in [0, " + bound + "], got " + twr_int_to_text(v) +
                    " (a negative value is refused, never wrapped to fit the field)");
            out = 0u;
            return false;
        }
        const uint64_t u = static_cast<uint64_t>(v);
        if (u > max_value) {
            bad(path + "." + key, ConfigReason::OutOfRange,
                "must be in [0, " + bound + "], got " + twr_int_to_text(v) +
                    " (out of range for this field; refused, not masked)");
            out = 0u;
            return false;
        }
        out = u;
        return true;
    }

    uint64_t get_u64(const json::Value* o, const std::string& path, const char* key,
                     uint64_t max_value)
    {
        const json::Value* v = key_of(o, path, key);
        if (!v)
            return 0u;
        int64_t raw = 0;
        if (v->type == json::Type::String) {
            if (!parse_int64_strict(v->text, raw)) {
                bad(path + "." + key, ConfigReason::IntegerPrecisionLoss,
                    "string is not a decimal int64: '" + v->text + "'");
                return 0u;
            }
        } else if (v->type != json::Type::Int) {
            bad(path + "." + key, ConfigReason::TypeMismatch,
                "expected an integer, got " + std::string(v->type_name()) +
                    " (a decimal fraction or exponent is never truncated silently)");
            return 0u;
        } else {
            raw = v->integer;
        }
        uint64_t out = 0u;
        check_unsigned(path, key, raw, max_value, out);
        return out;
    }

    uint8_t get_u8(const json::Value* o, const std::string& path, const char* key)
    {
        return static_cast<uint8_t>(get_u64(o, path, key, 0xFFu));
    }
    uint16_t get_u16(const json::Value* o, const std::string& path, const char* key)
    {
        return static_cast<uint16_t>(get_u64(o, path, key, 0xFFFFu));
    }
    uint32_t get_u32(const json::Value* o, const std::string& path, const char* key)
    {
        return static_cast<uint32_t>(get_u64(o, path, key, 0xFFFFFFFFu));
    }
    double get_double(const json::Value* o, const std::string& path, const char* key)
    {
        const json::Value* v = key_of(o, path, key);
        if (!v)
            return 0.0;
        if (!v->is_number()) {
            bad(path + "." + key, ConfigReason::TypeMismatch,
                "expected a number, got " + std::string(v->type_name()));
            return 0.0;
        }
        return v->as_double();
    }
    Duration get_dur(const json::Value* o, const std::string& path, const char* key)
    {
        return Duration::from_nanos(get_i64(o, path, key));
    }
    // -- optional getters ------------------------------------------------
    Opt<int64_t> get_opt_i64(const json::Value* o, const std::string& path,
        const char* key)
    {
        const json::Value* v = key_of(o, path, key, /*required=*/false);
        if (!v || v->type == json::Type::Null)
            return Opt<int64_t>::none();
        const std::string p = path + "." + key;
        Opt<int64_t> out;
        if (v->type == json::Type::String) {
            int64_t parsed = 0;
            if (!parse_int64_strict(v->text, parsed)) {
                bad(p, ConfigReason::IntegerPrecisionLoss,
                    "string is not a decimal int64: '" + v->text + "'");
                return Opt<int64_t>::none();
            }
            out = parsed;
        } else if (v->type == json::Type::Int) {
            out = v->integer;
        } else {
            bad(p, ConfigReason::TypeMismatch,
                "expected an integer or null, got " + std::string(v->type_name()));
        }
        return out;
    }
    Opt<uint32_t> get_opt_u32(const json::Value* o, const std::string& path,
        const char* key)
    {
        const Opt<int64_t> v = get_opt_i64(o, path, key);
        if (!v.has())
            return Opt<uint32_t>::none();
        if (v.value() < 0 || v.value() > 0xffffffffLL) {
            bad(path + "." + key, ConfigReason::OutOfRange, "outside the uint32 range");
            return Opt<uint32_t>::none();
        }
        return Opt<uint32_t>(static_cast<uint32_t>(v.value()));
    }
    Opt<double> get_opt_double(const json::Value* o, const std::string& path,
        const char* key)
    {
        const json::Value* v = key_of(o, path, key, /*required=*/false);
        if (!v || v->type == json::Type::Null)
            return Opt<double>::none();
        if (!v->is_number()) {
            bad(path + "." + key, ConfigReason::TypeMismatch,
                "expected a number or null, got " + std::string(v->type_name()));
            return Opt<double>::none();
        }
        return Opt<double>(v->as_double());
    }
    // -- enum getters ----------------------------------------------------
    template <typename E, typename F>
    E get_enum(const json::Value* o, const std::string& path, const char* key,
        F from_string)
    {
        const json::Value* v = key_of(o, path, key);
        if (!v)
            return static_cast<E>(0);
        if (v->type != json::Type::String) {
            bad(path + "." + key, ConfigReason::TypeMismatch,
                "expected a string enum, got " + std::string(v->type_name()));
            return static_cast<E>(0);
        }
        E out = static_cast<E>(0);
        if (!from_string(v->text, out)) {
            bad(path + "." + key, ConfigReason::UnknownEnumValue,
                "'" + v->text + "' is not a known value of this enum");
            return static_cast<E>(0);
        }
        return out;
    }
    TimedField get_timed(const json::Value* o, const std::string& path, const char* key)
    {
        TimedField t;
        const std::string p = path + "." + key;
        const json::Value* v = key_of(o, path, key);
        if (!v)
            return t;
        if (v->type != json::Type::Object) {
            bad(p, ConfigReason::TypeMismatch,
                "expected an object, got " + std::string(v->type_name()));
            return t;
        }
        t.value = Duration::from_nanos(get_i64(v, p, "ns"));
        t.domain = get_enum<TimeDomain>(v, p, "domain", time_domain_from_string);
        t.reference = get_enum<TimeReferenceEvent>(v, p, "reference",
                                                   time_reference_event_from_string);
        const json::Value* mk = key_of(v, p, "marker", /*required=*/false);
        if (mk && mk->type == json::Type::String) {
            bool found = false;
            for (int i = 0; i <= static_cast<int>(TimestampMarker::AntennaPlane); ++i) {
                const TimestampMarker m = static_cast<TimestampMarker>(i);
                if (mk->text == timestamp_marker_to_string(m)) {
                    t.marker = m;
                    found = true;
                    break;
                }
            }
            if (!found)
                bad(p + ".marker", ConfigReason::UnknownEnumValue,
                    "'" + mk->text + "' is not a known timestamp marker");
        } else if (mk && mk->type != json::Type::Null) {
            bad(p + ".marker", ConfigReason::TypeMismatch,
                "expected a marker name or null, got " + std::string(mk->type_name()));
        }
        t.required_quantisation_hz = get_double(v, p, "quantisation_hz");
        t.max_quantisation_error_ns = get_i64(v, p, "max_quantisation_error_ns");
        t.note = get_str(v, p, "note");
        return t;
    }

    static bool parse_int64_strict(const std::string& s, int64_t& out)
    {
        if (s.empty())
            return false;
        size_t i = 0;
        if (s[0] == '-')
            i = 1;
        if (i >= s.size())
            return false;
        for (size_t k = i; k < s.size(); ++k)
            if (s[k] < '0' || s[k] > '9')
                return false;
        errno = 0;
        char* end = nullptr;
        const long long v = std::strtoll(s.c_str(), &end, 10);
        if (errno != 0 || end == nullptr || *end != '\0')
            return false;
        out = static_cast<int64_t>(v);
        return true;
    }

    // Reject keys the schema does not define: an unknown key is a typo or an
    // unsupported feature, and either way it must not be ignored.
    void reject_unknown_keys(const json::Value* o, const std::string& path,
        const char* const* known)
    {
        if (!o || o->type != json::Type::Object)
            return;
        for (const auto& m : o->members) {
            bool found = false;
            for (size_t i = 0; known[i] != nullptr; ++i) {
                if (m.first == known[i]) {
                    found = true;
                    break;
                }
            }
            if (!found)
                bad(path + "." + m.first, ConfigReason::UnknownKey,
                    "key is not part of schema twr-config/2");
        }
    }

private:
    const json::Value* key_of(const json::Value* o,
                              const std::string& path,
                              const char* key,
                              bool required = true)
    {
        if (!o)
            return nullptr;
        const json::Value* v = o->find(key);
        if (!v) {
            if (required)
                bad(path + "." + key, ConfigReason::MissingKey, "missing required key");
            return nullptr;
        }
        return v;
    }
    ValidationReport m_report;
};

// ---------------------------------------------------------------------------

inline bool to_json_string(const TwrConfig& c, std::string& out, std::string& err)
{
    ConfigJsonWriter w;
    write_config_json(c, w);
    if (!w.ok()) {
        err = w.error();
        return false;
    }
    return json::dump(w.value(), out, err, /*pretty=*/true);
}

// Convenience: the frozen snapshot plus its report, as one JSON document.
inline bool to_json_string(const EffectiveConfig& e, std::string& out, std::string& err)
{
    ConfigJsonWriter w;
    w.current_object();
    w.put("ok", e.ok);
    w.put("schema_version", e.schema_version);
    w.put("profile_version", e.profile_version);
    w.put("calibration_version", e.calibration_version);
    w.put("config_hash", e.config_hash);
    write_config_json(e.effective, w);
    w.begin_array("changes");
    for (const auto& ch : e.changes) {
        w.push_object();
        w.put("path", ch.path);
        w.put("requested", ch.requested);
        w.put("effective", ch.effective);
        w.put("note", ch.note);
        w.pop_object();
    }
    w.end();
    w.begin_array("violations");
    for (const auto& v : e.validation.violations) {
        w.push_object();
        w.put("field", v.field);
        w.put("reason", config_reason_to_string(v.reason));
        w.put("status", exchange_status_to_string(v.status));
        w.put("message", v.message);
        w.put("requirement", v.requirement);
        w.pop_object();
    }
    w.end();
    w.begin_group("quantised");
    w.put("tick_rate_hz", e.tick_rate_hz);
    w.put("poll_start_ticks", e.poll_start_ticks);
    w.put("poll_to_response_ticks", e.poll_to_response_ticks);
    w.put("response_to_final_ticks", e.response_to_final_ticks);
    w.put("post_tx_rx_enable_ticks", e.post_tx_rx_enable_ticks);
    w.put_ns("poll_to_response_effective_ns", e.poll_to_response_effective);
    w.put_ns("response_to_final_effective_ns", e.response_to_final_effective);
    w.put_ns("post_tx_rx_enable_effective_ns", e.post_tx_rx_enable_effective);
    w.end();
    w.begin_group("frame_budget");
    w.put("poll_bytes", e.poll_bytes);
    w.put("response_bytes", e.response_bytes);
    w.put("final_bytes", e.final_bytes);
    w.put("max_psdu_bytes", e.max_psdu_bytes);
    w.end();
    if (!w.ok()) {
        err = w.error();
        return false;
    }
    return json::dump(w.value(), out, err, /*pretty=*/true);
}

inline ValidationReport from_json_string(const std::string& text, TwrConfig& out)
{
    ConfigJsonReader rd;
    json::Value root;
    std::string err;
    if (!json::parse(text, root, err)) {
        rd.bad("", ConfigReason::MalformedJson, err);
        return rd.report();
    }
    if (root.type != json::Type::Object) {
        rd.bad("", ConfigReason::TypeMismatch,
            "the config document must be a JSON object");
        return rd.report();
    }
    TwrConfig c;

    {
        const std::string p = "meta";
        const json::Value* g = rd.group(p, &root);
        if (g) {
            static const char* const kKeys[] = { "schema_version", "profile_version",
                                                 "calibration_version", "label", nullptr };
            rd.reject_unknown_keys(g, p, kKeys);
            c.meta.schema_version = rd.get_str(g, p, "schema_version");
            c.meta.profile_version = rd.get_str(g, p, "profile_version");
            c.meta.calibration_version = rd.get_str(g, p, "calibration_version");
            c.meta.label = rd.get_str(g, p, "label");
        }
    }
    {
        const std::string p = "session";
        const json::Value* g = rd.group(p, &root);
        if (g) {
            static const char* const kKeys[] = {
                "protocol", "role", "local_address", "peer_address", "pan_id", "session_id",
                "exchange_id", "sequence", "sequence_modulus", "measurement_count",
                "measurement_interval_ns", "max_attempts_per_exchange", "retry_backoff_ns",
                "max_in_flight_exchanges", "require_pan_match", "require_address_match", nullptr
            };
            rd.reject_unknown_keys(g, p, kKeys);
            c.session.protocol = rd.get_enum<Protocol>(g, p, "protocol",
                protocol_from_string);
            c.session.role = rd.get_enum<Role>(g, p, "role", role_from_string);
            c.session.local_address =
                static_cast<uint16_t>(rd.get_u16(g, p, "local_address"));
            c.session.peer_address = static_cast<uint16_t>(rd.get_u16(g, p,
                "peer_address"));
            c.session.pan_id = static_cast<uint16_t>(rd.get_u16(g, p, "pan_id"));
            c.session.session_id = static_cast<uint32_t>(rd.get_u32(g, p,
                "session_id"));
            c.session.exchange_id = static_cast<uint32_t>(rd.get_u32(g, p,
                "exchange_id"));
            c.session.sequence = static_cast<uint8_t>(rd.get_u8(g, p, "sequence"));
            c.session.sequence_modulus =
                static_cast<uint16_t>(rd.get_u16(g, p, "sequence_modulus"));
            c.session.measurement_count =
                static_cast<uint32_t>(rd.get_u32(g, p, "measurement_count"));
            c.session.measurement_interval = rd.get_dur(g, p,
                "measurement_interval_ns");
            c.session.max_attempts_per_exchange =
                static_cast<uint32_t>(rd.get_u32(g, p, "max_attempts_per_exchange"));
            c.session.retry_backoff = rd.get_dur(g, p, "retry_backoff_ns");
            c.session.max_in_flight_exchanges =
                static_cast<uint32_t>(rd.get_u32(g, p, "max_in_flight_exchanges"));
            c.session.require_pan_match = rd.get_bool(g, p, "require_pan_match");
            c.session.require_address_match = rd.get_bool(g, p,
                "require_address_match");
        }
    }
    {
        const std::string p = "phy";
        const json::Value* g = rd.group(p, &root);
        if (g) {
            static const char* const kKeys[] = {
                "channel", "center_frequency_hz", "tx_preamble_code", "rx_preamble_code",
                "preamble_symbols", "prf_class", "data_rate", "phr_rate", nullptr
            };
            rd.reject_unknown_keys(g, p, kKeys);
            c.phy.channel = static_cast<uint8_t>(rd.get_u8(g, p, "channel"));
            c.phy.center_frequency_hz = rd.get_double(g, p, "center_frequency_hz");
            c.phy.tx_preamble_code = static_cast<uint8_t>(rd.get_u8(g, p,
                "tx_preamble_code"));
            c.phy.rx_preamble_code = static_cast<uint8_t>(rd.get_u8(g, p,
                "rx_preamble_code"));
            c.phy.preamble_symbols = static_cast<uint16_t>(rd.get_u16(g, p,
                "preamble_symbols"));
            c.phy.prf_class = rd.get_enum<PrfClass>(g, p, "prf_class",
                prf_class_from_string);
            c.phy.data_rate = rd.get_enum<DataRate>(g, p, "data_rate",
                data_rate_from_string);
            // PHR rate, read in the PHR-rate domain.  A payload-rate name such
            // as "6p8m" is now an UnknownEnumValue here, which is the point:
            // the two axes no longer share a value domain.
            c.phy.phr_rate = rd.get_enum<PhrRate>(g, p, "phr_rate",
                phr_rate_from_string);
        }
    }
    {
        const std::string p = "frame";
        const json::Value* g = rd.group(p, &root);
        if (g) {
            static const char* const kKeys[] = {
                "frame_profile",
                "sfd_mode", "sfd_symbols", "sfd_timeout", "phr_mode", "ranging_bit", "geometry",
                "mac_psdu_bytes", "mac_psdu_includes_fcs", "fcs_append", "fcs_bytes",
                "application_payload_bytes", "sts_mode", "sts_length_symbols", nullptr
            };
            rd.reject_unknown_keys(g, p, kKeys);
            c.frame.frame_profile = rd.get_enum<FrameProfileId>(g, p, "frame_profile",
                frame_profile_id_from_string);
            c.frame.sfd_mode = rd.get_enum<SfdMode>(g, p, "sfd_mode",
                sfd_mode_from_string);
            c.frame.sfd_symbols = static_cast<uint16_t>(rd.get_u16(g, p,
                "sfd_symbols"));
            c.frame.sfd_timeout = rd.get_timed(g, p, "sfd_timeout");
            c.frame.phr_mode = rd.get_enum<PhrMode>(g, p, "phr_mode",
                phr_mode_from_string);
            c.frame.ranging_bit = rd.get_bool(g, p, "ranging_bit");
            const json::Value* geo = g->find("geometry");
            if (!geo)
                rd.bad(p + ".geometry", ConfigReason::MissingKey,
                    "missing required group");
            else if (geo->type != json::Type::Object)
                rd.bad(p + ".geometry", ConfigReason::TypeMismatch,
                    "expected an object");
            else {
                static const char* const kGeo[] = { "mac_header_bytes",
                    "timestamp_bytes",
                                                    "mac_footer_bytes", "mac_fcs_bytes",
                                                    "phr_bytes", nullptr };
                rd.reject_unknown_keys(geo, p + ".geometry", kGeo);
                c.frame.geometry.mac_header_bytes =
                    static_cast<uint16_t>(rd.get_u16(geo, p + ".geometry",
                        "mac_header_bytes"));
                c.frame.geometry.timestamp_bytes =
                    static_cast<uint16_t>(rd.get_u16(geo, p + ".geometry",
                        "timestamp_bytes"));
                c.frame.geometry.mac_footer_bytes =
                    static_cast<uint16_t>(rd.get_u16(geo, p + ".geometry",
                        "mac_footer_bytes"));
                c.frame.geometry.mac_fcs_bytes =
                    static_cast<uint16_t>(rd.get_u16(geo, p + ".geometry",
                        "mac_fcs_bytes"));
                c.frame.geometry.phr_bytes =
                    static_cast<uint16_t>(rd.get_u16(geo, p + ".geometry",
                        "phr_bytes"));
            }
            c.frame.mac_psdu_bytes = static_cast<uint16_t>(rd.get_u16(g, p,
                "mac_psdu_bytes"));
            c.frame.mac_psdu_includes_fcs = rd.get_bool(g, p, "mac_psdu_includes_fcs");
            c.frame.fcs_append =
                rd.get_enum<FcsAppender>(g, p, "fcs_append", fcs_appender_from_string);
            c.frame.fcs_bytes = static_cast<uint16_t>(rd.get_u16(g, p, "fcs_bytes"));
            c.frame.application_payload_bytes =
                static_cast<uint16_t>(rd.get_u16(g, p, "application_payload_bytes"));
            c.frame.sts_mode = rd.get_enum<StsMode>(g, p, "sts_mode",
                sts_mode_from_string);
            c.frame.sts_length_symbols =
                static_cast<uint16_t>(rd.get_u16(g, p, "sts_length_symbols"));
        }
    }
    {
        const std::string p = "tx";
        const json::Value* g = rd.group(p, &root);
        if (g) {
            static const char* const kKeys[] = {
                "port", "gain_db", "iq_amplitude", "calibrated_tx_power_dbm", "power_policy",
                "pulse_shaping", "vendor_power_word", "vendor_power_word_backend", nullptr
            };
            rd.reject_unknown_keys(g, p, kKeys);
            c.tx.port = static_cast<uint8_t>(rd.get_u8(g, p, "port"));
            c.tx.gain_db = rd.get_opt_double(g, p, "gain_db");
            c.tx.iq_amplitude = rd.get_opt_double(g, p, "iq_amplitude");
            c.tx.calibrated_tx_power_dbm = rd.get_opt_double(g, p,
                "calibrated_tx_power_dbm");
            c.tx.power_policy =
                rd.get_enum<TxPowerPolicy>(g, p, "power_policy",
                    tx_power_policy_from_string);
            c.tx.pulse_shaping =
                rd.get_enum<PulseShaping>(g, p, "pulse_shaping",
                    pulse_shaping_from_string);
            c.tx.vendor_power_word = rd.get_opt_u32(g, p, "vendor_power_word");
            c.tx.vendor_power_word_backend = rd.get_str(g, p,
                "vendor_power_word_backend");
        }
    }
    {
        const std::string p = "rx";
        const json::Value* g = rd.group(p, &root);
        if (g) {
            static const char* const kKeys[] = {
                "port", "gain_db", "agc", "bandwidth_hz", "detection_threshold",
                "correlation_threshold", "first_path_threshold", "first_path_index",
                "first_path_window", "vendor_pac_value", "vendor_pac_backend",
                "vendor_pac_applied_step", nullptr
            };
            rd.reject_unknown_keys(g, p, kKeys);
            c.rx.port = static_cast<uint8_t>(rd.get_u8(g, p, "port"));
            c.rx.gain_db = rd.get_opt_double(g, p, "gain_db");
            c.rx.agc = rd.get_enum<AgcMode>(g, p, "agc", agc_mode_from_string);
            c.rx.bandwidth_hz = rd.get_opt_double(g, p, "bandwidth_hz");
            c.rx.detection_threshold = rd.get_double(g, p, "detection_threshold");
            c.rx.correlation_threshold = rd.get_double(g, p, "correlation_threshold");
            c.rx.first_path_threshold = rd.get_double(g, p, "first_path_threshold");
            c.rx.first_path_index = static_cast<uint16_t>(rd.get_u16(g, p,
                "first_path_index"));
            c.rx.first_path_window = static_cast<uint16_t>(rd.get_u16(g, p,
                "first_path_window"));
            c.rx.vendor_pac_value = rd.get_opt_u32(g, p, "vendor_pac_value");
            c.rx.vendor_pac_backend = rd.get_str(g, p, "vendor_pac_backend");
            c.rx.vendor_pac_applied_step = rd.get_opt_double(g, p,
                "vendor_pac_applied_step");
        }
    }
    {
        const std::string p = "radio";
        const json::Value* g = rd.group(p, &root);
        if (g) {
            static const char* const kKeys[] = {
                "device_args", "tx_channel", "rx_channel", "native_sample_rate_hz",
                "clock_source", "time_source", "fpga_image", "dpdk_config", "peers", "readback",
                "require_readback", nullptr
            };
            rd.reject_unknown_keys(g, p, kKeys);
            c.radio.device_args = rd.get_str(g, p, "device_args");
            c.radio.tx_channel = static_cast<uint8_t>(rd.get_u8(g, p, "tx_channel"));
            c.radio.rx_channel = static_cast<uint8_t>(rd.get_u8(g, p, "rx_channel"));
            c.radio.native_sample_rate_hz = rd.get_double(g, p,
                "native_sample_rate_hz");
            c.radio.clock_source = rd.get_str(g, p, "clock_source");
            c.radio.time_source = rd.get_str(g, p, "time_source");
            c.radio.fpga_image = rd.get_str(g, p, "fpga_image");
            c.radio.dpdk_config = rd.get_str(g, p, "dpdk_config");
            const json::Value* peers = g->find("peers");
            if (!peers)
                rd.bad(p + ".peers", ConfigReason::MissingKey, "missing required key");
            else if (peers->type != json::Type::Array) {
                rd.bad(p + ".peers", ConfigReason::TypeMismatch, "expected an array");
            } else {
                for (size_t i = 0; i < peers->items.size(); ++i) {
                    const std::string pp = p + ".peers[" + twr_int_to_text(
                                                        static_cast<int64_t>(i)) + "]";
                    const json::Value& e = peers->items[i];
                    if (e.type != json::Type::Object) {
                        rd.bad(pp, ConfigReason::TypeMismatch, "expected an object");
                        continue;
                    }
                    static const char* const kPeer[] = { "id", "role", "tx_channel",
                        "rx_channel",
                                                         "native_sample_rate_hz",
                                                         "occupies_resources", nullptr };
                    rd.reject_unknown_keys(&e, pp, kPeer);
                    EndpointBinding b;
                    b.id = rd.get_str(&e, pp, "id");
                    b.role = rd.get_enum<Role>(&e, pp, "role", role_from_string);
                    b.tx_channel = static_cast<uint8_t>(rd.get_u8(&e, pp,
                        "tx_channel"));
                    b.rx_channel = static_cast<uint8_t>(rd.get_u8(&e, pp,
                        "rx_channel"));
                    b.native_sample_rate_hz = rd.get_double(&e, pp,
                        "native_sample_rate_hz");
                    b.occupies_resources = rd.get_bool(&e, pp, "occupies_resources");
                    c.radio.peers.push_back(b);
                }
            }
            const json::Value* rb = g->find("readback");
            if (!rb)
                rd.bad(p + ".readback", ConfigReason::MissingKey,
                    "missing required group");
            else if (rb->type != json::Type::Object)
                rd.bad(p + ".readback", ConfigReason::TypeMismatch,
                    "expected an object");
            else {
                const std::string rp = p + ".readback";
                static const char* const kRb[] = { "present", "sample_rate_hz",
                    "center_freq_hz",
                                                    "tx_channel", "rx_channel", "mpm_string",
                                                    "fpga_image", "uhd_version", "clock_source",
                                                    "time_source", nullptr };
                rd.reject_unknown_keys(rb, rp, kRb);
                c.radio.readback.present = rd.get_bool(rb, rp, "present");
                c.radio.readback.sample_rate_hz = rd.get_double(rb, rp,
                    "sample_rate_hz");
                c.radio.readback.center_freq_hz = rd.get_double(rb, rp,
                    "center_freq_hz");
                c.radio.readback.tx_channel = static_cast<uint8_t>(rd.get_u8(rb, rp,
                    "tx_channel"));
                c.radio.readback.rx_channel = static_cast<uint8_t>(rd.get_u8(rb, rp,
                    "rx_channel"));
                c.radio.readback.mpm_string = rd.get_str(rb, rp, "mpm_string");
                c.radio.readback.fpga_image = rd.get_str(rb, rp, "fpga_image");
                c.radio.readback.uhd_version = rd.get_str(rb, rp, "uhd_version");
                c.radio.readback.clock_source = rd.get_str(rb, rp, "clock_source");
                c.radio.readback.time_source = rd.get_str(rb, rp, "time_source");
            }
            c.radio.require_readback = rd.get_bool(g, p, "require_readback");
        }
    }
    {
        const std::string p = "timing";
        const json::Value* g = rd.group(p, &root);
        if (g) {
            static const char* const kKeys[] = { "poll_start", "poll_to_response",
                                                 "response_to_final", "final_to_report",
                                                 "post_tx_rx_enable", "min_tx_lead_time", nullptr };
            rd.reject_unknown_keys(g, p, kKeys);
            c.timing.poll_start = rd.get_timed(g, p, "poll_start");
            c.timing.poll_to_response = rd.get_timed(g, p, "poll_to_response");
            c.timing.response_to_final = rd.get_timed(g, p, "response_to_final");
            c.timing.final_to_report = rd.get_timed(g, p, "final_to_report");
            c.timing.post_tx_rx_enable = rd.get_timed(g, p, "post_tx_rx_enable");
            c.timing.min_tx_lead_time = rd.get_timed(g, p, "min_tx_lead_time");
        }
    }
    {
        const std::string p = "timeouts";
        const json::Value* g = rd.group(p, &root);
        if (g) {
            static const char* const kKeys[] = { "poll_rx_window", "response_rx_window",
                                                 "final_rx_window", "report_rx_window",
                                                 "rx_timeout", "exchange_timeout", "retry_interval",
                                                 nullptr };
            rd.reject_unknown_keys(g, p, kKeys);
            c.timeouts.poll_rx_window = rd.get_timed(g, p, "poll_rx_window");
            c.timeouts.response_rx_window = rd.get_timed(g, p, "response_rx_window");
            c.timeouts.final_rx_window = rd.get_timed(g, p, "final_rx_window");
            c.timeouts.report_rx_window = rd.get_timed(g, p, "report_rx_window");
            c.timeouts.rx_timeout = rd.get_timed(g, p, "rx_timeout");
            c.timeouts.exchange_timeout = rd.get_timed(g, p, "exchange_timeout");
            c.timeouts.retry_interval = rd.get_timed(g, p, "retry_interval");
        }
    }
    {
        const std::string p = "calibration";
        const json::Value* g = rd.group(p, &root);
        if (g) {
            static const char* const kKeys[] = {
                "tx_link_delay_ns", "rx_link_delay_ns", "antenna_delay_ns", "cable_delay_ns",
                "tx_link_delay_native_ticks", "rx_link_delay_native_ticks",
                "native_sample_rate_hz", "link_delay_unit", "first_path_algorithm",
                "cfo_compensation", "sfo_compensation", "calibration_id", "record",
                "applied_count", "calibration_required", nullptr
            };
            rd.reject_unknown_keys(g, p, kKeys);
            c.calibration.tx_link_delay = rd.get_dur(g, p, "tx_link_delay_ns");
            c.calibration.rx_link_delay = rd.get_dur(g, p, "rx_link_delay_ns");
            c.calibration.antenna_delay = rd.get_dur(g, p, "antenna_delay_ns");
            c.calibration.cable_delay = rd.get_dur(g, p, "cable_delay_ns");
            c.calibration.tx_link_delay_native_ticks =
                rd.get_i64(g, p, "tx_link_delay_native_ticks");
            c.calibration.rx_link_delay_native_ticks =
                rd.get_i64(g, p, "rx_link_delay_native_ticks");
            c.calibration.native_sample_rate_hz = rd.get_double(g, p,
                "native_sample_rate_hz");
            c.calibration.link_delay_unit =
                rd.get_enum<TimeUnit>(g, p, "link_delay_unit", time_unit_from_string);
            c.calibration.first_path_algorithm = rd.get_enum<FirstPathAlgorithm>(
                g, p, "first_path_algorithm", first_path_algorithm_from_string);
            c.calibration.cfo_compensation =
                rd.get_enum<CompensationFlag>(g, p, "cfo_compensation",
                    compensation_flag_from_string);
            c.calibration.sfo_compensation =
                rd.get_enum<CompensationFlag>(g, p, "sfo_compensation",
                    compensation_flag_from_string);
            c.calibration.calibration_id = rd.get_str(g, p, "calibration_id");
            const json::Value* rec = g->find("record");
            if (!rec)
                rd.bad(p + ".record", ConfigReason::MissingKey,
                    "missing required group");
            else if (rec->type != json::Type::Object)
                rd.bad(p + ".record", ConfigReason::TypeMismatch, "expected an object");
            else {
                const std::string rp = p + ".record";
                static const char* const kRec[] = { "calibration_id", "device_serial",
                    "channel",
                                                    "native_sample_rate_hz", "profile_version",
                                                    "gain_db", "valid_until_monotonic_ns",
                                                    nullptr };
                rd.reject_unknown_keys(rec, rp, kRec);
                c.calibration.record.calibration_id = rd.get_str(rec, rp,
                    "calibration_id");
                c.calibration.record.device_serial = rd.get_str(rec, rp,
                    "device_serial");
                c.calibration.record.channel = static_cast<uint8_t>(rd.get_u8(rec, rp,
                    "channel"));
                c.calibration.record.native_sample_rate_hz =
                    rd.get_double(rec, rp, "native_sample_rate_hz");
                c.calibration.record.profile_version = rd.get_str(rec, rp,
                    "profile_version");
                c.calibration.record.gain_db = rd.get_opt_double(rec, rp, "gain_db");
                c.calibration.record.valid_until_monotonic_ns =
                    rd.get_i64(rec, rp, "valid_until_monotonic_ns");
            }
            c.calibration.applied_count = static_cast<uint32_t>(rd.get_u32(g, p,
                "applied_count"));
            c.calibration.calibration_required = rd.get_bool(g, p,
                "calibration_required");
        }
    }
    {
        const std::string p = "diagnostics";
        const json::Value* g = rd.group(p, &root);
        if (g) {
            static const char* const kKeys[] = { "cir_capture_enabled",
                "cir_capture_max_bytes",
                                                 "cir_capture_stride", "short_iq_enabled",
                                                 "short_iq_max_bytes", "short_iq_stride",
                                                 "raw_frame_dump", "raw_frame_max_bytes",
                                                 "result_output_path", "result_queue_capacity",
                                                 "event_queue_capacity", "stats_cadence",
                                                 "io_on_realtime_thread", nullptr };
            rd.reject_unknown_keys(g, p, kKeys);
            c.diagnostics.cir_capture_enabled = rd.get_bool(g, p,
                "cir_capture_enabled");
            c.diagnostics.cir_capture_max_bytes =
                static_cast<uint64_t>(rd.get_u64(g, p, "cir_capture_max_bytes", 0xFFFFFFFFFFFFFFFFull));
            c.diagnostics.cir_capture_stride =
                static_cast<uint32_t>(rd.get_u32(g, p, "cir_capture_stride"));
            c.diagnostics.short_iq_enabled = rd.get_bool(g, p, "short_iq_enabled");
            c.diagnostics.short_iq_max_bytes =
                static_cast<uint64_t>(rd.get_u64(g, p, "short_iq_max_bytes", 0xFFFFFFFFFFFFFFFFull));
            c.diagnostics.short_iq_stride =
                static_cast<uint32_t>(rd.get_u32(g, p, "short_iq_stride"));
            c.diagnostics.raw_frame_dump = rd.get_bool(g, p, "raw_frame_dump");
            c.diagnostics.raw_frame_max_bytes =
                static_cast<uint64_t>(rd.get_u64(g, p, "raw_frame_max_bytes", 0xFFFFFFFFFFFFFFFFull));
            c.diagnostics.result_output_path = rd.get_str(g, p, "result_output_path");
            c.diagnostics.result_queue_capacity =
                static_cast<uint32_t>(rd.get_u32(g, p, "result_queue_capacity"));
            c.diagnostics.event_queue_capacity =
                static_cast<uint32_t>(rd.get_u32(g, p, "event_queue_capacity"));
            c.diagnostics.stats_cadence = rd.get_timed(g, p, "stats_cadence");
            c.diagnostics.io_on_realtime_thread = rd.get_bool(g, p,
                "io_on_realtime_thread");
        }
    }

    // Even a document that failed to import is handed back so the caller can
    // see which groups were readable; the report is the authority.
    out = c;
    return rd.report();
}

} // namespace twr
} // namespace uwb
} // namespace gr

#endif /* INCLUDED_OPENUWB_TWR_CONFIG_EFFECTIVE_H */
