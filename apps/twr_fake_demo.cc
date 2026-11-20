/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * M1-B demo CLI: run the deterministic TWO-ENDPOINT SS/DS-TWR simulation and
 * emit a structured, machine-readable result.
 *
 * ---------------------------------------------------------------------------
 * WHAT THIS IS
 * ---------------------------------------------------------------------------
 * A self-contained, pure C++ command line program.  It links NO gnuradio-uwb
 * library and includes NO GNU Radio / UHD / PMT header.  The only project
 * headers it uses are header-only:
 *
 *     uwb_twr_core.h          the real SS/DS endpoint FSM (two instances)
 *     uwb_twr_fake_link.h     the deterministic transport
 *     uwb_twr_config.h        the JSON reader/writer and the existing
 *                             TwrConfig validator (used if a TwrConfig is
 *                             supplied in the request)
 *
 * It does NOT contain a second state machine: the two endpoints ARE the real
 * `EndpointCore`s and the transport IS the real `FakeTwrLink`.
 *
 * ---------------------------------------------------------------------------
 * WHAT THE NUMBERS MEAN (and do not mean)
 * ---------------------------------------------------------------------------
 * The request declares `simulation.execution = "offline_simulation"` and
 * `simulation.mode = "protocol_estimate"`.  This program REFUSES to run when
 * either is missing or different, so a scenario can never silently claim to be
 * hardware ranging.  Every result carries:
 *
 *     execution_mode       == "simulation"
 *     peer_evidence        == "wire_claim"
 *     measurement_valid    == false
 *     tof                  exact signed rational ticks, NOT a distance
 *
 * and the endpoint APIs additionally report `yields_range() == false`,
 * `is_hardware_measurement() == false`, `is_validated_measurement() == false`.
 * No distance is ever printed.  A distance lives ONLY in the request's
 * `link.distance_m`, where it drives the fake transport's propagation model;
 * the observer-readable echo of that model is under `transport_model` and is
 * explicitly labelled as not an endpoint input.
 *
 * ---------------------------------------------------------------------------
 * REQUEST SCHEMA  ("twr-m1b-demo/1")
 * ---------------------------------------------------------------------------
 * {
 *   "schema": "twr-m1b-demo/1",
 *   "simulation": {"execution":"offline_simulation",
 *                  "mode":"protocol_estimate","seed":1,"scenario_id":"..."},
 *   "protocol": "ss" | "ds",
 *   "session": {"session_id":7,"session_generation":1,
 *               "sequence_modulus":4,"initial_sequence":0,"pan_id":4660},
 *   "wire": {"timestamp_bits":40,"timestamp_unit_hz":1000000000,
 *            "max_interval_ticks":1000000},
 *   "endpoints": {"a":{"local_address":257,"peer_address":514},
 *                 "b":{"local_address":514,"peer_address":257}},
 *   "domains": {"a":{"name":"...","tick_rate_hz":1e9,"epoch_id":1,
 *                    "timestamp_bits":0},
 *               "b":{...}},
 *   "clock_ratio": {"kind":"unity_same_clock" | "nominal_rate_ratio",
 *                   "fA_hz":..., "fB_hz":...},
 *   "calibration": {"id":"cal1","calibrated_epoch":1,
 *                   "valid_from_ticks":0,"valid_until_ticks":1000000000},
 *   "link": {"distance_m":29.9792458,
 *            "tx_air_latency_a_ticks":0,"tx_air_latency_b_ticks":0},
 *   "timeline": {"poll_air_ticks":2000,"response_reply_ticks":500,
 *                "final_reply_ticks":300},
 *   "twr_config": { ... optional existing TwrConfig JSON ... }
 * }
 *
 * `wire.timestamp_bits` is the frame profile field width.  `wire.
 * timestamp_unit_hz` is cross-checked against the local rate for the
 * same-clock case; for an independent-clock ratio each endpoint encodes and
 * interprets wire fields in its OWN domain, which is the whole point of the
 * session-level `WireTimestampBinding`.  `twr_config`, when present, is parsed
 * by the EXISTING `from_json_string` + `validate()` and frozen with
 * `effective_config()`; the endpoint cores are then built FROM that validated
 * snapshot (R09), and the run prints an explicit requested -> effective -> core
 * mapping.  A field the envelope also states must AGREE with the config -- a
 * conflict refuses the whole run -- and every requested field the offline core
 * does not consume is listed with its use and limit.  A rejection refuses the
 * whole run (B17: "demo rejects a wrong config").
 *
 * `config_sha256` / `profile_sha256` are taken over the ACTUAL executed values,
 * not over the request: the output publishes the canonical path/value list they
 * were computed from, so an independent verifier can recompute them and catch a
 * mutation.
 *
 * CLI:
 *     twr_fake_demo --request <in.json> --output <out.json>
 * `--output -` writes the machine JSON to stdout; logs always go to stderr.
 * Integers above 2^53 are emitted as decimal STRINGS by the shared dumper.
 *
 * BUILD (out of tree):
 *     g++ -std=c++17 -Wall -Wextra -I gr-uwb/include \
 *         -o twr_fake_demo gr-uwb/apps/twr_fake_demo.cc \
 *         gr-uwb/lib/uwb_twr_core.cc gr-uwb/lib/uwb_twr_fake_link.cc
 */

#include <gnuradio/uwb/uwb_twr_config.h>
#include <gnuradio/uwb/uwb_twr_core.h>
#include <gnuradio/uwb/uwb_twr_fake_link.h>

#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

namespace twr = gr::uwb::twr;
using twr::json::Type;
using twr::json::Value;

// ===========================================================================
// SHA-256 (self-contained; the repo has no crypto/JSON-library dependency)
// ===========================================================================
namespace demo {

class Sha256
{
public:
    Sha256() { reset(); }

    void reset()
    {
        d_h[0] = 0x6a09e667u;
        d_h[1] = 0xbb67ae85u;
        d_h[2] = 0x3c6ef372u;
        d_h[3] = 0xa54ff53au;
        d_h[4] = 0x510e527fu;
        d_h[5] = 0x9b05688cu;
        d_h[6] = 0x1f83d9abu;
        d_h[7] = 0x5be0cd19u;
        d_buf_len = 0;
        d_total = 0;
    }

    void update(const uint8_t* data, size_t len)
    {
        d_total += static_cast<uint64_t>(len);
        for (size_t i = 0; i < len; ++i) {
            d_buf[d_buf_len++] = data[i];
            if (d_buf_len == 64) {
                process(d_buf);
                d_buf_len = 0;
            }
        }
    }

    void update(const std::string& s)
    {
        update(reinterpret_cast<const uint8_t*>(s.data()), s.size());
    }

    std::string hex()
    {
        Sha256 copy = *this;
        return copy.finish_hex();
    }

private:
    static uint32_t rotr(uint32_t x, int n) { return (x >> n) | (x << (32 - n)); }

    static const uint32_t* k()
    {
        static const uint32_t K[64] = {
            0x428a2f98u, 0x71374491u, 0xb5c0fbcfu, 0xe9b5dba5u, 0x3956c25bu,
            0x59f111f1u, 0x923f82a4u, 0xab1c5ed5u, 0xd807aa98u, 0x12835b01u,
            0x243185beu, 0x550c7dc3u, 0x72be5d74u, 0x80deb1feu, 0x9bdc06a7u,
            0xc19bf174u, 0xe49b69c1u, 0xefbe4786u, 0x0fc19dc6u, 0x240ca1ccu,
            0x2de92c6fu, 0x4a7484aau, 0x5cb0a9dcu, 0x76f988dau, 0x983e5152u,
            0xa831c66du, 0xb00327c8u, 0xbf597fc7u, 0xc6e00bf3u, 0xd5a79147u,
            0x06ca6351u, 0x14292967u, 0x27b70a85u, 0x2e1b2138u, 0x4d2c6dfcu,
            0x53380d13u, 0x650a7354u, 0x766a0abbu, 0x81c2c92eu, 0x92722c85u,
            0xa2bfe8a1u, 0xa81a664bu, 0xc24b8b70u, 0xc76c51a3u, 0xd192e819u,
            0xd6990624u, 0xf40e3585u, 0x106aa070u, 0x19a4c116u, 0x1e376c08u,
            0x2748774cu, 0x34b0bcb5u, 0x391c0cb3u, 0x4ed8aa4au, 0x5b9cca4fu,
            0x682e6ff3u, 0x748f82eeu, 0x78a5636fu, 0x84c87814u, 0x8cc70208u,
            0x90befffau, 0xa4506cebu, 0xbef9a3f7u, 0xc67178f2u
        };
        return K;
    }

    static void be32(uint32_t v, uint8_t* out)
    {
        out[0] = static_cast<uint8_t>(v >> 24);
        out[1] = static_cast<uint8_t>(v >> 16);
        out[2] = static_cast<uint8_t>(v >> 8);
        out[3] = static_cast<uint8_t>(v);
    }

    void process(const uint8_t* p)
    {
        uint32_t w[64];
        for (int i = 0; i < 16; ++i) {
            w[i] = (static_cast<uint32_t>(p[4 * i]) << 24) |
                   (static_cast<uint32_t>(p[4 * i + 1]) << 16) |
                   (static_cast<uint32_t>(p[4 * i + 2]) << 8) |
                   static_cast<uint32_t>(p[4 * i + 3]);
        }
        for (int i = 16; i < 64; ++i) {
            const uint32_t s0 = rotr(w[i - 15], 7) ^ rotr(w[i - 15], 18) ^ (w[i - 15] >> 3);
            const uint32_t s1 = rotr(w[i - 2], 17) ^ rotr(w[i - 2], 19) ^ (w[i - 2] >> 10);
            w[i] = w[i - 16] + s0 + w[i - 7] + s1;
        }
        uint32_t a = d_h[0], b = d_h[1], c = d_h[2], d = d_h[3];
        uint32_t e = d_h[4], f = d_h[5], g = d_h[6], h = d_h[7];
        const uint32_t* K = k();
        for (int i = 0; i < 64; ++i) {
            const uint32_t S1 = rotr(e, 6) ^ rotr(e, 11) ^ rotr(e, 25);
            const uint32_t ch = (e & f) ^ (~e & g);
            const uint32_t t1 = h + S1 + ch + K[i] + w[i];
            const uint32_t S0 = rotr(a, 2) ^ rotr(a, 13) ^ rotr(a, 22);
            const uint32_t maj = (a & b) ^ (a & c) ^ (b & c);
            const uint32_t t2 = S0 + maj;
            h = g;
            g = f;
            f = e;
            e = d + t1;
            d = c;
            c = b;
            b = a;
            a = t1 + t2;
        }
        d_h[0] += a;
        d_h[1] += b;
        d_h[2] += c;
        d_h[3] += d;
        d_h[4] += e;
        d_h[5] += f;
        d_h[6] += g;
        d_h[7] += h;
    }

    std::string finish_hex()
    {
        const uint64_t bits = d_total * 8u;
        uint8_t pad = 0x80u;
        update(&pad, 1);
        const uint8_t zero = 0u;
        while (d_buf_len != 56)
            update(&zero, 1);
        uint8_t lenbuf[8];
        for (int i = 0; i < 8; ++i)
            lenbuf[i] = static_cast<uint8_t>(bits >> (8 * (7 - i)));
        update(lenbuf, 8);
        std::string out;
        out.reserve(64);
        static const char* hx = "0123456789abcdef";
        for (int i = 0; i < 8; ++i) {
            for (int j = 3; j >= 0; --j) {
                const uint8_t byte = static_cast<uint8_t>(d_h[i] >> (8 * j));
                out.push_back(hx[byte >> 4]);
                out.push_back(hx[byte & 0xf]);
            }
        }
        return out;
    }

    uint32_t d_h[8];
    uint8_t d_buf[64];
    size_t d_buf_len;
    uint64_t d_total;
};

std::string sha256_hex(const std::string& data)
{
    Sha256 s;
    s.update(data);
    return s.hex();
}

std::string read_file(const std::string& path, std::string& why)
{
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        why = "cannot open " + path;
        return std::string();
    }
    std::ostringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

std::string read_stream(std::istream& in)
{
    std::ostringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

std::string hex_bytes(const uint8_t* p, size_t n)
{
    static const char* hx = "0123456789abcdef";
    std::string s;
    s.reserve(n * 2);
    for (size_t i = 0; i < n; ++i) {
        s.push_back(hx[p[i] >> 4]);
        s.push_back(hx[p[i] & 0xf]);
    }
    return s;
}

// Small decimal formatting helpers for the config mapping view.
std::string fmt_u16(uint16_t v)
{
    return twr::twr_int_to_text(static_cast<int64_t>(v));
}
std::string fmt_u32(uint32_t v)
{
    return twr::twr_int_to_text(static_cast<int64_t>(v));
}

// ===========================================================================
// Small typed JSON readers.  Every failure names the offending path.
// ===========================================================================

bool as_object(const Value& v, const std::string& path, std::string& why)
{
    if (v.type != Type::Object) {
        why = path + " must be a JSON object, got " + v.type_name();
        return false;
    }
    return true;
}

const Value* need(const Value& obj, const char* key, const std::string& path, std::string& why)
{
    const Value* v = obj.find(key);
    if (v == nullptr) {
        why = path + "." + key + " is missing";
        return nullptr;
    }
    return v;
}

bool get_i64(const Value& obj, const char* key, const std::string& path, int64_t& out,
             std::string& why)
{
    const Value* v = need(obj, key, path, why);
    if (v == nullptr)
        return false;
    if (v->type == Type::Int) {
        out = v->integer;
        return true;
    }
    if (v->type == Type::String) {
        char* end = nullptr;
        errno = 0;
        const long long r = std::strtoll(v->text.c_str(), &end, 10);
        if (errno == 0 && end != nullptr && *end == '\0') {
            out = static_cast<int64_t>(r);
            return true;
        }
    }
    why = path + "." + key + " must be an integer (or decimal string), got " + v->type_name();
    return false;
}

bool get_u64(const Value& obj, const char* key, const std::string& path, uint64_t& out,
             std::string& why)
{
    int64_t s = 0;
    if (!get_i64(obj, key, path, s, why))
        return false;
    if (s < 0) {
        why = path + "." + key + " must be >= 0";
        return false;
    }
    out = static_cast<uint64_t>(s);
    return true;
}

bool get_u16(const Value& obj, const char* key, const std::string& path, uint16_t& out,
             std::string& why)
{
    uint64_t v = 0;
    if (!get_u64(obj, key, path, v, why))
        return false;
    if (v > 0xFFFFu) {
        why = path + "." + key + " exceeds 16 bits";
        return false;
    }
    out = static_cast<uint16_t>(v);
    return true;
}

bool get_double(const Value& obj, const char* key, const std::string& path, double& out,
                std::string& why)
{
    const Value* v = need(obj, key, path, why);
    if (v == nullptr)
        return false;
    if (v->type == Type::Double || v->type == Type::Int) {
        out = v->as_double();
        return true;
    }
    if (v->type == Type::String) {
        char* end = nullptr;
        errno = 0;
        const double r = std::strtod(v->text.c_str(), &end);
        if (errno == 0 && end != nullptr && *end == '\0') {
            out = r;
            return true;
        }
    }
    why = path + "." + key + " must be a number, got " + v->type_name();
    return false;
}

bool get_string(const Value& obj, const char* key, const std::string& path, std::string& out,
                std::string& why)
{
    const Value* v = need(obj, key, path, why);
    if (v == nullptr)
        return false;
    if (v->type != Type::String) {
        why = path + "." + key + " must be a string, got " + v->type_name();
        return false;
    }
    out = v->text;
    return true;
}

bool get_object(const Value& obj, const char* key, const std::string& path, const Value*& out,
                std::string& why)
{
    const Value* v = need(obj, key, path, why);
    if (v == nullptr)
        return false;
    if (v->type != Type::Object) {
        why = path + "." + key + " must be an object, got " + v->type_name();
        return false;
    }
    out = v;
    return true;
}

// ===========================================================================
// The scenario
// ===========================================================================

// R09: one row of the requested -> effective -> core field mapping.  The C++
// side parses and validates the requested TwrConfig with the EXISTING parser
// and validator, then builds the core snapshot FROM the validated values.  A
// row records where each value came from so a consumer can see that the config
// (not the discarded check) drove the run.
struct MappingRow {
    std::string field;     // logical name, e.g. "session.protocol"
    std::string requested; // the requested view (config JSON, or the envelope)
    std::string effective; // the validated/effective value
    std::string core;      // the value the endpoint core actually executed
    std::string source;    // "twr_config" | "envelope"
    bool consumed = true;  // false: recorded use/limit, not executed by M1-B
    std::string note;      // the conversion / limit, stated rather than implied
};

// R09: a requested field the OFFLINE core does not consume.  It is listed with
// its intended use and the limit of that use here; nothing is silently ignored.
struct UnconsumedRow {
    std::string field;
    std::string use;
    std::string limit;
};

struct Scenario {
    // simulation gate
    std::string execution;
    std::string mode;
    uint64_t seed = 1;
    std::string scenario_id;

    twr::Protocol protocol = twr::Protocol::Ss;

    // ---- envelope-stated values (the conflict view) ----------------------
    uint16_t session_id = 0;
    uint64_t session_generation = 1;
    uint16_t sequence_modulus = 4;
    uint16_t initial_sequence = 0;
    uint16_t pan_id = 0;

    uint8_t wire_timestamp_bits = 40;
    double wire_timestamp_unit_hz = 1.0e9;
    uint64_t max_interval_ticks = 0;

    uint16_t a_local = 0, a_peer = 0, b_local = 0, b_peer = 0;

    twr::ClockDomain dom_a, dom_b;
    twr::ClockRatio ratio;

    twr::CalibrationStamp cal;
    twr::FrameProfile profile_a, profile_b;

    double distance_m = 0.0;
    int64_t tx_latency_a = 0;
    int64_t tx_latency_b = 0;

    int64_t poll_air_ticks = 0;
    int64_t response_reply_ticks = 0;
    int64_t final_reply_ticks = 0;

    // ---- R09 execution authority -----------------------------------------
    //
    // When `twr_config` is present it is parsed with `twr::from_json_string`,
    // validated with `twr::validate()` and frozen with
    // `twr::effective_config()`; the fields below are then taken FROM that
    // validated snapshot (with an explicit mapping), and a value that
    // contradicts the envelope is a refusal, never a silent override.  When it
    // is absent the envelope values + documented core defaults are used.
    bool has_twr_config = false;
    Value twr_config; // the raw requested TwrConfig JSON

    bool config_resolved = false;
    std::string requested_config_json; // "" when no twr_config was supplied
    std::string effective_config_json; // the frozen effective snapshot view
    std::string requested_config_fnv;  // twr::config_hash(requested)
    std::string effective_config_fnv;  // twr::config_hash(effective)

    // Effective execution values the two endpoint cores are built from.
    uint32_t max_in_flight_exchanges = 1;
    uint32_t result_queue_capacity = 8;
    uint32_t event_queue_capacity = 16;
    uint32_t exchange_id = 0;

    // The three timing budgets stay in their REQUESTED representation (value +
    // domain + reference + quantisation) and are projected onto each
    // endpoint's own local tick grid only when the core config is built.  That
    // way a two-rate scenario projects each endpoint correctly instead of
    // reusing the initiator's ticks.
    twr::TimedField reply_budget;    // timing.poll_to_response (reply budget)
    twr::TimedField exchange_budget; // timeouts.exchange_timeout (whole exchange)
    twr::TimedField evidence_budget; // timeouts.rx_timeout (wait for the missing step)

    // The simulated adapter's plan constants.  These are DECLARED, not
    // measured: M1-B has no device, so the numeric plan is generated from a
    // stated TX-chain marker offset and host command lead.
    int64_t plan_marker_offset_ticks = 64;
    int64_t plan_command_lead_ticks = 32;

    std::vector<MappingRow> mappings;
    std::vector<UnconsumedRow> unconsumed;
};

// Project a TimedField onto an endpoint's LOCAL tick grid.  This is the one
// place an offline timing budget becomes core ticks; the requested domain /
// reference is preserved in the mapping row so the projection is visible, not
// implied.  A zero value is "disabled" and stays zero.
inline int64_t project_timed_to_local_ticks(const twr::TimedField& f,
                                            const twr::ClockDomain& dom,
                                            bool& ok)
{
    ok = true;
    if (f.value.is_zero())
        return 0;
    if (f.value.negative()) {
        ok = false;
        return 0;
    }
    const int64_t ticks = twr::quantise_duration(f.value, dom.tick_rate_hz, ok);
    if (!ok || ticks < 0) {
        ok = false;
        return 0;
    }
    return ticks;
}

// R09: build the per-endpoint CoreConfig from the RESOLVED scenario.  Both the
// driver and the output builder call this ONE function, so the config the core
// executed and the config the output describes cannot drift apart.
twr::CoreConfig make_core_config(const Scenario& sc, twr::Role role)
{
    twr::CoreConfig c;
    c.endpoint_id = (role == twr::Role::Initiator) ? "A" : "B";
    c.protocol = sc.protocol;
    c.role = role;
    c.pan_id = sc.pan_id;
    c.local_address = (role == twr::Role::Initiator) ? sc.a_local : sc.b_local;
    c.peer_address = (role == twr::Role::Initiator) ? sc.a_peer : sc.b_peer;
    c.session_id = sc.session_id;
    c.session_generation = sc.session_generation;
    c.sequence_modulus = sc.sequence_modulus;
    c.initial_sequence = sc.initial_sequence;
    c.local_domain = (role == twr::Role::Initiator) ? sc.dom_a : sc.dom_b;
    c.peer_binding.peer_domain = (role == twr::Role::Initiator) ? sc.dom_b : sc.dom_a;
    c.peer_binding.session_generation = sc.session_generation;
    c.peer_binding.binding_generation = 1;
    c.peer_binding.peer_marker = twr::TimestampMarker::RmarkerTx;
    c.peer_binding.unit_convention = "device_ticks";
    c.peer_binding.calibration_convention_id = sc.cal.id;
    c.peer_binding.max_interval_ticks = sc.max_interval_ticks;
    c.peer_binding.sequence_modulus = sc.sequence_modulus;
    c.ratio = sc.ratio;
    c.frame_profile = (role == twr::Role::Initiator) ? sc.profile_a : sc.profile_b;
    c.local_calibration = sc.cal;
    bool ok = true;
    c.reply_deadline_ticks = project_timed_to_local_ticks(sc.reply_budget, c.local_domain, ok);
    c.exchange_timeout_ticks =
        project_timed_to_local_ticks(sc.exchange_budget, c.local_domain, ok);
    c.evidence_wait_ticks = project_timed_to_local_ticks(sc.evidence_budget, c.local_domain, ok);
    c.result_queue_capacity = sc.result_queue_capacity;
    c.max_in_flight = sc.max_in_flight_exchanges;
    return c;
}

// ===========================================================================
// Endpoint tracing
// ===========================================================================

inline int type_index(twr::FrameType t) { return static_cast<int>(t); }

struct FrameTrace {
    twr::FrameType type = twr::FrameType::Poll;
    std::string dir; // "tx" / "rx"
    std::string peer_id;
    int64_t ticks = 0; // planned air tick (tx) or RX marker (rx)
    uint64_t token = 0;
    uint8_t bytes[twr::kMaxFrameBytes] = {};
    size_t nbytes = 0;
    twr::Frame frame;
    bool decoded = false;

    // R08: the NUMERIC transmit plan this frame was BUILT from (tx frames
    // only).  Emitted so an independent verifier can recompute the plan's
    // identity from the trace alone.
    bool has_plan = false;
    twr::TxPlan plan;
    bool deadline_verdict_feasible = false;
    int64_t deadline_slack_ticks = 0;
};

// One ArmRx action, recorded so a verifier can see the deadline the core
// handed the driver (R07's driver contract; also how a timeout change is
// observable without inventing a failure).
struct ArmRxTrace {
    twr::FrameType expect_type = twr::FrameType::Response;
    int64_t deadline_ticks = 0;
};

struct EndpointTrace {
    std::string endpoint_id;
    twr::Role role = twr::Role::Initiator;
    uint16_t local_address = 0;
    uint16_t peer_address = 0;

    std::vector<FrameTrace> frames;
    std::vector<ArmRxTrace> arm_rx;

    bool tx_planned_set[4] = { false, false, false, false };
    int64_t tx_planned_ticks[4] = { 0, 0, 0, 0 };
    bool rx_marker_set[4] = { false, false, false, false };
    int64_t rx_marker_ticks[4] = { 0, 0, 0, 0 };

    bool have_terminal = false;
    twr::ProtocolTofEstimate result;
    // The product's strong terminal type -- not an ExchangeStatus, so this
    // struct cannot hold a range-compatible status either (F's finding B-1).
    twr::ProtocolTerminalStatus terminal;
    std::string terminal_detail;

    twr::CoreCounters counters;

    const FrameTrace* find(twr::FrameType t, const std::string& dir) const
    {
        for (const auto& f : frames)
            if (f.type == t && f.dir == dir)
                return &f;
        return nullptr;
    }
};

// ===========================================================================
// The driver
// ===========================================================================

class DemoDriver
{
public:
    explicit DemoDriver(const Scenario& sc) : sc_(sc), link_() {}

    bool run(std::string& why)
    {
        twr::FakeLinkConfig lc;
        lc.domain_a = sc_.dom_a;
        lc.domain_b = sc_.dom_b;
        lc.distance_m = sc_.distance_m;
        lc.tx_air_latency_a_ticks = sc_.tx_latency_a;
        lc.tx_air_latency_b_ticks = sc_.tx_latency_b;
        lc.rx_queue_capacity = 8;
        lc.seed = sc_.seed;
        lc.drop_every_n_tx = 0;
        lc.bad_fcs_every_n_rx = 0;
        if (!link_.configure(lc, why))
            return false;

        twr::CoreConfig ca = make_core_config(sc_, twr::Role::Initiator);
        twr::CoreConfig cb = make_core_config(sc_, twr::Role::Responder);
        if (!core_a_.configure(ca, why))
            return false;
        if (!core_b_.configure(cb, why))
            return false;

        ta_.endpoint_id = "A";
        ta_.role = twr::Role::Initiator;
        ta_.local_address = sc_.a_local;
        ta_.peer_address = sc_.a_peer;
        tb_.endpoint_id = "B";
        tb_.role = twr::Role::Responder;
        tb_.local_address = sc_.b_local;
        tb_.peer_address = sc_.b_peer;

        // --- 1. A begins; the Poll plan, submit and outcome are all driven by
        //        the prepared token, and the link computes its arrival.
        {
            twr::CoreEvent ev;
            ev.kind = twr::CoreEventKind::Begin;
            ev.generation = sc_.session_generation;
            ev.exchange.valid = true;
            // R09: the requested exchange id is an execution value.  The config
            // validator allows 0 (the default), which is not a legal runtime
            // identity, so 0 becomes the documented runtime default of 1.
            ev.exchange.value = sc_.exchange_id != 0 ? sc_.exchange_id : 1;
            ev.event_id = 1;
            ev.now_ticks = 0;
            twr::CoreActionBatch b = core_a_.post(ev);
            if (!handle(core_a_, ta_, twr::kEndpointA, b, why))
                return false;
        }

        // --- 2. The responder receives the Poll (arrival from the transport).
        if (!deliver_one(twr::kEndpointB, why))
            return false;

        // --- 3. The initiator receives the Response.
        if (!deliver_one(twr::kEndpointA, why))
            return false;

        // --- 4. DS: the responder receives the Final.
        if (sc_.protocol == twr::Protocol::Ds) {
            if (!deliver_one(twr::kEndpointB, why))
                return false;
        }

        ta_.counters = core_a_.counters();
        tb_.counters = core_b_.counters();
        return true;
    }

    const EndpointTrace& trace_a() const { return ta_; }
    const EndpointTrace& trace_b() const { return tb_; }

private:
    const twr::ClockDomain& domain_of(uint8_t ep) const
    {
        return ep == twr::kEndpointA ? sc_.dom_a : sc_.dom_b;
    }

    const twr::FrameProfile& profile_of(uint8_t ep) const
    {
        return ep == twr::kEndpointA ? sc_.profile_a : sc_.profile_b;
    }

    twr::Timestamp rx_timestamp(int64_t ticks, const twr::ClockDomain& dom) const
    {
        twr::Timestamp ts;
        twr::Timestamp::from_ticks(ticks, dom, twr::TimestampMarker::RmarkerRx,
                                   twr::TimestampSource::HardwareMeasured,
                                   twr::timestamp_required_corrections(
                                       twr::TimestampMarker::RmarkerRx),
                                   ts);
        ts.calibration_id = sc_.cal.id;
        return ts;
    }

    // R08: build the NUMERIC transmit plan the core will validate.  The
    // internal identity the core checks is
    //
    //     calibrated_air_ticks == quantised_instant_ticks + marker_offset_ticks
    //
    // and every tick must be inside the endpoint's own domain.  The simulated
    // adapter therefore derives the plan from the desired air instant:
    //
    //   marker_offset_ticks   = min(declared offset, air)        >= 0
    //   quantised_instant     = air - marker_offset               >= 0
    //   command_time_ticks    = quantised - min(declared lead, quantised) >= 0
    //
    // The declared offset/lead are constants of THIS simulated adapter (M1-B
    // has no device to measure them); they are printed in the output and are
    // not a claim about any hardware.  The plan carries NO sent/completed
    // state: the outcome is a separate event.
    twr::TxPlan make_tx_plan(int64_t air_ticks, const twr::ClockDomain& dom) const
    {
        twr::TxPlan p;
        p.domain = dom;
        p.source = twr::TimestampSource::ScheduledCalibrated;
        p.applied_corrections =
            twr::timestamp_required_corrections(twr::TimestampMarker::RmarkerTx);
        p.calibration_id = sc_.cal.id;
        p.calibrated_air_ticks = air_ticks;
        p.marker_offset_ticks =
            air_ticks < sc_.plan_marker_offset_ticks ? air_ticks : sc_.plan_marker_offset_ticks;
        p.quantised_instant_ticks = air_ticks - p.marker_offset_ticks;
        const int64_t lead = p.quantised_instant_ticks < sc_.plan_command_lead_ticks
                                 ? p.quantised_instant_ticks
                                 : sc_.plan_command_lead_ticks;
        p.command_time_ticks = p.quantised_instant_ticks - lead;
        p.valid = true;
        return p;
    }

    bool choose_planned(const EndpointTrace& tr, twr::FrameType intent, int64_t& out,
                        std::string& why) const
    {
        switch (intent) {
        case twr::FrameType::Poll:
            out = sc_.poll_air_ticks;
            return true;
        case twr::FrameType::Response:
            if (!tr.rx_marker_set[type_index(twr::FrameType::Poll)]) {
                why = "Response plan requested before the Poll RX marker was recorded";
                return false;
            }
            out = tr.rx_marker_ticks[type_index(twr::FrameType::Poll)] +
                  sc_.response_reply_ticks;
            return true;
        case twr::FrameType::Final:
            if (!tr.rx_marker_set[type_index(twr::FrameType::Response)]) {
                why = "Final plan requested before the Response RX marker was recorded";
                return false;
            }
            out = tr.rx_marker_ticks[type_index(twr::FrameType::Response)] +
                  sc_.final_reply_ticks;
            return true;
        case twr::FrameType::Report:
            why = "report frames are not implemented";
            return false;
        }
        why = "unknown tx intent";
        return false;
    }

    bool deliver_one(uint8_t dest, std::string& why)
    {
        twr::FakeRx rx;
        if (!link_.pop_rx(dest, rx)) {
            why = std::string("no queued RX event for endpoint ") +
                  (dest == twr::kEndpointA ? "A" : "B") +
                  " (an expected frame never arrived)";
            return false;
        }
        return deliver(dest, rx, why);
    }

    bool deliver(uint8_t dest, const twr::FakeRx& rx, std::string& why)
    {
        twr::EndpointCore& core = (dest == twr::kEndpointA) ? core_a_ : core_b_;
        EndpointTrace& tr = (dest == twr::kEndpointA) ? ta_ : tb_;

        twr::Frame f;
        bool decoded = false;
        std::string derr;
        if (rx.decode_ok && rx.nbytes > 0) {
            decoded = twr::decode(rx.bytes, rx.nbytes, profile_of(dest), f, derr);
        }

        FrameTrace ft;
        ft.type = decoded ? f.function_code : twr::FrameType::Poll;
        ft.dir = "rx";
        ft.peer_id = (rx.source == twr::kEndpointA) ? "A" : "B";
        ft.ticks = rx.rx_marker_ticks;
        ft.token = rx.token;
        std::memcpy(ft.bytes, rx.bytes, rx.nbytes);
        ft.nbytes = rx.nbytes;
        ft.frame = f;
        ft.decoded = decoded;
        tr.frames.push_back(ft);

        if (decoded) {
            const int idx = type_index(f.function_code);
            tr.rx_marker_set[idx] = true;
            tr.rx_marker_ticks[idx] = rx.rx_marker_ticks;
        }

        twr::CoreEvent ev;
        ev.kind = twr::CoreEventKind::RxFrame;
        ev.generation = sc_.session_generation;
        ev.event_id = 1000u + static_cast<uint64_t>(rx.token);
        ev.fcs_passed = rx.fcs_passed;
        ev.decode_ok = rx.decode_ok && decoded;
        ev.frame = f;
        ev.rx_time = rx_timestamp(rx.rx_marker_ticks, domain_of(dest));
        ev.rx_first_path = rx.first_path;

        twr::CoreActionBatch b = core.post(ev);
        return handle(core, tr, dest, b, why);
    }

    bool handle(twr::EndpointCore& core, EndpointTrace& tr, uint8_t ep,
                const twr::CoreActionBatch& batch, std::string& why)
    {
        for (size_t i = 0; i < batch.count; ++i) {
            const twr::CoreAction& a = batch.actions[i];
            switch (a.kind) {
            case twr::CoreActionKind::ArmRx: {
                ArmRxTrace ar;
                ar.expect_type = a.expect_type;
                ar.deadline_ticks = a.deadline_ticks;
                tr.arm_rx.push_back(ar);
                break;
            }
            case twr::CoreActionKind::PrepareTx: {
                int64_t air = 0;
                if (!choose_planned(tr, a.tx_intent, air, why))
                    return false;
                PendingPlan p;
                p.intent = a.tx_intent;
                p.plan = make_tx_plan(air, domain_of(ep));
                p.deadline_verdict_feasible = true;
                p.deadline_slack_ticks = 0;
                pending_[a.token.value] = p;

                twr::CoreEvent ev;
                ev.kind = twr::CoreEventKind::TxPlanned;
                ev.generation = sc_.session_generation;
                ev.event_id = 2000u + a.token.value;
                ev.token = a.token;
                ev.tx_intent = a.tx_intent;
                ev.tx_plan = p.plan;
                ev.deadline_verdict_feasible = p.deadline_verdict_feasible;
                ev.deadline_slack_ticks = p.deadline_slack_ticks;
                twr::CoreActionBatch b = core.post(ev);
                if (!handle(core, tr, ep, b, why))
                    return false;
                break;
            }
            case twr::CoreActionKind::SubmitTx: {
                const auto it = pending_.find(a.token.value);
                if (it == pending_.end()) {
                    why = "SubmitTx for an unknown prepared token";
                    return false;
                }
                const PendingPlan& p = it->second;
                const int64_t air = p.plan.calibrated_air_ticks;
                const int idx = type_index(a.tx_intent);
                tr.tx_planned_set[idx] = true;
                tr.tx_planned_ticks[idx] = air;

                FrameTrace ft;
                ft.type = a.tx_intent;
                ft.dir = "tx";
                ft.peer_id = (ep == twr::kEndpointA) ? "B" : "A";
                ft.ticks = air;
                ft.token = a.token.value;
                ft.has_plan = true;
                ft.plan = p.plan;
                ft.deadline_verdict_feasible = p.deadline_verdict_feasible;
                ft.deadline_slack_ticks = p.deadline_slack_ticks;
                std::memcpy(ft.bytes, a.bytes, a.nbytes);
                ft.nbytes = a.nbytes;
                std::string derr;
                ft.decoded = twr::decode(a.bytes, a.nbytes, profile_of(ep), ft.frame, derr);
                tr.frames.push_back(ft);

                twr::FakeTx tx;
                tx.source = ep;
                tx.token = a.token.value;
                std::memcpy(tx.bytes, a.bytes, a.nbytes);
                tx.nbytes = a.nbytes;
                tx.air_ticks = air;
                std::string lerr;
                const twr::FakeLinkError le = link_.submit(tx, lerr);
                if (le != twr::FakeLinkError::Ok) {
                    why = std::string("fake link refused submit: ") +
                          twr::fake_link_error_to_string(le) + " (" + lerr + ")";
                    return false;
                }

                // TxAccepted is NOT completion (R01).  The success outcome is a
                // SEPARATE event, so the terminal result can only close after
                // the local transmit evidence has converged.
                twr::CoreEvent ea;
                ea.kind = twr::CoreEventKind::TxAccepted;
                ea.generation = sc_.session_generation;
                ea.event_id = 3000u + a.token.value;
                ea.token = a.token;
                (void)core.post(ea);

                twr::CoreEvent eo;
                eo.kind = twr::CoreEventKind::TxOutcomeResolved;
                eo.generation = sc_.session_generation;
                eo.event_id = 4000u + a.token.value;
                eo.token = a.token;
                eo.tx_outcome = twr::TxOutcome::Completed;
                eo.adapter_fault = twr::AdapterFault::None;
                twr::CoreActionBatch b = core.post(eo);
                if (!handle(core, tr, ep, b, why))
                    return false;
                break;
            }
            case twr::CoreActionKind::AbortPending:
                break;
            case twr::CoreActionKind::TerminalResult:
                tr.have_terminal = true;
                tr.result = a.result;
                tr.terminal = a.status;
                tr.terminal_detail = a.detail;
                break;
            }
        }
        return true;
    }

    // R08/R09: everything the adapter must remember between PrepareTx and
    // SubmitTx, keyed by the causal token the core created.
    struct PendingPlan {
        twr::TxPlan plan;
        twr::FrameType intent = twr::FrameType::Poll;
        bool deadline_verdict_feasible = true;
        int64_t deadline_slack_ticks = 0;
    };

    const Scenario& sc_;
    twr::FakeTwrLink link_;
    twr::EndpointCore core_a_;
    twr::EndpointCore core_b_;
    EndpointTrace ta_, tb_;
    std::map<uint64_t, PendingPlan> pending_;
};

// ===========================================================================
// Request parsing + validation (the explicit simulation envelope)
// ===========================================================================

bool parse_domain(const Value& v, const std::string& path, twr::ClockDomain& out,
                  std::string& why);
bool make_profile(const Scenario& sc, const twr::ClockDomain& dom, twr::FrameProfile& out,
                  std::string& why);

bool parse_scenario(const Value& root, Scenario& sc, std::string& why)
{
    if (!as_object(root, "<root>", why))
        return false;

    std::string schema;
    if (!get_string(root, "schema", "<root>", schema, why))
        return false;
    if (schema != "twr-m1b-demo/1") {
        why = "unsupported request schema \"" + schema + "\" (expected twr-m1b-demo/1)";
        return false;
    }

    // --- simulation gate: explicit, and never bypassed --------------------
    const Value* sim = nullptr;
    if (!get_object(root, "simulation", "<root>", sim, why))
        return false;
    if (!get_string(*sim, "execution", "simulation", sc.execution, why))
        return false;
    if (!get_string(*sim, "mode", "simulation", sc.mode, why))
        return false;
    if (sc.execution != "offline_simulation") {
        why = "simulation.execution must be \"offline_simulation\" (got \"" + sc.execution +
              "\"); a hardware profile cannot be licensed by this demo";
        return false;
    }
    if (sc.mode != "protocol_estimate") {
        why = "simulation.mode must be \"protocol_estimate\" (got \"" + sc.mode + "\")";
        return false;
    }
    if (!get_u64(*sim, "seed", "simulation", sc.seed, why))
        return false;
    if (!get_string(*sim, "scenario_id", "simulation", sc.scenario_id, why))
        return false;

    // --- protocol ---------------------------------------------------------
    std::string proto;
    if (!get_string(root, "protocol", "<root>", proto, why))
        return false;
    if (!twr::protocol_from_string(proto, sc.protocol)) {
        why = "protocol must be \"ss\" or \"ds\", got \"" + proto + "\"";
        return false;
    }

    // --- session ----------------------------------------------------------
    const Value* sess = nullptr;
    if (!get_object(root, "session", "<root>", sess, why))
        return false;
    if (!get_u16(*sess, "session_id", "session", sc.session_id, why))
        return false;
    if (!get_u64(*sess, "session_generation", "session", sc.session_generation, why))
        return false;
    if (!get_u16(*sess, "sequence_modulus", "session", sc.sequence_modulus, why))
        return false;
    if (!get_u16(*sess, "initial_sequence", "session", sc.initial_sequence, why))
        return false;
    if (!get_u16(*sess, "pan_id", "session", sc.pan_id, why))
        return false;
    if (sc.session_generation == 0) {
        why = "session.session_generation must be non-zero";
        return false;
    }

    // --- wire -------------------------------------------------------------
    const Value* wire = nullptr;
    if (!get_object(root, "wire", "<root>", wire, why))
        return false;
    int64_t bits = 0;
    if (!get_i64(*wire, "timestamp_bits", "wire", bits, why))
        return false;
    if (bits < 8 || bits > 64 || (bits % 8) != 0) {
        why = "wire.timestamp_bits must be a multiple of 8 in [8,64]";
        return false;
    }
    sc.wire_timestamp_bits = static_cast<uint8_t>(bits);
    if (!get_double(*wire, "timestamp_unit_hz", "wire", sc.wire_timestamp_unit_hz, why))
        return false;
    if (!(sc.wire_timestamp_unit_hz > 0.0) || !std::isfinite(sc.wire_timestamp_unit_hz)) {
        why = "wire.timestamp_unit_hz must be finite and > 0";
        return false;
    }
    if (!get_u64(*wire, "max_interval_ticks", "wire", sc.max_interval_ticks, why))
        return false;
    if (sc.max_interval_ticks == 0) {
        why = "wire.max_interval_ticks must be positive";
        return false;
    }

    // --- endpoints --------------------------------------------------------
    const Value* eps = nullptr;
    if (!get_object(root, "endpoints", "<root>", eps, why))
        return false;
    const Value* ea = nullptr;
    const Value* eb = nullptr;
    if (!get_object(*eps, "a", "endpoints", ea, why))
        return false;
    if (!get_object(*eps, "b", "endpoints", eb, why))
        return false;
    if (!get_u16(*ea, "local_address", "endpoints.a", sc.a_local, why))
        return false;
    if (!get_u16(*ea, "peer_address", "endpoints.a", sc.a_peer, why))
        return false;
    if (!get_u16(*eb, "local_address", "endpoints.b", sc.b_local, why))
        return false;
    if (!get_u16(*eb, "peer_address", "endpoints.b", sc.b_peer, why))
        return false;
    if (sc.a_local != sc.b_peer || sc.b_local != sc.a_peer) {
        why = "endpoint addressing is inconsistent: A.local must equal B.peer and "
              "B.local must equal A.peer";
        return false;
    }
    if (sc.a_local == sc.b_local) {
        why = "the two endpoints must have distinct local addresses";
        return false;
    }

    // --- domains ----------------------------------------------------------
    const Value* doms = nullptr;
    if (!get_object(root, "domains", "<root>", doms, why))
        return false;
    const Value* da = nullptr;
    const Value* db = nullptr;
    if (!get_object(*doms, "a", "domains", da, why))
        return false;
    if (!get_object(*doms, "b", "domains", db, why))
        return false;
    if (!parse_domain(*da, "domains.a", sc.dom_a, why))
        return false;
    if (!parse_domain(*db, "domains.b", sc.dom_b, why))
        return false;

    // --- clock ratio ------------------------------------------------------
    const Value* cr = nullptr;
    if (!get_object(root, "clock_ratio", "<root>", cr, why))
        return false;
    std::string kind;
    if (!get_string(*cr, "kind", "clock_ratio", kind, why))
        return false;
    bool ratio_ok = false;
    if (kind == "unity_same_clock") {
        if (!(twr::clock_domain_same_identity(sc.dom_a, sc.dom_b) &&
              twr::clock_domain_same_epoch(sc.dom_a, sc.dom_b))) {
            why = "clock_ratio.kind=unity_same_clock requires domains.a and domains.b to "
                  "be the same counter (name, rate, width and epoch)";
            return false;
        }
        if (!(sc.wire_timestamp_unit_hz == sc.dom_a.tick_rate_hz)) {
            why = "for unity_same_clock, wire.timestamp_unit_hz must equal the shared "
                  "domain tick rate";
            return false;
        }
        ratio_ok = twr::ClockRatio::unity_same_clock(sc.dom_a, sc.ratio);
    } else if (kind == "nominal_rate_ratio") {
        int64_t fa = 0, fb = 0;
        if (!get_i64(*cr, "fA_hz", "clock_ratio", fa, why))
            return false;
        if (!get_i64(*cr, "fB_hz", "clock_ratio", fb, why))
            return false;
        ratio_ok = twr::ClockRatio::from_nominal_rates(sc.dom_a, sc.dom_b, fa, fb, sc.ratio);
    } else {
        why = "clock_ratio.kind must be \"unity_same_clock\" or \"nominal_rate_ratio\", got \"" +
              kind + "\"";
        return false;
    }
    if (!ratio_ok) {
        why = "the clock ratio declaration is not usable by the exact-kernel convention";
        return false;
    }

    // --- calibration ------------------------------------------------------
    const Value* cal = nullptr;
    if (!get_object(root, "calibration", "<root>", cal, why))
        return false;
    if (!get_string(*cal, "id", "calibration", sc.cal.id, why))
        return false;
    if (!get_u64(*cal, "calibrated_epoch", "calibration", sc.cal.calibrated_epoch, why))
        return false;
    if (!get_i64(*cal, "valid_from_ticks", "calibration", sc.cal.valid_from_ticks, why))
        return false;
    if (!get_i64(*cal, "valid_until_ticks", "calibration", sc.cal.valid_until_ticks, why))
        return false;
    sc.cal.applications.clear();
    sc.cal.applications.push_back(
        twr::CalibrationApplication{ sc.cal.id, twr::CalibrationResult::Applied });
    if (sc.cal.id.empty()) {
        why = "calibration.id must not be empty";
        return false;
    }
    if (!sc.cal.is_well_formed()) {
        why = "calibration window is degenerate: need valid_from_ticks < valid_until_ticks";
        return false;
    }
    if (sc.cal.calibrated_epoch != sc.dom_a.epoch_id ||
        sc.cal.calibrated_epoch != sc.dom_b.epoch_id) {
        why = "calibration.calibrated_epoch must equal both domains' epoch_id "
              "(a calibration from another epoch cannot be reused)";
        return false;
    }

    // --- link (transport ground truth; NEVER an endpoint input) -----------
    const Value* link = nullptr;
    if (!get_object(root, "link", "<root>", link, why))
        return false;
    if (!get_double(*link, "distance_m", "link", sc.distance_m, why))
        return false;
    if (!std::isfinite(sc.distance_m) || sc.distance_m < 0.0) {
        why = "link.distance_m must be finite and >= 0";
        return false;
    }
    if (!get_i64(*link, "tx_air_latency_a_ticks", "link", sc.tx_latency_a, why))
        return false;
    if (!get_i64(*link, "tx_air_latency_b_ticks", "link", sc.tx_latency_b, why))
        return false;
    if (sc.tx_latency_a < 0 || sc.tx_latency_b < 0) {
        why = "link.tx_air_latency_*_ticks must be >= 0";
        return false;
    }

    // --- timeline ---------------------------------------------------------
    const Value* tl = nullptr;
    if (!get_object(root, "timeline", "<root>", tl, why))
        return false;
    if (!get_i64(*tl, "poll_air_ticks", "timeline", sc.poll_air_ticks, why))
        return false;
    if (!get_i64(*tl, "response_reply_ticks", "timeline", sc.response_reply_ticks, why))
        return false;
    if (!get_i64(*tl, "final_reply_ticks", "timeline", sc.final_reply_ticks, why))
        return false;
    if (sc.poll_air_ticks < 0 || sc.response_reply_ticks < 0 || sc.final_reply_ticks < 0) {
        why = "timeline ticks must be >= 0";
        return false;
    }

    // --- frame profiles, one per endpoint (unit == local rate) ------------
    if (!make_profile(sc, sc.dom_a, sc.profile_a, why))
        return false;
    if (!make_profile(sc, sc.dom_b, sc.profile_b, why))
        return false;
    if (twr::clock_domain_same_identity(sc.dom_a, sc.dom_b) &&
        sc.wire_timestamp_bits == 0) {
        why = "a zero wire timestamp width is not a usable frame field";
        return false;
    }

    // --- sequence modulus -------------------------------------------------
    if (!twr::sequence_modulus_is_supported(sc.sequence_modulus)) {
        why = "session.sequence_modulus must be one of 4/16/64/256";
        return false;
    }
    if (sc.initial_sequence >= sc.sequence_modulus) {
        why = "session.initial_sequence must be < sequence_modulus";
        return false;
    }

    // --- optional existing TwrConfig, validated by the existing validator --
    const Value* tcfg = root.find("twr_config");
    if (tcfg != nullptr) {
        sc.has_twr_config = true;
        sc.twr_config = *tcfg;
    }
    return true;
}

bool parse_domain(const Value& v, const std::string& path, twr::ClockDomain& out,
                  std::string& why)
{
    std::string name;
    double rate = 0.0;
    uint64_t epoch = 0;
    int64_t bits = 0;
    if (!get_string(v, "name", path, name, why))
        return false;
    if (!get_double(v, "tick_rate_hz", path, rate, why))
        return false;
    if (!get_u64(v, "epoch_id", path, epoch, why))
        return false;
    if (!get_i64(v, "timestamp_bits", path, bits, why))
        return false;
    if (bits < 0 || bits > 63) {
        why = path + ".timestamp_bits must be in [0,63] (0 == no wrap)";
        return false;
    }
    if (!twr::ClockDomain::make(name, rate, epoch, static_cast<uint32_t>(bits), out)) {
        why = path + " is not a valid ClockDomain (name non-empty, rate finite and > 0, "
                     "timestamp_bits in [0,63])";
        return false;
    }
    return true;
}

bool make_profile(const Scenario& sc, const twr::ClockDomain& dom, twr::FrameProfile& out,
                  std::string& why)
{
    out = twr::FrameProfile();
    out.version = twr::kFrameVersion;
    out.timestamp_bits = sc.wire_timestamp_bits;
    out.timestamp_unit_hz = dom.tick_rate_hz; // local -> wire unit is EXACT
    out.max_psdu_bytes = twr::kPhyMaxPsduBytes;
    out.fcs_appended_by_modulation_layer = true;
    std::string ferr;
    if (!twr::frame_profile_validate(out, ferr, nullptr)) {
        why = "frame profile is not usable: " + ferr;
        return false;
    }
    if (!(out.timestamp_unit_hz == dom.tick_rate_hz)) {
        why = "frame profile timestamp unit does not equal the local domain rate";
        return false;
    }
    return true;
}

// A host-monotonic timing budget, as the envelope's documented default (the
// offline core has a single deadline channel and no host clock; the mapping
// row says so).  This is NOT a substitute for an operator's measured value.
twr::TimedField envelope_host_budget(int64_t ns)
{
    twr::TimedField f;
    f.value = twr::Duration::from_nanos(ns);
    f.domain = twr::TimeDomain::MonotonicHost;
    f.reference = twr::TimeReferenceEvent::HostMonotonic;
    f.marker.reset();
    f.required_quantisation_hz = 1.0e9;
    f.max_quantisation_error_ns = 1;
    return f;
}

std::string budget_ns_text(const twr::TimedField& f)
{
    return twr::twr_duration_to_text(f.value);
}

// R09: every requested section the OFFLINE core does not consume, with its
// intended use and the limit of that use here.  This is deliberately a
// fixed list, not a silent omission.
void add_unconsumed_rows(Scenario& sc, bool config_present)
{
    const std::string basis = config_present
                                  ? "validated in the requested twr_config but not executed by M1-B"
                                  : "not stated (the run used only the envelope)";
    auto add = [&](const std::string& field, const std::string& use, const std::string& limit) {
        sc.unconsumed.push_back(UnconsumedRow{ field, use, limit });
    };
    add("phy", "channel / preamble codes / SYNC length / PRF class / payload+PHR rate; the "
               "radio profile M2 will program. " + basis,
        "the offline core has no PHY: it consumes only the frame codec's profile. The measured "
        "whitelist is still enforced by validate(), so an unsupported PHY is refused.");
    add("frame.layout (geometry, sfd, phr, ranging_bit, mac_psdu, fcs, sts)",
        "the MAC layout claim and the length/FCS rules. " + basis,
        "the codec (frame_profile=frame_v1) is the geometry authority and builds the bytes; the "
        "claim only has to AGREE with it. sts is phase-2 only.");
    add("tx", "port, gain_db, iq_amplitude, calibrated_tx_power_dbm, power policy, pulse shaping "
              "and a vendor power word. " + basis,
        "no RF chain exists offline; the three different power quantities are never merged.");
    add("rx", "port, gain, AGC, bandwidth, detection/correlation/first-path thresholds and "
              "vendor PAC. " + basis,
        "offline the first-path verdict comes from the fake link, not from these thresholds.");
    add("radio", "device args, channels, native rate, clock/time source, peers and the readback "
                 "record. " + basis,
        "no device is opened offline; require_readback is a startup-only rule.");
    add("timing.poll_start / response_to_final / final_to_report / post_tx_rx_enable / "
        "min_tx_lead_time",
        "the per-message timing plan (when the Poll goes out, the DS final-to-final delay, the "
        "post-TX RX enable and the measured UHD lead time). " + basis,
        "the offline core is receive-driven and has no device lead time; only the reply budget "
        "(poll_to_response) and the two deadlines are mapped.");
    add("calibration (link/antenna/cable delays, native ticks, first_path_algorithm, "
        "cfo/sfo compensation, record, applied_count)",
        "the versioned calibration constants and compensation policy. " + basis,
        "the demo's local_calibration comes from the versioned envelope block, which must name "
        "the same calibration id; applied_count must be 0 (applicable exactly once).");
    add("session.measurement_count / measurement_interval / max_attempts_per_exchange / "
        "retry_backoff",
        "the multi-measurement and retry policy. " + basis,
        "M1-B runs exactly one exchange with no retry; validate() already forces a consistent "
        "attempt/backoff pair.");
    add("session.require_pan_match / require_address_match",
        "the peer-matching policy. " + basis,
        "the M1-B core ALWAYS requires a PAN and address match; there is no offline switch that "
        "could disable it.");
    add("diagnostics (all except result_queue_capacity)",
        "CIR/short-IQ/raw-frame capture bounds, output path, stats cadence and the "
        "io_on_realtime_thread guard. " + basis,
        "the offline core emits a bounded action batch and no diagnostic I/O. "
        "event_queue_capacity is listed in the mapping above.");
}

// R09: resolve the execution authority.  When a `twr_config` is present it is
// parsed with the EXISTING `from_json_string`, validated with the EXISTING
// `validate()` and frozen with `effective_config()`; the core snapshot is then
// built FROM those validated values, and a field the envelope also states is
// REFUSED on disagreement rather than silently overridden.  Everything the
// offline core does not consume is listed with its use and limit.
bool resolve_execution_config(Scenario& sc, std::string& why)
{
    if (!sc.has_twr_config) {
        // Envelope-only path: document the core defaults explicitly.
        sc.exchange_budget = envelope_host_budget(1000000000); // 1 s
        sc.evidence_budget = envelope_host_budget(1000000000); // 1 s
        sc.reply_budget = twr::TimedField();
        const twr::CoreConfig ca = make_core_config(sc, twr::Role::Initiator);
        const twr::CoreConfig cb = make_core_config(sc, twr::Role::Responder);
        auto add = [&](const std::string& field, const std::string& v,
                       const std::string& core_v, const char* source, bool consumed,
                       const char* note) {
            sc.mappings.push_back(MappingRow{ field, v, v, core_v, source, consumed,
                                              note ? note : "" });
        };
        add("session.protocol", twr::protocol_to_string(sc.protocol),
            twr::protocol_to_string(ca.protocol), "envelope", true, "");
        add("session.local_address", fmt_u16(sc.a_local),
            "A=" + fmt_u16(ca.local_address) + " B=" + fmt_u16(cb.local_address),
            "envelope", true, "");
        add("session.peer_address", fmt_u16(sc.a_peer),
            "A=" + fmt_u16(ca.peer_address) + " B=" + fmt_u16(cb.peer_address),
            "envelope", true, "");
        add("session.pan_id", fmt_u16(sc.pan_id), fmt_u16(ca.pan_id), "envelope", true, "");
        add("session.session_id", fmt_u16(sc.session_id), fmt_u16(ca.session_id),
            "envelope", true, "");
        add("session.session_generation",
            twr::twr_int_to_text(static_cast<int64_t>(sc.session_generation)),
            "A=" + twr::twr_int_to_text(static_cast<int64_t>(ca.session_generation)),
            "envelope", true, "");
        add("session.sequence_modulus", fmt_u16(sc.sequence_modulus),
            fmt_u16(ca.sequence_modulus), "envelope", true, "");
        add("session.sequence", fmt_u16(sc.initial_sequence),
            fmt_u16(ca.initial_sequence), "envelope", true, "");
        add("session.exchange_id", fmt_u32(sc.exchange_id),
            twr::twr_int_to_text(static_cast<int64_t>(sc.exchange_id != 0 ? sc.exchange_id : 1)),
            "core_default", true,
            "0 is not a legal runtime identity; the Begin uses the documented default 1");
        add("session.max_in_flight_exchanges", fmt_u32(sc.max_in_flight_exchanges),
            fmt_u32(ca.max_in_flight), "core_default", true,
            "phase 1 allows exactly one in-flight exchange");
        add("diagnostics.result_queue_capacity", fmt_u32(sc.result_queue_capacity),
            fmt_u32(ca.result_queue_capacity), "core_default", true,
            "bounded completed-result storage (M1-B B13); a full queue refuses a new Begin");
        add("diagnostics.event_queue_capacity", fmt_u32(sc.event_queue_capacity),
            "not consumed", "core_default", false,
            "the offline two-endpoint driver has no adapter input queue; the core's bounded "
            "action batch and the fake link's rx queue are the actual bounds");
        add("timeouts.exchange_timeout", budget_ns_text(sc.exchange_budget),
            "A=" + twr::twr_int_to_text(ca.exchange_timeout_ticks) + "ticks B=" +
                twr::twr_int_to_text(cb.exchange_timeout_ticks) + "ticks",
            "core_default", true,
            "whole-exchange deadline: a host-monotonic ns budget projected onto the local device "
            "grid (the offline core has ONE deadline channel and no host clock)");
        add("timeouts.rx_timeout", budget_ns_text(sc.evidence_budget),
            "A=" + twr::twr_int_to_text(ca.evidence_wait_ticks) + "ticks B=" +
                twr::twr_int_to_text(cb.evidence_wait_ticks) + "ticks",
            "core_default", true,
            "used as the evidence wait (how long the core keeps waiting for the missing local "
            "step); M1-B has no separate evidence-wait config field");
        add("timing.poll_to_response (reply budget)", budget_ns_text(sc.reply_budget),
            "A=" + twr::twr_int_to_text(ca.reply_deadline_ticks) + "ticks",
            "core_default", true, "0 == disabled (the envelope states no reply budget)");
        add_unconsumed_rows(sc, /*config_present=*/false);
        sc.config_resolved = true;
        return true;
    }

    std::string text;
    std::string derr;
    if (!twr::json::dump(sc.twr_config, text, derr, false)) {
        why = "twr_config is not serialisable: " + derr;
        return false;
    }
    twr::TwrConfig requested;
    const twr::ValidationReport parsed = twr::from_json_string(text, requested);
    if (!parsed.ok()) {
        why = "twr_config failed the existing parser/validator: " + parsed.to_string();
        return false;
    }
    const twr::ValidationReport report = twr::validate(requested);
    if (!report.ok()) {
        why = "twr_config failed validate(): " + report.to_string();
        return false;
    }
    const twr::EffectiveConfig eff = twr::effective_config(requested);
    if (!eff.ok) {
        why = "twr_config produced no effective snapshot: " + eff.validation.to_string();
        return false;
    }
    const twr::TwrConfig& e = eff.effective;

    // ---- conflicts: a field both state must agree, or the run is refused ---
    auto conflict = [&](const std::string& field) {
        why = "twr_config conflicts with the envelope on " + field +
              " (a conflict is refused, never silently overridden)";
        return false;
    };
    if (e.session.role != twr::Role::Initiator) {
        why = "twr_config.session.role must be \"initiator\": this demo's endpoint A is the "
              "initiator and role reversal is a later milestone";
        return false;
    }
    if (e.session.protocol != sc.protocol)
        return conflict("session.protocol");
    if (static_cast<uint16_t>(e.session.session_id) != sc.session_id)
        return conflict("session.session_id");
    if (e.session.pan_id != sc.pan_id)
        return conflict("session.pan_id");
    if (e.session.sequence_modulus != sc.sequence_modulus)
        return conflict("session.sequence_modulus");
    if (static_cast<uint16_t>(e.session.sequence) != sc.initial_sequence)
        return conflict("session.sequence");
    if (e.session.local_address != sc.a_local || e.session.peer_address != sc.a_peer)
        return conflict("session.local_address/session.peer_address");
    if (!e.calibration.calibration_id.empty() &&
        e.calibration.calibration_id != sc.cal.id)
        return conflict("calibration.calibration_id");

    // ---- the config is the authority: adopt its validated values ----------
    sc.protocol = e.session.protocol;
    sc.session_id = static_cast<uint16_t>(e.session.session_id);
    sc.pan_id = e.session.pan_id;
    sc.sequence_modulus = e.session.sequence_modulus;
    sc.initial_sequence = static_cast<uint16_t>(e.session.sequence);
    sc.a_local = e.session.local_address;
    sc.a_peer = e.session.peer_address;
    sc.b_local = e.session.peer_address;
    sc.b_peer = e.session.local_address;
    sc.max_in_flight_exchanges = e.session.max_in_flight_exchanges;
    sc.exchange_id = e.session.exchange_id;
    sc.result_queue_capacity = e.diagnostics.result_queue_capacity;
    sc.event_queue_capacity = e.diagnostics.event_queue_capacity;
    sc.reply_budget = e.timing.poll_to_response;
    sc.exchange_budget = e.timeouts.exchange_timeout;
    sc.evidence_budget = e.timeouts.rx_timeout;

    std::string rj;
    if (!twr::to_json_string(requested, sc.requested_config_json, rj)) {
        why = "cannot serialise the requested twr_config: " + rj;
        return false;
    }
    std::string ej;
    if (!twr::to_json_string(eff, sc.effective_config_json, ej)) {
        why = "cannot serialise the effective config snapshot: " + ej;
        return false;
    }
    sc.requested_config_fnv = twr::config_hash(requested);
    sc.effective_config_fnv = twr::config_hash(eff.effective);

    // ---- the explicit requested -> effective -> core mapping --------------
    const twr::CoreConfig ca = make_core_config(sc, twr::Role::Initiator);
    const twr::CoreConfig cb = make_core_config(sc, twr::Role::Responder);
    auto add = [&](const std::string& field, const std::string& requested_v,
                   const std::string& effective_v, const std::string& core_v,
                   const char* note) {
        sc.mappings.push_back(MappingRow{ field, requested_v, effective_v, core_v,
                                          "twr_config", true, note ? note : "" });
    };
    add("session.protocol", twr::protocol_to_string(requested.session.protocol),
        twr::protocol_to_string(e.session.protocol),
        twr::protocol_to_string(ca.protocol), "");
    add("session.local_address", fmt_u16(requested.session.local_address),
        fmt_u16(e.session.local_address),
        "A=" + fmt_u16(ca.local_address) + " B=" + fmt_u16(cb.local_address), "");
    add("session.peer_address", fmt_u16(requested.session.peer_address),
        fmt_u16(e.session.peer_address),
        "A=" + fmt_u16(ca.peer_address) + " B=" + fmt_u16(cb.peer_address), "");
    add("session.pan_id", fmt_u16(requested.session.pan_id), fmt_u16(e.session.pan_id),
        fmt_u16(ca.pan_id), "");
    add("session.session_id", fmt_u16(requested.session.session_id),
        fmt_u16(e.session.session_id), fmt_u16(ca.session_id), "");
    add("session.session_generation",
        twr::twr_int_to_text(static_cast<int64_t>(sc.session_generation)),
        twr::twr_int_to_text(static_cast<int64_t>(sc.session_generation)),
        twr::twr_int_to_text(static_cast<int64_t>(ca.session_generation)),
        "the config has no generation field; the envelope's versioned value is used");
    add("session.sequence_modulus", fmt_u16(requested.session.sequence_modulus),
        fmt_u16(e.session.sequence_modulus), fmt_u16(ca.sequence_modulus), "");
    add("session.sequence", fmt_u16(requested.session.sequence),
        fmt_u16(e.session.sequence), fmt_u16(ca.initial_sequence),
        "the wire sequence is the core's initial_sequence");
    add("session.exchange_id", fmt_u32(requested.session.exchange_id),
        fmt_u32(e.session.exchange_id),
        twr::twr_int_to_text(static_cast<int64_t>(sc.exchange_id != 0 ? sc.exchange_id : 1)),
        sc.exchange_id != 0
            ? "the Begin exchange identity"
            : "0 is the config default and is not a legal runtime identity; Begin uses 1");
    add("session.max_in_flight_exchanges",
        fmt_u32(requested.session.max_in_flight_exchanges),
        fmt_u32(e.session.max_in_flight_exchanges), fmt_u32(ca.max_in_flight), "");
    add("diagnostics.result_queue_capacity",
        fmt_u32(requested.diagnostics.result_queue_capacity),
        fmt_u32(e.diagnostics.result_queue_capacity), fmt_u32(ca.result_queue_capacity),
        "bounded completed-result storage; a full queue refuses a new Begin");
    sc.mappings.push_back(MappingRow{
        "diagnostics.event_queue_capacity",
        fmt_u32(requested.diagnostics.event_queue_capacity),
        fmt_u32(e.diagnostics.event_queue_capacity), "not consumed", "twr_config", false,
        "the offline two-endpoint driver has no adapter input queue; the core's bounded action "
        "batch and the fake link's rx queue are the actual bounds" });
    add("timing.poll_to_response (reply budget)", budget_ns_text(requested.timing.poll_to_response),
        budget_ns_text(e.timing.poll_to_response),
        "A=" + twr::twr_int_to_text(ca.reply_deadline_ticks) + "ticks B=" +
            twr::twr_int_to_text(cb.reply_deadline_ticks) + "ticks",
        "projected onto each endpoint's local grid into CoreConfig.reply_deadline_ticks "
        "(validated); the M1-B receive-driven FSM does not currently act on it, so it is a "
        "stated per-message budget rather than a driver of the trace");
    add("timeouts.exchange_timeout", budget_ns_text(requested.timeouts.exchange_timeout),
        budget_ns_text(e.timeouts.exchange_timeout),
        "A=" + twr::twr_int_to_text(ca.exchange_timeout_ticks) + "ticks B=" +
            twr::twr_int_to_text(cb.exchange_timeout_ticks) + "ticks",
        "whole-exchange deadline: a host-monotonic ns budget projected onto the local device "
        "grid (the offline core has ONE deadline channel and no host clock)");
    add("timeouts.rx_timeout", budget_ns_text(requested.timeouts.rx_timeout),
        budget_ns_text(e.timeouts.rx_timeout),
        "A=" + twr::twr_int_to_text(ca.evidence_wait_ticks) + "ticks B=" +
            twr::twr_int_to_text(cb.evidence_wait_ticks) + "ticks",
        "used as the evidence wait (how long the core keeps waiting for the missing local "
        "step); M1-B has no separate evidence-wait field in the config");
    add("frame.frame_profile",
        twr::frame_profile_id_to_string(requested.frame.frame_profile),
        twr::frame_profile_id_to_string(e.frame.frame_profile),
        "frame_profile{bits=" + std::to_string(static_cast<unsigned>(ca.frame_profile.timestamp_bits)) +
            ",unit=" + twr::twr_double_to_text(ca.frame_profile.timestamp_unit_hz) + "}",
        "the codec profile selects the on-wire layout; the demo builds the equivalent "
        "FrameProfile from the envelope's wire geometry");

    // The core requires a finite termination bound.  Refuse here with a clear
    // message rather than letting configure() fail with a generic one.
    if (ca.exchange_timeout_ticks == 0 && ca.evidence_wait_ticks == 0 &&
        ca.reply_deadline_ticks == 0) {
        why = "the requested config leaves every timing budget disabled (exchange_timeout, "
              "rx_timeout and poll_to_response all project to 0): an accepted exchange would "
              "have no finite termination bound";
        return false;
    }

    add_unconsumed_rows(sc, /*config_present=*/true);
    sc.config_resolved = true;
    return true;
}

// ===========================================================================
// Output construction
// ===========================================================================

Value mkobj() { return Value::make_object(); }
Value mkint(int64_t v) { return Value::make_int(v); }
Value mkstr(const std::string& v) { return Value::make_string(v); }
Value mkbool(bool v) { return Value::make_bool(v); }
Value mknum(double v) { return Value::make_double(v); }

Value frame_fields_value(const twr::Frame& f)
{
    Value o = mkobj();
    o.set("version", mkint(f.version));
    o.set("function_code", mkint(static_cast<int64_t>(f.function_code)));
    o.set("function_name", mkstr(twr::frame_type_to_string(f.function_code)));
    o.set("session_id", mkint(f.session_id));
    o.set("seq", mkint(f.seq));
    o.set("pan_id", mkint(f.pan_id));
    o.set("src_addr", mkint(f.src_addr));
    o.set("dst_addr", mkint(f.dst_addr));
    o.set("flags", mkint(f.flags));
    Value ts = mkobj();
    const twr::TimestampField fields[5] = {
        twr::TimestampField::T1A, twr::TimestampField::T2B,
        twr::TimestampField::T3B, twr::TimestampField::T4A,
        twr::TimestampField::T5A
    };
    for (twr::TimestampField fld : fields) {
        uint64_t v = 0;
        if (f.get(fld, v))
            ts.set(twr::timestamp_field_name(fld), mkint(static_cast<int64_t>(v)));
    }
    o.set("timestamps", ts);
    return o;
}

// R08: the numeric plan, emitted so an independent verifier can recompute
//     calibrated_air_ticks == quantised_instant_ticks + marker_offset_ticks
// and check every field against the frame's t3B/t5A and the local evidence.
Value plan_value(const twr::TxPlan& p)
{
    Value o = mkobj();
    o.set("valid", mkbool(p.valid));
    o.set("domain", mkstr(p.domain.to_string()));
    o.set("domain_name", mkstr(p.domain.name));
    o.set("tick_rate_hz", mknum(p.domain.tick_rate_hz));
    o.set("epoch_id", mkint(static_cast<int64_t>(p.domain.epoch_id)));
    o.set("timestamp_bits", mkint(static_cast<int64_t>(p.domain.timestamp_bits)));
    o.set("source", mkstr(twr::timestamp_source_to_string(p.source)));
    o.set("applied_corrections", mkint(static_cast<int64_t>(p.applied_corrections)));
    o.set("corrections_text", mkstr(twr::timestamp_corrections_to_string(p.applied_corrections)));
    o.set("calibration_id", mkstr(p.calibration_id));
    o.set("command_time_ticks", mkstr(twr::twr_int_to_text(p.command_time_ticks)));
    o.set("quantised_instant_ticks", mkstr(twr::twr_int_to_text(p.quantised_instant_ticks)));
    o.set("marker_offset_ticks", mkstr(twr::twr_int_to_text(p.marker_offset_ticks)));
    o.set("calibrated_air_ticks", mkstr(twr::twr_int_to_text(p.calibrated_air_ticks)));
    o.set("identity_holds",
          mkbool(p.calibrated_air_ticks == p.quantised_instant_ticks + p.marker_offset_ticks));
    std::string why;
    o.set("internally_consistent", mkbool(p.internally_consistent(why)));
    o.set("consistency_note", mkstr(why));
    return o;
}

Value frames_value(const EndpointTrace& tr)
{
    Value arr = Value::make_array();
    for (const auto& ft : tr.frames) {
        Value o = mkobj();
        o.set("type", mkstr(twr::frame_type_to_string(ft.type)));
        o.set("dir", mkstr(ft.dir));
        o.set("peer", mkstr(ft.peer_id));
        o.set("ticks", mkint(ft.ticks));
        o.set("token", mkint(static_cast<int64_t>(ft.token)));
        o.set("nbytes", mkint(static_cast<int64_t>(ft.nbytes)));
        o.set("hex", mkstr(hex_bytes(ft.bytes, ft.nbytes)));
        o.set("decoded", mkbool(ft.decoded));
        Value fields = ft.decoded ? frame_fields_value(ft.frame) : mkobj();
        o.set("fields", fields);
        if (ft.has_plan) {
            o.set("plan", plan_value(ft.plan));
            o.set("deadline_verdict_feasible", mkbool(ft.deadline_verdict_feasible));
            o.set("deadline_slack_ticks", mkstr(twr::twr_int_to_text(ft.deadline_slack_ticks)));
        }
        arr.push(o);
    }
    return arr;
}

Value arm_rx_value(const EndpointTrace& tr)
{
    Value arr = Value::make_array();
    for (const auto& ar : tr.arm_rx) {
        Value o = mkobj();
        o.set("expect", mkstr(twr::frame_type_to_string(ar.expect_type)));
        o.set("deadline_ticks", mkstr(twr::twr_int_to_text(ar.deadline_ticks)));
        arr.push(o);
    }
    return arr;
}

Value local_evidence_value(const EndpointTrace& tr, bool same_clock, bool is_initiator)
{
    Value o = mkobj();
    Value txp = mkobj();
    Value rxm = mkobj();
    const twr::FrameType types[3] = {
        twr::FrameType::Poll, twr::FrameType::Response, twr::FrameType::Final
    };
    for (twr::FrameType t : types) {
        const int i = type_index(t);
        if (tr.tx_planned_set[i])
            txp.set(twr::frame_type_to_string(t), mkint(tr.tx_planned_ticks[i]));
        if (tr.rx_marker_set[i])
            rxm.set(twr::frame_type_to_string(t), mkint(tr.rx_marker_ticks[i]));
    }
    o.set("tx_planned_ticks", txp);
    o.set("rx_marker_ticks", rxm);
    (void)same_clock;
    (void)is_initiator;
    return o;
}

Value endpoint_value(const EndpointTrace& tr, const twr::CoreConfig& cfg, bool same_clock)
{
    Value o = mkobj();
    o.set("endpoint_id", mkstr(tr.endpoint_id));
    o.set("role", mkstr(twr::role_to_string(tr.role)));
    o.set("local_address", mkint(tr.local_address));
    o.set("peer_address", mkint(tr.peer_address));
    o.set("protocol", mkstr(twr::protocol_to_string(cfg.protocol)));

    if (tr.have_terminal) {
        const twr::ProtocolTofEstimate& e = tr.result;
        o.set("completion",
              mkstr(twr::protocol_completion_status_to_string(e.completion)));
        o.set("estimate_available", mkbool(e.estimate_available));
        o.set("local_evidence_complete", mkbool(e.local_evidence_complete));
        o.set("peer_evidence", mkstr("wire_claim"));
        o.set("measurement_valid", mkbool(false));
        o.set("execution_mode", mkstr("simulation"));
        o.set("protocol_completed",
              mkbool(e.completion == twr::ProtocolCompletionStatus::Complete));
        o.set("computed_at", mkstr(twr::computed_at_to_string(e.computed_at)));
        o.set("terminal_completion",
              mkstr(twr::protocol_completion_status_to_string(tr.terminal.completion)));
        {
            twr::ExchangeStatus reason = twr::ExchangeStatus::InternalError;
            if (tr.terminal.failure_reason(reason))
                o.set("terminal_failure_reason",
                      mkstr(twr::exchange_status_to_string(reason)));
        }
        {
            twr::ExchangeStatus est_reason = twr::ExchangeStatus::InternalError;
            if (e.failure_reason(est_reason))
                o.set("failure_reason",
                      mkstr(twr::exchange_status_to_string(est_reason)));
        }
        o.set("detail", mkstr(e.detail));

        Value tof = mkobj();
        if (e.estimate_available) {
            tof.set("available", mkbool(true));
            tof.set("num", mkstr(twr::twr_int_to_text(e.tof.num)));
            tof.set("den", mkstr(twr::twr_int_to_text(e.tof.den)));
            tof.set("domain", mkstr(tr.endpoint_id == "A" ? "a" : "b"));
            tof.set("tick_rate_hz", mknum(cfg.local_domain.tick_rate_hz));
            tof.set("epoch_id", mkint(static_cast<int64_t>(cfg.local_domain.epoch_id)));
            tof.set("math_status", mkstr(twr::tof_status_to_string(e.math_status)));
        } else {
            tof.set("available", mkbool(false));
            tof.set("num", Value::make_null());
            tof.set("den", Value::make_null());
            tof.set("domain", mkstr(tr.endpoint_id == "A" ? "a" : "b"));
            tof.set("note", mkstr(
                "this endpoint owns no ToF estimate by construction "
                "(SS responder / DS initiator); the peer's result is never copied"));
        }
        o.set("tof", tof);

        // The type-level guarantees, surfaced so a consumer cannot miss them.
        o.set("yields_range", mkbool(false));
        o.set("is_hardware_measurement", mkbool(false));
        o.set("is_validated_measurement", mkbool(false));
    } else {
        o.set("completion", mkstr("not_complete"));
        o.set("estimate_available", mkbool(false));
        o.set("local_evidence_complete", mkbool(false));
        o.set("peer_evidence", mkstr("wire_claim"));
        o.set("measurement_valid", mkbool(false));
        o.set("execution_mode", mkstr("simulation"));
        o.set("protocol_completed", mkbool(false));
        o.set("terminal_completion", mkstr("not_complete"));
        Value tof = mkobj();
        tof.set("available", mkbool(false));
        tof.set("num", Value::make_null());
        tof.set("den", Value::make_null());
        o.set("tof", tof);
    }

    std::string cj = tr.counters.to_json_string();
    Value cv;
    std::string cerr;
    if (twr::json::parse(cj, cv, cerr))
        o.set("counters", cv);
    else
        o.set("counters", mkstr(cj));

    o.set("local_evidence", local_evidence_value(tr, same_clock, tr.role == twr::Role::Initiator));
    o.set("arm_rx", arm_rx_value(tr));
    o.set("frames", frames_value(tr));
    return o;
}

// A path/value record used for the executed snapshot and its hashes.  The text
// layout (path "=" value "\n") is trivial to reproduce independently, which is
// what lets a Python verifier recompute the hash and catch a mutation.
struct KV {
    std::string path;
    std::string value;
};

void add_core_fields(std::vector<KV>& out, const twr::CoreConfig& c, const std::string& p)
{
    out.push_back({ p + ".endpoint_id", c.endpoint_id });
    out.push_back({ p + ".protocol", twr::protocol_to_string(c.protocol) });
    out.push_back({ p + ".role", twr::role_to_string(c.role) });
    out.push_back({ p + ".pan_id", fmt_u16(c.pan_id) });
    out.push_back({ p + ".local_address", fmt_u16(c.local_address) });
    out.push_back({ p + ".peer_address", fmt_u16(c.peer_address) });
    out.push_back({ p + ".session_id", fmt_u16(c.session_id) });
    out.push_back({ p + ".session_generation",
                    twr::twr_int_to_text(static_cast<int64_t>(c.session_generation)) });
    out.push_back({ p + ".sequence_modulus", fmt_u16(c.sequence_modulus) });
    out.push_back({ p + ".initial_sequence", fmt_u16(c.initial_sequence) });
    out.push_back({ p + ".reply_deadline_ticks",
                    twr::twr_int_to_text(c.reply_deadline_ticks) });
    out.push_back({ p + ".exchange_timeout_ticks",
                    twr::twr_int_to_text(c.exchange_timeout_ticks) });
    out.push_back({ p + ".evidence_wait_ticks",
                    twr::twr_int_to_text(c.evidence_wait_ticks) });
    out.push_back({ p + ".result_queue_capacity", fmt_u32(c.result_queue_capacity) });
    out.push_back({ p + ".max_in_flight", fmt_u32(c.max_in_flight) });
    out.push_back({ p + ".local_domain", c.local_domain.to_string() });
    out.push_back({ p + ".peer_domain", c.peer_binding.peer_domain.to_string() });
    out.push_back({ p + ".peer_max_interval_ticks",
                    twr::twr_int_to_text(
                        static_cast<int64_t>(c.peer_binding.max_interval_ticks)) });
    out.push_back({ p + ".ratio.k_num",
                    twr::twr_int_to_text(static_cast<int64_t>(c.ratio.k_num())) });
    out.push_back({ p + ".ratio.k_den",
                    twr::twr_int_to_text(static_cast<int64_t>(c.ratio.k_den())) });
    out.push_back({ p + ".calibration.id", c.local_calibration.id });
    out.push_back({ p + ".calibration.epoch",
                    twr::twr_int_to_text(
                        static_cast<int64_t>(c.local_calibration.calibrated_epoch)) });
    out.push_back({ p + ".calibration.valid_from",
                    twr::twr_int_to_text(c.local_calibration.valid_from_ticks) });
    out.push_back({ p + ".calibration.valid_until",
                    twr::twr_int_to_text(c.local_calibration.valid_until_ticks) });
}

// The ACTUAL executed values, in a stable order.  A mutation of any of these
// (or of the request that produced them) changes `config_sha256`.
std::vector<KV> executed_field_records(const Scenario& sc)
{
    std::vector<KV> out;
    out.push_back({ "schema", "twr-m1b-demo/1" });
    out.push_back({ "simulation.execution", "offline_simulation" });
    out.push_back({ "simulation.mode", "protocol_estimate" });
    out.push_back({ "simulation.seed", twr::twr_int_to_text(static_cast<int64_t>(sc.seed)) });
    out.push_back({ "simulation.scenario_id", sc.scenario_id });
    out.push_back({ "source",
                    sc.has_twr_config ? std::string("twr_config") : std::string("envelope") });
    out.push_back({ "requested_config_hash", sc.requested_config_fnv });
    out.push_back({ "effective_config_hash", sc.effective_config_fnv });
    out.push_back({ "runtime.exchange_id",
                    twr::twr_int_to_text(static_cast<int64_t>(sc.exchange_id != 0
                                                                  ? sc.exchange_id
                                                                  : 1)) });
    out.push_back({ "adapter.plan_marker_offset_ticks",
                    twr::twr_int_to_text(sc.plan_marker_offset_ticks) });
    out.push_back({ "adapter.plan_command_lead_ticks",
                    twr::twr_int_to_text(sc.plan_command_lead_ticks) });
    out.push_back({ "wire.timestamp_bits", fmt_u16(sc.wire_timestamp_bits) });
    add_core_fields(out, make_core_config(sc, twr::Role::Initiator), "core.a");
    add_core_fields(out, make_core_config(sc, twr::Role::Responder), "core.b");
    return out;
}

std::vector<KV> profile_field_records(const Scenario& sc)
{
    std::vector<KV> out;
    const twr::FrameProfile* ps[2] = { &sc.profile_a, &sc.profile_b };
    const char* names[2] = { "profile.a", "profile.b" };
    for (int i = 0; i < 2; ++i) {
        const std::string p = names[i];
        out.push_back({ p + ".version", twr::twr_int_to_text(ps[i]->version) });
        out.push_back({ p + ".timestamp_bits",
                        fmt_u16(ps[i]->timestamp_bits) });
        out.push_back({ p + ".timestamp_unit_hz",
                        twr::twr_double_to_text(ps[i]->timestamp_unit_hz) });
        out.push_back({ p + ".max_psdu_bytes",
                        fmt_u16(ps[i]->max_psdu_bytes) });
        out.push_back({ p + ".fcs_appended_by_modulation_layer",
                        twr::twr_bool_to_text(ps[i]->fcs_appended_by_modulation_layer) });
    }
    return out;
}

std::string kv_canonical_text(const std::vector<KV>& fields)
{
    std::string text;
    for (const KV& f : fields) {
        text += f.path;
        text += '=';
        text += f.value;
        text += '\n';
    }
    return text;
}

Value kv_array_value(const std::vector<KV>& fields)
{
    Value arr = Value::make_array();
    for (const KV& f : fields) {
        Value o = mkobj();
        o.set("path", mkstr(f.path));
        o.set("value", mkstr(f.value));
        arr.push(o);
    }
    return arr;
}

Value parse_json_or_null(const std::string& text)
{
    if (text.empty())
        return Value::make_null();
    Value v;
    std::string err;
    if (!twr::json::parse(text, v, err))
        return Value::make_string(text);
    return v;
}

// Forward declarations for helpers used while building the output.
int64_t driver_prop(const Scenario& sc, uint8_t source);

Value build_output(const Scenario& sc, const DemoDriver& driver, const std::string& input_sha,
                    const std::string& config_sha, const std::string& profile_sha)
{
    const EndpointTrace& ta = driver.trace_a();
    const EndpointTrace& tb = driver.trace_b();
    const bool same_clock = twr::clock_domain_same_identity(sc.dom_a, sc.dom_b) &&
                            twr::clock_domain_same_epoch(sc.dom_a, sc.dom_b);

    Value out = mkobj();
    out.set("schema", mkstr("twr-m1b-demo/1"));

    Value sim = mkobj();
    sim.set("execution", mkstr("offline_simulation"));
    sim.set("mode", mkstr("protocol_estimate"));
    sim.set("seed", mkint(static_cast<int64_t>(sc.seed)));
    sim.set("scenario_id", mkstr(sc.scenario_id));
    sim.set("protocol_estimate", mkbool(true));
    sim.set("hardware_ranging", mkbool(false));
    sim.set("note", mkstr("offline two-endpoint protocol simulation; NOT a hardware measurement"));
    out.set("simulation", sim);

    out.set("protocol", mkstr(twr::protocol_to_string(sc.protocol)));

    Value status = mkobj();
    status.set("ok", mkbool(true));
    out.set("status", status);

    Value prov = mkobj();
    prov.set("peer_evidence", mkstr("wire_claim"));
    prov.set("measurement_valid", mkbool(false));
    prov.set("execution_mode", mkstr("simulation"));
    prov.set("is_hardware_ranging", mkbool(false));
    prov.set("note", mkstr(
        "a wire tick is a protocol declaration, not a peer hardware event; the estimate "
        "cannot be read as a validated range"));
    out.set("provenance", prov);

    Value model = mkobj();
    model.set("note", mkstr(
        "observer-only echo of the transport ground truth; this is never copied into an "
        "endpoint and no distance is a measurement"));
    model.set("distance_m", mknum(sc.distance_m));
    model.set("a_to_b_propagation_ticks", mkint(driver_prop(sc, twr::kEndpointA)));
    model.set("b_to_a_propagation_ticks", mkint(driver_prop(sc, twr::kEndpointB)));
    out.set("transport_model", model);

    // The exact clock ratio the formulas consumed (k = fA/fB, direction fixed).
    Value cr = mkobj();
    cr.set("k_num", mkint(sc.ratio.k_num()));
    cr.set("k_den", mkint(sc.ratio.k_den()));
    cr.set("source", mkstr(twr::clock_ratio_source_to_string(sc.ratio.source())));
    cr.set("domain_a", mkstr(sc.dom_a.to_string()));
    cr.set("domain_b", mkstr(sc.dom_b.to_string()));
    out.set("clock_ratio", cr);

    // R09: the configuration authority.  `requested` and `effective` are the
    // config module's own views; `executed_fields` are the ACTUAL values both
    // endpoint cores ran with (their hash is `config_sha256`); `mapping` is the
    // explicit requested -> effective -> core table; `unconsumed` names every
    // requested field the offline core does not use, with its use and limit.
    const std::vector<KV> executed = executed_field_records(sc);
    const std::vector<KV> profile = profile_field_records(sc);
    Value cfg = mkobj();
    cfg.set("note", mkstr(
        "configuration authority: the core snapshot was built from the validated "
        "twr_config when present, otherwise from the envelope; this is a configuration echo, "
        "not a measurement"));
    cfg.set("source", mkstr(sc.has_twr_config ? "twr_config" : "envelope"));
    cfg.set("requested", parse_json_or_null(sc.requested_config_json));
    cfg.set("effective", parse_json_or_null(sc.effective_config_json));
    cfg.set("requested_config_hash", mkstr(sc.requested_config_fnv));
    cfg.set("effective_config_hash", mkstr(sc.effective_config_fnv));
    cfg.set("canonical_layout", mkstr("one \"path=value\" per entry, joined with '\\n'"));
    cfg.set("executed_fields", kv_array_value(executed));
    cfg.set("profile_fields", kv_array_value(profile));

    Value map_arr = Value::make_array();
    for (const MappingRow& m : sc.mappings) {
        Value o = mkobj();
        o.set("field", mkstr(m.field));
        o.set("requested", mkstr(m.requested));
        o.set("effective", mkstr(m.effective));
        o.set("core", mkstr(m.core));
        o.set("source", mkstr(m.source));
        o.set("consumed", mkbool(m.consumed));
        o.set("note", mkstr(m.note));
        map_arr.push(o);
    }
    cfg.set("mapping", map_arr);

    Value unc_arr = Value::make_array();
    for (const UnconsumedRow& u : sc.unconsumed) {
        Value o = mkobj();
        o.set("field", mkstr(u.field));
        o.set("use", mkstr(u.use));
        o.set("limit", mkstr(u.limit));
        unc_arr.push(o);
    }
    cfg.set("unconsumed", unc_arr);
    out.set("configuration", cfg);

    out.set("input_sha256", mkstr(input_sha));
    out.set("config_sha256", mkstr(config_sha));
    out.set("profile_sha256", mkstr(profile_sha));

    Value eps = mkobj();
    eps.set("a", endpoint_value(ta, make_core_config(sc, twr::Role::Initiator), same_clock));
    eps.set("b", endpoint_value(tb, make_core_config(sc, twr::Role::Responder), same_clock));
    out.set("endpoints", eps);

    return out;
}

// Compute the transport propagation exactly as the fake link does, for the
// observer echo only.
int64_t driver_prop(const Scenario& sc, uint8_t source)
{
    const twr::ClockDomain& dest =
        (source == twr::kEndpointA) ? sc.dom_b : sc.dom_a;
    const double seconds = sc.distance_m / twr::kFakeLinkSpeedOfLightMps;
    return static_cast<int64_t>(std::llround(seconds * dest.tick_rate_hz));
}

Value error_output(const std::string& message)
{
    Value out = mkobj();
    out.set("schema", mkstr("twr-m1b-demo/1"));
    Value status = mkobj();
    status.set("ok", mkbool(false));
    status.set("error", mkstr(message));
    out.set("status", status);
    return out;
}

int write_output(const std::string& path, const Value& v, std::string& why)
{
    std::string text;
    std::string err;
    if (!twr::json::dump(v, text, err, true)) {
        why = "cannot serialise the output JSON: " + err;
        return 1;
    }
    if (path == "-") {
        std::cout << text;
        if (!std::cout.good()) {
            why = "stdout write failed";
            return 1;
        }
        return 0;
    }
    std::ofstream ofs(path, std::ios::binary);
    if (!ofs) {
        why = "cannot open output " + path;
        return 1;
    }
    ofs << text;
    if (!ofs.good()) {
        why = "output write failed";
        return 1;
    }
    return 0;
}

} // namespace demo

// ===========================================================================
// main
// ===========================================================================

static void usage(const char* argv0)
{
    std::fprintf(stderr,
                 "usage: %s --request <in.json> --output <out.json|->\n"
                 "       runs the M1-B deterministic two-endpoint SS/DS TWR "
                 "protocol simulation.\n",
                 argv0);
}

int main(int argc, char** argv)
{
    std::string request_path;
    std::string output_path;

    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--request" && i + 1 < argc) {
            request_path = argv[++i];
        } else if (a == "--output" && i + 1 < argc) {
            output_path = argv[++i];
        } else if (a == "--help" || a == "-h") {
            usage(argv[0]);
            return 0;
        } else {
            std::fprintf(stderr, "unknown argument: %s\n", a.c_str());
            usage(argv[0]);
            return 2;
        }
    }
    if (request_path.empty() || output_path.empty()) {
        usage(argv[0]);
        return 2;
    }

    std::string request_text;
    std::string why;
    if (request_path == "-") {
        request_text = demo::read_stream(std::cin);
    } else {
        request_text = demo::read_file(request_path, why);
        if (!why.empty()) {
            std::fprintf(stderr, "error: %s\n", why.c_str());
            return 2;
        }
    }
    if (request_text.empty()) {
        std::fprintf(stderr, "error: request is empty\n");
        return 2;
    }

    const std::string input_sha = demo::sha256_hex(request_text);

    Value root;
    std::string perr;
    if (!twr::json::parse(request_text, root, perr)) {
        const std::string msg = "request is not valid JSON: " + perr;
        std::fprintf(stderr, "error: %s\n", msg.c_str());
        demo::write_output(output_path, demo::error_output(msg), why);
        return 2;
    }

    demo::Scenario sc;
    why.clear();
    if (!demo::parse_scenario(root, sc, why)) {
        std::fprintf(stderr, "error: %s\n", why.c_str());
        demo::write_output(output_path, demo::error_output(why), why);
        return 2;
    }

    why.clear();
    if (!demo::resolve_execution_config(sc, why)) {
        std::fprintf(stderr, "error: %s\n", why.c_str());
        demo::write_output(output_path, demo::error_output(why), why);
        return 2;
    }

    // The hashes are taken over the ACTUAL executed values (not over the
    // request), so a mutation of any executed field is detectable by
    // recomputing the canonical text the output itself publishes.
    const std::string config_sha =
        demo::sha256_hex(demo::kv_canonical_text(demo::executed_field_records(sc)));
    const std::string profile_sha =
        demo::sha256_hex(demo::kv_canonical_text(demo::profile_field_records(sc)));

    demo::DemoDriver driver(sc);
    why.clear();
    if (!driver.run(why)) {
        std::fprintf(stderr, "error: %s\n", why.c_str());
        demo::write_output(output_path, demo::error_output(why), why);
        return 2;
    }

    Value out = demo::build_output(sc, driver, input_sha, config_sha, profile_sha);

    // The output hash is taken over the canonical body WITHOUT this field, so a
    // verifier can reproduce it deterministically.  It is recorded last.
    std::string body_text;
    std::string berr;
    if (!twr::json::dump(out, body_text, berr, false)) {
        std::fprintf(stderr, "error: cannot serialise output body: %s\n", berr.c_str());
        return 1;
    }
    out.set("output_sha256", demo::mkstr(demo::sha256_hex(body_text)));

    why.clear();
    const int rc = demo::write_output(output_path, out, why);
    if (rc != 0) {
        std::fprintf(stderr, "error: %s\n", why.c_str());
        return 1;
    }

    std::fprintf(stderr,
                 "twr_fake_demo: %s protocol=%s scenario=%s -> %s "
                 "(simulation / protocol estimate; measurement_valid=false)\n",
                 request_path.c_str(), twr::protocol_to_string(sc.protocol),
                 sc.scenario_id.c_str(), output_path.c_str());
    // R09: the requested -> effective -> core mapping is BOTH in the machine
    // output (configuration.mapping) and printed here, so the authority of the
    // run is visible at a glance and a non-consumed field can never look used.
    std::fprintf(stderr, "twr_fake_demo: config authority=%s requested_hash=%s "
                         "effective_hash=%s\n",
                 sc.has_twr_config ? "twr_config" : "envelope",
                 sc.requested_config_fnv.empty() ? "(none)" : sc.requested_config_fnv.c_str(),
                 sc.effective_config_fnv.empty() ? "(none)" : sc.effective_config_fnv.c_str());
    for (const demo::MappingRow& m : sc.mappings) {
        std::fprintf(stderr, "  map %-44s %s -> %s -> %s [%s]%s%s\n", m.field.c_str(),
                     m.requested.c_str(), m.effective.c_str(), m.core.c_str(),
                     m.source.c_str(), m.consumed ? "" : " NOT-CONSUMED: ",
                     m.consumed ? "" : m.note.c_str());
    }
    return 0;
}
