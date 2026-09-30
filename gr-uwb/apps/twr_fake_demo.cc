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
 * by the EXISTING `from_json_string` + `validate()` and cross-checked; a
 * rejection refuses the whole run (B17: "demo rejects a wrong config").
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

struct Scenario {
    // simulation gate
    std::string execution;
    std::string mode;
    uint64_t seed = 1;
    std::string scenario_id;

    twr::Protocol protocol = twr::Protocol::Ss;

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

    // Optional existing TwrConfig, validated by the existing validator.
    bool has_twr_config = false;
    Value twr_config;
};

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
};

struct EndpointTrace {
    std::string endpoint_id;
    twr::Role role = twr::Role::Initiator;
    uint16_t local_address = 0;
    uint16_t peer_address = 0;

    std::vector<FrameTrace> frames;

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

        twr::CoreConfig ca = make_core_config(twr::Role::Initiator, sc_.a_local, sc_.a_peer);
        twr::CoreConfig cb = make_core_config(twr::Role::Responder, sc_.b_local, sc_.b_peer);
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
            ev.exchange.valid = true;
            ev.exchange.value = 1;
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
    twr::CoreConfig make_core_config(twr::Role role, uint16_t local, uint16_t peer) const
    {
        twr::CoreConfig c;
        c.endpoint_id = (role == twr::Role::Initiator) ? "A" : "B";
        c.protocol = sc_.protocol;
        c.role = role;
        c.pan_id = sc_.pan_id;
        c.local_address = local;
        c.peer_address = peer;
        c.session_id = sc_.session_id;
        c.session_generation = sc_.session_generation;
        c.sequence_modulus = sc_.sequence_modulus;
        c.initial_sequence = sc_.initial_sequence;
        c.local_domain = (role == twr::Role::Initiator) ? sc_.dom_a : sc_.dom_b;
        c.peer_binding.peer_domain = (role == twr::Role::Initiator) ? sc_.dom_b : sc_.dom_a;
        c.peer_binding.session_generation = sc_.session_generation;
        c.peer_binding.binding_generation = 1;
        c.peer_binding.peer_marker = twr::TimestampMarker::RmarkerTx;
        c.peer_binding.unit_convention = "device_ticks";
        c.peer_binding.calibration_convention_id = sc_.cal.id;
        c.peer_binding.max_interval_ticks = sc_.max_interval_ticks;
        c.peer_binding.sequence_modulus = sc_.sequence_modulus;
        c.ratio = sc_.ratio;
        c.frame_profile = (role == twr::Role::Initiator) ? sc_.profile_a : sc_.profile_b;
        c.local_calibration = sc_.cal;
        c.reply_deadline_ticks = 0;
        c.exchange_timeout_ticks = 0;
        c.evidence_wait_ticks = 0;
        c.result_queue_capacity = 8;
        c.max_in_flight = 1;
        return c;
    }

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

    twr::Timestamp tx_timestamp(int64_t ticks, const twr::ClockDomain& dom) const
    {
        twr::Timestamp ts;
        twr::Timestamp::from_ticks(ticks, dom, twr::TimestampMarker::RmarkerTx,
                                   twr::TimestampSource::ScheduledCalibrated,
                                   twr::timestamp_required_corrections(
                                       twr::TimestampMarker::RmarkerTx),
                                   ts);
        ts.calibration_id = sc_.cal.id;
        return ts;
    }

    static twr::TxSendEvidence plan_evidence()
    {
        twr::TxSendEvidence e;
        e.command_time_recorded = true;
        e.quantised_instant_recorded = true;
        e.marker_offset_recorded = true;
        e.calibrated_air_time_recorded = true;
        e.send_accepted = false;
        e.outcome = twr::TxOutcome::Unknown; // resolved by an explicit event
        return e;
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
            case twr::CoreActionKind::ArmRx:
                break;
            case twr::CoreActionKind::PrepareTx: {
                int64_t planned = 0;
                if (!choose_planned(tr, a.tx_intent, planned, why))
                    return false;
                planned_by_token_[a.token.value] = planned;
                planned_intent_by_token_[a.token.value] = a.tx_intent;

                twr::CoreEvent ev;
                ev.kind = twr::CoreEventKind::TxPlanned;
                ev.event_id = 2000u + a.token.value;
                ev.token = a.token;
                ev.tx_intent = a.tx_intent;
                ev.planned_tx_time = tx_timestamp(planned, domain_of(ep));
                ev.tx_evidence = plan_evidence();
                ev.deadline_verdict_feasible = true;
                ev.deadline_slack_ticks = 0;
                twr::CoreActionBatch b = core.post(ev);
                if (!handle(core, tr, ep, b, why))
                    return false;
                break;
            }
            case twr::CoreActionKind::SubmitTx: {
                const auto it = planned_by_token_.find(a.token.value);
                if (it == planned_by_token_.end()) {
                    why = "SubmitTx for an unknown prepared token";
                    return false;
                }
                const int64_t air = it->second;
                const int idx = type_index(a.tx_intent);
                tr.tx_planned_set[idx] = true;
                tr.tx_planned_ticks[idx] = air;

                FrameTrace ft;
                ft.type = a.tx_intent;
                ft.dir = "tx";
                ft.peer_id = (ep == twr::kEndpointA) ? "B" : "A";
                ft.ticks = air;
                ft.token = a.token.value;
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

                twr::CoreEvent ea;
                ea.kind = twr::CoreEventKind::TxAccepted;
                ea.event_id = 3000u + a.token.value;
                ea.token = a.token;
                (void)core.post(ea);

                twr::CoreEvent eo;
                eo.kind = twr::CoreEventKind::TxOutcomeResolved;
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

    const Scenario& sc_;
    twr::FakeTwrLink link_;
    twr::EndpointCore core_a_;
    twr::EndpointCore core_b_;
    EndpointTrace ta_, tb_;
    std::map<uint64_t, int64_t> planned_by_token_;
    std::map<uint64_t, twr::FrameType> planned_intent_by_token_;
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

// Validate the optional TwrConfig with the EXISTING parser/validator, and
// cross-check it against the envelope.
bool validate_twr_config(const Scenario& sc, std::string& why)
{
    if (!sc.has_twr_config)
        return true;
    std::string text;
    std::string derr;
    if (!twr::json::dump(sc.twr_config, text, derr, false)) {
        why = "twr_config is not serialisable: " + derr;
        return false;
    }
    twr::TwrConfig cfg;
    const twr::ValidationReport parsed = twr::from_json_string(text, cfg);
    if (!parsed.ok()) {
        why = "twr_config failed the existing parser/validator: " + parsed.to_string();
        return false;
    }
    const twr::ValidationReport report = twr::validate(cfg);
    if (!report.ok()) {
        why = "twr_config failed validate(): " + report.to_string();
        return false;
    }
    // Cross-check: the envelope and the config must name the same exchange.
    if (cfg.session.protocol != sc.protocol) {
        why = "twr_config.session.protocol does not match the envelope protocol";
        return false;
    }
    if (cfg.session.session_id != sc.session_id) {
        why = "twr_config.session.session_id does not match the envelope session_id";
        return false;
    }
    if (cfg.session.local_address != sc.a_local ||
        cfg.session.peer_address != sc.a_peer) {
        why = "twr_config session addresses do not match the envelope's initiator addressing";
        return false;
    }
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
        if (tr.terminal.failed())
            o.set("terminal_failure_reason",
                  mkstr(twr::exchange_status_to_string(tr.terminal.failure_reason)));
        o.set("failure_reason", mkstr(twr::exchange_status_to_string(e.failure_reason)));
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
    o.set("frames", frames_value(tr));
    return o;
}

std::string scenario_canonical_text(const Scenario& sc)
{
    char buf[1024];
    std::snprintf(buf, sizeof(buf),
                  "twr-m1b-demo/1|protocol=%s|session=%u|gen=%llu|mod=%u|init_seq=%u|"
                  "pan=%u|a=%u/%u|b=%u/%u|domA=%s|domB=%s|k=%lld/%lld|"
                  "cal=%s@%llu|wire_bits=%u|max_interval=%llu|distance=%.17g|"
                  "latA=%lld|latB=%lld|poll=%lld|reply=%lld|final=%lld|seed=%llu",
                  twr::protocol_to_string(sc.protocol),
                  static_cast<unsigned>(sc.session_id),
                  static_cast<unsigned long long>(sc.session_generation),
                  static_cast<unsigned>(sc.sequence_modulus),
                  static_cast<unsigned>(sc.initial_sequence), static_cast<unsigned>(sc.pan_id),
                  static_cast<unsigned>(sc.a_local), static_cast<unsigned>(sc.a_peer),
                  static_cast<unsigned>(sc.b_local), static_cast<unsigned>(sc.b_peer),
                  sc.dom_a.to_string().c_str(), sc.dom_b.to_string().c_str(),
                  static_cast<long long>(sc.ratio.k_num()),
                  static_cast<long long>(sc.ratio.k_den()), sc.cal.id.c_str(),
                  static_cast<unsigned long long>(sc.cal.calibrated_epoch),
                  static_cast<unsigned>(sc.wire_timestamp_bits),
                  static_cast<unsigned long long>(sc.max_interval_ticks), sc.distance_m,
                  static_cast<long long>(sc.tx_latency_a),
                  static_cast<long long>(sc.tx_latency_b),
                  static_cast<long long>(sc.poll_air_ticks),
                  static_cast<long long>(sc.response_reply_ticks),
                  static_cast<long long>(sc.final_reply_ticks),
                  static_cast<unsigned long long>(sc.seed));
    return std::string(buf);
}

std::string profile_canonical_text(const Scenario& sc)
{
    std::string s;
    s += "profileA{bits=" + std::to_string(static_cast<unsigned>(sc.profile_a.timestamp_bits)) +
         ",unit=" + twr::twr_double_to_text(sc.profile_a.timestamp_unit_hz) +
         ",max_psdu=" + std::to_string(static_cast<unsigned long long>(sc.profile_a.max_psdu_bytes)) +
         "}\n";
    s += "profileB{bits=" + std::to_string(static_cast<unsigned>(sc.profile_b.timestamp_bits)) +
         ",unit=" + twr::twr_double_to_text(sc.profile_b.timestamp_unit_hz) +
         ",max_psdu=" + std::to_string(static_cast<unsigned long long>(sc.profile_b.max_psdu_bytes)) +
         "}\n";
    return s;
}

// Forward declarations for helpers used while building the output.
int64_t driver_prop(const Scenario& sc, uint8_t source);
twr::CoreConfig CoreConfigView(const Scenario& sc, twr::Role role);

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

    // A validated echo of the execution snapshot (what the C++ core actually
    // used), so a consumer never has to re-derive it from the request.  This is
    // a CONFIGURATION echo, not a measurement.
    Value cfg = mkobj();
    cfg.set("note", mkstr("validated execution snapshot echo; not a measurement"));
    Value sess = mkobj();
    sess.set("session_id", mkint(sc.session_id));
    sess.set("session_generation", mkint(static_cast<int64_t>(sc.session_generation)));
    sess.set("sequence_modulus", mkint(sc.sequence_modulus));
    sess.set("initial_sequence", mkint(sc.initial_sequence));
    sess.set("pan_id", mkint(sc.pan_id));
    cfg.set("session", sess);
    Value eps2 = mkobj();
    Value ea2 = mkobj();
    ea2.set("local_address", mkint(sc.a_local));
    ea2.set("peer_address", mkint(sc.a_peer));
    Value eb2 = mkobj();
    eb2.set("local_address", mkint(sc.b_local));
    eb2.set("peer_address", mkint(sc.b_peer));
    eps2.set("a", ea2);
    eps2.set("b", eb2);
    cfg.set("endpoints", eps2);
    Value cals = mkobj();
    cals.set("id", mkstr(sc.cal.id));
    cals.set("calibrated_epoch", mkint(static_cast<int64_t>(sc.cal.calibrated_epoch)));
    cals.set("valid_from_ticks", mkint(sc.cal.valid_from_ticks));
    cals.set("valid_until_ticks", mkint(sc.cal.valid_until_ticks));
    cfg.set("calibration", cals);
    out.set("configuration", cfg);

    out.set("input_sha256", mkstr(input_sha));
    out.set("config_sha256", mkstr(config_sha));
    out.set("profile_sha256", mkstr(profile_sha));

    Value eps = mkobj();
    eps.set("a", endpoint_value(ta, CoreConfigView(sc, twr::Role::Initiator), same_clock));
    eps.set("b", endpoint_value(tb, CoreConfigView(sc, twr::Role::Responder), same_clock));
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

// A tiny view helper so endpoint_value does not need the driver's private
// mapping.  It reconstructs the endpoint's own CoreConfig fields from the
// scenario (the same values driver.make_core_config used).
twr::CoreConfig CoreConfigView(const Scenario& sc, twr::Role role)
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
    c.ratio = sc.ratio;
    c.frame_profile = (role == twr::Role::Initiator) ? sc.profile_a : sc.profile_b;
    c.local_calibration = sc.cal;
    return c;
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
    if (!demo::validate_twr_config(sc, why)) {
        std::fprintf(stderr, "error: %s\n", why.c_str());
        demo::write_output(output_path, demo::error_output(why), why);
        return 2;
    }

    demo::DemoDriver driver(sc);
    why.clear();
    if (!driver.run(why)) {
        std::fprintf(stderr, "error: %s\n", why.c_str());
        demo::write_output(output_path, demo::error_output(why), why);
        return 2;
    }

    const std::string config_sha = demo::sha256_hex(demo::scenario_canonical_text(sc));
    const std::string profile_sha = demo::sha256_hex(demo::profile_canonical_text(sc));

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
    return 0;
}
