/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * QA for the pure C++ pipeline output writer (Track 1).
 *
 * Cases
 * -----
 *  1. UCR4 header geometry and block-floating round trip (incl. the zero /
 *     non-finite fail-closed path).
 *  2. Aggregate write vs per-frame write: `<base>.ucr4` and `<base>.jsonl`
 *     are byte-identical (memcmp of the whole files).
 *  3. Failed frames: only ok frames reach `.ucr4`, `file_offset_taps` equals
 *     the cumulative ok tap count, failures still appear in the jsonl with
 *     `tap_count: 0`.
 *  4. `demod_results.jsonl` round-trips through
 *     `gr::uwb::core::parse_demod_jsonl_line()`, and `psdu_hex` /
 *     `psdu_fnv1a` describe the same bytes.
 *  5. Slow-writer ledger: tiny queue + a per-frame sleep in the writer thread;
 *     the submitting thread never blocks, `offered == written + dropped`,
 *     `dropped > 0`.
 *  6. Disabled writer is inert and creates nothing.
 *  7. `comm_write_only_ok` filters failed frames into `comm_skipped`.
 */

#define BOOST_TEST_MODULE qa_uwb_pipeline_output_writer
#include <boost/test/unit_test.hpp>

#include <gnuradio/uwb/uwb_pipeline_output_writer.h>
#include <gnuradio/uwb/uwb_scheduled_dump_io.h>
#include <gnuradio/uwb/uwb_ucr4_format.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <limits>
#include <string>
#include <thread>
#include <vector>

namespace pl = gr::uwb::pipeline;
namespace core = gr::uwb::core;

// Test-only hook defined in uwb_pipeline_output_writer.cc.  It is declared
// there with default visibility because the library is built with
// -fvisibility=hidden.
namespace gr {
namespace uwb {
namespace pipeline {
std::function<void()>& test_slow_hook_storage();
} // namespace pipeline
} // namespace uwb
} // namespace gr

namespace {

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
std::string
make_temp_dir(const std::string& tag)
{
    const char* env = std::getenv("TMPDIR");
    const std::filesystem::path base =
        (env != nullptr && env[0] != '\0') ? std::filesystem::path(env)
                                           : std::filesystem::temp_directory_path();
    const std::filesystem::path dir =
        base / ("uwb_qa_pipeline_writer_" + tag);
    std::error_code ec;
    std::filesystem::remove_all(dir, ec);
    std::filesystem::create_directories(dir, ec);
    return dir.string();
}

std::vector<char>
read_file(const std::string& path)
{
    std::ifstream f(path, std::ios::binary);
    BOOST_REQUIRE_MESSAGE(f.good(), "cannot open " + path);
    f.seekg(0, std::ios::end);
    const std::streamoff n = f.tellg();
    f.seekg(0, std::ios::beg);
    std::vector<char> out(static_cast<size_t>(n));
    if (n > 0)
        f.read(out.data(), n);
    return out;
}

std::vector<std::string>
read_lines(const std::string& path)
{
    std::ifstream f(path);
    BOOST_REQUIRE_MESSAGE(f.good(), "cannot open " + path);
    std::vector<std::string> lines;
    std::string line;
    while (std::getline(f, line)) {
        if (!line.empty())
            lines.push_back(line);
    }
    return lines;
}

uint64_t
fnv1a64(const uint8_t* data, size_t n)
{
    uint64_t h = 14695981039346656037ULL;
    for (size_t i = 0; i < n; ++i) {
        h ^= static_cast<uint64_t>(data[i]);
        h *= 1099511628211ULL;
    }
    return h;
}

std::string
to_hex(const std::vector<uint8_t>& bytes)
{
    static const char kHex[] = "0123456789abcdef";
    std::string out;
    for (uint8_t b : bytes) {
        out.push_back(kHex[b >> 4]);
        out.push_back(kHex[b & 0x0Fu]);
    }
    return out;
}

std::vector<uint8_t>
from_hex(const std::string& hex)
{
    auto nibble = [](char c) -> int {
        if (c >= '0' && c <= '9')
            return c - '0';
        if (c >= 'a' && c <= 'f')
            return c - 'a' + 10;
        if (c >= 'A' && c <= 'F')
            return c - 'A' + 10;
        return -1;
    };
    std::vector<uint8_t> out;
    if (hex.size() % 2 != 0)
        return out;
    for (size_t i = 0; i < hex.size(); i += 2) {
        const int hi = nibble(hex[i]);
        const int lo = nibble(hex[i + 1]);
        if (hi < 0 || lo < 0)
            return {};
        out.push_back(static_cast<uint8_t>((hi << 4) | lo));
    }
    return out;
}

// Raw token that follows "key": in a JSONL line (string tokens keep their
// quotes).  Empty when the key is absent.
std::string
raw_value(const std::string& line, const char* key)
{
    size_t i = 0;
    if (!core::json_min::find_key(line, key, i))
        return {};
    size_t j = i;
    if (j < line.size() && line[j] == '"') {
        ++j;
        while (j < line.size() && line[j] != '"') {
            if (line[j] == '\\' && j + 1 < line.size())
                ++j;
            ++j;
        }
        if (j < line.size())
            ++j;
        return line.substr(i, j - i);
    }
    while (j < line.size() && line[j] != ',' && line[j] != '}')
        ++j;
    return line.substr(i, j - i);
}

uint64_t
raw_u64(const std::string& line, const char* key, uint64_t def = 0)
{
    const std::string v = raw_value(line, key);
    if (v.empty())
        return def;
    try {
        return std::stoull(v);
    } catch (...) {
        return def;
    }
}

double
raw_f64(const std::string& line, const char* key, double def = 0.0)
{
    const std::string v = raw_value(line, key);
    if (v.empty() || v == "null")
        return def;
    try {
        return std::stod(v);
    } catch (...) {
        return def;
    }
}

std::string
raw_str(const std::string& line, const char* key)
{
    return core::json_min::as_str(line, key);
}

// Booleans are emitted unquoted; raw_value returns "true"/"false".
std::string
raw_bool(const std::string& line, const char* key)
{
    return raw_value(line, key);
}

// ---------------------------------------------------------------------------
// Deterministic frame builder
// ---------------------------------------------------------------------------
struct FrameSpec {
    uint64_t window_id = 0;
    bool radar_executed = true;
    bool radar_sfd_found = true;
    int64_t radar_detected_origin = 1000;
    int64_t radar_detected_start = 1016;
    int64_t radar_cir_origin = 1024;
    size_t cir_pre = 16;
    size_t cir_post = 100;
    size_t tap_count = 116;
    // The kernel's authoritative peak tap and the RX-detected SFD start.  Both
    // must be published verbatim: the writer must not recompute peak_tap with a
    // different norm (that would pair a tap with a metric measured by another
    // norm), and sfd_start_sample is NOT the detected SYNC origin.
    size_t peak_tap = 28;
    int64_t sfd_start_sample = 900;

    bool comm_attempted = false;
    bool comm_ok = false;
    bool comm_fcs_ok = false;
    const char* comm_status_reason = "";
    size_t psdu_n = 0;
};

uint32_t
lcg(uint32_t& state)
{
    state = state * 1664525u + 1013904223u;
    return state;
}

void
fill_slot(pl::OutputPayloadSlot& slot, const FrameSpec& f)
{
    slot.clear();
    pl::OutputWindowRecord& r = slot.record;
    r.window_id = f.window_id;
    r.schedule_index = f.window_id + 7;
    r.worker_id = static_cast<int>(f.window_id % 4);
    r.native_start_abs = static_cast<int64_t>(f.window_id) * 4000;
    r.native_sample_count = 4000;
    r.native_window_start = static_cast<int64_t>(f.window_id) * 4000;
    r.predicted_start_sample = 1000;

    r.sic_executed = true;
    r.sic_committed = true;
    r.actual_suppression_db = 28.25f;
    r.sic_status = "committed";

    r.radar_executed = f.radar_executed;
    r.radar_sfd_found = f.radar_sfd_found;
    r.radar_peak_metric = 0.078909032f; // the L2 magnitude at peak_tap
    r.radar_detected_start = f.radar_detected_start;
    r.radar_cfo_hz = 1234.5;
    r.radar_cir_origin = f.radar_cir_origin;
    r.radar_detected_origin = f.radar_detected_origin;
    r.radar_valid_reps = 8;
    r.peak_tap = f.peak_tap;
    r.sfd_start_sample = f.sfd_start_sample;
    r.sfd_metric = 0.5f;
    r.raw_l2_norm = 1.0f;
    r.cir_pre = f.cir_pre;
    r.cir_post = f.cir_post;

    r.comm_attempted = f.comm_attempted;
    r.comm_ok = f.comm_ok;
    r.comm_status_reason = f.comm_status_reason;
    r.comm_sync_origin = f.comm_attempted ? 1234 : -1;
    r.comm_sfd_start = f.comm_attempted ? 1300 : -1;
    r.comm_cfo_hz = 12345.0;
    r.comm_phr_ok = f.comm_ok;
    r.comm_phr_length = static_cast<uint16_t>(f.psdu_n);
    r.comm_fcs_ok = f.comm_fcs_ok;
    r.comm_payload_bytes = f.psdu_n;
    r.phr_rate_mbps = 6.81f;
    r.fcs_received = 0xABCD;
    r.fcs_calculated = f.comm_fcs_ok ? 0xABCD : 0x1234;
    r.phr_corrected = false;
    r.phr_uncorrectable = false;

    r.resample_us = 3.5;
    r.t_total_us = 12.5;
    r.sensing_us = 42.0;

    uint32_t state = 0x9E3779B9u ^ static_cast<uint32_t>(f.window_id);
    for (size_t i = 0; i < f.tap_count; ++i) {
        const float re =
            static_cast<float>(lcg(state) >> 8) / 65536.0f - 0.5f;
        const float im =
            static_cast<float>(lcg(state) >> 8) / 65536.0f - 0.5f;
        slot.cir_taps.push_back(std::complex<float>(re, im));
    }
    for (size_t i = 0; i < f.psdu_n; ++i) {
        slot.psdu_bytes.push_back(
            static_cast<uint8_t>((f.window_id * 31u + i * 7u) & 0xFFu));
    }
    r.comm_psdu_fnv1a = slot.psdu_bytes.empty()
                            ? 0
                            : fnv1a64(slot.psdu_bytes.data(),
                                      slot.psdu_bytes.size());
}

pl::OutputRunMeta
make_meta(size_t max_psdu)
{
    pl::OutputRunMeta meta;
    meta.consumer_mode = "both";
    meta.native_rate = 737.28e6;
    meta.work_rate = 998.4e6;
    meta.uhd_center_freq = 6.5e9;
    meta.sic_policy = "preamble";
    meta.sic_backend = "unified";
    meta.cir_pre = 16;
    meta.cir_post = 100;
    meta.code_index = 9;
    meta.sync_repetitions = 64;
    meta.samples_per_symbol = 1016.0;
    meta.max_psdu_bytes = max_psdu;
    return meta;
}

struct RunResult {
    std::string dir;
    pl::OutputWriterStats stats;
};

RunResult
run_frames(const std::string& dir,
           const std::vector<FrameSpec>& frames,
           size_t agg_bytes = 1u << 20,
           size_t agg_ms = 50,
           size_t queue_capacity = 256,
           bool comm_only_ok = false,
           size_t max_taps = 116,
           size_t max_psdu = 127)
{
    pl::OutputPersistenceConfig cfg;
    cfg.output_dir = dir;
    cfg.writer_queue_capacity = queue_capacity;
    cfg.writer_aggregate_bytes = agg_bytes;
    cfg.writer_aggregate_window_ms = agg_ms;
    cfg.comm_write_only_ok = comm_only_ok;
    cfg.live_log_mode = pl::LiveLogMode::Off;

    pl::PipelineOutputWriter writer;
    BOOST_REQUIRE(writer.start(cfg, make_meta(max_psdu), max_taps, max_psdu));
    BOOST_REQUIRE(writer.enabled());
    for (const FrameSpec& f : frames) {
        pl::OutputPayloadSlot* slot = writer.try_acquire_slot();
        BOOST_REQUIRE_MESSAGE(slot != nullptr, "pool exhausted unexpectedly");
        fill_slot(*slot, f);
        BOOST_REQUIRE(writer.try_submit(slot));
    }
    writer.stop_and_drain();

    RunResult r;
    r.dir = dir;
    r.stats = writer.stats();
    return r;
}

} // namespace

// ---------------------------------------------------------------------------
// 1. UCR4 header geometry + block-floating round trip
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_ucr4_header_and_block_float)
{
    BOOST_CHECK_EQUAL(sizeof(pl::Ucr4Header), static_cast<size_t>(52));
    BOOST_CHECK_EQUAL(pl::ucr4_record_bytes(0), static_cast<size_t>(52));
    BOOST_CHECK_EQUAL(pl::ucr4_record_bytes(116),
                      static_cast<size_t>(52 + 116 * 4));

    // Field-for-field identity with the GNU Radio block's header: same byte
    // offsets for the fields MATLAB's read_uwb_cir.m reads.
    BOOST_CHECK_EQUAL(offsetof(pl::Ucr4Header, magic), static_cast<size_t>(0));
    BOOST_CHECK_EQUAL(offsetof(pl::Ucr4Header, pulse_id),
                      static_cast<size_t>(4));
    BOOST_CHECK_EQUAL(offsetof(pl::Ucr4Header, status),
                      static_cast<size_t>(8));
    BOOST_CHECK_EQUAL(offsetof(pl::Ucr4Header, tap_count),
                      static_cast<size_t>(10));
    BOOST_CHECK_EQUAL(offsetof(pl::Ucr4Header, repetition_index),
                      static_cast<size_t>(12));
    BOOST_CHECK_EQUAL(offsetof(pl::Ucr4Header, repetition_count),
                      static_cast<size_t>(14));
    BOOST_CHECK_EQUAL(offsetof(pl::Ucr4Header, sfd_metric),
                      static_cast<size_t>(16));
    BOOST_CHECK_EQUAL(offsetof(pl::Ucr4Header, cir_peak_metric),
                      static_cast<size_t>(20));
    BOOST_CHECK_EQUAL(offsetof(pl::Ucr4Header, peak_tap),
                      static_cast<size_t>(24));
    BOOST_CHECK_EQUAL(offsetof(pl::Ucr4Header, estimator_us),
                      static_cast<size_t>(28));
    BOOST_CHECK_EQUAL(offsetof(pl::Ucr4Header, freq_hz),
                      static_cast<size_t>(32));
    BOOST_CHECK_EQUAL(offsetof(pl::Ucr4Header, freq_offset_hz),
                      static_cast<size_t>(40));
    BOOST_CHECK_EQUAL(offsetof(pl::Ucr4Header, cir_scale),
                      static_cast<size_t>(48));

    // Round trip within 1 LSB of the block-float scale.
    const size_t n = 116;
    std::vector<std::complex<float>> in(n);
    for (size_t i = 0; i < n; ++i) {
        in[i] = std::complex<float>(std::sin(0.11f * static_cast<float>(i)) *
                                        1000.0f,
                                    std::cos(0.07f * static_cast<float>(i)) *
                                        700.0f);
    }
    std::vector<int16_t> sc16(n * 2, 0);
    const float scale = pl::encode_cir_sc16(in.data(), n, sc16.data());
    BOOST_CHECK(scale > 0.0f);

    int16_t peak = 0;
    for (int16_t v : sc16)
        peak = std::max<int16_t>(peak, static_cast<int16_t>(std::abs(v)));
    BOOST_CHECK_EQUAL(peak, static_cast<int16_t>(32767));

    std::vector<std::complex<float>> out(n);
    pl::decode_cir_sc16(sc16.data(), n, scale, out.data());
    float max_err = 0.0f;
    for (size_t i = 0; i < n; ++i) {
        BOOST_CHECK(std::isfinite(out[i].real()));
        BOOST_CHECK(std::isfinite(out[i].imag()));
        max_err = std::max(max_err, std::abs(out[i] - in[i]));
    }
    BOOST_CHECK_LE(max_err, scale);

    // Zero frame: scale 0, all-zero taps, no NaN after decoding.
    std::vector<std::complex<float>> zero(n, std::complex<float>(0.0f, 0.0f));
    std::fill(sc16.begin(), sc16.end(), static_cast<int16_t>(1234));
    const float zscale = pl::encode_cir_sc16(zero.data(), n, sc16.data());
    BOOST_CHECK_EQUAL(zscale, 0.0f);
    for (int16_t v : sc16)
        BOOST_CHECK_EQUAL(v, 0);
    pl::decode_cir_sc16(sc16.data(), n, zscale, out.data());
    for (size_t i = 0; i < n; ++i) {
        BOOST_CHECK_EQUAL(out[i].real(), 0.0f);
        BOOST_CHECK_EQUAL(out[i].imag(), 0.0f);
        BOOST_CHECK(!std::isnan(out[i].real()));
    }

    // Non-finite input must not leak NaN into the file: fail closed to zeros.
    std::vector<std::complex<float>> bad(n, std::complex<float>(1.0f, 1.0f));
    bad[3] = std::complex<float>(std::numeric_limits<float>::quiet_NaN(), 0.0f);
    const float bscale = pl::encode_cir_sc16(bad.data(), n, sc16.data());
    BOOST_CHECK_EQUAL(bscale, 0.0f);
    for (int16_t v : sc16)
        BOOST_CHECK_EQUAL(v, 0);

    // Status names: the domain is closed, unknown codes never look like "ok".
    BOOST_CHECK_EQUAL(std::string(pl::cir_status_name(0)), std::string("ok"));
    BOOST_CHECK_EQUAL(std::string(pl::cir_status_name(1)),
                      std::string("sfd_failed"));
    BOOST_CHECK_EQUAL(std::string(pl::cir_status_name(2)),
                      std::string("timing_failed"));
    BOOST_CHECK_EQUAL(std::string(pl::cir_status_name(3)),
                      std::string("cir_failed"));
    BOOST_CHECK_EQUAL(std::string(pl::cir_status_name(4)),
                      std::string("invalid_input"));
    BOOST_CHECK_EQUAL(std::string(pl::cir_status_name(5)),
                      std::string("internal_error"));
    BOOST_CHECK_EQUAL(std::string(pl::cir_status_name(65535)),
                      std::string("internal_error"));
}

// ---------------------------------------------------------------------------
// 2. Aggregate write vs per-frame write: byte identity
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_aggregate_matches_per_frame)
{
    std::vector<FrameSpec> frames;
    for (uint64_t i = 0; i < 40; ++i) {
        FrameSpec f;
        f.window_id = i;
        f.tap_count = 116;
        f.comm_attempted = true;
        f.comm_ok = (i % 3) != 0;
        f.comm_fcs_ok = f.comm_ok;
        f.comm_status_reason = f.comm_ok ? "ok" : "fcs_failed";
        f.psdu_n = f.comm_ok ? 32 : 0;
        frames.push_back(f);
    }

    const std::string dir_per_frame = make_temp_dir("agg_zero");
    const std::string dir_aggregate = make_temp_dir("agg_big");

    const RunResult a = run_frames(dir_per_frame, frames, /*agg_bytes=*/0,
                                   /*agg_ms=*/0);
    const RunResult b = run_frames(dir_aggregate, frames, /*agg_bytes=*/1u << 20,
                                   /*agg_ms=*/50);

    // Nothing may be dropped, otherwise the comparison would be meaningless.
    BOOST_CHECK_EQUAL(a.stats.dropped, 0u);
    BOOST_CHECK_EQUAL(b.stats.dropped, 0u);
    BOOST_CHECK_EQUAL(a.stats.written, 40u);
    BOOST_CHECK_EQUAL(b.stats.written, 40u);
    BOOST_CHECK_EQUAL(a.stats.cir_written, 40u);
    BOOST_CHECK_EQUAL(b.stats.cir_written, 40u);

    const std::vector<char> ucr4_a =
        read_file(dir_per_frame + "/cir.ucr4");
    const std::vector<char> ucr4_b = read_file(dir_aggregate + "/cir.ucr4");
    const std::vector<char> jsonl_a =
        read_file(dir_per_frame + "/cir.jsonl");
    const std::vector<char> jsonl_b = read_file(dir_aggregate + "/cir.jsonl");

    BOOST_CHECK_EQUAL(ucr4_a.size(), 40u * pl::ucr4_record_bytes(116));
    BOOST_CHECK_EQUAL(ucr4_a.size(), ucr4_b.size());
    BOOST_CHECK_EQUAL(jsonl_a.size(), jsonl_b.size());
    BOOST_CHECK_MESSAGE(ucr4_a == ucr4_b,
                        "cir.ucr4 differs between per-frame and aggregate "
                        "write modes");
    BOOST_CHECK_MESSAGE(jsonl_a == jsonl_b,
                        "cir.jsonl differs between per-frame and aggregate "
                        "write modes");
    BOOST_TEST_MESSAGE("byte identity: cir.ucr4 "
                       << ucr4_a.size() << " B, cir.jsonl " << jsonl_a.size()
                       << " B; flushes per-frame=" << a.stats.aggregate_flushes
                       << " aggregate=" << b.stats.aggregate_flushes);

    // The aggregate path must actually aggregate, otherwise the identity
    // above would be trivially true.
    BOOST_CHECK_LT(b.stats.aggregate_flushes, a.stats.aggregate_flushes);

    std::error_code ec;
    std::filesystem::remove_all(dir_per_frame, ec);
    std::filesystem::remove_all(dir_aggregate, ec);
}

// ---------------------------------------------------------------------------
// 3. Failed frames: jsonl only, no ucr4 record, no offset advance
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_failed_frames_do_not_advance_offsets)
{
    std::vector<FrameSpec> frames;

    FrameSpec f0;
    f0.window_id = 0;
    f0.tap_count = 116;
    frames.push_back(f0);

    FrameSpec f1; // sfd_failed: taps present but the frame is not ok
    f1.window_id = 1;
    f1.tap_count = 116;
    f1.radar_sfd_found = false;
    frames.push_back(f1);

    FrameSpec f2;
    f2.window_id = 2;
    f2.tap_count = 100;
    frames.push_back(f2);

    FrameSpec f3; // timing_failed
    f3.window_id = 3;
    f3.tap_count = 116;
    f3.radar_detected_origin = -1;
    frames.push_back(f3);

    FrameSpec f4; // cir_failed: no taps at all
    f4.window_id = 4;
    f4.tap_count = 0;
    frames.push_back(f4);

    FrameSpec f5;
    f5.window_id = 5;
    f5.tap_count = 116;
    frames.push_back(f5);

    const std::string dir = make_temp_dir("failures");
    const RunResult r = run_frames(dir, frames);
    BOOST_CHECK_EQUAL(r.stats.written, 6u);
    BOOST_CHECK_EQUAL(r.stats.cir_written, 3u);
    BOOST_CHECK_EQUAL(r.stats.cir_failed, 3u);

    const std::vector<char> ucr4 = read_file(dir + "/cir.ucr4");
    const size_t expected = pl::ucr4_record_bytes(116) +
                            pl::ucr4_record_bytes(100) +
                            pl::ucr4_record_bytes(116);
    BOOST_CHECK_EQUAL(ucr4.size(), expected);

    const std::vector<std::string> lines = read_lines(dir + "/cir.jsonl");
    BOOST_REQUIRE_EQUAL(lines.size(), 6u);

    const char* kStatus[6] = { "ok", "sfd_failed", "ok",
                               "timing_failed", "cir_failed", "ok" };
    const uint64_t kTapCount[6] = { 116, 0, 100, 0, 0, 116 };
    const uint64_t kOffset[6] = { 0, 116, 116, 216, 216, 216 };

    for (size_t i = 0; i < 6; ++i) {
        BOOST_CHECK_EQUAL(raw_u64(lines[i], "pulse_id"), i);
        BOOST_CHECK_EQUAL(raw_str(lines[i], "status"), std::string(kStatus[i]));
        BOOST_CHECK_EQUAL(raw_u64(lines[i], "tap_count"), kTapCount[i]);
        BOOST_CHECK_EQUAL(raw_u64(lines[i], "file_offset_taps"), kOffset[i]);
        BOOST_CHECK_EQUAL(raw_u64(lines[i], "zero_delay_tap"), 16u);
        BOOST_CHECK_EQUAL(raw_u64(lines[i], "cir_pre_samples"), 16u);
        BOOST_CHECK_EQUAL(raw_u64(lines[i], "cir_post_samples"), 100u);
        BOOST_CHECK_EQUAL(raw_u64(lines[i], "estimator_us"), 42u);
        BOOST_CHECK_EQUAL(raw_u64(lines[i], "valid_repetitions"), 8u);
        BOOST_CHECK_EQUAL(raw_u64(lines[i], "schedule_index"), i + 7);
        BOOST_CHECK_EQUAL(raw_bool(lines[i], "sfd_ok"),
                          std::string(i == 1 ? "false" : "true"));
        BOOST_CHECK_EQUAL(raw_bool(lines[i], "timing_ok"),
                          std::string(i == 3 ? "false" : "true"));
        BOOST_CHECK_EQUAL(raw_bool(lines[i], "sic_committed"),
                          std::string("true"));
    }

    // The three ok frames are the ones physically in the file.
    size_t off = 0;
    for (size_t i = 0; i < 6; ++i) {
        if (kTapCount[i] == 0)
            continue;
        pl::Ucr4Header h;
        std::memcpy(&h, ucr4.data() + off, sizeof(h));
        BOOST_CHECK_EQUAL(std::string(h.magic, 4), std::string("UCR4"));
        BOOST_CHECK_EQUAL(h.pulse_id, static_cast<uint32_t>(i));
        BOOST_CHECK_EQUAL(h.status, 0);
        BOOST_CHECK_EQUAL(h.tap_count, kTapCount[i]);
        BOOST_CHECK_EQUAL(h.repetition_index, 0xFFFF);
        BOOST_CHECK_EQUAL(h.repetition_count, 8);
        BOOST_CHECK(h.cir_scale > 0.0f);
        off += pl::ucr4_record_bytes(kTapCount[i]);
    }
    BOOST_CHECK_EQUAL(off, expected);

    std::error_code ec;
    std::filesystem::remove_all(dir, ec);
}

// ---------------------------------------------------------------------------
// 4. demod_results.jsonl round-trips through the shared parser
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_demod_results_parse_round_trip)
{
    std::vector<FrameSpec> frames;
    for (uint64_t i = 0; i < 4; ++i) {
        FrameSpec f;
        f.window_id = i;
        f.tap_count = 116;
        f.comm_attempted = true;
        f.comm_ok = (i % 2) == 0;
        f.comm_fcs_ok = f.comm_ok;
        f.comm_status_reason = f.comm_ok ? "ok" : "payload_failed";
        f.psdu_n = f.comm_ok ? 32 : 0;
        frames.push_back(f);
    }

    const std::string dir = make_temp_dir("demod");
    const RunResult r = run_frames(dir, frames);
    BOOST_CHECK_EQUAL(r.stats.comm_written, 4u);
    BOOST_CHECK_EQUAL(r.stats.comm_skipped, 0u);

    const std::string path = dir + "/demod_results.jsonl";
    const std::vector<core::DemodResultMeta> parsed =
        core::load_demod_jsonl(path);
    BOOST_REQUIRE_EQUAL(parsed.size(), 4u);

    for (size_t i = 0; i < parsed.size(); ++i) {
        const bool ok = (i % 2) == 0;
        BOOST_CHECK_EQUAL(parsed[i].packet_id, static_cast<int64_t>(i));
        BOOST_CHECK_EQUAL(parsed[i].schedule_index,
                          static_cast<int64_t>(i + 7));
        BOOST_CHECK_EQUAL(parsed[i].status,
                          std::string(ok ? "ok" : "payload_failed"));
        BOOST_CHECK_EQUAL(parsed[i].fcs_pass, ok);
        BOOST_CHECK_EQUAL(parsed[i].detected_start_sample, 1234);
        BOOST_CHECK(parsed[i].has_detected_start);
        BOOST_CHECK_EQUAL(parsed[i].predicted_start_sample, 1000);
        BOOST_CHECK_EQUAL(parsed[i].native_window_start,
                          static_cast<int64_t>(i) * 4000);
        BOOST_CHECK_CLOSE(parsed[i].resample_us, 3.5, 1e-6);
        BOOST_CHECK_CLOSE(parsed[i].t_total_us, 12.5, 1e-6);
    }

    // psdu_hex must describe exactly the bytes whose FNV-1a is psdu_fnv1a.
    const std::vector<std::string> lines = read_lines(path);
    BOOST_REQUIRE_EQUAL(lines.size(), 4u);
    for (size_t i = 0; i < lines.size(); ++i) {
        const std::string hex = raw_str(lines[i], "psdu_hex");
        const std::vector<uint8_t> bytes = from_hex(hex);
        const uint64_t fnv = raw_u64(lines[i], "psdu_fnv1a");
        const bool ok = (i % 2) == 0;
        if (ok) {
            BOOST_REQUIRE_EQUAL(bytes.size(), 32u);
            BOOST_CHECK_EQUAL(hex, to_hex(bytes));
            BOOST_CHECK_EQUAL(fnv, fnv1a64(bytes.data(), bytes.size()));
            BOOST_CHECK_EQUAL(raw_u64(lines[i], "payload_nbytes"), 32u);
            BOOST_CHECK_EQUAL(raw_u64(lines[i], "phr_psdu_length"), 32u);
            BOOST_CHECK_EQUAL(raw_bool(lines[i], "phr_ok"), std::string("true"));
            BOOST_CHECK_EQUAL(raw_u64(lines[i], "fcs_received"), 0xABCDu);
            BOOST_CHECK_EQUAL(raw_u64(lines[i], "fcs_calculated"), 0xABCDu);
        } else {
            BOOST_CHECK(hex.empty());
            BOOST_CHECK(bytes.empty());
            BOOST_CHECK_EQUAL(raw_u64(lines[i], "payload_nbytes"), 0u);
            BOOST_CHECK_EQUAL(raw_bool(lines[i], "fcs_pass"),
                              std::string("false"));
        }
        BOOST_CHECK_CLOSE(raw_f64(lines[i], "sic_suppression_db"), 28.25, 1e-6);
        BOOST_CHECK_EQUAL(raw_str(lines[i], "sic_status"),
                          std::string("committed"));
        BOOST_CHECK_EQUAL(raw_bool(lines[i], "sic_executed"),
                          std::string("true"));
    }

    std::error_code ec;
    std::filesystem::remove_all(dir, ec);
}

// ---------------------------------------------------------------------------
// 5. Slow-writer ledger: never block, never deadlock, never lose a frame
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_slow_writer_ledger)
{
    std::function<void()>& hook = gr::uwb::pipeline::test_slow_hook_storage();
    hook = [] { std::this_thread::sleep_for(std::chrono::milliseconds(1)); };

    const std::string dir = make_temp_dir("slow");
    pl::OutputPersistenceConfig cfg;
    cfg.output_dir = dir;
    cfg.writer_queue_capacity = 4;
    cfg.writer_aggregate_bytes = 0;
    cfg.writer_aggregate_window_ms = 0;
    cfg.live_log_mode = pl::LiveLogMode::Off;

    pl::PipelineOutputWriter writer;
    BOOST_REQUIRE(writer.start(cfg, make_meta(127), 116, 127));

    constexpr size_t kFrames = 200;
    std::chrono::steady_clock::duration worst{ 0 };
    const auto t0 = std::chrono::steady_clock::now();
    for (size_t i = 0; i < kFrames; ++i) {
        const auto a = std::chrono::steady_clock::now();
        pl::OutputPayloadSlot* slot = writer.try_acquire_slot();
        if (slot != nullptr) {
            FrameSpec f;
            f.window_id = i;
            f.tap_count = 116;
            fill_slot(*slot, f);
            writer.try_submit(slot);
        }
        const auto b = std::chrono::steady_clock::now();
        worst = std::max(worst, b - a);
    }
    const auto submit_span = std::chrono::steady_clock::now() - t0;

    writer.stop_and_drain();
    hook = nullptr;

    const pl::OutputWriterStats s = writer.stats();
    BOOST_TEST_MESSAGE("slow writer: offered="
                       << s.offered << " written=" << s.written
                       << " dropped=" << s.dropped
                       << " queue_high_watermark=" << s.queue_high_watermark
                       << " slots=" << s.slots_allocated
                       << " submit_span_us="
                       << std::chrono::duration_cast<std::chrono::microseconds>(
                              submit_span)
                              .count()
                       << " worst_submit_us="
                       << std::chrono::duration_cast<std::chrono::microseconds>(
                              worst)
                              .count());

    BOOST_CHECK_EQUAL(s.offered, static_cast<uint64_t>(kFrames));
    BOOST_CHECK_EQUAL(s.offered, s.written + s.dropped);
    BOOST_CHECK_GT(s.dropped, 0u);
    BOOST_CHECK_GT(s.written, 0u);
    BOOST_CHECK_LE(s.queue_high_watermark, static_cast<uint64_t>(4));
    BOOST_CHECK_EQUAL(s.slots_allocated, static_cast<uint64_t>(64));

    // The submitting thread must never be slowed down by the writer thread.
    const auto worst_us =
        std::chrono::duration_cast<std::chrono::microseconds>(worst).count();
    BOOST_CHECK_MESSAGE(worst_us < 20000,
                        "a single try_acquire/try_submit blocked for "
                            << worst_us << " us");

    // Everything that was written is on disk, and the jsonl agrees with the
    // ucr4 record count.
    const std::vector<std::string> lines = read_lines(dir + "/cir.jsonl");
    BOOST_CHECK_EQUAL(lines.size(), s.written);
    const std::vector<char> ucr4 = read_file(dir + "/cir.ucr4");
    BOOST_CHECK_EQUAL(ucr4.size(), s.cir_written * pl::ucr4_record_bytes(116));

    std::error_code ec;
    std::filesystem::remove_all(dir, ec);
}

// ---------------------------------------------------------------------------
// 6. Disabled writer is inert
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_disabled_writer_is_inert)
{
    pl::PipelineOutputWriter writer;
    pl::OutputPersistenceConfig cfg; // empty output_dir, LiveLogMode::Off
    pl::OutputRunMeta meta;

    BOOST_CHECK(!cfg.enabled());
    BOOST_CHECK(!writer.start(cfg, meta, 116, 127));
    BOOST_CHECK(!writer.enabled());
    BOOST_CHECK(writer.directory().empty());
    BOOST_CHECK(writer.try_acquire_slot() == nullptr);
    BOOST_CHECK(!writer.try_submit(nullptr));

    const pl::OutputWriterStats s = writer.stats();
    BOOST_CHECK(!s.enabled);
    BOOST_CHECK_EQUAL(s.written, 0u);
    BOOST_CHECK_EQUAL(s.bytes_written, 0u);
    BOOST_CHECK_EQUAL(s.offered, s.written + s.dropped);

    writer.stop_and_drain();
    writer.stop_and_drain(); // idempotent
    BOOST_CHECK(!writer.enabled());

    // A queue capacity of 0 is rejected too (never a silent unbounded queue).
    pl::OutputPersistenceConfig bad;
    bad.output_dir = make_temp_dir("zero_queue");
    bad.writer_queue_capacity = 0;
    pl::PipelineOutputWriter w2;
    BOOST_CHECK(!w2.start(bad, meta, 116, 127));
    BOOST_CHECK(!w2.enabled());
    std::error_code ec;
    std::filesystem::remove_all(bad.output_dir, ec);
}

// ---------------------------------------------------------------------------
// 7. comm_write_only_ok filters failed frames into comm_skipped
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_comm_write_only_ok_filters_failures)
{
    std::vector<FrameSpec> frames;
    for (uint64_t i = 0; i < 4; ++i) {
        FrameSpec f;
        f.window_id = i;
        f.tap_count = 116;
        f.comm_attempted = true;
        f.comm_ok = (i % 2) == 0;
        f.comm_fcs_ok = f.comm_ok;
        f.comm_status_reason = f.comm_ok ? "ok" : "fcs_failed";
        f.psdu_n = f.comm_ok ? 16 : 0;
        frames.push_back(f);
    }

    const std::string dir = make_temp_dir("only_ok");
    const RunResult r = run_frames(dir, frames, /*agg_bytes=*/1u << 20,
                                   /*agg_ms=*/50, /*queue_capacity=*/256,
                                   /*comm_only_ok=*/true);
    BOOST_CHECK_EQUAL(r.stats.comm_written, 2u);
    BOOST_CHECK_EQUAL(r.stats.comm_skipped, 2u);
    BOOST_CHECK_EQUAL(r.stats.offered, r.stats.written + r.stats.dropped);

    const std::vector<std::string> lines =
        read_lines(dir + "/demod_results.jsonl");
    BOOST_REQUIRE_EQUAL(lines.size(), 2u);
    BOOST_CHECK_EQUAL(raw_u64(lines[0], "packet_id"), 0u);
    BOOST_CHECK_EQUAL(raw_u64(lines[1], "packet_id"), 2u);
    for (const std::string& line : lines) {
        BOOST_CHECK_EQUAL(raw_bool(line, "fcs_pass"), std::string("true"));
        BOOST_CHECK_EQUAL(raw_str(line, "status"), std::string("ok"));
    }

    // run.json must describe the on-disk format exactly.
    const std::vector<std::string> run = read_lines(dir + "/run.json");
    BOOST_REQUIRE_EQUAL(run.size(), 1u);
    BOOST_CHECK_EQUAL(raw_str(run[0], "tap_format"), std::string("ucr4"));
    BOOST_CHECK_EQUAL(raw_str(run[0], "byte_order"),
                      std::string("little-endian"));
    BOOST_CHECK_EQUAL(raw_u64(run[0], "header_bytes"), 52u);
    BOOST_CHECK_EQUAL(raw_u64(run[0], "bytes_per_tap"), 4u);
    BOOST_CHECK_EQUAL(raw_str(run[0], "reconstruction"),
                      std::string("fc32 = sc16 * cir_scale"));
    BOOST_CHECK_EQUAL(raw_str(run[0], "consumer_mode"), std::string("both"));
    BOOST_CHECK_EQUAL(raw_u64(run[0], "code_index"), 9u);
    BOOST_CHECK_EQUAL(raw_u64(run[0], "sync_repetitions"), 64u);
    BOOST_CHECK_EQUAL(raw_u64(run[0], "max_psdu_bytes"), 127u);
    const double range = std::stod(raw_value(run[0], "range_m_per_tap"));
    BOOST_CHECK_CLOSE(range, 299792458.0 / (2.0 * 998.4e6), 1e-6);

    std::error_code ec;
    std::filesystem::remove_all(dir, ec);
}

// ---------------------------------------------------------------------------
// 8. Summary mode must tick even when the queue is idle
// ---------------------------------------------------------------------------
// Plan §3.3: the writer thread uses cv_.wait_for() so a quiet pipeline still
// produces a dashboard line.  A plain cv_.wait() would hang here forever.
BOOST_AUTO_TEST_CASE(test_summary_mode_wakes_when_idle)
{
    const std::string dir = make_temp_dir("summary");
    pl::OutputPersistenceConfig cfg;
    cfg.output_dir = dir;
    cfg.writer_queue_capacity = 8;
    cfg.writer_aggregate_bytes = 0;
    cfg.writer_aggregate_window_ms = 0;
    cfg.live_log_mode = pl::LiveLogMode::Summary;
    cfg.live_log_interval_ms = 25;

    pl::PipelineOutputWriter writer;
    BOOST_REQUIRE(writer.start(cfg, make_meta(127), 116, 127));
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    const pl::OutputWriterStats idle = writer.stats();
    BOOST_CHECK_GT(idle.live_log_lines, 0u);
    BOOST_CHECK_EQUAL(idle.written, 0u);
    BOOST_CHECK_EQUAL(idle.offered, 0u);

    writer.stop_and_drain();
    BOOST_CHECK_EQUAL(writer.stats().offered,
                      writer.stats().written + writer.stats().dropped);

    std::error_code ec;
    std::filesystem::remove_all(dir, ec);
}

// ---------------------------------------------------------------------------
// 9. release_slot() converts an acquired slot into a drop
// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
// 10. peak_tap / sfd_start_sample are published VERBATIM
//
//   The CIR kernel owns both values: `peak_tap` is its argmax of |raw tap| (L2)
//   and `sfd_start_sample` is the RX-detected SFD start, which is a different
//   quantity from the detected SYNC origin.  The writer must copy them, not
//   re-derive them -- an earlier revision recomputed peak_tap with max(|re|,|im|)
//   and published the SYNC origin as the SFD start.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_peak_tap_and_sfd_start_are_verbatim)
{
    const std::string dir = make_temp_dir("verbatim");
    FrameSpec f;
    f.window_id = 0;
    f.tap_count = 116;
    f.peak_tap = 73;          // deliberately NOT the max(|re|,|im|) argmax
    f.sfd_start_sample = 4321;
    f.radar_detected_origin = 1000; // must NOT leak into sfd_start_sample
    f.radar_detected_start = 1016;

    const RunResult r = run_frames(dir, { f });
    BOOST_REQUIRE_EQUAL(r.stats.written, 1u);

    const std::vector<std::string> lines = read_lines(dir + "/cir.jsonl");
    BOOST_REQUIRE_EQUAL(lines.size(), 1u);
    const std::string& line = lines[0];
    BOOST_CHECK_EQUAL(raw_u64(line, "peak_tap"), static_cast<uint64_t>(73));
    BOOST_CHECK_EQUAL(static_cast<int64_t>(raw_u64(line, "sfd_start_sample")),
                      static_cast<int64_t>(4321));
    BOOST_CHECK_EQUAL(
        static_cast<int64_t>(raw_u64(line, "preamble_start_sample")),
        static_cast<int64_t>(1016));
    // cir_peak_metric is the record's value, not the magnitude at peak_tap.
    BOOST_CHECK_CLOSE(raw_f64(line, "cir_peak_metric"), 0.078909032, 1e-6);
    // peak_delay_from_calibration_ns uses (peak_tap - zero_delay_tap).
    const double expect_ns = (73.0 - 16.0) / 998.4e6 * 1.0e9;
    BOOST_CHECK_CLOSE(raw_f64(line, "peak_delay_from_calibration_ns"), expect_ns,
                      1e-6);

    // The UCR4 header carries the same peak_tap.
    const std::vector<char> bin = read_file(dir + "/cir.ucr4");
    BOOST_REQUIRE_GE(bin.size(), sizeof(pl::Ucr4Header));
    pl::Ucr4Header hdr;
    std::memcpy(&hdr, bin.data(), sizeof(hdr));
    BOOST_CHECK_EQUAL(hdr.peak_tap, static_cast<int32_t>(73));
    BOOST_CHECK_EQUAL(hdr.tap_count, static_cast<uint16_t>(116));

    std::error_code ec;
    std::filesystem::remove_all(dir, ec);
}

BOOST_AUTO_TEST_CASE(test_release_slot_balances_ledger)
{
    const std::string dir = make_temp_dir("release");
    pl::OutputPersistenceConfig cfg;
    cfg.output_dir = dir;
    cfg.writer_queue_capacity = 8;
    cfg.writer_aggregate_bytes = 0;
    cfg.live_log_mode = pl::LiveLogMode::Off;

    pl::PipelineOutputWriter writer;
    BOOST_REQUIRE(writer.start(cfg, make_meta(127), 116, 127));

    pl::OutputPayloadSlot* abandoned = writer.try_acquire_slot();
    BOOST_REQUIRE(abandoned != nullptr);
    writer.release_slot(abandoned);

    // The released slot must be reusable.
    pl::OutputPayloadSlot* reused = writer.try_acquire_slot();
    BOOST_REQUIRE(reused != nullptr);
    FrameSpec f;
    f.window_id = 1;
    f.tap_count = 116;
    fill_slot(*reused, f);
    BOOST_REQUIRE(writer.try_submit(reused));

    writer.stop_and_drain();
    const pl::OutputWriterStats s = writer.stats();
    BOOST_CHECK_EQUAL(s.offered, 2u);
    BOOST_CHECK_EQUAL(s.written, 1u);
    BOOST_CHECK_EQUAL(s.dropped, 1u);
    BOOST_CHECK_EQUAL(s.offered, s.written + s.dropped);
    BOOST_CHECK_EQUAL(s.cir_written, 1u);

    std::error_code ec;
    std::filesystem::remove_all(dir, ec);
}
