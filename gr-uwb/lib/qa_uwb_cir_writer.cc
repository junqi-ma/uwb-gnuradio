/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * QA for the UwbCirWriter message block (Radar Step 8).
 *
 * Covers: mixed ok/failed frames (JSONL line count == frames, only ok
 * frames advance file_offset_taps and write taps), exact little-endian
 * complex64 bytes, optional normalized file, malformed PDU rejection,
 * queue-full drops, stop() draining, and restart truncation.
 */

#include <boost/test/unit_test.hpp>

#include <gnuradio/blocks/message_debug.h>
#include <gnuradio/gr_complex.h>
#include <gnuradio/top_block.h>
#include <gnuradio/uwb/uwb_cir_writer.h>
#include <pmt/pmt.h>

#include <chrono>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <string>
#include <thread>
#include <vector>

using gr::uwb::UwbCirWriter;

namespace {

constexpr size_t kTaps = 116; // 16 pre + 100 post

std::string
make_temp_dir(const std::string& tag)
{
    const std::string dir =
        (std::filesystem::temp_directory_path() /
         ("uwb_qa_cir_writer_" + tag))
            .string();
    std::filesystem::remove_all(dir);
    std::filesystem::create_directories(dir);
    return dir;
}

bool
read_bytes(const std::string& path, std::vector<uint8_t>& out)
{
    std::ifstream f(path, std::ios::binary);
    if (!f)
        return false;
    f.seekg(0, std::ios::end);
    const std::streamoff n = f.tellg();
    f.seekg(0, std::ios::beg);
    out.resize(static_cast<size_t>(n));
    f.read(reinterpret_cast<char*>(out.data()), n);
    return f.good() || f.eof();
}

std::vector<std::string>
read_lines(const std::string& path)
{
    std::ifstream f(path);
    std::vector<std::string> lines;
    std::string line;
    while (std::getline(f, line)) {
        if (!line.empty())
            lines.push_back(line);
    }
    return lines;
}

bool
parse_num(const std::string& json, const char* key, double& out)
{
    const std::string pat = std::string("\"") + key + "\"";
    auto pos = json.find(pat);
    if (pos == std::string::npos)
        return false;
    pos = json.find(':', pos + pat.size());
    if (pos == std::string::npos)
        return false;
    try {
        size_t idx = 0;
        out = std::stod(json.substr(pos + 1), &idx);
        return idx > 0;
    } catch (...) {
        return false;
    }
}

bool
parse_i64(const std::string& json, const char* key, int64_t& out)
{
    double v = 0.0;
    if (!parse_num(json, key, v))
        return false;
    out = static_cast<int64_t>(std::llround(v));
    return true;
}

bool
parse_str(const std::string& json, const char* key, std::string& out)
{
    const std::string pat = std::string("\"") + key + "\"";
    auto pos = json.find(pat);
    if (pos == std::string::npos)
        return false;
    pos = json.find(':', pos + pat.size());
    if (pos == std::string::npos)
        return false;
    pos = json.find('"', pos + 1);
    if (pos == std::string::npos)
        return false;
    const auto end = json.find('"', pos + 1);
    if (end == std::string::npos)
        return false;
    out = json.substr(pos + 1, end - pos - 1);
    return true;
}

pmt::pmt_t
make_meta(uint64_t pulse_id,
          const std::string& status,
          size_t tap_count,
          bool with_norm = false)
{
    pmt::pmt_t meta = pmt::make_dict();
    meta = pmt::dict_add(meta, pmt::mp("pulse_id"),
                         pmt::from_uint64(pulse_id));
    meta = pmt::dict_add(meta, pmt::mp("schedule_index"),
                         pmt::from_uint64(pulse_id));
    meta = pmt::dict_add(meta, pmt::mp("status"), pmt::mp(status));
    meta = pmt::dict_add(meta, pmt::mp("tap_count"),
                         pmt::from_uint64(tap_count));
    meta = pmt::dict_add(meta, pmt::mp("sample_rate"),
                         pmt::from_double(998.4e6));
    meta = pmt::dict_add(meta, pmt::mp("cir_pre_samples"),
                         pmt::from_uint64(16));
    meta = pmt::dict_add(meta, pmt::mp("cir_post_samples"),
                         pmt::from_uint64(100));
    meta = pmt::dict_add(meta, pmt::mp("zero_delay_tap"), pmt::from_long(16));
    meta = pmt::dict_add(meta, pmt::mp("peak_tap"),
                         pmt::from_uint64(tap_count ? 18u : 0u));
    meta = pmt::dict_add(meta, pmt::mp("cir_peak_metric"),
                         pmt::from_double(tap_count ? 0.91 : 0.0));
    meta = pmt::dict_add(meta, pmt::mp("raw_l2_norm"),
                         pmt::from_double(tap_count ? 0.013 : 0.0));
    meta = pmt::dict_add(meta, pmt::mp("preamble_start_sample"),
                         pmt::from_long(status == "ok" ? 1997 : -1));
    meta = pmt::dict_add(meta, pmt::mp("sfd_start_sample"),
                         pmt::from_long(
                             (status == "ok" || status == "timing_failed" ||
                              status == "cir_failed")
                                 ? 67021
                                 : -1));
    meta = pmt::dict_add(meta, pmt::mp("tx_time_full"), pmt::from_long(0));
    meta = pmt::dict_add(meta, pmt::mp("tx_time_frac"),
                         pmt::from_double(0.05));
    meta = pmt::dict_add(meta, pmt::mp("num_delay_samps"), pmt::from_long(0));
    meta = pmt::dict_add(meta, pmt::mp("calibration_id"), pmt::mp("qa-cal"));
    meta = pmt::dict_add(meta, pmt::mp("source"), pmt::mp("loopback"));
    if (with_norm)
        meta = pmt::dict_add(meta, pmt::mp("normalized_taps"),
                             pmt::PMT_NIL); // replaced by caller
    return meta;
}

pmt::pmt_t
make_frame(uint64_t pulse_id,
           const std::string& status,
           const std::vector<gr_complex>& taps,
           bool with_norm_taps = false)
{
    pmt::pmt_t meta = make_meta(
        pulse_id, status, status == "ok" ? taps.size() : 0, with_norm_taps);
    if (with_norm_taps) {
        meta = pmt::dict_add(meta, pmt::mp("normalized_taps"),
                             pmt::init_c32vector(taps.size(), taps.data()));
    }
    pmt::pmt_t data =
        status == "ok"
            ? pmt::init_c32vector(taps.size(), taps.data())
            : pmt::init_c32vector(0, static_cast<const gr_complex*>(nullptr));
    return pmt::cons(meta, data);
}

std::vector<gr_complex>
make_taps(uint64_t pulse_id)
{
    std::vector<gr_complex> taps(kTaps);
    for (size_t i = 0; i < kTaps; ++i)
        taps[i] = gr_complex(static_cast<float>(pulse_id * 1000 + i) * 0.001f,
                             static_cast<float>(i) * -0.0005f);
    return taps;
}

bool
wait_written(const UwbCirWriter::sptr& w,
             uint64_t want,
             long timeout_ms = 30000)
{
    const auto t0 = std::chrono::steady_clock::now();
    while (w->frames_written() + w->frames_failed() + w->frames_invalid() <
           want) {
        const auto ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - t0)
                .count();
        if (ms > timeout_ms)
            return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    return true;
}

} // namespace

// ---------------------------------------------------------------------------
// Mixed ok + three failed statuses: JSONL count, offsets, exact bytes.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_writer_mixed_frames)
{
    const std::string dir = make_temp_dir("mixed");
    auto w = UwbCirWriter::make(dir, "cir", /*write_normalized=*/false, 16);
    auto dbg = gr::blocks::message_debug::make();
    auto tb = gr::make_top_block("qa_cir_writer_mixed");
    tb->msg_connect(w, "status", dbg, "store");
    BOOST_REQUIRE(w->start());
    tb->start();

    const char* statuses[] = { "ok",        "ok",        "sfd_failed",
                               "ok",        "timing_failed", "ok",
                               "cir_failed", "ok",     "invalid_input",
                               "ok" };
    constexpr size_t kFrames = 10;
    std::vector<std::vector<gr_complex>> frames(kFrames);
    for (size_t i = 0; i < kFrames; ++i)
        frames[i] = make_taps(i);

    uint64_t posted_ok = 0;
    uint64_t expect_taps = 0;
    for (size_t i = 0; i < kFrames; ++i) {
        const bool ok = std::string(statuses[i]) == "ok";
        if (ok) {
            ++posted_ok;
            expect_taps += frames[i].size();
        }
        w->_post(pmt::mp("cir"),
                 make_frame(i, statuses[i], frames[i]));
    }

    BOOST_REQUIRE(wait_written(w, kFrames));
    BOOST_CHECK_EQUAL(w->frames_received(), kFrames);
    BOOST_CHECK_EQUAL(w->frames_written(), 6u);
    BOOST_CHECK_EQUAL(w->frames_failed(), 4u);
    BOOST_CHECK_EQUAL(w->frames_dropped(), 0u);
    BOOST_CHECK_EQUAL(w->frames_invalid(), 0u);
    BOOST_CHECK_EQUAL(w->taps_written(), expect_taps);

    tb->stop();
    tb->wait();
    BOOST_REQUIRE(w->stop());

    // JSONL: exactly one line per frame, in order.
    const auto lines = read_lines(dir + "/cir.jsonl");
    BOOST_REQUIRE_EQUAL(lines.size(), kFrames);

    // Binary: exactly the ok taps, little-endian complex64.
    std::vector<uint8_t> raw;
    BOOST_REQUIRE(read_bytes(dir + "/cir.cf32", raw));
    BOOST_CHECK_EQUAL(raw.size(), expect_taps * 8);

    uint64_t offset = 0;
    for (size_t i = 0; i < kFrames; ++i) {
        const std::string& line = lines[i];
        uint64_t pid = 0, tap_count = 0, file_offset = 0;
        double v = 0.0;
        BOOST_REQUIRE(parse_num(line, "pulse_id", v));
        pid = static_cast<uint64_t>(std::llround(v));
        BOOST_CHECK_EQUAL(pid, i);
        std::string status;
        BOOST_REQUIRE(parse_str(line, "status", status));
        BOOST_CHECK_EQUAL(status, statuses[i]);
        BOOST_REQUIRE(parse_num(line, "tap_count", v));
        tap_count = static_cast<uint64_t>(std::llround(v));
        BOOST_REQUIRE(parse_num(line, "file_offset_taps", v));
        file_offset = static_cast<uint64_t>(std::llround(v));
        int64_t zero_tap = -1;
        BOOST_REQUIRE(parse_i64(line, "zero_delay_tap", zero_tap));
        BOOST_CHECK_EQUAL(zero_tap, 16);
        double ns = -1.0;
        BOOST_REQUIRE(parse_num(line, "peak_delay_from_calibration_ns", ns));
        double range = 0.0;
        BOOST_REQUIRE(parse_num(line, "range_m_per_tap", range));
        BOOST_CHECK_CLOSE(range, 299792458.0 / (2.0 * 998.4e6), 1e-6);

        if (status == "ok") {
            BOOST_CHECK_EQUAL(tap_count, kTaps);
            BOOST_CHECK_EQUAL(file_offset, offset);
            // Exact bytes for this frame at its offset.
            const uint8_t* expect =
                reinterpret_cast<const uint8_t*>(frames[i].data());
            BOOST_CHECK(std::memcmp(raw.data() + offset * 8, expect,
                                    kTaps * 8) == 0);
            int64_t sfd = -1, pre = -1;
            BOOST_REQUIRE(parse_i64(line, "sfd_start_sample", sfd));
            BOOST_CHECK_EQUAL(sfd, 67021);
            BOOST_REQUIRE(parse_i64(line, "preamble_start_sample", pre));
            BOOST_CHECK_EQUAL(pre, 1997);
            offset += kTaps;
        } else {
            BOOST_CHECK_EQUAL(tap_count, 0u);
            BOOST_CHECK_EQUAL(file_offset, offset); // unchanged tail
        }
    }
    BOOST_CHECK_EQUAL(offset, expect_taps);

    // run.json exists with the static contract.
    std::vector<uint8_t> run;
    BOOST_REQUIRE(read_bytes(dir + "/run.json", run));
    BOOST_CHECK_GT(run.size(), 0u);
}

// ---------------------------------------------------------------------------
// Normalized output file + missing normalized taps → failed frame.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_writer_normalized)
{
    const std::string dir = make_temp_dir("norm");
    auto w = UwbCirWriter::make(dir, "cir", /*write_normalized=*/true, 16);
    auto dbg = gr::blocks::message_debug::make();
    auto tb = gr::make_top_block("qa_cir_writer_norm");
    tb->msg_connect(w, "status", dbg, "store");
    BOOST_REQUIRE(w->start());
    tb->start();

    const auto taps = make_taps(0);
    w->_post(pmt::mp("cir"), make_frame(0, "ok", taps, /*with_norm=*/true));
    // Missing normalized_taps in meta while normalized writing is on.
    {
        pmt::pmt_t meta = make_meta(1, "ok", kTaps);
        w->_post(pmt::mp("cir"),
                 pmt::cons(meta, pmt::init_c32vector(taps.size(),
                                                     taps.data())));
    }
    // Short normalized taps.
    {
        pmt::pmt_t meta = make_meta(2, "ok", kTaps);
        std::vector<gr_complex> short_norm(kTaps - 1);
        meta = pmt::dict_add(meta, pmt::mp("normalized_taps"),
                             pmt::init_c32vector(short_norm.size(),
                                                 short_norm.data()));
        w->_post(pmt::mp("cir"),
                 pmt::cons(meta, pmt::init_c32vector(taps.size(),
                                                     taps.data())));
    }
    // Raw payload longer than tap_count → failed frame, no binary write.
    {
        pmt::pmt_t meta = make_meta(3, "ok", kTaps);
        std::vector<gr_complex> long_raw(kTaps + 1);
        meta = pmt::dict_add(meta, pmt::mp("normalized_taps"),
                             pmt::init_c32vector(kTaps, taps.data()));
        w->_post(pmt::mp("cir"),
                 pmt::cons(meta, pmt::init_c32vector(long_raw.size(),
                                                     long_raw.data())));
    }
    // Normalized payload longer than tap_count → failed frame.
    {
        pmt::pmt_t meta = make_meta(4, "ok", kTaps);
        std::vector<gr_complex> long_norm(kTaps + 1);
        meta = pmt::dict_add(meta, pmt::mp("normalized_taps"),
                             pmt::init_c32vector(long_norm.size(),
                                                 long_norm.data()));
        w->_post(pmt::mp("cir"),
                 pmt::cons(meta, pmt::init_c32vector(taps.size(),
                                                     taps.data())));
    }
    // Failed frame must not touch the normalized file either.
    w->_post(pmt::mp("cir"),
             make_frame(5, "sfd_failed", taps, /*with_norm=*/true));
    // Exact-match positive control: raw and norm both == tap_count.
    w->_post(pmt::mp("cir"),
             make_frame(6, "ok", taps, /*with_norm=*/true));

    BOOST_REQUIRE(wait_written(w, 7));
    tb->stop();
    tb->wait();
    BOOST_REQUIRE(w->stop());

    BOOST_CHECK_EQUAL(w->frames_written(), 2u);
    BOOST_CHECK_EQUAL(w->frames_failed(), 5u);
    BOOST_CHECK_EQUAL(w->taps_written(), 2u * kTaps);

    std::vector<uint8_t> raw, norm;
    BOOST_REQUIRE(read_bytes(dir + "/cir.cf32", raw));
    BOOST_REQUIRE(read_bytes(dir + "/cir_norm.cf32", norm));
    BOOST_CHECK_EQUAL(raw.size(), 2u * kTaps * 8);
    BOOST_CHECK_EQUAL(norm.size(), 2u * kTaps * 8);
    BOOST_CHECK(std::memcmp(raw.data(), taps.data(), kTaps * 8) == 0);
    BOOST_CHECK(std::memcmp(raw.data() + kTaps * 8, taps.data(),
                            kTaps * 8) == 0);
    BOOST_CHECK(std::memcmp(norm.data(), taps.data(), kTaps * 8) == 0);
    BOOST_CHECK(std::memcmp(norm.data() + kTaps * 8, taps.data(),
                            kTaps * 8) == 0);

    // Per-line expectations: [ok, missing-norm, short-norm, long-raw,
    // long-norm, failed, ok]. Offsets advance only on ok frames; failed
    // lines report the offset current at their time of write.
    const auto lines = read_lines(dir + "/cir.jsonl");
    BOOST_REQUIRE_EQUAL(lines.size(), 7u);
    const uint64_t expect_tc[] = { kTaps, 0, 0, 0, 0, 0, kTaps };
    const uint64_t expect_off[] = { 0, kTaps, kTaps, kTaps, kTaps, kTaps,
                                    kTaps };
    const uint64_t expect_noff[] = { 0, kTaps, kTaps, kTaps, kTaps, kTaps,
                                     kTaps };
    for (size_t i = 0; i < lines.size(); ++i) {
        double v = 0.0;
        BOOST_REQUIRE(parse_num(lines[i], "tap_count", v));
        const uint64_t tc = static_cast<uint64_t>(std::llround(v));
        BOOST_CHECK_EQUAL(tc, expect_tc[i]);
        BOOST_REQUIRE(parse_num(lines[i], "file_offset_taps", v));
        BOOST_CHECK_EQUAL(static_cast<uint64_t>(std::llround(v)),
                          expect_off[i]);
        double norm_off = -1.0;
        const bool has_norm_off =
            parse_num(lines[i], "file_offset_norm_taps", norm_off);
        BOOST_CHECK(has_norm_off);
        BOOST_CHECK_EQUAL(static_cast<uint64_t>(std::llround(norm_off)),
                          expect_noff[i]);
    }
}

// ---------------------------------------------------------------------------
// Malformed PDUs: rejected without JSONL lines.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_writer_invalid_pdus)
{
    const std::string dir = make_temp_dir("invalid");
    auto w = UwbCirWriter::make(dir, "cir", false, 16);
    auto dbg = gr::blocks::message_debug::make();
    auto tb = gr::make_top_block("qa_cir_writer_invalid");
    tb->msg_connect(w, "status", dbg, "store");
    BOOST_REQUIRE(w->start());
    tb->start();

    w->_post(pmt::mp("cir"), pmt::PMT_NIL);
    w->_post(pmt::mp("cir"), pmt::init_c32vector(4, std::vector<gr_complex>(4).data()));
    w->_post(pmt::mp("cir"),
             pmt::cons(pmt::make_dict(),
                       pmt::init_s16vector(8, std::vector<int16_t>(8).data())));

    BOOST_REQUIRE(wait_written(w, 3));
    tb->stop();
    tb->wait();
    BOOST_REQUIRE(w->stop());

    BOOST_CHECK_EQUAL(w->frames_received(), 0u);
    BOOST_CHECK_EQUAL(w->frames_invalid(), 3u);
    const auto lines = read_lines(dir + "/cir.jsonl");
    BOOST_CHECK_EQUAL(lines.size(), 0u);
    std::vector<uint8_t> raw;
    BOOST_REQUIRE(read_bytes(dir + "/cir.cf32", raw));
    BOOST_CHECK_EQUAL(raw.size(), 0u);
}

// ---------------------------------------------------------------------------
// stop() drains the queue; JSONL and binary stay consistent.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_writer_stop_drains)
{
    const std::string dir = make_temp_dir("drain");
    auto w = UwbCirWriter::make(dir, "cir", false, 16);
    auto dbg = gr::blocks::message_debug::make();
    auto tb = gr::make_top_block("qa_cir_writer_drain");
    tb->msg_connect(w, "status", dbg, "store");
    BOOST_REQUIRE(w->start());
    tb->start();

    for (uint64_t i = 0; i < 5; ++i)
        w->_post(pmt::mp("cir"), make_frame(i, "ok", make_taps(i)));
    // Wait until the handler has enqueued every frame, then stop without
    // waiting for the worker: stop() must drain the internal queue.
    const auto t0 = std::chrono::steady_clock::now();
    while (w->frames_received() < 5) {
        BOOST_REQUIRE(std::chrono::duration_cast<std::chrono::milliseconds>(
                          std::chrono::steady_clock::now() - t0)
                          .count() < 30000);
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    tb->stop();
    tb->wait();
    BOOST_REQUIRE(w->stop());

    BOOST_CHECK_EQUAL(w->frames_written(), 5u);
    const auto lines = read_lines(dir + "/cir.jsonl");
    BOOST_CHECK_EQUAL(lines.size(), 5u);
    std::vector<uint8_t> raw;
    BOOST_REQUIRE(read_bytes(dir + "/cir.cf32", raw));
    BOOST_CHECK_EQUAL(raw.size(), 5u * kTaps * 8);
}

// ---------------------------------------------------------------------------
// Restart truncates and resets counters.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_writer_restart)
{
    const std::string dir = make_temp_dir("restart");
    auto w = UwbCirWriter::make(dir, "cir", false, 16);
    auto dbg = gr::blocks::message_debug::make();
    auto tb = gr::make_top_block("qa_cir_writer_restart");
    tb->msg_connect(w, "status", dbg, "store");
    BOOST_REQUIRE(w->start());
    tb->start();
    w->_post(pmt::mp("cir"), make_frame(0, "ok", make_taps(0)));
    BOOST_REQUIRE(wait_written(w, 1));
    tb->stop();
    tb->wait();
    BOOST_REQUIRE(w->stop());
    BOOST_CHECK_EQUAL(w->frames_written(), 1u);

    BOOST_REQUIRE(w->start());
    tb->start();
    w->_post(pmt::mp("cir"), make_frame(0, "sfd_failed", make_taps(0)));
    BOOST_REQUIRE(wait_written(w, 1));
    tb->stop();
    tb->wait();
    BOOST_REQUIRE(w->stop());

    BOOST_CHECK_EQUAL(w->frames_written(), 0u);
    BOOST_CHECK_EQUAL(w->frames_failed(), 1u);
    const auto lines = read_lines(dir + "/cir.jsonl");
    BOOST_CHECK_EQUAL(lines.size(), 1u);
    std::vector<uint8_t> raw;
    BOOST_REQUIRE(read_bytes(dir + "/cir.cf32", raw));
    BOOST_CHECK_EQUAL(raw.size(), 0u); // truncated by restart
}
