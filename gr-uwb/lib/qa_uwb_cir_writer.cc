/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * QA for the UwbCirWriter message block (Radar Step 8).
 *
 * Covers: mixed ok/failed frames (JSONL line count == frames, only ok
 * frames advance file_offset_taps and write a UCR4 record), SC16
 * block-floating reconstruction, malformed PDU rejection, queue-full
 * drops, stop() draining, and restart truncation.
 *
 * Queue-unit semantics (per-pulse PDU metering): the bounded queue counts
 * PDU entries — with batched repetitions one entry holds every repetition
 * record of a pulse, so a full queue drops whole pulses.  The slow-disk
 * hook UWB_CIR_WRITER_TEST_WRITE_DELAY_US (QA-only, read at construction)
 * adds a fixed sleep before each dequeued write so full-queue drops,
 * FIFO order, stop() drain and the capacity->RSS trend are observable
 * deterministically.
 */

#include <boost/test/unit_test.hpp>

#include <gnuradio/blocks/message_debug.h>
#include <gnuradio/gr_complex.h>
#include <gnuradio/top_block.h>
#include <gnuradio/uwb/uwb_cir_writer.h>
#include <pmt/pmt.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <malloc.h>
#include <sstream>
#include <string>
#include <thread>
#include <sys/resource.h>
#include <unistd.h>
#include <vector>

using gr::uwb::UwbCirWriter;
using gr::uwb::Ucr4Header;
using gr::uwb::decode_cir_sc16;
using gr::uwb::encode_cir_sc16;
using gr::uwb::ucr4_record_bytes;

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

double
rel_l2(const std::vector<gr_complex>& a, const std::vector<gr_complex>& b)
{
    if (a.size() != b.size() || a.empty())
        return 1.0;
    double num = 0.0, den = 0.0;
    for (size_t i = 0; i < a.size(); ++i) {
        const gr_complex d = a[i] - b[i];
        num += static_cast<double>(std::norm(d));
        den += static_cast<double>(std::norm(b[i]));
    }
    return den > 0.0 ? std::sqrt(num / den) : std::sqrt(num);
}

bool
load_ucr4(const std::string& path, std::vector<std::vector<gr_complex>>& out)
{
    std::vector<uint8_t> raw;
    if (!read_bytes(path, raw))
        return false;
    size_t off = 0;
    while (off + sizeof(Ucr4Header) <= raw.size()) {
        Ucr4Header hdr{};
        std::memcpy(&hdr, raw.data() + off, sizeof(hdr));
        if (std::memcmp(hdr.magic, "UCR4", 4) != 0)
            return false;
        const size_t rec = ucr4_record_bytes(hdr.tap_count);
        if (off + rec > raw.size())
            return false;
        std::vector<gr_complex> taps(hdr.tap_count);
        if (hdr.tap_count > 0) {
            const auto* iq = reinterpret_cast<const int16_t*>(
                raw.data() + off + sizeof(Ucr4Header));
            decode_cir_sc16(iq, hdr.tap_count, hdr.cir_scale, taps.data());
        }
        out.push_back(std::move(taps));
        off += rec;
    }
    return off == raw.size();
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
    meta = pmt::dict_add(meta, pmt::mp("freq_hz"),
                         pmt::from_double(6.4896e9 + 500.0 * pulse_id));
    meta = pmt::dict_add(meta, pmt::mp("freq_offset_hz"),
                         pmt::from_double(500.0 * pulse_id));
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

pmt::pmt_t
make_repetition_frame(uint64_t pulse_id,
                      uint64_t repetition_index,
                      uint64_t repetition_ordinal,
                      uint64_t repetition_count,
                      const std::string& status,
                      const std::vector<gr_complex>& taps)
{
    pmt::pmt_t frame = make_frame(pulse_id, status, taps);
    pmt::pmt_t meta = pmt::car(frame);
    meta = pmt::dict_add(meta, pmt::mp("cir_output"),
                         pmt::mp("repetition"));
    meta = pmt::dict_add(meta, pmt::mp("repetition_index"),
                         pmt::from_uint64(repetition_index));
    meta = pmt::dict_add(meta, pmt::mp("repetition_ordinal"),
                         pmt::from_uint64(repetition_ordinal));
    meta = pmt::dict_add(meta, pmt::mp("repetition_count"),
                         pmt::from_uint64(repetition_count));
    meta = pmt::dict_add(meta, pmt::mp("estimator_us"),
                         pmt::from_uint64(300 + repetition_ordinal));
    return pmt::cons(meta, pmt::cdr(frame));
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

// Wait until the handler has accounted for every posted frame (written +
// failed + dropped == want) without requiring the writer thread to drain.
bool
wait_settled(const UwbCirWriter::sptr& w,
             uint64_t want,
             long timeout_ms = 30000)
{
    const auto t0 = std::chrono::steady_clock::now();
    for (;;) {
        const uint64_t settled =
            w->frames_written() + w->frames_failed() + w->frames_dropped();
        if (settled >= want)
            return true;
        const auto ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - t0)
                .count();
        if (ms > timeout_ms) {
            BOOST_TEST_MESSAGE("wait_settled timeout: want=" << want
                               << " written=" << w->frames_written()
                               << " failed=" << w->frames_failed()
                               << " dropped=" << w->frames_dropped()
                               << " received=" << w->frames_received()
                               << " invalid=" << w->frames_invalid());
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
}

// Current process RSS in bytes (from /proc/self/statm), for the
// queue-capacity shrink trend test.  getrusage maxrss is monotonic, so the
// live-queue comparison uses statm.
uint64_t
current_rss_bytes()
{
    std::ifstream f("/proc/self/statm");
    uint64_t total = 0, resident = 0;
    if (!(f >> total >> resident))
        return 0;
    return resident * static_cast<uint64_t>(::sysconf(_SC_PAGESIZE));
}

struct ScopedDelayEnv
{
    explicit ScopedDelayEnv(const char* us)
    {
        setenv("UWB_CIR_WRITER_TEST_WRITE_DELAY_US", us, 1);
    }
    ~ScopedDelayEnv() { unsetenv("UWB_CIR_WRITER_TEST_WRITE_DELAY_US"); }
    ScopedDelayEnv(const ScopedDelayEnv&) = delete;
    ScopedDelayEnv& operator=(const ScopedDelayEnv&) = delete;
};

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
        pmt::pmt_t frame = make_frame(i, statuses[i], frames[i]);
        if (i == 0) {
            // Jam-specific fields must continue to override the generic
            // sense-frequency fallback in the UCR4 header.
            pmt::pmt_t meta = pmt::car(frame);
            meta = pmt::dict_add(meta, pmt::mp("jam_freq_actual_hz"),
                                 pmt::from_double(6.4902e9));
            meta = pmt::dict_add(meta, pmt::mp("jam_freq_offset_hz"),
                                 pmt::from_double(600000.0));
            frame = pmt::cons(meta, pmt::cdr(frame));
        }
        w->_post(pmt::mp("cir"), frame);
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

    // Binary: one UCR4 record per ok frame.
    std::vector<uint8_t> raw;
    BOOST_REQUIRE(read_bytes(dir + "/cir.ucr4", raw));
    BOOST_CHECK_EQUAL(raw.size(), posted_ok * ucr4_record_bytes(kTaps));
    std::vector<std::vector<gr_complex>> decoded;
    BOOST_REQUIRE(load_ucr4(dir + "/cir.ucr4", decoded));
    BOOST_REQUIRE_EQUAL(decoded.size(), posted_ok);

    uint64_t offset = 0;
    size_t ok_i = 0;
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
            BOOST_CHECK_LT(rel_l2(decoded[ok_i], frames[i]), 1e-4);
            std::vector<int16_t> expect_sc16(kTaps * 2);
            const float scale =
                encode_cir_sc16(frames[i].data(), kTaps, expect_sc16.data());
            const size_t rec_off = ok_i * ucr4_record_bytes(kTaps);
            Ucr4Header hdr{};
            std::memcpy(&hdr, raw.data() + rec_off, sizeof(hdr));
            BOOST_CHECK_EQUAL(std::string(hdr.magic, 4), "UCR4");
            BOOST_CHECK_EQUAL(hdr.pulse_id, i);
            BOOST_CHECK_EQUAL(hdr.tap_count, kTaps);
            const double expect_freq = i == 0
                ? 6.4902e9
                : 6.4896e9 + 500.0 * static_cast<double>(i);
            const double expect_offset = i == 0
                ? 600000.0
                : 500.0 * static_cast<double>(i);
            BOOST_CHECK_EQUAL(hdr.freq_hz, expect_freq);
            BOOST_CHECK_EQUAL(hdr.freq_offset_hz, expect_offset);
            BOOST_CHECK_CLOSE(hdr.cir_scale, scale, 1e-5);
            BOOST_CHECK(std::memcmp(raw.data() + rec_off + sizeof(Ucr4Header),
                                    expect_sc16.data(),
                                    kTaps * 4) == 0);
            int64_t sfd = -1, pre = -1;
            BOOST_REQUIRE(parse_i64(line, "sfd_start_sample", sfd));
            BOOST_CHECK_EQUAL(sfd, 67021);
            BOOST_REQUIRE(parse_i64(line, "preamble_start_sample", pre));
            BOOST_CHECK_EQUAL(pre, 1997);
            offset += kTaps;
            ++ok_i;
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

    std::vector<uint8_t> raw;
    BOOST_REQUIRE(read_bytes(dir + "/cir.ucr4", raw));
    BOOST_CHECK_EQUAL(raw.size(), 2u * ucr4_record_bytes(kTaps));
    BOOST_CHECK(!std::filesystem::exists(dir + "/cir_norm.cf32"));
    std::vector<std::vector<gr_complex>> decoded;
    BOOST_REQUIRE(load_ucr4(dir + "/cir.ucr4", decoded));
    BOOST_REQUIRE_EQUAL(decoded.size(), 2u);
    BOOST_CHECK_LT(rel_l2(decoded[0], taps), 1e-4);
    BOOST_CHECK_LT(rel_l2(decoded[1], taps), 1e-4);

    // Per-line expectations: [ok, missing-norm, short-norm, long-raw,
    // long-norm, failed, ok]. Offsets advance only on ok frames; failed
    // lines report the offset current at their time of write.
    const auto lines = read_lines(dir + "/cir.jsonl");
    BOOST_REQUIRE_EQUAL(lines.size(), 7u);
    const uint64_t expect_tc[] = { kTaps, 0, 0, 0, 0, 0, kTaps };
    const uint64_t expect_off[] = { 0, kTaps, kTaps, kTaps, kTaps, kTaps,
                                    kTaps };
    for (size_t i = 0; i < lines.size(); ++i) {
        double v = 0.0;
        BOOST_REQUIRE(parse_num(lines[i], "tap_count", v));
        const uint64_t tc = static_cast<uint64_t>(std::llround(v));
        BOOST_CHECK_EQUAL(tc, expect_tc[i]);
        BOOST_REQUIRE(parse_num(lines[i], "file_offset_taps", v));
        BOOST_CHECK_EQUAL(static_cast<uint64_t>(std::llround(v)),
                          expect_off[i]);
        BOOST_CHECK(lines[i].find("file_offset_norm_taps") ==
                    std::string::npos);
    }
}

// ---------------------------------------------------------------------------
// Per-repetition input: one compact JSON line per pulse.  A complete packet
// flushes as soon as repetition_count records arrive; stop() preserves an
// incomplete final packet and marks it observable.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_writer_groups_repetitions_per_pulse)
{
    const std::string dir = make_temp_dir("repetition_group");
    auto w = UwbCirWriter::make(dir, "cir", false, 16);
    auto dbg = gr::blocks::message_debug::make();
    auto tb = gr::make_top_block("qa_cir_writer_repetition_group");
    tb->msg_connect(w, "status", dbg, "store");
    BOOST_REQUIRE(w->start());
    tb->start();

    for (uint64_t ordinal = 0; ordinal < 3; ++ordinal) {
        w->_post(pmt::mp("cir"),
                 make_repetition_frame(7, 10 + ordinal, ordinal, 3, "ok",
                                       make_taps(ordinal)));
    }
    // Deliberately omit ordinal 2 for pulse 8.  Its line must be emitted by
    // the worker drain during stop(), with repetition_complete=false.
    for (uint64_t ordinal = 0; ordinal < 2; ++ordinal) {
        w->_post(pmt::mp("cir"),
                 make_repetition_frame(8, 10 + ordinal, ordinal, 3, "ok",
                                       make_taps(10 + ordinal)));
    }

    BOOST_REQUIRE(wait_written(w, 5));
    tb->stop();
    tb->wait();
    BOOST_REQUIRE(w->stop());

    const auto lines = read_lines(dir + "/cir.jsonl");
    BOOST_REQUIRE_EQUAL(lines.size(), 2u);
    BOOST_CHECK(lines[0].find("\"pulse_id\":7") != std::string::npos);
    BOOST_CHECK(lines[0].find("\"repetition_count\":3") !=
                std::string::npos);
    BOOST_CHECK(lines[0].find("\"repetition_records\":3") !=
                std::string::npos);
    BOOST_CHECK(lines[0].find("\"repetition_complete\":true") !=
                std::string::npos);
    BOOST_CHECK(lines[0].find("\"repetition_index\":[10,11,12]") !=
                std::string::npos);
    BOOST_CHECK(lines[0].find("\"tap_count\":[116,116,116]") !=
                std::string::npos);
    BOOST_CHECK(lines[0].find("\"file_offset_taps\":[0,116,232]") !=
                std::string::npos);
    BOOST_CHECK(lines[0].find("\"estimator_us\":[300,301,302]") !=
                std::string::npos);

    BOOST_CHECK(lines[1].find("\"pulse_id\":8") != std::string::npos);
    BOOST_CHECK(lines[1].find("\"repetition_records\":2") !=
                std::string::npos);
    BOOST_CHECK(lines[1].find("\"repetition_complete\":false") !=
                std::string::npos);
    BOOST_CHECK(lines[1].find("\"file_offset_taps\":[348,464]") !=
                std::string::npos);

    std::vector<uint8_t> raw;
    BOOST_REQUIRE(read_bytes(dir + "/cir.ucr4", raw));
    BOOST_CHECK_EQUAL(raw.size(), 5u * ucr4_record_bytes(kTaps));
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
    BOOST_REQUIRE(read_bytes(dir + "/cir.ucr4", raw));
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
    BOOST_REQUIRE(read_bytes(dir + "/cir.ucr4", raw));
    BOOST_CHECK_EQUAL(raw.size(), 5u * ucr4_record_bytes(kTaps));
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
    BOOST_REQUIRE(read_bytes(dir + "/cir.ucr4", raw));
    BOOST_CHECK_EQUAL(raw.size(), 0u); // truncated by restart
}

// ---------------------------------------------------------------------------
// Simulated slow disk (QA hook UWB_CIR_WRITER_TEST_WRITE_DELAY_US): a full
// queue drops whole entries while the FIFO order of everything that was
// enqueued is preserved and the UCR4 record offsets stay continuous.
// Queue depth counts PDU entries — one entry per message (a batched
// repetition pulse is one entry covering repetition_count records).
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_writer_slow_disk_full_queue_drop_order)
{
    const std::string dir = make_temp_dir("slow_full");
    const ScopedDelayEnv delay("3000"); // 3 ms per record > post loop time
    constexpr size_t kCapacity = 8;
    constexpr size_t kFrames = 60;
    auto w = UwbCirWriter::make(dir, "cir", false, kCapacity);
    auto dbg = gr::blocks::message_debug::make();
    auto tb = gr::make_top_block("qa_cir_writer_slow_full");
    tb->msg_connect(w, "status", dbg, "store");
    BOOST_REQUIRE(w->start());
    tb->start();

    for (uint64_t i = 0; i < kFrames; ++i)
        w->_post(pmt::mp("cir"), make_frame(i, "ok", make_taps(i)));
    // Every posted frame must be accounted for: enqueued or dropped.
    BOOST_REQUIRE(wait_settled(w, kFrames));

    BOOST_CHECK_EQUAL(w->frames_received(), kFrames);
    // The queue is bounded: while the (slow) writer was busy, the depth
    // reached the capacity exactly.
    BOOST_CHECK_EQUAL(w->queue_high_watermark(), kCapacity);
    // Drop accounting is exact: nothing is lost silently.
    const uint64_t written = w->frames_written();
    const uint64_t dropped = w->frames_dropped();
    BOOST_REQUIRE_GT(written, 0u);
    BOOST_REQUIRE_GT(dropped, 0u);
    BOOST_CHECK_EQUAL(written + dropped, kFrames);

    tb->stop();
    tb->wait();
    BOOST_REQUIRE(w->stop()); // drain: remaining queued entries are written

    // Everything enqueued before the queue filled is written first, in
    // posting order, with continuous UCR4 file offsets (ok frames only).
    const auto lines = read_lines(dir + "/cir.jsonl");
    BOOST_REQUIRE_EQUAL(lines.size(), written);
    std::vector<uint8_t> raw;
    BOOST_REQUIRE(read_bytes(dir + "/cir.ucr4", raw));
    BOOST_CHECK_EQUAL(raw.size(), written * ucr4_record_bytes(kTaps));
    std::vector<std::vector<gr_complex>> decoded;
    BOOST_REQUIRE(load_ucr4(dir + "/cir.ucr4", decoded));
    BOOST_REQUIRE_EQUAL(decoded.size(), written);
    uint64_t offset = 0;
    uint64_t prev_pid = 0;
    for (size_t i = 0; i < lines.size(); ++i) {
        double v = 0.0;
        BOOST_REQUIRE(parse_num(lines[i], "pulse_id", v));
        const uint64_t pid = static_cast<uint64_t>(std::llround(v));
        // FIFO order is preserved for everything written: the first
        // posted frame is always enqueued (empty queue) and later frames
        // are written strictly in posting order.  Frames that arrived
        // while the queue was full are the dropped ones, so the written
        // ids are increasing but not necessarily consecutive.
        if (i == 0)
            BOOST_CHECK_EQUAL(pid, 0u);
        else
            BOOST_CHECK_GT(pid, prev_pid);
        prev_pid = pid;
        BOOST_REQUIRE(parse_num(lines[i], "file_offset_taps", v));
        BOOST_CHECK_EQUAL(static_cast<uint64_t>(std::llround(v)), offset);
        BOOST_CHECK_LT(rel_l2(decoded[i], make_taps(pid)), 1e-4);
        offset += kTaps;
    }
}

// ---------------------------------------------------------------------------
// Simulated slow disk + stop(): everything still queued at stop() time is
// drained to the files; JSONL line count == written frames, UCR4 records ==
// written frames, and received == written + failed + dropped exactly.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_writer_slow_disk_stop_drains_rest)
{
    const std::string dir = make_temp_dir("slow_drain");
    const ScopedDelayEnv delay("5000"); // 5 ms per record
    constexpr size_t kCapacity = 16;
    constexpr size_t kFrames = 40;
    auto w = UwbCirWriter::make(dir, "cir", false, kCapacity);
    auto dbg = gr::blocks::message_debug::make();
    auto tb = gr::make_top_block("qa_cir_writer_slow_drain");
    tb->msg_connect(w, "status", dbg, "store");
    BOOST_REQUIRE(w->start());
    tb->start();

    for (uint64_t i = 0; i < kFrames; ++i)
        w->_post(pmt::mp("cir"), make_frame(i, "ok", make_taps(i)));
    BOOST_REQUIRE(wait_settled(w, kFrames));
    // Stop while the queue still holds entries (the writer is ~5 ms/frame).
    tb->stop();
    tb->wait();
    BOOST_REQUIRE(w->stop());

    BOOST_CHECK_EQUAL(w->frames_received(), kFrames);
    const uint64_t written = w->frames_written();
    const uint64_t dropped = w->frames_dropped();
    BOOST_REQUIRE_GT(written, 0u);
    BOOST_CHECK_EQUAL(written + dropped, kFrames);
    // Drain left nothing behind: file contents match the counters exactly.
    const auto lines = read_lines(dir + "/cir.jsonl");
    BOOST_REQUIRE_EQUAL(lines.size(), written);
    std::vector<uint8_t> raw;
    BOOST_REQUIRE(read_bytes(dir + "/cir.ucr4", raw));
    BOOST_CHECK_EQUAL(raw.size(), written * ucr4_record_bytes(kTaps));
    // FIFO order preserved across the stop() drain: strictly increasing
    // pulse_ids starting at the first posted frame (drops leave gaps).
    uint64_t prev_pid = 0;
    for (size_t i = 0; i < lines.size(); ++i) {
        double v = 0.0;
        BOOST_REQUIRE(parse_num(lines[i], "pulse_id", v));
        const uint64_t pid = static_cast<uint64_t>(std::llround(v));
        if (i == 0)
            BOOST_CHECK_EQUAL(pid, 0u);
        else
            BOOST_CHECK_GT(pid, prev_pid);
        prev_pid = pid;
    }
}

// ---------------------------------------------------------------------------
// Queue-capacity shrink trend (256 -> 64): with large tap payloads, the
// resident set while the queue is full tracks the capacity.  getrusage
// maxrss is monotonic, so the live comparison samples /proc/self/statm at
// each full-queue point.  A trend observation, not a precision test.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_writer_queue_capacity_rss_trend)
{
    // Frame payload > glibc's default 128 KiB mmap threshold so each queued
    // frame is mmap-backed and returned to the OS on free.
    constexpr size_t kBigTaps = 20480; // 20480 * 8 B = 160 KiB per frame
    const std::vector<gr_complex> big_taps(kBigTaps, gr_complex(0.25f, -0.5f));
    const ScopedDelayEnv delay("5000");

    auto fill_and_sample = [&](size_t capacity, size_t frames,
                               uint64_t& full_rss) {
        const std::string dir =
            make_temp_dir("rss_" + std::to_string(capacity));
        auto w = UwbCirWriter::make(dir, "cir", false, capacity);
        auto dbg = gr::blocks::message_debug::make();
        auto tb = gr::make_top_block(
            ("qa_cir_writer_rss_" + std::to_string(capacity)).c_str());
        // The block must be part of the flowgraph (a message edge) for its
        // scheduler thread to dispatch the queued handlers.
        tb->msg_connect(w, "status", dbg, "store");
        BOOST_REQUIRE(w->start());
        tb->start();
        // Return previously freed heap to the OS so this phase's baseline
        // is not polluted by earlier tests' retained arenas.
        malloc_trim(0);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        const uint64_t rss_base = current_rss_bytes();
        for (uint64_t i = 0; i < frames; ++i)
            w->_post(pmt::mp("cir"), make_frame(i, "ok", big_taps));
        BOOST_REQUIRE(wait_settled(w, frames));
        // The writer thread is still sleeping before its next write, so
        // most of the queue is resident here.
        full_rss = current_rss_bytes() - rss_base;
        tb->stop();
        tb->wait();
        BOOST_REQUIRE(w->stop());
    };

    uint64_t rss_full_256 = 0, rss_full_64 = 0;
    fill_and_sample(256, 260, rss_full_256); // full queue (~256 entries)
    fill_and_sample(64, 68, rss_full_64);    // full queue (~64 entries)
    BOOST_REQUIRE_GT(rss_full_256, 0u);
    BOOST_REQUIRE_GT(rss_full_64, 0u);
    // Observability aids (no assertion): monotonic peak RSS from
    // getrusage captures the 256-entry phase's high-water mark.
    struct rusage ru {};
    getrusage(RUSAGE_SELF, &ru);
    BOOST_TEST_MESSAGE("queue_rss_trend full_256=" << rss_full_256
                       << " B full_64=" << rss_full_64
                       << " B maxrss=" << ru.ru_maxrss * 1024UL << " B");
    // The 256-entry queue holds ~256 * 160 KiB ~= 40 MiB live, the
    // 64-entry one ~10 MiB; assert a conservative 8 MiB gap between the
    // baselined RSS deltas so the shrink trend is observable.
    BOOST_CHECK_GT(rss_full_256, rss_full_64 + 8u * 1024u * 1024u);
}

// ---------------------------------------------------------------------------
// Batched repetition PDUs under a full queue: one entry = one pulse, so a
// drop removes repetition_count logical frames at once while a written
// batch keeps repetition count and record offsets intact.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_writer_slow_disk_batch_drop_counts)
{
    const std::string dir = make_temp_dir("slow_batch");
    const ScopedDelayEnv delay("4000");
    constexpr size_t kCapacity = 4;
    constexpr size_t kReps = 32; // one batch PDU = 32 repetition records
    auto w = UwbCirWriter::make(dir, "cir", false, kCapacity);
    auto dbg = gr::blocks::message_debug::make();
    auto tb = gr::make_top_block("qa_cir_writer_slow_batch");
    tb->msg_connect(w, "status", dbg, "store");
    BOOST_REQUIRE(w->start());
    tb->start();

    constexpr size_t kPulses = 12;
    for (uint64_t p = 0; p < kPulses; ++p) {
        std::vector<gr_complex> batch(kReps * kTaps);
        for (size_t r = 0; r < kReps; ++r) {
            const auto one = make_taps(p * kReps + r);
            std::copy_n(one.data(), kTaps, batch.begin() + r * kTaps);
        }
        pmt::pmt_t meta = pmt::make_dict();
        meta = pmt::dict_add(meta, pmt::mp("pulse_id"), pmt::from_uint64(p));
        meta = pmt::dict_add(meta, pmt::mp("schedule_index"),
                             pmt::from_uint64(p));
        meta = pmt::dict_add(meta, pmt::mp("status"), pmt::mp("ok"));
        meta = pmt::dict_add(meta, pmt::mp("tap_count"),
                             pmt::from_uint64(kTaps));
        meta = pmt::dict_add(meta, pmt::mp("sample_rate"),
                             pmt::from_double(998.4e6));
        meta = pmt::dict_add(meta, pmt::mp("zero_delay_tap"),
                             pmt::from_long(16));
        meta = pmt::dict_add(meta, pmt::mp("repetition_batch"), pmt::PMT_T);
        meta = pmt::dict_add(meta, pmt::mp("repetition_first"),
                             pmt::from_uint64(0));
        meta = pmt::dict_add(meta, pmt::mp("repetition_count"),
                             pmt::from_uint64(kReps));
        std::vector<uint64_t> status(kReps, 0);
        std::vector<uint64_t> peak(kReps, 18);
        std::vector<float> metric(kReps, 0.9f);
        std::vector<float> l2(kReps, 0.01f);
        meta = pmt::dict_add(meta, pmt::mp("repetition_status_code"),
                             pmt::init_u64vector(kReps, status.data()));
        meta = pmt::dict_add(meta, pmt::mp("repetition_peak_tap"),
                             pmt::init_u64vector(kReps, peak.data()));
        meta = pmt::dict_add(meta, pmt::mp("repetition_peak_metric"),
                             pmt::init_f32vector(kReps, metric.data()));
        meta = pmt::dict_add(meta, pmt::mp("repetition_raw_l2_norm"),
                             pmt::init_f32vector(kReps, l2.data()));
        w->_post(pmt::mp("cir"),
                 pmt::cons(meta, pmt::init_c32vector(batch.size(),
                                                     batch.data())));
    }
    // logical frames = pulses * repetition_count
    BOOST_REQUIRE(wait_settled(w, kPulses * kReps));
    BOOST_CHECK_EQUAL(w->frames_received(), kPulses * kReps);
    BOOST_CHECK_EQUAL(w->queue_high_watermark(), kCapacity);

    tb->stop();
    tb->wait();
    BOOST_REQUIRE(w->stop());

    // Whole pulses are dropped at once: written + dropped logical frames
    // == total, and both are multiples of kReps (never a partial pulse).
    const uint64_t written_frames = w->frames_written();
    const uint64_t dropped_frames = w->frames_dropped();
    BOOST_REQUIRE_GT(written_frames, 0u);
    BOOST_REQUIRE_GT(dropped_frames, 0u);
    BOOST_CHECK_EQUAL(written_frames + dropped_frames, kPulses * kReps);
    BOOST_CHECK_EQUAL(written_frames % kReps, 0u);
    BOOST_CHECK_EQUAL(dropped_frames % kReps, 0u);
    // Written pulses in order, one JSONL line per pulse, UCR4 offsets
    // contiguous across every repetition of every written pulse.
    const auto lines = read_lines(dir + "/cir.jsonl");
    const size_t written_pulses = written_frames / kReps;
    BOOST_REQUIRE_EQUAL(lines.size(), written_pulses);
    std::vector<uint8_t> raw;
    BOOST_REQUIRE(read_bytes(dir + "/cir.ucr4", raw));
    BOOST_CHECK_EQUAL(raw.size(), written_frames * ucr4_record_bytes(kTaps));
    uint64_t offset = 0;
    uint64_t prev_pid = 0;
    for (size_t i = 0; i < written_pulses; ++i) {
        double v = 0.0;
        BOOST_REQUIRE(parse_num(lines[i], "pulse_id", v));
        const uint64_t pid = static_cast<uint64_t>(std::llround(v));
        if (i == 0)
            BOOST_CHECK_EQUAL(pid, 0u);
        else
            BOOST_CHECK_GT(pid, prev_pid);
        prev_pid = pid;
        BOOST_CHECK(lines[i].find("\"repetition_count\":" +
                                  std::to_string(kReps)) !=
                    std::string::npos);
        // spot-check the offsets column is the continuous run we expect
        std::ostringstream want;
        for (size_t r = 0; r < kReps; ++r) {
            if (r)
                want << ',';
            want << (offset + r * kTaps);
        }
        BOOST_CHECK(lines[i].find("\"file_offset_taps\":[" + want.str() +
                                  "]") != std::string::npos);
        offset += kReps * kTaps;
    }
}
