/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * QA for header-only radar SYNC origin backtrack (998.4 MS/s).
 *
 * Synthetic construction uses testdata/reference_preamble.bin (code-9 SYNC).
 * Canonical MATLAB golden in testdata/uwb_radar/ is required
 * (generator=export_uwb_radar_golden.m).
 */

#include <boost/test/unit_test.hpp>

#include <gnuradio/gr_complex.h>
#include <gnuradio/uwb/uwb_phy_profile.h>
#include <gnuradio/uwb/uwb_radar_timing_core.h>

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <random>
#include <sstream>
#include <string>
#include <vector>

namespace {

using gr::uwb::demod::kQm35SamplesPerSymbol;
using gr::uwb::demod::Qm35825Profile;
using gr::uwb::radar::nominal_preamble_start;
using gr::uwb::radar::RadarTimingResult;
using gr::uwb::radar::refine_sync_origin;
using gr::uwb::radar::TimingStatus;

#ifndef UWB_TESTDATA_DIR
#define UWB_TESTDATA_DIR "../../../testdata"
#endif

constexpr float kTimingThreshold = Qm35825Profile{}.sfd_detection_threshold;
constexpr int64_t kDefaultMargin = 8; // demod timing_track_radius
constexpr int64_t kWideMargin = 64;
constexpr int64_t kPre = 256;
constexpr int64_t kTail = 256;
constexpr size_t kSynthSyncReps = 4;

bool load_cf32(const std::string& path, std::vector<gr_complex>& out)
{
    std::ifstream f(path, std::ios::binary);
    if (!f)
        return false;
    f.seekg(0, std::ios::end);
    const size_t bytes = static_cast<size_t>(f.tellg());
    f.seekg(0, std::ios::beg);
    if (bytes == 0 || bytes % 8 != 0)
        return false;
    out.resize(bytes / 8);
    f.read(reinterpret_cast<char*>(out.data()),
           static_cast<std::streamsize>(bytes));
    return static_cast<bool>(f);
}

std::string testdata_path(const std::string& rel)
{
    const std::string from_def = std::string(UWB_TESTDATA_DIR) + "/" + rel;
    {
        std::ifstream f(from_def, std::ios::binary);
        if (f)
            return from_def;
    }
    const char* prefixes[] = {
        "../../../testdata/",
        "../../testdata/",
        "../testdata/",
        "testdata/",
    };
    for (const char* p : prefixes) {
        const std::string path = std::string(p) + rel;
        std::ifstream f(path, std::ios::binary);
        if (f)
            return path;
    }
    return from_def;
}

std::vector<gr_complex> load_sync_template()
{
    std::vector<gr_complex> tmpl;
    const std::string path = testdata_path("reference_preamble.bin");
    BOOST_REQUIRE_MESSAGE(load_cf32(path, tmpl),
                          "cannot load reference_preamble.bin from " + path);
    BOOST_REQUIRE_EQUAL(tmpl.size(), kQm35SamplesPerSymbol);
    gr::uwb::core::uwb_l2_normalize(tmpl);
    return tmpl;
}

void place_waveform(std::vector<gr_complex>& rx,
                    int64_t start,
                    const std::vector<gr_complex>& w,
                    gr_complex gain = gr_complex(1.0f, 0.0f))
{
    BOOST_REQUIRE(start >= 0);
    BOOST_REQUIRE(static_cast<size_t>(start) + w.size() <= rx.size());
    for (size_t k = 0; k < w.size(); ++k)
        rx[static_cast<size_t>(start) + k] = gain * w[k];
}

bool parse_json_number(const std::string& json,
                       const char* key,
                       double& out)
{
    const std::string pat = std::string("\"") + key + "\"";
    auto pos = json.find(pat);
    if (pos == std::string::npos)
        return false;
    pos = json.find(':', pos + pat.size());
    if (pos == std::string::npos)
        return false;
    ++pos;
    while (pos < json.size() &&
           (json[pos] == ' ' || json[pos] == '\t' || json[pos] == '\n'))
        ++pos;
    try {
        size_t idx = 0;
        out = std::stod(json.substr(pos), &idx);
        return idx > 0;
    } catch (...) {
        return false;
    }
}

bool parse_json_string(const std::string& json,
                       const char* key,
                       std::string& out)
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

bool parse_nested_number(const std::string& json,
                         const char* object_key,
                         const char* field,
                         double& out)
{
    const std::string obj = std::string("\"") + object_key + "\"";
    auto rx = json.find(obj);
    if (rx == std::string::npos)
        return false;
    const std::string pat = std::string("\"") + field + "\"";
    auto pos = json.find(pat, rx);
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

struct SynthFrame {
    std::vector<gr_complex> rx;
    int64_t origin = 0;
    int64_t sfd_start = 0;
    size_t n_sync = 0;
};

SynthFrame make_sync_frame(const std::vector<gr_complex>& sync,
                           size_t n_sync = kSynthSyncReps)
{
    SynthFrame f;
    f.n_sync = n_sync;
    f.origin = kPre;
    f.sfd_start = f.origin + static_cast<int64_t>(n_sync) *
                                 static_cast<int64_t>(kQm35SamplesPerSymbol);
    f.rx.assign(static_cast<size_t>(f.sfd_start) +
                    kQm35SamplesPerSymbol + static_cast<size_t>(kTail),
                gr_complex(0.0f, 0.0f));
    for (size_t i = 0; i < n_sync; ++i)
        place_waveform(f.rx,
                       f.origin + static_cast<int64_t>(i) *
                                      static_cast<int64_t>(kQm35SamplesPerSymbol),
                       sync);
    return f;
}

void assert_refine_window(const RadarTimingResult& out, int64_t expected_n)
{
    BOOST_CHECK_EQUAL(out.refine_correlations,
                      static_cast<uint32_t>(expected_n));
    if (out.refine_hi >= out.refine_lo)
        BOOST_CHECK_EQUAL(out.refine_hi - out.refine_lo + 1, expected_n);
}

void assert_exhaustive_offsets(const std::vector<gr_complex>& rx,
                               int64_t true_origin,
                               int64_t true_sfd,
                               size_t n_sync,
                               int64_t margin,
                               float threshold,
                               const std::vector<gr_complex>& tmpl)
{
    int misses = 0;
    int64_t first_miss = 0;
    for (int64_t off = -margin; off <= margin; ++off) {
        const int64_t sfd = true_sfd + off;
        if (sfd < 0)
            continue;
        RadarTimingResult out;
        const bool ok =
            refine_sync_origin(rx.data(), rx.size(), sfd, n_sync,
                               kQm35SamplesPerSymbol, margin, threshold,
                               tmpl.data(), tmpl.size(), out);
        const bool hit =
            ok && out.status == TimingStatus::Ok &&
            std::llabs(out.preamble_start_sample - true_origin) <= 1;
        if (!hit) {
            if (misses == 0)
                first_miss = off;
            ++misses;
        }
    }
    BOOST_CHECK_MESSAGE(
        misses == 0,
        "exhaustive SYNC refine missed " + std::to_string(misses) +
            " offsets in [-" + std::to_string(margin) + ",+" +
            std::to_string(margin) + "]; first miss off=" +
            std::to_string(first_miss));
}

} // namespace

BOOST_AUTO_TEST_CASE(test_radar_timing_nominal_arithmetic)
{
    const size_t sps = kQm35SamplesPerSymbol;
    BOOST_CHECK_EQUAL(sps, size_t(1016));

    const int64_t sfd = 67021;
    BOOST_CHECK_EQUAL(nominal_preamble_start(sfd, 64, sps), int64_t(1997));
    BOOST_CHECK_EQUAL(sfd - int64_t(64) * int64_t(sps), int64_t(1997));
    BOOST_CHECK_EQUAL(nominal_preamble_start(sfd, 32, sps),
                      sfd - int64_t(32) * int64_t(sps));
    BOOST_CHECK_EQUAL(nominal_preamble_start(sfd, 32, sps), int64_t(34509));
    BOOST_CHECK_EQUAL(nominal_preamble_start(sfd, 128, sps),
                      sfd - int64_t(128) * int64_t(sps));
    BOOST_CHECK_EQUAL(nominal_preamble_start(sfd, 128, sps), int64_t(-63027));

    BOOST_CHECK_EQUAL(nominal_preamble_start(-1, 64, sps), int64_t(-1));
    BOOST_CHECK_EQUAL(nominal_preamble_start(sfd, 0, sps), int64_t(-1));
    BOOST_CHECK_EQUAL(nominal_preamble_start(sfd, 64, 0), int64_t(-1));

    const size_t huge = static_cast<size_t>(std::numeric_limits<int64_t>::max());
    BOOST_CHECK_EQUAL(nominal_preamble_start(sfd, huge, huge), int64_t(-1));
    int64_t prod = 0;
    BOOST_CHECK(!gr::uwb::radar::radar_i64_mul(
        std::numeric_limits<int64_t>::max(), 2, prod));
    int64_t sum = 0;
    BOOST_CHECK(!gr::uwb::radar::radar_i64_add(
        std::numeric_limits<int64_t>::max(), 1, sum));
}

BOOST_AUTO_TEST_CASE(test_radar_timing_extreme_coordinate_invalid_input)
{
    const auto tmpl = load_sync_template();
    std::vector<gr_complex> rx(tmpl.size(), gr_complex(0.0f, 0.0f));
    RadarTimingResult out;

    // nominal = INT64_MAX - 1016; adding this margin would overflow the
    // unclipped refine window. It must be rejected before any scan occurs.
    BOOST_CHECK(!refine_sync_origin(
        rx.data(), rx.size(), std::numeric_limits<int64_t>::max(), 1,
        kQm35SamplesPerSymbol, 2048, kTimingThreshold, tmpl.data(),
        tmpl.size(), out));
    BOOST_CHECK(out.status == TimingStatus::InvalidInput);
    BOOST_CHECK_EQUAL(out.preamble_start_sample, int64_t(-1));
}

BOOST_AUTO_TEST_CASE(test_radar_timing_synthetic_noiseless)
{
    const auto raw = [&]() {
        std::vector<gr_complex> t;
        const std::string path = testdata_path("reference_preamble.bin");
        BOOST_REQUIRE(load_cf32(path, t));
        return t;
    }();
    auto tmpl = raw;
    gr::uwb::core::uwb_l2_normalize(tmpl);
    BOOST_REQUIRE_EQUAL(tmpl.size(), kQm35SamplesPerSymbol);

    auto frame = make_sync_frame(raw, kSynthSyncReps);

    RadarTimingResult out0;
    BOOST_REQUIRE(refine_sync_origin(
        frame.rx.data(), frame.rx.size(), frame.sfd_start, kSynthSyncReps,
        kQm35SamplesPerSymbol, /*margin=*/0, kTimingThreshold, tmpl.data(),
        tmpl.size(), out0));
    BOOST_CHECK(out0.status == TimingStatus::Ok);
    BOOST_CHECK_EQUAL(out0.preamble_start_sample, frame.origin);
    BOOST_CHECK_EQUAL(out0.nominal_preamble_start, frame.origin);
    BOOST_CHECK_EQUAL(out0.sfd_start_sample, frame.sfd_start);
    BOOST_CHECK_EQUAL(out0.refine_lo, frame.origin);
    BOOST_CHECK_EQUAL(out0.refine_hi, frame.origin);
    BOOST_CHECK_GE(out0.metric, 0.95f);
    assert_refine_window(out0, 1);

    RadarTimingResult out8;
    BOOST_REQUIRE(refine_sync_origin(
        frame.rx.data(), frame.rx.size(), frame.sfd_start, kSynthSyncReps,
        kQm35SamplesPerSymbol, kDefaultMargin, kTimingThreshold, tmpl.data(),
        tmpl.size(), out8));
    BOOST_CHECK_EQUAL(out8.preamble_start_sample, frame.origin);
    BOOST_CHECK_EQUAL(out8.refine_lo, frame.origin - kDefaultMargin);
    BOOST_CHECK_EQUAL(out8.refine_hi, frame.origin + kDefaultMargin);
    BOOST_CHECK_GE(out8.metric, 0.95f);
    assert_refine_window(out8, 2 * kDefaultMargin + 1);

    RadarTimingResult out64;
    BOOST_REQUIRE(refine_sync_origin(
        frame.rx.data(), frame.rx.size(), frame.sfd_start, kSynthSyncReps,
        kQm35SamplesPerSymbol, kWideMargin, kTimingThreshold, tmpl.data(),
        tmpl.size(), out64));
    BOOST_CHECK_EQUAL(out64.preamble_start_sample, frame.origin);
    BOOST_CHECK_GE(out64.metric, 0.95f);
    assert_refine_window(out64, 2 * kWideMargin + 1);
}

BOOST_AUTO_TEST_CASE(test_radar_timing_exhaustive_synthetic_offsets)
{
    std::vector<gr_complex> raw;
    BOOST_REQUIRE(load_cf32(testdata_path("reference_preamble.bin"), raw));
    auto tmpl = raw;
    gr::uwb::core::uwb_l2_normalize(tmpl);

    auto frame = make_sync_frame(raw, kSynthSyncReps);
    BOOST_REQUIRE_GE(frame.origin, kWideMargin);
    assert_exhaustive_offsets(frame.rx, frame.origin, frame.sfd_start,
                              kSynthSyncReps, kDefaultMargin, kTimingThreshold,
                              tmpl);
    assert_exhaustive_offsets(frame.rx, frame.origin, frame.sfd_start,
                              kSynthSyncReps, kWideMargin, kTimingThreshold,
                              tmpl);
}

BOOST_AUTO_TEST_CASE(test_radar_timing_canonical_matlab_golden)
{
    const std::string meta_path = testdata_path("uwb_radar/metadata.json");
    const std::string rx_path = testdata_path("uwb_radar/rx_clean_998p4.cf32");
    const std::string tx_path = testdata_path("uwb_radar/tx_998p4.cf32");
    std::ifstream meta_f(meta_path);
    BOOST_REQUIRE_MESSAGE(meta_f.good(),
                          "canonical testdata/uwb_radar/metadata.json missing");
    std::ostringstream oss;
    oss << meta_f.rdbuf();
    const std::string json = oss.str();

    std::string generator;
    BOOST_REQUIRE(parse_json_string(json, "generator", generator));
    BOOST_REQUIRE_MESSAGE(
        generator.find("export_uwb_radar_golden.m") != std::string::npos,
        "canonical generator must be export_uwb_radar_golden.m, got " +
            generator);

    double reps = 0.0;
    BOOST_REQUIRE(parse_json_number(json, "sync_repetitions", reps));
    BOOST_CHECK_EQUAL(static_cast<int>(reps), 64);

    double sps_d = 0.0;
    BOOST_REQUIRE(parse_json_number(json, "samples_per_symbol", sps_d));
    BOOST_CHECK_EQUAL(static_cast<int>(sps_d), 1016);

    double sfd_start_d = -1.0;
    BOOST_REQUIRE(parse_nested_number(json, "rx_clean_998p4", "sfd_start",
                                      sfd_start_d));
    const int64_t sfd_truth = static_cast<int64_t>(std::llround(sfd_start_d));
    BOOST_CHECK_EQUAL(sfd_truth, int64_t(67021));

    double origin_d = -1.0;
    BOOST_REQUIRE(parse_nested_number(json, "rx_clean_998p4", "sync_origin",
                                      origin_d));
    const int64_t origin_truth = static_cast<int64_t>(std::llround(origin_d));
    BOOST_CHECK_EQUAL(origin_truth, int64_t(1997));
    BOOST_CHECK_EQUAL(nominal_preamble_start(sfd_truth, 64,
                                             kQm35SamplesPerSymbol),
                      origin_truth);

    std::vector<gr_complex> rx, tx;
    BOOST_REQUIRE(load_cf32(rx_path, rx));
    BOOST_REQUIRE(load_cf32(tx_path, tx));
    BOOST_REQUIRE_EQUAL(rx.size(), size_t(197005));
    BOOST_REQUIRE_GE(tx.size(), kQm35SamplesPerSymbol);
    std::vector<gr_complex> tmpl(tx.begin(),
                                 tx.begin() +
                                     static_cast<std::ptrdiff_t>(
                                         kQm35SamplesPerSymbol));
    gr::uwb::core::uwb_l2_normalize(tmpl);

    RadarTimingResult out8;
    BOOST_REQUIRE(refine_sync_origin(
        rx.data(), rx.size(), sfd_truth, 64, kQm35SamplesPerSymbol,
        kDefaultMargin, kTimingThreshold, tmpl.data(), tmpl.size(), out8));
    BOOST_CHECK(out8.status == TimingStatus::Ok);
    BOOST_CHECK_LE(std::llabs(out8.preamble_start_sample - origin_truth), 1);
    BOOST_CHECK_GE(out8.metric, 0.5f);
    std::printf("canonical origin measured=%lld truth=%lld metric=%.6f "
                "margin=8\n",
                static_cast<long long>(out8.preamble_start_sample),
                static_cast<long long>(origin_truth),
                static_cast<double>(out8.metric));

    RadarTimingResult out64;
    BOOST_REQUIRE(refine_sync_origin(
        rx.data(), rx.size(), sfd_truth, 64, kQm35SamplesPerSymbol,
        kWideMargin, kTimingThreshold, tmpl.data(), tmpl.size(), out64));
    BOOST_CHECK_LE(std::llabs(out64.preamble_start_sample - origin_truth), 1);
    BOOST_CHECK_GE(out64.metric, 0.5f);
    std::printf("canonical origin measured=%lld truth=%lld metric=%.6f "
                "margin=64\n",
                static_cast<long long>(out64.preamble_start_sample),
                static_cast<long long>(origin_truth),
                static_cast<double>(out64.metric));

    assert_exhaustive_offsets(rx, origin_truth, sfd_truth, 64, kDefaultMargin,
                              kTimingThreshold, tmpl);
}

BOOST_AUTO_TEST_CASE(test_radar_timing_integer_delay)
{
    const std::string meta_path = testdata_path("uwb_radar/metadata.json");
    std::ifstream meta_f(meta_path);
    BOOST_REQUIRE(meta_f.good());
    std::ostringstream oss;
    oss << meta_f.rdbuf();
    const std::string json = oss.str();

    double origin_clean_d = -1.0;
    BOOST_REQUIRE(parse_nested_number(json, "rx_clean_998p4", "sync_origin",
                                      origin_clean_d));
    const int64_t origin_clean =
        static_cast<int64_t>(std::llround(origin_clean_d));

    double origin_d = -1.0;
    BOOST_REQUIRE(parse_nested_number(json, "rx_delay_int_998p4", "sync_origin",
                                      origin_d));
    const int64_t origin_truth = static_cast<int64_t>(std::llround(origin_d));
    BOOST_CHECK_EQUAL(origin_truth, origin_clean + 37);
    BOOST_CHECK_EQUAL(origin_truth, int64_t(2034));

    double sfd_d = -1.0;
    BOOST_REQUIRE(parse_nested_number(json, "rx_delay_int_998p4", "sfd_start",
                                      sfd_d));
    const int64_t sfd_truth = static_cast<int64_t>(std::llround(sfd_d));

    std::vector<gr_complex> rx, tx;
    BOOST_REQUIRE(load_cf32(testdata_path("uwb_radar/rx_delay_int_998p4.cf32"),
                            rx));
    BOOST_REQUIRE(load_cf32(testdata_path("uwb_radar/tx_998p4.cf32"), tx));
    BOOST_REQUIRE_EQUAL(rx.size(), size_t(197005));
    std::vector<gr_complex> tmpl(tx.begin(),
                                 tx.begin() +
                                     static_cast<std::ptrdiff_t>(
                                         kQm35SamplesPerSymbol));
    gr::uwb::core::uwb_l2_normalize(tmpl);

    RadarTimingResult out;
    BOOST_REQUIRE(refine_sync_origin(
        rx.data(), rx.size(), sfd_truth, 64, kQm35SamplesPerSymbol,
        kDefaultMargin, kTimingThreshold, tmpl.data(), tmpl.size(), out));
    BOOST_CHECK_LE(std::llabs(out.preamble_start_sample - origin_truth), 1);
    std::printf("delay_int origin measured=%lld truth=%lld metric=%.6f\n",
                static_cast<long long>(out.preamble_start_sample),
                static_cast<long long>(origin_truth),
                static_cast<double>(out.metric));
}

BOOST_AUTO_TEST_CASE(test_radar_timing_destroyed_and_noise)
{
    std::vector<gr_complex> raw;
    BOOST_REQUIRE(load_cf32(testdata_path("reference_preamble.bin"), raw));
    auto tmpl = raw;
    gr::uwb::core::uwb_l2_normalize(tmpl);

    {
        auto frame = make_sync_frame(raw, kSynthSyncReps);
        std::fill(frame.rx.begin() + frame.origin,
                  frame.rx.begin() + frame.origin +
                      static_cast<int64_t>(kQm35SamplesPerSymbol),
                  gr_complex(0.0f, 0.0f));
        RadarTimingResult out;
        const bool ok = refine_sync_origin(
            frame.rx.data(), frame.rx.size(), frame.sfd_start, kSynthSyncReps,
            kQm35SamplesPerSymbol, kDefaultMargin, kTimingThreshold,
            tmpl.data(), tmpl.size(), out);
        BOOST_REQUIRE(!ok);
        BOOST_CHECK(out.status == TimingStatus::TimingFailed);
        BOOST_CHECK_EQUAL(out.preamble_start_sample, int64_t(-1));
        BOOST_CHECK(out.preamble_start_sample != out.nominal_preamble_start);
    }

    {
        auto frame = make_sync_frame(raw, kSynthSyncReps);
        std::mt19937 rng(11);
        std::shuffle(frame.rx.begin() + frame.origin,
                     frame.rx.begin() + frame.origin +
                         static_cast<int64_t>(kQm35SamplesPerSymbol),
                     rng);
        RadarTimingResult out;
        const bool ok = refine_sync_origin(
            frame.rx.data(), frame.rx.size(), frame.sfd_start, kSynthSyncReps,
            kQm35SamplesPerSymbol, kDefaultMargin, kTimingThreshold,
            tmpl.data(), tmpl.size(), out);
        BOOST_REQUIRE(!ok);
        BOOST_CHECK(out.status == TimingStatus::TimingFailed);
        BOOST_CHECK_EQUAL(out.preamble_start_sample, int64_t(-1));
    }

    auto proto = make_sync_frame(raw, kSynthSyncReps);
    const uint32_t seeds[] = { 20260904u, 7u, 99u, 12345u, 424242u };
    for (uint32_t seed : seeds) {
        std::vector<gr_complex> noise(proto.rx.size());
        std::mt19937 rng(seed);
        std::normal_distribution<float> g(0.0f, 1.0f);
        for (auto& v : noise)
            v = gr_complex(g(rng), g(rng));
        RadarTimingResult buried;
        const bool ok = refine_sync_origin(
            noise.data(), noise.size(), proto.sfd_start, kSynthSyncReps,
            kQm35SamplesPerSymbol, kDefaultMargin, kTimingThreshold,
            tmpl.data(), tmpl.size(), buried);
        BOOST_REQUIRE(!ok);
        BOOST_CHECK(buried.status == TimingStatus::TimingFailed);
        BOOST_CHECK_EQUAL(buried.preamble_start_sample, int64_t(-1));
        BOOST_CHECK(buried.preamble_start_sample !=
                    buried.nominal_preamble_start);
    }
}

BOOST_AUTO_TEST_CASE(test_radar_timing_clipped_negative_and_short)
{
    auto tmpl = load_sync_template();
    std::vector<gr_complex> rx(8192, gr_complex(0.1f, 0.0f));

    // Nominal origin is far negative; clipped refine window is empty.
    RadarTimingResult neg;
    const bool ok_neg = refine_sync_origin(
        rx.data(), rx.size(), /*sfd_start=*/100, 64, kQm35SamplesPerSymbol,
        kDefaultMargin, kTimingThreshold, tmpl.data(), tmpl.size(), neg);
    BOOST_REQUIRE(!ok_neg);
    BOOST_CHECK(neg.status == TimingStatus::TimingFailed);
    BOOST_CHECK_EQUAL(neg.preamble_start_sample, int64_t(-1));
    BOOST_CHECK_EQUAL(neg.sfd_start_sample, int64_t(100));
    BOOST_CHECK_LT(neg.nominal_preamble_start, int64_t(0));
    BOOST_CHECK(neg.preamble_start_sample != neg.nominal_preamble_start);

    // Buffer shorter than one SYNC: no legal start.
    std::vector<gr_complex> short_rx(kQm35SamplesPerSymbol / 2,
                                     gr_complex(0.1f, 0.0f));
    RadarTimingResult short_out;
    const bool ok_short = refine_sync_origin(
        short_rx.data(), short_rx.size(), /*sfd_start=*/4096, 64,
        kQm35SamplesPerSymbol, kDefaultMargin, kTimingThreshold, tmpl.data(),
        tmpl.size(), short_out);
    BOOST_REQUIRE(!ok_short);
    BOOST_CHECK(short_out.status == TimingStatus::TimingFailed);
    BOOST_CHECK_EQUAL(short_out.preamble_start_sample, int64_t(-1));
}

BOOST_AUTO_TEST_CASE(test_radar_timing_invalid_input)
{
    auto tmpl = load_sync_template();
    std::vector<gr_complex> rx(4096, gr_complex(0.0f, 0.0f));
    RadarTimingResult out;

    BOOST_CHECK(!refine_sync_origin(nullptr, rx.size(), 2000, 64,
                                    kQm35SamplesPerSymbol, kDefaultMargin,
                                    kTimingThreshold, tmpl.data(),
                                    tmpl.size(), out));
    BOOST_CHECK(out.status == TimingStatus::InvalidInput);
    BOOST_CHECK_EQUAL(out.preamble_start_sample, int64_t(-1));

    BOOST_CHECK(!refine_sync_origin(rx.data(), 0, 2000, 64,
                                    kQm35SamplesPerSymbol, kDefaultMargin,
                                    kTimingThreshold, tmpl.data(),
                                    tmpl.size(), out));
    BOOST_CHECK(out.status == TimingStatus::InvalidInput);

    BOOST_CHECK(!refine_sync_origin(rx.data(), rx.size(), -1, 64,
                                    kQm35SamplesPerSymbol, kDefaultMargin,
                                    kTimingThreshold, tmpl.data(),
                                    tmpl.size(), out));
    BOOST_CHECK(out.status == TimingStatus::InvalidInput);
    BOOST_CHECK_EQUAL(out.sfd_start_sample, int64_t(-1));
    BOOST_CHECK_EQUAL(out.nominal_preamble_start, int64_t(-1));

    BOOST_CHECK(!refine_sync_origin(rx.data(), rx.size(), 2000, 0,
                                    kQm35SamplesPerSymbol, kDefaultMargin,
                                    kTimingThreshold, tmpl.data(),
                                    tmpl.size(), out));
    BOOST_CHECK(out.status == TimingStatus::InvalidInput);

    BOOST_CHECK(!refine_sync_origin(rx.data(), rx.size(), 2000, 64, 0,
                                    kDefaultMargin, kTimingThreshold,
                                    tmpl.data(), tmpl.size(), out));
    BOOST_CHECK(out.status == TimingStatus::InvalidInput);

    BOOST_CHECK(!refine_sync_origin(rx.data(), rx.size(), 2000, 64,
                                    kQm35SamplesPerSymbol, -1, kTimingThreshold,
                                    tmpl.data(), tmpl.size(), out));
    BOOST_CHECK(out.status == TimingStatus::InvalidInput);

    BOOST_CHECK(!refine_sync_origin(rx.data(), rx.size(), 2000, 64,
                                    kQm35SamplesPerSymbol, kDefaultMargin, 0.0f,
                                    tmpl.data(), tmpl.size(), out));
    BOOST_CHECK(out.status == TimingStatus::InvalidInput);
    BOOST_CHECK(!refine_sync_origin(
        rx.data(), rx.size(), 2000, 64, kQm35SamplesPerSymbol, kDefaultMargin,
        std::numeric_limits<float>::quiet_NaN(), tmpl.data(), tmpl.size(),
        out));
    BOOST_CHECK(out.status == TimingStatus::InvalidInput);
    BOOST_CHECK(!refine_sync_origin(
        rx.data(), rx.size(), 2000, 64, kQm35SamplesPerSymbol, kDefaultMargin,
        std::numeric_limits<float>::infinity(), tmpl.data(), tmpl.size(),
        out));
    BOOST_CHECK(out.status == TimingStatus::InvalidInput);

    BOOST_CHECK(!refine_sync_origin(rx.data(), rx.size(), 2000, 64,
                                    kQm35SamplesPerSymbol, kDefaultMargin,
                                    kTimingThreshold, nullptr, tmpl.size(),
                                    out));
    BOOST_CHECK(out.status == TimingStatus::InvalidInput);
    BOOST_CHECK_EQUAL(out.preamble_start_sample, int64_t(-1));

    BOOST_CHECK(!refine_sync_origin(rx.data(), rx.size(), 2000, 64,
                                    kQm35SamplesPerSymbol, kDefaultMargin,
                                    kTimingThreshold, tmpl.data(), 0, out));
    BOOST_CHECK(out.status == TimingStatus::InvalidInput);

    BOOST_CHECK(!refine_sync_origin(rx.data(), rx.size(), 2000, 64,
                                    kQm35SamplesPerSymbol, kDefaultMargin,
                                    kTimingThreshold, tmpl.data(), 10, out));
    BOOST_CHECK(out.status == TimingStatus::InvalidInput);

    std::vector<gr_complex> nan_tmpl = tmpl;
    nan_tmpl[3] = gr_complex(std::numeric_limits<float>::quiet_NaN(), 0.0f);
    BOOST_CHECK(!refine_sync_origin(rx.data(), rx.size(), 2000, 64,
                                    kQm35SamplesPerSymbol, kDefaultMargin,
                                    kTimingThreshold, nan_tmpl.data(),
                                    nan_tmpl.size(), out));
    BOOST_CHECK(out.status == TimingStatus::InvalidInput);
}

BOOST_AUTO_TEST_CASE(test_radar_timing_nominal_outside_margin)
{
    std::vector<gr_complex> raw;
    BOOST_REQUIRE(load_cf32(testdata_path("reference_preamble.bin"), raw));
    auto tmpl = raw;
    gr::uwb::core::uwb_l2_normalize(tmpl);

    // Single SYNC so an extra-symbol prediction cannot land on a later copy.
    auto frame = make_sync_frame(raw, /*n_sync=*/1);

    RadarTimingResult miss_sym;
    const int64_t far_sfd =
        frame.sfd_start + static_cast<int64_t>(kQm35SamplesPerSymbol);
    const bool ok_sym = refine_sync_origin(
        frame.rx.data(), frame.rx.size(), far_sfd, /*n_sync=*/1,
        kQm35SamplesPerSymbol, kDefaultMargin, kTimingThreshold, tmpl.data(),
        tmpl.size(), miss_sym);
    BOOST_REQUIRE(!ok_sym);
    BOOST_CHECK(miss_sym.status == TimingStatus::TimingFailed);
    BOOST_CHECK_EQUAL(miss_sym.preamble_start_sample, int64_t(-1));
    BOOST_CHECK(miss_sym.preamble_start_sample !=
                miss_sym.nominal_preamble_start);
    BOOST_CHECK_EQUAL(miss_sym.nominal_preamble_start,
                      frame.origin +
                          static_cast<int64_t>(kQm35SamplesPerSymbol));

    RadarTimingResult miss_off;
    const int64_t off_sfd = frame.sfd_start + 37;
    const bool ok_off = refine_sync_origin(
        frame.rx.data(), frame.rx.size(), off_sfd, /*n_sync=*/1,
        kQm35SamplesPerSymbol, kDefaultMargin, kTimingThreshold, tmpl.data(),
        tmpl.size(), miss_off);
    BOOST_REQUIRE(!ok_off);
    BOOST_CHECK(miss_off.status == TimingStatus::TimingFailed);
    BOOST_CHECK_EQUAL(miss_off.preamble_start_sample, int64_t(-1));
}
