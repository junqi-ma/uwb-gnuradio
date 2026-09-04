/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * QA for header-only radar SFD narrow-window search (998.4 MS/s).
 *
 * Synthetic construction uses testdata/reference_preamble.bin (code-9 SYNC)
 * and demod::GetSfdSequence("4z2"). Canonical MATLAB golden in
 * testdata/uwb_radar/ is required (generator=export_uwb_radar_golden.m).
 */

#include <boost/test/unit_test.hpp>

#include <gnuradio/gr_complex.h>
#include <gnuradio/uwb/uwb_phy_profile.h>
#include <gnuradio/uwb/uwb_radar_sfd_core.h>

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <random>
#include <sstream>
#include <string>
#include <vector>

namespace {

using gr::uwb::demod::GetSfdSequence;
using gr::uwb::demod::kQm35SamplesPerSymbol;
using gr::uwb::demod::Qm35825Profile;
using gr::uwb::radar::prepare_sfd_template;
using gr::uwb::radar::RadarSfdResult;
using gr::uwb::radar::RadarSfdScratch;
using gr::uwb::radar::search_sfd;
using gr::uwb::radar::SfdStatus;

#ifndef UWB_TESTDATA_DIR
#define UWB_TESTDATA_DIR "../../../testdata"
#endif

constexpr float kSfdThreshold = Qm35825Profile{}.sfd_detection_threshold;
constexpr int64_t kMargin = 64;
constexpr int64_t kPre = 256;
constexpr int64_t kTail = 256;

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
    return tmpl;
}

std::vector<gr_complex> kron_sfd(const std::vector<int8_t>& seq,
                                 const std::vector<gr_complex>& sync)
{
    std::vector<gr_complex> w(seq.size() * sync.size());
    for (size_t i = 0; i < seq.size(); ++i) {
        const float s = static_cast<float>(seq[i]);
        for (size_t k = 0; k < sync.size(); ++k)
            w[i * sync.size() + k] =
                gr_complex(s * sync[k].real(), s * sync[k].imag());
    }
    return w;
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

bool prepare_seq(const std::vector<int8_t>& seq,
                 const std::vector<gr_complex>& sync,
                 RadarSfdScratch& scratch)
{
    return prepare_sfd_template(seq.data(), seq.size(), sync.data(),
                                sync.size(), scratch);
}

struct SynthFrame {
    std::vector<gr_complex> rx;
    int64_t sfd_start = 0;
    size_t sfd_len = 0;
};

SynthFrame make_frame(const std::vector<gr_complex>& sfd_wf,
                      int64_t delay = 0,
                      size_t n_sync_before = 2)
{
    SynthFrame f;
    f.sfd_len = sfd_wf.size();
    f.sfd_start = kPre + static_cast<int64_t>(n_sync_before) *
                              static_cast<int64_t>(kQm35SamplesPerSymbol) +
                  delay;
    f.rx.assign(static_cast<size_t>(f.sfd_start) + f.sfd_len +
                    static_cast<size_t>(kTail),
                gr_complex(0.0f, 0.0f));
    return f;
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

void assert_exhaustive_offsets(const std::vector<gr_complex>& rx,
                               int64_t true_start,
                               int64_t margin,
                               float threshold,
                               const RadarSfdScratch& scratch)
{
    int misses = 0;
    int64_t first_miss = 0;
    for (int64_t off = -margin; off <= margin; ++off) {
        const int64_t pred = true_start + off;
        if (pred < 0)
            continue;
        RadarSfdResult out;
        const bool ok = search_sfd(rx.data(), rx.size(), pred, margin,
                                   threshold, out, scratch);
        const bool hit =
            ok && out.status == SfdStatus::Ok &&
            std::llabs(out.sfd_start_sample - true_start) <= 1;
        if (!hit) {
            if (misses == 0)
                first_miss = off;
            ++misses;
        }
    }
    BOOST_CHECK_MESSAGE(
        misses == 0,
        "exhaustive SFD search missed " + std::to_string(misses) +
            " offsets in [-" + std::to_string(margin) + ",+" +
            std::to_string(margin) + "]; first miss off=" +
            std::to_string(first_miss));
}

} // namespace

BOOST_AUTO_TEST_CASE(test_radar_sfd_noiseless_hit)
{
    const auto sync = load_sync_template();
    const auto seq = GetSfdSequence("4z2");
    BOOST_REQUIRE_EQUAL(seq.size(), size_t(8));
    const auto sfd_wf = kron_sfd(seq, sync);

    RadarSfdScratch scratch;
    BOOST_REQUIRE(prepare_seq(seq, sync, scratch));
    BOOST_REQUIRE_EQUAL(scratch.sfd_template.size(), sfd_wf.size());

    auto frame = make_frame(sfd_wf);
    place_waveform(frame.rx, frame.sfd_start, sfd_wf);

    RadarSfdResult out;
    BOOST_REQUIRE(search_sfd(frame.rx.data(), frame.rx.size(), frame.sfd_start,
                             kMargin, kSfdThreshold, out, scratch));
    BOOST_CHECK(out.status == SfdStatus::Ok);
    BOOST_CHECK_EQUAL(out.sfd_start_sample, frame.sfd_start);
    BOOST_CHECK_EQUAL(out.predicted_start_sample, frame.sfd_start);
    BOOST_CHECK_EQUAL(out.search_lo, frame.sfd_start - kMargin);
    BOOST_CHECK_EQUAL(out.search_hi, frame.sfd_start + kMargin);
    BOOST_CHECK_GE(out.metric, 0.95f);
    BOOST_CHECK_EQUAL(out.coarse_correlations, 0u);
    BOOST_CHECK_EQUAL(out.fine_correlations,
                      static_cast<uint32_t>(out.search_hi - out.search_lo + 1));
}

BOOST_AUTO_TEST_CASE(test_radar_sfd_predicted_offset_inside_margin)
{
    const auto sync = load_sync_template();
    const auto seq = GetSfdSequence("4z2");
    const auto sfd_wf = kron_sfd(seq, sync);
    RadarSfdScratch scratch;
    BOOST_REQUIRE(prepare_seq(seq, sync, scratch));

    auto frame = make_frame(sfd_wf);
    place_waveform(frame.rx, frame.sfd_start, sfd_wf);

    for (int64_t off : { int64_t(37), int64_t(-37) }) {
        RadarSfdResult out;
        const int64_t pred = frame.sfd_start + off;
        BOOST_REQUIRE(search_sfd(frame.rx.data(), frame.rx.size(), pred,
                                 kMargin, kSfdThreshold, out, scratch));
        BOOST_CHECK(out.status == SfdStatus::Ok);
        BOOST_CHECK_LE(std::llabs(out.sfd_start_sample - frame.sfd_start), 1);
        BOOST_CHECK_GE(out.metric, 0.95f);
    }

    // Outside the caller margin: do not hunt extra symbols the way stage_sfd
    // does (±1..±4 symbol offsets).
    RadarSfdResult miss;
    const int64_t far = frame.sfd_start +
                        static_cast<int64_t>(kQm35SamplesPerSymbol);
    BOOST_CHECK(!search_sfd(frame.rx.data(), frame.rx.size(), far, kMargin,
                            kSfdThreshold, miss, scratch));
    BOOST_CHECK(miss.status == SfdStatus::SfdFailed);
    BOOST_CHECK_EQUAL(miss.sfd_start_sample, int64_t(-1));
    BOOST_CHECK(miss.sfd_start_sample != far);
}

BOOST_AUTO_TEST_CASE(test_radar_sfd_window_edges)
{
    const auto sync = load_sync_template();
    const auto seq = GetSfdSequence("4z2");
    const auto sfd_wf = kron_sfd(seq, sync);
    RadarSfdScratch scratch;
    BOOST_REQUIRE(prepare_seq(seq, sync, scratch));

    auto frame = make_frame(sfd_wf);
    place_waveform(frame.rx, frame.sfd_start, sfd_wf);

    RadarSfdResult left;
    const int64_t pred_left = frame.sfd_start + kMargin;
    BOOST_REQUIRE(search_sfd(frame.rx.data(), frame.rx.size(), pred_left,
                             kMargin, kSfdThreshold, left, scratch));
    BOOST_CHECK_EQUAL(left.search_lo, frame.sfd_start);
    BOOST_CHECK_EQUAL(left.sfd_start_sample, frame.sfd_start);
    BOOST_CHECK_GE(left.metric, 0.95f);

    RadarSfdResult right;
    const int64_t pred_right = frame.sfd_start - kMargin;
    BOOST_REQUIRE(search_sfd(frame.rx.data(), frame.rx.size(), pred_right,
                             kMargin, kSfdThreshold, right, scratch));
    BOOST_CHECK_EQUAL(right.search_hi, frame.sfd_start);
    BOOST_CHECK_EQUAL(right.sfd_start_sample, frame.sfd_start);
    BOOST_CHECK_GE(right.metric, 0.95f);
}

BOOST_AUTO_TEST_CASE(test_radar_sfd_truncated_window)
{
    const auto sync = load_sync_template();
    const auto seq = GetSfdSequence("4z2");
    RadarSfdScratch scratch;
    BOOST_REQUIRE(prepare_seq(seq, sync, scratch));
    const size_t sfd_len = scratch.sfd_template.size();

    // Buffer shorter than the SFD template: no legal start.
    std::vector<gr_complex> short_rx(sfd_len / 2, gr_complex(0.1f, 0.0f));
    RadarSfdResult out;
    BOOST_CHECK(!search_sfd(short_rx.data(), short_rx.size(), 0, kMargin,
                            kSfdThreshold, out, scratch));
    BOOST_CHECK(out.status == SfdStatus::SfdFailed ||
                out.status == SfdStatus::InvalidInput);
    BOOST_CHECK_EQUAL(out.sfd_start_sample, int64_t(-1));

    // Predicted near the end so the clipped window is empty.
    std::vector<gr_complex> rx(sfd_len + 16, gr_complex(0.1f, 0.0f));
    const int64_t pred = static_cast<int64_t>(rx.size()) - 4;
    RadarSfdResult end;
    BOOST_CHECK(!search_sfd(rx.data(), rx.size(), pred, kMargin, kSfdThreshold,
                            end, scratch));
    BOOST_CHECK(end.status == SfdStatus::SfdFailed ||
                end.status == SfdStatus::InvalidInput);
    BOOST_CHECK_EQUAL(end.sfd_start_sample, int64_t(-1));
    BOOST_CHECK(end.sfd_start_sample != pred);
}

BOOST_AUTO_TEST_CASE(test_radar_sfd_invalid_input)
{
    const auto sync = load_sync_template();
    const auto seq = GetSfdSequence("4z2");
    RadarSfdScratch scratch;
    BOOST_REQUIRE(prepare_seq(seq, sync, scratch));
    std::vector<gr_complex> rx(1024, gr_complex(0.0f, 0.0f));

    RadarSfdResult out;
    BOOST_CHECK(!search_sfd(nullptr, rx.size(), 0, kMargin, kSfdThreshold, out,
                            scratch));
    BOOST_CHECK(out.status == SfdStatus::InvalidInput);
    BOOST_CHECK_EQUAL(out.sfd_start_sample, int64_t(-1));

    BOOST_CHECK(!search_sfd(rx.data(), 0, 0, kMargin, kSfdThreshold, out,
                            scratch));
    BOOST_CHECK(out.status == SfdStatus::InvalidInput);
    BOOST_CHECK_EQUAL(out.sfd_start_sample, int64_t(-1));

    BOOST_CHECK(!search_sfd(rx.data(), rx.size(), -1, kMargin, kSfdThreshold,
                            out, scratch));
    BOOST_CHECK(out.status == SfdStatus::InvalidInput);
    BOOST_CHECK_EQUAL(out.sfd_start_sample, int64_t(-1));

    BOOST_CHECK(!search_sfd(rx.data(), rx.size(), 0, -1, kSfdThreshold, out,
                            scratch));
    BOOST_CHECK(out.status == SfdStatus::InvalidInput);
    BOOST_CHECK_EQUAL(out.sfd_start_sample, int64_t(-1));

    RadarSfdScratch empty;
    BOOST_CHECK(!search_sfd(rx.data(), rx.size(), 0, kMargin, kSfdThreshold,
                            out, empty));
    BOOST_CHECK(out.status == SfdStatus::InvalidInput);
    BOOST_CHECK_EQUAL(out.sfd_start_sample, int64_t(-1));

    BOOST_CHECK(!search_sfd(rx.data(), rx.size(), 0, kMargin, 0.0f, out,
                            scratch));
    BOOST_CHECK(out.status == SfdStatus::InvalidInput);
    BOOST_CHECK(!search_sfd(rx.data(), rx.size(), 0, kMargin, -0.1f, out,
                            scratch));
    BOOST_CHECK(out.status == SfdStatus::InvalidInput);
    BOOST_CHECK(!search_sfd(rx.data(), rx.size(), 0, kMargin,
                            std::numeric_limits<float>::quiet_NaN(), out,
                            scratch));
    BOOST_CHECK(out.status == SfdStatus::InvalidInput);
    BOOST_CHECK(!search_sfd(rx.data(), rx.size(), 0, kMargin,
                            std::numeric_limits<float>::infinity(), out,
                            scratch));
    BOOST_CHECK(out.status == SfdStatus::InvalidInput);

    // Search coordinates must not wrap when the predicted start is near the
    // signed sample-index limit. The result is a caller-contract error, not
    // an empty search window or a wrapped scan.
    BOOST_CHECK(!search_sfd(rx.data(), rx.size(),
                            std::numeric_limits<int64_t>::max(), 1,
                            kSfdThreshold, out, scratch));
    BOOST_CHECK(out.status == SfdStatus::InvalidInput);
    BOOST_CHECK_EQUAL(out.sfd_start_sample, int64_t(-1));

    std::vector<gr_complex> zeros(kQm35SamplesPerSymbol,
                                  gr_complex(0.0f, 0.0f));
    RadarSfdScratch zero_scratch;
    BOOST_CHECK(!prepare_seq(seq, zeros, zero_scratch));
    BOOST_CHECK(zero_scratch.sfd_template.empty());

    std::vector<gr_complex> nan_sync(kQm35SamplesPerSymbol,
                                     gr_complex(std::numeric_limits<float>::quiet_NaN(),
                                                0.0f));
    RadarSfdScratch nan_scratch;
    BOOST_CHECK(!prepare_seq(seq, nan_sync, nan_scratch));
    BOOST_CHECK(nan_scratch.sfd_template.empty());
}

BOOST_AUTO_TEST_CASE(test_radar_sfd_delay_phase_amplitude)
{
    const auto sync = load_sync_template();
    const auto seq = GetSfdSequence("4z2");
    const auto sfd_wf = kron_sfd(seq, sync);
    RadarSfdScratch scratch;
    BOOST_REQUIRE(prepare_seq(seq, sync, scratch));

    const gr_complex phase(std::cos(1.3f), std::sin(1.3f));
    const float scales[] = { 0.1f, 2.0f };
    for (float scale : scales) {
        auto frame = make_frame(sfd_wf, /*delay=*/5);
        place_waveform(frame.rx, frame.sfd_start, sfd_wf, scale * phase);
        RadarSfdResult out;
        BOOST_REQUIRE(search_sfd(frame.rx.data(), frame.rx.size(),
                                 frame.sfd_start, kMargin, kSfdThreshold, out,
                                 scratch));
        BOOST_CHECK_EQUAL(out.sfd_start_sample, frame.sfd_start);
        BOOST_CHECK_GE(out.metric, kSfdThreshold);
        BOOST_CHECK_GE(out.metric, 0.95f);
    }
}

BOOST_AUTO_TEST_CASE(test_radar_sfd_awgn_detects)
{
    const auto sync = load_sync_template();
    const auto seq = GetSfdSequence("4z2");
    const auto sfd_wf = kron_sfd(seq, sync);
    RadarSfdScratch scratch;
    BOOST_REQUIRE(prepare_seq(seq, sync, scratch));

    double energy = 0.0;
    for (size_t k = 0; k < sfd_wf.size(); ++k)
        energy += static_cast<double>(std::norm(sfd_wf[k]));
    const float rms =
        static_cast<float>(std::sqrt(energy / static_cast<double>(sfd_wf.size())));
    const float noise_std = 0.05f * rms;
    const uint32_t seeds[] = { 20260904u, 7u, 99u, 12345u };

    for (uint32_t seed : seeds) {
        auto frame = make_frame(sfd_wf);
        place_waveform(frame.rx, frame.sfd_start, sfd_wf);
        std::mt19937 rng(seed);
        std::normal_distribution<float> g(0.0f, noise_std);
        for (auto& v : frame.rx)
            v += gr_complex(g(rng), g(rng));
        RadarSfdResult out;
        BOOST_REQUIRE(search_sfd(frame.rx.data(), frame.rx.size(),
                                 frame.sfd_start, kMargin, kSfdThreshold, out,
                                 scratch));
        BOOST_CHECK_LE(std::llabs(out.sfd_start_sample - frame.sfd_start), 1);
    }
}

BOOST_AUTO_TEST_CASE(test_radar_sfd_pure_noise_false_alarm_zero)
{
    const auto sync = load_sync_template();
    const auto seq = GetSfdSequence("4z2");
    RadarSfdScratch scratch;
    BOOST_REQUIRE(prepare_seq(seq, sync, scratch));

    auto proto = make_frame(kron_sfd(seq, sync));
    const int64_t pred = proto.sfd_start + 11;
    const uint32_t seeds[] = { 20260904u, 7u, 99u, 12345u, 424242u };
    for (uint32_t seed : seeds) {
        std::vector<gr_complex> noise(proto.rx.size());
        std::mt19937 rng(seed);
        std::normal_distribution<float> g(0.0f, 1.0f);
        for (auto& v : noise)
            v = gr_complex(g(rng), g(rng));
        RadarSfdResult buried;
        const bool ok = search_sfd(noise.data(), noise.size(), pred, kMargin,
                                   kSfdThreshold, buried, scratch);
        BOOST_REQUIRE(!ok);
        BOOST_CHECK(buried.status == SfdStatus::SfdFailed);
        BOOST_CHECK_EQUAL(buried.sfd_start_sample, int64_t(-1));
        BOOST_CHECK(buried.sfd_start_sample != pred);
    }
}

BOOST_AUTO_TEST_CASE(test_radar_sfd_exhaustive_synthetic_offsets)
{
    const auto sync = load_sync_template();
    const auto seq = GetSfdSequence("4z2");
    const auto sfd_wf = kron_sfd(seq, sync);
    RadarSfdScratch scratch;
    BOOST_REQUIRE(prepare_seq(seq, sync, scratch));

    auto frame = make_frame(sfd_wf);
    place_waveform(frame.rx, frame.sfd_start, sfd_wf);
    BOOST_REQUIRE_GE(frame.sfd_start, kMargin);
    BOOST_REQUIRE_GE(static_cast<int64_t>(frame.rx.size()) -
                         static_cast<int64_t>(frame.sfd_len),
                     frame.sfd_start + kMargin);
    assert_exhaustive_offsets(frame.rx, frame.sfd_start, kMargin, kSfdThreshold,
                              scratch);
}

BOOST_AUTO_TEST_CASE(test_radar_sfd_destroyed_must_not_return_predicted)
{
    const auto sync = load_sync_template();
    const auto seq = GetSfdSequence("4z2");
    const auto sfd_wf = kron_sfd(seq, sync);
    RadarSfdScratch scratch;
    BOOST_REQUIRE(prepare_seq(seq, sync, scratch));

    const int64_t pred_off = 37;

    // Zero the SFD region.
    {
        auto frame = make_frame(sfd_wf);
        place_waveform(frame.rx, frame.sfd_start, sfd_wf);
        std::fill(frame.rx.begin() + frame.sfd_start,
                  frame.rx.begin() + frame.sfd_start +
                      static_cast<int64_t>(sfd_wf.size()),
                  gr_complex(0.0f, 0.0f));
        const int64_t pred = frame.sfd_start + pred_off;
        RadarSfdResult out;
        BOOST_CHECK(!search_sfd(frame.rx.data(), frame.rx.size(), pred, kMargin,
                                kSfdThreshold, out, scratch));
        BOOST_CHECK(out.status == SfdStatus::SfdFailed);
        BOOST_CHECK_EQUAL(out.sfd_start_sample, int64_t(-1));
        BOOST_CHECK(out.sfd_start_sample != pred);
    }

    // Replace SFD with SYNC repeats (same length, wrong polarity sequence).
    {
        auto frame = make_frame(sfd_wf);
        std::vector<int8_t> ones(seq.size(), 1);
        place_waveform(frame.rx, frame.sfd_start, kron_sfd(ones, sync));
        const int64_t pred = frame.sfd_start + pred_off;
        RadarSfdResult out;
        BOOST_CHECK(!search_sfd(frame.rx.data(), frame.rx.size(), pred, kMargin,
                                kSfdThreshold, out, scratch));
        BOOST_CHECK(out.status == SfdStatus::SfdFailed);
        BOOST_CHECK_EQUAL(out.sfd_start_sample, int64_t(-1));
        BOOST_CHECK(out.sfd_start_sample != pred);
    }

    // Scramble SFD chips.
    {
        auto frame = make_frame(sfd_wf);
        auto scrambled = sfd_wf;
        std::mt19937 rng(11);
        std::shuffle(scrambled.begin(), scrambled.end(), rng);
        place_waveform(frame.rx, frame.sfd_start, scrambled);
        const int64_t pred = frame.sfd_start + pred_off;
        RadarSfdResult out;
        BOOST_CHECK(!search_sfd(frame.rx.data(), frame.rx.size(), pred, kMargin,
                                kSfdThreshold, out, scratch));
        BOOST_CHECK(out.status == SfdStatus::SfdFailed);
        BOOST_CHECK_EQUAL(out.sfd_start_sample, int64_t(-1));
        BOOST_CHECK(out.sfd_start_sample != pred);
    }
}

BOOST_AUTO_TEST_CASE(test_radar_sfd_wrong_sfd_mode)
{
    const auto sync = load_sync_template();
    const auto seq_4z2 = GetSfdSequence("4z2");
    const auto seq_ieee = GetSfdSequence("ieee");
    BOOST_REQUIRE(!seq_ieee.empty());
    const auto wf_4z2 = kron_sfd(seq_4z2, sync);

    RadarSfdScratch ieee_scratch;
    BOOST_REQUIRE(prepare_seq(seq_ieee, sync, ieee_scratch));

    auto frame = make_frame(wf_4z2);
    place_waveform(frame.rx, frame.sfd_start, wf_4z2);

    RadarSfdResult out;
    const bool ok =
        search_sfd(frame.rx.data(), frame.rx.size(), frame.sfd_start, kMargin,
                   kSfdThreshold, out, ieee_scratch);
    BOOST_REQUIRE(!ok);
    BOOST_CHECK(out.status == SfdStatus::SfdFailed);
    BOOST_CHECK_EQUAL(out.sfd_start_sample, int64_t(-1));
    BOOST_CHECK(out.sfd_start_sample != frame.sfd_start);
}

BOOST_AUTO_TEST_CASE(test_radar_sfd_hot_path_no_growth)
{
    const auto sync = load_sync_template();
    const auto seq = GetSfdSequence("4z2");
    const auto sfd_wf = kron_sfd(seq, sync);
    RadarSfdScratch scratch;
    BOOST_REQUIRE(prepare_seq(seq, sync, scratch));
    const size_t cap = scratch.sfd_template.capacity();
    BOOST_REQUIRE_GT(cap, 0u);

    auto frame = make_frame(sfd_wf);
    place_waveform(frame.rx, frame.sfd_start, sfd_wf);

    for (int i = 0; i < 100; ++i) {
        RadarSfdResult out;
        BOOST_REQUIRE(search_sfd(frame.rx.data(), frame.rx.size(),
                                 frame.sfd_start, kMargin, kSfdThreshold, out,
                                 scratch));
        BOOST_CHECK_EQUAL(scratch.sfd_template.capacity(), cap);
        BOOST_CHECK_EQUAL(scratch.sfd_template.size(), sfd_wf.size());
    }
}

BOOST_AUTO_TEST_CASE(test_radar_sfd_canonical_matlab_golden)
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

    std::string sfd_mode;
    BOOST_REQUIRE(parse_json_string(json, "sfd_mode", sfd_mode));
    BOOST_CHECK_EQUAL(sfd_mode, std::string("4z2"));

    double reps = 0.0;
    BOOST_REQUIRE(parse_json_number(json, "sync_repetitions", reps));
    BOOST_CHECK_EQUAL(static_cast<int>(reps), 64);

    double fs = 0.0;
    BOOST_REQUIRE(parse_json_number(json, "rate_work_hz", fs));
    BOOST_CHECK_LT(std::abs(fs - 998.4e6), 1.0);

    double sfd_start_d = -1.0;
    BOOST_REQUIRE(parse_nested_number(json, "rx_clean_998p4", "sfd_start",
                                      sfd_start_d));
    const int64_t truth = static_cast<int64_t>(std::llround(sfd_start_d));

    std::vector<gr_complex> rx, tx;
    BOOST_REQUIRE(load_cf32(rx_path, rx));
    BOOST_REQUIRE(load_cf32(tx_path, tx));
    BOOST_REQUIRE_GE(tx.size(), kQm35SamplesPerSymbol);
    std::vector<gr_complex> sync(tx.begin(),
                                 tx.begin() +
                                     static_cast<std::ptrdiff_t>(
                                         kQm35SamplesPerSymbol));

    const auto seq = GetSfdSequence("4z2");
    RadarSfdScratch scratch;
    BOOST_REQUIRE(prepare_seq(seq, sync, scratch));

    RadarSfdResult out;
    BOOST_REQUIRE(search_sfd(rx.data(), rx.size(), truth, kMargin,
                             kSfdThreshold, out, scratch));
    BOOST_CHECK_LE(std::llabs(out.sfd_start_sample - truth), 1);
    BOOST_CHECK_GE(out.metric, 0.5f);

    assert_exhaustive_offsets(rx, truth, kMargin, kSfdThreshold, scratch);
}
