/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * QA for header-only radar CIR estimator (998.4 MS/s).
 *
 * Canonical MATLAB golden in testdata/uwb_radar/ is required
 * (generator=export_uwb_radar_golden.m).  CIR comparison is complex raw
 * and L2-normalized taps; demod CirResult / real-only goldens are not
 * the acceptance gate.
 */

#include <boost/test/unit_test.hpp>

#include <gnuradio/gr_complex.h>
#include <gnuradio/uwb/uwb_phy_profile.h>
#include <gnuradio/uwb/uwb_radar_cir_estimator.h>

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

namespace {

using gr::uwb::demod::GetPreambleCode;
using gr::uwb::demod::kQm35CodeLength;
using gr::uwb::demod::kQm35SamplesPerSymbol;
using gr::uwb::radar::CirStatus;
using gr::uwb::radar::estimate_radar_cir;
using gr::uwb::radar::prepare_radar_cir_code;
using gr::uwb::radar::RadarCirEstimate;
using gr::uwb::radar::RadarCirScratch;

#ifndef UWB_TESTDATA_DIR
#define UWB_TESTDATA_DIR "../../../testdata"
#endif

constexpr size_t kSkip = 10;
constexpr size_t kMaxRep = 54;
constexpr size_t kNSync = 64;
constexpr size_t kRadarPre = 16;
constexpr size_t kRadarPost = 100;
constexpr size_t kRadarTaps = kRadarPre + kRadarPost;
constexpr size_t kDemodPre = 8;
constexpr size_t kDemodPost = 30;

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

bool parse_json_number(const std::string& json, const char* key, double& out)
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

double relative_l2(const std::complex<float>* a,
                   const std::complex<float>* b,
                   size_t n)
{
    double num = 0.0;
    double den = 0.0;
    for (size_t i = 0; i < n; ++i) {
        const double dr = static_cast<double>(a[i].real()) -
                          static_cast<double>(b[i].real());
        const double di = static_cast<double>(a[i].imag()) -
                          static_cast<double>(b[i].imag());
        num += dr * dr + di * di;
        const double br = static_cast<double>(b[i].real());
        const double bi = static_cast<double>(b[i].imag());
        den += br * br + bi * bi;
    }
    if (!(den > 0.0))
        return (num > 0.0) ? std::numeric_limits<double>::infinity() : 0.0;
    return std::sqrt(num) / std::sqrt(den);
}

bool prepare_code9(RadarCirScratch& scratch,
                   size_t max_pre = kRadarPre,
                   size_t max_post = kRadarPost)
{
    return prepare_radar_cir_code(GetPreambleCode(9), kQm35CodeLength, max_pre,
                                  max_post, scratch);
}

bool run_cir(const std::vector<gr_complex>& rx,
             int64_t origin,
             size_t pre,
             size_t post,
             size_t skip,
             size_t max_rep,
             size_t n_sync,
             RadarCirEstimate& out,
             RadarCirScratch& scratch)
{
    return estimate_radar_cir(rx.data(), rx.size(), origin,
                              kQm35SamplesPerSymbol, pre, post, skip, max_rep,
                              n_sync, out, scratch);
}

struct CanonicalMeta {
    std::string json;
    std::string generator;
    int64_t origin_clean = 0;
    int64_t origin_delay_int = 0;
    size_t sps = 0;
    size_t n_sync = 0;
    size_t skip = 0;
    size_t pre = 0;
    size_t post = 0;
    size_t tap_count = 0;
    size_t peak_clean = 0;
    size_t peak_int = 0;
    size_t peak_frac = 0;
    size_t valid_clean = 0;
    size_t delay_int = 0;
    double delay_frac = 0.0;
    double code_energy = 0.0;
    size_t rx_len = 0;
};

CanonicalMeta load_canonical_meta()
{
    CanonicalMeta m;
    const std::string meta_path = testdata_path("uwb_radar/metadata.json");
    std::ifstream meta_f(meta_path);
    BOOST_REQUIRE_MESSAGE(meta_f.good(),
                          "canonical testdata/uwb_radar/metadata.json missing");
    std::ostringstream oss;
    oss << meta_f.rdbuf();
    m.json = oss.str();

    BOOST_REQUIRE(parse_json_string(m.json, "generator", m.generator));
    BOOST_REQUIRE_MESSAGE(
        m.generator.find("export_uwb_radar_golden.m") != std::string::npos,
        "canonical generator must be export_uwb_radar_golden.m, got " +
            m.generator);

    double v = 0.0;
    BOOST_REQUIRE(parse_nested_number(m.json, "rx_clean_998p4", "sync_origin",
                                      v));
    m.origin_clean = static_cast<int64_t>(std::llround(v));
    BOOST_REQUIRE(parse_nested_number(m.json, "rx_delay_int_998p4",
                                      "sync_origin", v));
    m.origin_delay_int = static_cast<int64_t>(std::llround(v));
    BOOST_REQUIRE(parse_json_number(m.json, "samples_per_symbol", v));
    m.sps = static_cast<size_t>(std::llround(v));
    BOOST_REQUIRE(parse_json_number(m.json, "sync_repetitions", v));
    m.n_sync = static_cast<size_t>(std::llround(v));
    BOOST_REQUIRE(parse_json_number(m.json, "skip_initial_repetitions", v));
    m.skip = static_cast<size_t>(std::llround(v));
    BOOST_REQUIRE(parse_nested_number(m.json, "radar", "pre", v));
    m.pre = static_cast<size_t>(std::llround(v));
    BOOST_REQUIRE(parse_nested_number(m.json, "radar", "post", v));
    m.post = static_cast<size_t>(std::llround(v));
    BOOST_REQUIRE(parse_nested_number(m.json, "radar", "tap_count", v));
    m.tap_count = static_cast<size_t>(std::llround(v));
    BOOST_REQUIRE(
        parse_nested_number(m.json, "radar", "peak_tap_measured_clean", v));
    m.peak_clean = static_cast<size_t>(std::llround(v));
    BOOST_REQUIRE(parse_nested_number(
        m.json, "radar", "peak_tap_measured_delay_int", v));
    m.peak_int = static_cast<size_t>(std::llround(v));
    BOOST_REQUIRE(parse_nested_number(
        m.json, "radar", "peak_tap_measured_delay_frac", v));
    m.peak_frac = static_cast<size_t>(std::llround(v));
    BOOST_REQUIRE(parse_nested_number(
        m.json, "radar", "valid_repetitions_clean", v));
    m.valid_clean = static_cast<size_t>(std::llround(v));
    BOOST_REQUIRE(parse_nested_number(m.json, "delay_int", "samples", v));
    m.delay_int = static_cast<size_t>(std::llround(v));
    BOOST_REQUIRE(parse_nested_number(m.json, "delay_frac", "samples", v));
    m.delay_frac = v;
    BOOST_REQUIRE(parse_json_number(m.json, "code_energy", v));
    m.code_energy = v;
    BOOST_REQUIRE(parse_nested_number(m.json, "rx_window", "length_998p4", v));
    m.rx_len = static_cast<size_t>(std::llround(v));
    return m;
}

std::vector<gr_complex> load_radar_cf32(const char* name, size_t expect)
{
    std::vector<gr_complex> x;
    const std::string path = testdata_path(std::string("uwb_radar/") + name);
    BOOST_REQUIRE_MESSAGE(load_cf32(path, x), "cannot load " + path);
    BOOST_REQUIRE_EQUAL(x.size(), expect);
    return x;
}

std::vector<gr_complex>
shift_scale(const std::vector<gr_complex>& rx, int64_t delay, gr_complex gain)
{
    std::vector<gr_complex> y(rx.size(), gr_complex(0.f, 0.f));
    if (delay >= 0) {
        const size_t d = static_cast<size_t>(delay);
        if (d >= rx.size())
            return y;
        for (size_t i = 0; i + d < rx.size(); ++i)
            y[i + d] = gain * rx[i];
    } else {
        const size_t d = static_cast<size_t>(-delay);
        if (d >= rx.size())
            return y;
        for (size_t i = d; i < rx.size(); ++i)
            y[i - d] = gain * rx[i];
    }
    return y;
}

} // namespace

BOOST_AUTO_TEST_CASE(test_radar_cir_prepare_code9)
{
    RadarCirScratch scratch;
    BOOST_REQUIRE(prepare_code9(scratch));
    BOOST_REQUIRE_EQUAL(scratch.sampled_code.size(), kQm35SamplesPerSymbol);
    BOOST_CHECK_CLOSE(scratch.code_energy, 64.f, 1e-4);
    BOOST_REQUIRE_EQUAL(scratch.raw_taps.size(), kRadarTaps);
    BOOST_REQUIRE_EQUAL(scratch.norm_taps.size(), kRadarTaps);
    BOOST_REQUIRE_EQUAL(scratch.avg.size(),
                        kQm35SamplesPerSymbol + kRadarTaps - 1);

    BOOST_REQUIRE(!prepare_radar_cir_code(nullptr, kQm35CodeLength, kRadarPre,
                                          kRadarPost, scratch));
    BOOST_CHECK(scratch.sampled_code.empty());
    BOOST_REQUIRE(!prepare_radar_cir_code(GetPreambleCode(9), 0, kRadarPre,
                                          kRadarPost, scratch));
    std::vector<int8_t> zeros(kQm35CodeLength, 0);
    BOOST_REQUIRE(!prepare_radar_cir_code(zeros.data(), zeros.size(), kRadarPre,
                                          kRadarPost, scratch));
    BOOST_CHECK(scratch.sampled_code.empty());
    BOOST_CHECK_EQUAL(scratch.code_energy, 0.f);

    BOOST_REQUIRE(!prepare_radar_cir_code(
        GetPreambleCode(9), kQm35CodeLength,
        std::numeric_limits<size_t>::max(), 1, scratch));
}

BOOST_AUTO_TEST_CASE(test_radar_cir_clean_16_100)
{
    const auto meta = load_canonical_meta();
    BOOST_REQUIRE_EQUAL(meta.sps, kQm35SamplesPerSymbol);
    BOOST_REQUIRE_EQUAL(meta.n_sync, kNSync);
    BOOST_REQUIRE_EQUAL(meta.skip, kSkip);
    BOOST_REQUIRE_EQUAL(meta.pre, kRadarPre);
    BOOST_REQUIRE_EQUAL(meta.post, kRadarPost);
    BOOST_REQUIRE_EQUAL(meta.tap_count, kRadarTaps);
    BOOST_REQUIRE_EQUAL(meta.peak_clean, size_t(18));
    BOOST_REQUIRE_EQUAL(meta.origin_clean, int64_t(1997));
    BOOST_REQUIRE_EQUAL(meta.rx_len, size_t(197005));
    BOOST_CHECK_CLOSE(meta.code_energy, 64.0, 1e-6);

    const auto rx = load_radar_cf32("rx_clean_998p4.cf32", meta.rx_len);
    const auto g_raw =
        load_radar_cf32("cir_raw_clean_radar.cf32", kRadarTaps);
    const auto g_norm =
        load_radar_cf32("cir_norm_clean_radar.cf32", kRadarTaps);

    RadarCirScratch scratch;
    BOOST_REQUIRE(prepare_code9(scratch));
    BOOST_CHECK_CLOSE(scratch.code_energy, 64.f, 1e-4);

    RadarCirEstimate out;
    BOOST_REQUIRE(run_cir(rx, meta.origin_clean, kRadarPre, kRadarPost,
                          kSkip, kMaxRep, kNSync, out, scratch));
    BOOST_CHECK(out.status == CirStatus::Ok);
    BOOST_REQUIRE_EQUAL(out.tap_count, kRadarTaps);
    BOOST_CHECK_EQUAL(out.peak_tap, meta.peak_clean);
    BOOST_CHECK_EQUAL(out.valid_repetitions, meta.valid_clean);
    BOOST_CHECK_EQUAL(out.valid_repetitions, size_t(54));
    BOOST_CHECK_EQUAL(out.first_repetition, kSkip);
    BOOST_CHECK_CLOSE(out.peak_abs, std::abs(scratch.raw_taps[out.peak_tap]),
                      1e-4);

    const double e_raw =
        relative_l2(scratch.raw_taps.data(), g_raw.data(), kRadarTaps);
    const double e_norm =
        relative_l2(scratch.norm_taps.data(), g_norm.data(), kRadarTaps);
    BOOST_CHECK_MESSAGE(e_raw < 1e-5,
                        "clean raw relative L2=" + std::to_string(e_raw));
    BOOST_CHECK_MESSAGE(e_norm < 1e-5,
                        "clean norm relative L2=" + std::to_string(e_norm));
}

BOOST_AUTO_TEST_CASE(test_radar_cir_integer_delay)
{
    const auto meta = load_canonical_meta();
    BOOST_REQUIRE_EQUAL(meta.delay_int, size_t(37));
    BOOST_REQUIRE_EQUAL(meta.peak_int, meta.peak_clean + meta.delay_int);
    BOOST_REQUIRE_EQUAL(meta.peak_int, size_t(55));
    BOOST_REQUIRE_EQUAL(meta.origin_delay_int,
                        meta.origin_clean + static_cast<int64_t>(meta.delay_int));

    const auto rx = load_radar_cf32("rx_delay_int_998p4.cf32", meta.rx_len);
    const auto g_raw =
        load_radar_cf32("cir_raw_delay_int_radar.cf32", kRadarTaps);
    const auto g_norm =
        load_radar_cf32("cir_norm_delay_int_radar.cf32", kRadarTaps);

    RadarCirScratch scratch;
    BOOST_REQUIRE(prepare_code9(scratch));
    RadarCirEstimate out;
    // TX-time origin (not the delayed SYNC origin) so extra delay maps to taps.
    BOOST_REQUIRE(run_cir(rx, meta.origin_clean, kRadarPre, kRadarPost,
                          kSkip, kMaxRep, kNSync, out, scratch));
    BOOST_CHECK(out.status == CirStatus::Ok);
    BOOST_CHECK_EQUAL(out.peak_tap, meta.peak_int);
    BOOST_CHECK_EQUAL(out.peak_tap, size_t(18 + 37));
    BOOST_CHECK_EQUAL(out.valid_repetitions, size_t(54));

    const double e_raw =
        relative_l2(scratch.raw_taps.data(), g_raw.data(), kRadarTaps);
    const double e_norm =
        relative_l2(scratch.norm_taps.data(), g_norm.data(), kRadarTaps);
    BOOST_CHECK_MESSAGE(e_raw < 1e-5,
                        "delay_int raw relative L2=" + std::to_string(e_raw));
    BOOST_CHECK_MESSAGE(
        e_norm < 1e-5,
        "delay_int norm relative L2=" + std::to_string(e_norm));
}

BOOST_AUTO_TEST_CASE(test_radar_cir_fractional_delay)
{
    const auto meta = load_canonical_meta();
    BOOST_REQUIRE_EQUAL(meta.peak_frac, size_t(30));

    const auto rx = load_radar_cf32("rx_delay_frac_998p4.cf32", meta.rx_len);
    const auto g_raw =
        load_radar_cf32("cir_raw_delay_frac_radar.cf32", kRadarTaps);
    const auto g_norm =
        load_radar_cf32("cir_norm_delay_frac_radar.cf32", kRadarTaps);

    RadarCirScratch scratch;
    BOOST_REQUIRE(prepare_code9(scratch));
    RadarCirEstimate out;
    BOOST_REQUIRE(run_cir(rx, meta.origin_clean, kRadarPre, kRadarPost,
                          kSkip, kMaxRep, kNSync, out, scratch));
    BOOST_CHECK(out.status == CirStatus::Ok);
    const int64_t peak_err = std::llabs(static_cast<int64_t>(out.peak_tap) -
                                        static_cast<int64_t>(meta.peak_frac));
    BOOST_CHECK_MESSAGE(peak_err <= 1,
                        "frac peak_tap=" + std::to_string(out.peak_tap) +
                            " metadata=" + std::to_string(meta.peak_frac));

    const double e_raw =
        relative_l2(scratch.raw_taps.data(), g_raw.data(), kRadarTaps);
    const double e_norm =
        relative_l2(scratch.norm_taps.data(), g_norm.data(), kRadarTaps);
    BOOST_CHECK_MESSAGE(e_raw < 1e-5,
                        "delay_frac raw relative L2=" + std::to_string(e_raw));
    BOOST_CHECK_MESSAGE(
        e_norm < 1e-5,
        "delay_frac norm relative L2=" + std::to_string(e_norm));
}

BOOST_AUTO_TEST_CASE(test_radar_cir_amplitude_linear)
{
    const auto meta = load_canonical_meta();
    const auto rx = load_radar_cf32("rx_clean_998p4.cf32", meta.rx_len);

    RadarCirScratch scratch;
    BOOST_REQUIRE(prepare_code9(scratch));
    RadarCirEstimate base;
    BOOST_REQUIRE(run_cir(rx, meta.origin_clean, kRadarPre, kRadarPost,
                          kSkip, kMaxRep, kNSync, base, scratch));
    const std::vector<gr_complex> raw0(scratch.raw_taps.begin(),
                                       scratch.raw_taps.begin() +
                                           static_cast<std::ptrdiff_t>(
                                               kRadarTaps));
    const std::vector<gr_complex> norm0(scratch.norm_taps.begin(),
                                        scratch.norm_taps.begin() +
                                            static_cast<std::ptrdiff_t>(
                                                kRadarTaps));

    const float scales[] = { 0.1f, 0.5f, 2.0f };
    for (float s : scales) {
        std::vector<gr_complex> scaled(rx.size());
        for (size_t i = 0; i < rx.size(); ++i)
            scaled[i] = rx[i] * s;
        RadarCirEstimate out;
        BOOST_REQUIRE(run_cir(scaled, meta.origin_clean, kRadarPre, kRadarPost,
                              kSkip, kMaxRep, kNSync, out, scratch));
        BOOST_CHECK_EQUAL(out.peak_tap, base.peak_tap);
        std::vector<gr_complex> raw_unscaled(kRadarTaps);
        for (size_t i = 0; i < kRadarTaps; ++i)
            raw_unscaled[i] = scratch.raw_taps[i] / s;
        const double e_raw =
            relative_l2(raw_unscaled.data(), raw0.data(), kRadarTaps);
        const double e_norm =
            relative_l2(scratch.norm_taps.data(), norm0.data(), kRadarTaps);
        BOOST_CHECK_MESSAGE(e_raw < 1e-4,
                            "scale=" + std::to_string(s) +
                                " raw/scale relative L2=" +
                                std::to_string(e_raw));
        BOOST_CHECK_MESSAGE(e_norm < 1e-5,
                            "scale=" + std::to_string(s) +
                                " norm relative L2=" + std::to_string(e_norm));
    }
}

BOOST_AUTO_TEST_CASE(test_radar_cir_multipath_linear)
{
    const auto meta = load_canonical_meta();
    const auto rx = load_radar_cf32("rx_clean_998p4.cf32", meta.rx_len);

    const int64_t delays[] = { 0, 25, 50 };
    const gr_complex gains[] = {
        gr_complex(0.30f, 0.00f),
        gr_complex(1.00f, 0.00f),
        gr_complex(0.25f, 0.40f),
    };
    const size_t n_paths = 3;
    const size_t strongest = 1;

    RadarCirScratch scratch;
    BOOST_REQUIRE(prepare_code9(scratch));

    std::vector<std::vector<gr_complex>> path_raw(n_paths);
    std::vector<gr_complex> combo(rx.size(), gr_complex(0.f, 0.f));
    for (size_t p = 0; p < n_paths; ++p) {
        auto one = shift_scale(rx, delays[p], gains[p]);
        for (size_t i = 0; i < combo.size(); ++i)
            combo[i] += one[i];
        RadarCirEstimate out;
        BOOST_REQUIRE(run_cir(one, meta.origin_clean, kRadarPre, kRadarPost,
                              kSkip, kMaxRep, kNSync, out, scratch));
        path_raw[p].assign(scratch.raw_taps.begin(),
                           scratch.raw_taps.begin() +
                               static_cast<std::ptrdiff_t>(kRadarTaps));
        BOOST_CHECK_EQUAL(out.peak_tap, meta.peak_clean +
                                            static_cast<size_t>(delays[p]));
    }

    RadarCirEstimate mp;
    BOOST_REQUIRE(run_cir(combo, meta.origin_clean, kRadarPre, kRadarPost,
                          kSkip, kMaxRep, kNSync, mp, scratch));
    BOOST_CHECK_EQUAL(mp.peak_tap,
                      meta.peak_clean + static_cast<size_t>(delays[strongest]));

    std::vector<gr_complex> lin(kRadarTaps, gr_complex(0.f, 0.f));
    for (size_t p = 0; p < n_paths; ++p) {
        for (size_t i = 0; i < kRadarTaps; ++i)
            lin[i] += path_raw[p][i];
    }
    const double e_lin =
        relative_l2(scratch.raw_taps.data(), lin.data(), kRadarTaps);
    BOOST_CHECK_MESSAGE(e_lin < 1e-4,
                        "multipath raw vs linear mix L2=" +
                            std::to_string(e_lin));
}

BOOST_AUTO_TEST_CASE(test_radar_cir_skip_count_and_truncation)
{
    const auto meta = load_canonical_meta();
    const auto rx = load_radar_cf32("rx_clean_998p4.cf32", meta.rx_len);

    RadarCirScratch scratch;
    BOOST_REQUIRE(prepare_code9(scratch));

    RadarCirEstimate full;
    BOOST_REQUIRE(run_cir(rx, meta.origin_clean, kRadarPre, kRadarPost,
                          kSkip, kMaxRep, kNSync, full, scratch));
    BOOST_CHECK_EQUAL(full.valid_repetitions, size_t(54));

    RadarCirEstimate twenty;
    BOOST_REQUIRE(run_cir(rx, meta.origin_clean, kRadarPre, kRadarPost,
                          kSkip, size_t(20), kNSync, twenty, scratch));
    BOOST_CHECK_EQUAL(twenty.valid_repetitions, size_t(20));
    BOOST_CHECK(twenty.status == CirStatus::Ok);

    RadarCirEstimate skip_fail;
    const bool skip_ok =
        run_cir(rx, meta.origin_clean, kRadarPre, kRadarPost, kNSync, kMaxRep,
                kNSync, skip_fail, scratch);
    BOOST_REQUIRE(!skip_ok);
    BOOST_CHECK(skip_fail.status == CirStatus::InvalidInput);
    BOOST_CHECK_EQUAL(skip_fail.tap_count, size_t(0));

    RadarCirEstimate skip_over;
    BOOST_REQUIRE(!run_cir(rx, meta.origin_clean, kRadarPre, kRadarPost,
                           kNSync + 1, kMaxRep, kNSync, skip_over, scratch));
    BOOST_CHECK(skip_over.status == CirStatus::InvalidInput);
    BOOST_CHECK_EQUAL(skip_over.tap_count, size_t(0));

    const size_t wlen = kQm35SamplesPerSymbol + kRadarTaps - 1;
    const auto n_for_k = [&](size_t k) -> size_t {
        const int64_t rs =
            meta.origin_clean +
            static_cast<int64_t>(k) *
                static_cast<int64_t>(kQm35SamplesPerSymbol);
        const int64_t hi = rs - static_cast<int64_t>(kRadarPre) +
                           static_cast<int64_t>(wlen);
        return static_cast<size_t>(hi);
    };

    std::vector<gr_complex> trunc = rx;
    trunc.resize(n_for_k(20));
    RadarCirEstimate t20;
    BOOST_REQUIRE(run_cir(trunc, meta.origin_clean, kRadarPre, kRadarPost,
                          kSkip, kMaxRep, kNSync, t20, scratch));
    BOOST_CHECK(t20.status == CirStatus::Ok);
    BOOST_CHECK_EQUAL(t20.valid_repetitions, size_t(11)); // k = 10..20
    BOOST_CHECK_GT(t20.valid_repetitions, size_t(0));
    BOOST_CHECK_LT(t20.valid_repetitions, size_t(54));

    trunc.resize(n_for_k(10));
    RadarCirEstimate t10;
    BOOST_REQUIRE(run_cir(trunc, meta.origin_clean, kRadarPre, kRadarPost,
                          kSkip, kMaxRep, kNSync, t10, scratch));
    BOOST_CHECK_EQUAL(t10.valid_repetitions, size_t(1));

    trunc.resize(n_for_k(10) - 1);
    RadarCirEstimate tnone;
    BOOST_REQUIRE(!run_cir(trunc, meta.origin_clean, kRadarPre, kRadarPost,
                           kSkip, kMaxRep, kNSync, tnone, scratch));
    BOOST_CHECK(tnone.status == CirStatus::CirFailed);
    BOOST_CHECK_EQUAL(tnone.tap_count, size_t(0));
}

BOOST_AUTO_TEST_CASE(test_radar_cir_invalid_and_failed)
{
    const auto meta = load_canonical_meta();
    const auto rx = load_radar_cf32("rx_clean_998p4.cf32", meta.rx_len);
    RadarCirScratch scratch;
    BOOST_REQUIRE(prepare_code9(scratch));
    RadarCirEstimate out;

    BOOST_REQUIRE(!estimate_radar_cir(nullptr, rx.size(), meta.origin_clean,
                                      kQm35SamplesPerSymbol, kRadarPre,
                                      kRadarPost, kSkip, kMaxRep, kNSync, out,
                                      scratch));
    BOOST_CHECK(out.status == CirStatus::InvalidInput);
    BOOST_CHECK_EQUAL(out.tap_count, size_t(0));

    BOOST_REQUIRE(!estimate_radar_cir(rx.data(), 0, meta.origin_clean,
                                      kQm35SamplesPerSymbol, kRadarPre,
                                      kRadarPost, kSkip, kMaxRep, kNSync, out,
                                      scratch));
    BOOST_CHECK(out.status == CirStatus::InvalidInput);
    BOOST_CHECK_EQUAL(out.tap_count, size_t(0));

    BOOST_REQUIRE(!run_cir(rx, int64_t(-1), kRadarPre, kRadarPost, kSkip,
                           kMaxRep, kNSync, out, scratch));
    BOOST_CHECK(out.status == CirStatus::InvalidInput);

    BOOST_REQUIRE(!estimate_radar_cir(rx.data(), rx.size(), meta.origin_clean,
                                      0, kRadarPre, kRadarPost, kSkip, kMaxRep,
                                      kNSync, out, scratch));
    BOOST_CHECK(out.status == CirStatus::InvalidInput);

    BOOST_REQUIRE(!run_cir(rx, meta.origin_clean, 0, 0, kSkip, kMaxRep, kNSync,
                           out, scratch));
    BOOST_CHECK(out.status == CirStatus::InvalidInput);
    BOOST_CHECK_EQUAL(out.tap_count, size_t(0));

    RadarCirScratch empty;
    BOOST_REQUIRE(!run_cir(rx, meta.origin_clean, kRadarPre, kRadarPost, kSkip,
                           kMaxRep, kNSync, out, empty));
    BOOST_CHECK(out.status == CirStatus::InvalidInput);
    BOOST_CHECK_EQUAL(out.tap_count, size_t(0));

    RadarCirScratch small;
    BOOST_REQUIRE(prepare_code9(small, kDemodPre, kDemodPost));
    BOOST_REQUIRE(!run_cir(rx, meta.origin_clean, kRadarPre, kRadarPost, kSkip,
                           kMaxRep, kNSync, out, small));
    BOOST_CHECK(out.status == CirStatus::InvalidInput);
    BOOST_CHECK_EQUAL(out.tap_count, size_t(0));

    std::vector<gr_complex> tiny(32, gr_complex(0.1f, 0.0f));
    BOOST_REQUIRE(!run_cir(tiny, meta.origin_clean, kRadarPre, kRadarPost,
                           kSkip, kMaxRep, kNSync, out, scratch));
    BOOST_CHECK(out.status == CirStatus::CirFailed);
    BOOST_CHECK_EQUAL(out.tap_count, size_t(0));

    RadarCirScratch wide;
    BOOST_REQUIRE(prepare_code9(wide, size_t(300000), size_t(8)));
    BOOST_REQUIRE(!run_cir(rx, meta.origin_clean, size_t(300000), size_t(8),
                           kSkip, kMaxRep, kNSync, out, wide));
    BOOST_CHECK(out.status == CirStatus::CirFailed);
    BOOST_CHECK_EQUAL(out.tap_count, size_t(0));

    // A later failure must not look like the previous success.
    RadarCirEstimate ok_then_fail;
    BOOST_REQUIRE(run_cir(rx, meta.origin_clean, kRadarPre, kRadarPost, kSkip,
                          kMaxRep, kNSync, ok_then_fail, scratch));
    BOOST_REQUIRE(!run_cir(tiny, meta.origin_clean, kRadarPre, kRadarPost,
                           kSkip, kMaxRep, kNSync, ok_then_fail, scratch));
    BOOST_CHECK(ok_then_fail.status != CirStatus::Ok);
    BOOST_CHECK_EQUAL(ok_then_fail.tap_count, size_t(0));
    BOOST_CHECK_EQUAL(ok_then_fail.peak_tap, size_t(0));
    BOOST_CHECK_EQUAL(ok_then_fail.valid_repetitions, size_t(0));
}

BOOST_AUTO_TEST_CASE(test_radar_cir_hot_path_no_growth)
{
    const auto meta = load_canonical_meta();
    const auto rx = load_radar_cf32("rx_clean_998p4.cf32", meta.rx_len);
    RadarCirScratch scratch;
    BOOST_REQUIRE(prepare_code9(scratch));
    const size_t cap_code = scratch.sampled_code.capacity();
    const size_t cap_avg = scratch.avg.capacity();
    const size_t cap_raw = scratch.raw_taps.capacity();
    const size_t cap_norm = scratch.norm_taps.capacity();
    const size_t sz_code = scratch.sampled_code.size();
    const size_t sz_avg = scratch.avg.size();
    const size_t sz_raw = scratch.raw_taps.size();
    const size_t sz_norm = scratch.norm_taps.size();

    for (int i = 0; i < 8; ++i) {
        RadarCirEstimate out;
        BOOST_REQUIRE(run_cir(rx, meta.origin_clean, kRadarPre, kRadarPost,
                              kSkip, kMaxRep, kNSync, out, scratch));
        BOOST_CHECK_EQUAL(scratch.sampled_code.capacity(), cap_code);
        BOOST_CHECK_EQUAL(scratch.avg.capacity(), cap_avg);
        BOOST_CHECK_EQUAL(scratch.raw_taps.capacity(), cap_raw);
        BOOST_CHECK_EQUAL(scratch.norm_taps.capacity(), cap_norm);
        BOOST_CHECK_EQUAL(scratch.sampled_code.size(), sz_code);
        BOOST_CHECK_EQUAL(scratch.avg.size(), sz_avg);
        BOOST_CHECK_EQUAL(scratch.raw_taps.size(), sz_raw);
        BOOST_CHECK_EQUAL(scratch.norm_taps.size(), sz_norm);
    }
}

BOOST_AUTO_TEST_CASE(test_radar_cir_optional_8_30)
{
    const auto meta = load_canonical_meta();
    const auto rx = load_radar_cf32("rx_clean_998p4.cf32", meta.rx_len);
    std::vector<gr_complex> g_raw, g_norm;
    const std::string raw_path =
        testdata_path("uwb_radar/cir_raw_clean_8_30.cf32");
    const std::string norm_path =
        testdata_path("uwb_radar/cir_norm_clean_8_30.cf32");
    BOOST_REQUIRE(load_cf32(raw_path, g_raw));
    BOOST_REQUIRE(load_cf32(norm_path, g_norm));
    BOOST_REQUIRE_EQUAL(g_raw.size(), kDemodPre + kDemodPost);
    BOOST_REQUIRE_EQUAL(g_norm.size(), kDemodPre + kDemodPost);

    double peak830 = 0.0;
    BOOST_REQUIRE(parse_nested_number(meta.json, "demod_8_30",
                                      "peak_tap_measured_clean", peak830));

    RadarCirScratch scratch;
    BOOST_REQUIRE(prepare_code9(scratch, kRadarPre, kRadarPost));
    RadarCirEstimate out;
    BOOST_REQUIRE(run_cir(rx, meta.origin_clean, kDemodPre, kDemodPost, kSkip,
                          kMaxRep, kNSync, out, scratch));
    BOOST_CHECK_EQUAL(out.tap_count, kDemodPre + kDemodPost);
    BOOST_CHECK_EQUAL(out.peak_tap, static_cast<size_t>(std::llround(peak830)));
    const double e_raw =
        relative_l2(scratch.raw_taps.data(), g_raw.data(), g_raw.size());
    const double e_norm =
        relative_l2(scratch.norm_taps.data(), g_norm.data(), g_norm.size());
    BOOST_CHECK_MESSAGE(e_raw < 1e-5,
                        "8/30 raw relative L2=" + std::to_string(e_raw));
    BOOST_CHECK_MESSAGE(e_norm < 1e-5,
                        "8/30 norm relative L2=" + std::to_string(e_norm));
}

void shift_taps(const std::vector<gr_complex>& src,
                int64_t delay,
                gr_complex gain,
                std::vector<gr_complex>& dst)
{
    dst.assign(src.size(), gr_complex(0.f, 0.f));
    for (size_t k = 0; k < src.size(); ++k) {
        const int64_t s = static_cast<int64_t>(k) - delay;
        if (s >= 0 && s < static_cast<int64_t>(src.size()))
            dst[k] = gain * src[static_cast<size_t>(s)];
    }
}

void l2_normalize(std::vector<gr_complex>& x)
{
    double n2 = 0.0;
    for (const auto& v : x)
        n2 += std::norm(v);
    const float n = static_cast<float>(std::sqrt(n2));
    BOOST_REQUIRE(n > 0.f);
    const float inv = 1.f / (n + 1e-12f);
    for (auto& v : x)
        v *= inv;
}

BOOST_AUTO_TEST_CASE(test_radar_cir_delay0_delay1_vs_matlab_clean)
{
    const auto meta = load_canonical_meta();
    const auto rx0 = load_radar_cf32("rx_clean_998p4.cf32", meta.rx_len);
    const auto g_raw = load_radar_cf32("cir_raw_clean_radar.cf32", kRadarTaps);
    const auto g_norm = load_radar_cf32("cir_norm_clean_radar.cf32", kRadarTaps);

    RadarCirScratch scratch;
    BOOST_REQUIRE(prepare_code9(scratch));

    RadarCirEstimate d0;
    BOOST_REQUIRE(run_cir(rx0, meta.origin_clean, kRadarPre, kRadarPost, kSkip,
                          kMaxRep, kNSync, d0, scratch));
    BOOST_CHECK_EQUAL(d0.peak_tap, meta.peak_clean);
    BOOST_CHECK_LT(relative_l2(scratch.raw_taps.data(), g_raw.data(),
                               kRadarTaps),
                   1e-5);

    auto rx1 = shift_scale(rx0, 1, gr_complex(1.f, 0.f));
    RadarCirEstimate d1;
    BOOST_REQUIRE(run_cir(rx1, meta.origin_clean, kRadarPre, kRadarPost, kSkip,
                          kMaxRep, kNSync, d1, scratch));
    BOOST_CHECK_EQUAL(d1.peak_tap, meta.peak_clean + 1);

    std::vector<gr_complex> exp_raw, exp_norm;
    shift_taps(g_raw, 1, gr_complex(1.f, 0.f), exp_raw);
    exp_norm = exp_raw;
    l2_normalize(exp_norm);
    BOOST_CHECK_LT(relative_l2(scratch.raw_taps.data(), exp_raw.data(),
                               kRadarTaps),
                   1e-4);
    BOOST_CHECK_LT(relative_l2(scratch.norm_taps.data(), exp_norm.data(),
                               kRadarTaps),
                   1e-4);
}

BOOST_AUTO_TEST_CASE(test_radar_cir_multipath_vs_matlab_clean_taps)
{
    const auto meta = load_canonical_meta();
    const auto rx0 = load_radar_cf32("rx_clean_998p4.cf32", meta.rx_len);
    const auto g_raw = load_radar_cf32("cir_raw_clean_radar.cf32", kRadarTaps);

    const int64_t delays[] = { 0, 25, 50 };
    const gr_complex gains[] = {
        gr_complex(0.30f, 0.00f),
        gr_complex(1.00f, 0.00f),
        gr_complex(0.25f, 0.40f),
    };
    std::vector<gr_complex> rx(rx0.size(), gr_complex(0.f, 0.f));
    std::vector<gr_complex> exp_raw(kRadarTaps, gr_complex(0.f, 0.f));
    for (size_t p = 0; p < 3; ++p) {
        auto one = shift_scale(rx0, delays[p], gains[p]);
        for (size_t i = 0; i < rx.size(); ++i)
            rx[i] += one[i];
        std::vector<gr_complex> sh;
        shift_taps(g_raw, delays[p], gains[p], sh);
        for (size_t i = 0; i < kRadarTaps; ++i)
            exp_raw[i] += sh[i];
    }
    auto exp_norm = exp_raw;
    l2_normalize(exp_norm);

    RadarCirScratch scratch;
    BOOST_REQUIRE(prepare_code9(scratch));
    RadarCirEstimate out;
    BOOST_REQUIRE(run_cir(rx, meta.origin_clean, kRadarPre, kRadarPost, kSkip,
                          kMaxRep, kNSync, out, scratch));
    BOOST_CHECK_EQUAL(out.peak_tap, meta.peak_clean + 25);
    BOOST_CHECK_LT(relative_l2(scratch.raw_taps.data(), exp_raw.data(),
                               kRadarTaps),
                   1e-4);
    BOOST_CHECK_LT(relative_l2(scratch.norm_taps.data(), exp_norm.data(),
                               kRadarTaps),
                   1e-4);

    const gr_complex g1 = scratch.raw_taps[meta.peak_clean + 25];
    const gr_complex g0 = scratch.raw_taps[meta.peak_clean + 0];
    const gr_complex g2 = scratch.raw_taps[meta.peak_clean + 50];
    BOOST_CHECK_GT(std::abs(g1), std::abs(g0));
    BOOST_CHECK_GT(std::abs(g1), std::abs(g2));
    const gr_complex r0 = g0 / g1;
    const gr_complex r2 = g2 / g1;
    BOOST_CHECK_LT(std::abs(r0 - gr_complex(0.30f, 0.f)), 0.05f);
    BOOST_CHECK_LT(std::abs(r2 - gr_complex(0.25f, 0.40f)), 0.08f);
}

BOOST_AUTO_TEST_CASE(test_radar_cir_tap_window_edges)
{
    const auto meta = load_canonical_meta();
    const auto rx0 = load_radar_cf32("rx_clean_998p4.cf32", meta.rx_len);
    const auto g_raw = load_radar_cf32("cir_raw_clean_radar.cf32", kRadarTaps);
    const int64_t peak = static_cast<int64_t>(meta.peak_clean);
    const int64_t last = static_cast<int64_t>(kRadarTaps - 1);

    RadarCirScratch scratch;
    BOOST_REQUIRE(prepare_code9(scratch));

    auto check_delay = [&](int64_t d, size_t expect_peak, const char* label) {
        auto rx = shift_scale(rx0, d, gr_complex(1.f, 0.f));
        RadarCirEstimate out;
        BOOST_REQUIRE_MESSAGE(
            run_cir(rx, meta.origin_clean, kRadarPre, kRadarPost, kSkip,
                    kMaxRep, kNSync, out, scratch),
            label);
        BOOST_CHECK_MESSAGE(out.peak_tap == expect_peak, label);
        std::vector<gr_complex> exp;
        shift_taps(g_raw, d, gr_complex(1.f, 0.f), exp);
        const double e =
            relative_l2(scratch.raw_taps.data(), exp.data(), kRadarTaps);
        BOOST_CHECK_MESSAGE(e < 5e-3, std::string(label) + " L2=" +
                                          std::to_string(e));
    };

    check_delay(-peak, 0, "first legal tap");
    check_delay(last - peak, static_cast<size_t>(last), "last legal tap");

    auto rx_before = shift_scale(rx0, -peak - 1, gr_complex(1.f, 0.f));
    RadarCirEstimate before;
    BOOST_REQUIRE(run_cir(rx_before, meta.origin_clean, kRadarPre, kRadarPost,
                          kSkip, kMaxRep, kNSync, before, scratch));
    BOOST_CHECK_EQUAL(before.tap_count, kRadarTaps);
    BOOST_CHECK_EQUAL(before.peak_tap, size_t(0));
    std::vector<gr_complex> exp_before;
    shift_taps(g_raw, -peak - 1, gr_complex(1.f, 0.f), exp_before);
    BOOST_CHECK_LT(relative_l2(scratch.raw_taps.data(), exp_before.data(),
                               kRadarTaps),
                   5e-3);

    auto rx_after = shift_scale(rx0, last - peak + 1, gr_complex(1.f, 0.f));
    RadarCirEstimate after;
    BOOST_REQUIRE(run_cir(rx_after, meta.origin_clean, kRadarPre, kRadarPost,
                          kSkip, kMaxRep, kNSync, after, scratch));
    BOOST_CHECK_EQUAL(after.tap_count, kRadarTaps);
    BOOST_CHECK_EQUAL(after.peak_tap, static_cast<size_t>(last));
    std::vector<gr_complex> exp_after;
    shift_taps(g_raw, last - peak + 1, gr_complex(1.f, 0.f), exp_after);
    BOOST_CHECK_LT(relative_l2(scratch.raw_taps.data(), exp_after.data(),
                               kRadarTaps),
                   5e-3);
}

BOOST_AUTO_TEST_CASE(test_radar_cir_pre0_post1_and_prepare_bound)
{
    const auto meta = load_canonical_meta();
    const auto rx = load_radar_cf32("rx_clean_998p4.cf32", meta.rx_len);

    RadarCirScratch tiny;
    BOOST_REQUIRE(prepare_code9(tiny, 0, 1));
    RadarCirEstimate one;
    BOOST_REQUIRE(run_cir(rx, meta.origin_clean, 0, 1, kSkip, kMaxRep, kNSync,
                          one, tiny));
    BOOST_CHECK_EQUAL(one.tap_count, size_t(1));
    BOOST_CHECK_EQUAL(one.peak_tap, size_t(0));
    BOOST_CHECK_GT(one.raw_l2_norm, 0.f);

    RadarCirScratch cap;
    BOOST_REQUIRE(prepare_code9(cap, kRadarPre, kRadarPost));
    RadarCirEstimate too_wide;
    BOOST_REQUIRE(!run_cir(rx, meta.origin_clean, kRadarPre, kRadarPost + 1,
                           kSkip, kMaxRep, kNSync, too_wide, cap));
    BOOST_CHECK(too_wide.status == CirStatus::InvalidInput);
    BOOST_CHECK_EQUAL(too_wide.tap_count, size_t(0));
}
