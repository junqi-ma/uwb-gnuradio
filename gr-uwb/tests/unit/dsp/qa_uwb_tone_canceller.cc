/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * QA for header-only tone canceller and one-shot 65/48 (offline path).
 */

#include <boost/test/unit_test.hpp>

#include <gnuradio/uwb/uwb_rational_resampler_core.h>
#include <gnuradio/uwb/uwb_tone_canceller.h>

#include <cmath>
#include <complex>
#include <fstream>
#include <string>
#include <vector>

using gr::uwb::core::correlate_cis;
using gr::uwb::core::estimate_tone;
using gr::uwb::core::RationalResampler65_48Core;
using gr::uwb::core::subtract_tone;
using gr::uwb::core::subtract_tone_sc16;

namespace {

constexpr double kFs = 737.28e6;
constexpr double kTone = -232.96e6; // 6256.640 − 6489.6 MHz

std::string find_path(const std::string& rel)
{
    const char* prefixes[] = {
        "", "../", "../../", "../../../", "../../../../",
    };
    for (const char* p : prefixes) {
        const std::string path = std::string(p) + rel;
        std::ifstream f(path, std::ios::binary);
        if (f)
            return path;
    }
    return rel;
}

std::vector<float> load_f32(const std::string& path)
{
    std::ifstream f(path, std::ios::binary | std::ios::ate);
    BOOST_REQUIRE_MESSAGE(f, "cannot open " + path);
    const auto bytes = static_cast<size_t>(f.tellg());
    f.seekg(0);
    std::vector<float> v(bytes / sizeof(float));
    f.read(reinterpret_cast<char*>(v.data()),
           static_cast<std::streamsize>(bytes));
    return v;
}

std::vector<std::complex<double>> tone_vec(size_t n, double fs, double f,
                                           std::complex<double> coef)
{
    std::vector<std::complex<double>> x(n);
    const double w = 2.0 * M_PI * f / fs;
    for (size_t k = 0; k < n; ++k) {
        const double ph = w * static_cast<double>(k);
        x[k] = coef * std::complex<double>(std::cos(ph), std::sin(ph));
    }
    return x;
}

} // namespace

BOOST_AUTO_TEST_CASE(test_estimate_known_cw)
{
    const size_t n = 16384;
    const auto coef = std::complex<double>(120.0, -35.0);
    auto x = tone_vec(n, kFs, kTone, coef);
    const double lo = kTone - 80e6;
    const double hi = kTone + 80e6;
    const auto est = estimate_tone(x.data(), n, kFs, lo, hi);
    // Last refine grid is ~140 Hz at n=16384; 200 Hz is within that bin.
    BOOST_CHECK_SMALL(est.baseband_hz - kTone, 200.0);
}

BOOST_AUTO_TEST_CASE(test_phasor_recurrence_matches_sincos)
{
    const size_t n = 8192;
    auto x = tone_vec(n, kFs, kTone, { 80.0, 20.0 });
    const double w = -2.0 * M_PI * kTone / kFs;
    std::complex<double> brute(0.0, 0.0);
    for (size_t k = 0; k < n; ++k) {
        const double ph = w * static_cast<double>(k);
        brute += x[k] * std::complex<double>(std::cos(ph), std::sin(ph));
    }
    const auto rec = correlate_cis(x.data(), n, w);
    BOOST_REQUIRE(std::abs(brute) > 1.0);
    BOOST_CHECK_SMALL(std::abs(rec - brute) / std::abs(brute), 1e-9);
}

BOOST_AUTO_TEST_CASE(test_subtract_suppresses_bin)
{
    const size_t n = 4096;
    const auto coef = std::complex<double>(80.0, 20.0);
    auto x = tone_vec(n, kFs, kTone, coef);
    std::vector<std::complex<double>> y(n);
    const auto r = subtract_tone(y.data(), x.data(), n, kFs, kTone);
    BOOST_CHECK_CLOSE(r.bin_before, std::abs(coef), 1e-6);
    BOOST_CHECK_SMALL(r.bin_after, 1e-6);
    BOOST_CHECK_GT(r.power_before, r.power_after);
    double resid = 0.0;
    for (auto z : y)
        resid = std::max(resid, std::abs(z));
    BOOST_CHECK_SMALL(resid, 1e-6);
}

BOOST_AUTO_TEST_CASE(test_n_from_window_start_not_abs)
{
    // Phase is referenced to n=0 of this window, not absolute sample index.
    const size_t n = 2048;
    const double f = -1.0e6;
    auto x = tone_vec(n, kFs, f, { 50.0, 0.0 });
    std::vector<std::complex<double>> y(n);
    const auto r = subtract_tone(y.data(), x.data(), n, kFs, f);
    BOOST_CHECK_SMALL(std::abs(r.coef - std::complex<double>(50.0, 0.0)), 1e-6);
}

BOOST_AUTO_TEST_CASE(test_sc16_clip_and_roundtrip)
{
    const size_t n = 256;
    std::vector<int16_t> x(n * 2, 0);
    std::vector<int16_t> y(n * 2, 0);
    const double f = 1.0e6;
    for (size_t k = 0; k < n; ++k) {
        const double ph = 2.0 * M_PI * f * static_cast<double>(k) / kFs;
        x[2 * k] = static_cast<int16_t>(std::lround(400.0 * std::cos(ph)));
        x[2 * k + 1] = static_cast<int16_t>(std::lround(400.0 * std::sin(ph)));
    }
    const auto r = subtract_tone_sc16(y.data(), x.data(), n, kFs, f);
    BOOST_CHECK_GT(r.bin_before, 300.0);
    BOOST_CHECK_LT(r.bin_after, 2.0);
    BOOST_CHECK_EQUAL(r.clip_count, 0u);
}

BOOST_AUTO_TEST_CASE(test_oneshot_65_48_matches_core)
{
    const auto taps = load_f32(find_path(
        "testdata/resampler_65_48/taps_quality_minorder.txt"));
    BOOST_REQUIRE(!taps.empty());
    std::vector<std::complex<float>> x(1024);
    for (size_t i = 0; i < x.size(); ++i) {
        const float t = static_cast<float>(i) * 0.01f;
        x[i] = std::complex<float>(std::cos(t), std::sin(t * 1.3f));
    }
    RationalResampler65_48Core a(taps);
    RationalResampler65_48Core b(taps);
    const size_t Lout =
        RationalResampler65_48Core::expected_output_length(x.size(),
                                                           taps.size());
    std::vector<std::complex<float>> ya(Lout + 64), yb(Lout + 64);
    size_t pa = a.process(x.data(), x.size(), ya.data(), ya.size()).produced;
    size_t pb = b.process(x.data(), x.size(), yb.data(), yb.size()).produced;
    while (pa < Lout) {
        const size_t n = a.flush(ya.data() + pa, ya.size() - pa);
        if (n == 0)
            break;
        pa += n;
    }
    while (pb < Lout) {
        const size_t n = b.flush(yb.data() + pb, yb.size() - pb);
        if (n == 0)
            break;
        pb += n;
    }
    BOOST_REQUIRE_EQUAL(pa, pb);
    float md = 0.0f;
    for (size_t i = 0; i < pa; ++i)
        md = std::max(md, std::abs(ya[i] - yb[i]));
    BOOST_CHECK_LE(md, 1e-6f);
    BOOST_CHECK_EQUAL(
        pa, RationalResampler65_48Core::expected_output_length(x.size(),
                                                               taps.size()));
}

BOOST_AUTO_TEST_CASE(test_map_formula)
{
    const auto taps = load_f32(find_path(
        "testdata/resampler_65_48/taps_quality_minorder.txt"));
    RationalResampler65_48Core core(taps);
    const int64_t p = 1000000;
    const int64_t got = core.map_input_offset_to_output(p);
    const double d = 0.5 * static_cast<double>(taps.size() - 1);
    const int64_t expect =
        static_cast<int64_t>(std::llround((p * 65.0 + d) / 48.0));
    BOOST_CHECK_EQUAL(got, expect);
}
