/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Core numeric QA for RationalResampler65_32Core vs scipy upfirdn golden.
 */

#include <boost/test/unit_test.hpp>

#include <gnuradio/uwb/uwb_rational_resampler_core.h>

#include <cmath>
#include <complex>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

using gr::uwb::core::RationalResampler65_32Core;
using gr_complex = std::complex<float>;

namespace {

constexpr float kTolAbs = 2e-3f;

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
    BOOST_REQUIRE_EQUAL(bytes % sizeof(float), 0u);
    f.seekg(0);
    std::vector<float> v(bytes / sizeof(float));
    f.read(reinterpret_cast<char*>(v.data()),
           static_cast<std::streamsize>(bytes));
    BOOST_REQUIRE(f);
    return v;
}

std::vector<gr_complex> load_cf32(const std::string& path)
{
    auto f = load_f32(path);
    BOOST_REQUIRE_EQUAL(f.size() % 2, 0u);
    std::vector<gr_complex> v(f.size() / 2);
    for (size_t i = 0; i < v.size(); ++i)
        v[i] = gr_complex(f[2 * i], f[2 * i + 1]);
    return v;
}

std::vector<gr_complex> run_core_full(RationalResampler65_32Core& core,
                                      const std::vector<gr_complex>& x)
{
    const size_t Lout = RationalResampler65_32Core::expected_output_length(
        x.size(), core.tap_count());
    std::vector<gr_complex> y(Lout + 64, gr_complex(0, 0));
    size_t produced = 0;
    if (!x.empty()) {
        auto r = core.process(x.data(), x.size(), y.data(), y.size());
        produced = r.produced;
        BOOST_CHECK_EQUAL(r.consumed, x.size());
    }
    while (true) {
        if (produced >= y.size())
            y.resize(produced + 256);
        const size_t n = core.flush(y.data() + produced, y.size() - produced);
        if (n == 0)
            break;
        produced += n;
    }
    y.resize(produced);
    return y;
}

const std::vector<float>& minorder_taps()
{
    static std::vector<float> taps = load_f32(find_path(
        "testdata/resampler_65_32/taps_quality_minorder.txt"));
    return taps;
}

float max_abs_diff(const std::vector<gr_complex>& a,
                   const std::vector<gr_complex>& b)
{
    BOOST_REQUIRE_EQUAL(a.size(), b.size());
    float m = 0.f;
    for (size_t i = 0; i < a.size(); ++i) {
        const float e = std::abs(a[i] - b[i]);
        if (e > m)
            m = e;
    }
    return m;
}

void check_golden(const char* name)
{
    const auto& taps = minorder_taps();
    RationalResampler65_32Core core(taps);
    const std::string in_path = find_path(
        std::string("testdata/resampler_65_32/golden_quality_minorder/") +
        name + "_in.cf32");
    const std::string suffix = "_in.cf32";
    BOOST_REQUIRE(in_path.size() > suffix.size());
    const std::string base =
        in_path.substr(0, in_path.size() - suffix.size());
    const auto x = load_cf32(base + "_in.cf32");
    const auto ref = load_cf32(base + "_out.cf32");
    BOOST_CHECK_EQUAL(
        ref.size(),
        RationalResampler65_32Core::expected_output_length(x.size(),
                                                           taps.size()));
    auto y = run_core_full(core, x);
    const float max_abs = max_abs_diff(y, ref);
    std::cout << "65_32 golden[" << name << "] N_in=" << x.size()
              << " N_out=" << y.size() << " max_abs=" << max_abs << "\n";
    BOOST_CHECK_LE(max_abs, kTolAbs);
}

} // namespace

BOOST_AUTO_TEST_CASE(test_65_32_expected_length)
{
    const auto& taps = minorder_taps();
    const size_t T = taps.size();
    BOOST_CHECK_EQUAL(T, 2707u);
    BOOST_CHECK_EQUAL(
        RationalResampler65_32Core::expected_output_length(4096, T), 8403u);
}

BOOST_AUTO_TEST_CASE(test_65_32_golden_impulse)
{
    check_golden("impulse");
}

BOOST_AUTO_TEST_CASE(test_65_32_golden_dc)
{
    check_golden("dc");
}

BOOST_AUTO_TEST_CASE(test_65_32_golden_tone_low)
{
    check_golden("tone_low");
}

BOOST_AUTO_TEST_CASE(test_65_32_golden_random)
{
    check_golden("random");
}

BOOST_AUTO_TEST_CASE(test_65_32_scalar_matches_default)
{
    const auto& taps = minorder_taps();
    const auto x = load_cf32(find_path(
        "testdata/resampler_65_32/golden_quality_minorder/random_in.cf32"));
    RationalResampler65_32Core a(taps);
    RationalResampler65_32Core b(taps);
    a.force_scalar_kernel();
    auto ya = run_core_full(a, x);
    auto yb = run_core_full(b, x);
    BOOST_CHECK_LE(max_abs_diff(ya, yb), kTolAbs);
}
