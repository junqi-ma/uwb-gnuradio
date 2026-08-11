/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Core numeric QA for RationalResampler65_48Core vs upfirdn golden.
 *
 * Golden: testdata/resampler_65_48/golden/<name>_in/out.cf32
 * Taps:   testdata/resampler_65_48/taps_quality.txt (T=5363)
 *
 * Tolerance justification (float32 golden export, quality T=5363, H=83):
 *   Each output is a sum of H real*complex MACs.  float32 accumulation and
 *   golden fwrite round-trip set a practical abs floor around 1e-3..2e-3 on
 *   unit-scale tones.  We use tol_abs = 2e-3 as the contract ceiling and
 *   report max_abs / relative_L2 / corr / gain / phase for every golden.
 */

#include <boost/test/unit_test.hpp>

#include <gnuradio/uwb/uwb_rational_resampler_core.h>

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

using gr::uwb::core::RationalResampler65_48Core;
using gr_complex = std::complex<float>;

namespace {

// Contract abs tolerance for quality taps vs float32 golden (see file header).
constexpr float kTolAbs = 2e-3f;

std::string find_path(const std::string& rel)
{
    // CTest CWD is typically gr-uwb/build; also allow gr-uwb/build/lib.
    const char* prefixes[] = {
        "",
        "../",
        "../../",
        "../../../",
        "../../../../",
        // Absolute-ish from repo conventions used by other QA files
        "../../../testdata/resampler_65_48/",
    };
    // If rel already contains the testdata prefix, try as-is under each root.
    for (const char* p : prefixes) {
        const std::string path = std::string(p) + rel;
        std::ifstream f(path, std::ios::binary);
        if (f)
            return path;
    }
    return rel;
}

/** Resolve a golden pair base path ending with e.g. .../impulse (no suffix). */
std::string find_golden_base(const char* name)
{
    const std::string rel =
        std::string("testdata/resampler_65_48/golden/") + name + "_in.cf32";
    const std::string full = find_path(rel);
    // Strip "_in.cf32"
    const std::string suffix = "_in.cf32";
    if (full.size() > suffix.size() &&
        full.compare(full.size() - suffix.size(), suffix.size(), suffix) == 0)
        return full.substr(0, full.size() - suffix.size());
    return std::string("testdata/resampler_65_48/golden/") + name;
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

struct ErrMetrics {
    float max_abs = 0.0f;
    double relative_L2 = 0.0;
    double corr = 0.0;
    double gain_error = 0.0;
    double phase_error_rad = 0.0;
    size_t n = 0;
};

ErrMetrics compare_iq(const std::vector<gr_complex>& y,
                      const std::vector<gr_complex>& ref)
{
    ErrMetrics m;
    BOOST_REQUIRE_EQUAL(y.size(), ref.size());
    m.n = y.size();
    if (m.n == 0)
        return m;

    double sum_err2 = 0.0;
    double sum_ref2 = 0.0;
    double sum_y2 = 0.0;
    double sum_re_cross = 0.0;
    double sum_im_cross = 0.0; // for complex correlation
    // Complex correlation: sum y * conj(ref) / norms
    std::complex<double> cross(0.0, 0.0);
    std::complex<double> sum_y(0.0, 0.0);
    std::complex<double> sum_r(0.0, 0.0);

    for (size_t i = 0; i < m.n; ++i) {
        const std::complex<double> a(y[i].real(), y[i].imag());
        const std::complex<double> b(ref[i].real(), ref[i].imag());
        const double e = std::abs(a - b);
        if (e > m.max_abs)
            m.max_abs = static_cast<float>(e);
        sum_err2 += e * e;
        sum_ref2 += std::norm(b);
        sum_y2 += std::norm(a);
        cross += a * std::conj(b);
        sum_y += a;
        sum_r += b;
    }

    m.relative_L2 =
        (sum_ref2 > 0.0) ? std::sqrt(sum_err2 / sum_ref2) : std::sqrt(sum_err2);
    const double denom = std::sqrt(sum_y2 * sum_ref2);
    m.corr = (denom > 0.0) ? (std::abs(cross) / denom) : 1.0;

    // Steady-state gain/phase from mean ratio on high-energy samples.
    std::complex<double> ratio_acc(0.0, 0.0);
    size_t ratio_n = 0;
    for (size_t i = 0; i < m.n; ++i) {
        const double rn = std::norm(ref[i]);
        if (rn < 1e-8)
            continue;
        const std::complex<double> a(y[i].real(), y[i].imag());
        const std::complex<double> b(ref[i].real(), ref[i].imag());
        ratio_acc += a / b;
        ++ratio_n;
    }
    if (ratio_n > 0) {
        const auto mean_r = ratio_acc / static_cast<double>(ratio_n);
        m.gain_error = std::abs(mean_r) - 1.0;
        m.phase_error_rad = std::arg(mean_r);
    }
    (void)sum_re_cross;
    (void)sum_im_cross;
    return m;
}

std::vector<gr_complex> run_core_full(RationalResampler65_48Core& core,
                                      const std::vector<gr_complex>& x)
{
    const size_t Lout =
        RationalResampler65_48Core::expected_output_length(x.size(),
                                                           core.tap_count());
    std::vector<gr_complex> y(Lout + 64, gr_complex(0, 0));
    size_t produced = 0;
    if (!x.empty()) {
        auto r = core.process(x.data(), x.size(), y.data(), y.size());
        produced = r.produced;
        BOOST_CHECK_EQUAL(r.consumed, x.size());
    }
    // Flush tail
    while (true) {
        if (produced >= y.size())
            y.resize(produced + 256);
        const size_t n =
            core.flush(y.data() + produced, y.size() - produced);
        if (n == 0)
            break;
        produced += n;
    }
    y.resize(produced);
    return y;
}

std::vector<float> g_taps;
bool g_taps_loaded = false;

const std::vector<float>& quality_taps()
{
    if (!g_taps_loaded) {
        g_taps = load_f32(find_path("testdata/resampler_65_48/taps_quality.txt"));
        g_taps_loaded = true;
        BOOST_REQUIRE_EQUAL(g_taps.size(), 5363u);
    }
    return g_taps;
}

void check_golden(const char* name)
{
    const auto& taps = quality_taps();
    RationalResampler65_48Core core(taps);

    const std::string base = find_golden_base(name);
    const auto x = load_cf32(base + "_in.cf32");
    const auto ref = load_cf32(base + "_out.cf32");

    const size_t expect_len =
        RationalResampler65_48Core::expected_output_length(x.size(),
                                                           taps.size());
    BOOST_CHECK_EQUAL(ref.size(), expect_len);

    auto y = run_core_full(core, x);
    BOOST_CHECK_EQUAL(y.size(), ref.size());

    const auto m = compare_iq(y, ref);
    std::cout << "golden[" << name << "] N_in=" << x.size()
              << " N_out=" << y.size() << " max_abs=" << m.max_abs
              << " rel_L2=" << m.relative_L2 << " corr=" << m.corr
              << " gain_err=" << m.gain_error
              << " phase_err_rad=" << m.phase_error_rad << "\n";

    BOOST_CHECK_MESSAGE(m.max_abs <= kTolAbs,
                        "max_abs " + std::to_string(m.max_abs) + " > " +
                            std::to_string(kTolAbs));
    BOOST_CHECK_MESSAGE(m.corr >= 0.999,
                        "correlation " + std::to_string(m.corr) + " < 0.999");
    BOOST_CHECK_MESSAGE(std::fabs(m.gain_error) < 0.02,
                        "gain_error " + std::to_string(m.gain_error));
    BOOST_CHECK_MESSAGE(std::fabs(m.phase_error_rad) < 0.05,
                        "phase_error " + std::to_string(m.phase_error_rad));
}

} // namespace

BOOST_AUTO_TEST_CASE(test_core_expected_length_formula)
{
    const auto& taps = quality_taps();
    const size_t T = taps.size();
    // N=0 → ceil((T-65)/48) = 111 for T=5363
    BOOST_CHECK_EQUAL(
        RationalResampler65_48Core::expected_output_length(0, T), 111u);
    BOOST_CHECK_EQUAL(
        RationalResampler65_48Core::expected_output_length(1, T), 112u);
    BOOST_CHECK_EQUAL(
        RationalResampler65_48Core::expected_output_length(2, T), 114u);
    BOOST_CHECK_EQUAL(
        RationalResampler65_48Core::expected_output_length(47, T), 175u);
    BOOST_CHECK_EQUAL(
        RationalResampler65_48Core::expected_output_length(48, T), 176u);
    BOOST_CHECK_EQUAL(
        RationalResampler65_48Core::expected_output_length(49, T), 177u);
    BOOST_CHECK_EQUAL(
        RationalResampler65_48Core::expected_output_length(95, T), 240u);
    BOOST_CHECK_EQUAL(
        RationalResampler65_48Core::expected_output_length(96, T), 241u);
    BOOST_CHECK_EQUAL(
        RationalResampler65_48Core::expected_output_length(4096, T), 5658u);
}

BOOST_AUTO_TEST_CASE(test_core_zero_input)
{
    const auto& taps = quality_taps();
    RationalResampler65_48Core core(taps);
    std::vector<gr_complex> empty;
    auto y = run_core_full(core, empty);
    const size_t expect =
        RationalResampler65_48Core::expected_output_length(0, taps.size());
    BOOST_CHECK_EQUAL(y.size(), expect);
    for (auto s : y) {
        BOOST_CHECK_EQUAL(s.real(), 0.0f);
        BOOST_CHECK_EQUAL(s.imag(), 0.0f);
        BOOST_CHECK(!std::isnan(s.real()));
        BOOST_CHECK(!std::isnan(s.imag()));
    }
}

BOOST_AUTO_TEST_CASE(test_core_short_inputs_length)
{
    const auto& taps = quality_taps();
    const size_t Ns[] = { 0, 1, 47, 48, 49, 95, 96 };
    for (size_t N : Ns) {
        RationalResampler65_48Core core(taps);
        std::vector<gr_complex> x(N, gr_complex(0, 0));
        if (N > 0)
            x[0] = gr_complex(1, 0);
        auto y = run_core_full(core, x);
        const size_t expect =
            RationalResampler65_48Core::expected_output_length(N, taps.size());
        BOOST_CHECK_MESSAGE(y.size() == expect,
                            "N=" + std::to_string(N) + " got " +
                                std::to_string(y.size()) + " expect " +
                                std::to_string(expect));
        BOOST_CHECK_EQUAL(core.input_items(), N);
        BOOST_CHECK_EQUAL(core.output_items(), y.size());
    }
}

BOOST_AUTO_TEST_CASE(test_core_golden_impulse)
{
    check_golden("impulse");
}
BOOST_AUTO_TEST_CASE(test_core_golden_dc) { check_golden("dc"); }
BOOST_AUTO_TEST_CASE(test_core_golden_tone_low)
{
    check_golden("tone_low");
}
BOOST_AUTO_TEST_CASE(test_core_golden_tone_pb)
{
    check_golden("tone_pb");
}
BOOST_AUTO_TEST_CASE(test_core_golden_tone_sb)
{
    check_golden("tone_sb");
}
BOOST_AUTO_TEST_CASE(test_core_golden_random)
{
    check_golden("random");
}
BOOST_AUTO_TEST_CASE(test_core_golden_uwb) { check_golden("uwb"); }

BOOST_AUTO_TEST_CASE(test_core_long_count_no_phase_drift)
{
    const auto& taps = quality_taps();
    RationalResampler65_48Core core(taps);

    // Many small chunks; phase must stay in [0,65) and counts must match.
    const size_t N = 48 * 200; // 9600
    std::vector<gr_complex> x(N);
    for (size_t i = 0; i < N; ++i)
        x[i] = gr_complex(std::sin(0.01f * i), std::cos(0.013f * i));

    std::vector<gr_complex> y;
    y.reserve(N * 2);
    size_t off = 0;
    while (off < N) {
        const size_t ch = std::min<size_t>(48, N - off);
        gr_complex tmp[128];
        auto r = core.process(x.data() + off, ch, tmp, 128);
        BOOST_CHECK_LE(core.phase(), 64u);
        for (size_t i = 0; i < r.produced; ++i)
            y.push_back(tmp[i]);
        BOOST_CHECK_EQUAL(r.consumed, ch);
        off += r.consumed;
    }
    gr_complex tail[4096];
    size_t ntail;
    do {
        ntail = core.flush(tail, 4096);
        for (size_t i = 0; i < ntail; ++i)
            y.push_back(tail[i]);
    } while (ntail > 0);

    const size_t expect =
        RationalResampler65_48Core::expected_output_length(N, taps.size());
    BOOST_CHECK_EQUAL(y.size(), expect);
    BOOST_CHECK_EQUAL(core.input_items(), N);
    BOOST_CHECK_EQUAL(core.output_items(), expect);
    BOOST_CHECK_LT(core.phase(), 65u);
}

BOOST_AUTO_TEST_CASE(test_core_reset_clears_state)
{
    const auto& taps = quality_taps();
    RationalResampler65_48Core core(taps);
    std::vector<gr_complex> x(100, gr_complex(1, 0));
    gr_complex out[256];
    (void)core.process(x.data(), x.size(), out, 256);
    BOOST_CHECK_GT(core.input_items(), 0u);
    core.reset();
    BOOST_CHECK_EQUAL(core.input_items(), 0u);
    BOOST_CHECK_EQUAL(core.output_items(), 0u);
    BOOST_CHECK_EQUAL(core.phase(), 0u);
    BOOST_CHECK_EQUAL(core.resets(), 1u);
}

BOOST_AUTO_TEST_CASE(test_core_arm_split_and_dc_sum)
{
    const auto& taps = quality_taps();
    RationalResampler65_48Core core(taps);
    BOOST_CHECK_EQUAL(core.arm_length(), 83u);
    double sum = 0.0;
    for (float t : taps)
        sum += t;
    BOOST_CHECK_SMALL(sum - 65.0, 1e-3);
    // Arm 0 first tap equals taps[0]
    BOOST_CHECK_EQUAL(core.arm_tap(0, 0), taps[0]);
    BOOST_CHECK_EQUAL(core.arm_tap(1, 0), taps[1]);
}

BOOST_AUTO_TEST_CASE(test_core_kernel_name_and_scalar_match)
{
    const auto& taps = quality_taps();
    RationalResampler65_48Core core_def(taps);
    const std::string kn = core_def.kernel_name();
    BOOST_CHECK(kn.find("volk") != std::string::npos ||
                kn.find("scalar") != std::string::npos ||
                kn.find("avx2") != std::string::npos);

    // Default (AVX2/VOLK macroblock) vs forced scalar — sample-exact within ulps.
    const auto x = load_cf32(find_golden_base("impulse") + "_in.cf32");
    RationalResampler65_48Core core_s(taps);
    core_s.force_scalar_kernel();
    BOOST_CHECK(std::string(core_s.kernel_name()).find("scalar") !=
                std::string::npos);

    auto y_def = run_core_full(core_def, x);
    core_s.reset();
    core_s.force_scalar_kernel();
    auto y_s = run_core_full(core_s, x);
    BOOST_REQUIRE_EQUAL(y_def.size(), y_s.size());
    float max_e = 0.0f;
    for (size_t i = 0; i < y_def.size(); ++i)
        max_e = std::max(max_e, std::abs(y_def[i] - y_s[i]));
    BOOST_CHECK_MESSAGE(max_e < 1e-4f,
                        "default vs scalar max_abs=" + std::to_string(max_e));

    // Macroblock vs legacy loop, same FIR kernel — bitwise preferred.
    RationalResampler65_48Core core_leg(taps);
    core_leg.set_kernel("volk_legacy");
    RationalResampler65_48Core core_mb(taps);
    core_mb.set_kernel("volk_macroblock");
    auto y_leg = run_core_full(core_leg, x);
    auto y_mb = run_core_full(core_mb, x);
    BOOST_REQUIRE_EQUAL(y_leg.size(), y_mb.size());
    float max_mb = 0.0f;
    for (size_t i = 0; i < y_leg.size(); ++i)
        max_mb = std::max(max_mb, std::abs(y_leg[i] - y_mb[i]));
    BOOST_CHECK_MESSAGE(max_mb == 0.0f,
                        "legacy vs macroblock max_abs=" +
                            std::to_string(max_mb));

    // Multi-worker vs single-thread — same outputs.
    RationalResampler65_48Core core_w1(taps);
    core_w1.set_num_workers(1);
    RationalResampler65_48Core core_w4(taps);
    core_w4.set_num_workers(4);
    auto y_w1 = run_core_full(core_w1, x);
    auto y_w4 = run_core_full(core_w4, x);
    BOOST_REQUIRE_EQUAL(y_w1.size(), y_w4.size());
    float max_w = 0.0f;
    for (size_t i = 0; i < y_w1.size(); ++i)
        max_w = std::max(max_w, std::abs(y_w1[i] - y_w4[i]));
    BOOST_CHECK_MESSAGE(max_w == 0.0f,
                        "workers1 vs workers4 max_abs=" +
                            std::to_string(max_w));

    std::cout << "kernel_match default=" << kn << " max_abs(def,scalar)="
              << max_e << " max_abs(leg,mb)=" << max_mb
              << " max_abs(w1,w4)=" << max_w << "\n";
}

BOOST_AUTO_TEST_CASE(test_core_multiworker_chunked_and_profile)
{
    const auto& taps = quality_taps();
    const auto x = load_cf32(find_golden_base("random") + "_in.cf32");

    RationalResampler65_48Core ref(taps);
    ref.set_num_workers(1);
    auto y_ref = run_core_full(ref, x);

    // Chunked multi-worker
    RationalResampler65_48Core mt(taps);
    mt.set_num_workers(8);
    std::vector<gr_complex> y;
    y.reserve(y_ref.size());
    size_t off = 0;
    const size_t chunks[] = { 1, 47, 48, 49, 100, 4096 };
    size_t ci = 0;
    while (off < x.size()) {
        size_t ch = chunks[ci % 6];
        ++ci;
        ch = std::min(ch, x.size() - off);
        std::vector<gr_complex> tmp(ch * 2 + 256);
        auto r = mt.process(x.data() + off, ch, tmp.data(), tmp.size());
        for (size_t i = 0; i < r.produced; ++i)
            y.push_back(tmp[i]);
        BOOST_REQUIRE_EQUAL(r.consumed, ch);
        off += r.consumed;
    }
    std::vector<gr_complex> tail(4096);
    size_t n;
    do {
        n = mt.flush(tail.data(), tail.size());
        for (size_t i = 0; i < n; ++i)
            y.push_back(tail[i]);
    } while (n > 0);

    BOOST_REQUIRE_EQUAL(y.size(), y_ref.size());
    float max_e = 0.0f;
    for (size_t i = 0; i < y.size(); ++i)
        max_e = std::max(max_e, std::abs(y[i] - y_ref[i]));
    BOOST_CHECK_MESSAGE(max_e == 0.0f,
                        "chunked multiworker max_abs=" + std::to_string(max_e));

    // Profiling accumulates without changing results.
    RationalResampler65_48Core prof(taps);
    prof.enable_profiling(true);
    auto y_p = run_core_full(prof, x);
    BOOST_CHECK_EQUAL(y_p.size(), y_ref.size());
    BOOST_CHECK_GT(prof.profile_stats().calls, 0u);
    BOOST_CHECK_GT(prof.profile_stats().ns_total, 0.0);
    std::cout << "profile assemble_ns=" << prof.profile_stats().ns_assemble
              << " schedule_ns=" << prof.profile_stats().ns_schedule
              << " fir_ns=" << prof.profile_stats().ns_fir
              << " state_ns=" << prof.profile_stats().ns_state
              << " total_ns=" << prof.profile_stats().ns_total
              << " outs=" << prof.profile_stats().outputs
              << " mblocks=" << prof.profile_stats().macroblocks << "\n";
}
