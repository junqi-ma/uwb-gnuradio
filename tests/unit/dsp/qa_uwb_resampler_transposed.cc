/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * RS-5 QA: cross-macroblock transposed resampler vs an independent float64
 * upfirdn reference.
 *
 * The transposed kernel deliberately uses a different accumulation order from
 * the bit-exact macroblock kernels (one vector accumulator along the tap axis
 * for 4/8 macroblocks at once).  It is therefore gated against a reference
 * written from the upfirdn definition,
 *
 *     y[m] = sum_{k=0..H-1} taps[(m*M mod L) + L*k] * x[floor(m*M/L) - k]
 *
 * with x = 0 outside [0, N), computed in double precision.  The contract is
 * |error| < 1e-5 (much tighter than the 1e-4 practical tolerance of the
 * production kernels); lengths, including the leading/trailing zero-padding,
 * must match exactly.
 *
 * Coverage:
 *   - 65/48 shipped taps: eq16 (H=16), realtime (H=21), eq32 (H=32),
 *     quality (H=42) -> exercises H not a multiple of 8;
 *   - 65/32 shipped taps: realtime (H=17), quality (H=42);
 *   - lengths 0, 1, 7, 47, 48, 49, 65, 128, 1024, 4096, 56000 and the
 *     production 450000 window;
 *   - LCG noise, full-scale rails and a single full-scale impulse;
 *   - unaligned input pointers;
 *   - explicit avx2 / scalar kernels, plus avx512 when the build and CPU
 *     support it;
 *   - the RationalResamplerLmCore::set_kernel("transposed") integration
 *     compared bit-for-bit with the standalone kernel.
 */

#define BOOST_TEST_MODULE qa_uwb_resampler_transposed
#include <boost/test/unit_test.hpp>

#include <gnuradio/uwb/uwb_rational_resampler_core.h>
#include <gnuradio/uwb/uwb_resample_window.h>
#include <gnuradio/uwb/uwb_resampler_transposed.h>
#include <gnuradio/uwb/uwb_simd.h>
#include <openuwb/support/simd_config.h>

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <map>
#include <string>
#include <vector>

using openuwb::core::RationalResampler65_32Core;
using openuwb::core::RationalResampler65_48Core;
using openuwb::core::simd::transposed::TransposedResampler;
using openuwb::core::simd::transposed::TransposedResampler65_32;
using openuwb::core::simd::transposed::TransposedResampler65_48;
using gr_complex = std::complex<float>;
using sc16 = std::complex<int16_t>;

namespace {

constexpr double kTolAbs = 1e-5;
constexpr float kUnitScale = 1.0f / 32768.0f;

/** The explicit kernel under test in this build. */
const char* primary_kernel()
{
#if UWB_SIMD_HAVE_AVX512
    if (openuwb::core::simd::runtime_supported())
        return "avx512";
#endif
#if UWB_SIMD_LEVEL >= 1
    return "avx2";
#else
    return "scalar";
#endif
}

// --------------------------------------------------------------- test data --

std::string find_testdata(const std::string& rel)
{
    std::vector<std::string> cands;
#ifdef UWB_TESTDATA_DIR
    cands.push_back(std::string(UWB_TESTDATA_DIR) + "/" + rel);
#endif
    const char* prefixes[] = { "", "../", "../../", "../../../", "../../../../" };
    for (const char* p : prefixes)
        cands.push_back(std::string(p) + "testdata/" + rel);
    for (const auto& c : cands) {
        std::ifstream f(c, std::ios::binary);
        if (f)
            return c;
    }
    return std::string();
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

const std::vector<float>& real_taps(const std::string& rel)
{
    static std::map<std::string, std::vector<float>> cache;
    const auto it = cache.find(rel);
    if (it != cache.end())
        return it->second;
    const std::string path = find_testdata(rel);
    BOOST_REQUIRE_MESSAGE(!path.empty(),
                          "cannot locate testdata/" + rel);
    auto ins = cache.emplace(rel, load_f32(path));
    BOOST_TEST_MESSAGE("taps[" << rel << "] T=" << ins.first->second.size()
                               << " from " << path);
    return ins.first->second;
}

// ------------------------------------------------------------- stimulus -----

enum class Pattern { Random, Rails, Impulse };

std::vector<sc16> make_input(size_t n, Pattern p, uint32_t seed)
{
    std::vector<sc16> v(n, sc16(0, 0));
    if (n == 0)
        return v;
    if (p == Pattern::Rails) {
        static const int16_t rails[] = { 32767, -32768, -32767, 0 };
        for (size_t i = 0; i < n; ++i)
            v[i] = sc16(rails[i % 4], rails[(i + 2) % 4]);
        return v;
    }
    if (p == Pattern::Impulse) {
        v[n / 3] = sc16(32767, -32768);
        return v;
    }
    uint32_t s = seed | 1u;
    for (size_t i = 0; i < n; ++i) {
        s = s * 1664525u + 1013904223u;
        const int32_t re = static_cast<int32_t>(s >> 16) - 32768;
        s = s * 1664525u + 1013904223u;
        const int32_t im = static_cast<int32_t>(s >> 16) - 32768;
        v[i] = sc16(static_cast<int16_t>(re), static_cast<int16_t>(im));
    }
    return v;
}

const char* pattern_name(Pattern p)
{
    switch (p) {
    case Pattern::Rails:
        return "rails";
    case Pattern::Impulse:
        return "impulse";
    default:
        return "random";
    }
}

// ------------------------------------------------------------- reference ----

template <uint32_t Interp, uint32_t Decim>
std::vector<std::complex<double>> reference_upfirdn(
    const std::vector<float>& taps, const sc16* in, size_t ninput,
    double scale)
{
    const size_t T = taps.size();
    const size_t H = (T + Interp - 1) / Interp;
    const int64_t num = (static_cast<int64_t>(ninput) - 1) * Interp +
                        static_cast<int64_t>(T);
    const size_t lout =
        num <= 0 ? 0
                 : static_cast<size_t>((num + Decim - 1) / Decim);
    std::vector<std::complex<double>> y(lout);
    for (size_t m = 0; m < lout; ++m) {
        const size_t arm = (m * Decim) % Interp;
        const size_t post = (m * Decim) / Interp;
        double re = 0.0;
        double im = 0.0;
        for (size_t k = 0; k < H; ++k) {
            const int64_t q = static_cast<int64_t>(post) -
                              static_cast<int64_t>(k);
            if (q < 0 || q >= static_cast<int64_t>(ninput))
                continue;
            const size_t ti = arm + k * Interp;
            if (ti >= T)
                continue;
            const double h = static_cast<double>(taps[ti]);
            re += h * static_cast<double>(in[q].real());
            im += h * static_cast<double>(in[q].imag());
        }
        y[m] = std::complex<double>(re * scale, im * scale);
    }
    return y;
}

double max_error(const std::vector<gr_complex>& got,
                 const std::vector<std::complex<double>>& ref)
{
    BOOST_REQUIRE_EQUAL(got.size(), ref.size());
    double e = 0.0;
    for (size_t i = 0; i < got.size(); ++i) {
        e = std::max(e, std::abs(std::complex<double>(got[i].real(),
                                                     got[i].imag()) -
                                 ref[i]));
    }
    return e;
}

// --------------------------------------------------------------- runners ----

template <class Kernel, uint32_t Interp, uint32_t Decim>
void check_kernel_case(const std::vector<float>& taps, const std::string& tag,
                       const char* kernel, size_t n, Pattern p)
{
    const std::string what = tag + " kernel=" + kernel + " n=" +
                             std::to_string(n) + " " + pattern_name(p);
    const auto in = make_input(n, p, 0x9e3779b9u + static_cast<uint32_t>(n));

    Kernel k(taps);
    k.set_kernel(kernel);
    const size_t lout = Kernel::expected_output_length(n, taps.size());
    std::vector<gr_complex> y(lout);
    const size_t made = k.process_window_sc16(
        in.empty() ? nullptr : in.data(), n, kUnitScale, y.data(), y.size());
    BOOST_REQUIRE_EQUAL(made, lout);

    const auto ref = reference_upfirdn<Interp, Decim>(
        taps, in.empty() ? nullptr : in.data(), n, kUnitScale);
    const double e = max_error(y, ref);
    BOOST_CHECK_MESSAGE(e < kTolAbs,
                        what << ": max_abs=" << e << " >= " << kTolAbs);
    if (e >= kTolAbs)
        BOOST_TEST_MESSAGE(what << " FAILED max_abs=" << e);
}

template <class Kernel, uint32_t Interp, uint32_t Decim>
void check_tap_set(const std::vector<float>& taps, const std::string& tag)
{
    const size_t H = (taps.size() + Interp - 1) / Interp;
    BOOST_TEST_MESSAGE(tag << ": T=" << taps.size() << " H=" << H);

    for (size_t n : { size_t(0), size_t(1), size_t(7), size_t(47),
                      size_t(48), size_t(49), size_t(65), size_t(128),
                      size_t(1024), size_t(4096) }) {
        check_kernel_case<Kernel, Interp, Decim>(taps, tag, primary_kernel(), n,
                                                 Pattern::Random);
    }

    // Rails / impulse exercise the sign and full-scale corners.
    for (Pattern p : { Pattern::Rails, Pattern::Impulse }) {
        check_kernel_case<Kernel, Interp, Decim>(taps, tag, primary_kernel(), 1024, p);
    }

    // Scalar fallback must also satisfy the same numeric contract.
    check_kernel_case<Kernel, Interp, Decim>(taps, tag, "scalar", 1024,
                                             Pattern::Random);

    // Production-length window.
    check_kernel_case<Kernel, Interp, Decim>(taps, tag, primary_kernel(), 56000,
                                             Pattern::Random);

#if UWB_SIMD_HAVE_AVX512
    if (openuwb::core::simd::runtime_supported()) {
        for (size_t n : { size_t(0), size_t(47), size_t(1024), size_t(56000) })
            check_kernel_case<Kernel, Interp, Decim>(taps, tag, "avx512", n,
                                                     Pattern::Random);
    }
#endif

    // Unaligned input pointer, exactly on the same samples the reference sees.
    {
        const size_t n = 1024;
        const auto src = make_input(n + 3, Pattern::Random, 0xabcdef01u);
        const sc16* p = src.data() + 1;
        Kernel k(taps);
        k.set_kernel(primary_kernel());
        const size_t lout = Kernel::expected_output_length(n, taps.size());
        std::vector<gr_complex> y(lout);
        BOOST_REQUIRE_EQUAL(
            k.process_window_sc16(p, n, kUnitScale, y.data(), y.size()),
            lout);
        const auto ref =
            reference_upfirdn<Interp, Decim>(taps, p, n, kUnitScale);
        BOOST_CHECK_LT(max_error(y, ref), kTolAbs);
    }
}

// --------------------------------------------------- core integration ------

template <class Core, uint32_t Interp, uint32_t Decim>
void check_core_integration(const std::vector<float>& taps,
                            const std::string& tag)
{
    const size_t n = 4096;
    const auto in = make_input(n, Pattern::Random, 0x1234abcdu);

    using Kernel = TransposedResampler<Interp, Decim>;
    Kernel direct(taps);
    direct.set_kernel(primary_kernel());
    const size_t lout = direct.expected_output_length(n);
    std::vector<gr_complex> y_direct(lout);
    BOOST_REQUIRE_EQUAL(direct.process_window_sc16(in.data(), n, kUnitScale,
                                                   y_direct.data(),
                                                   y_direct.size()),
                        lout);

    Core core(taps);
    BOOST_REQUIRE(!core.transposed_enabled());
    core.set_kernel("transposed");
    BOOST_REQUIRE(core.transposed_enabled());
    BOOST_CHECK_EQUAL(std::string(core.sc16_kernel_variant()),
                      direct.kernel_name());

    std::vector<gr_complex> y_core(lout);
    const auto r = core.process_window_sc16(in.data(), n, kUnitScale,
                                            y_core.data(), y_core.size());
    BOOST_REQUIRE_EQUAL(r.produced, lout);
    BOOST_REQUIRE_EQUAL(r.consumed, n);
    BOOST_CHECK_EQUAL(
        std::memcmp(y_core.data(), y_direct.data(),
                    lout * sizeof(gr_complex)),
        0);

    // The standalone class is validated against the independent reference in
    // check_tap_set(); the core integration is pinned to the same bits.

    // resample_sc16_window() drives the same one-shot path.
    std::vector<gr_complex> y_win;
    BOOST_CHECK_EQUAL(openuwb::core::resample_sc16_window(
                          core, in.data(), n, kUnitScale, y_win),
                      lout);
    BOOST_CHECK_EQUAL(std::memcmp(y_win.data(), y_direct.data(),
                                  lout * sizeof(gr_complex)),
                      0);

    // Streaming process_sc16() continues to use the macroblock kernels and the
    // explicit switch must not poison it: a switch back to "default" restores
    // the default kernel name.
    core.set_kernel("default");
    BOOST_CHECK(!core.transposed_enabled());
    BOOST_CHECK_EQUAL(std::string(core.sc16_kernel_variant()), "default");
}

/** The standalone kernel constructor also enforces the runtime ISA gate. */
bool backend_ready(const char* what)
{
    if (!openuwb::core::simd::runtime_supported()) {
        BOOST_TEST_MESSAGE("SKIP " << what
                                   << ": this CPU/OS cannot execute the "
                                      "compiled backend (required ISA="
                                   << openuwb::core::simd::required_isa()
                                   << ")");
        return false;
    }
    return true;
}

} // namespace

BOOST_AUTO_TEST_CASE(test_transposed_65_48_all_profiles)
{
    if (!backend_ready("65/48 transposed"))
        return;
    check_tap_set<TransposedResampler65_48, 65, 48>(
        real_taps("resampler_65_48/taps_realtime_eq16.txt"), "65/48 eq16");
    check_tap_set<TransposedResampler65_48, 65, 48>(
        real_taps("resampler_65_48/taps_realtime_minorder.txt"), "65/48 realtime");
    check_tap_set<TransposedResampler65_48, 65, 48>(
        real_taps("resampler_65_48/taps_quality_eq32.txt"), "65/48 eq32");
    check_tap_set<TransposedResampler65_48, 65, 48>(
        real_taps("resampler_65_48/taps_quality_minorder.txt"), "65/48 quality");
}

BOOST_AUTO_TEST_CASE(test_transposed_65_32_all_profiles)
{
    if (!backend_ready("65/32 transposed"))
        return;
    check_tap_set<TransposedResampler65_32, 65, 32>(
        real_taps("resampler_65_32/taps_realtime_minorder.txt"), "65/32 realtime");
    check_tap_set<TransposedResampler65_32, 65, 32>(
        real_taps("resampler_65_32/taps_quality_minorder.txt"), "65/32 quality");
}

BOOST_AUTO_TEST_CASE(test_transposed_production_450k)
{
    if (!backend_ready("transposed 450k"))
        return;
    const auto& taps =
        real_taps("resampler_65_48/taps_quality_minorder.txt");
    check_kernel_case<TransposedResampler65_48, 65, 48>(
        taps, "65/48 quality 450k", primary_kernel(), 450000, Pattern::Random);
}

BOOST_AUTO_TEST_CASE(test_transposed_core_integration)
{
    if (!backend_ready("transposed core integration"))
        return;
    check_core_integration<RationalResampler65_48Core, 65, 48>(
        real_taps("resampler_65_48/taps_realtime_minorder.txt"), "65/48 rt");
    check_core_integration<RationalResampler65_48Core, 65, 48>(
        real_taps("resampler_65_48/taps_quality_minorder.txt"), "65/48 q");
    check_core_integration<RationalResampler65_32Core, 65, 32>(
        real_taps("resampler_65_32/taps_realtime_minorder.txt"), "65/32 rt");
}
