/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * QA: AVX-512 SC16 rational-resampler kernels vs the AVX2/FMA reference.
 *
 * Contract under test (uwb_rational_resampler_avx512.h): the AVX-512 kernels
 * keep the AVX2 per-lane accumulation order and the same reduction tree, so
 * every shipped H_pad (16/24/32/48) must produce BIT-IDENTICAL output -- not
 * merely output within a tolerance.  This file is the gate for that claim.
 *
 *   1. kernel selection: set_kernel("avx512") -> "avx512_fma_macroblock",
 *      set_kernel("avx2") -> "avx2_fma_macroblock", the C1 fixed-phase variant
 *      name, and the per-H_pad default the final 2026-11-16 ABBA fixed
 *      (H_pad 24/48 -> AVX-512, H_pad 16/32 -> AVX2).
 *   2. AVX2 vs AVX-512 bit identity for 65/48 AND 32/65 over the four shipped
 *      taps files, fixed-phase on/off, 13 input lengths (1 .. 56000).
 *   3. SC16 extremes (both rails, alternating rails, one full-scale impulse,
 *      deterministic LCG noise) and unaligned / odd-length input pointers.
 *   4. chunked streaming vs the one-shot run, both on AVX-512, bitwise, plus
 *      AVX2 vs AVX-512 on the identical irregular chunk sequence.
 *   5. reset()/flush() cycles and tiny max_out grants.
 *   6. AVX-512 vs the forced scalar kernel inside the PRE-EXISTING practical
 *      tolerance of qa_uwb_rational_resampler_core.cc (max_abs < 1e-4).  No
 *      existing assertion is loosened here.
 *
 * Build-time gate: with UWB_SIMD_HAVE_AVX512 == 0 (the AVX2-only build) every
 * case prints a SKIP line and returns, so this target still runs green there.
 * The same happens on a CPU that cannot execute the compiled AVX-512 subset.
 *
 * Taps come from the real files under testdata/; nothing here synthesises a
 * filter, so a wrong taps path fails loudly instead of silently degrading
 * into a pass.
 *
 * Scope note (chunked streaming): the chunked-vs-one-shot assertion is made
 * only for the Decim < Interp (RX) direction.  In the Decim > Interp (TX)
 * direction the core has a PRE-EXISTING, ISA-independent streaming gap:
 * process_sc16() restores its cursor from floor(output_items*M/L) and may
 * then emit an output whose window reaches past the samples that have
 * actually arrived, so a chunk boundary can zero-pad a sample the one-shot
 * run has.  It reproduces identically with force_scalar_kernel(), it is in
 * code this branch does not touch, and fixing it is outside the AVX-512 work
 * (the core header is frozen here).  The ISA claim this file owns -- AVX2 ==
 * AVX-512 bit for bit -- is therefore asserted on the identical chunk
 * sequence for BOTH directions, and the one-shot comparison for RX only.
 */

#define BOOST_TEST_MODULE qa_uwb_resampler_avx512
#include <boost/test/unit_test.hpp>

#include <gnuradio/uwb/uwb_simd_config.h>
#include <gnuradio/uwb/uwb_simd.h>
#include <gnuradio/uwb/uwb_rational_resampler_core.h>

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

using gr::uwb::core::RationalResampler32_65Core;
using gr::uwb::core::RationalResampler65_48Core;
using gr_complex = std::complex<float>;
using sc16 = std::complex<int16_t>;

namespace {

// Sc16ScalePolicy::UnitRange, the production scaling of the live RX path.
constexpr float kUnitScale = 1.0f / 32768.0f;

// Practical abs tolerance used by qa_uwb_rational_resampler_core.cc for the
// non-golden "default kernel vs forced scalar" comparison.  Not loosened.
constexpr float kScalarTolAbs = 1e-4f;

// --------------------------------------------------------------- test data --

/** The four shipped taps files and the H_pad each one produces (T/65 rounded
 *  up to a multiple of 8).  Asserted against arm_length_pad() below, so a
 *  renamed or regenerated taps file cannot silently change the coverage. */
struct TapCase {
    const char* rel;
    size_t hpad;
};

const std::vector<TapCase>& tap_cases()
{
    static const std::vector<TapCase> v = {
        { "resampler_65_48/taps_realtime_eq16.txt", 16 },
        { "resampler_65_48/taps_realtime_minorder.txt", 24 },
        { "resampler_65_48/taps_quality_eq32.txt", 32 },
        { "resampler_65_48/taps_quality_minorder.txt", 48 },
    };
    return v;
}

/** Resolve a path relative to testdata/.  Empty result => not found. */
std::string find_testdata(const std::string& rel)
{
    std::vector<std::string> cands;
#ifdef UWB_TESTDATA_DIR
    cands.push_back(std::string(UWB_TESTDATA_DIR) + "/" + rel);
#endif
    // CTest CWD is the build dir (gr-uwb/build-*); a hand run is usually one
    // level deeper (gr-uwb/build-*/lib).
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

/** Load (and cache) a real taps file; a missing file fails the test. */
const std::vector<float>& real_taps(const std::string& rel)
{
    static std::map<std::string, std::vector<float>> cache;
    const auto it = cache.find(rel);
    if (it != cache.end())
        return it->second;
    const std::string path = find_testdata(rel);
    BOOST_REQUIRE_MESSAGE(
        !path.empty(),
        "cannot locate testdata/" + rel +
            " (set UWB_TESTDATA_DIR or run from the build tree)");
    auto ins = cache.emplace(rel, load_f32(path));
    BOOST_TEST_MESSAGE("taps[" << rel << "] T=" << ins.first->second.size()
                               << " from " << path);
    return ins.first->second;
}

// ------------------------------------------------------------- skip gate ----

/**
 * Requirement gate for every case in this file.  Prints an explicit SKIP line
 * and returns false when the AVX-512 backend is absent at compile time or the
 * CPU/OS cannot execute the compiled subset.  The test still PASSES.
 */
bool avx512_ready(const char* what)
{
#if !UWB_SIMD_HAVE_AVX512
    BOOST_TEST_MESSAGE("SKIP " << what << ": no AVX-512 backend in this build"
                               << " (UWB_SIMD=" << UWB_SIMD_NAME
                               << ", required ISA="
                               << gr::uwb::core::simd::required_isa() << ")");
    return false;
#else
    if (!gr::uwb::core::simd::runtime_supported()) {
        BOOST_TEST_MESSAGE("SKIP " << what
                                   << ": this CPU/OS cannot execute the "
                                      "compiled backend (required ISA="
                                   << gr::uwb::core::simd::required_isa()
                                   << ")");
        return false;
    }
    return true;
#endif
}

// ------------------------------------------------------------- stimulus ----

/** Deterministic LCG SC16 noise (same generator as the burst-opt QA). */
std::vector<sc16> make_lcg(size_t n, uint32_t seed)
{
    std::vector<sc16> v(n);
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

std::vector<sc16> make_constant(size_t n, int16_t re, int16_t im)
{
    return std::vector<sc16>(n, sc16(re, im));
}

/** Alternating rails: every sample is a different full-scale corner. */
std::vector<sc16> make_alternating(size_t n)
{
    std::vector<sc16> v(n);
    for (size_t i = 0; i < n; ++i)
        v[i] = (i % 2 == 0) ? sc16(32767, -32768) : sc16(-32768, 32767);
    return v;
}

/** One full-scale sample in an otherwise zero stream. */
std::vector<sc16> make_impulse(size_t n)
{
    std::vector<sc16> v(n, sc16(0, 0));
    if (n > 0)
        v[n / 3] = sc16(32767, -32768);
    return v;
}

// ------------------------------------------------------------- compare ------

struct BitDiff {
    bool identical = true;
    bool length_mismatch = false;
    size_t index = 0;
    float a_re = 0.0f, a_im = 0.0f;
    float b_re = 0.0f, b_im = 0.0f;
    uint32_t a_re_bits = 0, a_im_bits = 0;
    uint32_t b_re_bits = 0, b_im_bits = 0;
};

uint32_t float_bits(float f)
{
    uint32_t u = 0;
    std::memcpy(&u, &f, sizeof(u));
    return u;
}

std::string hex32(uint32_t u)
{
    char buf[16];
    std::snprintf(buf, sizeof(buf), "0x%08x", u);
    return std::string(buf);
}

/** Bitwise compare (memcmp), reporting the first differing sample. */
BitDiff compare_bitwise(const std::vector<gr_complex>& a,
                        const std::vector<gr_complex>& b)
{
    BitDiff d;
    if (a.size() != b.size()) {
        d.identical = false;
        d.length_mismatch = true;
        d.index = std::min(a.size(), b.size());
        return d;
    }
    if (a.empty())
        return d;
    if (std::memcmp(a.data(), b.data(), a.size() * sizeof(gr_complex)) == 0)
        return d;

    d.identical = false;
    for (size_t i = 0; i < a.size(); ++i) {
        if (std::memcmp(a.data() + i, b.data() + i, sizeof(gr_complex)) == 0)
            continue;
        d.index = i;
        d.a_re = a[i].real();
        d.a_im = a[i].imag();
        d.b_re = b[i].real();
        d.b_im = b[i].imag();
        d.a_re_bits = float_bits(d.a_re);
        d.a_im_bits = float_bits(d.a_im);
        d.b_re_bits = float_bits(d.b_re);
        d.b_im_bits = float_bits(d.b_im);
        return d;
    }
    return d;
}

float max_abs_diff(const std::vector<gr_complex>& a,
                   const std::vector<gr_complex>& b)
{
    const size_t n = std::min(a.size(), b.size());
    float m = 0.0f;
    for (size_t i = 0; i < n; ++i)
        m = std::max(m, std::abs(a[i] - b[i]));
    return m;
}

/**
 * Assert bit identity and, on failure, print everything needed to reproduce
 * it: core, taps file, input length, fixed-phase flag, case label, the output
 * lengths and the first differing sample with both float and bit patterns.
 */
void require_bitwise(const char* core_name,
                     const std::string& taps_rel,
                     size_t n_in,
                     bool fixed_phase,
                     const char* label,
                     const std::vector<gr_complex>& a,
                     const std::vector<gr_complex>& b)
{
    const BitDiff d = compare_bitwise(a, b);
    if (d.identical)
        return;

    std::ostringstream os;
    os << "BIT MISMATCH core=" << core_name << " taps=" << taps_rel
       << " N_in=" << n_in
       << " fixed_phase=" << (fixed_phase ? "on" : "off") << " case=" << label
       << " N_out ref=" << a.size() << " avx512=" << b.size();
    if (d.length_mismatch) {
        os << " (output LENGTH mismatch)";
    } else if (d.index < a.size()) {
        os << " first_mismatch_index=" << d.index << " ref=("
           << a[d.index].real() << "," << a[d.index].imag()
           << ") bits=(" << hex32(d.a_re_bits) << "," << hex32(d.a_im_bits)
           << ") avx512=(" << b[d.index].real() << "," << b[d.index].imag()
           << ") bits=(" << hex32(d.b_re_bits) << "," << hex32(d.b_im_bits)
           << ")";
    }
    BOOST_CHECK_MESSAGE(false, os.str());
}

// --------------------------------------------------------------- runners ----

/**
 * One process_sc16() call with a generous grant plus the full flush tail.
 * Also pins the output-length contract (expected_output_length) for both
 * kernels, which is what makes a length drift visible instead of silent.
 */
template <typename Core>
std::vector<gr_complex> run_oneshot_sc16(Core& core,
                                         const sc16* in,
                                         size_t n,
                                         float scale)
{
    const size_t lout = Core::expected_output_length(n, core.tap_count());
    std::vector<gr_complex> y(lout + 4096, gr_complex(0.0f, 0.0f));
    size_t produced = 0;

    // Call process_sc16() even for n == 0 so flush() takes the SC16 path.
    const auto r = core.process_sc16(in, n, scale, y.data(), y.size());
    BOOST_REQUIRE_EQUAL(r.consumed, n);
    BOOST_REQUIRE_LE(r.produced, y.size());
    produced = r.produced;

    for (;;) {
        if (produced + 1024 > y.size())
            y.resize(produced + 4096);
        const size_t k = core.flush(y.data() + produced, y.size() - produced);
        if (k == 0)
            break;
        produced += k;
        BOOST_REQUIRE_LT(produced, y.size());
    }
    y.resize(produced);
    BOOST_REQUIRE_EQUAL(y.size(), lout);
    return y;
}

/**
 * Irregular chunked process_sc16() stream plus one generous flush.
 * `grant == 0` gives each call a generous output grant; a non-zero grant
 * forces that many outputs per call (tiny max_out), which is the case where
 * process() returns produced > 0 with consumed == 0.
 */
template <typename Core>
std::vector<gr_complex> run_chunked_sc16(Core& core,
                                         const sc16* in,
                                         size_t n,
                                         float scale,
                                         const std::vector<size_t>& chunks,
                                         size_t grant)
{
    BOOST_REQUIRE(!chunks.empty());
    std::vector<gr_complex> y;
    y.reserve(Core::expected_output_length(n, core.tap_count()) + 4096);

    size_t off = 0;
    size_t ci = 0;
    while (off < n) {
        const size_t ch = std::min(chunks[ci % chunks.size()], n - off);
        ++ci;
        const size_t g =
            grant ? grant : (ch * Core::kInterp) / Core::kDecim + 512;
        std::vector<gr_complex> tmp(g);
        const auto r = core.process_sc16(in + off, ch, scale, tmp.data(), g);
        BOOST_REQUIRE_LE(r.produced, g);
        y.insert(y.end(), tmp.begin(),
                 tmp.begin() + static_cast<std::ptrdiff_t>(r.produced));
        off += r.consumed;
        BOOST_REQUIRE_MESSAGE(r.produced > 0 || r.consumed > 0,
                              "process_sc16 made no progress (produced=0, "
                              "consumed=0) with grant="
                                  << g << " chunk=" << ch);
    }

    std::vector<gr_complex> tail(
        Core::expected_output_length(n, core.tap_count()) + 4096);
    size_t tail_n = 0;
    for (;;) {
        const size_t k = core.flush(tail.data() + tail_n, tail.size() - tail_n);
        if (k == 0)
            break;
        tail_n += k;
        BOOST_REQUIRE_LT(tail_n, tail.size());
    }
    y.insert(y.end(), tail.begin(),
             tail.begin() + static_cast<std::ptrdiff_t>(tail_n));
    return y;
}

/**
 * Flush-only tail with a tiny rolling grant.  The process phase is one
 * generous call, so the only thing under test is repeated flush() with
 * max_out in {1,2,3,7}.
 */
template <typename Core>
std::vector<gr_complex> run_tiny_flush(Core& core,
                                       const sc16* in,
                                       size_t n,
                                       float scale)
{
    const size_t lout = Core::expected_output_length(n, core.tap_count());
    std::vector<gr_complex> y;
    y.reserve(lout + 4096);

    std::vector<gr_complex> head(lout + 4096);
    const auto r = core.process_sc16(in, n, scale, head.data(), head.size());
    BOOST_REQUIRE_EQUAL(r.consumed, n);
    y.insert(y.end(), head.begin(),
             head.begin() + static_cast<std::ptrdiff_t>(r.produced));

    const size_t grants[] = { 1, 2, 3, 7 };
    size_t gi = 0;
    size_t guard = 0;
    for (;;) {
        const size_t g = grants[gi % 4];
        ++gi;
        std::vector<gr_complex> tmp(g);
        const size_t k = core.flush(tmp.data(), g);
        if (k == 0)
            break;
        BOOST_REQUIRE_LE(k, g);
        y.insert(y.end(), tmp.begin(),
                 tmp.begin() + static_cast<std::ptrdiff_t>(k));
        BOOST_REQUIRE_MESSAGE(++guard < 1000000u,
                              "tiny-grant flush did not terminate");
    }
    return y;
}

// ------------------------------------------------------- shared matrices ----

const size_t* matrix_lengths(size_t& count)
{
    static const size_t v[] = { 1,   7,    8,    47,  48,  49,  64,
                                65,  127,  128,  1024, 4000, 56000 };
    count = sizeof(v) / sizeof(v[0]);
    return v;
}

/**
 * H_pad the core derives for this direction: H = ceil(T/Interp), padded up to
 * a multiple of 8.  It depends on Interp, so the 32/65 direction gets
 * different H_pads (16/24/32/48 are the 65/48 shipped profiles) and mostly
 * takes the generic runtime-bound AVX-512 entry point -- which is exactly
 * what this matrix should cover as well.
 */
template <typename Core>
size_t expected_hpad(size_t T)
{
    const size_t H = (T + Core::kInterp - 1) / Core::kInterp;
    return (H + 7) & ~static_cast<size_t>(7);
}

template <typename Core>
void check_bit_identity_matrix(const char* core_name)
{
    size_t nlen = 0;
    const size_t* lengths = matrix_lengths(nlen);

    for (const auto& tc : tap_cases()) {
        const auto& taps = real_taps(tc.rel);
        for (int fp_i = 0; fp_i < 2; ++fp_i) {
            const bool fp = (fp_i == 1);
            Core ca(taps);
            Core cb(taps);
            BOOST_REQUIRE_EQUAL(ca.arm_length_pad(),
                                expected_hpad<Core>(taps.size()));
            BOOST_REQUIRE_EQUAL(cb.arm_length_pad(),
                                expected_hpad<Core>(taps.size()));

            ca.set_kernel("avx2");
            cb.set_kernel("avx512");
            ca.set_fixed_phase_schedule(fp);
            cb.set_fixed_phase_schedule(fp);

            BOOST_CHECK_EQUAL(std::string(ca.kernel_name()),
                              "avx2_fma_macroblock");
            BOOST_CHECK_EQUAL(std::string(cb.kernel_name()),
                              "avx512_fma_macroblock");
            BOOST_CHECK_EQUAL(std::string(cb.sc16_kernel_variant()),
                              fp ? "c1_fixed_phase" : "default");

            float worst = 0.0f;
            for (size_t i = 0; i < nlen; ++i) {
                const size_t n = lengths[i];
                const auto x = make_lcg(
                    n, static_cast<uint32_t>(n * 2654435761u) ^
                           (fp ? 0x9e3779b9u : 0x85ebca6bu));
                ca.reset();
                cb.reset();
                const auto ya = run_oneshot_sc16(ca, x.data(), n, kUnitScale);
                const auto yb = run_oneshot_sc16(cb, x.data(), n, kUnitScale);
                require_bitwise(core_name, tc.rel, n, fp, "process_sc16+flush",
                                ya, yb);
                worst = std::max(worst, max_abs_diff(ya, yb));
            }
            BOOST_TEST_MESSAGE("bit-identity " << core_name
                                               << " taps=" << tc.rel
                                               << " H_pad="
                                               << ca.arm_length_pad()
                                               << " fixed_phase="
                                               << (fp ? "on" : "off")
                                               << " lengths=" << nlen
                                               << " max_abs=" << worst);
        }
    }
}

/**
 * Chunked streaming: AVX2 vs AVX-512 on the identical irregular chunk
 * sequence (both directions), plus chunked vs one-shot inside AVX-512 for the
 * Decim < Interp (RX) direction where the core's chunk-invariance contract
 * holds -- see the scope note in the file header.
 */
template <typename Core>
void check_chunked_streaming(const char* core_name)
{
    const std::vector<size_t> chunks = { 1, 7, 48, 65, 3, 500, 1024, 13 };
    const size_t lengths[] = { 1024, 4000, 56000 };
    constexpr bool oneshot_invariant = (Core::kDecim < Core::kInterp);

    for (const auto& tc : tap_cases()) {
        const auto& taps = real_taps(tc.rel);
        for (int fp_i = 0; fp_i < 2; ++fp_i) {
            const bool fp = (fp_i == 1);
            for (size_t n : lengths) {
                const auto x = make_lcg(n, 0xc0ffeeu ^
                                               static_cast<uint32_t>(n) ^
                                               (fp ? 0x1234u : 0u));
                Core c2(taps);
                Core c5(taps);
                c2.set_kernel("avx2");
                c5.set_kernel("avx512");
                c2.set_fixed_phase_schedule(fp);
                c5.set_fixed_phase_schedule(fp);

                const auto y2 = run_chunked_sc16(c2, x.data(), n, kUnitScale,
                                                 chunks, 0);
                const auto y5 = run_chunked_sc16(c5, x.data(), n, kUnitScale,
                                                 chunks, 0);
                require_bitwise(core_name, tc.rel, n, fp,
                                "chunked avx2 vs avx512", y2, y5);

                if (oneshot_invariant) {
                    Core one(taps);
                    one.set_kernel("avx512");
                    one.set_fixed_phase_schedule(fp);
                    const auto y1 =
                        run_oneshot_sc16(one, x.data(), n, kUnitScale);
                    require_bitwise(core_name, tc.rel, n, fp,
                                    "chunked vs one-shot (avx512)", y1, y5);
                }
            }
        }
    }
}

/** Tiny max_out grants: AVX2 vs AVX-512 on the identical call sequence. */
template <typename Core>
void check_tiny_grants(const char* core_name)
{
    const std::vector<size_t> chunks = { 1, 7, 48, 65 };
    const size_t n = 300;
    const auto x = make_lcg(n, 0xabcdef01u);
    constexpr bool oneshot_invariant = (Core::kDecim < Core::kInterp);

    for (const auto& tc : tap_cases()) {
        const auto& taps = real_taps(tc.rel);
        for (int fp_i = 0; fp_i < 2; ++fp_i) {
            const bool fp = (fp_i == 1);
            for (size_t grant : { size_t(1), size_t(3), size_t(17) }) {
                Core ca(taps);
                Core cb(taps);
                ca.set_kernel("avx2");
                cb.set_kernel("avx512");
                ca.set_fixed_phase_schedule(fp);
                cb.set_fixed_phase_schedule(fp);

                const auto ya = run_chunked_sc16(ca, x.data(), n, kUnitScale,
                                                 chunks, grant);
                const auto yb = run_chunked_sc16(cb, x.data(), n, kUnitScale,
                                                 chunks, grant);
                require_bitwise(core_name, tc.rel, n, fp,
                                "tiny max_out grant (avx2 vs avx512)", ya, yb);

                if (oneshot_invariant) {
                    // The RX direction's process phase is chunk-invariant
                    // (pinned by qa_uwb_resampler_burst_opt.cc item 5), so a
                    // tiny grant must reproduce the one-shot stream exactly.
                    Core one(taps);
                    one.set_kernel("avx512");
                    one.set_fixed_phase_schedule(fp);
                    const auto y1 =
                        run_oneshot_sc16(one, x.data(), n, kUnitScale);
                    require_bitwise(core_name, tc.rel, n, fp,
                                    "tiny max_out grant vs one-shot (avx512)",
                                    y1, yb);
                }
            }
        }
    }
}

/** reset()/flush() cycles and tiny flush grants. */
template <typename Core>
void check_reset_flush_cycles(const char* core_name)
{
    const size_t n = 1024;
    const auto x = make_lcg(n, 0x5eed1234u);

    for (const auto& tc : tap_cases()) {
        const auto& taps = real_taps(tc.rel);
        for (int fp_i = 0; fp_i < 2; ++fp_i) {
            const bool fp = (fp_i == 1);

            // (a) reset() must restore the exact initial state: the same input
            //     through the same instance twice, and through a fresh
            //     instance, must all agree bitwise.
            Core reused(taps);
            reused.set_kernel("avx512");
            reused.set_fixed_phase_schedule(fp);
            const auto y_first =
                run_oneshot_sc16(reused, x.data(), n, kUnitScale);
            BOOST_REQUIRE_EQUAL(reused.input_items(), n);
            BOOST_REQUIRE(reused.flush_complete());
            reused.reset();
            BOOST_CHECK_EQUAL(reused.input_items(), 0u);
            BOOST_CHECK_EQUAL(reused.output_items(), 0u);
            BOOST_CHECK_EQUAL(reused.phase(), 0u);
            const auto y_second =
                run_oneshot_sc16(reused, x.data(), n, kUnitScale);
            require_bitwise(core_name, tc.rel, n, fp,
                            "reset() cycle 2 vs cycle 1 (avx512)", y_first,
                            y_second);

            Core fresh(taps);
            fresh.set_kernel("avx512");
            fresh.set_fixed_phase_schedule(fp);
            const auto y_fresh =
                run_oneshot_sc16(fresh, x.data(), n, kUnitScale);
            require_bitwise(core_name, tc.rel, n, fp,
                            "reset() cycle vs fresh instance (avx512)",
                            y_fresh, y_second);

            // (b) tiny flush grants, AVX2 vs AVX-512 on the same sequence.
            Core fa(taps);
            Core fb(taps);
            fa.set_kernel("avx2");
            fb.set_kernel("avx512");
            fa.set_fixed_phase_schedule(fp);
            fb.set_fixed_phase_schedule(fp);
            const auto ta = run_tiny_flush(fa, x.data(), n, kUnitScale);
            const auto tb = run_tiny_flush(fb, x.data(), n, kUnitScale);
            require_bitwise(core_name, tc.rel, n, fp,
                            "tiny flush grants (avx2 vs avx512)", ta, tb);
            BOOST_CHECK_EQUAL(ta.size(),
                              Core::expected_output_length(n, taps.size()));
            BOOST_CHECK_EQUAL(tb.size(),
                              Core::expected_output_length(n, taps.size()));
        }
    }
}

/**
 * RS-1: AVX2 vs AVX-512 through the zero-copy whole-window entry point, for
 * every shipped taps file and both C1 states.  This pins the batched RS-2
 * macroblock entry (with leftover outputs) to the per-output AVX2 reference.
 */
template <typename Core>
void check_window_entry_identity(const char* core_name)
{
    static const size_t lengths[] = { 1,   7,   47,  48,  49,
                                      65,  128, 1024, 4096, 56000 };
    for (const auto& tc : tap_cases()) {
        const auto& taps = real_taps(tc.rel);
        for (int fp_i = 0; fp_i < 2; ++fp_i) {
            const bool fp = (fp_i == 1);
            for (size_t n : lengths) {
                const auto x = make_lcg(
                    n, 0x1b873593u ^ static_cast<uint32_t>(n) ^
                           (fp ? 0x9e3779b9u : 0u));
                const size_t lout =
                    Core::expected_output_length(n, taps.size());

                Core ca(taps);
                Core cb(taps);
                ca.set_kernel("avx2");
                cb.set_kernel("avx512");
                ca.set_fixed_phase_schedule(fp);
                cb.set_fixed_phase_schedule(fp);

                std::vector<gr_complex> ya(lout), yb(lout);
                const auto ra = ca.process_window_sc16(x.data(), n, kUnitScale,
                                                       ya.data(), ya.size());
                const auto rb = cb.process_window_sc16(x.data(), n, kUnitScale,
                                                       yb.data(), yb.size());
                BOOST_REQUIRE_EQUAL(ra.produced, lout);
                BOOST_REQUIRE_EQUAL(rb.produced, lout);
                BOOST_REQUIRE_EQUAL(ra.consumed, n);
                BOOST_REQUIRE_EQUAL(rb.consumed, n);
                require_bitwise(core_name, tc.rel, n, fp,
                                "process_window_sc16 avx2 vs avx512", ya, yb);
            }
        }
    }
}

// --------------------------------------------------------------- cases -----

BOOST_AUTO_TEST_CASE(test_avx512_backend_report)
{
    BOOST_TEST_MESSAGE("UWB_SIMD=" << UWB_SIMD_NAME
                                   << " level=" << UWB_SIMD_LEVEL
                                   << " requested=" << UWB_SIMD_REQUESTED
                                   << " required_isa="
                                   << gr::uwb::core::simd::required_isa()
                                   << " backend="
                                   << gr::uwb::core::simd::backend_name()
                                   << " runtime_supported="
                                   << gr::uwb::core::simd::runtime_supported());
    if (!avx512_ready("backend report"))
        return;
    BOOST_CHECK_EQUAL(std::string(gr::uwb::core::simd::backend_name()),
                      "avx512");
    BOOST_CHECK(gr::uwb::core::simd::runtime_supported());
}

/**
 * Requirement 7 + 8: explicit kernel selection and the per-H_pad default.
 *
 * The final 2026-11-16 ABBA acceptance (3 rounds, production 56k window) kept
 * the default on AVX-512 only for the two profiles that clear the 5% bar:
 * H_pad 24 (realtime) and 48 (quality).  H_pad 16 (the live default) and 32
 * measured below 5% (1.3-3.2% / 4.4-6.6%, depending on instantiation), so
 * they stay on AVX2; --kernel avx512 still forces the wide kernel.
 */
BOOST_AUTO_TEST_CASE(test_avx512_kernel_selection)
{
    if (!avx512_ready("kernel selection"))
        return;

    for (const auto& tc : tap_cases()) {
        const auto& taps = real_taps(tc.rel);

        RationalResampler65_48Core core(taps);
        BOOST_REQUIRE_EQUAL(core.arm_length_pad(), tc.hpad);

        const bool wide_default = (tc.hpad == 24 || tc.hpad == 48);
        const std::string want_default =
            wide_default ? "avx512_fma_macroblock" : "avx2_fma_macroblock";

        BOOST_CHECK_MESSAGE(std::string(core.kernel_name()) == want_default,
                            "default kernel for taps="
                                << tc.rel << " H_pad=" << tc.hpad << " is "
                                << core.kernel_name() << ", expected "
                                << want_default);

        // Forced AVX-512 (every H_pad, including the AVX2-default ones).
        core.set_kernel("avx512");
        BOOST_CHECK_EQUAL(std::string(core.kernel_name()),
                          "avx512_fma_macroblock");
        BOOST_CHECK_EQUAL(std::string(core.sc16_kernel_variant()), "default");
        core.set_fixed_phase_schedule(true);
        BOOST_CHECK_EQUAL(std::string(core.sc16_kernel_variant()),
                          "c1_fixed_phase");
        BOOST_CHECK(core.fixed_phase_table_built());

        // Forced AVX2 keeps the C1 variant name (the C1 schedule is shared).
        core.set_kernel("avx2");
        BOOST_CHECK_EQUAL(std::string(core.kernel_name()),
                          "avx2_fma_macroblock");
        BOOST_CHECK_EQUAL(std::string(core.sc16_kernel_variant()),
                          "c1_fixed_phase");
        core.set_fixed_phase_schedule(false);
        BOOST_CHECK_EQUAL(std::string(core.sc16_kernel_variant()), "default");

        // "default" restores the per-H_pad choice.
        core.set_kernel("default");
        BOOST_CHECK_MESSAGE(std::string(core.kernel_name()) == want_default,
                            "set_kernel(\"default\") for taps="
                                << tc.rel << " H_pad=" << tc.hpad << " gave "
                                << core.kernel_name());
    }
}

/** Requirement 3: AVX2 vs AVX-512 bit identity, 65/48 direction. */
BOOST_AUTO_TEST_CASE(test_avx512_bit_identity_65_48)
{
    if (!avx512_ready("bit identity 65/48"))
        return;
    check_bit_identity_matrix<RationalResampler65_48Core>("65_48");
}

/** Requirement 3: AVX2 vs AVX-512 bit identity, 32/65 direction. */
BOOST_AUTO_TEST_CASE(test_avx512_bit_identity_32_65)
{
    if (!avx512_ready("bit identity 32/65"))
        return;
    check_bit_identity_matrix<RationalResampler32_65Core>("32_65");
}

/** Requirement 3 (cont.): chunked streaming, both directions. */
BOOST_AUTO_TEST_CASE(test_avx512_chunked_streaming)
{
    if (!avx512_ready("chunked streaming"))
        return;
    check_chunked_streaming<RationalResampler65_48Core>("65_48");
    check_chunked_streaming<RationalResampler32_65Core>("32_65");
}

/** Requirement 3 (cont.) + 5: tiny max_out grants and reset()/flush(). */
BOOST_AUTO_TEST_CASE(test_avx512_tiny_grants_and_reset)
{
    if (!avx512_ready("tiny grants / reset"))
        return;
    check_tiny_grants<RationalResampler65_48Core>("65_48");
    check_tiny_grants<RationalResampler32_65Core>("32_65");
    check_reset_flush_cycles<RationalResampler65_48Core>("65_48");
    check_reset_flush_cycles<RationalResampler32_65Core>("32_65");
}

/** Requirement 4: SC16 extremes must stay bit-identical. */
BOOST_AUTO_TEST_CASE(test_avx512_sc16_extremes)
{
    if (!avx512_ready("SC16 extremes"))
        return;

    const size_t lengths[] = { 65, 1024 };

    for (const auto& tc : tap_cases()) {
        const auto& taps = real_taps(tc.rel);
        for (int fp_i = 0; fp_i < 2; ++fp_i) {
            const bool fp = (fp_i == 1);
            for (size_t n : lengths) {
                struct Case {
                    const char* label;
                    std::vector<sc16> x;
                };
                std::vector<Case> cases;
                cases.push_back({ "all INT16_MIN",
                                  make_constant(n, -32768, -32768) });
                cases.push_back({ "all INT16_MAX",
                                  make_constant(n, 32767, 32767) });
                cases.push_back({ "alternating rails", make_alternating(n) });
                cases.push_back({ "single full-scale impulse",
                                  make_impulse(n) });
                cases.push_back(
                    { "LCG random", make_lcg(n, 0x13579bdfu) });

                for (const auto& c : cases) {
                    RationalResampler65_48Core ca(taps);
                    RationalResampler65_48Core cb(taps);
                    ca.set_kernel("avx2");
                    cb.set_kernel("avx512");
                    ca.set_fixed_phase_schedule(fp);
                    cb.set_fixed_phase_schedule(fp);

                    const auto ya =
                        run_oneshot_sc16(ca, c.x.data(), n, kUnitScale);
                    const auto yb =
                        run_oneshot_sc16(cb, c.x.data(), n, kUnitScale);
                    require_bitwise("65_48", tc.rel, n, fp, c.label, ya, yb);
                }
            }
        }
    }
}

/** Requirement 5: unaligned input pointers and odd lengths. */
BOOST_AUTO_TEST_CASE(test_avx512_unaligned_and_odd_lengths)
{
    if (!avx512_ready("unaligned / odd lengths"))
        return;

    const size_t lengths[] = { 1, 3, 5, 7, 9, 65, 127, 1001 };
    const size_t offsets[] = { 1, 2, 3 };

    for (const auto& tc : tap_cases()) {
        const auto& taps = real_taps(tc.rel);
        for (int fp_i = 0; fp_i < 2; ++fp_i) {
            const bool fp = (fp_i == 1);
            for (size_t n : lengths) {
                for (size_t off : offsets) {
                    const auto src = make_lcg(n + off, 0x2468aceu + off);
                    const sc16* in = src.data() + off;

                    RationalResampler65_48Core ca(taps);
                    RationalResampler65_48Core cb(taps);
                    ca.set_kernel("avx2");
                    cb.set_kernel("avx512");
                    ca.set_fixed_phase_schedule(fp);
                    cb.set_fixed_phase_schedule(fp);

                    const auto ya = run_oneshot_sc16(ca, in, n, kUnitScale);
                    const auto yb = run_oneshot_sc16(cb, in, n, kUnitScale);
                    std::ostringstream label;
                    label << "unaligned in+" << off;
                    require_bitwise("65_48", tc.rel, n, fp,
                                    label.str().c_str(), ya, yb);
                }
            }
        }
    }
}

/** Requirement 6: AVX-512 vs the forced scalar kernel, 1e-4 abs. */
BOOST_AUTO_TEST_CASE(test_avx512_vs_scalar_tolerance)
{
    if (!avx512_ready("avx512 vs scalar"))
        return;

    const size_t n = 4000;
    const auto x = make_lcg(n, 0x0badf00du);

    for (const auto& tc : tap_cases()) {
        const auto& taps = real_taps(tc.rel);
        for (int fp_i = 0; fp_i < 2; ++fp_i) {
            const bool fp = (fp_i == 1);

            RationalResampler65_48Core cw(taps);
            RationalResampler65_48Core cs(taps);
            cw.set_kernel("avx512");
            cs.force_scalar_kernel();
            cw.set_fixed_phase_schedule(fp);

            const auto yw = run_oneshot_sc16(cw, x.data(), n, kUnitScale);
            const auto ys = run_oneshot_sc16(cs, x.data(), n, kUnitScale);
            BOOST_REQUIRE_EQUAL(yw.size(), ys.size());

            const float e = max_abs_diff(yw, ys);
            BOOST_CHECK_MESSAGE(e < kScalarTolAbs,
                                "avx512 vs scalar max_abs="
                                    << e << " >= " << kScalarTolAbs
                                    << " taps=" << tc.rel
                                    << " H_pad=" << tc.hpad << " N_in=" << n
                                    << " fixed_phase=" << (fp ? "on" : "off"));
            BOOST_TEST_MESSAGE("avx512-vs-scalar taps="
                               << tc.rel << " H_pad=" << tc.hpad
                               << " fixed_phase=" << (fp ? "on" : "off")
                               << " max_abs=" << e);
        }
    }
}

/**
 * The FC32 process() path also switches kernel with fir_kind_, and the header
 * claims fir_cf32()/fir_cf32_4x() are bit-identical to fir_avx2()/fir_avx2_4x().
 * Cheap to pin here, and it keeps the SC16 and FC32 paths from drifting apart.
 */
BOOST_AUTO_TEST_CASE(test_avx512_fc32_path_bit_identity)
{
    if (!avx512_ready("fc32 path"))
        return;

    const size_t lengths[] = { 65, 1024, 4000 };

    for (const auto& tc : tap_cases()) {
        const auto& taps = real_taps(tc.rel);
        for (size_t n : lengths) {
            const auto xi = make_lcg(n, 0xfeedfaceu);
            std::vector<gr_complex> x(n);
            for (size_t i = 0; i < n; ++i)
                x[i] = gr_complex(static_cast<float>(xi[i].real()) * kUnitScale,
                                  static_cast<float>(xi[i].imag()) * kUnitScale);

            RationalResampler65_48Core ca(taps);
            RationalResampler65_48Core cb(taps);
            ca.set_kernel("avx2");
            cb.set_kernel("avx512");

            const size_t lout =
                RationalResampler65_48Core::expected_output_length(n,
                                                                   taps.size());
            std::vector<gr_complex> ya(lout + 4096), yb(lout + 4096);
            const auto ra = ca.process(x.data(), n, ya.data(), ya.size());
            const auto rb = cb.process(x.data(), n, yb.data(), yb.size());
            BOOST_REQUIRE_EQUAL(ra.produced, rb.produced);
            BOOST_REQUIRE_EQUAL(ra.consumed, rb.consumed);
            ya.resize(ra.produced);
            yb.resize(rb.produced);

            // Flush both tails into fresh buffers and compare the whole stream.
            std::vector<gr_complex> ta(4096), tb(4096);
            const size_t na = ca.flush(ta.data(), ta.size());
            const size_t nb = cb.flush(tb.data(), tb.size());
            BOOST_REQUIRE_EQUAL(na, nb);
            ya.insert(ya.end(), ta.begin(),
                      ta.begin() + static_cast<std::ptrdiff_t>(na));
            yb.insert(yb.end(), tb.begin(),
                      tb.begin() + static_cast<std::ptrdiff_t>(nb));

            require_bitwise("65_48", tc.rel, n, false, "process() fc32 + flush",
                            ya, yb);
        }
    }
}

/**
 * RS-1/RS-2: the zero-copy whole-window entry must stay bit-identical between
 * the AVX2 and AVX-512 backends, with and without the C1 schedule.
 */
BOOST_AUTO_TEST_CASE(test_avx512_window_entry_bit_identity)
{
    if (!avx512_ready("window entry"))
        return;
    check_window_entry_identity<RationalResampler65_48Core>("65_48");
    check_window_entry_identity<RationalResampler32_65Core>("32_65");
}

} // namespace
