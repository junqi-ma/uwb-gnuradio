/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Phase-2 QA: AVX-512 demod / SIC kernels and the per-group backend selector.
 *
 * Plan: docs/performance/解调与SIC_AVX512内核扩展开发规划_2026-11-16.md (P2/P3/P5).
 *
 * What this file is allowed to assert:
 *
 *   - Every AVX-512 candidate is BIT-IDENTICAL to the AVX2 reference (and to
 *     an independent scalar reference where one exists) over the boundary
 *     cases the plan lists: length 0, lengths below/at/above the vector width,
 *     unaligned addresses, zero/random/high-dynamic-range input, non-finite
 *     input, every SFD polarity pattern, and the L > 65536 guard.
 *   - A forced backend really is the backend that runs.  The selector reports
 *     it, and the outputs of the forced AVX2 and forced AVX-512 paths are
 *     compared byte for byte, so a silent fallback cannot pass as AVX-512.
 *   - A forced backend that cannot be honoured is an error, never a quiet
 *     downgrade.
 *
 * What it does NOT assert: that AVX-512 is faster.  Speed is measured by
 * apps/bench_demod_sic_kernels and the A/B/C/D matrix, not here.
 */

#include <gnuradio/uwb/uwb_demod_sic_avx512.h>
#include <gnuradio/uwb/uwb_kernel_backend.h>
#include <gnuradio/uwb/uwb_rx_sync_opt.h>
#include <gnuradio/uwb/uwb_simd.h>
#include <gnuradio/uwb/uwb_simd_config.h>

#include <boost/test/unit_test.hpp>

#include <cmath>
#include <complex>
#include <cstring>
#include <limits>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

using cf = std::complex<float>;
namespace simd = gr::uwb::core::simd;
namespace rxopt = gr::uwb::demod::rxopt;

/** RAII: force one group for the duration of a scope, then restore. */
struct ScopedBackend {
    simd::KernelGroup group;
    explicit ScopedBackend(simd::KernelGroup g, const char* name)
        : group(g)
    {
        simd::set_kernel_backend(g, std::string(name));
    }
    ~ScopedBackend() { simd::set_kernel_backend(group, simd::Backend::Default); }
};

std::vector<cf> make_random(size_t n, uint32_t seed, float amp = 1.0f)
{
    std::mt19937 rng(seed);
    std::uniform_real_distribution<float> d(-amp, amp);
    std::vector<cf> v(n);
    for (auto& x : v)
        x = cf(d(rng), d(rng));
    return v;
}

/** Duplicated real template [t0,t0,t1,t1,...] of length L. */
std::vector<float> make_taps_dup(size_t L, uint32_t seed)
{
    std::mt19937 rng(seed);
    std::uniform_real_distribution<float> d(-1.0f, 1.0f);
    std::vector<float> t(2 * L);
    for (size_t k = 0; k < L; ++k) {
        const float v = d(rng);
        t[2 * k] = v;
        t[2 * k + 1] = v;
    }
    return t;
}

// ---------------------------------------------------------------------------
// Reference: the AVX2 4-lag kernel applied twice (this is exactly what the
// dispatcher does when the Sync group is not AVX-512, so it is the production
// reference, not a reimplementation).
// ---------------------------------------------------------------------------
void dot_8lag_reference(const cf* rx, const float* taps_dup, size_t L,
                        cf out[8])
{
    rxopt::real_template_dot_4lag(rx, taps_dup, L, out);
    rxopt::real_template_dot_4lag(rx + 4, taps_dup, L, out + 4);
}

// Independent scalar reference for the 8-lag dot.
void dot_8lag_scalar(const cf* rx, const float* taps_dup, size_t L, cf out[8])
{
    for (int m = 0; m < 8; ++m) {
        cf acc(0.f, 0.f);
        for (size_t k = 0; k < L; ++k)
            acc += rx[k + m] * taps_dup[2 * k];
        out[m] = acc;
    }
}

} // namespace

BOOST_AUTO_TEST_SUITE(uwb_demod_sic_avx512_suite)

// ---------------------------------------------------------------------------
// Backend selector
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(backend_names_round_trip)
{
    BOOST_CHECK_EQUAL(std::string(simd::kernel_group_name(simd::KernelGroup::Sync)),
                      "sync");
    BOOST_CHECK_EQUAL(
        std::string(simd::kernel_group_name(simd::KernelGroup::Cir)), "cir");
    BOOST_CHECK_EQUAL(
        std::string(simd::kernel_group_name(simd::KernelGroup::SicReconstruct)),
        "sic_reconstruct");

    simd::Backend b = simd::Backend::Default;
    BOOST_CHECK(simd::parse_backend("default", &b));
    BOOST_CHECK(b == simd::Backend::Default);
    BOOST_CHECK(simd::parse_backend("scalar", &b));
    BOOST_CHECK(b == simd::Backend::Scalar);
    BOOST_CHECK(simd::parse_backend("avx2", &b));
    BOOST_CHECK(b == simd::Backend::Avx2);
    BOOST_CHECK(simd::parse_backend("avx512", &b));
    BOOST_CHECK(b == simd::Backend::Avx512);
    BOOST_CHECK(!simd::parse_backend("avx1024", &b));
    BOOST_CHECK(!simd::parse_backend("", &b));
}

BOOST_AUTO_TEST_CASE(backend_override_is_visible_and_resettable)
{
    simd::reset_kernel_backends();
    BOOST_CHECK(simd::kernel_backend_override(simd::KernelGroup::Sync) ==
                simd::Backend::Default);

    simd::set_kernel_backend(simd::KernelGroup::Sync, "scalar");
    BOOST_CHECK(simd::kernel_backend_override(simd::KernelGroup::Sync) ==
                simd::Backend::Scalar);
    BOOST_CHECK_EQUAL(
        std::string(simd::resolved_backend_name(simd::KernelGroup::Sync)),
        "scalar");
    // Other groups are untouched.
    BOOST_CHECK(simd::kernel_backend_override(simd::KernelGroup::Cir) ==
                simd::Backend::Default);

    simd::reset_kernel_backends();
    BOOST_CHECK(simd::kernel_backend_override(simd::KernelGroup::Sync) ==
                simd::Backend::Default);
}

BOOST_AUTO_TEST_CASE(unknown_backend_and_group_are_rejected)
{
    BOOST_CHECK_THROW(
        simd::set_kernel_backend(simd::KernelGroup::Sync, std::string("neon")),
        std::invalid_argument);
    BOOST_CHECK_THROW(
        simd::set_kernel_backend(static_cast<simd::KernelGroup>(99),
                                 simd::Backend::Scalar),
        std::invalid_argument);
}

BOOST_AUTO_TEST_CASE(unavailable_backends_are_refused_not_downgraded)
{
#if !UWB_SIMD_HAVE_AVX512
    // A build without the wide TU must refuse, not silently run AVX2 while
    // reporting "avx512".
    BOOST_CHECK_THROW(
        simd::set_kernel_backend(simd::KernelGroup::Sync, simd::Backend::Avx512),
        std::runtime_error);
    BOOST_CHECK(simd::kernel_backend_override(simd::KernelGroup::Sync) ==
                simd::Backend::Default);
#endif
#if !UWB_SIMD_HAVE_AVX2
    BOOST_CHECK_THROW(
        simd::set_kernel_backend(simd::KernelGroup::Sync, simd::Backend::Avx2),
        std::runtime_error);
#endif
    BOOST_CHECK_EQUAL(simd::avx512_build(), UWB_SIMD_HAVE_AVX512 != 0);
    BOOST_CHECK_EQUAL(simd::avx2_build(), UWB_SIMD_HAVE_AVX2 != 0);
}

BOOST_AUTO_TEST_CASE(default_policy_never_reports_avx512_without_a_backend)
{
    simd::reset_kernel_backends();
    for (int g = 0; g < static_cast<int>(simd::KernelGroup::Count); ++g) {
        const auto kg = static_cast<simd::KernelGroup>(g);
        const simd::Backend b = simd::resolve_backend(kg);
        if (b == simd::Backend::Avx512) {
            BOOST_CHECK(UWB_SIMD_HAVE_AVX512 != 0);
            BOOST_CHECK(simd::runtime_supported());
        }
    }
    // The measured losers keep AVX2 by default even in an AVX-512 build.
    if (UWB_SIMD_HAVE_AVX512) {
        BOOST_CHECK(simd::resolve_backend(simd::KernelGroup::Cir) !=
                    simd::Backend::Avx512);
        BOOST_CHECK(simd::resolve_backend(
                        simd::KernelGroup::SicReconstruct) !=
                    simd::Backend::Avx512);
    }
}

// ---------------------------------------------------------------------------
// 8-lag real-template dot
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(dot_8lag_matches_avx2_reference_for_all_lengths)
{
    const size_t lengths[] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 15, 16, 17,
                               31, 32, 33, 63, 64, 65, 127, 128, 129,
                               1016, 2031, 2032, 2033 };
    for (size_t L : lengths) {
        const size_t pad = L + 16;
        std::vector<cf> rx = make_random(pad, 1234u + static_cast<uint32_t>(L));
        std::vector<float> taps = make_taps_dup(L, 77u + static_cast<uint32_t>(L));

        cf ref[8], got[8];
        dot_8lag_reference(rx.data(), taps.data(), L, ref);
        rxopt::real_template_dot_8lag(rx.data(), taps.data(), L, got);
        for (int m = 0; m < 8; ++m)
            BOOST_CHECK_MESSAGE(
                std::memcmp(&ref[m], &got[m], sizeof(cf)) == 0,
                "8-lag dot differs at L=" << L << " lag=" << m);
    }
}

BOOST_AUTO_TEST_CASE(dot_8lag_length_guard_matches_avx2)
{
    // Both paths must return zeros for L == 0 and L > 65536 (the AVX2 guard).
    std::vector<cf> rx(64, cf(1.f, -1.f));
    std::vector<float> taps(2 * 64, 1.0f);
    for (size_t L : { size_t(0), size_t(65537) }) {
        cf ref[8], got[8];
        dot_8lag_reference(rx.data(), taps.data(), L, ref);
        rxopt::real_template_dot_8lag(rx.data(), taps.data(), L, got);
        for (int m = 0; m < 8; ++m) {
            BOOST_CHECK(ref[m] == cf(0.f, 0.f));
            BOOST_CHECK(got[m] == cf(0.f, 0.f));
        }
    }
}

BOOST_AUTO_TEST_CASE(dot_8lag_handles_unaligned_addresses)
{
    const size_t L = 1016;
    std::vector<cf> rx = make_random(L + 32, 4242u);
    std::vector<float> taps = make_taps_dup(L, 4243u);
    for (size_t off = 0; off < 8; ++off) {
        cf ref[8], got[8];
        dot_8lag_reference(rx.data() + off, taps.data(), L, ref);
        rxopt::real_template_dot_8lag(rx.data() + off, taps.data(), L, got);
        for (int m = 0; m < 8; ++m)
            BOOST_CHECK_MESSAGE(
                std::memcmp(&ref[m], &got[m], sizeof(cf)) == 0,
                "unaligned offset " << off << " lag " << m);
    }
}

BOOST_AUTO_TEST_CASE(dot_8lag_high_dynamic_range_and_special_values)
{
    const size_t L = 64;
    std::vector<float> taps = make_taps_dup(L, 9u);
    for (int mode = 0; mode < 4; ++mode) {
        std::vector<cf> rx(L + 16, cf(0.f, 0.f));
        for (size_t i = 0; i < rx.size(); ++i) {
            switch (mode) {
            case 0: // all zeros
                rx[i] = cf(0.f, 0.f);
                break;
            case 1: // extreme magnitudes
                rx[i] = cf((i & 1) ? 3.0e18f : -3.0e18f,
                           (i & 2) ? 1.0e-18f : -1.0e-18f);
                break;
            case 2: // non-finite
                rx[i] = cf((i % 3 == 0) ? std::numeric_limits<float>::quiet_NaN()
                                        : 1.0f,
                           (i % 5 == 0) ? std::numeric_limits<float>::infinity()
                                        : -1.0f);
                break;
            default: // denormals
                rx[i] = cf(std::numeric_limits<float>::denorm_min(), 0.f);
                break;
            }
        }
        cf ref[8], got[8];
        dot_8lag_reference(rx.data(), taps.data(), L, ref);
        rxopt::real_template_dot_8lag(rx.data(), taps.data(), L, got);
        for (int m = 0; m < 8; ++m)
            BOOST_CHECK_MESSAGE(
                std::memcmp(&ref[m], &got[m], sizeof(cf)) == 0,
                "special mode " << mode << " lag " << m);
    }
}

BOOST_AUTO_TEST_CASE(dot_8lag_matches_independent_scalar_reference)
{
    // Third opinion: an independent scalar accumulation, tolerance-checked
    // (it necessarily sums in a different order, so it cannot be bit-exact).
    const size_t L = 1016;
    std::vector<cf> rx = make_random(L + 16, 555u, 0.5f);
    std::vector<float> taps = make_taps_dup(L, 556u);
    cf ref[8], got[8];
    dot_8lag_scalar(rx.data(), taps.data(), L, ref);
    rxopt::real_template_dot_8lag(rx.data(), taps.data(), L, got);
    for (int m = 0; m < 8; ++m) {
        const float tol = 1e-4f * (1.0f + std::abs(ref[m]));
        BOOST_CHECK_MESSAGE(std::abs(ref[m] - got[m]) <= tol,
                            "lag " << m << " ref=" << ref[m]
                                   << " got=" << got[m]);
    }
}

// ---------------------------------------------------------------------------
// Element-wise complex ops
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(element_wise_ops_match_scalar_for_all_counts)
{
    const size_t counts[] = { 0, 1, 2, 3, 4, 5, 7, 8, 9, 15, 16, 17,
                              31, 32, 33, 1053, 2032, 4097 };
    for (size_t n : counts) {
        // +8 so the unaligned-offset cases below never read past the end.
        const std::vector<cf> a = make_random(n + 8, 31u + static_cast<uint32_t>(n));
        const std::vector<cf> b = make_random(n + 8, 97u + static_cast<uint32_t>(n));
        for (size_t off = 0; off < 4; ++off) {
            // accumulate
            {
                std::vector<cf> got(a.begin() + off, a.begin() + off + n);
                std::vector<cf> ref = got;
                rxopt::vector_accumulate_complex(got.data(), b.data() + off, n);
                for (size_t m = 0; m < n; ++m)
                    ref[m] = ref[m] + b[off + m];
                BOOST_CHECK_MESSAGE(
                    std::memcmp(got.data(), ref.data(), n * sizeof(cf)) == 0,
                    "accumulate n=" << n << " off=" << off);
            }
            // sub
            {
                std::vector<cf> got(a.begin() + off, a.begin() + off + n);
                std::vector<cf> ref = got;
                rxopt::vector_sub_complex(got.data(), b.data() + off, n);
                for (size_t m = 0; m < n; ++m)
                    ref[m] = ref[m] - b[off + m];
                BOOST_CHECK_MESSAGE(
                    std::memcmp(got.data(), ref.data(), n * sizeof(cf)) == 0,
                    "sub n=" << n << " off=" << off);
            }
            // sub3
            {
                std::vector<cf> got(n + 8);
                std::vector<cf> ref(n + 8);
                rxopt::vector_sub3_complex(got.data() + off, a.data() + off,
                                           b.data() + off, n);
                for (size_t m = 0; m < n; ++m)
                    ref[off + m] = a[off + m] - b[off + m];
                BOOST_CHECK_MESSAGE(
                    std::memcmp(got.data() + off, ref.data() + off,
                                n * sizeof(cf)) == 0,
                    "sub3 n=" << n << " off=" << off);
            }
            // scale
            {
                std::vector<cf> got(a.begin() + off, a.begin() + off + n);
                std::vector<cf> ref = got;
                rxopt::vector_scale_complex(got.data(), 0.375f, n);
                for (size_t m = 0; m < n; ++m)
                    ref[m] = ref[m] * 0.375f;
                BOOST_CHECK_MESSAGE(
                    std::memcmp(got.data(), ref.data(), n * sizeof(cf)) == 0,
                    "scale n=" << n << " off=" << off);
            }
        }
    }
}

BOOST_AUTO_TEST_CASE(element_wise_ops_propagate_non_finite_identically)
{
    const size_t n = 64;
    std::vector<cf> a(n), b(n);
    for (size_t i = 0; i < n; ++i) {
        a[i] = cf((i % 4 == 0) ? std::numeric_limits<float>::quiet_NaN() : 1.0f,
                  (i % 6 == 0) ? std::numeric_limits<float>::infinity() : -2.0f);
        b[i] = cf((i % 3 == 0) ? -std::numeric_limits<float>::infinity() : 0.5f,
                  (i % 7 == 0) ? std::numeric_limits<float>::quiet_NaN() : 0.25f);
    }
    std::vector<cf> got = a;
    std::vector<cf> ref = a;
    rxopt::vector_sub_complex(got.data(), b.data(), n);
    for (size_t m = 0; m < n; ++m)
        ref[m] = ref[m] - b[m];
    // Bit patterns must agree, including NaN payloads produced by the same
    // operation on the same inputs.
    BOOST_CHECK(std::memcmp(got.data(), ref.data(), n * sizeof(cf)) == 0);
}

// ---------------------------------------------------------------------------
// Batched SFD fold
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(fold_sfd_window_matches_symbol_by_symbol_reference)
{
    const size_t sym = 1016;
    const size_t fold_len = 700;
    const size_t sfd_len = 8;
    std::vector<cf> rx = make_random(sfd_len * sym + fold_len + 8, 2024u);

    // Cover: first symbol +1, first symbol -1, leading zeros, interior zeros,
    // all zeros, and a single nonzero symbol.
    const std::vector<std::vector<int8_t>> patterns = {
        { 1, -1, 1, 1, -1, -1, 1, -1 },
        { -1, 1, -1, -1, 1, 1, -1, 1 },
        { 0, 0, 1, -1, 0, 1, 0, -1 },
        { 0, 0, 0, 0, 0, 0, 0, 0 },
        { 1, 0, 0, 0, 0, 0, 0, 0 },
        { 0, 0, 0, 0, 0, 0, 0, -1 },
    };
    for (const auto& seq : patterns) {
        std::vector<cf> got(fold_len, cf(9.f, 9.f));
        std::vector<cf> ref(fold_len, cf(0.f, 0.f));
        // Reference: the pre-phase-2 symbol-by-symbol implementation.
        size_t first = 0;
        while (first < sfd_len && seq[first] == 0)
            ++first;
        if (first == sfd_len) {
            std::fill(ref.begin(), ref.end(), cf(0.f, 0.f));
        } else {
            if (seq[first] == 1)
                std::memcpy(ref.data(), rx.data() + first * sym,
                            fold_len * sizeof(cf));
            else
                for (size_t k = 0; k < fold_len; ++k)
                    ref[k] = cf(0.f, 0.f) - rx[first * sym + k];
            for (size_t i = first + 1; i < sfd_len; ++i) {
                if (seq[i] == 1)
                    for (size_t k = 0; k < fold_len; ++k)
                        ref[k] = ref[k] + rx[i * sym + k];
                else if (seq[i] == -1)
                    for (size_t k = 0; k < fold_len; ++k)
                        ref[k] = ref[k] - rx[i * sym + k];
            }
        }
        rxopt::fold_sfd_window(rx.data(), sym, seq.data(), sfd_len, fold_len,
                               got.data());
        BOOST_CHECK_MESSAGE(std::memcmp(got.data(), ref.data(),
                                        fold_len * sizeof(cf)) == 0,
                            "fold pattern mismatch");
    }
}

BOOST_AUTO_TEST_CASE(fold_sfd_window_rejects_degenerate_arguments)
{
    std::vector<cf> rx(64, cf(1.f, 1.f));
    std::vector<cf> out(16, cf(7.f, 7.f));
    const std::vector<int8_t> seq = { 1, -1 };
    // Every degenerate argument must leave `out` untouched (same contract as
    // the AVX2 path).
    rxopt::fold_sfd_window(nullptr, 8, seq.data(), 2, 16, out.data());
    rxopt::fold_sfd_window(rx.data(), 0, seq.data(), 2, 16, out.data());
    rxopt::fold_sfd_window(rx.data(), 8, nullptr, 2, 16, out.data());
    rxopt::fold_sfd_window(rx.data(), 8, seq.data(), 0, 16, out.data());
    rxopt::fold_sfd_window(rx.data(), 8, seq.data(), 2, 0, out.data());
    rxopt::fold_sfd_window(rx.data(), 8, seq.data(), 2, 16, nullptr);
    for (const auto& v : out)
        BOOST_CHECK(v == cf(7.f, 7.f));
}

// ---------------------------------------------------------------------------
// Forced-backend identity: the whole point of the override is that the same
// binary can be compared against itself.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(forced_backends_produce_identical_output)
{
    const size_t L = 1016;
    const size_t fold_len = 700;
    const size_t sfd_len = 8;
    const size_t n = 1053;

    std::vector<cf> rx = make_random(L + 32, 8080u);
    std::vector<float> taps = make_taps_dup(L, 8081u);
    std::vector<cf> a = make_random(n, 8082u);
    std::vector<cf> b = make_random(n, 8083u);
    std::vector<cf> fold_rx = make_random(sfd_len * 1016 + fold_len, 8084u);
    const std::vector<int8_t> seq = { 1, -1, 1, 1, -1, -1, 1, -1 };

    auto snapshot = [&](std::vector<cf>& dot, std::vector<cf>& acc,
                        std::vector<cf>& sub, std::vector<cf>& scl,
                        std::vector<cf>& fold) {
        dot.assign(8, cf(0.f, 0.f));
        rxopt::real_template_dot_8lag(rx.data(), taps.data(), L, dot.data());
        acc = a;
        rxopt::vector_accumulate_complex(acc.data(), b.data(), n);
        sub = a;
        rxopt::vector_sub_complex(sub.data(), b.data(), n);
        scl = a;
        rxopt::vector_scale_complex(scl.data(), 0.5f, n);
        fold.assign(fold_len, cf(0.f, 0.f));
        rxopt::fold_sfd_window(fold_rx.data(), 1016, seq.data(), sfd_len,
                               fold_len, fold.data());
    };

    // Scalar is always available and is the common reference.
    std::vector<cf> ds, as, ss, cs, fs;
    {
        ScopedBackend guard(simd::KernelGroup::Sync, "scalar");
        BOOST_CHECK_EQUAL(
            std::string(simd::resolved_backend_name(simd::KernelGroup::Sync)),
            "scalar");
        snapshot(ds, as, ss, cs, fs);
    }

#if UWB_SIMD_HAVE_AVX2
    std::vector<cf> d2, a2, s2, c2, f2;
    {
        ScopedBackend guard(simd::KernelGroup::Sync, "avx2");
        BOOST_CHECK_EQUAL(
            std::string(simd::resolved_backend_name(simd::KernelGroup::Sync)),
            "avx2");
        snapshot(d2, a2, s2, c2, f2);
    }
    BOOST_CHECK_MESSAGE(std::memcmp(d2.data(), ds.data(), 8 * sizeof(cf)) == 0,
                        "8-lag dot: AVX2 vs scalar");
    BOOST_CHECK_MESSAGE(std::memcmp(a2.data(), as.data(), n * sizeof(cf)) == 0,
                        "accumulate: AVX2 vs scalar");
    BOOST_CHECK_MESSAGE(std::memcmp(s2.data(), ss.data(), n * sizeof(cf)) == 0,
                        "sub: AVX2 vs scalar");
    BOOST_CHECK_MESSAGE(std::memcmp(c2.data(), cs.data(), n * sizeof(cf)) == 0,
                        "scale: AVX2 vs scalar");
    BOOST_CHECK_MESSAGE(
        std::memcmp(f2.data(), fs.data(), fold_len * sizeof(cf)) == 0,
        "fold: AVX2 vs scalar");
#endif

#if UWB_SIMD_HAVE_AVX512
    std::vector<cf> d5, a5, s5, c5, f5;
    {
        ScopedBackend guard(simd::KernelGroup::Sync, "avx512");
        BOOST_CHECK_EQUAL(
            std::string(simd::resolved_backend_name(simd::KernelGroup::Sync)),
            "avx512");
        snapshot(d5, a5, s5, c5, f5);
    }
    BOOST_CHECK_MESSAGE(std::memcmp(d5.data(), ds.data(), 8 * sizeof(cf)) == 0,
                        "8-lag dot: AVX-512 vs scalar");
    BOOST_CHECK_MESSAGE(std::memcmp(a5.data(), as.data(), n * sizeof(cf)) == 0,
                        "accumulate: AVX-512 vs scalar");
    BOOST_CHECK_MESSAGE(std::memcmp(s5.data(), ss.data(), n * sizeof(cf)) == 0,
                        "sub: AVX-512 vs scalar");
    BOOST_CHECK_MESSAGE(std::memcmp(c5.data(), cs.data(), n * sizeof(cf)) == 0,
                        "scale: AVX-512 vs scalar");
    BOOST_CHECK_MESSAGE(
        std::memcmp(f5.data(), fs.data(), fold_len * sizeof(cf)) == 0,
        "fold: AVX-512 vs scalar");
    // And the wide TU really is linked in.
    BOOST_CHECK(gr::uwb::core::simd::avx512::available());
#endif
}

BOOST_AUTO_TEST_CASE(avx512_kernel_entry_points_exist_when_built)
{
#if UWB_SIMD_HAVE_AVX512
    BOOST_CHECK(gr::uwb::core::simd::avx512::available());
    // Direct calls to the TU, bypassing the dispatcher, must agree with the
    // dispatcher's AVX2 path.
    const size_t L = 2032;
    std::vector<cf> rx = make_random(L + 32, 6060u);
    std::vector<float> taps = make_taps_dup(L, 6061u);
    cf direct[8], reference[8];
    gr::uwb::core::simd::avx512::real_template_dot_8lag(rx.data(),
                                                        taps.data(), L, direct);
    dot_8lag_reference(rx.data(), taps.data(), L, reference);
    for (int m = 0; m < 8; ++m)
        BOOST_CHECK(std::memcmp(&direct[m], &reference[m], sizeof(cf)) == 0);
#else
    // The wide TU is not part of this build, so the symbol does not exist and
    // must not be referenced.
    BOOST_CHECK(UWB_SIMD_HAVE_AVX512 == 0);
#endif
}

BOOST_AUTO_TEST_SUITE_END()
