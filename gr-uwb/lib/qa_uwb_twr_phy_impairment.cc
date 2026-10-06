/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * M2-A impairment-model INDEPENDENT unit QA (G0 appendix A.3 / task §5).
 *
 * WHAT THIS FILE VERIFIES
 * -----------------------
 * This QA verifies the MODEL ITSELF, not "the demod still seems to work".
 * It drives the single public entry point
 *
 *     twr::m2a_apply_impairment(iq, native_rate_hz, imp, why)
 *
 * and measures the physical quantities back out of the output:
 *
 *   * CFO      -- the phase ramp / estimated frequency offset is measured from
 *                 the output and compared to `cfo_hz`; 0 Hz is a bitwise no-op.
 *   * AWGN     -- the per-component noise variance is estimated across many
 *                 seeds and compared to P_signal_valid / (2*10^(SNR/10)); the
 *                 seed reproduces the exact vector; the variance is unchanged
 *                 by extra leading zeros (power over the valid region).
 *   * Delay    -- an impulse lands at exactly `delay_int_samples`; the
 *                 fractional phase delay matches an independently measured
 *                 reference within a documented tolerance.
 *   * Multipath-- an impulse reproduces the tap set at the tap offsets; a
 *                 single tap [1] is a bitwise no-op.
 *   * Invalid  -- NaN CFO, den == 0, num >= den, zero first tap, a bad rate and
 *                 a non-finite input are refused with a NAMED reason and leave
 *                 the buffer untouched.  Every negative case has a positive
 *                 control.
 *
 * It never asserts that two of the project's own functions agree: every
 * expectation below is recomputed here in double precision from the documented
 * closed form, or measured directly from the output.  The one exception is
 * that this QA re-implements the documented 16-tap causal fractional-delay FIR
 * in order to cross-check the arithmetic -- that is independent code, not a
 * second call into the library.
 *
 * SCOPE / HONESTY
 * ---------------
 * This is a software channel model.  It is NOT a hardware measurement and it
 * says nothing about ToA, RMARKER, first path or ranging validity.  No MATLAB
 * was run.  `measurement_valid` stays false.
 */

#include <boost/test/unit_test.hpp>

#include <gnuradio/uwb/uwb_twr_phy.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <cstdint>
#include <cstring>
#include <limits>
#include <random>
#include <string>
#include <vector>

namespace twr = gr::uwb::twr;

using gr_complex = std::complex<float>;
using dcomplex = std::complex<double>;

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kNativeRate = 737280000.0; // UC200 native rate (G0 §1)

// ---------------------------------------------------------------------------
// Signals
// ---------------------------------------------------------------------------

// A deterministic, everywhere-nonzero pseudo-random signal.  Guaranteed
// magnitude > 0.05 so ratio-based estimators are safe.
std::vector<gr_complex> make_signal(size_t n, uint64_t seed = 0xC0FFEEull)
{
    std::mt19937_64 gen(seed);
    std::uniform_real_distribution<double> d(-1.0, 1.0);
    std::vector<gr_complex> v(n);
    for (auto& x : v)
        x = gr_complex(static_cast<float>(1.0 + 0.3 * d(gen)),
                       static_cast<float>(0.3 * d(gen)));
    return v;
}

std::vector<gr_complex> make_tone(size_t n, double k0)
{
    std::vector<gr_complex> v(n);
    for (size_t i = 0; i < n; ++i) {
        const double ph = 2.0 * kPi * k0 * static_cast<double>(i) /
                          static_cast<double>(n);
        v[i] = gr_complex(static_cast<float>(std::cos(ph)),
                          static_cast<float>(std::sin(ph)));
    }
    return v;
}

// ---------------------------------------------------------------------------
// Independent references (closed form, double precision)
// ---------------------------------------------------------------------------

std::vector<gr_complex> ref_cfo(const std::vector<gr_complex>& x, double cfo_hz,
                                double rate_hz)
{
    std::vector<gr_complex> y(x.size());
    const double w = 2.0 * kPi * cfo_hz / rate_hz;
    for (size_t n = 0; n < x.size(); ++n) {
        const double ph = w * static_cast<double>(n);
        y[n] = x[n] * gr_complex(static_cast<float>(std::cos(ph)),
                                 static_cast<float>(std::sin(ph)));
    }
    return y;
}

// The documented causal fractional-delay FIR, re-implemented here.
std::array<double, 16> ref_frac_taps(double f)
{
    std::array<double, 16> h{};
    double sum = 0.0;
    for (size_t k = 0; k < h.size(); ++k) {
        const double arg = static_cast<double>(k) - f;
        const double s = (arg == 0.0) ? 1.0 : std::sin(kPi * arg) / (kPi * arg);
        h[k] = s;
        sum += s;
    }
    for (double& v : h)
        v /= sum;
    return h;
}

std::vector<gr_complex> ref_delay(const std::vector<gr_complex>& x, int64_t di,
                                  int32_t num, uint32_t den)
{
    std::vector<gr_complex> y(x.size(), gr_complex(0.f, 0.f));
    if (num == 0) {
        for (size_t n = 0; n < x.size(); ++n)
            y[n] = (n >= static_cast<size_t>(di)) ? x[n - di] : gr_complex(0.f, 0.f);
        return y;
    }
    const double f = static_cast<double>(num) / static_cast<double>(den);
    const auto h = ref_frac_taps(f);
    for (size_t n = 0; n < x.size(); ++n) {
        double ar = 0.0;
        double ai = 0.0;
        for (size_t k = 0; k < h.size(); ++k) {
            const size_t idx = static_cast<size_t>(di) + k;
            if (n < idx)
                break;
            const auto& s = x[n - idx];
            ar += h[k] * static_cast<double>(s.real());
            ai += h[k] * static_cast<double>(s.imag());
        }
        y[n] = gr_complex(static_cast<float>(ar), static_cast<float>(ai));
    }
    return y;
}

std::vector<gr_complex> ref_multipath(const std::vector<gr_complex>& x,
                                      const std::vector<gr_complex>& taps)
{
    if (taps.empty())
        return x;
    std::vector<gr_complex> y(x.size() + taps.size() - 1, gr_complex(0.f, 0.f));
    for (size_t n = 0; n < x.size(); ++n)
        for (size_t k = 0; k < taps.size(); ++k)
            y[n + k] += taps[k] * x[n];
    return y;
}

// Independent phase-delay measurement of y relative to a tone x at bin k0:
// y[n] ~= exp(-j 2*pi*k0*D/N) * x[n]  =>  D = -angle(sum y*conj(x))/(2*pi*k0/N).
double measure_phase_delay(const std::vector<gr_complex>& x,
                           const std::vector<gr_complex>& y, double k0)
{
    dcomplex acc(0.0, 0.0);
    for (size_t n = 0; n < x.size(); ++n)
        acc += dcomplex(y[n]) * std::conj(dcomplex(x[n]));
    const double phi = std::arg(acc);
    return -phi / (2.0 * kPi * k0 / static_cast<double>(x.size()));
}

double max_abs_diff(const std::vector<gr_complex>& a,
                    const std::vector<gr_complex>& b)
{
    BOOST_REQUIRE_EQUAL(a.size(), b.size());
    double m = 0.0;
    for (size_t i = 0; i < a.size(); ++i)
        m = std::max(m, std::abs(dcomplex(a[i]) - dcomplex(b[i])));
    return m;
}

// True bit-pattern equality.  `==` is NOT enough: NaN != NaN, so a buffer that
// is legitimately left untouched but contains a NaN would compare unequal.
bool same_bits(float a, float b)
{
    uint32_t ua = 0;
    uint32_t ub = 0;
    static_assert(sizeof(float) == sizeof(uint32_t), "float is not 32-bit");
    std::memcpy(&ua, &a, sizeof(float));
    std::memcpy(&ub, &b, sizeof(float));
    return ua == ub;
}

bool bitwise_equal(const std::vector<gr_complex>& a,
                   const std::vector<gr_complex>& b)
{
    if (a.size() != b.size())
        return false;
    for (size_t i = 0; i < a.size(); ++i) {
        if (!same_bits(a[i].real(), b[i].real()) ||
            !same_bits(a[i].imag(), b[i].imag()))
            return false;
    }
    return true;
}

} // namespace

BOOST_AUTO_TEST_SUITE(uwb_twr_phy_impairment_suite)

// ===========================================================================
// Degenerate / identity paths
// ===========================================================================

BOOST_AUTO_TEST_CASE(disabled_and_identity_are_bitwise_noops)
{
    const auto x = make_signal(1024);

    // disabled, even with every field populated: no-op
    {
        twr::M2aImpairment imp;
        imp.enabled = false;
        imp.cfo_hz = 12345.0;
        imp.awgn_enabled = true;
        imp.awgn_snr_db = 3.0;
        imp.delay_int_samples = 7;
        imp.delay_frac_num = 1;
        imp.delay_frac_den = 2;
        imp.multipath = { gr_complex(0.5f, 0.f), gr_complex(1.0f, 0.f) };
        auto y = x;
        std::string why;
        BOOST_REQUIRE(twr::m2a_apply_impairment(y, kNativeRate, imp, why));
        BOOST_CHECK(bitwise_equal(x, y));
    }

    // enabled but all-zero: no-op
    {
        twr::M2aImpairment imp;
        imp.enabled = true;
        auto y = x;
        std::string why;
        BOOST_REQUIRE(twr::m2a_apply_impairment(y, kNativeRate, imp, why));
        BOOST_CHECK(bitwise_equal(x, y));
    }

    // 0 Hz CFO alone: bitwise no-op
    {
        twr::M2aImpairment imp;
        imp.enabled = true;
        imp.cfo_hz = 0.0;
        auto y = x;
        std::string why;
        BOOST_REQUIRE(twr::m2a_apply_impairment(y, kNativeRate, imp, why));
        BOOST_CHECK(bitwise_equal(x, y));
    }
}

// ===========================================================================
// CFO
// ===========================================================================

BOOST_AUTO_TEST_CASE(cfo_matches_phase_ramp_and_measured_offset)
{
    const auto x = make_signal(4096);
    for (double cfo : { 20000.0, -20000.0 }) {
        twr::M2aImpairment imp;
        imp.enabled = true;
        imp.cfo_hz = cfo;
        auto y = x;
        std::string why;
        BOOST_REQUIRE(twr::m2a_apply_impairment(y, kNativeRate, imp, why));

        // (a) closed-form ramp, float rounding only
        BOOST_CHECK_SMALL(max_abs_diff(y, ref_cfo(x, cfo, kNativeRate)), 1e-5);

        // (b) measure the frequency offset from the phase increment of y/x
        double dsum = 0.0;
        size_t cnt = 0;
        for (size_t n = 1; n < x.size(); ++n) {
            const dcomplex r0 = dcomplex(y[n - 1]) / dcomplex(x[n - 1]);
            const dcomplex r1 = dcomplex(y[n]) / dcomplex(x[n]);
            dsum += std::arg(r1 * std::conj(r0));
            ++cnt;
        }
        const double f_est =
            (dsum / static_cast<double>(cnt)) * kNativeRate / (2.0 * kPi);
        BOOST_CHECK_SMALL(std::fabs(f_est - cfo), 1.0); // within 1 Hz
    }
}

BOOST_AUTO_TEST_CASE(cfo_phase_is_continuous_across_the_buffer)
{
    // The phase is a function of the GLOBAL native sample index, not wall time:
    // sample n carries exactly n increments.  Verify at the chunk boundary
    // between the first and second half that no reset happens.
    const size_t half = 512;
    const auto x = make_signal(2 * half);
    twr::M2aImpairment imp;
    imp.enabled = true;
    imp.cfo_hz = 20000.0;
    auto y = x;
    std::string why;
    BOOST_REQUIRE(twr::m2a_apply_impairment(y, kNativeRate, imp, why));
    BOOST_CHECK_SMALL(max_abs_diff(y, ref_cfo(x, imp.cfo_hz, kNativeRate)), 1e-5);
    // The second-half phase at its first sample is exp(j*w*half), i.e. it does
    // NOT restart at exp(0).
    const double w = 2.0 * kPi * imp.cfo_hz / kNativeRate;
    const dcomplex expected = dcomplex(x[half]) *
                              dcomplex(std::cos(w * half), std::sin(w * half));
    BOOST_CHECK_SMALL(std::abs(dcomplex(y[half]) - expected), 1e-5);
}

// ===========================================================================
// AWGN
// ===========================================================================

BOOST_AUTO_TEST_CASE(awgn_variance_matches_formula_per_component)
{
    const size_t N = 8192;
    const double A = 0.7;
    const std::vector<gr_complex> x(N, gr_complex(static_cast<float>(A), 0.f));
    const double P = A * A; // valid region is the whole buffer (all nonzero)

    for (double snr : { 30.0, 10.0 }) {
        const double sigma2 = P / (2.0 * std::pow(10.0, snr / 10.0));
        double sum_re = 0.0;
        double sum_im = 0.0;
        size_t cnt = 0;
        for (uint64_t seed = 0; seed < 64; ++seed) {
            twr::M2aImpairment imp;
            imp.enabled = true;
            imp.awgn_enabled = true;
            imp.awgn_snr_db = snr;
            imp.awgn_seed = seed;
            auto y = x;
            std::string why;
            BOOST_REQUIRE(twr::m2a_apply_impairment(y, kNativeRate, imp, why));
            for (size_t n = 0; n < N; ++n) {
                const double nr = static_cast<double>(y[n].real()) - A;
                const double ni = static_cast<double>(y[n].imag());
                sum_re += nr * nr;
                sum_im += ni * ni;
                ++cnt;
            }
        }
        const double var_re = sum_re / static_cast<double>(cnt);
        const double var_im = sum_im / static_cast<double>(cnt);
        BOOST_CHECK_CLOSE(var_re, sigma2, 5.0);
        BOOST_CHECK_CLOSE(var_im, sigma2, 5.0);
    }
}

BOOST_AUTO_TEST_CASE(awgn_seed_is_deterministic)
{
    const auto x = make_signal(2048);
    twr::M2aImpairment imp;
    imp.enabled = true;
    imp.awgn_enabled = true;
    imp.awgn_snr_db = 10.0;
    imp.awgn_seed = 12345;

    auto y1 = x;
    auto y2 = x;
    std::string why;
    BOOST_REQUIRE(twr::m2a_apply_impairment(y1, kNativeRate, imp, why));
    BOOST_REQUIRE(twr::m2a_apply_impairment(y2, kNativeRate, imp, why));
    BOOST_CHECK(bitwise_equal(y1, y2)); // exact same vector

    imp.awgn_seed = 12346;
    auto y3 = x;
    BOOST_REQUIRE(twr::m2a_apply_impairment(y3, kNativeRate, imp, why));
    BOOST_CHECK(!bitwise_equal(y1, y3)); // a different seed differs
}

BOOST_AUTO_TEST_CASE(awgn_valid_region_is_independent_of_leading_zeros)
{
    // The SAME signal, padded with leading/trailing zeros, must receive the
    // SAME noise samples inside its valid region -- proving the power was taken
    // over the valid region and not diluted by the padding.
    const size_t N = 3000;
    const size_t lead = 257;
    const size_t trail = 91;
    const auto S = make_signal(N, 0xABCDEFull);

    twr::M2aImpairment imp;
    imp.enabled = true;
    imp.awgn_enabled = true;
    imp.awgn_snr_db = 10.0;
    imp.awgn_seed = 7;

    std::vector<gr_complex> unpadded = S;
    std::string why;
    BOOST_REQUIRE(twr::m2a_apply_impairment(unpadded, kNativeRate, imp, why));

    std::vector<gr_complex> padded(lead, gr_complex(0.f, 0.f));
    padded.insert(padded.end(), S.begin(), S.end());
    padded.insert(padded.end(), trail, gr_complex(0.f, 0.f));
    BOOST_REQUIRE(twr::m2a_apply_impairment(padded, kNativeRate, imp, why));

    // exact equality of the added noise over the valid region
    for (size_t n = 0; n < N; ++n) {
        const gr_complex nu = unpadded[n] - S[n];
        const gr_complex np = padded[lead + n] - S[n];
        BOOST_CHECK_EQUAL(nu.real(), np.real());
        BOOST_CHECK_EQUAL(nu.imag(), np.imag());
    }
    // guard region untouched
    for (size_t n = 0; n < lead; ++n)
        BOOST_CHECK_EQUAL(padded[n].real(), 0.0f);
    for (size_t n = lead + N; n < padded.size(); ++n)
        BOOST_CHECK_EQUAL(padded[n].real(), 0.0f);

    // and the scale is the valid-region formula, not the diluted whole-buffer
    // one: estimate the variance across seeds for the unpadded signal.
    const double P = [&] {
        double p = 0.0;
        for (const auto& s : S)
            p += static_cast<double>(std::norm(s));
        return p / static_cast<double>(N);
    }();
    const double sigma2 = P / (2.0 * std::pow(10.0, 10.0 / 10.0));
    double sum_re = 0.0;
    double sum_im = 0.0;
    size_t cnt = 0;
    for (uint64_t seed = 0; seed < 64; ++seed) {
        imp.awgn_seed = seed;
        auto y = S;
        BOOST_REQUIRE(twr::m2a_apply_impairment(y, kNativeRate, imp, why));
        for (size_t n = 0; n < N; ++n) {
            const double nr = static_cast<double>(y[n].real() - S[n].real());
            const double ni = static_cast<double>(y[n].imag() - S[n].imag());
            sum_re += nr * nr;
            sum_im += ni * ni;
            ++cnt;
        }
    }
    BOOST_CHECK_CLOSE(sum_re / static_cast<double>(cnt), sigma2, 5.0);
    BOOST_CHECK_CLOSE(sum_im / static_cast<double>(cnt), sigma2, 5.0);
}

// ===========================================================================
// Delay
// ===========================================================================

BOOST_AUTO_TEST_CASE(integer_delay_impulse_lands_exactly)
{
    const size_t N = 256;
    const size_t p = 40;
    const gr_complex v(1.0f, -0.5f);
    for (int64_t d : { 0, 1, 17 }) {
        std::vector<gr_complex> x(N, gr_complex(0.f, 0.f));
        x[p] = v;
        twr::M2aImpairment imp;
        imp.enabled = true;
        imp.delay_int_samples = d;
        auto y = x;
        std::string why;
        BOOST_REQUIRE(twr::m2a_apply_impairment(y, kNativeRate, imp, why));
        BOOST_REQUIRE_EQUAL(y.size(), N);
        for (size_t n = 0; n < N; ++n) {
            const gr_complex want =
                (n == p + static_cast<size_t>(d)) ? v : gr_complex(0.f, 0.f);
            BOOST_CHECK_EQUAL(y[n].real(), want.real());
            BOOST_CHECK_EQUAL(y[n].imag(), want.imag());
        }
    }
}

BOOST_AUTO_TEST_CASE(fractional_delay_matches_measured_reference)
{
    // Documented tolerance: the phase-delay measured at a low tone (bin 4 of
    // 8192) agrees with delay_int + num/den to better than 0.01 native sample.
    const size_t N = 8192;
    const double k0 = 4.0;
    const auto tone = make_tone(N, k0);

    struct Case {
        int64_t di;
        int32_t num;
        uint32_t den;
    };
    const Case cases[] = { { 0, 1, 4 }, { 0, 1, 2 },  { 0, 3, 4 },
                           { 17, 1, 4 }, { 17, 1, 2 }, { 17, 3, 4 } };
    for (const Case& c : cases) {
        twr::M2aImpairment imp;
        imp.enabled = true;
        imp.delay_int_samples = c.di;
        imp.delay_frac_num = c.num;
        imp.delay_frac_den = c.den;
        auto y = tone;
        std::string why;
        BOOST_REQUIRE(twr::m2a_apply_impairment(y, kNativeRate, imp, why));
        const double D = measure_phase_delay(tone, y, k0);
        const double want = static_cast<double>(c.di) +
                            static_cast<double>(c.num) / static_cast<double>(c.den);
        BOOST_CHECK_SMALL(std::fabs(D - want), 0.01);
    }
}

BOOST_AUTO_TEST_CASE(fractional_delay_matches_documented_fir)
{
    // Cross-check the arithmetic against this QA's own double-precision
    // re-implementation of the documented 16-tap causal FIR.
    const auto x = make_signal(1024);
    struct Case {
        int32_t num;
        uint32_t den;
    };
    const Case cases[] = { { 1, 4 }, { 1, 2 }, { 3, 4 }, { 2, 3 } };
    for (const Case& c : cases) {
        const double f = static_cast<double>(c.num) / static_cast<double>(c.den);
        // Recorded group-delay convention: the normalized first moment of the
        // documented taps is exactly f native samples.
        {
            const auto h = ref_frac_taps(f);
            double m0 = 0.0;
            double m1 = 0.0;
            for (size_t k = 0; k < h.size(); ++k) {
                m0 += h[k];
                m1 += static_cast<double>(k) * h[k];
            }
            BOOST_CHECK_CLOSE(m1 / m0, f, 1e-6);
        }
        twr::M2aImpairment imp;
        imp.enabled = true;
        imp.delay_frac_num = c.num;
        imp.delay_frac_den = c.den;
        auto y = x;
        std::string why;
        BOOST_REQUIRE(twr::m2a_apply_impairment(y, kNativeRate, imp, why));
        BOOST_CHECK_SMALL(max_abs_diff(y, ref_delay(x, 0, c.num, c.den)), 1e-5);
    }
}

// ===========================================================================
// Multipath
// ===========================================================================

BOOST_AUTO_TEST_CASE(multipath_impulse_reproduces_taps_at_offsets)
{
    const size_t N = 128;
    const size_t p = 20;
    const gr_complex v(1.0f, 0.0f);

    // Weak first path (index 0) and a strong post-path at +8 native samples.
    std::vector<gr_complex> taps(9, gr_complex(0.f, 0.f));
    taps[0] = gr_complex(0.35f, 0.0f);
    taps[8] = gr_complex(1.0f, 0.0f);

    std::vector<gr_complex> x(N, gr_complex(0.f, 0.f));
    x[p] = v;
    twr::M2aImpairment imp;
    imp.enabled = true;
    imp.multipath = taps;
    auto y = x;
    std::string why;
    BOOST_REQUIRE(twr::m2a_apply_impairment(y, kNativeRate, imp, why));

    // tail retained: N + M - 1
    BOOST_REQUIRE_EQUAL(y.size(), N + taps.size() - 1);
    for (size_t n = 0; n < y.size(); ++n) {
        gr_complex want(0.f, 0.f);
        if (n == p)
            want = taps[0] * v;
        else if (n == p + 8)
            want = taps[8] * v;
        BOOST_CHECK_EQUAL(y[n].real(), want.real());
        BOOST_CHECK_EQUAL(y[n].imag(), want.imag());
    }
}

BOOST_AUTO_TEST_CASE(multipath_single_unit_tap_is_noop)
{
    const auto x = make_signal(512);
    twr::M2aImpairment imp;
    imp.enabled = true;
    imp.multipath = { gr_complex(1.0f, 0.0f) };
    auto y = x;
    std::string why;
    BOOST_REQUIRE(twr::m2a_apply_impairment(y, kNativeRate, imp, why));
    BOOST_REQUIRE_EQUAL(y.size(), x.size());
    BOOST_CHECK(bitwise_equal(x, y));
}

// ===========================================================================
// Order: CFO -> AWGN -> delay -> multipath (deterministic composition)
// ===========================================================================

BOOST_AUTO_TEST_CASE(composition_follows_the_frozen_order)
{
    const auto x = make_signal(512);
    twr::M2aImpairment imp;
    imp.enabled = true;
    imp.cfo_hz = 20000.0;
    imp.delay_int_samples = 3;
    imp.delay_frac_num = 1;
    imp.delay_frac_den = 2;
    imp.multipath = { gr_complex(0.35f, 0.1f), gr_complex(0.f, 0.f),
                      gr_complex(0.f, 0.f), gr_complex(0.f, 0.f),
                      gr_complex(0.f, 0.f), gr_complex(1.0f, -0.2f) };

    auto y = x;
    std::string why;
    BOOST_REQUIRE(twr::m2a_apply_impairment(y, kNativeRate, imp, why));

    // independent composition in the documented order
    auto ref = ref_cfo(x, imp.cfo_hz, kNativeRate);
    ref = ref_delay(ref, imp.delay_int_samples, imp.delay_frac_num,
                    imp.delay_frac_den);
    ref = ref_multipath(ref, imp.multipath);
    BOOST_CHECK_SMALL(max_abs_diff(y, ref), 1e-5);
}

// ===========================================================================
// Invalid inputs: refused by name, buffer untouched, positive control
// ===========================================================================

BOOST_AUTO_TEST_CASE(invalid_inputs_are_refused_and_buffers_untouched)
{
    const auto x = make_signal(256);
    const auto orig = x;

    auto expect_reject = [&](const twr::M2aImpairment& imp, double rate,
                             const char* needle) {
        auto y = x;
        std::string why;
        const bool ok = twr::m2a_apply_impairment(y, rate, imp, why);
        BOOST_CHECK(!ok);
        BOOST_CHECK_MESSAGE(why.find(needle) != std::string::npos,
                            "reason '" + why + "' does not mention '" + needle + "'");
        BOOST_CHECK(bitwise_equal(x, y)); // untouched on failure
    };

    // NaN CFO
    {
        twr::M2aImpairment imp;
        imp.enabled = true;
        imp.cfo_hz = std::numeric_limits<double>::quiet_NaN();
        expect_reject(imp, kNativeRate, "cfo");
        imp.cfo_hz = 1000.0; // positive control
        auto y = x;
        std::string why;
        BOOST_CHECK(twr::m2a_apply_impairment(y, kNativeRate, imp, why));
    }
    // den == 0
    {
        twr::M2aImpairment imp;
        imp.enabled = true;
        imp.delay_frac_num = 1;
        imp.delay_frac_den = 0;
        expect_reject(imp, kNativeRate, "delay_frac_den");
        imp.delay_frac_den = 2; // positive control
        auto y = x;
        std::string why;
        BOOST_CHECK(twr::m2a_apply_impairment(y, kNativeRate, imp, why));
    }
    // num >= den
    {
        twr::M2aImpairment imp;
        imp.enabled = true;
        imp.delay_frac_num = 2;
        imp.delay_frac_den = 2;
        expect_reject(imp, kNativeRate, "delay_frac_num");
        imp.delay_frac_num = 1; // positive control
        auto y = x;
        std::string why;
        BOOST_CHECK(twr::m2a_apply_impairment(y, kNativeRate, imp, why));
    }
    // zero first tap
    {
        twr::M2aImpairment imp;
        imp.enabled = true;
        imp.multipath = { gr_complex(0.f, 0.f), gr_complex(1.0f, 0.f) };
        expect_reject(imp, kNativeRate, "multipath[0]");
        imp.multipath[0] = gr_complex(0.5f, 0.f); // positive control
        auto y = x;
        std::string why;
        BOOST_CHECK(twr::m2a_apply_impairment(y, kNativeRate, imp, why));
    }
    // non-zero CFO with an unusable rate
    {
        twr::M2aImpairment imp;
        imp.enabled = true;
        imp.cfo_hz = 1000.0;
        expect_reject(imp, 0.0, "native_rate_hz");
        auto y = x;
        std::string why;
        BOOST_CHECK(twr::m2a_apply_impairment(y, kNativeRate, imp, why));
    }
    // a non-finite sample in the input, with an active impairment
    {
        twr::M2aImpairment imp;
        imp.enabled = true;
        imp.cfo_hz = 1000.0;
        auto bad = x;
        bad[3] = gr_complex(std::numeric_limits<float>::quiet_NaN(), 0.f);
        auto y = bad;
        std::string why;
        BOOST_CHECK(!twr::m2a_apply_impairment(y, kNativeRate, imp, why));
        BOOST_CHECK(why.find("non-finite") != std::string::npos);
        BOOST_CHECK(bitwise_equal(bad, y)); // untouched on failure
    }
    // sanity: the pristine input still passes its positive control
    {
        twr::M2aImpairment imp;
        imp.enabled = true;
        imp.cfo_hz = 1000.0;
        auto y = x;
        std::string why;
        BOOST_CHECK(twr::m2a_apply_impairment(y, kNativeRate, imp, why));
    }
    BOOST_CHECK(bitwise_equal(x, orig));
}

BOOST_AUTO_TEST_SUITE_END()
