#include <gnuradio/uwb/uwb_rx_phase_sfo.h>
/* SPDX-License-Identifier: GPL-3.0-or-later */
/*
 * Independent QA for the PHR pilot phase/CFO refinement and the 8-phase
 * fractional-delay CIR bank in uwb_rx_phase_sfo.h.
 *
 * 1. PHR calibration: 21 known BPSK polarity symbols with a 35 degree static
 *    bias, +2.5 kHz / -3.0 kHz residual CFO and additive complex noise, then
 *    the production decision contract is verified (the returned rotation is
 *    applied as `Re(rot * complex_value)`, not its conjugate), the estimated
 *    residual CFO (converted with delta*fs/2pi) and the payload-start phase
 *    error are bounded, and pure noise must be rejected by the coherence gate.
 * 2. Fractional CIR bank: bin 0 is bit-exact, the delay direction/centroid is
 *    +bin/phase_bins, `decompose_sfo_position()` rounds to nearest with
 *    correct negative flooring, and a synthetic 64-chip pulse train at symbol
 *    1200 under 27/50 ppm SFO is correlated on the nominal grid, on the
 *    tracked integer grid and with the fractional bank.
 *
 * All inputs are generated here; nothing depends on GR blocks, UHD or the
 * full demodulator, so this QA also runs in a pure scalar build.
 */
#include <boost/test/unit_test.hpp>

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdint>
#include <cstring>
#include <limits>
#include <random>
#include <vector>

namespace {

using openuwb::demod::rxopt::PhrPhaseRefineResult;
using openuwb::demod::rxopt::build_fractional_cir_bank;
using openuwb::demod::rxopt::decompose_sfo_position;
using openuwb::demod::rxopt::kSfoPhaseBins;
using openuwb::demod::rxopt::refine_phase_cfo_from_phr;
using cf = std::complex<float>;
using cd = std::complex<double>;

constexpr double kPi = 3.14159265358979323846264338327950288;
constexpr double kTwoPi = 2.0 * kPi;
constexpr double kRadToDeg = 180.0 / kPi;
constexpr double kQaSampleRate = 998.4e6;

// HRP PHR geometry: 512 chips/symbol at 2 samples/chip, 21 symbols, payload
// s=0 starts one symbol after the last PHR symbol.
constexpr size_t kPhrSymbolsQa = 21;
constexpr double kPhrSymbolSamplesQa = 1024.0;
constexpr double kLastToPayloadQa = 1024.0;
// Payload s=0 window start relative to PHR symbol 0's window start.
constexpr double kPayloadOffsetSamplesQa = 21.0 * kPhrSymbolSamplesQa;

double wrap_rad(double a)
{
    while (a > kPi)
        a -= kTwoPi;
    while (a <= -kPi)
        a += kTwoPi;
    return a;
}

// Band-limited prototype used only by the SFO test: sinc(0.42 t) with a Hann
// taper over |t| < 7, so adjacent chips (2 samples apart) are nearly
// orthogonal while fractional shifts stay well defined analytically.
double qa_pulse(double t)
{
    const double half = 7.0;
    if (std::abs(t) >= half)
        return 0.0;
    const double x = 0.42 * t;
    const double s = (std::abs(x) < 1e-12) ? 1.0 : std::sin(kPi * x) / (kPi * x);
    const double w = 0.5 * (1.0 + std::cos(kPi * t / half));
    return s * w;
}

double l2_norm(const cf* v, size_t n)
{
    double acc = 0.0;
    for (size_t i = 0; i < n; ++i) {
        const double re = static_cast<double>(v[i].real());
        const double im = static_cast<double>(v[i].imag());
        acc += re * re + im * im;
    }
    return std::sqrt(acc);
}

} // namespace

// ---------------------------------------------------------------------------
// 1) PHR phase / residual CFO refinement.
//
// Truth model (consistent with the header's input contract):
//   phr_z_win[k] = pol*(1-2*g1[k]) * A * exp(j*(bias + w*k*S)) + noise
// so that tilde[k] = (1-2*g1[k])*pol*z = A*exp(j*(bias + w*k*S)) and the
// correct production rotation at the payload is exp(-j*(bias + w*payload)).
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(phr_phase_refine_accuracy)
{
    const double bias_rad = 35.0 / kRadToDeg;
    const double sigma = 0.0015; // per-quadrature noise on one symbol correlation
    const size_t trials = 8;

    double worst_cfo_hz = 0.0;
    double worst_phase_deg = 0.0;
    double worst_modulus = 0.0;
    double min_coherence = 1.0;
    size_t applied = 0;
    size_t total = 0;
    size_t sign_checks = 0;

    for (int8_t pol : { int8_t(1), int8_t(-1) }) {
        for (double cfo_hz : { 2500.0, -3000.0 }) {
            const double w = kTwoPi * cfo_hz / kQaSampleRate;
            const cf rot_init(
                static_cast<float>(std::cos(-bias_rad)),
                static_cast<float>(std::sin(-bias_rad)));
            for (size_t trial = 0; trial < trials; ++trial) {
                std::mt19937 rng(static_cast<uint32_t>(
                    9173u + 131u * trial + (cfo_hz > 0.0 ? 1u : 2u) +
                    (pol > 0 ? 0u : 7u)));
                std::normal_distribution<double> nd(0.0, sigma);

                std::vector<int8_t> g1(kPhrSymbolsQa);
                std::vector<cf> z(kPhrSymbolsQa);
                for (size_t k = 0; k < g1.size(); ++k)
                    g1[k] = static_cast<int8_t>((rng() >> 5) & 1u);
                for (size_t k = 0; k < z.size(); ++k) {
                    const double ph =
                        bias_rad + w * static_cast<double>(k) * kPhrSymbolSamplesQa;
                    const double sgn = static_cast<double>(1 - 2 * int(g1[k])) *
                                       static_cast<double>(pol);
                    cd v = std::polar(1.0, ph) * sgn;
                    v += cd(nd(rng), nd(rng));
                    z[k] = cf(static_cast<float>(v.real()),
                              static_cast<float>(v.imag()));
                }

                const PhrPhaseRefineResult r = refine_phase_cfo_from_phr(
                    z.data(), z.size(), g1.data(), pol, rot_init,
                    kPhrSymbolSamplesQa, kLastToPayloadQa);
                ++total;
                BOOST_REQUIRE_MESSAGE(r.applied,
                                      "PHR refine rejected a valid 21-symbol pilot");
                ++applied;
                min_coherence =
                    std::min(min_coherence, static_cast<double>(r.coherence));

                // |rot| must be 1 so chaining it never rescales the metrics.
                worst_modulus = std::max(
                    worst_modulus,
                    std::abs(static_cast<double>(std::abs(r.rot_at_payload_start)) -
                             1.0));

                // Residual CFO in Hz, converted exactly as documented.
                const double cfo_est_hz =
                    r.delta_cfo_w_rad * kQaSampleRate / kTwoPi;
                worst_cfo_hz =
                    std::max(worst_cfo_hz, std::abs(cfo_est_hz - cfo_hz));
                // Sign convention: Wave 2 adds delta to the kernel cfo_w_rad.
                BOOST_CHECK_EQUAL(r.delta_cfo_w_rad > 0.0, cfo_hz > 0.0);
                ++sign_checks;

                // Payload-start phase accuracy in the PRODUCTION contract:
                // signed = polarity * Re(rot * complex_value).  A positive
                // residual after applying rot means the rotation is wrong.
                const double ph_payload =
                    bias_rad + w * kPayloadOffsetSamplesQa;
                const cd z_true = std::polar(1.0, ph_payload);
                const double err = std::abs(wrap_rad(
                    std::arg(cd(r.rot_at_payload_start) * z_true))) *
                                   kRadToDeg;
                worst_phase_deg = std::max(worst_phase_deg, err);

                // Direct decision-metric check on a noiseless payload symbol.
                std::vector<int8_t> g1p(16);
                for (size_t j = 0; j < g1p.size(); ++j)
                    g1p[j] = static_cast<int8_t>((rng() >> 9) & 1u);
                for (size_t j = 0; j < g1p.size(); ++j) {
                    const double sgnp =
                        static_cast<double>(1 - 2 * int(g1p[j])) *
                        static_cast<double>(pol);
                    const cd zp = std::polar(1.0, ph_payload) * sgnp;
                    // The payload complex_value already carries polarity and
                    // the BPSK sign, so the rotated metric must land on the
                    // real axis: |Re(rot*complex_value)| = |A|.
                    const double re =
                        (cd(r.rot_at_payload_start) * zp).real();
                    BOOST_CHECK_GT(std::abs(re), std::cos(3.0 / kRadToDeg));
                }
            }
        }
    }

    BOOST_TEST_MESSAGE("PHR refine: trials=" << total << " applied=" << applied
                       << " noise_sigma=" << sigma << " (post-despread SNR ~"
                       << 10.0 * std::log10(1.0 / (2.0 * sigma * sigma))
                       << " dB/symbol) worst_cfo_err=" << worst_cfo_hz
                       << " Hz worst_payload_phase_err=" << worst_phase_deg
                       << " deg min_coherence=" << min_coherence
                       << " |rot|-1=" << worst_modulus);
    BOOST_CHECK_EQUAL(applied, total);
    BOOST_CHECK_EQUAL(sign_checks, total);
    BOOST_CHECK_LT(worst_cfo_hz, 200.0);
    BOOST_CHECK_LT(worst_phase_deg, 3.0);
    BOOST_CHECK_LT(worst_modulus, 1e-5);
    BOOST_CHECK_GE(min_coherence, 0.65);
}

// The PHR re-anchors the static phase: a rot_init with a deliberate leftover
// static error must still give a correct payload rotation, because the PHR
// pilot measures the absolute residual phase.
BOOST_AUTO_TEST_CASE(phr_phase_refine_reanchors_static_phase)
{
    const double bias_rad = 35.0 / kRadToDeg;
    const double static_error_rad = 20.0 / kRadToDeg;
    const double w = kTwoPi * 2500.0 / kQaSampleRate;
    const cf rot_init(
        static_cast<float>(std::cos(-(bias_rad + static_error_rad))),
        static_cast<float>(std::sin(-(bias_rad + static_error_rad))));

    std::vector<int8_t> g1(kPhrSymbolsQa);
    std::vector<cf> z(kPhrSymbolsQa);
    for (size_t k = 0; k < g1.size(); ++k) {
        g1[k] = static_cast<int8_t>((k * 7 + 3) & 1u);
        const double ph =
            bias_rad + w * static_cast<double>(k) * kPhrSymbolSamplesQa;
        const cd v = std::polar(1.0, ph) *
                     static_cast<double>(1 - 2 * int(g1[k]));
        z[k] = cf(static_cast<float>(v.real()), static_cast<float>(v.imag()));
    }
    const PhrPhaseRefineResult r = refine_phase_cfo_from_phr(
        z.data(), z.size(), g1.data(), 1, rot_init, kPhrSymbolSamplesQa,
        kLastToPayloadQa);
    BOOST_REQUIRE(r.applied);
    const double ph_payload = bias_rad + w * kPayloadOffsetSamplesQa;
    const double err_deg = std::abs(wrap_rad(std::arg(
        cd(r.rot_at_payload_start) * std::polar(1.0, ph_payload)))) *
                           kRadToDeg;
    BOOST_TEST_MESSAGE("PHR re-anchor: rot_init static error="
                       << static_error_rad * kRadToDeg
                       << " deg -> payload phase err=" << err_deg << " deg");
    BOOST_CHECK_LT(err_deg, 0.5);
}

// Pure noise must be rejected by the coherence gate.
BOOST_AUTO_TEST_CASE(phr_phase_refine_rejects_pure_noise)
{
    const size_t trials = 100;
    size_t rejected = 0;
    double coherence_sum = 0.0;

    for (size_t trial = 0; trial < trials; ++trial) {
        std::mt19937 rng(static_cast<uint32_t>(7717u + 31u * trial));
        std::normal_distribution<double> nd(0.0, 1.0);
        std::vector<int8_t> g1(kPhrSymbolsQa);
        std::vector<cf> z(kPhrSymbolsQa);
        for (size_t k = 0; k < g1.size(); ++k) {
            g1[k] = static_cast<int8_t>((rng() >> 7) & 1u);
            z[k] = cf(static_cast<float>(nd(rng)), static_cast<float>(nd(rng)));
        }
        const int8_t pol = (trial & 1u) ? int8_t(-1) : int8_t(1);
        const cf rot_init = (trial % 3u == 0) ? cf(1.0f, 0.0f)
                                              : cf(0.6f, -0.8f);
        const PhrPhaseRefineResult r = refine_phase_cfo_from_phr(
            z.data(), z.size(), g1.data(), pol, rot_init,
            kPhrSymbolSamplesQa, kLastToPayloadQa);
        coherence_sum += static_cast<double>(r.coherence);
        if (!r.applied) {
            ++rejected;
            BOOST_CHECK_EQUAL(r.delta_cfo_w_rad, 0.0);
            BOOST_CHECK(r.rot_at_payload_start == rot_init);
            BOOST_CHECK(!r.applied);
        } else {
            BOOST_TEST_MESSAGE("noise false accept: coherence="
                               << r.coherence);
        }
    }
    const double rate = 100.0 * static_cast<double>(rejected) /
                        static_cast<double>(trials);
    BOOST_TEST_MESSAGE("PHR coherence gate: rejected " << rejected << "/"
                       << trials << " (" << rate
                       << "%), mean coherence="
                       << coherence_sum / static_cast<double>(trials));
    BOOST_CHECK_GE(rate, 95.0);
}

BOOST_AUTO_TEST_CASE(phr_phase_refine_guards)
{
    std::vector<cf> z(kPhrSymbolsQa, cf(1.0f, 0.0f));
    std::vector<int8_t> g1(kPhrSymbolsQa, 0);
    const cf rot(0.6f, 0.8f);

    BOOST_REQUIRE(refine_phase_cfo_from_phr(z.data(), z.size(), g1.data(), 1,
                                            rot, kPhrSymbolSamplesQa,
                                            kLastToPayloadQa)
                      .applied);

    auto rejected_with_default_rot = [&](const PhrPhaseRefineResult& r) {
        return !r.applied && r.delta_cfo_w_rad == 0.0 &&
               r.rot_at_payload_start == rot;
    };

    BOOST_CHECK(!refine_phase_cfo_from_phr(nullptr, z.size(), g1.data(), 1,
                                           rot, kPhrSymbolSamplesQa,
                                           kLastToPayloadQa)
                     .applied);
    BOOST_CHECK(!refine_phase_cfo_from_phr(z.data(), z.size(), nullptr, 1, rot,
                                           kPhrSymbolSamplesQa,
                                           kLastToPayloadQa)
                     .applied);
    for (size_t n : { size_t(0), size_t(1), size_t(2), size_t(3) }) {
        BOOST_CHECK(rejected_with_default_rot(refine_phase_cfo_from_phr(
            z.data(), n, g1.data(), 1, rot, kPhrSymbolSamplesQa,
            kLastToPayloadQa)));
    }
    const double bad_double[] = { 0.0,
                                  -1024.0,
                                  std::numeric_limits<double>::quiet_NaN(),
                                  std::numeric_limits<double>::infinity() };
    for (double s : bad_double) {
        BOOST_CHECK(rejected_with_default_rot(refine_phase_cfo_from_phr(
            z.data(), z.size(), g1.data(), 1, rot, s, kLastToPayloadQa)));
    }
    BOOST_CHECK(rejected_with_default_rot(refine_phase_cfo_from_phr(
        z.data(), z.size(), g1.data(), 1, rot, kPhrSymbolSamplesQa,
        kLastToPayloadQa, std::numeric_limits<float>::quiet_NaN())));
    BOOST_CHECK(rejected_with_default_rot(refine_phase_cfo_from_phr(
        z.data(), z.size(), g1.data(), 1, rot, kPhrSymbolSamplesQa,
        kLastToPayloadQa, 0.65f, 0.0)));
    BOOST_CHECK(rejected_with_default_rot(refine_phase_cfo_from_phr(
        z.data(), z.size(), g1.data(), 1, rot, kPhrSymbolSamplesQa,
        kLastToPayloadQa, 0.65f,
        std::numeric_limits<double>::quiet_NaN())));

    // A non-finite rot_init is unusable: fall back to unit rotation.
    const PhrPhaseRefineResult nan_rot = refine_phase_cfo_from_phr(
        z.data(), z.size(), g1.data(), 1,
        cf(std::numeric_limits<float>::quiet_NaN(), 0.0f),
        kPhrSymbolSamplesQa, kLastToPayloadQa);
    BOOST_CHECK(!nan_rot.applied);
    BOOST_CHECK(nan_rot.rot_at_payload_start == cf(1.0f, 0.0f));

    // A non-finite input sample must be rejected without touching memory
    // past the arrays.
    z[7] = cf(std::numeric_limits<float>::infinity(), 0.0f);
    BOOST_CHECK(rejected_with_default_rot(refine_phase_cfo_from_phr(
        z.data(), z.size(), g1.data(), 1, rot, kPhrSymbolSamplesQa,
        kLastToPayloadQa)));
    z[7] = cf(1.0f, 0.0f);

    // Clamp: a deliberately huge per-symbol step must saturate the slope.
    const double w = 5e-5; // rad/sample, above max_delta_w_rad but coherent
    std::vector<cf> drift(kPhrSymbolsQa);
    for (size_t k = 0; k < drift.size(); ++k) {
        const cd v = std::polar(1.0, w * static_cast<double>(k) *
                                         kPhrSymbolSamplesQa);
        drift[k] = cf(static_cast<float>(v.real()),
                      static_cast<float>(v.imag()));
    }
    const PhrPhaseRefineResult clamped = refine_phase_cfo_from_phr(
        drift.data(), drift.size(), g1.data(), 1, rot, kPhrSymbolSamplesQa,
        kLastToPayloadQa);
    BOOST_REQUIRE(clamped.applied);
    BOOST_TEST_MESSAGE("clamp: unclamped step=" << w
                       << " -> delta=" << clamped.delta_cfo_w_rad);
    BOOST_CHECK_LE(std::abs(clamped.delta_cfo_w_rad), 2.5e-5 + 1e-15);
    BOOST_CHECK_GT(clamped.delta_cfo_w_rad, 0.0);
}

// ---------------------------------------------------------------------------
// 2) Fractional-delay CIR bank.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(fractional_cir_bank_bin0_identity_and_energy)
{
    std::mt19937 rng(5150);
    std::uniform_real_distribution<float> dist(-1.0f, 1.0f);
    const size_t tap_count = 38;
    std::vector<cf> taps(tap_count);
    for (auto& h : taps)
        h = cf(dist(rng), dist(rng));

    std::vector<cf> bank(tap_count * kSfoPhaseBins);
    BOOST_REQUIRE(build_fractional_cir_bank(taps.data(), taps.size(),
                                            kSfoPhaseBins, bank.data()));
    // bin 0: bit-for-bit identical, no scaling, no reordering.
    BOOST_CHECK(std::memcmp(bank.data(), taps.data(),
                            tap_count * sizeof(cf)) == 0);

    const double ref = l2_norm(taps.data(), tap_count);
    double worst_rel = 0.0;
    for (size_t b = 0; b < kSfoPhaseBins; ++b) {
        const double n = l2_norm(bank.data() + b * tap_count, tap_count);
        worst_rel = std::max(worst_rel, std::abs(n / ref - 1.0));
        for (size_t m = 0; m < tap_count; ++m) {
            BOOST_CHECK(std::isfinite(bank[b * tap_count + m].real()));
            BOOST_CHECK(std::isfinite(bank[b * tap_count + m].imag()));
        }
    }
    BOOST_TEST_MESSAGE("fractional bank: worst relative L2 error="
                       << worst_rel);
    BOOST_CHECK_LT(worst_rel, 1e-6);

    // Guards.
    BOOST_CHECK(!build_fractional_cir_bank(nullptr, tap_count, kSfoPhaseBins,
                                           bank.data()));
    BOOST_CHECK(!build_fractional_cir_bank(taps.data(), 0, kSfoPhaseBins,
                                           bank.data()));
    BOOST_CHECK(!build_fractional_cir_bank(taps.data(), tap_count, 0,
                                           bank.data()));
    BOOST_CHECK(!build_fractional_cir_bank(taps.data(), tap_count,
                                           kSfoPhaseBins, nullptr));

    // phase_bins == 1 writes only bin 0 and must not touch the next element.
    std::vector<cf> single(tap_count + 1, cf(9.0f, 9.0f));
    BOOST_CHECK(build_fractional_cir_bank(taps.data(), tap_count, 1,
                                         single.data()));
    BOOST_CHECK(std::memcmp(single.data(), taps.data(),
                            tap_count * sizeof(cf)) == 0);
    BOOST_CHECK(single[tap_count] == cf(9.0f, 9.0f));

    // All-zero taps: deterministic zeros, no NaN from the normalisation.
    std::vector<cf> zeros(tap_count, cf(0.0f, 0.0f));
    std::vector<cf> zero_bank(tap_count * kSfoPhaseBins, cf(7.0f, 7.0f));
    BOOST_REQUIRE(build_fractional_cir_bank(zeros.data(), tap_count,
                                            kSfoPhaseBins, zero_bank.data()));
    for (const auto& v : zero_bank) {
        BOOST_CHECK_EQUAL(v, cf(0.0f, 0.0f));
    }
}

// The bank must DELAY by +bin/phase_bins samples, not advance.
BOOST_AUTO_TEST_CASE(fractional_cir_bank_delays_by_positive_fraction)
{
    const size_t tap_count = 13;
    const size_t m0 = 6;
    std::vector<cf> h(tap_count);
    for (size_t m = 0; m < tap_count; ++m)
        h[m] = cf(static_cast<float>(qa_pulse(static_cast<double>(m) -
                                              static_cast<double>(m0))),
                  0.0f);
    std::vector<cf> bank(tap_count * kSfoPhaseBins);
    BOOST_REQUIRE(build_fractional_cir_bank(h.data(), tap_count, kSfoPhaseBins,
                                            bank.data()));

    auto centroid = [&](const cf* v) {
        double num = 0.0;
        double den = 0.0;
        for (size_t m = 0; m < tap_count; ++m) {
            const double mag = std::abs(cd(v[m]));
            num += static_cast<double>(m) * mag;
            den += mag;
        }
        return (den > 0.0) ? num / den : 0.0;
    };
    const double c0 = centroid(h.data());
    double worst = 0.0;
    for (size_t b = 1; b < kSfoPhaseBins; ++b) {
        const double want = c0 + static_cast<double>(b) /
                                     static_cast<double>(kSfoPhaseBins);
        worst = std::max(worst, std::abs(centroid(bank.data() + b * tap_count) -
                                         want));
    }
    BOOST_TEST_MESSAGE("bank delay direction: centroid(0)=" << c0
                       << " worst |centroid - (c0 + bin/P)|=" << worst);
    BOOST_CHECK_LT(worst, 0.05);

    // Half-sample delay of a symmetric pulse: taps m0-1/m0 must stay equal up
    // to the symmetric cubic (a sign-flipped bank would swap them).
    const cf* half = bank.data() + 4 * tap_count; // delta = 0.5
    BOOST_CHECK_CLOSE(std::abs(cd(half[m0])), std::abs(cd(half[m0 + 1])),
                      1e-3);
}

BOOST_AUTO_TEST_CASE(decompose_sfo_position_rounds_to_nearest_grid)
{
    const size_t p = 8;
    const double values[] = { 0.0,
                              0.0625,
                              0.07,
                              0.1875,
                              1.0,
                              2.125,
                              3.9375,
                              7.9,
                              -0.0625,
                              -0.07,
                              -0.3,
                              -2.125,
                              -3.9375,
                              -999.9,
                              999.9 };
    for (double v : values) {
        int64_t ip = -12345;
        size_t b = 999;
        decompose_sfo_position(v, p, ip, b);
        BOOST_CHECK_LT(b, p);
        const double got = static_cast<double>(ip) +
                           static_cast<double>(b) / static_cast<double>(p);
        const double want = std::floor(v * static_cast<double>(p) + 0.5) /
                            static_cast<double>(p);
        BOOST_CHECK_SMALL(got - want, 1e-12);
        BOOST_CHECK_LE(std::abs(got - v), 1.0 / (2.0 * static_cast<double>(p)) +
                                              1e-12);
    }

    // Documented exact cases, including negative flooring.
    int64_t ip = 0;
    size_t b = 0;
    decompose_sfo_position(2.125, p, ip, b);
    BOOST_CHECK_EQUAL(ip, int64_t(2));
    BOOST_CHECK_EQUAL(b, size_t(1));
    // -0.3 is nearest to -0.25 = -2/8 => (-1, 6); -1 + 7/8 = -0.125.
    decompose_sfo_position(-0.3, p, ip, b);
    BOOST_CHECK_EQUAL(ip, int64_t(-1));
    BOOST_CHECK_EQUAL(b, size_t(6));
    BOOST_CHECK_SMALL(static_cast<double>(ip) + 0.125 * double(b) + 0.25, 1e-12);

    decompose_sfo_position(-0.0625, p, ip, b); // tie rounds up: (0, 0)
    BOOST_CHECK_EQUAL(ip, int64_t(0));
    BOOST_CHECK_EQUAL(b, size_t(0));

    // Degenerate inputs.
    const double bad[] = { std::numeric_limits<double>::quiet_NaN(),
                           std::numeric_limits<double>::infinity(),
                           -std::numeric_limits<double>::infinity() };
    for (double v : bad) {
        ip = 77;
        b = 77;
        decompose_sfo_position(v, p, ip, b);
        BOOST_CHECK_EQUAL(ip, int64_t(0));
        BOOST_CHECK_EQUAL(b, size_t(0));
    }
    ip = 77;
    b = 77;
    decompose_sfo_position(1.5, 0, ip, b);
    BOOST_CHECK_EQUAL(ip, int64_t(0));
    BOOST_CHECK_EQUAL(b, size_t(0));
}

// ---------------------------------------------------------------------------
// 3) Symbol-1200 correlation under 27/50 ppm SFO.
//
// The received station samples a 64-chip burst at 2 samples/chip, so a symbol
// spans 128 samples; at symbol s the timing error is ppm*1e-6*s*128 samples.
// Three receivers are compared against the exact fractional-delay match:
//   fixed     : nominal integer grid, no SFO tracking at all,
//   int-track : runtime integer offset from decompose_sfo_position, bin 0,
//   frac      : runtime integer offset + the 8-bin fractional CIR bank.
// ---------------------------------------------------------------------------
namespace {

struct SfoCase {
    double ppm = 0.0;
    double drift = 0.0;
    int64_t int_pos = 0;
    size_t bin = 0;
    double ratio_fixed = 0.0;
    double ratio_int = 0.0;
    double ratio_frac = 0.0;
};

SfoCase run_sfo_case(double ppm)
{
    const size_t nchips = 64;
    const int m0 = 6;
    const size_t tap_count = 13;
    const int off = 32;

    std::mt19937 rng(static_cast<uint32_t>(4242 + int(ppm)));
    std::vector<double> a(nchips);
    for (auto& v : a)
        v = ((rng() >> 3) & 1u) ? 1.0 : -1.0;

    std::vector<cf> h(tap_count);
    for (size_t m = 0; m < tap_count; ++m)
        h[m] = cf(static_cast<float>(qa_pulse(static_cast<double>(m) -
                                              static_cast<double>(m0))),
                  0.0f);

    SfoCase c;
    c.ppm = ppm;
    c.drift = ppm * 1e-6 * 1200.0 * 128.0; // samples at symbol 1200
    decompose_sfo_position(c.drift, kSfoPhaseBins, c.int_pos, c.bin);
    const double delta_true = c.drift - static_cast<double>(c.int_pos);

    // Received burst for symbol 1200 (only the needed span is synthesised).
    std::vector<double> y(256, 0.0);
    for (size_t chip = 0; chip < nchips; ++chip) {
        const double center = static_cast<double>(off) +
                              2.0 * static_cast<double>(chip) + c.drift;
        for (size_t n = 0; n < y.size(); ++n) {
            y[n] += a[chip] * qa_pulse(static_cast<double>(n) - center);
        }
    }

    std::vector<cf> bank(tap_count * kSfoPhaseBins);
    BOOST_REQUIRE(build_fractional_cir_bank(h.data(), tap_count,
                                            kSfoPhaseBins, bank.data()));

    // Exact matched template for the residual fractional offset.
    std::vector<cf> ideal(tap_count);
    for (size_t m = 0; m < tap_count; ++m)
        ideal[m] = cf(static_cast<float>(qa_pulse(
                          static_cast<double>(m) - static_cast<double>(m0) -
                          delta_true)),
                      0.0f);
    const double ref_norm = l2_norm(h.data(), tap_count);
    const double ideal_norm = l2_norm(ideal.data(), tap_count);
    if (ideal_norm > 0.0) {
        const float scale = static_cast<float>(ref_norm / ideal_norm);
        for (auto& v : ideal)
            v *= scale;
    }

    auto correlate = [&](const cf* taps, long long origin) {
        cd acc(0.0, 0.0);
        for (size_t chip = 0; chip < nchips; ++chip) {
            cd inner(0.0, 0.0);
            for (size_t q = 0; q < tap_count; ++q) {
                const long long idx = origin +
                                      2 * static_cast<long long>(chip) +
                                      static_cast<long long>(q);
                if (idx < 0 || idx >= static_cast<long long>(y.size()))
                    continue;
                inner += cd(taps[q]) * y[static_cast<size_t>(idx)];
            }
            acc += a[chip] * inner;
        }
        return std::abs(acc);
    };

    const long long base_int =
        static_cast<long long>(off + c.int_pos - m0);
    const double m_ideal = correlate(ideal.data(), base_int);
    BOOST_REQUIRE_GT(m_ideal, 0.0);
    c.ratio_fixed =
        correlate(h.data(), static_cast<long long>(off - m0)) / m_ideal;
    c.ratio_int = correlate(h.data(), base_int) / m_ideal;
    c.ratio_frac = correlate(bank.data() + c.bin * tap_count, base_int) /
                   m_ideal;
    return c;
}

} // namespace

BOOST_AUTO_TEST_CASE(sfo_tail_correlation_fractional_bank)
{
    for (double ppm : { 27.0, 50.0 }) {
        const SfoCase c = run_sfo_case(ppm);
        BOOST_TEST_MESSAGE(
            "SFO " << c.ppm << " ppm: drift=" << c.drift
                   << " samples int_pos=" << c.int_pos << " bin=" << c.bin
                   << " (delta=" << double(c.bin) / double(kSfoPhaseBins)
                   << ") | fixed=" << 100.0 * c.ratio_fixed
                   << "% int-track=" << 100.0 * c.ratio_int
                   << "% frac-bank=" << 100.0 * c.ratio_frac << "%");
        // The tracked fractional grid must hold the peak at the packet tail.
        BOOST_CHECK_GT(c.ratio_frac, 0.97);
        // The untracked nominal integer grid loses the PN alignment.
        BOOST_CHECK_LT(c.ratio_fixed, 0.20);
        // Tracking the integer offset alone must already recover most of it,
        // proving decompose_sfo_position() handles the accumulated delay.
        BOOST_CHECK_GT(c.ratio_int, 0.80);
    }
}
