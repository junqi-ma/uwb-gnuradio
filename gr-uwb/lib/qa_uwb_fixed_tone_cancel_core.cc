/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * QA for the fixed-frequency CW cancellation core.
 *
 * Every expectation is derived independently from the documented mathematics
 * with a double reference, not by mirroring the implementation:
 *   - the fold projection is compared against the direct exponential sum
 *   - the tone table is rebuilt from the reported coefficient
 *   - the output is rebuilt sample by sample (nearest-even quantisation and
 *     int16 saturation), including the cross-fade blend
 *   - scalar and AVX2 kernels must agree bit for bit
 */

#include <boost/test/unit_test.hpp>

#include <gnuradio/uwb/uwb_fixed_tone_cancel_core.h>

#include <algorithm>
#include <cfenv>
#include <cmath>
#include <complex>
#include <cstdint>
#include <random>
#include <string>
#include <vector>

using namespace gr::uwb::core;

namespace {

constexpr double kFs = 737.28e6;
constexpr double kF0 = -232.96e6;
constexpr size_t kPeriod = 288;
constexpr int64_t kPeriodLength = 14745600; // 20 ms at 737.28 MS/s

FixedToneCancelConfig base_config()
{
    FixedToneCancelConfig cfg;
    cfg.enable = true;
    cfg.sample_rate = kFs;
    cfg.tone_frequency_hz = kF0;
    cfg.period_s = 20.0e-3;
    cfg.training_s = 0.5e-3;
    cfg.crossfade_samples = 256;
    return cfg;
}

// Short training windows keep the per-test vectors small; the projection and
// quantisation contracts do not depend on the window length.
FixedToneCancelConfig short_config(size_t training_samples, size_t crossfade = 256)
{
    FixedToneCancelConfig cfg = base_config();
    cfg.training_s = static_cast<double>(training_samples) / kFs;
    cfg.crossfade_samples = crossfade;
    return cfg;
}

// Independent nearest-even quantiser (same contract as the core: reject before
// rounding, never clip a tone component).
bool qref(double v, int16_t& out)
{
    if (!std::isfinite(v) || v < -32769.0 || v > 32768.0) {
        return false;
    }
    const double lo = std::floor(v);
    const double frac = v - lo;
    const double z =
        lo + ((frac > 0.5 || (frac == 0.5 && std::fmod(lo, 2.0) != 0.0)) ? 1.0 : 0.0);
    if (z < -32768.0 || z > 32767.0) {
        return false;
    }
    out = static_cast<int16_t>(z);
    return true;
}

std::complex<double> reference_coefficient(const std::vector<int16_t>& x,
                                           size_t n,
                                           double omega)
{
    std::complex<double> acc(0.0, 0.0);
    for (size_t k = 0; k < n; ++k) {
        const double ph = -omega * static_cast<double>(k);
        acc += std::complex<double>(static_cast<double>(x[2 * k]),
                                    static_cast<double>(x[2 * k + 1])) *
               std::complex<double>(std::cos(ph), std::sin(ph));
    }
    return acc / static_cast<double>(n);
}

std::vector<int16_t> reference_lut(std::complex<double> c, double omega)
{
    std::vector<int16_t> lut(2 * kPeriod, 0);
    for (size_t k = 0; k < kPeriod; ++k) {
        const double ph = omega * static_cast<double>(k);
        const double cs = std::cos(ph);
        const double sn = std::sin(ph);
        const double re = c.real() * cs - c.imag() * sn;
        const double im = c.real() * sn + c.imag() * cs;
        const bool ok = qref(re, lut[2 * k]) && qref(im, lut[2 * k + 1]);
        BOOST_REQUIRE_MESSAGE(ok, "reference tone component must fit int16");
    }
    return lut;
}

std::vector<int16_t> reference_blend(const std::vector<int16_t>& old_lut,
                                     int64_t old_epoch,
                                     const std::vector<int16_t>& new_lut,
                                     int64_t new_epoch,
                                     size_t crossfade)
{
    std::vector<int16_t> blend(2 * crossfade, 0);
    for (size_t j = 0; j < crossfade; ++j) {
        const double a = static_cast<double>(j + 1) / static_cast<double>(crossfade);
        int64_t rel = new_epoch + static_cast<int64_t>(j) - old_epoch;
        rel = ((rel % static_cast<int64_t>(kPeriod)) + static_cast<int64_t>(kPeriod)) %
              static_cast<int64_t>(kPeriod);
        const size_t old_idx = static_cast<size_t>(rel);
        const size_t new_idx = j % kPeriod;
        const double br = (1.0 - a) * static_cast<double>(old_lut[2 * old_idx]) +
                          a * static_cast<double>(new_lut[2 * new_idx]);
        const double bi = (1.0 - a) * static_cast<double>(old_lut[2 * old_idx + 1]) +
                          a * static_cast<double>(new_lut[2 * new_idx + 1]);
        const bool ok = qref(br, blend[2 * j]) && qref(bi, blend[2 * j + 1]);
        BOOST_REQUIRE_MESSAGE(ok, "reference cross-fade component must fit int16");
    }
    return blend;
}

// Independent output reference: out = saturate_int16(in - tone), tone selected
// by the absolute sample index relative to the model epoch.
std::vector<int16_t> reference_apply(const std::vector<int16_t>& src,
                                     int64_t start_abs,
                                     int64_t epoch,
                                     const std::vector<int16_t>& lut,
                                     const std::vector<int16_t>& blend,
                                     size_t crossfade)
{
    const size_t n = src.size() / 2;
    std::vector<int16_t> out(2 * n, 0);
    for (size_t k = 0; k < n; ++k) {
        const int64_t rel = start_abs + static_cast<int64_t>(k) - epoch;
        BOOST_REQUIRE_MESSAGE(rel >= 0, "reference input must not predate the epoch");
        int16_t tr = 0;
        int16_t ti = 0;
        if (static_cast<uint64_t>(rel) < crossfade) {
            tr = blend[2 * static_cast<size_t>(rel)];
            ti = blend[2 * static_cast<size_t>(rel) + 1];
        } else {
            const size_t idx = static_cast<size_t>(rel % static_cast<int64_t>(kPeriod));
            tr = lut[2 * idx];
            ti = lut[2 * idx + 1];
        }
        const int vi = int(src[2 * k]) - int(tr);
        const int vq = int(src[2 * k + 1]) - int(ti);
        out[2 * k] =
            static_cast<int16_t>(vi < -32768 ? -32768 : (vi > 32767 ? 32767 : vi));
        out[2 * k + 1] =
            static_cast<int16_t>(vq < -32768 ? -32768 : (vq > 32767 ? 32767 : vq));
    }
    return out;
}

// A synthetic complex tone plus bounded noise, so the fold projection is
// non-trivial and the reference is independent of the estimate.
std::vector<int16_t> make_tone_samples(size_t n,
                                       double amplitude,
                                       double phase,
                                       double omega,
                                       unsigned seed,
                                       int noise = 0)
{
    std::mt19937 gen(seed);
    std::uniform_int_distribution<int> dist(-noise, noise);
    std::vector<int16_t> x(2 * n, 0);
    for (size_t k = 0; k < n; ++k) {
        const double ph = omega * static_cast<double>(k) + phase;
        const double re = amplitude * std::cos(ph) + dist(gen);
        const double im = amplitude * std::sin(ph) + dist(gen);
        x[2 * k] =
            static_cast<int16_t>(std::lround(std::max(-32768.0, std::min(32767.0, re))));
        x[2 * k + 1] =
            static_cast<int16_t>(std::lround(std::max(-32768.0, std::min(32767.0, im))));
    }
    return x;
}

void train(FixedToneCancelCore& core,
           int64_t epoch,
           const std::vector<int16_t>& samples,
           size_t chunk)
{
    std::string err;
    BOOST_REQUIRE_MESSAGE(core.begin_training(epoch, &err), err);
    const size_t n = samples.size() / 2;
    size_t pos = 0;
    while (pos < n) {
        const size_t cnt = std::min(n - pos, chunk);
        BOOST_REQUIRE_MESSAGE(
            core.add_training_samples(samples.data() + 2 * pos, cnt, &err), err);
        pos += cnt;
    }
}

void train_and_require_ok(FixedToneCancelCore& core,
                          int64_t epoch,
                          const std::vector<int16_t>& samples,
                          size_t chunk)
{
    train(core, epoch, samples, chunk);
    std::string err;
    const auto status = core.finish_training(&err);
    BOOST_REQUIRE_MESSAGE(status == FixedToneTrainingStatus::Ok, err);
}

} // namespace

// ---------------------------------------------------------------------------
// Configuration / period resolution
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_prepare_resolves_validated_configuration)
{
    FixedToneCancelCore core;
    std::string err;
    BOOST_REQUIRE_MESSAGE(core.prepare(base_config(), &err), err);
    BOOST_CHECK(core.prepared());
    BOOST_CHECK(core.enabled());
    BOOST_CHECK_EQUAL(core.period_samples(), kPeriod);
    BOOST_CHECK_EQUAL(core.period_cycles(), -91);
    BOOST_CHECK_EQUAL(core.period_length_samples(), kPeriodLength);
    BOOST_CHECK_EQUAL(core.training_length_samples(), size_t(368640));
    BOOST_CHECK_EQUAL(core.max_fold_per_position(), size_t(1280));
    BOOST_CHECK_EQUAL(core.crossfade_samples(), size_t(256));
    BOOST_CHECK_CLOSE(core.omega_radians(), -2.0 * M_PI * 91.0 / 288.0, 1e-12);
    BOOST_CHECK(!core.has_model());
    BOOST_CHECK_EQUAL(core.active_kernel(), core.avx2_available() ? 2 : 1);
}

BOOST_AUTO_TEST_CASE(test_prepare_rejects_unsupported_configuration)
{
    {
        // 100 MHz / 737.28 MHz = 625/4608: the smallest exact period is 4608,
        // beyond the supported bound, so it must be rejected, never rounded to
        // a "close enough" 288-point table.
        FixedToneCancelConfig cfg = base_config();
        cfg.tone_frequency_hz = -100.0e6;
        FixedToneCancelCore core;
        std::string err;
        BOOST_CHECK(!core.prepare(cfg, &err));
        BOOST_CHECK(!err.empty());
        BOOST_CHECK(!core.prepared());
    }
    auto expect_reject = [](FixedToneCancelConfig cfg) {
        FixedToneCancelCore core;
        std::string err;
        BOOST_CHECK_MESSAGE(!core.prepare(cfg, &err), "config must be rejected");
        BOOST_CHECK(!err.empty());
    };
    {
        FixedToneCancelConfig cfg = base_config();
        cfg.sample_rate = 0.0;
        expect_reject(cfg);
    }
    {
        FixedToneCancelConfig cfg = base_config();
        cfg.tone_frequency_hz = 0.4 * cfg.sample_rate; // >= fs/2
        expect_reject(cfg);
    }
    {
        FixedToneCancelConfig cfg = base_config();
        cfg.training_s = cfg.period_s;
        expect_reject(cfg);
    }
    {
        FixedToneCancelConfig cfg = base_config();
        cfg.crossfade_samples = static_cast<size_t>(cfg.period_s * cfg.sample_rate) + 1;
        expect_reject(cfg);
    }
    {
        // 30 ms of training folds 76800 samples per position; the int32 fold
        // limit is 65535, so this must fail closed instead of overflowing.
        FixedToneCancelConfig cfg = base_config();
        cfg.period_s = 100.0e-3;
        cfg.training_s = 30.0e-3;
        expect_reject(cfg);
    }
}

BOOST_AUTO_TEST_CASE(test_disabled_core_is_pass_through)
{
    FixedToneCancelConfig cfg = base_config();
    cfg.enable = false;
    FixedToneCancelCore core;
    std::string err;
    BOOST_REQUIRE_MESSAGE(core.prepare(cfg, &err), err);
    BOOST_CHECK(core.prepared());
    BOOST_CHECK(!core.enabled());
    BOOST_CHECK(!core.begin_training(0, &err));

    std::mt19937 gen(7);
    std::vector<int16_t> src(2 * 4096);
    for (auto& v : src) {
        v = static_cast<int16_t>(gen());
    }
    std::vector<int16_t> dst(src.size(), 12345);
    const auto counts = core.copy_cancel(src.data(), dst.data(), 4096, 1000);
    BOOST_CHECK_EQUAL(counts.consumed, size_t(4096));
    BOOST_CHECK_EQUAL(counts.bypassed, size_t(4096));
    BOOST_CHECK_EQUAL(counts.cancelled, size_t(0));
    BOOST_CHECK(dst == src);
}

// ---------------------------------------------------------------------------
// Projection (training fold) against the direct double sum
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_fold_projection_matches_direct_sum)
{
    const double omega = 2.0 * M_PI * double(-91) / double(kPeriod);
    // 288 / 289 / 1000 (non-multiple of the period) / the production 0.5 ms.
    for (size_t n : { size_t(288), size_t(289), size_t(1000), size_t(368640) }) {
        const auto samples = make_tone_samples(n, 9000.0, 0.37, omega, 11 + unsigned(n), 40);
        FixedToneCancelCore core;
        std::string err;
        BOOST_REQUIRE_MESSAGE(core.prepare(short_config(n), &err), err);
        // Chunk size deliberately coprime with the 288-sample period.
        train_and_require_ok(core, 12345, samples, 97);
        BOOST_CHECK_EQUAL(core.training_samples_collected(), size_t(0));
        BOOST_REQUIRE(core.has_model());
        const auto expect = reference_coefficient(samples, n, omega);
        const auto got = core.model().coefficient;
        BOOST_CHECK_SMALL(std::abs(got - expect), 1e-9 * (1.0 + std::abs(expect)));
        BOOST_CHECK_EQUAL(core.model().epoch, int64_t(12345));
        BOOST_CHECK_EQUAL(core.model().generation, uint64_t(1));

        // The LUT must equal the nearest-even quantisation of c*exp(j w k).
        const auto lut = reference_lut(expect, omega);
        for (size_t k = 0; k < 2 * kPeriod; ++k) {
            BOOST_CHECK_EQUAL(core.tone_table()[k], lut[k]);
        }
    }
}

BOOST_AUTO_TEST_CASE(test_training_length_and_overflow_are_enforced)
{
    FixedToneCancelCore core;
    std::string err;
    BOOST_REQUIRE_MESSAGE(core.prepare(base_config(), &err), err);
    BOOST_CHECK(core.begin_training(0, &err));
    BOOST_CHECK(core.finish_training(&err) == FixedToneTrainingStatus::Incomplete);
    std::vector<int16_t> few(2 * 10, 0);
    BOOST_CHECK(core.add_training_samples(few.data(), 10, &err));
    BOOST_CHECK(core.finish_training(&err) == FixedToneTrainingStatus::Incomplete);
    // Overflow beyond training_s must be refused, not silently truncated.
    std::vector<int16_t> big(2 * 400000, 0);
    BOOST_CHECK(!core.add_training_samples(big.data(), 400000, &err));
    BOOST_CHECK(!core.add_training_samples(nullptr, 1, &err));
    // A fresh window accepts exactly the production 0.5 ms.
    BOOST_REQUIRE(core.begin_training(0, &err));
    std::vector<int16_t> full(2 * 368640, 0);
    BOOST_REQUIRE(core.add_training_samples(full.data(), 368640, &err));
    BOOST_CHECK(core.finish_training(&err) == FixedToneTrainingStatus::Ok);
}

// ---------------------------------------------------------------------------
// Output against the independent sample-by-sample reference
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_cancel_matches_reference_for_arbitrary_chunks)
{
    FixedToneCancelCore core;
    std::string err;
    BOOST_REQUIRE_MESSAGE(core.prepare(short_config(1000), &err), err);
    const double omega = core.omega_radians();
    const auto training = make_tone_samples(1000, 12000.0, -0.9, omega, 3, 25);
    train_and_require_ok(core, -5000, training, 288);
    const auto lut = reference_lut(core.model().coefficient, omega);

    std::mt19937 gen(99);
    const size_t n = 5000;
    std::vector<int16_t> src(2 * n);
    for (auto& v : src) {
        v = static_cast<int16_t>(gen());
    }
    // Cover the values just before the epoch (bypassed), the cross-fade region
    // of the second model, the periodic region, and large period multiples.
    for (int64_t start : { int64_t(-5000),
                           int64_t(-4744),
                           int64_t(14745600 - 100),
                           int64_t(14745600 + 37),
                           int64_t(29491200 + 289) }) {
        const auto expect = reference_apply(src, start, core.model().epoch, lut, {}, 0);
        for (size_t chunk : { size_t(1), size_t(7), size_t(288), size_t(289), size_t(4096) }) {
            std::vector<int16_t> dst(2 * n, 12345);
            size_t pos = 0;
            while (pos < n) {
                const size_t cnt = std::min(n - pos, chunk);
                const auto counts = core.copy_cancel(
                    src.data() + 2 * pos, dst.data() + 2 * pos, cnt, start + int64_t(pos));
                BOOST_REQUIRE_EQUAL(counts.consumed, cnt);
                BOOST_REQUIRE_EQUAL(counts.cancelled, cnt);
                BOOST_REQUIRE_EQUAL(counts.bypassed, size_t(0));
                pos += cnt;
            }
            BOOST_CHECK_MESSAGE(dst == expect,
                                "chunked cancellation diverged from the reference: start="
                                    << start << " chunk=" << chunk);
        }
        // Exactly one sample before the epoch can only be bypassed, never
        // corrected with a negative phase index.
        std::vector<int16_t> pre(2 * 4, 111);
        const auto pre_counts = core.copy_cancel(pre.data(), pre.data(), 4,
                                                 core.model().epoch - 4);
        BOOST_CHECK_EQUAL(pre_counts.bypassed, size_t(4));
        // In-place must be identical, and must not touch neighbours.
        std::vector<int16_t> guard(2 * n + 8, 777);
        std::copy(src.begin(), src.end(), guard.begin() + 4);
        core.cancel_in_place(guard.data() + 4, n, start);
        for (size_t k = 0; k < 2 * n; ++k) {
            BOOST_REQUIRE_EQUAL(guard[4 + k], expect[k]);
        }
        BOOST_CHECK_EQUAL(guard[0], 777);
        BOOST_CHECK_EQUAL(guard[1], 777);
        BOOST_CHECK_EQUAL(guard[3], 777);
        BOOST_CHECK_EQUAL(guard[4 + 2 * n], 777);
        BOOST_CHECK_EQUAL(guard[5 + 2 * n], 777);
    }
}

BOOST_AUTO_TEST_CASE(test_scalar_and_avx2_kernels_are_bit_identical)
{
    FixedToneCancelCore core;
    std::string err;
    BOOST_REQUIRE_MESSAGE(core.prepare(short_config(288 * 4), &err), err);
    const double omega = core.omega_radians();
    train_and_require_ok(core, 0, make_tone_samples(288 * 4, 20000.0, 0.2, omega, 5, 0), 288);

    std::mt19937 gen(1234);
    std::vector<int16_t> src(2 * 65536);
    for (auto& v : src) {
        v = static_cast<int16_t>(gen());
    }
    std::vector<int16_t> scalar(2 * 65536, 0);
    std::vector<int16_t> vector(2 * 65536, 0);
    core.set_kernel_mode_for_testing(1);
    BOOST_REQUIRE_EQUAL(core.active_kernel(), 1);
    core.copy_cancel(src.data(), scalar.data(), 65536, 7);
    const bool have_avx2 = core.avx2_available();
    core.set_kernel_mode_for_testing(2);
    BOOST_CHECK_EQUAL(core.active_kernel(), have_avx2 ? 2 : 1);
    core.copy_cancel(src.data(), vector.data(), 65536, 7);
    BOOST_CHECK_MESSAGE(scalar == vector, "scalar and AVX2 kernels must be bit identical");
    if (!have_avx2) {
        BOOST_TEST_MESSAGE("AVX2 unavailable on this CPU: scalar fallback compared with itself");
    }
    core.set_kernel_mode_for_testing(0);
    BOOST_CHECK_EQUAL(core.active_kernel(), have_avx2 ? 2 : 1);
}

BOOST_AUTO_TEST_CASE(test_int16_saturation)
{
    FixedToneCancelCore core;
    std::string err;
    BOOST_REQUIRE_MESSAGE(core.prepare(short_config(2880), &err), err);
    const double omega = core.omega_radians();
    // |c| ~ 28284 so the largest tone component approaches +-28284.
    train_and_require_ok(core, 0, make_tone_samples(2880, 28284.0, 0.0, omega, 17, 0), 288);
    const auto lut = reference_lut(core.model().coefficient, omega);

    const size_t n = 2 * kPeriod;
    std::vector<int16_t> src(2 * n);
    for (size_t k = 0; k < n; ++k) {
        src[2 * k] = (k % 2 == 0) ? int16_t(-32768) : int16_t(32767);
        src[2 * k + 1] = (k % 2 == 0) ? int16_t(32767) : int16_t(-32768);
    }
    const auto expect = reference_apply(src, 0, core.model().epoch, lut, {}, 0);
    std::vector<int16_t> got(2 * n, 0);
    core.copy_cancel(src.data(), got.data(), n, 0);
    BOOST_CHECK(got == expect);

    size_t saturated = 0;
    for (size_t k = 0; k < n; ++k) {
        const size_t idx = (k + 0) % kPeriod;
        for (size_t q = 0; q < 2; ++q) {
            const int v = int(src[2 * k + q]) - int(lut[2 * idx + q]);
            if (v < -32768 || v > 32767) {
                ++saturated;
            }
        }
    }
    BOOST_CHECK_GT(saturated, size_t(0));
}

// ---------------------------------------------------------------------------
// Quantisation contract
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_quantizer_nearest_even_and_fenv_independent)
{
    struct Case {
        double v;
        bool ok;
        int16_t want;
    };
    const Case cases[] = {
        { 0.5, true, 0 },        { 1.5, true, 2 },     { 2.5, true, 2 },
        { 3.5, true, 4 },        { -0.5, true, 0 },    { -1.5, true, -2 },
        { -2.5, true, -2 },      { -3.5, true, -4 },   { 0.4999, true, 0 },
        { 0.5001, true, 1 },     { 32767.4, true, 32767 },
        { 32767.6, false, 0 },   { -32768.4, true, -32768 },
        { -32768.6, false, 0 },  { 32768.0, false, 0 },
        { -32769.0, false, 0 },  { std::nan(""), false, 0 },
        { INFINITY, false, 0 },  { -INFINITY, false, 0 },
    };
    const int prior = std::fegetround();
    for (int mode : { FE_TONEAREST, FE_DOWNWARD, FE_UPWARD, FE_TOWARDZERO }) {
        std::fesetround(mode);
        for (const auto& c : cases) {
            int16_t out = 1234;
            const bool ok = FixedToneCancelCore::quantize_tone(c.v, &out);
            BOOST_REQUIRE_EQUAL(ok, c.ok);
            if (ok) {
                BOOST_REQUIRE_EQUAL(out, c.want);
            } else {
                BOOST_REQUIRE_EQUAL(out, 1234);
            }
        }
    }
    std::fesetround(prior);

    // The built table must not depend on the process rounding mode either.
    const auto samples = make_tone_samples(288, 4321.0, 1.2345, -2.0 * M_PI * 91.0 / 288.0,
                                           61, 0);
    std::vector<int16_t> lut_ref;
    for (int mode : { FE_TONEAREST, FE_DOWNWARD, FE_UPWARD, FE_TOWARDZERO }) {
        std::fesetround(mode);
        FixedToneCancelCore core;
        std::string err;
        BOOST_REQUIRE_MESSAGE(core.prepare(short_config(288), &err), err);
        train_and_require_ok(core, 0, samples, 288);
        std::vector<int16_t> lut(core.tone_table(), core.tone_table() + 2 * kPeriod);
        if (lut_ref.empty()) {
            lut_ref = lut;
        } else {
            BOOST_CHECK_MESSAGE(lut == lut_ref, "LUT depends on the process rounding mode");
        }
    }
    std::fesetround(prior);
}

BOOST_AUTO_TEST_CASE(test_invalid_model_keeps_previous_table)
{
    // A one-sample training window whose sample saturates both components
    // produces a coefficient whose rotated components cannot fit int16.
    FixedToneCancelConfig cfg = short_config(1);
    FixedToneCancelCore core;
    std::string err;
    BOOST_REQUIRE_MESSAGE(core.prepare(cfg, &err), err);

    const int16_t good[2] = { 4000, 0 };
    BOOST_REQUIRE(core.begin_training(0, &err));
    BOOST_REQUIRE(core.add_training_samples(good, 1, &err));
    BOOST_REQUIRE(core.finish_training(&err) == FixedToneTrainingStatus::Ok);
    std::vector<int16_t> before(core.tone_table(), core.tone_table() + 2 * kPeriod);
    const auto generation = core.model().generation;

    const int16_t bad[2] = { 32767, 32767 };
    BOOST_REQUIRE(core.begin_training(1000, &err));
    BOOST_REQUIRE(core.add_training_samples(bad, 1, &err));
    BOOST_CHECK(core.finish_training(&err) == FixedToneTrainingStatus::InvalidModel);
    BOOST_CHECK(!err.empty());
    BOOST_CHECK(core.has_model());
    BOOST_CHECK_EQUAL(core.model().generation, generation);
    std::vector<int16_t> after(core.tone_table(), core.tone_table() + 2 * kPeriod);
    BOOST_CHECK(before == after);
}

// ---------------------------------------------------------------------------
// Cross-fade
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_crossfade_matches_reference_and_loses_no_samples)
{
    FixedToneCancelCore core;
    std::string err;
    BOOST_REQUIRE_MESSAGE(core.prepare(short_config(288), &err), err);
    const double omega = core.omega_radians();

    // Model A at epoch 0.
    train_and_require_ok(core, 0, make_tone_samples(288, 8000.0, 0.1, omega, 31, 0), 288);
    const auto lut_a = reference_lut(core.model().coefficient, omega);
    BOOST_CHECK_EQUAL(core.crossfade_table_samples(), size_t(0)); // no previous model

    // Model B at 20 ms, blended from A over the first 256 samples.
    train_and_require_ok(core, kPeriodLength,
                         make_tone_samples(288, 12000.0, 2.1, omega, 32, 0), 288);
    const auto lut_b = reference_lut(core.model().coefficient, omega);
    BOOST_CHECK_EQUAL(core.crossfade_table_samples(), size_t(256));
    BOOST_CHECK_EQUAL(core.crossfade_epoch(), kPeriodLength);
    const auto blend = reference_blend(lut_a, 0, lut_b, kPeriodLength, 256);
    for (size_t j = 0; j < 2 * 256; ++j) {
        BOOST_REQUIRE_EQUAL(core.crossfade_table()[j], blend[j]);
    }

    std::mt19937 gen(555);
    const size_t n = 800;
    std::vector<int16_t> src(2 * n);
    for (auto& v : src) {
        v = static_cast<int16_t>(gen());
    }
    const auto expect =
        reference_apply(src, kPeriodLength, kPeriodLength, lut_b, blend, 256);
    std::vector<int16_t> dst(2 * n, 0);
    const auto counts = core.copy_cancel(src.data(), dst.data(), n, kPeriodLength);
    BOOST_CHECK_EQUAL(counts.consumed, n);
    BOOST_CHECK_EQUAL(counts.cancelled, n);
    BOOST_CHECK(dst == expect);

    // crossfade = 0 selects a hard switch (mathematical reference mode).
    FixedToneCancelCore hard;
    BOOST_REQUIRE_MESSAGE(hard.prepare(short_config(288, 0), &err), err);
    train_and_require_ok(hard, 0, make_tone_samples(288, 9000.0, 0.0, omega, 41, 0), 288);
    const auto lut_h1 = reference_lut(hard.model().coefficient, omega);
    train_and_require_ok(hard, kPeriodLength,
                         make_tone_samples(288, 5000.0, 1.0, omega, 42, 0), 288);
    BOOST_CHECK_EQUAL(hard.crossfade_table_samples(), size_t(0));
    const auto lut_h2 = reference_lut(hard.model().coefficient, omega);
    std::vector<int16_t> dst2(2 * n, 0);
    hard.copy_cancel(src.data(), dst2.data(), n, kPeriodLength);
    const auto expect2 = reference_apply(src, kPeriodLength, kPeriodLength, lut_h2, {}, 0);
    BOOST_CHECK(dst2 == expect2);
    BOOST_CHECK(lut_h1 != lut_h2);
}

BOOST_AUTO_TEST_CASE(test_invalidate_and_reset_force_pass_through)
{
    FixedToneCancelCore core;
    std::string err;
    BOOST_REQUIRE_MESSAGE(core.prepare(short_config(288), &err), err);
    const double omega = core.omega_radians();
    train_and_require_ok(core, 0, make_tone_samples(288, 7000.0, 0.0, omega, 51, 0), 288);

    std::vector<int16_t> src(2 * 64, 1234);
    std::vector<int16_t> dst(2 * 64, 0);
    core.invalidate_model();
    BOOST_CHECK(!core.has_model());
    const auto counts = core.copy_cancel(src.data(), dst.data(), 64, 10);
    BOOST_CHECK_EQUAL(counts.bypassed, size_t(64));
    BOOST_CHECK(dst == src);

    // reset() keeps the prepared configuration but drops everything else.
    core.reset();
    BOOST_CHECK(core.prepared());
    BOOST_CHECK(!core.has_model());
    BOOST_CHECK(!core.training_active());
    core.copy_cancel(src.data(), dst.data(), 64, 10);
    BOOST_CHECK(dst == src);
    // ... and a model can be rebuilt afterwards.
    train_and_require_ok(core, 0, make_tone_samples(288, 7000.0, 0.0, omega, 51, 0), 288);
    BOOST_CHECK(core.has_model());
}
