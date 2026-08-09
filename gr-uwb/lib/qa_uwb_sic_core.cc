/* -*- c++ -*- */
/* SPDX-License-Identifier: GPL-3.0-or-later */
#define BOOST_TEST_MODULE qa_uwb_sic_core
#include <boost/test/included/unit_test.hpp>
#include <gnuradio/uwb/uwb_sic_core.h>
#include <cmath>
#include <complex>
#include <vector>

namespace {
using gr::uwb::sic::CancelOptions;
using gr::uwb::sic::CancelScratch;
using gr::uwb::sic::CancelStatus;

std::vector<std::complex<float>> make_replica(size_t repetitions,
                                               size_t period)
{
    std::vector<std::complex<float>> replica(repetitions * period);
    for (size_t r = 0; r < repetitions; ++r) {
        for (size_t i = 0; i < period; ++i) {
            const float a = ((i * 17 + i * i * 3) % 29 < 9) ? 1.0f : 0.0f;
            const float sign = ((i * 7 + 3) % 11 < 5) ? 1.0f : -1.0f;
            replica[r * period + i] = { a * sign, a * 0.21f };
        }
    }
    return replica;
}

CancelOptions options_for_test()
{
    CancelOptions o;
    o.sample_rate = 1.0e6;
    o.search_radius = 6;
    o.period_samples = 64;
    o.alignment_first_repetition = 3;
    o.alignment_repetitions = 4;
    o.cfo_first_repetition = 3;
    o.cfo_last_repetition = 16;
    o.gain_first_repetition = 3;
    o.gain_last_repetition = 16;
    o.max_abs_cfo_hz = 5000.0;
    return o;
}
} // namespace

BOOST_AUTO_TEST_CASE(applies_integer_cfo_gain_cancel)
{
    const auto replica = make_replica(18, 64);
    std::vector<std::complex<float>> work(1600, { 0.002f, -0.001f });
    const size_t actual_start = 217;
    const double cfo_hz = 731.0;
    const std::complex<float> gain(0.43f, -0.27f);
    const auto options = options_for_test();
    for (size_t i = 0; i < replica.size(); ++i) {
        const double phase = 2.0 * M_PI * cfo_hz * i / options.sample_rate;
        const std::complex<float> rot(std::cos(phase), std::sin(phase));
        work[actual_start + i] += gain * replica[i] * rot;
    }
    const auto before = work;
    CancelScratch scratch;
    scratch.reserve(replica.size(), 18);
    const auto result = gr::uwb::sic::trial_cancel(
        work, replica.data(), replica.size(), 220, true, options, scratch);

    BOOST_TEST(static_cast<int>(result.status) ==
               static_cast<int>(CancelStatus::Applied));
    BOOST_TEST(result.sic_applied);
    BOOST_TEST(result.fitted_start == actual_start);
    BOOST_TEST(result.alignment_correlation > 0.90f);
    BOOST_TEST(result.fitted_cfo_hz == cfo_hz, boost::test_tools::tolerance(0.5));
    BOOST_TEST(std::abs(result.global_gain - gain) < 2e-3f);
    BOOST_TEST(result.suppression_db > 30.0f);
    BOOST_TEST(result.samples_subtracted == replica.size());
    BOOST_TEST(work != before);
}

BOOST_AUTO_TEST_CASE(fcs_failure_is_exact_bypass)
{
    const auto replica = make_replica(18, 64);
    std::vector<std::complex<float>> work(1500, { 0.1f, -0.2f });
    const auto before = work;
    CancelScratch scratch;
    const auto result = gr::uwb::sic::trial_cancel(
        work, replica.data(), replica.size(), 100, false,
        options_for_test(), scratch);
    BOOST_TEST(static_cast<int>(result.status) ==
               static_cast<int>(CancelStatus::FcsFailed));
    BOOST_TEST(!result.sic_applied);
    BOOST_TEST(work == before);
}

BOOST_AUTO_TEST_CASE(low_alignment_is_exact_bypass)
{
    const auto replica = make_replica(18, 64);
    std::vector<std::complex<float>> work(1500);
    for (size_t i = 0; i < work.size(); ++i)
        work[i] = { static_cast<float>((i * 13) % 31) / 31.0f,
                    static_cast<float>((i * 19) % 37) / 37.0f };
    const auto before = work;
    CancelScratch scratch;
    auto options = options_for_test();
    options.min_alignment_correlation = 0.95f;
    const auto result = gr::uwb::sic::trial_cancel(
        work, replica.data(), replica.size(), 100, true, options, scratch);
    BOOST_TEST(static_cast<int>(result.status) ==
               static_cast<int>(CancelStatus::AlignmentFailed));
    BOOST_TEST(!result.sic_applied);
    BOOST_TEST(work == before);
}

BOOST_AUTO_TEST_CASE(low_suppression_is_exact_bypass)
{
    const auto replica = make_replica(18, 64);
    std::vector<std::complex<float>> work(1500, { 0.0f, 0.0f });
    const size_t start = 100;
    for (size_t i = 0; i < replica.size(); ++i)
        work[start + i] = replica[i] + std::complex<float>(3.0f, -2.0f);
    const auto before = work;
    CancelScratch scratch;
    auto options = options_for_test();
    options.min_alignment_correlation = 0.0f;
    options.min_suppression_db = 20.0f;
    const auto result = gr::uwb::sic::trial_cancel(
        work, replica.data(), replica.size(), start, true, options, scratch);
    BOOST_TEST(static_cast<int>(result.status) ==
               static_cast<int>(CancelStatus::SuppressionFailed));
    BOOST_TEST(!result.sic_applied);
    BOOST_TEST(work == before);
}

BOOST_AUTO_TEST_CASE(short_context_is_exact_bypass)
{
    const auto replica = make_replica(18, 64);
    std::vector<std::complex<float>> work(64, { 0.2f, -0.1f });
    const auto before = work;
    CancelScratch scratch;
    const auto result = gr::uwb::sic::trial_cancel(
        work, replica.data(), replica.size(), 0, true,
        options_for_test(), scratch);

    BOOST_TEST(static_cast<int>(result.status) ==
               static_cast<int>(CancelStatus::AlignmentFailed));
    BOOST_TEST(!result.sic_applied);
    BOOST_TEST(result.samples_subtracted == 0U);
    BOOST_TEST(work == before);
}
