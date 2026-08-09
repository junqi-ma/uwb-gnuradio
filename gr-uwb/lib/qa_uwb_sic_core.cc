/* -*- c++ -*- */
/* SPDX-License-Identifier: GPL-3.0-or-later */
#define BOOST_TEST_MODULE qa_uwb_sic_core
#include <boost/test/included/unit_test.hpp>
#include <gnuradio/uwb/uwb_sic_core.h>
#include <cmath>
#include <complex>
#include <fstream>
#include <string>
#include <vector>

namespace {
using gr::uwb::sic::CancelOptions;
using gr::uwb::sic::CancelScratch;
using gr::uwb::sic::CancelStatus;

bool load_cf32(const std::string& path,
               std::vector<std::complex<float>>& out)
{
    std::ifstream f(path, std::ios::binary);
    if (!f)
        return false;
    f.seekg(0, std::ios::end);
    const size_t bytes = static_cast<size_t>(f.tellg());
    f.seekg(0, std::ios::beg);
    if (bytes == 0 || bytes % sizeof(std::complex<float>) != 0)
        return false;
    out.resize(bytes / sizeof(std::complex<float>));
    f.read(reinterpret_cast<char*>(out.data()),
           static_cast<std::streamsize>(bytes));
    return static_cast<bool>(f);
}

bool load_f32(const std::string& path, std::vector<float>& out)
{
    std::ifstream f(path, std::ios::binary);
    if (!f)
        return false;
    f.seekg(0, std::ios::end);
    const size_t bytes = static_cast<size_t>(f.tellg());
    f.seekg(0, std::ios::beg);
    if (bytes == 0 || bytes % sizeof(float) != 0)
        return false;
    out.resize(bytes / sizeof(float));
    f.read(reinterpret_cast<char*>(out.data()),
           static_cast<std::streamsize>(bytes));
    return static_cast<bool>(f);
}

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

BOOST_AUTO_TEST_CASE(real_dw1000_replica_and_trial_match_matlab)
{
    const std::string dir = "../../../testdata/dw1000_realtime_golden";
    std::vector<std::complex<float>> window, replica, golden_received;
    std::vector<std::complex<float>> golden_model, golden_residual;
    std::vector<float> impulses;
    BOOST_REQUIRE(load_cf32(dir + "/window.cfile", window));
    BOOST_REQUIRE(load_cf32(dir + "/tx_cir_replica.cfile", replica));
    BOOST_REQUIRE(load_cf32(dir + "/trial_received.cfile", golden_received));
    BOOST_REQUIRE(load_cf32(dir + "/trial_model.cfile", golden_model));
    BOOST_REQUIRE(load_cf32(dir + "/trial_residual.cfile", golden_residual));
    BOOST_REQUIRE(load_f32(dir + "/tx_pulse_impulses.f32", impulses));
    BOOST_REQUIRE_EQUAL(window.size(), size_t(198140));
    BOOST_REQUIRE_EQUAL(replica.size(), size_t(178112));
    BOOST_REQUIRE_EQUAL(impulses.size(), replica.size());
    BOOST_REQUIRE_EQUAL(golden_received.size(), replica.size());
    BOOST_REQUIRE_EQUAL(golden_model.size(), replica.size());
    BOOST_REQUIRE_EQUAL(golden_residual.size(), replica.size());

    std::ifstream fields(dir + "/field_bounds.csv");
    BOOST_REQUIRE(fields.good());
    std::vector<std::string> field_lines;
    for (std::string line; std::getline(fields, line);) {
        if (!line.empty() && line.back() == '\r')
            line.pop_back();
        field_lines.push_back(line);
    }
    const std::vector<std::string> expected_fields = {
        "field,begin_0based,end_exclusive_0based",
        "SYNC,0,130048",
        "SFD,130048,138176",
        "PHR,138176,159680",
        "Payload,159680,178112"
    };
    BOOST_CHECK(field_lines == expected_fields);

    size_t nonzero_impulses = 0;
    for (float value : impulses) {
        BOOST_REQUIRE(std::isfinite(value));
        BOOST_REQUIRE(value == -1.0f || value == 0.0f || value == 1.0f);
        nonzero_impulses += value != 0.0f;
    }
    BOOST_CHECK_GT(nonzero_impulses, size_t(8000));

    constexpr size_t start = 10000;
    float received_max_error = 0.0f;
    float golden_identity_max_error = 0.0f;
    for (size_t i = 0; i < replica.size(); ++i) {
        received_max_error = std::max(
            received_max_error,
            std::abs(golden_received[i] - window[start + i]));
        golden_identity_max_error = std::max(
            golden_identity_max_error,
            std::abs(golden_residual[i] -
                     (golden_received[i] - golden_model[i])));
    }
    BOOST_CHECK_EQUAL(received_max_error, 0.0f);
    BOOST_CHECK_LT(golden_identity_max_error, 0.01f);

    CancelOptions options;
    options.sample_rate = 998.4e6;
    options.search_radius = 128;
    options.period_samples = 1016;
    options.alignment_first_repetition = 24;
    options.alignment_repetitions = 32;
    options.cfo_first_repetition = 24;
    options.cfo_last_repetition = 128;
    options.gain_first_repetition = 24;
    options.gain_last_repetition = 128;
    options.min_alignment_correlation = 0.70f;
    options.min_suppression_db = 0.20f;
    options.max_abs_cfo_hz = 100000.0;

    CancelScratch scratch;
    scratch.reserve(replica.size(), 128);
    const auto before_trial = window;
    const auto result = gr::uwb::sic::trial_cancel(
        window, replica.data(), replica.size(), start, true, options, scratch);
    BOOST_REQUIRE(static_cast<int>(result.status) ==
                  static_cast<int>(CancelStatus::Applied));
    BOOST_REQUIRE(result.sic_applied);
    BOOST_CHECK_EQUAL(result.fitted_start, start);
    BOOST_CHECK_EQUAL(result.samples_subtracted, replica.size());
    BOOST_CHECK_SMALL(result.alignment_correlation - 0.980969548225f, 2e-5f);
    BOOST_CHECK_SMALL(result.fitted_cfo_hz - (-2667.70901808), 0.2);
    BOOST_CHECK_LT(std::abs(result.global_gain -
                            std::complex<float>(-2620.63720703f,
                                                -13453.2910156f)),
                   2.0f);
    BOOST_CHECK_SMALL(result.suppression_db - 16.2803249359f, 0.01f);

    float model_max_error = 0.0f;
    float residual_max_error = 0.0f;
    float outside_interval_max_error = 0.0f;
    double golden_model_energy = 0.0;
    double model_error_energy = 0.0;
    for (size_t i = 0; i < replica.size(); ++i) {
        const std::complex<float> cpp_model =
            result.global_gain * scratch.model[i];
        const std::complex<float> model_error = cpp_model - golden_model[i];
        model_max_error = std::max(model_max_error, std::abs(model_error));
        residual_max_error = std::max(
            residual_max_error,
            std::abs(window[start + i] - golden_residual[i]));
        golden_model_energy += std::norm(golden_model[i]);
        model_error_energy += std::norm(model_error);
    }
    for (size_t i = 0; i < start; ++i)
        outside_interval_max_error = std::max(
            outside_interval_max_error, std::abs(window[i] - before_trial[i]));
    for (size_t i = start + replica.size(); i < window.size(); ++i)
        outside_interval_max_error = std::max(
            outside_interval_max_error, std::abs(window[i] - before_trial[i]));
    const double relative_model_l2 =
        std::sqrt(model_error_energy / golden_model_energy);
    BOOST_TEST_MESSAGE("real DW1000 TX replica model_max_error="
                       << model_max_error
                       << " residual_max_error=" << residual_max_error
                       << " outside_interval_max_error="
                       << outside_interval_max_error
                       << " relative_model_l2=" << relative_model_l2);
    BOOST_CHECK_LT(relative_model_l2, 2e-4);
    BOOST_CHECK_LT(model_max_error, 4.0f);
    BOOST_CHECK_LT(residual_max_error, 4.0f);
    BOOST_CHECK_EQUAL(outside_interval_max_error, 0.0f);

    const auto require_exact_bypass = [&](CancelOptions rejected_options,
                                          bool fcs_pass,
                                          CancelStatus expected_status) {
        auto rejected_work = before_trial;
        CancelScratch rejected_scratch;
        rejected_scratch.reserve(replica.size(), 128);
        const auto rejected = gr::uwb::sic::trial_cancel(
            rejected_work, replica.data(), replica.size(), start, fcs_pass,
            rejected_options, rejected_scratch);
        BOOST_CHECK(static_cast<int>(rejected.status) ==
                    static_cast<int>(expected_status));
        BOOST_CHECK(!rejected.sic_applied);
        BOOST_CHECK_EQUAL(rejected.samples_subtracted, size_t(0));
        BOOST_CHECK(rejected_work == before_trial);
    };

    require_exact_bypass(options, false, CancelStatus::FcsFailed);
    auto correlation_reject = options;
    correlation_reject.min_alignment_correlation = 0.99f;
    require_exact_bypass(
        correlation_reject, true, CancelStatus::AlignmentFailed);
    auto suppression_reject = options;
    suppression_reject.min_suppression_db = 17.0f;
    require_exact_bypass(
        suppression_reject, true, CancelStatus::SuppressionFailed);
}
