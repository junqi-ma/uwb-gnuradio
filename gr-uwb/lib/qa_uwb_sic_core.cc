/* -*- c++ -*- */
/* SPDX-License-Identifier: GPL-3.0-or-later */
#define BOOST_TEST_MODULE qa_uwb_sic_core
#include <boost/test/included/unit_test.hpp>
#include <gnuradio/uwb/uwb_demod_core.h>
#include <gnuradio/uwb/uwb_sic_core.h>
#include <gnuradio/uwb/uwb_tx_reconstructor.h>
#include <cmath>
#include <complex>
#include <fstream>
#include <string>
#include <vector>

namespace {
using gr::uwb::sic::CancelOptions;
using gr::uwb::sic::CancelScratch;
using gr::uwb::sic::CancelStatus;
using gr::uwb::sic::ReconstructStatus;
using gr::uwb::sic::TxReconstruction;
using gr::uwb::sic::TxReconstructionScratch;

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

BOOST_AUTO_TEST_CASE(dw1000_rs_encoder_roundtrips_partial_and_multiple_blocks)
{
    for (size_t byte_count : { size_t(2), size_t(12), size_t(42), size_t(128) }) {
        std::vector<uint8_t> psdu(byte_count);
        for (size_t i = 0; i < psdu.size(); ++i)
            psdu[i] = static_cast<uint8_t>((i * 73 + byte_count * 11) & 0xff);
        std::vector<int8_t> coded, source_bits, decoded_bits;
        BOOST_REQUIRE(gr::uwb::sic::tx_detail::rs_encode_stream(
            psdu, coded, source_bits));
        BOOST_REQUIRE(gr::uwb::demod::core::detail::rs_decode_stream(
            coded, psdu.size() * 8, decoded_bits));
        BOOST_CHECK(decoded_bits == source_bits);
    }
}

BOOST_AUTO_TEST_CASE(real_dw1000_replica_and_trial_match_matlab)
{
    const std::string dir = "../../../testdata/dw1000_realtime_golden";
    std::vector<std::complex<float>> window, replica, golden_received;
    std::vector<std::complex<float>> golden_model, golden_residual;
    std::vector<std::complex<float>> reference, golden_cir;
    std::vector<float> impulses;
    BOOST_REQUIRE(load_cf32(dir + "/window.cfile", window));
    BOOST_REQUIRE(load_cf32(dir + "/tx_cir_replica.cfile", replica));
    BOOST_REQUIRE(load_cf32(dir + "/trial_received.cfile", golden_received));
    BOOST_REQUIRE(load_cf32(dir + "/trial_model.cfile", golden_model));
    BOOST_REQUIRE(load_cf32(dir + "/trial_residual.cfile", golden_residual));
    BOOST_REQUIRE(load_cf32(dir + "/reference_preamble.cfile", reference));
    BOOST_REQUIRE(load_cf32(dir + "/stage_cir.cfile", golden_cir));
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

    auto profile = gr::uwb::demod::Dw1000Profile::Default().as_qm35825();
    gr::uwb::demod::core::DemodScratch demod_scratch;
    demod_scratch.reserve(window.size());
    const auto decoded = gr::uwb::demod::core::demodulate_one(
        window.data(), window.size(), profile, 1,
        333687 /* absolute predicted start */,
        323687 /* absolute window start */, reference, demod_scratch);
    BOOST_REQUIRE(decoded.status == gr::uwb::demod::DemodStatus::Success);
    BOOST_REQUIRE(decoded.payload.fcs_pass);

    TxReconstruction matlab_channel;
    matlab_channel.reserve(replica.size());
    TxReconstructionScratch tx_scratch;
    tx_scratch.reserve(128);
    BOOST_REQUIRE(gr::uwb::sic::reconstruct_dw1000(
        decoded.phr.phr_bits, decoded.payload.bytes, decoded.payload.fcs_pass,
        golden_cir, 8, matlab_channel, tx_scratch));
    BOOST_CHECK(static_cast<int>(matlab_channel.status) ==
                static_cast<int>(ReconstructStatus::Success));
    BOOST_REQUIRE_EQUAL(matlab_channel.pulse_impulses.size(), impulses.size());
    BOOST_REQUIRE_EQUAL(matlab_channel.replica.size(), replica.size());
    BOOST_CHECK(matlab_channel.pulse_impulses == impulses);
    BOOST_CHECK_EQUAL(matlab_channel.sync.begin, size_t(0));
    BOOST_CHECK_EQUAL(matlab_channel.sync.end, size_t(130048));
    BOOST_CHECK_EQUAL(matlab_channel.sfd.begin, size_t(130048));
    BOOST_CHECK_EQUAL(matlab_channel.sfd.end, size_t(138176));
    BOOST_CHECK_EQUAL(matlab_channel.phr.begin, size_t(138176));
    BOOST_CHECK_EQUAL(matlab_channel.phr.end, size_t(159680));
    BOOST_CHECK_EQUAL(matlab_channel.payload.begin, size_t(159680));
    BOOST_CHECK_EQUAL(matlab_channel.payload.end, size_t(178112));

    float reconstructed_replica_max_error = 0.0f;
    double reconstructed_error_energy = 0.0;
    double golden_replica_energy = 0.0;
    for (size_t i = 0; i < replica.size(); ++i) {
        const auto error = matlab_channel.replica[i] - replica[i];
        reconstructed_replica_max_error = std::max(
            reconstructed_replica_max_error, std::abs(error));
        reconstructed_error_energy += std::norm(error);
        golden_replica_energy += std::norm(replica[i]);
    }
    const double reconstructed_replica_relative_l2 =
        std::sqrt(reconstructed_error_energy / golden_replica_energy);
    BOOST_TEST_MESSAGE("C++ DW1000 reconstruction replica_max_error="
                       << reconstructed_replica_max_error
                       << " relative_l2=" << reconstructed_replica_relative_l2);
    BOOST_CHECK_LT(reconstructed_replica_relative_l2, 2e-6);
    BOOST_CHECK_LT(reconstructed_replica_max_error, 2e-6f);

    TxReconstruction cpp_channel;
    cpp_channel.reserve(replica.size());
    BOOST_REQUIRE(gr::uwb::sic::reconstruct_dw1000(
        decoded.phr.phr_bits, decoded.payload.bytes, decoded.payload.fcs_pass,
        decoded.cir.cir_complex_values, decoded.cir.pre_samples, cpp_channel,
        tx_scratch));
    BOOST_CHECK(cpp_channel.pulse_impulses == impulses);

    const size_t psdu_bits_capacity = tx_scratch.psdu_bits.capacity();
    const size_t rs_capacity = tx_scratch.rs_bits.capacity();
    const size_t encoder_capacity = tx_scratch.encoder_input.capacity();
    const size_t spread_capacity = tx_scratch.spread.capacity();
    const size_t lfsr_capacity = tx_scratch.lfsr_sequence.capacity();
    TxReconstruction reused_channel;
    reused_channel.reserve(replica.size());
    BOOST_REQUIRE(gr::uwb::sic::reconstruct_dw1000(
        decoded.phr.phr_bits, decoded.payload.bytes, decoded.payload.fcs_pass,
        decoded.cir.cir_complex_values, decoded.cir.pre_samples,
        reused_channel, tx_scratch));
    BOOST_CHECK_EQUAL(tx_scratch.psdu_bits.capacity(), psdu_bits_capacity);
    BOOST_CHECK_EQUAL(tx_scratch.rs_bits.capacity(), rs_capacity);
    BOOST_CHECK_EQUAL(tx_scratch.encoder_input.capacity(), encoder_capacity);
    BOOST_CHECK_EQUAL(tx_scratch.spread.capacity(), spread_capacity);
    BOOST_CHECK_EQUAL(tx_scratch.lfsr_sequence.capacity(), lfsr_capacity);

    auto corrupt_psdu = decoded.payload.bytes;
    corrupt_psdu.front() ^= 1;
    TxReconstruction rejected_tx;
    TxReconstructionScratch rejected_tx_scratch;
    rejected_tx_scratch.reserve(128);
    BOOST_CHECK(!gr::uwb::sic::reconstruct_dw1000(
        decoded.phr.phr_bits, corrupt_psdu, true,
        decoded.cir.cir_complex_values, decoded.cir.pre_samples,
        rejected_tx, rejected_tx_scratch));
    BOOST_CHECK(static_cast<int>(rejected_tx.status) ==
                static_cast<int>(ReconstructStatus::FcsFailed));
    BOOST_CHECK(rejected_tx.pulse_impulses.empty());
    BOOST_CHECK(rejected_tx.replica.empty());

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

    auto cpp_reconstructed_work = before_trial;
    CancelScratch cpp_reconstructed_scratch;
    cpp_reconstructed_scratch.reserve(cpp_channel.replica.size(), 128);
    const auto cpp_reconstructed_trial = gr::uwb::sic::trial_cancel(
        cpp_reconstructed_work, cpp_channel.replica.data(),
        cpp_channel.replica.size(), start, decoded.payload.fcs_pass, options,
        cpp_reconstructed_scratch);
    BOOST_REQUIRE(static_cast<int>(cpp_reconstructed_trial.status) ==
                  static_cast<int>(CancelStatus::Applied));
    BOOST_CHECK_EQUAL(cpp_reconstructed_trial.fitted_start, start);
    BOOST_CHECK_GT(cpp_reconstructed_trial.alignment_correlation, 0.97f);
    BOOST_CHECK_SMALL(cpp_reconstructed_trial.fitted_cfo_hz - (-2667.70901808),
                      5.0);
    BOOST_CHECK_GT(cpp_reconstructed_trial.suppression_db, 10.0f);
    float cpp_pipeline_residual_max_error = 0.0f;
    double cpp_pipeline_residual_error_energy = 0.0;
    double golden_residual_energy = 0.0;
    for (size_t i = 0; i < cpp_channel.replica.size(); ++i) {
        const auto error = cpp_reconstructed_work[start + i] -
                           golden_residual[i];
        cpp_pipeline_residual_max_error = std::max(
            cpp_pipeline_residual_max_error, std::abs(error));
        cpp_pipeline_residual_error_energy += std::norm(error);
        golden_residual_energy += std::norm(golden_residual[i]);
    }
    const double cpp_pipeline_residual_relative_l2 =
        std::sqrt(cpp_pipeline_residual_error_energy / golden_residual_energy);
    BOOST_TEST_MESSAGE("C++ decode/reconstruct/cancel suppression="
                       << cpp_reconstructed_trial.suppression_db
                       << " residual_max_error="
                       << cpp_pipeline_residual_max_error
                       << " residual_relative_l2="
                       << cpp_pipeline_residual_relative_l2);
    // The exact-impulse and MATLAB-CIR tests above prevent profile/index
    // errors from hiding here. This end-to-end branch uses the independently
    // estimated C++ CIR (raw tap max error <0.02), whose error is amplified by
    // the fitted ~13.7k complex gain; bound that propagation separately.
    BOOST_CHECK_LT(cpp_pipeline_residual_max_error, 50.0f);
    BOOST_CHECK_LT(cpp_pipeline_residual_relative_l2, 0.03);
}
