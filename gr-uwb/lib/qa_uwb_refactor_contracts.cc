// SPDX-License-Identifier: GPL-3.0-or-later
// Deliberately uses the packet API, not the stage aggregate.
#include <boost/test/unit_test.hpp>
#include <gnuradio/uwb/uwb_hrp_codec.h>
#include <gnuradio/uwb/uwb_demod_execution.h>
#include <gnuradio/uwb/uwb_demod_finalize.h>
#include <gnuradio/uwb/uwb_demod_pipeline.h>
#include <gnuradio/uwb/uwb_resample_window.h>
#include "../apps/uwb_live_packet_processor.h"

using namespace gr::uwb;

BOOST_AUTO_TEST_CASE(shared_codec_known_crc_and_field_inverse)
{
    const uint8_t bytes[] = {'1','2','3','4','5','6','7','8','9'};
    BOOST_CHECK_EQUAL(codec::crc16_802154(bytes, 9), 0x2189);
    for (int i = 1; i < 64; ++i)
        BOOST_CHECK_EQUAL(codec::rs_gf_mul(i, codec::rs_gf_inv(i)), 1);
}

BOOST_AUTO_TEST_CASE(packet_initialization_keeps_templates_and_workspace)
{
    demod::core::DemodScratch scratch;
    scratch.reserve(100);
    scratch.fused_phr.g0 = {1, 0};
    scratch.fused_phr.llr0 = {1.f};
    scratch.fused_phr.decisions_valid = true;
    scratch.fused_phr.pn = {1, -1};
    scratch.sfd_template_key = 42;
    const auto capacity = scratch.work.capacity();
    scratch.begin_packet();
    BOOST_CHECK(!scratch.fused_phr.decisions_valid);
    BOOST_CHECK(scratch.fused_phr.g0.empty());
    BOOST_CHECK(scratch.fused_phr.llr0.empty());
    BOOST_CHECK_EQUAL(scratch.fused_phr.pn.size(), 2u);
    BOOST_CHECK_EQUAL(scratch.sfd_template_key, 42u);
    BOOST_CHECK_EQUAL(scratch.work.capacity(), capacity);
}

BOOST_AUTO_TEST_CASE(one_grid_admission_controls_effective_path)
{
    demod::Qm35825Profile profile;
    profile.cir_soft_chip_mode = demod::CirSoftChipMode::Full;
    profile.rx_fused_payload_cmf = true;
    demod::TimingResult timing;
    timing.measured_period = demod::kQm35SamplesPerSymbol;
    auto plan = demod::core::plan_packet_execution(profile, timing);
    BOOST_CHECK(plan.integer_grid_ok);
    BOOST_CHECK(plan.effective_profile.cir_soft_chip_mode == demod::CirSoftChipMode::Full);
    timing.measured_period *= (1.0 - 27e-6);
    plan = demod::core::plan_packet_execution(profile, timing);
    BOOST_CHECK(!plan.integer_grid_ok);
    BOOST_CHECK(plan.effective_profile.cir_soft_chip_mode == demod::CirSoftChipMode::Bypass);
    BOOST_CHECK(profile.cir_soft_chip_mode == demod::CirSoftChipMode::Full);
    profile.rx_fused_payload_cmf = false;
    profile.rx_fused_phr_cmf = false;
    profile.rx_softchip_on_demand = false;
    plan = demod::core::plan_packet_execution(profile, timing);
    BOOST_CHECK(plan.integer_grid_ok);
    BOOST_CHECK(plan.effective_profile.cir_soft_chip_mode == demod::CirSoftChipMode::Full);
}

template<class Resampler> void independent_windows()
{
    const std::vector<float> taps(1039, 0.01f);
    Resampler shared(taps), reference(taps);
    std::vector<std::complex<float>> work, expected;
    for (size_t count : {193u, 91u}) {
        std::vector<std::complex<int16_t>> input(count);
        for (size_t i = 0; i < count; ++i)
            input[i] = {int16_t(i * 11 - 250), int16_t(700 - i * 7)};
        const auto made = core::resample_sc16_window(shared, input.data(), count,
                                                    1.f/32768, work);
        // Legacy one-window path, independent reset including full FIR tail.
        reference.reset();
        expected.resize(reference.expected_output_length(count));
        size_t produced = reference.process_sc16(input.data(), count, 1.f/32768,
                                                  expected.data(), expected.size()).produced;
        while (produced < expected.size()) {
            const auto n = reference.flush(expected.data() + produced, expected.size() - produced);
            BOOST_REQUIRE(n > 0);
            produced += n;
        }
        BOOST_REQUIRE_EQUAL(made, produced);
        BOOST_CHECK_EQUAL_COLLECTIONS(work.begin(), work.begin()+made,
                                      expected.begin(), expected.end());
    }
}
BOOST_AUTO_TEST_CASE(shared_resampling_preserves_reset_scaling_and_tail)
{
    independent_windows<core::RationalResampler65_48Core>();
    independent_windows<core::RationalResampler65_32Core>();
}

BOOST_AUTO_TEST_CASE(live_retry_policy_and_observation_provenance)
{
    demod::Qm35825Profile profile;
    demod::DemodResult decoded;
    decoded.timing.ok = decoded.sfd.ok = true;
    decoded.status = demod::DemodStatus::PayloadFailed;
    BOOST_CHECK(live::should_retry_full_cir(decoded, profile, true));
    BOOST_CHECK(!live::should_retry_full_cir(decoded, profile, false));
    profile.rx_stop_after_sfd = true;
    BOOST_CHECK(!live::should_retry_full_cir(decoded, profile, true));
    live::PacketRequest request;
    request.id = 7; request.slot_index = 19; request.generation = 3;
    request.window_start = 10000; request.seed = 10048; request.target_slot = true;
    decoded.timing.preamble_start_sample = 65;
    const auto obs = live::make_target_observation(request, decoded, 998.4e6, 737.28e6, 0);
    BOOST_CHECK_EQUAL(obs.observed_origin, 10048);
    BOOST_CHECK_EQUAL(obs.residual, 0);
    BOOST_CHECK_EQUAL(obs.schedule_generation, 3u);
    BOOST_CHECK_EQUAL(obs.slot_index, 19u);
    BOOST_CHECK_EQUAL(obs.packet_id, 7u);
    BOOST_CHECK(obs.from_target_window);
    BOOST_CHECK(!obs.fcs_pass);
    decoded.timing.ok = false;
    const auto failed = live::make_target_observation(request, decoded, 998.4e6, 737.28e6, 0);
    BOOST_CHECK(!failed.timing_ok);
    BOOST_CHECK_EQUAL(failed.observed_origin, -1);
}

BOOST_AUTO_TEST_CASE(finalization_rebases_only_valid_sample_outputs)
{
    demod::DemodResult result;
    result.timing.ok = true;
    result.timing.preamble_start_sample = 10;
    result.timing.preamble_start_uncropped = 10;
    result.timing.peak_samples = {100, 1116};
    result.sfd.ok = true;
    result.sfd.sfd_start_sample = 2000;
    result.sfd.sfd_end_sample = 10127;
    result.sfd.sfd_start_chip = 123;
    const auto initial_cir_sample = result.cir.first_path_sample;
    demod::core::finalize_demod_result(result, {}, 50000);
    BOOST_CHECK_EQUAL(result.timing.preamble_start_sample, 50010);
    BOOST_CHECK_EQUAL(result.timing.peak_samples[1], 51116);
    BOOST_CHECK_EQUAL(result.sfd.sfd_start_sample, 52000);
    BOOST_CHECK_EQUAL(result.sfd.sfd_start_chip, 123);
    BOOST_CHECK_EQUAL(result.cir.first_path_sample, initial_cir_sample);
    BOOST_CHECK_EQUAL(result.cfo.fit_first_peak_sample, -1);
}
