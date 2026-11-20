// SPDX-License-Identifier: GPL-3.0-or-later
// Deliberately uses the packet API, not the stage aggregate.
#include <boost/test/unit_test.hpp>
#include <gnuradio/uwb/uwb_hrp_codec.h>
#include <gnuradio/uwb/uwb_hrp_mod_core.h>
#include <gnuradio/uwb/uwb_demod_execution.h>
#include <gnuradio/uwb/uwb_demod_finalize.h>
#include <gnuradio/uwb/uwb_demod_pipeline.h>
#include <gnuradio/uwb/uwb_resample_window.h>
#include "../apps/uwb_live_packet_processor.h"
#include <condition_variable>
#include <map>
#include <mutex>
#include <string>

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
    profile.rx_fused_phr_cmf = true;
    profile.rx_softchip_on_demand = true;
    demod::TimingResult timing;
    timing.measured_period = demod::kQm35SamplesPerSymbol;
    auto plan = demod::core::plan_packet_execution(profile, timing);
    BOOST_CHECK(plan.integer_grid_ok);
    BOOST_CHECK(plan.effective_profile.cir_soft_chip_mode == demod::CirSoftChipMode::Full);
    BOOST_CHECK(plan.effective_profile.rx_fused_payload_cmf);
    timing.measured_period *= (1.0 - 27e-6);
    plan = demod::core::plan_packet_execution(profile, timing);
    BOOST_CHECK(!plan.integer_grid_ok);
    BOOST_CHECK(plan.effective_profile.cir_soft_chip_mode == demod::CirSoftChipMode::Full);
    BOOST_CHECK(!plan.effective_profile.rx_fused_payload_cmf);
    BOOST_CHECK(!plan.effective_profile.rx_fused_phr_cmf);
    BOOST_CHECK(!plan.effective_profile.rx_softchip_on_demand);
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

BOOST_AUTO_TEST_CASE(live_submission_accounting_single_slot_deterministic)
{
    std::vector<std::complex<int16_t>> ring(16);
    for (size_t i = 0; i < ring.size(); ++i)
        ring[i] = {static_cast<int16_t>(100 + i), static_cast<int16_t>(-100 - i)};

    std::mutex state_mutex;
    std::condition_variable state_cv;
    bool job1_entered = false;
    bool release_worker = false;
    std::map<uint64_t, std::string> terminals;
    uint64_t submit_attempts = 0;
    uint64_t admitted = 0;
    uint64_t dropped_queue = 0;
    uint64_t demod_completed = 0;
    std::vector<std::complex<int16_t>> copied_iq;

    live::PacketSlotPool pool(/*capacity=*/1, /*max_window=*/8);
    pool.start(/*num_workers=*/1, [&](size_t, live::PacketSlotPool::WorkerLoop& loop) {
        loop.run([&](live::PacketJob& job) {
            {
                std::unique_lock<std::mutex> lock(state_mutex);
                copied_iq.assign(job.iq.begin(), job.iq.begin() + static_cast<long>(job.count));
                job1_entered = true;
                state_cv.notify_all();
                state_cv.wait(lock, [&] { return release_worker; });
            }
            std::lock_guard<std::mutex> lock(state_mutex);
            ++demod_completed;
            terminals[job.id] = "COMPLETED";
        });
    });

    auto submit_one = [&](uint64_t id, int64_t window_start, int64_t seed) -> bool {
        live::PacketRequest req;
        req.id = id;
        req.seed = seed;
        req.window_start = window_start;
        req.predicted_rel_work = 16;
        req.guard = 4;
        req.generation = 1;
        req.feeds_lock = true;
        ++submit_attempts;
        const bool ok = pool.submit(req, /*count=*/8, ring);
        if (ok) {
            ++admitted;
        } else {
            std::lock_guard<std::mutex> lock(state_mutex);
            ++dropped_queue;
            terminals[req.id] = "DROPPED_QUEUE";
        }
        return ok;
    };

    BOOST_REQUIRE(submit_one(1, 12, 16));
    {
        std::unique_lock<std::mutex> lock(state_mutex);
        state_cv.wait(lock, [&] { return job1_entered; });
    }
    BOOST_CHECK(!submit_one(2, 2, 6));

    {
        std::lock_guard<std::mutex> lock(state_mutex);
        release_worker = true;
    }
    state_cv.notify_all();
    pool.stop();

    BOOST_CHECK_EQUAL(submit_attempts, 2u);
    BOOST_CHECK_EQUAL(admitted, 1u);
    BOOST_CHECK_EQUAL(dropped_queue, 1u);
    BOOST_CHECK_EQUAL(demod_completed, 1u);
    BOOST_CHECK_EQUAL(pool.backlog(), 0u);
    BOOST_REQUIRE_EQUAL(terminals.size(), 2u);
    BOOST_CHECK_EQUAL(terminals.at(1), "COMPLETED");
    BOOST_CHECK_EQUAL(terminals.at(2), "DROPPED_QUEUE");
    BOOST_REQUIRE_EQUAL(copied_iq.size(), 8u);
    for (size_t i = 0; i < 8; ++i)
        BOOST_CHECK(copied_iq[i] == ring[(12 + i) & 15]);
}

BOOST_AUTO_TEST_CASE(demod_window_range_geometry_and_derotated_span)
{
    using namespace demod::core;

    // Clamping at both window boundaries and empty intervals.
    BOOST_CHECK(clamp_sample_range(-50, 0, 1000).empty());
    BOOST_CHECK(clamp_sample_range(1200, 1400, 1000).empty());
    BOOST_CHECK(clamp_sample_range(0, 100, 0).empty());
    BOOST_CHECK((clamp_sample_range(-20, 1500, 1000) == SampleRange{0, 1000}));

    // Same hi is never sufficient if lo does not cover the required start, and
    // CFO / phase origin must also match.
    const DerotatedSpan span{SampleRange{500, 2000}, 1250.0, 0, true};
    BOOST_CHECK(span.covers_with_cfo(SampleRange{500, 2000}, 1250.0, 0));
    BOOST_CHECK(span.covers_with_cfo(SampleRange{600, 1800}, 1250.0, 0));
    BOOST_CHECK(!span.covers_with_cfo(SampleRange{499, 2000}, 1250.0, 0));
    BOOST_CHECK(!span.covers_with_cfo(SampleRange{500, 2000}, 1250.5, 0));
    BOOST_CHECK(!span.covers_with_cfo(SampleRange{500, 2000}, 1250.0, 10));
    BOOST_CHECK(!span.covers_with_cfo(SampleRange{}, 1250.0, 0));

    // SFD window range: narrow vs wide rescue, boundary clamping and out-of-window.
    demod::TimingResult timing;
    timing.measured_period = demod::kQm35SamplesPerSymbol;
    timing.preamble_start_sample = 40000;
    const size_t n = 608000;
    const size_t sfd_len = 8 * demod::kQm35SamplesPerSymbol;
    const SampleRange narrow_sfd = compute_sfd_window_range(
        n, timing, 256, sfd_len, /*include_wide_grid=*/false);
    const SampleRange wide_sfd = compute_sfd_window_range(
        n, timing, 256, sfd_len, /*include_wide_grid=*/true);
    BOOST_CHECK(!narrow_sfd.empty());
    BOOST_CHECK(wide_sfd.covers(narrow_sfd));
    BOOST_CHECK(wide_sfd.size() > narrow_sfd.size());
    BOOST_CHECK(compute_sfd_window_range(0, timing, 256, sfd_len, false).empty());
    BOOST_CHECK(compute_sfd_window_range(100000, timing, 256, sfd_len, false).empty());

    // Predicted single-pass range vs anchored prefix range under SFD origin shift.
    demod::Qm35825Profile profile;
    profile.code_index = 10;
    profile.preamble_repetitions = 256;
    profile.cir_repetitions = 246;
    profile.sfd_mode = "4z2";
    profile.rx_softchip_on_demand = true;
    profile.rx_fused_payload_cmf = true;
    profile.rx_fused_phr_cmf = true;
    profile.cfo_refit_mode = 0;
    BOOST_CHECK(compute_predicted_single_pass_range(
                    n, profile, timing, narrow_sfd).empty());

    profile.cfo_refit_mode = 1;
    const SampleRange predicted = compute_predicted_single_pass_range(
        n, profile, timing, narrow_sfd);
    BOOST_CHECK(!predicted.empty());

    demod::TimingResult anchored_timing = timing;
    anchored_timing.preamble_start_sample +=
        static_cast<int64_t>(demod::kQm35SamplesPerSymbol) + 17;
    const SampleRange final_sfd = compute_sfd_window_range(
        n, anchored_timing, 256, sfd_len, false);
    const SampleRange anchored_int = compute_anchored_prefix_range(
        n, profile, /*integer_grid_ok=*/true, anchored_timing, final_sfd);
    BOOST_CHECK(predicted.covers(anchored_int));
    BOOST_CHECK(anchored_int.hi < n);

    // Fractional grid widens the anchored range to the full window.
    const SampleRange anchored_frac = compute_anchored_prefix_range(
        n, profile, /*integer_grid_ok=*/false, anchored_timing, final_sfd);
    BOOST_CHECK((anchored_frac == SampleRange{0, n}));
    BOOST_CHECK(!predicted.covers(anchored_frac));

    // SFD-only mode bounds the anchored range strictly to final_sfd.
    profile.rx_stop_after_sfd = true;
    const SampleRange anchored_sfd_only = compute_anchored_prefix_range(
        n, profile, /*integer_grid_ok=*/true, anchored_timing, final_sfd);
    BOOST_CHECK(anchored_sfd_only == final_sfd);

    // Payload and on-demand extension rotation ranges clamp to n and skip covered prefixes.
    profile.rx_stop_after_sfd = false;
    demod::NsSfdResult ns_sfd;
    ns_sfd.sfd_end_chip = 264 * 508 - 1;
    demod::PhrResult phr;
    phr.psdu_length = 127;
    const SampleRange pay_range = compute_fused_payload_rotation_range(
        n, profile, anchored_timing, 38, ns_sfd, phr, anchored_int.hi);
    BOOST_CHECK(!pay_range.empty());
    BOOST_CHECK(pay_range.hi < n);
    const SampleRange pay_clamped = compute_fused_payload_rotation_range(
        450000, profile, anchored_timing, 38, ns_sfd, phr, anchored_int.hi);
    BOOST_CHECK_EQUAL(pay_clamped.hi, 450000u);
    BOOST_CHECK(compute_fused_payload_rotation_range(
                    200000, profile, anchored_timing, 38, ns_sfd, phr, 200000).empty());
    const SampleRange ext_range = compute_ondemand_extension_rotation_range(
        n, profile, anchored_timing, 38, 180000, anchored_int.hi);
    BOOST_CHECK(!ext_range.empty());
    BOOST_CHECK_EQUAL(ext_range.lo, anchored_int.hi);
    BOOST_CHECK(ext_range.hi <= n);
    BOOST_CHECK(compute_ondemand_extension_rotation_range(
                    n, profile, anchored_timing, 38, 180000, n).empty());
}

BOOST_AUTO_TEST_CASE(demod_frame_decode_paths_and_fallback_publication)
{
    std::vector<uint8_t> wire(30);
    for (size_t j = 0; j < wire.size(); ++j)
        wire[j] = static_cast<uint8_t>(19 * j + 10);
    mod::append_ieee_fcs(wire);

    mod::HrpModConfig cfg;
    cfg.code_index = 10;
    cfg.sync_repetitions = 64;
    cfg.sfd_mode = "4z2";
    cfg.peak_amplitude = 0.8f;
    const size_t count = mod::packet_samples_998p4(
        64, 8, wire.size(), false, &cfg.pulse);
    mod::HrpModScratch mod_scratch;
    mod_scratch.reserve(127, count);
    std::vector<std::complex<float>> wave(count);
    size_t written = 0;
    BOOST_REQUIRE(mod::modulate_one(wire.data(), wire.size(), cfg,
                                    mod_scratch, wave.data(), wave.size(), written));
    BOOST_REQUIRE_EQUAL(written, count);

    constexpr size_t kSeed = 2048;
    std::vector<std::complex<float>> sync(wave.begin(), wave.begin() + 1016);
    std::vector<std::complex<float>> iq(kSeed + count + 2048, std::complex<float>(0.0f, 0.0f));
    std::copy(wave.begin(), wave.end(), iq.begin() + kSeed);

    auto base_profile = demod::Qm35825Profile::Default();
    base_profile.code_index = 10;
    base_profile.timing_search_margin = 256;
    base_profile.timing_coarse_stride = 1;

    demod::core::DemodScratch scratch;
    scratch.reserve(iq.size());

    // 1. Fused fast path vs primary full-stream path.
    auto fused_profile = base_profile;
    fused_profile.rx_softchip_on_demand = true;
    fused_profile.rx_fused_payload_cmf = true;
    fused_profile.rx_fused_phr_cmf = true;
    const auto fused_res = demod::core::demodulate_one(
        iq.data(), iq.size(), fused_profile, 101, kSeed, 0, sync, scratch);
    BOOST_REQUIRE_EQUAL(static_cast<int>(fused_res.status),
                        static_cast<int>(demod::DemodStatus::Success));
    BOOST_CHECK(fused_res.payload.fcs_pass);
    BOOST_CHECK(fused_res.payload.bytes == wire);
    BOOST_CHECK_EQUAL(fused_res.stage_fused_attempts, 1u);
    BOOST_CHECK_EQUAL(fused_res.stage_phr_fused_attempts, 1u);
    BOOST_CHECK_EQUAL(fused_res.stage_softchip_fallbacks, 0u);

    auto full_profile = base_profile;
    full_profile.rx_softchip_on_demand = false;
    full_profile.rx_fused_payload_cmf = false;
    full_profile.rx_fused_phr_cmf = false;
    const auto full_res = demod::core::demodulate_one(
        iq.data(), iq.size(), full_profile, 102, kSeed, 0, sync, scratch);
    BOOST_REQUIRE_EQUAL(static_cast<int>(full_res.status),
                        static_cast<int>(demod::DemodStatus::Success));
    BOOST_CHECK(full_res.payload.fcs_pass);
    BOOST_CHECK(full_res.payload.bytes == wire);
    BOOST_CHECK_EQUAL(full_res.stage_fused_attempts, 0u);
    BOOST_CHECK_EQUAL(full_res.stage_phr_fused_attempts, 0u);
    BOOST_CHECK_EQUAL(full_res.stage_softchip_fallbacks, 0u);

    // 2. On-demand extension path (without fused CMF).
    auto ondemand_profile = base_profile;
    ondemand_profile.rx_softchip_on_demand = true;
    ondemand_profile.rx_fused_payload_cmf = false;
    ondemand_profile.rx_fused_phr_cmf = false;
    const auto ondemand_res = demod::core::demodulate_one(
        iq.data(), iq.size(), ondemand_profile, 103, kSeed, 0, sync, scratch);
    BOOST_REQUIRE_EQUAL(static_cast<int>(ondemand_res.status),
                        static_cast<int>(demod::DemodStatus::Success));
    BOOST_CHECK(ondemand_res.payload.fcs_pass);
    BOOST_CHECK(ondemand_res.payload.bytes == wire);
    BOOST_CHECK_EQUAL(ondemand_res.stage_fused_attempts, 0u);
    BOOST_CHECK_EQUAL(ondemand_res.stage_softchip_fallbacks, 0u);
    BOOST_CHECK_GT(ondemand_res.cir.on_demand_extended, 0u);

    // 3. Full-stream fallback publication when RAKE rejects on-demand prefix.
    auto fallback_profile = fused_profile;
    fallback_profile.cir_soft_chip_mode = demod::CirSoftChipMode::Rake;
    fallback_profile.cir_rake_top_k = 4;
    const auto fb_res = demod::core::demodulate_one(
        iq.data(), iq.size(), fallback_profile, 104, kSeed, 0, sync, scratch);
    BOOST_REQUIRE_EQUAL(static_cast<int>(fb_res.status),
                        static_cast<int>(demod::DemodStatus::Success));
    BOOST_CHECK(fb_res.payload.fcs_pass);
    BOOST_CHECK(fb_res.payload.bytes == wire);
    BOOST_CHECK_EQUAL(fb_res.stage_softchip_fallbacks, 1u);
    BOOST_CHECK_EQUAL(fb_res.cir.on_demand_fallbacks, 1u);

    // 4. Fractional-grid rejection forces primary full-stream decode.
    auto frac_profile = fused_profile;
    frac_profile.fused_gate_ppm = -1.0;
    const auto frac_res = demod::core::demodulate_one(
        iq.data(), iq.size(), frac_profile, 105, kSeed, 0, sync, scratch);
    BOOST_REQUIRE_EQUAL(static_cast<int>(frac_res.status),
                        static_cast<int>(demod::DemodStatus::Success));
    BOOST_CHECK(frac_res.payload.fcs_pass);
    BOOST_CHECK(frac_res.payload.bytes == wire);
    BOOST_CHECK_EQUAL(frac_res.stage_grid_rejects, 1u);
    BOOST_CHECK_EQUAL(frac_res.stage_fused_attempts, 0u);
    BOOST_CHECK_EQUAL(frac_res.stage_softchip_fallbacks, 0u);

    // 5. SFD-only early exit.
    auto sfd_only_profile = fused_profile;
    sfd_only_profile.rx_stop_after_sfd = true;
    const auto sfd_res = demod::core::demodulate_one(
        iq.data(), iq.size(), sfd_only_profile, 106, kSeed, 1000, sync, scratch);
    BOOST_REQUIRE_EQUAL(static_cast<int>(sfd_res.status),
                        static_cast<int>(demod::DemodStatus::Success));
    BOOST_CHECK(sfd_res.sfd.ok);
    BOOST_CHECK(!sfd_res.cir.ok);
    BOOST_CHECK_EQUAL(sfd_res.stage_cir_us, 0u);
    BOOST_CHECK_EQUAL(sfd_res.stage_total_us,
                      sfd_res.stage_timing_us + sfd_res.stage_cfo_us +
                          sfd_res.stage_sfd_us);
}
