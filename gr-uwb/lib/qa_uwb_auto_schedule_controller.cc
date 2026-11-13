/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Unit tests for pure-C++ AutoScheduleController, interference-aware
 * asymmetric window geometry, multi-rate native SYNC template synthesis +
 * UwbPreambleVerifierSc16, known-period SFO OLS/PLL lock, Holdover/Reacquire,
 * and gap discontinuity handling.
 */

#define BOOST_TEST_MODULE qa_uwb_auto_schedule_controller
#include <boost/test/unit_test.hpp>

#include <gnuradio/uwb/uwb_auto_schedule_controller.h>
#include <gnuradio/uwb/uwb_defaults.h>
#include <gnuradio/uwb/uwb_hrp_mod_core.h>
#include <gnuradio/uwb/uwb_preamble_verifier_sc16.h>
#include <gnuradio/uwb/uwb_rational_resampler_core.h>

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace {

using cf = std::complex<float>;
using sc16 = std::complex<int16_t>;

namespace mod = gr::uwb::mod;
namespace demod = gr::uwb::demod;
namespace core = gr::uwb::core;
namespace pipeline = gr::uwb::pipeline;

constexpr double kWorkRate = gr::uwb::defaults::kSampleRateHz;       // 998.4e6
constexpr double kNativeRate737 = gr::uwb::defaults::kNativeSampleRateHz; // 737.28e6
constexpr size_t kWorkSym = demod::kQm35SamplesPerSymbol;            // 1016

std::vector<float> make_downsample_48_65_taps(size_t half_wing_out_bins = 8)
{
    // Symmetric lowpass FIR on the 48 * 65 = 3120x grid.
    // T = 1 + 2 * 65 * half_wing_out_bins gives exact integer output delay
    // ((T - 1) / 2) / 65 = half_wing_out_bins samples in the 737.28 MS/s output.
    const size_t half = 65 * half_wing_out_bins;
    const size_t T = 2 * half + 1;
    std::vector<float> taps(T, 0.0f);
    constexpr double kPi = 3.14159265358979323846;
    const double cutoff = 48.0 / 65.0; // normalized in work-grid units
    double sum = 0.0;
    for (size_t i = 0; i < T; ++i) {
        const double x =
            (static_cast<double>(i) - static_cast<double>(half)) / 48.0;
        const double arg = cutoff * x;
        const double sinc_v =
            (std::abs(arg) < 1e-12) ? 1.0 : std::sin(kPi * arg) / (kPi * arg);
        const double u = static_cast<double>(i) / static_cast<double>(T - 1);
        const double win = 0.42 - 0.50 * std::cos(2.0 * kPi * u) +
                           0.08 * std::cos(4.0 * kPi * u);
        const double v = sinc_v * win;
        taps[i] = static_cast<float>(v);
        sum += v;
    }
    // RationalResamplerLmCore<48, 65> requires DC tap sum == 48 (kInterp).
    const double gain = 48.0 / sum;
    for (float& t : taps) {
        t = static_cast<float>(t * gain);
    }
    return taps;
}

std::vector<cf> generate_work_packet(size_t code_index,
                                     size_t sync_reps,
                                     const char* sfd_mode,
                                     bool with_payload)
{
    mod::HrpModConfig cfg;
    cfg.code_index = code_index;
    cfg.sync_repetitions = sync_reps;
    cfg.sfd_mode = sfd_mode;
    cfg.insert_sts = false;
    cfg.peak_amplitude = 0.8f;

    mod::HrpModScratch scratch;
    if (!with_payload) {
        std::vector<cf> wf(sync_reps * kWorkSym + 16 * kWorkSym + 512);
        size_t n = 0;
        BOOST_REQUIRE(mod::modulate_prefix_waveform(cfg, scratch, wf.data(),
                                                    wf.size(), n));
        wf.resize(n);
        return wf;
    }

    const auto psdu = mod::make_random_psdu(16, 0x12345678U, /*append_fcs=*/true);
    const size_t sfd_syms = pipeline::sfd_symbol_count_for_mode(sfd_mode);
    const size_t est_len =
        mod::packet_samples_998p4(sync_reps, sfd_syms, psdu.size(), false, &cfg.pulse) +
        512;
    std::vector<cf> wf(est_len);
    size_t n = 0;
    BOOST_REQUIRE(mod::modulate_one(psdu.data(), psdu.size(), cfg, scratch,
                                    wf.data(), wf.size(), n));
    wf.resize(n);
    return wf;
}

std::vector<sc16> float_to_sc16(const std::vector<cf>& in, float full_scale = 20000.0f)
{
    float peak = 0.0f;
    for (const auto& s : in) {
        peak = std::max(peak, std::max(std::abs(s.real()), std::abs(s.imag())));
    }
    const float scale = (peak > 0.0f) ? (full_scale / peak) : 0.0f;
    std::vector<sc16> out(in.size());
    for (size_t i = 0; i < in.size(); ++i) {
        const long re = std::lround(in[i].real() * scale);
        const long im = std::lround(in[i].imag() * scale);
        out[i] = sc16(
            static_cast<int16_t>(std::clamp<long>(re, -32767, 32767)),
            static_cast<int16_t>(std::clamp<long>(im, -32767, 32767)));
    }
    return out;
}

void stamp_packet(std::vector<sc16>& dst,
                  int64_t dst_start_abs,
                  int64_t pkt_start_abs,
                  const std::vector<sc16>& pkt)
{
    const int64_t dst_end_abs = dst_start_abs + static_cast<int64_t>(dst.size());
    const int64_t pkt_end_abs = pkt_start_abs + static_cast<int64_t>(pkt.size());
    const int64_t lo = std::max(dst_start_abs, pkt_start_abs);
    const int64_t hi = std::min(dst_end_abs, pkt_end_abs);
    for (int64_t pos = lo; pos < hi; ++pos) {
        const size_t di = static_cast<size_t>(pos - dst_start_abs);
        const size_t si = static_cast<size_t>(pos - pkt_start_abs);
        const int32_t re = static_cast<int32_t>(dst[di].real()) + pkt[si].real();
        const int32_t im = static_cast<int32_t>(dst[di].imag()) + pkt[si].imag();
        dst[di] = sc16(
            static_cast<int16_t>(std::clamp<int32_t>(re, -32767, 32767)),
            static_cast<int16_t>(std::clamp<int32_t>(im, -32767, 32767)));
    }
}

} // namespace

BOOST_AUTO_TEST_CASE(test_interference_aware_window_geometry)
{
    // Mode A: Sensing target (16 SYNC + 8 SFD "4z2"), 64-SYNC interferer.
    {
        pipeline::InterferenceGuardSpec spec;
        spec.native_rate = kWorkRate;
        spec.work_rate = kWorkRate;
        spec.target_is_comm = false;
        spec.target_sync_reps = 16;
        spec.target_sfd_mode = "4z2";
        spec.int_sync_reps = 64;
        spec.int_sfd_mode = "4z2";
        spec.int_pre_sfd_sync_reps = 16;
        spec.early_margin_work = 512;
        spec.late_margin_work = 512;
        spec.resampler_native_halo = 256;

        const auto g998 = pipeline::compute_interference_aware_window(spec);
        const size_t int_sfd_syms = 8;
        const size_t tgt_sfd_syms = 8;

        BOOST_CHECK_GE(g998.pre_guard_work,
                       (spec.int_pre_sfd_sync_reps + int_sfd_syms) * kWorkSym);
        BOOST_CHECK_EQUAL(
            g998.pre_guard_work,
            (spec.int_pre_sfd_sync_reps + int_sfd_syms) * kWorkSym +
                spec.early_margin_work);

        BOOST_CHECK_GE(g998.post_guard_work,
                       (spec.int_sync_reps + int_sfd_syms) * kWorkSym);
        BOOST_CHECK_EQUAL(
            g998.post_guard_work,
            (spec.int_sync_reps + int_sfd_syms) * kWorkSym +
                spec.late_margin_work);

        BOOST_CHECK_EQUAL(g998.capture_work,
                          (spec.target_sync_reps + tgt_sfd_syms) * kWorkSym);

        // At native_rate == work_rate, no resampler halo is added.
        BOOST_CHECK_EQUAL(g998.pre_guard_native, g998.pre_guard_work);
        BOOST_CHECK_EQUAL(g998.capture_native, g998.capture_work);
        BOOST_CHECK_EQUAL(g998.post_guard_native, g998.post_guard_work);

        // At native_rate = 737.28e6, native lengths scale by 48/65 + halo.
        spec.native_rate = kNativeRate737;
        const auto g737 = pipeline::compute_interference_aware_window(spec);
        BOOST_CHECK_EQUAL(g737.pre_guard_work, g998.pre_guard_work);
        BOOST_CHECK_EQUAL(g737.capture_work, g998.capture_work);
        BOOST_CHECK_EQUAL(g737.post_guard_work, g998.post_guard_work);

        const double ratio = kNativeRate737 / kWorkRate;
        BOOST_CHECK_EQUAL(
            g737.pre_guard_native,
            static_cast<size_t>(std::ceil(g737.pre_guard_work * ratio)) +
                spec.resampler_native_halo);
        BOOST_CHECK_EQUAL(
            g737.capture_native,
            static_cast<size_t>(std::ceil(g737.capture_work * ratio)) +
                spec.resampler_native_halo);
        BOOST_CHECK_EQUAL(
            g737.post_guard_native,
            static_cast<size_t>(std::ceil(g737.post_guard_work * ratio)) +
                spec.resampler_native_halo);
    }

    // Mode B: Comm target (64 SYNC + "4z2", 32-byte PSDU), 16-SYNC "4z1" interferer.
    {
        pipeline::InterferenceGuardSpec spec;
        spec.native_rate = kNativeRate737;
        spec.work_rate = kWorkRate;
        spec.target_is_comm = true;
        spec.target_sync_reps = 64;
        spec.target_sfd_mode = "4z2";
        spec.target_max_psdu_bytes = 32;
        spec.target_pulse.shape = mod::PulseShape::Gaussian;
        spec.int_sync_reps = 16;
        spec.int_sfd_mode = "4z1"; // 4 symbols
        spec.int_pre_sfd_sync_reps = 12;
        spec.early_margin_work = 256;
        spec.late_margin_work = 384;
        spec.resampler_native_halo = 256;

        const auto g = pipeline::compute_interference_aware_window(spec);
        BOOST_CHECK_EQUAL(pipeline::sfd_symbol_count_for_mode("4z1"), 4u);
        BOOST_CHECK_EQUAL(pipeline::sfd_symbol_count_for_mode("4z3"), 16u);
        BOOST_CHECK_EQUAL(pipeline::sfd_symbol_count_for_mode(nullptr), 8u);

        BOOST_CHECK_GE(g.pre_guard_work, (12u + 4u) * kWorkSym);
        BOOST_CHECK_GE(g.post_guard_work, (16u + 4u) * kWorkSym);

        const size_t expected_comm_cap = mod::packet_samples_998p4(
            64, 8, 32, /*insert_sts=*/false, &spec.target_pulse);
        BOOST_CHECK_EQUAL(g.capture_work, expected_comm_cap);
        BOOST_CHECK_GT(g.capture_work, (64u + 8u) * kWorkSym);

        const double ratio = kNativeRate737 / kWorkRate;
        BOOST_CHECK_EQUAL(
            g.capture_native,
            static_cast<size_t>(std::ceil(expected_comm_cap * ratio)) + 256u);
    }
}

BOOST_AUTO_TEST_CASE(test_native_sync_template_and_verifier_multi_rate)
{
    const auto taps48_65 = make_downsample_48_65_taps(/*half_wing_out_bins=*/8);
    constexpr size_t kResampOutDelay = 8;

    const size_t codes[] = { 9, 10 };
    const size_t sync_reps_list[] = { 16, 64 };
    const double rates[] = { kWorkRate, kNativeRate737 };

    for (size_t code_index : codes) {
        for (size_t sync_reps : sync_reps_list) {
            const auto work_wf =
                generate_work_packet(code_index, sync_reps, "4z2",
                                     /*with_payload=*/true);

            // Use an exact 65:48 ratio lead length so native start is exact integer.
            constexpr size_t kLeadWork = 65 * 64;   // 4160 @ 998.4M
            constexpr size_t kLeadNative = 48 * 64; // 3072 @ 737.28M
            constexpr size_t kTailWork = 65 * 64;

            std::vector<cf> padded_work(kLeadWork + work_wf.size() + kTailWork,
                                        cf(0.0f, 0.0f));
            std::copy(work_wf.begin(), work_wf.end(),
                      padded_work.begin() + static_cast<std::ptrdiff_t>(kLeadWork));

            for (double native_rate : rates) {
                std::vector<sc16> rx_sc16;
                size_t expected_start = 0;

                if (std::abs(native_rate - kWorkRate) <= 1.0) {
                    rx_sc16 = float_to_sc16(padded_work);
                    expected_start = kLeadWork;
                } else {
                    core::RationalResampler48_65Core resamp(
                        taps48_65.data(), taps48_65.size());
                    const size_t out_cap =
                        resamp.expected_output_length(padded_work.size()) + 128;
                    std::vector<cf> downsampled(out_cap);
                    const auto pr = resamp.process(
                        padded_work.data(), padded_work.size(),
                        downsampled.data(), downsampled.size());
                    const size_t flushed = resamp.flush(
                        downsampled.data() + pr.produced,
                        downsampled.size() - pr.produced);
                    downsampled.resize(pr.produced + flushed);
                    rx_sc16 = float_to_sc16(downsampled);
                    expected_start = kLeadNative + kResampOutDelay;
                }

                const mod::PulseSpec pulse{};
                const auto tmpl = pipeline::synthesize_native_sync_template(
                    code_index, pulse, "4z2", native_rate, kWorkRate);
                const size_t expected_tmpl_len = static_cast<size_t>(
                    std::llround(static_cast<double>(kWorkSym) * native_rate /
                                 kWorkRate));
                BOOST_REQUIRE_EQUAL(tmpl.size(), expected_tmpl_len);

                // Verify L2 normalization of synthesized template.
                double norm2 = 0.0;
                for (const auto& v : tmpl) {
                    norm2 += std::norm(v);
                }
                BOOST_CHECK_CLOSE(norm2, 1.0, 1e-3);

                core::UwbPreambleVerifierSc16 verifier;
                core::UwbPreambleVerifierSc16::Config vcfg;
                vcfg.coarse_stride = 4;
                vcfg.fine_threshold = 0.5f;
                vcfg.backtrack_threshold = 0.2f;
                vcfg.max_backtrack = 16;
                vcfg.sync_repetitions = sync_reps;
                vcfg.sfd_symbols = 8;
                verifier.configure(tmpl, vcfg);
                verifier.reserve_coarse(rx_sc16.size());

                // Emulate energy-gate candidate trigger 2 symbols inside SYNC.
                const size_t candidate_off =
                    expected_start + 2 * expected_tmpl_len;
                const auto vr = verifier.verify(
                    rx_sc16.data(), rx_sc16.size(), candidate_off);

                BOOST_CHECK(vr.confirmed);
                BOOST_CHECK_GE(vr.detection_metric, 0.5f);
                const int64_t err =
                    static_cast<int64_t>(vr.start_offset) -
                    static_cast<int64_t>(expected_start);
                BOOST_CHECK_LE(std::abs(err), 4);
            }
        }
    }
}

BOOST_AUTO_TEST_CASE(test_known_period_sfo_lock_and_holdover_reacquire)
{
    constexpr double kFs = kWorkRate; // 998.4 MS/s
    constexpr double kNominalPeriodS = 0.001; // 1 ms -> 998,400 samples
    constexpr double kNominalPeriodSamples = kNominalPeriodS * kFs;
    constexpr double kSfoFactor = 1.0 + 200e-6; // +200 ppm
    constexpr double kTruePeriodSamples = kNominalPeriodSamples * kSfoFactor;
    constexpr int64_t kTrueT0 = 15377; // non-zero initial offset

    const auto wf = generate_work_packet(9, 64, "4z2", /*with_payload=*/true);
    const auto pkt_sc16 = float_to_sc16(wf, 20000.0f);

    pipeline::AutoScheduleConfig acfg;
    acfg.period_s = kNominalPeriodS;
    acfg.max_period_rel_err = 0.02;
    acfg.energy_threshold = 0.004f;
    acfg.detector_gate_decimation = 100;
    acfg.detector_gate_window = 32;
    acfg.detector_holdoff_decimated = 8;
    acfg.verifier_coarse_stride = 1;
    acfg.provisional_guard_us = 25.0;
    acfg.lock_observations = 10;
    acfg.holdover_miss_count = 3;
    acfg.reacquire_miss_count = 8;
    acfg.neighborhood_slots = 4;
    acfg.require_identity_match = true;
    acfg.require_fcs_pass = true;

    pipeline::AutoScheduleController::TargetProfile target;
    target.code_index = 9;
    target.sync_repetitions = 64;
    target.sfd_symbols = 8;
    target.sfd_mode = "4z2";
    target.is_comm = true;

    const size_t base_pre = 2048;
    const size_t base_cap = pkt_sc16.size();
    const size_t base_post = 2048;

    pipeline::AutoScheduleController ctrl;
    ctrl.configure(acfg, target, kFs, kFs, base_pre, base_cap, base_post);

    BOOST_CHECK(ctrl.state() == core::Qm35LockState::UnlockedAcquire);
    BOOST_CHECK(!ctrl.scheduled_path_active());
    BOOST_CHECK_EQUAL(ctrl.acq_windows(), 0u);
    BOOST_CHECK_EQUAL(ctrl.scheduled_windows(), 0u);

    auto packet_start_for_idx = [&](uint64_t idx) -> int64_t {
        return static_cast<int64_t>(
            std::llround(static_cast<double>(kTrueT0) +
                         static_cast<double>(idx) * kTruePeriodSamples));
    };

    // Helper to synthesize a slice [start_abs, start_abs + len) containing any
    // packets in the train that overlap it.
    auto make_chunk = [&](int64_t start_abs, size_t len) {
        std::vector<sc16> buf(len, sc16(0, 0));
        const int64_t end_abs = start_abs + static_cast<int64_t>(len);
        const double approx_lo =
            (static_cast<double>(start_abs - static_cast<int64_t>(pkt_sc16.size()) - kTrueT0)) /
            kTruePeriodSamples;
        const int64_t k_lo = std::max<int64_t>(0, static_cast<int64_t>(std::floor(approx_lo)));
        const int64_t k_hi = std::max<int64_t>(
            0, static_cast<int64_t>(std::ceil(
                   static_cast<double>(end_abs - kTrueT0) / kTruePeriodSamples)));
        for (int64_t k = k_lo; k <= k_hi; ++k) {
            stamp_packet(buf, start_abs,
                         packet_start_for_idx(static_cast<uint64_t>(k)), pkt_sc16);
        }
        return buf;
    };

    // Step 1: Feed packet 0 via continuous acquisition chunks, then reject it
    // due to wrong code_index / FCS fail before locking.
    constexpr size_t kChunkLen = 16384;
    int64_t cursor = 0;
    pipeline::AutoScheduleController::EmittedAcqSlot acq0;
    bool got_acq0 = false;
    while (cursor < 300000 && !got_acq0) {
        auto chunk = make_chunk(cursor, kChunkLen);
        ctrl.feed_acquisition(chunk.data(), chunk.size(), cursor);
        cursor += static_cast<int64_t>(kChunkLen);
        got_acq0 = ctrl.poll_verified_candidate(acq0);
    }
    BOOST_REQUIRE(got_acq0);
    BOOST_CHECK(ctrl.state() == core::Qm35LockState::CandidateVerify);
    BOOST_CHECK(ctrl.waiting_candidate_verify());
    BOOST_CHECK_LE(std::abs(acq0.predicted_start_abs - packet_start_for_idx(0)), 4);

    // Reject wrong identity (code_index = 10) -> must revert to UnlockedAcquire.
    {
        pipeline::LockFeedback bad_id;
        bad_id.window_id = 0;
        bad_id.acquisition_epoch = acq0.acquisition_epoch;
        bad_id.schedule_generation = acq0.schedule_generation;
        bad_id.window_kind = pipeline::WindowKind::Acquisition;
        bad_id.slot_native_start_abs = acq0.win_start_abs;
        bad_id.detected_start_work_rel =
            acq0.predicted_start_abs - acq0.win_start_abs;
        bad_id.work_rate = kFs;
        bad_id.native_rate = kFs;
        bad_id.timing_ok = true;
        bad_id.identity_ok = true;
        bad_id.fcs_pass = true;
        bad_id.code_index = 10;
        bad_id.preamble_repetitions = 64;
        bad_id.sfd_mode = "4z2";
        ctrl.ingest_feedback({ bad_id }, cursor);
        BOOST_CHECK(ctrl.state() == core::Qm35LockState::UnlockedAcquire);
        BOOST_CHECK(!ctrl.scheduled_path_active());
    }

    // Step 2: Advance to packet 1, emit candidate, reject FCS fail when
    // require_fcs_pass == true, then advance to packet 2 and confirm identity!
    auto acquire_next_packet = [&](pipeline::AutoScheduleController::EmittedAcqSlot& out) {
        const int64_t limit = cursor + static_cast<int64_t>(2 * kNominalPeriodSamples);
        while (cursor < limit) {
            auto chunk = make_chunk(cursor, kChunkLen);
            ctrl.feed_acquisition(chunk.data(), chunk.size(), cursor);
            cursor += static_cast<int64_t>(kChunkLen);
            if (ctrl.poll_verified_candidate(out)) {
                return true;
            }
        }
        return false;
    };

    pipeline::AutoScheduleController::EmittedAcqSlot acq1;
    BOOST_REQUIRE(acquire_next_packet(acq1));
    BOOST_CHECK_LE(std::abs(acq1.predicted_start_abs - packet_start_for_idx(1)), 4);

    // Reject FCS failure -> stays UnlockedAcquire, never locks.
    {
        pipeline::LockFeedback bad_fcs;
        bad_fcs.window_id = 1;
        bad_fcs.acquisition_epoch = acq1.acquisition_epoch;
        bad_fcs.schedule_generation = acq1.schedule_generation;
        bad_fcs.window_kind = pipeline::WindowKind::Acquisition;
        bad_fcs.slot_native_start_abs = acq1.win_start_abs;
        bad_fcs.detected_start_work_rel =
            acq1.predicted_start_abs - acq1.win_start_abs;
        bad_fcs.work_rate = kFs;
        bad_fcs.native_rate = kFs;
        bad_fcs.timing_ok = true;
        bad_fcs.identity_ok = true;
        bad_fcs.fcs_pass = false;
        bad_fcs.code_index = 9;
        bad_fcs.preamble_repetitions = 64;
        bad_fcs.sfd_mode = "4z2";
        ctrl.ingest_feedback({ bad_fcs }, cursor);
        BOOST_CHECK(ctrl.state() == core::Qm35LockState::UnlockedAcquire);
        BOOST_CHECK(!ctrl.scheduled_path_active());
    }

    // Packet 2 confirms identity -> ProvisionalTrack.
    pipeline::AutoScheduleController::EmittedAcqSlot acq2;
    BOOST_REQUIRE(acquire_next_packet(acq2));
    BOOST_CHECK_LE(std::abs(acq2.predicted_start_abs - packet_start_for_idx(2)), 4);
    {
        pipeline::LockFeedback good_acq;
        good_acq.window_id = 2;
        good_acq.acquisition_epoch = acq2.acquisition_epoch;
        good_acq.schedule_generation = acq2.schedule_generation;
        good_acq.window_kind = pipeline::WindowKind::Acquisition;
        good_acq.slot_native_start_abs = acq2.win_start_abs;
        good_acq.detected_start_work_rel =
            packet_start_for_idx(2) - acq2.win_start_abs;
        good_acq.work_rate = kFs;
        good_acq.native_rate = kFs;
        good_acq.timing_ok = true;
        good_acq.identity_ok = true;
        good_acq.fcs_pass = true;
        good_acq.code_index = 9;
        good_acq.preamble_repetitions = 64;
        good_acq.sfd_mode = "4z2";
        ctrl.ingest_feedback({ good_acq }, cursor);
    }

    BOOST_CHECK(ctrl.state() == core::Qm35LockState::ProvisionalTrack);
    BOOST_CHECK(ctrl.scheduled_path_active());

    // Step 3: Drive scheduled slots across packets 3..15 so OLS converges
    // (+200 ppm) and enters Locked.
    uint64_t next_win_id = 3;
    uint64_t true_pkt_idx = 3;
    pipeline::AutoScheduleController::ScheduledSlotPlan last_plan;
    for (int step = 0; step < 12; ++step) {
        pipeline::AutoScheduleController::ScheduledSlotPlan plan;
        BOOST_REQUIRE(ctrl.next_scheduled_slot(cursor, plan));
        last_plan = plan;
        const int64_t true_start = packet_start_for_idx(true_pkt_idx);
        BOOST_CHECK_GE(true_start, plan.win_start_abs);
        BOOST_CHECK_LT(true_start + static_cast<int64_t>(base_cap), plan.win_end_abs);

        cursor = plan.win_end_abs;

        pipeline::LockFeedback fb;
        fb.window_id = next_win_id++;
        fb.acquisition_epoch = plan.acquisition_epoch;
        fb.schedule_generation = plan.schedule_generation;
        fb.schedule_index = plan.schedule_index;
        fb.window_kind = pipeline::WindowKind::Scheduled;
        fb.slot_native_start_abs = plan.win_start_abs;
        fb.detected_start_work_rel = true_start - plan.win_start_abs;
        fb.work_rate = kFs;
        fb.native_rate = kFs;
        fb.timing_ok = true;
        fb.identity_ok = true;
        fb.fcs_pass = true;
        fb.code_index = 9;
        fb.preamble_repetitions = 64;
        fb.sfd_mode = "4z2";
        ctrl.ingest_feedback({ fb }, cursor);

        ++true_pkt_idx;
    }

    BOOST_CHECK(ctrl.state() == core::Qm35LockState::Locked);
    BOOST_CHECK_NE(ctrl.locked_at_window_id(),
                   std::numeric_limits<uint64_t>::max());
    BOOST_CHECK_CLOSE(ctrl.period_samples(), kTruePeriodSamples, 1e-3); // <10 ppm

    // Stale-generation feedback must be counted and ignored.
    {
        const uint64_t prev_stale = ctrl.stale_feedback();
        const double prev_period = ctrl.period_samples();
        pipeline::LockFeedback stale;
        stale.window_id = next_win_id++;
        stale.acquisition_epoch = last_plan.acquisition_epoch;
        stale.schedule_generation = last_plan.schedule_generation - 1;
        stale.schedule_index = last_plan.schedule_index;
        stale.window_kind = pipeline::WindowKind::Scheduled;
        stale.slot_native_start_abs = last_plan.win_start_abs;
        stale.detected_start_work_rel = 1234;
        stale.work_rate = kFs;
        stale.native_rate = kFs;
        stale.timing_ok = true;
        stale.identity_ok = true;
        stale.fcs_pass = true;
        ctrl.ingest_feedback({ stale }, cursor);
        BOOST_CHECK_EQUAL(ctrl.stale_feedback(), prev_stale + 1);
        BOOST_CHECK_EQUAL(ctrl.period_samples(), prev_period);
        BOOST_CHECK(ctrl.state() == core::Qm35LockState::Locked);
    }

    // Step 4: Inject holdover_miss_count (3) misses -> Holdover.
    for (size_t m = 0; m < acfg.holdover_miss_count; ++m) {
        pipeline::AutoScheduleController::ScheduledSlotPlan plan;
        BOOST_REQUIRE(ctrl.next_scheduled_slot(cursor, plan));
        cursor = plan.win_end_abs;
        pipeline::LockFeedback miss;
        miss.window_id = next_win_id++;
        miss.acquisition_epoch = plan.acquisition_epoch;
        miss.schedule_generation = plan.schedule_generation;
        miss.schedule_index = plan.schedule_index;
        miss.window_kind = pipeline::WindowKind::Scheduled;
        miss.slot_native_start_abs = plan.win_start_abs;
        miss.detected_start_work_rel = -1;
        miss.timing_ok = false;
        ctrl.ingest_feedback({ miss }, cursor);
        if (m + 1 < acfg.holdover_miss_count) {
            BOOST_CHECK(ctrl.state() == core::Qm35LockState::Locked);
        }
    }
    BOOST_CHECK(ctrl.state() == core::Qm35LockState::Holdover);

    // Holdover expands guards (is_provisional == true).
    {
        pipeline::AutoScheduleController::ScheduledSlotPlan hold_plan;
        BOOST_REQUIRE(ctrl.next_scheduled_slot(cursor, hold_plan));
        BOOST_CHECK(hold_plan.is_provisional);
        BOOST_CHECK_GT(hold_plan.pre_guard, base_pre);
    }

    // Inject remaining misses up to reacquire_miss_count (8) -> Reacquire.
    for (size_t m = acfg.holdover_miss_count; m < acfg.reacquire_miss_count; ++m) {
        pipeline::AutoScheduleController::ScheduledSlotPlan plan;
        BOOST_REQUIRE(ctrl.next_scheduled_slot(cursor, plan));
        cursor = plan.win_end_abs;
        pipeline::LockFeedback miss;
        miss.window_id = next_win_id++;
        miss.acquisition_epoch = plan.acquisition_epoch;
        miss.schedule_generation = plan.schedule_generation;
        miss.schedule_index = plan.schedule_index;
        miss.window_kind = pipeline::WindowKind::Scheduled;
        miss.slot_native_start_abs = plan.win_start_abs;
        miss.detected_start_work_rel = -1;
        miss.timing_ok = false;
        ctrl.ingest_feedback({ miss }, cursor);
    }
    BOOST_CHECK(ctrl.state() == core::Qm35LockState::Reacquire);

    // Exhaust neighborhood_slots (4) -> global energy Reacquire.
    for (size_t m = 0; m < acfg.neighborhood_slots; ++m) {
        pipeline::AutoScheduleController::ScheduledSlotPlan plan;
        BOOST_REQUIRE(ctrl.next_scheduled_slot(cursor, plan));
        cursor = plan.win_end_abs;
        pipeline::LockFeedback miss;
        miss.window_id = next_win_id++;
        miss.acquisition_epoch = plan.acquisition_epoch;
        miss.schedule_generation = plan.schedule_generation;
        miss.schedule_index = plan.schedule_index;
        miss.window_kind = pipeline::WindowKind::Scheduled;
        miss.slot_native_start_abs = plan.win_start_abs;
        miss.detected_start_work_rel = -1;
        miss.timing_ok = false;
        ctrl.ingest_feedback({ miss }, cursor);
    }
    BOOST_CHECK(ctrl.state() == core::Qm35LockState::Reacquire);
    BOOST_CHECK(!ctrl.scheduled_path_active());

    // Step 5: Stream discontinuity gap_samples = 5000 -> increments
    // discontinuities(), advances io_read_end_abs, drops stale pre-gap
    // feedback, and re-acquires cleanly on the next packet.
    const uint64_t pre_gap_disc = ctrl.discontinuities();
    const uint64_t pre_gap_gen = ctrl.generation();
    const uint64_t pre_gap_epoch = ctrl.epoch();
    int64_t io_read_end_abs = cursor;

    const bool jumped = ctrl.handle_stream_meta(
        cursor, kChunkLen, /*has_time_spec=*/false, 0.0,
        /*is_gap=*/true, /*gap_samples=*/5000, io_read_end_abs);
    BOOST_REQUIRE(jumped);
    BOOST_CHECK_EQUAL(ctrl.discontinuities(), pre_gap_disc + 1);
    BOOST_CHECK_EQUAL(io_read_end_abs, cursor + 5000);
    BOOST_CHECK_EQUAL(ctrl.scan_cursor_abs(), io_read_end_abs);
    BOOST_CHECK_GT(ctrl.generation(), pre_gap_gen);
    BOOST_CHECK_GT(ctrl.epoch(), pre_gap_epoch);
    BOOST_CHECK(ctrl.state() == core::Qm35LockState::Reacquire);
    BOOST_CHECK(!ctrl.scheduled_path_active());

    // Pre-gap feedback must be ignored as stale and must not re-confirm t0.
    {
        const uint64_t stale_before = ctrl.stale_feedback();
        pipeline::LockFeedback pre_gap_fb;
        pre_gap_fb.window_id = next_win_id++;
        pre_gap_fb.acquisition_epoch = pre_gap_epoch;
        pre_gap_fb.schedule_generation = pre_gap_gen;
        pre_gap_fb.window_kind = pipeline::WindowKind::Acquisition;
        pre_gap_fb.slot_native_start_abs = cursor - 4000;
        pre_gap_fb.detected_start_work_rel = 1000;
        pre_gap_fb.work_rate = kFs;
        pre_gap_fb.native_rate = kFs;
        pre_gap_fb.timing_ok = true;
        pre_gap_fb.identity_ok = true;
        pre_gap_fb.fcs_pass = true;
        pre_gap_fb.code_index = 9;
        pre_gap_fb.preamble_repetitions = 64;
        pre_gap_fb.sfd_mode = "4z2";
        ctrl.ingest_feedback({ pre_gap_fb }, io_read_end_abs);
        BOOST_CHECK_EQUAL(ctrl.stale_feedback(), stale_before + 1);
        BOOST_CHECK(!ctrl.scheduled_path_active());
    }

    // Re-acquire on a fresh post-gap packet train at new origin.
    cursor = io_read_end_abs;
    const int64_t post_gap_t0 = cursor + 24000;
    auto post_gap_pkt_start = [&](uint64_t k) -> int64_t {
        return static_cast<int64_t>(
            std::llround(static_cast<double>(post_gap_t0) +
                         static_cast<double>(k) * kTruePeriodSamples));
    };

    pipeline::AutoScheduleController::EmittedAcqSlot post_acq;
    bool got_post_acq = false;
    const int64_t post_limit = cursor + 400000;
    while (cursor < post_limit && !got_post_acq) {
        std::vector<sc16> chunk(kChunkLen, sc16(0, 0));
        stamp_packet(chunk, cursor, post_gap_pkt_start(0), pkt_sc16);
        ctrl.feed_acquisition(chunk.data(), chunk.size(), cursor);
        cursor += static_cast<int64_t>(kChunkLen);
        got_post_acq = ctrl.poll_verified_candidate(post_acq);
    }
    BOOST_REQUIRE(got_post_acq);
    BOOST_CHECK_EQUAL(post_acq.acquisition_epoch, ctrl.epoch());
    BOOST_CHECK_EQUAL(post_acq.schedule_generation, ctrl.generation());
    BOOST_CHECK_LE(std::abs(post_acq.predicted_start_abs - post_gap_pkt_start(0)), 4);

    pipeline::LockFeedback post_confirm;
    post_confirm.window_id = next_win_id++;
    post_confirm.acquisition_epoch = post_acq.acquisition_epoch;
    post_confirm.schedule_generation = post_acq.schedule_generation;
    post_confirm.window_kind = pipeline::WindowKind::Acquisition;
    post_confirm.slot_native_start_abs = post_acq.win_start_abs;
    post_confirm.detected_start_work_rel =
        post_gap_pkt_start(0) - post_acq.win_start_abs;
    post_confirm.work_rate = kFs;
    post_confirm.native_rate = kFs;
    post_confirm.timing_ok = true;
    post_confirm.identity_ok = true;
    post_confirm.fcs_pass = true;
    post_confirm.code_index = 9;
    post_confirm.preamble_repetitions = 64;
    post_confirm.sfd_mode = "4z2";
    ctrl.ingest_feedback({ post_confirm }, cursor);

    BOOST_CHECK(ctrl.state() == core::Qm35LockState::ProvisionalTrack);
    BOOST_CHECK(ctrl.scheduled_path_active());
    BOOST_CHECK_CLOSE(ctrl.t0_exact(), static_cast<double>(post_gap_pkt_start(0)), 1e-6);

    for (uint64_t k = 1; k < acfg.lock_observations; ++k) {
        pipeline::AutoScheduleController::ScheduledSlotPlan plan;
        BOOST_REQUIRE(ctrl.next_scheduled_slot(cursor, plan));
        cursor = plan.win_end_abs;
        pipeline::LockFeedback fb;
        fb.window_id = next_win_id++;
        fb.acquisition_epoch = plan.acquisition_epoch;
        fb.schedule_generation = plan.schedule_generation;
        fb.schedule_index = plan.schedule_index;
        fb.window_kind = pipeline::WindowKind::Scheduled;
        fb.slot_native_start_abs = plan.win_start_abs;
        fb.detected_start_work_rel = post_gap_pkt_start(k) - plan.win_start_abs;
        fb.work_rate = kFs;
        fb.native_rate = kFs;
        fb.timing_ok = true;
        fb.identity_ok = true;
        fb.fcs_pass = true;
        ctrl.ingest_feedback({ fb }, cursor);
    }
    BOOST_CHECK(ctrl.state() == core::Qm35LockState::Locked);
}

BOOST_AUTO_TEST_CASE(test_clamping_time_spec_jump_and_feedback_queue)
{
    // Verify win_start_abs clamping when packet arrives closer to start_abs than
    // acq_pre_, ensuring out.samples and out.win_start_abs stay aligned.
    pipeline::AutoScheduleConfig acfg;
    acfg.period_s = 0.0; // must fall back to defaults::kQm35PacketIntervalS
    acfg.energy_threshold = 0.004f;
    acfg.acq_pre_guard_samples = 12000;
    acfg.acq_post_guard_samples = 2000;
    acfg.detector_gate_decimation = 100;
    acfg.detector_gate_window = 32;
    acfg.detector_holdoff_decimated = 8;

    pipeline::AutoScheduleController::TargetProfile target;
    target.code_index = 9;
    target.sync_repetitions = 16;
    target.sfd_symbols = 0; // must resolve from sfd_mode "4z2" -> 8
    target.sfd_mode = "4z2";
    target.is_comm = false;

    const auto wf = generate_work_packet(9, 16, "4z2", /*with_payload=*/false);
    const auto pkt_sc16 = float_to_sc16(wf, 20000.0f);

    pipeline::AutoScheduleController ctrl;
    ctrl.configure(acfg, target, kWorkRate, kWorkRate, 1024, pkt_sc16.size(), 1024);
    BOOST_CHECK_CLOSE(ctrl.period_s(), gr::uwb::defaults::kQm35PacketIntervalS, 1e-9);

    // Place packet at sample 2000 (< acq_pre_guard_samples = 12000) and feed
    // enough trailing zeros for capture completion.
    constexpr int64_t kEarlyStart = 2000;
    std::vector<sc16> stream(kEarlyStart + pkt_sc16.size() + 65536, sc16(0, 0));
    stamp_packet(stream, 0, kEarlyStart, pkt_sc16);
    ctrl.feed_acquisition(stream.data(), stream.size(), 0);
    ctrl.flush_acquisition();

    pipeline::AutoScheduleController::EmittedAcqSlot slot;
    BOOST_REQUIRE(ctrl.poll_verified_candidate(slot));
    BOOST_CHECK_EQUAL(slot.win_start_abs, 0);
    BOOST_CHECK_LE(std::abs(slot.predicted_start_abs - kEarlyStart), 4);
    BOOST_CHECK_LE(slot.sample_count, ctrl.max_slot_native_capacity());
    BOOST_REQUIRE_GT(slot.sample_count, static_cast<size_t>(slot.predicted_start_abs));

    // Check sample alignment across the emitted slot against the source stream.
    for (size_t i = 0; i < slot.sample_count; ++i) {
        BOOST_REQUIRE_EQUAL(slot.samples[i],
                            stream[static_cast<size_t>(slot.win_start_abs) + i]);
    }
    const size_t rel_off =
        static_cast<size_t>(slot.predicted_start_abs - slot.win_start_abs);
    int64_t sym_energy = 0;
    for (size_t i = 0; i < kWorkSym && rel_off + i < slot.sample_count; ++i) {
        sym_energy += static_cast<int64_t>(slot.samples[rel_off + i].real()) *
                          slot.samples[rel_off + i].real() +
                      static_cast<int64_t>(slot.samples[rel_off + i].imag()) *
                          slot.samples[rel_off + i].imag();
    }
    BOOST_CHECK_GT(sym_energy, 0);

    // Check timestamp discontinuity detection via has_time_spec jump.
    int64_t end_abs = static_cast<int64_t>(stream.size());
    BOOST_CHECK(!ctrl.handle_stream_meta(0, 4000, true, 0.0, false, 0, end_abs));
    const double jumped_time_s =
        static_cast<double>(stream.size() + 8000) / kWorkRate;
    // With period_s=0.005, tol=2.5 ms; test a jump > 3 ms.
    const int64_t big_gap = static_cast<int64_t>(std::llround(0.004 * kWorkRate));
    const double big_jump_s =
        static_cast<double>(static_cast<int64_t>(stream.size()) + big_gap) /
        kWorkRate;
    const int64_t saved_end = end_abs;
    BOOST_CHECK(ctrl.handle_stream_meta(
        static_cast<int64_t>(stream.size()), 4000, true, big_jump_s, false, 0,
        end_abs));
    BOOST_CHECK_EQUAL(end_abs, saved_end + big_gap);
    (void)jumped_time_s;

    // BoundedFeedbackQueue overflow and drain invariants.
    pipeline::BoundedFeedbackQueue q(4);
    for (uint64_t i = 0; i < 6; ++i) {
        pipeline::LockFeedback fb;
        fb.window_id = i;
        q.push(fb);
    }
    BOOST_CHECK_EQUAL(q.dropped(), 2u);
    std::vector<pipeline::LockFeedback> drained;
    BOOST_CHECK_EQUAL(q.drain(drained), 4u);
    BOOST_CHECK_EQUAL(drained.front().window_id, 2u);
    BOOST_CHECK_EQUAL(drained.back().window_id, 5u);
    q.wake_all();
    BOOST_CHECK(!q.wait_and_drain(drained, 10));
}

BOOST_AUTO_TEST_CASE(test_autocorr_gate_rejects_noise_and_detects_sync)
{
    pipeline::AutoScheduleConfig acfg;
    acfg.period_s = 0.002;
    acfg.use_autocorr_gate = true;
    acfg.autocorr_threshold = 0.35f;
    acfg.autocorr_min_energy = 1e-4f;
    acfg.autocorr_segment_len = 250;
    acfg.autocorr_window_segments = 8;
    acfg.autocorr_holdoff_segments = 8;
    // Set energy_threshold below noise floor so only autocorr gate can reject noise.
    acfg.energy_threshold = 0.001f;
    acfg.acq_pre_guard_samples = 4096;
    acfg.acq_post_guard_samples = 2048;
    acfg.verifier_coarse_stride = 1;

    pipeline::AutoScheduleController::TargetProfile target;
    target.code_index = 9;
    target.sync_repetitions = 16;
    target.sfd_mode = "4z2";
    target.is_comm = false;

    const auto wf = generate_work_packet(9, 16, "4z2", /*with_payload=*/false);
    // Test 1: Work rate (998.4 MS/s, exact lag = 1016)
    {
        const auto pkt_sc16 = float_to_sc16(wf, 6000.0f);
        constexpr size_t kNoiseLen = 40000;
        constexpr int64_t kSyncStart = 48000;
        std::vector<sc16> stream(kSyncStart + pkt_sc16.size() + 40000, sc16(0, 0));

        // Deterministic broadband complex noise burst above energy_threshold (rms ~ 0.12)
        uint32_t rng = 0x13572468U;
        auto next_i16 = [&]() -> int16_t {
            rng = rng * 1664525U + 1013904223U;
            const int32_t v = static_cast<int32_t>((rng >> 16) & 0xFFFFU) - 32768;
            return static_cast<int16_t>(v / 10); // +-3276 full scale
        };
        for (size_t i = 2000; i < kNoiseLen; ++i) {
            stream[i] = sc16(next_i16(), next_i16());
        }
        stamp_packet(stream, 0, kSyncStart, pkt_sc16);

        pipeline::AutoScheduleController ctrl;
        ctrl.configure(acfg, target, kWorkRate, kWorkRate, 1024, pkt_sc16.size(), 1024);

        // Feed only the broadband noise region first: autocorr gate must stay quiet.
        ctrl.feed_acquisition(stream.data(), kNoiseLen, 0);
        pipeline::AutoScheduleController::EmittedAcqSlot slot;
        BOOST_CHECK(!ctrl.poll_verified_candidate(slot));

        // Feed remaining samples including the SYNC packet and trailing tail.
        ctrl.feed_acquisition(stream.data() + kNoiseLen, stream.size() - kNoiseLen, kNoiseLen);
        ctrl.flush_acquisition();
        BOOST_REQUIRE(ctrl.poll_verified_candidate(slot));
        BOOST_CHECK_LE(std::abs(slot.predicted_start_abs - kSyncStart), 4);
    }

    // Test 2: Native 737.28 MS/s rate (dual-lag 750/751)
    {
        const auto ds_taps = make_downsample_48_65_taps(8);
        gr::uwb::core::RationalResampler48_65Core ds(ds_taps.data(), ds_taps.size());
        std::vector<cf> wf_737(ds.expected_output_length(wf.size()) + 128);
        const auto pr = ds.process(wf.data(), wf.size(), wf_737.data(), wf_737.size());
        const size_t flushed = ds.flush(wf_737.data() + pr.produced, wf_737.size() - pr.produced);
        wf_737.resize(pr.produced + flushed);
        const auto pkt_737 = float_to_sc16(wf_737, 6000.0f);

        constexpr size_t kNoiseLen = 32000;
        constexpr int64_t kSyncStart = 40000;
        std::vector<sc16> stream(kSyncStart + pkt_737.size() + 40000, sc16(0, 0));

        uint32_t rng = 0x24681357U;
        auto next_i16 = [&]() -> int16_t {
            rng = rng * 1664525U + 1013904223U;
            const int32_t v = static_cast<int32_t>((rng >> 16) & 0xFFFFU) - 32768;
            return static_cast<int16_t>(v / 10);
        };
        for (size_t i = 2000; i < kNoiseLen; ++i) {
            stream[i] = sc16(next_i16(), next_i16());
        }
        stamp_packet(stream, 0, kSyncStart, pkt_737);

        pipeline::AutoScheduleController ctrl;
        ctrl.configure(acfg, target, kNativeRate737, kWorkRate, 1024, wf.size(), 1024);

        ctrl.feed_acquisition(stream.data(), kNoiseLen, 0);
        pipeline::AutoScheduleController::EmittedAcqSlot slot;
        BOOST_CHECK(!ctrl.poll_verified_candidate(slot));

        ctrl.feed_acquisition(stream.data() + kNoiseLen, stream.size() - kNoiseLen, kNoiseLen);
        ctrl.flush_acquisition();
        BOOST_REQUIRE(ctrl.poll_verified_candidate(slot));
        // Resampler group delay shifts the 737.28 MS/s envelope peak slightly (~8 samples)
        BOOST_CHECK_LE(std::abs(slot.predicted_start_abs - kSyncStart), 24);
    }
}
