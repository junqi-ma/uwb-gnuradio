/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * QA for Unified UWB SIC Interface (uwb_sic_unified.h).
 */

#include <boost/test/unit_test.hpp>

#include <gnuradio/uwb/uwb_sic_unified.h>
#include <gnuradio/uwb/uwb_hrp_mod_core.h>

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdint>
#include <string>
#include <vector>

namespace {

using namespace gr::uwb;
using namespace gr::uwb::sic;
using cf = std::complex<float>;

constexpr double kFs = 998.4e6;
constexpr size_t kSps = 1016;
constexpr size_t kReps = 8;
constexpr size_t kTrainLen = kReps * kSps;

struct TestPacket {
    mod::HrpModConfig profile;
    mod::HrpModScratch mod_scratch;
    std::vector<cf> wave;
    std::vector<cf> train;
    std::vector<cf> sync;
    std::vector<uint8_t> psdu;

    explicit TestPacket(size_t code = 10, size_t psdu_len = 32, size_t sync_reps = 64)
    {
        profile.code_index = code;
        profile.sync_repetitions = sync_reps;
        profile.sfd_mode = "4z2";
        profile.peak_amplitude = 0.8f;

        psdu = mod::make_random_psdu(psdu_len - 2, static_cast<uint32_t>(1000 + code), true);
        const size_t cap = mod::packet_samples_998p4(sync_reps, 8, psdu.size(), false, &profile.pulse) + 512;
        mod_scratch.reserve(127, cap);
        wave.resize(cap);
        size_t plen = 0;
        BOOST_REQUIRE(mod::modulate_one(psdu.data(), psdu.size(), profile,
                                        mod_scratch, wave.data(), cap, plen));
        wave.resize(plen);
        train.assign(wave.begin(), wave.begin() + kTrainLen);
        sync.assign(wave.begin(), wave.begin() + kSps);
    }
};

SicUnifiedTemplates make_templates(const TestPacket& p)
{
    SicUnifiedTemplates t;
    t.sync = p.sync.data();
    t.sync_len = p.sync.size();
    t.preamble = p.wave.data();
    t.preamble_len = p.wave.size();
    return t;
}

} // namespace

BOOST_AUTO_TEST_SUITE(qa_uwb_sic_unified)

BOOST_AUTO_TEST_CASE(test_unified_enums_and_bitmasks)
{
    // Test SicPolicy
    BOOST_CHECK(SicPolicy::PreambleOnly != SicPolicy::FullPacket);
    BOOST_CHECK(SicPolicy::AutoFallback != SicPolicy::FullPacket);

    // Test SicEstimatorBackend
    BOOST_CHECK(SicEstimatorBackend::DiscreteCir != SicEstimatorBackend::PathFractional);
    BOOST_CHECK(SicEstimatorBackend::DiscreteCir != SicEstimatorBackend::Ipatov);

    // Test SicCancellationLevel
    BOOST_CHECK_EQUAL(static_cast<uint8_t>(SicCancellationLevel::L0_Bypass), 0);
    BOOST_CHECK_EQUAL(static_cast<uint8_t>(SicCancellationLevel::L1_Preamble), 1);
    BOOST_CHECK_EQUAL(static_cast<uint8_t>(SicCancellationLevel::L2_FullPacket), 2);

    // Test SicKnownFields bitwise operations
    SicKnownFields fields = SicKnownFields::Sync | SicKnownFields::Sfd;
    BOOST_CHECK(has_field(fields, SicKnownFields::Sync));
    BOOST_CHECK(has_field(fields, SicKnownFields::Sfd));
    BOOST_CHECK(!has_field(fields, SicKnownFields::Payload));

    // Verify: Preamble is a special case of FullPacket
    BOOST_CHECK(has_field(SicKnownFields::FullPacket, SicKnownFields::Preamble));
    BOOST_CHECK(has_field(SicKnownFields::FullPacket, SicKnownFields::Payload));
    BOOST_CHECK(has_field(SicKnownFields::FullPacket, SicKnownFields::Fcs));
}

BOOST_AUTO_TEST_CASE(test_unified_channel_conversion)
{
    DiscreteCirModel dm;
    dm.valid = true;
    dm.status = "model_ready";
    dm.model_id = 0x12345678ULL;
    dm.q = 500;
    dm.cfo_hz = 12000.0;
    dm.lag_lo = -4;
    dm.tap_count = 16;
    dm.active_count = 2;
    dm.active_lags[0] = 0;
    dm.active_lags[1] = 5;
    dm.taps[0] = cf(0.8f, 0.2f);
    dm.taps[5] = cf(0.1f, -0.3f);

    SicUnifiedChannel ch;
    ch.from_discrete_cir(dm);
    BOOST_CHECK(ch.valid);
    BOOST_CHECK(ch.backend == SicEstimatorBackend::DiscreteCir);
    BOOST_CHECK_EQUAL(ch.model_id, 0x12345678ULL);
    BOOST_CHECK_EQUAL(ch.q, 500);
    BOOST_CHECK_CLOSE(ch.cfo_hz, 12000.0, 1e-4);
    BOOST_CHECK_EQUAL(ch.tap_count, 16);
    BOOST_CHECK_EQUAL(ch.active_count, 2);

    DiscreteCirModel dm_back;
    ch.to_discrete_cir(dm_back);
    BOOST_CHECK_EQUAL(dm_back.model_id, dm.model_id);
    BOOST_CHECK_EQUAL(dm_back.q, dm.q);
    BOOST_CHECK_EQUAL(dm_back.tap_count, dm.tap_count);
    BOOST_CHECK_EQUAL(dm_back.taps[0], dm.taps[0]);

    // Test PathFractional conversion
    SicModel sm;
    sm.valid = true;
    sm.status = "model_ready";
    sm.model_id = 0x87654321ULL;
    sm.origin = 600;
    sm.cfo_hz = -5000.0;
    sm.n_paths = 2;
    sm.tau_abs[0] = 600.25;
    sm.tau_abs[1] = 605.50;
    sm.gain[0] = cf(0.7f, 0.1f);
    sm.gain[1] = cf(0.2f, -0.1f);

    ch.from_parametric(sm);
    BOOST_CHECK(ch.valid);
    BOOST_CHECK(ch.backend == SicEstimatorBackend::PathFractional);
    BOOST_CHECK_EQUAL(ch.model_id, 0x87654321ULL);
    BOOST_CHECK_EQUAL(ch.q, 600);
    BOOST_CHECK_EQUAL(ch.n_paths, 2);

    SicModel sm_back;
    ch.to_parametric(sm_back);
    BOOST_CHECK_EQUAL(sm_back.model_id, sm.model_id);
    BOOST_CHECK_EQUAL(sm_back.origin, sm.origin);
    BOOST_CHECK_EQUAL(sm_back.n_paths, sm.n_paths);
    BOOST_CHECK_EQUAL(sm_back.tau_abs[0], sm.tau_abs[0]);
}

BOOST_AUTO_TEST_CASE(test_unified_prepare_and_zero_alloc_scratch)
{
    TestPacket p(10, 32);
    SicUnifiedTemplates t = make_templates(p);

    SicUnifiedConfig cfg;
    cfg.comm_code = 10;
    cfg.samples_per_symbol = kSps;
    cfg.sample_rate = kFs;
    cfg.max_window_samples = p.wave.size() + 2048;
    cfg.max_packet_samples = p.wave.size() + 2048;

    SicUnifiedScratch scratch;
    std::string err;
    BOOST_REQUIRE(prepare_sic_unified(cfg, t, scratch, &err));
    BOOST_CHECK(scratch.prepared);
    BOOST_CHECK_GE(scratch.capacity_window_samples, p.wave.size());
}

BOOST_AUTO_TEST_CASE(test_unified_estimate_strict_prefix)
{
    TestPacket p(10, 32);
    SicUnifiedTemplates t = make_templates(p);

    const size_t q = 500;
    const size_t n = p.wave.size() + 1000;
    std::vector<cf> y(n, cf(0.f, 0.f));
    std::copy(p.wave.begin(), p.wave.end(), y.begin() + q);

    SicUnifiedConfig cfg;
    cfg.comm_code = 10;
    cfg.samples_per_symbol = kSps;
    cfg.sample_rate = kFs;
    cfg.roi_lo = q - 32;
    cfg.roi_hi = q + 32;
    cfg.max_window_samples = n;
    cfg.max_packet_samples = n;
    // Strict prefix: only the first 8 SYNC symbols are visible
    const int64_t obs_end = q + 8 * kSps;
    cfg.observed_end_sample = obs_end;

    SicUnifiedScratch scratch;
    BOOST_REQUIRE(prepare_sic_unified(cfg, t, scratch));

    SicUnifiedChannel ch;
    SicUnifiedTimings timings;
    BOOST_REQUIRE(estimate_channel_unified(y.data(), n, obs_end, t, cfg, scratch, ch, &timings));

    BOOST_CHECK(ch.valid);
    BOOST_CHECK_EQUAL(ch.q, static_cast<int64_t>(q));
    BOOST_CHECK_EQUAL(ch.observed_end_sample, obs_end);
    BOOST_CHECK_GT(timings.estimate_us, 0.0);
}

BOOST_AUTO_TEST_CASE(test_unified_process_preamble_only)
{
    TestPacket p(10, 32);
    SicUnifiedTemplates t = make_templates(p);

    const size_t q = 300;
    const size_t n = p.wave.size() + 1000;
    std::vector<cf> y(n, cf(0.f, 0.f));
    std::copy(p.wave.begin(), p.wave.end(), y.begin() + q);

    SicUnifiedConfig cfg;
    cfg.policy = SicPolicy::PreambleOnly;
    cfg.backend = SicEstimatorBackend::DiscreteCir;
    cfg.comm_code = 10;
    cfg.samples_per_symbol = kSps;
    cfg.sample_rate = kFs;
    cfg.roi_lo = q - 32;
    cfg.roi_hi = q + 32;
    cfg.sub_roi_lo = q;
    cfg.sub_roi_hi = q + 64 * kSps;
    cfg.max_window_samples = n;
    cfg.max_packet_samples = n;

    SicUnifiedScratch scratch;
    BOOST_REQUIRE(prepare_sic_unified(cfg, t, scratch));

    std::vector<cf> out(n);
    std::vector<cf> i_hat(n);

    SicUnifiedResult res;
    res.out = out.data();
    res.i_hat = i_hat.data();

    BOOST_REQUIRE(sic_process_unified(y.data(), n, t, cfg, scratch, res));

    BOOST_CHECK(res.committed);
    BOOST_CHECK_EQUAL(static_cast<uint8_t>(res.level), static_cast<uint8_t>(SicCancellationLevel::L1_Preamble));
    BOOST_CHECK(has_field(res.cancelled_fields, SicKnownFields::Preamble));
    BOOST_CHECK(has_field(res.cancelled_fields, SicKnownFields::Sfd)); // Now covers SFD!
    BOOST_CHECK(!has_field(res.cancelled_fields, SicKnownFields::Payload));
    BOOST_CHECK_GT(res.suppression_db, 20.0); // Clean synthetic packet has high suppression
}

BOOST_AUTO_TEST_CASE(test_unified_auto_fallback_on_corrupted_payload)
{
    TestPacket p(10, 32);
    SicUnifiedTemplates t = make_templates(p);

    const size_t q = 400;
    const size_t n = p.wave.size() + 1000;
    std::vector<cf> y(n, cf(0.f, 0.f));
    std::copy(p.wave.begin(), p.wave.end(), y.begin() + q);

    // Corrupt payload region (after SYNC + SFD) so demodulation / FCS fails!
    const size_t payload_start = q + 64 * kSps + 8 * kSps;
    for (size_t i = payload_start; i < q + p.wave.size(); ++i) {
        y[i] += cf(10.0f, -10.0f); // severe corruption to destroy FCS/PHR
    }

    SicUnifiedConfig cfg;
    cfg.policy = SicPolicy::AutoFallback; // Adaptive policy!
    cfg.backend = SicEstimatorBackend::DiscreteCir;
    cfg.comm_code = 10;
    cfg.samples_per_symbol = kSps;
    cfg.sample_rate = kFs;
    cfg.roi_lo = q - 32;
    cfg.roi_hi = q + 32;
    cfg.sub_roi_lo = q;
    cfg.sub_roi_hi = q + 64 * kSps;
    cfg.max_window_samples = n;
    cfg.max_packet_samples = n;

    SicUnifiedScratch scratch;
    BOOST_REQUIRE(prepare_sic_unified(cfg, t, scratch));

    std::vector<cf> out(n);
    std::vector<cf> i_hat(n);

    SicUnifiedResult res;
    res.out = out.data();
    res.i_hat = i_hat.data();

    BOOST_REQUIRE(sic_process_unified(y.data(), n, t, cfg, scratch, res));

    // KEY REQUIREMENT: In AutoFallback, it must NOT bypass to L0;
    // it must gracefully degrade to L1_Preamble!
    BOOST_CHECK(res.committed);
    BOOST_CHECK_EQUAL(static_cast<uint8_t>(res.level), static_cast<uint8_t>(SicCancellationLevel::L1_Preamble));
    BOOST_CHECK(has_field(res.cancelled_fields, SicKnownFields::Preamble));
    BOOST_CHECK(has_field(res.cancelled_fields, SicKnownFields::Sfd)); // Now covers SFD!
    BOOST_CHECK(!has_field(res.cancelled_fields, SicKnownFields::Payload));
    BOOST_CHECK(res.fallback_reason != nullptr); // Fallback reason is explicitly recorded
    BOOST_CHECK_GT(res.suppression_db, 15.0); // Preamble is still successfully eliminated!

    // Now test with strict SicPolicy::FullPacket on the SAME corrupted signal:
    // Strict full-packet policy must abort to L0_Bypass!
    cfg.policy = SicPolicy::FullPacket;
    SicUnifiedResult res_strict;
    res_strict.out = out.data();
    res_strict.i_hat = i_hat.data();

    BOOST_REQUIRE(sic_process_unified(y.data(), n, t, cfg, scratch, res_strict));

    BOOST_CHECK(!res_strict.committed);
    BOOST_CHECK_EQUAL(static_cast<uint8_t>(res_strict.level), static_cast<uint8_t>(SicCancellationLevel::L0_Bypass));
    BOOST_CHECK_EQUAL(res_strict.suppression_db, 0.0);
    // Bit-identical fail-closed guarantee: out == y
    for (size_t i = 0; i < n; ++i) {
        BOOST_CHECK_EQUAL(out[i], y[i]);
    }
}

BOOST_AUTO_TEST_CASE(test_unified_fail_closed_on_invalid_input)
{
    SicUnifiedConfig cfg;
    SicUnifiedTemplates t;
    SicUnifiedScratch scratch;

    std::vector<cf> y(100, cf(1.f, 1.f));
    std::vector<cf> out(100, cf(0.f, 0.f));

    SicUnifiedResult res;
    res.out = out.data();

    // Calling sic_process_unified with unprepared scratch -> fail closed, out == y
    BOOST_CHECK(!sic_process_unified(y.data(), y.size(), t, cfg, scratch, res));
    for (size_t i = 0; i < y.size(); ++i) {
        BOOST_CHECK_EQUAL(out[i], y[i]);
    }
}

BOOST_AUTO_TEST_CASE(test_scratch_reuse_no_realloc)
{
    TestPacket p(10, 32);
    SicUnifiedTemplates t = make_templates(p);

    const size_t q = 300;
    const size_t n = p.wave.size() + 1000;
    std::vector<cf> y(n, cf(0.f, 0.f));
    std::copy(p.wave.begin(), p.wave.end(), y.begin() + q);

    SicUnifiedConfig cfg;
    cfg.policy = SicPolicy::PreambleOnly;
    cfg.backend = SicEstimatorBackend::Ipatov;
    cfg.sparse_pulse_reconstruct = true;
    cfg.comm_code = 10;
    cfg.samples_per_symbol = kSps;
    cfg.sample_rate = kFs;
    cfg.roi_lo = q - 32;
    cfg.roi_hi = q + 32;
    cfg.sub_roi_lo = q;
    cfg.sub_roi_hi = q + 64 * kSps;
    cfg.max_window_samples = n;
    cfg.max_packet_samples = n;

    SicUnifiedScratch scratch;
    BOOST_REQUIRE(prepare_sic_unified(cfg, t, scratch));
    BOOST_CHECK(scratch.preamble_cache_valid);

    const void* const p_events = scratch.cached_preamble_events.data();
    const void* const p_sync = scratch.cached_sync_vec.data();
    const void* const p_wf = scratch.cached_preamble_waveform.data();
    const size_t cap_events = scratch.cached_preamble_events.capacity();
    const size_t cap_sync = scratch.cached_sync_vec.capacity();
    const size_t cap_wf = scratch.cached_preamble_waveform.capacity();

    std::vector<cf> out(n);
    std::vector<cf> i_hat(n);

    // Repeated calls with same config reuse scratch without re-allocation
    for (int iter = 0; iter < 5; ++iter) {
        SicUnifiedResult res;
        res.out = out.data();
        res.i_hat = i_hat.data();
        BOOST_REQUIRE(sic_process_unified(y.data(), n, t, cfg, scratch, res));
        BOOST_CHECK(res.committed);

        BOOST_CHECK_EQUAL(scratch.cached_preamble_events.data(), p_events);
        BOOST_CHECK_EQUAL(scratch.cached_sync_vec.data(), p_sync);
        BOOST_CHECK_EQUAL(scratch.cached_preamble_waveform.data(), p_wf);
        BOOST_CHECK_EQUAL(scratch.cached_preamble_events.capacity(), cap_events);
        BOOST_CHECK_EQUAL(scratch.cached_sync_vec.capacity(), cap_sync);
        BOOST_CHECK_EQUAL(scratch.cached_preamble_waveform.capacity(), cap_wf);
    }
}

BOOST_AUTO_TEST_CASE(test_unified_ipatov_preamble_uses_unit_impulse_prefix)
{
    TestPacket p(10, 32);
    SicUnifiedTemplates t = make_templates(p);

    const size_t q = 300;
    const size_t n = p.wave.size() + 1000;
    std::vector<cf> y(n, cf(0.f, 0.f));
    std::copy(p.wave.begin(), p.wave.end(), y.begin() + q);

    SicUnifiedConfig cfg;
    cfg.policy = SicPolicy::PreambleOnly;
    cfg.backend = SicEstimatorBackend::Ipatov;
    cfg.sparse_pulse_reconstruct = false; // exercise the non-sparse apply path
    cfg.comm_code = 10;
    cfg.samples_per_symbol = kSps;
    cfg.sample_rate = kFs;
    cfg.roi_lo = q - 32;
    cfg.roi_hi = q + 32;
    cfg.sub_roi_lo = q;
    cfg.sub_roi_hi = q + 64 * kSps;
    cfg.max_window_samples = n;
    cfg.max_packet_samples = n;

    SicUnifiedScratch scratch;
    BOOST_REQUIRE(prepare_sic_unified(cfg, t, scratch));

    // The cached Ipatov prefix must be the UNIT impulse SYNC + SFD span, not
    // the shaped waveform and not the PHR-extended span.
    const size_t prefix_span = 64 * kSps + 8 * kSps;
    BOOST_REQUIRE_EQUAL(scratch.cached_preamble_samples, prefix_span);
    BOOST_REQUIRE_EQUAL(scratch.cached_preamble_unit_waveform.size(),
                        prefix_span);
    const int8_t* code = demod::GetPreambleCode(10);
    BOOST_REQUIRE(code != nullptr);
    for (size_t i = 0; i < kSps; ++i) {
        const float want = (i % 8 == 0 && i / 8 < demod::kQm35CodeLength)
                               ? static_cast<float>(code[i / 8])
                               : 0.f;
        BOOST_REQUIRE_EQUAL(scratch.cached_preamble_unit_waveform[i].imag(), 0.f);
        BOOST_REQUIRE_EQUAL(scratch.cached_preamble_unit_waveform[i].real(), want);
    }

    std::vector<cf> out(n);
    std::vector<cf> i_hat(n);
    SicUnifiedResult res;
    res.out = out.data();
    res.i_hat = i_hat.data();

    BOOST_REQUIRE(sic_process_unified(y.data(), n, t, cfg, scratch, res));

    BOOST_CHECK(res.committed);
    BOOST_CHECK_EQUAL(std::string(res.reason), "committed_preamble_ipatov");
    BOOST_CHECK_EQUAL(static_cast<uint8_t>(res.level),
                      static_cast<uint8_t>(SicCancellationLevel::L1_Preamble));
    BOOST_CHECK(has_field(res.cancelled_fields, SicKnownFields::Preamble));
    BOOST_CHECK(has_field(res.cancelled_fields, SicKnownFields::Sfd));
    BOOST_CHECK(!has_field(res.cancelled_fields, SicKnownFields::Payload));
    // The unit impulse prefix is matched exactly against the unit reference,
    // so the single waveform scale must be exactly 1.
    BOOST_CHECK_CLOSE(res.waveform_scale, 1.0, 1e-6);
    BOOST_CHECK_GT(res.suppression_db, 20.0);

    // Committed transaction: out == y - i_hat inside the subtraction ROI and
    // untouched outside it.
    BOOST_REQUIRE(res.sub_hi > res.sub_lo);
    for (int64_t j = 0; j < static_cast<int64_t>(n); ++j) {
        const size_t s = static_cast<size_t>(j);
        if (j < res.sub_lo || j >= res.sub_hi) {
            BOOST_CHECK_EQUAL(out[s], y[s]);
        } else {
            BOOST_CHECK_LT(std::abs(out[s] - (y[s] - i_hat[s])), 1e-6f);
        }
    }

    // The same configuration with the sparse representation must also commit;
    // the two paths are distinct reasons, never a silent substitution.
    cfg.sparse_pulse_reconstruct = true;
    BOOST_REQUIRE(prepare_sic_unified(cfg, t, scratch));
    SicUnifiedResult res_sparse;
    res_sparse.out = out.data();
    res_sparse.i_hat = i_hat.data();
    BOOST_REQUIRE(sic_process_unified(y.data(), n, t, cfg, scratch, res_sparse));
    BOOST_CHECK(res_sparse.committed);
    BOOST_CHECK_EQUAL(std::string(res_sparse.reason),
                      "committed_preamble_sparse_pulse");
    BOOST_CHECK_GT(res_sparse.suppression_db, 20.0);
}

BOOST_AUTO_TEST_CASE(test_ipatov_256_sync_cancels_tail_without_extending_training)
{
    TestPacket packet(10,127,256);
    auto templates=make_templates(packet);
    const size_t q=300,n=packet.wave.size()+1024;
    std::vector<cf> y(n),out(n),ihat(n);
    std::copy(packet.wave.begin(),packet.wave.end(),y.begin()+q);
    SicUnifiedConfig cfg;
    cfg.policy=SicPolicy::PreambleOnly;cfg.backend=SicEstimatorBackend::Ipatov;
    cfg.comm_code=10;cfg.sync_reps=256;cfg.fit_reps=8;
    cfg.roi_lo=q-32;cfg.roi_hi=q+32;cfg.sub_roi_lo=q;cfg.sub_roi_hi=q+256*kSps;
    cfg.max_window_samples=n;cfg.max_packet_samples=n;
    SicUnifiedScratch scratch;std::string error;
    BOOST_REQUIRE_MESSAGE(prepare_sic_unified(cfg,templates,scratch,&error),error);
    BOOST_CHECK_EQUAL(scratch.cached_preamble_samples,(256+8)*kSps);
    SicUnifiedResult result;result.out=out.data();result.i_hat=ihat.data();
    BOOST_REQUIRE(sic_process_unified(y.data(),n,templates,cfg,scratch,result));
    BOOST_REQUIRE(result.committed);
    BOOST_CHECK_EQUAL(std::string(result.reason),"committed_preamble_sparse_pulse");
    BOOST_CHECK_GT(result.suppression_db,20.0);
    BOOST_CHECK_LE(result.channel.training_hi,static_cast<int64_t>(q+8*kSps));
    double original_tail=0,residual_tail=0;
    for(size_t j=q+252*kSps;j<q+256*kSps;++j) {
        original_tail+=std::norm(y[j]);residual_tail+=std::norm(out[j]);
    }
    BOOST_CHECK_LT(residual_tail,original_tail*.01);
    for(size_t j=q+256*kSps;j<n;++j) BOOST_REQUIRE_EQUAL(out[j],y[j]);
    cfg.sync_reps=100;
    BOOST_CHECK(!prepare_sic_unified(cfg,templates,scratch,&error));
    BOOST_CHECK_EQUAL(error,"unsupported Ipatov geometry");
}

BOOST_AUTO_TEST_CASE(test_config_change_invalidates_and_regenerates)
{
    TestPacket p10(10, 32);
    SicUnifiedTemplates t10 = make_templates(p10);

    const size_t q = 300;
    const size_t n = p10.wave.size() + 1000;
    std::vector<cf> y(n, cf(0.f, 0.f));
    std::copy(p10.wave.begin(), p10.wave.end(), y.begin() + q);

    SicUnifiedConfig cfg;
    cfg.policy = SicPolicy::PreambleOnly;
    cfg.backend = SicEstimatorBackend::Ipatov;
    cfg.sparse_pulse_reconstruct = true;
    cfg.comm_code = 10;
    cfg.samples_per_symbol = kSps;
    cfg.sample_rate = kFs;
    cfg.roi_lo = q - 32;
    cfg.roi_hi = q + 32;
    cfg.sub_roi_lo = q;
    cfg.sub_roi_hi = q + 64 * kSps;
    cfg.max_window_samples = n;
    cfg.max_packet_samples = n;

    SicUnifiedScratch scratch;
    BOOST_REQUIRE(prepare_sic_unified(cfg, t10, scratch));
    BOOST_CHECK(scratch.preamble_cache_valid);

    const uint64_t hash10 = scratch.config_hash;
    std::vector<mod::PulseEvent> events10 = scratch.cached_preamble_events;
    BOOST_CHECK(!events10.empty());

    // Changing comm_code without re-preparing must cause cache_matches to return false
    cfg.comm_code = 9;
    BOOST_CHECK(!scratch.cache_matches(cfg, t10));

    std::vector<cf> out(n);
    SicUnifiedResult res;
    res.out = out.data();

    // Hot path rejects config mismatch fail-closed
    BOOST_CHECK(!sic_process_unified(y.data(), n, t10, cfg, scratch, res));
    BOOST_CHECK_EQUAL(std::string(res.reason), "config_or_template_mismatch");

    // Cleanly re-prepare with code 9
    TestPacket p9(9, 32);
    SicUnifiedTemplates t9 = make_templates(p9);
    BOOST_REQUIRE(prepare_sic_unified(cfg, t9, scratch));
    BOOST_CHECK(scratch.preamble_cache_valid);
    BOOST_CHECK(scratch.cache_matches(cfg, t9));
    BOOST_CHECK_NE(scratch.config_hash, hash10);

    // Code 9 cached pulse events must differ from code 10
    const auto& events9 = scratch.cached_preamble_events;
    BOOST_CHECK(!events9.empty());
    bool different = (events9.size() != events10.size());
    if (!different) {
        for (size_t i = 0; i < events9.size(); ++i) {
            if (events9[i].sample_offset != events10[i].sample_offset ||
                events9[i].sign != events10[i].sign) {
                different = true;
                break;
            }
        }
    }
    BOOST_CHECK(different);
}

BOOST_AUTO_TEST_CASE(test_fullpacket_modulation_does_not_corrupt_cached_events)
{
    TestPacket p(10, 32);
    SicUnifiedTemplates t = make_templates(p);

    const size_t q = 300;
    const size_t n = p.wave.size() + 1000;
    std::vector<cf> y(n, cf(0.f, 0.f));
    std::copy(p.wave.begin(), p.wave.end(), y.begin() + q);

    SicUnifiedConfig cfg;
    cfg.policy = SicPolicy::FullPacket;
    cfg.backend = SicEstimatorBackend::Ipatov;
    cfg.sparse_pulse_reconstruct = true;
    cfg.comm_code = 10;
    cfg.samples_per_symbol = kSps;
    cfg.sample_rate = kFs;
    cfg.roi_lo = q - 32;
    cfg.roi_hi = q + 32;
    cfg.sub_roi_lo = q;
    cfg.sub_roi_hi = q + p.wave.size();
    cfg.max_window_samples = n;
    cfg.max_packet_samples = n;

    SicUnifiedScratch scratch;
    BOOST_REQUIRE(prepare_sic_unified(cfg, t, scratch));
    BOOST_CHECK(scratch.preamble_cache_valid);

    const std::vector<mod::PulseEvent> golden_events = scratch.cached_preamble_events;
    const size_t golden_samples = scratch.cached_preamble_samples;
    BOOST_REQUIRE(!golden_events.empty());

    std::vector<cf> out(n);
    std::vector<cf> i_hat(n);
    SicUnifiedResult res;
    res.out = out.data();
    res.i_hat = i_hat.data();

    // Process full-packet cancellation (triggers full PSDU modulation and pulse generation)
    BOOST_REQUIRE(sic_process_unified(y.data(), n, t, cfg, scratch, res));
    BOOST_CHECK(res.committed);
    BOOST_CHECK_EQUAL(static_cast<uint8_t>(res.level), static_cast<uint8_t>(SicCancellationLevel::L2_FullPacket));

    // Verify cached preamble events were NOT corrupted by full packet modulation
    BOOST_CHECK_EQUAL(scratch.cached_preamble_events.size(), golden_events.size());
    BOOST_CHECK_EQUAL(scratch.cached_preamble_samples, golden_samples);
    for (size_t i = 0; i < golden_events.size(); ++i) {
        BOOST_CHECK_EQUAL(scratch.cached_preamble_events[i].sample_offset, golden_events[i].sample_offset);
        BOOST_CHECK_EQUAL(scratch.cached_preamble_events[i].sign, golden_events[i].sign);
    }

    // Now switch to PreambleOnly policy to confirm it still works seamlessly using untainted cache
    cfg.policy = SicPolicy::PreambleOnly;
    SicUnifiedResult res_pre;
    res_pre.out = out.data();
    res_pre.i_hat = i_hat.data();
    BOOST_REQUIRE(sic_process_unified(y.data(), n, t, cfg, scratch, res_pre));
    BOOST_CHECK(res_pre.committed);
    BOOST_CHECK_EQUAL(static_cast<uint8_t>(res_pre.level), static_cast<uint8_t>(SicCancellationLevel::L1_Preamble));
}

BOOST_AUTO_TEST_CASE(test_simd_roi_energy_reduction)
{
    constexpr size_t kTotal = 8192;
    std::vector<cf> y(kTotal);
    std::vector<cf> out(kTotal);

    for (size_t i = 0; i < kTotal; ++i) {
        float f1 = static_cast<float>(std::sin(0.017 * static_cast<double>(i)));
        float f2 = static_cast<float>(std::cos(0.023 * static_cast<double>(i)));
        float f3 = 0.1f * static_cast<float>(std::sin(0.031 * static_cast<double>(i)));
        float f4 = 0.1f * static_cast<float>(std::cos(0.047 * static_cast<double>(i)));
        y[i] = cf(f1, f2);
        out[i] = cf(f3, f4);
    }

    // Test different slice ranges (aligned, unaligned, odd, large)
    const std::vector<std::pair<int64_t, int64_t>> ranges = {
        {0, 0},         // Empty
        {10, 10},       // Empty offset
        {0, 1},         // Single sample
        {0, 3},         // 3 samples (sub-vector tail)
        {0, 4},         // 4 samples
        {0, 7},         // 7 samples
        {0, 8},         // 8 samples (1 unroll group)
        {0, 15},        // 15 samples
        {0, 16},        // 16 samples
        {3, 19},        // Unaligned start and end (16 samples)
        {7, 101},       // Unaligned start and end (94 samples)
        {0, 1016},      // Standard symbol length
        {128, 5000},    // Large slice
        {0, static_cast<int64_t>(kTotal)} // Full vector
    };

    for (const auto& r : ranges) {
        int64_t start = r.first;
        int64_t end = r.second;

        double e_y_simd = 0.0;
        double e_out_simd = 0.0;
        compute_roi_energies(y.data(), out.data(), start, end, e_y_simd, e_out_simd);

        // Scalar double reference loop
        double e_y_ref = 0.0;
        double e_out_ref = 0.0;
        for (int64_t i = start; i < end; ++i) {
            e_y_ref += std::norm(y[i]);
            e_out_ref += std::norm(out[i]);
        }

        if (e_y_ref > 0.0) {
            const double rel_err_y = std::abs(e_y_simd - e_y_ref) / e_y_ref;
            BOOST_CHECK_LE(rel_err_y, 1e-6);
        } else {
            BOOST_CHECK_EQUAL(e_y_simd, 0.0);
        }

        if (e_out_ref > 0.0) {
            const double rel_err_out = std::abs(e_out_simd - e_out_ref) / e_out_ref;
            BOOST_CHECK_LE(rel_err_out, 1e-6);
        } else {
            BOOST_CHECK_EQUAL(e_out_simd, 0.0);
        }
    }

    // Test non-finite input handling
    y[50] = cf(std::numeric_limits<float>::quiet_NaN(), 0.f);
    double e_y_nan = 0.0;
    double e_out_nan = 0.0;
    compute_roi_energies(y.data(), out.data(), 0, 100, e_y_nan, e_out_nan);
    BOOST_CHECK(!std::isfinite(e_y_nan));
}

BOOST_AUTO_TEST_CASE(test_discrete_autocorr_prefilter_and_target_discrimination)
{
    TestPacket p_int(10, 32, 64);
    TestPacket p_target(9, 16, 16);
    SicUnifiedTemplates t = make_templates(p_int);

    constexpr size_t kWin = 160000;
    SicUnifiedConfig cfg;
    cfg.policy = SicPolicy::PreambleOnly;
    cfg.backend = SicEstimatorBackend::DiscreteCir;
    cfg.comm_code = 10;
    cfg.sync_reps = 64;
    cfg.samples_per_symbol = kSps;
    cfg.sample_rate = kFs;
    cfg.max_window_samples = kWin;
    cfg.max_packet_samples = p_int.wave.size() + 4096;
    cfg.full_window_search = true;
    cfg.coarse_stride = 8;
    cfg.autocorr_prefilter = true;
    cfg.autocorr_threshold = 0.20;

    SicUnifiedScratch scratch;
    std::string err;
    BOOST_REQUIRE_MESSAGE(prepare_sic_unified(cfg, t, scratch, &err), err);

    std::vector<cf> out(kWin);
    std::vector<cf> ihat(kWin);

    // 1. Pure noise wide ROI must short-circuit via "no_autocorr_preamble".
    {
        std::vector<cf> y_noise(kWin);
        uint32_t rng = 0x9E3779B9U;
        for (size_t i = 0; i < kWin; ++i) {
            rng = rng * 1664525U + 1013904223U;
            const float re = static_cast<float>(static_cast<int32_t>(rng >> 16) - 32768) * 1e-6f;
            rng = rng * 1664525U + 1013904223U;
            const float im = static_cast<float>(static_cast<int32_t>(rng >> 16) - 32768) * 1e-6f;
            y_noise[i] = cf(re, im);
        }
        SicUnifiedResult res;
        res.out = out.data();
        res.i_hat = ihat.data();
        BOOST_REQUIRE(sic_process_unified(y_noise.data(), kWin, t, cfg, scratch, res));
        BOOST_CHECK(!res.committed);
        BOOST_CHECK_EQUAL(std::string(res.reason ? res.reason : ""), "no_autocorr_preamble");
    }

    // 2. Target inside [target_preamble_lo, target_preamble_hi) + external code-10 interferer outside.
    {
        std::vector<cf> y(kWin, cf(0.0f, 0.0f));
        const size_t target_q = 12000;
        const size_t int_q = 65000;
        for (size_t i = 0; i < p_target.wave.size() && target_q + i < kWin; ++i) {
            y[target_q + i] += p_target.wave[i] * 0.25f;
        }
        for (size_t i = 0; i < p_int.wave.size() && int_q + i < kWin; ++i) {
            y[int_q + i] += p_int.wave[i];
        }

        SicUnifiedConfig cfg_ext = cfg;
        cfg_ext.target_preamble_lo = 10000;
        cfg_ext.target_preamble_hi = 32000;
        cfg_ext.roi_lo = 10000;
        cfg_ext.roi_hi = 150000;

        SicUnifiedResult res;
        res.out = out.data();
        res.i_hat = ihat.data();
        BOOST_REQUIRE(sic_process_unified(y.data(), kWin, t, cfg_ext, scratch, res));
        BOOST_CHECK_MESSAGE(res.committed, "ext reason=" << (res.reason ? res.reason : "null"));
        BOOST_CHECK_GT(res.suppression_db, 10.0);
        BOOST_CHECK_LE(std::abs(res.channel.q - static_cast<int64_t>(int_q)), 8);
    }

    // 2b. Clean target-only window (no external interferer): default behavior bypasses cleanly when outside_ivs is empty.
    {
        std::vector<cf> y(kWin, cf(0.0f, 0.0f));
        const size_t target_q = 12000;
        for (size_t i = 0; i < p_target.wave.size() && target_q + i < kWin; ++i) {
            y[target_q + i] += p_target.wave[i];
        }

        SicUnifiedConfig cfg_clean = cfg;
        cfg_clean.target_preamble_lo = 10000;
        cfg_clean.target_preamble_hi = 40000;
        cfg_clean.roi_lo = 10000;
        cfg_clean.roi_hi = 150000;

        SicUnifiedResult res;
        res.out = out.data();
        res.i_hat = ihat.data();
        BOOST_REQUIRE(sic_process_unified(y.data(), kWin, t, cfg_clean, scratch, res));
        BOOST_CHECK(!res.committed);
        BOOST_CHECK_EQUAL(std::string(res.reason ? res.reason : ""), "no_outside_autocorr_preamble");
    }

    // 3. Overlapping code-10 interferer inside [target_preamble_lo, target_preamble_hi) with explicit inside-search opt-in.
    {
        std::vector<cf> y(kWin, cf(0.0f, 0.0f));
        const size_t target_q = 12000;
        const size_t int_q = 12500;
        for (size_t i = 0; i < p_target.wave.size() && target_q + i < kWin; ++i) {
            y[target_q + i] += p_target.wave[i] * 0.2f;
        }
        for (size_t i = 0; i < p_int.wave.size() && int_q + i < kWin; ++i) {
            y[int_q + i] += p_int.wave[i];
        }

        SicUnifiedConfig cfg_in = cfg;
        cfg_in.autocorr_search_inside_target = true;
        cfg_in.target_preamble_lo = 10000;
        cfg_in.target_preamble_hi = 32000;
        cfg_in.roi_lo = 10000;
        cfg_in.roi_hi = 150000;

        SicUnifiedResult res;
        res.out = out.data();
        res.i_hat = ihat.data();
        BOOST_REQUIRE(sic_process_unified(y.data(), kWin, t, cfg_in, scratch, res));
        BOOST_CHECK_MESSAGE(res.committed, "in reason=" << (res.reason ? res.reason : "null"));
        BOOST_CHECK_GT(res.suppression_db, 10.0);
        BOOST_CHECK_LE(std::abs(res.channel.q - static_cast<int64_t>(int_q)), 8);
    }
}

// ---------------------------------------------------------------------------
// copy_input_on_bypass / null i_hat / template identity cache
// ---------------------------------------------------------------------------

namespace {

// Deterministic sentinel: "untouched" must be distinguishable from "copied y"
// and from any real reconstruction output.
cf bypass_sentinel(size_t i)
{
    const float k = static_cast<float>(i) * 1e-3f;
    return cf(1000.0f + k, -1000.0f - k);
}

void fill_sentinel(std::vector<cf>& v)
{
    for (size_t i = 0; i < v.size(); ++i) {
        v[i] = bypass_sentinel(i);
    }
}

bool all_sentinel(const std::vector<cf>& v)
{
    for (size_t i = 0; i < v.size(); ++i) {
        if (v[i] != bypass_sentinel(i)) {
            return false;
        }
    }
    return true;
}

struct BypassProbe {
    bool ok = false;          ///< sic_process_unified() return value
    bool committed = false;
    const char* reason = nullptr;
    bool out_equals_y = false; ///< out was overwritten with the input
    bool out_untouched = false;///< out still holds the sentinel
};

// Runs one window with an explicit copy_input_on_bypass setting and reports the
// observable out-buffer contract.
BypassProbe probe_bypass(const std::vector<cf>& y,
                         size_t n,
                         const SicUnifiedTemplates& t,
                         const SicUnifiedConfig& cfg,
                         SicUnifiedScratch& scratch,
                         bool copy_input_on_bypass)
{
    std::vector<cf> out(n);
    fill_sentinel(out);

    SicUnifiedConfig c = cfg;
    c.copy_input_on_bypass = copy_input_on_bypass;

    SicUnifiedResult res;
    res.out = out.data();
    res.i_hat = nullptr;

    BypassProbe p;
    p.ok = sic_process_unified(y.data(), n, t, c, scratch, res);
    p.committed = res.committed;
    p.reason = res.reason;
    p.out_equals_y = std::equal(out.begin(), out.end(), y.begin());
    p.out_untouched = all_sentinel(out);
    return p;
}

} // namespace

BOOST_AUTO_TEST_CASE(test_copy_input_on_bypass_flag_contract)
{
    TestPacket p(10, 32);
    SicUnifiedTemplates t = make_templates(p);

    const size_t q = 300;
    const size_t n = p.wave.size() + 1000;
    std::vector<cf> y(n, cf(0.f, 0.f));
    std::copy(p.wave.begin(), p.wave.end(), y.begin() + q);

    SicUnifiedConfig cfg;
    cfg.policy = SicPolicy::PreambleOnly;
    cfg.backend = SicEstimatorBackend::Ipatov;
    cfg.sparse_pulse_reconstruct = true;
    cfg.comm_code = 10;
    cfg.samples_per_symbol = kSps;
    cfg.sample_rate = kFs;
    cfg.roi_lo = q - 32;
    cfg.roi_hi = q + 32;
    cfg.sub_roi_lo = q;
    cfg.sub_roi_hi = q + 64 * kSps;
    cfg.max_window_samples = n;
    cfg.max_packet_samples = n;

    // The historical default must stay true: every pre-existing caller and test
    // depends on out == y on every non-committed exit.
    BOOST_CHECK(cfg.copy_input_on_bypass);

    SicUnifiedScratch scratch;
    BOOST_REQUIRE(prepare_sic_unified(cfg, t, scratch));

    // ---- committed path: both settings must produce bit-identical out ------
    std::vector<cf> out_copy(n);
    std::vector<cf> out_skip(n);
    fill_sentinel(out_skip);

    SicUnifiedResult res_copy;
    res_copy.out = out_copy.data();
    BOOST_REQUIRE(sic_process_unified(y.data(), n, t, cfg, scratch, res_copy));
    BOOST_CHECK(res_copy.committed);

    SicUnifiedConfig cfg_skip = cfg;
    cfg_skip.copy_input_on_bypass = false;
    // The flag is deliberately not part of the config identity, so the very
    // same scratch stays valid: no re-prepare, no config_or_template_mismatch.
    BOOST_CHECK(scratch.cache_matches(cfg_skip, t));

    SicUnifiedResult res_skip;
    res_skip.out = out_skip.data();
    BOOST_REQUIRE(sic_process_unified(y.data(), n, t, cfg_skip, scratch, res_skip));
    BOOST_CHECK(res_skip.committed);

    BOOST_CHECK_EQUAL(std::string(res_copy.reason), std::string(res_skip.reason));
    BOOST_CHECK_EQUAL(res_copy.sub_lo, res_skip.sub_lo);
    BOOST_CHECK_EQUAL(res_copy.sub_hi, res_skip.sub_hi);
    BOOST_CHECK_EQUAL(res_copy.suppression_db, res_skip.suppression_db);
    for (size_t i = 0; i < n; ++i) {
        BOOST_CHECK_EQUAL(out_copy[i], out_skip[i]);
    }
    // A committed transaction writes the WHOLE window, so nothing of the
    // sentinel survives and a caller may safely read out.
    BOOST_CHECK(!all_sentinel(out_skip));
    BOOST_REQUIRE(res_skip.sub_hi > res_skip.sub_lo);
    for (size_t i = 0; i < n; ++i) {
        const int64_t j = static_cast<int64_t>(i);
        if (j < res_skip.sub_lo || j >= res_skip.sub_hi) {
            BOOST_CHECK_EQUAL(out_skip[i], y[i]);
        }
    }

    // ---- every L0 bypass exit: copy=true writes y, copy=false writes nothing
    // Exit 2: estimation failed (all-zero window -> no autocorr preamble).
    // Exit 4: strict FullPacket abort (preamble-only window -> FCS cannot pass).
    // Exit 1: config_or_template_mismatch (returns false, not true).
    struct ExitCase {
        const char* label;
        std::vector<cf> window;
        SicUnifiedConfig cfg;
        bool expect_ok;
        const char* expect_reason;
        SicUnifiedScratch* scratch;
        // True for the top-level L0 bypass exits, which must leave `out`
        // byte-untouched when copy_input_on_bypass == false.  False for the
        // reconstruction-level fail-closed, where the frozen discrete-CIR apply
        // API (uwb_discrete_cir_sic.h) still performs its own y -> out copy;
        // that is harmless because committed stays false.
        bool expect_out_untouched;
    };

    std::vector<cf> y_clean(n, cf(0.f, 0.f));

    // Preamble-only window: SYNC + SFD present, no PHR/payload.
    const size_t n_prefix = 64 * kSps + 8 * kSps;
    BOOST_REQUIRE_GE(p.wave.size(), n_prefix);
    std::vector<cf> y_prefix(n, cf(0.f, 0.f));
    for (size_t i = 0; i < n_prefix; ++i) {
        y_prefix[q + i] = p.wave[i];
    }
    // SYNC-only window + a mismatched SFD profile: estimation succeeds (it only
    // reads SYNC) while the PreambleOnly SFD search cannot find its pattern.
    std::vector<cf> y_sync_only(n, cf(0.f, 0.f));
    for (size_t i = 0; i < 64 * kSps; ++i) {
        y_sync_only[q + i] = p.wave[i];
    }
    SicUnifiedConfig cfg_sfd = cfg;
    cfg_sfd.sfd_mode = "ieee";
    // sfd_mode is part of the config identity, so the mismatched profile needs
    // its own prepared scratch (the estimator itself only reads SYNC).
    SicUnifiedScratch scratch_sfd;
    BOOST_REQUIRE(prepare_sic_unified(cfg_sfd, t, scratch_sfd));

    SicUnifiedConfig cfg_full = cfg;
    cfg_full.policy = SicPolicy::FullPacket;

    SicUnifiedConfig cfg_mismatch = cfg;
    cfg_mismatch.comm_code = 9;

    // Reconstruction-level fail-closed: a valid channel but a sub-ROI that
    // cannot overlap the reconstructed support ("no_overlap").  This exercises
    // the separate fail_closed() lambda inside reconstruct_and_subtract_unified.
    SicUnifiedConfig cfg_nooverlap = cfg;
    cfg_nooverlap.sub_roi_lo = 1;
    cfg_nooverlap.sub_roi_hi = 2;

    std::vector<ExitCase> exits;
    exits.push_back({ "estimation_failed", y_clean, cfg, true, nullptr, &scratch, true });
    exits.push_back({ "sfd_detect_failed", y_sync_only, cfg_sfd, true, nullptr, &scratch_sfd, true });
    exits.push_back({ "fullpacket_strict_abort", y_prefix, cfg_full, true, nullptr, &scratch, true });
    exits.push_back({ "reconstruct_fail_closed_no_overlap", y, cfg_nooverlap, true,
                      "no_overlap", &scratch, false });
    exits.push_back({ "config_or_template_mismatch", y, cfg_mismatch, false,
                      "config_or_template_mismatch", &scratch, true });

    for (const ExitCase& e : exits) {
        const BypassProbe with_copy =
            probe_bypass(e.window, n, t, e.cfg, *e.scratch, true);
        const BypassProbe no_copy =
            probe_bypass(e.window, n, t, e.cfg, *e.scratch, false);

        BOOST_TEST_MESSAGE("bypass exit '" << e.label << "': reason="
                           << (with_copy.reason ? with_copy.reason : "null"));
        BOOST_CHECK_EQUAL(with_copy.ok, e.expect_ok);
        BOOST_CHECK_EQUAL(no_copy.ok, e.expect_ok);
        BOOST_CHECK(!with_copy.committed);
        BOOST_CHECK(!no_copy.committed);
        BOOST_CHECK_EQUAL(static_cast<uint8_t>(with_copy.committed ? 1 : 0),
                          static_cast<uint8_t>(no_copy.committed ? 1 : 0));
        BOOST_REQUIRE(with_copy.reason != nullptr);
        BOOST_REQUIRE(no_copy.reason != nullptr);
        BOOST_CHECK_EQUAL(std::string(with_copy.reason), std::string(no_copy.reason));
        if (e.expect_reason) {
            BOOST_CHECK_EQUAL(std::string(with_copy.reason), std::string(e.expect_reason));
        }
        // Default flag: out is a complete copy of the input.
        BOOST_CHECK_MESSAGE(with_copy.out_equals_y,
                            e.label << ": copy=true must write y into out");
        // Optimised flag: committed==false is the contract.  The top-level L0
        // exits additionally leave out byte-untouched.
        if (e.expect_out_untouched) {
            BOOST_CHECK_MESSAGE(no_copy.out_untouched,
                                e.label << ": copy=false must leave out untouched");
        }
    }
}

BOOST_AUTO_TEST_CASE(test_null_ihat_is_safe_on_bypass_and_committed_paths)
{
    TestPacket p(10, 32);
    SicUnifiedTemplates t = make_templates(p);

    const size_t q = 300;
    const size_t n = p.wave.size() + 1000;
    std::vector<cf> y(n, cf(0.f, 0.f));
    std::copy(p.wave.begin(), p.wave.end(), y.begin() + q);

    SicUnifiedConfig cfg;
    cfg.policy = SicPolicy::PreambleOnly;
    cfg.backend = SicEstimatorBackend::Ipatov;
    cfg.sparse_pulse_reconstruct = true;
    cfg.comm_code = 10;
    cfg.samples_per_symbol = kSps;
    cfg.sample_rate = kFs;
    cfg.roi_lo = q - 32;
    cfg.roi_hi = q + 32;
    cfg.sub_roi_lo = q;
    cfg.sub_roi_hi = q + 64 * kSps;
    cfg.max_window_samples = n;
    cfg.max_packet_samples = n;

    SicUnifiedScratch scratch;
    BOOST_REQUIRE(prepare_sic_unified(cfg, t, scratch));

    // Reference run with a real i_hat buffer (historical pipeline behaviour).
    std::vector<cf> out_ref(n);
    std::vector<cf> ihat_ref(n);
    SicUnifiedResult ref;
    ref.out = out_ref.data();
    ref.i_hat = ihat_ref.data();
    BOOST_REQUIRE(sic_process_unified(y.data(), n, t, cfg, scratch, ref));
    BOOST_REQUIRE(ref.committed);

    // Committed path with i_hat == nullptr and the bypass copy disabled: the
    // reconstruction must use internal scratch and still write the whole
    // window bit-identically.
    std::vector<cf> out_null(n);
    fill_sentinel(out_null);
    SicUnifiedConfig cfg_skip = cfg;
    cfg_skip.copy_input_on_bypass = false;
    SicUnifiedResult null_res;
    null_res.out = out_null.data();
    null_res.i_hat = nullptr;
    BOOST_REQUIRE(sic_process_unified(y.data(), n, t, cfg_skip, scratch, null_res));
    BOOST_CHECK(null_res.committed);
    BOOST_CHECK_EQUAL(std::string(null_res.reason), std::string(ref.reason));
    BOOST_CHECK_EQUAL(null_res.suppression_db, ref.suppression_db);
    BOOST_CHECK(!all_sentinel(out_null));
    for (size_t i = 0; i < n; ++i) {
        BOOST_CHECK_EQUAL(out_null[i], out_ref[i]);
    }

    // Bypass path with i_hat == nullptr: both flag settings must stay safe.
    std::vector<cf> y_clean(n, cf(0.f, 0.f));
    const BypassProbe with_copy = probe_bypass(y_clean, n, t, cfg, scratch, true);
    BOOST_CHECK(with_copy.ok);
    BOOST_CHECK(!with_copy.committed);
    BOOST_CHECK(with_copy.out_equals_y);

    const BypassProbe no_copy = probe_bypass(y_clean, n, t, cfg, scratch, false);
    BOOST_CHECK(no_copy.ok);
    BOOST_CHECK(!no_copy.committed);
    BOOST_CHECK(no_copy.out_untouched);
}

BOOST_AUTO_TEST_CASE(test_template_identity_cache_and_invalidation)
{
    TestPacket p(10, 32);
    SicUnifiedTemplates t = make_templates(p);

    SicUnifiedConfig cfg;
    cfg.policy = SicPolicy::PreambleOnly;
    cfg.backend = SicEstimatorBackend::Ipatov;
    cfg.comm_code = 10;
    cfg.samples_per_symbol = kSps;
    cfg.sample_rate = kFs;
    cfg.max_window_samples = 4096;
    cfg.max_packet_samples = 4096;

    SicUnifiedScratch scratch;
    BOOST_REQUIRE(prepare_sic_unified(cfg, t, scratch));
    BOOST_CHECK(scratch.template_identity.valid);
    BOOST_CHECK(scratch.cache_matches(cfg, t));
    BOOST_CHECK(scratch.cache_matches(cfg, t, true)); // force path agrees

    // Steady state: the digest is reused, not recomputed.
    const uint64_t digest = scratch.template_identity.digest;
    BOOST_CHECK_EQUAL(digest, scratch.template_hash);
    BOOST_CHECK(scratch.cache_matches(cfg, t));
    BOOST_CHECK_EQUAL(scratch.template_identity.digest, digest);

    // A different SYNC buffer holding identical content is accepted.
    std::vector<cf> sync_copy(p.sync.begin(), p.sync.end());
    SicUnifiedTemplates t_copy = t;
    t_copy.sync = sync_copy.data();
    BOOST_CHECK(scratch.cache_matches(cfg, t_copy));
    BOOST_CHECK_EQUAL(scratch.template_identity.digest, digest);

    // A different SYNC length is rejected.
    SicUnifiedTemplates t_len = t_copy;
    t_len.sync_len = sync_copy.size() - 8;
    BOOST_CHECK(!scratch.cache_matches(cfg, t_len));

    // DOCUMENTED ALIASING HOLE: same pointer + same length + mutated content is
    // invisible to the cached tuple.  This is the residual risk the header
    // documents; the escape hatch below is what a mutating caller must use.
    std::vector<cf> sync_mut(p.sync.begin(), p.sync.end());
    SicUnifiedTemplates t_mut = t;
    t_mut.sync = sync_mut.data();
    BOOST_CHECK(scratch.cache_matches(cfg, t_mut)); // seeds the tuple
    sync_mut[17] *= cf(0.5f, -0.25f);
    BOOST_CHECK_MESSAGE(
        scratch.cache_matches(cfg, t_mut),
        "cached tuple intentionally cannot see in-place content mutation");

    // invalidate_template_identity() forces a recompute and detects it.
    scratch.invalidate_template_identity();
    BOOST_CHECK(!scratch.cache_matches(cfg, t_mut));

    // force_full_hash detects it without touching the epoch.
    sync_mut[17] = p.sync[17];
    BOOST_CHECK(scratch.cache_matches(cfg, t_mut, true)); // re-seeds restored content
    sync_mut[17] *= cf(0.5f, -0.25f);
    BOOST_CHECK(!scratch.cache_matches(cfg, t_mut, true));

    // The config hash is never weakened.
    SicUnifiedConfig cfg9 = cfg;
    cfg9.comm_code = 9;
    BOOST_CHECK(!scratch.cache_matches(cfg9, t));
    BOOST_CHECK(!scratch.cache_matches(cfg9, t, true));

    // invalidate_cache() bumps the epoch and forces a full refresh.
    const uint64_t epoch_before = scratch.template_epoch;
    scratch.invalidate_cache();
    BOOST_CHECK_GT(scratch.template_epoch, epoch_before);
    BOOST_CHECK(!scratch.template_identity.valid);
    BOOST_CHECK(!scratch.cache_matches(cfg, t));
}

BOOST_AUTO_TEST_SUITE_END()
