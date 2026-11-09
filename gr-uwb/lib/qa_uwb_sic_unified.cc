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

    explicit TestPacket(size_t code = 10, size_t psdu_len = 32)
    {
        profile.code_index = code;
        profile.sync_repetitions = 64;
        profile.sfd_mode = "4z2";
        profile.peak_amplitude = 0.8f;

        psdu = mod::make_random_psdu(psdu_len - 2, static_cast<uint32_t>(1000 + code), true);
        const size_t cap = mod::packet_samples_998p4(64, 8, psdu.size(), false, &profile.pulse) + 512;
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

BOOST_AUTO_TEST_SUITE_END()
