/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * QA for the UwbRadarCirEstimator message block (Radar Step 7).
 *
 * Canonical numerics come from testdata/uwb_radar (MATLAB
 * export_uwb_radar_golden.m).  The block output must be bit-exact against
 * a direct radar_cir_one call with the same predicted SFD, failed stages
 * must publish an empty CIR, invalid PDUs must never be enqueued, queue
 * overflow must drop with counters, ordering must be preserved, and
 * stop/restart/destructor must be deadlock free.
 */

#include <boost/test/unit_test.hpp>

#include <gnuradio/blocks/message_debug.h>
#include <gnuradio/gr_complex.h>
#include <gnuradio/top_block.h>
#include <gnuradio/uwb/uwb_phy_profile.h>
#include <gnuradio/uwb/uwb_radar_cir_core.h>
#include <gnuradio/uwb/uwb_radar_cir_estimator_block.h>
#include <pmt/pmt.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <limits>
#include <string>
#include <thread>
#include <vector>

using gr::uwb::UwbRadarCirEstimator;
using gr::uwb::radar::radar_cir_one;
using gr::uwb::radar::prepare_radar_cir_core;
using gr::uwb::radar::RadarCirConfig;
using gr::uwb::radar::RadarCirCoreScratch;
using gr::uwb::radar::RadarCirResult;

#ifndef UWB_TESTDATA_DIR
#define UWB_TESTDATA_DIR "../../../testdata"
#endif

namespace {

constexpr size_t kSps = gr::uwb::demod::kQm35SamplesPerSymbol;
constexpr size_t kCirPre = 16;
constexpr size_t kCirPost = 100;
constexpr int64_t kPreGuard = 1997; // golden rx_clean sync origin
constexpr double kWorkRate = 998.4e6;

bool
load_cf32(const std::string& path, std::vector<gr_complex>& out)
{
    std::ifstream f(path, std::ios::binary);
    if (!f)
        return false;
    f.seekg(0, std::ios::end);
    const std::streamoff bytes = f.tellg();
    f.seekg(0, std::ios::beg);
    if (bytes <= 0 || bytes % 8 != 0)
        return false;
    out.resize(static_cast<size_t>(bytes) / 8);
    f.read(reinterpret_cast<char*>(out.data()), bytes);
    return f.good() || f.eof();
}

std::string
testdata_path(const std::string& rel)
{
    return std::string(UWB_TESTDATA_DIR) + "/" + rel;
}

std::string
load_canonical_sfd_truth(int64_t& sfd_truth)
{
    std::ifstream meta_f(testdata_path("uwb_radar/metadata.json"));
    BOOST_REQUIRE_MESSAGE(meta_f.good(),
                          "testdata/uwb_radar/metadata.json missing");
    std::ostringstream oss;
    oss << meta_f.rdbuf();
    const std::string json = oss.str();
    // Nested: metadata.rx_clean_998p4.sfd_start (67021 for the 64-SYNC
    // canonical golden; a bare top-level search would hit other sections).
    const std::string section = "\"rx_clean_998p4\"";
    auto sec = json.find(section);
    BOOST_REQUIRE(sec != std::string::npos);
    const std::string pat = "\"sfd_start\"";
    auto pos = json.find(pat, sec);
    BOOST_REQUIRE(pos != std::string::npos);
    pos = json.find(':', pos + pat.size());
    BOOST_REQUIRE(pos != std::string::npos);
    const double v = std::stod(json.substr(pos + 1));
    sfd_truth = static_cast<int64_t>(std::llround(v));
    return json;
}

// Sync pulse template = first 1016 samples of the 998.4 TX packet golden,
// written to a temp file so the block loads it like production does.
std::string
write_template_file(const std::vector<gr_complex>& tx)
{
    BOOST_REQUIRE_GE(tx.size(), kSps);
    const std::string dir =
        (std::filesystem::temp_directory_path() /
         "uwb_qa_radar_cir_estimator")
            .string();
    std::filesystem::create_directories(dir);
    const std::string path = dir + "/sync_template.cf32";
    std::ofstream f(path, std::ios::binary | std::ios::trunc);
    BOOST_REQUIRE_MESSAGE(f.good(), "cannot write " + path);
    f.write(reinterpret_cast<const char*>(tx.data()),
            static_cast<std::streamsize>(kSps * sizeof(gr_complex)));
    BOOST_REQUIRE(f.good());
    return path;
}

pmt::pmt_t
make_meta(uint64_t pulse_id,
          int64_t pre_guard,
          bool with_rate = true,
          bool with_cal = true,
          bool with_profile = true,
          bool with_pre_guard = true)
{
    pmt::pmt_t meta = pmt::make_dict();
    meta = pmt::dict_add(meta, pmt::mp("pulse_id"),
                         pmt::from_uint64(pulse_id));
    meta = pmt::dict_add(meta, pmt::mp("schedule_index"),
                         pmt::from_uint64(pulse_id));
    meta = pmt::dict_add(meta, pmt::mp("freq_hz"),
                         pmt::from_double(6.490091e9));
    meta = pmt::dict_add(meta, pmt::mp("freq_offset_hz"),
                         pmt::from_double(491000.0));
    if (with_rate)
        meta = pmt::dict_add(meta, pmt::mp("sample_rate"),
                             pmt::from_double(kWorkRate));
    if (with_pre_guard)
        meta = pmt::dict_add(meta, pmt::mp("pre_guard_samples"),
                             pmt::from_long(pre_guard));
    if (with_cal)
        meta = pmt::dict_add(meta, pmt::mp("calibration_delay_native_samples"),
                             pmt::from_double(0.0));
    if (with_profile) {
        meta = pmt::dict_add(meta, pmt::mp("sync_repetitions"),
                             pmt::from_long(64));
        meta = pmt::dict_add(meta, pmt::mp("sfd_mode"), pmt::mp("4z2"));
        meta = pmt::dict_add(meta, pmt::mp("code_index"), pmt::from_long(9));
    }
    meta = pmt::dict_add(meta, pmt::mp("tx_time_full"), pmt::from_long(0));
    meta = pmt::dict_add(meta, pmt::mp("tx_time_frac"),
                         pmt::from_double(0.05));
    meta = pmt::dict_add(meta, pmt::mp("rx_time_full"), pmt::from_long(0));
    meta = pmt::dict_add(meta, pmt::mp("rx_time_frac"),
                         pmt::from_double(0.048));
    meta = pmt::dict_add(meta, pmt::mp("num_delay_samps"), pmt::from_long(0));
    meta = pmt::dict_add(meta, pmt::mp("calibration_id"), pmt::mp("qa-cal"));
    meta = pmt::dict_add(meta, pmt::mp("source"), pmt::mp("loopback"));
    meta = pmt::dict_add(meta, pmt::mp("jam_delay_native"),
                         pmt::from_long(-4));
    meta = pmt::dict_add(meta, pmt::mp("jam_delay_us"),
                         pmt::from_double(0.005));
    meta = pmt::dict_add(meta, pmt::mp("jam_delay_mode"), pmt::mp("uniform"));
    meta = pmt::dict_add(meta, pmt::mp("jam_delay_seed"),
                         pmt::from_uint64(7));
    meta = pmt::dict_add(meta, pmt::mp("jam_freq_plan_hz"),
                         pmt::from_double(6.4896e9));
    meta = pmt::dict_add(meta, pmt::mp("jam_freq_actual_hz"),
                         pmt::from_double(6.4896e9));
    meta = pmt::dict_add(meta, pmt::mp("jam_freq_offset_hz"),
                         pmt::from_double(0.0));
    meta = pmt::dict_add(meta, pmt::mp("jam_retune_seq"),
                         pmt::from_uint64(1));
    meta = pmt::dict_add(meta, pmt::mp("sense_offset_native"),
                         pmt::from_uint64(4));
    meta = pmt::dict_add(meta, pmt::mp("tx_fragment_count"),
                         pmt::from_uint64(1));
    meta = pmt::dict_add(meta, pmt::mp("sample_format"), pmt::mp("fc32"));
    meta = pmt::dict_add(meta, pmt::mp("uhd_error"), pmt::mp("none"));
    return meta;
}

pmt::pmt_t
make_pdu(pmt::pmt_t meta, const std::vector<gr_complex>& iq)
{
    return pmt::cons(meta, pmt::init_c32vector(iq.size(), iq.data()));
}

bool
wait_frames(const gr::blocks::message_debug::sptr& dbg,
            size_t want,
            long timeout_ms = 30000)
{
    const auto t0 = std::chrono::steady_clock::now();
    while (dbg->num_messages() < want) {
        const auto ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - t0)
                .count();
        if (ms > timeout_ms)
            return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    return true;
}

template <typename Pred>
bool
wait_until(Pred&& pred, long timeout_ms = 30000)
{
    const auto t0 = std::chrono::steady_clock::now();
    while (!pred()) {
        const auto ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - t0)
                .count();
        if (ms > timeout_ms)
            return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    return true;
}

bool
wait_drained(const UwbRadarCirEstimator::sptr& est, long timeout_ms = 30000)
{
    return wait_until([&] { return est->drained(); }, timeout_ms);
}

bool
status_seen(const gr::blocks::message_debug::sptr& dbg,
            const std::string& event)
{
    for (size_t i = 0; i < dbg->num_messages(); ++i) {
        pmt::pmt_t st = dbg->get_message(i);
        if (pmt::is_dict(st) &&
            pmt::eqv(pmt::dict_ref(st, pmt::mp("event"), pmt::PMT_NIL),
                     pmt::mp(event)))
            return true;
    }
    return false;
}

RadarCirConfig
direct_cfg()
{
    RadarCirConfig cfg;
    cfg.sync_repetitions = 64;
    cfg.samples_per_symbol = kSps;
    cfg.sfd_mode = "4z2";
    cfg.sfd_search_margin = 64;
    cfg.sync_refine_margin = 8;
    cfg.sfd_threshold = 0.3f;
    cfg.sync_refine_threshold = 0.3f;
    cfg.cir_pre = kCirPre;
    cfg.cir_post = kCirPost;
    cfg.cir_skip_initial = 10;
    cfg.cir_repetitions = 54;
    return cfg;
}

std::string
meta_str(pmt::pmt_t meta, const char* key)
{
    pmt::pmt_t v = pmt::dict_ref(meta, pmt::mp(key), pmt::PMT_NIL);
    if (pmt::is_symbol(v))
        return pmt::symbol_to_string(v);
    return {};
}

struct FrameCheck {
    std::string status;
    size_t tap_count = 0;
};

FrameCheck
check_common(pmt::pmt_t msg, uint64_t pulse_id)
{
    BOOST_REQUIRE(pmt::is_pair(msg));
    pmt::pmt_t meta = pmt::car(msg);
    BOOST_REQUIRE(pmt::is_dict(meta));
    BOOST_CHECK_EQUAL(pmt::to_double(pmt::dict_ref(
                          meta, pmt::mp("freq_hz"), pmt::from_double(0.0))),
                      6.490091e9);
    BOOST_CHECK_EQUAL(pmt::to_double(pmt::dict_ref(
                          meta, pmt::mp("freq_offset_hz"),
                          pmt::from_double(-1.0))),
                      491000.0);
    pmt::pmt_t data = pmt::cdr(msg);
    BOOST_REQUIRE(pmt::is_c32vector(data));

    BOOST_CHECK_EQUAL(pmt::to_uint64(pmt::dict_ref(
                          meta, pmt::mp("pulse_id"), pmt::from_uint64(0))),
                      pulse_id);
    BOOST_CHECK_EQUAL(pmt::to_uint64(pmt::dict_ref(
                          meta, pmt::mp("schedule_index"),
                          pmt::from_uint64(0))),
                      pulse_id);
    // Lineage must survive: the estimator must not rebuild a bare dict.
    BOOST_CHECK_EQUAL(
        pmt::to_long(pmt::dict_ref(meta, pmt::mp("tx_time_full"),
                                   pmt::from_long(-1))),
        0);
    BOOST_CHECK_CLOSE(pmt::to_double(pmt::dict_ref(
                          meta, pmt::mp("tx_time_frac"),
                          pmt::from_double(-1))),
                      0.05,
                      1e-9);
    BOOST_CHECK_EQUAL(meta_str(meta, "calibration_id"), "qa-cal");
    BOOST_CHECK_EQUAL(meta_str(meta, "source"), "loopback");
    BOOST_CHECK_EQUAL(meta_str(meta, "sfd_mode"), "4z2");
    BOOST_CHECK_EQUAL(pmt::to_long(pmt::dict_ref(
                          meta, pmt::mp("jam_delay_native"),
                          pmt::from_long(0))),
                      -4);
    BOOST_CHECK_EQUAL(meta_str(meta, "jam_delay_mode"), "uniform");
    BOOST_CHECK_EQUAL(pmt::to_uint64(pmt::dict_ref(
                          meta, pmt::mp("jam_delay_seed"),
                          pmt::from_uint64(0))),
                      7u);
    BOOST_CHECK_EQUAL(pmt::to_uint64(pmt::dict_ref(
                          meta, pmt::mp("sense_offset_native"),
                          pmt::from_uint64(0))),
                      4u);
    BOOST_CHECK_EQUAL(pmt::to_uint64(pmt::dict_ref(
                          meta, pmt::mp("tx_fragment_count"),
                          pmt::from_uint64(0))),
                      1u);

    FrameCheck fc;
    fc.status = meta_str(meta, "status");
    fc.tap_count = pmt::to_uint64(pmt::dict_ref(
        meta, pmt::mp("tap_count"), pmt::from_uint64(0)));
    return fc;
}

} // namespace

// ---------------------------------------------------------------------------
// Valid PDU → ok CIR, bit-exact against direct radar_cir_one.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_estimator_ok_matches_core_direct)
{
    int64_t sfd_truth = -1;
    load_canonical_sfd_truth(sfd_truth);
    std::vector<gr_complex> rx, tx;
    BOOST_REQUIRE(
        load_cf32(testdata_path("uwb_radar/rx_clean_998p4.cf32"), rx));
    BOOST_REQUIRE(load_cf32(testdata_path("uwb_radar/tx_998p4.cf32"), tx));
    const std::string tmpl_path = write_template_file(tx);

    auto est = UwbRadarCirEstimator::make(tmpl_path, 64, "4z2", 9, kCirPre,
                                          kCirPost, 10, 0, 64, 8, 0.3f,
                                          0.3f, true, 16);
    auto dbg = gr::blocks::message_debug::make();
    auto dbg_status = gr::blocks::message_debug::make();
    auto tb = gr::make_top_block("qa_radar_estimator_ok");
    tb->msg_connect(est, "cir", dbg, "store");
    tb->msg_connect(est, "status", dbg_status, "store");

    // Direct core reference with the same prepared profile.
    RadarCirCoreScratch scratch;
    BOOST_REQUIRE(prepare_radar_cir_core(direct_cfg(), tx.data(), kSps,
                                         kCirPre, kCirPost, scratch));
    RadarCirResult ref;
    const int64_t predicted = kPreGuard + 64 * static_cast<int64_t>(kSps);
    BOOST_REQUIRE(radar_cir_one(rx.data(), rx.size(), predicted, direct_cfg(),
                                scratch, ref));
    std::vector<gr_complex> ref_raw(ref.tap_count), ref_norm(ref.tap_count);
    std::copy(scratch.cir.raw_taps.begin(),
              scratch.cir.raw_taps.begin() +
                  static_cast<std::ptrdiff_t>(ref.tap_count),
              ref_raw.begin());
    std::copy(scratch.cir.norm_taps.begin(),
              scratch.cir.norm_taps.begin() +
                  static_cast<std::ptrdiff_t>(ref.tap_count),
              ref_norm.begin());

    tb->start();
    est->_post(pmt::mp("rx"), make_pdu(make_meta(5, kPreGuard), rx));
    BOOST_REQUIRE(wait_frames(dbg, 1));
    BOOST_REQUIRE(wait_drained(est));
    tb->stop();
    tb->wait();

    BOOST_CHECK_EQUAL(est->pdus_received(), 1u);
    BOOST_CHECK_EQUAL(est->pdus_enqueued(), 1u);
    BOOST_CHECK_EQUAL(est->pdus_completed(), 1u);
    BOOST_CHECK_EQUAL(est->pdus_failed(), 0u);
    BOOST_CHECK_EQUAL(est->pdus_dropped(), 0u);
    BOOST_CHECK_EQUAL(est->invalid_inputs(), 0u);
    BOOST_CHECK_EQUAL(est->worker_exceptions(), 0u);
    BOOST_REQUIRE_EQUAL(dbg->num_messages(), 1u);

    pmt::pmt_t msg = dbg->get_message(0);
    FrameCheck fc = check_common(msg, 5);
    BOOST_CHECK_EQUAL(fc.status, "ok");
    BOOST_CHECK_EQUAL(fc.tap_count, kCirPre + kCirPost);

    pmt::pmt_t meta = pmt::car(msg);
    BOOST_CHECK_EQUAL(pmt::to_long(pmt::dict_ref(
                          meta, pmt::mp("sfd_start_sample"),
                          pmt::from_long(-1))),
                      sfd_truth);
    BOOST_CHECK_EQUAL(pmt::to_long(pmt::dict_ref(
                          meta, pmt::mp("predicted_sfd_start_sample"),
                          pmt::from_long(-1))),
                      predicted);
    BOOST_CHECK_EQUAL(pmt::to_long(pmt::dict_ref(
                          meta, pmt::mp("cir_origin_sample"),
                          pmt::from_long(-1))),
                      kPreGuard);
    BOOST_CHECK_EQUAL(pmt::to_long(pmt::dict_ref(
                          meta, pmt::mp("zero_delay_tap"),
                          pmt::from_long(-1))),
                      kCirPre);
    BOOST_CHECK(pmt::to_bool(pmt::dict_ref(meta, pmt::mp("sfd_ok"),
                                           pmt::PMT_F)));
    BOOST_CHECK(pmt::to_bool(pmt::dict_ref(meta, pmt::mp("timing_ok"),
                                           pmt::PMT_F)));
    BOOST_CHECK_EQUAL(pmt::to_uint64(pmt::dict_ref(
                          meta, pmt::mp("valid_repetitions"),
                          pmt::from_uint64(0))),
                      54u);
    BOOST_CHECK_EQUAL(pmt::to_uint64(pmt::dict_ref(
                          meta, pmt::mp("peak_tap"), pmt::from_uint64(0))),
                      ref.peak_tap);

    size_t n = 0;
    const gr_complex* taps = pmt::c32vector_elements(pmt::cdr(msg), n);
    BOOST_REQUIRE_EQUAL(n, ref.tap_count);
    BOOST_CHECK(std::memcmp(taps, ref_raw.data(),
                            n * sizeof(gr_complex)) == 0);

    // Normalized taps are attached when emit_normalized is set.
    pmt::pmt_t norm_v = pmt::dict_ref(meta, pmt::mp("normalized_taps"),
                                      pmt::PMT_NIL);
    BOOST_REQUIRE(pmt::is_c32vector(norm_v));
    size_t nn = 0;
    const gr_complex* norm = pmt::c32vector_elements(norm_v, nn);
    BOOST_REQUIRE_EQUAL(nn, ref.tap_count);
    BOOST_CHECK(std::memcmp(norm, ref_norm.data(),
                            nn * sizeof(gr_complex)) == 0);
}

BOOST_AUTO_TEST_CASE(test_estimator_emits_each_selected_repetition)
{
    std::vector<gr_complex> rx, tx;
    BOOST_REQUIRE(
        load_cf32(testdata_path("uwb_radar/rx_clean_998p4.cf32"), rx));
    BOOST_REQUIRE(load_cf32(testdata_path("uwb_radar/tx_998p4.cf32"), tx));
    const std::string tmpl_path = write_template_file(tx);

    auto est = UwbRadarCirEstimator::make(
        tmpl_path, 64, "4z2", 9, kCirPre, kCirPost, 10, 0, 64, 8,
        0.3f, 0.3f, true, 16, true, true);
    auto dbg = gr::blocks::message_debug::make();
    auto tb = gr::make_top_block("qa_radar_estimator_individual");
    tb->msg_connect(est, "cir", dbg, "store");
    tb->start();
    est->_post(pmt::mp("rx"), make_pdu(make_meta(9, kPreGuard), rx));
    BOOST_REQUIRE(wait_frames(dbg, 54));
    BOOST_REQUIRE(wait_drained(est));
    tb->stop();
    tb->wait();

    BOOST_CHECK(est->emit_individual_repetitions());
    BOOST_CHECK_EQUAL(est->pdus_completed(), 1u);
    BOOST_CHECK_EQUAL(est->pdus_published(), 54u);
    BOOST_REQUIRE_EQUAL(dbg->num_messages(), 54u);
    for (size_t ordinal = 0; ordinal < 54; ++ordinal) {
        const pmt::pmt_t msg = dbg->get_message(ordinal);
        const FrameCheck fc = check_common(msg, 9);
        BOOST_CHECK_EQUAL(fc.status, "ok");
        BOOST_CHECK_EQUAL(fc.tap_count, kCirPre + kCirPost);
        const pmt::pmt_t meta = pmt::car(msg);
        BOOST_CHECK_EQUAL(meta_str(meta, "cir_output"), "repetition");
        BOOST_CHECK_EQUAL(pmt::to_uint64(pmt::dict_ref(
                              meta, pmt::mp("repetition_index"),
                              pmt::from_uint64(0))),
                          ordinal + 10);
        BOOST_CHECK_EQUAL(pmt::to_uint64(pmt::dict_ref(
                              meta, pmt::mp("repetition_ordinal"),
                              pmt::from_uint64(99))),
                          ordinal);
        BOOST_CHECK_EQUAL(pmt::to_uint64(pmt::dict_ref(
                              meta, pmt::mp("repetition_count"),
                              pmt::from_uint64(0))),
                          54u);
        BOOST_CHECK_EQUAL(pmt::to_uint64(pmt::dict_ref(
                              meta, pmt::mp("valid_repetitions"),
                              pmt::from_uint64(0))),
                          1u);
    }
}

// both-mode: every SYNC repetition on "cir", and the legacy skip-10
// coherent average on "cir_avg".  The average is computed from the same
// repetition window with the per-repetition kernel
// (estimate_radar_cir_from_repetitions): same taps/equalities as
// radar_cir_one within the mean-vs-modified-double-accumulation
// reordering tolerance (Phase-1 stage-1 permission), and identical
// metadata/tap_count/valid_repetitions.  On clean signals the two differ
// only by float rounding; use the guard from
// qa_uwb_radar_cir_estimator.cc: individual_mean vs average relative L2
// < 2e-6 reference and check values match within that bound.
BOOST_AUTO_TEST_CASE(test_estimator_repetitions_and_average)
{
    std::vector<gr_complex> rx, tx;
    BOOST_REQUIRE(
        load_cf32(testdata_path("uwb_radar/rx_clean_998p4.cf32"), rx));
    BOOST_REQUIRE(load_cf32(testdata_path("uwb_radar/tx_998p4.cf32"), tx));
    const std::string tmpl_path = write_template_file(tx);

    RadarCirConfig cfg = direct_cfg();
    cfg.use_predicted_timing = true;
    cfg.cir_skip_initial = 10;
    cfg.cir_repetitions = 54;
    RadarCirCoreScratch scratch;
    BOOST_REQUIRE(prepare_radar_cir_core(cfg, tx.data(), kSps, kCirPre,
                                         kCirPost, scratch));
    RadarCirResult ref;
    const int64_t predicted = kPreGuard + 64 * static_cast<int64_t>(kSps);
    BOOST_REQUIRE(radar_cir_one(rx.data(), rx.size(), predicted, cfg, scratch,
                                ref));
    BOOST_REQUIRE(ref.status == gr::uwb::radar::RadarCirStatus::Ok);
    BOOST_REQUIRE_EQUAL(ref.tap_count, kCirPre + kCirPost);
    BOOST_REQUIRE_EQUAL(ref.valid_repetitions, 54u);

    auto est = UwbRadarCirEstimator::make(
        tmpl_path, 64, "4z2", 9, kCirPre, kCirPost,
        /*cir_skip_initial=*/0, /*cir_repetitions=*/0, 64, 8, 0.3f, 0.3f,
        true, 16, true, true, true, true, 10);
    BOOST_CHECK(est->emit_individual_repetitions());
    BOOST_CHECK(est->batch_individual_repetitions());
    BOOST_CHECK(est->emit_repetition_average());
    BOOST_CHECK_EQUAL(est->repetition_average_skip(), 10u);

    auto dbg = gr::blocks::message_debug::make();
    auto dbg_avg = gr::blocks::message_debug::make();
    auto tb = gr::make_top_block("qa_radar_estimator_both");
    tb->msg_connect(est, "cir", dbg, "store");
    tb->msg_connect(est, "cir_avg", dbg_avg, "store");
    tb->start();
    est->_post(pmt::mp("rx"), make_pdu(make_meta(9, kPreGuard), rx));
    BOOST_REQUIRE(wait_frames(dbg, 1));
    BOOST_REQUIRE(wait_frames(dbg_avg, 1));
    BOOST_REQUIRE(wait_drained(est));
    tb->stop();
    tb->wait();

    BOOST_CHECK_EQUAL(est->pdus_completed(), 1u);
    BOOST_CHECK_EQUAL(est->pdus_published(), 64u);
    BOOST_CHECK_EQUAL(est->averages_published(), 1u);
    BOOST_REQUIRE_EQUAL(dbg->num_messages(), 1u);
    BOOST_REQUIRE_EQUAL(dbg_avg->num_messages(), 1u);

    const pmt::pmt_t batch = pmt::car(dbg->get_message(0));
    BOOST_CHECK(pmt::to_bool(pmt::dict_ref(batch, pmt::mp("repetition_batch"),
                                           pmt::PMT_F)));
    BOOST_CHECK_EQUAL(pmt::to_uint64(pmt::dict_ref(
                          batch, pmt::mp("repetition_count"),
                          pmt::from_uint64(0))),
                      64u);
    BOOST_CHECK_EQUAL(pmt::to_uint64(pmt::dict_ref(
                          batch, pmt::mp("repetition_first"),
                          pmt::from_uint64(99))),
                      0u);

    const pmt::pmt_t avg_msg = dbg_avg->get_message(0);
    const FrameCheck fc = check_common(avg_msg, 9);
    BOOST_CHECK_EQUAL(fc.status, "ok");
    BOOST_CHECK_EQUAL(fc.tap_count, ref.tap_count);
    const pmt::pmt_t meta = pmt::car(avg_msg);
    BOOST_CHECK_EQUAL(pmt::to_double(pmt::dict_ref(
                          meta, pmt::mp("freq_hz"), pmt::from_double(0.0))),
                      6.490091e9);
    BOOST_CHECK_EQUAL(pmt::to_double(pmt::dict_ref(
                          meta, pmt::mp("freq_offset_hz"),
                          pmt::from_double(-1.0))),
                      491000.0);
    BOOST_CHECK_EQUAL(meta_str(meta, "cir_output"), "average");
    BOOST_CHECK(!pmt::dict_has_key(meta, pmt::mp("repetition_index")));
    BOOST_CHECK_EQUAL(pmt::to_uint64(pmt::dict_ref(
                          meta, pmt::mp("averaged_repetition_first"),
                          pmt::from_uint64(0))),
                      10u);
    BOOST_CHECK_EQUAL(pmt::to_uint64(pmt::dict_ref(
                          meta, pmt::mp("repetition_count"),
                          pmt::from_uint64(0))),
                      54u);
    BOOST_CHECK_EQUAL(pmt::to_uint64(pmt::dict_ref(
                          meta, pmt::mp("valid_repetitions"),
                          pmt::from_uint64(0))),
                      ref.valid_repetitions);
    BOOST_CHECK_EQUAL(pmt::to_uint64(pmt::dict_ref(
                          meta, pmt::mp("peak_tap"), pmt::from_uint64(0))),
                      ref.peak_tap);

    size_t n = 0;
    const gr_complex* taps = pmt::c32vector_elements(pmt::cdr(avg_msg), n);
    BOOST_REQUIRE_EQUAL(n, ref.tap_count);
    // Phase-1: the average now runs the per-repetition kernel over the same
    // skip-10 window.  Accumulation order differs from the legacy coherent
    // estimate_radar_cir, so require the same tolerance the core QA already
    // uses for individual-mean vs average (relative L2 < 2e-6) instead of
    // bit-exact memcmp.
    BOOST_TEST_MESSAGE("avg taps relative L2 (vs radar_cir_one): computed below");
    {
        double num = 0.0, den = 0.0;
        for (size_t i = 0; i < n; ++i) {
            const double dr = static_cast<double>(taps[i].real()) -
                              static_cast<double>(scratch.cir.raw_taps[i]
                                                      .real());
            const double di = static_cast<double>(taps[i].imag()) -
                              static_cast<double>(scratch.cir.raw_taps[i]
                                                      .imag());
            num += dr * dr + di * di;
            const double br = static_cast<double>(
                scratch.cir.raw_taps[i].real());
            const double bi = static_cast<double>(
                scratch.cir.raw_taps[i].imag());
            den += br * br + bi * bi;
        }
        const double rel = den > 0.0 ? std::sqrt(num) / std::sqrt(den)
                                     : (num > 0.0
                                            ? std::numeric_limits<
                                                  double>::infinity()
                                            : 0.0);
        BOOST_CHECK_MESSAGE(rel < 2e-6,
                            "avg vs radar_cir_one raw relative L2=" +
                                std::to_string(rel));
    }
    pmt::pmt_t norm_v = pmt::dict_ref(meta, pmt::mp("normalized_taps"),
                                      pmt::PMT_NIL);
    BOOST_REQUIRE(pmt::is_c32vector(norm_v));
    size_t nn = 0;
    const gr_complex* norm = pmt::c32vector_elements(norm_v, nn);
    BOOST_REQUIRE_EQUAL(nn, ref.tap_count);
    {
        double num = 0.0, den = 0.0;
        for (size_t i = 0; i < nn; ++i) {
            const double dr = static_cast<double>(norm[i].real()) -
                              static_cast<double>(scratch.cir.norm_taps[i]
                                                      .real());
            const double di = static_cast<double>(norm[i].imag()) -
                              static_cast<double>(scratch.cir.norm_taps[i]
                                                      .imag());
            num += dr * dr + di * di;
            const double br = static_cast<double>(
                scratch.cir.norm_taps[i].real());
            const double bi = static_cast<double>(
                scratch.cir.norm_taps[i].imag());
            den += br * br + bi * bi;
        }
        const double rel = den > 0.0 ? std::sqrt(num) / std::sqrt(den)
                                     : (num > 0.0
                                            ? std::numeric_limits<
                                                  double>::infinity()
                                            : 0.0);
        BOOST_CHECK_MESSAGE(rel < 2e-6,
                            "avg vs radar_cir_one norm relative L2=" +
                                std::to_string(rel));
    }
}

// ---------------------------------------------------------------------------
// Stage-3 (按消费者选择 normalized CIR): with emit_normalized=false the
// block must NOT create the normalized_taps c32vector PMT (neither the
// per-record copy nor the batch vector), while raw_l2_norm keeps being
// computed from the raw taps and published.  Raw taps stay bit-exact vs
// the direct radar_cir_one reference.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_estimator_normalized_off_omits_pmt_keeps_metrics)
{
    std::vector<gr_complex> rx, tx;
    BOOST_REQUIRE(
        load_cf32(testdata_path("uwb_radar/rx_clean_998p4.cf32"), rx));
    BOOST_REQUIRE(load_cf32(testdata_path("uwb_radar/tx_998p4.cf32"), tx));
    const std::string tmpl_path = write_template_file(tx);

    RadarCirCoreScratch scratch;
    BOOST_REQUIRE(prepare_radar_cir_core(direct_cfg(), tx.data(), kSps,
                                         kCirPre, kCirPost, scratch));
    RadarCirResult ref;
    const int64_t predicted = kPreGuard + 64 * static_cast<int64_t>(kSps);
    BOOST_REQUIRE(radar_cir_one(rx.data(), rx.size(), predicted, direct_cfg(),
                                scratch, ref));
    BOOST_REQUIRE(ref.status == gr::uwb::radar::RadarCirStatus::Ok);
    BOOST_REQUIRE_EQUAL(ref.tap_count, kCirPre + kCirPost);
    std::vector<gr_complex> ref_raw(ref.tap_count);
    std::copy(scratch.cir.raw_taps.begin(),
              scratch.cir.raw_taps.begin() +
                  static_cast<std::ptrdiff_t>(ref.tap_count),
              ref_raw.begin());

    auto est = UwbRadarCirEstimator::make(tmpl_path, 64, "4z2", 9, kCirPre,
                                          kCirPost, 10, 0, 64, 8, 0.3f,
                                          0.3f, /*emit_normalized=*/false,
                                          16);
    BOOST_CHECK(!est->emit_normalized());
    auto dbg = gr::blocks::message_debug::make();
    auto tb = gr::make_top_block("qa_radar_estimator_norm_off");
    tb->msg_connect(est, "cir", dbg, "store");
    tb->start();
    est->_post(pmt::mp("rx"), make_pdu(make_meta(6, kPreGuard), rx));
    BOOST_REQUIRE(wait_frames(dbg, 1));
    BOOST_REQUIRE(wait_drained(est));
    tb->stop();
    tb->wait();

    BOOST_CHECK_EQUAL(est->pdus_completed(), 1u);
    BOOST_REQUIRE_EQUAL(dbg->num_messages(), 1u);
    pmt::pmt_t msg = dbg->get_message(0);
    FrameCheck fc = check_common(msg, 6);
    BOOST_CHECK_EQUAL(fc.status, "ok");
    BOOST_CHECK_EQUAL(fc.tap_count, kCirPre + kCirPost);
    pmt::pmt_t meta = pmt::car(msg);

    // (a) The normalized PMT is not created at all.
    BOOST_CHECK(!pmt::dict_has_key(meta, pmt::mp("normalized_taps")));
    // (b) The raw L2 metric is still published.
    BOOST_CHECK_EQUAL(pmt::to_double(pmt::dict_ref(
                          meta, pmt::mp("raw_l2_norm"),
                          pmt::from_double(-1.0))),
                      static_cast<double>(ref.raw_l2_norm));
    // Raw taps are unchanged vs the original implementation.
    size_t n = 0;
    const gr_complex* taps = pmt::c32vector_elements(pmt::cdr(msg), n);
    BOOST_REQUIRE_EQUAL(n, ref.tap_count);
    BOOST_CHECK(std::memcmp(taps, ref_raw.data(),
                            n * sizeof(gr_complex)) == 0);
    BOOST_CHECK_EQUAL(pmt::to_uint64(pmt::dict_ref(
                          meta, pmt::mp("peak_tap"), pmt::from_uint64(0))),
                      ref.peak_tap);
}

// ---------------------------------------------------------------------------
// Stage-3: batched repetitions with emit_normalized=false vs true on the
// same input — the batch meta must omit normalized_taps when disabled and
// the raw batch columns/vectors must be tap-for-tap identical.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_estimator_normalized_off_batch_matches_on)
{
    std::vector<gr_complex> rx, tx;
    BOOST_REQUIRE(
        load_cf32(testdata_path("uwb_radar/rx_clean_998p4.cf32"), rx));
    BOOST_REQUIRE(load_cf32(testdata_path("uwb_radar/tx_998p4.cf32"), tx));
    const std::string tmpl_path = write_template_file(tx);

    auto est_on = UwbRadarCirEstimator::make(
        tmpl_path, 64, "4z2", 9, kCirPre, kCirPost,
        /*cir_skip_initial=*/0, /*cir_repetitions=*/0, 64, 8, 0.3f, 0.3f,
        /*emit_normalized=*/true, 16, true, true, true, true, 10);
    auto est_off = UwbRadarCirEstimator::make(
        tmpl_path, 64, "4z2", 9, kCirPre, kCirPost,
        /*cir_skip_initial=*/0, /*cir_repetitions=*/0, 64, 8, 0.3f, 0.3f,
        /*emit_normalized=*/false, 16, true, true, true, true, 10);
    auto dbg_on = gr::blocks::message_debug::make();
    auto dbg_off = gr::blocks::message_debug::make();
    auto tb = gr::make_top_block("qa_radar_estimator_norm_batch");
    tb->msg_connect(est_on, "cir", dbg_on, "store");
    tb->msg_connect(est_off, "cir", dbg_off, "store");
    tb->start();
    est_on->_post(pmt::mp("rx"), make_pdu(make_meta(11, kPreGuard), rx));
    est_off->_post(pmt::mp("rx"), make_pdu(make_meta(12, kPreGuard), rx));
    BOOST_REQUIRE(wait_frames(dbg_on, 1));
    BOOST_REQUIRE(wait_frames(dbg_off, 1));
    BOOST_REQUIRE(wait_drained(est_on));
    BOOST_REQUIRE(wait_drained(est_off));
    tb->stop();
    tb->wait();

    BOOST_REQUIRE_EQUAL(dbg_on->num_messages(), 1u);
    BOOST_REQUIRE_EQUAL(dbg_off->num_messages(), 1u);
    const pmt::pmt_t meta_on = pmt::car(dbg_on->get_message(0));
    const pmt::pmt_t meta_off = pmt::car(dbg_off->get_message(0));
    BOOST_CHECK(pmt::dict_has_key(meta_on, pmt::mp("normalized_taps")));
    BOOST_CHECK(!pmt::dict_has_key(meta_off, pmt::mp("normalized_taps")));

    // The legacy contract keeps the normalized batch vector, tap-exact.
    const size_t kTaps = kCirPre + kCirPost;
    const pmt::pmt_t norm_v = pmt::dict_ref(meta_on,
                                            pmt::mp("normalized_taps"),
                                            pmt::PMT_NIL);
    BOOST_REQUIRE(pmt::is_c32vector(norm_v));
    size_t nn = 0;
    const gr_complex* norm = pmt::c32vector_elements(norm_v, nn);
    BOOST_REQUIRE_EQUAL(nn, 64u * kTaps);
    for (size_t i = 0; i < nn; ++i)
        BOOST_CHECK(std::isfinite(norm[i].real()) &&
                    std::isfinite(norm[i].imag()));

    // Raw payload and every raw column are identical between on and off.
    size_t rn_on = 0, rn_off = 0;
    const gr_complex* raw_on = pmt::c32vector_elements(
        pmt::cdr(dbg_on->get_message(0)), rn_on);
    const gr_complex* raw_off = pmt::c32vector_elements(
        pmt::cdr(dbg_off->get_message(0)), rn_off);
    BOOST_REQUIRE_EQUAL(rn_on, 64u * kTaps);
    BOOST_REQUIRE_EQUAL(rn_off, 64u * kTaps);
    BOOST_CHECK(std::memcmp(raw_on, raw_off,
                            rn_on * sizeof(gr_complex)) == 0);

    const pmt::pmt_t l2_on = pmt::dict_ref(
        meta_on, pmt::mp("repetition_raw_l2_norm"), pmt::PMT_NIL);
    const pmt::pmt_t l2_off = pmt::dict_ref(
        meta_off, pmt::mp("repetition_raw_l2_norm"), pmt::PMT_NIL);
    BOOST_REQUIRE(pmt::is_f32vector(l2_on));
    BOOST_REQUIRE(pmt::is_f32vector(l2_off));
    size_t ln_on = 0, ln_off = 0;
    const float* l2p_on = pmt::f32vector_elements(l2_on, ln_on);
    const float* l2p_off = pmt::f32vector_elements(l2_off, ln_off);
    BOOST_REQUIRE_EQUAL(ln_on, 64u);
    BOOST_REQUIRE_EQUAL(ln_off, 64u);
    BOOST_CHECK(std::memcmp(l2p_on, l2p_off, ln_on * sizeof(float)) == 0);

    const pmt::pmt_t st_on = pmt::dict_ref(
        meta_on, pmt::mp("repetition_status_code"), pmt::PMT_NIL);
    const pmt::pmt_t st_off = pmt::dict_ref(
        meta_off, pmt::mp("repetition_status_code"), pmt::PMT_NIL);
    BOOST_REQUIRE(pmt::is_u64vector(st_on));
    BOOST_REQUIRE(pmt::is_u64vector(st_off));
    size_t sn_on = 0, sn_off = 0;
    const uint64_t* stp_on = pmt::u64vector_elements(st_on, sn_on);
    const uint64_t* stp_off = pmt::u64vector_elements(st_off, sn_off);
    BOOST_REQUIRE_EQUAL(sn_on, 64u);
    BOOST_REQUIRE_EQUAL(sn_off, 64u);
    BOOST_CHECK(std::memcmp(stp_on, stp_off, sn_on * sizeof(uint64_t)) == 0);
}

// ---------------------------------------------------------------------------
// Failed stages publish empty CIR frames only.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_estimator_failed_frames_no_taps)
{
    int64_t sfd_truth = -1;
    load_canonical_sfd_truth(sfd_truth);
    std::vector<gr_complex> rx, tx;
    BOOST_REQUIRE(
        load_cf32(testdata_path("uwb_radar/rx_clean_998p4.cf32"), rx));
    BOOST_REQUIRE(load_cf32(testdata_path("uwb_radar/tx_998p4.cf32"), tx));
    const std::string tmpl_path = write_template_file(tx);

    auto est = UwbRadarCirEstimator::make(tmpl_path, 64, "4z2", 9, kCirPre,
                                          kCirPost, 10, 0, 64, 8, 0.3f,
                                          0.3f, true, 16);
    auto dbg = gr::blocks::message_debug::make();
    auto tb = gr::make_top_block("qa_radar_estimator_failed");
    tb->msg_connect(est, "cir", dbg, "store");
    tb->start();

    // sfd_failed: silence — every search window has zero power, so the
    // metric is 0 everywhere and the SFD stage must fail.
    {
        std::vector<gr_complex> bad(rx.size(), gr_complex(0.0f, 0.0f));
        est->_post(pmt::mp("rx"), make_pdu(make_meta(1, kPreGuard), bad));
    }
    // timing_failed: zero the SYNC region.
    {
        std::vector<gr_complex> bad = rx;
        std::fill(bad.begin() + kPreGuard,
                  bad.begin() + sfd_truth, gr_complex(0.0f, 0.0f));
        est->_post(pmt::mp("rx"), make_pdu(make_meta(2, kPreGuard), bad));
    }
    // cir_failed: zero every averaging repetition window exactly (the
    // core averages k = skip..63; the k = 63 window reaches
    // origin + 63*sps - pre + wlen, clipping the first SFD samples), so
    // the coherent average is exactly zero → zero L2 norm → cir_failed,
    // while the refine gate (k = 0) and the SFD body stay intact.
    {
        std::vector<gr_complex> bad = rx;
        const int64_t first =
            kPreGuard + 10 * static_cast<int64_t>(kSps) -
            static_cast<int64_t>(kCirPre);
        const int64_t wlen = static_cast<int64_t>(kSps + kCirPre + kCirPost) -
                             1;
        const int64_t last =
            kPreGuard + 63 * static_cast<int64_t>(kSps) -
            static_cast<int64_t>(kCirPre) + wlen;
        std::fill(bad.begin() + first, bad.begin() + last,
                  gr_complex(0.0f, 0.0f));
        est->_post(pmt::mp("rx"), make_pdu(make_meta(3, kPreGuard), bad));
    }

    BOOST_REQUIRE(wait_frames(dbg, 3));
    BOOST_REQUIRE(wait_drained(est));
    tb->stop();
    tb->wait();

    BOOST_CHECK_EQUAL(est->pdus_completed(), 0u);
    BOOST_CHECK_EQUAL(est->pdus_failed(), 3u);

    pmt::pmt_t m1 = dbg->get_message(0);
    FrameCheck f1 = check_common(m1, 1);
    BOOST_CHECK_EQUAL(f1.status, "sfd_failed");
    BOOST_CHECK_EQUAL(f1.tap_count, 0u);
    BOOST_CHECK_EQUAL(pmt::length(pmt::cdr(m1)), 0u);
    BOOST_CHECK_EQUAL(pmt::to_long(pmt::dict_ref(
                          pmt::car(m1), pmt::mp("sfd_start_sample"),
                          pmt::from_long(0))),
                      -1);
    BOOST_CHECK_EQUAL(pmt::to_long(pmt::dict_ref(
                          pmt::car(m1), pmt::mp("preamble_start_sample"),
                          pmt::from_long(0))),
                      -1);
    BOOST_CHECK(!pmt::dict_has_key(pmt::car(m1),
                                   pmt::mp("normalized_taps")));
    BOOST_CHECK(!pmt::to_bool(pmt::dict_ref(pmt::car(m1), pmt::mp("sfd_ok"),
                                            pmt::PMT_T)));

    pmt::pmt_t m2 = dbg->get_message(1);
    FrameCheck f2 = check_common(m2, 2);
    BOOST_CHECK_EQUAL(f2.status, "timing_failed");
    BOOST_CHECK_EQUAL(f2.tap_count, 0u);
    BOOST_CHECK_EQUAL(pmt::to_long(pmt::dict_ref(
                          pmt::car(m2), pmt::mp("sfd_start_sample"),
                          pmt::from_long(0))),
                      sfd_truth);
    BOOST_CHECK_EQUAL(pmt::to_long(pmt::dict_ref(
                          pmt::car(m2), pmt::mp("preamble_start_sample"),
                          pmt::from_long(0))),
                      -1);
    BOOST_CHECK(!pmt::to_bool(pmt::dict_ref(pmt::car(m2),
                                            pmt::mp("timing_ok"),
                                            pmt::PMT_T)));

    pmt::pmt_t m3 = dbg->get_message(2);
    FrameCheck f3 = check_common(m3, 3);
    BOOST_CHECK_EQUAL(f3.status, "cir_failed");
    BOOST_CHECK_EQUAL(f3.tap_count, 0u);
    BOOST_CHECK_EQUAL(pmt::length(pmt::cdr(m3)), 0u);
    BOOST_CHECK(pmt::to_bool(pmt::dict_ref(pmt::car(m3), pmt::mp("sfd_ok"),
                                           pmt::PMT_F)));
    BOOST_CHECK(pmt::to_bool(pmt::dict_ref(pmt::car(m3),
                                           pmt::mp("timing_ok"),
                                           pmt::PMT_F)));
}

// ---------------------------------------------------------------------------
// Invalid PDUs: bad rate/profile/metadata must be rejected without enqueue.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_estimator_invalid_inputs)
{
    std::vector<gr_complex> rx, tx;
    BOOST_REQUIRE(
        load_cf32(testdata_path("uwb_radar/rx_clean_998p4.cf32"), rx));
    BOOST_REQUIRE(load_cf32(testdata_path("uwb_radar/tx_998p4.cf32"), tx));
    const std::string tmpl_path = write_template_file(tx);

    auto est = UwbRadarCirEstimator::make(tmpl_path, 64, "4z2", 9, kCirPre,
                                          kCirPost, 10, 0, 64, 8, 0.3f,
                                          0.3f, true, 16);
    auto dbg = gr::blocks::message_debug::make();
    auto dbg_status = gr::blocks::message_debug::make();
    auto tb = gr::make_top_block("qa_radar_estimator_invalid");
    tb->msg_connect(est, "cir", dbg, "store");
    tb->msg_connect(est, "status", dbg_status, "store");
    tb->start();

    // Plain vector (not a PDU pair).
    est->_post(pmt::mp("rx"), pmt::init_c32vector(rx.size(), rx.data()));
    // Wrong dtype: s16vector payload.
    {
        std::vector<int16_t> s16(2 * rx.size(), 0);
        est->_post(pmt::mp("rx"),
                   pmt::cons(make_meta(10, kPreGuard),
                             pmt::init_s16vector(s16.size(), s16.data())));
    }
    // Empty payload.
    {
        pmt::pmt_t meta = make_meta(11, kPreGuard);
        est->_post(pmt::mp("rx"),
                   pmt::cons(meta,
                             pmt::init_c32vector(
                                 0, static_cast<const gr_complex*>(nullptr))));
    }
    // Missing sample_rate.
    est->_post(pmt::mp("rx"),
               make_pdu(make_meta(12, kPreGuard, /*with_rate=*/false), rx));
    // Wrong rate (native).
    {
        pmt::pmt_t meta = make_meta(13, kPreGuard);
        meta = pmt::dict_add(meta, pmt::mp("sample_rate"),
                             pmt::from_double(737.28e6));
        est->_post(pmt::mp("rx"), make_pdu(meta, rx));
    }
    // Profile mismatches vs prepared (64/4z2/9).
    {
        pmt::pmt_t meta = make_meta(14, kPreGuard);
        meta = pmt::dict_add(meta, pmt::mp("sync_repetitions"),
                             pmt::from_long(32));
        est->_post(pmt::mp("rx"), make_pdu(meta, rx));
    }
    {
        pmt::pmt_t meta = make_meta(15, kPreGuard);
        meta = pmt::dict_add(meta, pmt::mp("sfd_mode"), pmt::mp("ieee"));
        est->_post(pmt::mp("rx"), make_pdu(meta, rx));
    }
    {
        pmt::pmt_t meta = make_meta(16, kPreGuard);
        meta = pmt::dict_add(meta, pmt::mp("code_index"), pmt::from_long(10));
        est->_post(pmt::mp("rx"), make_pdu(meta, rx));
    }
    // Missing pre_guard.
    est->_post(pmt::mp("rx"),
               make_pdu(make_meta(17, kPreGuard, true, true, true,
                                  /*with_pre_guard=*/false),
                        rx));
    // Missing calibration delay.
    est->_post(pmt::mp("rx"),
               make_pdu(make_meta(18, kPreGuard, true, /*with_cal=*/false),
                        rx));
    // Negative pre_guard.
    est->_post(pmt::mp("rx"), make_pdu(make_meta(19, -1), rx));
    // pre_guard that overflows the checked prediction arithmetic.
    est->_post(pmt::mp("rx"),
               make_pdu(make_meta(20, std::numeric_limits<int64_t>::max() -
                                          100),
                        rx));

    BOOST_REQUIRE(wait_until([&] {
        return est->invalid_inputs() == 12u && est->drained();
    }));
    tb->stop();
    tb->wait();

    BOOST_CHECK_EQUAL(est->pdus_received(), 9u);
    BOOST_CHECK_EQUAL(est->pdus_enqueued(), 0u);
    BOOST_CHECK_EQUAL(est->invalid_inputs(), 12u);
    BOOST_CHECK_EQUAL(dbg->num_messages(), 0u);
    BOOST_CHECK(status_seen(dbg_status, "bad_input_rate"));
    BOOST_CHECK(status_seen(dbg_status, "invalid_profile"));
    BOOST_CHECK(status_seen(dbg_status, "invalid_metadata"));
    BOOST_CHECK(status_seen(dbg_status, "invalid_input"));
}

// ---------------------------------------------------------------------------
// Extreme calibration delay values must be rejected *before* the checked
// rounding (llround on values near INT64_MAX is undefined behaviour).
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_estimator_extreme_calibration_delay)
{
    std::vector<gr_complex> rx, tx;
    BOOST_REQUIRE(
        load_cf32(testdata_path("uwb_radar/rx_clean_998p4.cf32"), rx));
    BOOST_REQUIRE(load_cf32(testdata_path("uwb_radar/tx_998p4.cf32"), tx));
    const std::string tmpl_path = write_template_file(tx);

    auto est = UwbRadarCirEstimator::make(tmpl_path, 64, "4z2", 9, kCirPre,
                                          kCirPost, 10, 0, 64, 8, 0.3f,
                                          0.3f, true, 16);
    auto dbg = gr::blocks::message_debug::make();
    auto dbg_status = gr::blocks::message_debug::make();
    auto tb = gr::make_top_block("qa_radar_estimator_extreme_cal");
    tb->msg_connect(est, "cir", dbg, "store");
    tb->msg_connect(est, "status", dbg_status, "store");
    tb->start();

    auto post_cal = [&](uint64_t pulse_id, pmt::pmt_t cal_value) {
        pmt::pmt_t meta = make_meta(pulse_id, kPreGuard);
        meta = pmt::dict_add(meta, pmt::mp("calibration_delay_work_samples"),
                             cal_value);
        est->_post(pmt::mp("rx"), make_pdu(meta, rx));
    };

    // double > INT64_MAX (finite, non-negative).
    post_cal(30, pmt::from_double(std::numeric_limits<double>::max()));
    // double ≈ 2^63 (INT64_MAX boundary as double).
    post_cal(31, pmt::from_double(9.2233720368547758e18));
    // uint64 above INT64_MAX: not representable as int64.
    post_cal(32, pmt::from_uint64(static_cast<uint64_t>(INT64_MAX) + 1));
    // Negative integer.
    post_cal(33, pmt::from_long(-5));
    // Negative real.
    post_cal(34, pmt::from_double(-1.5));

    BOOST_REQUIRE(wait_until([&] {
        return est->invalid_inputs() == 5u && est->drained();
    }));
    tb->stop();
    tb->wait();

    BOOST_CHECK_EQUAL(est->pdus_received(), 5u);
    BOOST_CHECK_EQUAL(est->pdus_enqueued(), 0u);
    BOOST_CHECK_EQUAL(est->invalid_inputs(), 5u);
    BOOST_CHECK_EQUAL(dbg->num_messages(), 0u);
    size_t n_invalid_metadata = 0;
    for (size_t i = 0; i < dbg_status->num_messages(); ++i) {
        if (pmt::eqv(pmt::dict_ref(dbg_status->get_message(i),
                                   pmt::mp("event"), pmt::PMT_NIL),
                     pmt::mp("invalid_metadata")))
            ++n_invalid_metadata;
    }
    BOOST_CHECK_EQUAL(n_invalid_metadata, 5u);
}

// ---------------------------------------------------------------------------
// Queue-full drop policy.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_estimator_queue_full)
{
    std::vector<gr_complex> rx, tx;
    BOOST_REQUIRE(
        load_cf32(testdata_path("uwb_radar/rx_clean_998p4.cf32"), rx));
    BOOST_REQUIRE(load_cf32(testdata_path("uwb_radar/tx_998p4.cf32"), tx));
    const std::string tmpl_path = write_template_file(tx);

    auto est = UwbRadarCirEstimator::make(tmpl_path, 64, "4z2", 9, kCirPre,
                                          kCirPost, 10, 0, 64, 8, 0.3f,
                                          0.3f, true, /*queue_capacity=*/2);
    auto dbg = gr::blocks::message_debug::make();
    auto dbg_status = gr::blocks::message_debug::make();
    auto tb = gr::make_top_block("qa_radar_estimator_queue_full");
    tb->msg_connect(est, "cir", dbg, "store");
    tb->msg_connect(est, "status", dbg_status, "store");
    tb->start();

    constexpr uint64_t kPost = 12;
    for (uint64_t i = 0; i < kPost; ++i)
        est->_post(pmt::mp("rx"), make_pdu(make_meta(i, kPreGuard), rx));

    BOOST_REQUIRE(wait_until([&] {
        return est->pdus_received() == kPost && est->drained();
    }));
    tb->stop();
    tb->wait();

    BOOST_CHECK_EQUAL(est->pdus_received(), kPost);
    BOOST_CHECK_EQUAL(est->pdus_enqueued() + est->pdus_dropped(), kPost);
    BOOST_CHECK_GT(est->pdus_dropped(), 0u);
    BOOST_CHECK_LE(est->queue_high_watermark(), size_t(2));
    BOOST_CHECK_EQUAL(dbg->num_messages(), est->pdus_enqueued());
    BOOST_CHECK(status_seen(dbg_status, "queue_full"));
    // Output order matches input order for the enqueued subset.
    uint64_t expect = 0;
    for (size_t i = 0; i < dbg->num_messages(); ++i) {
        pmt::pmt_t meta = pmt::car(dbg->get_message(i));
        const uint64_t id = pmt::to_uint64(pmt::dict_ref(
            meta, pmt::mp("pulse_id"), pmt::from_uint64(0)));
        while (id > expect)
            ++expect; // dropped pulse ids in between
        BOOST_CHECK_EQUAL(id, expect);
        ++expect;
    }
}

// ---------------------------------------------------------------------------
// Ordering: enqueued jobs are published in pulse-id order.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_estimator_output_order)
{
    std::vector<gr_complex> rx, tx;
    BOOST_REQUIRE(
        load_cf32(testdata_path("uwb_radar/rx_clean_998p4.cf32"), rx));
    BOOST_REQUIRE(load_cf32(testdata_path("uwb_radar/tx_998p4.cf32"), tx));
    const std::string tmpl_path = write_template_file(tx);

    auto est = UwbRadarCirEstimator::make(tmpl_path, 64, "4z2", 9, kCirPre,
                                          kCirPost, 10, 0, 64, 8, 0.3f,
                                          0.3f, true, 8);
    auto dbg = gr::blocks::message_debug::make();
    auto tb = gr::make_top_block("qa_radar_estimator_order");
    tb->msg_connect(est, "cir", dbg, "store");
    tb->start();

    constexpr uint64_t kPost = 10;
    constexpr uint64_t kChunk = 5; // < queue capacity → no drops
    for (uint64_t i = 0; i < kPost; ++i) {
        est->_post(pmt::mp("rx"),
                   make_pdu(make_meta(100 + i, kPreGuard), rx));
        if ((i + 1) % kChunk == 0)
            BOOST_REQUIRE(wait_until([&] { return est->drained(); }));
    }
    BOOST_REQUIRE(wait_until([&] {
        return dbg->num_messages() >= kPost && est->drained();
    }));
    tb->stop();
    tb->wait();

    BOOST_CHECK_EQUAL(dbg->num_messages(), kPost);
    for (size_t i = 0; i < kPost; ++i) {
        pmt::pmt_t meta = pmt::car(dbg->get_message(i));
        BOOST_CHECK_EQUAL(pmt::to_uint64(pmt::dict_ref(
                              meta, pmt::mp("pulse_id"),
                              pmt::from_uint64(0))),
                          100 + i);
    }
}

// ---------------------------------------------------------------------------
// stop / restart / destructor with pending jobs must drain, not deadlock.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_estimator_stop_restart_destructor)
{
    std::vector<gr_complex> rx, tx;
    BOOST_REQUIRE(
        load_cf32(testdata_path("uwb_radar/rx_clean_998p4.cf32"), rx));
    BOOST_REQUIRE(load_cf32(testdata_path("uwb_radar/tx_998p4.cf32"), tx));
    const std::string tmpl_path = write_template_file(tx);

    auto est = UwbRadarCirEstimator::make(tmpl_path, 64, "4z2", 9, kCirPre,
                                          kCirPost, 10, 0, 64, 8, 0.3f,
                                          0.3f, true, 8);
    auto dbg = gr::blocks::message_debug::make();
    auto tb = gr::make_top_block("qa_radar_estimator_lifecycle");
    tb->msg_connect(est, "cir", dbg, "store");

    // stop() drains pending jobs before the worker exits.
    tb->start();
    est->_post(pmt::mp("rx"), make_pdu(make_meta(1, kPreGuard), rx));
    est->_post(pmt::mp("rx"), make_pdu(make_meta(2, kPreGuard), rx));
    BOOST_REQUIRE(est->stop()); // drains + joins
    BOOST_CHECK_EQUAL(est->pdus_enqueued(), 2u);
    BOOST_CHECK_EQUAL(est->pdus_completed(), 2u);
    BOOST_REQUIRE(wait_frames(dbg, 2));
    BOOST_CHECK_EQUAL(dbg->num_messages(), 2u);

    // restart: the worker must accept jobs again.
    BOOST_REQUIRE(est->start());
    est->_post(pmt::mp("rx"), make_pdu(make_meta(3, kPreGuard), rx));
    BOOST_REQUIRE(wait_frames(dbg, 3));
    BOOST_REQUIRE(wait_drained(est));

    // Destructor with a pending job must drain + join without deadlock.
    // The top block is stopped here, so the drained frame is not asserted
    // on the message_debug side; only absence of deadlock is required.
    tb->stop();
    tb->wait();
    est->_post(pmt::mp("rx"), make_pdu(make_meta(4, kPreGuard), rx));
    est.reset(); // ~UwbRadarCirEstimator drains + joins
}

// ---------------------------------------------------------------------------
// Performance: 200 pulse/s budget → mean service time well below 5 ms.
// Records P95/P99 in the test log for the Step 7 report.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_estimator_service_time_200pps)
{
    std::vector<gr_complex> rx, tx;
    BOOST_REQUIRE(
        load_cf32(testdata_path("uwb_radar/rx_clean_998p4.cf32"), rx));
    BOOST_REQUIRE(load_cf32(testdata_path("uwb_radar/tx_998p4.cf32"), tx));
    const std::string tmpl_path = write_template_file(tx);

    auto est = UwbRadarCirEstimator::make(tmpl_path, 64, "4z2", 9, kCirPre,
                                          kCirPost, 10, 0, 64, 8, 0.3f,
                                          0.3f, true, 16);
    auto dbg = gr::blocks::message_debug::make();
    auto tb = gr::make_top_block("qa_radar_estimator_perf");
    tb->msg_connect(est, "cir", dbg, "store");

    // Warm-up (first run may page in template/scratch).
    tb->start();
    est->_post(pmt::mp("rx"), make_pdu(make_meta(0, kPreGuard), rx));
    BOOST_REQUIRE(wait_frames(dbg, 1));
    BOOST_REQUIRE(wait_drained(est));
    est->reset_stats();

    constexpr uint64_t kPost = 64;
    constexpr uint64_t kChunk = 8; // < queue capacity → no drops
    const auto wall0 = std::chrono::steady_clock::now();
    for (uint64_t i = 0; i < kPost; ++i) {
        est->_post(pmt::mp("rx"), make_pdu(make_meta(i, kPreGuard), rx));
        if ((i + 1) % kChunk == 0) {
            BOOST_REQUIRE(wait_until([&] { return est->drained(); }));
        }
    }
    BOOST_REQUIRE(wait_until([&] {
        return dbg->num_messages() >= 1 + kPost && est->drained();
    }, 120000));
    const auto wall1 = std::chrono::steady_clock::now();
    tb->stop();
    tb->wait();

    BOOST_CHECK_EQUAL(est->pdus_completed(), kPost);
    BOOST_CHECK_EQUAL(est->pdus_dropped(), 0u);
    const uint64_t mean = est->service_mean_us();
    const uint64_t p95 = est->service_p95_us();
    const uint64_t p99 = est->service_p99_us();
    const uint64_t mx = est->service_max_us();
    const uint64_t wall_ms = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(wall1 - wall0)
            .count());
    BOOST_TEST_MESSAGE("estimator service time: mean_us="
                       << mean << " p95_us=" << p95 << " p99_us=" << p99
                       << " max_us=" << mx << " wall_ms=" << wall_ms
                       << " n=" << kPost);
    // 200 pulse/s = 5 ms/pulse; a single worker must leave headroom.
    BOOST_CHECK_LT(mean, 5000u);
    BOOST_CHECK_LT(p99, 20000u);
}

// ---------------------------------------------------------------------------
// Unified 65/48 coordinate convention (Step 9): the SFD prediction must add
// the mapped window_start_sample so the native chain has no systematic
// FIR-head bias, and zero_delay_tap is cir_pre (the core's CIR axis origin
// already includes the calibration delay).
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_estimator_window_start_prediction)
{
    int64_t sfd_truth = -1;
    load_canonical_sfd_truth(sfd_truth);
    std::vector<gr_complex> rx, tx;
    BOOST_REQUIRE(
        load_cf32(testdata_path("uwb_radar/rx_clean_998p4.cf32"), rx));
    BOOST_REQUIRE(load_cf32(testdata_path("uwb_radar/tx_998p4.cf32"), tx));
    const std::string tmpl_path = write_template_file(tx);

    auto est = UwbRadarCirEstimator::make(tmpl_path, 64, "4z2", 9, kCirPre,
                                          kCirPost, 10, 0, 64, 8, 0.3f,
                                          0.3f, true, 16);
    auto dbg = gr::blocks::message_debug::make();
    auto dbg_status = gr::blocks::message_debug::make();
    auto tb = gr::make_top_block("qa_radar_estimator_window_start");
    tb->msg_connect(est, "cir", dbg, "store");
    tb->msg_connect(est, "status", dbg_status, "store");
    tb->start();

    // window_start 1000 + pre_guard 997 == the golden pre-guard 1997, so
    // the prediction must be identical to the no-window case.
    {
        pmt::pmt_t meta = make_meta(21, 997);
        meta = pmt::dict_add(meta, pmt::mp("window_start_sample"),
                             pmt::from_long(1000));
        est->_post(pmt::mp("rx"), make_pdu(meta, rx));
    }
    // Negative window start is invalid metadata.
    {
        pmt::pmt_t meta = make_meta(22, 997);
        meta = pmt::dict_add(meta, pmt::mp("window_start_sample"),
                             pmt::from_long(-1));
        est->_post(pmt::mp("rx"), make_pdu(meta, rx));
    }
    // Window start that overflows the checked prediction arithmetic.
    {
        pmt::pmt_t meta = make_meta(23, 997);
        meta = pmt::dict_add(meta, pmt::mp("window_start_sample"),
                             pmt::from_long(std::numeric_limits<int64_t>::max() -
                                            100));
        est->_post(pmt::mp("rx"), make_pdu(meta, rx));
    }

    BOOST_REQUIRE(wait_until([&] {
        return est->invalid_inputs() == 2u && est->drained();
    }));
    tb->stop();
    tb->wait();

    BOOST_CHECK_EQUAL(est->pdus_received(), 3u);
    BOOST_CHECK_EQUAL(est->pdus_enqueued(), 1u);
    BOOST_CHECK_EQUAL(est->pdus_completed(), 1u);
    BOOST_CHECK_EQUAL(est->invalid_inputs(), 2u);
    BOOST_REQUIRE_EQUAL(dbg->num_messages(), 1u);
    pmt::pmt_t meta = pmt::car(dbg->get_message(0));
    BOOST_CHECK_EQUAL(pmt::to_long(pmt::dict_ref(
                          meta, pmt::mp("predicted_sfd_start_sample"),
                          pmt::from_long(-1))),
                      kPreGuard + 64 * static_cast<int64_t>(kSps));
    BOOST_CHECK_EQUAL(pmt::to_long(pmt::dict_ref(
                          meta, pmt::mp("sfd_start_sample"),
                          pmt::from_long(-1))),
                      sfd_truth);
    BOOST_CHECK(status_seen(dbg_status, "invalid_metadata"));
}

BOOST_AUTO_TEST_CASE(test_estimator_zero_delay_tap_convention)
{
    int64_t sfd_truth = -1;
    load_canonical_sfd_truth(sfd_truth);
    std::vector<gr_complex> rx, tx;
    BOOST_REQUIRE(
        load_cf32(testdata_path("uwb_radar/rx_delay_int_998p4.cf32"), rx));
    BOOST_REQUIRE(load_cf32(testdata_path("uwb_radar/tx_998p4.cf32"), tx));
    const std::string tmpl_path = write_template_file(tx);
    BOOST_REQUIRE_EQUAL(sfd_truth, kPreGuard + 64 * static_cast<int64_t>(kSps));

    auto est = UwbRadarCirEstimator::make(tmpl_path, 64, "4z2", 9, kCirPre,
                                          kCirPost, 10, 0, 64, 8, 0.3f,
                                          0.3f, true, 16);
    auto dbg = gr::blocks::message_debug::make();
    auto tb = gr::make_top_block("qa_radar_estimator_zero_tap");
    tb->msg_connect(est, "cir", dbg, "store");
    tb->start();

    // rx_delay_int golden: channel delay 37 reported as the calibration
    // delay.  The predicted axis lands on the true origin, so the
    // calibrated leakage reads zero_delay_tap = cir_pre with the known +2
    // pulse-shape offset (metadata peak_tap_measured_delay_int = 55 on the
    // cal=0 grid == 18 + 37; the physical relationship peak-zero == 2).
    pmt::pmt_t meta = make_meta(30, kPreGuard);
    meta = pmt::dict_add(meta, pmt::mp("calibration_delay_native_samples"),
                         pmt::from_double(37.0));
    est->_post(pmt::mp("rx"), make_pdu(meta, rx));

    BOOST_REQUIRE(wait_frames(dbg, 1));
    BOOST_REQUIRE(wait_drained(est));
    tb->stop();
    tb->wait();

    BOOST_CHECK_EQUAL(est->pdus_completed(), 1u);
    pmt::pmt_t out = pmt::car(dbg->get_message(0));
    const int64_t predicted =
        kPreGuard + 37 + 64 * static_cast<int64_t>(kSps);
    BOOST_CHECK_EQUAL(pmt::to_long(pmt::dict_ref(
                          out, pmt::mp("predicted_sfd_start_sample"),
                          pmt::from_long(-1))),
                      predicted);
    BOOST_CHECK_LE(
        std::llabs(pmt::to_long(pmt::dict_ref(
                       out, pmt::mp("sfd_start_sample"), pmt::from_long(-1))) -
                   predicted),
        2);
    BOOST_CHECK_EQUAL(pmt::to_long(pmt::dict_ref(
                          out, pmt::mp("cir_origin_sample"),
                          pmt::from_long(-1))),
                      kPreGuard + 37);
    BOOST_CHECK_LE(
        std::llabs(pmt::to_long(pmt::dict_ref(
                       out, pmt::mp("preamble_start_sample"),
                       pmt::from_long(-1))) -
                   (kPreGuard + 37)),
        1);
    const int64_t zero = pmt::to_long(pmt::dict_ref(
        out, pmt::mp("zero_delay_tap"), pmt::from_long(-1)));
    const int64_t peak = static_cast<int64_t>(pmt::to_uint64(
        pmt::dict_ref(out, pmt::mp("peak_tap"), pmt::from_uint64(0))));
    BOOST_CHECK_EQUAL(zero, static_cast<int64_t>(kCirPre));
    BOOST_CHECK_LE(std::llabs(peak - 18), 1);
    BOOST_CHECK_LE(std::llabs((peak - zero) - 2), 1);
}
