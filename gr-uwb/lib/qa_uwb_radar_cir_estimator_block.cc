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
using gr::uwb::radar::radar_cir_average_range;
using gr::uwb::radar::radar_cir_coverage_ok;
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
        // R03 item 5: per-repetition output is a RAW, uncompensated diagnostic
        // and is named as such in both the field and the metadata.
        BOOST_CHECK_EQUAL(meta_str(meta, "cir_output"),
                          "repetition_raw_uncompensated");
        BOOST_CHECK(!pmt::to_bool(pmt::dict_ref(meta, pmt::mp("cfo_compensated"),
                                                pmt::PMT_T)));
        BOOST_CHECK_EQUAL(meta_str(meta, "cfo_source"), "none");
        BOOST_CHECK_EQUAL(pmt::to_uint64(pmt::dict_ref(
                              meta, pmt::mp("cfo_epoch"), pmt::from_uint64(99))),
                          0u);
        BOOST_CHECK_EQUAL(pmt::to_double(pmt::dict_ref(
                              meta, pmt::mp("final_cfo_hz"),
                              pmt::from_double(-1.0))),
                          0.0);
        // The pulse-level estimate is still visible, but only as a reference.
        BOOST_CHECK(pmt::dict_has_key(meta, pmt::mp("pulse_final_cfo_hz")));
        BOOST_CHECK(pmt::dict_has_key(meta, pmt::mp("pulse_cfo_epoch")));
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
// coherent average on "cir_avg".  The average must be bit-exact against
// radar_cir_one configured the way --cir-output average is.
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
    BOOST_CHECK(std::memcmp(taps, scratch.cir.raw_taps.data(),
                            n * sizeof(gr_complex)) == 0);
    pmt::pmt_t norm_v = pmt::dict_ref(meta, pmt::mp("normalized_taps"),
                                      pmt::PMT_NIL);
    BOOST_REQUIRE(pmt::is_c32vector(norm_v));
    size_t nn = 0;
    const gr_complex* norm = pmt::c32vector_elements(norm_v, nn);
    BOOST_REQUIRE_EQUAL(nn, ref.tap_count);
    BOOST_CHECK(std::memcmp(norm, scratch.cir.norm_taps.data(),
                            nn * sizeof(gr_complex)) == 0);
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
    // Detection source changed (workstream C): the shared preamble front-end
    // acquires SYNC first, so an all-zero window fails at timing.  The
    // pre-refactor radar searched the SFD first and reported sfd_failed.  The
    // fail-closed outcome (no taps, no coordinates, no normalized_taps) is
    // unchanged.
    BOOST_CHECK_EQUAL(f1.status, "timing_failed");
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
    // Detection source changed (workstream C): the shared front-end stops at
    // the failed timing stage and never runs the SFD search, so it publishes no
    // partial SFD coordinate (the pre-refactor radar searched the SFD first).
    BOOST_CHECK_EQUAL(pmt::to_long(pmt::dict_ref(
                          pmt::car(m2), pmt::mp("sfd_start_sample"),
                          pmt::from_long(0))),
                      -1);
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
    // Detection source changed (workstream C): zeroing repetitions 10..63 also
    // destroys the 40-SYNC tail the shared front-end uses for its final CFO
    // refit, so the front-end fails closed BEFORE the CIR kernel runs and
    // publishes no coordinates.  The status stays cir_failed (a CFO/coverage
    // failure has no CIR to report) and the frame still carries zero taps.
    BOOST_CHECK(!pmt::to_bool(pmt::dict_ref(pmt::car(m3), pmt::mp("sfd_ok"),
                                            pmt::PMT_T)));
    BOOST_CHECK(!pmt::to_bool(pmt::dict_ref(pmt::car(m3),
                                            pmt::mp("timing_ok"),
                                            pmt::PMT_T)));
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
    // The queue-full policy is about ENQUEUE RATE vs worker rate.  Building a
    // fresh 1.5 MB PMT c32vector for every message made the producer slower
    // than the worker once the sensing path got ~4x faster (R04), so the
    // capacity-2 queue never filled and the test became flaky.  Share one
    // immutable payload (PMT is refcounted) and vary only the metadata, which
    // is what a real burst producer does; the overload is then deterministic.
    const pmt::pmt_t shared_payload =
        pmt::init_c32vector(rx.size(), rx.data());
    for (uint64_t i = 0; i < kPost; ++i)
        est->_post(pmt::mp("rx"),
                   pmt::cons(make_meta(i, kPreGuard), shared_payload));

    BOOST_REQUIRE(wait_until([&] {
        return est->pdus_received() == kPost && est->drained();
    }));
    tb->stop();
    tb->wait();

    BOOST_CHECK_EQUAL(est->pdus_received(), kPost);
    BOOST_CHECK_EQUAL(est->pdus_enqueued() + est->pdus_dropped(), kPost);
    BOOST_CHECK_GT(est->pdus_dropped(), 0u);
    BOOST_CHECK_LE(est->queue_high_watermark(), size_t(2));
    // `drained()` covers the worker queue, not the message_debug block's own
    // queue: wait for the stored frames before comparing counts.
    BOOST_REQUIRE(wait_frames(dbg, est->pdus_enqueued()));
    BOOST_CHECK_EQUAL(dbg->num_messages(), est->pdus_enqueued());
    BOOST_REQUIRE(wait_until([&] {
        return status_seen(dbg_status, "queue_full");
    }));
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
    // Pre-build both payloads so the producer is not slower than the worker
    // (see the variable-window test for the R04 rationale).
    const pmt::pmt_t shared_payload =
        pmt::init_c32vector(rx.size(), rx.data());
    est->_post(pmt::mp("rx"),
               pmt::cons(make_meta(1, kPreGuard), shared_payload));
    est->_post(pmt::mp("rx"),
               pmt::cons(make_meta(2, kPreGuard), shared_payload));
    // `_post` is asynchronous: stop() must not run before the block's message
    // thread has handled both messages, otherwise the second is never enqueued
    // and the counters are racy (exposed once the worker got ~4x faster).
    BOOST_REQUIRE(wait_until([&] { return est->pdus_received() == 2u; }));
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
    // The `cir` message travels through a second block's queue, so `drained()`
    // does not imply message_debug has stored it yet.
    BOOST_REQUIRE(wait_frames(dbg, 1));
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

// ---------------------------------------------------------------------------
// R02 — worker scratch ownership with variable-length windows.
//
// The message handler must only validate, assemble the Job and enqueue; it must
// never reserve/resize/prepare the worker's scratch.  Because this block has no
// public maximum window length, growth is dynamic and happens EXCLUSIVELY in
// the worker at the start of a job, and it is counted:
//   scratch_growths() / scratch_growth_us() / scratch_capacity_samples().
//
// This test posts a backlog of windows whose lengths alternate and grow while
// the worker is busy, then compares every emitted frame against a serial
// reference (the same window fed to a FRESH block, one at a time), and covers
// drain, stop and recovery after a failure.  "Repeated fixed-length posts do
// not crash" is not the evidence here: the lengths differ, the queue backs up,
// and the outputs are compared field by field and tap by tap.
// ---------------------------------------------------------------------------
namespace {

std::vector<gr_complex>
window_of(const std::vector<gr_complex>& rx, size_t len)
{
    std::vector<gr_complex> w(len, gr_complex(0.0f, 0.0f));
    const size_t copy = std::min(len, rx.size());
    std::copy_n(rx.begin(), copy, w.begin());
    return w;
}

struct FrameSnapshot {
    std::string status;
    std::string status_reason;
    std::string cfo_source;
    std::string cir_output;
    size_t tap_count = 0;
    size_t peak_tap = 0;
    int64_t sfd_start = -2;
    int64_t cir_origin = -2;
    int64_t covered_lo = -2;
    int64_t covered_hi = -2;
    uint64_t cfo_epoch = 99;
    double final_cfo_hz = 0.0;
    double bootstrap_cfo_hz = 0.0;
    std::vector<gr_complex> taps;
};

FrameSnapshot
snapshot_frame(pmt::pmt_t msg)
{
    FrameSnapshot s;
    pmt::pmt_t meta = pmt::car(msg);
    s.status = meta_str(meta, "status");
    s.status_reason = meta_str(meta, "status_reason");
    s.cfo_source = meta_str(meta, "cfo_source");
    s.cir_output = meta_str(meta, "cir_output");
    s.tap_count = pmt::to_uint64(
        pmt::dict_ref(meta, pmt::mp("tap_count"), pmt::from_uint64(0)));
    s.peak_tap = pmt::to_uint64(
        pmt::dict_ref(meta, pmt::mp("peak_tap"), pmt::from_uint64(0)));
    s.sfd_start = pmt::to_long(
        pmt::dict_ref(meta, pmt::mp("sfd_start_sample"), pmt::from_long(-2)));
    s.cir_origin = pmt::to_long(
        pmt::dict_ref(meta, pmt::mp("cir_origin_sample"), pmt::from_long(-2)));
    s.covered_lo = pmt::to_long(
        pmt::dict_ref(meta, pmt::mp("covered_lo"), pmt::from_long(-2)));
    s.covered_hi = pmt::to_long(
        pmt::dict_ref(meta, pmt::mp("covered_hi"), pmt::from_long(-2)));
    s.cfo_epoch = pmt::to_uint64(
        pmt::dict_ref(meta, pmt::mp("cfo_epoch"), pmt::from_uint64(99)));
    s.final_cfo_hz = pmt::to_double(
        pmt::dict_ref(meta, pmt::mp("final_cfo_hz"), pmt::from_double(0.0)));
    s.bootstrap_cfo_hz = pmt::to_double(pmt::dict_ref(
        meta, pmt::mp("bootstrap_cfo_hz"), pmt::from_double(0.0)));
    size_t n = 0;
    const gr_complex* p = pmt::c32vector_elements(pmt::cdr(msg), n);
    if (n > 0 && p != nullptr)
        s.taps.assign(p, p + n);
    return s;
}

double
rel_l2_block(const std::vector<gr_complex>& a,
             const std::vector<gr_complex>& b)
{
    if (a.size() != b.size() || a.empty())
        return std::numeric_limits<double>::infinity();
    double num = 0.0;
    double den = 0.0;
    for (size_t i = 0; i < a.size(); ++i) {
        num += std::norm(a[i] - b[i]);
        den += std::norm(b[i]);
    }
    if (den == 0.0)
        return num == 0.0 ? 0.0 : std::numeric_limits<double>::infinity();
    return std::sqrt(num / den);
}

void
compare_frames(const FrameSnapshot& got,
               const FrameSnapshot& ref,
               const std::string& tag)
{
    BOOST_TEST_MESSAGE(tag << ": status=" << got.status
                           << " reason=" << got.status_reason
                           << " taps=" << got.tap_count
                           << " peak=" << got.peak_tap
                           << " cfo_source=" << got.cfo_source);
    BOOST_CHECK_MESSAGE(got.status == ref.status,
                        tag << " status " << got.status << " vs " << ref.status);
    BOOST_CHECK_MESSAGE(got.status_reason == ref.status_reason,
                        tag << " status_reason " << got.status_reason << " vs "
                            << ref.status_reason);
    BOOST_CHECK_MESSAGE(got.cfo_source == ref.cfo_source,
                        tag << " cfo_source " << got.cfo_source << " vs "
                            << ref.cfo_source);
    BOOST_CHECK_MESSAGE(got.tap_count == ref.tap_count,
                        tag << " tap_count " << got.tap_count << " vs "
                            << ref.tap_count);
    BOOST_CHECK_MESSAGE(got.peak_tap == ref.peak_tap,
                        tag << " peak_tap " << got.peak_tap << " vs "
                            << ref.peak_tap);
    BOOST_CHECK_MESSAGE(got.sfd_start == ref.sfd_start,
                        tag << " sfd_start " << got.sfd_start << " vs "
                            << ref.sfd_start);
    BOOST_CHECK_MESSAGE(got.cir_origin == ref.cir_origin,
                        tag << " cir_origin " << got.cir_origin << " vs "
                            << ref.cir_origin);
    BOOST_CHECK_MESSAGE(got.covered_lo == ref.covered_lo,
                        tag << " covered_lo " << got.covered_lo << " vs "
                            << ref.covered_lo);
    BOOST_CHECK_MESSAGE(got.covered_hi == ref.covered_hi,
                        tag << " covered_hi " << got.covered_hi << " vs "
                            << ref.covered_hi);
    BOOST_CHECK_MESSAGE(got.cfo_epoch == ref.cfo_epoch,
                        tag << " cfo_epoch " << got.cfo_epoch << " vs "
                            << ref.cfo_epoch);
    BOOST_REQUIRE_EQUAL(got.taps.size(), ref.taps.size());
    if (!got.taps.empty()) {
        BOOST_CHECK_MESSAGE(
            std::memcmp(got.taps.data(), ref.taps.data(),
                        got.taps.size() * sizeof(gr_complex)) == 0,
            tag << " taps differ");
    }
}

// One window through a FRESH block: the serial reference.
FrameSnapshot
run_single_window(const std::string& tmpl,
                  const std::vector<gr_complex>& w,
                  uint64_t pulse_id)
{
    auto est = UwbRadarCirEstimator::make(tmpl, 64, "4z2", 9, kCirPre,
                                          kCirPost, 10, 0, 64, 8, 0.3f, 0.3f,
                                          true, 4);
    auto dbg = gr::blocks::message_debug::make();
    auto tb = gr::make_top_block("qa_radar_estimator_serial_ref");
    tb->msg_connect(est, "cir", dbg, "store");
    tb->start();
    est->_post(pmt::mp("rx"), make_pdu(make_meta(pulse_id, kPreGuard), w));
    BOOST_REQUIRE(wait_frames(dbg, 1));
    BOOST_REQUIRE(wait_drained(est));
    tb->stop();
    tb->wait();
    BOOST_REQUIRE_EQUAL(dbg->num_messages(), 1u);
    return snapshot_frame(dbg->get_message(0));
}

} // namespace

BOOST_AUTO_TEST_CASE(test_estimator_variable_windows_worker_owns_scratch)
{
    std::vector<gr_complex> rx, tx;
    BOOST_REQUIRE(
        load_cf32(testdata_path("uwb_radar/rx_clean_998p4.cf32"), rx));
    BOOST_REQUIRE(load_cf32(testdata_path("uwb_radar/tx_998p4.cf32"), tx));
    const std::string tmpl_path = write_template_file(tx);

    // Alternating and growing.  Anything longer than the fixture is zero
    // padded, which is a legal window (the frame sits at the start).
    const size_t lens[] = { 78000, rx.size(), 86000, 260000, 94000, 300000 };
    constexpr size_t kW = sizeof(lens) / sizeof(lens[0]);

    std::vector<FrameSnapshot> ref;
    for (size_t i = 0; i < kW; ++i) {
        ref.push_back(run_single_window(tmpl_path, window_of(rx, lens[i]),
                                       500 + i));
    }

    auto est = UwbRadarCirEstimator::make(tmpl_path, 64, "4z2", 9, kCirPre,
                                          kCirPost, 10, 0, 64, 8, 0.3f, 0.3f,
                                          true, /*queue_capacity=*/8);
    auto dbg = gr::blocks::message_debug::make();
    auto tb = gr::make_top_block("qa_radar_estimator_variable_windows");
    tb->msg_connect(est, "cir", dbg, "store");

    // By construction: before any job runs, only the worker can have grown the
    // scratch, so both counters must still be zero.
    BOOST_CHECK_EQUAL(est->scratch_capacity_samples(), 0u);
    BOOST_CHECK_EQUAL(est->scratch_growths(), 0u);
    BOOST_CHECK_EQUAL(est->scratch_growth_us(), 0u);

    tb->start();
    // Pre-build every message (metadata + payload) BEFORE the post loop.  A
    // 1.5 MB PMT c32vector built inside the loop made the producer slower
    // than the worker once the sensing path got ~4x faster (R04), so the
    // backlog never formed and the queue high-watermark assertion became
    // flaky.  The test is about queue/ownership behaviour, not PMT cost.
    std::vector<pmt::pmt_t> msgs;
    msgs.reserve(kW);
    for (size_t i = 0; i < kW; ++i) {
        const std::vector<gr_complex> w = window_of(rx, lens[i]);
        msgs.push_back(pmt::cons(make_meta(500 + i, kPreGuard),
                                 pmt::init_c32vector(w.size(), w.data())));
    }
    // Post the whole backlog back-to-back: the worker is still busy with the
    // first window while the message thread assembles the rest.
    for (size_t i = 0; i < kW; ++i)
        est->_post(pmt::mp("rx"), msgs[i]);
    BOOST_REQUIRE(wait_frames(dbg, kW, 60000));
    est->drain();
    BOOST_CHECK(est->drained());

    BOOST_CHECK_EQUAL(est->pdus_received(), kW);
    BOOST_CHECK_EQUAL(est->pdus_enqueued(), kW);
    BOOST_CHECK_EQUAL(est->pdus_dropped(), 0u);
    BOOST_CHECK_EQUAL(est->worker_exceptions(), 0u);
    BOOST_CHECK_GE(est->queue_high_watermark(), size_t(2));
    BOOST_REQUIRE_EQUAL(dbg->num_messages(), kW);

    for (size_t i = 0; i < kW; ++i) {
        const FrameSnapshot got = snapshot_frame(dbg->get_message(i));
        BOOST_CHECK_EQUAL(pmt::to_uint64(pmt::dict_ref(
                              pmt::car(dbg->get_message(i)),
                              pmt::mp("pulse_id"), pmt::from_uint64(0))),
                          500 + i);
        compare_frames(got, ref[i],
                       "variable window len=" + std::to_string(lens[i]));
        BOOST_CHECK_EQUAL(got.status, "ok");
    }

    // Growth accounting: the worker grew the scratch exactly once per new
    // maximum in FIFO order (78000, 197005, 260000, 300000), never from the
    // message thread.
    BOOST_CHECK_EQUAL(est->scratch_growths(), 4u);
    BOOST_CHECK_EQUAL(est->scratch_capacity_samples(), size_t(300000));
    BOOST_TEST_MESSAGE("scratch growths=" << est->scratch_growths()
                                          << " growth_us="
                                          << est->scratch_growth_us()
                                          << " capacity="
                                          << est->scratch_capacity_samples());

    // stop() drains the pending job and joins; a larger window posted just
    // before it must still be processed and counted as one more growth.  The
    // PDU travels through the block's message queue, so wait until the handler
    // has ENQUEUED it before stopping (otherwise stop() can win the race and
    // leave the job in a queue nobody drains).
    est->_post(pmt::mp("rx"),
               make_pdu(make_meta(600, kPreGuard), window_of(rx, 320000)));
    BOOST_REQUIRE(wait_until([&] { return est->pdus_enqueued() == kW + 1; }));
    BOOST_REQUIRE(est->stop());
    BOOST_REQUIRE(wait_frames(dbg, kW + 1, 60000));
    BOOST_CHECK_EQUAL(est->pdus_completed(), kW + 1);
    BOOST_CHECK_EQUAL(est->scratch_growths(), 5u);
    BOOST_CHECK_EQUAL(est->scratch_capacity_samples(), size_t(320000));

    // Restart, then fail and recover on the same worker-owned scratch.
    BOOST_REQUIRE(est->start());
    const std::vector<gr_complex> zero(300000, gr_complex(0.0f, 0.0f));
    est->_post(pmt::mp("rx"), make_pdu(make_meta(700, kPreGuard), zero));
    est->_post(pmt::mp("rx"),
               make_pdu(make_meta(701, kPreGuard), window_of(rx, rx.size())));
    BOOST_REQUIRE(wait_frames(dbg, kW + 3, 60000));
    est->drain();
    BOOST_CHECK(est->drained());

    const FrameSnapshot failed = snapshot_frame(dbg->get_message(kW + 1));
    BOOST_CHECK_NE(failed.status, "ok");
    BOOST_CHECK_EQUAL(failed.tap_count, 0u);
    BOOST_CHECK(failed.taps.empty());
    BOOST_CHECK(!failed.status_reason.empty());
    const FrameSnapshot recovered = snapshot_frame(dbg->get_message(kW + 2));
    compare_frames(recovered, ref[1], "recovery after failure");
    BOOST_CHECK_EQUAL(recovered.status, "ok");
    // The failed window did not grow the scratch (300000 <= 320000), so the
    // recovery is not a "fresh scratch" result.
    BOOST_CHECK_EQUAL(est->scratch_growths(), 5u);

    tb->stop();
    tb->wait();
}

// ---------------------------------------------------------------------------
// R03 item 2 — timed mode with an UNKNOWN target CFO.
//
// Before: it fell through to `cir_rx = rx` and returned Ok with cfo_epoch == 0,
// i.e. a silently uncompensated CIR.  Now it runs the shared front-end (seeded
// with the TX-time axis, period_hint = samples_per_symbol) to estimate the CFO,
// and fails closed with a named reason when the echo cannot be acquired.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_estimator_timed_mode_estimates_target_cfo)
{
    std::vector<gr_complex> rx, tx;
    BOOST_REQUIRE(
        load_cf32(testdata_path("uwb_radar/rx_clean_998p4.cf32"), rx));
    BOOST_REQUIRE(load_cf32(testdata_path("uwb_radar/tx_998p4.cf32"), tx));
    const std::string tmpl_path = write_template_file(tx);

    // Inject a +50 kHz target CFO on the whole window (a public input: the
    // estimator never sees the injected value).
    const double cfo = 50000.0;
    const double w = 2.0 * M_PI * cfo / kWorkRate;
    std::vector<gr_complex> rx_cfo = rx;
    for (size_t i = 0; i < rx_cfo.size(); ++i) {
        const double ph = w * static_cast<double>(i);
        rx_cfo[i] *= gr_complex(static_cast<float>(std::cos(ph)),
                                static_cast<float>(std::sin(ph)));
    }

    // (a) Timed mode, unknown CFO: the front-end estimates and compensates.
    auto est_timed = UwbRadarCirEstimator::make(
        tmpl_path, 64, "4z2", 9, kCirPre, kCirPost, 10, 0, 64, 8, 0.3f, 0.3f,
        true, 8, /*use_predicted_timing=*/true);
    auto dbg_timed = gr::blocks::message_debug::make();
    auto tb1 = gr::make_top_block("qa_radar_estimator_timed_cfo");
    tb1->msg_connect(est_timed, "cir", dbg_timed, "store");
    tb1->start();
    est_timed->_post(pmt::mp("rx"),
                     make_pdu(make_meta(800, kPreGuard), rx_cfo));
    BOOST_REQUIRE(wait_frames(dbg_timed, 1));
    BOOST_REQUIRE(wait_drained(est_timed));
    tb1->stop();
    tb1->wait();
    BOOST_REQUIRE_EQUAL(est_timed->pdus_completed(), 1u);
    const FrameSnapshot timed = snapshot_frame(dbg_timed->get_message(0));
    BOOST_CHECK_EQUAL(timed.status, "ok");
    BOOST_CHECK_EQUAL(timed.cfo_source, "estimated_timed");
    BOOST_CHECK_EQUAL(timed.cfo_epoch, 2u);
    BOOST_TEST_MESSAGE("timed estimated cfo=" << timed.final_cfo_hz << " Hz");
    BOOST_CHECK_LT(std::abs(timed.final_cfo_hz - cfo), 2000.0);
    BOOST_CHECK_GE(timed.covered_lo, 0);
    BOOST_CHECK_GT(timed.covered_hi, timed.covered_lo);
    // The timed contract is unchanged: the published detection coordinates are
    // the PREDICTED ones, not the front-end's detected peak.
    BOOST_CHECK_EQUAL(timed.sfd_start, kPreGuard + 64 * static_cast<int64_t>(kSps));
    BOOST_CHECK_EQUAL(timed.cir_origin, kPreGuard);

    // (b) The detected path runs the same front-end on the same window, so the
    // compensated taps must be identical (the only difference is provenance).
    auto est_det = UwbRadarCirEstimator::make(
        tmpl_path, 64, "4z2", 9, kCirPre, kCirPost, 10, 0, 64, 8, 0.3f, 0.3f,
        true, 8, /*use_predicted_timing=*/false);
    auto dbg_det = gr::blocks::message_debug::make();
    auto tb2 = gr::make_top_block("qa_radar_estimator_detected_cfo");
    tb2->msg_connect(est_det, "cir", dbg_det, "store");
    tb2->start();
    est_det->_post(pmt::mp("rx"), make_pdu(make_meta(801, kPreGuard), rx_cfo));
    BOOST_REQUIRE(wait_frames(dbg_det, 1));
    BOOST_REQUIRE(wait_drained(est_det));
    tb2->stop();
    tb2->wait();
    const FrameSnapshot det = snapshot_frame(dbg_det->get_message(0));
    BOOST_CHECK_EQUAL(det.status, "ok");
    BOOST_CHECK_EQUAL(det.cfo_source, "estimated_detected");
    BOOST_CHECK_EQUAL(det.cfo_epoch, 2u);
    BOOST_CHECK_EQUAL(det.final_cfo_hz, timed.final_cfo_hz);
    BOOST_CHECK_EQUAL(det.peak_tap, timed.peak_tap);
    BOOST_REQUIRE_EQUAL(det.taps.size(), timed.taps.size());
    BOOST_CHECK(std::memcmp(det.taps.data(), timed.taps.data(),
                            det.taps.size() * sizeof(gr_complex)) == 0);

    // (c) Fail closed: a timed window whose SFD is destroyed cannot be
    // acquired, so the call must NOT return Ok with an uncompensated CIR.
    std::vector<gr_complex> bad = rx_cfo;
    const size_t sfd_len =
        gr::uwb::demod::GetSfdSequence("4z2").size() * kSps;
    const int64_t sfd_start = kPreGuard + 64 * static_cast<int64_t>(kSps);
    std::fill(bad.begin() + sfd_start,
              bad.begin() + sfd_start + static_cast<int64_t>(sfd_len),
              gr_complex(0.0f, 0.0f));
    auto est_bad = UwbRadarCirEstimator::make(
        tmpl_path, 64, "4z2", 9, kCirPre, kCirPost, 10, 0, 64, 8, 0.3f, 0.3f,
        true, 8, /*use_predicted_timing=*/true);
    auto dbg_bad = gr::blocks::message_debug::make();
    auto tb3 = gr::make_top_block("qa_radar_estimator_timed_cfo_fail");
    tb3->msg_connect(est_bad, "cir", dbg_bad, "store");
    tb3->start();
    est_bad->_post(pmt::mp("rx"), make_pdu(make_meta(802, kPreGuard), bad));
    BOOST_REQUIRE(wait_frames(dbg_bad, 1));
    BOOST_REQUIRE(wait_drained(est_bad));
    tb3->stop();
    tb3->wait();
    BOOST_CHECK_EQUAL(est_bad->pdus_failed(), 1u);
    const FrameSnapshot f = snapshot_frame(dbg_bad->get_message(0));
    BOOST_CHECK_NE(f.status, "ok");
    BOOST_CHECK_EQUAL(f.tap_count, 0u);
    BOOST_CHECK(f.taps.empty());
    BOOST_CHECK_EQUAL(f.cfo_epoch, 0u);
    BOOST_TEST_MESSAGE("timed unknown-CFO failure: " << f.status << " / "
                                                     << f.status_reason);
    BOOST_CHECK(!f.status_reason.empty());
    BOOST_CHECK(f.status_reason == "timed_cfo_estimate_failed" ||
                f.status_reason == "timed_cfo_sfd_outside_roi" ||
                f.status_reason == "timed_cfo_unavailable");
}

// ---------------------------------------------------------------------------
// R03 item 4 — the alongside average is a SECOND derotated consumer.
//
// Main CIR: skip 10 / count 54.  Alongside average: skip 0 / count 64 (the
// block default).  Its windows therefore reach EARLIER samples than the main
// CIR's union.  The final rotation must be planned over the union of both, and
// the average must be bit-exact against the same kernel fed the same
// compensated buffer.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_estimator_alongside_average_union_coverage)
{
    std::vector<gr_complex> rx, tx;
    BOOST_REQUIRE(
        load_cf32(testdata_path("uwb_radar/rx_clean_998p4.cf32"), rx));
    BOOST_REQUIRE(load_cf32(testdata_path("uwb_radar/tx_998p4.cf32"), tx));
    const std::string tmpl_path = write_template_file(tx);

    // Non-zero target CFO so "compensated" is observable, not an identity.
    const double cfo = -50000.0;
    const double w = 2.0 * M_PI * cfo / kWorkRate;
    std::vector<gr_complex> rx_cfo = rx;
    for (size_t i = 0; i < rx_cfo.size(); ++i) {
        const double ph = w * static_cast<double>(i);
        rx_cfo[i] *= gr_complex(static_cast<float>(std::cos(ph)),
                                static_cast<float>(std::sin(ph)));
    }

    // ---- reference: same core call with the union declared ----------------
    RadarCirConfig cfg = direct_cfg();
    cfg.extra_cir_skip_initial = 0;   // alongside average: skip 0
    cfg.extra_cir_repetitions = 64;   // ... count 64
    RadarCirCoreScratch scratch;
    BOOST_REQUIRE(prepare_radar_cir_core(cfg, tx.data(), kSps, kCirPre,
                                         kCirPost, scratch));
    const int64_t predicted = kPreGuard + 64 * static_cast<int64_t>(kSps);
    RadarCirResult main_ref;
    BOOST_REQUIRE(radar_cir_one(rx_cfo.data(), rx_cfo.size(), predicted, cfg,
                                scratch, main_ref));
    BOOST_REQUIRE_EQUAL(main_ref.valid_repetitions, 54u);
    BOOST_REQUIRE_GE(main_ref.covered_lo, 0);
    // The union must reach EARLIER than the main CIR's own first window
    // (origin + 10*P - pre), i.e. the extra consumer really widened it.
    const int64_t main_first =
        main_ref.cir_axis_origin + 10 * static_cast<int64_t>(kSps) - kCirPre;
    BOOST_CHECK_LE(main_ref.covered_lo, main_ref.cir_axis_origin - kCirPre);
    BOOST_CHECK_LT(main_ref.covered_lo, main_first);

    RadarCirResult avg_ref;
    BOOST_REQUIRE(radar_cir_average_range(
        scratch.demod.derotated.data(), rx_cfo.size(), cfg,
        main_ref.cir_axis_origin, main_ref.cir_samples_per_symbol, 0, 64,
        scratch, avg_ref, main_ref.covered_lo, main_ref.covered_hi));
    BOOST_REQUIRE_EQUAL(avg_ref.valid_repetitions, 64u);
    std::vector<gr_complex> avg_ref_taps(
        scratch.cir.raw_taps.begin(),
        scratch.cir.raw_taps.begin() +
            static_cast<std::ptrdiff_t>(avg_ref.tap_count));

    // The OLD plan (main windows only) does NOT contain the skip-0 range: that
    // is exactly the silently-uncompensated read this item closes.
    RadarCirConfig main_only = cfg;
    main_only.extra_cir_repetitions = 0;
    RadarCirCoreScratch scratch_main_only;
    BOOST_REQUIRE(prepare_radar_cir_core(main_only, tx.data(), kSps, kCirPre,
                                         kCirPost, scratch_main_only));
    RadarCirResult main_only_ref;
    BOOST_REQUIRE(radar_cir_one(rx_cfo.data(), rx_cfo.size(), predicted,
                                main_only, scratch_main_only, main_only_ref));
    int64_t need_lo = -1;
    int64_t need_hi = -1;
    BOOST_CHECK(!radar_cir_coverage_ok(
        cfg, main_only_ref.cir_axis_origin, main_only_ref.cir_samples_per_symbol,
        0, 64, 64, rx_cfo.size(), main_only_ref.covered_lo,
        main_only_ref.covered_hi, need_lo, need_hi));
    // ... and the union plan does contain it.
    BOOST_CHECK(radar_cir_coverage_ok(
        cfg, main_ref.cir_axis_origin, main_ref.cir_samples_per_symbol, 0, 64,
        64, rx_cfo.size(), main_ref.covered_lo, main_ref.covered_hi, need_lo,
        need_hi));

    // ---- the block -------------------------------------------------------
    auto est = UwbRadarCirEstimator::make(
        tmpl_path, 64, "4z2", 9, kCirPre, kCirPost,
        /*cir_skip_initial=*/10, /*cir_repetitions=*/0, 64, 8, 0.3f, 0.3f,
        true, 8, /*use_predicted_timing=*/false,
        /*emit_individual_repetitions=*/true,
        /*batch_individual_repetitions=*/true,
        /*emit_repetition_average=*/true,
        /*repetition_average_skip=*/0);
    BOOST_CHECK_EQUAL(est->repetition_average_skip(), 0u);
    auto dbg = gr::blocks::message_debug::make();
    auto dbg_avg = gr::blocks::message_debug::make();
    auto tb = gr::make_top_block("qa_radar_estimator_alongside_union");
    tb->msg_connect(est, "cir", dbg, "store");
    tb->msg_connect(est, "cir_avg", dbg_avg, "store");
    tb->start();
    est->_post(pmt::mp("rx"), make_pdu(make_meta(900, kPreGuard), rx_cfo));
    BOOST_REQUIRE(wait_frames(dbg_avg, 1));
    BOOST_REQUIRE(wait_drained(est));
    tb->stop();
    tb->wait();
    BOOST_REQUIRE_EQUAL(est->pdus_completed(), 1u);
    BOOST_REQUIRE_EQUAL(dbg_avg->num_messages(), 1u);

    const FrameSnapshot avg = snapshot_frame(dbg_avg->get_message(0));
    BOOST_CHECK_EQUAL(avg.status, "ok");
    BOOST_CHECK_EQUAL(avg.cir_output, "average");
    BOOST_CHECK_EQUAL(avg.cfo_epoch, 2u);
    BOOST_CHECK_EQUAL(avg.covered_lo, main_ref.covered_lo);
    BOOST_CHECK_EQUAL(avg.covered_hi, main_ref.covered_hi);
    BOOST_REQUIRE_EQUAL(avg.taps.size(), avg_ref_taps.size());
    BOOST_CHECK_MESSAGE(std::memcmp(avg.taps.data(), avg_ref_taps.data(),
                                    avg.taps.size() * sizeof(gr_complex)) == 0,
                        "alongside average taps differ from the union reference");
    pmt::pmt_t avg_meta = pmt::car(dbg_avg->get_message(0));
    BOOST_CHECK_EQUAL(pmt::to_uint64(pmt::dict_ref(
                          avg_meta, pmt::mp("averaged_repetition_first"),
                          pmt::from_uint64(99))),
                      0u);
    BOOST_CHECK_EQUAL(pmt::to_uint64(pmt::dict_ref(
                          avg_meta, pmt::mp("valid_repetitions"),
                          pmt::from_uint64(0))),
                      64u);
    BOOST_CHECK(pmt::to_bool(pmt::dict_ref(avg_meta, pmt::mp("cfo_compensated"),
                                           pmt::PMT_F)));

    // The per-repetition records on "cir" are the RAW diagnostic, never the
    // compensated average.  Wait for the store: `drained()` does not cover the
    // message_debug block's own queue.
    BOOST_REQUIRE(wait_frames(dbg, 1));
    BOOST_REQUIRE_EQUAL(dbg->num_messages(), 1u);
    pmt::pmt_t batch_meta = pmt::car(dbg->get_message(0));
    BOOST_CHECK_EQUAL(meta_str(batch_meta, "cir_output"),
                      "repetition_raw_uncompensated");
    BOOST_CHECK(!pmt::to_bool(pmt::dict_ref(
        batch_meta, pmt::mp("cfo_compensated"), pmt::PMT_T)));
    BOOST_CHECK_EQUAL(pmt::to_uint64(pmt::dict_ref(
                          batch_meta, pmt::mp("cfo_epoch"),
                          pmt::from_uint64(99))),
                      0u);
    BOOST_CHECK_CLOSE(pmt::to_double(pmt::dict_ref(
                          batch_meta, pmt::mp("pulse_final_cfo_hz"),
                          pmt::from_double(0.0))),
                      main_ref.final_cfo_hz, 1e-9);
}

// ---------------------------------------------------------------------------
// R03 item 1/2 — timed mode with a KNOWN target CFO.
//
// A calibrated link declares its offset (including an explicit ZERO) and the
// timed path derotates directly, without a front-end pass.  `cfo_epoch == 2`
// and `cfo_source == "caller_known"` record that the CIR is compensated, so a
// declared zero can never be confused with the old "assumed zero" fallthrough.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_estimator_timed_mode_known_cfo)
{
    std::vector<gr_complex> rx, tx;
    BOOST_REQUIRE(
        load_cf32(testdata_path("uwb_radar/rx_clean_998p4.cf32"), rx));
    BOOST_REQUIRE(load_cf32(testdata_path("uwb_radar/tx_998p4.cf32"), tx));
    const std::string tmpl_path = write_template_file(tx);

    const double cfo = 50000.0;
    const double w = 2.0 * M_PI * cfo / kWorkRate;
    std::vector<gr_complex> rx_cfo = rx;
    for (size_t i = 0; i < rx_cfo.size(); ++i) {
        const double ph = w * static_cast<double>(i);
        rx_cfo[i] *= gr_complex(static_cast<float>(std::cos(ph)),
                                static_cast<float>(std::sin(ph)));
    }

    auto run = [&](const std::vector<gr_complex>& window, bool known,
                   double declared, uint64_t id) {
        auto est = UwbRadarCirEstimator::make(
            tmpl_path, 64, "4z2", 9, kCirPre, kCirPost, 10, 0, 64, 8, 0.3f,
            0.3f, true, 8, /*use_predicted_timing=*/true,
            /*emit_individual_repetitions=*/false,
            /*batch_individual_repetitions=*/false,
            /*emit_repetition_average=*/false,
            /*repetition_average_skip=*/10, known, declared);
        BOOST_CHECK_EQUAL(est->target_cfo_known(), known);
        BOOST_CHECK_EQUAL(est->target_cfo_hz(), declared);
        auto dbg = gr::blocks::message_debug::make();
        auto tb = gr::make_top_block("qa_radar_estimator_known_cfo");
        tb->msg_connect(est, "cir", dbg, "store");
        tb->start();
        est->_post(pmt::mp("rx"), make_pdu(make_meta(id, kPreGuard), window));
        BOOST_REQUIRE(wait_frames(dbg, 1));
        BOOST_REQUIRE(wait_drained(est));
        tb->stop();
        tb->wait();
        BOOST_REQUIRE_EQUAL(dbg->num_messages(), 1u);
        return snapshot_frame(dbg->get_message(0));
    };

    // Reference: zero CFO, declared zero.
    const FrameSnapshot zero = run(rx, true, 0.0, 850);
    BOOST_CHECK_EQUAL(zero.status, "ok");
    BOOST_CHECK_EQUAL(zero.cfo_epoch, 2u);
    BOOST_CHECK_EQUAL(zero.cfo_source, "caller_known");
    BOOST_CHECK_EQUAL(zero.final_cfo_hz, 0.0);
    BOOST_CHECK_EQUAL(zero.bootstrap_cfo_hz, 0.0);

    // Declared +50 kHz on a +50 kHz window: the peak and the taps match the
    // zero-CFO reference (a compensated, not an assumed, result).
    const FrameSnapshot comp = run(rx_cfo, true, cfo, 851);
    BOOST_CHECK_EQUAL(comp.status, "ok");
    BOOST_CHECK_EQUAL(comp.cfo_epoch, 2u);
    BOOST_CHECK_EQUAL(comp.cfo_source, "caller_known");
    BOOST_CHECK_EQUAL(comp.final_cfo_hz, cfo);
    BOOST_CHECK_EQUAL(comp.peak_tap, zero.peak_tap);
    BOOST_REQUIRE_EQUAL(comp.taps.size(), zero.taps.size());
    double num = 0.0;
    double den = 0.0;
    for (size_t i = 0; i < comp.taps.size(); ++i) {
        num += std::norm(comp.taps[i] - zero.taps[i]);
        den += std::norm(zero.taps[i]);
    }
    const double rel = std::sqrt(num / std::max(den, 1e-30));
    BOOST_TEST_MESSAGE("declared-CFO taps vs zero-CFO reference relL2=" << rel);
    BOOST_CHECK_LT(rel, 1e-4);

    // Declared zero on a +50 kHz window: an explicit zero is still a
    // compensation claim, so the result must NOT look like the compensated
    // one (it is the caller's wrong calibration, not a silent fallthrough).
    const FrameSnapshot wrong = run(rx_cfo, true, 0.0, 852);
    BOOST_CHECK_EQUAL(wrong.status, "ok");
    BOOST_CHECK_EQUAL(wrong.cfo_epoch, 2u);
    BOOST_CHECK_EQUAL(wrong.final_cfo_hz, 0.0);
    BOOST_CHECK_GT(rel_l2_block(wrong.taps, zero.taps), 0.1);
}

// ---------------------------------------------------------------------------
// R03 item 4 — the alongside average on the TIMED path with a declared CFO.
//
// The known-CFO timed arm plans its derotation range from the union request
// too, so the extra consumer (skip 0 / count 64) is covered even though the
// main CIR reads skip 10 / count 54.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_estimator_timed_known_cfo_alongside_average)
{
    std::vector<gr_complex> rx, tx;
    BOOST_REQUIRE(
        load_cf32(testdata_path("uwb_radar/rx_clean_998p4.cf32"), rx));
    BOOST_REQUIRE(load_cf32(testdata_path("uwb_radar/tx_998p4.cf32"), tx));
    const std::string tmpl_path = write_template_file(tx);

    const double cfo = 50000.0;
    const double w = 2.0 * M_PI * cfo / kWorkRate;
    std::vector<gr_complex> rx_cfo = rx;
    for (size_t i = 0; i < rx_cfo.size(); ++i) {
        const double ph = w * static_cast<double>(i);
        rx_cfo[i] *= gr_complex(static_cast<float>(std::cos(ph)),
                                static_cast<float>(std::sin(ph)));
    }

    auto est = UwbRadarCirEstimator::make(
        tmpl_path, 64, "4z2", 9, kCirPre, kCirPost,
        /*cir_skip_initial=*/10, /*cir_repetitions=*/0, 64, 8, 0.3f, 0.3f,
        true, 8, /*use_predicted_timing=*/true,
        /*emit_individual_repetitions=*/true,
        /*batch_individual_repetitions=*/true,
        /*emit_repetition_average=*/true,
        /*repetition_average_skip=*/0,
        /*target_cfo_known=*/true, /*target_cfo_hz=*/cfo);
    auto dbg = gr::blocks::message_debug::make();
    auto dbg_avg = gr::blocks::message_debug::make();
    auto tb = gr::make_top_block("qa_radar_estimator_timed_known_avg");
    tb->msg_connect(est, "cir", dbg, "store");
    tb->msg_connect(est, "cir_avg", dbg_avg, "store");
    tb->start();
    est->_post(pmt::mp("rx"), make_pdu(make_meta(950, kPreGuard), rx_cfo));
    BOOST_REQUIRE(wait_frames(dbg_avg, 1));
    BOOST_REQUIRE(wait_drained(est));
    tb->stop();
    tb->wait();

    BOOST_REQUIRE_EQUAL(dbg_avg->num_messages(), 1u);
    const FrameSnapshot avg = snapshot_frame(dbg_avg->get_message(0));
    BOOST_TEST_MESSAGE("timed known-CFO alongside average: status="
                       << avg.status << " reason=" << avg.status_reason
                       << " covered=[" << avg.covered_lo << ","
                       << avg.covered_hi << ")");
    BOOST_CHECK_EQUAL(avg.status, "ok");
    BOOST_CHECK_EQUAL(avg.cfo_epoch, 2u);
    BOOST_CHECK_EQUAL(avg.cfo_source, "caller_known");
    BOOST_CHECK_EQUAL(avg.covered_lo, kPreGuard - static_cast<int64_t>(kCirPre));
    pmt::pmt_t meta = pmt::car(dbg_avg->get_message(0));
    BOOST_CHECK_EQUAL(pmt::to_uint64(pmt::dict_ref(
                          meta, pmt::mp("valid_repetitions"),
                          pmt::from_uint64(0))),
                      64u);
    BOOST_CHECK_EQUAL(pmt::to_uint64(pmt::dict_ref(
                          meta, pmt::mp("averaged_repetition_first"),
                          pmt::from_uint64(99))),
                      0u);
}
