/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * QA for header-only radar CIR integrator (SFD → SYNC backtrack → CIR).
 *
 * Canonical MATLAB identity uses testdata/uwb_radar/ (generator=
 * export_uwb_radar_golden.m). 32/64/128 structural tests splice
 * testdata/reference_preamble.bin + kron(4z2, SYNC); that splice is not
 * lrwpanWaveformGenerator.
 */

#include <boost/test/unit_test.hpp>

#include <gnuradio/gr_complex.h>
#include <gnuradio/uwb/uwb_phy_profile.h>
#include <gnuradio/uwb/uwb_radar_cir_core.h>

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

namespace {

using gr::uwb::demod::GetPreambleCode;
using gr::uwb::demod::GetSfdSequence;
using gr::uwb::demod::kQm35CodeLength;
using gr::uwb::demod::kQm35SamplesPerSymbol;
using gr::uwb::radar::prepare_radar_cir_core;
using gr::uwb::radar::radar_cir_one;
using gr::uwb::radar::RadarCirConfig;
using gr::uwb::radar::RadarCirCoreScratch;
using gr::uwb::radar::RadarCirResult;
using gr::uwb::radar::RadarCirStatus;

#ifndef UWB_TESTDATA_DIR
#define UWB_TESTDATA_DIR "../../../testdata"
#endif

constexpr float kThreshold = 0.3f;
constexpr int64_t kSfdMargin = 64;
constexpr int64_t kSyncMargin = 8;
constexpr size_t kCirPre = 16;
constexpr size_t kCirPost = 100;
constexpr size_t kCirSkip = 10;
constexpr size_t kCirReps = 54;
constexpr int64_t kSynthPreGuard = 64;
constexpr int64_t kSynthTail = 256;

bool load_cf32(const std::string& path, std::vector<gr_complex>& out)
{
    std::ifstream f(path, std::ios::binary);
    if (!f)
        return false;
    f.seekg(0, std::ios::end);
    const size_t bytes = static_cast<size_t>(f.tellg());
    f.seekg(0, std::ios::beg);
    if (bytes == 0 || bytes % 8 != 0)
        return false;
    out.resize(bytes / 8);
    f.read(reinterpret_cast<char*>(out.data()),
           static_cast<std::streamsize>(bytes));
    return static_cast<bool>(f);
}

std::string testdata_path(const std::string& rel)
{
    const std::string from_def = std::string(UWB_TESTDATA_DIR) + "/" + rel;
    {
        std::ifstream f(from_def, std::ios::binary);
        if (f)
            return from_def;
    }
    const char* prefixes[] = {
        "../../../testdata/",
        "../../testdata/",
        "../testdata/",
        "testdata/",
    };
    for (const char* p : prefixes) {
        const std::string path = std::string(p) + rel;
        std::ifstream f(path, std::ios::binary);
        if (f)
            return path;
    }
    return from_def;
}

std::vector<gr_complex> load_sync_pulse()
{
    std::vector<gr_complex> tmpl;
    const std::string path = testdata_path("reference_preamble.bin");
    BOOST_REQUIRE_MESSAGE(load_cf32(path, tmpl),
                          "cannot load reference_preamble.bin from " + path);
    BOOST_REQUIRE_EQUAL(tmpl.size(), kQm35SamplesPerSymbol);
    return tmpl;
}

std::vector<gr_complex> kron_sfd(const std::vector<int8_t>& seq,
                                 const std::vector<gr_complex>& sync)
{
    std::vector<gr_complex> w(seq.size() * sync.size());
    for (size_t i = 0; i < seq.size(); ++i) {
        const float s = static_cast<float>(seq[i]);
        for (size_t k = 0; k < sync.size(); ++k)
            w[i * sync.size() + k] =
                gr_complex(s * sync[k].real(), s * sync[k].imag());
    }
    return w;
}

void place_waveform(std::vector<gr_complex>& rx,
                    int64_t start,
                    const std::vector<gr_complex>& w,
                    gr_complex gain = gr_complex(1.0f, 0.0f))
{
    BOOST_REQUIRE(start >= 0);
    BOOST_REQUIRE(static_cast<size_t>(start) + w.size() <= rx.size());
    for (size_t k = 0; k < w.size(); ++k)
        rx[static_cast<size_t>(start) + k] = gain * w[k];
}

RadarCirConfig make_cfg(size_t n_sync = 64)
{
    RadarCirConfig cfg;
    cfg.sync_repetitions = n_sync;
    cfg.samples_per_symbol = kQm35SamplesPerSymbol;
    cfg.sfd_mode = "4z2";
    cfg.sfd_search_margin = kSfdMargin;
    cfg.sync_refine_margin = kSyncMargin;
    cfg.sfd_threshold = kThreshold;
    cfg.sync_refine_threshold = kThreshold;
    cfg.cir_pre = kCirPre;
    cfg.cir_post = kCirPost;
    cfg.cir_skip_initial = kCirSkip;
    cfg.cir_repetitions = kCirReps;
    return cfg;
}

bool prepare_from_sync(const std::vector<gr_complex>& sync,
                       RadarCirCoreScratch& scratch,
                       RadarCirConfig cfg = RadarCirConfig{},
                       size_t max_pre = kCirPre,
                       size_t max_post = kCirPost)
{
    if (cfg.sync_repetitions == 0)
        cfg = make_cfg(64);
    cfg.samples_per_symbol = kQm35SamplesPerSymbol;
    return prepare_radar_cir_core(cfg, sync.data(), sync.size(), max_pre,
                                  max_post, scratch);
}

struct SynthPacket {
    std::vector<gr_complex> rx;
    int64_t origin = 0;
    int64_t sfd_start = 0;
    size_t n_sync = 0;
};

SynthPacket make_sync_sfd_packet(const std::vector<gr_complex>& sync,
                                 size_t n_sync,
                                 int64_t pre_guard = kSynthPreGuard,
                                 int64_t tail = kSynthTail)
{
    const auto seq = GetSfdSequence("4z2");
    const auto sfd_wf = kron_sfd(seq, sync);
    SynthPacket p;
    p.n_sync = n_sync;
    p.origin = pre_guard;
    p.sfd_start = pre_guard + static_cast<int64_t>(n_sync) *
                                  static_cast<int64_t>(sync.size());
    const size_t n = static_cast<size_t>(pre_guard) + n_sync * sync.size() +
                     sfd_wf.size() + static_cast<size_t>(tail);
    p.rx.assign(n, gr_complex(0.0f, 0.0f));
    for (size_t i = 0; i < n_sync; ++i)
        place_waveform(p.rx,
                       p.origin + static_cast<int64_t>(i) *
                                      static_cast<int64_t>(sync.size()),
                       sync);
    place_waveform(p.rx, p.sfd_start, sfd_wf);
    return p;
}

bool parse_json_string(const std::string& json,
                       const char* key,
                       std::string& out)
{
    const std::string pat = std::string("\"") + key + "\"";
    auto pos = json.find(pat);
    if (pos == std::string::npos)
        return false;
    pos = json.find(':', pos + pat.size());
    if (pos == std::string::npos)
        return false;
    pos = json.find('"', pos + 1);
    if (pos == std::string::npos)
        return false;
    const auto end = json.find('"', pos + 1);
    if (end == std::string::npos)
        return false;
    out = json.substr(pos + 1, end - pos - 1);
    return true;
}

bool parse_nested_number(const std::string& json,
                         const char* object_key,
                         const char* field,
                         double& out)
{
    const std::string obj = std::string("\"") + object_key + "\"";
    auto rx = json.find(obj);
    if (rx == std::string::npos)
        return false;
    const std::string pat = std::string("\"") + field + "\"";
    auto pos = json.find(pat, rx);
    if (pos == std::string::npos)
        return false;
    pos = json.find(':', pos + pat.size());
    if (pos == std::string::npos)
        return false;
    try {
        size_t idx = 0;
        out = std::stod(json.substr(pos + 1), &idx);
        return idx > 0;
    } catch (...) {
        return false;
    }
}

std::string load_canonical_metadata()
{
    const std::string meta_path = testdata_path("uwb_radar/metadata.json");
    std::ifstream meta_f(meta_path);
    BOOST_REQUIRE_MESSAGE(meta_f.good(),
                          "canonical testdata/uwb_radar/metadata.json missing");
    std::ostringstream oss;
    oss << meta_f.rdbuf();
    const std::string json = oss.str();
    std::string generator;
    BOOST_REQUIRE(parse_json_string(json, "generator", generator));
    BOOST_REQUIRE_MESSAGE(
        generator.find("export_uwb_radar_golden.m") != std::string::npos,
        "canonical generator must be export_uwb_radar_golden.m, got " +
            generator);
    return json;
}

double rel_l2(const std::vector<gr_complex>& a, const std::vector<gr_complex>& b)
{
    BOOST_REQUIRE_EQUAL(a.size(), b.size());
    double num = 0.0;
    double den = 0.0;
    for (size_t i = 0; i < a.size(); ++i) {
        const gr_complex d = a[i] - b[i];
        num += static_cast<double>(d.real()) * d.real() +
               static_cast<double>(d.imag()) * d.imag();
        den += static_cast<double>(b[i].real()) * b[i].real() +
               static_cast<double>(b[i].imag()) * b[i].imag();
    }
    if (den == 0.0)
        return num == 0.0 ? 0.0 : std::numeric_limits<double>::infinity();
    return std::sqrt(num / den);
}

void copy_taps(const RadarCirCoreScratch& scratch,
               size_t tap_count,
               std::vector<gr_complex>& raw,
               std::vector<gr_complex>& nrm)
{
    BOOST_REQUIRE_GE(scratch.cir.raw_taps.size(), tap_count);
    BOOST_REQUIRE_GE(scratch.cir.norm_taps.size(), tap_count);
    raw.assign(scratch.cir.raw_taps.begin(),
               scratch.cir.raw_taps.begin() +
                   static_cast<std::ptrdiff_t>(tap_count));
    nrm.assign(scratch.cir.norm_taps.begin(),
               scratch.cir.norm_taps.begin() +
                   static_cast<std::ptrdiff_t>(tap_count));
}

void assert_empty_cir(const RadarCirResult& out)
{
    BOOST_CHECK_EQUAL(out.tap_count, size_t(0));
    BOOST_CHECK_EQUAL(out.peak_tap, size_t(0));
    BOOST_CHECK_EQUAL(out.valid_repetitions, size_t(0));
}

void assert_failed_stage_indices(const RadarCirResult& out, bool sfd_ok,
                                 bool timing_ok)
{
    if (!sfd_ok) {
        BOOST_CHECK_EQUAL(out.sfd_start_sample, int64_t(-1));
        BOOST_CHECK_EQUAL(out.preamble_start_sample, int64_t(-1));
        BOOST_CHECK(out.sfd_start_sample != out.predicted_sfd_start ||
                    out.predicted_sfd_start < 0);
    } else {
        BOOST_CHECK_GE(out.sfd_start_sample, int64_t(0));
        if (!timing_ok)
            BOOST_CHECK_EQUAL(out.preamble_start_sample, int64_t(-1));
        else
            BOOST_CHECK_GE(out.preamble_start_sample, int64_t(0));
    }
}

} // namespace

BOOST_AUTO_TEST_CASE(test_radar_cir_invalid_input)
{
    const auto sync = load_sync_pulse();
    RadarCirCoreScratch scratch;
    BOOST_REQUIRE(prepare_from_sync(sync, scratch));
    auto pkt = make_sync_sfd_packet(sync, 64);
    const RadarCirConfig cfg = make_cfg(64);
    RadarCirResult out;

    BOOST_REQUIRE(!radar_cir_one(nullptr, pkt.rx.size(), pkt.sfd_start, cfg,
                                 scratch, out));
    BOOST_CHECK(out.status == RadarCirStatus::InvalidInput);
    BOOST_CHECK_EQUAL(out.sfd_start_sample, int64_t(-1));
    BOOST_CHECK_EQUAL(out.preamble_start_sample, int64_t(-1));
    BOOST_CHECK_EQUAL(out.cir_origin_sample, int64_t(-1));
    assert_empty_cir(out);

    BOOST_REQUIRE(!radar_cir_one(pkt.rx.data(), 0, pkt.sfd_start, cfg, scratch,
                                 out));
    BOOST_CHECK(out.status == RadarCirStatus::InvalidInput);
    assert_empty_cir(out);

    BOOST_REQUIRE(!radar_cir_one(pkt.rx.data(), pkt.rx.size(), -1, cfg, scratch,
                                 out));
    BOOST_CHECK(out.status == RadarCirStatus::InvalidInput);
    BOOST_CHECK_EQUAL(out.sfd_start_sample, int64_t(-1));
    assert_empty_cir(out);

    RadarCirConfig bad = cfg;
    bad.sfd_threshold = 0.0f;
    BOOST_REQUIRE(!radar_cir_one(pkt.rx.data(), pkt.rx.size(), pkt.sfd_start,
                                 bad, scratch, out));
    BOOST_CHECK(out.status == RadarCirStatus::InvalidInput);

    bad = cfg;
    bad.sfd_threshold = std::numeric_limits<float>::quiet_NaN();
    BOOST_REQUIRE(!radar_cir_one(pkt.rx.data(), pkt.rx.size(), pkt.sfd_start,
                                 bad, scratch, out));
    BOOST_CHECK(out.status == RadarCirStatus::InvalidInput);

    bad = cfg;
    bad.sync_refine_threshold = -0.1f;
    BOOST_REQUIRE(!radar_cir_one(pkt.rx.data(), pkt.rx.size(), pkt.sfd_start,
                                 bad, scratch, out));
    BOOST_CHECK(out.status == RadarCirStatus::InvalidInput);

    bad = cfg;
    bad.sfd_mode = "";
    BOOST_REQUIRE(!radar_cir_one(pkt.rx.data(), pkt.rx.size(), pkt.sfd_start,
                                 bad, scratch, out));
    BOOST_CHECK(out.status == RadarCirStatus::InvalidInput);

    bad = cfg;
    bad.sfd_mode = "not-a-mode";
    BOOST_REQUIRE(!radar_cir_one(pkt.rx.data(), pkt.rx.size(), pkt.sfd_start,
                                 bad, scratch, out));
    BOOST_CHECK(out.status == RadarCirStatus::InvalidInput);

    bad = cfg;
    bad.samples_per_symbol = 0;
    BOOST_REQUIRE(!radar_cir_one(pkt.rx.data(), pkt.rx.size(), pkt.sfd_start,
                                 bad, scratch, out));
    BOOST_CHECK(out.status == RadarCirStatus::InvalidInput);

    bad = cfg;
    bad.sync_repetitions = 0;
    BOOST_REQUIRE(!radar_cir_one(pkt.rx.data(), pkt.rx.size(), pkt.sfd_start,
                                 bad, scratch, out));
    BOOST_CHECK(out.status == RadarCirStatus::InvalidInput);

    bad = cfg;
    bad.cir_post = kCirPost + 1;
    BOOST_REQUIRE(!radar_cir_one(pkt.rx.data(), pkt.rx.size(), pkt.sfd_start,
                                 bad, scratch, out));
    BOOST_CHECK(out.status == RadarCirStatus::InvalidInput);

    RadarCirCoreScratch empty;
    BOOST_REQUIRE(!radar_cir_one(pkt.rx.data(), pkt.rx.size(), pkt.sfd_start,
                                 cfg, empty, out));
    BOOST_CHECK(out.status == RadarCirStatus::InvalidInput);
    BOOST_CHECK_EQUAL(out.sfd_start_sample, int64_t(-1));
    assert_empty_cir(out);
}

BOOST_AUTO_TEST_CASE(test_radar_cir_sfd_failed)
{
    const auto sync = load_sync_pulse();
    RadarCirCoreScratch scratch;
    BOOST_REQUIRE(prepare_from_sync(sync, scratch));
    auto pkt = make_sync_sfd_packet(sync, 64);
    const size_t sfd_len = GetSfdSequence("4z2").size() * sync.size();
    std::fill(pkt.rx.begin() + pkt.sfd_start,
              pkt.rx.begin() + pkt.sfd_start + static_cast<int64_t>(sfd_len),
              gr_complex(0.0f, 0.0f));

    const RadarCirConfig cfg = make_cfg(64);
    RadarCirResult out;
    const bool ok = radar_cir_one(pkt.rx.data(), pkt.rx.size(), pkt.sfd_start,
                                  cfg, scratch, out);
    BOOST_REQUIRE(!ok);
    BOOST_CHECK(out.status == RadarCirStatus::SfdFailed);
    BOOST_CHECK_EQUAL(out.predicted_sfd_start, pkt.sfd_start);
    BOOST_CHECK_EQUAL(out.sfd_start_sample, int64_t(-1));
    BOOST_CHECK(out.sfd_start_sample != out.predicted_sfd_start);
    BOOST_CHECK_EQUAL(out.preamble_start_sample, int64_t(-1));
    BOOST_CHECK_EQUAL(out.cir_origin_sample, int64_t(-1));
    assert_empty_cir(out);
    assert_failed_stage_indices(out, false, false);
}

BOOST_AUTO_TEST_CASE(test_radar_cir_timing_failed_zeroed_sync)
{
    const auto sync = load_sync_pulse();
    RadarCirCoreScratch scratch;
    BOOST_REQUIRE(prepare_from_sync(sync, scratch));
    auto pkt = make_sync_sfd_packet(sync, 64);
    std::fill(pkt.rx.begin() + pkt.origin, pkt.rx.begin() + pkt.sfd_start,
              gr_complex(0.0f, 0.0f));

    const RadarCirConfig cfg = make_cfg(64);
    RadarCirResult out;
    const bool ok = radar_cir_one(pkt.rx.data(), pkt.rx.size(), pkt.sfd_start,
                                  cfg, scratch, out);
    BOOST_REQUIRE(!ok);
    BOOST_CHECK(out.status == RadarCirStatus::TimingFailed);
    BOOST_CHECK_EQUAL(out.sfd_start_sample, pkt.sfd_start);
    BOOST_CHECK_EQUAL(out.preamble_start_sample, int64_t(-1));
    BOOST_CHECK_EQUAL(out.cir_origin_sample, int64_t(-1));
    BOOST_CHECK(out.preamble_start_sample != pkt.origin);
    assert_empty_cir(out);
}

BOOST_AUTO_TEST_CASE(test_radar_cir_timing_failed_origin_clipped)
{
    const auto sync = load_sync_pulse();
    RadarCirCoreScratch scratch;
    BOOST_REQUIRE(prepare_from_sync(sync, scratch));
    const auto seq = GetSfdSequence("4z2");
    const auto sfd_wf = kron_sfd(seq, sync);
    std::vector<gr_complex> rx(sfd_wf.size() + 128, gr_complex(0.0f, 0.0f));
    const int64_t sfd_start = 32;
    place_waveform(rx, sfd_start, sfd_wf);

    const RadarCirConfig cfg = make_cfg(64);
    RadarCirResult out;
    const bool ok =
        radar_cir_one(rx.data(), rx.size(), sfd_start, cfg, scratch, out);
    BOOST_REQUIRE(!ok);
    BOOST_CHECK(out.status == RadarCirStatus::TimingFailed);
    BOOST_CHECK_EQUAL(out.sfd_start_sample, sfd_start);
    BOOST_CHECK_EQUAL(out.preamble_start_sample, int64_t(-1));
    assert_empty_cir(out);
}

BOOST_AUTO_TEST_CASE(test_radar_cir_cir_failed_skip_too_large)
{
    const auto sync = load_sync_pulse();
    RadarCirCoreScratch scratch;
    BOOST_REQUIRE(prepare_from_sync(sync, scratch));
    auto pkt = make_sync_sfd_packet(sync, 64);
    RadarCirConfig cfg = make_cfg(64);
    cfg.cir_skip_initial = 64;

    RadarCirResult out;
    const bool ok = radar_cir_one(pkt.rx.data(), pkt.rx.size(), pkt.sfd_start,
                                  cfg, scratch, out);
    BOOST_REQUIRE(!ok);
    BOOST_CHECK(out.status == RadarCirStatus::CirFailed);
    BOOST_CHECK_EQUAL(out.sfd_start_sample, pkt.sfd_start);
    BOOST_CHECK_EQUAL(out.preamble_start_sample, pkt.origin);
    BOOST_CHECK_EQUAL(out.cir_origin_sample, pkt.origin);
    assert_empty_cir(out);
}

BOOST_AUTO_TEST_CASE(test_radar_cir_canonical_matlab_clean)
{
    const std::string json = load_canonical_metadata();

    double sfd_d = -1.0;
    BOOST_REQUIRE(parse_nested_number(json, "rx_clean_998p4", "sfd_start",
                                      sfd_d));
    const int64_t sfd_truth = static_cast<int64_t>(std::llround(sfd_d));
    double origin_d = -1.0;
    BOOST_REQUIRE(parse_nested_number(json, "rx_clean_998p4", "sync_origin",
                                      origin_d));
    const int64_t origin_truth = static_cast<int64_t>(std::llround(origin_d));
    BOOST_CHECK_EQUAL(origin_truth, int64_t(1997));
    BOOST_CHECK_EQUAL(sfd_truth, origin_truth + int64_t(64) * int64_t(1016));

    std::vector<gr_complex> rx, tx, raw_g, nrm_g;
    BOOST_REQUIRE(load_cf32(testdata_path("uwb_radar/rx_clean_998p4.cf32"), rx));
    BOOST_REQUIRE(load_cf32(testdata_path("uwb_radar/tx_998p4.cf32"), tx));
    BOOST_REQUIRE(
        load_cf32(testdata_path("uwb_radar/cir_raw_clean_radar.cf32"), raw_g));
    BOOST_REQUIRE(
        load_cf32(testdata_path("uwb_radar/cir_norm_clean_radar.cf32"), nrm_g));
    BOOST_REQUIRE_GE(tx.size(), kQm35SamplesPerSymbol);
    BOOST_REQUIRE_EQUAL(raw_g.size(), kCirPre + kCirPost);
    BOOST_REQUIRE_EQUAL(nrm_g.size(), kCirPre + kCirPost);

    std::vector<gr_complex> sync(
        tx.begin(),
        tx.begin() + static_cast<std::ptrdiff_t>(kQm35SamplesPerSymbol));
    RadarCirCoreScratch scratch;
    BOOST_REQUIRE(prepare_from_sync(sync, scratch));

    const RadarCirConfig cfg = make_cfg(64);
    RadarCirResult out;
    BOOST_REQUIRE(radar_cir_one(rx.data(), rx.size(), sfd_truth, cfg, scratch,
                                out));
    BOOST_CHECK(out.status == RadarCirStatus::Ok);
    BOOST_CHECK_LE(std::llabs(out.sfd_start_sample - sfd_truth), 1);
    BOOST_CHECK_LE(std::llabs(out.preamble_start_sample - origin_truth), 1);
    BOOST_CHECK_EQUAL(out.cir_origin_sample, origin_truth);
    BOOST_CHECK_EQUAL(out.predicted_sfd_start, sfd_truth);
    BOOST_CHECK_EQUAL(out.tap_count, kCirPre + kCirPost);
    BOOST_CHECK_EQUAL(out.peak_tap, size_t(18));
    BOOST_CHECK(out.peak_tap != kCirPre);
    BOOST_CHECK_EQUAL(out.valid_repetitions, size_t(54));
    BOOST_CHECK_GT(out.peak_abs, 0.0f);
    BOOST_CHECK_GT(out.raw_l2_norm, 0.0f);

    std::vector<gr_complex> raw, nrm;
    copy_taps(scratch, out.tap_count, raw, nrm);
    BOOST_CHECK_LT(rel_l2(raw, raw_g), 1e-5);
    BOOST_CHECK_LT(rel_l2(nrm, nrm_g), 1e-5);
}

BOOST_AUTO_TEST_CASE(test_radar_cir_canonical_integer_delay_peak)
{
    const std::string json = load_canonical_metadata();

    double pred_d = -1.0;
    BOOST_REQUIRE(parse_nested_number(json, "rx_clean_998p4", "sfd_start",
                                      pred_d));
    const int64_t predicted = static_cast<int64_t>(std::llround(pred_d));
    double sfd_d = -1.0;
    BOOST_REQUIRE(parse_nested_number(json, "rx_delay_int_998p4", "sfd_start",
                                      sfd_d));
    const int64_t sfd_truth = static_cast<int64_t>(std::llround(sfd_d));
    double origin_d = -1.0;
    BOOST_REQUIRE(parse_nested_number(json, "rx_delay_int_998p4", "sync_origin",
                                      origin_d));
    const int64_t origin_truth = static_cast<int64_t>(std::llround(origin_d));
    BOOST_CHECK_EQUAL(origin_truth, int64_t(1997 + 37));
    BOOST_CHECK_EQUAL(sfd_truth, predicted + 37);

    std::vector<gr_complex> rx, tx, raw_g, nrm_g;
    BOOST_REQUIRE(
        load_cf32(testdata_path("uwb_radar/rx_delay_int_998p4.cf32"), rx));
    BOOST_REQUIRE(load_cf32(testdata_path("uwb_radar/tx_998p4.cf32"), tx));
    BOOST_REQUIRE(load_cf32(
        testdata_path("uwb_radar/cir_raw_delay_int_radar.cf32"), raw_g));
    BOOST_REQUIRE(load_cf32(
        testdata_path("uwb_radar/cir_norm_delay_int_radar.cf32"), nrm_g));
    std::vector<gr_complex> sync(
        tx.begin(),
        tx.begin() + static_cast<std::ptrdiff_t>(kQm35SamplesPerSymbol));
    RadarCirCoreScratch scratch;
    BOOST_REQUIRE(prepare_from_sync(sync, scratch));

    const RadarCirConfig cfg = make_cfg(64);
    RadarCirResult out;
    BOOST_REQUIRE(
        radar_cir_one(rx.data(), rx.size(), predicted, cfg, scratch, out));
    BOOST_CHECK(out.status == RadarCirStatus::Ok);
    BOOST_CHECK_LE(std::llabs(out.sfd_start_sample - sfd_truth), 1);
    BOOST_CHECK_LE(std::llabs(out.preamble_start_sample - origin_truth), 1);
    BOOST_CHECK_EQUAL(out.cir_origin_sample, int64_t(1997));
    BOOST_CHECK(out.cir_origin_sample != out.preamble_start_sample);
    BOOST_CHECK_EQUAL(out.predicted_sfd_start, predicted);
    BOOST_CHECK_EQUAL(out.peak_tap, size_t(18 + 37));
    BOOST_CHECK_EQUAL(out.peak_tap, size_t(55));
    BOOST_CHECK_EQUAL(out.tap_count, kCirPre + kCirPost);

    std::vector<gr_complex> raw, nrm;
    copy_taps(scratch, out.tap_count, raw, nrm);
    BOOST_CHECK_LT(rel_l2(raw, raw_g), 1e-5);
    BOOST_CHECK_LT(rel_l2(nrm, nrm_g), 1e-5);
}

BOOST_AUTO_TEST_CASE(test_radar_cir_canonical_fractional_delay)
{
    const std::string json = load_canonical_metadata();

    double pred_d = -1.0;
    BOOST_REQUIRE(parse_nested_number(json, "rx_clean_998p4", "sfd_start",
                                      pred_d));
    const int64_t predicted = static_cast<int64_t>(std::llround(pred_d));

    std::vector<gr_complex> rx, tx, raw_g, nrm_g;
    BOOST_REQUIRE(
        load_cf32(testdata_path("uwb_radar/rx_delay_frac_998p4.cf32"), rx));
    BOOST_REQUIRE(load_cf32(testdata_path("uwb_radar/tx_998p4.cf32"), tx));
    BOOST_REQUIRE(load_cf32(
        testdata_path("uwb_radar/cir_raw_delay_frac_radar.cf32"), raw_g));
    BOOST_REQUIRE(load_cf32(
        testdata_path("uwb_radar/cir_norm_delay_frac_radar.cf32"), nrm_g));
    std::vector<gr_complex> sync(
        tx.begin(),
        tx.begin() + static_cast<std::ptrdiff_t>(kQm35SamplesPerSymbol));
    RadarCirCoreScratch scratch;
    BOOST_REQUIRE(prepare_from_sync(sync, scratch));

    const RadarCirConfig cfg = make_cfg(64);
    RadarCirResult out;
    BOOST_REQUIRE(
        radar_cir_one(rx.data(), rx.size(), predicted, cfg, scratch, out));
    BOOST_CHECK(out.status == RadarCirStatus::Ok);
    BOOST_CHECK_EQUAL(out.cir_origin_sample, int64_t(1997));
    BOOST_CHECK_LE(std::llabs(static_cast<int64_t>(out.peak_tap) - int64_t(30)),
                   1);
    BOOST_CHECK_EQUAL(out.tap_count, kCirPre + kCirPost);

    std::vector<gr_complex> raw, nrm;
    copy_taps(scratch, out.tap_count, raw, nrm);
    BOOST_CHECK_LT(rel_l2(raw, raw_g), 1e-5);
    BOOST_CHECK_LT(rel_l2(nrm, nrm_g), 1e-5);
}

BOOST_AUTO_TEST_CASE(test_radar_cir_amplitude_linearity)
{
    const std::string json = load_canonical_metadata();
    double sfd_d = -1.0;
    BOOST_REQUIRE(parse_nested_number(json, "rx_clean_998p4", "sfd_start",
                                      sfd_d));
    const int64_t sfd_truth = static_cast<int64_t>(std::llround(sfd_d));

    std::vector<gr_complex> rx0, tx;
    BOOST_REQUIRE(load_cf32(testdata_path("uwb_radar/rx_clean_998p4.cf32"), rx0));
    BOOST_REQUIRE(load_cf32(testdata_path("uwb_radar/tx_998p4.cf32"), tx));
    std::vector<gr_complex> sync(
        tx.begin(),
        tx.begin() + static_cast<std::ptrdiff_t>(kQm35SamplesPerSymbol));
    RadarCirCoreScratch scratch;
    BOOST_REQUIRE(prepare_from_sync(sync, scratch));
    const RadarCirConfig cfg = make_cfg(64);

    RadarCirResult ref_out;
    BOOST_REQUIRE(radar_cir_one(rx0.data(), rx0.size(), sfd_truth, cfg, scratch,
                                ref_out));
    std::vector<gr_complex> raw_ref, nrm_ref;
    copy_taps(scratch, ref_out.tap_count, raw_ref, nrm_ref);

    const float scales[] = { 0.1f, 0.5f, 2.0f };
    for (float s : scales) {
        std::vector<gr_complex> rx = rx0;
        for (auto& v : rx)
            v *= s;
        RadarCirResult out;
        BOOST_REQUIRE(radar_cir_one(rx.data(), rx.size(), sfd_truth, cfg,
                                    scratch, out));
        BOOST_CHECK(out.status == RadarCirStatus::Ok);
        BOOST_CHECK_EQUAL(out.peak_tap, ref_out.peak_tap);
        BOOST_CHECK_EQUAL(out.tap_count, ref_out.tap_count);
        std::vector<gr_complex> raw, nrm;
        copy_taps(scratch, out.tap_count, raw, nrm);

        double raw_num = 0.0;
        double raw_den = 0.0;
        for (size_t i = 0; i < raw.size(); ++i) {
            const gr_complex e = raw[i] - s * raw_ref[i];
            raw_num += std::norm(e);
            raw_den += std::norm(s * raw_ref[i]);
        }
        BOOST_CHECK_LT(std::sqrt(raw_num / (raw_den + 1e-30)), 1e-5);
        BOOST_CHECK_LT(rel_l2(nrm, nrm_ref), 1e-5);
        BOOST_CHECK_CLOSE(out.raw_l2_norm, s * ref_out.raw_l2_norm, 1e-3);
    }
}

BOOST_AUTO_TEST_CASE(test_radar_cir_bit_stable_repeat)
{
    const std::string json = load_canonical_metadata();
    double sfd_d = -1.0;
    BOOST_REQUIRE(parse_nested_number(json, "rx_clean_998p4", "sfd_start",
                                      sfd_d));
    const int64_t sfd_truth = static_cast<int64_t>(std::llround(sfd_d));

    std::vector<gr_complex> rx, tx;
    BOOST_REQUIRE(load_cf32(testdata_path("uwb_radar/rx_clean_998p4.cf32"), rx));
    BOOST_REQUIRE(load_cf32(testdata_path("uwb_radar/tx_998p4.cf32"), tx));
    std::vector<gr_complex> sync(
        tx.begin(),
        tx.begin() + static_cast<std::ptrdiff_t>(kQm35SamplesPerSymbol));
    RadarCirCoreScratch scratch;
    BOOST_REQUIRE(prepare_from_sync(sync, scratch));
    const RadarCirConfig cfg = make_cfg(64);

    RadarCirResult a, b;
    BOOST_REQUIRE(radar_cir_one(rx.data(), rx.size(), sfd_truth, cfg, scratch,
                                a));
    std::vector<gr_complex> raw_a, nrm_a;
    copy_taps(scratch, a.tap_count, raw_a, nrm_a);
    BOOST_REQUIRE(radar_cir_one(rx.data(), rx.size(), sfd_truth, cfg, scratch,
                                b));
    std::vector<gr_complex> raw_b, nrm_b;
    copy_taps(scratch, b.tap_count, raw_b, nrm_b);

    BOOST_CHECK(a.status == b.status);
    BOOST_CHECK_EQUAL(a.sfd_start_sample, b.sfd_start_sample);
    BOOST_CHECK_EQUAL(a.preamble_start_sample, b.preamble_start_sample);
    BOOST_CHECK_EQUAL(a.cir_origin_sample, b.cir_origin_sample);
    BOOST_CHECK_EQUAL(a.predicted_sfd_start, b.predicted_sfd_start);
    BOOST_CHECK_EQUAL(a.tap_count, b.tap_count);
    BOOST_CHECK_EQUAL(a.peak_tap, b.peak_tap);
    BOOST_CHECK_EQUAL(a.valid_repetitions, b.valid_repetitions);
    BOOST_REQUIRE_EQUAL(raw_a.size(), raw_b.size());
    BOOST_CHECK(std::memcmp(raw_a.data(), raw_b.data(),
                            raw_a.size() * sizeof(gr_complex)) == 0);
    BOOST_CHECK(std::memcmp(nrm_a.data(), nrm_b.data(),
                            nrm_a.size() * sizeof(gr_complex)) == 0);
    BOOST_CHECK(std::memcmp(&a.sfd_metric, &b.sfd_metric, sizeof(float)) == 0);
    BOOST_CHECK(std::memcmp(&a.sync_metric, &b.sync_metric, sizeof(float)) ==
                0);
    BOOST_CHECK(std::memcmp(&a.peak_abs, &b.peak_abs, sizeof(float)) == 0);
    BOOST_CHECK(std::memcmp(&a.raw_l2_norm, &b.raw_l2_norm, sizeof(float)) ==
                0);
}

BOOST_AUTO_TEST_CASE(test_radar_cir_sync_reps_32_to_2048_synthetic)
{
    const auto sync = load_sync_pulse();
    const size_t ns[] = { 32, 64, 128, 256, 512, 1024, 2048 };
    for (size_t n_sync : ns) {
        auto pkt = make_sync_sfd_packet(sync, n_sync);
        BOOST_CHECK_EQUAL(pkt.sfd_start,
                          pkt.origin + static_cast<int64_t>(n_sync) *
                                           static_cast<int64_t>(1016));

        const size_t skip = kCirSkip;
        const size_t count = std::min(kCirReps, n_sync - skip);
        BOOST_REQUIRE_LT(skip, n_sync);
        const size_t wlen = kQm35SamplesPerSymbol + kCirPre + kCirPost - 1;
        const int64_t first_lo =
            pkt.origin + static_cast<int64_t>(skip) * 1016 -
            static_cast<int64_t>(kCirPre);
        const int64_t last_hi =
            pkt.origin +
            static_cast<int64_t>(skip + count - 1) * 1016 -
            static_cast<int64_t>(kCirPre) + static_cast<int64_t>(wlen) - 1;
        BOOST_CHECK_GE(first_lo, int64_t(0));
        BOOST_CHECK_LT(static_cast<size_t>(last_hi), pkt.rx.size());

        const RadarCirConfig cfg = make_cfg(n_sync);
        RadarCirCoreScratch scratch;
        BOOST_REQUIRE(prepare_from_sync(sync, scratch, cfg));
        RadarCirResult out;
        BOOST_REQUIRE(radar_cir_one(pkt.rx.data(), pkt.rx.size(), pkt.sfd_start,
                                    cfg, scratch, out));
        BOOST_CHECK(out.status == RadarCirStatus::Ok);
        BOOST_CHECK_EQUAL(out.sfd_start_sample, pkt.sfd_start);
        BOOST_CHECK_EQUAL(out.preamble_start_sample, pkt.origin);
        BOOST_CHECK_EQUAL(out.cir_origin_sample, pkt.origin);
        BOOST_CHECK_EQUAL(out.tap_count, kCirPre + kCirPost);
        BOOST_CHECK_EQUAL(out.valid_repetitions, count);
        BOOST_CHECK_EQUAL(out.valid_repetitions,
                          std::min(kCirReps, n_sync - skip));
    }
}

BOOST_AUTO_TEST_CASE(test_radar_cir_prepare_rejects_bad_sync)
{
    const auto sync = load_sync_pulse();
    const RadarCirConfig cfg = make_cfg(64);
    RadarCirCoreScratch scratch;

    BOOST_REQUIRE(!prepare_radar_cir_core(cfg, nullptr, sync.size(), kCirPre,
                                          kCirPost, scratch));
    BOOST_CHECK(scratch.sync_template.empty());
    BOOST_CHECK(!scratch.prepared.ready);

    std::vector<gr_complex> zero(sync.size(), gr_complex(0.0f, 0.0f));
    BOOST_REQUIRE(!prepare_radar_cir_core(cfg, zero.data(), zero.size(),
                                          kCirPre, kCirPost, scratch));
    BOOST_CHECK(scratch.sync_template.empty());

    std::vector<gr_complex> nan_sync = sync;
    nan_sync[0] = gr_complex(std::numeric_limits<float>::quiet_NaN(), 0.0f);
    BOOST_REQUIRE(!prepare_radar_cir_core(cfg, nan_sync.data(), nan_sync.size(),
                                          kCirPre, kCirPost, scratch));
    BOOST_CHECK(scratch.sync_template.empty());
    BOOST_CHECK(!scratch.prepared.ready);
}

BOOST_AUTO_TEST_CASE(test_radar_cir_profile_mismatch_invalid_input)
{
    const auto sync = load_sync_pulse();
    auto pkt = make_sync_sfd_packet(sync, 64);
    const RadarCirConfig cfg64 = make_cfg(64);
    RadarCirCoreScratch scratch;
    BOOST_REQUIRE(prepare_from_sync(sync, scratch, cfg64));

    auto expect_mismatch = [&](RadarCirConfig bad, const char* label) {
        RadarCirResult out;
        const bool ok = radar_cir_one(pkt.rx.data(), pkt.rx.size(),
                                      pkt.sfd_start, bad, scratch, out);
        BOOST_REQUIRE_MESSAGE(!ok, label);
        BOOST_CHECK_MESSAGE(out.status == RadarCirStatus::InvalidInput, label);
        BOOST_CHECK_EQUAL(out.tap_count, size_t(0));
        BOOST_CHECK_EQUAL(out.sfd_start_sample, int64_t(-1));
        BOOST_CHECK_EQUAL(out.preamble_start_sample, int64_t(-1));
        BOOST_CHECK_EQUAL(out.cir_origin_sample, int64_t(-1));
    };

    RadarCirConfig ieee = cfg64;
    ieee.sfd_mode = "ieee";
    expect_mismatch(ieee, "sfd_mode ieee vs prepared 4z2");

    RadarCirConfig code10 = cfg64;
    code10.code_index = 10;
    expect_mismatch(code10, "code_index 10 vs prepared 9");

    RadarCirConfig n32 = cfg64;
    n32.sync_repetitions = 32;
    expect_mismatch(n32, "sync_repetitions 32 vs prepared 64");

    RadarCirConfig n128 = cfg64;
    n128.sync_repetitions = 128;
    expect_mismatch(n128, "sync_repetitions 128 vs prepared 64");

    RadarCirConfig sps = cfg64;
    sps.samples_per_symbol = 1015;
    expect_mismatch(sps, "samples_per_symbol mismatch");

    RadarCirConfig pid = cfg64;
    pid.tx_profile_id = 7;
    expect_mismatch(pid, "tx_profile_id mismatch");
}

BOOST_AUTO_TEST_CASE(test_radar_cir_canonical_64_rejects_32_config)
{
    const std::string json = load_canonical_metadata();
    double sfd_d = -1.0;
    BOOST_REQUIRE(parse_nested_number(json, "rx_clean_998p4", "sfd_start",
                                      sfd_d));
    const int64_t predicted = static_cast<int64_t>(std::llround(sfd_d));
    std::vector<gr_complex> rx, tx;
    BOOST_REQUIRE(load_cf32(testdata_path("uwb_radar/rx_clean_998p4.cf32"), rx));
    BOOST_REQUIRE(load_cf32(testdata_path("uwb_radar/tx_998p4.cf32"), tx));
    std::vector<gr_complex> sync(
        tx.begin(),
        tx.begin() + static_cast<std::ptrdiff_t>(kQm35SamplesPerSymbol));

    RadarCirConfig cfg64 = make_cfg(64);
    RadarCirCoreScratch scratch;
    BOOST_REQUIRE(prepare_from_sync(sync, scratch, cfg64));

    RadarCirConfig cfg32 = make_cfg(32);
    RadarCirResult out;
    BOOST_REQUIRE(!radar_cir_one(rx.data(), rx.size(), predicted, cfg32,
                                 scratch, out));
    BOOST_CHECK(out.status == RadarCirStatus::InvalidInput);
    BOOST_CHECK_EQUAL(out.tap_count, size_t(0));
    BOOST_CHECK_EQUAL(out.preamble_start_sample, int64_t(-1));
    BOOST_CHECK(out.cir_origin_sample != int64_t(34509));
}

bool parse_json_number(const std::string& json, const char* key, double& out)
{
    const std::string pat = std::string("\"") + key + "\"";
    auto pos = json.find(pat);
    if (pos == std::string::npos)
        return false;
    pos = json.find(':', pos + pat.size());
    if (pos == std::string::npos)
        return false;
    try {
        size_t idx = 0;
        out = std::stod(json.substr(pos + 1), &idx);
        return idx > 0;
    } catch (...) {
        return false;
    }
}

BOOST_AUTO_TEST_CASE(test_radar_cir_complete_packets_32_64_128)
{
    struct Case {
        const char* rel;
        size_t n_sync;
        const char* generator;
    };
    const Case cases[] = {
        { "uwb_radar", 64, "export_uwb_radar_golden.m" },
        { "uwb_radar/packets/sync32", 32, "export_uwb_radar_packet.m" },
        { "uwb_radar/packets/sync128", 128, "export_uwb_radar_packet.m" },
    };

    for (const auto& c : cases) {
        const std::string meta_path =
            testdata_path(std::string(c.rel) + "/metadata.json");
        std::ifstream meta_f(meta_path);
        BOOST_REQUIRE_MESSAGE(meta_f.good(),
                              "missing complete packet metadata " + meta_path);
        std::ostringstream oss;
        oss << meta_f.rdbuf();
        const std::string json = oss.str();
        std::string generator;
        BOOST_REQUIRE(parse_json_string(json, "generator", generator));
        BOOST_REQUIRE_MESSAGE(generator.find(c.generator) != std::string::npos,
                              "generator for " + std::string(c.rel) + " is " +
                                  generator);

        double v = 0.0;
        BOOST_REQUIRE(parse_json_number(json, "sync_repetitions", v));
        BOOST_CHECK_EQUAL(static_cast<size_t>(std::llround(v)), c.n_sync);
        BOOST_REQUIRE(parse_json_number(json, "tx_length_998p4", v));
        const size_t tx_len = static_cast<size_t>(std::llround(v));
        BOOST_REQUIRE(parse_json_number(json, "tx_length_737p28", v));
        const size_t native_len = static_cast<size_t>(std::llround(v));
        BOOST_CHECK_GT(tx_len, c.n_sync * kQm35SamplesPerSymbol);
        BOOST_CHECK_GT(native_len, size_t(0));
        BOOST_REQUIRE(parse_nested_number(json, "resample", "interp", v));
        BOOST_CHECK_EQUAL(static_cast<int>(std::llround(v)), 48);
        BOOST_REQUIRE(parse_nested_number(json, "resample", "decim", v));
        BOOST_CHECK_EQUAL(static_cast<int>(std::llround(v)), 65);
        std::vector<gr_complex> native;
        BOOST_REQUIRE(load_cf32(
            testdata_path(std::string(c.rel) + "/tx_737p28.cf32"), native));
        BOOST_CHECK_EQUAL(native.size(), native_len);

        BOOST_REQUIRE(parse_nested_number(json, "rx_clean_998p4", "sfd_start",
                                          v));
        const int64_t sfd = static_cast<int64_t>(std::llround(v));
        BOOST_REQUIRE(parse_nested_number(json, "rx_clean_998p4", "sync_origin",
                                          v));
        const int64_t origin = static_cast<int64_t>(std::llround(v));
        BOOST_CHECK_EQUAL(sfd, origin + static_cast<int64_t>(c.n_sync) *
                                            int64_t(1016));

        std::vector<gr_complex> tx, rx, raw_g, nrm_g;
        BOOST_REQUIRE(load_cf32(
            testdata_path(std::string(c.rel) + "/tx_998p4.cf32"), tx));
        BOOST_REQUIRE(load_cf32(
            testdata_path(std::string(c.rel) + "/rx_clean_998p4.cf32"), rx));
        BOOST_REQUIRE(load_cf32(
            testdata_path(std::string(c.rel) + "/cir_raw_clean_radar.cf32"),
            raw_g));
        BOOST_REQUIRE(load_cf32(
            testdata_path(std::string(c.rel) + "/cir_norm_clean_radar.cf32"),
            nrm_g));
        BOOST_REQUIRE_EQUAL(tx.size(), tx_len);
        BOOST_REQUIRE_GE(tx.size(), kQm35SamplesPerSymbol);
        BOOST_REQUIRE_EQUAL(raw_g.size(), kCirPre + kCirPost);
        BOOST_REQUIRE_EQUAL(nrm_g.size(), kCirPre + kCirPost);

        const size_t skip = kCirSkip;
        const size_t count = std::min(kCirReps, c.n_sync - skip);
        const size_t wlen = kQm35SamplesPerSymbol + kCirPre + kCirPost - 1;
        const int64_t first_lo =
            origin + static_cast<int64_t>(skip) * 1016 -
            static_cast<int64_t>(kCirPre);
        const int64_t last_hi =
            origin + static_cast<int64_t>(skip + count - 1) * 1016 -
            static_cast<int64_t>(kCirPre) + static_cast<int64_t>(wlen) - 1;
        BOOST_CHECK_GE(first_lo, int64_t(0));
        BOOST_CHECK_LT(static_cast<size_t>(last_hi), rx.size());

        std::vector<gr_complex> sync(
            tx.begin(),
            tx.begin() + static_cast<std::ptrdiff_t>(kQm35SamplesPerSymbol));
        RadarCirConfig cfg = make_cfg(c.n_sync);
        RadarCirCoreScratch scratch;
        BOOST_REQUIRE(prepare_radar_cir_core(cfg, sync.data(), sync.size(),
                                             kCirPre, kCirPost, scratch));
        RadarCirResult out;
        BOOST_REQUIRE(radar_cir_one(rx.data(), rx.size(), sfd, cfg, scratch,
                                    out));
        BOOST_CHECK(out.status == RadarCirStatus::Ok);
        BOOST_CHECK_LE(std::llabs(out.sfd_start_sample - sfd), 1);
        BOOST_CHECK_LE(std::llabs(out.preamble_start_sample - origin), 1);
        BOOST_CHECK_EQUAL(out.cir_origin_sample, origin);
        BOOST_CHECK_EQUAL(out.tap_count, kCirPre + kCirPost);
        BOOST_CHECK_EQUAL(out.valid_repetitions, count);
        std::vector<gr_complex> raw, nrm;
        copy_taps(scratch, out.tap_count, raw, nrm);
        const double e_raw = rel_l2(raw, raw_g);
        const double e_nrm = rel_l2(nrm, nrm_g);
        BOOST_CHECK_MESSAGE(e_raw < 1e-5,
                            "n_sync=" + std::to_string(c.n_sync) +
                                " raw L2=" + std::to_string(e_raw));
        BOOST_CHECK_MESSAGE(e_nrm < 1e-5,
                            "n_sync=" + std::to_string(c.n_sync) +
                                " norm L2=" + std::to_string(e_nrm));
    }
}
