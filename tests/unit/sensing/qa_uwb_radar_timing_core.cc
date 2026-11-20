/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * QA for header-only radar SYNC origin backtrack (998.4 MS/s).
 *
 * Synthetic construction uses testdata/reference_preamble.bin (code-9 SYNC).
 * Canonical MATLAB golden in testdata/uwb_radar/ is required
 * (generator=export_uwb_radar_golden.m).
 */

#include <boost/test/unit_test.hpp>

#include <gnuradio/uwb/uwb_gr_complex.h>
#include <gnuradio/uwb/uwb_demod_core.h>
#include <gnuradio/uwb/uwb_phy_profile.h>
#include <gnuradio/uwb/uwb_radar_timing_core.h>

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <random>
#include <sstream>
#include <string>
#include <vector>

namespace {

using openuwb::demod::kQm35SamplesPerSymbol;
using openuwb::demod::Qm35825Profile;
using openuwb::radar::nominal_preamble_start;
using openuwb::radar::RadarTimingResult;
using openuwb::radar::refine_sync_origin;
using openuwb::radar::TimingStatus;
using openuwb::demod::SyncStrategy;
using openuwb::demod::TimingResult;
using openuwb::demod::core::DemodScratch;
using openuwb::demod::core::stage_timing;

#ifndef UWB_TESTDATA_DIR
#define UWB_TESTDATA_DIR "../../../testdata"
#endif

constexpr float kTimingThreshold = Qm35825Profile{}.sfd_detection_threshold;
constexpr int64_t kDefaultMargin = 8; // demod timing_track_radius
constexpr int64_t kWideMargin = 64;
constexpr int64_t kPre = 256;
constexpr int64_t kTail = 256;
constexpr size_t kSynthSyncReps = 4;

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

std::vector<gr_complex> load_sync_template()
{
    std::vector<gr_complex> tmpl;
    const std::string path = testdata_path("reference_preamble.bin");
    BOOST_REQUIRE_MESSAGE(load_cf32(path, tmpl),
                          "cannot load reference_preamble.bin from " + path);
    BOOST_REQUIRE_EQUAL(tmpl.size(), kQm35SamplesPerSymbol);
    openuwb::core::uwb_l2_normalize(tmpl);
    return tmpl;
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

bool parse_json_number(const std::string& json,
                       const char* key,
                       double& out)
{
    const std::string pat = std::string("\"") + key + "\"";
    auto pos = json.find(pat);
    if (pos == std::string::npos)
        return false;
    pos = json.find(':', pos + pat.size());
    if (pos == std::string::npos)
        return false;
    ++pos;
    while (pos < json.size() &&
           (json[pos] == ' ' || json[pos] == '\t' || json[pos] == '\n'))
        ++pos;
    try {
        size_t idx = 0;
        out = std::stod(json.substr(pos), &idx);
        return idx > 0;
    } catch (...) {
        return false;
    }
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

struct SynthFrame {
    std::vector<gr_complex> rx;
    int64_t origin = 0;
    int64_t sfd_start = 0;
    size_t n_sync = 0;
};

SynthFrame make_sync_frame(const std::vector<gr_complex>& sync,
                           size_t n_sync = kSynthSyncReps)
{
    SynthFrame f;
    f.n_sync = n_sync;
    f.origin = kPre;
    f.sfd_start = f.origin + static_cast<int64_t>(n_sync) *
                                 static_cast<int64_t>(kQm35SamplesPerSymbol);
    f.rx.assign(static_cast<size_t>(f.sfd_start) +
                    kQm35SamplesPerSymbol + static_cast<size_t>(kTail),
                gr_complex(0.0f, 0.0f));
    for (size_t i = 0; i < n_sync; ++i)
        place_waveform(f.rx,
                       f.origin + static_cast<int64_t>(i) *
                                      static_cast<int64_t>(kQm35SamplesPerSymbol),
                       sync);
    return f;
}

void assert_refine_window(const RadarTimingResult& out, int64_t expected_n)
{
    BOOST_CHECK_EQUAL(out.refine_correlations,
                      static_cast<uint32_t>(expected_n));
    if (out.refine_hi >= out.refine_lo)
        BOOST_CHECK_EQUAL(out.refine_hi - out.refine_lo + 1, expected_n);
}

void assert_exhaustive_offsets(const std::vector<gr_complex>& rx,
                               int64_t true_origin,
                               int64_t true_sfd,
                               size_t n_sync,
                               int64_t margin,
                               float threshold,
                               const std::vector<gr_complex>& tmpl)
{
    int misses = 0;
    int64_t first_miss = 0;
    for (int64_t off = -margin; off <= margin; ++off) {
        const int64_t sfd = true_sfd + off;
        if (sfd < 0)
            continue;
        RadarTimingResult out;
        const bool ok =
            refine_sync_origin(rx.data(), rx.size(), sfd, n_sync,
                               kQm35SamplesPerSymbol, margin, threshold,
                               tmpl.data(), tmpl.size(), out);
        const bool hit =
            ok && out.status == TimingStatus::Ok &&
            std::llabs(out.preamble_start_sample - true_origin) <= 1;
        if (!hit) {
            if (misses == 0)
                first_miss = off;
            ++misses;
        }
    }
    BOOST_CHECK_MESSAGE(
        misses == 0,
        "exhaustive SYNC refine missed " + std::to_string(misses) +
            " offsets in [-" + std::to_string(margin) + ",+" +
            std::to_string(margin) + "]; first miss off=" +
            std::to_string(first_miss));
}

} // namespace

// ---------------------------------------------------------------------------
// Low-SNR acquisition QA: PAR fallback in the narrow seed searches and the
// on-demand K=64 periodic-fold fallback of `demod::core::stage_timing`.
//
// Fixtures are 256-SYNC preambles tiled from testdata/reference_preamble.bin on
// the 998.4 MS/s WORK grid (where one SYNC is exactly 1016 samples), plus white
// AWGN scaled to an IN-BAND SNR over the 500 MHz UWB channel inside the
// 998.4 MHz work band.  That is the same convention as
// tools/rx/proto_low_snr_signal.py, so these numbers are directly comparable
// with docs/performance/data_low_snr_lock_20261117/fold_timing.json.
// ---------------------------------------------------------------------------
namespace {

constexpr size_t kLowSnrSyncReps = 256;
constexpr double kWorkBandHz = 998.4e6;
constexpr double kInBandHz = 500.0e6;

struct LowSnrFrame {
    std::vector<gr_complex> rx;
    int64_t start = 0;
    size_t reps = 0;
};

LowSnrFrame make_low_snr_frame(const std::vector<gr_complex>& sync,
                               size_t reps,
                               int64_t pre,
                               int64_t tail,
                               double snr_db,
                               uint32_t seed)
{
    LowSnrFrame f;
    f.reps = reps;
    f.start = pre;
    const size_t sym = kQm35SamplesPerSymbol;
    f.rx.assign(static_cast<size_t>(pre) + reps * sym +
                    static_cast<size_t>(tail),
                gr_complex(0.0f, 0.0f));
    for (size_t i = 0; i < reps; ++i)
        place_waveform(f.rx, pre + static_cast<int64_t>(i * sym), sync);

    double p_sig = 0.0;
    for (size_t i = 0; i < sym; ++i)
        p_sig += std::norm(sync[i]);
    p_sig /= static_cast<double>(sym);
    const double snr_lin = std::pow(10.0, snr_db / 10.0);
    const double p_noise = p_sig * (kWorkBandHz / kInBandHz) / snr_lin;
    const float sigma = static_cast<float>(std::sqrt(p_noise / 2.0));
    std::mt19937 rng(seed);
    std::normal_distribution<float> g(0.0f, 1.0f);
    for (auto& v : f.rx)
        v += gr_complex(sigma * g(rng), sigma * g(rng));
    return f;
}

Qm35825Profile low_snr_profile(size_t margin, size_t narrow_radius)
{
    auto p = Qm35825Profile::Default();
    p.preamble_repetitions = kLowSnrSyncReps;
    p.sync_strategy = SyncStrategy::AcquireLegacy;
    p.timing_search_margin = margin;
    p.timing_seed_refine_radius = narrow_radius;
    return p;
}

std::vector<gr_complex> load_normalized_sync()
{
    std::vector<gr_complex> raw;
    BOOST_REQUIRE(load_cf32(testdata_path("reference_preamble.bin"), raw));
    BOOST_REQUIRE_EQUAL(raw.size(), kQm35SamplesPerSymbol);
    auto tmpl = raw;
    openuwb::core::uwb_l2_normalize(tmpl);
    return tmpl;
}

} // namespace

BOOST_AUTO_TEST_CASE(test_low_snr_par_fallback_acquires_at_minus_4_and_8_db)
{
    // Task 1: the two NARROW legacy seed searches accept a sub-threshold SYNC
    // peak through the PAR gate.  At -4/-8 dB in-band SNR every SYNC peak sits
    // above 0.08 with PAR > 200 but only a handful of 256 clear the 0.25
    // absolute threshold, so the absolute gate alone loses the packet.
    std::vector<gr_complex> raw;
    BOOST_REQUIRE(load_cf32(testdata_path("reference_preamble.bin"), raw));
    const auto tmpl = load_normalized_sync();

    const double snrs[] = { -4.0, -8.0 };
    for (double snr : snrs) {
        auto frame = make_low_snr_frame(raw, kLowSnrSyncReps, 8192, 2048, snr,
                                        20261117u);
        // Narrow radius 128 < the legacy 256 so the narrow path actually runs;
        // the margin is kept small so the wide coarse scan cannot mask the
        // narrow search's behaviour.
        auto prof = low_snr_profile(/*margin=*/2048, /*narrow_radius=*/128);
        DemodScratch scratch;
        scratch.reserve(frame.rx.size());
        TimingResult tr;
        const bool ok = stage_timing(frame.rx.data(), frame.rx.size(), prof,
                                     tmpl, frame.start, tr, scratch);
        std::printf("low-snr PAR: snr=%.0f dB ok=%d metric=%.4f narrow=%u/%u "
                    "start_err=%lld peaks=%zu ps=%zu\n",
                    snr, ok ? 1 : 0, static_cast<double>(tr.metric),
                    tr.seed_refine_narrow_success,
                    tr.seed_refine_narrow_attempts,
                    static_cast<long long>(tr.preamble_start_sample -
                                           frame.start),
                    tr.detected_peaks, tr.peak_samples.size());
        BOOST_REQUIRE_MESSAGE(ok, "stage_timing failed at " << snr
                                                            << " dB in-band");
        BOOST_CHECK_EQUAL(tr.seed_refine_narrow_attempts, 1u);
        BOOST_CHECK_EQUAL(tr.seed_refine_narrow_success, 1u);
        BOOST_CHECK_EQUAL(tr.seed_refine_fallbacks, 0u);
        // The accepted peak is BELOW the absolute threshold: the PAR gate, not
        // the absolute gate, is what acquired it.
        BOOST_CHECK_LT(tr.metric, prof.radar_verification_threshold);
        BOOST_CHECK_LE(std::llabs(tr.preamble_start_sample - frame.start), 2);
        // A sub-threshold train can lose a few SYNCs to the 0.06 floor; what
        // matters is that it is long enough for fit_cfo and the CIR average.
        BOOST_CHECK_GE(tr.detected_peaks, size_t(200));
    }
}

BOOST_AUTO_TEST_CASE(test_low_snr_fold_fallback_acquires_at_minus_12_and_14_db)
{
    // Task 2/3: at -12/-14 dB in-band SNR even the PAR gate cannot help (the
    // per-SYNC metric falls below the 0.06 PAR floor), so acquisition depends
    // on the K=64 periodic fold.
    std::vector<gr_complex> raw;
    BOOST_REQUIRE(load_cf32(testdata_path("reference_preamble.bin"), raw));
    const auto tmpl = load_normalized_sync();

    constexpr int kFrames = 50;
    constexpr int kMinHits = 48;
    // The live frontend seed is accurate to a few SYMBOLS, not samples
    // (measured -6.5..+4.4 SYNC periods at -14 dB in-band on the native
    // fixture).  Sweep that error so the test exercises the leading-edge
    // disambiguation instead of handing the fold an oracle seed.
    const int64_t seed_offsets[] = { 0, -5, -3, -1, 2, 4, 6, -6 };
    const double snrs[] = { -12.0, -14.0 };
    for (double snr : snrs) {
        auto prof = low_snr_profile(/*margin=*/8192, /*narrow_radius=*/0);
        int hits = 0;
        int64_t worst_err = 0;
        float best_metric = 0.0f;
        for (int f = 0; f < kFrames; ++f) {
            auto frame = make_low_snr_frame(raw, kLowSnrSyncReps, 8192, 2048,
                                            snr, 1000u + static_cast<uint32_t>(f));
            const int64_t off =
                seed_offsets[f % (sizeof(seed_offsets) / sizeof(int64_t))] *
                static_cast<int64_t>(kQm35SamplesPerSymbol);
            DemodScratch scratch;
            scratch.reserve(frame.rx.size());
            TimingResult tr;
            if (!stage_timing(frame.rx.data(), frame.rx.size(), prof, tmpl,
                              frame.start + off, tr, scratch))
                continue;
            BOOST_CHECK(tr.sync_peaks_folded);
            BOOST_CHECK_EQUAL(tr.detected_peaks, tr.peak_samples.size());
            BOOST_CHECK_LE(tr.detected_peaks, 16u);
            BOOST_CHECK_EQUAL(tr.cir_grid_repetitions, kLowSnrSyncReps);
            const int64_t err =
                std::llabs(tr.preamble_start_sample - frame.start);
            worst_err = std::max(worst_err, err);
            best_metric = std::max(best_metric, tr.metric);
            if (err <= 2)
                ++hits;
        }
        std::printf("low-snr fold: snr=%.0f dB acquired %d/%d, worst "
                    "|start err|=%lld, best metric=%.4f\n",
                    snr, hits, kFrames, static_cast<long long>(worst_err),
                    static_cast<double>(best_metric));
        BOOST_CHECK_GE(hits, kMinHits);
    }
}

BOOST_AUTO_TEST_CASE(test_low_snr_fold_resolves_sparse_anchor_cfo_alias)
{
    std::vector<gr_complex> raw;
    BOOST_REQUIRE(load_cf32(testdata_path("reference_preamble.bin"), raw));
    const auto tmpl = load_normalized_sync();
    const auto profile = low_snr_profile(8192, 0);
    for (double cfo_hz : {-40000.0, 40000.0}) {
        auto frame = make_low_snr_frame(raw, kLowSnrSyncReps, 8192, 2048,
                                        -12.0, 14231u);
        for (size_t i = 0; i < frame.rx.size(); ++i)
            frame.rx[i] *= std::polar(1.0f, static_cast<float>(
                2.0 * M_PI * cfo_hz * i / kWorkBandHz));
        DemodScratch scratch;
        TimingResult timing;
        BOOST_REQUIRE(stage_timing(frame.rx.data(), frame.rx.size(), profile, tmpl,
                                   frame.start + 2 * kQm35SamplesPerSymbol,
                                   timing, scratch));
        BOOST_CHECK(timing.sync_peaks_folded);
        BOOST_CHECK_LE(std::llabs(timing.preamble_start_sample - frame.start), 2);
        openuwb::demod::CfoResult cfo;
        BOOST_REQUIRE(openuwb::demod::core::fit_cfo(
            frame.rx.data(), frame.rx.size(), profile, timing, cfo, scratch));
        BOOST_CHECK_SMALL(cfo.cfo_hz - cfo_hz, 150.0);
    }
}

BOOST_AUTO_TEST_CASE(test_low_snr_fold_fallback_rejects_pure_noise)
{
    // Task 3: the 0.05 folded threshold must not fire on noise.  Measured
    // reference: the maximum folded metric over 200 pure-noise positions is
    // 0.0107, so 0.05 leaves ~5x of margin; this test is the production-side
    // guard that the margin actually holds for the full search window.
    const auto tmpl = load_normalized_sync();
    auto prof = low_snr_profile(/*margin=*/1024, /*narrow_radius=*/0);

    constexpr int kWindows = 100;
    int false_alarms = 0;
    float worst_metric = 0.0f;
    for (int w = 0; w < kWindows; ++w) {
        std::vector<gr_complex> noise(200000);
        std::mt19937 rng(777000u + static_cast<uint32_t>(w));
        std::normal_distribution<float> g(0.0f, 1.0f);
        for (auto& v : noise)
            v = gr_complex(g(rng), g(rng));
        DemodScratch scratch;
        scratch.reserve(noise.size());
        TimingResult tr;
        if (stage_timing(noise.data(), noise.size(), prof, tmpl, 8192, tr,
                         scratch)) {
            ++false_alarms;
            worst_metric = std::max(worst_metric, tr.metric);
        }
    }
    std::printf("low-snr fold noise: %d/%d false alarms (worst metric %.4f)\n",
                false_alarms, kWindows, static_cast<double>(worst_metric));
    BOOST_CHECK_EQUAL(false_alarms, 0);
}

BOOST_AUTO_TEST_CASE(test_low_snr_fold_fallback_not_used_at_high_snr)
{
    // Task 4: the fold is a FALLBACK.  At high SNR the legacy per-SYNC train
    // must still produce the full peak train (256 peaks), not the 16
    // super-anchors the fold path reports, and the origin must be exact.
    std::vector<gr_complex> raw;
    BOOST_REQUIRE(load_cf32(testdata_path("reference_preamble.bin"), raw));
    const auto tmpl = load_normalized_sync();

    auto frame = make_low_snr_frame(raw, kLowSnrSyncReps, 8192, 2048, 20.0,
                                    4242u);
    auto prof = low_snr_profile(/*margin=*/8192, /*narrow_radius=*/0);
    DemodScratch scratch;
    scratch.reserve(frame.rx.size());
    TimingResult tr;
    BOOST_REQUIRE(stage_timing(frame.rx.data(), frame.rx.size(), prof, tmpl,
                               frame.start, tr, scratch));
    std::printf("high-snr: start_err=%lld peaks=%zu period=%.6f metric=%.4f\n",
                static_cast<long long>(tr.preamble_start_sample - frame.start),
                tr.detected_peaks, tr.measured_period,
                static_cast<double>(tr.metric));
    BOOST_CHECK_EQUAL(tr.preamble_start_sample, frame.start);
    BOOST_CHECK_EQUAL(tr.detected_peaks, kLowSnrSyncReps);
    // 256 per-SYNC peaks == the legacy train ran; the fold path would leave 16.
    BOOST_CHECK_EQUAL(tr.peak_samples.size(), kLowSnrSyncReps);
    BOOST_CHECK_SMALL(std::abs(tr.measured_period - 1016.0), 0.05);
    BOOST_CHECK_GE(tr.metric, 0.9f);
    BOOST_CHECK_EQUAL(tr.backtrack_symbols, 0u);
}

BOOST_AUTO_TEST_CASE(test_radar_timing_nominal_arithmetic)
{
    const size_t sps = kQm35SamplesPerSymbol;
    BOOST_CHECK_EQUAL(sps, size_t(1016));

    const int64_t sfd = 67021;
    BOOST_CHECK_EQUAL(nominal_preamble_start(sfd, 64, sps), int64_t(1997));
    BOOST_CHECK_EQUAL(sfd - int64_t(64) * int64_t(sps), int64_t(1997));
    BOOST_CHECK_EQUAL(nominal_preamble_start(sfd, 32, sps),
                      sfd - int64_t(32) * int64_t(sps));
    BOOST_CHECK_EQUAL(nominal_preamble_start(sfd, 32, sps), int64_t(34509));
    BOOST_CHECK_EQUAL(nominal_preamble_start(sfd, 128, sps),
                      sfd - int64_t(128) * int64_t(sps));
    BOOST_CHECK_EQUAL(nominal_preamble_start(sfd, 128, sps), int64_t(-63027));

    BOOST_CHECK_EQUAL(nominal_preamble_start(-1, 64, sps), int64_t(-1));
    BOOST_CHECK_EQUAL(nominal_preamble_start(sfd, 0, sps), int64_t(-1));
    BOOST_CHECK_EQUAL(nominal_preamble_start(sfd, 64, 0), int64_t(-1));

    const size_t huge = static_cast<size_t>(std::numeric_limits<int64_t>::max());
    BOOST_CHECK_EQUAL(nominal_preamble_start(sfd, huge, huge), int64_t(-1));
    int64_t prod = 0;
    BOOST_CHECK(!openuwb::radar::radar_i64_mul(
        std::numeric_limits<int64_t>::max(), 2, prod));
    int64_t sum = 0;
    BOOST_CHECK(!openuwb::radar::radar_i64_add(
        std::numeric_limits<int64_t>::max(), 1, sum));
}

BOOST_AUTO_TEST_CASE(test_radar_timing_extreme_coordinate_invalid_input)
{
    const auto tmpl = load_sync_template();
    std::vector<gr_complex> rx(tmpl.size(), gr_complex(0.0f, 0.0f));
    RadarTimingResult out;

    // nominal = INT64_MAX - 1016; adding this margin would overflow the
    // unclipped refine window. It must be rejected before any scan occurs.
    BOOST_CHECK(!refine_sync_origin(
        rx.data(), rx.size(), std::numeric_limits<int64_t>::max(), 1,
        kQm35SamplesPerSymbol, 2048, kTimingThreshold, tmpl.data(),
        tmpl.size(), out));
    BOOST_CHECK(out.status == TimingStatus::InvalidInput);
    BOOST_CHECK_EQUAL(out.preamble_start_sample, int64_t(-1));
}

BOOST_AUTO_TEST_CASE(test_radar_timing_synthetic_noiseless)
{
    const auto raw = [&]() {
        std::vector<gr_complex> t;
        const std::string path = testdata_path("reference_preamble.bin");
        BOOST_REQUIRE(load_cf32(path, t));
        return t;
    }();
    auto tmpl = raw;
    openuwb::core::uwb_l2_normalize(tmpl);
    BOOST_REQUIRE_EQUAL(tmpl.size(), kQm35SamplesPerSymbol);

    auto frame = make_sync_frame(raw, kSynthSyncReps);

    RadarTimingResult out0;
    BOOST_REQUIRE(refine_sync_origin(
        frame.rx.data(), frame.rx.size(), frame.sfd_start, kSynthSyncReps,
        kQm35SamplesPerSymbol, /*margin=*/0, kTimingThreshold, tmpl.data(),
        tmpl.size(), out0));
    BOOST_CHECK(out0.status == TimingStatus::Ok);
    BOOST_CHECK_EQUAL(out0.preamble_start_sample, frame.origin);
    BOOST_CHECK_EQUAL(out0.nominal_preamble_start, frame.origin);
    BOOST_CHECK_EQUAL(out0.sfd_start_sample, frame.sfd_start);
    BOOST_CHECK_EQUAL(out0.refine_lo, frame.origin);
    BOOST_CHECK_EQUAL(out0.refine_hi, frame.origin);
    BOOST_CHECK_GE(out0.metric, 0.95f);
    assert_refine_window(out0, 1);

    RadarTimingResult out8;
    BOOST_REQUIRE(refine_sync_origin(
        frame.rx.data(), frame.rx.size(), frame.sfd_start, kSynthSyncReps,
        kQm35SamplesPerSymbol, kDefaultMargin, kTimingThreshold, tmpl.data(),
        tmpl.size(), out8));
    BOOST_CHECK_EQUAL(out8.preamble_start_sample, frame.origin);
    BOOST_CHECK_EQUAL(out8.refine_lo, frame.origin - kDefaultMargin);
    BOOST_CHECK_EQUAL(out8.refine_hi, frame.origin + kDefaultMargin);
    BOOST_CHECK_GE(out8.metric, 0.95f);
    assert_refine_window(out8, 2 * kDefaultMargin + 1);

    RadarTimingResult out64;
    BOOST_REQUIRE(refine_sync_origin(
        frame.rx.data(), frame.rx.size(), frame.sfd_start, kSynthSyncReps,
        kQm35SamplesPerSymbol, kWideMargin, kTimingThreshold, tmpl.data(),
        tmpl.size(), out64));
    BOOST_CHECK_EQUAL(out64.preamble_start_sample, frame.origin);
    BOOST_CHECK_GE(out64.metric, 0.95f);
    assert_refine_window(out64, 2 * kWideMargin + 1);
}

BOOST_AUTO_TEST_CASE(test_radar_timing_exhaustive_synthetic_offsets)
{
    std::vector<gr_complex> raw;
    BOOST_REQUIRE(load_cf32(testdata_path("reference_preamble.bin"), raw));
    auto tmpl = raw;
    openuwb::core::uwb_l2_normalize(tmpl);

    auto frame = make_sync_frame(raw, kSynthSyncReps);
    BOOST_REQUIRE_GE(frame.origin, kWideMargin);
    assert_exhaustive_offsets(frame.rx, frame.origin, frame.sfd_start,
                              kSynthSyncReps, kDefaultMargin, kTimingThreshold,
                              tmpl);
    assert_exhaustive_offsets(frame.rx, frame.origin, frame.sfd_start,
                              kSynthSyncReps, kWideMargin, kTimingThreshold,
                              tmpl);
}

BOOST_AUTO_TEST_CASE(test_radar_timing_canonical_matlab_golden)
{
    const std::string meta_path = testdata_path("uwb_radar/metadata.json");
    const std::string rx_path = testdata_path("uwb_radar/rx_clean_998p4.cf32");
    const std::string tx_path = testdata_path("uwb_radar/tx_998p4.cf32");
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

    double reps = 0.0;
    BOOST_REQUIRE(parse_json_number(json, "sync_repetitions", reps));
    BOOST_CHECK_EQUAL(static_cast<int>(reps), 64);

    double sps_d = 0.0;
    BOOST_REQUIRE(parse_json_number(json, "samples_per_symbol", sps_d));
    BOOST_CHECK_EQUAL(static_cast<int>(sps_d), 1016);

    double sfd_start_d = -1.0;
    BOOST_REQUIRE(parse_nested_number(json, "rx_clean_998p4", "sfd_start",
                                      sfd_start_d));
    const int64_t sfd_truth = static_cast<int64_t>(std::llround(sfd_start_d));
    BOOST_CHECK_EQUAL(sfd_truth, int64_t(67021));

    double origin_d = -1.0;
    BOOST_REQUIRE(parse_nested_number(json, "rx_clean_998p4", "sync_origin",
                                      origin_d));
    const int64_t origin_truth = static_cast<int64_t>(std::llround(origin_d));
    BOOST_CHECK_EQUAL(origin_truth, int64_t(1997));
    BOOST_CHECK_EQUAL(nominal_preamble_start(sfd_truth, 64,
                                             kQm35SamplesPerSymbol),
                      origin_truth);

    std::vector<gr_complex> rx, tx;
    BOOST_REQUIRE(load_cf32(rx_path, rx));
    BOOST_REQUIRE(load_cf32(tx_path, tx));
    BOOST_REQUIRE_EQUAL(rx.size(), size_t(197005));
    BOOST_REQUIRE_GE(tx.size(), kQm35SamplesPerSymbol);
    std::vector<gr_complex> tmpl(tx.begin(),
                                 tx.begin() +
                                     static_cast<std::ptrdiff_t>(
                                         kQm35SamplesPerSymbol));
    openuwb::core::uwb_l2_normalize(tmpl);

    RadarTimingResult out8;
    BOOST_REQUIRE(refine_sync_origin(
        rx.data(), rx.size(), sfd_truth, 64, kQm35SamplesPerSymbol,
        kDefaultMargin, kTimingThreshold, tmpl.data(), tmpl.size(), out8));
    BOOST_CHECK(out8.status == TimingStatus::Ok);
    BOOST_CHECK_LE(std::llabs(out8.preamble_start_sample - origin_truth), 1);
    BOOST_CHECK_GE(out8.metric, 0.5f);
    std::printf("canonical origin measured=%lld truth=%lld metric=%.6f "
                "margin=8\n",
                static_cast<long long>(out8.preamble_start_sample),
                static_cast<long long>(origin_truth),
                static_cast<double>(out8.metric));

    RadarTimingResult out64;
    BOOST_REQUIRE(refine_sync_origin(
        rx.data(), rx.size(), sfd_truth, 64, kQm35SamplesPerSymbol,
        kWideMargin, kTimingThreshold, tmpl.data(), tmpl.size(), out64));
    BOOST_CHECK_LE(std::llabs(out64.preamble_start_sample - origin_truth), 1);
    BOOST_CHECK_GE(out64.metric, 0.5f);
    std::printf("canonical origin measured=%lld truth=%lld metric=%.6f "
                "margin=64\n",
                static_cast<long long>(out64.preamble_start_sample),
                static_cast<long long>(origin_truth),
                static_cast<double>(out64.metric));

    assert_exhaustive_offsets(rx, origin_truth, sfd_truth, 64, kDefaultMargin,
                              kTimingThreshold, tmpl);
}

BOOST_AUTO_TEST_CASE(test_radar_timing_integer_delay)
{
    const std::string meta_path = testdata_path("uwb_radar/metadata.json");
    std::ifstream meta_f(meta_path);
    BOOST_REQUIRE(meta_f.good());
    std::ostringstream oss;
    oss << meta_f.rdbuf();
    const std::string json = oss.str();

    double origin_clean_d = -1.0;
    BOOST_REQUIRE(parse_nested_number(json, "rx_clean_998p4", "sync_origin",
                                      origin_clean_d));
    const int64_t origin_clean =
        static_cast<int64_t>(std::llround(origin_clean_d));

    double origin_d = -1.0;
    BOOST_REQUIRE(parse_nested_number(json, "rx_delay_int_998p4", "sync_origin",
                                      origin_d));
    const int64_t origin_truth = static_cast<int64_t>(std::llround(origin_d));
    BOOST_CHECK_EQUAL(origin_truth, origin_clean + 37);
    BOOST_CHECK_EQUAL(origin_truth, int64_t(2034));

    double sfd_d = -1.0;
    BOOST_REQUIRE(parse_nested_number(json, "rx_delay_int_998p4", "sfd_start",
                                      sfd_d));
    const int64_t sfd_truth = static_cast<int64_t>(std::llround(sfd_d));

    std::vector<gr_complex> rx, tx;
    BOOST_REQUIRE(load_cf32(testdata_path("uwb_radar/rx_delay_int_998p4.cf32"),
                            rx));
    BOOST_REQUIRE(load_cf32(testdata_path("uwb_radar/tx_998p4.cf32"), tx));
    BOOST_REQUIRE_EQUAL(rx.size(), size_t(197005));
    std::vector<gr_complex> tmpl(tx.begin(),
                                 tx.begin() +
                                     static_cast<std::ptrdiff_t>(
                                         kQm35SamplesPerSymbol));
    openuwb::core::uwb_l2_normalize(tmpl);

    RadarTimingResult out;
    BOOST_REQUIRE(refine_sync_origin(
        rx.data(), rx.size(), sfd_truth, 64, kQm35SamplesPerSymbol,
        kDefaultMargin, kTimingThreshold, tmpl.data(), tmpl.size(), out));
    BOOST_CHECK_LE(std::llabs(out.preamble_start_sample - origin_truth), 1);
    std::printf("delay_int origin measured=%lld truth=%lld metric=%.6f\n",
                static_cast<long long>(out.preamble_start_sample),
                static_cast<long long>(origin_truth),
                static_cast<double>(out.metric));
}

BOOST_AUTO_TEST_CASE(test_radar_timing_destroyed_and_noise)
{
    std::vector<gr_complex> raw;
    BOOST_REQUIRE(load_cf32(testdata_path("reference_preamble.bin"), raw));
    auto tmpl = raw;
    openuwb::core::uwb_l2_normalize(tmpl);

    {
        auto frame = make_sync_frame(raw, kSynthSyncReps);
        std::fill(frame.rx.begin() + frame.origin,
                  frame.rx.begin() + frame.origin +
                      static_cast<int64_t>(kQm35SamplesPerSymbol),
                  gr_complex(0.0f, 0.0f));
        RadarTimingResult out;
        const bool ok = refine_sync_origin(
            frame.rx.data(), frame.rx.size(), frame.sfd_start, kSynthSyncReps,
            kQm35SamplesPerSymbol, kDefaultMargin, kTimingThreshold,
            tmpl.data(), tmpl.size(), out);
        BOOST_REQUIRE(!ok);
        BOOST_CHECK(out.status == TimingStatus::TimingFailed);
        BOOST_CHECK_EQUAL(out.preamble_start_sample, int64_t(-1));
        BOOST_CHECK(out.preamble_start_sample != out.nominal_preamble_start);
    }

    {
        auto frame = make_sync_frame(raw, kSynthSyncReps);
        std::mt19937 rng(11);
        std::shuffle(frame.rx.begin() + frame.origin,
                     frame.rx.begin() + frame.origin +
                         static_cast<int64_t>(kQm35SamplesPerSymbol),
                     rng);
        RadarTimingResult out;
        const bool ok = refine_sync_origin(
            frame.rx.data(), frame.rx.size(), frame.sfd_start, kSynthSyncReps,
            kQm35SamplesPerSymbol, kDefaultMargin, kTimingThreshold,
            tmpl.data(), tmpl.size(), out);
        BOOST_REQUIRE(!ok);
        BOOST_CHECK(out.status == TimingStatus::TimingFailed);
        BOOST_CHECK_EQUAL(out.preamble_start_sample, int64_t(-1));
    }

    auto proto = make_sync_frame(raw, kSynthSyncReps);
    const uint32_t seeds[] = { 20260904u, 7u, 99u, 12345u, 424242u };
    for (uint32_t seed : seeds) {
        std::vector<gr_complex> noise(proto.rx.size());
        std::mt19937 rng(seed);
        std::normal_distribution<float> g(0.0f, 1.0f);
        for (auto& v : noise)
            v = gr_complex(g(rng), g(rng));
        RadarTimingResult buried;
        const bool ok = refine_sync_origin(
            noise.data(), noise.size(), proto.sfd_start, kSynthSyncReps,
            kQm35SamplesPerSymbol, kDefaultMargin, kTimingThreshold,
            tmpl.data(), tmpl.size(), buried);
        BOOST_REQUIRE(!ok);
        BOOST_CHECK(buried.status == TimingStatus::TimingFailed);
        BOOST_CHECK_EQUAL(buried.preamble_start_sample, int64_t(-1));
        BOOST_CHECK(buried.preamble_start_sample !=
                    buried.nominal_preamble_start);
    }
}

BOOST_AUTO_TEST_CASE(test_radar_timing_clipped_negative_and_short)
{
    auto tmpl = load_sync_template();
    std::vector<gr_complex> rx(8192, gr_complex(0.1f, 0.0f));

    // Nominal origin is far negative; clipped refine window is empty.
    RadarTimingResult neg;
    const bool ok_neg = refine_sync_origin(
        rx.data(), rx.size(), /*sfd_start=*/100, 64, kQm35SamplesPerSymbol,
        kDefaultMargin, kTimingThreshold, tmpl.data(), tmpl.size(), neg);
    BOOST_REQUIRE(!ok_neg);
    BOOST_CHECK(neg.status == TimingStatus::TimingFailed);
    BOOST_CHECK_EQUAL(neg.preamble_start_sample, int64_t(-1));
    BOOST_CHECK_EQUAL(neg.sfd_start_sample, int64_t(100));
    BOOST_CHECK_LT(neg.nominal_preamble_start, int64_t(0));
    BOOST_CHECK(neg.preamble_start_sample != neg.nominal_preamble_start);

    // Buffer shorter than one SYNC: no legal start.
    std::vector<gr_complex> short_rx(kQm35SamplesPerSymbol / 2,
                                     gr_complex(0.1f, 0.0f));
    RadarTimingResult short_out;
    const bool ok_short = refine_sync_origin(
        short_rx.data(), short_rx.size(), /*sfd_start=*/4096, 64,
        kQm35SamplesPerSymbol, kDefaultMargin, kTimingThreshold, tmpl.data(),
        tmpl.size(), short_out);
    BOOST_REQUIRE(!ok_short);
    BOOST_CHECK(short_out.status == TimingStatus::TimingFailed);
    BOOST_CHECK_EQUAL(short_out.preamble_start_sample, int64_t(-1));
}

BOOST_AUTO_TEST_CASE(test_radar_timing_invalid_input)
{
    auto tmpl = load_sync_template();
    std::vector<gr_complex> rx(4096, gr_complex(0.0f, 0.0f));
    RadarTimingResult out;

    BOOST_CHECK(!refine_sync_origin(nullptr, rx.size(), 2000, 64,
                                    kQm35SamplesPerSymbol, kDefaultMargin,
                                    kTimingThreshold, tmpl.data(),
                                    tmpl.size(), out));
    BOOST_CHECK(out.status == TimingStatus::InvalidInput);
    BOOST_CHECK_EQUAL(out.preamble_start_sample, int64_t(-1));

    BOOST_CHECK(!refine_sync_origin(rx.data(), 0, 2000, 64,
                                    kQm35SamplesPerSymbol, kDefaultMargin,
                                    kTimingThreshold, tmpl.data(),
                                    tmpl.size(), out));
    BOOST_CHECK(out.status == TimingStatus::InvalidInput);

    BOOST_CHECK(!refine_sync_origin(rx.data(), rx.size(), -1, 64,
                                    kQm35SamplesPerSymbol, kDefaultMargin,
                                    kTimingThreshold, tmpl.data(),
                                    tmpl.size(), out));
    BOOST_CHECK(out.status == TimingStatus::InvalidInput);
    BOOST_CHECK_EQUAL(out.sfd_start_sample, int64_t(-1));
    BOOST_CHECK_EQUAL(out.nominal_preamble_start, int64_t(-1));

    BOOST_CHECK(!refine_sync_origin(rx.data(), rx.size(), 2000, 0,
                                    kQm35SamplesPerSymbol, kDefaultMargin,
                                    kTimingThreshold, tmpl.data(),
                                    tmpl.size(), out));
    BOOST_CHECK(out.status == TimingStatus::InvalidInput);

    BOOST_CHECK(!refine_sync_origin(rx.data(), rx.size(), 2000, 64, 0,
                                    kDefaultMargin, kTimingThreshold,
                                    tmpl.data(), tmpl.size(), out));
    BOOST_CHECK(out.status == TimingStatus::InvalidInput);

    BOOST_CHECK(!refine_sync_origin(rx.data(), rx.size(), 2000, 64,
                                    kQm35SamplesPerSymbol, -1, kTimingThreshold,
                                    tmpl.data(), tmpl.size(), out));
    BOOST_CHECK(out.status == TimingStatus::InvalidInput);

    BOOST_CHECK(!refine_sync_origin(rx.data(), rx.size(), 2000, 64,
                                    kQm35SamplesPerSymbol, kDefaultMargin, 0.0f,
                                    tmpl.data(), tmpl.size(), out));
    BOOST_CHECK(out.status == TimingStatus::InvalidInput);
    BOOST_CHECK(!refine_sync_origin(
        rx.data(), rx.size(), 2000, 64, kQm35SamplesPerSymbol, kDefaultMargin,
        std::numeric_limits<float>::quiet_NaN(), tmpl.data(), tmpl.size(),
        out));
    BOOST_CHECK(out.status == TimingStatus::InvalidInput);
    BOOST_CHECK(!refine_sync_origin(
        rx.data(), rx.size(), 2000, 64, kQm35SamplesPerSymbol, kDefaultMargin,
        std::numeric_limits<float>::infinity(), tmpl.data(), tmpl.size(),
        out));
    BOOST_CHECK(out.status == TimingStatus::InvalidInput);

    BOOST_CHECK(!refine_sync_origin(rx.data(), rx.size(), 2000, 64,
                                    kQm35SamplesPerSymbol, kDefaultMargin,
                                    kTimingThreshold, nullptr, tmpl.size(),
                                    out));
    BOOST_CHECK(out.status == TimingStatus::InvalidInput);
    BOOST_CHECK_EQUAL(out.preamble_start_sample, int64_t(-1));

    BOOST_CHECK(!refine_sync_origin(rx.data(), rx.size(), 2000, 64,
                                    kQm35SamplesPerSymbol, kDefaultMargin,
                                    kTimingThreshold, tmpl.data(), 0, out));
    BOOST_CHECK(out.status == TimingStatus::InvalidInput);

    BOOST_CHECK(!refine_sync_origin(rx.data(), rx.size(), 2000, 64,
                                    kQm35SamplesPerSymbol, kDefaultMargin,
                                    kTimingThreshold, tmpl.data(), 10, out));
    BOOST_CHECK(out.status == TimingStatus::InvalidInput);

    std::vector<gr_complex> nan_tmpl = tmpl;
    nan_tmpl[3] = gr_complex(std::numeric_limits<float>::quiet_NaN(), 0.0f);
    BOOST_CHECK(!refine_sync_origin(rx.data(), rx.size(), 2000, 64,
                                    kQm35SamplesPerSymbol, kDefaultMargin,
                                    kTimingThreshold, nan_tmpl.data(),
                                    nan_tmpl.size(), out));
    BOOST_CHECK(out.status == TimingStatus::InvalidInput);
}

BOOST_AUTO_TEST_CASE(test_radar_timing_nominal_outside_margin)
{
    std::vector<gr_complex> raw;
    BOOST_REQUIRE(load_cf32(testdata_path("reference_preamble.bin"), raw));
    auto tmpl = raw;
    openuwb::core::uwb_l2_normalize(tmpl);

    // Single SYNC so an extra-symbol prediction cannot land on a later copy.
    auto frame = make_sync_frame(raw, /*n_sync=*/1);

    RadarTimingResult miss_sym;
    const int64_t far_sfd =
        frame.sfd_start + static_cast<int64_t>(kQm35SamplesPerSymbol);
    const bool ok_sym = refine_sync_origin(
        frame.rx.data(), frame.rx.size(), far_sfd, /*n_sync=*/1,
        kQm35SamplesPerSymbol, kDefaultMargin, kTimingThreshold, tmpl.data(),
        tmpl.size(), miss_sym);
    BOOST_REQUIRE(!ok_sym);
    BOOST_CHECK(miss_sym.status == TimingStatus::TimingFailed);
    BOOST_CHECK_EQUAL(miss_sym.preamble_start_sample, int64_t(-1));
    BOOST_CHECK(miss_sym.preamble_start_sample !=
                miss_sym.nominal_preamble_start);
    BOOST_CHECK_EQUAL(miss_sym.nominal_preamble_start,
                      frame.origin +
                          static_cast<int64_t>(kQm35SamplesPerSymbol));

    RadarTimingResult miss_off;
    const int64_t off_sfd = frame.sfd_start + 37;
    const bool ok_off = refine_sync_origin(
        frame.rx.data(), frame.rx.size(), off_sfd, /*n_sync=*/1,
        kQm35SamplesPerSymbol, kDefaultMargin, kTimingThreshold, tmpl.data(),
        tmpl.size(), miss_off);
    BOOST_REQUIRE(!ok_off);
    BOOST_CHECK(miss_off.status == TimingStatus::TimingFailed);
    BOOST_CHECK_EQUAL(miss_off.preamble_start_sample, int64_t(-1));
}

BOOST_AUTO_TEST_CASE(test_fold_acquire_window_is_clamped_to_half_a_period)
{
    // Task 1 (compute-cost): fold_acquire can only ever select a lag within
    // +/-P/2 of the seed, so it now clamps its fold + correlation window to
    // exactly that sub-window instead of folding the whole search radius.  At
    // AcquireLegacy's 8192-sample radius that is 16384 -> 1016 lags (-93.8%).
    //
    // The clamp must not change WHICH lag wins: the folded vector is indexed
    // absolutely (folded[i] = sum_k rx[w0 + i + k*P]), so every retained lag
    // sees the same samples as before.  This test pins both halves: the window
    // really shrinks, and the reported start/metric still equal a direct
    // normalised correlation at that start.
    const auto tmpl = load_normalized_sync();
    const size_t sym = kQm35SamplesPerSymbol;
    auto frame = make_low_snr_frame(tmpl, kLowSnrSyncReps, 8192, 2048,
                                    /*snr_db=*/2.0, 424242u);
    DemodScratch scratch;
    scratch.reserve(frame.rx.size());
    scratch.work = tmpl;
    openuwb::core::uwb_l2_normalize(scratch.work);
    const float Et = openuwb::core::uwb_template_energy(scratch.work);
    openuwb::demod::rxopt::build_real_sync_template(scratch.work, scratch.real_sync_template);
    BOOST_REQUIRE(scratch.real_sync_template.is_real);

    const int64_t period = static_cast<int64_t>(sym);
    // Seed 3 SYMBOLS late: inside the fold's +/-P/2 resolution, and far enough
    // that a wrong absolute grid phase would be visible.
    const int64_t centre = frame.start + 3 * period;
    const int64_t radius = 8192;
    const auto res = openuwb::demod::core::detail::fold_acquire(
        frame.rx.data(), frame.rx.size(), centre, centre - radius,
        centre + radius, scratch.work.data(), scratch.work.size(), Et,
        scratch.real_sync_template, openuwb::demod::core::detail::kFoldMaxSymbols,
        kLowSnrSyncReps, 0.0f);
    BOOST_REQUIRE(res.ok);
    // Clamped window: at most P + 1 lags, not 2*radius + 1.
    BOOST_CHECK_LE(res.window_lags, static_cast<size_t>(period + 1));
    BOOST_CHECK_LE(res.window_lags, static_cast<size_t>(2 * radius + 1));
    // The clamp must not move the answer outside the +/-P/2 ambiguity window.
    BOOST_CHECK_LE(std::llabs(res.start - centre), period / 2);
    // ... and the reported start is a real SYNC origin (the fold is
    // P-periodic, so "true start + k*P" is the correct answer class).
    BOOST_CHECK_LE(std::llabs((res.start - frame.start) % period), int64_t(2));
    // The metric is the NORMALISED correlation of the K-fold coherent sum, so
    // it is far above any single-SYMBOL metric.  Recompute it from the folded
    // vector to prove the clamp did not change which samples were summed.
    const int64_t w0 = centre - period / 2;
    const int64_t j = res.start - w0;
    BOOST_REQUIRE_GE(j, 0);
    BOOST_REQUIRE_LT(j, period + 1);
    std::vector<gr_complex> folded(scratch.work.size(),
                                   gr_complex(0.0f, 0.0f));
    for (size_t k = 0; k < res.symbols_folded; ++k)
        for (size_t i = 0; i < folded.size(); ++i)
            folded[i] += frame.rx[static_cast<size_t>(res.start) +
                                  k * static_cast<size_t>(period) + i];
    const std::complex<float> acc =
        openuwb::demod::rxopt::real_template_dot_single(
            folded.data(), scratch.real_sync_template.taps_dup.data(),
            folded.size());
    float pw = 0.0f;
    for (size_t i = 0; i < folded.size(); ++i)
        pw += std::norm(folded[i]);
    const float direct = std::norm(acc) / (pw * Et + 1e-12f);
    BOOST_CHECK_CLOSE(res.metric, direct, 1.0);

    // A seed more than half a period outside the true start still lands on the
    // nearest legal lag (the fold is P-periodic, so this is the documented
    // ambiguity, not an error).
    const int64_t far_centre = frame.start + 40 * period;
    const auto far = openuwb::demod::core::detail::fold_acquire(
        frame.rx.data(), frame.rx.size(), far_centre, far_centre - radius,
        far_centre + radius, scratch.work.data(), scratch.work.size(), Et,
        scratch.real_sync_template, openuwb::demod::core::detail::kFoldMaxSymbols,
        kLowSnrSyncReps, 0.0f);
    BOOST_REQUIRE(far.ok);
    BOOST_CHECK_LE(far.window_lags, static_cast<size_t>(period + 1));
    BOOST_CHECK_EQUAL((far.start - frame.start) % period, 0);
}
