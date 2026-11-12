/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * QA for the sensing adapter on the SHARED preamble front-end and the SHARED
 * SYNC-CIR kernel (workstream C of
 * docs/performance/OpenCode_Sensing与Comm统一前端和CIR_开发方案_2026-11-12.md).
 *
 * Everything is synthesised from testdata/reference_preamble.bin (one
 * pulse-shaped code-9 SYNC symbol) and GetSfdSequence(), so the tests exercise
 * the production entry point `radar::radar_cir_one()` with public inputs only:
 * no injected CFO truth, no injected delay truth, no sidecar.
 *
 * Covered:
 *   1. TARGET CFO: 0 / +50 kHz / -50 kHz must leave the CIR peak tap and
 *      amplitude intact, while the uncompensated control arm (the defect this
 *      workstream closes) must degrade.
 *   2. Distance axis: integer and fractional delays appear as tap shifts, and
 *      changing the DETECTED start must not re-zero the physical peak.
 *   3. Amplitude / phase: raw scales with input gain, norm is unit L2, an input
 *      phase rotation rotates the raw CIR by the same angle (the documented
 *      "phase referenced to the buffer origin" convention).
 *   4. Failure paths: noise only, wrong code, wrong SFD, missing prefix/halo
 *      fail closed with tap_count == 0 and no stale taps.
 *   5. Legacy cross-check: the shared kernel matches the legacy scalar
 *      `estimate_radar_cir` on the same zero-CFO input (relative L2 <= 2e-5,
 *      identical peak tap), which is the only thing the scalar math is kept for.
 */

#ifndef BOOST_TEST_MODULE
#define BOOST_TEST_MODULE qa_uwb_radar_shared_cir
#endif
// GR_ADD_CPP_TEST already passes -DBOOST_TEST_DYN_LINK; the guard keeps a plain
// standalone `g++ qa_uwb_radar_shared_cir.cc -lboost_unit_test_framework`
// working (Boost.Test only emits main() for the dyn-link variant).
#ifndef BOOST_TEST_DYN_LINK
#define BOOST_TEST_DYN_LINK
#endif

#include <boost/test/unit_test.hpp>

#include <gnuradio/gr_complex.h>
#include <gnuradio/uwb/uwb_phy_profile.h>
#include <gnuradio/uwb/uwb_radar_cir_core.h>
#include <gnuradio/uwb/uwb_radar_cir_estimator.h>

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <limits>
#include <string>
#include <vector>

namespace {

using gr::uwb::demod::GetPreambleCode;
using gr::uwb::demod::GetSfdSequence;
using gr::uwb::demod::kQm35CodeLength;
using gr::uwb::demod::kQm35SamplesPerSymbol;
using gr::uwb::radar::estimate_radar_cir;
using gr::uwb::radar::prepare_radar_cir_code;
using gr::uwb::radar::prepare_radar_cir_core;
using gr::uwb::radar::radar_cir_one;
using gr::uwb::radar::RadarCirConfig;
using gr::uwb::radar::RadarCirCoreScratch;
using gr::uwb::radar::RadarCirEstimate;
using gr::uwb::radar::RadarCirResult;
using gr::uwb::radar::RadarCirScratch;
using gr::uwb::radar::RadarCirStatus;

constexpr double kFs = 998.4e6;
constexpr size_t kCirPre = 16;
constexpr size_t kCirPost = 100;
constexpr size_t kCirSkip = 10;
constexpr size_t kCirReps = 54;
constexpr size_t kNSync = 64;
constexpr int64_t kPreGuard = 64;
constexpr int64_t kTail = 256;
constexpr int64_t kSfdMargin = 64;
constexpr float kThreshold = 0.3f;

// ---------------------------------------------------------------------------
// testdata
// ---------------------------------------------------------------------------
bool load_cf32(const std::string& path, std::vector<gr_complex>& out)
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
    return static_cast<bool>(f);
}

std::string testdata_path(const std::string& rel)
{
    if (const char* env = std::getenv("UWB_TESTDATA_DIR")) {
        const std::string p = std::string(env) + "/" + rel;
        std::ifstream f(p, std::ios::binary);
        if (f)
            return p;
    }
    const char* prefixes[] = {
        "../../../testdata/", "../../testdata/", "../testdata/", "testdata/",
    };
    for (const char* p : prefixes) {
        const std::string path = std::string(p) + rel;
        std::ifstream f(path, std::ios::binary);
        if (f)
            return path;
    }
    return std::string("testdata/") + rel;
}

// One pulse-shaped SYNC symbol (code 9 / code 10), 1016 samples.
std::vector<gr_complex> load_sync_pulse(const char* rel)
{
    std::vector<gr_complex> tmpl;
    const std::string path = testdata_path(rel);
    BOOST_REQUIRE_MESSAGE(load_cf32(path, tmpl), "cannot load " + path);
    BOOST_REQUIRE_EQUAL(tmpl.size(), kQm35SamplesPerSymbol);
    return tmpl;
}

// ---------------------------------------------------------------------------
// synthesis
// ---------------------------------------------------------------------------
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

void place(std::vector<gr_complex>& rx,
           int64_t start,
           const std::vector<gr_complex>& w)
{
    BOOST_REQUIRE(start >= 0);
    BOOST_REQUIRE(static_cast<size_t>(start) + w.size() <= rx.size());
    for (size_t k = 0; k < w.size(); ++k)
        rx[static_cast<size_t>(start) + k] = w[k];
}

// A clean frame: `n_sync` SYNC symbols at `sync_origin`, then the SFD, then
// `tail` zero samples.  The SFD starts exactly n_sync*sps after the origin.
struct Frame {
    std::vector<gr_complex> rx;
    int64_t sync_origin = 0;
    int64_t sfd_start = 0;
};

Frame make_frame(const std::vector<gr_complex>& sync,
                 size_t n_sync = kNSync,
                 int64_t sync_origin = kPreGuard,
                 const char* sfd_mode = "4z2",
                 int64_t tail = kTail)
{
    const auto seq = GetSfdSequence(sfd_mode);
    BOOST_REQUIRE(!seq.empty());
    const auto sfd_wf = kron_sfd(seq, sync);
    Frame f;
    f.sync_origin = sync_origin;
    f.sfd_start = sync_origin + static_cast<int64_t>(n_sync) * 1016;
    const size_t n = static_cast<size_t>(sync_origin) + n_sync * sync.size() +
                     sfd_wf.size() + static_cast<size_t>(tail);
    f.rx.assign(n, gr_complex(0.0f, 0.0f));
    for (size_t i = 0; i < n_sync; ++i)
        place(f.rx, sync_origin + static_cast<int64_t>(i) * 1016, sync);
    place(f.rx, f.sfd_start, sfd_wf);
    return f;
}

// Multiply sample i by exp(+j 2 pi f i / fs).  The shared front-end derotates
// with dst[i] = src[i] * exp(-j 2 pi f i / fs) anchored at the window origin,
// so a correct estimate cancels this exactly.
void apply_cfo(std::vector<gr_complex>& rx, double cfo_hz)
{
    if (cfo_hz == 0.0)
        return;
    const double w = 2.0 * M_PI * cfo_hz / kFs;
    for (size_t i = 0; i < rx.size(); ++i) {
        const double ph = w * static_cast<double>(i);
        rx[i] *= gr_complex(static_cast<float>(std::cos(ph)),
                            static_cast<float>(std::sin(ph)));
    }
}

// y[i] = sum_m x[i-m] * sinc(m - tau) * hann(m): a delay of `tau` samples.
std::vector<gr_complex> fractional_delay(const std::vector<gr_complex>& x,
                                         double tau)
{
    const int M = 16;
    std::vector<double> h(2 * M + 1, 0.0);
    double sum = 0.0;
    for (int m = -M; m <= M; ++m) {
        const double d = static_cast<double>(m) - tau;
        const double sinc =
            (std::abs(d) < 1e-12) ? 1.0 : std::sin(M_PI * d) / (M_PI * d);
        const double w =
            0.5 * (1.0 + std::cos(M_PI * static_cast<double>(m) / (M + 1)));
        h[static_cast<size_t>(m + M)] = sinc * w;
        sum += sinc * w;
    }
    BOOST_REQUIRE(std::abs(sum) > 1e-9);
    for (auto& v : h)
        v /= sum;
    std::vector<gr_complex> y(x.size(), gr_complex(0.0f, 0.0f));
    for (size_t i = 0; i < x.size(); ++i) {
        gr_complex acc(0.0f, 0.0f);
        for (int m = -M; m <= M; ++m) {
            const long j = static_cast<long>(i) - m;
            if (j < 0 || j >= static_cast<long>(x.size()))
                continue;
            acc += x[static_cast<size_t>(j)] *
                   static_cast<float>(h[static_cast<size_t>(m + M)]);
        }
        y[i] = acc;
    }
    return y;
}

// ---------------------------------------------------------------------------
// helpers
// ---------------------------------------------------------------------------
RadarCirConfig make_cfg()
{
    RadarCirConfig cfg;
    cfg.sync_repetitions = kNSync;
    cfg.samples_per_symbol = kQm35SamplesPerSymbol;
    cfg.sfd_mode = "4z2";
    cfg.sfd_search_margin = kSfdMargin;
    cfg.sync_refine_margin = 8;
    cfg.sfd_threshold = kThreshold;
    cfg.sync_refine_threshold = kThreshold;
    cfg.cir_pre = kCirPre;
    cfg.cir_post = kCirPost;
    cfg.cir_skip_initial = kCirSkip;
    cfg.cir_repetitions = kCirReps;
    return cfg;
}

bool prepare(const std::vector<gr_complex>& sync,
             RadarCirCoreScratch& scratch,
             const RadarCirConfig& cfg = RadarCirConfig{})
{
    RadarCirConfig use = cfg;
    if (use.sync_repetitions == 0)
        use = make_cfg();
    use.samples_per_symbol = kQm35SamplesPerSymbol;
    return prepare_radar_cir_core(use, sync.data(), sync.size(), kCirPre,
                                  kCirPost, scratch);
}

double rel_l2(const std::vector<gr_complex>& a, const std::vector<gr_complex>& b)
{
    BOOST_REQUIRE_EQUAL(a.size(), b.size());
    double num = 0.0;
    double den = 0.0;
    for (size_t i = 0; i < a.size(); ++i) {
        const gr_complex d = a[i] - b[i];
        num += std::norm(d);
        den += std::norm(b[i]);
    }
    if (den == 0.0)
        return num == 0.0 ? 0.0 : std::numeric_limits<double>::infinity();
    return std::sqrt(num / den);
}

std::vector<gr_complex> copy_taps(const RadarCirCoreScratch& scratch,
                                  size_t tap_count)
{
    BOOST_REQUIRE_GE(scratch.cir.raw_taps.size(), tap_count);
    return std::vector<gr_complex>(
        scratch.cir.raw_taps.begin(),
        scratch.cir.raw_taps.begin() + static_cast<std::ptrdiff_t>(tap_count));
}

std::vector<gr_complex> copy_norm(const RadarCirCoreScratch& scratch,
                                  size_t tap_count)
{
    BOOST_REQUIRE_GE(scratch.cir.norm_taps.size(), tap_count);
    return std::vector<gr_complex>(
        scratch.cir.norm_taps.begin(),
        scratch.cir.norm_taps.begin() + static_cast<std::ptrdiff_t>(tap_count));
}

bool all_zero(const std::vector<gr_complex>& v, size_t count)
{
    for (size_t i = 0; i < count; ++i) {
        if (v[i] != gr_complex(0.0f, 0.0f))
            return false;
    }
    return true;
}

double db_ratio(double num, double den)
{
    if (!(num > 0.0) || !(den > 0.0))
        return -std::numeric_limits<double>::infinity();
    return 20.0 * std::log10(num / den);
}

// Absolute sample of the physical peak on the CIR axis:
//   tap j  <->  cir_axis_origin - cir_pre + j
int64_t physical_peak_sample(const RadarCirResult& r, size_t pre)
{
    return r.cir_axis_origin - static_cast<int64_t>(pre) +
           static_cast<int64_t>(r.peak_tap);
}

// Run one production call and require success.
RadarCirResult run_ok(const std::vector<gr_complex>& rx,
                      int64_t predicted_sfd,
                      const RadarCirConfig& cfg,
                      RadarCirCoreScratch& scratch)
{
    RadarCirResult out;
    const bool ok = radar_cir_one(rx.data(), rx.size(), predicted_sfd, cfg,
                                  scratch, out);
    BOOST_REQUIRE_MESSAGE(ok, "radar_cir_one failed, status="
                                  << static_cast<int>(out.status));
    BOOST_REQUIRE(out.status == RadarCirStatus::Ok);
    BOOST_REQUIRE_EQUAL(out.tap_count, kCirPre + kCirPost);
    return out;
}

} // namespace

// ---------------------------------------------------------------------------
// 1. Target CFO: estimate + compensate.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_shared_cir_compensates_target_cfo)
{
    const auto sync = load_sync_pulse("reference_preamble.bin");
    RadarCirCoreScratch scratch;
    BOOST_REQUIRE(prepare(sync, scratch));
    const RadarCirConfig cfg = make_cfg();
    const Frame f = make_frame(sync);
    gr::uwb::radar::reserve_radar_cir_core(f.rx.size(), scratch);

    // Clean reference (CFO = 0, compensated path).
    const RadarCirResult ref = run_ok(f.rx, f.sfd_start, cfg, scratch);
    const auto ref_raw = copy_taps(scratch, ref.tap_count);
    BOOST_CHECK_EQUAL(ref.cfo_epoch, 2u);
    BOOST_CHECK_EQUAL(ref.phase_reference_sample, int64_t(0));
    BOOST_CHECK_EQUAL(ref.valid_repetitions, kCirReps);
    BOOST_CHECK_EQUAL(ref.cir_axis_origin, f.sync_origin);
    BOOST_CHECK_EQUAL(ref.detected_sync_origin, f.sync_origin);
    BOOST_CHECK_EQUAL(ref.cir_origin_sample, ref.cir_axis_origin);
    BOOST_CHECK_GT(ref.peak_abs, 0.0f);

    const double cfos[] = { 0.0, 50000.0, -50000.0 };
    for (double cfo : cfos) {
        std::vector<gr_complex> rx = f.rx;
        apply_cfo(rx, cfo);

        // Compensated (production default).
        const RadarCirResult got = run_ok(rx, f.sfd_start, cfg, scratch);
        const auto raw = copy_taps(scratch, got.tap_count);
        BOOST_TEST_MESSAGE("cfo=" << cfo << " Hz: estimated="
                                  << got.final_cfo_hz << " Hz peak_tap="
                                  << got.peak_tap << " peak_abs="
                                  << got.peak_abs << " (ref " << ref.peak_abs
                                  << ")");
        BOOST_CHECK_EQUAL(got.cfo_epoch, 2u);
        BOOST_CHECK_EQUAL(got.peak_tap, ref.peak_tap);
        BOOST_CHECK_EQUAL(got.valid_repetitions, ref.valid_repetitions);
        // Amplitude within 0.1 dB of the zero-CFO reference.
        BOOST_CHECK_LT(std::abs(db_ratio(got.peak_abs, ref.peak_abs)), 0.1);
        if (cfo == 0.0) {
            BOOST_CHECK_LT(rel_l2(raw, ref_raw), 1e-3);
        } else {
            // The estimate must be close to the injected offset; the residual
            // is what the 0.1 dB amplitude budget above actually measures.
            BOOST_CHECK_LT(std::abs(got.final_cfo_hz - cfo), 2000.0);
        }

        // Uncompensated control arm: this is the historical defect (the raw
        // SYNC windows are averaged while the target CFO keeps rotating them).
        RadarCirConfig raw_cfg = cfg;
        raw_cfg.compensate_target_cfo = false;
        const RadarCirResult unc = run_ok(rx, f.sfd_start, raw_cfg, scratch);
        const double loss_db = db_ratio(unc.peak_abs, ref.peak_abs);
        BOOST_TEST_MESSAGE("cfo=" << cfo << " Hz uncompensated peak_abs="
                                  << unc.peak_abs << " loss=" << loss_db
                                  << " dB");
        // The estimate is still reported, but cfo_epoch == 0 says it was NOT
        // applied, so this can never be mistaken for a compensated result.
        BOOST_CHECK_EQUAL(unc.cfo_epoch, 0u);
        if (cfo == 0.0) {
            BOOST_CHECK_EQUAL(unc.peak_tap, ref.peak_tap);
            BOOST_CHECK_LT(std::abs(loss_db), 0.1);
        } else {
            BOOST_CHECK_LT(loss_db, -3.0);
        }
    }
}

// ---------------------------------------------------------------------------
// 2. Distance axis: delays appear as tap shifts; the detected start may move
//    without re-zeroing the physical peak.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_shared_cir_distance_axis)
{
    const auto sync = load_sync_pulse("reference_preamble.bin");
    RadarCirCoreScratch scratch;
    BOOST_REQUIRE(prepare(sync, scratch));
    const RadarCirConfig cfg = make_cfg();

    const Frame clean = make_frame(sync);
    gr::uwb::radar::reserve_radar_cir_core(clean.rx.size() + 64, scratch);
    const RadarCirResult base = run_ok(clean.rx, clean.sfd_start, cfg, scratch);
    const int64_t base_phys = physical_peak_sample(base, kCirPre);
    BOOST_CHECK_EQUAL(base.cir_axis_origin, clean.sync_origin);

    // Integer delay: the whole echo arrives 37 samples later, the TX-time
    // prediction (and therefore the CIR axis) is unchanged.
    {
        const Frame delayed = make_frame(sync, kNSync, kPreGuard + 37);
        const RadarCirResult got =
            run_ok(delayed.rx, clean.sfd_start, cfg, scratch);
        BOOST_CHECK_EQUAL(got.cir_axis_origin, base.cir_axis_origin);
        BOOST_CHECK_EQUAL(got.peak_tap, base.peak_tap + 37);
        BOOST_CHECK_EQUAL(physical_peak_sample(got, kCirPre), base_phys + 37);
        BOOST_CHECK_EQUAL(got.detected_sync_origin, clean.sync_origin + 37);
        BOOST_CHECK_NE(got.detected_sync_origin, got.cir_axis_origin);
        BOOST_CHECK_LT(std::abs(db_ratio(got.peak_abs, base.peak_abs)), 0.1);
    }

    // Fractional delay 37.4: the main lobe splits, the peak stays within one
    // tap of the integer part and the axis is still not re-zeroed.
    {
        const Frame delayed = make_frame(sync, kNSync, kPreGuard + 37);
        const auto rx = fractional_delay(delayed.rx, 0.4);
        const RadarCirResult got = run_ok(rx, clean.sfd_start, cfg, scratch);
        BOOST_CHECK_EQUAL(got.cir_axis_origin, base.cir_axis_origin);
        BOOST_CHECK_LE(std::llabs(static_cast<int64_t>(got.peak_tap) -
                                  static_cast<int64_t>(base.peak_tap + 37)),
                       1);
        BOOST_CHECK_LT(std::abs(db_ratio(got.peak_abs, base.peak_abs)), 0.5);
    }

    // Different detected start: the prediction is 40 samples early, so the
    // front-end locks the same physical frame while the CIR axis moves by 40.
    // The physical peak must NOT move with the detected start.
    {
        const int64_t shifted_pred = clean.sfd_start - 40;
        const RadarCirResult got =
            run_ok(clean.rx, shifted_pred, cfg, scratch);
        BOOST_CHECK_EQUAL(got.detected_sync_origin, base.detected_sync_origin);
        BOOST_CHECK_EQUAL(got.cir_axis_origin, base.cir_axis_origin - 40);
        BOOST_CHECK_EQUAL(static_cast<int64_t>(got.peak_tap),
                          static_cast<int64_t>(base.peak_tap) + 40);
        BOOST_CHECK_EQUAL(physical_peak_sample(got, kCirPre), base_phys);
        BOOST_CHECK_LT(std::abs(db_ratio(got.peak_abs, base.peak_abs)), 0.1);
    }
}

// ---------------------------------------------------------------------------
// 3. Amplitude and phase.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_shared_cir_amplitude_and_phase)
{
    const auto sync = load_sync_pulse("reference_preamble.bin");
    RadarCirCoreScratch scratch;
    BOOST_REQUIRE(prepare(sync, scratch));
    const RadarCirConfig cfg = make_cfg();
    const Frame f = make_frame(sync);
    gr::uwb::radar::reserve_radar_cir_core(f.rx.size(), scratch);

    const RadarCirResult base = run_ok(f.rx, f.sfd_start, cfg, scratch);
    const auto base_raw = copy_taps(scratch, base.tap_count);
    const auto base_norm = copy_norm(scratch, base.tap_count);

    // norm is unit L2 and raw keeps the true amplitude.
    double nrm = 0.0;
    for (const auto& v : base_norm)
        nrm += std::norm(v);
    BOOST_CHECK_CLOSE(std::sqrt(nrm), 1.0, 1e-3);
    double raw_l2 = 0.0;
    for (const auto& v : base_raw)
        raw_l2 += std::norm(v);
    BOOST_CHECK_CLOSE(std::sqrt(raw_l2), static_cast<double>(base.raw_l2_norm),
                      1e-3);
    BOOST_CHECK_GT(base.raw_l2_norm, 0.0f);

    // Input gain scales raw linearly; norm is invariant.
    const float gains[] = { 0.25f, 2.5f };
    for (float g : gains) {
        std::vector<gr_complex> rx = f.rx;
        for (auto& v : rx)
            v *= g;
        const RadarCirResult got = run_ok(rx, f.sfd_start, cfg, scratch);
        const auto raw = copy_taps(scratch, got.tap_count);
        const auto norm = copy_norm(scratch, got.tap_count);
        BOOST_CHECK_EQUAL(got.peak_tap, base.peak_tap);
        std::vector<gr_complex> expected(base_raw.size());
        for (size_t i = 0; i < base_raw.size(); ++i)
            expected[i] = g * base_raw[i];
        BOOST_CHECK_LT(rel_l2(raw, expected), 1e-5);
        BOOST_CHECK_LT(rel_l2(norm, base_norm), 1e-5);
        BOOST_CHECK_CLOSE(got.raw_l2_norm, g * base.raw_l2_norm, 1e-3);
    }

    // Phase reference: the raw CIR phase is referenced to the buffer origin, so
    // rotating the input by phi rotates every tap by exactly phi (and leaves
    // the magnitudes and the peak tap untouched).
    const double phi = 0.7;
    {
        std::vector<gr_complex> rx = f.rx;
        const gr_complex rot(static_cast<float>(std::cos(phi)),
                             static_cast<float>(std::sin(phi)));
        for (auto& v : rx)
            v *= rot;
        const RadarCirResult got = run_ok(rx, f.sfd_start, cfg, scratch);
        const auto raw = copy_taps(scratch, got.tap_count);
        BOOST_CHECK_EQUAL(got.phase_reference_sample, int64_t(0));
        BOOST_CHECK_EQUAL(got.peak_tap, base.peak_tap);
        std::vector<gr_complex> expected(base_raw.size());
        for (size_t i = 0; i < base_raw.size(); ++i)
            expected[i] = rot * base_raw[i];
        BOOST_CHECK_LT(rel_l2(raw, expected), 1e-5);
        BOOST_CHECK_LT(std::abs(db_ratio(got.peak_abs, base.peak_abs)), 0.01);
    }
}

// ---------------------------------------------------------------------------
// 4. Failure paths: fail closed, tap_count == 0, no stale taps.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_shared_cir_failure_paths)
{
    const auto sync = load_sync_pulse("reference_preamble.bin");
    RadarCirCoreScratch scratch;
    BOOST_REQUIRE(prepare(sync, scratch));
    const RadarCirConfig cfg = make_cfg();
    const Frame f = make_frame(sync);
    gr::uwb::radar::reserve_radar_cir_core(f.rx.size(), scratch);

    // A successful call first, so every failure below is checked against a
    // scratch that still holds real taps.
    const RadarCirResult ok = run_ok(f.rx, f.sfd_start, cfg, scratch);
    BOOST_REQUIRE_EQUAL(ok.tap_count, kCirPre + kCirPost);
    const size_t taps = ok.tap_count;

    auto expect_fail_closed = [&](const std::vector<gr_complex>& rx,
                                  int64_t predicted,
                                  const RadarCirConfig& c,
                                  const char* label) {
        RadarCirResult out;
        const bool ok2 =
            radar_cir_one(rx.data(), rx.size(), predicted, c, scratch, out);
        BOOST_CHECK_MESSAGE(!ok2, label);
        BOOST_CHECK_MESSAGE(out.status != RadarCirStatus::Ok, label);
        BOOST_CHECK_MESSAGE(out.tap_count == size_t(0), label);
        BOOST_CHECK_MESSAGE(out.valid_repetitions == size_t(0), label);
        BOOST_CHECK_MESSAGE(out.peak_abs == 0.0f, label);
        BOOST_CHECK_MESSAGE(out.raw_l2_norm == 0.0f, label);
        // No stale taps may survive a failed call.
        BOOST_CHECK_MESSAGE(
            all_zero(scratch.cir.raw_taps, taps), label);
        BOOST_CHECK_MESSAGE(
            all_zero(scratch.cir.norm_taps, taps), label);
    };

    // (a) noise only: deterministic pseudo-random complex noise.
    {
        std::vector<gr_complex> noise(f.rx.size());
        uint32_t s = 0x12345678u;
        auto next = [&]() {
            s = s * 1664525u + 1013904223u;
            return static_cast<float>((s >> 8) & 0xFFFFu) / 65536.0f - 0.5f;
        };
        for (auto& v : noise)
            v = gr_complex(0.01f * next(), 0.01f * next());
        expect_fail_closed(noise, f.sfd_start, cfg, "noise only");
    }

    // (b) wrong code: the frame carries the code-10 SYNC symbol while the core
    // was prepared for code 9, so the target preamble is simply not there.
    {
        const auto sync10 = load_sync_pulse("reference_preamble_code10_998p4.cf32");
        const Frame other = make_frame(sync10);
        expect_fail_closed(other.rx, other.sfd_start, cfg, "wrong code");
    }

    // (c) wrong SFD: the frame carries a 4z1 SFD while the core searches 4z2.
    {
        const Frame other = make_frame(sync, kNSync, kPreGuard, "4z1");
        expect_fail_closed(other.rx, other.sfd_start, cfg, "wrong SFD");
    }

    // (d) missing prefix/halo: a timed echo whose CIR union does not fit in the
    // buffer.  The clip rule drops every window that does not fit, so a partial
    // fit degrades gracefully (fewer valid repetitions) while a buffer that
    // cannot hold even one window fails closed instead of reading outside it.
    {
        RadarCirConfig timed = cfg;
        timed.use_predicted_timing = true;
        const int64_t predicted = f.sfd_start;
        // Only 12 repetitions present: window k = 10 fits, k = 11 is clipped.
        const size_t n_partial =
            static_cast<size_t>(f.sync_origin) + 12 * 1016;
        std::vector<gr_complex> partial(f.rx.begin(),
                                        f.rx.begin() + n_partial);
        RadarCirResult out;
        BOOST_REQUIRE(radar_cir_one(partial.data(), partial.size(), predicted,
                                    timed, scratch, out));
        BOOST_CHECK(out.status == RadarCirStatus::Ok);
        BOOST_CHECK_EQUAL(out.valid_repetitions, size_t(1));
        BOOST_CHECK_EQUAL(out.tap_count, kCirPre + kCirPost);

        // Only the skipped repetitions present: no window can fit at all.
        const size_t n_none =
            static_cast<size_t>(f.sync_origin) + kCirSkip * 1016;
        std::vector<gr_complex> none(f.rx.begin(), f.rx.begin() + n_none);
        expect_fail_closed(none, predicted, timed, "missing halo");
    }

    // (e) an invalid CIR request (skip >= repetitions) fails closed before any
    // detection and publishes no coordinate.
    {
        RadarCirConfig bad = cfg;
        bad.cir_skip_initial = kNSync;
        RadarCirResult out;
        const bool ok2 =
            radar_cir_one(f.rx.data(), f.rx.size(), f.sfd_start, bad, scratch,
                          out);
        BOOST_CHECK(!ok2);
        BOOST_CHECK(out.status == RadarCirStatus::CirFailed);
        BOOST_CHECK_EQUAL(out.tap_count, size_t(0));
        BOOST_CHECK_EQUAL(out.sfd_start_sample, int64_t(-1));
        BOOST_CHECK_EQUAL(out.preamble_start_sample, int64_t(-1));
        BOOST_CHECK(all_zero(scratch.cir.raw_taps, taps));
    }
}

// ---------------------------------------------------------------------------
// 5. Legacy cross-check: shared kernel vs the legacy float64 scalar math.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_shared_cir_matches_legacy_scalar)
{
    const auto sync = load_sync_pulse("reference_preamble.bin");
    RadarCirCoreScratch scratch;
    BOOST_REQUIRE(prepare(sync, scratch));
    const RadarCirConfig cfg = make_cfg();
    const Frame f = make_frame(sync);
    gr::uwb::radar::reserve_radar_cir_core(f.rx.size(), scratch);

    const RadarCirResult shared = run_ok(f.rx, f.sfd_start, cfg, scratch);
    const auto shared_raw = copy_taps(scratch, shared.tap_count);
    const auto shared_norm = copy_norm(scratch, shared.tap_count);

    RadarCirScratch legacy_scratch;
    const int8_t* code = GetPreambleCode(9);
    BOOST_REQUIRE(code != nullptr);
    BOOST_REQUIRE(prepare_radar_cir_code(code, kQm35CodeLength, kCirPre,
                                         kCirPost, legacy_scratch));
    RadarCirEstimate legacy;
    BOOST_REQUIRE(estimate_radar_cir(
        f.rx.data(), f.rx.size(), shared.cir_axis_origin,
        kQm35SamplesPerSymbol, kCirPre, kCirPost, kCirSkip, kCirReps, kNSync,
        legacy, legacy_scratch));
    BOOST_REQUIRE_EQUAL(legacy.tap_count, shared.tap_count);
    BOOST_CHECK_EQUAL(legacy.peak_tap, shared.peak_tap);
    BOOST_CHECK_EQUAL(legacy.valid_repetitions, shared.valid_repetitions);

    std::vector<gr_complex> legacy_raw(
        legacy_scratch.raw_taps.begin(),
        legacy_scratch.raw_taps.begin() +
            static_cast<std::ptrdiff_t>(legacy.tap_count));
    std::vector<gr_complex> legacy_norm(
        legacy_scratch.norm_taps.begin(),
        legacy_scratch.norm_taps.begin() +
            static_cast<std::ptrdiff_t>(legacy.tap_count));
    const double e_raw = rel_l2(shared_raw, legacy_raw);
    const double e_norm = rel_l2(shared_norm, legacy_norm);
    BOOST_TEST_MESSAGE("shared vs legacy scalar: raw relL2="
                       << e_raw << " norm relL2=" << e_norm);
    BOOST_CHECK_LT(e_raw, 2e-5);
    BOOST_CHECK_LT(e_norm, 2e-5);
    BOOST_CHECK_CLOSE(shared.peak_abs, legacy.peak_abs, 1e-3);
    BOOST_CHECK_CLOSE(shared.raw_l2_norm, legacy.raw_l2_norm, 1e-3);
}
