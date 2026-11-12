/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Round-2 §B (SA-B): ROI prefix vs FULL-window PDU through the 65/32 PDU
 * resampler + Radar CIR estimator.  No hardware.
 *
 * WHAT THIS TEST PROVES
 * ---------------------
 * Given ONE deterministic FC32 native-rate (491.52 MS/s) window that holds a
 * REAL 64-SYNC code-9 4z2 frame, feeding
 *   FULL: sample_count = full_N, payload = whole window
 *   ROI : sample_count = full_N, payload = first K samples,
 *         published_samples = K
 * through two independent, identically-configured
 * UwbPduRationalResamplerCcf65_32 (FC32 input, FullWindow emit, 1 worker)
 * + UwbRadarCirEstimator (cir_pre=16, cir_post=100, skip=10, 64 reps,
 * 4z2/code 9) chains yields IDENTICAL CIR frames:
 *   status, predicted_sfd_start_sample, sfd_start_sample,
 *   preamble_start_sample, cir_origin_sample, peak_tap,
 *   cir_peak_metric, raw_l2_norm, raw taps, normalized taps.
 * This is asserted both for the predicted-timing path
 * (use_predicted_timing=true) and for the require-SFD path
 * (use_predicted_timing=false).
 *
 * THE FIXTURE IS A FRAME, NOT NOISE
 * ---------------------------------
 * The require-SFD path now runs the shared comm preamble front-end
 * (demod::acquire_preamble, PreamblePurpose::CirOnly), which needs a coherent
 * SYNC train: the tail re-track verifies every SYNC against a hard-coded 0.20
 * per-SYNC floor.  LCG noise cannot satisfy that, so the window is built from
 * a real modulated packet:
 *
 *   uwb_hrp_mod_core::modulate_one (998.4 MS/s work grid, code 9, 64 SYNC,
 *   4z2, 20-byte PSDU, Legacy pulse)
 *        --core::RationalResampler32_65Core (998.4 -> 491.52 MS/s)-->
 *   kFullNative-sample native window
 *
 * Geometry.  The estimator derives its prediction from the resampler's
 * MAPPED output metadata, not from the raw input metadata:
 *   window_start_sample = map(0)              = 42
 *   pre_guard_samples   = map(16) - map(0)    = 33
 *   predicted_sfd       = 42 + 33 + 0 + 64*1016 = 65099
 * with map(p) = round((p*65 + (T-1)/2)/32) and T = 2707 taps.  The frame's
 * SYNC origin therefore has to land on work sample map(16) = 75 and its SFD
 * on 65099.  The 32/65 core places the frame origin at native sample 21, so
 * the window starts kNativeHeadDrop = 5 native samples into the frame; that
 * is 10 work samples = 5 chips of the first SYNC symbol, and the detected
 * coordinates land on 74 / 65098 (map() rounds to whole work samples).
 *
 * WHY IT HOLDS (and what is assumed)
 * ----------------------------------
 * - The resampler maps its output window/pre_guard metadata from the
 *   DECLARED physical window (sample_count / pre_guard / capture /
 *   post_guard), not from the payload length, so FULL and ROI emit the same
 *   coordinate metadata.  "sample_count" is the physical native window
 *   length in both PDUs; only the payload and published_samples differ.
 * - The estimator only reads a bounded span around the predicted SFD
 *   (SFD search: [predicted-64, predicted+64] x 8128 samples; SYNC refine:
 *   nominal ±8 x 1016; CIR: origin + k*1016 - 16 for k=10..63, length 1131).
 *   K (40960 native) maps to 83283 work samples, which covers the whole read
 *   span plus thousands of native samples of margin over the FIR support
 *   (H = ceil(2707/65) = 42 input samples), so every output sample the
 *   estimator consumes is bit-identical between the two chains.
 * - FC32 input only: this QA does not exercise the SC16 scale policy (SA-A).
 * - The implementation rejects sfd_threshold / sync_refine_threshold <= 0
 *   (finite_positive gate), so the require-SFD variant keeps the tiny
 *   positive threshold (1e-6).  The fixture is a real frame, so detection
 *   succeeds on its own merit; the low threshold is kept only so that a
 *   failure here means "the ROI prefix does not reproduce the FULL window",
 *   never "the threshold was mistuned".  This test does NOT claim DW3000
 *   detection sensitivity.
 * - This proves "the ROI prefix covers the CIR / SFD read range" under the
 *   frozen metadata contract.  The safety upper bound for arbitrary
 *   hardware captures (where the ROI may not cover the require-SFD search)
 *   is the app-side full-window fallback (rectification §3.1 / §C), NOT
 *   this QA.
 *
 * Runtime is deterministic and well under 30 s (a handful of short FIR +
 * CIR runs, no top-block pacing beyond message waits).  If the testdata
 * template/taps are unavailable the cases skip cleanly with BOOST_TEST_MESSAGE.
 */

#include <boost/test/unit_test.hpp>

#include <gnuradio/blocks/message_debug.h>
#include <gnuradio/gr_complex.h>
#include <gnuradio/top_block.h>
#include <gnuradio/uwb/uwb_hrp_mod_core.h>
#include <gnuradio/uwb/uwb_phy_profile.h>
#include <gnuradio/uwb/uwb_pdu_rational_resampler_ccf_65_32.h>
#include <gnuradio/uwb/uwb_radar_cir_estimator_block.h>
#include <gnuradio/uwb/uwb_rational_resampler_core.h>
#include <pmt/pmt.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <string>
#include <thread>
#include <vector>

using gr::uwb::UwbPduRationalResamplerCcf65_32;
using gr::uwb::UwbRadarCirEstimator;
using gr_complex = std::complex<float>;

#ifndef UWB_TESTDATA_DIR
#define UWB_TESTDATA_DIR "../../../testdata"
#endif

namespace {

constexpr size_t kSps = gr::uwb::demod::kQm35SamplesPerSymbol; // 1016
constexpr size_t kCirPre = 16;
constexpr size_t kCirPost = 100;
constexpr size_t kSkip = 10;
constexpr size_t kSyncReps = 64;
constexpr size_t kSfdSymbols = 8; // "4z2"
constexpr int64_t kSfdMargin = 64;
constexpr int64_t kRefineMargin = 8;
constexpr double kNativeRate = 491.52e6; // resampler input rate (CG400)
constexpr double kOutputRate = UwbPduRationalResamplerCcf65_32::kOutputRateHz;
// FULL physical window (payload and sample_count).
constexpr size_t kFullNative = 65536;
// ROI prefix payload.  40960 native -> 83283 work samples at 65/32, which
// covers the whole SFD search + SYNC refine + CIR averaging read span.
constexpr size_t kRoiNative = 40960;
// The core rejects non-positive thresholds; keep a tiny positive value so the
// require-SFD code path runs.  The fixture is a real frame, so detection does
// not depend on this number.
constexpr float kLowThreshold = 1e-6f;
// Cross-resampler tap comparison tolerance (chains are expected bit-identical;
// this only absorbs a theoretical kernel-scheduling difference).
constexpr double kTapTolAbs = 1e-5;
// Real-frame fixture: 20-byte PSDU (18 data bytes + FCS) and the number of
// native samples the window starts into the frame (see the file header).
constexpr size_t kPsduBytes = 20;
constexpr size_t kNativeHeadDrop = 5;
// Work-grid length needed to fill the native window through the 32/65 core;
// the +100 covers the FIR tail (H = ceil(2707/32) = 85 input samples).
constexpr size_t kTxWorkLen = ((kFullNative + kNativeHeadDrop) * 65) / 32 + 100;

std::string
testdata_path(const std::string& rel)
{
    const std::string from_def = std::string(UWB_TESTDATA_DIR) + "/" + rel;
    {
        std::ifstream f(from_def, std::ios::binary);
        if (f)
            return from_def;
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
    return from_def;
}

bool
load_cf32(const std::string& path, std::vector<gr_complex>& out)
{
    std::ifstream f(path, std::ios::binary | std::ios::ate);
    if (!f)
        return false;
    const auto bytes = static_cast<size_t>(f.tellg());
    if (bytes == 0 || bytes % sizeof(gr_complex) != 0)
        return false;
    f.seekg(0);
    out.resize(bytes / sizeof(gr_complex));
    f.read(reinterpret_cast<char*>(out.data()),
           static_cast<std::streamsize>(bytes));
    return static_cast<bool>(f);
}

bool
load_f32(const std::string& path, std::vector<float>& out)
{
    std::ifstream f(path, std::ios::binary | std::ios::ate);
    if (!f)
        return false;
    const auto bytes = static_cast<size_t>(f.tellg());
    if (bytes == 0 || bytes % sizeof(float) != 0)
        return false;
    f.seekg(0);
    out.resize(bytes / sizeof(float));
    f.read(reinterpret_cast<char*>(out.data()),
           static_cast<std::streamsize>(bytes));
    return static_cast<bool>(f);
}

// Real-frame native fixture.  A 64-SYNC code-9 4z2 packet is modulated on the
// 998.4 MS/s work grid and converted to the 491.52 MS/s native grid with the
// same 32/65 core (and the same taps) the production TX path uses, so the
// require-SFD front-end sees a coherent SYNC train instead of noise.
//
// The frame is placed at work sample 0 of the TX segment (the resampler's
// zero history is then exactly the true signal) and the native window starts
// kNativeHeadDrop samples into it, which puts the SYNC origin on the work
// sample the estimator's mapped prediction names (map(16) = 75).
std::vector<gr_complex>
make_frame_window_native(const std::vector<float>& taps)
{
    const std::vector<uint8_t> psdu = gr::uwb::mod::make_random_psdu(
        kPsduBytes - 2, 4242u, /*append_fcs=*/true);
    BOOST_REQUIRE_EQUAL(psdu.size(), kPsduBytes);

    gr::uwb::mod::HrpModConfig cfg;
    cfg.code_index = 9;
    cfg.sync_repetitions = kSyncReps;
    cfg.sfd_mode = "4z2";

    const size_t frame_len =
        gr::uwb::mod::packet_samples_998p4(kSyncReps, kSfdSymbols, psdu.size());
    BOOST_REQUIRE_LE(frame_len, kTxWorkLen);

    gr::uwb::mod::HrpModScratch scratch;
    scratch.reserve(psdu.size(), frame_len);
    std::vector<gr_complex> tx(frame_len, gr_complex(0.0f, 0.0f));
    size_t tx_n = 0;
    BOOST_REQUIRE(gr::uwb::mod::modulate_one(psdu.data(), psdu.size(), cfg,
                                             scratch, tx.data(), tx.size(),
                                             tx_n));
    BOOST_REQUIRE_EQUAL(tx_n, frame_len);

    std::vector<gr_complex> work(kTxWorkLen, gr_complex(0.0f, 0.0f));
    std::copy(tx.begin(), tx.end(), work.begin());

    // 998.4 MS/s -> 491.52 MS/s (32/65), the production TX direction.
    gr::uwb::core::RationalResampler32_65Core down(taps.data(), taps.size());
    std::vector<gr_complex> native(kTxWorkLen, gr_complex(0.0f, 0.0f));
    const size_t native_n =
        down.process(work.data(), work.size(), native.data(), native.size())
            .produced;
    BOOST_REQUIRE_GE(native_n, kFullNative + kNativeHeadDrop);

    std::vector<gr_complex> win(kFullNative, gr_complex(0.0f, 0.0f));
    std::copy(native.begin() + kNativeHeadDrop,
              native.begin() + kNativeHeadDrop + kFullNative, win.begin());
    return win;
}

pmt::pmt_t
make_pdu_meta(int64_t sample_count,
              int64_t pre,
              int64_t cap,
              int64_t post,
              int64_t published,
              bool with_published)
{
    pmt::pmt_t m = pmt::make_dict();
    m = pmt::dict_add(m, pmt::mp("pulse_id"), pmt::from_uint64(7));
    m = pmt::dict_add(m, pmt::mp("schedule_index"), pmt::from_uint64(7));
    m = pmt::dict_add(m, pmt::mp("sample_rate"), pmt::from_double(kNativeRate));
    m = pmt::dict_add(m, pmt::mp("window_start_sample"), pmt::from_long(0));
    m = pmt::dict_add(m, pmt::mp("pre_guard_samples"), pmt::from_long(pre));
    m = pmt::dict_add(m, pmt::mp("capture_samples"), pmt::from_long(cap));
    m = pmt::dict_add(m, pmt::mp("post_guard_samples"), pmt::from_long(post));
    m = pmt::dict_add(m, pmt::mp("sample_count"), pmt::from_long(sample_count));
    m = pmt::dict_add(m, pmt::mp("calibration_delay_native_samples"),
                      pmt::from_double(0.0));
    m = pmt::dict_add(m, pmt::mp("sync_repetitions"),
                      pmt::from_long(static_cast<long>(kSyncReps)));
    m = pmt::dict_add(m, pmt::mp("sfd_mode"), pmt::mp("4z2"));
    m = pmt::dict_add(m, pmt::mp("code_index"), pmt::from_long(9));
    m = pmt::dict_add(m, pmt::mp("source"), pmt::mp("synthetic"));
    m = pmt::dict_add(m, pmt::mp("sample_format"), pmt::mp("fc32"));
    if (with_published)
        m = pmt::dict_add(m, pmt::mp("published_samples"),
                          pmt::from_long(published));
    return m;
}

// FULL: payload = whole physical window; sample_count = whole window.
pmt::pmt_t
make_full_pdu(const std::vector<gr_complex>& x)
{
    const int64_t pre = static_cast<int64_t>(kCirPre);
    const int64_t n = static_cast<int64_t>(x.size());
    const int64_t cap = n - pre;
    pmt::pmt_t meta = make_pdu_meta(n, pre, cap, /*post=*/0, n, false);
    return pmt::cons(meta, pmt::init_c32vector(x.size(), x.data()));
}

// ROI: payload = first k samples, but sample_count still describes the
// physical window; published_samples records the emitted prefix length.
pmt::pmt_t
make_roi_pdu(const std::vector<gr_complex>& x, size_t k)
{
    const int64_t pre = static_cast<int64_t>(kCirPre);
    const int64_t n = static_cast<int64_t>(x.size());
    const int64_t cap = n - pre;
    pmt::pmt_t meta =
        make_pdu_meta(n, pre, cap, /*post=*/0, static_cast<int64_t>(k), true);
    return pmt::cons(meta, pmt::init_c32vector(k, x.data()));
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

std::string
symbol_str(pmt::pmt_t dict, const char* key)
{
    pmt::pmt_t v = pmt::dict_ref(dict, pmt::mp(key), pmt::PMT_NIL);
    if (pmt::is_symbol(v))
        return pmt::symbol_to_string(v);
    return {};
}

struct Frame {
    std::string status;
    int64_t predicted = -1;
    int64_t sfd_start = -1;
    int64_t preamble_start = -1;
    int64_t cir_origin = -1;
    uint64_t peak_tap = 0;
    double peak_metric = 0.0;
    double raw_l2 = 0.0;
    std::vector<gr_complex> raw;
    std::vector<gr_complex> norm;
};

Frame
extract_frame(pmt::pmt_t msg)
{
    Frame f;
    BOOST_REQUIRE(pmt::is_pair(msg));
    pmt::pmt_t meta = pmt::car(msg);
    pmt::pmt_t data = pmt::cdr(msg);
    BOOST_REQUIRE(pmt::is_dict(meta));

    f.status = symbol_str(meta, "status");
    f.predicted = pmt::to_long(pmt::dict_ref(
        meta, pmt::mp("predicted_sfd_start_sample"), pmt::from_long(-1)));
    f.sfd_start = pmt::to_long(
        pmt::dict_ref(meta, pmt::mp("sfd_start_sample"), pmt::from_long(-1)));
    f.preamble_start = pmt::to_long(pmt::dict_ref(
        meta, pmt::mp("preamble_start_sample"), pmt::from_long(-1)));
    f.cir_origin = pmt::to_long(pmt::dict_ref(
        meta, pmt::mp("cir_origin_sample"), pmt::from_long(-1)));
    f.peak_tap = pmt::to_uint64(
        pmt::dict_ref(meta, pmt::mp("peak_tap"), pmt::from_uint64(0)));
    f.peak_metric = pmt::to_double(
        pmt::dict_ref(meta, pmt::mp("cir_peak_metric"), pmt::from_double(0)));
    f.raw_l2 = pmt::to_double(
        pmt::dict_ref(meta, pmt::mp("raw_l2_norm"), pmt::from_double(0)));

    size_t n = 0;
    const gr_complex* p = pmt::c32vector_elements(data, n);
    f.raw.assign(p, p + n);

    pmt::pmt_t nv = pmt::dict_ref(meta, pmt::mp("normalized_taps"),
                                  pmt::PMT_NIL);
    if (pmt::is_c32vector(nv)) {
        size_t nn = 0;
        const gr_complex* np = pmt::c32vector_elements(nv, nn);
        f.norm.assign(np, np + nn);
    }
    return f;
}

struct ChainOut {
    pmt::pmt_t resampler_meta = pmt::PMT_NIL;
    std::vector<gr_complex> resampled;
    Frame frame;
};

// One real message chain: 65/32 resampler (packet) -> estimator (rx), with
// both stages tapped by message_debug for assertions.
ChainOut
run_chain(const std::vector<float>& taps,
          const std::string& tmpl_path,
          pmt::pmt_t pdu,
          bool use_predicted_timing,
          float threshold)
{
    auto resamp = UwbPduRationalResamplerCcf65_32::make_from_taps(
        taps,
        kOutputRate,
        /*validate_input_rate=*/true,
        UwbPduRationalResamplerCcf65_32::EmitPolicy::FullWindow,
        /*max_input_samples=*/kFullNative,
        /*num_workers=*/1);
    auto est = UwbRadarCirEstimator::make(tmpl_path,
                                          kSyncReps,
                                          "4z2",
                                          9,
                                          kCirPre,
                                          kCirPost,
                                          kSkip,
                                          /*cir_repetitions=*/0, // auto
                                          kSfdMargin,
                                          kRefineMargin,
                                          threshold,
                                          threshold,
                                          /*emit_normalized=*/true,
                                          /*queue_capacity=*/8,
                                          use_predicted_timing);
    auto dbg_res = gr::blocks::message_debug::make();
    auto dbg_cir = gr::blocks::message_debug::make();
    auto tb = gr::make_top_block("qa_uwb_pdu_roi_cir_65_32");
    tb->msg_connect(resamp, "packet", est, "rx");
    tb->msg_connect(resamp, "packet", dbg_res, "store");
    tb->msg_connect(est, "cir", dbg_cir, "store");
    tb->start();
    resamp->_post(pmt::mp("packet"), pdu);
    BOOST_REQUIRE(wait_until([&] {
        return dbg_cir->num_messages() >= 1u && est->drained();
    }));
    tb->stop();
    tb->wait();

    BOOST_REQUIRE_EQUAL(resamp->pdus_received(), 1u);
    BOOST_REQUIRE_EQUAL(resamp->pdus_emitted(), 1u);
    BOOST_REQUIRE_EQUAL(resamp->pdus_dropped(), 0u);
    BOOST_REQUIRE_EQUAL(est->invalid_inputs(), 0u);
    BOOST_REQUIRE_EQUAL(est->pdus_completed() + est->pdus_failed(), 1u);
    BOOST_REQUIRE_EQUAL(dbg_res->num_messages(), 1u);
    BOOST_REQUIRE_EQUAL(dbg_cir->num_messages(), 1u);

    ChainOut out;
    out.resampler_meta = pmt::car(dbg_res->get_message(0));
    {
        size_t n = 0;
        const gr_complex* p =
            pmt::c32vector_elements(pmt::cdr(dbg_res->get_message(0)), n);
        out.resampled.assign(p, p + n);
    }
    out.frame = extract_frame(dbg_cir->get_message(0));
    return out;
}

void
check_i64(int64_t a, int64_t b, const std::string& tag, const char* what)
{
    BOOST_CHECK_MESSAGE(a == b,
                        tag << " " << what << ": " << a << " vs " << b);
}

void
check_f64(double a, double b, const std::string& tag, const char* what)
{
    const double tol =
        1e-9 * std::max(1.0, std::max(std::abs(a), std::abs(b)));
    BOOST_CHECK_MESSAGE(std::abs(a - b) <= tol,
                        tag << " " << what << ": " << a << " vs " << b);
}

void
check_vec(const std::vector<gr_complex>& a,
          const std::vector<gr_complex>& b,
          const std::string& tag,
          const char* what)
{
    BOOST_REQUIRE_MESSAGE(a.size() == b.size(),
                          tag << " " << what << " size: " << a.size()
                              << " vs " << b.size());
    double max_diff = 0.0;
    double max_mag = 0.0;
    size_t arg = 0;
    for (size_t i = 0; i < a.size(); ++i) {
        const double d = std::abs(a[i] - b[i]);
        if (d > max_diff) {
            max_diff = d;
            arg = i;
        }
        max_mag = std::max(max_mag, static_cast<double>(std::abs(a[i])));
    }
    BOOST_CHECK_MESSAGE(max_diff <= kTapTolAbs * std::max(1.0, max_mag),
                        tag << " " << what << " max|diff|=" << max_diff
                            << " at " << arg << " ref_mag=" << max_mag);
    BOOST_TEST_MESSAGE(tag << " " << what << " max|diff|=" << max_diff
                           << " ref_mag=" << max_mag);
}

void
check_frames_equal(const Frame& a, const Frame& b, const std::string& tag)
{
    BOOST_CHECK_MESSAGE(a.status == b.status,
                        tag << " status: " << a.status << " vs " << b.status);
    check_i64(a.predicted, b.predicted, tag, "predicted_sfd_start_sample");
    check_i64(a.sfd_start, b.sfd_start, tag, "sfd_start_sample");
    check_i64(a.preamble_start, b.preamble_start, tag, "preamble_start_sample");
    check_i64(a.cir_origin, b.cir_origin, tag, "cir_origin_sample");
    check_i64(static_cast<int64_t>(a.peak_tap),
              static_cast<int64_t>(b.peak_tap), tag, "peak_tap");
    check_f64(a.peak_metric, b.peak_metric, tag, "cir_peak_metric");
    check_f64(a.raw_l2, b.raw_l2, tag, "raw_l2_norm");
    check_vec(a.raw, b.raw, tag, "raw_taps");
    check_vec(a.norm, b.norm, tag, "normalized_taps");
}

bool
load_assets(std::vector<float>& taps, std::string& tmpl_path)
{
    const std::string taps_path =
        testdata_path("resampler_65_32/taps_quality_minorder.txt");
    const std::string template_path =
        testdata_path("uwb_radar/sync_template_998p4.cf32");
    std::vector<gr_complex> tmpl;
    if (!load_f32(taps_path, taps) || taps.empty())
        return false;
    if (!load_cf32(template_path, tmpl) || tmpl.size() != kSps)
        return false;
    tmpl_path = template_path;
    return true;
}

void
compare_full_vs_roi(bool use_predicted_timing)
{
    std::vector<float> taps;
    std::string tmpl_path;
    if (!load_assets(taps, tmpl_path)) {
        BOOST_TEST_MESSAGE(
            "SKIP qa_uwb_pdu_roi_cir_65_32: testdata template/taps not "
            "available (set UWB_TESTDATA_DIR); expected "
            "uwb_radar/sync_template_998p4.cf32 and "
            "resampler_65_32/taps_quality_minorder.txt");
        return;
    }

    const std::string tag = use_predicted_timing ? "predicted_timing"
                                                 : "require_sfd";
    const float threshold = use_predicted_timing ? 0.3f : kLowThreshold;

    const std::vector<gr_complex> x = make_frame_window_native(taps);
    const pmt::pmt_t full_pdu = make_full_pdu(x);
    const pmt::pmt_t roi_pdu = make_roi_pdu(x, kRoiNative);

    const ChainOut full =
        run_chain(taps, tmpl_path, full_pdu, use_predicted_timing, threshold);
    const ChainOut roi =
        run_chain(taps, tmpl_path, roi_pdu, use_predicted_timing, threshold);

    BOOST_REQUIRE_MESSAGE(full.frame.status == "ok",
                          tag << " FULL status=" << full.frame.status);
    BOOST_REQUIRE_MESSAGE(roi.frame.status == "ok",
                          tag << " ROI status=" << roi.frame.status);
    BOOST_REQUIRE_EQUAL(full.frame.raw.size(), kCirPre + kCirPost);
    BOOST_REQUIRE_EQUAL(full.frame.norm.size(), kCirPre + kCirPost);

    // The ROI prefix must cover the complete estimator read range.  If this
    // fails the comparison below is not meaningful (the ROI simply does not
    // carry enough samples); this is exactly the safety property the app's
    // full-window fallback protects.
    const int64_t sfd_len = static_cast<int64_t>(kSfdSymbols * kSps);
    const int64_t sfd_need =
        full.frame.predicted + kSfdMargin + sfd_len; // idx + 1
    const int64_t wlen =
        static_cast<int64_t>(kSps + kCirPre + kCirPost - 1);
    const int64_t cir_need =
        full.frame.cir_origin +
        static_cast<int64_t>(kSyncReps - 1) * static_cast<int64_t>(kSps) -
        static_cast<int64_t>(kCirPre) + wlen;
    BOOST_TEST_MESSAGE(tag << ": full_resampled=" << full.resampled.size()
                           << " roi_resampled=" << roi.resampled.size()
                           << " predicted=" << full.frame.predicted
                           << " origin=" << full.frame.cir_origin
                           << " sfd_need=" << sfd_need
                           << " cir_need=" << cir_need);
    BOOST_REQUIRE_GE(static_cast<int64_t>(roi.resampled.size()), sfd_need);
    BOOST_REQUIRE_GE(static_cast<int64_t>(roi.resampled.size()), cir_need);

    // The fixture must be a real frame sitting on the predicted coordinate,
    // otherwise "FULL == ROI" would only prove that two chains agree on
    // garbage.  The prediction is built from the resampler's mapped metadata
    // (map(16) + 64*1016 = 65099); map() rounds to whole work samples, so the
    // detected SFD and SYNC origin may sit one sample away.
    BOOST_TEST_MESSAGE(tag << ": sfd=" << full.frame.sfd_start
                           << " origin=" << full.frame.preamble_start
                           << " peak_tap=" << full.frame.peak_tap
                           << " peak_metric=" << full.frame.peak_metric);
    BOOST_REQUIRE_LE(std::llabs(full.frame.sfd_start - full.frame.predicted),
                     2);
    BOOST_REQUIRE_LE(std::llabs(full.frame.preamble_start -
                                full.frame.cir_origin),
                     2);

    check_frames_equal(full.frame, roi.frame, tag);

    // Same declared physical window => identical mapped coordinate metadata.
    BOOST_REQUIRE(pmt::is_dict(full.resampler_meta));
    BOOST_REQUIRE(pmt::is_dict(roi.resampler_meta));
    check_i64(pmt::to_long(pmt::dict_ref(full.resampler_meta,
                                         pmt::mp("window_start_sample"),
                                         pmt::from_long(-1))),
              pmt::to_long(pmt::dict_ref(roi.resampler_meta,
                                         pmt::mp("window_start_sample"),
                                         pmt::from_long(-1))),
              tag, "output window_start_sample");
    check_i64(pmt::to_long(pmt::dict_ref(full.resampler_meta,
                                         pmt::mp("pre_guard_samples"),
                                         pmt::from_long(-1))),
              pmt::to_long(pmt::dict_ref(roi.resampler_meta,
                                         pmt::mp("pre_guard_samples"),
                                         pmt::from_long(-1))),
              tag, "output pre_guard_samples");

    // published_samples passthrough is SA-A's change; assert it when present
    // (ROI visibility) but do not fail if SA-A is not merged in this tree.
    if (pmt::dict_has_key(roi.resampler_meta, pmt::mp("published_samples"))) {
        check_i64(pmt::to_long(pmt::dict_ref(roi.resampler_meta,
                                             pmt::mp("published_samples"),
                                             pmt::from_long(-1))),
                  static_cast<int64_t>(kRoiNative), tag,
                  "output published_samples");
    } else {
        BOOST_TEST_MESSAGE(
            tag << ": published_samples not forwarded on output meta "
                   "(SA-A passthrough not present)");
    }
}

} // namespace

// Predicted-timing path (timed monostatic echo: no SFD gate).  The CIR is
// taken at the predicted origin, so this case proves the ROI prefix carries
// the whole CIR union of a real frame.
BOOST_AUTO_TEST_CASE(test_roi_prefix_matches_full_predicted_timing)
{
    compare_full_vs_roi(/*use_predicted_timing=*/true);
}

// require-SFD semantic (use_predicted_timing=false).  The shared comm preamble
// front-end must detect the real 64-SYNC frame (tail re-track with the 0.20
// per-SYNC floor), and the ROI prefix must reproduce the FULL window's SFD,
// SYNC origin and CIR exactly.
BOOST_AUTO_TEST_CASE(test_roi_prefix_matches_full_require_sfd)
{
    compare_full_vs_roi(/*use_predicted_timing=*/false);
}
