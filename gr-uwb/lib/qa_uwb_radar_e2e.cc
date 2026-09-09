/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Phase-A offline end-to-end QA (Radar Step 9).
 *
 * Paths under test (docs/phase1/开发计划_UWB自发自收Radar.md §12), real
 * block-to-block message chains, no test-side metadata overrides:
 *
 *   998.4 direct : packet source (998.4) → UwbLoopbackEcho (work channel)
 *                  → UwbRadarCirEstimator → UwbCirWriter
 *   native 65/48 : packet source (native 737.28) → UwbLoopbackEcho
 *                  (native channel) → UwbPduRationalResamplerCcf65_48
 *                  (validate_input_rate=true) → estimator → writer
 *
 * Unified coordinate/calibration conventions asserted here:
 *
 *   Predicted SFD (estimator):
 *     predicted = window_start_sample + pre_guard + round(cal_work)
 *                 + sync_reps * 1016
 *   The 65/48 resampler emits the group-delay-centered
 *   window_start_sample = map(window_start_native) and maps pre_guard as
 *   map(ws+pre)-map(ws); adding the window start back removes the FIR head
 *   so the native chain prediction lands within ±2 work samples of the
 *   true SFD (not margin-absorbed).
 *
 *   Native chain ground truth (pre_guard 1475 native = 2 µs, D = 37
 *   native, native SYNC span ceil(64*1016*48/65) = 48018):
 *     predicted = map(1475) + 50 + 65024 = 67100
 *     true SFD  = map(1475 + 37 + 48018) = 67100   (exact)
 *     cir_origin = predicted - 65024 = 2076 = map(1512) (true origin)
 *     peak_tap ≈ (preamble + 2) - (cir_origin - 16) = 18
 *   where map(p) = round((p*65 + (T-1)/2)/48) with T = 2707 taps.
 *
 *   CIR axis / zero delay: the core anchors the CIR axis at the predicted
 *   SYNC origin (which includes the calibration delay), so
 *     zero_delay_tap = cir_pre = 16
 *   and the calibrated leakage reads peak_tap - zero_delay_tap = +2 (the
 *   known pulse-shape offset from the MATLAB golden).  A target at extra
 *   delay τ beyond the calibrated chain lands at tap 16 + τ, independent
 *   of the per-pulse channel delay.
 *
 *   Direct path: same relationships with cal == D == 37 (golden
 *   rx_delay_int_998p4 coordinates: sfd 67058, origin 2034).
 *
 * Soak runtime: test e2e_soak_200pps_30s runs a real
 * UWB_E2E_SOAK_SECONDS (default 30) wall-clock replay; the CTest
 * registration should use the default (30 s) per the Phase-A exit
 * condition.  Set the env var to shorten local iterations.
 */

#include <boost/test/unit_test.hpp>

#include <gnuradio/blocks/message_debug.h>
#include <gnuradio/gr_complex.h>
#include <gnuradio/top_block.h>
#include <gnuradio/uwb/uwb_cir_writer.h>
#include <gnuradio/uwb/uwb_loopback_echo.h>
#include <gnuradio/uwb/uwb_phy_profile.h>
#include <gnuradio/uwb/uwb_pdu_rational_resampler_ccf_65_48.h>
#include <gnuradio/uwb/uwb_pdu_rational_resampler_ccf_65_32.h>
#include <gnuradio/uwb/uwb_radar_cir_core.h>
#include <gnuradio/uwb/uwb_radar_cir_estimator_block.h>
#include <gnuradio/uwb/uwb_radar_packet_source.h>
#include <gnuradio/uwb/uwb_radar_pdu_meta.h>
#include <pmt/pmt.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>

using gr::uwb::UwbCirWriter;
using gr::uwb::UwbLoopbackEcho;
using gr::uwb::UwbPduRationalResamplerCcf65_48;
using gr::uwb::UwbPduRationalResamplerCcf65_32;
using gr::uwb::UwbRadarCirEstimator;
using gr::uwb::UwbRadarPacketSource;
using gr::uwb::radar::prepare_radar_cir_core;
using gr::uwb::radar::radar_cir_one;
using gr::uwb::radar::RadarCirConfig;
using gr::uwb::radar::RadarCirCoreScratch;
using gr::uwb::radar::RadarCirResult;
using gr_complex = std::complex<float>;

#ifndef UWB_TESTDATA_DIR
#define UWB_TESTDATA_DIR "../../../testdata"
#endif

namespace {

constexpr size_t kSps = gr::uwb::demod::kQm35SamplesPerSymbol; // 1016
constexpr size_t kCirPre = 16;
constexpr size_t kCirPost = 100;
constexpr size_t kTaps = kCirPre + kCirPost; // 116
constexpr size_t kPreGuardWork = 1997;       // golden rx pre-guard (work)
constexpr size_t kPreGuardNative = 1475;     // 2 µs at 737.28 MS/s
constexpr size_t kTail = 4096;
constexpr double kFsWork = 998.4e6;
constexpr double kFsNative = 737.28e6;
constexpr double kDInt = 37.0; // golden delay_int channel delay

// Canonical 64-SYNC golden coordinates (testdata/uwb_radar/metadata.json,
// coordinates_0based.rx_clean_998p4 / rx_delay_int_998p4).
constexpr int64_t kSfdClean = 67021;  // = 1997 + 64*1016
constexpr int64_t kOriginClean = 1997;
constexpr int64_t kSfdD37 = 67058;    // = 1997 + 37 + 64*1016
constexpr int64_t kOriginD37 = 2034;
constexpr int64_t kPeakClean = 18;    // = 16 + pulse-shape offset 2

// Native 64-SYNC golden (testdata/uwb_radar/tx_737p28.cf32).  Native SYNC
// span convention: ceil(64*1016*48/65) = 48018.
constexpr size_t kNativeTxLen = 140982;
constexpr int64_t kSyncNative64 = 48018;
constexpr double kFsCg400 = 491.52e6;
constexpr size_t kPreGuardCg400 = 983; // 2 µs at 491.52 MS/s
constexpr size_t kNativeTxLenCg400 = 93988;
constexpr int64_t kSyncNative64Cg400 = 32012; // ceil(64*1016*32/65)
constexpr double kDIntCg400 = 25.0;           // ~50.9 ns on the CG400 grid

gr_complex
polar_gain(double mag, double phase)
{
    return gr_complex(static_cast<float>(mag * std::cos(phase)),
                      static_cast<float>(mag * std::sin(phase)));
}

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

std::vector<float>
load_taps_f32(const std::string& path)
{
    std::ifstream f(path, std::ios::binary | std::ios::ate);
    BOOST_REQUIRE_MESSAGE(f, "cannot open taps file " + path);
    const auto bytes = static_cast<size_t>(f.tellg());
    BOOST_REQUIRE_EQUAL(bytes % sizeof(float), 0u);
    f.seekg(0);
    std::vector<float> v(bytes / sizeof(float));
    f.read(reinterpret_cast<char*>(v.data()),
           static_cast<std::streamsize>(bytes));
    return v;
}

std::string
make_out_dir(const std::string& tag)
{
    const std::string dir = (std::filesystem::temp_directory_path() /
                             ("uwb_qa_radar_e2e_" + tag))
                                .string();
    std::filesystem::remove_all(dir);
    std::filesystem::create_directories(dir);
    return dir;
}

std::string
write_template_file(const std::vector<gr_complex>& tx, const std::string& tag)
{
    BOOST_REQUIRE_GE(tx.size(), kSps);
    const std::string dir =
        (std::filesystem::temp_directory_path() /
         ("uwb_qa_radar_e2e_tmpl_" + tag))
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

std::vector<std::string>
read_lines(const std::string& path)
{
    std::ifstream f(path);
    std::vector<std::string> lines;
    std::string line;
    while (std::getline(f, line)) {
        if (!line.empty())
            lines.push_back(line);
    }
    return lines;
}

bool
read_bytes(const std::string& path, std::vector<uint8_t>& out)
{
    std::ifstream f(path, std::ios::binary);
    if (!f)
        return false;
    f.seekg(0, std::ios::end);
    const auto n = static_cast<size_t>(f.tellg());
    f.seekg(0, std::ios::beg);
    out.resize(n);
    f.read(reinterpret_cast<char*>(out.data()),
           static_cast<std::streamsize>(n));
    return f.good() || f.eof();
}

bool
parse_num(const std::string& json, const char* key, double& out)
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

bool
parse_i64(const std::string& json, const char* key, int64_t& out)
{
    double v = 0.0;
    if (!parse_num(json, key, v))
        return false;
    out = static_cast<int64_t>(std::llround(v));
    return true;
}

bool
parse_u64(const std::string& json, const char* key, uint64_t& out)
{
    int64_t v = 0;
    if (!parse_i64(json, key, v) || v < 0)
        return false;
    out = static_cast<uint64_t>(v);
    return true;
}

bool
parse_str(const std::string& json, const char* key, std::string& out)
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

bool
parse_bool(const std::string& json, const char* key, bool& out)
{
    const std::string pat = std::string("\"") + key + "\"";
    auto pos = json.find(pat);
    if (pos == std::string::npos)
        return false;
    pos = json.find(':', pos + pat.size());
    if (pos == std::string::npos)
        return false;
    if (json.compare(pos + 1, 4, "true") == 0) {
        out = true;
        return true;
    }
    if (json.compare(pos + 1, 5, "false") == 0) {
        out = false;
        return true;
    }
    return false;
}

struct JsonlLine {
    bool parsed = false;
    std::string status;
    std::string source;
    uint64_t pulse_id = 0;
    uint64_t tap_count = 0;
    uint64_t file_offset = 0;
    double sample_rate = 0.0;
    double cal_work = -1.0;
    int64_t zero_delay = -9999;
    int64_t peak_tap = -1;
    int64_t sfd = -9999;
    int64_t preamble = -9999;
    int64_t cir_origin = -9999;
    int64_t predicted = -9999;
    bool sfd_ok = false;
    bool timing_ok = false;
};

JsonlLine
parse_jsonl(const std::string& line)
{
    JsonlLine j;
    j.parsed = true;
    j.status = parse_str(line, "status", j.status) ? j.status : std::string();
    (void)parse_str(line, "source", j.source);
    (void)parse_u64(line, "pulse_id", j.pulse_id);
    (void)parse_u64(line, "tap_count", j.tap_count);
    (void)parse_u64(line, "file_offset_taps", j.file_offset);
    (void)parse_num(line, "sample_rate", j.sample_rate);
    (void)parse_num(line, "calibration_delay_work_samples", j.cal_work);
    (void)parse_i64(line, "zero_delay_tap", j.zero_delay);
    (void)parse_i64(line, "peak_tap", j.peak_tap);
    (void)parse_i64(line, "sfd_start_sample", j.sfd);
    (void)parse_i64(line, "preamble_start_sample", j.preamble);
    (void)parse_i64(line, "cir_origin_sample", j.cir_origin);
    (void)parse_i64(line, "predicted_sfd_start_sample", j.predicted);
    (void)parse_bool(line, "sfd_ok", j.sfd_ok);
    (void)parse_bool(line, "timing_ok", j.timing_ok);
    return j;
}

double
rel_l2(const std::vector<gr_complex>& a, const std::vector<gr_complex>& b)
{
    BOOST_REQUIRE_EQUAL(a.size(), b.size());
    double num = 0.0, den = 0.0;
    for (size_t i = 0; i < a.size(); ++i) {
        const gr_complex d = a[i] - b[i];
        num += static_cast<double>(d.real()) * d.real() +
               static_cast<double>(d.imag()) * d.imag();
        den += static_cast<double>(b[i].real()) * b[i].real() +
               static_cast<double>(b[i].imag()) * b[i].imag();
    }
    if (den == 0.0)
        return num == 0.0 ? 0.0 : 1.0;
    return std::sqrt(num / den);
}

double
l2_norm(const std::vector<gr_complex>& a)
{
    double s = 0.0;
    for (const auto& v : a)
        s += static_cast<double>(v.real()) * v.real() +
             static_cast<double>(v.imag()) * v.imag();
    return std::sqrt(s);
}

// Peak-tap relationship on the estimator's CIR axis: tap j ↔ sample
// (cir_origin - cir_pre + j); the main path sits pulse-shape-offset +2
// after the RX-refined SYNC origin (golden peak_tap_measured_clean = 18).
int64_t
expected_peak_tap(int64_t preamble_start, int64_t cir_origin)
{
    return (preamble_start + 2) - (cir_origin - static_cast<int64_t>(kCirPre));
}

// ---- process RSS (no unbounded-growth check) ----------------------------
bool
read_rss_bytes(size_t* out)
{
    std::ifstream f("/proc/self/statm");
    if (!f)
        return false;
    unsigned long total_pages = 0;
    unsigned long resident_pages = 0;
    if (!(f >> total_pages >> resident_pages))
        return false;
    const long page = ::sysconf(_SC_PAGESIZE);
    if (page <= 0)
        return false;
    *out = static_cast<size_t>(resident_pages) * static_cast<size_t>(page);
    return true;
}

// ---- wait helpers --------------------------------------------------------
template <typename Pred>
bool
wait_until(Pred&& pred, long timeout_ms = 60000)
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

// ---- direct-core reference ------------------------------------------------
RadarCirConfig
direct_cfg(size_t sync_reps)
{
    RadarCirConfig cfg;
    cfg.sync_repetitions = sync_reps;
    cfg.samples_per_symbol = kSps;
    cfg.sfd_mode = "4z2";
    cfg.sfd_search_margin = 64;
    cfg.sync_refine_margin = 8;
    cfg.sfd_threshold = 0.3f;
    cfg.sync_refine_threshold = 0.3f;
    cfg.cir_pre = kCirPre;
    cfg.cir_post = kCirPost;
    cfg.cir_skip_initial = 10;
    cfg.cir_repetitions = sync_reps - 10;
    return cfg;
}

void
direct_core_reference(const std::vector<gr_complex>& tx,
                      const std::vector<gr_complex>& rx,
                      size_t sync_reps,
                      int64_t predicted,
                      std::vector<gr_complex>& raw,
                      std::vector<gr_complex>& norm)
{
    RadarCirCoreScratch scratch;
    BOOST_REQUIRE(prepare_radar_cir_core(direct_cfg(sync_reps), tx.data(),
                                         kSps, kCirPre, kCirPost, scratch));
    RadarCirResult ref;
    BOOST_REQUIRE(radar_cir_one(rx.data(), rx.size(), predicted,
                                direct_cfg(sync_reps), scratch, ref));
    BOOST_REQUIRE_EQUAL(ref.tap_count, kTaps);
    raw.assign(scratch.cir.raw_taps.begin(),
               scratch.cir.raw_taps.begin() + static_cast<std::ptrdiff_t>(
                                                 ref.tap_count));
    norm.assign(scratch.cir.norm_taps.begin(),
                scratch.cir.norm_taps.begin() + static_cast<std::ptrdiff_t>(
                                                  ref.tap_count));
}

pmt::pmt_t
rx_meta(uint64_t pulse_id, int64_t pre_guard, double cal_native)
{
    pmt::pmt_t meta = pmt::make_dict();
    meta = pmt::dict_add(meta, pmt::mp("pulse_id"),
                         pmt::from_uint64(pulse_id));
    meta = pmt::dict_add(meta, pmt::mp("schedule_index"),
                         pmt::from_uint64(pulse_id));
    meta = pmt::dict_add(meta, pmt::mp("sample_rate"),
                         pmt::from_double(kFsWork));
    meta = pmt::dict_add(meta, pmt::mp("pre_guard_samples"),
                         pmt::from_long(pre_guard));
    meta = pmt::dict_add(meta, pmt::mp("calibration_delay_native_samples"),
                         pmt::from_double(cal_native));
    meta = pmt::dict_add(meta, pmt::mp("sync_repetitions"), pmt::from_long(64));
    meta = pmt::dict_add(meta, pmt::mp("sfd_mode"), pmt::mp("4z2"));
    meta = pmt::dict_add(meta, pmt::mp("code_index"), pmt::from_long(9));
    meta = pmt::dict_add(meta, pmt::mp("tx_time_full"), pmt::from_long(0));
    meta = pmt::dict_add(meta, pmt::mp("tx_time_frac"),
                         pmt::from_double(0.005 * static_cast<double>(
                                                          pulse_id)));
    meta = pmt::dict_add(meta, pmt::mp("rx_time_full"), pmt::from_long(0));
    meta = pmt::dict_add(meta, pmt::mp("rx_time_frac"),
                         pmt::from_double(0.005 * static_cast<double>(
                                                          pulse_id) -
                                          2e-6));
    meta = pmt::dict_add(meta, pmt::mp("num_delay_samps"), pmt::from_long(0));
    meta = pmt::dict_add(meta, pmt::mp("calibration_id"), pmt::mp("qa-e2e"));
    meta = pmt::dict_add(meta, pmt::mp("source"), pmt::mp("loopback"));
    meta = pmt::dict_add(meta, pmt::mp("uhd_error"), pmt::mp("none"));
    return meta;
}

// ---- chain runners ---------------------------------------------------------

struct RunResult {
    std::vector<gr_complex> rx; // channel output (= estimator input)
};

/**
 * 998.4 direct chain: packet source → loopback → estimator → writer.
 * `n_frames` emits are posted to the source (pulse ids 0..n-1).
 */
RunResult
run_direct(const std::string& tx_path,
           const std::vector<double>& delays,
           const std::vector<gr_complex>& gains,
           float noise_std,
           uint32_t seed,
           const std::string& tmpl_path,
           size_t sync_reps,
           const std::string& out_dir,
           bool write_norm,
           size_t n_frames,
           size_t est_queue_cap = 16)
{
    RunResult res;
    auto src = UwbRadarPacketSource::make(tx_path, kFsWork);
    auto echo = UwbLoopbackEcho::make(
        kPreGuardWork, kTail, delays, gains, noise_std, seed);
    auto est = UwbRadarCirEstimator::make(tmpl_path,
                                          sync_reps,
                                          "4z2",
                                          9,
                                          kCirPre,
                                          kCirPost,
                                          10,
                                          0, // auto repetitions
                                          64,
                                          8,
                                          0.3f,
                                          0.3f,
                                          true,
                                          est_queue_cap);
    auto w = UwbCirWriter::make(out_dir, "cir", write_norm, 64);
    auto dbg = gr::blocks::message_debug::make();
    auto tb = gr::make_top_block("qa_radar_e2e_direct");
    tb->msg_connect(src, "tx", echo, "tx");
    tb->msg_connect(echo, "rx", est, "rx");
    tb->msg_connect(est, "cir", w, "cir");
    tb->msg_connect(echo, "rx", dbg, "store");
    tb->start();
    for (size_t i = 0; i < n_frames; ++i)
        src->_post(pmt::mp("emit"), pmt::make_dict());
    BOOST_REQUIRE(wait_until([&] {
        return w->frames_written() + w->frames_failed() >= n_frames &&
               est->drained();
    }));
    BOOST_REQUIRE_EQUAL(dbg->num_messages(), n_frames);
    {
        size_t n = 0;
        const gr_complex* p =
            pmt::c32vector_elements(pmt::cdr(dbg->get_message(0)), n);
        res.rx.assign(p, p + n);
    }
    tb->stop();
    tb->wait();
    BOOST_REQUIRE(w->stop());
    return res;
}

struct NativeRunResult {
    std::vector<gr_complex> resampled; // resampler output (= estimator input)
    pmt::pmt_t resampler_meta;         // mapped output meta (for assertions)
};

/**
 * Native 65/48 chain with the real production order and no test-side
 * overrides: packet source (737.28) → loopback (native channel,
 * pre_guard 1475 = 2 µs) → PDU 65/48 (validate_input_rate=true) →
 * estimator → writer.
 */
NativeRunResult
run_native(const std::string& native_tx_path,
           const std::vector<double>& delays,
           const std::vector<gr_complex>& gains,
           const std::string& tmpl_path,
           const std::string& out_dir,
           size_t n_frames,
           bool write_norm,
           size_t est_queue_cap = 16)
{
    NativeRunResult res;
    const auto taps = load_taps_f32(
        testdata_path("resampler_65_48/taps_quality_minorder.txt"));
    auto src = UwbRadarPacketSource::make(native_tx_path, kFsNative);
    auto echo = UwbLoopbackEcho::make(
        kPreGuardNative, kTail, delays, gains);
    auto resamp = UwbPduRationalResamplerCcf65_48::make_from_taps(
        taps, UwbPduRationalResamplerCcf65_48::kOutputRateHz,
        /*validate_input_rate=*/true);
    auto est = UwbRadarCirEstimator::make(tmpl_path,
                                          64,
                                          "4z2",
                                          9,
                                          kCirPre,
                                          kCirPost,
                                          10,
                                          0,
                                          64,
                                          8,
                                          0.3f,
                                          0.3f,
                                          true,
                                          est_queue_cap);
    auto w = UwbCirWriter::make(out_dir, "cir", write_norm, 64);
    auto dbg_res = gr::blocks::message_debug::make();
    auto tb = gr::make_top_block("qa_radar_e2e_native");
    tb->msg_connect(src, "tx", echo, "tx");
    tb->msg_connect(echo, "rx", resamp, "packet");
    tb->msg_connect(resamp, "packet", est, "rx");
    tb->msg_connect(est, "cir", w, "cir");
    tb->msg_connect(resamp, "packet", dbg_res, "store");
    tb->start();
    for (size_t i = 0; i < n_frames; ++i)
        src->_post(pmt::mp("emit"), pmt::make_dict());
    BOOST_REQUIRE(wait_until([&] {
        return w->frames_written() + w->frames_failed() >= n_frames &&
               est->drained();
    }));
    BOOST_REQUIRE_EQUAL(dbg_res->num_messages(), n_frames);
    {
        size_t n = 0;
        const gr_complex* p =
            pmt::c32vector_elements(pmt::cdr(dbg_res->get_message(0)), n);
        res.resampled.assign(p, p + n);
        res.resampler_meta = pmt::car(dbg_res->get_message(0));
    }
    tb->stop();
    tb->wait();
    BOOST_REQUIRE(w->stop());
    return res;
}

/**
 * Frame-count matrix runner (plan §12: 10/100/1000 frames) on the real
 * native chain: packet source → loopback (native channel) → PDU 65/48 →
 * estimator → writer.  Asserts per-frame JSONL count/order/offsets/
 * coordinate consistency, zero drops in every block, queue watermark and
 * RSS stability.  Runs with in-process emit pacing (chunks smaller than
 * the estimator queue), not realtime.
 */
void
run_frame_matrix(size_t n_frames,
                 const std::string& tag,
                 size_t est_queue_cap = 16)
{
    std::vector<gr_complex> tx_work;
    BOOST_REQUIRE(load_cf32(testdata_path("uwb_radar/tx_998p4.cf32"), tx_work));
    const std::string tmpl = write_template_file(tx_work, tag);
    const auto taps = load_taps_f32(
        testdata_path("resampler_65_48/taps_quality_minorder.txt"));
    const std::string dir = make_out_dir(tag);

    auto src = UwbRadarPacketSource::make(
        testdata_path("uwb_radar/tx_737p28.cf32"), kFsNative);
    auto echo = UwbLoopbackEcho::make(kPreGuardNative,
                                      kTail,
                                      { kDInt },
                                      { polar_gain(0.4, 0.7) });
    auto res = UwbPduRationalResamplerCcf65_48::make_from_taps(
        taps, UwbPduRationalResamplerCcf65_48::kOutputRateHz, true);
    auto est = UwbRadarCirEstimator::make(tmpl,
                                          64,
                                          "4z2",
                                          9,
                                          kCirPre,
                                          kCirPost,
                                          10,
                                          0,
                                          64,
                                          8,
                                          0.3f,
                                          0.3f,
                                          true,
                                          est_queue_cap);
    auto w = UwbCirWriter::make(dir, "cir", /*write_norm=*/false, 128);
    auto tb = gr::make_top_block("qa_radar_e2e_matrix_" + tag);
    tb->msg_connect(src, "tx", echo, "tx");
    tb->msg_connect(echo, "rx", res, "packet");
    tb->msg_connect(res, "packet", est, "rx");
    tb->msg_connect(est, "cir", w, "cir");
    tb->start();

    // Warmup frame (pages in every scratch), then RSS snapshot.
    src->_post(pmt::mp("emit"), pmt::make_dict());
    BOOST_REQUIRE(wait_until([&] {
        return w->frames_written() >= 1 && est->drained();
    }));
    size_t rss_before = 0, rss_after = 0;
    BOOST_REQUIRE(read_rss_bytes(&rss_before));

    const auto wall0 = std::chrono::steady_clock::now();
    const size_t chunk = std::min<size_t>(8, est_queue_cap - 1);
    for (size_t i = 1; i <= n_frames; ++i) {
        src->_post(pmt::mp("emit"), pmt::make_dict());
        if (i % chunk == 0) {
            BOOST_REQUIRE(wait_until([&] {
                return est->drained() &&
                       w->frames_written() + w->frames_failed() >= i + 1;
            }));
        }
    }
    BOOST_REQUIRE(wait_until([&] {
        return w->frames_written() + w->frames_failed() >= n_frames + 1 &&
               est->drained();
    }));
    const auto wall1 = std::chrono::steady_clock::now();
    tb->stop();
    tb->wait();
    BOOST_REQUIRE(w->stop());
    BOOST_REQUIRE(read_rss_bytes(&rss_after));

    // Zero drops in every block (n_frames + 1 = warmup + n_frames).
    BOOST_CHECK_EQUAL(src->pdus_dropped(), 0u);
    BOOST_CHECK_EQUAL(echo->pdus_dropped(), 0u);
    BOOST_CHECK_EQUAL(echo->pdus_emitted(), n_frames + 1);
    BOOST_CHECK_EQUAL(res->pdus_dropped(), 0u);
    BOOST_CHECK_EQUAL(res->pdus_emitted(), n_frames + 1);
    BOOST_CHECK_EQUAL(est->pdus_received(), n_frames + 1);
    BOOST_CHECK_EQUAL(est->pdus_enqueued(), n_frames + 1);
    BOOST_CHECK_EQUAL(est->pdus_dropped(), 0u);
    BOOST_CHECK_EQUAL(est->pdus_completed(), n_frames + 1);
    BOOST_CHECK_EQUAL(est->pdus_failed(), 0u);
    BOOST_CHECK_EQUAL(est->invalid_inputs(), 0u);
    BOOST_CHECK_EQUAL(w->frames_received(), n_frames + 1);
    BOOST_CHECK_EQUAL(w->frames_written(), n_frames + 1);
    BOOST_CHECK_EQUAL(w->frames_failed(), 0u);
    BOOST_CHECK_EQUAL(w->frames_dropped(), 0u);
    BOOST_CHECK_EQUAL(w->frames_invalid(), 0u);
    BOOST_CHECK_GE(est->queue_high_watermark(), 1u);
    BOOST_CHECK_LT(est->queue_high_watermark(), est->queue_capacity());
    BOOST_TEST_MESSAGE("e2e_matrix[" << tag << "]: frames=" << n_frames
                                     << " wall_ms="
                                     << std::chrono::duration_cast<
                                            std::chrono::milliseconds>(
                                            wall1 - wall0)
                                            .count()
                                     << " watermark="
                                     << est->queue_high_watermark()
                                     << " rss_drift="
                                     << (rss_after > rss_before
                                             ? rss_after - rss_before
                                             : 0));

    // JSONL: one line per frame, ordered, monotonic offsets, consistent
    // unified coordinates.
    const auto lines = read_lines(dir + "/cir.jsonl");
    BOOST_REQUIRE_EQUAL(lines.size(), n_frames + 1);
    for (size_t i = 0; i < lines.size(); ++i) {
        const JsonlLine j = parse_jsonl(lines[i]);
        BOOST_CHECK_EQUAL(j.status, "ok");
        BOOST_CHECK_EQUAL(j.pulse_id, i);
        BOOST_CHECK_EQUAL(j.tap_count, kTaps);
        BOOST_CHECK_EQUAL(j.file_offset, static_cast<uint64_t>(i) * kTaps);
        BOOST_CHECK_LE(std::llabs(j.sfd - j.predicted), 2);
        BOOST_CHECK_LE(std::llabs(j.preamble - j.cir_origin), 2);
        BOOST_CHECK_EQUAL(j.zero_delay, static_cast<int64_t>(kCirPre));
    }
    std::vector<uint8_t> raw;
    BOOST_REQUIRE(read_bytes(dir + "/cir.cf32", raw));
    BOOST_CHECK_EQUAL(raw.size(), (n_frames + 1) * kTaps * 8);

    // RSS must be bounded (allocator arena retention allowed).
    const size_t drift =
        rss_after > rss_before ? rss_after - rss_before : 0;
    BOOST_CHECK_LT(drift, 200u * 1024 * 1024);
}

} // namespace

// ---------------------------------------------------------------------------
// e2e_998p4_direct: 998.4 packet → loopback → estimator → writer.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(e2e_998p4_direct)
{
    std::vector<gr_complex> tx;
    BOOST_REQUIRE(load_cf32(testdata_path("uwb_radar/tx_998p4.cf32"), tx));
    BOOST_REQUIRE_EQUAL(tx.size(), 190912u);
    const std::string tmpl = write_template_file(tx, "direct");

    // ---- run 1: clean channel (delay 0, unity gain) ------------------------
    const std::string dir1 = make_out_dir("direct");
    const RunResult r1 = run_direct(testdata_path("uwb_radar/tx_998p4.cf32"),
                                    { 0.0 },
                                    { gr_complex(1.0f, 0.0f) },
                                    0.0f,
                                    1,
                                    tmpl,
                                    64,
                                    dir1,
                                    /*write_norm=*/true,
                                    1);

    const auto lines1 = read_lines(dir1 + "/cir.jsonl");
    BOOST_REQUIRE_EQUAL(lines1.size(), 1u);
    const JsonlLine j1 = parse_jsonl(lines1[0]);
    BOOST_CHECK_EQUAL(j1.status, "ok");
    BOOST_CHECK_EQUAL(j1.pulse_id, 0u);
    BOOST_CHECK_EQUAL(j1.tap_count, kTaps);
    BOOST_CHECK_EQUAL(j1.predicted, kSfdClean);
    BOOST_CHECK_LE(std::llabs(j1.sfd - kSfdClean), 2);
    BOOST_CHECK_EQUAL(j1.cir_origin, kOriginClean);
    BOOST_CHECK_LE(std::llabs(j1.preamble - kOriginClean), 1);
    BOOST_CHECK_EQUAL(j1.zero_delay, static_cast<int64_t>(kCirPre));
    BOOST_CHECK_LE(std::llabs(j1.peak_tap - kPeakClean), 1);
    BOOST_CHECK_LE(std::llabs((j1.peak_tap - j1.zero_delay) - 2), 1);
    BOOST_CHECK_EQUAL(j1.file_offset, 0u);
    BOOST_CHECK_CLOSE(j1.sample_rate, kFsWork, 1e-9);
    BOOST_CHECK(j1.sfd_ok);
    BOOST_CHECK(j1.timing_ok);
    BOOST_CHECK_EQUAL(j1.source, std::string("loopback"));

    // Binary files: exact size, MATLAB golden alignment, unit L2 norm.
    std::vector<uint8_t> raw_bytes, norm_bytes, run_json;
    BOOST_REQUIRE(read_bytes(dir1 + "/cir.cf32", raw_bytes));
    BOOST_REQUIRE(read_bytes(dir1 + "/cir_norm.cf32", norm_bytes));
    BOOST_REQUIRE(read_bytes(dir1 + "/run.json", run_json));
    BOOST_CHECK_EQUAL(raw_bytes.size(), kTaps * 8);
    BOOST_CHECK_EQUAL(norm_bytes.size(), kTaps * 8);
    BOOST_CHECK_GT(run_json.size(), 0u);
    std::vector<gr_complex> raw1(kTaps), norm1(kTaps), gold_raw, gold_norm;
    std::memcpy(raw1.data(), raw_bytes.data(), kTaps * 8);
    std::memcpy(norm1.data(), norm_bytes.data(), kTaps * 8);
    BOOST_REQUIRE(load_cf32(
        testdata_path("uwb_radar/cir_raw_clean_radar.cf32"), gold_raw));
    BOOST_REQUIRE(load_cf32(
        testdata_path("uwb_radar/cir_norm_clean_radar.cf32"), gold_norm));
    BOOST_CHECK_LT(rel_l2(raw1, gold_raw), 1e-5);
    BOOST_CHECK_LT(rel_l2(norm1, gold_norm), 1e-5);
    BOOST_CHECK_CLOSE(l2_norm(norm1), 1.0, 1e-3);

    // Taps bit-exact vs a direct radar_cir_one reference on the same input.
    std::vector<gr_complex> ref_raw, ref_norm;
    direct_core_reference(
        tx, r1.rx, 64, kSfdClean, ref_raw, ref_norm);
    BOOST_CHECK(std::memcmp(raw1.data(), ref_raw.data(), kTaps * 8) == 0);
    BOOST_CHECK(std::memcmp(norm1.data(), ref_norm.data(), kTaps * 8) == 0);

    // ---- run 2: integer delay 37 with calibrated axis (cal == D) ----------
    const std::string dir2 = make_out_dir("direct_d37");
    const RunResult r2 = run_direct(testdata_path("uwb_radar/tx_998p4.cf32"),
                                    { kDInt },
                                    { polar_gain(0.4, 0.7) },
                                    0.0f,
                                    2,
                                    tmpl,
                                    64,
                                    dir2,
                                    /*write_norm=*/true,
                                    1);

    const auto lines2 = read_lines(dir2 + "/cir.jsonl");
    BOOST_REQUIRE_EQUAL(lines2.size(), 1u);
    const JsonlLine j2 = parse_jsonl(lines2[0]);
    BOOST_CHECK_EQUAL(j2.status, "ok");
    // predicted = pre_guard + round(cal=D) + 64*1016 (loopback reports the
    // front path delay as the calibration delay).
    BOOST_CHECK_EQUAL(j2.predicted, kSfdD37);
    // Within plan tolerance of the ground truth, not margin-absorbed.
    BOOST_CHECK_LE(std::llabs(j2.sfd - kSfdD37), 2);
    BOOST_CHECK_EQUAL(j2.cir_origin, kOriginD37);
    BOOST_CHECK_LE(std::llabs(j2.preamble - kOriginD37), 1);
    // Unified zero-delay convention: zero_delay_tap = cir_pre; the
    // calibrated leakage reads +2 taps (pulse-shape offset).
    BOOST_CHECK_EQUAL(j2.zero_delay, static_cast<int64_t>(kCirPre));
    BOOST_CHECK_EQUAL(j2.peak_tap, expected_peak_tap(j2.preamble, j2.cir_origin));
    BOOST_CHECK_LE(std::llabs(j2.peak_tap - kPeakClean), 2);
    BOOST_CHECK_LE(std::llabs((j2.peak_tap - j2.zero_delay) - 2), 1);
    BOOST_CHECK_EQUAL(j2.tap_count, kTaps);
    BOOST_CHECK_CLOSE(j2.cal_work, kDInt, 1e-6);

    std::vector<gr_complex> raw2(kTaps), norm2(kTaps);
    std::vector<uint8_t> raw2_bytes, norm2_bytes;
    BOOST_REQUIRE(read_bytes(dir2 + "/cir.cf32", raw2_bytes));
    BOOST_REQUIRE(read_bytes(dir2 + "/cir_norm.cf32", norm2_bytes));
    std::memcpy(raw2.data(), raw2_bytes.data(), kTaps * 8);
    std::memcpy(norm2.data(), norm2_bytes.data(), kTaps * 8);
    std::vector<gr_complex> ref2_raw, ref2_norm;
    direct_core_reference(tx, r2.rx, 64, kSfdD37, ref2_raw, ref2_norm);
    BOOST_CHECK(std::memcmp(raw2.data(), ref2_raw.data(), kTaps * 8) == 0);
    BOOST_CHECK(std::memcmp(norm2.data(), ref2_norm.data(), kTaps * 8) == 0);
}

// ---------------------------------------------------------------------------
// e2e_native_65_48: native 737.28 packet → loopback (native channel) →
// PDU 65/48 (validate_input_rate=true) → estimator (998.4) → writer.
// Asserts the resampler metadata mapping (sample_rate, calibration delay
// native→work, mapped window/pre_guard/capture, profile identity) and the
// unified coordinates (no systematic FIR-head bias).
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(e2e_native_65_48)
{
    std::vector<gr_complex> tx_work;
    BOOST_REQUIRE(load_cf32(testdata_path("uwb_radar/tx_998p4.cf32"), tx_work));
    const std::string tmpl = write_template_file(tx_work, "native");

    const std::string dir = make_out_dir("native");
    const NativeRunResult r = run_native(
        testdata_path("uwb_radar/tx_737p28.cf32"),
        { kDInt },
        { polar_gain(0.4, 0.7) },
        tmpl,
        dir,
        1,
        /*write_norm=*/true);

    // Counters: nothing dropped or rejected along the chain is asserted by
    // the writer/JSONL checks below; the resampled PDU must exist.
    BOOST_CHECK_GT(r.resampled.size(), 0u);

    const auto taps =
        load_taps_f32(testdata_path("resampler_65_48/taps_quality_minorder.txt"));
    auto map_probe = UwbPduRationalResamplerCcf65_48::make_from_taps(
        taps, UwbPduRationalResamplerCcf65_48::kOutputRateHz, true);
    const auto map_of = [&](int64_t p) {
        return map_probe->map_input_offset_to_output(p);
    };
    const int64_t map0 = map_of(0); // FIR group-delay head (28 for T=2707)

    // Resampler output meta: the estimator's required keys, mapped.
    pmt::pmt_t mo = r.resampler_meta;
    BOOST_REQUIRE(pmt::is_dict(mo));
    BOOST_CHECK_CLOSE(
        pmt::to_double(pmt::dict_ref(mo, pmt::mp("sample_rate"),
                                     pmt::from_double(0))),
        kFsWork, 1e-9);
    BOOST_CHECK_EQUAL(
        pmt::to_long(pmt::dict_ref(mo, pmt::mp("window_start_sample"),
                                   pmt::from_long(-1))),
        map0);
    BOOST_CHECK_EQUAL(
        pmt::to_long(pmt::dict_ref(mo, pmt::mp("pre_guard_samples"),
                                   pmt::from_long(-1))),
        map_of(kPreGuardNative) - map0);
    BOOST_CHECK_EQUAL(
        pmt::to_long(pmt::dict_ref(mo, pmt::mp("capture_samples"),
                                   pmt::from_long(-1))),
        map_of(kPreGuardNative + static_cast<int64_t>(kNativeTxLen)) -
            map_of(kPreGuardNative));
    BOOST_CHECK_CLOSE(
        pmt::to_double(pmt::dict_ref(
            mo, pmt::mp("calibration_delay_work_samples"),
            pmt::from_double(0))),
        kDInt * 65.0 / 48.0, 1e-9);
    BOOST_CHECK_EQUAL(
        pmt::to_long(pmt::dict_ref(mo, pmt::mp("sync_repetitions"),
                                   pmt::from_long(-1))),
        64);
    BOOST_CHECK_EQUAL(
        pmt::symbol_to_string(pmt::dict_ref(mo, pmt::mp("sfd_mode"),
                                            pmt::PMT_NIL)),
        "4z2");
    BOOST_CHECK_EQUAL(
        pmt::to_long(pmt::dict_ref(mo, pmt::mp("code_index"),
                                   pmt::from_long(-1))),
        9);
    BOOST_CHECK_EQUAL(
        pmt::symbol_to_string(pmt::dict_ref(mo, pmt::mp("source"),
                                            pmt::PMT_NIL)),
        "loopback");
    BOOST_CHECK_EQUAL(
        pmt::symbol_to_string(pmt::dict_ref(mo, pmt::mp("sample_format"),
                                            pmt::PMT_NIL)),
        "fc32");

    // Estimator prediction with the unified convention.
    const double cal_work = kDInt * 65.0 / 48.0; // 50.104166...
    const int64_t cal_round = static_cast<int64_t>(std::llround(cal_work));
    const int64_t predicted = map0 + (map_of(kPreGuardNative) - map0) +
                              cal_round + static_cast<int64_t>(64 * kSps);
    std::vector<gr_complex> ref_raw, ref_norm;
    direct_core_reference(tx_work, r.resampled, 64, predicted, ref_raw,
                          ref_norm);

    std::vector<uint8_t> raw_bytes, norm_bytes;
    BOOST_REQUIRE(read_bytes(dir + "/cir.cf32", raw_bytes));
    BOOST_REQUIRE(read_bytes(dir + "/cir_norm.cf32", norm_bytes));
    BOOST_CHECK_EQUAL(raw_bytes.size(), kTaps * 8);
    BOOST_CHECK_EQUAL(norm_bytes.size(), kTaps * 8);
    std::vector<gr_complex> raw1(kTaps), norm1(kTaps);
    std::memcpy(raw1.data(), raw_bytes.data(), kTaps * 8);
    std::memcpy(norm1.data(), norm_bytes.data(), kTaps * 8);
    BOOST_CHECK(std::memcmp(raw1.data(), ref_raw.data(), kTaps * 8) == 0);
    BOOST_CHECK(std::memcmp(norm1.data(), ref_norm.data(), kTaps * 8) == 0);
    BOOST_CHECK_CLOSE(l2_norm(norm1), 1.0, 1e-3);

    const auto lines = read_lines(dir + "/cir.jsonl");
    BOOST_REQUIRE_EQUAL(lines.size(), 1u);
    const JsonlLine j = parse_jsonl(lines[0]);
    BOOST_CHECK_EQUAL(j.status, "ok");
    BOOST_CHECK_EQUAL(j.tap_count, kTaps);
    BOOST_CHECK_EQUAL(j.predicted, predicted);
    BOOST_CHECK_EQUAL(j.cir_origin, predicted - static_cast<int64_t>(64 * kSps));
    BOOST_CHECK_EQUAL(j.zero_delay, static_cast<int64_t>(kCirPre));
    BOOST_CHECK_CLOSE(j.cal_work, cal_work, 1e-6);
    BOOST_CHECK_CLOSE(j.sample_rate, kFsWork, 1e-9);
    BOOST_CHECK_EQUAL(j.source, std::string("loopback"));
    BOOST_CHECK(j.sfd_ok);
    BOOST_CHECK(j.timing_ok);

    // Unified coordinates: the prediction must land on the truth within
    // the plan tolerance (±2 work samples), not be margin-absorbed.
    const int64_t sfd_truth_native =
        map_of(kPreGuardNative + static_cast<int64_t>(kDInt) + kSyncNative64);
    // The mapped ground truth sits on the prediction (±2 work samples; the
    // residual is only the native SYNC ceil-rounding, ±1 native sample).
    BOOST_CHECK_LE(std::llabs(sfd_truth_native - predicted), 2);
    BOOST_CHECK_LE(std::llabs(j.sfd - predicted), 2);
    // RX-refined origin sits on the true (resampled) packet start and on
    // the CIR axis: no FIR-head bias between origin and axis.
    BOOST_CHECK_LE(std::llabs(j.preamble - j.cir_origin), 2);
    // Peak on the CIR axis (same formula as the direct path; the 65/48 FIR
    // ripple can round the pulse-shape offset ±1, so ±2 total).
    BOOST_CHECK_LE(std::llabs(j.peak_tap -
                              expected_peak_tap(j.preamble, j.cir_origin)),
                   2);
    BOOST_CHECK_LE(std::llabs(j.peak_tap - kPeakClean), 2);
    BOOST_CHECK_LE(std::llabs((j.peak_tap - j.zero_delay) - 2), 2);
    BOOST_CHECK_EQUAL(j.file_offset, 0u);
}

BOOST_AUTO_TEST_CASE(e2e_native_65_32)
{
    std::vector<gr_complex> tx_work;
    BOOST_REQUIRE(load_cf32(testdata_path("uwb_radar/tx_998p4.cf32"), tx_work));
    const std::string tmpl = write_template_file(tx_work, "native_65_32");
    const std::string dir = make_out_dir("native_65_32");

    const auto taps = load_taps_f32(
        testdata_path("resampler_65_32/taps_quality_minorder.txt"));
    auto src = UwbRadarPacketSource::make(
        testdata_path("uwb_radar/tx_491p52.cf32"), kFsCg400);
    auto echo = UwbLoopbackEcho::make(
        kPreGuardCg400, kTail, { kDIntCg400 }, { polar_gain(0.4, 0.7) });
    auto resamp = UwbPduRationalResamplerCcf65_32::make_from_taps(
        taps, UwbPduRationalResamplerCcf65_32::kOutputRateHz,
        /*validate_input_rate=*/true);
    auto est = UwbRadarCirEstimator::make(tmpl, 64, "4z2", 9, kCirPre,
                                          kCirPost, 10, 0, 64, 8, 0.3f, 0.3f,
                                          true, 16);
    auto w = UwbCirWriter::make(dir, "cir", true, 64);
    auto dbg_res = gr::blocks::message_debug::make();
    auto tb = gr::make_top_block("qa_radar_e2e_native_65_32");
    tb->msg_connect(src, "tx", echo, "tx");
    tb->msg_connect(echo, "rx", resamp, "packet");
    tb->msg_connect(resamp, "packet", est, "rx");
    tb->msg_connect(est, "cir", w, "cir");
    tb->msg_connect(resamp, "packet", dbg_res, "store");
    tb->start();
    src->_post(pmt::mp("emit"), pmt::make_dict());
    BOOST_REQUIRE(wait_until([&] {
        return w->frames_written() + w->frames_failed() >= 1 && est->drained();
    }));
    BOOST_REQUIRE_EQUAL(dbg_res->num_messages(), 1);
    tb->stop();
    tb->wait();
    BOOST_REQUIRE(w->stop());

    pmt::pmt_t mo = pmt::car(dbg_res->get_message(0));
    BOOST_REQUIRE(pmt::is_dict(mo));
    BOOST_CHECK_CLOSE(
        pmt::to_double(pmt::dict_ref(mo, pmt::mp("sample_rate"),
                                     pmt::from_double(0))),
        kFsWork, 1e-9);
    BOOST_CHECK_CLOSE(
        pmt::to_double(pmt::dict_ref(
            mo, pmt::mp("calibration_delay_work_samples"),
            pmt::from_double(0))),
        kDIntCg400 * 65.0 / 32.0, 1e-9);
    BOOST_CHECK_EQUAL(
        pmt::to_long(pmt::dict_ref(mo, pmt::mp("resample_decim"),
                                   pmt::from_long(-1))),
        32);
    BOOST_CHECK_EQUAL(
        pmt::to_long(pmt::dict_ref(mo, pmt::mp("capture_samples"),
                                   pmt::from_long(-1))),
        resamp->map_input_offset_to_output(
            static_cast<int64_t>(kPreGuardCg400 + kNativeTxLenCg400)) -
            resamp->map_input_offset_to_output(
                static_cast<int64_t>(kPreGuardCg400)));

    const int64_t map0 = resamp->map_input_offset_to_output(0);
    const double cal_work = kDIntCg400 * 65.0 / 32.0;
    const int64_t cal_round = static_cast<int64_t>(std::llround(cal_work));
    const int64_t predicted =
        map0 +
        (resamp->map_input_offset_to_output(
             static_cast<int64_t>(kPreGuardCg400)) -
         map0) +
        cal_round + static_cast<int64_t>(64 * kSps);
    const int64_t sfd_truth = resamp->map_input_offset_to_output(
        static_cast<int64_t>(kPreGuardCg400) +
        static_cast<int64_t>(kDIntCg400) + kSyncNative64Cg400);

    const auto lines = read_lines(dir + "/cir.jsonl");
    BOOST_REQUIRE_EQUAL(lines.size(), 1u);
    const JsonlLine j = parse_jsonl(lines[0]);
    BOOST_CHECK_EQUAL(j.status, "ok");
    BOOST_CHECK(j.sfd_ok);
    BOOST_CHECK(j.timing_ok);
    BOOST_CHECK_LE(std::llabs(sfd_truth - predicted), 2);
    BOOST_CHECK_LE(std::llabs(j.sfd - predicted), 2);
    BOOST_CHECK_LE(std::llabs(j.preamble - j.cir_origin), 2);
    BOOST_CHECK_LE(std::llabs(j.peak_tap -
                              expected_peak_tap(j.preamble, j.cir_origin)),
                   2);
}

// ---------------------------------------------------------------------------
// e2e_conditions_matrix: fractional delay + multipath + AWGN + complex
// phase (all supported loopback channel parameters, in one run), plus
// raw-CIR amplitude linearity across two loopback gains (0.5 / 2.0) with
// an invariant normalized CIR.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(e2e_conditions_matrix)
{
    std::vector<gr_complex> tx;
    BOOST_REQUIRE(load_cf32(testdata_path("uwb_radar/tx_998p4.cf32"), tx));
    const std::string tmpl = write_template_file(tx, "cond");

    // ---- conditions run: frac delay + multipath + AWGN + complex phase ----
    const std::string dirc = make_out_dir("cond");
    const RunResult rc = run_direct(testdata_path("uwb_radar/tx_998p4.cf32"),
                                    { 12.4, 40.0 }, // fractional + integer
                                    { polar_gain(0.55, -1.1),
                                      polar_gain(0.3, 0.3) },
                                    0.02f, // AWGN
                                    7,     // fixed seed
                                    tmpl,
                                    64,
                                    dirc,
                                    true,
                                    1);
    (void)rc;
    const auto lc = read_lines(dirc + "/cir.jsonl");
    BOOST_REQUIRE_EQUAL(lc.size(), 1u);
    const JsonlLine jc = parse_jsonl(lc[0]);
    BOOST_CHECK_EQUAL(jc.status, "ok");
    BOOST_CHECK_EQUAL(jc.tap_count, kTaps);
    // cal = front path delay 12.4 → round 12; prediction lands on truth.
    BOOST_CHECK_EQUAL(jc.predicted,
                      kOriginClean + 12 + static_cast<int64_t>(64 * kSps));
    BOOST_CHECK_LE(std::llabs(jc.preamble - (kOriginClean + 12)), 2);
    BOOST_CHECK_EQUAL(jc.cir_origin, kOriginClean + 12);
    BOOST_CHECK_LE(std::llabs(jc.peak_tap - expected_peak_tap(
                                             jc.preamble, jc.cir_origin)),
                   2);
    BOOST_CHECK_EQUAL(jc.zero_delay, static_cast<int64_t>(kCirPre));
    {
        std::vector<uint8_t> nrmc;
        BOOST_REQUIRE(read_bytes(dirc + "/cir_norm.cf32", nrmc));
        BOOST_REQUIRE_EQUAL(nrmc.size(), kTaps * 8);
        std::vector<gr_complex> nc(kTaps);
        std::memcpy(nc.data(), nrmc.data(), kTaps * 8);
        BOOST_CHECK_CLOSE(l2_norm(nc), 1.0, 1e-3);
    }

    // ---- amplitude linearity: gains 0.5 and 2.0 ----------------------------
    const std::string dir05 = make_out_dir("gain05");
    const std::string dir20 = make_out_dir("gain20");
    (void)run_direct(testdata_path("uwb_radar/tx_998p4.cf32"),
                     { kDInt },
                     { polar_gain(0.5, 0.0) },
                     0.0f,
                     3,
                     tmpl,
                     64,
                     dir05,
                     true,
                     1);
    (void)run_direct(testdata_path("uwb_radar/tx_998p4.cf32"),
                     { kDInt },
                     { polar_gain(2.0, 0.0) },
                     0.0f,
                     4,
                     tmpl,
                     64,
                     dir20,
                     true,
                     1);

    std::vector<uint8_t> raw05, raw20, nrm05, nrm20;
    BOOST_REQUIRE(read_bytes(dir05 + "/cir.cf32", raw05));
    BOOST_REQUIRE(read_bytes(dir20 + "/cir.cf32", raw20));
    BOOST_REQUIRE(read_bytes(dir05 + "/cir_norm.cf32", nrm05));
    BOOST_REQUIRE(read_bytes(dir20 + "/cir_norm.cf32", nrm20));
    BOOST_REQUIRE_EQUAL(raw05.size(), kTaps * 8);
    BOOST_REQUIRE_EQUAL(raw20.size(), kTaps * 8);
    std::vector<gr_complex> a05(kTaps), a20(kTaps), n05(kTaps), n20(kTaps);
    std::memcpy(a05.data(), raw05.data(), kTaps * 8);
    std::memcpy(a20.data(), raw20.data(), kTaps * 8);
    std::memcpy(n05.data(), nrm05.data(), kTaps * 8);
    std::memcpy(n20.data(), nrm20.data(), kTaps * 8);

    // raw CIR scales linearly with the gain ratio (2.0/0.5 = 4).
    double worst = 0.0, scale = 0.0;
    for (size_t i = 0; i < kTaps; ++i) {
        const gr_complex expect = static_cast<float>(4.0) * a05[i];
        worst = std::max(worst, static_cast<double>(std::abs(a20[i] - expect)));
        scale = std::max(scale, static_cast<double>(std::abs(expect)));
    }
    BOOST_CHECK_LT(worst, 1e-4 * std::max(1.0, scale));
    // normalized CIR invariant.
    BOOST_CHECK_LT(rel_l2(n05, n20), 1e-6);
    // both runs keep the same coordinates.
    const JsonlLine j05 = parse_jsonl(read_lines(dir05 + "/cir.jsonl")[0]);
    const JsonlLine j20 = parse_jsonl(read_lines(dir20 + "/cir.jsonl")[0]);
    BOOST_CHECK_EQUAL(j05.status, "ok");
    BOOST_CHECK_EQUAL(j20.status, "ok");
    BOOST_CHECK_EQUAL(j05.predicted, j20.predicted);
    BOOST_CHECK_EQUAL(j05.peak_tap, j20.peak_tap);
    BOOST_CHECK_EQUAL(j05.zero_delay, static_cast<int64_t>(kCirPre));
    BOOST_CHECK_EQUAL(j20.zero_delay, static_cast<int64_t>(kCirPre));
}

// ---------------------------------------------------------------------------
// e2e_sync_repetitions: 32/64/128 SYNC through the direct e2e chain.
// All three profiles are supported by the packet source (per-profile
// goldens under testdata/uwb_radar/packets/<reps>/) and the estimator.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(e2e_sync_repetitions)
{
    struct Case {
        size_t reps;
        const char* tx_path; // per-profile complete packet golden
        int64_t sfd_truth;   // packets/<reps>/metadata.json rx_clean_998p4
        uint64_t valid_reps; // reps - skip(10)
    };
    const Case cases[] = {
        { 32, "uwb_radar/packets/sync32/tx_998p4.cf32", 34509, 22 },
        // 64-SYNC canonical golden lives at the top level.
        { 64, "uwb_radar/tx_998p4.cf32", 67021, 54 },
        // The estimator's cir_repetitions=0 auto-derives reps-skip=118 for
        // 128 SYNC (the MATLAB packet golden recorded 54 with an explicit
        // repetition cap; the block auto policy is reps-skip).
        { 128, "uwb_radar/packets/sync128/tx_998p4.cf32", 132045, 118 },
    };

    for (const auto& c : cases) {
        const std::string tx_path = testdata_path(c.tx_path);
        std::vector<gr_complex> tx;
        BOOST_REQUIRE_MESSAGE(load_cf32(tx_path, tx),
                              "missing " + tx_path);
        const std::string tmpl =
            write_template_file(tx, "sync" + std::to_string(c.reps));
        const std::string dir =
            make_out_dir("sync" + std::to_string(c.reps));

        auto src = UwbRadarPacketSource::make(
            tx_path, kFsWork, "fc32", c.reps, "4z2", 9);
        auto echo = UwbLoopbackEcho::make(kPreGuardWork,
                                          kTail,
                                          { 0.0 },
                                          { gr_complex(1.0f, 0.0f) });
        auto est = UwbRadarCirEstimator::make(tmpl,
                                              c.reps,
                                              "4z2",
                                              9,
                                              kCirPre,
                                              kCirPost,
                                              10,
                                              0, // auto = reps - skip
                                              64,
                                              8,
                                              0.3f,
                                              0.3f,
                                              true,
                                              16);
        auto w = UwbCirWriter::make(dir, "cir", true, 64);
        auto dbg_cir = gr::blocks::message_debug::make();
        auto tb = gr::make_top_block("qa_radar_e2e_sync");
        tb->msg_connect(src, "tx", echo, "tx");
        tb->msg_connect(echo, "rx", est, "rx");
        tb->msg_connect(est, "cir", w, "cir");
        tb->msg_connect(est, "cir", dbg_cir, "store");
        tb->start();
        src->_post(pmt::mp("emit"), pmt::make_dict());
        BOOST_REQUIRE(wait_until([&] {
            return w->frames_written() + w->frames_failed() >= 1 &&
                   est->drained();
        }));
        tb->stop();
        tb->wait();
        BOOST_REQUIRE(w->stop());

        BOOST_CHECK_EQUAL(est->pdus_completed(), 1u);
        BOOST_CHECK_EQUAL(est->pdus_failed(), 0u);
        BOOST_REQUIRE_EQUAL(dbg_cir->num_messages(), 1u);
        {
            pmt::pmt_t meta = pmt::car(dbg_cir->get_message(0));
            BOOST_CHECK_EQUAL(
                pmt::to_uint64(pmt::dict_ref(
                    meta, pmt::mp("valid_repetitions"),
                    pmt::from_uint64(0))),
                c.valid_reps);
        }

        const auto lines = read_lines(dir + "/cir.jsonl");
        BOOST_REQUIRE_EQUAL(lines.size(), 1u);
        const JsonlLine j = parse_jsonl(lines[0]);
        BOOST_CHECK_EQUAL(j.status, "ok");
        BOOST_CHECK_EQUAL(j.tap_count, kTaps);
        BOOST_CHECK_EQUAL(j.predicted, kOriginClean +
                                            c.reps *
                                                static_cast<int64_t>(kSps));
        BOOST_CHECK_LE(std::llabs(j.sfd - c.sfd_truth), 2);
        BOOST_CHECK_EQUAL(j.cir_origin, kOriginClean);
        BOOST_CHECK_LE(std::llabs(j.preamble - kOriginClean), 1);
        BOOST_CHECK_LE(std::llabs(j.peak_tap - kPeakClean), 1);
        BOOST_CHECK_EQUAL(j.zero_delay, static_cast<int64_t>(kCirPre));
        std::vector<uint8_t> raw;
        BOOST_REQUIRE(read_bytes(dir + "/cir.cf32", raw));
        BOOST_CHECK_EQUAL(raw.size(), kTaps * 8);
    }
}

// ---------------------------------------------------------------------------
// e2e_missing_sfd: normal SFD → ok; silent channel → sfd_failed with no
// taps but exactly one JSONL line per frame.
// (A "SFD region zeroed" packet is NOT used as the negative: at the real
// golden amplitude the SFD metric can produce a false peak inside the
// search window (Step 7-8 report §6); silence fails deterministically.)
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(e2e_missing_sfd)
{
    std::vector<gr_complex> tx;
    BOOST_REQUIRE(load_cf32(testdata_path("uwb_radar/tx_998p4.cf32"), tx));
    const std::string tmpl = write_template_file(tx, "missfd");

    // Silent TX packet in a sidecar-free temp dir (work rate needs no
    // descriptor, but the canonical sidecar would reject the length).
    const std::string zero_dir =
        (std::filesystem::temp_directory_path() /
         "uwb_qa_radar_e2e_missfd_zeros")
            .string();
    std::filesystem::remove_all(zero_dir);
    std::filesystem::create_directories(zero_dir);
    const std::string zero_path = zero_dir + "/tx_zero_998p4.cf32";
    {
        std::vector<gr_complex> zeros(tx.size(), gr_complex(0.f, 0.f));
        std::ofstream f(zero_path, std::ios::binary | std::ios::trunc);
        BOOST_REQUIRE_MESSAGE(f.good(), "cannot write " + zero_path);
        f.write(reinterpret_cast<const char*>(zeros.data()),
                static_cast<std::streamsize>(zeros.size() *
                                             sizeof(gr_complex)));
        BOOST_REQUIRE(f.good());
    }

    const std::string dir = make_out_dir("missfd");
    auto src_ok = UwbRadarPacketSource::make(
        testdata_path("uwb_radar/tx_998p4.cf32"), kFsWork);
    auto src_zero = UwbRadarPacketSource::make(zero_path, kFsWork);
    auto echo = UwbLoopbackEcho::make(kPreGuardWork,
                                      kTail,
                                      { 0.0 },
                                      { gr_complex(1.0f, 0.0f) });
    auto est = UwbRadarCirEstimator::make(tmpl,
                                          64,
                                          "4z2",
                                          9,
                                          kCirPre,
                                          kCirPost,
                                          10,
                                          0,
                                          64,
                                          8,
                                          0.3f,
                                          0.3f,
                                          true,
                                          16);
    auto w = UwbCirWriter::make(dir, "cir", true, 64);
    auto tb = gr::make_top_block("qa_radar_e2e_missfd");
    tb->msg_connect(src_ok, "tx", echo, "tx");
    tb->msg_connect(src_zero, "tx", echo, "tx");
    tb->msg_connect(echo, "rx", est, "rx");
    tb->msg_connect(est, "cir", w, "cir");
    tb->start();
    // Post sequentially and wait for each frame: two different source
    // blocks deliver through independent scheduler queues, so the echo
    // receive order is not guaranteed unless we stage the posts.
    src_ok->_post(pmt::mp("emit"), pmt::make_dict());
    BOOST_REQUIRE(wait_until([&] { return w->frames_written() >= 1; }));
    {
        pmt::pmt_t d = pmt::make_dict();
        d = pmt::dict_add(d, pmt::mp("pulse_id"), pmt::from_uint64(1));
        src_zero->_post(pmt::mp("emit"), d);
    }
    BOOST_REQUIRE(wait_until([&] {
        return w->frames_written() + w->frames_failed() >= 2 && est->drained();
    }));
    tb->stop();
    tb->wait();
    BOOST_REQUIRE(w->stop());

    BOOST_CHECK_EQUAL(est->pdus_completed(), 1u);
    BOOST_CHECK_EQUAL(est->pdus_failed(), 1u);
    BOOST_CHECK_EQUAL(est->pdus_dropped(), 0u);

    const auto lines = read_lines(dir + "/cir.jsonl");
    BOOST_REQUIRE_EQUAL(lines.size(), 2u); // no missing frames

    const JsonlLine j0 = parse_jsonl(lines[0]);
    BOOST_CHECK_EQUAL(j0.status, "ok");
    BOOST_CHECK_EQUAL(j0.pulse_id, 0u);
    BOOST_CHECK_EQUAL(j0.tap_count, kTaps);
    BOOST_CHECK_EQUAL(j0.file_offset, 0u);
    BOOST_CHECK_LE(std::llabs(j0.sfd - kSfdClean), 2);

    const JsonlLine j1 = parse_jsonl(lines[1]);
    BOOST_CHECK_EQUAL(j1.status, "sfd_failed");
    BOOST_CHECK_EQUAL(j1.pulse_id, 1u);
    BOOST_CHECK_EQUAL(j1.tap_count, 0u);
    // offset must not advance on a failed frame
    BOOST_CHECK_EQUAL(j1.file_offset, kTaps);
    BOOST_CHECK_EQUAL(j1.sfd, -1);
    BOOST_CHECK(!j1.sfd_ok);

    std::vector<uint8_t> raw, norm;
    BOOST_REQUIRE(read_bytes(dir + "/cir.cf32", raw));
    BOOST_REQUIRE(read_bytes(dir + "/cir_norm.cf32", norm));
    BOOST_CHECK_EQUAL(raw.size(), kTaps * 8);  // only the ok frame
    BOOST_CHECK_EQUAL(norm.size(), kTaps * 8); // only the ok frame
}

// ---------------------------------------------------------------------------
// e2e_100_frames: 100 frames through the real native e2e chain
// (source → loopback → 65/48 → estimator → writer) with in-process emit
// pacing.  Checks line count, order, counters, offsets, queue watermark
// and RSS stability.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(e2e_100_frames)
{
    std::vector<gr_complex> tx_work;
    BOOST_REQUIRE(load_cf32(testdata_path("uwb_radar/tx_998p4.cf32"), tx_work));
    const std::string tmpl = write_template_file(tx_work, "f100");

    const auto taps =
        load_taps_f32(testdata_path("resampler_65_48/taps_quality_minorder.txt"));
    const std::string dir = make_out_dir("f100");

    auto src = UwbRadarPacketSource::make(
        testdata_path("uwb_radar/tx_737p28.cf32"), kFsNative);
    auto echo = UwbLoopbackEcho::make(kPreGuardNative,
                                      kTail,
                                      { kDInt },
                                      { polar_gain(0.4, 0.7) });
    auto res = UwbPduRationalResamplerCcf65_48::make_from_taps(
        taps, UwbPduRationalResamplerCcf65_48::kOutputRateHz, true);
    auto est = UwbRadarCirEstimator::make(tmpl,
                                          64,
                                          "4z2",
                                          9,
                                          kCirPre,
                                          kCirPost,
                                          10,
                                          0,
                                          64,
                                          8,
                                          0.3f,
                                          0.3f,
                                          true,
                                          16);
    auto w = UwbCirWriter::make(dir, "cir", /*write_norm=*/false, 128);
    auto tb = gr::make_top_block("qa_radar_e2e_100");
    tb->msg_connect(src, "tx", echo, "tx");
    tb->msg_connect(echo, "rx", res, "packet");
    tb->msg_connect(res, "packet", est, "rx");
    tb->msg_connect(est, "cir", w, "cir");
    tb->start();

    // Warmup frame (pages in every scratch), then RSS snapshot.
    src->_post(pmt::mp("emit"), pmt::make_dict());
    BOOST_REQUIRE(wait_until([&] {
        return w->frames_written() >= 1 && est->drained();
    }));
    size_t rss_before = 0, rss_after = 0;
    BOOST_REQUIRE(read_rss_bytes(&rss_before));

    constexpr uint64_t kFrames = 100;
    constexpr uint64_t kChunk = 8; // < estimator queue capacity
    const auto wall0 = std::chrono::steady_clock::now();
    for (uint64_t i = 1; i <= kFrames; ++i) {
        src->_post(pmt::mp("emit"), pmt::make_dict());
        if (i % kChunk == 0) {
            BOOST_REQUIRE(wait_until([&] {
                return est->drained() &&
                       w->frames_written() + w->frames_failed() >= i + 1;
            }));
        }
    }
    BOOST_REQUIRE(wait_until([&] {
        return w->frames_written() + w->frames_failed() >= kFrames + 1 &&
               est->drained();
    }));
    const auto wall1 = std::chrono::steady_clock::now();
    tb->stop();
    tb->wait();
    BOOST_REQUIRE(w->stop());
    BOOST_REQUIRE(read_rss_bytes(&rss_after));

    // Counters: no drops anywhere in the chain (101 = warmup + 100).
    BOOST_CHECK_EQUAL(src->pdus_dropped(), 0u);
    BOOST_CHECK_EQUAL(echo->pdus_dropped(), 0u);
    BOOST_CHECK_EQUAL(echo->pdus_emitted(), kFrames + 1);
    BOOST_CHECK_EQUAL(res->pdus_dropped(), 0u);
    BOOST_CHECK_EQUAL(res->pdus_emitted(), kFrames + 1);
    BOOST_CHECK_EQUAL(est->pdus_received(), kFrames + 1);
    BOOST_CHECK_EQUAL(est->pdus_enqueued(), kFrames + 1);
    BOOST_CHECK_EQUAL(est->pdus_dropped(), 0u);
    BOOST_CHECK_EQUAL(est->pdus_completed(), kFrames + 1);
    BOOST_CHECK_EQUAL(est->invalid_inputs(), 0u);
    BOOST_CHECK_EQUAL(w->frames_written(), kFrames + 1);
    BOOST_CHECK_EQUAL(w->frames_failed(), 0u);
    BOOST_CHECK_EQUAL(w->frames_dropped(), 0u);
    BOOST_CHECK_EQUAL(w->frames_invalid(), 0u);
    BOOST_TEST_MESSAGE("e2e_100_frames: wall_ms="
                       << std::chrono::duration_cast<std::chrono::milliseconds>(
                              wall1 - wall0)
                              .count()
                       << " est_queue_high_watermark="
                       << est->queue_high_watermark()
                       << " rss_before=" << rss_before
                       << " rss_after=" << rss_after
                       << " rss_drift="
                       << (rss_after > rss_before ? rss_after - rss_before
                                                  : 0));
    BOOST_CHECK_GE(est->queue_high_watermark(), 1u);
    BOOST_CHECK_LT(est->queue_high_watermark(), est->queue_capacity());

    // JSONL: one line per frame, ordered, monotonic offsets, all ok.
    const auto lines = read_lines(dir + "/cir.jsonl");
    BOOST_REQUIRE_EQUAL(lines.size(), kFrames + 1);
    for (size_t i = 0; i < lines.size(); ++i) {
        const JsonlLine j = parse_jsonl(lines[i]);
        BOOST_CHECK_EQUAL(j.status, "ok");
        BOOST_CHECK_EQUAL(j.pulse_id, i);
        BOOST_CHECK_EQUAL(j.tap_count, kTaps);
        // Coordinate consistency per frame (unified convention).
        BOOST_CHECK_LE(std::llabs(j.sfd - j.predicted), 2);
        BOOST_CHECK_LE(std::llabs(j.preamble - j.cir_origin), 2);
        BOOST_CHECK_EQUAL(j.file_offset,
                          static_cast<uint64_t>(i) * kTaps);
    }

    std::vector<uint8_t> raw;
    BOOST_REQUIRE(read_bytes(dir + "/cir.cf32", raw));
    BOOST_CHECK_EQUAL(raw.size(), (kFrames + 1) * kTaps * 8);

    // RSS must be bounded (allocator arena retention allowed, no growth
    // proportional to the frame count).
    const size_t drift =
        rss_after > rss_before ? rss_after - rss_before : 0;
    BOOST_CHECK_LT(drift, 200u * 1024 * 1024);
}

// ---------------------------------------------------------------------------
// e2e_10_frames / e2e_1000_frames: the plan §12 frame matrix 10/100/1000
// (100 = e2e_100_frames above, 6000 = the soak below).  Each size runs the
// real native chain with independent assertions: line count, order,
// offsets, zero drops, queue watermark and RSS stability.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(e2e_10_frames)
{
    run_frame_matrix(10, "f10");
}

BOOST_AUTO_TEST_CASE(e2e_1000_frames)
{
    run_frame_matrix(1000, "f1000");
}

// ---------------------------------------------------------------------------
// e2e_soak_200pps_30s: Phase-A exit condition — a real wall-clock 30 s
// software replay at 200 PDU/s (6000 PDUs) feeding estimator + writer
// with normalization on.  0 drops, bounded queue, counters consistent.
//
// Runtime note: ~35 s by default.  Gated by UWB_E2E_SOAK_SECONDS
// (default 30); the CTest registration uses the default so CI keeps the
// Phase-A exit condition coverage.  Not a UHD run: PDUs are posted into
// the estimator from a std::thread paced on the steady clock.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(e2e_soak_200pps_30s)
{
    long soak_s = 30;
    if (const char* env = std::getenv("UWB_E2E_SOAK_SECONDS")) {
        const long v = std::atol(env);
        if (v > 0)
            soak_s = std::min<long>(v, 120);
    }
    const uint64_t n_frames = static_cast<uint64_t>(200) *
                              static_cast<uint64_t>(soak_s);
    BOOST_TEST_MESSAGE("e2e_soak: seconds=" << soak_s
                                            << " frames=" << n_frames);

    std::vector<gr_complex> tx;
    BOOST_REQUIRE(load_cf32(testdata_path("uwb_radar/tx_998p4.cf32"), tx));
    const std::string tmpl = write_template_file(tx, "soak");
    std::vector<gr_complex> rx;
    BOOST_REQUIRE(load_cf32(testdata_path("uwb_radar/rx_clean_998p4.cf32"),
                            rx));
    BOOST_REQUIRE_EQUAL(rx.size(), 197005u);

    const std::string dir = make_out_dir("soak");
    auto est = UwbRadarCirEstimator::make(tmpl,
                                          64,
                                          "4z2",
                                          9,
                                          kCirPre,
                                          kCirPost,
                                          10,
                                          0,
                                          64,
                                          8,
                                          0.3f,
                                          0.3f,
                                          true,
                                          64);
    auto w = UwbCirWriter::make(dir, "cir", /*write_norm=*/true, 64);
    auto tb = gr::make_top_block("qa_radar_e2e_soak");
    tb->msg_connect(est, "cir", w, "cir");
    tb->start();

    // Shared immutable payload; only the (small) metadata dict is rebuilt
    // per frame so pulse ids stay observable in cir.jsonl.
    pmt::pmt_t samples_pmt = pmt::init_c32vector(rx.size(), rx.data());

    size_t rss_before = 0, rss_after = 0;
    BOOST_REQUIRE(read_rss_bytes(&rss_before));

    const auto t0 = std::chrono::steady_clock::now();
    std::thread poster([&] {
        const double period_s = 1.0 / 200.0; // 200 pulse/s → 5 ms
        for (uint64_t i = 0; i < n_frames; ++i) {
            const auto due = t0 + std::chrono::duration_cast<
                                     std::chrono::steady_clock::duration>(
                                     std::chrono::duration<double>(
                                         static_cast<double>(i) * period_s));
            std::this_thread::sleep_until(due);
            pmt::pmt_t meta = rx_meta(i, kPreGuardWork, 0.0);
            est->_post(pmt::mp("rx"), pmt::cons(meta, samples_pmt));
        }
    });
    poster.join();
    const auto t_post_done = std::chrono::steady_clock::now();

    BOOST_REQUIRE(wait_until([&] {
        return est->drained() &&
               w->frames_written() + w->frames_failed() >= n_frames;
    },
              (soak_s + 60) * 1000));
    const auto t1 = std::chrono::steady_clock::now();
    tb->stop();
    tb->wait();
    BOOST_REQUIRE(w->stop());
    BOOST_REQUIRE(read_rss_bytes(&rss_after));

    const auto wall_ms = [](auto a, auto b) {
        return std::chrono::duration_cast<std::chrono::milliseconds>(b - a)
            .count();
    };
    BOOST_TEST_MESSAGE("e2e_soak: post_ms=" << wall_ms(t0, t_post_done)
                                            << " total_ms=" << wall_ms(t0, t1)
                                            << " written=" << w->frames_written()
                                            << " failed=" << w->frames_failed()
                                            << " dropped=" << w->frames_dropped()
                                            << " received=" << w->frames_received()
                                            << " watermark="
                                            << est->queue_high_watermark()
                                            << " rss_drift="
                                            << (rss_after > rss_before
                                                    ? rss_after - rss_before
                                                    : 0));

    // Pacing must be honest: the replay spans the full soak window.
    BOOST_CHECK_GE(wall_ms(t0, t_post_done),
                   soak_s * 1000 - 100);

    // Counters: 0 drops, everything consistent.
    BOOST_CHECK_EQUAL(est->pdus_received(), n_frames);
    BOOST_CHECK_EQUAL(est->pdus_enqueued(), n_frames);
    BOOST_CHECK_EQUAL(est->pdus_dropped(), 0u);
    BOOST_CHECK_EQUAL(est->pdus_completed(), n_frames);
    BOOST_CHECK_EQUAL(est->pdus_failed(), 0u);
    BOOST_CHECK_EQUAL(est->invalid_inputs(), 0u);
    BOOST_CHECK_EQUAL(est->worker_exceptions(), 0u);
    BOOST_CHECK_EQUAL(w->frames_received(), n_frames);
    BOOST_CHECK_EQUAL(w->frames_written(), n_frames);
    BOOST_CHECK_EQUAL(w->frames_failed(), 0u);
    BOOST_CHECK_EQUAL(w->frames_dropped(), 0u);
    BOOST_CHECK_EQUAL(w->frames_invalid(), 0u);
    BOOST_CHECK_EQUAL(w->taps_written(), n_frames * kTaps);
    // Queue must stay bounded, never reaching capacity.
    BOOST_CHECK_GE(est->queue_high_watermark(), 1u);
    BOOST_CHECK_LT(est->queue_high_watermark(), est->queue_capacity());

    // Files: exactly one line per frame, ordered, offsets monotonic.
    const auto lines = read_lines(dir + "/cir.jsonl");
    BOOST_REQUIRE_EQUAL(lines.size(), n_frames);
    for (size_t i = 0; i < lines.size(); ++i) {
        const JsonlLine j = parse_jsonl(lines[i]);
        BOOST_CHECK_EQUAL(j.status, "ok");
        BOOST_CHECK_EQUAL(j.pulse_id, i);
        BOOST_CHECK_EQUAL(j.tap_count, kTaps);
        BOOST_CHECK_EQUAL(j.file_offset,
                          static_cast<uint64_t>(i) * kTaps);
    }
    std::vector<uint8_t> raw, norm;
    BOOST_REQUIRE(read_bytes(dir + "/cir.cf32", raw));
    BOOST_REQUIRE(read_bytes(dir + "/cir_norm.cf32", norm));
    BOOST_CHECK_EQUAL(raw.size(), n_frames * kTaps * 8);
    BOOST_CHECK_EQUAL(norm.size(), n_frames * kTaps * 8);

    // RSS must not grow with the frame count (bounded queues).
    const size_t drift =
        rss_after > rss_before ? rss_after - rss_before : 0;
    BOOST_CHECK_LT(drift, 200u * 1024 * 1024);
}
