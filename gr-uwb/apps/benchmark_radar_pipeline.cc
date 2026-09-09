/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Host-side timing of the monostatic radar sensing pipeline.
 *
 * Measures the production burst path after an RX window is already on the
 * host (UHD send/recv is radio occupancy, not timed here):
 *
 *   native SC16/FC32 window
 *     → PDU 65/48 (UC200) or 65/32 (CG400)   [scheduler-thread handler]
 *     → UwbRadarCirEstimator                  [single worker]
 *     → UwbCirWriter                          [single worker]
 *
 * Also splits radar_cir_one into SFD / SYNC-refine / CIR cores.
 *
 * Usage:
 *   benchmark_radar_pipeline [rounds] [pdus]
 *     rounds  timed repetitions per microbench (default 32)
 *     pdus    frames in block / e2e runs     (default 64)
 *
 * Run from the workspace root or gr-uwb/build so testdata/ is visible.
 */

#include <gnuradio/blocks/message_debug.h>
#include <gnuradio/top_block.h>
#include <gnuradio/uwb/uwb_cir_writer.h>
#include <gnuradio/uwb/uwb_loopback_echo.h>
#include <gnuradio/uwb/uwb_pdu_rational_resampler_ccf_65_32.h>
#include <gnuradio/uwb/uwb_pdu_rational_resampler_ccf_65_48.h>
#include <gnuradio/uwb/uwb_radar_cir_core.h>
#include <gnuradio/uwb/uwb_radar_cir_estimator_block.h>
#include <gnuradio/uwb/uwb_radar_packet_source.h>
#include <gnuradio/uwb/uwb_radar_pdu_meta.h>
#include <gnuradio/uwb/uwb_rational_resampler_core.h>
#include <pmt/pmt.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>

using gr::uwb::UwbCirWriter;
using gr::uwb::UwbLoopbackEcho;
using gr::uwb::UwbPduRationalResamplerCcf65_32;
using gr::uwb::UwbPduRationalResamplerCcf65_48;
using gr::uwb::UwbRadarCirEstimator;
using gr::uwb::UwbRadarPacketSource;
using gr::uwb::core::RationalResampler65_32Core;
using gr::uwb::core::RationalResampler65_48Core;
using gr::uwb::radar::estimate_radar_cir;
using gr::uwb::radar::prepare_radar_cir_core;
using gr::uwb::radar::radar_cir_one;
using gr::uwb::radar::RadarCirConfig;
using gr::uwb::radar::RadarCirCoreScratch;
using gr::uwb::radar::RadarCirEstimate;
using gr::uwb::radar::RadarCirResult;
using gr::uwb::radar::RadarCirStatus;
using gr::uwb::radar::RadarSfdResult;
using gr::uwb::radar::RadarTimingResult;
using gr::uwb::radar::refine_sync_origin;
using gr::uwb::radar::search_sfd;
using gr_complex = std::complex<float>;

namespace {

constexpr double kC = 299792458.0;
constexpr double kFsWork = 998.4e6;
constexpr double kFsUc200 = 737.28e6;
constexpr double kFsCg400 = 491.52e6;
constexpr size_t kSps = 1016;
constexpr size_t kSyncReps = 64;
constexpr size_t kSfdSyms = 8;
constexpr size_t kCirPre = 16;
constexpr size_t kCirPost = 100;
constexpr size_t kCirSkip = 10;
constexpr int64_t kSfdMargin = 64;
constexpr int64_t kSyncMargin = 8;
constexpr float kThr = 0.3f;
constexpr double kPreUs = 2.0;
constexpr double kTailUs = 4.1;
constexpr double kRangeM = 15.0;
constexpr uint32_t kInterp = 65;
constexpr size_t kTxWork = 190912;
constexpr size_t kTxUc200 = 140982;
constexpr size_t kTxCg400 = 93988;

struct Stats {
    size_t n = 0;
    double min_us = 0;
    double max_us = 0;
    double mean_us = 0;
    double p50_us = 0;
    double p95_us = 0;
    double p99_us = 0;
};

struct Geom {
    const char* name = "";
    double rate_hz = 0;
    uint32_t decim = 0;
    size_t tx_samples = 0;
    size_t pre = 0;
    size_t sync = 0;
    size_t sfd = 0;
    size_t range = 0;
    size_t tail = 0;
    size_t rx = 0;
    double tx_us = 0;
    double rx_us = 0;
    double occupancy_us = 0;
    double radio_max_pps = 0;
};

int64_t
llround_i64(double x)
{
    return static_cast<int64_t>(std::llround(x));
}

size_t
ceildiv(uint64_t a, uint64_t b)
{
    return static_cast<size_t>((a + b - 1) / b);
}

Geom
make_geom(const char* name, double rate, uint32_t decim, size_t tx)
{
    Geom g;
    g.name = name;
    g.rate_hz = rate;
    g.decim = decim;
    g.tx_samples = tx;
    g.pre = static_cast<size_t>(llround_i64(kPreUs * 1e-6 * rate));
    g.sync = ceildiv(static_cast<uint64_t>(kSyncReps) * kSps * decim, kInterp);
    g.sfd = ceildiv(static_cast<uint64_t>(kSfdSyms) * kSps * decim, kInterp);
    g.range = static_cast<size_t>(
        std::ceil(2.0 * kRangeM / kC * rate));
    g.tail = static_cast<size_t>(llround_i64(kTailUs * 1e-6 * rate));
    g.rx = g.pre + g.sync + g.sfd + g.range + g.tail;
    g.tx_us = 1e6 * static_cast<double>(tx) / rate;
    g.rx_us = 1e6 * static_cast<double>(g.rx) / rate;
    const double pre_us = 1e6 * static_cast<double>(g.pre) / rate;
    g.occupancy_us = pre_us + std::max(g.tx_us, g.rx_us - pre_us);
    g.radio_max_pps = 1e6 / g.occupancy_us;
    return g;
}

Stats
summarize(std::vector<double> xs)
{
    Stats s;
    if (xs.empty())
        return s;
    std::sort(xs.begin(), xs.end());
    s.n = xs.size();
    s.min_us = xs.front();
    s.max_us = xs.back();
    s.mean_us = std::accumulate(xs.begin(), xs.end(), 0.0) /
                static_cast<double>(xs.size());
    auto at = [&](double p) {
        const double idx = p * static_cast<double>(xs.size() - 1);
        return xs[static_cast<size_t>(std::llround(idx))];
    };
    s.p50_us = at(0.50);
    s.p95_us = at(0.95);
    s.p99_us = at(0.99);
    return s;
}

void
print_stats(const char* name, const Stats& s)
{
    std::printf("  %-36s n=%3zu  mean=%8.1f  p50=%8.1f  p95=%8.1f  "
                "p99=%8.1f  max=%8.1f us\n",
                name, s.n, s.mean_us, s.p50_us, s.p95_us, s.p99_us, s.max_us);
}

template <typename F>
Stats
time_loop(size_t warmup, size_t rounds, F&& fn)
{
    for (size_t i = 0; i < warmup; ++i)
        fn();
    std::vector<double> xs;
    xs.reserve(rounds);
    for (size_t i = 0; i < rounds; ++i) {
        const auto t0 = std::chrono::steady_clock::now();
        fn();
        const auto t1 = std::chrono::steady_clock::now();
        xs.push_back(1e-3 * static_cast<double>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0)
                .count()));
    }
    return summarize(std::move(xs));
}

std::string
testdata_path(const std::string& rel)
{
    const char* env = std::getenv("UWB_TESTDATA_DIR");
    if (env && env[0]) {
        const std::string p = std::string(env) + "/" + rel;
        if (std::ifstream(p, std::ios::binary))
            return p;
    }
    const char* prefixes[] = {
        "testdata/",
        "../testdata/",
        "../../testdata/",
        "../../../testdata/",
        "../../../../testdata/",
        "/home/junqima/workspace/uwb-gnuradio/testdata/",
    };
    for (const char* p : prefixes) {
        const std::string path = std::string(p) + rel;
        if (std::ifstream(path, std::ios::binary))
            return path;
    }
    throw std::runtime_error("cannot find testdata/" + rel);
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
load_taps(const std::string& rel)
{
    const std::string path = testdata_path(rel);
    std::ifstream f(path, std::ios::binary | std::ios::ate);
    if (!f)
        throw std::runtime_error("cannot open taps " + path);
    const auto bytes = static_cast<size_t>(f.tellg());
    if (bytes == 0 || bytes % sizeof(float) != 0)
        throw std::runtime_error("bad taps size " + path);
    f.seekg(0);
    std::vector<float> v(bytes / sizeof(float));
    f.read(reinterpret_cast<char*>(v.data()),
           static_cast<std::streamsize>(bytes));
    return v;
}

std::vector<gr_complex>
embed(const std::vector<gr_complex>& tx, size_t pre, size_t n_rx)
{
    std::vector<gr_complex> rx(n_rx, gr_complex(0.f, 0.f));
    if (pre >= n_rx)
        return rx;
    const size_t ncopy = std::min(tx.size(), n_rx - pre);
    std::memcpy(rx.data() + pre, tx.data(), ncopy * sizeof(gr_complex));
    return rx;
}

std::vector<int16_t>
to_sc16(const std::vector<gr_complex>& x)
{
    float peak = 0.f;
    for (const auto& z : x)
        peak = std::max(peak, std::abs(z));
    const float s = (peak > 0.f) ? (30000.f / peak) : 1.f;
    std::vector<int16_t> y(x.size() * 2);
    for (size_t i = 0; i < x.size(); ++i) {
        y[2 * i] = static_cast<int16_t>(std::lround(x[i].real() * s));
        y[2 * i + 1] = static_cast<int16_t>(std::lround(x[i].imag() * s));
    }
    return y;
}

std::string
write_template(const std::vector<gr_complex>& tx)
{
    if (tx.size() < kSps)
        throw std::runtime_error("TX shorter than one SYNC");
    const auto dir = std::filesystem::temp_directory_path() /
                     "uwb_radar_pipeline_bench";
    std::filesystem::create_directories(dir);
    const std::string path = (dir / "sync_template.cf32").string();
    std::ofstream f(path, std::ios::binary | std::ios::trunc);
    f.write(reinterpret_cast<const char*>(tx.data()),
            static_cast<std::streamsize>(kSps * sizeof(gr_complex)));
    if (!f)
        throw std::runtime_error("cannot write " + path);
    return path;
}

RadarCirConfig
cfg()
{
    RadarCirConfig c;
    c.sync_repetitions = kSyncReps;
    c.samples_per_symbol = kSps;
    c.sfd_mode = "4z2";
    c.sfd_search_margin = kSfdMargin;
    c.sync_refine_margin = kSyncMargin;
    c.sfd_threshold = kThr;
    c.sync_refine_threshold = kThr;
    c.cir_pre = kCirPre;
    c.cir_post = kCirPost;
    c.cir_skip_initial = kCirSkip;
    c.cir_repetitions = kSyncReps - kCirSkip;
    return c;
}

pmt::pmt_t
native_meta(uint64_t pulse_id,
            double rate,
            size_t pre,
            size_t capture,
            size_t post,
            size_t n)
{
    pmt::pmt_t m = pmt::make_dict();
    m = pmt::dict_add(m, pmt::mp("pulse_id"), pmt::from_uint64(pulse_id));
    m = pmt::dict_add(m, pmt::mp("schedule_index"), pmt::from_uint64(pulse_id));
    m = pmt::dict_add(m, pmt::mp("sample_rate"), pmt::from_double(rate));
    m = pmt::dict_add(m, pmt::mp("window_start_sample"), pmt::from_long(0));
    m = pmt::dict_add(m, pmt::mp("pre_guard_samples"),
                      pmt::from_long(static_cast<long>(pre)));
    m = pmt::dict_add(m, pmt::mp("capture_samples"),
                      pmt::from_long(static_cast<long>(capture)));
    m = pmt::dict_add(m, pmt::mp("post_guard_samples"),
                      pmt::from_long(static_cast<long>(post)));
    m = pmt::dict_add(m, pmt::mp("sample_count"),
                      pmt::from_long(static_cast<long>(n)));
    m = pmt::dict_add(m, pmt::mp("calibration_delay_native_samples"),
                      pmt::from_double(0.0));
    m = pmt::dict_add(m, pmt::mp("sync_repetitions"), pmt::from_long(64));
    m = pmt::dict_add(m, pmt::mp("sfd_mode"), pmt::mp("4z2"));
    m = pmt::dict_add(m, pmt::mp("code_index"), pmt::from_long(9));
    m = pmt::dict_add(m, pmt::mp("source"), pmt::mp("bench"));
    m = pmt::dict_add(m, pmt::mp("uhd_error"), pmt::mp("none"));
    return m;
}

pmt::pmt_t
work_meta(uint64_t pulse_id, int64_t pre)
{
    pmt::pmt_t m = pmt::make_dict();
    m = pmt::dict_add(m, pmt::mp("pulse_id"), pmt::from_uint64(pulse_id));
    m = pmt::dict_add(m, pmt::mp("schedule_index"), pmt::from_uint64(pulse_id));
    m = pmt::dict_add(m, pmt::mp("sample_rate"), pmt::from_double(kFsWork));
    m = pmt::dict_add(m, pmt::mp("pre_guard_samples"), pmt::from_long(pre));
    m = pmt::dict_add(m, pmt::mp("calibration_delay_native_samples"),
                      pmt::from_double(0.0));
    m = pmt::dict_add(m, pmt::mp("sync_repetitions"), pmt::from_long(64));
    m = pmt::dict_add(m, pmt::mp("sfd_mode"), pmt::mp("4z2"));
    m = pmt::dict_add(m, pmt::mp("code_index"), pmt::from_long(9));
    m = pmt::dict_add(m, pmt::mp("source"), pmt::mp("bench"));
    m = pmt::dict_add(m, pmt::mp("uhd_error"), pmt::mp("none"));
    return m;
}

template <typename Pred>
bool
wait_until(Pred&& pred, long timeout_ms)
{
    const auto t0 = std::chrono::steady_clock::now();
    while (!pred()) {
        const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                            std::chrono::steady_clock::now() - t0)
                            .count();
        if (ms > timeout_ms)
            return false;
        std::this_thread::sleep_for(std::chrono::microseconds(200));
    }
    return true;
}

void
print_geom(const Geom& g)
{
    std::printf("  %-8s  fs=%.2f MS/s  TX=%zu (%.1f us)  RX=%zu (%.1f us)\n",
                g.name, g.rate_hz / 1e6, g.tx_samples, g.tx_us, g.rx, g.rx_us);
    std::printf("           pre=%zu  SYNC=%zu  SFD=%zu  range=%zu  tail=%zu\n",
                g.pre, g.sync, g.sfd, g.range, g.tail);
    std::printf("           occupancy=%.1f us  radio-max=%.0f pulse/s  "
                "(TX airtime bound)\n",
                g.occupancy_us, g.radio_max_pps);
}

} // namespace

int
main(int argc, char** argv)
{
    try {
    const size_t rounds = (argc > 1) ? std::stoul(argv[1]) : 32;
    const size_t n_pdus = (argc > 2) ? std::stoul(argv[2]) : 64;
    const size_t warmup = 4;

    std::printf("== UWB radar sensing pipeline timing ==\n");
    std::printf("host rounds=%zu pdus=%zu warmup=%zu\n", rounds, n_pdus,
                warmup);

    const Geom uc200 = make_geom("UC200", kFsUc200, 48, kTxUc200);
    const Geom cg400 = make_geom("CG400", kFsCg400, 32, kTxCg400);
    const Geom workg = make_geom("work", kFsWork, 65, kTxWork);

    std::printf("\n-- geometry (CLI defaults: 2 us pre, 15 m range, 4.1 us "
                "tail, 64-SYNC 4z2) --\n");
    print_geom(uc200);
    print_geom(cg400);
    std::printf("  work     fs=998.40 MS/s  TX=%zu (%.1f us)  "
                "RX-equivalent pre+SYNC+SFD+range+tail=%zu\n",
                workg.tx_samples, workg.tx_us, workg.rx);

    std::vector<gr_complex> tx_work, tx_uc, tx_cg;
    if (!load_cf32(testdata_path("uwb_radar/tx_998p4.cf32"), tx_work) ||
        tx_work.size() != kTxWork)
        throw std::runtime_error("tx_998p4.cf32 missing or wrong length");
    if (!load_cf32(testdata_path("uwb_radar/tx_737p28.cf32"), tx_uc) ||
        tx_uc.size() != kTxUc200)
        throw std::runtime_error("tx_737p28.cf32 missing or wrong length");
    if (!load_cf32(testdata_path("uwb_radar/tx_491p52.cf32"), tx_cg) ||
        tx_cg.size() != kTxCg400)
        throw std::runtime_error("tx_491p52.cf32 missing or wrong length");

    const std::string tmpl = write_template(tx_work);
    const auto rx_work = embed(tx_work, workg.pre, workg.rx);
    const auto rx_uc = embed(tx_uc, uc200.pre, uc200.rx);
    const auto rx_cg = embed(tx_cg, cg400.pre, cg400.rx);
    const auto rx_uc_s16 = to_sc16(rx_uc);
    const auto rx_cg_s16 = to_sc16(rx_cg);
    const int64_t pred_work =
        static_cast<int64_t>(workg.pre) +
        static_cast<int64_t>(kSyncReps) * static_cast<int64_t>(kSps);

    RadarCirCoreScratch scratch;
    if (!prepare_radar_cir_core(cfg(), tx_work.data(), kSps, kCirPre, kCirPost,
                                scratch))
        throw std::runtime_error("prepare_radar_cir_core failed");

    RadarCirResult one;
    if (!radar_cir_one(rx_work.data(), rx_work.size(), pred_work, cfg(),
                       scratch, one) ||
        one.status != RadarCirStatus::Ok) {
        throw std::runtime_error("production-window radar_cir_one failed");
    }
    std::printf("\nproduction work window CIR ok: n=%zu predicted_sfd=%ld "
                "sfd=%ld origin=%ld peak_tap=%zu\n",
                rx_work.size(), static_cast<long>(pred_work),
                static_cast<long>(one.sfd_start_sample),
                static_cast<long>(one.cir_origin_sample), one.peak_tap);

    // ------------------------------------------------------------------
    // Core microbenchmarks
    // ------------------------------------------------------------------
    std::printf("\n-- core (no GNU Radio scheduler) --\n");

    print_stats("SFD search ±64", time_loop(warmup, rounds, [&] {
                    RadarSfdResult sfd;
                    search_sfd(rx_work.data(), rx_work.size(), pred_work,
                               kSfdMargin, kThr, sfd, scratch.sfd);
                    if (sfd.status != gr::uwb::radar::SfdStatus::Ok)
                        throw std::runtime_error("SFD failed");
                }));
    print_stats("SYNC refine ±8", time_loop(warmup, rounds, [&] {
                    RadarTimingResult t;
                    refine_sync_origin(rx_work.data(), rx_work.size(),
                                       pred_work, kSyncReps, kSps, kSyncMargin,
                                       kThr, scratch.sync_template.data(),
                                       scratch.sync_template.size(), t);
                    if (t.status != gr::uwb::radar::TimingStatus::Ok)
                        throw std::runtime_error("timing failed");
                }));
    print_stats("estimateCir 54-rep", time_loop(warmup, rounds, [&] {
                    RadarCirEstimate cir;
                    if (!estimate_radar_cir(rx_work.data(), rx_work.size(),
                                            static_cast<int64_t>(workg.pre),
                                            kSps, kCirPre, kCirPost, kCirSkip,
                                            kSyncReps - kCirSkip, kSyncReps,
                                            cir, scratch.cir) ||
                        cir.status != gr::uwb::radar::CirStatus::Ok)
                        throw std::runtime_error("CIR failed");
                }));
    print_stats("radar_cir_one (all 3)", time_loop(warmup, rounds, [&] {
                    RadarCirResult r;
                    if (!radar_cir_one(rx_work.data(), rx_work.size(),
                                       pred_work, cfg(), scratch, r) ||
                        r.status != RadarCirStatus::Ok)
                        throw std::runtime_error("radar_cir_one failed");
                }));

    auto taps48 = load_taps("resampler_65_48/taps_quality_minorder.txt");
    auto taps32 = load_taps("resampler_65_32/taps_quality_minorder.txt");
    auto taps48rt = load_taps("resampler_65_48/taps_realtime_minorder.txt");
    auto taps32rt = load_taps("resampler_65_32/taps_realtime_minorder.txt");
    std::printf("  taps quality_minorder 65/48=%zu  65/32=%zu; "
                "realtime_minorder 65/48=%zu  65/32=%zu\n",
                taps48.size(), taps32.size(), taps48rt.size(), taps32rt.size());

    auto bench_resample = [&](const char* name, auto& core,
                              const std::vector<gr_complex>& xin) {
        const size_t nout = core.expected_output_length(xin.size());
        std::vector<gr_complex> y(nout + 8);
        print_stats(name, time_loop(warmup, rounds, [&] {
                        core.reset();
                        const auto pr =
                            core.process(xin.data(), xin.size(), y.data(),
                                         y.size());
                        const size_t nflush =
                            core.flush(y.data() + pr.produced,
                                       y.size() - pr.produced);
                        volatile size_t sink = pr.produced + nflush;
                        (void)sink;
                    }));
    };

    {
        RationalResampler65_48Core c48(taps48);
        RationalResampler65_32Core c32(taps32);
        RationalResampler65_48Core c48rt(taps48rt);
        RationalResampler65_32Core c32rt(taps32rt);
        std::printf("  kernel 65/48=%s  65/32=%s\n", c48.kernel_name(),
                    c32.kernel_name());
        bench_resample("65/48 quality RX window", c48, rx_uc);
        bench_resample("65/32 quality RX window", c32, rx_cg);
        bench_resample("65/48 realtime RX window", c48rt, rx_uc);
        bench_resample("65/32 realtime RX window", c32rt, rx_cg);

        const auto rx_uc_full = embed(tx_uc, uc200.pre, uc200.pre + kTxUc200 +
                                                            uc200.tail);
        const auto rx_cg_full = embed(tx_cg, cg400.pre, cg400.pre + kTxCg400 +
                                                            cg400.tail);
        bench_resample("65/48 quality full TX+tail", c48, rx_uc_full);
        bench_resample("65/32 quality full TX+tail", c32, rx_cg_full);
    }

    print_stats("SC16→FC32 UC200 window", time_loop(warmup, rounds, [&] {
                    std::vector<gr_complex> tmp(rx_uc.size());
                    const int16_t* s = rx_uc_s16.data();
                    for (size_t i = 0; i < rx_uc.size(); ++i)
                        tmp[i] = gr_complex(static_cast<float>(s[2 * i]),
                                            static_cast<float>(s[2 * i + 1]));
                    volatile float sink = tmp[0].real();
                    (void)sink;
                }));
    print_stats("SC16→FC32 CG400 window", time_loop(warmup, rounds, [&] {
                    std::vector<gr_complex> tmp(rx_cg.size());
                    const int16_t* s = rx_cg_s16.data();
                    for (size_t i = 0; i < rx_cg.size(); ++i)
                        tmp[i] = gr_complex(static_cast<float>(s[2 * i]),
                                            static_cast<float>(s[2 * i + 1]));
                    volatile float sink = tmp[0].real();
                    (void)sink;
                }));

    // ------------------------------------------------------------------
    // PDU blocks
    // ------------------------------------------------------------------
    std::printf("\n-- PDU blocks (GNU Radio message handlers / workers) --\n");

    auto bench_resampler_block = [&](const char* name, auto blk,
                                     pmt::pmt_t pdu_proto) {
        auto dbg = gr::blocks::message_debug::make();
        auto tb = gr::make_top_block("bench_resamp");
        tb->msg_connect(blk, "packet", dbg, "store");
        tb->start();
        auto post_wait = [&](uint64_t id) {
            pmt::pmt_t meta = pmt::car(pdu_proto);
            meta = pmt::dict_add(meta, pmt::mp("pulse_id"),
                                 pmt::from_uint64(id));
            pmt::pmt_t pdu = pmt::cons(meta, pmt::cdr(pdu_proto));
            const uint64_t before = blk->pdus_emitted();
            blk->_post(pmt::mp("packet"), pdu);
            if (!wait_until([&] { return blk->pdus_emitted() >= before + 1; },
                            5000))
                throw std::runtime_error(std::string(name) + " timeout");
        };
        post_wait(0);
        blk->reset_stats();
        std::vector<double> xs;
        xs.reserve(n_pdus);
        for (size_t i = 0; i < n_pdus; ++i) {
            const auto t0 = std::chrono::steady_clock::now();
            post_wait(i + 1);
            const auto t1 = std::chrono::steady_clock::now();
            xs.push_back(1e-3 * static_cast<double>(
                std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0)
                    .count()));
        }
        tb->stop();
        tb->wait();
        print_stats(name, summarize(xs));
        if (blk->pdus_emitted() > 0) {
            const double hmean = static_cast<double>(blk->handler_total_us()) /
                                 static_cast<double>(blk->pdus_emitted());
            const double rmean = static_cast<double>(blk->resample_total_us()) /
                                 static_cast<double>(blk->pdus_emitted());
            const double cmean =
                static_cast<double>(blk->input_convert_total_us()) /
                static_cast<double>(blk->pdus_emitted());
            const double pmean = static_cast<double>(blk->publish_total_us()) /
                                 static_cast<double>(blk->pdus_emitted());
            std::printf("           internal  handler=%.1f  fir=%.1f  "
                        "sc16=%.1f  publish=%.1f us/pdu  max_fir=%llu us\n",
                        hmean, rmean, cmean, pmean,
                        static_cast<unsigned long long>(blk->resample_max_us()));
        }
        return summarize(xs);
    };

    pmt::pmt_t pdu_uc_fc32 = pmt::cons(
        native_meta(0, kFsUc200, uc200.pre, uc200.sync,
                    uc200.rx - uc200.pre - uc200.sync, rx_uc.size()),
        pmt::init_c32vector(rx_uc.size(), rx_uc.data()));
    pmt::pmt_t pdu_uc_sc16 = pmt::cons(
        native_meta(0, kFsUc200, uc200.pre, uc200.sync,
                    uc200.rx - uc200.pre - uc200.sync, rx_uc.size()),
        pmt::init_s16vector(rx_uc_s16.size(), rx_uc_s16.data()));
    pmt::pmt_t pdu_cg_fc32 = pmt::cons(
        native_meta(0, kFsCg400, cg400.pre, cg400.sync,
                    cg400.rx - cg400.pre - cg400.sync, rx_cg.size()),
        pmt::init_c32vector(rx_cg.size(), rx_cg.data()));
    pmt::pmt_t pdu_cg_sc16 = pmt::cons(
        native_meta(0, kFsCg400, cg400.pre, cg400.sync,
                    cg400.rx - cg400.pre - cg400.sync, rx_cg.size()),
        pmt::init_s16vector(rx_cg_s16.size(), rx_cg_s16.data()));

    const Stats st_r48 = bench_resampler_block(
        "PDU 65/48 FC32 quality",
        UwbPduRationalResamplerCcf65_48::make_from_taps(taps48), pdu_uc_fc32);
    const Stats st_r48s = bench_resampler_block(
        "PDU 65/48 SC16 quality",
        UwbPduRationalResamplerCcf65_48::make_from_taps(taps48), pdu_uc_sc16);
    const Stats st_r32 = bench_resampler_block(
        "PDU 65/32 FC32 quality",
        UwbPduRationalResamplerCcf65_32::make_from_taps(taps32), pdu_cg_fc32);
    const Stats st_r32s = bench_resampler_block(
        "PDU 65/32 SC16 quality",
        UwbPduRationalResamplerCcf65_32::make_from_taps(taps32), pdu_cg_sc16);
    (void)st_r48;
    (void)st_r48s;
    (void)st_r32;
    (void)st_r32s;

    bench_resampler_block(
        "PDU 65/48 FC32 realtime",
        UwbPduRationalResamplerCcf65_48::make_from_taps(taps48rt),
        pdu_uc_fc32);
    bench_resampler_block(
        "PDU 65/32 FC32 realtime",
        UwbPduRationalResamplerCcf65_32::make_from_taps(taps32rt),
        pdu_cg_fc32);

    // CIR estimator (work-domain production window)
    {
        auto est = UwbRadarCirEstimator::make(tmpl, 64, "4z2", 9, kCirPre,
                                              kCirPost, 10, 0, 64, 8, kThr,
                                              kThr, true, 64);
        auto dbg = gr::blocks::message_debug::make();
        auto tb = gr::make_top_block("bench_est");
        tb->msg_connect(est, "cir", dbg, "store");
        tb->start();
        pmt::pmt_t samples =
            pmt::init_c32vector(rx_work.size(), rx_work.data());
        auto post_one = [&](uint64_t id) {
            est->_post(pmt::mp("rx"),
                       pmt::cons(work_meta(id, static_cast<int64_t>(workg.pre)),
                                 samples));
        };
        post_one(0);
        if (!wait_until([&] { return est->drained() && est->pdus_completed() >= 1; },
                        5000))
            throw std::runtime_error("estimator warmup timeout");
        est->reset_stats();
        std::vector<double> xs;
        xs.reserve(n_pdus);
        for (size_t i = 0; i < n_pdus; ++i) {
            const uint64_t before = est->pdus_completed();
            const auto t0 = std::chrono::steady_clock::now();
            post_one(i + 1);
            if (!wait_until(
                    [&] {
                        return est->pdus_completed() >= before + 1 &&
                               est->drained();
                    },
                    5000))
                throw std::runtime_error("estimator timeout");
            const auto t1 = std::chrono::steady_clock::now();
            xs.push_back(1e-3 * static_cast<double>(
                std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0)
                    .count()));
        }
        tb->stop();
        tb->wait();
        print_stats("CirEstimator serial (work window)", summarize(xs));
        std::printf("           service  mean=%llu  p95=%llu  p99=%llu  "
                    "max=%llu us  completed=%llu dropped=%llu\n",
                    static_cast<unsigned long long>(est->service_mean_us()),
                    static_cast<unsigned long long>(est->service_p95_us()),
                    static_cast<unsigned long long>(est->service_p99_us()),
                    static_cast<unsigned long long>(est->service_max_us()),
                    static_cast<unsigned long long>(est->pdus_completed()),
                    static_cast<unsigned long long>(est->pdus_dropped()));
    }

    // Writer: replay one ok CIR PDU.  The writer must sit in a running
    // top_block so its message handler is dispatched.
    {
        auto est = UwbRadarCirEstimator::make(tmpl, 64, "4z2", 9, kCirPre,
                                              kCirPost, 10, 0, 64, 8, kThr,
                                              kThr, true, 8);
        auto dbg = gr::blocks::message_debug::make();
        auto tb = gr::make_top_block("bench_one_cir");
        tb->msg_connect(est, "cir", dbg, "store");
        tb->start();
        est->_post(pmt::mp("rx"),
                   pmt::cons(work_meta(0, static_cast<int64_t>(workg.pre)),
                             pmt::init_c32vector(rx_work.size(),
                                                 rx_work.data())));
        if (!wait_until([&] { return dbg->num_messages() >= 1; }, 5000))
            throw std::runtime_error("writer warmup CIR timeout");
        tb->stop();
        tb->wait();
        pmt::pmt_t cir_pdu = dbg->get_message(0);

        const auto dir = std::filesystem::temp_directory_path() /
                         "uwb_radar_pipeline_bench_cir";
        std::filesystem::remove_all(dir);
        std::filesystem::create_directories(dir);
        auto w = UwbCirWriter::make(dir.string(), "cir", true, 64);
        auto dbg_st = gr::blocks::message_debug::make();
        auto tbw = gr::make_top_block("bench_writer");
        tbw->msg_connect(w, "status", dbg_st, "store");
        tbw->start();
        w->_post(pmt::mp("cir"), cir_pdu);
        if (!wait_until([&] { return w->frames_written() >= 1; }, 5000))
            throw std::runtime_error("writer warmup timeout");
        const uint64_t base = w->frames_written();
        std::vector<double> xs;
        xs.reserve(n_pdus);
        for (size_t i = 0; i < n_pdus; ++i) {
            const auto t0 = std::chrono::steady_clock::now();
            w->_post(pmt::mp("cir"), cir_pdu);
            if (!wait_until(
                    [&] { return w->frames_written() >= base + i + 1; }, 5000))
                throw std::runtime_error("writer timeout");
            const auto t1 = std::chrono::steady_clock::now();
            xs.push_back(1e-3 * static_cast<double>(
                std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0)
                    .count()));
        }
        tbw->stop();
        tbw->wait();
        print_stats("CirWriter serial (116 taps+jsonl)", summarize(xs));
    }

    // Loopback (QA only): full packet insert.
    {
        auto src = UwbRadarPacketSource::make(
            testdata_path("uwb_radar/tx_737p28.cf32"), kFsUc200, "fc32");
        auto echo = UwbLoopbackEcho::make(uc200.pre, uc200.tail, { 0.0 },
                                          { gr_complex(1.f, 0.f) });
        auto dbg = gr::blocks::message_debug::make();
        auto tb = gr::make_top_block("bench_loop");
        tb->msg_connect(src, "tx", echo, "tx");
        tb->msg_connect(echo, "rx", dbg, "store");
        tb->start();
        src->_post(pmt::mp("emit"), pmt::make_dict());
        if (!wait_until([&] { return echo->pdus_emitted() >= 1; }, 5000))
            throw std::runtime_error("loopback warmup timeout");
        std::vector<double> xs;
        xs.reserve(n_pdus);
        for (size_t i = 0; i < n_pdus; ++i) {
            const uint64_t before = echo->pdus_emitted();
            const auto t0 = std::chrono::steady_clock::now();
            src->_post(pmt::mp("emit"), pmt::make_dict());
            if (!wait_until([&] { return echo->pdus_emitted() >= before + 1; },
                            5000))
                throw std::runtime_error("loopback timeout");
            const auto t1 = std::chrono::steady_clock::now();
            xs.push_back(1e-3 * static_cast<double>(
                std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0)
                    .count()));
        }
        tb->stop();
        tb->wait();
        print_stats("LoopbackEcho full native TX (QA)", summarize(xs));
    }

    // ------------------------------------------------------------------
    // Saturated e2e (production window, keep 16 in flight)
    // ------------------------------------------------------------------
    std::printf("\n-- saturated e2e (production RX window, depth 8) --\n");

    auto run_e2e = [&](const char* name, auto res, pmt::pmt_t pdu_proto,
                       size_t n) {
        const auto dir = std::filesystem::temp_directory_path() /
                         ("uwb_radar_pipeline_e2e_" + std::string(name));
        std::filesystem::remove_all(dir);
        std::filesystem::create_directories(dir);
        auto est = UwbRadarCirEstimator::make(tmpl, 64, "4z2", 9, kCirPre,
                                              kCirPost, 10, 0, 64, 8, kThr,
                                              kThr, true, 64);
        auto w = UwbCirWriter::make(dir.string(), "cir", true, 64);
        auto tb = gr::make_top_block("bench_e2e");
        tb->msg_connect(res, "packet", est, "rx");
        tb->msg_connect(est, "cir", w, "cir");
        tb->start();

        auto post = [&](uint64_t id) {
            pmt::pmt_t meta = pmt::car(pdu_proto);
            meta = pmt::dict_add(meta, pmt::mp("pulse_id"),
                                 pmt::from_uint64(id));
            meta = pmt::dict_add(meta, pmt::mp("schedule_index"),
                                 pmt::from_uint64(id));
            res->_post(pmt::mp("packet"), pmt::cons(meta, pmt::cdr(pdu_proto)));
        };
        post(0);
        if (!wait_until(
                [&] {
                    return w->frames_written() + w->frames_failed() >= 1 &&
                           est->drained();
                },
                10000))
            throw std::runtime_error(std::string(name) + " warmup timeout");
        est->reset_stats();
        res->reset_stats();

        const size_t depth = 8;
        const uint64_t base =
            w->frames_written() + w->frames_failed();
        const auto t0 = std::chrono::steady_clock::now();
        size_t posted = 0;
        auto done_now = [&]() -> uint64_t {
            return w->frames_written() + w->frames_failed() - base;
        };
        while (done_now() < n) {
            const uint64_t done = done_now();
            while (posted < n && (posted - done) < depth) {
                post(posted + 1);
                ++posted;
            }
            std::this_thread::sleep_for(std::chrono::microseconds(200));
            const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                std::chrono::steady_clock::now() - t0)
                                .count();
            if (ms > 30000) {
                std::ostringstream oss;
                oss << name << " e2e timeout posted=" << posted
                    << " written=" << w->frames_written()
                    << " failed=" << w->frames_failed()
                    << " est_done=" << est->pdus_completed()
                    << " est_fail=" << est->pdus_failed()
                    << " est_drop=" << est->pdus_dropped()
                    << " est_inv=" << est->invalid_inputs()
                    << " res_emitted=" << res->pdus_emitted()
                    << " res_drop=" << res->pdus_dropped();
                throw std::runtime_error(oss.str());
            }
        }
        if (!wait_until(
                [&] {
                    return est->drained() && done_now() >= n;
                },
                30000))
            throw std::runtime_error(std::string(name) + " drain timeout");
        const auto t1 = std::chrono::steady_clock::now();
        tb->stop();
        tb->wait();
        w->stop();

        const double wall_s =
            1e-9 * static_cast<double>(
                       std::chrono::duration_cast<std::chrono::nanoseconds>(
                           t1 - t0)
                           .count());
        const double pps = static_cast<double>(n) / wall_s;
        std::printf("  %-28s  n=%zu  wall=%.3f s  %.1f pulse/s  "
                    "ok=%llu fail=%llu drop_est=%llu drop_res=%llu "
                    "drop_w=%llu wm=%zu  est_mean=%llu us\n",
                    name, n, wall_s, pps,
                    static_cast<unsigned long long>(w->frames_written()),
                    static_cast<unsigned long long>(w->frames_failed()),
                    static_cast<unsigned long long>(est->pdus_dropped()),
                    static_cast<unsigned long long>(res->pdus_dropped()),
                    static_cast<unsigned long long>(w->frames_dropped()),
                    est->queue_high_watermark(),
                    static_cast<unsigned long long>(est->service_mean_us()));
        return pps;
    };

    const double pps48 = run_e2e(
        "UC200_65_48_SC16",
        UwbPduRationalResamplerCcf65_48::make_from_taps(taps48), pdu_uc_sc16,
        n_pdus);
    const double pps32 = run_e2e(
        "CG400_65_32_SC16",
        UwbPduRationalResamplerCcf65_32::make_from_taps(taps32), pdu_cg_sc16,
        n_pdus);
    const double pps48rt = run_e2e(
        "UC200_65_48_SC16_rt",
        UwbPduRationalResamplerCcf65_48::make_from_taps(taps48rt), pdu_uc_sc16,
        n_pdus);
    const double pps32rt = run_e2e(
        "CG400_65_32_SC16_rt",
        UwbPduRationalResamplerCcf65_32::make_from_taps(taps32rt), pdu_cg_sc16,
        n_pdus);

    // ------------------------------------------------------------------
    // Capacity
    // ------------------------------------------------------------------
    std::printf("\n-- capacity (this host, software only) --\n");
    std::printf("  Radio occupancy does not bind below ~%.0f pulse/s "
                "(TX airtime %.1f us).\n",
                uc200.radio_max_pps, uc200.tx_us);
    std::printf("  Resampler handler and CIR worker overlap in steady state;\n"
                "  throughput ≈ 1 / max(T_resample, T_cir, T_write).\n");
    std::printf("  Saturated e2e (quality taps):  UC200 %.0f  CG400 %.0f "
                "pulse/s\n",
                pps48, pps32);
    std::printf("  Saturated e2e (realtime taps): UC200 %.0f  CG400 %.0f "
                "pulse/s\n",
                pps48rt, pps32rt);
    std::printf("  Default target 200 pulse/s (PRI=5 ms) is a QM35-slot "
                "choice, not a host ceiling.\n");
    std::printf("  Hardware UHD send/recv, late_drops and overflow are NOT "
                "in these numbers.\n");
    return 0;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "benchmark_radar_pipeline: %s\n", e.what());
        return 1;
    }
}
