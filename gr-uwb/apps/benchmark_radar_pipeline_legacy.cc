/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * ===========================================================================
 * LEGACY OFFLINE CONTROL -- NOT THE PRODUCTION PATH.
 * ===========================================================================
 *
 * This tool exists for ONE purpose: to keep the pre-refactor sensing
 * decomposition (independent `search_sfd`, `refine_sync_origin` and the
 * scalar `estimate_radar_cir`) measurable as a historical control after
 * `benchmark_radar_pipeline` stopped reporting it.
 *
 * It is NOT a production receiver and every number it prints is a CONTROL
 * number:
 *
 *   - `search_sfd` / `refine_sync_origin` / `estimate_radar_cir` are the
 *     legacy scalar kernels.  The production sensing path no longer calls
 *     them; it runs the shared preamble front-end and the shared SYNC-CIR
 *     kernel inside `radar::radar_cir_one()`.
 *   - The three legacy stages are timed on the RAW window with no CFO
 *     compensation, so their sum is not a valid decomposition of the
 *     production entry and must never be quoted as one.
 *   - `radar_cir_one (production, reference)` is printed only so a reader can
 *     see the ratio against the legacy kernels in the same process.
 *
 * For the production decomposition use `benchmark_radar_pipeline` (or
 * `tools/sic/benchmark_shared_cir`, which also reports the shared front-end
 * split including the final-CFO derotation of the coverage interval).
 *
 * Usage:
 *   benchmark_radar_pipeline_legacy [rounds]
 *     rounds  timed repetitions per control microbench (default 32)
 *
 * Run from the workspace root or gr-uwb/build so testdata/ is visible.
 */

#include <gnuradio/uwb/uwb_radar_cir_core.h>
#include <gnuradio/uwb/uwb_radar_cir_estimator.h>
#include <gnuradio/uwb/uwb_radar_sfd_core.h>
#include <gnuradio/uwb/uwb_radar_timing_core.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iostream>
#include <numeric>
#include <stdexcept>
#include <string>
#include <vector>

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

constexpr size_t kSps = 1016;
constexpr size_t kSyncReps = 64;
constexpr size_t kCirPre = 16;
constexpr size_t kCirPost = 100;
constexpr size_t kCirSkip = 10;
constexpr int64_t kSfdMargin = 64;
constexpr int64_t kSyncMargin = 8;
constexpr float kThr = 0.3f;
constexpr double kPreUs = 2.0;
constexpr size_t kTxWork = 190912;

struct Stats {
    size_t n = 0;
    double mean_us = 0;
    double p50_us = 0;
    double p95_us = 0;
    double max_us = 0;
};

Stats
summarize(std::vector<double> xs)
{
    Stats s;
    if (xs.empty())
        return s;
    std::sort(xs.begin(), xs.end());
    s.n = xs.size();
    s.mean_us = std::accumulate(xs.begin(), xs.end(), 0.0) /
                static_cast<double>(xs.size());
    s.p50_us = xs[static_cast<size_t>(std::llround(0.50 * (xs.size() - 1)))];
    s.p95_us = xs[static_cast<size_t>(std::llround(0.95 * (xs.size() - 1)))];
    s.max_us = xs.back();
    return s;
}

void
print_stats(const char* name, const Stats& s)
{
    std::printf("  %-40s n=%3zu  mean=%8.1f  p50=%8.1f  p95=%8.1f  "
                "max=%8.1f us\n",
                name, s.n, s.mean_us, s.p50_us, s.p95_us, s.max_us);
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

} // namespace

int
main(int argc, char** argv)
{
    try {
        const size_t rounds = (argc > 1) ? std::stoul(argv[1]) : 32;
        const size_t warmup = 4;

        std::printf("== LEGACY OFFLINE CONTROL: pre-refactor sensing "
                    "decomposition ==\n");
        std::printf("!! NOT THE PRODUCTION PATH.  These are the legacy scalar "
                    "kernels on the\n");
        std::printf("!! RAW window (no CFO compensation).  Their sum is not a "
                    "decomposition\n");
        std::printf("!! of the production entry; use benchmark_radar_pipeline "
                    "for that.\n");
        std::printf("host rounds=%zu warmup=%zu\n", rounds, warmup);

        const size_t pre = static_cast<size_t>(std::llround(kPreUs * 1e-6 * 998.4e6));
        const size_t rx_len = pre + kTxWork + 4096;

        std::vector<gr_complex> tx_work;
        if (!load_cf32(testdata_path("uwb_radar/tx_998p4.cf32"), tx_work) ||
            tx_work.size() != kTxWork)
            throw std::runtime_error("tx_998p4.cf32 missing or wrong length");
        const auto rx_work = embed(tx_work, pre, rx_len);
        const int64_t pred_work =
            static_cast<int64_t>(pre) +
            static_cast<int64_t>(kSyncReps) * static_cast<int64_t>(kSps);

        RadarCirCoreScratch scratch;
        if (!prepare_radar_cir_core(cfg(), tx_work.data(), kSps, kCirPre,
                                    kCirPost, scratch))
            throw std::runtime_error("prepare_radar_cir_core failed");
        // Same pre-reservation the production pipeline performs at init, so
        // the reference numbers describe the production shape.
        gr::uwb::radar::reserve_radar_cir_core(rx_work.size(), scratch);

        std::printf("\n-- legacy control kernels (raw window, no CFO) --\n");
        print_stats("search_sfd ±64 (legacy)", time_loop(warmup, rounds, [&] {
                        RadarSfdResult sfd;
                        search_sfd(rx_work.data(), rx_work.size(), pred_work,
                                   kSfdMargin, kThr, sfd, scratch.sfd);
                        if (sfd.status != gr::uwb::radar::SfdStatus::Ok)
                            throw std::runtime_error("SFD failed");
                    }));
        print_stats("refine_sync_origin ±8 (legacy)", time_loop(warmup, rounds, [&] {
                        RadarTimingResult t;
                        refine_sync_origin(rx_work.data(), rx_work.size(),
                                           pred_work, kSyncReps, kSps,
                                           kSyncMargin, kThr,
                                           scratch.sync_template.data(),
                                           scratch.sync_template.size(), t);
                        if (t.status != gr::uwb::radar::TimingStatus::Ok)
                            throw std::runtime_error("timing failed");
                    }));
        print_stats("estimate_radar_cir 54-rep (legacy scalar)",
                    time_loop(warmup, rounds, [&] {
                        RadarCirEstimate cir;
                        if (!estimate_radar_cir(
                                rx_work.data(), rx_work.size(),
                                static_cast<int64_t>(pre), kSps, kCirPre,
                                kCirPost, kCirSkip, kSyncReps - kCirSkip,
                                kSyncReps, cir, scratch.cir) ||
                            cir.status != gr::uwb::radar::CirStatus::Ok)
                            throw std::runtime_error("CIR failed");
                    }));

        std::printf("\n-- production entry, reference only --\n");
        // Warm the front-end scratch first: the first call after the legacy
        // kernels still pays page faults on the shared front-end buffers and
        // would print a cold snapshot.
        {
            RadarCirResult warm;
            (void)radar_cir_one(rx_work.data(), rx_work.size(), pred_work, cfg(),
                                scratch, warm);
        }
        RadarCirResult prod;
        if (!radar_cir_one(rx_work.data(), rx_work.size(), pred_work, cfg(),
                           scratch, prod) ||
            prod.status != RadarCirStatus::Ok)
            throw std::runtime_error("radar_cir_one failed");
        print_stats("radar_cir_one (production, reference)",
                    time_loop(warmup, rounds, [&] {
                        RadarCirResult r;
                        if (!radar_cir_one(rx_work.data(), rx_work.size(),
                                           pred_work, cfg(), scratch, r) ||
                            r.status != RadarCirStatus::Ok)
                            throw std::runtime_error("radar_cir_one failed");
                    }));
        std::printf("  production front-end (timing+cfo+sfd)=%llu us "
                    "[coverage planning %llu us is a child of sfd]\n",
                    static_cast<unsigned long long>(prod.frontend_timing_us +
                                                    prod.frontend_cfo_us +
                                                    prod.frontend_sfd_us),
                    static_cast<unsigned long long>(prod.coverage_us));
        std::printf("  production shared SYNC-CIR kernel (cir_us)=%llu us\n",
                    static_cast<unsigned long long>(prod.cir_us));

        std::printf("\n!! control numbers only: the legacy kernels above are "
                    "not called by the\n");
        std::printf("!! production sensing path.\n");
        return 0;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "benchmark_radar_pipeline_legacy: %s\n", e.what());
        return 1;
    }
}
