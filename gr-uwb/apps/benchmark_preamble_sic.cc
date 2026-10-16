/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * S0-D layered benchmark for the preamble-only SIC branch
 * (docs/sic/OpenCode开发指示_S0_前导碰撞仿真与耗时验证.md §7, §8.4, §9).
 *
 * Three layers, each measured in its OWN table:
 *
 *   (a) core   : direct gr::uwb::sic::sic_one on synthetic main-geometry
 *                windows (sps=1016, 64 SYNC, K=4, fit 8, fractional+CFO).
 *                Reports every SicTimings stage plus the outer wall time.
 *   (b) gr     : the UwbPreambleSic PDU adapter end-to-end (message in/out),
 *                single worker: handler + queue wait + core + PMT wrap/publish
 *                + completion latency.
 *   (c) native : 491.52 MS/s native window -> existing PDU 65/32 resampler ->
 *                SIC on the work grid, using the production resampler block.
 *
 * Rules encoded here (task §8.4 / §9):
 *   - Release, single worker, serial (one window in flight at a time).
 *   - 200-window warmup, excluded from every statistic.
 *   - >= windows x rounds iterations for the rotating distinct-content set.
 *   - BOTH a hot-cache single window and a >=64 distinct-content set.
 *   - Scratch reused; the pure core never allocates per window.
 *   - Every result is CONSUMED (checksum + counters) so the calls cannot be
 *     optimised away.
 *   - mean / P50 / P95 / P99 / max per stage, CPU seconds, RSS, queue and
 *     in-flight high watermarks, deadline misses.
 *   - A round longer than 60 s prints batch progress with running stats.
 *   - CSV + JSON are written under --output (a private build/tmp path); the
 *     tool refuses to write inside a git-tracked source directory.
 *
 * Usage:
 *   benchmark_preamble_sic --preset main --mode core|gr|native
 *       --consumer demod|cir --windows 10000 --rounds 3 --output DIR
 *   optional: --distinct N   (distinct-content window count, default 64)
 *             --hot-windows N (hot single-window iterations, default windows)
 *
 * This program contains NO SIC algorithm: it only drives the frozen core /
 * adapter / resampler contracts and measures them.
 */

#include <gnuradio/block.h>
#include <gnuradio/io_signature.h>
#include <gnuradio/top_block.h>
#include <gnuradio/uwb/uwb_pdu_rational_resampler_ccf_65_32.h>
#include <gnuradio/uwb/uwb_preamble_sic.h>
#include <gnuradio/uwb/uwb_preamble_sic_core.h>
#include <pmt/pmt.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <numeric>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <sys/resource.h>
#include <thread>
#include <vector>

using cplx = std::complex<float>;
namespace sic = gr::uwb::sic;
using gr::uwb::UwbPduRationalResamplerCcf65_32;
using gr::uwb::UwbPreambleSic;

namespace {

// ---------------------------------------------------------------------------
// Main geometry (frozen by the S0 task / cases/main.json)
// ---------------------------------------------------------------------------
constexpr double kPi = 3.14159265358979323846;
constexpr size_t kSps = 1016;
constexpr size_t kSyncReps = 64;
constexpr size_t kPre = 2000;
constexpr double kWorkRate = 998.4e6;
constexpr double kNativeRate = 491.52e6;
constexpr size_t kSfdSyms = 8;
constexpr size_t kRangeGuard = 100;
constexpr size_t kTailSamples = 4094;
constexpr size_t kCirPad = 2000;

constexpr int kNStage = 11;
const char* const kStageNames[kNStage] = {
    "detect_coarse",   "detect_fine",       "cfo",
    "cir",             "path_select",       "fractional_refine",
    "matrix_build",    "gain_fit",          "reconstruct",
    "validate_commit", "core_total"
};

struct Path {
    double tau;
    cplx g;
};
const Path kCommPaths[4] = {
    { 0.0, cplx(1.0f, 0.0f) },
    { 7.4, cplx(0.0f, 0.5f) },
    { 23.0, cplx(0.276f, -0.116f) },
    { 51.2, cplx(0.2f, 0.0f) },
};

constexpr double kDetectThreshold = 0.5;
constexpr double kValidateThreshold = 0.25;

// ---------------------------------------------------------------------------
// CLI
// ---------------------------------------------------------------------------
struct Options {
    std::string preset = "main";
    std::string mode = "core";
    std::string consumer = "cir";
    size_t windows = 10000;
    size_t rounds = 3;
    size_t hot_windows = 0;
    size_t distinct = 64;
    std::string output;
};

void
usage(const char* argv0)
{
    std::printf(
        "usage: %s --preset main --mode core|gr|native --consumer demod|cir\n"
        "          --windows N --rounds R --output DIR [--distinct N]\n"
        "          [--hot-windows N]\n",
        argv0);
}

bool
parse_args(int argc, char** argv, Options& o)
{
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        auto need = [&](const char* name) -> std::string {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "error: %s requires a value\n", name);
                std::exit(2);
            }
            return argv[++i];
        };
        if (a == "--preset")
            o.preset = need("--preset");
        else if (a == "--mode")
            o.mode = need("--mode");
        else if (a == "--consumer")
            o.consumer = need("--consumer");
        else if (a == "--windows")
            o.windows = std::stoul(need("--windows"));
        else if (a == "--rounds")
            o.rounds = std::stoul(need("--rounds"));
        else if (a == "--hot-windows")
            o.hot_windows = std::stoul(need("--hot-windows"));
        else if (a == "--distinct")
            o.distinct = std::stoul(need("--distinct"));
        else if (a == "--output")
            o.output = need("--output");
        else if (a == "-h" || a == "--help") {
            usage(argv[0]);
            std::exit(0);
        } else {
            std::fprintf(stderr, "error: unknown argument '%s'\n", a.c_str());
            usage(argv[0]);
            std::exit(2);
        }
    }
    if (o.preset != "main") {
        std::fprintf(stderr, "error: unknown --preset '%s' (only 'main')\n",
                     o.preset.c_str());
        return false;
    }
    if (o.mode != "core" && o.mode != "gr" && o.mode != "native") {
        std::fprintf(stderr, "error: unknown --mode '%s' (core|gr|native)\n",
                     o.mode.c_str());
        return false;
    }
    if (o.consumer != "demod" && o.consumer != "cir") {
        std::fprintf(stderr, "error: unknown --consumer '%s' (demod|cir)\n",
                     o.consumer.c_str());
        return false;
    }
    if (o.windows == 0 || o.rounds == 0 || o.distinct == 0) {
        std::fprintf(stderr,
                     "error: --windows/--rounds/--distinct must be > 0\n");
        return false;
    }
    // The hot-cache single-window set is a cache probe, not the rotating
    // distinct-content workload, so it is bounded by default (>=64 windows is
    // the only hard floor required by the task).
    if (o.hot_windows == 0)
        o.hot_windows = std::min<size_t>(o.windows, 2000);
    if (o.output.empty()) {
        std::fprintf(stderr, "error: --output DIR is required\n");
        return false;
    }
    return true;
}

// ---------------------------------------------------------------------------
// Stats / process metrics
// ---------------------------------------------------------------------------
struct Stats {
    size_t n = 0;
    double mean_us = 0, p50_us = 0, p95_us = 0, p99_us = 0, max_us = 0;
    double min_us = 0;
};

Stats
summarize(std::vector<double>& xs)
{
    Stats s;
    if (xs.empty())
        return s;
    std::sort(xs.begin(), xs.end());
    s.n = xs.size();
    s.min_us = xs.front();
    s.max_us = xs.back();
    s.mean_us =
        std::accumulate(xs.begin(), xs.end(), 0.0) / static_cast<double>(xs.size());
    auto at = [&](double p) {
        const double idx = p * static_cast<double>(xs.size() - 1);
        return xs[static_cast<size_t>(std::llround(idx))];
    };
    s.p50_us = at(0.50);
    s.p95_us = at(0.95);
    s.p99_us = at(0.99);
    return s;
}

double
rss_mb()
{
    struct rusage ru;
    if (::getrusage(RUSAGE_SELF, &ru) != 0)
        return 0.0;
    return static_cast<double>(ru.ru_maxrss) / 1024.0;
}

double
cpu_seconds()
{
    struct rusage ru;
    if (::getrusage(RUSAGE_SELF, &ru) != 0)
        return 0.0;
    auto tv = [](const struct timeval& t) {
        return static_cast<double>(t.tv_sec) + 1e-6 * static_cast<double>(t.tv_usec);
    };
    return tv(ru.ru_utime) + tv(ru.ru_stime);
}

double
elapsed_us(std::chrono::steady_clock::time_point a,
           std::chrono::steady_clock::time_point b)
{
    return 1e-3 * static_cast<double>(
                      std::chrono::duration_cast<std::chrono::nanoseconds>(b - a)
                          .count());
}

size_t
work_window_len(const Options& o)
{
    return (o.consumer == "demod")
        ? kPre + kSyncReps * kSps + kSfdSyms * kSps + kRangeGuard + kTailSamples
        : kPre + kSyncReps * kSps + kCirPad;
}

// ---------------------------------------------------------------------------
// Synthetic main-geometry model (independent comm code; never the sensing code)
// ---------------------------------------------------------------------------
double
sinc_pi(double t)
{
    if (std::abs(t) < 1e-12)
        return 1.0;
    const double p = kPi * t;
    return std::sin(p) / p;
}

cplx
gen_interp(const std::vector<cplx>& p, double x)
{
    constexpr int H = 24;
    const int64_t i0 = static_cast<int64_t>(std::floor(x));
    const double frac = x - std::floor(x);
    double ar = 0.0, ai = 0.0, wt = 0.0;
    for (int j = -H; j <= H; ++j) {
        const int64_t idx = i0 + j;
        if (idx < 0 || idx >= static_cast<int64_t>(p.size()))
            continue;
        const double t = static_cast<double>(j) - frac;
        const double w =
            0.42 + 0.5 * std::cos(kPi * t / H) + 0.08 * std::cos(2.0 * kPi * t / H);
        const double tap = sinc_pi(t) * w;
        ar += tap * p[static_cast<size_t>(idx)].real();
        ai += tap * p[static_cast<size_t>(idx)].imag();
        wt += tap;
    }
    if (std::abs(wt) > 1e-9) {
        ar /= wt;
        ai /= wt;
    }
    return cplx(static_cast<float>(ar), static_cast<float>(ai));
}

struct Model {
    std::vector<cplx> preamble;
    std::vector<cplx> sync;
};

Model
build_model(size_t code, size_t reps)
{
    Model m;
    const size_t cps = kSps / 2;
    const size_t T = 9;
    std::vector<double> pulse(T);
    double ps = 0.0;
    for (size_t i = 0; i < T; ++i) {
        const double d = static_cast<double>(i) - 4.0;
        pulse[i] = std::exp(-(d * d) / (2.0 * 1.2 * 1.2));
        ps += pulse[i];
    }
    for (double& v : pulse)
        v /= ps;
    uint64_t s =
        0x9E3779B97F4A7C15ull ^ (static_cast<uint64_t>(code) * 2654435761ull);
    std::vector<int> chips(cps);
    for (size_t i = 0; i < cps; ++i) {
        s ^= s << 13;
        s ^= s >> 7;
        s ^= s << 17;
        chips[i] = (s & 1u) ? 1 : -1;
    }
    const size_t total = reps * cps;
    std::vector<double> up(total * 2, 0.0);
    for (size_t k = 0; k < total; ++k)
        up[2 * k] = static_cast<double>(chips[k % cps]);
    const size_t plen = up.size() + T - 1;
    m.preamble.assign(plen, cplx(0.f, 0.f));
    for (size_t i = 0; i < up.size(); ++i) {
        if (up[i] == 0.0)
            continue;
        for (size_t j = 0; j < T; ++j)
            m.preamble[i + j] += cplx(static_cast<float>(up[i] * pulse[j]), 0.f);
    }
    m.sync.assign(m.preamble.begin(), m.preamble.begin() + kSps);
    return m;
}

void
add_path(std::vector<cplx>& acc, const std::vector<cplx>& pre, int64_t base,
         double tau, cplx g, double cfo_hz)
{
    const int64_t span0 = base + static_cast<int64_t>(std::floor(tau)) - 32;
    const int64_t span1 = base + static_cast<int64_t>(std::ceil(tau)) +
                          static_cast<int64_t>(pre.size()) + 32;
    const int64_t lo = std::max<int64_t>(0, span0);
    const int64_t hi = std::min<int64_t>(static_cast<int64_t>(acc.size()), span1);
    const double w = 2.0 * kPi * cfo_hz / kWorkRate;
    for (int64_t i = lo; i < hi; ++i) {
        const cplx v =
            gen_interp(pre, static_cast<double>(i) - static_cast<double>(base) - tau);
        const double ph = w * static_cast<double>(i);
        acc[static_cast<size_t>(i)] +=
            g * v * cplx(static_cast<float>(std::cos(ph)),
                         static_cast<float>(std::sin(ph)));
    }
}

std::vector<cplx>
build_window(const Model& m, size_t n, int64_t comm_start, double noise_std,
             uint64_t seed)
{
    std::vector<cplx> i(n, cplx(0.f, 0.f));
    std::mt19937_64 rng(seed);
    std::uniform_real_distribution<double> ph(0.0, 2.0 * kPi);
    for (const Path& p : kCommPaths) {
        const double extra = ph(rng);
        const cplx g = p.g * cplx(static_cast<float>(std::cos(extra)),
                                  static_cast<float>(std::sin(extra)));
        add_path(i, m.preamble, comm_start, p.tau, g, 0.0);
    }
    std::normal_distribution<double> nd(0.0, noise_std);
    std::vector<cplx> y(n);
    for (size_t k = 0; k < n; ++k)
        y[k] = i[k] +
               cplx(static_cast<float>(nd(rng)), static_cast<float>(nd(rng)));
    return y;
}

std::vector<cplx>
decimate_to_native(const std::vector<cplx>& work)
{
    const size_t n_native = static_cast<size_t>(
                                std::ceil(static_cast<double>(work.size()) *
                                          32.0 / 65.0)) +
                            8;
    std::vector<cplx> out(n_native, cplx(0.f, 0.f));
    for (size_t m = 0; m < n_native; ++m) {
        const double x = static_cast<double>(m) * 65.0 / 32.0;
        if (x >= static_cast<double>(work.size()))
            break;
        out[m] = gen_interp(work, x);
    }
    return out;
}

// ---------------------------------------------------------------------------
// Consume-only sink
// ---------------------------------------------------------------------------
uint64_t
dict_u64(pmt::pmt_t dict, const char* key, uint64_t def)
{
    if (!pmt::is_dict(dict))
        return def;
    pmt::pmt_t v = pmt::dict_ref(dict, pmt::mp(key), pmt::from_uint64(def));
    if (pmt::is_uint64(v))
        return pmt::to_uint64(v);
    if (pmt::is_integer(v))
        return static_cast<uint64_t>(pmt::to_long(v));
    return def;
}

bool
dict_bool(pmt::pmt_t dict, const char* key, bool def)
{
    if (!pmt::is_dict(dict))
        return def;
    pmt::pmt_t v = pmt::dict_ref(dict, pmt::mp(key), pmt::from_bool(def));
    if (pmt::is_bool(v))
        return pmt::to_bool(v);
    return def;
}

class BenchSink : public gr::block
{
public:
    using sptr = std::shared_ptr<BenchSink>;
    static sptr make() { return gnuradio::get_initial_sptr(new BenchSink()); }

    std::atomic<uint64_t> count{ 0 };
    std::atomic<uint64_t> committed{ 0 };
    double checksum = 0.0;
    uint64_t sum_core_us = 0, sum_handler_us = 0, sum_queue_wait_us = 0;
    uint64_t sum_wrap_us = 0, sum_completion_us = 0, sum_resample_us = 0;

    // Called only while the worker is idle (after the warmup drain).
    void reset_stats()
    {
        count.store(0);
        committed.store(0);
        checksum = 0.0;
        sum_core_us = sum_handler_us = sum_queue_wait_us = 0;
        sum_wrap_us = sum_completion_us = sum_resample_us = 0;
    }

    ~BenchSink() override = default;

private:
    BenchSink()
        : gr::block("bench_preamble_sic_sink",
                    gr::io_signature::make(0, 0, 0),
                    gr::io_signature::make(0, 0, 0))
    {
        message_port_register_in(pmt::mp("in"));
        set_msg_handler(pmt::mp("in"), [this](pmt::pmt_t m) { handle(m); });
    }

    void handle(pmt::pmt_t m)
    {
        if (!pmt::is_pair(m))
            return;
        pmt::pmt_t meta = pmt::car(m);
        size_t n = 0;
        const gr_complex* v = pmt::c32vector_elements(pmt::cdr(m), n);
        if (v != nullptr && n > 0) {
            checksum += static_cast<double>(std::abs(v[0])) +
                        static_cast<double>(std::abs(v[n / 2])) +
                        static_cast<double>(std::abs(v[n - 1]));
        }
        if (pmt::is_dict(meta)) {
            sum_core_us += dict_u64(meta, "core_total_us", 0);
            sum_handler_us += dict_u64(meta, "handler_us", 0);
            sum_queue_wait_us += dict_u64(meta, "queue_wait_us", 0);
            sum_wrap_us += dict_u64(meta, "output_wrap_publish_us", 0);
            sum_completion_us += dict_u64(meta, "completion_latency_us", 0);
            sum_resample_us += dict_u64(meta, "resample_us", 0);
            if (dict_bool(meta, "sic_committed", false))
                committed.fetch_add(1, std::memory_order_relaxed);
        }
        count.fetch_add(1, std::memory_order_relaxed);
    }
};

// ---------------------------------------------------------------------------
// testdata taps + output guard
// ---------------------------------------------------------------------------
std::string
find_testdata(const std::string& rel)
{
    const char* env = std::getenv("UWB_TESTDATA_DIR");
    if (env && env[0]) {
        const std::string p = std::string(env) + "/" + rel;
        if (std::ifstream(p, std::ios::binary))
            return p;
    }
    const char* prefixes[] = {
        "testdata/",         "../testdata/",         "../../testdata/",
        "../../../testdata/", "../../../../testdata/",
        "/home/junqima/workspace/uwb-gnuradio/testdata/",
    };
    for (const char* p : prefixes) {
        const std::string path = std::string(p) + rel;
        if (std::ifstream(path, std::ios::binary))
            return path;
    }
    return std::string();
}

std::vector<float>
load_taps(const std::string& rel)
{
    const std::string path = find_testdata(rel);
    if (path.empty())
        throw std::runtime_error("cannot find testdata/" + rel);
    std::ifstream f(path, std::ios::binary | std::ios::ate);
    const auto bytes = static_cast<size_t>(f.tellg());
    if (!f || bytes == 0 || bytes % sizeof(float) != 0)
        throw std::runtime_error("bad taps file " + path);
    f.seekg(0);
    std::vector<float> v(bytes / sizeof(float));
    f.read(reinterpret_cast<char*>(v.data()), static_cast<std::streamsize>(bytes));
    return v;
}

void
ensure_private_output(const std::string& dir)
{
    namespace fs = std::filesystem;
    const fs::path p = fs::absolute(dir).lexically_normal();
    fs::path root = p;
    bool is_git = false;
    while (root.has_parent_path() && root != root.root_path()) {
        if (fs::exists(root / ".git")) {
            is_git = true;
            break;
        }
        root = root.parent_path();
    }
    if (is_git) {
        bool buildish = false;
        for (const auto& c : p) {
            const std::string s = c.string();
            if (s.rfind("build", 0) == 0 || s == "tmp")
                buildish = true;
        }
        if (!buildish)
            throw std::runtime_error("--output is inside the git repo but not "
                                     "under a build*/tmp directory: " +
                                     p.string());
    }
    fs::create_directories(p);
}

// ---------------------------------------------------------------------------
// Per-run collector + stats printing
// ---------------------------------------------------------------------------
struct LayerRun {
    std::vector<double> wall;
    std::array<std::vector<double>, kNStage> stage;
    uint64_t miss_1ms = 0, miss_500us = 0;
};

void
print_row(const char* label, const Stats& s)
{
    std::printf("  %-22s n=%-6zu mean=%9.1f p50=%9.1f p95=%9.1f p99=%9.1f "
                "max=%9.1f us\n",
                label, s.n, s.mean_us, s.p50_us, s.p95_us, s.p99_us, s.max_us);
}

void
stats_obj(std::ostringstream& j, const Stats& s)
{
    j << "{\"n\":" << s.n << ",\"mean_us\":" << s.mean_us
      << ",\"p50_us\":" << s.p50_us << ",\"p95_us\":" << s.p95_us
      << ",\"p99_us\":" << s.p99_us << ",\"max_us\":" << s.max_us
      << ",\"min_us\":" << s.min_us << "}";
}

void
stats_json(std::ostringstream& j, const char* key, const Stats& s)
{
    j << "\"" << key << "\":";
    stats_obj(j, s);
}

// ---------------------------------------------------------------------------
// Layer (a): core
// ---------------------------------------------------------------------------
void
run_core(const Options& o, const Model& m,
         const std::vector<std::vector<cplx>>& wins, std::ofstream& csv,
         std::ostringstream& json)
{
    const size_t n = work_window_len(o);

    sic::SicConfig cfg;
    cfg.mode = sic::SicMode::Estimate;
    cfg.comm_code = 10;
    cfg.sync_reps = kSyncReps;
    cfg.samples_per_symbol = kSps;
    cfg.sample_rate = kWorkRate;
    cfg.k_paths = 4;
    cfg.fit_reps = 8;
    cfg.validation_reps = 8;
    cfg.roi_lo = static_cast<int64_t>(kPre);
    cfg.roi_hi = static_cast<int64_t>(kPre + kSyncReps * kSps);
    cfg.coarse_stride = 14;
    cfg.post_tail = 100;
    cfg.enable_fractional = true;
    cfg.enable_cfo = true;
    cfg.detect_threshold = kDetectThreshold;
    cfg.validate_threshold = kValidateThreshold;

    sic::SicScratch scratch;
    std::string err;
    if (!sic::prepare_sic_core(cfg, m.sync.data(), m.sync.size(),
                               m.preamble.data(), m.preamble.size(), scratch,
                               &err))
        throw std::runtime_error("prepare_sic_core: " + err);

    sic::SicTemplates tpl;
    tpl.sync = m.sync.data();
    tpl.sync_len = m.sync.size();
    tpl.preamble = m.preamble.data();
    tpl.preamble_len = m.preamble.size();

    std::vector<cplx> out(n, cplx(0.f, 0.f));
    std::vector<cplx> ihat(n, cplx(0.f, 0.f));
    sic::SicResult r;
    r.out = out.data();
    r.i_hat = ihat.data();
    auto one = [&](const std::vector<cplx>& y) {
        return sic::sic_one(y.data(), n, tpl, cfg, scratch, r, nullptr);
    };

    std::printf("\n== layer (a) core ==\n");
    std::printf("  sps=%zu sync_reps=%zu n=%zu fit=8 val=8 K=4 fractional=on "
                "cfo=on roi=[%zu,%zu)\n",
                kSps, kSyncReps, n, kPre, kPre + kSyncReps * kSps);
    std::printf("  scratch peak_bytes=%zu window=%zu B distinct=%zu\n",
                scratch.peak_bytes, n * sizeof(cplx), wins.size());

    for (size_t w = 0; w < 200; ++w)
        if (!one(wins[w % wins.size()]))
            throw std::runtime_error("warmup sic_one returned false");

    double cks = 0.0;
    json << "\"core\":{\n";
    const char* case_names[2] = { "distinct", "hot" };
    for (int c = 0; c < 2; ++c) {
        const bool hot = (c == 1);
        const size_t nwin = hot ? o.hot_windows : o.windows;
        std::printf("\n  -- %s set (%s window%s) --\n", case_names[c],
                    hot ? "single hot" : "rotating distinct",
                    hot ? "" : "s");
        std::vector<double> agg_wall;
        std::array<std::vector<double>, kNStage> agg_stage;
        uint64_t miss1 = 0, miss500 = 0;
        for (size_t rd = 0; rd < o.rounds; ++rd) {
            // Rotate the sub-case visit order each round and the window offset.
            const size_t win_off = ((rd + (hot ? 1 : 0)) * 17) % wins.size();
            const auto round_t0 = std::chrono::steady_clock::now();
            for (size_t k = 0; k < nwin; ++k) {
                const size_t widx =
                    hot ? 0 : static_cast<size_t>((k + win_off) % wins.size());
                const std::vector<cplx>& y = wins[widx];
                const auto t0 = std::chrono::steady_clock::now();
                if (!one(y))
                    throw std::runtime_error("sic_one returned false");
                const auto t1 = std::chrono::steady_clock::now();
                const double wus = elapsed_us(t0, t1);
                agg_wall.push_back(wus);
                if (wus > 1000.0)
                    ++miss1;
                if (wus > 500.0)
                    ++miss500;
                const sic::SicTimings& tt = r.timings;
                agg_stage[0].push_back(tt.detect_coarse_us);
                agg_stage[1].push_back(tt.detect_fine_us);
                agg_stage[2].push_back(tt.cfo_us);
                agg_stage[3].push_back(tt.cir_us);
                agg_stage[4].push_back(tt.path_select_us);
                agg_stage[5].push_back(tt.fractional_refine_us);
                agg_stage[6].push_back(tt.matrix_build_us);
                agg_stage[7].push_back(tt.gain_fit_us);
                agg_stage[8].push_back(tt.reconstruct_us);
                agg_stage[9].push_back(tt.validate_commit_us);
                agg_stage[10].push_back(tt.core_total_us);
                cks += static_cast<double>(std::abs(out[(k * 97) % n])) +
                       static_cast<double>(std::abs(ihat[(k * 89) % n]));
                csv << "core," << o.consumer << ',' << rd << ','
                    << case_names[c] << ',' << widx << ',' << wus << ','
                    << tt.core_total_us << ',' << tt.detect_coarse_us << ','
                    << tt.fractional_refine_us << ',' << tt.reconstruct_us << ','
                    << tt.detect_fine_us << '\n';
                if ((k % 1000) == 0 && k > 0 &&
                    elapsed_us(round_t0, std::chrono::steady_clock::now()) > 60e6) {
                    const double m =
                        std::accumulate(agg_wall.begin(), agg_wall.end(), 0.0) /
                        static_cast<double>(agg_wall.size());
                    std::printf("      [round %zu] batch %zu/%zu running wall "
                                "mean=%.3f ms\n",
                                rd, k, nwin, m / 1000.0);
                    std::fflush(stdout);
                }
            }
        }
        Stats wall = summarize(agg_wall);
        print_row("wall (per sic_one)", wall);
        for (int s = 0; s < kNStage; ++s) {
            Stats st = summarize(agg_stage[s]);
            print_row(kStageNames[s], st);
        }
        const double wps = wall.mean_us > 0 ? 1e6 / wall.mean_us : 0;
        std::printf("    windows/round=%zu rounds=%zu total=%zu  %.1f win/s  "
                    "deadline misses: >1ms=%llu  >0.5ms=%llu\n",
                    nwin, o.rounds, wall.n, wps,
                    static_cast<unsigned long long>(miss1),
                    static_cast<unsigned long long>(miss500));

        std::ostringstream entry;
        entry << "\"" << case_names[c] << "\":{";
        bool first = true;
        auto add = [&](const char* key, const Stats& s) {
            if (!first)
                entry << ",";
            first = false;
            stats_json(entry, key, s);
        };
        add("wall", wall);
        for (int s = 0; s < kNStage; ++s) {
            Stats st = summarize(agg_stage[s]);
            add(kStageNames[s], st);
        }
        entry << ",\"windows_per_round\":" << nwin << ",\"rounds\":" << o.rounds
              << ",\"deadline_miss_1ms\":" << miss1
              << ",\"deadline_miss_500us\":" << miss500 << ",\"win_per_s\":" << wps
              << "}";
        json << (c == 1 ? ",\n" : "") << entry.str();
        if (c == 1)
            json << "\n";
    }
    json << "}";
    std::printf("  consume checksum=%.6f (out/i_hat read so nothing is dead)\n",
                cks);
}

// ---------------------------------------------------------------------------
// SIC adapter factory + metadata
// ---------------------------------------------------------------------------
UwbPreambleSic::sptr
make_sic(const Model& m, int64_t roi_lo, int64_t roi_hi, size_t max_window)
{
    return UwbPreambleSic::make_from_templates(
        m.sync, m.preamble, 10, kSyncReps, kSps, kWorkRate, 4, 8, 8, roi_lo,
        roi_hi, 14, "estimate", kDetectThreshold, kValidateThreshold,
        /*in_flight_max=*/8, /*iq_budget_bytes=*/size_t(64) << 20,
        /*queue_capacity=*/8, /*publish_reconstruction=*/false,
        /*enable_fractional=*/true, /*enable_cfo=*/true,
        /*path_min_spacing=*/0, /*post_tail=*/100,
        /*max_window_samples=*/max_window, /*pending_bypass_max=*/0);
}

pmt::pmt_t
work_meta(uint64_t id, int64_t n)
{
    pmt::pmt_t d = pmt::make_dict();
    d = pmt::dict_add(d, pmt::mp("window_id"), pmt::from_uint64(id));
    d = pmt::dict_add(d, pmt::mp("run_id"), pmt::from_uint64(1));
    d = pmt::dict_add(d, pmt::mp("sample_rate"), pmt::from_double(kWorkRate));
    d = pmt::dict_add(d, pmt::mp("sample_count"), pmt::from_long(n));
    d = pmt::dict_add(d, pmt::mp("sensing_code"), pmt::from_long(9));
    d = pmt::dict_add(d, pmt::mp("sensing_sync_reps"), pmt::from_long(64));
    d = pmt::dict_add(d, pmt::mp("sensing_sfd_mode"), pmt::mp("4z2"));
    return d;
}

pmt::pmt_t
native_meta(uint64_t id, int64_t n)
{
    pmt::pmt_t d = pmt::make_dict();
    d = pmt::dict_add(d, pmt::mp("window_id"), pmt::from_uint64(id));
    d = pmt::dict_add(d, pmt::mp("packet_id"), pmt::from_uint64(id));
    d = pmt::dict_add(d, pmt::mp("schedule_index"), pmt::from_uint64(id));
    d = pmt::dict_add(d, pmt::mp("sample_rate"), pmt::from_double(kNativeRate));
    d = pmt::dict_add(d, pmt::mp("window_start_sample"), pmt::from_long(0));
    d = pmt::dict_add(d, pmt::mp("pre_guard_samples"), pmt::from_long(0));
    d = pmt::dict_add(d, pmt::mp("capture_samples"), pmt::from_long(n));
    d = pmt::dict_add(d, pmt::mp("sample_count"), pmt::from_long(n));
    d = pmt::dict_add(d, pmt::mp("calibration_delay_native_samples"),
                      pmt::from_double(0.0));
    d = pmt::dict_add(d, pmt::mp("sensing_code"), pmt::from_long(9));
    return d;
}

// ---------------------------------------------------------------------------
// Layer (b): GR/PDU adapter end-to-end
// ---------------------------------------------------------------------------
struct GrResult {
    Stats wall;
    double mean_core_us = 0, mean_handler_us = 0, mean_queue_wait_us = 0;
    double mean_wrap_us = 0, mean_completion_us = 0, mean_resample_us = 0;
    uint64_t completions = 0, committed = 0, miss1 = 0, miss500 = 0;
    size_t inflight_hwm = 0, queue_hwm = 0;
    double checksum = 0.0;
};

template <typename PostFn>
GrResult
run_serial_chain(const Options& o, UwbPreambleSic::sptr blk, PostFn post,
                 size_t n_windows, std::ofstream& csv, const char* layer_tag)
{
    auto sink = BenchSink::make();
    auto tb = gr::make_top_block("bench_preamble_sic");
    tb->msg_connect(blk, "out", sink, "in");
    tb->start();

    // Warmup 200 (excluded).
    for (size_t w = 0; w < 200; ++w) {
        const uint64_t before = sink->count.load();
        post(1000000 + w, w % n_windows);
        const auto t0 = std::chrono::steady_clock::now();
        while (sink->count.load() < before + 1) {
            std::this_thread::sleep_for(std::chrono::microseconds(50));
            if (elapsed_us(t0, std::chrono::steady_clock::now()) > 30e6)
                throw std::runtime_error("warmup timeout");
        }
    }
    sink->reset_stats();

    GrResult res;
    std::vector<double> wall;
    for (size_t rd = 0; rd < o.rounds; ++rd) {
        const size_t win_off = (rd * 23) % n_windows;
        const auto round_t0 = std::chrono::steady_clock::now();
        for (size_t k = 0; k < o.windows; ++k) {
            const uint64_t before = sink->count.load();
            const uint64_t id = (static_cast<uint64_t>(rd) << 40) + k + 1;
            const size_t widx = (k + win_off) % n_windows;
            const auto t0 = std::chrono::steady_clock::now();
            post(id, widx);
            const auto spin0 = std::chrono::steady_clock::now();
            while (sink->count.load() < before + 1) {
                std::this_thread::sleep_for(std::chrono::microseconds(50));
                if (elapsed_us(spin0, std::chrono::steady_clock::now()) > 60e6)
                    throw std::runtime_error("sink timeout");
            }
            const double wus = elapsed_us(t0, std::chrono::steady_clock::now());
            wall.push_back(wus);
            if (wus > 1000.0)
                ++res.miss1;
            if (wus > 500.0)
                ++res.miss500;
            if ((k % 1000) == 0 && k > 0 &&
                elapsed_us(round_t0, std::chrono::steady_clock::now()) > 60e6) {
                const double m =
                    std::accumulate(wall.begin(), wall.end(), 0.0) /
                    static_cast<double>(wall.size());
                std::printf("      [round %zu] batch %zu/%zu running wall mean="
                            "%.3f ms\n",
                            rd, k, o.windows, m / 1000.0);
                std::fflush(stdout);
            }
        }
    }
    tb->stop();
    tb->wait();

    res.wall = summarize(wall);
    const uint64_t n = sink->count.load();
    res.completions = n;
    res.committed = sink->committed.load();
    res.checksum = sink->checksum;
    auto mean_of = [&](uint64_t sum) {
        return n ? static_cast<double>(sum) / static_cast<double>(n) : 0.0;
    };
    res.mean_core_us = mean_of(sink->sum_core_us);
    res.mean_handler_us = mean_of(sink->sum_handler_us);
    res.mean_queue_wait_us = mean_of(sink->sum_queue_wait_us);
    res.mean_wrap_us = mean_of(sink->sum_wrap_us);
    res.mean_completion_us = mean_of(sink->sum_completion_us);
    res.mean_resample_us = mean_of(sink->sum_resample_us);
    res.inflight_hwm = blk->inflight_high_watermark();
    res.queue_hwm = blk->queue_high_watermark();
    for (size_t i = 0; i < wall.size(); ++i)
        csv << layer_tag << ',' << o.consumer << ',' << (i / o.windows) << ','
            << "rotating" << ',' << (i % n_windows) << ',' << wall[i] << ','
            << "0" << ',' << "0" << '\n';
    return res;
}

void
run_gr(const Options& o, const Model& m,
       const std::vector<std::vector<cplx>>& wins, std::ofstream& csv,
       std::ostringstream& json)
{
    const size_t n = work_window_len(o);
    auto blk = make_sic(m, static_cast<int64_t>(kPre),
                        static_cast<int64_t>(kPre + kSyncReps * kSps), n);
    std::printf("\n== layer (b) GR/PDU (UwbPreambleSic adapter, single worker) "
                "==\n");
    std::printf("  n=%zu windows=%zu rounds=%zu in_flight_max=%zu "
                "queue_capacity=%zu\n",
                n, o.windows, o.rounds, blk->in_flight_max(), blk->queue_capacity());

    auto post = [&](uint64_t id, size_t widx) {
        const auto& y = wins[widx];
        pmt::pmt_t pdu = pmt::cons(work_meta(id, static_cast<int64_t>(n)),
                                   pmt::init_c32vector(y.size(), y.data()));
        blk->_post(pmt::mp("in"), pdu);
    };
    GrResult res = run_serial_chain(o, blk, post, wins.size(), csv, "gr");
    print_row("wall (post->out)", res.wall);
    std::printf("  adapter means: handler=%.1f queue_wait=%.1f core=%.1f "
                "wrap_publish=%.1f completion=%.1f us\n",
                res.mean_handler_us, res.mean_queue_wait_us, res.mean_core_us,
                res.mean_wrap_us, res.mean_completion_us);
    const double wps = res.wall.mean_us > 0 ? 1e6 / res.wall.mean_us : 0;
    std::printf("  completions=%llu committed=%llu inflight_hwm=%zu queue_hwm=%zu "
                "misses>1ms=%llu >0.5ms=%llu checksum=%.6f  %.1f win/s\n",
                static_cast<unsigned long long>(res.completions),
                static_cast<unsigned long long>(res.committed), res.inflight_hwm,
                res.queue_hwm, static_cast<unsigned long long>(res.miss1),
                static_cast<unsigned long long>(res.miss500), res.checksum, wps);

    json << "\"gr\":{\"n\":" << n << ",\"windows\":" << o.windows
         << ",\"rounds\":" << o.rounds << ",\"wall\":";
    stats_obj(json, res.wall);
    json << ",\"mean_handler_us\":" << res.mean_handler_us
         << ",\"mean_queue_wait_us\":" << res.mean_queue_wait_us
         << ",\"mean_core_us\":" << res.mean_core_us
         << ",\"mean_wrap_us\":" << res.mean_wrap_us
         << ",\"mean_completion_us\":" << res.mean_completion_us
         << ",\"completions\":" << res.completions
         << ",\"committed\":" << res.committed
         << ",\"inflight_hwm\":" << res.inflight_hwm
         << ",\"queue_hwm\":" << res.queue_hwm
         << ",\"deadline_miss_1ms\":" << res.miss1
         << ",\"deadline_miss_500us\":" << res.miss500
         << ",\"win_per_s\":" << wps << ",\"checksum\":" << res.checksum << "}";
}

// ---------------------------------------------------------------------------
// Layer (c): native 491.52 -> PDU 65/32 -> SIC on work grid
// ---------------------------------------------------------------------------
void
run_native(const Options& o, const Model& m,
           const std::vector<std::vector<cplx>>& work_wins, std::ofstream& csv,
           std::ostringstream& json)
{
    const size_t n_work = work_window_len(o);
    std::vector<std::vector<cplx>> nat(work_wins.size());
    for (size_t w = 0; w < work_wins.size(); ++w)
        nat[w] = decimate_to_native(work_wins[w]);

    auto taps = load_taps("resampler_65_32/taps_quality_minorder.txt");
    auto rs = UwbPduRationalResamplerCcf65_32::make_from_taps(taps);
    const size_t max_out = rs->max_output_samples();
    auto blk = make_sic(m, 0, static_cast<int64_t>(n_work), max_out);

    auto sink = BenchSink::make();
    auto tb = gr::make_top_block("bench_preamble_sic_native");
    tb->msg_connect(rs, "packet", blk, "in");
    tb->msg_connect(blk, "out", sink, "in");
    tb->start();

    std::printf("\n== layer (c) native 491.52 -> PDU 65/32 -> SIC ==\n");
    std::printf("  taps=%zu native_in=%zu max_out=%zu windows=%zu rounds=%zu\n",
                rs->tap_count(), nat[0].size(), max_out, o.windows, o.rounds);

    auto post = [&](uint64_t id, size_t widx) {
        const auto& y = nat[widx];
        pmt::pmt_t pdu = pmt::cons(native_meta(id, static_cast<int64_t>(y.size())),
                                   pmt::init_c32vector(y.size(), y.data()));
        rs->_post(pmt::mp("packet"), pdu);
    };
    for (size_t w = 0; w < 200; ++w) {
        const uint64_t before = sink->count.load();
        post(2000000 + w, w % nat.size());
        const auto t0 = std::chrono::steady_clock::now();
        while (sink->count.load() < before + 1) {
            std::this_thread::sleep_for(std::chrono::microseconds(50));
            if (elapsed_us(t0, std::chrono::steady_clock::now()) > 30e6)
                throw std::runtime_error("native warmup timeout");
        }
    }
    sink->reset_stats();
    rs->reset_stats();

    std::vector<double> wall;
    uint64_t miss1 = 0, miss500 = 0;
    for (size_t rd = 0; rd < o.rounds; ++rd) {
        const size_t win_off = (rd * 29) % nat.size();
        const auto round_t0 = std::chrono::steady_clock::now();
        for (size_t k = 0; k < o.windows; ++k) {
            const uint64_t before = sink->count.load();
            const uint64_t id = (static_cast<uint64_t>(rd) << 40) + k + 1;
            const size_t widx = (k + win_off) % nat.size();
            const auto t0 = std::chrono::steady_clock::now();
            post(id, widx);
            const auto spin0 = std::chrono::steady_clock::now();
            while (sink->count.load() < before + 1) {
                std::this_thread::sleep_for(std::chrono::microseconds(50));
                if (elapsed_us(spin0, std::chrono::steady_clock::now()) > 60e6)
                    throw std::runtime_error("native sink timeout");
            }
            const double wus = elapsed_us(t0, std::chrono::steady_clock::now());
            wall.push_back(wus);
            if (wus > 1000.0)
                ++miss1;
            if (wus > 500.0)
                ++miss500;
            if ((k % 1000) == 0 && k > 0 &&
                elapsed_us(round_t0, std::chrono::steady_clock::now()) > 60e6) {
                const double mm =
                    std::accumulate(wall.begin(), wall.end(), 0.0) /
                    static_cast<double>(wall.size());
                std::printf("      [round %zu] batch %zu/%zu running wall mean="
                            "%.3f ms\n",
                            rd, k, o.windows, mm / 1000.0);
                std::fflush(stdout);
            }
        }
    }
    tb->stop();
    tb->wait();

    Stats w = summarize(wall);
    const uint64_t n = sink->count.load();
    auto mean_of = [&](uint64_t sum) {
        return n ? static_cast<double>(sum) / static_cast<double>(n) : 0.0;
    };
    print_row("wall (post->out)", w);
    std::printf("  per-window means: resample(meta)=%.1f core=%.1f handler=%.1f "
                "wrap=%.1f completion=%.1f us\n",
                mean_of(sink->sum_resample_us), mean_of(sink->sum_core_us),
                mean_of(sink->sum_handler_us), mean_of(sink->sum_wrap_us),
                mean_of(sink->sum_completion_us));
    if (rs->pdus_emitted() > 0) {
        std::printf("  resampler: emitted=%llu handler_mean=%.1f fir_mean=%.1f "
                    "publish_mean=%.1f us\n",
                    static_cast<unsigned long long>(rs->pdus_emitted()),
                    static_cast<double>(rs->handler_total_us()) /
                        static_cast<double>(rs->pdus_emitted()),
                    static_cast<double>(rs->resample_total_us()) /
                        static_cast<double>(rs->pdus_emitted()),
                    static_cast<double>(rs->publish_total_us()) /
                        static_cast<double>(rs->pdus_emitted()));
    }
    const double wps = w.mean_us > 0 ? 1e6 / w.mean_us : 0;
    std::printf("  completions=%llu committed=%llu misses>1ms=%llu >0.5ms=%llu "
                "checksum=%.6f  %.1f win/s\n",
                static_cast<unsigned long long>(n),
                static_cast<unsigned long long>(sink->committed.load()),
                static_cast<unsigned long long>(miss1),
                static_cast<unsigned long long>(miss500), sink->checksum, wps);
    for (size_t i = 0; i < wall.size(); ++i)
        csv << "native," << o.consumer << ',' << (i / o.windows) << ','
            << "rotating" << ',' << (i % nat.size()) << ',' << wall[i] << ','
            << "0,0\n";

    json << "\"native\":{\"n_work\":" << n_work
         << ",\"n_native\":" << nat[0].size() << ",\"windows\":" << o.windows
         << ",\"rounds\":" << o.rounds << ",\"wall\":";
    stats_obj(json, w);
    json << ",\"mean_resample_us\":" << mean_of(sink->sum_resample_us)
         << ",\"mean_core_us\":" << mean_of(sink->sum_core_us)
         << ",\"mean_handler_us\":" << mean_of(sink->sum_handler_us)
         << ",\"mean_wrap_us\":" << mean_of(sink->sum_wrap_us)
         << ",\"mean_completion_us\":" << mean_of(sink->sum_completion_us)
         << ",\"completions\":" << n
         << ",\"committed\":" << sink->committed.load()
         << ",\"deadline_miss_1ms\":" << miss1
         << ",\"deadline_miss_500us\":" << miss500 << ",\"win_per_s\":" << wps
         << ",\"checksum\":" << sink->checksum << "}";
}

} // namespace

int
main(int argc, char** argv)
{
    try {
        Options o;
        if (!parse_args(argc, argv, o))
            return 2;
        ensure_private_output(o.output);

        std::printf("== S0 preamble-only SIC layered benchmark ==\n");
        std::printf("preset=%s mode=%s consumer=%s windows=%zu rounds=%zu "
                    "hot_windows=%zu distinct=%zu output=%s\n",
                    o.preset.c_str(), o.mode.c_str(), o.consumer.c_str(),
                    o.windows, o.rounds, o.hot_windows, o.distinct,
                    o.output.c_str());
        std::printf("single worker, serial, one window in flight\n");

        const auto gen0 = std::chrono::steady_clock::now();
        Model m = build_model(10, kSyncReps);
        const size_t n_work = work_window_len(o);
        std::vector<std::vector<cplx>> wins(o.distinct);
        for (size_t w = 0; w < o.distinct; ++w)
            wins[w] = build_window(m, n_work, static_cast<int64_t>(kPre), 1e-3,
                                   1000 + w);
        const double gen_s = 1e-9 * static_cast<double>(
                                        std::chrono::duration_cast<
                                            std::chrono::nanoseconds>(
                                            std::chrono::steady_clock::now() -
                                            gen0)
                                            .count());
        std::printf("model preamble_len=%zu sync_len=%zu; %zu distinct windows "
                    "n=%zu; load gen=%.2f s (not timed)\n",
                    m.preamble.size(), m.sync.size(), wins.size(), n_work, gen_s);

        const std::string csv_path = o.output + "/" + o.mode + "_windows.csv";
        std::ofstream csv(csv_path, std::ios::trunc);
        if (!csv)
            throw std::runtime_error("cannot open " + csv_path);
        csv << "layer,consumer,round,case,window,wall_us,core_total_us,"
               "detect_coarse_us,fractional_refine_us,reconstruct_us,"
               "detect_fine_us\n";

        std::ostringstream json;
        json << std::setprecision(6);
        json << "{\n  \"mode\":\"" << o.mode << "\",\n  \"preset\":\""
             << o.preset << "\",\n  \"consumer\":\"" << o.consumer
             << "\",\n  \"windows\":" << o.windows << ",\n  \"rounds\":"
             << o.rounds << ",\n  \"hot_windows\":" << o.hot_windows
             << ",\n  \"distinct\":" << o.distinct << ",\n  \"sps\":" << kSps
             << ",\n  \"sync_reps\":" << kSyncReps
             << ",\n  \"k_paths\":4,\n  \"fit_reps\":8,\n  \"validation_reps\":8"
             << ",\n  \"detect_threshold\":" << kDetectThreshold
             << ",\n  \"validate_threshold\":" << kValidateThreshold
             << ",\n  \"n_work\":" << n_work << ",\n  \"layers\": {\n";

        const double cpu0 = cpu_seconds();
        if (o.mode == "core")
            run_core(o, m, wins, csv, json);
        else if (o.mode == "gr")
            run_gr(o, m, wins, csv, json);
        else
            run_native(o, m, wins, csv, json);
        const double cpu = cpu_seconds() - cpu0;

        json << "\n  },\n  \"cpu_seconds\":" << cpu << ",\n  \"rss_mb\":"
             << rss_mb() << ",\n  \"windows_per_s_target\":200"
             << ",\n  \"deadline_target\":{\"mean_us\":500.0,\"p99_us\":1000.0}"
             << "\n}\n";
        csv.close();
        const std::string json_path = o.output + "/" + o.mode + "_summary.json";
        std::ofstream jf(json_path, std::ios::trunc);
        jf << json.str();
        jf.close();

        std::printf("\nwrote %s and %s\n", csv_path.c_str(), json_path.c_str());
        std::printf("cpu_seconds=%.2f rss_mb=%.1f\n", cpu, rss_mb());
        return 0;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "benchmark_preamble_sic: %s\n", e.what());
        return 1;
    }
}
