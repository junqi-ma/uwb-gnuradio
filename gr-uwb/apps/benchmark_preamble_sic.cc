/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * S0 layered benchmark for the preamble-only SIC branch
 * (docs/sic/OpenCode开发指示_S0_前导碰撞仿真与耗时验证.md §7/§8.4/§9,
 *  rectification R03 in docs/sic/S0_整改意见报告_2026-10-16.md).
 *
 * ============================ WHAT THIS MEASURES ===========================
 * The OFFICIAL rows are measured on the SAME real code9/code10 HRP waveforms,
 * pulse, channel, ISR/SNR and sensing geometry as the S0 effect set:
 *
 *   - the corpus is re-derived with the effect-set generator
 *     (gr-uwb/apps/sic_preamble_sim.py) using the `main` case preset's nominal
 *     cell and the coverage-1.0 fit/validation split (default fit=8,
 *     validation=56 == `--cover-preamble`);
 *   - every input window / template is written as raw CF32 with a SHA-256 in
 *     manifest.json + inputs.txt, and the benchmark RE-VERIFIES those hashes at
 *     load time and (optionally) cross-checks them against an existing effect
 *     run (default: the `main_r07_v56` effect run) whose `fixtures/` npz hold
 *     the same `y` arrays;
 *   - the resolved config (ROIs, fit/val, thresholds, k, ISR/SNR, geometry) is
 *     recorded together with its SHA-256 (`config_hash`), plus the generator
 *     module/script hashes, the benchmark binary/source hashes and the loaded
 *     libgnuradio-uwb path.
 *
 * Three pipelines are measured separately, each in its OWN table, and the
 * consumer is a REAL GNU Radio block connected by msg_connect (never merely a
 * different array length):
 *
 *   (a) --consumer sic   : SIC only (direct core call or PDU adapter).
 *   (b) --consumer cir   : SIC -> UwbRadarCirEstimator (radar CIR-only).
 *   (c) --consumer demod : SIC -> UwbRealtimeDemodulator (full sensing demod).
 *
 * Layers (--mode) select the SIC execution form:
 *   core   : direct gr::uwb::sic::sic_one on the real work-grid windows.
 *   gr     : the UwbPreambleSic PDU adapter end-to-end (message in/out),
 *            single worker, real msg_connect to the consumer.
 *   native : real work windows -> documented PCHIP work->native decimation ->
 *            production PDU 65/32 resampler -> SIC on the work grid -> consumer.
 *            This is an APPROXIMATE native input (the native generator CLI,
 *            R08, is still open) and is labelled as such; it is NOT the
 *            official native acceptance evidence.
 *
 * Rules encoded here:
 *   - Release, single worker, serial (one window in flight at a time).
 *   - >= 64 distinct-content real windows are generated/loaded BEFORE timing.
 *   - 200-window warmup, excluded from every statistic, then rounds x windows.
 *   - During timing every window is CONSUMED and its TERMINAL STATE is checked
 *     (committed / bypassed / failed / ambiguous) AND the actual output effect
 *     is verified (support covers the whole injected overlap + downstream
 *     status / FCS / PSDU).  A run that silently measures a bypass /
 *     partial-subtraction / early-exit path is recorded as effect_verified
 *     false and exits non-zero: it cannot be quoted as full-processing speed.
 *   - mean / P50 / P95 / P99 / max, CPU seconds, RSS, queue and in-flight high
 *     watermarks, deadline misses.
 *   - CSV + JSON under --output (a private build/tmp path only); the tool
 *     refuses to write inside a git-tracked source directory.
 *
 * The old pseudo-random / 9-tap-Gaussian synthetic model is kept but RENAMED
 * ("synthetic microbenchmark", --input-source synthetic) and is NEVER labelled
 * as production-profile evidence.  Its numbers must not be used for S0
 * acceptance (R03).
 *
 * Usage:
 *   # once (or to refresh) the real corpus, then per pipeline:
 *   benchmark_preamble_sic --preset main --mode gr --consumer sic \
 *       --windows 2000 --rounds 2 --generate-inputs \
 *       --output gr-uwb/build-sic/sic-results/real_core
 *   benchmark_preamble_sic --preset main --mode gr --consumer cir  ...
 *   benchmark_preamble_sic --preset main --mode gr --consumer demod ...
 *   # old synthetic microbenchmark (NOT evidence):
 *   benchmark_preamble_sic --input-source synthetic --consumer cir ...
 */

#include <gnuradio/block.h>
#include <gnuradio/io_signature.h>
#include <gnuradio/top_block.h>
#include <gnuradio/uwb/uwb_pdu_rational_resampler_ccf_65_32.h>
#include <gnuradio/uwb/uwb_preamble_sic.h>
#include <gnuradio/uwb/uwb_preamble_sic_core.h>
#include <gnuradio/uwb/uwb_radar_cir_estimator_block.h>
#include <gnuradio/uwb/uwb_realtime_demodulator.h>
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
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <map>
#include <mutex>
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
using gr::uwb::UwbRadarCirEstimator;
using gr::uwb::UwbRealtimeDemodulator;

namespace {

// ---------------------------------------------------------------------------
// Frozen geometry / targets (the corpus manifest is authoritative for values
// that the generator resolves; these are only the tool's fallbacks).
// ---------------------------------------------------------------------------
constexpr double kPi = 3.14159265358979323846;
constexpr size_t kSps = 1016;
constexpr size_t kSyncReps = 64;
constexpr double kWorkRate = 998.4e6;
constexpr double kNativeRate = 491.52e6;
constexpr const char* kSfdMode = "4z2";
constexpr const char* kCanonicalPsdu =
    "47261DF66F4C1BEF45C8F77CE77BD7D8C4D180FB1221";
constexpr double kDetectThreshold = 0.40;
constexpr double kValidateThreshold = 0.25;
constexpr size_t kCoarseStride = 14;
constexpr size_t kPostTail = 100;

// ---------------------------------------------------------------------------
// CLI
// ---------------------------------------------------------------------------
struct Options {
    std::string preset = "main";
    std::string mode = "core";
    std::string consumer = "sic";
    std::string input_source = "real"; // real | synthetic
    std::string inputs;
    std::string output;
    std::string build_dir;
    std::string check_against; // effect run to hash-bind (default autodetect)
    bool generate_inputs = false;
    size_t windows = 2000;
    size_t rounds = 2;
    size_t distinct = 64;
    int fit_reps = 8;
    int validation_reps = 56;
    bool cover_preamble = false;
};

void
usage(const char* argv0)
{
    std::printf(
        "usage: %s --preset main --mode core|gr|native --consumer sic|cir|demod\n"
        "          --windows N --rounds R --output DIR\n"
        "          [--input-source real|synthetic] [--inputs DIR]\n"
        "          [--generate-inputs] [--build-dir DIR] [--check-against DIR]\n"
        "          [--distinct N]\n"
        "          [--fit-reps N --validation-reps N | --cover-preamble]\n"
        "\n"
        "official rows use real effect-set input (--input-source real, the\n"
        "default); --input-source synthetic is the OLD synthetic microbenchmark\n"
        "and is NOT production-profile evidence.\n",
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
        else if (a == "--input-source")
            o.input_source = need("--input-source");
        else if (a == "--inputs")
            o.inputs = need("--inputs");
        else if (a == "--output")
            o.output = need("--output");
        else if (a == "--build-dir")
            o.build_dir = need("--build-dir");
        else if (a == "--check-against")
            o.check_against = need("--check-against");
        else if (a == "--generate-inputs")
            o.generate_inputs = true;
        else if (a == "--windows")
            o.windows = std::stoul(need("--windows"));
        else if (a == "--rounds")
            o.rounds = std::stoul(need("--rounds"));
        else if (a == "--distinct")
            o.distinct = std::stoul(need("--distinct"));
        else if (a == "--fit-reps")
            o.fit_reps = std::stoi(need("--fit-reps"));
        else if (a == "--validation-reps")
            o.validation_reps = std::stoi(need("--validation-reps"));
        else if (a == "--cover-preamble")
            o.cover_preamble = true;
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
    if (o.consumer != "sic" && o.consumer != "cir" && o.consumer != "demod") {
        std::fprintf(stderr, "error: unknown --consumer '%s' (sic|cir|demod)\n",
                     o.consumer.c_str());
        return false;
    }
    if (o.input_source != "real" && o.input_source != "synthetic") {
        std::fprintf(stderr,
                     "error: unknown --input-source '%s' (real|synthetic)\n",
                     o.input_source.c_str());
        return false;
    }
    if (o.windows == 0 || o.rounds == 0 || o.distinct == 0) {
        std::fprintf(stderr, "error: --windows/--rounds/--distinct must be > 0\n");
        return false;
    }
    if (o.distinct < 64 && o.input_source == "real") {
        std::fprintf(stderr,
                     "error: --distinct must be >= 64 for real input (R03)\n");
        return false;
    }
    if (o.cover_preamble)
        o.validation_reps = static_cast<int>(kSyncReps) - o.fit_reps;
    if (o.fit_reps <= 0 || o.validation_reps < 8 ||
        o.fit_reps + o.validation_reps > static_cast<int>(kSyncReps)) {
        std::fprintf(stderr,
                     "error: fit/validation reps unsatisfiable: %d/%d "
                     "(needs >0, >=8 and fit+val<=64)\n",
                     o.fit_reps, o.validation_reps);
        return false;
    }
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
        return static_cast<double>(t.tv_sec) +
               1e-6 * static_cast<double>(t.tv_usec);
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

// Host load, so a shared-machine run records its contention context.
double
loadavg_1m()
{
    std::ifstream f("/proc/loadavg");
    double a = 0.0, b = 0.0, c = 0.0;
    if (!(f >> a >> b >> c))
        return -1.0;
    return a;
}

std::string
shell_capture(const std::string& cmd)
{
    std::FILE* p = ::popen(cmd.c_str(), "r");
    if (p == nullptr)
        return std::string();
    char buf[512] = { 0 };
    std::string out;
    while (std::fgets(buf, sizeof(buf), p) != nullptr)
        out += buf;
    ::pclose(p);
    while (!out.empty() && (out.back() == '\n' || out.back() == '\r'))
        out.pop_back();
    return out;
}

std::string
shell_sha256(const std::string& path)
{    if (path.empty() || !std::ifstream(path, std::ios::binary).good())
        return std::string();
    const std::string cmd = "sha256sum '" + path + "' 2>/dev/null";
    std::FILE* p = ::popen(cmd.c_str(), "r");
    if (p == nullptr)
        return std::string();
    char buf[256] = { 0 };
    std::string out;
    while (std::fgets(buf, sizeof(buf), p) != nullptr)
        out += buf;
    ::pclose(p);
    const size_t sp = out.find_first_of(" \t");
    return (sp == std::string::npos) ? std::string() : out.substr(0, sp);
}

bool
file_exists(const std::string& p)
{
    return !p.empty() && std::ifstream(p, std::ios::binary).good();
}

// SHA-256 of an in-memory string (used for the dirty-patch identity).
std::string
sha256_text(const std::string& text)
{
    const std::string tmp = "/tmp/opencode/_bench_sha256_tmp";
    std::ofstream f(tmp, std::ios::binary | std::ios::trunc);
    if (!f)
        return std::string();
    f << text;
    f.close();
    return shell_sha256(tmp);
}

// git provenance of the working tree the measurement ran in (R10).
struct GitInfo {
    std::string head, branch, status, status_sha;
};
GitInfo
git_info(const std::string& repo)
{
    GitInfo g;
    const std::string d = "cd '" + repo + "' && ";
    g.head = shell_capture(d + "git rev-parse HEAD 2>/dev/null");
    g.branch = shell_capture(d + "git rev-parse --abbrev-ref HEAD 2>/dev/null");
    g.status = shell_capture(d + "git status --porcelain 2>/dev/null");
    g.status_sha = sha256_text(g.status);
    return g;
}

std::string
self_exe_path()
{
    std::error_code ec;
    const auto p = std::filesystem::read_symlink("/proc/self/exe", ec);
    if (ec)
        return std::string();
    return p.string();
}

std::string
loaded_lib_path(const std::string& needle)
{
    std::ifstream f("/proc/self/maps");
    std::string line;
    while (std::getline(f, line)) {
        if (line.find(needle) == std::string::npos)
            continue;
        const size_t sp = line.find_last_of(' ');
        if (sp == std::string::npos)
            continue;
        std::string p = line.substr(sp + 1);
        while (!p.empty() && (p.back() == '\n' || p.back() == ' '))
            p.pop_back();
        if (!p.empty() && p[0] == '/')
            return p;
    }
    return std::string();
}

// Find the repository root: the directory that holds
// gr-uwb/apps/sic_preamble_sim.py.  Walks up from cwd and the executable.
std::string
find_repo_root()
{
    namespace fs = std::filesystem;
    std::vector<fs::path> starts;
    starts.push_back(fs::current_path());
    const std::string exe = self_exe_path();
    if (!exe.empty())
        starts.push_back(fs::path(exe).parent_path());
    const char* env = std::getenv("UWB_REPO_ROOT");
    if (env && env[0])
        starts.push_back(fs::path(env));
    for (const auto& s : starts) {
        fs::path p = fs::absolute(s);
        for (int i = 0; i < 10 && p.has_parent_path(); ++i) {
            if (fs::exists(p / "gr-uwb" / "apps" / "sic_preamble_sim.py"))
                return p.string();
            if (p == p.root_path())
                break;
            p = p.parent_path();
        }
    }
    return std::string();
}

// ---------------------------------------------------------------------------
// PMT helpers
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

int64_t
dict_i64(pmt::pmt_t dict, const char* key, int64_t def)
{
    if (!pmt::is_dict(dict))
        return def;
    pmt::pmt_t v = pmt::dict_ref(dict, pmt::mp(key), pmt::from_long(def));
    if (pmt::is_integer(v))
        return static_cast<int64_t>(pmt::to_long(v));
    if (pmt::is_uint64(v))
        return static_cast<int64_t>(pmt::to_uint64(v));
    return def;
}

double
dict_f64(pmt::pmt_t dict, const char* key, double def)
{
    if (!pmt::is_dict(dict))
        return def;
    pmt::pmt_t v = pmt::dict_ref(dict, pmt::mp(key), pmt::from_double(def));
    if (pmt::is_real(v))
        return pmt::to_double(v);
    if (pmt::is_integer(v))
        return static_cast<double>(pmt::to_long(v));
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

std::string
dict_str(pmt::pmt_t dict, const char* key, const std::string& def = std::string())
{
    if (!pmt::is_dict(dict))
        return def;
    pmt::pmt_t v = pmt::dict_ref(dict, pmt::mp(key), pmt::PMT_NIL);
    if (pmt::is_symbol(v))
        return pmt::symbol_to_string(v);
    return def;
}

std::string
to_upper_hex(const uint8_t* b, size_t n)
{
    static const char* H = "0123456789ABCDEF";
    std::string s;
    s.reserve(2 * n);
    for (size_t i = 0; i < n; ++i) {
        s.push_back(H[(b[i] >> 4) & 0xF]);
        s.push_back(H[b[i] & 0xF]);
    }
    return s;
}

std::string
json_escape(const std::string& s)
{
    std::string o;
    for (char c : s) {
        if (c == '"' || c == '\\')
            o.push_back('\\'), o.push_back(c);
        else if (c == '\n')
            o += "\\n";
        else
            o.push_back(c);
    }
    return o;
}

// CSV field: the reason strings contain separators like "partial_support[1,2]".
std::string
csv_field(const std::string& s)
{
    std::string o = s;
    for (char& c : o)
        if (c == ',' || c == '\n' || c == '\r')
            c = ';';
    return o;
}

// ---------------------------------------------------------------------------
// Terminal-state sink: records the last message's state AND the effect fields.
// ---------------------------------------------------------------------------
struct Snap {
    std::string sic_state; // committed / bypassed / failed / ambiguous
    std::string status;    // downstream status ("ok"/"success"/...)
    bool has_sic_state = false;
    bool committed = false;
    int64_t support_lo = -1, support_hi = -1;
    bool fcs_pass = false;
    bool has_psdu = false;
    std::string psdu_hex;
    uint64_t peak_tap = 0, valid_reps = 0, tap_count = 0;
    double residual_corr = 0.0;
    double peak_metric = 0.0;
    uint64_t core_total_us = 0, handler_us = 0, queue_wait_us = 0;
    uint64_t completion_us = 0;     // adapter out latency (SIC step)
    uint64_t consumer_total_us = 0; // worker-side total (demod/estimator)
    uint64_t queue_delay_us = 0;
};

class BenchSink : public gr::block
{
public:
    using sptr = std::shared_ptr<BenchSink>;
    static sptr make() { return gnuradio::get_initial_sptr(new BenchSink()); }

    std::atomic<uint64_t> count{ 0 };
    std::string expect_psdu; // uppercase hex, empty = no expectation
    std::mutex mu;
    Snap snap;

    Snap snapshot()
    {
        std::lock_guard<std::mutex> lk(mu);
        return snap;
    }

    void reset_stats()
    {
        count.store(0);
        std::lock_guard<std::mutex> lk(mu);
        snap = Snap();
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
        pmt::pmt_t data = pmt::cdr(m);
        Snap s;
        size_t n = 0;
        if (pmt::is_u8vector(data)) {
            const uint8_t* b = pmt::u8vector_elements(data, n);
            if (b != nullptr && n > 0) {
                s.has_psdu = true;
                s.psdu_hex = to_upper_hex(b, n);
            }
        } else if (pmt::is_c32vector(data)) {
            const gr_complex* v = pmt::c32vector_elements(data, n);
            if (v != nullptr && n > 0)
                checksum_ += static_cast<double>(std::abs(v[0])) +
                             static_cast<double>(std::abs(v[n / 2])) +
                             static_cast<double>(std::abs(v[n - 1]));
        }
        if (pmt::is_dict(meta)) {
            if (pmt::dict_has_key(meta, pmt::mp("sic_state"))) {
                s.has_sic_state = true;
                s.sic_state = dict_str(meta, "sic_state");
            }
            s.status = dict_str(meta, "status");
            s.committed = dict_bool(meta, "sic_committed", false);
            s.support_lo = dict_i64(meta, "sic_support_lo", -1);
            s.support_hi = dict_i64(meta, "sic_support_hi", -1);
            s.fcs_pass = dict_bool(meta, "fcs_pass", false);
            s.peak_tap = dict_u64(meta, "peak_tap", 0);
            s.valid_reps = dict_u64(meta, "valid_repetitions", 0);
            s.tap_count = dict_u64(meta, "tap_count", 0);
            s.residual_corr = dict_f64(meta, "sic_residual_corr", 0.0);
            s.peak_metric = dict_f64(meta, "cir_peak_metric", 0.0);
            s.core_total_us = dict_u64(meta, "core_total_us", 0);
            s.handler_us = dict_u64(meta, "handler_us", 0);
            s.queue_wait_us = dict_u64(meta, "queue_wait_us", 0);
            s.completion_us = dict_u64(meta, "completion_latency_us", 0);
            s.consumer_total_us =
                std::max(dict_u64(meta, "demod_latency_us", 0),
                         std::max(dict_u64(meta, "wall_latency_us", 0),
                                  dict_u64(meta, "estimator_us", 0)));
            s.queue_delay_us = dict_u64(meta, "queue_delay_us", 0);
        }
        {
            std::lock_guard<std::mutex> lk(mu);
            snap = s;
        }
        count.fetch_add(1, std::memory_order_release);
    }

public:
    double checksum_ = 0.0;
};

// ---------------------------------------------------------------------------
// Real-input corpus (R03)
// ---------------------------------------------------------------------------
struct WindowTruth {
    size_t index = 0;
    uint64_t seed = 0;
    size_t length = 0;
    std::string sha256;
    std::string file;
    int64_t overlap_lo = -1, overlap_hi = -1;
    double isr_db = std::nan("");
    double noise_std = 0.0;
};

struct Corpus {
    std::string schema;
    std::string generator_module;
    std::string generator_module_sha256;
    std::string generator_script_sha256;
    std::string config_hash;
    std::string reference_run;
    bool crosscheck_templates_match = false;
    bool crosscheck_fixture_match = false;
    size_t crosscheck_fixture_windows = 0;

    bool real_input = true;
    std::string preset = "main";
    int sensing_code = 9, comm_code = 10, sync_reps = 64;
    int fit_reps = 8, validation_reps = 56, k_paths = 4;
    double rate = kWorkRate, snr_db = 30.0, isr_db = 10.0;
    std::string sfd_mode = kSfdMode;
    std::string psdu_hex = kCanonicalPsdu;
    std::string pulse_shape = "legacy";
    int pulse_taps = 48;
    std::string frac_model = "pchip";
    std::string comm_template_mode;
    bool comm_template_r07_pass = false;
    int64_t pre = 1997, n = 0;
    int64_t search_lo = 0, search_hi = 0, sub_lo = 0, sub_hi = 0;
    double detect_threshold = kDetectThreshold;
    double validate_threshold = kValidateThreshold;
    size_t coarse_stride = kCoarseStride, post_tail = kPostTail;

    std::string sensing_sync_path, comm_sync_path, comm_preamble_path;
    std::string sensing_sync_sha, comm_sync_sha, comm_preamble_sha;
    size_t sensing_sync_len = 0, comm_sync_len = 0, comm_preamble_len = 0;
    std::vector<cplx> sensing_sync, comm_sync, comm_preamble;
    std::vector<WindowTruth> truths;
    std::vector<std::vector<cplx>> wins; // y
    std::vector<std::vector<cplx>> nat;  // derived native (approximation)

    size_t count() const { return wins.size(); }
};

std::vector<cplx>
load_cf32(const std::string& path, size_t& len_out)
{
    std::ifstream f(path, std::ios::binary | std::ios::ate);
    if (!f)
        throw std::runtime_error("cannot open " + path);
    const auto bytes = static_cast<size_t>(f.tellg());
    if (bytes % sizeof(cplx) != 0)
        throw std::runtime_error("bad CF32 size: " + path);
    f.seekg(0);
    std::vector<cplx> v(bytes / sizeof(cplx));
    f.read(reinterpret_cast<char*>(v.data()), static_cast<std::streamsize>(bytes));
    len_out = v.size();
    return v;
}

std::map<std::string, std::string>
parse_kv_file(const std::string& path)
{
    std::ifstream f(path);
    if (!f)
        throw std::runtime_error("cannot open " + path);
    std::map<std::string, std::string> kv;
    std::string line;
    while (std::getline(f, line)) {
        if (line.empty() || line[0] == '#')
            continue;
        std::istringstream is(line);
        std::string k;
        is >> k;
        std::string rest;
        std::getline(is, rest);
        const size_t a = rest.find_first_not_of(" \t");
        kv[k] = (a == std::string::npos) ? std::string() : rest.substr(a);
    }
    return kv;
}

std::string
kv_get(const std::map<std::string, std::string>& kv, const std::string& k,
       const std::string& def = std::string())
{
    auto it = kv.find(k);
    return (it == kv.end()) ? def : it->second;
}

// The R03 real-corpus generator, written next to its output so the exact
// generator text is archived and hashed.  It is a verbatim re-derivation of the
// effect set (same module, same nominal cell, same seeds).
std::string
generator_script_text()
{
    return R"PY(#!/usr/bin/env python3
"""R03 real-input corpus generator for benchmark_preamble_sic (work grid).

Re-derives the SAME code9/code10 HRP waveforms, pulse, channel, ISR/SNR and
window geometry as the S0 effect set (gr-uwb/apps/sic_preamble_sim.py, `main`
preset nominal cell) and writes raw CF32 windows plus a hash manifest.  It is
the OFFLINE generator side: the online SIC never reads anything from here.
"""
import hashlib
import importlib.util
import json
import os
import sys
import time

import numpy as np


def sha256_file(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def sha256_arr(a):
    return hashlib.sha256(np.ascontiguousarray(a).tobytes()).hexdigest()


def main():
    repo, build_dir, out, count, fit, val, post_tail, check_dir = sys.argv[1:9]
    count, fit, val, post_tail = int(count), int(fit), int(val), int(post_tail)
    os.makedirs(out, exist_ok=True)

    sim_path = os.path.join(repo, "gr-uwb", "apps", "sic_preamble_sim.py")
    spec = importlib.util.spec_from_file_location("sic_preamble_sim", sim_path)
    S = importlib.util.module_from_spec(spec)
    sys.modules["sic_preamble_sim"] = S
    spec.loader.exec_module(S)
    uwb = S.load_uwb(build_dir)

    preset_path = os.path.join(repo, "testdata", "sic_preamble", "cases",
                               "main.json")
    with open(preset_path) as f:
        preset = json.load(f)
    g = preset["generator"]
    nom = preset.get("nominal_for_benchmark", {})
    sensing_code = int(g.get("sensing_code", 9))
    comm_code = int(g.get("comm_code", 10))
    sync_reps = int(g.get("sync_reps", 64))
    snr_db = float(g.get("snr_db", 30.0))
    isr_db = nom.get("isr_db", 10.0)
    isr_db = None if isr_db is None else float(isr_db)
    start_off = float(nom.get("start_offset_work", 0.0))
    cfo_hz = float(nom.get("cfo_hz", 0.0))
    comm_paths = g.get("comm_paths", {}).get("single_int", [[0, [1.0, 0.0]]])

    gen = S.SicSignalGenerator(
        uwb, sensing_code=sensing_code, comm_code=comm_code,
        sync_reps=sync_reps, sfd_mode=S.SFD_MODE, psdu_hex=S.DEFAULT_PSDU_HEX,
        comm_present=True, build_comm=True, snr_db=snr_db, isr_db=isr_db,
        verbose=True)
    if gen.comm_wave is None:
        raise SystemExit("comm template was not built")

    search_roi = (max(0, gen.pre), min(gen.n, gen.pre + sync_reps * S.SPS))
    sub_roi = (search_roi[0],
               min(gen.n, search_roi[1] + (fit + val) * S.SPS + post_tail))
    core_roi = (sub_roi[0], sub_roi[1])

    tpl = {
        "sensing_sync_code9": (gen.template, "sync_template_code9_998p4.cf32"),
        "comm_sync_code10": (gen.comm_sync, "comm_sync_code10_998p4.cf32"),
        "comm_preamble_code10": (gen.comm_wave,
                                 "comm_preamble_code10_998p4.cf32"),
    }
    tpl_meta = {}
    for name, (arr, fn) in tpl.items():
        arr = np.ascontiguousarray(arr, dtype=np.complex64)
        arr.tofile(os.path.join(out, fn))
        tpl_meta[name] = {"file": fn, "length": int(arr.size),
                          "sha256": sha256_arr(arr)}

    r07 = gen.comm_template_evidence() or {}
    r07_pass = bool(r07.get("pass"))

    wins = []
    for i in range(count):
        seed = 1000000 + i
        w = gen.build(seed=seed, comm_paths=comm_paths,
                      start_offset_work=start_off, cfo_hz=cfo_hz,
                      isr_db=isr_db, snr_db=snr_db)
        y = np.ascontiguousarray(w["y"], dtype=np.complex64)
        fn = "win_%04d.cf32" % i
        y.tofile(os.path.join(out, fn))
        wins.append({"index": i, "seed": int(seed), "file": fn,
                     "length": int(y.size), "sha256": sha256_arr(y),
                     "overlap_lo": int(w["overlap"]["lo"]),
                     "overlap_hi": int(w["overlap"]["hi"]),
                     "isr_db": (None if w["isr_db"] is None
                                else float(w["isr_db"])),
                     "noise_std": float(w["noise_std"])})

    cross = {"reference_run": (os.path.basename(check_dir.rstrip("/"))
                               if check_dir else None)}
    if check_dir and os.path.isdir(check_dir):
        tpl_map = {
            "sensing_sync_code9": "sync_template_code9_998p4.cf32",
            "comm_sync_code10": "comm_sync_code10_998p4.cf32",
            "comm_preamble_code10": "comm_preamble_code10_998p4.cf32",
        }
        th, tm = {}, {}
        for k, fn in tpl_map.items():
            p = os.path.join(check_dir, "templates", fn)
            th[k] = sha256_file(p) if os.path.isfile(p) else None
            tm[k] = bool(th[k] is not None and th[k] == tpl_meta[k]["sha256"])
        cross["template_hashes"] = th
        cross["templates_match"] = bool(all(tm.values()))
        n_fix, fix_ok = 0, True
        for i in range(count):
            p = os.path.join(check_dir, "fixtures", "window_%04d.npz" % i)
            if not os.path.isfile(p):
                break
            z = np.load(p)
            if "y" not in z or not np.array_equal(
                    np.ascontiguousarray(z["y"], dtype=np.complex64),
                    np.fromfile(os.path.join(out, wins[i]["file"]),
                                dtype=np.complex64)):
                fix_ok = False
                break
            n_fix += 1
        cross["fixture_windows_compared"] = n_fix
        cross["fixture_y_match"] = bool(fix_ok and n_fix > 0)

    cfg = {
        "preset": "main", "input_grid": "work",
        "sensing_code": sensing_code, "comm_code": comm_code,
        "sync_reps": sync_reps, "sfd_mode": S.SFD_MODE,
        "psdu_hex": S.DEFAULT_PSDU_HEX,
        "pulse_shape": gen.pulse_shape, "pulse_taps": int(gen.pulse_taps),
        "frac_model": gen.frac_model,
        "sensing_paths": [[p.delay_samples, [p.gain.real, p.gain.imag]]
                          for p in gen.sensing_paths],
        "comm_paths": comm_paths, "comm_present": True,
        "start_offset_work": start_off, "cfo_hz": cfo_hz,
        "snr_db": snr_db, "isr_db": isr_db,
        "fit_reps": fit, "validation_reps": val,
        "k_paths": int(preset.get("runtime", {}).get("sic_k", 4)),
        "post_tail": post_tail, "coarse_stride": 14,
        "detect_threshold": 0.40, "validate_threshold": 0.25,
        "pre_guard": int(gen.pre), "window_len": int(gen.n),
        "geometry": {k: int(v) for k, v in gen.geom.items()},
        "search_roi": [int(search_roi[0]), int(search_roi[1])],
        "subtraction_roi": [int(sub_roi[0]), int(sub_roi[1])],
        "core_roi": [int(core_roi[0]), int(core_roi[1])],
        "samples_per_symbol": int(S.SPS), "sample_rate": float(S.WORK_HZ),
        "seed_set": "calibration", "seed_start": 1000000,
        "windows": count,
    }
    config_hash = hashlib.sha256(
        json.dumps(cfg, sort_keys=True).encode()).hexdigest()

    manifest = {
        "schema": "sic_perf_real_inputs_v1",
        "generated_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "generator": {
            "module": sim_path,
            "module_sha256": sha256_file(sim_path),
            "script": os.path.abspath(__file__),
            "script_sha256": sha256_file(os.path.abspath(__file__)),
        },
        "resolved_config": cfg,
        "config_hash": config_hash,
        "templates": tpl_meta,
        "comm_template_mode": gen.comm_meta.get("mode"),
        "comm_template_r07_pass": r07_pass,
        "comm_template_length": int(gen.comm_wave.size),
        "crosscheck": cross,
        "count": count,
        "windows": wins,
    }
    with open(os.path.join(out, "manifest.json"), "w") as f:
        json.dump(manifest, f, indent=2, sort_keys=True)

    L = []

    def add(k, v):
        L.append("%s %s" % (k, v))

    add("schema", "sic_perf_real_inputs_v1")
    add("generator_module", sim_path)
    add("generator_module_sha256", manifest["generator"]["module_sha256"])
    add("generator_script_sha256", manifest["generator"]["script_sha256"])
    add("config_hash", config_hash)
    for k in sorted(cfg):
        v = cfg[k]
        if isinstance(v, (list, dict)):
            v = json.dumps(v, separators=(",", ":"))
        add("cfg." + k, v)
    for name in tpl_meta:
        add("template.%s.file" % name, tpl_meta[name]["file"])
        add("template.%s.length" % name, tpl_meta[name]["length"])
        add("template.%s.sha256" % name, tpl_meta[name]["sha256"])
    add("comm_template_mode", gen.comm_meta.get("mode"))
    add("comm_template_r07_pass", str(r07_pass).lower())
    add("crosscheck.reference_run", cross.get("reference_run"))
    add("crosscheck.templates_match",
        str(cross.get("templates_match", False)).lower())
    add("crosscheck.fixture_windows", cross.get("fixture_windows_compared", 0))
    add("crosscheck.fixture_y_match",
        str(cross.get("fixture_y_match", False)).lower())
    add("windows", count)
    for w in wins:
        L.append("window %d %d %d %s %d %d %s %s %s" % (
            w["index"], w["seed"], w["length"], w["sha256"],
            w["overlap_lo"], w["overlap_hi"],
            ("nan" if w["isr_db"] is None else repr(w["isr_db"])),
            repr(w["noise_std"]), w["file"]))
    with open(os.path.join(out, "inputs.txt"), "w") as f:
        f.write("\n".join(L) + "\n")
    print("real corpus: %d windows of %d samples; config_hash=%s; "
          "templates_match=%s fixture_y_match=%s"
          % (count, int(gen.n), config_hash, cross.get("templates_match"),
             cross.get("fixture_y_match")), flush=True)


if __name__ == "__main__":
    main()
)PY";
}

void
run_generator(const Options& o, const std::string& repo,
              const std::string& build_dir, const std::string& inputs)
{
    namespace fs = std::filesystem;
    fs::create_directories(inputs);
    const std::string script = inputs + "/generate_real_inputs.py";
    {
        std::ofstream f(script, std::ios::trunc);
        if (!f)
            throw std::runtime_error("cannot write " + script);
        f << generator_script_text();
    }
    const std::string cmd = "python3 '" + script + "' '" + repo + "' '" +
                            build_dir + "' '" + inputs + "' " +
                            std::to_string(o.distinct) + " " +
                            std::to_string(o.fit_reps) + " " +
                            std::to_string(o.validation_reps) + " " +
                            std::to_string(kPostTail) + " '" + o.check_against +
                            "'";
    std::printf("generating real corpus:\n  %s\n", cmd.c_str());
    std::fflush(stdout);
    const int rc = std::system(cmd.c_str());
    if (rc != 0)
        throw std::runtime_error("real-corpus generator failed (rc=" +
                                 std::to_string(rc) + "): " + cmd);
}

Corpus
load_real_corpus(const Options& o, const std::string& inputs)
{
    const std::string kv_path = inputs + "/inputs.txt";
    if (!file_exists(kv_path))
        throw std::runtime_error(
            "missing " + kv_path +
            " (run with --generate-inputs, or point --inputs at a corpus dir)");
    const auto kv = parse_kv_file(kv_path);

    Corpus c;
    c.real_input = true;
    c.schema = kv_get(kv, "schema");
    if (c.schema != "sic_perf_real_inputs_v1")
        throw std::runtime_error("unexpected corpus schema: " + c.schema);
    c.generator_module = kv_get(kv, "generator_module");
    c.generator_module_sha256 = kv_get(kv, "generator_module_sha256");
    c.generator_script_sha256 = kv_get(kv, "generator_script_sha256");
    c.config_hash = kv_get(kv, "config_hash");
    c.reference_run = kv_get(kv, "crosscheck.reference_run");
    c.crosscheck_templates_match =
        kv_get(kv, "crosscheck.templates_match") == "true";
    c.crosscheck_fixture_match =
        kv_get(kv, "crosscheck.fixture_y_match") == "true";
    c.crosscheck_fixture_windows =
        std::stoul(kv_get(kv, "crosscheck.fixture_windows", "0"));

    auto kv_i = [&](const char* k) { return std::stoll(kv_get(kv, k, "0")); };
    auto kv_d = [&](const char* k) { return std::stod(kv_get(kv, k, "0")); };
    c.preset = kv_get(kv, "cfg.preset", "main");
    c.sensing_code = static_cast<int>(kv_i("cfg.sensing_code"));
    c.comm_code = static_cast<int>(kv_i("cfg.comm_code"));
    c.sync_reps = static_cast<int>(kv_i("cfg.sync_reps"));
    c.fit_reps = static_cast<int>(kv_i("cfg.fit_reps"));
    c.validation_reps = static_cast<int>(kv_i("cfg.validation_reps"));
    c.k_paths = static_cast<int>(kv_i("cfg.k_paths"));
    c.rate = kv_d("cfg.sample_rate");
    c.snr_db = kv_d("cfg.snr_db");
    c.isr_db = kv_d("cfg.isr_db");
    c.sfd_mode = kv_get(kv, "cfg.sfd_mode", kSfdMode);
    c.psdu_hex = kv_get(kv, "cfg.psdu_hex", kCanonicalPsdu);
    c.pulse_shape = kv_get(kv, "cfg.pulse_shape", "legacy");
    c.pulse_taps = static_cast<int>(kv_i("cfg.pulse_taps"));
    c.frac_model = kv_get(kv, "cfg.frac_model", "pchip");
    c.pre = kv_i("cfg.pre_guard");
    c.n = kv_i("cfg.window_len");
    c.detect_threshold = kv_d("cfg.detect_threshold");
    c.validate_threshold = kv_d("cfg.validate_threshold");
    c.coarse_stride = static_cast<size_t>(kv_i("cfg.coarse_stride"));
    c.post_tail = static_cast<size_t>(kv_i("cfg.post_tail"));
    {
        auto roi = [&](const char* key, int64_t& lo, int64_t& hi) {
            std::string s = kv_get(kv, key, "[0,0]");
            for (char& ch : s)
                if (ch == '[' || ch == ']' || ch == ',')
                    ch = ' ';
            std::istringstream is(s);
            is >> lo >> hi;
        };
        roi("cfg.search_roi", c.search_lo, c.search_hi);
        roi("cfg.subtraction_roi", c.sub_lo, c.sub_hi);
    }
    c.comm_template_mode = kv_get(kv, "comm_template_mode");
    c.comm_template_r07_pass = kv_get(kv, "comm_template_r07_pass") == "true";

    c.sensing_sync_path =
        inputs + "/" + kv_get(kv, "template.sensing_sync_code9.file");
    c.comm_sync_path =
        inputs + "/" + kv_get(kv, "template.comm_sync_code10.file");
    c.comm_preamble_path =
        inputs + "/" + kv_get(kv, "template.comm_preamble_code10.file");
    c.sensing_sync_sha = kv_get(kv, "template.sensing_sync_code9.sha256");
    c.comm_sync_sha = kv_get(kv, "template.comm_sync_code10.sha256");
    c.comm_preamble_sha = kv_get(kv, "template.comm_preamble_code10.sha256");
    c.sensing_sync_len =
        std::stoul(kv_get(kv, "template.sensing_sync_code9.length", "0"));
    c.comm_sync_len =
        std::stoul(kv_get(kv, "template.comm_sync_code10.length", "0"));
    c.comm_preamble_len =
        std::stoul(kv_get(kv, "template.comm_preamble_code10.length", "0"));

    // Re-verify every recorded hash before the corpus is used as evidence.
    auto check = [&](const std::string& p, const std::string& want,
                     const char* what) {
        const std::string got = shell_sha256(p);
        if (got != want)
            throw std::runtime_error(std::string("corpus hash mismatch for ") +
                                     what + ": " + p + " got " + got +
                                     " want " + want);
    };
    check(c.sensing_sync_path, c.sensing_sync_sha, "sensing_sync_code9");
    check(c.comm_sync_path, c.comm_sync_sha, "comm_sync_code10");
    check(c.comm_preamble_path, c.comm_preamble_sha, "comm_preamble_code10");

    const size_t nwin = std::stoul(kv_get(kv, "windows", "0"));
    c.wins.resize(nwin);
    c.truths.resize(nwin);
    std::ifstream f(kv_path);
    std::string line;
    while (std::getline(f, line)) {
        if (line.rfind("window ", 0) != 0)
            continue;
        std::istringstream is(line);
        std::string tag;
        WindowTruth t;
        std::string isr_s, noise_s;
        is >> tag >> t.index >> t.seed >> t.length >> t.sha256 >>
            t.overlap_lo >> t.overlap_hi >> isr_s >> noise_s >> t.file;
        t.isr_db = (isr_s == "nan") ? std::nan("") : std::stod(isr_s);
        t.noise_std = std::stod(noise_s);
        if (t.index >= nwin)
            throw std::runtime_error("window index out of range in " + kv_path);
        c.truths[t.index] = t;
    }
    for (size_t i = 0; i < nwin; ++i) {
        const WindowTruth& t = c.truths[i];
        if (t.file.empty())
            throw std::runtime_error("missing window record " +
                                     std::to_string(i));
        const std::string p = inputs + "/" + t.file;
        const std::string got = shell_sha256(p);
        if (got != t.sha256)
            throw std::runtime_error("window hash mismatch " + p + " got " +
                                     got + " want " + t.sha256);
        size_t len = 0;
        c.wins[i] = load_cf32(p, len);
        if (len != t.length || static_cast<int64_t>(len) != c.n)
            throw std::runtime_error("window length mismatch " + p);
    }
    if (c.n <= 0 || c.wins.empty())
        throw std::runtime_error("empty real corpus");
    if (c.k_paths > 16)
        throw std::runtime_error("corpus k_paths out of range");
    // The corpus must be the exact configuration the run claims: the ROI and
    // the SIC behaviour depend on fit/val, so a mismatch is a hard error (the
    // perf row may only bind to the case the corpus was generated from).
    if (c.fit_reps != o.fit_reps || c.validation_reps != o.validation_reps)
        throw std::runtime_error(
            "corpus config mismatch: corpus has fit/val " +
            std::to_string(c.fit_reps) + "/" + std::to_string(c.validation_reps) +
            " but the run requested " + std::to_string(o.fit_reps) + "/" +
            std::to_string(o.validation_reps) +
            "; regenerate with --generate-inputs (or point --inputs at the "
            "matching corpus)");
    if (o.distinct > c.count())
        throw std::runtime_error(
            "corpus has " + std::to_string(c.count()) +
            " windows but --distinct " + std::to_string(o.distinct) +
            " was requested; regenerate with --generate-inputs");
    c.sensing_sync = load_cf32(c.sensing_sync_path, c.sensing_sync_len);
    c.comm_sync = load_cf32(c.comm_sync_path, c.comm_sync_len);
    c.comm_preamble = load_cf32(c.comm_preamble_path, c.comm_preamble_len);
    return c;
}

// ---------------------------------------------------------------------------
// Synthetic microbenchmark corpus (the OLD model, renamed; NOT evidence)
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
        const double w = 0.42 + 0.5 * std::cos(kPi * t / H) +
                         0.08 * std::cos(2.0 * kPi * t / H);
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

struct SynModel {
    std::vector<cplx> preamble;
    std::vector<cplx> sync;
};

SynModel
build_syn_model(size_t code, size_t reps)
{
    SynModel m;
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
add_syn_path(std::vector<cplx>& acc, const std::vector<cplx>& pre, int64_t base,
             double tau, cplx g, double cfo_hz)
{
    const int64_t span0 = base + static_cast<int64_t>(std::floor(tau)) - 32;
    const int64_t span1 = base + static_cast<int64_t>(std::ceil(tau)) +
                          static_cast<int64_t>(pre.size()) + 32;
    const int64_t lo = std::max<int64_t>(0, span0);
    const int64_t hi = std::min<int64_t>(static_cast<int64_t>(acc.size()), span1);
    const double w = 2.0 * kPi * cfo_hz / kWorkRate;
    for (int64_t i = lo; i < hi; ++i) {
        const cplx v = gen_interp(pre, static_cast<double>(i) -
                                           static_cast<double>(base) - tau);
        const double ph = w * static_cast<double>(i);
        acc[static_cast<size_t>(i)] +=
            g * v * cplx(static_cast<float>(std::cos(ph)),
                         static_cast<float>(std::sin(ph)));
    }
}

Corpus
build_synthetic_corpus(const Options& o)
{
    constexpr size_t kPreSyn = 2000;
    struct SPath {
        double tau;
        cplx g;
    };
    const SPath paths[4] = {
        { 0.0, cplx(1.0f, 0.0f) },
        { 7.4, cplx(0.0f, 0.5f) },
        { 23.0, cplx(0.276f, -0.116f) },
        { 51.2, cplx(0.2f, 0.0f) },
    };
    const size_t n = kPreSyn + kSyncReps * kSps + 8 * kSps + 100 + 4094;

    Corpus c;
    c.real_input = false;
    c.preset = "synthetic";
    c.sensing_code = 9;
    c.comm_code = 10;
    c.sync_reps = static_cast<int>(kSyncReps);
    c.fit_reps = o.fit_reps;
    c.validation_reps = o.validation_reps;
    c.rate = kWorkRate;
    c.pre = static_cast<int64_t>(kPreSyn);
    c.n = static_cast<int64_t>(n);
    c.search_lo = static_cast<int64_t>(kPreSyn);
    c.search_hi = static_cast<int64_t>(kPreSyn + kSyncReps * kSps);
    c.sub_lo = c.search_lo;
    c.sub_hi = c.search_hi + static_cast<int64_t>(kPostTail);
    c.psdu_hex = kCanonicalPsdu;
    c.comm_template_mode = "synthetic_microbenchmark";
    c.config_hash = "synthetic";
    SynModel m = build_syn_model(10, kSyncReps);
    c.comm_sync = m.sync;
    c.comm_preamble = m.preamble;
    c.sensing_sync = m.sync;
    c.wins.resize(o.distinct);
    c.truths.resize(o.distinct);
    for (size_t w = 0; w < o.distinct; ++w) {
        std::vector<cplx> i(n, cplx(0.f, 0.f));
        std::mt19937_64 rng(1000 + w);
        std::uniform_real_distribution<double> ph(0.0, 2.0 * kPi);
        for (const SPath& p : paths) {
            const double extra = ph(rng);
            const cplx g = p.g * cplx(static_cast<float>(std::cos(extra)),
                                      static_cast<float>(std::sin(extra)));
            add_syn_path(i, m.preamble, static_cast<int64_t>(kPreSyn), p.tau, g,
                         0.0);
        }
        std::normal_distribution<double> nd(0.0, 1e-3);
        std::vector<cplx> y(n);
        for (size_t k = 0; k < n; ++k)
            y[k] = i[k] + cplx(static_cast<float>(nd(rng)),
                               static_cast<float>(nd(rng)));
        c.wins[w] = std::move(y);
        WindowTruth& t = c.truths[w];
        t.index = w;
        t.seed = 1000 + w;
        t.length = n;
        t.overlap_lo = static_cast<int64_t>(kPreSyn);
        t.overlap_hi = static_cast<int64_t>(kPreSyn + kSyncReps * kSps);
        t.isr_db = std::nan("");
        t.noise_std = 1e-3;
    }
    return c;
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
// Output guard
// ---------------------------------------------------------------------------
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
// Stats printing / JSON
// ---------------------------------------------------------------------------
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

// ---------------------------------------------------------------------------
// testdata taps
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
        "testdata/",          "../testdata/",         "../../testdata/",
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

// ---------------------------------------------------------------------------
// Per-window pipeline outcome
// ---------------------------------------------------------------------------
struct Outcome {
    double wall_us = 0.0;     // submit -> downstream terminal result
    double sic_us = 0.0;      // SIC step only
    double consumer_us = 0.0; // consumer step only
    Snap sic;
    Snap cons;
    bool effect_ok = false;
    std::string effect_reason;
    std::string downstream; // "sic_only" | status string
};

// ---------------------------------------------------------------------------
// Pipeline: SIC (core / adapter / native) + optional real consumer
// ---------------------------------------------------------------------------
class Pipeline
{
public:
    Pipeline(const Options& o, const Corpus& c) : o_(o), c_(c)
    {
        tb_ = gr::make_top_block("bench_preamble_sic");

        if (o_.consumer == "cir") {
            cir_ = UwbRadarCirEstimator::make(
                c_.sensing_sync_path, kSyncReps, kSfdMode,
                static_cast<size_t>(c_.sensing_code), 16, 100, 10, 0, 128, 32,
                0.12f, 0.02f, true, 64);
            consumer_blk_ = cir_;
            cons_sink_ = BenchSink::make();
            tb_->msg_connect(cir_, "cir", cons_sink_, "in");
        } else if (o_.consumer == "demod") {
            demod_ = UwbRealtimeDemodulator::make(
                c_.sensing_sync_path, 1, 64, kSfdMode, 0, "bypass",
                static_cast<size_t>(c_.sensing_code), kSyncReps, 14);
            consumer_blk_ = demod_;
            cons_sink_ = BenchSink::make();
            cons_sink_->expect_psdu = c_.psdu_hex;
            tb_->msg_connect(demod_, "result", cons_sink_, "in");
        }

        if (o_.mode != "core") {
            if (o_.mode == "native") {
                auto taps =
                    load_taps("resampler_65_32/taps_quality_minorder.txt");
                rs_ = UwbPduRationalResamplerCcf65_32::make_from_taps(taps);
                max_out_ = rs_->max_output_samples();
            }
            sic_ = make_sic(c);
            if (rs_)
                tb_->msg_connect(rs_, "packet", sic_, "in");
            tb_->msg_connect(sic_, "out", sic_sink_, "in");
            if (consumer_blk_)
                tb_->msg_connect(sic_, "out", consumer_blk_, consumer_port());
        }
    }

    void start() { tb_->start(); }
    void stop()
    {
        tb_->stop();
        tb_->wait();
    }

    bool real_consumer() const { return consumer_blk_ != nullptr; }
    UwbPreambleSic::sptr sic() const { return sic_; }

    uint64_t sic_count() const { return sic_sink_->count.load(); }
    uint64_t cons_count() const
    {
        return consumer_blk_ ? cons_sink_->count.load() : sic_sink_->count.load();
    }
    Snap sic_snap() { return sic_sink_->snapshot(); }
    Snap cons_snap()
    {
        return consumer_blk_ ? cons_sink_->snapshot() : sic_sink_->snapshot();
    }
    double checksum() const
    {
        return sic_sink_->checksum_ +
               (consumer_blk_ ? cons_sink_->checksum_ : 0.0) + core_checksum_;
    }
    void reset_sinks()
    {
        if (sic_sink_)
            sic_sink_->reset_stats();
        if (cons_sink_)
            cons_sink_->reset_stats();
        if (rs_)
            rs_->reset_stats();
    }
    size_t inflight_hwm() const
    {
        return sic_ ? sic_->inflight_high_watermark() : 0;
    }
    size_t queue_hwm() const { return sic_ ? sic_->queue_high_watermark() : 0; }
    size_t consumer_queue_hwm() const
    {
        if (cir_)
            return cir_->queue_high_watermark();
        if (demod_)
            return demod_->queue_high_watermark();
        return 0;
    }
    uint64_t consumer_jobs_dropped() const
    {
        if (cir_)
            return cir_->pdus_dropped();
        if (demod_)
            return demod_->jobs_dropped();
        return 0;
    }
    double consumer_service_mean_us() const
    {
        if (cir_)
            return static_cast<double>(cir_->service_mean_us());
        if (demod_)
            return static_cast<double>(demod_->stage_mean_total_us());
        return 0.0;
    }

    // Submit window `widx`; block until the downstream terminal state.
    Outcome submit(uint64_t id, size_t widx, double timeout_us);

private:
    const char* consumer_port() const
    {
        return (o_.consumer == "cir") ? "rx" : "samples";
    }

    pmt::pmt_t base_meta(uint64_t id) const
    {
        pmt::pmt_t d = pmt::make_dict();
        d = pmt::dict_add(d, pmt::mp("window_id"), pmt::from_uint64(id));
        d = pmt::dict_add(d, pmt::mp("packet_id"), pmt::from_uint64(id));
        d = pmt::dict_add(d, pmt::mp("schedule_index"), pmt::from_uint64(id));
        d = pmt::dict_add(d, pmt::mp("epoch_id"), pmt::from_uint64(0));
        d = pmt::dict_add(d, pmt::mp("run_id"), pmt::from_uint64(1));
        d = pmt::dict_add(d, pmt::mp("sample_count"),
                          pmt::from_long(static_cast<long>(c_.n)));
        d = pmt::dict_add(d, pmt::mp("sample_rate"), pmt::from_double(kWorkRate));
        d = pmt::dict_add(d, pmt::mp("sensing_code"),
                          pmt::from_long(c_.sensing_code));
        d = pmt::dict_add(d, pmt::mp("sensing_sync_reps"),
                          pmt::from_long(c_.sync_reps));
        d = pmt::dict_add(d, pmt::mp("sensing_sfd_mode"), pmt::mp(kSfdMode));
        d = pmt::dict_add(d, pmt::mp("sfd_mode"), pmt::mp(kSfdMode));
        d = pmt::dict_add(d, pmt::mp("sensing_profile_id"),
                          pmt::mp("code" + std::to_string(c_.sensing_code) +
                                  "-" + std::to_string(c_.sync_reps) + "SYNC-" +
                                  kSfdMode));
        // CIR contract
        d = pmt::dict_add(d, pmt::mp("pulse_id"), pmt::from_uint64(id));
        d = pmt::dict_add(d, pmt::mp("pre_guard_samples"),
                          pmt::from_long(c_.pre));
        d = pmt::dict_add(d, pmt::mp("window_start_sample"), pmt::from_long(0));
        d = pmt::dict_add(d, pmt::mp("sync_repetitions"),
                          pmt::from_long(c_.sync_reps));
        d = pmt::dict_add(d, pmt::mp("code_index"),
                          pmt::from_long(c_.sensing_code));
        d = pmt::dict_add(d, pmt::mp("calibration_delay_native_samples"),
                          pmt::from_double(0.0));
        // demod contract
        d = pmt::dict_add(d, pmt::mp("predicted_start_sample"),
                          pmt::from_long(c_.pre));
        d = pmt::dict_add(d, pmt::mp("predicted_sensing_start"),
                          pmt::from_long(c_.pre));
        d = pmt::dict_add(d, pmt::mp("template_id"),
                          pmt::mp("sync_template_code" +
                                  std::to_string(c_.sensing_code) + "_998p4"));
        return d;
    }

    UwbPreambleSic::sptr make_sic(const Corpus& c)
    {
        const size_t max_window =
            (o_.mode == "native" && max_out_ > 0) ? max_out_
                                                  : static_cast<size_t>(c.n);
        return UwbPreambleSic::make_from_templates(
            c.comm_sync, c.comm_preamble, static_cast<size_t>(c.comm_code),
            static_cast<size_t>(c.sync_reps), kSps, kWorkRate,
            static_cast<size_t>(c.k_paths), static_cast<size_t>(c.fit_reps),
            static_cast<size_t>(c.validation_reps),
            static_cast<int64_t>(c.search_lo), static_cast<int64_t>(c.sub_hi),
            c.coarse_stride, "estimate", c.detect_threshold,
            c.validate_threshold, /*in_flight_max=*/8,
            /*iq_budget_bytes=*/size_t(64) << 20, /*queue_capacity=*/8,
            /*publish_reconstruction=*/false, /*enable_fractional=*/true,
            /*enable_cfo=*/true, /*path_min_spacing=*/0,
            /*post_tail=*/c.post_tail, /*max_window_samples=*/max_window,
            /*pending_bypass_max=*/0);
    }

    Snap run_core_one(const std::vector<cplx>& y);

    void post_sic(uint64_t id, size_t widx)
    {
        const auto& y = c_.wins[widx];
        pmt::pmt_t pdu = pmt::cons(base_meta(id),
                                   pmt::init_c32vector(y.size(), y.data()));
        sic_->_post(pmt::mp("in"), pdu);
    }

    void post_native(uint64_t id, size_t widx)
    {
        const auto& y = c_.nat[widx];
        pmt::pmt_t d = pmt::make_dict();
        d = pmt::dict_add(d, pmt::mp("window_id"), pmt::from_uint64(id));
        d = pmt::dict_add(d, pmt::mp("packet_id"), pmt::from_uint64(id));
        d = pmt::dict_add(d, pmt::mp("schedule_index"), pmt::from_uint64(id));
        d = pmt::dict_add(d, pmt::mp("sample_rate"),
                          pmt::from_double(kNativeRate));
        d = pmt::dict_add(d, pmt::mp("window_start_sample"), pmt::from_long(0));
        d = pmt::dict_add(d, pmt::mp("pre_guard_samples"), pmt::from_long(0));
        d = pmt::dict_add(d, pmt::mp("capture_samples"),
                          pmt::from_long(static_cast<long>(y.size())));
        d = pmt::dict_add(d, pmt::mp("sample_count"),
                          pmt::from_long(static_cast<long>(y.size())));
        d = pmt::dict_add(d, pmt::mp("calibration_delay_native_samples"),
                          pmt::from_double(0.0));
        d = pmt::dict_add(d, pmt::mp("sensing_code"),
                          pmt::from_long(c_.sensing_code));
        d = pmt::dict_add(d, pmt::mp("pulse_id"), pmt::from_uint64(id));
        pmt::pmt_t pdu = pmt::cons(d, pmt::init_c32vector(y.size(), y.data()));
        rs_->_post(pmt::mp("packet"), pdu);
    }

    void post_consumer(uint64_t id, const std::vector<cplx>& out_iq)
    {
        pmt::pmt_t pdu =
            pmt::cons(base_meta(id),
                      pmt::init_c32vector(out_iq.size(), out_iq.data()));
        consumer_blk_->to_basic_block()->_post(pmt::mp(consumer_port()), pdu);
    }

    void wait_count(std::atomic<uint64_t>& ctr, uint64_t target,
                    double timeout_us, const char* what)
    {
        const auto t0 = std::chrono::steady_clock::now();
        while (ctr.load(std::memory_order_acquire) < target) {
            std::this_thread::sleep_for(std::chrono::microseconds(50));
            if (elapsed_us(t0, std::chrono::steady_clock::now()) > timeout_us)
                throw std::runtime_error(std::string("timeout waiting for ") +
                                         what);
        }
    }

    void verify(size_t widx, Outcome& out) const
    {
        const WindowTruth& t = c_.truths[widx];
        std::string sic_state = out.sic.sic_state;
        if (sic_state.empty())
            sic_state = "bypassed";
        if (sic_state != "committed") {
            out.effect_reason = "sic_state=" + sic_state;
            return;
        }
        if (!(out.sic.support_lo <= t.overlap_lo &&
              out.sic.support_hi >= t.overlap_hi)) {
            out.effect_reason = "partial_support[" +
                                std::to_string(out.sic.support_lo) + "," +
                                std::to_string(out.sic.support_hi) + "]";
            return;
        }
        if (!consumer_blk_) {
            out.downstream = "sic_only";
        } else if (o_.consumer == "cir") {
            out.downstream = out.cons.status.empty() ? "none" : out.cons.status;
            if (out.cons.status != "ok" || out.cons.valid_reps == 0) {
                out.effect_reason = "cir=" + out.downstream;
                return;
            }
        } else {
            out.downstream = out.cons.status.empty() ? "none" : out.cons.status;
            if (out.cons.status != "success" || !out.cons.fcs_pass) {
                out.effect_reason =
                    "demod=" + out.downstream +
                    (out.cons.status == "success" ? "/fcs" : "");
                return;
            }
            if (out.cons.has_psdu && out.cons.psdu_hex != c_.psdu_hex) {
                out.effect_reason = "psdu_mismatch";
                return;
            }
        }
        out.effect_ok = true;
    }

    const Options& o_;
    const Corpus& c_;
    gr::top_block_sptr tb_;
    UwbPreambleSic::sptr sic_;
    UwbPduRationalResamplerCcf65_32::sptr rs_;
    gr::basic_block_sptr consumer_blk_;
    UwbRadarCirEstimator::sptr cir_;
    UwbRealtimeDemodulator::sptr demod_;
    BenchSink::sptr sic_sink_ = BenchSink::make();
    BenchSink::sptr cons_sink_;
    size_t max_out_ = 0;

    // core-mode state
    sic::SicConfig core_cfg_;
    sic::SicScratch core_scratch_;
    sic::SicTemplates core_tpl_{};
    std::vector<cplx> core_out_;
    bool core_ready_ = false;
    double core_checksum_ = 0.0;
};

Outcome
Pipeline::submit(uint64_t id, size_t widx, double timeout_us)
{
    Outcome out;
    auto& sctr = sic_sink_->count;
    const uint64_t before_sic = sctr.load(std::memory_order_acquire);
    const uint64_t before_cons =
        consumer_blk_ ? cons_sink_->count.load(std::memory_order_acquire) : 0;
    const auto t0 = std::chrono::steady_clock::now();

    if (o_.mode == "core") {
        out.sic = run_core_one(c_.wins[widx]);
        const auto ts = std::chrono::steady_clock::now();
        out.sic_us = elapsed_us(t0, ts);
        if (consumer_blk_) {
            post_consumer(id, core_out_);
            wait_count(cons_sink_->count, before_cons + 1, timeout_us,
                       "consumer");
            const auto tc = std::chrono::steady_clock::now();
            out.consumer_us = elapsed_us(ts, tc);
            out.cons = cons_sink_->snapshot();
        }
    } else if (o_.mode == "gr") {
        post_sic(id, widx);
        wait_count(sctr, before_sic + 1, timeout_us, "sic out");
        const auto ts = std::chrono::steady_clock::now();
        out.sic_us = elapsed_us(t0, ts);
        out.sic = sic_sink_->snapshot();
        if (consumer_blk_) {
            wait_count(cons_sink_->count, before_cons + 1, timeout_us,
                       "consumer");
            const auto tc = std::chrono::steady_clock::now();
            out.consumer_us = elapsed_us(ts, tc);
            out.cons = cons_sink_->snapshot();
        }
    } else { // native
        post_native(id, widx);
        wait_count(sctr, before_sic + 1, timeout_us, "native sic out");
        const auto ts = std::chrono::steady_clock::now();
        out.sic_us = elapsed_us(t0, ts);
        out.sic = sic_sink_->snapshot();
        if (consumer_blk_) {
            wait_count(cons_sink_->count, before_cons + 1, timeout_us,
                       "consumer");
            const auto tc = std::chrono::steady_clock::now();
            out.consumer_us = elapsed_us(ts, tc);
            out.cons = cons_sink_->snapshot();
        }
    }
    out.wall_us = elapsed_us(t0, std::chrono::steady_clock::now());
    verify(widx, out);
    return out;
}

Snap
Pipeline::run_core_one(const std::vector<cplx>& y)
{
    if (!core_ready_) {
        sic::SicConfig cfg;
        cfg.mode = sic::SicMode::Estimate;
        cfg.comm_code = static_cast<size_t>(c_.comm_code);
        cfg.sync_reps = static_cast<size_t>(c_.sync_reps);
        cfg.samples_per_symbol = kSps;
        cfg.sample_rate = c_.rate;
        cfg.k_paths = static_cast<size_t>(c_.k_paths);
        cfg.fit_reps = static_cast<size_t>(c_.fit_reps);
        cfg.validation_reps = static_cast<size_t>(c_.validation_reps);
        cfg.roi_lo = c_.search_lo;
        cfg.roi_hi = c_.sub_hi;
        cfg.coarse_stride = c_.coarse_stride;
        cfg.post_tail = c_.post_tail;
        cfg.enable_fractional = true;
        cfg.enable_cfo = true;
        cfg.detect_threshold = c_.detect_threshold;
        cfg.validate_threshold = c_.validate_threshold;
        std::string err;
        if (!sic::prepare_sic_core(cfg, c_.comm_sync.data(), c_.comm_sync.size(),
                                   c_.comm_preamble.data(),
                                   c_.comm_preamble.size(), core_scratch_,
                                   &err))
            throw std::runtime_error("prepare_sic_core: " + err);
        core_cfg_ = cfg;
        core_tpl_.sync = c_.comm_sync.data();
        core_tpl_.sync_len = c_.comm_sync.size();
        core_tpl_.preamble = c_.comm_preamble.data();
        core_tpl_.preamble_len = c_.comm_preamble.size();
        core_out_.assign(static_cast<size_t>(c_.n), cplx(0.f, 0.f));
        core_ready_ = true;
    }
    sic::SicResult r;
    r.out = core_out_.data();
    // The adapter attaches i_hat only when publish_reconstruction=true; mirror
    // that here (the diagnostic i_hat writer is out of the timed path).
    r.i_hat = nullptr;
    if (!sic::sic_one(y.data(), static_cast<size_t>(c_.n), core_tpl_,
                      core_cfg_, core_scratch_, r, nullptr))
        throw std::runtime_error("sic_one returned false");
    Snap s;
    switch (r.state) {
    case sic::SicState::Committed: s.sic_state = "committed"; break;
    case sic::SicState::Bypassed: s.sic_state = "bypassed"; break;
    case sic::SicState::Failed: s.sic_state = "failed"; break;
    case sic::SicState::Ambiguous: s.sic_state = "ambiguous"; break;
    }
    s.has_sic_state = true;
    s.committed = r.committed;
    s.support_lo = r.support_lo;
    s.support_hi = r.support_hi;
    s.residual_corr = r.residual_corr;
    s.core_total_us = r.timings.core_total_us;
    core_checksum_ += static_cast<double>(std::abs(core_out_[c_.n / 2])) +
                      static_cast<double>(std::abs(core_out_[c_.n / 3]));
    return s;
}

// ---------------------------------------------------------------------------
// Timing loop + report
// ---------------------------------------------------------------------------
struct RunResult {
    Stats wall, sic, consumer;
    uint64_t completions = 0, effect_ok = 0, effect_fail = 0, warmup = 0;
    uint64_t term_committed = 0, term_bypassed = 0, term_failed = 0,
             term_ambiguous = 0;
    uint64_t downstream_ok = 0, fcs_pass = 0, psdu_match = 0, psdu_mismatch = 0;
    uint64_t miss1 = 0, miss500 = 0;
    size_t inflight_hwm = 0, queue_hwm = 0, consumer_queue_hwm = 0;
    double checksum = 0.0;
    double mean_adapter_core_us = 0.0;
    double mean_residual_corr = 0.0;
    std::map<std::string, uint64_t> fail_reasons;
    std::string first_fail;
};

RunResult
run_pipeline(const Options& o, const Corpus& c, Pipeline& pipe,
             std::ofstream& csv, double& cpu_out)
{
    RunResult res;
    const size_t nwin = c.count();
    const double timeout_us = 120e6;

    std::printf("warmup: 200 windows (excluded)\n");
    std::fflush(stdout);
    for (size_t w = 0; w < 200; ++w) {
        pipe.submit(100000000ull + w, w % nwin, timeout_us);
        ++res.warmup;
    }
    pipe.reset_sinks();

    std::vector<double> wall_v, sic_v, cons_v;
    double sum_adapter_core = 0.0, sum_resid = 0.0;
    uint64_t n_resid = 0;
    const double cpu0 = cpu_seconds();

    for (size_t rd = 0; rd < o.rounds; ++rd) {
        const size_t win_off = (rd * 23) % nwin;
        const auto round_t0 = std::chrono::steady_clock::now();
        for (size_t k = 0; k < o.windows; ++k) {
            const uint64_t id = (static_cast<uint64_t>(rd) << 40) + k + 1;
            const size_t widx = (k + win_off) % nwin;
            const auto t0 = std::chrono::steady_clock::now();
            Outcome out = pipe.submit(id, widx, timeout_us);
            const double wus = elapsed_us(t0, std::chrono::steady_clock::now());
            wall_v.push_back(wus);
            sic_v.push_back(out.sic_us);
            if (pipe.real_consumer())
                cons_v.push_back(out.consumer_us);
            if (wus > 1000.0)
                ++res.miss1;
            if (wus > 500.0)
                ++res.miss500;
            if (out.effect_ok)
                ++res.effect_ok;
            else {
                ++res.effect_fail;
                res.fail_reasons[out.effect_reason]++;
                if (res.first_fail.empty())
                    res.first_fail = "window=" + std::to_string(widx) +
                                     " round=" + std::to_string(rd) +
                                     " reason=" + out.effect_reason;
            }
            const std::string& st = out.sic.sic_state;
            if (st == "committed")
                ++res.term_committed;
            else if (st == "bypassed")
                ++res.term_bypassed;
            else if (st == "ambiguous")
                ++res.term_ambiguous;
            else
                ++res.term_failed;
            if (out.cons.status == "ok" || out.cons.status == "success")
                ++res.downstream_ok;
            if (out.cons.fcs_pass)
                ++res.fcs_pass;
            if (out.cons.has_psdu) {
                if (out.cons.psdu_hex == c.psdu_hex)
                    ++res.psdu_match;
                else
                    ++res.psdu_mismatch;
            }
            sum_adapter_core += static_cast<double>(out.sic.core_total_us);
            if (out.sic.has_sic_state) {
                sum_resid += out.sic.residual_corr;
                ++n_resid;
            }
            if ((k % 1000) == 0 && k > 0 &&
                elapsed_us(round_t0, std::chrono::steady_clock::now()) > 60e6) {
                const double m =
                    std::accumulate(wall_v.begin(), wall_v.end(), 0.0) /
                    static_cast<double>(wall_v.size());
                std::printf("      [round %zu] batch %zu/%zu running wall "
                            "mean=%.3f ms\n",
                            rd, k, o.windows, m / 1000.0);
                std::fflush(stdout);
            }
            csv << "pipeline," << o.mode << ',' << o.consumer << ','
                << (c.real_input ? "main-real-effect" : "synthetic") << ','
                << c.config_hash << ','
                << (widx < c.truths.size() ? c.truths[widx].sha256 : "") << ','
                << c.comm_preamble_sha << ',' << c.n << ',' << rd << ',' << widx
                << ',' << wus << ',' << out.sic_us << ',' << out.consumer_us
                << ',' << (out.sic.sic_state.empty() ? "none" : out.sic.sic_state)
                << ',' << out.sic.support_lo << ',' << out.sic.support_hi << ','
                << (widx < c.truths.size() ? c.truths[widx].overlap_lo : -1)
                << ','
                << (widx < c.truths.size() ? c.truths[widx].overlap_hi : -1)
                << ','
                << (out.downstream.empty() ? "none" : out.downstream) << ','
                << (out.cons.fcs_pass ? 1 : 0) << ','
                << (out.cons.has_psdu ? out.cons.psdu_hex : "") << ','
                << (out.effect_ok ? 1 : 0) << ','
                << csv_field(out.effect_reason) << '\n';
        }
    }
    cpu_out = cpu_seconds() - cpu0;

    res.wall = summarize(wall_v);
    res.sic = summarize(sic_v);
    res.consumer = summarize(cons_v);
    res.completions = res.wall.n;
    res.inflight_hwm = pipe.inflight_hwm();
    res.queue_hwm = pipe.queue_hwm();
    res.consumer_queue_hwm = pipe.consumer_queue_hwm();
    res.checksum = pipe.checksum();
    res.mean_adapter_core_us =
        res.completions ? sum_adapter_core / static_cast<double>(res.completions)
                        : 0.0;
    res.mean_residual_corr =
        n_resid ? sum_resid / static_cast<double>(n_resid) : 0.0;
    return res;
}

} // namespace

// ===========================================================================
int
main(int argc, char** argv)
{
    try {
        Options o;
        if (!parse_args(argc, argv, o))
            return 2;

        const std::string repo = find_repo_root();
        if (repo.empty())
            throw std::runtime_error(
                "cannot locate repo root (gr-uwb/apps/sic_preamble_sim.py)");
        std::string build_dir = o.build_dir;
        if (build_dir.empty()) {
            const std::string exe = self_exe_path();
            if (!exe.empty())
                build_dir = std::filesystem::path(exe)
                                .parent_path()
                                .parent_path()
                                .string();
            else
                build_dir = repo + "/gr-uwb/build-sic";
        }
        if (o.check_against.empty()) {
            const size_t slash = o.output.find_last_of('/');
            const std::string parent =
                (slash == std::string::npos) ? std::string(".")
                                             : o.output.substr(0, slash);
            o.check_against = parent + "/main_r07_v56";
        }
        if (o.inputs.empty() && o.input_source == "real") {
            const size_t slash = o.output.find_last_of('/');
            const std::string parent =
                (slash == std::string::npos) ? std::string(".")
                                             : o.output.substr(0, slash);
            o.inputs = parent + "/real_inputs";
        }

        ensure_private_output(o.output);

        std::printf("== S0 preamble-only SIC layered benchmark (R03) ==\n");
        std::printf("preset=%s mode=%s consumer=%s input_source=%s\n",
                    o.preset.c_str(), o.mode.c_str(), o.consumer.c_str(),
                    o.input_source.c_str());
        std::printf("windows=%zu rounds=%zu distinct=%zu fit=%d val=%d "
                    "output=%s\n",
                    o.windows, o.rounds, o.distinct, o.fit_reps,
                    o.validation_reps, o.output.c_str());
        if (o.input_source == "synthetic")
            std::printf("*** SYNTHETIC MICROBENCHMARK: pseudo-random +/-1 chips "
                        "+ 9-tap Gaussian pulse.  NOT production-profile "
                        "evidence (R03). ***\n");
        std::printf("single worker, serial, one window in flight\n");

        Corpus c;
        if (o.input_source == "real") {
            if (o.generate_inputs || !file_exists(o.inputs + "/inputs.txt"))
                run_generator(o, repo, build_dir, o.inputs);
            const auto t0 = std::chrono::steady_clock::now();
            c = load_real_corpus(o, o.inputs);
            const double load_s =
                1e-9 *
                static_cast<double>(
                    std::chrono::duration_cast<std::chrono::nanoseconds>(
                        std::chrono::steady_clock::now() - t0)
                        .count());
            std::printf(
                "real corpus: %zu windows n=%lld pre=%lld fit=%d val=%d "
                "search_roi=[%lld,%lld) core/sub_hi=%lld snr=%.1f isr=%.1f "
                "config_hash=%s\n",
                c.count(), static_cast<long long>(c.n),
                static_cast<long long>(c.pre), c.fit_reps, c.validation_reps,
                static_cast<long long>(c.search_lo),
                static_cast<long long>(c.search_hi),
                static_cast<long long>(c.sub_hi), c.snr_db, c.isr_db,
                c.config_hash.c_str());
            std::printf("  templates: sensing_sync=%s comm_sync=%s "
                        "comm_preamble=%s (mode=%s r07_pass=%s len=%zu)\n",
                        c.sensing_sync_sha.c_str(), c.comm_sync_sha.c_str(),
                        c.comm_preamble_sha.c_str(),
                        c.comm_template_mode.c_str(),
                        c.comm_template_r07_pass ? "true" : "false",
                        c.comm_preamble_len);
            std::printf("  generator module sha=%s\n  generator script sha=%s\n",
                        c.generator_module_sha256.c_str(),
                        c.generator_script_sha256.c_str());
            std::printf("  crosscheck vs %s: templates_match=%s "
                        "fixture_y_match=%s (%zu fixtures compared)\n",
                        c.reference_run.c_str(),
                        c.crosscheck_templates_match ? "true" : "false",
                        c.crosscheck_fixture_match ? "true" : "false",
                        c.crosscheck_fixture_windows);
            std::printf("  load+hash verify = %.2f s (not timed)\n", load_s);
        } else {
            c = build_synthetic_corpus(o);
            std::printf("synthetic corpus: %zu windows n=%lld (NOT evidence)\n",
                        c.count(), static_cast<long long>(c.n));
        }

        if (o.mode == "native") {
            const auto t0 = std::chrono::steady_clock::now();
            c.nat.resize(c.count());
            for (size_t w = 0; w < c.count(); ++w)
                c.nat[w] = decimate_to_native(c.wins[w]);
            const double s =
                1e-9 *
                static_cast<double>(
                    std::chrono::duration_cast<std::chrono::nanoseconds>(
                        std::chrono::steady_clock::now() - t0)
                        .count());
            std::printf("native input derivation (work->native PCHIP "
                        "decimation, APPROXIMATION; R08 native generator open): "
                        "%zu x %zu samples in %.2f s (not timed)\n",
                        c.nat.size(), c.nat[0].size(), s);
        }

        const std::string csv_path =
            o.output + "/" + o.mode + "_" + o.consumer + "_windows.csv";
        std::ofstream csv(csv_path, std::ios::trunc);
        if (!csv)
            throw std::runtime_error("cannot open " + csv_path);
        csv << "pipeline,mode,consumer,case_id,config_hash,input_sha256,"
               "template_sha256,input_len,round,window,wall_us,sic_us,"
               "consumer_us,sic_state,support_lo,support_hi,overlap_lo,"
               "overlap_hi,downstream_status,fcs_pass,psdu_hex,effect_ok,"
               "effect_reason\n";

        Pipeline pipe(o, c);
        pipe.start();
        double cpu = 0.0;
        RunResult res = run_pipeline(o, c, pipe, csv, cpu);
        pipe.stop();

        const char* pipeline_name = nullptr;
        if (o.input_source == "synthetic")
            pipeline_name = "synthetic_microbenchmark";
        else if (o.consumer == "sic")
            pipeline_name = "sic_only";
        else if (o.consumer == "cir")
            pipeline_name = "sic_to_radar_cir";
        else
            pipeline_name = "sic_to_full_demod";

        std::printf("\n== perf table (pipeline=%s, mode=%s) ==\n",
                    pipeline_name, o.mode.c_str());
        std::printf("  case=%s input_len=%lld config_hash=%s input_sha=%s "
                    "tpl_sha=%s\n",
                    c.real_input ? "main-real-effect" : "synthetic",
                    static_cast<long long>(c.n), c.config_hash.c_str(),
                    c.truths.empty() ? "-" : c.truths[0].sha256.c_str(),
                    c.comm_preamble_sha.empty() ? "-" : c.comm_preamble_sha.c_str());
        print_row("wall (submit->end)", res.wall);
        print_row("sic step", res.sic);
        if (pipe.real_consumer())
            print_row("consumer step", res.consumer);

        const double wps = res.wall.mean_us > 0 ? 1e6 / res.wall.mean_us : 0;
        std::printf("  windows/round=%zu rounds=%zu total=%zu  %.1f win/s  "
                    "deadline misses: >1ms=%llu >0.5ms=%llu\n",
                    o.windows, o.rounds, res.wall.n, wps,
                    static_cast<unsigned long long>(res.miss1),
                    static_cast<unsigned long long>(res.miss500));
        std::printf("  host: nproc=%u loadavg_1m=%.2f (shared machine: report the "
                    "load with the row)\n",
                    std::thread::hardware_concurrency(), loadavg_1m());
        std::printf("  terminal: committed=%llu bypassed=%llu failed=%llu "
                    "ambiguous=%llu | downstream_ok=%llu fcs_pass=%llu "
                    "psdu_match=%llu psdu_mismatch=%llu consumer_dropped=%llu\n",
                    static_cast<unsigned long long>(res.term_committed),
                    static_cast<unsigned long long>(res.term_bypassed),
                    static_cast<unsigned long long>(res.term_failed),
                    static_cast<unsigned long long>(res.term_ambiguous),
                    static_cast<unsigned long long>(res.downstream_ok),
                    static_cast<unsigned long long>(res.fcs_pass),
                    static_cast<unsigned long long>(res.psdu_match),
                    static_cast<unsigned long long>(res.psdu_mismatch),
                    static_cast<unsigned long long>(pipe.consumer_jobs_dropped()));
        const bool verified = (res.effect_fail == 0 && res.effect_ok > 0);
        std::printf("  effect: ok=%llu fail=%llu  verified=%s\n",
                    static_cast<unsigned long long>(res.effect_ok),
                    static_cast<unsigned long long>(res.effect_fail),
                    verified ? "true" : "false");
        if (!res.first_fail.empty())
            std::printf("  first effect failure: %s\n", res.first_fail.c_str());
        std::printf("  hwm: sic_inflight=%zu sic_queue=%zu consumer_queue=%zu "
                    "checksum=%.6f cpu=%.2fs rss=%.1fMB "
                    "mean_adapter_core=%.1f us mean_resid_corr=%.4f\n",
                    res.inflight_hwm, res.queue_hwm, res.consumer_queue_hwm,
                    res.checksum, cpu, rss_mb(), res.mean_adapter_core_us,
                    res.mean_residual_corr);
        for (const auto& kv : res.fail_reasons)
            std::printf("  effect failure reason: %-32s %llu\n",
                        kv.first.c_str(),
                        static_cast<unsigned long long>(kv.second));

        // ---- JSON ---------------------------------------------------------
        std::ostringstream json;
        const GitInfo git = git_info(repo);
        json << std::setprecision(6);
        json << "{\n  \"schema\":\"sic_perf_v2\",\n  \"mode\":\"" << o.mode
             << "\",\n  \"consumer\":\"" << o.consumer
             << "\",\n  \"pipeline\":\"" << pipeline_name
             << "\",\n  \"input_source\":\"" << o.input_source
             << "\",\n  \"production_profile\":"
             << ((c.real_input && o.mode != "native") ? "true" : "false")
             << ",\n  \"official_evidence\":"
             << ((c.real_input && o.mode != "native") ? "true" : "false")
             << ",\n  \"windows\":" << o.windows << ",\n  \"rounds\":" << o.rounds
             << ",\n  \"distinct\":" << c.count() << ",\n  \"n_work\":" << c.n
             << ",\n  \"config_hash\":\"" << c.config_hash << "\",\n"
             << "  \"templates\":{\"sensing_sync_code9_sha256\":\""
             << c.sensing_sync_sha << "\",\"comm_sync_code10_sha256\":\""
             << c.comm_sync_sha << "\",\"comm_preamble_code10_sha256\":\""
             << c.comm_preamble_sha << "\",\"comm_preamble_length\":"
             << c.comm_preamble_len << ",\"comm_template_mode\":\""
             << c.comm_template_mode << "\",\"r07_pass\":"
             << (c.comm_template_r07_pass ? "true" : "false") << "},\n"
             << "  \"input_windows\":[";
        for (size_t i = 0; i < c.count(); ++i) {
            const WindowTruth& t = c.truths[i];
            json << (i ? "," : "") << "{\"index\":" << i
                 << ",\"seed\":" << t.seed << ",\"length\":" << t.length
                 << ",\"sha256\":\"" << t.sha256 << "\",\"overlap_lo\":"
                 << t.overlap_lo << ",\"overlap_hi\":" << t.overlap_hi
                 << ",\"isr_db\":";
            if (std::isnan(t.isr_db))
                json << "null";
            else
                json << t.isr_db;
            json << "}";
        }
        json << "],\n  \"provenance\":{\"repo_root\":\"" << json_escape(repo)
             << "\",\"build_dir\":\"" << json_escape(build_dir)
             << "\",\"git_head\":\"" << git.head << "\",\"git_branch\":\""
             << git.branch << "\",\"git_dirty\":"
             << (git.status.empty() ? "false" : "true")
             << ",\"git_status_sha256\":\"" << git.status_sha
             << "\",\"benchmark_exe_sha256\":\"" << shell_sha256(self_exe_path())
             << "\",\"benchmark_source_sha256\":\""
             << shell_sha256(repo + "/gr-uwb/apps/benchmark_preamble_sic.cc")
             << "\",\"libgnuradio_uwb_path\":\""
             << json_escape(loaded_lib_path("libgnuradio-uwb.so"))
             << "\",\"libgnuradio_uwb_sha256\":\""
             << shell_sha256(loaded_lib_path("libgnuradio-uwb.so"))
             << "\",\"generator_module\":\"" << json_escape(c.generator_module)
             << "\",\"generator_module_sha256\":\"" << c.generator_module_sha256
             << "\",\"generator_script_sha256\":\""
             << c.generator_script_sha256 << "\",\"reference_run\":\""
             << json_escape(c.reference_run)
             << "\",\"crosscheck_templates_match\":"
             << (c.crosscheck_templates_match ? "true" : "false")
             << ",\"crosscheck_fixture_y_match\":"
             << (c.crosscheck_fixture_match ? "true" : "false")
             << ",\"crosscheck_fixture_windows\":"
             << c.crosscheck_fixture_windows << "},\n";
        json << "  \"resolved_config\":{\"preset\":\"" << c.preset
             << "\",\"sensing_code\":" << c.sensing_code << ",\"comm_code\":"
             << c.comm_code << ",\"sync_reps\":" << c.sync_reps
             << ",\"fit_reps\":" << c.fit_reps << ",\"validation_reps\":"
             << c.validation_reps << ",\"k_paths\":" << c.k_paths
             << ",\"snr_db\":" << c.snr_db << ",\"isr_db\":" << c.isr_db
             << ",\"pre_guard\":" << c.pre << ",\"search_roi\":[" << c.search_lo
             << "," << c.search_hi << "],\"subtraction_roi\":[" << c.sub_lo
             << "," << c.sub_hi << "],\"detect_threshold\":"
             << c.detect_threshold << ",\"validate_threshold\":"
             << c.validate_threshold << ",\"coarse_stride\":" << c.coarse_stride
             << ",\"post_tail\":" << c.post_tail << ",\"psdu_hex\":\""
             << c.psdu_hex << "\",\"pulse_shape\":\"" << c.pulse_shape
             << "\",\"pulse_taps\":" << c.pulse_taps << ",\"frac_model\":\""
             << c.frac_model << "\"},\n";
        json << "  \"native_input_derivation\":\""
             << (o.mode == "native"
                     ? "work->native PCHIP decimation then production PDU 65/32 "
                       "(approximation; R08 native generator open; NOT native "
                       "acceptance evidence)"
                     : "n/a")
             << "\",\n";
        json << "  \"stats\":{\"wall\":";
        stats_obj(json, res.wall);
        json << ",\"sic\":";
        stats_obj(json, res.sic);
        json << ",\"consumer\":";
        stats_obj(json, res.consumer);
        json << "},\n  \"terminal\":{\"completions\":" << res.completions
             << ",\"committed\":" << res.term_committed << ",\"bypassed\":"
             << res.term_bypassed << ",\"failed\":" << res.term_failed
             << ",\"ambiguous\":" << res.term_ambiguous
             << ",\"downstream_ok\":" << res.downstream_ok << ",\"fcs_pass\":"
             << res.fcs_pass << ",\"psdu_match\":" << res.psdu_match
             << ",\"psdu_mismatch\":" << res.psdu_mismatch
             << ",\"consumer_dropped\":" << pipe.consumer_jobs_dropped()
             << "},\n";
        json << "  \"effect\":{\"ok\":" << res.effect_ok
             << ",\"fail\":" << res.effect_fail << ",\"verified\":"
             << (verified ? "true" : "false") << ",\"first_failure\":\""
             << json_escape(res.first_fail) << "\",\"reasons\":{";
        bool first = true;
        for (const auto& kv : res.fail_reasons) {
            json << (first ? "" : ",") << "\"" << json_escape(kv.first)
                 << "\":" << kv.second;
            first = false;
        }
        json << "}},\n";
        json << "  \"resources\":{\"cpu_seconds\":" << cpu
             << ",\"rss_mb\":" << rss_mb() << ",\"sic_inflight_hwm\":"
             << res.inflight_hwm << ",\"sic_queue_hwm\":" << res.queue_hwm
             << ",\"consumer_queue_hwm\":" << res.consumer_queue_hwm
             << ",\"consumer_service_mean_us\":" << pipe.consumer_service_mean_us()
             << ",\"deadline_miss_1ms\":" << res.miss1
             << ",\"deadline_miss_500us\":" << res.miss500
             << ",\"win_per_s\":" << wps << ",\"checksum\":" << res.checksum
             << ",\"mean_adapter_core_us\":" << res.mean_adapter_core_us
             << ",\"mean_residual_corr\":" << res.mean_residual_corr
             << ",\"warmup_windows\":" << res.warmup
             << ",\"loadavg_1m\":" << loadavg_1m()
             << ",\"nproc\":" << std::thread::hardware_concurrency()
             << "},\n";
        json << "  \"deadline_target\":{\"mean_us\":500.0,\"p99_us\":1000.0,"
                "\"win_per_s\":200},\n";
        json << "  \"superseded\":[{\"runs\":[\"core*\",\"core_iter*\",\"gr*\","
                "\"native*\",\"main_est\",\"main_iter*\",\"main_opt1\","
                "\"effect_iter5*\",\"effect_reviewfix*\",\"paced200\","
                "\"baseline*\"],\"reason\":\"old synthetic-input and/or "
                "pre-R07-template and/or non-production-profile benchmark "
                "numbers; the R03 official rows are the gr-uwb/build-sic/"
                "sic-results/real_* tables (real effect-set corpus with hash "
                "binding)\",\"production_profile_input\":"
             << ((c.real_input && o.mode != "native") ? "true" : "false")
             << "}]\n}\n";

        csv.close();
        const std::string json_path =
            o.output + "/" + o.mode + "_" + o.consumer + "_summary.json";
        std::ofstream jf(json_path, std::ios::trunc);
        jf << json.str();
        jf.close();
        std::printf("\nwrote %s and %s\n", csv_path.c_str(), json_path.c_str());

        if (c.real_input && o.mode == "native" && res.effect_fail > 0) {
            std::fprintf(
                stderr,
                "benchmark_preamble_sic: NATIVE mode effect not verified "
                "(%llu/%llu windows): the work->native PCHIP approximation does "
                "not reproduce the effect-set overlap/coordinates.  This is a "
                "labelled APPROXIMATION (the native generator CLI, R08, is "
                "still open) and the native rows are NOT native acceptance "
                "evidence.  Rows are still written for diagnosis.\n",
                static_cast<unsigned long long>(res.effect_fail),
                static_cast<unsigned long long>(res.effect_ok + res.effect_fail));
            return 4;
        }
        if (c.real_input && res.effect_fail > 0) {
            std::fprintf(stderr,
                         "benchmark_preamble_sic: EFFECT VERIFICATION FAILED "
                         "(%llu/%llu windows): the measured rows include a "
                         "bypass / partial-subtraction / failed consumer path "
                         "and cannot be quoted as full-processing speed.\n",
                         static_cast<unsigned long long>(res.effect_fail),
                         static_cast<unsigned long long>(res.effect_ok +
                                                        res.effect_fail));
            return 3;
        }
        if (c.real_input && res.effect_ok == 0) {
            std::fprintf(stderr,
                         "benchmark_preamble_sic: no window was effect-verified "
                         "(measured workload is not bound to a passing case)\n");
            return 3;
        }
        return 0;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "benchmark_preamble_sic: %s\n", e.what());
        return 1;
    }
}
