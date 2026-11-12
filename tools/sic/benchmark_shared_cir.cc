// ABBA-paired, same-process, same-build benchmark of the shared SYNC-CIR
// kernel and the shared preamble front-end against the pre-refactor sensing
// sequence.
//
// Workstream D of
// docs/performance/OpenCode_Sensing与Comm统一前端和CIR_开发方案_2026-11-12.md
// §8 ("性能验收").  Nothing here is a GNU Radio block and nothing needs the
// GNU Radio runtime: every header used is header-only and the only external
// dependency is libvolk.
//
// ---------------------------------------------------------------------------
// Arms
// ---------------------------------------------------------------------------
//   control   the EXACT pre-refactor sensing sequence, still present in the
//             tree, so the comparison needs no cross-build noise:
//               radar::search_sfd
//               -> radar::refine_sync_origin
//               -> radar::estimate_radar_cir   (scalar, 116 taps, double
//                                               accumulation, dense 1016-entry
//                                               code scan, NO CFO compensation)
//             The CIR axis is the public TX-time prediction
//             `nominal_preamble_start(predicted_sfd, sync_reps, sps)`.
//   candidate radar::radar_cir_one with compensate_target_cfo = true, i.e. the
//             new production path: shared preamble front-end (detection, CFO
//             estimate, final-CFO rotation over the CIR union) + the shared
//             sync_cir::estimate_sync_cir kernel.
//
// Both arms run on the SAME raw window, in the same process, in the same
// binary, interleaved ABBA.  The only difference is the algorithm.
//
// ---------------------------------------------------------------------------
// Fixture
// ---------------------------------------------------------------------------
// A REAL modulated packet, not a synthetic correlation peak:
//   mod::modulate_one(code 9, SFD 4z2, Legacy 48-tap pulse, 998.4 MS/s)
// placed in a zero window with a `pre_guard` lead-in, exactly the geometry
// gr-uwb/lib/qa_uwb_radar_e2e.cc uses:
//   predicted_sfd = pre_guard + sync_reps * 1016
//   cir_axis      = predicted_sfd - sync_reps * 1016 = pre_guard
// so a physical delay would appear as a tap shift and is never re-zeroed.
//
// Two configurations: 64 SYNC / 32-byte PSDU and 256 SYNC / 127-byte PSDU.
//
// ---------------------------------------------------------------------------
// Correctness guard (runs BEFORE any timing)
// ---------------------------------------------------------------------------
// Both arms must succeed on the fixture, agree on peak_tap on the 116-tap
// grid, agree on valid_repetitions, and the raw CIR relative L2 difference
// must be <= 2e-5.  Otherwise the process aborts with a clear message and
// prints NO timings: a failing or bypassed path is never used as a baseline.
//
// ---------------------------------------------------------------------------
// Protocol
// ---------------------------------------------------------------------------
//   * exclusive flock on /tmp/uwb-sic-benchmark.lock for the whole run
//   * >= 32 warm-up iterations per arm
//   * >= 512 valid observations per arm per session
//   * 5 ABBA groups per session (A B B A), N iterations per slot
//   * 3 independent sessions = 3 separate process invocations
//   * per-session summary + combined summary, CSV to stdout and to --csv
//
// Every observation is validated (status, tap count, peak) before it is
// accepted; a single invalid observation aborts the session.

#include <gnuradio/uwb/uwb_demod_core.h>
#include <gnuradio/uwb/uwb_demod_result.h>
#include <gnuradio/uwb/uwb_hrp_mod_core.h>
#include <gnuradio/uwb/uwb_phy_profile.h>
#include <gnuradio/uwb/uwb_radar_cir_core.h>
#include <gnuradio/uwb/uwb_radar_cir_estimator.h>
#include <gnuradio/uwb/uwb_radar_sfd_core.h>
#include <gnuradio/uwb/uwb_radar_timing_core.h>
#include <gnuradio/uwb/uwb_sync_cir_core.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <sstream>
#include <string>
#include <sys/file.h>
#include <sys/stat.h>
#include <sys/sysinfo.h>
#include <unistd.h>
#include <vector>

using cf = std::complex<float>;
using Clock = std::chrono::steady_clock;

namespace {

// ---------------------------------------------------------------------------
// Small utilities
// ---------------------------------------------------------------------------

double elapsed_us(Clock::time_point a, Clock::time_point b)
{
    return std::chrono::duration<double, std::micro>(b - a).count();
}

[[noreturn]] void die(const std::string& why)
{
    std::cerr << "benchmark_shared_cir: FATAL: " << why << "\n";
    std::exit(2);
}

std::string read_cmd(const char* cmd)
{
    std::string out;
    FILE* p = ::popen(cmd, "r");
    if (!p)
        return out;
    char buf[512];
    while (std::fgets(buf, sizeof(buf), p))
        out += buf;
    ::pclose(p);
    while (!out.empty() && (out.back() == '\n' || out.back() == '\r'))
        out.pop_back();
    return out;
}

uint64_t fnv1a(const void* data, size_t bytes)
{
    const auto* p = static_cast<const uint8_t*>(data);
    uint64_t h = 1469598103934665603ull;
    for (size_t i = 0; i < bytes; ++i) {
        h ^= p[i];
        h *= 1099511628211ull;
    }
    return h;
}

std::vector<cf> load_cf32(const std::string& path)
{
    std::ifstream f(path, std::ios::binary | std::ios::ate);
    if (!f)
        return {};
    const auto bytes = static_cast<size_t>(f.tellg());
    if (bytes == 0 || bytes % sizeof(cf) != 0)
        return {};
    f.seekg(0);
    std::vector<cf> v(bytes / sizeof(cf));
    f.read(reinterpret_cast<char*>(v.data()), static_cast<std::streamsize>(bytes));
    if (!f)
        return {};
    return v;
}

double rel_l2(const cf* a, const cf* b, size_t n)
{
    double num = 0.0;
    double den = 0.0;
    for (size_t i = 0; i < n; ++i) {
        const cf d = a[i] - b[i];
        num += static_cast<double>(d.real()) * d.real() +
               static_cast<double>(d.imag()) * d.imag();
        den += static_cast<double>(b[i].real()) * b[i].real() +
               static_cast<double>(b[i].imag()) * b[i].imag();
    }
    if (den == 0.0)
        return num == 0.0 ? 0.0 : 1.0;
    return std::sqrt(num / den);
}

struct Stats {
    double mean = 0.0, p50 = 0.0, p95 = 0.0, min = 0.0, max = 0.0;
    size_t n = 0;
};

Stats summarise(std::vector<double> v)
{
    Stats s;
    if (v.empty())
        return s;
    std::sort(v.begin(), v.end());
    double sum = 0.0;
    for (double x : v)
        sum += x;
    s.n = v.size();
    s.mean = sum / static_cast<double>(v.size());
    s.p50 = v[v.size() / 2];
    // Nearest-rank P95, identical to tools/sic/run_chain_workers.py.
    size_t idx = static_cast<size_t>(std::ceil(0.95 * static_cast<double>(v.size())));
    if (idx == 0)
        idx = 1;
    if (idx > v.size())
        idx = v.size();
    s.p95 = v[idx - 1];
    s.min = v.front();
    s.max = v.back();
    return s;
}

// ---------------------------------------------------------------------------
// Fixture
// ---------------------------------------------------------------------------

struct Fixture {
    std::string name;
    size_t sync_reps = 0;
    size_t psdu_bytes = 0; // total PSDU length INCLUDING the 2-byte FCS
    size_t pre_guard = 0;
    size_t packet_samples = 0;
    int64_t predicted_sfd = -1;
    int64_t cir_axis = -1;
    std::vector<cf> window;      // pre_guard zeros + packet + tail zeros
    std::vector<cf> sync_pulse;  // raw pulse-shaped SYNC symbol, 1016 samples
    std::vector<uint8_t> psdu;
    uint64_t window_hash = 0;
    uint64_t psdu_hash = 0;
    double template_rel_l2 = -1.0; // testdata template vs modulated SYNC head
};

constexpr size_t kSps = gr::uwb::demod::kQm35SamplesPerSymbol; // 1016
constexpr size_t kCirPre = 16;
constexpr size_t kCirPost = 100;
constexpr size_t kTaps = kCirPre + kCirPost; // 116
constexpr size_t kCirSkip = 10;
constexpr size_t kPreGuard = 1997; // same lead-in as qa_uwb_radar_e2e.cc
constexpr size_t kTail = 4096;
constexpr double kSampleRateHz = 998.4e6;

Fixture build_fixture(const std::string& name,
                      size_t sync_reps,
                      size_t psdu_bytes,
                      const std::string& testdata_dir)
{
    Fixture fx;
    fx.name = name;
    fx.sync_reps = sync_reps;
    fx.psdu_bytes = psdu_bytes;
    fx.pre_guard = kPreGuard;

    gr::uwb::mod::HrpModConfig mcfg;
    mcfg.code_index = 9;
    mcfg.sync_repetitions = sync_reps;
    mcfg.sfd_mode = "4z2";
    // Legacy 48-tap pulse + the default 0.8 peak amplitude: the same modulator
    // settings the production TX uses for this profile.

    const auto sfd_seq = gr::uwb::demod::GetSfdSequence("4z2");
    if (sfd_seq.empty())
        die("SFD sequence 4z2 is unavailable");

    const size_t cap = gr::uwb::mod::packet_samples_998p4(
        sync_reps, sfd_seq.size(), psdu_bytes, false, &mcfg.pulse);
    if (cap == 0)
        die("packet_samples_998p4 returned 0");

    gr::uwb::mod::HrpModScratch mscratch;
    mscratch.reserve(psdu_bytes, cap);

    // Deterministic PSDU: `psdu_bytes - 2` random data bytes + IEEE FCS.
    if (psdu_bytes < 2)
        die("PSDU must be at least 2 bytes (FCS)");
    fx.psdu = gr::uwb::mod::make_random_psdu(psdu_bytes - 2, 0x5EED1234u, true);
    if (fx.psdu.size() != psdu_bytes)
        die("PSDU length mismatch");

    std::vector<cf> packet(cap + 64, cf(0.f, 0.f));
    size_t packet_n = 0;
    if (!gr::uwb::mod::modulate_one(fx.psdu.data(), fx.psdu.size(), mcfg, mscratch,
                                    packet.data(), packet.size(), packet_n))
        die("modulate_one failed for " + name);
    if (packet_n == 0 || packet_n > cap + 64)
        die("modulate_one produced an invalid length for " + name);
    packet.resize(packet_n);
    fx.packet_samples = packet_n;

    // The pulse-shaped SYNC symbol: first 1016 samples of the modulated
    // packet.  Cross-checked against the frozen testdata template so a
    // modulator/pulse regression cannot silently change the fixture.
    fx.sync_pulse.assign(packet.begin(), packet.begin() + static_cast<long>(kSps));
    const auto frozen =
        load_cf32(testdata_dir + "/uwb_radar/sync_template_998p4.cf32");
    if (frozen.size() == kSps)
        fx.template_rel_l2 = rel_l2(fx.sync_pulse.data(), frozen.data(), kSps);
    else
        fx.template_rel_l2 = -1.0; // testdata absent: reported, not fatal

    fx.window.assign(fx.pre_guard + packet_n + kTail, cf(0.f, 0.f));
    std::copy(packet.begin(), packet.end(),
              fx.window.begin() + static_cast<long>(fx.pre_guard));

    fx.predicted_sfd = static_cast<int64_t>(fx.pre_guard) +
                       static_cast<int64_t>(sync_reps) * static_cast<int64_t>(kSps);
    fx.cir_axis = gr::uwb::radar::nominal_preamble_start(fx.predicted_sfd, sync_reps, kSps);
    if (fx.cir_axis < 0)
        die("nominal_preamble_start failed for " + name);
    if (fx.cir_axis != static_cast<int64_t>(fx.pre_guard))
        die("CIR axis is not the TX-time SYNC origin for " + name);

    fx.window_hash = fnv1a(fx.window.data(), fx.window.size() * sizeof(cf));
    fx.psdu_hash = fnv1a(fx.psdu.data(), fx.psdu.size());
    return fx;
}

// ---------------------------------------------------------------------------
// Control arm: the exact pre-refactor sensing sequence.
// ---------------------------------------------------------------------------

struct ControlArm {
    gr::uwb::radar::RadarSfdScratch sfd;
    gr::uwb::radar::RadarCirScratch cir;
    std::vector<cf> sync_template; // L2-normalized, 1016

    struct Timing {
        double total_us = 0.0;
        double sfd_us = 0.0;
        double refine_us = 0.0;
        double cir_us = 0.0;
        size_t peak_tap = 0;
        size_t valid_repetitions = 0;
        int64_t sfd_start = -1;
        int64_t origin = -1;
        int64_t cir_axis = -1;
        float sfd_metric = 0.f;
        float sync_metric = 0.f;
    };

    bool prepare(const Fixture& fx)
    {
        const auto sfd_seq = gr::uwb::demod::GetSfdSequence("4z2");
        if (sfd_seq.empty())
            return false;
        if (!gr::uwb::radar::prepare_sfd_template(sfd_seq.data(), sfd_seq.size(),
                                                  fx.sync_pulse.data(), kSps, sfd))
            return false;
        if (!gr::uwb::radar::prepare_radar_cir_code(
                gr::uwb::demod::GetPreambleCode(9), gr::uwb::demod::kQm35CodeLength,
                kCirPre, kCirPost, cir))
            return false;
        sync_template = fx.sync_pulse;
        gr::uwb::core::uwb_l2_normalize(sync_template);
        return true;
    }

    bool run(const Fixture& fx, Timing& t)
    {
        const auto* rx = fx.window.data();
        const size_t n = fx.window.size();
        const size_t reps = fx.sync_reps;
        const size_t count = reps - kCirSkip;

        const auto t0 = Clock::now();
        gr::uwb::radar::RadarSfdResult sfd_res;
        if (!gr::uwb::radar::search_sfd(rx, n, fx.predicted_sfd, 64, 0.3f, sfd_res,
                                        sfd))
            return false;
        const auto t1 = Clock::now();
        gr::uwb::radar::RadarTimingResult timing;
        if (!gr::uwb::radar::refine_sync_origin(
                rx, n, sfd_res.sfd_start_sample, reps, kSps, 8, 0.3f,
                sync_template.data(), sync_template.size(), timing))
            return false;
        const auto t2 = Clock::now();
        gr::uwb::radar::RadarCirEstimate est;
        if (!gr::uwb::radar::estimate_radar_cir(rx, n, fx.cir_axis, kSps, kCirPre,
                                                kCirPost, kCirSkip, count, reps, est,
                                                cir))
            return false;
        const auto t3 = Clock::now();

        t.total_us = elapsed_us(t0, t3);
        t.sfd_us = elapsed_us(t0, t1);
        t.refine_us = elapsed_us(t1, t2);
        t.cir_us = elapsed_us(t2, t3);
        t.peak_tap = est.peak_tap;
        t.valid_repetitions = est.valid_repetitions;
        t.sfd_start = sfd_res.sfd_start_sample;
        t.origin = timing.preamble_start_sample;
        t.cir_axis = fx.cir_axis;
        t.sfd_metric = sfd_res.metric;
        t.sync_metric = timing.metric;
        return est.tap_count == kTaps;
    }
};

// ---------------------------------------------------------------------------
// Candidate arm: the new production sensing entry point.
// ---------------------------------------------------------------------------

struct CandidateArm {
    gr::uwb::radar::RadarCirConfig cfg;
    gr::uwb::radar::RadarCirCoreScratch scratch;
    // Second scratch for the kernel-only measurement so the production
    // published taps are never disturbed by the diagnostic call.
    gr::uwb::demod::sync_cir::SyncCirScratch kernel_scratch;
    gr::uwb::demod::sync_cir::PreparedSyncCode kernel_code;

    struct Timing {
        double total_us = 0.0;
        double kernel_us = 0.0; // shared kernel alone, high resolution
        uint64_t frontend_timing_us = 0;
        uint64_t frontend_cfo_us = 0;
        uint64_t frontend_sfd_us = 0;
        uint64_t coverage_us = 0;
        uint64_t cir_us = 0; // RadarCirResult::cir_us (1 us resolution)
        size_t peak_tap = 0;
        size_t valid_repetitions = 0;
        int64_t sfd_start = -1;
        int64_t origin = -1;
        int64_t cir_axis = -1;
        double cfo_hz = 0.0;
        uint32_t cfo_epoch = 0;
        double period = 0.0;
    };

    bool prepare(const Fixture& fx)
    {
        cfg = gr::uwb::radar::RadarCirConfig{};
        cfg.code_index = 9;
        cfg.sync_repetitions = fx.sync_reps;
        cfg.samples_per_symbol = kSps;
        cfg.sfd_mode = "4z2";
        cfg.sfd_search_margin = 64;
        cfg.sync_refine_margin = 8;
        cfg.sfd_threshold = 0.3f;
        cfg.sync_refine_threshold = 0.3f;
        cfg.cir_pre = kCirPre;
        cfg.cir_post = kCirPost;
        cfg.cir_skip_initial = kCirSkip;
        cfg.cir_repetitions = fx.sync_reps - kCirSkip;
        cfg.use_predicted_timing = false;
        cfg.compensate_target_cfo = true; // the new production path
        cfg.target_cfo_known = false;

        if (!gr::uwb::radar::prepare_radar_cir_core(cfg, fx.sync_pulse.data(), kSps,
                                                    kCirPre, kCirPost, scratch))
            return false;
        gr::uwb::radar::reserve_radar_cir_core(fx.window.size(), scratch);

        // Independent scratch for the kernel-only arm: same code, same
        // capacity, so the diagnostic call cannot perturb production state.
        if (!gr::uwb::demod::sync_cir::prepare_sync_code(
                gr::uwb::demod::GetPreambleCode(9),
                gr::uwb::demod::sync_cir::kSyncCirCodeLength, 9, kCirPre, kCirPost,
                kernel_code, kernel_scratch))
            return false;
        return true;
    }

    bool run(const Fixture& fx, Timing& t, gr::uwb::radar::RadarCirResult& res)
    {
        const auto t0 = Clock::now();
        const bool ok = gr::uwb::radar::radar_cir_one(
            fx.window.data(), fx.window.size(), fx.predicted_sfd, cfg, scratch, res);
        t.total_us = elapsed_us(t0, Clock::now());
        if (!ok)
            return false;

        t.frontend_timing_us = res.frontend_timing_us;
        t.frontend_cfo_us = res.frontend_cfo_us;
        t.frontend_sfd_us = res.frontend_sfd_us;
        t.coverage_us = res.coverage_us;
        t.cir_us = res.cir_us;
        t.peak_tap = res.peak_tap;
        t.valid_repetitions = res.valid_repetitions;
        t.sfd_start = res.sfd_start_sample;
        t.origin = res.detected_sync_origin;
        t.cir_axis = res.cir_axis_origin;
        t.cfo_hz = res.final_cfo_hz;
        t.cfo_epoch = res.cfo_epoch;
        t.period = res.cir_samples_per_symbol;

        // Kernel-only measurement on exactly the buffer production consumed.
        gr::uwb::demod::sync_cir::SyncCirRequest req;
        req.cir_axis_origin = res.cir_axis_origin;
        req.samples_per_symbol = res.cir_samples_per_symbol;
        req.total_repetitions = fx.sync_reps;
        req.skip_initial = kCirSkip;
        req.max_repetitions = fx.sync_reps - kCirSkip;
        req.pre = kCirPre;
        req.post = kCirPost;
        const auto* derot = scratch.demod.derotated.data();
        gr::uwb::demod::sync_cir::SyncCirResult cir_res;
        const auto k0 = Clock::now();
        const bool kernel_ok = gr::uwb::demod::sync_cir::estimate_sync_cir(
            derot, fx.window.size(), req, kernel_code, kernel_scratch, cir_res);
        t.kernel_us = elapsed_us(k0, Clock::now());
        if (!kernel_ok || cir_res.tap_count != kTaps ||
            cir_res.peak_tap != res.peak_tap)
            return false;
        return res.tap_count == kTaps;
    }
};

// ---------------------------------------------------------------------------
// Comm arm (regression probe): demodulate_one on the 64/32 fixture.
// ---------------------------------------------------------------------------

struct CommArm {
    gr::uwb::demod::Qm35825Profile profile;
    gr::uwb::demod::core::DemodScratch scratch;

    struct Timing {
        double total_us = 0.0;
        uint64_t stage_cir_us = 0;
        bool fcs = false;
        bool exact = false;
    };

    bool prepare(const Fixture& fx, bool fast)
    {
        profile = gr::uwb::demod::Qm35825Profile::Default();
        profile.code_index = 9;
        profile.preamble_repetitions = fx.sync_reps;
        profile.sfd_mode = "4z2";
        profile.cir_pre_samples = 8;
        profile.cir_post_samples = 30;
        profile.cir_skip_initial_repetitions = 10;
        profile.cir_repetitions = fx.sync_reps - 10;
        profile.timing_coarse_stride = 1;
        if (fast) {
            // The production fast profile used by the pipeline benchmark.
            profile.rx_fast_sfd = true;
            profile.rx_fast_cfo_rotation = true;
            profile.rx_seed_refine_tail = true;
            profile.rx_softchip_on_demand = true;
            profile.rx_fused_payload_cmf = true;
            profile.rx_fused_phr_cmf = true;
        }
        scratch.reserve(fx.window.size());
        return true;
    }

    bool run(const Fixture& fx, Timing& t)
    {
        // demodulate_one's `predicted_start` is the SYNC ORIGIN seed (the
        // comm front-end then backtracks/anchors the SFD itself), unlike
        // radar_cir_one which takes the predicted SFD.  Passing the SFD here
        // would seed the tracker one full preamble late.
        const auto t0 = Clock::now();
        const auto res = gr::uwb::demod::core::demodulate_one(
            fx.window.data(), fx.window.size(), profile, 0, fx.cir_axis, 0,
            fx.sync_pulse, scratch);
        t.total_us = elapsed_us(t0, Clock::now());
        t.stage_cir_us = res.stage_cir_us;
        t.fcs = res.payload.fcs_pass;
        t.exact = (res.payload.bytes == fx.psdu);
        return res.status == gr::uwb::demod::DemodStatus::Success && t.fcs && t.exact;
    }
};

// ---------------------------------------------------------------------------
// Observation bookkeeping
// ---------------------------------------------------------------------------

struct Series {
    std::vector<double> control_total, control_sfd, control_refine, control_cir;
    std::vector<double> cand_total, cand_kernel, cand_cir, cand_timing, cand_cfo,
        cand_sfd, cand_coverage;
    size_t observations = 0;
};

struct CommSeries {
    std::vector<double> legacy_total, legacy_cir;
    std::vector<double> fast_total, fast_cir;
    size_t observations = 0;
};

struct ConfigSpec {
    std::string name;
    size_t sync_reps;
    size_t psdu_bytes;
};

struct Options {
    std::string csv_path;
    std::string testdata_dir = "testdata";
    std::string lock_path = "/tmp/uwb-sic-benchmark.lock";
    int session = 1;
    int sessions = 3;
    int groups = 5;
    int iterations_per_slot = 52; // 5 groups * 2 slots * 52 = 520 per arm
    int warmup = 40;
    bool skip_comm = false;
};

// ---------------------------------------------------------------------------
// CSV emission
// ---------------------------------------------------------------------------

struct Row {
    std::string record;
    int session = 0;
    std::string config;
    std::string arm;
    std::string metric;
    Stats stats;
    double ratio = -1.0; // candidate mean / control mean for the same metric
};

std::string fmt(double v, int prec = 4)
{
    std::ostringstream s;
    s << std::fixed << std::setprecision(prec) << v;
    return s.str();
}

std::string fmtsci(double v)
{
    std::ostringstream s;
    s << std::scientific << std::setprecision(3) << v;
    return s.str();
}

std::string stats_line(const Row& r)
{
    std::ostringstream s;
    s << r.record << ',' << r.session << ',' << r.config << ',' << r.arm << ','
      << r.metric << ',' << r.stats.n << ',' << fmt(r.stats.mean) << ','
      << fmt(r.stats.p50) << ',' << fmt(r.stats.p95) << ',' << fmt(r.stats.min)
      << ',' << fmt(r.stats.max) << ','
      << (r.ratio < 0.0 ? std::string() : fmt(r.ratio, 6));
    return s.str();
}

const char* kCsvHeader =
    "record_type,session,config,arm,metric,n,mean_us,p50_us,p95_us,min_us,max_us,"
    "ratio_vs_control";

} // namespace

int main(int argc, char** argv)
{
    Options opt;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        auto next = [&]() -> std::string {
            if (i + 1 >= argc)
                die("missing value after " + a);
            return argv[++i];
        };
        if (a == "--csv")
            opt.csv_path = next();
        else if (a == "--testdata")
            opt.testdata_dir = next();
        else if (a == "--lock")
            opt.lock_path = next();
        else if (a == "--session")
            opt.session = std::stoi(next());
        else if (a == "--sessions")
            opt.sessions = std::stoi(next());
        else if (a == "--groups")
            opt.groups = std::stoi(next());
        else if (a == "--iters")
            opt.iterations_per_slot = std::stoi(next());
        else if (a == "--warmup")
            opt.warmup = std::stoi(next());
        else if (a == "--skip-comm")
            opt.skip_comm = true;
        else if (a == "--help" || a == "-h") {
            std::cout
                << "usage: benchmark_shared_cir [--csv PATH] [--session N] "
                   "[--sessions N]\n"
                   "       [--groups N] [--iters N] [--warmup N] [--testdata DIR]\n"
                   "       [--lock PATH] [--skip-comm]\n";
            return 0;
        } else
            die("unknown option " + a);
    }
    if (opt.groups < 1 || opt.iterations_per_slot < 1 || opt.warmup < 32)
        die("groups/iters must be >= 1 and warmup must be >= 32");
    if (opt.session < 1 || opt.session > opt.sessions)
        die("session must be in [1, sessions]");

    // ---- exclusive benchmark lock for the whole run -----------------------
    int lock_fd = ::open(opt.lock_path.c_str(), O_CREAT | O_RDWR, 0644);
    if (lock_fd < 0)
        die("cannot open lock file " + opt.lock_path + ": " + std::strerror(errno));
    if (::flock(lock_fd, LOCK_EX) != 0)
        die("cannot take exclusive flock on " + opt.lock_path);

    // ---- provenance -------------------------------------------------------
    std::string exe_path(4096, '\0');
    {
        const ssize_t k = ::readlink("/proc/self/exe", exe_path.data(), exe_path.size() - 1);
        exe_path.resize(k > 0 ? static_cast<size_t>(k) : 0);
    }
    const std::string git_sha = read_cmd("git rev-parse HEAD 2>/dev/null");
    const std::string git_dirty = read_cmd("git status --porcelain 2>/dev/null | wc -l");
    cpu_set_t mask;
    CPU_ZERO(&mask);
    std::string affinity = "unknown";
    if (::sched_getaffinity(0, sizeof(mask), &mask) == 0) {
        std::ostringstream s;
        int first = 1;
        for (int c = 0; c < CPU_SETSIZE; ++c) {
            if (CPU_ISSET(c, &mask)) {
                s << (first ? "" : "|") << c;
                first = 0;
            }
        }
        affinity = s.str();
    }
    std::vector<int> cpu_samples;
    for (int i = 0; i < 8; ++i)
        cpu_samples.push_back(::sched_getcpu());

    std::vector<ConfigSpec> configs = {
        { "sync64_psdu32", 64, 32 },
        { "sync256_psdu127", 256, 127 },
    };

    // ---- build fixtures and check both arms BEFORE any timing -------------
    std::vector<Fixture> fixtures;
    for (const auto& c : configs)
        fixtures.push_back(
            build_fixture(c.name, c.sync_reps, c.psdu_bytes, opt.testdata_dir));

    std::vector<ControlArm> controls(fixtures.size());
    std::vector<CandidateArm> candidates(fixtures.size());
    std::vector<gr::uwb::radar::RadarCirResult> cand_res(fixtures.size());
    std::vector<size_t> guard_valid_repetitions(fixtures.size(), 0);

    for (size_t i = 0; i < fixtures.size(); ++i) {
        const Fixture& fx = fixtures[i];
        if (!controls[i].prepare(fx))
            die("control arm prepare failed for " + fx.name);
        if (!candidates[i].prepare(fx))
            die("candidate arm prepare failed for " + fx.name);

        // --- correctness guard: both arms, one untimed pass each ----------
        ControlArm::Timing ct;
        if (!controls[i].run(fx, ct))
            die("control arm FAILED on fixture " + fx.name +
                " (no timings reported)");
        CandidateArm::Timing qt;
        if (!candidates[i].run(fx, qt, cand_res[i]))
            die("candidate arm FAILED on fixture " + fx.name +
                " (no timings reported)");

        const auto& res = cand_res[i];
        if (res.status != gr::uwb::radar::RadarCirStatus::Ok)
            die("candidate status is not Ok on fixture " + fx.name);
        if (res.tap_count != kTaps || ct.peak_tap >= kTaps)
            die("tap count mismatch on fixture " + fx.name);
        if (res.peak_tap != ct.peak_tap)
            die("peak_tap mismatch on fixture " + fx.name + ": candidate " +
                std::to_string(res.peak_tap) + " vs control " +
                std::to_string(ct.peak_tap));
        if (res.valid_repetitions != ct.valid_repetitions)
            die("valid_repetitions mismatch on fixture " + fx.name + ": " +
                std::to_string(res.valid_repetitions) + " vs " +
                std::to_string(ct.valid_repetitions));
        const double dl2 = rel_l2(candidates[i].scratch.cir_scratch.raw.data(),
                                  controls[i].cir.raw_taps.data(), kTaps);
        if (!(dl2 <= 2e-5))
            die("raw CIR relative L2 " + fmt(dl2, 3) + " exceeds 2e-5 on fixture " +
                fx.name);
        if (res.cfo_epoch != 2u)
            die("candidate did not consume a final-CFO compensated buffer on " +
                fx.name);
        guard_valid_repetitions[i] = res.valid_repetitions;
        // The kernel-only diagnostic must reproduce the production peak.
        if (qt.peak_tap != res.peak_tap)
            die("kernel-only arm disagrees with production on " + fx.name);
        std::cerr << "guard[" << fx.name << "]: ok, peak_tap=" << res.peak_tap
                  << " valid_repetitions=" << res.valid_repetitions
                  << " raw_cir_rel_l2=" << fmtsci(dl2) << " (limit 2e-5)"
                  << " final_cfo_hz=" << fmt(res.final_cfo_hz, 3)
                  << " cfo_epoch=" << res.cfo_epoch
                  << " cir_samples_per_symbol=" << fmt(res.cir_samples_per_symbol, 6)
                  << "\n";
    }

    // ---- one-shot untimed front-end diagnostic ---------------------------
    // `RadarCirResult` reports `frontend_cfo_us` for the CFO FIT plus the small
    // bootstrap rotation only; the final-CFO rotation over the whole CIR union
    // runs inside the SFD anchor pass and is therefore counted in
    // `frontend_sfd_us`.  To explain where the time actually goes we run the
    // shared front-end ONCE per fixture on a private scratch and print its
    // internal split.  This is a diagnostic, never part of the timed matrix.
    struct FrontendDiag {
        std::string config;
        uint64_t timing_us = 0, cfo_us = 0, sfd_us = 0, coverage_us = 0;
        uint64_t cfo_fit_us = 0, cfo_rotate_us = 0, cfo_rotated_samples = 0;
        uint32_t cfo_rotation_passes = 0;
        uint64_t sfd_bootstrap_us = 0, sfd_tail_retrack_us = 0,
                 sfd_cfo_refit_us = 0, sfd_final_us = 0;
        size_t covered_lo = 0, covered_hi = 0;
        double final_cfo_hz = 0.0;
        bool ok = false;
    };
    std::vector<FrontendDiag> diags;
    for (size_t i = 0; i < fixtures.size(); ++i) {
        const Fixture& fx = fixtures[i];
        FrontendDiag d;
        d.config = fx.name;
        gr::uwb::demod::sync_cir::SyncCirRequest req;
        req.cir_axis_origin = fx.cir_axis;
        req.samples_per_symbol = static_cast<double>(kSps);
        req.total_repetitions = fx.sync_reps;
        req.skip_initial = kCirSkip;
        req.max_repetitions = fx.sync_reps - kCirSkip;
        req.pre = kCirPre;
        req.post = kCirPost;
        gr::uwb::demod::core::DemodScratch private_scratch;
        private_scratch.reserve(fx.window.size());
        // Repeat so the reported split is a WARM snapshot (the first call pays
        // page faults on every scratch buffer).  All sub-timers come from the
        // same call: the one with the smallest parent total.
        uint64_t best_total = std::numeric_limits<uint64_t>::max();
        for (int rep = 0; rep < 8; ++rep) {
            gr::uwb::demod::PreambleRxContext ctx;
            const bool ok = gr::uwb::demod::acquire_preamble(
                fx.window.data(), fx.window.size(),
                candidates[i].scratch.demod_profile,
                candidates[i].scratch.sync_pulse, fx.cir_axis,
                gr::uwb::demod::PreamblePurpose::CirOnly, &req, private_scratch, ctx,
                static_cast<double>(kSps));
            const uint64_t total = ctx.timing_us + ctx.cfo_us + ctx.sfd_us;
            if (rep == 0 || (ok && total < best_total)) {
                best_total = total;
                d.ok = ok;
                d.timing_us = ctx.timing_us;
                d.cfo_us = ctx.cfo_us;
                d.sfd_us = ctx.sfd_us;
                d.coverage_us = ctx.coverage_us;
                d.cfo_fit_us = ctx.cfo_fit_us;
                d.cfo_rotate_us = ctx.cfo_rotate_us;
                d.cfo_rotated_samples = ctx.cfo_rotated_samples;
                d.cfo_rotation_passes = ctx.cfo_rotation_passes;
                d.sfd_bootstrap_us = ctx.sfd_bootstrap_us;
                d.sfd_tail_retrack_us = ctx.sfd_tail_retrack_us;
                d.sfd_cfo_refit_us = ctx.sfd_cfo_refit_us;
                d.sfd_final_us = ctx.sfd_final_us;
                d.covered_lo = ctx.covered_lo;
                d.covered_hi = ctx.covered_hi;
                d.final_cfo_hz = ctx.final_cfo_hz;
            }
        }
        diags.push_back(d);
    }

    // ---- comm arm ---------------------------------------------------------
    std::vector<CommArm> comm_legacy(1), comm_fast(1);
    if (!opt.skip_comm) {
        if (!comm_legacy[0].prepare(fixtures[0], /*fast=*/false))
            die("comm legacy arm prepare failed");
        if (!comm_fast[0].prepare(fixtures[0], /*fast=*/true))
            die("comm fast arm prepare failed");
        CommArm::Timing a, b;
        if (!comm_legacy[0].run(fixtures[0], a))
            die("comm legacy-profile arm FAILED (no FCS/exact PSDU) "
                "(no timings reported)");
        if (!comm_fast[0].run(fixtures[0], b))
            die("comm fast-profile arm FAILED (no FCS/exact PSDU) "
                "(no timings reported)");
        std::cerr << "guard[comm]: legacy profile ok (stage_cir_us="
                  << a.stage_cir_us << "), fast profile ok (stage_cir_us="
                  << b.stage_cir_us << ")\n";
    }

    // ---- header block -----------------------------------------------------
    std::ostringstream header;
    header << "# benchmark_shared_cir (shared SYNC-CIR kernel / shared preamble "
              "front-end)\n";
    header << "# session=" << opt.session << "/" << opt.sessions
           << " groups=" << opt.groups
           << " iters_per_slot=" << opt.iterations_per_slot
           << " warmup=" << opt.warmup << "\n";
    header << "# git_sha=" << (git_sha.empty() ? "unknown" : git_sha)
           << " dirty_entries=" << (git_dirty.empty() ? "?" : git_dirty) << "\n";
    header << "# binary=" << exe_path << "\n";
    header << "# nproc_online=" << ::get_nprocs() << " nproc_conf=" << ::get_nprocs_conf()
           << " affinity=" << affinity << " sched_getcpu_samples=";
    for (size_t i = 0; i < cpu_samples.size(); ++i)
        header << (i ? "|" : "") << cpu_samples[i];
    header << "\n";
    header << "# sample_rate_hz=" << fmt(kSampleRateHz, 0)
           << " sync_period_samples=" << kSps << " cir_pre=" << kCirPre
           << " cir_post=" << kCirPost << " taps=" << kTaps
           << " cir_skip_initial=" << kCirSkip
           << " sfd_mode=4z2 code_index=9 pulse=legacy48 peak_amplitude=0.8\n";
    header << "# thresholds: sfd=0.3 sync_refine=0.3 sfd_search_margin=64 "
              "sync_refine_margin=8 timing_search_margin(front-end)=64 "
              "timing_coarse_stride=1 timing_track_radius=8\n";
    for (size_t fi = 0; fi < fixtures.size(); ++fi) {
        const Fixture& fx = fixtures[fi];
        const size_t cfo_union =
            [&]() {
                gr::uwb::demod::sync_cir::SyncCirRequest req;
                req.cir_axis_origin = fx.cir_axis;
                req.samples_per_symbol = static_cast<double>(kSps);
                req.total_repetitions = fx.sync_reps;
                req.skip_initial = kCirSkip;
                req.max_repetitions = fx.sync_reps - kCirSkip;
                req.pre = kCirPre;
                req.post = kCirPost;
                int64_t lo = -1, hi = -1;
                if (!gr::uwb::demod::sync_cir::required_input_range(
                        req, gr::uwb::demod::sync_cir::kSyncCirCodeLength,
                        fx.window.size(), lo, hi))
                    return size_t(0);
                return static_cast<size_t>(hi - lo);
            }();
        header << "# fixture=" << fx.name << " sync_reps=" << fx.sync_reps
               << " psdu_bytes=" << fx.psdu_bytes
               << " psdu_sha_fnv1a=" << std::hex << fx.psdu_hash << std::dec
               << " pre_guard=" << fx.pre_guard
               << " packet_samples=" << fx.packet_samples
               << " window_samples=" << fx.window.size()
               << " predicted_sfd=" << fx.predicted_sfd
               << " cir_axis=" << fx.cir_axis
               << " window_fnv1a=" << std::hex << fx.window_hash << std::dec
               << " cir_union_samples=" << cfo_union
               << " valid_repetitions=" << guard_valid_repetitions[fi]
               << " sync_template_rel_l2_vs_testdata="
               << (fx.template_rel_l2 < 0.0 ? std::string("testdata_missing")
                                            : fmt(fx.template_rel_l2, 3))
               << "\n";
    }
    header << "# comm_arm: demodulate_one on sync64_psdu32; "
              "legacy_profile=all rx_fast_* off (full-stream soft chips), "
              "fast_profile=production flags on\n";
    for (const auto& d : diags) {
        header << "# frontend_diag=" << d.config << " ok=" << (d.ok ? 1 : 0)
               << " timing_us=" << d.timing_us << " cfo_us=" << d.cfo_us
               << " sfd_us=" << d.sfd_us << " coverage_us=" << d.coverage_us
               << " | cfo_fit_us=" << d.cfo_fit_us
               << " cfo_rotate_us=" << d.cfo_rotate_us
               << " cfo_rotated_samples=" << d.cfo_rotated_samples
               << " cfo_rotation_passes=" << d.cfo_rotation_passes
               << " | sfd_bootstrap_us=" << d.sfd_bootstrap_us
               << " sfd_tail_retrack_us=" << d.sfd_tail_retrack_us
               << " sfd_cfo_refit_us=" << d.sfd_cfo_refit_us
               << " sfd_final_us=" << d.sfd_final_us
               << " | covered=[" << d.covered_lo << "," << d.covered_hi << ")"
               << " final_cfo_hz=" << fmt(d.final_cfo_hz, 3)
               << " (single untimed call; diagnostics only)\n";
    }

    // ---- measurement ------------------------------------------------------
    std::vector<Series> series(fixtures.size());
    CommSeries comm_series;
    const int per_slot = opt.iterations_per_slot;

    auto measure_config = [&](size_t i) {
        const Fixture& fx = fixtures[i];
        ControlArm& ca = controls[i];
        CandidateArm& qa = candidates[i];
        Series& s = series[i];

        auto run_control = [&]() {
            ControlArm::Timing t;
            if (!ca.run(fx, t))
                die("control arm failed during measurement on " + fx.name);
            if (t.peak_tap != cand_res[i].peak_tap)
                die("control peak drifted during measurement on " + fx.name);
            s.control_total.push_back(t.total_us);
            s.control_sfd.push_back(t.sfd_us);
            s.control_refine.push_back(t.refine_us);
            s.control_cir.push_back(t.cir_us);
        };
        auto run_candidate = [&]() {
            CandidateArm::Timing t;
            gr::uwb::radar::RadarCirResult res;
            if (!qa.run(fx, t, res))
                die("candidate arm failed during measurement on " + fx.name);
            if (res.peak_tap != cand_res[i].peak_tap)
                die("candidate peak drifted during measurement on " + fx.name);
            s.cand_total.push_back(t.total_us);
            s.cand_kernel.push_back(t.kernel_us);
            s.cand_cir.push_back(static_cast<double>(t.cir_us));
            s.cand_timing.push_back(static_cast<double>(t.frontend_timing_us));
            s.cand_cfo.push_back(static_cast<double>(t.frontend_cfo_us));
            s.cand_sfd.push_back(static_cast<double>(t.frontend_sfd_us));
            s.cand_coverage.push_back(static_cast<double>(t.coverage_us));
            ++s.observations;
        };

        for (int w = 0; w < opt.warmup; ++w) {
            run_control();
            run_candidate();
        }
        s.control_total.clear();
        s.control_sfd.clear();
        s.control_refine.clear();
        s.control_cir.clear();
        s.cand_total.clear();
        s.cand_kernel.clear();
        s.cand_cir.clear();
        s.cand_timing.clear();
        s.cand_cfo.clear();
        s.cand_sfd.clear();
        s.cand_coverage.clear();
        s.observations = 0;

        for (int g = 0; g < opt.groups; ++g) {
            for (int k = 0; k < per_slot; ++k)
                run_control();
            for (int k = 0; k < per_slot; ++k)
                run_candidate();
            for (int k = 0; k < per_slot; ++k)
                run_candidate();
            for (int k = 0; k < per_slot; ++k)
                run_control();
        }
        if (s.control_total.size() != s.cand_total.size())
            die("unbalanced ABBA observation counts on " + fx.name);
    };

    auto measure_comm = [&]() {
        const Fixture& fx = fixtures[0];
        auto run_legacy = [&]() {
            CommArm::Timing t;
            if (!comm_legacy[0].run(fx, t))
                die("comm legacy-profile arm failed during measurement");
            comm_series.legacy_total.push_back(t.total_us);
            comm_series.legacy_cir.push_back(static_cast<double>(t.stage_cir_us));
        };
        auto run_fast = [&]() {
            CommArm::Timing t;
            if (!comm_fast[0].run(fx, t))
                die("comm fast-profile arm failed during measurement");
            comm_series.fast_total.push_back(t.total_us);
            comm_series.fast_cir.push_back(static_cast<double>(t.stage_cir_us));
            ++comm_series.observations;
        };
        for (int w = 0; w < opt.warmup; ++w) {
            run_legacy();
            run_fast();
        }
        comm_series.legacy_total.clear();
        comm_series.legacy_cir.clear();
        comm_series.fast_total.clear();
        comm_series.fast_cir.clear();
        comm_series.observations = 0;
        for (int g = 0; g < opt.groups; ++g) {
            for (int k = 0; k < per_slot; ++k)
                run_legacy();
            for (int k = 0; k < per_slot; ++k)
                run_fast();
            for (int k = 0; k < per_slot; ++k)
                run_fast();
            for (int k = 0; k < per_slot; ++k)
                run_legacy();
        }
    };

    for (size_t i = 0; i < fixtures.size(); ++i)
        measure_config(i);
    if (!opt.skip_comm)
        measure_comm();

    // ---- rows -------------------------------------------------------------
    std::vector<Row> rows;
    auto add = [&](const std::string& config, const std::string& arm,
                   const std::string& metric, const std::vector<double>& v,
                   double ratio) {
        Row r;
        r.record = "summary";
        r.session = opt.session;
        r.config = config;
        r.arm = arm;
        r.metric = metric;
        r.stats = summarise(v);
        r.ratio = ratio;
        rows.push_back(r);
    };

    for (size_t i = 0; i < fixtures.size(); ++i) {
        const Series& s = series[i];
        const std::string& cfg = fixtures[i].name;
        const double ctrl_total = summarise(s.control_total).mean;
        const double ctrl_cir = summarise(s.control_cir).mean;
        const double cand_total = summarise(s.cand_total).mean;
        const double cand_cir = summarise(s.cand_kernel).mean;

        add(cfg, "control", "total", s.control_total, -1.0);
        add(cfg, "control", "sfd", s.control_sfd, -1.0);
        add(cfg, "control", "refine", s.control_refine, -1.0);
        add(cfg, "control", "cir_scalar_kernel", s.control_cir, -1.0);

        add(cfg, "candidate", "total", s.cand_total,
            ctrl_total > 0 ? cand_total / ctrl_total : -1.0);
        add(cfg, "candidate", "frontend_timing", s.cand_timing, -1.0);
        add(cfg, "candidate", "frontend_cfo", s.cand_cfo, -1.0);
        add(cfg, "candidate", "frontend_sfd", s.cand_sfd, -1.0);
        add(cfg, "candidate", "coverage", s.cand_coverage, -1.0);
        add(cfg, "candidate", "cir_us_reported", s.cand_cir, -1.0);
        add(cfg, "candidate", "cir_shared_kernel", s.cand_kernel,
            ctrl_cir > 0 ? cand_cir / ctrl_cir : -1.0);
    }
    if (!opt.skip_comm) {
        const double lt = summarise(comm_series.legacy_total).mean;
        const double lc = summarise(comm_series.legacy_cir).mean;
        const double ft = summarise(comm_series.fast_total).mean;
        const double fc = summarise(comm_series.fast_cir).mean;
        add("comm64_psdu32", "comm_legacy_profile", "total", comm_series.legacy_total,
            -1.0);
        add("comm64_psdu32", "comm_legacy_profile", "stage_cir",
            comm_series.legacy_cir, -1.0);
        add("comm64_psdu32", "comm_fast_profile", "total", comm_series.fast_total,
            lt > 0 ? ft / lt : -1.0);
        add("comm64_psdu32", "comm_fast_profile", "stage_cir", comm_series.fast_cir,
            lc > 0 ? fc / lc : -1.0);
    }

    // ---- emit -------------------------------------------------------------
    std::ostringstream csv;
    csv << kCsvHeader << "\n";
    for (const auto& r : rows)
        csv << stats_line(r) << "\n";

    std::cout << header.str();
    std::cout << csv.str();
    std::cout.flush();

    if (!opt.csv_path.empty()) {
        std::ofstream f(opt.csv_path, std::ios::trunc);
        if (!f)
            die("cannot write CSV to " + opt.csv_path);
        f << header.str() << csv.str();
        if (!f)
            die("CSV write failed for " + opt.csv_path);
    }

    // ---- human-readable per-session summary ------------------------------
    std::cerr << "\n=== session " << opt.session << "/" << opt.sessions
              << " summary (microseconds) ===\n";
    for (const auto& r : rows) {
        std::cerr << std::left << std::setw(18) << r.config << std::setw(22)
                  << r.arm << std::setw(20) << r.metric << std::right
                  << " n=" << std::setw(4) << r.stats.n << " mean=" << std::setw(10)
                  << fmt(r.stats.mean, 2) << " p50=" << std::setw(10)
                  << fmt(r.stats.p50, 2) << " p95=" << std::setw(10)
                  << fmt(r.stats.p95, 2);
        if (r.ratio >= 0.0)
            std::cerr << " ratio=" << fmt(r.ratio, 4);
        std::cerr << "\n";
    }

    ::flock(lock_fd, LOCK_UN);
    ::close(lock_fd);
    return 0;
}
