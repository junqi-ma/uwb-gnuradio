/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Benchmark runner for UHD SC16 Direct RX and Multi-Core Stream Pipeline.
 * Bypasses GNU Radio scheduler and PMT/PDU messaging overhead.
 */

#include <cmath>
using std::isinf;
using std::isnan;

#include <gnuradio/uwb/uwb_uhd_sc16_pipeline.h>

#include "pipeline_bench_setup.h"
#include "pipeline_result_json.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <numeric>
#include <string>
#include <sys/file.h>
#include <sys/stat.h>
#include <unistd.h>
#include <utility>
#include <vector>

using namespace gr::uwb::pipeline;

#ifndef UWB_TESTDATA_DIR
#define UWB_TESTDATA_DIR "testdata"
#endif

namespace {

struct BenchmarkLockGuard {
    int fd = -1;
    bool locked = false;

    bool acquire()
    {
        fd = ::open("/tmp/uwb-sic-benchmark.lock", O_CREAT | O_RDWR, 0666);
        if (fd < 0) {
            std::cerr << "Warning: Could not open /tmp/uwb-sic-benchmark.lock: "
                      << std::strerror(errno) << std::endl;
            return false;
        }
        std::cout << "[Benchmark] Waiting for exclusive lock /tmp/uwb-sic-benchmark.lock..."
                  << std::endl;
        if (::flock(fd, LOCK_EX) != 0) {
            std::cerr << "Warning: Could not acquire flock on /tmp/uwb-sic-benchmark.lock: "
                      << std::strerror(errno) << std::endl;
            ::close(fd);
            fd = -1;
            return false;
        }
        locked = true;
        std::cout << "[Benchmark] Acquired exclusive lock /tmp/uwb-sic-benchmark.lock"
                  << std::endl;
        return true;
    }

    ~BenchmarkLockGuard()
    {
        if (locked && fd >= 0) {
            ::flock(fd, LOCK_UN);
            ::close(fd);
            std::cout << "[Benchmark] Released lock /tmp/uwb-sic-benchmark.lock" << std::endl;
            locked = false;
            fd = -1;
        }
    }
};

struct PhaseStats {
    size_t n = 0;
    double mean = 0.0;
    double p50 = 0.0;
    double p95 = 0.0;
    double p99 = 0.0;
    double min = 0.0;
    double max = 0.0;
};

PhaseStats compute_phase_stats(std::vector<double>& values)
{
    PhaseStats s;
    if (values.empty()) {
        return s;
    }
    std::sort(values.begin(), values.end());
    s.n = values.size();
    s.min = values.front();
    s.max = values.back();
    double sum = std::accumulate(values.begin(), values.end(), 0.0);
    s.mean = sum / static_cast<double>(s.n);

    auto at = [&](double p) {
        if (s.n == 1) {
            return values[0];
        }
        const double idx = p * static_cast<double>(s.n - 1);
        size_t pos = static_cast<size_t>(std::llround(idx));
        if (pos >= s.n) {
            pos = s.n - 1;
        }
        return values[pos];
    };

    s.p50 = at(0.50);
    s.p95 = at(0.95);
    s.p99 = at(0.99);
    return s;
}

// Per-DSP-worker aggregation for the Burst-level multi-core report.
// worker_id == -1 marks windows that never reached a worker (drops/cancelled).
struct WorkerDistribution {
    int worker_id = -1;
    size_t windows = 0;   // every terminal state seen for this worker
    size_t completed = 0; // WindowTerminalState::Completed only
    PhaseStats e2e;
    PhaseStats resample;
    PhaseStats sic;
};

// Print a fixed-precision value right-aligned in `width`, or N/A when the
// value is not meaningful (no windows, zero elapsed time, nan/inf).
void print_stat_value(bool valid, double value, int precision, int width)
{
    std::cout << std::right;
    if (!valid || isnan(value) || isinf(value)) {
        std::cout << std::setw(width) << "N/A";
        return;
    }
    std::cout << std::fixed << std::setprecision(precision) << std::setw(width) << value;
}

void print_system_throughput(int dsp_workers,
                             size_t completed_windows,
                             double total_elapsed_s,
                             size_t native_window_capacity,
                             double total_resample_us)
{
    const bool has_windows = (completed_windows > 0);
    const bool has_time = (total_elapsed_s > 0.0);
    const bool has_rate = has_windows && has_time;

    const double interval_us = has_rate
                                   ? (total_elapsed_s * 1e6 /
                                      static_cast<double>(completed_windows))
                                   : 0.0;
    const double window_rate = has_rate
                                   ? (static_cast<double>(completed_windows) / total_elapsed_s)
                                   : 0.0;
    const double sample_throughput_msps =
        has_rate ? (static_cast<double>(completed_windows) *
                    static_cast<double>(native_window_capacity) / total_elapsed_s / 1e6)
                 : 0.0;
    const double resample_equiv_us =
        has_windows ? (total_resample_us / static_cast<double>(completed_windows)) : 0.0;

    std::cout << std::right << "System Throughput (Burst-level multi-core):\n"
              << "  DSP Workers Configured:            " << std::setw(10) << dsp_workers << "\n"
              << "  Completed Windows:                 " << std::setw(10) << completed_windows << "\n"
              << "  Total Wall Time:                   ";
    print_stat_value(has_time, total_elapsed_s, 3, 10);
    std::cout << " s\n"
              << "  Effective Window Interval:         ";
    print_stat_value(has_rate, interval_us, 2, 10);
    std::cout << " us/window\n"
              << "  Sustained Window Rate:             ";
    print_stat_value(has_rate, window_rate, 2, 10);
    std::cout << " win/s\n"
              << "  Effective Native Sample Throughput:";
    print_stat_value(has_rate, sample_throughput_msps, 2, 10);
    std::cout << " MS/s\n"
              << "  Effective Resample Equivalent:     ";
    print_stat_value(has_windows, resample_equiv_us, 2, 10);
    std::cout << " us/window\n\n";
}

void print_worker_distribution(const std::vector<WorkerDistribution>& rows,
                               size_t total_windows,
                               int dsp_workers)
{
    std::cout << "Per-Worker Distribution (DSP Workers: " << dsp_workers << "):\n"
              << "  -----------------------------------------------------------------------------\n"
              << "  " << std::left << std::setw(12) << "Worker"
              << std::right << std::setw(8) << "Windows"
              << std::setw(10) << "Share(%)"
              << std::setw(14) << "Mean e2e(us)"
              << std::setw(19) << "Mean resample(us)"
              << std::setw(14) << "Mean sic(us)" << "\n"
              << "  -----------------------------------------------------------------------------\n";

    size_t completed_min = 0;
    size_t completed_max = 0;
    bool have_completed = false;

    for (const auto& row : rows) {
        const double share =
            (total_windows > 0)
                ? (100.0 * static_cast<double>(row.windows) /
                   static_cast<double>(total_windows))
                : 0.0;

        if (row.worker_id >= 0) {
            std::cout << "  " << std::left << std::setw(12) << row.worker_id;
        } else {
            std::cout << "  " << std::left << std::setw(12) << "unassigned";
        }
        std::cout << std::right << std::setw(8) << row.windows;
        print_stat_value(total_windows > 0, share, 1, 10);
        print_stat_value(row.completed > 0, row.e2e.mean, 2, 14);
        print_stat_value(row.completed > 0, row.resample.mean, 2, 19);
        print_stat_value(row.completed > 0, row.sic.mean, 2, 14);
        std::cout << "\n";

        // Load balance only considers real workers that completed >= 1 window.
        if (row.worker_id >= 0 && row.completed > 0) {
            if (!have_completed) {
                completed_min = row.completed;
                completed_max = row.completed;
                have_completed = true;
            } else {
                completed_min = std::min(completed_min, row.completed);
                completed_max = std::max(completed_max, row.completed);
            }
        }
    }

    std::cout << "  -----------------------------------------------------------------------------\n"
              << "  Load Balance (max/min):            ";
    if (have_completed && completed_min > 0) {
        std::cout << std::fixed << std::setprecision(2)
                  << (static_cast<double>(completed_max) / static_cast<double>(completed_min))
                  << "x\n";
    } else {
        std::cout << "N/A\n";
    }
    std::cout << "\n";
}

// Strict non-negative integer parse shared by --dsp-workers and the
// comma-separated --cpu-dsp list.  Rejects empty tokens, trailing garbage and
// negative values instead of silently truncating them.
bool parse_non_negative_int(const std::string& token,
                            const char* what,
                            int& value,
                            std::string& err)
{
    if (token.empty()) {
        err = std::string("empty ") + what + " token";
        return false;
    }
    size_t consumed = 0;
    int parsed = 0;
    try {
        parsed = std::stoi(token, &consumed);
    } catch (const std::exception&) {
        err = "non-numeric " + std::string(what) + " '" + token + "'";
        return false;
    }
    if (consumed != token.size()) {
        err = "non-numeric " + std::string(what) + " '" + token + "'";
        return false;
    }
    if (parsed < 0) {
        err = "negative " + std::string(what) + " " + std::to_string(parsed);
        return false;
    }
    value = parsed;
    return true;
}

// Parse a comma-separated DSP affinity list (e.g. "1,2,3,4").  Empty tokens
// (including a trailing comma) are rejected.
bool parse_cpu_core_list(const std::string& spec, std::vector<int>& out, std::string& err)
{
    out.clear();
    size_t begin = 0;
    while (true) {
        const size_t comma = spec.find(',', begin);
        const std::string token = (comma == std::string::npos)
                                      ? spec.substr(begin)
                                      : spec.substr(begin, comma - begin);
        int value = 0;
        if (!parse_non_negative_int(token, "CPU core", value, err)) {
            out.clear();
            return false;
        }
        out.push_back(value);
        if (comma == std::string::npos) {
            break;
        }
        begin = comma + 1;
    }
    if (out.empty()) {
        err = "no CPU cores specified";
        return false;
    }
    return true;
}

std::string join_ints(const std::vector<int>& values)
{
    std::string out;
    for (size_t i = 0; i < values.size(); ++i) {
        if (i != 0) {
            out += ',';
        }
        out += std::to_string(values[i]);
    }
    return out;
}

void print_help(const char* prog_name)
{
    std::cout << "Usage: " << prog_name << " [options]\n\n"
              << "UHD SC16 Direct RX and Multi-Core Stream Pipeline Benchmark Runner\n\n"
              << "Options:\n"
              << "  --source <replay|uhd>       Data stream source (default: replay)\n"
              << "  --file <path>               Input SC16 file path for replay source\n"
              << "  --native-rate <hz>          Native sampling rate in Hz (default: 998400000.0)\n"
              << "  --work-rate <hz>            DSP work rate in Hz (default: 998400000.0)\n"
              << "  --t0 <samples>              Initial schedule offset in samples (default: 0.0)\n"
              << "  --period <samples>          Schedule period in samples (default: 998400.0)\n"
              << "  --pre-guard <n>             Pre-guard samples per window (default: 400)\n"
              << "  --capture <n>               Capture samples per window (default: 3200)\n"
              << "  --post-guard <n>            Post-guard samples per window (default: 400)\n"
              << "  --consumer <sic|sensing|both> Consumer pipeline mode (default: sic)\n"
              << "  --replay-mode <fullspeed|paced> Replay timing mode (default: fullspeed)\n"
              << "  --cpu-rx <core>             Affinity CPU core for RX thread (-1 = unpinned)\n"
              << "  --cpu-dsp <core[,core...]>  DSP affinity: single core (-1 = unpinned) or one core per DSP worker\n"
              << "  --dsp-workers <n>           Burst-level DSP worker threads, run-to-completion (default: 1)\n"
              << "  --pool-size <n>             Ring pool slot capacity per DSP worker (default: 8)\n"
              << "  --max-windows <n>           Maximum windows to process (default: 100)\n"
              << "  --warmup-windows <n>        Completed windows excluded from steady stats (default: 0)\n"
              << "  --measure-windows <n>       Completed windows in the steady region (default: 0 = all remaining)\n"
              << "  --diag-rx                   Enable RX diagnostics counters\n"
              << "  --diag-commit               Enable commit diagnostics counters\n"
              << "  --label <name>              Run label written into the JSON meta\n"
              << "  --json-out <path>           Write window-by-window results JSON\n"
              << "  --csv-out <path>            Write window-by-window results CSV\n"
              << "  --resampler-profile <name>  Resampler profile (quality_minorder|realtime_minorder|quality|realtime, default: quality_minorder)\n"
              << "  --resampler-workers <n>     Resampler thread worker count (default: 1)\n"
              << "  --sic-policy <p>            SIC policy: preamble|fullpacket|auto (default: auto)\n"
              << "  --sic-backend <b>           SIC estimator: ipatov|discrete|fractional (default: ipatov)\n"
              << "  --sic-comm-code <n>         Comm SYNC code index for SIC (default: 10)\n"
              << "  --sic-sync-template <path>  One SYNC symbol CF32 @998.4M (default: testdata/reference_preamble.bin, code 9)\n"
              << "  --sic-roi-margin <n>        SIC coarse-search half-width around the expected preamble (default: 64)\n"
              << "  --sic-tap-count <n>         SIC discrete-CIR tap count (default: 20)\n"
              << "  --benchmark                 Acquire exclusive lock /tmp/uwb-sic-benchmark.lock\n"
              << "  --help, -h                  Show this help message and exit\n"
              << "\n"
              << "Optional UHD options:\n"
              << "  --uhd-args <args>           UHD device arguments\n"
              << "  --uhd-subdev <spec>         UHD subdev specification\n"
              << "  --uhd-channel <chan>        UHD channel\n"
              << "  --freq <hz>                 Center frequency in Hz (default: 3993.6e6)\n"
              << "  --gain <db>                 RX gain in dB (default: 30.0)\n"
              << std::endl;
}

bool file_exists(const std::string& path)
{
    struct stat buffer;
    return (::stat(path.c_str(), &buffer) == 0);
}

} // namespace

int main(int argc, char* argv[])
{
    PipelineConfig cfg;

    std::string source_str = "replay";
    std::string file_path = "";
    double period_samples = 998400.0;
    std::string consumer_str = "sic";
    std::string replay_mode_str = "fullspeed";
    bool benchmark_mode = false;
    // SIC coarse-search ROI half-width in work samples, relative to the
    // expected preamble position (pre_guard mapped to the work grid).
    long long sic_roi_margin = 64;
    // 2026-11-11 round 2 measurement contract: steady region selection and
    // machine-readable output.  Defaults keep the historical cold-start
    // protocol (all completed windows are reported).
    long long warmup_windows = 0;
    long long measure_windows = 0;
    std::string label = "runner";
    std::string json_out;
    std::string csv_out;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            print_help(argv[0]);
            return 0;
        } else if (arg == "--benchmark") {
            benchmark_mode = true;
        } else if (arg == "--source") {
            if (i + 1 >= argc) {
                std::cerr << "Error: --source requires an argument (replay|uhd)\n";
                print_help(argv[0]);
                return 1;
            }
            source_str = argv[++i];
        } else if (arg == "--file") {
            if (i + 1 >= argc) {
                std::cerr << "Error: --file requires a file path argument\n";
                print_help(argv[0]);
                return 1;
            }
            file_path = argv[++i];
        } else if (arg == "--native-rate") {
            if (i + 1 >= argc) {
                std::cerr << "Error: --native-rate requires a numeric rate in Hz\n";
                print_help(argv[0]);
                return 1;
            }
            cfg.native_rate = std::stod(argv[++i]);
        } else if (arg == "--work-rate") {
            if (i + 1 >= argc) {
                std::cerr << "Error: --work-rate requires a numeric rate in Hz\n";
                print_help(argv[0]);
                return 1;
            }
            cfg.work_rate = std::stod(argv[++i]);
        } else if (arg == "--t0") {
            if (i + 1 >= argc) {
                std::cerr << "Error: --t0 requires a numeric sample offset\n";
                print_help(argv[0]);
                return 1;
            }
            cfg.schedule_t0 = std::stod(argv[++i]);
        } else if (arg == "--period") {
            if (i + 1 >= argc) {
                std::cerr << "Error: --period requires a period in samples\n";
                print_help(argv[0]);
                return 1;
            }
            period_samples = std::stod(argv[++i]);
        } else if (arg == "--pre-guard") {
            if (i + 1 >= argc) {
                std::cerr << "Error: --pre-guard requires a sample count\n";
                print_help(argv[0]);
                return 1;
            }
            cfg.pre_guard_samples = static_cast<size_t>(std::stoul(argv[++i]));
        } else if (arg == "--capture") {
            if (i + 1 >= argc) {
                std::cerr << "Error: --capture requires a sample count\n";
                print_help(argv[0]);
                return 1;
            }
            cfg.capture_samples = static_cast<size_t>(std::stoul(argv[++i]));
        } else if (arg == "--post-guard") {
            if (i + 1 >= argc) {
                std::cerr << "Error: --post-guard requires a sample count\n";
                print_help(argv[0]);
                return 1;
            }
            cfg.post_guard_samples = static_cast<size_t>(std::stoul(argv[++i]));
        } else if (arg == "--consumer") {
            if (i + 1 >= argc) {
                std::cerr << "Error: --consumer requires an argument (sic|sensing|both)\n";
                print_help(argv[0]);
                return 1;
            }
            consumer_str = argv[++i];
        } else if (arg == "--replay-mode") {
            if (i + 1 >= argc) {
                std::cerr << "Error: --replay-mode requires an argument (fullspeed|paced)\n";
                print_help(argv[0]);
                return 1;
            }
            replay_mode_str = argv[++i];
        } else if (arg == "--cpu-rx") {
            if (i + 1 >= argc) {
                std::cerr << "Error: --cpu-rx requires a CPU core index\n";
                print_help(argv[0]);
                return 1;
            }
            cfg.cpu_core_rx = std::stoi(argv[++i]);
        } else if (arg == "--cpu-dsp") {
            if (i + 1 >= argc) {
                std::cerr << "Error: --cpu-dsp requires a CPU core index or a comma-separated list\n";
                print_help(argv[0]);
                return 1;
            }
            const std::string spec = argv[++i];
            if (spec.find(',') == std::string::npos) {
                // Backward-compatible single core form; -1 means unpinned.
                size_t consumed = 0;
                int core = 0;
                bool parsed_ok = true;
                try {
                    core = std::stoi(spec, &consumed);
                } catch (const std::exception&) {
                    parsed_ok = false;
                }
                if (!parsed_ok || spec.empty() || consumed != spec.size()) {
                    std::cerr << "Error: Invalid --cpu-dsp value '" << spec
                              << "': expected a CPU core index (-1 = unpinned) or a "
                                 "comma-separated list such as 1,2,3,4\n";
                    print_help(argv[0]);
                    return 1;
                }
                cfg.cpu_core_dsp = core;
                cfg.cpu_cores_dsp.clear();
            } else {
                std::vector<int> cores;
                std::string err;
                if (!parse_cpu_core_list(spec, cores, err)) {
                    std::cerr << "Error: Invalid --cpu-dsp list '" << spec << "': " << err
                              << "\n";
                    print_help(argv[0]);
                    return 1;
                }
                cfg.cpu_cores_dsp = cores;
                cfg.cpu_core_dsp = cores.front();
            }
        } else if (arg == "--dsp-workers") {
            if (i + 1 >= argc) {
                std::cerr << "Error: --dsp-workers requires a worker count\n";
                print_help(argv[0]);
                return 1;
            }
            const std::string spec = argv[++i];
            int workers = 0;
            std::string err;
            if (!parse_non_negative_int(spec, "worker count", workers, err)) {
                std::cerr << "Error: Invalid --dsp-workers value '" << spec << "': " << err
                          << "\n";
                print_help(argv[0]);
                return 1;
            }
            if (workers < 1) {
                std::cerr << "Error: --dsp-workers must be >= 1 (got " << workers << ")\n";
                print_help(argv[0]);
                return 1;
            }
            cfg.dsp_workers = workers;
        } else if (arg == "--pool-size") {
            if (i + 1 >= argc) {
                std::cerr << "Error: --pool-size requires a slot count\n";
                print_help(argv[0]);
                return 1;
            }
            cfg.pool_capacity = static_cast<size_t>(std::stoul(argv[++i]));
        } else if (arg == "--max-windows") {
            if (i + 1 >= argc) {
                std::cerr << "Error: --max-windows requires a window count\n";
                print_help(argv[0]);
                return 1;
            }
            cfg.max_windows = static_cast<uint64_t>(std::stoull(argv[++i]));
        } else if (arg == "--warmup-windows") {
            if (i + 1 >= argc) {
                std::cerr << "Error: --warmup-windows requires a window count\n";
                print_help(argv[0]);
                return 1;
            }
            warmup_windows = std::stoll(argv[++i]);
        } else if (arg == "--measure-windows") {
            if (i + 1 >= argc) {
                std::cerr << "Error: --measure-windows requires a window count\n";
                print_help(argv[0]);
                return 1;
            }
            measure_windows = std::stoll(argv[++i]);
        } else if (arg == "--diag-rx") {
            cfg.enable_rx_diagnostics = true;
        } else if (arg == "--diag-commit") {
            cfg.enable_commit_diagnostics = true;
        } else if (arg == "--label") {
            if (i + 1 >= argc) {
                std::cerr << "Error: --label requires a name\n";
                print_help(argv[0]);
                return 1;
            }
            label = argv[++i];
        } else if (arg == "--json-out") {
            if (i + 1 >= argc) {
                std::cerr << "Error: --json-out requires a path\n";
                print_help(argv[0]);
                return 1;
            }
            json_out = argv[++i];
        } else if (arg == "--csv-out") {
            if (i + 1 >= argc) {
                std::cerr << "Error: --csv-out requires a path\n";
                print_help(argv[0]);
                return 1;
            }
            csv_out = argv[++i];
        } else if (arg == "--resampler-profile") {
            if (i + 1 >= argc) {
                std::cerr << "Error: --resampler-profile requires a profile name\n";
                print_help(argv[0]);
                return 1;
            }
            cfg.resampler_profile = argv[++i];
        } else if (arg == "--resampler-workers") {
            if (i + 1 >= argc) {
                std::cerr << "Error: --resampler-workers requires worker count\n";
                print_help(argv[0]);
                return 1;
            }
            cfg.resampler_workers = std::stoi(argv[++i]);
        } else if (arg == "--sic-policy") {
            if (i + 1 >= argc) {
                std::cerr << "Error: --sic-policy requires an argument (preamble|fullpacket|auto)\n";
                print_help(argv[0]);
                return 1;
            }
            const std::string v = argv[++i];
            if (v == "preamble") {
                cfg.sic_cfg.policy = gr::uwb::sic::SicPolicy::PreambleOnly;
            } else if (v == "fullpacket") {
                cfg.sic_cfg.policy = gr::uwb::sic::SicPolicy::FullPacket;
            } else if (v == "auto") {
                cfg.sic_cfg.policy = gr::uwb::sic::SicPolicy::AutoFallback;
            } else {
                std::cerr << "Error: Invalid --sic-policy '" << v
                          << "'. Expected preamble|fullpacket|auto.\n";
                print_help(argv[0]);
                return 1;
            }
        } else if (arg == "--sic-backend") {
            if (i + 1 >= argc) {
                std::cerr << "Error: --sic-backend requires an argument (ipatov|discrete|fractional)\n";
                print_help(argv[0]);
                return 1;
            }
            const std::string v = argv[++i];
            if (v == "ipatov") {
                cfg.sic_cfg.backend = gr::uwb::sic::SicEstimatorBackend::Ipatov;
            } else if (v == "discrete") {
                cfg.sic_cfg.backend = gr::uwb::sic::SicEstimatorBackend::DiscreteCir;
            } else if (v == "fractional") {
                cfg.sic_cfg.backend = gr::uwb::sic::SicEstimatorBackend::PathFractional;
            } else {
                std::cerr << "Error: Invalid --sic-backend '" << v
                          << "'. Expected ipatov|discrete|fractional.\n";
                print_help(argv[0]);
                return 1;
            }
        } else if (arg == "--sic-comm-code") {
            if (i + 1 >= argc) {
                std::cerr << "Error: --sic-comm-code requires a code index\n";
                print_help(argv[0]);
                return 1;
            }
            cfg.sic_cfg.comm_code = static_cast<size_t>(std::stoul(argv[++i]));
        } else if (arg == "--sic-sync-template") {
            if (i + 1 >= argc) {
                std::cerr << "Error: --sic-sync-template requires a cf32 file path\n";
                print_help(argv[0]);
                return 1;
            }
            cfg.sic_sync_template_file = argv[++i];
        } else if (arg == "--sic-roi-margin") {
            if (i + 1 >= argc) {
                std::cerr << "Error: --sic-roi-margin requires a sample count\n";
                print_help(argv[0]);
                return 1;
            }
            sic_roi_margin = std::stoll(argv[++i]);
        } else if (arg == "--sic-tap-count") {
            if (i + 1 >= argc) {
                std::cerr << "Error: --sic-tap-count requires a tap count\n";
                print_help(argv[0]);
                return 1;
            }
            cfg.sic_cfg.tap_count = static_cast<size_t>(std::stoul(argv[++i]));
        } else if (arg == "--uhd-args") {
            if (i + 1 >= argc) {
                std::cerr << "Error: --uhd-args requires argument string\n";
                return 1;
            }
            cfg.uhd_args = argv[++i];
        } else if (arg == "--uhd-subdev") {
            if (i + 1 >= argc) {
                std::cerr << "Error: --uhd-subdev requires subdev spec\n";
                return 1;
            }
            cfg.uhd_subdev = argv[++i];
        } else if (arg == "--uhd-channel") {
            if (i + 1 >= argc) {
                std::cerr << "Error: --uhd-channel requires channel string\n";
                return 1;
            }
            cfg.uhd_channel = argv[++i];
        } else if (arg == "--freq") {
            if (i + 1 >= argc) {
                std::cerr << "Error: --freq requires center frequency in Hz\n";
                return 1;
            }
            cfg.uhd_center_freq = std::stod(argv[++i]);
        } else if (arg == "--gain") {
            if (i + 1 >= argc) {
                std::cerr << "Error: --gain requires gain in dB\n";
                return 1;
            }
            cfg.uhd_gain = std::stod(argv[++i]);
        } else {
            std::cerr << "Error: Unrecognized option '" << arg << "'\n";
            print_help(argv[0]);
            return 1;
        }
    }

    // Validate the Burst-level multi-core configuration (fail-closed).
    if (!cfg.cpu_cores_dsp.empty() &&
        static_cast<int>(cfg.cpu_cores_dsp.size()) != cfg.dsp_workers) {
        std::cerr << "Error: --cpu-dsp lists " << cfg.cpu_cores_dsp.size()
                  << " CPU core(s) [" << join_ints(cfg.cpu_cores_dsp) << "] but --dsp-workers is "
                  << cfg.dsp_workers
                  << "; the affinity list must contain exactly one core per DSP worker.\n";
        return 1;
    }
    if (cfg.dsp_workers > 1 && cfg.resampler_workers > 1) {
        std::cerr << "Warning: --dsp-workers " << cfg.dsp_workers
                  << " > 1 with --resampler-workers " << cfg.resampler_workers
                  << " > 1; the pipeline forces each DSP worker's nested resampler pool to 1 "
                     "(burst-level multi-core).\n";
    }

    // Configure source type
    if (source_str == "replay") {
        cfg.source_type = StreamSourceType::Replay;
    } else if (source_str == "uhd") {
        cfg.source_type = StreamSourceType::UhdDevice;
    } else {
        std::cerr << "Error: Invalid source '" << source_str << "'. Expected 'replay' or 'uhd'.\n";
        return 1;
    }

    // Configure consumer mode
    if (consumer_str == "sic") {
        cfg.consumer_mode = ConsumerMode::SicOnly;
    } else if (consumer_str == "sensing") {
        cfg.consumer_mode = ConsumerMode::SensingCir;
    } else if (consumer_str == "both") {
        cfg.consumer_mode = ConsumerMode::Both;
    } else {
        std::cerr << "Error: Invalid consumer mode '" << consumer_str
                  << "'. Expected 'sic', 'sensing', or 'both'.\n";
        return 1;
    }

    // Configure replay pacing
    if (replay_mode_str == "fullspeed") {
        cfg.paced_replay = false;
    } else if (replay_mode_str == "paced") {
        cfg.paced_replay = true;
    } else {
        std::cerr << "Error: Invalid replay mode '" << replay_mode_str
                  << "'. Expected 'fullspeed' or 'paced'.\n";
        return 1;
    }

    // Calculate period in seconds
    if (cfg.native_rate <= 0.0) {
        std::cerr << "Error: native rate must be positive.\n";
        return 1;
    }
    cfg.period_s = period_samples / cfg.native_rate;

    // ------------------------------------------------------------------
    // Derive the SIC geometry from the window layout.  Without this the
    // default SicUnifiedConfig has a zero-width search ROI and zero-sized
    // scratch, so sic_process_unified() can only ever fail closed.
    // Shared with benchmark_sc16_pipeline_steady so the two drivers cannot
    // drift apart (pipeline_bench_setup.h).
    // ------------------------------------------------------------------
    bench::derive_sic_geometry(cfg, sic_roi_margin);

    // Handle replay file path
    if (cfg.source_type == StreamSourceType::Replay) {
        if (!file_path.empty()) {
            cfg.replay_file = file_path;
        } else {
            // Default testdata file
            std::string default_cand1 = std::string(UWB_TESTDATA_DIR) + "/uwb_radar/tx_998p4.sc16";
            std::string default_cand2 = "testdata/uwb_radar/tx_998p4.sc16";
            if (file_exists(default_cand1)) {
                cfg.replay_file = default_cand1;
            } else if (file_exists(default_cand2)) {
                cfg.replay_file = default_cand2;
            } else {
                std::cerr << "Warning: No --file specified and default testdata sc16 not found.\n";
            }
        }
    }

    // Benchmark locking if requested
    BenchmarkLockGuard bench_guard;
    if (benchmark_mode) {
        if (!bench_guard.acquire()) {
            std::cerr << "Error: Failed to acquire benchmark lock.\n";
            return 1;
        }
    }

    std::string dsp_affinity_str;
    if (!cfg.cpu_cores_dsp.empty()) {
        dsp_affinity_str = join_ints(cfg.cpu_cores_dsp);
    } else if (cfg.cpu_core_dsp >= 0) {
        if (cfg.dsp_workers == 1) {
            dsp_affinity_str = std::to_string(cfg.cpu_core_dsp);
        } else {
            dsp_affinity_str = std::to_string(cfg.cpu_core_dsp) + ".." +
                               std::to_string(cfg.cpu_core_dsp + cfg.dsp_workers - 1) +
                               " (cpu_core_dsp + w)";
        }
    } else {
        dsp_affinity_str = "unpinned";
    }

    // The pipeline forces each DSP worker's nested resampler pool to 1 so that
    // a burst stays run-to-completion on a single core.
    const std::string resampler_workers_str =
        (cfg.dsp_workers > 1) ? std::string("1 (forced, burst-level multi-core)")
                              : std::to_string(cfg.resampler_workers);

    const char* sic_policy_str = "auto";
    switch (cfg.sic_cfg.policy) {
    case gr::uwb::sic::SicPolicy::PreambleOnly: sic_policy_str = "preamble"; break;
    case gr::uwb::sic::SicPolicy::FullPacket:   sic_policy_str = "fullpacket"; break;
    case gr::uwb::sic::SicPolicy::AutoFallback: sic_policy_str = "auto"; break;
    }
    const char* sic_backend_str = "ipatov";
    switch (cfg.sic_cfg.backend) {
    case gr::uwb::sic::SicEstimatorBackend::Ipatov:         sic_backend_str = "ipatov"; break;
    case gr::uwb::sic::SicEstimatorBackend::DiscreteCir:    sic_backend_str = "discrete"; break;
    case gr::uwb::sic::SicEstimatorBackend::PathFractional: sic_backend_str = "fractional"; break;
    }

    std::cout << "Starting UHD SC16 Pipeline...\n"
              << "  Source:        " << (cfg.source_type == StreamSourceType::Replay ? "Replay (" + cfg.replay_file + ")" : "UHD Device") << "\n"
              << "  Native Rate:   " << cfg.native_rate / 1e6 << " MS/s\n"
              << "  Work Rate:     " << cfg.work_rate / 1e6 << " MS/s\n"
              << "  Schedule:      t0=" << cfg.schedule_t0 << " samples, period=" << period_samples << " samples (" << cfg.period_s * 1e3 << " ms)\n"
              << "  Window Size:   pre=" << cfg.pre_guard_samples << ", cap=" << cfg.capture_samples << ", post=" << cfg.post_guard_samples
              << " (total=" << cfg.native_window_capacity() << ")\n"
              << "  Consumer:      " << consumer_str << "\n"
              << "  Replay Mode:   " << replay_mode_str << "\n"
              << "  Pool Size:     " << cfg.pool_capacity << "\n"
              << "  Max Windows:   " << cfg.max_windows << "\n"
              << "  DSP Workers:   " << cfg.dsp_workers << "\n"
              << "  DSP Affinity:  " << dsp_affinity_str << "\n"
              << "  Resampler:     profile=" << cfg.resampler_profile << ", workers=" << resampler_workers_str << "\n"
              << "  SIC:           policy=" << sic_policy_str << ", backend=" << sic_backend_str
              << ", code=" << cfg.sic_cfg.comm_code
              << ", roi=[" << cfg.sic_cfg.roi_lo << "," << cfg.sic_cfg.roi_hi << ")"
              << ", taps=" << cfg.sic_cfg.tap_count << "\n"
              << "  SIC Template:  "
              << (cfg.sic_sync_template_file.empty() ? std::string("reference_preamble.bin (default)")
                                                     : cfg.sic_sync_template_file)
              << "\n"
              << "  Affinity:      RX=" << cfg.cpu_core_rx << ", DSP=" << cfg.cpu_core_dsp << "\n"
              << std::endl;

    // Instantiate pipeline
    UhdSc16Pipeline pipeline(cfg);

    // Authoritative, post-wait() copy of every terminal window.
    const std::vector<WindowResult>& collected_results = pipeline.results();
    std::atomic<uint64_t> callback_count{ 0 };
    // The callback still runs inside commit_result (so its cost stays in
    // cold_total, exactly as in the historical protocol), but the authoritative
    // per-window records are read from pipeline.results() after wait(): the
    // callback fires BEFORE commit_time is stamped on the stored result, so a
    // copy taken here would always carry a default-initialised commit_time.
    pipeline.set_result_callback([&](const WindowResult& res) {
        (void)res;
        callback_count.fetch_add(1, std::memory_order_relaxed);
    });

    if (!pipeline.initialize()) {
        std::cerr << "Error: Failed to initialize UhdSc16Pipeline.\n";
        return 1;
    }

    auto start_time = std::chrono::steady_clock::now();
    pipeline.start();
    // Freeze the measurement contract: startup_ready ends when every DSP
    // worker has published Ready (or the pipeline failed).  This call is a
    // no-op for the historical cold-start number because wait() would have
    // waited for readiness anyway.
    const bool ready_ok = pipeline.wait_ready(30000);
    auto ready_time = std::chrono::steady_clock::now();
    pipeline.wait();
    auto end_time = std::chrono::steady_clock::now();

    double total_elapsed_s = std::chrono::duration<double>(end_time - start_time).count();
    double startup_ready_ms =
        std::chrono::duration<double, std::milli>(ready_time - start_time).count();

    // Verify ledger invariants
    const auto& ledger = pipeline.ledger();
    std::string inv_err;
    bool inv_ok = ledger.verify_invariants(&inv_err);

    // Filter completed windows
    std::vector<WindowResult> completed_windows;
    for (const auto& r : collected_results) {
        if (r.state == WindowTerminalState::Completed) {
            completed_windows.push_back(r);
        }
    }

    // Compute timing vectors
    std::vector<double> v_slice, v_queue, v_convert, v_resample, v_sic, v_sensing, v_e2e;
    v_slice.reserve(completed_windows.size());
    v_queue.reserve(completed_windows.size());
    v_convert.reserve(completed_windows.size());
    v_resample.reserve(completed_windows.size());
    v_sic.reserve(completed_windows.size());
    v_sensing.reserve(completed_windows.size());
    v_e2e.reserve(completed_windows.size());

    size_t sic_executed_count = 0;
    size_t sic_committed_count = 0;
    double total_suppression_db = 0.0;
    double total_resample_us = 0.0;

    size_t radar_executed_count = 0;
    size_t radar_sfd_found_count = 0;

    for (const auto& r : completed_windows) {
        v_slice.push_back(r.timing.slice_us);
        v_queue.push_back(r.timing.queue_us);
        v_convert.push_back(r.timing.convert_us);
        v_resample.push_back(r.timing.resample_us);
        v_sic.push_back(r.timing.sic_us);
        v_sensing.push_back(r.timing.sensing_us);
        v_e2e.push_back(r.timing.e2e_us);
        total_resample_us += r.timing.resample_us;

        if (r.sic_executed) {
            sic_executed_count++;
            if (r.sic_committed) {
                sic_committed_count++;
                total_suppression_db += static_cast<double>(r.actual_suppression_db);
            }
        }
        if (r.radar_executed) {
            radar_executed_count++;
            if (r.radar_sfd_found) {
                radar_sfd_found_count++;
            }
        }
    }

    PhaseStats st_slice = compute_phase_stats(v_slice);
    PhaseStats st_queue = compute_phase_stats(v_queue);
    PhaseStats st_convert = compute_phase_stats(v_convert);
    PhaseStats st_resample = compute_phase_stats(v_resample);
    PhaseStats st_sic = compute_phase_stats(v_sic);
    PhaseStats st_sensing = compute_phase_stats(v_sensing);
    PhaseStats st_e2e = compute_phase_stats(v_e2e);

    // ------------------------------------------------------------------
    // Steady-region accounting (2026-11-11 round 2 measurement contract).
    // The historical cold-start numbers above are NOT redefined.  The steady
    // region is a contiguous slice of COMPLETED windows in commit order:
    // [--warmup-windows, +--measure-windows).  No window inside it is skipped,
    // so a drop or a slow window can never be filtered out.
    // ------------------------------------------------------------------
    std::vector<double> m_convert, m_resample, m_sic, m_sensing, m_e2e;
    size_t measured_begin =
        (warmup_windows > 0)
            ? std::min<size_t>(static_cast<size_t>(warmup_windows),
                               completed_windows.size())
            : 0;
    size_t measured_end = completed_windows.size();
    if (measure_windows > 0) {
        measured_end =
            std::min<size_t>(measured_begin + static_cast<size_t>(measure_windows),
                             completed_windows.size());
    }
    for (size_t i = measured_begin; i < measured_end; ++i) {
        const auto& r = completed_windows[i];
        m_convert.push_back(r.timing.convert_us);
        m_resample.push_back(r.timing.resample_us);
        m_sic.push_back(r.timing.sic_us);
        m_sensing.push_back(r.timing.sensing_us);
        m_e2e.push_back(r.timing.e2e_us);
    }
    const PhaseStats mst_convert = compute_phase_stats(m_convert);
    const PhaseStats mst_resample = compute_phase_stats(m_resample);
    const PhaseStats mst_sic = compute_phase_stats(m_sic);
    const PhaseStats mst_sensing = compute_phase_stats(m_sensing);
    const PhaseStats mst_e2e = compute_phase_stats(m_e2e);

    // Steady interval boundary: the commit timestamps of the FIRST and LAST
    // measured windows.  The completed-window count for that interval is
    // (n-1), because n commit stamps delimit n-1 inter-completion intervals.
    // Using commit cadence instead of the RX push of the first window removes
    // the RX read-ahead that happened BEFORE the region's processing, which
    // otherwise inflates us/window by the look-ahead depth.
    bool have_steady = false;
    std::chrono::steady_clock::time_point t_first_push{};
    std::chrono::steady_clock::time_point t_first_commit{};
    std::chrono::steady_clock::time_point t_last_commit{};
    for (size_t i = measured_begin; i < measured_end; ++i) {
        const auto& r = completed_windows[i];
        if (r.commit_time == std::chrono::steady_clock::time_point{}) {
            continue;
        }
        if (!have_steady) {
            t_first_commit = r.commit_time;
            t_last_commit = r.commit_time;
            t_first_push = r.rx_push_time;
            have_steady = true;
        } else {
            if (r.commit_time < t_first_commit) {
                t_first_commit = r.commit_time;
            }
            if (r.commit_time > t_last_commit) {
                t_last_commit = r.commit_time;
            }
            if (r.rx_push_time < t_first_push) {
                t_first_push = r.rx_push_time;
            }
        }
    }
    const size_t measured_n = measured_end - measured_begin;
    const size_t steady_span_windows = (measured_n > 1) ? (measured_n - 1) : 0;
    const double steady_s =
        have_steady
            ? std::chrono::duration<double>(t_last_commit - t_first_commit).count()
            : 0.0;
    const double steady_lookahead_ms =
        have_steady
            ? std::chrono::duration<double, std::milli>(t_first_commit - t_first_push).count()
            : 0.0;
    const double steady_window_us =
        (steady_s > 0.0) ? steady_s * 1e6 / static_cast<double>(steady_span_windows) : 0.0;
    const double steady_window_rate =
        (steady_s > 0.0) ? static_cast<double>(steady_span_windows) / steady_s : 0.0;
    const double steady_sample_msps =
        (steady_s > 0.0)
            ? static_cast<double>(steady_span_windows) *
                  static_cast<double>(cfg.native_window_capacity()) / steady_s / 1e6
            : 0.0;
    const double drain_tail_ms =
        have_steady
            ? std::chrono::duration<double, std::milli>(end_time - t_last_commit).count()
            : 0.0;
    const double rx_read_msps =
        (total_elapsed_s > 0.0)
            ? static_cast<double>(ledger.total_samples_received) / total_elapsed_s / 1e6
            : 0.0;

    // Aggregate per DSP worker for the Burst-level multi-core report.  Timing
    // means only use Completed windows; drops/cancelled land in the
    // "unassigned" bucket (worker_id outside [0, dsp_workers)).
    struct WorkerAccumulator {
        size_t windows = 0;
        size_t completed = 0;
        std::vector<double> e2e_us;
        std::vector<double> resample_us;
        std::vector<double> sic_us;
    };

    std::vector<WorkerAccumulator> worker_acc(static_cast<size_t>(cfg.dsp_workers));
    WorkerAccumulator unassigned_acc;
    for (const auto& r : collected_results) {
        const bool assigned = (r.worker_id >= 0 && r.worker_id < cfg.dsp_workers);
        WorkerAccumulator& acc =
            assigned ? worker_acc[static_cast<size_t>(r.worker_id)] : unassigned_acc;
        acc.windows++;
        if (r.state == WindowTerminalState::Completed) {
            acc.completed++;
            acc.e2e_us.push_back(r.timing.e2e_us);
            acc.resample_us.push_back(r.timing.resample_us);
            acc.sic_us.push_back(r.timing.sic_us);
        }
    }

    std::vector<WorkerDistribution> worker_rows;
    worker_rows.reserve(worker_acc.size() + 1);
    for (size_t w = 0; w < worker_acc.size(); ++w) {
        WorkerDistribution row;
        row.worker_id = static_cast<int>(w);
        row.windows = worker_acc[w].windows;
        row.completed = worker_acc[w].completed;
        row.e2e = compute_phase_stats(worker_acc[w].e2e_us);
        row.resample = compute_phase_stats(worker_acc[w].resample_us);
        row.sic = compute_phase_stats(worker_acc[w].sic_us);
        worker_rows.push_back(std::move(row));
    }
    if (unassigned_acc.windows > 0) {
        WorkerDistribution row;
        row.worker_id = -1;
        row.windows = unassigned_acc.windows;
        row.completed = unassigned_acc.completed;
        row.e2e = compute_phase_stats(unassigned_acc.e2e_us);
        row.resample = compute_phase_stats(unassigned_acc.resample_us);
        row.sic = compute_phase_stats(unassigned_acc.sic_us);
        worker_rows.push_back(std::move(row));
    }

    // Print summary and tables
    std::cout << "\n================================================================================\n"
              << "                    UHD SC16 Direct RX Pipeline Results\n"
              << "================================================================================\n"
              << "Ledger Invariants Check: " << (inv_ok ? "PASS" : "FAIL (" + inv_err + ")") << "\n\n"
              << "Pipeline Summary:\n"
              << "  Offered Windows:                   " << std::setw(10) << ledger.offered_windows << "\n"
              << "  Admitted Windows:                  " << std::setw(10) << ledger.admitted_windows << "\n"
              << "  Completed Windows:                 " << std::setw(10) << ledger.completed_windows << "\n"
              << "  Dropped Windows (Buffer Full):     " << std::setw(10) << ledger.dropped_admission << "\n"
              << "  Invalid Gap Windows:               " << std::setw(10) << (ledger.invalid_gap_before_admission + ledger.invalid_gap_after_admission) << "\n"
              << "  Cancelled Windows:                 " << std::setw(10) << ledger.cancelled_windows << "\n"
              << "  Processing Failed:                 " << std::setw(10) << ledger.processing_failed << "\n"
              << "  Total Samples Processed:           " << std::setw(10) << ledger.total_samples_received << "\n"
              << "  Sample Rate:                       " << std::fixed << std::setprecision(1) << cfg.native_rate / 1e6 << " MS/s\n"
              << "  Total Wall Time:                   " << std::fixed << std::setprecision(3) << total_elapsed_s << " s\n"
              << "\n";

    // Print Phase Timing Table
    std::cout << "Phase Timing Statistics (Completed Windows: " << completed_windows.size() << "):\n"
              << "  --------------------------------------------------------------------------------\n"
              << "  Phase                  Mean (us)     P50 (us)     P95 (us)     Min (us)     Max (us)\n"
              << "  --------------------------------------------------------------------------------\n";

    auto print_row = [](const char* name, const PhaseStats& st) {
        std::cout << "  " << std::left << std::setw(18) << name << " "
                  << std::right << std::fixed << std::setprecision(2)
                  << std::setw(11) << st.mean << "  "
                  << std::setw(11) << st.p50 << "  "
                  << std::setw(11) << st.p95 << "  "
                  << std::setw(11) << st.min << "  "
                  << std::setw(11) << st.max << "\n";
    };

    print_row("slice_us", st_slice);
    print_row("queue_us", st_queue);
    print_row("convert_us", st_convert);
    print_row("resample_us", st_resample);
    print_row("sic_us", st_sic);
    print_row("sensing_us", st_sensing);
    print_row("e2e_us", st_e2e);

    std::cout << "  --------------------------------------------------------------------------------\n\n";

    // Lifecycle + steady-region report (new fields; the cold-start numbers
    // above keep their historical meaning).
    std::cout << "Lifecycle / Measurement Contract:\n"
              << "  startup_ready_ms:                  " << std::fixed << std::setprecision(3)
              << startup_ready_ms << (ready_ok ? "" : "  [READY FAILED]") << "\n"
              << "  drain_tail_ms:                     " << std::fixed << std::setprecision(3)
              << drain_tail_ms << "\n"
              << "  rx_total_received_samples:         " << ledger.total_samples_received << "\n"
              << "  rx_read_msps (over cold_total):    " << std::fixed << std::setprecision(2)
              << rx_read_msps << " MS/s\n"
              << "  warmup_windows:                    " << warmup_windows << "\n"
              << "  measure_windows_requested:         " << measure_windows << "\n"
              << "  measure_windows_actual:            " << measured_n << "\n"
              << "  steady_interval_s:                 " << std::fixed << std::setprecision(6)
              << steady_s << "\n"
              << "  steady_interval_boundary:          first measured commit_time .. last measured commit_time\n"
              << "  steady_span_windows:               " << steady_span_windows << "\n"
              << "  steady_lookahead_ms:               " << std::fixed << std::setprecision(3)
              << steady_lookahead_ms << " (first push -> first measured commit)\n"
              << "  steady_window_us:                  " << std::fixed << std::setprecision(2)
              << steady_window_us << " us/window\n"
              << "  steady_window_rate:                " << std::fixed << std::setprecision(2)
              << steady_window_rate << " win/s\n"
              << "  steady_window_sample_throughput:   " << std::fixed << std::setprecision(2)
              << steady_sample_msps << " MS/s\n"
              << "\n";

    std::cout << "Steady-Region Service Times (Completed Windows: " << measured_n << "):\n"
              << "  --------------------------------------------------------------------------------\n"
              << "  Phase                  Mean (us)     P50 (us)     P95 (us)     P99 (us)     n\n"
              << "  --------------------------------------------------------------------------------\n";
    auto print_steady_row = [](const char* name, const PhaseStats& st) {
        std::cout << "  " << std::left << std::setw(18) << name << " "
                  << std::right << std::fixed << std::setprecision(2)
                  << std::setw(11) << st.mean << "  "
                  << std::setw(11) << st.p50 << "  "
                  << std::setw(11) << st.p95 << "  "
                  << std::setw(11) << st.p99 << "  "
                  << std::setw(8) << st.n << "\n";
    };
    print_steady_row("convert_us", mst_convert);
    print_steady_row("resample_us", mst_resample);
    print_steady_row("sic_us", mst_sic);
    print_steady_row("sensing_us", mst_sensing);
    print_steady_row("e2e_us", mst_e2e);
    std::cout << "  --------------------------------------------------------------------------------\n\n";

    if (cfg.enable_rx_diagnostics) {
        const auto& rx = pipeline.rx_diagnostics();
        std::cout << "RX Diagnostics (cumulative):\n"
                  << "  read_calls=" << rx.read_calls
                  << " samples=" << rx.samples_read
                  << " bytes=" << rx.bytes_read << "\n"
                  << "  published=" << rx.windows_published
                  << " dropped=" << rx.windows_dropped
                  << " slot_copy_bytes=" << rx.slot_copy_bytes << "\n"
                  << "  stream_buf_resize_count=" << rx.stream_buf_resize_count
                  << " zero_init_bytes=" << rx.stream_buf_zero_init_bytes
                  << " compact_move_bytes=" << rx.compact_move_bytes
                  << " max_buffered_samples=" << rx.max_buffered_samples << "\n"
                  << "  read_us=" << std::fixed << std::setprecision(1) << rx.read_us
                  << " assemble_us=" << rx.assemble_us
                  << " slot_copy_us=" << rx.slot_copy_us
                  << " dispatch_us=" << rx.dispatch_us
                  << " ledger_wait_us=" << rx.ledger_wait_us << "\n\n";
    }
    if (cfg.enable_commit_diagnostics) {
        const auto& cm = pipeline.commit_diagnostics();
        std::cout << "Commit Diagnostics:\n"
                  << "  commit_calls=" << cm.commit_calls
                  << " results_emitted=" << cm.results_emitted
                  << " max_out_of_order_span=" << cm.max_out_of_order_span << "\n"
                  << "  lock_wait_us=" << std::fixed << std::setprecision(1) << cm.lock_wait_us
                  << " reorder_us=" << cm.reorder_us
                  << " callback_us=" << cm.callback_us << "\n\n";
    }


    // Burst-level multi-core throughput and per-worker distribution
    print_system_throughput(cfg.dsp_workers,
                            completed_windows.size(),
                            total_elapsed_s,
                            cfg.native_window_capacity(),
                            total_resample_us);
    print_worker_distribution(worker_rows, collected_results.size(), cfg.dsp_workers);

    // Print SIC / Sensing metrics
    double mean_suppression = (sic_committed_count > 0) ? (total_suppression_db / static_cast<double>(sic_committed_count)) : 0.0;
    double sic_commit_rate = (sic_executed_count > 0) ? (100.0 * static_cast<double>(sic_committed_count) / static_cast<double>(sic_executed_count)) : 0.0;
    double radar_sfd_rate = (radar_executed_count > 0) ? (100.0 * static_cast<double>(radar_sfd_found_count) / static_cast<double>(radar_executed_count)) : 0.0;

    std::cout << "SIC / Sensing Metrics:\n"
              << "  SIC Executed Windows:              " << std::setw(10) << sic_executed_count << "\n"
              << "  SIC Committed Windows:             " << std::setw(10) << sic_committed_count
              << " (" << std::fixed << std::setprecision(1) << sic_commit_rate << "%)\n"
              << "  Mean Actual Suppression:           " << std::fixed << std::setprecision(2) << mean_suppression << " dB\n"
              << "  Radar Executed Windows:            " << std::setw(10) << radar_executed_count << "\n"
              << "  Radar SFD Detection Count:         " << std::setw(10) << radar_sfd_found_count
              << " / " << radar_executed_count
              << " (" << std::fixed << std::setprecision(1) << radar_sfd_rate << "%)\n"
              << "================================================================================\n"
              << std::endl;

    // ---- machine-readable output (same schema as the steady driver) ----
    bench::MetaList meta;
    meta.emplace_back("driver", bench::meta_string("uwb_uhd_sc16_pipeline_runner"));
    meta.emplace_back("label", bench::meta_string(label));
    meta.emplace_back("file", bench::meta_string(cfg.replay_file));
    meta.emplace_back("native_rate", bench::json_number(cfg.native_rate));
    meta.emplace_back("work_rate", bench::json_number(cfg.work_rate));
    meta.emplace_back("capture_samples", bench::json_number(static_cast<uint64_t>(cfg.capture_samples)));
    meta.emplace_back("pre_guard_samples", bench::json_number(static_cast<uint64_t>(cfg.pre_guard_samples)));
    meta.emplace_back("post_guard_samples", bench::json_number(static_cast<uint64_t>(cfg.post_guard_samples)));
    meta.emplace_back("period_samples", bench::json_number(period_samples));
    meta.emplace_back("dsp_workers", bench::json_number(static_cast<int64_t>(cfg.dsp_workers)));
    meta.emplace_back("pool_capacity", bench::json_number(static_cast<uint64_t>(cfg.pool_capacity)));
    meta.emplace_back("resampler_profile", bench::meta_string(cfg.resampler_profile));
    meta.emplace_back("consumer", bench::meta_string(consumer_str));
    meta.emplace_back("max_windows", bench::json_number(static_cast<uint64_t>(cfg.max_windows)));
    meta.emplace_back("warmup_windows", bench::json_number(warmup_windows));
    meta.emplace_back("measure_windows_requested", bench::json_number(measure_windows));
    meta.emplace_back("measure_windows_actual", bench::json_number(static_cast<uint64_t>(measured_n)));
    meta.emplace_back("diag_rx", bench::meta_bool(cfg.enable_rx_diagnostics));
    meta.emplace_back("diag_commit", bench::meta_bool(cfg.enable_commit_diagnostics));
    meta.emplace_back("ready_ok", bench::meta_bool(ready_ok && !pipeline.failed()));
    meta.emplace_back("ledger_invariants_ok", bench::meta_bool(inv_ok));
    meta.emplace_back("cold_total_s", bench::json_number(total_elapsed_s));
    meta.emplace_back("startup_ready_ms", bench::json_number(startup_ready_ms));
    meta.emplace_back("drain_tail_ms", bench::json_number(drain_tail_ms));
    meta.emplace_back("steady_interval_s", bench::json_number(steady_s));
    meta.emplace_back("steady_span_windows", bench::json_number(static_cast<uint64_t>(steady_span_windows)));
    meta.emplace_back("steady_lookahead_ms", bench::json_number(steady_lookahead_ms));
    meta.emplace_back("steady_window_us", bench::json_number(steady_window_us));
    meta.emplace_back("steady_window_rate", bench::json_number(steady_window_rate));
    meta.emplace_back("steady_window_sample_msps", bench::json_number(steady_sample_msps));
    meta.emplace_back("rx_read_msps", bench::json_number(rx_read_msps));
    meta.emplace_back("resample_us_mean", bench::json_number(st_resample.mean));
    meta.emplace_back("resample_us_p50", bench::json_number(st_resample.p50));
    meta.emplace_back("resample_us_p95", bench::json_number(st_resample.p95));
    meta.emplace_back("sic_us_mean", bench::json_number(st_sic.mean));
    meta.emplace_back("e2e_us_mean", bench::json_number(st_e2e.mean));
    meta.emplace_back("e2e_us_p50", bench::json_number(st_e2e.p50));
    meta.emplace_back("e2e_us_p95", bench::json_number(st_e2e.p95));
    meta.emplace_back("rx_read_us", bench::json_number(pipeline.rx_diagnostics().read_us));
    meta.emplace_back("rx_assemble_us", bench::json_number(pipeline.rx_diagnostics().assemble_us));
    meta.emplace_back("rx_slot_copy_us", bench::json_number(pipeline.rx_diagnostics().slot_copy_us));
    meta.emplace_back("rx_dispatch_us", bench::json_number(pipeline.rx_diagnostics().dispatch_us));
    meta.emplace_back("rx_stream_buf_resize_count",
                      bench::json_number(pipeline.rx_diagnostics().stream_buf_resize_count));
    meta.emplace_back("rx_zero_init_bytes",
                      bench::json_number(pipeline.rx_diagnostics().stream_buf_zero_init_bytes));
    meta.emplace_back("rx_compact_move_bytes",
                      bench::json_number(pipeline.rx_diagnostics().compact_move_bytes));
    meta.emplace_back("commit_lock_wait_us",
                      bench::json_number(pipeline.commit_diagnostics().lock_wait_us));
    meta.emplace_back("commit_reorder_us",
                      bench::json_number(pipeline.commit_diagnostics().reorder_us));
    meta.emplace_back("commit_callback_us",
                      bench::json_number(pipeline.commit_diagnostics().callback_us));
    meta.emplace_back("commit_max_out_of_order_span",
                      bench::json_number(pipeline.commit_diagnostics().max_out_of_order_span));

    if (!bench::write_results_json(json_out, meta, ledger, collected_results)) {
        std::cerr << "Error: cannot write " << json_out << "\n";
        return 1;
    }
    if (!bench::write_results_csv(csv_out, collected_results)) {
        std::cerr << "Error: cannot write " << csv_out << "\n";
        return 1;
    }

    return 0;
}
