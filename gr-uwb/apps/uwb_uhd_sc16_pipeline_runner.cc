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
    return s;
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
              << "  --cpu-dsp <core>            Affinity CPU core for DSP thread (-1 = unpinned)\n"
              << "  --pool-size <n>             Ring pool slot capacity (default: 8)\n"
              << "  --max-windows <n>           Maximum windows to process (default: 100)\n"
              << "  --resampler-profile <name>  Resampler profile (quality_minorder|realtime_minorder|quality|realtime, default: quality_minorder)\n"
              << "  --resampler-workers <n>     Resampler thread worker count (default: 1)\n"
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
                std::cerr << "Error: --cpu-dsp requires a CPU core index\n";
                print_help(argv[0]);
                return 1;
            }
            cfg.cpu_core_dsp = std::stoi(argv[++i]);
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
              << "  Resampler:     profile=" << cfg.resampler_profile << ", workers=" << cfg.resampler_workers << "\n"
              << "  Affinity:      RX=" << cfg.cpu_core_rx << ", DSP=" << cfg.cpu_core_dsp << "\n"
              << std::endl;

    // Instantiate pipeline
    UhdSc16Pipeline pipeline(cfg);

    std::vector<WindowResult> collected_results;
    std::mutex results_mutex;
    pipeline.set_result_callback([&](const WindowResult& res) {
        std::lock_guard<std::mutex> lock(results_mutex);
        collected_results.push_back(res);
    });

    if (!pipeline.initialize()) {
        std::cerr << "Error: Failed to initialize UhdSc16Pipeline.\n";
        return 1;
    }

    auto start_time = std::chrono::steady_clock::now();
    pipeline.start();
    pipeline.wait();
    auto end_time = std::chrono::steady_clock::now();

    double total_elapsed_s = std::chrono::duration<double>(end_time - start_time).count();

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

    return 0;
}
