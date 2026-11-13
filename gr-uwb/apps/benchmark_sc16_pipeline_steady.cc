/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Steady-state benchmark driver for the UHD SC16 burst pipeline.
 *
 * Measurement contract (docs/performance/接口冻结_UHD_SC16第二轮优化_2026-11-11.md §5):
 *   cold_total        : start() -> wait() return, historical semantics
 *   startup_ready     : start() -> all workers Ready
 *   warm_service      : per-window resample/convert/SIC/sensing over the
 *                       measurement region only
 *   steady_throughput : completed windows inside
 *                       [min rx_push_time, max commit_time] / that interval
 *   drain_tail        : wait() return - max commit_time
 *
 * The feeder is a looping replay file with WINDOW-LEVEL flow control so that a
 * bounded pool can be used without turning the run into a drop storm.  The
 * flow control lives in this benchmark feeder only; it is never present in the
 * production RX path.  It is reported explicitly (`flow_waits`,
 * `flow_wait_us`, `flow_in_flight`), so an input-limited result cannot be
 * mistaken for a capacity result.  The file I/O of the feeder is reported
 * separately (`feeder_read_us`, `feeder_read_calls`, `feeder_read_bytes`);
 * `flow_wait_us` covers only the in-flight gate and `feeder_read_us` covers
 * only the ::read() syscalls, so neither is a stand-in for the pipeline's
 * `rx.read_us` (which spans the whole `read_samples()` call).
 */

#include <gnuradio/uwb/uwb_uhd_sc16_pipeline.h>

#include "pipeline_bench_setup.h"
#include "pipeline_result_json.h"

#include <algorithm>
#include <atomic>
#include <condition_variable>
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
#include <numeric>
#include <string>
#include <sys/file.h>
#include <sys/stat.h>
#include <thread>
#include <unistd.h>
#include <vector>

using namespace gr::uwb::pipeline;

namespace {

struct BenchLockGuard {
    int fd = -1;
    bool locked = false;
    bool acquire()
    {
        fd = ::open("/tmp/uwb-sic-benchmark.lock", O_CREAT | O_RDWR, 0666);
        if (fd < 0) {
            std::cerr << "Warning: cannot open /tmp/uwb-sic-benchmark.lock: "
                      << std::strerror(errno) << "\n";
            return false;
        }
        std::cout << "[Benchmark] Waiting for exclusive lock "
                     "/tmp/uwb-sic-benchmark.lock...\n";
        if (::flock(fd, LOCK_EX) != 0) {
            std::cerr << "Warning: cannot flock: " << std::strerror(errno) << "\n";
            ::close(fd);
            fd = -1;
            return false;
        }
        locked = true;
        std::cout << "[Benchmark] Acquired exclusive lock\n";
        return true;
    }
    ~BenchLockGuard()
    {
        if (locked && fd >= 0) {
            ::flock(fd, LOCK_UN);
            ::close(fd);
            std::cout << "[Benchmark] Released lock\n";
        }
    }
};

struct PhaseStats {
    size_t n = 0;
    double mean = 0.0, p50 = 0.0, p95 = 0.0, p99 = 0.0, min = 0.0, max = 0.0;
};

PhaseStats compute_phase_stats(std::vector<double>& v)
{
    PhaseStats s;
    if (v.empty()) {
        return s;
    }
    std::sort(v.begin(), v.end());
    s.n = v.size();
    s.min = v.front();
    s.max = v.back();
    s.mean = std::accumulate(v.begin(), v.end(), 0.0) / static_cast<double>(s.n);
    auto at = [&](double p) {
        if (s.n == 1) {
            return v[0];
        }
        const double idx = p * static_cast<double>(s.n - 1);
        size_t pos = static_cast<size_t>(std::llround(idx));
        if (pos >= s.n) {
            pos = s.n - 1;
        }
        return v[pos];
    };
    s.p50 = at(0.50);
    s.p95 = at(0.95);
    s.p99 = at(0.99);
    return s;
}

// ---------------------------------------------------------------------------
// Looping replay feeder with optional window-level flow control.
// ---------------------------------------------------------------------------
class LoopingReplaySc16Source : public ISc16StreamSource {
public:
    LoopingReplaySc16Source(std::string path, double rate, bool loop)
        : path_(std::move(path)), rate_(rate), loop_(loop), start_(Clock::now())
    {
    }

    ~LoopingReplaySc16Source() override { close(); }

    bool open() override
    {
        close();
        fd_ = ::open(path_.c_str(), O_RDONLY);
        if (fd_ < 0) {
            return false;
        }
        struct stat st {};
        if (::fstat(fd_, &st) != 0 || st.st_size <= 0) {
            close();
            return false;
        }
        file_samples_ = static_cast<int64_t>(st.st_size) /
                        static_cast<int64_t>(sizeof(std::complex<int16_t>));
        if (file_samples_ <= 0) {
            close();
            return false;
        }
        eof_ = false;
        abs_ = 0;
        delivered_ = 0;
        loops_ = 0;
        start_ = Clock::now();
        return true;
    }

    void close() override
    {
        if (fd_ >= 0) {
            ::close(fd_);
            fd_ = -1;
        }
        eof_ = true;
    }

    // Window-level in-flight limit.  `period` is the scheduled window spacing
    // in native samples; `completed` is maintained by the driver's result
    // callback.  Never used by the production RX path.
    //
    // The gate is a condition variable, not a yield() spin: at 4k windows the
    // per-window budget is ~5 us, so a scheduler round-trip on every restart
    // would dominate the measurement and mask real RX changes.
    void set_flow_control(int64_t period,
                          int64_t max_in_flight,
                          std::atomic<uint64_t>* completed,
                          std::mutex* gate_mutex,
                          std::condition_variable* gate_cv,
                          double timeout_ms)
    {
        flow_period_ = period;
        flow_max_in_flight_ = max_in_flight;
        flow_completed_ = completed;
        flow_gate_mutex_ = gate_mutex;
        flow_gate_cv_ = gate_cv;
        flow_timeout_ms_ = timeout_ms;
        flow_enabled_ = (period > 0 && max_in_flight > 0 && completed != nullptr &&
                         gate_mutex != nullptr && gate_cv != nullptr);
    }

    uint64_t flow_waits() const { return flow_waits_; }
    double flow_wait_us() const { return flow_wait_us_; }
    uint64_t loops() const { return loops_; }
    uint64_t delivered_samples() const { return delivered_; }
    int64_t file_samples() const { return file_samples_; }

    // File I/O cost of the feeder, kept strictly separate from the flow-control
    // wait above.  `read_samples()` blocks both on the in-flight gate and in
    // ::read(); the pipeline-side `rx.read_us` covers the whole call, so it
    // cannot be read as a page-cache/bandwidth limit.  These three counters
    // measure only the ::read() syscalls made by this feeder.  A call rejected
    // by the EOF/zero-length guard performs no ::read() and is not counted.
    uint64_t read_calls() const { return read_calls_; }
    uint64_t read_bytes() const { return read_bytes_; }
    double read_us() const { return read_us_; }

    size_t read_samples(std::complex<int16_t>* dest,
                        size_t max_samples,
                        StreamReadMeta& meta) override
    {
        meta = StreamReadMeta{};
        if (fd_ < 0 || max_samples == 0 || !dest) {
            eof_ = true;
            meta.is_eof = true;
            return 0;
        }
        maybe_wait_for_flow();

        const size_t want_bytes = max_samples * sizeof(std::complex<int16_t>);
        char* p = reinterpret_cast<char*>(dest);
        size_t got = 0;
        // Time only the ::read() syscalls.  The flow-control wait above and the
        // ::lseek() loop restart are deliberately excluded, so this is file I/O
        // and not the in-flight gate.
        bool did_read = false;
        double read_us_local = 0.0;
        while (got < want_bytes) {
            const auto read_t0 = Clock::now();
            ssize_t n = ::read(fd_, p + got, want_bytes - got);
            read_us_local +=
                std::chrono::duration<double, std::micro>(Clock::now() - read_t0).count();
            did_read = true;
            if (n > 0) {
                got += static_cast<size_t>(n);
                continue;
            }
            if (n == 0) {
                if (loop_) {
                    if (::lseek(fd_, 0, SEEK_SET) != 0) {
                        eof_ = true;
                        break;
                    }
                    ++loops_;
                    continue;
                }
                eof_ = true;
                break;
            }
            if (errno == EINTR) {
                continue;
            }
            meta.error_code = errno;
            break;
        }
        if (did_read) {
            ++read_calls_;
            read_bytes_ += got;
            read_us_ += read_us_local;
        }

        const size_t samples = got / sizeof(std::complex<int16_t>);
        meta.start_sample_abs = abs_;
        meta.has_time_spec = true;
        meta.time_spec_s = static_cast<double>(abs_) / rate_;
        meta.is_gap = false;
        meta.gap_samples = 0;
        meta.is_eof = eof_;

        abs_ += static_cast<int64_t>(samples);
        delivered_ += samples;
        return samples;
    }

    double actual_rx_rate() const override { return rate_; }
    bool is_eof() const override { return eof_; }

private:
    using Clock = std::chrono::steady_clock;

    void maybe_wait_for_flow()
    {
        if (!flow_enabled_) {
            return;
        }
        auto in_flight = [&]() -> int64_t {
            const int64_t offered =
                static_cast<int64_t>(delivered_) / flow_period_;
            const int64_t done =
                static_cast<int64_t>(flow_completed_->load(std::memory_order_relaxed));
            return offered - done;
        };
        if (in_flight() <= flow_max_in_flight_) {
            return;
        }
        const auto t0 = Clock::now();
        ++flow_waits_;
        // Adaptive gate: spin briefly first (a 4k window costs ~5 us, so a
        // futex round-trip would dominate and mask real RX changes), then fall
        // back to sleeping so a long backlog never burns a core.
        constexpr double kSpinUs = 25.0;
        while (in_flight() > flow_max_in_flight_) {
            const double spun_us =
                std::chrono::duration<double, std::micro>(Clock::now() - t0).count();
            if (spun_us > kSpinUs) {
                break;
            }
            std::this_thread::yield();
        }
        if (in_flight() > flow_max_in_flight_) {
            std::unique_lock<std::mutex> lk(*flow_gate_mutex_);
            flow_gate_cv_->wait_for(
                lk, std::chrono::milliseconds(static_cast<long long>(flow_timeout_ms_)),
                [this, &in_flight] { return in_flight() <= flow_max_in_flight_; });
            lk.unlock();
        }
        flow_wait_us_ +=
            std::chrono::duration<double, std::micro>(Clock::now() - t0).count();
    }

    std::string path_;
    double rate_ = 998.4e6;
    bool loop_ = true;
    int fd_ = -1;
    bool eof_ = false;
    int64_t file_samples_ = 0;
    int64_t abs_ = 0;
    uint64_t delivered_ = 0;
    uint64_t loops_ = 0;
    Clock::time_point start_;

    bool flow_enabled_ = false;
    int64_t flow_period_ = 0;
    int64_t flow_max_in_flight_ = 0;
    double flow_timeout_ms_ = 2000.0;
    std::atomic<uint64_t>* flow_completed_ = nullptr;
    std::mutex* flow_gate_mutex_ = nullptr;
    std::condition_variable* flow_gate_cv_ = nullptr;
    uint64_t flow_waits_ = 0;
    double flow_wait_us_ = 0.0;
    uint64_t read_calls_ = 0;
    uint64_t read_bytes_ = 0;
    double read_us_ = 0.0;
};

bool file_exists(const std::string& path)
{
    struct stat st {};
    return ::stat(path.c_str(), &st) == 0;
}

bool parse_bool_flag(const std::string& v, bool& out)
{
    if (v == "1" || v == "true" || v == "yes" || v == "on") {
        out = true;
        return true;
    }
    if (v == "0" || v == "false" || v == "no" || v == "off") {
        out = false;
        return true;
    }
    return false;
}

bool parse_sfd_mode_literal(const std::string& v, const char*& out)
{
    static constexpr const char* kModes[] = {
        "decawave", "ieee", "4z1", "4z2", "4z3", "4z4"
    };
    for (const char* m : kModes) {
        if (v == m) {
            out = m;
            return true;
        }
    }
    return false;
}

bool parse_cli_pulse(const std::string& v, gr::uwb::mod::PulseSpec& out)
{
    gr::uwb::mod::PulseShape shape = gr::uwb::mod::PulseShape::Legacy;
    if (!gr::uwb::mod::parse_pulse_shape(v, shape) ||
        shape == gr::uwb::mod::PulseShape::External) {
        return false;
    }
    out.shape = shape;
    return true;
}

void print_help(const char* prog)
{
    std::cout
        << "Usage: " << prog << " [options]\n\n"
        << "Steady-state benchmark for the UHD SC16 burst pipeline.\n\n"
        << "  --file <path>              SC16 replay input (required)\n"
        << "  --native-rate <hz>         native sample rate (default 737280000)\n"
        << "  --work-rate <hz>           work sample rate (default 998400000)\n"
        << "  --capture <n>              capture samples per window (default 56000)\n"
        << "  --pre-guard <n>            pre-guard samples (default 0)\n"
        << "  --post-guard <n>           post-guard samples (default 0)\n"
        << "  --period <n>               schedule period in native samples (default = capacity)\n"
        << "  --t0 <samples>             schedule t0 (default 0)\n"
        << "  --consumer <sic|sensing|both|sic_sensing|comm|sic_comm>  (default sic)\n"
        << "  --resampler-profile <name> (default realtime_minorder)\n"
        << "  --resampler-experiment <none|fixed_phase|convert_cache|both>\n"
        << "  --iq-dump-dir <dir>        diagnostic post-resample IQ dump directory\n"
        << "  --iq-dump-tag <tag>        filename suffix (default: none)\n"
        << "  --iq-dump-windows <n>      number of windows to dump (default 0)\n"
        << "  --iq-dump-window-limit <n> only window_id < n (default 0 = no limit)\n"
        << "  --dsp-workers <n>          (default 1)\n"
        << "  --cpu-rx <core>            (default -1)\n"
        << "  --cpu-dsp <core[,core]>    (default -1)\n"
        << "  --pool-size <n>            per-worker slot count (default 16)\n"
        << "  --warmup-windows <n>       completed windows excluded from stats (default 0)\n"
        << "  --measure-windows <n>      completed windows in the measurement region (default 10000)\n"
        << "  --flow-in-flight <n>       feeder window-level in-flight limit (0 = off, default 24)\n"
        << "  --flow-timeout-ms <ms>     feeder fail-open timeout (default 2000)\n"
        << "  --diag-rx                  enable RX diagnostics\n"
        << "  --diag-commit              enable commit diagnostics\n"
        << "  --label <name>             run label written into the JSON meta\n"
        << "  --json <path>              write results JSON\n"
        << "  --csv <path>               write per-window CSV\n"
        << "  --benchmark                acquire /tmp/uwb-sic-benchmark.lock\n"
        << "  --sic-code <n>             interferer code index 9..12 (default 10)\n"
        << "  --sic-comm-code <n>        alias for --sic-code\n"
        << "  --sic-sync-reps <n>        interferer SYNC reps (default 64)\n"
        << "  --sic-sfd-mode <m>         decawave|ieee|4z1|4z2|4z3|4z4 (default 4z2)\n"
        << "  --sic-pulse <p>            legacy|gaussian|blackman (default legacy)\n"
        << "  --sic-policy <p>           preamble|fullpacket|auto (default auto; sic_comm defaults preamble)\n"
        << "  --sic-backend <b>          ipatov|discrete|fractional (default ipatov)\n"
        << "  --sic-stride <n>           SIC acquisition coarse stride (default 1)\n"
        << "  --sic-full-window <0|1>    SIC full-window search (default 0; sic_comm defaults 1)\n"
        << "  --sic-sync-template <path> one SYNC symbol CF32 @ work rate\n"
        << "  --sic-roi-margin <n>       (default 64)\n"
        << "  --sic-tap-count <n>        (default 20)\n"
        << "  --radar-code <n>           target sensing code 9..12 (default 9)\n"
        << "  --radar-sync-reps <n>      target sensing SYNC reps (default 64)\n"
        << "  --target-sync-reps <n>     alias for --radar-sync-reps\n"
        << "  --radar-sfd-mode <m>       decawave|ieee|4z1|4z2|4z3|4z4 (default 4z2)\n"
        << "  --radar-pulse <p>          legacy|gaussian|blackman (default legacy)\n"
        << "  --radar-full-window <0|1>  target sensing full-window search (default 1)\n"
        << "  --radar-stride <n>         target sensing coarse stride (default 1; full-window>=8)\n"
        << "  --radar-sync-template <p>  optional 1-symbol SYNC override\n"
        << "  --comm-code <n>            target comm code 9..12 (default 9)\n"
        << "  --comm-sync-reps <n>       target comm SYNC reps (default 64)\n"
        << "  --comm-sfd-mode <m>        decawave|ieee|4z1|4z2|4z3|4z4 (default 4z2)\n"
        << "  --comm-pulse <p>           legacy|gaussian|blackman (default legacy)\n"
        << "  --comm-full-window <0|1>   target comm full-window search (default 1)\n"
        << "  --comm-stride <n>          target comm coarse stride (default 8)\n"
        << "  --comm-sync-template <p>   optional 1-symbol SYNC override\n"
        << "  --help, -h\n";
}

} // namespace

int main(int argc, char* argv[])
{
    PipelineConfig cfg;
    std::string file;
    std::string consumer = "sic";
    std::string label = "unlabeled";
    std::string json_path, csv_path;
    double period_samples = 0.0;
    long long warmup = 0;
    long long measure = 10000;
    long long flow_in_flight = 24;
    double flow_timeout_ms = 2000.0;
    long long sic_roi_margin = 64;
    bool sic_roi_margin_explicit = false;
    bool sic_policy_explicit = false;
    bool sic_full_window_explicit = false;
    bool benchmark_mode = false;

    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        auto need = [&](int& i) -> const char* {
            if (i + 1 >= argc) {
                std::cerr << "Error: " << argv[i] << " requires a value\n";
                std::exit(1);
            }
            return argv[++i];
        };
        if (a == "--help" || a == "-h") {
            print_help(argv[0]);
            return 0;
        } else if (a == "--file") {
            file = need(i);
        } else if (a == "--native-rate") {
            cfg.native_rate = std::stod(need(i));
        } else if (a == "--work-rate") {
            cfg.work_rate = std::stod(need(i));
        } else if (a == "--capture") {
            cfg.capture_samples = static_cast<size_t>(std::stoull(need(i)));
        } else if (a == "--pre-guard") {
            cfg.pre_guard_samples = static_cast<size_t>(std::stoull(need(i)));
        } else if (a == "--post-guard") {
            cfg.post_guard_samples = static_cast<size_t>(std::stoull(need(i)));
        } else if (a == "--period") {
            period_samples = std::stod(need(i));
        } else if (a == "--t0") {
            cfg.schedule_t0 = std::stod(need(i));
        } else if (a == "--consumer") {
            consumer = need(i);
        } else if (a == "--target-sync-reps" || a == "--radar-sync-reps") {
            const size_t reps = std::stoull(need(i));
            if (reps < 8 || reps > 4096) {
                std::cerr << "Error: radar SYNC reps must be in [8, 4096]\n";
                return 1;
            }
            cfg.radar_cfg.sync_repetitions = reps;
            cfg.radar_cfg.cir_repetitions =
                (reps > cfg.radar_cfg.cir_skip_initial)
                    ? (reps - cfg.radar_cfg.cir_skip_initial)
                    : reps;
        } else if (a == "--radar-code") {
            const size_t code = std::stoull(need(i));
            if (code < 9 || code > 12) {
                std::cerr << "Error: --radar-code must be 9..12\n";
                return 1;
            }
            cfg.radar_cfg.code_index = code;
        } else if (a == "--radar-sfd-mode") {
            if (!parse_sfd_mode_literal(need(i), cfg.radar_cfg.sfd_mode)) {
                std::cerr << "Error: bad --radar-sfd-mode\n";
                return 1;
            }
        } else if (a == "--radar-pulse") {
            if (!parse_cli_pulse(need(i), cfg.radar_pulse)) {
                std::cerr << "Error: bad --radar-pulse\n";
                return 1;
            }
        } else if (a == "--radar-full-window") {
            if (!parse_bool_flag(need(i), cfg.radar_full_window_search)) {
                std::cerr << "Error: bad --radar-full-window\n";
                return 1;
            }
        } else if (a == "--radar-stride") {
            const size_t stride = std::stoull(need(i));
            if (stride == 0) {
                std::cerr << "Error: --radar-stride must be >= 1\n";
                return 1;
            }
            cfg.radar_cfg.timing_coarse_stride = stride;
        } else if (a == "--radar-sync-template") {
            cfg.radar_sync_template_file = need(i);
        } else if (a == "--comm-code") {
            const size_t code = std::stoull(need(i));
            if (code < 9 || code > 12) {
                std::cerr << "Error: --comm-code must be 9..12\n";
                return 1;
            }
            cfg.comm_cfg.code_index = code;
        } else if (a == "--comm-sync-reps") {
            const size_t reps = std::stoull(need(i));
            if (reps < 8 || reps > 4096) {
                std::cerr << "Error: comm SYNC reps must be in [8, 4096]\n";
                return 1;
            }
            cfg.comm_cfg.preamble_repetitions = reps;
        } else if (a == "--comm-sfd-mode") {
            if (!parse_sfd_mode_literal(need(i), cfg.comm_cfg.sfd_mode)) {
                std::cerr << "Error: bad --comm-sfd-mode\n";
                return 1;
            }
        } else if (a == "--comm-pulse") {
            if (!parse_cli_pulse(need(i), cfg.comm_pulse)) {
                std::cerr << "Error: bad --comm-pulse\n";
                return 1;
            }
        } else if (a == "--comm-full-window") {
            if (!parse_bool_flag(need(i), cfg.comm_full_window_search)) {
                std::cerr << "Error: bad --comm-full-window\n";
                return 1;
            }
        } else if (a == "--comm-stride") {
            const size_t stride = std::stoull(need(i));
            if (stride == 0) {
                std::cerr << "Error: --comm-stride must be >= 1\n";
                return 1;
            }
            cfg.comm_timing_coarse_stride = stride;
            cfg.comm_cfg.timing_coarse_stride = stride;
        } else if (a == "--comm-sync-template") {
            cfg.comm_sync_template_file = need(i);
        } else if (a == "--sic-sync-reps") {
            const size_t reps = std::stoull(need(i));
            if (reps < 8 || reps > 4096) {
                std::cerr << "Error: SIC SYNC reps must be in [8, 4096]\n";
                return 1;
            }
            cfg.sic_cfg.sync_reps = reps;
        } else if (a == "--resampler-profile") {
            cfg.resampler_profile = need(i);
        } else if (a == "--resampler-experiment") {
            cfg.resampler_experiment = need(i);
        } else if (a == "--iq-dump-dir") {
            cfg.iq_dump_dir = need(i);
        } else if (a == "--iq-dump-tag") {
            cfg.iq_dump_tag = need(i);
        } else if (a == "--iq-dump-windows") {
            cfg.iq_dump_windows = static_cast<size_t>(std::stoull(need(i)));
        } else if (a == "--iq-dump-window-limit") {
            cfg.iq_dump_window_limit = static_cast<size_t>(std::stoull(need(i)));
        } else if (a == "--dsp-workers") {
            cfg.dsp_workers = std::stoi(need(i));
        } else if (a == "--cpu-rx") {
            cfg.cpu_core_rx = std::stoi(need(i));
        } else if (a == "--cpu-dsp") {
            const std::string spec = need(i);
            cfg.cpu_cores_dsp.clear();
            size_t begin = 0;
            while (true) {
                const size_t comma = spec.find(',', begin);
                const std::string tok =
                    (comma == std::string::npos) ? spec.substr(begin)
                                                 : spec.substr(begin, comma - begin);
                if (tok.empty()) {
                    std::cerr << "Error: empty CPU core token in '" << spec << "'\n";
                    return 1;
                }
                cfg.cpu_cores_dsp.push_back(std::stoi(tok));
                if (comma == std::string::npos) {
                    break;
                }
                begin = comma + 1;
            }
            if (cfg.cpu_cores_dsp.size() == 1) {
                cfg.cpu_core_dsp = cfg.cpu_cores_dsp.front();
            }
        } else if (a == "--pool-size") {
            cfg.pool_capacity = static_cast<size_t>(std::stoull(need(i)));
        } else if (a == "--warmup-windows") {
            warmup = std::stoll(need(i));
        } else if (a == "--measure-windows") {
            measure = std::stoll(need(i));
        } else if (a == "--flow-in-flight") {
            flow_in_flight = std::stoll(need(i));
        } else if (a == "--flow-timeout-ms") {
            flow_timeout_ms = std::stod(need(i));
        } else if (a == "--diag-rx") {
            cfg.enable_rx_diagnostics = true;
        } else if (a == "--diag-commit") {
            cfg.enable_commit_diagnostics = true;
        } else if (a == "--label") {
            label = need(i);
        } else if (a == "--json") {
            json_path = need(i);
        } else if (a == "--csv") {
            csv_path = need(i);
        } else if (a == "--benchmark") {
            benchmark_mode = true;
        } else if (a == "--sic-policy") {
            const std::string v = need(i);
            sic_policy_explicit = true;
            if (v == "preamble") {
                cfg.sic_cfg.policy = gr::uwb::sic::SicPolicy::PreambleOnly;
            } else if (v == "fullpacket") {
                cfg.sic_cfg.policy = gr::uwb::sic::SicPolicy::FullPacket;
            } else if (v == "auto") {
                cfg.sic_cfg.policy = gr::uwb::sic::SicPolicy::AutoFallback;
            } else {
                std::cerr << "Error: bad --sic-policy\n";
                return 1;
            }
        } else if (a == "--sic-backend") {
            const std::string v = need(i);
            if (v == "ipatov") {
                cfg.sic_cfg.backend = gr::uwb::sic::SicEstimatorBackend::Ipatov;
            } else if (v == "discrete") {
                cfg.sic_cfg.backend = gr::uwb::sic::SicEstimatorBackend::DiscreteCir;
            } else if (v == "fractional") {
                cfg.sic_cfg.backend = gr::uwb::sic::SicEstimatorBackend::PathFractional;
            } else {
                std::cerr << "Error: bad --sic-backend\n";
                return 1;
            }
        } else if (a == "--sic-comm-code" || a == "--sic-code") {
            const size_t code = std::stoull(need(i));
            if (code < 9 || code > 12) {
                std::cerr << "Error: --sic-code must be 9..12\n";
                return 1;
            }
            cfg.sic_cfg.comm_code = code;
        } else if (a == "--sic-sfd-mode") {
            if (!parse_sfd_mode_literal(need(i), cfg.sic_cfg.sfd_mode)) {
                std::cerr << "Error: bad --sic-sfd-mode\n";
                return 1;
            }
        } else if (a == "--sic-pulse") {
            if (!parse_cli_pulse(need(i), cfg.sic_cfg.pulse)) {
                std::cerr << "Error: bad --sic-pulse\n";
                return 1;
            }
        } else if (a == "--sic-stride") {
            const size_t stride = std::stoull(need(i));
            if (stride == 0) {
                std::cerr << "Error: --sic-stride must be >= 1\n";
                return 1;
            }
            cfg.sic_cfg.coarse_stride = stride;
        } else if (a == "--sic-full-window") {
            if (!parse_bool_flag(need(i), cfg.sic_cfg.full_window_search)) {
                std::cerr << "Error: bad --sic-full-window\n";
                return 1;
            }
            sic_full_window_explicit = true;
        } else if (a == "--sic-sync-template") {
            cfg.sic_sync_template_file = need(i);
        } else if (a == "--sic-roi-margin") {
            sic_roi_margin = std::stoll(need(i));
            sic_roi_margin_explicit = true;
        } else if (a == "--sic-tap-count") {
            cfg.sic_cfg.tap_count = static_cast<size_t>(std::stoull(need(i)));
        } else {
            std::cerr << "Error: unrecognized option '" << a << "'\n";
            print_help(argv[0]);
            return 1;
        }
    }

    if (file.empty()) {
        std::cerr << "Error: --file is required\n";
        return 1;
    }
    if (!file_exists(file)) {
        std::cerr << "Error: input file not found: " << file << "\n";
        return 1;
    }
    if (cfg.dsp_workers < 1) {
        std::cerr << "Error: --dsp-workers must be >= 1\n";
        return 1;
    }
    if (!cfg.cpu_cores_dsp.empty() &&
        static_cast<int>(cfg.cpu_cores_dsp.size()) != cfg.dsp_workers) {
        std::cerr << "Error: --cpu-dsp must list exactly one core per DSP worker\n";
        return 1;
    }
    if (cfg.native_rate <= 0.0) {
        std::cerr << "Error: native rate must be positive\n";
        return 1;
    }
    if (period_samples <= 0.0) {
        period_samples = static_cast<double>(cfg.native_window_capacity());
    }
    // A window-level in-flight limit larger than the total pool depth would
    // turn the feeder back into a drop storm, so clamp it to what the pools can
    // actually hold.  Reported in the meta so the effective value is never
    // implicit.
    if (flow_in_flight > 0) {
        const long long pool_bound =
            static_cast<long long>(cfg.pool_capacity > 2 ? cfg.pool_capacity - 2 : 1) *
            static_cast<long long>(cfg.dsp_workers);
        if (flow_in_flight > pool_bound) {
            std::cout << "[feeder] flow_in_flight " << flow_in_flight
                      << " clamped to " << pool_bound
                      << " (pool_size-2 per worker)" << std::endl;
            flow_in_flight = pool_bound;
        }
    }
    cfg.period_s = period_samples / cfg.native_rate;
    cfg.source_type = StreamSourceType::Replay;
    cfg.replay_file = file;
    cfg.paced_replay = false;
    cfg.max_windows = static_cast<uint64_t>(warmup + measure);

    if (consumer == "sic") {
        cfg.consumer_mode = ConsumerMode::SicOnly;
    } else if (consumer == "sensing") {
        cfg.consumer_mode = ConsumerMode::SensingCir;
    } else if (consumer == "both" || consumer == "sic_sensing") {
        cfg.consumer_mode = ConsumerMode::SicPlusSensing;
    } else if (consumer == "comm") {
        cfg.consumer_mode = ConsumerMode::CommOnly;
    } else if (consumer == "sic_comm") {
        cfg.consumer_mode = ConsumerMode::SicPlusComm;
        if (!sic_policy_explicit) {
            cfg.sic_cfg.policy = gr::uwb::sic::SicPolicy::PreambleOnly;
        }
        if (!sic_full_window_explicit && !sic_roi_margin_explicit) {
            cfg.sic_cfg.full_window_search = true;
        }
    } else {
        std::cerr << "Error: bad --consumer\n";
        return 1;
    }

    cfg.radar_cfg.full_window_search = cfg.radar_full_window_search;
    if (!cfg.sic_cfg.full_window_search) {
        bench::derive_sic_geometry(cfg, sic_roi_margin);
    }

    BenchLockGuard bench_guard;
    if (benchmark_mode && !bench_guard.acquire()) {
        std::cerr << "Error: failed to acquire benchmark lock\n";
        return 1;
    }

    std::cout << "=== steady-state SC16 pipeline benchmark ===\n"
              << "  label:        " << label << "\n"
              << "  file:         " << file << "\n"
              << "  native/work:  " << cfg.native_rate / 1e6 << " / "
              << cfg.work_rate / 1e6 << " MS/s\n"
              << "  window:       pre=" << cfg.pre_guard_samples
              << " cap=" << cfg.capture_samples
              << " post=" << cfg.post_guard_samples
              << " (total=" << cfg.native_window_capacity() << ")\n"
              << "  period:       " << period_samples << " samples ("
              << cfg.period_s * 1e3 << " ms)\n"
              << "  worker:       dsp=" << cfg.dsp_workers
              << " pool=" << cfg.pool_capacity << "\n"
              << "  windows:      warmup=" << warmup << " measure=" << measure
              << " total=" << cfg.max_windows << "\n"
              << "  feeder flow:  in_flight=" << flow_in_flight
              << " timeout=" << flow_timeout_ms << " ms\n"
              << "  diag:         rx=" << (cfg.enable_rx_diagnostics ? 1 : 0)
              << " commit=" << (cfg.enable_commit_diagnostics ? 1 : 0) << "\n"
              << "  experiment:   " << cfg.resampler_experiment << "\n"
              << std::endl;

    // Feeder gate: the result callback advances the completed counter and
    // wakes the RX thread, which is blocked only when the pipeline is behind.
    std::atomic<uint64_t> completed_counter{ 0 };
    std::mutex flow_gate_mutex;
    std::condition_variable flow_gate_cv;

    auto source = std::make_unique<LoopingReplaySc16Source>(file, cfg.native_rate, true);
    LoopingReplaySc16Source* source_raw = source.get();

    UhdSc16Pipeline pipeline(cfg);
    pipeline.set_result_callback([&](const WindowResult&) {
        completed_counter.fetch_add(1, std::memory_order_relaxed);
        flow_gate_cv.notify_one();
    });

    if (!pipeline.initialize(std::move(source))) {
        std::cerr << "Error: pipeline initialize() failed\n";
        return 1;
    }

    source_raw->set_flow_control(static_cast<int64_t>(std::llround(period_samples)),
                                 flow_in_flight, &completed_counter,
                                 &flow_gate_mutex, &flow_gate_cv, flow_timeout_ms);

    const auto t_before_start = std::chrono::steady_clock::now();
    pipeline.start();
    const bool ready = pipeline.wait_ready(30000);
    const auto t_ready = std::chrono::steady_clock::now();
    pipeline.wait();
    const auto t_after_wait = std::chrono::steady_clock::now();

    const double cold_total_s =
        std::chrono::duration<double>(t_after_wait - t_before_start).count();
    const double startup_ready_ms =
        std::chrono::duration<double, std::milli>(t_ready - t_before_start).count();

    const auto& ledger = pipeline.ledger();
    std::string inv_err;
    const bool inv_ok = ledger.verify_invariants(&inv_err);

    if (!ready || pipeline.failed()) {
        std::cerr << "Error: pipeline did not become ready: "
                  << pipeline.init_error() << "\n";
    }

    const auto& all_results = pipeline.results();

    std::vector<const WindowResult*> completed;
    completed.reserve(all_results.size());
    for (const auto& r : all_results) {
        if (r.state == WindowTerminalState::Completed) {
            completed.push_back(&r);
        }
    }

    // Measurement region: [warmup, warmup + measure) over COMPLETED windows in
    // commit order.  No window is ever skipped inside the region.
    std::vector<const WindowResult*> measured;
    size_t measured_begin = 0;
    if (warmup > 0) {
        measured_begin = std::min<size_t>(static_cast<size_t>(warmup), completed.size());
    }
    size_t measured_end = completed.size();
    if (measure > 0) {
        measured_end = std::min<size_t>(measured_begin + static_cast<size_t>(measure),
                                        completed.size());
    }
    for (size_t i = measured_begin; i < measured_end; ++i) {
        measured.push_back(completed[i]);
    }

    std::vector<double> v_queue, v_convert, v_resample, v_sic, v_sensing, v_comm, v_e2e;
    for (const auto* r : measured) {
        v_queue.push_back(r->timing.queue_us);
        v_convert.push_back(r->timing.convert_us);
        v_resample.push_back(r->timing.resample_us);
        v_sic.push_back(r->timing.sic_us);
        v_sensing.push_back(r->timing.sensing_us);
        v_comm.push_back(r->timing.comm_us);
        v_e2e.push_back(r->timing.e2e_us);
    }
    const PhaseStats st_queue = compute_phase_stats(v_queue);
    const PhaseStats st_convert = compute_phase_stats(v_convert);
    const PhaseStats st_resample = compute_phase_stats(v_resample);
    const PhaseStats st_sic = compute_phase_stats(v_sic);
    const PhaseStats st_sensing = compute_phase_stats(v_sensing);
    const PhaseStats st_comm = compute_phase_stats(v_comm);
    const PhaseStats st_e2e = compute_phase_stats(v_e2e);

    // Steady interval: bounded by real monotonic timestamps of the measured
    // windows: first RX push .. last commit.  Explicitly reported.
    // Steady interval boundary: commit timestamps of the FIRST and LAST
    // measured windows; n commit stamps delimit n-1 inter-completion intervals.
    // Commit cadence is immune to the RX look-ahead that happened before the
    // region started processing.
    bool have_interval = false;
    std::chrono::steady_clock::time_point t_first_push{};
    std::chrono::steady_clock::time_point t_first_commit{};
    std::chrono::steady_clock::time_point t_last_commit{};
    for (const auto* r : measured) {
        if (r->commit_time == std::chrono::steady_clock::time_point{}) {
            continue;
        }
        if (!have_interval) {
            t_first_commit = r->commit_time;
            t_last_commit = r->commit_time;
            t_first_push = r->rx_push_time;
            have_interval = true;
        } else {
            t_first_commit = std::min(t_first_commit, r->commit_time);
            t_last_commit = std::max(t_last_commit, r->commit_time);
            t_first_push = std::min(t_first_push, r->rx_push_time);
        }
    }
    const size_t steady_span_windows = (measured.size() > 1) ? (measured.size() - 1) : 0;
    const double steady_s = have_interval
                                ? std::chrono::duration<double>(t_last_commit - t_first_commit).count()
                                : 0.0;
    const double steady_lookahead_ms =
        have_interval
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
        have_interval
            ? std::chrono::duration<double, std::milli>(t_after_wait - t_last_commit).count()
            : 0.0;
    const double rx_read_msps =
        (cold_total_s > 0.0)
            ? static_cast<double>(ledger.total_samples_received) / cold_total_s / 1e6
            : 0.0;

    size_t sic_executed = 0, sic_committed = 0, radar_executed = 0, radar_found = 0;
    size_t comm_attempted = 0, comm_ok = 0, comm_phr_ok = 0, comm_fcs_ok = 0;
    double suppression_sum = 0.0;
    for (const auto* r : measured) {
        if (r->sic_executed) {
            ++sic_executed;
            if (r->sic_committed) {
                ++sic_committed;
                suppression_sum += static_cast<double>(r->actual_suppression_db);
            }
        }
        if (r->radar_executed) {
            ++radar_executed;
            if (r->radar_sfd_found) {
                ++radar_found;
            }
        }
        if (r->comm_attempted) {
            ++comm_attempted;
            if (r->comm_ok) {
                ++comm_ok;
            }
            if (r->comm_phr_ok) {
                ++comm_phr_ok;
            }
            if (r->comm_fcs_ok) {
                ++comm_fcs_ok;
            }
        }
    }

    const RxDiagnostics& rx = pipeline.rx_diagnostics();
    const CommitDiagnostics& cm = pipeline.commit_diagnostics();

    auto print_row = [](const char* name, const PhaseStats& s) {
        std::cout << "  " << std::left << std::setw(14) << name << std::right
                  << std::fixed << std::setprecision(2)
                  << std::setw(11) << s.mean << std::setw(11) << s.p50
                  << std::setw(11) << s.p95 << std::setw(11) << s.p99
                  << std::setw(11) << s.min << std::setw(11) << s.max
                  << std::setw(8) << s.n << "\n";
    };

    std::cout << "\n--- lifecycle ---\n"
              << "  cold_total:      " << std::fixed << std::setprecision(3)
              << cold_total_s * 1e3 << " ms\n"
              << "  startup_ready:   " << std::fixed << std::setprecision(3)
              << startup_ready_ms << " ms (ok=" << (ready ? 1 : 0) << ")\n"
              << "  drain_tail:      " << std::fixed << std::setprecision(3)
              << drain_tail_ms << " ms\n"
              << "  fed loops:       " << source_raw->loops()
              << " (file " << source_raw->file_samples() << " samples)\n"
              << "  feeder waits:    " << source_raw->flow_waits()
              << " / " << std::fixed << std::setprecision(3)
              << source_raw->flow_wait_us() * 1e-3 << " ms\n"
              << "  feeder reads:    " << source_raw->read_calls()
              << " / " << source_raw->read_bytes() << " B / "
              << std::fixed << std::setprecision(3)
              << source_raw->read_us() * 1e-3 << " ms\n"
              << "\n--- ledger (invariants "
              << (inv_ok ? "PASS" : "FAIL: " + inv_err) << ") ---\n"
              << "  offered=" << ledger.offered_windows
              << " admitted=" << ledger.admitted_windows
              << " completed=" << ledger.completed_windows
              << " dropped=" << ledger.dropped_admission
              << " cancelled=" << ledger.cancelled_windows
              << " failed=" << ledger.processing_failed
              << " gap_before=" << ledger.invalid_gap_before_admission
              << " gap_after=" << ledger.invalid_gap_after_admission << "\n"
              << "  samples_read=" << ledger.total_samples_received
              << " (" << rx_read_msps << " MS/s over cold_total)\n"
              << "\n--- steady interval (measured windows only) ---\n"
              << "  windows:         " << measured.size()
              << " (warmup=" << warmup << ", requested=" << measure << ")\n"
              << "  interval:        " << std::fixed << std::setprecision(4)
              << steady_s << " s (first..last measured commit_time)\n"
              << "  span windows:    " << steady_span_windows << "\n"
              << "  lookahead:       " << std::fixed << std::setprecision(3)
              << steady_lookahead_ms << " ms (first push -> first measured commit)\n"
              << "  us/window:       " << std::fixed << std::setprecision(2)
              << steady_window_us << "\n"
              << "  win/s:           " << std::fixed << std::setprecision(2)
              << steady_window_rate << "\n"
              << "  window MS/s:     " << std::fixed << std::setprecision(2)
              << steady_sample_msps << "\n"
              << "\n--- warm service (us) ---\n"
              << "  phase              mean       p50        p95        p99        min        max       n\n";
    print_row("queue", st_queue);
    print_row("convert", st_convert);
    print_row("resample", st_resample);
    print_row("sic", st_sic);
    print_row("sensing", st_sensing);
    print_row("comm", st_comm);
    print_row("e2e", st_e2e);

    const double mean_suppression_db =
        sic_committed > 0 ? suppression_sum / static_cast<double>(sic_committed) : 0.0;
    std::cout << "\n--- functional ---\n"
              << "  sic_executed=" << sic_executed
              << " sic_committed=" << sic_committed
              << " mean_suppression_db=" << std::fixed << std::setprecision(2)
              << mean_suppression_db
              << "\n  radar_executed=" << radar_executed
              << " radar_sfd_found=" << radar_found
              << "\n  comm_attempted=" << comm_attempted
              << " comm_ok=" << comm_ok
              << " phr_ok=" << comm_phr_ok
              << " fcs_ok=" << comm_fcs_ok
              << " comm_us=" << std::fixed << std::setprecision(2) << st_comm.mean << "\n";

    if (cfg.enable_rx_diagnostics) {
        std::cout << "\n--- RX diagnostics (cumulative) ---\n"
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
                  << " ledger_wait_us=" << rx.ledger_wait_us << "\n";
    }
    if (cfg.enable_commit_diagnostics) {
        std::cout << "\n--- commit diagnostics ---\n"
                  << "  commit_calls=" << cm.commit_calls
                  << " results_emitted=" << cm.results_emitted
                  << " max_out_of_order_span=" << cm.max_out_of_order_span << "\n"
                  << "  lock_wait_us=" << std::fixed << std::setprecision(1) << cm.lock_wait_us
                  << " reorder_us=" << cm.reorder_us
                  << " callback_us=" << cm.callback_us << "\n";
    }

    // ---- machine-readable output ----
    bench::MetaList meta;
    meta.emplace_back("driver", bench::meta_string("benchmark_sc16_pipeline_steady"));
    meta.emplace_back("label", bench::meta_string(label));
    meta.emplace_back("file", bench::meta_string(file));
    meta.emplace_back("native_rate", bench::json_number(cfg.native_rate));
    meta.emplace_back("work_rate", bench::json_number(cfg.work_rate));
    meta.emplace_back("capture_samples", bench::json_number(static_cast<uint64_t>(cfg.capture_samples)));
    meta.emplace_back("pre_guard_samples", bench::json_number(static_cast<uint64_t>(cfg.pre_guard_samples)));
    meta.emplace_back("post_guard_samples", bench::json_number(static_cast<uint64_t>(cfg.post_guard_samples)));
    meta.emplace_back("period_samples", bench::json_number(period_samples));
    meta.emplace_back("dsp_workers", bench::json_number(static_cast<int64_t>(cfg.dsp_workers)));
    meta.emplace_back("pool_capacity", bench::json_number(static_cast<uint64_t>(cfg.pool_capacity)));
    meta.emplace_back("resampler_profile", bench::meta_string(cfg.resampler_profile));
    meta.emplace_back("resampler_experiment", bench::meta_string(cfg.resampler_experiment));
    meta.emplace_back("consumer", bench::meta_string(consumer));
    meta.emplace_back("warmup_windows", bench::json_number(warmup));
    meta.emplace_back("measure_windows_requested", bench::json_number(measure));
    meta.emplace_back("measure_windows_actual", bench::json_number(static_cast<uint64_t>(measured.size())));
    meta.emplace_back("flow_in_flight", bench::json_number(flow_in_flight));
    meta.emplace_back("diag_rx", bench::meta_bool(cfg.enable_rx_diagnostics));
    meta.emplace_back("diag_commit", bench::meta_bool(cfg.enable_commit_diagnostics));
    meta.emplace_back("ready_ok", bench::meta_bool(ready && !pipeline.failed()));
    meta.emplace_back("ledger_invariants_ok", bench::meta_bool(inv_ok));
    meta.emplace_back("cold_total_s", bench::json_number(cold_total_s));
    meta.emplace_back("startup_ready_ms", bench::json_number(startup_ready_ms));
    meta.emplace_back("drain_tail_ms", bench::json_number(drain_tail_ms));
    meta.emplace_back("steady_interval_s", bench::json_number(steady_s));
    meta.emplace_back("steady_span_windows", bench::json_number(static_cast<uint64_t>(steady_span_windows)));
    meta.emplace_back("steady_lookahead_ms", bench::json_number(steady_lookahead_ms));
    meta.emplace_back("steady_window_us", bench::json_number(steady_window_us));
    meta.emplace_back("steady_window_rate", bench::json_number(steady_window_rate));
    meta.emplace_back("steady_window_sample_msps", bench::json_number(steady_sample_msps));
    meta.emplace_back("rx_read_msps", bench::json_number(rx_read_msps));
    meta.emplace_back("feeder_loops", bench::json_number(source_raw->loops()));
    meta.emplace_back("feeder_flow_waits", bench::json_number(source_raw->flow_waits()));
    meta.emplace_back("feeder_flow_wait_us", bench::json_number(source_raw->flow_wait_us()));
    meta.emplace_back("feeder_read_us", bench::json_number(source_raw->read_us()));
    meta.emplace_back("feeder_read_calls", bench::json_number(source_raw->read_calls()));
    meta.emplace_back("feeder_read_bytes", bench::json_number(source_raw->read_bytes()));
    meta.emplace_back("resample_us_mean", bench::json_number(st_resample.mean));
    meta.emplace_back("resample_us_p50", bench::json_number(st_resample.p50));
    meta.emplace_back("resample_us_p95", bench::json_number(st_resample.p95));
    meta.emplace_back("resample_us_p99", bench::json_number(st_resample.p99));
    meta.emplace_back("sic_us_mean", bench::json_number(st_sic.mean));
    meta.emplace_back("sensing_us_mean", bench::json_number(st_sensing.mean));
    meta.emplace_back("comm_us_mean", bench::json_number(st_comm.mean));
    meta.emplace_back("comm_us_p50", bench::json_number(st_comm.p50));
    meta.emplace_back("comm_us_p95", bench::json_number(st_comm.p95));
    meta.emplace_back("comm_us_p99", bench::json_number(st_comm.p99));
    meta.emplace_back("sic_executed", bench::json_number(static_cast<uint64_t>(sic_executed)));
    meta.emplace_back("sic_committed", bench::json_number(static_cast<uint64_t>(sic_committed)));
    meta.emplace_back("mean_suppression_db", bench::json_number(mean_suppression_db));
    meta.emplace_back("radar_executed", bench::json_number(static_cast<uint64_t>(radar_executed)));
    meta.emplace_back("radar_sfd_found", bench::json_number(static_cast<uint64_t>(radar_found)));
    meta.emplace_back("comm_attempted", bench::json_number(static_cast<uint64_t>(comm_attempted)));
    meta.emplace_back("comm_ok", bench::json_number(static_cast<uint64_t>(comm_ok)));
    meta.emplace_back("comm_phr_ok", bench::json_number(static_cast<uint64_t>(comm_phr_ok)));
    meta.emplace_back("comm_fcs_ok", bench::json_number(static_cast<uint64_t>(comm_fcs_ok)));
    meta.emplace_back("sic_code", bench::json_number(static_cast<uint64_t>(cfg.sic_cfg.comm_code)));
    meta.emplace_back("sic_sync_reps", bench::json_number(static_cast<uint64_t>(cfg.sic_cfg.sync_reps)));
    meta.emplace_back("sic_sfd_mode", bench::meta_string(cfg.sic_cfg.sfd_mode ? cfg.sic_cfg.sfd_mode : ""));
    meta.emplace_back("sic_pulse", bench::meta_string(gr::uwb::mod::pulse_shape_name(cfg.sic_cfg.pulse.shape)));
    meta.emplace_back("radar_code", bench::json_number(static_cast<uint64_t>(cfg.radar_cfg.code_index)));
    meta.emplace_back("radar_sync_reps", bench::json_number(static_cast<uint64_t>(cfg.radar_cfg.sync_repetitions)));
    meta.emplace_back("radar_sfd_mode", bench::meta_string(cfg.radar_cfg.sfd_mode ? cfg.radar_cfg.sfd_mode : ""));
    meta.emplace_back("radar_pulse", bench::meta_string(gr::uwb::mod::pulse_shape_name(cfg.radar_pulse.shape)));
    meta.emplace_back("comm_code", bench::json_number(static_cast<uint64_t>(cfg.comm_cfg.code_index)));
    meta.emplace_back("comm_sync_reps", bench::json_number(static_cast<uint64_t>(cfg.comm_cfg.preamble_repetitions)));
    meta.emplace_back("comm_sfd_mode", bench::meta_string(cfg.comm_cfg.sfd_mode ? cfg.comm_cfg.sfd_mode : ""));
    meta.emplace_back("comm_pulse", bench::meta_string(gr::uwb::mod::pulse_shape_name(cfg.comm_pulse.shape)));
    meta.emplace_back("e2e_us_mean", bench::json_number(st_e2e.mean));
    meta.emplace_back("e2e_us_p50", bench::json_number(st_e2e.p50));
    meta.emplace_back("e2e_us_p95", bench::json_number(st_e2e.p95));
    meta.emplace_back("e2e_us_p99", bench::json_number(st_e2e.p99));
    meta.emplace_back("rx_read_us", bench::json_number(rx.read_us));
    meta.emplace_back("rx_assemble_us", bench::json_number(rx.assemble_us));
    meta.emplace_back("rx_slot_copy_us", bench::json_number(rx.slot_copy_us));
    meta.emplace_back("rx_dispatch_us", bench::json_number(rx.dispatch_us));
    meta.emplace_back("rx_ledger_wait_us", bench::json_number(rx.ledger_wait_us));
    meta.emplace_back("rx_stream_buf_resize_count", bench::json_number(rx.stream_buf_resize_count));
    meta.emplace_back("rx_zero_init_bytes", bench::json_number(rx.stream_buf_zero_init_bytes));
    meta.emplace_back("rx_compact_move_bytes", bench::json_number(rx.compact_move_bytes));
    meta.emplace_back("rx_max_buffered_samples", bench::json_number(rx.max_buffered_samples));
    meta.emplace_back("commit_lock_wait_us", bench::json_number(cm.lock_wait_us));
    meta.emplace_back("commit_reorder_us", bench::json_number(cm.reorder_us));
    meta.emplace_back("commit_callback_us", bench::json_number(cm.callback_us));
    meta.emplace_back("commit_max_out_of_order_span", bench::json_number(cm.max_out_of_order_span));

    if (!bench::write_results_json(json_path, meta, ledger, all_results)) {
        std::cerr << "Error: cannot write " << json_path << "\n";
        return 1;
    }
    if (!bench::write_results_csv(csv_path, all_results)) {
        std::cerr << "Error: cannot write " << csv_path << "\n";
        return 1;
    }

    if (!inv_ok) {
        return 3;
    }
    if (!ready || pipeline.failed()) {
        return 2;
    }
    return 0;
}
