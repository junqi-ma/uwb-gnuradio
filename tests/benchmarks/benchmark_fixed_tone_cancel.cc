/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Standalone benchmark for the fixed-frequency CW cancellation core.
 *
 * It measures the production core (not a prototype), on the validated
 * configuration, with the two ownership strategies the task distinguishes:
 *
 *   baseline (two passes): memcpy(raw -> storage) + cancel_in_place(storage)
 *   fused    (one pass)  : copy_cancel(raw -> storage)
 *
 * ABBA pairing, median + paired change, and an output equality check so a
 * faster-but-different kernel can never be reported as a win.  Sizes cover the
 * real raw-read block sizes (16 KiB and 64 KiB complex samples) and one whole
 * 20 ms period.
 *
 * It is NOT evidence that the complete receive chain sustains 737.28 MS/s: it
 * excludes UHD, the raw queue, the reader thread, the window assembler and the
 * downstream DSP competition.
 */

#include <gnuradio/uwb/uwb_fixed_tone_cancel_core.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <numeric>
#include <random>
#include <string>
#include <sys/file.h>
#include <unistd.h>
#include <vector>

using namespace openuwb::core;

namespace {

struct LockGuard {
    int fd = -1;
    bool acquire()
    {
        fd = ::open("/tmp/uwb-sic-benchmark.lock", O_CREAT | O_RDWR, 0666);
        if (fd < 0) {
            std::cerr << "cannot open benchmark lock: " << std::strerror(errno) << "\n";
            return false;
        }
        if (::flock(fd, LOCK_EX) != 0) {
            std::cerr << "cannot flock benchmark lock: " << std::strerror(errno) << "\n";
            ::close(fd);
            fd = -1;
            return false;
        }
        return true;
    }
    ~LockGuard()
    {
        if (fd >= 0) {
            ::flock(fd, LOCK_UN);
            ::close(fd);
        }
    }
};

double median(std::vector<double> v)
{
    if (v.empty()) {
        return 0.0;
    }
    std::sort(v.begin(), v.end());
    const size_t n = v.size();
    return (n % 2 == 1) ? v[n / 2] : 0.5 * (v[n / 2 - 1] + v[n / 2]);
}

struct Stats {
    double median_us = 0.0;
    double min_us = 0.0;
    double max_us = 0.0;
    double paired_median_pct = 0.0; // fused vs baseline, ABBA pairs
    size_t pairs = 0;
};

template <typename Fn>
double time_us(Fn&& fn)
{
    const auto t0 = std::chrono::steady_clock::now();
    fn();
    const auto t1 = std::chrono::steady_clock::now();
    return std::chrono::duration<double, std::micro>(t1 - t0).count();
}

constexpr size_t kPeriod = 288;
constexpr size_t kPeriodLength = 14745600; // 20 ms at 737.28 MS/s
constexpr size_t kTraining = 368640;       // 0.5 ms

} // namespace

int main(int argc, char** argv)
{
    int session = 1;
    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) == "--sessions" && i + 1 < argc) {
            session = std::max(1, std::atoi(argv[++i]));
        }
    }

    LockGuard lock;
    if (!lock.acquire()) {
        return 1;
    }

    FixedToneCancelConfig cfg;
    cfg.enable = true;
    cfg.sample_rate = 737.28e6;
    cfg.tone_frequency_hz = -232.96e6;
    cfg.period_s = 20.0e-3;
    cfg.training_s = 0.5e-3;
    cfg.crossfade_samples = 256;

    FixedToneCancelCore core;
    std::string err;
    if (!core.prepare(cfg, &err)) {
        std::cerr << "prepare failed: " << err << "\n";
        return 1;
    }
    std::cout << "fixed_tone_cancel_core: fs=" << cfg.sample_rate
              << " f0=" << cfg.tone_frequency_hz << " period=" << core.period_samples()
              << " cycles=" << core.period_cycles()
              << " Np=" << core.period_length_samples()
              << " Nt=" << core.training_length_samples()
              << " crossfade=" << core.crossfade_samples()
              << " kernel=" << (core.active_kernel() == 2 ? "avx2" : "scalar") << "\n";

    // Deterministic full-scale-covering input, including int16 extremes so the
    // saturating subtract is exercised.
    std::vector<int16_t> training(2 * kTraining);
    std::mt19937 gen(12345);
    for (size_t i = 0; i < training.size(); ++i) {
        const int r = static_cast<int>(gen() % 65536) - 32768;
        training[i] = static_cast<int16_t>(i % 1024 == 0 ? (i % 2048 ? 32767 : -32768) : r);
    }
    // A tone component so the model is realistic (not a zero-coefficient LUT).
    for (size_t i = 0; i < kTraining; ++i) {
        const double ph = core.omega_radians() * static_cast<double>(i);
        const double re = 4000.0 * std::cos(ph);
        const double im = 4000.0 * std::sin(ph);
        training[2 * i] = static_cast<int16_t>(
            std::clamp<int>(static_cast<int>(training[2 * i]) + int(std::lround(re)),
                            -32768, 32767));
        training[2 * i + 1] = static_cast<int16_t>(
            std::clamp<int>(static_cast<int>(training[2 * i + 1]) + int(std::lround(im)),
                            -32768, 32767));
    }

    // --- estimation + LUT cost ---------------------------------------------
    std::vector<double> est_us;
    std::vector<double> lut_us;
    for (int rep = 0; rep < 9; ++rep) {
        est_us.push_back(time_us([&] {
            std::string e;
            core.begin_training(0, &e);
            core.add_training_samples(training.data(), kTraining, &e);
        }));
        lut_us.push_back(time_us([&] {
            std::string e;
            core.finish_training(&e);
        }));
    }
    std::cout << "estimate_us median=" << median(est_us) << " min="
              << *std::min_element(est_us.begin(), est_us.end()) << " max="
              << *std::max_element(est_us.begin(), est_us.end()) << "\n";
    std::cout << "lut_build_us median=" << median(lut_us) << " min="
              << *std::min_element(lut_us.begin(), lut_us.end()) << " max="
              << *std::max_element(lut_us.begin(), lut_us.end()) << "\n";

    // Rebuild once so the LUT corresponds to the training above.
    {
        std::string e;
        core.begin_training(0, &e);
        core.add_training_samples(training.data(), kTraining, &e);
        if (core.finish_training(&e) != FixedToneTrainingStatus::Ok) {
            std::cerr << "model build failed: " << e << "\n";
            return 1;
        }
    }

    struct Cell {
        size_t n;
        const char* name;
    };
    const Cell cells[] = { { 16384, "16384 (single read block)" },
                           { 65536, "65536 (default raw block)" },
                           { kPeriodLength, "14745600 (whole 20 ms)" } };

    for (const auto& cell : cells) {
        std::vector<int16_t> src(2 * cell.n);
        for (size_t i = 0; i < src.size(); ++i) {
            const int r = static_cast<int>(gen() % 65536) - 32768;
            src[i] = static_cast<int16_t>((i % 4096 == 0) ? (i % 8192 ? 32767 : -32768) : r);
        }
        std::vector<int16_t> dst_base(2 * cell.n);
        std::vector<int16_t> dst_fused(2 * cell.n);

        auto baseline = [&] {
            std::memcpy(dst_base.data(), src.data(), src.size() * sizeof(int16_t));
            return core.cancel_in_place(dst_base.data(), cell.n, 0);
        };
        auto fused = [&] {
            return core.copy_cancel(src.data(), dst_fused.data(), cell.n, 0);
        };

        baseline();
        fused();
        if (dst_base != dst_fused) {
            std::cerr << "ERROR: fused and two-pass outputs differ at n=" << cell.n
                      << "; refusing to report a performance number\n";
            return 1;
        }

        std::vector<double> base_us;
        std::vector<double> fused_us;
        std::vector<double> paired;
        const int warm = 2;
        const int reps = 9;
        for (int s = 0; s < session; ++s) {
            for (int r = 0; r < warm; ++r) {
                baseline();
                fused();
            }
            // ABBA: alternate the order inside each pair.
            for (int r = 0; r < reps; ++r) {
                double b = 0.0;
                double f = 0.0;
                if ((r + s) % 2 == 0) {
                    b = time_us(baseline);
                    f = time_us(fused);
                } else {
                    f = time_us(fused);
                    b = time_us(baseline);
                }
                base_us.push_back(b);
                fused_us.push_back(f);
                paired.push_back(100.0 * (f - b) / b);
            }
        }
        Stats st;
        st.median_us = median(fused_us);
        st.min_us = *std::min_element(fused_us.begin(), fused_us.end());
        st.max_us = *std::max_element(fused_us.begin(), fused_us.end());
        st.paired_median_pct = median(paired);
        st.pairs = paired.size();
        std::cout << "cell=" << cell.name << "\n"
                  << "  two_pass_us median=" << median(base_us) << " min="
                  << *std::min_element(base_us.begin(), base_us.end()) << " max="
                  << *std::max_element(base_us.begin(), base_us.end()) << "\n"
                  << "  fused_us    median=" << st.median_us << " min=" << st.min_us
                  << " max=" << st.max_us << "\n"
                  << "  paired_change_pct median=" << st.paired_median_pct
                  << " pairs=" << st.pairs << "\n";
        const double gbps = 2.0 * static_cast<double>(cell.n) * 4.0 /
                            (st.median_us * 1e-6) / 1e9;
        std::cout << "  fused_logical_gbps=" << gbps << "\n";
    }

    std::cout << "note: single-core kernel only; no UHD, queue, thread or DSP "
                 "competition included\n";
    return 0;
}
