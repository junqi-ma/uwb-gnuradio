/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Offline replay driver for the fixed-frequency CW cancellation stage.
 *
 * This is the real production decorator (uwb_fixed_tone_cancel_source.h) over
 * the real ReplaySc16Source, so the 40 ms capture verification exercises the
 * same reader thread, bounded raw ring, training/estimate/apply state machine
 * and fused copy-and-cancel path as the UHD pipeline.  It writes the clean
 * SC16 stream so an independent fixed-frequency reference can be compared
 * byte for byte, and prints the model of every period.
 *
 * Not a GNU Radio block, no device, no install.
 */

#include <gnuradio/uwb/uwb_fixed_tone_cancel_source.h>
#include <gnuradio/uwb/uwb_uhd_sc16_pipeline.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

using namespace openuwb::pipeline;
namespace core = openuwb::core;

namespace {

void usage(const char* prog)
{
    std::cout
        << "Usage: " << prog << " [options]\n"
        << "  --file <path>            Raw interleaved SC16 input (required)\n"
        << "  --rate <hz>              Native sample rate (default 737280000)\n"
        << "  --freq <hz>              Receive baseband tone frequency (default -232960000)\n"
        << "  --period-ms <ms>         Model update period (default 20)\n"
        << "  --training-ms <ms>       Training window (default 0.5)\n"
        << "  --crossfade <n>          Cross-fade samples (default 256, 0 = hard switch)\n"
        << "  --block <n>              Raw ring block samples (default 65536)\n"
        << "  --queue <n>              Raw ring blocks (default 32)\n"
        << "  --limit-ms <ms>          Process only the first N ms (default: whole file)\n"
        << "  --out <path>             Write the clean SC16 output\n"
        << "  --models-out <path>      Write one line per model update (CSV)\n"
        << "  --paced                  Pace the replay at the configured rate\n"
        << std::endl;
}

} // namespace

int main(int argc, char** argv)
{
    std::string file;
    std::string out_path;
    std::string models_path;
    double rate = 737.28e6;
    double freq = -232.96e6;
    double period_ms = 20.0;
    double training_ms = 0.5;
    size_t crossfade = 256;
    size_t block = 65536;
    size_t queue = 32;
    double limit_ms = 0.0;
    bool paced = false;

    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        auto need = [&](const char* what) {
            if (i + 1 >= argc) {
                std::cerr << "Error: " << a << " requires " << what << "\n";
                std::exit(1);
            }
            return std::string(argv[++i]);
        };
        if (a == "--help" || a == "-h") {
            usage(argv[0]);
            return 0;
        } else if (a == "--file") {
            file = need("a path");
        } else if (a == "--rate") {
            rate = std::stod(need("Hz"));
        } else if (a == "--freq") {
            freq = std::stod(need("Hz"));
        } else if (a == "--period-ms") {
            period_ms = std::stod(need("ms"));
        } else if (a == "--training-ms") {
            training_ms = std::stod(need("ms"));
        } else if (a == "--crossfade") {
            crossfade = static_cast<size_t>(std::stoul(need("samples")));
        } else if (a == "--block") {
            block = static_cast<size_t>(std::stoul(need("samples")));
        } else if (a == "--queue") {
            queue = static_cast<size_t>(std::stoul(need("blocks")));
        } else if (a == "--limit-ms") {
            limit_ms = std::stod(need("ms"));
        } else if (a == "--out") {
            out_path = need("a path");
        } else if (a == "--models-out") {
            models_path = need("a path");
        } else if (a == "--paced") {
            paced = true;
        } else {
            std::cerr << "Error: unknown argument " << a << "\n";
            return 1;
        }
    }
    if (file.empty()) {
        usage(argv[0]);
        return 1;
    }

    core::FixedToneCancelConfig cfg;
    cfg.enable = true;
    cfg.sample_rate = rate;
    cfg.tone_frequency_hz = freq;
    cfg.period_s = period_ms * 1e-3;
    cfg.training_s = training_ms * 1e-3;
    cfg.crossfade_samples = crossfade;

    auto inner = std::make_unique<ReplaySc16Source>(file, rate, paced);
    FixedToneCancelSc16Source stage(std::move(inner), cfg, block, queue);
    if (!stage.open()) {
        std::cerr << "Error: cannot open '" << file << "' or the tone configuration "
                     "is unsupported\n";
        return 1;
    }
    const auto applied = stage.applied_config();
    std::cout << "applied: enabled=" << applied.enabled << " fs=" << applied.sample_rate
              << " f0=" << applied.tone_frequency_hz
              << " f0/fs=" << applied.frequency_ratio << " period=" << applied.period_samples
              << " cycles=" << applied.period_cycles
              << " Np=" << applied.period_length_samples
              << " Nt=" << applied.training_samples
              << " crossfade=" << applied.crossfade_samples
              << " block=" << applied.raw_block_samples
              << " queue=" << applied.raw_queue_blocks
              << " kernel=" << (applied.active_kernel == 2 ? "avx2" : "scalar") << "\n";

    const auto t_start_stream = std::chrono::steady_clock::now();
    if (!stage.start_stream()) {
        std::cerr << "Error: start_stream failed\n";
        return 1;
    }
    double first_sample_us = -1.0;

    const size_t max_samples = (limit_ms > 0.0)
                                   ? static_cast<size_t>(limit_ms * 1e-3 * rate)
                                   : 0;
    // Buffer the clean stream in memory while it is produced.  Doing file I/O
    // inside the paced consumption loop makes the offline driver (not the
    // pipeline) the bottleneck and would turn a slow disk into artificial ring
    // overflow.  A 40 ms capture is ~118 MB, comfortably inside the cap; a
    // larger run falls back to streaming with a warning.
    constexpr size_t kMaxBufferedSamples = size_t(1) << 27; // 128 M complex samples
    std::vector<std::complex<int16_t>> buffered;
    std::ofstream out;
    const bool stream_output = !out_path.empty() && max_samples > kMaxBufferedSamples;
    if (!out_path.empty()) {
        if (stream_output) {
            std::cerr << "Warning: --out is streamed (run longer than the "
                      << kMaxBufferedSamples << "-sample in-memory buffer); a slow "
                         "disk can cause ring overflow in this driver\n";
            out.open(out_path, std::ios::binary | std::ios::trunc);
            if (!out) {
                std::cerr << "Error: cannot write " << out_path << "\n";
                return 1;
            }
        } else {
            buffered.reserve(max_samples > 0 ? max_samples
                                             : static_cast<size_t>(rate * 0.05));
        }
    }
    std::ofstream models;
    if (!models_path.empty()) {
        models.open(models_path, std::ios::trunc);
        if (!models) {
            std::cerr << "Error: cannot write " << models_path << "\n";
            return 1;
        }
        models << "generation,epoch,amplitude,phase_rad,re,im\n";
        models << std::setprecision(17);
    }

    std::vector<std::complex<int16_t>> buf(65536);
    uint64_t total = 0;
    uint64_t last_generation = 0;
    double read_us = 0.0;
    size_t zero_reads = 0;
    const auto t_start = std::chrono::steady_clock::now();
    while (true) {
        const size_t want =
            (max_samples > 0 && total >= max_samples)
                ? 0
                : ((max_samples > 0) ? std::min<size_t>(buf.size(), max_samples - total)
                                     : buf.size());
        if (want == 0) {
            break;
        }
        StreamReadMeta meta;
        const auto t0 = std::chrono::steady_clock::now();
        const size_t n = stage.read_samples(buf.data(), want, meta);
        read_us += std::chrono::duration<double, std::micro>(
                       std::chrono::steady_clock::now() - t0)
                       .count();
        if (n > 0 && first_sample_us < 0.0) {
            first_sample_us = std::chrono::duration<double, std::micro>(
                                  std::chrono::steady_clock::now() - t_start_stream)
                                  .count();
        }
        if (n == 0) {
            if (meta.is_eof || stage.is_eof()) {
                break;
            }
            std::string err;
            if (stage.fatal_error(&err)) {
                std::cerr << "Error: " << err << "\n";
                return 1;
            }
            ++zero_reads;
            continue;
        }
        if (out.is_open()) {
            out.write(reinterpret_cast<const char*>(buf.data()),
                      static_cast<std::streamsize>(n * sizeof(std::complex<int16_t>)));
        } else if (!out_path.empty()) {
            buffered.insert(buffered.end(), buf.begin(), buf.begin() + n);
        }
        if (models.is_open()) {
            const auto& model = stage.tone_core().model();
            if (model.generation != last_generation) {
                last_generation = model.generation;
                models << model.generation << ',' << model.epoch << ','
                       << model.amplitude << ',' << model.phase_radians << ','
                       << model.coefficient.real() << ',' << model.coefficient.imag()
                       << "\n";
            }
        }
        total += n;
    }
    const double wall_s = std::chrono::duration<double>(
                              std::chrono::steady_clock::now() - t_start)
                              .count();
    stage.close();
    if (!out_path.empty() && !out.is_open()) {
        std::ofstream f(out_path, std::ios::binary | std::ios::trunc);
        if (!f) {
            std::cerr << "Error: cannot write " << out_path << "\n";
            return 1;
        }
        f.write(reinterpret_cast<const char*>(buffered.data()),
                static_cast<std::streamsize>(buffered.size() *
                                             sizeof(std::complex<int16_t>)));
        if (!f) {
            std::cerr << "Error: failed writing " << out_path << "\n";
            return 1;
        }
    }

    const auto& s = stage.stats();
    std::cout << std::setprecision(9);
    std::cout << "first_sample_us=" << first_sample_us
              << " samples_out=" << total << " wall_s=" << wall_s
              << " read_us_total=" << read_us << " zero_reads=" << zero_reads << "\n";
    std::cout << "source_read_calls=" << s.source_read_calls
              << " source_samples=" << s.source_samples
              << " queued_samples=" << s.queued_samples
              << " dropped_samples=" << s.dropped_samples
              << " consumed_samples=" << s.consumed_samples
              << " pending_samples=" << s.pending_samples << "\n";
    std::cout << "training_windows=" << s.training_windows
              << " model_updates=" << s.model_updates
              << " model_rejects=" << s.model_rejects
              << " partial_training_bypasses=" << s.partial_training_bypasses
              << " epoch_resets=" << s.epoch_resets
              << " cancelled_samples=" << s.cancelled_samples
              << " bypassed_samples=" << s.bypassed_samples
              << " gaps_forwarded=" << s.gaps_forwarded
              << " gap_samples_forwarded=" << s.gap_samples_forwarded << "\n";
    std::cout << "estimate_cpu_us_total=" << s.estimate_us
              << " cancel_cpu_us_total=" << s.cancel_us << "\n";
    const bool conserved =
        s.source_samples == s.queued_samples + s.dropped_samples &&
        s.queued_samples == s.consumed_samples + s.pending_samples;
    std::cout << "conservation=" << (conserved ? "PASS" : "FAIL") << "\n";
    return conserved ? 0 : 1;
}
