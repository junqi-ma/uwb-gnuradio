/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * UHD SC16 direct capture + native-grid packet analysis.
 *
 * This tool is the C++ counterpart of the GNU Radio capture flow: it uses the
 * pipeline's own UhdDeviceSc16Source (UHD -> SC16, no GNU Radio scheduler, no
 * PMT/PDU) to acquire a contiguous block of samples, and then runs the native
 * detector cores (DetectorEnergyGate + UwbPreambleVerifierSc16, the same code
 * UwbDetectorSc16 wraps) directly over the captured buffer.
 *
 * It is a diagnostic entry point, not a production block:
 *   * the whole capture is held in RAM so the disk write never steals RX
 *     bandwidth (a 737.28 MS/s SC16 stream is ~2.95 GB/s),
 *   * detection runs synchronously on the captured block, so the reported
 *     coordinates are deterministic and independent of scheduler timing,
 *   * nothing here replaces the existing GNU Radio blocks.
 *
 * Nothing in this file runs at a fixed geometry: pre-trigger / capture only
 * shape the energy Region, they never change the thresholds.
 */

#include <gnuradio/uwb/uwb_defaults.h>
#include <gnuradio/uwb/uwb_detector_core.h>
#include <gnuradio/uwb/uwb_preamble_verifier_sc16.h>
#include <gnuradio/uwb/uwb_uhd_sc16_pipeline.h>

#include <volk/volk.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

namespace defaults = openuwb::defaults;
using openuwb::UwbDetectorStateMachineSc16;
using openuwb::core::UwbPreambleVerifierSc16;
using openuwb::pipeline::PipelineConfig;
using openuwb::pipeline::StreamReadMeta;
using openuwb::pipeline::StreamSourceType;
using openuwb::pipeline::UhdDeviceSc16Source;

namespace {

struct Options {
    std::string out_path;
    double duration_ms = 20.0;
    double settle_ms = 2.0;
    double native_rate = 737.28e6;
    std::string uhd_args;
    std::string uhd_subdev;
    std::string uhd_channel;
    std::string uhd_antenna;
    double uhd_freq = 6489.6e6;
    double uhd_gain = 60.0;
    std::string template_file = "testdata/reference_preamble_code10_737p28.cf32";
    float energy_threshold = defaults::kDetectorSc16EnergyThreshold;
    float fine_threshold = defaults::kDetectorSc16FineThreshold;
    size_t pre_trigger = 7373;
    size_t capture = 143106;
    size_t coarse_decimation = defaults::kDetectorCoarseDecimation;
    size_t coarse_repetitions = defaults::kDetectorCoarseRepetitions;
    size_t coarse_margin = defaults::kDetectorCoarseMargin;
    bool detect = true;
};

void print_help(const char* prog)
{
    std::cout
        << "Usage: " << prog << " [options]\n\n"
        << "UHD SC16 direct capture + native-grid packet analysis (C++ only).\n\n"
        << "Capture:\n"
        << "  --out <path>                Raw SC16 little-endian output (optional)\n"
        << "  --ms <double>               Capture duration in ms (default 20)\n"
        << "  --settle-ms <double>        Samples discarded right after start (default 2)\n"
        << "  --native-rate <hz>          Device sample rate (default 737.28e6)\n"
        << "\nUHD:\n"
        << "  --uhd-args <args>           UHD device args (e.g. addr=192.168.10.2)\n"
        << "  --uhd-subdev <spec>         RX subdev spec (e.g. B:1)\n"
        << "  --uhd-channel <n>           RX channel index\n"
        << "  --uhd-antenna <name>        RX antenna (e.g. RX1; empty keeps device default)\n"
        << "  --freq <hz>                 Center frequency (default 6489.6e6)\n"
        << "  --gain <db>                 RX gain (default 60)\n"
        << "\nAnalysis:\n"
        << "  --template <path>           CF32 SYNC template @ native rate\n"
        << "                              (default testdata/reference_preamble_code10_737p28.cf32)\n"
        << "  --energy-threshold <f>      Energy gate threshold, 16x mean |x|^2\n"
        << "                              (default 4e-3)\n"
        << "  --fine-threshold <f>        Full-rate normalized metric (default 0.1)\n"
        << "  --pre-trigger <n>           Region pre-trigger samples (default 7373)\n"
        << "  --capture <n>               Region post-trigger capture (default 143106)\n"
        << "  --coarse-decimation <n>     Coarse scan decimation (default 4)\n"
        << "  --coarse-repetitions <n>    Coarse SYNC repetitions (default 1)\n"
        << "  --coarse-margin <n>         Coarse-to-fine half width (default 16)\n"
        << "  --no-detect                 Capture only, skip detection\n"
        << "  --help, -h                  Show this help\n";
}

bool read_complex_file(const std::string& path, std::vector<std::complex<float>>& out)
{
    std::ifstream f(path, std::ios::binary);
    if (!f) {
        std::cerr << "Error: cannot open template '" << path << "'\n";
        return false;
    }
    f.seekg(0, std::ios::end);
    const std::streamsize bytes = f.tellg();
    f.seekg(0, std::ios::beg);
    if (bytes <= 0 || (bytes % static_cast<std::streamsize>(sizeof(std::complex<float>))) != 0) {
        std::cerr << "Error: template '" << path << "' is not a CF32 file\n";
        return false;
    }
    out.resize(static_cast<size_t>(bytes) / sizeof(std::complex<float>));
    f.read(reinterpret_cast<char*>(out.data()), bytes);
    return static_cast<bool>(f) || f.eof();
}

// Same gate the detector uses: sum |x|^2 over the first 16 samples of every
// 100-sample block, average over 32 blocks, then scale by 16.  Returned as a
// trajectory so the noise floor of THIS capture can be inspected directly.
std::vector<float> energy_metric_trajectory(const std::complex<int16_t>* x, size_t n)
{
    constexpr size_t kBlock = 100;
    constexpr size_t kPerBlock = 16;
    constexpr size_t kWindow = 32;
    std::vector<float> out;
    if (n < kBlock * kPerBlock) {
        return out;
    }
    out.reserve(n / kBlock);
    constexpr double full_scale_squared = 32768.0 * 32768.0;
    double ring[kWindow] = {};
    size_t pos = 0;
    size_t count = 0;
    double window_sum = 0.0;
    for (size_t b = 0; b + kBlock <= n; b += kBlock) {
        double acc = 0.0;
        for (size_t i = 0; i < kPerBlock; ++i) {
            const int64_t re = x[b + i].real();
            const int64_t im = x[b + i].imag();
            acc += static_cast<double>(re * re + im * im);
        }
        window_sum -= ring[pos];
        ring[pos] = acc;
        window_sum += acc;
        pos = (pos + 1) % kWindow;
        if (count < kWindow) {
            ++count;
        }
        // metric = 16 * mean |x|^2: the detector compares window_sum against
        // threshold*FS^2*count, i.e. window_sum/(count*16*FS^2) >= threshold/16,
        // so the reported quantity is 16 x the mean power.
        out.push_back(static_cast<float>(
            16.0 * window_sum / (static_cast<double>(count) * static_cast<double>(kPerBlock)) /
            full_scale_squared));
    }
    return out;
}

float percentile_of(std::vector<float> v, double p)
{
    if (v.empty()) {
        return 0.0f;
    }
    std::sort(v.begin(), v.end());
    const double idx = p * static_cast<double>(v.size() - 1);
    size_t pos = static_cast<size_t>(std::llround(idx));
    if (pos >= v.size()) {
        pos = v.size() - 1;
    }
    return v[pos];
}

struct Packet {
    uint64_t start = 0;
    float metric = 0.0f;
    float start_metric = 0.0f;
    size_t backtracked = 0;
};

struct RegionStat {
    uint64_t start = 0;
    uint64_t candidate = 0;
    size_t samples = 0;
    bool confirmed = false;
    float metric = 0.0f;
    float gate = 0.0f;
};

} // namespace

int main(int argc, char* argv[])
{
    Options opt;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        auto need = [&](const char* what) -> const char* {
            if (i + 1 >= argc) {
                std::cerr << "Error: " << what << " requires an argument\n";
                std::exit(1);
            }
            return argv[++i];
        };
        if (arg == "--help" || arg == "-h") {
            print_help(argv[0]);
            return 0;
        } else if (arg == "--out") {
            opt.out_path = need("--out");
        } else if (arg == "--ms") {
            opt.duration_ms = std::stod(need("--ms"));
        } else if (arg == "--settle-ms") {
            opt.settle_ms = std::stod(need("--settle-ms"));
        } else if (arg == "--native-rate") {
            opt.native_rate = std::stod(need("--native-rate"));
        } else if (arg == "--uhd-args") {
            opt.uhd_args = need("--uhd-args");
        } else if (arg == "--uhd-subdev") {
            opt.uhd_subdev = need("--uhd-subdev");
        } else if (arg == "--uhd-channel") {
            opt.uhd_channel = need("--uhd-channel");
        } else if (arg == "--uhd-antenna") {
            opt.uhd_antenna = need("--uhd-antenna");
        } else if (arg == "--freq") {
            opt.uhd_freq = std::stod(need("--freq"));
        } else if (arg == "--gain") {
            opt.uhd_gain = std::stod(need("--gain"));
        } else if (arg == "--template") {
            opt.template_file = need("--template");
        } else if (arg == "--energy-threshold") {
            opt.energy_threshold = std::stof(need("--energy-threshold"));
        } else if (arg == "--fine-threshold") {
            opt.fine_threshold = std::stof(need("--fine-threshold"));
        } else if (arg == "--pre-trigger") {
            opt.pre_trigger = static_cast<size_t>(std::stoull(need("--pre-trigger")));
        } else if (arg == "--capture") {
            opt.capture = static_cast<size_t>(std::stoull(need("--capture")));
        } else if (arg == "--coarse-decimation") {
            opt.coarse_decimation =
                static_cast<size_t>(std::stoull(need("--coarse-decimation")));
        } else if (arg == "--coarse-repetitions") {
            opt.coarse_repetitions =
                static_cast<size_t>(std::stoull(need("--coarse-repetitions")));
        } else if (arg == "--coarse-margin") {
            opt.coarse_margin = static_cast<size_t>(std::stoull(need("--coarse-margin")));
        } else if (arg == "--no-detect") {
            opt.detect = false;
        } else {
            std::cerr << "Error: unrecognized option '" << arg << "'\n";
            print_help(argv[0]);
            return 1;
        }
    }

    if (!(opt.duration_ms > 0.0)) {
        std::cerr << "Error: --ms must be > 0\n";
        return 1;
    }
    if (!(opt.native_rate > 0.0)) {
        std::cerr << "Error: --native-rate must be > 0\n";
        return 1;
    }
    if (!(opt.fine_threshold > 0.0f && opt.fine_threshold <= 1.0f)) {
        std::cerr << "Error: --fine-threshold must be in (0, 1]\n";
        return 1;
    }

    std::vector<std::complex<float>> tmpl;
    if (opt.detect && !read_complex_file(opt.template_file, tmpl)) {
        return 1;
    }

    const size_t target_samples =
        static_cast<size_t>(std::llround(opt.duration_ms * 1e-3 * opt.native_rate));
    const size_t settle_samples =
        static_cast<size_t>(std::llround(std::max(0.0, opt.settle_ms) * 1e-3 * opt.native_rate));

    PipelineConfig cfg;
    cfg.source_type = StreamSourceType::UhdDevice;
    cfg.native_rate = opt.native_rate;
    cfg.work_rate = opt.native_rate;
    cfg.uhd_args = opt.uhd_args;
    cfg.uhd_subdev = opt.uhd_subdev;
    cfg.uhd_channel = opt.uhd_channel;
    cfg.uhd_antenna = opt.uhd_antenna;
    cfg.uhd_center_freq = opt.uhd_freq;
    cfg.uhd_gain = opt.uhd_gain;

    UhdDeviceSc16Source source(cfg);
    if (!source.open()) {
        std::cerr << "Error: UHD open failed (args='" << opt.uhd_args << "', subdev='"
                  << opt.uhd_subdev << "'). Check uhd_find_devices.\n";
        return 1;
    }

    const double actual_rate = source.actual_rx_rate();
    std::cout << "uwb-uhd-sc16-capture\n"
              << "  uhd args      : " << (opt.uhd_args.empty() ? "<none>" : opt.uhd_args) << "\n"
              << "  subdev        : " << (opt.uhd_subdev.empty() ? "<default>" : opt.uhd_subdev)
              << "\n"
              << "  channel       : " << (opt.uhd_channel.empty() ? "<default>" : opt.uhd_channel)
              << "\n"
              << "  freq          : " << std::fixed << std::setprecision(3)
              << opt.uhd_freq / 1e6 << " MHz\n"
              << "  gain          : " << std::setprecision(1) << opt.uhd_gain << " dB\n"
              << "  rate          : " << std::setprecision(3) << actual_rate / 1e6
              << " MS/s (requested " << opt.native_rate / 1e6 << ")\n"
              << "  target        : " << std::setprecision(3) << opt.duration_ms << " ms ("
              << target_samples << " samples)\n"
              << "  settle        : " << std::setprecision(3) << opt.settle_ms << " ms ("
              << settle_samples << " samples)\n"
              << std::flush;

    std::vector<std::complex<int16_t>> buf(target_samples);
    size_t got = 0;
    size_t settled = 0;
    size_t overflows = 0;
    size_t timeouts = 0;
    std::vector<std::complex<int16_t>> scratch(1u << 16);

    if (!source.start_stream()) {
        std::cerr << "Error: issue_stream_cmd(START_CONTINUOUS) failed\n";
        source.close();
        return 1;
    }

    const auto t_start = std::chrono::steady_clock::now();
    // Settle: drain the first samples so device/stream transients never reach
    // the capture buffer.
    while (settled < settle_samples) {
        StreamReadMeta meta;
        const size_t want = std::min(scratch.size(), settle_samples - settled);
        const size_t n = source.read_samples(scratch.data(), want, meta);
        if (meta.is_gap) {
            ++overflows;
        }
        if (n == 0) {
            if (++timeouts > 16) {
                std::cerr << "Error: no samples during settle (persistent timeout)\n";
                source.close();
                return 1;
            }
            continue;
        }
        timeouts = 0;
        settled += n;
    }

    while (got < target_samples) {
        StreamReadMeta meta;
        const size_t want = std::min(scratch.size(), target_samples - got);
        const size_t n = source.read_samples(scratch.data(), want, meta);
        if (meta.is_gap) {
            ++overflows;
        }
        if (n == 0) {
            if (meta.is_gap) {
                // A short read paired with a gap is normal overflow recovery.
                continue;
            }
            if (++timeouts > 16) {
                std::cerr << "Warning: capture ended early after repeated timeouts\n";
                break;
            }
            continue;
        }
        timeouts = 0;
        std::memcpy(buf.data() + got, scratch.data(), n * sizeof(std::complex<int16_t>));
        got += n;
    }
    const auto t_end = std::chrono::steady_clock::now();
    source.close();
    buf.resize(got);

    const double wall_s = std::chrono::duration<double>(t_end - t_start).count();
    std::cout << "  received      : " << got << " samples\n"
              << "  wall          : " << std::fixed << std::setprecision(3) << wall_s * 1e3
              << " ms\n"
              << "  stream msps   : " << std::setprecision(2)
              << (wall_s > 0.0 ? static_cast<double>(settled + got) / wall_s / 1e6 : 0.0)
              << " MS/s\n"
              << "  overflows     : " << overflows << "\n"
              << "  timeouts      : " << timeouts << "\n";

    if (got == 0) {
        std::cerr << "Error: no samples captured\n";
        return 1;
    }

    // Basic amplitude statistics on the captured block.
    float max_abs = 0.0f;
    double sum_pow = 0.0;
    for (const auto& s : buf) {
        const float re = static_cast<float>(s.real()) / 32768.0f;
        const float im = static_cast<float>(s.imag()) / 32768.0f;
        const float a = std::sqrt(re * re + im * im);
        max_abs = std::max(max_abs, a);
        sum_pow += static_cast<double>(re) * re + static_cast<double>(im) * im;
    }
    const double mean_pow = sum_pow / static_cast<double>(buf.size());
    std::cout << "  max |x|       : " << std::setprecision(4) << max_abs << "\n"
              << "  mean power    : " << std::scientific << std::setprecision(3) << mean_pow
              << std::defaultfloat << "\n";

    if (!opt.out_path.empty()) {
        std::ofstream of(opt.out_path, std::ios::binary | std::ios::trunc);
        if (!of) {
            std::cerr << "Error: cannot open output '" << opt.out_path << "'\n";
            return 1;
        }
        of.write(reinterpret_cast<const char*>(buf.data()),
                 static_cast<std::streamsize>(buf.size() * sizeof(std::complex<int16_t>)));
        of.close();
        if (!of) {
            std::cerr << "Error: write failed for '" << opt.out_path << "'\n";
            return 1;
        }
        std::cout << "  raw sc16      : " << opt.out_path << " ("
                  << (buf.size() * sizeof(std::complex<int16_t>) / (1024 * 1024)) << " MiB)\n";
    }

    if (!opt.detect) {
        std::cout << "\nDetection skipped (--no-detect).\n";
        return 0;
    }

    // ------------------------------------------------------------------
    // Native-grid detection over the captured block.
    // ------------------------------------------------------------------
    std::vector<float> trajectory = energy_metric_trajectory(buf.data(), buf.size());
    std::cout << "\nEnergy gate (16 x mean |x|^2, 32 blocks of 100 samples):\n";
    if (trajectory.empty()) {
        std::cout << "  <capture too short>\n";
    } else {
        std::vector<float> sorted = trajectory;
        std::sort(sorted.begin(), sorted.end());
        std::cout << "  blocks        : " << trajectory.size() << "\n"
                  << "  median        : " << std::scientific << std::setprecision(3)
                  << percentile_of(trajectory, 0.5) << "\n"
                  << "  p99           : " << percentile_of(trajectory, 0.99) << "\n"
                  << "  max           : " << sorted.back() << std::defaultfloat << "\n"
                  << "  threshold     : " << std::scientific << std::setprecision(3)
                  << opt.energy_threshold << std::defaultfloat << "\n";
    }

    UwbDetectorStateMachineSc16 sm(opt.pre_trigger + 32 * 100 + tmpl.size(),
                                  opt.energy_threshold,
                                  100,
                                  /*gate_window=*/32,
                                  /*holdoff_decimated=*/8,
                                  opt.capture);

    UwbPreambleVerifierSc16::Config vcfg;
    vcfg.coarse_decimation = opt.coarse_decimation;
    vcfg.coarse_repetitions = opt.coarse_repetitions;
    vcfg.coarse_margin = opt.coarse_margin;
    vcfg.fine_threshold = opt.fine_threshold;
    UwbPreambleVerifierSc16 verifier;
    verifier.configure(tmpl, vcfg);
    verifier.reserve_coarse(
        opt.pre_trigger + 32 * 100 + tmpl.size() +
        tmpl.size() * (openuwb::core::kUwbSyncSymbols + openuwb::core::kUwbSfdSymbols + 4));

    std::vector<RegionStat> regions;
    std::vector<Packet> packets;

    auto drain_regions = [&](uint64_t processed_abs) {
        while (sm.region_ready()) {
            const auto handle = sm.take_region();
            const auto& region = sm.region(handle);
            const auto vr = verifier.verify(region.samples.data(), region.samples.size(),
                                            region.candidate_offset);
            RegionStat st;
            st.start = region.start_abs;
            st.candidate = region.start_abs + region.candidate_offset;
            st.samples = region.samples.size();
            st.confirmed = vr.confirmed;
            st.metric = vr.detection_metric;
            regions.push_back(st);
            if (vr.confirmed) {
                Packet p;
                p.start = region.start_abs + static_cast<uint64_t>(vr.start_offset);
                p.metric = vr.detection_metric;
                p.start_metric = vr.start_metric;
                p.backtracked = vr.backtracked_symbols;
                packets.push_back(p);
            }
            sm.release_region(handle);
            (void)processed_abs;
        }
    };

    constexpr size_t kChunk = 1u << 16;
    for (size_t off = 0; off < buf.size(); off += kChunk) {
        const size_t n = std::min(kChunk, buf.size() - off);
        sm.process(buf.data() + off, n, off);
        drain_regions(off + n);
    }
    sm.flush_region();
    drain_regions(buf.size());

    std::sort(packets.begin(), packets.end(),
              [](const Packet& a, const Packet& b) { return a.start < b.start; });

    const double rate = actual_rate > 0.0 ? actual_rate : opt.native_rate;
    const auto to_ms = [rate](uint64_t s) {
        return static_cast<double>(s) / rate * 1e3;
    };

    std::cout << "\nDetection:\n"
              << "  template      : " << opt.template_file << " (" << tmpl.size()
              << " samples)\n"
              << "  thresholds    : energy=" << std::scientific << std::setprecision(3)
              << opt.energy_threshold << std::defaultfloat << " fine=" << std::fixed
              << std::setprecision(2) << opt.fine_threshold << "\n"
              << "  regions       : " << regions.size() << " (dropped "
              << sm.dropped_regions() << ")\n"
              << "  confirmed     : " << packets.size() << "\n";

    if (!regions.empty()) {
        std::cout << "\n  Regions:\n"
                  << "  ------------------------------------------------------------------\n"
                  << "   idx   region_start (ms)   samples   confirmed   metric\n"
                  << "  ------------------------------------------------------------------\n";
        for (size_t i = 0; i < regions.size(); ++i) {
            const auto& r = regions[i];
            std::cout << "  " << std::setw(4) << i << "  " << std::fixed
                      << std::setprecision(4) << std::setw(16) << to_ms(r.start) << "  "
                      << std::setw(8) << r.samples << "   " << (r.confirmed ? "yes" : "no ")
                      << "        " << std::setprecision(4) << r.metric << "\n";
        }
        std::cout << "  ------------------------------------------------------------------\n";
    }

    if (!packets.empty()) {
        std::cout << "\n  Packets:\n"
                  << "  ------------------------------------------------------------------\n"
                  << "   idx   start_sample      start (ms)    metric   backtrack\n"
                  << "  ------------------------------------------------------------------\n";
        for (size_t i = 0; i < packets.size(); ++i) {
            const auto& p = packets[i];
            std::cout << "  " << std::setw(4) << i << "  " << std::setw(14) << p.start << "  "
                      << std::fixed << std::setprecision(4) << std::setw(12) << to_ms(p.start)
                      << "  " << std::setprecision(4) << std::setw(8) << p.metric << "  "
                      << std::setw(9) << p.backtracked << "\n";
        }
        std::cout << "  ------------------------------------------------------------------\n";
        if (packets.size() > 1) {
            std::cout << "  spacing (ms)  : ";
            double sum = 0.0;
            for (size_t i = 1; i < packets.size(); ++i) {
                const double d = to_ms(packets[i].start - packets[i - 1].start);
                sum += d;
                std::cout << std::fixed << std::setprecision(4) << d
                          << (i + 1 < packets.size() ? ", " : "");
            }
            std::cout << "  (mean " << std::setprecision(4)
                      << sum / static_cast<double>(packets.size() - 1) << ")\n";
        }
    }

    // Non-zero exit when nothing was confirmed would be wrong for a diagnostic
    // tool on a quiet band; the run itself succeeded.
    return 0;
}
