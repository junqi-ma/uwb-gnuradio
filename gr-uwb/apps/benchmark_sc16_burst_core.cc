/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Pure-core throughput benchmark for the rational resampler SC16 path.
 * Owner: Subagent A (resampler core).  Contract:
 *   docs/performance/接口冻结_UHD_SC16第二轮优化_2026-11-11.md §6
 *   docs/performance/OpenCode_UHD_SC16重采样与流水线优化任务书_2026-11-11.md §7
 *
 * It measures EXACTLY the production per-window sequence of
 * uwb_uhd_sc16_pipeline.cc (same calls, same output grant, same drain loop):
 *
 *     core.reset();
 *     r = core.process_sc16(in, N, scale, out, cap);      // cap = expected
 *     produced = r.produced;
 *     while (produced < expected) {                       // flush to completion
 *         n = core.flush(out + produced, cap - produced);
 *         if (n == 0) break;
 *         produced += n;
 *     }
 *
 * Everything is pre-allocated before the first measured repeat; the measured
 * region contains no allocation, no I/O and no argument parsing.
 *
 * Deterministic input (documented so any reported checksum is reproducible):
 *   uint32 s = seed | 1
 *   per sample, real then imag:
 *       s = s * 1664525 + 1013904223            (mod 2^32, Numerical Recipes LCG)
 *       v = (int32)(s >> 16) - 32768            -> int16 range [-32768, 32767]
 * i.e. hash-of-LCG high half, identical to the SC16 stimulus used by
 * lib/qa_uwb_resampler_burst_opt.cc.  --input rails|impulse|zero replace it
 * with pinned extremes / a single full-scale sample / silence for debugging;
 * the default is the LCG.
 *
 * Output checksum: FNV-1a 64 over the whole concatenated CF32 output of a
 * repeat (produced * 8 bytes).  It is recomputed after every repeat, outside
 * the timed region, both to stop the compiler from eliminating anything and
 * to prove repeat-to-repeat determinism (all repeats must agree, otherwise
 * checksum_stable is false and the run exits non-zero).
 *
 * Percentiles use the nearest-rank definition on the per-repeat samples:
 *   index = ceil(p/100 * R) - 1, clamped to [0, R-1].
 *
 * Timing fields (all per repeat, microseconds):
 *   reset    core.reset()
 *   process  core.process_sc16() only
 *   core     reset + process            (the non-flush part of the window)
 *   flush    the whole flush-to-completion drain
 *   total    core + flush               (comparable to the pipeline's
 *                                        WindowResult.timing.resample_us)
 *
 * Throughput is derived from the MEAN of total / core, and "window" means one
 * measured window of N native input samples producing `output_samples`
 * CF32 samples.
 */

#include <gnuradio/uwb/uwb_rational_resampler_core.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace {

using gr_complex = std::complex<float>;
using sc16 = std::complex<int16_t>;

using gr::uwb::core::RationalResampler32_65Core;
using gr::uwb::core::RationalResampler48_65Core;
using gr::uwb::core::RationalResampler65_32Core;
using gr::uwb::core::RationalResampler65_48Core;

// ------------------------------------------------------------------ utils --

uint64_t fnv1a64(const void* data, size_t bytes, uint64_t h = 1469598103934665603ULL)
{
    const unsigned char* p = static_cast<const unsigned char*>(data);
    for (size_t i = 0; i < bytes; ++i) {
        h ^= static_cast<uint64_t>(p[i]);
        h *= 1099511628211ULL;
    }
    return h;
}

std::string hex64(uint64_t v)
{
    char buf[24];
    std::snprintf(buf, sizeof(buf), "0x%016llx",
                  static_cast<unsigned long long>(v));
    return std::string(buf);
}

std::vector<float> load_taps(const std::string& path)
{
    std::ifstream f(path, std::ios::binary | std::ios::ate);
    if (!f) {
        std::cerr << "error: cannot open taps file '" << path << "'\n";
        std::exit(2);
    }
    const std::streamoff sz = f.tellg();
    if (sz <= 0 || (sz % static_cast<std::streamoff>(sizeof(float))) != 0) {
        std::cerr << "error: '" << path
                  << "' is not a non-empty binary float32 file (size " << sz
                  << ")\n";
        std::exit(2);
    }
    f.seekg(0);
    std::vector<float> v(static_cast<size_t>(sz) / sizeof(float));
    f.read(reinterpret_cast<char*>(v.data()),
           static_cast<std::streamsize>(sz));
    if (!f) {
        std::cerr << "error: short read on '" << path << "'\n";
        std::exit(2);
    }
    return v;
}

std::vector<float> load_taps_resolved(const std::string& path)
{
    // Absolute / existing path first, then the usual testdata/ prefixes so a
    // hand run from the build tree or from build/lib both work.
    {
        std::ifstream probe(path, std::ios::binary);
        if (probe)
            return load_taps(path);
    }
#ifdef UWB_TESTDATA_DIR
    {
        const std::string p = std::string(UWB_TESTDATA_DIR) + "/" + path;
        std::ifstream probe(p, std::ios::binary);
        if (probe)
            return load_taps(p);
    }
#endif
    const char* prefixes[] = { "", "../", "../../", "../../../", "../../../../" };
    for (const char* pre : prefixes) {
        const std::string p = std::string(pre) + "testdata/" + path;
        std::ifstream probe(p, std::ios::binary);
        if (probe)
            return load_taps(p);
    }
    return load_taps(path); // fails loudly
}

/** Documented deterministic SC16 stimulus (see file header). */
std::vector<sc16> make_input(size_t n, uint32_t seed, const std::string& mode)
{
    std::vector<sc16> v(n, sc16(0, 0));
    if (mode == "zero")
        return v;
    if (mode == "rails") {
        static const int16_t rails[] = { 32767, -32768, -32767, 0, 32767, -32768 };
        for (size_t i = 0; i < n; ++i) {
            v[i] = sc16(rails[i % 6], rails[(i + 3) % 6]);
        }
        return v;
    }
    if (mode == "impulse") {
        if (n > 0)
            v[n / 2] = sc16(32767, -32768);
        return v;
    }
    uint32_t s = seed | 1u;
    for (size_t i = 0; i < n; ++i) {
        s = s * 1664525u + 1013904223u;
        const int32_t re = static_cast<int32_t>(s >> 16) - 32768;
        s = s * 1664525u + 1013904223u;
        const int32_t im = static_cast<int32_t>(s >> 16) - 32768;
        v[i] = sc16(static_cast<int16_t>(re), static_cast<int16_t>(im));
    }
    return v;
}

double pct_sorted(const std::vector<double>& sorted, double p)
{
    if (sorted.empty())
        return 0.0;
    const size_t n = sorted.size();
    double idx = std::ceil(p * 0.01 * static_cast<double>(n)) - 1.0;
    if (idx < 0.0)
        idx = 0.0;
    if (idx > static_cast<double>(n - 1))
        idx = static_cast<double>(n - 1);
    return sorted[static_cast<size_t>(idx)];
}

struct Stats {
    double mean = 0.0;
    double min = 0.0;
    double p50 = 0.0;
    double p95 = 0.0;
    double p99 = 0.0;
    double max = 0.0;
};

Stats summarize(std::vector<double> v)
{
    Stats s;
    if (v.empty())
        return s;
    double sum = 0.0;
    for (double x : v)
        sum += x;
    s.mean = sum / static_cast<double>(v.size());
    std::sort(v.begin(), v.end());
    s.min = v.front();
    s.max = v.back();
    s.p50 = pct_sorted(v, 50.0);
    s.p95 = pct_sorted(v, 95.0);
    s.p99 = pct_sorted(v, 99.0);
    return s;
}

struct Options {
    std::string taps;
    std::string rate = "65_48";
    std::string scale_mode = "unit";
    std::string kernel = "default";
    std::string json;
    std::string dump_iq;
    std::string input = "random";
    size_t length = 4096;
    size_t reps = 50;
    size_t warmup = 20;
    uint32_t seed = 0x1234567u;
    bool fixed_phase = false;
    bool convert_cache = false;
};

void usage(const char* argv0)
{
    std::printf(
        "usage: %s --taps <path> [options]\n"
        "\n"
        "  --taps <path>          binary float32 taps "
        "(testdata/resampler_65_48/taps_{realtime,quality}_minorder.txt)\n"
        "  --rate <name>          65_48 (default) | 65_32 | 48_65 | 32_65\n"
        "  --scale-mode <m>       unit (1/32768, default) | raw (1.0)\n"
        "  --length <N>           native input samples per measured window "
        "(default 4096)\n"
        "  --reps <R>             measured repeats (default 50)\n"
        "  --warmup <W>           warmup repeats (default 20)\n"
        "  --kernel <name>        pass-through to set_kernel() "
        "(default avx2_fma_macroblock)\n"
        "  --fixed-phase <0|1>    C1 fixed-phase schedule (default 0)\n"
        "  --convert-cache <0|1>  C2 per-macroblock float cache (default 0)\n"
        "  --input <mode>         random (default) | rails | impulse | zero\n"
        "  --seed <n>             LCG seed (default 0x1234567)\n"
        "  --json <path>          write the JSON result file\n"
        "  --dump-iq <path>       write one repeat of CF32 output (raw bytes)\n"
        "  --help                 this text\n",
        argv0);
}

bool parse_bool01(const std::string& v, bool& out)
{
    if (v == "1" || v == "true" || v == "on") {
        out = true;
        return true;
    }
    if (v == "0" || v == "false" || v == "off") {
        out = false;
        return true;
    }
    return false;
}

// ----------------------------------------------------------- measurement ---

struct WindowResult {
    size_t produced = 0;
    double reset_us = 0.0;
    double process_us = 0.0;
    double flush_us = 0.0;
    uint64_t checksum = 0;
};

/**
 * One production window on `core`: reset -> process_sc16 -> flush to
 * completion.  Timed with steady_clock around exactly those calls.
 * `core_us` is reset + process; `flush_us` is the drain.
 */
template <typename Core>
WindowResult run_window(Core& core,
                        const std::vector<sc16>& in,
                        float scale,
                        std::vector<gr_complex>& out,
                        size_t expect)
{
    using Clock = std::chrono::steady_clock;
    WindowResult w;

    const auto t0 = Clock::now();
    core.reset();
    const auto t1 = Clock::now();

    size_t produced = 0;
    if (!in.empty()) {
        auto r = core.process_sc16(in.data(), in.size(), scale, out.data(),
                                   out.size());
        produced = r.produced;
    }
    const auto t2 = Clock::now();

    while (produced < expect) {
        const size_t n = core.flush(out.data() + produced, out.size() - produced);
        if (n == 0)
            break;
        produced += n;
    }
    const auto t3 = Clock::now();

    w.produced = produced;
    w.reset_us =
        std::chrono::duration<double, std::micro>(t1 - t0).count();
    w.process_us =
        std::chrono::duration<double, std::micro>(t2 - t1).count();
    w.flush_us =
        std::chrono::duration<double, std::micro>(t3 - t2).count();
    w.checksum = fnv1a64(out.data(), produced * sizeof(gr_complex));
    return w;
}

template <typename Core>
int benchmark_core(const Options& opt,
                   const std::vector<float>& taps,
                   size_t tap_fnv)
{
    using Clock = std::chrono::steady_clock;
    const float scale = (opt.scale_mode == "raw") ? 1.0f : (1.0f / 32768.0f);

    // ---- prepare (timed separately; never inside the measured region) ----
    const auto t_prep0 = Clock::now();

    std::vector<sc16> in = make_input(opt.length, opt.seed, opt.input);
    Core core(taps);
    core.set_kernel(opt.kernel);
    core.set_fixed_phase_schedule(opt.fixed_phase);
    core.set_convert_cache(opt.convert_cache);

    const size_t expect =
        Core::expected_output_length(in.size(), core.tap_count());
    // Same grant the pipeline uses (cf32_work.size() == expected_len in the
    // steady state) plus a little slack so the drain cannot be truncated by
    // the buffer itself.
    std::vector<gr_complex> out(expect + 4096, gr_complex(0.0f, 0.0f));

    const auto t_prep1 = Clock::now();
    const double prepare_us =
        std::chrono::duration<double, std::micro>(t_prep1 - t_prep0).count();

    // ---- warmup (not measured) ----
    uint64_t warm_checksum = 0;
    size_t warm_produced = 0;
    for (size_t i = 0; i < opt.warmup; ++i) {
        WindowResult w = run_window(core, in, scale, out, expect);
        warm_checksum = w.checksum;
        warm_produced = w.produced;
    }
    if (opt.warmup > 0 && warm_produced != expect) {
        std::cerr << "error: warmup produced " << warm_produced
                  << " samples, expected " << expect << "\n";
        return 4;
    }

    // ---- measured repeats ----
    std::vector<double> reset_us, process_us, core_us, flush_us, total_us;
    reset_us.reserve(opt.reps);
    process_us.reserve(opt.reps);
    core_us.reserve(opt.reps);
    flush_us.reserve(opt.reps);
    total_us.reserve(opt.reps);

    uint64_t checksum = 0;
    bool checksum_stable = true;
    size_t produced_last = 0;
    bool length_ok = true;

    const auto t_meas0 = Clock::now();
    for (size_t i = 0; i < opt.reps; ++i) {
        WindowResult w = run_window(core, in, scale, out, expect);
        reset_us.push_back(w.reset_us);
        process_us.push_back(w.process_us);
        core_us.push_back(w.reset_us + w.process_us);
        flush_us.push_back(w.flush_us);
        total_us.push_back(w.reset_us + w.process_us + w.flush_us);
        if (i == 0)
            checksum = w.checksum;
        else if (w.checksum != checksum)
            checksum_stable = false;
        if (opt.warmup > 0 && w.checksum != warm_checksum)
            checksum_stable = false;
        produced_last = w.produced;
        if (w.produced != expect)
            length_ok = false;
    }
    const auto t_meas1 = Clock::now();
    const double wall_us =
        std::chrono::duration<double, std::micro>(t_meas1 - t_meas0).count();

    const Stats s_reset = summarize(reset_us);
    const Stats s_process = summarize(process_us);
    const Stats s_core = summarize(core_us);
    const Stats s_flush = summarize(flush_us);
    const Stats s_total = summarize(total_us);

    const double n = static_cast<double>(opt.length);
    const double msps_core = (s_core.mean > 0.0) ? n / s_core.mean : 0.0;
    const double msps_total = (s_total.mean > 0.0) ? n / s_total.mean : 0.0;
    const double win_core = (s_core.mean > 0.0) ? 1.0e6 / s_core.mean : 0.0;
    const double win_total = (s_total.mean > 0.0) ? 1.0e6 / s_total.mean : 0.0;

    // ---- optional byte-exact IQ dump (last repeat; all repeats agree) ----
    bool dumped = false;
    if (!opt.dump_iq.empty()) {
        std::ofstream f(opt.dump_iq, std::ios::binary | std::ios::trunc);
        if (!f) {
            std::cerr << "error: cannot open --dump-iq '" << opt.dump_iq
                      << "'\n";
            return 3;
        }
        f.write(reinterpret_cast<const char*>(out.data()),
                static_cast<std::streamsize>(produced_last * sizeof(gr_complex)));
        if (!f) {
            std::cerr << "error: write failed for --dump-iq\n";
            return 3;
        }
        dumped = true;
    }

    // ---- human readable summary ----
    std::printf("benchmark_sc16_burst_core\n");
    std::printf("  taps              : %s (T=%zu, FNV-1a %s)\n",
                opt.taps.c_str(), taps.size(), hex64(tap_fnv).c_str());
    std::printf("  rate              : %s (L=%u M=%u)  H=%zu H_pad=%zu\n",
                opt.rate.c_str(), Core::kInterp, Core::kDecim,
                core.arm_length(), core.arm_length_pad());
    std::printf("  kernel            : %s (%s)\n", core.kernel_name(),
                core.sc16_kernel_variant());
    std::printf("  switches          : fixed-phase=%d convert-cache=%d\n",
                opt.fixed_phase ? 1 : 0, opt.convert_cache ? 1 : 0);
    std::printf("  scale             : %s (%.10g)\n", opt.scale_mode.c_str(),
                static_cast<double>(scale));
    std::printf("  input             : %s seed=0x%08x\n", opt.input.c_str(),
                opt.seed);
    std::printf("  window            : %zu native samples -> %zu CF32 (%s)\n",
                opt.length, produced_last, length_ok ? "length ok" : "LENGTH MISMATCH");
    std::printf("  repeats           : %zu measured, %zu warmup\n", opt.reps,
                opt.warmup);
    std::printf("  prepare           : %.3f us\n", prepare_us);
    std::printf("  reset/process     : %.3f / %.3f us (mean)\n", s_reset.mean,
                s_process.mean);
    std::printf("  core (reset+proc) : mean %.3f  P50 %.3f  P95 %.3f  P99 %.3f  "
                "min %.3f  max %.3f us\n",
                s_core.mean, s_core.p50, s_core.p95, s_core.p99, s_core.min,
                s_core.max);
    std::printf("  flush             : mean %.3f  P50 %.3f  P95 %.3f  P99 %.3f us\n",
                s_flush.mean, s_flush.p50, s_flush.p95, s_flush.p99);
    std::printf("  total             : mean %.3f  P50 %.3f  P95 %.3f  P99 %.3f  "
                "min %.3f  max %.3f us\n",
                s_total.mean, s_total.p50, s_total.p95, s_total.p99, s_total.min,
                s_total.max);
    std::printf("  throughput        : core %.3f native MS/s (%.1f win/s) | "
                "total %.3f native MS/s (%.1f win/s)\n",
                msps_core, win_core, msps_total, win_total);
    std::printf("  wall (all reps)   : %.3f ms\n", wall_us / 1000.0);
    std::printf("  checksum          : %s (%s)\n", hex64(checksum).c_str(),
                checksum_stable ? "stable" : "UNSTABLE");
    if (dumped)
        std::printf("  dump-iq           : %s (%zu bytes)\n",
                    opt.dump_iq.c_str(),
                    produced_last * sizeof(gr_complex));

    // ---- JSON ----
    if (!opt.json.empty()) {
        std::ofstream f(opt.json, std::ios::trunc);
        if (!f) {
            std::cerr << "error: cannot open --json '" << opt.json << "'\n";
            return 3;
        }
        auto stats_json = [](std::ostringstream& o, const char* key,
                             const Stats& s) {
            o << "    \"" << key << "\": {\"mean\": " << s.mean
              << ", \"min\": " << s.min << ", \"p50\": " << s.p50
              << ", \"p95\": " << s.p95 << ", \"p99\": " << s.p99
              << ", \"max\": " << s.max << "}";
        };
        std::ostringstream o;
        o.precision(10);
        o << "{\n";
        o << "  \"tool\": \"benchmark_sc16_burst_core\",\n";
        o << "  \"contract\": \"docs/performance/接口冻结_UHD_SC16第二轮优化_2026-11-11.md#6\",\n";
        o << "  \"taps_path\": \"" << opt.taps << "\",\n";
        o << "  \"taps_count\": " << taps.size() << ",\n";
        o << "  \"taps_fnv1a64\": \"" << hex64(tap_fnv) << "\",\n";
        o << "  \"rate\": \"" << opt.rate << "\",\n";
        o << "  \"interp\": " << Core::kInterp << ",\n";
        o << "  \"decim\": " << Core::kDecim << ",\n";
        o << "  \"arm_length\": " << core.arm_length() << ",\n";
        o << "  \"arm_length_pad\": " << core.arm_length_pad() << ",\n";
        o << "  \"kernel\": \"" << core.kernel_name() << "\",\n";
        o << "  \"sc16_kernel_variant\": \"" << core.sc16_kernel_variant()
          << "\",\n";
        o << "  \"fixed_phase\": " << (opt.fixed_phase ? 1 : 0) << ",\n";
        o << "  \"convert_cache\": " << (opt.convert_cache ? 1 : 0) << ",\n";
        o << "  \"convert_cache_span\": " << core.convert_cache_span() << ",\n";
        o << "  \"scale_mode\": \"" << opt.scale_mode << "\",\n";
        o << "  \"scale\": " << static_cast<double>(scale) << ",\n";
        o << "  \"length\": " << opt.length << ",\n";
        o << "  \"reps\": " << opt.reps << ",\n";
        o << "  \"warmup\": " << opt.warmup << ",\n";
        o << "  \"input_mode\": \"" << opt.input << "\",\n";
        o << "  \"seed\": " << opt.seed << ",\n";
        o << "  \"expected_output_samples\": " << expect << ",\n";
        o << "  \"output_samples\": " << produced_last << ",\n";
        o << "  \"length_ok\": " << (length_ok ? "true" : "false") << ",\n";
        o << "  \"checksum_fnv1a64\": \"" << hex64(checksum) << "\",\n";
        o << "  \"checksum_stable\": " << (checksum_stable ? "true" : "false")
          << ",\n";
        o << "  \"prepare_us\": " << prepare_us << ",\n";
        o << "  \"wall_us_all_reps\": " << wall_us << ",\n";
        o << "  \"timing_us\": {\n";
        stats_json(o, "reset", s_reset);
        o << ",\n";
        stats_json(o, "process", s_process);
        o << ",\n";
        stats_json(o, "core", s_core);
        o << ",\n";
        stats_json(o, "flush", s_flush);
        o << ",\n";
        stats_json(o, "total", s_total);
        o << "\n  },\n";
        o << "  \"throughput\": {\"native_msps_core\": " << msps_core
          << ", \"native_msps_total\": " << msps_total
          << ", \"windows_per_s_core\": " << win_core
          << ", \"windows_per_s_total\": " << win_total << "},\n";
        o << "  \"percentile_method\": \"nearest-rank\",\n";
        o << "  \"input_lcg\": \"s=s*1664525+1013904223; v=(int32)(s>>16)-32768\",\n";
        o << "  \"dump_iq\": ";
        if (dumped)
            o << "\"" << opt.dump_iq << "\"";
        else
            o << "null";
        o << ",\n";
        o << "  \"argv\": [";
        o << "\"--taps\", \"" << opt.taps << "\", \"--rate\", \"" << opt.rate
          << "\", \"--scale-mode\", \"" << opt.scale_mode << "\", \"--length\", "
          << opt.length << ", \"--reps\", " << opt.reps << ", \"--warmup\", "
          << opt.warmup << ", \"--kernel\", \"" << opt.kernel
          << "\", \"--fixed-phase\", " << (opt.fixed_phase ? 1 : 0)
          << ", \"--convert-cache\", " << (opt.convert_cache ? 1 : 0)
          << ", \"--input\", \"" << opt.input << "\", \"--seed\"," << opt.seed
          << "]";
        o << "\n}\n";
        f << o.str();
        if (!f) {
            std::cerr << "error: write failed for --json\n";
            return 3;
        }
        std::printf("  json              : %s\n", opt.json.c_str());
    }

    if (!checksum_stable) {
        std::cerr << "error: output checksum differs between repeats\n";
        return 4;
    }
    if (!length_ok) {
        std::cerr << "error: produced " << produced_last << " samples, expected "
                  << expect << "\n";
        return 4;
    }
    return 0;
}

} // namespace

int main(int argc, char** argv)
{
    Options opt;

    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        auto need = [&](const char* what) -> std::string {
            if (i + 1 >= argc) {
                std::cerr << "error: " << a << " needs a value (" << what
                          << ")\n";
                std::exit(2);
            }
            return std::string(argv[++i]);
        };
        auto need_ulong = [&](const char* what) -> unsigned long {
            const std::string v = need(what);
            char* end = nullptr;
            const unsigned long x = std::strtoul(v.c_str(), &end, 0);
            if (end == v.c_str() || (end != nullptr && *end != '\0')) {
                std::cerr << "error: " << a << " expects a number, got '" << v
                          << "'\n";
                std::exit(2);
            }
            return x;
        };

        if (a == "--help" || a == "-h") {
            usage(argv[0]);
            return 0;
        } else if (a == "--taps") {
            opt.taps = need("path");
        } else if (a == "--rate") {
            opt.rate = need("name");
        } else if (a == "--scale-mode") {
            opt.scale_mode = need("unit|raw");
        } else if (a == "--length") {
            opt.length = static_cast<size_t>(need_ulong("N"));
        } else if (a == "--reps") {
            opt.reps = static_cast<size_t>(need_ulong("R"));
        } else if (a == "--warmup") {
            opt.warmup = static_cast<size_t>(need_ulong("W"));
        } else if (a == "--kernel") {
            opt.kernel = need("name");
        } else if (a == "--fixed-phase") {
            const std::string v = need("0|1");
            if (!parse_bool01(v, opt.fixed_phase)) {
                std::cerr << "error: --fixed-phase expects 0 or 1\n";
                return 2;
            }
        } else if (a == "--convert-cache") {
            const std::string v = need("0|1");
            if (!parse_bool01(v, opt.convert_cache)) {
                std::cerr << "error: --convert-cache expects 0 or 1\n";
                return 2;
            }
        } else if (a == "--input") {
            opt.input = need("mode");
            if (opt.input != "random" && opt.input != "rails" &&
                opt.input != "impulse" && opt.input != "zero") {
                std::cerr << "error: --input expects random|rails|impulse|zero\n";
                return 2;
            }
        } else if (a == "--seed") {
            opt.seed = static_cast<uint32_t>(need_ulong("n"));
        } else if (a == "--json") {
            opt.json = need("path");
        } else if (a == "--dump-iq") {
            opt.dump_iq = need("path");
        } else {
            std::cerr << "error: unknown option '" << a << "'\n";
            usage(argv[0]);
            return 2;
        }
    }

    if (opt.taps.empty()) {
        std::cerr << "error: --taps <path> is required\n";
        usage(argv[0]);
        return 2;
    }
    if (opt.reps == 0) {
        std::cerr << "error: --reps must be >= 1\n";
        return 2;
    }
    if (opt.scale_mode != "unit" && opt.scale_mode != "raw") {
        std::cerr << "error: --scale-mode expects unit|raw\n";
        return 2;
    }

    const std::vector<float> taps = load_taps_resolved(opt.taps);
    const uint64_t tap_fnv = fnv1a64(taps.data(), taps.size() * sizeof(float));

    try {
        if (opt.rate == "65_48")
            return benchmark_core<RationalResampler65_48Core>(opt, taps, tap_fnv);
        if (opt.rate == "65_32")
            return benchmark_core<RationalResampler65_32Core>(opt, taps, tap_fnv);
        if (opt.rate == "48_65")
            return benchmark_core<RationalResampler48_65Core>(opt, taps, tap_fnv);
        if (opt.rate == "32_65")
            return benchmark_core<RationalResampler32_65Core>(opt, taps, tap_fnv);
    } catch (const std::exception& e) {
        std::cerr << "error: " << e.what() << "\n";
        return 5;
    }
    std::cerr << "error: --rate expects 65_48|65_32|48_65|32_65\n";
    return 2;
}
