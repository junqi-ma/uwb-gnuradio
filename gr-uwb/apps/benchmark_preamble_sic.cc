/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * S1 layered benchmark for the preamble-only SIC branch
 * (docs/sic/OpenCode修改指示_S1_八符号估计与全前导消除_2026-10-17.md §8/S1-02,
 *  docs/sic/S1_八符号模型API.md; S0: OpenCode开发指示_S0_...§7/§8.4/§9, R03).
 *
 * ============================ WHAT THIS MEASURES ===========================
 * The OFFICIAL rows are measured on the SAME real code9/code10 HRP waveforms,
 * pulse, channel, ISR/SNR and sensing geometry as the S1 effect set, with the
 * SAME effective config / effective_config_hash / template / input length:
 *
 *   - `--model-mode estimate8_frozen` (default) is the S1 path: the estimation
 *     visibility prefix is the PUBLIC plan `search_roi_lo + fit_reps*sps`
 *     (never the injected comm start), fit_reps is fixed at 8,
 *     online_validation_reps is 0, and the frozen model cancels the whole
 *     nominal 64-SYNC preamble + post_tail support. `confirmed_legacy` is the
 *     explicit S0 comparison and can only run with `--nonproduction-control`.
 *   - the corpus is re-derived with the effect-set generator
 *     (gr-uwb/apps/sic_preamble_sim.py) using the `main` case preset's nominal
 *     cell, the S1 fit/validation split (fit=8, validation=8 holdout CAPACITY,
 *     evaluation_reps=56 offline-only), acceptance seeds, and the same
 *     search/subtraction/core/sensing ROIs as `uwb_sim_preamble_sic.py`;
 *   - the corpus records the S1 `effective_config` dict (identical key set and
 *     values to the effect runner) and its canonical SHA-256. At load time the
 *     tool re-serializes that dict canonically, re-hashes it, and REQUIRES the
 *     hash to equal both the corpus record and an effect run's
 *     `effective_config_hash` (default: newest effect run under the output
 *     parent whose `gates.eff_pass`/`gates.s1_pass` are true; `--effect-run`
 *     pins it).
 *     A real-input row that cannot be bound to a passing effect run is written
 *     with `effect_verified=false` and the tool exits non-zero (anti-cheat).
 *   - every input window / template is written as raw CF32 with a SHA-256 in
 *     manifest.json + inputs.txt, and the benchmark RE-VERIFIES those hashes at
 *     load time;
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
 * ==================== S1 TIMING SPLIT (instruction §8) ====================
 * Training and apply are NEVER averaged together. Per window the tool records:
 *
 *   acquisition      detect_coarse + detect_fine   (core stage children)
 *   estimate8        cfo + cir + path_select + fractional_refine +
 *                    matrix_build + gain_fit       (the 8-SYNC parameter fit)
 *   model_prepare    fit-residual reconstruction + model freeze
 *   train_total      estimate_total_us (the parent of the three above)
 *   backfill         frozen apply over [0, observed_end)   (first 8 SYNC)
 *   apply_forward    frozen apply over [observed_end, n)   (the forward loop)
 *   apply            backfill + apply_forward
 *   adapter_wrap     PMT wrap/publish timer from the adapter out meta (gr mode)
 *   consumer         CIR estimator / full demodulator worker latency
 *
 * `apply` throughput is reported in ns/sample, Msample/s and the structural
 * K-path operation count (K complex MAC + CFO rotate + subtract per output
 * sample = 8*K+8 FLOPs), computed on committed windows only so bypasses cannot
 * dilute it. The ~8.14 us observation time of 8x1016 samples is NOT a CPU
 * readiness time and is not reported as one.
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
 *     A FORMAL row is windows >= 10000 and rounds >= 3 on the same config;
 *     anything shorter is labelled a qualification row.
 *   - During timing every window is CONSUMED and its TERMINAL STATE is checked
 *     (committed / bypassed / failed / ambiguous) AND the actual output effect
 *     is verified (support covers the whole injected overlap + downstream
 *     status / FCS / PSDU). A run that silently measures a bypass /
 *     partial-subtraction / early-exit path is recorded as effect_verified
 *     false and exits non-zero: it cannot be quoted as full-processing speed.
 *   - mean / P50 / P95 / P99 / max per stage, CPU seconds, RSS, queue and
 *     in-flight high watermarks, deadline misses, per-collision-class
 *     conditional cost (model_prepare is only paid on committed windows).
 *   - CSV + JSON under --output (a private build/tmp path only); the tool
 *     refuses to write inside a git-tracked source directory.
 *
 * The old pseudo-random / 9-tap-Gaussian synthetic model is kept but RENAMED
 * ("synthetic microbenchmark", --input-source synthetic) and is NEVER labelled
 * as production-profile evidence.  Its numbers must not be used for S1
 * acceptance.
 *
 * Usage:
 *   # once (or to refresh) the real corpus, then per pipeline:
 *   benchmark_preamble_sic --preset main --mode gr --consumer sic \
 *       --model-mode estimate8_frozen --windows 10000 --rounds 3 \
 *       --generate-inputs --output gr-uwb/build-sic/sic-results/s1_perf_gr
 *   benchmark_preamble_sic --preset main --mode gr --consumer cir  ...
 *   benchmark_preamble_sic --preset main --mode gr --consumer demod ...
 *   # explicit S0 comparison (labelled NON-PRODUCTION control):
 *   benchmark_preamble_sic --model-mode confirmed_legacy --cover-preamble \
 *       --nonproduction-control ...
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
#include <cctype>
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
#include <set>
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
    std::string model_mode = "estimate8_frozen"; // estimate8_frozen|confirmed_legacy
    std::string input_source = "real"; // real | synthetic
    std::string inputs;
    std::string output;
    std::string build_dir;
    std::string effect_run;   // effect run to bind the effective config to
    std::string check_against; // effect run whose fixtures are cross-checked
    std::string case_id = "nominal";
    std::string seed_set = "acceptance"; // acceptance | calibration
    bool generate_inputs = false;
    bool nonproduction_control = false; // explicit non-production label
    size_t windows = 10000;
    size_t rounds = 3;
    size_t distinct = 64;
    int fit_reps = 8;
    int validation_reps = 8;  // S1 holdout CAPACITY (effect config: 8)
    int online_validation_reps = 0; // S1 default: no holdout wait
    int evaluation_reps = 56; // offline ([8,64)) only; never fed to training
    int64_t seed_start = 2000000; // acceptance seed set (effect run: 2e6)
    std::string collision_mix = "0,0,64"; // N(no-collision),N(10%),N(100%)
    bool cover_preamble = false;
};

void
usage(const char* argv0)
{
    std::printf(
        "usage: %s --preset main --mode core|gr|native --consumer sic|cir|demod\n"
        "          --model-mode estimate8_frozen|confirmed_legacy\n"
        "          --windows N --rounds R --output DIR\n"
        "          [--input-source real|synthetic] [--inputs DIR]\n"
        "          [--generate-inputs] [--build-dir DIR]\n"
        "          [--effect-run DIR] [--check-against DIR]\n"
        "          [--distinct N] [--case-id NAME]\n"
        "          [--seed-set acceptance|calibration] [--seed-start N]\n"
        "          [--fit-reps N --validation-reps N --online-validation-reps N\n"
        "           --evaluation-reps N] [--collision-mix N0,N10,N100]\n"
        "          [--nonproduction-control] [--cover-preamble]\n"
        "\n"
        "official rows use real effect-set input (--input-source real, the\n"
        "default) with --model-mode estimate8_frozen (8-SYNC frozen model,\n"
        "whole-nominal cancellation) and must bind to a passing effect run; a\n"
        "real row that cannot is written with effect_verified=false and exits\n"
        "non-zero. --model-mode confirmed_legacy / --cover-preamble are the\n"
        "explicit S0 comparison and REQUIRE --nonproduction-control.\n"
        "--input-source synthetic is the OLD synthetic microbenchmark and is\n"
        "NOT production-profile evidence.\n",
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
        else if (a == "--model-mode")
            o.model_mode = need("--model-mode");
        else if (a == "--input-source")
            o.input_source = need("--input-source");
        else if (a == "--inputs")
            o.inputs = need("--inputs");
        else if (a == "--output")
            o.output = need("--output");
        else if (a == "--build-dir")
            o.build_dir = need("--build-dir");
        else if (a == "--effect-run")
            o.effect_run = need("--effect-run");
        else if (a == "--check-against")
            o.check_against = need("--check-against");
        else if (a == "--case-id")
            o.case_id = need("--case-id");
        else if (a == "--seed-set")
            o.seed_set = need("--seed-set");
        else if (a == "--seed-start")
            o.seed_start = std::stoll(need("--seed-start"));
        else if (a == "--generate-inputs")
            o.generate_inputs = true;
        else if (a == "--nonproduction-control")
            o.nonproduction_control = true;
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
        else if (a == "--online-validation-reps")
            o.online_validation_reps = std::stoi(need("--online-validation-reps"));
        else if (a == "--evaluation-reps")
            o.evaluation_reps = std::stoi(need("--evaluation-reps"));
        else if (a == "--collision-mix")
            o.collision_mix = need("--collision-mix");
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
    if (o.model_mode != "estimate8_frozen" &&
        o.model_mode != "confirmed_legacy") {
        std::fprintf(stderr,
                     "error: unknown --model-mode '%s' "
                     "(estimate8_frozen|confirmed_legacy)\n",
                     o.model_mode.c_str());
        return false;
    }
    if (o.input_source != "real" && o.input_source != "synthetic") {
        std::fprintf(stderr,
                     "error: unknown --input-source '%s' (real|synthetic)\n",
                     o.input_source.c_str());
        return false;
    }
    if (o.seed_set != "acceptance" && o.seed_set != "calibration") {
        std::fprintf(stderr,
                     "error: unknown --seed-set '%s' (acceptance|calibration)\n",
                     o.seed_set.c_str());
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
    if (o.cover_preamble) {
        // S1-01: cover-preamble is the OLD support derivation, kept only as an
        // explicitly labelled S0 comparison.
        o.nonproduction_control = true;
        if (o.model_mode != "confirmed_legacy") {
            std::fprintf(stderr,
                         "error: --cover-preamble is the S0 legacy support; it "
                         "requires --model-mode confirmed_legacy (it is no "
                         "longer an implicit switch on the S1 path)\n");
            return false;
        }
        o.validation_reps = static_cast<int>(kSyncReps) - o.fit_reps;
    }
    if (o.model_mode == "confirmed_legacy") {
        if (!o.nonproduction_control) {
            std::fprintf(stderr,
                         "error: --model-mode confirmed_legacy is the explicit "
                         "S0 comparison and cannot be a production row; pass "
                         "--nonproduction-control to label it as such\n");
            return false;
        }
    } else {
        // S1 default path contract.
        if (o.fit_reps != 8) {
            std::fprintf(stderr,
                         "error: estimate8_frozen requires --fit-reps 8 "
                         "(got %d); the S1 model is the strict 8-SYNC estimate\n",
                         o.fit_reps);
            return false;
        }
        if (o.online_validation_reps != 0 && !o.nonproduction_control) {
            std::fprintf(stderr,
                         "error: the estimate8_frozen default is "
                         "online_validation_reps=0 (no holdout wait); the "
                         "explicit conservative 8+%d holdout control needs "
                         "--nonproduction-control\n",
                         o.online_validation_reps);
            return false;
        }
    }
    if (o.fit_reps <= 0 || o.validation_reps < 8 ||
        o.fit_reps + o.validation_reps > static_cast<int>(kSyncReps) ||
        o.online_validation_reps < 0 ||
        o.online_validation_reps > o.validation_reps) {
        std::fprintf(stderr,
                     "error: fit/validation/online reps unsatisfiable: "
                     "%d/%d/%d (needs fit>0, val>=8, fit+val<=64, "
                     "0<=online<=val)\n",
                     o.fit_reps, o.validation_reps, o.online_validation_reps);
        return false;
    }
    if (o.evaluation_reps < 0 ||
        o.fit_reps + o.evaluation_reps > static_cast<int>(kSyncReps)) {
        std::fprintf(stderr,
                     "error: --evaluation-reps %d does not fit [fit,%d) SYNC\n",
                     o.evaluation_reps, o.fit_reps);
        return false;
    }
    if (o.input_source == "real" && !o.nonproduction_control &&
        (o.model_mode != "estimate8_frozen" || o.online_validation_reps != 0 ||
         o.fit_reps != 8 || o.cover_preamble)) {
        std::fprintf(stderr,
                     "error: refusing a real-input row that is not the "
                     "estimate8_frozen effect profile; pass "
                     "--nonproduction-control to label it explicitly\n");
        return false;
    }
    {
        // --collision-mix N0,N10,N100: distinct-window counts per class.
        long long v[3] = { 0, 0, 0 };
        char sep1 = 0, sep2 = 0;
        const int got = std::sscanf(o.collision_mix.c_str(), "%lld,%lld,%lld%c%c",
                                    &v[0], &v[1], &v[2], &sep1, &sep2);
        if (got != 3 || v[0] < 0 || v[1] < 0 || v[2] < 0 ||
            v[0] + v[1] + v[2] != static_cast<long long>(o.distinct)) {
            std::fprintf(stderr,
                         "error: --collision-mix '%s' must be N0,N10,N100 "
                         "non-negative counts summing to --distinct (%zu)\n",
                         o.collision_mix.c_str(), o.distinct);
            return false;
        }
    }
    if (o.output.empty()) {
        std::fprintf(stderr, "error: --output DIR is required\n");
        return false;
    }
    return true;
}

// ---------------------------------------------------------------------------
// The S1 effective config (S1-02): the EXACT identity the effect runner
// (`uwb_sim_preamble_sic.py`) hashes.  Key set and canonical serialization are
// frozen here; the corpus and the bound effect run must both reproduce it.
// ---------------------------------------------------------------------------
struct EffectiveConfig {
    std::string case_id = "nominal";
    long long coarse_stride = 14;
    long long comm_code = 10;
    std::string comm_template_id;
    long long comm_template_length = 0;
    std::string comm_template_sha256;
    long long core_lo = -1, core_hi = -1;
    bool cover_preamble = false;
    double detect_threshold = kDetectThreshold;
    long long evaluation_reps = 56;
    long long fit_reps = 8;
    std::string fit_reps_source = "preset";
    std::string input_grid = "work";
    long long input_length = 0;
    long long k_paths = 4;
    long long online_validation_reps = 0;
    long long post_tail = 100;
    std::string preset = "main";
    double sample_rate = kWorkRate;
    long long samples_per_symbol = 1016;
    long long search_lo = -1, search_hi = -1;
    long long sensing_code = 9;
    long long sensing_lo = -1, sensing_hi = -1;
    std::string sic_model_mode = "estimate8_frozen";
    long long sub_lo = -1, sub_hi = -1;
    long long sync_reps = 64;
    double validate_threshold = kValidateThreshold;
    long long validation_reps = 8;
};

// Python-compatible repr of a double for the values this config uses
// (|v| in [1e-4, 1e16)): shortest fixed-point form that round-trips, with a
// trailing ".0" for integral values (Python prints 998400000.0, not 998400000).
std::string
py_number_repr(double v)
{
    if (!std::isfinite(v))
        return "null";
    if (v == 0.0)
        return std::signbit(v) ? "-0.0" : "0.0";
    const double av = std::abs(v);
    if (av < 1e-4 || av >= 1e16) {
        char buf[64];
        std::snprintf(buf, sizeof(buf), "%.17g", v);
        return std::string(buf);
    }
    for (int dec = 0; dec <= 17; ++dec) {
        char buf[64];
        std::snprintf(buf, sizeof(buf), "%.*f", dec, v);
        if (std::strtod(buf, nullptr) == v) {
            std::string s(buf);
            if (s.find('.') == std::string::npos)
                s += ".0";
            return s;
        }
    }
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%.17g", v);
    return std::string(buf);
}

std::string
json_str(const std::string& s)
{
    std::string o = "\"";
    for (char c : s) {
        switch (c) {
        case '"': o += "\\\""; break;
        case '\\': o += "\\\\"; break;
        case '\n': o += "\\n"; break;
        case '\r': o += "\\r"; break;
        case '\t': o += "\\t"; break;
        default:
            if (static_cast<unsigned char>(c) < 0x20) {
                char b[8];
                std::snprintf(b, sizeof(b), "\\u%04x",
                              static_cast<unsigned>(c) & 0xFFu);
                o += b;
            } else {
                o.push_back(c);
            }
        }
    }
    o.push_back('"');
    return o;
}

// Canonical per-key JSON values of the effective config: keys sorted, values in
// Python repr form. `json.dumps(obj, sort_keys=True, separators=(",", ":"))`
// produces the concatenation of these, so sha256(canonical) is the effect
// runner's `effective_config_hash`.
std::map<std::string, std::string>
effective_config_fields(const EffectiveConfig& e)
{
    std::map<std::string, std::string> m;
    auto b = [](bool v) { return std::string(v ? "true" : "false"); };
    auto i = [](long long v) { return std::to_string(v); };
    auto pair = [&](long long lo, long long hi) {
        return "[" + std::to_string(lo) + "," + std::to_string(hi) + "]";
    };
    m["case_id"] = json_str(e.case_id);
    m["coarse_stride"] = i(e.coarse_stride);
    m["comm_code"] = i(e.comm_code);
    m["comm_template_id"] = json_str(e.comm_template_id);
    m["comm_template_length"] = i(e.comm_template_length);
    m["comm_template_sha256"] = json_str(e.comm_template_sha256);
    m["core_roi"] = pair(e.core_lo, e.core_hi);
    m["cover_preamble"] = b(e.cover_preamble);
    m["detect_threshold"] = py_number_repr(e.detect_threshold);
    m["evaluation_reps"] = i(e.evaluation_reps);
    m["fit_reps"] = i(e.fit_reps);
    m["fit_reps_source"] = json_str(e.fit_reps_source);
    m["input_grid"] = json_str(e.input_grid);
    m["input_length"] = i(e.input_length);
    m["k_paths"] = i(e.k_paths);
    m["online_validation_reps"] = i(e.online_validation_reps);
    m["post_tail"] = i(e.post_tail);
    m["preset"] = json_str(e.preset);
    m["sample_rate"] = py_number_repr(e.sample_rate);
    m["samples_per_symbol"] = i(e.samples_per_symbol);
    m["search_roi"] = pair(e.search_lo, e.search_hi);
    m["sensing_code"] = i(e.sensing_code);
    m["sensing_roi"] = pair(e.sensing_lo, e.sensing_hi);
    m["sic_model_mode"] = json_str(e.sic_model_mode);
    m["subtraction_roi"] = pair(e.sub_lo, e.sub_hi);
    m["sync_reps"] = i(e.sync_reps);
    m["validate_threshold"] = py_number_repr(e.validate_threshold);
    m["validation_reps"] = i(e.validation_reps);
    return m;
}

std::string
effective_config_json(const EffectiveConfig& e)
{
    const auto fields = effective_config_fields(e);
    std::ostringstream j;
    j << "{";
    bool first = true;
    for (const auto& kv : fields) {
        j << (first ? "" : ",") << json_str(kv.first) << ":" << kv.second;
        first = false;
    }
    j << "}";
    return j.str();
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
    // S1 timing split (adapter out-meta keys; 0 when the layer does not
    // produce them, e.g. core mode fills them from SicTimings instead).
    std::string model_mode;
    uint64_t model_id = 0;
    int64_t observed_end = 0;
    std::string support_source;
    uint64_t detect_coarse_us = 0, detect_fine_us = 0;
    uint64_t cfo_us = 0, cir_us = 0, path_select_us = 0;
    uint64_t fractional_refine_us = 0, matrix_build_us = 0, gain_fit_us = 0;
    uint64_t estimate_us = 0, model_prepare_us = 0;
    uint64_t apply_us = 0, backfill_us = 0, apply_forward_us = 0;
    uint64_t adapter_wrap_us = 0;
    bool has_timings = false;
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
            s.model_mode = dict_str(meta, "sic_model_mode");
            s.model_id = dict_u64(meta, "sic_model_id", 0);
            s.observed_end = dict_i64(meta, "sic_observed_end_sample", 0);
            s.support_source = dict_str(meta, "sic_support_source");
            s.detect_coarse_us = dict_u64(meta, "sic_detect_coarse_us", 0);
            s.detect_fine_us = dict_u64(meta, "sic_detect_fine_us", 0);
            s.cfo_us = dict_u64(meta, "sic_cfo_us", 0);
            s.cir_us = dict_u64(meta, "sic_cir_us", 0);
            s.path_select_us = dict_u64(meta, "sic_path_select_us", 0);
            s.fractional_refine_us =
                dict_u64(meta, "sic_fractional_refine_us", 0);
            s.matrix_build_us = dict_u64(meta, "sic_matrix_build_us", 0);
            s.gain_fit_us = dict_u64(meta, "sic_gain_fit_us", 0);
            s.estimate_us = dict_u64(meta, "sic_estimate_us", 0);
            s.model_prepare_us = dict_u64(meta, "sic_model_prepare_us", 0);
            s.apply_us = dict_u64(meta, "sic_apply_us", 0);
            s.backfill_us = dict_u64(meta, "sic_backfill_us", 0);
            s.apply_forward_us = dict_u64(meta, "sic_apply_forward_us", 0);
            s.adapter_wrap_us = dict_u64(meta, "output_wrap_publish_us", 0);
            s.has_timings = pmt::dict_has_key(meta, pmt::mp("sic_estimate_us"));
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
    bool comm_present = true;
    std::string collision_class = "collision_100"; // derived from realized truth
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
    int fit_reps = 8, validation_reps = 8, k_paths = 4;
    int online_validation_reps = 0, evaluation_reps = 56;
    double rate = kWorkRate, snr_db = 30.0, isr_db = 10.0;
    std::string sfd_mode = kSfdMode;
    std::string psdu_hex = kCanonicalPsdu;
    std::string pulse_shape = "legacy";
    int pulse_taps = 48;
    std::string frac_model = "pchip";
    std::string comm_template_mode;
    bool comm_template_r07_pass = false;
    std::string model_mode = "estimate8_frozen";
    std::string seed_set = "acceptance";
    long long seed_start = 0;
    int64_t pre = 1997, n = 0;
    int64_t search_lo = 0, search_hi = 0, sub_lo = 0, sub_hi = 0;
    int64_t sensing_lo = 0, sensing_hi = 0;
    int64_t observed_end = 0; // S1 estimation visibility (public plan)
    double detect_threshold = kDetectThreshold;
    double validate_threshold = kValidateThreshold;
    size_t coarse_stride = kCoarseStride, post_tail = kPostTail;

    // S1 effective-config identity (see EffectiveConfig).
    EffectiveConfig ec;
    std::string effective_config_hash;     // recorded by the generator
    std::string effective_config_hash_recomputed;
    std::string effective_config_json;     // canonical bytes

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

// ---------------------------------------------------------------------------
// Minimal JSON reader + canonical writer.
//
// Used to read an effect run's `run_manifest.json` (effective_config + its
// recorded hash) and `summary.json` (gates).  A tiny self-contained parser is
// used instead of a dependency because the benchmark must not silently bind to
// a config it cannot actually parse; a parse failure is a hard failure.
// Numbers keep their raw token, so canonical output reproduces the Python
// `json.dumps` bytes the effect runner hashed.
// ---------------------------------------------------------------------------
struct JsonValue {
    enum class Kind { Null, Bool, Number, String, Array, Object };
    Kind kind = Kind::Null;
    bool b = false;
    std::string text; // string content or raw number token
    std::vector<JsonValue> arr;
    std::map<std::string, JsonValue> obj;

    const JsonValue* find(const std::string& k) const
    {
        auto it = obj.find(k);
        return it == obj.end() ? nullptr : &it->second;
    }
    bool bool_or(bool def) const
    {
        return (kind == Kind::Bool) ? b : def;
    }
    long long int_or(long long def) const
    {
        if (kind != Kind::Number)
            return def;
        try {
            return std::stoll(text);
        } catch (...) {
            return def;
        }
    }
    std::string str_or(const std::string& def) const
    {
        return (kind == Kind::String) ? text : def;
    }
};

class JsonReader
{
public:
    explicit JsonReader(const std::string& s) : s_(s) {}

    bool parse(JsonValue& out)
    {
        skip();
        if (!value(out))
            return false;
        skip();
        return i_ == s_.size();
    }

private:
    void skip()
    {
        while (i_ < s_.size()) {
            const char c = s_[i_];
            if (c == ' ' || c == '\t' || c == '\n' || c == '\r')
                ++i_;
            else
                break;
        }
    }
    bool literal(const char* lit)
    {
        const size_t n = std::strlen(lit);
        if (s_.compare(i_, n, lit) != 0)
            return false;
        i_ += n;
        return true;
    }
    bool value(JsonValue& v)
    {
        skip();
        if (i_ >= s_.size())
            return false;
        const char c = s_[i_];
        if (c == '{')
            return object(v);
        if (c == '[')
            return array(v);
        if (c == '"') {
            v.kind = JsonValue::Kind::String;
            return string(v.text);
        }
        if (c == 't') {
            if (!literal("true"))
                return false;
            v.kind = JsonValue::Kind::Bool;
            v.b = true;
            return true;
        }
        if (c == 'f') {
            if (!literal("false"))
                return false;
            v.kind = JsonValue::Kind::Bool;
            v.b = false;
            return true;
        }
        if (c == 'n') {
            if (!literal("null"))
                return false;
            v.kind = JsonValue::Kind::Null;
            return true;
        }
        const size_t start = i_;
        if (c == '-' || c == '+')
            ++i_;
        while (i_ < s_.size() &&
               (std::isdigit(static_cast<unsigned char>(s_[i_])) ||
                s_[i_] == '.' || s_[i_] == 'e' || s_[i_] == 'E' ||
                s_[i_] == '+' || s_[i_] == '-'))
            ++i_;
        if (i_ == start)
            return false;
        v.kind = JsonValue::Kind::Number;
        v.text = s_.substr(start, i_ - start);
        return true;
    }
    bool string(std::string& out)
    {
        if (i_ >= s_.size() || s_[i_] != '"')
            return false;
        ++i_;
        out.clear();
        while (i_ < s_.size()) {
            const char c = s_[i_++];
            if (c == '"')
                return true;
            if (c != '\\') {
                out.push_back(c);
                continue;
            }
            if (i_ >= s_.size())
                return false;
            const char e = s_[i_++];
            switch (e) {
            case '"': out.push_back('"'); break;
            case '\\': out.push_back('\\'); break;
            case '/': out.push_back('/'); break;
            case 'b': out.push_back('\b'); break;
            case 'f': out.push_back('\f'); break;
            case 'n': out.push_back('\n'); break;
            case 'r': out.push_back('\r'); break;
            case 't': out.push_back('\t'); break;
            case 'u': {
                if (i_ + 4 > s_.size())
                    return false;
                unsigned cp = 0;
                for (int k = 0; k < 4; ++k) {
                    const char h = s_[i_++];
                    cp <<= 4;
                    if (h >= '0' && h <= '9')
                        cp |= static_cast<unsigned>(h - '0');
                    else if (h >= 'a' && h <= 'f')
                        cp |= static_cast<unsigned>(h - 'a' + 10);
                    else if (h >= 'A' && h <= 'F')
                        cp |= static_cast<unsigned>(h - 'A' + 10);
                    else
                        return false;
                }
                // ASCII-only config strings.
                out.push_back(static_cast<char>(cp & 0x7F));
                break;
            }
            default: return false;
            }
        }
        return false;
    }
    bool object(JsonValue& v)
    {
        v.kind = JsonValue::Kind::Object;
        v.obj.clear();
        ++i_; // {
        skip();
        if (i_ < s_.size() && s_[i_] == '}') {
            ++i_;
            return true;
        }
        while (true) {
            skip();
            std::string key;
            if (!string(key))
                return false;
            skip();
            if (i_ >= s_.size() || s_[i_] != ':')
                return false;
            ++i_;
            JsonValue val;
            if (!value(val))
                return false;
            v.obj[key] = std::move(val);
            skip();
            if (i_ >= s_.size())
                return false;
            if (s_[i_] == ',') {
                ++i_;
                continue;
            }
            if (s_[i_] == '}') {
                ++i_;
                return true;
            }
            return false;
        }
    }
    bool array(JsonValue& v)
    {
        v.kind = JsonValue::Kind::Array;
        v.arr.clear();
        ++i_; // [
        skip();
        if (i_ < s_.size() && s_[i_] == ']') {
            ++i_;
            return true;
        }
        while (true) {
            JsonValue val;
            if (!value(val))
                return false;
            v.arr.push_back(std::move(val));
            skip();
            if (i_ >= s_.size())
                return false;
            if (s_[i_] == ',') {
                ++i_;
                continue;
            }
            if (s_[i_] == ']') {
                ++i_;
                return true;
            }
            return false;
        }
    }

    const std::string& s_;
    size_t i_ = 0;
};

// Compact canonical JSON: sorted keys, no whitespace, raw number tokens.
std::string
json_canonical(const JsonValue& v)
{
    std::ostringstream o;
    switch (v.kind) {
    case JsonValue::Kind::Null: o << "null"; break;
    case JsonValue::Kind::Bool: o << (v.b ? "true" : "false"); break;
    case JsonValue::Kind::Number: o << v.text; break;
    case JsonValue::Kind::String: o << json_str(v.text); break;
    case JsonValue::Kind::Array: {
        o << "[";
        for (size_t k = 0; k < v.arr.size(); ++k)
            o << (k ? "," : "") << json_canonical(v.arr[k]);
        o << "]";
        break;
    }
    case JsonValue::Kind::Object: {
        o << "{";
        bool first = true;
        for (const auto& kv : v.obj) {
            o << (first ? "" : ",") << json_str(kv.first) << ":"
              << json_canonical(kv.second);
            first = false;
        }
        o << "}";
        break;
    }
    }
    return o.str();
}

bool
read_json_file(const std::string& path, JsonValue& out, std::string& err)
{
    std::ifstream f(path, std::ios::binary);
    if (!f) {
        err = "cannot open " + path;
        return false;
    }
    std::ostringstream ss;
    ss << f.rdbuf();
    const std::string text = ss.str(); // must outlive the reader
    JsonReader r(text);
    if (!r.parse(out)) {
        err = "JSON parse failed: " + path;
        return false;
    }
    return true;
}

// An effect run the measured row may bind to.  `ok` means: the manifest's
// effective_config reproduces its own recorded hash, the hash equals the
// benchmark's canonical hash, the effect gates pass, and both agree on the
// template sha256 / input length / model mode.
struct EffectRun {
    bool found = false;
    std::string dir, run_id;
    std::string model_mode;
    std::string effective_config_hash;            // recorded by the effect run
    std::string effective_config_hash_recomputed; // from its own dict
    std::string config_json;                      // canonical
    JsonValue config;                             // parsed dict
    bool hash_ok = false;
    bool eff_pass = false;
    bool s1_pass = false;
    long long windows_completed = 0;
    bool config_match = false;
    std::vector<std::string> mismatch;
    std::string error;

    bool basic_ok() const
    {
        return found && hash_ok && eff_pass && s1_pass && error.empty();
    }
    bool ok() const
    {
        return basic_ok() && config_match && mismatch.empty();
    }
};

// Load one effect run directory (must contain run_manifest.json + summary.json).
bool
load_effect_run(const std::string& dir, EffectRun& e)
{
    e = EffectRun();
    e.dir = dir;
    const std::string man_path = dir + "/run_manifest.json";
    const std::string sum_path = dir + "/summary.json";
    JsonValue man, sum;
    std::string err;
    if (!read_json_file(man_path, man, err)) {
        e.error = err;
        return false;
    }
    if (!read_json_file(sum_path, sum, err)) {
        e.error = err;
        return false;
    }
    e.found = true;
    e.run_id = man.find("run_id") ? man.find("run_id")->str_or("") : "";
    e.model_mode =
        man.find("sic_model_mode") ? man.find("sic_model_mode")->str_or("") : "";
    const JsonValue* ec = man.find("effective_config");
    if (ec == nullptr || ec->kind != JsonValue::Kind::Object) {
        e.error = "manifest has no effective_config object";
        return false;
    }
    if (e.model_mode.empty() && ec->find("sic_model_mode"))
        e.model_mode = ec->find("sic_model_mode")->str_or("");
    e.config_json = json_canonical(*ec);
    e.config = *ec;
    e.effective_config_hash = man.find("effective_config_hash")
                                  ? man.find("effective_config_hash")->str_or("")
                                  : "";
    e.effective_config_hash_recomputed = sha256_text(e.config_json);
    e.hash_ok = !e.effective_config_hash.empty() &&
                e.effective_config_hash == e.effective_config_hash_recomputed;
    const JsonValue* gates = sum.find("gates");
    if (gates != nullptr) {
        if (const JsonValue* v = gates->find("eff_pass"))
            e.eff_pass = v->bool_or(false);
        if (const JsonValue* v = gates->find("s1_pass"))
            e.s1_pass = v->bool_or(false);
    }
    if (const JsonValue* v = sum.find("windows_completed"))
        e.windows_completed = v->int_or(0);
    return true;
}

// Compare the benchmark's own effective config against the effect run's dict:
// per-field canonical rendering, so a mismatch names the exact keys.
void
compare_effective_config(const EffectiveConfig& mine, const JsonValue& theirs,
                         std::vector<std::string>& mismatch)
{
    const auto f = effective_config_fields(mine);
    for (const auto& kv : f) {
        const JsonValue* v = theirs.find(kv.first);
        if (v == nullptr) {
            mismatch.push_back(kv.first + ": missing in effect run");
            continue;
        }
        const std::string a = json_canonical(*v);
        if (a != kv.second)
            mismatch.push_back(kv.first + ": benchmark=" + kv.second +
                               " effect=" + a);
    }
    for (const auto& kv : theirs.obj) {
        if (f.find(kv.first) == f.end())
            mismatch.push_back(kv.first + ": extra key in effect run");
    }
}

// Candidate effect-run directories under `parent` (run_manifest.json +
// summary.json present), newest first.
std::vector<std::string>
list_effect_run_dirs(const std::string& parent)
{
    namespace fs = std::filesystem;
    std::vector<std::string> out;
    std::error_code ec;
    if (!fs::is_directory(parent, ec))
        return out;
    struct Cand {
        std::string dir;
        fs::file_time_type t;
    };
    std::vector<Cand> cands;
    for (const auto& ent : fs::directory_iterator(parent, ec)) {
        if (!ent.is_directory())
            continue;
        const std::string d = ent.path().string();
        if (!fs::exists(d + "/run_manifest.json") ||
            !fs::exists(d + "/summary.json"))
            continue;
        cands.push_back({ d, fs::last_write_time(ent.path(), ec) });
    }
    std::sort(cands.begin(), cands.end(),
              [](const Cand& a, const Cand& b) { return a.t > b.t; });
    for (const Cand& c : cands)
        out.push_back(c.dir);
    return out;
}

// Directory of the newest effect run that may bind this model mode.  Prefers an
// explicit --effect-run; otherwise scans the sibling directories of `parent`.
std::string
find_effect_run_dir(const std::string& pinned, const std::string& parent,
                    const std::string& model_mode, std::vector<std::string>& rejected)
{
    namespace fs = std::filesystem;
    if (!pinned.empty())
        return pinned;
    for (const std::string& dir : list_effect_run_dirs(parent)) {
        EffectRun e;
        if (!load_effect_run(dir, e))
            continue;
        if (!e.basic_ok()) {
            rejected.push_back(fs::path(dir).filename().string() + "(gates=" +
                               std::string(e.eff_pass ? "eff" : "-") +
                               std::string(e.s1_pass ? "/s1" : "/-") +
                               (e.hash_ok ? "" : ",hash") + ")");
            continue;
        }
        if (!e.model_mode.empty() && model_mode != e.model_mode) {
            rejected.push_back(fs::path(dir).filename().string() +
                               "(model_mode=" + e.model_mode + ")");
            continue;
        }
        return dir;
    }
    return std::string();
}

// Exact-config search used after the corpus is loaded: among the passing effect
// runs, bind to the newest one whose canonical effective_config equals ours.
bool
find_effect_run_for_config(const std::string& parent,
                           const std::string& want_json, EffectRun& out)
{
    for (const std::string& dir : list_effect_run_dirs(parent)) {
        EffectRun e;
        if (!load_effect_run(dir, e))
            continue;
        if (!e.basic_ok())
            continue;
        if (e.config_json != want_json)
            continue;
        out = e;
        return true;
    }
    return false;
}

// The S1 real-corpus generator, written next to its output so the exact
// generator text is archived and hashed.  It re-derives the code9/code10 HRP
// waveforms of the effect set (same module, same nominal cell, acceptance
// seeds) and records the shared effective config.
std::string
generator_script_text()
{
    return R"PY(#!/usr/bin/env python3
"""S1 real-input corpus generator for benchmark_preamble_sic (work grid).

Re-derives the SAME code9/code10 HRP waveforms, pulse, channel, ISR/SNR and
window geometry as the S1 effect set (gr-uwb/apps/sic_preamble_sim.py, `main`
preset nominal cell) and writes raw CF32 windows plus a hash manifest.  It also
records the S1 `effective_config` dict (identical key set/values to
`uwb_sim_preamble_sic.py`) and its canonical SHA-256, so the benchmark can bind
its measured row to the effect run instead of merely claiming the same names.

It is the OFFLINE generator side: the online SIC never reads anything from here.
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


def canonical_hash(obj):
    blob = json.dumps(obj, sort_keys=True, separators=(",", ":")).encode()
    return hashlib.sha256(blob).hexdigest()


def main():
    (repo, build_dir, out, count, fit, val, post_tail, check_dir, model_mode,
     online_val, eval_reps, seed_set, seed_start, mix0, mix10, mix100,
     case_id) = sys.argv[1:18]
    count, fit, val, post_tail = int(count), int(fit), int(val), int(post_tail)
    online_val, eval_reps = int(online_val), int(eval_reps)
    seed_start = int(seed_start)
    mix = [int(mix0), int(mix10), int(mix100)]
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
    # No-interferer generator for the 0% collision class.  Same sensing packet,
    # pulse and SNR, no comm signal and no ISR normalisation.
    gen_clean = None
    if mix[0] > 0:
        gen_clean = S.SicSignalGenerator(
            uwb, sensing_code=sensing_code, comm_code=comm_code,
            sync_reps=sync_reps, sfd_mode=S.SFD_MODE,
            psdu_hex=S.DEFAULT_PSDU_HEX, comm_present=False,
            build_comm=False, snr_db=snr_db, isr_db=None, verbose=False)

    search_roi = (max(0, gen.pre), min(gen.n, gen.pre + sync_reps * S.SPS))
    sensing_roi = (search_roi[0], search_roi[1])
    # Same widening rule as uwb_sim_preamble_sic.py: the core ROI covers the
    # whole nominal preamble so the natural tail is never clipped.
    core_tail_reps = max(int(fit) + int(val), int(sync_reps))
    sub_roi = (search_roi[0],
               min(gen.n, search_roi[1] + core_tail_reps * S.SPS + post_tail))
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
    comm_mode = gen.comm_meta.get("mode") or "unknown"
    comm_sha = tpl_meta["comm_preamble_code10"]["sha256"]
    comm_len = tpl_meta["comm_preamble_code10"]["length"]

    # ---- windows -----------------------------------------------------------
    # class plan: 0% (clean, no comm), 10% (comm starts 90% into the sensing
    # SYNC span, so only ~10% of the comm preamble overlaps it) and 100%
    # (nominal start == ROI start).  10% is a strict-prefix OUT-OF-PREFIX case:
    # the comm starts after the public 8-SYNC visibility prefix, so the correct
    # terminal state is a bypass, not a cancellation.
    plan = (["no_collision"] * mix[0] + ["collision_10"] * mix[1] +
            ["collision_100"] * mix[2])
    if len(plan) != count:
        raise SystemExit("collision plan %r does not match count %d"
                         % (mix, count))
    late_off = 0.9 * sync_reps * S.SPS

    wins = []
    for i in range(count):
        cls = plan[i]
        seed = seed_start + i
        if cls == "no_collision":
            w = gen_clean.build(seed=seed, comm_paths=comm_paths,
                                start_offset_work=start_off, cfo_hz=cfo_hz,
                                isr_db=None, snr_db=snr_db)
            comm_present = False
        elif cls == "collision_10":
            w = gen.build(seed=seed, comm_paths=comm_paths,
                          start_offset_work=start_off + late_off,
                          cfo_hz=cfo_hz, isr_db=isr_db, snr_db=snr_db)
            comm_present = True
        else:
            w = gen.build(seed=seed, comm_paths=comm_paths,
                          start_offset_work=start_off, cfo_hz=cfo_hz,
                          isr_db=isr_db, snr_db=snr_db)
            comm_present = True
        y = np.ascontiguousarray(w["y"], dtype=np.complex64)
        fn = "win_%04d.cf32" % i
        y.tofile(os.path.join(out, fn))
        wins.append({"index": i, "seed": int(seed), "file": fn,
                     "length": int(y.size), "sha256": sha256_arr(y),
                     "overlap_lo": int(w["overlap"]["lo"]),
                     "overlap_hi": int(w["overlap"]["hi"]),
                     "collision_class": cls,
                     "comm_present": bool(comm_present),
                     "isr_db": (None if w["isr_db"] is None
                                else float(w["isr_db"])),
                     "noise_std": float(w["noise_std"])})

    # ---- S1 effective config (identical key set / values to the effect run) --
    effective_config = {
        "case_id": str(case_id),
        "preset": "main", "input_grid": "work",
        "input_length": int(gen.n),
        "sample_rate": float(S.WORK_HZ), "samples_per_symbol": int(S.SPS),
        "sensing_code": sensing_code, "comm_code": comm_code,
        "sync_reps": sync_reps,
        "k_paths": int(preset.get("runtime", {}).get("sic_k", 4)),
        "fit_reps": int(fit),
        "validation_reps": int(val),
        "fit_reps_source": "preset",
        "cover_preamble": False,
        "sic_model_mode": str(model_mode),
        "online_validation_reps": int(online_val),
        "evaluation_reps": int(eval_reps),
        "search_roi": [int(search_roi[0]), int(search_roi[1])],
        "subtraction_roi": [int(sub_roi[0]), int(sub_roi[1])],
        "core_roi": [int(core_roi[0]), int(core_roi[1])],
        "sensing_roi": [int(sensing_roi[0]), int(sensing_roi[1])],
        "coarse_stride": 14,
        "post_tail": int(post_tail),
        "detect_threshold": 0.40,
        "validate_threshold": 0.25,
        "comm_template_id": "comm_preamble_code%d_%s" % (comm_code, comm_mode),
        "comm_template_sha256": comm_sha,
        "comm_template_length": int(comm_len),
    }
    effective_config_hash = canonical_hash(effective_config)

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
        "preset": "main", "input_grid": "work", "case_id": str(case_id),
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
        "online_validation_reps": online_val, "evaluation_reps": eval_reps,
        "sic_model_mode": str(model_mode),
        "k_paths": int(preset.get("runtime", {}).get("sic_k", 4)),
        "post_tail": post_tail, "coarse_stride": 14,
        "detect_threshold": 0.40, "validate_threshold": 0.25,
        "pre_guard": int(gen.pre), "window_len": int(gen.n),
        "geometry": {k: int(v) for k, v in gen.geom.items()},
        "search_roi": [int(search_roi[0]), int(search_roi[1])],
        "subtraction_roi": [int(sub_roi[0]), int(sub_roi[1])],
        "core_roi": [int(core_roi[0]), int(core_roi[1])],
        "sensing_roi": [int(sensing_roi[0]), int(sensing_roi[1])],
        "samples_per_symbol": int(S.SPS), "sample_rate": float(S.WORK_HZ),
        "seed_set": str(seed_set), "seed_start": seed_start,
        "collision_mix": mix, "windows": count,
        "effective_config_hash": effective_config_hash,
    }
    config_hash = canonical_hash(cfg)

    manifest = {
        "schema": "sic_perf_real_inputs_s2",
        "generated_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "generator": {
            "module": sim_path,
            "module_sha256": sha256_file(sim_path),
            "script": os.path.abspath(__file__),
            "script_sha256": sha256_file(os.path.abspath(__file__)),
        },
        "resolved_config": cfg,
        "config_hash": config_hash,
        "effective_config": effective_config,
        "effective_config_hash": effective_config_hash,
        "templates": tpl_meta,
        "comm_template_mode": comm_mode,
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

    add("schema", "sic_perf_real_inputs_s2")
    add("generator_module", sim_path)
    add("generator_module_sha256", manifest["generator"]["module_sha256"])
    add("generator_script_sha256", manifest["generator"]["script_sha256"])
    add("config_hash", config_hash)
    add("effective_config_hash", effective_config_hash)
    for k in sorted(effective_config):
        v = effective_config[k]
        add("effective_config." + k, json.dumps(v, separators=(",", ":")))
    for k in sorted(cfg):
        v = cfg[k]
        if isinstance(v, (list, dict)):
            v = json.dumps(v, separators=(",", ":"))
        add("cfg." + k, v)
    for name in tpl_meta:
        add("template.%s.file" % name, tpl_meta[name]["file"])
        add("template.%s.length" % name, tpl_meta[name]["length"])
        add("template.%s.sha256" % name, tpl_meta[name]["sha256"])
    add("comm_template_mode", comm_mode)
    add("comm_template_r07_pass", str(r07_pass).lower())
    add("crosscheck.reference_run", cross.get("reference_run"))
    add("crosscheck.templates_match",
        str(cross.get("templates_match", False)).lower())
    add("crosscheck.fixture_windows", cross.get("fixture_windows_compared", 0))
    add("crosscheck.fixture_y_match",
        str(cross.get("fixture_y_match", False)).lower())
    add("windows", count)
    for w in wins:
        L.append("window %d %d %d %s %d %d %s %s %s %s %s" % (
            w["index"], w["seed"], w["length"], w["sha256"],
            w["overlap_lo"], w["overlap_hi"],
            ("nan" if w["isr_db"] is None else repr(w["isr_db"])),
            repr(w["noise_std"]), w["file"], w["collision_class"],
            ("1" if w["comm_present"] else "0")))
    with open(os.path.join(out, "inputs.txt"), "w") as f:
        f.write("\n".join(L) + "\n")
    print("real corpus: %d windows of %d samples; effective_config_hash=%s; "
          "config_hash=%s; templates_match=%s fixture_y_match=%s"
          % (count, int(gen.n), effective_config_hash, config_hash,
             cross.get("templates_match"), cross.get("fixture_y_match")),
          flush=True)


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
    std::string m0 = "0", m1 = "0", m2 = "0";
    {
        long long a = 0, b = 0, c = 0;
        if (std::sscanf(o.collision_mix.c_str(), "%lld,%lld,%lld", &a, &b, &c) ==
            3) {
            m0 = std::to_string(a);
            m1 = std::to_string(b);
            m2 = std::to_string(c);
        }
    }
    const std::string cmd = "python3 '" + script + "' '" + repo + "' '" +
                            build_dir + "' '" + inputs + "' " +
                            std::to_string(o.distinct) + " " +
                            std::to_string(o.fit_reps) + " " +
                            std::to_string(o.validation_reps) + " " +
                            std::to_string(kPostTail) + " '" + o.check_against +
                            "' " + o.model_mode + " " +
                            std::to_string(o.online_validation_reps) + " " +
                            std::to_string(o.evaluation_reps) + " " +
                            o.seed_set + " " + std::to_string(o.seed_start) +
                            " " + m0 + " " + m1 + " " + m2 + " " + o.case_id;
    std::printf("generating real corpus:\n  %s\n", cmd.c_str());
    std::fflush(stdout);
    const int rc = std::system(cmd.c_str());
    if (rc != 0)
        throw std::runtime_error("real-corpus generator failed (rc=" +
                                 std::to_string(rc) + "): " + cmd +
                                 " (a 10% collision class needs "
                                 "--distinct >= N0+N10+N100)");
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
    if (c.schema != "sic_perf_real_inputs_s2")
        throw std::runtime_error(
            "unexpected corpus schema '" + c.schema +
            "' (S1 requires sic_perf_real_inputs_s2: regenerate with "
            "--generate-inputs)");
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
    c.online_validation_reps =
        static_cast<int>(kv_i("cfg.online_validation_reps"));
    c.evaluation_reps = static_cast<int>(kv_i("cfg.evaluation_reps"));
    c.k_paths = static_cast<int>(kv_i("cfg.k_paths"));
    c.rate = kv_d("cfg.sample_rate");
    c.snr_db = kv_d("cfg.snr_db");
    c.isr_db = kv_d("cfg.isr_db");
    c.sfd_mode = kv_get(kv, "cfg.sfd_mode", kSfdMode);
    c.psdu_hex = kv_get(kv, "cfg.psdu_hex", kCanonicalPsdu);
    c.pulse_shape = kv_get(kv, "cfg.pulse_shape", "legacy");
    c.pulse_taps = static_cast<int>(kv_i("cfg.pulse_taps"));
    c.frac_model = kv_get(kv, "cfg.frac_model", "pchip");
    c.model_mode = kv_get(kv, "cfg.sic_model_mode", "estimate8_frozen");
    c.seed_set = kv_get(kv, "cfg.seed_set", "acceptance");
    c.seed_start = kv_i("cfg.seed_start");
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
        roi("cfg.sensing_roi", c.sensing_lo, c.sensing_hi);
    }
    c.comm_template_mode = kv_get(kv, "comm_template_mode");
    c.comm_template_r07_pass = kv_get(kv, "comm_template_r07_pass") == "true";

    // ---- S1 effective config: recorded dict + verified canonical hash ------
    auto ecv = [&](const char* k) -> JsonValue {
        const std::string raw =
            kv_get(kv, std::string("effective_config.") + k, "");
        JsonValue v;
        std::string err;
        JsonReader r(raw);
        if (raw.empty() || !r.parse(v))
            throw std::runtime_error(
                std::string("corpus is missing/bad effective_config.") + k +
                " (regenerate with --generate-inputs)");
        return v;
    };
    auto ec_ll = [&](const char* k) { return ecv(k).int_or(0); };
    auto ec_d = [&](const char* k) {
        const JsonValue v = ecv(k);
        return (v.kind == JsonValue::Kind::Number) ? std::stod(v.text) : 0.0;
    };
    auto ec_s = [&](const char* k) { return ecv(k).str_or(""); };
    auto ec_roi = [&](const char* k, long long& lo, long long& hi) {
        const JsonValue v = ecv(k);
        if (v.kind != JsonValue::Kind::Array || v.arr.size() != 2)
            throw std::runtime_error(std::string("bad effective_config.") + k);
        lo = v.arr[0].int_or(-1);
        hi = v.arr[1].int_or(-1);
    };
    EffectiveConfig& ec = c.ec;
    ec.case_id = ec_s("case_id");
    ec.coarse_stride = ec_ll("coarse_stride");
    ec.comm_code = ec_ll("comm_code");
    ec.comm_template_id = ec_s("comm_template_id");
    ec.comm_template_length = ec_ll("comm_template_length");
    ec.comm_template_sha256 = ec_s("comm_template_sha256");
    ec_roi("core_roi", ec.core_lo, ec.core_hi);
    ec.cover_preamble = ecv("cover_preamble").bool_or(false);
    ec.detect_threshold = ec_d("detect_threshold");
    ec.evaluation_reps = ec_ll("evaluation_reps");
    ec.fit_reps = ec_ll("fit_reps");
    ec.fit_reps_source = ec_s("fit_reps_source");
    ec.input_grid = ec_s("input_grid");
    ec.input_length = ec_ll("input_length");
    ec.k_paths = ec_ll("k_paths");
    ec.online_validation_reps = ec_ll("online_validation_reps");
    ec.post_tail = ec_ll("post_tail");
    ec.preset = ec_s("preset");
    ec.sample_rate = ec_d("sample_rate");
    ec.samples_per_symbol = ec_ll("samples_per_symbol");
    ec_roi("search_roi", ec.search_lo, ec.search_hi);
    ec.sensing_code = ec_ll("sensing_code");
    ec_roi("sensing_roi", ec.sensing_lo, ec.sensing_hi);
    ec.sic_model_mode = ec_s("sic_model_mode");
    ec_roi("subtraction_roi", ec.sub_lo, ec.sub_hi);
    ec.sync_reps = ec_ll("sync_reps");
    ec.validate_threshold = ec_d("validate_threshold");
    ec.validation_reps = ec_ll("validation_reps");

    c.effective_config_hash = kv_get(kv, "effective_config_hash");
    c.effective_config_json = effective_config_json(ec);
    c.effective_config_hash_recomputed = sha256_text(c.effective_config_json);
    if (c.effective_config_hash.empty() ||
        c.effective_config_hash != c.effective_config_hash_recomputed) {
        throw std::runtime_error(
            "corpus effective_config hash mismatch: recorded '" +
            c.effective_config_hash + "' recomputed '" +
            c.effective_config_hash_recomputed +
            "' (the corpus and this tool disagree on the S1 effective config; "
            "regenerate with --generate-inputs)");
    }
    // The recorded effective config and the loose cfg.* fields must agree, so a
    // hand-edited inputs.txt cannot create two different identities.
    auto need = [&](const std::string& what, long long a, long long b) {
        if (a != b)
            throw std::runtime_error("corpus identity mismatch: " + what +
                                     " cfg=" + std::to_string(a) +
                                     " effective_config=" + std::to_string(b));
    };
    need("input_length", c.n, ec.input_length);
    need("fit_reps", c.fit_reps, ec.fit_reps);
    need("validation_reps", c.validation_reps, ec.validation_reps);
    need("online_validation_reps", c.online_validation_reps,
         ec.online_validation_reps);
    need("evaluation_reps", c.evaluation_reps, ec.evaluation_reps);
    need("k_paths", c.k_paths, ec.k_paths);
    need("sync_reps", c.sync_reps, ec.sync_reps);
    need("sensing_code", c.sensing_code, ec.sensing_code);
    need("comm_code", c.comm_code, ec.comm_code);
    need("search_lo", c.search_lo, ec.search_lo);
    need("search_hi", c.search_hi, ec.search_hi);
    need("sub_lo", c.sub_lo, ec.sub_lo);
    need("sub_hi", c.sub_hi, ec.sub_hi);
    need("sensing_lo", c.sensing_lo, ec.sensing_lo);
    need("sensing_hi", c.sensing_hi, ec.sensing_hi);
    need("core_hi", c.sub_hi, ec.core_hi);
    need("post_tail", static_cast<long long>(c.post_tail), ec.post_tail);
    need("coarse_stride", static_cast<long long>(c.coarse_stride),
         ec.coarse_stride);
    need("samples_per_symbol", static_cast<long long>(kSps),
         ec.samples_per_symbol);
    if (c.model_mode != ec.sic_model_mode)
        throw std::runtime_error(
            "corpus model mode mismatch: cfg=" + c.model_mode +
            " effective_config=" + ec.sic_model_mode);

    // The S1 estimation visibility prefix is the PUBLIC plan: candidate ROI
    // start + fit_reps SYNC.  It is 0 (whole window) only in the explicit
    // confirmed_legacy comparison, matching uwb_sim_preamble_sic.py.
    if (c.model_mode == "estimate8_frozen") {
        c.observed_end = std::min<int64_t>(
            c.n, c.search_lo + static_cast<int64_t>(c.fit_reps) *
                                   static_cast<int64_t>(kSps));
    } else {
        c.observed_end = 0;
    }

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
    if (c.comm_preamble_sha != ec.comm_template_sha256)
        throw std::runtime_error(
            "corpus template sha mismatch: corpus=" + c.comm_preamble_sha +
            " effective_config=" + ec.comm_template_sha256);
    if (static_cast<long long>(c.comm_preamble_len) != ec.comm_template_length)
        throw std::runtime_error(
            "corpus template length mismatch: corpus=" +
            std::to_string(c.comm_preamble_len) + " effective_config=" +
            std::to_string(ec.comm_template_length));

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
        std::string isr_s, noise_s, cls_s, comm_s;
        is >> tag >> t.index >> t.seed >> t.length >> t.sha256 >>
            t.overlap_lo >> t.overlap_hi >> isr_s >> noise_s >> t.file >>
            cls_s >> comm_s;
        t.isr_db = (isr_s == "nan") ? std::nan("") : std::stod(isr_s);
        t.noise_std = std::stod(noise_s);
        t.collision_class = cls_s.empty() ? "collision_100" : cls_s;
        t.comm_present = (comm_s != "0");
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
    // the SIC behaviour depend on fit/val/online/model mode, so a mismatch is a
    // hard error (the perf row may only bind to the case the corpus was
    // generated from).
    if (c.fit_reps != o.fit_reps || c.validation_reps != o.validation_reps ||
        c.online_validation_reps != o.online_validation_reps ||
        c.model_mode != o.model_mode || c.evaluation_reps != o.evaluation_reps) {
        throw std::runtime_error(
            "corpus config mismatch: corpus has fit/val/online/eval " +
            std::to_string(c.fit_reps) + "/" + std::to_string(c.validation_reps) +
            "/" + std::to_string(c.online_validation_reps) + "/" +
            std::to_string(c.evaluation_reps) + " model=" + c.model_mode +
            " but the run requested " + std::to_string(o.fit_reps) + "/" +
            std::to_string(o.validation_reps) + "/" +
            std::to_string(o.online_validation_reps) + "/" +
            std::to_string(o.evaluation_reps) + " model=" + o.model_mode +
            "; regenerate with --generate-inputs (or point --inputs at the "
            "matching corpus)");
    }
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
    c.online_validation_reps = o.online_validation_reps;
    c.evaluation_reps = o.evaluation_reps;
    c.model_mode = o.model_mode;
    c.seed_set = "synthetic";
    c.seed_start = 1000;
    c.rate = kWorkRate;
    c.pre = static_cast<int64_t>(kPreSyn);
    c.n = static_cast<int64_t>(n);
    c.search_lo = static_cast<int64_t>(kPreSyn);
    c.search_hi = static_cast<int64_t>(kPreSyn + kSyncReps * kSps);
    c.sensing_lo = c.search_lo;
    c.sensing_hi = c.search_hi;
    c.sub_lo = c.search_lo;
    c.sub_hi = std::min<int64_t>(
        c.n, c.search_hi + static_cast<int64_t>(kSyncReps * kSps) + kPostTail);
    c.observed_end =
        (o.model_mode == "estimate8_frozen")
            ? std::min<int64_t>(
                  c.n, c.search_lo + static_cast<int64_t>(o.fit_reps) * kSps)
            : 0;
    c.psdu_hex = kCanonicalPsdu;
    c.comm_template_mode = "synthetic_microbenchmark";
    c.config_hash = "synthetic";
    c.effective_config_hash = "synthetic";
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

// JSON number that cannot produce a bare `nan`/`inf` token.
std::string
json_num(double v)
{
    if (!std::isfinite(v))
        return "null";
    std::ostringstream o;
    o << std::setprecision(6) << v;
    return o.str();
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
// Per-window S1 timing split. Training (acq/estimate8/model_prepare) and apply
// (backfill/forward) are kept in separate fields and are never averaged
// together; `train_total` is the estimate_step parent of the first three.
// ---------------------------------------------------------------------------
struct StageTimes {
    double acq = 0.0;           // detect_coarse + detect_fine
    double estimate8 = 0.0;     // cfo+cir+path_select+frac+matrix+gain
    double model_prepare = 0.0; // fit residual + freeze
    double train_total = 0.0;   // estimate_total_us (parent)
    double backfill = 0.0;      // apply over [0, observed_end)
    double apply_forward = 0.0; // apply over [observed_end, n)
    double apply = 0.0;         // backfill + forward
    double adapter_wrap = 0.0;  // adapter PMT wrap/publish timer
    double handler = 0.0, queue_wait = 0.0, core_total = 0.0;
    bool available = false;
    bool publish_available = false;
    size_t observed_end = 0, backfill_samples = 0, forward_samples = 0;
};

// Per-window pipeline outcome
struct Outcome {
    double wall_us = 0.0;     // submit -> downstream terminal result
    double sic_us = 0.0;      // SIC step only
    double consumer_us = 0.0; // consumer step only
    Snap sic;
    Snap cons;
    StageTimes t;
    std::string collision_class;
    bool expect_commit = true; // 100% class must commit
    bool bypass_bit_identical = false;
    bool informational_commit = false; // 10% class committed (reported, not gated)
    bool effect_ok = false;
    std::string effect_reason;
    std::string downstream; // "sic_only" | status string
};

// Fill the split from an adapter out-meta snapshot (gr / native mode).
StageTimes
stage_times_from_snap(const Snap& s, size_t n)
{
    StageTimes t;
    if (!s.has_timings)
        return t;
    t.available = true;
    t.publish_available = true;
    t.acq = static_cast<double>(s.detect_coarse_us + s.detect_fine_us);
    t.estimate8 = static_cast<double>(s.cfo_us + s.cir_us + s.path_select_us +
                                      s.fractional_refine_us + s.matrix_build_us +
                                      s.gain_fit_us);
    t.model_prepare = static_cast<double>(s.model_prepare_us);
    t.train_total = static_cast<double>(s.estimate_us);
    t.backfill = static_cast<double>(s.backfill_us);
    t.apply_forward = static_cast<double>(s.apply_forward_us);
    t.apply = static_cast<double>(s.apply_us > 0
                                      ? s.apply_us
                                      : s.backfill_us + s.apply_forward_us);
    t.adapter_wrap = static_cast<double>(s.adapter_wrap_us);
    t.handler = static_cast<double>(s.handler_us);
    t.queue_wait = static_cast<double>(s.queue_wait_us);
    t.core_total = static_cast<double>(s.core_total_us);
    t.observed_end = s.observed_end > 0
                         ? static_cast<size_t>(
                               std::min<int64_t>(s.observed_end,
                                                 static_cast<int64_t>(n)))
                         : 0;
    // observed_end == 0 means "the whole window is visible": sic_one then runs
    // the WHOLE apply as the backfill pass and the forward pass is empty.
    t.backfill_samples = t.observed_end > 0 ? t.observed_end : n;
    t.forward_samples = t.observed_end > 0 ? n - t.observed_end : 0;
    return t;
}

// Fill the split from the core's SicTimings (core mode). There is no adapter
// wrap timer on this layer, so publish_available stays false.
StageTimes
stage_times_from_core(const sic::SicTimings& tt, size_t n, size_t observed_end)
{
    StageTimes t;
    t.available = true;
    t.acq = static_cast<double>(tt.detect_coarse_us + tt.detect_fine_us);
    t.estimate8 = static_cast<double>(tt.cfo_us + tt.cir_us + tt.path_select_us +
                                      tt.fractional_refine_us +
                                      tt.matrix_build_us + tt.gain_fit_us);
    t.model_prepare = static_cast<double>(tt.model_prepare_us);
    t.train_total = static_cast<double>(tt.estimate_total_us);
    t.backfill = static_cast<double>(tt.backfill_us);
    t.apply_forward = static_cast<double>(tt.apply_forward_us);
    t.apply = static_cast<double>(tt.apply_us > 0
                                      ? tt.apply_us
                                      : tt.backfill_us + tt.apply_forward_us);
    t.core_total = static_cast<double>(tt.core_total_us);
    t.observed_end = std::min<size_t>(observed_end, n);
    t.backfill_samples = t.observed_end > 0 ? t.observed_end : n;
    t.forward_samples = t.observed_end > 0 ? n - t.observed_end : 0;
    return t;
}

// Structural apply cost per output sample for K interference paths: one complex
// MAC into the reconstruction accumulator per path (6 FLOPs: 4 mul + 2 add),
// one complex CFO rotate (6 FLOPs) and one complex subtract (2 FLOPs).
// This is the hardware-loop operation count the S1 instruction asks to report,
// NOT a measured instruction count.
double
apply_flops_per_sample(size_t k_paths)
{
    return 6.0 * static_cast<double>(k_paths) + 8.0;
}

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
        // S1: the SENSING processing ROI (the subtraction/commit region) and
        // the PUBLIC-PLAN estimation visibility prefix. Both come from the
        // effect-run effective config; the injected comm start is never used.
        d = pmt::dict_add(d, pmt::mp("sensing_roi_lo"),
                          pmt::from_long(c_.sensing_lo));
        d = pmt::dict_add(d, pmt::mp("sensing_roi_hi"),
                          pmt::from_long(c_.sensing_hi));
        if (c_.observed_end > 0)
            d = pmt::dict_add(d, pmt::mp("observed_end_sample"),
                              pmt::from_long(c_.observed_end));
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
            /*pending_bypass_max=*/0,
            /*support_mode=*/(c.model_mode == "confirmed_legacy"
                                  ? "confirmed_legacy"
                                  : "configured_nominal"),
            /*online_validation_reps=*/
            static_cast<size_t>(c.online_validation_reps));
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
        d = pmt::dict_add(d, pmt::mp("sensing_roi_lo"),
                          pmt::from_long(c_.sensing_lo));
        d = pmt::dict_add(d, pmt::mp("sensing_roi_hi"),
                          pmt::from_long(c_.sensing_hi));
        if (c_.observed_end > 0)
            d = pmt::dict_add(d, pmt::mp("observed_end_sample"),
                              pmt::from_long(c_.observed_end));
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
        out.collision_class = t.collision_class;
        // Expected terminal state by collision class. `collision_100` (the
        // nominal effect-set cell) must commit AND cover the whole injected
        // overlap.  `no_collision` (no comm at all) and `collision_10` (comm
        // starts AFTER the public 8-SYNC visibility prefix) are expected to be
        // bypassed -- a bypass is the correct answer there, not a failure, but
        // a *failed* state or a changed output still is.
        // Expected terminal state by collision class:
        //   collision_100 : must commit AND cover the whole injected overlap
        //                   (the nominal effect-set cell).
        //   no_collision  : there is no interferer at all, so a commit is a
        //                   FALSE COMMIT and the output must be y unchanged.
        //   collision_10  : the comm starts AFTER the public 8-SYNC visibility
        //                   prefix. This class is INFORMATIONAL: it shows the
        //                   conditional model_prepare/apply cost and whether an
        //                   out-of-prefix interferer causes a mis-localised
        //                   commit. A commit is counted and reported, not used
        //                   as the effect gate (the gate is the 100% cell).
        const std::string& cls = t.collision_class;
        out.expect_commit = (cls == "collision_100");
        const bool strict_no_interference = (cls == "no_collision");
        std::string sic_state = out.sic.sic_state;
        if (sic_state.empty())
            sic_state = "bypassed";
        if (o_.mode == "core" && sic_state != "committed") {
            const auto& y = c_.wins[widx];
            out.bypass_bit_identical =
                core_out_.size() == y.size() &&
                std::memcmp(core_out_.data(), y.data(),
                            y.size() * sizeof(cplx)) == 0;
        }
        if (!out.expect_commit) {
            if (sic_state == "failed") {
                out.effect_reason = "sic_state=failed(unexpected)";
                return;
            }
            if (strict_no_interference) {
                if (sic_state == "committed") {
                    out.effect_reason = "false_commit_no_interference";
                    return;
                }
                if (o_.mode == "core" && !out.bypass_bit_identical) {
                    out.effect_reason = "bypass_not_bit_identical";
                    return;
                }
            } else if (sic_state == "committed") {
                out.informational_commit = true;
            }
            if (!consumer_blk_) {
                out.downstream = "sic_only_bypass";
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
            return;
        }
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
    StageTimes last_core_t_;
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
        out.t = last_core_t_;
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
        out.t = stage_times_from_snap(out.sic, static_cast<size_t>(c_.n));
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
        out.t = stage_times_from_snap(out.sic, static_cast<size_t>(c_.n));
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
        // S1: the SENSING processing ROI (subtraction/commit region) and the
        // public-plan visibility prefix, exactly as the effect runner sets them
        // (roi_lo/roi_hi = candidate search + the widened core ROI;
        // sub_roi = the sensing ROI).
        cfg.sub_roi_lo = c_.sensing_lo;
        cfg.sub_roi_hi = c_.sensing_hi;
        cfg.sub_min_overlap = 0;
        cfg.support_mode = (c_.model_mode == "confirmed_legacy")
                               ? sic::SicSupportMode::ConfirmedLegacy
                               : sic::SicSupportMode::ConfiguredNominal;
        cfg.observed_end_sample = c_.observed_end;
        cfg.online_validation_reps =
            static_cast<size_t>(c_.online_validation_reps);
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
    s.model_mode = (c_.model_mode == "confirmed_legacy") ? "confirmed_legacy"
                                                         : "estimate8_frozen";
    s.model_id = r.model_id;
    s.observed_end = r.observed_end_sample;
    s.support_source = r.support_source ? r.support_source : "";
    s.detect_coarse_us = r.timings.detect_coarse_us;
    s.detect_fine_us = r.timings.detect_fine_us;
    s.cfo_us = r.timings.cfo_us;
    s.cir_us = r.timings.cir_us;
    s.path_select_us = r.timings.path_select_us;
    s.fractional_refine_us = r.timings.fractional_refine_us;
    s.matrix_build_us = r.timings.matrix_build_us;
    s.gain_fit_us = r.timings.gain_fit_us;
    s.estimate_us = r.timings.estimate_total_us;
    s.model_prepare_us = r.timings.model_prepare_us;
    s.apply_us = r.timings.apply_us;
    s.backfill_us = r.timings.backfill_us;
    s.apply_forward_us = r.timings.apply_forward_us;
    s.has_timings = true;
    last_core_t_ = stage_times_from_core(r.timings, static_cast<size_t>(c_.n),
                                         static_cast<size_t>(c_.observed_end));
    last_core_t_.observed_end = static_cast<size_t>(
        std::max<int64_t>(0, std::min<int64_t>(r.observed_end_sample > 0
                                                   ? r.observed_end_sample
                                                   : c_.observed_end,
                                               c_.n)));
    core_checksum_ += static_cast<double>(std::abs(core_out_[c_.n / 2])) +
                      static_cast<double>(std::abs(core_out_[c_.n / 3]));
    return s;
}

// ---------------------------------------------------------------------------
// Timing loop + report
// ---------------------------------------------------------------------------
struct Timings {
    Stats acq, estimate8, model_prepare, train_total, backfill, apply_forward,
        apply, adapter_wrap, consumer, wall, handler, queue_wait, core_total;
};

void
print_timings(const Timings& t, bool publish_available)
{
    print_row("acquisition", t.acq);
    print_row("estimate8 (8-SYNC)", t.estimate8);
    print_row("model_prepare", t.model_prepare);
    print_row("train total (parent)", t.train_total);
    print_row("backfill (first 8 SYNC)", t.backfill);
    print_row("apply forward", t.apply_forward);
    print_row("apply (back+fwd)", t.apply);
    if (publish_available)
        print_row("adapter PMT wrap", t.adapter_wrap);
    print_row("core total", t.core_total);
}

void
timings_obj(std::ostringstream& j, const Timings& t)
{
    j << "{\"acquisition\":";
    stats_obj(j, t.acq);
    j << ",\"estimate8\":";
    stats_obj(j, t.estimate8);
    j << ",\"model_prepare\":";
    stats_obj(j, t.model_prepare);
    j << ",\"train_total\":";
    stats_obj(j, t.train_total);
    j << ",\"backfill\":";
    stats_obj(j, t.backfill);
    j << ",\"apply_forward\":";
    stats_obj(j, t.apply_forward);
    j << ",\"apply\":";
    stats_obj(j, t.apply);
    j << ",\"adapter_wrap\":";
    stats_obj(j, t.adapter_wrap);
    j << ",\"consumer\":";
    stats_obj(j, t.consumer);
    j << ",\"wall\":";
    stats_obj(j, t.wall);
    j << ",\"core_total\":";
    stats_obj(j, t.core_total);
    j << "}";
}

// Accumulates the S1 split. Apply statistics only take windows where the frozen
// model actually ran (committed), so a bypass cannot dilute apply throughput
// and training is never folded into it.
struct StageAcc {
    std::vector<double> acq, est8, prepare, train, backfill, fwd, apply, wrap,
        consumer, wall, handler, queue_wait, core_total;
    uint64_t n_windows = 0, n_committed = 0, n_apply = 0;

    void add(const Outcome& o)
    {
        ++n_windows;
        wall.push_back(o.wall_us);
        if (!o.t.available)
            return;
        const bool committed = (o.sic.sic_state == "committed");
        if (committed)
            ++n_committed;
        // Training is only meaningful when the estimator ran to a model freeze.
        acq.push_back(o.t.acq);
        est8.push_back(o.t.estimate8);
        prepare.push_back(o.t.model_prepare);
        train.push_back(o.t.train_total);
        core_total.push_back(o.t.core_total);
        handler.push_back(o.t.handler);
        queue_wait.push_back(o.t.queue_wait);
        if (o.t.publish_available)
            wrap.push_back(o.t.adapter_wrap);
        if (committed && o.t.apply > 0.0) {
            ++n_apply;
            if (o.t.backfill_samples > 0)
                backfill.push_back(o.t.backfill);
            if (o.t.forward_samples > 0)
                fwd.push_back(o.t.apply_forward);
            apply.push_back(o.t.apply);
        }
        if (o.consumer_us > 0.0)
            consumer.push_back(o.consumer_us);
    }

    Timings finish()
    {
        Timings t;
        t.acq = summarize(acq);
        t.estimate8 = summarize(est8);
        t.model_prepare = summarize(prepare);
        t.train_total = summarize(train);
        t.backfill = summarize(backfill);
        t.apply_forward = summarize(fwd);
        t.apply = summarize(apply);
        t.adapter_wrap = summarize(wrap);
        t.consumer = summarize(consumer);
        t.wall = summarize(wall);
        t.handler = summarize(handler);
        t.queue_wait = summarize(queue_wait);
        t.core_total = summarize(core_total);
        return t;
    }
};

struct RunResult {
    Stats wall, sic, consumer;
    Timings timings;                       // all timed windows
    Timings committed;                     // committed windows only
    std::map<std::string, Timings> by_class; // conditional cost per class
    uint64_t completions = 0, effect_ok = 0, effect_fail = 0, warmup = 0;
    uint64_t term_committed = 0, term_bypassed = 0, term_failed = 0,
             term_ambiguous = 0;
    uint64_t downstream_ok = 0, fcs_pass = 0, psdu_match = 0, psdu_mismatch = 0;
    uint64_t miss1 = 0, miss500 = 0;
    uint64_t bypass_bit_identical = 0, bypass_checked = 0;
    uint64_t n_apply = 0;
    uint64_t false_commit_no_interference = 0;
    uint64_t informational_commit_10 = 0;
    size_t forward_samples = 0, backfill_samples = 0, observed_end = 0;
    std::map<std::string, uint64_t> class_windows;
    std::map<std::string, uint64_t> class_committed;
    std::map<std::string, uint64_t> class_bypassed;
    std::set<std::string> model_modes_seen;
    size_t inflight_hwm = 0, queue_hwm = 0, consumer_queue_hwm = 0;
    double checksum = 0.0;
    double mean_adapter_core_us = 0.0;
    double mean_residual_corr = 0.0;
    std::map<std::string, uint64_t> fail_reasons;
    std::string first_fail;
};

RunResult
run_pipeline(const Options& o, const Corpus& c, Pipeline& pipe,
             std::ofstream& csv, double& cpu_out, bool effect_verified)
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

    res.observed_end = static_cast<size_t>(std::max<int64_t>(0, c.observed_end));
    // observed_end == 0 = whole window visible: the whole apply is the backfill.
    res.backfill_samples = res.observed_end > 0
                               ? std::min<size_t>(res.observed_end,
                                                  static_cast<size_t>(c.n))
                               : static_cast<size_t>(c.n);
    res.forward_samples = res.observed_end > 0
                              ? static_cast<size_t>(c.n) - res.backfill_samples
                              : 0;

    std::vector<double> wall_v, sic_v, cons_v;
    StageAcc acc_all;
    StageAcc acc_committed;
    std::map<std::string, StageAcc> acc_class;
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
            acc_all.add(out);
            acc_class[out.collision_class].add(out);
            ++res.class_windows[out.collision_class];
            const std::string& st = out.sic.sic_state;
            if (st == "committed") {
                acc_committed.add(out);
                ++res.term_committed;
                ++res.class_committed[out.collision_class];
                ++res.n_apply;
            } else if (st == "bypassed") {
                ++res.term_bypassed;
                ++res.class_bypassed[out.collision_class];
            } else if (st == "ambiguous")
                ++res.term_ambiguous;
            else
                ++res.term_failed;
            if (o.mode == "core" && st != "committed") {
                ++res.bypass_checked;
                if (out.bypass_bit_identical)
                    ++res.bypass_bit_identical;
            }
            if (out.informational_commit)
                ++res.informational_commit_10;
            if (out.effect_reason == "false_commit_no_interference")
                ++res.false_commit_no_interference;
            if (!out.sic.model_mode.empty())
                res.model_modes_seen.insert(out.sic.model_mode);
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
            const double fwd_samples = static_cast<double>(out.t.forward_samples);
            const double apply_ns_per_sample =
                (out.t.forward_samples > 0 && out.t.apply_forward > 0.0)
                    ? 1000.0 * out.t.apply_forward / fwd_samples
                    : 0.0;
            const double apply_msps =
                (out.t.forward_samples > 0 && out.t.apply_forward > 0.0)
                    ? fwd_samples / out.t.apply_forward
                    : 0.0;
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
                << csv_field(out.effect_reason)
                // ---- S1 appended columns (stable order) -------------------
                << ',' << c.model_mode
                << ',' << (effect_verified ? "true" : "false")
                << ',' << c.effective_config_hash
                << ',' << (out.collision_class.empty() ? "unknown"
                                                       : out.collision_class)
                << ',' << out.sic.observed_end << ',' << out.sic.model_id
                << ',' << out.t.acq << ',' << out.t.estimate8 << ','
                << out.t.model_prepare << ',' << out.t.train_total << ','
                << out.t.backfill << ',' << out.t.apply_forward << ','
                << out.t.apply << ',' << out.t.adapter_wrap << ','
                << out.t.forward_samples << ',' << out.t.backfill_samples << ','
                << apply_ns_per_sample << ',' << apply_msps << ','
                << apply_flops_per_sample(static_cast<size_t>(c.k_paths)) << ','
                << out.t.handler << ',' << out.t.queue_wait << '\n';
        }
    }
    cpu_out = cpu_seconds() - cpu0;

    res.wall = summarize(wall_v);
    res.sic = summarize(sic_v);
    res.consumer = summarize(cons_v);
    res.completions = res.wall.n;
    res.timings = acc_all.finish();
    res.committed = acc_committed.finish();
    for (auto& kv : acc_class)
        res.by_class[kv.first] = kv.second.finish();
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
        auto parent_of = [](const std::string& p) {
            const size_t slash = p.find_last_of('/');
            return (slash == std::string::npos) ? std::string(".")
                                                : p.substr(0, slash);
        };
        const std::string out_parent = parent_of(o.output);
        if (o.inputs.empty() && o.input_source == "real")
            o.inputs = out_parent + "/real_inputs";

        // S1-02: locate the effect run this row must bind to. The benchmark's
        // effective config is compared field-by-field AND by canonical hash
        // against it, so "same config" is verified, not asserted.
        std::string effect_dir;
        std::vector<std::string> effect_rejected;
        if (o.input_source == "real")
            effect_dir = find_effect_run_dir(o.effect_run, out_parent,
                                             o.model_mode, effect_rejected);
        if (!effect_dir.empty() && o.check_against.empty())
            o.check_against = effect_dir;

        ensure_private_output(o.output);

        std::printf("== S1 preamble-only SIC layered benchmark (8-SYNC frozen) "
                    "==\n");
        std::printf("preset=%s mode=%s consumer=%s input_source=%s "
                    "model_mode=%s\n",
                    o.preset.c_str(), o.mode.c_str(), o.consumer.c_str(),
                    o.input_source.c_str(), o.model_mode.c_str());
        std::printf("windows=%zu rounds=%zu distinct=%zu fit=%d val=%d "
                    "online_val=%d eval=%d output=%s\n",
                    o.windows, o.rounds, o.distinct, o.fit_reps,
                    o.validation_reps, o.online_validation_reps,
                    o.evaluation_reps, o.output.c_str());
        if (o.nonproduction_control)
            std::printf("*** NON-PRODUCTION CONTROL RUN: %s%s.  NOT the "
                        "estimate8_frozen effect profile and NOT official "
                        "evidence. ***\n",
                        o.model_mode.c_str(),
                        o.cover_preamble ? " + --cover-preamble (S0 legacy "
                                           "support)" : "");
        if (o.input_source == "synthetic")
            std::printf("*** SYNTHETIC MICROBENCHMARK: pseudo-random +/-1 chips "
                        "+ 9-tap Gaussian pulse.  NOT production-profile "
                        "evidence. ***\n");
        std::printf("single worker, serial, one window in flight; %s\n",
                    (o.windows >= 10000 && o.rounds >= 3)
                        ? "FORMAL perf (>=10000 windows x 3 rounds)"
                        : "QUALIFICATION run (not the formal 10000x3 row)");
        if (!effect_dir.empty())
            std::printf("effect run binding: %s\n", effect_dir.c_str());
        else if (o.input_source == "real")
            std::printf("effect run binding: NONE FOUND in %s%s\n",
                        out_parent.c_str(),
                        effect_rejected.empty()
                            ? ""
                            : " (rejected: see the JSON effect_binding.rejected)");

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
                "online_val=%d eval=%d model=%s seed_set=%s seed_start=%lld "
                "search_roi=[%lld,%lld) sub_roi=[%lld,%lld) "
                "sensing_roi=[%lld,%lld) snr=%.1f isr=%.1f "
                "observed_end=%lld\n",
                c.count(), static_cast<long long>(c.n),
                static_cast<long long>(c.pre), c.fit_reps, c.validation_reps,
                c.online_validation_reps, c.evaluation_reps,
                c.model_mode.c_str(), c.seed_set.c_str(), c.seed_start,
                static_cast<long long>(c.search_lo),
                static_cast<long long>(c.search_hi),
                static_cast<long long>(c.sub_lo),
                static_cast<long long>(c.sub_hi),
                static_cast<long long>(c.sensing_lo),
                static_cast<long long>(c.sensing_hi), c.snr_db, c.isr_db,
                static_cast<long long>(c.observed_end));
            std::printf("  config_hash=%s effective_config_hash=%s\n",
                        c.config_hash.c_str(), c.effective_config_hash.c_str());
            std::printf("  effective_config: %s\n",
                        c.effective_config_json.c_str());
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

        // ---- S1-02 effect binding -----------------------------------------
        // The measured row may only be quoted for the effect set if its
        // effective config (canonical hash + every field), template and input
        // length match a PASSING effect run.  manifest -> hash -> gates -> diff.
        EffectRun eff;
        if (effect_dir.empty()) {
            eff.error = "no effect run found (searched " + out_parent + ")";
        } else if (!load_effect_run(effect_dir, eff)) {
            // eff.error set by the loader
        } else {
            compare_effective_config(c.ec, eff.config, eff.mismatch);
            eff.config_match = eff.mismatch.empty();
            if (!eff.hash_ok)
                eff.mismatch.push_back(
                    "effect run's own recorded hash does not match its "
                    "manifest effective_config");
        }
        // The heuristic "newest passing effect run" may be a different cell; if
        // it does not carry OUR exact config, bind to the newest passing run
        // that does (the row may only be quoted for an identical config).
        if (c.real_input && o.effect_run.empty() &&
            (!eff.basic_ok() || eff.config_json != c.effective_config_json)) {
            EffectRun alt;
            if (find_effect_run_for_config(out_parent, c.effective_config_json,
                                           alt)) {
                eff = alt;
                effect_dir = alt.dir;
                compare_effective_config(c.ec, eff.config, eff.mismatch);
                eff.config_match = eff.mismatch.empty();
            }
        }
        const bool legacy_or_control =
            (o.model_mode != "estimate8_frozen" || o.online_validation_reps != 0 ||
             o.cover_preamble || o.nonproduction_control);
        const bool effect_verified = eff.ok() &&
                                     c.effective_config_json == eff.config_json &&
                                     (c.effective_config_hash ==
                                      eff.effective_config_hash);
        const bool production_profile =
            c.real_input && o.mode != "native" && !legacy_or_control &&
            effect_verified;
        const bool formal_perf = production_profile && o.windows >= 10000 &&
                                 o.rounds >= 3 && o.distinct >= 64;

        if (c.real_input) {
            std::printf("effect binding: %s\n",
                        effect_verified ? "VERIFIED" : "NOT VERIFIED");
            if (eff.found) {
                std::printf("  effect_run=%s run_id=%s model_mode=%s "
                            "gates(eff=%s,s1=%s) windows=%lld\n",
                            std::filesystem::path(eff.dir).filename().c_str(),
                            eff.run_id.c_str(), eff.model_mode.c_str(),
                            eff.eff_pass ? "true" : "false",
                            eff.s1_pass ? "true" : "false",
                            eff.windows_completed);
                std::printf("  recorded hash=%s recomputed=%s match=%s\n",
                            eff.effective_config_hash.c_str(),
                            eff.effective_config_hash_recomputed.c_str(),
                            eff.hash_ok ? "true" : "false");
            } else if (!eff.error.empty()) {
                std::printf("  reason: %s\n", eff.error.c_str());
            }
            for (const auto& m : eff.mismatch)
                std::printf("  config mismatch: %s\n", m.c_str());
            std::printf("  production_profile=%s formal_perf=%s\n",
                        production_profile ? "true" : "false",
                        formal_perf ? "true" : "false");
        }
        if (c.real_input && !legacy_or_control && o.mode != "native" &&
            !effect_verified) {
            std::fprintf(stderr,
                         "benchmark_preamble_sic: REFUSING to label this "
                         "real-input estimate8_frozen row as production: its "
                         "config is not effect-verified (effect_run='%s').  "
                         "Rows are written with effect_verified=false for "
                         "diagnosis only and the exit code is non-zero.\n",
                         effect_dir.empty() ? "<none>" : effect_dir.c_str());
        }
        if (!c.real_input) {
            std::printf("effect binding: n/a (synthetic microbenchmark)\n");
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
               "effect_reason,model_mode,effect_verified,effective_config_hash,"
               "collision_class,observed_end_sample,"
               "model_id,acq_us,estimate8_us,model_prepare_us,train_total_us,"
               "backfill_us,apply_forward_us,apply_us,adapter_wrap_us,"
               "forward_samples,backfill_samples,apply_ns_per_sample,"
               "apply_msamples_per_s,apply_flops_per_sample,handler_us,"
               "queue_wait_us\n";

        Pipeline pipe(o, c);
        pipe.start();
        double cpu = 0.0;
        RunResult res = run_pipeline(o, c, pipe, csv, cpu, effect_verified);
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
        std::printf("  case=%s input_len=%lld config_hash=%s "
                    "effective_config_hash=%s\n",
                    c.real_input ? "main-real-effect" : "synthetic",
                    static_cast<long long>(c.n), c.config_hash.c_str(),
                    c.effective_config_hash.empty()
                        ? "-"
                        : c.effective_config_hash.c_str());
        std::printf("  model_mode=%s effect_verified=%s production=%s "
                    "formal=%s collision_classes=%zu\n",
                    c.model_mode.c_str(),
                    (c.real_input ? (effect_verified ? "true" : "false") : "n/a"),
                    (c.real_input ? (production_profile ? "true" : "false")
                                  : "false"),
                    (c.real_input ? (formal_perf ? "true" : "false") : "false"),
                    res.class_windows.size());
        print_row("wall (submit->end)", res.wall);
        print_row("sic step", res.sic);
        if (pipe.real_consumer())
            print_row("consumer step", res.consumer);
        std::printf("  -- S1 split, ALL timed windows (n=%zu) --\n", res.wall.n);
        print_timings(res.timings, o.mode != "core");
        std::printf("  -- S1 split, COMMITTED windows only (n_apply=%llu) --\n",
                    static_cast<unsigned long long>(res.n_apply));
        print_timings(res.committed, o.mode != "core");

        // Apply throughput: computed on committed windows only, never diluted
        // by bypasses, and never mixed with training.
        const bool whole_window_apply = (res.forward_samples == 0);
        const Stats& af = whole_window_apply ? res.committed.backfill
                                             : res.committed.apply_forward;
        const double fwd_samples = static_cast<double>(
            whole_window_apply ? res.backfill_samples : res.forward_samples);
        const double ns_per_sample =
            (af.n > 0 && af.mean_us > 0.0 && fwd_samples > 0.0)
                ? 1000.0 * af.mean_us / fwd_samples
                : 0.0;
        const double msps = (ns_per_sample > 0.0) ? 1e3 / ns_per_sample : 0.0;
        const double flops =
            apply_flops_per_sample(static_cast<size_t>(c.k_paths));
        std::printf("  apply throughput (%s, K=%d paths, len=%zu): "
                    "%.3f ns/sample  %.1f Msample/s  %.1f flops/sample  "
                    "%.2f GFLOP/s  (p50=%.3f ns/sample p99=%.3f)\n",
                    whole_window_apply ? "whole-window backfill pass"
                                       : "forward pass",
                    c.k_paths,
                    whole_window_apply ? res.backfill_samples
                                       : res.forward_samples,
                    ns_per_sample, msps, flops, msps * 1e6 * flops / 1e9,
                    (af.n > 0 && af.p50_us > 0.0 && fwd_samples > 0.0)
                        ? 1000.0 * af.p50_us / fwd_samples
                        : 0.0,
                    (af.n > 0 && af.p99_us > 0.0 && fwd_samples > 0.0)
                        ? 1000.0 * af.p99_us / fwd_samples
                        : 0.0);
        if (res.backfill_samples > 0 && res.committed.backfill.n > 0 &&
            !whole_window_apply) {
            const double bns =
                1000.0 * res.committed.backfill.mean_us /
                static_cast<double>(res.backfill_samples);
            std::printf("  backfill throughput (first 8 SYNC, %zu samples "
                        "retro-cleaned): %.3f ns/sample  %.1f Msample/s\n",
                        res.backfill_samples, bns,
                        bns > 0.0 ? 1e3 / bns : 0.0);
        } else if (whole_window_apply) {
            std::printf("  backfill: the WHOLE window is the backfill pass "
                        "(observed_end=0: no declared visibility prefix, the "
                        "confirmed_legacy contract)\n");
        }
        // Conditional (per-collision-class) training cost: model_prepare is only
        // paid on committed windows, which average-trigger rates would hide.
        for (const auto& kv : res.by_class) {
            const Timings& t = kv.second;
            std::printf("  class %-14s win=%llu committed=%llu prepare=%.1f us "
                        "train=%.1f us apply=%.1f us (backfill=%.1f "
                        "forward=%.1f)\n",
                        kv.first.c_str(),
                        static_cast<unsigned long long>(
                            res.class_windows.count(kv.first)
                                ? res.class_windows.at(kv.first)
                                : 0),
                        static_cast<unsigned long long>(
                            res.class_committed.count(kv.first)
                                ? res.class_committed.at(kv.first)
                                : 0),
                        t.model_prepare.mean_us, t.train_total.mean_us,
                        t.apply.mean_us, t.backfill.mean_us,
                        t.apply_forward.mean_us);
        }

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
        std::printf("  per-class terminal: ");
        for (const auto& kv : res.class_windows) {
            const std::string& cls = kv.first;
            std::printf("%s[w=%llu c=%llu b=%llu] ", cls.c_str(),
                        static_cast<unsigned long long>(kv.second),
                        static_cast<unsigned long long>(
                            res.class_committed.count(cls)
                                ? res.class_committed.at(cls)
                                : 0),
                        static_cast<unsigned long long>(
                            res.class_bypassed.count(cls)
                                ? res.class_bypassed.at(cls)
                                : 0));
        }
        std::printf("\n  bypass out==y bit-identical (core mode): %llu/%llu\n",
                    static_cast<unsigned long long>(res.bypass_bit_identical),
                    static_cast<unsigned long long>(res.bypass_checked));
        std::printf("  negatives: false_commit_no_interference=%llu  "
                    "informational_commit_collision_10=%llu\n",
                    static_cast<unsigned long long>(
                        res.false_commit_no_interference),
                    static_cast<unsigned long long>(
                        res.informational_commit_10));
        std::printf("  model modes seen: ");
        for (const auto& m : res.model_modes_seen)
            std::printf("%s ", m.c_str());
        std::printf("\n");

        // ---- JSON ---------------------------------------------------------
        std::ostringstream json;
        const GitInfo git = git_info(repo);
        json << std::setprecision(6);
        json << "{\n  \"schema\":\"sic_perf_s1_v1\",\n  \"mode\":\"" << o.mode
             << "\",\n  \"consumer\":\"" << o.consumer
             << "\",\n  \"pipeline\":\"" << pipeline_name
             << "\",\n  \"input_source\":\"" << o.input_source
             << "\",\n  \"model_mode\":\"" << c.model_mode
             << "\",\n  \"nonproduction_control\":"
             << (o.nonproduction_control ? "true" : "false")
             << ",\n  \"cover_preamble\":" << (o.cover_preamble ? "true" : "false")
             << ",\n  \"production_profile\":"
             << (production_profile ? "true" : "false")
             << ",\n  \"official_evidence\":"
             << ((production_profile && formal_perf) ? "true" : "false")
             << ",\n  \"formal_perf\":" << (formal_perf ? "true" : "false")
             << ",\n  \"qualification\":"
             << (!formal_perf ? "true" : "false")
             << ",\n  \"windows\":" << o.windows << ",\n  \"rounds\":" << o.rounds
             << ",\n  \"distinct\":" << c.count() << ",\n  \"n_work\":" << c.n
             << ",\n  \"config_hash\":\"" << c.config_hash << "\",\n"
             << "  \"effective_config_hash\":\"" << c.effective_config_hash
             << "\",\n  \"effective_config\":" << c.effective_config_json
             << ",\n"
             << "  \"effect_verified\":"
             << (c.real_input ? (effect_verified ? "true" : "false") : "false")
             << ",\n  \"effect_binding\":{\"effect_run\":\""
             << json_escape(effect_dir.empty()
                                ? std::string()
                                : std::filesystem::path(effect_dir)
                                      .filename()
                                      .string())
             << "\",\"run_id\":\"" << json_escape(eff.run_id)
             << "\",\"found\":" << (eff.found ? "true" : "false")
             << ",\"error\":\"" << json_escape(eff.error)
             << "\",\"model_mode\":\"" << json_escape(eff.model_mode)
             << "\",\"gates\":{\"eff_pass\":" << (eff.eff_pass ? "true" : "false")
             << ",\"s1_pass\":" << (eff.s1_pass ? "true" : "false") << "}"
             << ",\"windows_completed\":" << eff.windows_completed
             << ",\"recorded_hash\":\"" << eff.effective_config_hash
             << "\",\"recomputed_hash\":\""
             << eff.effective_config_hash_recomputed
             << "\",\"hash_ok\":" << (eff.hash_ok ? "true" : "false")
             << ",\"config_match\":" << (eff.config_match ? "true" : "false")
             << ",\"verified\":"
             << (effect_verified ? "true" : "false") << ",\"mismatch\":[";
        for (size_t i = 0; i < eff.mismatch.size(); ++i)
            json << (i ? "," : "") << "\"" << json_escape(eff.mismatch[i])
                 << "\"";
        json << "],\"rejected\":[";
        for (size_t i = 0; i < effect_rejected.size(); ++i)
            json << (i ? "," : "") << "\"" << json_escape(effect_rejected[i])
                 << "\"";
        json << "]},\n"
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
                 << ",\"collision_class\":\"" << t.collision_class
                 << "\",\"comm_present\":"
                 << (t.comm_present ? "true" : "false") << ",\"isr_db\":";
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
             << "\",\"effective_config_hash_recomputed\":\""
             << c.effective_config_hash_recomputed
             << "\",\"collision_mix\":\"" << json_escape(o.collision_mix)
             << "\",\"observed_end_source\":\""
             << (c.model_mode == "confirmed_legacy"
                     ? "whole_window_default"
                     : "public_plan_search_roi_lo_plus_fit_reps")
             << "\",\"benchmark_build_lib_verified_by\":\"ldd "
                "gr-uwb/build-sic/libgnuradio-uwb.so\""
             << ",\"crosscheck_templates_match\":"
             << (c.crosscheck_templates_match ? "true" : "false")
             << ",\"crosscheck_fixture_y_match\":"
             << (c.crosscheck_fixture_match ? "true" : "false")
             << ",\"crosscheck_fixture_windows\":"
             << c.crosscheck_fixture_windows << "},\n";
        json << "  \"resolved_config\":{\"preset\":\"" << c.preset
             << "\",\"case_id\":\"" << json_escape(c.ec.case_id)
             << "\",\"model_mode\":\"" << c.model_mode
             << "\",\"sensing_code\":" << c.sensing_code << ",\"comm_code\":"
             << c.comm_code << ",\"sync_reps\":" << c.sync_reps
             << ",\"fit_reps\":" << c.fit_reps << ",\"validation_reps\":"
             << c.validation_reps << ",\"online_validation_reps\":"
             << c.online_validation_reps << ",\"evaluation_reps\":"
             << c.evaluation_reps << ",\"k_paths\":" << c.k_paths
             << ",\"snr_db\":" << c.snr_db << ",\"isr_db\":" << c.isr_db
             << ",\"pre_guard\":" << c.pre << ",\"search_roi\":[" << c.search_lo
             << "," << c.search_hi << "],\"subtraction_roi\":[" << c.sub_lo
             << "," << c.sub_hi << "],\"sensing_roi\":[" << c.sensing_lo << ","
             << c.sensing_hi << "],\"observed_end_sample\":" << c.observed_end
             << ",\"detect_threshold\":"
             << c.detect_threshold << ",\"validate_threshold\":"
             << c.validate_threshold << ",\"coarse_stride\":" << c.coarse_stride
             << ",\"post_tail\":" << c.post_tail << ",\"seed_set\":\""
             << c.seed_set << "\",\"seed_start\":" << c.seed_start
             << ",\"psdu_hex\":\""
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
        json << "},\n  \"timings_all\":";
        timings_obj(json, res.timings);
        json << ",\n  \"timings_committed\":";
        timings_obj(json, res.committed);
        json << ",\n  \"timings_conditional\":{";
        {
            bool firstc = true;
            for (const auto& kv : res.by_class) {
                json << (firstc ? "" : ",") << "\"" << json_escape(kv.first)
                     << "\":{\"windows\":"
                     << (res.class_windows.count(kv.first)
                             ? res.class_windows.at(kv.first)
                             : 0)
                     << ",\"committed\":"
                     << (res.class_committed.count(kv.first)
                             ? res.class_committed.at(kv.first)
                             : 0)
                     << ",\"bypassed\":"
                     << (res.class_bypassed.count(kv.first)
                             ? res.class_bypassed.at(kv.first)
                             : 0)
                     << ",\"timings\":";
                timings_obj(json, kv.second);
                json << "}";
                firstc = false;
            }
        }
        json << "},\n  \"apply_throughput\":{\"k_paths\":" << c.k_paths
             << ",\"measured_on\":\""
             << (whole_window_apply ? "whole_window_backfill"
                                    : "forward_pass")
             << "\",\"forward_samples\":" << res.forward_samples
             << ",\"backfill_samples\":" << res.backfill_samples
             << ",\"observed_end_sample\":" << c.observed_end
             << ",\"n_apply\":" << res.n_apply << ",\"ns_per_sample\":"
             << ns_per_sample << ",\"msamples_per_s\":" << msps
             << ",\"flops_per_sample\":" << apply_flops_per_sample(c.k_paths)
             << "},\n";
        json << "  \"terminal\":{\"completions\":" << res.completions
             << ",\"committed\":" << res.term_committed << ",\"bypassed\":"
             << res.term_bypassed << ",\"failed\":" << res.term_failed
             << ",\"ambiguous\":" << res.term_ambiguous
             << ",\"downstream_ok\":" << res.downstream_ok << ",\"fcs_pass\":"
             << res.fcs_pass << ",\"psdu_match\":" << res.psdu_match
             << ",\"psdu_mismatch\":" << res.psdu_mismatch
             << ",\"bypass_bit_identical\":" << res.bypass_bit_identical
             << ",\"bypass_checked\":" << res.bypass_checked
             << ",\"false_commit_no_interference\":"
             << res.false_commit_no_interference
             << ",\"informational_commit_collision_10\":"
             << res.informational_commit_10
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
             << ",\"consumer_service_mean_us\":"
             << json_num(pipe.consumer_service_mean_us())
             << ",\"deadline_miss_1ms\":" << res.miss1
             << ",\"deadline_miss_500us\":" << res.miss500
             << ",\"win_per_s\":" << wps << ",\"checksum\":" << res.checksum
             << ",\"mean_adapter_core_us\":" << res.mean_adapter_core_us
             << ",\"mean_residual_corr\":" << json_num(res.mean_residual_corr)
             << ",\"warmup_windows\":" << res.warmup
             << ",\"loadavg_1m\":" << json_num(loadavg_1m())
             << ",\"nproc\":" << std::thread::hardware_concurrency()
             << "},\n";
        json << "  \"deadline_target\":{\"mean_us\":500.0,\"p99_us\":1000.0,"
                "\"win_per_s\":200},\n";
        json << "  \"superseded\":[{\"runs\":[\"real_core\",\"real_cir\","
                "\"real_demod\",\"real_native\",\"core*\",\"gr*\",\"native*\","
                "\"main_est\",\"paced200\"],\"reason\":\"S0 / pre-S1 benchmark "
                "rows: fit/validation-coupled support (8/52, ~93.9% coverage), "
                "synthetic or calibration-seed input and no effective_config "
                "binding to an S1 effect run.  The S1 rows are the "
                "gr-uwb/build-sic/sic-results/s1_perf_* tables (real "
                "acceptance-seed corpus, model_mode estimate8_frozen, "
                "effective_config_hash-bound).\",\"production_profile_input\":"
             << (production_profile ? "true" : "false") << "}]\n}\n";

        csv.close();
        const std::string json_path =
            o.output + "/" + o.mode + "_" + o.consumer + "_summary.json";
        std::ofstream jf(json_path, std::ios::trunc);
        jf << json.str();
        jf.close();
        std::printf("\nwrote %s and %s\n", csv_path.c_str(), json_path.c_str());

        // ---- refusals (S1-02) ---------------------------------------------
        // A production real-input estimate8_frozen row whose config is not
        // effect-verified, or any real row that uses confirmed_legacy while
        // labelled production, is refused with a clear message and non-zero
        // exit.  The artifacts are still written, marked effect_verified=false.
        if (c.real_input && o.mode != "native" && !legacy_or_control &&
            !effect_verified) {
            std::fprintf(
                stderr,
                "benchmark_preamble_sic: REFUSED: real-input row with "
                "model_mode=%s is not bound to a passing effect run (effect "
                "config/template/input-length mismatch or no effect run); "
                "artifacts written with effect_verified=false.\n",
                c.model_mode.c_str());
            return 5;
        }
        if (o.model_mode != "estimate8_frozen" && !o.nonproduction_control) {
            std::fprintf(stderr,
                         "benchmark_preamble_sic: REFUSED: %s requires "
                         "--nonproduction-control (it is not the estimate8_frozen "
                         "effect profile).\n",
                         o.model_mode.c_str());
            return 5;
        }
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
        if (production_profile && !formal_perf)
            std::fprintf(stderr,
                         "benchmark_preamble_sic: qualification row "
                         "(%zu windows x %zu rounds); a formal row needs "
                         ">=10000 windows x 3 rounds (exit 0).\n",
                         o.windows, o.rounds);
        return 0;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "benchmark_preamble_sic: %s\n", e.what());
        return 1;
    }
}
