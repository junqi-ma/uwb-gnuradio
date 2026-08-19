/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Offline process: scheduled SC16 dump → per-window notch → 65/48 @998.4.
 *
 * Not a GNU Radio block.  No UHD.  Reuses RationalResampler65_48Core.
 * Never overwrites DIR/capture.iq.
 */

#include <gnuradio/uwb/uwb_defaults.h>
#include <gnuradio/uwb/uwb_rational_resampler_core.h>
#include <gnuradio/uwb/uwb_scheduled_dump_io.h>
#include <gnuradio/uwb/uwb_tone_canceller.h>
#include <gnuradio/uwb/uwb_window_crop_core.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

using gr::uwb::core::DemodResultMeta;
using gr::uwb::core::DumpWindowMeta;
using gr::uwb::core::JsonField;
using gr::uwb::core::RationalResampler65_48Core;
using gr::uwb::core::ToneEstimate;
using gr::uwb::core::ToneSubtractResult;
using gr::uwb::core::WindowCropGeom;
using gr::uwb::defaults::kNativeScheduledCapture;
using gr::uwb::defaults::kNativeScheduledPostGuard;
using gr::uwb::defaults::kNativeScheduledPreGuard;

namespace {

constexpr double kFs737 = 737.28e6;
constexpr double kFs998 = 998.4e6;
constexpr double kDefaultCenterHz = 6489.6e6;
constexpr double kDefaultToneRfHz = 6256.640e6;
constexpr double kDefaultSearchHz = 80e6;

struct Options {
    std::string dir;
    double center_hz = kDefaultCenterHz;
    double tone_rf_hz = 0.0;
    double tone_search_hz = kDefaultSearchHz;
    std::string select = "all";
    std::string emit = "full";
    std::string taps = "quality_minorder";
    std::string out_format = "cf32";
    bool skip_notch = false;
    bool keep_native_notch = false;
    bool dry_run = false;
};

void usage(const char* argv0)
{
    std::cerr
        << "Usage: " << argv0 << " DIR [options]\n"
        << "  --center-hz HZ          LO used at capture (default 6489.6e6)\n"
        << "  --tone-rf-hz HZ         known CW RF; skip coarse FFT search\n"
        << "  --tone-search-hz HZ     half-width around hint (default 80e6)\n"
        << "  --select all|fcs_pass|scheduled\n"
        << "  --emit full|qm35\n"
        << "  --taps quality_minorder|PATH\n"
        << "  --out-format cf32|sc16\n"
        << "  --skip-notch\n"
        << "  --keep-native-notch     also write capture_notch.iq\n"
        << "  --dry-run\n";
}

bool parse_args(int argc, char** argv, Options& o)
{
    if (argc < 2) {
        usage(argv[0]);
        return false;
    }
    o.dir = argv[1];
    if (!o.dir.empty() && o.dir[0] == '-') {
        usage(argv[0]);
        return false;
    }
    for (int i = 2; i < argc; ++i) {
        const std::string a = argv[i];
        auto need = [&](const char* name) -> const char* {
            if (i + 1 >= argc)
                throw std::runtime_error(std::string("missing value for ") +
                                         name);
            return argv[++i];
        };
        if (a == "--center-hz")
            o.center_hz = std::stod(need("--center-hz"));
        else if (a == "--tone-rf-hz")
            o.tone_rf_hz = std::stod(need("--tone-rf-hz"));
        else if (a == "--tone-search-hz")
            o.tone_search_hz = std::stod(need("--tone-search-hz"));
        else if (a == "--select")
            o.select = need("--select");
        else if (a == "--emit")
            o.emit = need("--emit");
        else if (a == "--taps")
            o.taps = need("--taps");
        else if (a == "--out-format")
            o.out_format = need("--out-format");
        else if (a == "--skip-notch")
            o.skip_notch = true;
        else if (a == "--keep-native-notch")
            o.keep_native_notch = true;
        else if (a == "--dry-run")
            o.dry_run = true;
        else if (a == "-h" || a == "--help") {
            usage(argv[0]);
            return false;
        } else {
            std::cerr << "unknown option: " << a << "\n";
            usage(argv[0]);
            return false;
        }
    }
    if (o.select != "all" && o.select != "fcs_pass" &&
        o.select != "scheduled") {
        std::cerr << "--select must be all|fcs_pass|scheduled\n";
        return false;
    }
    if (o.emit != "full" && o.emit != "qm35") {
        std::cerr << "--emit must be full|qm35\n";
        return false;
    }
    if (o.out_format != "cf32" && o.out_format != "sc16") {
        std::cerr << "--out-format must be cf32|sc16\n";
        return false;
    }
    return true;
}

std::string join_path(const std::string& dir, const std::string& name)
{
    if (dir.empty())
        return name;
    if (dir.back() == '/')
        return dir + name;
    return dir + "/" + name;
}

bool file_ok(const std::string& path)
{
    std::ifstream f(path, std::ios::binary);
    return static_cast<bool>(f);
}

std::string find_taps_file(const std::string& taps_or_profile)
{
    if (taps_or_profile != "quality" && taps_or_profile != "realtime" &&
        taps_or_profile != "quality_minorder" &&
        taps_or_profile != "realtime_minorder") {
        if (file_ok(taps_or_profile))
            return taps_or_profile;
        throw std::runtime_error("cannot open taps: " + taps_or_profile);
    }
    const std::string fname = "taps_" + taps_or_profile + ".txt";
    const char* env = std::getenv("UWB_TESTDATA");
    std::vector<std::string> cands;
    if (env && *env)
        cands.push_back(std::string(env) + "/resampler_65_48/" + fname);
    const char* prefixes[] = {
        "testdata/resampler_65_48/",
        "../testdata/resampler_65_48/",
        "../../testdata/resampler_65_48/",
        "../../../testdata/resampler_65_48/",
        "../../../../testdata/resampler_65_48/",
    };
    for (const char* p : prefixes)
        cands.push_back(std::string(p) + fname);
    for (const auto& c : cands) {
        if (file_ok(c))
            return c;
    }
    throw std::runtime_error("cannot find " + fname +
                             " under testdata/resampler_65_48/");
}

std::vector<float> load_taps_f32(const std::string& path)
{
    std::ifstream f(path, std::ios::binary | std::ios::ate);
    if (!f)
        throw std::runtime_error("cannot open taps: " + path);
    const auto bytes = static_cast<size_t>(f.tellg());
    if (bytes == 0 || bytes % sizeof(float) != 0)
        throw std::runtime_error("taps size is not a float32 multiple: " + path);
    f.seekg(0);
    std::vector<float> taps(bytes / sizeof(float));
    f.read(reinterpret_cast<char*>(taps.data()),
           static_cast<std::streamsize>(bytes));
    if (!f)
        throw std::runtime_error("failed reading taps: " + path);
    return taps;
}

int64_t map_native(int64_t p, size_t T)
{
    const double d = 0.5 * static_cast<double>(T > 0 ? T - 1 : 0);
    const double m = (static_cast<double>(p) * 65.0 + d) / 48.0;
    const int64_t r = static_cast<int64_t>(std::llround(m));
    return r < 0 ? 0 : r;
}

void resample_oneshot(RationalResampler65_48Core& core,
                      const std::vector<float>& taps,
                      const std::complex<float>* in,
                      size_t n_in,
                      std::vector<std::complex<float>>& out)
{
    core.reset();
    const size_t Lout =
        RationalResampler65_48Core::expected_output_length(n_in, taps.size());
    out.resize(Lout + 64);
    size_t produced = 0;
    if (n_in > 0) {
        auto r = core.process(in, n_in, out.data(), out.size());
        produced = r.produced;
    }
    while (produced < Lout) {
        if (produced >= out.size())
            out.resize(produced + 256);
        const size_t n =
            core.flush(out.data() + produced, out.size() - produced);
        if (n == 0)
            break;
        produced += n;
    }
    out.resize(produced);
}

float quantize_cf32_to_sc16(const std::complex<float>* in,
                            size_t n,
                            std::vector<int16_t>& out)
{
    float peak = 0.0f;
    for (size_t i = 0; i < n; ++i) {
        peak = std::max(peak, std::abs(in[i].real()));
        peak = std::max(peak, std::abs(in[i].imag()));
    }
    const float scale = (peak > 0.0f) ? (32767.0f / peak) : 1.0f;
    out.resize(n * 2);
    uint64_t clips = 0;
    for (size_t i = 0; i < n; ++i) {
        out[2 * i] = gr::uwb::core::clip_i16(
            static_cast<double>(in[i].real()) * scale, &clips);
        out[2 * i + 1] = gr::uwb::core::clip_i16(
            static_cast<double>(in[i].imag()) * scale, &clips);
    }
    return scale;
}

bool want_window(const DumpWindowMeta& w,
                 const DemodResultMeta* d,
                 const Options& o)
{
    if (o.select == "all")
        return true;
    if (o.select == "scheduled")
        return w.capture_mode == "scheduled" || w.capture_mode == "provisional";
    if (o.select == "fcs_pass")
        return d != nullptr && d->fcs_pass;
    return true;
}

const DumpWindowMeta* pick_probe(const std::vector<DumpWindowMeta>& wins)
{
    const DumpWindowMeta* fallback = nullptr;
    for (const auto& w : wins) {
        if (w.sample_count < 10000)
            continue;
        if (!fallback)
            fallback = &w;
        if (w.capture_mode == "scheduled" || w.capture_mode == "provisional")
            return &w;
    }
    return fallback;
}

} // namespace

int main(int argc, char** argv)
{
    Options opt;
    try {
        if (!parse_args(argc, argv, opt))
            return 1;
    } catch (const std::exception& e) {
        std::cerr << e.what() << "\n";
        return 1;
    }

    const std::string iq_path = join_path(opt.dir, "capture.iq");
    const std::string jsonl_path = join_path(opt.dir, "capture.jsonl");
    const std::string demod_path = join_path(opt.dir, "demod_results.jsonl");
    if (!file_ok(iq_path) || !file_ok(jsonl_path)) {
        std::cerr << "need " << iq_path << " and " << jsonl_path << "\n";
        return 1;
    }

    std::vector<DumpWindowMeta> windows;
    std::vector<DemodResultMeta> demod;
    try {
        windows = gr::uwb::core::load_capture_jsonl(jsonl_path);
        if (file_ok(demod_path))
            demod = gr::uwb::core::load_demod_jsonl(demod_path);
    } catch (const std::exception& e) {
        std::cerr << e.what() << "\n";
        return 1;
    }
    if (windows.empty()) {
        std::cerr << "empty capture.jsonl\n";
        return 1;
    }
    if (opt.select == "fcs_pass" && demod.empty()) {
        std::cerr << "--select fcs_pass requires " << demod_path << "\n";
        return 1;
    }

    std::map<int64_t, const DemodResultMeta*> demod_by_id;
    for (const auto& d : demod) {
        if (d.packet_id >= 0)
            demod_by_id[d.packet_id] = &d;
    }

    std::vector<size_t> selected;
    selected.reserve(windows.size());
    for (size_t i = 0; i < windows.size(); ++i) {
        const DemodResultMeta* d = nullptr;
        auto it = demod_by_id.find(windows[i].packet_id);
        if (it != demod_by_id.end())
            d = it->second;
        if (want_window(windows[i], d, opt))
            selected.push_back(i);
    }

    std::string taps_path;
    std::vector<float> taps;
    try {
        taps_path = find_taps_file(opt.taps);
        taps = load_taps_f32(taps_path);
    } catch (const std::exception& e) {
        std::cerr << e.what() << "\n";
        return 1;
    }

    const WindowCropGeom demod_geom{
        static_cast<int64_t>(kNativeScheduledPreGuard),
        static_cast<int64_t>(kNativeScheduledCapture),
        static_cast<int64_t>(kNativeScheduledPostGuard)
    };

    std::cout << "dump: " << opt.dir << "\n";
    std::cout << "windows=" << windows.size() << " selected=" << selected.size()
              << " select=" << opt.select << " emit=" << opt.emit << "\n";
    std::cout << "taps=" << taps_path << " T=" << taps.size() << "\n";
    std::cout << "skip_notch=" << (opt.skip_notch ? "true" : "false")
              << " out=" << opt.out_format << "\n";

    if (opt.dry_run) {
        std::cout << "dry-run: not writing output files\n";
        return 0;
    }

    std::ifstream iq(iq_path, std::ios::binary);
    if (!iq) {
        std::cerr << "cannot open " << iq_path << "\n";
        return 1;
    }

    ToneEstimate tone;
    if (!opt.skip_notch) {
        const DumpWindowMeta* probe = pick_probe(windows);
        if (!probe) {
            std::cerr << "no usable window for tone estimate\n";
            return 2;
        }
        std::vector<int16_t> raw;
        if (!gr::uwb::core::read_sc16_window(
                iq, probe->file_offset_samples,
                static_cast<size_t>(probe->sample_count), raw)) {
            std::cerr << "short IQ reading probe packet_id=" << probe->packet_id
                      << "\n";
            return 2;
        }
        std::vector<std::complex<double>> x(
            static_cast<size_t>(probe->sample_count));
        for (size_t i = 0; i < x.size(); ++i) {
            x[i] = std::complex<double>(static_cast<double>(raw[2 * i]),
                                        static_cast<double>(raw[2 * i + 1]));
        }
        if (opt.tone_rf_hz != 0.0) {
            const double f0 = opt.tone_rf_hz - opt.center_hz;
            const double span =
                std::max(200.0, kFs737 / static_cast<double>(x.size()) * 8.0);
            tone.coarse_hz = f0;
            tone.baseband_hz =
                gr::uwb::core::refine_tone_freq(x.data(), x.size(), kFs737, f0,
                                                span);
        } else {
            const double hint = kDefaultToneRfHz - opt.center_hz;
            const double lo = hint - opt.tone_search_hz;
            const double hi = hint + opt.tone_search_hz;
            tone = gr::uwb::core::estimate_tone(x.data(), x.size(), kFs737, lo,
                                                hi);
        }
        std::cout << "probe packet_id=" << probe->packet_id
                  << " n=" << probe->sample_count << "\n";
        std::cout << "tone baseband=" << tone.baseband_hz
                  << " Hz  RF=" << (opt.center_hz + tone.baseband_hz) / 1e6
                  << " MHz\n";
    }

    const std::string out_iq =
        join_path(opt.dir,
                  opt.out_format == "sc16" ? "capture_998p4.iq"
                                           : "capture_998p4.cf32");
    const std::string out_jsonl = join_path(opt.dir, "capture_998p4.jsonl");
    const std::string notch_path = join_path(opt.dir, "capture_notch.iq");
    std::ofstream out(out_iq, std::ios::binary | std::ios::trunc);
    std::ofstream out_meta(out_jsonl, std::ios::trunc);
    std::ofstream notch_out;
    if (opt.keep_native_notch && !opt.skip_notch)
        notch_out.open(notch_path, std::ios::binary | std::ios::trunc);
    if (!out || !out_meta) {
        std::cerr << "cannot write output in " << opt.dir << "\n";
        return 2;
    }

    RationalResampler65_48Core core(taps);
    std::vector<int16_t> raw;
    std::vector<int16_t> notched;
    std::vector<std::complex<float>> cf;
    std::vector<std::complex<float>> rs;
    std::vector<int16_t> sc16_out;
    uint64_t out_off = 0;
    uint64_t total_clip = 0;
    double power_sum = 0.0;
    double bin_sum = 0.0;
    size_t notch_n = 0;

    for (size_t idx : selected) {
        const DumpWindowMeta& w = windows[idx];
        const DemodResultMeta* d = nullptr;
        auto it = demod_by_id.find(w.packet_id);
        if (it != demod_by_id.end())
            d = it->second;

        if (!gr::uwb::core::read_sc16_window(
                iq, w.file_offset_samples, static_cast<size_t>(w.sample_count),
                raw)) {
            std::cerr << "short IQ packet_id=" << w.packet_id << "\n";
            return 2;
        }

        const int16_t* src = raw.data();
        size_t n_native = static_cast<size_t>(w.sample_count);
        int64_t native_ws = w.window_start_sample;
        int64_t native_pred = w.predicted_start_sample;
        if (d && d->native_predicted_start >= 0)
            native_pred = d->native_predicted_start;
        int64_t pre = w.pre_guard_samples;
        int64_t cap = w.capture_samples;
        int64_t post = w.post_guard_samples;
        ToneSubtractResult sub;

        if (!opt.skip_notch) {
            notched.resize(raw.size());
            sub = gr::uwb::core::subtract_tone_sc16(
                notched.data(), raw.data(), n_native, kFs737, tone.baseband_hz);
            src = notched.data();
            total_clip += sub.clip_count;
            if (sub.power_after > 0.0)
                power_sum += 10.0 * std::log10(sub.power_before / sub.power_after);
            if (sub.bin_after > 0.0)
                bin_sum += 20.0 * std::log10((sub.bin_before + 1e-18) /
                                             (sub.bin_after + 1e-18));
            ++notch_n;
            if (notch_out.is_open())
                gr::uwb::core::write_sc16_window(notch_out, src, n_native);
        }

        if (opt.emit == "qm35") {
            if (native_pred < 0) {
                std::cerr << "emit qm35 needs predicted_start packet_id="
                          << w.packet_id << "\n";
                return 2;
            }
            const bool acq = (w.capture_mode == "acquisition");
            const auto plan = gr::uwb::core::plan_window_crop(
                native_ws, static_cast<int64_t>(n_native), native_pred,
                demod_geom, acq);
            if (plan.empty) {
                std::cerr << "empty qm35 crop packet_id=" << w.packet_id << "\n";
                return 2;
            }
            if (!plan.passthrough) {
                const size_t off = static_cast<size_t>(plan.in_offset);
                const size_t nn = static_cast<size_t>(plan.out_count);
                if (src == raw.data()) {
                    notched.assign(raw.begin() + static_cast<std::ptrdiff_t>(off * 2),
                                   raw.begin() + static_cast<std::ptrdiff_t>(
                                                     (off + nn) * 2));
                    src = notched.data();
                } else {
                    std::memmove(notched.data(),
                                 notched.data() + off * 2,
                                 nn * 2 * sizeof(int16_t));
                    notched.resize(nn * 2);
                    src = notched.data();
                }
                n_native = nn;
                native_ws = plan.window_start;
                pre = plan.pre;
                cap = plan.capture;
                post = plan.post;
            }
        }

        cf.resize(n_native);
        gr::uwb::core::sc16_to_cf32(src, n_native, cf.data());
        resample_oneshot(core, taps, cf.data(), n_native, rs);

        const int64_t ws_out = map_native(native_ws, taps.size());
        int64_t pre_out = 0;
        int64_t cap_out = static_cast<int64_t>(rs.size());
        int64_t post_out = 0;
        if (pre >= 0 && cap >= 0) {
            pre_out = map_native(native_ws + pre, taps.size()) - ws_out;
            cap_out = map_native(native_ws + pre + cap, taps.size()) -
                      map_native(native_ws + pre, taps.size());
            if (post >= 0) {
                post_out = map_native(native_ws + pre + cap + post,
                                      taps.size()) -
                           map_native(native_ws + pre + cap, taps.size());
            }
            if (pre_out < 0)
                pre_out = 0;
            if (cap_out < 0)
                cap_out = 0;
            if (post_out < 0)
                post_out = 0;
            const int64_t remain = static_cast<int64_t>(rs.size()) - pre_out - cap_out;
            if (post < 0 || post_out > remain)
                post_out = remain > 0 ? remain : 0;
        }
        const int64_t pred_out =
            (native_pred >= 0) ? map_native(native_pred, taps.size()) : -1;

        float iq_scale = 1.0f;
        if (opt.out_format == "sc16") {
            iq_scale = quantize_cf32_to_sc16(rs.data(), rs.size(), sc16_out);
            gr::uwb::core::write_sc16_window(out, sc16_out.data(), rs.size());
        } else {
            gr::uwb::core::write_cf32_window(out, rs.data(), rs.size());
        }

        std::vector<JsonField> fields = {
            { "packet_id", gr::uwb::core::json_i64(w.packet_id) },
            { "schedule_index", gr::uwb::core::json_i64(w.schedule_index) },
            { "sample_rate", gr::uwb::core::json_i64(998400000) },
            { "sample_format", gr::uwb::core::json_str(opt.out_format) },
            { "file_offset_samples",
              gr::uwb::core::json_u64(out_off) },
            { "sample_count",
              gr::uwb::core::json_i64(static_cast<int64_t>(rs.size())) },
            { "window_start_sample", gr::uwb::core::json_i64(ws_out) },
            { "predicted_start_sample", gr::uwb::core::json_i64(pred_out) },
            { "pre_guard_samples", gr::uwb::core::json_i64(pre_out) },
            { "capture_samples", gr::uwb::core::json_i64(cap_out) },
            { "post_guard_samples", gr::uwb::core::json_i64(post_out) },
            { "input_native_sample_count",
              gr::uwb::core::json_i64(static_cast<int64_t>(n_native)) },
        };
        if (w.capture_mode.size())
            fields.push_back(
                { "capture_mode", gr::uwb::core::json_str(w.capture_mode) });
        if (d && d->has_detected_start)
            fields.push_back({ "detected_start_sample",
                               gr::uwb::core::json_i64(d->detected_start_sample) });
        if (!opt.skip_notch) {
            fields.push_back({ "tone_baseband_hz",
                               gr::uwb::core::json_f64(tone.baseband_hz) });
            fields.push_back(
                { "tone_coef_re", gr::uwb::core::json_f64(sub.coef.real()) });
            fields.push_back(
                { "tone_coef_im", gr::uwb::core::json_f64(sub.coef.imag()) });
        }
        if (opt.out_format == "sc16")
            fields.push_back(
                { "iq_scale", gr::uwb::core::json_f64(iq_scale) });
        gr::uwb::core::write_json_object(out_meta, fields);
        out_off += rs.size();
    }

    out.flush();
    out_meta.flush();
    if (notch_out.is_open())
        notch_out.flush();

    std::cout << "wrote " << out_iq << " windows=" << selected.size()
              << " samples=" << out_off << "\n";
    std::cout << "wrote " << out_jsonl << "\n";
    if (notch_n) {
        std::cout << "tone_bin_db mean=" << (bin_sum / static_cast<double>(notch_n))
                  << "  power_suppression_db mean="
                  << (power_sum / static_cast<double>(notch_n))
                  << "  clips=" << total_clip << "\n";
    }
    if (opt.keep_native_notch && notch_out.is_open())
        std::cout << "wrote " << notch_path << "\n";
    return 0;
}
