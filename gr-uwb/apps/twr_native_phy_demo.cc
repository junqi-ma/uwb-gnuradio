/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * M2-A offline native-rate PHY demo CLI (G0 §10, task §7 A12/A13).
 *
 * ---------------------------------------------------------------------------
 * WHAT THIS IS
 * ---------------------------------------------------------------------------
 * A self-contained, radio-free command line program that runs ONE real TWR
 * frame (Poll / Response / Final) through the frozen M2-A closed loop
 * `gr::uwb::twr::m2a_native_roundtrip()` and writes a `twr-m2a-native/1`
 * manifest (G0 §8).
 *
 * It links the static `uwb_twr_phy` archive and NOTHING from GNU Radio / UHD /
 * PMT.  The frame bytes come from the real `uwb_twr_frame.h` codec; the FCS is
 * appended by the HRP layer inside the closed loop; the two frozen tap tables
 * are loaded from disk and their sha256 recorded.  No stage re-implements a
 * core and no impairment is injected into the decoder's input as "truth".
 *
 * ---------------------------------------------------------------------------
 * EVIDENCE BOUNDARY (G0 §9)
 * ---------------------------------------------------------------------------
 * `measurement_valid` is ALWAYS false and `hardware_readback` is ALWAYS null.
 * `evidence.level` is `native_roundtrip_verified` ONLY when the closed loop
 * actually succeeded (byte-exact decode on the native grid for THIS
 * rate/profile/taps/format); otherwise it is `work_decode_verified` /
 * `not_measured`.  Nothing here produces an RMARKER, a first path, a ToA or a
 * distance.  `allows(NativeRoundtripVerified, Ranging)` stays false.
 *
 * ---------------------------------------------------------------------------
 * CLI (G0 §10)
 * ---------------------------------------------------------------------------
 *   twr_native_phy_demo --frame poll|response|final
 *                       --native-rate 737280000|491520000
 *                       --iq cf32|sc16
 *                       [--seed N] [--scenario clean]
 *                       [--testdata DIR] [--repo DIR]
 *                       [--native-iq PATH] [--no-native-iq]
 *                       --output PATH.json
 *
 * `--scenario` accepts `clean` only: the frozen helper exposes no impairment
 * model, so any other scenario is refused explicitly rather than silently
 * treated as clean.  `--output -` writes the JSON to stdout and suppresses the
 * native IQ artifact.
 *
 * Exit 0 iff the closed loop succeeded (bytes byte-exact).  Non-zero otherwise,
 * with the reason on stderr.
 *
 * ---------------------------------------------------------------------------
 * BUILD (CMake, reported to the coordinator)
 * ---------------------------------------------------------------------------
 *   add_executable(twr_native_phy_demo twr_native_phy_demo.cc)
 *   target_include_directories(twr_native_phy_demo PRIVATE
 *       ${CMAKE_CURRENT_SOURCE_DIR}/../include)
 *   target_link_libraries(twr_native_phy_demo PRIVATE
 *       uwb_twr_phy Volk::volk Threads::Threads)
 *   target_compile_definitions(twr_native_phy_demo PRIVATE
 *       "UWB_TESTDATA_DIR=\"${CMAKE_SOURCE_DIR}/../testdata\"")
 */

#include <gnuradio/uwb/uwb_twr_capability_evidence.h>
#include <gnuradio/uwb/uwb_twr_frame.h>
#include <gnuradio/uwb/uwb_twr_phy.h>

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <complex>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <unistd.h>

#ifndef UWB_TESTDATA_DIR
#define UWB_TESTDATA_DIR ""
#endif

namespace twr = gr::uwb::twr;
namespace evidence = gr::uwb::twr::evidence;
using gr_complex = std::complex<float>;

namespace {

// ===========================================================================
// Small total helpers
// ===========================================================================

std::string json_escape(const std::string& s)
{
    std::string out;
    out.reserve(s.size() + 8);
    for (char c : s) {
        switch (c) {
        case '"':
            out += "\\\"";
            break;
        case '\\':
            out += "\\\\";
            break;
        case '\n':
            out += "\\n";
            break;
        case '\r':
            out += "\\r";
            break;
        case '\t':
            out += "\\t";
            break;
        default:
            if (static_cast<unsigned char>(c) < 0x20) {
                char buf[8];
                std::snprintf(buf, sizeof(buf), "\\u%04x", c);
                out += buf;
            } else {
                out += c;
            }
        }
    }
    return out;
}

std::string jstr(const std::string& s) { return "\"" + json_escape(s) + "\""; }

std::string jbool(bool b) { return b ? "true" : "false"; }

std::string jint(int64_t v) { return std::to_string(v); }

std::string juint(uint64_t v) { return std::to_string(v); }

// Finite doubles only; anything else is `null` so a non-finite never becomes
// an invalid JSON token.
std::string jnum(double v)
{
    if (!std::isfinite(v))
        return "null";
    std::ostringstream os;
    os << std::setprecision(17) << v;
    std::string s = os.str();
    if (s.find('.') == std::string::npos && s.find('e') == std::string::npos &&
        s.find('E') == std::string::npos)
        s += ".0";
    return s;
}

std::string hex_bytes(const std::vector<uint8_t>& v)
{
    static const char* kHex = "0123456789abcdef";
    std::string s;
    s.reserve(v.size() * 2);
    for (uint8_t b : v) {
        s.push_back(kHex[(b >> 4) & 0xf]);
        s.push_back(kHex[b & 0xf]);
    }
    return s;
}

std::string hex4(uint16_t v)
{
    static const char* kHex = "0123456789abcdef";
    std::string s = "0x";
    for (int shift = 12; shift >= 0; shift -= 4)
        s.push_back(kHex[(v >> shift) & 0xf]);
    return s;
}

std::string hex4_plain(uint16_t v)
{
    static const char* kHex = "0123456789abcdef";
    std::string s;
    for (int shift = 12; shift >= 0; shift -= 4)
        s.push_back(kHex[(v >> shift) & 0xf]);
    return s;
}

std::string readlink_exe()
{
    char buf[4096];
    const ssize_t n = ::readlink("/proc/self/exe", buf, sizeof(buf) - 1);
    if (n <= 0)
        return std::string();
    buf[n] = '\0';
    return std::string(buf);
}

// ===========================================================================
// Options
// ===========================================================================

struct Options {
    std::string frame = "poll";
    uint64_t native_rate = 737280000;
    std::string iq = "cf32";
    uint64_t seed = 0;
    std::string scenario = "clean";
    std::string testdata;
    std::string repo;
    std::string native_iq;
    bool no_native_iq = false;
    std::string output;
};

void usage(std::ostream& os)
{
    os << "usage: twr_native_phy_demo --frame poll|response|final\n"
          "                           --native-rate 737280000|491520000\n"
          "                           --iq cf32|sc16\n"
          "                           [--seed N] [--scenario clean]\n"
          "                           [--testdata DIR] [--repo DIR]\n"
          "                           [--native-iq PATH] [--no-native-iq]\n"
          "                           --output PATH.json\n";
}

bool parse_u64(const std::string& text, uint64_t& out)
{
    if (text.empty())
        return false;
    errno = 0;
    char* end = nullptr;
    const unsigned long long v = std::strtoull(text.c_str(), &end, 10);
    if (errno != 0 || end == nullptr || *end != '\0')
        return false;
    out = static_cast<uint64_t>(v);
    return true;
}

bool parse_args(int argc, char** argv, Options& o, std::string& why)
{
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        auto need = [&](std::string& dst) -> bool {
            if (i + 1 >= argc) {
                why = "missing value for " + a;
                return false;
            }
            dst = argv[++i];
            return true;
        };
        if (a == "--frame") {
            if (!need(o.frame))
                return false;
        } else if (a == "--native-rate") {
            std::string v;
            if (!need(v))
                return false;
            if (!parse_u64(v, o.native_rate)) {
                why = "--native-rate must be an integer, got " + v;
                return false;
            }
        } else if (a == "--iq") {
            if (!need(o.iq))
                return false;
        } else if (a == "--seed") {
            std::string v;
            if (!need(v))
                return false;
            if (!parse_u64(v, o.seed)) {
                why = "--seed must be a non-negative integer, got " + v;
                return false;
            }
        } else if (a == "--scenario") {
            if (!need(o.scenario))
                return false;
        } else if (a == "--testdata") {
            if (!need(o.testdata))
                return false;
        } else if (a == "--repo") {
            if (!need(o.repo))
                return false;
        } else if (a == "--native-iq") {
            if (!need(o.native_iq))
                return false;
        } else if (a == "--no-native-iq") {
            o.no_native_iq = true;
        } else if (a == "--output") {
            if (!need(o.output))
                return false;
        } else if (a == "-h" || a == "--help") {
            usage(std::cout);
            std::exit(0);
        } else {
            why = "unknown argument \"" + a + "\"";
            return false;
        }
    }
    if (o.output.empty()) {
        why = "--output PATH.json is required";
        return false;
    }
    return true;
}

// ===========================================================================
// Tap loading (float32 LE, raw bytes).  The M2-A config carries these.
// ===========================================================================

std::vector<std::string> testdata_candidates(const std::string& explicit_dir)
{
    std::vector<std::string> cands;
    if (!explicit_dir.empty())
        cands.push_back(explicit_dir);
    const char* env = std::getenv("UWB_TESTDATA_DIR");
    if (env && *env)
        cands.push_back(env);
    const std::string compiled = UWB_TESTDATA_DIR;
    if (!compiled.empty())
        cands.push_back(compiled);
    for (const char* p : { "", "../", "../../", "../../../", "../../../../" })
        cands.push_back(std::string(p) + "testdata");
    return cands;
}

bool load_f32(const std::string& path, std::vector<float>& out, std::string& why)
{
    std::ifstream f(path, std::ios::binary | std::ios::ate);
    if (!f) {
        why = "cannot open " + path;
        return false;
    }
    const auto bytes = static_cast<size_t>(f.tellg());
    if (bytes % sizeof(float) != 0) {
        why = path + " is not a whole number of float32 samples";
        return false;
    }
    f.seekg(0);
    out.resize(bytes / sizeof(float));
    if (bytes > 0)
        f.read(reinterpret_cast<char*>(out.data()), static_cast<std::streamsize>(bytes));
    if (!f) {
        why = "short read on " + path;
        return false;
    }
    return true;
}

std::string join_path(const std::string& dir, const std::string& rel)
{
    if (dir.empty())
        return rel;
    if (dir.back() == '/')
        return dir + rel;
    return dir + "/" + rel;
}

bool load_taps(const Options& o, twr::M2aNativeRate rate, std::vector<float>& tx,
               std::vector<float>& rx, std::string& tx_path, std::string& rx_path,
               std::string& why)
{
    const bool uc200 = rate == twr::M2aNativeRate::Uc200_737280000;
    const std::string tx_rel = uc200 ? "twr/m2a/taps/tx_48_65.f32"
                                     : "twr/m2a/taps/tx_32_65.f32";
    const std::string rx_rel = uc200 ? "resampler_65_48/taps_quality_minorder.txt"
                                     : "resampler_65_32/taps_quality_minorder.txt";
    std::string last;
    for (const auto& base : testdata_candidates(o.testdata)) {
        const std::string tp = join_path(base, tx_rel);
        const std::string rp = join_path(base, rx_rel);
        std::string e1;
        if (load_f32(tp, tx, e1)) {
            std::string e2;
            if (load_f32(rp, rx, e2)) {
                tx_path = tp;
                rx_path = rp;
                return true;
            }
            last = e2;
        } else {
            last = e1;
        }
    }
    why = "cannot locate the frozen taps (last error: " + last + ")";
    return false;
}

// ===========================================================================
// Frame construction (real codec input; deterministic in `seed`)
// ===========================================================================

twr::Frame make_frame(twr::FrameType t, uint64_t seed)
{
    twr::Frame f;
    f.version = twr::kFrameVersion;
    f.function_code = t;
    f.session_id = static_cast<uint16_t>(0x1000 + (seed & 0xff));
    f.seq = static_cast<uint16_t>(0x0042 + 7 * (seed & 0x1ff));
    f.pan_id = 0x1234;
    f.src_addr = static_cast<uint16_t>(0x0005 + (seed & 0x0f));
    f.dst_addr = static_cast<uint16_t>(0x0007 + (seed & 0x0f));
    f.flags = twr::make_flags(true);
    switch (t) {
    case twr::FrameType::Poll:
        break;
    case twr::FrameType::Response:
        f.set(twr::TimestampField::T2B, 0x1122334455ULL + seed);
        f.set(twr::TimestampField::T3B, 0x2233445566ULL + seed);
        break;
    case twr::FrameType::Final:
        f.set(twr::TimestampField::T1A, 0x00000000A1ULL + seed);
        f.set(twr::TimestampField::T4A, 0x00000000A2ULL + seed);
        f.set(twr::TimestampField::T5A, 0x00000000A3ULL + seed);
        break;
    default:
        break;
    }
    return f;
}

const char* demod_status_name(gr::uwb::demod::DemodStatus s)
{
    using gr::uwb::demod::DemodStatus;
    switch (s) {
    case DemodStatus::Success:
        return "success";
    case DemodStatus::InvalidInput:
        return "invalid_input";
    case DemodStatus::TimingFailed:
        return "timing_failed";
    case DemodStatus::CfoFailed:
        return "cfo_failed";
    case DemodStatus::SfdFailed:
        return "sfd_failed";
    case DemodStatus::PhrFailed:
        return "phr_failed";
    case DemodStatus::PayloadFailed:
        return "payload_failed";
    case DemodStatus::FcsFailed:
        return "fcs_failed";
    case DemodStatus::QueueFull:
        return "queue_full";
    case DemodStatus::InternalError:
        return "internal_error";
    case DemodStatus::CirFailed:
        return "cir_failed";
    }
    return "invalid";
}

// The canonical effective-config text.  INTEGER/STRING values only, so C++ and
// an independent Python verifier agree bit-for-bit without float formatting
// ambiguity.  `config.executed_sha256` is sha256() of exactly this text.
std::string effective_canonical(const twr::M2aConfig& cfg, const std::string& frame_name,
                                uint32_t l, uint32_t m)
{
    const int64_t peak_micro = static_cast<int64_t>(std::llround(
        static_cast<double>(cfg.peak_amplitude) * 1.0e6));
    const int64_t scale_micro = static_cast<int64_t>(std::llround(
        static_cast<double>(cfg.effective_sc16_scale()) * 1.0e6));
    std::ostringstream os;
    os << "frame=" << frame_name << "\n"
       << "native_rate=" << static_cast<uint64_t>(twr::m2a_rate_hz(cfg.native_rate)) << "\n"
       << "tx_l=" << l << "\n"
       << "tx_m=" << m << "\n"
       << "rx_l=65\n"
       << "rx_m=" << l << "\n"
       << "iq_format=" << twr::m2a_iq_format_to_string(cfg.iq_format) << "\n"
       << "code_index=" << cfg.code_index << "\n"
       << "sync_repetitions=" << cfg.sync_repetitions << "\n"
       << "sfd_mode=" << cfg.sfd_mode << "\n"
       << "insert_sts=" << (cfg.insert_sts ? "true" : "false") << "\n"
       << "ranging=" << (cfg.ranging ? "true" : "false") << "\n"
       << "peak_amplitude_micro=" << peak_micro << "\n"
       << "sc16_scale_micro=" << scale_micro << "\n"
       << "tx_taps=" << cfg.tx_taps.size() << "\n"
       << "rx_taps=" << cfg.rx_taps.size() << "\n"
       << "timestamp_bits=40\n";
    return os.str();
}

// ===========================================================================
// JSON emission
// ===========================================================================

std::string stage_json(const twr::M2aStageTrace& s)
{
    std::ostringstream os;
    os << "{\"name\":" << jstr(s.name) << ",\"rate_hz\":" << jnum(s.rate_hz)
       << ",\"l\":" << juint(s.interp) << ",\"m\":" << juint(s.decim)
       << ",\"origin\":" << jint(s.origin) << ",\"in_count\":" << juint(s.in_count)
       << ",\"out_count\":" << juint(s.out_count) << ",\"phase\":" << juint(s.phase)
       << ",\"trim\":" << jint(s.trim) << ",\"padding\":" << jint(s.padding)
       << ",\"filter_delay\":" << jnum(s.filter_delay)
       << ",\"valid_from\":" << juint(s.valid_from)
       << ",\"valid_to\":" << juint(s.valid_to) << "}";
    return os.str();
}

std::string frame_fields_json(const twr::Frame& f)
{
    std::ostringstream os;
    os << "{\"version\":" << static_cast<int>(f.version)
       << ",\"function_code\":" << static_cast<int>(f.function_code)
       << ",\"session_id\":" << juint(f.session_id) << ",\"seq\":" << juint(f.seq)
       << ",\"pan_id\":" << juint(f.pan_id) << ",\"src_addr\":" << juint(f.src_addr)
       << ",\"dst_addr\":" << juint(f.dst_addr) << ",\"flags\":" << juint(f.flags)
       << ",\"timestamps\":[";
    const size_t n = f.timestamp_count();
    for (size_t i = 0; i < n; ++i) {
        if (i)
            os << ",";
        os << juint(f.timestamps[i]);
    }
    os << "]}";
    return os.str();
}

struct DemoResult {
    bool ok = false;
    std::string why;
    std::string json;
};

} // namespace

// ===========================================================================
// main
// ===========================================================================

int main(int argc, char** argv)
{
    Options o;
    std::string why;
    if (!parse_args(argc, argv, o, why)) {
        std::cerr << "error: " << why << "\n";
        usage(std::cerr);
        return 2;
    }

    // ---- parse/validate enums fail-closed (domain test before switch) -----
    twr::FrameType frame_type = twr::FrameType::Poll;
    if (!twr::frame_type_from_string(o.frame, frame_type) ||
        frame_type == twr::FrameType::Report) {
        std::cerr << "error: --frame must be poll|response|final, got \"" << o.frame
                  << "\"\n";
        return 2;
    }
    twr::M2aNativeRate rate;
    if (o.native_rate == 737280000ULL)
        rate = twr::M2aNativeRate::Uc200_737280000;
    else if (o.native_rate == 491520000ULL)
        rate = twr::M2aNativeRate::Cg400_491520000;
    else {
        std::cerr << "error: --native-rate must be 737280000 or 491520000, got "
                  << o.native_rate << "\n";
        return 2;
    }
    twr::M2aIqFormat fmt;
    if (o.iq == "cf32")
        fmt = twr::M2aIqFormat::Cf32;
    else if (o.iq == "sc16")
        fmt = twr::M2aIqFormat::Sc16;
    else {
        std::cerr << "error: --iq must be cf32|sc16, got \"" << o.iq << "\"\n";
        return 2;
    }
    if (o.scenario != "clean") {
        std::cerr << "error: --scenario \"" << o.scenario
                  << "\" is not implemented: the frozen M2-A helper exposes no "
                     "impairment model, so a non-clean scenario cannot be run "
                     "honestly (only \"clean\" is accepted)\n";
        return 2;
    }

    // ---- frozen config ----------------------------------------------------
    twr::M2aConfig cfg;
    cfg.native_rate = rate;
    cfg.iq_format = fmt;
    cfg.code_index = 9;
    cfg.sync_repetitions = 64;
    cfg.sfd_mode = "ieee";
    cfg.insert_sts = false;
    cfg.ranging = true;
    cfg.peak_amplitude = 0.8f;

    std::string tx_path, rx_path;
    if (!load_taps(o, rate, cfg.tx_taps, cfg.rx_taps, tx_path, rx_path, why)) {
        std::cerr << "error: " << why << "\n";
        return 2;
    }
    if (!cfg.is_valid(why)) {
        std::cerr << "error: frozen config is invalid: " << why << "\n";
        return 2;
    }

    uint32_t l = 0, m = 0;
    if (!twr::m2a_rate_tx_lm(rate, l, m)) {
        std::cerr << "error: no TX ratio for this native rate\n";
        return 2;
    }

    // ---- build the real frame via the codec -------------------------------
    const twr::Frame frame = make_frame(frame_type, o.seed);
    const twr::FrameProfile prof;
    std::vector<uint8_t> mac;
    std::string ferr;
    if (!twr::encode(frame, prof, mac, ferr)) {
        std::cerr << "error: codec refused the frame: " << ferr << "\n";
        return 2;
    }

    // ---- run the closed loop ---------------------------------------------
    twr::M2aResult res;
    std::string rwhy;
    const bool ok = twr::m2a_native_roundtrip(frame, prof, cfg, res, rwhy);

    // ---- provenance -------------------------------------------------------
    std::string repo = o.repo;
    if (repo.empty()) {
        const char* env = std::getenv("UWB_REPO_DIR");
        if (env && *env)
            repo = env;
    }
    if (repo.empty())
        repo = evidence::capture_command("git rev-parse --show-toplevel");
    std::string revision;
    bool dirty = false;
    if (!repo.empty()) {
        revision = evidence::capture_command("git -C '" + repo + "' rev-parse HEAD");
        const std::string status =
            evidence::capture_command("git -C '" + repo + "' status --porcelain");
        dirty = !status.empty();
    }
    const std::string compiler = evidence::compiler_string();
    const std::string volk = evidence::volk_setting();
    const std::string loaded_library = readlink_exe();

    // Taps hashes (raw float32 LE bytes).
    evidence::Sha256 htx;
    htx.update(cfg.tx_taps.data(), cfg.tx_taps.size() * sizeof(float));
    const std::string tx_hash = htx.hex();
    evidence::Sha256 hrx;
    hrx.update(cfg.rx_taps.data(), cfg.rx_taps.size() * sizeof(float));
    const std::string rx_hash = hrx.hex();
    evidence::Sha256 hboth;
    hboth.update(cfg.tx_taps.data(), cfg.tx_taps.size() * sizeof(float));
    hboth.update(cfg.rx_taps.data(), cfg.rx_taps.size() * sizeof(float));
    const std::string taps_hash = hboth.hex();

    // Input hash = the exact MAC payload bytes the codec produced.
    evidence::Sha256 hin;
    hin.update(mac.data(), mac.size());
    const std::string input_hash = hin.hex();
    // Output hash = the decoded payload bytes the closed loop returned.
    evidence::Sha256 hout;
    hout.update(res.decoded_bytes.data(), res.decoded_bytes.size());
    const std::string output_hash = hout.hex();

    const std::string effective_text = effective_canonical(cfg, o.frame, l, m);
    evidence::Sha256 hcfg;
    hcfg.update(effective_text);
    const std::string executed_hash = hcfg.hex();

    // ---- optional native IQ artifact (same public stages, for MATLAB #2) --
    std::string native_iq_path;
    bool wrote_iq = false;
    if (!o.no_native_iq && o.output != "-") {
        std::vector<gr_complex> work_tx, native;
        twr::M2aStageTrace t;
        std::string iqwhy;
        if (twr::m2a_modulate_to_work(mac.data(), mac.size(), cfg, work_tx, t, iqwhy) &&
            twr::m2a_tx_resample(work_tx.data(), work_tx.size(), cfg, native, t, iqwhy)) {
            if (fmt == twr::M2aIqFormat::Sc16) {
                std::vector<gr_complex> q;
                size_t sat = 0;
                if (!twr::m2a_sc16_roundtrip(native.data(), native.size(),
                                             cfg.effective_sc16_scale(), q, sat, iqwhy)) {
                    native.clear();
                } else {
                    native.swap(q);
                }
            }
        }
        if (!native.empty()) {
            native_iq_path = o.native_iq;
            if (native_iq_path.empty()) {
                native_iq_path = o.output;
                const size_t dot = native_iq_path.rfind(".json");
                if (dot != std::string::npos && dot + 5 == native_iq_path.size())
                    native_iq_path = native_iq_path.substr(0, dot);
                native_iq_path += ".cf32";
            }
            std::ofstream of(native_iq_path, std::ios::binary | std::ios::trunc);
            if (of) {
                of.write(reinterpret_cast<const char*>(native.data()),
                         static_cast<std::streamsize>(native.size() *
                                                      sizeof(gr_complex)));
                wrote_iq = of.good();
            }
            if (!wrote_iq)
                native_iq_path.clear();
        }
    }

    // ---- emit the manifest ------------------------------------------------
    std::ostringstream os;
    os << "{\n";
    os << "  \"schema\": \"twr-m2a-native/1\",\n";
    os << "  \"rate\": {\"work_hz\": " << jnum(cfg.tx_rate_hz())
       << ", \"native_hz\": " << jnum(cfg.native_rate_hz())
       << ", \"tx_l\": " << l << ", \"tx_m\": " << m << ", \"rx_l\": 65, \"rx_m\": " << l
       << ", \"tx_taps\": " << cfg.tx_taps.size() << ", \"rx_taps\": " << cfg.rx_taps.size()
       << ", \"taps_sha256\": " << jstr(taps_hash)
       << ", \"tx_taps_sha256\": " << jstr(tx_hash)
       << ", \"rx_taps_sha256\": " << jstr(rx_hash) << "},\n";
    os << "  \"frame\": {\"type\": " << jstr(o.frame) << ", \"mac_bytes\": " << mac.size()
       << ", \"psdu_bytes\": " << res.psdu_bytes.size()
       << ", \"mac_hex\": " << jstr(hex_bytes(mac))
       << ", \"fcs\": " << jstr(hex4(res.fcs))
       << ", \"fcs_hex\": " << jstr(hex4_plain(res.fcs))
       << ", \"fields\": " << frame_fields_json(frame) << "},\n";
    os << "  \"stages\": [";
    for (size_t i = 0; i < res.stages.size(); ++i) {
        if (i)
            os << ", ";
        os << stage_json(res.stages[i]);
    }
    os << "],\n";
    os << "  \"samples\": {\"work_tx\": " << res.work_tx_samples
       << ", \"native\": " << res.native_samples << ", \"work_rx\": " << res.work_rx_samples
       << ", \"returned\": " << res.decoded_bytes.size() << "},\n";
    os << "  \"filter\": {\"tx_taps\": " << cfg.tx_taps.size()
       << ", \"rx_taps\": " << cfg.rx_taps.size()
       << ", \"scale\": " << jnum(cfg.effective_sc16_scale())
       << ", \"phase\": 0, \"crop\": 0, \"group_delay\": "
       << jnum(0.5 * static_cast<double>(cfg.tx_taps.size() - 1))
       << ", \"valid_from\": 0, \"valid_to\": " << res.work_tx_samples << "},\n";
    os << "  \"decode\": {\"status\": " << jstr(demod_status_name(res.demod_status))
       << ", \"phr_psdu_length\": " << res.expected_psdu_length
       << ", \"payload_hex\": " << jstr(hex_bytes(res.decoded_bytes))
       << ", \"fcs_pass\": " << jbool(res.fcs_pass)
       << ", \"diagnostic\": {\"sfd_start_sample\": " << jint(res.sfd_start_sample)
       << ", \"packet_start_sample\": " << jint(res.packet_start_sample) << "}},\n";
    os << "  \"compare\": {\"bytes_exact\": " << jbool(res.bytes_exact)
       << ", \"max_abs_error\": " << jnum(res.max_abs_error)
       << ", \"relative_l2\": " << jnum(res.relative_l2) << "},\n";
    os << "  \"seed\": " << o.seed << ",\n";
    os << "  \"artifacts\": {\"native_cf32\": "
       << (wrote_iq ? jstr(native_iq_path) : "null") << "},\n";
    os << "  \"provenance\": {\"revision\": " << jstr(revision)
       << ", \"dirty\": " << jbool(dirty) << ", \"compiler\": " << jstr(compiler)
       << ", \"volk\": " << jstr(volk) << ", \"loaded_library\": " << jstr(loaded_library)
       << ", \"loaded_library_kind\": \"static_executable\""
       << ", \"input_sha256\": " << jstr(input_hash)
       << ", \"output_sha256\": " << jstr(output_hash)
       << ", \"matlab\": {\"executed\": false, \"version\": null, \"command\": null, "
          "\"exit_code\": null}},\n";

    const bool measured = ok && res.ok && res.bytes_exact &&
                          res.demod_status == gr::uwb::demod::DemodStatus::Success &&
                          res.fcs_pass;
    os << "  \"evidence\": {\"level\": "
       << jstr(measured ? "native_roundtrip_verified" : "work_decode_verified")
       << ", \"kind\": " << jstr(measured ? "measured" : "not_measured")
       << ", \"scope\": \"m2a-native-roundtrip/1\""
       << ", \"hardware_readback\": null, \"measurement_valid\": false},\n";

    os << "  \"config\": {\"requested\": {\"frame\": " << jstr(o.frame)
       << ", \"native_rate\": " << o.native_rate << ", \"iq\": " << jstr(o.iq)
       << ", \"seed\": " << o.seed << ", \"scenario\": " << jstr(o.scenario) << "}";
    os << ", \"effective\": {\"frame\": " << jstr(o.frame)
       << ", \"native_rate\": " << static_cast<uint64_t>(twr::m2a_rate_hz(cfg.native_rate))
       << ", \"tx_l\": " << l << ", \"tx_m\": " << m << ", \"rx_l\": 65, \"rx_m\": " << l
       << ", \"iq_format\": " << jstr(twr::m2a_iq_format_to_string(cfg.iq_format))
       << ", \"code_index\": " << cfg.code_index
       << ", \"sync_repetitions\": " << cfg.sync_repetitions
       << ", \"sfd_mode\": " << jstr(cfg.sfd_mode)
       << ", \"insert_sts\": " << jbool(cfg.insert_sts)
       << ", \"ranging\": " << jbool(cfg.ranging)
       << ", \"peak_amplitude_micro\": "
       << static_cast<int64_t>(std::llround(static_cast<double>(cfg.peak_amplitude) * 1.0e6))
       << ", \"sc16_scale_micro\": "
       << static_cast<int64_t>(std::llround(static_cast<double>(cfg.effective_sc16_scale()) * 1.0e6))
       << ", \"tx_taps\": " << cfg.tx_taps.size()
       << ", \"rx_taps\": " << cfg.rx_taps.size() << ", \"timestamp_bits\": 40}"
       << ", \"executed_sha256\": " << jstr(executed_hash) << "},\n";

    os << "  \"detail\": " << jstr(res.detail) << ",\n";
    os << "  \"status\": {\"ok\": " << jbool(res.ok) << ", \"exit_code\": " << (ok ? 0 : 1)
       << ", \"reason\": " << jstr(rwhy) << "}\n";
    os << "}\n";

    const std::string json = os.str();
    if (o.output == "-") {
        std::cout << json;
    } else {
        std::ofstream of(o.output, std::ios::binary | std::ios::trunc);
        if (!of) {
            std::cerr << "error: cannot write " << o.output << "\n";
            return 2;
        }
        of << json;
        if (!of.good()) {
            std::cerr << "error: failed while writing " << o.output << "\n";
            return 2;
        }
    }

    if (!ok) {
        std::cerr << "error: closed loop failed: status="
                  << twr::m2a_status_to_string(res.status) << " reason=" << rwhy << "\n";
        return 1;
    }
    std::cerr << "ok: frame=" << o.frame << " rate=" << o.native_rate << " iq=" << o.iq
              << " work_tx=" << res.work_tx_samples << " native=" << res.native_samples
              << " work_rx=" << res.work_rx_samples
              << " sc16_saturated=" << res.sc16_saturated << "\n";
    return 0;
}
