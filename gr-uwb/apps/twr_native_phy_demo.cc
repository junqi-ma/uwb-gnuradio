/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * M2-A offline native-rate PHY demo CLI (G0 §10, appendix A.3/A.5; task §5/§6).
 *
 * ---------------------------------------------------------------------------
 * WHAT THIS IS
 * ---------------------------------------------------------------------------
 * A self-contained, radio-free command line program that runs ONE real TWR
 * frame (Poll / Response / Final) through the frozen M2-A closed loop and
 * writes a `twr-m2a-native/2` manifest (G0 appendix A.5).
 *
 * The CLEAN path runs through the prepared `M2aContext` (G0 A.1).  For an
 * IMPAIRMENT scenario the frozen `M2aContext` deliberately exposes no input for
 * a channel model, so this CLI drives the SAME public production stage helpers
 * (`m2a_modulate_to_work` -> `m2a_tx_resample` -> `m2a_apply_impairment` ->
 * optional SC16 -> `m2a_rx_resample` -> `m2a_demod_work`) and applies the model
 * on the NATIVE CF32 grid between TX resampling and the (optional) RX SC16
 * quantisation.  There is no second PHY pipeline: every stage is the one core
 * the closed loop uses.  The decoder is never told the frame position and never
 * receives a "truth".
 *
 * It links the static `uwb_twr_phy` archive (which owns `uwb_twr_phy.cc` and
 * `uwb_twr_phy_impairment.cc`) and NOTHING from GNU Radio / UHD / PMT.
 *
 * ---------------------------------------------------------------------------
 * EVIDENCE BOUNDARY (G0 §9)
 * ---------------------------------------------------------------------------
 * `measurement_valid` is ALWAYS false and `hardware_readback` is ALWAYS null.
 * `evidence.level` is `native_roundtrip_verified` ONLY when the closed loop
 * actually succeeded (byte-exact decode on the native grid for THIS
 * rate/profile/taps/format); otherwise it is `work_decode_verified` /
 * `not_measured`.  A failed scenario never gets native evidence, and a clean
 * success never excuses a pressure failure.  Nothing here produces an RMARKER,
 * a first path, a ToA or a distance.
 *
 * ---------------------------------------------------------------------------
 * CLI (G0 §10, appendix A.3)
 * ---------------------------------------------------------------------------
 *   twr_native_phy_demo --frame poll|response|final
 *                       --native-rate 737280000|491520000
 *                       --iq cf32|sc16
 *                       [--scenario clean|cfo|awgn|delay|multipath|combo]
 *                       [--cfo-hz F] [--awgn-snr-db F] [--awgn-seed N]
 *                       [--delay-int N] [--delay-frac-num N] [--delay-frac-den N]
 *                       [--multipath "re:im@off,..."] [--repeat N] [--measure]
 *                       [--seed N]
 *                       [--testdata DIR] [--repo DIR]
 *                       [--native-iq PATH] [--no-native-iq]
 *                       --output PATH.json
 *
 * `--output -` writes the JSON to stdout and suppresses the native IQ
 * artifact.  Exit 0 iff the closed loop succeeded (bytes byte-exact).  A
 * decode failure under an impairment is a recorded terminal state (exit 1) and
 * never claims native evidence.
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
 *
 * `uwb_twr_phy` already carries `uwb_twr_phy_impairment.cc` (agent B), so NO
 * new CMake line is needed for the channel model.
 */

#include <gnuradio/uwb/uwb_cir_fir_simd.h>
#include <gnuradio/uwb/uwb_twr_capability_evidence.h>
#include <gnuradio/uwb/uwb_twr_frame.h>
#include <gnuradio/uwb/uwb_twr_phy.h>

#include <algorithm>
#include <cerrno>
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

std::string jnum_or_null(bool present, double v)
{
    return present ? jnum(v) : std::string("null");
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

// Resident set size in KiB, read from /proc; 0 when unavailable.
uint64_t read_rss_kb()
{
    std::ifstream f("/proc/self/status");
    if (!f)
        return 0;
    std::string line;
    while (std::getline(f, line)) {
        if (line.compare(0, 6, "VmRSS:") == 0) {
            std::istringstream is(line.substr(6));
            uint64_t kb = 0;
            is >> kb;
            return kb;
        }
    }
    return 0;
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

    // Impairment parameters.
    double cfo_hz = 0.0;
    bool cfo_set = false;
    double awgn_snr_db = 0.0;
    bool awgn_set = false;
    uint64_t awgn_seed = 0;
    int64_t delay_int = 0;
    int32_t delay_frac_num = 0;
    uint32_t delay_frac_den = 1;
    bool frac_set = false;
    std::string multipath;
    bool multipath_set = false;

    // Observation.
    uint64_t repeat = 1;
    bool measure = false;
};

void usage(std::ostream& os)
{
    os << "usage: twr_native_phy_demo --frame poll|response|final\n"
          "                           --native-rate 737280000|491520000\n"
          "                           --iq cf32|sc16\n"
          "                           [--scenario clean|cfo|awgn|delay|multipath|combo]\n"
          "                           [--cfo-hz F] [--awgn-snr-db F] [--awgn-seed N]\n"
          "                           [--delay-int N] [--delay-frac-num N] [--delay-frac-den N]\n"
          "                           [--multipath \"re:im@off,...\"] [--repeat N] [--measure]\n"
          "                           [--seed N]\n"
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

bool parse_i64(const std::string& text, int64_t& out)
{
    if (text.empty())
        return false;
    errno = 0;
    char* end = nullptr;
    const long long v = std::strtoll(text.c_str(), &end, 10);
    if (errno != 0 || end == nullptr || *end != '\0')
        return false;
    out = static_cast<int64_t>(v);
    return true;
}

bool parse_double(const std::string& text, double& out)
{
    if (text.empty())
        return false;
    errno = 0;
    char* end = nullptr;
    const double v = std::strtod(text.c_str(), &end);
    if (errno != 0 || end == nullptr || *end != '\0')
        return false;
    out = v;
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
        } else if (a == "--cfo-hz") {
            std::string v;
            if (!need(v))
                return false;
            if (!parse_double(v, o.cfo_hz)) {
                why = "--cfo-hz must be a number, got " + v;
                return false;
            }
            o.cfo_set = true;
        } else if (a == "--awgn-snr-db") {
            std::string v;
            if (!need(v))
                return false;
            if (!parse_double(v, o.awgn_snr_db)) {
                why = "--awgn-snr-db must be a number, got " + v;
                return false;
            }
            o.awgn_set = true;
        } else if (a == "--awgn-seed") {
            std::string v;
            if (!need(v))
                return false;
            if (!parse_u64(v, o.awgn_seed)) {
                why = "--awgn-seed must be a non-negative integer, got " + v;
                return false;
            }
        } else if (a == "--delay-int") {
            std::string v;
            if (!need(v))
                return false;
            if (!parse_i64(v, o.delay_int)) {
                why = "--delay-int must be an integer, got " + v;
                return false;
            }
        } else if (a == "--delay-frac-num") {
            std::string v;
            if (!need(v))
                return false;
            int64_t n = 0;
            if (!parse_i64(v, n) || n < 0 || n > INT32_MAX) {
                why = "--delay-frac-num must be a non-negative integer, got " + v;
                return false;
            }
            o.delay_frac_num = static_cast<int32_t>(n);
            o.frac_set = true;
        } else if (a == "--delay-frac-den") {
            std::string v;
            if (!need(v))
                return false;
            uint64_t d = 0;
            if (!parse_u64(v, d) || d == 0 || d > UINT32_MAX) {
                why = "--delay-frac-den must be a positive integer, got " + v;
                return false;
            }
            o.delay_frac_den = static_cast<uint32_t>(d);
            o.frac_set = true;
        } else if (a == "--multipath") {
            if (!need(o.multipath))
                return false;
            o.multipath_set = true;
        } else if (a == "--repeat") {
            std::string v;
            if (!need(v))
                return false;
            if (!parse_u64(v, o.repeat) || o.repeat == 0) {
                why = "--repeat must be a positive integer, got " + v;
                return false;
            }
        } else if (a == "--measure") {
            o.measure = true;
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
// Impairment construction (G0 A.3).  The model itself lives in the archive.
// ===========================================================================

bool parse_multipath_spec(const std::string& spec,
                          std::vector<gr_complex>& taps,
                          std::string& why)
{
    taps.clear();
    if (spec.empty())
        return true;
    size_t pos = 0;
    while (true) {
        const size_t comma = spec.find(',', pos);
        const std::string item =
            spec.substr(pos, comma == std::string::npos ? std::string::npos
                                                        : comma - pos);
        if (item.empty()) {
            why = "multipath has an empty entry";
            return false;
        }
        const size_t at = item.find('@');
        if (at == std::string::npos) {
            why = "multipath entry \"" + item + "\" is missing @offset";
            return false;
        }
        const std::string amp = item.substr(0, at);
        const std::string off_text = item.substr(at + 1);
        double re = 0.0;
        double im = 0.0;
        const size_t colon = amp.find(':');
        if (colon == std::string::npos) {
            if (!parse_double(amp, re)) {
                why = "multipath amplitude \"" + amp + "\" is not a number";
                return false;
            }
        } else {
            if (!parse_double(amp.substr(0, colon), re) ||
                !parse_double(amp.substr(colon + 1), im)) {
                why = "multipath complex amplitude \"" + amp + "\" is malformed";
                return false;
            }
        }
        int64_t off = 0;
        if (!parse_i64(off_text, off) || off < 0 || off > 100000) {
            why = "multipath offset \"" + off_text + "\" is not a sane non-negative integer";
            return false;
        }
        if (taps.size() < static_cast<size_t>(off) + 1)
            taps.resize(static_cast<size_t>(off) + 1, gr_complex(0.0f, 0.0f));
        taps[static_cast<size_t>(off)] = gr_complex(static_cast<float>(re),
                                                    static_cast<float>(im));
        if (comma == std::string::npos)
            break;
        pos = comma + 1;
    }
    return true;
}

twr::M2aImpairment build_impairment(const Options& o, std::string& why)
{
    twr::M2aImpairment imp;
    imp.enabled = (o.scenario != "clean");
    if (o.scenario == "clean")
        return imp;

    const bool cfo_scn = (o.scenario == "cfo" || o.scenario == "combo");
    const bool awgn_scn = (o.scenario == "awgn" || o.scenario == "combo");
    const bool delay_scn = (o.scenario == "delay" || o.scenario == "combo");
    const bool mp_scn = (o.scenario == "multipath" || o.scenario == "combo");

    if (cfo_scn)
        imp.cfo_hz = o.cfo_set ? o.cfo_hz : 20000.0;
    if (awgn_scn) {
        imp.awgn_enabled = true;
        imp.awgn_snr_db = o.awgn_set ? o.awgn_snr_db : 30.0;
        imp.awgn_seed = o.awgn_seed;
    }
    if (delay_scn) {
        imp.delay_int_samples = o.delay_int;
        if (o.frac_set) {
            imp.delay_frac_num = o.delay_frac_num;
            imp.delay_frac_den = o.delay_frac_den;
        } else if (o.scenario == "combo") {
            imp.delay_frac_num = 1;
            imp.delay_frac_den = 2;
        }
    }
    if (mp_scn) {
        const std::string spec = o.multipath_set ? o.multipath : "0.35:0@0,1.0:0@8";
        std::vector<gr_complex> taps;
        if (!parse_multipath_spec(spec, taps, why))
            return imp;
        imp.multipath = taps;
    }
    if (!imp.is_valid(why))
        return imp;
    return imp;
}

bool impairment_is_identity(const twr::M2aImpairment& imp)
{
    return !imp.enabled || (imp.cfo_hz == 0.0 && !imp.awgn_enabled &&
                            imp.delay_int_samples == 0 && imp.delay_frac_num == 0 &&
                            imp.multipath.empty());
}

// ===========================================================================
// Kernel names (the ACTUAL default selection for this build/taps).
// ===========================================================================

const char* demod_kernel_name()
{
    // No `default`: -Wswitch keeps this exhaustive over cir_fir::Kernel.
    switch (gr::uwb::demod::cir_fir::kDefaultKernel) {
    case gr::uwb::demod::cir_fir::Kernel::MultiAcc8:
        return "multi_acc8";
    case gr::uwb::demod::cir_fir::Kernel::Volk:
        return "volk";
    case gr::uwb::demod::cir_fir::Kernel::Avx2Fixed:
        return "avx2_fixed38";
    }
    return "invalid";
}

std::string tx_kernel_name(const twr::M2aConfig& cfg)
{
    if (cfg.native_rate == twr::M2aNativeRate::Uc200_737280000) {
        gr::uwb::core::RationalResampler48_65Core c(cfg.tx_taps.data(),
                                                    cfg.tx_taps.size());
        return c.kernel_name();
    }
    gr::uwb::core::RationalResampler32_65Core c(cfg.tx_taps.data(),
                                                cfg.tx_taps.size());
    return c.kernel_name();
}

std::string rx_kernel_name(const twr::M2aConfig& cfg)
{
    if (cfg.native_rate == twr::M2aNativeRate::Uc200_737280000) {
        gr::uwb::core::RationalResampler65_48Core c(cfg.rx_taps.data(),
                                                    cfg.rx_taps.size());
        return c.kernel_name();
    }
    gr::uwb::core::RationalResampler65_32Core c(cfg.rx_taps.data(),
                                                cfg.rx_taps.size());
    return c.kernel_name();
}

// ===========================================================================
// Impairment statistics (the valid region / power / variance the model uses).
//
// The model derives the AWGN region as the NONZERO SUPPORT of the (CFO-rotated)
// native buffer and defines SNR over that region's mean power (G0 A.3).  CFO is
// a unit-magnitude rotation, so |x| is unchanged; the harness can therefore
// compute the identical region and power from the CLEAN native grid it hands to
// the model.  Nothing here is fed to the decoder.
// ===========================================================================

struct ImpStats {
    bool have = false;
    int64_t lo = 0;
    int64_t hi = 0;
    size_t buffer_samples = 0;
    double p_valid = 0.0;
    double p_full = 0.0;
    double sigma2 = 0.0;
};

void compute_imp_stats(const std::vector<gr_complex>& clean_native,
                       const twr::M2aImpairment& imp, ImpStats& st)
{
    st = ImpStats{};
    st.have = true;
    st.buffer_samples = clean_native.size();
    size_t lo = 0;
    size_t hi = clean_native.size();
    while (lo < hi && clean_native[lo] == gr_complex(0.0f, 0.0f))
        ++lo;
    while (hi > lo && clean_native[hi - 1] == gr_complex(0.0f, 0.0f))
        --hi;
    st.lo = static_cast<int64_t>(lo);
    st.hi = static_cast<int64_t>(hi);
    const size_t nsig = hi - lo;
    double pv = 0.0;
    for (size_t n = lo; n < hi; ++n) {
        const double re = static_cast<double>(clean_native[n].real());
        const double im = static_cast<double>(clean_native[n].imag());
        pv += re * re + im * im;
    }
    if (nsig > 0)
        pv /= static_cast<double>(nsig);
    double pf = 0.0;
    for (const gr_complex& x : clean_native) {
        const double re = static_cast<double>(x.real());
        const double im = static_cast<double>(x.imag());
        pf += re * re + im * im;
    }
    if (!clean_native.empty())
        pf /= static_cast<double>(clean_native.size());
    st.p_valid = pv;
    st.p_full = pf;
    if (imp.awgn_enabled) {
        const double denom = 2.0 * std::pow(10.0, imp.awgn_snr_db / 10.0);
        st.sigma2 = pv / denom;
    }
}

// ===========================================================================
// The run outcome
// ===========================================================================

struct RunOutcome {
    twr::M2aResult res;
    std::string why;
    bool measured = false;
    std::string path;
    ImpStats stats;
    size_t native_impaired = 0; // native-grid length handed to the RX resampler
    std::vector<gr_complex> native_rx;
};

bool is_measured(const twr::M2aResult& r)
{
    return r.ok && r.bytes_exact &&
           r.demod_status == gr::uwb::demod::DemodStatus::Success && r.fcs_pass;
}

// Build the native artifact (post TX resample, optional SC16) via the same
// production stage helpers.  Used only when an artifact is requested on the
// clean path, where the prepared context does not expose its native buffer.
bool build_clean_native(const std::vector<uint8_t>& mac, const twr::M2aConfig& cfg,
                        std::vector<gr_complex>& native, std::string& why)
{
    std::vector<gr_complex> work;
    twr::M2aStageTrace t;
    if (!twr::m2a_modulate_to_work(mac.data(), mac.size(), cfg, work, t, why))
        return false;
    if (!twr::m2a_tx_resample(work.data(), work.size(), cfg, native, t, why))
        return false;
    if (cfg.iq_format == twr::M2aIqFormat::Sc16) {
        std::vector<gr_complex> q;
        size_t sat = 0;
        if (!twr::m2a_sc16_roundtrip(native.data(), native.size(),
                                     cfg.effective_sc16_scale(), q, sat, why))
            return false;
        native.swap(q);
    }
    return true;
}

// The impairment runner: the SAME public production stage helpers the prepared
// context uses, with the harness channel model applied on the NATIVE CF32 grid
// between TX resampling and the optional RX SC16 quantisation (G0 A.3).  The
// model is never handed to the decoder as a truth and the decoder is never told
// where the frame is.
bool run_stage_chain(const std::vector<uint8_t>& mac,
                     const twr::FrameProfile& profile,
                     const twr::M2aConfig& cfg,
                     const twr::M2aImpairment& imp,
                     RunOutcome& o,
                     std::string& why)
{
    o = RunOutcome{};
    twr::M2aResult& res = o.res;
    res.native_rate = cfg.native_rate;
    res.iq_format = cfg.iq_format;
    res.measurement_valid = false;
    o.path = "stage_runner_impairment";

    res.mac_bytes = mac;
    res.psdu_bytes = mac;
    gr::uwb::mod::append_ieee_fcs(res.psdu_bytes);
    res.fcs = static_cast<uint16_t>(
        static_cast<uint16_t>(res.psdu_bytes[mac.size()]) |
        static_cast<uint16_t>(static_cast<uint16_t>(res.psdu_bytes[mac.size() + 1])
                              << 8));
    res.expected_psdu_length = res.psdu_bytes.size();

    std::vector<gr_complex> work_tx;
    std::vector<gr_complex> native;
    std::vector<gr_complex> work_rx;
    twr::M2aStageTrace t;

    if (!twr::m2a_modulate_to_work(mac.data(), mac.size(), cfg, work_tx, t, why)) {
        res.status = twr::M2aStatus::ModulateFailed;
        res.detail = why;
        return false;
    }
    res.work_tx_samples = work_tx.size();
    res.stages.push_back(t);

    if (!twr::m2a_tx_resample(work_tx.data(), work_tx.size(), cfg, native, t, why)) {
        res.status = twr::M2aStatus::TxResampleFailed;
        res.detail = why;
        return false;
    }
    res.native_samples = native.size();
    res.stages.push_back(t);

    // Statistics on the clean native grid (magnitude is CFO-invariant).
    compute_imp_stats(native, imp, o.stats);

    if (!twr::m2a_apply_impairment(native, cfg.native_rate_hz(), imp, why)) {
        res.status = twr::M2aStatus::QuantiseFailed;
        res.detail = why;
        return false;
    }

    if (cfg.iq_format == twr::M2aIqFormat::Sc16) {
        std::vector<gr_complex> q;
        size_t sat = 0;
        if (!twr::m2a_sc16_roundtrip(native.data(), native.size(),
                                     cfg.effective_sc16_scale(), q, sat, why)) {
            res.status = twr::M2aStatus::QuantiseFailed;
            res.detail = why;
            return false;
        }
        res.sc16_saturated = sat;
        native.swap(q);
    }
    o.native_impaired = native.size();
    o.native_rx = native;

    if (!twr::m2a_rx_resample(native.data(), native.size(), cfg, work_rx, t, why)) {
        res.status = twr::M2aStatus::RxResampleFailed;
        res.detail = why;
        return false;
    }
    res.work_rx_samples = work_rx.size();
    res.stages.push_back(t);

    gr::uwb::demod::DemodResult dr;
    if (!twr::m2a_demod_work(work_rx.data(), work_rx.size(), cfg, dr, t, why)) {
        res.status = twr::M2aStatus::DemodFailed;
        res.detail = why;
        return false;
    }
    res.stages.push_back(t);

    res.demod_status = dr.status;
    res.fcs_pass = dr.payload.fcs_pass;
    res.decoded_bytes = dr.payload.bytes;
    res.sfd_start_sample = dr.sfd.sfd_start_sample;
    res.packet_start_sample = dr.timing.preamble_start_sample;

    {
        std::ostringstream os;
        os << "demod_search{guard_front=" << t.search_guard_front
           << ",guard_back=" << t.search_guard_back
           << ",buffer=" << (t.in_count + t.padding) << ",roi=["
           << t.search_roi_from << "," << t.search_roi_to
           << "),predicted_start=-1,window_start=0}";
        res.detail = os.str();
    }

    auto fail = [&](twr::M2aStatus s, const std::string& d) -> bool {
        res.ok = false;
        res.status = s;
        res.detail = res.detail.empty() ? d : (d + "; " + res.detail);
        why = d;
        return false;
    };

    if (dr.status == gr::uwb::demod::DemodStatus::FcsFailed)
        return fail(twr::M2aStatus::FcsFailed,
                    "demod decoded a frame whose FCS failed");
    if (dr.status != gr::uwb::demod::DemodStatus::Success)
        return fail(twr::M2aStatus::DemodFailed,
                    std::string("demod status is not Success: ") +
                        std::to_string(static_cast<int>(dr.status)));
    if (!dr.payload.fcs_pass)
        return fail(twr::M2aStatus::FcsFailed, "demod FCS check failed");
    if (dr.payload.bytes.size() != res.expected_psdu_length)
        return fail(twr::M2aStatus::LengthMismatch,
                    "decoded PSDU length " +
                        std::to_string(dr.payload.bytes.size()) + " != expected " +
                        std::to_string(res.expected_psdu_length));
    if (dr.payload.bytes != res.psdu_bytes)
        return fail(twr::M2aStatus::BytesMismatch,
                    "decoded PSDU bytes differ from the input PSDU");

    const uint8_t* macp = nullptr;
    size_t mac_n = 0;
    std::string derr;
    twr::FrameError dcode = twr::FrameError::None;
    if (!twr::mac_payload_from_psdu(dr.payload.bytes.data(), dr.payload.bytes.size(),
                                    profile, macp, mac_n, derr, &dcode))
        return fail(twr::M2aStatus::InternalError,
                    "mac_payload_from_psdu refused the decoded PSDU: " + derr);
    twr::Frame decoded;
    if (!twr::decode(macp, mac_n, profile, decoded, derr, &dcode))
        return fail(twr::M2aStatus::InternalError,
                    "codec decode refused the decoded MAC payload: " + derr);

    res.ok = true;
    res.status = twr::M2aStatus::Ok;
    res.bytes_exact = true;
    res.max_abs_error = 0.0;
    res.relative_l2 = 0.0;
    res.measurement_valid = false;
    why.clear();
    o.measured = true;
    return true;
}

// ===========================================================================
// JSON emission
// ===========================================================================

std::string stage_json(const twr::M2aStageTrace& s)
{
    std::ostringstream os;
    os << "{\"name\":" << jstr(s.name)
       << ",\"in_unit\":" << jstr(s.in_unit) << ",\"out_unit\":" << jstr(s.out_unit)
       << ",\"unit\":" << jstr(s.unit)
       << ",\"rate_hz\":" << jnum(s.rate_hz) << ",\"l\":" << juint(s.interp)
       << ",\"m\":" << juint(s.decim) << ",\"origin\":" << jint(s.origin)
       << ",\"in_count\":" << juint(s.in_count) << ",\"out_count\":" << juint(s.out_count)
       << ",\"phase\":" << juint(s.phase) << ",\"trim\":" << jint(s.trim)
       << ",\"pad_front\":" << jint(s.pad_front) << ",\"pad_back\":" << jint(s.pad_back)
       << ",\"padding\":" << jint(s.padding)
       << ",\"search_guard_front\":" << jint(s.search_guard_front)
       << ",\"search_guard_back\":" << jint(s.search_guard_back)
       << ",\"search_roi\":[" << juint(s.search_roi_from) << ","
       << juint(s.search_roi_to) << "]"
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

std::string multipath_json(const std::vector<gr_complex>& taps)
{
    std::ostringstream os;
    os << "[";
    for (size_t i = 0; i < taps.size(); ++i) {
        if (i)
            os << ",";
        os << "{\"re\":" << jnum(static_cast<double>(taps[i].real()))
           << ",\"im\":" << jnum(static_cast<double>(taps[i].imag())) << "}";
    }
    os << "]";
    return os.str();
}

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
    if (o.scenario != "clean" && o.scenario != "cfo" && o.scenario != "awgn" &&
        o.scenario != "delay" && o.scenario != "multipath" && o.scenario != "combo") {
        std::cerr << "error: --scenario must be clean|cfo|awgn|delay|multipath|combo, got \""
                  << o.scenario << "\"\n";
        return 2;
    }

    // ---- build the impairment --------------------------------------------
    twr::M2aImpairment imp = build_impairment(o, why);
    if (!why.empty()) {
        std::cerr << "error: bad impairment parameters: " << why << "\n";
        return 2;
    }
    if (!imp.is_valid(why)) {
        std::cerr << "error: impairment is invalid: " << why << "\n";
        return 2;
    }
    const bool identity = impairment_is_identity(imp);

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

    // ---- prepared context (G0 A.1).  Always prepared, so the manifest can
    //      report the frozen capacity even when the impairment seam is used.
    twr::M2aContext ctx;
    std::string pwhy;
    if (!ctx.prepare(cfg, pwhy)) {
        std::cerr << "error: M2aContext.prepare failed: " << pwhy << "\n";
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

    // ---- run (repeat) -----------------------------------------------------
    RunOutcome first;
    uint64_t success = 0;
    std::vector<double> times_us;
    times_us.reserve(static_cast<size_t>(o.repeat));

    for (uint64_t it = 0; it < o.repeat; ++it) {
        RunOutcome cur;
        std::string rwhy;
        const auto t0 = std::chrono::steady_clock::now();
        if (identity) {
            (void)ctx.run(frame, prof, cur.res, rwhy);
            cur.why = rwhy;
            cur.path = "prepared_context";
            cur.measured = is_measured(cur.res);
            cur.native_impaired = cur.res.native_samples;
        } else {
            (void)run_stage_chain(mac, prof, cfg, imp, cur, rwhy);
            cur.why = rwhy;
        }
        const auto t1 = std::chrono::steady_clock::now();
        times_us.push_back(
            std::chrono::duration<double, std::micro>(t1 - t0).count());
        if (it == 0)
            first = cur;
        if (cur.measured)
            ++success;
    }

    const twr::M2aResult& res = first.res;
    const bool measured = first.measured;
    const bool overall_ok = (success == o.repeat);

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

    const std::string tx_kernel = tx_kernel_name(cfg);
    const std::string rx_kernel = rx_kernel_name(cfg);
    const std::string dm_kernel = demod_kernel_name();

    // ---- optional native IQ artifact --------------------------------------
    std::string native_iq_path;
    bool wrote_iq = false;
    if (!o.no_native_iq && o.output != "-") {
        std::vector<gr_complex> native;
        if (identity) {
            std::string iqwhy;
            (void)build_clean_native(mac, cfg, native, iqwhy);
        } else {
            native = first.native_rx;
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
    os << "  \"schema\": \"twr-m2a-native/2\",\n";
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

    // impairment block (G0 A.5).  SNR is defined over the signal valid region.
    os << "  \"impairment\": {\"enabled\": " << jbool(imp.enabled)
       << ", \"cfo_hz\": " << jnum(imp.cfo_hz)
       << ", \"awgn_enabled\": " << jbool(imp.awgn_enabled)
       << ", \"awgn_snr_db\": " << jnum_or_null(imp.awgn_enabled, imp.awgn_snr_db)
       << ", \"awgn_seed\": " << (imp.awgn_enabled ? juint(imp.awgn_seed) : "null")
       << ", \"delay_int_samples\": " << jint(imp.delay_int_samples)
       << ", \"delay_frac_num\": " << jint(static_cast<int64_t>(imp.delay_frac_num))
       << ", \"delay_frac_den\": " << juint(imp.delay_frac_den)
       << ", \"multipath\": " << multipath_json(imp.multipath)
       << ", \"snr_definition\": \"valid-region mean power\"";
    if (first.stats.have && imp.awgn_enabled) {
        os << ", \"valid_region\": [" << jint(first.stats.lo) << ","
           << jint(first.stats.hi) << "]"
           << ", \"region_samples\": " << jint(first.stats.hi - first.stats.lo)
           << ", \"buffer_samples\": " << juint(first.stats.buffer_samples)
           << ", \"signal_power_valid\": " << jnum(first.stats.p_valid)
           << ", \"signal_power_full\": " << jnum(first.stats.p_full)
           << ", \"noise_sigma2_per_dim\": " << jnum(first.stats.sigma2);
    } else {
        os << ", \"valid_region\": null, \"region_samples\": null,"
              " \"buffer_samples\": "
           << (first.stats.have ? juint(first.stats.buffer_samples) : std::string("null"))
           << ", \"signal_power_valid\": null, \"signal_power_full\": null,"
              " \"noise_sigma2_per_dim\": null";
    }
    os << "},\n";

    // context block (G0 A.5).  capacity_samples is the frozen kM2aMaxSamples
    // bound the prepared context enforces on every buffer.
    os << "  \"context\": {\"prepared\": " << jbool(ctx.prepared())
       << ", \"capacity_samples\": " << juint(twr::kM2aMaxSamples)
       << ", \"path\": " << jstr(first.path) << "},\n";

    // kernel block (G0 A.5): the ACTUAL kernel names in use.
    os << "  \"kernel\": {\"tx\": " << jstr(tx_kernel) << ", \"rx\": " << jstr(rx_kernel)
       << ", \"demod\": " << jstr(dm_kernel) << "},\n";

    os << "  \"stages\": [";
    for (size_t i = 0; i < res.stages.size(); ++i) {
        if (i)
            os << ", ";
        os << stage_json(res.stages[i]);
    }
    os << "],\n";
    os << "  \"samples\": {\"work_tx\": " << res.work_tx_samples
       << ", \"native\": " << res.native_samples
       << ", \"native_impaired\": " << first.native_impaired
       << ", \"work_rx\": " << res.work_rx_samples
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

    os << "  \"evidence\": {\"level\": "
       << jstr(measured ? "native_roundtrip_verified" : "none")
       << ", \"kind\": " << jstr(measured ? "measured" : "not_measured")
       << ", \"scope\": \"m2a-native-roundtrip/1\""
       << ", \"hardware_readback\": null, \"measurement_valid\": false},\n";

    os << "  \"config\": {\"requested\": {\"frame\": " << jstr(o.frame)
       << ", \"native_rate\": " << o.native_rate << ", \"iq\": " << jstr(o.iq)
       << ", \"seed\": " << o.seed << ", \"scenario\": " << jstr(o.scenario)
       << ", \"cfo_hz\": " << jnum(imp.cfo_hz)
       << ", \"awgn_snr_db\": " << jnum_or_null(imp.awgn_enabled, imp.awgn_snr_db)
       << ", \"awgn_seed\": " << (imp.awgn_enabled ? juint(imp.awgn_seed) : "null")
       << ", \"delay_int\": " << jint(imp.delay_int_samples)
       << ", \"delay_frac_num\": " << jint(static_cast<int64_t>(imp.delay_frac_num))
       << ", \"delay_frac_den\": " << juint(imp.delay_frac_den)
       << ", \"multipath\": " << multipath_json(imp.multipath)
       << ", \"repeat\": " << o.repeat << "}";
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

    // observation block: emitted when the demo is asked to measure.
    if (o.measure || o.repeat > 1) {
        std::vector<double> sorted = times_us;
        std::sort(sorted.begin(), sorted.end());
        auto pct = [&](double p) -> double {
            if (sorted.empty())
                return 0.0;
            size_t idx = static_cast<size_t>(p * static_cast<double>(sorted.size() - 1) + 0.5);
            if (idx >= sorted.size())
                idx = sorted.size() - 1;
            return sorted[idx];
        };
        const double cold = times_us.empty() ? 0.0 : times_us.front();
        double mx = 0.0;
        for (double v : times_us)
            mx = std::max(mx, v);
        os << "  \"observation\": {\"iters\": " << o.repeat
           << ", \"p50_us\": " << jnum(pct(0.50)) << ", \"p95_us\": " << jnum(pct(0.95))
           << ", \"p99_us\": " << jnum(pct(0.99)) << ", \"max_us\": " << jnum(mx)
           << ", \"cold_us\": " << jnum(cold)
           << ", \"rss_kb\": " << juint(read_rss_kb())
           << ", \"alloc\": null"
           << ", \"counter_scope\": \"timing only in this CLI; allocation counting is "
              "a separate instrumented program (agent C)\"},\n";
    }

    os << "  \"detail\": " << jstr(res.detail) << ",\n";
    os << "  \"status\": {\"ok\": " << jbool(overall_ok)
       << ", \"exit_code\": " << (overall_ok ? 0 : 1)
       << ", \"reason\": " << jstr(first.why)
       << ", \"attempted\": " << o.repeat
       << ", \"exact_success\": " << success
       << ", \"explicit_failure\": " << (o.repeat - success)
       << ", \"m2a_status\": " << jstr(twr::m2a_status_to_string(res.status)) << "}\n";
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

    if (!overall_ok) {
        std::cerr << "error: closed loop failed: scenario=" << o.scenario
                  << " status=" << twr::m2a_status_to_string(res.status)
                  << " reason=" << first.why << "\n";
        return 1;
    }
    std::cerr << "ok: frame=" << o.frame << " rate=" << o.native_rate << " iq=" << o.iq
              << " scenario=" << o.scenario
              << " work_tx=" << res.work_tx_samples << " native=" << res.native_samples
              << " work_rx=" << res.work_rx_samples
              << " sc16_saturated=" << res.sc16_saturated << "\n";
    return 0;
}
