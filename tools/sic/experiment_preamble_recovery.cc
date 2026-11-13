/* -*- c++ -*- */
/*
 * Offline functional matrix for the unified preamble front-end (R06, plan
 * §4.2): demod + preamble-only SIC + sensing CIR on synthetic mixtures.
 *
 * ---------------------------------------------------------------------------
 * ONLINE INPUTS (everything the receiver is allowed to read)
 * ---------------------------------------------------------------------------
 *   - the mixed IQ window `raw` (target + interferer + noise + channel)
 *   - the public pulse-shaped SYNC template of the TARGET code
 *   - the public interferer preamble waveform (SIC template) and its code index
 *   - the configured coarse ROI around the interferer's nominal start
 *   - the configured nominal target start (the public TX-time axis)
 *
 * ---------------------------------------------------------------------------
 * TRUTH ONLY (never read by the online path; written to the CSV for scoring)
 * ---------------------------------------------------------------------------
 *   - which packet is the target and which is the interferer
 *   - the applied target / interferer carrier offset in Hz
 *   - the injected ISR, noise sigma, and the complex channel (path delays,
 *     gains, phases) of the fractional-multipath and dual-target cases
 *   - the PSDU bytes, the clean reference IQ, and the clean reference CIR
 *   - `expected_peak_tap`, the main-lobe exclusion region, the weak-target tap
 *
 * The online calls (`demod::core::demodulate_one`, `sic::sic_process_unified`,
 * `radar::radar_cir_one`) receive only the columns listed under ONLINE INPUTS.
 * Nothing in the truth list is passed to them; the CSV columns that carry
 * truth are marked `truth:` in the header row comment below.
 *
 * ---------------------------------------------------------------------------
 * Matrix axes (every one is an explicit CLI dimension)
 * ---------------------------------------------------------------------------
 *   config   : zip(--sync-reps, --psdu-bytes)        64/32 and 256/127
 *   codes    : zip(--target-code, --interferer-code) 9<-10 and 10<-9
 *   multipath: --multipath {single,fractional}
 *   ISR      : --isr-db (product)
 *   CFO      : --target-cfo-hz x --interferer-cfo-hz (product)
 *   grid     : --grid {work,native}
 *   seeds    : --seeds N (seed cases, not cells)
 *
 * The standard matrix is 2 x 2 x 2 x 3 x 3 x 3 = 432 cells and 432 * 16 = 6912
 * seed cases.  `--list-cells` prints the enumeration and its count; the
 * `--self-check` mode asserts the count is exactly 432 / 6912.
 *
 * ---------------------------------------------------------------------------
 * Fixed measurement axis (PSLR / ISLR / main-lobe width)
 * ---------------------------------------------------------------------------
 * The CIR tap axis is `tap i <-> lag (i - cir_pre)` relative to the PUBLIC
 * TX-time prediction (`cir_axis_origin`), so it is identical for the ideal,
 * clean, raw and SIC variants of a cell.  The main-lobe exclusion region is
 * derived ONCE per cell from the generator's truth path delays (each truth
 * path rounds to a tap, +/- kMlHalf taps) and is applied unchanged to every
 * variant; it is never re-derived from the observed peak.  The region bounds
 * and tap count are written to the CSV so the reader can audit them.
 *
 * ---------------------------------------------------------------------------
 * Legacy positional CLI
 * ---------------------------------------------------------------------------
 * `binary <seeds> <work|native|dump> <dir> [scenario] [reps] [psdu] [ireps]`
 * is kept byte-compatible (same CSV header, same dump file names) because
 * `tools/sic/run_chain_workers.py` and `tools/sic/run_native_preamble_recovery.py`
 * consume it and are outside this file's ownership.
 */

#include <gnuradio/uwb/uwb_demod_core.h>
#include <gnuradio/uwb/uwb_hrp_mod_core.h>
#include <gnuradio/uwb/uwb_radar_cir_core.h>
#include <gnuradio/uwb/uwb_rational_resampler_core.h>
#include <gnuradio/uwb/uwb_sic_unified.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

using namespace gr::uwb;
using cf = std::complex<float>;

namespace {

constexpr size_t kSps = 1016;          // samples per SYNC symbol at 998.4 MS/s
constexpr size_t kSfdSymbols = 8;
constexpr size_t kQ = 512;             // nominal packet origin (work samples)
constexpr double kFs = 998.4e6;
constexpr double kPi = 3.14159265358979323846;

constexpr size_t kCirPre = 16;
constexpr size_t kCirPost = 100;
constexpr size_t kCirSkip = 10;

// A zero-delay path peaks at lag kAxisOffset, not lag 0: the Legacy TX pulse
// (kPulse48) has a fixed 2-sample group delay that the CIR axis does not
// remove.  It is a constant of the chain, measured once by --self-check and
// identical for every code and preamble length, so `expected_peak_tap` is
// kCirPre + kAxisOffset + round(truth delay) and never a per-variant fit.
constexpr size_t kAxisOffset = 2;

// Main-lobe exclusion half width in taps.  The Legacy pulse response decays
// slowly (|h| at +/-6 taps from the peak is about -25 dB), so a narrower region
// would measure the pulse tail instead of the sidelobes.  Fixed for every
// variant of a cell; the bounds and tap count are written to the CSV.
constexpr size_t kMlHalf = 6;

// Fractional multipath: second path at +7.3 samples, complex gain 0.6*e^{j0.7}.
// The delay is deliberately non-integer so the delay axis is exercised
// sub-tap; the gain is documented here and in the CSV `multipath` column.
constexpr double kFracDelay = 7.3;
constexpr double kFracGain = 0.6;
constexpr double kFracPhase = 0.7;
constexpr size_t kFracHalfTaps = 32;   // 65-tap Kaiser-windowed sinc

// Optional dual-target special case (not part of the 432-cell matrix).
constexpr double kWeakDelay = 40.0;
constexpr double kWeakGainDb = -20.0;
constexpr double kWeakPhase = 0.9;

constexpr double kSnrDb = 20.0;        // desired-packet-average SNR
constexpr double kIntPhase = 0.4;      // interferer carrier phase offset

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

// ---------------------------------------------------------------------------
// Small helpers
// ---------------------------------------------------------------------------
std::vector<std::string> split_list(const std::string& text)
{
    std::vector<std::string> out;
    std::string cur;
    for (char c : text) {
        if (c == ',') {
            if (!cur.empty())
                out.push_back(cur);
            cur.clear();
        } else {
            cur.push_back(c);
        }
    }
    if (!cur.empty())
        out.push_back(cur);
    return out;
}

double kaiser(double r, double beta)
{
    const double t = 1.0 - r * r;
    if (t <= 0.0)
        return 0.0;
    return std::cyl_bessel_i(0.0, beta * std::sqrt(t)) /
           std::cyl_bessel_i(0.0, beta);
}

double energy(const std::vector<cf>& x)
{
    double e = 0;
    for (const auto& a : x)
        e += std::norm(std::complex<double>(a));
    return e;
}

const char* demod_status_name(int status)
{
    switch (static_cast<demod::DemodStatus>(status)) {
    case demod::DemodStatus::Success:
        return "success";
    case demod::DemodStatus::InvalidInput:
        return "invalid_input";
    case demod::DemodStatus::TimingFailed:
        return "timing_failed";
    case demod::DemodStatus::CfoFailed:
        return "cfo_failed";
    case demod::DemodStatus::SfdFailed:
        return "sfd_failed";
    case demod::DemodStatus::PhrFailed:
        return "phr_failed";
    case demod::DemodStatus::PayloadFailed:
        return "payload_failed";
    case demod::DemodStatus::FcsFailed:
        return "fcs_failed";
    case demod::DemodStatus::QueueFull:
        return "queue_full";
    case demod::DemodStatus::InternalError:
        return "internal_error";
    case demod::DemodStatus::CirFailed:
        return "cir_failed";
    }
    return "unknown";
}

const char* radar_status_name(int status)
{
    switch (static_cast<radar::RadarCirStatus>(status)) {
    case radar::RadarCirStatus::Ok:
        return "ok";
    case radar::RadarCirStatus::SfdFailed:
        return "sfd_failed";
    case radar::RadarCirStatus::TimingFailed:
        return "timing_failed";
    case radar::RadarCirStatus::CirFailed:
        return "cir_failed";
    case radar::RadarCirStatus::InvalidInput:
        return "invalid_input";
    }
    return "unknown";
}

// ---------------------------------------------------------------------------
// Packets
// ---------------------------------------------------------------------------
struct Packet {
    mod::HrpModConfig cfg;
    mod::HrpModScratch scratch;
    std::vector<uint8_t> bytes;
    std::vector<cf> wave;   // full packet, no carrier offset applied
    std::vector<cf> sync;   // one SYNC symbol (public template)
    std::vector<cf> prefix; // SYNC + SFD only (public SIC template)

    Packet(size_t code, unsigned seed, size_t repetitions, size_t psdu_bytes)
    {
        cfg.code_index = code;
        cfg.sync_repetitions = repetitions;
        cfg.sfd_mode = "4z2";
        cfg.peak_amplitude = .8f;
        if (psdu_bytes < 2 || psdu_bytes > 127)
            throw std::runtime_error("PSDU must include the 2-byte FCS and fit 127 bytes");
        if (repetitions == 0)
            throw std::runtime_error("sync repetitions must be positive");
        bytes = mod::make_random_psdu(psdu_bytes - 2, seed, true);
        const size_t cap =
            mod::packet_samples_998p4(repetitions, kSfdSymbols, bytes.size(), false, &cfg.pulse) + 512;
        scratch.reserve(127, cap);
        wave.resize(cap);
        prefix.resize(cap);
        size_t len = 0;
        if (!mod::modulate_one(bytes.data(), bytes.size(), cfg, scratch, wave.data(), cap, len))
            throw std::runtime_error("packet generation failed");
        wave.resize(len);
        sync.assign(wave.begin(), wave.begin() + kSps);
        if (!mod::modulate_prefix_waveform(cfg, scratch, prefix.data(), cap, len))
            throw std::runtime_error("prefix generation failed");
        prefix.resize(len);
    }
};

// Multiply x[0..n) by exp(j(2*pi*cfo*j/fs + phase0)).  Applied to the packet's
// own sample coordinate, so the phase slope across the SYNC train is exactly
// the carrier offset the receiver has to estimate.
void apply_cfo(cf* x, size_t n, double cfo_hz, double phase0)
{
    if (cfo_hz == 0.0) {
        if (phase0 == 0.0)
            return;
        const cf rot(float(std::cos(phase0)), float(std::sin(phase0)));
        for (size_t j = 0; j < n; ++j)
            x[j] *= rot;
        return;
    }
    const double w = 2.0 * kPi * cfo_hz / kFs;
    for (size_t j = 0; j < n; ++j) {
        const double ph = w * double(j) + phase0;
        x[j] *= cf(float(std::cos(ph)), float(std::sin(ph)));
    }
}

// Band-limited fractional delay y[n] ~= x[n - delay] with unity DC gain.
// The integer part is an exact shift and the fractional part is a 65-tap
// Kaiser-windowed sinc (beta 8.6) normalised so sum(h) == 1.
std::vector<cf> fractional_delay(const std::vector<cf>& x, double delay)
{
    const int half = int(kFracHalfTaps);
    const long whole = long(std::floor(delay));
    const double frac = delay - double(whole);
    std::vector<double> h(size_t(2 * half + 1));
    double sum = 0;
    for (int k = -half; k <= half; ++k) {
        const double t = double(k) - frac;
        const double s = (std::abs(t) < 1e-12) ? 1.0 : std::sin(kPi * t) / (kPi * t);
        const double w = kaiser(double(k) / double(half), 8.6);
        h[size_t(k + half)] = s * w;
        sum += h[size_t(k + half)];
    }
    if (std::abs(sum) < 1e-12)
        throw std::runtime_error("fractional delay window has zero DC gain");
    for (auto& v : h)
        v /= sum;

    std::vector<cf> y(x.size() + size_t(whole) + size_t(half), cf(0.f, 0.f));
    for (size_t n = 0; n < y.size(); ++n) {
        cf acc(0.f, 0.f);
        for (int k = -half; k <= half; ++k) {
            const long idx = long(n) - whole - long(k);
            if (idx >= 0 && idx < long(x.size()))
                acc += float(h[size_t(k + half)]) * x[size_t(idx)];
        }
        y[n] = acc;
    }
    return y;
}

// ---------------------------------------------------------------------------
// Observation of one buffer (demod + radar CIR)
// ---------------------------------------------------------------------------
struct Observation {
    int demod_status = -1;
    bool sfd = false;
    bool phr = false;
    bool fcs = false;
    bool psdu_exact = false;
    size_t psdu_len = 0;
    double measured_cfo_hz = kNaN; // truth-scored: the receiver's own estimate

    int radar_status = -1;
    int64_t detected = -1;
    int64_t cir_axis_origin = -1;
    double radar_cfo_hz = 0.0;
    uint32_t cfo_epoch = 0;
    const char* radar_reason = "";
    size_t peak = 0;
    double peak_abs = 0.0;
    std::vector<cf> cir; // raw taps, tap i <-> lag (i - kCirPre)
};

Observation observe(const std::vector<cf>& y,
                    const Packet& target,
                    int64_t predicted,
                    size_t coarse_stride,
                    size_t search_margin)
{
    Observation out;
    auto profile = demod::Qm35825Profile::Default();
    profile.code_index = target.cfg.code_index;
    profile.preamble_repetitions = target.cfg.sync_repetitions;
    profile.cir_repetitions =
        target.cfg.sync_repetitions - profile.cir_skip_initial_repetitions;
    profile.timing_coarse_stride = coarse_stride;
    profile.timing_search_margin = search_margin;
    profile.rx_fast_sfd = true;
    profile.rx_fast_cfo_rotation = true;
    profile.rx_seed_refine_tail = true;
    profile.rx_softchip_on_demand = true;
    profile.rx_fused_payload_cmf = true;
    profile.rx_fused_phr_cmf = true;
    demod::core::DemodScratch ds;
    ds.reserve(y.size());
    auto d = demod::core::demodulate_one(y.data(), y.size(), profile, 0, predicted, 0,
                                         target.sync, ds);
    out.demod_status = int(d.status);
    out.sfd = d.sfd.ok;
    out.phr = d.phr.ok;
    out.fcs = d.payload.fcs_pass;
    out.psdu_exact = d.payload.bytes == target.bytes;
    out.psdu_len = d.phr.psdu_length;
    out.measured_cfo_hz = d.cfo.ok ? d.cfo.cfo_hz : kNaN;

    radar::RadarCirConfig cfg;
    cfg.code_index = target.cfg.code_index;
    cfg.sync_repetitions = target.cfg.sync_repetitions;
    cfg.cir_pre = kCirPre;
    cfg.cir_post = kCirPost;
    cfg.cir_skip_initial = kCirSkip;
    cfg.cir_repetitions = cfg.sync_repetitions - cfg.cir_skip_initial;
    radar::RadarCirCoreScratch scratch;
    if (!radar::prepare_radar_cir_core(cfg, target.sync.data(), kSps, cfg.cir_pre,
                                       cfg.cir_post, scratch))
        throw std::runtime_error("radar preparation failed");
    radar::RadarCirResult r;
    radar::radar_cir_one(y.data(), y.size(),
                         predicted + int64_t(cfg.sync_repetitions) * int64_t(kSps), cfg,
                         scratch, r);
    out.radar_status = int(r.status);
    out.detected = r.preamble_start_sample;
    out.cir_axis_origin = r.cir_axis_origin;
    out.radar_cfo_hz = r.final_cfo_hz;
    out.cfo_epoch = r.cfo_epoch;
    out.radar_reason = r.status_reason ? r.status_reason : "";
    out.peak = r.peak_tap;
    out.peak_abs = r.peak_abs;
    out.cir.assign(scratch.cir.raw_taps.begin(),
                   scratch.cir.raw_taps.begin() + long(r.tap_count));
    return out;
}

// ---------------------------------------------------------------------------
// Truth geometry and the fixed metric axis
// ---------------------------------------------------------------------------
struct Truth {
    std::vector<double> delays; // target path delays in samples (truth)
    size_t expected_peak = 0;   // tap of the strongest truth path
    std::vector<uint8_t> mainlobe; // fixed exclusion mask (tap_count entries)
    size_t ml_lo = 0, ml_hi = 0, ml_taps = 0;
    bool has_weak = false;
    size_t weak_tap = 0;
    size_t weak_excl_lo = 0, weak_excl_hi = 0;
};

Truth make_truth(bool fractional, bool dual, double weak_delay = kWeakDelay)
{
    Truth t;
    const size_t tap_count = kCirPre + kCirPost;
    t.delays.push_back(0.0);
    if (fractional)
        t.delays.push_back(kFracDelay);
    t.expected_peak =
        kCirPre + kAxisOffset + size_t(std::llround(t.delays.front()));
    t.mainlobe.assign(tap_count, 0);
    size_t lo = tap_count, hi = 0, count = 0;
    for (double d : t.delays) {
        const long centre = long(kCirPre) + long(kAxisOffset) + std::llround(d);
        const long a = std::max<long>(0, centre - long(kMlHalf));
        const long b = std::min<long>(long(tap_count) - 1, centre + long(kMlHalf));
        for (long i = a; i <= b; ++i) {
            if (!t.mainlobe[size_t(i)]) {
                t.mainlobe[size_t(i)] = 1;
                ++count;
            }
            lo = std::min(lo, size_t(i));
            hi = std::max(hi, size_t(i));
        }
    }
    t.ml_lo = lo;
    t.ml_hi = hi;
    t.ml_taps = count;
    if (dual) {
        t.has_weak = true;
        t.weak_tap = kCirPre + kAxisOffset + size_t(std::llround(weak_delay));
        // The weak target is a separate peak, not a main-lobe tap: it is
        // excluded from the sidelobe sums so PSLR/ISLR stay a main-lobe
        // quality metric.  The exclusion region is fixed, not per variant.
        t.weak_excl_lo = t.weak_tap > 1 ? t.weak_tap - 1 : 0;
        t.weak_excl_hi = std::min(tap_count - 1, t.weak_tap + 1);
    }
    return t;
}

struct Metrics {
    double peak_shift = kNaN;
    double gain_db = kNaN;
    double relative_l2 = -1.0;
    double pslr_db = kNaN;
    double islr_db = kNaN;
    size_t width = 0;
    double weak_abs = kNaN;
    double weak_gain_db = kNaN;
    int weak_preserved = 0;
};

double tap_abs(const std::vector<cf>& cir, size_t i)
{
    return i < cir.size() ? double(std::abs(cir[i])) : 0.0;
}

Metrics compute_metrics(const Observation& obs, const Observation& clean, const Truth& truth)
{
    Metrics m;
    if (obs.radar_status != 0 || clean.radar_status != 0)
        return m;
    if (obs.cir.size() != clean.cir.size() || obs.cir.empty())
        return m;

    double err = 0, ref = 0;
    for (size_t j = 0; j < obs.cir.size(); ++j) {
        err += std::norm(std::complex<double>(obs.cir[j] - clean.cir[j]));
        ref += std::norm(std::complex<double>(clean.cir[j]));
    }
    m.relative_l2 = std::sqrt(err / std::max(ref, 1e-30));
    m.peak_shift = double(obs.peak) - double(clean.peak);
    m.gain_db = 20.0 * std::log10(std::max(obs.peak_abs, 1e-30) /
                                  std::max(clean.peak_abs, 1e-30));

    double peak_ml = 0, energy_ml = 0, peak_sl = 0, energy_sl = 0;
    for (size_t i = 0; i < obs.cir.size(); ++i) {
        const double a = tap_abs(obs.cir, i);
        const bool in_weak = truth.has_weak && i >= truth.weak_excl_lo &&
                             i <= truth.weak_excl_hi;
        if (truth.mainlobe[i]) {
            peak_ml = std::max(peak_ml, a);
            energy_ml += a * a;
        } else if (!in_weak) {
            peak_sl = std::max(peak_sl, a);
            energy_sl += a * a;
        }
    }
    if (peak_ml > 0)
        m.pslr_db = 20.0 * std::log10(std::max(peak_sl, 1e-30) / peak_ml);
    if (energy_ml > 0)
        m.islr_db = 10.0 * std::log10(std::max(energy_sl, 1e-30) / energy_ml);

    // -3 dB main-lobe width around the MEASURED peak, contiguous.
    if (obs.peak < obs.cir.size()) {
        const double half = tap_abs(obs.cir, obs.peak) / std::sqrt(2.0);
        size_t lo = obs.peak, hi = obs.peak;
        while (lo > 0 && tap_abs(obs.cir, lo - 1) >= half)
            --lo;
        while (hi + 1 < obs.cir.size() && tap_abs(obs.cir, hi + 1) >= half)
            ++hi;
        m.width = hi - lo + 1;
    }

    if (truth.has_weak) {
        double best = 0;
        for (size_t i = truth.weak_excl_lo; i <= truth.weak_excl_hi; ++i)
            best = std::max(best, tap_abs(obs.cir, i));
        m.weak_abs = best;
        const double clean_weak = [&] {
            double v = 0;
            for (size_t i = truth.weak_excl_lo; i <= truth.weak_excl_hi; ++i)
                v = std::max(v, tap_abs(clean.cir, i));
            return v;
        }();
        if (clean_weak > 0)
            m.weak_gain_db = 20.0 * std::log10(std::max(best, 1e-30) / clean_weak);
        m.weak_preserved = (m.weak_gain_db > -3.0 && best > 0) ? 1 : 0;
    }
    return m;
}

// ---------------------------------------------------------------------------
// Cells
// ---------------------------------------------------------------------------
struct Cell {
    size_t sync_reps = 64;
    size_t psdu_bytes = 32;
    size_t target_code = 9;
    size_t interferer_code = 10;
    std::string multipath = "single";
    double isr_db = 0.0;
    double target_cfo_hz = 0.0;
    double interferer_cfo_hz = 0.0;

    bool fractional() const { return multipath == "fractional"; }

    std::string code_dir() const
    {
        return std::to_string(target_code) + "<-" + std::to_string(interferer_code);
    }

    std::string id() const
    {
        std::ostringstream s;
        s << "sync" << sync_reps << "_psdu" << psdu_bytes << "_tgt" << target_code
          << "_int" << interferer_code << "_" << multipath << "_isr"
          << std::llround(isr_db) << "_ct" << std::llround(target_cfo_hz) << "_ci"
          << std::llround(interferer_cfo_hz);
        return s.str();
    }
};

struct Options {
    std::vector<size_t> sync_reps{64};
    std::vector<size_t> psdu_bytes{32};
    std::vector<size_t> target_codes{9};
    std::vector<size_t> interferer_codes{10};
    std::vector<std::string> multipath{"single"};
    std::vector<double> isr_db{0.0};
    std::vector<double> target_cfo{0.0};
    std::vector<double> interferer_cfo{0.0};
    unsigned seeds = 16;
    std::string grid = "work";
    std::string taps;
    std::string out = "-";
    std::string stage = "run";
    std::string dump_dir;
    std::string sc16_dir;
    std::string overlap = "sync";
    bool dual = false;
    double weak_delay = kWeakDelay;
    double weak_gain_db = kWeakGainDb;
    bool list_cells = false;
    bool debug_cir = false;
    size_t coarse_stride = 1;
    size_t search_margin = 40960;
};

std::vector<Cell> enumerate(const Options& o)
{
    if (o.sync_reps.size() != o.psdu_bytes.size() &&
        o.sync_reps.size() != 1 && o.psdu_bytes.size() != 1)
        throw std::runtime_error("--sync-reps and --psdu-bytes must zip: equal length or one entry");
    if (o.target_codes.size() != o.interferer_codes.size() &&
        o.target_codes.size() != 1 && o.interferer_codes.size() != 1)
        throw std::runtime_error("--target-code and --interferer-code must zip: equal length or one entry");
    const size_t n_cfg = std::max(o.sync_reps.size(), o.psdu_bytes.size());
    const size_t n_code = std::max(o.target_codes.size(), o.interferer_codes.size());
    std::vector<Cell> cells;
    for (size_t c = 0; c < n_cfg; ++c) {
        const size_t reps = o.sync_reps[o.sync_reps.size() == 1 ? 0 : c];
        const size_t psdu = o.psdu_bytes[o.psdu_bytes.size() == 1 ? 0 : c];
        for (size_t k = 0; k < n_code; ++k) {
            const size_t tgt = o.target_codes[o.target_codes.size() == 1 ? 0 : k];
            const size_t itf = o.interferer_codes[o.interferer_codes.size() == 1 ? 0 : k];
            for (const auto& mp : o.multipath)
                for (double isr : o.isr_db)
                    for (double tc : o.target_cfo)
                        for (double ic : o.interferer_cfo) {
                            Cell cell;
                            cell.sync_reps = reps;
                            cell.psdu_bytes = psdu;
                            cell.target_code = tgt;
                            cell.interferer_code = itf;
                            cell.multipath = mp;
                            cell.isr_db = isr;
                            cell.target_cfo_hz = tc;
                            cell.interferer_cfo_hz = ic;
                            cells.push_back(cell);
                        }
        }
    }
    return cells;
}

// ---------------------------------------------------------------------------
// Mixture construction (shared by run / dump / self-check)
// ---------------------------------------------------------------------------
struct Mixture {
    std::vector<cf> clean; // target channel + noise
    std::vector<cf> raw;   // clean + interferer preamble
    size_t n = 0;
    int64_t q = kQ;
    int64_t iq = kQ;
};

Mixture build_mixture(const Cell& cell, unsigned seed, bool native_phase, bool noise,
                      bool dual, double weak_delay, double weak_gain_db,
                      size_t interference_offset, const Packet& target,
                      const Packet& interferer)
{
    Mixture mix;
    mix.q = int64_t(kQ) + (native_phase ? int64_t(seed % 16) : 0);
    const size_t q = size_t(mix.q);
    mix.iq = int64_t(q + interference_offset);

    size_t n = std::max(q + target.wave.size(),
                        size_t(mix.iq) + interferer.prefix.size()) +
               1024;
    if (cell.fractional() || dual)
        n = std::max(n, q + target.wave.size() + size_t(std::ceil(
                            std::max(kFracDelay, dual ? weak_delay : 0.0))) + 256);
    mix.n = n;
    mix.clean.assign(n, cf(0.f, 0.f));
    mix.raw.assign(n, cf(0.f, 0.f));

    // Target channel: direct path at q with the target carrier offset.
    std::copy(target.wave.begin(), target.wave.end(), mix.clean.begin() + long(q));
    apply_cfo(mix.clean.data() + q, target.wave.size(), cell.target_cfo_hz, 0.0);

    // Fractional multipath: a delayed copy of the TRANSMITTED (already
    // CFO-rotated) waveform, so the channel acts on the real signal.
    if (cell.fractional()) {
        std::vector<cf> tx(target.wave.size());
        std::copy(mix.clean.begin() + long(q), mix.clean.begin() + long(q + tx.size()),
                  tx.begin());
        auto echo = fractional_delay(tx, kFracDelay);
        const cf gain(float(kFracGain * std::cos(kFracPhase)),
                      float(kFracGain * std::sin(kFracPhase)));
        for (size_t j = 0; j < echo.size() && q + j < n; ++j)
            mix.clean[q + j] += gain * echo[j];
    }

    // Dual-target special case: one weak echo of the same packet.
    if (dual) {
        std::vector<cf> tx(target.wave.size());
        std::copy(mix.clean.begin() + long(q), mix.clean.begin() + long(q + tx.size()),
                  tx.begin());
        auto echo = fractional_delay(tx, weak_delay);
        const double g = std::pow(10.0, weak_gain_db / 20.0);
        const cf gain(float(g * std::cos(kWeakPhase)), float(g * std::sin(kWeakPhase)));
        for (size_t j = 0; j < echo.size() && q + j < n; ++j)
            mix.clean[q + j] += gain * echo[j];
    }

    if (noise) {
        std::mt19937 rng(3000 + seed);
        std::normal_distribution<float> normal(0, 1);
        const double sigma = std::sqrt(energy(target.wave) / double(target.wave.size()) *
                                       std::pow(10.0, -kSnrDb / 10.0) / 2.0);
        for (auto& z : mix.clean)
            z += cf(normal(rng) * float(sigma), normal(rng) * float(sigma));
    }
    mix.raw = mix.clean;

    // Interferer preamble (SYNC + SFD) with its own independent carrier offset.
    double es = 0;
    for (size_t j = 0; j < std::min<size_t>(64 * kSps, target.wave.size()); ++j)
        es += std::norm(target.wave[j]);
    double ei = 0;
    for (size_t j = 0; j < std::min<size_t>(64 * kSps, interferer.prefix.size()); ++j)
        ei += std::norm(interferer.prefix[j]);
    if (ei <= 0)
        throw std::runtime_error("interferer prefix has no energy");
    const double gain = std::sqrt(es / ei * std::pow(10.0, cell.isr_db / 10.0));
    std::vector<cf> itf(interferer.prefix);
    apply_cfo(itf.data(), itf.size(), cell.interferer_cfo_hz, kIntPhase);
    for (size_t j = 0; j < itf.size() && size_t(mix.iq) + j < n; ++j)
        mix.raw[size_t(mix.iq) + j] += itf[j] * float(gain);
    return mix;
}

// One generated scene: both packets plus the paired clean/raw buffers.
struct Scene {
    Packet target;
    Packet interferer;
    Mixture mix;

    Scene(const Cell& cell, unsigned seed, bool native_phase, bool noise, bool dual,
          double weak_delay, double weak_gain_db, size_t interference_offset)
        : target(cell.target_code, 1000 + seed, cell.sync_reps, cell.psdu_bytes),
          interferer(cell.interferer_code, 2000 + seed, cell.sync_reps, cell.psdu_bytes)
    {
        mix = build_mixture(cell, seed, native_phase, noise, dual, weak_delay,
                            weak_gain_db, interference_offset, target, interferer);
    }
};

size_t interference_offset_for(const Options& opt)
{
    // `sync` is the plan §4.2 case (preamble-on-preamble).  `payload` keeps the
    // historical extra scenario available as a diagnostic only.
    return opt.overlap == "payload" ? 104 * kSps : 0;
}

// ---------------------------------------------------------------------------
// CSV row
// ---------------------------------------------------------------------------
const char* kCsvHeader =
    "case_id,grid,code_dir,sync_reps,psdu_bytes,target_code,interferer_code,multipath,"
    "dual_target,isr_db,target_cfo_hz,interferer_cfo_hz,seed,path,"
    "demod_status,demod_status_name,sfd_ok,phr_ok,psdu_len,fcs_pass,psdu_exact,"
    "radar_status,radar_status_name,detected_start,cir_axis_origin,cir_pre,"
    "measured_cfo_hz,radar_cfo_hz,cfo_epoch,"
    "peak_tap,expected_peak_tap,peak_shift,peak_abs,peak_gain_db,relative_l2,"
    "pslr_db,islr_db,mainlobe_width_taps,mainlobe_lo_tap,mainlobe_hi_tap,mainlobe_taps,"
    "weak_peak_tap,weak_peak_abs,weak_gain_vs_clean_db,weak_preserved,"
    "sic_committed,sic_reason,residual_improvement_db,failure_reason";

struct SicInfo {
    bool committed = false;
    std::string reason = "not_run";
    double residual_db = 0.0;
};

std::string failure_text(const Observation& obs)
{
    if (obs.demod_status < 0 && obs.radar_status < 0)
        return std::string(); // nothing ran for this row
    std::vector<std::string> parts;
    if (obs.demod_status > 0)
        parts.push_back(std::string("demod:") + demod_status_name(obs.demod_status));
    if (!obs.sfd)
        parts.push_back("sfd");
    if (!obs.phr)
        parts.push_back("phr");
    if (!obs.fcs)
        parts.push_back("fcs");
    if (!obs.psdu_exact)
        parts.push_back("psdu");
    if (obs.radar_status > 0) {
        std::string r = std::string("radar:") + radar_status_name(obs.radar_status);
        if (obs.radar_reason && obs.radar_reason[0])
            r += std::string("(") + obs.radar_reason + ")";
        parts.push_back(r);
    }
    std::string out;
    for (size_t i = 0; i < parts.size(); ++i) {
        if (i)
            out += ";";
        out += parts[i];
    }
    return out;
}

std::string combined_failure(const Observation& obs, const std::string& extra)
{
    std::string text = failure_text(obs);
    if (extra.empty())
        return text;
    if (!text.empty())
        text += ";";
    return text + extra;
}


void emit_row(std::ostream& os, const Cell& cell, const std::string& grid, unsigned seed,
              const char* path, const Observation& obs, const Truth& truth,
              const Metrics& m, const SicInfo& sic, bool dual,
              const std::string& extra_failure = std::string())
{
    os << cell.id() << ',' << grid << ',' << cell.code_dir() << ',' << cell.sync_reps << ','
       << cell.psdu_bytes << ',' << cell.target_code << ',' << cell.interferer_code << ','
       << cell.multipath << ',' << (dual ? 1 : 0) << ',' << cell.isr_db << ','
       << cell.target_cfo_hz << ',' << cell.interferer_cfo_hz << ',' << seed << ',' << path
       << ',' << obs.demod_status << ',' << demod_status_name(obs.demod_status) << ','
       << (obs.sfd ? 1 : 0) << ',' << (obs.phr ? 1 : 0) << ',' << obs.psdu_len << ','
       << (obs.fcs ? 1 : 0) << ',' << (obs.psdu_exact ? 1 : 0) << ',' << obs.radar_status
       << ',' << radar_status_name(obs.radar_status) << ',' << obs.detected << ','
       << obs.cir_axis_origin << ',' << kCirPre << ',' << obs.measured_cfo_hz << ','
       << obs.radar_cfo_hz << ',' << obs.cfo_epoch << ',' << obs.peak << ','
       << truth.expected_peak << ',' << m.peak_shift << ',' << obs.peak_abs << ','
       << m.gain_db << ',' << m.relative_l2 << ',' << m.pslr_db << ',' << m.islr_db << ','
       << m.width << ',' << truth.ml_lo << ',' << truth.ml_hi << ',' << truth.ml_taps << ','
       << (truth.has_weak ? long(truth.weak_tap) : -1) << ',' << m.weak_abs << ','
       << m.weak_gain_db << ',' << (truth.has_weak ? m.weak_preserved : -1) << ','
       << (sic.committed ? 1 : 0) << ',' << sic.reason << ',' << sic.residual_db << ','
       << combined_failure(obs, extra_failure) << '\n';
}

// ---------------------------------------------------------------------------
// SIC (preamble-only, interferer code)
// ---------------------------------------------------------------------------
SicInfo run_sic(const std::vector<cf>& raw, size_t n, const Packet& interferer,
                int64_t interference_start, size_t sync_reps, std::vector<cf>& out,
                std::vector<cf>& ihat)
{
    sic::SicUnifiedConfig cfg;
    cfg.policy = sic::SicPolicy::PreambleOnly;
    cfg.backend = sic::SicEstimatorBackend::Ipatov;
    cfg.comm_code = interferer.cfg.code_index;
    cfg.samples_per_symbol = kSps;
    cfg.sync_reps = sync_reps;
    cfg.sample_rate = kFs;
    cfg.roi_lo = interference_start - 64;
    cfg.roi_hi = interference_start + 64;
    cfg.sub_roi_lo = 0;
    cfg.sub_roi_hi = int64_t(n);
    cfg.max_window_samples = n;
    cfg.max_packet_samples = n;
    sic::SicUnifiedTemplates t;
    t.sync = interferer.sync.data();
    t.sync_len = kSps;
    t.preamble = interferer.prefix.data();
    t.preamble_len = interferer.prefix.size();
    sic::SicUnifiedScratch scratch;
    std::string error;
    if (!sic::prepare_sic_unified(cfg, t, scratch, &error))
        throw std::runtime_error("SIC prepare failed: " + error);
    sic::SicUnifiedResult result;
    result.out = out.data();
    result.i_hat = ihat.data();
    SicInfo info;
    if (!sic::sic_process_unified(raw.data(), n, t, cfg, scratch, result)) {
        info.reason = "api_failure";
        return info;
    }
    info.committed = result.committed;
    info.reason = result.reason ? result.reason : "unknown";
    return info;
}

double residual_improvement_db(const std::vector<cf>& raw, const std::vector<cf>& out,
                               const std::vector<cf>& clean, size_t n)
{
    double before = 0, after = 0;
    for (size_t j = 0; j < n; ++j) {
        before += std::norm(std::complex<double>(raw[j] - clean[j]));
        after += std::norm(std::complex<double>(out[j] - clean[j]));
    }
    return 10.0 * std::log10(std::max(before, 1e-30) / std::max(after, 1e-30));
}

// ---------------------------------------------------------------------------
// Native 65/48 chain helpers
// ---------------------------------------------------------------------------
std::vector<float> load_taps(const std::string& path)
{
    std::ifstream f(path, std::ios::binary | std::ios::ate);
    if (!f)
        throw std::runtime_error("cannot open taps: " + path);
    const size_t bytes = size_t(f.tellg());
    if (bytes == 0 || bytes % sizeof(float))
        throw std::runtime_error("invalid tap file size: " + path);
    std::vector<float> taps(bytes / sizeof(float));
    f.seekg(0);
    f.read(reinterpret_cast<char*>(taps.data()), long(bytes));
    if (!f)
        throw std::runtime_error("truncated tap file: " + path);
    return taps;
}

struct Sc16Source {
    uint64_t count = 0;
    double inverse_scale = 0.0;
    std::vector<std::complex<int16_t>> iq;
};

Sc16Source load_sc16(const std::string& path)
{
    std::ifstream f(path, std::ios::binary);
    if (!f)
        throw std::runtime_error("cannot open sc16 source: " + path);
    Sc16Source s;
    f.read(reinterpret_cast<char*>(&s.count), sizeof(s.count));
    f.read(reinterpret_cast<char*>(&s.inverse_scale), sizeof(s.inverse_scale));
    if (!f || s.count == 0 || s.count > 20000000 || !(s.inverse_scale > 0))
        throw std::runtime_error("invalid sc16 header: " + path);
    s.iq.resize(size_t(s.count));
    f.read(reinterpret_cast<char*>(s.iq.data()), long(s.count * sizeof(s.iq[0])));
    if (!f)
        throw std::runtime_error("truncated sc16 payload: " + path);
    return s;
}

void resample_native(const std::vector<float>& taps,
                     const std::complex<int16_t>* iq,
                     uint64_t count,
                     double inverse_scale,
                     std::vector<cf>& out)
{
    core::RationalResampler65_48Core resampler(taps);
    resampler.set_num_workers(1);
    resampler.set_fixed_phase_schedule(true);
    const size_t expected = resampler.expected_output_length(count);
    out.resize(expected);
    resampler.reset();
    auto r = resampler.process_sc16(iq, count, float(inverse_scale), out.data(), out.size());
    if (r.consumed != count)
        throw std::runtime_error("native input not fully consumed");
    size_t produced = r.produced;
    while (produced < expected) {
        const size_t z = resampler.flush(out.data() + produced, expected - produced);
        if (!z)
            break;
        produced += z;
    }
    if (produced != expected)
        throw std::runtime_error("resampler output length mismatch");
}

// Legacy single-file SC16 source: header (count, inverse_scale) followed by TWO
// consecutive blocks of `count` samples (clean, then raw).  Preserved verbatim
// because `tools/sic/run_native_preamble_recovery.py` writes exactly this.
struct Sc16Pair {
    uint64_t count = 0;
    double inverse_scale = 0.0;
    std::vector<std::complex<int16_t>> iq;
};

Sc16Pair load_sc16_pair(const std::string& path)
{
    std::ifstream f(path, std::ios::binary);
    if (!f)
        throw std::runtime_error("cannot open sc16 source: " + path);
    Sc16Pair s;
    f.read(reinterpret_cast<char*>(&s.count), sizeof(s.count));
    f.read(reinterpret_cast<char*>(&s.inverse_scale), sizeof(s.inverse_scale));
    if (!f || s.count == 0 || s.count > 20000000 || !(s.inverse_scale > 0))
        throw std::runtime_error("invalid native header: " + path);
    s.iq.resize(size_t(2 * s.count));
    f.read(reinterpret_cast<char*>(s.iq.data()), long(s.iq.size() * sizeof(s.iq[0])));
    if (!f)
        throw std::runtime_error("truncated native input: " + path);
    return s;
}

// ---------------------------------------------------------------------------
// One cell
// ---------------------------------------------------------------------------
void run_cell(const Cell& cell, const Options& opt, std::ostream& csv)
{
    const bool native = opt.grid == "native";
    const size_t reps = cell.sync_reps;
    const Truth truth = make_truth(cell.fractional(), opt.dual, opt.weak_delay);
    const size_t offset = interference_offset_for(opt);
    std::vector<float> taps;
    if (native)
        taps = load_taps(opt.taps);

    for (unsigned seed = 0; seed < opt.seeds; ++seed) {
        try {
            Scene scene(cell, seed, native, true, opt.dual, opt.weak_delay, opt.weak_gain_db,
                        offset);
            std::vector<cf> clean_buf = scene.mix.clean;
            std::vector<cf> raw_buf = scene.mix.raw;
            size_t n = scene.mix.n;
            int64_t predicted = scene.mix.q;
            int64_t interference_start = scene.mix.iq;

            Observation ideal;
            if (native) {
                ideal = observe(clean_buf, scene.target, predicted, opt.coarse_stride,
                                opt.search_margin);
                const std::string stem =
                    opt.sc16_dir + "/" + cell.id() + "_" + std::to_string(seed);
                Sc16Source clean_src = load_sc16(stem + "_clean.sc16");
                Sc16Source raw_src = load_sc16(stem + "_raw.sc16");
                resample_native(taps, clean_src.iq.data(), clean_src.count,
                                clean_src.inverse_scale, clean_buf);
                resample_native(taps, raw_src.iq.data(), raw_src.count, raw_src.inverse_scale,
                                raw_buf);
                n = clean_buf.size();
                if (raw_buf.size() != n)
                    throw std::runtime_error("native clean/raw length mismatch");
                // Public nominal coordinates shifted by the known FIR group delay;
                // the front-end refines the fractional residual from detection.
                const double delay = double(taps.size() - 1) / 96.0;
                predicted = std::llround(double(scene.mix.q) + delay);
                interference_start = std::llround(double(scene.mix.iq) + delay);
            }

            std::vector<cf> out(n), ihat(n);
            SicInfo sic =
                run_sic(raw_buf, n, scene.interferer, interference_start, reps, out, ihat);
            sic.residual_db = residual_improvement_db(raw_buf, out, clean_buf, n);

            const Observation clean_obs =
                observe(clean_buf, scene.target, predicted, opt.coarse_stride, opt.search_margin);
            const Observation raw_obs =
                observe(raw_buf, scene.target, predicted, opt.coarse_stride, opt.search_margin);
            const Observation sic_obs =
                observe(out, scene.target, predicted, opt.coarse_stride, opt.search_margin);

            if (native)
                emit_row(csv, cell, opt.grid, seed, "ideal", ideal, truth,
                         compute_metrics(ideal, ideal, truth), SicInfo{}, opt.dual);
            emit_row(csv, cell, opt.grid, seed, "clean", clean_obs, truth,
                     compute_metrics(clean_obs, clean_obs, truth), SicInfo{}, opt.dual);
            emit_row(csv, cell, opt.grid, seed, "raw", raw_obs, truth,
                     compute_metrics(raw_obs, clean_obs, truth), SicInfo{}, opt.dual);
            emit_row(csv, cell, opt.grid, seed, "sic", sic_obs, truth,
                     compute_metrics(sic_obs, clean_obs, truth), sic, opt.dual);
            csv.flush();
        } catch (const std::exception& e) {
            // A failing seed is never dropped: emit the full row set with an
            // explicit status and reason so the cell stays countable.
            Observation bad;
            bad.demod_status = -1;
            bad.radar_status = -1;
            const std::string why = std::string("exception:") + e.what();
            SicInfo failed;
            failed.reason = "exception";
            const Metrics none;
            if (native)
                emit_row(csv, cell, opt.grid, seed, "ideal", bad, truth, none, failed,
                         opt.dual, why);
            emit_row(csv, cell, opt.grid, seed, "clean", bad, truth, none, failed, opt.dual,
                     why);
            emit_row(csv, cell, opt.grid, seed, "raw", bad, truth, none, failed, opt.dual,
                     why);
            emit_row(csv, cell, opt.grid, seed, "sic", bad, truth, none, failed, opt.dual,
                     why);
            csv.flush();
        }
    }
}

// ---------------------------------------------------------------------------
// Dump stage (native grid source synthesis)
// ---------------------------------------------------------------------------
void dump_cell(const Cell& cell, const Options& opt)
{
    const size_t offset = interference_offset_for(opt);
    for (unsigned seed = 0; seed < opt.seeds; ++seed) {
        Scene scene(cell, seed, true, true, opt.dual, opt.weak_delay, opt.weak_gain_db,
                    offset);
        const std::string path =
            opt.dump_dir + "/" + cell.id() + "_" + std::to_string(seed) + ".cf32";
        std::ofstream f(path, std::ios::binary);
        if (!f)
            throw std::runtime_error("cannot write source dump: " + path);
        f.write(reinterpret_cast<const char*>(scene.mix.clean.data()),
                long(scene.mix.clean.size() * sizeof(cf)));
        f.write(reinterpret_cast<const char*>(scene.mix.raw.data()),
                long(scene.mix.raw.size() * sizeof(cf)));
        if (!f)
            throw std::runtime_error("source dump failed: " + path);
    }
}

// ---------------------------------------------------------------------------
// Self-check (unit-style assertions on the generator and the receiver)
// ---------------------------------------------------------------------------
int self_check(const Options& opt)
{
    int failures = 0;
    auto check = [&](bool ok, const std::string& what, const std::string& detail) {
        std::cout << (ok ? "PASS " : "FAIL ") << what << " : " << detail << '\n';
        if (!ok)
            ++failures;
    };

    // 1. Cell enumeration must be exactly 432 cells / 6912 seed cases.
    {
        Options m = opt;
        m.sync_reps = {64, 256};
        m.psdu_bytes = {32, 127};
        m.target_codes = {9, 10};
        m.interferer_codes = {10, 9};
        m.multipath = {"single", "fractional"};
        m.isr_db = {0, 10, 20};
        m.target_cfo = {0, 50000, -50000};
        m.interferer_cfo = {0, 50000, -50000};
        m.seeds = 16;
        const auto cells = enumerate(m);
        // The grid axis (work / native) is a RUN dimension: the C++ tool
        // enumerates one grid per invocation, the Python runner doubles it.
        const size_t pairs = cells.size() * 2;
        check(pairs == 432, "matrix_cell_count",
              std::to_string(cells.size()) + " cells/grid x 2 grids = " +
                  std::to_string(pairs) + " cell-grid pairs");
        check(pairs * m.seeds == 6912, "matrix_seed_cases",
              std::to_string(pairs * m.seeds) + " seed cases");
    }

    // 2. Target CFO must produce a measurable phase slope on the target SYNC
    //    train, with the right sign, and must not leak from the interferer.
    for (double cfo : { 0.0, 50000.0, -50000.0 }) {
        Cell cell;
        cell.target_cfo_hz = cfo;
        cell.isr_db = 0.0;
        Scene scene(cell, 0, false, false, false, kWeakDelay, kWeakGainDb, 0);
        const Observation obs = observe(scene.mix.clean, scene.target, scene.mix.q, 1, 40960);
        const double err = std::abs(obs.measured_cfo_hz - cfo);
        std::ostringstream d;
        d << "applied " << cfo << " Hz, measured " << obs.measured_cfo_hz << " Hz";
        check(std::isfinite(obs.measured_cfo_hz) && err <= 3000.0, "target_cfo_slope", d.str());
    }
    // The two carrier offsets are independent: the target estimate must follow
    // the TARGET offset and ignore the interferer's, in both directions.
    for (auto pair : { std::pair<double, double>{ 0.0, 50000.0 },
                       std::pair<double, double>{ 50000.0, -50000.0 },
                       std::pair<double, double>{ -50000.0, 50000.0 } }) {
        Cell cell;
        cell.target_cfo_hz = pair.first;
        cell.interferer_cfo_hz = pair.second;
        cell.isr_db = 0.0;
        Scene scene(cell, 0, false, false, false, kWeakDelay, kWeakGainDb, 0);
        const Observation obs = observe(scene.mix.raw, scene.target, scene.mix.q, 1, 40960);
        std::ostringstream d;
        d << "target " << pair.first << " Hz, interferer " << pair.second
          << " Hz, measured " << obs.measured_cfo_hz << " Hz";
        check(std::isfinite(obs.measured_cfo_hz) &&
                  std::abs(obs.measured_cfo_hz - pair.first) <= 3000.0,
              "independent_cfo_no_leak", d.str());
    }
    {
        // At 20 dB ISR the target preamble is masked and acquisition fails
        // closed: the receiver must publish NO CFO rather than the interferer's.
        Cell cell;
        cell.target_cfo_hz = 0.0;
        cell.interferer_cfo_hz = 50000.0;
        cell.isr_db = 20.0;
        Scene scene(cell, 0, false, false, false, kWeakDelay, kWeakGainDb, 0);
        const Observation obs = observe(scene.mix.raw, scene.target, scene.mix.q, 1, 40960);
        std::ostringstream d;
        d << "demod " << demod_status_name(obs.demod_status) << ", measured "
          << obs.measured_cfo_hz << " Hz";
        check(obs.demod_status != 0 && !std::isfinite(obs.measured_cfo_hz),
              "masked_target_fails_closed", d.str());
    }

    // 3. Fractional multipath: the +7.3 sample path must land on tap pre+7 with
    //    the documented 0.6 amplitude, and be absent in the single-path cell.
    {
        Cell single;
        Scene a(single, 0, false, false, false, kWeakDelay, kWeakGainDb, 0);
        Cell frac;
        frac.multipath = "fractional";
        Scene b(frac, 0, false, false, false, kWeakDelay, kWeakGainDb, 0);
        const Observation oa = observe(a.mix.clean, a.target, a.mix.q, 1, 40960);
        const Observation ob = observe(b.mix.clean, b.target, b.mix.q, 1, 40960);
        // The second path is at lag kAxisOffset + 7.3, i.e. tap pre+9.
        const size_t frac_tap = kCirPre + kAxisOffset + size_t(std::llround(kFracDelay));
        const double tap_single = tap_abs(oa.cir, frac_tap);
        const double peak_single = tap_abs(oa.cir, oa.peak);
        const double tap_frac = tap_abs(ob.cir, frac_tap);
        const double peak_frac = tap_abs(ob.cir, ob.peak);
        const double ratio = tap_frac / std::max(peak_frac, 1e-30);
        const double single_ratio = tap_single / std::max(peak_single, 1e-30);
        std::ostringstream d;
        d << "single path tap" << frac_tap << "/peak=" << single_ratio
          << " fractional tap" << frac_tap << "/peak=" << ratio << " (expect 0.6)";
        check(std::abs(ratio - kFracGain) <= 0.12 && single_ratio < 0.1,
              "fractional_multipath_delay_axis", d.str());
        check(ob.peak == size_t(kCirPre + kAxisOffset), "fractional_multipath_peak_tap",
              "peak tap " + std::to_string(ob.peak) + " (expect " +
                  std::to_string(kCirPre + kAxisOffset) + ")");
    }

    // 4. ISR definition: measured interferer/target power ratio over the first
    //    64 SYNC symbols must match the configured label.
    for (double isr : { 0.0, 10.0, 20.0 }) {
        Cell cell;
        cell.isr_db = isr;
        Scene scene(cell, 0, false, false, false, kWeakDelay, kWeakGainDb, 0);
        double et = 0, ei = 0;
        for (size_t j = 0; j < 64 * kSps; ++j) {
            et += std::norm(std::complex<double>(scene.mix.clean[scene.mix.q + j]));
            ei += std::norm(std::complex<double>(scene.mix.raw[scene.mix.iq + j] -
                                                scene.mix.clean[scene.mix.iq + j]));
        }
        const double measured = 10.0 * std::log10(ei / std::max(et, 1e-30));
        std::ostringstream d;
        d << "configured " << isr << " dB, measured " << measured << " dB";
        check(std::abs(measured - isr) <= 0.5, "isr_definition", d.str());
    }

    // 5. Dual-target protection path must place the weak echo on its tap.
    {
        Cell cell;
        Scene scene(cell, 0, false, false, true, kWeakDelay, kWeakGainDb, 0);
        const Observation obs = observe(scene.mix.clean, scene.target, scene.mix.q, 1, 40960);
        const size_t tap =
            kCirPre + kAxisOffset + size_t(std::llround(kWeakDelay));
        const double weak = tap_abs(obs.cir, tap);
        const double strong = tap_abs(obs.cir, kCirPre + kAxisOffset);
        const double ratio_db =
            20.0 * std::log10(std::max(weak, 1e-30) / std::max(strong, 1e-30));
        std::ostringstream d;
        d << "weak tap " << tap << " ratio " << ratio_db << " dB (expect " << kWeakGainDb
          << " dB)";
        check(std::abs(ratio_db - kWeakGainDb) <= 2.0, "dual_target_weak_echo", d.str());
    }

    std::cout << (failures ? "SELF-CHECK FAILED: " : "SELF-CHECK PASSED: ") << failures
              << " failure(s)\n";
    return failures ? 1 : 0;
}

// ---------------------------------------------------------------------------
// Legacy positional CLI (kept byte-compatible for the two external runners)
// ---------------------------------------------------------------------------
int run_legacy(int argc, char** argv)
{
    unsigned seeds = std::stoul(argv[1]);
    const std::string mode = argc > 2 ? argv[2] : "work";
    const std::string directory = argc > 3 ? argv[3] : "";
    const size_t target_reps = mode == "dump" && argc > 5 ? std::stoull(argv[5]) : 64;
    const size_t psdu_bytes = mode == "dump" && argc > 6 ? std::stoull(argv[6]) : 32;
    const size_t interference_reps = mode == "dump" && argc > 7 ? std::stoull(argv[7]) : 64;
    std::vector<float> taps;
    if (mode == "native") {
        if (argc != 5)
            throw std::runtime_error("native mode requires directory and tap file");
        taps = load_taps(argv[4]);
    } else if (mode != "dump" && mode != "work") {
        throw std::runtime_error("unknown mode");
    }
    std::cout << "role,scenario,seed,isr_db,cfo_hz,path,demod_status,sfd,fcs,psdu_exact,"
                 "radar_status,detected_start,cir_peak_shift,cir_peak_gain_db,"
                 "cir_relative_l2,sic_committed,sic_reason,residual_improvement_db\n";
    struct Scenario {
        const char* name;
        double isr, cfo;
        size_t offset;
        bool interference, noise;
    };
    const Scenario scenarios[] = {
        { "clean_control", 0, 0, 0, false, false },
        { "noise_control", 0, 0, 0, false, true },
        { "sync_isr0", 0, 0, 0, true, true },
        { "sync_isr10", 10, 0, 0, true, true },
        { "sync_isr20", 20, 0, 0, true, true },
        { "sync_isr20_cfo50k", 20, 50000, 0, true, true },
        // SYNC+SFD ends at 72*sps, PHR at about 94*sps; start interference at
        // 104*sps so this cell overlaps the actual payload.
        { "payload_isr20", 20, 0, 104 * kSps, true, true }
    };
    for (size_t code : { size_t(9), size_t(10) }) {
        for (unsigned seed = 0; seed < seeds; ++seed) {
            const size_t q = kQ + (mode == "work" ? 0 : seed % 16);
            Packet target(code, 1000 + seed, target_reps, psdu_bytes);
            Packet interferer(code == 9 ? 10 : 9, 2000 + seed, interference_reps, psdu_bytes);
            if (mode == "dump") {
                std::ofstream sync(directory + "/sync_" + std::to_string(code) + ".bin",
                                   std::ios::binary);
                sync.write(reinterpret_cast<const char*>(target.sync.data()),
                           long(target.sync.size() * sizeof(cf)));
                std::ofstream psdu(directory + "/psdu_" + std::to_string(code) + "_" +
                                       std::to_string(seed) + ".bin",
                                   std::ios::binary);
                psdu.write(reinterpret_cast<const char*>(target.bytes.data()),
                           long(target.bytes.size()));
            }
            for (const auto& sc : scenarios) {
                if (mode == "dump" && argc > 4 && std::string(sc.name) != argv[4])
                    continue;
                const size_t iq = q + sc.offset;
                size_t n = std::max(q + target.wave.size(), iq + interferer.prefix.size()) + 1024;
                std::vector<cf> clean(n), raw(n), out(n), ihat(n);
                std::copy(target.wave.begin(), target.wave.end(), clean.begin() + long(q));
                std::mt19937 rng(3000 + seed);
                std::normal_distribution<float> normal(0, 1);
                const double sigma =
                    sc.noise ? std::sqrt(energy(target.wave) / double(target.wave.size()) *
                                         0.01 / 2.0)
                             : 0.0;
                for (auto& z : clean)
                    z += cf(normal(rng) * float(sigma), normal(rng) * float(sigma));
                raw = clean;
                double es = 0;
                for (size_t j = 0; j < 64 * kSps; ++j)
                    es += std::norm(target.wave[j]);
                double ei = 0;
                for (size_t j = 0; j < 64 * kSps; ++j)
                    ei += std::norm(interferer.prefix[j]);
                const double gain = std::sqrt(es / ei * std::pow(10.0, sc.isr / 10.0));
                if (sc.interference) {
                    std::vector<cf> itf(interferer.prefix);
                    apply_cfo(itf.data(), itf.size(), sc.cfo, kIntPhase);
                    for (size_t j = 0; j < itf.size(); ++j)
                        raw[iq + j] += itf[j] * float(gain);
                }
                const std::string stem = directory + "/" + std::to_string(code) + "_" +
                                         std::to_string(seed) + "_" + sc.name;
                if (mode == "dump") {
                    std::ofstream file(stem + ".cf32", std::ios::binary);
                    file.write(reinterpret_cast<const char*>(clean.data()),
                               long(clean.size() * sizeof(cf)));
                    file.write(reinterpret_cast<const char*>(raw.data()),
                               long(raw.size() * sizeof(cf)));
                    if (!file)
                        throw std::runtime_error("source dump failed");
                    continue;
                }
                int64_t predicted = int64_t(q);
                int64_t interference_start = int64_t(iq);
                Observation ideal;
                if (mode == "native") {
                    ideal = observe(clean, target, predicted, 1, 40960);
                    Sc16Pair src = load_sc16_pair(stem + ".sc16");
                    resample_native(taps, src.iq.data(), src.count, src.inverse_scale,
                                    clean);
                    resample_native(taps, src.iq.data() + src.count, src.count,
                                    src.inverse_scale, raw);
                    n = clean.size();
                    out.resize(n);
                    ihat.resize(n);
                    const double delay = double(taps.size() - 1) / 96.0;
                    predicted = std::llround(double(q) + delay);
                    interference_start = std::llround(double(iq) + delay);
                }
                SicInfo sic = run_sic(raw, n, interferer, interference_start,
                                      interference_reps, out, ihat);
                const double suppression =
                    sc.interference ? residual_improvement_db(raw, out, clean, n) : 0.0;
                const char* role = code == 9 ? "sensing9" : "comm10";
                const Observation baseline = observe(clean, target, predicted, 1, 40960);
                auto legacy_emit = [&](const char* path, const Observation& o,
                                       const Observation& ref, bool committed,
                                       const char* reason, double residual) {
                    const Metrics m = compute_metrics(o, ref, make_truth(false, false));
                    // The legacy schema uses -999 for "not measurable", not nan.
                    auto legacy = [](double v) {
                        return std::isfinite(v) ? v : -999.0;
                    };
                    std::cout << role << ',' << sc.name << ',' << seed << ',' << sc.isr << ','
                              << sc.cfo << ',' << path << ',' << o.demod_status << ','
                              << (o.sfd ? 1 : 0) << ',' << (o.fcs ? 1 : 0) << ','
                              << (o.psdu_exact ? 1 : 0) << ',' << o.radar_status << ','
                              << o.detected << ',' << legacy(m.peak_shift) << ','
                              << legacy(m.gain_db) << ',' << legacy(m.relative_l2) << ','
                              << (committed ? 1 : 0) << ',' << reason << ',' << residual
                              << '\n';
                };
                if (mode == "native")
                    legacy_emit("ideal", ideal, ideal, false, "not_run", 0);
                legacy_emit("clean", baseline, mode == "native" ? ideal : baseline, false,
                            "not_run", 0);
                legacy_emit("raw", observe(raw, target, predicted, 1, 40960), baseline, false,
                            "not_run", 0);
                legacy_emit("sic", observe(out, target, predicted, 1, 40960), baseline,
                            sic.committed, sic.reason.c_str(), suppression);
            }
        }
    }
    return 0;
}

// ---------------------------------------------------------------------------
// CLI
// ---------------------------------------------------------------------------
void usage()
{
    std::cout <<
        "usage: experiment_preamble_recovery [options]\n"
        "  --sync-reps N[,N]        SYNC repetitions per config (zips with --psdu-bytes)\n"
        "  --psdu-bytes N[,N]       PSDU length INCLUDING the 2-byte FCS (zips)\n"
        "  --target-code N[,N]      target preamble code (zips with --interferer-code)\n"
        "  --interferer-code N[,N]  interferer preamble code (zips)\n"
        "  --multipath LIST         single | fractional (0.6*e^{j0.7} at +7.3 samples)\n"
        "  --isr-db LIST            interferer/target power ratio over 64 SYNC symbols\n"
        "  --target-cfo-hz LIST     target carrier offset (product axis)\n"
        "  --interferer-cfo-hz LIST interferer carrier offset (product axis)\n"
        "  --seeds N                seed cases per cell (default 16)\n"
        "  --grid work|native       work grid or the real 65/48 resampling chain\n"
        "  --taps FILE              native 65/48 tap file (raw float32)\n"
        "  --out FILE               CSV output ('-' = stdout)\n"
        "  --stage run|dump         run the pipeline, or dump native source pairs\n"
        "  --dump-dir DIR           dump destination (--stage dump)\n"
        "  --sc16-dir DIR           native 737.28 MS/s SC16 source directory\n"
        "  --overlap sync|payload   interferer placement (matrix uses sync)\n"
        "  --dual-target            add a -20 dB weak echo at +40 samples\n"
        "  --weak-delay-samples D   dual-target delay (default 40)\n"
        "  --weak-gain-db G         dual-target gain (default -20)\n"
        "  --coarse-stride N        timing coarse search stride (default 1)\n"
        "  --search-margin N        timing search margin in samples (default 40960)\n"
        "  --list-cells             print the cell enumeration and exit\n"
        "  --debug-cir              print the clean CIR of the first cell and exit\n"
        "  --self-check             run the generator/receiver assertions and exit\n"
        "  --smoke                  representative subset of the matrix\n"
        "\n"
        "Lists accept repeated flags or comma-separated values.\n";
}

Options parse_cli(int argc, char** argv)
{
    Options o;
    bool seen_sync = false, seen_psdu = false, seen_tgt = false, seen_itf = false;
    bool seen_mp = false, seen_isr = false, seen_tc = false, seen_ic = false;
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        auto need = [&](const char* name) -> std::string {
            if (i + 1 >= argc)
                throw std::runtime_error(std::string("missing value for ") + name);
            return argv[++i];
        };
        if (a == "--sync-reps") {
            if (!seen_sync) { o.sync_reps.clear(); seen_sync = true; }
            for (const auto& p : split_list(need("--sync-reps")))
                o.sync_reps.push_back(std::stoull(p));
        } else if (a == "--psdu-bytes") {
            if (!seen_psdu) { o.psdu_bytes.clear(); seen_psdu = true; }
            for (const auto& p : split_list(need("--psdu-bytes")))
                o.psdu_bytes.push_back(std::stoull(p));
        } else if (a == "--target-code") {
            if (!seen_tgt) { o.target_codes.clear(); seen_tgt = true; }
            for (const auto& p : split_list(need("--target-code")))
                o.target_codes.push_back(std::stoull(p));
        } else if (a == "--interferer-code") {
            if (!seen_itf) { o.interferer_codes.clear(); seen_itf = true; }
            for (const auto& p : split_list(need("--interferer-code")))
                o.interferer_codes.push_back(std::stoull(p));
        } else if (a == "--multipath") {
            if (!seen_mp) { o.multipath.clear(); seen_mp = true; }
            for (const auto& p : split_list(need("--multipath"))) {
                if (p != "single" && p != "fractional")
                    throw std::runtime_error("--multipath must be single or fractional");
                o.multipath.push_back(p);
            }
        } else if (a == "--isr-db") {
            if (!seen_isr) { o.isr_db.clear(); seen_isr = true; }
            for (const auto& p : split_list(need("--isr-db")))
                o.isr_db.push_back(std::stod(p));
        } else if (a == "--target-cfo-hz") {
            if (!seen_tc) { o.target_cfo.clear(); seen_tc = true; }
            for (const auto& p : split_list(need("--target-cfo-hz")))
                o.target_cfo.push_back(std::stod(p));
        } else if (a == "--interferer-cfo-hz") {
            if (!seen_ic) { o.interferer_cfo.clear(); seen_ic = true; }
            for (const auto& p : split_list(need("--interferer-cfo-hz")))
                o.interferer_cfo.push_back(std::stod(p));
        } else if (a == "--seeds") {
            o.seeds = unsigned(std::stoul(need("--seeds")));
        } else if (a == "--grid") {
            o.grid = need("--grid");
            if (o.grid != "work" && o.grid != "native")
                throw std::runtime_error("--grid must be work or native");
        } else if (a == "--taps") {
            o.taps = need("--taps");
        } else if (a == "--out") {
            o.out = need("--out");
        } else if (a == "--stage") {
            o.stage = need("--stage");
            if (o.stage != "run" && o.stage != "dump")
                throw std::runtime_error("--stage must be run or dump");
        } else if (a == "--dump-dir") {
            o.dump_dir = need("--dump-dir");
        } else if (a == "--sc16-dir") {
            o.sc16_dir = need("--sc16-dir");
        } else if (a == "--overlap") {
            o.overlap = need("--overlap");
            if (o.overlap != "sync" && o.overlap != "payload")
                throw std::runtime_error("--overlap must be sync or payload");
        } else if (a == "--dual-target") {
            o.dual = true;
        } else if (a == "--weak-delay-samples") {
            o.weak_delay = std::stod(need("--weak-delay-samples"));
        } else if (a == "--weak-gain-db") {
            o.weak_gain_db = std::stod(need("--weak-gain-db"));
        } else if (a == "--coarse-stride") {
            o.coarse_stride = size_t(std::stoul(need("--coarse-stride")));
        } else if (a == "--search-margin") {
            o.search_margin = size_t(std::stoul(need("--search-margin")));
        } else if (a == "--list-cells") {
            o.list_cells = true;
        } else if (a == "--debug-cir") {
            o.debug_cir = true;
        } else if (a == "--self-check") {
            o.stage = "self-check";
        } else if (a == "--smoke") {
            o.sync_reps = { 64, 256 };
            o.psdu_bytes = { 32, 127 };
            o.target_codes = { 9, 10 };
            o.interferer_codes = { 10, 9 };
            o.multipath = { "single", "fractional" };
            o.isr_db = { 0, 20 };
            o.target_cfo = { 0 };
            o.interferer_cfo = { 0 };
            o.seeds = 2;
            seen_sync = seen_psdu = seen_tgt = seen_itf = true;
            seen_mp = seen_isr = seen_tc = seen_ic = true;
        } else if (a == "--help" || a == "-h") {
            usage();
            std::exit(0);
        } else {
            throw std::runtime_error("unknown option: " + a);
        }
    }
    if (o.seeds == 0)
        throw std::runtime_error("--seeds must be positive");
    if (o.grid == "native" && o.stage == "run" && o.sc16_dir.empty())
        throw std::runtime_error("--grid native --stage run requires --sc16-dir");
    if (o.grid == "native" && o.stage == "run" && o.taps.empty())
        throw std::runtime_error("--grid native --stage run requires --taps");
    if (o.stage == "dump" && o.dump_dir.empty())
        throw std::runtime_error("--stage dump requires --dump-dir");
    return o;
}

} // namespace

int main(int argc, char** argv)
{
    try {
        // Legacy positional form: <seeds> <work|native|dump> <dir> ...
        if (argc > 1 && argv[1][0] != '-')
            return run_legacy(argc, argv);

        const Options opt = parse_cli(argc, argv);
        if (opt.stage == "self-check")
            return self_check(opt);

        const std::vector<Cell> cells = enumerate(opt);
        if (opt.list_cells) {
            for (const auto& c : cells)
                std::cout << c.id() << '\n';
            std::cout << "cells=" << cells.size() << " seed_cases="
                      << cells.size() * opt.seeds << '\n';
            return 0;
        }
        if (cells.empty())
            throw std::runtime_error("empty cell enumeration");

        if (opt.stage == "dump") {
            for (const auto& c : cells)
                dump_cell(c, opt);
            return 0;
        }

        if (opt.debug_cir) {
            // Diagnostic: print the clean reference CIR of the first cell so a
            // reader can audit the fixed tap axis and the main-lobe region.
            const Cell& c = cells.front();
            const Truth truth = make_truth(c.fractional(), opt.dual, opt.weak_delay);
            Scene scene(c, 0, false, true, opt.dual, opt.weak_delay, opt.weak_gain_db,
                        interference_offset_for(opt));
            const Observation o =
                observe(scene.mix.clean, scene.target, scene.mix.q, opt.coarse_stride,
                        opt.search_margin);
            std::cout << "cell," << c.id() << "\nstatus," << o.radar_status
                      << "\npeak_tap," << o.peak << "\nexpected_peak_tap,"
                      << truth.expected_peak << "\nmainlobe_lo," << truth.ml_lo
                      << "\nmainlobe_hi," << truth.ml_hi << "\ntap,lag,abs,phase_rad\n";
            for (size_t i = 0; i < o.cir.size(); ++i)
                std::cout << i << ',' << (long(i) - long(kCirPre)) << ','
                          << std::abs(o.cir[i]) << ',' << std::arg(o.cir[i]) << '\n';
            return 0;
        }

        std::ofstream file;
        std::ostream* os = &std::cout;
        if (opt.out != "-") {
            file.open(opt.out);
            if (!file)
                throw std::runtime_error("cannot open output: " + opt.out);
            os = &file;
        }
        *os << kCsvHeader << '\n';
        for (const auto& c : cells) {
            std::cerr << "cell " << c.id() << " (" << opt.grid << ")\n";
            run_cell(c, opt, *os);
        }
        return 0;
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
