// Cross-library SIC fingerprint.  Compile twice, once against each worktree's
// header, and link only that worktree's libgnuradio-uwb.  Do not load both
// libraries in one process.
//
// Identity is the printed model fields plus the raw out / i_hat bytes.
// Timings are written separately and are not identity.  This file uses only
// the API present on both the frozen baseline and the performance branch:
// apply_model is called without the later out_preinitialized argument.
#include <gnuradio/uwb/uwb_preamble_sic_core.h>

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <random>
#include <string>
#include <vector>

namespace {

using cplx = std::complex<float>;
using gr::uwb::sic::SicConfig;
using gr::uwb::sic::SicMode;
using gr::uwb::sic::SicModel;
using gr::uwb::sic::SicResult;
using gr::uwb::sic::SicScratch;
using gr::uwb::sic::SicTemplates;

constexpr double kPi = 3.14159265358979323846;
constexpr size_t kSps = 1016;
constexpr size_t kPre = 2000;
constexpr double kFs = 998.4e6;
constexpr double kNoise = 1e-3;

double sinc_pi(double t)
{
    if (std::abs(t) < 1e-12)
        return 1.0;
    const double p = kPi * t;
    return std::sin(p) / p;
}

cplx gen_interp(const std::vector<cplx>& p, double x)
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

struct Model {
    std::vector<cplx> preamble;
    std::vector<cplx> sync;
};

Model build_model(size_t code, size_t reps)
{
    Model m;
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
    uint64_t s = 0x9E3779B97F4A7C15ull ^ (static_cast<uint64_t>(code) * 2654435761ull);
    std::vector<int> chips(cps);
    for (size_t i = 0; i < cps; ++i) {
        s ^= s << 13;
        s ^= s >> 7;
        s ^= s << 17;
        chips[i] = (s & 1u) ? 1 : -1;
    }
    const size_t total_chips = reps * cps;
    std::vector<double> up(total_chips * 2, 0.0);
    for (size_t k = 0; k < total_chips; ++k)
        up[2 * k] = static_cast<double>(chips[k % cps]);
    m.preamble.assign(up.size() + T - 1, cplx(0.f, 0.f));
    for (size_t i = 0; i < up.size(); ++i) {
        if (up[i] == 0.0)
            continue;
        for (size_t j = 0; j < T; ++j)
            m.preamble[i + j] += cplx(static_cast<float>(up[i] * pulse[j]), 0.f);
    }
    m.sync.assign(m.preamble.begin(), m.preamble.begin() + kSps);
    return m;
}

struct Path {
    double tau;
    cplx g;
};

void add_path(std::vector<cplx>& acc, const std::vector<cplx>& pre, int64_t base,
              double tau, cplx g, double cfo_hz)
{
    const int64_t span0 = base + static_cast<int64_t>(std::floor(tau)) - 32;
    const int64_t span1 = base + static_cast<int64_t>(std::ceil(tau)) +
                          static_cast<int64_t>(pre.size()) + 32;
    const int64_t lo = std::max<int64_t>(0, span0);
    const int64_t hi = std::min<int64_t>(static_cast<int64_t>(acc.size()), span1);
    const double w = 2.0 * kPi * cfo_hz / kFs;
    for (int64_t i = lo; i < hi; ++i) {
        const cplx v =
            gen_interp(pre, static_cast<double>(i) - static_cast<double>(base) - tau);
        const double ph = w * static_cast<double>(i);
        acc[static_cast<size_t>(i)] +=
            g * v * cplx(static_cast<float>(std::cos(ph)),
                         static_cast<float>(std::sin(ph)));
    }
}

std::vector<cplx> build_y(const Model& m, size_t n, int64_t comm_start,
                          const std::vector<Path>& paths, double cfo_hz,
                          uint64_t seed)
{
    std::vector<cplx> i(n, cplx(0.f, 0.f));
    for (const Path& p : paths)
        add_path(i, m.preamble, comm_start, p.tau, p.g, cfo_hz);
    std::mt19937_64 rng(seed);
    std::normal_distribution<double> nd(0.0, kNoise);
    std::vector<cplx> y(n);
    for (size_t k = 0; k < n; ++k) {
        const cplx w(static_cast<float>(nd(rng)), static_cast<float>(nd(rng)));
        y[k] = i[k] + w;
    }
    return y;
}

std::string hex_bytes(const void* p, size_t n)
{
    static const char* hexd = "0123456789abcdef";
    const auto* b = static_cast<const unsigned char*>(p);
    std::string s;
    s.resize(n * 2);
    for (size_t i = 0; i < n; ++i) {
        s[2 * i] = hexd[b[i] >> 4];
        s[2 * i + 1] = hexd[b[i] & 0xf];
    }
    return s;
}

std::string hex_double(double x)
{
    return hex_bytes(&x, sizeof(x));
}

std::string hex_float(float x)
{
    return hex_bytes(&x, sizeof(x));
}

struct Case {
    const char* name;
    size_t k;
    std::vector<Path> paths;
    double cfo_hz;
    uint64_t seed;
    int prefix_reps; // observed_end = origin + prefix_reps * sps; 0 => whole
    bool fractional;
    int64_t origin = 0; // 0 => kPre.  4194304 is the R01 rel-0 boundary.
    int reps = 0;       // 0 => 32
    float perturb = 0.f;
    uint64_t perturb_seed = 0;
};

bool dump_vec(const std::string& path, const std::vector<cplx>& v)
{
    std::ofstream f(path, std::ios::binary);
    f.write(reinterpret_cast<const char*>(v.data()),
            static_cast<std::streamsize>(v.size() * sizeof(cplx)));
    return static_cast<bool>(f);
}

void write_model(std::ostream& meta, const char* tag, const SicModel& m)
{
    meta << tag << " valid=" << (m.valid ? 1 : 0)
         << " state=" << static_cast<int>(m.state)
         << " status=" << (m.status ? m.status : "")
         << " reason=" << (m.reason ? m.reason : "")
         << " model=" << m.model_id << " template=" << m.template_hash
         << " config=" << m.config_hash << " origin=" << m.origin
         << " cfo=" << hex_double(m.cfo_hz) << " n_paths=" << m.n_paths
         << " cond=" << hex_double(m.gain_condition)
         << " fit=" << hex_double(m.fit_residual) << " train=" << m.training_lo
         << ":" << m.training_hi << " observed=" << m.observed_end_sample
         << " ready=" << m.estimation_ready_sample
         << " support=" << m.predicted_support_lo << ":" << m.predicted_support_hi
         << " source=" << (m.support_source ? m.support_source : "")
         << " sub=" << m.sub_lo << ":" << m.sub_hi
         << " confirmed=" << m.training_reps_confirmed
         << " nominal=" << m.nominal_sync_reps << " sps=" << m.samples_per_symbol
         << " fs=" << hex_double(m.sample_rate)
         << " phase_ref=" << m.phase_reference_sample;
    for (size_t k = 0; k < m.n_paths && k < 8; ++k) {
        const float gr = m.gain[k].real();
        const float gi = m.gain[k].imag();
        meta << " tau" << k << "=" << hex_double(m.tau_abs[k]) << " gr=" << hex_float(gr)
             << " gi=" << hex_float(gi);
    }
    meta << "\n";
}

bool run_case(const Case& c, const std::string& dir, std::ostream& meta,
              std::ostream& times)
{
    constexpr size_t code = 10;
    const int reps = c.reps > 0 ? c.reps : 32;
    const Model built = build_model(code, static_cast<size_t>(reps));
    Model m = built;
    if (c.perturb != 0.f) {
        std::mt19937 rng(static_cast<uint32_t>(c.perturb_seed));
        std::uniform_real_distribution<float> dist(-c.perturb, c.perturb);
        for (cplx& z : m.preamble)
            z = cplx(z.real() + dist(rng), z.imag() + dist(rng));
        m.sync.assign(m.preamble.begin(), m.preamble.begin() + kSps);
    }
    const int64_t comm_start =
        c.origin > 0 ? c.origin : static_cast<int64_t>(kPre);
    const size_t n =
        static_cast<size_t>(comm_start + static_cast<int64_t>(reps) * static_cast<int64_t>(kSps) +
                            3000);
    SicConfig cfg;
    cfg.mode = SicMode::Estimate;
    cfg.comm_code = code;
    cfg.sync_reps = static_cast<size_t>(reps);
    cfg.samples_per_symbol = kSps;
    cfg.sample_rate = kFs;
    cfg.k_paths = c.k;
    cfg.fit_reps = 8;
    cfg.validation_reps = 8;
    cfg.roi_lo = comm_start - 64;
    cfg.roi_hi = comm_start + static_cast<int64_t>(reps) * static_cast<int64_t>(kSps) + 64;
    cfg.coarse_stride = 14;
    cfg.post_tail = 100;
    cfg.enable_fractional = c.fractional;
    cfg.enable_cfo = true;
    cfg.detect_threshold = 0.5;
    cfg.validate_threshold = 0.25;
    if (c.prefix_reps > 0)
        cfg.observed_end_sample =
            comm_start + static_cast<int64_t>(c.prefix_reps) * static_cast<int64_t>(kSps);

    SicScratch scratch;
    std::string err;
    if (!gr::uwb::sic::prepare_sic_core(cfg, m.sync.data(), m.sync.size(),
                                        m.preamble.data(), m.preamble.size(),
                                        scratch, &err)) {
        std::cerr << c.name << " prepare failed: " << err << "\n";
        return false;
    }
    const std::vector<cplx> y =
        build_y(m, n, comm_start, c.paths, c.cfo_hz, c.seed);
    std::vector<cplx> out(n), ihat(n);
    SicResult r;
    r.out = out.data();
    r.i_hat = ihat.data();
    SicTemplates t;
    t.sync = m.sync.data();
    t.sync_len = m.sync.size();
    t.preamble = m.preamble.data();
    t.preamble_len = m.preamble.size();
    if (!gr::uwb::sic::sic_one(y.data(), y.size(), t, cfg, scratch, r, nullptr)) {
        std::cerr << c.name << " sic_one returned false\n";
        return false;
    }
    meta << c.name << " state=" << static_cast<int>(r.state)
         << " model=" << r.model_id << " n_paths=" << r.n_paths
         << " n_taus=" << r.n_taus << " start=" << r.detected_start
         << " support=" << r.support_lo << ":" << r.support_hi
         << " cfo=" << hex_double(r.cfo_hz) << " fit=" << hex_double(r.fit_residual)
         << " cond=" << hex_double(r.gain_condition);
    for (size_t k = 0; k < r.n_taus && k < 16; ++k)
        meta << " tau" << k << "=" << hex_double(r.taus[k]);
    meta << "\n";

    times << c.name << " core=" << r.timings.core_total_us
          << " estimate=" << r.timings.estimate_total_us
          << " cfo=" << r.timings.cfo_us << " cir=" << r.timings.cir_us
          << " refine=" << r.timings.fractional_refine_us
          << " matrix=" << r.timings.matrix_build_us
          << " gain=" << r.timings.gain_fit_us << " apply=" << r.timings.apply_us
          << " backfill=" << r.timings.backfill_us
          << " forward=" << r.timings.apply_forward_us << "\n";

    return dump_vec(dir + "/" + c.name + ".out", out) &&
           dump_vec(dir + "/" + c.name + ".ihat", ihat);
}

// Whole-window apply versus mixed chunks, then a second model and a return
// to the first.  Chunk bytes and the restored first model are identity.
bool run_chunk_switch(const std::string& dir, std::ostream& meta)
{
    constexpr size_t reps = 32;
    constexpr size_t code = 10;
    const Model m = build_model(code, reps);
    const int64_t comm_start = static_cast<int64_t>(kPre);
    const size_t n = kPre + reps * kSps + 3000;
    auto make_cfg = [&](size_t k) {
        SicConfig cfg;
        cfg.mode = SicMode::Estimate;
        cfg.comm_code = code;
        cfg.sync_reps = reps;
        cfg.samples_per_symbol = kSps;
        cfg.sample_rate = kFs;
        cfg.k_paths = k;
        cfg.fit_reps = 8;
        cfg.validation_reps = 8;
        cfg.roi_lo = comm_start - 64;
        cfg.roi_hi = comm_start + static_cast<int64_t>(reps * kSps) + 64;
        cfg.coarse_stride = 14;
        cfg.post_tail = 100;
        cfg.enable_fractional = true;
        cfg.enable_cfo = true;
        cfg.detect_threshold = 0.5;
        cfg.validate_threshold = 0.25;
        cfg.observed_end_sample =
            comm_start + 8 * static_cast<int64_t>(kSps);
        return cfg;
    };
    SicTemplates templ;
    templ.sync = m.sync.data();
    templ.sync_len = m.sync.size();
    templ.preamble = m.preamble.data();
    templ.preamble_len = m.preamble.size();

    const std::vector<Path> paths_a = { { 0.35, cplx(1.f, 0.f) },
                                        { 14.2, cplx(0.3f, -0.4f) } };
    const std::vector<Path> paths_b = { { 0.0, cplx(1.f, 0.f) } };
    const std::vector<cplx> ya =
        build_y(m, n, comm_start, paths_a, 25000.0, 9101);
    const std::vector<cplx> yb =
        build_y(m, n, comm_start, paths_b, 0.0, 9201);

    SicConfig cfg_a = make_cfg(2);
    SicScratch scratch;
    std::string err;
    if (!gr::uwb::sic::prepare_sic_core(cfg_a, m.sync.data(), m.sync.size(),
                                        m.preamble.data(), m.preamble.size(),
                                        scratch, &err)) {
        std::cerr << "chunk prepare failed: " << err << "\n";
        return false;
    }
    SicModel model_a;
    if (!gr::uwb::sic::estimate_model(ya.data(), ya.size(),
                                      cfg_a.observed_end_sample, templ, cfg_a,
                                      scratch, model_a, nullptr, nullptr)) {
        std::cerr << "chunk estimate A failed\n";
        return false;
    }
    std::vector<cplx> whole(n), whole_ih(n), chunk(n), chunk_ih(n);
    if (!gr::uwb::sic::apply_model(model_a, ya.data(), n, 0, templ, cfg_a, scratch,
                                   whole.data(), whole_ih.data(), nullptr)) {
        std::cerr << "chunk whole apply failed\n";
        return false;
    }
    const size_t cuts[] = { 1, 17, 1000, 4099 };
    size_t off = 0;
    size_t cut_i = 0;
    while (off < n) {
        size_t len = n - off;
        if (cut_i < 4 && cuts[cut_i] < len) {
            len = cuts[cut_i];
            ++cut_i;
        }
        if (!gr::uwb::sic::apply_model(model_a, ya.data() + off, len,
                                       static_cast<int64_t>(off), templ, cfg_a,
                                       scratch, chunk.data() + off,
                                       chunk_ih.data() + off, nullptr)) {
            std::cerr << "chunk apply failed at " << off << "\n";
            return false;
        }
        off += len;
    }
    if (std::memcmp(whole.data(), chunk.data(), n * sizeof(cplx)) != 0 ||
        std::memcmp(whole_ih.data(), chunk_ih.data(), n * sizeof(cplx)) != 0) {
        std::cerr << "chunk apply is not byte-identical to the whole window\n";
        return false;
    }

    SicConfig cfg_b = make_cfg(1);
    SicScratch scratch_b;
    if (!gr::uwb::sic::prepare_sic_core(cfg_b, m.sync.data(), m.sync.size(),
                                        m.preamble.data(), m.preamble.size(),
                                        scratch_b, &err)) {
        std::cerr << "switch prepare failed: " << err << "\n";
        return false;
    }
    SicModel model_b;
    if (!gr::uwb::sic::estimate_model(yb.data(), yb.size(),
                                      cfg_b.observed_end_sample, templ, cfg_b,
                                      scratch_b, model_b, nullptr, nullptr)) {
        std::cerr << "chunk estimate B failed\n";
        return false;
    }
    std::vector<cplx> switched(n), switched_ih(n);
    if (!gr::uwb::sic::apply_model(model_b, yb.data(), n, 0, templ, cfg_b,
                                   scratch_b, switched.data(), switched_ih.data(),
                                   nullptr)) {
        std::cerr << "switch apply B failed\n";
        return false;
    }
    std::vector<cplx> again(n), again_ih(n);
    if (!gr::uwb::sic::apply_model(model_a, ya.data(), n, 0, templ, cfg_a, scratch,
                                   again.data(), again_ih.data(), nullptr)) {
        std::cerr << "switch reapply A failed\n";
        return false;
    }
    if (std::memcmp(whole.data(), again.data(), n * sizeof(cplx)) != 0 ||
        std::memcmp(whole_ih.data(), again_ih.data(), n * sizeof(cplx)) != 0) {
        std::cerr << "model switch did not restore model A\n";
        return false;
    }
    write_model(meta, "chunk_model_a", model_a);
    write_model(meta, "chunk_model_b", model_b);
    return dump_vec(dir + "/chunk_apply.out", chunk) &&
           dump_vec(dir + "/chunk_apply.ihat", chunk_ih) &&
           dump_vec(dir + "/model_switch.out", again) &&
           dump_vec(dir + "/model_switch.ihat", again_ih);
}

} // namespace

int main(int argc, char** argv)
{
    if (argc != 2) {
        std::cerr << "usage: sic_fp OUTDIR\n";
        return 2;
    }
    const std::string dir = argv[1];
    std::ofstream meta(dir + "/meta.txt");
    std::ofstream times(dir + "/timings.txt");
    if (!meta || !times) {
        std::cerr << "cannot write " << dir << "\n";
        return 2;
    }
    meta << "identity_excludes timings.txt\n";
    meta << "generator code=10 noise=1e-3 fs=998.4e6 sps=1016\n";
    const std::vector<Case> cases = {
        { "frac_cfo25k", 4,
          { { 0.35, cplx(1.f, 0.f) }, { 14.2, cplx(0.3f, -0.4f) } },
          25000.0, 9101, 8, true },
        { "k1_int", 1, { { 0.0, cplx(1.f, 0.f) } }, 0.0, 9201, 8, true },
        { "k4_multi", 4,
          { { 0.0, cplx(1.f, 0.f) },
            { 7.0, cplx(0.f, 0.5f) },
            { 23.0, cplx(0.276f, -0.116f) },
            { 51.0, cplx(0.2f, 0.f) } },
          0.0, 9204, 8, true },
        { "cfo15k", 2,
          { { 0.2, cplx(1.f, 0.f) }, { 11.4, cplx(0.25f, 0.1f) } },
          15000.0, 9401, 8, true },
        { "short6", 2, { { 0.0, cplx(1.f, 0.f) } }, 0.0, 9302, 6, true },
        { "frac_off", 2,
          { { 0.0, cplx(1.f, 0.f) }, { 18.0, cplx(0.2f, 0.2f) } },
          12000.0, 9301, 8, false },
        // 2^22: rel=0 first mixes self-energy hits and misses.  reps=8 keeps
        // the preamble inside the visible prefix.  Cache on and off must match
        // each other and the frozen baseline.
        { "r01_boundary", 1, { { 0.0, cplx(1.f, 0.f) } }, 0.0, 9501, 8, true,
          4194304, 8, 0.f, 0 },
        // Small complex perturbation of a real code-10 preamble.  Not Ipatov.
        { "random_cplx", 1, { { 0.0, cplx(1.f, 0.f) } }, 0.0, 9502, 8, true, 0,
          0, 0.05f, 42 },
    };
    for (const Case& c : cases) {
        if (!run_case(c, dir, meta, times))
            return 1;
    }
    if (!run_chunk_switch(dir, meta))
        return 1;
    return 0;
}
