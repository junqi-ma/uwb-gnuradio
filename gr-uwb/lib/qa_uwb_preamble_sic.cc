/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * QA for the S0 preamble-only SIC PDU adapter
 * (gr-uwb/include/gnuradio/uwb/uwb_preamble_sic.h,
 *  docs/sic/S0_sic_adapter_api.md).
 *
 * Covers every item in the frozen adapter QA list:
 *   - bypass bit-exactness (SIC off / no detection / wrong template);
 *   - one `out` per offered `in`, in order; offered == admitted + missed;
 *   - admission capacity (byte budget deterministic; in_flight/queue burst)
 *     with no drop and no reorder, bypassed output bit-identical;
 *   - invalid input (bad PMT / wrong vector type / empty / sample_count
 *     mismatch) counted, never crashing;
 *   - stop/drain: completed == admitted, drained(), no residual thread;
 *   - oracle sets sic_uses_truth=true; estimate never does;
 *   - all adapter + core timers present; core children are sub-timers of
 *     core_total (never double counted).
 *
 * The comm signal is generated synthetically (a shaped SYNC code convolved
 * with a Gaussian pulse) so the QA needs no testdata.  One OPTIONAL case loads
 * testdata/sic_preamble/golden/int_single/io.npz when it exists and SKIPS
 * otherwise (UWB_SIC_GOLDEN_DIR overrides the golden root).
 */

#include <boost/test/unit_test.hpp>

#include <gnuradio/blocks/message_debug.h>
#include <gnuradio/gr_complex.h>
#include <gnuradio/top_block.h>
#include <gnuradio/uwb/uwb_preamble_sic.h>
#include <pmt/pmt.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <random>
#include <string>
#include <sys/stat.h>
#include <thread>
#include <vector>

namespace {

using cplx = std::complex<float>;
using Blk = gr::uwb::UwbPreambleSic;

constexpr double kPi = 3.14159265358979323846;
constexpr size_t kSps = 1016;
constexpr double kFs = 998.4e6;

// ---------------------------------------------------------------------------
// Synthetic comm model (same convention as qa_uwb_preamble_sic_core.cc).
// ---------------------------------------------------------------------------
double sinc_pi(double t)
{
    if (std::abs(t) < 1e-12)
        return 1.0;
    const double p = kPi * t;
    return std::sin(p) / p;
}

// Generator-side fractional placement model (Blackman-windowed sinc, H=24);
// deliberately different from the core's own Hamming interpolator.
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
    const size_t cps = kSps / 2; // 2 samples/chip
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
    const size_t total_chips = reps * cps;
    std::vector<double> up(total_chips * 2, 0.0);
    for (size_t k = 0; k < total_chips; ++k)
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

struct Path {
    double tau;
    cplx g;
};

void add_path(std::vector<cplx>& acc,
              const std::vector<cplx>& pre,
              int64_t base,
              double tau,
              cplx g,
              double cfo_hz)
{
    const int64_t span0 =
        base + static_cast<int64_t>(std::floor(tau)) - 32;
    const int64_t span1 = base + static_cast<int64_t>(std::ceil(tau)) +
                          static_cast<int64_t>(pre.size()) + 32;
    const int64_t lo = std::max<int64_t>(0, span0);
    const int64_t hi =
        std::min<int64_t>(static_cast<int64_t>(acc.size()), span1);
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

// Build y = i + w (sensing s is irrelevant to the adapter contract).
std::vector<cplx> make_signal(const Model& m,
                              size_t n,
                              int64_t start,
                              const std::vector<Path>& paths,
                              double cfo_hz,
                              double noise_std,
                              uint64_t seed)
{
    std::vector<cplx> i(n, cplx(0.f, 0.f));
    for (const Path& p : paths)
        add_path(i, m.preamble, start, p.tau, p.g, cfo_hz);
    std::vector<cplx> y(n, cplx(0.f, 0.f));
    std::mt19937_64 rng(seed);
    std::normal_distribution<double> nd(0.0, 1.0);
    for (size_t k = 0; k < n; ++k) {
        cplx w(0.f, 0.f);
        if (noise_std > 0.0)
            w = cplx(static_cast<float>(nd(rng) * noise_std),
                     static_cast<float>(nd(rng) * noise_std));
        y[k] = i[k] + w;
    }
    return y;
}

// ---------------------------------------------------------------------------
// PMT / harness helpers
// ---------------------------------------------------------------------------
std::string sym(pmt::pmt_t v)
{
    return pmt::is_symbol(v) ? pmt::symbol_to_string(v) : std::string();
}

pmt::pmt_t make_pdu(const std::vector<cplx>& y,
                    uint64_t wid,
                    int64_t sample_count = -1)
{
    pmt::pmt_t meta = pmt::make_dict();
    // SENSING description that must pass through untouched.
    meta = pmt::dict_add(meta, pmt::mp("window_id"), pmt::from_uint64(wid));
    meta = pmt::dict_add(meta, pmt::mp("code_index"), pmt::from_long(9));
    meta = pmt::dict_add(meta, pmt::mp("sync_repetitions"), pmt::from_long(64));
    meta = pmt::dict_add(meta, pmt::mp("sfd_mode"), pmt::mp("4z2"));
    meta = pmt::dict_add(
        meta, pmt::mp("sample_count"),
        pmt::from_long(sample_count >= 0 ? sample_count
                                         : static_cast<int64_t>(y.size())));
    return pmt::cons(meta, pmt::init_c32vector(y.size(), y.data()));
}

std::vector<cplx> pdu_iq(pmt::pmt_t pdu)
{
    size_t n = 0;
    const gr_complex* p = pmt::c32vector_elements(pmt::cdr(pdu), n);
    return std::vector<cplx>(p, p + n);
}

pmt::pmt_t pdu_meta(pmt::pmt_t pdu) { return pmt::car(pdu); }

bool all_zero(const std::vector<cplx>& v)
{
    for (const cplx& z : v)
        if (z != cplx(0.f, 0.f))
            return false;
    return true;
}

bool bit_equal(const std::vector<cplx>& a, const std::vector<cplx>& b)
{
    if (a.size() != b.size())
        return false;
    return std::memcmp(a.data(), b.data(), a.size() * sizeof(cplx)) == 0;
}

struct Harness {
    Blk::sptr blk;
    gr::top_block_sptr tb;
    gr::blocks::message_debug::sptr out_dbg;
    gr::blocks::message_debug::sptr status_dbg;
    gr::blocks::message_debug::sptr ihat_dbg;

    explicit Harness(Blk::sptr b, bool connect_ihat = false) : blk(std::move(b))
    {
        out_dbg = gr::blocks::message_debug::make();
        status_dbg = gr::blocks::message_debug::make();
        tb = gr::make_top_block("qa_sic_adapter");
        tb->msg_connect(blk, "out", out_dbg, "store");
        tb->msg_connect(blk, "status", status_dbg, "store");
        if (connect_ihat) {
            ihat_dbg = gr::blocks::message_debug::make();
            tb->msg_connect(blk, "i_hat", ihat_dbg, "store");
        }
        tb->start();
    }

    void stop()
    {
        tb->stop();
        tb->wait();
    }
    ~Harness() { stop(); }
};

template <typename Pred>
bool wait_for(Pred pred, size_t timeout_ms = 120000)
{
    const auto t0 = std::chrono::steady_clock::now();
    while (!pred()) {
        if (std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - t0)
                .count() > static_cast<long long>(timeout_ms))
            return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    return true;
}

bool wait_drained(const Blk::sptr& b, uint64_t want_offered)
{
    return wait_for([&] {
        return b->windows_offered() >= want_offered && b->drained();
    });
}

bool status_seen(gr::blocks::message_debug::sptr dbg, const std::string& event)
{
    for (size_t i = 0; i < dbg->num_messages(); ++i) {
        pmt::pmt_t m = dbg->get_message(i);
        if (pmt::is_dict(m) &&
            sym(pmt::dict_ref(m, pmt::mp("event"), pmt::PMT_NIL)) == event)
            return true;
    }
    return false;
}

// First status dict with event=="window" and the given window_id (PMT_NIL if
// none).  Used to read the pub-inclusive adapter timers mirrored on status.
pmt::pmt_t find_window_status(gr::blocks::message_debug::sptr dbg,
                              uint64_t window_id)
{
    for (size_t i = 0; i < dbg->num_messages(); ++i) {
        pmt::pmt_t m = dbg->get_message(i);
        if (!pmt::is_dict(m))
            continue;
        if (sym(pmt::dict_ref(m, pmt::mp("event"), pmt::PMT_NIL)) != "window")
            continue;
        pmt::pmt_t wv = pmt::dict_ref(m, pmt::mp("window_id"),
                                      pmt::from_uint64(0));
        if (pmt::to_uint64(wv) == window_id)
            return m;
    }
    return pmt::PMT_NIL;
}

const char* const kOutMetaKeys[] = {
    "sic_state",          "sic_detected",          "sic_committed",
    "sic_partial",        "sic_ambiguous",         "sic_bypass_reason",
    "sic_uses_truth",     "sic_detected_start",    "sic_cfo_hz",
    "sic_n_paths",        "sic_support_lo",        "sic_support_hi",
    "sic_residual_corr",  "handler_us",            "queue_wait_us",
    "core_total_us",      "output_wrap_publish_us", "completion_latency_us",
    "sic_detect_coarse_us", "sic_detect_fine_us",  "sic_cfo_us",
    "sic_cir_us",         "sic_path_select_us",    "sic_fractional_refine_us",
    "sic_matrix_build_us", "sic_gain_fit_us",      "sic_reconstruct_us",
    "sic_validate_commit_us"
};

void check_out_meta_keys(pmt::pmt_t meta)
{
    for (const char* k : kOutMetaKeys)
        BOOST_CHECK_MESSAGE(pmt::dict_has_key(meta, pmt::mp(k)),
                            "out meta missing key: " << k);
}

uint64_t meta_u64(pmt::pmt_t meta, const char* k)
{
    pmt::pmt_t v = pmt::dict_ref(meta, pmt::mp(k), pmt::from_uint64(0));
    if (pmt::is_uint64(v))
        return pmt::to_uint64(v);
    if (pmt::is_integer(v))
        return static_cast<uint64_t>(pmt::to_long(v));
    return 0;
}

size_t meta_children_sum(pmt::pmt_t meta)
{
    static const char* const kids[] = {
        "sic_detect_coarse_us", "sic_detect_fine_us", "sic_cfo_us",
        "sic_cir_us", "sic_path_select_us", "sic_fractional_refine_us",
        "sic_matrix_build_us", "sic_gain_fit_us", "sic_reconstruct_us",
        "sic_validate_commit_us"
    };
    size_t s = 0;
    for (const char* k : kids)
        s += meta_u64(meta, k);
    return s;
}

// ---------------------------------------------------------------------------
// Optional golden fixture reader (npz, STORED only)
// ---------------------------------------------------------------------------
bool read_file(const std::string& path, std::vector<uint8_t>& bytes)
{
    std::ifstream f(path, std::ios::binary);
    if (!f)
        return false;
    f.seekg(0, std::ios::end);
    const std::streamoff sz = f.tellg();
    if (sz <= 0)
        return false;
    f.seekg(0, std::ios::beg);
    bytes.resize(static_cast<size_t>(sz));
    f.read(reinterpret_cast<char*>(bytes.data()), sz);
    return static_cast<bool>(f);
}

uint16_t le16(const uint8_t* p)
{
    return static_cast<uint16_t>(p[0] | (p[1] << 8));
}
uint32_t le32(const uint8_t* p)
{
    return static_cast<uint32_t>(p[0] | (p[1] << 8) | (p[2] << 16) | (p[3] << 24));
}

bool npz_get(const std::vector<uint8_t>& z,
             const std::string& name,
             std::vector<uint8_t>& out)
{
    if (z.size() < 22)
        return false;
    size_t eocd = std::string::npos;
    for (size_t i = z.size() - 22 + 1; i-- > 0;) {
        if (le32(&z[i]) == 0x06054b50u) {
            eocd = i;
            break;
        }
        if (z.size() - i > 66000)
            break;
    }
    if (eocd == std::string::npos)
        return false;
    const uint16_t nent = le16(&z[eocd + 10]);
    size_t cd = le32(&z[eocd + 16]);
    for (uint16_t e = 0; e < nent; ++e) {
        if (cd + 46 > z.size() || le32(&z[cd]) != 0x02014b50u)
            return false;
        const uint16_t method = le16(&z[cd + 10]);
        const uint32_t csize = le32(&z[cd + 20]);
        const uint16_t fnlen = le16(&z[cd + 28]);
        const uint16_t extlen = le16(&z[cd + 30]);
        const uint16_t cmlen = le16(&z[cd + 32]);
        const uint32_t lho = le32(&z[cd + 42]);
        const std::string fname(reinterpret_cast<const char*>(&z[cd + 46]), fnlen);
        if (fname == name) {
            if (method != 0)
                return false;
            if (lho + 30 > z.size() || le32(&z[lho]) != 0x04034b50u)
                return false;
            const uint16_t lfn = le16(&z[lho + 26]);
            const uint16_t lex = le16(&z[lho + 28]);
            const size_t d0 = lho + 30 + lfn + lex;
            if (d0 + csize > z.size())
                return false;
            out.assign(z.begin() + d0, z.begin() + d0 + csize);
            return true;
        }
        cd += 46u + fnlen + extlen + cmlen;
    }
    return false;
}

bool npy_complex(const std::vector<uint8_t>& np, std::vector<cplx>& out)
{
    if (np.size() < 10 || np[0] != 0x93 || np[1] != 'N' || np[2] != 'U' ||
        np[3] != 'M' || np[4] != 'P' || np[5] != 'Y')
        return false;
    const uint8_t major = np[6];
    size_t hlen = 0, hoff = 0;
    if (major <= 1) {
        hlen = le16(&np[8]);
        hoff = 10;
    } else {
        hlen = le32(&np[8]);
        hoff = 12;
    }
    if (hoff + hlen > np.size())
        return false;
    const std::string h(reinterpret_cast<const char*>(&np[hoff]), hlen);
    if (h.find("'fortran_order': True") != std::string::npos)
        return false;
    const size_t sp = h.find("'shape': (");
    if (sp == std::string::npos)
        return false;
    const size_t ne = h.find(')', sp);
    if (ne == std::string::npos)
        return false;
    size_t count = 1;
    {
        std::string sh = h.substr(sp + 10, ne - sp - 10);
        std::string cur;
        for (char c : sh + ",") {
            if (c == ',' || c == ' ') {
                if (!cur.empty()) {
                    count *= static_cast<size_t>(std::stoul(cur));
                    cur.clear();
                }
            } else {
                cur.push_back(c);
            }
        }
    }
    const size_t d0 = hoff + hlen;
    if (h.find("'<c8'") != std::string::npos) {
        if (d0 + count * 8 > np.size())
            return false;
        out.resize(count);
        std::memcpy(out.data(), &np[d0], count * 8);
        return true;
    }
    if (h.find("'<c16'") != std::string::npos) {
        if (d0 + count * 16 > np.size())
            return false;
        out.resize(count);
        const double* d = reinterpret_cast<const double*>(&np[d0]);
        for (size_t i = 0; i < count; ++i)
            out[i] = cplx(static_cast<float>(d[2 * i]),
                          static_cast<float>(d[2 * i + 1]));
        return true;
    }
    return false;
}

bool golden_npz(const std::string& name, std::vector<uint8_t>& bytes)
{
    std::string root;
    const char* e = std::getenv("UWB_SIC_GOLDEN_DIR");
#ifdef UWB_TESTDATA_DIR
    root = std::string(UWB_TESTDATA_DIR) + "/sic_preamble/golden";
#endif
    if (e && *e)
        root = e;
    if (root.empty())
        return false;
    struct stat st;
    if (::stat(root.c_str(), &st) != 0 || !S_ISDIR(st.st_mode))
        return false;
    return read_file(root + "/" + name, bytes);
}

} // namespace

// ---------------------------------------------------------------------------
// 1. Bypass bit-exactness: SIC off.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_adapter_off_bypass_bit_exact)
{
    const Model m = build_model(10, 16);
    const size_t n = 20480;
    const int64_t start = 2500;
    const std::vector<cplx> y =
        make_signal(m, n, start, { { 0.0, cplx(1.f, 0.f) } }, 0.0, 1e-3, 1);

    auto blk = Blk::make_from_templates(m.sync, m.preamble, 10, 16, kSps, kFs,
                                        4, 8, 8, 0, 0, 14, "off", 0.0, 0.0,
                                        8, size_t(1) << 30, 8, true);
    Harness h(blk, true);
    blk->_post(pmt::mp("in"), make_pdu(y, 1));
    BOOST_REQUIRE(wait_drained(blk, 1));
    h.stop();

    BOOST_REQUIRE_EQUAL(h.out_dbg->num_messages(), 1u);
    BOOST_REQUIRE_EQUAL(h.ihat_dbg->num_messages(), 1u);
    const pmt::pmt_t out = h.out_dbg->get_message(0);
    const pmt::pmt_t meta = pdu_meta(out);
    BOOST_CHECK(bit_equal(pdu_iq(out), y));
    BOOST_CHECK(all_zero(pdu_iq(h.ihat_dbg->get_message(0))));
    BOOST_CHECK_EQUAL(sym(pmt::dict_ref(meta, pmt::mp("sic_state"), pmt::PMT_NIL)),
                      "bypassed");
    BOOST_CHECK_EQUAL(
        sym(pmt::dict_ref(meta, pmt::mp("sic_bypass_reason"), pmt::PMT_NIL)),
        "mode_off");
    BOOST_CHECK(pmt::to_bool(
        pmt::dict_ref(meta, pmt::mp("sic_uses_truth"), pmt::PMT_T)) == false);
    // Sensing description copied through untouched.
    BOOST_CHECK_EQUAL(pmt::to_long(pmt::dict_ref(
                          meta, pmt::mp("code_index"), pmt::from_long(-1))),
                      9);
    BOOST_CHECK_EQUAL(pmt::to_long(pmt::dict_ref(
                          meta, pmt::mp("sync_repetitions"), pmt::from_long(-1))),
                      64);
    BOOST_CHECK_EQUAL(
        sym(pmt::dict_ref(meta, pmt::mp("sfd_mode"), pmt::PMT_NIL)), "4z2");
    check_out_meta_keys(meta);
}

// ---------------------------------------------------------------------------
// 2. Bypass bit-exactness: no detection (all-zero window, strict threshold).
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_adapter_no_detection_bypass)
{
    const Model m = build_model(10, 16);
    const std::vector<cplx> y(2048, cplx(0.f, 0.f));

    auto blk = Blk::make_from_templates(m.sync, m.preamble, 10, 16, kSps, kFs,
                                        4, 8, 8, 0, 0, 14, "estimate", 0.99,
                                        0.25, 8, size_t(1) << 30, 8, true);
    Harness h(blk, true);
    blk->_post(pmt::mp("in"), make_pdu(y, 5));
    BOOST_REQUIRE(wait_drained(blk, 1));
    h.stop();

    BOOST_REQUIRE_EQUAL(h.out_dbg->num_messages(), 1u);
    const pmt::pmt_t meta = pdu_meta(h.out_dbg->get_message(0));
    BOOST_CHECK(bit_equal(pdu_iq(h.out_dbg->get_message(0)), y));
    BOOST_CHECK(all_zero(pdu_iq(h.ihat_dbg->get_message(0))));
    BOOST_CHECK_EQUAL(sym(pmt::dict_ref(meta, pmt::mp("sic_state"), pmt::PMT_NIL)),
                      "bypassed");
    BOOST_CHECK(pmt::to_bool(
        pmt::dict_ref(meta, pmt::mp("sic_detected"), pmt::PMT_T)) == false);
}

// ---------------------------------------------------------------------------
// 3. Bypass bit-exactness: wrong comm template (code 11 vs a code-10 signal).
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_adapter_wrong_template_bypass)
{
    const Model m10 = build_model(10, 16);
    const Model m11 = build_model(11, 16);
    const size_t n = 20480;
    const int64_t start = 2500;
    const std::vector<cplx> y =
        make_signal(m10, n, start, { { 0.0, cplx(1.f, 0.f) } }, 0.0, 1e-3, 3);

    auto blk = Blk::make_from_templates(m11.sync, m11.preamble, 11, 16, kSps,
                                        kFs, 4, 8, 8, 0, 0, 14, "estimate", 0.5,
                                        0.25, 8, size_t(1) << 30, 8, false);
    Harness h(blk);
    blk->_post(pmt::mp("in"), make_pdu(y, 9));
    BOOST_REQUIRE(wait_drained(blk, 1));
    h.stop();

    BOOST_REQUIRE_EQUAL(h.out_dbg->num_messages(), 1u);
    const pmt::pmt_t meta = pdu_meta(h.out_dbg->get_message(0));
    BOOST_CHECK(bit_equal(pdu_iq(h.out_dbg->get_message(0)), y));
    BOOST_CHECK(pmt::to_bool(
        pmt::dict_ref(meta, pmt::mp("sic_committed"), pmt::PMT_T)) == false);
}

// ---------------------------------------------------------------------------
// 4. One `out` per offered `in`, in order; offered == admitted + missed.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_adapter_one_out_per_offered_in_order)
{
    const Model m = build_model(10, 16);
    const size_t n = 4096;
    auto blk = Blk::make_from_templates(m.sync, m.preamble, 10, 16, kSps, kFs,
                                        4, 8, 8, 0, 0, 14, "off", 0.0, 0.0,
                                        8, size_t(1) << 30, 8, false);
    Harness h(blk);

    const uint64_t N = 8;
    std::vector<std::vector<cplx>> inputs;
    for (uint64_t k = 0; k < N; ++k) {
        std::vector<cplx> y(n, cplx(0.f, 0.f));
        y[0] = cplx(static_cast<float>(k + 1), 0.f);
        y[1] = cplx(0.f, static_cast<float>(-(int64_t)k - 1));
        inputs.push_back(y);
        blk->_post(pmt::mp("in"), make_pdu(y, k));
    }
    BOOST_REQUIRE(wait_drained(blk, N));
    h.stop();

    BOOST_REQUIRE_EQUAL(h.out_dbg->num_messages(), N);
    for (uint64_t k = 0; k < N; ++k) {
        const pmt::pmt_t out = h.out_dbg->get_message(k);
        BOOST_CHECK_EQUAL(
            meta_u64(pdu_meta(out), "window_id"), k); // strict order
        BOOST_CHECK(bit_equal(pdu_iq(out), inputs[k])); // marker proves order
    }
    BOOST_CHECK_EQUAL(blk->windows_offered(), N);
    BOOST_CHECK_EQUAL(blk->windows_admitted(), N);
    BOOST_CHECK_EQUAL(blk->windows_missed_admission(), 0u);
    BOOST_CHECK_EQUAL(blk->windows_completed(), N);
    BOOST_CHECK_EQUAL(blk->windows_failed(), 0u);
    BOOST_CHECK_EQUAL(blk->windows_dropped(), 0u);
    BOOST_CHECK_EQUAL(blk->windows_offered(),
                      blk->windows_admitted() + blk->windows_missed_admission());
    std::printf("[sic-adapter-qa] order: offered=%llu admitted=%llu completed=%llu "
                "bypassed=%llu missed=%llu\n",
                static_cast<unsigned long long>(blk->windows_offered()),
                static_cast<unsigned long long>(blk->windows_admitted()),
                static_cast<unsigned long long>(blk->windows_completed()),
                static_cast<unsigned long long>(blk->windows_bypassed()),
                static_cast<unsigned long long>(
                    blk->windows_missed_admission()));
}

// ---------------------------------------------------------------------------
// 5. Capacity: byte budget forces missed_admission; bypass is ordered and
//    bit-identical; nothing is dropped.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_adapter_byte_budget_missed_admission)
{
    const Model m = build_model(10, 16);
    const size_t n = 20480;
    const int64_t start = 2500;
    const std::vector<cplx> y =
        make_signal(m, n, start, { { 0.0, cplx(1.f, 0.f) } }, 0.0, 1e-3, 5);

    // Budget smaller than one window: every window is rejected by admission.
    auto blk = Blk::make_from_templates(m.sync, m.preamble, 10, 16, kSps, kFs,
                                        4, 8, 8, 0, 0, 14, "estimate", 0.5,
                                        0.25, 8, 4096, 8, true);
    Harness h(blk, true);

    const uint64_t N = 6;
    for (uint64_t k = 0; k < N; ++k)
        blk->_post(pmt::mp("in"), make_pdu(y, 100 + k));
    BOOST_REQUIRE(wait_drained(blk, N));
    h.stop();

    BOOST_REQUIRE_EQUAL(h.out_dbg->num_messages(), N);
    for (uint64_t k = 0; k < N; ++k) {
        const pmt::pmt_t out = h.out_dbg->get_message(k);
        const pmt::pmt_t meta = pdu_meta(out);
        BOOST_CHECK_EQUAL(meta_u64(meta, "window_id"), 100 + k); // no reorder
        BOOST_CHECK(bit_equal(pdu_iq(out), y));
        BOOST_CHECK_EQUAL(sym(pmt::dict_ref(meta, pmt::mp("sic_state"),
                                            pmt::PMT_NIL)),
                          "bypassed");
        BOOST_CHECK_EQUAL(
            sym(pmt::dict_ref(meta, pmt::mp("sic_bypass_reason"), pmt::PMT_NIL)),
            "iq_budget");
        BOOST_CHECK(pmt::to_bool(pmt::dict_ref(
            meta, pmt::mp("sic_missed_admission"), pmt::PMT_F)));
    }
    BOOST_CHECK_EQUAL(blk->windows_offered(), N);
    BOOST_CHECK_EQUAL(blk->windows_admitted(), 0u);
    BOOST_CHECK_EQUAL(blk->windows_missed_admission(), N);
    BOOST_CHECK_EQUAL(blk->windows_completed(), 0u);
    BOOST_CHECK_EQUAL(blk->windows_dropped(), 0u);
    BOOST_CHECK_EQUAL(blk->windows_offered(),
                      blk->windows_admitted() + blk->windows_missed_admission());
    std::printf("[sic-adapter-qa] iq_budget: offered=%llu admitted=%llu missed=%llu "
                "(all bypassed bit-exact)\n",
                static_cast<unsigned long long>(blk->windows_offered()),
                static_cast<unsigned long long>(blk->windows_admitted()),
                static_cast<unsigned long long>(
                    blk->windows_missed_admission()));
}

// ---------------------------------------------------------------------------
// 6. Capacity: in_flight / queue limits under a burst.  A blocked upstream
//    source is reported as a miss, never hidden by slowing the input rate; the
//    bypassed windows stay ordered and bit-identical.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_adapter_inflight_burst_missed_no_reorder)
{
    const Model m = build_model(10, 16);
    const size_t n = 21256;
    const int64_t start = 2500;
    const std::vector<cplx> y =
        make_signal(m, n, start, { { 0.0, cplx(1.f, 0.f) } }, 0.0, 1e-3, 7);

    // in_flight_max = queue_capacity = 2, budget generous.  The bypass FIFO is
    // explicitly large here so ordering/bit-exactness of the ordered bypass can
    // be checked in isolation (the bounded-FIFO case is a separate test).
    auto blk = Blk::make_from_templates(m.sync, m.preamble, 10, 16, kSps, kFs,
                                        4, 8, 8, 0, 0, 14, "estimate", 0.5,
                                        0.25, 2, size_t(1) << 30, 2, false,
                                        true, true, 0, 100, 0, 4096);
    Harness h(blk);

    const uint64_t N = 128;
    pmt::pmt_t vec = pmt::init_c32vector(y.size(), y.data());
    for (uint64_t k = 0; k < N; ++k) {
        pmt::pmt_t meta = pmt::make_dict();
        meta = pmt::dict_add(meta, pmt::mp("window_id"),
                             pmt::from_uint64(k));
        meta = pmt::dict_add(meta, pmt::mp("code_index"),
                             pmt::from_long(9));
        meta = pmt::dict_add(meta, pmt::mp("sample_count"),
                             pmt::from_long(static_cast<long>(y.size())));
        blk->_post(pmt::mp("in"), pmt::cons(meta, vec));
    }
    BOOST_REQUIRE(wait_drained(blk, N));
    h.stop();

    BOOST_REQUIRE_EQUAL(h.out_dbg->num_messages(), N);
    uint64_t prev = 0;
    bool first = true;
    uint64_t missed = 0;
    for (uint64_t k = 0; k < N; ++k) {
        const pmt::pmt_t out = h.out_dbg->get_message(k);
        const pmt::pmt_t meta = pdu_meta(out);
        const uint64_t wid = meta_u64(meta, "window_id");
        if (!first)
            BOOST_CHECK(wid > prev); // strictly increasing, no reorder
        prev = wid;
        first = false;
        const bool miss = pmt::to_bool(
            pmt::dict_ref(meta, pmt::mp("sic_missed_admission"), pmt::PMT_F));
        if (miss) {
            ++missed;
            BOOST_CHECK(bit_equal(pdu_iq(out), y));
            const std::string reason = sym(
                pmt::dict_ref(meta, pmt::mp("sic_bypass_reason"), pmt::PMT_NIL));
            BOOST_CHECK(reason == "in_flight_limit" || reason == "queue_full" ||
                        reason == "iq_budget");
        }
    }
    BOOST_CHECK_MESSAGE(missed >= 1,
                        "burst did not exceed in_flight/queue: missed=" << missed);
    BOOST_CHECK_EQUAL(blk->windows_offered(), N);
    BOOST_CHECK_EQUAL(blk->windows_offered(),
                      blk->windows_admitted() + blk->windows_missed_admission());
    BOOST_CHECK_EQUAL(blk->windows_missed_admission(), missed);
    BOOST_CHECK_EQUAL(blk->windows_completed(), blk->windows_admitted());
    BOOST_CHECK_EQUAL(blk->windows_dropped(), 0u);
    std::printf("[sic-adapter-qa] burst: offered=%llu admitted=%llu missed=%llu "
                "inflight_hwm=%zu queue_hwm=%zu bypass_fifo_hwm=%zu\n",
                static_cast<unsigned long long>(blk->windows_offered()),
                static_cast<unsigned long long>(blk->windows_admitted()),
                static_cast<unsigned long long>(
                    blk->windows_missed_admission()),
                blk->inflight_high_watermark(), blk->queue_high_watermark(),
                blk->bypass_fifo_high_watermark());
}

// ---------------------------------------------------------------------------
// 6b. Bounded bypass FIFO.  When the pending-bypass FIFO is full, the window is
//     terminated as queue_overflow (recorded, never a silent drop); emitted
//     bypasses stay ordered and bit-identical.  Upstream credit keeps this off
//     the normal path.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_adapter_bounded_bypass_fifo_queue_overflow)
{
    const Model m = build_model(10, 16);
    const size_t n = 21256;
    const int64_t start = 2500;
    const std::vector<cplx> y =
        make_signal(m, n, start, { { 0.0, cplx(1.f, 0.f) } }, 0.0, 1e-3, 13);

    // in_flight_max = queue_capacity = 1 -> default bypass cap = 2.
    auto blk = Blk::make_from_templates(m.sync, m.preamble, 10, 16, kSps, kFs,
                                        4, 8, 8, 0, 0, 14, "estimate", 0.5,
                                        0.25, 1, size_t(1) << 30, 1, false);
    BOOST_CHECK_EQUAL(blk->pending_bypass_max(), size_t(2));
    Harness h(blk);

    const uint64_t N = 48;
    pmt::pmt_t vec = pmt::init_c32vector(y.size(), y.data());
    for (uint64_t k = 0; k < N; ++k) {
        pmt::pmt_t meta = pmt::make_dict();
        meta = pmt::dict_add(meta, pmt::mp("window_id"), pmt::from_uint64(k));
        meta = pmt::dict_add(meta, pmt::mp("sample_count"),
                             pmt::from_long(static_cast<long>(y.size())));
        blk->_post(pmt::mp("in"), pmt::cons(meta, vec));
    }
    BOOST_REQUIRE(wait_drained(blk, N));
    h.stop();

    BOOST_CHECK_GT(blk->windows_queue_overflow(), uint64_t(0));
    BOOST_CHECK_EQUAL(blk->windows_dropped(), blk->windows_queue_overflow());
    BOOST_CHECK_LE(blk->bypass_fifo_high_watermark(),
                   blk->pending_bypass_max());
    BOOST_CHECK_EQUAL(blk->windows_offered(),
                      blk->windows_admitted() + blk->windows_missed_admission());
    // The emitted bypasses are still ordered and byte-exact.
    uint64_t prev = 0;
    bool first = true;
    for (size_t i = 0; i < h.out_dbg->num_messages(); ++i) {
        const pmt::pmt_t out = h.out_dbg->get_message(i);
        const pmt::pmt_t meta = pdu_meta(out);
        const uint64_t wid = meta_u64(meta, "window_id");
        if (!first)
            BOOST_CHECK(wid > prev);
        prev = wid;
        first = false;
        if (pmt::to_bool(pmt::dict_ref(meta, pmt::mp("sic_missed_admission"),
                                       pmt::PMT_F)))
            BOOST_CHECK(bit_equal(pdu_iq(out), y));
    }
    BOOST_CHECK(status_seen(h.status_dbg, "queue_overflow"));
    std::printf("[sic-adapter-qa] overflow: offered=%llu admitted=%llu missed=%llu "
                "queue_overflow=%llu dropped=%llu outs=%zu bypass_fifo_hwm=%zu "
                "cap=%zu\n",
                static_cast<unsigned long long>(blk->windows_offered()),
                static_cast<unsigned long long>(blk->windows_admitted()),
                static_cast<unsigned long long>(
                    blk->windows_missed_admission()),
                static_cast<unsigned long long>(blk->windows_queue_overflow()),
                static_cast<unsigned long long>(blk->windows_dropped()),
                h.out_dbg->num_messages(),
                blk->bypass_fifo_high_watermark(), blk->pending_bypass_max());
}

// ---------------------------------------------------------------------------
// 7. Invalid input is counted, never crashes; no queued work is produced.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_adapter_invalid_input)
{
    const Model m = build_model(10, 16);
    auto blk = Blk::make_from_templates(m.sync, m.preamble, 10, 16, kSps, kFs,
                                        4, 8, 8, 0, 0, 14, "off", 0.0, 0.0,
                                        8, size_t(1) << 30, 8, false);
    Harness h(blk);

    // 1. a symbol, not a PDU pair
    blk->_post(pmt::mp("in"), pmt::mp("not_a_pdu"));
    // 2. a pair whose payload is not c32vector
    blk->_post(pmt::mp("in"),
               pmt::cons(pmt::make_dict(),
                         pmt::init_u8vector(4, std::vector<uint8_t>(4, 0))));
    // 3. empty c32vector
    blk->_post(pmt::mp("in"),
               pmt::cons(pmt::make_dict(),
                         pmt::init_c32vector(
                             0, static_cast<const gr_complex*>(nullptr))));
    // 4. declared sample_count disagrees with the vector length
    {
        std::vector<cplx> y(64, cplx(1.f, 0.f));
        blk->_post(pmt::mp("in"), make_pdu(y, 3, 100));
    }

    BOOST_REQUIRE(wait_for([&] { return blk->windows_invalid() >= 4; }));
    h.stop();

    BOOST_CHECK_EQUAL(blk->windows_invalid(), 4u);
    BOOST_CHECK_EQUAL(blk->windows_offered(), 0u);
    BOOST_CHECK_EQUAL(blk->windows_admitted(), 0u);
    BOOST_CHECK_EQUAL(h.out_dbg->num_messages(), 0u);
    BOOST_CHECK(status_seen(h.status_dbg, "invalid_input"));
    std::printf("[sic-adapter-qa] invalid: invalid=%llu offered=%llu\n",
                static_cast<unsigned long long>(blk->windows_invalid()),
                static_cast<unsigned long long>(blk->windows_offered()));
}

// ---------------------------------------------------------------------------
// 8. stop()/drain(): completed == admitted, drained(), no residual thread.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_adapter_stop_drain)
{
    const Model m = build_model(10, 16);
    const size_t n = 4096;
    auto blk = Blk::make_from_templates(m.sync, m.preamble, 10, 16, kSps, kFs,
                                        4, 8, 8, 0, 0, 14, "off", 0.0, 0.0,
                                        8, size_t(1) << 30, 8, false);
    Harness h(blk);

    const uint64_t N = 5;
    for (uint64_t k = 0; k < N; ++k) {
        std::vector<cplx> y(n, cplx(0.f, 0.f));
        y[2] = cplx(static_cast<float>(k), 1.f);
        blk->_post(pmt::mp("in"), make_pdu(y, k));
    }
    BOOST_REQUIRE(wait_drained(blk, N));
    blk->drain(); // must return immediately
    h.stop();

    BOOST_CHECK_EQUAL(blk->windows_completed(), blk->windows_admitted());
    BOOST_CHECK_EQUAL(blk->windows_completed(), N);
    BOOST_CHECK_EQUAL(blk->windows_dropped(), 0u);
    BOOST_CHECK(blk->drained());
    BOOST_REQUIRE_EQUAL(h.out_dbg->num_messages(), N);
    // Counters are stable after stop(): no thread still publishing.
    const uint64_t c0 = blk->windows_completed();
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    BOOST_CHECK_EQUAL(blk->windows_completed(), c0);
    BOOST_CHECK(blk->drained());
    std::printf("[sic-adapter-qa] stop/drain: offered=%llu admitted=%llu "
                "completed=%llu drained=%d\n",
                static_cast<unsigned long long>(blk->windows_offered()),
                static_cast<unsigned long long>(blk->windows_admitted()),
                static_cast<unsigned long long>(blk->windows_completed()),
                static_cast<int>(blk->drained()));
}

// ---------------------------------------------------------------------------
// 9. Oracle sets sic_uses_truth=true; estimate never does.  Also checks the
//    timer keys and the core_total vs children (no double counting).
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_adapter_oracle_vs_estimate_uses_truth)
{
    const Model m = build_model(10, 16);
    const size_t n = 21256;
    const int64_t start = 2500;
    const std::vector<cplx> y =
        make_signal(m, n, start, { { 0.0, cplx(1.f, 0.f) } }, 0.0, 1e-3, 11);

    auto truth_msg = [&](uint64_t wid) {
        pmt::pmt_t t = pmt::make_dict();
        t = pmt::dict_add(t, pmt::mp("window_id"), pmt::from_uint64(wid));
        t = pmt::dict_add(t, pmt::mp("start_sample"),
                          pmt::from_double(static_cast<double>(start)));
        t = pmt::dict_add(t, pmt::mp("cfo_hz"), pmt::from_double(0.0));
        t = pmt::dict_add(t, pmt::mp("n_paths"), pmt::from_long(1));
        t = pmt::dict_add(t, pmt::mp("tau"),
                          pmt::init_f64vector(1, std::vector<double>{ 0.0 }));
        t = pmt::dict_add(t, pmt::mp("gain_re"),
                          pmt::init_f64vector(1, std::vector<double>{ 1.0 }));
        t = pmt::dict_add(t, pmt::mp("gain_im"),
                          pmt::init_f64vector(1, std::vector<double>{ 0.0 }));
        return t;
    };

    // ---- oracle: consumes the explicit truth ---------------------------
    {
        auto blk = Blk::make_from_templates(m.sync, m.preamble, 10, 16, kSps,
                                            kFs, 4, 8, 8, 0, 0, 14, "oracle",
                                            0.0, 0.0, 8, size_t(1) << 30, 8,
                                            false);
        BOOST_CHECK(blk->truth_port_enabled());
        BOOST_CHECK(blk->has_msg_port(pmt::mp("truth")));
        Harness h(blk);
        blk->_post(pmt::mp("truth"), truth_msg(42));
        blk->_post(pmt::mp("in"), make_pdu(y, 42));
        BOOST_REQUIRE(wait_drained(blk, 1));
        h.stop();

        BOOST_REQUIRE_EQUAL(h.out_dbg->num_messages(), 1u);
        const pmt::pmt_t meta = pdu_meta(h.out_dbg->get_message(0));
        BOOST_CHECK(pmt::to_bool(pmt::dict_ref(
            meta, pmt::mp("sic_uses_truth"), pmt::PMT_F)));
        BOOST_CHECK_EQUAL(
            sym(pmt::dict_ref(meta, pmt::mp("sic_state"), pmt::PMT_NIL)),
            "committed");
        BOOST_CHECK(pmt::to_bool(
            pmt::dict_ref(meta, pmt::mp("sic_committed"), pmt::PMT_F)));
        BOOST_CHECK(!bit_equal(pdu_iq(h.out_dbg->get_message(0)), y));
    }

    // ---- estimate: never reads truth, never sets uses_truth -------------
    {
        auto blk = Blk::make_from_templates(m.sync, m.preamble, 10, 16, kSps,
                                            kFs, 4, 8, 8, 0, 0, 14, "estimate",
                                            0.5, 0.25, 8, size_t(1) << 30, 8,
                                            false);
        // Truth port is oracle-only: not registered in estimate mode and the
        // block stores no truth object.  Posting to it is an error.
        BOOST_CHECK(!blk->truth_port_enabled());
        BOOST_CHECK(!blk->has_msg_port(pmt::mp("truth")));
        BOOST_CHECK_THROW(blk->_post(pmt::mp("truth"), truth_msg(42)),
                          std::runtime_error);
        Harness h(blk);
        blk->_post(pmt::mp("in"), make_pdu(y, 42));
        BOOST_REQUIRE(wait_drained(blk, 1));
        h.stop();

        BOOST_REQUIRE_EQUAL(h.out_dbg->num_messages(), 1u);
        const pmt::pmt_t meta = pdu_meta(h.out_dbg->get_message(0));
        BOOST_CHECK(!pmt::to_bool(pmt::dict_ref(
            meta, pmt::mp("sic_uses_truth"), pmt::PMT_T)));
        check_out_meta_keys(meta);
        // The estimate must actually have run and committed on this clean
        // synthetic single-path signal.
        BOOST_CHECK_EQUAL(
            sym(pmt::dict_ref(meta, pmt::mp("sic_state"), pmt::PMT_NIL)),
            "committed");
        // core_total_us is the outer clock; the stage children are sub-timers
        // and must not be double counted into it.
        const uint64_t core = meta_u64(meta, "core_total_us");
        const size_t kids = meta_children_sum(meta);
        const uint64_t max_kid = std::max({
            meta_u64(meta, "sic_detect_coarse_us"),
            meta_u64(meta, "sic_detect_fine_us"),
            meta_u64(meta, "sic_cfo_us"),
            meta_u64(meta, "sic_cir_us"),
            meta_u64(meta, "sic_path_select_us"),
            meta_u64(meta, "sic_fractional_refine_us"),
            meta_u64(meta, "sic_matrix_build_us"),
            meta_u64(meta, "sic_gain_fit_us"),
            meta_u64(meta, "sic_reconstruct_us"),
            meta_u64(meta, "sic_validate_commit_us")
        });
        std::printf("[sic-adapter-qa] estimate timers: core_total=%llu sum_children=%zu "
                    "wrap=%llu queue_wait=%llu handler=%llu latency=%llu\n",
                    static_cast<unsigned long long>(core), kids,
                    static_cast<unsigned long long>(
                        meta_u64(meta, "output_wrap_publish_us")),
                    static_cast<unsigned long long>(
                        meta_u64(meta, "queue_wait_us")),
                    static_cast<unsigned long long>(meta_u64(meta, "handler_us")),
                    static_cast<unsigned long long>(
                        meta_u64(meta, "completion_latency_us")));
        BOOST_CHECK_GT(core, uint64_t(0));
        BOOST_CHECK_GE(core, max_kid);
        BOOST_CHECK_LE(kids, core + 200); // children are sub-timers, not summed in
        BOOST_CHECK_GT(meta_u64(meta, "output_wrap_publish_us") + 1, uint64_t(0));
        // The status event mirrors the PUB-INCLUSIVE adapter timers (the out
        // meta can only carry the construction-measured portion).
        const pmt::pmt_t st_meta = find_window_status(h.status_dbg, 42);
        BOOST_REQUIRE(pmt::is_dict(st_meta));
        const uint64_t wrap_status =
            meta_u64(st_meta, "output_wrap_publish_us");
        const uint64_t lat_status =
            meta_u64(st_meta, "completion_latency_us");
        std::printf("[sic-adapter-qa] status timers: wrap_pub=%llu "
                    "latency_pub=%llu (out-meta wrap=%llu)\n",
                    static_cast<unsigned long long>(wrap_status),
                    static_cast<unsigned long long>(lat_status),
                    static_cast<unsigned long long>(
                        meta_u64(meta, "output_wrap_publish_us")));
        BOOST_CHECK_GE(wrap_status,
                       meta_u64(meta, "output_wrap_publish_us"));
        BOOST_CHECK_GE(lat_status, meta_u64(meta, "completion_latency_us"));
    }
}

// ---------------------------------------------------------------------------
// 10. Rev 2 core knobs reach the core config, and the output pool allows at
//     most ONE counted grow (never per-window growth).
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_adapter_rev2_knobs_and_one_time_grow)
{
    const Model m = build_model(10, 16);

    // Distinctive knob values must reach the core configuration.
    auto blk = Blk::make_from_templates(m.sync, m.preamble, 10, 16, kSps, kFs,
                                        4, 8, 8, 0, 0, 14, "off", 0.0, 0.0,
                                        8, size_t(1) << 30, 8, false,
                                        /*enable_fractional=*/false,
                                        /*enable_cfo=*/false,
                                        /*path_min_spacing=*/17,
                                        /*post_tail=*/64,
                                        /*max_window_samples=*/0,
                                        /*pending_bypass_max=*/5);
    BOOST_CHECK(!blk->enable_fractional());
    BOOST_CHECK(!blk->enable_cfo());
    BOOST_CHECK_EQUAL(blk->path_min_spacing(), size_t(17));
    BOOST_CHECK_EQUAL(blk->post_tail(), size_t(64));
    BOOST_CHECK_EQUAL(blk->max_window_samples(), size_t(0));
    BOOST_CHECK_EQUAL(blk->pending_bypass_max(), size_t(5));
    BOOST_CHECK_EQUAL(blk->window_grow_count(), uint64_t(0));

    // Pool growth: the first window sizes the pool; a larger second window is
    // the ONE counted grow; a larger third window is fail-closed.
    Harness h(blk);
    auto post_off = [&](uint64_t wid, size_t len) {
        std::vector<cplx> y(len, cplx(0.f, 0.f));
        y[0] = cplx(static_cast<float>(wid + 1), 0.f);
        blk->_post(pmt::mp("in"), make_pdu(y, wid));
        return y;
    };
    const std::vector<cplx> y1 = post_off(0, 1024);
    const std::vector<cplx> y2 = post_off(1, 2048);
    const std::vector<cplx> y3 = post_off(2, 4096);
    BOOST_REQUIRE(wait_drained(blk, 3));
    h.stop();

    BOOST_CHECK_EQUAL(blk->window_grow_count(), uint64_t(1));
    BOOST_CHECK_EQUAL(blk->windows_pool_capacity_exceeded(), uint64_t(1));
    BOOST_REQUIRE_EQUAL(h.out_dbg->num_messages(), 3u);
    BOOST_CHECK(bit_equal(pdu_iq(h.out_dbg->get_message(0)), y1));
    BOOST_CHECK(bit_equal(pdu_iq(h.out_dbg->get_message(1)), y2));
    // The oversized window is still terminated, bypassed bit-exactly.
    BOOST_CHECK(bit_equal(pdu_iq(h.out_dbg->get_message(2)), y3));
    BOOST_CHECK_EQUAL(sym(pmt::dict_ref(pdu_meta(h.out_dbg->get_message(2)),
                                        pmt::mp("sic_bypass_reason"),
                                        pmt::PMT_NIL)),
                      "pool_capacity_exceeded");
    // A fourth larger window must NOT grow the pool again.
    BOOST_CHECK_EQUAL(blk->window_grow_count(), uint64_t(1));

    // Pre-sized pool (max_window_samples) must not grow at all for a window
    // that fits, and must grow at most once when the first window exceeds it.
    {
        auto b2 = Blk::make_from_templates(m.sync, m.preamble, 10, 16, kSps, kFs,
                                           4, 8, 8, 0, 0, 14, "off", 0.0, 0.0,
                                           8, size_t(1) << 30, 8, false, true,
                                           true, 0, 100, /*max_window=*/2048);
        BOOST_CHECK_EQUAL(b2->max_window_samples(), size_t(2048));
        Harness h2(b2);
        b2->_post(pmt::mp("in"), make_pdu(std::vector<cplx>(1024), 0));
        BOOST_REQUIRE(wait_drained(b2, 1));
        BOOST_CHECK_EQUAL(b2->window_grow_count(), uint64_t(0));
        b2->_post(pmt::mp("in"), make_pdu(std::vector<cplx>(4096), 1));
        BOOST_REQUIRE(wait_drained(b2, 2));
        h2.stop();
        BOOST_CHECK_EQUAL(b2->window_grow_count(), uint64_t(1));
        BOOST_CHECK_EQUAL(b2->windows_pool_capacity_exceeded(), uint64_t(0));
    }

    std::printf("[sic-adapter-qa] grow: window_grow_count=%llu "
                "pool_capacity_exceeded=%llu\n",
                static_cast<unsigned long long>(blk->window_grow_count()),
                static_cast<unsigned long long>(
                    blk->windows_pool_capacity_exceeded()));
}

// ---------------------------------------------------------------------------
// Optional: shared golden fixture (testdata/sic_preamble/golden).  Uses the
// small sps=127 case; SIC off, so it only checks the PDU/PMT contract and
// byte-exact bypass on a real frozen input.  SKIPPED when absent.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_adapter_golden_fixture_optional)
{
    std::vector<uint8_t> zip;
    if (!golden_npz("int_single/io.npz", zip)) {
        std::printf("[sic-adapter-qa] golden fixture absent; skipping\n");
        return;
    }
    std::vector<uint8_t> npy;
    std::vector<cplx> y, sync, preamble;
    BOOST_REQUIRE(npz_get(zip, "y.npy", npy));
    BOOST_REQUIRE(npy_complex(npy, y));
    BOOST_REQUIRE(npz_get(zip, "sync.npy", npy));
    BOOST_REQUIRE(npy_complex(npy, sync));
    BOOST_REQUIRE(npz_get(zip, "preamble.npy", npy));
    BOOST_REQUIRE(npy_complex(npy, preamble));
    std::printf("[sic-adapter-qa] golden int_single: |y|=%zu |sync|=%zu |preamble|=%zu\n",
                y.size(), sync.size(), preamble.size());

    auto blk = Blk::make_from_templates(sync, preamble, 10, 16, 127, kFs, 4, 8,
                                        8, 0, 0, 14, "off", 0.0, 0.0, 8,
                                        size_t(1) << 30, 8, true);
    Harness h(blk, true);
    blk->_post(pmt::mp("in"), make_pdu(y, 1));
    BOOST_REQUIRE(wait_drained(blk, 1));
    h.stop();

    BOOST_REQUIRE_EQUAL(h.out_dbg->num_messages(), 1u);
    BOOST_CHECK(bit_equal(pdu_iq(h.out_dbg->get_message(0)), y));
    BOOST_CHECK(all_zero(pdu_iq(h.ihat_dbg->get_message(0))));
}
