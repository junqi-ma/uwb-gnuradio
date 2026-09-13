/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * QA for the SC16 amplitude contract of UwbPduRationalResamplerCcf65_32
 * (491.52 -> 998.4 MS/s, fixed 65/32).  Contract: round-2 spec section A
 * (docs/phase1/整改指南_UWB_Radar_PDU速率优化验收.md §2).
 *
 * What this proves:
 *   A: native SC16 PDU with Sc16ScalePolicy::UnitRange
 *       (fc32 = float(int16) / 32768)
 *   B: FC32 PDU built as complex(int16_i / 32768, int16_q / 32768) with the
 *       default Sc16ScalePolicy::RawInteger
 *   -> the two paths emit sample-identical payload and metadata, i.e. the
 *      UnitRange SC16 chain matches the legacy UHD Python FC32 contract.
 *   C: native SC16 PDU with the default RawInteger policy
 *       -> C output == A output * 32768 (within float tolerance), proving the
 *          default keeps the legacy integer-amplitude communication chain
 *          unchanged.
 *
 * It also checks input_iq_scale / output_iq_scale metadata and the
 * published_samples passthrough required for ROI visibility.
 *
 * Taps: uses UWB_TESTDATA_DIR when the target defines it
 * (resampler_65_32/taps_quality_minorder.txt, as the existing QA does),
 * otherwise a deterministic in-memory windowed-sinc so the test is
 * self-contained.
 */

#include <boost/test/unit_test.hpp>

#include <gnuradio/blocks/message_debug.h>
#include <gnuradio/top_block.h>
#include <gnuradio/uwb/uwb_pdu_rational_resampler_ccf_65_32.h>
#include <pmt/pmt.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <limits>
#include <string>
#include <thread>
#include <vector>

using gr::uwb::UwbPduRationalResamplerCcf65_32;
using gr_complex = std::complex<float>;
using Sc16ScalePolicy = UwbPduRationalResamplerCcf65_32::Sc16ScalePolicy;

namespace {

constexpr double kInRate = 491.52e6;
constexpr double kOutRate = 998.4e6;
constexpr float kMatchTol = 1e-7f; // A (UnitRange SC16) vs B (FC32)
constexpr float kRawRelTol = 1e-4f; // C (RawInteger) vs A * 32768
constexpr size_t kMaxIn = 262144;
constexpr double kPi = 3.14159265358979323846;

std::vector<float> synth_taps(size_t T)
{
    std::vector<float> h(T);
    const double c = 0.5 * static_cast<double>(T > 0 ? T - 1 : 0);
    for (size_t i = 0; i < T; ++i) {
        const double x = static_cast<double>(i) - c;
        const double s =
            (std::abs(x) < 1e-9) ? 1.0 : std::sin(0.35 * x) / (0.35 * x);
        const double w = (T > 1)
                             ? 0.5 - 0.5 * std::cos(2.0 * kPi *
                                                   static_cast<double>(i) /
                                                   static_cast<double>(T - 1))
                             : 1.0;
        h[i] = static_cast<float>(s * w / 4.0);
    }
    return h;
}

bool try_load_real_taps(std::vector<float>& out)
{
#ifdef UWB_TESTDATA_DIR
    const std::string path =
        std::string(UWB_TESTDATA_DIR) +
        "/resampler_65_32/taps_quality_minorder.txt";
    std::ifstream f(path, std::ios::binary | std::ios::ate);
    if (!f)
        return false;
    const auto bytes = static_cast<size_t>(f.tellg());
    if (bytes == 0 || bytes % sizeof(float) != 0)
        return false;
    f.seekg(0);
    out.resize(bytes / sizeof(float));
    f.read(reinterpret_cast<char*>(out.data()),
           static_cast<std::streamsize>(bytes));
    return static_cast<bool>(f);
#else
    (void)out;
    return false;
#endif
}

const std::vector<float>& test_taps()
{
    static const std::vector<float> t = [] {
        std::vector<float> v;
        if (try_load_real_taps(v))
            return v;
        return synth_taps(1024);
    }();
    return t;
}

pmt::pmt_t make_meta(int64_t ws,
                     int64_t pre,
                     int64_t cap,
                     int64_t post,
                     double rate,
                     bool with_published,
                     int64_t published)
{
    pmt::pmt_t meta = pmt::make_dict();
    meta = pmt::dict_add(meta, pmt::mp("sample_rate"), pmt::from_double(rate));
    meta =
        pmt::dict_add(meta, pmt::mp("window_start_sample"), pmt::from_long(ws));
    meta = pmt::dict_add(meta, pmt::mp("pre_guard_samples"),
                         pmt::from_long(pre));
    meta = pmt::dict_add(meta, pmt::mp("capture_samples"), pmt::from_long(cap));
    meta = pmt::dict_add(meta, pmt::mp("post_guard_samples"),
                         pmt::from_long(post));
    if (with_published) {
        meta = pmt::dict_add(meta, pmt::mp("published_samples"),
                             pmt::from_long(published));
    }
    return meta;
}

pmt::pmt_t make_c32_pdu(const std::vector<gr_complex>& x,
                        int64_t ws,
                        int64_t pre,
                        int64_t cap,
                        int64_t post,
                        double rate,
                        bool with_published,
                        int64_t published)
{
    pmt::pmt_t meta = make_meta(ws, pre, cap, post, rate, with_published,
                                published);
    meta = pmt::dict_add(meta, pmt::mp("sample_count"),
                         pmt::from_long(static_cast<long>(x.size())));
    pmt::pmt_t vec = pmt::init_c32vector(x.size(), x.data());
    return pmt::cons(meta, vec);
}

pmt::pmt_t make_s16_pdu(const std::vector<int16_t>& iq,
                        int64_t ws,
                        int64_t pre,
                        int64_t cap,
                        int64_t post,
                        double rate,
                        bool with_published,
                        int64_t published)
{
    pmt::pmt_t meta = make_meta(ws, pre, cap, post, rate, with_published,
                                published);
    meta = pmt::dict_add(meta, pmt::mp("sample_count"),
                         pmt::from_long(static_cast<long>(iq.size() / 2)));
    pmt::pmt_t vec = pmt::init_s16vector(iq.size(), iq.data());
    return pmt::cons(meta, vec);
}

struct RunResult {
    size_t packets = 0;
    std::vector<pmt::pmt_t> statuses;
    pmt::pmt_t packet{ pmt::PMT_NIL };
};

RunResult run_pdu(UwbPduRationalResamplerCcf65_32::sptr blk,
                  pmt::pmt_t pdu,
                  int timeout_ms = 10000)
{
    auto dbg = gr::blocks::message_debug::make();
    auto st = gr::blocks::message_debug::make();
    auto tb = gr::make_top_block("qa_pdu_sc16_scale_65_32");
    tb->msg_connect(blk, "packet", dbg, "store");
    tb->msg_connect(blk, "status", st, "store");
    tb->start();
    blk->_post(pmt::mp("packet"), pdu);

    const int iters = timeout_ms / 5;
    for (int i = 0; i < iters; ++i) {
        if (dbg->num_messages() > 0)
            break;
        if (st->num_messages() > 0 && i > 40)
            break;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    tb->stop();
    tb->wait();

    RunResult r;
    r.packets = dbg->num_messages();
    if (r.packets > 0)
        r.packet = dbg->get_message(0);
    for (size_t i = 0; i < st->num_messages(); ++i)
        r.statuses.push_back(st->get_message(i));
    return r;
}

std::vector<gr_complex> payload_of(pmt::pmt_t pdu)
{
    size_t n = 0;
    const gr_complex* p = pmt::c32vector_elements(pmt::cdr(pdu), n);
    return std::vector<gr_complex>(p, p + n);
}

float max_abs_diff(const std::vector<gr_complex>& a,
                   const std::vector<gr_complex>& b)
{
    if (a.size() != b.size())
        return std::numeric_limits<float>::infinity();
    float m = 0.f;
    for (size_t i = 0; i < a.size(); ++i) {
        const float e = std::abs(a[i] - b[i]);
        if (e > m)
            m = e;
    }
    return m;
}

float max_abs(const std::vector<gr_complex>& a)
{
    float m = 0.f;
    for (const auto& v : a) {
        const float e = std::abs(v);
        if (e > m)
            m = e;
    }
    return m;
}

int64_t meta_long(pmt::pmt_t meta, const char* key)
{
    return pmt::to_long(
        pmt::dict_ref(meta, pmt::mp(key), pmt::from_long(-1)));
}

bool meta_has(pmt::pmt_t meta, const char* key)
{
    return pmt::is_dict(meta) && pmt::dict_has_key(meta, pmt::mp(key));
}

std::vector<std::string> events_of(const std::vector<pmt::pmt_t>& statuses)
{
    std::vector<std::string> out;
    for (pmt::pmt_t s : statuses) {
        pmt::pmt_t e = pmt::dict_ref(s, pmt::mp("event"), pmt::PMT_NIL);
        out.push_back(pmt::is_symbol(e) ? pmt::symbol_to_string(e)
                                        : std::string());
    }
    return out;
}

// Deterministic int16 IQ code words shared by all three arms.
std::vector<int16_t> make_sc16(size_t n)
{
    std::vector<int16_t> iq(2 * n);
    for (size_t i = 0; i < n; ++i) {
        const int16_t re = static_cast<int16_t>((i * 337 + 11) % 60000 - 30000);
        const int16_t im = static_cast<int16_t>((i * 197 + 7) % 60000 - 30000);
        iq[2 * i] = re;
        iq[2 * i + 1] = im;
    }
    return iq;
}

std::vector<gr_complex> unit_range_fc32(const std::vector<int16_t>& iq)
{
    const size_t n = iq.size() / 2;
    std::vector<gr_complex> fc(n);
    for (size_t i = 0; i < n; ++i) {
        fc[i] = gr_complex(static_cast<float>(iq[2 * i]) / 32768.0f,
                           static_cast<float>(iq[2 * i + 1]) / 32768.0f);
    }
    return fc;
}

constexpr size_t kN = 32768;
constexpr int64_t kPre = 8192;
constexpr int64_t kPost = 8192;
constexpr int64_t kCap = static_cast<int64_t>(kN) - kPre - kPost;
constexpr int64_t kPublished = 12345;

} // namespace

// ---------------------------------------------------------------------------
// A (SC16 UnitRange) == B (FC32 / 32768, default RawInteger)
// C (SC16 RawInteger) == A * 32768
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_65_32_sc16_unit_range_matches_fc32)
{
    const auto taps = test_taps();

    auto blk_a = UwbPduRationalResamplerCcf65_32::make_from_taps(
        taps, kOutRate, true,
        UwbPduRationalResamplerCcf65_32::EmitPolicy::FullWindow, kMaxIn,
        /*num_workers=*/1, Sc16ScalePolicy::UnitRange);
    auto blk_b = UwbPduRationalResamplerCcf65_32::make_from_taps(
        taps, kOutRate, true,
        UwbPduRationalResamplerCcf65_32::EmitPolicy::FullWindow, kMaxIn,
        /*num_workers=*/1); // default RawInteger
    auto blk_c = UwbPduRationalResamplerCcf65_32::make_from_taps(
        taps, kOutRate, true,
        UwbPduRationalResamplerCcf65_32::EmitPolicy::FullWindow, kMaxIn,
        /*num_workers=*/1, Sc16ScalePolicy::RawInteger);

    // Policy accessors.
    BOOST_CHECK(blk_a->sc16_scale() == Sc16ScalePolicy::UnitRange);
    BOOST_CHECK(blk_b->sc16_scale() == Sc16ScalePolicy::RawInteger);
    blk_c->set_sc16_scale(Sc16ScalePolicy::UnitRange);
    BOOST_CHECK(blk_c->sc16_scale() == Sc16ScalePolicy::UnitRange);
    blk_c->set_sc16_scale(Sc16ScalePolicy::RawInteger);
    BOOST_CHECK(blk_c->sc16_scale() == Sc16ScalePolicy::RawInteger);

    const auto iq = make_sc16(kN);
    const auto fc = unit_range_fc32(iq);

    auto ra = run_pdu(blk_a,
                      make_s16_pdu(iq, 0, kPre, kCap, kPost, kInRate, true,
                                   kPublished));
    auto rb = run_pdu(blk_b,
                      make_c32_pdu(fc, 0, kPre, kCap, kPost, kInRate, true,
                                   kPublished));
    auto rc = run_pdu(blk_c,
                      make_s16_pdu(iq, 0, kPre, kCap, kPost, kInRate, true,
                                   kPublished));

    BOOST_REQUIRE_EQUAL(ra.packets, 1u);
    BOOST_REQUIRE_EQUAL(rb.packets, 1u);
    BOOST_REQUIRE_EQUAL(rc.packets, 1u);

    // status consistency (same events, same order; clean runs → empty).
    BOOST_CHECK(events_of(ra.statuses) == events_of(rb.statuses));
    BOOST_CHECK_EQUAL(blk_a->pdus_dropped(), 0u);
    BOOST_CHECK_EQUAL(blk_b->pdus_dropped(), 0u);
    BOOST_CHECK_EQUAL(blk_c->pdus_dropped(), 0u);

    const auto ya = payload_of(ra.packet);
    const auto yb = payload_of(rb.packet);
    const auto yc = payload_of(rc.packet);
    BOOST_REQUIRE_EQUAL(ya.size(), yb.size());
    BOOST_REQUIRE_EQUAL(ya.size(), yc.size());

    const float diff_ab = max_abs_diff(ya, yb);
    BOOST_TEST_MESSAGE("UnitRange SC16 vs FC32/32768: size="
                       << ya.size() << " max_abs_diff=" << diff_ab);
    BOOST_CHECK_LE(diff_ab, kMatchTol);

    // C = A * 32768 (float tolerance; power-of-two scaling is exact inputs).
    std::vector<gr_complex> ya_scaled(ya.size());
    for (size_t i = 0; i < ya.size(); ++i)
        ya_scaled[i] = ya[i] * 32768.0f;
    const float diff_ca = max_abs_diff(yc, ya_scaled);
    const float scale = std::max(1.0f, max_abs(yc));
    const float diff_ca_rel = diff_ca / scale;
    BOOST_TEST_MESSAGE("RawInteger SC16 vs UnitRange*32768: max_abs_diff="
                       << diff_ca << " max|C|=" << max_abs(yc)
                       << " rel=" << diff_ca_rel);
    BOOST_CHECK_LE(diff_ca_rel, kRawRelTol);

    // Geometry metadata must be field-for-field identical across A/B/C.
    pmt::pmt_t ma = pmt::car(ra.packet);
    pmt::pmt_t mb = pmt::car(rb.packet);
    pmt::pmt_t mc = pmt::car(rc.packet);
    for (const char* key :
         { "sample_count", "capture_samples", "pre_guard_samples",
           "post_guard_samples", "window_start_sample",
           "full_output_sample_count", "input_sample_count" }) {
        const int64_t va = meta_long(ma, key);
        const int64_t vb = meta_long(mb, key);
        const int64_t vc = meta_long(mc, key);
        BOOST_TEST_MESSAGE("meta " << key << ": A=" << va << " B=" << vb
                                   << " C=" << vc);
        BOOST_CHECK_EQUAL(va, vb);
        BOOST_CHECK_EQUAL(va, vc);
    }

    // Amplitude contract metadata.
    // A: SC16 UnitRange -> both scales are 1.
    BOOST_CHECK_EQUAL(meta_long(ma, "input_iq_scale"), 1);
    BOOST_CHECK_EQUAL(meta_long(ma, "output_iq_scale"), 1);
    BOOST_CHECK_EQUAL(std::string(pmt::symbol_to_string(
                          pmt::dict_ref(ma, pmt::mp("input_sample_format"),
                                        pmt::mp("?")))),
                      "sc16");
    // B: FC32 input -> input scale 1 regardless of the RawInteger policy.
    BOOST_CHECK_EQUAL(meta_long(mb, "input_iq_scale"), 1);
    BOOST_CHECK_EQUAL(meta_long(mb, "output_iq_scale"), 1);
    BOOST_CHECK_EQUAL(std::string(pmt::symbol_to_string(
                          pmt::dict_ref(mb, pmt::mp("input_sample_format"),
                                        pmt::mp("?")))),
                      "fc32");
    // C: SC16 RawInteger -> legacy integer amplitudes, scale 32768.
    BOOST_CHECK_EQUAL(meta_long(mc, "input_iq_scale"), 32768);
    BOOST_CHECK_EQUAL(meta_long(mc, "output_iq_scale"), 1);

    // published_samples passthrough (present in -> present out, unchanged).
    BOOST_CHECK_EQUAL(meta_long(ma, "published_samples"), kPublished);
    BOOST_CHECK_EQUAL(meta_long(mb, "published_samples"), kPublished);
    BOOST_CHECK_EQUAL(meta_long(mc, "published_samples"), kPublished);
}

// ---------------------------------------------------------------------------
// published_samples is only copied when present; never invented.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_65_32_published_samples_absent_stays_absent)
{
    auto blk = UwbPduRationalResamplerCcf65_32::make_from_taps(
        test_taps(), kOutRate, true,
        UwbPduRationalResamplerCcf65_32::EmitPolicy::FullWindow, kMaxIn, 1,
        Sc16ScalePolicy::UnitRange);
    const auto iq = make_sc16(kN);
    auto r = run_pdu(blk,
                     make_s16_pdu(iq, 0, kPre, kCap, kPost, kInRate,
                                  /*with_published=*/false, 0));
    BOOST_REQUIRE_EQUAL(r.packets, 1u);
    pmt::pmt_t m = pmt::car(r.packet);
    BOOST_CHECK(!meta_has(m, "published_samples"));
    // UnitRange SC16 still advertises scale 1.
    BOOST_CHECK_EQUAL(meta_long(m, "input_iq_scale"), 1);
    BOOST_CHECK_EQUAL(meta_long(m, "output_iq_scale"), 1);
}
