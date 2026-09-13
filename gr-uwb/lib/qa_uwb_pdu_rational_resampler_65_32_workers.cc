/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * QA for the multi-worker path of UwbPduRationalResamplerCcf65_32
 * (491.52 -> 998.4 MS/s, fixed 65/32).
 *
 * Contract (docs/... pdu_iface_spec.md section B):
 *   - make()/make_from_taps()/constructor take a trailing `int num_workers`
 *     that builds the core's persistent FIR pool at construction;
 *   - set_num_workers()/num_workers()/resampler_kernel() are readable;
 *   - the output must be sample-identical across 1/4/8 workers
 *     (max|diff| <= 1e-6, expected bit-exact);
 *   - malformed / oversize PDUs still drop, repeated start is safe.
 *
 * Taps: uses UWB_TESTDATA_DIR when the target defines it (real
 * taps_quality_minorder), otherwise a deterministic in-memory windowed-sinc
 * so the test is self-contained.
 */

#include <boost/test/unit_test.hpp>

#include <gnuradio/blocks/message_debug.h>
#include <gnuradio/top_block.h>
#include <gnuradio/uwb/uwb_pdu_rational_resampler_ccf_65_32.h>
#include <pmt/pmt.h>

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

namespace {

constexpr double kInRate = 491.52e6;
constexpr double kOutRate = 998.4e6;
constexpr float kWorkerTol = 1e-6f;
constexpr size_t kMaxIn = 262144; // scratch bound for the large-window cases
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

pmt::pmt_t make_window_pdu(const std::vector<gr_complex>& x,
                           int64_t ws,
                           int64_t pre,
                           int64_t cap,
                           int64_t post,
                           double rate)
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
    meta = pmt::dict_add(meta, pmt::mp("sample_count"),
                         pmt::from_long(static_cast<long>(x.size())));
    pmt::pmt_t vec = pmt::init_c32vector(x.size(), x.data());
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
    auto tb = gr::make_top_block("qa_pdu_65_32_workers");
    tb->msg_connect(blk, "packet", dbg, "store");
    tb->msg_connect(blk, "status", st, "store");
    tb->start();
    blk->_post(pmt::mp("packet"), pdu);

    const int iters = timeout_ms / 5;
    for (int i = 0; i < iters; ++i) {
        if (dbg->num_messages() > 0)
            break;
        // A pure drop only produces a status; settle briefly then give up.
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

int64_t meta_long(pmt::pmt_t meta, const char* key)
{
    return pmt::to_long(
        pmt::dict_ref(meta, pmt::mp(key), pmt::from_long(-1)));
}

std::vector<gr_complex> make_input(size_t n, uint32_t seed)
{
    std::vector<gr_complex> x(n);
    uint32_t s = seed ? seed : 1u;
    for (size_t i = 0; i < n; ++i) {
        s = s * 1664525u + 1013904223u; // LCG, deterministic
        const float re = static_cast<float>((s >> 8) & 0xffff) / 32768.0f - 1.0f;
        s = s * 1664525u + 1013904223u;
        const float im = static_cast<float>((s >> 8) & 0xffff) / 32768.0f - 1.0f;
        x[i] = gr_complex(re, im);
    }
    return x;
}

struct WindowCase {
    size_t n_in;
    int worker;
    std::vector<gr_complex> payload;
    pmt::pmt_t meta;
    size_t emitted;
    size_t dropped;
    pmt::pmt_t pdu;
};

WindowCase run_window(size_t n_in, int workers)
{
    auto blk = UwbPduRationalResamplerCcf65_32::make_from_taps(
        test_taps(), kOutRate, true,
        UwbPduRationalResamplerCcf65_32::EmitPolicy::FullWindow, kMaxIn,
        workers);
    // Input must be independent of the worker count, otherwise the
    // bit-identical comparison below is meaningless.
    const auto x = make_input(n_in, static_cast<uint32_t>(n_in));
    const int64_t pre = 512;
    const int64_t post = 256;
    const int64_t cap = static_cast<int64_t>(n_in) - pre - post;
    auto res = run_pdu(blk, make_window_pdu(x, 0, pre, cap, post, kInRate));

    WindowCase wc;
    wc.n_in = n_in;
    wc.worker = workers;
    wc.emitted = blk->pdus_emitted();
    wc.dropped = blk->pdus_dropped();
    wc.pdu = res.packet;
    if (res.packets > 0) {
        wc.payload = payload_of(res.packet);
        wc.meta = pmt::car(res.packet);
    }
    return wc;
}

std::string event_of(pmt::pmt_t status)
{
    pmt::pmt_t e = pmt::dict_ref(status, pmt::mp("event"), pmt::PMT_NIL);
    return pmt::is_symbol(e) ? pmt::symbol_to_string(e) : std::string();
}

} // namespace

BOOST_AUTO_TEST_CASE(test_65_32_worker_output_identical)
{
    for (size_t n_in : { size_t(1000), size_t(100000) }) {
        const WindowCase base = run_window(n_in, 1);
        BOOST_REQUIRE_MESSAGE(base.emitted == 1,
                              "n_in=" << n_in << " worker=1 did not emit");
        BOOST_CHECK_EQUAL(base.dropped, 0u);

        for (int nw : { 4, 8 }) {
            const WindowCase w = run_window(n_in, nw);
            BOOST_REQUIRE_MESSAGE(w.emitted == 1,
                                  "n_in=" << n_in << " worker=" << nw
                                          << " did not emit");
            BOOST_CHECK_EQUAL(w.dropped, 0u);
            BOOST_REQUIRE_EQUAL(w.payload.size(), base.payload.size());

            const float diff = max_abs_diff(w.payload, base.payload);
            BOOST_TEST_MESSAGE("n_in=" << n_in << " workers=" << nw
                                       << " size=" << w.payload.size()
                                       << " max_abs_diff=" << diff);
            BOOST_CHECK_LE(diff, kWorkerTol);
            BOOST_CHECK_EQUAL(w.payload.size(), base.payload.size());

            // The 65/32 mapping metadata must not depend on worker count.
            BOOST_CHECK_EQUAL(meta_long(w.meta, "sample_count"),
                              meta_long(base.meta, "sample_count"));
            BOOST_CHECK_EQUAL(meta_long(w.meta, "capture_samples"),
                              meta_long(base.meta, "capture_samples"));
            BOOST_CHECK_EQUAL(meta_long(w.meta, "pre_guard_samples"),
                              meta_long(base.meta, "pre_guard_samples"));
            BOOST_CHECK_EQUAL(meta_long(w.meta, "post_guard_samples"),
                              meta_long(base.meta, "post_guard_samples"));
            BOOST_CHECK_EQUAL(meta_long(w.meta, "window_start_sample"),
                              meta_long(base.meta, "window_start_sample"));
            BOOST_CHECK_EQUAL(meta_long(w.meta, "full_output_sample_count"),
                              meta_long(base.meta, "full_output_sample_count"));
            BOOST_CHECK_EQUAL(meta_long(w.meta, "input_sample_count"),
                              meta_long(base.meta, "input_sample_count"));
        }
    }
}

BOOST_AUTO_TEST_CASE(test_65_32_worker_default_is_one)
{
    const auto taps = test_taps();
    // Default construction (no num_workers argument) must report 1 and match
    // an explicit num_workers=1 block bit-for-bit.
    auto def = UwbPduRationalResamplerCcf65_32::make_from_taps(
        taps, kOutRate, true,
        UwbPduRationalResamplerCcf65_32::EmitPolicy::FullWindow, kMaxIn);
    BOOST_CHECK_EQUAL(def->num_workers(), 1);

    auto explicit1 = UwbPduRationalResamplerCcf65_32::make_from_taps(
        taps, kOutRate, true,
        UwbPduRationalResamplerCcf65_32::EmitPolicy::FullWindow, kMaxIn, 1);
    BOOST_CHECK_EQUAL(explicit1->num_workers(), 1);

    const auto x = make_input(4096, 12345u);
    const int64_t pre = 256;
    const int64_t post = 256;
    const int64_t cap = static_cast<int64_t>(x.size()) - pre - post;
    auto rd = run_pdu(def, make_window_pdu(x, 0, pre, cap, post, kInRate));
    auto re = run_pdu(explicit1, make_window_pdu(x, 0, pre, cap, post, kInRate));
    BOOST_REQUIRE_EQUAL(rd.packets, 1u);
    BOOST_REQUIRE_EQUAL(re.packets, 1u);
    const float diff = max_abs_diff(payload_of(rd.packet), payload_of(re.packet));
    BOOST_TEST_MESSAGE("default-vs-explicit-1 max_abs_diff=" << diff);
    BOOST_CHECK_LE(diff, kWorkerTol);
}

BOOST_AUTO_TEST_CASE(test_65_32_worker_accessors_and_set)
{
    auto blk = UwbPduRationalResamplerCcf65_32::make_from_taps(
        test_taps(), kOutRate, true,
        UwbPduRationalResamplerCcf65_32::EmitPolicy::FullWindow, kMaxIn, 4);
    BOOST_CHECK_EQUAL(blk->num_workers(), 4);
    BOOST_REQUIRE(blk->resampler_kernel() != nullptr);
    const std::string kernel(blk->resampler_kernel());
    BOOST_TEST_MESSAGE("resampler_kernel()=" << kernel);
    BOOST_CHECK(kernel == "avx2_fma_macroblock" ||
                kernel == "volk_macroblock" ||
                kernel == "scalar_macroblock");

    // Clamp below 1.
    blk->set_num_workers(0);
    BOOST_CHECK_EQUAL(blk->num_workers(), 1);
    blk->set_num_workers(-7);
    BOOST_CHECK_EQUAL(blk->num_workers(), 1);
    blk->set_num_workers(3);
    BOOST_CHECK_EQUAL(blk->num_workers(), 3);

    // set_num_workers() after construction must be numerically equivalent to
    // constructing with that count.
    auto built1 = UwbPduRationalResamplerCcf65_32::make_from_taps(
        test_taps(), kOutRate, true,
        UwbPduRationalResamplerCcf65_32::EmitPolicy::FullWindow, kMaxIn, 1);
    built1->set_num_workers(3);
    BOOST_CHECK_EQUAL(built1->num_workers(), 3);

    const auto x = make_input(8192, 777u);
    const int64_t pre = 512;
    const int64_t post = 512;
    const int64_t cap = static_cast<int64_t>(x.size()) - pre - post;
    auto a = run_pdu(blk, make_window_pdu(x, 0, pre, cap, post, kInRate));
    auto b = run_pdu(built1, make_window_pdu(x, 0, pre, cap, post, kInRate));
    BOOST_REQUIRE_EQUAL(a.packets, 1u);
    BOOST_REQUIRE_EQUAL(b.packets, 1u);
    const float diff = max_abs_diff(payload_of(a.packet), payload_of(b.packet));
    BOOST_TEST_MESSAGE("set-in-place-vs-built 3-worker max_abs_diff=" << diff);
    BOOST_CHECK_LE(diff, kWorkerTol);
}

BOOST_AUTO_TEST_CASE(test_65_32_worker_kernel_for_long_taps)
{
    // H = ceil((T + 64)/65).  H > 48 selects the VOLK kernel deterministically.
    const auto long_taps = synth_taps(8192); // H = 128
    auto blk = UwbPduRationalResamplerCcf65_32::make_from_taps(
        long_taps, kOutRate, true,
        UwbPduRationalResamplerCcf65_32::EmitPolicy::FullWindow, kMaxIn, 2);
    BOOST_REQUIRE(blk->resampler_kernel() != nullptr);
    BOOST_CHECK_EQUAL(std::string(blk->resampler_kernel()), "volk_macroblock");
}

BOOST_AUTO_TEST_CASE(test_65_32_worker_drops_malformed_and_oversize)
{
    auto blk = UwbPduRationalResamplerCcf65_32::make_from_taps(
        test_taps(), kOutRate, true,
        UwbPduRationalResamplerCcf65_32::EmitPolicy::FullWindow, size_t(64), 2);
    BOOST_CHECK_EQUAL(blk->num_workers(), 2);

    // Not a pair.
    auto r1 = run_pdu(blk, pmt::from_long(5));
    BOOST_CHECK_EQUAL(r1.packets, 0u);

    // Pair but payload is not a c32/s16 vector.
    pmt::pmt_t meta = pmt::make_dict();
    meta = pmt::dict_add(meta, pmt::mp("sample_rate"),
                         pmt::from_double(kInRate));
    auto r2 = run_pdu(blk, pmt::cons(meta, pmt::from_long(1)));
    BOOST_CHECK_EQUAL(r2.packets, 0u);

    // Empty vector.
    std::vector<gr_complex> empty;
    auto r3 = run_pdu(blk, make_window_pdu(empty, 0, 0, 0, 0, kInRate));
    BOOST_CHECK_EQUAL(r3.packets, 0u);

    // Oversize: n_in > max_input_samples (64).
    const auto big = make_input(1000, 99u);
    auto r4 = run_pdu(blk, make_window_pdu(big, 0, 10, 1000, 14, kInRate));
    BOOST_CHECK_EQUAL(r4.packets, 0u);

    // All four failed without emitting and were counted as drops/receives.
    BOOST_CHECK_EQUAL(blk->pdus_emitted(), 0u);
    BOOST_CHECK_EQUAL(blk->pdus_received(), 4u);
    BOOST_CHECK_GE(blk->pdus_dropped(), 4u);

    bool saw_bad_rate = false;
    bool saw_invalid_window = false;
    for (pmt::pmt_t s : r4.statuses) {
        const std::string e = event_of(s);
        if (e == "invalid_window")
            saw_invalid_window = true;
    }
    auto r5 = run_pdu(blk, make_window_pdu(big, 0, 10, 1000, 14, 737.28e6));
    BOOST_CHECK_EQUAL(r5.packets, 0u);
    for (pmt::pmt_t s : r5.statuses) {
        if (event_of(s) == "bad_input_rate")
            saw_bad_rate = true;
    }
    BOOST_TEST_MESSAGE("saw_invalid_window=" << saw_invalid_window
                                             << " saw_bad_rate=" << saw_bad_rate);
    BOOST_CHECK(saw_invalid_window);
    BOOST_CHECK(saw_bad_rate);
}

BOOST_AUTO_TEST_CASE(test_65_32_worker_restart_repeatable)
{
    auto blk = UwbPduRationalResamplerCcf65_32::make_from_taps(
        test_taps(), kOutRate, true,
        UwbPduRationalResamplerCcf65_32::EmitPolicy::FullWindow, kMaxIn, 4);
    const auto x = make_input(2048, 2026u);
    const int64_t pre = 256;
    const int64_t post = 256;
    const int64_t cap = static_cast<int64_t>(x.size()) - pre - post;

    std::vector<gr_complex> first;
    for (int rep = 0; rep < 3; ++rep) {
        auto r = run_pdu(blk, make_window_pdu(x, 0, pre, cap, post, kInRate));
        BOOST_REQUIRE_EQUAL(r.packets, 1u);
        auto y = payload_of(r.packet);
        if (rep == 0)
            first = y;
        else
            BOOST_CHECK_LE(max_abs_diff(y, first), kWorkerTol);
    }
    BOOST_CHECK_EQUAL(blk->pdus_received(), 3u);
    BOOST_CHECK_EQUAL(blk->pdus_emitted(), 3u);
    BOOST_CHECK_EQUAL(blk->pdus_dropped(), 0u);
}

// ---------------------------------------------------------------------------
// SC16 input equals the FC32 path for the same int16 code words.
//
// This is the payload contract the C++ PDU EchoTimer relies on: it publishes
// native SC16 and the resampler converts internally (no scaling by 32768),
// so an int16 PDU must resample exactly like the FC32 PDU built from
// static_cast<float>(int16).
// ---------------------------------------------------------------------------
pmt::pmt_t make_window_pdu_sc16(const std::vector<int16_t>& iq,
                                int64_t ws,
                                int64_t pre,
                                int64_t cap,
                                int64_t post,
                                double rate)
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
    meta = pmt::dict_add(meta, pmt::mp("sample_count"),
                         pmt::from_long(iq.size() / 2));
    pmt::pmt_t vec = pmt::init_s16vector(iq.size(), iq.data());
    return pmt::cons(meta, vec);
}

BOOST_AUTO_TEST_CASE(test_65_32_sc16_input_matches_fc32)
{
    auto blk = UwbPduRationalResamplerCcf65_32::make_from_taps(
        test_taps(), kOutRate, true,
        UwbPduRationalResamplerCcf65_32::EmitPolicy::FullWindow, kMaxIn, 2);

    const size_t n = 4096;
    std::vector<int16_t> iq(2 * n);
    std::vector<gr_complex> fc(n);
    for (size_t i = 0; i < n; ++i) {
        const int16_t re = static_cast<int16_t>((i * 337 + 11) % 60000 - 30000);
        const int16_t im = static_cast<int16_t>((i * 197 + 7) % 60000 - 30000);
        iq[2 * i] = re;
        iq[2 * i + 1] = im;
        fc[i] = gr_complex(static_cast<float>(re), static_cast<float>(im));
    }
    const int64_t pre = 512;
    const int64_t post = 256;
    const int64_t cap = static_cast<int64_t>(n) - pre - post;

    auto r_sc16 =
        run_pdu(blk, make_window_pdu_sc16(iq, 0, pre, cap, post, kInRate));
    auto r_fc = run_pdu(blk, make_window_pdu(fc, 0, pre, cap, post, kInRate));
    BOOST_REQUIRE_EQUAL(r_sc16.packets, 1u);
    BOOST_REQUIRE_EQUAL(r_fc.packets, 1u);
    BOOST_REQUIRE_EQUAL(payload_of(r_sc16.packet).size(),
                        payload_of(r_fc.packet).size());
    BOOST_CHECK_LE(max_abs_diff(payload_of(r_sc16.packet),
                                payload_of(r_fc.packet)),
                   kWorkerTol);
    // Geometry metadata must match; input_sample_format is the only
    // intended difference and is asserted separately.
    pmt::pmt_t ms = pmt::car(r_sc16.packet);
    pmt::pmt_t mf = pmt::car(r_fc.packet);
    for (const char* key : { "sample_count", "capture_samples",
                             "pre_guard_samples", "post_guard_samples",
                             "window_start_sample", "full_output_sample_count",
                             "input_sample_count" }) {
        BOOST_CHECK_EQUAL(meta_long(ms, key), meta_long(mf, key));
    }
    BOOST_CHECK_EQUAL(
        pmt::symbol_to_string(pmt::dict_ref(ms, pmt::mp("input_sample_format"),
                                            pmt::mp("?"))),
        "sc16");
    BOOST_CHECK_EQUAL(
        pmt::symbol_to_string(pmt::dict_ref(mf, pmt::mp("input_sample_format"),
                                            pmt::mp("?"))),
        "fc32");
}
