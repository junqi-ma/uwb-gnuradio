/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * QA for UwbPduRationalResamplerCcf65_48 (PDU-level 65/48).
 *
 * 1. Full-window PDU resample == upfirdn / golden quality_minorder
 * 2. Coordinate mapping formulas
 * 3. CaptureOnly crop
 * 4. bad_input_rate drop + status
 * 5. Short guards clamp + short_guard status
 * 6. e2e: scheduled extractor @737.28 → PDU resampler → realtime demod FCS
 * 7. Throughput sanity vs slot rate
 */

#include <boost/test/unit_test.hpp>

#include <gnuradio/blocks/message_debug.h>
#include <gnuradio/blocks/vector_source.h>
#include <gnuradio/top_block.h>
#include <gnuradio/uwb/uwb_pdu_rational_resampler_ccf_65_48.h>
#include <gnuradio/uwb/uwb_rational_resampler_core.h>
#include <gnuradio/uwb/uwb_realtime_demodulator.h>
#include <gnuradio/uwb/uwb_scheduled_extractor.h>
#include <pmt/pmt.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

using gr::uwb::UwbPduRationalResamplerCcf65_48;
using gr::uwb::core::RationalResampler65_48Core;
using gr_complex = std::complex<float>;

namespace {

constexpr float kTolAbs = 2e-3f;
constexpr double kInRate = 737.28e6;
constexpr double kOutRate = 998.4e6;

std::string find_path(const std::string& rel)
{
    const char* prefixes[] = {
        "", "../", "../../", "../../../", "../../../../",
    };
    for (const char* p : prefixes) {
        const std::string path = std::string(p) + rel;
        std::ifstream f(path, std::ios::binary);
        if (f)
            return path;
    }
    return rel;
}

std::vector<float> load_f32(const std::string& path)
{
    std::ifstream f(path, std::ios::binary | std::ios::ate);
    BOOST_REQUIRE_MESSAGE(f, "cannot open " + path);
    const auto bytes = static_cast<size_t>(f.tellg());
    f.seekg(0);
    std::vector<float> v(bytes / sizeof(float));
    f.read(reinterpret_cast<char*>(v.data()),
           static_cast<std::streamsize>(bytes));
    return v;
}

std::vector<gr_complex> load_cf32(const std::string& path)
{
    auto f = load_f32(path);
    std::vector<gr_complex> v(f.size() / 2);
    for (size_t i = 0; i < v.size(); ++i)
        v[i] = gr_complex(f[2 * i], f[2 * i + 1]);
    return v;
}

const std::vector<float>& quality_minorder_taps()
{
    static std::vector<float> taps = load_f32(find_path(
        "testdata/resampler_65_48/taps_quality_minorder.txt"));
    return taps;
}

std::vector<gr_complex> core_oneshot(const std::vector<gr_complex>& x,
                                     const std::vector<float>& taps)
{
    RationalResampler65_48Core core(taps);
    const size_t Lout =
        RationalResampler65_48Core::expected_output_length(x.size(),
                                                           taps.size());
    std::vector<gr_complex> y(Lout + 128);
    size_t produced = 0;
    if (!x.empty()) {
        auto r = core.process(x.data(), x.size(), y.data(), y.size());
        produced = r.produced;
    }
    while (true) {
        if (produced >= y.size())
            y.resize(produced + 256);
        size_t n = core.flush(y.data() + produced, y.size() - produced);
        if (n == 0)
            break;
        produced += n;
    }
    y.resize(produced);
    return y;
}

float max_abs_diff(const std::vector<gr_complex>& a,
                   const std::vector<gr_complex>& b)
{
    BOOST_REQUIRE_EQUAL(a.size(), b.size());
    float m = 0.0f;
    for (size_t i = 0; i < a.size(); ++i)
        m = std::max(m, std::abs(a[i] - b[i]));
    return m;
}

pmt::pmt_t make_window_pdu(const std::vector<gr_complex>& iq,
                           int64_t window_start,
                           int64_t pre_guard,
                           int64_t capture,
                           int64_t post_guard,
                           int64_t predicted_start,
                           double sample_rate = kInRate,
                           uint64_t packet_id = 1,
                           int64_t detected_start = -1)
{
    pmt::pmt_t meta = pmt::make_dict();
    meta = pmt::dict_add(meta, pmt::mp("packet_id"),
                         pmt::from_uint64(packet_id));
    meta = pmt::dict_add(meta, pmt::mp("schedule_index"),
                         pmt::from_uint64(packet_id));
    meta = pmt::dict_add(meta, pmt::mp("schedule_generation"),
                         pmt::from_uint64(0));
    meta = pmt::dict_add(meta, pmt::mp("window_start_sample"),
                         pmt::from_long(window_start));
    meta = pmt::dict_add(meta, pmt::mp("pre_guard_samples"),
                         pmt::from_long(pre_guard));
    meta = pmt::dict_add(meta, pmt::mp("capture_samples"),
                         pmt::from_long(capture));
    meta = pmt::dict_add(meta, pmt::mp("post_guard_samples"),
                         pmt::from_long(post_guard));
    meta = pmt::dict_add(meta, pmt::mp("sample_count"),
                         pmt::from_long(static_cast<long>(iq.size())));
    meta = pmt::dict_add(meta, pmt::mp("sample_rate"),
                         pmt::from_double(sample_rate));
    meta = pmt::dict_add(meta, pmt::mp("predicted_start_sample"),
                         pmt::from_long(predicted_start));
    if (detected_start >= 0) {
        meta = pmt::dict_add(meta, pmt::mp("detected_start_sample"),
                             pmt::from_long(detected_start));
    }
    return pmt::cons(meta, pmt::init_c32vector(iq.size(), iq.data()));
}

pmt::pmt_t make_window_pdu_sc16(const std::vector<int16_t>& iq,
                                int64_t window_start,
                                int64_t pre_guard,
                                int64_t capture,
                                int64_t post_guard,
                                int64_t predicted_start)
{
    BOOST_REQUIRE_EQUAL(iq.size() % 2, 0u);
    pmt::pmt_t meta = pmt::make_dict();
    meta = pmt::dict_add(meta, pmt::mp("window_start_sample"),
                         pmt::from_long(window_start));
    meta = pmt::dict_add(meta, pmt::mp("pre_guard_samples"),
                         pmt::from_long(pre_guard));
    meta = pmt::dict_add(meta, pmt::mp("capture_samples"),
                         pmt::from_long(capture));
    meta = pmt::dict_add(meta, pmt::mp("post_guard_samples"),
                         pmt::from_long(post_guard));
    meta = pmt::dict_add(meta, pmt::mp("sample_count"),
                         pmt::from_long(static_cast<long>(iq.size() / 2)));
    meta = pmt::dict_add(meta, pmt::mp("sample_rate"), pmt::from_double(kInRate));
    meta = pmt::dict_add(meta, pmt::mp("predicted_start_sample"),
                         pmt::from_long(predicted_start));
    return pmt::cons(meta, pmt::init_s16vector(iq.size(), iq.data()));
}

/** Run one PDU through the block in a real top_block; return emitted packet. */
pmt::pmt_t run_one_pdu(UwbPduRationalResamplerCcf65_48::sptr blk,
                       pmt::pmt_t pdu,
                       gr::blocks::message_debug::sptr* status_out = nullptr)
{
    auto dbg = gr::blocks::message_debug::make();
    auto st = gr::blocks::message_debug::make();
    auto tb = gr::make_top_block("qa_pdu_resamp");
    tb->msg_connect(blk, "packet", dbg, "store");
    tb->msg_connect(blk, "status", st, "store");
    tb->start();
    blk->_post(pmt::mp("packet"), pdu);
    // Synchronous handler: result is available immediately after _post.
    for (int i = 0; i < 200 && dbg->num_messages() == 0 &&
                    blk->pdus_dropped() == 0; ++i)
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    tb->stop();
    tb->wait();
    if (status_out)
        *status_out = st;
    if (dbg->num_messages() == 0)
        return pmt::PMT_NIL;
    return dbg->get_message(0);
}

bool status_has(gr::blocks::message_debug::sptr dbg, const std::string& event)
{
    if (!dbg)
        return false;
    for (size_t i = 0; i < dbg->num_messages(); ++i) {
        pmt::pmt_t st = dbg->get_message(i);
        if (pmt::is_dict(st) &&
            pmt::eqv(pmt::dict_ref(st, pmt::mp("event"), pmt::PMT_NIL),
                     pmt::mp(event)))
            return true;
    }
    return false;
}

/**
 * Test-local 48/65 anti-alias decimator (upfirdn with up=48, down=65).
 * Used only to build a 737.28 MS/s stream from the 998.4 cfile for e2e.
 * Phase law matches scipy: y[m] = sum_k h[(m*65 mod 48) + 48k] * x[floor(m*65/48)-k]
 * with zero padding — same as RationalResampler but L=48,M=65.
 */
std::vector<gr_complex> upfirdn_48_65(const std::vector<float>& h,
                                      const std::vector<gr_complex>& x)
{
    const size_t T = h.size();
    const size_t N = x.size();
    if (T == 0 || N == 0)
        return {};
    const int64_t L = 48;
    const int64_t M = 65;
    const int64_t num =
        (static_cast<int64_t>(N) - 1) * L + static_cast<int64_t>(T);
    const size_t Lout =
        static_cast<size_t>((num + M - 1) / M);
    std::vector<gr_complex> y(Lout, gr_complex(0.f, 0.f));
    for (size_t m = 0; m < Lout; ++m) {
        const int64_t mm = static_cast<int64_t>(m);
        const int64_t arm = (mm * M) % L;
        const int64_t x0 = (mm * M) / L;
        gr_complex acc(0.f, 0.f);
        for (size_t k = 0;; ++k) {
            const size_t ti = static_cast<size_t>(arm) + static_cast<size_t>(L) * k;
            if (ti >= T)
                break;
            const int64_t xi = x0 - static_cast<int64_t>(k);
            if (xi >= 0 && static_cast<size_t>(xi) < N)
                acc += h[ti] * x[static_cast<size_t>(xi)];
        }
        y[m] = acc;
    }
    return y;
}

} // namespace

// ---------------------------------------------------------------------------
// 1. PDU resample == upfirdn / golden
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_pdu_resample_matches_upfirdn_golden)
{
    const auto& taps = quality_minorder_taps();
    const auto xin = load_cf32(find_path(
        "testdata/resampler_65_48/golden_quality_minorder/uwb_in.cf32"));
    const auto ygold = load_cf32(find_path(
        "testdata/resampler_65_48/golden_quality_minorder/uwb_out.cf32"));
    BOOST_REQUIRE(!xin.empty());
    BOOST_REQUIRE_EQUAL(
        ygold.size(),
        RationalResampler65_48Core::expected_output_length(xin.size(),
                                                           taps.size()));

    // Synthetic guards on the short golden window (pre/capture/post).
    const int64_t pre = 512;
    const int64_t post = 256;
    const int64_t cap = static_cast<int64_t>(xin.size()) - pre - post;
    BOOST_REQUIRE(cap > 0);
    const int64_t ws = 100000;
    const int64_t pred = ws + pre;

    auto blk = UwbPduRationalResamplerCcf65_48::make_from_taps(taps);
    auto pdu = make_window_pdu(xin, ws, pre, cap, post, pred);
    auto out = run_one_pdu(blk, pdu);
    BOOST_REQUIRE(pmt::is_pair(out));

    pmt::pmt_t meta = pmt::car(out);
    pmt::pmt_t vec = pmt::cdr(out);
    BOOST_REQUIRE(pmt::is_c32vector(vec));
    size_t n = 0;
    const gr_complex* yp = pmt::c32vector_elements(vec, n);
    std::vector<gr_complex> y(yp, yp + n);

    const float max_abs = max_abs_diff(y, ygold);
    std::cout << "pdu_upfirdn: N_in=" << xin.size() << " Lout=" << y.size()
              << " max_abs=" << max_abs << " pdus_emitted=" << blk->pdus_emitted()
              << std::endl;
    BOOST_CHECK_EQUAL(y.size(), ygold.size());
    BOOST_CHECK_LT(max_abs, kTolAbs);
    BOOST_CHECK_EQUAL(blk->pdus_received(), 1u);
    BOOST_CHECK_EQUAL(blk->pdus_emitted(), 1u);
    BOOST_CHECK_EQUAL(blk->pdus_dropped(), 0u);

    // Capture-region interior: skip FIR warm-up edge of the capture slice.
    const int64_t pre_out =
        blk->map_input_offset_to_output(ws + pre) -
        blk->map_input_offset_to_output(ws);
    const int64_t cap_out =
        blk->map_input_offset_to_output(ws + pre + cap) -
        blk->map_input_offset_to_output(ws + pre);
    const size_t i0 = static_cast<size_t>(std::max<int64_t>(0, pre_out + 64));
    const size_t i1 = static_cast<size_t>(
        std::min<int64_t>(static_cast<int64_t>(y.size()),
                          pre_out + cap_out - 64));
    BOOST_REQUIRE(i1 > i0);
    float cap_max = 0.f;
    for (size_t i = i0; i < i1; ++i)
        cap_max = std::max(cap_max, std::abs(y[i] - ygold[i]));
    std::cout << "capture_interior max_abs=" << cap_max << " [" << i0 << ","
              << i1 << ")" << std::endl;
    BOOST_CHECK_LT(cap_max, kTolAbs);

    // Cross-check vs core oneshot (same block kernel).
    auto ycore = core_oneshot(xin, taps);
    BOOST_CHECK_LT(max_abs_diff(y, ycore), 1e-5f);
}

// ---------------------------------------------------------------------------
// 2. Coordinate mapping
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_pdu_coordinate_mapping)
{
    const auto& taps = quality_minorder_taps();
    const size_t N = 4096;
    std::vector<gr_complex> iq(N, gr_complex(1.f, 0.f));
    const int64_t ws = 50000;
    const int64_t pre = 1000;
    const int64_t cap = 2000;
    const int64_t post = N - pre - cap;
    const int64_t pred = ws + pre;
    const int64_t det = pred + 3;

    auto blk = UwbPduRationalResamplerCcf65_48::make_from_taps(taps);
    auto pdu = make_window_pdu(iq, ws, pre, cap, post, pred, kInRate, 7, det);
    auto out = run_one_pdu(blk, pdu);
    BOOST_REQUIRE(pmt::is_pair(out));
    pmt::pmt_t meta = pmt::car(out);

    const int64_t expect_ws = blk->map_input_offset_to_output(ws);
    const int64_t expect_pre =
        blk->map_input_offset_to_output(ws + pre) - expect_ws;
    const int64_t expect_cap =
        blk->map_input_offset_to_output(ws + pre + cap) -
        blk->map_input_offset_to_output(ws + pre);
    const size_t Lout =
        RationalResampler65_48Core::expected_output_length(N, taps.size());
    const int64_t expect_post =
        static_cast<int64_t>(Lout) - expect_pre - expect_cap;
    const int64_t expect_pred = blk->map_input_offset_to_output(pred);
    const int64_t expect_det = blk->map_input_offset_to_output(det);

    BOOST_CHECK_EQUAL(
        pmt::to_long(pmt::dict_ref(meta, pmt::mp("window_start_sample"),
                                   pmt::from_long(-1))),
        expect_ws);
    BOOST_CHECK_EQUAL(
        pmt::to_long(pmt::dict_ref(meta, pmt::mp("pre_guard_samples"),
                                   pmt::from_long(-1))),
        expect_pre);
    BOOST_CHECK_EQUAL(
        pmt::to_long(pmt::dict_ref(meta, pmt::mp("capture_samples"),
                                   pmt::from_long(-1))),
        expect_cap);
    BOOST_CHECK_EQUAL(
        pmt::to_long(pmt::dict_ref(meta, pmt::mp("post_guard_samples"),
                                   pmt::from_long(-1))),
        expect_post);
    BOOST_CHECK_EQUAL(
        pmt::to_long(pmt::dict_ref(meta, pmt::mp("sample_count"),
                                   pmt::from_long(-1))),
        static_cast<long>(Lout));
    BOOST_CHECK_EQUAL(
        pmt::to_long(pmt::dict_ref(meta, pmt::mp("predicted_start_sample"),
                                   pmt::from_long(-1))),
        expect_pred);
    BOOST_CHECK_EQUAL(
        pmt::to_long(pmt::dict_ref(meta, pmt::mp("detected_start_sample"),
                                   pmt::from_long(-1))),
        expect_det);
    BOOST_CHECK_CLOSE(
        pmt::to_double(pmt::dict_ref(meta, pmt::mp("sample_rate"),
                                     pmt::from_double(0))),
        kOutRate, 1e-9);
    BOOST_CHECK_EQUAL(
        pmt::to_long(pmt::dict_ref(meta, pmt::mp("resample_interp"),
                                   pmt::from_long(0))),
        65);
    BOOST_CHECK_EQUAL(
        pmt::to_long(pmt::dict_ref(meta, pmt::mp("resample_decim"),
                                   pmt::from_long(0))),
        48);
    // Timing provenance must be present even when a fast host rounds the
    // measured duration down to 0 us for this small QA PDU.
    BOOST_CHECK(pmt::is_uint64(
        pmt::dict_ref(meta, pmt::mp("resample_us"), pmt::PMT_NIL)));

    // Direct formula check for predicted.
    const double d = 0.5 * static_cast<double>(taps.size() - 1);
    const int64_t formula =
        static_cast<int64_t>(std::llround(
            (static_cast<double>(pred) * 65.0 + d) / 48.0));
    BOOST_CHECK_EQUAL(expect_pred, formula);

    std::cout << "coord_map: ws " << ws << "->" << expect_ws
              << " pre/cap/post " << expect_pre << "/" << expect_cap << "/"
              << expect_post << " pred " << pred << "->" << expect_pred
              << " Lout=" << Lout << std::endl;
}

BOOST_AUTO_TEST_CASE(test_pdu_sc16_input_matches_fc32)
{
    const auto& taps = quality_minorder_taps();
    std::vector<int16_t> s16(512 * 2);
    std::vector<gr_complex> c32(512);
    for (size_t i = 0; i < c32.size(); ++i) {
        s16[2 * i] = static_cast<int16_t>((37 * i) % 2000 - 1000);
        s16[2 * i + 1] = static_cast<int16_t>((53 * i) % 2000 - 1000);
        c32[i] = gr_complex(static_cast<float>(s16[2 * i]),
                             static_cast<float>(s16[2 * i + 1]));
    }
    auto a = UwbPduRationalResamplerCcf65_48::make_from_taps(taps);
    auto b = UwbPduRationalResamplerCcf65_48::make_from_taps(taps);
    auto out_c = run_one_pdu(a, make_window_pdu(c32, 1000, 64, 384, 64, 1064));
    auto out_s = run_one_pdu(b, make_window_pdu_sc16(s16, 1000, 64, 384, 64, 1064));
    BOOST_REQUIRE(pmt::is_pair(out_c));
    BOOST_REQUIRE(pmt::is_pair(out_s));
    size_t nc = 0, ns = 0;
    const auto* yc = pmt::c32vector_elements(pmt::cdr(out_c), nc);
    const auto* ys = pmt::c32vector_elements(pmt::cdr(out_s), ns);
    std::vector<gr_complex> vc(yc, yc + nc), vs(ys, ys + ns);
    BOOST_CHECK_LT(max_abs_diff(vc, vs), 1e-5f);
    const auto md = pmt::car(out_s);
    BOOST_CHECK_EQUAL(pmt::symbol_to_string(pmt::dict_ref(
                          md, pmt::mp("input_sample_format"), pmt::PMT_NIL)),
                      "sc16");
    BOOST_CHECK_GT(b->resample_total_us(), 0u);
    BOOST_CHECK_GE(b->handler_total_us(), b->resample_total_us());
    BOOST_CHECK_GT(b->input_convert_total_us(), 0u);
}

// ---------------------------------------------------------------------------
// 3. CaptureOnly crop
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_pdu_capture_only_crop)
{
    const auto& taps = quality_minorder_taps();
    const auto xin = load_cf32(find_path(
        "testdata/resampler_65_48/golden_quality_minorder/uwb_in.cf32"));
    const auto yfull = core_oneshot(xin, taps);

    const int64_t pre = 512;
    const int64_t post = 256;
    const int64_t cap = static_cast<int64_t>(xin.size()) - pre - post;
    const int64_t ws = 2000;
    const int64_t pred = ws + pre;

    auto blk = UwbPduRationalResamplerCcf65_48::make_from_taps(
        taps, kOutRate, true,
        UwbPduRationalResamplerCcf65_48::EmitPolicy::CaptureOnly);
    auto pdu = make_window_pdu(xin, ws, pre, cap, post, pred);
    auto out = run_one_pdu(blk, pdu);
    BOOST_REQUIRE(pmt::is_pair(out));

    pmt::pmt_t meta = pmt::car(out);
    pmt::pmt_t vec = pmt::cdr(out);
    size_t n = 0;
    const gr_complex* yp = pmt::c32vector_elements(vec, n);

    const int64_t pre_out =
        blk->map_input_offset_to_output(ws + pre) -
        blk->map_input_offset_to_output(ws);
    const int64_t cap_out =
        blk->map_input_offset_to_output(ws + pre + cap) -
        blk->map_input_offset_to_output(ws + pre);

    BOOST_CHECK_EQUAL(static_cast<int64_t>(n), cap_out);
    BOOST_CHECK_EQUAL(
        pmt::to_long(pmt::dict_ref(meta, pmt::mp("sample_count"),
                                   pmt::from_long(-1))),
        cap_out);
    BOOST_CHECK_EQUAL(
        pmt::to_long(pmt::dict_ref(meta, pmt::mp("pre_guard_samples"),
                                   pmt::from_long(-1))),
        0);
    BOOST_CHECK_EQUAL(
        pmt::to_long(pmt::dict_ref(meta, pmt::mp("capture_samples"),
                                   pmt::from_long(-1))),
        cap_out);

    // Body equals full-upfirdn slice [pre_out, pre_out+cap_out).
    float max_abs = 0.f;
    for (size_t i = 0; i < n; ++i) {
        const size_t j = static_cast<size_t>(pre_out) + i;
        BOOST_REQUIRE(j < yfull.size());
        max_abs = std::max(max_abs, std::abs(yp[i] - yfull[j]));
    }
    std::cout << "capture_only: n=" << n << " pre_out=" << pre_out
              << " max_abs_vs_slice=" << max_abs << std::endl;
    BOOST_CHECK_LT(max_abs, kTolAbs);
}

// ---------------------------------------------------------------------------
// 4. bad input rate
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_pdu_bad_input_rate)
{
    const auto& taps = quality_minorder_taps();
    std::vector<gr_complex> iq(1024, gr_complex(0.5f, 0.f));
    auto blk = UwbPduRationalResamplerCcf65_48::make_from_taps(
        taps, kOutRate, /*validate=*/true);
    auto pdu =
        make_window_pdu(iq, 0, 100, 800, 124, 100, /*rate=*/998.4e6);
    gr::blocks::message_debug::sptr st;
    auto out = run_one_pdu(blk, pdu, &st);
    BOOST_CHECK(pmt::is_null(out) || !pmt::is_pair(out) ||
                pmt::length(pmt::cdr(out)) == 0 ||
                blk->pdus_emitted() == 0);
    BOOST_CHECK_EQUAL(blk->pdus_emitted(), 0u);
    BOOST_CHECK_EQUAL(blk->pdus_dropped(), 1u);
    BOOST_CHECK(status_has(st, "bad_input_rate"));
    std::cout << "bad_input_rate: dropped=" << blk->pdus_dropped()
              << " status_ok=" << status_has(st, "bad_input_rate") << std::endl;
}

// ---------------------------------------------------------------------------
// 5. Short guards
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_pdu_short_guards)
{
    const auto& taps = quality_minorder_taps();
    // Tiny window + tiny guards → still clamp, no OOB, short_guard once.
    std::vector<gr_complex> iq(64, gr_complex(1.f, 0.25f));
    auto blk = UwbPduRationalResamplerCcf65_48::make_from_taps(taps);
    auto pdu = make_window_pdu(iq, 0, /*pre=*/2, /*cap=*/50, /*post=*/12,
                               /*pred=*/2);
    gr::blocks::message_debug::sptr st;
    auto out = run_one_pdu(blk, pdu, &st);
    BOOST_REQUIRE(pmt::is_pair(out));
    pmt::pmt_t vec = pmt::cdr(out);
    size_t n = 0;
    (void)pmt::c32vector_elements(vec, n);
    BOOST_CHECK_GT(n, 0u);
    BOOST_CHECK_EQUAL(blk->pdus_emitted(), 1u);
    BOOST_CHECK(status_has(st, "short_guard"));
    BOOST_CHECK_GE(blk->short_guard_events(), 1u);

    // Second short PDU should not re-spam status (count still increments).
    auto out2 = run_one_pdu(blk, pdu, &st);
    BOOST_REQUIRE(pmt::is_pair(out2));
    BOOST_CHECK_EQUAL(blk->pdus_emitted(), 2u);
    BOOST_CHECK_GE(blk->short_guard_events(), 2u);
    std::cout << "short_guard: Lout=" << n
              << " events=" << blk->short_guard_events() << std::endl;
}

// ---------------------------------------------------------------------------
// 6. e2e: extractor @737.28 → PDU resampler → realtime demod FCS
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_pdu_e2e_scheduled_resample_demod)
{
    const auto& up_taps = quality_minorder_taps();
    const auto dec_taps_raw = load_f32(
        find_path("testdata/resampler_65_48/taps_realtime.txt"));
    std::vector<float> dec_taps(dec_taps_raw.size());
    for (size_t i = 0; i < dec_taps.size(); ++i)
        dec_taps[i] = dec_taps_raw[i] * (48.0f / 65.0f);

    const auto x998 = load_cf32(find_path(
        "testdata/uwb_code9_preamble64_payload128_standard_sfd.cfile"));
    BOOST_REQUIRE(x998.size() > 5200000u);
    const int64_t ORIG_START = 4992000; // 998.4 domain

    // Decimate a chunk covering pre-packet + packet body.  The cfile only
    // has ~256k samples after ORIG_START; pad silence after so the scheduled
    // window and demod capture geometry fit (same idiom as
    // regress_uwb_resampled_demod.py TRAILING_SILENCE).
    const int64_t chunk0 = std::max<int64_t>(0, ORIG_START - 300000);
    const int64_t chunk1 = static_cast<int64_t>(x998.size());
    BOOST_REQUIRE(chunk1 > chunk0 + 200000);
    std::vector<gr_complex> chunk(
        x998.begin() + chunk0, x998.begin() + chunk1);
    auto x737 = upfirdn_48_65(dec_taps, chunk);
    BOOST_REQUIRE(x737.size() > 100000u);

    // Packet start in the chunked 737 domain (upfirdn map of local origin).
    // Do not energy-search refine: residual energy continues through the
    // rest of the cfile and the map prediction is the correct seed (verified
    // against scipy + demod FCS in the Python twin of this QA).
    const double dd = 0.5 * static_cast<double>(dec_taps.size() - 1);
    const int64_t local998 = ORIG_START - chunk0;
    const int64_t pkt737 = static_cast<int64_t>(std::llround(
        (static_cast<double>(local998) * 48.0 + dd) / 65.0));
    BOOST_REQUIRE(pkt737 > 5000);
    std::cout << "e2e: x737=" << x737.size() << " pkt737=" << pkt737
              << std::endl;

    // Geometry at 737.28: enough for demod after 65/48 (≈9984+309184 out).
    const size_t pre737 = 8000;
    const size_t cap737 = 240000;
    const size_t post737 = 4000;
    BOOST_REQUIRE(pkt737 >= static_cast<int64_t>(pre737));

    // Ensure stream covers the full window + trailing silence for async drain.
    const size_t need =
        static_cast<size_t>(pkt737) + pre737 + cap737 + post737 + 500000;
    std::vector<gr_complex> stream = x737;
    if (stream.size() < need)
        stream.resize(need, gr_complex(0.f, 0.f));
    else
        stream.resize(stream.size() + 500000, gr_complex(0.f, 0.f));

    auto ext = gr::uwb::UwbScheduledExtractor::make(
        kInRate,
        /*packet_interval_s=*/0.05, // only one slot in this stream
        static_cast<uint64_t>(pkt737),
        pre737,
        cap737,
        post737,
        /*pool_size=*/4,
        gr::uwb::UwbScheduledExtractor::EmitPolicy::EverySlot,
        /*verification=*/false);

    auto resamp = UwbPduRationalResamplerCcf65_48::make_from_taps(
        up_taps, kOutRate, true,
        UwbPduRationalResamplerCcf65_48::EmitPolicy::FullWindow);

    auto tmpl = load_cf32(find_path("testdata/reference_preamble.bin"));

    // Stage A: extractor @737.28 → PDU resampler (real stream flowgraph).
    auto dbg_pkt = gr::blocks::message_debug::make();
    auto src = gr::blocks::vector_source_c::make(stream);
    auto tb = gr::make_top_block("qa_pdu_e2e_capture");
    tb->connect(src, 0, ext, 0);
    tb->msg_connect(ext, "packet", resamp, "packet");
    tb->msg_connect(resamp, "packet", dbg_pkt, "store");

    const auto t0 = std::chrono::steady_clock::now();
    tb->run();
    const auto t_cap = std::chrono::steady_clock::now();

    std::cout << "e2e_capture: ext_emitted=" << ext->emitted_windows()
              << " resamp_emitted=" << resamp->pdus_emitted()
              << " pkts=" << dbg_pkt->num_messages()
              << " wall_s="
              << std::chrono::duration<double>(t_cap - t0).count()
              << std::endl;
    BOOST_REQUIRE_GE(ext->emitted_windows(), 1u);
    BOOST_REQUIRE_GE(resamp->pdus_emitted(), 1u);
    BOOST_REQUIRE_GE(dbg_pkt->num_messages(), 1u);

    pmt::pmt_t resampled_pdu = dbg_pkt->get_message(0);
    BOOST_REQUIRE(pmt::is_pair(resampled_pdu));

    // Stage B: feed resampled PDU into realtime demodulator with the FG kept
    // alive until the async worker publishes (regress_uwb_resampled_demod.py
    // gotcha: results published after stop() are not delivered).
    auto demod = gr::uwb::UwbRealtimeDemodulator::make_from_template(
        tmpl, /*workers=*/2, /*q=*/16, "ieee");
    auto dbg = gr::blocks::message_debug::make();
    auto tb2 = gr::make_top_block("qa_pdu_e2e_demod");
    tb2->msg_connect(demod, "result", dbg, "store");
    tb2->start();
    demod->_post(pmt::mp("samples"), resampled_pdu);
    for (int i = 0; i < 60000; ++i) {
        if (dbg->num_messages() > 0)
            break;
        if (demod->jobs_completed() + demod->jobs_failed() >= 1 &&
            demod->drained()) {
            for (int j = 0; j < 200 && dbg->num_messages() == 0; ++j)
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    const auto t1 = std::chrono::steady_clock::now();
    tb2->stop();
    tb2->wait();
    const double wall_s =
        std::chrono::duration<double>(t1 - t0).count();

    std::cout << "e2e_demod: rx=" << demod->jobs_received()
              << " completed=" << demod->jobs_completed()
              << " failed=" << demod->jobs_failed()
              << " results=" << dbg->num_messages()
              << " wall_s=" << wall_s << std::endl;

    BOOST_REQUIRE_GE(dbg->num_messages(), 1u);

    pmt::pmt_t result = dbg->get_message(0);
    BOOST_REQUIRE(pmt::is_pair(result));
    pmt::pmt_t meta = pmt::car(result);
    BOOST_CHECK(pmt::to_bool(
        pmt::dict_ref(meta, pmt::mp("fcs_pass"), pmt::PMT_F)));
    BOOST_CHECK(pmt::eqv(
        pmt::dict_ref(meta, pmt::mp("status"), pmt::PMT_NIL),
        pmt::mp("success")));

    const int64_t det = pmt::to_long(
        pmt::dict_ref(meta, pmt::mp("detected_start_sample"),
                      pmt::from_long(-1)));
    const int64_t pred_out = resamp->map_input_offset_to_output(pkt737);
    std::cout << "e2e FCS; detected_start=" << det
              << " mapped_predicted~" << pred_out
              << " |det-pred|=" << std::abs(det - pred_out)
              << " fcs="
              << pmt::to_bool(
                     pmt::dict_ref(meta, pmt::mp("fcs_pass"), pmt::PMT_F))
              << std::endl;
    // Group delay + decimation residual; allow generous margin.
    BOOST_CHECK_LT(std::abs(det - pred_out), 5000);

    // Throughput of the capture stage (extractor + resampler).
    const double cap_s =
        std::chrono::duration<double>(t_cap - t0).count();
    const double out_msps =
        cap_s > 0
            ? 1e-6 * static_cast<double>(resamp->total_output_samples()) /
                  cap_s
            : 0;
    const double in_msps =
        cap_s > 0
            ? 1e-6 * static_cast<double>(resamp->total_input_samples()) / cap_s
            : 0;
    std::cout << "e2e_capture_tput: wall=" << cap_s
              << "s  resamp_in_MS/s=" << in_msps
              << "  resamp_out_MS/s=" << out_msps << std::endl;
    BOOST_CHECK_GT(resamp->pdus_emitted(), 0u);
    BOOST_CHECK_GT(resamp->total_output_samples(), 0u);
}

// ---------------------------------------------------------------------------
// 7b. Pure PDU resampler throughput (no demod) — many windows
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_pdu_resampler_throughput_sanity)
{
    // Throughput option = realtime_minorder (1319 taps).  Correctness default
    // remains quality_minorder (covered by tests 1–6).
    const auto taps = load_f32(find_path(
        "testdata/resampler_65_48/taps_realtime_minorder.txt"));
    // Realistic radar window length at 737.28 (scaled production geometry).
    const size_t pre = 8000;
    const size_t cap = 140000;
    const size_t post = 4000;
    const size_t N = pre + cap + post;
    std::vector<gr_complex> iq(N);
    for (size_t i = 0; i < N; ++i) {
        const float ph = 0.01f * static_cast<float>(i);
        iq[i] = gr_complex(std::cos(ph), std::sin(ph));
    }

    auto blk = UwbPduRationalResamplerCcf65_48::make_from_taps(
        taps, kOutRate, true);
    auto dbg = gr::blocks::message_debug::make();
    auto tb = gr::make_top_block("qa_pdu_tput");
    tb->msg_connect(blk, "packet", dbg, "store");
    tb->start();

    const int n_pdus = 80;
    const auto t0 = std::chrono::steady_clock::now();
    for (int k = 0; k < n_pdus; ++k) {
        auto pdu = make_window_pdu(
            iq, /*ws=*/static_cast<int64_t>(k) * 1000000,
            static_cast<int64_t>(pre), static_cast<int64_t>(cap),
            static_cast<int64_t>(post),
            /*pred=*/static_cast<int64_t>(k) * 1000000 +
                static_cast<int64_t>(pre),
            kInRate, static_cast<uint64_t>(k));
        blk->_post(pmt::mp("packet"), pdu);
    }
    // Drain: wait until all emitted (sync handler → immediate).
    for (int i = 0; i < 500 && blk->pdus_emitted() < static_cast<uint64_t>(n_pdus);
         ++i)
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    const auto t1 = std::chrono::steady_clock::now();
    tb->stop();
    tb->wait();

    const double wall =
        std::chrono::duration<double>(t1 - t0).count();
    const double pdus_s =
        wall > 0 ? static_cast<double>(blk->pdus_emitted()) / wall : 0;
    const double out_msps =
        wall > 0
            ? 1e-6 * static_cast<double>(blk->total_output_samples()) / wall
            : 0;
    const double in_msps =
        wall > 0
            ? 1e-6 * static_cast<double>(blk->total_input_samples()) / wall
            : 0;
    // Continuous path best realtime_minorder multi-worker: 0.354× RT
    // (≈261 MS/s full-stream in).  PDU path only pays for window samples.
    const double cont_best_in_msps = 260.7;
    const double vs_cont = in_msps / cont_best_in_msps;
    const double headroom_200 = pdus_s / 200.0;
    const double headroom_1000 = pdus_s / 1000.0;

    std::cout << "pdu_tput(realtime_minorder): n=" << blk->pdus_emitted()
              << " wall=" << wall << "s  pdus/s=" << pdus_s
              << "  in_MS/s=" << in_msps << "  out_MS/s=" << out_msps
              << "  vs_cont_best_in=" << vs_cont << "x"
              << "  headroom_vs_200slot/s=" << headroom_200 << "x"
              << "  headroom_vs_1000slot/s=" << headroom_1000 << "x"
              << std::endl;

    BOOST_REQUIRE_EQUAL(blk->pdus_emitted(), static_cast<uint64_t>(n_pdus));
    // Comfortably above low-end QM35825 slot rate (200 slot/s).
    BOOST_CHECK_GT(pdus_s, 400.0);
    BOOST_CHECK_GT(headroom_200, 2.0);
    // FIR sample throughput should be meaningful (tens of MS/s+).
    BOOST_CHECK_GT(in_msps, 50.0);
}
