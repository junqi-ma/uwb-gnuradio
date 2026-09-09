/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * QA for UwbPduRationalResamplerCcf65_32 (491.52 -> 998.4).
 */

#include <boost/test/unit_test.hpp>

#include <gnuradio/blocks/message_debug.h>
#include <gnuradio/top_block.h>
#include <gnuradio/uwb/uwb_pdu_rational_resampler_ccf_65_32.h>
#include <pmt/pmt.h>

#include <chrono>
#include <cmath>
#include <complex>
#include <fstream>
#include <string>
#include <thread>
#include <vector>

using gr::uwb::UwbPduRationalResamplerCcf65_32;
using gr::uwb::core::RationalResampler65_32Core;
using gr_complex = std::complex<float>;

namespace {

constexpr float kTolAbs = 2e-3f;
constexpr double kInRate = 491.52e6;
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

const std::vector<float>& taps()
{
    static std::vector<float> t = load_f32(find_path(
        "testdata/resampler_65_32/taps_quality_minorder.txt"));
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
    meta = pmt::dict_add(meta, pmt::mp("window_start_sample"),
                         pmt::from_long(ws));
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

pmt::pmt_t run_one_pdu(UwbPduRationalResamplerCcf65_32::sptr blk, pmt::pmt_t pdu)
{
    auto dbg = gr::blocks::message_debug::make();
    auto st = gr::blocks::message_debug::make();
    auto tb = gr::make_top_block("qa_pdu_65_32");
    tb->msg_connect(blk, "packet", dbg, "store");
    tb->msg_connect(blk, "status", st, "store");
    tb->start();
    blk->_post(pmt::mp("packet"), pdu);
    for (int i = 0; i < 200 && dbg->num_messages() + st->num_messages() == 0;
         ++i)
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    tb->stop();
    tb->wait();
    if (dbg->num_messages() > 0)
        return dbg->get_message(0);
    return pmt::PMT_NIL;
}

float max_abs_diff(const std::vector<gr_complex>& a,
                   const std::vector<gr_complex>& b)
{
    BOOST_REQUIRE_EQUAL(a.size(), b.size());
    float m = 0.f;
    for (size_t i = 0; i < a.size(); ++i) {
        const float e = std::abs(a[i] - b[i]);
        if (e > m)
            m = e;
    }
    return m;
}

} // namespace

BOOST_AUTO_TEST_CASE(test_65_32_constants)
{
    BOOST_CHECK_EQUAL(UwbPduRationalResamplerCcf65_32::kInterp, 65u);
    BOOST_CHECK_EQUAL(UwbPduRationalResamplerCcf65_32::kDecim, 32u);
    BOOST_CHECK_CLOSE(UwbPduRationalResamplerCcf65_32::kInputRateHz, kInRate,
                      1e-12);
    BOOST_CHECK_CLOSE(UwbPduRationalResamplerCcf65_32::kOutputRateHz, kOutRate,
                      1e-12);
}

BOOST_AUTO_TEST_CASE(test_65_32_pdu_matches_golden)
{
    const auto xin = load_cf32(find_path(
        "testdata/resampler_65_32/golden_quality_minorder/impulse_in.cf32"));
    const auto ygold = load_cf32(find_path(
        "testdata/resampler_65_32/golden_quality_minorder/impulse_out.cf32"));
    const int64_t pre = 512;
    const int64_t post = 256;
    const int64_t cap = static_cast<int64_t>(xin.size()) - pre - post;
    auto blk = UwbPduRationalResamplerCcf65_32::make_from_taps(taps());
    auto out = run_one_pdu(blk, make_window_pdu(xin, 0, pre, cap, post, kInRate));
    BOOST_REQUIRE(pmt::is_pair(out));
    size_t n = 0;
    const gr_complex* yp = pmt::c32vector_elements(pmt::cdr(out), n);
    std::vector<gr_complex> y(yp, yp + n);
    BOOST_CHECK_LT(max_abs_diff(y, ygold), kTolAbs);
    BOOST_CHECK_EQUAL(blk->pdus_emitted(), 1u);
    BOOST_CHECK_EQUAL(blk->pdus_dropped(), 0u);
    BOOST_CHECK_CLOSE(
        pmt::to_double(pmt::dict_ref(pmt::car(out), pmt::mp("sample_rate"),
                                     pmt::from_double(0))),
        kOutRate, 1e-9);
    BOOST_CHECK_EQUAL(
        pmt::to_long(pmt::dict_ref(pmt::car(out), pmt::mp("resample_interp"),
                                   pmt::from_long(0))),
        65);
    BOOST_CHECK_EQUAL(
        pmt::to_long(pmt::dict_ref(pmt::car(out), pmt::mp("resample_decim"),
                                   pmt::from_long(0))),
        32);
}

BOOST_AUTO_TEST_CASE(test_65_32_map_formula)
{
    auto blk = UwbPduRationalResamplerCcf65_32::make_from_taps(taps());
    const double d = 0.5 * static_cast<double>(taps().size() - 1);
    for (int64_t p : { 0, 1, 31, 32, 33, 983, 32012 }) {
        const int64_t got = blk->map_input_offset_to_output(p);
        const int64_t expect = static_cast<int64_t>(
            std::llround((static_cast<double>(p) * 65.0 + d) / 32.0));
        BOOST_CHECK_EQUAL(got, expect);
    }
}

BOOST_AUTO_TEST_CASE(test_65_32_rejects_737p28)
{
    std::vector<gr_complex> x(1024, gr_complex(1.f, 0.f));
    auto blk = UwbPduRationalResamplerCcf65_32::make_from_taps(taps());
    auto st = gr::blocks::message_debug::make();
    auto dbg = gr::blocks::message_debug::make();
    auto tb = gr::make_top_block("qa_pdu_65_32_bad_rate");
    tb->msg_connect(blk, "status", st, "store");
    tb->msg_connect(blk, "packet", dbg, "store");
    tb->start();
    blk->_post(pmt::mp("packet"),
               make_window_pdu(x, 0, 10, 1000, 14, 737.28e6));
    for (int i = 0; i < 200 && st->num_messages() == 0; ++i)
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    tb->stop();
    tb->wait();
    BOOST_CHECK_EQUAL(dbg->num_messages(), 0);
    BOOST_REQUIRE_GE(st->num_messages(), 1);
    BOOST_CHECK_EQUAL(
        pmt::symbol_to_string(pmt::dict_ref(
            st->get_message(0), pmt::mp("event"), pmt::PMT_NIL)),
        "bad_input_rate");
    BOOST_CHECK_EQUAL(blk->pdus_dropped(), 1u);
}
