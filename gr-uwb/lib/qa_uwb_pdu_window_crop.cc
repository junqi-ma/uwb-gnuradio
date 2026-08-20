/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * QA for UwbPduWindowCrop and plan_window_crop.
 *
 * Scheduler under test: message handler only.  Crop is in the input
 * sample-rate domain (SC16 or CF32); CaptureOnly on the 65/48 block is
 * not used.
 */

#include <boost/test/unit_test.hpp>

#include <gnuradio/blocks/message_debug.h>
#include <gnuradio/top_block.h>
#include <gnuradio/uwb/uwb_defaults.h>
#include <gnuradio/uwb/uwb_pdu_window_crop.h>
#include <gnuradio/uwb/uwb_window_crop_core.h>
#include <pmt/pmt.h>

#include <cstdint>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

using gr::uwb::UwbPduWindowCrop;
using gr::uwb::core::plan_window_crop;
using gr::uwb::core::WindowCropGeom;
using gr::uwb::defaults::kNativeInterferencePostGuard;
using gr::uwb::defaults::kNativeInterferencePreGuard;
using gr::uwb::defaults::kNativeScheduledCapture;
using gr::uwb::defaults::kNativeScheduledPostGuard;
using gr::uwb::defaults::kNativeScheduledPreGuard;

namespace {

constexpr double kInRate = 737.28e6;

int64_t dict_i64(pmt::pmt_t dict, const char* key, int64_t def)
{
    pmt::pmt_t v = pmt::dict_ref(dict, pmt::mp(key), pmt::from_long(def));
    if (pmt::is_uint64(v))
        return static_cast<int64_t>(pmt::to_uint64(v));
    if (pmt::is_integer(v))
        return static_cast<int64_t>(pmt::to_long(v));
    return def;
}

std::string dict_str(pmt::pmt_t dict, const char* key)
{
    pmt::pmt_t v = pmt::dict_ref(dict, pmt::mp(key), pmt::PMT_NIL);
    if (pmt::is_symbol(v))
        return pmt::symbol_to_string(v);
    return {};
}

pmt::pmt_t make_sc16_pdu(const std::vector<int16_t>& iq,
                         int64_t window_start,
                         int64_t pre,
                         int64_t capture,
                         int64_t post,
                         int64_t predicted,
                         const char* mode,
                         uint64_t packet_id = 1)
{
    BOOST_REQUIRE_EQUAL(iq.size() % 2, 0u);
    pmt::pmt_t meta = pmt::make_dict();
    meta = pmt::dict_add(meta, pmt::mp("packet_id"),
                         pmt::from_uint64(packet_id));
    meta = pmt::dict_add(meta, pmt::mp("schedule_index"),
                         pmt::from_uint64(packet_id));
    meta = pmt::dict_add(meta, pmt::mp("window_start_sample"),
                         pmt::from_long(window_start));
    meta = pmt::dict_add(meta, pmt::mp("pre_guard_samples"),
                         pmt::from_long(pre));
    meta = pmt::dict_add(meta, pmt::mp("capture_samples"),
                         pmt::from_long(capture));
    meta = pmt::dict_add(meta, pmt::mp("post_guard_samples"),
                         pmt::from_long(post));
    meta = pmt::dict_add(meta, pmt::mp("sample_count"),
                         pmt::from_long(static_cast<long>(iq.size() / 2)));
    meta = pmt::dict_add(meta, pmt::mp("sample_rate"),
                         pmt::from_double(kInRate));
    if (predicted >= 0) {
        meta = pmt::dict_add(meta, pmt::mp("predicted_start_sample"),
                             pmt::from_long(predicted));
    }
    meta = pmt::dict_add(meta, pmt::mp("capture_mode"), pmt::mp(mode));
    return pmt::cons(meta, pmt::init_s16vector(iq.size(), iq.data()));
}

pmt::pmt_t make_c32_pdu(const std::vector<gr_complex>& iq,
                        int64_t window_start,
                        int64_t predicted,
                        const char* mode)
{
    pmt::pmt_t meta = pmt::make_dict();
    meta = pmt::dict_add(meta, pmt::mp("window_start_sample"),
                         pmt::from_long(window_start));
    meta = pmt::dict_add(meta, pmt::mp("sample_count"),
                         pmt::from_long(static_cast<long>(iq.size())));
    meta = pmt::dict_add(meta, pmt::mp("predicted_start_sample"),
                         pmt::from_long(predicted));
    meta = pmt::dict_add(meta, pmt::mp("capture_mode"), pmt::mp(mode));
    return pmt::cons(meta, pmt::init_c32vector(iq.size(), iq.data()));
}

pmt::pmt_t run_one(UwbPduWindowCrop::sptr blk,
                   pmt::pmt_t pdu,
                   gr::blocks::message_debug::sptr* status_out = nullptr)
{
    auto dbg = gr::blocks::message_debug::make();
    auto st = gr::blocks::message_debug::make();
    auto tb = gr::make_top_block("qa_pdu_crop");
    tb->msg_connect(blk, "packet", dbg, "store");
    tb->msg_connect(blk, "status", st, "store");
    tb->start();
    blk->_post(pmt::mp("packet"), pdu);
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

std::vector<int16_t> ramp_sc16(size_t n)
{
    std::vector<int16_t> v(n * 2);
    for (size_t i = 0; i < n; ++i) {
        v[2 * i] = static_cast<int16_t>(i & 0x7fff);
        v[2 * i + 1] = static_cast<int16_t>((-static_cast<int>(i)) & 0x7fff);
    }
    return v;
}

} // namespace

BOOST_AUTO_TEST_CASE(test_plan_long_dump_to_demod)
{
    const WindowCropGeom geom{ static_cast<int64_t>(kNativeScheduledPreGuard),
                               static_cast<int64_t>(kNativeScheduledCapture),
                               static_cast<int64_t>(kNativeScheduledPostGuard) };
    const int64_t pred = 1000000;
    const int64_t dump_pre = static_cast<int64_t>(kNativeInterferencePreGuard);
    const int64_t dump_post = static_cast<int64_t>(kNativeInterferencePostGuard);
    const int64_t dump_cap = static_cast<int64_t>(kNativeScheduledCapture);
    const int64_t ws = pred - dump_pre;
    const int64_t n = dump_pre + dump_cap + dump_post;
    const auto plan = plan_window_crop(ws, n, pred, geom, false);
    BOOST_CHECK(!plan.passthrough);
    BOOST_CHECK(!plan.clamped_start);
    BOOST_CHECK_EQUAL(plan.out_count,
                      static_cast<int64_t>(kNativeScheduledPreGuard +
                                           kNativeScheduledCapture +
                                           kNativeScheduledPostGuard));
    BOOST_CHECK_EQUAL(plan.window_start, pred - geom.pre);
    BOOST_CHECK_EQUAL(plan.pre, geom.pre);
    BOOST_CHECK_EQUAL(plan.capture, geom.capture);
    BOOST_CHECK_EQUAL(plan.post, geom.post);
    BOOST_CHECK_EQUAL(plan.in_offset, dump_pre - geom.pre);
}

BOOST_AUTO_TEST_CASE(test_plan_clamp_to_zero)
{
    const WindowCropGeom geom{ 7373, 140083, 3023 };
    const int64_t pred = 1000;
    const auto plan = plan_window_crop(0, 200000, pred, geom, false);
    BOOST_CHECK(plan.clamped_start);
    BOOST_CHECK_EQUAL(plan.window_start, 0);
    BOOST_CHECK_EQUAL(plan.pre, pred);
    BOOST_CHECK_EQUAL(plan.capture, geom.capture);
    BOOST_CHECK_EQUAL(std::string(plan.reason), "clamped");
}

BOOST_AUTO_TEST_CASE(test_plan_acquisition_passthrough)
{
    const WindowCropGeom geom{ 7373, 140083, 3023 };
    const auto plan = plan_window_crop(10, 202032, 2032, geom, true);
    BOOST_CHECK(plan.passthrough);
    BOOST_CHECK_EQUAL(plan.out_count, 202032);
    BOOST_CHECK_EQUAL(plan.window_start, 10);
    BOOST_CHECK_EQUAL(std::string(plan.reason), "acquisition_passthrough");
}

BOOST_AUTO_TEST_CASE(test_crop_long_sc16_bit_exact)
{
    const int64_t dump_pre = static_cast<int64_t>(kNativeInterferencePreGuard);
    const int64_t dump_cap = static_cast<int64_t>(kNativeScheduledCapture);
    const int64_t dump_post = static_cast<int64_t>(kNativeInterferencePostGuard);
    const int64_t n = dump_pre + dump_cap + dump_post;
    const int64_t pred = 500000;
    const int64_t ws = pred - dump_pre;
    auto iq = ramp_sc16(static_cast<size_t>(n));
    auto blk = UwbPduWindowCrop::make();
    auto pdu = make_sc16_pdu(iq, ws, dump_pre, dump_cap, dump_post, pred,
                             "scheduled");
    auto out = run_one(blk, pdu);
    BOOST_REQUIRE(pmt::is_pair(out));
    pmt::pmt_t meta = pmt::car(out);
    pmt::pmt_t data = pmt::cdr(out);
    BOOST_CHECK(pmt::is_s16vector(data));
    const int64_t expect_n = static_cast<int64_t>(
        kNativeScheduledPreGuard + kNativeScheduledCapture +
        kNativeScheduledPostGuard);
    BOOST_CHECK_EQUAL(dict_i64(meta, "sample_count", -1), expect_n);
    BOOST_CHECK_EQUAL(dict_i64(meta, "window_start_sample", -1),
                      pred - static_cast<int64_t>(kNativeScheduledPreGuard));
    BOOST_CHECK_EQUAL(dict_i64(meta, "predicted_start_sample", -1), pred);
    BOOST_CHECK_EQUAL(dict_i64(meta, "pre_guard_samples", -1),
                      static_cast<int64_t>(kNativeScheduledPreGuard));
    BOOST_CHECK_EQUAL(dict_i64(meta, "capture_samples", -1),
                      static_cast<int64_t>(kNativeScheduledCapture));
    BOOST_CHECK_EQUAL(dict_i64(meta, "post_guard_samples", -1),
                      static_cast<int64_t>(kNativeScheduledPostGuard));
    BOOST_CHECK_EQUAL(dict_i64(meta, "packet_id", -1), 1);
    size_t n_elem = 0;
    const int16_t* out_s = pmt::s16vector_elements(data, n_elem);
    BOOST_REQUIRE_EQUAL(n_elem, static_cast<size_t>(expect_n) * 2);
    const size_t off =
        static_cast<size_t>(dump_pre - kNativeScheduledPreGuard);
    BOOST_CHECK_EQUAL(0, std::memcmp(out_s, iq.data() + off * 2,
                                     n_elem * sizeof(int16_t)));
    BOOST_CHECK_EQUAL(blk->pdus_cropped(), 1u);
    BOOST_CHECK_EQUAL(blk->pdus_passthrough(), 0u);
    BOOST_CHECK_EQUAL(blk->total_input_samples(), static_cast<uint64_t>(n));
    BOOST_CHECK_EQUAL(blk->total_output_samples(),
                      static_cast<uint64_t>(expect_n));
}

BOOST_AUTO_TEST_CASE(test_crop_already_short_is_identity)
{
    const int64_t pre = static_cast<int64_t>(kNativeScheduledPreGuard);
    const int64_t cap = static_cast<int64_t>(kNativeScheduledCapture);
    const int64_t post = static_cast<int64_t>(kNativeScheduledPostGuard);
    const int64_t n = pre + cap + post;
    const int64_t pred = 80000;
    const int64_t ws = pred - pre;
    auto iq = ramp_sc16(static_cast<size_t>(n));
    auto blk = UwbPduWindowCrop::make();
    auto out = run_one(blk, make_sc16_pdu(iq, ws, pre, cap, post, pred,
                                          "scheduled"));
    BOOST_REQUIRE(pmt::is_pair(out));
    BOOST_CHECK_EQUAL(dict_i64(pmt::car(out), "sample_count", -1), n);
    BOOST_CHECK_EQUAL(dict_i64(pmt::car(out), "window_start_sample", -1), ws);
    size_t n_elem = 0;
    const int16_t* s = pmt::s16vector_elements(pmt::cdr(out), n_elem);
    BOOST_CHECK_EQUAL(0, std::memcmp(s, iq.data(), iq.size() * sizeof(int16_t)));
}

BOOST_AUTO_TEST_CASE(test_crop_clamp_and_status)
{
    const int64_t pred = 1000;
    auto iq = ramp_sc16(200000);
    auto blk = UwbPduWindowCrop::make();
    gr::blocks::message_debug::sptr st;
    auto out = run_one(blk, make_sc16_pdu(iq, 0, 0, 200000, 0, pred,
                                          "scheduled"),
                       &st);
    BOOST_REQUIRE(pmt::is_pair(out));
    pmt::pmt_t meta = pmt::car(out);
    BOOST_CHECK_EQUAL(dict_i64(meta, "window_start_sample", -1), 0);
    BOOST_CHECK_EQUAL(dict_i64(meta, "pre_guard_samples", -1), pred);
    BOOST_CHECK_EQUAL(dict_i64(meta, "predicted_start_sample", -1), pred);
    BOOST_CHECK(status_has(st, "clamped"));
    BOOST_CHECK_EQUAL(blk->pdus_clamped(), 1u);
}

BOOST_AUTO_TEST_CASE(test_acquisition_not_cropped_to_dump)
{
    auto iq = ramp_sc16(202032);
    auto blk = UwbPduWindowCrop::make();
    gr::blocks::message_debug::sptr st;
    auto out = run_one(blk, make_sc16_pdu(iq, 100, 2032, 200000, 0, 2132,
                                          "acquisition"),
                       &st);
    BOOST_REQUIRE(pmt::is_pair(out));
    BOOST_CHECK_EQUAL(dict_i64(pmt::car(out), "sample_count", -1), 202032);
    BOOST_CHECK_EQUAL(dict_str(pmt::car(out), "capture_mode"), "acquisition");
    BOOST_CHECK_EQUAL(blk->pdus_passthrough(), 1u);
    BOOST_CHECK_EQUAL(blk->pdus_cropped(), 0u);
    BOOST_CHECK(status_has(st, "passthrough"));
}

BOOST_AUTO_TEST_CASE(test_missing_predicted_passthrough)
{
    auto iq = ramp_sc16(1000);
    auto blk = UwbPduWindowCrop::make();
    auto out = run_one(blk, make_sc16_pdu(iq, 0, 0, 1000, 0, -1, "scheduled"));
    BOOST_REQUIRE(pmt::is_pair(out));
    BOOST_CHECK_EQUAL(pmt::length(pmt::cdr(out)), 2000u);
    BOOST_CHECK_EQUAL(blk->pdus_passthrough(), 1u);
}

BOOST_AUTO_TEST_CASE(test_cf32_crop)
{
    const int64_t n = 10000;
    const int64_t pred = 5000;
    const int64_t ws = 0;
    std::vector<gr_complex> iq(static_cast<size_t>(n));
    for (int64_t i = 0; i < n; ++i)
        iq[static_cast<size_t>(i)] = gr_complex(static_cast<float>(i), -1.f);
    auto blk = UwbPduWindowCrop::make(100, 200, 50);
    auto out = run_one(blk, make_c32_pdu(iq, ws, pred, "scheduled"));
    BOOST_REQUIRE(pmt::is_pair(out));
    BOOST_CHECK(pmt::is_c32vector(pmt::cdr(out)));
    BOOST_CHECK_EQUAL(dict_i64(pmt::car(out), "sample_count", -1), 350);
    BOOST_CHECK_EQUAL(dict_i64(pmt::car(out), "window_start_sample", -1),
                      pred - 100);
    size_t n_elem = 0;
    const gr_complex* c = pmt::c32vector_elements(pmt::cdr(out), n_elem);
    BOOST_REQUIRE_EQUAL(n_elem, 350u);
    BOOST_CHECK_EQUAL(c[0].real(), 4900.f);
}

BOOST_AUTO_TEST_CASE(test_invalid_input_dropped)
{
    auto blk = UwbPduWindowCrop::make();
    auto tb = gr::make_top_block("qa_crop_bad");
    auto st = gr::blocks::message_debug::make();
    tb->msg_connect(blk, "status", st, "store");
    tb->start();
    blk->_post(pmt::mp("packet"), pmt::mp("not-a-pdu"));
    for (int i = 0; i < 50 && blk->pdus_dropped() == 0; ++i)
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    tb->stop();
    tb->wait();
    BOOST_CHECK_EQUAL(blk->pdus_dropped(), 1u);
    BOOST_CHECK(status_has(st, "invalid_input"));
}
