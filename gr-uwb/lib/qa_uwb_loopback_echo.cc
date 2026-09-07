/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * QA for UwbLoopbackEcho (software channel, no UHD).
 *
 * Integer/frac goldens: testdata/uwb_radar/rx_delay_{int,frac}_998p4.cf32
 * from export_uwb_radar_golden.m (embed_tx / interp1 pchip).
 */

#include <boost/test/unit_test.hpp>

#include <gnuradio/blocks/message_debug.h>
#include <gnuradio/top_block.h>
#include <gnuradio/uwb/uwb_loopback_echo.h>
#include <gnuradio/uwb/uwb_radar_packet_source.h>
#include <gnuradio/uwb/uwb_radar_pdu_meta.h>
#include <pmt/pmt.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <string>
#include <thread>
#include <vector>

using gr::uwb::UwbLoopbackEcho;
using gr::uwb::UwbRadarPacketSource;
using gr_complex = std::complex<float>;

namespace {

#ifndef UWB_TESTDATA_DIR
#define UWB_TESTDATA_DIR "../../../testdata"
#endif

constexpr size_t kPreGuard = 1997;
constexpr size_t kTail = 4096;
constexpr double kFsWork = 998.4e6;
constexpr double kFsNative = 737.28e6;

std::string testdata_path(const std::string& rel)
{
    const std::string from_def = std::string(UWB_TESTDATA_DIR) + "/" + rel;
    {
        std::ifstream f(from_def, std::ios::binary);
        if (f)
            return from_def;
    }
    const char* prefixes[] = {
        "../../../testdata/",
        "../../testdata/",
        "../testdata/",
        "testdata/",
    };
    for (const char* p : prefixes) {
        const std::string path = std::string(p) + rel;
        std::ifstream f(path, std::ios::binary);
        if (f)
            return path;
    }
    return from_def;
}

bool load_cf32(const std::string& path, std::vector<gr_complex>& out)
{
    std::ifstream f(path, std::ios::binary | std::ios::ate);
    if (!f)
        return false;
    const auto bytes = static_cast<size_t>(f.tellg());
    if (bytes == 0 || bytes % sizeof(gr_complex) != 0)
        return false;
    f.seekg(0);
    out.resize(bytes / sizeof(gr_complex));
    f.read(reinterpret_cast<char*>(out.data()),
           static_cast<std::streamsize>(bytes));
    return static_cast<bool>(f);
}

double dict_f64(pmt::pmt_t dict, const char* key, double def)
{
    return gr::uwb::radar_meta::to_f64(
        pmt::dict_ref(dict, pmt::mp(key), pmt::from_double(def)), def);
}

int64_t dict_i64(pmt::pmt_t dict, const char* key, int64_t def)
{
    return gr::uwb::radar_meta::to_i64(
        pmt::dict_ref(dict, pmt::mp(key), pmt::from_long(def)), def);
}

std::string dict_str(pmt::pmt_t dict, const char* key)
{
    pmt::pmt_t v = pmt::dict_ref(dict, pmt::mp(key), pmt::PMT_NIL);
    if (pmt::is_symbol(v))
        return pmt::symbol_to_string(v);
    return {};
}

pmt::pmt_t make_tx_pdu(const std::vector<gr_complex>& iq,
                       double sample_rate,
                       uint64_t pulse_id = 0,
                       size_t sync_reps = 64,
                       const char* sfd_mode = "4z2",
                       size_t code_index = 9)
{
    pmt::pmt_t meta = pmt::make_dict();
    meta = pmt::dict_add(meta, pmt::mp("pulse_id"),
                         pmt::from_uint64(pulse_id));
    meta = pmt::dict_add(meta, pmt::mp("schedule_index"),
                         pmt::from_uint64(pulse_id));
    meta = pmt::dict_add(meta, pmt::mp("sample_rate"),
                         pmt::from_double(sample_rate));
    meta = pmt::dict_add(meta, pmt::mp("sample_format"), pmt::mp("fc32"));
    meta = pmt::dict_add(meta, pmt::mp("sync_repetitions"),
                         pmt::from_long(static_cast<long>(sync_reps)));
    meta = pmt::dict_add(meta, pmt::mp("sfd_mode"), pmt::mp(sfd_mode));
    meta = pmt::dict_add(meta, pmt::mp("code_index"),
                         pmt::from_long(static_cast<long>(code_index)));
    meta = pmt::dict_add(meta, pmt::mp("tx_packet_samples"),
                         pmt::from_long(static_cast<long>(iq.size())));
    meta = pmt::dict_add(meta, pmt::mp("source"), pmt::mp("packet_source"));
    return pmt::cons(meta, pmt::init_c32vector(iq.size(), iq.data()));
}

pmt::pmt_t run_one(UwbLoopbackEcho::sptr blk,
                   pmt::pmt_t pdu,
                   gr::blocks::message_debug::sptr* status_out = nullptr)
{
    auto dbg = gr::blocks::message_debug::make();
    auto st = gr::blocks::message_debug::make();
    auto tb = gr::make_top_block("qa_loopback_echo");
    tb->msg_connect(blk, "rx", dbg, "store");
    tb->msg_connect(blk, "status", st, "store");
    tb->start();
    blk->_post(pmt::mp("tx"), pdu);
    for (int i = 0; i < 400 && dbg->num_messages() == 0 &&
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

std::vector<gr_complex> pdu_c32(pmt::pmt_t pdu)
{
    BOOST_REQUIRE(pmt::is_pair(pdu));
    pmt::pmt_t data = pmt::cdr(pdu);
    BOOST_REQUIRE(pmt::is_c32vector(data));
    size_t n = 0;
    const gr_complex* p = pmt::c32vector_elements(data, n);
    return std::vector<gr_complex>(p, p + n);
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

double rel_l2(const std::vector<gr_complex>& a, const std::vector<gr_complex>& b)
{
    BOOST_REQUIRE_EQUAL(a.size(), b.size());
    double num = 0.0, den = 0.0;
    for (size_t i = 0; i < a.size(); ++i) {
        const gr_complex d = a[i] - b[i];
        num += static_cast<double>(d.real()) * d.real() +
               static_cast<double>(d.imag()) * d.imag();
        den += static_cast<double>(b[i].real()) * b[i].real() +
               static_cast<double>(b[i].imag()) * b[i].imag();
    }
    if (den == 0.0)
        return num == 0.0 ? 0.0 : 1.0;
    return std::sqrt(num / den);
}

gr_complex polar_gain(double mag, double phase)
{
    return gr_complex(static_cast<float>(mag * std::cos(phase)),
                      static_cast<float>(mag * std::sin(phase)));
}

} // namespace

BOOST_AUTO_TEST_CASE(test_integer_delay_matches_golden)
{
    const std::string tx_path = testdata_path("uwb_radar/tx_998p4.cf32");
    const std::string rx_path = testdata_path("uwb_radar/rx_delay_int_998p4.cf32");
    std::vector<gr_complex> golden;
    BOOST_REQUIRE_MESSAGE(load_cf32(rx_path, golden), "cannot load " + rx_path);

    auto src = UwbRadarPacketSource::make(tx_path, kFsWork);
    auto echo = UwbLoopbackEcho::make(
        kPreGuard, kTail, { 37.0 }, { polar_gain(0.4, 0.7) });

    auto dbg = gr::blocks::message_debug::make();
    auto st = gr::blocks::message_debug::make();
    auto tb = gr::make_top_block("qa_loopback_int");
    tb->msg_connect(src, "tx", echo, "tx");
    tb->msg_connect(echo, "rx", dbg, "store");
    tb->msg_connect(echo, "status", st, "store");
    tb->start();
    src->_post(pmt::mp("emit"), pmt::make_dict());
    for (int i = 0; i < 400 && dbg->num_messages() == 0 &&
                    echo->pdus_dropped() == 0; ++i)
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    tb->stop();
    tb->wait();

    BOOST_REQUIRE_EQUAL(dbg->num_messages(), 1u);
    auto rx = pdu_c32(dbg->get_message(0));
    BOOST_REQUIRE_EQUAL(rx.size(), golden.size());
    BOOST_CHECK_EQUAL(golden.size(), 197005u);
    const float md = max_abs_diff(rx, golden);
    BOOST_CHECK_MESSAGE(md < 1e-5f, "integer delay max abs diff " +
                                        std::to_string(md));

    pmt::pmt_t meta = pmt::car(dbg->get_message(0));
    BOOST_CHECK_EQUAL(dict_str(meta, "source"), std::string("loopback"));
    BOOST_CHECK_EQUAL(dict_str(meta, "uhd_error"), std::string("none"));
    BOOST_CHECK_EQUAL(dict_i64(meta, "pre_guard_samples", -1),
                      static_cast<int64_t>(kPreGuard));
    // Scheduled-capture geometry (PDU 65/48 contract).
    BOOST_CHECK_EQUAL(dict_i64(meta, "window_start_sample", -1), 0);
    BOOST_CHECK_EQUAL(dict_i64(meta, "capture_samples", -1), 190912);
    BOOST_CHECK_EQUAL(dict_i64(meta, "post_guard_samples", -1),
                      static_cast<int64_t>(kTail));
    BOOST_CHECK_EQUAL(dict_i64(meta, "sample_count", -1), 197005);
    BOOST_CHECK_EQUAL(dict_i64(meta, "range_guard_samples", -1),
                      static_cast<int64_t>(kTail));
    BOOST_CHECK_EQUAL(dict_i64(meta, "tx_packet_samples", -1), 190912);
    BOOST_CHECK_EQUAL(dict_i64(meta, "rx_capture_samples", -1), 197005);
    BOOST_CHECK_EQUAL(dict_i64(meta, "sync_samples", -1), 64 * 1016);
    BOOST_CHECK_EQUAL(dict_i64(meta, "sfd_samples", -1), 8 * 1016);
    BOOST_CHECK_CLOSE(dict_f64(meta, "calibration_delay_native_samples", -1),
                      37.0, 1e-6);
    BOOST_CHECK_CLOSE(dict_f64(meta, "sample_rate", 0.0), kFsWork, 1e-4);
    BOOST_CHECK_EQUAL(dict_str(meta, "sample_format"), std::string("fc32"));
    BOOST_CHECK_EQUAL(dict_i64(meta, "sync_repetitions", -1), 64);
    BOOST_CHECK_EQUAL(dict_str(meta, "sfd_mode"), std::string("4z2"));
    BOOST_CHECK_EQUAL(dict_i64(meta, "code_index", -1), 9);
    BOOST_CHECK_EQUAL(dict_i64(meta, "tx_time_full", 99), 0);
    BOOST_CHECK_SMALL(dict_f64(meta, "tx_time_frac", 1.0), 1e-12);
    const double rx_t = static_cast<double>(dict_i64(meta, "rx_time_full", 0)) +
                        dict_f64(meta, "rx_time_frac", 0.0);
    BOOST_CHECK_CLOSE(rx_t, -static_cast<double>(kPreGuard) / kFsWork, 1e-6);
}

BOOST_AUTO_TEST_CASE(test_frac_delay_matches_golden_pchip)
{
    const std::string tx_path = testdata_path("uwb_radar/tx_998p4.cf32");
    const std::string rx_path =
        testdata_path("uwb_radar/rx_delay_frac_998p4.cf32");
    std::vector<gr_complex> golden;
    BOOST_REQUIRE_MESSAGE(load_cf32(rx_path, golden), "cannot load " + rx_path);

    auto src = UwbRadarPacketSource::make(tx_path, kFsWork);
    auto echo = UwbLoopbackEcho::make(
        kPreGuard, kTail, { 12.4 }, { polar_gain(0.55, -1.1) });

    auto dbg = gr::blocks::message_debug::make();
    auto tb = gr::make_top_block("qa_loopback_frac");
    tb->msg_connect(src, "tx", echo, "tx");
    tb->msg_connect(echo, "rx", dbg, "store");
    tb->start();
    src->_post(pmt::mp("emit"), pmt::make_dict());
    for (int i = 0; i < 400 && dbg->num_messages() == 0 &&
                    echo->pdus_dropped() == 0; ++i)
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    tb->stop();
    tb->wait();

    BOOST_REQUIRE_EQUAL(dbg->num_messages(), 1u);
    auto rx = pdu_c32(dbg->get_message(0));
    BOOST_REQUIRE_EQUAL(rx.size(), golden.size());
    const double e = rel_l2(rx, golden);
    BOOST_CHECK_MESSAGE(e < 1e-3, "frac delay relative L2 " + std::to_string(e));
}

BOOST_AUTO_TEST_CASE(test_pchip_unit_grid_shift)
{
    // base = [0,0,1,2,0,0], delay 0.4 → SciPy/MATLAB pchip
    std::vector<gr_complex> tx = { gr_complex(1.f, 0.f), gr_complex(2.f, 0.f) };
    auto echo = UwbLoopbackEcho::make(2, 2, { 0.4 }, { gr_complex(1.f, 0.f) });
    pmt::pmt_t pdu = run_one(echo, make_tx_pdu(tx, kFsWork));
    auto rx = pdu_c32(pdu);
    BOOST_REQUIRE_EQUAL(rx.size(), 6u);
    const float expected[] = { 0.f, 0.f, 0.504f, 1.744f, 0.704f, 0.f };
    for (size_t i = 0; i < 6; ++i)
        BOOST_CHECK_SMALL(std::abs(rx[i].real() - expected[i]), 2e-6f);
}

BOOST_AUTO_TEST_CASE(test_multipath_energy_and_linearity)
{
    std::vector<gr_complex> tx(16, gr_complex(1.f, 0.f));
    const std::vector<double> delays = { 10.0, 25.0, 40.0 };
    const std::vector<gr_complex> gains = { gr_complex(0.8f, 0.1f),
                                            gr_complex(0.5f, -0.2f),
                                            gr_complex(-0.3f, 0.4f) };
    const size_t pre = 0;
    const size_t tail = 64;

    auto sum_blk = UwbLoopbackEcho::make(pre, tail, delays, gains);
    pmt::pmt_t tx_pdu = make_tx_pdu(tx, kFsWork);
    auto sum_pdu = run_one(sum_blk, tx_pdu);
    auto sum = pdu_c32(sum_pdu);

    std::vector<gr_complex> acc(sum.size(), gr_complex(0.f, 0.f));
    for (size_t p = 0; p < delays.size(); ++p) {
        auto one = UwbLoopbackEcho::make(
            pre, tail, { delays[p] }, { gains[p] });
        auto y = pdu_c32(run_one(one, tx_pdu));
        BOOST_REQUIRE_EQUAL(y.size(), acc.size());
        float e = 0.f;
        const size_t loc = static_cast<size_t>(delays[p]);
        for (size_t k = 0; k < tx.size(); ++k)
            e += std::norm(y[loc + k]);
        BOOST_CHECK_MESSAGE(e > 1e-3f,
                            "no energy at delay " + std::to_string(delays[p]));
        for (size_t i = 0; i < acc.size(); ++i)
            acc[i] += y[i];
    }
    BOOST_CHECK_LT(max_abs_diff(sum, acc), 1e-5f);
    BOOST_CHECK_LT(rel_l2(sum, acc), 1e-5);
}

BOOST_AUTO_TEST_CASE(test_fixed_seed_noise_repeatable)
{
    std::vector<gr_complex> tx(64, gr_complex(0.2f, -0.1f));
    pmt::pmt_t pdu = make_tx_pdu(tx, kFsWork);
    auto a = UwbLoopbackEcho::make(8, 8, { 3.0 }, { gr_complex(1.f, 0.f) },
                                   0.05f, 123u);
    auto b = UwbLoopbackEcho::make(8, 8, { 3.0 }, { gr_complex(1.f, 0.f) },
                                   0.05f, 123u);
    auto ya = pdu_c32(run_one(a, pdu));
    auto yb = pdu_c32(run_one(b, pdu));
    BOOST_REQUIRE_EQUAL(ya.size(), yb.size());
    BOOST_REQUIRE_EQUAL(std::memcmp(ya.data(), yb.data(),
                                    ya.size() * sizeof(gr_complex)),
                        0);

    auto c = UwbLoopbackEcho::make(8, 8, { 3.0 }, { gr_complex(1.f, 0.f) },
                                   0.05f, 124u);
    auto yc = pdu_c32(run_one(c, pdu));
    BOOST_CHECK(std::memcmp(ya.data(), yc.data(),
                            ya.size() * sizeof(gr_complex)) != 0);
}

BOOST_AUTO_TEST_CASE(test_invalid_window_tx_longer_than_max_rx)
{
    std::vector<gr_complex> tx(40, gr_complex(1.f, 0.f));
    auto echo = UwbLoopbackEcho::make(
        0, 0, { 0.0 }, { gr_complex(1.f, 0.f) }, 0.f, 1u, 64, 32);
    gr::blocks::message_debug::sptr st;
    pmt::pmt_t pdu = run_one(echo, make_tx_pdu(tx, kFsWork), &st);
    BOOST_CHECK(pmt::is_null(pdu));
    BOOST_REQUIRE_EQUAL(echo->pdus_emitted(), 0u);
    BOOST_REQUIRE(status_has(st, "invalid_window"));
}

BOOST_AUTO_TEST_CASE(test_invalid_window_tx_longer_than_max_tx)
{
    std::vector<gr_complex> tx(80, gr_complex(1.f, 0.f));
    auto echo = UwbLoopbackEcho::make(
        0, 0, { 0.0 }, { gr_complex(1.f, 0.f) }, 0.f, 1u, 32, 256);
    gr::blocks::message_debug::sptr st;
    pmt::pmt_t pdu = run_one(echo, make_tx_pdu(tx, kFsWork), &st);
    BOOST_CHECK(pmt::is_null(pdu));
    BOOST_REQUIRE_EQUAL(echo->pdus_emitted(), 0u);
    BOOST_REQUIRE(status_has(st, "invalid_window"));
}

BOOST_AUTO_TEST_CASE(test_invalid_input_not_pdu)
{
    auto echo = UwbLoopbackEcho::make(0, 0);
    gr::blocks::message_debug::sptr st;
    pmt::pmt_t pdu = run_one(echo, pmt::mp("not_a_pdu"), &st);
    BOOST_CHECK(pmt::is_null(pdu));
    BOOST_REQUIRE_EQUAL(echo->pdus_emitted(), 0u);
    BOOST_REQUIRE(status_has(st, "invalid_input"));
}

BOOST_AUTO_TEST_CASE(test_empty_delay_defaults_to_unity)
{
    std::vector<gr_complex> tx = { gr_complex(0.3f, -0.2f),
                                   gr_complex(1.f, 0.5f) };
    auto echo = UwbLoopbackEcho::make(1, 1);
    auto rx = pdu_c32(run_one(echo, make_tx_pdu(tx, kFsWork)));
    BOOST_REQUIRE_EQUAL(rx.size(), 4u);
    BOOST_CHECK_SMALL(std::abs(rx[0]), 1e-7f);
    BOOST_CHECK_SMALL(std::abs(rx[1] - tx[0]), 1e-7f);
    BOOST_CHECK_SMALL(std::abs(rx[2] - tx[1]), 1e-7f);
    BOOST_CHECK_SMALL(std::abs(rx[3]), 1e-7f);
}

BOOST_AUTO_TEST_CASE(test_invalid_profile_table)
{
    std::vector<gr_complex> tx(16, gr_complex(0.4f, 0.1f));
    auto base = make_tx_pdu(tx, kFsWork);

    struct Row {
        const char* name;
        const char* event;
        pmt::pmt_t (*mut)(pmt::pmt_t);
    };

    auto drop_key = [](pmt::pmt_t meta, const char* key) {
        return pmt::dict_delete(meta, pmt::mp(key));
    };

    const Row rows[] = {
        { "missing_rate", "bad_input_rate",
          [](pmt::pmt_t m) {
              return pmt::dict_delete(m, pmt::mp("sample_rate"));
          } },
        { "wrong_rate", "bad_input_rate",
          [](pmt::pmt_t m) {
              return pmt::dict_add(m, pmt::mp("sample_rate"),
                                   pmt::from_double(1.0e6));
          } },
        { "format_mismatch", "invalid_profile",
          [](pmt::pmt_t m) {
              return pmt::dict_add(m, pmt::mp("sample_format"),
                                   pmt::mp("sc16"));
          } },
        { "sync_zero", "invalid_profile",
          [](pmt::pmt_t m) {
              return pmt::dict_add(m, pmt::mp("sync_repetitions"),
                                   pmt::from_long(0));
          } },
        { "sync_16", "invalid_profile",
          [](pmt::pmt_t m) {
              return pmt::dict_add(m, pmt::mp("sync_repetitions"),
                                   pmt::from_long(16));
          } },
        { "sfd_unknown", "invalid_profile",
          [](pmt::pmt_t m) {
              return pmt::dict_add(m, pmt::mp("sfd_mode"), pmt::mp("nope"));
          } },
        { "code_8", "invalid_profile",
          [](pmt::pmt_t m) {
              return pmt::dict_add(m, pmt::mp("code_index"),
                                   pmt::from_long(8));
          } },
        { "code_13", "invalid_profile",
          [](pmt::pmt_t m) {
              return pmt::dict_add(m, pmt::mp("code_index"),
                                   pmt::from_long(13));
          } },
        { "len_mismatch", "invalid_profile",
          [](pmt::pmt_t m) {
              return pmt::dict_add(m, pmt::mp("tx_packet_samples"),
                                   pmt::from_long(99));
          } },
        { "neg_guard", "invalid_profile",
          [](pmt::pmt_t m) {
              return pmt::dict_add(m, pmt::mp("pre_guard_samples"),
                                   pmt::from_long(-4));
          } },
        { "missing_sfd", "invalid_profile",
          [](pmt::pmt_t m) {
              return pmt::dict_delete(m, pmt::mp("sfd_mode"));
          } },
    };

    for (const auto& row : rows) {
        auto echo = UwbLoopbackEcho::make(0, 0);
        pmt::pmt_t meta = row.mut(pmt::car(base));
        gr::blocks::message_debug::sptr st;
        pmt::pmt_t pdu = run_one(echo, pmt::cons(meta, pmt::cdr(base)), &st);
        BOOST_CHECK_MESSAGE(pmt::is_null(pdu), row.name);
        BOOST_CHECK_EQUAL(echo->pdus_emitted(), 0u);
        BOOST_CHECK_EQUAL(echo->pdus_dropped(), 1u);
        BOOST_REQUIRE_EQUAL(st->num_messages(), 1u);
        BOOST_CHECK_MESSAGE(status_has(st, row.event),
                            std::string(row.name) + " expected " + row.event);
    }
    (void)drop_key;
}

// ---------------------------------------------------------------------------
// Native 737.28 MS/s input (Step 9 production order: native packet ->
// loopback -> PDU 65/48).  The loopback must accept the native grid, apply
// delays on native sample indices and emit the scheduled-capture geometry
// (window_start_sample / pre_guard / capture / post_guard) the 65/48
// contract consumes.
// ---------------------------------------------------------------------------

BOOST_AUTO_TEST_CASE(test_native_rate_integer_delay)
{
    std::vector<gr_complex> tx(64);
    for (size_t i = 0; i < tx.size(); ++i)
        tx[i] = gr_complex(static_cast<float>(i) * 0.01f,
                           static_cast<float>(i) * -0.005f);
    constexpr size_t kPre = 8;
    constexpr size_t kTail = 48; // must cover the 37-sample channel delay
    const gr_complex gain = polar_gain(0.4, 0.7);

    auto echo = UwbLoopbackEcho::make(kPre, kTail, { 37.0 }, { gain });
    pmt::pmt_t pdu = run_one(echo, make_tx_pdu(tx, kFsNative));
    BOOST_REQUIRE(pmt::is_pair(pdu));
    auto rx = pdu_c32(pdu);
    BOOST_REQUIRE_EQUAL(rx.size(), kPre + tx.size() + kTail);
    // Delay applied on the native sample grid.
    for (size_t k = 0; k < tx.size(); ++k)
        BOOST_CHECK_SMALL(std::abs(rx[kPre + 37 + k] - gain * tx[k]), 1e-6f);

    pmt::pmt_t meta = pmt::car(pdu);
    BOOST_CHECK_CLOSE(dict_f64(meta, "sample_rate", 0.0), kFsNative, 1e-4);
    BOOST_CHECK_EQUAL(dict_str(meta, "sample_format"), std::string("fc32"));
    BOOST_CHECK_EQUAL(dict_i64(meta, "window_start_sample", -1), 0);
    BOOST_CHECK_EQUAL(dict_i64(meta, "pre_guard_samples", -1), 8);
    BOOST_CHECK_EQUAL(dict_i64(meta, "capture_samples", -1), 64);
    BOOST_CHECK_EQUAL(dict_i64(meta, "post_guard_samples", -1), 48);
    BOOST_CHECK_EQUAL(dict_i64(meta, "sample_count", -1), 120);
    BOOST_CHECK_EQUAL(dict_i64(meta, "tx_packet_samples", -1), 64);
    BOOST_CHECK_EQUAL(dict_i64(meta, "rx_capture_samples", -1), 120);
    BOOST_CHECK_CLOSE(dict_f64(meta, "calibration_delay_native_samples", -1),
                      37.0, 1e-9);
    BOOST_CHECK_EQUAL(dict_str(meta, "source"), std::string("loopback"));
    // RX window opened pre_guard before TX on the native grid.
    const double rx_t =
        static_cast<double>(dict_i64(meta, "rx_time_full", 0)) +
        dict_f64(meta, "rx_time_frac", 0.0);
    BOOST_CHECK_CLOSE(rx_t, -static_cast<double>(kPre) / kFsNative, 1e-6);
}

BOOST_AUTO_TEST_CASE(test_native_rate_frac_delay)
{
    std::vector<gr_complex> tx(16, gr_complex(0.3f, -0.1f));
    auto echo = UwbLoopbackEcho::make(4, 8, { 12.4 }, { polar_gain(0.5, 0.2) });
    pmt::pmt_t pdu = run_one(echo, make_tx_pdu(tx, kFsNative));
    BOOST_REQUIRE(pmt::is_pair(pdu));
    BOOST_CHECK_EQUAL(echo->pdus_emitted(), 1u);
    BOOST_CHECK_EQUAL(echo->pdus_dropped(), 0u);
    BOOST_CHECK_CLOSE(dict_f64(pmt::car(pdu),
                               "calibration_delay_native_samples", -1),
                      12.4, 1e-9);
    BOOST_CHECK_EQUAL(dict_i64(pmt::car(pdu), "sample_count", -1), 28);
}

BOOST_AUTO_TEST_CASE(test_native_sync_span_validation)
{
    // Native grid validates provided SYNC/SFD spans with the ceil
    // convention (canonical native SYNC spans 24009/48018/96036 for
    // 32/64/128 repetitions); work-domain spans are rejected.
    auto with_sync = [](double rate, int64_t sync_samples) {
        std::vector<gr_complex> tx(8, gr_complex(0.2f, 0.0f));
        pmt::pmt_t pdu = make_tx_pdu(tx, rate);
        pmt::pmt_t meta = pmt::dict_add(pmt::car(pdu),
                                        pmt::mp("sync_samples"),
                                        pmt::from_long(sync_samples));
        return pmt::cons(meta, pmt::cdr(pdu));
    };

    {
        auto echo = UwbLoopbackEcho::make(0, 0);
        BOOST_REQUIRE(pmt::is_pair(run_one(echo, with_sync(kFsNative, 48018))));
        BOOST_CHECK_EQUAL(echo->pdus_emitted(), 1u);
    }
    {
        auto echo = UwbLoopbackEcho::make(0, 0);
        gr::blocks::message_debug::sptr st;
        BOOST_CHECK(pmt::is_null(run_one(echo, with_sync(kFsNative, 65024), &st)));
        BOOST_CHECK_EQUAL(echo->pdus_dropped(), 1u);
        BOOST_REQUIRE(status_has(st, "invalid_profile"));
    }
    // Still rejected outright for rates that are neither grid.
    {
        auto echo = UwbLoopbackEcho::make(0, 0);
        gr::blocks::message_debug::sptr st;
        BOOST_CHECK(pmt::is_null(
            run_one(echo, make_tx_pdu(std::vector<gr_complex>(8), 1.0e6), &st)));
        BOOST_CHECK_EQUAL(echo->pdus_dropped(), 1u);
        BOOST_REQUIRE(status_has(st, "bad_input_rate"));
    }
}
