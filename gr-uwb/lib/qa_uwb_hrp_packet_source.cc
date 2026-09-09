/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <boost/test/unit_test.hpp>

#include <gnuradio/blocks/message_debug.h>
#include <gnuradio/gr_complex.h>
#include <gnuradio/top_block.h>
#include <gnuradio/uwb/uwb_hrp_packet_source.h>
#include <gnuradio/uwb/uwb_loopback_echo.h>
#include <gnuradio/uwb/uwb_radar_cir_estimator_block.h>
#include <pmt/pmt.h>

#include <chrono>
#include <cstdint>
#include <fstream>
#include <string>
#include <thread>
#include <vector>

namespace {

using gr::uwb::UwbHrpPacketSource;
using gr::uwb::UwbLoopbackEcho;
using gr::uwb::UwbRadarCirEstimator;

#ifdef UWB_TESTDATA_DIR
const char* kTestdata = UWB_TESTDATA_DIR;
#else
const char* kTestdata = "../../../testdata";
#endif

bool load_u8(const std::string& path, std::vector<uint8_t>& out)
{
    std::ifstream f(path, std::ios::binary);
    if (!f)
        return false;
    f.seekg(0, std::ios::end);
    const auto bytes = static_cast<size_t>(f.tellg());
    f.seekg(0);
    out.resize(bytes);
    f.read(reinterpret_cast<char*>(out.data()),
           static_cast<std::streamsize>(bytes));
    return static_cast<bool>(f);
}

pmt::pmt_t run_emit(UwbHrpPacketSource::sptr blk, pmt::pmt_t emit_msg)
{
    auto dbg = gr::blocks::message_debug::make();
    auto tb = gr::make_top_block("qa_hrp_packet_source");
    tb->msg_connect(blk, "tx", dbg, "store");
    tb->start();
    blk->_post(pmt::mp("emit"), emit_msg);
    for (int i = 0; i < 200 && dbg->num_messages() == 0 &&
                    blk->pdus_dropped() == 0; ++i)
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    tb->stop();
    tb->wait();
    if (dbg->num_messages() == 0)
        return pmt::PMT_NIL;
    return dbg->get_message(0);
}

} // namespace

BOOST_AUTO_TEST_CASE(test_hrp_packet_source_emits_p1_length)
{
    std::vector<uint8_t> bytes;
    BOOST_REQUIRE(load_u8(std::string(kTestdata) +
                              "/realtime_demod_golden/stage_payload_bytes.bin",
                          bytes));
    BOOST_REQUIRE_EQUAL(bytes.size(), size_t(127));

    auto blk = UwbHrpPacketSource::make(bytes, 64, "ieee", 9);
    BOOST_REQUIRE_EQUAL(blk->num_samples(), 249280u);
    BOOST_REQUIRE_EQUAL(blk->samples().size(), 249280u);

    pmt::pmt_t pdu = run_emit(blk, pmt::make_dict());
    BOOST_REQUIRE(pmt::is_pair(pdu));
    pmt::pmt_t meta = pmt::car(pdu);
    pmt::pmt_t vec = pmt::cdr(pdu);
    BOOST_CHECK(pmt::is_c32vector(vec));
    BOOST_CHECK_EQUAL(pmt::length(vec), 249280);
    BOOST_CHECK_EQUAL(
        pmt::symbol_to_string(pmt::dict_ref(meta, pmt::mp("source"),
                                            pmt::PMT_NIL)),
        std::string("hrp_packet_source"));
    BOOST_CHECK_EQUAL(blk->pdus_emitted(), 1u);
}

BOOST_AUTO_TEST_CASE(test_hrp_packet_source_rejects_bad_sync)
{
    std::vector<uint8_t> b{ 1 };
    BOOST_CHECK_THROW(UwbHrpPacketSource::make(b, 16, "ieee", 9),
                      std::invalid_argument);
}

BOOST_AUTO_TEST_CASE(test_hrp_packet_source_sts_requires_4z)
{
    std::vector<uint8_t> b{ 1, 2 };
    BOOST_CHECK_THROW(
        UwbHrpPacketSource::make(b, 64, "ieee", 9, 0.8f, 0.005, false, true),
        std::invalid_argument);
    auto blk = UwbHrpPacketSource::make(b, 64, "4z2", 9, 0.8f, 0.005, false,
                                        true, false);
    BOOST_CHECK(blk->insert_sts());
    BOOST_CHECK_EQUAL(blk->num_samples(),
                      gr::uwb::mod::packet_samples_998p4(64, 8, 2, true));
}

BOOST_AUTO_TEST_CASE(test_hrp_packet_source_append_fcs_and_emit_override)
{
    std::vector<uint8_t> data(8, 0x11);
    auto blk = UwbHrpPacketSource::make(data, 64, "ieee", 9, 0.8f, 0.005, false,
                                        false, true);
    BOOST_REQUIRE_EQUAL(blk->psdu().size(), size_t(10));
    const size_t n0 = blk->num_samples();
    BOOST_CHECK_EQUAL(n0, gr::uwb::mod::packet_samples_998p4(64, 8, 10));

    std::vector<uint8_t> other(4, 0x22);
    pmt::pmt_t pdu = run_emit(
        blk, pmt::cons(pmt::make_dict(),
                       pmt::init_u8vector(other.size(), other.data())));
    BOOST_REQUIRE(pmt::is_pair(pdu));
    BOOST_CHECK_EQUAL(blk->psdu().size(), size_t(4));
    BOOST_CHECK_EQUAL(blk->num_samples(),
                      gr::uwb::mod::packet_samples_998p4(64, 8, 4));
}

BOOST_AUTO_TEST_CASE(test_hrp_packet_source_radar_loopback_cir)
{
    // Same TX profile as uwb_radar_loopback_cir_synth.grc (radar golden).
    const uint8_t psdu22[] = {
        0x47, 0x26, 0x1D, 0xF6, 0x6F, 0x4C, 0x1B, 0xEF, 0x45, 0xC8, 0xF7,
        0x7C, 0xE7, 0x7B, 0xD7, 0xD8, 0xC4, 0xD1, 0x80, 0xFB, 0x12, 0x21
    };
    std::vector<uint8_t> psdu(psdu22, psdu22 + sizeof(psdu22));
    auto src = UwbHrpPacketSource::make(psdu, 64, "4z2", 9, 0.8f, 0.005,
                                        false, true, false);
    BOOST_REQUIRE_EQUAL(src->num_samples(), 190912u);

    auto echo = UwbLoopbackEcho::make(
        1997, 4096, { 0.0 }, { gr_complex(1.0f, 0.0f) }, 0.0f, 1,
        4194304, 4194304);
    const std::string tmpl =
        std::string(kTestdata) + "/uwb_radar/sync_template_998p4.cf32";
    auto est = UwbRadarCirEstimator::make(tmpl, 64, "4z2", 9, 16, 100, 10,
                                          0, 64, 8, 0.3f, 0.3f, true, 8);
    auto dbg = gr::blocks::message_debug::make();
    auto tb = gr::make_top_block("qa_hrp_radar_loopback");
    tb->msg_connect(src, "tx", echo, "tx");
    tb->msg_connect(echo, "rx", est, "rx");
    tb->msg_connect(est, "cir", dbg, "store");
    tb->start();
    src->_post(pmt::mp("emit"), pmt::make_dict());
    const auto t0 = std::chrono::steady_clock::now();
    while (dbg->num_messages() == 0) {
        const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                            std::chrono::steady_clock::now() - t0)
                            .count();
        BOOST_REQUIRE_LT(ms, 15000);
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    BOOST_REQUIRE(est->drained());
    tb->stop();
    tb->wait();

    BOOST_REQUIRE_EQUAL(dbg->num_messages(), 1);
    pmt::pmt_t pdu = dbg->get_message(0);
    BOOST_REQUIRE(pmt::is_pair(pdu));
    pmt::pmt_t meta = pmt::car(pdu);
    BOOST_CHECK_EQUAL(
        pmt::symbol_to_string(pmt::dict_ref(meta, pmt::mp("status"),
                                            pmt::PMT_NIL)),
        std::string("ok"));
    BOOST_CHECK_EQUAL(
        pmt::to_uint64(pmt::dict_ref(meta, pmt::mp("tap_count"),
                                     pmt::from_uint64(0))),
        116u);
    BOOST_CHECK_GE(est->pdus_completed(), 1u);
}
