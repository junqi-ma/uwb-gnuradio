/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <boost/test/unit_test.hpp>

#include <gnuradio/blocks/message_strobe.h>
#include <gnuradio/gr_complex.h>
#include <gnuradio/top_block.h>
#include <gnuradio/uwb/uwb_packet_writer.h>
#include <pmt/pmt.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace {

pmt::pmt_t make_pdu(long id,
                    uint64_t start,
                    const std::vector<gr_complex>& iq,
                    double metric = 0.99)
{
    pmt::pmt_t meta = pmt::make_dict();
    meta = pmt::dict_add(meta, pmt::mp("packet_id"), pmt::from_long(id));
    meta = pmt::dict_add(meta, pmt::mp("start_sample"), pmt::from_uint64(start));
    meta = pmt::dict_add(meta, pmt::mp("trigger_sample"), pmt::from_uint64(start));
    meta = pmt::dict_add(meta, pmt::mp("sample_rate"),
                         pmt::from_double(998400000.0));
    meta = pmt::dict_add(meta, pmt::mp("sample_count"),
                         pmt::from_long(static_cast<long>(iq.size())));
    meta = pmt::dict_add(meta, pmt::mp("detection_metric"), pmt::from_double(metric));
    meta = pmt::dict_add(meta, pmt::mp("pre_trigger_samples"), pmt::from_long(2032));
    return pmt::cons(meta, pmt::init_c32vector(iq.size(), iq));
}

std::string read_file(const std::string& path)
{
    std::ifstream f(path, std::ios::binary);
    std::ostringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

void run_strobe(const pmt::pmt_t& pdu,
                gr::uwb::UwbPacketWriter::sptr w,
                long period_ms = 5)
{
    auto strobe = gr::blocks::message_strobe::make(pdu, period_ms);
    auto tb = gr::make_top_block("qa_writer");
    tb->msg_connect(strobe, "strobe", w, "packet");
    tb->start();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    tb->stop();
    tb->wait();
}

} // namespace

BOOST_AUTO_TEST_CASE(test_packet_writer_shared_sc16)
{
    const std::string dir = "qa_writer_out";
    std::filesystem::remove_all(dir);

    auto w = gr::uwb::UwbPacketWriter::make(dir, "capture", false);
    // Peak |I|/|Q| = 4 → scale = 32767/4
    std::vector<gr_complex> iq = {
        gr_complex(1, 0), gr_complex(2, 1), gr_complex(3, 0), gr_complex(4, -1)};
    run_strobe(make_pdu(0, 1234, iq, 0.99), w);

    BOOST_CHECK_GE(w->packets_written(), 1);
    BOOST_CHECK_GE(w->samples_written(), 4);

    const std::string iq_bin = read_file(dir + "/capture.iq");
    // SC16: 4 bytes per complex sample
    BOOST_CHECK_EQUAL(iq_bin.size(), w->samples_written() * 4u);

    const auto* s = reinterpret_cast<const int16_t*>(iq_bin.data());
    const float scale = 32767.0f / 4.0f;
    BOOST_CHECK_EQUAL(s[0], static_cast<int16_t>(std::round(1.0f * scale)));
    BOOST_CHECK_EQUAL(s[1], 0);
    BOOST_CHECK_EQUAL(s[6], static_cast<int16_t>(std::round(4.0f * scale)));
    BOOST_CHECK_EQUAL(s[7], static_cast<int16_t>(std::round(-1.0f * scale)));

    const std::string jsonl = read_file(dir + "/capture.jsonl");
    BOOST_CHECK(jsonl.find("\"packet_id\":0") != std::string::npos);
    BOOST_CHECK(jsonl.find("\"start_sample\":1234") != std::string::npos);
    BOOST_CHECK(jsonl.find("\"sample_format\":\"sc16\"") != std::string::npos);
    BOOST_CHECK(jsonl.find("\"iq_scale\"") != std::string::npos);

    std::filesystem::remove_all(dir);
}

BOOST_AUTO_TEST_CASE(test_packet_writer_one_file_per_packet_sc16)
{
    const std::string dir = "qa_writer_pkt";
    std::filesystem::remove_all(dir);

    auto w = gr::uwb::UwbPacketWriter::make(dir, "capture", true);
    std::vector<gr_complex> iq(3, gr_complex(1.5f, -0.5f));
    run_strobe(make_pdu(7, 999, iq), w);

    BOOST_CHECK_GE(w->packets_written(), 1);
    BOOST_CHECK(std::filesystem::exists(dir + "/packet_7.iq"));
    BOOST_CHECK_EQUAL(read_file(dir + "/packet_7.iq").size(), 3u * 4u);

    const std::string jsonl = read_file(dir + "/capture.jsonl");
    BOOST_CHECK(jsonl.find("\"file\":\"packet_7.iq\"") != std::string::npos);
    BOOST_CHECK(jsonl.find("\"sample_format\":\"sc16\"") != std::string::npos);

    std::filesystem::remove_all(dir);
}

BOOST_AUTO_TEST_CASE(test_packet_writer_accepts_scheduled_metadata)
{
    const std::string dir = "qa_writer_scheduled";
    std::filesystem::remove_all(dir);
    auto w = gr::uwb::UwbPacketWriter::make(dir, "capture", false);
    std::vector<gr_complex> iq(8, gr_complex(0.25f, -0.125f));
    pmt::pmt_t meta = pmt::make_dict();
    meta = pmt::dict_add(meta, pmt::mp("packet_id"), pmt::from_uint64(42));
    meta = pmt::dict_add(meta, pmt::mp("window_start_sample"),
                         pmt::from_long(100000));
    meta = pmt::dict_add(meta, pmt::mp("predicted_start_sample"),
                         pmt::from_long(109984));
    meta = pmt::dict_add(meta, pmt::mp("sample_rate"),
                         pmt::from_double(998.4e6));
    run_strobe(pmt::cons(meta, pmt::init_c32vector(iq.size(), iq)), w);
    const std::string jsonl = read_file(dir + "/capture.jsonl");
    BOOST_CHECK(jsonl.find("\"packet_id\":42") != std::string::npos);
    BOOST_CHECK(jsonl.find("\"start_sample\":100000") != std::string::npos);
    BOOST_CHECK(jsonl.find("\"trigger_sample\":109984") != std::string::npos);
    std::filesystem::remove_all(dir);
}
