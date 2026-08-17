#define BOOST_TEST_MODULE qa_uwb_scheduled_extractor_sc16
#include <boost/test/unit_test.hpp>

#include <gnuradio/blocks/message_debug.h>
#include <gnuradio/blocks/stream_to_vector.h>
#include <gnuradio/blocks/vector_source.h>
#include <gnuradio/top_block.h>
#include <gnuradio/uwb/uwb_packet_writer.h>
#include <gnuradio/uwb/uwb_scheduled_extractor_sc16.h>
#include <pmt/pmt.h>

#include <cstdint>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <string>
#include <thread>
#include <vector>

BOOST_AUTO_TEST_CASE(test_sc16_scheduled_window_is_bit_exact)
{
    std::vector<int16_t> raw;
    for (int16_t i = 0; i < 64; ++i) {
        raw.push_back(i);
        raw.push_back(static_cast<int16_t>(-i));
    }
    auto src = gr::blocks::vector_source_s::make(raw, false);
    // Turn consecutive I/Q shorts into one 4-byte stream item expected by the
    // SC16 extractor; no numeric conversion occurs in this chain.
    auto pack = gr::blocks::stream_to_vector::make(sizeof(int16_t), 2);
    auto ext = gr::uwb::UwbScheduledExtractorSc16::make(
        1.0, 1000.0, 20, 4, 8, 4, 2);
    auto dbg = gr::blocks::message_debug::make();
    auto tb = gr::make_top_block("qa_scheduled_sc16");
    tb->connect(src, 0, pack, 0);
    tb->connect(pack, 0, ext, 0);
    tb->msg_connect(ext, "packet", dbg, "store");
    tb->start();
    for (int i = 0; i < 100 && dbg->num_messages() == 0; ++i)
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    tb->stop();
    tb->wait();

    BOOST_REQUIRE_EQUAL(dbg->num_messages(), 1);
    const auto pdu = dbg->get_message(0);
    BOOST_REQUIRE(pmt::is_pair(pdu));
    const auto meta = pmt::car(pdu);
    const auto data = pmt::cdr(pdu);
    BOOST_REQUIRE(pmt::is_s16vector(data));
    size_t n = 0;
    const int16_t* iq = pmt::s16vector_elements(data, n);
    BOOST_REQUIRE_EQUAL(n, 32u); // 16 complex samples × I/Q
    for (size_t k = 0; k < 16; ++k) {
        BOOST_CHECK_EQUAL(iq[2 * k], static_cast<int16_t>(16 + k));
        BOOST_CHECK_EQUAL(iq[2 * k + 1], static_cast<int16_t>(-(16 + k)));
    }
    BOOST_CHECK_EQUAL(pmt::to_long(pmt::dict_ref(
                          meta, pmt::mp("window_start_sample"), pmt::from_long(-1))),
                      16);
    BOOST_CHECK_EQUAL(pmt::symbol_to_string(pmt::dict_ref(
                          meta, pmt::mp("sample_format"), pmt::PMT_NIL)),
                      "sc16");
}

BOOST_AUTO_TEST_CASE(test_sc16_extractor_writer_keeps_head_body_tail)
{
    const std::string dir = "qa_sched_sc16_dump";
    std::filesystem::remove_all(dir);

    // 64 complex samples. predicted=20, pre=4, body=8, post=4 → window [16,32).
    std::vector<int16_t> raw;
    raw.reserve(128);
    for (int16_t i = 0; i < 64; ++i) {
        raw.push_back(i);
        raw.push_back(static_cast<int16_t>(-i));
    }
    auto src = gr::blocks::vector_source_s::make(raw, false);
    auto pack = gr::blocks::stream_to_vector::make(sizeof(int16_t), 2);
    auto ext = gr::uwb::UwbScheduledExtractorSc16::make(
        1.0, 1000.0, 20, 4, 8, 4, 2);
    auto w = gr::uwb::UwbPacketWriter::make(dir, "capture", false);
    auto tb = gr::make_top_block("qa_sched_sc16_dump");
    tb->connect(src, 0, pack, 0);
    tb->connect(pack, 0, ext, 0);
    tb->msg_connect(ext, "packet", w, "packet");
    tb->start();
    for (int i = 0; i < 100 && w->packets_written() == 0; ++i)
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    tb->stop();
    tb->wait();

    BOOST_REQUIRE_GE(w->packets_written(), 1);
    BOOST_CHECK_EQUAL(w->packets_dropped(), 0);

    std::ifstream jf(dir + "/capture.jsonl");
    std::string jsonl;
    std::getline(jf, jsonl);
    BOOST_CHECK(jsonl.find("\"window_start_sample\":16") != std::string::npos);
    BOOST_CHECK(jsonl.find("\"predicted_start_sample\":20") != std::string::npos);
    BOOST_CHECK(jsonl.find("\"pre_guard_samples\":4") != std::string::npos);
    BOOST_CHECK(jsonl.find("\"capture_samples\":8") != std::string::npos);
    BOOST_CHECK(jsonl.find("\"post_guard_samples\":4") != std::string::npos);

    std::ifstream iq(dir + "/capture.iq", std::ios::binary);
    std::vector<int16_t> dumped(32);
    iq.read(reinterpret_cast<char*>(dumped.data()),
            static_cast<std::streamsize>(dumped.size() * sizeof(int16_t)));
    BOOST_REQUIRE_EQUAL(iq.gcount(),
                        static_cast<std::streamsize>(dumped.size() *
                                                     sizeof(int16_t)));
    for (size_t k = 0; k < 16; ++k) {
        BOOST_CHECK_EQUAL(dumped[2 * k], static_cast<int16_t>(16 + k));
        BOOST_CHECK_EQUAL(dumped[2 * k + 1], static_cast<int16_t>(-(16 + k)));
    }
    std::filesystem::remove_all(dir);
}
