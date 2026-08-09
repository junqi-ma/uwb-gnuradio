/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <boost/test/unit_test.hpp>

#include <gnuradio/blocks/message_debug.h>
#include <gnuradio/blocks/vector_source.h>
#include <gnuradio/gr_complex.h>
#include <gnuradio/top_block.h>
#include <gnuradio/uwb/uwb_detector.h>
#include <gnuradio/uwb/uwb_detector_core.h>
#include <pmt/pmt.h>

#include <cmath>
#include <complex>
#include <vector>

namespace {

void run_detector(const std::vector<gr_complex>& x,
                  gr::uwb::UwbDetector::sptr det,
                  gr::blocks::message_debug::sptr dbg)
{
    auto src = gr::blocks::vector_source_c::make(x);
    auto tb = gr::make_top_block("qa_detector");
    tb->connect(src, 0, det, 0);
    tb->msg_connect(det, "packet", dbg, "store");
    tb->run();
}

} // namespace

BOOST_AUTO_TEST_CASE(test_ring_buffer_bulk_wrap_keeps_latest)
{
    gr::uwb::RingBuffer ring(5);
    std::vector<gr_complex> x(13);
    for (size_t i = 0; i < x.size(); ++i)
        x[i] = gr_complex(static_cast<float>(i + 1), -static_cast<float>(i + 1));

    ring.push(x.data(), 3);
    ring.push(x.data() + 3, x.size() - 3);
    const auto got = ring.to_vector();

    BOOST_REQUIRE_EQUAL(got.size(), 5);
    for (size_t i = 0; i < got.size(); ++i)
        BOOST_CHECK_EQUAL(got[i], x[x.size() - got.size() + i]);
}

BOOST_AUTO_TEST_CASE(test_detector_preamble_pdu)
{
    // A repeated-symbol preamble surrounded by silence must produce exactly one
    // PDU whose start_sample is near the first symbol and whose metric is a
    // confirmed correlation (> 0.5).
    const size_t L = 128;
    std::vector<gr_complex> tmpl(L);
    for (size_t k = 0; k < L; ++k)
        tmpl[k] = gr_complex(std::cos(0.21f * k), 0.4f * std::sin(0.13f * k));
    gr::uwb::core::uwb_l2_normalize(tmpl);

    const int lead = 512;
    const int reps = 10;
    const int tail = 2048;
    const int N = lead + reps * static_cast<int>(L) + tail;
    std::vector<gr_complex> sig(static_cast<size_t>(N), gr_complex(0.0f, 0.0f));
    for (int r = 0; r < reps; ++r)
        for (size_t k = 0; k < L; ++k)
            sig[static_cast<size_t>(lead + r * static_cast<int>(L) + k)] = tmpl[k];

    auto det = gr::uwb::UwbDetector::make(tmpl,
                                          /*pre_trigger=*/64,
                                          /*capture=*/500,
                                          /*energy_threshold=*/1e-3f,
                                          /*energy_gate_decimation=*/4,
                                          /*coarse_decimation=*/4,
                                          /*coarse_repetitions=*/1,
                                          /*coarse_margin=*/8);
    BOOST_CHECK_EQUAL(det->pre_trigger(), 64);
    BOOST_CHECK_EQUAL(det->capture(), 500);

    auto dbg = gr::blocks::message_debug::make();
    run_detector(sig, det, dbg);
    BOOST_CHECK_EQUAL(det->dropped_regions(), 0);

    BOOST_REQUIRE_EQUAL(dbg->num_messages(), 1);
    pmt::pmt_t meta = pmt::car(dbg->get_message(0));

    const uint64_t start = pmt::to_uint64(
        pmt::dict_ref(meta, pmt::mp("start_sample"), pmt::PMT_NIL));
    BOOST_CHECK_EQUAL(
        pmt::to_uint64(pmt::dict_ref(meta, pmt::mp("predicted_start_sample"),
                                    pmt::PMT_NIL)),
        start);
    BOOST_CHECK_EQUAL(
        pmt::to_uint64(pmt::dict_ref(meta, pmt::mp("window_start_sample"),
                                    pmt::PMT_NIL)),
        start - 64);
    const long n = pmt::to_long(
        pmt::dict_ref(meta, pmt::mp("sample_count"), pmt::PMT_NIL));
    const double metric = pmt::to_double(
        pmt::dict_ref(meta, pmt::mp("detection_metric"), pmt::PMT_NIL));

    BOOST_CHECK_GE(start, static_cast<uint64_t>(lead));
    BOOST_CHECK_LT(start, static_cast<uint64_t>(lead + 2 * L));
    BOOST_CHECK_EQUAL(n, 64 + 500); // capture = pre_trigger + capture samples
    BOOST_CHECK_GT(metric, 0.5f);
}

BOOST_AUTO_TEST_CASE(test_detector_silence_no_packet)
{
    // Pure silence must not produce a PDU (and the state machine must not
    // crash or trigger on the decimated energy gate noise floor).
    std::vector<gr_complex> x(50000, gr_complex(0.0f, 0.0f));
    std::vector<gr_complex> tmpl(128, gr_complex(0.125f, 0.0f));

    auto det = gr::uwb::UwbDetector::make(tmpl, 64, 2000, 1e-3f, 4, 4, 1, 8);
    auto dbg = gr::blocks::message_debug::make();
    run_detector(x, det, dbg);

    BOOST_CHECK_EQUAL(dbg->num_messages(), 0);
}

// work() instrumentation is used by layered benchmarks for buffer attribution.
// Drive the real UwbDetector work() path and assert stats are filled (not
// hard-coded expectations for chunk sizes — only that the shipped counters run).
BOOST_AUTO_TEST_CASE(test_detector_work_chunk_stats)
{
    std::vector<gr_complex> x(100000, gr_complex(0.0f, 0.0f));
    std::vector<gr_complex> tmpl(128, gr_complex(0.125f, 0.0f));
    auto det = gr::uwb::UwbDetector::make(tmpl, 64, 2000, 1e30f, 4, 4, 1, 8);
    det->reset_work_stats();
    BOOST_CHECK_EQUAL(det->work_calls(), 0);

    auto dbg = gr::blocks::message_debug::make();
    run_detector(x, det, dbg);

    BOOST_CHECK_EQUAL(dbg->num_messages(), 0);
    BOOST_CHECK_GT(det->work_calls(), 0);
    BOOST_CHECK_GT(det->work_items_total(), 0);
    BOOST_CHECK_GT(det->work_min_noutput_items(), 0);
    BOOST_CHECK_GE(det->work_max_noutput_items(), det->work_min_noutput_items());
    BOOST_CHECK_GT(det->work_mean_noutput_items(), 0.0);
    uint64_t hist[5] = {};
    det->work_noutput_histogram(hist);
    uint64_t hist_sum = hist[0] + hist[1] + hist[2] + hist[3] + hist[4];
    BOOST_CHECK_EQUAL(hist_sum, det->work_calls());
}

// Long trailing payload after a short synthetic preamble must still detect.
// Exercises the production preamble-horizon clamp: coarse scans only the
// SYNC+SFD prefix, not the full multi-100k region (publish_packet P0).
BOOST_AUTO_TEST_CASE(test_detector_long_region_horizon_still_detects)
{
    const size_t L = 128;
    std::vector<gr_complex> tmpl(L);
    for (size_t k = 0; k < L; ++k)
        tmpl[k] = gr_complex(std::cos(0.21f * k), 0.4f * std::sin(0.13f * k));
    gr::uwb::core::uwb_l2_normalize(tmpl);

    const size_t lead = 1024;
    const size_t reps = 16; // enough SYNC-like symbols for coarse existence
    const size_t payload = 200000; // long payload tail (would dominate full scan)
    const size_t tail = 4096;
    std::vector<gr_complex> x(lead + reps * L + payload + tail,
                              gr_complex(0.0f, 0.0f));
    for (size_t r = 0; r < reps; ++r)
        for (size_t k = 0; k < L; ++k)
            x[lead + r * L + k] = tmpl[k];
    // Non-zero but non-preamble payload so energy gate stays up longer.
    for (size_t i = 0; i < payload; ++i)
        x[lead + reps * L + i] =
            gr_complex(0.05f * std::sin(0.03f * static_cast<float>(i)),
                       0.05f * std::cos(0.02f * static_cast<float>(i)));

    auto det = gr::uwb::UwbDetector::make(tmpl,
                                          /*pre_trigger=*/64,
                                          /*capture=*/500,
                                          /*energy_threshold=*/1e-3f,
                                          /*energy_gate_decimation=*/4,
                                          /*coarse_decimation=*/4,
                                          /*coarse_repetitions=*/1,
                                          /*coarse_margin=*/8);
    auto dbg = gr::blocks::message_debug::make();
    run_detector(x, det, dbg);
    BOOST_CHECK_EQUAL(det->dropped_regions(), 0);
    BOOST_REQUIRE_EQUAL(dbg->num_messages(), 1);

    const pmt::pmt_t meta = pmt::car(dbg->get_message(0));
    const uint64_t start = pmt::to_uint64(
        pmt::dict_ref(meta, pmt::mp("start_sample"), pmt::PMT_NIL));
    const double metric = pmt::to_double(
        pmt::dict_ref(meta, pmt::mp("detection_metric"), pmt::PMT_NIL));
    BOOST_CHECK_GE(start, static_cast<uint64_t>(lead));
    BOOST_CHECK_LT(start, static_cast<uint64_t>(lead + 2 * L));
    BOOST_CHECK_GT(metric, 0.5);
}

BOOST_AUTO_TEST_CASE(test_detector_two_packets_payload_matches_input)
{
    const size_t L = 128;
    const size_t pre = 64;
    const size_t capture = 640;
    std::vector<gr_complex> tmpl(L);
    for (size_t k = 0; k < L; ++k)
        tmpl[k] = gr_complex(std::cos(0.21f * k), 0.4f * std::sin(0.13f * k));
    gr::uwb::core::uwb_l2_normalize(tmpl);

    const size_t first = 1024;
    const size_t second = 8192;
    const size_t reps = 10;
    std::vector<gr_complex> x(20000, gr_complex(0.0f, 0.0f));
    for (const size_t start : { first, second }) {
        for (size_t r = 0; r < reps; ++r)
            for (size_t k = 0; k < L; ++k)
                x[start + r * L + k] = tmpl[k];
    }

    auto det = gr::uwb::UwbDetector::make(tmpl,
                                          pre,
                                          capture,
                                          /*energy_threshold=*/1e-3f,
                                          /*energy_gate_decimation=*/4,
                                          /*coarse_decimation=*/4,
                                          /*coarse_repetitions=*/1,
                                          /*coarse_margin=*/8);
    auto dbg = gr::blocks::message_debug::make();
    run_detector(x, det, dbg);
    BOOST_CHECK_EQUAL(det->dropped_regions(), 0);

    BOOST_REQUIRE_EQUAL(dbg->num_messages(), 2);
    uint64_t previous_start = 0;
    for (size_t message_index = 0; message_index < 2; ++message_index) {
        const pmt::pmt_t msg = dbg->get_message(message_index);
        const pmt::pmt_t meta = pmt::car(msg);
        const pmt::pmt_t data = pmt::cdr(msg);
        const uint64_t start = pmt::to_uint64(
            pmt::dict_ref(meta, pmt::mp("start_sample"), pmt::PMT_NIL));
        const auto iq = pmt::c32vector_elements(data);

        BOOST_REQUIRE_GE(start, pre);
        BOOST_REQUIRE_EQUAL(iq.size(), pre + capture);
        if (message_index > 0)
            BOOST_CHECK_GT(start, previous_start);
        previous_start = start;

        const size_t input_begin = static_cast<size_t>(start) - pre;
        BOOST_REQUIRE_LE(input_begin + iq.size(), x.size());
        for (size_t i = 0; i < iq.size(); ++i)
            BOOST_CHECK_EQUAL(iq[i], x[input_begin + i]);
    }
}

// A short energetic preamble followed by silence must not truncate a configured
// capture.  Detection currently happens only after the energy Region closes,
// so this also exercises collection of the post-detection tail across scheduler
// chunks.
BOOST_AUTO_TEST_CASE(test_detector_short_region_emits_fixed_capture)
{
    const size_t L = 128;
    const size_t pre = 64;
    const size_t capture = 5000;
    const size_t start_expected = 2048;
    const size_t reps = 10;

    std::vector<gr_complex> tmpl(L);
    for (size_t k = 0; k < L; ++k)
        tmpl[k] = gr_complex(std::cos(0.21f * k), 0.4f * std::sin(0.13f * k));
    gr::uwb::core::uwb_l2_normalize(tmpl);

    std::vector<gr_complex> x(start_expected + capture + 4096,
                              gr_complex(0.0f, 0.0f));
    for (size_t r = 0; r < reps; ++r)
        for (size_t k = 0; k < L; ++k)
            x[start_expected + r * L + k] = tmpl[k];

    auto det = gr::uwb::UwbDetector::make(tmpl,
                                          pre,
                                          capture,
                                          /*energy_threshold=*/1e-3f,
                                          /*energy_gate_decimation=*/4,
                                          /*coarse_decimation=*/4,
                                          /*coarse_repetitions=*/1,
                                          /*coarse_margin=*/8);
    auto dbg = gr::blocks::message_debug::make();
    run_detector(x, det, dbg);

    BOOST_REQUIRE_EQUAL(dbg->num_messages(), 1);
    const pmt::pmt_t msg = dbg->get_message(0);
    const pmt::pmt_t meta = pmt::car(msg);
    const auto iq = pmt::c32vector_elements(pmt::cdr(msg));
    const uint64_t start = pmt::to_uint64(
        pmt::dict_ref(meta, pmt::mp("start_sample"), pmt::PMT_NIL));

    BOOST_REQUIRE_EQUAL(iq.size(), pre + capture);
    BOOST_REQUIRE_GE(start, pre);
    const size_t input_begin = static_cast<size_t>(start) - pre;
    BOOST_REQUIRE_LE(input_begin + iq.size(), x.size());
    for (size_t i = 0; i < iq.size(); ++i)
        BOOST_CHECK_EQUAL(iq[i], x[input_begin + i]);
}
