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

    BOOST_REQUIRE_EQUAL(dbg->num_messages(), 1);
    pmt::pmt_t meta = pmt::car(dbg->get_message(0));

    const uint64_t start = pmt::to_uint64(
        pmt::dict_ref(meta, pmt::mp("start_sample"), pmt::PMT_NIL));
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
