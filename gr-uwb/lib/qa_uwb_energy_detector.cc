/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <boost/test/unit_test.hpp>

#include <gnuradio/blocks/vector_sink.h>
#include <gnuradio/blocks/vector_source.h>
#include <gnuradio/gr_complex.h>
#include <gnuradio/top_block.h>
#include <gnuradio/uwb/uwb_detector_core.h>
#include <gnuradio/uwb/uwb_energy_detector.h>

#include <cmath>
#include <vector>

namespace {

// Feed `in` through the block in a real flowgraph; return metric + flag.
void run_flowgraph(const std::vector<gr_complex>& in,
                   gr::uwb::UwbEnergyDetector::sptr blk,
                   std::vector<float>& metric,
                   std::vector<unsigned char>& flag)
{
    auto src = gr::blocks::vector_source_c::make(in);
    auto snk_metric = gr::blocks::vector_sink_f::make();
    auto snk_flag = gr::blocks::vector_sink_b::make();
    auto tb = gr::make_top_block("qa_energy");
    tb->connect(src, 0, blk, 0);
    tb->connect(blk, 0, snk_metric, 0);
    tb->connect(blk, 1, snk_flag, 0);
    tb->run();
    metric = snk_metric->data();
    flag = snk_flag->data();
}

} // namespace

BOOST_AUTO_TEST_CASE(test_core_window_energy)
{
    // N = 4, window = 3, history + signal = 6 samples.
    // Note: |complex(2,0)|^2 = 4, so the trailing averages are [1,2,3,4].
    const size_t W = 3;
    const size_t N = 4;
    std::vector<gr_complex> in = {
        gr_complex(1, 0), gr_complex(1, 0), gr_complex(1, 0),
        gr_complex(2, 0), gr_complex(2, 0), gr_complex(2, 0)};
    std::vector<float> out(N);
    gr::uwb::core::uwb_window_energy(in.data(), N, W, out.data());

    BOOST_CHECK_CLOSE(out[0], 1.0f, 1e-4);
    BOOST_CHECK_CLOSE(out[1], 2.0f, 1e-4);
    BOOST_CHECK_CLOSE(out[2], 3.0f, 1e-4);
    BOOST_CHECK_CLOSE(out[3], 4.0f, 1e-4);
}

BOOST_AUTO_TEST_CASE(test_constant_signal_energy)
{
    const size_t W = 64;
    const float A = 2.0f;
    const float expected = A * A;

    auto blk = gr::uwb::UwbEnergyDetector::make(expected, W);
    BOOST_CHECK_EQUAL(blk->window(), W);
    BOOST_CHECK_CLOSE(blk->threshold(), expected, 1e-3);

    // Change parameters at runtime and confirm they take effect.
    blk->set_window(32);
    BOOST_CHECK_EQUAL(blk->window(), 32);
    blk->set_window(W);

    const int N = 20000;
    std::vector<gr_complex> in(static_cast<size_t>(N), gr_complex(A, 0.0f));
    std::vector<float> metric;
    std::vector<unsigned char> flag;
    run_flowgraph(in, blk, metric, flag);

    BOOST_REQUIRE_EQUAL(metric.size(), static_cast<size_t>(N));
    BOOST_REQUIRE_EQUAL(flag.size(), static_cast<size_t>(N));

    // After the W-1 sample warm-up (zero-filled scheduler history) the trailing
    // window sees only amplitude A, so the metric is A^2 and the gate is set.
    for (size_t j = W; j + W < metric.size(); ++j) {
        BOOST_CHECK_CLOSE(metric[j], expected, 0.01);
        BOOST_CHECK_EQUAL(flag[j], 1);
    }
}

BOOST_AUTO_TEST_CASE(test_silence_below_threshold)
{
    const size_t W = 32;
    auto blk = gr::uwb::UwbEnergyDetector::make(1.0f, W); // threshold above silence

    const int N = 4096;
    std::vector<gr_complex> in(static_cast<size_t>(N), gr_complex(0.0f, 0.0f));
    std::vector<float> metric;
    std::vector<unsigned char> flag;
    run_flowgraph(in, blk, metric, flag);

    BOOST_REQUIRE_EQUAL(metric.size(), static_cast<size_t>(N));
    for (size_t j = 0; j < metric.size(); ++j) {
        BOOST_CHECK_SMALL(metric[j], 1e-6f);
        BOOST_CHECK_EQUAL(flag[j], 0);
    }
}
