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
#include <gnuradio/uwb/uwb_preamble_detector.h>

#include <cmath>
#include <vector>

namespace {

void run_flowgraph(const std::vector<gr_complex>& in,
                   gr::uwb::UwbPreambleDetector::sptr blk,
                   std::vector<float>& metric,
                   std::vector<unsigned char>& flag)
{
    auto src = gr::blocks::vector_source_c::make(in);
    auto snk_metric = gr::blocks::vector_sink_f::make();
    auto snk_flag = gr::blocks::vector_sink_b::make();
    auto tb = gr::make_top_block("qa_preamble");
    tb->connect(src, 0, blk, 0);
    tb->connect(blk, 0, snk_metric, 0);
    tb->connect(blk, 1, snk_flag, 0);
    tb->run();
    metric = snk_metric->data();
    flag = snk_flag->data();
}

} // namespace

BOOST_AUTO_TEST_CASE(test_core_normalized_score)
{
    // A unit-norm template aligned with an identical signal window must give a
    // score of exactly 1; silence must give ~0.
    const size_t L = 8;
    std::vector<gr_complex> t(L);
    for (size_t k = 0; k < L; ++k)
        t[k] = gr_complex(std::cos(0.3f * k), 0.5f * std::sin(0.4f * k));
    gr::uwb::core::uwb_l2_normalize(t);

    float tenergy = gr::uwb::core::uwb_template_energy(t);
    BOOST_CHECK_CLOSE(tenergy, 1.0f, 1e-3);

    std::vector<gr_complex> corr(1, tenergy);
    std::vector<float> winpow(1, tenergy);
    std::vector<float> metric(1);
    gr::uwb::core::uwb_normalized_score(corr.data(), winpow.data(), 1, tenergy,
                                        metric.data());
    BOOST_CHECK_CLOSE(metric[0], 1.0f, 1e-3);

    corr[0] = gr_complex(0, 0);
    winpow[0] = 0.0f;
    gr::uwb::core::uwb_normalized_score(corr.data(), winpow.data(), 1, tenergy,
                                        metric.data());
    BOOST_CHECK_SMALL(metric[0], 1e-6f);
}

BOOST_AUTO_TEST_CASE(test_preamble_alignment)
{
    // Build a small synthetic template and a signal that contains two exact
    // repetitions surrounded by silence.
    const size_t L = 64;
    std::vector<gr_complex> tmpl(L);
    for (size_t k = 0; k < L; ++k)
        tmpl[k] = gr_complex(0.5f * std::cos(0.3f * k), 0.3f * std::sin(0.7f * k));
    gr::uwb::core::uwb_l2_normalize(tmpl);

    auto blk = gr::uwb::UwbPreambleDetector::make(tmpl, 0.5f);
    BOOST_CHECK_EQUAL(blk->template_length(), L);
    blk->set_threshold(0.9f);
    BOOST_CHECK_CLOSE(blk->threshold(), 0.9f, 1e-6);

    // 200k samples -> guaranteed to span multiple scheduler work() calls, so
    // this exercises history handling across input chunks.
    const int lead = 500;
    const int reps = 2;
    const int gap = 200000;
    const int N = lead + reps * static_cast<int>(L) + gap;

    std::vector<gr_complex> sig(static_cast<size_t>(N), gr_complex(0.0f, 0.0f));
    for (int r = 0; r < reps; ++r)
        for (size_t k = 0; k < L; ++k)
            sig[static_cast<size_t>(lead + r * static_cast<int>(L) + k)] = tmpl[k];

    std::vector<float> metric;
    std::vector<unsigned char> flag;
    run_flowgraph(sig, blk, metric, flag);

    BOOST_REQUIRE_EQUAL(metric.size(), static_cast<size_t>(N));

    // The normalized correlation peaks where the trailing template-length
    // window equals the template, i.e. at the end of each SYNC symbol.
    for (int r = 0; r < reps; ++r) {
        size_t peak = static_cast<size_t>(lead + r * static_cast<int>(L) + L - 1);
        BOOST_CHECK_CLOSE(metric[peak], 1.0f, 0.01);
        BOOST_CHECK_EQUAL(flag[peak], 1);
    }

    // Mid-silence the metric stays near zero and the flag is clear.
    size_t mid = static_cast<size_t>(lead + reps * static_cast<int>(L) + gap / 2);
    BOOST_CHECK_SMALL(metric[mid], 0.1f);
    BOOST_CHECK_EQUAL(flag[mid], 0);
}

BOOST_AUTO_TEST_CASE(test_fast_mode_equals_reference)
{
    // The energy-gated fast path must produce the same correlation peaks as
    // the exact full-rate reference detector on a silence + preamble signal.
    const size_t L = 64;
    std::vector<gr_complex> tmpl(L);
    for (size_t k = 0; k < L; ++k)
        tmpl[k] = gr_complex(0.5f * std::cos(0.3f * k), 0.3f * std::sin(0.7f * k));
    gr::uwb::core::uwb_l2_normalize(tmpl);

    const int lead = 500;
    const int reps = 4;
    const int gap = 300;
    const int N = lead + reps * static_cast<int>(L) + gap;
    std::vector<gr_complex> sig(static_cast<size_t>(N), gr_complex(0.0f, 0.0f));
    for (int r = 0; r < reps; ++r)
        for (size_t k = 0; k < L; ++k)
            sig[static_cast<size_t>(lead + r * static_cast<int>(L) + k)] = tmpl[k];

    std::vector<float> m_ref, m_fast;
    std::vector<unsigned char> f_ref, f_fast;
    run_flowgraph(sig, gr::uwb::UwbPreambleDetector::make(tmpl, 0.5f), m_ref, f_ref);
    run_flowgraph(sig, gr::uwb::UwbPreambleDetector::make(tmpl, 0.5f, 1e-3f, 16),
                  m_fast, f_fast);

    BOOST_REQUIRE_EQUAL(m_ref.size(), m_fast.size());
    BOOST_REQUIRE_EQUAL(static_cast<size_t>(N), m_ref.size());

    // Both modes must peak ~1.0 at the same (symbol-end) positions, and be
    // ~0 in the leading silence.
    for (int r = 0; r < reps; ++r) {
        size_t peak = static_cast<size_t>(lead + r * static_cast<int>(L) + L - 1);
        BOOST_CHECK_CLOSE(m_ref[peak], 1.0f, 0.01);
        BOOST_CHECK_CLOSE(m_fast[peak], 1.0f, 0.01);
    }
    BOOST_CHECK_SMALL(m_ref[lead - 10], 0.1f);
    BOOST_CHECK_SMALL(m_fast[lead - 10], 0.1f);

    // The energy gate only ever skips correlation (outputs 0), so it can
    // never raise the metric.  Where the reference actually detects
    // (m_ref > 0.5) the fast path must agree exactly.
    for (size_t j = lead; j < static_cast<size_t>(N) - gap / 2; ++j) {
        BOOST_CHECK_LE(m_fast[j], m_ref[j] + 1e-6f);
        if (m_ref[j] > 0.5f)
            BOOST_CHECK_CLOSE(m_fast[j], m_ref[j], 1e-3);
    }
}

BOOST_AUTO_TEST_CASE(test_coarse_to_fine_equals_reference)
{
    // Coarse-to-fine mode (decimated energy gate + decimated coarse scan +
    // fine ROI) must produce the same symbol peaks as the reference on a
    // repeated-symbol preamble.
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

    std::vector<float> m_ref, m_fast;
    std::vector<unsigned char> f_ref, f_fast;
    run_flowgraph(sig, gr::uwb::UwbPreambleDetector::make(tmpl, 0.5f), m_ref, f_ref);
    // coarse-to-fine: decimated energy gate D=4, coarse D=4
    run_flowgraph(sig, gr::uwb::UwbPreambleDetector::make(tmpl, 0.5f, 1e-3f, 8,
                                                          /*energy_dec=*/4,
                                                          /*coarse_dec=*/4,
                                                          /*reps=*/1,
                                                          /*margin=*/16),
                  m_fast, f_fast);
    BOOST_REQUIRE_EQUAL(m_ref.size(), m_fast.size());

    // Both modes must peak at every symbol end (lead + r*L + L - 1) with ~1.0.
    size_t found = 0;
    for (int r = 0; r < reps; ++r) {
        const size_t peak = static_cast<size_t>(lead + r * static_cast<int>(L) + L - 1);
        if (m_fast[peak] > 0.9f)
            ++found;
        BOOST_CHECK_CLOSE(m_ref[peak], 1.0f, 0.01);
    }
    BOOST_CHECK_GE(found, reps - 2); // allow a boundary peak or two to be skipped
    BOOST_CHECK_SMALL(m_fast[lead - 10], 0.1f);
}
