/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Block / scheduler / tag QA for UwbRationalResamplerCcf65_48.
 * Instantiates real flowgraphs (work() is protected).
 */

#include <boost/test/unit_test.hpp>

#include <gnuradio/blocks/head.h>
#include <gnuradio/blocks/null_sink.h>
#include <gnuradio/blocks/tag_debug.h>
#include <gnuradio/blocks/vector_sink.h>
#include <gnuradio/blocks/vector_source.h>
#include <gnuradio/top_block.h>
#include <gnuradio/uwb/uwb_rational_resampler_ccf_65_48.h>
#include <gnuradio/uwb/uwb_rational_resampler_core.h>
#include <pmt/pmt.h>

#include <cmath>
#include <complex>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <random>
#include <string>
#include <vector>

using gr::uwb::UwbRationalResamplerCcf65_48;
using gr::uwb::core::RationalResampler65_48Core;
using gr_complex = std::complex<float>;

namespace {

constexpr float kTolAbs = 2e-3f;

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

const std::vector<float>& quality_taps()
{
    static std::vector<float> taps =
        load_f32(find_path("testdata/resampler_65_48/taps_quality.txt"));
    return taps;
}

std::vector<gr_complex> core_oneshot(const std::vector<gr_complex>& x,
                                     const std::vector<float>& taps)
{
    RationalResampler65_48Core core(taps);
    const size_t Lout =
        RationalResampler65_48Core::expected_output_length(x.size(),
                                                           taps.size());
    std::vector<gr_complex> y(Lout + 128);
    size_t produced = 0;
    if (!x.empty()) {
        auto r = core.process(x.data(), x.size(), y.data(), y.size());
        produced = r.produced;
    }
    while (true) {
        if (produced >= y.size())
            y.resize(produced + 256);
        size_t n = core.flush(y.data() + produced, y.size() - produced);
        if (n == 0)
            break;
        produced += n;
    }
    y.resize(produced);
    return y;
}

std::vector<gr_complex> core_chunked(const std::vector<gr_complex>& x,
                                     const std::vector<float>& taps,
                                     const std::vector<size_t>& chunks)
{
    RationalResampler65_48Core core(taps);
    const size_t Lout =
        RationalResampler65_48Core::expected_output_length(x.size(),
                                                           taps.size());
    std::vector<gr_complex> y;
    y.reserve(Lout);
    size_t off = 0;
    size_t ci = 0;
    while (off < x.size()) {
        size_t ch = (ci < chunks.size()) ? chunks[ci++] : 48;
        if (ch == 0)
            ch = 1;
        ch = std::min(ch, x.size() - off);
        std::vector<gr_complex> tmp(ch * 2 + 128);
        auto r = core.process(x.data() + off, ch, tmp.data(), tmp.size());
        for (size_t i = 0; i < r.produced; ++i)
            y.push_back(tmp[i]);
        BOOST_REQUIRE_EQUAL(r.consumed, ch);
        off += r.consumed;
    }
    std::vector<gr_complex> tail(4096);
    size_t n;
    do {
        n = core.flush(tail.data(), tail.size());
        for (size_t i = 0; i < n; ++i)
            y.push_back(tail[i]);
    } while (n > 0);
    return y;
}

float max_abs_diff(const std::vector<gr_complex>& a,
                   const std::vector<gr_complex>& b)
{
    BOOST_REQUIRE_EQUAL(a.size(), b.size());
    float m = 0.0f;
    for (size_t i = 0; i < a.size(); ++i) {
        m = std::max(m, std::abs(a[i] - b[i]));
    }
    return m;
}

std::vector<gr_complex> run_block_flowgraph(const std::vector<gr_complex>& x,
                                            const std::vector<float>& taps,
                                            bool tag_prop = true)
{
    auto src = gr::blocks::vector_source_c::make(x, false, 1);
    auto res =
        UwbRationalResamplerCcf65_48::make_from_taps(taps, tag_prop, true);
    auto sink = gr::blocks::vector_sink_c::make();
    auto tb = gr::make_top_block("qa_resampler");
    tb->connect(src, 0, res, 0);
    tb->connect(res, 0, sink, 0);
    tb->run();
    return sink->data();
}

} // namespace

// ---------------------------------------------------------------------------
// Chunk invariance (core)
// ---------------------------------------------------------------------------

BOOST_AUTO_TEST_CASE(test_chunk_invariance_oneshot_vs_splits)
{
    const auto& taps = quality_taps();
    const auto x = load_cf32(
        find_path("testdata/resampler_65_48/golden/impulse_in.cf32"));
    const auto y0 = core_oneshot(x, taps);

    auto y48 = core_chunked(x, taps, std::vector<size_t>(200, 48));
    BOOST_CHECK_EQUAL(y48.size(), y0.size());
    BOOST_CHECK_LE(max_abs_diff(y0, y48), 0.0f); // exact (same scalar path)

    auto y4096 = core_chunked(x, taps, { 4096 });
    BOOST_CHECK_EQUAL(y4096.size(), y0.size());
    BOOST_CHECK_LE(max_abs_diff(y0, y4096), 0.0f);

    std::vector<size_t> mixed = { 1, 2, 47, 48, 49 };
    std::mt19937 rng(42);
    std::uniform_int_distribution<size_t> dist(1, 200);
    while (true) {
        size_t sum = 0;
        for (auto c : mixed)
            sum += c;
        if (sum >= x.size())
            break;
        mixed.push_back(dist(rng));
    }
    auto ymix = core_chunked(x, taps, mixed);
    BOOST_CHECK_EQUAL(ymix.size(), y0.size());
    BOOST_CHECK_LE(max_abs_diff(y0, ymix), 0.0f);

    std::cout << "chunk_invariance: oneshot/48/4096/mixed exact match, N_out="
              << y0.size() << "\n";
}

// ---------------------------------------------------------------------------
// Block vs core / golden
// ---------------------------------------------------------------------------

BOOST_AUTO_TEST_CASE(test_block_matches_core_and_golden)
{
    const auto& taps = quality_taps();
    const auto x = load_cf32(
        find_path("testdata/resampler_65_48/golden/random_in.cf32"));
    const auto ref = load_cf32(
        find_path("testdata/resampler_65_48/golden/random_out.cf32"));

    const auto y_core = core_oneshot(x, taps);
    const auto y_blk = run_block_flowgraph(x, taps);

    BOOST_CHECK_EQUAL(y_core.size(), ref.size());
    BOOST_CHECK_EQUAL(y_blk.size(), ref.size());
    BOOST_CHECK_LE(max_abs_diff(y_core, ref), kTolAbs);
    BOOST_CHECK_LE(max_abs_diff(y_blk, ref), kTolAbs);
    BOOST_CHECK_LE(max_abs_diff(y_blk, y_core), kTolAbs);

    std::cout << "block_vs_golden random: max_abs(block,ref)="
              << max_abs_diff(y_blk, ref) << "\n";
}

BOOST_AUTO_TEST_CASE(test_block_eos_finite_source)
{
    const auto& taps = quality_taps();
    // Awkward length not a multiple of 48
    std::vector<gr_complex> x(1000);
    for (size_t i = 0; i < x.size(); ++i)
        x[i] = gr_complex(0.1f * std::sin(0.02f * i), 0.05f);

    const auto y_core = core_oneshot(x, taps);
    const auto y_blk = run_block_flowgraph(x, taps);
    BOOST_CHECK_EQUAL(y_blk.size(), y_core.size());
    BOOST_CHECK_LE(max_abs_diff(y_blk, y_core), kTolAbs);
}

BOOST_AUTO_TEST_CASE(test_block_stats_and_reset)
{
    const auto& taps = quality_taps();
    auto res =
        UwbRationalResamplerCcf65_48::make_from_taps(taps, false, true);
    std::vector<gr_complex> x(480, gr_complex(1, 0));
    auto src = gr::blocks::vector_source_c::make(x);
    auto sink = gr::blocks::null_sink::make(sizeof(gr_complex));
    auto tb = gr::make_top_block("qa_stats");
    tb->connect(src, 0, res, 0);
    tb->connect(res, 0, sink, 0);
    tb->run();
    BOOST_CHECK_EQUAL(res->input_items(), x.size());
    BOOST_CHECK_GT(res->output_items(), 0u);
    BOOST_CHECK(!std::string(res->kernel_name()).empty());
    BOOST_CHECK_EQUAL(res->tap_count(), taps.size());

    res->reset();
    BOOST_CHECK_GE(res->resets(), 1u);
}

BOOST_AUTO_TEST_CASE(test_block_forecast_no_livelock_small_buffer)
{
    const auto& taps = quality_taps();
    std::vector<gr_complex> x(500, gr_complex(0.25f, -0.1f));
    auto src = gr::blocks::vector_source_c::make(x);
    auto res =
        UwbRationalResamplerCcf65_48::make_from_taps(taps, false, true);
    auto sink = gr::blocks::vector_sink_c::make();
    auto tb = gr::make_top_block("qa_smallbuf");
    // Small max noutput to stress scheduler.
    res->set_max_noutput_items(65 * 2);
    tb->connect(src, 0, res, 0);
    tb->connect(res, 0, sink, 0);
    tb->run(); // must complete (no livelock)
    const auto y_core = core_oneshot(x, taps);
    BOOST_CHECK_EQUAL(sink->data().size(), y_core.size());
}

BOOST_AUTO_TEST_CASE(test_block_stop_restart)
{
    const auto& taps = quality_taps();
    std::vector<gr_complex> x(200, gr_complex(1, 0));
    auto res =
        UwbRationalResamplerCcf65_48::make_from_taps(taps, false, true);

    {
        auto src = gr::blocks::vector_source_c::make(x);
        auto sink = gr::blocks::null_sink::make(sizeof(gr_complex));
        auto tb = gr::make_top_block("qa_r1");
        tb->connect(src, 0, res, 0);
        tb->connect(res, 0, sink, 0);
        tb->run();
    }
    res->reset();
    {
        auto src = gr::blocks::vector_source_c::make(x);
        auto sink = gr::blocks::null_sink::make(sizeof(gr_complex));
        auto tb = gr::make_top_block("qa_r2");
        tb->connect(src, 0, res, 0);
        tb->connect(res, 0, sink, 0);
        tb->run();
    }
    BOOST_CHECK_GE(res->resets(), 1u);
}

// ---------------------------------------------------------------------------
// Tag / rx_time QA
// ---------------------------------------------------------------------------

BOOST_AUTO_TEST_CASE(test_tag_offset_mapping_law)
{
    const auto& taps = quality_taps();
    RationalResampler65_48Core core(taps);
    // Group delay d = (T-1)/2 = 2681 virtual → ~55.85 output samples
    const int64_t offsets[] = { 0, 1, 47, 48, 49, 100, 1000 };
    for (int64_t p : offsets) {
        const int64_t m = core.map_input_offset_to_output(p);
        const double d = 0.5 * (taps.size() - 1);
        const int64_t expect = static_cast<int64_t>(
            std::llround((p * 65.0 + d) / 48.0));
        BOOST_CHECK_EQUAL(m, expect < 0 ? 0 : expect);
    }
    std::cout << "tag_map p=0 -> " << core.map_input_offset_to_output(0)
              << " (expect ~56 for quality group delay)\n";
}

BOOST_AUTO_TEST_CASE(test_tag_rx_time_propagation)
{
    const auto& taps = quality_taps();
    const size_t N = 500;
    std::vector<gr_complex> x(N, gr_complex(0.5f, 0.0f));

    // Place tags at several input offsets
    const uint64_t tag_offs[] = { 0, 1, 47, 48, 49 };
    std::vector<gr::tag_t> tags;
    for (uint64_t off : tag_offs) {
        gr::tag_t t;
        t.offset = off;
        t.key = pmt::intern("rx_time");
        t.value = pmt::make_tuple(pmt::from_uint64(100 + off),
                                  pmt::from_double(0.125));
        t.srcid = pmt::intern("qa");
        tags.push_back(t);

        gr::tag_t tr;
        tr.offset = off;
        tr.key = pmt::intern("rx_rate");
        tr.value = pmt::from_double(737.28e6);
        tr.srcid = pmt::intern("qa");
        tags.push_back(tr);
    }

    auto src = gr::blocks::vector_source_c::make(x, false, 1, tags);
    auto res =
        UwbRationalResamplerCcf65_48::make_from_taps(taps, true, true);
    auto sink = gr::blocks::vector_sink_c::make();
    auto tb = gr::make_top_block("qa_tags");
    tb->connect(src, 0, res, 0);
    tb->connect(res, 0, sink, 0);
    tb->run();

    const auto& out_tags = sink->tags();
    const size_t n_tag_offs = sizeof(tag_offs) / sizeof(tag_offs[0]);
    BOOST_CHECK_GE(out_tags.size(), n_tag_offs); // at least the rx_times

    size_t rx_time_count = 0;
    size_t rx_rate_ok = 0;
    for (const auto& t : out_tags) {
        if (pmt::symbol_to_string(t.key) == "rx_time") {
            ++rx_time_count;
            // Value should be unchanged (tuple still present)
            BOOST_CHECK(pmt::is_tuple(t.value));
        }
        if (pmt::symbol_to_string(t.key) == "rx_rate") {
            BOOST_CHECK_CLOSE(pmt::to_double(t.value), 998.4e6, 1e-6);
            ++rx_rate_ok;
        }
    }
    BOOST_CHECK_EQUAL(rx_time_count, n_tag_offs);
    BOOST_CHECK_EQUAL(rx_rate_ok, rx_time_count);

    // Check mapped offsets roughly follow the law for p=0,48
    RationalResampler65_48Core core(taps);
    for (const auto& t : out_tags) {
        if (pmt::symbol_to_string(t.key) != "rx_time")
            continue;
        // Recover input offset from tuple first element (100+off)
        const uint64_t marker =
            pmt::to_uint64(pmt::tuple_ref(t.value, 0));
        const int64_t p = static_cast<int64_t>(marker - 100);
        const int64_t expect = core.map_input_offset_to_output(p);
        // Allow clamping at 0 for early tags
        BOOST_CHECK_MESSAGE(
            static_cast<int64_t>(t.offset) == expect ||
                (expect < 0 && t.offset == 0) ||
                (static_cast<int64_t>(t.offset) == 0 && expect >= 0 &&
                 expect < 65),
            "tag p=" + std::to_string(p) + " out_off=" +
                std::to_string(t.offset) + " expect=" +
                std::to_string(expect));
    }
    std::cout << "tag_rx_time: " << rx_time_count
              << " tags propagated, rx_rate rewritten to 998.4e6\n";
}

BOOST_AUTO_TEST_CASE(test_tag_discontinuity_resets_core)
{
    const auto& taps = quality_taps();
    const size_t N = 400;
    std::vector<gr_complex> x(N, gr_complex(1.0f, 0.0f));

    std::vector<gr::tag_t> tags;
    gr::tag_t t;
    t.offset = 100;
    t.key = pmt::intern("overflow");
    t.value = pmt::PMT_T;
    t.srcid = pmt::intern("qa");
    tags.push_back(t);

    auto src = gr::blocks::vector_source_c::make(x, false, 1, tags);
    auto res =
        UwbRationalResamplerCcf65_48::make_from_taps(taps, true, true);
    auto sink = gr::blocks::vector_sink_c::make();
    auto tb = gr::make_top_block("qa_disc");
    tb->connect(src, 0, res, 0);
    tb->connect(res, 0, sink, 0);
    tb->run();

    BOOST_CHECK_GE(res->discontinuities(), 1u);
    BOOST_CHECK_GE(res->resets(), 1u);

    bool saw_reset = false;
    bool saw_overflow = false;
    for (const auto& ot : sink->tags()) {
        const std::string k = pmt::symbol_to_string(ot.key);
        if (k == "resampler_reset")
            saw_reset = true;
        if (k == "overflow")
            saw_overflow = true;
    }
    BOOST_CHECK(saw_reset);
    BOOST_CHECK(saw_overflow);
    std::cout << "discontinuity: resets=" << res->resets()
              << " disc=" << res->discontinuities() << "\n";
}

BOOST_AUTO_TEST_CASE(test_tag_propagation_disabled_drops_tags)
{
    const auto& taps = quality_taps();
    std::vector<gr_complex> x(100, gr_complex(1, 0));
    std::vector<gr::tag_t> tags;
    gr::tag_t t;
    t.offset = 0;
    t.key = pmt::intern("rx_time");
    t.value = pmt::from_long(1);
    t.srcid = pmt::intern("qa");
    tags.push_back(t);

    auto src = gr::blocks::vector_source_c::make(x, false, 1, tags);
    auto res =
        UwbRationalResamplerCcf65_48::make_from_taps(taps, /*tag_prop=*/false,
                                                     true);
    auto sink = gr::blocks::vector_sink_c::make();
    auto tb = gr::make_top_block("qa_notag");
    tb->connect(src, 0, res, 0);
    tb->connect(res, 0, sink, 0);
    tb->run();
    BOOST_CHECK_EQUAL(sink->tags().size(), 0u);
}

BOOST_AUTO_TEST_CASE(test_two_rx_time_epochs)
{
    const auto& taps = quality_taps();
    std::vector<gr_complex> x(300, gr_complex(0.2f, 0.1f));
    std::vector<gr::tag_t> tags;
    for (uint64_t off : { 0ull, 150ull }) {
        gr::tag_t t;
        t.offset = off;
        t.key = pmt::intern("rx_time");
        t.value = pmt::make_tuple(pmt::from_uint64(off == 0 ? 1 : 2),
                                  pmt::from_double(0.0));
        t.srcid = pmt::intern("qa");
        tags.push_back(t);
    }
    auto src = gr::blocks::vector_source_c::make(x, false, 1, tags);
    auto res =
        UwbRationalResamplerCcf65_48::make_from_taps(taps, true, true);
    auto sink = gr::blocks::vector_sink_c::make();
    auto tb = gr::make_top_block("qa_epochs");
    tb->connect(src, 0, res, 0);
    tb->connect(res, 0, sink, 0);
    tb->run();

    size_t n_rx = 0;
    for (const auto& t : sink->tags()) {
        if (pmt::symbol_to_string(t.key) == "rx_time")
            ++n_rx;
    }
    BOOST_CHECK_EQUAL(n_rx, 2u);
}

// ---------------------------------------------------------------------------
// Multi-worker block: output == single-threaded (oneshot + chunked EOS)
// ---------------------------------------------------------------------------

BOOST_AUTO_TEST_CASE(test_block_multiworker_matches_single)
{
    const auto& taps = quality_taps();
    const auto x = load_cf32(
        find_path("testdata/resampler_65_48/golden/random_in.cf32"));

    const auto y1 = run_block_flowgraph(x, taps);

    auto src = gr::blocks::vector_source_c::make(x, false, 1);
    auto res = UwbRationalResamplerCcf65_48::make_from_taps(
        taps, /*tag_prop=*/false, /*reset=*/true, /*num_workers=*/8);
    BOOST_CHECK_EQUAL(res->num_workers(), 8);
    auto sink = gr::blocks::vector_sink_c::make();
    auto tb = gr::make_top_block("qa_mt");
    tb->connect(src, 0, res, 0);
    tb->connect(res, 0, sink, 0);
    tb->run();
    const auto y8 = sink->data();

    BOOST_REQUIRE_EQUAL(y1.size(), y8.size());
    float max_e = max_abs_diff(y1, y8);
    // Core multi-worker is bitwise-exact; block flowgraph may differ by a few
    // ulps if GR grant sizes select MT vs macroblock paths with different
    // float reduction order across kernels.  Require << golden tol.
    BOOST_CHECK_MESSAGE(max_e < 1e-5f,
                        "block workers=1 vs 8 max_abs=" +
                            std::to_string(max_e));

    // EOS tail length matches core oneshot.
    const auto y_core = core_oneshot(x, taps);
    BOOST_CHECK_EQUAL(y8.size(), y_core.size());
    BOOST_CHECK_LE(max_abs_diff(y8, y_core), kTolAbs);
    std::cout << "multiworker block: N_out=" << y8.size()
              << " max_abs(w1,w8)=" << max_e << "\n";
}

BOOST_AUTO_TEST_CASE(test_block_multiworker_discontinuity_reset)
{
    const auto& taps = quality_taps();
    const size_t N = 800;
    std::vector<gr_complex> x(N, gr_complex(1.0f, 0.0f));
    std::vector<gr::tag_t> tags;
    gr::tag_t t;
    t.offset = 200;
    t.key = pmt::intern("overflow");
    t.value = pmt::PMT_T;
    t.srcid = pmt::intern("qa");
    tags.push_back(t);

    auto src = gr::blocks::vector_source_c::make(x, false, 1, tags);
    auto res = UwbRationalResamplerCcf65_48::make_from_taps(
        taps, true, true, /*num_workers=*/4);
    auto sink = gr::blocks::vector_sink_c::make();
    auto tb = gr::make_top_block("qa_mt_disc");
    tb->connect(src, 0, res, 0);
    tb->connect(res, 0, sink, 0);
    tb->run();

    BOOST_CHECK_GE(res->discontinuities(), 1u);
    BOOST_CHECK_GE(res->resets(), 1u);
    // Must complete without hang; output non-empty after reset.
    BOOST_CHECK_GT(sink->data().size(), 0u);
}
