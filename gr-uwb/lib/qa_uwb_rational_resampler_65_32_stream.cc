/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * QA for UwbRationalResamplerCcf65_32 (tagged stream).  Builds a real
 * flowgraph with a tagged native window, checks the per-window reset/flush
 * output against the raw core, and checks the radar tag mapping.
 */

#include <boost/test/unit_test.hpp>

#include <gnuradio/blocks/stream_to_tagged_stream.h>
#include <gnuradio/blocks/vector_sink.h>
#include <gnuradio/blocks/vector_source.h>
#include <gnuradio/tags.h>
#include <gnuradio/top_block.h>
#include <gnuradio/uwb/uwb_rational_resampler_ccf_65_32.h>
#include <gnuradio/uwb/uwb_rational_resampler_core.h>
#include <pmt/pmt.h>

#include <cmath>
#include <complex>
#include <cstdint>
#include <fstream>
#include <string>
#include <vector>

using gr::uwb::UwbRationalResamplerCcf65_32;
using gr::uwb::core::RationalResampler65_32Core;
using gr_complex = std::complex<float>;

namespace {

std::string find_path(const std::string& rel)
{
    const char* prefixes[] = { "", "../", "../../", "../../../", "../../../../" };
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

gr::tag_t make_tag(int64_t off, const char* key, pmt::pmt_t val)
{
    gr::tag_t t;
    t.offset = static_cast<uint64_t>(off);
    t.key = pmt::mp(key);
    t.value = val;
    t.srcid = pmt::mp("qa");
    return t;
}

} // namespace

BOOST_AUTO_TEST_CASE(tag_stream_65_32_matches_core_and_maps_tags)
{
    auto taps = load_f32(find_path("testdata/resampler_65_32/taps_quality_minorder.txt"));
    BOOST_REQUIRE(!taps.empty());

    const int L = 4096;
    std::vector<gr_complex> x(L);
    for (int i = 0; i < L; ++i)
        x[i] = gr_complex(std::sin(0.01f * i), std::cos(0.013f * i));

    std::vector<gr::tag_t> tags;
    tags.push_back(make_tag(0, "pulse_id", pmt::from_long(7)));
    tags.push_back(make_tag(0, "schedule_index", pmt::from_long(7)));
    tags.push_back(make_tag(0, "sample_rate", pmt::from_double(491.52e6)));
    tags.push_back(make_tag(0, "window_start_sample", pmt::from_long(0)));
    tags.push_back(make_tag(0, "pre_guard_samples", pmt::from_long(983)));
    tags.push_back(make_tag(0, "capture_samples", pmt::from_long(L - 983)));
    tags.push_back(make_tag(0, "post_guard_samples", pmt::from_long(0)));
    tags.push_back(make_tag(0, "sample_count", pmt::from_long(L)));
    tags.push_back(make_tag(0, "calibration_delay_native_samples",
                            pmt::from_double(334.0)));
    tags.push_back(make_tag(0, "sync_repetitions", pmt::from_long(64)));
    tags.push_back(make_tag(0, "sfd_mode", pmt::intern("4z2")));
    tags.push_back(make_tag(0, "code_index", pmt::from_long(9)));

    auto src = gr::blocks::vector_source_c::make(x, false, 1, tags);
    auto s2ts = gr::blocks::stream_to_tagged_stream::make(
        sizeof(gr_complex), 1, L, "packet_len");
    auto rs = UwbRationalResamplerCcf65_32::make_from_taps(
        taps, true, 1, "packet_len");
    auto snk = gr::blocks::vector_sink_c::make(1);

    auto tb = gr::make_top_block("qa_rs65_32_stream");
    tb->connect(src, 0, s2ts, 0);
    tb->connect(s2ts, 0, rs, 0);
    tb->connect(rs, 0, snk, 0);
    tb->run();

    RationalResampler65_32Core core(taps);
    const size_t exp = core.expected_output_length(static_cast<uint64_t>(L));
    BOOST_CHECK_EQUAL(snk->data().size(), exp);
    BOOST_CHECK_EQUAL(rs->windows(), 1u);
    BOOST_CHECK_EQUAL(rs->tag_errors(), 0u);

    // Per-window output must equal a fresh core process+flush.
    std::vector<gr_complex> ref(exp);
    core.reset();
    auto r = core.process(x.data(), static_cast<size_t>(L), ref.data(), exp);
    size_t produced = r.produced;
    if (produced < exp)
        produced += core.flush(ref.data() + produced, exp - produced);
    BOOST_REQUIRE_EQUAL(produced, exp);
    float maxd = 0.f;
    for (size_t i = 0; i < exp; ++i)
        maxd = std::max(maxd, std::abs(snk->data()[i] - ref[i]));
    BOOST_CHECK_LT(maxd, 2e-3f);

    // Tag mapping.
    auto find = [&](const char* key) -> pmt::pmt_t {
        for (const auto& t : snk->tags())
            if (pmt::eqv(t.key, pmt::mp(key)))
                return t.value;
        return pmt::PMT_NIL;
    };
    BOOST_CHECK_CLOSE(pmt::to_double(find("sample_rate")), 998.4e6, 1e-6);
    const int64_t ws = pmt::to_long(find("window_start_sample"));
    const int64_t pre = pmt::to_long(find("pre_guard_samples"));
    BOOST_CHECK_EQUAL(ws, core.map_input_offset_to_output(0));
    BOOST_CHECK_EQUAL(pre,
                      core.map_input_offset_to_output(983) -
                          core.map_input_offset_to_output(0));
    BOOST_CHECK_CLOSE(pmt::to_double(find("calibration_delay_work_samples")),
                      334.0 * 65.0 / 32.0, 1e-6);
    BOOST_CHECK_EQUAL(pmt::to_long(find("pulse_id")), 7);
    BOOST_CHECK_EQUAL(pmt::to_long(find("packet_len")),
                      static_cast<int64_t>(exp));
}
