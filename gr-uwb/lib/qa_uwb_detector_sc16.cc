#define BOOST_TEST_MODULE qa_uwb_detector_sc16
#include <boost/test/unit_test.hpp>

#include <gnuradio/blocks/message_debug.h>
#include <gnuradio/blocks/vector_source.h>
#include <gnuradio/top_block.h>
#include <gnuradio/uwb/uwb_detector_core.h>
#include <gnuradio/uwb/uwb_detector_sc16.h>

#include <cmath>
#include <complex>
#include <vector>

BOOST_AUTO_TEST_CASE(test_sc16_fixed_capture_and_bit_exact_pdu)
{
    constexpr size_t L = 128;
    constexpr size_t pre = 64;
    constexpr size_t capture = 5000;
    constexpr size_t packet_start = 2048;
    std::vector<gr_complex> tmpl(L);
    for (size_t k = 0; k < L; ++k)
        tmpl[k] = gr_complex(std::cos(0.21f * k), 0.4f * std::sin(0.13f * k));
    gr::uwb::core::uwb_l2_normalize(tmpl);

    std::vector<int16_t> interleaved((packet_start + capture + 4096) * 2, 0);
    for (size_t r = 0; r < 10; ++r) {
        for (size_t k = 0; k < L; ++k) {
            const size_t i = packet_start + r * L + k;
            interleaved[2 * i] = static_cast<int16_t>(
                std::lround(tmpl[k].real() * 20000.0f));
            interleaved[2 * i + 1] = static_cast<int16_t>(
                std::lround(tmpl[k].imag() * 20000.0f));
        }
    }

    auto src = gr::blocks::vector_source_s::make(interleaved, false, 2);
    auto det = gr::uwb::UwbDetectorSc16::make(
        tmpl, pre, capture, 1e-5f, 4, 4, 1, 8);
    auto dbg = gr::blocks::message_debug::make();
    auto tb = gr::make_top_block("qa_detector_sc16");
    tb->connect(src, 0, det, 0);
    tb->msg_connect(det, "packet", dbg, "store");
    tb->run();

    BOOST_REQUIRE_EQUAL(dbg->num_messages(), 1);
    const auto msg = dbg->get_message(0);
    const auto meta = pmt::car(msg);
    const auto data = pmt::cdr(msg);
    BOOST_REQUIRE(pmt::is_s16vector(data));
    const uint64_t start = pmt::to_uint64(
        pmt::dict_ref(meta, pmt::mp("start_sample"), pmt::PMT_NIL));
    BOOST_CHECK_EQUAL(start, packet_start);
    const auto iq = pmt::s16vector_elements(data);
    BOOST_REQUIRE_EQUAL(iq.size(), (pre + capture) * 2);
    const size_t begin = static_cast<size_t>(start - pre) * 2;
    for (size_t i = 0; i < iq.size(); ++i)
        BOOST_CHECK_EQUAL(iq[i], interleaved[begin + i]);
    BOOST_CHECK_EQUAL(det->dropped_regions(), 0);
}
