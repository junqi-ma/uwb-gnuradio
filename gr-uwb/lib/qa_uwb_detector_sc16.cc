#define BOOST_TEST_MODULE qa_uwb_detector_sc16
#include <boost/test/unit_test.hpp>

#include <gnuradio/blocks/message_debug.h>
#include <gnuradio/blocks/vector_source.h>
#include <gnuradio/top_block.h>
#include <gnuradio/uwb/uwb_detector_core.h>
#include <gnuradio/uwb/uwb_detector_sc16.h>

#include <cmath>
#include <complex>
#include <cstdint>
#include <vector>

namespace {

template <typename StateMachine>
std::vector<std::pair<uint64_t, size_t>> drain_regions(StateMachine& sm)
{
    std::vector<std::pair<uint64_t, size_t>> regions;
    while (sm.region_ready()) {
        const auto handle = sm.take_region();
        const auto& region = sm.region(handle);
        regions.emplace_back(region.start_abs, region.samples.size());
        sm.release_region(handle);
    }
    return regions;
}

} // namespace

BOOST_AUTO_TEST_CASE(test_sc16_integer_gate_matches_fc32_across_chunks)
{
    constexpr size_t n = 12000;
    constexpr size_t pre = 127;
    constexpr size_t D = 25;
    constexpr size_t window = 7;
    constexpr size_t holdoff = 3;
    constexpr float threshold = 0.02f;

    std::vector<std::complex<int16_t>> sc16(n, { 0, 0 });
    for (size_t i = 2100; i < 3900; ++i) {
        const int16_t re = static_cast<int16_t>(7000 + (i % 29) * 31);
        const int16_t im = static_cast<int16_t>(-5100 + (i % 17) * 23);
        sc16[i] = { re, im };
    }
    for (size_t i = 7300; i < 9100; ++i) {
        const int16_t re = static_cast<int16_t>(-8200 + (i % 31) * 19);
        const int16_t im = static_cast<int16_t>(4300 - (i % 13) * 37);
        sc16[i] = { re, im };
    }

    std::vector<gr_complex> fc32(n);
    constexpr float inv_full_scale = 1.0f / 32768.0f;
    for (size_t i = 0; i < n; ++i) {
        fc32[i] = { static_cast<float>(sc16[i].real()) * inv_full_scale,
                    static_cast<float>(sc16[i].imag()) * inv_full_scale };
    }

    gr::uwb::UwbDetectorStateMachine sm_f(pre, threshold, D, window, holdoff);
    gr::uwb::UwbDetectorStateMachineSc16 sm_s(pre, threshold, D, window, holdoff);
    const size_t chunks[] = { 1, 17, 113, 409, 3, 997, 64 };
    size_t offset = 0;
    size_t chunk_index = 0;
    std::vector<std::pair<uint64_t, size_t>> regions_f;
    std::vector<std::pair<uint64_t, size_t>> regions_s;
    while (offset < n) {
        const size_t count = std::min(chunks[chunk_index % 7], n - offset);
        sm_f.process(fc32.data() + offset, count, offset);
        sm_s.process(sc16.data() + offset, count, offset);
        const auto ready_f = drain_regions(sm_f);
        const auto ready_s = drain_regions(sm_s);
        regions_f.insert(regions_f.end(), ready_f.begin(), ready_f.end());
        regions_s.insert(regions_s.end(), ready_s.begin(), ready_s.end());
        offset += count;
        ++chunk_index;
    }
    sm_f.flush_region();
    sm_s.flush_region();
    const auto tail_f = drain_regions(sm_f);
    const auto tail_s = drain_regions(sm_s);
    regions_f.insert(regions_f.end(), tail_f.begin(), tail_f.end());
    regions_s.insert(regions_s.end(), tail_s.begin(), tail_s.end());

    BOOST_REQUIRE_EQUAL(regions_f.size(), 2);
    BOOST_REQUIRE_EQUAL(regions_s.size(), regions_f.size());
    for (size_t i = 0; i < regions_f.size(); ++i) {
        BOOST_CHECK_EQUAL(regions_s[i].first, regions_f[i].first);
        BOOST_CHECK_EQUAL(regions_s[i].second, regions_f[i].second);
    }
}

BOOST_AUTO_TEST_CASE(test_sc16_q15_coarse_peaks_match_fc32)
{
    constexpr size_t L = 128;
    constexpr size_t D = 4;
    constexpr size_t repetitions = 8;
    constexpr size_t prefix = 311;
    std::vector<gr_complex> tmpl(L);
    for (size_t k = 0; k < L; ++k)
        tmpl[k] = { std::cos(0.17f * k), 0.7f * std::sin(0.11f * k) };
    gr::uwb::core::uwb_l2_normalize(tmpl);

    std::vector<std::complex<int16_t>> signal_s(
        prefix + repetitions * L + 600, { 0, 0 });
    for (size_t r = 0; r < repetitions; ++r) {
        for (size_t k = 0; k < L; ++k) {
            const size_t i = prefix + r * L + k;
            signal_s[i] = {
                static_cast<int16_t>(std::lround(tmpl[k].real() * 22000.0f)),
                static_cast<int16_t>(std::lround(tmpl[k].imag() * 22000.0f))
            };
        }
    }
    std::vector<gr_complex> signal_f(signal_s.size());
    constexpr float inv = 1.0f / 32768.0f;
    for (size_t i = 0; i < signal_s.size(); ++i)
        signal_f[i] = { signal_s[i].real() * inv, signal_s[i].imag() * inv };

    std::vector<gr_complex> tmpl_f_ds;
    std::vector<std::complex<int16_t>> tmpl_s_ds;
    std::vector<std::complex<int16_t>> tmpl_s_imag_ds;
    for (size_t k = 0; k < L; k += D)
        tmpl_f_ds.push_back(tmpl[k]);
    gr::uwb::core::uwb_l2_normalize(tmpl_f_ds);
    for (const auto& sample : tmpl_f_ds) {
        tmpl_s_ds.emplace_back(
            static_cast<int16_t>(std::lround(sample.real() * 32767.0f)),
            static_cast<int16_t>(std::lround(sample.imag() * 32767.0f)));
    }
    for (const auto& sample : tmpl_s_ds)
        tmpl_s_imag_ds.emplace_back(static_cast<int16_t>(-sample.imag()),
                                    sample.real());

    std::vector<gr_complex> sig_f;
    std::vector<float> pow_f, score_f, metric_f;
    std::vector<std::complex<int16_t>> sig_s;
    std::vector<uint64_t> pow_s;
    std::vector<float> score_s, metric_s;
    std::vector<size_t> peaks_f, peaks_s;
    float max_f = 0.0f;
    float max_s = 0.0f;
    gr::uwb::core::uwb_coarse_peaks(
        signal_f.data(), 0, signal_f.size(), tmpl_f_ds.data(), tmpl_f_ds.size(),
        D, 1, L / D, 0.5f, 0.5f, 1, sig_f, pow_f, score_f, metric_f,
        peaks_f, &max_f);
    gr::uwb::core::uwb_coarse_peaks_sc16(
        signal_s.data(), 0, signal_s.size(), tmpl_s_ds.data(),
        tmpl_s_imag_ds.data(), tmpl_s_ds.size(), D, 1, L / D, 0.5f, 0.5f, 1,
        sig_s, pow_s, score_s, metric_s,
        peaks_s, &max_s);

    BOOST_REQUIRE(!peaks_f.empty());
    BOOST_REQUIRE_EQUAL(peaks_s.size(), peaks_f.size());
    for (size_t i = 0; i < peaks_f.size(); ++i)
        BOOST_CHECK_EQUAL(peaks_s[i], peaks_f[i]);
    BOOST_CHECK_CLOSE(std::sqrt(max_s), max_f, 0.2f);
}

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
