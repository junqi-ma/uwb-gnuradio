/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * QA for UwbEchoTimerStream (tagged-stream EchoTimer, Radar Stage 2).
 * Builds a real flowgraph with a repeated tagged TX packet and a
 * FakeBurstBackend (no hardware): asserts the RX window length per frame,
 * the radar metadata tags on every window, the SC16 RX payload, the fake
 * backend send/recv counters and the block burst counters.
 */

#include <boost/test/unit_test.hpp>

#include <gnuradio/blocks/head.h>
#include <gnuradio/blocks/stream_to_tagged_stream.h>
#include <gnuradio/blocks/vector_sink.h>
#include <gnuradio/blocks/vector_source.h>
#include <gnuradio/tags.h>
#include <gnuradio/top_block.h>
#include <gnuradio/uwb/uwb_echo_burst_backend.h>
#include <gnuradio/uwb/uwb_echo_scheduler_core.h>
#include <gnuradio/uwb/uwb_echo_timer_stream.h>
#include <gnuradio/uwb/uwb_fake_burst_backend.h>
#include <pmt/pmt.h>

#include <cmath>
#include <complex>
#include <cstdint>
#include <string>
#include <vector>

using gr::uwb::UwbEchoTimerStream;
namespace echo = gr::uwb::echo;
using gr_complex = std::complex<float>;

namespace {

constexpr uint64_t kTxSamples = 64;
constexpr uint64_t kRxSamples = 48;
constexpr uint64_t kFrames = 4;
constexpr int64_t kT0Ticks = 1000000;
constexpr int64_t kPreGuardTicks = 1475;
constexpr double kSampleRateHz = 737280000.0;
constexpr uint64_t kPreGuardSamples = 1475;
constexpr uint64_t kCaptureSamples = 1000;
constexpr uint64_t kPostGuardSamples = 200;
constexpr double kCalDelay = 334.0;

echo::EchoSchedulerConfig base_grid_cfg()
{
    echo::EchoSchedulerConfig cfg;
    cfg.pri_num = 3686400; // 5 ms @ 737.28 MS/s
    cfg.pri_den = 1;
    cfg.pre_guard_ticks = kPreGuardTicks;
    cfg.max_fragment_size = 4096;
    return cfg;
}

pmt::pmt_t find_tag(const std::vector<gr::tag_t>& tags,
                    const char* key,
                    uint64_t offset)
{
    for (const auto& t : tags) {
        if (t.offset == offset && pmt::eqv(t.key, pmt::mp(key)))
            return t.value;
    }
    return pmt::PMT_NIL;
}

size_t count_tag(const std::vector<gr::tag_t>& tags, const char* key)
{
    size_t n = 0;
    for (const auto& t : tags)
        if (pmt::eqv(t.key, pmt::mp(key)))
            ++n;
    return n;
}

int64_t tag_i64(pmt::pmt_t v)
{
    if (pmt::is_uint64(v))
        return static_cast<int64_t>(pmt::to_uint64(v));
    if (pmt::is_integer(v))
        return static_cast<int64_t>(pmt::to_long(v));
    return 0;
}

} // namespace

BOOST_AUTO_TEST_CASE(test_echo_timer_stream_flowgraph)
{
    // Repeated single-packet TX tagged stream.
    std::vector<gr_complex> tx(static_cast<size_t>(kTxSamples));
    for (size_t i = 0; i < tx.size(); ++i)
        tx[i] = gr_complex(0.5f * std::sin(0.1f * i),
                           0.25f * std::cos(0.13f * i));

    auto fake = std::make_shared<echo::FakeBurstBackend>(
        echo::FakeBurstBackend::Config{});

    auto blk = UwbEchoTimerStream::make(base_grid_cfg(),
                                        fake,
                                        kTxSamples,
                                        kRxSamples,
                                        kSampleRateHz,
                                        kPreGuardSamples,
                                        kCaptureSamples,
                                        kPostGuardSamples,
                                        /*sync_repetitions=*/64,
                                        "4z2",
                                        /*code_index=*/9,
                                        kCalDelay,
                                        kT0Ticks,
                                        /*arm_margin_s=*/0.0,
                                        kFrames,
                                        /*collect_wait_ms=*/1000,
                                        static_cast<size_t>(kTxSamples),
                                        static_cast<size_t>(kRxSamples),
                                        "packet_len");
    BOOST_REQUIRE(blk);

    // Thread-safe tune/calibration accessors (before the flow starts).
    blk->set_freq(8.1e9);
    BOOST_CHECK_CLOSE(blk->freq(), 8.1e9, 1e-9);
    BOOST_CHECK_CLOSE(fake->center_freq_hz(), 8.1e9, 1e-9);
    blk->set_cal_delay_native(kCalDelay);
    BOOST_CHECK_CLOSE(blk->cal_delay_native(), kCalDelay, 1e-9);
    BOOST_CHECK_EQUAL(blk->rx_scratch_capacity(),
                      static_cast<size_t>(kRxSamples) * 2);
    BOOST_CHECK_EQUAL(blk->tx_scratch_capacity(),
                      static_cast<size_t>(kTxSamples) * 2);

    auto src = gr::blocks::vector_source_c::make(tx, true, 1,
                                                 std::vector<gr::tag_t>());
    auto s2ts = gr::blocks::stream_to_tagged_stream::make(
        sizeof(gr_complex), 1, kTxSamples, "packet_len");
    auto head = gr::blocks::head::make(sizeof(gr_complex),
                                       kFrames * kRxSamples);
    auto snk = gr::blocks::vector_sink_c::make(1);

    auto tb = gr::make_top_block("qa_echo_timer_stream");
    tb->connect(src, 0, s2ts, 0);
    tb->connect(s2ts, 0, blk, 0);
    tb->connect(blk, 0, head, 0);
    tb->connect(head, 0, snk, 0);
    tb->run();

    // Exact frame geometry.
    BOOST_CHECK_EQUAL(snk->data().size(),
                      static_cast<size_t>(kFrames * kRxSamples));
    BOOST_CHECK_EQUAL(count_tag(snk->tags(), "packet_len"), kFrames);
    BOOST_CHECK_EQUAL(count_tag(snk->tags(), "sample_count"), kFrames);
    BOOST_CHECK_EQUAL(count_tag(snk->tags(), "burst_status"), 0u);
    BOOST_CHECK_EQUAL(count_tag(snk->tags(), "rx_time"), kFrames);

    // Every output window carries the radar geometry + identity tags.
    for (uint64_t k = 0; k < kFrames; ++k) {
        const uint64_t off = k * kRxSamples;
        BOOST_CHECK_CLOSE(
            pmt::to_double(find_tag(snk->tags(), "sample_rate", off)),
            kSampleRateHz, 1e-9);
        BOOST_CHECK_EQUAL(
            pmt::to_long(find_tag(snk->tags(), "pre_guard_samples", off)),
            static_cast<long>(kPreGuardSamples));
        BOOST_CHECK_EQUAL(
            pmt::to_long(find_tag(snk->tags(), "capture_samples", off)),
            static_cast<long>(kCaptureSamples));
        BOOST_CHECK_EQUAL(
            pmt::to_long(find_tag(snk->tags(), "post_guard_samples", off)),
            static_cast<long>(kPostGuardSamples));
        BOOST_CHECK_EQUAL(
            pmt::to_long(find_tag(snk->tags(), "sample_count", off)),
            static_cast<long>(kRxSamples));
        BOOST_CHECK_EQUAL(
            pmt::to_long(find_tag(snk->tags(), "window_start_sample", off)),
            0);
        BOOST_CHECK_EQUAL(
            pmt::to_long(find_tag(snk->tags(), "code_index", off)), 9);
        BOOST_CHECK_EQUAL(
            pmt::to_long(find_tag(snk->tags(), "sync_repetitions", off)), 64);
        BOOST_CHECK_EQUAL(
            tag_i64(find_tag(snk->tags(), "pulse_id", off)),
            static_cast<int64_t>(k));
        BOOST_CHECK_EQUAL(
            tag_i64(find_tag(snk->tags(), "schedule_index", off)),
            static_cast<int64_t>(k));
        BOOST_CHECK_CLOSE(
            pmt::to_double(find_tag(
                snk->tags(), "calibration_delay_native_samples", off)),
            kCalDelay, 1e-9);
        BOOST_CHECK(pmt::is_symbol(find_tag(snk->tags(), "sfd_mode", off)));
        BOOST_CHECK_EQUAL(
            pmt::symbol_to_string(find_tag(snk->tags(), "sfd_mode", off)),
            "4z2");
        BOOST_CHECK(pmt::is_tuple(find_tag(snk->tags(), "rx_time", off)));

        // SC16 payload: the fake device writes expected_rx_sample for the
        // burst's schedule index; the block divides by 32768.
        for (uint64_t p = 0; p < kRxSamples; ++p) {
            const float re = snk->data()[off + p].real();
            const float im = snk->data()[off + p].imag();
            const float ere =
                static_cast<float>(echo::FakeBurstBackend::expected_rx_sample(
                    k, p * 2)) /
                32768.0f;
            const float eim =
                static_cast<float>(echo::FakeBurstBackend::expected_rx_sample(
                    k, p * 2 + 1)) /
                32768.0f;
            BOOST_CHECK_CLOSE(re, ere, 1e-3);
            BOOST_CHECK_CLOSE(im, eim, 1e-3);
        }
    }

    // Block counters.
    BOOST_CHECK_EQUAL(blk->frames(), kFrames);
    BOOST_CHECK_EQUAL(blk->bursts_ok(), kFrames);
    BOOST_CHECK_EQUAL(blk->bursts_failed(), 0u);
    BOOST_CHECK_EQUAL(blk->late_slot_skips(), 0u);
    BOOST_CHECK_EQUAL(blk->grid_errors(), 0u);
    BOOST_CHECK_EQUAL(blk->rx_samples(), kFrames * kRxSamples);
    BOOST_CHECK_EQUAL(blk->tx_samples(), kFrames * kTxSamples);

    // Fake backend saw exactly one ok burst per window with the full
    // TX/RX sample counts.
    BOOST_CHECK_EQUAL(fake->burst_count(), static_cast<size_t>(kFrames));
    for (uint64_t k = 0; k < kFrames; ++k) {
        echo::FakeBurstBackend::FakeBurstRecord rec;
        BOOST_REQUIRE(fake->find_record(k, rec));
        BOOST_CHECK(rec.status == echo::BurstStatus::Ok);
        BOOST_CHECK_EQUAL(rec.tx_samples_requested, kTxSamples);
        BOOST_CHECK_EQUAL(rec.tx_samples_sent, kTxSamples);
        BOOST_CHECK_EQUAL(rec.rx_stitched.size(),
                          static_cast<size_t>(kRxSamples) * 2);
        BOOST_CHECK_EQUAL(rec.rx_samples_received, kRxSamples);
        BOOST_CHECK_EQUAL(rec.rx_time_ticks,
                          kT0Ticks + static_cast<int64_t>(k) * 3686400 -
                              kPreGuardTicks);
    }
}
