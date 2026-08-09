/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * QA for the asynchronous UwbRealtimeDemodulator message block (R5).
 *
 * Covers: golden window PDU round-trip (payload/FCS == golden), bounded-queue
 * backpressure (queue-full drops with the no-loss counter invariant),
 * stop()/drain semantics, worker exception fault injection, and invalid-input
 * rejection.
 */

#include <boost/test/unit_test.hpp>

#include <gnuradio/blocks/message_debug.h>
#include <gnuradio/gr_complex.h>
#include <gnuradio/top_block.h>
#include <gnuradio/uwb/uwb_demod_result.h>
#include <gnuradio/uwb/uwb_realtime_demodulator.h>
#include <pmt/pmt.h>

#include <chrono>
#include <complex>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <thread>
#include <vector>

using gr::uwb::demod::DemodStatus;

namespace {

std::string golden_dir()
{
    return "../../../testdata/realtime_demod_golden";
}

bool load_cf32(const std::string& path, std::vector<gr_complex>& out)
{
    std::ifstream f(path, std::ios::binary);
    if (!f)
        return false;
    f.seekg(0, std::ios::end);
    const std::streamoff bytes = f.tellg();
    f.seekg(0, std::ios::beg);
    if (bytes <= 0 || bytes % static_cast<std::streamoff>(sizeof(gr_complex)) != 0)
        return false;
    out.resize(static_cast<size_t>(bytes) / sizeof(gr_complex));
    f.read(reinterpret_cast<char*>(out.data()), bytes);
    return f.good() || f.eof();
}

bool load_u8(const std::string& path, std::vector<uint8_t>& out)
{
    std::ifstream f(path, std::ios::binary);
    if (!f)
        return false;
    f.seekg(0, std::ios::end);
    const std::streamoff bytes = f.tellg();
    f.seekg(0, std::ios::beg);
    if (bytes <= 0)
        return false;
    out.resize(static_cast<size_t>(bytes));
    f.read(reinterpret_cast<char*>(out.data()), bytes);
    return f.good() || f.eof();
}

std::vector<gr_complex> load_template()
{
    std::vector<gr_complex> t;
    load_cf32("../../../testdata/reference_preamble.bin", t);
    return t;
}

pmt::pmt_t make_samples_pdu(const std::vector<gr_complex>& iq,
                            uint64_t packet_id,
                            int64_t predicted_start)
{
    pmt::pmt_t meta = pmt::make_dict();
    meta = pmt::dict_add(meta, pmt::mp("packet_id"),
                         pmt::from_uint64(packet_id));
    meta = pmt::dict_add(meta, pmt::mp("schedule_index"),
                         pmt::from_uint64(0));
    meta = pmt::dict_add(meta, pmt::mp("predicted_start_sample"),
                         pmt::from_long(predicted_start));
    meta = pmt::dict_add(meta, pmt::mp("window_start_sample"),
                         pmt::from_long(0));
    meta = pmt::dict_add(meta, pmt::mp("sample_count"),
                         pmt::from_long(static_cast<long>(iq.size())));
    meta = pmt::dict_add(meta, pmt::mp("sample_rate"),
                         pmt::from_double(998.4e6));
    return pmt::cons(meta, pmt::init_c32vector(iq.size(), iq.data()));
}

bool wait_until(const gr::uwb::UwbRealtimeDemodulator::sptr& d,
                uint64_t want_received,
                size_t timeout_ms = 60000)
{
    const auto t0 = std::chrono::steady_clock::now();
    while (d->jobs_received() < want_received || !d->drained()) {
        if (std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - t0)
                .count() > static_cast<long long>(timeout_ms))
            return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return true;
}

bool wait_invalid(const gr::uwb::UwbRealtimeDemodulator::sptr& d,
                  uint64_t want,
                  size_t timeout_ms = 5000)
{
    const auto t0 = std::chrono::steady_clock::now();
    while (d->invalid_inputs() < want) {
        if (std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - t0)
                .count() > static_cast<long long>(timeout_ms))
            return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return true;
}

bool status_contains(gr::blocks::message_debug::sptr dbg,
                     const std::string& event)
{
    for (size_t i = 0; i < dbg->num_messages(); ++i) {
        pmt::pmt_t st = dbg->get_message(i);
        if (pmt::is_dict(st) &&
            pmt::eqv(pmt::dict_ref(st, pmt::mp("event"), pmt::PMT_NIL),
                     pmt::mp(event)))
            return true;
    }
    return false;
}

} // namespace

// ---------------------------------------------------------------------------
// Golden window PDU → result PDU round trip: payload bytes and FCS must
// exactly match the MATLAB golden export.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_golden_round_trip)
{
    std::vector<gr_complex> iq, tmpl;
    BOOST_REQUIRE(load_cf32(golden_dir() + "/window.cfile", iq));
    BOOST_REQUIRE(load_cf32("../../../testdata/reference_preamble.bin", tmpl));

    std::vector<uint8_t> golden_bytes;
    BOOST_REQUIRE(load_u8(golden_dir() + "/stage_payload_bytes.bin",
                          golden_bytes));
    BOOST_REQUIRE_EQUAL(golden_bytes.size(), 127u);

    auto demod =
        gr::uwb::UwbRealtimeDemodulator::make_from_template(tmpl, 2, 64,
                                                            "ieee");
    auto dbg = gr::blocks::message_debug::make();
    auto dbg_status = gr::blocks::message_debug::make();
    auto tb = gr::make_top_block("qa_realtime_roundtrip");
    tb->msg_connect(demod, "result", dbg, "store");
    tb->msg_connect(demod, "status", dbg_status, "store");

    tb->start();
    demod->_post(pmt::mp("samples"), make_samples_pdu(iq, 7, 9984));
    BOOST_REQUIRE(wait_until(demod, 1));
    tb->stop();
    tb->wait();

    BOOST_CHECK_EQUAL(demod->jobs_received(), 1u);
    BOOST_CHECK_EQUAL(demod->jobs_completed(), 1u);
    BOOST_CHECK_EQUAL(demod->jobs_failed(), 0u);
    BOOST_CHECK_EQUAL(demod->jobs_dropped(), 0u);
    BOOST_CHECK_EQUAL(demod->invalid_inputs(), 0u);
    BOOST_CHECK_EQUAL(demod->worker_exceptions(), 0u);
    BOOST_REQUIRE_EQUAL(dbg->num_messages(), 1u);

    pmt::pmt_t result = dbg->get_message(0);
    BOOST_REQUIRE(pmt::is_pair(result));
    pmt::pmt_t meta = pmt::car(result);
    BOOST_REQUIRE(pmt::is_dict(meta));

    BOOST_CHECK_EQUAL(pmt::to_long(pmt::dict_ref(
                          meta, pmt::mp("status_code"), pmt::from_long(-1))),
                      0); // Success
    BOOST_CHECK(pmt::eqv(
        pmt::dict_ref(meta, pmt::mp("status"), pmt::PMT_NIL),
        pmt::mp("success")));
    BOOST_CHECK(pmt::to_bool(
        pmt::dict_ref(meta, pmt::mp("fcs_pass"), pmt::PMT_F)));
    BOOST_CHECK_EQUAL(pmt::to_uint64(pmt::dict_ref(
                          meta, pmt::mp("payload_nbytes"),
                          pmt::from_uint64(0))),
                      127);
    BOOST_CHECK_EQUAL(pmt::to_uint64(pmt::dict_ref(
                          meta, pmt::mp("fcs_received"),
                          pmt::from_uint64(0))),
                      0x584b);
    BOOST_CHECK_EQUAL(pmt::to_uint64(pmt::dict_ref(
                          meta, pmt::mp("fcs_calculated"),
                          pmt::from_uint64(0))),
                      0x584b);
    BOOST_CHECK_EQUAL(pmt::to_uint64(pmt::dict_ref(
                          meta, pmt::mp("packet_id"), pmt::from_uint64(0))),
                      7);
    const int64_t det = pmt::to_long(pmt::dict_ref(
        meta, pmt::mp("detected_start_sample"), pmt::from_long(-1)));
    BOOST_CHECK(std::llabs(det - 9984) <= 2);

    // Payload bytes must match golden stage_payload_bytes.bin exactly.
    pmt::pmt_t vec = pmt::cdr(result);
    BOOST_REQUIRE(pmt::is_u8vector(vec));
    const size_t n = pmt::length(vec);
    BOOST_REQUIRE_EQUAL(n, golden_bytes.size());
    const std::vector<uint8_t>& b = pmt::u8vector_elements(vec);
    for (size_t i = 0; i < n; ++i)
        BOOST_CHECK_EQUAL(static_cast<int>(b[i]),
                          static_cast<int>(golden_bytes[i]));
}

// ---------------------------------------------------------------------------
// Bounded queue backpressure: a capacity-2 queue fed a 40-PDU burst must drop
// jobs (publishing queue_full) while preserving received == completed +
// failed + dropped, and never exceed the configured high-watermark.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_queue_full_drops_and_counters)
{
    std::vector<gr_complex> iq, tmpl;
    BOOST_REQUIRE(load_cf32(golden_dir() + "/window.cfile", iq));
    BOOST_REQUIRE(load_cf32("../../../testdata/reference_preamble.bin", tmpl));

    auto demod =
        gr::uwb::UwbRealtimeDemodulator::make_from_template(tmpl, 2, 2,
                                                            "ieee");
    auto dbg = gr::blocks::message_debug::make();
    auto dbg_status = gr::blocks::message_debug::make();
    auto tb = gr::make_top_block("qa_realtime_queuefull");
    tb->msg_connect(demod, "result", dbg, "store");
    tb->msg_connect(demod, "status", dbg_status, "store");

    const uint64_t N = 40;
    pmt::pmt_t pdu = make_samples_pdu(iq, 100, 9984);
    tb->start();
    for (uint64_t i = 0; i < N; ++i)
        demod->_post(pmt::mp("samples"), pdu);
    BOOST_REQUIRE(wait_until(demod, N));
    tb->stop();
    tb->wait();

    const uint64_t recv = demod->jobs_received();
    const uint64_t comp = demod->jobs_completed();
    const uint64_t fail = demod->jobs_failed();
    const uint64_t drop = demod->jobs_dropped();
    BOOST_CHECK_EQUAL(recv, N);
    BOOST_CHECK_MESSAGE(recv == comp + fail + drop,
                        "no-loss invariant broken: recv=" << recv
                        << " comp=" << comp << " fail=" << fail
                        << " drop=" << drop);
    BOOST_CHECK(drop > 0);
    BOOST_CHECK_LE(demod->queue_high_watermark(), 2u);
    BOOST_CHECK_EQUAL(dbg->num_messages(), comp + fail);
    BOOST_CHECK(status_contains(dbg_status, "queue_full"));
}

// ---------------------------------------------------------------------------
// stop()/drain: every accepted job gets exactly one result; counters are
// consistent and stop() joins the worker pool without hanging.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_stop_drains_queued_jobs)
{
    std::vector<gr_complex> iq, tmpl;
    BOOST_REQUIRE(load_cf32(golden_dir() + "/window.cfile", iq));
    BOOST_REQUIRE(load_cf32("../../../testdata/reference_preamble.bin", tmpl));

    auto demod =
        gr::uwb::UwbRealtimeDemodulator::make_from_template(tmpl, 1, 64,
                                                            "ieee");
    auto dbg = gr::blocks::message_debug::make();
    auto tb = gr::make_top_block("qa_realtime_stop");
    tb->msg_connect(demod, "result", dbg, "store");

    const uint64_t N = 3;
    tb->start();
    for (uint64_t i = 0; i < N; ++i)
        demod->_post(pmt::mp("samples"),
                     make_samples_pdu(iq, 200 + i, 9984));
    BOOST_REQUIRE(wait_until(demod, N));
    tb->stop();
    tb->wait();

    BOOST_CHECK_EQUAL(demod->jobs_received(), N);
    BOOST_CHECK_EQUAL(demod->jobs_completed() + demod->jobs_failed(), N);
    BOOST_CHECK_EQUAL(demod->jobs_dropped(), 0u);
    BOOST_CHECK_EQUAL(dbg->num_messages(), N);
    BOOST_CHECK(demod->drained());
}

// ---------------------------------------------------------------------------
// Worker exception handling: a TEST-ONLY fault-injection control message makes
// the worker throw; the block must catch it, emit worker_exception status and
// an InternalError result with empty bytes, and keep running.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_worker_exception_handled)
{
    std::vector<gr_complex> iq, tmpl;
    BOOST_REQUIRE(load_cf32(golden_dir() + "/window.cfile", iq));
    BOOST_REQUIRE(load_cf32("../../../testdata/reference_preamble.bin", tmpl));

    auto demod =
        gr::uwb::UwbRealtimeDemodulator::make_from_template(tmpl, 2, 16,
                                                            "ieee");
    auto dbg = gr::blocks::message_debug::make();
    auto dbg_status = gr::blocks::message_debug::make();
    auto tb = gr::make_top_block("qa_realtime_exception");
    tb->msg_connect(demod, "result", dbg, "store");
    tb->msg_connect(demod, "status", dbg_status, "store");

    tb->start();
    // Arm the fault for packet_id 99, then feed a matching PDU.
    pmt::pmt_t ctrl = pmt::dict_add(pmt::make_dict(), pmt::mp("cmd"),
                                    pmt::mp("fail_packet"));
    ctrl = pmt::dict_add(ctrl, pmt::mp("packet_id"), pmt::from_uint64(99));
    demod->_post(pmt::mp("control"), ctrl);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    demod->_post(pmt::mp("samples"), make_samples_pdu(iq, 99, 9984));
    BOOST_REQUIRE(wait_until(demod, 1));
    tb->stop();
    tb->wait();

    BOOST_CHECK_EQUAL(demod->worker_exceptions(), 1u);
    BOOST_CHECK(status_contains(dbg_status, "worker_exception"));
    BOOST_REQUIRE_EQUAL(dbg->num_messages(), 1u);
    pmt::pmt_t result = dbg->get_message(0);
    BOOST_REQUIRE(pmt::is_pair(result));
    pmt::pmt_t meta = pmt::car(result);
    BOOST_CHECK_EQUAL(pmt::to_long(pmt::dict_ref(
                          meta, pmt::mp("status_code"), pmt::from_long(-1))),
                      static_cast<long>(DemodStatus::InternalError));
    BOOST_CHECK_EQUAL(pmt::length(pmt::cdr(result)), 0u); // empty bytes
}

// ---------------------------------------------------------------------------
// Invalid input rejection: non-PDU messages and empty c32vectors are counted
// as invalid_inputs and never enter the job queue.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_invalid_input_rejected)
{
    std::vector<gr_complex> tmpl = load_template();
    auto demod =
        gr::uwb::UwbRealtimeDemodulator::make_from_template(tmpl, 2, 8,
                                                            "ieee");
    auto dbg_status = gr::blocks::message_debug::make();
    auto tb = gr::make_top_block("qa_realtime_invalid");
    tb->msg_connect(demod, "status", dbg_status, "store");

    tb->start();
    demod->_post(pmt::mp("samples"), pmt::mp("not_a_pdu")); // symbol, not pair
    pmt::pmt_t meta = pmt::make_dict();
    // Disambiguate nullptr: init_c32vector is overloaded on float* vs
    // std::complex<float>*.
    demod->_post(pmt::mp("samples"),
                 pmt::cons(meta,
                           pmt::init_c32vector(
                               0, static_cast<const gr_complex*>(nullptr)))); // empty
    BOOST_REQUIRE(wait_invalid(demod, 2));
    tb->stop();
    tb->wait();

    BOOST_CHECK_EQUAL(demod->invalid_inputs(), 2u);
    BOOST_CHECK_EQUAL(demod->jobs_received(), 0u);
    BOOST_CHECK(status_contains(dbg_status, "invalid_input"));
}
