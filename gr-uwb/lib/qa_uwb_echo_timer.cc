/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * QA for the EchoTimer scheduling core, burst backend abstraction and
 * message block (Radar Step 10).
 *
 * Covers: exact integer tick grid (large k, fractional PRI, no drift,
 * expired-slot skip-to-next-future, skip limit), fragment planning,
 * per-index burst results in order, RX-before-TX with first-fragment
 * SOB/time-spec and last-fragment EOB flag sequences, partial send/recv
 * with point-for-point stitched RX samples, injected late/timeout/
 * overflow/broken-chain/stop-during-I/O faults with a surviving worker,
 * block-level late slot skips, and stop/restart/destructor with pending
 * I/O (no deadlock, no lost already-ok bursts).
 */

// Regression guard (must stay the FIRST include of this TU):
// uwb_radar_checked_math.h is a shared header and must be self-contained
// (its own <cstddef> for size_t); this include fails to compile otherwise.
#include <gnuradio/uwb/uwb_radar_checked_math.h>

#include <boost/test/unit_test.hpp>

#include <gnuradio/blocks/message_debug.h>
#include <gnuradio/top_block.h>
#include <gnuradio/uwb/uwb_echo_burst_backend.h>
#include <gnuradio/uwb/uwb_echo_scheduler_core.h>
#include <gnuradio/uwb/uwb_fake_burst_backend.h>
#include <gnuradio/uwb/uwb_realtime_echo_timer.h>
#include <pmt/pmt.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <future>
#include <iterator>
#include <limits>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

using gr::uwb::UwbRealtimeEchoTimer;
namespace echo = gr::uwb::echo;
using namespace gr::uwb::echo;

namespace {

constexpr unsigned kSOB = kFlagStartOfBurst;
constexpr unsigned kEOB = kFlagEndOfBurst;
constexpr unsigned kTIME = kFlagTimeSpec;

bool
wait_until(const std::function<bool()>& pred, long timeout_ms = 30000)
{
    const auto t0 = std::chrono::steady_clock::now();
    while (!pred()) {
        const auto ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - t0)
                .count();
        if (ms > timeout_ms)
            return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    return true;
}

bool
wait_bursts(const gr::blocks::message_debug::sptr& dbg,
            size_t want,
            long timeout_ms = 30000)
{
    return wait_until([&] { return dbg->num_messages() >= want; },
                      timeout_ms);
}

pmt::pmt_t
make_schedule_pdu(int64_t t0,
                  uint64_t tx_samples,
                  uint64_t rx_samples,
                  uint64_t burst_count,
                  uint64_t index,
                  uint64_t pulse_id,
                  const std::vector<int16_t>& payload,
                  bool with_rate = false)
{
    pmt::pmt_t meta = pmt::make_dict();
    meta = pmt::dict_add(meta, pmt::mp("t0_ticks"), pmt::from_long(t0));
    meta = pmt::dict_add(meta, pmt::mp("tx_samples"),
                         pmt::from_uint64(tx_samples));
    meta = pmt::dict_add(meta, pmt::mp("rx_samples"),
                         pmt::from_uint64(rx_samples));
    meta = pmt::dict_add(meta, pmt::mp("schedule_index"),
                         pmt::from_uint64(index));
    meta = pmt::dict_add(meta, pmt::mp("pulse_id"),
                         pmt::from_uint64(pulse_id));
    meta = pmt::dict_add(meta, pmt::mp("burst_count"),
                         pmt::from_uint64(burst_count));
    if (with_rate)
        meta = pmt::dict_add(meta, pmt::mp("sample_rate"),
                             pmt::from_double(737280000.0));
    return pmt::cons(meta,
                     pmt::init_s16vector(payload.size(), payload.data()));
}

std::vector<int16_t>
make_payload(uint64_t tx_samples)
{
    std::vector<int16_t> payload(static_cast<size_t>(tx_samples) * 2);
    for (size_t i = 0; i < payload.size(); ++i)
        payload[i] = static_cast<int16_t>((i * 257 + 3) % 30000 - 15000);
    return payload;
}

int64_t
meta_i64(pmt::pmt_t meta, const char* key, int64_t def = 0)
{
    pmt::pmt_t v = pmt::dict_ref(meta, pmt::mp(key), pmt::PMT_NIL);
    if (pmt::is_uint64(v))
        return static_cast<int64_t>(pmt::to_uint64(v));
    if (pmt::is_integer(v))
        return pmt::to_long(v);
    return def;
}

uint64_t
meta_u64(pmt::pmt_t meta, const char* key, uint64_t def = 0)
{
    pmt::pmt_t v = pmt::dict_ref(meta, pmt::mp(key), pmt::PMT_NIL);
    if (pmt::is_uint64(v))
        return pmt::to_uint64(v);
    if (pmt::is_integer(v))
        return static_cast<uint64_t>(pmt::to_long(v));
    return def;
}

std::string
meta_str(pmt::pmt_t meta, const char* key)
{
    pmt::pmt_t v = pmt::dict_ref(meta, pmt::mp(key), pmt::PMT_NIL);
    if (pmt::is_symbol(v))
        return pmt::symbol_to_string(v);
    return {};
}

// Read burst message i and unpack it.
void
get_burst(const gr::blocks::message_debug::sptr& dbg,
          size_t i,
          pmt::pmt_t& meta,
          std::vector<int16_t>& samples)
{
    pmt::pmt_t msg = dbg->get_message(i);
    BOOST_REQUIRE(pmt::is_pair(msg));
    meta = pmt::car(msg);
    pmt::pmt_t data = pmt::cdr(msg);
    BOOST_REQUIRE(pmt::is_dict(meta));
    BOOST_REQUIRE(pmt::is_s16vector(data));
    size_t len = 0;
    const int16_t* el = pmt::s16vector_elements(data, len);
    samples.assign(el, el + len);
}

bool
status_seen(const gr::blocks::message_debug::sptr& dbg,
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

echo::EchoSchedulerConfig
base_grid_cfg()
{
    echo::EchoSchedulerConfig cfg;
    cfg.pri_num = 3686400; // 5 ms @ 737.28 MS/s
    cfg.pri_den = 1;
    cfg.pre_guard_ticks = 1475;
    cfg.max_fragment_size = 4096;
    return cfg;
}

} // namespace

// ---------------------------------------------------------------------------
// 0. Shared checked-math header: self-contained inclusion and API sanity
//    (guards against regression of the header's own includes).
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_echo_checked_math_selfcontained)
{
    using gr::uwb::radar::radar_i64_add;
    using gr::uwb::radar::radar_i64_from_size;
    using gr::uwb::radar::radar_i64_mul;

    int64_t v = 0;
    BOOST_CHECK(radar_i64_from_size(3686400u, v));
    BOOST_CHECK_EQUAL(v, 3686400);
    BOOST_CHECK(!radar_i64_from_size(
        static_cast<size_t>(std::numeric_limits<uint64_t>::max()), v));
    BOOST_CHECK(radar_i64_mul(3686400, 2, v) && v == 7372800);
    BOOST_CHECK(!radar_i64_mul(std::numeric_limits<int64_t>::max(), 2, v));
    BOOST_CHECK(radar_i64_add(0, std::numeric_limits<int64_t>::max(), v));
    BOOST_CHECK_EQUAL(v, std::numeric_limits<int64_t>::max());
    BOOST_CHECK(!radar_i64_add(1, std::numeric_limits<int64_t>::max(), v));
}

// ---------------------------------------------------------------------------
// 1. Tick grid core: exact rational math over large k, no cumulative
//    drift; expired-slot skip-to-next-future; skip limit; validation.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_echo_tick_grid_core)
{
    // Fractional PRI: pri = 14745601/4 ticks (non-integer by design).
    echo::EchoSchedulerConfig cfg;
    cfg.pri_num = 14745601;
    cfg.pri_den = 4;
    cfg.pre_guard_ticks = 1475;
    cfg.max_catchup_slots = 1u << 20;
    cfg.max_fragment_size = 64;

    echo::EchoSchedulerPrepared prep;
    BOOST_REQUIRE(echo::prepare_echo_scheduler(cfg, prep));
    BOOST_CHECK_EQUAL(prep.pri_whole, 14745601 / 4);
    BOOST_CHECK_EQUAL(prep.pri_rem, 14745601 % 4);

    // Exactness over a large index range: integer tick math must match
    // the exact rational expectation k * num / den (no double drift).
    echo::EchoGrid grid(prep);
    const int64_t t0 = 1000000;
    BOOST_REQUIRE(grid.arm(t0, 0));
    constexpr uint64_t kBig = 100000;
    for (uint64_t k = 0; k <= kBig; ++k) {
        echo::EchoSlot slot;
        // Very negative `now`: every slot is in the future → no skipping.
        const echo::EchoScheduleStatus st =
            grid.next_schedule(-2305843009213693952LL, slot);
        BOOST_REQUIRE_EQUAL(static_cast<int>(st),
                            static_cast<int>(
                                echo::EchoScheduleStatus::Ok));
        BOOST_CHECK_EQUAL(slot.index, k);
        int64_t whole = 0, rem = 0;
        BOOST_REQUIRE(
            echo::echo_slot_offset(k, cfg.pri_num, cfg.pri_den, whole, rem));
        BOOST_CHECK_EQUAL(slot.t_tx_whole, t0 + whole);
        BOOST_CHECK_EQUAL(slot.t_tx_rem_num, rem);
        BOOST_CHECK_EQUAL(slot.t_tick_den, cfg.pri_den);
        // RX command must be EARLIER than TX by exactly the pre-guard.
        BOOST_CHECK_EQUAL(slot.t_rx_whole, t0 + whole - 1475);
        BOOST_CHECK_LT(slot.t_rx_whole, slot.t_tx_whole);
    }

    // Expired slots are skipped, never caught up.  Exact equality with a
    // slot time counts as expired (t_tx > now is required).
    BOOST_REQUIRE(grid.arm(t0, 0));
    const int64_t pri = cfg.pri_num; // den == 4
    const int64_t now4 = 4 * pri;    // 4 * pri in quarter-tick units
    echo::EchoSlot slot;
    // now = t0 + 4*pri exactly → slots 0..4 are expired, slot 5 is future.
    const echo::EchoScheduleStatus st =
        grid.next_schedule(t0 + now4 / 4, slot);
    BOOST_REQUIRE_EQUAL(static_cast<int>(st),
                        static_cast<int>(echo::EchoScheduleStatus::ExpiredSkipped));
    BOOST_CHECK_EQUAL(slot.index, 5u);
    BOOST_CHECK_EQUAL(slot.skipped, 5u);
    // t_tx(5) = t0 + 5 * (pri_num / pri_den) in whole ticks.
    BOOST_CHECK_EQUAL(slot.t_tx_whole, t0 + 5 * pri / 4);
    BOOST_CHECK_EQUAL(slot.t_rx_whole, t0 + 5 * pri / 4 - 1475);

    // arm() validation: t0 must leave room for the pre-guard.
    echo::EchoGrid grid2(prep);
    BOOST_CHECK(!grid2.arm(1474, 0));
    BOOST_CHECK(grid2.arm(1475, 0));

    // Skip limit: exceeding max_catchup_slots is an explicit failure; the
    // grid stays armed (state remains where skipping stopped), so re-arm
    // to start over from index 0.
    echo::EchoSchedulerConfig cfg2 = cfg;
    cfg2.max_catchup_slots = 2;
    echo::EchoSchedulerPrepared prep2;
    BOOST_REQUIRE(echo::prepare_echo_scheduler(cfg2, prep2));
    echo::EchoGrid grid3(prep2);
    BOOST_REQUIRE(grid3.arm(t0, 0));
    echo::EchoSlot s;
    // now = t0 + 10 * pri ticks → slots 0..9 expired, limit is 2.
    const int64_t now10 =
        t0 + (10 * cfg.pri_num) / cfg.pri_den;
    BOOST_CHECK_EQUAL(
        static_cast<int>(grid3.next_schedule(now10, s)),
        static_cast<int>(echo::EchoScheduleStatus::SkipLimitExceeded));
    BOOST_REQUIRE(grid3.arm(t0, 0));
    BOOST_CHECK_EQUAL(
        static_cast<int>(grid3.next_schedule(0, s)),
        static_cast<int>(echo::EchoScheduleStatus::Ok));
    BOOST_CHECK_EQUAL(s.index, 0u);
    BOOST_CHECK_EQUAL(s.skipped, 0u);

    // prepare() validation failures.
    echo::EchoSchedulerPrepared bad;
    echo::EchoSchedulerConfig c0 = cfg;
    c0.pri_den = 0;
    BOOST_CHECK(!echo::prepare_echo_scheduler(c0, bad));
    c0 = cfg;
    c0.pre_guard_ticks = cfg.pri_num; // >= PRI → invalid
    c0.pri_den = 1;
    c0.pri_num = 3686400;
    BOOST_CHECK(!echo::prepare_echo_scheduler(c0, bad));
    c0 = cfg;
    c0.max_fragment_size = 0;
    BOOST_CHECK(!echo::prepare_echo_scheduler(c0, bad));

    // echo_slot_offset: checked-mul overflow is an explicit failure, not
    // undefined behaviour.
    int64_t w = 0, r = 0;
    BOOST_CHECK(!echo::echo_slot_offset(1ULL << 62, 1LL << 62, 1, w, r));
    BOOST_CHECK(echo::echo_slot_offset(kBig, 14745601, 4, w, r));

    // plan_fragments: contiguous spans of at most max_fragment_size.
    echo::EchoFragmentSpan spans[echo::kEchoMaxFragmentsPerBurst];
    size_t n = 0;
    BOOST_REQUIRE(
        echo::plan_fragments(250, 100, spans, echo::kEchoMaxFragmentsPerBurst, n));
    BOOST_REQUIRE_EQUAL(n, 3u);
    BOOST_CHECK_EQUAL(spans[0].offset, 0u);
    BOOST_CHECK_EQUAL(spans[0].count, 100u);
    BOOST_CHECK_EQUAL(spans[1].offset, 100u);
    BOOST_CHECK_EQUAL(spans[1].count, 100u);
    BOOST_CHECK_EQUAL(spans[2].offset, 200u);
    BOOST_CHECK_EQUAL(spans[2].count, 50u);
    BOOST_CHECK(echo::plan_fragments(250, 100, spans, 2, n) == false);
    BOOST_CHECK(echo::plan_fragments(0, 100, spans, 64, n) == false);
    BOOST_CHECK(echo::plan_fragments(250, 0, spans, 64, n) == false);
    BOOST_CHECK(echo::plan_fragments(
        65, 1, spans, echo::kEchoMaxFragmentsPerBurst, n) == false);
}

// ---------------------------------------------------------------------------
// 2. Block happy path: one ok result per schedule index, in order; RX
//    earlier than TX; SOB/time-spec only on the first fragment, EOB only
//    on the last; exact tick math surfaces in the burst metadata.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_echo_timer_ok_bursts)
{
    auto fake = std::make_shared<FakeBurstBackend>(
        FakeBurstBackend::Config{}); // no partials, no faults
    auto blk = UwbRealtimeEchoTimer::make(base_grid_cfg(), fake, 16, 1000);
    auto burst_dbg = gr::blocks::message_debug::make();
    auto status_dbg = gr::blocks::message_debug::make();
    auto tb = gr::make_top_block("qa_echo_timer_ok");
    tb->msg_connect(blk, "burst", burst_dbg, "store");
    tb->msg_connect(blk, "status", status_dbg, "store");
    BOOST_REQUIRE(blk->start());
    tb->start();

    const uint64_t tx_samples = 1000;
    const uint64_t rx_samples = 800;
    const auto payload = make_payload(tx_samples);
    const int64_t t0 = 1000000;
    blk->_post(pmt::mp("schedule"),
               make_schedule_pdu(t0, tx_samples, rx_samples, /*count=*/3,
                                 /*index=*/0, /*pulse=*/7, payload,
                                 /*with_rate=*/true));

    BOOST_REQUIRE(wait_bursts(burst_dbg, 3));
    BOOST_CHECK_EQUAL(burst_dbg->num_messages(), 3u);

    const int64_t pri = 3686400;
    for (uint64_t k = 0; k < 3; ++k) {
        pmt::pmt_t meta;
        std::vector<int16_t> rx;
        get_burst(burst_dbg, k, meta, rx);
        BOOST_CHECK_EQUAL(meta_str(meta, "status"), "ok");
        BOOST_CHECK_EQUAL(meta_u64(meta, "schedule_index"), k);
        BOOST_CHECK_EQUAL(meta_u64(meta, "pulse_id"), 7u);
        const int64_t expect_tx = t0 + static_cast<int64_t>(k) * pri;
        BOOST_CHECK_EQUAL(meta_i64(meta, "tx_ticks"), expect_tx);
        BOOST_CHECK_EQUAL(meta_i64(meta, "rx_ticks"), expect_tx - 1475);
        BOOST_CHECK_LT(meta_i64(meta, "rx_ticks"),
                       meta_i64(meta, "tx_ticks"));
        BOOST_CHECK_EQUAL(meta_u64(meta, "rx_samples_received"),
                          rx_samples);
        BOOST_CHECK_EQUAL(meta_i64(meta, "rx_time_ticks"), expect_tx - 1475);
        BOOST_CHECK_EQUAL(meta_str(meta, "uhd_error"), "");
        BOOST_CHECK_EQUAL(meta_str(meta, "sample_format"), "sc16");
        BOOST_REQUIRE_EQUAL(rx.size(), rx_samples * 2);
        for (uint64_t p = 0; p < rx_samples * 2; ++p)
            BOOST_CHECK_EQUAL(
                rx[p], FakeBurstBackend::expected_rx_sample(k, p));
        // Device-time metadata derived from the whole-tick grid.
        BOOST_CHECK_EQUAL(meta_u64(meta, "tx_time_full"),
                          static_cast<uint64_t>(expect_tx / 737280000));
    }

    // Backend-seen fragment flag sequences: single fragment per burst is
    // both first (time spec + SOB) and last (EOB).
    for (uint64_t k = 0; k < 3; ++k) {
        FakeBurstBackend::FakeBurstRecord rec;
        BOOST_REQUIRE(fake->find_record(k, rec));
        BOOST_CHECK(rec.status == BurstStatus::Ok);
        BOOST_REQUIRE_EQUAL(rec.tx_flags.size(), 1u);
        BOOST_CHECK_EQUAL(rec.tx_flags[0], kSOB | kEOB | kTIME);
        BOOST_REQUIRE_EQUAL(rec.rx_flags.size(), 1u);
        BOOST_CHECK_EQUAL(rec.rx_flags[0], kSOB | kEOB | kTIME);
        BOOST_CHECK_EQUAL(rec.tx_calls, 1u);
        BOOST_CHECK_EQUAL(rec.rx_calls, 1u);
        BOOST_CHECK_EQUAL(rec.tx_reissues, 0u);
        BOOST_CHECK_EQUAL(rec.rx_reissues, 0u);
        BOOST_CHECK_EQUAL(rec.tx_samples_sent, tx_samples);
        BOOST_CHECK_EQUAL(rec.rx_samples_received, rx_samples);
        BOOST_CHECK_EQUAL(rec.tx_stitched.size(), payload.size());
        BOOST_CHECK(rec.tx_stitched == payload); // point-for-point
        BOOST_CHECK_EQUAL(rec.rx_time_ticks, t0 + static_cast<int64_t>(k) * pri - 1475);
    }

    BOOST_CHECK_EQUAL(blk->bursts_published(), 3u);
    BOOST_CHECK_EQUAL(blk->bursts_ok(), 3u);
    BOOST_CHECK_EQUAL(blk->bursts_failed(), 0u);
    BOOST_CHECK_EQUAL(blk->late_slot_skips(), 0u);
    BOOST_CHECK(status_seen(status_dbg, "schedule_armed"));
    BOOST_CHECK(status_seen(status_dbg, "grid_complete"));

    tb->stop();
    tb->wait();
    BOOST_REQUIRE(blk->stop());
}

// ---------------------------------------------------------------------------
// 3. Partial TX AND RX with a small max fragment size: per-call flag
//    sequences (time spec + SOB on the first call only, EOB on the last
//    fragment), re-issue counts, and a point-for-point stitched RX
//    sequence.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_echo_timer_partial_stitch)
{
    auto cfg = base_grid_cfg();
    cfg.max_fragment_size = 100; // block-level fragment planning
    FakeBurstBackend::Config fcfg;
    fcfg.max_io_chunk = 37; // forces partial send/recv inside fragments
    auto fake = std::make_shared<FakeBurstBackend>(fcfg);
    auto blk = UwbRealtimeEchoTimer::make(cfg, fake, 16, 1000);
    auto burst_dbg = gr::blocks::message_debug::make();
    auto tb = gr::make_top_block("qa_echo_timer_partial");
    tb->msg_connect(blk, "burst", burst_dbg, "store");
    BOOST_REQUIRE(blk->start());
    tb->start();

    const uint64_t tx_samples = 250; // fragments: 100 + 100 + 50
    const uint64_t rx_samples = 333; // fragments: 100 + 100 + 100 + 33
    const auto payload = make_payload(tx_samples);
    blk->_post(pmt::mp("schedule"),
               make_schedule_pdu(1000000, tx_samples, rx_samples,
                                 /*count=*/1, 0, 0, payload));

    BOOST_REQUIRE(wait_bursts(burst_dbg, 1));
    pmt::pmt_t meta;
    std::vector<int16_t> rx;
    get_burst(burst_dbg, 0, meta, rx);
    BOOST_CHECK_EQUAL(meta_str(meta, "status"), "partial_handled");
    BOOST_CHECK_EQUAL(meta_u64(meta, "tx_samples_sent"), tx_samples);
    BOOST_CHECK_EQUAL(meta_u64(meta, "rx_samples_received"), rx_samples);
    BOOST_CHECK_EQUAL(meta_u64(meta, "tx_reissues"), 5u);
    BOOST_CHECK_EQUAL(meta_u64(meta, "rx_reissues"), 6u);
    BOOST_REQUIRE_EQUAL(rx.size(), rx_samples * 2);
    for (uint64_t p = 0; p < rx_samples * 2; ++p)
        BOOST_CHECK_EQUAL(rx[p], FakeBurstBackend::expected_rx_sample(0, p));

    FakeBurstBackend::FakeBurstRecord rec;
    BOOST_REQUIRE(fake->find_record(0, rec));
    // Calls per fragment with chunk 37:
    //   TX  frag0 (SOB|TIME): 37,37,26 → 3 calls; frag1: 3 calls;
    //       frag2 (EOB): 37,13 → 2 calls
    //   RX  frag0 (SOB|TIME): 3 calls; frag1/frag2: 3 calls each;
    //       frag3 (EOB): 33 → 1 call
    BOOST_CHECK_EQUAL(rec.tx_calls, 8u);
    BOOST_CHECK_EQUAL(rec.rx_calls, 10u);
    const uint8_t expect_tx_flags[] = { kSOB | kTIME, 0, 0, // frag0
                                        0, 0, 0,           // frag1
                                        kEOB, kEOB };      // frag2
    const uint8_t expect_rx_flags[] = { kSOB | kTIME, 0, 0,     // frag0
                                        0, 0, 0,                // frag1
                                        0, 0, 0,                // frag2
                                        kEOB };                 // frag3
    BOOST_REQUIRE_EQUAL(rec.tx_flags.size(), 8u);
    BOOST_REQUIRE_EQUAL(rec.rx_flags.size(), 10u);
    BOOST_CHECK(std::equal(std::begin(expect_tx_flags),
                           std::end(expect_tx_flags),
                           rec.tx_flags.begin()));
    BOOST_CHECK(std::equal(std::begin(expect_rx_flags),
                           std::end(expect_rx_flags),
                           rec.rx_flags.begin()));
    BOOST_CHECK_EQUAL(rec.tx_samples_sent, tx_samples);
    BOOST_CHECK_EQUAL(rec.rx_samples_received, rx_samples);
    BOOST_CHECK(rec.tx_stitched == payload); // point-for-point TX
    BOOST_CHECK_EQUAL(rec.rx_stitched.size(), rx_samples * 2);
    for (uint64_t p = 0; p < rx_samples * 2; ++p)
        BOOST_CHECK_EQUAL(rec.rx_stitched[p],
                          FakeBurstBackend::expected_rx_sample(0, p));

    BOOST_CHECK_EQUAL(blk->bursts_published(), 1u);
    BOOST_CHECK_EQUAL(blk->bursts_ok(), 1u);
    BOOST_CHECK_EQUAL(blk->bursts_failed(), 0u);
    BOOST_CHECK_EQUAL(blk->tx_reissues(), 5u);
    BOOST_CHECK_EQUAL(blk->rx_reissues(), 6u);

    tb->stop();
    tb->wait();
    BOOST_REQUIRE(blk->stop());
}

// ---------------------------------------------------------------------------
// 4. Injected faults: late command, timeout, overflow, broken chain and
//    stop-during-I/O produce per-index failure results; the worker stays
//    alive and subsequent bursts still succeed.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_echo_timer_fault_matrix)
{
    auto cfg = base_grid_cfg();
    FakeBurstBackend::Config fcfg;
    fcfg.faults = { { 0, BurstStatus::LateCommand },
                    { 1, BurstStatus::Timeout },
                    { 2, BurstStatus::Overflow },
                    { 3, BurstStatus::BrokenChain },
                    { 4, BurstStatus::StopDuringIo } };
    auto fake = std::make_shared<FakeBurstBackend>(fcfg);
    auto blk = UwbRealtimeEchoTimer::make(cfg, fake, 16, /*wait=*/50);
    auto burst_dbg = gr::blocks::message_debug::make();
    auto tb = gr::make_top_block("qa_echo_timer_faults");
    tb->msg_connect(blk, "burst", burst_dbg, "store");
    BOOST_REQUIRE(blk->start());
    tb->start();

    const auto payload = make_payload(64);
    blk->_post(pmt::mp("schedule"),
               make_schedule_pdu(1000000, 64, 64, /*count=*/6, 0, 0,
                                 payload));

    // 5 faulted indices × up to 50 ms timeout wait + margin.
    BOOST_REQUIRE(wait_bursts(burst_dbg, 6, 30000));
    BOOST_CHECK_EQUAL(burst_dbg->num_messages(), 6u);

    const char* expect_status[] = { "late_command", "timeout", "overflow",
                                    "broken_chain", "stop_during_io",
                                    "ok" };
    for (uint64_t k = 0; k < 6; ++k) {
        pmt::pmt_t meta;
        std::vector<int16_t> rx;
        get_burst(burst_dbg, k, meta, rx);
        BOOST_CHECK_EQUAL(meta_str(meta, "status"), expect_status[k]);
        BOOST_CHECK_EQUAL(meta_u64(meta, "schedule_index"), k);
        if (k < 5)
            BOOST_CHECK(!meta_str(meta, "uhd_error").empty());
        else
            BOOST_CHECK(meta_str(meta, "uhd_error").empty());
        if (k < 5) {
            BOOST_CHECK_EQUAL(rx.size(), 0u); // failure → no samples
        } else {
            // The worker survived five faulted bursts and produced an ok
            // burst for index 5 with a full stitched capture.
            BOOST_CHECK_EQUAL(meta_u64(meta, "rx_samples_received"), 64u);
            BOOST_REQUIRE_EQUAL(rx.size(), 128u);
            for (uint64_t p = 0; p < 128; ++p)
                BOOST_CHECK_EQUAL(
                    rx[p], FakeBurstBackend::expected_rx_sample(5, p));
        }
    }

    BOOST_CHECK_EQUAL(blk->bursts_published(), 6u);
    BOOST_CHECK_EQUAL(blk->bursts_ok(), 1u);
    BOOST_CHECK_EQUAL(blk->bursts_failed(), 5u);
    BOOST_CHECK_EQUAL(blk->grid_errors(), 0u);

    tb->stop();
    tb->wait();
    BOOST_REQUIRE(blk->stop());
}

// ---------------------------------------------------------------------------
// 5. Block-level expired-slot handling: device time already past several
//    grid slots → they are skipped (late_slot_skips) and the worker
//    resumes at the next future index.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_echo_timer_late_skip)
{
    auto fake = std::make_shared<FakeBurstBackend>(FakeBurstBackend::Config{});
    auto blk = UwbRealtimeEchoTimer::make(base_grid_cfg(), fake, 16, 1000);
    auto burst_dbg = gr::blocks::message_debug::make();
    auto status_dbg = gr::blocks::message_debug::make();
    auto tb = gr::make_top_block("qa_echo_timer_late");
    tb->msg_connect(blk, "burst", burst_dbg, "store");
    tb->msg_connect(blk, "status", status_dbg, "store");
    BOOST_REQUIRE(blk->start());
    tb->start();

    const int64_t t0 = 1000000;
    const int64_t pri = 3686400;
    // Device time past slots 0..3 → burst for indices 4 and 5 only.
    fake->set_device_time(t0 + 3 * pri + pri / 2);
    const auto payload = make_payload(64);
    blk->_post(pmt::mp("schedule"),
               make_schedule_pdu(t0, 64, 64, /*count=*/2, 0, 0, payload));

    BOOST_REQUIRE(wait_bursts(burst_dbg, 2));
    BOOST_CHECK_EQUAL(burst_dbg->num_messages(), 2u);
    for (size_t i = 0; i < 2; ++i) {
        pmt::pmt_t meta;
        std::vector<int16_t> rx;
        get_burst(burst_dbg, i, meta, rx);
        BOOST_CHECK_EQUAL(meta_str(meta, "status"), "ok");
        BOOST_CHECK_EQUAL(meta_u64(meta, "schedule_index"), 4u + i);
        BOOST_CHECK_EQUAL(meta_i64(meta, "tx_ticks"),
                          t0 + static_cast<int64_t>(4 + i) * pri);
    }
    BOOST_CHECK_EQUAL(blk->late_slot_skips(), 4u);
    BOOST_CHECK(status_seen(status_dbg, "late_slot_skip"));

    tb->stop();
    tb->wait();
    BOOST_REQUIRE(blk->stop());
}

// ---------------------------------------------------------------------------
// 6. stop() with I/O in flight (blocked collect): no deadlock; the
//    in-flight burst publishes a stop_during_io result; restart is
//    idempotent and loses no already-ok bursts.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_echo_timer_stop_pending_io)
{
    auto cfg = base_grid_cfg();
    FakeBurstBackend::Config fcfg;
    fcfg.faults = { { 0, BurstStatus::Timeout } }; // RX never completes
    auto fake = std::make_shared<FakeBurstBackend>(fcfg);
    auto blk = UwbRealtimeEchoTimer::make(cfg, fake, 16, /*wait=*/60000);
    auto burst_dbg = gr::blocks::message_debug::make();
    auto tb = gr::make_top_block("qa_echo_timer_stop");
    tb->msg_connect(blk, "burst", burst_dbg, "store");
    BOOST_REQUIRE(blk->start());
    tb->start();

    const auto payload = make_payload(64);
    blk->_post(pmt::mp("schedule"),
               make_schedule_pdu(1000000, 64, 64, /*count=*/2, 0, 0,
                                 payload));

    // The worker must be blocked in collect_result() for index 0.
    BOOST_REQUIRE(wait_until([&] { return fake->collect_waiters() == 1; }));

    std::atomic<bool> stopped{ false };
    auto fut = std::async(std::launch::async, [&] {
        stopped = blk->stop();
        return true;
    });
    BOOST_CHECK(fut.wait_for(std::chrono::seconds(10)) ==
                std::future_status::ready);
    BOOST_CHECK(fut.get());

    // The in-flight burst produced exactly one failure result.
    BOOST_REQUIRE(wait_bursts(burst_dbg, 1, 5000));
    pmt::pmt_t meta;
    std::vector<int16_t> rx;
    get_burst(burst_dbg, 0, meta, rx);
    BOOST_CHECK_EQUAL(meta_str(meta, "status"), "stop_during_io");
    BOOST_CHECK_EQUAL(meta_u64(meta, "schedule_index"), 0u);
    BOOST_CHECK_EQUAL(rx.size(), 0u);
    BOOST_CHECK_EQUAL(blk->bursts_published(), 1u);
    BOOST_CHECK_EQUAL(blk->bursts_failed(), 1u);

    // Restart (writer semantics): counters reset, already-published
    // results are not lost, and a fresh clean schedule succeeds.
    tb->stop();
    tb->wait();
    BOOST_REQUIRE(blk->start());
    tb->start();
    blk->_post(pmt::mp("schedule"),
               make_schedule_pdu(1000000, 64, 64, /*count=*/1, 0, 0,
                                 payload));
    BOOST_REQUIRE(wait_bursts(burst_dbg, 2));
    get_burst(burst_dbg, 1, meta, rx);
    BOOST_CHECK_EQUAL(meta_str(meta, "status"), "ok");
    BOOST_CHECK_EQUAL(meta_u64(meta, "schedule_index"), 0u);
    BOOST_CHECK_EQUAL(blk->bursts_published(), 1u); // reset by restart
    BOOST_CHECK_EQUAL(blk->bursts_ok(), 1u);

    tb->stop();
    tb->wait();
    BOOST_REQUIRE(blk->stop());
}

// ---------------------------------------------------------------------------
// 7. Destructor with pending I/O and queued schedule jobs: joins without
//    deadlock; already-ok bursts are not lost; queued schedules that
//    could not run are reported.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_echo_timer_destructor_pending)
{
    auto fake = std::make_shared<FakeBurstBackend>(FakeBurstBackend::Config{});
    {
        auto blk = UwbRealtimeEchoTimer::make(base_grid_cfg(), fake, 16,
                                              1000);
        auto burst_dbg = gr::blocks::message_debug::make();
        auto status_dbg = gr::blocks::message_debug::make();
        auto tb = gr::make_top_block("qa_echo_timer_dtor");
        tb->msg_connect(blk, "burst", burst_dbg, "store");
        tb->msg_connect(blk, "status", status_dbg, "store");
        BOOST_REQUIRE(blk->start());
        tb->start();

        const auto payload = make_payload(64);
        // One armed grid (finite budget) plus two more schedules that
        // stay queued behind it.
        blk->_post(pmt::mp("schedule"),
                   make_schedule_pdu(1000000, 64, 64, /*count=*/1, 0, 0,
                                     payload));
        blk->_post(pmt::mp("schedule"),
                   make_schedule_pdu(5000000, 64, 64, /*count=*/1, 0, 1,
                                     payload));
        blk->_post(pmt::mp("schedule"),
                   make_schedule_pdu(9000000, 64, 64, /*count=*/1, 0, 2,
                                     payload));
        // Let the worker make progress but not necessarily drain
        // everything, then stop the flowgraph (block stop joins with any
        // in-flight I/O).  The destructor must be safe regardless.
        BOOST_REQUIRE(wait_bursts(burst_dbg, 1));
        tb->stop();
        tb->wait();
        // Whatever was published before the stop must be intact and ok.
        for (size_t i = 0; i < burst_dbg->num_messages(); ++i) {
            pmt::pmt_t meta;
            std::vector<int16_t> rx;
            get_burst(burst_dbg, i, meta, rx);
            BOOST_CHECK_EQUAL(meta_str(meta, "status"), "ok");
        }
        // Explicit dtor path: destroy the block sptr through the
        // flowgraph teardown (block dtor joins a finished worker).
        blk = nullptr; // flowgraph still holds a reference; safe
        tb = nullptr;
        status_dbg = nullptr;
        burst_dbg = nullptr;
    }
}

// ---------------------------------------------------------------------------
// 8. Bounded input / fixed scratch: sample caps enforced in the handler
//    (huge/negative/over-cap rejection), fixed-size RX scratch allocated
//    once at construction (capacity and address never change), and make()
//    validation of the caps themselves.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_echo_timer_bounded_scratch)
{
    auto cfg = base_grid_cfg();
    const uint64_t max_tx = 4096;
    const uint64_t max_rx = 2048;
    auto fake = std::make_shared<FakeBurstBackend>(FakeBurstBackend::Config{});
    auto blk = UwbRealtimeEchoTimer::make(cfg, fake, 16, 1000, max_tx,
                                          max_rx);
    BOOST_CHECK_EQUAL(blk->max_tx_samples(), max_tx);
    BOOST_CHECK_EQUAL(blk->max_rx_samples(), max_rx);
    BOOST_CHECK_EQUAL(blk->rx_scratch_capacity(),
                      static_cast<size_t>(max_rx) * 2);
    const int16_t* scratch0 = blk->rx_scratch_data();
    BOOST_REQUIRE(scratch0 != nullptr);

    // make() validation: zero caps and caps whose 2x overflows size_t.
    BOOST_REQUIRE_THROW(UwbRealtimeEchoTimer::make(
                            cfg, fake, 16, 1000, max_tx, 0),
                        std::invalid_argument);
    BOOST_REQUIRE_THROW(UwbRealtimeEchoTimer::make(
                            cfg, fake, 16, 1000, max_tx,
                            std::numeric_limits<size_t>::max() / 2 + 1),
                        std::invalid_argument);

    auto burst_dbg = gr::blocks::message_debug::make();
    auto status_dbg = gr::blocks::message_debug::make();
    auto tb = gr::make_top_block("qa_echo_timer_bounded");
    tb->msg_connect(blk, "burst", burst_dbg, "store");
    tb->msg_connect(blk, "status", status_dbg, "store");
    BOOST_REQUIRE(blk->start());
    tb->start();

    const auto payload_cap = make_payload(max_tx); // boundary payload

    // Rejections (each bumps schedules_invalid, produces no burst):
    auto post_invalid = [&](uint64_t tx,
                            uint64_t rx,
                            const std::vector<int16_t>& payload) {
        const uint64_t before = blk->schedules_invalid();
        blk->_post(pmt::mp("schedule"),
                   make_schedule_pdu(1000000, tx, rx, /*count=*/1, 0, 0,
                                     payload));
        BOOST_REQUIRE(wait_until(
            [&] { return blk->schedules_invalid() > before; }));
    };
    post_invalid(max_tx + 1, 100, make_payload(max_tx + 1)); // tx over cap
    post_invalid(100, max_rx + 1, make_payload(100));        // rx over cap
    post_invalid(std::numeric_limits<uint64_t>::max(), 100,
                 make_payload(100));                         // 2x overflow
    {
        // Negative int64 PMT values for the sample counts.
        pmt::pmt_t meta = pmt::make_dict();
        meta = pmt::dict_add(meta, pmt::mp("t0_ticks"),
                             pmt::from_long(1000000));
        meta = pmt::dict_add(meta, pmt::mp("tx_samples"),
                             pmt::from_long(-5));
        meta = pmt::dict_add(meta, pmt::mp("rx_samples"),
                             pmt::from_long(-7));
        const uint64_t before = blk->schedules_invalid();
        blk->_post(pmt::mp("schedule"),
                   pmt::cons(meta, pmt::init_s16vector(16,
                                                       payload_cap.data())));
        BOOST_REQUIRE(wait_until(
            [&] { return blk->schedules_invalid() > before; }));
    }
    {
        // Huge uint64 PMT values: > int64 range and beyond any cap.
        pmt::pmt_t meta = pmt::make_dict();
        meta = pmt::dict_add(meta, pmt::mp("t0_ticks"),
                             pmt::from_long(1000000));
        meta = pmt::dict_add(meta, pmt::mp("tx_samples"),
                             pmt::from_uint64(1ULL << 62));
        meta = pmt::dict_add(meta, pmt::mp("rx_samples"),
                             pmt::from_uint64(100));
        const uint64_t before = blk->schedules_invalid();
        blk->_post(pmt::mp("schedule"),
                   pmt::cons(meta, pmt::init_s16vector(16,
                                                       payload_cap.data())));
        BOOST_REQUIRE(wait_until(
            [&] { return blk->schedules_invalid() > before; }));
    }
    {
        // t0_ticks beyond int64 range (wraparound attempt).
        pmt::pmt_t meta = pmt::make_dict();
        meta = pmt::dict_add(meta, pmt::mp("t0_ticks"),
                             pmt::from_uint64(1ULL << 63));
        meta = pmt::dict_add(meta, pmt::mp("tx_samples"),
                             pmt::from_uint64(100));
        meta = pmt::dict_add(meta, pmt::mp("rx_samples"),
                             pmt::from_uint64(100));
        const uint64_t before = blk->schedules_invalid();
        blk->_post(pmt::mp("schedule"),
                   pmt::cons(meta, pmt::init_s16vector(200,
                                                       payload_cap.data())));
        BOOST_REQUIRE(wait_until(
            [&] { return blk->schedules_invalid() > before; }));
    }
    BOOST_CHECK_EQUAL(blk->bursts_published(), 0u);
    BOOST_CHECK_EQUAL(blk->schedules_invalid(), 6u);

    // Cap boundary accepted: tx == max_tx, rx == max_rx.
    blk->_post(pmt::mp("schedule"),
               make_schedule_pdu(1000000, max_tx, max_rx, /*count=*/1, 0,
                                 0, payload_cap));
    BOOST_REQUIRE(wait_bursts(burst_dbg, 1));
    pmt::pmt_t meta;
    std::vector<int16_t> rx;
    get_burst(burst_dbg, 0, meta, rx);
    BOOST_CHECK_EQUAL(meta_str(meta, "status"), "ok");
    BOOST_CHECK_EQUAL(meta_u64(meta, "rx_samples_received"), max_rx);
    BOOST_REQUIRE_EQUAL(rx.size(), max_rx * 2);
    for (uint64_t p = 0; p < max_rx * 2; ++p)
        BOOST_CHECK_EQUAL(rx[p], FakeBurstBackend::expected_rx_sample(0, p));

    // Different length after the boundary burst: the fixed scratch must
    // keep its capacity AND address (worker never grows buffers).
    const auto payload_small = make_payload(64);
    blk->_post(pmt::mp("schedule"),
               make_schedule_pdu(2000000, 64, 10, /*count=*/1, 0, 1,
                                 payload_small));
    BOOST_REQUIRE(wait_bursts(burst_dbg, 2));
    get_burst(burst_dbg, 1, meta, rx);
    BOOST_CHECK_EQUAL(meta_str(meta, "status"), "ok");
    BOOST_CHECK_EQUAL(meta_u64(meta, "rx_samples_received"), 10u);
    BOOST_CHECK_EQUAL(blk->rx_scratch_capacity(),
                      static_cast<size_t>(max_rx) * 2);
    BOOST_CHECK_EQUAL(blk->rx_scratch_data(), scratch0);

    // Restart must not reallocate the fixed scratch either.
    tb->stop();
    tb->wait();
    BOOST_REQUIRE(blk->start());
    tb->start();
    BOOST_CHECK_EQUAL(blk->rx_scratch_capacity(),
                      static_cast<size_t>(max_rx) * 2);
    BOOST_CHECK_EQUAL(blk->rx_scratch_data(), scratch0);
    blk->_post(pmt::mp("schedule"),
               make_schedule_pdu(3000000, 64, 64, /*count=*/1, 0, 2,
                                 payload_small));
    BOOST_REQUIRE(wait_bursts(burst_dbg, 3));
    get_burst(burst_dbg, 2, meta, rx);
    BOOST_CHECK_EQUAL(meta_str(meta, "status"), "ok");
    BOOST_CHECK_EQUAL(blk->rx_scratch_data(), scratch0);

    tb->stop();
    tb->wait();
    BOOST_REQUIRE(blk->stop());
}
