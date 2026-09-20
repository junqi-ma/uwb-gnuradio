/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * QA for the UhdBurstBackend contract surface that is UHD-FREE (Radar
 * Step 11).  Covers: configuration validation, strict rate-readback
 * policy, UHD error-code → BurstStatus mappings (RX stream codes and TX
 * async event bits), the device-unavailable message contract, device-time
 * tick conversions, planned-burst fragment flag validation, the dry-run
 * plan (build + print) for the default radar contract, and one
 * EchoTimer + FakeBurstBackend run with UHD-shaped burst geometry
 * (proving the injected-backend contract accepts the real backend's
 * parameter space — the "fake adapter" QA the Step 11 plan requires).
 *
 * This TU must compile and run on machines WITHOUT UHD: it includes no
 * UHD header and does not reference UhdBurstBackend (whose TU only builds
 * when CMake found UHD).
 */

// Regression guard (must stay the FIRST include of this TU):
// uwb_radar_checked_math.h is a shared header and must be self-contained.
#include <gnuradio/uwb/uwb_radar_checked_math.h>

#include <boost/test/unit_test.hpp>

#include <gnuradio/blocks/message_debug.h>
#include <gnuradio/top_block.h>
#include <gnuradio/uwb/uwb_echo_burst_backend.h>
#include <gnuradio/uwb/uwb_echo_scheduler_core.h>
#include <gnuradio/uwb/uwb_fake_burst_backend.h>
#include <gnuradio/uwb/uwb_realtime_echo_timer.h>
#include <gnuradio/uwb/uwb_uhd_backend_config.h>
#include <pmt/pmt.h>

#include <cmath>
#include <chrono>
#include <cstdint>
#include <functional>
#include <limits>
#include <string>
#include <thread>
#include <vector>

using gr::uwb::UwbRealtimeEchoTimer;
namespace echo = gr::uwb::echo;
using namespace gr::uwb::echo;
namespace uhd_cfg = gr::uwb::uhd;

BOOST_AUTO_TEST_SUITE(uhd_backend)

namespace {

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
    return pred();
}

std::vector<int16_t>
make_payload(uint64_t tx_samples)
{
    std::vector<int16_t> payload(static_cast<size_t>(tx_samples) * 2);
    for (size_t i = 0; i < payload.size(); ++i)
        payload[i] = static_cast<int16_t>((i * 257 + 3) % 30000 - 15000);
    return payload;
}

pmt::pmt_t
make_schedule_pdu(int64_t t0,
                  uint64_t tx_samples,
                  uint64_t rx_samples,
                  uint64_t burst_count,
                  uint64_t index,
                  uint64_t pulse_id,
                  const std::vector<int16_t>& payload)
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
    meta = pmt::dict_add(meta, pmt::mp("sample_rate"),
                         pmt::from_double(737280000.0));
    return pmt::cons(meta,
                     pmt::init_s16vector(payload.size(), payload.data()));
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

// Default radar contract pieces used by several cases.
uhd_cfg::UhdBurstBackendConfig
radar_config()
{
    uhd_cfg::UhdBurstBackendConfig cfg;
    cfg.device_args = "addr=192.168.10.2";
    cfg.sample_rate_hz = 737280000.0;
    cfg.clock_source = "internal";
    cfg.time_source = "internal";
    cfg.tx_channel = 0;
    cfg.rx_channel = 1;
    return cfg;
}

echo::EchoSchedulerConfig
radar_sched_base()
{
    echo::EchoSchedulerConfig sched;
    sched.pri_num = 3686400; // 5 ms @ 737.28 MS/s
    sched.pri_den = 1;
    sched.pre_guard_ticks = 1475; // 2 µs
    sched.max_fragment_size = 65536;
    return sched;
}

echo::EchoSchedulerPrepared
radar_grid()
{
    echo::EchoSchedulerPrepared grid;
    BOOST_REQUIRE(
        gr::uwb::echo::prepare_echo_scheduler(radar_sched_base(), grid));
    return grid;
}

} // namespace

// ---------------------------------------------------------------------------
// 1. Backend configuration validation.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_uhd_backend_config_validation)
{
    std::string err;
    BOOST_CHECK(uhd_cfg::validate_uhd_burst_backend_config(radar_config(),
                                                           &err));

    auto expect_fail = [](uhd_cfg::UhdBurstBackendConfig cfg) {
        std::string e;
        const bool ok = uhd_cfg::validate_uhd_burst_backend_config(cfg, &e);
        BOOST_CHECK(!ok);
        BOOST_CHECK(!e.empty());
    };

    auto cfg = radar_config();
    cfg.sample_rate_hz = 0.0;
    expect_fail(cfg);
    cfg.sample_rate_hz = -1.0;
    expect_fail(cfg);
    cfg.sample_rate_hz = std::numeric_limits<double>::infinity();
    expect_fail(cfg);
    cfg.sample_rate_hz = std::numeric_limits<double>::quiet_NaN();
    expect_fail(cfg);
    cfg.sample_rate_hz = 750e6;
    expect_fail(cfg);

    cfg = radar_config();
    cfg.sample_rate_hz = 491520000.0;
    BOOST_CHECK(uhd_cfg::validate_uhd_burst_backend_config(cfg, &err));

    cfg = radar_config();
    cfg.rate_tolerance_rel = 0.0;
    expect_fail(cfg);
    cfg.rate_tolerance_rel = 1e-3; // not strict enough
    expect_fail(cfg);

    cfg = radar_config();
    cfg.send_timeout_s = 0.0;
    expect_fail(cfg);
    cfg.send_timeout_s = 11.0;
    expect_fail(cfg);
    cfg.recv_timeout_s = -0.1;
    expect_fail(cfg);
    cfg.recv_timeout_s = std::numeric_limits<double>::infinity();
    expect_fail(cfg);

    cfg = radar_config();
    cfg.center_freq_hz = -1.0;
    expect_fail(cfg);
    cfg.center_freq_hz = std::numeric_limits<double>::quiet_NaN();
    expect_fail(cfg);

    // RX-only tune offset: finite and |offset| <= 1e6; the realistic CFO
    // correction and the 0 sentinel stay valid.
    cfg = radar_config();
    cfg.rx_freq_offset_hz = std::numeric_limits<double>::quiet_NaN();
    expect_fail(cfg);
    cfg.rx_freq_offset_hz = 2.0e6;
    expect_fail(cfg);
    cfg.rx_freq_offset_hz = -2.0e6;
    expect_fail(cfg);
    cfg.rx_freq_offset_hz = -23.9e3;
    BOOST_CHECK(uhd_cfg::validate_uhd_burst_backend_config(cfg, &err));
    cfg.rx_freq_offset_hz = 0.0;
    BOOST_CHECK(uhd_cfg::validate_uhd_burst_backend_config(cfg, &err));

    cfg = radar_config();
    cfg.tx_gain_db = std::numeric_limits<double>::quiet_NaN();
    expect_fail(cfg);
    cfg.tx_gain_db = 150.0; // set but out of range
    expect_fail(cfg);
    cfg.rx_gain_db = 150.0;
    expect_fail(cfg);

    // Negative gains stay valid (the "leave untouched" sentinel).
    cfg = radar_config();
    cfg.tx_gain_db = -1.0;
    cfg.rx_gain_db = -1.0;
    BOOST_CHECK(uhd_cfg::validate_uhd_burst_backend_config(cfg, &err));
    // An explicit 0 dB gain is a real setting, not the sentinel.
    cfg.tx_gain_db = 0.0;
    cfg.rx_gain_db = 0.0;
    BOOST_CHECK(uhd_cfg::validate_uhd_burst_backend_config(cfg, &err));
}

// ---------------------------------------------------------------------------
// 2. Strict rate readback policy: exact / within tolerance / coerced.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_uhd_backend_rate_strict)
{
    const double rate = 737280000.0;
    BOOST_CHECK(uhd_cfg::rate_matches_strict(rate, rate, 1e-9));
    // Sub-tolerance fp noise (a readback of the same double through UHD).
    BOOST_CHECK(uhd_cfg::rate_matches_strict(rate, rate * (1.0 + 1e-12),
                                             1e-9));
    // Silent coercion: even 1 kHz off is rejected.
    BOOST_CHECK(!uhd_cfg::rate_matches_strict(rate, rate + 1000.0, 1e-9));
    BOOST_CHECK(!uhd_cfg::rate_matches_strict(rate, rate * 0.75, 1e-9));
    BOOST_CHECK(!uhd_cfg::rate_matches_strict(rate, 1e8, 1e-9));
    BOOST_CHECK(!uhd_cfg::rate_matches_strict(rate, rate, 0.0)); // bad tol
}

// ---------------------------------------------------------------------------
// 3. UHD ABI constants and error mappings.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_uhd_backend_error_mapping)
{
    // Documented ABI values (REAL UHD 4.1 enums — NOT contiguous:
    // late=0x2, broken=0x4, overflow=0x8; cross-checked with static_assert
    // in the UHD-linked TU; asserted here so a constant typo cannot hide).
    BOOST_CHECK_EQUAL(uhd_cfg::kUhdRxErrorNone, 0x0);
    BOOST_CHECK_EQUAL(uhd_cfg::kUhdRxErrorTimeout, 0x1);
    BOOST_CHECK_EQUAL(uhd_cfg::kUhdRxErrorLateCommand, 0x2);
    BOOST_CHECK_EQUAL(uhd_cfg::kUhdRxErrorBrokenChain, 0x4);
    BOOST_CHECK_EQUAL(uhd_cfg::kUhdRxErrorOverflow, 0x8);
    BOOST_CHECK_EQUAL(uhd_cfg::kUhdAsyncBurstAck, 0x1u);
    BOOST_CHECK_EQUAL(uhd_cfg::kUhdAsyncUnderflow, 0x2u);
    BOOST_CHECK_EQUAL(uhd_cfg::kUhdAsyncSeqError, 0x4u);
    BOOST_CHECK_EQUAL(uhd_cfg::kUhdAsyncTimeError, 0x8u);
    BOOST_CHECK_EQUAL(uhd_cfg::kUhdAsyncUnderflowInPacket, 0x10u);
    BOOST_CHECK_EQUAL(uhd_cfg::kUhdAsyncSeqErrorInBurst, 0x20u);
    BOOST_CHECK_EQUAL(uhd_cfg::kUhdAsyncUserPayload, 0x40u);

    // RX stream error codes.
    using S = BurstStatus;
    BOOST_CHECK(uhd_cfg::map_uhd_rx_error_code(0x0) == S::Ok);
    BOOST_CHECK(uhd_cfg::map_uhd_rx_error_code(0x1) == S::Timeout);
    BOOST_CHECK(uhd_cfg::map_uhd_rx_error_code(0x2) == S::LateCommand);
    BOOST_CHECK(uhd_cfg::map_uhd_rx_error_code(0x4) == S::BrokenChain);
    BOOST_CHECK(uhd_cfg::map_uhd_rx_error_code(0x8) == S::Overflow);
    BOOST_CHECK(uhd_cfg::map_uhd_rx_error_code(0xc) == S::BackendError);
    BOOST_CHECK(uhd_cfg::map_uhd_rx_error_code(0xf) == S::BackendError);
    BOOST_CHECK(uhd_cfg::map_uhd_rx_error_code(99) == S::BackendError);
    BOOST_CHECK(uhd_cfg::map_uhd_rx_error_code(-1) == S::BackendError);

    // TX async event bits.
    std::string note;
    BOOST_CHECK(uhd_cfg::map_uhd_tx_async_event(0, note) == S::Ok);
    BOOST_CHECK(uhd_cfg::map_uhd_tx_async_event(
                    uhd_cfg::kUhdAsyncBurstAck, note) == S::Ok);
    // USER_PAYLOAD is informational, not an error.
    note.clear();
    BOOST_CHECK(uhd_cfg::map_uhd_tx_async_event(
                    uhd_cfg::kUhdAsyncUserPayload, note) == S::Ok);
    note.clear();
    BOOST_CHECK(uhd_cfg::map_uhd_tx_async_event(
                    uhd_cfg::kUhdAsyncTimeError, note) == S::LateCommand);
    BOOST_CHECK_EQUAL(note, "tx_time_error");
    note.clear();
    BOOST_CHECK(uhd_cfg::map_uhd_tx_async_event(
                    uhd_cfg::kUhdAsyncUnderflow, note) == S::BackendError);
    BOOST_CHECK_EQUAL(note, "tx_underflow");
    note.clear();
    BOOST_CHECK(uhd_cfg::map_uhd_tx_async_event(
                    uhd_cfg::kUhdAsyncSeqErrorInBurst, note) ==
                S::BackendError);
    BOOST_CHECK_EQUAL(note, "tx_seq_error");
    note.clear();
    // Combined bits: late command wins; all notes listed.
    BOOST_CHECK(
        uhd_cfg::map_uhd_tx_async_event(
            uhd_cfg::kUhdAsyncTimeError | uhd_cfg::kUhdAsyncBurstAck,
            note) == S::LateCommand);
    BOOST_CHECK_EQUAL(note, "tx_time_error");
    note.clear();
    BOOST_CHECK(uhd_cfg::map_uhd_tx_async_event(
                    uhd_cfg::kUhdAsyncUnderflowInPacket |
                        uhd_cfg::kUhdAsyncSeqError,
                    note) == S::BackendError);
    BOOST_CHECK_EQUAL(note, "tx_underflow, tx_seq_error");
    note.clear();
    // Unknown high bits are surfaced as backend errors, not dropped.
    BOOST_CHECK(uhd_cfg::map_uhd_tx_async_event(0x80u, note) ==
                S::BackendError);
    BOOST_CHECK_EQUAL(note, "tx_unknown_event");
}

// ---------------------------------------------------------------------------
// 4. Device-unavailable message contract.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_uhd_backend_device_unavailable_message)
{
    const std::string msg = uhd_cfg::make_device_unavailable_error(
        "no devices found for ''");
    BOOST_CHECK_EQUAL(
        msg, std::string("device unavailable: no devices found for ''"));
    BOOST_CHECK(msg.find("device unavailable:") == 0u);
}

// ---------------------------------------------------------------------------
// 5. Device-time tick conversions (deterministic round trip).
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_uhd_backend_tick_conversions)
{
    const double rate = 737280000.0;
    const std::vector<int64_t> ticks_vec = {
        0, 1, 1475, 55591, 3686400, 737279999, 140982, 1'000'000'000'000
    };
    for (const int64_t t : ticks_vec) {
        int64_t full = 0;
        double frac = 0.0;
        BOOST_REQUIRE(uhd_cfg::time_parts_from_ticks(t, rate, full, frac));
        BOOST_CHECK_GE(full, 0);
        BOOST_CHECK(frac >= 0.0 && frac < 1.0);
        int64_t back = -1;
        BOOST_REQUIRE(
            uhd_cfg::ticks_from_time_parts(full, frac, rate, back));
        BOOST_CHECK_EQUAL(back, t);
    }

    // Invalid inputs fail explicitly (never wrap around).
    int64_t full = 0;
    double frac = 0.0;
    int64_t ticks = 0;
    BOOST_CHECK(!uhd_cfg::time_parts_from_ticks(-1, rate, full, frac));
    BOOST_CHECK(!uhd_cfg::time_parts_from_ticks(10, 0.0, full, frac));
    BOOST_CHECK(!uhd_cfg::ticks_from_time_parts(-1, 0.5, rate, ticks));
    BOOST_CHECK(!uhd_cfg::ticks_from_time_parts(1, 1.0, rate, ticks));
    BOOST_CHECK(!uhd_cfg::ticks_from_time_parts(1, -0.1, rate, ticks));
    BOOST_CHECK(!uhd_cfg::ticks_from_time_parts(1, 0.5, 0.0, ticks));
}

// ---------------------------------------------------------------------------
// 6. Planned-burst fragment flag validation.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_uhd_backend_fragment_flags)
{
    std::string err;
    BurstFragment one{};
    one.count = 100;
    one.flags = kFlagTimeSpec | kFlagStartOfBurst | kFlagEndOfBurst;
    BOOST_CHECK(uhd_cfg::validate_fragment_flags(&one, 1, &err));

    BurstFragment three[3];
    three[0] = BurstFragment{};
    three[0].count = 10;
    three[0].flags = kFlagTimeSpec | kFlagStartOfBurst;
    three[1] = BurstFragment{};
    three[1].count = 10;
    three[2] = BurstFragment{};
    three[2].count = 10;
    three[2].flags = kFlagEndOfBurst;
    BOOST_CHECK(uhd_cfg::validate_fragment_flags(three, 3, &err));

    auto expect_fail = [](const BurstFragment* f, size_t n) {
        std::string e;
        const bool ok = uhd_cfg::validate_fragment_flags(f, n, &e);
        BOOST_CHECK(!ok);
        BOOST_CHECK(!e.empty());
    };
    expect_fail(nullptr, 1);
    expect_fail(three, 0);

    BurstFragment bad[2];
    bad[0] = BurstFragment{};
    bad[0].count = 10;
    bad[0].flags = kFlagTimeSpec; // missing SOB
    bad[1] = BurstFragment{};
    bad[1].count = 10;
    bad[1].flags = kFlagEndOfBurst;
    expect_fail(bad, 2);

    bad[0] = BurstFragment{};
    bad[0].count = 10;
    bad[0].flags = kFlagTimeSpec | kFlagStartOfBurst;
    bad[1] = BurstFragment{};
    bad[1].count = 10; // missing EOB
    expect_fail(bad, 2);

    bad[0] = BurstFragment{};
    bad[0].count = 10;
    bad[0].flags = kFlagTimeSpec | kFlagStartOfBurst;
    bad[1] = BurstFragment{};
    bad[1].count = 10;
    bad[1].flags = kFlagStartOfBurst | kFlagEndOfBurst; // mid SOB + EOB
    expect_fail(bad, 2);

    BurstFragment zero{};
    zero.count = 0;
    zero.flags = kFlagTimeSpec | kFlagStartOfBurst | kFlagEndOfBurst;
    expect_fail(&zero, 1);
}

// ---------------------------------------------------------------------------
// 7. Dry-run plan: default radar contract, exact math, failure modes.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_uhd_backend_dry_run_plan)
{
    const auto cfg = radar_config();
    const auto grid = radar_grid();
    const int64_t t0 = 36864000; // 0.05 s arm delay
    uhd_cfg::UhdDryRunPlan plan;
    std::string err;
    BOOST_REQUIRE(uhd_cfg::build_uhd_dry_run_plan(
        cfg, grid, t0, 0, /*tx=*/140982, /*rx=*/55591, plan, &err));

    // Echoed configuration.
    BOOST_CHECK_EQUAL(plan.device_args, "addr=192.168.10.2");
    BOOST_CHECK_EQUAL(plan.sample_rate_hz, 737280000.0);
    BOOST_CHECK_EQUAL(plan.tx_channel, 0u);
    BOOST_CHECK_EQUAL(plan.rx_channel, 1u);
    BOOST_CHECK_EQUAL(plan.clock_source, "internal");
    BOOST_CHECK_EQUAL(plan.time_source, "internal");

    // Scheduler contract (exact integer ticks).
    BOOST_CHECK_EQUAL(plan.pri_num, 3686400);
    BOOST_CHECK_EQUAL(plan.pri_den, 1);
    BOOST_CHECK_EQUAL(plan.pre_guard_ticks, 1475);
    BOOST_CHECK_EQUAL(plan.max_fragment_size, 65536u);

    // Geometry: RX strictly before TX, exact offset.
    BOOST_CHECK_EQUAL(plan.t0_ticks, t0);
    BOOST_CHECK_EQUAL(plan.tx_ticks, t0);
    BOOST_CHECK_EQUAL(plan.rx_ticks, t0 - 1475);
    BOOST_CHECK_LT(plan.rx_ticks, plan.tx_ticks);
    BOOST_CHECK_EQUAL(plan.tx_samples, 140982u);
    BOOST_CHECK_EQUAL(plan.rx_samples, 55591u);

    // Derived values.
    BOOST_CHECK(std::fabs(plan.pri_s - 0.005) < 1e-15);
    BOOST_CHECK(std::fabs(plan.pre_guard_s - 1475.0 / 737280000.0) < 1e-15);
    BOOST_CHECK(std::fabs(plan.tx_window_us -
                          140982.0 / 737280000.0 * 1e6) < 1e-9);
    BOOST_CHECK_EQUAL(plan.tx_bytes, 140982u * 4);
    BOOST_CHECK_EQUAL(plan.rx_bytes, 55591u * 4);
    BOOST_CHECK_EQUAL(plan.tx_fragments, 3u);  // ceil(140982 / 65536)
    BOOST_CHECK_EQUAL(plan.rx_fragments, 1u);  // ceil(55591 / 65536)

    // Fractional PRI stays exact in the rational representation.
    echo::EchoSchedulerConfig fsched;
    fsched.pri_num = 1;
    fsched.pri_den = 3; // 1/3 tick — pathological but exact
    fsched.pre_guard_ticks = 0;
    echo::EchoSchedulerPrepared fgrid;
    BOOST_REQUIRE(gr::uwb::echo::prepare_echo_scheduler(fsched, fgrid));
    uhd_cfg::UhdDryRunPlan fplan;
    BOOST_REQUIRE(uhd_cfg::build_uhd_dry_run_plan(
        cfg, fgrid, 10, 0, 64, 64, fplan, &err));
    BOOST_CHECK_EQUAL(fplan.pri_num, 1);
    BOOST_CHECK_EQUAL(fplan.pri_den, 3);

    // Failure modes: t0 must leave room for the RX pre-guard window.
    // t0 == pre_guard → rx_ticks = 0 → legal (the window just starts at
    // device time zero); only t0 < pre_guard fails.
    BOOST_CHECK(uhd_cfg::build_uhd_dry_run_plan(
        cfg, grid, 1475, 0, 140982, 55591, plan, &err));
    BOOST_CHECK(!uhd_cfg::build_uhd_dry_run_plan(
        cfg, grid, 1474, 0, 140982, 55591, plan, &err));
    BOOST_CHECK(!uhd_cfg::build_uhd_dry_run_plan(
        cfg, grid, t0, 0, 0, 55591, plan, &err));
    BOOST_CHECK(!uhd_cfg::build_uhd_dry_run_plan(
        cfg, grid, t0, 0, 140982, 0, plan, &err));
    // Too many fragments (max_fragment_size = 1, burst > 64).
    echo::EchoSchedulerConfig sched1 = radar_sched_base();
    sched1.max_fragment_size = 1;
    echo::EchoSchedulerPrepared grid1;
    BOOST_REQUIRE(gr::uwb::echo::prepare_echo_scheduler(sched1, grid1));
    BOOST_CHECK(!uhd_cfg::build_uhd_dry_run_plan(
        cfg, grid1, t0, 0, 100, 100, plan, &err));
    // Invalid config propagates.
    auto bad = radar_config();
    bad.sample_rate_hz = 0.0;
    BOOST_CHECK(!uhd_cfg::build_uhd_dry_run_plan(
        bad, grid, t0, 0, 140982, 55591, plan, &err));
}

// ---------------------------------------------------------------------------
// 8. Dry-run print: the full contract is on stdout, no-device asserted.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_uhd_backend_dry_run_print)
{
    const auto plan = [] {
        uhd_cfg::UhdDryRunPlan p;
        std::string err;
        BOOST_REQUIRE(uhd_cfg::build_uhd_dry_run_plan(
            radar_config(), radar_grid(), 36864000, 0, 140982, 55591, p,
            &err));
        return p;
    }();
    const std::string text = uhd_cfg::print_uhd_dry_run_plan(plan);

    BOOST_CHECK(text.find("DRY-RUN (no device opened)") !=
                std::string::npos);
    BOOST_CHECK(text.find("device_args              = addr=192.168.10.2") !=
                std::string::npos);
    BOOST_CHECK(text.find("sample_rate_hz           = 737280000") !=
                std::string::npos);
    BOOST_CHECK(text.find("silent coercion rejected") != std::string::npos);
    BOOST_CHECK(text.find("otw_format               = sc16") !=
                std::string::npos);
    BOOST_CHECK(text.find("pri_ticks                = 3686400/1") !=
                std::string::npos);
    BOOST_CHECK(text.find("pre_guard_ticks          = 1475") !=
                std::string::npos);
    BOOST_CHECK(text.find("(rx BEFORE tx)") != std::string::npos);
    BOOST_CHECK(text.find("tx_samples / rx_samples  = 140982 / 55591") !=
                std::string::npos);
    BOOST_CHECK(text.find("issue_rx(NUM_SAMPS_AND_DONE, timed) BEFORE "
                          "issue_tx(send loop)") != std::string::npos);
    BOOST_CHECK(text.find("NUM_SAMPS_AND_DONE") != std::string::npos);
    BOOST_CHECK(text.find("0x0:none 0x1:timeout 0x2:late_command "
                          "0x4:broken_chain 0x8:overflow") !=
                std::string::npos);
    BOOST_CHECK(text.find("time_error:late_command") != std::string::npos);
    BOOST_CHECK(text.find("dry-run complete: no device was opened") !=
                std::string::npos);
}

// ---------------------------------------------------------------------------
// 9. Fake adapter: the EchoTimer + FakeBurstBackend contract accepts the
//    UHD-shaped burst geometry (real backend's parameter space) and
//    produces exactly one ok burst per index with the expected sample
//    counts — the parameter-validation / error-mapping QA the Step 11
//    plan requires while real UHD is not available on this machine.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_uhd_backend_fake_adapter_geometry)
{
    // UHD-shaped geometry: the full 64-SYNC native TX packet, the ~75.4 µs
    // RX window, small fragments (like a real device's transport MTU) and
    // a per-call chunk cap forcing partial send/recv inside the backend.
    const uint64_t tx_samples = 140982;
    const uint64_t rx_samples = 55591;

    echo::EchoSchedulerConfig sched = radar_sched_base();
    FakeBurstBackend::Config fcfg;
    fcfg.max_io_chunk = 30000; // forces TX (5 fragments) AND RX re-issues
    auto fake = std::make_shared<FakeBurstBackend>(fcfg);
    auto blk = gr::uwb::UwbRealtimeEchoTimer::make(sched, fake, 16,
                                                   /*wait_ms=*/1000);
    auto burst_dbg = gr::blocks::message_debug::make();
    auto tb = gr::make_top_block("qa_uhd_backend_fake_adapter");
    tb->msg_connect(blk, "burst", burst_dbg, "store");
    BOOST_REQUIRE(blk->start());
    tb->start();

    const int64_t t0 = 36864000;
    blk->_post(pmt::mp("schedule"),
               make_schedule_pdu(t0, tx_samples, rx_samples, /*count=*/2, 0,
                                 42, make_payload(tx_samples)));
    BOOST_REQUIRE(wait_until(
        [&] { return burst_dbg->num_messages() >= 2; }));

    const int64_t pri = 3686400;
    for (uint64_t k = 0; k < 2; ++k) {
        pmt::pmt_t meta;
        std::vector<int16_t> rx;
        pmt::pmt_t msg = burst_dbg->get_message(k);
        BOOST_REQUIRE(pmt::is_pair(msg));
        meta = pmt::car(msg);
        pmt::pmt_t data = pmt::cdr(msg);
        BOOST_REQUIRE(pmt::is_s16vector(data));
        size_t len = 0;
        pmt::s16vector_elements(data, len);
        BOOST_CHECK_EQUAL(meta_str(meta, "status"), "partial_handled");
        BOOST_CHECK_EQUAL(meta_u64(meta, "schedule_index"), k);
        BOOST_CHECK_EQUAL(meta_u64(meta, "pulse_id"), 42u);
        BOOST_CHECK_EQUAL(meta_i64(meta, "tx_ticks"),
                          t0 + static_cast<int64_t>(k) * pri);
        BOOST_CHECK_EQUAL(meta_i64(meta, "rx_ticks"),
                          t0 + static_cast<int64_t>(k) * pri - 1475);
        BOOST_CHECK_EQUAL(meta_u64(meta, "rx_samples_received"),
                          rx_samples);
        BOOST_REQUIRE_EQUAL(len, rx_samples * 2);
        BOOST_CHECK_GT(meta_u64(meta, "tx_reissues"), 0u);
        BOOST_CHECK_GT(meta_u64(meta, "rx_reissues"), 0u);
        // Stitched RX data is the deterministic fake sequence, point for
        // point (proves the fragments landed in the right order despite
        // the partial re-issues).
        const int16_t* el = pmt::s16vector_elements(data, len);
        for (uint64_t e = 0; e < rx_samples * 2; e += 7913)
            BOOST_CHECK_EQUAL(
                el[e],
                FakeBurstBackend::expected_rx_sample(k, e));
    }

    BOOST_CHECK_EQUAL(blk->bursts_published(), 2u);
    BOOST_CHECK_EQUAL(blk->bursts_ok(), 2u);
    BOOST_CHECK_EQUAL(blk->bursts_failed(), 0u);

    tb->stop();
    tb->wait();
    BOOST_REQUIRE(blk->stop());
}

// ---------------------------------------------------------------------------
// M1: multi-TX channel configuration (2/4 valid; dup/out-of-range/count
// rejected; per-channel freq/gain validated).
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_uhd_backend_multitx_config)
{
    std::string err;
    auto two_ch = [] {
        uhd_cfg::UhdBurstBackendConfig cfg;
        uhd_cfg::UhdTxChannelConfig a, b;
        a.channel = 0;
        a.gain_db = 10.0;
        a.center_freq_hz = 7.4e9;
        b.channel = 1;
        b.gain_db = 20.0;
        b.center_freq_hz = 7.5e9;
        cfg.tx_channels = { a, b };
        return cfg;
    };
    BOOST_CHECK(uhd_cfg::validate_uhd_burst_backend_config(two_ch(), &err));

    {
        uhd_cfg::UhdBurstBackendConfig cfg;
        for (size_t c = 0; c < 4; ++c) {
            uhd_cfg::UhdTxChannelConfig e;
            e.channel = c;
            cfg.tx_channels.push_back(e);
        }
        BOOST_CHECK(
            uhd_cfg::validate_uhd_burst_backend_config(cfg, &err));
        BOOST_CHECK_EQUAL(uhd_cfg::effective_tx_channels(cfg).size(), 4u);
    }

    auto expect_fail = [](uhd_cfg::UhdBurstBackendConfig cfg) {
        std::string e;
        BOOST_CHECK(!uhd_cfg::validate_uhd_burst_backend_config(cfg, &e));
        BOOST_CHECK(!e.empty());
    };
    { // duplicate physical channel
        auto cfg = two_ch();
        cfg.tx_channels[1].channel = 0;
        expect_fail(cfg);
    }
    { // out of range (>= kEchoMaxTxChannels)
        auto cfg = two_ch();
        cfg.tx_channels[1].channel = echo::kEchoMaxTxChannels;
        expect_fail(cfg);
    }
    { // too many entries
        uhd_cfg::UhdBurstBackendConfig cfg;
        for (size_t c = 0; c <= echo::kEchoMaxTxChannels; ++c) {
            uhd_cfg::UhdTxChannelConfig e;
            e.channel = c % echo::kEchoMaxTxChannels;
            cfg.tx_channels.push_back(e);
        }
        expect_fail(cfg);
    }
    { // bad per-channel freq / gain
        auto cfg = two_ch();
        cfg.tx_channels[0].center_freq_hz = -1.0;
        expect_fail(cfg);
        cfg = two_ch();
        cfg.tx_channels[0].center_freq_hz =
            std::numeric_limits<double>::quiet_NaN();
        expect_fail(cfg);
        cfg = two_ch();
        cfg.tx_channels[1].gain_db = 150.0;
        expect_fail(cfg);
        cfg = two_ch();
        cfg.tx_channels[1].gain_db =
            std::numeric_limits<double>::quiet_NaN();
        expect_fail(cfg);
    }
    { // negative gain sentinel stays valid per channel
        auto cfg = two_ch();
        cfg.tx_channels[0].gain_db = -1.0;
        cfg.tx_channels[1].gain_db = -1.0;
        BOOST_CHECK(
            uhd_cfg::validate_uhd_burst_backend_config(cfg, &err));
    }
    { // dry-run reserve follows the effective count, print frozen
        auto cfg = two_ch();
        uhd_cfg::UhdDryRunPlan plan;
        BOOST_REQUIRE(uhd_cfg::build_uhd_dry_run_plan(
            cfg, radar_grid(), 36864000, 0, 140982, 55591, plan, &err));
        BOOST_CHECK_EQUAL(plan.tx_channel_count, 2u);
        BOOST_REQUIRE_EQUAL(plan.tx_physical_channels.size(), 2u);
        BOOST_CHECK_EQUAL(plan.tx_physical_channels[0], 0u);
        BOOST_CHECK_EQUAL(plan.tx_physical_channels[1], 1u);
        BOOST_CHECK_EQUAL(plan.tx_wire_bytes, plan.tx_bytes * 2);
        const std::string text = uhd_cfg::print_uhd_dry_run_plan(plan);
        BOOST_CHECK(text.find("dry-run complete: no device was opened") !=
                    std::string::npos);
    }
}

// ---------------------------------------------------------------------------
// M1: legacy scalar → effective single-channel equivalence.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_uhd_backend_effective_single_equiv)
{
    auto cfg = radar_config();
    cfg.tx_antenna = "TX/RX";
    cfg.tx_gain_db = 12.5;
    cfg.center_freq_hz = 7.4e9;
    const auto eff = uhd_cfg::effective_tx_channels(cfg);
    BOOST_REQUIRE_EQUAL(eff.size(), 1u);
    BOOST_CHECK_EQUAL(eff[0].channel, cfg.tx_channel);
    BOOST_CHECK_EQUAL(eff[0].antenna, "TX/RX");
    BOOST_CHECK_EQUAL(eff[0].gain_db, 12.5);
    BOOST_CHECK_EQUAL(eff[0].center_freq_hz, 7.4e9);
    // Explicit vector wins verbatim over the scalars.
    uhd_cfg::UhdTxChannelConfig e;
    e.channel = 1;
    e.antenna = "TX/RX";
    e.gain_db = 3.0;
    e.center_freq_hz = 7.6e9;
    cfg.tx_channels = { e };
    const auto eff2 = uhd_cfg::effective_tx_channels(cfg);
    BOOST_REQUIRE_EQUAL(eff2.size(), 1u);
    BOOST_CHECK_EQUAL(eff2[0].channel, 1u);
    BOOST_CHECK_EQUAL(eff2[0].center_freq_hz, 7.6e9);
    // Legacy dry-run still reports a single channel.
    std::string err;
    uhd_cfg::UhdDryRunPlan plan;
    BOOST_REQUIRE(uhd_cfg::build_uhd_dry_run_plan(
        radar_config(), radar_grid(), 36864000, 0, 140982, 55591, plan,
        &err));
    BOOST_CHECK_EQUAL(plan.tx_channel_count, 1u);
    BOOST_CHECK_EQUAL(plan.tx_wire_bytes, plan.tx_bytes);
}

// ---------------------------------------------------------------------------
// M1: JamDelayRng fixed-seed repeatability and inclusive-boundary mapping.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_multitx_rng_repeat_bounds)
{
    // Same seed → identical stream (fixed PCG, no STL distribution).
    echo::JamDelayRng a(12345), b(12345);
    for (int i = 0; i < 64; ++i)
        BOOST_CHECK_EQUAL(a.next_u32(), b.next_u32());
    // Different seeds diverge (sanity: not a constant stream).
    echo::JamDelayRng c(1), d(2);
    bool differ = false;
    for (int i = 0; i < 8; ++i)
        differ = differ || (c.next_u32() != d.next_u32());
    BOOST_CHECK(differ);

    echo::JamDelayRng rng(7);
    BOOST_CHECK_EQUAL(rng.next_range_inclusive(5, 5), 5); // zero span
    BOOST_CHECK_EQUAL(rng.next_range_inclusive(0, 0), 0);
    BOOST_CHECK_EQUAL(rng.next_range_inclusive(9, 3), 9); // hi <= lo → lo
    // Closed-interval containment, incl. negative spans and full [-D, +D].
    for (int i = 0; i < 512; ++i) {
        const int64_t v = rng.next_range_inclusive(-20, 20);
        BOOST_CHECK(v >= -20 && v <= 20);
    }
    for (int i = 0; i < 256; ++i) {
        const int64_t v = rng.next_range_inclusive(-100, -1);
        BOOST_CHECK(v >= -100 && v <= -1);
    }
    // Endpoints reachable over a long draw (span 41, 4096 draws).
    bool seen_lo = false, seen_hi = false;
    for (int i = 0; i < 4096; ++i) {
        const int64_t v = rng.next_range_inclusive(-20, 20);
        seen_lo = seen_lo || (v == -20);
        seen_hi = seen_hi || (v == 20);
    }
    BOOST_CHECK(seen_lo);
    BOOST_CHECK(seen_hi);
}

// ---------------------------------------------------------------------------
// M1: prepare_multitx_geometry L formula and multitx_burst_bounds edges.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_multitx_geometry_L_bounds)
{
    std::string err;
    echo::MultiTxGeometry g;
    // L = max(D + sense, 2D + jam).
    BOOST_REQUIRE(
        echo::prepare_multitx_geometry(100, 50, 20, g, &err));
    BOOST_CHECK_EQUAL(g.phys_len_L, 120u); // max(120, 90)
    BOOST_CHECK_EQUAL(g.sense_begin, 20u);
    BOOST_REQUIRE(echo::prepare_multitx_geometry(50, 100, 20, g, &err));
    BOOST_CHECK_EQUAL(g.phys_len_L, 140u); // max(70, 140)
    // Silent jammer: legacy length when D == 0, parked at D otherwise.
    BOOST_REQUIRE(echo::prepare_multitx_geometry(100, 0, 0, g, &err));
    BOOST_CHECK_EQUAL(g.phys_len_L, 100u);
    BOOST_REQUIRE(echo::prepare_multitx_geometry(100, 0, 20, g, &err));
    BOOST_CHECK_EQUAL(g.phys_len_L, 120u);
    BOOST_CHECK(!echo::prepare_multitx_geometry(0, 50, 20, g, &err));
    BOOST_CHECK(!err.empty());

    // Boundary set: sorted unique, [0, L], delay extremes pin jam edges.
    BOOST_REQUIRE(
        echo::prepare_multitx_geometry(100, 50, 20, g, &err)); // L = 120
    uint64_t bounds[5] = {};
    const int64_t D = 20;
    for (const int64_t delay : { -D, int64_t(0), D }) {
        const size_t n =
            echo::multitx_burst_bounds(g, delay, bounds, 5);
        BOOST_CHECK(n >= 2 && n <= 5);
        BOOST_CHECK_EQUAL(bounds[0], 0u);
        BOOST_CHECK_EQUAL(bounds[n - 1], g.phys_len_L);
        for (size_t i = 1; i < n; ++i)
            BOOST_CHECK_LT(bounds[i - 1], bounds[i]);
        // Jammer window present and inside [0, L].
        const uint64_t jb = static_cast<uint64_t>(D + delay);
        bool has_jb = false, has_je = false;
        for (size_t i = 0; i < n; ++i) {
            has_jb = has_jb || (bounds[i] == jb);
            has_je = has_je || (bounds[i] == jb + 50);
        }
        BOOST_CHECK(has_jb);
        BOOST_CHECK(has_je);
    }
    // delay = -D parks jam at 0 (merges with origin); delay = +D ends at
    // 2D + jam <= L.
    BOOST_CHECK_EQUAL(
        echo::multitx_burst_bounds(g, -D, bounds, 5) >= 2, true);
    BOOST_CHECK_LE(2u * 20u + 50u, g.phys_len_L);
}

// ---------------------------------------------------------------------------
// M1: multitx_channel_ptr zero-region / slice / null-count / straddle.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_multitx_channel_ptr_slices)
{
    const uint64_t L = 120;
    std::vector<int16_t> wave(100 * 2, 0); // sense at [20, 120)
    for (size_t i = 0; i < wave.size(); ++i)
        wave[i] = static_cast<int16_t>(i % 1000);
    std::vector<int16_t> zero(L * 2, 0);
    // Inside the waveform: slice pointer with element-pair stride.
    const int16_t* p =
        echo::multitx_channel_ptr(wave.data(), 20, 100, zero.data(), 30, 10);
    BOOST_CHECK_EQUAL(p, wave.data() + (30 - 20) * 2);
    BOOST_CHECK_EQUAL(p[0], wave[(30 - 20) * 2]);
    // Zero region: shared scratch at the same offset.
    const int16_t* z =
        echo::multitx_channel_ptr(wave.data(), 20, 100, zero.data(), 0, 20);
    BOOST_CHECK_EQUAL(z, zero.data() + 0 * 2);
    const int16_t* z2 =
        echo::multitx_channel_ptr(wave.data(), 20, 100, zero.data(), 5, 5);
    BOOST_CHECK_EQUAL(z2, zero.data() + 5 * 2);
    // Empty fragment: nullptr (no pointer to resolve).
    BOOST_CHECK(echo::multitx_channel_ptr(wave.data(), 20, 100, zero.data(),
                                          30, 0) == nullptr);
    // Straddling fragment: never OOB — falls back to zero scratch so a
    // planner bug fails loudly in sample checks, not in memory.
    const int16_t* s = echo::multitx_channel_ptr(wave.data(), 20, 100,
                                                 zero.data(), 15, 10);
    BOOST_CHECK_EQUAL(s, zero.data() + 15 * 2);
}

BOOST_AUTO_TEST_SUITE_END()
