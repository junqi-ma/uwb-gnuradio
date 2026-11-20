/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * UHD-gated runtime QA for UhdBurstBackend (Radar Step 11).  Built ONLY
 * when UHD is available (UWB_HAVE_UHD).  Verifies, WITHOUT any USRP:
 *
 *   - a missing/unreachable device makes prepare() fail cleanly with the
 *     "device unavailable: ..." contract message — no throw escapes the
 *     backend interface, no crash, no hang (skipped automatically when a
 *     device IS present, so this QA is safe on hardware machines);
 *   - the unprepared backend is inert: issue_rx / issue_tx /
 *     collect_result / abort_rx / device_time_ticks are safe no-ops or
 *     BackendError/StopDuringIo results;
 *   - the constructor validates the frozen config (invalid_argument).
 */

#include <boost/test/unit_test.hpp>

#include <gnuradio/uwb/uwb_echo_burst_backend.h>
#include <gnuradio/uwb/uwb_uhd_burst_backend.h>
#include <gnuradio/uwb/uwb_uhd_backend_config.h>

#include <uhd/device.hpp>

#include <chrono>
#include <stdexcept>
#include <string>

using namespace gr::uwb::echo;
namespace uhd_cfg = gr::uwb::uhd;

BOOST_AUTO_TEST_SUITE(uhd_backend_device)

namespace {

uhd_cfg::UhdBurstBackendConfig
radar_config()
{
    uhd_cfg::UhdBurstBackendConfig cfg;
    cfg.device_args = "";
    cfg.sample_rate_hz = 737280000.0;
    cfg.tx_channel = 0;
    cfg.rx_channel = 1;
    return cfg;
}

// True when UHD can discover at least one device with the given args.
bool devices_present(const std::string& args)
{
    try {
        return !::uhd::device::find(::uhd::device_addr_t(args)).empty();
    } catch (const std::exception&) {
        return false; // discovery itself failing counts as "unavailable"
    }
}

} // namespace

// The unprepared backend is inert and never throws across the interface.
BOOST_AUTO_TEST_CASE(test_uhd_backend_unprepared_is_inert)
{
    uhd_cfg::UhdBurstBackend backend(radar_config());

    BOOST_CHECK(!backend.stop_requested());
    BOOST_CHECK_EQUAL(backend.device_time_ticks(), 0);

    std::string err;
    RxCommand rx{};
    rx.schedule_index = 0;
    rx.rx_ticks = 36862525;
    rx.total_samples = 100;
    BOOST_CHECK(backend.issue_rx(rx, err) == BurstStatus::BackendError);
    BOOST_CHECK(!err.empty());

    int16_t buf[8] = {};
    BurstFragment frag{};
    frag.rx_data = buf;
    frag.count = 4;
    frag.flags = kFlagTimeSpec | kFlagStartOfBurst | kFlagEndOfBurst;
    rx.fragments = &frag;
    rx.fragment_count = 1;
    BOOST_CHECK(backend.issue_rx(rx, err) == BurstStatus::BackendError);

    TxCommand tx{};
    tx.schedule_index = 0;
    tx.tx_ticks = 36864000;
    tx.total_samples = 4;
    tx.fragments = &frag;
    tx.fragment_count = 1;
    BOOST_CHECK(backend.issue_tx(tx, err) == BurstStatus::BackendError);

    BurstResult out;
    BOOST_CHECK(backend.collect_result(0, 10, out));
    BOOST_CHECK(out.status == BurstStatus::BackendError);

    backend.abort_rx(); // no-op, must not throw
    backend.request_stop();
    BOOST_CHECK(backend.stop_requested());
}

// Constructor rejects an invalid frozen config.
BOOST_AUTO_TEST_CASE(test_uhd_backend_constructor_validation)
{
    auto cfg = radar_config();
    cfg.sample_rate_hz = 0.0;
    try {
        uhd_cfg::UhdBurstBackend backend(cfg);
        BOOST_FAIL("expected std::invalid_argument");
    } catch (const std::invalid_argument&) {
        BOOST_CHECK(true);
    }
}

// prepare() on a machine with UHD but no USRP: clean, explicit, no
// crash, no hang.  Skipped when a device is actually present.
BOOST_AUTO_TEST_CASE(test_uhd_backend_device_unavailable)
{
    if (devices_present("")) {
        BOOST_TEST_MESSAGE(
            "USRP device(s) present; skipping the device-unavailable case");
        return;
    }
    uhd_cfg::UhdBurstBackend backend(radar_config());
    std::string err;
    const bool ok = backend.prepare(err);
    BOOST_CHECK(!ok);
    BOOST_CHECK(!err.empty());
    BOOST_CHECK_MESSAGE(err.find("device unavailable:") == 0,
                        "expected 'device unavailable:' prefix, got: "
                            << err);
}

BOOST_AUTO_TEST_SUITE_END()
