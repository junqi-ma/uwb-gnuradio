/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * UhdBurstBackend — IRadioBurstBackend implementation over a real
 * multi_usrp device (Radar Step 11).
 *
 * Same single multi_usrp device, one TX and one RX SC16 streamer
 * (cpu "s16" / otw "sc16"), driven by the EchoTimer scheduling worker:
 *
 *   issue_rx()     rx_streamer->issue_stream_cmd(STREAM_MODE_NUM_SAMPS_AND_DONE,
 *                  stream_now=false, time_spec=rx_ticks)  — armed FIRST
 *   issue_tx()     tx_streamer send() loop over all fragments; the first
 *                  send call carries has_time_spec + start_of_burst; a
 *                  dedicated zero-length send with end_of_burst=true
 *                  terminates the burst after ALL data samples were
 *                  accepted (partial sends re-issued; data calls never
 *                  claim EOB); pending TX async metadata is drained
 *                  (time error → late_command, underflow/seq →
 *                  backend_error)
 *   collect_result() blocking rx_streamer recv() loop until the window is
 *                  complete or the wait budget expires; per-call timeout,
 *                  partial recvs re-issued; rx error codes mapped to per-
 *                  burst statuses (timeout/overflow/late_command/
 *                  broken_chain); first-packet time_spec becomes
 *                  rx_time_ticks; request_stop() wakes the loop and the
 *                  burst reports StopDuringIo
 *   abort_rx()     issue_stream_cmd(STREAM_MODE_STOP_CONTINUOUS)
 *
 * prepare() creates the device, applies clock/time/antenna/frequency/gain
 * configuration, sets the radio rate on both directions and READS IT BACK
 * STRICTLY (737.28 MS/s; any silent UHD coercion is a hard prepare
 * failure).  A missing/unreachable device is reported as
 * "device unavailable: ..." through prepare(error) — never a throw across
 * the IRadioBurstBackend interface and never a crash.
 *
 * Semantics references (local source, recorded in the Step 11 report):
 *   gr-uhd/lib/usrp_sink_impl.cc   tx tags → tx_metadata (SOB/EOB/time),
 *                                  async_event_loop EVENT_CODE_* mapping
 *   gr-uhd/lib/usrp_source_impl.cc stream_cmd NUM_SAMPS_AND_DONE timed
 *                                  burst, recv() ERROR_CODE_* mapping
 *   gr-radar/lib/usrp_echotimer_cc_impl.cc timed-burst geometry (TX/RX on
 *                                  one time base; RX command before TX)
 *
 * This header requires the UHD headers and is only compiled when the
 * build has UHD enabled (UWB_HAVE_UHD defined by CMake).
 */

#ifndef INCLUDED_GNURADIO_UWB_UWB_UHD_BURST_BACKEND_H
#define INCLUDED_GNURADIO_UWB_UWB_UHD_BURST_BACKEND_H

#ifndef UWB_HAVE_UHD
#error \
    "uwb_uhd_burst_backend.h requires UHD support (build with UHD found; "\
"ENABLE_UHD_BACKEND). The rest of gr-uwb builds and tests without UHD."
#endif

#include <gnuradio/uwb/api.h>
#include <gnuradio/uwb/uwb_echo_burst_backend.h>
#include <gnuradio/uwb/uwb_uhd_backend_config.h>

#include <atomic>
#include <memory>
#include <string>

namespace gr {
namespace uwb {
namespace uhd {

class UWB_API UhdBurstBackend : public echo::IRadioBurstBackend
{
public:
    // Config is validated once here (invalid_argument on violation) and
    // frozen; the device is NOT touched by the constructor — prepare()
    // opens it, so a factory-built backend is always safe to construct
    // without hardware.
    explicit UhdBurstBackend(const UhdBurstBackendConfig& cfg);

    ~UhdBurstBackend() override;

    // --- IRadioBurstBackend (see uwb_echo_burst_backend.h) ------------------

    bool prepare(std::string& error) override;
    echo::BurstStatus issue_rx(const echo::RxCommand& cmd,
                               std::string& error) override;
    echo::BurstStatus issue_tx(const echo::TxCommand& cmd,
                               std::string& error) override;
    bool collect_result(uint64_t schedule_index,
                        uint64_t wait_ms,
                        echo::BurstResult& out) override;
    void abort_rx() override;
    void request_stop() override;
    bool stop_requested() const override;
    int64_t device_time_ticks() const override;

    const UhdBurstBackendConfig& config() const { return d_cfg_; }

private:
    // PIMPL: all UHD types stay out of this header (and out of every
    // non-UHD TU).
    struct Impl;
    std::unique_ptr<Impl> d_impl_;

    UhdBurstBackendConfig d_cfg_;
    // Sticky stop flag (mirrors FakeBurstBackend semantics: set by
    // request_stop() at any lifecycle point — even before prepare() —
    // and cleared by a fresh prepare()).
    std::atomic<bool> d_stop_requested_{ false };
};

} // namespace uhd
} // namespace uwb
} // namespace gr

#endif /* INCLUDED_GNURADIO_UWB_UWB_UHD_BURST_BACKEND_H */
