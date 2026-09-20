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
#include <cstddef>
#include <memory>
#include <string>
#include <type_traits>
#include <vector>

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
    echo::BurstStatus tune(double freq_hz, std::string& error) override;

    // --- Multi-TX (X410 dual-TX M2; planning §5.1/§5.2/§5.5/§5.6) -----------
    //
    // Number of configured TX channels (1 = legacy single-TX).  Resolved
    // from the frozen config — no device needed, safe before prepare().
    size_t tx_channel_count() const;

    // Jammer-only retune (§5.5): tune logical TX channel [0, count),
    // mapped onto the configured physical channel; the sense TX and RX
    // are left untouched.  The caller (EchoTimer) serializes this at a
    // burst boundary — never concurrent with issue_*/collect_result, and
    // the radio worker keeps the single-threaded send contract.
    echo::BurstStatus tune_tx_channel(size_t logical_channel,
                                      double hz,
                                      double& actual_hz,
                                      std::string& error) override;

    // Cumulative TX async counters (§5.6 snapshot; thread-safe).
    echo::TxAsyncCounts tx_async_counts() const override;

    // Last frequency read back from the device by tune()/prepare().
    double center_freq_hz() const;

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

// --- M2 transitional multi-TX config helpers (UHD-free) ----------------------
//
// Planning §5.1 extends UhdBurstBackendConfig with
//   std::vector<UhdTxChannelConfig> tx_channels;   // {channel, antenna,
//                                                  //  gain_db, center_freq_hz}
// (empty = legacy single-TX truth from tx_channel/tx_antenna/tx_gain_db/
// center_freq_hz).  That extension lands with M1 (Fake/config/QA owner);
// until then these helpers resolve the legacy single channel, so every TU
// using them compiles and behaves exactly like the single-TX baseline on
// both sides of the M1 landing.  The member/field names below follow §5.1
// verbatim; a renamed M1 landing fails LOUDLY here at compile time (never
// a silent single-channel fallback).

namespace multitx_detail {

template <typename C, typename = void>
struct has_tx_channels : std::false_type {};
template <typename C>
struct has_tx_channels<
    C,
    std::void_t<decltype(std::declval<const C&>().tx_channels)>>
    : std::true_type {};

} // namespace multitx_detail

// Effective TX channel count: M1's tx_channels size when present and
// non-empty, else the legacy 1.
inline size_t configured_tx_channel_count(const UhdBurstBackendConfig& cfg)
{
    if constexpr (multitx_detail::has_tx_channels<
                      UhdBurstBackendConfig>::value) {
        if (!cfg.tx_channels.empty())
            return cfg.tx_channels.size();
    }
    return 1;
}

// Fill M1's per-channel vector from parallel arrays (all four must share
// one length; the caller validates).  No-op while the member is absent —
// the legacy scalar fields then remain the single source of truth.
inline void assign_tx_channels(UhdBurstBackendConfig& cfg,
                               const std::vector<size_t>& channels,
                               const std::vector<std::string>& antennas,
                               const std::vector<double>& gains_db,
                               const std::vector<double>& freqs_hz)
{
    if constexpr (multitx_detail::has_tx_channels<
                      UhdBurstBackendConfig>::value) {
        cfg.tx_channels.clear();
        for (size_t i = 0; i < channels.size(); ++i) {
            typename std::decay_t<decltype(cfg.tx_channels)>::value_type e;
            e.channel = channels[i];
            e.antenna = antennas[i];
            e.gain_db = gains_db[i];
            e.center_freq_hz = freqs_hz[i];
            cfg.tx_channels.push_back(e);
        }
    }
}

} // namespace uhd
} // namespace uwb
} // namespace gr

#endif /* INCLUDED_GNURADIO_UWB_UWB_UHD_BURST_BACKEND_H */
