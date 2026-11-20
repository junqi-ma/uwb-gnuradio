/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * UhdBurstBackend implementation.  Built ONLY when CMake found UHD
 * (ENABLE_UHD_BACKEND); the rest of gr-uwb (Phase A, EchoTimer, fake
 * backend, all other QA) builds and tests without this TU.
 *
 * Threading: the EchoTimer scheduling worker drives issue_rx → issue_tx →
 * collect_result sequentially (all data I/O on one thread); one dedicated
 * async-event thread (started in prepare(), mirroring gr-uhd
 * usrp_sink_impl::async_event_loop) polls the TX streamer's async
 * metadata queue (recv_async_msg, 100 ms) and records non-ack events.
 * collect_result() attributes TIME_ERROR/underflow/seq events to the
 * in-flight burst by matching the event's device time ticks against the
 * burst TX ticks, so a TX late-command reported around the RX window end
 * is still mapped onto the correct burst.
 *
 * EOB handling: data send() calls never claim end_of_burst (a partial
 * send() must never have declared EOB); after every data sample has been
 * accepted, a dedicated zero-length send() with end_of_burst=true
 * terminates the burst — the same mechanism gr-uhd usrp_sink uses to
 * terminate bursts (stop() sends a 0-length EOB packet).
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gnuradio/uwb/uwb_uhd_burst_backend.h>
#include <gnuradio/uwb/uwb_echo_multitx.h> // echo::kEchoMaxTxChannels (hot-path array bound)

// UHD 4.1 header layout (pre-4.2): combined uhd/stream.hpp holds both
// streamers, uhd/types/metadata.hpp holds rx_metadata_t/async_metadata_t.
// Newer UHD layouts keep these names at the same paths via compat shim
// headers, so this include set stays valid across 4.x.
#include <uhd/exception.hpp>
#include <uhd/stream.hpp>
#include <uhd/types/metadata.hpp>
#include <uhd/types/stream_cmd.hpp>
#include <uhd/usrp/multi_usrp.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <deque>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>

namespace gr {
namespace uwb {
namespace uhd {

// ---------------------------------------------------------------------------
// ABI cross-checks: the UHD-free mappings in uwb_uhd_backend_config.h rely
// on these stable integer values; if a future UHD changes them, this TU
// fails to compile instead of silently mis-mapping.
// ---------------------------------------------------------------------------
static_assert(static_cast<int32_t>(::uhd::rx_metadata_t::ERROR_CODE_NONE) ==
                  kUhdRxErrorNone,
              "UHD rx error ABI changed (none)");
static_assert(
    static_cast<int32_t>(::uhd::rx_metadata_t::ERROR_CODE_TIMEOUT) ==
        kUhdRxErrorTimeout,
    "UHD rx error ABI changed (timeout)");
static_assert(
    static_cast<int32_t>(::uhd::rx_metadata_t::ERROR_CODE_OVERFLOW) ==
        kUhdRxErrorOverflow,
    "UHD rx error ABI changed (overflow)");
static_assert(
    static_cast<int32_t>(::uhd::rx_metadata_t::ERROR_CODE_LATE_COMMAND) ==
        kUhdRxErrorLateCommand,
    "UHD rx error ABI changed (late command)");
static_assert(
    static_cast<int32_t>(::uhd::rx_metadata_t::ERROR_CODE_BROKEN_CHAIN) ==
        kUhdRxErrorBrokenChain,
    "UHD rx error ABI changed (broken chain)");
static_assert(
    static_cast<uint32_t>(::uhd::async_metadata_t::EVENT_CODE_BURST_ACK) ==
        kUhdAsyncBurstAck,
    "UHD tx async ABI changed (burst ack)");
static_assert(
    static_cast<uint32_t>(::uhd::async_metadata_t::EVENT_CODE_UNDERFLOW) ==
        kUhdAsyncUnderflow,
    "UHD tx async ABI changed (underflow)");
static_assert(
    static_cast<uint32_t>(::uhd::async_metadata_t::EVENT_CODE_SEQ_ERROR) ==
        kUhdAsyncSeqError,
    "UHD tx async ABI changed (seq error)");
static_assert(
    static_cast<uint32_t>(::uhd::async_metadata_t::EVENT_CODE_TIME_ERROR) ==
        kUhdAsyncTimeError,
    "UHD tx async ABI changed (time error)");
static_assert(
    static_cast<uint32_t>(
        ::uhd::async_metadata_t::EVENT_CODE_UNDERFLOW_IN_PACKET) ==
        kUhdAsyncUnderflowInPacket,
    "UHD tx async ABI changed (underflow in packet)");
static_assert(
    static_cast<uint32_t>(
        ::uhd::async_metadata_t::EVENT_CODE_SEQ_ERROR_IN_BURST) ==
        kUhdAsyncSeqErrorInBurst,
    "UHD tx async ABI changed (seq error in burst)");

namespace {

std::string uhd_error_string(const std::string& context,
                             const std::exception& e)
{
    std::ostringstream os;
    os << context << ": " << e.what();
    return os.str();
}

int64_t async_event_ticks(const ::uhd::async_metadata_t& md, double rate_hz)
{
    if (!md.has_time_spec)
        return -1;
    int64_t ticks = -1;
    if (!ticks_from_time_parts(md.time_spec.get_full_secs(),
                               md.time_spec.get_frac_secs(), rate_hz,
                               ticks))
        return -1;
    return ticks;
}

// --- M2 multi-TX channel resolution ---------------------------------------
// Effective per-channel prepare truth (§5.1): M1's tx_channels vector when
// present and non-empty, else the legacy single-TX scalars (exactly today's
// prepare() semantics).  Only prepare() may allocate (std::vector); the
// hot path uses the frozen Impl map below.
struct ResolvedTxChannel {
    size_t phys_channel = 0;
    std::string antenna;
    double gain_db = -1.0;
    double freq_hz = 0.0;
};

bool resolve_tx_channels(const UhdBurstBackendConfig& cfg,
                         std::vector<ResolvedTxChannel>& out,
                         std::string& error)
{
    out.clear();
    if constexpr (multitx_detail::has_tx_channels<
                      UhdBurstBackendConfig>::value) {
        const auto& v = cfg.tx_channels;
        if (!v.empty()) {
            if (v.size() > echo::kEchoMaxTxChannels) {
                error = "too many TX channels";
                return false;
            }
            for (const auto& e : v) {
                ResolvedTxChannel r;
                r.phys_channel = e.channel;
                r.antenna = e.antenna;
                r.gain_db = e.gain_db;
                r.freq_hz = e.center_freq_hz;
                out.push_back(r);
            }
        }
    }
    if (out.empty()) {
        ResolvedTxChannel r;
        r.phys_channel = cfg.tx_channel;
        r.antenna = cfg.tx_antenna;
        r.gain_db = cfg.tx_gain_db;
        r.freq_hz = cfg.center_freq_hz;
        out.push_back(r);
    }
    // Defensive re-check (M1's validate_* owns the full contract; prepare
    // never trusts it blindly): physical channels unique, freq/gain sane.
    for (size_t i = 0; i < out.size(); ++i) {
        for (size_t j = i + 1; j < out.size(); ++j) {
            if (out[i].phys_channel == out[j].phys_channel) {
                error = "duplicate TX physical channel";
                return false;
            }
        }
        if (!std::isfinite(out[i].freq_hz) || out[i].freq_hz < 0.0) {
            error = "TX channel frequency must be >= 0 and finite";
            return false;
        }
        if (!std::isfinite(out[i].gain_db)) {
            error = "TX channel gain must be finite (negative = not set)";
            return false;
        }
        if (out[i].gain_db >= 0.0 && out[i].gain_db > 120.0) {
            error = "TX channel gain must be <= 120 when set";
            return false;
        }
    }
    return true;
}

// TxBurstFragment flag contract (§5.2): the same SOB/time-spec/EOB
// placement rules as validate_fragment_flags, plus one rule the single
// path cannot express — every configured channel must carry a non-null
// slice for every fragment (equal-length buffers per send).
bool validate_tx_multi_flags(const echo::TxBurstFragment* frags,
                             size_t count,
                             size_t expect_channels,
                             std::string& error)
{
    auto fail = [&](const char* what) {
        error = what;
        return false;
    };
    if (frags == nullptr || count == 0 ||
        count > echo::kEchoMaxFragmentsPerBurst)
        return fail("empty or oversized TX fragment list");
    if (expect_channels == 0 ||
        expect_channels > echo::kEchoMaxTxChannels)
        return fail("TX channel count out of range");
    for (size_t i = 0; i < count; ++i) {
        const unsigned f = frags[i].flags;
        if (frags[i].count == 0)
            return fail("TX fragment with zero samples");
        for (size_t c = 0; c < expect_channels; ++c) {
            if (frags[i].tx_data[c] == nullptr)
                return fail("TX fragment has null data on a channel");
        }
        if (i == 0) {
            if (!(f & echo::kFlagTimeSpec) || !(f & echo::kFlagStartOfBurst))
                return fail("first TX fragment lacks time spec + SOB");
        } else if (f & (echo::kFlagTimeSpec | echo::kFlagStartOfBurst)) {
            return fail("SOB / time spec outside the first TX fragment");
        }
        if (i + 1 == count) {
            if (!(f & echo::kFlagEndOfBurst))
                return fail("last TX fragment lacks EOB");
        } else if (f & echo::kFlagEndOfBurst) {
            return fail("EOB outside the last TX fragment");
        }
    }
    return true;
}

} // namespace

struct UhdBurstBackend::Impl
{
    ::uhd::usrp::multi_usrp::sptr dev;
    ::uhd::tx_streamer::sptr tx_stream;
    ::uhd::rx_streamer::sptr rx_stream;

    // Frozen TX channel map (prepare-time; worker read-only afterwards):
    // logical index [0, tx_channel_count) → physical device channel.
    std::array<size_t, echo::kEchoMaxTxChannels> tx_phys_channels = {};
    size_t tx_channel_count = 0;
    // Per-logical-channel TX frequency actually read back from the device
    // by prepare()/tune_tx_channel() (0 = untouched).
    std::array<double, echo::kEchoMaxTxChannels> tx_actual_freq_hz = {};

    // Armed RX state (issue_rx → collect_result, worker thread only).
    bool rx_armed = false;
    uint64_t armed_index = 0;
    int64_t armed_rx_ticks = 0; // commanded RX window start (ticks)
    uint64_t armed_total_samples = 0;
    int16_t* rx_base = nullptr; // burst-linear write base (fixed scratch)

    // In-flight TX state for event attribution and result reporting
    // (worker thread only).  Requested/sent stay PER-CHANNEL (== L); the
    // wire (transport) totals are channels × per-channel, folded in
    // collect_result().
    uint64_t pending_tx_ticks = 0;
    uint64_t pending_tx_sent = 0;
    uint64_t pending_tx_requested = 0;
    size_t pending_tx_channels = 0;

    // Last frequency applied through tune()/prepare() (read back from the
    // device); atomic because the setter runs on the caller thread while
    // the scheduler worker runs the bursts.  Jammer-only tune_tx_channel()
    // deliberately leaves this co-tuned sense/RX reference untouched.
    std::atomic<double> center_freq_hz{ 0.0 };

    // Async events (§5.6; async-thread producer, worker consumer).
    struct AsyncEvent {
        uint32_t event_code = 0;
        size_t channel = 0;    // UHD async channel (MIMO config index)
        int64_t tx_ticks = -1; // device ticks from time_spec; -1 = none
    };
    static constexpr size_t kMaxAsyncEvents = 128;
    // Lifecycle flag doubling as "stop requested" (async thread runs
    // while true; request_stop clears it, which stop_requested reports).
    std::atomic<bool> async_run{ false };
    std::thread async_thread;
    // NOTE: the collect drain uses a worker-owned std::array (never
    // std::vector); this ring only needs to be bounded, not array-shaped.
    std::mutex async_mutex;
    std::deque<AsyncEvent> async_events; // bounded, oldest dropped
    // Cumulative §5.6 counters.  The async thread bumps dropped_; the
    // worker folds ack/underflow/seq/time/unmatched at collect_result();
    // tx_async_counts() snapshots all six (atomics: lock-free read).
    std::atomic<uint64_t> async_ack_{ 0 };
    std::atomic<uint64_t> async_underflow_{ 0 };
    std::atomic<uint64_t> async_seq_error_{ 0 };
    std::atomic<uint64_t> async_time_error_{ 0 };
    std::atomic<uint64_t> async_unmatched_{ 0 };
    std::atomic<uint64_t> async_dropped_{ 0 };

    void push_event(const AsyncEvent& ev)
    {
        std::lock_guard<std::mutex> lock(async_mutex);
        if (async_events.size() >= kMaxAsyncEvents) {
            async_events.pop_front();
            async_dropped_.fetch_add(1, std::memory_order_relaxed);
        }
        async_events.push_back(ev);
    }

    // Drain for one collect_result(): EVERY drained event is classified by
    // its event code FIRST (§5.6: a no-time-spec / foreign underflow must
    // still be counted as an underflow — a full-length send() is never a
    // substitute for "no U"), and events whose device ticks match this
    // burst's TX ticks are additionally copied into the worker-owned
    // fixed-capacity array for per-burst status mapping.  Events that could
    // not be attributed to this burst (no time spec, or foreign ticks from
    // startup/stale bursts) are counted as unmatched so the acceptance
    // report can separate startup/unmatched from steady-state.  All drained
    // events are erased: the cumulative counters ARE the record (§5.6).
    size_t take_events_for(int64_t tx_ticks,
                           AsyncEvent* out,
                           size_t cap,
                           uint64_t& ack,
                           uint64_t& underflow,
                           uint64_t& seq_error,
                           uint64_t& time_error,
                           uint64_t& unmatched)
    {
        std::lock_guard<std::mutex> lock(async_mutex);
        size_t n = 0;
        ack = underflow = seq_error = time_error = unmatched = 0;
        for (auto it = async_events.begin(); it != async_events.end();) {
            const uint32_t code = it->event_code;
            if (code == 0 || code == kUhdAsyncBurstAck)
                ++ack;
            if (code & kUhdAsyncTimeError)
                ++time_error;
            if (code & (kUhdAsyncUnderflow | kUhdAsyncUnderflowInPacket))
                ++underflow;
            if (code & (kUhdAsyncSeqError | kUhdAsyncSeqErrorInBurst))
                ++seq_error;
            if (it->tx_ticks >= 0 && it->tx_ticks == tx_ticks) {
                if (n < cap) {
                    out[n++] = *it;
                } else {
                    async_dropped_.fetch_add(1, std::memory_order_relaxed);
                }
            } else {
                ++unmatched;
            }
            it = async_events.erase(it);
        }
        async_ack_.fetch_add(ack, std::memory_order_relaxed);
        async_underflow_.fetch_add(underflow, std::memory_order_relaxed);
        async_seq_error_.fetch_add(seq_error, std::memory_order_relaxed);
        async_time_error_.fetch_add(time_error, std::memory_order_relaxed);
        async_unmatched_.fetch_add(unmatched, std::memory_order_relaxed);
        return n;
    }

    void reset_async_counts()
    {
        async_ack_.store(0, std::memory_order_relaxed);
        async_underflow_.store(0, std::memory_order_relaxed);
        async_seq_error_.store(0, std::memory_order_relaxed);
        async_time_error_.store(0, std::memory_order_relaxed);
        async_unmatched_.store(0, std::memory_order_relaxed);
        async_dropped_.store(0, std::memory_order_relaxed);
    }

    void start_async_thread(UhdBurstBackend* self)
    {
        async_run.store(true, std::memory_order_release);
        async_thread = std::thread([this, self]() {
            ::uhd::async_metadata_t md;
            const double rate = self->d_cfg_.sample_rate_hz;
            while (async_run.load(std::memory_order_acquire)) {
                // 100 ms poll, exactly like gr-uhd's async_event_loop.
                if (!tx_stream || !tx_stream->recv_async_msg(md, 0.1))
                    continue;
                push_event(AsyncEvent{
                    static_cast<uint32_t>(md.event_code), md.channel,
                    async_event_ticks(md, rate) });
            }
        });
    }

    void stop_async_thread()
    {
        // Single-caller contract (the EchoTimer scheduler thread): after
        // a successful join, joinable() is false and this is a no-op.
        async_run.store(false, std::memory_order_release);
        if (async_thread.joinable())
            async_thread.join();
    }
};

// ---------------------------------------------------------------------------
// Construction / destruction
// ---------------------------------------------------------------------------

UhdBurstBackend::UhdBurstBackend(const UhdBurstBackendConfig& cfg)
    : d_cfg_(cfg)
{
    std::string err;
    if (!validate_uhd_burst_backend_config(cfg, &err))
        throw std::invalid_argument("UhdBurstBackend: " + err);
}

UhdBurstBackend::~UhdBurstBackend()
{
    if (d_impl_)
        d_impl_->stop_async_thread();
}

// ---------------------------------------------------------------------------
// prepare()
// ---------------------------------------------------------------------------

bool UhdBurstBackend::prepare(std::string& error)
{
    std::string err;
    if (!validate_uhd_burst_backend_config(d_cfg_, &err)) {
        error = "invalid backend config: " + err;
        return false;
    }

    // Restart semantics: a fresh prepare() clears a previous request_stop
    // (the block calls request_stop() before every start()).
    d_stop_requested_.store(false, std::memory_order_release);

    auto impl = std::make_unique<Impl>();

    try {
        // Device creation: a missing/unreachable USRP is the expected,
        // clearly-reported condition on machines with UHD but no hardware.
        ::uhd::device_addr_t args(d_cfg_.device_args);
        impl->dev = ::uhd::usrp::multi_usrp::make(args);
    } catch (const std::exception& e) {
        error = make_device_unavailable_error(uhd_error_string(
            "multi_usrp::make(" + d_cfg_.device_args + ")", e));
        return false;
    }

    try {
        const size_t mboard = 0;
        if (!d_cfg_.time_source.empty())
            impl->dev->set_time_source(d_cfg_.time_source, mboard);
        if (!d_cfg_.clock_source.empty())
            impl->dev->set_clock_source(d_cfg_.clock_source, mboard);

        // Resolve the effective TX channel list (§5.1; legacy single-TX
        // scalars when M1's tx_channels is absent/empty).
        std::vector<ResolvedTxChannel> txs;
        {
            std::string cfg_err;
            if (!resolve_tx_channels(d_cfg_, txs, cfg_err)) {
                error = std::string("invalid TX channel config: ") + cfg_err;
                return false;
            }
        }
        const size_t ntx = txs.size();

        // Strict sample-rate contract on EVERY TX channel plus RX: set,
        // then read back, then reject any silent coercion.
        for (const auto& t : txs) {
            impl->dev->set_tx_rate(d_cfg_.sample_rate_hz, t.phys_channel);
            const double tx_rate =
                impl->dev->get_tx_rate(t.phys_channel);
            if (!rate_matches_strict(d_cfg_.sample_rate_hz, tx_rate,
                                     d_cfg_.rate_tolerance_rel)) {
                std::ostringstream os;
                os << "strict rate readback failed on TX channel "
                   << t.phys_channel << ": requested="
                   << d_cfg_.sample_rate_hz << " readback=" << tx_rate
                   << " (silent coercion rejected; rel_tol="
                   << d_cfg_.rate_tolerance_rel << ")";
                error = os.str();
                return false;
            }
        }
        impl->dev->set_rx_rate(d_cfg_.sample_rate_hz, d_cfg_.rx_channel);
        const double rx_rate = impl->dev->get_rx_rate(d_cfg_.rx_channel);
        if (!rate_matches_strict(d_cfg_.sample_rate_hz, rx_rate,
                                 d_cfg_.rate_tolerance_rel)) {
            std::ostringstream os;
            os << "strict rate readback failed on RX channel "
               << d_cfg_.rx_channel << ": requested=" << d_cfg_.sample_rate_hz
               << " readback=" << rx_rate
               << " (silent coercion rejected; rel_tol="
               << d_cfg_.rate_tolerance_rel << ")";
            error = os.str();
            return false;
        }

        // Per-channel TX frequencies (0 = leave untouched, mirroring the
        // legacy center_freq_hz == 0 sentinel).  The RX keeps the
        // deliberate rx_freq_offset_hz off the SENSE (logical-0) reference
        // (see UhdBurstBackendConfig); legacy single-TX resolves to exactly
        // today's co-tuned behavior.
        for (const auto& t : txs) {
            if (t.freq_hz > 0.0) {
                impl->dev->set_tx_freq(::uhd::tune_request_t(t.freq_hz),
                                       t.phys_channel);
            }
        }
        if (txs[0].freq_hz > 0.0) {
            impl->dev->set_rx_freq(
                ::uhd::tune_request_t(txs[0].freq_hz +
                                      d_cfg_.rx_freq_offset_hz),
                d_cfg_.rx_channel);
            // "Center" is the co-tuned sense reference: strip the
            // deliberate RX offset back out of the readback average.
            impl->center_freq_hz.store(
                0.5 * (impl->dev->get_tx_freq(txs[0].phys_channel) +
                       impl->dev->get_rx_freq(d_cfg_.rx_channel) -
                       d_cfg_.rx_freq_offset_hz));
        }
        for (const auto& t : txs) {
            if (t.gain_db >= 0.0)
                impl->dev->set_tx_gain(t.gain_db, t.phys_channel);
            if (!t.antenna.empty())
                impl->dev->set_tx_antenna(t.antenna, t.phys_channel);
        }
        if (d_cfg_.rx_gain_db >= 0.0)
            impl->dev->set_rx_gain(d_cfg_.rx_gain_db, d_cfg_.rx_channel);
        if (!d_cfg_.rx_antenna.empty())
            impl->dev->set_rx_antenna(d_cfg_.rx_antenna, d_cfg_.rx_channel);

        // Same device, one multi-channel TX SC16 streamer plus one RX SC16
        // streamer (cpu s16 / otw sc16 — the production native wire
        // format).  channels=[sense, jam, ...] in logical order.
        ::uhd::stream_args_t tx_args("sc16", "sc16");
        tx_args.channels.clear();
        for (const auto& t : txs)
            tx_args.channels.push_back(t.phys_channel);
        impl->tx_stream = impl->dev->get_tx_stream(tx_args);
        ::uhd::stream_args_t rx_args("sc16", "sc16");
        rx_args.channels = std::vector<size_t>{ d_cfg_.rx_channel };
        impl->rx_stream = impl->dev->get_rx_stream(rx_args);
        if (!impl->tx_stream || !impl->rx_stream) {
            error = "failed to create SC16 TX/RX streamers";
            return false;
        }
        // The streamer must expose exactly the requested channel count —
        // a coerced single-channel streamer under a dual-TX schedule
        // would silently misdeliver the jammer row.
        const size_t streamer_nch = impl->tx_stream->get_num_channels();
        if (streamer_nch != ntx) {
            std::ostringstream os;
            os << "TX streamer channel count mismatch: requested=" << ntx
               << " streamer=" << streamer_nch;
            error = os.str();
            return false;
        }
        for (size_t i = 0; i < ntx; ++i)
            impl->tx_phys_channels[i] = txs[i].phys_channel;
        impl->tx_channel_count = ntx;

        // Re-assert every configured TX frequency AFTER the multi-channel
        // streamer exists and verify each readback strictly (§5.1 per-TX
        // frequency contract).  On the X410 the NCO/LO solution for a
        // second TX channel is not fully committed by the prepare-time
        // set_tx_freq alone: measured 2026-09-19, a fixed jammer CFO
        // programmed only here misses the narrow repetition-average null
        // (floor +7.9 dB) while the SAME frequency reached through a
        // runtime tune_tx_channel reaches the full null (+0.1 dB), with
        // identical get_tx_freq readbacks.  Re-applying after streamer
        // creation makes the arm-time state match the retune state.
        for (const auto& t : txs) {
            if (t.freq_hz > 0.0) {
                impl->dev->set_tx_freq(::uhd::tune_request_t(t.freq_hz),
                                       t.phys_channel);
            }
        }
        for (size_t i = 0; i < ntx; ++i) {
            if (!(txs[i].freq_hz > 0.0))
                continue; // 0 = leave untouched (legacy sentinel)
            const double got = impl->dev->get_tx_freq(txs[i].phys_channel);
            if (std::fabs(got - txs[i].freq_hz) > kUhdTxFreqReadbackTolHz) {
                std::ostringstream os;
                os << "strict TX frequency readback failed on logical "
                      "channel " << i << " (phys " << txs[i].phys_channel
                   << "): requested=" << txs[i].freq_hz
                   << " readback=" << got << " (tol="
                   << kUhdTxFreqReadbackTolHz << " Hz)";
                error = os.str();
                return false;
            }
            impl->tx_actual_freq_hz[i] = got;
        }
    } catch (const std::exception& e) {
        error = uhd_error_string("device configuration failed", e);
        return false;
    }

    // Reset all per-burst state (restart-safe; the block calls prepare()
    // on every start()).
    impl->rx_armed = false;
    impl->armed_index = 0;
    impl->armed_total_samples = 0;
    impl->rx_base = nullptr;
    impl->pending_tx_ticks = 0;
    impl->pending_tx_sent = 0;
    impl->pending_tx_requested = 0;
    impl->pending_tx_channels = 0;
    {
        std::lock_guard<std::mutex> lock(impl->async_mutex);
        impl->async_events.clear();
    }
    impl->reset_async_counts();

    d_impl_ = std::move(impl);
    d_impl_->start_async_thread(this);
    return true;
}

// ---------------------------------------------------------------------------
// issue_rx(): arm the timed RX stream command FIRST
// ---------------------------------------------------------------------------

echo::BurstStatus UhdBurstBackend::issue_rx(const echo::RxCommand& cmd,
                                            std::string& error)
{
    if (!d_impl_ || !d_impl_->rx_stream) {
        error = "backend not prepared";
        return echo::BurstStatus::BackendError;
    }
    if (stop_requested()) {
        error = "backend stopped";
        return echo::BurstStatus::StopDuringIo;
    }
    Impl& impl = *d_impl_;

    std::string flag_err;
    if (!validate_fragment_flags(cmd.fragments, cmd.fragment_count,
                                 &flag_err)) {
        error = std::string("broken chain: ") + flag_err;
        return echo::BurstStatus::BrokenChain;
    }

    // Timed NUM_SAMPS_AND_DONE command (gr-uhd usrp_source_impl
    // issue_stream_cmd semantics).  The RX command is armed BEFORE the TX
    // burst so the device has the RX schedule committed when the TX
    // window fires.
    try {
        int64_t full = 0;
        double frac = 0.0;
        if (!time_parts_from_ticks(cmd.rx_ticks, d_cfg_.sample_rate_hz,
                                   full, frac)) {
            error = "rx_ticks out of representable device time range";
            return echo::BurstStatus::BackendError;
        }
        ::uhd::stream_cmd_t stream_cmd(
            ::uhd::stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_DONE);
        stream_cmd.num_samps = cmd.total_samples;
        stream_cmd.stream_now = false;
        stream_cmd.time_spec = ::uhd::time_spec_t(full, frac);
        impl.rx_stream->issue_stream_cmd(stream_cmd);
    } catch (const std::exception& e) {
        error = uhd_error_string("issue_stream_cmd(NUM_SAMPS_AND_DONE)", e);
        return echo::BurstStatus::BackendError;
    }

    // Arm state for collect_result (burst-linear write base; the planned
    // fragments are contiguous spans of the fixed RX scratch).
    if (cmd.fragment_count == 0 || cmd.fragments == nullptr ||
        cmd.fragments[0].rx_data == nullptr) {
        error = "rx fragment list has no data buffer";
        return echo::BurstStatus::BackendError;
    }
    impl.rx_armed = true;
    impl.armed_index = cmd.schedule_index;
    impl.armed_rx_ticks = cmd.rx_ticks;
    impl.armed_total_samples = cmd.total_samples;
    impl.rx_base = cmd.fragments[0].rx_data -
                   cmd.fragments[0].offset * 2; // burst-linear base
    return echo::BurstStatus::Ok;
}

// ---------------------------------------------------------------------------
// issue_tx(): send loop over all planned fragments
// ---------------------------------------------------------------------------

echo::BurstStatus UhdBurstBackend::issue_tx(const echo::TxCommand& cmd,
                                            std::string& error)
{
    if (!d_impl_ || !d_impl_->tx_stream) {
        error = "backend not prepared";
        return echo::BurstStatus::BackendError;
    }
    if (stop_requested()) {
        error = "backend stopped";
        return echo::BurstStatus::StopDuringIo;
    }
    Impl& impl = *d_impl_;
    const size_t nch = impl.tx_channel_count;
    if (nch == 0 || nch > echo::kEchoMaxTxChannels) {
        error = "backend not prepared";
        return echo::BurstStatus::BackendError;
    }
    if (cmd.tx_channel_count == 0) {
        error = "broken chain: TX command has zero channels";
        return echo::BurstStatus::BackendError;
    }
    if (cmd.tx_channel_count != nch) {
        std::ostringstream os;
        os << "TX channel count mismatch: command=" << cmd.tx_channel_count
           << " backend=" << nch;
        error = os.str();
        return echo::BurstStatus::BackendError;
    }

    if (!impl.rx_armed || impl.armed_index != cmd.schedule_index) {
        error = "broken chain: TX issued without armed RX for the same "
                "schedule index";
        return echo::BurstStatus::BrokenChain;
    }

    // Single-TX legacy path: validate the classic fragment list now so
    // the multi-TX branch below owns the TxBurstFragment contract only.
    if (nch == 1) {
        if (cmd.fragment_count == 0 || cmd.fragments == nullptr) {
            error = "broken chain: empty TX fragment list";
            return echo::BurstStatus::BrokenChain;
        }
        std::string flag_err;
        if (!validate_fragment_flags(cmd.fragments, cmd.fragment_count,
                                     &flag_err)) {
            error = std::string("broken chain: ") + flag_err;
            return echo::BurstStatus::BrokenChain;
        }
    }

    uint64_t sent_total = 0; // per-channel accepted samples (== L on Ok)
    bool first_send_call = true;
    bool saw_eob_flag = false;
    try {
        if (nch == 1) {
            for (size_t i = 0; i < cmd.fragment_count; ++i) {
                const echo::BurstFragment& f = cmd.fragments[i];
                if (f.tx_data == nullptr || f.count == 0) {
                    error = "broken chain: TX fragment has no data";
                    return echo::BurstStatus::BrokenChain;
                }
                if (f.flags & echo::kFlagEndOfBurst)
                    saw_eob_flag = true;
                uint64_t done = 0;
                while (done < f.count) {
                    const uint64_t remaining = f.count - done;
                    ::uhd::tx_metadata_t md;
                    // gr-uhd usrp_sink mapping: time spec + SOB travel on
                    // the FIRST send call of the burst only; data calls
                    // NEVER carry EOB (a partial send must not have
                    // declared one).
                    if (first_send_call && (f.flags & echo::kFlagTimeSpec)) {
                        int64_t full = 0;
                        double frac = 0.0;
                        if (!time_parts_from_ticks(cmd.tx_ticks,
                                                   d_cfg_.sample_rate_hz,
                                                   full, frac)) {
                            error = "tx_ticks out of representable device "
                                    "time range";
                            return echo::BurstStatus::BackendError;
                        }
                        md.has_time_spec = true;
                        md.time_spec = ::uhd::time_spec_t(full, frac);
                    }
                    if (first_send_call &&
                        (f.flags & echo::kFlagStartOfBurst))
                        md.start_of_burst = true;

                    // No allocation: one stack pointer, viewed through
                    // ref_vector (buffers.size() == 1).
                    std::array<const void*, 1> buffs{
                        static_cast<const void*>(f.tx_data + done * 2)
                    };
                    const size_t ntransfered = impl.tx_stream->send(
                        ::uhd::ref_vector<const void*>(buffs.data(), 1),
                        remaining, md, d_cfg_.send_timeout_s);
                    first_send_call = false;
                    done += ntransfered;
                    sent_total += ntransfered;
                    // ntransfered < remaining → partial send; the loop
                    // re-issues the remainder without SOB/time spec.
                }
            }
        } else {
            // Multi-TX path (§5.2): every send carries buffers.size() ==
            // tx_channel_count with IDENTICAL nsamps_per_buff; time_spec +
            // SOB only on the burst's first send; NO data send carries
            // EOB; a partial send re-issues ALL channels from the same
            // sample offset (one send() transfers the same count on every
            // channel, so a single `done` cursor stays exact).
            std::string flag_err;
            if (!validate_tx_multi_flags(cmd.tx_multi_fragments,
                                         cmd.fragment_count, nch,
                                         flag_err)) {
                error = std::string("broken chain: ") + flag_err;
                return echo::BurstStatus::BrokenChain;
            }
            for (size_t i = 0; i < cmd.fragment_count; ++i) {
                const echo::TxBurstFragment& f = cmd.tx_multi_fragments[i];
                if (f.flags & echo::kFlagEndOfBurst)
                    saw_eob_flag = true;
                uint64_t done = 0;
                while (done < f.count) {
                    const uint64_t remaining = f.count - done;
                    ::uhd::tx_metadata_t md;
                    if (first_send_call && (f.flags & echo::kFlagTimeSpec)) {
                        int64_t full = 0;
                        double frac = 0.0;
                        if (!time_parts_from_ticks(cmd.tx_ticks,
                                                   d_cfg_.sample_rate_hz,
                                                   full, frac)) {
                            error = "tx_ticks out of representable device "
                                    "time range";
                            return echo::BurstStatus::BackendError;
                        }
                        md.has_time_spec = true;
                        md.time_spec = ::uhd::time_spec_t(full, frac);
                    }
                    if (first_send_call &&
                        (f.flags & echo::kFlagStartOfBurst))
                        md.start_of_burst = true;

                    // No allocation: fixed stack array, viewed through
                    // ref_vector(ptr, count) — buffers.size() == nch.
                    std::array<const void*, echo::kEchoMaxTxChannels> buffs{};
                    for (size_t c = 0; c < nch; ++c)
                        buffs[c] = static_cast<const void*>(
                            f.tx_data[c] + done * 2);
                    const size_t ntransfered = impl.tx_stream->send(
                        ::uhd::ref_vector<const void*>(buffs.data(), nch),
                        remaining, md, d_cfg_.send_timeout_s);
                    first_send_call = false;
                    done += ntransfered;
                    sent_total += ntransfered;
                }
            }
        }

        // Burst termination: a dedicated zero-length send with EOB (the
        // gr-uhd usrp_sink stop() mechanism).  Exactly one EOB, after ALL
        // channels' data samples were accepted — immune to partial sends.
        if (saw_eob_flag) {
            ::uhd::tx_metadata_t md;
            md.end_of_burst = true;
            std::array<const void*, echo::kEchoMaxTxChannels> buffs{};
            impl.tx_stream->send(
                ::uhd::ref_vector<const void*>(buffs.data(), nch), 0, md,
                d_cfg_.send_timeout_s);
        }
    } catch (const std::exception& e) {
        error = uhd_error_string("tx_streamer::send", e);
        return echo::BurstStatus::BackendError;
    }

    impl.pending_tx_ticks = static_cast<uint64_t>(cmd.tx_ticks);
    impl.pending_tx_sent = sent_total;
    impl.pending_tx_requested = cmd.total_samples;
    impl.pending_tx_channels = nch;

    // Non-blocking async drain: pick up an event already queued (e.g. an
    // immediately-detected late command).  The async thread keeps
    // draining; attribution happens in collect_result.
    ::uhd::async_metadata_t amd;
    if (impl.tx_stream->recv_async_msg(amd, 0.0)) {
        impl.push_event(Impl::AsyncEvent{
            static_cast<uint32_t>(amd.event_code), amd.channel,
            async_event_ticks(amd, d_cfg_.sample_rate_hz) });
    }

    return echo::BurstStatus::Ok;
}

// ---------------------------------------------------------------------------
// collect_result(): blocking RX completion wait
// ---------------------------------------------------------------------------

bool UhdBurstBackend::collect_result(uint64_t schedule_index,
                                     uint64_t wait_ms,
                                     echo::BurstResult& out)
{
    out = echo::BurstResult{};
    out.schedule_index = schedule_index;
    if (!d_impl_ || !d_impl_->rx_stream) {
        out.status = echo::BurstStatus::BackendError;
        out.error = "backend not prepared";
        return true;
    }
    Impl& impl = *d_impl_;

    if (!impl.rx_armed || impl.armed_index != schedule_index) {
        out.status = echo::BurstStatus::BackendError;
        out.error = "collect for a burst whose RX command is not armed";
        return true;
    }

    const int64_t pending_tx_ticks =
        static_cast<int64_t>(impl.pending_tx_ticks);
    const uint64_t tx_sent = impl.pending_tx_sent;
    const uint64_t tx_requested = impl.pending_tx_requested;
    const size_t tx_channels = impl.pending_tx_channels;

    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(wait_ms);
    int16_t* cursor = impl.rx_base;
    uint64_t received = 0;
    bool have_first_time = false;
    int64_t rx_time_ticks = -1;
    uint64_t rx_reissues = 0;
    echo::BurstStatus status = echo::BurstStatus::Ok;
    std::string error_note;

    auto finish = [&](echo::BurstStatus st) -> bool {
        // Faulted completions leave the RX stream in a clean state for
        // the next burst (a successful NUM_SAMPS_AND_DONE stream ends by
        // itself; the STOP command is then a harmless no-op).
        if (st != echo::BurstStatus::Ok &&
            st != echo::BurstStatus::PartialHandled)
            abort_rx();
        out.rx_time_ticks = rx_time_ticks;
        out.tx_ticks = pending_tx_ticks;
        out.rx_ticks = impl.armed_rx_ticks;
        out.tx_samples_requested = tx_requested; // per-channel (== L)
        out.tx_samples_sent = tx_sent;           // per-channel
        out.tx_channel_count = tx_channels;
        out.tx_wire_samples_requested = // channels × per-channel
            tx_channels * tx_requested;
        out.tx_wire_samples_sent = tx_channels * tx_sent;
        out.rx_samples_received = received;
        out.rx_reissues = rx_reissues;
        out.status = st;
        out.error = error_note;
        impl.rx_armed = false;
        return true;
    };

    for (;;) {
        if (stop_requested()) {
            abort_rx();
            error_note = "stop while RX recv in flight";
            return finish(echo::BurstStatus::StopDuringIo);
        }
        const auto now = std::chrono::steady_clock::now();
        if (now >= deadline) {
            error_note = "RX window did not complete within the collect "
                         "wait budget";
            return finish(echo::BurstStatus::Timeout);
        }

        double per_call_s =
            std::chrono::duration<double>(deadline - now).count();
        per_call_s = std::min(per_call_s, d_cfg_.recv_timeout_s);

        const uint64_t remaining_samples = impl.armed_total_samples - received;
        ::uhd::rx_metadata_t md;
        size_t nrecvd = 0;
        try {
            std::vector<void*> buffs{ static_cast<void*>(cursor) };
            nrecvd = impl.rx_stream->recv(buffs, remaining_samples, md,
                                          per_call_s,
                                          /*one_packet=*/false);
        } catch (const std::exception& e) {
            error_note = uhd_error_string("rx_streamer::recv", e);
            return finish(echo::BurstStatus::BackendError);
        }

        // Error codes first: overflow/late/broken-chain VOID the burst
        // (requirement: an overflowed pulse is reported, never delivered
        // to the CIR chain) — even when a partial packet arrived.
        const echo::BurstStatus mapped = map_uhd_rx_error_code(
            static_cast<int32_t>(md.error_code));
        if (mapped != echo::BurstStatus::Ok &&
            mapped != echo::BurstStatus::Timeout) {
            error_note = std::string("rx error: ") + md.strerror();
            return finish(mapped);
        }

        if (nrecvd > 0) {
            if (!have_first_time && md.has_time_spec) {
                // gr-uhd usrp_source: the first packet's time_spec is the
                // device time of the first received sample.
                int64_t first_ticks = 0;
                if (ticks_from_time_parts(md.time_spec.get_full_secs(),
                                          md.time_spec.get_frac_secs(),
                                          d_cfg_.sample_rate_hz,
                                          first_ticks)) {
                    rx_time_ticks = first_ticks;
                    have_first_time = true;
                }
            }
            received += nrecvd;
            cursor += nrecvd * 2;
            if (nrecvd < remaining_samples)
                ++rx_reissues; // partial recv → loop re-issues
            if (received == impl.armed_total_samples) {
                // Window complete.  Attribute async TX events (§5.6):
                // take_events_for classifies EVERY drained event by code
                // (ack/underflow/seq/time) and additionally counts events it
                // cannot tie to this burst as unmatched.  The drain target
                // is a worker-owned FIXED array — no std::vector on the hot
                // path; excess matches are counted as dropped.
                std::array<Impl::AsyncEvent, Impl::kMaxAsyncEvents> events{};
                uint64_t ev_ack = 0, ev_underflow = 0, ev_seq_error = 0;
                uint64_t ev_time_error = 0, ev_unmatched = 0;
                const size_t n_events = impl.take_events_for(
                    pending_tx_ticks, events.data(), events.size(),
                    ev_ack, ev_underflow, ev_seq_error, ev_time_error,
                    ev_unmatched);
                (void)ev_ack;
                (void)ev_underflow;
                (void)ev_seq_error;
                (void)ev_time_error;
                (void)ev_unmatched; // folded into tx_async_counts()
                for (size_t k = 0; k < n_events; ++k) {
                    const auto& ev = events[k];
                    if (ev.event_code == 0 ||
                        ev.event_code == kUhdAsyncBurstAck) {
                        continue;
                    }
                    std::string note;
                    const echo::BurstStatus st =
                        map_uhd_tx_async_event(ev.event_code, note);
                    if (st != echo::BurstStatus::Ok) {
                        if (st == echo::BurstStatus::LateCommand)
                            status = echo::BurstStatus::LateCommand;
                        else if (status == echo::BurstStatus::Ok)
                            status = st;
                        error_note +=
                            error_note.empty() ? note : (", " + note);
                    }
                }
                return finish(status == echo::BurstStatus::Ok &&
                                      rx_reissues > 0
                                  ? echo::BurstStatus::PartialHandled
                                  : status);
            }
            continue;
        }

        // nrecvd == 0 with ERROR_CODE_NONE or ERROR_CODE_TIMEOUT: the
        // command time has not been reached yet (bounded by the
        // deadline above) — keep waiting.
        continue;
    }
}

// ---------------------------------------------------------------------------
// abort_rx / request_stop / device time
// ---------------------------------------------------------------------------

void UhdBurstBackend::abort_rx()
{
    if (!d_impl_ || !d_impl_->rx_stream)
        return;
    try {
        d_impl_->rx_stream->issue_stream_cmd(
            ::uhd::stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS);
    } catch (const std::exception&) {
        // Best-effort abort; the next burst arms with a fresh command.
    }
    d_impl_->rx_armed = false;
}

void UhdBurstBackend::request_stop()
{
    d_stop_requested_.store(true, std::memory_order_release);
    if (d_impl_)
        d_impl_->stop_async_thread();
}

bool UhdBurstBackend::stop_requested() const
{
    return d_stop_requested_.load(std::memory_order_acquire);
}

int64_t UhdBurstBackend::device_time_ticks() const
{
    if (!d_impl_ || !d_impl_->dev)
        return 0;
    try {
        const ::uhd::time_spec_t t = d_impl_->dev->get_time_now(0);
        int64_t ticks = 0;
        if (ticks_from_time_parts(t.get_full_secs(), t.get_frac_secs(),
                                  d_cfg_.sample_rate_hz, ticks))
            return ticks;
        return 0;
    } catch (const std::exception&) {
        return 0; // device gone: the worker skips expired grid slots
    }
}

echo::BurstStatus UhdBurstBackend::tune(double freq_hz, std::string& error)
{
    if (!d_impl_ || !d_impl_->dev) {
        error = "backend not prepared";
        return echo::BurstStatus::BackendError;
    }
    if (stop_requested()) {
        error = "backend stopped";
        return echo::BurstStatus::StopDuringIo;
    }
    if (!(freq_hz > 0.0) || !std::isfinite(freq_hz)) {
        error = "tune: freq_hz must be > 0 and finite";
        return echo::BurstStatus::BackendError;
    }
    try {
        // Same multi_usrp device: retune BOTH directions (the RX keeps the
        // deliberate rx_freq_offset_hz), then read back.  The TX leg uses
        // the frozen sense (logical-0) physical channel — identical to the
        // legacy scalar in single-TX, correct when a multi-TX config was
        // built with a divergent scalar.
        Impl& impl = *d_impl_;
        const size_t sense_phys =
            impl.tx_channel_count > 0 ? impl.tx_phys_channels[0]
                                      : d_cfg_.tx_channel;
        d_impl_->dev->set_tx_freq(::uhd::tune_request_t(freq_hz),
                                  sense_phys);
        d_impl_->dev->set_rx_freq(
            ::uhd::tune_request_t(freq_hz + d_cfg_.rx_freq_offset_hz),
            d_cfg_.rx_channel);
        const double tx = d_impl_->dev->get_tx_freq(sense_phys);
        const double rx = d_impl_->dev->get_rx_freq(d_cfg_.rx_channel);
        d_impl_->center_freq_hz.store(
            0.5 * (tx + rx - d_cfg_.rx_freq_offset_hz));
    } catch (const std::exception& e) {
        error = uhd_error_string("tune", e);
        return echo::BurstStatus::BackendError;
    }
    return echo::BurstStatus::Ok;
}

double UhdBurstBackend::center_freq_hz() const
{
    return d_impl_ ? d_impl_->center_freq_hz.load() : 0.0;
}

// ---------------------------------------------------------------------------
// Multi-TX (§5.1/§5.2/§5.5/§5.6)
// ---------------------------------------------------------------------------

size_t UhdBurstBackend::tx_channel_count() const
{
    return configured_tx_channel_count(d_cfg_);
}

echo::BurstStatus UhdBurstBackend::tune_tx_channel(size_t logical_channel,
                                                  double hz,
                                                  double& actual_hz,
                                                  std::string& error)
{
    if (!d_impl_ || !d_impl_->dev) {
        error = "backend not prepared";
        return echo::BurstStatus::BackendError;
    }
    if (stop_requested()) {
        error = "backend stopped";
        return echo::BurstStatus::StopDuringIo;
    }
    if (!(hz > 0.0) || !std::isfinite(hz)) {
        error = "tune_tx_channel: freq_hz must be > 0 and finite";
        return echo::BurstStatus::BackendError;
    }
    Impl& impl = *d_impl_;
    if (logical_channel >= impl.tx_channel_count) {
        std::ostringstream os;
        os << "tune_tx_channel: logical channel " << logical_channel
           << " out of range (tx_channel_count=" << impl.tx_channel_count
           << ")";
        error = os.str();
        return echo::BurstStatus::BackendError;
    }
    try {
        // Jammer-only: exactly one physical TX channel moves; the sense
        // TX and the RX (plus the co-tuned center_freq_hz_ reference)
        // are untouched.  Burst-boundary serialization is the caller's
        // contract (EchoTimer M5); this never runs concurrently with
        // issue_*/collect_result.
        const size_t phys = impl.tx_phys_channels[logical_channel];
        impl.dev->set_tx_freq(::uhd::tune_request_t(hz), phys);
        actual_hz = impl.dev->get_tx_freq(phys);
        // A retune that does not actually land on the requested frequency
        // must NOT be recorded as the new jammer frequency (§5.5): fail
        // explicitly so the worker keeps the previous value and retries.
        if (std::fabs(actual_hz - hz) > kUhdTxFreqReadbackTolHz) {
            std::ostringstream os;
            os << "tune_tx_channel readback mismatch on logical channel "
               << logical_channel << " (phys " << phys << "): requested="
               << hz << " actual=" << actual_hz << " (tol="
               << kUhdTxFreqReadbackTolHz << " Hz)";
            error = os.str();
            return echo::BurstStatus::BackendError;
        }
        impl.tx_actual_freq_hz[logical_channel] = actual_hz;
    } catch (const std::exception& e) {
        error = uhd_error_string("tune_tx_channel", e);
        return echo::BurstStatus::BackendError;
    }
    return echo::BurstStatus::Ok;
}

echo::TxAsyncCounts UhdBurstBackend::tx_async_counts() const
{
    echo::TxAsyncCounts c;
    if (!d_impl_)
        return c;
    const Impl& impl = *d_impl_;
    c.ack = impl.async_ack_.load(std::memory_order_relaxed);
    c.underflow = impl.async_underflow_.load(std::memory_order_relaxed);
    c.seq_error = impl.async_seq_error_.load(std::memory_order_relaxed);
    c.time_error = impl.async_time_error_.load(std::memory_order_relaxed);
    c.unmatched = impl.async_unmatched_.load(std::memory_order_relaxed);
    c.dropped = impl.async_dropped_.load(std::memory_order_relaxed);
    return c;
}

} // namespace uhd
} // namespace uwb
} // namespace gr
