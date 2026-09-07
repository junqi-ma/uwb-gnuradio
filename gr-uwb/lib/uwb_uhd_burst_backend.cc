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
#include <atomic>
#include <chrono>
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

} // namespace

struct UhdBurstBackend::Impl
{
    ::uhd::usrp::multi_usrp::sptr dev;
    ::uhd::tx_streamer::sptr tx_stream;
    ::uhd::rx_streamer::sptr rx_stream;

    // Armed RX state (issue_rx → collect_result, worker thread only).
    bool rx_armed = false;
    uint64_t armed_index = 0;
    int64_t armed_rx_ticks = 0; // commanded RX window start (ticks)
    uint64_t armed_total_samples = 0;
    int16_t* rx_base = nullptr; // burst-linear write base (fixed scratch)

    // In-flight TX state for event attribution and result reporting
    // (worker thread only).
    uint64_t pending_tx_ticks = 0;
    uint64_t pending_tx_sent = 0;

    // Async events (async thread producer, worker consumer).
    struct AsyncEvent {
        uint32_t event_code = 0;
        int64_t tx_ticks = -1;
    };
    std::mutex async_mutex;
    std::deque<AsyncEvent> async_events; // bounded, oldest dropped
    uint64_t async_events_dropped = 0;

    // Lifecycle flag doubling as "stop requested" (async thread runs
    // while true; request_stop clears it, which stop_requested reports).
    std::atomic<bool> async_run{ false };
    std::thread async_thread;

    static constexpr size_t kMaxAsyncEvents = 128;

    void push_event(const AsyncEvent& ev)
    {
        std::lock_guard<std::mutex> lock(async_mutex);
        if (async_events.size() >= kMaxAsyncEvents) {
            async_events.pop_front();
            ++async_events_dropped;
        }
        async_events.push_back(ev);
    }

    // Consume events whose device ticks match this burst's TX ticks;
    // unmatched events stay queued (their own burst will claim them).
    void take_events_for(int64_t tx_ticks, std::vector<AsyncEvent>& matched)
    {
        std::lock_guard<std::mutex> lock(async_mutex);
        for (auto it = async_events.begin(); it != async_events.end();) {
            if (it->tx_ticks == tx_ticks) {
                matched.push_back(*it);
                it = async_events.erase(it);
            } else {
                ++it;
            }
        }
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
                    static_cast<uint32_t>(md.event_code),
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

        // Strict sample-rate contract on BOTH directions: set, then read
        // back, then reject any silent coercion.
        impl->dev->set_tx_rate(d_cfg_.sample_rate_hz, d_cfg_.tx_channel);
        const double tx_rate = impl->dev->get_tx_rate(d_cfg_.tx_channel);
        if (!rate_matches_strict(d_cfg_.sample_rate_hz, tx_rate,
                                 d_cfg_.rate_tolerance_rel)) {
            std::ostringstream os;
            os << "strict rate readback failed on TX channel "
               << d_cfg_.tx_channel << ": requested=" << d_cfg_.sample_rate_hz
               << " readback=" << tx_rate
               << " (silent coercion rejected; rel_tol="
               << d_cfg_.rate_tolerance_rel << ")";
            error = os.str();
            return false;
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

        if (d_cfg_.center_freq_hz > 0.0) {
            impl->dev->set_tx_freq(
                ::uhd::tune_request_t(d_cfg_.center_freq_hz),
                d_cfg_.tx_channel);
            impl->dev->set_rx_freq(
                ::uhd::tune_request_t(d_cfg_.center_freq_hz),
                d_cfg_.rx_channel);
        }
        if (d_cfg_.tx_gain_db >= 0.0)
            impl->dev->set_tx_gain(d_cfg_.tx_gain_db, d_cfg_.tx_channel);
        if (d_cfg_.rx_gain_db >= 0.0)
            impl->dev->set_rx_gain(d_cfg_.rx_gain_db, d_cfg_.rx_channel);
        if (!d_cfg_.tx_antenna.empty())
            impl->dev->set_tx_antenna(d_cfg_.tx_antenna, d_cfg_.tx_channel);
        if (!d_cfg_.rx_antenna.empty())
            impl->dev->set_rx_antenna(d_cfg_.rx_antenna, d_cfg_.rx_channel);

        // Same device, two SC16 streamers (cpu s16 / otw sc16 — the
        // production native wire format).
        ::uhd::stream_args_t tx_args("s16", "sc16");
        tx_args.channels = std::vector<size_t>{ d_cfg_.tx_channel };
        impl->tx_stream = impl->dev->get_tx_stream(tx_args);
        ::uhd::stream_args_t rx_args("s16", "sc16");
        rx_args.channels = std::vector<size_t>{ d_cfg_.rx_channel };
        impl->rx_stream = impl->dev->get_rx_stream(rx_args);
        if (!impl->tx_stream || !impl->rx_stream) {
            error = "failed to create SC16 TX/RX streamers";
            return false;
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
    {
        std::lock_guard<std::mutex> lock(impl->async_mutex);
        impl->async_events.clear();
        impl->async_events_dropped = 0;
    }

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

    if (cmd.fragment_count == 0 || cmd.fragments == nullptr) {
        error = "broken chain: empty TX fragment list";
        return echo::BurstStatus::BrokenChain;
    }
    {
        std::string flag_err;
        if (!validate_fragment_flags(cmd.fragments, cmd.fragment_count,
                                     &flag_err)) {
            error = std::string("broken chain: ") + flag_err;
            return echo::BurstStatus::BrokenChain;
        }
    }
    if (!impl.rx_armed || impl.armed_index != cmd.schedule_index) {
        error = "broken chain: TX issued without armed RX for the same "
                "schedule index";
        return echo::BurstStatus::BrokenChain;
    }

    uint64_t sent_total = 0;
    bool first_send_call = true;
    bool saw_eob_flag = false;
    try {
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
                // gr-uhd usrp_sink mapping: time spec + SOB travel on the
                // FIRST send call of the burst only; data calls NEVER
                // carry EOB (a partial send must not have declared one).
                if (first_send_call && (f.flags & echo::kFlagTimeSpec)) {
                    int64_t full = 0;
                    double frac = 0.0;
                    if (!time_parts_from_ticks(cmd.tx_ticks,
                                               d_cfg_.sample_rate_hz, full,
                                               frac)) {
                        error = "tx_ticks out of representable device "
                                "time range";
                        return echo::BurstStatus::BackendError;
                    }
                    md.has_time_spec = true;
                    md.time_spec = ::uhd::time_spec_t(full, frac);
                }
                if (first_send_call && (f.flags & echo::kFlagStartOfBurst))
                    md.start_of_burst = true;

                const std::vector<const void*> buffs{
                    static_cast<const void*>(f.tx_data + done * 2)
                };
                const size_t ntransfered = impl.tx_stream->send(
                    buffs, remaining, md, d_cfg_.send_timeout_s);
                first_send_call = false;
                done += ntransfered;
                sent_total += ntransfered;
                // ntransfered < remaining → partial send; the loop
                // re-issues the remainder without SOB/time spec.
            }
        }

        // Burst termination: a dedicated zero-length send with EOB (the
        // gr-uhd usrp_sink stop() mechanism).  Exactly one EOB, after ALL
        // data samples were accepted — immune to partial data sends.
        if (saw_eob_flag) {
            ::uhd::tx_metadata_t md;
            md.end_of_burst = true;
            const std::vector<const void*> buffs{ nullptr };
            impl.tx_stream->send(buffs, 0, md, d_cfg_.send_timeout_s);
        }
    } catch (const std::exception& e) {
        error = uhd_error_string("tx_streamer::send", e);
        return echo::BurstStatus::BackendError;
    }

    impl.pending_tx_ticks = static_cast<uint64_t>(cmd.tx_ticks);
    impl.pending_tx_sent = sent_total;

    // Non-blocking async drain: pick up an event already queued (e.g. an
    // immediately-detected late command).  The async thread keeps
    // draining; attribution happens in collect_result.
    ::uhd::async_metadata_t amd;
    if (impl.tx_stream->recv_async_msg(amd, 0.0)) {
        impl.push_event(Impl::AsyncEvent{
            static_cast<uint32_t>(amd.event_code),
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
        out.tx_samples_sent = tx_sent;
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
                // Window complete.  Attribute the async TX events whose
                // device ticks match this burst's TX ticks.
                std::vector<Impl::AsyncEvent> events;
                impl.take_events_for(pending_tx_ticks, events);
                for (const auto& ev : events) {
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

} // namespace uhd
} // namespace uwb
} // namespace gr
