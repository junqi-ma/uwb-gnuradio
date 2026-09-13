/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * UwbEchoTimerStream implementation (Radar Stage 2).  One tagged-stream
 * burst per work() call, blocking until the backend RX completion.  The
 * burst execution is the exact sequence of
 * UwbRealtimeEchoTimer::run_one_burst(): grid slot -> fragment planning
 * (fixed arrays) -> issue_rx() BEFORE issue_tx() -> collect_result() with
 * stop handling.  No allocation is performed on the hot path: the TX/RX
 * SC16 scratch and the fragment arrays are fixed members.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gnuradio/io_signature.h>
#include <gnuradio/uwb/uwb_echo_timer_stream.h>
#include <pmt/pmt.h>

#ifdef UWB_HAVE_UHD
#include <gnuradio/uwb/uwb_uhd_burst_backend.h>
#endif

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <stdexcept>
#include <utility>

namespace gr {
namespace uwb {

namespace {

inline int16_t to_sc16(float v)
{
    const float s = v * 32768.0f;
    if (s >= 32767.0f)
        return static_cast<int16_t>(32767);
    if (s <= -32768.0f)
        return static_cast<int16_t>(-32768);
    return static_cast<int16_t>(std::lrintf(s));
}

} // namespace

// ---------------------------------------------------------------------------
// Construction / factory
// ---------------------------------------------------------------------------

UwbEchoTimerStream::UwbEchoTimerStream(
    const echo::EchoSchedulerConfig& sched_cfg,
    std::shared_ptr<echo::IRadioBurstBackend> backend,
    uint64_t tx_samples,
    uint64_t rx_samples,
    double sample_rate_hz,
    uint64_t pre_guard_samples,
    uint64_t capture_samples,
    uint64_t post_guard_samples,
    size_t sync_repetitions,
    const std::string& sfd_mode,
    size_t code_index,
    double calibration_delay_native_samples,
    int64_t t0_ticks,
    double arm_margin_s,
    uint64_t max_frames,
    uint64_t collect_wait_ms,
    size_t max_tx_samples,
    size_t max_rx_samples,
    const std::string& lengthtagname)
    : gr::tagged_stream_block(
          "uwb_echo_timer_stream",
          gr::io_signature::make(1, 1, sizeof(gr_complex)),
          gr::io_signature::make(1, 1, sizeof(gr_complex)),
          lengthtagname),
      d_backend_(std::move(backend)),
      d_tx_samples_(tx_samples),
      d_rx_samples_(rx_samples),
      d_sample_rate_hz_(sample_rate_hz),
      d_pre_guard_samples_(pre_guard_samples),
      d_capture_samples_(capture_samples),
      d_post_guard_samples_(post_guard_samples),
      d_sync_repetitions_(sync_repetitions),
      d_sfd_mode_(sfd_mode),
      d_code_index_(code_index),
      d_t0_ticks_(t0_ticks),
      d_arm_margin_s_(arm_margin_s),
      d_max_frames_(max_frames),
      d_collect_wait_ms_(collect_wait_ms),
      d_max_tx_samples_(max_tx_samples),
      d_max_rx_samples_(max_rx_samples),
      d_cal_delay_native_(calibration_delay_native_samples),
      d_grid_(d_sched_)
{
    if (!d_backend_)
        throw std::invalid_argument(
            "UwbEchoTimerStream: backend must not be null");
    if (tx_samples == 0 || rx_samples == 0)
        throw std::invalid_argument(
            "UwbEchoTimerStream: tx_samples and rx_samples must be > 0");
    if (max_tx_samples == 0 || max_rx_samples == 0)
        throw std::invalid_argument(
            "UwbEchoTimerStream: max_tx_samples and max_rx_samples must be "
            "> 0");
    if (tx_samples > static_cast<uint64_t>(max_tx_samples) ||
        rx_samples > static_cast<uint64_t>(max_rx_samples))
        throw std::invalid_argument(
            "UwbEchoTimerStream: tx/rx samples exceed the fixed scratch caps");
    // 2 * cap int16 elements must fit size_t; the output length (and the
    // min output buffer) must fit int/long.
    if (max_tx_samples >
            std::numeric_limits<size_t>::max() / 2 ||
        max_rx_samples >
            std::numeric_limits<size_t>::max() / 2 ||
        rx_samples >
            static_cast<uint64_t>(std::numeric_limits<int>::max()))
        throw std::invalid_argument(
            "UwbEchoTimerStream: scratch caps / rx_samples too large");
    if (!(sample_rate_hz > 0.0) || !std::isfinite(sample_rate_hz))
        throw std::invalid_argument(
            "UwbEchoTimerStream: sample_rate_hz must be > 0 and finite");
    if (collect_wait_ms == 0)
        throw std::invalid_argument(
            "UwbEchoTimerStream: collect_wait_ms must be > 0");
    if (!(arm_margin_s >= 0.0) || !std::isfinite(arm_margin_s))
        throw std::invalid_argument(
            "UwbEchoTimerStream: arm_margin_s must be >= 0 and finite");

    // One-time validation of the frozen grid identity (the per-burst path
    // only reads the prepared struct).
    std::string err;
    if (!echo::prepare_echo_scheduler(sched_cfg, d_sched_, &err))
        throw std::invalid_argument("UwbEchoTimerStream: " + err);

    // Fixed scratch: allocated exactly once here, never resized.
    d_tx_sc16_.assign(static_cast<size_t>(tx_samples) * 2, 0);
    d_rx_buf_.assign(static_cast<size_t>(max_rx_samples) * 2, 0);

    set_tag_propagation_policy(TPP_DONT);
    set_min_output_buffer(0, static_cast<long>(4 * rx_samples));
}

UwbEchoTimerStream::~UwbEchoTimerStream() = default;

UwbEchoTimerStream::sptr UwbEchoTimerStream::make(
    const echo::EchoSchedulerConfig& sched_cfg,
    std::shared_ptr<echo::IRadioBurstBackend> backend,
    uint64_t tx_samples,
    uint64_t rx_samples,
    double sample_rate_hz,
    uint64_t pre_guard_samples,
    uint64_t capture_samples,
    uint64_t post_guard_samples,
    size_t sync_repetitions,
    const std::string& sfd_mode,
    size_t code_index,
    double calibration_delay_native_samples,
    int64_t t0_ticks,
    double arm_margin_s,
    uint64_t max_frames,
    uint64_t collect_wait_ms,
    size_t max_tx_samples,
    size_t max_rx_samples,
    const std::string& lengthtagname)
{
    return gnuradio::make_block_sptr<UwbEchoTimerStream>(
        sched_cfg,
        std::move(backend),
        tx_samples,
        rx_samples,
        sample_rate_hz,
        pre_guard_samples,
        capture_samples,
        post_guard_samples,
        sync_repetitions,
        sfd_mode,
        code_index,
        calibration_delay_native_samples,
        t0_ticks,
        arm_margin_s,
        max_frames,
        collect_wait_ms,
        max_tx_samples,
        max_rx_samples,
        lengthtagname);
}

#ifdef UWB_HAVE_UHD
UwbEchoTimerStream::sptr UwbEchoTimerStream::make_uhd(
    const uhd::UhdBurstBackendConfig& uhd_cfg,
    const echo::EchoSchedulerConfig& sched_cfg,
    uint64_t tx_samples,
    uint64_t rx_samples,
    double sample_rate_hz,
    uint64_t pre_guard_samples,
    uint64_t capture_samples,
    uint64_t post_guard_samples,
    size_t sync_repetitions,
    const std::string& sfd_mode,
    size_t code_index,
    double calibration_delay_native_samples,
    int64_t t0_ticks,
    double arm_margin_s,
    uint64_t max_frames,
    uint64_t collect_wait_ms,
    size_t max_tx_samples,
    size_t max_rx_samples,
    const std::string& lengthtagname)
{
    auto backend = std::make_shared<uhd::UhdBurstBackend>(uhd_cfg);
    return make(sched_cfg,
                std::move(backend),
                tx_samples,
                rx_samples,
                sample_rate_hz,
                pre_guard_samples,
                capture_samples,
                post_guard_samples,
                sync_repetitions,
                sfd_mode,
                code_index,
                calibration_delay_native_samples,
                t0_ticks,
                arm_margin_s,
                max_frames,
                collect_wait_ms,
                max_tx_samples,
                max_rx_samples,
                lengthtagname);
}
#endif

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

bool UwbEchoTimerStream::start()
{
    std::string err;
    if (!d_backend_->prepare(err))
        return false;
    d_stop_.store(false, std::memory_order_relaxed);
    d_grid_armed_ = false;
    d_armed_t0_ = 0;
    d_grid_ = echo::EchoGrid(d_sched_);
    d_frames_.store(0, std::memory_order_relaxed);
    d_bursts_ok_.store(0, std::memory_order_relaxed);
    d_bursts_failed_.store(0, std::memory_order_relaxed);
    d_late_slot_skips_.store(0, std::memory_order_relaxed);
    d_grid_errors_.store(0, std::memory_order_relaxed);
    d_rx_samples_total_.store(0, std::memory_order_relaxed);
    d_tx_samples_total_.store(0, std::memory_order_relaxed);
    return true;
}

bool UwbEchoTimerStream::stop()
{
    d_stop_.store(true, std::memory_order_relaxed);
    d_backend_->request_stop();
    return true;
}

// ---------------------------------------------------------------------------
// Thread-safe accessors
// ---------------------------------------------------------------------------

void UwbEchoTimerStream::set_freq(double hz)
{
    std::string err;
    if (d_backend_->tune(hz, err) == echo::BurstStatus::Ok)
        d_freq_hz_.store(hz, std::memory_order_relaxed);
}

double UwbEchoTimerStream::freq() const
{
    return d_freq_hz_.load(std::memory_order_relaxed);
}

void UwbEchoTimerStream::set_cal_delay_native(double v)
{
    d_cal_delay_native_.store(v, std::memory_order_relaxed);
}

double UwbEchoTimerStream::cal_delay_native() const
{
    return d_cal_delay_native_.load(std::memory_order_relaxed);
}

// ---------------------------------------------------------------------------
// Tagged-stream length and burst execution
// ---------------------------------------------------------------------------

int UwbEchoTimerStream::calculate_output_stream_length(
    const gr_vector_int& /*ninput_items*/)
{
    // Fixed output geometry: one RX window per input packet.
    return static_cast<int>(d_rx_samples_);
}

void UwbEchoTimerStream::convert_tx(const gr_complex* in, int nin)
{
    const size_t n = static_cast<size_t>(d_tx_samples_);
    if (in != nullptr && nin == static_cast<int>(d_tx_samples_)) {
        for (size_t i = 0; i < n; ++i) {
            d_tx_sc16_[2 * i] = to_sc16(in[i].real());
            d_tx_sc16_[2 * i + 1] = to_sc16(in[i].imag());
        }
    } else {
        // Length mismatch: emit a zero TX burst rather than dropping the
        // stream (the RX window is still produced).
        std::fill_n(d_tx_sc16_.begin(),
                    static_cast<size_t>(2 * d_tx_samples_),
                    static_cast<int16_t>(0));
    }
}

void UwbEchoTimerStream::finish_burst(gr_complex* out,
                                      const echo::BurstResult& r,
                                      bool ok)
{
    {
        std::lock_guard<std::mutex> lk(d_err_mutex_);
        d_last_error_ = ok ? std::string() : r.error;
    }
    const size_t n = static_cast<size_t>(d_rx_samples_);
    if (ok) {
        for (size_t i = 0; i < n; ++i) {
            out[i] = gr_complex(
                static_cast<float>(d_rx_buf_[2 * i]) / 32768.0f,
                static_cast<float>(d_rx_buf_[2 * i + 1]) / 32768.0f);
        }
    } else {
        std::fill_n(out, n, gr_complex(0.0f, 0.0f));
    }

    const uint64_t out0 = nitems_written(0);
    add_item_tag(0, out0, pmt::mp("pulse_id"),
                 pmt::from_uint64(r.schedule_index));
    add_item_tag(0, out0, pmt::mp("schedule_index"),
                 pmt::from_uint64(r.schedule_index));
    add_item_tag(0, out0, pmt::mp("sample_rate"),
                 pmt::from_double(d_sample_rate_hz_));
    add_item_tag(0, out0, pmt::mp("window_start_sample"),
                 pmt::from_long(0));
    add_item_tag(0, out0, pmt::mp("pre_guard_samples"),
                 pmt::from_long(static_cast<long>(d_pre_guard_samples_)));
    add_item_tag(0, out0, pmt::mp("capture_samples"),
                 pmt::from_long(static_cast<long>(d_capture_samples_)));
    add_item_tag(0, out0, pmt::mp("post_guard_samples"),
                 pmt::from_long(static_cast<long>(d_post_guard_samples_)));
    add_item_tag(0, out0, pmt::mp("sample_count"),
                 pmt::from_long(static_cast<long>(d_rx_samples_)));
    add_item_tag(
        0, out0, pmt::mp("calibration_delay_native_samples"),
        pmt::from_double(d_cal_delay_native_.load(std::memory_order_relaxed)));
    add_item_tag(0, out0, pmt::mp("sync_repetitions"),
                 pmt::from_long(static_cast<long>(d_sync_repetitions_)));
    add_item_tag(0, out0, pmt::mp("sfd_mode"), pmt::intern(d_sfd_mode_));
    add_item_tag(0, out0, pmt::mp("code_index"),
                 pmt::from_long(static_cast<long>(d_code_index_)));

    if (ok && r.rx_time_ticks >= 0 && d_sample_rate_hz_ > 0.0) {
        const int64_t rate =
            static_cast<int64_t>(std::llround(d_sample_rate_hz_));
        if (rate > 0) {
            const int64_t ticks = r.rx_time_ticks;
            const int64_t full = ticks / rate;
            const double frac =
                static_cast<double>(ticks % rate) / static_cast<double>(rate);
            add_item_tag(0, out0, pmt::mp("rx_time"),
                         pmt::make_tuple(
                             pmt::from_uint64(
                                 static_cast<uint64_t>(full)),
                             pmt::from_double(frac)));
        }
    } else if (!ok) {
        add_item_tag(0, out0, pmt::mp("burst_status"),
                     pmt::mp(echo::burst_status_to_string(r.status)));
    }

    if (ok)
        d_bursts_ok_.fetch_add(1, std::memory_order_relaxed);
    else
        d_bursts_failed_.fetch_add(1, std::memory_order_relaxed);
    d_frames_.fetch_add(1, std::memory_order_relaxed);
    d_rx_samples_total_.fetch_add(d_rx_samples_, std::memory_order_relaxed);
    d_tx_samples_total_.fetch_add(d_tx_samples_, std::memory_order_relaxed);
}

int UwbEchoTimerStream::work(int /*noutput_items*/,
                             gr_vector_int& ninput_items,
                             gr_vector_const_void_star& input_items,
                             gr_vector_void_star& output_items)
{
    if (d_stop_.load(std::memory_order_relaxed) ||
        d_backend_->stop_requested())
        return WORK_DONE;
    if (d_max_frames_ > 0 &&
        d_frames_.load(std::memory_order_relaxed) >= d_max_frames_)
        return WORK_DONE;

    auto* out = static_cast<gr_complex*>(output_items[0]);
    const auto* in = static_cast<const gr_complex*>(input_items[0]);
    convert_tx(in, ninput_items[0]);

    // Lazy arm on the first burst: t0 from the config, or (t0_ticks == 0)
    // device time plus the arm margin.
    if (!d_grid_armed_) {
        int64_t t0 = d_t0_ticks_;
        if (t0 == 0) {
            const int64_t margin_ticks = static_cast<int64_t>(
                std::llround(d_arm_margin_s_ * d_sample_rate_hz_));
            const int64_t now = d_backend_->device_time_ticks();
            if (!radar::radar_i64_add(now, margin_ticks, t0)) {
                d_grid_errors_.fetch_add(1, std::memory_order_relaxed);
                echo::BurstResult er;
                er.status = echo::BurstStatus::BackendError;
                finish_burst(out, er, false);
                d_stop_.store(true, std::memory_order_relaxed);
                return static_cast<int>(d_rx_samples_);
            }
        }
        if (!d_grid_.arm(t0, 0)) {
            d_grid_errors_.fetch_add(1, std::memory_order_relaxed);
            echo::BurstResult er;
            er.status = echo::BurstStatus::BackendError;
            finish_burst(out, er, false);
            d_stop_.store(true, std::memory_order_relaxed);
            return static_cast<int>(d_rx_samples_);
        }
        d_grid_armed_ = true;
        d_armed_t0_ = t0;
    }

    // Next future grid slot (expired slots skipped, never caught up).
    echo::EchoSlot slot;
    const int64_t now = d_backend_->device_time_ticks();
    const echo::EchoScheduleStatus gs = d_grid_.next_schedule(now, slot);
    if (gs != echo::EchoScheduleStatus::Ok &&
        gs != echo::EchoScheduleStatus::ExpiredSkipped) {
        d_grid_errors_.fetch_add(1, std::memory_order_relaxed);
        echo::BurstResult er;
        er.status = echo::BurstStatus::BackendError;
        finish_burst(out, er, false);
        d_stop_.store(true, std::memory_order_relaxed);
        return static_cast<int>(d_rx_samples_);
    }
    d_late_slot_skips_.fetch_add(slot.skipped, std::memory_order_relaxed);

    // Fixed RX scratch: zero the used prefix only.
    std::fill_n(d_rx_buf_.begin(),
                static_cast<size_t>(d_rx_samples_) * 2,
                static_cast<int16_t>(0));

    echo::BurstResult r;
    r.schedule_index = slot.index;
    r.tx_ticks = slot.t_tx_whole;
    r.rx_ticks = slot.t_rx_whole;
    r.skipped_slots = slot.skipped;
    r.tx_samples_requested = d_tx_samples_;
    r.rx_samples_requested = d_rx_samples_;

    // Finalize a failed burst: keep the stream flowing with a zero window.
    auto emit_failure = [&](echo::BurstStatus st, const std::string& e) -> int {
        echo::BurstResult fr = r;
        fr.status = st;
        fr.error = e;
        d_backend_->abort_rx();
        finish_burst(out, fr, false);
        return static_cast<int>(d_rx_samples_);
    };

    // Fragment planning (fixed-size arrays; no hot-path allocation).
    echo::EchoFragmentSpan tx_spans[echo::kEchoMaxFragmentsPerBurst];
    echo::EchoFragmentSpan rx_spans[echo::kEchoMaxFragmentsPerBurst];
    size_t ntx = 0;
    size_t nrx = 0;
    if (!echo::plan_fragments(d_tx_samples_, d_sched_.max_fragment_size,
                              tx_spans, echo::kEchoMaxFragmentsPerBurst, ntx) ||
        !echo::plan_fragments(d_rx_samples_, d_sched_.max_fragment_size,
                              rx_spans, echo::kEchoMaxFragmentsPerBurst, nrx)) {
        return emit_failure(echo::BurstStatus::BackendError,
                            "fragment planning failed");
    }
    for (size_t i = 0; i < ntx; ++i) {
        echo::BurstFragment f;
        f.tx_data = d_tx_sc16_.data() + tx_spans[i].offset * 2;
        f.offset = tx_spans[i].offset;
        f.count = tx_spans[i].count;
        f.flags = 0;
        if (i == 0) { // first TX fragment: time spec + SOB
            f.flags |= echo::kFlagTimeSpec | echo::kFlagStartOfBurst;
            f.device_ticks = slot.t_tx_whole;
        }
        if (i + 1 == ntx) // last TX fragment: EOB
            f.flags |= echo::kFlagEndOfBurst;
        d_txf_[i] = f;
    }
    for (size_t i = 0; i < nrx; ++i) {
        echo::BurstFragment f;
        f.rx_data = d_rx_buf_.data() + rx_spans[i].offset * 2;
        f.offset = rx_spans[i].offset;
        f.count = rx_spans[i].count;
        f.flags = 0;
        if (i == 0) { // first RX fragment: time spec + SOB
            f.flags |= echo::kFlagTimeSpec | echo::kFlagStartOfBurst;
            f.device_ticks = slot.t_rx_whole;
        }
        if (i + 1 == nrx) // last RX fragment: EOB
            f.flags |= echo::kFlagEndOfBurst;
        d_rxf_[i] = f;
    }

    const echo::RxCommand rx_cmd{
        r.schedule_index, slot.t_rx_whole, d_rx_samples_, d_rxf_, nrx
    };
    const echo::TxCommand tx_cmd{
        r.schedule_index, slot.t_tx_whole, d_tx_samples_, d_txf_, ntx
    };

    std::string err;
    // RX command BEFORE the TX burst.
    const echo::BurstStatus rx_st = d_backend_->issue_rx(rx_cmd, err);
    if (rx_st != echo::BurstStatus::Ok)
        return emit_failure(rx_st, err);
    if (d_stop_.load(std::memory_order_relaxed) ||
        d_backend_->stop_requested())
        return emit_failure(echo::BurstStatus::StopDuringIo,
                            "stop after RX command, before TX");

    const echo::BurstStatus tx_st = d_backend_->issue_tx(tx_cmd, err);
    if (tx_st != echo::BurstStatus::Ok)
        return emit_failure(tx_st, err);
    if (d_stop_.load(std::memory_order_relaxed) ||
        d_backend_->stop_requested())
        return emit_failure(echo::BurstStatus::StopDuringIo,
                            "stop after TX burst, before RX collect");

    // Asynchronous RX-completion wait (never a busy wait).
    echo::BurstResult done;
    if (d_backend_->collect_result(r.schedule_index, d_collect_wait_ms_,
                                   done)) {
        r.status = done.status;
        r.rx_time_ticks = done.rx_time_ticks;
        r.tx_samples_sent = done.tx_samples_sent;
        r.rx_samples_received = done.rx_samples_received;
        r.tx_reissues = done.tx_reissues;
        r.rx_reissues = done.rx_reissues;
        r.error = done.error;
        const bool ok = r.status == echo::BurstStatus::Ok ||
                        r.status == echo::BurstStatus::PartialHandled;
        finish_burst(out, r, ok);
        return static_cast<int>(d_rx_samples_);
    }
    if (d_stop_.load(std::memory_order_relaxed) ||
        d_backend_->stop_requested())
        return emit_failure(echo::BurstStatus::StopDuringIo,
                            "stop while RX I/O in flight");
    return emit_failure(echo::BurstStatus::Timeout,
                        "RX did not complete within collect wait");
}

} // namespace uwb
} // namespace gr
