/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * UwbRealtimeEchoTimer implementation.  Style mirrors
 * UwbRadarCirEstimator / UwbCirWriter: bounded queue, message_port_pub
 * from the worker thread, publish_status snapshots, stop() drains then
 * joins, start() idempotent with counter reset.  The handler never
 * computes and never blocks; all grid math and backend I/O live on the
 * dedicated scheduling worker.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gnuradio/uwb/uwb_realtime_echo_timer.h>
#include <gnuradio/uwb/uwb_radar_pdu_meta.h>
#include <gnuradio/io_signature.h>

#ifdef UWB_HAVE_UHD
#include <gnuradio/uwb/uwb_uhd_burst_backend.h>
#endif

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <new>
#include <stdexcept>
#include <utility>

namespace gr {
namespace uwb {

namespace {

uint64_t dict_u64(pmt::pmt_t dict, const char* key, uint64_t def)
{
    if (!pmt::is_dict(dict))
        return def;
    pmt::pmt_t v = pmt::dict_ref(dict, pmt::mp(key), pmt::from_uint64(def));
    if (pmt::is_uint64(v))
        return pmt::to_uint64(v);
    if (pmt::is_integer(v))
        return static_cast<uint64_t>(pmt::to_long(v));
    return def;
}

int64_t dict_i64(pmt::pmt_t dict, const char* key, int64_t def)
{
    if (!pmt::is_dict(dict))
        return def;
    pmt::pmt_t v = pmt::dict_ref(dict, pmt::mp(key), pmt::from_long(def));
    if (pmt::is_uint64(v))
        return static_cast<int64_t>(pmt::to_uint64(v));
    if (pmt::is_integer(v))
        return pmt::to_long(v);
    return def;
}

double dict_f64(pmt::pmt_t dict, const char* key, double def)
{
    if (!pmt::is_dict(dict))
        return def;
    pmt::pmt_t v = pmt::dict_ref(dict, pmt::mp(key), pmt::PMT_NIL);
    if (pmt::is_real(v) || pmt::is_integer(v) || pmt::is_uint64(v))
        return pmt::to_double(v);
    return def;
}

bool dict_has(pmt::pmt_t dict, const char* key)
{
    return pmt::is_dict(dict) && pmt::dict_has_key(dict, pmt::mp(key));
}

// Strict i64 metadata read: rejects non-integers and uint64 values that do
// not fit int64 (dict_i64 above wraps those silently; schedule validation
// must not accept them).
bool dict_i64_strict(pmt::pmt_t dict, const char* key, int64_t& out)
{
    if (!pmt::is_dict(dict))
        return false;
    const pmt::pmt_t v =
        pmt::dict_ref(dict, pmt::mp(key), pmt::PMT_NIL);
    if (pmt::is_integer(v) && !pmt::is_uint64(v)) {
        out = pmt::to_long(v);
        return true;
    }
    if (pmt::is_uint64(v) &&
        pmt::to_uint64(v) <=
            static_cast<uint64_t>(std::numeric_limits<int64_t>::max())) {
        out = static_cast<int64_t>(pmt::to_uint64(v));
        return true;
    }
    return false;
}

// List-like PMT containers accepted for schedule vectors: PMT vectors and
// tuples (Python lists may arrive as either), plus the numeric vector types
// the jam app emits (u64vector for lengths/offsets, f64vector for freqs).
bool is_pmt_list(pmt::pmt_t v)
{
    return pmt::is_vector(v) || pmt::is_tuple(v);
}

size_t pmt_list_len(pmt::pmt_t v)
{
    return pmt::length(v); // valid for vectors and tuples
}

pmt::pmt_t pmt_list_ref(pmt::pmt_t v, size_t i)
{
    return pmt::is_vector(v) ? pmt::vector_ref(v, i)
                             : pmt::tuple_ref(v, i);
}

// Read a fixed-size u64 list (tx_waveform_samples / tx_base_offsets_native).
// Accepts u64vector/s32vector/vector/tuple with integer/uint64 entries.
bool read_u64_list(pmt::pmt_t v,
                   size_t want_n,
                   uint64_t* out,
                   std::string& error)
{
    if (pmt::is_u64vector(v)) {
        size_t len = 0;
        (void)pmt::u64vector_elements(v, len);
        if (len != want_n) {
            error = "u64 list length mismatch";
            return false;
        }
        const uint64_t* el = pmt::u64vector_elements(v, len);
        for (size_t i = 0; i < want_n; ++i)
            out[i] = el[i];
        return true;
    }
    if (pmt::is_s32vector(v)) {
        size_t len = 0;
        (void)pmt::s32vector_elements(v, len);
        if (len != want_n) {
            error = "s32 list length mismatch";
            return false;
        }
        const int32_t* el = pmt::s32vector_elements(v, len);
        for (size_t i = 0; i < want_n; ++i) {
            if (el[i] < 0) {
                error = "list entry must be >= 0";
                return false;
            }
            out[i] = static_cast<uint64_t>(el[i]);
        }
        return true;
    }
    if (!is_pmt_list(v) || pmt_list_len(v) != want_n) {
        error = "list length mismatch (want vector/tuple/u64vector)";
        return false;
    }
    for (size_t i = 0; i < want_n; ++i) {
        const pmt::pmt_t e = pmt_list_ref(v, i);
        if (pmt::is_uint64(e)) {
            out[i] = pmt::to_uint64(e);
        } else if (pmt::is_integer(e)) {
            const long l = pmt::to_long(e);
            if (l < 0) {
                error = "list entry must be >= 0";
                return false;
            }
            out[i] = static_cast<uint64_t>(l);
        } else {
            error = "list entry must be an integer";
            return false;
        }
    }
    return true;
}

// Parse jam_delay_mode: "fixed"/"uniform" symbols or 0/1 integers.
bool parse_jam_delay_mode(pmt::pmt_t dict,
                          gr::uwb::echo::JamDelayMode& out,
                          std::string& error)
{
    using gr::uwb::echo::JamDelayMode;
    if (!dict_has(dict, "jam_delay_mode")) {
        out = JamDelayMode::Fixed;
        return true;
    }
    const pmt::pmt_t v =
        pmt::dict_ref(dict, pmt::mp("jam_delay_mode"), pmt::PMT_NIL);
    if (pmt::is_symbol(v)) {
        const std::string s = pmt::symbol_to_string(v);
        if (s == "fixed") {
            out = JamDelayMode::Fixed;
            return true;
        }
        if (s == "uniform") {
            out = JamDelayMode::Uniform;
            return true;
        }
        error = "jam_delay_mode must be fixed/uniform";
        return false;
    }
    if (pmt::is_uint64(v) || pmt::is_integer(v)) {
        const long l = pmt::is_uint64(v)
                           ? static_cast<long>(pmt::to_uint64(v))
                           : pmt::to_long(v);
        if (l == 0) {
            out = JamDelayMode::Fixed;
            return true;
        }
        if (l == 1) {
            out = JamDelayMode::Uniform;
            return true;
        }
    }
    error = "jam_delay_mode must be fixed/uniform (or 0/1)";
    return false;
}

// Steady-clock delta in whole microseconds, clamped at 0 (diagnostics).
uint64_t elapsed_us(const std::chrono::steady_clock::time_point& a,
                    const std::chrono::steady_clock::time_point& b)
{
    const auto us =
        std::chrono::duration_cast<std::chrono::microseconds>(b - a).count();
    return us > 0 ? static_cast<uint64_t>(us) : uint64_t{ 0 };
}

// Accumulate a segment duration into its atomic total and running max.
// Relaxed ordering, no allocation, never throws; diagnostics only.
void accumulate_us(std::atomic<uint64_t>& total,
                   std::atomic<uint64_t>& max,
                   uint64_t us)
{
    total.fetch_add(us, std::memory_order_relaxed);
    uint64_t cur = max.load(std::memory_order_relaxed);
    while (us > cur &&
           !max.compare_exchange_weak(cur, us, std::memory_order_relaxed)) {
    }
}

// channels × per-channel transport accounting (§5.2), saturating instead of
// wrapping on absurd inputs (unreachable: caps bound both operands).
uint64_t wire_sat(size_t nch, uint64_t per_ch)
{
    if (nch == 0 || per_ch == 0)
        return 0;
    if (per_ch >
        std::numeric_limits<uint64_t>::max() / static_cast<uint64_t>(nch))
        return std::numeric_limits<uint64_t>::max();
    return static_cast<uint64_t>(nch) * per_ch;
}

} // namespace

// ---------------------------------------------------------------------------
// Construction / factory
// ---------------------------------------------------------------------------

UwbRealtimeEchoTimer::UwbRealtimeEchoTimer(
    const echo::EchoSchedulerConfig& sched_cfg,
    std::shared_ptr<echo::IRadioBurstBackend> backend,
    size_t queue_capacity,
    uint64_t rx_collect_wait_ms,
    uint64_t max_tx_samples,
    uint64_t max_rx_samples)
    : gr::block("uwb_realtime_echo_timer",
                gr::io_signature::make(0, 0, 0),
                gr::io_signature::make(0, 0, 0)),
      d_backend_(std::move(backend)),
      d_queue_capacity_(queue_capacity),
      d_collect_wait_ms_(rx_collect_wait_ms),
      d_max_tx_samples_(max_tx_samples),
      d_max_rx_samples_(max_rx_samples),
      d_grid_(d_sched_)
{
    if (!d_backend_)
        throw std::invalid_argument(
            "UwbRealtimeEchoTimer: backend must not be null");
    if (queue_capacity == 0)
        throw std::invalid_argument(
            "UwbRealtimeEchoTimer: queue_capacity must be > 0");
    if (rx_collect_wait_ms == 0)
        throw std::invalid_argument(
            "UwbRealtimeEchoTimer: rx_collect_wait_ms must be > 0");
    // Fixed scratch caps: strictly positive, and the SC16 pair count
    // doubled into int16 elements must stay inside size_t (no wraparound,
    // no unbounded allocation anywhere on the hot path).
    if (max_tx_samples == 0 || max_rx_samples == 0)
        throw std::invalid_argument(
            "UwbRealtimeEchoTimer: max_tx_samples and max_rx_samples "
            "must be > 0");
    if (max_tx_samples >
            static_cast<uint64_t>(std::numeric_limits<size_t>::max() / 2) ||
        max_rx_samples >
            static_cast<uint64_t>(std::numeric_limits<size_t>::max() / 2))
        throw std::invalid_argument(
            "UwbRealtimeEchoTimer: sample caps too large "
            "(2 * cap must fit size_t)");

    // One-time validation of the frozen grid identity (prepare-style
    // config function; the per-burst path only reads the prepared struct).
    std::string err;
    if (!echo::prepare_echo_scheduler(sched_cfg, d_sched_, &err))
        throw std::invalid_argument("UwbRealtimeEchoTimer: " + err);

    // Fixed RX scratch: allocated exactly once here, never resized.  The
    // worker only zero-fills the used prefix when a schedule is armed.
    d_rx_buf_.assign(static_cast<size_t>(max_rx_samples) * 2, 0);
    // Fixed TX zero scratch (multi-TX, §5.4): one all-zero region of
    // max_tx_samples pairs; per-burst fragments point into it wherever no
    // channel waveform is active.  Worker read-only; address stable.
    d_zero_scratch_.assign(static_cast<size_t>(max_tx_samples) * 2, 0);
    // Pre-reserve re-arm holders so re-arming a schedule never allocates:
    // channel PMT refs (<= kEchoMaxTxChannels) and the dwell scan plan.
    d_mtx_payloads_.reserve(echo::kEchoMaxTxChannels);
    d_mtx_freqs_.reserve(2048);

    message_port_register_in(pmt::mp("schedule"));
    message_port_register_out(pmt::mp("burst"));
    message_port_register_out(pmt::mp("status"));
    set_msg_handler(pmt::mp("schedule"),
                    [this](pmt::pmt_t msg) { handle_schedule(msg); });
}

UwbRealtimeEchoTimer::~UwbRealtimeEchoTimer()
{
    // Join the worker even if stop() was never called (I/O in flight is
    // aborted through the backend so the join cannot deadlock).
    stop_worker_and_join();
}

std::shared_ptr<UwbRealtimeEchoTimer>
UwbRealtimeEchoTimer::make(const echo::EchoSchedulerConfig& sched_cfg,
                           std::shared_ptr<echo::IRadioBurstBackend> backend,
                           size_t queue_capacity,
                           uint64_t rx_collect_wait_ms,
                           uint64_t max_tx_samples,
                           uint64_t max_rx_samples)
{
    return gnuradio::get_initial_sptr(
        new UwbRealtimeEchoTimer(sched_cfg, std::move(backend),
                                 queue_capacity, rx_collect_wait_ms,
                                 max_tx_samples, max_rx_samples));
}

#ifdef UWB_HAVE_UHD
std::shared_ptr<UwbRealtimeEchoTimer>
UwbRealtimeEchoTimer::make_uhd(const uhd::UhdBurstBackendConfig& uhd_cfg,
                               const echo::EchoSchedulerConfig& sched_cfg,
                               size_t queue_capacity,
                               uint64_t rx_collect_wait_ms,
                               uint64_t max_tx_samples,
                               uint64_t max_rx_samples)
{
    auto backend = std::make_shared<uhd::UhdBurstBackend>(uhd_cfg);
    return make(sched_cfg, std::move(backend), queue_capacity,
                rx_collect_wait_ms, max_tx_samples, max_rx_samples);
}
#endif

// ---------------------------------------------------------------------------
// Stats accessors
// ---------------------------------------------------------------------------

uint64_t UwbRealtimeEchoTimer::schedules_received() const
{
    return d_received_.load(std::memory_order_relaxed);
}
uint64_t UwbRealtimeEchoTimer::schedules_enqueued() const
{
    return d_enqueued_.load(std::memory_order_relaxed);
}
uint64_t UwbRealtimeEchoTimer::schedules_dropped() const
{
    return d_dropped_.load(std::memory_order_relaxed);
}
uint64_t UwbRealtimeEchoTimer::schedules_invalid() const
{
    return d_invalid_.load(std::memory_order_relaxed);
}
uint64_t UwbRealtimeEchoTimer::schedules_dropped_on_stop() const
{
    return d_dropped_on_stop_.load(std::memory_order_relaxed);
}
uint64_t UwbRealtimeEchoTimer::bursts_published() const
{
    return d_bursts_published_.load(std::memory_order_relaxed);
}
uint64_t UwbRealtimeEchoTimer::bursts_ok() const
{
    return d_bursts_ok_.load(std::memory_order_relaxed);
}
uint64_t UwbRealtimeEchoTimer::bursts_failed() const
{
    return d_bursts_failed_.load(std::memory_order_relaxed);
}
uint64_t UwbRealtimeEchoTimer::late_slot_skips() const
{
    return d_late_slot_skips_.load(std::memory_order_relaxed);
}
uint64_t UwbRealtimeEchoTimer::tx_reissues() const
{
    return d_tx_reissues_.load(std::memory_order_relaxed);
}
uint64_t UwbRealtimeEchoTimer::rx_reissues() const
{
    return d_rx_reissues_.load(std::memory_order_relaxed);
}
uint64_t UwbRealtimeEchoTimer::grid_errors() const
{
    return d_grid_errors_.load(std::memory_order_relaxed);
}
size_t UwbRealtimeEchoTimer::queue_depth() const
{
    return d_queue_depth_.load(std::memory_order_relaxed);
}
size_t UwbRealtimeEchoTimer::queue_high_watermark() const
{
    return d_queue_high_watermark_.load(std::memory_order_relaxed);
}

// ---------------------------------------------------------------------------
// Runtime control / publish ROI / worker timing (M0/M1/M2)
// ---------------------------------------------------------------------------

void UwbRealtimeEchoTimer::set_freq(double hz)
{
    // Store a pending tune only; the radio worker applies it at the next
    // burst boundary so it is serialized with all UHD I/O (never a
    // concurrent tune from a Python/message thread).
    if (hz > 0.0 && std::isfinite(hz)) {
        d_tune_hz_.store(hz, std::memory_order_relaxed);
        d_tune_pending_.store(true, std::memory_order_relaxed);
    }
}

double UwbRealtimeEchoTimer::freq() const
{
    return d_freq_hz_.load(std::memory_order_relaxed);
}

void UwbRealtimeEchoTimer::set_cal_delay_native(double native_samples)
{
    d_cal_delay_native_.store(native_samples, std::memory_order_relaxed);
}

double UwbRealtimeEchoTimer::cal_delay_native() const
{
    return d_cal_delay_native_.load(std::memory_order_relaxed);
}

void UwbRealtimeEchoTimer::set_publish_native(uint64_t n)
{
    d_publish_native_.store(n, std::memory_order_relaxed);
}

uint64_t UwbRealtimeEchoTimer::publish_native() const
{
    return d_publish_native_.load(std::memory_order_relaxed);
}

uint64_t UwbRealtimeEchoTimer::published_samples_total() const
{
    return d_published_samples_.load(std::memory_order_relaxed);
}

int64_t UwbRealtimeEchoTimer::device_time_ticks() const
{
    // Backend clock; the caller reads this BEFORE arming a schedule to
    // compute the first future t0.  Returns 0 when the backend is not
    // prepared (device unavailable), never throws.
    return d_backend_ ? d_backend_->device_time_ticks() : 0;
}

uint64_t UwbRealtimeEchoTimer::last_worker_us() const
{
    return d_worker_us_last_.load(std::memory_order_relaxed);
}

uint64_t UwbRealtimeEchoTimer::max_worker_us() const
{
    return d_worker_us_max_.load(std::memory_order_relaxed);
}

uint64_t UwbRealtimeEchoTimer::mean_worker_us() const
{
    const uint64_t n = d_bursts_published_.load(std::memory_order_relaxed);
    if (n == 0)
        return 0;
    return d_worker_us_total_.load(std::memory_order_relaxed) / n;
}

// --- per-burst segment timing (D; diagnostics only) ---

uint64_t UwbRealtimeEchoTimer::slot_lead_us_last() const
{
    return d_slot_lead_us_last_.load(std::memory_order_relaxed);
}

uint64_t UwbRealtimeEchoTimer::issue_rx_us_mean() const
{
    const uint64_t n = d_bursts_published_.load(std::memory_order_relaxed);
    if (n == 0)
        return 0;
    return d_issue_rx_us_total_.load(std::memory_order_relaxed) / n;
}

uint64_t UwbRealtimeEchoTimer::tx_send_us_mean() const
{
    const uint64_t n = d_bursts_published_.load(std::memory_order_relaxed);
    if (n == 0)
        return 0;
    return d_tx_send_us_total_.load(std::memory_order_relaxed) / n;
}

uint64_t UwbRealtimeEchoTimer::rx_collect_us_mean() const
{
    const uint64_t n = d_bursts_published_.load(std::memory_order_relaxed);
    if (n == 0)
        return 0;
    return d_rx_collect_us_total_.load(std::memory_order_relaxed) / n;
}

uint64_t UwbRealtimeEchoTimer::pdu_build_us_mean() const
{
    const uint64_t n = d_bursts_published_.load(std::memory_order_relaxed);
    if (n == 0)
        return 0;
    return d_pdu_build_us_total_.load(std::memory_order_relaxed) / n;
}

uint64_t UwbRealtimeEchoTimer::pdu_publish_us_mean() const
{
    const uint64_t n = d_bursts_published_.load(std::memory_order_relaxed);
    if (n == 0)
        return 0;
    return d_pdu_publish_us_total_.load(std::memory_order_relaxed) / n;
}

uint64_t UwbRealtimeEchoTimer::worker_total_us_mean() const
{
    // Alias: the whole-burst wall time already includes both publish
    // segments plus all backend I/O.
    return mean_worker_us();
}

// --- multi-TX introspection (M3/M4/M5) ---

size_t UwbRealtimeEchoTimer::tx_channel_count() const
{
    return d_atomic_tx_channels_.load(std::memory_order_relaxed);
}

size_t UwbRealtimeEchoTimer::zero_scratch_capacity() const
{
    return d_zero_scratch_.size();
}

const int16_t* UwbRealtimeEchoTimer::zero_scratch_data() const
{
    return d_zero_scratch_.data();
}

const echo::MultiTxWindowBank* UwbRealtimeEchoTimer::armed_window_bank() const
{
    return d_mtx_bank_.get();
}

size_t UwbRealtimeEchoTimer::jam_logical_channel() const
{
    return d_atomic_jam_ch_.load(std::memory_order_relaxed);
}

int64_t UwbRealtimeEchoTimer::jam_delay_native_last() const
{
    return d_jam_delay_last_.load(std::memory_order_relaxed);
}

double UwbRealtimeEchoTimer::jam_delay_us_last() const
{
    return d_jam_delay_us_last_.load(std::memory_order_relaxed);
}

double UwbRealtimeEchoTimer::jam_freq_plan_hz_last() const
{
    return d_jam_plan_last_.load(std::memory_order_relaxed);
}

double UwbRealtimeEchoTimer::jam_freq_actual_hz_last() const
{
    return d_jam_actual_last_.load(std::memory_order_relaxed);
}

double UwbRealtimeEchoTimer::jam_freq_offset_hz_last() const
{
    return d_jam_offset_last_.load(std::memory_order_relaxed);
}

uint64_t UwbRealtimeEchoTimer::jam_retunes() const
{
    return d_jam_retunes_.load(std::memory_order_relaxed);
}

uint64_t UwbRealtimeEchoTimer::jam_retune_failures() const
{
    return d_jam_retune_fails_.load(std::memory_order_relaxed);
}

pmt::pmt_t UwbRealtimeEchoTimer::tx_async_counts() const
{
    const echo::TxAsyncCounts c =
        d_backend_ ? d_backend_->tx_async_counts() : echo::TxAsyncCounts{};
    pmt::pmt_t d = pmt::make_dict();
    d = pmt::dict_add(d, pmt::mp("underflow"),
                      pmt::from_uint64(c.underflow));
    d = pmt::dict_add(d, pmt::mp("seq_error"),
                      pmt::from_uint64(c.seq_error));
    d = pmt::dict_add(d, pmt::mp("time_error"),
                      pmt::from_uint64(c.time_error));
    d = pmt::dict_add(d, pmt::mp("unmatched"),
                      pmt::from_uint64(c.unmatched));
    d = pmt::dict_add(d, pmt::mp("dropped"), pmt::from_uint64(c.dropped));
    d = pmt::dict_add(d, pmt::mp("ack"), pmt::from_uint64(c.ack));
    return d;
}

std::string UwbRealtimeEchoTimer::last_error() const
{
    std::lock_guard<std::mutex> lock(d_err_mutex_);
    return d_last_error_;
}

bool UwbRealtimeEchoTimer::drained() const
{
    std::lock_guard<std::mutex> lock(d_queue_mutex_);
    return d_queue_.empty() &&
           !d_worker_busy_.load(std::memory_order_relaxed);
}

void UwbRealtimeEchoTimer::drain()
{
    std::unique_lock<std::mutex> lock(d_queue_mutex_);
    d_queue_cv_.wait(lock, [this] {
        return d_queue_.empty() &&
               !d_worker_busy_.load(std::memory_order_relaxed);
    });
}

// ---------------------------------------------------------------------------
// start / stop
// ---------------------------------------------------------------------------

void
UwbRealtimeEchoTimer::stop_worker_and_join()
{
    {
        std::lock_guard<std::mutex> lock(d_queue_mutex_);
        d_stop_.store(true, std::memory_order_relaxed);
    }
    d_queue_cv_.notify_all();
    // Abort in-flight backend I/O so a worker blocked in collect_result()
    // (stop-during-I/O injection) wakes up and the join terminates.
    if (d_backend_)
        d_backend_->request_stop();
    if (d_worker_.joinable())
        d_worker_.join();
}

bool
UwbRealtimeEchoTimer::start()
{
    // Idempotent: join any leftover worker first (never under the queue
    // mutex — a worker blocked on it would deadlock the join).
    stop_worker_and_join();

    std::string err;
    if (!d_backend_->prepare(err)) {
        pmt::pmt_t extra = pmt::make_dict();
        extra = pmt::dict_add(extra, pmt::mp("error"),
                              pmt::string_to_symbol(err));
        publish_status("backend_prepare_failed", extra);
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(d_queue_mutex_);
        d_queue_.clear();
        d_queue_depth_.store(0, std::memory_order_relaxed);
        d_stop_.store(false, std::memory_order_relaxed);
        d_armed_.store(false, std::memory_order_relaxed);
    }
    // Writer restart semantics: reset all counters and re-attach a
    // disarmed grid.
    d_received_.store(0, std::memory_order_relaxed);
    d_enqueued_.store(0, std::memory_order_relaxed);
    d_dropped_.store(0, std::memory_order_relaxed);
    d_invalid_.store(0, std::memory_order_relaxed);
    d_dropped_on_stop_.store(0, std::memory_order_relaxed);
    d_bursts_published_.store(0, std::memory_order_relaxed);
    d_bursts_ok_.store(0, std::memory_order_relaxed);
    d_bursts_failed_.store(0, std::memory_order_relaxed);
    d_late_slot_skips_.store(0, std::memory_order_relaxed);
    d_tx_reissues_.store(0, std::memory_order_relaxed);
    d_rx_reissues_.store(0, std::memory_order_relaxed);
    d_grid_errors_.store(0, std::memory_order_relaxed);
    d_queue_high_watermark_.store(0, std::memory_order_relaxed);
    d_published_samples_.store(0, std::memory_order_relaxed);
    d_worker_us_total_.store(0, std::memory_order_relaxed);
    d_worker_us_max_.store(0, std::memory_order_relaxed);
    d_worker_us_last_.store(0, std::memory_order_relaxed);
    d_slot_lead_us_last_.store(0, std::memory_order_relaxed);
    d_slot_lead_us_total_.store(0, std::memory_order_relaxed);
    d_slot_lead_us_max_.store(0, std::memory_order_relaxed);
    d_issue_rx_us_total_.store(0, std::memory_order_relaxed);
    d_issue_rx_us_max_.store(0, std::memory_order_relaxed);
    d_tx_send_us_total_.store(0, std::memory_order_relaxed);
    d_tx_send_us_max_.store(0, std::memory_order_relaxed);
    d_rx_collect_us_total_.store(0, std::memory_order_relaxed);
    d_rx_collect_us_max_.store(0, std::memory_order_relaxed);
    d_pdu_build_us_total_.store(0, std::memory_order_relaxed);
    d_pdu_build_us_max_.store(0, std::memory_order_relaxed);
    d_pdu_publish_us_total_.store(0, std::memory_order_relaxed);
    d_pdu_publish_us_max_.store(0, std::memory_order_relaxed);
    // Tuning restarts disarmed (cal delay is retained across start()).
    d_freq_hz_.store(0.0, std::memory_order_relaxed);
    d_tune_pending_.store(false, std::memory_order_relaxed);
    d_tune_hz_.store(0.0, std::memory_order_relaxed);
    d_grid_ = echo::EchoGrid(d_sched_);
    d_tx_samples_ = d_rx_samples_ = 0;
    d_tx_ptr_ = nullptr;
    d_tx_payload_ = pmt::PMT_NIL;
    d_sched_meta_ = pmt::PMT_NIL;
    d_burst_publish_native_ = 0;
    d_pulse_id_increment_ = 0;
    d_sched_index_start_ = 0;
    // Multi-TX grids restart disarmed; scratch/plan capacities are retained
    // (clear() never shrinks), so re-arming still never allocates.
    d_mtx_armed_ = false;
    d_mtx_N_ = 1;
    d_mtx_geom_ = echo::MultiTxGeometry{};
    d_mtx_wave_len_.fill(0);
    d_mtx_base_.fill(0);
    d_mtx_jam_ch_ = 1;
    d_mtx_delay_mode_ = echo::JamDelayMode::Fixed;
    d_mtx_delay_lo_ = d_mtx_delay_hi_ = 0;
    d_mtx_payloads_.clear();
    d_mtx_bank_.reset();
    d_mtx_delay_seed_ = 0;
    for (size_t i = 0; i < echo::kEchoMaxTxChannels; ++i)
        d_mtx_wave_ptrs_[i] = nullptr;
    d_mtx_freqs_.clear();
    d_mtx_dwell_ = 0;
    d_mtx_settle_ticks_ = 0;
    d_mtx_base_hz_ = 0.0;
    d_mtx_step_ = std::numeric_limits<size_t>::max();
    d_mtx_ok_in_step_ = 0;
    d_mtx_advance_pending_ = false;
    d_mtx_jam_offset_hz_ = 0.0;
    d_mtx_jam_actual_hz_ = 0.0;
    d_atomic_tx_channels_.store(1, std::memory_order_relaxed);
    d_atomic_jam_ch_.store(1, std::memory_order_relaxed);
    d_jam_delay_last_.store(0, std::memory_order_relaxed);
    d_jam_delay_us_last_.store(0.0, std::memory_order_relaxed);
    d_jam_plan_last_.store(0.0, std::memory_order_relaxed);
    d_jam_actual_last_.store(0.0, std::memory_order_relaxed);
    d_jam_offset_last_.store(0.0, std::memory_order_relaxed);
    d_jam_retunes_.store(0, std::memory_order_relaxed);
    d_jam_retune_fails_.store(0, std::memory_order_relaxed);
    {
        std::lock_guard<std::mutex> lock(d_err_mutex_);
        d_last_error_.clear();
    }

    d_worker_ = std::thread(&UwbRealtimeEchoTimer::worker_loop, this);
    publish_status("started");
    return true;
}

bool
UwbRealtimeEchoTimer::stop()
{
    stop_worker_and_join();
    publish_status("stopped");
    return true;
}

// ---------------------------------------------------------------------------
// Message handler (validate + bounded enqueue only)
// ---------------------------------------------------------------------------

void
UwbRealtimeEchoTimer::handle_schedule(pmt::pmt_t msg)
{
    // Diagnosable rejection: the event stays "invalid_schedule" (as before)
    // while the reason symbol names the failed check.
    auto reject = [&](const std::string& reason) {
        d_invalid_.fetch_add(1, std::memory_order_relaxed);
        pmt::pmt_t extra = pmt::make_dict();
        extra = pmt::dict_add(extra, pmt::mp("reason"), pmt::mp(reason));
        publish_status("invalid_schedule", extra);
    };

    if (!pmt::is_pair(msg)) {
        reject("not_a_pair");
        return;
    }
    const pmt::pmt_t meta = pmt::car(msg);
    const pmt::pmt_t payload = pmt::cdr(msg);
    if (!pmt::is_dict(meta)) {
        reject("meta_not_a_dict");
        return;
    }
    // Legacy single-TX cdr: one s16vector.  Multi-TX cdr (§5.3): a PMT
    // vector/tuple of 1..kEchoMaxTxChannels s16vectors (one effective
    // waveform per channel).
    const bool is_single = pmt::is_s16vector(payload);
    const bool is_chan_list =
        !is_single && is_pmt_list(payload) &&
        pmt_list_len(payload) >= 1 &&
        pmt_list_len(payload) <= echo::kEchoMaxTxChannels;
    if (!is_single && !is_chan_list) {
        reject("payload_not_s16vector_or_channel_vector");
        return;
    }
    if (!dict_has(meta, "t0_ticks") || !dict_has(meta, "tx_samples") ||
        !dict_has(meta, "rx_samples")) {
        reject("missing_required_key");
        return;
    }
    // t0_ticks: must be a non-negative int64-representable device tick
    // (explicitly reject huge uint64 PMT values that would wrap around).
    const pmt::pmt_t t0_val =
        pmt::dict_ref(meta, pmt::mp("t0_ticks"), pmt::PMT_NIL);
    const int64_t t0 = dict_i64(meta, "t0_ticks", -1);
    if (t0 < 0 || !pmt::is_integer(t0_val) ||
        (pmt::is_uint64(t0_val) &&
         pmt::to_uint64(t0_val) >
             static_cast<uint64_t>(std::numeric_limits<int64_t>::max()))) {
        reject("bad_t0_ticks");
        return;
    }
    // Burst lengths: strictly positive, within the fixed make() caps, and
    // their SC16 element counts (2 * samples) must fit size_t without
    // overflow.  The caps bound every allocation in the block's lifetime;
    // nothing on the worker is allowed to grow.
    const uint64_t tx_samples = dict_u64(meta, "tx_samples", 0);
    const uint64_t rx_samples = dict_u64(meta, "rx_samples", 0);
    if (tx_samples == 0 || rx_samples == 0 ||
        tx_samples > d_max_tx_samples_ || rx_samples > d_max_rx_samples_ ||
        tx_samples >
            static_cast<uint64_t>(std::numeric_limits<size_t>::max() / 2) ||
        rx_samples >
            static_cast<uint64_t>(std::numeric_limits<size_t>::max() / 2)) {
        reject("samples_over_cap");
        return;
    }
    // Optional per-schedule fragment-size override; 0 (absent) = block
    // default from the prepared grid config.  Present-but-zero is invalid.
    const uint64_t max_frag_in = dict_u64(meta, "max_fragment_size", 0);
    if (dict_has(meta, "max_fragment_size") && max_frag_in == 0) {
        reject("bad_max_fragment_size");
        return;
    }
    const uint64_t frag =
        max_frag_in != 0 ? max_frag_in : d_sched_.max_fragment_size;

    Job job;
    job.t0_ticks = t0;
    job.tx_samples = tx_samples;
    job.rx_samples = rx_samples;
    job.schedule_index = dict_u64(meta, "schedule_index", 0);
    job.pulse_id = dict_u64(meta, "pulse_id", job.schedule_index);
    job.burst_count = dict_u64(meta, "burst_count", 0);
    job.max_fragment_size = max_frag_in;
    // Optional per-burst publish ROI (M2): 0/absent = use the block-level
    // set_publish_native(); an explicit value overrides it for this burst.
    job.publish_native = dict_u64(meta, "publish_native", 0);
    job.pulse_id_increment = dict_u64(meta, "pulse_id_increment", 0);
    job.sample_rate = dict_f64(meta, "sample_rate", 0.0);
    job.meta = meta;

    // Strict u64/int metadata read for multi-TX keys (present key with a
    // non-integer value is a rejection, never a silent default).
    auto strict_u64 = [&](const char* key, uint64_t& out) {
        if (!dict_has(meta, key))
            return true; // absent: caller keeps its default
        const pmt::pmt_t v =
            pmt::dict_ref(meta, pmt::mp(key), pmt::PMT_NIL);
        if (pmt::is_uint64(v)) {
            out = pmt::to_uint64(v);
            return true;
        }
        if (pmt::is_integer(v)) {
            const long l = pmt::to_long(v);
            if (l < 0)
                return false;
            out = static_cast<uint64_t>(l);
            return true;
        }
        return false;
    };

    if (is_single) {
        size_t payload_len = 0;
        pmt::s16vector_elements(payload, payload_len);
        // The payload holds tx_samples SC16 pairs = 2 * tx_samples int16;
        // tx_samples <= max_tx_samples <= SIZE_MAX/2 keeps this multiply safe.
        const size_t expect_elems = static_cast<size_t>(tx_samples) * 2;
        if (payload_len != expect_elems) {
            reject("payload_length_mismatch");
            return;
        }
        // A lone s16vector carries exactly one channel.
        uint64_t nch = 1;
        if (!strict_u64("tx_channel_count", nch) || nch != 1) {
            reject("tx_channel_count_mismatch");
            return;
        }
        job.tx_channel_count = 1;
        job.payload = payload;
    } else {
        // --- multi-TX channel payloads ---
        const size_t nch = pmt_list_len(payload);
        uint64_t pairs[echo::kEchoMaxTxChannels] = {};
        std::vector<pmt::pmt_t> elems;
        elems.reserve(nch);
        for (size_t i = 0; i < nch; ++i) {
            const pmt::pmt_t e = pmt_list_ref(payload, i);
            if (!pmt::is_s16vector(e)) {
                reject("channel_payload_not_s16vector");
                return;
            }
            size_t elen = 0;
            pmt::s16vector_elements(e, elen);
            if (elen % 2 != 0) {
                reject("channel_payload_odd_sc16");
                return;
            }
            pairs[i] = static_cast<uint64_t>(elen / 2);
            elems.push_back(e);
        }
        uint64_t nch_meta = nch;
        if (!strict_u64("tx_channel_count", nch_meta) ||
            nch_meta != nch) {
            reject("tx_channel_count_mismatch");
            return;
        }
        if (nch == 1) {
            // Single-element vector ≡ legacy single-TX (bit-exact path).
            const size_t expect_elems = static_cast<size_t>(tx_samples) * 2;
            size_t elen = 0;
            pmt::s16vector_elements(elems[0], elen);
            if (elen != expect_elems) {
                reject("payload_length_mismatch");
                return;
            }
            job.tx_channel_count = 1;
            job.payload = elems[0];
        } else {
            // --- full multi-TX validation (M3/M4/M5) ---
            uint64_t wave[echo::kEchoMaxTxChannels] = {};
            uint64_t base[echo::kEchoMaxTxChannels] = {};
            for (size_t i = 0; i < nch; ++i)
                wave[i] = pairs[i];
            if (dict_has(meta, "tx_waveform_samples")) {
                uint64_t tmp[echo::kEchoMaxTxChannels] = {};
                std::string err;
                if (!read_u64_list(pmt::dict_ref(meta,
                                                pmt::mp("tx_waveform_samples"),
                                                pmt::PMT_NIL),
                                  nch, tmp, err)) {
                    reject("bad_tx_waveform_samples");
                    return;
                }
                for (size_t i = 0; i < nch; ++i) {
                    if (tmp[i] != pairs[i]) {
                        reject("waveform_length_payload_mismatch");
                        return;
                    }
                    wave[i] = tmp[i];
                }
            }
            if (dict_has(meta, "tx_base_offsets_native")) {
                std::string err;
                if (!read_u64_list(pmt::dict_ref(meta,
                                                pmt::mp("tx_base_offsets_"
                                                        "native"),
                                                pmt::PMT_NIL),
                                  nch, base, err)) {
                    reject("bad_tx_base_offsets_native");
                    return;
                }
            }
            // Upper bounds: every channel footprint must fit inside L.
            for (size_t i = 0; i < nch; ++i) {
                if (base[i] > tx_samples ||
                    wave[i] > tx_samples - base[i]) {
                    reject("channel_geometry_out_of_bounds");
                    return;
                }
            }
            if (wave[0] == 0) {
                reject("sense_waveform_empty");
                return;
            }
            size_t jam_ch = 1;
            if (dict_has(meta, "jam_logical_channel")) {
                const pmt::pmt_t jv = pmt::dict_ref(
                    meta, pmt::mp("jam_logical_channel"), pmt::PMT_NIL);
                uint64_t j = 0;
                if (pmt::is_uint64(jv)) {
                    j = pmt::to_uint64(jv);
                } else if (pmt::is_integer(jv)) {
                    const long l = pmt::to_long(jv);
                    if (l < 0) {
                        reject("bad_jam_logical_channel");
                        return;
                    }
                    j = static_cast<uint64_t>(l);
                } else {
                    reject("bad_jam_logical_channel");
                    return;
                }
                if (j < 1 || j >= nch) {
                    reject("jam_logical_channel_out_of_range");
                    return;
                }
                jam_ch = static_cast<size_t>(j);
            } else if (nch <= 1) {
                reject("jam_logical_channel_out_of_range");
                return;
            }
            echo::JamDelayMode mode = echo::JamDelayMode::Fixed;
            {
                std::string err;
                if (!parse_jam_delay_mode(meta, mode, err)) {
                    reject(err);
                    return;
                }
            }
            int64_t lo = 0, hi = 0;
            if (dict_has(meta, "jam_delay_lo_native") &&
                !dict_i64_strict(meta, "jam_delay_lo_native", lo)) {
                reject("bad_jam_delay_lo_native");
                return;
            }
            if (dict_has(meta, "jam_delay_hi_native") &&
                !dict_i64_strict(meta, "jam_delay_hi_native", hi)) {
                reject("bad_jam_delay_hi_native");
                return;
            }
            uint64_t seed = 0;
            if (!strict_u64("jam_delay_seed", seed)) {
                reject("bad_jam_delay_seed");
                return;
            }
            // D = sense base offset; tx_samples <= SIZE_MAX/2 <= INT64_MAX
            // on 64-bit, so the cast below cannot wrap.
            const int64_t D = static_cast<int64_t>(base[0]);
            if (lo > hi) {
                reject("jam_delay_lo_gt_hi");
                return;
            }
            if (mode == echo::JamDelayMode::Fixed) {
                if (lo != hi) {
                    reject("fixed_delay_requires_lo_eq_hi");
                    return;
                }
                // Nominal jammer footprint must equal D + lo ...
                if (base[jam_ch] != static_cast<uint64_t>(D + lo)) {
                    reject("fixed_jam_base_mismatch");
                    return;
                }
                // ... and L must exactly cover every channel footprint.
                uint64_t need = 0;
                for (size_t i = 0; i < nch; ++i)
                    need = std::max(need, base[i] + wave[i]);
                if (need != tx_samples) {
                    reject("fixed_tx_samples_mismatch");
                    return;
                }
            } else {
                // Uniform span (§5.4): per-pulse jammer begin = D + delay
                // with delay in [-D, +D]; L is fixed by the geometry.
                if (lo < -D || hi < -D || lo > D || hi > D) {
                    reject("jam_delay_out_of_span");
                    return;
                }
                echo::MultiTxGeometry geom;
                std::string gerr;
                if (!echo::prepare_multitx_geometry(
                        wave[0], wave[jam_ch], base[0], geom, &gerr) ||
                    geom.phys_len_L != tx_samples) {
                    reject("multitx_geometry_L_mismatch");
                    return;
                }
            }
            // Dwell scan plan (empty/absent = no retune, mirroring the
            // Python jammer: dwell <= 0 disables retunes).
            std::vector<double> freqs;
            if (dict_has(meta, "jam_freq_offsets_hz")) {
                const pmt::pmt_t fv = pmt::dict_ref(
                    meta, pmt::mp("jam_freq_offsets_hz"), pmt::PMT_NIL);
                auto push_finite = [&](double v) {
                    if (!std::isfinite(v)) {
                        reject("bad_jam_freq_offsets_hz");
                        return false;
                    }
                    freqs.push_back(v);
                    return true;
                };
                bool ok = true;
                if (pmt::is_f64vector(fv)) {
                    size_t flen = 0;
                    const double* fel = pmt::f64vector_elements(fv, flen);
                    if (flen > 65536) {
                        reject("jam_freq_plan_too_long");
                        return;
                    }
                    for (size_t i = 0; i < flen && ok; ++i)
                        ok = push_finite(fel[i]);
                } else if (pmt::is_f32vector(fv)) {
                    size_t flen = 0;
                    const float* fel = pmt::f32vector_elements(fv, flen);
                    if (flen > 65536) {
                        reject("jam_freq_plan_too_long");
                        return;
                    }
                    for (size_t i = 0; i < flen && ok; ++i)
                        ok = push_finite(static_cast<double>(fel[i]));
                } else if (is_pmt_list(fv)) {
                    const size_t flen = pmt_list_len(fv);
                    if (flen > 65536) {
                        reject("jam_freq_plan_too_long");
                        return;
                    }
                    for (size_t i = 0; i < flen && ok; ++i) {
                        const pmt::pmt_t e = pmt_list_ref(fv, i);
                        if (pmt::is_real(e))
                            ok = push_finite(pmt::to_double(e));
                        else if (pmt::is_uint64(e))
                            ok = push_finite(
                                static_cast<double>(pmt::to_uint64(e)));
                        else if (pmt::is_integer(e))
                            ok = push_finite(
                                static_cast<double>(pmt::to_long(e)));
                        else {
                            reject("bad_jam_freq_offsets_hz");
                            return;
                        }
                    }
                } else {
                    reject("bad_jam_freq_offsets_hz");
                    return;
                }
                if (!ok)
                    return; // rejection already published above
            }
            uint64_t dwell = 0, settle = 0;
            if (!strict_u64("jam_dwell", dwell)) {
                reject("bad_jam_dwell");
                return;
            }
            if (!strict_u64("jam_freq_settle_ticks", settle)) {
                reject("bad_jam_freq_settle_ticks");
                return;
            }
            if (!freqs.empty() && dwell > 0) {
                if (freqs.size() >
                    std::numeric_limits<uint64_t>::max() / dwell) {
                    reject("jam_scan_sample_target_overflow");
                    return;
                }
                const uint64_t target =
                    static_cast<uint64_t>(freqs.size()) * dwell;
                if (job.burst_count != target) {
                    reject("jam_scan_burst_count_mismatch");
                    return;
                }
            }
            // Contiguous-window contract: one data fragment of length L.
            // Application-layer splits at sensing/jammer edges are the
            // delay-dependent send path this remediation removes.
            if (frag < tx_samples) {
                reject("max_fragment_size_lt_L");
                return;
            }
            job.tx_channel_count = nch;
            job.tx_payloads = std::move(elems);
            for (size_t i = 0; i < nch; ++i) {
                job.tx_waveform_samples[i] = wave[i];
                job.tx_base_offsets[i] = base[i];
            }
            job.jam_logical_channel = jam_ch;
            job.jam_delay_mode = mode;
            job.jam_delay_lo_native = lo;
            job.jam_delay_hi_native = hi;
            job.jam_delay_seed = seed;
            job.jam_freq_offsets_hz = std::move(freqs);
            job.jam_dwell = dwell;
            job.jam_freq_settle_ticks = settle;
            {
                const int16_t* waves[echo::kEchoMaxTxChannels] = {};
                for (size_t i = 0; i < nch; ++i) {
                    size_t elen = 0;
                    waves[i] = pmt::s16vector_elements(job.tx_payloads[i],
                                                       elen);
                }
                auto bank = std::make_shared<echo::MultiTxWindowBank>();
                std::string berr;
                try {
                    if (!echo::materialize_multitx_window_bank(
                            waves, wave, base, nch, jam_ch, tx_samples,
                            mode, d_max_tx_samples_, *bank, &berr)) {
                        reject(berr.empty() ? "window_bank_materialize"
                                            : berr);
                        return;
                    }
                } catch (const std::bad_alloc&) {
                    reject("window_bank_alloc_failed");
                    return;
                }
                job.window_bank = std::move(bank);
            }
            // Contract/safety gate (§5.5): a jam plan with a NONZERO dwell
            // offset is an ABSOLUTE tune (base + offset).  Without an
            // absolute base — schedule "freq_hz" absent AND no applied
            // set_freq — the worker would tune the jammer to the bare
            // offset; measured 2026-09-19 that request (491339 Hz) was
            // coerced by the device to 1 MHz, silently moving the jammer
            // out of band.  Reject here, before enqueue.
            bool needs_base = false;
            for (double f : job.jam_freq_offsets_hz) {
                if (std::isfinite(f) && std::fabs(f) > 0.0) {
                    needs_base = true;
                    break;
                }
            }
            if (needs_base && !radar_meta::dict_has(meta, "freq_hz") &&
                !(d_freq_hz_.load(std::memory_order_relaxed) > 0.0)) {
                reject("jam_scan_base_unknown");
                return;
            }
        }
    }

    d_received_.fetch_add(1, std::memory_order_relaxed);
    if (!enqueue(std::move(job))) {
        d_dropped_.fetch_add(1, std::memory_order_relaxed);
        pmt::pmt_t extra = pmt::make_dict();
        extra = pmt::dict_add(extra, pmt::mp("queue_depth"),
                              pmt::from_uint64(static_cast<uint64_t>(
                                  d_queue_depth_.load(
                                      std::memory_order_relaxed))));
        publish_status("queue_full", extra);
        return;
    }
    d_enqueued_.fetch_add(1, std::memory_order_relaxed);
    d_queue_cv_.notify_one();
}

bool
UwbRealtimeEchoTimer::enqueue(Job&& job)
{
    std::lock_guard<std::mutex> lock(d_queue_mutex_);
    if (d_queue_.size() >= d_queue_capacity_)
        return false;
    d_queue_.push_back(std::move(job));
    const size_t depth = d_queue_.size();
    d_queue_depth_.store(depth, std::memory_order_relaxed);
    size_t hw = d_queue_high_watermark_.load(std::memory_order_relaxed);
    while (depth > hw && !d_queue_high_watermark_.compare_exchange_weak(
                             hw, depth, std::memory_order_relaxed)) {
        // retry with updated hw
    }
    return true;
}

// ---------------------------------------------------------------------------
// Worker
// ---------------------------------------------------------------------------

void
UwbRealtimeEchoTimer::worker_loop()
{
    std::unique_lock<std::mutex> lock(d_queue_mutex_);
    for (;;) {
        d_worker_busy_.store(false, std::memory_order_relaxed);
        d_queue_cv_.notify_all(); // wake drain() waiters

        if (d_stop_.load(std::memory_order_relaxed)) {
            // Drain-on-stop contract: complete the current burst (already
            // published by run_one_burst), drop remaining queued schedule
            // jobs and report them — never silently exit.
            const size_t dropped = d_queue_.size();
            d_queue_.clear();
            d_queue_depth_.store(0, std::memory_order_relaxed);
            d_dropped_on_stop_.fetch_add(dropped,
                                         std::memory_order_relaxed);
            lock.unlock();
            if (dropped > 0) {
                pmt::pmt_t extra = pmt::make_dict();
                extra = pmt::dict_add(extra, pmt::mp("dropped"),
                                      pmt::from_uint64(dropped));
                publish_status("schedule_dropped_on_stop", extra);
            }
            return;
        }

        if (!d_queue_.empty()) {
            Job job = std::move(d_queue_.front());
            d_queue_.pop_front();
            d_queue_depth_.store(d_queue_.size(),
                                 std::memory_order_relaxed);
            d_worker_busy_.store(true, std::memory_order_relaxed);
            lock.unlock();
            apply_schedule(job);
            lock.lock();
            continue;
        }

        if (d_armed_.load(std::memory_order_relaxed)) {
            d_worker_busy_.store(true, std::memory_order_relaxed);
            lock.unlock();
            run_one_burst();
            lock.lock();
            continue;
        }

        d_queue_cv_.wait(lock, [this] {
            return d_stop_.load(std::memory_order_relaxed) ||
                   !d_queue_.empty() ||
                   d_armed_.load(std::memory_order_relaxed);
        });
    }
}

void
UwbRealtimeEchoTimer::apply_schedule(const Job& job)
{
    // Worker thread, once per armed schedule PDU.  No allocation here:
    // the RX scratch was preallocated once at construction for
    // max_rx_samples; arming only zero-fills the used prefix.  The
    // handler already enforced the caps; this is a defensive re-check so
    // the worker can never index past the fixed scratch.
    if (job.tx_samples > d_max_tx_samples_ ||
        job.rx_samples > d_max_rx_samples_ ||
        job.rx_samples > d_rx_buf_.size() / 2) {
        d_invalid_.fetch_add(1, std::memory_order_relaxed);
        pmt::pmt_t extra = pmt::make_dict();
        extra = pmt::dict_add(extra, pmt::mp("pulse_id"),
                              pmt::from_uint64(job.pulse_id));
        extra = pmt::dict_add(extra, pmt::mp("reason"),
                              pmt::mp("samples_over_fixed_cap"));
        publish_status("invalid_schedule", extra);
        return;
    }
    if (!d_grid_.arm(job.t0_ticks, job.schedule_index)) {
        d_invalid_.fetch_add(1, std::memory_order_relaxed);
        pmt::pmt_t extra = pmt::make_dict();
        extra = pmt::dict_add(extra, pmt::mp("pulse_id"),
                              pmt::from_uint64(job.pulse_id));
        extra = pmt::dict_add(extra, pmt::mp("reason"),
                              pmt::mp("t0_before_pre_guard"));
        publish_status("invalid_schedule", extra);
        return;
    }
    d_tx_samples_ = job.tx_samples;
    d_rx_samples_ = job.rx_samples;
    // 0 = use the prepared block default (per-schedule override only).
    d_max_frag_ = job.max_fragment_size != 0 ? job.max_fragment_size
                                             : d_sched_.max_fragment_size;
    d_burst_budget_ = job.burst_count == 0
                          ? std::numeric_limits<uint64_t>::max()
                          : job.burst_count;
    d_bursts_this_grid_ = 0;
    d_pulse_id_ = job.pulse_id;
    d_sample_rate_ = job.sample_rate;
    d_tx_payload_ = job.payload;
    d_sched_meta_ = job.meta;
    // Per-burst ROI override (0 = block-level set_publish_native()).
    d_burst_publish_native_ = job.publish_native;
    // Per-burst pulse_id lineage (single multi-burst schedule → unique ids).
    d_pulse_id_increment_ = job.pulse_id_increment;
    d_sched_index_start_ = job.schedule_index;
    // The schedule PDU carries the calibration-delay default; a later
    // set_cal_delay_native() overrides it and is never touched by the I/O
    // path.  Only seed when the schedule actually provides the key so a
    // restart preserves a runtime value.
    if (radar_meta::dict_has(job.meta,
                             "calibration_delay_native_samples")) {
        const double v = radar_meta::to_f64(
            pmt::dict_ref(job.meta,
                          pmt::mp("calibration_delay_native_samples"),
                          pmt::from_double(0.0)),
            0.0);
        if (std::isfinite(v))
            d_cal_delay_native_.store(v, std::memory_order_relaxed);
    }
    size_t len = 0;
    if (job.tx_channel_count > 1) {
        // Multi-TX grids resolve per-channel pointers from the frozen
        // d_mtx_wave_ptrs_ (apply below); the legacy single pointer stays
        // null and the hot path never touches it.
        d_tx_ptr_ = nullptr;
    } else {
        d_tx_ptr_ = pmt::s16vector_elements(job.payload, len);
    }
    // Fixed scratch: zero the used prefix only; capacity/address stable.
    std::fill_n(d_rx_buf_.begin(),
                static_cast<size_t>(d_rx_samples_) * 2,
                static_cast<int16_t>(0));
    // --- multi-TX freeze (M3/M4/M5; once per armed schedule, never per
    // burst).  Capacities were reserved at construction, so the assigns
    // below never allocate; the per-burst path only reads this state.
    d_mtx_armed_ = job.tx_channel_count > 1;
    d_mtx_N_ = d_mtx_armed_ ? job.tx_channel_count : 1;
    d_atomic_tx_channels_.store(d_mtx_N_, std::memory_order_relaxed);
    if (d_mtx_armed_) {
        const size_t nch = job.tx_channel_count;
        d_mtx_wave_len_ = job.tx_waveform_samples;
        d_mtx_base_ = job.tx_base_offsets;
        d_mtx_jam_ch_ = job.jam_logical_channel;
        d_mtx_delay_mode_ = job.jam_delay_mode;
        d_mtx_delay_lo_ = job.jam_delay_lo_native;
        d_mtx_delay_hi_ = job.jam_delay_hi_native;
        d_mtx_payloads_.clear();
        for (size_t i = 0; i < nch; ++i) {
            d_mtx_payloads_.push_back(job.tx_payloads[i]);
            size_t elen = 0;
            const int16_t* el =
                pmt::s16vector_elements(job.tx_payloads[i], elen);
            // Empty (silent) channels resolve to nullptr; the planner maps
            // every such fragment slice to the zero scratch.
            d_mtx_wave_ptrs_[i] =
                (elen == 0) ? nullptr : el;
        }
        for (size_t i = nch; i < echo::kEchoMaxTxChannels; ++i)
            d_mtx_wave_ptrs_[i] = nullptr;
        // Frozen geometry: L was validated against tx_samples in the
        // handler; re-derive defensively (uniform) so the worker owns one
        // ground truth even if the handler is ever bypassed.
        d_mtx_geom_ = echo::MultiTxGeometry{};
        {
            std::string gerr;
            if (job.jam_delay_mode == echo::JamDelayMode::Uniform) {
                echo::prepare_multitx_geometry(
                    d_mtx_wave_len_[0], d_mtx_wave_len_[d_mtx_jam_ch_],
                    d_mtx_base_[0], d_mtx_geom_, &gerr);
            } else {
                // Fixed mode: physical length is the frozen tx_samples;
                // sense anchor D is the channel-0 base offset.
                d_mtx_geom_.sense_len = d_mtx_wave_len_[0];
                d_mtx_geom_.jam_len = d_mtx_wave_len_[d_mtx_jam_ch_];
                d_mtx_geom_.half_span_D = d_mtx_base_[0];
                d_mtx_geom_.phys_len_L = job.tx_samples;
                d_mtx_geom_.sense_begin = d_mtx_base_[0];
            }
        }
        d_mtx_rng_.seed_rng(job.jam_delay_seed);
        d_mtx_delay_seed_ = job.jam_delay_seed;
        d_mtx_bank_ = job.window_bank;
        if (!d_mtx_bank_ || d_mtx_bank_->tx_len != job.tx_samples ||
            d_mtx_bank_->channel_count != nch) {
            d_invalid_.fetch_add(1, std::memory_order_relaxed);
            pmt::pmt_t extra = pmt::make_dict();
            extra = pmt::dict_add(extra, pmt::mp("pulse_id"),
                                  pmt::from_uint64(job.pulse_id));
            extra = pmt::dict_add(extra, pmt::mp("reason"),
                                  pmt::mp("window_bank_missing"));
            publish_status("invalid_schedule", extra);
            d_mtx_armed_ = false;
            d_mtx_bank_.reset();
            d_armed_.store(false, std::memory_order_relaxed);
            return;
        }
        d_mtx_freqs_.clear();
        for (double f : job.jam_freq_offsets_hz)
            d_mtx_freqs_.push_back(f);
        d_mtx_dwell_ = job.jam_dwell;
        d_mtx_settle_ticks_ = job.jam_freq_settle_ticks;
        // Absolute scan base: schedule "freq_hz" when provided, else the
        // last applied centre frequency (0 = offsets act as-is, e.g. QA).
        d_mtx_base_hz_ = 0.0;
        if (radar_meta::dict_has(job.meta, "freq_hz")) {
            const double fh = radar_meta::to_f64(
                pmt::dict_ref(job.meta, pmt::mp("freq_hz"),
                              pmt::from_double(0.0)),
                0.0);
            if (std::isfinite(fh) && fh > 0.0)
                d_mtx_base_hz_ = fh;
        }
        if (d_mtx_base_hz_ == 0.0)
            d_mtx_base_hz_ = d_freq_hz_.load(std::memory_order_relaxed);
        // Safety/contract guard: a jam plan with a NONZERO dwell offset is
        // an ABSOLUTE tune (base + offset).  With an unknown base (no
        // schedule "freq_hz" and no applied set_freq) the worker would tune
        // the jammer to the bare offset — measured 2026-09-19: an
        // offset-only 491339 Hz request was coerced by the device to 1 MHz,
        // silently moving the jammer out of band and making a "null" look
        // like interference cancellation.  Refuse to arm instead.
        bool needs_base = false;
        for (double f : d_mtx_freqs_) {
            if (std::isfinite(f) && std::fabs(f) > 0.0) {
                needs_base = true;
                break;
            }
        }
        if (needs_base && !(d_mtx_base_hz_ > 0.0)) {
            // Defensive: the handler already rejects this, so reaching here
            // means the PDU bypassed validation.  Disarm the grid too (it
            // was armed above) so no burst is transmitted on a bogus plan.
            d_invalid_.fetch_add(1, std::memory_order_relaxed);
            pmt::pmt_t extra = pmt::make_dict();
            extra = pmt::dict_add(extra, pmt::mp("pulse_id"),
                                  pmt::from_uint64(job.pulse_id));
            extra = pmt::dict_add(
                extra, pmt::mp("reason"),
                pmt::mp("jam_scan_base_unknown"));
            publish_status("invalid_schedule", extra);
            d_mtx_armed_ = false;
            d_armed_.store(false, std::memory_order_relaxed);
            return;
        }
        // The app prepares the jammer at the first plan offset.  From here
        // on the worker owns a monotonic scan state: a step advances only
        // after dwell successful RX captures, never from pulse_id/index.
        d_mtx_step_ = d_mtx_freqs_.empty()
                          ? std::numeric_limits<size_t>::max()
                          : 0;
        d_mtx_ok_in_step_ = 0;
        d_mtx_advance_pending_ = false;
        d_mtx_jam_offset_hz_ =
            d_mtx_freqs_.empty() ? 0.0 : d_mtx_freqs_[0];
        d_mtx_jam_actual_hz_ = d_mtx_base_hz_ + d_mtx_jam_offset_hz_;
        d_atomic_jam_ch_.store(d_mtx_jam_ch_, std::memory_order_relaxed);
        d_jam_delay_last_.store(0, std::memory_order_relaxed);
        d_jam_delay_us_last_.store(0.0, std::memory_order_relaxed);
        d_jam_plan_last_.store(d_mtx_jam_actual_hz_,
                               std::memory_order_relaxed);
        d_jam_actual_last_.store(d_mtx_jam_actual_hz_,
                                 std::memory_order_relaxed);
        d_jam_offset_last_.store(d_mtx_jam_offset_hz_,
                                 std::memory_order_relaxed);
        d_jam_retunes_.store(0, std::memory_order_relaxed);
        d_jam_retune_fails_.store(0, std::memory_order_relaxed);
    } else {
        d_mtx_geom_ = echo::MultiTxGeometry{};
        d_mtx_payloads_.clear();
        d_mtx_bank_.reset();
        d_mtx_delay_seed_ = 0;
        d_mtx_freqs_.clear();
        d_mtx_dwell_ = 0;
        d_mtx_settle_ticks_ = 0;
        d_mtx_step_ = std::numeric_limits<size_t>::max();
        d_mtx_ok_in_step_ = 0;
        d_mtx_advance_pending_ = false;
        d_mtx_jam_offset_hz_ = 0.0;
        d_mtx_jam_actual_hz_ = 0.0;
    }
    d_armed_.store(true, std::memory_order_relaxed);

    pmt::pmt_t extra = pmt::make_dict();
    extra = pmt::dict_add(extra, pmt::mp("pulse_id"),
                          pmt::from_uint64(job.pulse_id));
    extra = pmt::dict_add(extra, pmt::mp("t0_ticks"),
                          pmt::from_long(job.t0_ticks));
    extra = pmt::dict_add(extra, pmt::mp("tx_samples"),
                          pmt::from_uint64(job.tx_samples));
    extra = pmt::dict_add(extra, pmt::mp("rx_samples"),
                          pmt::from_uint64(job.rx_samples));
    extra = pmt::dict_add(extra, pmt::mp("burst_count"),
                          pmt::from_uint64(job.burst_count));
    publish_status("schedule_armed", extra);
}

void
UwbRealtimeEchoTimer::run_one_burst()
{
    // Pending retune (M1.1): applied at a burst boundary, i.e. serialized
    // with all UHD I/O and never concurrently with issue_*/collect.  On
    // failure the request stays pending and is retried on the next burst.
    if (d_tune_pending_.load(std::memory_order_relaxed)) {
        const double hz = d_tune_hz_.load(std::memory_order_relaxed);
        std::string tune_err;
        const echo::BurstStatus ts = d_backend_->tune(hz, tune_err);
        if (ts == echo::BurstStatus::Ok) {
            d_freq_hz_.store(hz, std::memory_order_relaxed);
            d_tune_pending_.store(false, std::memory_order_relaxed);
        } else {
            {
                std::lock_guard<std::mutex> lock(d_err_mutex_);
                d_last_error_ = tune_err;
            }
            pmt::pmt_t extra = pmt::make_dict();
            extra = pmt::dict_add(extra, pmt::mp("hz"),
                                  pmt::from_double(hz));
            extra = pmt::dict_add(extra, pmt::mp("error"),
                                  pmt::string_to_symbol(tune_err));
            publish_status("tune_failed", extra);
        }
    }

    // Grid slot: skip expired slots (no catch-up), take the first future
    // one.  The grid advances past the returned slot, so every valid
    // schedule index is produced exactly once, in order.
    echo::EchoSlot slot;
    int64_t now = d_backend_->device_time_ticks();
    const echo::EchoScheduleStatus gs = d_grid_.next_schedule(now, slot);
    if (gs == echo::EchoScheduleStatus::NotArmed)
        return;
    if (gs != echo::EchoScheduleStatus::Ok &&
        gs != echo::EchoScheduleStatus::ExpiredSkipped) {
        d_grid_errors_.fetch_add(1, std::memory_order_relaxed);
        pmt::pmt_t extra = pmt::make_dict();
        extra = pmt::dict_add(
            extra, pmt::mp("reason"),
            pmt::mp(gs == echo::EchoScheduleStatus::SkipLimitExceeded
                        ? "skip_limit_exceeded"
                        : "grid_overflow"));
        publish_status("grid_error", extra);
        d_armed_.store(false, std::memory_order_relaxed);
        return;
    }
    d_late_slot_skips_.fetch_add(slot.skipped, std::memory_order_relaxed);
    if (slot.skipped > 0) {
        pmt::pmt_t extra = pmt::make_dict();
        extra = pmt::dict_add(extra, pmt::mp("schedule_index"),
                              pmt::from_uint64(slot.index));
        extra = pmt::dict_add(extra, pmt::mp("skipped"),
                              pmt::from_uint64(slot.skipped));
        publish_status("late_slot_skip", extra);
    }

    // Per-burst publish extras (M3/M4/M5).  Legacy default reproduces the
    // exact old metadata plus the always-present §5.2/§5.6 accounting keys.
    PubExtra px;

    // --- multi-TX per-burst preamble (M4/M5) ---
    // Runs serially on this worker BEFORE any issue_* call: dwell-boundary
    // jammer retune (+settle re-anchor) and the per-pulse random delay.  No
    // Python callbacks, I/O, prints, or allocation (stack integers and the
    // frozen RNG state only).
    const bool use_multi = d_mtx_armed_ && d_mtx_N_ > 1;
    int64_t mtx_delay = 0; // relative jammer delay, native samples
    if (use_multi) {
        // M5 dwell/scan: the step is worker-owned and advances only after
        // the current frequency has produced d_mtx_dwell_ valid captures.
        // schedule_index may jump over late slots, but that must not consume
        // dwell or skip a frequency.
        double step_target_off = d_mtx_jam_offset_hz_;
        if (d_mtx_dwell_ > 0 && !d_mtx_freqs_.empty()) {
            const size_t step = d_mtx_advance_pending_
                                    ? d_mtx_step_ + 1
                                    : d_mtx_step_;
            if (step >= d_mtx_freqs_.size()) {
                d_grid_errors_.fetch_add(1, std::memory_order_relaxed);
                pmt::pmt_t extra = pmt::make_dict();
                extra = pmt::dict_add(extra, pmt::mp("reason"),
                                      pmt::mp("jam_scan_step_overflow"));
                publish_status("grid_error", extra);
                d_armed_.store(false, std::memory_order_relaxed);
                return;
            }
            step_target_off = d_mtx_freqs_[step];
            if (d_mtx_advance_pending_) {
                const double target_abs = d_mtx_base_hz_ + step_target_off;
                double actual = 0.0;
                std::string terr;
                const echo::BurstStatus mts = d_backend_->tune_tx_channel(
                    d_mtx_jam_ch_, target_abs, actual, terr);
                if (mts == echo::BurstStatus::Ok) {
                    d_mtx_jam_offset_hz_ = step_target_off;
                    d_mtx_jam_actual_hz_ = actual;
                    d_mtx_step_ = step;
                    d_mtx_ok_in_step_ = 0;
                    d_mtx_advance_pending_ = false;
                    d_jam_retunes_.fetch_add(1, std::memory_order_relaxed);
                    d_jam_plan_last_.store(target_abs,
                                           std::memory_order_relaxed);
                    d_jam_actual_last_.store(actual,
                                             std::memory_order_relaxed);
                    d_jam_offset_last_.store(step_target_off,
                                             std::memory_order_relaxed);
                    // Settle re-anchor (§5.5): the current index moves to
                    // now + settle_ticks (settle == 0 still steps 1 tick so
                    // the re-fetched slot is strictly in the future).
                    // pulse_id/index stay continuous, never counted as late.
                    now = d_backend_->device_time_ticks();
                    const uint64_t add = d_mtx_settle_ticks_ == 0
                                             ? 1
                                             : d_mtx_settle_ticks_;
                    const uint64_t nowu =
                        now < 0 ? 0 : static_cast<uint64_t>(now);
                    const __int128 t128 =
                        static_cast<__int128>(nowu) + add;
                    const int64_t t0new =
                        t128 > std::numeric_limits<int64_t>::max()
                            ? std::numeric_limits<int64_t>::max()
                            : static_cast<int64_t>(t128);
                    if (d_grid_.arm(t0new, slot.index)) {
                        echo::EchoSlot rs;
                        const echo::EchoScheduleStatus rgs =
                            d_grid_.next_schedule(now, rs);
                        if (rgs == echo::EchoScheduleStatus::Ok ||
                            rgs == echo::EchoScheduleStatus::ExpiredSkipped) {
                            d_late_slot_skips_.fetch_add(
                                rs.skipped, std::memory_order_relaxed);
                            slot = rs; // same index, re-anchored time
                        } else {
                            d_grid_errors_.fetch_add(
                                1, std::memory_order_relaxed);
                            pmt::pmt_t extra = pmt::make_dict();
                            extra = pmt::dict_add(
                                extra, pmt::mp("reason"),
                                pmt::mp("reanchor_grid_overflow"));
                            publish_status("grid_error", extra);
                            d_armed_.store(false, std::memory_order_relaxed);
                            return;
                        }
                    }
                } else {
                    // Never transmit another sample at the old frequency
                    // after its dwell is complete.  Keep the transition
                    // pending and retry on a later grid slot.
                    d_jam_retune_fails_.fetch_add(1,
                                                  std::memory_order_relaxed);
                    {
                        std::lock_guard<std::mutex> lock(d_err_mutex_);
                        d_last_error_ = terr;
                    }
                    pmt::pmt_t extra = pmt::make_dict();
                    extra = pmt::dict_add(
                        extra, pmt::mp("logical_channel"),
                        pmt::from_uint64(
                            static_cast<uint64_t>(d_mtx_jam_ch_)));
                    extra = pmt::dict_add(extra, pmt::mp("plan_hz"),
                                          pmt::from_double(target_abs));
                    extra = pmt::dict_add(extra, pmt::mp("error"),
                                          pmt::string_to_symbol(terr));
                    publish_status("tune_failed", extra);
                    return;
                }
            }
        }
        // M4 per-pulse jammer delay: uniform draws consume exactly one PRNG
        // step per attempted burst (fixed seed → reproducible sequence);
        // fixed mode uses the constant.  A silent jammer (jam_len == 0)
        // ignores the value in planning but still reports/draws it, so the
        // sequence never depends on the jammer length.
        if (d_mtx_delay_mode_ == echo::JamDelayMode::Uniform)
            mtx_delay = d_mtx_rng_.next_range_inclusive(
                d_mtx_delay_lo_, d_mtx_delay_hi_);
        else
            mtx_delay = d_mtx_delay_lo_;
        px.multi = true;
        px.jam_delay_native = mtx_delay;
        px.jam_delay_us =
            (d_sample_rate_ > 0.0)
                ? static_cast<double>(mtx_delay) / d_sample_rate_ * 1.0e6
                : 0.0;
        px.jam_freq_offset_hz = step_target_off;
        px.jam_freq_plan_hz = d_mtx_base_hz_ + step_target_off;
        px.jam_freq_actual_hz = d_mtx_jam_actual_hz_;
        px.jam_retune_seq =
            d_jam_retunes_.load(std::memory_order_relaxed);
        px.jam_scan_step = d_mtx_step_;
        px.jam_dwell_target = d_mtx_dwell_;
        px.jam_dwell_successes_before = d_mtx_ok_in_step_;
        px.jam_delay_mode = d_mtx_delay_mode_;
        px.jam_delay_seed = d_mtx_delay_seed_;
        px.sense_offset_native =
            d_mtx_bank_ ? d_mtx_bank_->sense_offset : d_mtx_base_[0];
        px.tx_fragment_count = 1;
        d_jam_delay_last_.store(mtx_delay, std::memory_order_relaxed);
        d_jam_delay_us_last_.store(px.jam_delay_us,
                                   std::memory_order_relaxed);
        d_jam_plan_last_.store(px.jam_freq_plan_hz,
                               std::memory_order_relaxed);
        d_jam_offset_last_.store(px.jam_freq_offset_hz,
                                 std::memory_order_relaxed);
    }

    // D: planned lead time of this slot at burst start, i.e. how far in the
    // future the TX tick was when the worker picked the slot.  Diagnostics
    // only; with no device sample rate we report 0 (never a divide).
    uint64_t slot_lead_us = 0;
    if (d_sample_rate_ > 0.0) {
        const int64_t lead_ticks = slot.t_tx_whole - now;
        if (lead_ticks > 0) {
            const double us = static_cast<double>(lead_ticks) /
                              d_sample_rate_ * 1.0e6;
            if (std::isfinite(us) && us > 0.0 &&
                us <= static_cast<double>(
                          std::numeric_limits<uint64_t>::max()))
                slot_lead_us = static_cast<uint64_t>(us);
        }
    }
    d_slot_lead_us_last_.store(slot_lead_us, std::memory_order_relaxed);
    accumulate_us(d_slot_lead_us_total_, d_slot_lead_us_max_, slot_lead_us);

    echo::BurstResult r;
    r.schedule_index = slot.index;
    r.tx_ticks = slot.t_tx_whole;
    r.rx_ticks = slot.t_rx_whole;
    r.skipped_slots = slot.skipped;
    r.tx_samples_requested = d_tx_samples_;
    r.rx_samples_requested = d_rx_samples_;
    // §5.2 accounting: per-channel counts stay compatible; wire counts are
    // channels × per-channel for transport.
    r.tx_channel_count = use_multi ? d_mtx_N_ : 1;
    r.tx_wire_samples_requested =
        wire_sat(r.tx_channel_count, r.tx_samples_requested);

    // Whole-burst worker wall time (M0/M1): fragment planning → issue RX →
    // issue TX → collect → publish.  No printing on the hot path.
    const auto burst_t0 = std::chrono::steady_clock::now();
    const auto record_worker_us = [&]() {
        const uint64_t us = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::steady_clock::now() - burst_t0)
                .count());
        d_worker_us_last_.store(us, std::memory_order_relaxed);
        d_worker_us_total_.fetch_add(us, std::memory_order_relaxed);
        uint64_t cur = d_worker_us_max_.load(std::memory_order_relaxed);
        while (us > cur &&
               !d_worker_us_max_.compare_exchange_weak(
                   cur, us, std::memory_order_relaxed)) {
        }
    };

    // Legacy grids consume their finite budget per attempt.  A frequency
    // scan instead consumes its budget only for valid RX captures, because
    // burst_count is the requested number of output samples (Nfreq*dwell).
    const bool success_counted_scan =
        use_multi && d_mtx_dwell_ > 0 && !d_mtx_freqs_.empty();
    bool end_after = false;
    if (!success_counted_scan &&
        d_burst_budget_ != std::numeric_limits<uint64_t>::max()) {
        --d_burst_budget_;
        end_after = d_burst_budget_ == 0;
    }
    ++d_bursts_this_grid_;

    const auto fail_burst = [&](echo::BurstStatus st, const std::string& e) {
        r.status = st;
        r.error = e;
        {
            std::lock_guard<std::mutex> lock(d_err_mutex_);
            d_last_error_ = e;
        }
        d_bursts_published_.fetch_add(1, std::memory_order_relaxed);
        d_bursts_failed_.fetch_add(1, std::memory_order_relaxed);
        d_backend_->abort_rx();
        publish_burst(r, /*with_samples=*/false, px);
        record_worker_us();
        if (end_after)
            finish_grid();
    };

    // Fragment planning (fixed-size arrays; no hot-path allocation).
    // RX planning is shared; TX planning branches on the armed grid.
    echo::EchoFragmentSpan rx_spans[echo::kEchoMaxFragmentsPerBurst];
    size_t nrx = 0;
    if (!echo::plan_fragments(d_rx_samples_, d_max_frag_, rx_spans,
                              echo::kEchoMaxFragmentsPerBurst, nrx)) {
        fail_burst(echo::BurstStatus::BackendError,
                   "fragment planning failed (too many fragments)");
        return;
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

    echo::TxCommand tx_cmd;
    tx_cmd.schedule_index = r.schedule_index;
    tx_cmd.tx_ticks = slot.t_tx_whole;
    tx_cmd.total_samples = d_tx_samples_;
    if (use_multi) {
        // Contiguous-window send: one fragment of length L.  Delay only
        // slides the jammer backing pointer; TX0 pointer/count/offset
        // are identical every burst.  No fill, copy, or resize.
        if (!d_mtx_bank_ || d_max_frag_ < d_tx_samples_) {
            fail_burst(echo::BurstStatus::BackendError,
                       "multi-TX contiguous window unavailable");
            return;
        }
        const echo::MultiTxWindowBank& bank = *d_mtx_bank_;
        const uint64_t L = bank.tx_len;
        if (L != d_tx_samples_ || bank.channel_count != d_mtx_N_) {
            fail_burst(echo::BurstStatus::BackendError,
                       "multi-TX window bank mismatch");
            return;
        }
        echo::TxBurstFragment& f = d_mtxf_[0];
        f.offset = 0;
        f.count = L;
        f.flags = echo::kFlagTimeSpec | echo::kFlagStartOfBurst |
                  echo::kFlagEndOfBurst;
        f.device_ticks = slot.t_tx_whole;
        for (size_t c = 0; c < d_mtx_N_; ++c) {
            if (bank.dense_rows[c].size() != L * 2) {
                fail_burst(echo::BurstStatus::BackendError,
                           "multi-TX dense row length mismatch");
                return;
            }
            f.tx_data[c] = bank.dense_rows[c].data();
        }
        std::string perr;
        const int16_t* jam_ptr =
            echo::multitx_jam_ptr(bank, mtx_delay, &perr);
        if (jam_ptr == nullptr) {
            fail_burst(echo::BurstStatus::BackendError,
                       perr.empty() ? "jam_window_ptr"
                                    : perr);
            return;
        }
        f.tx_data[d_mtx_jam_ch_] = jam_ptr;
        px.tx_fragment_count = 1;
        px.sense_offset_native = bank.sense_offset;
        px.sense_tx_ticks =
            slot.t_tx_whole + static_cast<int64_t>(bank.sense_offset);
        px.jam_tx_ticks = px.sense_tx_ticks + mtx_delay;
        tx_cmd.fragments = nullptr;
        tx_cmd.fragment_count = 1;
        tx_cmd.tx_channel_count = d_mtx_N_;
        tx_cmd.tx_multi_fragments = d_mtxf_;
    } else {
        echo::EchoFragmentSpan tx_spans[echo::kEchoMaxFragmentsPerBurst];
        size_t ntx = 0;
        if (!echo::plan_fragments(d_tx_samples_, d_max_frag_, tx_spans,
                                  echo::kEchoMaxFragmentsPerBurst, ntx)) {
            fail_burst(echo::BurstStatus::BackendError,
                       "fragment planning failed (too many fragments)");
            return;
        }
        for (size_t i = 0; i < ntx; ++i) {
            echo::BurstFragment f;
            f.tx_data = d_tx_ptr_ + tx_spans[i].offset * 2;
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
        tx_cmd.fragments = d_txf_;
        tx_cmd.fragment_count = ntx;
    }

    const echo::RxCommand rx_cmd{ r.schedule_index,
                                  slot.t_rx_whole,
                                  d_rx_samples_,
                                  d_rxf_,
                                  nrx };
    std::string err;

    // The RX command is always issued BEFORE the TX burst.
    const auto issue_rx_t0 = std::chrono::steady_clock::now();
    const echo::BurstStatus rx_st = d_backend_->issue_rx(rx_cmd, err);
    accumulate_us(d_issue_rx_us_total_, d_issue_rx_us_max_,
                  elapsed_us(issue_rx_t0, std::chrono::steady_clock::now()));
    if (rx_st != echo::BurstStatus::Ok) {
        fail_burst(rx_st, err);
        return;
    }
    if (d_stop_.load(std::memory_order_relaxed)) {
        fail_burst(echo::BurstStatus::StopDuringIo,
                   "stop after RX command, before TX");
        return;
    }
    const auto issue_tx_t0 = std::chrono::steady_clock::now();
    const echo::BurstStatus tx_st = d_backend_->issue_tx(tx_cmd, err);
    accumulate_us(d_tx_send_us_total_, d_tx_send_us_max_,
                  elapsed_us(issue_tx_t0, std::chrono::steady_clock::now()));
    if (tx_st != echo::BurstStatus::Ok) {
        fail_burst(tx_st, err);
        return;
    }
    if (d_stop_.load(std::memory_order_relaxed)) {
        fail_burst(echo::BurstStatus::StopDuringIo,
                   "stop after TX burst, before RX collect");
        return;
    }

    // Asynchronous RX-completion wait (backend event / UHD recv) — never
    // a busy wait.  Exactly one result per schedule index either way.
    echo::BurstResult done;
    const auto collect_t0 = std::chrono::steady_clock::now();
    const bool collected = d_backend_->collect_result(
        r.schedule_index, d_collect_wait_ms_, done);
    accumulate_us(d_rx_collect_us_total_, d_rx_collect_us_max_,
                  elapsed_us(collect_t0, std::chrono::steady_clock::now()));
    if (collected) {
        r.status = done.status;
        r.rx_time_ticks = done.rx_time_ticks;
        r.tx_samples_sent = done.tx_samples_sent;
        r.rx_samples_received = done.rx_samples_received;
        r.tx_reissues = done.tx_reissues;
        r.rx_reissues = done.rx_reissues;
        r.error = done.error;
        // §5.2 transport accounting: channels × per-channel (== per-channel
        // for legacy single-TX, so old consumers stay bit-exact).
        r.tx_wire_samples_sent =
            wire_sat(r.tx_channel_count, r.tx_samples_sent);
        d_tx_reissues_.fetch_add(done.tx_reissues,
                                 std::memory_order_relaxed);
        d_rx_reissues_.fetch_add(done.rx_reissues,
                                 std::memory_order_relaxed);
        const bool status_ok = r.status == echo::BurstStatus::Ok ||
                               r.status == echo::BurstStatus::PartialHandled;
        const bool ok = status_ok &&
                        r.rx_samples_received == r.rx_samples_requested;
        if (status_ok && !ok) {
            r.status = echo::BurstStatus::BackendError;
            r.error = "successful RX status returned an incomplete capture";
        }
        {
            std::lock_guard<std::mutex> lock(d_err_mutex_);
            d_last_error_ = ok ? std::string() : r.error;
        }
        if (ok)
            d_bursts_ok_.fetch_add(1, std::memory_order_relaxed);
        else
            d_bursts_failed_.fetch_add(1, std::memory_order_relaxed);
        d_bursts_published_.fetch_add(1, std::memory_order_relaxed);
        publish_burst(r, /*with_samples=*/ok, px);
        record_worker_us();
        if (ok && success_counted_scan) {
            if (d_burst_budget_ != std::numeric_limits<uint64_t>::max() &&
                d_burst_budget_ > 0)
                --d_burst_budget_;
            ++d_mtx_ok_in_step_;
            if (d_mtx_ok_in_step_ == d_mtx_dwell_) {
                pmt::pmt_t extra = pmt::make_dict();
                extra = pmt::dict_add(
                    extra, pmt::mp("jam_scan_step"),
                    pmt::from_uint64(static_cast<uint64_t>(d_mtx_step_)));
                extra = pmt::dict_add(extra, pmt::mp("samples"),
                                      pmt::from_uint64(d_mtx_ok_in_step_));
                publish_status("jam_frequency_complete", extra);
                if (d_mtx_step_ + 1 == d_mtx_freqs_.size())
                    end_after = true;
                else
                    d_mtx_advance_pending_ = true;
            }
        }
    } else if (d_stop_.load(std::memory_order_relaxed)) {
        fail_burst(echo::BurstStatus::StopDuringIo,
                   "stop while RX I/O in flight");
        return;
    } else {
        fail_burst(echo::BurstStatus::Timeout,
                   "RX did not complete within collect wait");
        return;
    }
    if (end_after)
        finish_grid();
}

void
UwbRealtimeEchoTimer::finish_grid()
{
    d_armed_.store(false, std::memory_order_relaxed);
    pmt::pmt_t extra = pmt::make_dict();
    extra = pmt::dict_add(extra, pmt::mp("pulse_id"),
                          pmt::from_uint64(d_pulse_id_));
    extra = pmt::dict_add(extra, pmt::mp("bursts"),
                          pmt::from_uint64(d_bursts_this_grid_));
    if (d_mtx_armed_ && d_mtx_dwell_ > 0 && !d_mtx_freqs_.empty()) {
        extra = pmt::dict_add(
            extra, pmt::mp("jam_scan_steps"),
            pmt::from_uint64(static_cast<uint64_t>(d_mtx_freqs_.size())));
        extra = pmt::dict_add(extra, pmt::mp("jam_dwell"),
                              pmt::from_uint64(d_mtx_dwell_));
        extra = pmt::dict_add(extra, pmt::mp("jam_scan_successes"),
                              pmt::from_uint64(
                                  static_cast<uint64_t>(d_mtx_freqs_.size()) *
                                  d_mtx_dwell_));
    }
    publish_status("grid_complete", extra);
}

// ---------------------------------------------------------------------------
// Publishing
// ---------------------------------------------------------------------------

void
UwbRealtimeEchoTimer::publish_burst(const echo::BurstResult& r,
                                    bool with_samples,
                                    const PubExtra& px)
{
    // D: split the publish path into metadata/PMT build vs the actual
    // message_port_pub.  Diagnostics only; the code below is unchanged.
    const auto pdu_build_t0 = std::chrono::steady_clock::now();
    pmt::pmt_t meta = pmt::make_dict();
    meta = pmt::dict_add(meta, pmt::mp("status"),
                         pmt::mp(echo::burst_status_to_string(r.status)));
    meta = pmt::dict_add(meta, pmt::mp("status_code"),
                         pmt::from_long(static_cast<long>(r.status)));
    meta = pmt::dict_add(meta, pmt::mp("schedule_index"),
                         pmt::from_uint64(r.schedule_index));
    // One multi-burst schedule can request per-burst unique ids so the
    // downstream CIR writer/analyzer sees 0..N-1 exactly once.
    const uint64_t burst_pulse_id =
        d_pulse_id_increment_ != 0
            ? d_pulse_id_ +
                  (r.schedule_index - d_sched_index_start_)
            : d_pulse_id_;
    meta = pmt::dict_add(meta, pmt::mp("pulse_id"),
                         pmt::from_uint64(burst_pulse_id));
    // Only when incrementing do we own packet_id; otherwise it stays the
    // schedule passthrough value copied by the whitelist below.
    if (d_pulse_id_increment_ != 0) {
        meta = pmt::dict_add(meta, pmt::mp("packet_id"),
                             pmt::from_uint64(burst_pulse_id));
    }
    meta = pmt::dict_add(meta, pmt::mp("tx_ticks"),
                         pmt::from_long(r.tx_ticks));
    meta = pmt::dict_add(meta, pmt::mp("rx_ticks"),
                         pmt::from_long(r.rx_ticks));
    meta = pmt::dict_add(meta, pmt::mp("rx_time_ticks"),
                         pmt::from_long(r.rx_time_ticks));
    meta = pmt::dict_add(meta, pmt::mp("skipped_slots"),
                         pmt::from_uint64(r.skipped_slots));
    meta = pmt::dict_add(meta, pmt::mp("tx_samples_requested"),
                         pmt::from_uint64(r.tx_samples_requested));
    meta = pmt::dict_add(meta, pmt::mp("tx_samples_sent"),
                         pmt::from_uint64(r.tx_samples_sent));
    meta = pmt::dict_add(meta, pmt::mp("rx_samples_requested"),
                         pmt::from_uint64(r.rx_samples_requested));
    meta = pmt::dict_add(meta, pmt::mp("rx_samples_received"),
                         pmt::from_uint64(r.rx_samples_received));
    meta = pmt::dict_add(meta, pmt::mp("tx_reissues"),
                         pmt::from_uint64(r.tx_reissues));
    meta = pmt::dict_add(meta, pmt::mp("rx_reissues"),
                         pmt::from_uint64(r.rx_reissues));
    // §5.2 multi-TX accounting (present on every burst; == per-channel
    // values for legacy single-TX).
    meta = pmt::dict_add(
        meta, pmt::mp("tx_channel_count"),
        pmt::from_uint64(static_cast<uint64_t>(r.tx_channel_count)));
    meta = pmt::dict_add(meta, pmt::mp("tx_wire_samples_requested"),
                         pmt::from_uint64(r.tx_wire_samples_requested));
    meta = pmt::dict_add(meta, pmt::mp("tx_wire_samples_sent"),
                         pmt::from_uint64(r.tx_wire_samples_sent));
    // §5.6 TX async snapshot (backend-owned counters; zero with no backend).
    const echo::TxAsyncCounts async =
        d_backend_ ? d_backend_->tx_async_counts() : echo::TxAsyncCounts{};
    meta = pmt::dict_add(meta, pmt::mp("tx_async_underflow"),
                         pmt::from_uint64(async.underflow));
    meta = pmt::dict_add(meta, pmt::mp("tx_async_seq_error"),
                         pmt::from_uint64(async.seq_error));
    meta = pmt::dict_add(meta, pmt::mp("tx_async_time_error"),
                         pmt::from_uint64(async.time_error));
    meta = pmt::dict_add(meta, pmt::mp("tx_async_unmatched"),
                         pmt::from_uint64(async.unmatched));
    meta = pmt::dict_add(meta, pmt::mp("tx_async_dropped"),
                         pmt::from_uint64(async.dropped));
    meta = pmt::dict_add(meta, pmt::mp("tx_async_ack"),
                         pmt::from_uint64(async.ack));
    // §5.4/§5.5 per-burst jammer state (multi-TX bursts only; single-TX
    // metadata is otherwise unchanged).
    if (px.multi) {
        meta = pmt::dict_add(meta, pmt::mp("jam_delay_native"),
                             pmt::from_long(px.jam_delay_native));
        meta = pmt::dict_add(meta, pmt::mp("jam_delay_us"),
                             pmt::from_double(px.jam_delay_us));
        meta = pmt::dict_add(meta, pmt::mp("jam_freq_plan_hz"),
                             pmt::from_double(px.jam_freq_plan_hz));
        meta = pmt::dict_add(meta, pmt::mp("jam_freq_actual_hz"),
                             pmt::from_double(px.jam_freq_actual_hz));
        meta = pmt::dict_add(meta, pmt::mp("jam_freq_offset_hz"),
                             pmt::from_double(px.jam_freq_offset_hz));
        meta = pmt::dict_add(meta, pmt::mp("jam_retune_seq"),
                             pmt::from_uint64(px.jam_retune_seq));
        meta = pmt::dict_add(meta, pmt::mp("jam_scan_step"),
                             pmt::from_uint64(px.jam_scan_step));
        meta = pmt::dict_add(meta, pmt::mp("jam_dwell_target"),
                             pmt::from_uint64(px.jam_dwell_target));
        meta = pmt::dict_add(
            meta, pmt::mp("jam_dwell_successes_before"),
            pmt::from_uint64(px.jam_dwell_successes_before));
        meta = pmt::dict_add(
            meta, pmt::mp("jam_delay_mode"),
            pmt::mp(px.jam_delay_mode == echo::JamDelayMode::Uniform
                        ? "uniform"
                        : "fixed"));
        meta = pmt::dict_add(meta, pmt::mp("jam_delay_seed"),
                             pmt::from_uint64(px.jam_delay_seed));
        meta = pmt::dict_add(meta, pmt::mp("sense_offset_native"),
                             pmt::from_uint64(px.sense_offset_native));
        meta = pmt::dict_add(meta, pmt::mp("tx_fragment_count"),
                             pmt::from_uint64(px.tx_fragment_count));
        meta = pmt::dict_add(meta, pmt::mp("sense_tx_ticks"),
                             pmt::from_long(px.sense_tx_ticks));
        meta = pmt::dict_add(meta, pmt::mp("jam_tx_ticks"),
                             pmt::from_long(px.jam_tx_ticks));
    }
    meta = pmt::dict_add(meta, pmt::mp("uhd_error"),
                         pmt::string_to_symbol(r.error));
    meta = pmt::dict_add(meta, pmt::mp("sample_format"), pmt::mp("sc16"));
    if (d_sample_rate_ > 0.0) {
        // Native sample rate: required by the PDU 65/32 resampler's
        // validate_input_rate contract.
        meta = pmt::dict_add(meta, pmt::mp("sample_rate"),
                             pmt::from_double(d_sample_rate_));
        // Whole-tick → (full, frac) device time; the fractional tick part
        // (rem/den) is ignored in this metadata convenience only.
        const double fs = d_sample_rate_;
        const uint64_t tx_full = static_cast<uint64_t>(
            static_cast<double>(r.tx_ticks) / fs);
        const uint64_t rx_full = static_cast<uint64_t>(
            static_cast<double>(r.rx_ticks) / fs);
        meta = pmt::dict_add(meta, pmt::mp("tx_time_full"),
                             pmt::from_uint64(tx_full));
        meta = pmt::dict_add(
            meta, pmt::mp("tx_time_frac"),
            pmt::from_double((static_cast<double>(r.tx_ticks) -
                              static_cast<double>(tx_full) * fs) /
                             fs));
        meta = pmt::dict_add(meta, pmt::mp("rx_time_full"),
                             pmt::from_uint64(rx_full));
        meta = pmt::dict_add(
            meta, pmt::mp("rx_time_frac"),
            pmt::from_double((static_cast<double>(r.rx_ticks) -
                              static_cast<double>(rx_full) * fs) /
                             fs));
    }
    // M1.2: whitelisted radar metadata pass-through from the schedule PDU.
    // copy_if_present keeps any key already emitted above and only copies
    // keys the schedule actually provides.
    static const char* const kScheduleMetaKeys[] = {
        "packet_id",           "window_start_sample", "pre_guard_samples",
        "capture_samples",     "post_guard_samples",  "sample_count",
        "rx_capture_samples",  "sync_samples",        "sfd_samples",
        "tx_packet_samples",   "num_delay_samps",     "sync_repetitions",
        "sfd_mode",            "code_index",          "source",
        "freq_hz",             "freq_offset_hz",      "calibration_id",
        "schedule_generation", "acquisition_epoch"
    };
    if (pmt::is_dict(d_sched_meta_)) {
        for (const char* key : kScheduleMetaKeys)
            radar_meta::copy_if_present(meta, d_sched_meta_, key);
    }
    // sample_count is the PHYSICAL RX window length: the schedule value is
    // authoritative when present, else r.rx_samples_received.  It is NOT
    // the truncated publish length — downstream derives post_guard from it.
    if (!radar_meta::dict_has(meta, "sample_count"))
        meta = pmt::dict_add(meta, pmt::mp("sample_count"),
                             pmt::from_uint64(r.rx_samples_received));

    // Calibration delay is block-owned (seeded from the schedule default in
    // apply_schedule, overridable via set_cal_delay_native()); the 65/32
    // work-grid alias is derived with the shared resampler constants.
    const double cal_native =
        d_cal_delay_native_.load(std::memory_order_relaxed);
    meta = pmt::dict_add(meta, pmt::mp("calibration_delay_native_samples"),
                         pmt::from_double(cal_native));
    meta = pmt::dict_add(
        meta, pmt::mp("calibration_delay_work_samples"),
        pmt::from_double(cal_native *
                         static_cast<double>(radar_meta::kResampleInterp) /
                         static_cast<double>(radar_meta::kCg400NativeDecim)));

    // M2 publish ROI: UHD still captures the full physical RX window; the
    // PDU payload only carries the leading native SC16 pairs the CIR reads.
    // rx_samples_received / sample_count stay PHYSICAL so geometry parity
    // with the golden chain is preserved.
    pmt::pmt_t vec;
    uint64_t published = 0;
    if (with_samples && r.rx_samples_received > 0 &&
        r.rx_samples_received <= d_rx_samples_) {
        const uint64_t physical = r.rx_samples_received;
        const uint64_t want = d_burst_publish_native_ != 0
                                  ? d_burst_publish_native_
                                  : d_publish_native_.load(
                                        std::memory_order_relaxed);
        published = (want > 0 && want < physical) ? want : physical;
        vec = pmt::init_s16vector(static_cast<size_t>(published) * 2,
                                  d_rx_buf_.data());
    } else {
        vec = pmt::init_s16vector(0, static_cast<const int16_t*>(nullptr));
    }
    meta = pmt::dict_add(meta, pmt::mp("published_samples"),
                         pmt::from_uint64(published));
    if (published > 0)
        d_published_samples_.fetch_add(published,
                                       std::memory_order_relaxed);
    const auto pdu_publish_t0 = std::chrono::steady_clock::now();
    accumulate_us(d_pdu_build_us_total_, d_pdu_build_us_max_,
                  elapsed_us(pdu_build_t0, pdu_publish_t0));
    message_port_pub(pmt::mp("burst"), pmt::cons(meta, vec));
    accumulate_us(d_pdu_publish_us_total_, d_pdu_publish_us_max_,
                  elapsed_us(pdu_publish_t0, std::chrono::steady_clock::now()));
}

void
UwbRealtimeEchoTimer::snapshot_stats(pmt::pmt_t& meta) const
{
    meta = pmt::dict_add(meta, pmt::mp("schedules_received"),
                         pmt::from_uint64(schedules_received()));
    meta = pmt::dict_add(meta, pmt::mp("schedules_enqueued"),
                         pmt::from_uint64(schedules_enqueued()));
    meta = pmt::dict_add(meta, pmt::mp("schedules_dropped"),
                         pmt::from_uint64(schedules_dropped()));
    meta = pmt::dict_add(meta, pmt::mp("schedules_invalid"),
                         pmt::from_uint64(schedules_invalid()));
    meta = pmt::dict_add(meta, pmt::mp("schedules_dropped_on_stop"),
                         pmt::from_uint64(schedules_dropped_on_stop()));
    meta = pmt::dict_add(meta, pmt::mp("bursts_published"),
                         pmt::from_uint64(bursts_published()));
    meta = pmt::dict_add(meta, pmt::mp("bursts_ok"),
                         pmt::from_uint64(bursts_ok()));
    meta = pmt::dict_add(meta, pmt::mp("bursts_failed"),
                         pmt::from_uint64(bursts_failed()));
    meta = pmt::dict_add(meta, pmt::mp("late_slot_skips"),
                         pmt::from_uint64(late_slot_skips()));
    meta = pmt::dict_add(meta, pmt::mp("tx_reissues"),
                         pmt::from_uint64(tx_reissues()));
    meta = pmt::dict_add(meta, pmt::mp("rx_reissues"),
                         pmt::from_uint64(rx_reissues()));
    meta = pmt::dict_add(meta, pmt::mp("grid_errors"),
                         pmt::from_uint64(grid_errors()));
    meta = pmt::dict_add(meta, pmt::mp("queue_depth"),
                         pmt::from_uint64(
                             static_cast<uint64_t>(queue_depth())));
    meta = pmt::dict_add(
        meta, pmt::mp("queue_high_watermark"),
        pmt::from_uint64(static_cast<uint64_t>(queue_high_watermark())));
}

void
UwbRealtimeEchoTimer::publish_status(const std::string& event,
                                     pmt::pmt_t extra)
{
    pmt::pmt_t meta = pmt::make_dict();
    meta = pmt::dict_add(meta, pmt::mp("event"), pmt::mp(event));
    snapshot_stats(meta);
    if (pmt::is_dict(extra)) {
        pmt::pmt_t items = pmt::dict_items(extra);
        for (size_t i = 0; i < pmt::length(items); ++i) {
            pmt::pmt_t kv = pmt::nth(i, items);
            meta = pmt::dict_add(meta, pmt::car(kv), pmt::cdr(kv));
        }
    }
    message_port_pub(pmt::mp("status"), meta);
}

} // namespace uwb
} // namespace gr
