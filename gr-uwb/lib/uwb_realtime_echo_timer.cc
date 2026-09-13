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
    auto reject = [&](const char* event) {
        d_invalid_.fetch_add(1, std::memory_order_relaxed);
        publish_status(event);
    };

    if (!pmt::is_pair(msg)) {
        reject("invalid_schedule");
        return;
    }
    const pmt::pmt_t meta = pmt::car(msg);
    const pmt::pmt_t payload = pmt::cdr(msg);
    if (!pmt::is_dict(meta) || !pmt::is_s16vector(payload)) {
        reject("invalid_schedule");
        return;
    }
    if (!dict_has(meta, "t0_ticks") || !dict_has(meta, "tx_samples") ||
        !dict_has(meta, "rx_samples")) {
        reject("invalid_schedule");
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
        reject("invalid_schedule");
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
        reject("invalid_schedule");
        return;
    }
    size_t payload_len = 0;
    pmt::s16vector_elements(payload, payload_len);
    // The payload holds tx_samples SC16 pairs = 2 * tx_samples int16;
    // tx_samples <= max_tx_samples <= SIZE_MAX/2 keeps this multiply safe.
    const size_t expect_elems = static_cast<size_t>(tx_samples) * 2;
    if (payload_len != expect_elems) {
        reject("invalid_schedule");
        return;
    }
    // Optional per-schedule fragment-size override; 0 (absent) = block
    // default from the prepared grid config.  Present-but-zero is invalid.
    const uint64_t max_frag_in = dict_u64(meta, "max_fragment_size", 0);
    if (dict_has(meta, "max_fragment_size") && max_frag_in == 0) {
        reject("invalid_schedule");
        return;
    }

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
    job.payload = payload;

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
    d_tx_ptr_ = pmt::s16vector_elements(job.payload, len);
    // Fixed scratch: zero the used prefix only; capacity/address stable.
    std::fill_n(d_rx_buf_.begin(),
                static_cast<size_t>(d_rx_samples_) * 2,
                static_cast<int16_t>(0));
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
    const int64_t now = d_backend_->device_time_ticks();
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

    // Burst budget (finite grids publish "grid_complete" after the last
    // burst; unlimited grids run until re-armed or stopped).  Every
    // produced burst — ok or failed — counts toward the budget and the
    // per-grid burst count.
    bool end_after = false;
    if (d_burst_budget_ != std::numeric_limits<uint64_t>::max()) {
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
        publish_burst(r, /*with_samples=*/false);
        record_worker_us();
        if (end_after)
            finish_grid();
    };

    // Fragment planning (fixed-size arrays; no hot-path allocation).
    echo::EchoFragmentSpan tx_spans[echo::kEchoMaxFragmentsPerBurst];
    echo::EchoFragmentSpan rx_spans[echo::kEchoMaxFragmentsPerBurst];
    size_t ntx = 0;
    size_t nrx = 0;
    if (!echo::plan_fragments(d_tx_samples_, d_max_frag_, tx_spans,
                              echo::kEchoMaxFragmentsPerBurst, ntx) ||
        !echo::plan_fragments(d_rx_samples_, d_max_frag_, rx_spans,
                              echo::kEchoMaxFragmentsPerBurst, nrx)) {
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

    const echo::RxCommand rx_cmd{ r.schedule_index,
                                  slot.t_rx_whole,
                                  d_rx_samples_,
                                  d_rxf_,
                                  nrx };
    const echo::TxCommand tx_cmd{ r.schedule_index,
                                  slot.t_tx_whole,
                                  d_tx_samples_,
                                  d_txf_,
                                  ntx };
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
        d_tx_reissues_.fetch_add(done.tx_reissues,
                                 std::memory_order_relaxed);
        d_rx_reissues_.fetch_add(done.rx_reissues,
                                 std::memory_order_relaxed);
        const bool ok = r.status == echo::BurstStatus::Ok ||
                        r.status == echo::BurstStatus::PartialHandled;
        {
            std::lock_guard<std::mutex> lock(d_err_mutex_);
            d_last_error_ = ok ? std::string() : r.error;
        }
        if (ok)
            d_bursts_ok_.fetch_add(1, std::memory_order_relaxed);
        else
            d_bursts_failed_.fetch_add(1, std::memory_order_relaxed);
        d_bursts_published_.fetch_add(1, std::memory_order_relaxed);
        publish_burst(r, /*with_samples=*/ok);
        record_worker_us();
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
    publish_status("grid_complete", extra);
}

// ---------------------------------------------------------------------------
// Publishing
// ---------------------------------------------------------------------------

void
UwbRealtimeEchoTimer::publish_burst(const echo::BurstResult& r,
                                    bool with_samples)
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
