/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * UwbRadarCirEstimator implementation.  Style mirrors UwbRealtimeDemodulator:
 * bounded queue, message_port_pub from the worker thread, publish_status
 * snapshots, stop() drains then joins.  Handlers never allocate scratch; the
 * only hot-path allocations are the unavoidable output PMTs.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gnuradio/uwb/uwb_radar_cir_estimator_block.h>
#include <gnuradio/uwb/uwb_radar_pdu_meta.h>
#include <gnuradio/io_signature.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <stdexcept>
#include <utility>

namespace gr {
namespace uwb {

namespace {

constexpr size_t kSps = demod::kQm35SamplesPerSymbol; // 1016 @ 998.4 MS/s
constexpr double kSpeedOfLight = 299792458.0;

inline uint64_t
elapsed_us(std::chrono::steady_clock::time_point t0,
           std::chrono::steady_clock::time_point t1)
{
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count());
}

uint64_t
dict_u64(pmt::pmt_t dict, const char* key, uint64_t def)
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

int64_t
dict_i64(pmt::pmt_t dict, const char* key, int64_t def)
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

bool dict_has(pmt::pmt_t dict, const char* key)
{
    return pmt::is_dict(dict) &&
           pmt::dict_has_key(dict, pmt::mp(key));
}

std::string dict_str(pmt::pmt_t dict, const char* key)
{
    if (!pmt::is_dict(dict))
        return {};
    pmt::pmt_t v = pmt::dict_ref(dict, pmt::mp(key), pmt::PMT_NIL);
    if (pmt::is_symbol(v))
        return pmt::symbol_to_string(v);
    return {};
}

bool dict_has_str(pmt::pmt_t dict, const char* key)
{
    return dict_has(dict, key) && pmt::is_symbol(
        pmt::dict_ref(dict, pmt::mp(key), pmt::PMT_NIL));
}

// Validate one calibration-delay PMT value and convert it to a finite,
// non-negative double without precision loss at the int64 boundary:
// - uint64 values above INT64_MAX are rejected (symbolic overflow),
// - negative integers are rejected,
// - real values must be finite and non-negative.
bool cal_delay_value(pmt::pmt_t v, double& out)
{
    if (pmt::is_uint64(v)) {
        const uint64_t u = pmt::to_uint64(v);
        if (u > static_cast<uint64_t>(INT64_MAX))
            return false; // > INT64_MAX: not representable
        out = static_cast<double>(u);
        return true;
    }
    if (pmt::is_integer(v)) {
        const long l = pmt::to_long(v);
        if (l < 0)
            return false;
        out = static_cast<double>(l);
        return true;
    }
    if (pmt::is_real(v)) {
        const double d = pmt::to_double(v);
        if (!std::isfinite(d) || d < 0.0)
            return false;
        out = d;
        return true;
    }
    return false;
}

// Calibration delay on the 998.4 work grid.  The 65/48 adapter stores the
// mapped value under calibration_delay_work_samples; direct 998.4 PDUs (no
// resampler mapping) only carry calibration_delay_native_samples, which is
// already work-domain there.
bool dict_cal_delay_work(pmt::pmt_t dict, double& out)
{
    if (dict_has(dict, radar_meta::kCalDelayWork)) {
        return cal_delay_value(
            pmt::dict_ref(dict, pmt::mp(radar_meta::kCalDelayWork),
                          pmt::PMT_NIL),
            out);
    }
    if (dict_has(dict, radar_meta::kCalDelayNative)) {
        return cal_delay_value(
            pmt::dict_ref(dict, pmt::mp(radar_meta::kCalDelayNative),
                          pmt::PMT_NIL),
            out);
    }
    return false;
}

// Checked double → int64 rounding.  llround() is only well-defined for
// values that fit the target integer; anything near or beyond INT64_MAX
// (or non-finite / negative) is rejected before rounding.  The bound is
// kept strictly inside INT64_MAX so the double comparison is exact.
bool cal_round_checked(double cal_work, int64_t& out)
{
    constexpr double kMaxWork = 9.0e18; // < INT64_MAX (≈9.223e18)
    if (!std::isfinite(cal_work) || cal_work < 0.0 || cal_work > kMaxWork)
        return false;
    out = static_cast<int64_t>(std::llround(cal_work));
    return true;
}

void copy_lineage(pmt::pmt_t& dst, pmt::pmt_t src)
{
    for (const char* key : radar_meta::kPassthrough) {
        radar_meta::copy_if_present(dst, src, key);
    }
    for (const char* key : { "sync_samples", "sfd_samples",
                             "range_guard_samples", "tx_packet_samples",
                             "rx_capture_samples", "sample_format" }) {
        radar_meta::copy_if_present(dst, src, key);
    }
}

} // namespace

// ---------------------------------------------------------------------------
// Construction / factory
// ---------------------------------------------------------------------------

std::vector<gr_complex>
UwbRadarCirEstimator::load_cf32_file(const std::string& path)
{
    std::ifstream f(path, std::ios::binary);
    if (!f) {
        throw std::invalid_argument(
            "UwbRadarCirEstimator: cannot open template file " + path);
    }
    f.seekg(0, std::ios::end);
    const std::streamoff bytes = f.tellg();
    f.seekg(0, std::ios::beg);
    if (bytes <= 0 ||
        bytes % static_cast<std::streamoff>(sizeof(gr_complex)) != 0) {
        throw std::invalid_argument(
            "UwbRadarCirEstimator: template file empty or invalid size: " +
            path);
    }
    std::vector<gr_complex> tmpl(static_cast<size_t>(bytes) /
                                 sizeof(gr_complex));
    f.read(reinterpret_cast<char*>(tmpl.data()), bytes);
    if (!f && !f.eof()) {
        throw std::invalid_argument(
            "UwbRadarCirEstimator: failed reading template file " + path);
    }
    return tmpl;
}

UwbRadarCirEstimator::UwbRadarCirEstimator(
    const std::string& template_path,
    size_t sync_repetitions,
    const std::string& sfd_mode,
    size_t code_index,
    size_t cir_pre,
    size_t cir_post,
    size_t cir_skip_initial,
    size_t cir_repetitions,
    int64_t sfd_search_margin,
    int64_t sync_refine_margin,
    float sfd_threshold,
    float sync_refine_threshold,
    bool emit_normalized,
    size_t queue_capacity)
    : gr::block("uwb_radar_cir_estimator",
                gr::io_signature::make(0, 0, 0),
                gr::io_signature::make(0, 0, 0)),
      d_template_path_(template_path),
      d_sfd_mode_(sfd_mode),
      d_emit_normalized_(emit_normalized),
      d_queue_capacity_(queue_capacity)
{
    if (queue_capacity == 0) {
        throw std::invalid_argument(
            "UwbRadarCirEstimator: queue_capacity must be > 0");
    }
    if (d_template_path_.empty()) {
        throw std::invalid_argument(
            "UwbRadarCirEstimator: template_path is empty");
    }

    d_cfg_.sync_repetitions = sync_repetitions;
    d_cfg_.code_index = code_index;
    d_cfg_.samples_per_symbol = kSps;
    d_cfg_.cir_pre = cir_pre;
    d_cfg_.cir_post = cir_post;
    d_cfg_.cir_skip_initial = cir_skip_initial;
    d_cfg_.sfd_search_margin = sfd_search_margin;
    d_cfg_.sync_refine_margin = sync_refine_margin;
    d_cfg_.sfd_threshold = sfd_threshold;
    d_cfg_.sync_refine_threshold = sync_refine_threshold;
    // Match the demod convention: skip the first settling SYNCs and average
    // every remaining repetition.  For preambles shorter than the skip,
    // average all repetitions instead.
    if (cir_repetitions == 0) {
        if (sync_repetitions > cir_skip_initial) {
            d_cfg_.cir_repetitions = sync_repetitions - cir_skip_initial;
        } else {
            d_cfg_.cir_skip_initial = 0;
            d_cfg_.cir_repetitions = sync_repetitions;
        }
    } else {
        d_cfg_.cir_repetitions = cir_repetitions;
    }

    std::vector<gr_complex> tmpl = load_cf32_file(d_template_path_);
    if (tmpl.size() != kSps) {
        throw std::invalid_argument(
            "UwbRadarCirEstimator: template must hold exactly " +
            std::to_string(kSps) + " samples, got " +
            std::to_string(tmpl.size()) + ": " + d_template_path_);
    }
    if (!radar_meta::sync_reps_supported(d_cfg_.sync_repetitions)) {
        throw std::invalid_argument(
            "UwbRadarCirEstimator: sync_repetitions must be 32, 64 or 128");
    }
    if (!radar_meta::code_index_supported(d_cfg_.code_index)) {
        throw std::invalid_argument(
            "UwbRadarCirEstimator: code_index must be in [9, 12]");
    }
    if (!radar::detail::radar_sfd_mode_known(d_sfd_mode_.c_str())) {
        throw std::invalid_argument("UwbRadarCirEstimator: unknown sfd_mode '" +
                                    d_sfd_mode_ + "'");
    }
    d_cfg_.sfd_mode = d_sfd_mode_.c_str();

    // Freeze the TX-profile identity and preallocate every scratch buffer.
    if (!radar::prepare_radar_cir_core(d_cfg_, tmpl.data(), tmpl.size(),
                                       d_cfg_.cir_pre, d_cfg_.cir_post,
                                       d_scratch_)) {
        throw std::invalid_argument(
            "UwbRadarCirEstimator: failed to prepare radar core from " +
            d_template_path_);
    }

    message_port_register_in(pmt::mp("rx"));
    message_port_register_out(pmt::mp("cir"));
    message_port_register_out(pmt::mp("status"));
    set_msg_handler(pmt::mp("rx"),
                    [this](pmt::pmt_t msg) { handle_rx(msg); });
}

UwbRadarCirEstimator::~UwbRadarCirEstimator()
{
    // Join the worker even if stop() was never called.
    {
        std::lock_guard<std::mutex> lock(d_queue_mutex_);
        d_stop_.store(true, std::memory_order_relaxed);
    }
    d_queue_cv_.notify_all();
    if (d_worker_.joinable())
        d_worker_.join();
}

std::shared_ptr<UwbRadarCirEstimator>
UwbRadarCirEstimator::make(const std::string& template_path,
                           size_t sync_repetitions,
                           const std::string& sfd_mode,
                           size_t code_index,
                           size_t cir_pre,
                           size_t cir_post,
                           size_t cir_skip_initial,
                           size_t cir_repetitions,
                           int64_t sfd_search_margin,
                           int64_t sync_refine_margin,
                           float sfd_threshold,
                           float sync_refine_threshold,
                           bool emit_normalized,
                           size_t queue_capacity)
{
    return gnuradio::get_initial_sptr(new UwbRadarCirEstimator(
        template_path, sync_repetitions, sfd_mode, code_index, cir_pre,
        cir_post, cir_skip_initial, cir_repetitions, sfd_search_margin,
        sync_refine_margin, sfd_threshold, sync_refine_threshold,
        emit_normalized, queue_capacity));
}

// ---------------------------------------------------------------------------
// Stats accessors
// ---------------------------------------------------------------------------

uint64_t UwbRadarCirEstimator::pdus_received() const
{
    return d_received_.load(std::memory_order_relaxed);
}
uint64_t UwbRadarCirEstimator::pdus_enqueued() const
{
    return d_enqueued_.load(std::memory_order_relaxed);
}
uint64_t UwbRadarCirEstimator::pdus_completed() const
{
    return d_completed_.load(std::memory_order_relaxed);
}
uint64_t UwbRadarCirEstimator::pdus_failed() const
{
    return d_failed_.load(std::memory_order_relaxed);
}
uint64_t UwbRadarCirEstimator::pdus_dropped() const
{
    return d_dropped_.load(std::memory_order_relaxed);
}
uint64_t UwbRadarCirEstimator::invalid_inputs() const
{
    return d_invalid_.load(std::memory_order_relaxed);
}
uint64_t UwbRadarCirEstimator::worker_exceptions() const
{
    return d_exceptions_.load(std::memory_order_relaxed);
}
size_t UwbRadarCirEstimator::queue_depth() const
{
    return d_queue_depth_.load(std::memory_order_relaxed);
}
size_t UwbRadarCirEstimator::queue_high_watermark() const
{
    return d_queue_high_watermark_.load(std::memory_order_relaxed);
}

uint64_t UwbRadarCirEstimator::service_mean_us() const
{
    std::lock_guard<std::mutex> lock(d_hist_mutex_);
    uint64_t total = d_service_overflow_;
    for (int i = 0; i < 64; ++i)
        total += d_service_buckets_[i];
    return total ? d_service_sum_us_ / total : 0;
}

uint64_t UwbRadarCirEstimator::service_percentile_us(double pct) const
{
    std::lock_guard<std::mutex> lock(d_hist_mutex_);
    uint64_t total = d_service_overflow_;
    for (int i = 0; i < 64; ++i)
        total += d_service_buckets_[i];
    if (total == 0)
        return 0;
    const uint64_t threshold = std::max<uint64_t>(
        1,
        static_cast<uint64_t>(std::ceil(pct * static_cast<double>(total))));
    uint64_t cum = 0;
    for (int i = 0; i < 64; ++i) {
        cum += d_service_buckets_[i];
        if (cum >= threshold)
            return static_cast<uint64_t>(i + 1) * 64;
    }
    const uint64_t mx = d_service_max_us_.load(std::memory_order_relaxed);
    return mx > 0 ? mx : static_cast<uint64_t>(64) * 64;
}

uint64_t UwbRadarCirEstimator::service_p95_us() const
{
    return service_percentile_us(0.95);
}
uint64_t UwbRadarCirEstimator::service_p99_us() const
{
    return service_percentile_us(0.99);
}
uint64_t UwbRadarCirEstimator::service_max_us() const
{
    return d_service_max_us_.load(std::memory_order_relaxed);
}

void UwbRadarCirEstimator::service_histogram(uint64_t buckets[64],
                                             uint64_t* overflow) const
{
    std::lock_guard<std::mutex> lock(d_hist_mutex_);
    for (int i = 0; i < 64; ++i)
        buckets[i] = d_service_buckets_[i];
    if (overflow)
        *overflow = d_service_overflow_;
}

bool UwbRadarCirEstimator::drained() const
{
    std::lock_guard<std::mutex> lock(d_queue_mutex_);
    return d_queue_.empty() &&
           !d_worker_busy_.load(std::memory_order_relaxed);
}

void UwbRadarCirEstimator::drain()
{
    std::unique_lock<std::mutex> lock(d_queue_mutex_);
    d_queue_cv_.wait(lock, [this] {
        return d_queue_.empty() &&
               !d_worker_busy_.load(std::memory_order_relaxed);
    });
}

void UwbRadarCirEstimator::reset_stats()
{
    d_received_.store(0, std::memory_order_relaxed);
    d_enqueued_.store(0, std::memory_order_relaxed);
    d_completed_.store(0, std::memory_order_relaxed);
    d_failed_.store(0, std::memory_order_relaxed);
    d_dropped_.store(0, std::memory_order_relaxed);
    d_invalid_.store(0, std::memory_order_relaxed);
    d_exceptions_.store(0, std::memory_order_relaxed);
    d_queue_high_watermark_.store(
        d_queue_depth_.load(std::memory_order_relaxed),
        std::memory_order_relaxed);
    d_service_max_us_.store(0, std::memory_order_relaxed);
    std::lock_guard<std::mutex> lock(d_hist_mutex_);
    for (int i = 0; i < 64; ++i)
        d_service_buckets_[i] = 0;
    d_service_overflow_ = 0;
    d_service_sum_us_ = 0;
}

// ---------------------------------------------------------------------------
// start / stop
// ---------------------------------------------------------------------------

bool
UwbRadarCirEstimator::start()
{
    // Join any leftover worker without holding the queue mutex (join under
    // the lock would deadlock a worker blocked on d_queue_mutex_).
    {
        std::lock_guard<std::mutex> lock(d_queue_mutex_);
        d_stop_.store(true, std::memory_order_relaxed);
    }
    d_queue_cv_.notify_all();
    if (d_worker_.joinable())
        d_worker_.join();

    {
        std::lock_guard<std::mutex> lock(d_queue_mutex_);
        d_stop_.store(false, std::memory_order_relaxed);
        d_worker_busy_.store(false, std::memory_order_relaxed);
    }
    d_worker_ = std::thread(&UwbRadarCirEstimator::worker_loop, this);
    return true;
}

bool
UwbRadarCirEstimator::stop()
{
    {
        std::lock_guard<std::mutex> lock(d_queue_mutex_);
        d_stop_.store(true, std::memory_order_relaxed);
    }
    d_queue_cv_.notify_all();
    if (d_worker_.joinable())
        d_worker_.join();
    publish_status("stopped");
    return true;
}

// ---------------------------------------------------------------------------
// Message handler
// ---------------------------------------------------------------------------

void
UwbRadarCirEstimator::handle_rx(pmt::pmt_t msg)
{
    if (!pmt::is_pair(msg)) {
        d_invalid_.fetch_add(1, std::memory_order_relaxed);
        publish_status("invalid_input");
        return;
    }
    pmt::pmt_t meta = pmt::car(msg);
    pmt::pmt_t payload = pmt::cdr(msg);
    if (!pmt::is_dict(meta) || !pmt::is_c32vector(payload)) {
        d_invalid_.fetch_add(1, std::memory_order_relaxed);
        publish_status("invalid_input");
        return;
    }
    const size_t n = pmt::length(payload);
    if (n == 0) {
        d_invalid_.fetch_add(1, std::memory_order_relaxed);
        publish_status("invalid_input");
        return;
    }
    d_received_.fetch_add(1, std::memory_order_relaxed);

    const uint64_t pulse_id = dict_u64(meta, radar_meta::kPulseId, 0);
    auto reject = [&](const char* event, pmt::pmt_t extra = pmt::PMT_NIL) {
        d_invalid_.fetch_add(1, std::memory_order_relaxed);
        pmt::pmt_t m = pmt::make_dict();
        m = pmt::dict_add(m, pmt::mp("event"), pmt::mp(event));
        m = pmt::dict_add(m, pmt::mp("pulse_id"),
                          pmt::from_uint64(pulse_id));
        if (pmt::is_dict(extra)) {
            pmt::pmt_t items = pmt::dict_items(extra);
            for (size_t i = 0; i < pmt::length(items); ++i) {
                pmt::pmt_t kv = pmt::nth(i, items);
                m = pmt::dict_add(m, pmt::car(kv), pmt::cdr(kv));
            }
        }
        message_port_pub(pmt::mp("status"), m);
    };

    // Rate gate: the estimator only accepts 998.4 MS/s work-domain CF32.
    if (!dict_has(meta, "sample_rate")) {
        reject("bad_input_rate");
        return;
    }
    pmt::pmt_t rate_v =
        pmt::dict_ref(meta, pmt::mp("sample_rate"), pmt::PMT_NIL);
    if (!pmt::is_real(rate_v) ||
        !radar_meta::is_work_rate(pmt::to_double(rate_v))) {
        reject("bad_input_rate");
        return;
    }

    // Profile identity must match the prepared TX profile.
    if (!dict_has_str(meta, radar_meta::kSfdMode) ||
        !dict_has(meta, radar_meta::kSyncRepetitions) ||
        !dict_has(meta, radar_meta::kCodeIndex)) {
        reject("invalid_profile");
        return;
    }
    const std::string pdu_sfd = dict_str(meta, radar_meta::kSfdMode);
    const int64_t pdu_sync_reps =
        dict_i64(meta, radar_meta::kSyncRepetitions, -1);
    const int64_t pdu_code =
        dict_i64(meta, radar_meta::kCodeIndex, -1);
    if (pdu_sfd != d_sfd_mode_ ||
        pdu_sync_reps !=
            static_cast<int64_t>(d_cfg_.sync_repetitions) ||
        pdu_code != static_cast<int64_t>(d_cfg_.code_index)) {
        reject("invalid_profile");
        return;
    }

    // Prediction inputs.
    if (!dict_has(meta, radar_meta::kPulseId) ||
        !dict_has(meta, "pre_guard_samples")) {
        reject("invalid_metadata");
        return;
    }
    const int64_t pre_guard =
        dict_i64(meta, "pre_guard_samples", -1);
    double cal_work = 0.0;
    if (pre_guard < 0 || !dict_cal_delay_work(meta, cal_work)) {
        reject("invalid_metadata");
        return;
    }
    // Coordinate convention (65/48 unified): the resampler emits the
    // group-delay-centered window_start_sample = map(window_start_native);
    // pre_guard_samples is mapped as map(ws+pre)-map(ws), i.e. relative to
    // the mapped window start.  The prediction must therefore add the
    // window start back so predicted_sfd lands on the actual content:
    //
    //   predicted_sfd = window_start_sample (0 if absent)
    //                   + pre_guard + round(cal_work)
    //                   + sync_repetitions * 1016
    //
    // Without this term the native chain carries a systematic +map(0)
    // (FIR group-delay head, ~28 work samples for the 2707-tap quality
    // profile) bias that the search margin would otherwise absorb.
    int64_t window_start = 0;
    if (dict_has(meta, "window_start_sample")) {
        window_start = dict_i64(meta, "window_start_sample", -1);
        if (window_start < 0) {
            reject("invalid_metadata");
            return;
        }
    }

    int64_t sync_span = 0;
    int64_t cal_rounded = 0;
    int64_t origin = 0;
    int64_t predicted = 0;
    int64_t zero_tap = -1;
    {
        int64_t sps_i = 0;
        int64_t reps_i = 0;
        int64_t pre_taps = 0;
        int64_t ws_pre = 0;
        if (!radar::radar_i64_from_size(kSps, sps_i) ||
            !radar::radar_i64_from_size(d_cfg_.sync_repetitions, reps_i) ||
            !radar::radar_i64_mul(reps_i, sps_i, sync_span) ||
            !radar::radar_i64_from_size(d_cfg_.cir_pre, pre_taps) ||
            // Checked double→int64 conversion *before* rounding: llround()
            // on values near INT64_MAX is undefined behaviour.
            !cal_round_checked(cal_work, cal_rounded) ||
            !radar::radar_i64_add(window_start, pre_guard, ws_pre) ||
            !radar::radar_i64_add(ws_pre, cal_rounded, origin) ||
            !radar::radar_i64_add(origin, sync_span, predicted) ||
            predicted < 0) {
            reject("invalid_metadata");
            return;
        }
        // zero_delay_tap is the tap where a path at exactly the calibrated
        // delay lands on the core's CIR axis.  The axis origin is the
        // predicted SYNC origin (it includes the calibration delay), so
        // the calibrated reference is cir_pre itself — NOT cir_pre+cal
        // (that bookkeeping belongs to a cal=0-anchored axis and would
        // report the calibrated leakage at a negative delay).
        zero_tap = pre_taps;
    }

    Job job;
    job.pulse_id = pulse_id;
    job.schedule_index =
        static_cast<uint64_t>(std::max<int64_t>(
            0, dict_i64(meta, radar_meta::kScheduleIndex,
                        static_cast<int64_t>(pulse_id))));
    job.predicted_sfd = predicted;
    job.calibration_delay_work = cal_work;
    job.zero_delay_tap = zero_tap;
    job.meta = meta;
    job.samples = payload;
    job.enqueued_at = std::chrono::steady_clock::now();

    if (!enqueue(std::move(job))) {
        d_dropped_.fetch_add(1, std::memory_order_relaxed);
        pmt::pmt_t extra = pmt::make_dict();
        extra = pmt::dict_add(extra, pmt::mp("pulse_id"),
                              pmt::from_uint64(pulse_id));
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
UwbRadarCirEstimator::enqueue(Job&& job)
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
UwbRadarCirEstimator::worker_loop()
{
    d_worker_busy_.store(false, std::memory_order_relaxed);
    for (;;) {
        Job job;
        {
            std::unique_lock<std::mutex> lock(d_queue_mutex_);
            d_worker_busy_.store(false, std::memory_order_relaxed);
            d_queue_cv_.notify_all(); // wake drain() waiters
            d_queue_cv_.wait(lock, [this] {
                return d_stop_.load(std::memory_order_relaxed) ||
                       !d_queue_.empty();
            });
            if (d_queue_.empty() &&
                d_stop_.load(std::memory_order_relaxed)) {
                d_worker_busy_.store(false, std::memory_order_relaxed);
                return;
            }
            d_worker_busy_.store(true, std::memory_order_relaxed);
            job = std::move(d_queue_.front());
            d_queue_.pop_front();
            d_queue_depth_.store(d_queue_.size(), std::memory_order_relaxed);
        }

        size_t n = 0;
        const gr_complex* rx =
            pmt::c32vector_elements(job.samples, n);

        const auto t0 = std::chrono::steady_clock::now();
        radar::RadarCirResult r;
        bool threw = false;
        std::string what;
        try {
            radar::radar_cir_one(rx, n, job.predicted_sfd, d_cfg_,
                                 d_scratch_, r);
        } catch (const std::exception& e) {
            threw = true;
            what = e.what();
        } catch (...) {
            threw = true;
            what = "unknown exception";
        }
        const auto t1 = std::chrono::steady_clock::now();
        const uint64_t queue_us = elapsed_us(job.enqueued_at, t0);
        const uint64_t service_us = elapsed_us(t0, t1);
        record_service_time(service_us);

        if (threw) {
            d_exceptions_.fetch_add(1, std::memory_order_relaxed);
            radar::RadarCirResult err;
            err.status = radar::RadarCirStatus::InvalidInput;
            pmt::pmt_t extra = pmt::make_dict();
            extra = pmt::dict_add(extra, pmt::mp("what"),
                                  pmt::string_to_symbol(what));
            publish_status("worker_exception", extra);
            publish_frame(job, err, queue_us, service_us);
            d_queue_cv_.notify_all();
            continue;
        }

        if (r.status == radar::RadarCirStatus::Ok)
            d_completed_.fetch_add(1, std::memory_order_relaxed);
        else
            d_failed_.fetch_add(1, std::memory_order_relaxed);

        publish_frame(job, r, queue_us, service_us);
        d_queue_cv_.notify_all();
    }
}

const char*
UwbRadarCirEstimator::status_to_string(radar::RadarCirStatus s)
{
    using gr::uwb::radar::RadarCirStatus;
    switch (s) {
    case RadarCirStatus::Ok:
        return "ok";
    case RadarCirStatus::SfdFailed:
        return "sfd_failed";
    case RadarCirStatus::TimingFailed:
        return "timing_failed";
    case RadarCirStatus::CirFailed:
        return "cir_failed";
    case RadarCirStatus::InvalidInput:
        return "invalid_input";
    }
    return "internal_error";
}

void
UwbRadarCirEstimator::publish_frame(const Job& job,
                                    const radar::RadarCirResult& r,
                                    uint64_t queue_us,
                                    uint64_t service_us)
{
    const bool ok = r.status == radar::RadarCirStatus::Ok;
    pmt::pmt_t meta = pmt::make_dict();
    copy_lineage(meta, job.meta);

    meta = pmt::dict_add(meta, pmt::mp("status"),
                         pmt::mp(status_to_string(r.status)));
    meta = pmt::dict_add(meta, pmt::mp("status_code"),
                         pmt::from_long(static_cast<long>(r.status)));
    meta = pmt::dict_add(meta, pmt::mp("pulse_id"),
                         pmt::from_uint64(job.pulse_id));
    meta = pmt::dict_add(meta, pmt::mp("schedule_index"),
                         pmt::from_uint64(job.schedule_index));

    meta = pmt::dict_add(meta, pmt::mp("predicted_sfd_start_sample"),
                         pmt::from_long(r.predicted_sfd_start));
    meta = pmt::dict_add(meta, pmt::mp("sfd_start_sample"),
                         pmt::from_long(r.sfd_start_sample));
    meta = pmt::dict_add(meta, pmt::mp("preamble_start_sample"),
                         pmt::from_long(r.preamble_start_sample));
    meta = pmt::dict_add(meta, pmt::mp("cir_origin_sample"),
                         pmt::from_long(r.cir_origin_sample));
    meta = pmt::dict_add(meta, pmt::mp("sfd_metric"),
                         pmt::from_double(r.sfd_metric));
    meta = pmt::dict_add(meta, pmt::mp("sync_metric"),
                         pmt::from_double(r.sync_metric));

    const size_t tap_count = ok ? r.tap_count : 0;
    meta = pmt::dict_add(meta, pmt::mp("tap_count"),
                         pmt::from_uint64(static_cast<uint64_t>(tap_count)));
    meta = pmt::dict_add(meta, pmt::mp("cir_pre_samples"),
                         pmt::from_uint64(static_cast<uint64_t>(
                             d_cfg_.cir_pre)));
    meta = pmt::dict_add(meta, pmt::mp("cir_post_samples"),
                         pmt::from_uint64(static_cast<uint64_t>(
                             d_cfg_.cir_post)));
    meta = pmt::dict_add(meta, pmt::mp("peak_tap"),
                         pmt::from_uint64(ok
                                              ? static_cast<uint64_t>(
                                                    r.peak_tap)
                                              : uint64_t(0)));
    meta = pmt::dict_add(meta, pmt::mp("cir_peak_metric"),
                         pmt::from_double(ok ? r.peak_abs : 0.0));
    meta = pmt::dict_add(meta, pmt::mp("raw_l2_norm"),
                         pmt::from_double(ok ? r.raw_l2_norm : 0.0));
    meta = pmt::dict_add(meta, pmt::mp("valid_repetitions"),
                         pmt::from_uint64(ok
                                              ? static_cast<uint64_t>(
                                                    r.valid_repetitions)
                                              : uint64_t(0)));

    meta = pmt::dict_add(meta, pmt::mp("zero_delay_tap"),
                         pmt::from_long(job.zero_delay_tap));
    meta = pmt::dict_add(meta, pmt::mp("calibration_delay_work_samples"),
                         pmt::from_double(job.calibration_delay_work));
    meta = pmt::dict_add(
        meta, pmt::mp("range_m_per_tap"),
        pmt::from_double(kSpeedOfLight /
                         (2.0 * radar_meta::kWorkRateHz)));
    meta = pmt::dict_add(meta, pmt::mp("sfd_ok"),
                         r.sfd_start_sample >= 0 ? pmt::PMT_T : pmt::PMT_F);
    meta = pmt::dict_add(
        meta, pmt::mp("timing_ok"),
        r.preamble_start_sample >= 0 ? pmt::PMT_T : pmt::PMT_F);
    meta = pmt::dict_add(meta, pmt::mp("queue_delay_us"),
                         pmt::from_uint64(queue_us));
    meta = pmt::dict_add(meta, pmt::mp("estimator_us"),
                         pmt::from_uint64(service_us));

    pmt::pmt_t vec;
    if (ok && tap_count > 0) {
        vec = pmt::init_c32vector(tap_count,
                                  d_scratch_.cir.raw_taps.data());
        if (d_emit_normalized_) {
            meta = pmt::dict_add(
                meta, pmt::mp("normalized_taps"),
                pmt::init_c32vector(tap_count,
                                    d_scratch_.cir.norm_taps.data()));
        }
    } else {
        vec = pmt::init_c32vector(0, static_cast<const gr_complex*>(nullptr));
    }

    message_port_pub(pmt::mp("cir"), pmt::cons(meta, vec));
}

void
UwbRadarCirEstimator::snapshot_stats(pmt::pmt_t& meta)
{
    meta = pmt::dict_add(meta, pmt::mp("pdus_received"),
                         pmt::from_uint64(pdus_received()));
    meta = pmt::dict_add(meta, pmt::mp("pdus_enqueued"),
                         pmt::from_uint64(pdus_enqueued()));
    meta = pmt::dict_add(meta, pmt::mp("pdus_completed"),
                         pmt::from_uint64(pdus_completed()));
    meta = pmt::dict_add(meta, pmt::mp("pdus_failed"),
                         pmt::from_uint64(pdus_failed()));
    meta = pmt::dict_add(meta, pmt::mp("pdus_dropped"),
                         pmt::from_uint64(pdus_dropped()));
    meta = pmt::dict_add(meta, pmt::mp("invalid_inputs"),
                         pmt::from_uint64(invalid_inputs()));
    meta = pmt::dict_add(meta, pmt::mp("worker_exceptions"),
                         pmt::from_uint64(worker_exceptions()));
    meta = pmt::dict_add(meta, pmt::mp("queue_depth"),
                         pmt::from_uint64(
                             static_cast<uint64_t>(queue_depth())));
    meta = pmt::dict_add(
        meta, pmt::mp("queue_high_watermark"),
        pmt::from_uint64(
            static_cast<uint64_t>(queue_high_watermark())));
    meta = pmt::dict_add(meta, pmt::mp("service_mean_us"),
                         pmt::from_uint64(service_mean_us()));
    meta = pmt::dict_add(meta, pmt::mp("service_p95_us"),
                         pmt::from_uint64(service_p95_us()));
    meta = pmt::dict_add(meta, pmt::mp("service_p99_us"),
                         pmt::from_uint64(service_p99_us()));
    meta = pmt::dict_add(meta, pmt::mp("service_max_us"),
                         pmt::from_uint64(service_max_us()));
}

void
UwbRadarCirEstimator::publish_status(const std::string& event,
                                     pmt::pmt_t extra)
{
    pmt::pmt_t meta = pmt::make_dict();
    meta = pmt::dict_add(meta, pmt::mp("event"), pmt::mp(event));
    snapshot_stats(meta);
    if (!pmt::eq(extra, pmt::PMT_NIL) && pmt::is_dict(extra)) {
        pmt::pmt_t items = pmt::dict_items(extra);
        for (size_t i = 0; i < pmt::length(items); ++i) {
            pmt::pmt_t kv = pmt::nth(i, items);
            meta = pmt::dict_add(meta, pmt::car(kv), pmt::cdr(kv));
        }
    }
    message_port_pub(pmt::mp("status"), meta);
}

void
UwbRadarCirEstimator::record_service_time(uint64_t us)
{
    {
        std::lock_guard<std::mutex> lock(d_hist_mutex_);
        const size_t bin = static_cast<size_t>(us / 64);
        if (bin >= 64)
            ++d_service_overflow_;
        else
            ++d_service_buckets_[bin];
        d_service_sum_us_ += us;
    }
    uint64_t cur = d_service_max_us_.load(std::memory_order_relaxed);
    while (us > cur &&
           !d_service_max_us_.compare_exchange_weak(cur, us,
                                                    std::memory_order_relaxed)) {
        // retry
    }
}

} // namespace uwb
} // namespace gr
