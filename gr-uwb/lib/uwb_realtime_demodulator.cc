/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * UwbRealtimeDemodulator — async worker-pool PDU demodulator (R5).
 *
 * Style mirrors UwbScheduledExtractor: bounded queue, message_port_pub from
 * worker threads, publish_status snapshots, stop() drains then joins.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gnuradio/uwb/uwb_realtime_demodulator.h>
#include <gnuradio/io_signature.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <stdexcept>
#include <utility>

namespace gr {
namespace uwb {

namespace {

// Detector/scheduled-extractor production window: 9,984 pre-trigger plus
// 309,184 capture samples. Reserve every worker before start() so the normal
// hot path never grows its scratch vectors. Larger diagnostic PDUs still work
// and fall back to the core's existing capacity growth.
constexpr size_t kDefaultScratchSamples = 319168;

gr::uwb::demod::CirSoftChipMode parse_cir_filter_mode(
    const std::string& mode, size_t rake_top_k)
{
    using Mode = gr::uwb::demod::CirSoftChipMode;
    if (mode == "auto")
        return Mode::Auto;
    if (mode == "full")
        return Mode::Full;
    if (mode == "bypass")
        return Mode::Bypass;
    if (mode == "rake") {
        if (rake_top_k == 0 || rake_top_k >= 38)
            throw std::invalid_argument(
                "UwbRealtimeDemodulator: rake mode requires Top-K in [1,37]");
        return Mode::Rake;
    }
    throw std::invalid_argument(
        "UwbRealtimeDemodulator: cir_filter_mode must be auto, full, rake, or bypass");
}

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
        return static_cast<int64_t>(pmt::to_long(v));
    return def;
}

double
dict_f64(pmt::pmt_t dict, const char* key, double def)
{
    if (!pmt::is_dict(dict))
        return def;
    pmt::pmt_t v = pmt::dict_ref(dict, pmt::mp(key), pmt::from_double(def));
    if (pmt::is_real(v) || pmt::is_integer(v) || pmt::is_uint64(v))
        return pmt::to_double(v);
    return def;
}

} // namespace

// ---------------------------------------------------------------------------
// Construction / factory
// ---------------------------------------------------------------------------

std::vector<gr_complex>
UwbRealtimeDemodulator::load_cf32_file(const std::string& path)
{
    std::ifstream f(path, std::ios::binary);
    if (!f) {
        throw std::invalid_argument(
            "UwbRealtimeDemodulator: cannot open template file " + path);
    }
    f.seekg(0, std::ios::end);
    const std::streamoff bytes = f.tellg();
    f.seekg(0, std::ios::beg);
    if (bytes <= 0 ||
        bytes % static_cast<std::streamoff>(sizeof(gr_complex)) != 0) {
        throw std::invalid_argument(
            "UwbRealtimeDemodulator: template file empty or invalid size: " +
            path);
    }
    std::vector<gr_complex> tmpl(static_cast<size_t>(bytes) /
                                 sizeof(gr_complex));
    f.read(reinterpret_cast<char*>(tmpl.data()), bytes);
    if (!f && !f.eof()) {
        throw std::invalid_argument(
            "UwbRealtimeDemodulator: failed reading template file " + path);
    }
    if (tmpl.empty()) {
        throw std::invalid_argument(
            "UwbRealtimeDemodulator: template file is empty: " + path);
    }
    return tmpl;
}

UwbRealtimeDemodulator::UwbRealtimeDemodulator(
    const std::vector<gr_complex>& template_wf,
    size_t num_workers,
    size_t queue_capacity,
    const std::string& sfd_mode,
    size_t cir_rake_top_k,
    const std::string& cir_filter_mode,
    size_t code_index,
    size_t preamble_repetitions,
    size_t timing_coarse_stride)
    : gr::block("uwb_realtime_demodulator",
                gr::io_signature::make(0, 0, 0),
                gr::io_signature::make(0, 0, 0)),
      d_template_wf_(template_wf),
      d_sfd_mode_(sfd_mode),
      d_profile_(gr::uwb::demod::Qm35825Profile::Default()),
      d_num_workers_(num_workers),
      d_queue_capacity_(queue_capacity),
      d_scratch_(num_workers),
      d_worker_busy_us_(num_workers, 0),
      d_worker_total_us_(num_workers, 0)
{
    if (num_workers == 0) {
        throw std::invalid_argument(
            "UwbRealtimeDemodulator: num_workers must be > 0");
    }
    if (queue_capacity == 0) {
        throw std::invalid_argument(
            "UwbRealtimeDemodulator: queue_capacity must be > 0");
    }
    if (d_template_wf_.empty()) {
        throw std::invalid_argument(
            "UwbRealtimeDemodulator: template waveform is empty");
    }
    if (code_index < 9 || code_index > 12) {
        throw std::invalid_argument(
            "UwbRealtimeDemodulator: unsupported code_index " +
            std::to_string(code_index) + " (supported: 9, 10, 11, 12)");
    }
    if (preamble_repetitions == 0) {
        throw std::invalid_argument(
            "UwbRealtimeDemodulator: preamble_repetitions must be > 0");
    }
    if (timing_coarse_stride == 0) {
        throw std::invalid_argument(
            "UwbRealtimeDemodulator: timing_coarse_stride must be > 0");
    }
    d_profile_.code_index = code_index;
    d_profile_.preamble_repetitions = preamble_repetitions;
    d_profile_.timing_coarse_stride = timing_coarse_stride;
    // Match UWB_demodulation/run_decode_uwb_all.m: explicitly skip the first
    // ten settling SYNCs and estimate CIR from every remaining repetition.
    // For unusually short preambles, retain all repetitions instead.
    if (preamble_repetitions > d_profile_.cir_skip_initial_repetitions) {
        d_profile_.cir_repetitions =
            preamble_repetitions - d_profile_.cir_skip_initial_repetitions;
    } else {
        d_profile_.cir_skip_initial_repetitions = 0;
        d_profile_.cir_repetitions = preamble_repetitions;
    }
    // sfd_mode selects the SFD template used by stages 3/5.  The golden
    // MATLAB generator (lrwpan default) emits "ieee"; QM35825 uses "4z2".
    // Store the mode in the owned member so d_profile_.sfd_mode (a char*)
    // does not dangle after construction.
    if (gr::uwb::demod::GetSfdSequence(d_sfd_mode_.c_str()).empty()) {
        throw std::invalid_argument(
            "UwbRealtimeDemodulator: unknown sfd_mode '" + d_sfd_mode_ + "'");
    }
    d_profile_.sfd_mode = d_sfd_mode_.c_str();
    if (cir_rake_top_k > 64) {
        throw std::invalid_argument(
            "UwbRealtimeDemodulator: cir_rake_top_k must be <= 64");
    }
    d_profile_.cir_rake_top_k = cir_rake_top_k;
    d_profile_.cir_soft_chip_mode =
        parse_cir_filter_mode(cir_filter_mode, cir_rake_top_k);

    for (auto& scratch : d_scratch_)
        scratch.reserve(kDefaultScratchSamples);

    message_port_register_in(pmt::mp("samples"));
    message_port_register_in(pmt::mp("control"));
    message_port_register_out(pmt::mp("result"));
    message_port_register_out(pmt::mp("status"));
    // FCS-pass observations for UwbScheduledExtractor "lock_obs" (schedule lock).
    message_port_register_out(pmt::mp("schedule_feedback"));

    set_msg_handler(pmt::mp("samples"),
                    [this](pmt::pmt_t msg) { handle_samples(msg); });
    set_msg_handler(pmt::mp("control"),
                    [this](pmt::pmt_t msg) { handle_control(msg); });
}

UwbRealtimeDemodulator::~UwbRealtimeDemodulator()
{
    // Ensure workers are joined even if stop() was never called.
    {
        std::lock_guard<std::mutex> lock(d_queue_mutex_);
        d_stop_.store(true, std::memory_order_relaxed);
    }
    d_queue_cv_.notify_all();
    for (auto& t : d_workers_) {
        if (t.joinable())
            t.join();
    }
    d_workers_.clear();
}

std::shared_ptr<UwbRealtimeDemodulator>
UwbRealtimeDemodulator::make(const std::string& template_path,
                             size_t num_workers,
                             size_t queue_capacity,
                             const std::string& sfd_mode,
                             size_t cir_rake_top_k,
                             const std::string& cir_filter_mode,
                             size_t code_index,
                             size_t preamble_repetitions,
                             size_t timing_coarse_stride)
{
    auto tmpl = load_cf32_file(template_path);
    return gnuradio::get_initial_sptr(new UwbRealtimeDemodulator(
        tmpl, num_workers, queue_capacity, sfd_mode, cir_rake_top_k,
        cir_filter_mode, code_index, preamble_repetitions,
        timing_coarse_stride));
}

std::shared_ptr<UwbRealtimeDemodulator>
UwbRealtimeDemodulator::make_from_template(
    const std::vector<gr_complex>& template_wf,
    size_t num_workers,
    size_t queue_capacity,
    const std::string& sfd_mode,
    size_t cir_rake_top_k,
    const std::string& cir_filter_mode,
    size_t code_index,
    size_t preamble_repetitions,
    size_t timing_coarse_stride)
{
    return gnuradio::get_initial_sptr(new UwbRealtimeDemodulator(
        template_wf, num_workers, queue_capacity, sfd_mode, cir_rake_top_k,
        cir_filter_mode, code_index, preamble_repetitions,
        timing_coarse_stride));
}

// ---------------------------------------------------------------------------
// Stats accessors
// ---------------------------------------------------------------------------

uint64_t
UwbRealtimeDemodulator::jobs_received() const
{
    return d_jobs_received_.load(std::memory_order_relaxed);
}
uint64_t
UwbRealtimeDemodulator::jobs_completed() const
{
    return d_jobs_completed_.load(std::memory_order_relaxed);
}
uint64_t
UwbRealtimeDemodulator::jobs_failed() const
{
    return d_jobs_failed_.load(std::memory_order_relaxed);
}
uint64_t
UwbRealtimeDemodulator::jobs_dropped() const
{
    return d_jobs_dropped_.load(std::memory_order_relaxed);
}
uint64_t
UwbRealtimeDemodulator::invalid_inputs() const
{
    return d_invalid_inputs_.load(std::memory_order_relaxed);
}
uint64_t
UwbRealtimeDemodulator::worker_exceptions() const
{
    return d_worker_exceptions_.load(std::memory_order_relaxed);
}
size_t
UwbRealtimeDemodulator::queue_depth() const
{
    return d_queue_depth_.load(std::memory_order_relaxed);
}
size_t
UwbRealtimeDemodulator::queue_high_watermark() const
{
    return d_queue_high_watermark_.load(std::memory_order_relaxed);
}
size_t
UwbRealtimeDemodulator::num_workers() const
{
    return d_num_workers_;
}
size_t
UwbRealtimeDemodulator::timing_coarse_stride() const
{
    return d_profile_.timing_coarse_stride;
}
uint64_t
UwbRealtimeDemodulator::latency_p50_us() const
{
    return latency_percentile_us(0.50);
}
uint64_t
UwbRealtimeDemodulator::latency_p95_us() const
{
    return latency_percentile_us(0.95);
}
uint64_t
UwbRealtimeDemodulator::latency_p99_us() const
{
    return latency_percentile_us(0.99);
}
uint64_t
UwbRealtimeDemodulator::latency_max_us() const
{
    return d_latency_max_us_.load(std::memory_order_relaxed);
}

void
UwbRealtimeDemodulator::latency_histogram(uint64_t buckets[64],
                                          uint64_t* overflow) const
{
    std::lock_guard<std::mutex> lock(d_hist_mutex_);
    for (int i = 0; i < 64; ++i)
        buckets[i] = d_latency_buckets_[i];
    if (overflow)
        *overflow = d_latency_overflow_;
}

double
UwbRealtimeDemodulator::worker_utilization_pct() const
{
    std::lock_guard<std::mutex> lock(d_worker_stats_mutex_);
    uint64_t sum_busy = 0;
    uint64_t sum_total = 0;
    for (size_t i = 0; i < d_num_workers_; ++i) {
        sum_busy += d_worker_busy_us_[i];
        sum_total += d_worker_total_us_[i];
    }
    const uint64_t denom = std::max<uint64_t>(1, sum_total);
    return 100.0 * static_cast<double>(sum_busy) /
           static_cast<double>(denom);
}

bool
UwbRealtimeDemodulator::drained() const
{
    std::lock_guard<std::mutex> lock(d_queue_mutex_);
    return d_queue_.empty() &&
           d_idle_workers_.load(std::memory_order_relaxed) == d_num_workers_;
}

void
UwbRealtimeDemodulator::drain()
{
    std::unique_lock<std::mutex> lock(d_queue_mutex_);
    d_queue_cv_.wait(lock, [this] {
        return d_queue_.empty() &&
               d_idle_workers_.load(std::memory_order_relaxed) ==
                   d_num_workers_;
    });
}

void
UwbRealtimeDemodulator::reset_stats()
{
    d_jobs_received_.store(0, std::memory_order_relaxed);
    d_jobs_completed_.store(0, std::memory_order_relaxed);
    d_jobs_failed_.store(0, std::memory_order_relaxed);
    d_jobs_dropped_.store(0, std::memory_order_relaxed);
    d_invalid_inputs_.store(0, std::memory_order_relaxed);
    d_worker_exceptions_.store(0, std::memory_order_relaxed);
    d_queue_high_watermark_.store(
        d_queue_depth_.load(std::memory_order_relaxed),
        std::memory_order_relaxed);
    d_latency_max_us_.store(0, std::memory_order_relaxed);
    {
        std::lock_guard<std::mutex> lock(d_hist_mutex_);
        for (int i = 0; i < 64; ++i)
            d_latency_buckets_[i] = 0;
        d_latency_overflow_ = 0;
    }
    {
        std::lock_guard<std::mutex> lock(d_worker_stats_mutex_);
        std::fill(d_worker_busy_us_.begin(), d_worker_busy_us_.end(), 0);
        std::fill(d_worker_total_us_.begin(), d_worker_total_us_.end(), 0);
    }
    {
        std::lock_guard<std::mutex> lock(d_stage_mutex_);
        std::fill(std::begin(d_stage_sums_), std::end(d_stage_sums_), 0);
        d_stage_n_ = 0;
    }
}

// ---------------------------------------------------------------------------
// start / stop
// ---------------------------------------------------------------------------

bool
UwbRealtimeDemodulator::start()
{
    // Join any leftover workers without holding the queue mutex (join under
    // the lock would deadlock a worker blocked on d_queue_mutex_).
    {
        std::lock_guard<std::mutex> lock(d_queue_mutex_);
        d_stop_.store(true, std::memory_order_relaxed);
    }
    d_queue_cv_.notify_all();
    for (auto& t : d_workers_) {
        if (t.joinable())
            t.join();
    }
    d_workers_.clear();

    {
        std::lock_guard<std::mutex> lock(d_queue_mutex_);
        d_stop_.store(false, std::memory_order_relaxed);
        d_idle_workers_.store(0, std::memory_order_relaxed);
    }
    d_workers_.reserve(d_num_workers_);
    for (size_t i = 0; i < d_num_workers_; ++i) {
        d_workers_.emplace_back(&UwbRealtimeDemodulator::worker_loop, this, i);
    }
    return true;
}

bool
UwbRealtimeDemodulator::stop()
{
    {
        std::lock_guard<std::mutex> lock(d_queue_mutex_);
        d_stop_.store(true, std::memory_order_relaxed);
    }
    d_queue_cv_.notify_all();
    for (auto& t : d_workers_) {
        if (t.joinable())
            t.join();
    }
    d_workers_.clear();
    publish_status("stopped");
    return true;
}

// ---------------------------------------------------------------------------
// Message handlers
// ---------------------------------------------------------------------------

namespace {

// Convert interleaved s16 I/Q (I0,Q0,I1,Q1,…) to a c32vector with scale
// 1/32767.  Returns PMT_NIL on odd length / empty input.
pmt::pmt_t s16_interleaved_to_c32(pmt::pmt_t s16v)
{
    if (!pmt::is_s16vector(s16v))
        return pmt::PMT_NIL;
    const size_t n16 = pmt::length(s16v);
    if (n16 == 0 || (n16 & 1u) != 0)
        return pmt::PMT_NIL;
    const size_t n = n16 / 2;
    const std::vector<int16_t>& elems = pmt::s16vector_elements(s16v);
    // Build CF32 in a temporary; pmt::init_c32vector copies into the PMT.
    std::vector<gr_complex> cf(n);
    constexpr float kScale = 1.0f / 32767.0f;
    for (size_t i = 0; i < n; ++i) {
        cf[i] = gr_complex(static_cast<float>(elems[2 * i]) * kScale,
                           static_cast<float>(elems[2 * i + 1]) * kScale);
    }
    return pmt::init_c32vector(n, cf.data());
}

} // namespace

void
UwbRealtimeDemodulator::handle_samples(pmt::pmt_t msg)
{
    // Accept:
    //   cons(meta, c32vector)  — primary CF32 path
    //   cons(meta, s16vector)  — interleaved int16 I/Q → CF32
    //   plain s16vector        — interleaved int16 I/Q, empty meta
    pmt::pmt_t meta = pmt::PMT_NIL;
    pmt::pmt_t samples = pmt::PMT_NIL;

    if (pmt::is_pair(msg)) {
        meta = pmt::car(msg);
        pmt::pmt_t payload = pmt::cdr(msg);
        if (pmt::is_c32vector(payload)) {
            samples = payload;
        } else if (pmt::is_s16vector(payload)) {
            samples = s16_interleaved_to_c32(payload);
            if (pmt::is_null(samples)) {
                d_invalid_inputs_.fetch_add(1, std::memory_order_relaxed);
                publish_status("invalid_input");
                return;
            }
        } else {
            d_invalid_inputs_.fetch_add(1, std::memory_order_relaxed);
            publish_status("invalid_input");
            return;
        }
    } else if (pmt::is_s16vector(msg)) {
        meta = pmt::make_dict();
        samples = s16_interleaved_to_c32(msg);
        if (pmt::is_null(samples)) {
            d_invalid_inputs_.fetch_add(1, std::memory_order_relaxed);
            publish_status("invalid_input");
            return;
        }
    } else {
        d_invalid_inputs_.fetch_add(1, std::memory_order_relaxed);
        publish_status("invalid_input");
        return;
    }

    if (!pmt::is_dict(meta))
        meta = pmt::make_dict();

    const size_t n = pmt::length(samples);
    if (n == 0) {
        d_invalid_inputs_.fetch_add(1, std::memory_order_relaxed);
        publish_status("invalid_input");
        return;
    }

    Job job;
    job.packet_id = dict_u64(meta, "packet_id", 0);
    job.schedule_index = dict_u64(meta, "schedule_index", 0);
    // Seed stage_timing from the schedule prediction.  A large coarse search
    // margin absorbs t0 bias and ppm-level SFO; optional detected_start is
    // diagnostics-only (and can land on a strong mid-preamble SYNC).
    job.predicted_start_sample =
        dict_i64(meta, "predicted_start_sample", -1);
    job.window_start_sample = dict_i64(meta, "window_start_sample", 0);
    job.sample_count =
        dict_i64(meta, "sample_count", static_cast<int64_t>(n));
    job.sample_rate = dict_f64(meta, "sample_rate", 0.0);
    // Provenance from the PDU 65/48 resampler (for schedule_feedback mapping).
    job.native_sample_rate = dict_f64(meta, "input_sample_rate", 0.0);
    if (job.native_sample_rate <= 0.0)
        job.native_sample_rate =
            dict_f64(meta, "native_sample_rate", 0.0);
    job.resample_filter_delay =
        dict_f64(meta, "resample_filter_delay", 0.0);
    job.resample_us = dict_u64(meta, "resample_us", 0);
    job.samples = samples; // always c32vector after optional s16 conversion
    job.enqueued_at = std::chrono::steady_clock::now();

    d_jobs_received_.fetch_add(1, std::memory_order_relaxed);

    if (!enqueue(std::move(job))) {
        d_jobs_dropped_.fetch_add(1, std::memory_order_relaxed);
        pmt::pmt_t extra = pmt::make_dict();
        extra = pmt::dict_add(extra, pmt::mp("packet_id"),
                              pmt::from_uint64(dict_u64(meta, "packet_id", 0)));
        extra = pmt::dict_add(
            extra, pmt::mp("queue_depth"),
            pmt::from_uint64(static_cast<uint64_t>(
                d_queue_depth_.load(std::memory_order_relaxed))));
        publish_status("queue_full", extra);
        return;
    }
    d_queue_cv_.notify_one();
}

void
UwbRealtimeDemodulator::handle_control(pmt::pmt_t msg)
{
    // Symbol: reset_stats
    if (pmt::is_symbol(msg)) {
        if (pmt::symbol_to_string(msg) == "reset_stats")
            reset_stats();
        return;
    }

    if (!pmt::is_dict(msg))
        return;

    pmt::pmt_t cmd_p = pmt::dict_ref(msg, pmt::mp("cmd"), pmt::PMT_NIL);
    if (!pmt::is_symbol(cmd_p))
        return;
    const std::string cmd = pmt::symbol_to_string(cmd_p);

    if (cmd == "reset_stats") {
        reset_stats();
        return;
    }
    if (cmd == "drain") {
        drain();
        return;
    }
    if (cmd == "fail_packet") {
        // TEST-ONLY fault injection.
        const uint64_t pid = dict_u64(msg, "packet_id", 0);
        std::lock_guard<std::mutex> lock(d_fail_mutex_);
        d_fail_packet_id_ = pid;
        return;
    }
}

bool
UwbRealtimeDemodulator::enqueue(Job&& job)
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
UwbRealtimeDemodulator::worker_loop(size_t wid)
{
    const auto worker_start = std::chrono::steady_clock::now();
    gr::uwb::demod::core::DemodScratch& scratch = d_scratch_[wid];

    for (;;) {
        Job job;
        {
            std::unique_lock<std::mutex> lock(d_queue_mutex_);
            d_idle_workers_.fetch_add(1, std::memory_order_relaxed);
            d_queue_cv_.notify_all(); // wake drain() waiters
            d_queue_cv_.wait(lock, [this] {
                return d_stop_.load(std::memory_order_relaxed) ||
                       !d_queue_.empty();
            });
            if (d_queue_.empty() &&
                d_stop_.load(std::memory_order_relaxed)) {
                // Stay counted as idle so drained() is true after stop().
                return;
            }
            d_idle_workers_.fetch_sub(1, std::memory_order_relaxed);
            job = std::move(d_queue_.front());
            d_queue_.pop_front();
            d_queue_depth_.store(d_queue_.size(), std::memory_order_relaxed);
        }

        // Keep job.samples alive for the entire decode and borrow its uniform
        // vector storage directly. The vector-returning overload copies the
        // full PDU (2.55 MB for the standard 319,168-sample window) per job.
        size_t n = 0;
        const gr_complex* rx = pmt::c32vector_elements(job.samples, n);

        const auto t0 = std::chrono::steady_clock::now();
        gr::uwb::demod::DemodResult r;
        bool threw = false;
        std::string what;

        try {
            // TEST-ONLY fault injection.
            {
                std::lock_guard<std::mutex> fl(d_fail_mutex_);
                if (d_fail_packet_id_.has_value() &&
                    job.packet_id == *d_fail_packet_id_) {
                    throw std::runtime_error("injected fault");
                }
            }

            r = gr::uwb::demod::core::demodulate_one(
                rx, n, d_profile_, job.packet_id, job.predicted_start_sample,
                job.window_start_sample, d_template_wf_, scratch);
        } catch (const std::exception& e) {
            threw = true;
            what = e.what();
        } catch (...) {
            threw = true;
            what = "unknown exception";
        }

        const auto t1 = std::chrono::steady_clock::now();
        const uint64_t queue_us = elapsed_us(job.enqueued_at, t0);
        const uint64_t demod_us = elapsed_us(t0, t1);
        const uint64_t wall_us = elapsed_us(job.enqueued_at, t1);

        {
            std::lock_guard<std::mutex> lock(d_worker_stats_mutex_);
            d_worker_busy_us_[wid] += demod_us;
            d_worker_total_us_[wid] = elapsed_us(worker_start, t1);
        }

        if (threw) {
            d_worker_exceptions_.fetch_add(1, std::memory_order_relaxed);
            d_jobs_failed_.fetch_add(1, std::memory_order_relaxed);

            pmt::pmt_t extra = pmt::make_dict();
            extra = pmt::dict_add(extra, pmt::mp("packet_id"),
                                  pmt::from_uint64(job.packet_id));
            extra = pmt::dict_add(extra, pmt::mp("what"),
                                  pmt::string_to_symbol(what));
            publish_status("worker_exception", extra);

            gr::uwb::demod::DemodResult err;
            err.status = gr::uwb::demod::DemodStatus::InternalError;
            err.packet_id = job.packet_id;
            err.schedule_index = job.schedule_index;
            err.predicted_start_sample = job.predicted_start_sample;
            err.window_start_sample = job.window_start_sample;
            err.input_sample_count = n;
            err.queue_delay_us = queue_us;
            err.demod_latency_us = demod_us;
            err.wall_latency_us = wall_us;
            err.worker_id = static_cast<uint32_t>(wid);
            err.resample_us = job.resample_us;
            record_latency(wall_us);
            publish_result(job, err);
            d_queue_cv_.notify_all();
            continue;
        }

        r.schedule_index = job.schedule_index;
        r.input_sample_count = n;
        r.queue_delay_us = queue_us;
        r.demod_latency_us = demod_us;
        r.wall_latency_us = wall_us;
        r.worker_id = static_cast<uint32_t>(wid);
        r.resample_us = job.resample_us;

        if (r.status == gr::uwb::demod::DemodStatus::Success)
            d_jobs_completed_.fetch_add(1, std::memory_order_relaxed);
        else
            d_jobs_failed_.fetch_add(1, std::memory_order_relaxed);

        record_latency(r.wall_latency_us);
        publish_result(job, r);
        d_queue_cv_.notify_all();
    }
}

// ---------------------------------------------------------------------------
// Publish helpers
// ---------------------------------------------------------------------------

const char*
UwbRealtimeDemodulator::status_to_string(gr::uwb::demod::DemodStatus s)
{
    using gr::uwb::demod::DemodStatus;
    switch (s) {
    case DemodStatus::Success:
        return "success";
    case DemodStatus::InvalidInput:
        return "invalid_input";
    case DemodStatus::TimingFailed:
        return "timing_failed";
    case DemodStatus::CfoFailed:
        return "cfo_failed";
    case DemodStatus::SfdFailed:
        return "sfd_failed";
    case DemodStatus::PhrFailed:
        return "phr_failed";
    case DemodStatus::PayloadFailed:
        return "payload_failed";
    case DemodStatus::FcsFailed:
        return "fcs_failed";
    case DemodStatus::QueueFull:
        return "queue_full";
    case DemodStatus::InternalError:
        return "internal_error";
    case DemodStatus::CirFailed:
        return "cir_failed";
    }
    return "internal_error";
}

void
UwbRealtimeDemodulator::publish_result(const Job& job,
                                       const gr::uwb::demod::DemodResult& r)
{
    pmt::pmt_t meta = pmt::make_dict();

    meta = pmt::dict_add(meta, pmt::mp("status"),
                         pmt::string_to_symbol(status_to_string(r.status)));
    meta = pmt::dict_add(meta, pmt::mp("status_code"),
                         pmt::from_long(static_cast<long>(r.status)));

    meta = pmt::dict_add(meta, pmt::mp("packet_id"),
                         pmt::from_uint64(job.packet_id));
    meta = pmt::dict_add(meta, pmt::mp("schedule_index"),
                         pmt::from_uint64(job.schedule_index));

    const int64_t detected = r.timing.preamble_start_sample;
    meta = pmt::dict_add(meta, pmt::mp("predicted_start_sample"),
                         pmt::from_long(job.predicted_start_sample));
    meta = pmt::dict_add(meta, pmt::mp("detected_start_sample"),
                         pmt::from_long(detected));
    meta = pmt::dict_add(meta, pmt::mp("window_start_sample"),
                         pmt::from_long(job.window_start_sample));
    meta = pmt::dict_add(
        meta, pmt::mp("input_sample_count"),
        pmt::from_uint64(static_cast<uint64_t>(
            r.input_sample_count > 0
                ? r.input_sample_count
                : static_cast<size_t>(std::max<int64_t>(0, job.sample_count)))));

    meta = pmt::dict_add(meta, pmt::mp("timing_start_sample"),
                         pmt::from_long(r.timing.preamble_start_sample));
    meta = pmt::dict_add(meta, pmt::mp("timing_metric"),
                         pmt::from_double(r.timing.metric));
    meta = pmt::dict_add(
        meta, pmt::mp("timing_peaks"),
        pmt::from_uint64(static_cast<uint64_t>(r.timing.detected_peaks)));
    meta = pmt::dict_add(
        meta, pmt::mp("timing_expected_peaks"),
        pmt::from_uint64(static_cast<uint64_t>(r.timing.expected_peaks)));
    meta = pmt::dict_add(meta, pmt::mp("measured_period"),
                         pmt::from_double(r.timing.measured_period));

    meta = pmt::dict_add(meta, pmt::mp("cfo_hz"),
                         pmt::from_double(r.cfo.cfo_hz));
    meta = pmt::dict_add(meta, pmt::mp("cfo_phase"),
                         pmt::from_double(r.cfo.residual_phase));
    meta = pmt::dict_add(
        meta, pmt::mp("cfo_peaks_used"),
        pmt::from_uint64(static_cast<uint64_t>(r.cfo.peaks_used)));
    meta = pmt::dict_add(
        meta, pmt::mp("cfo_skipped_peaks"),
        pmt::from_uint64(static_cast<uint64_t>(r.cfo.skipped_peaks)));
    meta = pmt::dict_add(meta, pmt::mp("cfo_fit_first_peak_sample"),
                         pmt::from_long(r.cfo.fit_first_peak_sample));
    meta = pmt::dict_add(meta, pmt::mp("cfo_fit_last_peak_sample"),
                         pmt::from_long(r.cfo.fit_last_peak_sample));

    meta = pmt::dict_add(meta, pmt::mp("sfd_start_sample"),
                         pmt::from_long(r.sfd.sfd_start_sample));
    meta = pmt::dict_add(meta, pmt::mp("sfd_start_chip"),
                         pmt::from_long(r.sfd.sfd_start_chip));
    meta = pmt::dict_add(meta, pmt::mp("sfd_polarity"),
                         pmt::from_long(static_cast<long>(r.sfd.polarity)));
    meta = pmt::dict_add(meta, pmt::mp("sfd_metric"),
                         pmt::from_double(r.sfd.metric));
    meta = pmt::dict_add(meta, pmt::mp("sfd_initial_predicted_sample"),
                         pmt::from_long(r.sfd_initial_predicted_sample));
    meta = pmt::dict_add(meta, pmt::mp("sfd_bootstrap_detected_sample"),
                         pmt::from_long(r.sfd_bootstrap_detected_sample));
    meta = pmt::dict_add(
        meta, pmt::mp("sfd_bootstrap_first_threshold_backtrack_symbols"),
        pmt::from_long(r.sfd_bootstrap_first_threshold_backtrack_symbols));

    meta = pmt::dict_add(
        meta, pmt::mp("cir_first_path_sample"),
        pmt::from_uint64(static_cast<uint64_t>(r.cir.first_path_sample)));
    meta = pmt::dict_add(meta, pmt::mp("cir_peak_metric"),
                         pmt::from_double(r.cir.cir_peak_metric));
    meta = pmt::dict_add(
        meta, pmt::mp("soft_chip_count"),
        pmt::from_uint64(static_cast<uint64_t>(r.cir.soft_chip_count)));

    meta = pmt::dict_add(meta, pmt::mp("ns_sfd_start_chip"),
                         pmt::from_long(r.ns_sfd.sfd_start_chip));
    meta = pmt::dict_add(meta, pmt::mp("ns_sfd_end_chip"),
                         pmt::from_long(r.ns_sfd.sfd_end_chip));
    meta = pmt::dict_add(
        meta, pmt::mp("ns_sfd_polarity"),
        pmt::from_long(static_cast<long>(r.ns_sfd.polarity)));
    meta = pmt::dict_add(meta, pmt::mp("ns_sfd_metric"),
                         pmt::from_double(r.ns_sfd.metric));

    meta = pmt::dict_add(
        meta, pmt::mp("phr_psdu_length"),
        pmt::from_uint64(static_cast<uint64_t>(r.phr.psdu_length)));
    meta = pmt::dict_add(meta, pmt::mp("phr_rate_mbps"),
                         pmt::from_double(r.phr.data_rate_mbps));
    meta = pmt::dict_add(meta, pmt::mp("phr_corrected"),
                         pmt::from_bool(r.phr.secded_corrected));
    meta = pmt::dict_add(meta, pmt::mp("phr_uncorrectable"),
                         pmt::from_bool(r.phr.secded_uncorrectable));

    meta = pmt::dict_add(
        meta, pmt::mp("payload_nbytes"),
        pmt::from_uint64(static_cast<uint64_t>(r.payload.bytes.size())));
    meta = pmt::dict_add(meta, pmt::mp("fcs_pass"),
                         pmt::from_bool(r.payload.fcs_pass));
    meta = pmt::dict_add(
        meta, pmt::mp("fcs_received"),
        pmt::from_uint64(static_cast<uint64_t>(r.payload.received_fcs)));
    meta = pmt::dict_add(
        meta, pmt::mp("fcs_calculated"),
        pmt::from_uint64(static_cast<uint64_t>(r.payload.calculated_fcs)));

    // Rate provenance for schedule lock mapping (998.4 → 737.28).
    if (job.sample_rate > 0.0)
        meta = pmt::dict_add(meta, pmt::mp("sample_rate"),
                             pmt::from_double(job.sample_rate));
    if (job.native_sample_rate > 0.0) {
        meta = pmt::dict_add(meta, pmt::mp("input_sample_rate"),
                             pmt::from_double(job.native_sample_rate));
        meta = pmt::dict_add(meta, pmt::mp("native_sample_rate"),
                             pmt::from_double(job.native_sample_rate));
    }
    if (job.resample_filter_delay > 0.0)
        meta = pmt::dict_add(meta, pmt::mp("resample_filter_delay"),
                             pmt::from_double(job.resample_filter_delay));

    meta = pmt::dict_add(meta, pmt::mp("queue_delay_us"),
                         pmt::from_uint64(r.queue_delay_us));
    meta = pmt::dict_add(meta, pmt::mp("demod_latency_us"),
                         pmt::from_uint64(r.demod_latency_us));
    meta = pmt::dict_add(meta, pmt::mp("wall_latency_us"),
                         pmt::from_uint64(r.wall_latency_us));
    // per-stage demod timing (µs) — recorded by demodulate_one
    meta = pmt::dict_add(meta, pmt::mp("stage_timing_us"),
                         pmt::from_uint64(r.stage_timing_us));
    meta = pmt::dict_add(meta, pmt::mp("stage_timing_coarse_us"),
                         pmt::from_uint64(r.stage_timing_coarse_us));
    meta = pmt::dict_add(meta, pmt::mp("stage_timing_fine_track_us"),
                         pmt::from_uint64(r.stage_timing_fine_track_us));
    meta = pmt::dict_add(meta, pmt::mp("stage_cfo_us"),
                         pmt::from_uint64(r.stage_cfo_us));
    meta = pmt::dict_add(meta, pmt::mp("stage_sfd_us"),
                         pmt::from_uint64(r.stage_sfd_us));
    meta = pmt::dict_add(meta, pmt::mp("sfd_bootstrap_windows"),
                         pmt::from_uint64(r.sfd_bootstrap_windows));
    meta = pmt::dict_add(meta, pmt::mp("sfd_bootstrap_coarse_correlations"),
                         pmt::from_uint64(r.sfd_bootstrap_coarse_correlations));
    meta = pmt::dict_add(meta, pmt::mp("sfd_bootstrap_fine_correlations"),
                         pmt::from_uint64(r.sfd_bootstrap_fine_correlations));
    meta = pmt::dict_add(meta, pmt::mp("sfd_final_windows"),
                         pmt::from_uint64(r.sfd_final_windows));
    meta = pmt::dict_add(meta, pmt::mp("sfd_final_coarse_correlations"),
                         pmt::from_uint64(r.sfd_final_coarse_correlations));
    meta = pmt::dict_add(meta, pmt::mp("sfd_final_fine_correlations"),
                         pmt::from_uint64(r.sfd_final_fine_correlations));
    meta = pmt::dict_add(meta, pmt::mp("stage_cir_us"),
                         pmt::from_uint64(r.stage_cir_us));
    meta = pmt::dict_add(meta, pmt::mp("stage_ns_sfd_us"),
                         pmt::from_uint64(r.stage_ns_sfd_us));
    meta = pmt::dict_add(meta, pmt::mp("stage_phr_us"),
                         pmt::from_uint64(r.stage_phr_us));
    meta = pmt::dict_add(meta, pmt::mp("stage_payload_us"),
                         pmt::from_uint64(r.stage_payload_us));
    meta = pmt::dict_add(meta, pmt::mp("stage_total_us"),
                         pmt::from_uint64(r.stage_total_us));
    meta = pmt::dict_add(meta, pmt::mp("resample_us"),
                         pmt::from_uint64(r.resample_us));
    // CIR sub-stage breakdown (P0 diagnostics)
    meta = pmt::dict_add(meta, pmt::mp("cir_estimate_us"),
                         pmt::from_uint64(r.cir.cir_estimate_us));
    meta = pmt::dict_add(meta, pmt::mp("cir_softfir_us"),
                         pmt::from_uint64(r.cir.soft_fir_us));
    meta = pmt::dict_add(meta, pmt::mp("cir_postprocess_us"),
                         pmt::from_uint64(r.cir.postprocess_us));
    meta = pmt::dict_add(
        meta, pmt::mp("worker_id"),
        pmt::from_uint64(static_cast<uint64_t>(r.worker_id)));

    // Accumulate per-stage means (index 7 = total).
    {
        std::lock_guard<std::mutex> lock(d_stage_mutex_);
        d_stage_sums_[0] += r.stage_timing_us;
        d_stage_sums_[1] += r.stage_cfo_us;
        d_stage_sums_[2] += r.stage_sfd_us;
        d_stage_sums_[3] += r.stage_cir_us;
        d_stage_sums_[4] += r.stage_ns_sfd_us;
        d_stage_sums_[5] += r.stage_phr_us;
        d_stage_sums_[6] += r.stage_payload_us;
        d_stage_sums_[7] += r.stage_total_us;
        ++d_stage_n_;
    }

    pmt::pmt_t vec;
    if (r.payload.ok && !r.payload.bytes.empty()) {
        vec = pmt::init_u8vector(r.payload.bytes.size(),
                                 r.payload.bytes.data());
    } else {
        vec = pmt::init_u8vector(0, static_cast<const uint8_t*>(nullptr));
    }

    message_port_pub(pmt::mp("result"), pmt::cons(meta, vec));

    // Drive schedule lock from SYNC/preamble timing (learn-then-freeze).
    // FCS is NOT the fast sensor: a good SYNC peak is enough to learn δ.
    // Wide search covers residual until Hold freezes (b, δ).
    if (r.timing.ok && r.timing.preamble_start_sample >= 0) {
        publish_schedule_feedback(job, r);
    }
}

void
UwbRealtimeDemodulator::publish_schedule_feedback(
    const Job& job, const gr::uwb::demod::DemodResult& r)
{
    pmt::pmt_t fb = pmt::make_dict();
    fb = pmt::dict_add(fb, pmt::mp("command"), pmt::mp("observe"));
    fb = pmt::dict_add(fb, pmt::mp("schedule_index"),
                       pmt::from_uint64(job.schedule_index));
    fb = pmt::dict_add(fb, pmt::mp("packet_id"),
                       pmt::from_uint64(job.packet_id));
    fb = pmt::dict_add(fb, pmt::mp("detected_start_sample"),
                       pmt::from_long(r.timing.preamble_start_sample));
    fb = pmt::dict_add(fb, pmt::mp("timing_start_sample"),
                       pmt::from_long(r.timing.preamble_start_sample));
    fb = pmt::dict_add(fb, pmt::mp("predicted_start_sample"),
                       pmt::from_long(job.predicted_start_sample));
    fb = pmt::dict_add(fb, pmt::mp("timing_ok"), pmt::PMT_T);
    // Overall demod status / FCS are diagnostics only for the lock path.
    fb = pmt::dict_add(fb, pmt::mp("status"),
                       pmt::string_to_symbol(status_to_string(r.status)));
    fb = pmt::dict_add(fb, pmt::mp("fcs_pass"),
                       r.payload.fcs_pass ? pmt::PMT_T : pmt::PMT_F);
    fb = pmt::dict_add(fb, pmt::mp("timing_metric"),
                       pmt::from_double(r.timing.metric));
    fb = pmt::dict_add(
        fb, pmt::mp("timing_peaks"),
        pmt::from_uint64(static_cast<uint64_t>(r.timing.detected_peaks)));
    if (job.sample_rate > 0.0)
        fb = pmt::dict_add(fb, pmt::mp("sample_rate"),
                           pmt::from_double(job.sample_rate));
    if (job.native_sample_rate > 0.0) {
        fb = pmt::dict_add(fb, pmt::mp("input_sample_rate"),
                           pmt::from_double(job.native_sample_rate));
        fb = pmt::dict_add(fb, pmt::mp("native_sample_rate"),
                           pmt::from_double(job.native_sample_rate));
    }
    if (job.resample_filter_delay > 0.0)
        fb = pmt::dict_add(fb, pmt::mp("resample_filter_delay"),
                           pmt::from_double(job.resample_filter_delay));
    message_port_pub(pmt::mp("schedule_feedback"), fb);
}

uint64_t
UwbRealtimeDemodulator::stage_mean_us(size_t stage) const
{
    if (stage >= 8)
        return 0;
    std::lock_guard<std::mutex> lock(d_stage_mutex_);
    return d_stage_n_ ? d_stage_sums_[stage] / d_stage_n_ : 0;
}

uint64_t
UwbRealtimeDemodulator::stage_mean_total_us() const
{
    std::lock_guard<std::mutex> lock(d_stage_mutex_);
    return d_stage_n_ ? d_stage_sums_[7] / d_stage_n_ : 0;
}

void
UwbRealtimeDemodulator::snapshot_stats(pmt::pmt_t& meta)
{
    meta = pmt::dict_add(meta, pmt::mp("jobs_received"),
                         pmt::from_uint64(jobs_received()));
    meta = pmt::dict_add(meta, pmt::mp("jobs_completed"),
                         pmt::from_uint64(jobs_completed()));
    meta = pmt::dict_add(meta, pmt::mp("jobs_failed"),
                         pmt::from_uint64(jobs_failed()));
    meta = pmt::dict_add(meta, pmt::mp("jobs_dropped"),
                         pmt::from_uint64(jobs_dropped()));
    meta = pmt::dict_add(meta, pmt::mp("invalid_inputs"),
                         pmt::from_uint64(invalid_inputs()));
    meta = pmt::dict_add(meta, pmt::mp("worker_exceptions"),
                         pmt::from_uint64(worker_exceptions()));
    meta = pmt::dict_add(
        meta, pmt::mp("queue_depth"),
        pmt::from_uint64(static_cast<uint64_t>(queue_depth())));
    meta = pmt::dict_add(
        meta, pmt::mp("queue_high_watermark"),
        pmt::from_uint64(static_cast<uint64_t>(queue_high_watermark())));
    meta = pmt::dict_add(meta, pmt::mp("latency_p50_us"),
                         pmt::from_uint64(latency_p50_us()));
    meta = pmt::dict_add(meta, pmt::mp("latency_p95_us"),
                         pmt::from_uint64(latency_p95_us()));
    meta = pmt::dict_add(meta, pmt::mp("latency_p99_us"),
                         pmt::from_uint64(latency_p99_us()));
    meta = pmt::dict_add(meta, pmt::mp("latency_max_us"),
                         pmt::from_uint64(latency_max_us()));
    meta = pmt::dict_add(meta, pmt::mp("worker_utilization_pct"),
                         pmt::from_double(worker_utilization_pct()));

    // per-stage mean demod time (µs) over published jobs
    const char* stage_names[8] = { "timing", "cfo", "sfd", "cir",
                                   "ns_sfd", "phr", "payload", "total" };
    {
        std::lock_guard<std::mutex> lock(d_stage_mutex_);
        for (size_t s = 0; s < 8; ++s) {
            const uint64_t mean =
                d_stage_n_ ? d_stage_sums_[s] / d_stage_n_ : 0;
            meta = pmt::dict_add(
                meta, pmt::mp("stage_mean_" + std::string(stage_names[s]) + "_us"),
                pmt::from_uint64(mean));
        }
    }
}

void
UwbRealtimeDemodulator::publish_status(const std::string& event,
                                       pmt::pmt_t extra)
{
    pmt::pmt_t meta = pmt::make_dict();
    meta = pmt::dict_add(meta, pmt::mp("event"),
                         pmt::string_to_symbol(event));
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
UwbRealtimeDemodulator::record_latency(uint64_t us)
{
    {
        std::lock_guard<std::mutex> lock(d_hist_mutex_);
        const size_t bin = static_cast<size_t>(us / 64);
        if (bin >= 64)
            ++d_latency_overflow_;
        else
            ++d_latency_buckets_[bin];
    }
    uint64_t cur = d_latency_max_us_.load(std::memory_order_relaxed);
    while (us > cur && !d_latency_max_us_.compare_exchange_weak(
                           cur, us, std::memory_order_relaxed)) {
        // retry
    }
}

uint64_t
UwbRealtimeDemodulator::latency_percentile_us(double pct) const
{
    std::lock_guard<std::mutex> lock(d_hist_mutex_);
    uint64_t total = d_latency_overflow_;
    for (int i = 0; i < 64; ++i)
        total += d_latency_buckets_[i];
    if (total == 0)
        return 0;

    // First bin whose cumulative count reaches the threshold; report the
    // bin upper bound: (bin + 1) * 64 µs.
    const uint64_t threshold = std::max<uint64_t>(
        1, static_cast<uint64_t>(std::ceil(pct * static_cast<double>(total))));
    uint64_t cum = 0;
    for (int i = 0; i < 64; ++i) {
        cum += d_latency_buckets_[i];
        if (cum >= threshold)
            return static_cast<uint64_t>(i + 1) * 64;
    }
    // Threshold lies in the overflow tail — report max latency if known,
    // else the open upper bound of the last finite bin.
    const uint64_t mx = d_latency_max_us_.load(std::memory_order_relaxed);
    return mx > 0 ? mx : static_cast<uint64_t>(64) * 64;
}

} // namespace uwb
} // namespace gr
