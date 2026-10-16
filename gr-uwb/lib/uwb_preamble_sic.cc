/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * UwbPreambleSic — thin GNU Radio PDU adapter for the S0 preamble-only SIC
 * math core.  See uwb_preamble_sic.h (scheduler semantics) and
 * docs/sic/S0_sic_adapter_api.md (frozen contract).
 *
 * This file contains NO SIC algorithm: it validates PDUs, applies admission
 * control, runs the core on a single worker, wraps PMTs and counts.  All
 * reconstruction logic lives in uwb_preamble_sic_core.cc.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gnuradio/io_signature.h>
#include <gnuradio/uwb/uwb_preamble_sic.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fstream>
#include <stdexcept>
#include <utility>

namespace gr {
namespace uwb {

namespace {

inline uint64_t elapsed_us(std::chrono::steady_clock::time_point t0,
                           std::chrono::steady_clock::time_point t1)
{
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count());
}

int64_t dict_i64(pmt::pmt_t dict, const char* key, int64_t def)
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

double dict_f64(pmt::pmt_t dict, const char* key, double def)
{
    if (!pmt::is_dict(dict))
        return def;
    pmt::pmt_t v = pmt::dict_ref(dict, pmt::mp(key), pmt::from_double(def));
    if (pmt::is_real(v) || pmt::is_integer(v) || pmt::is_uint64(v))
        return pmt::to_double(v);
    return def;
}

// Copy an input meta dict (the sensing description) into a fresh dict so the
// adapter's own keys can be added without mutating the producer's object.  Every
// input key is preserved verbatim: code_index / sync_repetitions / sfd_mode /
// pulse_id / window ids are the SENSING description and must pass through
// untouched (the comm profile is block config and never overwrites them).
pmt::pmt_t copy_dict(pmt::pmt_t in)
{
    pmt::pmt_t out = pmt::make_dict();
    if (!pmt::is_dict(in))
        return out;
    pmt::pmt_t items = pmt::dict_items(in);
    const size_t n = pmt::length(items);
    for (size_t i = 0; i < n; ++i) {
        pmt::pmt_t kv = pmt::nth(i, items);
        out = pmt::dict_add(out, pmt::car(kv), pmt::cdr(kv));
    }
    return out;
}

} // namespace

// ---------------------------------------------------------------------------
// Construction / factory
// ---------------------------------------------------------------------------

std::vector<gr_complex>
UwbPreambleSic::load_cf32_file(const std::string& path)
{
    std::ifstream f(path, std::ios::binary);
    if (!f) {
        throw std::invalid_argument(
            "UwbPreambleSic: cannot open template file " + path);
    }
    f.seekg(0, std::ios::end);
    const std::streamoff bytes = f.tellg();
    f.seekg(0, std::ios::beg);
    if (bytes <= 0 ||
        bytes % static_cast<std::streamoff>(sizeof(gr_complex)) != 0) {
        throw std::invalid_argument(
            "UwbPreambleSic: template file empty or invalid size: " + path);
    }
    std::vector<gr_complex> tmpl(static_cast<size_t>(bytes) /
                                 sizeof(gr_complex));
    f.read(reinterpret_cast<char*>(tmpl.data()), bytes);
    if (!f && !f.eof()) {
        throw std::invalid_argument(
            "UwbPreambleSic: failed reading template file " + path);
    }
    if (tmpl.empty()) {
        throw std::invalid_argument(
            "UwbPreambleSic: template file is empty: " + path);
    }
    return tmpl;
}

bool UwbPreambleSic::parse_mode(const std::string& mode, sic::SicMode& out)
{
    if (mode == "off") {
        out = sic::SicMode::Off;
        return true;
    }
    if (mode == "estimate") {
        out = sic::SicMode::Estimate;
        return true;
    }
    if (mode == "oracle") {
        out = sic::SicMode::Oracle;
        return true;
    }
    return false;
}

UwbPreambleSic::UwbPreambleSic(const std::vector<gr_complex>& sync,
                               const std::vector<gr_complex>& preamble,
                               const std::string& mode_name,
                               size_t comm_code,
                               size_t sync_reps,
                               size_t samples_per_symbol,
                               double sample_rate,
                               size_t k_paths,
                               size_t fit_reps,
                               size_t validation_reps,
                               int64_t roi_lo,
                               int64_t roi_hi,
                               size_t coarse_stride,
                               double detect_threshold,
                               double validate_threshold,
                               size_t in_flight_max,
                               size_t iq_budget_bytes,
                               size_t queue_capacity,
                               bool publish_reconstruction,
                               bool enable_fractional,
                               bool enable_cfo,
                               size_t path_min_spacing,
                               size_t post_tail,
                               size_t max_window_samples,
                               size_t pending_bypass_max)
    : gr::block("uwb_preamble_sic",
                gr::io_signature::make(0, 0, 0),
                gr::io_signature::make(0, 0, 0)),
      d_sync_(sync),
      d_preamble_(preamble),
      d_mode_name_(mode_name),
      d_in_flight_max_(in_flight_max),
      d_iq_budget_bytes_(iq_budget_bytes),
      d_queue_capacity_(queue_capacity),
      d_publish_reconstruction_(publish_reconstruction),
      d_max_window_samples_(max_window_samples)
{
    if (d_sync_.empty()) {
        throw std::invalid_argument("UwbPreambleSic: sync template is empty");
    }
    if (d_preamble_.empty()) {
        throw std::invalid_argument(
            "UwbPreambleSic: preamble template is empty");
    }
    if (d_in_flight_max_ == 0) {
        throw std::invalid_argument(
            "UwbPreambleSic: in_flight_max must be > 0");
    }
    if (d_iq_budget_bytes_ == 0) {
        throw std::invalid_argument(
            "UwbPreambleSic: iq_budget_bytes must be > 0");
    }
    if (d_queue_capacity_ == 0) {
        throw std::invalid_argument(
            "UwbPreambleSic: queue_capacity must be > 0");
    }
    // Default bound on the pending-bypass FIFO: one in-flight round of work
    // plus the admitted work queue.  0 => computed default.
    d_pending_bypass_max_ = (pending_bypass_max > 0)
        ? pending_bypass_max
        : (d_in_flight_max_ + d_queue_capacity_);

    sic::SicMode m;
    if (!parse_mode(mode_name, m)) {
        throw std::invalid_argument("UwbPreambleSic: unknown mode '" +
                                    mode_name + "' (off|estimate|oracle)");
    }
    d_truth_oracle_ = (m == sic::SicMode::Oracle);
    d_cfg_.mode = m;
    d_cfg_.comm_code = comm_code;
    d_cfg_.sync_reps = sync_reps;
    d_cfg_.samples_per_symbol = samples_per_symbol;
    d_cfg_.sample_rate = sample_rate;
    d_cfg_.k_paths = k_paths;
    d_cfg_.fit_reps = fit_reps;
    d_cfg_.validation_reps = validation_reps;
    d_cfg_.roi_lo = roi_lo;
    d_cfg_.roi_hi = roi_hi;
    d_cfg_.coarse_stride = coarse_stride;
    d_cfg_.detect_threshold = detect_threshold;
    d_cfg_.validate_threshold = validate_threshold;
    d_cfg_.enable_fractional = enable_fractional;
    d_cfg_.enable_cfo = enable_cfo;
    d_cfg_.path_min_spacing = path_min_spacing;
    d_cfg_.post_tail = post_tail;

    // Pre-size the output pool when the caller declared a maximum window.
    if (d_max_window_samples_ > 0) {
        d_out_.assign(d_max_window_samples_, gr_complex(0.0f, 0.0f));
        if (d_publish_reconstruction_)
            d_ihat_.assign(d_max_window_samples_, gr_complex(0.0f, 0.0f));
        d_pool_capacity_ = d_max_window_samples_;
    }

    std::string err;
    if (!sic::prepare_sic_core(d_cfg_,
                               d_sync_.data(),
                               d_sync_.size(),
                               d_preamble_.data(),
                               d_preamble_.size(),
                               d_scratch_,
                               &err)) {
        throw std::invalid_argument("UwbPreambleSic: " + err);
    }
    // Estimate mode is the only mode usable for effect/real-time acceptance; it
    // needs fixed, non-zero thresholds or the core fails closed at run time.
    if (m == sic::SicMode::Estimate &&
        (!(detect_threshold > 0.0) || !(validate_threshold > 0.0))) {
        throw std::invalid_argument(
            "UwbPreambleSic: estimate mode requires non-zero detect/validate "
            "thresholds");
    }

    message_port_register_in(pmt::mp("in"));
    // The truth port is oracle-only: in estimate/off it does not exist and no
    // truth object is ever stored on the block.
    if (d_truth_oracle_) {
        message_port_register_in(pmt::mp("truth"));
        set_msg_handler(pmt::mp("truth"),
                        [this](pmt::pmt_t msg) { handle_truth(msg); });
    }
    message_port_register_out(pmt::mp("out"));
    message_port_register_out(pmt::mp("status"));
    if (d_publish_reconstruction_)
        message_port_register_out(pmt::mp("i_hat"));

    set_msg_handler(pmt::mp("in"),
                    [this](pmt::pmt_t msg) { handle_in(msg); });
}

UwbPreambleSic::~UwbPreambleSic()
{
    {
        std::lock_guard<std::mutex> lock(d_queue_mutex_);
        d_stop_ = true;
    }
    d_queue_cv_.notify_all();
    if (d_worker_.joinable())
        d_worker_.join();
}

UwbPreambleSic::sptr
UwbPreambleSic::make(const std::string& sync_template_path,
                     const std::string& preamble_template_path,
                     size_t comm_code,
                     size_t sync_reps,
                     size_t samples_per_symbol,
                     double sample_rate,
                     size_t k_paths,
                     size_t fit_reps,
                     size_t validation_reps,
                     int64_t roi_lo,
                     int64_t roi_hi,
                     size_t coarse_stride,
                     const std::string& mode,
                     double detect_threshold,
                     double validate_threshold,
                     size_t in_flight_max,
                     size_t iq_budget_bytes,
                     size_t queue_capacity,
                     bool publish_reconstruction,
                     bool enable_fractional,
                     bool enable_cfo,
                     size_t path_min_spacing,
                     size_t post_tail,
                     size_t max_window_samples,
                     size_t pending_bypass_max)
{
    std::vector<gr_complex> sync = load_cf32_file(sync_template_path);
    std::vector<gr_complex> preamble = load_cf32_file(preamble_template_path);
    return make_from_templates(sync,
                               preamble,
                               comm_code,
                               sync_reps,
                               samples_per_symbol,
                               sample_rate,
                               k_paths,
                               fit_reps,
                               validation_reps,
                               roi_lo,
                               roi_hi,
                               coarse_stride,
                               mode,
                               detect_threshold,
                               validate_threshold,
                               in_flight_max,
                               iq_budget_bytes,
                               queue_capacity,
                               publish_reconstruction,
                               enable_fractional,
                               enable_cfo,
                               path_min_spacing,
                               post_tail,
                               max_window_samples,
                               pending_bypass_max);
}

UwbPreambleSic::sptr
UwbPreambleSic::make_from_templates(const std::vector<gr_complex>& sync,
                                    const std::vector<gr_complex>& preamble,
                                    size_t comm_code,
                                    size_t sync_reps,
                                    size_t samples_per_symbol,
                                    double sample_rate,
                                    size_t k_paths,
                                    size_t fit_reps,
                                    size_t validation_reps,
                                    int64_t roi_lo,
                                    int64_t roi_hi,
                                    size_t coarse_stride,
                                    const std::string& mode,
                                    double detect_threshold,
                                    double validate_threshold,
                                    size_t in_flight_max,
                                    size_t iq_budget_bytes,
                                    size_t queue_capacity,
                                    bool publish_reconstruction,
                                    bool enable_fractional,
                                    bool enable_cfo,
                                    size_t path_min_spacing,
                                    size_t post_tail,
                                    size_t max_window_samples,
                                    size_t pending_bypass_max)
{
    return gnuradio::get_initial_sptr(new UwbPreambleSic(sync,
                                                         preamble,
                                                         mode,
                                                         comm_code,
                                                         sync_reps,
                                                         samples_per_symbol,
                                                         sample_rate,
                                                         k_paths,
                                                         fit_reps,
                                                         validation_reps,
                                                         roi_lo,
                                                         roi_hi,
                                                         coarse_stride,
                                                         detect_threshold,
                                                         validate_threshold,
                                                         in_flight_max,
                                                         iq_budget_bytes,
                                                         queue_capacity,
                                                         publish_reconstruction,
                                                         enable_fractional,
                                                         enable_cfo,
                                                         path_min_spacing,
                                                         post_tail,
                                                         max_window_samples,
                                                         pending_bypass_max));
}

// ---------------------------------------------------------------------------
// Lifecycle / stats
// ---------------------------------------------------------------------------

size_t
UwbPreambleSic::queue_depth() const
{
    std::lock_guard<std::mutex> lock(d_queue_mutex_);
    return d_queue_.size();
}

bool
UwbPreambleSic::drained() const
{
    std::lock_guard<std::mutex> lock(d_queue_mutex_);
    return d_queue_.empty() && d_worker_idle_;
}

void
UwbPreambleSic::drain()
{
    std::unique_lock<std::mutex> lock(d_queue_mutex_);
    d_queue_cv_.wait(
        lock, [this] { return d_queue_.empty() && d_worker_idle_; });
}

bool
UwbPreambleSic::start()
{
    // Join any leftover worker without holding the queue mutex (join under the
    // lock would deadlock a worker blocked on d_queue_mutex_).
    {
        std::lock_guard<std::mutex> lock(d_queue_mutex_);
        if (d_worker_.joinable())
            d_stop_ = true;
    }
    d_queue_cv_.notify_all();
    if (d_worker_.joinable())
        d_worker_.join();
    {
        std::lock_guard<std::mutex> lock(d_queue_mutex_);
        d_stop_ = false;
        d_worker_idle_ = true;
    }
    d_worker_ = std::thread(&UwbPreambleSic::worker_loop, this);
    return true;
}

bool
UwbPreambleSic::stop()
{
    {
        std::lock_guard<std::mutex> lock(d_queue_mutex_);
        d_stop_ = true;
    }
    d_queue_cv_.notify_all();
    if (d_worker_.joinable())
        d_worker_.join();
    publish_status("stopped");
    return true;
}

// ---------------------------------------------------------------------------
// Message handlers
// ---------------------------------------------------------------------------

void
UwbPreambleSic::handle_in(pmt::pmt_t msg)
{
    const auto t0 = std::chrono::steady_clock::now();

    // ---- validate --------------------------------------------------------
    if (!pmt::is_pair(msg)) {
        d_invalid_.fetch_add(1, std::memory_order_relaxed);
        publish_status("invalid_input");
        return;
    }
    pmt::pmt_t meta_in = pmt::car(msg);
    pmt::pmt_t data = pmt::cdr(msg);
    if (!pmt::is_dict(meta_in))
        meta_in = pmt::make_dict();
    if (!pmt::is_c32vector(data)) {
        d_invalid_.fetch_add(1, std::memory_order_relaxed);
        publish_status("invalid_input");
        return;
    }
    const size_t n = pmt::length(data);
    if (n == 0) {
        d_invalid_.fetch_add(1, std::memory_order_relaxed);
        publish_status("invalid_input");
        return;
    }
    // "wrong length": a declared sample_count must agree with the vector.
    if (pmt::dict_has_key(meta_in, pmt::mp("sample_count"))) {
        const int64_t sc = dict_i64(meta_in, "sample_count", -1);
        if (sc < 0 || static_cast<uint64_t>(sc) != static_cast<uint64_t>(n)) {
            d_invalid_.fetch_add(1, std::memory_order_relaxed);
            publish_status("invalid_input");
            return;
        }
    }
    size_t nelem = 0;
    const gr_complex* y = pmt::c32vector_elements(data, nelem);
    if (y == nullptr || nelem != n) {
        d_invalid_.fetch_add(1, std::memory_order_relaxed);
        publish_status("invalid_input");
        return;
    }

    Pending p;
    p.meta_in = meta_in;
    p.iq_in = data;
    p.n = n;
    p.bytes = n * sizeof(gr_complex);
    if (pmt::dict_has_key(meta_in, pmt::mp("planned_publish_ns"))) {
        const int64_t plan_ns = dict_i64(meta_in, "planned_publish_ns", 0);
        p.offered_at = (plan_ns > 0)
            ? std::chrono::steady_clock::time_point(
                  std::chrono::nanoseconds(plan_ns))
            : t0;
    } else {
        p.offered_at = t0;
    }

    d_offered_.fetch_add(1, std::memory_order_relaxed);

    // ---- admission -------------------------------------------------------
    const char* reject_reason = nullptr;
    bool overflow = false;
    {
        std::lock_guard<std::mutex> lock(d_queue_mutex_);
        const bool room_inflight = d_in_flight_ < d_in_flight_max_;
        const bool room_bytes =
            (d_pending_bytes_ + p.bytes) <= d_iq_budget_bytes_;
        const bool room_queue = d_queue_admitted_ < d_queue_capacity_;
        // "Stop admitting" once the ordered-bypass FIFO is at its cap.
        const bool room_bypass = d_pending_bypass_ < d_pending_bypass_max_;
        if (room_inflight && room_bytes && room_queue && room_bypass) {
            p.admitted = true;
            d_admitted_.fetch_add(1, std::memory_order_relaxed);
            ++d_in_flight_;
            ++d_queue_admitted_;
            d_pending_bytes_ += p.bytes;
            d_inflight_hwm_ = std::max(d_inflight_hwm_, d_in_flight_);
        } else {
            p.admitted = false;
            if (!room_inflight)
                reject_reason = "in_flight_limit";
            else if (!room_bytes)
                reject_reason = "iq_budget";
            else if (!room_queue)
                reject_reason = "queue_full";
            else
                reject_reason = "bypass_fifo_full";
            p.bypass_reason = reject_reason;
            d_missed_admission_.fetch_add(1, std::memory_order_relaxed);
            // Honest bounded bypass FIFO: if parking this ordered bypass would
            // exceed the cap, terminate it as queue_overflow instead (recorded,
            // never a silent drop).  Upstream credit keeps this off the normal
            // path (offered == admitted + missed_admission).
            if (d_pending_bypass_ >= d_pending_bypass_max_) {
                overflow = true;
            } else {
                ++d_pending_bypass_;
                if (d_pending_bypass_ > d_bypass_fifo_hwm_)
                    d_bypass_fifo_hwm_ = d_pending_bypass_;
            }
        }
        if (!overflow) {
            // Enqueue in offered order.  Bypass entries carry no scratch and
            // reuse the original IQ; this is what keeps `out` ordered.
            p.enqueued_at = std::chrono::steady_clock::now();
            p.handler_us = elapsed_us(t0, p.enqueued_at);
            d_queue_.push_back(std::move(p));
            if (d_queue_.size() > d_queue_hwm_)
                d_queue_hwm_ = d_queue_.size();
        }
    }
    if (overflow) {
        d_queue_overflow_.fetch_add(1, std::memory_order_relaxed);
        d_dropped_.fetch_add(1, std::memory_order_relaxed);
        pmt::pmt_t extra = pmt::make_dict();
        extra = pmt::dict_add(extra, pmt::mp("window_id"),
                              pmt::from_uint64(dict_u64(meta_in,
                                                        "window_id", 0)));
        extra = pmt::dict_add(extra, pmt::mp("sic_state"),
                              pmt::mp("queue_overflow"));
        extra = pmt::dict_add(extra, pmt::mp("reason"),
                              pmt::mp("bypass_fifo_full"));
        extra = pmt::dict_add(extra, pmt::mp("handler_us"),
                              pmt::from_uint64(elapsed_us(
                                  t0, std::chrono::steady_clock::now())));
        publish_status("queue_overflow", extra);
        return;
    }
    d_queue_cv_.notify_one();
}

void
UwbPreambleSic::handle_truth(pmt::pmt_t msg)
{
    // Oracle-only diagnostic sidecar.  Never consulted in Estimate mode.
    if (!pmt::is_dict(msg) || !pmt::dict_has_key(msg, pmt::mp("window_id"))) {
        publish_status("invalid_truth");
        return;
    }
    sic::SicTruth t;
    t.valid = true;
    t.start_sample = dict_f64(msg, "start_sample", 0.0);
    t.cfo_hz = dict_f64(msg, "cfo_hz", 0.0);

    size_t nt = 0;
    const double* tau = nullptr;
    if (pmt::dict_has_key(msg, pmt::mp("tau"))) {
        pmt::pmt_t v = pmt::dict_ref(msg, pmt::mp("tau"), pmt::PMT_NIL);
        if (pmt::is_f64vector(v))
            tau = pmt::f64vector_elements(v, nt);
    }
    size_t ng = 0, ni = 0;
    const double* gr = nullptr;
    const double* gi = nullptr;
    if (pmt::dict_has_key(msg, pmt::mp("gain_re"))) {
        pmt::pmt_t v = pmt::dict_ref(msg, pmt::mp("gain_re"), pmt::PMT_NIL);
        if (pmt::is_f64vector(v))
            gr = pmt::f64vector_elements(v, ng);
    }
    if (pmt::dict_has_key(msg, pmt::mp("gain_im"))) {
        pmt::pmt_t v = pmt::dict_ref(msg, pmt::mp("gain_im"), pmt::PMT_NIL);
        if (pmt::is_f64vector(v))
            gi = pmt::f64vector_elements(v, ni);
    }
    const size_t avail =
        std::min({ nt, ng, ni, static_cast<size_t>(sic::kSicMaxPaths) });
    size_t np = static_cast<size_t>(
        std::max<int64_t>(0, dict_i64(msg, "n_paths", (int64_t)avail)));
    np = std::min(np, avail);
    t.n_paths = np;
    for (size_t k = 0; k < np; ++k) {
        t.tau[k] = (tau != nullptr) ? tau[k] : 0.0;
        const double re = (gr != nullptr) ? gr[k] : 0.0;
        const double im = (gi != nullptr) ? gi[k] : 0.0;
        t.gain[k] = std::complex<float>(static_cast<float>(re),
                                        static_cast<float>(im));
    }

    const uint64_t wid = dict_u64(msg, "window_id", 0);
    std::lock_guard<std::mutex> lock(d_truth_mutex_);
    d_truth_[wid] = t;
}

// ---------------------------------------------------------------------------
// Worker
// ---------------------------------------------------------------------------

void
UwbPreambleSic::worker_loop()
{
    for (;;) {
        Pending p;
        {
            std::unique_lock<std::mutex> lock(d_queue_mutex_);
            d_worker_idle_ = true;
            d_queue_cv_.notify_all();
            d_queue_cv_.wait(lock, [this] {
                return d_stop_ || !d_queue_.empty();
            });
            if (d_queue_.empty() && d_stop_)
                return;
            d_worker_idle_ = false;
            p = std::move(d_queue_.front());
            d_queue_.pop_front();
            if (p.admitted && d_queue_admitted_ > 0)
                --d_queue_admitted_;
            if (!p.admitted && d_pending_bypass_ > 0)
                --d_pending_bypass_;
        }
        process(p);
        d_queue_cv_.notify_all();
    }
}

bool
UwbPreambleSic::ensure_pool(size_t n)
{
    if (n <= d_pool_capacity_)
        return true;
    // Unsized (max_window_samples == 0): the first window sizes the pool.  That
    // initial sizing is not a "grow" event.
    if (d_pool_capacity_ == 0) {
        d_pool_capacity_ = n;
        d_out_.assign(n, gr_complex(0.0f, 0.0f));
        if (d_publish_reconstruction_)
            d_ihat_.assign(n, gr_complex(0.0f, 0.0f));
        return true;
    }
    // One-time grow only, counted.  After that, a larger window is fail-closed
    // (never grow per window on the hot path).
    if (d_window_grow_count_.load(std::memory_order_relaxed) == 0) {
        d_pool_capacity_ = n;
        d_out_.assign(n, gr_complex(0.0f, 0.0f));
        if (d_publish_reconstruction_)
            d_ihat_.assign(n, gr_complex(0.0f, 0.0f));
        d_window_grow_count_.fetch_add(1, std::memory_order_relaxed);
        return true;
    }
    return false;
}

const char*
UwbPreambleSic::state_name(sic::SicState s)
{
    switch (s) {
    case sic::SicState::Bypassed:
        return "bypassed";
    case sic::SicState::Committed:
        return "committed";
    case sic::SicState::Failed:
        return "failed";
    case sic::SicState::Ambiguous:
        return "ambiguous";
    }
    return "failed";
}

void
UwbPreambleSic::process(Pending& p)
{
    const auto t_worker0 = std::chrono::steady_clock::now();
    const uint64_t queue_wait_us = elapsed_us(p.enqueued_at, t_worker0);

    // Size/hold the output pool.  Sized from max_window_samples at
    // construction, else from the first window; at most ONE counted grow, then
    // a larger window is fail-closed (never per-window growth on the hot path).
    const bool pool_ok = ensure_pool(p.n);
    bool pool_exceeded = false;

    sic::SicResult r;
    bool sic_ran = false;
    bool failed = false;
    const char* reason = p.bypass_reason;
    if (p.admitted && !pool_ok) {
        pool_exceeded = true;
        d_pool_capacity_exceeded_.fetch_add(1, std::memory_order_relaxed);
        reason = "pool_capacity_exceeded";
    }

    if (p.admitted && pool_ok) {
        r.out = d_out_.data();
        r.i_hat = d_publish_reconstruction_ ? d_ihat_.data() : nullptr;

        // Oracle-only: an explicit, window-matched truth struct.  Estimate mode
        // never reaches this branch, so it can never read injected truth.
        sic::SicTruth truth;
        const sic::SicTruth* truth_ptr = nullptr;
        if (d_cfg_.mode == sic::SicMode::Oracle) {
            const uint64_t wid = dict_u64(p.meta_in, "window_id", 0);
            std::lock_guard<std::mutex> lock(d_truth_mutex_);
            auto it = d_truth_.find(wid);
            if (it != d_truth_.end()) {
                truth = it->second;
                truth_ptr = &truth;
            }
        }

        sic::SicTemplates tpl;
        tpl.sync = d_sync_.data();
        tpl.sync_len = d_sync_.size();
        tpl.preamble = d_preamble_.data();
        tpl.preamble_len = d_preamble_.size();

        // R10: the SENSING processing ROI (the subtraction / commit region) is a
        // sensing property carried in the input PDU meta and kept SEPARATE from
        // the wide candidate search ROI in the block config.  It gates the commit
        // so a real comm detected outside the sensing preamble/CIR window is
        // bypassed instead of committed (out == y).  Absent keys leave the gate
        // disabled (legacy behaviour: subtraction = support ∩ search ROI).
        sic::SicConfig cfg = d_cfg_;
        if (pmt::dict_has_key(p.meta_in, pmt::mp("sensing_roi_lo")) ||
            pmt::dict_has_key(p.meta_in, pmt::mp("sensing_roi_hi"))) {
            cfg.sub_roi_lo = dict_i64(p.meta_in, "sensing_roi_lo", 0);
            cfg.sub_roi_hi = dict_i64(p.meta_in, "sensing_roi_hi", 0);
            cfg.sub_min_overlap =
                dict_u64(p.meta_in, "sensing_min_overlap", d_cfg_.sub_min_overlap);
        }

        size_t nelem = 0;
        const gr_complex* y = pmt::c32vector_elements(p.iq_in, nelem);
        bool ok = false;
        try {
            ok = sic::sic_one(y, nelem, tpl, cfg, d_scratch_, r, truth_ptr);
        } catch (...) {
            ok = false;
        }
        sic_ran = true;
        if (!ok) {
            failed = true;
            reason = (r.reject_reason != nullptr) ? r.reject_reason
                                                  : "sic_one_failed";
        } else {
            reason = r.bypass_reason;
        }
    }

    const bool committed = p.admitted && sic_ran && !failed && r.committed;
    const bool uses_truth = p.admitted && sic_ran && !failed && r.uses_truth;

    // Diagnostic reconstruction is exactly zero on every bypass path.  It can
    // only be materialised when the pool holds the window (a window larger than
    // the one-time-grown pool is fail-closed and skips the diagnostic i_hat).
    const bool can_publish_ihat =
        d_publish_reconstruction_ && (committed || pool_ok);
    if (can_publish_ihat && !committed) {
        std::fill(d_ihat_.begin(),
                  d_ihat_.begin() + static_cast<std::ptrdiff_t>(p.n),
                  gr_complex(0.0f, 0.0f));
    }

    // ---- wrap + publish ---------------------------------------------------
    const auto t_wrap0 = std::chrono::steady_clock::now();
    pmt::pmt_t meta = copy_dict(p.meta_in);
    auto put_u64 = [&meta](const char* k, uint64_t v) {
        meta = pmt::dict_add(meta, pmt::mp(k), pmt::from_uint64(v));
    };
    auto put_i64 = [&meta](const char* k, int64_t v) {
        meta = pmt::dict_add(meta, pmt::mp(k), pmt::from_long(v));
    };
    auto put_f64 = [&meta](const char* k, double v) {
        meta = pmt::dict_add(meta, pmt::mp(k), pmt::from_double(v));
    };
    auto put_bool = [&meta](const char* k, bool v) {
        meta = pmt::dict_add(meta, pmt::mp(k), pmt::from_bool(v));
    };

    const sic::SicState st =
        (p.admitted && sic_ran) ? r.state : sic::SicState::Bypassed;

    meta = pmt::dict_add(meta, pmt::mp("sic_state"),
                         pmt::string_to_symbol(state_name(st)));
    put_bool("sic_detected", p.admitted && sic_ran && r.detected);
    put_bool("sic_committed", committed);
    put_bool("sic_partial", p.admitted && sic_ran && r.partial);
    put_bool("sic_ambiguous", p.admitted && sic_ran && r.ambiguous);
    meta = pmt::dict_add(
        meta, pmt::mp("sic_bypass_reason"),
        (reason != nullptr) ? pmt::string_to_symbol(reason) : pmt::mp(""));
    put_bool("sic_uses_truth", uses_truth);
    // Additive diagnostic: true when this window was rejected by admission and
    // passed through unchanged (the reason is in sic_bypass_reason).
    put_bool("sic_missed_admission", !p.admitted);
    put_i64("sic_detected_start",
            (p.admitted && sic_ran) ? r.detected_start : int64_t(-1));
    put_f64("sic_cfo_hz", (p.admitted && sic_ran) ? r.cfo_hz : 0.0);
    put_u64("sic_n_paths",
            (p.admitted && sic_ran) ? static_cast<uint64_t>(r.n_paths) : 0);
    put_i64("sic_support_lo",
            (p.admitted && sic_ran) ? r.support_lo : int64_t(-1));
    put_i64("sic_support_hi",
            (p.admitted && sic_ran) ? r.support_hi : int64_t(-1));
    // Actionable overlap of the detected support with the sensing processing ROI
    // (the subtraction/commit gate).  -1 when no sensing ROI was supplied.
    put_i64("sic_sub_overlap",
            (p.admitted && sic_ran) ? r.sub_overlap : int64_t(-1));
    put_f64("sic_residual_corr",
            (p.admitted && sic_ran) ? r.residual_corr : 0.0);

    // Adapter timers.  Values written here are measured through CONSTRUCTION:
    // a message cannot carry the duration of its own enqueue.  The
    // pub-inclusive mirrors are published on the `status` event below.
    put_u64("handler_us", p.handler_us);
    put_u64("queue_wait_us", queue_wait_us);
    put_u64("core_total_us",
            (p.admitted && sic_ran) ? r.timings.core_total_us : 0);

    // Core stage timers (children of core_total_us; never double counted).
    const sic::SicTimings& tt = r.timings;
    const bool have_core = p.admitted && sic_ran;
    put_u64("sic_detect_coarse_us", have_core ? tt.detect_coarse_us : 0);
    put_u64("sic_detect_fine_us", have_core ? tt.detect_fine_us : 0);
    put_u64("sic_cfo_us", have_core ? tt.cfo_us : 0);
    put_u64("sic_cir_us", have_core ? tt.cir_us : 0);
    put_u64("sic_path_select_us", have_core ? tt.path_select_us : 0);
    put_u64("sic_fractional_refine_us",
            have_core ? tt.fractional_refine_us : 0);
    put_u64("sic_matrix_build_us", have_core ? tt.matrix_build_us : 0);
    put_u64("sic_gain_fit_us", have_core ? tt.gain_fit_us : 0);
    put_u64("sic_reconstruct_us", have_core ? tt.reconstruct_us : 0);
    put_u64("sic_validate_commit_us", have_core ? tt.validate_commit_us : 0);

    // Byte-exact bypass: reuse the producer's c32vector PMT unchanged.  A
    // committed window is a fresh vector copied from the cleaned output buffer.
    pmt::pmt_t out_vec =
        committed ? pmt::init_c32vector(p.n, d_out_.data()) : p.iq_in;
    pmt::pmt_t ihat_vec;
    if (can_publish_ihat)
        ihat_vec = pmt::init_c32vector(p.n, d_ihat_.data());

    // Pre-publish timers go into the out meta (see note above).
    {
        const auto t_now = std::chrono::steady_clock::now();
        put_u64("output_wrap_publish_us", elapsed_us(t_wrap0, t_now));
        put_u64("completion_latency_us", elapsed_us(p.offered_at, t_now));
        put_bool("sic_pool_capacity_exceeded", pool_exceeded);
    }

    message_port_pub(pmt::mp("out"), pmt::cons(meta, out_vec));
    if (can_publish_ihat)
        message_port_pub(pmt::mp("i_hat"), pmt::cons(meta, ihat_vec));
    const auto t_pub_done = std::chrono::steady_clock::now();
    // Pub-inclusive adapter timers: output_wrap_publish_us spans PMT
    // construction AND the actual message_port_pub calls; completion_latency_us
    // spans the window's planned publish to the terminal publish.
    const uint64_t wrap_full_us = elapsed_us(t_wrap0, t_pub_done);
    const uint64_t latency_full_us = elapsed_us(p.offered_at, t_pub_done);

    // ---- status + counters ----------------------------------------------
    {
        pmt::pmt_t extra = pmt::make_dict();
        extra = pmt::dict_add(extra, pmt::mp("window_id"),
                              pmt::from_uint64(dict_u64(p.meta_in,
                                                        "window_id", 0)));
        extra = pmt::dict_add(extra, pmt::mp("sic_state"),
                              pmt::string_to_symbol(state_name(st)));
        extra = pmt::dict_add(
            extra, pmt::mp("sic_bypass_reason"),
            (reason != nullptr) ? pmt::string_to_symbol(reason) : pmt::mp(""));
        extra = pmt::dict_add(extra, pmt::mp("missed_admission"),
                              pmt::from_bool(!p.admitted));
        extra = pmt::dict_add(extra, pmt::mp("handler_us"),
                              pmt::from_uint64(p.handler_us));
        extra = pmt::dict_add(extra, pmt::mp("queue_wait_us"),
                              pmt::from_uint64(queue_wait_us));
        extra = pmt::dict_add(
            extra, pmt::mp("core_total_us"),
            pmt::from_uint64(have_core ? tt.core_total_us : 0));
        extra = pmt::dict_add(extra, pmt::mp("output_wrap_publish_us"),
                              pmt::from_uint64(wrap_full_us));
        extra = pmt::dict_add(extra, pmt::mp("completion_latency_us"),
                              pmt::from_uint64(latency_full_us));
        publish_status("window", extra);
    }

    if (p.admitted) {
        d_completed_.fetch_add(1, std::memory_order_relaxed);
        if (failed || (sic_ran && r.state == sic::SicState::Failed))
            d_failed_.fetch_add(1, std::memory_order_relaxed);
        if (!committed)
            d_bypassed_.fetch_add(1, std::memory_order_relaxed);
        std::lock_guard<std::mutex> lock(d_queue_mutex_);
        if (d_in_flight_ > 0)
            --d_in_flight_;
        d_pending_bytes_ =
            (d_pending_bytes_ >= p.bytes) ? d_pending_bytes_ - p.bytes : 0;
    } else {
        d_bypassed_.fetch_add(1, std::memory_order_relaxed);
    }
}

void
UwbPreambleSic::publish_status(const std::string& event, pmt::pmt_t extra)
{
    pmt::pmt_t meta = pmt::make_dict();
    meta = pmt::dict_add(meta, pmt::mp("event"),
                         pmt::string_to_symbol(event));
    meta = pmt::dict_add(meta, pmt::mp("windows_offered"),
                         pmt::from_uint64(windows_offered()));
    meta = pmt::dict_add(meta, pmt::mp("windows_admitted"),
                         pmt::from_uint64(windows_admitted()));
    meta = pmt::dict_add(meta, pmt::mp("windows_completed"),
                         pmt::from_uint64(windows_completed()));
    meta = pmt::dict_add(meta, pmt::mp("windows_failed"),
                         pmt::from_uint64(windows_failed()));
    meta = pmt::dict_add(meta, pmt::mp("windows_bypassed"),
                         pmt::from_uint64(windows_bypassed()));
    meta = pmt::dict_add(meta, pmt::mp("windows_dropped"),
                         pmt::from_uint64(windows_dropped()));
    meta = pmt::dict_add(meta, pmt::mp("windows_invalid"),
                         pmt::from_uint64(windows_invalid()));
    meta = pmt::dict_add(meta, pmt::mp("windows_missed_admission"),
                         pmt::from_uint64(windows_missed_admission()));
    if (pmt::is_dict(extra)) {
        pmt::pmt_t items = pmt::dict_items(extra);
        const size_t n = pmt::length(items);
        for (size_t i = 0; i < n; ++i) {
            pmt::pmt_t kv = pmt::nth(i, items);
            meta = pmt::dict_add(meta, pmt::car(kv), pmt::cdr(kv));
        }
    }
    message_port_pub(pmt::mp("status"), meta);
}

} // namespace uwb
} // namespace gr
