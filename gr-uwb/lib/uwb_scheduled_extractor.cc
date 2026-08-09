/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gnuradio/uwb/uwb_scheduled_extractor.h>
#include <gnuradio/uwb/uwb_detector_core.h>
#include <gnuradio/io_signature.h>

#include <algorithm>
#include <stdexcept>
#include <utility>

namespace gr {
namespace uwb {

namespace {
void normalize_tmpl(std::vector<std::complex<float>>& t)
{
    if (t.empty())
        return;
    core::uwb_l2_normalize(t);
}
} // namespace

UwbScheduledExtractor::UwbScheduledExtractor(
    double sample_rate,
    double packet_interval_s,
    uint64_t first_packet_sample,
    size_t pre_guard_samples,
    size_t capture_samples,
    size_t post_guard_samples,
    size_t pool_size,
    EmitPolicy emit_policy,
    bool verification_enabled,
    const std::vector<std::complex<float>>& radar_template,
    float radar_threshold,
    const std::vector<std::complex<float>>& comm_template,
    float comm_threshold)
    : gr::sync_block("uwb_scheduled_extractor",
                     gr::io_signature::make(1, 1, sizeof(gr_complex)),
                     gr::io_signature::make(0, 0, 0)),
      d_emit_policy_(emit_policy),
      d_verification_enabled_(verification_enabled),
      d_radar_threshold_(radar_threshold),
      d_comm_threshold_(comm_threshold),
      d_radar_tmpl_(radar_template),
      d_comm_tmpl_(comm_template)
{
    if (sample_rate <= 0.0 || packet_interval_s <= 0.0) {
        throw std::invalid_argument(
            "UwbScheduledExtractor: sample_rate and packet_interval_s must be > 0");
    }
    if (pre_guard_samples + capture_samples + post_guard_samples == 0) {
        throw std::invalid_argument(
            "UwbScheduledExtractor: window capacity must be > 0");
    }

    set_max_noutput_items(1048576);
    message_port_register_out(pmt::mp("packet"));
    message_port_register_out(pmt::mp("status"));
    message_port_register_in(pmt::mp("schedule"));
    set_msg_handler(pmt::mp("schedule"),
                    [this](pmt::pmt_t msg) { handle_schedule_msg(msg); });

    normalize_tmpl(d_radar_tmpl_);
    normalize_tmpl(d_comm_tmpl_);

    core::ScheduleConfig cfg;
    cfg.sample_rate = sample_rate;
    cfg.packet_interval_s = packet_interval_s;
    cfg.first_packet_sample_exact =
        static_cast<double>(first_packet_sample);
    cfg.pre_guard_samples = pre_guard_samples;
    cfg.capture_samples = capture_samples;
    cfg.post_guard_samples = post_guard_samples;
    cfg.pool_size = pool_size > 0 ? pool_size : 8;
    cfg.emit_policy = emit_policy;
    core_.configure(cfg);
    core_.set_schedule(cfg.first_packet_sample_exact, packet_interval_s,
                       sample_rate);
}

UwbScheduledExtractor::~UwbScheduledExtractor() { shutdown_worker(); }

std::shared_ptr<UwbScheduledExtractor>
UwbScheduledExtractor::make(
    double sample_rate,
    double packet_interval_s,
    uint64_t first_packet_sample,
    size_t pre_guard_samples,
    size_t capture_samples,
    size_t post_guard_samples,
    size_t pool_size,
    EmitPolicy emit_policy,
    bool verification_enabled,
    const std::vector<std::complex<float>>& radar_template,
    float radar_threshold,
    const std::vector<std::complex<float>>& comm_template,
    float comm_threshold)
{
    return gnuradio::get_initial_sptr(new UwbScheduledExtractor(
        sample_rate, packet_interval_s, first_packet_sample, pre_guard_samples,
        capture_samples, post_guard_samples, pool_size, emit_policy,
        verification_enabled, radar_template, radar_threshold, comm_template,
        comm_threshold));
}

void
UwbScheduledExtractor::set_schedule(uint64_t first_packet_sample,
                                    double packet_interval_s)
{
    std::lock_guard<std::mutex> lock(d_cfg_mutex_);
    d_pending_ = core_.config();
    d_pending_.first_packet_sample_exact =
        static_cast<double>(first_packet_sample);
    if (packet_interval_s > 0.0)
        d_pending_.packet_interval_s = packet_interval_s;
    d_pending_cfg_ = true;
}

void
UwbScheduledExtractor::set_schedule_time(double first_packet_time_s,
                                         double packet_interval_s)
{
    const auto cfg = core_.config();
    const double sample =
        first_packet_time_s * cfg.sample_rate;
    set_schedule(static_cast<uint64_t>(std::llround(sample)),
                 packet_interval_s);
}

void
UwbScheduledExtractor::pause_schedule()
{
    std::lock_guard<std::mutex> lock(d_cfg_mutex_);
    d_pending_pause_ = true;
}

void
UwbScheduledExtractor::resume_schedule()
{
    std::lock_guard<std::mutex> lock(d_cfg_mutex_);
    d_pending_resume_ = true;
}

void
UwbScheduledExtractor::reset_schedule()
{
    std::lock_guard<std::mutex> lock(d_cfg_mutex_);
    d_pending_reset_ = true;
}

void
UwbScheduledExtractor::set_emit_policy(EmitPolicy p)
{
    d_emit_policy_ = p;
}

UwbScheduledExtractor::EmitPolicy
UwbScheduledExtractor::emit_policy() const
{
    return d_emit_policy_;
}

void
UwbScheduledExtractor::set_partial_eos_policy(PartialEosPolicy p)
{
    std::lock_guard<std::mutex> lock(d_cfg_mutex_);
    d_pending_ = core_.config();
    d_pending_.partial_eos_policy = p;
    d_pending_cfg_ = true;
}

void
UwbScheduledExtractor::set_verification_enabled(bool en)
{
    d_verification_enabled_ = en;
}

uint64_t UwbScheduledExtractor::scheduled_windows() const
{
    return core_.stats().scheduled_windows;
}
uint64_t UwbScheduledExtractor::completed_windows() const
{
    return core_.stats().completed_windows;
}
uint64_t UwbScheduledExtractor::emitted_windows() const
{
    return d_emitted_.load(std::memory_order_relaxed);
}
uint64_t UwbScheduledExtractor::dropped_windows() const
{
    return core_.stats().dropped_windows +
           d_dropped_jobs_.load(std::memory_order_relaxed);
}
uint64_t UwbScheduledExtractor::partial_windows() const
{
    return core_.stats().partial_windows;
}
uint64_t UwbScheduledExtractor::queue_high_watermark() const
{
    return core_.stats().queue_high_watermark;
}
uint64_t UwbScheduledExtractor::verification_failures() const
{
    return core_.stats().verification_failures;
}
uint64_t UwbScheduledExtractor::collisions() const
{
    return core_.stats().collisions;
}

uint64_t UwbScheduledExtractor::work_calls() const { return d_work_calls_; }
uint64_t UwbScheduledExtractor::work_items_total() const
{
    return d_work_items_total_;
}
int UwbScheduledExtractor::work_min_noutput_items() const { return d_work_min_n_; }
int UwbScheduledExtractor::work_max_noutput_items() const { return d_work_max_n_; }
double UwbScheduledExtractor::work_mean_noutput_items() const
{
    return d_work_calls_ > 0
               ? static_cast<double>(d_work_items_total_) /
                     static_cast<double>(d_work_calls_)
               : 0.0;
}
void UwbScheduledExtractor::work_noutput_histogram(uint64_t out[5]) const
{
    for (int i = 0; i < 5; ++i)
        out[i] = d_work_hist_[i];
}
void UwbScheduledExtractor::reset_work_stats()
{
    d_work_calls_ = 0;
    d_work_items_total_ = 0;
    d_work_min_n_ = 0;
    d_work_max_n_ = 0;
    for (int i = 0; i < 5; ++i)
        d_work_hist_[i] = 0;
}

void
UwbScheduledExtractor::handle_schedule_msg(pmt::pmt_t msg)
{
    // Accept dict or pair (command, dict).
    pmt::pmt_t dict = msg;
    if (pmt::is_pair(msg) && !pmt::is_dict(msg))
        dict = pmt::cdr(msg);
    if (!pmt::is_dict(dict))
        return;

    const pmt::pmt_t cmd_p =
        pmt::dict_ref(dict, pmt::mp("command"), pmt::mp("set"));
    const std::string cmd =
        pmt::is_symbol(cmd_p) ? pmt::symbol_to_string(cmd_p) : "set";

    std::lock_guard<std::mutex> lock(d_cfg_mutex_);
    if (cmd == "pause") {
        d_pending_pause_ = true;
        return;
    }
    if (cmd == "resume") {
        d_pending_resume_ = true;
        return;
    }
    if (cmd == "reset") {
        d_pending_reset_ = true;
        return;
    }

    // command == "set" (default)
    d_pending_ = core_.config();
    if (pmt::dict_has_key(dict, pmt::mp("first_packet_sample"))) {
        d_pending_.first_packet_sample_exact = static_cast<double>(
            pmt::to_uint64(pmt::dict_ref(
                dict, pmt::mp("first_packet_sample"), pmt::from_uint64(0))));
    }
    if (pmt::dict_has_key(dict, pmt::mp("first_packet_time_s"))) {
        const double t = pmt::to_double(pmt::dict_ref(
            dict, pmt::mp("first_packet_time_s"), pmt::from_double(0.0)));
        d_pending_.first_packet_sample_exact = t * d_pending_.sample_rate;
    }
    if (pmt::dict_has_key(dict, pmt::mp("packet_interval_s"))) {
        d_pending_.packet_interval_s = pmt::to_double(pmt::dict_ref(
            dict, pmt::mp("packet_interval_s"),
            pmt::from_double(d_pending_.packet_interval_s)));
    }
    if (pmt::dict_has_key(dict, pmt::mp("pre_guard_samples"))) {
        d_pending_.pre_guard_samples = static_cast<size_t>(pmt::to_uint64(
            pmt::dict_ref(dict, pmt::mp("pre_guard_samples"),
                          pmt::from_uint64(d_pending_.pre_guard_samples))));
    }
    if (pmt::dict_has_key(dict, pmt::mp("capture_samples"))) {
        d_pending_.capture_samples = static_cast<size_t>(pmt::to_uint64(
            pmt::dict_ref(dict, pmt::mp("capture_samples"),
                          pmt::from_uint64(d_pending_.capture_samples))));
    }
    d_pending_cfg_ = true;
}

void
UwbScheduledExtractor::apply_pending_config()
{
    // Snapshot pending flags under cfg mutex, then (if pool rebuild) drain
    // worker-held handles before touching the free list / pool samples.
    bool do_reset = false;
    bool do_cfg = false;
    bool do_pause = false;
    bool do_resume = false;
    core::ScheduleConfig pending;
    {
        std::lock_guard<std::mutex> lock(d_cfg_mutex_);
        do_reset = d_pending_reset_;
        do_cfg = d_pending_cfg_;
        do_pause = d_pending_pause_;
        do_resume = d_pending_resume_;
        pending = d_pending_;
    }

    if (do_reset || do_cfg) {
        // Move ready → job queue, then wait until worker has released every
        // checked-out window so configure/reset never wipes in-flight IQ.
        enqueue_ready_windows();
        wait_for_worker_idle();
    }

    {
        std::lock_guard<std::mutex> lock(d_cfg_mutex_);
        if (d_pending_reset_) {
            core_.reset();
            d_current_sample_ = 0;
            d_pending_reset_ = false;
            publish_status("schedule_updated");
        }
        if (d_pending_cfg_) {
            const auto old_abs = d_current_sample_;
            core_.configure(d_pending_);
            d_current_sample_ = old_abs;
            core_.set_abs_cursor(old_abs);
            core_.set_schedule(d_pending_.first_packet_sample_exact,
                               d_pending_.packet_interval_s,
                               d_pending_.sample_rate);
            d_pending_cfg_ = false;
            publish_status("schedule_updated");
        }
        if (d_pending_pause_) {
            core_.pause();
            d_pending_pause_ = false;
            publish_status("schedule_lost");
        }
        if (d_pending_resume_) {
            core_.resume();
            d_pending_resume_ = false;
            publish_status("schedule_updated");
        }
    }
    (void)pending;
    (void)do_pause;
    (void)do_resume;
}

int
UwbScheduledExtractor::work(int noutput_items,
                            gr_vector_const_void_star& input_items,
                            gr_vector_void_star& /*output_items*/)
{
    if (noutput_items > 0) {
        ++d_work_calls_;
        d_work_items_total_ += static_cast<uint64_t>(noutput_items);
        if (d_work_min_n_ == 0 || noutput_items < d_work_min_n_)
            d_work_min_n_ = noutput_items;
        if (noutput_items > d_work_max_n_)
            d_work_max_n_ = noutput_items;
        if (noutput_items <= 8192)
            ++d_work_hist_[0];
        else if (noutput_items <= 32768)
            ++d_work_hist_[1];
        else if (noutput_items <= 131072)
            ++d_work_hist_[2];
        else if (noutput_items <= 524288)
            ++d_work_hist_[3];
        else
            ++d_work_hist_[4];
    }

    apply_pending_config();

    const auto* in = reinterpret_cast<const gr_complex*>(input_items[0]);
    // Leave a one-sample sentinel for worker drain (UwbDetector EOS idiom).
    const int consumed =
        (noutput_items > 1) ? noutput_items - 1 : noutput_items;

    if (consumed > 0) {
        core_.process_chunk(in, static_cast<size_t>(consumed), d_current_sample_);
        d_current_sample_ += static_cast<uint64_t>(consumed);
    }

    enqueue_ready_windows();
    if (noutput_items == 1)
        wait_for_worker_idle();

    return consumed;
}

bool
UwbScheduledExtractor::start()
{
    std::lock_guard<std::mutex> lock(d_job_mutex_);
    d_job_head_ = 0;
    d_job_tail_ = 0;
    d_job_count_ = 0;
    d_jobs_in_flight_ = 0;
    d_worker_stop_ = false;
    d_worker_ = std::thread(&UwbScheduledExtractor::worker_loop, this);
    return true;
}

void
UwbScheduledExtractor::enqueue_ready_windows()
{
    while (core_.window_ready()) {
        const auto handle = core_.take_window();
        bool queued = false;
        {
            std::lock_guard<std::mutex> lock(d_job_mutex_);
            if (d_job_count_ < d_job_queue_.size()) {
                d_job_queue_[d_job_tail_] = handle;
                d_job_tail_ = (d_job_tail_ + 1) % d_job_queue_.size();
                ++d_job_count_;
                core_.note_queue_depth(d_job_count_ + d_jobs_in_flight_);
                queued = true;
            }
        }
        if (queued) {
            d_job_cv_.notify_one();
        } else {
            core_.release_window(handle);
            d_dropped_jobs_.fetch_add(1, std::memory_order_relaxed);
        }
    }
}

void
UwbScheduledExtractor::worker_loop()
{
    for (;;) {
        core::ScheduledWindowCore::WindowHandle handle;
        {
            std::unique_lock<std::mutex> lock(d_job_mutex_);
            d_job_cv_.wait(lock, [this] {
                return d_worker_stop_ || d_job_count_ > 0;
            });
            if (d_job_count_ == 0 && d_worker_stop_)
                return;
            handle = d_job_queue_[d_job_head_];
            d_job_head_ = (d_job_head_ + 1) % d_job_queue_.size();
            --d_job_count_;
            ++d_jobs_in_flight_;
        }

        try {
            publish_window(handle);
        } catch (const std::exception& error) {
            d_logger->error("UwbScheduledExtractor worker failed: {}",
                            std::string(error.what()));
        } catch (...) {
            d_logger->error(
                "UwbScheduledExtractor worker failed with unknown exception");
        }
        core_.release_window(handle);
        {
            std::lock_guard<std::mutex> lock(d_job_mutex_);
            --d_jobs_in_flight_;
        }
        d_job_cv_.notify_all();
    }
}

void
UwbScheduledExtractor::shutdown_worker()
{
    if (!d_worker_.joinable())
        return;
    {
        std::lock_guard<std::mutex> lock(d_job_mutex_);
        d_worker_stop_ = true;
    }
    d_job_cv_.notify_one();
    d_worker_.join();
}

void
UwbScheduledExtractor::wait_for_worker_idle()
{
    std::unique_lock<std::mutex> lock(d_job_mutex_);
    d_job_cv_.wait(lock, [this] {
        return d_job_count_ == 0 && d_jobs_in_flight_ == 0;
    });
}

bool
UwbScheduledExtractor::stop()
{
    core_.flush_eos();
    enqueue_ready_windows();
    shutdown_worker();
    return true;
}

void
UwbScheduledExtractor::publish_status(const std::string& event, pmt::pmt_t extra)
{
    pmt::pmt_t meta = pmt::make_dict();
    meta = pmt::dict_add(meta, pmt::mp("event"), pmt::string_to_symbol(event));
    meta = pmt::dict_add(meta, pmt::mp("abs_sample"),
                         pmt::from_uint64(d_current_sample_));
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
UwbScheduledExtractor::publish_window(
    core::ScheduledWindowCore::WindowHandle handle)
{
    auto& slot = core_.window(handle);
    core::WindowMeta& meta = slot.meta;
    const size_t n = meta.sample_count;
    if (n == 0 || slot.samples.empty())
        return;

    // Optional post-copy verification (never gates every_slot emit).
    if (d_verification_enabled_ && !d_radar_tmpl_.empty()) {
        const size_t L = d_radar_tmpl_.size();
        const size_t search_start = 0;
        // Search only near predicted: pre_guard + small margin (not full window).
        const size_t pre = core_.config().pre_guard_samples;
        const size_t search_len =
            std::min(n > L ? n - L + 1 : 0, pre * 2 + 64);
        float metric = 0.0f;
        size_t off = 0;
        const bool ok = core::verify_template_in_window(
            slot.samples.data(), n, d_radar_tmpl_.data(), L, search_start,
            search_len, d_radar_threshold_, &metric, &off);
        meta.radar_metric = metric;
        meta.radar_verified = ok;
        if (ok) {
            meta.detected_start_sample =
                meta.window_start_sample + static_cast<int64_t>(off);
            meta.timing_error_samples =
                meta.detected_start_sample - meta.predicted_start_sample;
        } else {
            core_.note_verification_failure();
        }
    }

    if (d_verification_enabled_ && !d_comm_tmpl_.empty()) {
        const size_t L = d_comm_tmpl_.size();
        float metric = 0.0f;
        size_t off = 0;
        const size_t search_len = n > L ? n - L + 1 : 0;
        // Cap comm search to same short horizon as radar for cost control.
        const size_t pre = core_.config().pre_guard_samples;
        const size_t capped = std::min(search_len, pre * 2 + 64);
        const bool ok = core::verify_template_in_window(
            slot.samples.data(), n, d_comm_tmpl_.data(), L, 0, capped,
            d_comm_threshold_, &metric, &off);
        meta.comm_metric = metric;
        meta.comm_present = ok;
        meta.collision = meta.radar_verified && meta.comm_present;
        if (meta.collision)
            core_.note_collision();
    }

    if (d_emit_policy_ == EmitPolicy::VerifiedOnly && !meta.radar_verified) {
        // Explicit non-emit for production filter; research default is every_slot.
        return;
    }

    pmt::pmt_t pmt_meta = pmt::make_dict();
    pmt_meta = pmt::dict_add(pmt_meta, pmt::mp("packet_id"),
                             pmt::from_uint64(meta.schedule_index));
    pmt_meta = pmt::dict_add(pmt_meta, pmt::mp("schedule_index"),
                             pmt::from_uint64(meta.schedule_index));
    pmt_meta = pmt::dict_add(pmt_meta, pmt::mp("predicted_start_sample"),
                             pmt::from_long(meta.predicted_start_sample));
    pmt_meta = pmt::dict_add(pmt_meta, pmt::mp("window_start_sample"),
                             pmt::from_long(meta.window_start_sample));
    pmt_meta = pmt::dict_add(pmt_meta, pmt::mp("window_end_sample"),
                             pmt::from_long(meta.window_end_sample));
    pmt_meta = pmt::dict_add(pmt_meta, pmt::mp("sample_count"),
                             pmt::from_long(static_cast<long>(n)));
    pmt_meta = pmt::dict_add(
        pmt_meta, pmt::mp("sample_rate"),
        pmt::from_double(core_.config().sample_rate));
    pmt_meta = pmt::dict_add(
        pmt_meta, pmt::mp("packet_interval_s"),
        pmt::from_double(core_.config().packet_interval_s));
    pmt_meta = pmt::dict_add(
        pmt_meta, pmt::mp("pre_guard_samples"),
        pmt::from_long(static_cast<long>(core_.config().pre_guard_samples)));
    pmt_meta = pmt::dict_add(
        pmt_meta, pmt::mp("capture_samples"),
        pmt::from_long(static_cast<long>(core_.config().capture_samples)));
    pmt_meta = pmt::dict_add(pmt_meta, pmt::mp("radar_verified"),
                             pmt::from_bool(meta.radar_verified));
    pmt_meta = pmt::dict_add(pmt_meta, pmt::mp("radar_metric"),
                             pmt::from_double(meta.radar_metric));
    pmt_meta = pmt::dict_add(pmt_meta, pmt::mp("comm_present"),
                             pmt::from_bool(meta.comm_present));
    pmt_meta = pmt::dict_add(pmt_meta, pmt::mp("comm_metric"),
                             pmt::from_double(meta.comm_metric));
    pmt_meta = pmt::dict_add(pmt_meta, pmt::mp("collision"),
                             pmt::from_bool(meta.collision));
    pmt_meta = pmt::dict_add(pmt_meta, pmt::mp("partial"),
                             pmt::from_bool(meta.partial));
    pmt_meta = pmt::dict_add(pmt_meta, pmt::mp("detected_start_sample"),
                             pmt::from_long(meta.detected_start_sample));
    pmt_meta = pmt::dict_add(pmt_meta, pmt::mp("timing_error_samples"),
                             pmt::from_long(meta.timing_error_samples));
    pmt_meta = pmt::dict_add(pmt_meta, pmt::mp("schedule_generation"),
                             pmt::from_uint64(meta.schedule_generation));

    // PDU vector owns a copy of IQ (CF32).
    pmt::pmt_t vec =
        pmt::init_c32vector(n, reinterpret_cast<const gr_complex*>(
                                   slot.samples.data()));
    message_port_pub(pmt::mp("packet"), pmt::cons(pmt_meta, vec));
    d_emitted_.fetch_add(1, std::memory_order_relaxed);
    core_.note_emitted();
}

} // namespace uwb
} // namespace gr
