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
#include <cmath>
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
    message_port_register_in(pmt::mp("lock_obs"));
    set_msg_handler(pmt::mp("schedule"),
                    [this](pmt::pmt_t msg) { handle_schedule_msg(msg); });
    set_msg_handler(pmt::mp("lock_obs"),
                    [this](pmt::pmt_t msg) { handle_lock_obs_msg(msg); });

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
    d_lock_.reset(cfg.first_packet_sample_exact,
                  packet_interval_s * sample_rate);
    // Default OFF; enable via set_schedule_lock_enabled(true) and feed
    // SYNC/preamble timing into "lock_obs" (demod schedule_feedback preferred).
    d_lock_.enabled = false;
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

void
UwbScheduledExtractor::set_schedule_lock_enabled(bool en)
{
    std::lock_guard<std::mutex> lock(d_cfg_mutex_);
    d_lock_.enabled = en;
    if (!en) {
        const auto cfg = core_.config();
        d_lock_.reset(cfg.first_packet_sample_exact,
                      cfg.period_samples_exact());
        d_pending_lock_ = false;
    }
}

bool
UwbScheduledExtractor::schedule_lock_enabled() const
{
    std::lock_guard<std::mutex> lock(d_cfg_mutex_);
    return d_lock_.enabled;
}

int
UwbScheduledExtractor::schedule_lock_state() const
{
    std::lock_guard<std::mutex> lock(d_cfg_mutex_);
    return static_cast<int>(d_lock_.state);
}

uint64_t
UwbScheduledExtractor::schedule_lock_updates() const
{
    std::lock_guard<std::mutex> lock(d_cfg_mutex_);
    return d_lock_.lock_updates;
}

double
UwbScheduledExtractor::locked_packet_interval_s() const
{
    std::lock_guard<std::mutex> lock(d_cfg_mutex_);
    const double fs = core_.config().sample_rate;
    if (fs <= 0.0 || d_lock_.period_samples <= 0.0)
        return core_.config().packet_interval_s;
    return d_lock_.period_samples / fs;
}

double
UwbScheduledExtractor::locked_first_packet_sample() const
{
    std::lock_guard<std::mutex> lock(d_cfg_mutex_);
    return d_lock_.t0_exact;
}

double
UwbScheduledExtractor::locked_delta_period_samples() const
{
    std::lock_guard<std::mutex> lock(d_cfg_mutex_);
    return d_lock_.delta_period;
}

double
UwbScheduledExtractor::locked_bias_t0_samples() const
{
    std::lock_guard<std::mutex> lock(d_cfg_mutex_);
    return d_lock_.bias_t0;
}

void
UwbScheduledExtractor::observe_detection(uint64_t schedule_index,
                                         int64_t detected_start_sample)
{
    note_lock_observation(schedule_index, detected_start_sample);
}

int64_t
UwbScheduledExtractor::map_obs_sample_to_native(int64_t obs_sample,
                                                double obs_rate,
                                                double filter_delay) const
{
    const double native = core_.config().sample_rate;
    if (obs_sample < 0 || obs_rate <= 0.0 || native <= 0.0)
        return obs_sample;
    // Same domain.
    if (std::abs(obs_rate - native) <= 1e-6 * std::max(obs_rate, native))
        return obs_sample;

    // Preferred path: inverse of PDU 65/48 map
    //   out = round((in * 65 + gd) / 48)
    //   in  = round((out * 48 - gd) / 65)
    // when obs is 998.4e6 and native is 737.28e6 (ratio 65/48).
    constexpr double k998 = 998.4e6;
    constexpr double k737 = 737.28e6;
    const bool obs_998 =
        std::abs(obs_rate - k998) <= 1e-6 * k998;
    const bool nat_737 =
        std::abs(native - k737) <= 1e-6 * k737;
    if (obs_998 && nat_737) {
        const double gd = filter_delay; // samples at 998.4
        return static_cast<int64_t>(std::llround(
            (static_cast<double>(obs_sample) * 48.0 - gd) / 65.0));
    }
    // Generic fallback (no group-delay compensation).
    return static_cast<int64_t>(
        std::llround(static_cast<double>(obs_sample) * native / obs_rate));
}

bool
UwbScheduledExtractor::note_lock_observation(uint64_t schedule_index,
                                             int64_t detected_native)
{
    std::lock_guard<std::mutex> lock(d_cfg_mutex_);
    if (!d_lock_.enabled || detected_native < 0)
        return false;
    const auto cfg = core_.config();
    const auto prev_state = d_lock_.state;
    if (!d_lock_.observe(schedule_index, detected_native,
                         cfg.period_samples_exact()))
        return false;
    d_pending_lock_ = true;
    d_pending_lock_t0_ = d_lock_.t0_exact;
    d_pending_lock_period_s_ =
        d_lock_.period_samples / std::max(cfg.sample_rate, 1.0);
    // Force absolute learned (b, δ) only when transitioning INTO Hold (the big
    // jump to the learned t0).  Continuous Hold updates apply softly so the
    // imminent window does not jump under the slow tracking nudge.
    d_pending_lock_force_t0_ =
        (prev_state != core::ScheduleLockTracker::State::Hold &&
         d_lock_.state == core::ScheduleLockTracker::State::Hold);
    (void)prev_state;
    return true;
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

    // command == "observe" → demod / external lock observation
    if (cmd == "observe") {
        handle_lock_obs_msg(dict);
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
UwbScheduledExtractor::handle_lock_obs_msg(pmt::pmt_t msg)
{
    pmt::pmt_t dict = msg;
    if (pmt::is_pair(msg) && !pmt::is_dict(msg))
        dict = pmt::cdr(msg);
    // Also accept a result PDU: cons(meta, payload).
    if (pmt::is_pair(dict) && pmt::is_dict(pmt::car(dict)))
        dict = pmt::car(dict);
    if (!pmt::is_dict(dict))
        return;

    // SYNC-primary lock: accept timing-ok observations.  FCS is not required
    // (and is not a fast sensor).  Explicit timing_ok=false rejects the msg.
    // Without timing_ok, a present detected_start_sample is enough.
    if (pmt::dict_has_key(dict, pmt::mp("timing_ok"))) {
        if (!pmt::to_bool(
                pmt::dict_ref(dict, pmt::mp("timing_ok"), pmt::PMT_F)))
            return;
    }

    auto as_u64 = [](pmt::pmt_t v, uint64_t def) -> uint64_t {
        if (pmt::is_uint64(v))
            return pmt::to_uint64(v);
        if (pmt::is_integer(v))
            return static_cast<uint64_t>(pmt::to_long(v));
        return def;
    };
    auto as_i64 = [](pmt::pmt_t v, int64_t def) -> int64_t {
        if (pmt::is_uint64(v))
            return static_cast<int64_t>(pmt::to_uint64(v));
        if (pmt::is_integer(v))
            return pmt::to_long(v);
        return def;
    };
    auto as_f64 = [](pmt::pmt_t v, double def) -> double {
        if (pmt::is_real(v))
            return pmt::to_double(v);
        if (pmt::is_integer(v))
            return static_cast<double>(pmt::to_long(v));
        return def;
    };

    const uint64_t k = as_u64(
        pmt::dict_ref(dict, pmt::mp("schedule_index"),
                      pmt::dict_ref(dict, pmt::mp("packet_id"),
                                    pmt::from_uint64(0))),
        0);
    int64_t det = as_i64(
        pmt::dict_ref(dict, pmt::mp("detected_start_sample"),
                      pmt::dict_ref(dict, pmt::mp("timing_start_sample"),
                                    pmt::from_long(-1))),
        -1);
    if (det < 0)
        return;

    const double obs_rate = as_f64(
        pmt::dict_ref(dict, pmt::mp("sample_rate"), pmt::from_double(0.0)),
        0.0);
    double filter_delay = as_f64(
        pmt::dict_ref(dict, pmt::mp("resample_filter_delay"),
                      pmt::from_double(0.0)),
        0.0);
    // Prefer explicit native rate; fall back to input_sample_rate from the
    // PDU resampler provenance fields.
    const double native_hint = as_f64(
        pmt::dict_ref(
            dict, pmt::mp("native_sample_rate"),
            pmt::dict_ref(dict, pmt::mp("input_sample_rate"),
                          pmt::from_double(core_.config().sample_rate))),
        core_.config().sample_rate);
    (void)native_hint;

    if (obs_rate > 0.0)
        det = map_obs_sample_to_native(det, obs_rate, filter_delay);

    note_lock_observation(k, det);
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
    bool do_lock = false;
    double lock_t0 = 0.0;
    double lock_period_s = 0.0;
    core::ScheduleConfig pending;
    {
        std::lock_guard<std::mutex> lock(d_cfg_mutex_);
        do_reset = d_pending_reset_;
        do_cfg = d_pending_cfg_;
        do_pause = d_pending_pause_;
        do_resume = d_pending_resume_;
        do_lock = d_pending_lock_;
        lock_t0 = d_pending_lock_t0_;
        lock_period_s = d_pending_lock_period_s_;
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
            d_pending_lock_ = false;
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
            d_lock_.reset(d_pending_.first_packet_sample_exact,
                          d_pending_.packet_interval_s *
                              d_pending_.sample_rate);
            d_pending_cfg_ = false;
            d_pending_lock_ = false;
            publish_status("schedule_updated");
        }
        // Soft lock update: continuity-preserving at next_k (see core).
        if (d_pending_lock_) {
            const double t0 = d_pending_lock_t0_;
            const double Ts = d_pending_lock_period_s_;
            const bool force = d_pending_lock_force_t0_;
            d_pending_lock_ = false;
            d_pending_lock_force_t0_ = false;
            if (core_.update_locked_params(t0, Ts, force)) {
                // Keep tracker t0 aligned with what was actually applied
                // (continuity re-anchor may differ from tracker t0).
                d_lock_.t0_exact = core_.config().first_packet_sample_exact;
                d_lock_.period_samples = core_.config().period_samples_exact();
                pmt::pmt_t extra = pmt::make_dict();
                extra = pmt::dict_add(extra, pmt::mp("lock_state"),
                                      pmt::from_long(static_cast<long>(
                                          d_lock_.state)));
                extra = pmt::dict_add(
                    extra, pmt::mp("locked_t0"),
                    pmt::from_double(core_.config().first_packet_sample_exact));
                extra = pmt::dict_add(extra, pmt::mp("locked_period_s"),
                                      pmt::from_double(Ts));
                extra = pmt::dict_add(
                    extra, pmt::mp("locked_period_samples"),
                    pmt::from_double(d_lock_.period_samples));
                extra = pmt::dict_add(extra, pmt::mp("force_t0"),
                                      pmt::from_bool(force));
                publish_status("schedule_locked", extra);
            } else {
                publish_status("schedule_lock_rejected");
            }
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
    (void)do_lock;
    (void)lock_t0;
    (void)lock_period_s;
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
    // Search aggressively around the predicted start: real QM35 slots drift by
    // thousands of samples over 0.5 s under a fixed nominal T.  Stride-16
    // coarse + fine keeps cost bounded while covering ~2*pre_guard.
    if (d_verification_enabled_ && !d_radar_tmpl_.empty()) {
        const size_t L = d_radar_tmpl_.size();
        const size_t search_start = 0;
        const size_t pre = core_.config().pre_guard_samples;
        const size_t max_align = n > L ? n - L + 1 : 0;
        const size_t search_len =
            std::min(max_align, std::max(pre * 2 + 1024, pre + 8192));
        float metric = 0.0f;
        size_t off = 0;
        const bool ok = core::verify_template_in_window(
            slot.samples.data(), n, d_radar_tmpl_.data(), L, search_start,
            search_len, d_radar_threshold_, &metric, &off,
            /*coarse_stride=*/16);
        meta.radar_metric = metric;
        meta.radar_verified = ok;
        if (ok) {
            meta.detected_start_sample =
                meta.window_start_sample + static_cast<int64_t>(off);
            meta.timing_error_samples =
                meta.detected_start_sample - meta.predicted_start_sample;

            // Secondary SYNC source for learn-then-freeze (prefer demod
            // schedule_feedback).  Strong radar peaks only.
            if (metric >= 0.6f)
                note_lock_observation(meta.schedule_index,
                                      meta.detected_start_sample);
        } else {
            core_.note_verification_failure();
        }
    }

    if (d_verification_enabled_ && !d_comm_tmpl_.empty()) {
        const size_t L = d_comm_tmpl_.size();
        float metric = 0.0f;
        size_t off = 0;
        const size_t search_len = n > L ? n - L + 1 : 0;
        const size_t pre = core_.config().pre_guard_samples;
        const size_t capped =
            std::min(search_len, std::max(pre * 2 + 1024, pre + 8192));
        const bool ok = core::verify_template_in_window(
            slot.samples.data(), n, d_comm_tmpl_.data(), L, 0, capped,
            d_comm_threshold_, &metric, &off, /*coarse_stride=*/16);
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
    // Keep schedule predicted_start as the demod seed.  stage_timing now
    // coarse-scans a large margin around it; substituting a mid-preamble
    // verification peak (or a false lock) as the seed caused worse SFD/CIR
    // failures than the plain schedule + wide search path.
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
