/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gnuradio/uwb/uwb_auto_scheduled_extractor_sc16.h>
#include <gnuradio/io_signature.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <stdexcept>
#include <utility>

namespace gr {
namespace uwb {

namespace {

pmt::pmt_t dict_or_car(pmt::pmt_t msg)
{
    if (pmt::is_dict(msg))
        return msg;
    if (pmt::is_pair(msg) && pmt::is_dict(pmt::car(msg)))
        return pmt::car(msg);
    if (pmt::is_pair(msg) && pmt::is_dict(pmt::cdr(msg)))
        return pmt::cdr(msg);
    return pmt::PMT_NIL;
}

uint64_t as_u64(pmt::pmt_t v, uint64_t def)
{
    if (pmt::is_uint64(v))
        return pmt::to_uint64(v);
    if (pmt::is_integer(v))
        return static_cast<uint64_t>(pmt::to_long(v));
    return def;
}

int64_t as_i64(pmt::pmt_t v, int64_t def)
{
    if (pmt::is_uint64(v))
        return static_cast<int64_t>(pmt::to_uint64(v));
    if (pmt::is_integer(v))
        return pmt::to_long(v);
    return def;
}

double as_f64(pmt::pmt_t v, double def)
{
    if (pmt::is_real(v))
        return pmt::to_double(v);
    if (pmt::is_integer(v))
        return static_cast<double>(pmt::to_long(v));
    return def;
}

std::string as_str(pmt::pmt_t v)
{
    if (pmt::is_symbol(v))
        return pmt::symbol_to_string(v);
    if (pmt::is_bool(v))
        return pmt::to_bool(v) ? "true" : "false";
    return {};
}

bool as_bool(pmt::pmt_t v, bool def)
{
    if (pmt::is_bool(v))
        return pmt::to_bool(v);
    return def;
}

} // namespace

UwbAutoScheduledExtractorSc16::UwbAutoScheduledExtractorSc16(
    const std::vector<std::complex<float>>& known_preamble,
    double sample_rate,
    double packet_interval_s,
    size_t pre_guard_samples,
    size_t capture_samples,
    size_t post_guard_samples,
    float energy_threshold,
    size_t energy_gate_decimation,
    size_t coarse_decimation,
    size_t coarse_repetitions,
    size_t coarse_margin,
    size_t lock_observations,
    size_t holdover_miss_count,
    size_t reacquire_miss_count,
    double provisional_guard_us,
    size_t acquire_pre_trigger,
    size_t acquire_capture,
    size_t scheduled_pool_size)
    : gr::sync_block("uwb_auto_scheduled_extractor_sc16",
                     gr::io_signature::make(1, 1, sizeof(std::complex<int16_t>)),
                     gr::io_signature::make(0, 0, 0)),
      sm_(acquire_pre_trigger + 32 * energy_gate_decimation +
              known_preamble.size(),
          energy_threshold,
          energy_gate_decimation,
          /*gate_window=*/32,
          /*holdoff_decimated=*/8,
          /*post_trigger_capture=*/acquire_capture),
      d_sample_rate_(sample_rate),
      d_interval_s_(packet_interval_s),
      d_pre_(pre_guard_samples),
      d_cap_(capture_samples),
      d_post_(post_guard_samples),
      d_acq_pre_(acquire_pre_trigger),
      d_acq_cap_(acquire_capture),
      d_prov_guard_us_(provisional_guard_us)
{
    if (sample_rate <= 0.0 || packet_interval_s <= 0.0) {
        throw std::invalid_argument(
            "UwbAutoScheduledExtractorSc16: sample_rate and "
            "packet_interval_s must be > 0");
    }
    if (pre_guard_samples + capture_samples + post_guard_samples == 0) {
        throw std::invalid_argument(
            "UwbAutoScheduledExtractorSc16: window capacity must be > 0");
    }
    if (known_preamble.empty()) {
        throw std::invalid_argument(
            "UwbAutoScheduledExtractorSc16: known_preamble is empty");
    }

    set_max_noutput_items(1048576);
    message_port_register_out(pmt::mp("packet"));
    message_port_register_out(pmt::mp("status"));
    message_port_register_in(pmt::mp("lock_obs"));
    message_port_register_in(pmt::mp("control"));
    set_msg_handler(pmt::mp("lock_obs"),
                    [this](pmt::pmt_t msg) { handle_lock_obs_msg(msg); });
    set_msg_handler(pmt::mp("control"),
                    [this](pmt::pmt_t msg) { handle_control_msg(msg); });

    core::UwbPreambleVerifierSc16::Config vcfg;
    vcfg.coarse_decimation = coarse_decimation;
    vcfg.coarse_repetitions = coarse_repetitions;
    vcfg.coarse_margin = coarse_margin;
    verifier_.configure(known_preamble, vcfg);
    const size_t region_pre =
        acquire_pre_trigger + 32 * energy_gate_decimation +
        known_preamble.size();
    verifier_.reserve_coarse(region_pre + known_preamble.size() *
                                              (core::kUwbSyncSymbols +
                                               core::kUwbSfdSymbols + 4));

    core::Qm35AcquisitionConfig acfg;
    acfg.sample_rate = sample_rate;
    acfg.nominal_interval_s = packet_interval_s;
    acfg.lock_observations = lock_observations;
    acfg.holdover_miss_count = holdover_miss_count;
    acfg.reacquire_miss_count = reacquire_miss_count;
    acfg.provisional_guard_us = provisional_guard_us;
    tracker_.configure(acfg);

    core::ScheduleConfig scfg;
    scfg.sample_rate = sample_rate;
    scfg.packet_interval_s = packet_interval_s;
    scfg.first_packet_sample_exact = 0.0;
    scfg.pre_guard_samples = pre_guard_samples;
    scfg.capture_samples = capture_samples;
    scfg.post_guard_samples = post_guard_samples;
    scfg.pool_size = scheduled_pool_size > 0 ? scheduled_pool_size : 8;
    scfg.emit_policy = core::EmitPolicy::EverySlot;
    sched_.set_sc16_windows(true);
    sched_.configure(scfg);
}

UwbAutoScheduledExtractorSc16::~UwbAutoScheduledExtractorSc16()
{
    shutdown_worker();
}

std::vector<std::complex<float>>
UwbAutoScheduledExtractorSc16::load_template(const std::string& path)
{
    std::ifstream f(path, std::ios::binary);
    if (!f) {
        throw std::runtime_error(
            "UwbAutoScheduledExtractorSc16: cannot open template " + path);
    }
    f.seekg(0, std::ios::end);
    const std::streamsize bytes = f.tellg();
    f.seekg(0, std::ios::beg);
    if (bytes <= 0 || bytes % sizeof(gr_complex) != 0) {
        throw std::runtime_error(
            "UwbAutoScheduledExtractorSc16: template file has invalid size");
    }
    std::vector<std::complex<float>> tmpl(
        static_cast<size_t>(bytes) / sizeof(gr_complex));
    f.read(reinterpret_cast<char*>(tmpl.data()), bytes);
    return tmpl;
}

std::shared_ptr<UwbAutoScheduledExtractorSc16>
UwbAutoScheduledExtractorSc16::make(
    const std::vector<std::complex<float>>& known_preamble,
    double sample_rate,
    double packet_interval_s,
    size_t pre_guard_samples,
    size_t capture_samples,
    size_t post_guard_samples,
    float energy_threshold,
    size_t energy_gate_decimation,
    size_t coarse_decimation,
    size_t coarse_repetitions,
    size_t coarse_margin,
    size_t lock_observations,
    size_t holdover_miss_count,
    size_t reacquire_miss_count,
    double provisional_guard_us,
    size_t acquire_pre_trigger,
    size_t acquire_capture,
    size_t scheduled_pool_size)
{
    return gnuradio::get_initial_sptr(new UwbAutoScheduledExtractorSc16(
        known_preamble, sample_rate, packet_interval_s, pre_guard_samples,
        capture_samples, post_guard_samples, energy_threshold,
        energy_gate_decimation, coarse_decimation, coarse_repetitions,
        coarse_margin, lock_observations, holdover_miss_count,
        reacquire_miss_count, provisional_guard_us, acquire_pre_trigger,
        acquire_capture, scheduled_pool_size));
}

std::shared_ptr<UwbAutoScheduledExtractorSc16>
UwbAutoScheduledExtractorSc16::make_from_file(
    const std::string& template_file,
    double sample_rate,
    double packet_interval_s,
    size_t pre_guard_samples,
    size_t capture_samples,
    size_t post_guard_samples,
    float energy_threshold,
    size_t energy_gate_decimation,
    size_t coarse_decimation,
    size_t coarse_repetitions,
    size_t coarse_margin,
    size_t lock_observations,
    size_t holdover_miss_count,
    size_t reacquire_miss_count,
    double provisional_guard_us,
    size_t acquire_pre_trigger,
    size_t acquire_capture,
    size_t scheduled_pool_size)
{
    return make(load_template(template_file), sample_rate, packet_interval_s,
                pre_guard_samples, capture_samples, post_guard_samples,
                energy_threshold, energy_gate_decimation, coarse_decimation,
                coarse_repetitions, coarse_margin, lock_observations,
                holdover_miss_count, reacquire_miss_count, provisional_guard_us,
                acquire_pre_trigger, acquire_capture, scheduled_pool_size);
}

int UwbAutoScheduledExtractorSc16::lock_state() const
{
    std::lock_guard<std::mutex> lock(d_cfg_mutex_);
    return static_cast<int>(tracker_.state());
}

const char* UwbAutoScheduledExtractorSc16::lock_state_name() const
{
    std::lock_guard<std::mutex> lock(d_cfg_mutex_);
    return core::qm35_lock_state_name(tracker_.state());
}

uint64_t UwbAutoScheduledExtractorSc16::schedule_generation() const
{
    std::lock_guard<std::mutex> lock(d_cfg_mutex_);
    return tracker_.generation();
}

uint64_t UwbAutoScheduledExtractorSc16::acquisition_epoch() const
{
    std::lock_guard<std::mutex> lock(d_cfg_mutex_);
    return tracker_.epoch();
}

bool UwbAutoScheduledExtractorSc16::identity_confirmed() const
{
    std::lock_guard<std::mutex> lock(d_cfg_mutex_);
    return tracker_.identity_confirmed();
}

double UwbAutoScheduledExtractorSc16::locked_t0() const
{
    std::lock_guard<std::mutex> lock(d_cfg_mutex_);
    return tracker_.t0_exact();
}

double UwbAutoScheduledExtractorSc16::locked_period_s() const
{
    std::lock_guard<std::mutex> lock(d_cfg_mutex_);
    return tracker_.period_s();
}

uint64_t UwbAutoScheduledExtractorSc16::energy_regions() const
{
    return d_energy_regions_;
}

uint64_t UwbAutoScheduledExtractorSc16::energy_regions_after_lock() const
{
    return d_energy_after_lock_.load(std::memory_order_relaxed);
}

uint64_t UwbAutoScheduledExtractorSc16::candidates_emitted() const
{
    return d_candidates_.load(std::memory_order_relaxed);
}

uint64_t UwbAutoScheduledExtractorSc16::candidates_rejected() const
{
    return d_rejected_.load(std::memory_order_relaxed);
}

uint64_t UwbAutoScheduledExtractorSc16::scheduled_windows() const
{
    return sched_.stats().scheduled_windows;
}

uint64_t UwbAutoScheduledExtractorSc16::emitted_windows() const
{
    return d_emitted_.load(std::memory_order_relaxed);
}

uint64_t UwbAutoScheduledExtractorSc16::dropped_windows() const
{
    return sched_.stats().dropped_windows +
           d_pool_drops_.load(std::memory_order_relaxed) +
           d_queue_full_.load(std::memory_order_relaxed);
}

uint64_t UwbAutoScheduledExtractorSc16::pool_drops() const
{
    return d_pool_drops_.load(std::memory_order_relaxed);
}

uint64_t UwbAutoScheduledExtractorSc16::queue_full_drops() const
{
    return d_queue_full_.load(std::memory_order_relaxed);
}

uint64_t UwbAutoScheduledExtractorSc16::stale_feedback() const
{
    std::lock_guard<std::mutex> lock(d_cfg_mutex_);
    return tracker_.stale_feedback();
}

uint64_t UwbAutoScheduledExtractorSc16::unmapped_feedback() const
{
    std::lock_guard<std::mutex> lock(d_cfg_mutex_);
    return tracker_.unmapped_rejected();
}

uint64_t UwbAutoScheduledExtractorSc16::discontinuities() const
{
    return d_discontinuities_.load(std::memory_order_relaxed);
}

uint64_t UwbAutoScheduledExtractorSc16::current_sample() const
{
    return d_current_sample_;
}

void UwbAutoScheduledExtractorSc16::post_lock_obs(pmt::pmt_t msg)
{
    handle_lock_obs_msg(msg);
}

void UwbAutoScheduledExtractorSc16::post_control(pmt::pmt_t msg)
{
    handle_control_msg(msg);
}

void UwbAutoScheduledExtractorSc16::handle_lock_obs_msg(pmt::pmt_t msg)
{
    const pmt::pmt_t dict = dict_or_car(msg);
    if (!pmt::is_dict(dict))
        return;

    core::Qm35LockObservation obs;
    if (pmt::dict_has_key(dict, pmt::mp("packet_id")))
        obs.packet_id = as_u64(
            pmt::dict_ref(dict, pmt::mp("packet_id"), pmt::from_uint64(0)), 0);
    if (pmt::dict_has_key(dict, pmt::mp("acquisition_epoch"))) {
        obs.has_epoch = true;
        obs.acquisition_epoch = as_u64(
            pmt::dict_ref(dict, pmt::mp("acquisition_epoch"),
                          pmt::from_uint64(0)),
            0);
    }
    if (pmt::dict_has_key(dict, pmt::mp("schedule_generation"))) {
        obs.has_generation = true;
        obs.schedule_generation = as_u64(
            pmt::dict_ref(dict, pmt::mp("schedule_generation"),
                          pmt::from_uint64(0)),
            0);
    }
    if (pmt::dict_has_key(dict, pmt::mp("schedule_index"))) {
        obs.schedule_index = as_u64(
            pmt::dict_ref(dict, pmt::mp("schedule_index"),
                          pmt::from_uint64(0)),
            0);
    }
    obs.detected_start_sample = as_i64(
        pmt::dict_ref(dict, pmt::mp("detected_start_sample"),
                      pmt::dict_ref(dict, pmt::mp("timing_start_sample"),
                                    pmt::from_long(-1))),
        -1);
    if (pmt::dict_has_key(dict, pmt::mp("sample_rate"))) {
        obs.has_sample_rate = true;
        obs.sample_rate = as_f64(
            pmt::dict_ref(dict, pmt::mp("sample_rate"), pmt::from_double(0.0)),
            0.0);
    }
    if (pmt::dict_has_key(dict, pmt::mp("native_sample_rate")) ||
        pmt::dict_has_key(dict, pmt::mp("input_sample_rate"))) {
        obs.has_native_sample_rate = true;
        obs.native_sample_rate = as_f64(
            pmt::dict_ref(dict, pmt::mp("native_sample_rate"),
                          pmt::dict_ref(dict, pmt::mp("input_sample_rate"),
                                        pmt::from_double(0.0))),
            0.0);
    }
    if (pmt::dict_has_key(dict, pmt::mp("resample_filter_delay"))) {
        obs.has_filter_delay = true;
        obs.resample_filter_delay = as_f64(
            pmt::dict_ref(dict, pmt::mp("resample_filter_delay"),
                          pmt::from_double(0.0)),
            0.0);
    }
    if (pmt::dict_has_key(dict, pmt::mp("timing_ok"))) {
        obs.has_timing_ok = true;
        obs.timing_ok = as_bool(
            pmt::dict_ref(dict, pmt::mp("timing_ok"), pmt::PMT_F), false);
    }
    if (pmt::dict_has_key(dict, pmt::mp("fcs_pass"))) {
        obs.fcs_pass = as_bool(
            pmt::dict_ref(dict, pmt::mp("fcs_pass"), pmt::PMT_F), false);
    }
    if (pmt::dict_has_key(dict, pmt::mp("status"))) {
        obs.status =
            as_str(pmt::dict_ref(dict, pmt::mp("status"), pmt::PMT_NIL));
    }
    if (pmt::dict_has_key(dict, pmt::mp("code_index"))) {
        obs.code_index = static_cast<int>(as_i64(
            pmt::dict_ref(dict, pmt::mp("code_index"), pmt::from_long(-1)),
            -1));
    }
    if (pmt::dict_has_key(dict, pmt::mp("preamble_repetitions"))) {
        obs.preamble_repetitions = static_cast<int>(as_i64(
            pmt::dict_ref(dict, pmt::mp("preamble_repetitions"),
                          pmt::from_long(-1)),
            -1));
    }
    if (pmt::dict_has_key(dict, pmt::mp("sfd_mode"))) {
        obs.sfd_mode =
            as_str(pmt::dict_ref(dict, pmt::mp("sfd_mode"), pmt::PMT_NIL));
    }

    std::lock_guard<std::mutex> lock(d_cfg_mutex_);
    if (d_pending_obs_n_ < kPendingObsCap)
        d_pending_obs_[d_pending_obs_n_++] = obs;
}

void UwbAutoScheduledExtractorSc16::handle_control_msg(pmt::pmt_t msg)
{
    const pmt::pmt_t dict = dict_or_car(msg);
    if (!pmt::is_dict(dict))
        return;
    const pmt::pmt_t cmd_p =
        pmt::dict_ref(dict, pmt::mp("command"), pmt::PMT_NIL);
    std::string cmd = as_str(cmd_p);
    if (cmd.empty() && pmt::dict_has_key(dict, pmt::mp("event")))
        cmd = as_str(pmt::dict_ref(dict, pmt::mp("event"), pmt::PMT_NIL));

    std::lock_guard<std::mutex> lock(d_cfg_mutex_);
    if (cmd == "discontinuity" || cmd == "overflow" || cmd == "rx_discontinuity") {
        d_pending_disc_ = true;
        d_pending_disc_reason_ = cmd.empty() ? "control" : cmd;
        return;
    }
    if (cmd == "reset") {
        d_pending_reset_ = true;
        return;
    }
}

void UwbAutoScheduledExtractorSc16::apply_pending()
{
    bool do_disc = false;
    bool do_reset = false;
    std::array<core::Qm35LockObservation, kPendingObsCap> obs_q{};
    size_t n_obs = 0;
    std::string disc_reason;
    {
        std::lock_guard<std::mutex> lock(d_cfg_mutex_);
        n_obs = d_pending_obs_n_;
        for (size_t i = 0; i < n_obs; ++i)
            obs_q[i] = d_pending_obs_[i];
        d_pending_obs_n_ = 0;
        do_disc = d_pending_disc_;
        do_reset = d_pending_reset_;
        disc_reason = d_pending_disc_reason_;
        d_pending_disc_ = false;
        d_pending_reset_ = false;
    }

    if (do_reset) {
        enqueue_ready_jobs();
        wait_for_worker_idle();
        sm_.reset();
        sched_.reset();
        {
            std::lock_guard<std::mutex> lock(d_cfg_mutex_);
            tracker_.configure(tracker_.config());
            d_last_applied_state_ = tracker_.state();
        }
        publish_status("acquisition_started");
    }

    if (do_disc) {
        handle_discontinuity(disc_reason.c_str());
        // Observations queued with the disc (in-flight demod feedback
        // from the dead schedule) must not re-seed t0.
        n_obs = 0;
    }

    for (size_t i = 0; i < n_obs; ++i)
        apply_one_obs(obs_q[i]);
}

void UwbAutoScheduledExtractorSc16::apply_one_obs(
    const core::Qm35LockObservation& obs)
{
    int64_t native = -1;
    bool mapped = false;
    const bool ok = core::classify_lock_obs_mapping(
        obs, d_sample_rate_, &native, &mapped);
    core::Qm35ObsAction action;
    LockState prev;
    {
        std::lock_guard<std::mutex> lock(d_cfg_mutex_);
        prev = tracker_.state();
        if (!ok) {
            action = tracker_.on_lock_obs(obs, -1);
        } else {
            action = tracker_.on_lock_obs(obs, native);
        }
    }

    if (action == core::Qm35ObsAction::RejectedUnmapped) {
        publish_status("candidate_rejected");
        return;
    }
    if (action == core::Qm35ObsAction::IgnoredStale) {
        pmt::pmt_t extra = pmt::make_dict();
        extra = pmt::dict_add(extra, pmt::mp("reason"), pmt::mp("stale"));
        publish_status("candidate_rejected", extra);
        return;
    }
    if (action == core::Qm35ObsAction::RejectedIdentity) {
        d_rejected_.fetch_add(1, std::memory_order_relaxed);
        publish_status("candidate_rejected");
        return;
    }
    if (action == core::Qm35ObsAction::ConfirmedIdentity) {
        apply_identity_schedule();
        publish_status("qm35_identity_confirmed");
        publish_status("provisional_schedule_started");
        return;
    }
    if (action == core::Qm35ObsAction::TimingUpdate) {
        std::lock_guard<std::mutex> lock(d_cfg_mutex_);
        if (tracker_.scheduled_path_active()) {
            sched_.update_locked_params(tracker_.t0_exact(),
                                        tracker_.period_s(),
                                        /*force_t0=*/false);
        }
        if (prev != LockState::Locked &&
            tracker_.state() == LockState::Locked) {
            d_energy_regions_at_lock_ = d_energy_regions_;
            apply_guard_for_state();
            publish_status("schedule_locked");
        } else if (prev == LockState::Holdover &&
                   tracker_.state() == LockState::Locked) {
            apply_guard_for_state();
            publish_status("schedule_locked");
        }
        return;
    }
    if (action == core::Qm35ObsAction::TimingMiss) {
        std::lock_guard<std::mutex> lock(d_cfg_mutex_);
        if (prev != tracker_.state()) {
            apply_guard_for_state();
            if (tracker_.state() == LockState::Holdover)
                publish_status("schedule_holdover");
            else if (tracker_.state() == LockState::Reacquire) {
                if (tracker_.global_energy()) {
                    sched_.pause();
                    sm_.reset();
                    publish_status("reacquisition_started");
                } else {
                    publish_status("reacquisition_started");
                }
            }
        }
    }
}

void UwbAutoScheduledExtractorSc16::apply_identity_schedule()
{
    enqueue_ready_jobs();
    wait_for_worker_idle();
    sm_.reset();

    const double t0 = tracker_.t0_exact();
    const double Ts = tracker_.period_s();
    const size_t extra = static_cast<size_t>(
        std::llround(d_prov_guard_us_ * 1e-6 * d_sample_rate_));
    core::ScheduleConfig cfg = sched_.config();
    cfg.first_packet_sample_exact = t0;
    cfg.packet_interval_s = Ts;
    cfg.sample_rate = d_sample_rate_;
    cfg.pre_guard_samples = d_pre_ + extra;
    cfg.capture_samples = d_cap_;
    cfg.post_guard_samples = d_post_ + extra;
    cfg.emit_policy = core::EmitPolicy::EverySlot;
    sched_.set_sc16_windows(true);
    sched_.configure(cfg);
    sched_.set_abs_cursor(d_current_sample_);
    sched_.set_schedule(t0, Ts, d_sample_rate_);
    d_last_applied_state_ = LockState::ProvisionalTrack;
}

void UwbAutoScheduledExtractorSc16::apply_guard_for_state()
{
    const auto st = tracker_.state();
    size_t extra = 0;
    if (st == LockState::ProvisionalTrack || st == LockState::Holdover ||
        (st == LockState::Reacquire && !tracker_.global_energy())) {
        extra = static_cast<size_t>(
            std::llround(d_prov_guard_us_ * 1e-6 * d_sample_rate_));
    }
    sched_.update_guards_keep_generation(d_pre_ + extra, d_cap_, d_post_ + extra);
    d_last_applied_state_ = st;
}

void UwbAutoScheduledExtractorSc16::scan_rx_time_tags(int nitems)
{
    if (nitems <= 0)
        return;
    const uint64_t abs0 = nitems_read(0);
    std::vector<gr::tag_t> tags;
    get_tags_in_window(tags, 0, 0, nitems);
    for (const auto& tag : tags) {
        const std::string key = pmt::symbol_to_string(tag.key);
        if (key == "rx_rate" && pmt::is_real(tag.value)) {
            d_rx_rate_ = pmt::to_double(tag.value);
            continue;
        }
        if (key != "rx_time")
            continue;
        double t_s = 0.0;
        if (pmt::is_tuple(tag.value) && pmt::length(tag.value) >= 2) {
            t_s = static_cast<double>(pmt::to_uint64(pmt::tuple_ref(tag.value, 0))) +
                  pmt::to_double(pmt::tuple_ref(tag.value, 1));
        } else if (pmt::is_pair(tag.value)) {
            t_s = static_cast<double>(as_u64(pmt::car(tag.value), 0)) +
                  as_f64(pmt::cdr(tag.value), 0.0);
        } else {
            continue;
        }
        const uint64_t abs = tag.offset;
        if (d_have_rx_time_) {
            const double dt = t_s - d_rx_time_s_;
            const double rate =
                d_rx_rate_ > 0.0 ? d_rx_rate_ : d_sample_rate_;
            const double expected =
                static_cast<double>(abs - d_rx_time_abs_) / rate;
            // Half a native period is already a hard discontinuity.
            const double tol = std::max(1e-6, 0.5 * d_interval_s_);
            if (std::abs(dt - expected) > tol) {
                handle_discontinuity("rx_time");
            }
        }
        d_have_rx_time_ = true;
        d_rx_time_abs_ = abs;
        d_rx_time_s_ = t_s;
        (void)abs0;
    }
}

void UwbAutoScheduledExtractorSc16::handle_discontinuity(const char* reason)
{
    enqueue_ready_jobs();
    wait_for_worker_idle();
    {
        std::lock_guard<std::mutex> lock(d_cfg_mutex_);
        tracker_.note_discontinuity();
        d_pending_obs_n_ = 0;
    }
    sched_.reset();
    sched_.pause();
    sm_.reset();
    d_discontinuities_.fetch_add(1, std::memory_order_relaxed);
    pmt::pmt_t extra = pmt::make_dict();
    extra = pmt::dict_add(extra, pmt::mp("reason"),
                          pmt::string_to_symbol(reason ? reason : "unknown"));
    extra = pmt::dict_add(extra, pmt::mp("schedule_generation"),
                          pmt::from_uint64(tracker_.generation()));
    publish_status("rx_discontinuity", extra);
    publish_status("schedule_lost", extra);
    publish_status("reacquisition_started", extra);
    d_last_applied_state_ = LockState::Reacquire;
}

void UwbAutoScheduledExtractorSc16::enqueue_ready_jobs()
{
    while (sm_.region_ready()) {
        const auto handle = sm_.take_region();
        ++d_energy_regions_;
        if (tracker_.scheduled_path_active() &&
            tracker_.state() == LockState::Locked) {
            d_energy_after_lock_.fetch_add(1, std::memory_order_relaxed);
        }
        bool queued = false;
        {
            std::lock_guard<std::mutex> lock(d_job_mutex_);
            if (d_job_count_ < kJobQueueSize) {
                d_job_queue_[d_job_tail_] = { JobKind::Acquisition, handle };
                d_job_tail_ = (d_job_tail_ + 1) % kJobQueueSize;
                ++d_job_count_;
                queued = true;
            }
        }
        if (queued) {
            d_job_cv_.notify_one();
        } else {
            sm_.release_region(handle);
            d_queue_full_.fetch_add(1, std::memory_order_relaxed);
            d_pool_drops_.fetch_add(1, std::memory_order_relaxed);
            publish_status("queue_full");
        }
    }
    while (sched_.window_ready()) {
        const auto handle = sched_.take_window();
        bool queued = false;
        {
            std::lock_guard<std::mutex> lock(d_job_mutex_);
            if (d_job_count_ < kJobQueueSize) {
                d_job_queue_[d_job_tail_] = { JobKind::Scheduled, handle };
                d_job_tail_ = (d_job_tail_ + 1) % kJobQueueSize;
                ++d_job_count_;
                queued = true;
            }
        }
        if (queued) {
            d_job_cv_.notify_one();
        } else {
            sched_.release_window(handle);
            d_queue_full_.fetch_add(1, std::memory_order_relaxed);
            d_pool_drops_.fetch_add(1, std::memory_order_relaxed);
            publish_status("window_dropped");
            publish_status("queue_full");
        }
    }
}

int UwbAutoScheduledExtractorSc16::work(int noutput_items,
                                        gr_vector_const_void_star& input_items,
                                        gr_vector_void_star&)
{
    apply_pending();
    scan_rx_time_tags(noutput_items);

    const auto* in =
        reinterpret_cast<const std::complex<int16_t>*>(input_items[0]);
    const int consumed =
        (noutput_items > 1) ? noutput_items - 1 : noutput_items;

    if (consumed > 0) {
        bool energy = false;
        bool scheduled = false;
        {
            std::lock_guard<std::mutex> lock(d_cfg_mutex_);
            energy = tracker_.energy_path_active();
            scheduled = tracker_.scheduled_path_active();
        }
        if (energy) {
            sm_.process(in, static_cast<size_t>(consumed), d_current_sample_);
        }
        if (scheduled) {
            sched_.process_chunk_sc16(
                in, static_cast<size_t>(consumed), d_current_sample_);
        }
        d_current_sample_ += static_cast<uint64_t>(consumed);
    }

    enqueue_ready_jobs();
    // While acquiring, let the worker publish the candidate so lock_obs can
    // land before the next chunk.  Locked scheduled mode never waits.
    if (tracker_.energy_path_active())
        wait_for_worker_idle();
    // Apply obs/control posted by this chunk's PDUs on the chunk boundary.
    // After a discontinuity this is what drops queued demod-shaped lock_obs
    // before they can re-confirm the dead native t0.
    apply_pending();
    if (noutput_items == 1) {
        wait_for_worker_idle();
        apply_pending();
    }
    return consumed;
}

bool UwbAutoScheduledExtractorSc16::start()
{
    std::lock_guard<std::mutex> lock(d_job_mutex_);
    d_job_head_ = d_job_tail_ = d_job_count_ = d_jobs_in_flight_ = 0;
    d_worker_stop_ = false;
    d_worker_ = std::thread(&UwbAutoScheduledExtractorSc16::worker_loop, this);
    publish_status("acquisition_started");
    return true;
}

bool UwbAutoScheduledExtractorSc16::stop()
{
    apply_pending();
    sm_.flush_region();
    sched_.flush_eos();
    enqueue_ready_jobs();
    wait_for_worker_idle();
    // PDUs flushed above may have posted disc / demod-shaped lock_obs.
    apply_pending();
    shutdown_worker();
    return true;
}

void UwbAutoScheduledExtractorSc16::worker_loop()
{
    for (;;) {
        Job job;
        {
            std::unique_lock<std::mutex> lock(d_job_mutex_);
            d_job_cv_.wait(lock, [this] {
                return d_worker_stop_ || d_job_count_ > 0;
            });
            if (d_job_count_ == 0 && d_worker_stop_)
                return;
            job = d_job_queue_[d_job_head_];
            d_job_head_ = (d_job_head_ + 1) % kJobQueueSize;
            --d_job_count_;
            ++d_jobs_in_flight_;
        }
        try {
            if (job.kind == JobKind::Acquisition)
                publish_acquisition(job.handle);
            else
                publish_scheduled(job.handle);
        } catch (const std::exception& error) {
            d_logger->error("UwbAutoScheduledExtractorSc16 worker failed: {}",
                            std::string(error.what()));
        } catch (...) {
            d_logger->error(
                "UwbAutoScheduledExtractorSc16 worker failed with unknown "
                "exception");
        }
        if (job.kind == JobKind::Acquisition)
            sm_.release_region(job.handle);
        else
            sched_.release_window(job.handle);
        {
            std::lock_guard<std::mutex> lock(d_job_mutex_);
            --d_jobs_in_flight_;
        }
        d_job_cv_.notify_all();
    }
}

void UwbAutoScheduledExtractorSc16::shutdown_worker()
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

void UwbAutoScheduledExtractorSc16::wait_for_worker_idle()
{
    std::unique_lock<std::mutex> lock(d_job_mutex_);
    d_job_cv_.wait(lock, [this] {
        return d_job_count_ == 0 && d_jobs_in_flight_ == 0;
    });
}

void UwbAutoScheduledExtractorSc16::publish_status(const std::string& event,
                                                   pmt::pmt_t extra)
{
    pmt::pmt_t meta = pmt::make_dict();
    meta = pmt::dict_add(meta, pmt::mp("event"), pmt::string_to_symbol(event));
    meta = pmt::dict_add(meta, pmt::mp("abs_sample"),
                         pmt::from_uint64(d_current_sample_));
    meta = pmt::dict_add(
        meta, pmt::mp("lock_state"),
        pmt::string_to_symbol(core::qm35_lock_state_name(tracker_.state())));
    meta = pmt::dict_add(meta, pmt::mp("acquisition_epoch"),
                         pmt::from_uint64(tracker_.epoch()));
    meta = pmt::dict_add(meta, pmt::mp("schedule_generation"),
                         pmt::from_uint64(tracker_.generation()));
    if (!pmt::eq(extra, pmt::PMT_NIL) && pmt::is_dict(extra)) {
        pmt::pmt_t items = pmt::dict_items(extra);
        for (size_t i = 0; i < pmt::length(items); ++i) {
            pmt::pmt_t kv = pmt::nth(i, items);
            meta = pmt::dict_add(meta, pmt::car(kv), pmt::cdr(kv));
        }
    }
    message_port_pub(pmt::mp("status"), meta);
}

void UwbAutoScheduledExtractorSc16::publish_acquisition(
    UwbDetectorStateMachineSc16::RegionHandle handle)
{
    const auto& region = sm_.region(handle);
    const size_t n = region.samples.size();
    if (n == 0)
        return;
    const auto vr = verifier_.verify(
        region.samples.data(), n, region.candidate_offset);
    if (!vr.confirmed)
        return;

    const uint64_t packet_start =
        region.start_abs + static_cast<uint64_t>(vr.start_offset);
    const uint64_t timing_seed =
        region.start_abs + static_cast<uint64_t>(vr.confirmed_offset);
    const uint64_t trigger = region.start_abs + region.candidate_offset;
    const uint64_t lo = (packet_start >= d_acq_pre_)
                            ? packet_start - d_acq_pre_
                            : 0;
    const size_t lo_off =
        (lo >= region.start_abs) ? static_cast<size_t>(lo - region.start_abs)
                                 : 0;
    const size_t cap = std::min(d_acq_pre_ + d_acq_cap_, n - lo_off);

    {
        std::lock_guard<std::mutex> lock(d_cfg_mutex_);
        tracker_.note_candidate_emitted(static_cast<int64_t>(packet_start));
    }
    d_candidates_.fetch_add(1, std::memory_order_relaxed);

    pmt::pmt_t meta = pmt::make_dict();
    meta = pmt::dict_add(meta, pmt::mp("packet_id"),
                         pmt::from_uint64(d_packet_id_++));
    meta = pmt::dict_add(meta, pmt::mp("capture_mode"), pmt::mp("acquisition"));
    meta = pmt::dict_add(meta, pmt::mp("acquisition_epoch"),
                         pmt::from_uint64(tracker_.epoch()));
    meta = pmt::dict_add(meta, pmt::mp("schedule_generation"),
                         pmt::from_uint64(tracker_.generation()));
    meta = pmt::dict_add(meta, pmt::mp("start_sample"),
                         pmt::from_uint64(packet_start));
    meta = pmt::dict_add(meta, pmt::mp("predicted_start_sample"),
                         pmt::from_uint64(timing_seed));
    meta = pmt::dict_add(meta, pmt::mp("timing_seed_sample"),
                         pmt::from_uint64(timing_seed));
    meta = pmt::dict_add(meta, pmt::mp("window_start_sample"),
                         pmt::from_uint64(lo));
    meta = pmt::dict_add(meta, pmt::mp("trigger_sample"),
                         pmt::from_uint64(trigger));
    meta = pmt::dict_add(meta, pmt::mp("sample_rate"),
                         pmt::from_double(d_sample_rate_));
    meta = pmt::dict_add(meta, pmt::mp("sample_format"), pmt::mp("sc16"));
    meta = pmt::dict_add(meta, pmt::mp("sample_count"),
                         pmt::from_long(static_cast<long>(cap)));
    meta = pmt::dict_add(meta, pmt::mp("detection_metric"),
                         pmt::from_double(vr.detection_metric));
    meta = pmt::dict_add(meta, pmt::mp("start_metric"),
                         pmt::from_double(vr.start_metric));
    meta = pmt::dict_add(meta, pmt::mp("start_backtracked_symbols"),
                         pmt::from_long(static_cast<long>(vr.backtracked_symbols)));
    meta = pmt::dict_add(meta, pmt::mp("packet_interval_s"),
                         pmt::from_double(d_interval_s_));
    meta = pmt::dict_add(meta, pmt::mp("lock_state"),
                         pmt::string_to_symbol(core::qm35_lock_state_name(
                             tracker_.state())));
    if (d_have_rx_time_) {
        const double t =
            d_rx_time_s_ +
            static_cast<double>(static_cast<int64_t>(lo) -
                                static_cast<int64_t>(d_rx_time_abs_)) /
                d_sample_rate_;
        meta = pmt::dict_add(meta, pmt::mp("rx_time_s"), pmt::from_double(t));
    }

    static_assert(sizeof(std::complex<int16_t>) == 2 * sizeof(int16_t),
                  "SC16 complex samples must be packed I/Q");
    const int16_t* iq =
        reinterpret_cast<const int16_t*>(region.samples.data() + lo_off);
    pmt::pmt_t data = pmt::init_s16vector(cap * 2, iq);
    message_port_pub(pmt::mp("packet"), pmt::cons(meta, data));
    d_emitted_.fetch_add(1, std::memory_order_relaxed);
    publish_status("candidate_emitted");
}

void UwbAutoScheduledExtractorSc16::publish_scheduled(
    core::ScheduledWindowCore::WindowHandle handle)
{
    auto& slot = sched_.window(handle);
    core::WindowMeta& wmeta = slot.meta;
    const size_t n = wmeta.sample_count;
    if (n == 0 || slot.samples_sc16.empty())
        return;

    pmt::pmt_t meta = pmt::make_dict();
    meta = pmt::dict_add(meta, pmt::mp("packet_id"),
                         pmt::from_uint64(d_packet_id_++));
    const char* mode = tracker_.capture_mode_name();
    meta = pmt::dict_add(meta, pmt::mp("capture_mode"),
                         pmt::string_to_symbol(mode));
    meta = pmt::dict_add(meta, pmt::mp("acquisition_epoch"),
                         pmt::from_uint64(tracker_.epoch()));
    meta = pmt::dict_add(meta, pmt::mp("schedule_generation"),
                         pmt::from_uint64(tracker_.generation()));
    meta = pmt::dict_add(meta, pmt::mp("schedule_index"),
                         pmt::from_uint64(wmeta.schedule_index));
    meta = pmt::dict_add(meta, pmt::mp("start_sample"),
                         pmt::from_long(wmeta.predicted_start_sample));
    meta = pmt::dict_add(meta, pmt::mp("predicted_start_sample"),
                         pmt::from_long(wmeta.predicted_start_sample));
    meta = pmt::dict_add(meta, pmt::mp("window_start_sample"),
                         pmt::from_long(wmeta.window_start_sample));
    meta = pmt::dict_add(meta, pmt::mp("sample_rate"),
                         pmt::from_double(d_sample_rate_));
    meta = pmt::dict_add(meta, pmt::mp("sample_format"), pmt::mp("sc16"));
    meta = pmt::dict_add(meta, pmt::mp("sample_count"),
                         pmt::from_long(static_cast<long>(n)));
    meta = pmt::dict_add(meta, pmt::mp("packet_interval_s"),
                         pmt::from_double(sched_.config().packet_interval_s));
    meta = pmt::dict_add(meta, pmt::mp("lock_state"),
                         pmt::string_to_symbol(core::qm35_lock_state_name(
                             tracker_.state())));
    meta = pmt::dict_add(meta, pmt::mp("partial"), pmt::from_bool(wmeta.partial));
    if (d_have_rx_time_) {
        const double t =
            d_rx_time_s_ +
            static_cast<double>(wmeta.window_start_sample -
                                static_cast<int64_t>(d_rx_time_abs_)) /
                d_sample_rate_;
        meta = pmt::dict_add(meta, pmt::mp("rx_time_s"), pmt::from_double(t));
    }

    const int16_t* iq =
        reinterpret_cast<const int16_t*>(slot.samples_sc16.data());
    pmt::pmt_t data = pmt::init_s16vector(n * 2, iq);
    message_port_pub(pmt::mp("packet"), pmt::cons(meta, data));
    d_emitted_.fetch_add(1, std::memory_order_relaxed);
    sched_.note_emitted();
}

} // namespace uwb
} // namespace gr
