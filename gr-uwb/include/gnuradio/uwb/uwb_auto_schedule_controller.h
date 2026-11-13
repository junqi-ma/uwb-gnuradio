/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Pure C++ packet auto-acquisition, known-period OLS/PLL refinement, and
 * interference-aware asymmetric guard computation for UhdSc16Pipeline.
 *
 * Reuses the GNU Radio-independent core building blocks:
 *   - UwbDetectorStateMachineSc16 (sliding SC16 energy gate)
 *   - UwbPreambleVerifierSc16     (Q15 coarse + full-rate fine + backtrack)
 *   - Qm35AcquisitionTracker      (state machine + native<->work mapping)
 *   - ScheduleLockTracker         (OLS (t0, T) fine-tuning + 1st-order PLL)
 */

#pragma once

#include <gnuradio/uwb/uwb_defaults.h>
#include <gnuradio/uwb/uwb_detector_core.h>
#include <gnuradio/uwb/uwb_hrp_mod_core.h>
#include <gnuradio/uwb/uwb_preamble_verifier_sc16.h>
#include <gnuradio/uwb/uwb_qm35_acquisition_tracker.h>
#include <gnuradio/uwb/uwb_scheduled_extractor_core.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <complex>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <deque>
#include <limits>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace gr {
namespace uwb {
namespace pipeline {

enum class ScheduleMode {
    Fixed = 0,      ///< Default: static (schedule_t0, period_s) fixed slicing
    AutoAcquire = 1 ///< Known-period energy acquire -> OLS/PLL narrow schedule
};

enum class WindowKind : uint8_t {
    Fixed = 0,
    Acquisition = 1,
    Scheduled = 2
};

struct AutoScheduleConfig {
    // Nominal known repetition interval (s). <= 0 uses PipelineConfig::period_s.
    double period_s = 0.0;
    double max_period_rel_err = 0.02; // +/- 2% for OLS fine-tuning around known T

    // Energy / Segmented-Autocorrelation detector & verifier (native SC16 stream)
    // When `use_autocorr_gate` is true (default), segmented delay-SYNC
    // autocorrelation triggers Acquisition only on periodic UWB preambles
    // (scale-invariant, CFO-immune, ignores broadband noise and PSDU).
    bool use_autocorr_gate = true;
    float autocorr_threshold = 0.35f;
    float autocorr_min_energy = 1e-4f;
    size_t autocorr_segment_len = 250;
    size_t autocorr_window_segments = 8;
    size_t autocorr_holdoff_segments = 8;
    // Legacy/shaped sparse Ipatov UWB at 0.5-0.8 FS has ~0.008-0.018 mean |x|^2
    // (above X410 ~1.2e-3 CW floor, below dense-waveform 0.02f default).
    float energy_threshold = 0.004f;
    size_t detector_pre_trigger = 0; // 0 => auto (>= pre_guard + 4096)
    size_t detector_gate_decimation = 100;
    size_t detector_gate_window = 32;
    size_t detector_holdoff_decimated = 8;
    // Keep stride=1 so single-symbol coarse verification (R=1) never skips
    // narrow HRP pulses between decimated grid points.
    size_t verifier_coarse_stride = 1;
    float verifier_fine_threshold = defaults::kDetectorFineThreshold;
    float verifier_backtrack_threshold = defaults::kDetectorBacktrackThreshold;
    size_t verifier_max_backtrack = 16;

    // Schedule lock & maintenance state machine
    double provisional_guard_us = 150.0;
    size_t lock_observations = 10;
    double converge_residual_samples = 128.0;
    double unlock_residual_samples = 512.0;
    size_t holdover_miss_count = 3;
    size_t reacquire_miss_count = 8;
    size_t neighborhood_slots = 8;

    // Dynamic Target Seeded search margins in work samples (998.4 MS/s)
    size_t locked_search_margin_work = 256;
    size_t provisional_search_margin_work = 1024;
    size_t acquisition_search_margin_work = 2048;

    // Acquisition slot shaping (0 => inherit effective scheduled guards/cap)
    size_t acq_pre_guard_samples = 0;
    size_t acq_post_guard_samples = 0;

    // Target identity gating
    bool require_identity_match = true;
    bool require_fcs_pass = true; // Enforced for Comm target; ignored for Sensing

    // Interference-aware asymmetric guard helper (optional auto-compute)
    bool auto_interference_guards = false;
    size_t early_int_pre_sfd_sync_reps = 16; // >= fit_reps SYNC reps before interferer SFD
    size_t late_int_margin_work = 512;
    size_t early_int_margin_work = 512;
    size_t target_max_psdu_bytes = 127; // Used when computing Mode B Comm capture length

    // Offline replay synchronization: wait briefly on CandidateVerify feedback
    // after emitting an Acquisition slot during unpaced file replay so packet 1+
    // lock immediately instead of being skipped by fast SSD reads.
    bool sync_candidate_on_unpaced_replay = true;
    uint32_t candidate_wait_timeout_ms = 200;
};

struct InterferenceGuardSpec {
    double native_rate = 998.4e6;
    double work_rate = 998.4e6;

    bool target_is_comm = false;
    size_t target_sync_reps = 64;
    const char* target_sfd_mode = "4z2";
    size_t target_max_psdu_bytes = 127;
    mod::PulseSpec target_pulse{};

    size_t int_sync_reps = 64;
    const char* int_sfd_mode = "4z2";
    size_t int_pre_sfd_sync_reps = 16; // SYNC symbols before interferer's SFD
    size_t early_margin_work = 512;
    size_t late_margin_work = 512;
    size_t resampler_native_halo = 256;
};

struct AutoWindowGeometry {
    size_t pre_guard_native = 0;
    size_t capture_native = 0;
    size_t post_guard_native = 0;

    size_t pre_guard_work = 0;
    size_t capture_work = 0;
    size_t post_guard_work = 0;
};

inline size_t sfd_symbol_count_for_mode(const char* sfd_mode)
{
    const int8_t* ptr = nullptr;
    size_t n = 0;
    if (sfd_mode && mod::detail::sfd_lookup(sfd_mode, ptr, n) && n > 0) {
        return n;
    }
    return 8;
}

/**
 * Compute interference-aware asymmetric capture geometry on native grid:
 *   - pre_guard: large enough that an earlier-arriving interferer keeps its SFD
 *                plus `int_pre_sfd_sync_reps` SYNC preamble symbols before SFD
 *                at the start of the window (+ halo).
 *   - capture:   full target body (SYNC+SFD for Sensing; full max-PSDU frame
 *                for Comm).
 *   - post_guard: extends past target body by at least the interferer's full
 *                 SYNC + SFD span (+ halo), ensuring late interferers arriving
 *                 anywhere inside the target still expose their SFD for
 *                 reliable detection.
 */
inline AutoWindowGeometry compute_interference_aware_window(
    const InterferenceGuardSpec& spec)
{
    constexpr size_t kSym = demod::kQm35SamplesPerSymbol; // 1016 @ 998.4M
    const size_t int_sfd_syms = sfd_symbol_count_for_mode(spec.int_sfd_mode);
    const size_t tgt_sfd_syms = sfd_symbol_count_for_mode(spec.target_sfd_mode);

    AutoWindowGeometry g;
    const size_t pre_reps = std::max<size_t>(1, spec.int_pre_sfd_sync_reps);
    g.pre_guard_work =
        (pre_reps + int_sfd_syms) * kSym + spec.early_margin_work;

    if (spec.target_is_comm) {
        g.capture_work = mod::packet_samples_998p4(
            spec.target_sync_reps, tgt_sfd_syms, spec.target_max_psdu_bytes,
            /*insert_sts=*/false, &spec.target_pulse);
    } else {
        g.capture_work =
            (spec.target_sync_reps + tgt_sfd_syms) * kSym +
            mod::pulse_tail_extra(spec.target_pulse);
    }

    const size_t late_reps = std::max<size_t>(1, spec.int_sync_reps);
    g.post_guard_work =
        (late_reps + int_sfd_syms) * kSym + spec.late_margin_work;

    const double ratio =
        (spec.work_rate > 0.0 && spec.native_rate > 0.0)
            ? (spec.native_rate / spec.work_rate)
            : 1.0;
    const bool resampled = std::abs(ratio - 1.0) > 1e-6;
    const size_t halo = resampled ? spec.resampler_native_halo : 0;

    auto to_native = [&](size_t work_len) -> size_t {
        return static_cast<size_t>(
                   std::ceil(static_cast<double>(work_len) * ratio)) +
               halo;
    };

    g.pre_guard_native = to_native(g.pre_guard_work);
    g.capture_native = to_native(g.capture_work);
    g.post_guard_native = to_native(g.post_guard_work);
    return g;
}

/**
 * Synthesize an L2-normalized 1-symbol SYNC template at an arbitrary host/native
 * rate >= 250 MS/s in pure C++. Uses windowed-sinc fractional resampling of a
 * 3-symbol periodic modulated preamble when `native_rate != 998.4e6`, so
 * 737.28 MS/s and 491.52 MS/s work cleanly for any code index and PulseSpec
 * without needing GNU Radio filters or external tap files.
 */
inline std::vector<std::complex<float>> synthesize_native_sync_template(
    size_t code_index,
    const mod::PulseSpec& pulse,
    const char* sfd_mode,
    double native_rate,
    double work_rate = defaults::kSampleRateHz)
{
    constexpr size_t kWorkSym = demod::kQm35SamplesPerSymbol; // 1016
    const double eff_work = (work_rate > 0.0) ? work_rate : defaults::kSampleRateHz;
    const double eff_native = (native_rate > 0.0) ? native_rate : eff_work;

    mod::HrpModConfig mod_cfg;
    mod_cfg.code_index = code_index;
    mod_cfg.sync_repetitions = 3; // 3 periodic copies avoid boundary edges
    mod_cfg.sfd_mode = (sfd_mode && sfd_mode[0] != '\0') ? sfd_mode : "4z2";
    mod_cfg.insert_sts = false;
    mod_cfg.pulse = pulse;

    mod::HrpModScratch scratch;
    std::vector<std::complex<float>> prefix_wf(65536);
    size_t produced = 0;
    if (!mod::modulate_prefix_waveform(mod_cfg, scratch, prefix_wf.data(),
                                       prefix_wf.size(), produced) ||
        produced < 3 * kWorkSym) {
        throw std::runtime_error(
            "synthesize_native_sync_template: failed for code_index=" +
            std::to_string(code_index));
    }

    if (std::abs(eff_native - eff_work) <= 1.0) {
        std::vector<std::complex<float>> out(
            prefix_wf.begin() + static_cast<std::ptrdiff_t>(kWorkSym),
            prefix_wf.begin() + static_cast<std::ptrdiff_t>(2 * kWorkSym));
        core::uwb_l2_normalize(out);
        return out;
    }

    const size_t native_sym = static_cast<size_t>(
        std::llround(static_cast<double>(kWorkSym) * eff_native / eff_work));
    std::vector<std::complex<float>> out(native_sym);

    const double step = eff_work / eff_native; // > 1 when downsampling 998.4 -> 737.28
    const double bw = std::min(1.0, eff_native / eff_work);
    const int half_taps = 24;
    constexpr double kPi = 3.14159265358979323846;

    for (size_t i = 0; i < native_sym; ++i) {
        const double center = static_cast<double>(kWorkSym) + static_cast<double>(i) * step;
        const int n0 = static_cast<int>(std::floor(center));
        std::complex<float> acc(0.0f, 0.0f);
        double wsum = 0.0;
        for (int m = n0 - half_taps; m <= n0 + half_taps; ++m) {
            if (m < 0 || static_cast<size_t>(m) >= 3 * kWorkSym)
                continue;
            const double dx = center - static_cast<double>(m);
            const double s_arg = bw * dx;
            const double sinc_v =
                (std::abs(s_arg) < 1e-12)
                    ? 1.0
                    : std::sin(kPi * s_arg) / (kPi * s_arg);
            const double u =
                (dx + static_cast<double>(half_taps)) / (2.0 * half_taps);
            const double win = 0.42 - 0.50 * std::cos(2.0 * kPi * u) +
                               0.08 * std::cos(4.0 * kPi * u);
            const double w = sinc_v * win;
            acc += static_cast<float>(w) * prefix_wf[static_cast<size_t>(m)];
            wsum += w;
        }
        out[i] = (wsum > 0.0) ? (acc / static_cast<float>(wsum)) : acc;
    }
    core::uwb_l2_normalize(out);
    return out;
}

/**
 * Feedback item pushed by DSP consumer workers back to the RX producer thread.
 * Mirrors Qm35LockObservation without any PMT dependency.
 */
struct LockFeedback {
    uint64_t window_id = 0;
    uint64_t acquisition_epoch = 0;
    uint64_t schedule_generation = 0;
    uint64_t schedule_index = std::numeric_limits<uint64_t>::max();
    WindowKind window_kind = WindowKind::Acquisition;

    // Work-sample detection relative to the start of `slot->sc16_samples`
    // mapped via native_start_abs + work->native inverse.
    int64_t slot_native_start_abs = 0;
    int64_t detected_start_work_rel = -1;
    double work_rate = defaults::kSampleRateHz;
    double native_rate = defaults::kSampleRateHz;
    double resample_filter_delay = 0.0;

    bool timing_ok = false;
    bool identity_ok = false;
    bool fcs_pass = false;
    int code_index = -1;
    int preamble_repetitions = -1;
    std::string sfd_mode;
};

class BoundedFeedbackQueue {
public:
    explicit BoundedFeedbackQueue(size_t capacity = 256)
        : capacity_(capacity > 0 ? capacity : 256) {}

    void push(const LockFeedback& fb)
    {
        {
            std::lock_guard<std::mutex> lk(mu_);
            if (q_.size() >= capacity_) {
                q_.pop_front();
                ++dropped_;
            }
            q_.push_back(fb);
        }
        cv_.notify_one();
    }

    size_t drain(std::vector<LockFeedback>& out)
    {
        out.clear();
        std::lock_guard<std::mutex> lk(mu_);
        while (!q_.empty()) {
            out.push_back(std::move(q_.front()));
            q_.pop_front();
        }
        return out.size();
    }

    bool wait_and_drain(std::vector<LockFeedback>& out, uint32_t timeout_ms)
    {
        std::unique_lock<std::mutex> lk(mu_);
        if (q_.empty() && timeout_ms > 0) {
            cv_.wait_for(lk, std::chrono::milliseconds(timeout_ms),
                         [this] { return !q_.empty() || stopped_; });
        }
        out.clear();
        while (!q_.empty()) {
            out.push_back(std::move(q_.front()));
            q_.pop_front();
        }
        return !out.empty();
    }

    void wake_all()
    {
        {
            std::lock_guard<std::mutex> lk(mu_);
            stopped_ = true;
        }
        cv_.notify_all();
    }

    void reset()
    {
        std::lock_guard<std::mutex> lk(mu_);
        q_.clear();
        stopped_ = false;
        dropped_ = 0;
    }

    uint64_t dropped() const
    {
        std::lock_guard<std::mutex> lk(mu_);
        return dropped_;
    }

private:
    const size_t capacity_;
    mutable std::mutex mu_;
    std::condition_variable cv_;
    std::deque<LockFeedback> q_;
    bool stopped_ = false;
    uint64_t dropped_ = 0;
};

/**
 * Pure C++ RX-side auto-acquisition & predictive-schedule state machine.
 * Single-writer ownership inside RX thread; communicates with DSP workers via
 * slot metadata (RX -> Worker) and BoundedFeedbackQueue (Worker -> RX).
 */
class AutoScheduleController {
public:
    struct TargetProfile {
        int code_index = 9;
        size_t sync_repetitions = 64;
        size_t sfd_symbols = 8;
        std::string sfd_mode = "4z2";
        mod::PulseSpec pulse{};
        bool is_comm = false;
        std::vector<std::complex<float>> native_sync_template_override;
    };

    struct EmittedAcqSlot {
        int64_t win_start_abs = 0;
        int64_t predicted_start_abs = 0;
        size_t sample_count = 0;
        const std::complex<int16_t>* samples = nullptr;
        uint64_t acquisition_epoch = 0;
        uint64_t schedule_generation = 0;
        float start_metric = 0.0f;
        float detection_metric = 0.0f;
    };

    struct ScheduledSlotPlan {
        uint64_t schedule_index = 0;
        int64_t predicted_start_abs = 0;
        int64_t win_start_abs = 0;
        int64_t win_end_abs = 0;
        size_t pre_guard = 0;
        size_t capture = 0;
        size_t post_guard = 0;
        size_t search_margin_work = 256;
        uint64_t acquisition_epoch = 0;
        uint64_t schedule_generation = 0;
        bool is_provisional = true;
    };

    void configure(const AutoScheduleConfig& acfg,
                   const TargetProfile& target,
                   double native_rate,
                   double work_rate,
                   size_t base_pre_guard,
                   size_t base_capture,
                   size_t base_post_guard)
    {
        cfg_ = acfg;
        if (cfg_.period_s <= 0.0) {
            cfg_.period_s = defaults::kQm35PacketIntervalS;
        }
        target_ = target;
        native_rate_ = native_rate > 0.0 ? native_rate : defaults::kSampleRateHz;
        work_rate_ = work_rate > 0.0 ? work_rate : defaults::kSampleRateHz;
        base_pre_ = base_pre_guard;
        base_cap_ = base_capture;
        base_post_ = base_post_guard;

        acq_pre_ = (cfg_.acq_pre_guard_samples > 0)
                       ? cfg_.acq_pre_guard_samples
                       : std::max<size_t>(base_pre_, 256);
        acq_post_ = (cfg_.acq_post_guard_samples > 0)
                        ? cfg_.acq_post_guard_samples
                        : base_post_;
        acq_cap_ = base_cap_ + acq_post_;

        prov_guard_samples_ = static_cast<size_t>(
            std::llround(std::max(0.0, cfg_.provisional_guard_us) * 1e-6 *
                         native_rate_));

        const double sym_exact =
            static_cast<double>(demod::kQm35SamplesPerSymbol) *
            native_rate_ / work_rate_;
        const size_t lag0 = static_cast<size_t>(
            std::max<int64_t>(1, static_cast<int64_t>(std::llround(sym_exact))));
        size_t lag1 = 0;
        if (std::abs(sym_exact - static_cast<double>(lag0)) > 0.1) {
            const int64_t alt =
                (sym_exact > static_cast<double>(lag0))
                    ? (static_cast<int64_t>(lag0) + 1)
                    : (static_cast<int64_t>(lag0) - 1);
            if (alt > 0) {
                lag1 = static_cast<size_t>(alt);
            }
        }
        const size_t ac_lag_max = std::max(lag0, lag1);
        const size_t ac_gate_lag =
            cfg_.use_autocorr_gate
                ? (cfg_.autocorr_segment_len * cfg_.autocorr_window_segments +
                   ac_lag_max)
                : 0;
        const size_t gate_lag = std::max(
            cfg_.detector_gate_decimation * cfg_.detector_gate_window,
            ac_gate_lag);
        const size_t sym_margin = static_cast<size_t>(
            std::ceil(4.0 * demod::kQm35SamplesPerSymbol * native_rate_ /
                      work_rate_));
        const size_t min_pre_trig = acq_pre_ + std::max(gate_lag, sym_margin);
        const size_t pre_trigger = std::max<size_t>(
            defaults::kDetectorPreTrigger,
            std::max(cfg_.detector_pre_trigger, min_pre_trig));
        const size_t post_capture = acq_cap_ + static_cast<size_t>(
            std::ceil(2.0 * demod::kQm35SamplesPerSymbol * native_rate_ /
                      work_rate_));

        sm_.emplace(pre_trigger, cfg_.energy_threshold,
                    cfg_.detector_gate_decimation,
                    cfg_.detector_gate_window,
                    cfg_.detector_holdoff_decimated,
                    post_capture);
        if (cfg_.use_autocorr_gate) {
            AutocorrGateConfig ag;
            ag.lag0 = lag0;
            ag.lag1 = lag1;
            ag.segment_len = cfg_.autocorr_segment_len;
            ag.window_segments = cfg_.autocorr_window_segments;
            ag.corr_threshold = cfg_.autocorr_threshold;
            ag.min_mean_power = cfg_.autocorr_min_energy;
            sm_->configure_autocorr_gate(ag, cfg_.autocorr_holdoff_segments);
        }

        std::vector<std::complex<float>> tmpl =
            target_.native_sync_template_override;
        if (tmpl.empty()) {
            tmpl = synthesize_native_sync_template(
                static_cast<size_t>(target_.code_index), target_.pulse,
                target_.sfd_mode.c_str(), native_rate_, work_rate_);
        }
        core::UwbPreambleVerifierSc16::Config vcfg;
        vcfg.coarse_stride =
            cfg_.verifier_coarse_stride > 0 ? cfg_.verifier_coarse_stride : 1;
        vcfg.fine_threshold = cfg_.verifier_fine_threshold;
        vcfg.backtrack_threshold = cfg_.verifier_backtrack_threshold;
        vcfg.max_backtrack = cfg_.verifier_max_backtrack;
        vcfg.sync_repetitions = target_.sync_repetitions;
        vcfg.sfd_symbols = (target_.sfd_symbols > 0)
                               ? target_.sfd_symbols
                               : sfd_symbol_count_for_mode(target_.sfd_mode.c_str());
        verifier_.configure(tmpl, vcfg);
        verifier_.reserve_coarse(pre_trigger + post_capture + 65536);

        core::Qm35AcquisitionConfig qcfg;
        qcfg.sample_rate = native_rate_;
        qcfg.nominal_interval_s = cfg_.period_s;
        qcfg.max_period_rel_err = cfg_.max_period_rel_err;
        qcfg.lock_observations = std::max<size_t>(2, cfg_.lock_observations);
        qcfg.holdover_miss_count = cfg_.holdover_miss_count;
        qcfg.reacquire_miss_count = cfg_.reacquire_miss_count;
        qcfg.neighborhood_slots = cfg_.neighborhood_slots;
        qcfg.identity.code_index = target_.code_index;
        qcfg.identity.preamble_repetitions =
            static_cast<int>(target_.sync_repetitions);
        qcfg.identity.sfd_mode = target_.sfd_mode;
        qcfg.identity.require_status_success = cfg_.require_identity_match;
        qcfg.identity.require_fcs_pass =
            target_.is_comm && cfg_.require_fcs_pass;
        tracker_.configure(qcfg);

        next_k_ = 0;
        have_rx_time_ = false;
        rx_time_abs_ = 0;
        rx_time_s_ = 0.0;
        scan_cursor_abs_ = 0;
        locked_at_slot_ = std::numeric_limits<uint64_t>::max();
        acq_windows_ = 0;
        scheduled_windows_ = 0;
        discontinuities_ = 0;
        held_regions_.clear();
    }

    size_t max_slot_native_capacity() const
    {
        const size_t sched_max =
            base_pre_ + base_cap_ + base_post_ + 2 * prov_guard_samples_;
        const size_t acq_max = acq_pre_ + acq_cap_;
        return std::max(sched_max, acq_max);
    }

    bool scheduled_path_active() const
    {
        return tracker_.scheduled_path_active();
    }

    bool waiting_candidate_verify() const
    {
        return tracker_.state() == core::Qm35LockState::CandidateVerify;
    }

    core::Qm35LockState state() const { return tracker_.state(); }
    uint64_t epoch() const { return tracker_.epoch(); }
    uint64_t generation() const { return tracker_.generation(); }
    double t0_exact() const { return tracker_.t0_exact(); }
    double period_s() const { return tracker_.period_s(); }
    double period_samples() const { return tracker_.period_s() * native_rate_; }
    uint64_t acq_windows() const { return acq_windows_; }
    uint64_t scheduled_windows() const { return scheduled_windows_; }
    uint64_t discontinuities() const { return discontinuities_; }
    uint64_t stale_feedback() const { return tracker_.stale_feedback(); }
    int64_t scan_cursor_abs() const { return scan_cursor_abs_; }
    uint64_t locked_at_window_id() const { return locked_at_slot_; }

    /**
     * Check hardware timestamp continuity / overflow gaps. If a discontinuity
     * occurred, rebases `scan_cursor_abs_` onto corrected monotonic sample
     * coordinates, bumps tracker generation, and returns true.
     */
    bool handle_stream_meta(int64_t pre_read_abs,
                            size_t n_read,
                            bool has_time_spec,
                            double time_spec_s,
                            bool is_gap,
                            int64_t gap_samples,
                            int64_t& io_read_end_abs)
    {
        bool jump = is_gap;
        int64_t extra_skipped = (is_gap && gap_samples > 0) ? gap_samples : 0;
        if (has_time_spec) {
            if (have_rx_time_) {
                const double dt_hw = time_spec_s - rx_time_s_;
                const double dt_expected =
                    static_cast<double>(pre_read_abs - rx_time_abs_) /
                    native_rate_;
                const double tol = std::max(1e-6, 0.5 * cfg_.period_s);
                if (std::abs(dt_hw - dt_expected) > tol) {
                    jump = true;
                    const int64_t expected_total = static_cast<int64_t>(
                        std::llround(dt_hw * native_rate_));
                    const int64_t counted = pre_read_abs - rx_time_abs_;
                    if (expected_total > counted) {
                        extra_skipped = std::max(extra_skipped, expected_total - counted);
                    }
                }
            }
            have_rx_time_ = true;
            rx_time_s_ = time_spec_s;
            rx_time_abs_ = pre_read_abs + extra_skipped;
        }
        if (jump) {
            ++discontinuities_;
            if (extra_skipped > 0) {
                io_read_end_abs += extra_skipped;
            }
            note_discontinuity_internal(io_read_end_abs);
            return true;
        }
        (void)n_read;
        return false;
    }

    void feed_acquisition(const std::complex<int16_t>* samples,
                          size_t count,
                          int64_t start_abs)
    {
        if (!sm_ || !samples || count == 0 || start_abs < 0)
            return;
        sm_->process(samples, count, static_cast<uint64_t>(start_abs));
        scan_cursor_abs_ = start_abs + static_cast<int64_t>(count);
    }

    void flush_acquisition()
    {
        if (sm_)
            sm_->flush_region();
    }

    bool poll_verified_candidate(EmittedAcqSlot& out)
    {
        if (!sm_)
            return false;
        while (sm_->region_ready()) {
            const auto handle = sm_->take_region();
            const auto& region = sm_->region(handle);
            const size_t n = region.samples.size();
            if (n == 0) {
                sm_->release_region(handle);
                continue;
            }
            const auto vr = verifier_.verify(
                region.samples.data(), n, region.candidate_offset);
            if (!vr.confirmed) {
                sm_->release_region(handle);
                continue;
            }

            const int64_t region_start_abs =
                static_cast<int64_t>(region.start_abs);
            const int64_t packet_start =
                region_start_abs + static_cast<int64_t>(vr.start_offset);
            const int64_t lo = std::max(
                region_start_abs,
                packet_start - static_cast<int64_t>(acq_pre_));
            const size_t lo_off = static_cast<size_t>(lo - region_start_abs);
            const size_t max_cap = max_slot_native_capacity();
            const size_t desired =
                static_cast<size_t>(packet_start - lo) + acq_cap_;
            const size_t cap = std::min({desired, max_cap, n - lo_off});
            if (cap == 0) {
                sm_->release_region(handle);
                continue;
            }

            tracker_.note_candidate_emitted(packet_start);
            ++acq_windows_;

            out.win_start_abs = lo;
            out.predicted_start_abs = packet_start;
            out.sample_count = cap;
            out.samples = region.samples.data() + lo_off;
            out.acquisition_epoch = tracker_.epoch();
            out.schedule_generation = tracker_.generation();
            out.start_metric = vr.start_metric;
            out.detection_metric = vr.detection_metric;

            // Keep only the currently emitted handle alive until caller copies
            // `out.samples`, releasing any previously held handle.
            release_held_regions();
            held_regions_.push_back(handle);
            return true;
        }
        return false;
    }

    void release_held_regions()
    {
        if (!sm_)
            return;
        for (auto h : held_regions_) {
            sm_->release_region(h);
        }
        held_regions_.clear();
    }

    bool next_scheduled_slot(int64_t current_cursor_abs, ScheduledSlotPlan& out)
    {
        if (!tracker_.scheduled_path_active())
            return false;

        const bool provisional =
            (tracker_.state() == core::Qm35LockState::ProvisionalTrack ||
             tracker_.state() == core::Qm35LockState::Holdover ||
             (tracker_.state() == core::Qm35LockState::Reacquire &&
              !tracker_.global_energy()));
        const size_t extra = provisional ? prov_guard_samples_ : 0;
        const size_t eff_pre = base_pre_ + extra;
        const size_t eff_cap = base_cap_;
        const size_t eff_post = base_post_ + extra;

        const uint64_t min_k =
            tracker_.next_future_k(static_cast<uint64_t>(std::max<int64_t>(0, current_cursor_abs)),
                                   eff_pre);
        if (next_k_ < min_k) {
            next_k_ = min_k;
        }

        const double T_samples = tracker_.period_s() * native_rate_;
        const core::WindowBounds wb = core::window_bounds_for_slot(
            tracker_.t0_exact(), T_samples, next_k_, eff_pre, eff_cap, eff_post);

        out.schedule_index = next_k_;
        out.predicted_start_abs = wb.predicted_start;
        out.win_start_abs = wb.window_start;
        out.win_end_abs = wb.window_end;
        out.pre_guard = eff_pre;
        out.capture = eff_cap;
        out.post_guard = eff_post;
        out.search_margin_work = provisional
                                     ? cfg_.provisional_search_margin_work
                                     : cfg_.locked_search_margin_work;
        out.acquisition_epoch = tracker_.epoch();
        out.schedule_generation = tracker_.generation();
        out.is_provisional = provisional;

        ++next_k_;
        ++scheduled_windows_;
        return true;
    }

    void ingest_feedback(const std::vector<LockFeedback>& items,
                         int64_t current_cursor_abs)
    {
        for (const auto& fb : items) {
            core::Qm35LockObservation obs;
            obs.packet_id = fb.window_id;
            obs.acquisition_epoch = fb.acquisition_epoch;
            obs.schedule_generation = fb.schedule_generation;
            obs.schedule_index = fb.schedule_index;
            obs.has_epoch = true;
            obs.has_generation = true;
            obs.sample_rate = fb.work_rate;
            obs.native_sample_rate = fb.native_rate;
            obs.resample_filter_delay = fb.resample_filter_delay;
            obs.has_sample_rate = (fb.work_rate > 0.0);
            obs.has_native_sample_rate = (fb.native_rate > 0.0);
            obs.has_filter_delay = true;
            obs.timing_ok = fb.timing_ok;
            obs.has_timing_ok = true;
            obs.fcs_pass = fb.fcs_pass;
            obs.status = fb.identity_ok ? "success" : "fail";
            obs.code_index = fb.code_index;
            obs.preamble_repetitions = fb.preamble_repetitions;
            obs.sfd_mode = fb.sfd_mode;

            int64_t native_rel = -1;
            bool mapped = false;
            int64_t abs_det = -1;
            if (fb.detected_start_work_rel >= 0) {
                obs.detected_start_sample = fb.detected_start_work_rel;
                if (core::classify_lock_obs_mapping(obs, native_rate_,
                                                    &native_rel, &mapped) &&
                    native_rel >= 0) {
                    abs_det = fb.slot_native_start_abs + native_rel;
                }
            } else if (!fb.timing_ok) {
                if (tracker_.identity_confirmed()) {
                    // Scheduled slot missed: supply non-negative sentinel so
                    // Qm35AcquisitionTracker verifies epoch/generation and
                    // invokes note_miss().
                    abs_det = 0;
                } else {
                    // Acquisition candidate missed without sample offset:
                    // reject identity after epoch/generation check so
                    // CandidateVerify returns to UnlockedAcquire.
                    obs.status = "fail";
                    obs.code_index = std::numeric_limits<int>::max();
                    abs_det = 0;
                }
            } else if (!tracker_.identity_confirmed() &&
                       (!fb.identity_ok ||
                        (target_.is_comm && cfg_.require_fcs_pass &&
                         !fb.fcs_pass))) {
                obs.status = "fail";
                obs.code_index = std::numeric_limits<int>::max();
                abs_det = 0;
            }

            const auto prev = tracker_.state();
            const auto action = tracker_.on_lock_obs(obs, abs_det);
            if (action == core::Qm35ObsAction::ConfirmedIdentity) {
                next_k_ = tracker_.next_future_k(
                    static_cast<uint64_t>(std::max<int64_t>(0, current_cursor_abs)),
                    base_pre_ + prov_guard_samples_);
                if (next_k_ == 0) {
                    next_k_ = 1;
                }
            } else if (action == core::Qm35ObsAction::TimingUpdate) {
                if (prev != core::Qm35LockState::Locked &&
                    tracker_.state() == core::Qm35LockState::Locked &&
                    locked_at_slot_ == std::numeric_limits<uint64_t>::max()) {
                    locked_at_slot_ = fb.window_id;
                }
            } else if (action == core::Qm35ObsAction::TimingMiss) {
                if (tracker_.state() == core::Qm35LockState::Reacquire &&
                    tracker_.global_energy()) {
                    reset_detector_for_reacquire(current_cursor_abs);
                }
            }
        }
    }

private:
    void reset_detector_for_reacquire(int64_t resume_abs)
    {
        release_held_regions();
        if (sm_)
            sm_->reset();
        scan_cursor_abs_ = resume_abs;
        next_k_ = 0;
    }

    void note_discontinuity_internal(int64_t resume_abs)
    {
        tracker_.note_discontinuity();
        reset_detector_for_reacquire(resume_abs);
    }

    AutoScheduleConfig cfg_{};
    TargetProfile target_{};
    double native_rate_ = defaults::kSampleRateHz;
    double work_rate_ = defaults::kSampleRateHz;
    size_t base_pre_ = 400;
    size_t base_cap_ = 3200;
    size_t base_post_ = 400;
    size_t acq_pre_ = 400;
    size_t acq_post_ = 400;
    size_t acq_cap_ = 3600;
    size_t prov_guard_samples_ = 0;

    std::optional<UwbDetectorStateMachineSc16> sm_;
    core::UwbPreambleVerifierSc16 verifier_;
    core::Qm35AcquisitionTracker tracker_;
    std::vector<UwbDetectorStateMachineSc16::RegionHandle> held_regions_;

    uint64_t next_k_ = 0;
    bool have_rx_time_ = false;
    int64_t rx_time_abs_ = 0;
    double rx_time_s_ = 0.0;
    int64_t scan_cursor_abs_ = 0;
    uint64_t locked_at_slot_ = std::numeric_limits<uint64_t>::max();
    uint64_t acq_windows_ = 0;
    uint64_t scheduled_windows_ = 0;
    uint64_t discontinuities_ = 0;
};

} // namespace pipeline
} // namespace uwb
} // namespace gr
