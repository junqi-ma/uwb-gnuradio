// SPDX-License-Identifier: GPL-3.0-or-later
#include <gnuradio/uwb/uwb_demod_core.h>
#include <gnuradio/uwb/uwb_softchip_fir_diag.h>
#include <gnuradio/uwb/uwb_demod_execution.h>
#include <gnuradio/uwb/uwb_demod_finalize.h>
namespace gr::uwb::demod::core {
namespace {

struct StageClock {
    std::chrono::steady_clock::time_point t_prev = std::chrono::steady_clock::now();

    void lap(uint64_t& dst, uint64_t& total_us) {
        const auto now = std::chrono::steady_clock::now();
        dst = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::microseconds>(now - t_prev)
                .count());
        total_us += dst;
        t_prev = now;
    }

    void add_elapsed(uint64_t& dst, uint64_t& total_us) {
        const auto now = std::chrono::steady_clock::now();
        const uint64_t dt = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::microseconds>(now - t_prev)
                .count());
        dst += dt;
        total_us += dt;
        t_prev = now;
    }
};

struct RotationStepResult {
    uint64_t rotate_us = 0;
    uint32_t kernel = 0;
};

struct SyncBootstrapState {
    size_t sfd_len = 0;
    CfoResult stage2_cfo;
    DerotatedSpan predicted_span;
    DerotatedSpan derotated_span;
};

struct SfdAnchorConfig {
    size_t full_repetitions = 0;
    size_t tail_repetitions = 0;
    size_t tail_first = 0;
    int64_t bootstrap_sfd_start = 0;
    double period_hint = 0.0;
    int64_t tail_start = 0;
    size_t template_len = 0;
    Qm35825Profile cfo_anchor_profile;
};

RotationStepResult rotate_window_range(
    const std::complex<float>* rx,
    size_t n,
    const Qm35825Profile& profile,
    double cfo_hz,
    const SampleRange& range,
    uint32_t kernel_flag,
    DemodScratch& scratch,
    DemodResult& res)
{
    RotationStepResult step;
    derotate_cfo_range(rx, n, profile, cfo_hz, range.lo, range.hi,
                       scratch, step.rotate_us, step.kernel);
    res.stage_cfo_rotate_us += step.rotate_us;
    res.stage_cfo_rotation_kernels |= (step.kernel ? kernel_flag : 0u);
    res.stage_cfo_rotated_samples += range.size();
    ++res.stage_cfo_rotation_passes;
    return step;
}

bool search_sfd_with_wide_rescue(
    const std::complex<float>* rx,
    size_t n,
    const Qm35825Profile& profile,
    const std::vector<int8_t>& sfd_seq,
    const std::vector<std::complex<float>>& template_wf,
    size_t sfd_len,
    DemodScratch& scratch,
    DemodResult& res)
{
    return stage_sfd(
        scratch.derotated.data(), n, profile, res.timing, sfd_seq, template_wf,
        res.sfd, scratch, [&]() {
            const SampleRange wide_range = compute_sfd_window_range(
                n, res.timing, profile.preamble_repetitions, sfd_len, true);
            (void)rotate_window_range(
                rx, n, profile, res.cfo.cfo_hz, wide_range, 2u, scratch, res);
        });
}

bool run_initial_cfo_and_bootstrap_sfd(
    const std::complex<float>* rx,
    size_t n,
    const Qm35825Profile& profile,
    const std::vector<std::complex<float>>& template_wf,
    StageClock& clock,
    DemodScratch& scratch,
    DemodResult& res,
    SyncBootstrapState& sync_state)
{
    // Stage 2: CFO fit + bootstrap SFD search window rotation (Task A1)
    if (!fit_cfo(rx, n, profile, res.timing, res.cfo, scratch)) {
        clock.lap(res.stage_cfo_us, res.stage_total_us);
        res.stage_cfo_fit_us += res.cfo.fit_us;
        res.status = DemodStatus::CfoFailed;
        return false;
    }
    scratch.cached_seq.update(profile.code_index, profile.sfd_mode);
    const std::vector<int8_t>& sfd_seq = scratch.cached_seq.sfd_seq;
    sync_state.sfd_len = sfd_seq.size() * kQm35SamplesPerSymbol;
    const SampleRange sfd1_range = compute_sfd_window_range(
        n, res.timing, profile.preamble_repetitions, sync_state.sfd_len);
    // Stage-2 CFO, kept so the single-pass rotation mode can restore it after
    // the SFD-anchored refit has been evaluated.
    sync_state.stage2_cfo = res.cfo;
    res.cfo_initial_hz = sync_state.stage2_cfo.cfo_hz;
    res.cfo_initial_peaks =
        static_cast<uint32_t>(sync_state.stage2_cfo.peaks_used);
    // Predicted single-pass rotation range.  `scratch.derotated` is the only
    // CFO-compensated buffer that the downstream stages read, so one rotation
    // with the Stage-2 estimate can replace the SFD-anchored second pass only
    // if it already covers everything those stages will read.  The SFD anchor
    // is not known yet, so the range is predicted from the Stage-1 timing with
    // an explicit symbol/backtrack slack and re-validated against the actual
    // [sec_range.lo, sec_range.hi) after the bootstrap SFD (a miss falls back
    // to the legacy second rotation).
    const SampleRange predicted_range = compute_predicted_single_pass_range(
        n, profile, res.timing, sfd1_range);
    sync_state.predicted_span = DerotatedSpan{
        predicted_range, sync_state.stage2_cfo.cfo_hz, 0,
        !predicted_range.empty()};
    const SampleRange rot1_range = sync_state.predicted_span.valid
        ? sync_state.predicted_span.range
        : sfd1_range;
    const RotationStepResult rot1 = rotate_window_range(
        rx, n, profile, res.cfo.cfo_hz, rot1_range, 1u, scratch, res);
    sync_state.derotated_span = DerotatedSpan{
        rot1_range, res.cfo.cfo_hz, 0, !rot1_range.empty()};
    res.cfo.rotate_us = rot1.rotate_us;
    res.cfo.rotation_kernel = rot1.kernel;
    clock.lap(res.stage_cfo_us, res.stage_total_us);
    res.stage_cfo_fit_us += res.cfo.fit_us;

    // Stage 3: bootstrap SFD timing.  Timing acquisition can start several
    // SYNCs late when a weak leading repetition falls below its tracking
    // threshold.  Its 64-point phase train would then spill into the SFD and
    // corrupt a CFO fit, even though the SFD search can recover the true frame
    // origin by testing earlier hypotheses.  Use this first SFD only to anchor
    // the preamble; the final reported CFO is re-fitted below from that anchor.
    scratch.initial_timing = res.timing;
    const auto sfd_bootstrap_t0 = std::chrono::steady_clock::now();
    const bool bootstrap_sfd_ok = search_sfd_with_wide_rescue(
        rx, n, profile, sfd_seq, template_wf, sync_state.sfd_len, scratch, res);
    res.stage_sfd_bootstrap_us = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - sfd_bootstrap_t0).count());
    res.sfd_bootstrap_windows = res.sfd.search_windows;
    res.sfd_bootstrap_coarse_correlations = res.sfd.coarse_correlations;
    res.sfd_bootstrap_fine_correlations = res.sfd.fine_correlations;
    res.sfd_initial_predicted_sample = res.sfd.expected_start_sample;
    res.sfd_bootstrap_detected_sample = res.sfd.sfd_start_sample;
    res.sfd_bootstrap_first_threshold_backtrack_symbols =
        res.sfd.first_threshold_backtrack_symbols;
    if (!bootstrap_sfd_ok) {
        clock.lap(res.stage_sfd_us, res.stage_total_us);
        res.status = DemodStatus::SfdFailed;
        return false;
    }
    return true;
}

bool try_reuse_initial_tail_timing(
    const TimingResult& initial_timing,
    const Qm35825Profile& profile,
    const Qm35825Profile& anchor,
    const SfdAnchorConfig& cfg,
    size_t seed_radius,
    TimingResult& tail_timing)
{
    const int64_t start1 =
        (!initial_timing.peak_samples.empty() && cfg.template_len > 0)
            ? initial_timing.peak_samples[0] -
                  static_cast<int64_t>(cfg.template_len - 1)
            : -1;
    const int64_t expected_tail_start = (start1 >= 0)
        ? start1 + static_cast<int64_t>(cfg.tail_first * kQm35SamplesPerSymbol)
        : -2;
    const int64_t actual_tail_first_start =
        (cfg.tail_first < initial_timing.peak_samples.size() &&
         cfg.template_len > 0)
            ? initial_timing.peak_samples[cfg.tail_first] -
                  static_cast<int64_t>(cfg.template_len - 1)
            : -3;

    if (seed_radius != 8 || !initial_timing.ok ||
        initial_timing.detected_peaks != cfg.full_repetitions ||
        initial_timing.peak_samples.size() != cfg.full_repetitions ||
        initial_timing.peak_metrics.size() != cfg.full_repetitions ||
        initial_timing.peak_corr.size() != cfg.full_repetitions ||
        cfg.template_len == 0 ||
        cfg.tail_repetitions == 0 ||
        cfg.tail_repetitions > cfg.full_repetitions ||
        cfg.tail_first == 0 ||
        profile.timing_track_radius != 8 ||
        anchor.timing_track_radius != 8 ||
        anchor.timing_max_backtrack_symbols != 0 ||
        cfg.tail_start != expected_tail_start ||
        actual_tail_first_start != cfg.tail_start) {
        return false;
    }

    for (size_t k = cfg.tail_first; k < cfg.full_repetitions; ++k) {
        const float m = initial_timing.peak_metrics[k];
        if (!std::isfinite(m) || m < anchor.radar_verification_threshold) {
            return false;
        }
    }

    tail_timing.peak_samples.assign(
        initial_timing.peak_samples.begin() + cfg.tail_first,
        initial_timing.peak_samples.end());
    tail_timing.peak_metrics.assign(
        initial_timing.peak_metrics.begin() + cfg.tail_first,
        initial_timing.peak_metrics.end());
    tail_timing.peak_corr.assign(
        initial_timing.peak_corr.begin() + cfg.tail_first,
        initial_timing.peak_corr.end());
    tail_timing.detected_peaks = cfg.tail_repetitions;
    tail_timing.expected_peaks = cfg.tail_repetitions;
    tail_timing.metric = tail_timing.peak_metrics[0];
    tail_timing.coarse_search_us = 0;
    tail_timing.fine_track_us = 0;
    tail_timing.seed_search_us = 0;
    tail_timing.seed_refine_narrow_attempts = 0;
    tail_timing.seed_refine_narrow_success = 0;
    tail_timing.seed_refine_fallbacks = 0;

    const size_t np = cfg.tail_repetitions;
    const int64_t period = static_cast<int64_t>(kQm35SamplesPerSymbol);
    if (np >= 2) {
        double sx = 0.0, sy = 0.0, sxx = 0.0, sxy = 0.0;
        for (size_t k = 0; k < np; ++k) {
            const double x = static_cast<double>(k);
            const double y = static_cast<double>(tail_timing.peak_samples[k]);
            sx += x;
            sy += y;
            sxx += x * x;
            sxy += x * y;
        }
        const double denom = static_cast<double>(np) * sxx - sx * sx;
        if (std::abs(denom) > 1e-12) {
            tail_timing.measured_period =
                (static_cast<double>(np) * sxy - sx * sy) / denom;
            const double intercept_end =
                (sy - tail_timing.measured_period * sx) / np;
            tail_timing.preamble_start_sample = std::max<int64_t>(
                0, static_cast<int64_t>(std::llround(intercept_end)) -
                       (static_cast<int64_t>(cfg.template_len) - 1));
        } else {
            tail_timing.measured_period = static_cast<double>(period);
            tail_timing.preamble_start_sample = cfg.tail_start;
        }
    } else {
        tail_timing.measured_period = static_cast<double>(period);
        tail_timing.preamble_start_sample = cfg.tail_start;
    }
    tail_timing.ok = true;
    return true;
}

bool retrack_tail_timing(
    const std::complex<float>* rx,
    size_t n,
    const Qm35825Profile& profile,
    const std::vector<std::complex<float>>& template_wf,
    const SfdAnchorConfig& cfg,
    size_t seed_radius,
    DemodScratch& scratch,
    DemodResult& res,
    TimingResult& tail_timing)
{
    Qm35825Profile anchor = cfg.cfo_anchor_profile;
    anchor.timing_seed_refine_radius = seed_radius;
    const auto sfd_tail_t0 = std::chrono::steady_clock::now();

    bool tail_timing_ok = try_reuse_initial_tail_timing(
        scratch.initial_timing, profile, anchor, cfg, seed_radius, tail_timing);
    if (!tail_timing_ok) {
        tail_timing_ok = stage_timing(
            rx, n, anchor, template_wf, cfg.tail_start, tail_timing, scratch);
        if ((!tail_timing_ok ||
             tail_timing.detected_peaks < 4) &&
            seed_radius != 0) {
            const auto retry_t0 = std::chrono::steady_clock::now();
            anchor.timing_seed_refine_radius = 0;
            TimingResult retry_timing;
            const bool retry_ok = stage_timing(
                rx, n, anchor, template_wf, cfg.tail_start, retry_timing, scratch);
            res.stage_sfd_tail_retrack_fallback_us += static_cast<uint64_t>(
                std::chrono::duration_cast<std::chrono::microseconds>(
                    std::chrono::steady_clock::now() - retry_t0).count());
            ++res.stage_tail_retrack_fallbacks;
            if (retry_ok &&
                retry_timing.detected_peaks > tail_timing.detected_peaks) {
                // Keep the narrow-attempt diagnostics visible even though the
                // legacy retry (radius 0) produced the accepted timing.
                retry_timing.seed_search_us += tail_timing.seed_search_us;
                retry_timing.seed_refine_narrow_attempts +=
                    tail_timing.seed_refine_narrow_attempts;
                retry_timing.seed_refine_narrow_success +=
                    tail_timing.seed_refine_narrow_success;
                retry_timing.seed_refine_fallbacks +=
                    tail_timing.seed_refine_fallbacks;
                tail_timing = std::move(retry_timing);
                tail_timing_ok = true;
            }
        }
    }
    res.stage_sfd_tail_retrack_us += static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - sfd_tail_t0).count());
    return tail_timing_ok && tail_timing.detected_peaks >= 4;
}

void build_sfd_anchored_timing(
    const SfdAnchorConfig& cfg,
    const TimingResult& prev_timing,
    const TimingResult& initial_timing,
    const TimingResult& tail_timing,
    bool have_tail_peaks,
    TimingResult& cfo_timing)
{
    // Keep the SFD-anchored full-frame grid on the original timing
    // estimate.  Only the tail peaks have matched-filter measurements;
    // leading peaks are generated from the SFD-anchored grid.
    const double measured_period = cfg.period_hint;
    const int64_t full_start = cfg.bootstrap_sfd_start -
        static_cast<int64_t>(std::llround(
            static_cast<double>(cfg.full_repetitions) * measured_period));
    // The SFD-anchored grid replaces the seeded search result, but the
    // synchronization-strategy diagnostics describe the FIRST seeded search
    // (the only one the strategy controls), so carry them over.
    cfo_timing.bounded_attempts = prev_timing.bounded_attempts;
    cfo_timing.bounded_success = prev_timing.bounded_success;
    cfo_timing.bounded_failures = prev_timing.bounded_failures;
    cfo_timing.bounded_legacy_scans = prev_timing.bounded_legacy_scans;
    cfo_timing.bounded_inner_used = prev_timing.bounded_inner_used;
    cfo_timing.bounded_roi_samples = prev_timing.bounded_roi_samples;
    cfo_timing.backtrack_symbols = prev_timing.backtrack_symbols;
    // Diagnostic-only carry-over (no interval math): the locked-tracking
    // diagnostics describe the FIRST seeded search, which is the only one
    // the strategy controls, and res.timing is replaced below.
    cfo_timing.grid_anchor_attempts = prev_timing.grid_anchor_attempts;
    cfo_timing.grid_anchor_hits = prev_timing.grid_anchor_hits;
    cfo_timing.grid_single_points = prev_timing.grid_single_points;
    cfo_timing.grid_single_valid = prev_timing.grid_single_valid;
    cfo_timing.grid_fallbacks = prev_timing.grid_fallbacks;
    cfo_timing.tail_probe_attempts = prev_timing.tail_probe_attempts;
    cfo_timing.tail_probe_hits = prev_timing.tail_probe_hits;
    cfo_timing.tail_probe_k = prev_timing.tail_probe_k;
    cfo_timing.tail_probe_best_metric = prev_timing.tail_probe_best_metric;
    cfo_timing.tail_probe_raw_max = prev_timing.tail_probe_raw_max;
    cfo_timing.inner_probe_raw_max = prev_timing.inner_probe_raw_max;
    cfo_timing.initial_coarse_search_us = prev_timing.coarse_search_us;
    cfo_timing.initial_fine_track_us = prev_timing.fine_track_us;
    cfo_timing.ok = true;
    cfo_timing.preamble_start_sample = full_start;
    cfo_timing.preamble_start_uncropped = full_start;
    cfo_timing.measured_period = measured_period;
    cfo_timing.metric =
        have_tail_peaks ? tail_timing.metric : prev_timing.metric;
    cfo_timing.detected_peaks = initial_timing.detected_peaks;
    cfo_timing.sync_peaks_folded = initial_timing.sync_peaks_folded;
    cfo_timing.cir_grid_repetitions = cfg.full_repetitions;
    cfo_timing.expected_peaks = cfg.full_repetitions;
    cfo_timing.coarse_search_us = tail_timing.coarse_search_us;
    cfo_timing.fine_track_us = tail_timing.fine_track_us;
    cfo_timing.seed_search_us = tail_timing.seed_search_us;
    cfo_timing.seed_refine_narrow_attempts =
        tail_timing.seed_refine_narrow_attempts;
    cfo_timing.seed_refine_narrow_success =
        tail_timing.seed_refine_narrow_success;
    cfo_timing.seed_refine_fallbacks = tail_timing.seed_refine_fallbacks;
    cfo_timing.peak_samples.resize(cfg.full_repetitions);
    cfo_timing.peak_metrics.assign(cfg.full_repetitions, 0.0f);
    cfo_timing.peak_corr.assign(
        cfg.full_repetitions, std::complex<float>(0.0f, 0.0f));
    for (size_t k = 0; k < cfg.full_repetitions; ++k) {
        cfo_timing.peak_samples[k] = full_start +
            static_cast<int64_t>(std::llround(
                static_cast<double>(k) * measured_period)) +
            static_cast<int64_t>(cfg.template_len - 1);
    }
    if (!have_tail_peaks)
        return;

    if (tail_timing.peak_samples.size() == cfg.tail_repetitions) {
        for (size_t k = 0; k < cfg.tail_repetitions; ++k) {
            const size_t dst = cfg.tail_first + k;
            cfo_timing.peak_samples[dst] = tail_timing.peak_samples[k];
            cfo_timing.peak_metrics[dst] = tail_timing.peak_metrics[k];
            cfo_timing.peak_corr[dst] = tail_timing.peak_corr[k];
        }
        return;
    }

    const int64_t tail_pk0 =
        cfg.tail_start + static_cast<int64_t>(cfg.template_len - 1);
    for (size_t i = 0; i < tail_timing.peak_samples.size(); ++i) {
        const int64_t rel_sym = std::llround(
            static_cast<double>(tail_timing.peak_samples[i] - tail_pk0) /
            measured_period);
        if (rel_sym >= 0 &&
            static_cast<size_t>(rel_sym) < cfg.tail_repetitions) {
            const size_t dst = cfg.tail_first + static_cast<size_t>(rel_sym);
            cfo_timing.peak_samples[dst] = tail_timing.peak_samples[i];
            cfo_timing.peak_metrics[dst] = tail_timing.peak_metrics[i];
            cfo_timing.peak_corr[dst] = tail_timing.peak_corr[i];
        }
    }
}

bool refit_tail_cfo_and_rotate_anchored_prefix(
    const std::complex<float>* rx,
    size_t n,
    const Qm35825Profile& profile,
    bool integer_grid_ok,
    const SfdAnchorConfig& cfg,
    const TimingResult& tail_timing,
    const TimingResult& cfo_timing,
    bool have_tail_peaks,
    DemodScratch& scratch,
    DemodResult& res,
    SyncBootstrapState& sync_state)
{
    const auto sfd_cfo_t0 = std::chrono::steady_clock::now();
    // Only the tail has measured correlation phases.  The reconstructed
    // full-frame grid above contains zero placeholders for earlier SYNCs
    // and is for downstream timing/CIR, not carrier estimation.  Startup
    // exclusion belongs to the initial acquisition fit; this SFD-anchored
    // tail is already the selected stable training interval.
    if (have_tail_peaks) {
        // Avoid overwriting a well-sampled full-preamble Stage-2 CFO
        // estimate (e.g. >=16 peaks spanning 256 symbols) with a high-
        // variance short-baseline tail refit (<16 noisy peaks at ~-10 dB).
        const bool refit_well_conditioned =
            tail_timing.peak_samples.size() >= 16 ||
            sync_state.stage2_cfo.peaks_used < 16;
        if (refit_well_conditioned) {
            Qm35825Profile tail_fit_profile = profile;
            tail_fit_profile.cfo_skip_initial_repetitions = 0;
            CfoResult tail_cfo;
            const bool cfo_fit_ok =
                fit_cfo(rx, n, tail_fit_profile, tail_timing, tail_cfo, scratch);
            if (cfo_fit_ok) {
                res.cfo = tail_cfo;
                res.cfo.skipped_peaks += cfg.tail_first;
                res.cfo_refit_hz = tail_cfo.cfo_hz;
                res.cfo_refit_peaks = static_cast<uint32_t>(tail_cfo.peaks_used);
                res.cfo_refit_fit_us = tail_cfo.fit_us;
            }
            res.stage_cfo_fit_us += res.cfo.fit_us;
        }
    }
    res.stage_sfd_cfo_refit_us += static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - sfd_cfo_t0).count());
    if (!res.cfo.ok)
        return false;

    // Second rotation (Task A1): covers final SFD, CIR (54 SYNCs starting
    // from 10th), phase estimation, and initial soft-chips / PHR.
    const SampleRange final_sfd_range = compute_sfd_window_range(
        n, cfo_timing, cfg.full_repetitions, sync_state.sfd_len);
    const SampleRange sec_range = compute_anchored_prefix_range(
        n, profile, integer_grid_ok, cfo_timing, final_sfd_range);

    // ---- Single-pass rotation decision ---------------------------------
    // The second rotation exists only because the SFD-anchored refit moves
    // the estimate.  When the Stage-2 estimate is good enough, the pass-1
    // rotation (already widened to the predicted union range) is the only
    // derotation the packet needs.  Every condition below is a hard
    // requirement; any miss keeps the legacy refit + second rotation.
    const uint64_t refit_span = static_cast<uint64_t>(sec_range.size());
    const double refit_delta_hz = res.cfo.cfo_hz - sync_state.stage2_cfo.cfo_hz;
    const double refit_drift_rad = std::abs(
        2.0 * M_PI * refit_delta_hz *
        static_cast<double>(refit_span) / profile.sample_rate);
    res.cfo_refit_span_samples = refit_span;
    res.cfo_refit_phase_drift_rad = refit_drift_rad;
    bool want_skip = false;
    uint32_t skip_action = 0;
    if (profile.cfo_refit_mode == 2) {
        want_skip = true;
        skip_action = 2;
    } else if (profile.cfo_refit_mode == 1) {
        want_skip =
            sync_state.stage2_cfo.peaks_used >= profile.cfo_refit_min_peaks &&
            refit_drift_rad <= profile.cfo_refit_skip_rad;
        skip_action = 1;
    }
    if (want_skip &&
        sync_state.predicted_span.covers_with_cfo(
            sec_range, sync_state.stage2_cfo.cfo_hz, 0)) {
        // Keep the Stage-2 estimate: pass 1 already applied it to the whole
        // predicted span, which provably contains [sec_range.lo, sec_range.hi).
        res.cfo = sync_state.stage2_cfo;
        res.cfo_refit_action = skip_action;
        sync_state.derotated_span = sync_state.predicted_span;
        return true;
    }
    if (profile.cfo_refit_mode != 0)
        res.cfo_refit_action = 3; // requested skip, fell back (fail-closed)
    const RotationStepResult rot2 = rotate_window_range(
        rx, n, profile, res.cfo.cfo_hz, sec_range, 2u, scratch, res);
    res.cfo.rotate_us = rot2.rotate_us;
    res.cfo.rotation_kernel = rot2.kernel;
    res.cfo_refit_rotate_us = rot2.rotate_us;
    res.stage_sfd_cfo_refit_us += rot2.rotate_us;
    sync_state.derotated_span = DerotatedSpan{
        sec_range, res.cfo.cfo_hz, 0, !sec_range.empty()};
    return true;
}

bool run_tail_retrack_and_cfo_refit(
    const std::complex<float>* rx,
    size_t n,
    const Qm35825Profile& profile,
    const std::vector<std::complex<float>>& template_wf,
    bool integer_grid_ok,
    const SfdAnchorConfig& cfg,
    size_t seed_radius,
    DemodScratch& scratch,
    DemodResult& res,
    SyncBootstrapState& sync_state,
    TimingResult& tail_timing,
    TimingResult& cfo_timing)
{
    const bool have_tail_peaks = retrack_tail_timing(
        rx, n, profile, template_wf, cfg, seed_radius, scratch, res,
        tail_timing);
    build_sfd_anchored_timing(
        cfg, res.timing, scratch.initial_timing, tail_timing,
        have_tail_peaks, cfo_timing);
    return refit_tail_cfo_and_rotate_anchored_prefix(
        rx, n, profile, integer_grid_ok, cfg, tail_timing, cfo_timing,
        have_tail_peaks, scratch, res, sync_state);
}

bool anchor_and_verify_final_sfd(
    const std::complex<float>* rx,
    size_t n,
    const Qm35825Profile& profile,
    const std::vector<std::complex<float>>& template_wf,
    bool integer_grid_ok,
    SyncBootstrapState& sync_state,
    StageClock& clock,
    DemodScratch& scratch,
    DemodResult& res)
{
    // Use the SFD as the frame anchor.  The first SYNCs can be weak or can be
    // missed by acquisition, so do not re-track the whole preamble from its
    // reconstructed first repetition.  Track only the stable tail used by
    // CIR/CFO, then rebuild the complete SYNC grid from the SFD coordinate.
    const size_t full_repetitions = profile.preamble_repetitions;
    const size_t tail_repetitions = std::min(
        full_repetitions,
        std::max<size_t>(profile.cfo_min_fit_repetitions, 40));
    if (tail_repetitions < 2 || res.timing.measured_period <= 0.0) {
        clock.lap(res.stage_sfd_us, res.stage_total_us);
        res.status = DemodStatus::CfoFailed;
        return false;
    }

    SfdAnchorConfig cfg;
    cfg.full_repetitions = full_repetitions;
    cfg.tail_repetitions = tail_repetitions;
    cfg.tail_first = (full_repetitions >= tail_repetitions)
        ? (full_repetitions - tail_repetitions)
        : 0;
    cfg.bootstrap_sfd_start = res.sfd.sfd_start_sample;
    cfg.period_hint = res.timing.measured_period;
    cfg.tail_start = cfg.bootstrap_sfd_start -
        static_cast<int64_t>(std::llround(
            static_cast<double>(tail_repetitions) * cfg.period_hint));
    cfg.template_len = template_wf.size();
    cfg.cfo_anchor_profile = profile;
    cfg.cfo_anchor_profile.preamble_repetitions = tail_repetitions;
    // The SFD anchor supplies the absolute tail origin.  Use a local ±8 sample
    // refinement at each tail SYNC, but do not allow acquisition to jump to a
    // different symbol or walk backwards toward weak leading repetitions.
    cfg.cfo_anchor_profile.timing_search_margin = 0;
    cfg.cfo_anchor_profile.timing_coarse_stride = 1;
    cfg.cfo_anchor_profile.timing_max_backtrack_symbols = 0;
    cfg.cfo_anchor_profile.timing_track_radius = 8;
    // Opt-in: the SFD anchor predicts this tail origin to within a few samples;
    // try ±8 first and fall back to the legacy ±max(256,stride) search below.
    cfg.cfo_anchor_profile.timing_seed_refine_radius =
        profile.rx_seed_refine_tail ? 8 : 0;
    // Tail tracking floor.  MATLAB's 0.20 of the candidate energy is an
    // ABSOLUTE gate: at -3.5..-4 dB in-band SNR the tail SYNC peaks sit at
    // 0.18..0.23, so a hard 0.20 rejects half of them and the refit dies with
    // `detected_peaks < 4` (measured: 23/100 CFO_FAILED at -3.5 dB).  The tail
    // origin is already pinned by bootstrap_sfd to +-8 samples, so the
    // false-alarm risk that the absolute gate was protecting against does not
    // apply here; use the same 0.08 floor LockedBounded uses for its bounded
    // tail probe.  `stage_timing`'s narrow seed searches additionally enable
    // the PAR gate (PAR >= 20, floor 0.06).
    cfg.cfo_anchor_profile.radar_verification_threshold =
        std::min(profile.radar_verification_threshold, 0.08f);

    const std::vector<int8_t>& sfd_seq = scratch.cached_seq.sfd_seq;
    auto run_final_sfd = [&]() -> bool {
        const auto sfd_final_t0 = std::chrono::steady_clock::now();
        const bool ok = search_sfd_with_wide_rescue(
            rx, n, profile, sfd_seq, template_wf, sync_state.sfd_len,
            scratch, res);
        res.stage_sfd_final_us += static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::steady_clock::now() - sfd_final_t0).count());
        return ok;
    };

    TimingResult tail_timing;
    TimingResult cfo_timing;
    const size_t first_seed_radius = profile.rx_seed_refine_tail ? 8 : 0;
    if (!run_tail_retrack_and_cfo_refit(
            rx, n, profile, template_wf, integer_grid_ok, cfg,
            first_seed_radius, scratch, res, sync_state,
            tail_timing, cfo_timing)) {
        clock.lap(res.stage_sfd_us, res.stage_total_us);
        res.status = DemodStatus::CfoFailed;
        return false;
    }
    res.timing = std::move(cfo_timing);

    // Re-run SFD on the final MATLAB-aligned CFO compensation.  This removes
    // any bootstrap-CFO bias from the SFD/CIR path while retaining the same
    // full-rate refinement contract as MATLAB refineTimingWithNsSfd.
    bool final_sfd_ok = run_final_sfd();
    if (!final_sfd_ok && profile.rx_seed_refine_tail) {
        // The narrow tail seed can pick a wrong local peak and still pass the
        // per-SYNC threshold.  On a final-SFD/period inconsistency, retry the
        // complete tail -> CFO refit -> final SFD chain with the legacy radius.
        ++res.stage_tail_retrack_fallbacks;
        TimingResult retry_tail;
        TimingResult retry_cfo;
        if (run_tail_retrack_and_cfo_refit(
                rx, n, profile, template_wf, integer_grid_ok, cfg, 0,
                scratch, res, sync_state, retry_tail, retry_cfo)) {
            // Preserve the first (narrow) chain's seed diagnostics: the
            // radius-0 retry reports zero attempts of its own.
            retry_cfo.seed_search_us += res.timing.seed_search_us;
            retry_cfo.seed_refine_narrow_attempts +=
                res.timing.seed_refine_narrow_attempts;
            retry_cfo.seed_refine_narrow_success +=
                res.timing.seed_refine_narrow_success;
            retry_cfo.seed_refine_fallbacks +=
                res.timing.seed_refine_fallbacks;
            res.timing = std::move(retry_cfo);
            final_sfd_ok = run_final_sfd();
        }
    }
    res.sfd_final_windows = res.sfd.search_windows;
    res.sfd_final_coarse_correlations = res.sfd.coarse_correlations;
    res.sfd_final_fine_correlations = res.sfd.fine_correlations;
    if (!final_sfd_ok) {
        clock.lap(res.stage_sfd_us, res.stage_total_us);
        res.status = DemodStatus::SfdFailed;
        return false;
    }
    res.timing.preamble_start_sample =
        res.sfd.sfd_start_sample - static_cast<int64_t>(std::llround(
            static_cast<double>(profile.preamble_repetitions) *
            res.timing.measured_period));
    clock.lap(res.stage_sfd_us, res.stage_total_us);
    // Diagnostic remainder of the SFD parent stage.  The four child sub-timers
    // above are strictly children of `stage_sfd_us` — the parent already
    // includes them, so this value is diagnostics only and is never added to
    // `stage_sfd_us` or `stage_total_us`.  Clamp at 0 so an independent-clock
    // overshoot can never produce a negative (wrapped) value.
    {
        const uint64_t sfd_children_us =
            res.stage_sfd_bootstrap_us + res.stage_sfd_tail_retrack_us +
            res.stage_sfd_cfo_refit_us + res.stage_sfd_final_us;
        res.stage_sfd_anchor_other_us =
            (res.stage_sfd_us >= sfd_children_us)
                ? res.stage_sfd_us - sfd_children_us
                : 0;
    }
    return true;
}

// Diagnostic-only segmented FIR replay.  It is intentionally outside the
// lap/add_elapsed parents and only runs when the explicit diagnostic
// profile flag is set; formal timing runs leave it off.
void maybe_diagnose_fir(
    const Qm35825Profile& profile,
    size_t n,
    DemodScratch& scratch,
    DemodResult& res)
{
    if (!scratch.diagnose_fir_segments)
        return;
    if (!(res.cir.ok && res.ns_sfd.ok && res.phr.ok && res.payload.ok))
        return;
    const size_t tap_count = res.cir.cir_filter_taps.size();
    if (tap_count == 0 || tap_count > 64 || res.cir.soft_chip_count == 0)
        return;
    const int64_t chip_start =
        res.timing.preamble_start_sample + static_cast<int64_t>(tap_count) -
        static_cast<int64_t>(profile.cir_pre_samples) - 1;
    if (chip_start < 0)
        return;
    size_t payload_symbols = 0;
    if (res.phr.psdu_length > 0) {
        const size_t psdu_bits =
            static_cast<size_t>(res.phr.psdu_length) * 8;
        const size_t num_blocks = (psdu_bits + 329) / 330;
        payload_symbols = psdu_bits + 48 * num_blocks;
    }
    detail::SoftFirSegments segments;
    if (!detail::diagnose_softchip_fir_segments(
            scratch.derotated.data(), n, res.cir.cir_filter_taps.data(),
            tap_count, chip_start, res.cir.soft_chip_count,
            res.cir.samples_per_chip, res.ns_sfd, res.phr,
            payload_symbols, scratch.corr, segments))
        return;
    res.diag_fir_valid = 1;
    res.diag_fir_preamble_us = segments.preamble_us;
    res.diag_fir_sfd_us = segments.sfd_us;
    res.diag_fir_phr_us = segments.phr_us;
    res.diag_fir_payload_us = segments.payload_us;
    res.diag_fir_extra_us = segments.extra_us;
    res.diag_fir_preamble_chips = static_cast<uint32_t>(
        segments.preamble_chips);
    res.diag_fir_sfd_chips = static_cast<uint32_t>(segments.sfd_chips);
    res.diag_fir_phr_chips = static_cast<uint32_t>(segments.phr_chips);
    res.diag_fir_payload_chips = static_cast<uint32_t>(
        segments.payload_chips);
    res.diag_fir_extra_chips = static_cast<uint32_t>(segments.extra_chips);
}

struct FullSoftchipDecodeOutputs {
    CirResult& cir;
    NsSfdResult& ns_sfd;
    PhrResult& phr;
    PayloadResult& payload;
};

bool run_full_softchip_decode_stages(
    size_t n,
    const Qm35825Profile& cir_profile,
    const Qm35825Profile& decode_profile,
    const TimingResult& timing,
    const std::vector<int8_t>& pcode,
    const std::vector<int8_t>& nsfd_seq,
    StageClock* clock,
    PayloadSubTimers* payload_sub,
    DemodScratch& scratch,
    FullSoftchipDecodeOutputs out,
    DemodResult& res,
    DemodStatus& status_out)
{
    const auto lap_if = [&](uint64_t& dst) {
        if (clock != nullptr)
            clock->lap(dst, res.stage_total_us);
    };

    if (!stage_cir_softchips(scratch.derotated.data(), n, cir_profile, timing,
                             pcode, out.cir, scratch)) {
        lap_if(res.stage_cir_us);
        status_out = DemodStatus::CirFailed;
        return false;
    }
    lap_if(res.stage_cir_us);

    if (!stage_ns_sfd(scratch.soft_chips, decode_profile, nsfd_seq,
                      kQm35ChipsPerSymbol, out.ns_sfd, scratch)) {
        lap_if(res.stage_ns_sfd_us);
        status_out = DemodStatus::SfdFailed;
        return false;
    }
    lap_if(res.stage_ns_sfd_us);

    if (!stage_phr(scratch.soft_chips, decode_profile, out.ns_sfd,
                   kQm35ChipsPerSymbol, out.phr, scratch)) {
        lap_if(res.stage_phr_us);
        status_out = DemodStatus::PhrFailed;
        return false;
    }
    lap_if(res.stage_phr_us);

    if (!stage_payload_fcs(scratch.soft_chips, decode_profile, out.phr,
                           out.ns_sfd, kQm35ChipsPerSymbol, out.payload,
                           scratch, payload_sub)) {
        lap_if(res.stage_payload_us);
        status_out = DemodStatus::PayloadFailed;
        return false;
    }
    lap_if(res.stage_payload_us);

    status_out = (out.payload.ok && out.payload.fcs_pass)
                     ? DemodStatus::Success
                     : DemodStatus::FcsFailed;
    return true;
}

void run_primary_full_stream_decode(
    size_t n,
    const Qm35825Profile& profile,
    const Qm35825Profile& eff,
    StageClock& clock,
    PayloadSubTimers& payload_sub,
    DemodScratch& scratch,
    DemodResult& res)
{
    FullSoftchipDecodeOutputs out{res.cir, res.ns_sfd, res.phr, res.payload};
    if (run_full_softchip_decode_stages(
            n, eff, profile, res.timing,
            scratch.cached_seq.pcode, scratch.cached_seq.sfd_seq,
            &clock, &payload_sub, scratch, out, res, res.status)) {
        maybe_diagnose_fir(profile, n, scratch, res);
    }
}

void run_full_stream_fallback(
    const std::complex<float>* rx,
    size_t n,
    const Qm35825Profile& profile,
    DerotatedSpan& derotated_span,
    StageClock& clock,
    DemodScratch& scratch,
    DemodResult& res)
{
    // Single full-stream retry on the legacy path.  Measure it as a
    // separate diagnostic bucket so the first-pass stage timings are
    // not silently replaced or double-counted.
    ++res.stage_softchip_fallbacks;
    // Fallback: compensate full window on legacy retry (Task A1).
    // Record fallback_t0 before rotation so entire fallback cost is
    // captured in stage_softchip_fallback_us and stage_total_us.
    const auto fallback_t0 = std::chrono::steady_clock::now();
    uint64_t rot_fb_us = 0;
    uint32_t rk_fb = 0;
    derotate_cfo_range(rx, n, profile, res.cfo.cfo_hz, 0, n, scratch,
                       rot_fb_us, rk_fb);
    res.stage_cfo_rotate_us += rot_fb_us;
    res.stage_cfo_rotated_samples += n;
    ++res.stage_cfo_rotation_passes;
    derotated_span = DerotatedSpan{
        SampleRange{0, n}, res.cfo.cfo_hz, 0, n > 0};

    CirResult full_cir;
    NsSfdResult full_ns;
    PhrResult full_phr;
    PayloadResult full_payload;
    DemodStatus fallback_status = DemodStatus::PayloadFailed;
    FullSoftchipDecodeOutputs out{full_cir, full_ns, full_phr, full_payload};
    const bool all_stages_ok = run_full_softchip_decode_stages(
        n, profile, profile, res.timing,
        scratch.cached_seq.pcode, scratch.cached_seq.sfd_seq,
        /*clock=*/nullptr, /*payload_sub=*/nullptr,
        scratch, out, res, fallback_status);

    res.stage_softchip_fallback_us = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - fallback_t0).count());
    res.stage_total_us += res.stage_softchip_fallback_us;
    clock.t_prev = std::chrono::steady_clock::now();

    // Publish the fallback attempt even when it fails: the status
    // below is decided by this legacy run, so the reported CIR /
    // NS-SFD / PHR / payload must describe the same run.  Otherwise a
    // short prefix result is paired with a full-stream failure status.
    res.cir = std::move(full_cir);
    res.cir.on_demand_fallbacks = 1;
    res.ns_sfd = std::move(full_ns);
    res.phr = std::move(full_phr);
    res.payload = std::move(full_payload);
    res.status = fallback_status;
    if (all_stages_ok) {
        maybe_diagnose_fir(profile, n, scratch, res);
    }
}

uint64_t rotate_uncovered_tail_range(
    const std::complex<float>* rx,
    size_t n,
    const Qm35825Profile& profile,
    const SampleRange& pay_range,
    DerotatedSpan& derotated_span,
    DemodScratch& scratch,
    DemodResult& res)
{
    uint64_t rot_pay_us = 0;
    uint32_t rk_pay = 0;
    derotate_cfo_range(rx, n, profile, res.cfo.cfo_hz,
                       pay_range.lo, pay_range.hi, scratch,
                       rot_pay_us, rk_pay);
    res.stage_cfo_rotate_us += rot_pay_us;
    res.stage_cfo_rotated_samples += pay_range.size();
    ++res.stage_cfo_rotation_passes;
    if (!derotated_span.valid ||
        derotated_span.cfo_hz != res.cfo.cfo_hz) {
        derotated_span = DerotatedSpan{
            pay_range, res.cfo.cfo_hz, 0, true};
    } else {
        derotated_span.range.lo = std::min(
            derotated_span.range.lo, pay_range.lo);
        derotated_span.range.hi = std::max(
            derotated_span.range.hi, pay_range.hi);
    }
    return rot_pay_us;
}

struct PrefixPhrAttempt {
    size_t prefix_chips = 0;
    bool phr_fused_used = false;
};

bool run_prefix_cir_ns_sfd_and_phr(
    size_t n,
    const Qm35825Profile& profile,
    const Qm35825Profile& eff,
    bool integer_grid_ok,
    StageClock& clock,
    DemodScratch& scratch,
    DemodResult& res,
    PrefixPhrAttempt& out)
{
    const std::vector<int8_t>& pcode = scratch.cached_seq.pcode;
    const std::vector<int8_t>& nsfd_seq = scratch.cached_seq.sfd_seq;
    const bool phr_fusion_requested = profile.rx_fused_phr_cmf;
    const bool phr_fusion_cfg_supported =
        profile.rx_fused_payload_cmf &&
        profile.cir_rake_top_k == 0 &&
        profile.cir_soft_chip_mode != CirSoftChipMode::Rake &&
        profile.cir_soft_chip_mode != CirSoftChipMode::Bypass;
    const size_t prefix_budget =
        phr_fusion_requested ? rxopt::fused_phr_prefix_budget(profile)
                             : rxopt::initial_chip_budget(profile);
    bool ok = stage_cir_softchips(scratch.derotated.data(), n, eff,
                                  res.timing, pcode, res.cir, scratch,
                                  prefix_budget, 0);
    clock.lap(res.stage_cir_us, res.stage_total_us);
    out.prefix_chips = res.cir.soft_chip_count;
    if (ok) {
        ok = stage_ns_sfd(scratch.soft_chips, profile, nsfd_seq,
                          kQm35ChipsPerSymbol, res.ns_sfd, scratch);
        clock.lap(res.stage_ns_sfd_us, res.stage_total_us);
    }
    out.phr_fused_used = false;
    if (ok) {
        if (phr_fusion_requested && integer_grid_ok) {
            const bool supported =
                phr_fusion_cfg_supported &&
                !res.cir.cir_filter_taps.empty() &&
                integer_chip_grid_at(res.cir.samples_per_chip,
                                     rxfusion::kSamplesPerChip,
                                     profile.fused_gate_ppm * 1e-6) &&
                scratch.softchip.prepared;
            if (supported) {
                ++res.stage_phr_fused_attempts;
                uint64_t phr_template_us = 0;
                uint64_t phr_correlate_us = 0;
                uint64_t phr_decode_us = 0;
                uint64_t phr_decode_ns = 0;
                size_t phr_symbols = 0;
                const bool fused_ok = stage_phr_fused_cmf(
                    scratch.derotated.data(), n, profile, res.timing,
                    res.cir, scratch.soft_chips, res.ns_sfd, res.phr,
                    scratch, phr_template_us, phr_correlate_us,
                    phr_decode_us, phr_decode_ns, phr_symbols);
                res.stage_phr_fused_template_us += phr_template_us;
                res.stage_phr_fused_correlate_us += phr_correlate_us;
                res.stage_phr_fused_decode_us += phr_decode_us;
                res.stage_phr_fused_decode_ns += phr_decode_ns;
                res.stage_phr_fused_symbols =
                    static_cast<uint32_t>(phr_symbols);
                clock.lap(res.stage_phr_us, res.stage_total_us);
                // A SECDED correction needs the full-stream tail-energy
                // guard.  The reduced prefix cannot reproduce it, so keep
                // the existing conservative full-stream retry.
                if (fused_ok && res.phr.ok && !res.phr.secded_corrected) {
                    out.phr_fused_used = true;
                } else {
                    ++res.stage_phr_fused_fallbacks;
                    ok = false;
                }
            } else {
                // The explicit option only supports the full integer-grid
                // CMF.  Do not mix a fused PHR request with the legacy PHR
                // kernel; let the single full-stream retry below decide.
                ok = false;
            }
        } else {
            ok = stage_phr(scratch.soft_chips, profile, res.ns_sfd,
                           kQm35ChipsPerSymbol, res.phr, scratch);
            clock.lap(res.stage_phr_us, res.stage_total_us);
        }
    }
    if (ok && !out.phr_fused_used && res.phr.secded_corrected) {
        // The legacy PHR path can inspect the full-range tail energy when
        // SECDED corrected a longer length.  The prefix cannot, so retry
        // the complete stream rather than silently changing that decision.
        ok = false;
    }
    return ok;
}

enum class FusedPayloadOutcome {
    NotRun,
    Succeeded,
    FailedToFallback,
};

FusedPayloadOutcome try_fused_payload_decode(
    const std::complex<float>* rx,
    size_t n,
    const Qm35825Profile& profile,
    bool integer_grid_ok,
    bool phr_fused_used,
    DerotatedSpan& derotated_span,
    StageClock& clock,
    PayloadSubTimers& payload_sub,
    DemodScratch& scratch,
    DemodResult& res)
{
    if (!profile.rx_fused_payload_cmf || !integer_grid_ok)
        return FusedPayloadOutcome::NotRun;

    const bool fused_supported =
        !res.cir.cir_filter_taps.empty() &&
        integer_chip_grid_at(res.cir.samples_per_chip,
                             rxfusion::kSamplesPerChip,
                             profile.fused_gate_ppm * 1e-6) &&
        profile.cir_rake_top_k == 0 &&
        profile.cir_soft_chip_mode != CirSoftChipMode::Rake &&
        profile.cir_soft_chip_mode != CirSoftChipMode::Bypass &&
        scratch.softchip.prepared;
    if (!fused_supported)
        return FusedPayloadOutcome::NotRun;

    // Compensate payload interval with refit CFO (Task A1)
    const size_t covered_hi =
        derotated_span.covers_with_cfo(
            derotated_span.range, res.cfo.cfo_hz, 0)
            ? derotated_span.range.hi
            : 0;
    const SampleRange pay_range =
        compute_fused_payload_rotation_range(
            n, profile, res.timing, res.cir.cir_filter_taps.size(),
            res.ns_sfd, res.phr, covered_hi);
    if (!pay_range.empty()) {
        // This payload-region CFO derotation runs inside the
        // `stage_payload_us` lap but outside the payload decoder, so
        // attribute it to the payload `other` bucket explicitly;
        // otherwise the child sum would fall short of the parent by
        // the rotation cost (~36 µs on the golden window).
        const auto t_payrot = std::chrono::steady_clock::now();
        (void)rotate_uncovered_tail_range(
            rx, n, profile, pay_range, derotated_span, scratch, res);
        const uint64_t payrot_us =
            detail::payload_elapsed_us(t_payrot);
        payload_sub.other_us += payrot_us;
        payload_sub.other_derotate_us += payrot_us;
    }

    ++res.stage_fused_attempts;
    uint64_t fused_template_us = 0;
    uint64_t fused_correlate_us = 0;
    uint64_t fused_decode_us = 0;
    uint64_t fused_decode_ns = 0;
    size_t fused_symbols = 0;
    const std::vector<int8_t>* cached_g0 =
        (phr_fused_used && scratch.fused_phr.decisions_valid)
            ? &scratch.fused_phr.g0
            : nullptr;
    const std::vector<int8_t>* cached_g1 =
        (phr_fused_used && scratch.fused_phr.decisions_valid)
            ? &scratch.fused_phr.g1
            : nullptr;
    // Same cache, soft form: the fused PHR LLRs feed the payload
    // soft retry without re-demodulating the PHR field.
    const std::vector<float>* cached_llr0 =
        (phr_fused_used && scratch.fused_phr.decisions_valid)
            ? &scratch.fused_phr.llr0
            : nullptr;
    const std::vector<float>* cached_llr1 =
        (phr_fused_used && scratch.fused_phr.decisions_valid)
            ? &scratch.fused_phr.llr1
            : nullptr;
    const bool fused_ok = stage_payload_fused_cmf(
        scratch.derotated.data(), n, profile, res.timing, res.cir,
        scratch.soft_chips, res.ns_sfd, res.phr, res.payload,
        scratch, fused_template_us, fused_correlate_us,
        fused_decode_us, fused_decode_ns, fused_symbols,
        cached_g0, cached_g1, &payload_sub, cached_llr0,
        cached_llr1);
    res.stage_fused_template_us += fused_template_us;
    res.stage_fused_correlate_us += fused_correlate_us;
    res.stage_fused_decode_us += fused_decode_us;
    res.stage_fused_decode_ns += fused_decode_ns;
    res.stage_fused_symbols = static_cast<uint32_t>(fused_symbols);
    clock.lap(res.stage_payload_us, res.stage_total_us);
    if (fused_ok && res.payload.fcs_pass) {
        res.status = DemodStatus::Success;
        maybe_diagnose_fir(profile, n, scratch, res);
        return FusedPayloadOutcome::Succeeded;
    }
    ++res.stage_fused_fallbacks;
    return FusedPayloadOutcome::FailedToFallback;
}

bool try_ondemand_extension_and_payload_decode(
    const std::complex<float>* rx,
    size_t n,
    const Qm35825Profile& profile,
    const Qm35825Profile& eff,
    size_t prefix_chips,
    DerotatedSpan& derotated_span,
    StageClock& clock,
    PayloadSubTimers& payload_sub,
    DemodScratch& scratch,
    DemodResult& res)
{
    const size_t final_budget = rxopt::final_chip_budget(
        profile, res.ns_sfd.sfd_end_chip, res.phr.psdu_length);
    if (final_budget > prefix_chips) {
        // Compensate extension interval with refit CFO (Task A1)
        const size_t covered_hi =
            derotated_span.covers_with_cfo(
                derotated_span.range, res.cfo.cfo_hz, 0)
                ? derotated_span.range.hi
                : 0;
        const SampleRange pay_range =
            compute_ondemand_extension_rotation_range(
                n, profile, res.timing, res.cir.cir_filter_taps.size(),
                final_budget, covered_hi);
        if (!pay_range.empty()) {
            (void)rotate_uncovered_tail_range(
                rx, n, profile, pay_range, derotated_span, scratch, res);
        }

        CirResult ext;
        const bool ext_ok = stage_cir_softchips(
            scratch.derotated.data(), n, eff, res.timing,
            scratch.cached_seq.pcode, ext, scratch,
            final_budget, prefix_chips);
        clock.add_elapsed(res.stage_cir_us, res.stage_total_us);
        if (!ext_ok)
            return false;

        res.cir.soft_extend_us =
            ext.soft_fir_us + ext.postprocess_us;
        res.cir.soft_fir_us += ext.soft_fir_us;
        res.cir.postprocess_us += ext.postprocess_us;
        // The extension re-runs the deterministic CIR estimate;
        // count it so cir_estimate_us is the true total.
        res.cir.cir_estimate_us += ext.cir_estimate_us;
        res.cir.soft_chip_count = ext.soft_chip_count;
        res.cir.extended_chip_count = ext.extended_chip_count;
        res.cir.on_demand_extended = ext.on_demand_extended;
        res.cir.samples_per_chip = ext.samples_per_chip;
    }

    const bool ok = stage_payload_fcs(
        scratch.soft_chips, profile, res.phr, res.ns_sfd,
        kQm35ChipsPerSymbol, res.payload, scratch, &payload_sub);
    clock.lap(res.stage_payload_us, res.stage_total_us);
    if (!ok || !res.payload.fcs_pass)
        return false;

    res.status = DemodStatus::Success;
    maybe_diagnose_fir(profile, n, scratch, res);
    return true;
}

void run_on_demand_with_full_stream_fallback(
    const std::complex<float>* rx,
    size_t n,
    const Qm35825Profile& profile,
    const Qm35825Profile& eff,
    bool integer_grid_ok,
    DerotatedSpan& derotated_span,
    StageClock& clock,
    PayloadSubTimers& payload_sub,
    DemodScratch& scratch,
    DemodResult& res)
{
    PrefixPhrAttempt prefix_phr;
    bool ok = run_prefix_cir_ns_sfd_and_phr(
        n, profile, eff, integer_grid_ok, clock, scratch, res, prefix_phr);
    if (ok) {
        const FusedPayloadOutcome fused_outcome = try_fused_payload_decode(
            rx, n, profile, integer_grid_ok, prefix_phr.phr_fused_used,
            derotated_span, clock, payload_sub, scratch, res);
        if (fused_outcome == FusedPayloadOutcome::Succeeded)
            return;
        if (fused_outcome == FusedPayloadOutcome::FailedToFallback) {
            ok = false;
        } else {
            ok = try_ondemand_extension_and_payload_decode(
                rx, n, profile, eff, prefix_phr.prefix_chips,
                derotated_span, clock, payload_sub, scratch, res);
        }
    }
    if (!ok) {
        run_full_stream_fallback(
            rx, n, profile, derotated_span, clock, scratch, res);
    }
}

} // namespace

DemodResult demodulate_one(const std::complex<float>* rx,
                                  size_t n,
                                  const Qm35825Profile& profile,
                                  uint64_t packet_id,
                                  int64_t predicted_start,
                                  int64_t window_start,
                                  const std::vector<std::complex<float>>& template_wf,
                                  DemodScratch& scratch)
{
    DemodResult res;
    res.packet_id = packet_id;
    res.predicted_start_sample = predicted_start;
    res.window_start_sample = window_start;
    res.sync_strategy = profile.sync_strategy;
    // The fused PHR decisions are packet-scoped; never let the joint payload
    // Viterbi consume a previous packet's cache.
    scratch.begin_packet();

    // predicted_start is ABSOLUTE.  The core stages index rx as a 0-based
    // window buffer, so seed with the window-relative position.  All stage
    // outputs are window-relative; `rebase` re-bases the absolute sample
    // fields (timing / SFD sample positions / CIR first path) by window_start
    // before each return, so result coordinates match the schema (absolute).
    const int64_t rel_seed =
        (predicted_start >= 0) ? predicted_start - window_start : -1;
    // Diagnostic payload sub-stage timers.  They are children of
    // `stage_payload_us` (never added to `stage_total_us`) and are published
    // from `rebase()` so every exit path reports them without touching any
    // parent lap interval.
    PayloadSubTimers payload_sub;
    auto rebase = [&]() { finalize_demod_result(res, payload_sub, window_start); };

    // Per-stage wall-clock timing.  `tick` advances a lap clock; each lap
    // records the elapsed µs into the stage field AND accumulates the running
    // total, so early-failure paths still attribute the failed stage's time.
    StageClock clock;

    // Stage 1: timing (seed = rel_seed if inside this window, else -1)
    const bool t_ok = stage_timing(rx, n, profile, template_wf, rel_seed,
                                   res.timing, scratch);
    clock.lap(res.stage_timing_us, res.stage_total_us);
    res.stage_timing_coarse_us = res.timing.coarse_search_us;
    res.stage_timing_fine_track_us = res.timing.fine_track_us;
    if (!t_ok) {
        res.status = DemodStatus::TimingFailed;
        rebase();
        return res;
    }
    // Integer chip-grid admission: use the period fit that Stage 1 already
    // produced. The decision must be made here
    // and not later, because it controls three coupled things:
    //   * the second CFO rotation range (a narrowed range is only valid when
    //     the on-demand prefix is actually used),
    //   * whether the on-demand prefix is generated at all,
    //   * whether the fused PHR/payload CMF may run.
    // Mixing them -- e.g. narrowing the rotation and shortening the prefix but
    // then refusing the fused CMF -- leaves the payload decoder without PHR
    // soft chips, which fails the FCS and triggers a full-stream retry
    // (~16 ms/packet instead of ~2.6 ms).  Measured on the X410 capture with a
    // 1.5 ppm gate: 16% of packets hit exactly that path.
    const auto execution = plan_packet_execution(profile, res.timing);
    const bool integer_grid_ok = execution.integer_grid_ok;
    const auto& eff = execution.effective_profile;
    if (!integer_grid_ok)
        ++res.stage_grid_rejects;

    SyncBootstrapState sync_state;
    if (!run_initial_cfo_and_bootstrap_sfd(
            rx, n, profile, template_wf, clock, scratch, res, sync_state)) {
        rebase();
        return res;
    }

    if (!anchor_and_verify_final_sfd(
            rx, n, profile, template_wf, integer_grid_ok,
            sync_state, clock, scratch, res)) {
        rebase();
        return res;
    }

    // Early termination if caller only requested preamble (SYNC + SFD) detection.
    if (profile.rx_stop_after_sfd) {
        res.status = DemodStatus::Success;
        rebase();
        res.stage_total_us = res.stage_timing_us + res.stage_cfo_us + res.stage_sfd_us;
        return res;
    }

    // Stage 4: CIR + soft chips + PHR + payload/FCS on the CFO-compensated frame.
    const bool on_demand_path = integer_grid_ok &&
                                (profile.rx_softchip_on_demand ||
                                 profile.rx_fused_payload_cmf ||
                                 profile.rx_fused_phr_cmf);
    if (on_demand_path) {
        run_on_demand_with_full_stream_fallback(
            rx, n, profile, eff, integer_grid_ok,
            sync_state.derotated_span, clock, payload_sub, scratch, res);
    } else {
        run_primary_full_stream_decode(
            n, profile, eff, clock, payload_sub, scratch, res);
    }
    rebase();
    return res;
}


} // namespace gr::uwb::demod::core
