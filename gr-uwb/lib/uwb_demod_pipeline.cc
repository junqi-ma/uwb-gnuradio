// SPDX-License-Identifier: GPL-3.0-or-later
#include <gnuradio/uwb/uwb_demod_core.h>
#include <gnuradio/uwb/uwb_softchip_fir_diag.h>
#include <gnuradio/uwb/uwb_demod_execution.h>
#include <gnuradio/uwb/uwb_demod_finalize.h>
namespace gr::uwb::demod::core {
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
    auto t_prev = std::chrono::steady_clock::now();
    auto lap = [&](uint64_t& dst) {
        const auto now = std::chrono::steady_clock::now();
        dst = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::microseconds>(now - t_prev)
                .count());
        res.stage_total_us += dst;
        t_prev = now;
    };
    // Add (rather than replace) elapsed time.  Used for the on-demand extension
    // and failure fallback, which are not single sequential stage laps.
    auto add_elapsed = [&](uint64_t& dst) {
        const auto now = std::chrono::steady_clock::now();
        const uint64_t dt = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::microseconds>(now - t_prev)
                .count());
        dst += dt;
        res.stage_total_us += dt;
        t_prev = now;
    };

    // Diagnostic-only segmented FIR replay.  It is intentionally outside the
    // lap/add_elapsed parents and only runs when the explicit diagnostic
    // profile flag is set; formal timing runs leave it off.
    auto maybe_diagnose_fir = [&]() {
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
    };

    // Stage 1: timing (seed = rel_seed if inside this window, else -1)
    const bool t_ok = stage_timing(rx, n, profile, template_wf, rel_seed,
                                   res.timing, scratch);
    lap(res.stage_timing_us);
    res.stage_timing_coarse_us = res.timing.coarse_search_us;
    res.stage_timing_fine_track_us = res.timing.fine_track_us;
    if (!t_ok) {
        res.status = DemodStatus::TimingFailed;
        rebase();
        return res;
    }
    // Integer chip-grid admission: decide ONCE per packet, from the 256-peak
    // period fit that Stage 1 already produced.  The decision must be made here
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

    // Stage 2: CFO fit + bootstrap SFD search window rotation (Task A1)
    if (!fit_cfo(rx, n, profile, res.timing, res.cfo, scratch)) {
        lap(res.stage_cfo_us);
        res.stage_cfo_fit_us += res.cfo.fit_us;
        res.status = DemodStatus::CfoFailed;
        rebase();
        return res;
    }
    scratch.cached_seq.update(profile.code_index, profile.sfd_mode);
    const std::vector<int8_t>& sfd_seq = scratch.cached_seq.sfd_seq;
    const size_t sfd_len = sfd_seq.size() * kQm35SamplesPerSymbol;
    const auto sfd1_range = rxopt::compute_sfd_search_range(
        n, res.timing, profile.preamble_repetitions, sfd_len,
        kQm35SamplesPerSymbol);
    // Stage-2 CFO, kept so the single-pass rotation mode can restore it after
    // the SFD-anchored refit has been evaluated (see the refit lambda below).
    const CfoResult stage2_cfo = res.cfo;
    res.cfo_initial_hz = stage2_cfo.cfo_hz;
    res.cfo_initial_peaks = static_cast<uint32_t>(stage2_cfo.peaks_used);
    // Highest sample index that any post-SFD consumer reads from
    // `scratch.derotated`, given a frame origin.  Used twice: to predict the
    // single-pass rotation range before the bootstrap SFD and to bound the
    // actual second rotation afterwards.
    //
    // The bound is the real requirement, not the legacy 40-PHR-head budget:
    //   * the soft-chip stage only generates the prefix the active decoder
    //     needs (`fused_phr_prefix_budget` when the PHR is read from raw IQ,
    //     else the full `initial_chip_budget`),
    //   * the fused PHR kernel reads its 21 symbols straight from the
    //     derotated IQ: 20*512 + max candidate offset (320) chips plus the
    //     template length,
    //   * the final SFD search range.
    // Everything from the payload start onwards is derotated by the payload
    // pass (`pay_lo = min(sec_hi, base0)`), so covering past that point here
    // would derotate the payload region twice with the same CFO.
    auto required_derotated_hi = [&](int64_t frame_start,
                                     size_t sfd_search_hi) -> int64_t {
        const size_t pre = profile.cir_pre_samples;
        const size_t post = profile.cir_post_samples;
        const size_t tap_count = pre + post;
        const bool phr_fusion = profile.rx_fused_phr_cmf;
        const size_t chip_prefix = phr_fusion
            ? rxopt::fused_phr_prefix_budget(profile)
            : rxopt::initial_chip_budget(profile);
        const int64_t chip_start = frame_start + static_cast<int64_t>(post) - 1;
        int64_t hi = chip_start + static_cast<int64_t>(chip_prefix) * 2 +
                     static_cast<int64_t>(tap_count) + 128;
        if (phr_fusion) {
            // NS-SFD end is not known before stage_ns_sfd(); the nominal grid
            // plus the +8-chip search covers it.
            const int64_t sfd_end_chip_est =
                static_cast<int64_t>(profile.preamble_repetitions +
                                     rxopt::kSfdSymbols) *
                    static_cast<int64_t>(kQm35ChipsPerSymbol) + 8;
            const int64_t phr_start_est = sfd_end_chip_est + 1;
            const int64_t phr_len =
                static_cast<int64_t>(tap_count) +
                2 * static_cast<int64_t>(rxfusion::kPhrChipsPerBurst - 1);
            const int64_t phr_hi = chip_start +
                (phr_start_est +
                 static_cast<int64_t>((rxfusion::kPhrSymbols - 1) *
                                      rxfusion::kPhrChipsPerSymbol) +
                 static_cast<int64_t>(rxfusion::kPhrCandidateChipOffset[3])) * 2 +
                phr_len + 64;
            hi = std::max(hi, phr_hi);
        }
        return std::max<int64_t>(hi, static_cast<int64_t>(sfd_search_hi));
    };
    // Predicted single-pass rotation range.  `scratch.derotated` is the only
    // CFO-compensated buffer that the downstream stages read, so one rotation
    // with the Stage-2 estimate can replace the SFD-anchored second pass only
    // if it already covers everything those stages will read.  The SFD anchor
    // is not known yet, so the range is predicted from the Stage-1 timing with
    // an explicit symbol/backtrack slack and re-validated against the actual
    // [sec_lo, sec_hi) after the bootstrap SFD (a miss falls back to the legacy
    // second rotation).
    size_t union_lo = 0;
    size_t union_hi = 0;
    bool union_ok = false;
    if (profile.cfo_refit_mode != 0) {
        const bool u_on_demand = profile.rx_softchip_on_demand ||
                                 profile.rx_fused_payload_cmf ||
                                 profile.rx_fused_phr_cmf;
        if (!profile.rx_stop_after_sfd && !u_on_demand) {
            // The legacy path rotates the whole window, so there is nothing to
            // predict: covering [0, n) is trivially conservative and the only
            // saving is the redundant first pass.
            union_lo = 0;
            union_hi = n;
            union_ok = n > 0;
        } else {
            const size_t u_pre = profile.cir_pre_samples;
            const size_t u_first = profile.cir_skip_initial_repetitions;
            const size_t u_full = profile.preamble_repetitions;
            const size_t u_reps = (profile.cir_repetitions > 0)
                ? std::min(profile.cir_repetitions,
                           u_full > u_first ? u_full - u_first : 0)
                : 0;
            const double u_period = (res.timing.measured_period > 0.0)
                ? res.timing.measured_period
                : static_cast<double>(kQm35SamplesPerSymbol);
            const int64_t u_start = res.timing.preamble_start_sample;
            const int64_t u_cir_lo = (u_reps > 0)
                ? u_start + static_cast<int64_t>(std::llround(
                              static_cast<double>(u_first) * u_period)) -
                      static_cast<int64_t>(u_pre)
                : u_start;
            // Mirror the actual sec_lo rule: the fused PHR+payload path skips
            // the SYNC soft-chip FIR, so it does not need the preamble head.
            const bool u_skip_sync_fir =
                profile.rx_fused_phr_cmf && profile.rx_fused_payload_cmf;
            int64_t u_lo = u_cir_lo;
            if (!u_skip_sync_fir)
                u_lo = std::min<int64_t>(u_lo,
                                         u_start - static_cast<int64_t>(u_pre));
            u_lo = std::min<int64_t>(u_lo,
                                     static_cast<int64_t>(sfd1_range.first));
            int64_t u_hi = required_derotated_hi(u_start, sfd1_range.second);
            // Slack: the SFD anchor can move the frame origin by up to the
            // search backtrack (4 symbols + 64 samples).
            const int64_t u_slack =
                static_cast<int64_t>(4 * kQm35SamplesPerSymbol + 64 + 128);
            union_lo = static_cast<size_t>(std::max<int64_t>(0, u_lo - u_slack));
            union_hi = static_cast<size_t>(
                std::min<int64_t>(static_cast<int64_t>(n), u_hi + u_slack));
            union_ok = union_hi > union_lo;
        }
    }
    const size_t rot1_lo = union_ok ? union_lo : sfd1_range.first;
    const size_t rot1_hi = union_ok ? union_hi : sfd1_range.second;
    uint64_t rot1_us = 0;
    uint32_t rk1 = 0;
    derotate_cfo_range(rx, n, profile, res.cfo.cfo_hz, rot1_lo, rot1_hi,
                       scratch, rot1_us, rk1);
    res.cfo.rotate_us = rot1_us;
    res.cfo.rotation_kernel = rk1;
    res.stage_cfo_rotated_samples +=
        (rot1_hi > rot1_lo ? (rot1_hi - rot1_lo) : 0);
    ++res.stage_cfo_rotation_passes;
    lap(res.stage_cfo_us);
    res.stage_cfo_fit_us += res.cfo.fit_us;
    res.stage_cfo_rotate_us += rot1_us;
    res.stage_cfo_rotation_kernels |= (rk1 ? 1u : 0u);
    // Stage 3: bootstrap SFD timing.  Timing acquisition can start several
    // SYNCs late when a weak leading repetition falls below its tracking
    // threshold.  Its 64-point phase train would then spill into the SFD and
    // corrupt a CFO fit, even though the SFD search can recover the true frame
    // origin by testing earlier hypotheses.  Use this first SFD only to anchor
    // the preamble; the final reported CFO is re-fitted below from that anchor.
    scratch.initial_timing = res.timing;
    const std::complex<float>* sfd_rx =
        scratch.derotated.empty() ? rx : scratch.derotated.data();
    const auto sfd_bootstrap_t0 = std::chrono::steady_clock::now();
    const bool bootstrap_sfd_ok = stage_sfd(
        sfd_rx, n, profile, res.timing, sfd_seq, template_wf, res.sfd, scratch);
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
        lap(res.stage_sfd_us);
        res.status = DemodStatus::SfdFailed;
        rebase();
        return res;
    }

    // Use the SFD as the frame anchor.  The first SYNCs can be weak or can be
    // missed by acquisition, so do not re-track the whole preamble from its
    // reconstructed first repetition.  Track only the stable tail used by
    // CIR/CFO, then rebuild the complete SYNC grid from the SFD coordinate.
    const size_t full_repetitions = profile.preamble_repetitions;
    const size_t tail_repetitions = std::min(
        full_repetitions,
        std::max<size_t>(profile.cfo_min_fit_repetitions, 40));
    if (tail_repetitions < 2 || res.timing.measured_period <= 0.0) {
        lap(res.stage_sfd_us);
        res.status = DemodStatus::CfoFailed;
        rebase();
        return res;
    }

    Qm35825Profile cfo_anchor_profile = profile;
    cfo_anchor_profile.preamble_repetitions = tail_repetitions;
    // The SFD anchor supplies the absolute tail origin.  Use a local ±8 sample
    // refinement at each tail SYNC, but do not allow acquisition to jump to a
    // different symbol or walk backwards toward weak leading repetitions.
    cfo_anchor_profile.timing_search_margin = 0;
    cfo_anchor_profile.timing_coarse_stride = 1;
    cfo_anchor_profile.timing_max_backtrack_symbols = 0;
    cfo_anchor_profile.timing_track_radius = 8;
    // Opt-in: the SFD anchor predicts this tail origin to within a few samples;
    // try ±8 first and fall back to the legacy ±max(256,stride) search below.
    cfo_anchor_profile.timing_seed_refine_radius =
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
    cfo_anchor_profile.radar_verification_threshold =
        std::min(profile.radar_verification_threshold, 0.08f);

    const int64_t bootstrap_sfd_start = res.sfd.sfd_start_sample;
    const double period_hint = res.timing.measured_period;
    const int64_t tail_start = bootstrap_sfd_start -
        static_cast<int64_t>(std::llround(
            static_cast<double>(tail_repetitions) * period_hint));
    const size_t template_len = template_wf.size();

    size_t sec_derotated_hi = 0;

    // Tail re-track + CFO refit.  `seed_radius` is 0 for the legacy wide first
    // search or 8 for the SFD-anchored narrow experiment.  A failed narrow
    // search or an incomplete tail train falls back to 0 once inside.
    auto run_tail_and_cfo_refit = [&](size_t seed_radius,
                                      TimingResult& tail_timing,
                                      TimingResult& cfo_timing) -> bool {
        Qm35825Profile anchor = cfo_anchor_profile;
        anchor.timing_seed_refine_radius = seed_radius;
        const auto sfd_tail_t0 = std::chrono::steady_clock::now();

        bool tail_timing_ok = false;
        const size_t tail_first = (full_repetitions >= tail_repetitions)
            ? (full_repetitions - tail_repetitions) : 0;
        const int64_t start1 = (!scratch.initial_timing.peak_samples.empty() && template_len > 0)
            ? scratch.initial_timing.peak_samples[0] - static_cast<int64_t>(template_len - 1)
            : -1;
        const int64_t expected_tail_start = (start1 >= 0)
            ? start1 + static_cast<int64_t>(tail_first * kQm35SamplesPerSymbol)
            : -2;
        const int64_t actual_tail_first_start = (tail_first < scratch.initial_timing.peak_samples.size() && template_len > 0)
            ? scratch.initial_timing.peak_samples[tail_first] - static_cast<int64_t>(template_len - 1)
            : -3;

        bool can_reuse = (seed_radius == 8) &&
                         scratch.initial_timing.ok &&
                         (scratch.initial_timing.detected_peaks == full_repetitions) &&
                         (scratch.initial_timing.peak_samples.size() == full_repetitions) &&
                         (scratch.initial_timing.peak_metrics.size() == full_repetitions) &&
                         (scratch.initial_timing.peak_corr.size() == full_repetitions) &&
                         (template_len > 0) &&
                         (tail_repetitions > 0 && tail_repetitions <= full_repetitions) &&
                         (tail_first > 0) &&
                         (profile.timing_track_radius == 8) &&
                         (anchor.timing_track_radius == 8) &&
                         (anchor.timing_max_backtrack_symbols == 0) &&
                         (tail_start == expected_tail_start) &&
                         (actual_tail_first_start == tail_start);

        if (can_reuse) {
            for (size_t k = tail_first; k < full_repetitions; ++k) {
                const float m = scratch.initial_timing.peak_metrics[k];
                if (!std::isfinite(m) || m < anchor.radar_verification_threshold) {
                    can_reuse = false;
                    break;
                }
            }
        }

        if (can_reuse) {
            tail_timing.peak_samples.assign(
                scratch.initial_timing.peak_samples.begin() + tail_first,
                scratch.initial_timing.peak_samples.end());
            tail_timing.peak_metrics.assign(
                scratch.initial_timing.peak_metrics.begin() + tail_first,
                scratch.initial_timing.peak_metrics.end());
            tail_timing.peak_corr.assign(
                scratch.initial_timing.peak_corr.begin() + tail_first,
                scratch.initial_timing.peak_corr.end());
            tail_timing.detected_peaks = tail_repetitions;
            tail_timing.expected_peaks = tail_repetitions;
            tail_timing.metric = tail_timing.peak_metrics[0];
            tail_timing.coarse_search_us = 0;
            tail_timing.fine_track_us = 0;
            tail_timing.seed_search_us = 0;
            tail_timing.seed_refine_narrow_attempts = 0;
            tail_timing.seed_refine_narrow_success = 0;
            tail_timing.seed_refine_fallbacks = 0;

            const size_t np = tail_repetitions;
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
                    tail_timing.measured_period = (static_cast<double>(np) * sxy - sx * sy) / denom;
                    const double intercept_end = (sy - tail_timing.measured_period * sx) / np;
                    tail_timing.preamble_start_sample = std::max<int64_t>(
                        0, static_cast<int64_t>(std::llround(intercept_end)) -
                               (static_cast<int64_t>(template_len) - 1));
                } else {
                    tail_timing.measured_period = static_cast<double>(period);
                    tail_timing.preamble_start_sample = tail_start;
                }
            } else {
                tail_timing.measured_period = static_cast<double>(period);
                tail_timing.preamble_start_sample = tail_start;
            }
            tail_timing.ok = true;
            tail_timing_ok = true;
        } else {
            tail_timing_ok = stage_timing(
                rx, n, anchor, template_wf, tail_start, tail_timing, scratch);
            if ((!tail_timing_ok ||
                 tail_timing.detected_peaks < 4) &&
                seed_radius != 0) {
                const auto retry_t0 = std::chrono::steady_clock::now();
                anchor.timing_seed_refine_radius = 0;
                TimingResult retry_timing;
                const bool retry_ok = stage_timing(
                    rx, n, anchor, template_wf, tail_start, retry_timing, scratch);
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
        const bool have_tail_peaks =
            tail_timing_ok && tail_timing.detected_peaks >= 4;

        // Keep the SFD-anchored full-frame grid on the original timing
        // estimate.  Only the tail peaks have matched-filter measurements;
        // leading peaks are generated from the SFD-anchored grid.
        const double measured_period = period_hint;
        const int64_t full_start = bootstrap_sfd_start -
            static_cast<int64_t>(std::llround(
                static_cast<double>(full_repetitions) * measured_period));
        // The SFD-anchored grid replaces the seeded search result, but the
        // synchronization-strategy diagnostics describe the FIRST seeded search
        // (the only one the strategy controls), so carry them over.
        cfo_timing.bounded_attempts = res.timing.bounded_attempts;
        cfo_timing.bounded_success = res.timing.bounded_success;
        cfo_timing.bounded_failures = res.timing.bounded_failures;
        cfo_timing.bounded_legacy_scans = res.timing.bounded_legacy_scans;
        cfo_timing.bounded_inner_used = res.timing.bounded_inner_used;
        cfo_timing.bounded_roi_samples = res.timing.bounded_roi_samples;
        cfo_timing.backtrack_symbols = res.timing.backtrack_symbols;
        // Diagnostic-only carry-over (no interval math): the locked-tracking
        // diagnostics describe the FIRST seeded search, which is the only one
        // the strategy controls, and res.timing is replaced below.
        cfo_timing.grid_anchor_attempts = res.timing.grid_anchor_attempts;
        cfo_timing.grid_anchor_hits = res.timing.grid_anchor_hits;
        cfo_timing.grid_single_points = res.timing.grid_single_points;
        cfo_timing.grid_single_valid = res.timing.grid_single_valid;
        cfo_timing.grid_fallbacks = res.timing.grid_fallbacks;
        cfo_timing.tail_probe_attempts = res.timing.tail_probe_attempts;
        cfo_timing.tail_probe_hits = res.timing.tail_probe_hits;
        cfo_timing.tail_probe_k = res.timing.tail_probe_k;
        cfo_timing.tail_probe_best_metric = res.timing.tail_probe_best_metric;
        cfo_timing.tail_probe_raw_max = res.timing.tail_probe_raw_max;
        cfo_timing.inner_probe_raw_max = res.timing.inner_probe_raw_max;
        cfo_timing.initial_coarse_search_us = res.timing.coarse_search_us;
        cfo_timing.initial_fine_track_us = res.timing.fine_track_us;
        cfo_timing.ok = true;
        cfo_timing.preamble_start_sample = full_start;
        cfo_timing.preamble_start_uncropped = full_start;
        cfo_timing.measured_period = measured_period;
        cfo_timing.metric =
            have_tail_peaks ? tail_timing.metric : res.timing.metric;
        cfo_timing.detected_peaks = full_repetitions;
        cfo_timing.expected_peaks = full_repetitions;
        cfo_timing.coarse_search_us = tail_timing.coarse_search_us;
        cfo_timing.fine_track_us = tail_timing.fine_track_us;
        cfo_timing.seed_search_us = tail_timing.seed_search_us;
        cfo_timing.seed_refine_narrow_attempts =
            tail_timing.seed_refine_narrow_attempts;
        cfo_timing.seed_refine_narrow_success =
            tail_timing.seed_refine_narrow_success;
        cfo_timing.seed_refine_fallbacks = tail_timing.seed_refine_fallbacks;
        cfo_timing.peak_samples.resize(full_repetitions);
        cfo_timing.peak_metrics.assign(full_repetitions, 0.0f);
        cfo_timing.peak_corr.assign(
            full_repetitions, std::complex<float>(0.0f, 0.0f));
        for (size_t k = 0; k < full_repetitions; ++k) {
            cfo_timing.peak_samples[k] = full_start +
                static_cast<int64_t>(std::llround(
                    static_cast<double>(k) * measured_period)) +
                static_cast<int64_t>(template_len - 1);
        }
        if (have_tail_peaks) {
            if (tail_timing.peak_samples.size() == tail_repetitions) {
                for (size_t k = 0; k < tail_repetitions; ++k) {
                    const size_t dst = tail_first + k;
                    cfo_timing.peak_samples[dst] = tail_timing.peak_samples[k];
                    cfo_timing.peak_metrics[dst] = tail_timing.peak_metrics[k];
                    cfo_timing.peak_corr[dst] = tail_timing.peak_corr[k];
                }
            } else {
                const int64_t tail_pk0 =
                    tail_start + static_cast<int64_t>(template_len - 1);
                for (size_t i = 0; i < tail_timing.peak_samples.size(); ++i) {
                    const int64_t rel_sym = std::llround(
                        static_cast<double>(tail_timing.peak_samples[i] - tail_pk0) /
                        measured_period);
                    if (rel_sym >= 0 &&
                        static_cast<size_t>(rel_sym) < tail_repetitions) {
                        const size_t dst =
                            tail_first + static_cast<size_t>(rel_sym);
                        cfo_timing.peak_samples[dst] =
                            tail_timing.peak_samples[i];
                        cfo_timing.peak_metrics[dst] =
                            tail_timing.peak_metrics[i];
                        cfo_timing.peak_corr[dst] = tail_timing.peak_corr[i];
                    }
                }
            }
        }

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
                stage2_cfo.peaks_used < 16;
            if (refit_well_conditioned) {
                Qm35825Profile tail_fit_profile = profile;
                tail_fit_profile.cfo_skip_initial_repetitions = 0;
                CfoResult tail_cfo;
                const bool cfo_fit_ok =
                    fit_cfo(rx, n, tail_fit_profile, tail_timing, tail_cfo, scratch);
                if (cfo_fit_ok) {
                    res.cfo = tail_cfo;
                    res.cfo.skipped_peaks += tail_first;
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
        const size_t sym = kQm35SamplesPerSymbol;
        const auto final_sfd_range = rxopt::compute_sfd_search_range(
            n, cfo_timing, full_repetitions, sfd_len, sym);
        const size_t pre = profile.cir_pre_samples;
        const size_t post = profile.cir_post_samples;
        const size_t first_rep = profile.cir_skip_initial_repetitions;
        const size_t rep_count = (profile.cir_repetitions > 0)
            ? std::min(profile.cir_repetitions,
                       full_repetitions > first_rep
                           ? full_repetitions - first_rep
                           : 0)
            : 0;
        const double period = cfo_timing.measured_period > 0.0
            ? cfo_timing.measured_period
            : static_cast<double>(sym);
        const int64_t start = cfo_timing.preamble_start_sample;
        const int64_t cir_lo = (rep_count > 0)
            ? start + static_cast<int64_t>(std::llround(
                          static_cast<double>(first_rep) * period)) -
                  static_cast<int64_t>(pre)
            : start;

        const bool on_demand = profile.rx_softchip_on_demand ||
                               profile.rx_fused_payload_cmf ||
                               profile.rx_fused_phr_cmf;

        const double nominal_spc = profile.sample_rate / 499.2e6;
        const bool is_integer_grid = (std::abs(nominal_spc - 2.0) < 1e-6);

        size_t sec_lo = 0;
        size_t sec_hi = n;
        if (profile.rx_stop_after_sfd) {
            sec_lo = static_cast<size_t>(std::max<int64_t>(0, static_cast<int64_t>(final_sfd_range.first)));
            sec_hi = static_cast<size_t>(std::min<int64_t>(static_cast<int64_t>(n), static_cast<int64_t>(final_sfd_range.second)));
        } else if (on_demand && is_integer_grid && integer_grid_ok) {
            // Second rotation covers the final SFD search, the CIR window, the
            // phase estimation and the soft-chip prefix / fused PHR reads.
            // `required_derotated_hi` bounds the real requirement (see its
            // definition above): using the legacy 40-PHR-head budget here
            // derotated ~35k samples past the payload start, which the payload
            // pass then derotated a second time with the same CFO.
            const bool skip_sync_fir =
                profile.rx_fused_phr_cmf && profile.rx_fused_payload_cmf;

            int64_t req_lo = cir_lo;
            if (!skip_sync_fir)
                req_lo = std::min(req_lo, start - static_cast<int64_t>(pre));
            req_lo = std::min(req_lo, static_cast<int64_t>(final_sfd_range.first));
            sec_lo = static_cast<size_t>(std::max<int64_t>(0, req_lo));

            const int64_t req_hi =
                required_derotated_hi(start, final_sfd_range.second);
            sec_hi = static_cast<size_t>(
                std::min<int64_t>(static_cast<int64_t>(n), req_hi));
        }

        uint64_t rot2_us = 0;
        uint32_t rk2 = 0;
        // ---- Single-pass rotation decision ---------------------------------
        // The second rotation exists only because the SFD-anchored refit moves
        // the estimate.  When the Stage-2 estimate is good enough, the pass-1
        // rotation (already widened to the predicted union range) is the only
        // derotation the packet needs.  Every condition below is a hard
        // requirement; any miss keeps the legacy refit + second rotation.
        const uint64_t refit_span =
            (sec_hi > sec_lo) ? static_cast<uint64_t>(sec_hi - sec_lo) : 0;
        const double refit_delta_hz = res.cfo.cfo_hz - stage2_cfo.cfo_hz;
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
                stage2_cfo.peaks_used >= profile.cfo_refit_min_peaks &&
                refit_drift_rad <= profile.cfo_refit_skip_rad;
            skip_action = 1;
        }
        const bool union_covers =
            union_ok && sec_lo >= union_lo && sec_hi <= union_hi;
        if (want_skip && union_covers) {
            // Keep the Stage-2 estimate: pass 1 already applied it to the whole
            // predicted span, which provably contains [sec_lo, sec_hi).
            res.cfo = stage2_cfo;
            res.cfo_refit_action = skip_action;
            sec_derotated_hi = union_hi;
            return true;
        }
        if (profile.cfo_refit_mode != 0)
            res.cfo_refit_action = 3; // requested skip, fell back (fail-closed)
        derotate_cfo_range(rx, n, profile, res.cfo.cfo_hz, sec_lo, sec_hi,
                           scratch, rot2_us, rk2);
        res.cfo.rotate_us = rot2_us;
        res.cfo.rotation_kernel = rk2;
        res.cfo_refit_rotate_us = rot2_us;
        res.stage_cfo_rotate_us += rot2_us;
        res.stage_cfo_rotation_kernels |= (rk2 ? 2u : 0u);
        res.stage_cfo_rotated_samples += (sec_hi > sec_lo ? (sec_hi - sec_lo) : 0);
        ++res.stage_cfo_rotation_passes;
        res.stage_sfd_cfo_refit_us += rot2_us;
        sec_derotated_hi = sec_hi;
        return true;
    };

    // Final full-rate SFD verification on the current CFO compensation.
    auto run_final_sfd = [&]() -> bool {
        sfd_rx = scratch.derotated.empty() ? rx : scratch.derotated.data();
        const auto sfd_final_t0 = std::chrono::steady_clock::now();
        const bool ok = stage_sfd(
            sfd_rx, n, profile, res.timing, sfd_seq, template_wf,
            res.sfd, scratch);
        res.stage_sfd_final_us += static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::steady_clock::now() - sfd_final_t0).count());
        return ok;
    };

    TimingResult tail_timing;
    TimingResult cfo_timing;
    const size_t first_seed_radius = profile.rx_seed_refine_tail ? 8 : 0;
    if (!run_tail_and_cfo_refit(first_seed_radius, tail_timing, cfo_timing)) {
        lap(res.stage_sfd_us);
        res.status = DemodStatus::CfoFailed;
        rebase();
        return res;
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
        if (run_tail_and_cfo_refit(0, retry_tail, retry_cfo)) {
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
        lap(res.stage_sfd_us);
        res.status = DemodStatus::SfdFailed;
        rebase();
        return res;
    }
    res.timing.preamble_start_sample =
        res.sfd.sfd_start_sample - static_cast<int64_t>(std::llround(
            static_cast<double>(profile.preamble_repetitions) *
            res.timing.measured_period));
    lap(res.stage_sfd_us);
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

    // Early termination if caller only requested preamble (SYNC + SFD) detection.
    if (profile.rx_stop_after_sfd) {
        res.status = DemodStatus::Success;
        rebase();
        res.stage_total_us = res.stage_timing_us + res.stage_cfo_us + res.stage_sfd_us;
        return res;
    }

    // Stage 4: CIR + soft chips on the CFO-compensated frame.
    const std::vector<int8_t>& pcode = scratch.cached_seq.pcode;
    const std::vector<int8_t>& nsfd_seq = scratch.cached_seq.sfd_seq;
    const bool on_demand_path = integer_grid_ok &&
                                (profile.rx_softchip_on_demand ||
                                 profile.rx_fused_payload_cmf ||
                                 profile.rx_fused_phr_cmf);
    if (on_demand_path) {
        // Experimental two-phase path: generate only the prefix required by
        // the active decoder.  Payload fusion needs SYNC/SFD/PHR soft chips;
        // PHR fusion stops at the NS-SFD end because PHR is read from raw IQ.
        // Any unsupported grid or failure retries the complete legacy path once
        // and records the fallback.  No injected/sidecar length is used.
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
        lap(res.stage_cir_us);
        const size_t prefix_chips = res.cir.soft_chip_count;
        if (ok) {
            ok = stage_ns_sfd(scratch.soft_chips, profile, nsfd_seq,
                              kQm35ChipsPerSymbol, res.ns_sfd, scratch);
            lap(res.stage_ns_sfd_us);
        }
        bool phr_fused_used = false;
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
                    lap(res.stage_phr_us);
                    // A SECDED correction needs the full-stream tail-energy
                    // guard.  The reduced prefix cannot reproduce it, so keep
                    // the existing conservative full-stream retry.
                    if (fused_ok && res.phr.ok &&
                        !res.phr.secded_corrected) {
                        phr_fused_used = true;
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
                lap(res.stage_phr_us);
            }
        }
        if (ok && !phr_fused_used && res.phr.secded_corrected) {
            // The legacy PHR path can inspect the full-range tail energy when
            // SECDED corrected a longer length.  The prefix cannot, so retry
            // the complete stream rather than silently changing that decision.
            ok = false;
        }
        // Experimental fused payload path: after the validated PHR, produce the
        // four BPM metrics directly from the CFO-compensated IQ instead of
        // extending the soft-chip stream to the payload range.  Only the full
        // integer-grid CMF is supported; fractional SFO, RAKE and bypass keep
        // the legacy on-demand extension.  Any failure retries the complete
        // legacy stream once through the existing fallback block below.
        if (ok && profile.rx_fused_payload_cmf && integer_grid_ok) {
            const bool fused_supported =
                !res.cir.cir_filter_taps.empty() &&
                integer_chip_grid_at(res.cir.samples_per_chip,
                                     rxfusion::kSamplesPerChip,
                                     profile.fused_gate_ppm * 1e-6) &&
                profile.cir_rake_top_k == 0 &&
                profile.cir_soft_chip_mode != CirSoftChipMode::Rake &&
                profile.cir_soft_chip_mode != CirSoftChipMode::Bypass &&
                scratch.softchip.prepared;
            if (fused_supported) {
                // Compensate payload interval with refit CFO (Task A1)
                const size_t psdu_bits =
                    static_cast<size_t>(res.phr.psdu_length) * 8;
                const size_t num_blocks = (psdu_bits + 329) / 330;
                const size_t nsym = psdu_bits + 48 * num_blocks;
                const int64_t phr_start = res.ns_sfd.sfd_end_chip + 1;
                const int64_t payload_start = phr_start + 21 * 512;
                const int64_t chip_start =
                    res.timing.preamble_start_sample +
                    static_cast<int64_t>(res.cir.cir_filter_taps.size()) -
                    static_cast<int64_t>(profile.cir_pre_samples) - 1;
                const int64_t base0 = chip_start + payload_start * 2 -
                    static_cast<int64_t>(res.cir.cir_filter_taps.size()) + 1;
                const int64_t last_chip =
                    static_cast<int64_t>(nsym - 1) * 64 + 40;
                const size_t len =
                    res.cir.cir_filter_taps.size() + 2 * (8 - 1);
                const int64_t last_sample =
                    base0 + last_chip * 2 + static_cast<int64_t>(len);
                const size_t pay_lo = std::min(
                    sec_derotated_hi,
                    static_cast<size_t>(std::max<int64_t>(0, base0)));
                const size_t pay_hi = static_cast<size_t>(
                    std::min<int64_t>(static_cast<int64_t>(n), last_sample + 64));
                if (pay_lo < pay_hi) {
                    // This payload-region CFO derotation runs inside the
                    // `stage_payload_us` lap but outside the payload decoder, so
                    // attribute it to the payload `other` bucket explicitly;
                    // otherwise the child sum would fall short of the parent by
                    // the rotation cost (~36 µs on the golden window).
                    const auto t_payrot = std::chrono::steady_clock::now();
                    uint64_t rot_pay_us = 0;
                    uint32_t rk_pay = 0;
                    derotate_cfo_range(rx, n, profile, res.cfo.cfo_hz, pay_lo,
                                       pay_hi, scratch, rot_pay_us, rk_pay);
                    res.stage_cfo_rotate_us += rot_pay_us;
                    res.stage_cfo_rotated_samples += (pay_hi - pay_lo);
                    ++res.stage_cfo_rotation_passes;
                    sec_derotated_hi = std::max(sec_derotated_hi, pay_hi);
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
                lap(res.stage_payload_us);
                if (fused_ok && res.payload.fcs_pass) {
                    res.status = DemodStatus::Success;
                    maybe_diagnose_fir();
                    rebase();
                    return res;
                }
                ++res.stage_fused_fallbacks;
                ok = false;
            }
        }
        if (ok) {
            const size_t final_budget = rxopt::final_chip_budget(
                profile, res.ns_sfd.sfd_end_chip, res.phr.psdu_length);
            if (final_budget > prefix_chips) {
                // Compensate extension interval with refit CFO (Task A1)
                const int64_t chip_start =
                    res.timing.preamble_start_sample +
                    static_cast<int64_t>(res.cir.cir_filter_taps.size()) -
                    static_cast<int64_t>(profile.cir_pre_samples) - 1;
                const int64_t last_pos =
                    chip_start + static_cast<int64_t>(final_budget) * 2 +
                    static_cast<int64_t>(res.cir.cir_filter_taps.size()) + 64;
                const size_t pay_lo = sec_derotated_hi;
                const size_t pay_hi = static_cast<size_t>(
                    std::min<int64_t>(static_cast<int64_t>(n), last_pos));
                if (pay_lo < pay_hi) {
                    uint64_t rot_pay_us = 0;
                    uint32_t rk_pay = 0;
                    derotate_cfo_range(rx, n, profile, res.cfo.cfo_hz, pay_lo,
                                       pay_hi, scratch, rot_pay_us, rk_pay);
                    res.stage_cfo_rotate_us += rot_pay_us;
                    res.stage_cfo_rotated_samples += (pay_hi - pay_lo);
                    ++res.stage_cfo_rotation_passes;
                    sec_derotated_hi = std::max(sec_derotated_hi, pay_hi);
                }

                CirResult ext;
                const bool ext_ok = stage_cir_softchips(
                    scratch.derotated.data(), n, eff, res.timing, pcode,
                    ext, scratch, final_budget, prefix_chips);
                add_elapsed(res.stage_cir_us);
                if (ext_ok) {
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
                } else {
                    ok = false;
                }
            }
        }
        if (ok) {
            ok = stage_payload_fcs(scratch.soft_chips, profile, res.phr,
                                   res.ns_sfd, kQm35ChipsPerSymbol,
                                   res.payload, scratch, &payload_sub);
            lap(res.stage_payload_us);
            if (ok && !res.payload.fcs_pass)
                ok = false;
        }
        if (!ok) {
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
            sec_derotated_hi = n;
            CirResult full_cir;
            NsSfdResult full_ns;
            PhrResult full_phr;
            PayloadResult full_payload;
            const bool full_cir_ok = stage_cir_softchips(
                scratch.derotated.data(), n, profile, res.timing, pcode,
                full_cir, scratch);
            const bool full_ns_ok =
                full_cir_ok &&
                stage_ns_sfd(scratch.soft_chips, profile, nsfd_seq,
                             kQm35ChipsPerSymbol, full_ns, scratch);
            const bool full_phr_ok =
                full_ns_ok &&
                stage_phr(scratch.soft_chips, profile, full_ns,
                          kQm35ChipsPerSymbol, full_phr, scratch);
            const bool full_payload_ok =
                full_phr_ok &&
                stage_payload_fcs(scratch.soft_chips, profile, full_phr,
                                  full_ns, kQm35ChipsPerSymbol, full_payload,
                                  scratch);
            res.stage_softchip_fallback_us = static_cast<uint64_t>(
                std::chrono::duration_cast<std::chrono::microseconds>(
                    std::chrono::steady_clock::now() - fallback_t0).count());
            res.stage_total_us += res.stage_softchip_fallback_us;
            t_prev = std::chrono::steady_clock::now();
            // Publish the fallback attempt even when it fails: the status
            // below is decided by this legacy run, so the reported CIR /
            // NS-SFD / PHR / payload must describe the same run.  Otherwise a
            // short prefix result is paired with a full-stream failure status.
            res.cir = std::move(full_cir);
            res.cir.on_demand_fallbacks = 1;
            res.ns_sfd = std::move(full_ns);
            res.phr = std::move(full_phr);
            res.payload = std::move(full_payload);
            if (full_cir_ok && full_ns_ok && full_phr_ok && full_payload_ok) {
                res.status = (res.payload.ok && res.payload.fcs_pass)
                                 ? DemodStatus::Success
                                 : DemodStatus::FcsFailed;
                maybe_diagnose_fir();
                rebase();
                return res;
            }
            if (!full_cir_ok)
                res.status = DemodStatus::CirFailed;
            else if (!full_ns_ok)
                res.status = DemodStatus::SfdFailed;
            else if (!full_phr_ok)
                res.status = DemodStatus::PhrFailed;
            else
                res.status = DemodStatus::PayloadFailed;
            rebase();
            return res;
        }
        res.status = (res.payload.ok && res.payload.fcs_pass)
                         ? DemodStatus::Success
                         : DemodStatus::FcsFailed;
        maybe_diagnose_fir();
        rebase();
        return res;
    }
    // Legacy full-stream path (also the degraded path when the integer grid was
    // refused).  Uses `eff` so the Full->Bypass downgrade applies here too.
    if (!stage_cir_softchips(scratch.derotated.data(), n, eff, res.timing,
                             pcode, res.cir, scratch)) {
        lap(res.stage_cir_us);
        res.status = DemodStatus::CirFailed;
        rebase();
        return res;
    }
    lap(res.stage_cir_us);
    // Stage 5: NS-SFD location in the soft-chip stream.
    if (!stage_ns_sfd(scratch.soft_chips, profile, nsfd_seq, kQm35ChipsPerSymbol,
                      res.ns_sfd, scratch)) {
        lap(res.stage_ns_sfd_us);
        res.status = DemodStatus::SfdFailed;
        rebase();
        return res;
    }
    lap(res.stage_ns_sfd_us);
    // Stage 6: BPRF PHR demod + conv decode + SECDED.
    if (!stage_phr(scratch.soft_chips, profile, res.ns_sfd, kQm35ChipsPerSymbol,
                   res.phr, scratch)) {
        lap(res.stage_phr_us);
        res.status = DemodStatus::PhrFailed;
        rebase();
        return res;
    }
    lap(res.stage_phr_us);
    // Stage 7: payload BPM-BPSK + RS + FCS.
    if (!stage_payload_fcs(scratch.soft_chips, profile, res.phr, res.ns_sfd,
                           kQm35ChipsPerSymbol, res.payload, scratch,
                           &payload_sub)) {
        lap(res.stage_payload_us);
        res.status = DemodStatus::PayloadFailed;
        rebase();
        return res;
    }
    lap(res.stage_payload_us);
    res.status = (res.payload.ok && res.payload.fcs_pass)
                     ? DemodStatus::Success
                     : DemodStatus::FcsFailed;
    maybe_diagnose_fir();
    rebase();
    return res;
}


} // namespace gr::uwb::demod::core
