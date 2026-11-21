// SPDX-License-Identifier: GPL-3.0-or-later
//
// Out-of-line definitions for the HRP timing stage.  The declarations stay in
// the public headers (demod_timing.h / demod_sync.h / rx_sync_opt.h /
// rx_matched_filter.h); this translation unit hosts the coarse stage entry
// point, the low-SNR fold acquisition helpers, the SFD template caches and the
// matched-filter scan loops.
#include <openuwb/phy/hrp/rx/demod_timing.h>
#include <openuwb/phy/hrp/rx/demod_sync.h>
#include <openuwb/phy/hrp/rx/rx_matched_filter.h>
#include <openuwb/phy/hrp/rx/rx_sync_opt.h>

namespace openuwb::demod::core {

bool stage_timing(const std::complex<float>* rx,
                  size_t n,
                  const Qm35825Profile& profile,
                  const std::vector<std::complex<float>>& template_wf,
                  int64_t seed_start, // absolute predicted start, or -1
                  TimingResult& out,
                  DemodScratch& scratch)
{
    out = TimingResult{};
    const auto timing_begin = std::chrono::steady_clock::now();
    auto finish_timing_profile = [&]() {
        const uint64_t total = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::steady_clock::now() - timing_begin).count());
        out.fine_track_us =
            (total >= out.coarse_search_us) ? total - out.coarse_search_us : 0;
    };
    const size_t L = template_wf.size();
    if (L == 0 || n < L) {
        finish_timing_profile();
        return false;
    }

    // The sparse path never reads the dense template, so building it would be
    // pure waste (copy + L2 normalize + duplicate real taps on every call).
    // Only the dense path materializes the normalized template, and the real
    // duplicate is reused when its content key matches.
    const auto* sparse_tmpl = detail::prepare_sparse_sync_template(profile, L, scratch);
    // MF-4: factorize the dense template into (r * code) * p.  This replaces
    // the dense matched filter (same metric scale, same lag convention, same
    // pulse-matched gain) at a fraction of its cost; it does not change the
    // sparse path's peak-sample convention.  The builder rejects any template
    // that is not code*p, so a custom waveform silently keeps its old path.
    // The code template is needed even when the sparse timing path is off.
    const rxopt::ShapedSparseTemplate* shaped_tmpl = nullptr;
    if (profile.rx_timing_shaped_sparse && L == 2 * kQm35ChipsPerSymbol) {
        const rxopt::SparseCirTemplate* code_tmpl = sparse_tmpl;
        if (!code_tmpl) {
            if (scratch.cached_seq.code_index != profile.code_index ||
                scratch.cached_seq.pcode.empty())
                scratch.cached_seq.update(profile.code_index, profile.sfd_mode);
            const auto& pcode = scratch.cached_seq.pcode;
            rxopt::ensure_sparse_cir_template(pcode.data(), pcode.size(),
                                              scratch.cir_template);
            if (scratch.cir_template.code_energy > 0.0f)
                code_tmpl = &scratch.cir_template;
        }
        if (code_tmpl)
            shaped_tmpl = rxopt::ensure_shaped_sparse_template(
                template_wf.data(), L, *code_tmpl,
                scratch.shaped_sync_template);
    }
    float dense_Et = 0.0f;
    if (!sparse_tmpl && !shaped_tmpl) {
        scratch.work.assign(template_wf.begin(), template_wf.end());
        openuwb::core::uwb_l2_normalize(scratch.work);
        dense_Et = openuwb::core::uwb_template_energy(scratch.work);
        rxopt::ensure_real_sync_template(scratch.work,
                                         scratch.real_sync_template);
    }
    const float Et = shaped_tmpl
        ? shaped_tmpl->energy
        : (sparse_tmpl ? sparse_tmpl->code_energy : dense_Et);
    const auto& real = scratch.real_sync_template;
    const rxopt::SyncMatchedFilter filter(
        scratch.work.data(), L, real.is_real ? real.taps_dup.data() : nullptr,
        sparse_tmpl, true, shaped_tmpl);

    // Determine the ROI around the seed (absolute coordinates).
    size_t roi_start = 0;
    size_t roi_end = n;
    if (seed_start >= 0) {
        const int64_t seed = seed_start;
        const int64_t margin = static_cast<int64_t>(profile.timing_search_margin);
        const int64_t lo = std::max<int64_t>(0, seed - margin);
        const int64_t hi =
            std::min<int64_t>(static_cast<int64_t>(n), seed + margin + (int64_t)L);
        roi_start = static_cast<size_t>(lo);
        roi_end = static_cast<size_t>(hi);
    }

    // Local normalized-correlation helper over a small window around `want`:
    // finds the SYNC START that best matches the template, returns position,
    // metric, and the complex matched-filter value at the best position (the
    // CFO stage uses its phase).
    bool raw_par_hit = false;
    auto local_best = [&](int64_t want, int64_t radius, float min_metric,
                          int64_t* pos, float* met,
                          std::complex<float>* corr_out,
                          bool allow_par_fallback = false,
                          float* raw_max_out = nullptr) -> bool {
        const int64_t w0 = std::max<int64_t>(0, want - radius);
        const int64_t w1 =
            std::min<int64_t>(static_cast<int64_t>(n) - static_cast<int64_t>(L),
                              want + radius);
        if (w1 <= w0)
            return false;
        const int64_t count = w1 - w0;
        const bool track_par =
            allow_par_fallback && count >= 64 && count <= 1024;
        constexpr float kSyncParFloor = 0.06f;
        constexpr float kSyncParMin = 20.0f;
        constexpr int64_t kSyncParGuard = 12;
        float lag_metrics[1024];
        if (track_par && profile.rx_sync_par_only) {
            const auto peak = rxopt::scan_raw_sync(
                filter, rx + w0, static_cast<size_t>(count), lag_metrics);
            const bool accepted = rxopt::raw_lag_par_passes(
                lag_metrics, static_cast<size_t>(count), peak, kSyncParGuard, 1,
                kSyncParMin, profile.rx_sync_par_three_samples);
            const int64_t bp = w0 + static_cast<int64_t>(peak.lag);
            // Keep the normalized metric for downstream tracking and the
            // locked probe-width heuristic. Only score the winner.
            float normalized = 0.0f;
            if (accepted || raw_max_out)
                normalized = detail::normalized_correlation_power(
                    peak.corr, window_power_8acc(rx + bp, L), Et);
            if (raw_max_out)
                *raw_max_out = normalized;
            if (!accepted)
                return false;
            raw_par_hit = true;
            *pos = bp;
            *met = normalized;
            *corr_out = peak.corr;
            return true;
        }
        const auto peak = rxopt::scan_normalized_sync(
            filter, rx + w0, static_cast<size_t>(count),
            window_power_8acc(rx + w0, L), Et, track_par ? lag_metrics : nullptr);
        const float bm = peak.metric;
        const int64_t bp = w0 + static_cast<int64_t>(peak.lag);
        // Report the window's raw maximum before the acceptance gate: the
        // low-SNR early-stop decisions below need to know how far the best
        // candidate was from the threshold, not just whether it passed.
        if (raw_max_out)
            *raw_max_out = bm;
        if (bm < min_metric) {
            bool par_accept = false;
            if (track_par && bm >= kSyncParFloor) {
                double bg_sum = 0.0;
                int64_t bg_cnt = 0;
                const int64_t bp_idx = bp - w0;
                for (int64_t idx = 0; idx < count; ++idx) {
                    if (std::abs(idx - bp_idx) > kSyncParGuard) {
                        bg_sum += static_cast<double>(lag_metrics[idx]);
                        ++bg_cnt;
                    }
                }
                if (bg_cnt > 0) {
                    const double bg_mean =
                        std::max(bg_sum / static_cast<double>(bg_cnt), 1e-9);
                    par_accept =
                        (static_cast<double>(bm) / bg_mean) >= kSyncParMin;
                }
            }
            if (!par_accept)
                return false;
        }
        *pos = bp;
        *met = bm;
        *corr_out = peak.corr;
        return true;
    };

    // Locate the first SYNC start near the seed.  MATLAB detectSeededPreamble
    // searches only max(64, ceil(symbolLength/32)) samples around the seed at
    // full rate and tracks from the strongest hit there; the wide stride-S
    // coarse scan is retained only as a fallback when the seed-narrow search
    // fails (a seed that lands a whole SYNC or more off the true start).
    int64_t first_start = -1;
    size_t first_hit_k = 0;
    float first_metric = 0.0f;
    std::complex<float> first_corr(0.0f, 0.0f);
    const float thr = profile.radar_verification_threshold;
    const bool bounded = profile.sync_strategy == SyncStrategy::LockedBounded;

    // ---- On-demand K-symbol periodic-fold acquisition ---------------------
    // Reached ONLY after every single-symbol path failed.  The fold costs
    // ~1 ms (fold + full-window correlation), so it must never run on a packet
    // the normal path already acquired.  Both strategies share this helper;
    // only the search window differs (bounded ROI vs a capped acquisition
    // margin), and the fold keeps the absolute position because it folds over
    // the window rather than into one symbol period.
    bool fold_used = false;
    // Non-bounded acquisition seed, hoisted so the post-train fold retry below
    // can reuse the same window.
    int64_t acq_seed = seed_start;
    auto try_fold_acquire = [&](int64_t centre, int64_t radius) -> bool {
        const auto phase_probe = detail::probe_fold_phase(
            rx, n, centre, profile.preamble_repetitions);
        const double coarse_phase = phase_probe.phase();
        auto acquire_fold = [&](double phase) {
            return detail::fold_acquire(
                rx, n, centre, centre - radius, centre + radius,
                scratch.work.data(), L, Et, scratch.real_sync_template,
                detail::kFoldMaxSymbols, profile.preamble_repetitions,
                detail::kFoldThreshold, phase,
                sparse_tmpl, &scratch.fold_buf, shaped_tmpl);
        };
        // Check energy only when the coarse phase can change the fold choice.
        // A rejected large phase also rules out the phase-aligned retry.
        const bool could_alias = std::abs(coarse_phase) >=
            M_PI / detail::kFoldAnchorGroupSymbols;
        double fold_phase = could_alias && phase_probe.coherent()
            ? coarse_phase : 0.0;
        auto fold = acquire_fold(fold_phase);
        if (!fold.ok && !could_alias &&
            std::abs(coarse_phase) >= M_PI / (2 * detail::kFoldMaxSymbols) &&
            phase_probe.coherent()) {
            fold_phase = coarse_phase;
            fold = acquire_fold(fold_phase);
        }
        if (!fold.ok)
            return false;
        // ---- Absolute-symbol disambiguation ------------------------------
        // The preamble is 256 VERBATIM copies of one SYNC symbol, so the fold
        // metric is exactly P-periodic: `fold.start` is only the grid phase
        // nearest the seed, and the live frontend seed is accurate to a few
        // SYMBOLS (measured -6.5..+4.4 SYNC periods at -14 dB in-band), not a
        // few samples.  The one non-periodic feature available before the SFD
        // is the preamble's leading edge: immediately before the true start
        // there is noise (every acquisition window carries a pre-guard), while
        // every other candidate has a SYNC before it.  Score each candidate by
        // metric(c) - metric(c-P) and take the best.  At -14 dB in-band the
        // single-SYNC metric is ~0.038 against ~0.001 in noise.
        int64_t fold_start = fold.start;
        {
            const int64_t P = static_cast<int64_t>(kQm35SamplesPerSymbol);
            const int64_t max_j = radius / P + 2;
            auto ss_metric = [&](int64_t pos) -> float {
                if (pos < 0 ||
                    pos + static_cast<int64_t>(L) > static_cast<int64_t>(n))
                    return -1.0f;
                const float pw = window_power_8acc(rx + pos, L);
                const auto acc = filter.dot(rx + pos);
                return detail::normalized_correlation_power(acc, pw, Et);
            };
            // One pass over the candidate grid: the same metrics are needed
            // twice (once for the strongest slot, once for the first crossing
            // of the leading-edge threshold), and each is a full single-SYMBOL
            // correlation.  Caching them halves this cost.
            const size_t ncand = static_cast<size_t>(2 * max_j + 1);
            std::vector<float> cand_metric(ncand, -1.0f);
            float max_here = 0.0f;
            for (size_t i = 0; i < ncand; ++i) {
                const int64_t j = static_cast<int64_t>(i) -
                                  static_cast<int64_t>(max_j);
                const float m = ss_metric(fold.start + j * P);
                cand_metric[i] = m;
                if (m > 0.0f)
                    max_here = std::max(max_here, m);
            }
            // Leading-edge detection: scan the candidate grid from EARLY to
            // LATE and take the first slot whose single-SYNC metric clears a
            // noise/signal threshold.  Every slot before the true start is
            // noise (~0.001 at -14 dB) and every slot after it is a SYNC
            // (~0.038), so the first crossing IS the packet start.  The
            // threshold is relative to the strongest slot seen (a noise-only
            // window would otherwise never cross) and floored at 0.003, which
            // is ~3x the largest of 21 exponential noise metrics (1/L * ln21).
            const float edge_thr = std::max(0.003f, 0.2f * max_here);
            for (size_t i = 0; i < ncand; ++i) {
                const float here = cand_metric[i];
                if (here < 0.0f)
                    continue;
                if (here >= edge_thr) {
                    const int64_t j = static_cast<int64_t>(i) -
                                      static_cast<int64_t>(max_j);
                    fold_start = fold.start + j * P;
                    break;
                }
            }
        }
        // Period/origin from the 16-SYMBOL super-anchor fit.  A degenerate fit
        // still leaves a usable origin (the refined fold argmax) with the
        // nominal period, so the packet is not thrown away for a fit failure.
        double period_fit = static_cast<double>(kQm35SamplesPerSymbol);
        int64_t origin_fit = fold_start;
        const size_t groups = std::min<size_t>(
            detail::kFoldAnchorGroupSymbols,
            profile.preamble_repetitions / detail::kFoldAnchorGroupSymbols);
        if (groups >= 2) {
            if (!detail::fold_anchor_fit(
                    rx, n, fold_start, scratch.work.data(), L, Et,
                    scratch.real_sync_template, groups, &period_fit,
                    &origin_fit, out.peak_samples, out.peak_metrics,
                    out.peak_corr, fold_phase,
                    sparse_tmpl, &scratch.fold_sup_buf, shaped_tmpl)) {
                out.peak_samples.clear();
                out.peak_metrics.clear();
                out.peak_corr.clear();
            }
        }
        if (out.peak_samples.empty()) {
            out.peak_samples.push_back(fold_start +
                                       static_cast<int64_t>(L) - 1);
            out.peak_metrics.push_back(fold.metric);
            out.peak_corr.push_back(fold.corr);
        }
        out.measured_period = period_fit;
        out.fold_phase_per_symbol = fold_phase;
        out.preamble_start_sample = origin_fit;
        out.metric = fold.metric;
        first_start = fold_start;
        first_metric = fold.metric;
        first_corr = fold.corr;
        first_hit_k = 0;
        fold_used = true;
        return true;
    };

    if (bounded) {
        // Locked tracking contract: the target lock predicted this origin, so
        // only a bounded ROI around the seed is legal.  A miss is a tracking
        // failure -- never a wide scan and never a long backtrack.
        ++out.bounded_attempts;
        if (seed_start < 0) {
            ++out.bounded_failures;
            finish_timing_profile();
            return false;
        }
        const int64_t roi = std::max<int64_t>(
            1, static_cast<int64_t>(profile.locked_roi_samples));
        out.bounded_roi_samples = static_cast<size_t>(roi);
        // Probe the predicted SYNC first, then a bounded number of inner SYNCs
        // on the known symbol grid.  A weak startup SYNC must not cost the
        // packet, but the probe set stays finite: this is NOT a scan.
        const size_t inner = profile.locked_inner_symbols;
        int64_t pos = -1;
        float met = 0.0f;
        std::complex<float> corr(0.0f, 0.0f);
        size_t used_inner = 0;
        bool found = false;
        // Low-SNR early stop.  m = 0 keeps the full ROI (it is the predicted
        // SYNC, so a large residual must still be catchable).  If it misses
        // AND its raw window maximum is far below the PAR floor, the whole
        // leading region is deep-noise / deep-low-SNR, not merely mis-seeded:
        // the locked residual is a few samples, so the inner SYNCs can then be
        // probed at +/-32 instead of +/-roi (4 x 513 -> 4 x 65 correlations).
        // Anything this skips is still covered by the tail probe and, failing
        // that, by the fold -- both of which keep the full ROI.
        float m0_raw_max = 0.0f;
        for (size_t m = 0; m <= inner; ++m) {
            const int64_t want =
                static_cast<int64_t>(seed_start) +
                static_cast<int64_t>(m) * static_cast<int64_t>(kQm35SamplesPerSymbol);
            if (want + static_cast<int64_t>(L) >
                static_cast<int64_t>(n))
                break;
            const bool narrow_inner = (m != 0) && (m0_raw_max < kLockedInnerNarrowRawGate);
            const int64_t inner_rad =
                narrow_inner ? std::min<int64_t>(roi, 32) : roi;
            float raw_max = 0.0f;
            bool inner_hit = false;
            if (m == 0 && roi > 32) {
                // Steady-state lock residual is a few samples.  Probe +/-32
                // first (65 lags, the PAR background minimum) and only pay for
                // the full +/-roi scan when that misses.  A miss keeps the old
                // behaviour exactly, including m0_raw_max for the early-stop
                // heuristic below.
                inner_hit = local_best(want, 32, thr, &pos, &met, &corr,
                                       /*allow_par_fallback=*/true, &raw_max);
                if (!inner_hit) {
                    float wide_raw = 0.0f;
                    inner_hit = local_best(want, roi, thr, &pos, &met, &corr,
                                           /*allow_par_fallback=*/true,
                                           &wide_raw);
                    raw_max = std::max(raw_max, wide_raw);
                }
            } else {
                inner_hit = local_best(want, inner_rad, thr, &pos, &met, &corr,
                                       /*allow_par_fallback=*/true, &raw_max);
            }
            if (inner_hit) {
                used_inner = m;
                found = true;
                break;
            }
            if (m == 0) {
                m0_raw_max = raw_max;
                out.inner_probe_raw_max = raw_max;
            }
        }
        // Failure path only.  The leading SYNCs are all below threshold --
        // typically a burst interferer parked on the first symbols while the
        // tail of the preamble is clean (measured on the 5.2 m capture:
        // first_5_max 0.029..0.053, med_tail_32_255 0.14..0.20).  The old code
        // declared a tracking failure without ever looking past
        // `locked_inner_symbols`.  Keep the bounded ROI (the grid is known, so
        // only the residual is unknown) and probe sparse tail anchors on the
        // symbol grid.
        //
        // The threshold is the TRACKING threshold, not the acquisition one: a
        // low-SNR tail sits at 0.14..0.20, well under
        // radar_verification_threshold (0.25) but far above the per-SYNC
        // tracking floor.  The neighbour gate is what keeps a +/-roi false
        // alarm from being accepted at this lower threshold.
        const float tail_thr =
            std::min(profile.radar_verification_threshold, 0.08f);
        const size_t reps = profile.preamble_repetitions;
        const size_t step =
            std::max<size_t>(1, profile.locked_tail_probe_step);
        const size_t need_confirm =
            std::max<size_t>(1, profile.locked_tail_probe_confirm);
        // Two probe widths.  The locked residual is a few samples, so the
        // narrow (+/-32) sweep finds a clean tail cheaply.  The wide sweep
        // (+/-roi) only matters when the residual is larger than the narrow
        // window -- and it costs 32 x 513 = 16,416 single-SYMBOL correlations,
        // so it is NOT run before the fold: the fold covers the identical
        // window with a 64-SYMBOL coherent gain (~18 dB more sensitive) and is
        // the correct rescue in that regime.  The wide sweep is therefore the
        // last resort, reached only when the fold itself could not rescue the
        // packet.  Measured at -12 dB in-band (200 packets): the narrow sweep
        // never produced a confirmable hit and the fold rescued every packet,
        // so this ordering removes ~1.2 ms/packet of doomed work without
        // changing any outcome.
        auto tail_sweep = [&](int64_t trad) {
            for (size_t m = inner + 1; m < reps; m += step) {
                const int64_t want = static_cast<int64_t>(seed_start) +
                    static_cast<int64_t>(m) *
                        static_cast<int64_t>(kQm35SamplesPerSymbol);
                if (want + static_cast<int64_t>(L) > static_cast<int64_t>(n))
                    break;
                ++out.tail_probe_attempts;
                int64_t cpos = -1;
                float cmet = 0.0f;
                std::complex<float> ccorr(0.0f, 0.0f);
                float raw_max = 0.0f;
                const bool hit =
                    local_best(want, trad, tail_thr, &cpos, &cmet, &ccorr,
                               /*allow_par_fallback=*/true, &raw_max);
                if (raw_max > out.tail_probe_raw_max)
                    out.tail_probe_raw_max = raw_max;
                if (!hit)
                    continue;
                if (cmet > out.tail_probe_best_metric)
                    out.tail_probe_best_metric = cmet;
                // Consistency gate: a real grid hit is confirmed by its
                // neighbouring SYMBOLS.  A false alarm (wrong lag inside the
                // ROI) is not, so probing continues instead of locking onto a
                // bogus origin.
                size_t confirm = 0;
                const int64_t nb_delta[3] = {1, 2, -1};
                for (int64_t d : nb_delta) {
                    const int64_t nb = static_cast<int64_t>(m) + d;
                    if (nb < 0 || nb >= static_cast<int64_t>(reps))
                        continue;
                    const int64_t nb_want =
                        static_cast<int64_t>(seed_start) +
                        nb * static_cast<int64_t>(kQm35SamplesPerSymbol);
                    if (nb_want + static_cast<int64_t>(L) >
                        static_cast<int64_t>(n))
                        continue;
                    int64_t np = -1;
                    float nm = 0.0f;
                    std::complex<float> nc(0.0f, 0.0f);
                    float nb_raw = 0.0f;
                    const bool nb_hit =
                        local_best(nb_want, trad, tail_thr, &np, &nm, &nc,
                                   /*allow_par_fallback=*/false, &nb_raw);
                    if (nb_raw > out.tail_probe_raw_max)
                        out.tail_probe_raw_max = nb_raw;
                    if (nb_hit)
                        ++confirm;
                }
                if (confirm < need_confirm)
                    continue;
                ++out.tail_probe_hits;
                out.tail_probe_k = static_cast<int64_t>(m);
                pos = cpos;
                met = cmet;
                corr = ccorr;
                used_inner = m;
                found = true;
                return;
            }
        };
        if (!found && profile.locked_tail_probe)
            tail_sweep(std::min<int64_t>(roi, 32));
        if (!found) {
            // Low-SNR fallback: every single-symbol probe inside the bounded
            // ROI missed.  The fold keeps the same bounded window -- it is not
            // an escalation to a wide scan.
            //
            // Gate: LockedBounded's failure-path rescue is a single opt-out
            // tier.  `locked_tail_probe == false` is how a caller asks for the
            // original hard-fail tracking contract (no rescue at all), so the
            // fold honours the same switch instead of silently replacing the
            // tail probe with a different rescue.  qa_uwb_demod_core's
            // "tail probe rescues leading interference" case pins that
            // contract: with rescue disabled the packet MUST stay lost.
            const bool fold_ok =
                profile.locked_tail_probe &&
                try_fold_acquire(
                    static_cast<int64_t>(seed_start),
                    std::max<int64_t>(1, static_cast<int64_t>(
                                             profile.locked_roi_samples)));
            if (!fold_ok && profile.locked_tail_probe) {
                // Last resort (see the note above): the fold could not rescue
                // this packet either, so the wide single-SYMBOL sweep is the
                // only remaining option.  Paying its cost here cannot slow down
                // any packet the fold already handles.
                tail_sweep(roi);
            }
            if (!fold_ok && !found) {
                ++out.bounded_failures;
                finish_timing_profile();
                return false;
            }
        }
        if (!fold_used) {
            // Convert the inner-SYNC hit back to the packet start on the grid.
            first_start = pos - static_cast<int64_t>(used_inner) *
                                    static_cast<int64_t>(kQm35SamplesPerSymbol);
            first_hit_k = used_inner;
            first_metric = met;
            first_corr = corr;
            if (used_inner != 0)
                ++out.bounded_inner_used;
        }
        ++out.bounded_success;
    } else {
        const int64_t seed =
            (seed_start >= 0) ? seed_start
                              : (int64_t)(roi_start + (roi_end - roi_start) / 2);
        acq_seed = seed;
        const int64_t margin =
            static_cast<int64_t>(profile.timing_search_margin);
        const int64_t stride = std::max<int64_t>(
            1, static_cast<int64_t>(profile.timing_coarse_stride));
        const int64_t j_lo = std::max<int64_t>(0, seed - margin);
        const int64_t j_hi =
            std::min<int64_t>(static_cast<int64_t>(n) - static_cast<int64_t>(L),
                              seed + margin);

        auto consider = [&](int64_t pos, float met, std::complex<float> corr) {
            if (met >= thr && met > first_metric) {
                first_metric = met;
                first_start = pos;
                first_corr = corr;
            }
        };

        // MATLAB detectSeededPreamble: prefer a narrow FULL-RATE search around
        // the seed (max(256, ceil(symbolLength/32)) samples each side).  A wide
        // stride-S coarse scan can miss a narrow symbol-0 peak (sub-symbol seed
        // offset) and then prefer a later STRONGER SYNC -- exactly what made
        // the DW1000 preamble_start land ~6 SYNCs late.  The seed-nearby
        // result is authoritative; the wide scan is only a fallback.
        bool seed_acquired = false;
        const int64_t legacy_seed_radius =
            std::max<int64_t>(256, stride);
        if (profile.timing_seed_refine_radius != 0) {
            const int64_t narrow = rxopt::seed_refine_radius(
                profile.timing_seed_refine_radius, stride);
            if (narrow < legacy_seed_radius) {
                const auto narrow_begin = std::chrono::steady_clock::now();
                int64_t pos = -1;
                float met = 0.0f;
                std::complex<float> corr(0.0f, 0.0f);
                // PAR fallback is enabled on the two NARROW local searches
                // only.  A 256-SYNC preamble at -4..-8 dB in-band SNR has
                // every SYNC peak above 0.08 with PAR 232..253, but only
                // 11-24 of 256 clear the 0.25 absolute threshold, so the
                // absolute gate alone loses the packet.  The wide stride scan
                // and the whole-SYNC grid probes stay absolute-only: at
                // thousands of lags a pure-noise false alarm would clear any
                // PAR gate.
                const bool narrow_ok =
                    local_best(seed, narrow, thr, &pos, &met, &corr,
                               /*allow_par_fallback=*/true);
                out.seed_search_us = static_cast<uint64_t>(
                    std::chrono::duration_cast<std::chrono::microseconds>(
                        std::chrono::steady_clock::now() - narrow_begin).count());
                ++out.seed_refine_narrow_attempts;
                if (narrow_ok) {
                    first_start = pos;
                    first_metric = met;
                    first_corr = corr;
                    seed_acquired = true;
                    ++out.seed_refine_narrow_success;
                } else {
                    ++out.seed_refine_fallbacks;
                }
            }
        }
        if (!seed_acquired) {
            // Legacy first seeded refine.  Also the fallback after a failed
            // narrow search: a recovered packet must not be lost just because
            // the local peak was outside the narrow radius.
            int64_t pos = -1;
            float met = 0.0f;
            std::complex<float> corr(0.0f, 0.0f);
            if (local_best(seed, legacy_seed_radius, thr, &pos, &met, &corr,
                           /*allow_par_fallback=*/true)) {
                first_start = pos;
                first_metric = met;
                first_corr = corr;
                seed_acquired = true;
            }
        }

        // ---- Wide-window interval probe (K-symbol local fold) --------------
        // Reached only for AcquireLegacy when the blind search spans many
        // SYMBOL periods.  Each probe coherently folds K SYNC periods
        // (+3 dB at K=2, +6 dB at K=4) and correlates over one period.
        // The coarse probe grid only skips *start* positions, never a whole
        // preamble: a probe
        // anywhere inside the preamble still sees the same P-periodic code
        // (its peak lag resolves back to a true symbol boundary), so the
        // normal backtrack / SYNC-train / SFD path recovers the origin.
        //
        // On an all-miss, the historical stride-14 coarse scan, whole-SYNC
        // grid and fold rescue are skipped so an SIC candidate that contains
        // no target preamble fails fast to L0_Bypass instead of paying the
        // full wide acquisition.  rx_wide_probe_fallback restores the old
        // behaviour for callers that prefer recall over latency.
        if (!seed_acquired && profile.rx_wide_interval_probe) {
            const int64_t P = static_cast<int64_t>(kQm35SamplesPerSymbol);
            if (P > 0 && j_hi - j_lo >= 16 * P) {
                const size_t fold_k = std::max<size_t>(
                    2, std::min<size_t>(profile.rx_wide_probe_fold_symbols, 4));
                size_t step_symbols = profile.rx_wide_probe_step_symbols
                    ? profile.rx_wide_probe_step_symbols : 42;
                const size_t m_cap = std::max<size_t>(
                    8, profile.preamble_repetitions / 3);
                step_symbols = std::min(step_symbols, m_cap);
                if (profile.preamble_repetitions > fold_k)
                    step_symbols = std::min(
                        step_symbols, profile.preamble_repetitions - fold_k);
                else
                    step_symbols = 0;
                // The probe may land up to `step_symbols` into the preamble;
                // the backtrack must be able to walk that far back.
                if (profile.timing_max_backtrack_symbols > 0)
                    step_symbols = std::min(step_symbols,
                                            profile.timing_max_backtrack_symbols);
                const int64_t last_probe =
                    std::min(j_hi,
                             static_cast<int64_t>(n) -
                                 static_cast<int64_t>(fold_k) * P -
                                 static_cast<int64_t>(L) + 1);
                if (step_symbols > 0 && last_probe >= j_lo) {
                    const int64_t step =
                        static_cast<int64_t>(step_symbols) * P;
                    int64_t best_pos = -1;
                    float best_met = -1.0f;
                    std::complex<float> best_corr(0.0f, 0.0f);
                    int64_t last_scanned = -1;
                    auto probe_one = [&](int64_t p) {
                        if (p == last_scanned)
                            return;
                        last_scanned = p;
                        const auto r = detail::fold_probe_acquire(
                            rx, n, p, scratch.work.data(), L, Et,
                            scratch.real_sync_template, fold_k, thr,
                            sparse_tmpl, &scratch.fold_buf, shaped_tmpl);
                        if (r.ok && r.start >= j_lo && r.start <= j_hi &&
                            r.metric > best_met) {
                            best_met = r.metric;
                            best_pos = r.start;
                            best_corr = r.corr;
                        }
                    };
                    for (int64_t p = j_lo; p <= last_probe; p += step)
                        probe_one(p);
                    probe_one(last_probe);
                    if (best_pos >= 0) {
                        first_start = best_pos;
                        first_metric = best_met;
                        first_corr = best_corr;
                        seed_acquired = true;
                    } else if (!profile.rx_wide_probe_fallback) {
                        finish_timing_profile();
                        return false;
                    }
                }
            }
        }

        // ---- Phase probe + symbol-grid acquisition (TM-1) ------------------
        // The preamble is N verbatim copies of one P=1016-sample SYNC symbol.
        // A window of P consecutive lags therefore contains one code-phase
        // match, so the exact sample phase can be recovered from a handful of
        // probe windows instead of correlating all 2*margin lags.  A strict
        // threshold keeps this zero-risk: if the probe does not see a solid
        // SYNC (low SNR, packet outside the probes), everything below falls
        // through to the original coarse scan unchanged.
        //
        // Cheap pre-gate: the frontend seed is accurate to a few SYMBOLS, so
        // the code phase at the seed is preserved and one single-lag dot tells
        // us whether this window is deep noise.  The 0.2 floor is close to the
        // tracking threshold on purpose: at -12/-14 dB the single-SYNC metric
        // is ~0.04-0.1, so the three P-length probes (which would all fail)
        // are skipped and the fold path is not slowed down by them.
        float probe_gate = -1.0f;
        {
            const int64_t gp = std::min(std::max(seed, j_lo), j_hi);
            if (gp >= 0 && gp + static_cast<int64_t>(L) <=
                               static_cast<int64_t>(n)) {
                const auto acc = filter.dot(rx + gp);
                probe_gate = detail::normalized_correlation_power(
                    acc, window_power_8acc(rx + gp, L), Et);
            }
        }
        if (probe_gate >= 0.2f && !seed_acquired && j_hi > j_lo &&
            j_hi - j_lo >= 2 * static_cast<int64_t>(kQm35SamplesPerSymbol)) {
            const int64_t P = static_cast<int64_t>(kQm35SamplesPerSymbol);
            int64_t probes[3];
            size_t nprobe = 0;
            auto push_probe = [&](int64_t p) {
                if (p < j_lo || p > j_hi)
                    return;
                if (p + static_cast<int64_t>(L) + 64 > static_cast<int64_t>(n))
                    return;
                if (nprobe < 3)
                    probes[nprobe++] = p;
            };
            push_probe(std::min(std::max(seed, j_lo), j_hi));
            if (margin >= 32 * P) {
                push_probe(seed - 32 * P);
                push_probe(seed + 32 * P);
            }
            float best_probe = -1.0f;
            int64_t anchor = -1;
            for (size_t i = 0; i < nprobe; ++i) {
                const int64_t p = probes[i];
                const int64_t avail =
                    static_cast<int64_t>(n) - static_cast<int64_t>(L) - p + 1;
                const size_t lags = static_cast<size_t>(
                    std::min<int64_t>(P, std::max<int64_t>(0, avail)));
                if (lags < 64)
                    continue;
                const auto pk = rxopt::scan_normalized_sync(
                    filter, rx + p, lags, window_power_8acc(rx + p, L), Et);
                if (pk.metric > best_probe) {
                    best_probe = pk.metric;
                    anchor = p + static_cast<int64_t>(pk.lag);
                }
            }
            if (anchor >= 0 && best_probe >= thr) {
                // Same phase modulo P for every SYNC, so only the symbol grid
                // remains: ~2*margin/P grid points at +/-2 samples each.
                const int64_t rem = ((anchor - j_lo) % P + P) % P;
                for (int64_t want = j_lo + rem; want <= j_hi; want += P) {
                    int64_t pos = -1;
                    float met = 0.0f;
                    std::complex<float> corr(0.0f, 0.0f);
                    if (local_best(want, 2, thr, &pos, &met, &corr))
                        consider(pos, met, corr);
                }
                if (first_start >= 0)
                    seed_acquired = true;
            }
        }

        // (a) Coarse scan (fallback only) -- batched SIMD when stride is 1 or 2,
        // sliding power conjugate dots at stride otherwise.
        const auto coarse_begin = std::chrono::steady_clock::now();
        if (!seed_acquired && j_hi > j_lo) {
            float best_coarse_m = -1.0f;
            int64_t best_coarse_j = j_lo;
            std::complex<float> best_coarse_c(0.0f, 0.0f);
            if (stride == 1) {
                const size_t lag_count = static_cast<size_t>(j_hi - j_lo + 1);
                const auto pk = rxopt::scan_normalized_sync(
                    filter, rx + j_lo, lag_count,
                    window_power_8acc(rx + j_lo, L), Et);
                best_coarse_j = j_lo + static_cast<int64_t>(pk.lag);
                best_coarse_m = pk.metric;
                best_coarse_c = pk.corr;
            } else if (stride == 2 && sparse_tmpl && !shaped_tmpl && (L % 2 == 0)) {
                const size_t half_lags =
                    static_cast<size_t>((j_hi - j_lo) / 2 + 1);
                const size_t half_len = L / 2;
                const size_t even_count = (half_lags - 1) + half_len;
                scratch.corr.resize(even_count);
                rxopt::pack_even_complex(rx + j_lo, even_count,
                                         scratch.corr.data());
                rxopt::ensure_half_rate_sparse_template(
                    *sparse_tmpl, scratch.half_cir_template);
                const rxopt::SyncMatchedFilter half_filter(
                    nullptr, half_len, nullptr, &scratch.half_cir_template);
                const auto pk = rxopt::scan_normalized_sync(
                    half_filter, scratch.corr.data(), half_lags,
                    window_power_8acc(scratch.corr.data(), half_len), Et);
                best_coarse_j = j_lo + static_cast<int64_t>(pk.lag) * 2;
                best_coarse_c = pk.corr;
                best_coarse_m = detail::normalized_correlation_power(
                    pk.corr, window_power_8acc(rx + best_coarse_j, L), Et);
            } else {
                float pw = window_power_8acc(rx + j_lo, L);
                // Slide the window power forward by `cnt` samples with eight
                // independent accumulators, so the two |x|^2 streams vectorise and
                // only ONE serial float add per stride block remains.  The
                // per-sample form of this update was the single most expensive loop
                // in low-SNR acquisition: 262,144 iterations (the live search
                // margin is +-131,072 samples) at ~8 cycles each = ~2.2 ms/packet,
                // 85% of the whole acquisition search.  The sum is mathematically
                // identical; only the float association order changes.
                auto advance_power = [&](int64_t j, int64_t cnt) {
                    const float* a =
                        reinterpret_cast<const float*>(rx + j + static_cast<int64_t>(L));
                    const float* b = reinterpret_cast<const float*>(rx + j);
                    const size_t m = static_cast<size_t>(2 * cnt);
                    float s0 = 0.f, s1 = 0.f, s2 = 0.f, s3 = 0.f;
                    float s4 = 0.f, s5 = 0.f, s6 = 0.f, s7 = 0.f;
                    size_t i = 0;
                    for (; i + 8 <= m; i += 8) {
                        s0 += a[i + 0] * a[i + 0] - b[i + 0] * b[i + 0];
                        s1 += a[i + 1] * a[i + 1] - b[i + 1] * b[i + 1];
                        s2 += a[i + 2] * a[i + 2] - b[i + 2] * b[i + 2];
                        s3 += a[i + 3] * a[i + 3] - b[i + 3] * b[i + 3];
                        s4 += a[i + 4] * a[i + 4] - b[i + 4] * b[i + 4];
                        s5 += a[i + 5] * a[i + 5] - b[i + 5] * b[i + 5];
                        s6 += a[i + 6] * a[i + 6] - b[i + 6] * b[i + 6];
                        s7 += a[i + 7] * a[i + 7] - b[i + 7] * b[i + 7];
                    }
                    float s = ((s0 + s1) + (s2 + s3)) + ((s4 + s5) + (s6 + s7));
                    for (; i < m; ++i)
                        s += a[i] * a[i] - b[i] * b[i];
                    pw += s;
                };
                for (int64_t j = j_lo; j <= j_hi; j += stride) {
                    const auto acc = filter.dot(rx + j);
                    const float m = detail::normalized_correlation_power(acc, pw, Et);
                    if (m > best_coarse_m) {
                        best_coarse_m = m;
                        best_coarse_j = j;
                        best_coarse_c = acc;
                    }
                    const int64_t cnt = std::min<int64_t>(stride, j_hi + 1 - j);
                    advance_power(j, cnt);
                }
            }
            out.coarse_search_us = static_cast<uint64_t>(
                std::chrono::duration_cast<std::chrono::microseconds>(
                    std::chrono::steady_clock::now() - coarse_begin).count());
            // (b) Fine refine around the coarse peak.
            int64_t pos = -1;
            float met = 0.0f;
            std::complex<float> corr(0.0f, 0.0f);
            if (local_best(best_coarse_j, stride, thr, &pos, &met, &corr))
                consider(pos, met, corr);
            else if (best_coarse_m >= thr)
                consider(best_coarse_j, best_coarse_m, best_coarse_c);
        }

        // (c) Whole-SYNC grid probes (fallback only; covers a seed that lands
        // mid-preamble and a sub-symbol confirmation that must not replace an
        // SFD-refined anchor with a stronger neighbouring SYNC).
        if (!seed_acquired) {
            int64_t pos = -1;
            float met = 0.0f;
            std::complex<float> corr(0.0f, 0.0f);
            const int64_t grid_half = margin / kQm35SamplesPerSymbol;
            for (int64_t n_sym = -grid_half; n_sym <= grid_half; ++n_sym) {
                if (local_best(seed + n_sym * static_cast<int64_t>(
                                               kQm35SamplesPerSymbol),
                               8, thr, &pos, &met, &corr))
                    consider(pos, met, corr);
            }
        }
        if (first_start < 0) {
            // Low-SNR fallback: narrow seed refine, legacy seed refine, stride
            // coarse scan and whole-SYNC grid probes all missed.  Fold over a
            // capped acquisition window (never the full 40k margin: the fold
            // cost grows with the window and the seed is already close).
            if (!try_fold_acquire(
                    seed, std::min<int64_t>(margin, 8192))) {
                finish_timing_profile();
                return false;
            }
        }
    }
    // A real QM35825 has a repeatable startup transient: the first few SYNCs
    // can score below the normal verification threshold, so the detector seeds
    // the first strong SYNC.  Once locked, walk backwards on the known grid
    // with a relaxed threshold, stopping at the first missing predecessor.
    // LockedBounded disables this by default: walking backwards is exactly how
    // a lock silently drifts onto SYNC #2/#3.
    //
    // The fold path disables it unconditionally: its origin came from a
    // 64-symbol coherent fold, which is far more reliable than a single-SYNC
    // relaxed-threshold probe, and a single SYNC at -12..-14 dB cannot clear
    // 0.7*0.25 anyway.
    size_t backtracked_symbols = 0;
    const size_t backtrack_limit =
        (fold_used || (bounded && !profile.locked_allow_backtrack))
            ? 0
            : profile.timing_max_backtrack_symbols;
    for (size_t back = 0; back < backtrack_limit; ++back) {
        int64_t previous_start = -1;
        float previous_metric = 0.0f;
        std::complex<float> previous_corr(0.0f, 0.0f);
        const float relaxed = 0.7f * profile.radar_verification_threshold;
        if (local_best(first_start - static_cast<int64_t>(kQm35SamplesPerSymbol),
                       8, relaxed, &previous_start, &previous_metric,
                       &previous_corr)) {
            first_start = previous_start;
            first_metric = previous_metric;
            first_corr = previous_corr;
            ++backtracked_symbols;
        } else
            break;
    }
    out.backtrack_symbols = static_cast<uint32_t>(backtracked_symbols);
    const int64_t start = first_start;
    if (start < 0) {
        finish_timing_profile();
        return false;
    }
    const int64_t period = static_cast<int64_t>(kQm35SamplesPerSymbol);

    // ---- LockedBounded grid tracking: sparse anchors + single point --------
    // The locked contract already knows the symbol grid (the target lock
    // predicted this origin), so the only unknowns are the residual origin and
    // the residual period.  Measure a sparse set of SYNC anchors with the
    // normal +/-track_radius search, fit period+origin from the anchors, then
    // evaluate every other SYNC with ONE correlation at start+round(k*period).
    // The peak train keeps the same meaning (CFO needs peak_corr phases, CIR
    // needs the fitted period), at ~1/17 of the per-symbol search cost.
    // measured_period MUST come from the anchor fit: it feeds
    // integer_chip_grid()/spc_ppm and the fused-CMF admission, so a period
    // that degenerates to the nominal 1016 would silently disable the
    // fractional-grid fallback for an untrimmed clock.
    bool grid_applied = false;
    double grid_period = static_cast<double>(period);
    int64_t grid_start = start;
    auto grid_track = [&]() -> bool {
        const size_t reps = profile.preamble_repetitions;
        if (reps < 8)
            return false;
        const float gthr =
            std::min(profile.radar_verification_threshold, 0.08f);
        const int64_t arad = std::max<int64_t>(
            8, static_cast<int64_t>(profile.timing_track_radius));
        const double nominal = static_cast<double>(period);

        // Sparse anchor set on the symbol grid: the confirmed first SYNC, the
        // last SYNC of the train, and evenly spaced points in between.
        std::array<size_t, 32> ak{};
        size_t na = 0;
        ak[na++] = first_hit_k;
        const size_t last_k = reps - 1;
        if (last_k > first_hit_k) {
            const size_t want_anchors = std::min<size_t>(
                std::max<size_t>(4, profile.locked_grid_anchor_count), ak.size());
            const size_t intervals = (want_anchors > 1) ? (want_anchors - 1) : 1;
            const double span = static_cast<double>(last_k - first_hit_k);
            for (size_t i = 1; i <= intervals && na < ak.size(); ++i) {
                const size_t k = first_hit_k + static_cast<size_t>(std::llround(
                    span * static_cast<double>(i) /
                    static_cast<double>(intervals)));
                if (k != ak[na - 1])
                    ak[na++] = k;
            }
        }

        // One least-squares pass over the anchors: peak(k) = start + k*period,
        // in the same symbol-END convention as the legacy fit.
        auto fit_anchors = [&](double period_hint, int64_t start_hint,
                               int64_t radius, double* period_out,
                               int64_t* start_out, float* median_out,
                               size_t* hits_out) -> bool {
            detail::LinearFit fit;
            std::array<float, 32> mets{};
            size_t hits = 0;
            size_t kmin = 0, kmax = 0;
            for (size_t i = 0; i < na; ++i) {
                const size_t k = ak[i];
                const int64_t want = start_hint + static_cast<int64_t>(
                    std::llround(static_cast<double>(k) * period_hint));
                if (want < 0 ||
                    want + static_cast<int64_t>(L) > static_cast<int64_t>(n))
                    continue;
                ++out.grid_anchor_attempts;
                int64_t pos = -1;
                float met = 0.0f;
                std::complex<float> c(0.0f, 0.0f);
                if (!local_best(want, radius, gthr, &pos, &met, &c))
                    continue;
                const double x = static_cast<double>(k);
                const double y = static_cast<double>(pos + (int64_t)(L - 1));
                fit.add(x, y);
                if (hits == 0) {
                    kmin = k;
                    kmax = k;
                } else {
                    kmin = std::min(kmin, k);
                    kmax = std::max(kmax, k);
                }
                mets[hits++] = met;
            }
            *hits_out = hits;
            if (hits < std::max<size_t>(2, profile.locked_grid_min_anchors))
                return false;
            if (kmax - kmin < 8)
                return false; // anchors too bunched to measure a slope
            const double denom = fit.determinant();
            if (!(std::abs(denom) > 1e-9))
                return false;
            const double p = fit.slope(denom);
            if (!std::isfinite(p) || !(p > 0.0))
                return false;
            const double intercept_end = fit.intercept(p);
            *period_out = p;
            *start_out = std::max<int64_t>(
                0, static_cast<int64_t>(std::llround(intercept_end)) -
                       static_cast<int64_t>(L - 1));
            std::sort(mets.begin(), mets.begin() + hits);
            *median_out = mets[hits / 2];
            return true;
        };

        double p1 = nominal;
        int64_t s1 = start;
        float med1 = 0.0f;
        size_t hits1 = 0;
        if (!fit_anchors(nominal, start, arad, &p1, &s1, &med1, &hits1))
            return false;
        // A fitted period outside 0.5% of the nominal work-grid SYNC period
        // means the anchors did not land on the expected symbol indices.
        if (std::abs(p1 - nominal) > 0.005 * nominal)
            return false;
        double p2 = p1;
        int64_t s2 = s1;
        float med2 = med1;
        size_t hits2 = hits1;
        // Second pass only when the nominal grid would put the last anchor
        // more than 4 samples off (an untrimmed clock).  With a trimmed
        // crystal the first pass is already unbiased and this costs nothing.
        if (std::abs(p1 - nominal) * static_cast<double>(last_k) > 4.0) {
            if (!fit_anchors(p1, s1, std::max<int64_t>(arad, 16), &p2, &s2,
                             &med2, &hits2))
                return false;
            if (std::abs(p2 - nominal) > 0.005 * nominal)
                return false;
        }
        out.grid_anchor_hits = static_cast<uint32_t>(hits2);

        // Single-point stage: evaluate eight consecutive lags around the fitted
        // grid position and keep the best.  The 8-lag kernel has independent
        // accumulators and is far cheaper per lag than the single-lag kernel
        // (which is one long FMA dependency chain), so eight lags cost barely
        // more than one -- and taking the argmax keeps the peak positions and
        // the CFO phases identical to the legacy per-SYNC search instead of
        // pinning them to the fitted grid.
        scratch.metric.assign(reps, 0.0f);
        size_t valid = 0;
        // Least-squares accumulators over every refined peak (see the final
        // fit note after the loop).
        detail::LinearFit final_fit;
        size_t fn = 0;
        for (size_t k = 0; k < reps; ++k) {
            if (k == first_hit_k) {
                // The bounded probe already measured this SYNC with a real
                // +/-roi search; keep that measurement instead of
                // re-projecting it onto the fitted grid.
                scratch.metric[k] = first_metric;
                ++out.grid_single_points;
                if (first_metric >= gthr) {
                    ++valid;
                    out.peak_samples.push_back(start + static_cast<int64_t>(L - 1));
                    out.peak_metrics.push_back(first_metric);
                    out.peak_corr.push_back(first_corr);
                    const double x = static_cast<double>(k);
                    const double y = static_cast<double>(start + (int64_t)(L - 1));
                    final_fit.add(x, y);
                    ++fn;
                }
                continue;
            }
            const int64_t centre = s2 + static_cast<int64_t>(
                std::llround(static_cast<double>(k) * p2));
            int64_t base = centre - 3;
            int nl = 8;
            if (base + 7 + static_cast<int64_t>(L) > static_cast<int64_t>(n)) {
                // Window edge: fall back to the single lag at the prediction
                // so the last SYNCs are still measured.
                if (centre + static_cast<int64_t>(L) >
                    static_cast<int64_t>(n))
                    continue;
                base = centre;
                nl = 1;
            }
            if (base < 0)
                continue;
            float pw = window_power_8acc(rx + base, L);
            rxopt::MatchedFilterPeak peak;
            filter.for_each_lag(rx + base, static_cast<size_t>(nl),
                                [&](size_t lag, std::complex<float> corr, bool) {
                if (lag != 0)
                    pw += std::norm(rx[base + lag - 1 + L]) -
                          std::norm(rx[base + lag - 1]);
                const float metric = detail::normalized_correlation_power(corr, pw, Et);
                if (lag == 0 || metric > peak.metric)
                    peak = { lag, metric, corr };
            });
            const int64_t want = base + static_cast<int64_t>(peak.lag);
            const float m = peak.metric;
            scratch.metric[k] = m;
            ++out.grid_single_points;
            if (m < gthr)
                continue;
            ++valid;
            out.peak_samples.push_back(want + static_cast<int64_t>(L - 1));
            out.peak_metrics.push_back(m);
            out.peak_corr.push_back(peak.corr);
            const double x = static_cast<double>(k);
            const double y = static_cast<double>(want + (int64_t)(L - 1));
            final_fit.add(x, y);
            ++fn;
        }
        out.grid_single_valid = static_cast<uint32_t>(valid);
        if (valid < 4)
            return false;
        // Grid sanity: if the fitted grid does not land on real peaks, the
        // single-point metrics collapse relative to the anchor peaks.  Use the
        // 90th percentile so a weak (interfered) part of the preamble does not
        // by itself reject a correct grid.
        if (reps <= 512) {
            const size_t idx = static_cast<size_t>(
                0.9 * static_cast<double>(reps - 1));
            std::nth_element(scratch.metric.begin(),
                             scratch.metric.begin() + idx,
                             scratch.metric.end());
            if (!(scratch.metric[idx] >= 0.5f * med2))
                return false;
        }
        // Final period/origin: least squares over EVERY refined peak, which is
        // exactly the estimator the legacy 256-point fit uses.  The anchors are
        // the bootstrap that puts the single-point windows in the right place
        // (and the fallback if this fit is degenerate); they are not the final
        // estimate, because a 16-anchor fit is ~4x noisier than the legacy
        // 256-point fit and that noise measurably flips marginal payloads
        // (measured: 1 packet / 2000 at 2.5 m).  The refined positions are real
        // argmax measurements, NOT the quantised grid, so measured_period stays
        // a genuine measurement and integer_chip_grid()/spc_ppm keeps working.
        if (fn >= 8) {
            const double denom = final_fit.determinant();
            if (std::abs(denom) > 1e-9) {
                const double pr = final_fit.slope(denom);
                const double intercept_end = final_fit.intercept(pr);
                if (std::isfinite(pr) && pr > 0.0 &&
                    std::abs(pr - p2) <= 0.01) { // 10 ppm sanity vs anchors
                    p2 = pr;
                    s2 = std::max<int64_t>(
                        0, static_cast<int64_t>(std::llround(intercept_end)) -
                               static_cast<int64_t>(L - 1));
                }
            }
        }
        grid_period = p2;
        grid_start = s2;
        grid_applied = true;
        return true;
    };
    if (bounded && profile.locked_grid_timing && !fold_used) {
        if (!grid_track()) {
            ++out.grid_fallbacks;
            out.peak_samples.clear();
            out.peak_metrics.clear();
            out.peak_corr.clear();
        }
    }

    // Track all SYNC starts: expected = start + k*period, then convert each to
    // symbol-END convention (start + L - 1) to match the detector / MATLAB.
    // When bounded inner-SYNC probing hit symbol `first_hit_k > 0`, anchor the
    // first peak at its true symbol coordinate and continue from `first_hit_k + 1`
    // rather than restarting at k=1 (which already failed).  Also allow up to 2
    // consecutive missed SYNCs so a brief 1-2 symbol interference dip does not
    // prematurely truncate the 256-symbol train.
    std::vector<size_t> peak_indices;
    peak_indices.reserve(profile.preamble_repetitions);
    // LockedBounded's per-SYMBOL tracking floor.  A first peak below it cannot
    // be the head of a usable train: the tail probe already swept 32 tail
    // symbols at exactly this floor and found nothing confirmable, so running
    // the 256-SYMBOL train here is provably doomed (it would truncate to a
    // single peak and fall through to the post-train fold retry anyway).
    // Skipping it saves 256 x 27 single-SYMBOL correlations; the fold below is
    // the path that actually rescues these packets.  A first peak ABOVE the
    // floor (e.g. the 5.2 m tail-probe rescue at 0.14..0.20) still runs the
    // full train, because there the clean tail really does track.
    const float bounded_track_floor =
        std::min(profile.radar_verification_threshold, 0.08f);
    const bool skip_doomed_bounded_train =
        bounded && profile.locked_tail_probe && !grid_applied && !fold_used &&
        (first_metric < bounded_track_floor);
    if (!grid_applied && !fold_used && !skip_doomed_bounded_train) {
        constexpr size_t kMaxConsecutiveSyncMisses = 2;
        // One tracking pass at a given floor.  `weak` selects the 0.08 floor
        // LockedBounded already uses for its tail probe; the legacy absolute
        // threshold is used otherwise.
        auto track_train = [&](bool weak) {
            out.peak_samples.clear();
            out.peak_metrics.clear();
            out.peak_corr.clear();
            peak_indices.clear();
            const int64_t first_hit_start =
                start + static_cast<int64_t>(first_hit_k) * period;
            out.peak_samples.push_back(first_hit_start +
                                       static_cast<int64_t>(L - 1));
            out.peak_metrics.push_back(first_metric);
            out.peak_corr.push_back(first_corr);
            peak_indices.push_back(first_hit_k);
            size_t consecutive_misses = 0;
            for (size_t k = first_hit_k + 1;
                 k < profile.preamble_repetitions; ++k) {
                const int64_t want = start + static_cast<int64_t>(k * period);
                int64_t pk = -1;
                float pm = 0.0f;
                std::complex<float> pk_corr(0.0f, 0.0f);
                const float tracking_threshold =
                    (bounded || weak)
                        ? std::min(profile.radar_verification_threshold,
                                   bounded ? 0.08f : 0.06f)
                        : ((k < backtracked_symbols)
                               ? 0.7f * profile.radar_verification_threshold
                               : profile.radar_verification_threshold);
                const int64_t track_rad =
                    (bounded && consecutive_misses > 0)
                        ? std::max<int64_t>(
                              12, static_cast<int64_t>(
                                      profile.timing_track_radius))
                        : static_cast<int64_t>(profile.timing_track_radius);
                if (!local_best(want, track_rad, tracking_threshold, &pk, &pm,
                                &pk_corr)) {
                    ++consecutive_misses;
                    if (!bounded &&
                        consecutive_misses > kMaxConsecutiveSyncMisses)
                        break; // lost the train
                    continue;
                }
                consecutive_misses = 0;
                out.peak_samples.push_back(pk + static_cast<int64_t>(L - 1));
                out.peak_metrics.push_back(pm);
                out.peak_corr.push_back(pk_corr);
                peak_indices.push_back(k);
            }
            return out.peak_samples.size();
        };
        // First pass at the legacy threshold: high-SNR packets are complete
        // here and pay nothing.  Only a train that the absolute threshold
        // truncated (a low-SNR acquisition whose SYNCs all sit at the same
        // sub-threshold level) is retried at the 0.08 floor.  Without this the
        // packet is lost one stage later anyway (fit_cfo needs >= 4 peaks and
        // stage_cir_softchips needs more than cir_skip_initial_repetitions).
        const size_t min_peaks =
            std::max<size_t>(4, profile.min_valid_peaks);
        size_t tracked = track_train(false);
        if (!bounded && tracked < min_peaks)
            tracked = track_train(true);
        (void)tracked;
    }

    // ---- Post-train fold retry -------------------------------------------
    // "All single-symbol paths failed" is also observable one stage later: at
    // -4..-8 dB in-band SNR the narrow PAR search DOES acquire SYNC #0 (Task 1)
    // but the per-SYNC tracking threshold (radar_verification_threshold = 0.25)
    // still truncates the train to a single peak.  That result is unusable --
    // fit_cfo needs >= 4 peaks and stage_cir_softchips needs more than
    // cir_skip_initial_repetitions -- so the packet was already lost, just at
    // CfoFailed instead of TimingFailed.  Retry the fold, which measures a
    // usable train directly.  profile.min_valid_peaks ("min SYNC peaks to
    // accept timing") is the existing contract for "this train is too short to
    // be a timing result"; it was declared but never enforced, and this is
    // where it belongs.
    const bool allow_post_train_fold = !bounded || profile.locked_tail_probe;
    size_t min_train_peaks = std::max<size_t>(4, profile.min_valid_peaks);
    if (raw_par_hit && profile.rx_par_confirm) {
        // A weak single-SYNC PAR hit may start tracking with too few phase
        // points for a stable CFO estimate. Give the existing fold a chance
        // to build its coherent anchors in that case.
        min_train_peaks = std::max(
            min_train_peaks,
            std::min(profile.cfo_min_fit_repetitions, profile.preamble_repetitions));
    }
    if (allow_post_train_fold && !fold_used && !grid_applied &&
        out.peak_samples.size() < min_train_peaks) {
        const int64_t fold_centre =
            bounded ? static_cast<int64_t>(seed_start) : acq_seed;
        const int64_t fold_radius = bounded
            ? std::max<int64_t>(1, static_cast<int64_t>(profile.locked_roi_samples))
            : std::min<int64_t>(static_cast<int64_t>(profile.timing_search_margin),
                                8192);
        if (fold_centre >= 0) {
            std::vector<int64_t> saved_ps = std::move(out.peak_samples);
            std::vector<float> saved_pm = std::move(out.peak_metrics);
            std::vector<std::complex<float>> saved_pc = std::move(out.peak_corr);
            out.peak_samples.clear();
            out.peak_metrics.clear();
            out.peak_corr.clear();
            if (!try_fold_acquire(fold_centre, fold_radius)) {
                out.peak_samples = std::move(saved_ps);
                out.peak_metrics = std::move(saved_pm);
                out.peak_corr = std::move(saved_pc);
            }
        }
    }

    out.detected_peaks = out.peak_samples.size();
    out.sync_peaks_folded = fold_used;
    if (fold_used)
        out.cir_grid_repetitions = profile.preamble_repetitions;
    out.expected_peaks = profile.preamble_repetitions;
    out.metric = first_metric;

    // Linear fit of peak positions vs repetition index -> measured period and
    // the preamble start.  peak(k) = start + k*period  =>  least squares
    // slope = period, intercept = the SYNC-start at index 0.  This matches
    // MATLAB trackCandidatePreamble: startSample = round(polyfit(...,1).coef
    // (2)), i.e. the start is derived from the WHOLE tracked train rather than
    // the first peak, so a slightly-off first peak does not bias the origin.
    const size_t np = out.peak_samples.size();
    if (fold_used) {
        // measured_period / preamble_start_sample were already set by the
        // fold path's 16-super-anchor fit (see try_fold_acquire).  The legacy
        // per-SYNC train never ran, so there is nothing to refit here.
    } else if (grid_applied) {
        // LockedBounded grid: period and origin come from the anchor
        // least-squares fit, NOT from the single-point train (whose positions
        // are `period` by construction and would make integer_chip_grid()
        // blind to a real clock offset).
        out.measured_period = grid_period;
        out.preamble_start_sample = grid_start;
    } else if (np >= 2) {
        detail::LinearFit fit;
        for (size_t k = 0; k < np; ++k) {
            const double x = static_cast<double>(peak_indices[k]);
            const double y = static_cast<double>(out.peak_samples[k]);
            fit.add(x, y);
        }
        const double denom = fit.determinant();
        if (std::abs(denom) > 1e-12) {
            out.measured_period = fit.slope(denom);
            // peak_samples use the symbol-END convention (start + L - 1), so
            // the intercept is also symbol-END; convert back to the SYNC start.
            const double intercept_end = fit.intercept(out.measured_period);
            out.preamble_start_sample = std::max<int64_t>(
                0, static_cast<int64_t>(std::llround(intercept_end)) -
                       (static_cast<int64_t>(L) - 1));
        } else {
            out.measured_period = static_cast<double>(period);
            out.preamble_start_sample = start;
        }
    } else {
        out.measured_period = static_cast<double>(period);
        out.preamble_start_sample = start;
    }

    out.ok = true;
    finish_timing_profile();
    return true;
}

namespace detail {

size_t
normalized_mf(const std::complex<float>* rx,
              size_t n,
              size_t roi_start,
              size_t roi_end,
              const std::complex<float>* tmpl,
              size_t L,
              float tmpl_energy,
              std::vector<float>& metric, // caller-owned, >= n
              float* best_metric)
{
    const size_t jmax = (roi_end >= L) ? roi_end - L + 1 : roi_start;
    size_t best = roi_start;
    float bm = -1.0f;
    if (jmax <= roi_start)
        return best;
    // window power of rx over the same L-length window (running sum)
    float wsum = sample_power_sum(rx + roi_start, L);
    for (size_t j = roi_start; j < jmax; ++j) {
        const auto corr = conjugate_dot_scalar(rx + j, tmpl, L);
        const float m = normalized_correlation_power(corr, wsum, tmpl_energy);
        metric[j] = m;
        if (m > bm) {
            bm = m;
            best = j;
        }
        if (j + 1 < jmax) {
            wsum -= std::norm(rx[j]);
            wsum += std::norm(rx[j + L]);
        }
    }
    if (best_metric)
        *best_metric = bm;
    return best;
}

FoldPhaseProbe probe_fold_phase(const std::complex<float>* rx,
                                size_t n,
                                int64_t centre,
                                size_t repetitions)
{
    const size_t sym = kQm35SamplesPerSymbol;
    if (repetitions < 32)
        return {};
    const int64_t lo = centre + static_cast<int64_t>((repetitions / 4) * sym);
    if (lo < 0 || static_cast<size_t>(lo) >= n ||
        n - static_cast<size_t>(lo) <= sym)
        return {};
    const size_t count = std::min(std::min<size_t>(repetitions / 2, 128) * sym,
                                  n - static_cast<size_t>(lo) - sym);
    if (count < 16 * sym)
        return {};
    const auto a = rx + lo;
    FoldPhaseProbe probe;
    probe.samples = a;
    probe.count = count;
    probe.corr = timing_conjugate_dot(a + sym, a, count);
    probe.ea = timing_conjugate_dot(a, a, count).real();
    float head = 0.0f;
    float tail = 0.0f;
    for (size_t i = 0; i < sym; ++i) {
        head += std::norm(a[i]);
        tail += std::norm(a[count + i]);
    }
    probe.eb = probe.ea - head + tail;
    return probe;
}

FoldAcquireResult fold_acquire(const std::complex<float>* rx,
                               size_t n,
                               int64_t centre,
                               int64_t w0_in,
                               int64_t w1_in,
                               const std::complex<float>* tmpl,
                               size_t L,
                               float Et,
                               const rxopt::RealSyncTemplate& real_tmpl,
                               size_t max_symbols,
                               size_t preamble_repetitions,
                               float threshold,
                               double phase_per_symbol,
                               const rxopt::SparseCirTemplate* sparse_tmpl,
                               std::vector<std::complex<float>>* fold_buf,
                               const rxopt::ShapedSparseTemplate* shaped_tmpl)
{
    FoldAcquireResult res;
    const int64_t P = static_cast<int64_t>(kQm35SamplesPerSymbol);
    const int64_t Li = static_cast<int64_t>(L);
    const bool have_sparse = sparse_tmpl && sparse_tmpl->code_energy > 0.0f;
    const bool have_shaped = shaped_tmpl && shaped_tmpl->valid;
    if (!rx || L == 0 || n < L || (!tmpl && !have_sparse && !have_shaped))
        return res;

    int64_t w0 = std::max<int64_t>(0, w0_in);
    int64_t w1 = std::min<int64_t>(w1_in, static_cast<int64_t>(n) - Li);
    if (w1 < w0)
        return res;

    // The deepest sample the fold touches is w1 + L - 1 + (K-1)*P, so the
    // buffer must hold K whole symbol periods starting at w0.
    const int64_t room = static_cast<int64_t>(n) - w0 - Li;
    if (room < 0)
        return res;
    int64_t k_eff = std::min<int64_t>(static_cast<int64_t>(max_symbols),
                                      room / P + 1);
    // Folding past the end of the preamble only adds noise, never signal.
    k_eff = std::min<int64_t>(k_eff,
                              static_cast<int64_t>(preamble_repetitions));
    if (k_eff < static_cast<int64_t>(kFoldMinSymbols))
        return res;
    w1 = std::min<int64_t>(w1, static_cast<int64_t>(n) - Li - (k_eff - 1) * P);
    if (w1 < w0)
        return res;

    // Only lags inside +/-P/2 of the seed can be selected (see the note below),
    // so the fold and the correlation only have to cover that sub-window.
    // Clamping here instead of inside the correlation loop shrinks the fold
    // accumulation by the same factor: under AcquireLegacy the search radius is
    // 8192 samples, i.e. span 16384 -> 1016 lags (-93.8%), and every discarded
    // lag was unreachable by the argmax anyway.  `room` / `k_eff` above are
    // still derived from the ORIGINAL w0 so the fold depth (and therefore the
    // folded vector itself) is unchanged.
    const int64_t half_period = P / 2;
    const int64_t c0 = std::max<int64_t>(w0, centre - half_period);
    const int64_t c1 = std::min<int64_t>(w1, centre + half_period);
    if (c1 < c0)
        return res; // the seed lies entirely outside the legal window
    w0 = c0;
    w1 = c1;

    const int64_t span = w1 - w0;
    const size_t fold_len = static_cast<size_t>(span + Li);
    // Reuse the worker buffer when the caller provides one: the fold path runs
    // at most once per packet, but it is also reached on retries, and a
    // per-packet allocation is pure allocator traffic on the low-SNR path.
    std::vector<std::complex<float>> local_folded;
    std::vector<std::complex<float>>& folded =
        fold_buf ? *fold_buf : local_folded;
    if (folded.size() < fold_len)
        folded.resize(fold_len);
    std::fill(folded.begin(), folded.begin() + fold_len,
              std::complex<float>(0.0f, 0.0f));
    for (int64_t k = 0; k < k_eff; ++k)
        fold_accumulate(folded.data(), rx + w0 + k * P, fold_len,
                        -k * phase_per_symbol);

    const float* real_taps = real_tmpl.is_real && real_tmpl.length == L
        ? real_tmpl.taps_dup.data() : nullptr;
    const rxopt::SyncMatchedFilter filter(tmpl, L, real_taps, sparse_tmpl,
                                          true, shaped_tmpl);
    if (shaped_tmpl && shaped_tmpl->valid)
        Et = shaped_tmpl->energy;
    else if (sparse_tmpl && sparse_tmpl->code_energy > 0.0f)
        Et = sparse_tmpl->code_energy;
    const size_t lags = static_cast<size_t>(span + 1);
    const auto peak = rxopt::scan_normalized_sync(
        filter, folded.data(), lags, window_power_8acc(folded.data(), L), Et);

    res.symbols_folded = static_cast<size_t>(k_eff);
    res.window_lags = lags;
    if (!(peak.metric >= threshold))
        return res;
    res.ok = true;
    res.start = w0 + static_cast<int64_t>(peak.lag);
    res.metric = peak.metric;
    res.corr = peak.corr;
    return res;
}

FoldAcquireResult fold_probe_acquire(
    const std::complex<float>* rx,
    size_t n,
    int64_t probe,
    const std::complex<float>* tmpl,
    size_t L,
    float Et,
    const rxopt::RealSyncTemplate& real_tmpl,
    size_t fold_symbols,
    float threshold,
    const rxopt::SparseCirTemplate* sparse_tmpl,
    std::vector<std::complex<float>>* fold_buf,
    const rxopt::ShapedSparseTemplate* shaped_tmpl)
{
    FoldAcquireResult res;
    const int64_t P = static_cast<int64_t>(kQm35SamplesPerSymbol);
    const int64_t Li = static_cast<int64_t>(L);
    const bool have_sparse = sparse_tmpl && sparse_tmpl->code_energy > 0.0f;
    const bool have_shaped = shaped_tmpl && shaped_tmpl->valid;
    if (!rx || L == 0 || n < L || fold_symbols == 0 || probe < 0 ||
        (!tmpl && !have_sparse && !have_shaped))
        return res;
    if (fold_symbols > static_cast<size_t>(P))
        fold_symbols = static_cast<size_t>(P);
    const int64_t K = static_cast<int64_t>(fold_symbols);
    // Buffer covers one period of possible start phases plus the template.
    const int64_t fold_len = P + Li - 1;
    // The deepest sample is probe + (K-1)*P + fold_len - 1.
    if (probe + K * P + Li - 1 > static_cast<int64_t>(n))
        return res;

    std::vector<std::complex<float>> local_folded;
    std::vector<std::complex<float>>& folded =
        fold_buf ? *fold_buf : local_folded;
    const size_t flen = static_cast<size_t>(fold_len);
    if (folded.size() < flen)
        folded.resize(flen);
    std::fill(folded.begin(), folded.begin() + flen,
              std::complex<float>(0.0f, 0.0f));
    for (int64_t k = 0; k < K; ++k)
        fold_accumulate(folded.data(), rx + probe + k * P, flen, 0.0);

    const float* real_taps = real_tmpl.is_real && real_tmpl.length == L
        ? real_tmpl.taps_dup.data() : nullptr;
    const rxopt::SyncMatchedFilter filter(tmpl, L, real_taps, sparse_tmpl,
                                          true, shaped_tmpl);
    if (shaped_tmpl && shaped_tmpl->valid)
        Et = shaped_tmpl->energy;
    else if (sparse_tmpl && sparse_tmpl->code_energy > 0.0f)
        Et = sparse_tmpl->code_energy;
    const auto peak = rxopt::scan_normalized_sync(
        filter, folded.data(), static_cast<size_t>(P),
        window_power_8acc(folded.data(), L), Et);

    res.symbols_folded = fold_symbols;
    res.window_lags = static_cast<size_t>(P);
    if (!(peak.metric >= threshold))
        return res;
    res.ok = true;
    res.start = probe + static_cast<int64_t>(peak.lag);
    res.metric = peak.metric;
    res.corr = peak.corr;
    return res;
}

bool fold_anchor_fit(const std::complex<float>* rx,
                     size_t n,
                     int64_t start,
                     const std::complex<float>* tmpl,
                     size_t L,
                     float Et,
                     const rxopt::RealSyncTemplate& real_tmpl,
                     size_t groups,
                     double* period_out,
                     int64_t* preamble_start_out,
                     std::vector<int64_t>& peak_samples,
                     std::vector<float>& peak_metrics,
                     std::vector<std::complex<float>>& peak_corr,
                     double phase_per_symbol,
                     const rxopt::SparseCirTemplate* sparse_tmpl,
                     std::vector<std::complex<float>>* sup_buf,
                     const rxopt::ShapedSparseTemplate* shaped_tmpl)
{
    peak_samples.clear();
    peak_metrics.clear();
    peak_corr.clear();
    const int64_t P = static_cast<int64_t>(kQm35SamplesPerSymbol);
    const int64_t Li = static_cast<int64_t>(L);
    const int64_t refine = kFoldAnchorRefine;
    const size_t sup_len = L + 2 * static_cast<size_t>(refine);
    if (groups < 2 || L == 0 || !rx ||
        (!tmpl && !(sparse_tmpl && sparse_tmpl->code_energy > 0.0f) &&
         !(shaped_tmpl && shaped_tmpl->valid)))
        return false;

    std::vector<std::complex<float>> local_sup;
    std::vector<std::complex<float>>& sup = sup_buf ? *sup_buf : local_sup;
    if (sup.size() < sup_len)
        sup.resize(sup_len);
    const float* real_taps = real_tmpl.is_real && real_tmpl.length == L
        ? real_tmpl.taps_dup.data() : nullptr;
    const rxopt::SyncMatchedFilter filter(tmpl, L, real_taps, sparse_tmpl,
                                          true, shaped_tmpl);
    if (shaped_tmpl && shaped_tmpl->valid)
        Et = shaped_tmpl->energy;
    else if (sparse_tmpl && sparse_tmpl->code_energy > 0.0f)
        Et = sparse_tmpl->code_energy;

    LinearFit fit;
    size_t used = 0;
    for (size_t g = 0; g < groups; ++g) {
        const int64_t base =
            start + static_cast<int64_t>(g) *
                        static_cast<int64_t>(kFoldAnchorGroupSymbols) * P -
            refine;
        if (base < 0)
            continue;
        if (base + static_cast<int64_t>(sup_len) +
                static_cast<int64_t>(kFoldAnchorGroupSymbols - 1) * P >
            static_cast<int64_t>(n))
            continue;
        std::fill(sup.begin(), sup.end(), std::complex<float>(0.0f, 0.0f));
        for (int64_t j = 0; j < static_cast<int64_t>(kFoldAnchorGroupSymbols);
             ++j)
            fold_accumulate(sup.data(), rx + base + j * P, sup_len,
                            -j * phase_per_symbol);
        const auto peak = rxopt::scan_normalized_sync(
            filter, sup.data(), sup_len - L + 1,
            window_power_8acc(sup.data(), L), Et);
        const int64_t pos = base + static_cast<int64_t>(peak.lag);
        const double x = static_cast<double>(
            static_cast<int64_t>(g) *
            static_cast<int64_t>(kFoldAnchorGroupSymbols));
        const double y = static_cast<double>(pos + Li - 1);
        fit.add(x, y);
        peak_samples.push_back(pos + Li - 1);
        peak_metrics.push_back(peak.metric);
        peak_corr.push_back(peak.corr);
        ++used;
    }
    if (used < 2)
        return false;
    const double denom = fit.determinant();
    if (!(std::abs(denom) > 1e-9))
        return false;
    const double p = fit.slope(denom);
    if (!std::isfinite(p) || !(p > 0.0))
        return false;
    // Same 0.5% sanity gate the LockedBounded anchor fit uses: a period that
    // far off the nominal grid means the anchors did not land on the expected
    // symbol indices at all.
    if (std::abs(p - static_cast<double>(P)) > 0.005 * static_cast<double>(P))
        return false;
    const double intercept_end = fit.intercept(p);
    *period_out = p;
    *preamble_start_out = std::max<int64_t>(
        0, static_cast<int64_t>(std::llround(intercept_end)) - (Li - 1));
    return true;
}

} // namespace detail

} // namespace openuwb::demod::core

namespace openuwb::demod::rxopt {

void build_real_sync_template(const std::vector<std::complex<float>>& template_wf,
                              RealSyncTemplate& out)
{
    out.reset();
    if (template_wf.empty())
        return;
    const size_t L = template_wf.size();
    bool real_only = true;
    for (size_t k = 0; k < L; ++k) {
        if (std::abs(template_wf[k].imag()) > 1e-7f) {
            real_only = false;
            break;
        }
    }
    out.is_real = real_only;
    out.length = L;
    if (real_only) {
        out.taps_dup.resize(2 * L);
        float e = 0.0f;
        for (size_t k = 0; k < L; ++k) {
            const float r = template_wf[k].real();
            out.taps_dup[2 * k] = r;
            out.taps_dup[2 * k + 1] = r;
            e += r * r;
        }
        out.energy = e;
    }
}

uint64_t
sfd_template_key(const std::vector<std::complex<float>>& template_wf,
                 const std::vector<int8_t>& sfd_sequence)
{
    static_assert(sizeof(float) == 4, "SFD cache key assumes 32-bit float");
    uint64_t h = detail::kFnvOffsetBasis;
    h = detail::fnv1a_u64(h, static_cast<uint64_t>(template_wf.size()));
    for (const std::complex<float>& v : template_wf) {
        uint32_t re_bits = 0;
        uint32_t im_bits = 0;
        const float re = v.real();
        const float im = v.imag();
        std::memcpy(&re_bits, &re, sizeof(re_bits));
        std::memcpy(&im_bits, &im, sizeof(im_bits));
        h = detail::fnv1a_u32(h, re_bits);
        h = detail::fnv1a_u32(h, im_bits);
    }
    h = detail::fnv1a_u64(h, static_cast<uint64_t>(sfd_sequence.size()));
    if (!sfd_sequence.empty())
        h = detail::fnv1a_bytes(h, sfd_sequence.data(), sfd_sequence.size());
    return h;
}

void
build_sfd_template(const std::vector<std::complex<float>>& template_wf,
                   const std::vector<int8_t>& sfd_sequence,
                   std::vector<std::complex<float>>& out)
{
    out.clear();
    const size_t sym = kQm35SamplesPerSymbol;
    if (sfd_sequence.empty() || template_wf.size() < sym)
        return; // the legacy loop would read past the template: fail closed
    if (sfd_sequence.size() > std::numeric_limits<size_t>::max() / sym)
        return;
    out.resize(sfd_sequence.size() * sym);
    for (size_t i = 0; i < sfd_sequence.size(); ++i) {
        const int8_t s = sfd_sequence[i];
        for (size_t k = 0; k < sym; ++k)
            out[i * sym + k] =
                std::complex<float>(static_cast<float>(s) * template_wf[k].real(),
                                    static_cast<float>(s) * template_wf[k].imag());
    }
    openuwb::core::uwb_l2_normalize(out);
}

void vector_accumulate_complex(std::complex<float>* dst,
                               const std::complex<float>* src,
                               size_t count)
{
    if (count == 0)
        return;
#if UWB_SIMD_HAVE_AVX512
    if (use_avx512(openuwb::core::simd::KernelGroup::Sync)) {
        openuwb::core::simd::avx512::accumulate_cf32(dst, src, count);
        return;
    }
#endif
#if UWB_RXOPT_HAVE_AVX2
    if (has_avx2_fma()) {
        vector_accumulate_complex_avx2(dst, src, count);
        return;
    }
#endif
    for (size_t m = 0; m < count; ++m)
        dst[m] += src[m];
}

void vector_sub_complex(std::complex<float>* dst,
                        const std::complex<float>* src,
                        size_t count)
{
    if (count == 0)
        return;
#if UWB_SIMD_HAVE_AVX512
    if (use_avx512(openuwb::core::simd::KernelGroup::Sync)) {
        openuwb::core::simd::avx512::sub_cf32(dst, src, count);
        return;
    }
#endif
#if UWB_RXOPT_HAVE_AVX2
    if (has_avx2_fma()) {
        vector_sub_complex_avx2(dst, src, count);
        return;
    }
#endif
    for (size_t m = 0; m < count; ++m)
        dst[m] -= src[m];
}

void vector_sub3_complex(std::complex<float>* dst,
                         const std::complex<float>* a,
                         const std::complex<float>* b,
                         size_t count)
{
    if (count == 0)
        return;
#if UWB_SIMD_HAVE_AVX512
    if (use_avx512(openuwb::core::simd::KernelGroup::Sync)) {
        openuwb::core::simd::avx512::sub3_cf32(dst, a, b, count);
        return;
    }
#endif
    for (size_t m = 0; m < count; ++m)
        dst[m] = a[m] - b[m];
}

void vector_accumulate_scaled_complex(std::complex<float>* dst,
                                      const std::complex<float>* src,
                                      std::complex<float> alpha,
                                      size_t count)
{
    if (count == 0)
        return;
#if UWB_RXOPT_HAVE_AVX2
    if (has_avx2_fma()) {
        vector_accumulate_scaled_complex_avx2(dst, src, alpha.real(),
                                              alpha.imag(), count);
        return;
    }
#endif
    for (size_t j = 0; j < count; ++j)
        dst[j] += alpha * src[j];
}

void vector_mul_complex(std::complex<float>* dst,
                        const std::complex<float>* mul,
                        size_t count)
{
    if (count == 0)
        return;
#if UWB_RXOPT_HAVE_AVX2
    if (has_avx2_fma()) {
        vector_mul_complex_avx2(dst, mul, count);
        return;
    }
#endif
    for (size_t j = 0; j < count; ++j)
        dst[j] *= mul[j];
}

void vector_scale_complex(std::complex<float>* dst,
                          float scale,
                          size_t count)
{
    if (count == 0)
        return;
#if UWB_SIMD_HAVE_AVX512
    if (use_avx512(openuwb::core::simd::KernelGroup::Sync)) {
        openuwb::core::simd::avx512::scale_cf32(dst, scale, count);
        return;
    }
#endif
#if UWB_RXOPT_HAVE_AVX2
    if (has_avx2_fma()) {
        vector_scale_complex_avx2(dst, scale, count);
        return;
    }
#endif
    for (size_t m = 0; m < count; ++m)
        dst[m] *= scale;
}

void fold_sfd_window(const std::complex<float>* rx_base,
                     size_t sym,
                     const int8_t* sfd_seq,
                     size_t sfd_len,
                     size_t fold_len,
                     std::complex<float>* out)
{
    if (!rx_base || !sfd_seq || !out || sym == 0 || sfd_len == 0 || fold_len == 0)
        return;
#if UWB_SIMD_HAVE_AVX512
    if (use_avx512(openuwb::core::simd::KernelGroup::Sync)) {
        // Batched: the whole symbol loop runs inside the wide translation
        // unit, so the fold pays one call instead of one per symbol.
        openuwb::core::simd::avx512::fold_sfd_window(rx_base, sym, sfd_seq,
                                                     sfd_len, fold_len, out);
        return;
    }
#endif
#if UWB_RXOPT_HAVE_AVX2
    if (has_avx2_fma()) {
        fold_sfd_window_avx2(rx_base, sym, sfd_seq, sfd_len, fold_len, out);
        return;
    }
#endif
    size_t idx[8];
    int8_t sg[8];
    bool have_out = false;
    for (size_t chunk = 0; chunk < sfd_len; chunk += 8) {
        const size_t chunk_end = std::min(chunk + 8, sfd_len);
        size_t cnt = 0;
        for (size_t i = chunk; i < chunk_end; ++i) {
            if (sfd_seq[i] == 0)
                continue;
            idx[cnt] = i;
            sg[cnt] = sfd_seq[i];
            ++cnt;
        }
        if (cnt == 0)
            continue;
        const bool first_chunk = !have_out;
        for (size_t k = 0; k < fold_len; ++k) {
            size_t j = 0;
            std::complex<float> acc;
            if (first_chunk) {
                const std::complex<float> v = rx_base[idx[0] * sym + k];
                acc = (sg[0] == 1) ? v : std::complex<float>(0.0f, 0.0f) - v;
                j = 1;
            } else {
                acc = out[k];
            }
            for (; j < cnt; ++j) {
                const std::complex<float> v = rx_base[idx[j] * sym + k];
                acc = (sg[j] == 1) ? acc + v : acc - v;
            }
            out[k] = acc;
        }
        have_out = true;
    }
    if (!have_out)
        std::fill(out, out + fold_len, std::complex<float>(0.0f, 0.0f));
}

void build_rotation_ramp(double w, size_t count,
                         std::complex<float>* table)
{
    if (count == 0)
        return;
    constexpr size_t kB = 1024;
    const float s4r = static_cast<float>(std::cos(-4.0 * w));
    const float s4i = static_cast<float>(std::sin(-4.0 * w));
    for (size_t b0 = 0; b0 < count; b0 += kB) {
        const size_t b1 = std::min(count, b0 + kB);
        float cr[4], ci[4];
        for (int c = 0; c < 4; ++c) {
            const double ph = -w * static_cast<double>(b0 + c);
            cr[c] = static_cast<float>(std::cos(ph));
            ci[c] = static_cast<float>(std::sin(ph));
        }
        size_t i = b0;
        for (; i + 4 <= b1; i += 4) {
            for (int c = 0; c < 4; ++c) {
                table[i + c] = std::complex<float>(cr[c], ci[c]);
                const float nr = cr[c] * s4r - ci[c] * s4i;
                const float ni = cr[c] * s4i + ci[c] * s4r;
                cr[c] = nr;
                ci[c] = ni;
            }
        }
        for (; i < b1; ++i) {
            const double ph = -w * static_cast<double>(i);
            table[i] = std::complex<float>(static_cast<float>(std::cos(ph)),
                                           static_cast<float>(std::sin(ph)));
        }
    }
}

void build_sparse_cir_template(const int8_t* code,
                               size_t code_len,
                               SparseCirTemplate& tmpl)
{
    tmpl.reset();
    tmpl.key = cir_template_key(code, code_len);
    const std::vector<int8_t> spread = BuildSampledCode(code, code_len);
    tmpl.entries.reserve(code_len);
    for (size_t j = 0; j < spread.size(); ++j) {
        if (spread[j] != 0) {
            const size_t pos = 2 * j;
            tmpl.entries.push_back({ pos, spread[j] });
            if (spread[j] > 0) {
                tmpl.pos_offsets.push_back(2 * pos);
                if (tmpl.pos_count < kMaxSparseCirOffsets)
                    tmpl.pos_off[tmpl.pos_count++] =
                        static_cast<uint16_t>(2 * pos);
            } else {
                tmpl.neg_offsets.push_back(2 * pos);
                if (tmpl.neg_count < kMaxSparseCirOffsets)
                    tmpl.neg_off[tmpl.neg_count++] =
                        static_cast<uint16_t>(2 * pos);
            }
        }
    }
    tmpl.code_energy = static_cast<float>(tmpl.entries.size());
}

void build_half_rate_sparse_template(const SparseCirTemplate& full,
                                     SparseCirTemplate& half)
{
    half.reset();
    half.key = full.key ^ 0x9e3779b97f4a7c15ULL;
    half.code_energy = full.code_energy;
    half.entries.reserve(full.entries.size());
    for (const auto& e : full.entries)
        half.entries.push_back({ e.pos / 2, e.sign });
    half.pos_offsets.reserve(full.pos_offsets.size());
    for (const size_t off : full.pos_offsets)
        half.pos_offsets.push_back(off / 2);
    half.neg_offsets.reserve(full.neg_offsets.size());
    for (const size_t off : full.neg_offsets)
        half.neg_offsets.push_back(off / 2);
    half.pos_count = full.pos_count;
    for (size_t i = 0; i < full.pos_count; ++i)
        half.pos_off[i] = static_cast<uint16_t>(full.pos_off[i] / 2);
    half.neg_count = full.neg_count;
    for (size_t i = 0; i < full.neg_count; ++i)
        half.neg_off[i] = static_cast<uint16_t>(full.neg_off[i] / 2);
}

bool build_shaped_sparse_template(const std::complex<float>* tmpl,
                                  size_t len,
                                  const SparseCirTemplate& code,
                                  ShapedSparseTemplate& out)
{
    out.reset();
    if (!tmpl || len < 32 || code.entries.empty() ||
        code.entries.size() > kMaxSparseCirOffsets)
        return false;
    const SparseCirEntry& first = code.entries.front();
    const size_t base = first.pos;
    const size_t last = code.entries.back().pos;
    if (first.sign == 0 || base >= len || last >= len)
        return false;
    const size_t nt = std::min(ShapedSparseTemplate::kMaxPulse, len - last);
    if (nt < 3)
        return false;

    double energy = 0.0;
    double imag_energy = 0.0;
    for (size_t k = 0; k < len; ++k) {
        energy += static_cast<double>(std::norm(tmpl[k]));
        imag_energy += static_cast<double>(tmpl[k].imag()) *
                       static_cast<double>(tmpl[k].imag());
    }
    if (energy <= 0.0 || imag_energy > 1e-12 * energy)
        return false;

    // First pulse is uncontaminated up to the next chip (8 samples).
    const float sign0 = static_cast<float>(first.sign);
    for (size_t k = 0; k < nt; ++k)
        out.pulse[k] = tmpl[base + k].real() / sign0;

    // Reconstruction gate. The residual is the omitted pulse tail, which is
    // ~0.3% of the energy for kPulse48; anything above 5% is not code*p.
    double resid = 0.0;
    for (const auto& e : code.entries) {
        for (size_t k = 0; k < nt; ++k) {
            const size_t idx = e.pos + k;
            if (idx >= len)
                break;
            const double rec =
                static_cast<double>(e.sign) * out.pulse[k];
            const double d = rec - tmpl[idx].real();
            resid += d * d;
        }
    }
    if (!(resid <= 0.05 * energy))
        return false;

    out.code = code;
    out.nt = static_cast<uint8_t>(nt);
    out.energy = static_cast<float>(energy);
    out.key = shaped_template_key(tmpl, len) ^ code.key;
    out.valid = true;
    return true;
}

const ShapedSparseTemplate* ensure_shaped_sparse_template(
    const std::complex<float>* tmpl, size_t len,
    const SparseCirTemplate& code, ShapedSparseTemplate& out)
{
    const uint64_t key = shaped_template_key(tmpl, len) ^ code.key;
    if (out.valid && out.key == key)
        return &out;
    if (!build_shaped_sparse_template(tmpl, len, code, out))
        return nullptr;
    return &out;
}

MatchedFilterPeak scan_normalized_sync(const SyncMatchedFilter& filter,
                                       const std::complex<float>* rx,
                                       size_t lag_count,
                                       float power,
                                       float template_energy,
                                       float* metrics)
{
    MatchedFilterPeak peak;
    filter.for_each_lag(rx, lag_count, [&](size_t lag, std::complex<float> corr, bool) {
        const float metric =
            openuwb::core::normalized_correlation_power(corr, power, template_energy);
        if (metrics)
            metrics[lag] = metric;
        if (metric > peak.metric)
            peak = { lag, metric, corr };
        if (lag + 1 < lag_count)
            power += std::norm(rx[lag + filter.length()]) - std::norm(rx[lag]);
    });
    return peak;
}

MatchedFilterPeak scan_raw_sync(const SyncMatchedFilter& filter,
                                const std::complex<float>* rx,
                                size_t lag_count,
                                float* powers)
{
    MatchedFilterPeak peak;
    filter.for_each_lag(rx, lag_count, [&](size_t lag, std::complex<float> corr, bool) {
        const float power = std::norm(corr);
        powers[lag] = power;
        if (power > peak.metric)
            peak = { lag, power, corr };
    });
    return peak;
}

void correlate_cir_sparse_raw(const std::complex<float>* corr,
                              const SparseCirTemplate& tmpl,
                              size_t tap_count,
                              std::complex<float>* values)
{
    if (tap_count == 0)
        return;
    const SyncMatchedFilter filter(nullptr, 0, nullptr, &tmpl);
    filter.for_each_lag(corr, tap_count, [&](size_t lag, std::complex<float> acc, bool) {
        values[lag] = acc;
    });
}

void correlate_cir_sparse(const std::complex<float>* corr,
                          const SparseCirTemplate& tmpl,
                          size_t tap_count,
                          std::complex<float>* values)
{
    if (tap_count == 0 || tmpl.code_energy <= 0.0f)
        return;
    correlate_cir_sparse_raw(corr, tmpl, tap_count, values);
    for (size_t i = 0; i < tap_count; ++i)
        values[i] /= tmpl.code_energy;
}

void correlate_cir_sparse(const std::complex<float>* corr,
                          const SparseCirTemplate& tmpl,
                          size_t tap_count,
                          std::vector<std::complex<float>>& values)
{
    values.resize(tap_count);
    if (tmpl.code_energy <= 0.0f || tap_count == 0)
        return;
    correlate_cir_sparse(corr, tmpl, tap_count, values.data());
}

} // namespace openuwb::demod::rxopt
