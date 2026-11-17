// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <gnuradio/uwb/uwb_demod_sync.h>

namespace gr::uwb::demod::core {
// ---------------------------------------------------------------------------
// Stage 1 — seeded timing (R1).
// Uses the detector-supplied absolute seed_start (or, if -1, scans the whole
// buffer) to run a full-rate matched filter over a narrow ROI, then tracks
// the repeated SYNC peaks spaced ~samples_per_symbol apart, and fits a linear
// model to get measured_period.  Peak positions are symbol-END convention
// (first peak = start + (L-1)), matching the detector / MATLAB.
// ---------------------------------------------------------------------------
// Windowed |x|^2 with eight independent accumulators.  A single-accumulator
// float reduction cannot be vectorised without -ffast-math, and the power
// turned out to be HALF the cost of the single-point stage (measured 2x faster
// with eight accumulators).  The summation order differs from a plain scalar
// loop, but this value only normalises a correlation metric.
inline bool stage_timing(const std::complex<float>* rx,
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

    // Copy + L2-normalize the template (own copy so callers may pass raw).
    scratch.work.assign(template_wf.begin(), template_wf.end());
    gr::uwb::core::uwb_l2_normalize(scratch.work);
    const float Et = gr::uwb::core::uwb_template_energy(scratch.work);
    rxopt::build_real_sync_template(scratch.work, scratch.real_sync_template);

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
        float lag_metrics[1024];
        float bm = -1.0f;
        int64_t bp = w0;
        std::complex<float> best_acc(0.0f, 0.0f);
        // Adjacent candidate windows overlap by L-1 samples.  Compute power
        // once, then update it in O(1); correlation remains an exact long dot
        // product handled by VOLK or AVX2.  No allocation occurs in this hot loop.
        // The initial power uses the 8-accumulator reduction: for the narrow
        // windows (17 lags) this 1016-sample sum is a large fraction of the
        // whole call, and the legacy per-SYNC path keeps its own O(1) sliding
        // update untouched.
        float pw = window_power_8acc(rx + w0, L);

        const bool use_real_opt = scratch.real_sync_template.is_real;
        const float* taps_dup = scratch.real_sync_template.taps_dup.data();

        int64_t j = w0;
        if (use_real_opt) {
            // Eight lags per call: the AVX-512 backend pairs lag m with lag
            // m+4 in one 512-bit register (-37% per lag, bit-identical).  The
            // metric/power update order below is unchanged, so the selected
            // peak is identical to the four-lag loop it replaces.
            for (; j + 7 < w1; j += 8) {
                std::complex<float> acc[8];
                rxopt::real_template_dot_8lag(rx + j, taps_dup, L, acc);
                for (int m = 0; m < 8; ++m) {
                    const float metric = std::norm(acc[m]) / (pw * Et + 1e-12f);
                    if (track_par)
                        lag_metrics[(j + m) - w0] = metric;
                    if (metric > bm) {
                        bm = metric;
                        bp = j + m;
                        best_acc = acc[m];
                    }
                    if (j + m + 1 < w1) {
                        pw += std::norm(rx[j + m + static_cast<int64_t>(L)]) -
                              std::norm(rx[j + m]);
                    }
                }
            }
            for (; j < w1; ++j) {
                const std::complex<float> acc =
                    rxopt::real_template_dot_single(rx + j, taps_dup, L);
                const float metric = std::norm(acc) / (pw * Et + 1e-12f);
                if (track_par)
                    lag_metrics[j - w0] = metric;
                if (metric > bm) {
                    bm = metric;
                    bp = j;
                    best_acc = acc;
                }
                if (j + 1 < w1) {
                    pw += std::norm(rx[j + static_cast<int64_t>(L)]) -
                          std::norm(rx[j]);
                }
            }
        } else {
            for (; j < w1; ++j) {
                const std::complex<float> acc =
                    detail::timing_conjugate_dot(rx + j, scratch.work.data(), L);
                const float metric = std::norm(acc) / (pw * Et + 1e-12f);
                if (track_par)
                    lag_metrics[j - w0] = metric;
                if (metric > bm) {
                    bm = metric;
                    bp = j;
                    best_acc = acc;
                }
                if (j + 1 < w1) {
                    pw += std::norm(rx[j + static_cast<int64_t>(L)]) -
                          std::norm(rx[j]);
                }
            }
        }
        // Report the window's raw maximum before the acceptance gate: the
        // low-SNR early-stop decisions below need to know how far the best
        // candidate was from the threshold, not just whether it passed.
        if (raw_max_out)
            *raw_max_out = bm;
        if (bm < min_metric) {
            constexpr float kSyncParFloor = 0.06f;
            constexpr float kSyncParMin = 20.0f;
            constexpr int64_t kSyncParGuard = 12;
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
        *corr_out = best_acc;
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
        const detail::FoldAcquireResult fold = detail::fold_acquire(
            rx, n, centre, centre - radius, centre + radius,
            scratch.work.data(), L, Et, scratch.real_sync_template,
            detail::kFoldMaxSymbols, profile.preamble_repetitions,
            detail::kFoldThreshold);
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
                const std::complex<float> acc =
                    scratch.real_sync_template.is_real
                        ? rxopt::real_template_dot_single(
                              rx + pos,
                              scratch.real_sync_template.taps_dup.data(), L)
                        : detail::timing_conjugate_dot(rx + pos,
                                                       scratch.work.data(), L);
                return std::norm(acc) / (pw * Et + 1e-12f);
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
                    out.peak_corr)) {
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
            if (local_best(want, inner_rad, thr, &pos, &met, &corr,
                           /*allow_par_fallback=*/true, &raw_max)) {
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

        // (a) Coarse scan (fallback only) -- sliding power, VOLK-grade
        // conjugate dots at stride.
        const auto coarse_begin = std::chrono::steady_clock::now();
        if (!seed_acquired && j_hi > j_lo) {
            float pw = window_power_8acc(rx + j_lo, L);
            float best_coarse_m = -1.0f;
            int64_t best_coarse_j = j_lo;
            std::complex<float> best_coarse_c(0.0f, 0.0f);
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
                const std::complex<float> acc =
                    scratch.real_sync_template.is_real
                        ? rxopt::real_template_dot_single(
                              rx + j, scratch.real_sync_template.taps_dup.data(), L)
                        : detail::timing_conjugate_dot(
                              rx + j, scratch.work.data(), L);
                const float m = std::norm(acc) / (pw * Et + 1e-12f);
                if (m > best_coarse_m) {
                    best_coarse_m = m;
                    best_coarse_j = j;
                    best_coarse_c = acc;
                }
                const int64_t cnt = std::min<int64_t>(stride, j_hi + 1 - j);
                advance_power(j, cnt);
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
    // Full->Bypass (grid_reject) protection for an untrimmed clock.
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
            double sx = 0.0, sy = 0.0, sxx = 0.0, sxy = 0.0;
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
                sx += x;
                sy += y;
                sxx += x * x;
                sxy += x * y;
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
            const double denom = static_cast<double>(hits) * sxx - sx * sx;
            if (!(std::abs(denom) > 1e-9))
                return false;
            const double p =
                (static_cast<double>(hits) * sxy - sx * sy) / denom;
            if (!std::isfinite(p) || !(p > 0.0))
                return false;
            const double intercept_end =
                (sy - p * sx) / static_cast<double>(hits);
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
        const bool use_real = scratch.real_sync_template.is_real;
        const float* taps_dup = scratch.real_sync_template.taps_dup.data();
        scratch.metric.assign(reps, 0.0f);
        size_t valid = 0;
        // Least-squares accumulators over every refined peak (see the final
        // fit note after the loop).
        double fsx = 0.0, fsy = 0.0, fsxx = 0.0, fsxy = 0.0;
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
                    fsx += x;
                    fsy += y;
                    fsxx += x * x;
                    fsxy += x * y;
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
            std::complex<float> acc[8];
            if (use_real) {
                rxopt::real_template_dot_8lag(rx + base, taps_dup, L, acc);
            } else {
                for (int m = 0; m < nl; ++m)
                    acc[m] = detail::timing_conjugate_dot(
                        rx + base + m, scratch.work.data(), L);
            }
            int best_m = 0;
            float best = std::norm(acc[0]) / (pw * Et + 1e-12f);
            for (int m = 1; m < nl; ++m) {
                pw += std::norm(rx[base + m - 1 + static_cast<int64_t>(L)]) -
                      std::norm(rx[base + m - 1]);
                const float met = std::norm(acc[m]) / (pw * Et + 1e-12f);
                if (met > best) {
                    best = met;
                    best_m = m;
                }
            }
            const int64_t want = base + best_m;
            const float m = best;
            scratch.metric[k] = m;
            ++out.grid_single_points;
            if (m < gthr)
                continue;
            ++valid;
            out.peak_samples.push_back(want + static_cast<int64_t>(L - 1));
            out.peak_metrics.push_back(m);
            out.peak_corr.push_back(acc[best_m]);
            const double x = static_cast<double>(k);
            const double y = static_cast<double>(want + (int64_t)(L - 1));
            fsx += x;
            fsy += y;
            fsxx += x * x;
            fsxy += x * y;
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
            const double denom = static_cast<double>(fn) * fsxx - fsx * fsx;
            if (std::abs(denom) > 1e-9) {
                const double pr =
                    (static_cast<double>(fn) * fsxy - fsx * fsy) / denom;
                const double intercept_end =
                    (fsy - pr * fsx) / static_cast<double>(fn);
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
    if (allow_post_train_fold && !fold_used && !grid_applied &&
        out.peak_samples.size() <
            std::max<size_t>(4, profile.min_valid_peaks)) {
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

    // The fold path measures `groups` super-anchors rather than every SYNC, but
    // stage_cir_softchips derives its coherent-averaging count from
    // detected_peaks: reporting `groups` (8 for 128 SYNCs, 16 for 256) would
    // average 8 SYNCs instead of 54 and lose ~8 dB of CIR SNR.  Report the
    // number of SYNC symbols the fitted grid actually covers.
    out.detected_peaks = fold_used ? profile.preamble_repetitions
                                   : out.peak_samples.size();
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
        double sx = 0.0, sy = 0.0, sxx = 0.0, sxy = 0.0;
        for (size_t k = 0; k < np; ++k) {
            const double x = static_cast<double>(peak_indices[k]);
            const double y = static_cast<double>(out.peak_samples[k]);
            sx += x;
            sy += y;
            sxx += x * x;
            sxy += x * y;
        }
        const double denom = static_cast<double>(np) * sxx - sx * sx;
        if (std::abs(denom) > 1e-12) {
            out.measured_period = (static_cast<double>(np) * sxy - sx * sy) /
                                  denom;
            // peak_samples use the symbol-END convention (start + L - 1), so
            // the intercept is also symbol-END; convert back to the SYNC start.
            const double intercept_end = (sy - out.measured_period * sx) / np;
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

} // namespace gr::uwb::demod::core
