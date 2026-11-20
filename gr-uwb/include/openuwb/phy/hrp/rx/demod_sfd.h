// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <openuwb/phy/hrp/rx/demod_sync.h>
#include <functional>

namespace gr::uwb::demod::core {
// ---------------------------------------------------------------------------
// Stage 3 — SFD timing refinement (R1).
// Selects the best SFD template (from the profile; golden uses IEEE legacy
// SFD #0 = {0,1,0,-1,1,0,0,-1}) and refines the preamble start at full rate.
// For Phase-1 the SFD search is done on the symbol grid: the SFD starts one
// symbol after the last SYNC (i.e. after preamble_repetitions symbols).
// We locate the SFD by correlating the SFD template over a narrow window
// around the expected position, then refine the preamble start.
// ---------------------------------------------------------------------------

inline bool stage_sfd(const std::complex<float>* rx,
                      size_t n,
                      const Qm35825Profile& profile,
                      const TimingResult& timing,
                      const std::vector<int8_t>& sfd_sequence,
                      const std::vector<std::complex<float>>& template_wf,
                      SfdResult& out,
                      DemodScratch& scratch,
                      const std::function<void()>& prepare_grid = {})
{
    out = SfdResult{};
    if (sfd_sequence.empty() || timing.peak_samples.empty() ||
        template_wf.empty())
        return false;
    const size_t sym = kQm35SamplesPerSymbol;

    // Build (or reuse) the SFD template: kron(sfd_sequence, preamble_waveform),
    // then L2-normalize — same as MATLAB refineTimingWithNsSfd.  The fast path
    // caches the normalized template in scratch across packets; the legacy
    // path keeps constructing it in scratch.corr exactly as before.
    const size_t len = sfd_sequence.size();
    const std::complex<float>* sfd_tmpl = nullptr;
    size_t sfd_len = len * sym;
    float sfd_scale = 1.0f;

    const auto* sparse_tmpl = detail::prepare_sparse_sync_template(profile, sym, scratch);
    if (!sparse_tmpl && template_wf.size() < sym)
        return false;
    if (!sparse_tmpl && profile.rx_fast_sfd &&
        (!scratch.real_sync_template.is_real || scratch.real_sync_template.length != sym)) {
        rxopt::build_real_sync_template(template_wf, scratch.real_sync_template);
    }
    const bool use_real_sfd_fast =
        sparse_tmpl || (profile.rx_fast_sfd && scratch.real_sync_template.is_real);
    if (sparse_tmpl || profile.rx_fast_sfd) {
        float sfd_norm_sq = 0.0f;
        for (int8_t s : sfd_sequence)
            sfd_norm_sq += static_cast<float>(s * s);
        const float energy = sparse_tmpl ? sparse_tmpl->code_energy
                                         : scratch.real_sync_template.energy;
        if (sfd_norm_sq > 0.0f && energy > 0.0f)
            sfd_scale = 1.0f / std::sqrt(sfd_norm_sq * energy);
    }

    if (!use_real_sfd_fast) {
        if (profile.rx_fast_sfd) {
            const uint64_t key =
                rxopt::sfd_template_key(template_wf, sfd_sequence);
            if (key != scratch.sfd_template_key ||
                scratch.sfd_template.size() != len * sym) {
                rxopt::build_sfd_template(template_wf, sfd_sequence,
                                          scratch.sfd_template);
                scratch.sfd_template_key =
                    (scratch.sfd_template.size() == len * sym) ? key : 0;
            }
            sfd_tmpl = scratch.sfd_template.data();
            sfd_len = scratch.sfd_template.size();
        } else {
            rxopt::build_sfd_template(template_wf, sfd_sequence, scratch.corr);
            sfd_tmpl = scratch.corr.data();
            sfd_len = scratch.corr.size();
        }
        if (sfd_len == 0)
            return false;
    }

    // The fallback uses the full complex SFD template. The fast search
    // correlates a folded window against a single SYNC symbol instead.
    auto sfd_dot = [&](size_t j) {
        return profile.rx_fast_sfd
            ? rxopt::conjugate_dot(rx + j, sfd_tmpl, sfd_len)
            : detail::conjugate_dot_scalar(rx + j, sfd_tmpl, sfd_len);
    };

    // MATLAB strategy (refineTimingWithNsSfd): SFD start = preamble start +
    // preamble_repetitions × measured_period.  This is physically correct even
    // when the trailing preamble SYNC(s) score below the acquisition threshold
    // and are not tracked (partial train).
    // Live search: ±64-sample narrow window at that center, then the same
    // narrow window at ±1..±4 symbol offsets if the nominal center misses.
    const double period = timing.measured_period > 0.0
        ? timing.measured_period : static_cast<double>(sym);
    const int64_t expected = timing.preamble_start_sample +
        std::llround(static_cast<double>(profile.preamble_repetitions) * period);
    out.expected_start_sample = expected;
    // Score every sample in each candidate window. The fast path folds
    // the signed SFD symbols before the dot, while power covers the raw SFD.
    constexpr auto kSfdNarrowHalf = rxopt::kSfdNarrowHalf;
    constexpr auto kSfdSymbolOffsets = rxopt::kSfdSymbolOffsets;
    float best = -1.0f;
    size_t best_j = 0;
    const bool par_only = profile.rx_sfd_par_only;

    const float* real_taps = use_real_sfd_fast && !sparse_tmpl
        ? scratch.real_sync_template.taps_dup.data() : nullptr;
    const rxopt::SyncMatchedFilter sync_filter(
        template_wf.data(), sym, real_taps, sparse_tmpl, /*volk_complex=*/false);

    // Coherent sign check over the first min(len,16) SFD symbols.  This is the
    // hard lock that keeps a low-floor / discrete-grid hit from being a
    // shifted-SFD sidelobe or a noise coincidence: 8 random symbols all match
    // a fixed +-1 pattern with probability 2^-8.  Shared by the existing
    // 0.02..0.08 low-floor path and the new discrete grid.
    auto polarity_ok = [&](int64_t pos) -> bool {
        const size_t n_check = std::min<size_t>(len, 16);
        if (pos < 0 ||
            pos + static_cast<int64_t>(n_check * sym) >
                static_cast<int64_t>(n))
            return false;
        std::complex<float> sym_corr[16];
        std::complex<double> coh_ref(0.0, 0.0);
        for (size_t si = 0; si < n_check; ++si) {
            const size_t p = static_cast<size_t>(pos) + si * sym;
            const auto c = sync_filter.dot(rx + p);
            sym_corr[si] = c;
            coh_ref += static_cast<double>(sfd_sequence[si]) *
                       std::complex<double>(c);
        }
        if (!(std::norm(coh_ref) > 1e-24))
            return false;
        for (size_t si = 0; si < n_check; ++si) {
            if (sfd_sequence[si] == 0)
                continue;
            const double proj =
                static_cast<double>(sym_corr[si].real()) * coh_ref.real() +
                static_cast<double>(sym_corr[si].imag()) * coh_ref.imag();
            if (!(proj * static_cast<double>(sfd_sequence[si]) > 0.0))
                return false;
        }
        return true;
    };

    auto scan_window = [&](int64_t center, int64_t half_width,
                           int64_t backtrack_symbols, double raw_par_min = 25.0) {
        if (center < 0 || n < sfd_len)
            return false;
        const int64_t lo = std::max<int64_t>(0, center - half_width);
        const int64_t hi = std::min<int64_t>(
            static_cast<int64_t>(n - sfd_len), center + half_width);
        if (hi < lo)
            return false;
        ++out.search_windows;
        const size_t j_lo = static_cast<size_t>(lo);
        const size_t j_end = static_cast<size_t>(hi);
        float window_best = -1.0f;
        size_t window_best_j = j_lo;
        float pwr = par_only ? 0.0f : detail::sample_power_sum(rx + j_lo, sfd_len);
        const size_t lag_count = j_end - j_lo + 1;
        scratch.metric.resize(lag_count);
        auto score_lag = [&](size_t lag, std::complex<float> corr, bool coarse) {
            const size_t j = j_lo + lag;
            if (!par_only && lag > 0)
                pwr += std::norm(rx[j + sfd_len - 1]) - std::norm(rx[j - 1]);
            if (coarse)
                ++out.coarse_correlations;
            else
                ++out.fine_correlations;
            const float metric = par_only ? std::norm(corr)
                : detail::normalized_correlation_power(corr, pwr);
            scratch.metric[lag] = metric;
            if (metric > window_best) {
                window_best = metric;
                window_best_j = j;
            }
        };
        if (use_real_sfd_fast) {
            const size_t fold_len = lag_count - 1 + sym;
            if (scratch.sfd_folded.size() < fold_len)
                scratch.sfd_folded.resize(fold_len);
            rxopt::fold_sfd_window(rx + j_lo, sym, sfd_sequence.data(), len,
                                   fold_len, scratch.sfd_folded.data());
            sync_filter.for_each_lag(
                scratch.sfd_folded.data(), lag_count,
                [&](size_t lag, std::complex<float> corr, bool batch) {
                    score_lag(lag, par_only ? corr : corr * sfd_scale, batch);
                });
        } else {
            for (size_t lag = 0; lag < lag_count; ++lag)
                score_lag(lag, sfd_dot(j_lo + lag), (lag & 1u) == 0u);
        }
        if (out.fine_correlations == 0)
            ++out.fine_correlations;
        bool window_passed = !par_only && (window_best >= profile.sfd_detection_threshold);
        if (par_only) {
            const rxopt::MatchedFilterPeak peak{window_best_j - j_lo, window_best, {}};
            window_passed = rxopt::raw_lag_par_passes(
                scratch.metric.data(), lag_count, peak, 12, 16, raw_par_min,
                profile.rx_sfd_par_three_samples) &&
                polarity_ok(static_cast<int64_t>(window_best_j));
        } else if (!window_passed && window_best >= 0.02f &&
                   scratch.metric.size() == (j_end - j_lo + 1)) {
            const size_t best_off = window_best_j - j_lo;
            double bg_sum = 0.0;
            size_t bg_cnt = 0;
            for (size_t idx = 0; idx < scratch.metric.size(); ++idx) {
                if (idx + 12 < best_off || idx > best_off + 12) {
                    bg_sum += static_cast<double>(scratch.metric[idx]);
                    ++bg_cnt;
                }
            }
            if (bg_cnt >= 16) {
                const double bg_mean = bg_sum / static_cast<double>(bg_cnt);
                const double par =
                    static_cast<double>(window_best) / (bg_mean + 1e-12);
                if (par >= 25.0) {
                    if (window_best >= 0.08f) {
                        window_passed = true;
                    } else if (polarity_ok(static_cast<int64_t>(window_best_j))) {
                        // Low-floor fallback (0.02 <= pk < 0.08): verify that
                        // all SFD symbols coherently match sfd_sequence signs
                        // to reject shifted-SFD sidelobes (e.g. 0.0625).
                        window_passed = true;
                    }
                }
            }
        }
        if (window_passed) {
            best = window_best;
            best_j = window_best_j;
        }
        if (out.first_threshold_backtrack_symbols < 0 && window_passed) {
            out.first_threshold_backtrack_symbols = backtrack_symbols;
        }
        return window_passed;
    };

    bool sfd_passed = scan_window(expected, kSfdNarrowHalf, 0);
    // A weak centre window says nothing about an offset window. Retain full
    // sample resolution here for errors with both symbol and sample offsets.
    for (int k = 1; k <= kSfdSymbolOffsets && !sfd_passed; ++k) {
        const int64_t step = std::llround(static_cast<double>(k) * period);
        sfd_passed = scan_window(expected + step, kSfdNarrowHalf, k);
        if (sfd_passed)
            break;
        sfd_passed = scan_window(expected - step, kSfdNarrowHalf, k);
    }

    // ---- Low-SNR discrete integer-symbol grid fallback --------------------
    // Reached ONLY when the fast path above failed, so high and medium SNR
    // (and multipath) keep the existing search bit-for-bit.  At -10..-14 dB
    // in-band the SFD peak is 0.008..0.05, below the 0.02 low-floor gate, and
    // the origin can be several SYMBOLS off (the low-SNR timing fold resolves
    // the grid phase but not always the absolute SYNC index).  Scan the
    // integer symbol grid +-36 symbols at delta = 0, refine +-1 sample around
    // the best grid point, then gate on: peak >= 0.008, a weak discrete-grid
    // PAR >= 1.5, and the shared 8-symbol coherent polarity check.
    //
    // The PAR gate is deliberately weak (the plan suggested 5.5).  Measured on
    // this fixture the true SFD peak is only ~2x the discrete-grid background,
    // because the background grid points sit on SYNC symbols and the 4z2
    // sequence (sum = -4) correlates with a plain SYNC at 1/4 power, not at
    // zero.  A 5.5 gate would therefore reject the -14 dB peak it is meant to
    // accept.  The real false-alarm guard is the 8-symbol polarity lock
    // (2^-8 per candidate) plus the 0.008 floor, which is 15x the pure-noise
    // maximum (0.00053) measured over 200 windows.
    if (!sfd_passed) {
        if (prepare_grid)
            prepare_grid();
        constexpr auto kGridHalf = rxopt::kSfdGridHalf;
        const int64_t max_pos =
            static_cast<int64_t>(n) - static_cast<int64_t>(sfd_len);
        // Per-SYMBOL decomposition of the SFD correlation.  SFD =
        // kron(sfd_seq, sync), so a window at `pos` is the sfd_seq-weighted sum
        // of `len` single-SYMBOL correlations, and its power is the sum of the
        // same `len` single-SYMBOL powers.
        //
        // The grid spacing is `period`, which need not equal `sym`: with SFO
        // pos = expected + round(k*period), so the symbol offset relative to
        // the nominal k*sym grid differs by delta_k = round(k*period)-k*sym.
        // delta_k takes only a handful of integer values (<= 2 for +27 ppm
        // over +-36 symbols), so one symbol table per delta covers the whole
        // grid.  The old code disabled this optimisation whenever
        // round(kGridHalf*(period-sym)) != 0, i.e. exactly on an untrimmed
        // clock; the per-delta cache keeps it enabled with identical numerics.
        const size_t n_sym_slots = static_cast<size_t>(2 * kGridHalf) + len;
        std::array<std::vector<std::complex<float>>, 8> sym_corr;
        std::array<std::vector<float>, 8> sym_pwr;
        std::array<std::vector<uint8_t>, 8> sym_valid;
        std::array<int64_t, 8> delta_value{};
        size_t n_delta = 0;
        auto delta_index = [&](int64_t k) -> int {
            const int64_t d = std::llround(static_cast<double>(k) * period) -
                              k * static_cast<int64_t>(sym);
            for (size_t i = 0; i < n_delta; ++i)
                if (delta_value[i] == d)
                    return static_cast<int>(i);
            if (n_delta >= delta_value.size())
                return -1;
            delta_value[n_delta] = d;
            return static_cast<int>(n_delta++);
        };
        auto ensure_sym_cache = [&](size_t d) {
            if (!sym_valid[d].empty())
                return;
            sym_corr[d].assign(n_sym_slots, std::complex<float>(0.0f, 0.0f));
            if (!par_only)
                sym_pwr[d].assign(n_sym_slots, 0.0f);
            sym_valid[d].assign(n_sym_slots, 0);
            for (size_t i = 0; i < n_sym_slots; ++i) {
                const int64_t m = static_cast<int64_t>(i) -
                                  static_cast<int64_t>(kGridHalf);
                const int64_t spos = expected + delta_value[d] +
                                     m * static_cast<int64_t>(sym);
                if (spos < 0 ||
                    spos + static_cast<int64_t>(sym) >
                        static_cast<int64_t>(n))
                    continue;
                sym_corr[d][i] = sync_filter.dot(rx + spos);
                if (!par_only)
                    sym_pwr[d][i] = window_power_8acc(rx + spos, sym);
                sym_valid[d][i] = 1;
            }
        };
        auto sfd_metric_at = [&](int64_t pos) -> float {
            if (pos < 0 || pos > max_pos)
                return -1.0f;
            if (use_real_sfd_fast) {
                // Same summation order as sfd_real_dot_single(), but the power
                // uses the 8-accumulator window_power_8acc instead of a scalar loop.
                float pwr = 0.0f;
                std::complex<float> acc(0.0f, 0.0f);
                for (size_t s = 0; s < len; ++s) {
                    const int64_t p = pos + static_cast<int64_t>(s) *
                                                static_cast<int64_t>(sym);
                    if (!par_only)
                        pwr += window_power_8acc(rx + p, sym);
                    const int8_t sg = sfd_sequence[s];
                    if (sg == 0)
                        continue;
                    const auto d = sync_filter.dot(rx + p);
                    if (sg > 0)
                        acc += d;
                    else
                        acc -= d;
                }
                if (par_only)
                    return std::norm(acc);
                acc *= sfd_scale;
                return detail::normalized_correlation_power(acc, pwr);
            }
            const std::complex<float> acc = sfd_dot(static_cast<size_t>(pos));
            if (par_only)
                return std::norm(acc);
            const float pwr = window_power_8acc(rx + pos, sfd_len);
            return detail::normalized_correlation_power(acc, pwr);
        };
        struct GridCand {
            int64_t pos;
            float metric;
        };
        std::array<GridCand, 2 * kGridHalf + 1> cands{};
        std::array<int8_t, 2 * kGridHalf + 1> cand_delta{};
        cand_delta.fill(-1);
        size_t nc = 0;
        if (max_pos >= 0) {
            for (int64_t k = -kGridHalf; k <= kGridHalf; ++k) {
                const int64_t pos = expected + std::llround(static_cast<double>(k) * period);
                if (pos < 0 || pos > max_pos)
                    continue;
                int d = -1;
                if (use_real_sfd_fast) {
                    d = delta_index(k);
                }
                if (d >= 0) {
                    ensure_sym_cache(static_cast<size_t>(d));
                    const size_t base = static_cast<size_t>(k + kGridHalf);
                    bool ok = true;
                    float pwr = 0.0f;
                    std::complex<float> acc_unscaled(0.0f, 0.0f);
                    for (size_t s = 0; s < len; ++s) {
                        const size_t idx = base + s;
                        if (!sym_valid[d][idx]) {
                            ok = false;
                            break;
                        }
                        if (!par_only)
                            pwr += sym_pwr[d][idx];
                        const int8_t sg = sfd_sequence[s];
                        if (sg == 0)
                            continue;
                        if (sg > 0)
                            acc_unscaled += sym_corr[d][idx];
                        else
                            acc_unscaled -= sym_corr[d][idx];
                    }
                    if (!ok)
                        continue;
                    const float metric = par_only ? std::norm(acc_unscaled)
                        : detail::normalized_correlation_power(acc_unscaled * sfd_scale, pwr);
                    cand_delta[nc] = static_cast<int8_t>(d);
                    cands[nc++] = { pos, metric };
                } else {
                    cands[nc++] = { pos, sfd_metric_at(pos) };
                }
            }
        }
        // Reuse the per-symbol cache for the polarity check: when the winner is
        // exactly the best grid point, its `len` symbol correlations are already
        // in sym_corr[d], so the 8-16 dot products of polarity_ok() are free.
        auto cache_complete_for = [&](int d, int64_t k) -> bool {
            if (d < 0)
                return false;
            const size_t n_check = std::min<size_t>(len, 16);
            const int64_t base = k + kGridHalf;
            if (base < 0 ||
                static_cast<size_t>(base) + n_check > n_sym_slots)
                return false;
            for (size_t si = 0; si < n_check; ++si)
                if (!sym_valid[d][static_cast<size_t>(base) + si])
                    return false;
            return true;
        };
        auto polarity_ok_cached = [&](int d, int64_t k) -> bool {
            const size_t n_check = std::min<size_t>(len, 16);
            const size_t base = static_cast<size_t>(k + kGridHalf);
            std::complex<float> sym_corr_local[16];
            std::complex<double> coh_ref(0.0, 0.0);
            for (size_t si = 0; si < n_check; ++si) {
                sym_corr_local[si] = sym_corr[d][base + si];
                coh_ref += static_cast<double>(sfd_sequence[si]) *
                           std::complex<double>(sym_corr_local[si]);
            }
            if (!(std::norm(coh_ref) > 1e-24))
                return false;
            for (size_t si = 0; si < n_check; ++si) {
                if (sfd_sequence[si] == 0)
                    continue;
                const double proj =
                    static_cast<double>(sym_corr_local[si].real()) *
                        coh_ref.real() +
                    static_cast<double>(sym_corr_local[si].imag()) *
                        coh_ref.imag();
                if (!(proj * static_cast<double>(sfd_sequence[si]) > 0.0))
                    return false;
            }
            return true;
        };
        if (nc >= 8) {
            ++out.search_windows;
            size_t bi = 0;
            for (size_t i = 1; i < nc; ++i)
                if (cands[i].metric > cands[bi].metric)
                    bi = i;
            int64_t gpos = cands[bi].pos;
            // Refine around the best grid point.  The grid already evaluated
            // delta = 0, so only the two neighbours are scored; refining only
            // the winner keeps the noise maximum at 75 evaluations.
            constexpr int64_t refine_rad = 1;
            int64_t best_pos = gpos;
            float best_metric = cands[bi].metric;
            for (int64_t d = -refine_rad; d <= refine_rad; ++d) {
                if (d == 0)
                    continue;
                const int64_t pos = gpos + d;
                if (pos < 0 || pos > max_pos)
                    continue;
                const float m = sfd_metric_at(pos);
                if (m > best_metric) {
                    best_metric = m;
                    best_pos = pos;
                }
            }
            // Background mean over the discrete grid, excluding the best grid
            // point and its +-2 symbol neighbours.
            double bg_sum = 0.0;
            size_t bg_cnt = 0;
            for (size_t i = 0; i < nc; ++i) {
                if (std::llabs(cands[i].pos - gpos) <=
                    std::llround(2.0 * period))
                    continue;
                bg_sum += static_cast<double>(cands[i].metric);
                ++bg_cnt;
            }
            if (bg_cnt >= 16) {
                const bool passes_gate = par_only
                    ? rxopt::raw_par_passes(best_metric, bg_sum, bg_cnt, kSfdGridParFloor)
                    : best_metric >= kSfdGridPeakFloor &&
                          static_cast<double>(best_metric) /
                              std::max(bg_sum / static_cast<double>(bg_cnt), 1e-12) >=
                              kSfdGridParFloor;
                if (passes_gate && par_only && profile.rx_par_confirm) {
                    // On the symbol grid an ordinary SYNC correlates with
                    // 4z2, so its PAR threshold must stay low. Confirm on
                    // off-peak sample lags to distinguish SFD from noise.
                    sfd_passed = scan_window(best_pos, kSfdNarrowHalf, -1, 20.0);
                } else if (passes_gate) {
                    const int64_t g_k = std::llround(
                        static_cast<double>(gpos - expected) / period);
                    const int g_delta = cand_delta[bi];
                    const bool reuse = best_pos == gpos && g_delta >= 0 &&
                                       cache_complete_for(g_delta, g_k);
                    const bool pol_ok = reuse
                        ? polarity_ok_cached(g_delta, g_k)
                        : polarity_ok(best_pos);
                    if (pol_ok) {
                        best = best_metric;
                        best_j = static_cast<size_t>(best_pos);
                        sfd_passed = true;
                    }
                }
            }
        }
    }
    if (!sfd_passed)
        return false;

    out.ok = true;
    out.sfd_mode = profile.sfd_mode;
    out.sfd_start_sample = static_cast<int64_t>(best_j);
    out.sfd_end_sample = static_cast<int64_t>(best_j) +
                         static_cast<int64_t>(sfd_len) - 1;
    // Keep the reported metric in normalized power units.
    if (par_only) {
        if (use_real_sfd_fast)
            best *= sfd_scale * sfd_scale;
        const float power = window_power_8acc(rx + best_j, sfd_len);
        best /= power + gr::uwb::core::kUwbEpsilon;
    }
    out.metric = best;
    out.polarity = 1;
    return true;
}

} // namespace gr::uwb::demod::core
