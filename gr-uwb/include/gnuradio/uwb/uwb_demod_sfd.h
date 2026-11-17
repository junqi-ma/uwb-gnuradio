// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <gnuradio/uwb/uwb_demod_sync.h>

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
                      DemodScratch& scratch)
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

    if (profile.rx_fast_sfd) {
        if (!scratch.real_sync_template.is_real ||
            scratch.real_sync_template.length != sym) {
            rxopt::build_real_sync_template(template_wf, scratch.real_sync_template);
        }
        float sfd_norm_sq = 0.0f;
        for (int8_t s : sfd_sequence)
            sfd_norm_sq += static_cast<float>(s * s);
        if (sfd_norm_sq > 0.0f && scratch.real_sync_template.energy > 0.0f) {
            sfd_scale = 1.0f / std::sqrt(sfd_norm_sq * scratch.real_sync_template.energy);
        }
    }

    if (!profile.rx_fast_sfd || !scratch.real_sync_template.is_real) {
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
            scratch.corr.clear();
            scratch.corr.reserve(len * sym);
            for (size_t i = 0; i < len; ++i) {
                const int8_t s = sfd_sequence[i];
                for (size_t k = 0; k < sym; ++k)
                    scratch.corr.push_back(
                        std::complex<float>(static_cast<float>(s) *
                                                template_wf[k].real(),
                                            static_cast<float>(s) *
                                                template_wf[k].imag()));
            }
            gr::uwb::core::uwb_l2_normalize(scratch.corr);
            sfd_tmpl = scratch.corr.data();
            sfd_len = scratch.corr.size();
        }
        if (sfd_len == 0)
            return false;
    }

    // Candidate correlation: legacy scalar loop by default, VOLK dot when the
    // explicit fast path is enabled.  Candidate set/threshold/order unchanged.
    auto sfd_dot = [&](size_t j) -> std::complex<float> {
        if (profile.rx_fast_sfd && scratch.real_sync_template.is_real) {
            return rxopt::sfd_real_dot_single(
                rx + j, scratch.real_sync_template.taps_dup.data(),
                sym, sfd_sequence.data(), len, sfd_scale);
        }
        if (profile.rx_fast_sfd)
            return rxopt::conjugate_dot(rx + j, sfd_tmpl, sfd_len);
        std::complex<float> acc(0.0f, 0.0f);
        for (size_t k = 0; k < sfd_len; ++k)
            acc += rx[j + k] * std::conj(sfd_tmpl[k]);
        return acc;
    };

    // MATLAB strategy (refineTimingWithNsSfd): SFD start = preamble start +
    // preamble_repetitions × measured_period.  This is physically correct even
    // when the trailing preamble SYNC(s) score below the acquisition threshold
    // and are not tracked (partial train).
    // Live search: ±64-sample narrow window at that center, then the same
    // narrow window at ±1..±4 symbol offsets if the nominal center misses.
    const int64_t period =
        (timing.measured_period > 0.0)
            ? static_cast<int64_t>(std::llround(timing.measured_period))
            : static_cast<int64_t>(sym);
    const int64_t expected = timing.preamble_start_sample +
        static_cast<int64_t>(profile.preamble_repetitions) * period;
    out.expected_start_sample = expected;
    // Full-rate correlation (normalized) over the search window.
    // The signal is 2x oversampled and the SFD template is kron(sfd, preamble)
    // (8 symbols), so the correlation peak is broad: a decimated coarse pass
    // (stride D) localizes the region, then a small full-rate refinement
    // recovers the exact sample.  Window power is a sliding sum so the inner
    // loop only accumulates the correlation.  The result (best_j, metric) is
    // identical to the exhaustive search for a unique peak.
    constexpr int64_t kSfdNarrowHalf = 64;
    constexpr int kSfdSymbolOffsets = 4;
    float best = -1.0f;
    size_t best_j = 0;

    // Shared per-symbol SFD primitive.  `use_real_sfd_fast` / `taps_dup` are
    // hoisted out of scan_window so the low-SNR discrete-grid fallback below
    // can reuse the exact same kernels.
    const bool use_real_sfd_fast =
        profile.rx_fast_sfd && scratch.real_sync_template.is_real;
    const float* taps_dup =
        use_real_sfd_fast ? scratch.real_sync_template.taps_dup.data()
                          : nullptr;

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
            std::complex<float> c(0.0f, 0.0f);
            if (use_real_sfd_fast) {
                c = rxopt::real_template_dot_single(rx + p, taps_dup, sym);
            } else {
                for (size_t k = 0; k < sym; ++k)
                    c += rx[p + k] * std::conj(template_wf[k]);
            }
            sym_corr[si] = c;
            coh_ref += static_cast<double>(sfd_sequence[si]) *
                       std::complex<double>(c);
        }
        if (!(std::norm(coh_ref) > 1e-24))
            return false;
        size_t sign_ok = 0;
        for (size_t si = 0; si < n_check; ++si) {
            const double proj =
                static_cast<double>(sym_corr[si].real()) * coh_ref.real() +
                static_cast<double>(sym_corr[si].imag()) * coh_ref.imag();
            if (proj * static_cast<double>(sfd_sequence[si]) > 0.0)
                ++sign_ok;
        }
        return sign_ok == n_check;
    };

    auto scan_window = [&](int64_t center, int64_t half_width,
                           int64_t backtrack_symbols) {
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
        float pwr = 0.0f;
        for (size_t k = 0; k < sfd_len; ++k)
            pwr += std::norm(rx[j_lo + k]);

        const size_t fold_len = (j_end - j_lo) + sym;
        if (use_real_sfd_fast) {
            if (scratch.sfd_folded.size() < fold_len)
                scratch.sfd_folded.resize(fold_len);
            rxopt::fold_sfd_window(rx + j_lo, sym, sfd_sequence.data(), len, fold_len, scratch.sfd_folded.data());
        }

        if (use_real_sfd_fast) {
            // Full-rate single pass using the 8-lag AVX-512 kernel: avoids
            // stride-8 peak-straddling misses on narrow SFD correlation peaks
            // while eliminating the second window-power initialization.
            scratch.metric.resize(j_end - j_lo + 1);
            for (size_t j = j_lo; j <= j_end;) {
                if (j + 7 <= j_end) {
                    std::complex<float> acc8[8];
                    rxopt::real_template_dot_8lag(
                        scratch.sfd_folded.data() + (j - j_lo), taps_dup, sym,
                        acc8);
                    for (int m = 0; m < 8; ++m) {
                        const size_t cur_j = j + static_cast<size_t>(m);
                        if (cur_j > j_lo)
                            pwr += std::norm(rx[cur_j + sfd_len - 1]) -
                                   std::norm(rx[cur_j - 1]);
                        ++out.coarse_correlations;
                        const float metric =
                            std::norm(acc8[m] * sfd_scale) / (pwr + 1e-12f);
                        scratch.metric[cur_j - j_lo] = metric;
                        if (metric > window_best) {
                            window_best = metric;
                            window_best_j = cur_j;
                        }
                    }
                    j += 8;
                } else {
                    if (j > j_lo)
                        pwr += std::norm(rx[j + sfd_len - 1]) -
                               std::norm(rx[j - 1]);
                    ++out.fine_correlations;
                    const std::complex<float> acc =
                        rxopt::real_template_dot_single(
                            scratch.sfd_folded.data() + (j - j_lo), taps_dup,
                            sym) *
                        sfd_scale;
                    const float m = std::norm(acc) / (pwr + 1e-12f);
                    scratch.metric[j - j_lo] = m;
                    if (m > window_best) {
                        window_best = m;
                        window_best_j = j;
                    }
                    ++j;
                }
            }
            if (out.fine_correlations == 0)
                ++out.fine_correlations;
        } else {
            // Fallback non-fast path: full-rate pass so metric vector & PAR
            // match the fast path identically.
            scratch.metric.resize(j_end - j_lo + 1);
            for (size_t j = j_lo; j <= j_end; ++j) {
                if (j > j_lo)
                    pwr += std::norm(rx[j + sfd_len - 1]) -
                           std::norm(rx[j - 1]);
                if (((j - j_lo) & 1u) == 0u)
                    ++out.coarse_correlations;
                else
                    ++out.fine_correlations;
                const std::complex<float> acc = sfd_dot(j);
                const float m = std::norm(acc) / (pwr + 1e-12f);
                scratch.metric[j - j_lo] = m;
                if (m > window_best) {
                    window_best = m;
                    window_best_j = j;
                }
            }
            if (out.fine_correlations == 0)
                ++out.fine_correlations;
        }
        bool window_passed = (window_best >= profile.sfd_detection_threshold);
        if (!window_passed && window_best >= 0.02f &&
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
        if (window_best > best) {
            best = window_best;
            best_j = window_best_j;
        }
        if (out.first_threshold_backtrack_symbols < 0 && window_passed) {
            out.first_threshold_backtrack_symbols = backtrack_symbols;
        }
        return window_passed;
    };

    bool sfd_passed = scan_window(expected, kSfdNarrowHalf, 0);
    // Low-SNR early stop for the +-1..+-4 symbol offsets.  `best` is now the
    // k = 0 window's maximum.  When it cannot even reach 75% of the 0.02 low
    // floor, the offsets cannot pass either: the SFD correlation peak is
    // 0.008..0.05 at -10..-14 dB in-band, so the offsets are 8 x 129 = 1,032
    // doomed lag correlations.  The discrete grid below covers the same
    // +-4 symbols (and +-36) with a lower, polarity-locked acceptance gate, so
    // nothing is lost.  The `detected_peaks`/`metric` terms keep this off the
    // high-SNR path, where a large seed error can produce a low k = 0 maximum
    // but a perfectly good offset window.
    const bool skip_symbol_offsets =
        !sfd_passed && best < kSfdSkipNarrowGate &&
        timing.detected_peaks == profile.preamble_repetitions &&
        timing.metric < profile.radar_verification_threshold;
    for (int k = 1;
         k <= kSfdSymbolOffsets && !sfd_passed && !skip_symbol_offsets; ++k) {
        const int64_t step = static_cast<int64_t>(k) * static_cast<int64_t>(sym);
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
        constexpr int64_t kGridHalf = 36;
        const int64_t max_pos =
            static_cast<int64_t>(n) - static_cast<int64_t>(sfd_len);
        // Per-SYMBOL decomposition of the SFD correlation.  SFD =
        // kron(sfd_seq, sync), so a window at `pos` is the sfd_seq-weighted sum
        // of `len` single-SYMBOL correlations, and its power is the sum of the
        // same `len` single-SYMBOL powers.  The grid points are exactly
        // `period` apart, so when period == sym the 73 grid windows share
        // almost all of their symbols: 2*36 + len = 80 symbol positions cover
        // every one of them.  Precomputing those 80 with the SIMD kernels
        // replaces 73 x 8128 scalar complex multiply-accumulates and 73 x 8128
        // scalar norms with 80 x 1016 SIMD dots, and turns each grid metric
        // into `len` complex adds plus `len` float adds.
        const bool sym_decompose =
            use_real_sfd_fast && (period == static_cast<int64_t>(sym));
        const size_t n_sym_pos = static_cast<size_t>(2 * kGridHalf) + len;
        std::vector<std::complex<float>> sym_corr;
        std::vector<float> sym_pwr;
        std::vector<uint8_t> sym_valid;
        if (sym_decompose) {
            sym_corr.assign(n_sym_pos, std::complex<float>(0.0f, 0.0f));
            sym_pwr.assign(n_sym_pos, 0.0f);
            sym_valid.assign(n_sym_pos, 0);
            for (size_t i = 0; i < n_sym_pos; ++i) {
                const int64_t m = static_cast<int64_t>(i) -
                                  static_cast<int64_t>(kGridHalf);
                const int64_t spos =
                    expected + m * static_cast<int64_t>(sym);
                if (spos < 0 ||
                    spos + static_cast<int64_t>(sym) >
                        static_cast<int64_t>(n))
                    continue;
                sym_corr[i] =
                    rxopt::real_template_dot_single(rx + spos, taps_dup, sym);
                sym_pwr[i] = window_power_8acc(rx + spos, sym);
                sym_valid[i] = 1;
            }
        }
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
                    pwr += window_power_8acc(rx + p, sym);
                    const int8_t sg = sfd_sequence[s];
                    if (sg == 0)
                        continue;
                    const std::complex<float> d =
                        rxopt::real_template_dot_single(rx + p, taps_dup, sym);
                    if (sg > 0)
                        acc += d;
                    else
                        acc -= d;
                }
                acc *= sfd_scale;
                return std::norm(acc) / (pwr + 1e-12f);
            }
            const float pwr = window_power_8acc(rx + pos, sfd_len);
            const std::complex<float> acc = sfd_dot(static_cast<size_t>(pos));
            return std::norm(acc) / (pwr + 1e-12f);
        };
        struct GridCand {
            int64_t pos;
            float metric;
        };
        std::array<GridCand, 2 * kGridHalf + 1> cands{};
        size_t nc = 0;
        if (max_pos >= 0) {
            for (int64_t k = -kGridHalf; k <= kGridHalf; ++k) {
                const int64_t pos = expected + k * static_cast<int64_t>(period);
                if (pos < 0 || pos > max_pos)
                    continue;
                if (sym_decompose) {
                    const size_t base = static_cast<size_t>(k + kGridHalf);
                    bool ok = true;
                    float pwr = 0.0f;
                    std::complex<float> acc_unscaled(0.0f, 0.0f);
                    for (size_t s = 0; s < len; ++s) {
                        const size_t idx = base + s;
                        if (!sym_valid[idx]) {
                            ok = false;
                            break;
                        }
                        pwr += sym_pwr[idx];
                        const int8_t sg = sfd_sequence[s];
                        if (sg == 0)
                            continue;
                        if (sg > 0)
                            acc_unscaled += sym_corr[idx];
                        else
                            acc_unscaled -= sym_corr[idx];
                    }
                    if (!ok)
                        continue;
                    const std::complex<float> acc = acc_unscaled * sfd_scale;
                    cands[nc++] = { pos, std::norm(acc) / (pwr + 1e-12f) };
                } else {
                    cands[nc++] = { pos, sfd_metric_at(pos) };
                }
            }
        }
        if (nc >= 8) {
            ++out.search_windows;
            size_t bi = 0;
            for (size_t i = 1; i < nc; ++i)
                if (cands[i].metric > cands[bi].metric)
                    bi = i;
            int64_t gpos = cands[bi].pos;
            // Refine around the best grid point.  Evaluating delta = 0 on all
            // 73 grid points first and refining only the winner keeps the noise
            // maximum at 75 evaluations instead of 219.  When the +-1..+-4
            // symbol offset windows were skipped, the winner can be several
            // symbols off the nominal centre, so the sample refinement widens
            // to +-4.
            const int64_t refine_rad = skip_symbol_offsets ? 4 : 1;
            int64_t best_pos = gpos;
            float best_metric = cands[bi].metric;
            for (int64_t d = -refine_rad; d <= refine_rad; ++d) {
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
                    2 * static_cast<int64_t>(period))
                    continue;
                bg_sum += static_cast<double>(cands[i].metric);
                ++bg_cnt;
            }
            if (bg_cnt >= 16) {
                const double bg_mean =
                    std::max(bg_sum / static_cast<double>(bg_cnt), 1e-12);
                const double par =
                    static_cast<double>(best_metric) / bg_mean;
                if (best_metric >= kSfdGridPeakFloor && par >= kSfdGridParFloor &&
                    polarity_ok(best_pos)) {
                    best = best_metric;
                    best_j = static_cast<size_t>(best_pos);
                    sfd_passed = true;
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
    out.metric = best;
    out.polarity = 1;
    return true;
}

} // namespace gr::uwb::demod::core
