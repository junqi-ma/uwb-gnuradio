// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <gnuradio/uwb/uwb_demod_scratch.h>
#include <gnuradio/uwb/uwb_demod_grid.h>

namespace gr::uwb::demod::core {
// ---------------------------------------------------------------------------
// R1 implementations: Stage 1 (timing), Stage 2 (CFO), Stage 3 (SFD refine).
// Coordinates are 0-based positions within the input window. The complete
// packet entry point converts external absolute seeds and rebases results.  The template `template_wf` is the 1016-sample L2-normalized
// SYNC waveform.
// ---------------------------------------------------------------------------

namespace detail {

// Long complex matched-filter dot product used by timing acquisition/tracking.
// VOLK computes sum(input[k] * conj(taps[k])) and runtime-dispatches to the
// fastest implementation available on the host.  Timing uses 1016-point dots,
// so dispatch overhead is amortized (unlike the 38-tap CIR hot loop).
inline std::complex<float>
timing_conjugate_dot(const std::complex<float>* input,
                     const std::complex<float>* taps,
                     size_t count)
{
    std::complex<float> acc(0.0f, 0.0f);
    volk_32fc_x2_conjugate_dot_prod_32fc(
        &acc,
        reinterpret_cast<const lv_32fc_t*>(input),
        reinterpret_cast<const lv_32fc_t*>(taps),
        static_cast<unsigned int>(count));
    return acc;
}

// Full-rate normalized matched filter over [roi_start, roi_end).  Fills
// metric[j] for j in [roi_start, roi_end-L), with
//   metric[j] = |<rx, tmpl>|^2 / (winpow[j] * Et + eps)
// Returns the argmax position and its metric.
inline size_t
normalized_mf(const std::complex<float>* rx,
              size_t n,
              size_t roi_start,
              size_t roi_end,
              const std::complex<float>* tmpl,
              size_t L,
              float tmpl_energy,
              std::vector<float>& metric, // caller-owned, >= n
              float* best_metric = nullptr)
{
    const size_t jmax = (roi_end >= L) ? roi_end - L + 1 : roi_start;
    size_t best = roi_start;
    float bm = -1.0f;
    if (jmax <= roi_start)
        return best;
    // window power of rx over the same L-length window (running sum)
    float wsum = 0.0f;
    for (size_t k = 0; k < L; ++k)
        wsum += std::norm(rx[roi_start + k]);
    for (size_t j = roi_start; j < jmax; ++j) {
        std::complex<float> corr(0.0f, 0.0f);
        for (size_t k = 0; k < L; ++k)
            corr += rx[j + k] * std::conj(tmpl[k]);
        const float d = wsum * tmpl_energy + 1e-12f;
        const float m = std::norm(corr) / d;
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

// ---------------------------------------------------------------------------
// K-symbol periodic-fold acquisition (low-SNR fallback).
//
// Why this exists
// ---------------
// The HRP preamble is N verbatim copies of ONE 1016-sample SYNC symbol, so the
// single-symbol normalized correlation at -4..-8 dB in-band SNR has a per-SYNC
// expectation of only ~0.1-0.2.  `radar_verification_threshold` (0.25) then
// rejects almost every SYNC and acquisition fails even though the preamble is
// plainly present (measured on the 256-SYNC fixture: all 256 peaks are > 0.08
// with PAR 232..253, but only 11-24 of 256 clear 0.25).
//
// Coherently folding K symbol periods before correlating buys 10*log10(K) of
// SNR (+18 dB at K=64).  Measured at -14 dB in-band: the folded metric is
// 0.532 (min) / 0.560 (median), while the maximum over 200 pure-noise fold
// positions is 0.0107 -- roughly 50x of margin, hence the 0.05 gate.
//
// The fold is PERIODIC, and so is the preamble: 256 verbatim copies of one
// SYNC symbol carry no information that distinguishes SYNC #0 from SYNC #7, so
// the folded metric is exactly P-periodic in the lag.  Folding over the SEARCH
// WINDOW (fold_len = (w1-w0) + L, not one period) still matters -- it keeps the
// fold and the correlation on the same absolute grid and lets the window be
// searched at all -- but it does NOT by itself recover an absolute position:
// every symbol start in the window scores the same.  The only absolute
// reference available before the SFD is the seed, so the reported position is
// the strongest lag within +/-P/2 of it.  The reference prototype measured the
// same limit (docs/performance/data_low_snr_lock_20261117/fold_timing.json,
// "coarse offset sweep": the total error stays inside +/-508 samples = half a
// SYMBOL period and slips by whole periods beyond that).
// ---------------------------------------------------------------------------
inline constexpr size_t kFoldMaxSymbols = 64;
inline constexpr size_t kFoldMinSymbols = 16;
inline constexpr float kFoldThreshold = 0.05f;
inline constexpr size_t kFoldAnchorGroupSymbols = 16;
inline constexpr int64_t kFoldAnchorRefine = 8;

struct FoldAcquireResult {
    bool ok = false;
    int64_t start = -1;          // absolute SYMBOL START of the packet
    float metric = 0.0f;         // folded normalized correlation at `start`
    std::complex<float> corr{ 0.0f, 0.0f };
    size_t symbols_folded = 0;
    size_t window_lags = 0;
};

// Fold `[w0, w1]` (a window of legal SYMBOL START positions) by up to
// `max_symbols` periods and return the strongest normalized correlation lag
// within +/-P/2 of `centre`.  Degrades K when the buffer cannot hold
// `max_symbols` periods from `w0`, and refuses below kFoldMinSymbols (the gate
// is not safe without the fold gain).
inline FoldAcquireResult fold_acquire(const std::complex<float>* rx,
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
                                      float threshold)
{
    FoldAcquireResult res;
    const int64_t P = static_cast<int64_t>(kQm35SamplesPerSymbol);
    const int64_t Li = static_cast<int64_t>(L);
    if (!rx || !tmpl || L == 0 || n < L)
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
    // Rare failure path: allocation here is acceptable (this runs at most once
    // per packet, and only for packets the single-symbol search lost).
    std::vector<std::complex<float>> folded(fold_len,
                                           std::complex<float>(0.0f, 0.0f));
    for (int64_t k = 0; k < k_eff; ++k)
        rxopt::vector_accumulate_complex(folded.data(), rx + w0 + k * P,
                                         fold_len);

    const bool use_real = real_tmpl.is_real && real_tmpl.length == L;
    const float* taps_dup = real_tmpl.taps_dup.data();
    float pw = 0.0f;
    for (size_t m = 0; m < L; ++m)
        pw += std::norm(folded[m]);

    // Position ambiguity: only lags inside +/-P/2 of the seed can be told apart
    // (see the note above).  The window was already clamped to exactly that
    // sub-window, so the argmax range below normally covers all of it; the
    // general expressions are kept because a buffer-edge clamp can still cut
    // one side short.
    const int64_t lo_lag = std::max<int64_t>(0, (centre - w0) - half_period);
    const int64_t hi_lag = std::min<int64_t>(span, (centre - w0) + half_period);

    float bm = -1.0f;
    int64_t bj = lo_lag;
    std::complex<float> bacc(0.0f, 0.0f);
    int64_t j = 0;
    if (use_real) {
        for (; j + 7 <= span; j += 8) {
            std::complex<float> acc[8];
            rxopt::real_template_dot_8lag(folded.data() + j, taps_dup, L, acc);
            for (int m = 0; m < 8; ++m) {
                const float met = std::norm(acc[m]) / (pw * Et + 1e-12f);
                const int64_t lag = j + m;
                if (lag >= lo_lag && lag <= hi_lag && met > bm) {
                    bm = met;
                    bj = lag;
                    bacc = acc[m];
                }
                if (lag < span)
                    pw += std::norm(folded[lag + Li]) - std::norm(folded[lag]);
            }
        }
    }
    for (; j <= span; ++j) {
        const std::complex<float> acc =
            use_real ? rxopt::real_template_dot_single(folded.data() + j,
                                                       taps_dup, L)
                     : timing_conjugate_dot(folded.data() + j, tmpl, L);
        const float met = std::norm(acc) / (pw * Et + 1e-12f);
        if (j >= lo_lag && j <= hi_lag && met > bm) {
            bm = met;
            bj = j;
            bacc = acc;
        }
        if (j < span)
            pw += std::norm(folded[j + Li]) - std::norm(folded[j]);
    }

    res.symbols_folded = static_cast<size_t>(k_eff);
    res.window_lags = static_cast<size_t>(span + 1);
    if (!(bm >= threshold) || hi_lag < lo_lag)
        return res;
    res.ok = true;
    res.start = w0 + bj;
    res.metric = bm;
    res.corr = bacc;
    return res;
}

// 16-SYMBOL super-anchor period/origin fit for the fold path.
//
// The folded metric resolves the packet origin to ~1 sample, but its period
// would be the nominal grid by construction: the fold has no resolution along
// the symbol index.  `measured_period` feeds integer_chip_grid()/spc_ppm and
// the fused-CMF admission, so it must stay a genuine measurement.  Group the
// SYNCs into `groups` blocks of kFoldAnchorGroupSymbols, coherently sum each
// block into one super-symbol (+12 dB), refine its position with a
// +/-kFoldAnchorRefine normalized search, then least-squares fit
//     peak(g) = start + (g*kFoldAnchorGroupSymbols)*period + (L-1)
// exactly like the legacy 256-peak fit (symbol-END convention; the intercept is
// converted back to a SYMBOL START by subtracting L-1).
inline bool fold_anchor_fit(const std::complex<float>* rx,
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
                            std::vector<std::complex<float>>& peak_corr)
{
    peak_samples.clear();
    peak_metrics.clear();
    peak_corr.clear();
    const int64_t P = static_cast<int64_t>(kQm35SamplesPerSymbol);
    const int64_t Li = static_cast<int64_t>(L);
    const int64_t refine = kFoldAnchorRefine;
    const size_t sup_len = L + 2 * static_cast<size_t>(refine);
    if (groups < 2 || L == 0 || !rx || !tmpl)
        return false;

    std::vector<std::complex<float>> sup(sup_len,
                                         std::complex<float>(0.0f, 0.0f));
    const bool use_real = real_tmpl.is_real && real_tmpl.length == L;
    const float* taps_dup = real_tmpl.taps_dup.data();

    double sx = 0.0, sy = 0.0, sxx = 0.0, sxy = 0.0;
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
            rxopt::vector_accumulate_complex(sup.data(), rx + base + j * P,
                                             sup_len);
        float pw = 0.0f;
        for (size_t m = 0; m < L; ++m)
            pw += std::norm(sup[m]);
        float bm = -1.0f;
        int64_t bl = 0;
        std::complex<float> bacc(0.0f, 0.0f);
        int64_t lag = 0;
        if (use_real) {
            // Eight lags per call: the 8-lag kernel keeps eight independent
            // accumulators, so it costs barely more than one long FMA chain and
            // is bit-identical per lag to real_template_dot_single().  The
            // metric/power update order is unchanged, so the selected anchor is
            // identical to the single-lag loop it replaces.
            for (; lag + 7 <= 2 * refine; lag += 8) {
                std::complex<float> acc8[8];
                rxopt::real_template_dot_8lag(sup.data() + lag, taps_dup, L,
                                              acc8);
                for (int m = 0; m < 8; ++m) {
                    const int64_t cur = lag + m;
                    const float met = std::norm(acc8[m]) / (pw * Et + 1e-12f);
                    if (met > bm) {
                        bm = met;
                        bl = cur;
                        bacc = acc8[m];
                    }
                    if (cur < 2 * refine)
                        pw += std::norm(sup[cur + Li]) - std::norm(sup[cur]);
                }
            }
        }
        for (; lag <= 2 * refine; ++lag) {
            const std::complex<float> acc =
                use_real ? rxopt::real_template_dot_single(sup.data() + lag,
                                                           taps_dup, L)
                         : timing_conjugate_dot(sup.data() + lag, tmpl, L);
            const float met = std::norm(acc) / (pw * Et + 1e-12f);
            if (met > bm) {
                bm = met;
                bl = lag;
                bacc = acc;
            }
            if (lag < 2 * refine)
                pw += std::norm(sup[lag + Li]) - std::norm(sup[lag]);
        }
        const int64_t pos = base + bl;
        const double x = static_cast<double>(
            static_cast<int64_t>(g) *
            static_cast<int64_t>(kFoldAnchorGroupSymbols));
        const double y = static_cast<double>(pos + Li - 1);
        sx += x;
        sy += y;
        sxx += x * x;
        sxy += x * y;
        peak_samples.push_back(pos + Li - 1);
        peak_metrics.push_back(bm);
        peak_corr.push_back(bacc);
        ++used;
    }
    if (used < 2)
        return false;
    const double denom = static_cast<double>(used) * sxx - sx * sx;
    if (!(std::abs(denom) > 1e-9))
        return false;
    const double p = (static_cast<double>(used) * sxy - sx * sy) / denom;
    if (!std::isfinite(p) || !(p > 0.0))
        return false;
    // Same 0.5% sanity gate the LockedBounded anchor fit uses: a period that
    // far off the nominal grid means the anchors did not land on the expected
    // symbol indices at all.
    if (std::abs(p - static_cast<double>(P)) > 0.005 * static_cast<double>(P))
        return false;
    const double intercept_end = (sy - p * sx) / static_cast<double>(used);
    *period_out = p;
    *preamble_start_out = std::max<int64_t>(
        0, static_cast<int64_t>(std::llround(intercept_end)) - (Li - 1));
    return true;
}

} // namespace detail

inline float window_power_8acc(const std::complex<float>* x, size_t count)
{
    const float* f = reinterpret_cast<const float*>(x);
    const size_t m = 2 * count;
    float a0 = 0.f, a1 = 0.f, a2 = 0.f, a3 = 0.f;
    float a4 = 0.f, a5 = 0.f, a6 = 0.f, a7 = 0.f;
    size_t i = 0;
    for (; i + 8 <= m; i += 8) {
        a0 += f[i + 0] * f[i + 0];
        a1 += f[i + 1] * f[i + 1];
        a2 += f[i + 2] * f[i + 2];
        a3 += f[i + 3] * f[i + 3];
        a4 += f[i + 4] * f[i + 4];
        a5 += f[i + 5] * f[i + 5];
        a6 += f[i + 6] * f[i + 6];
        a7 += f[i + 7] * f[i + 7];
    }
    float s = ((a0 + a1) + (a2 + a3)) + ((a4 + a5) + (a6 + a7));
    for (; i < m; ++i)
        s += f[i] * f[i];
    return s;
}


} // namespace gr::uwb::demod::core
