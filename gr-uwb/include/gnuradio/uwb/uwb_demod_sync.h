// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <gnuradio/uwb/uwb_demod_scratch.h>
#include <gnuradio/uwb/uwb_demod_grid.h>
#include <gnuradio/uwb/uwb_linear_fit.h>
#include <gnuradio/uwb/uwb_rx_matched_filter.h>
#include <gnuradio/uwb/uwb_detector_core.h>

namespace gr::uwb::demod::core {
// ---------------------------------------------------------------------------
// R1 implementations: Stage 1 (timing), Stage 2 (CFO), Stage 3 (SFD refine).
// Coordinates are 0-based positions within the input window. The complete
// packet entry point converts external absolute seeds and rebases results.  The template `template_wf` is the 1016-sample L2-normalized
// SYNC waveform.
// ---------------------------------------------------------------------------

namespace detail {

using gr::uwb::core::LinearFit;
using gr::uwb::core::normalized_correlation_power;
using gr::uwb::core::sample_power_sum;
using gr::uwb::core::conjugate_dot_scalar;

inline std::complex<float> timing_conjugate_dot(const std::complex<float>* input,
                                                const std::complex<float>* taps,
                                                size_t count)
{
    return gr::uwb::core::conjugate_dot(input, taps, count);
}

inline const rxopt::SparseCirTemplate* prepare_sparse_sync_template(
    const Qm35825Profile& profile, size_t length, DemodScratch& scratch)
{
    if (!profile.rx_timing_sparse || length != 2 * kQm35ChipsPerSymbol)
        return nullptr;
    if (scratch.cached_seq.code_index != profile.code_index ||
        scratch.cached_seq.pcode.empty())
        scratch.cached_seq.update(profile.code_index, profile.sfd_mode);
    const auto& code = scratch.cached_seq.pcode;
    rxopt::ensure_sparse_cir_template(code.data(), code.size(), scratch.cir_template);
    return scratch.cir_template.code_energy > 0.0f ? &scratch.cir_template : nullptr;
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

struct FoldPhaseProbe {
    const std::complex<float>* samples = nullptr;
    size_t count = 0;
    std::complex<float> corr{};

    double phase() const { return std::arg(corr); }

    bool coherent() const
    {
        if (count == 0)
            return false;
        const auto* delayed = samples + kQm35SamplesPerSymbol;
        const float ea = timing_conjugate_dot(samples, samples, count).real();
        const float eb = timing_conjugate_dot(delayed, delayed, count).real();
        // Complex white noise has mean normalized squared correlation 1/count.
        return std::norm(corr) > (12.0 / count) * ea * eb;
    }
};

// A one-symbol lag has a wider unambiguous CFO range than the sparse anchors.
// Integrate in the body of the preamble to avoid the uncertain leading edge.
inline FoldPhaseProbe probe_fold_phase(const std::complex<float>* rx,
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
    return { a, count, timing_conjugate_dot(a + sym, a, count) };
}

inline void fold_accumulate(std::complex<float>* dst,
                            const std::complex<float>* src,
                            size_t count,
                            double phase)
{
    if (phase == 0.0) {
        rxopt::vector_accumulate_complex(dst, src, count);
        return;
    }
    const float c = static_cast<float>(std::cos(phase));
    const float s = static_cast<float>(std::sin(phase));
    for (size_t j = 0; j < count; ++j) {
        const auto v = src[j];
        dst[j] += std::complex<float>(c * v.real() - s * v.imag(),
                                      s * v.real() + c * v.imag());
    }
}

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
                                      float threshold,
                                      double phase_per_symbol = 0.0,
                                      const rxopt::SparseCirTemplate* sparse_tmpl = nullptr)
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
        fold_accumulate(folded.data(), rx + w0 + k * P, fold_len,
                        -k * phase_per_symbol);

    const float* real_taps = real_tmpl.is_real && real_tmpl.length == L
        ? real_tmpl.taps_dup.data() : nullptr;
    const rxopt::SyncMatchedFilter filter(tmpl, L, real_taps, sparse_tmpl);
    if (sparse_tmpl && sparse_tmpl->code_energy > 0.0f)
        Et = sparse_tmpl->code_energy;
    const size_t lags = static_cast<size_t>(span + 1);
    const auto peak = rxopt::scan_normalized_sync(
        filter, folded.data(), lags, sample_power_sum(folded.data(), L), Et);

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
                            std::vector<std::complex<float>>& peak_corr,
                            double phase_per_symbol = 0.0,
                            const rxopt::SparseCirTemplate* sparse_tmpl = nullptr)
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
    const float* real_taps = real_tmpl.is_real && real_tmpl.length == L
        ? real_tmpl.taps_dup.data() : nullptr;
    const rxopt::SyncMatchedFilter filter(tmpl, L, real_taps, sparse_tmpl);
    if (sparse_tmpl && sparse_tmpl->code_energy > 0.0f)
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
            sample_power_sum(sup.data(), L), Et);
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

// Windowed |x|^2 with eight independent accumulators.  A single-accumulator
// float reduction cannot be vectorised without -ffast-math, and the power
// turned out to be HALF the cost of the single-point stage (measured 2x faster
// with eight accumulators).  The summation order differs from a plain scalar
// loop, but this value only normalises a correlation metric.
inline float window_power_8acc(const std::complex<float>* x, size_t count)
{
    const float* f = reinterpret_cast<const float*>(x);
    const size_t m = 2 * count;
    float a0 = 0.f, a1 = 0.f, a2 = 0.f, a3 = 0.f;
    float a4 = 0.f, a5 = 0.f, a6 = 0.f, a7 = 0.f;
    size_t i = 0;
    const size_t bulk = m & ~size_t{7};
    for (; i < bulk; i += 8) {
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
