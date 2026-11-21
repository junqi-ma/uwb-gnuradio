// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <openuwb/phy/hrp/rx/demod_scratch.h>
#include <openuwb/phy/hrp/rx/demod_grid.h>
#include <openuwb/support/linear_fit.h>
#include <openuwb/phy/hrp/rx/rx_matched_filter.h>
#include <openuwb/dsp/detector_core.h>

namespace openuwb::demod::core {
// ---------------------------------------------------------------------------
// R1 implementations: Stage 1 (timing), Stage 2 (CFO), Stage 3 (SFD refine).
// Coordinates are 0-based positions within the input window. The complete
// packet entry point converts external absolute seeds and rebases results.  The template `template_wf` is the 1016-sample L2-normalized
// SYNC waveform.
// ---------------------------------------------------------------------------

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

namespace detail {

using openuwb::core::LinearFit;
using openuwb::core::normalized_correlation_power;
using openuwb::core::sample_power_sum;
using openuwb::core::conjugate_dot_scalar;

inline std::complex<float> timing_conjugate_dot(const std::complex<float>* input,
                                                const std::complex<float>* taps,
                                                size_t count)
{
    return openuwb::core::conjugate_dot(input, taps, count);
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
size_t
normalized_mf(const std::complex<float>* rx,
              size_t n,
              size_t roi_start,
              size_t roi_end,
              const std::complex<float>* tmpl,
              size_t L,
              float tmpl_energy,
              std::vector<float>& metric, // caller-owned, >= n
              float* best_metric = nullptr);

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
    float ea = 0.0f; // energy of [0, count)
    float eb = 0.0f; // energy of [sym, sym + count)

    double phase() const { return std::arg(corr); }

    bool coherent() const
    {
        if (count == 0)
            return false;
        // Complex white noise has mean normalized squared correlation 1/count.
        return std::norm(corr) > (12.0f / static_cast<float>(count)) * ea * eb;
    }
};

// A one-symbol lag has a wider unambiguous CFO range than the sparse anchors.
// Integrate in the body of the preamble to avoid the uncertain leading edge.
//
// The two energies overlap by count - sym samples, so only the non-overlapping
// head/tail need to be summed: eb = ea - head + tail.  That removes a second
// full-window scan (~130k samples) without changing the gate.  The 128-symbol
// cap is kept: 64 symbols measurably hurt the -14 dB fold acquisition.
FoldPhaseProbe probe_fold_phase(const std::complex<float>* rx,
                                size_t n,
                                int64_t centre,
                                size_t repetitions);

#if UWB_RXOPT_HAVE_AVX2
__attribute__((target("avx2")))
inline void fold_accumulate_rotated_avx2(std::complex<float>* dst,
                                         const std::complex<float>* src,
                                         size_t count, float c, float s)
{
    // (re,im) * (c,s):  real = re*c - im*s, imag = im*c + re*s.
    // addsub(a,b) subtracts on even lanes and adds on odd lanes, which is
    // exactly the complex multiply; dst += product keeps the scalar order.
    const __m256 vc = _mm256_set1_ps(c);
    const __m256 vs = _mm256_set1_ps(s);
    size_t j = 0;
    const size_t limit = count & ~size_t(3);
    for (; j < limit; j += 4) {
        const __m256 v =
            _mm256_loadu_ps(reinterpret_cast<const float*>(src + j));
        const __m256 sw = _mm256_permute_ps(v, 0xB1);
        const __m256 rot = _mm256_addsub_ps(_mm256_mul_ps(v, vc),
                                            _mm256_mul_ps(sw, vs));
        const __m256 d =
            _mm256_loadu_ps(reinterpret_cast<const float*>(dst + j));
        _mm256_storeu_ps(reinterpret_cast<float*>(dst + j),
                         _mm256_add_ps(d, rot));
    }
    for (; j < count; ++j) {
        const auto v = src[j];
        dst[j] += std::complex<float>(c * v.real() - s * v.imag(),
                                      s * v.real() + c * v.imag());
    }
}
#endif

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
#if UWB_RXOPT_HAVE_AVX2
    if (count >= 4 && rxopt::has_avx2_fma()) {
        fold_accumulate_rotated_avx2(dst, src, count, c, s);
        return;
    }
#endif
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
                               double phase_per_symbol = 0.0,
                               const rxopt::SparseCirTemplate* sparse_tmpl = nullptr,
                               std::vector<std::complex<float>>* fold_buf = nullptr,
                               const rxopt::ShapedSparseTemplate* shaped_tmpl = nullptr);

// Interval local probe: coherently fold `fold_symbols` SYNC periods starting at
// `probe`, then correlate over ONE symbol period and report the best absolute
// SYMBOL start (`probe + peak.lag`).
//
// Unlike fold_acquire(), which collapses the WHOLE search window into one
// period and is therefore limited to +/-P/2 of the seed, this keeps the fold
// local.  A probe can sit anywhere inside a long preamble: the P-spaced comb
// still lands on P-spaced SYNC copies, so the folded vector carries K aligned
// symbols regardless of the probe's phase modulo P.  With K=4 that is +6 dB
// of coherent gain (K=2 still gives +3 dB with more CFO margin), and one probe
// costs one P-lag correlation instead of a stride-14 scan of the entire blind
// margin.
//
// Normalization is the same |corr|^2 / (power * Et) as the single-symbol path.
// The folded signal and its window power are measured on the SAME buffer, so no
// extra K factor is applied: dividing by K would wrongly cap a perfect K-fold
// at 1/K instead of 1.  (fold_acquire uses the identical convention.)
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
    const rxopt::SparseCirTemplate* sparse_tmpl = nullptr,
    std::vector<std::complex<float>>* fold_buf = nullptr,
    const rxopt::ShapedSparseTemplate* shaped_tmpl = nullptr);

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
                     double phase_per_symbol = 0.0,
                     const rxopt::SparseCirTemplate* sparse_tmpl = nullptr,
                     std::vector<std::complex<float>>* sup_buf = nullptr,
                     const rxopt::ShapedSparseTemplate* shaped_tmpl = nullptr);

} // namespace detail

} // namespace openuwb::demod::core
