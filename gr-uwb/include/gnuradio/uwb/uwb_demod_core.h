/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * GNU Radio-independent UWB demodulation core.  Each stage is a pure function
 * that can be unit-tested and benchmarked without linking against GNU Radio.
 *
 * This header is the Phase-1 SKELETON: it declares the per-stage function
 * signatures and the worker's scratch state.  No algorithm is implemented
 * here until the MATLAB golden reference (R0) is frozen.
 *
 * Coordinate convention: all sample indices are 0-based absolute.  Each stage
 * takes the cropped frame + the previous stage's output and fills the next.
 */

#pragma once

#include <gnuradio/uwb/uwb_cir_fir_simd.h>
#include <gnuradio/uwb/uwb_demod_result.h>
#include <gnuradio/uwb/uwb_demod_result.h>
#include <gnuradio/uwb/uwb_detector_core.h>
#include <gnuradio/uwb/uwb_phy_profile.h>
#include <gnuradio/uwb/uwb_rx_cfo_opt.h>
#include <gnuradio/uwb/uwb_rx_sync_opt.h>
#include <gnuradio/uwb/uwb_rx_softchip_opt.h>
#include <gnuradio/uwb/uwb_rx_fused_cmf.h>
#include <gnuradio/uwb/uwb_softchip_fir_diag.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstdint>
#include <cstdlib>
#include <cstdio>
#include <limits>
#include <vector>

namespace gr {
namespace uwb {
namespace demod {
namespace core {

// ---------------------------------------------------------------------------
// Integer chip-grid admission
//
// The AVX2 soft-chip kernel and the fused CMF both index samples on an exact
// kSamplesPerChip grid, so the real question is not "is samples_per_chip
// exactly 2" but "does the grid drift accumulated across the frame stay inside
// the correlation tolerance".
//
// Physical ceiling.  The longest supported frame is 127 B, whose BPRF payload
// spans 154,624 samples.  One sample of drift over that span is a sample-clock
// offset of 1/154624 = 6.5 ppm; past that the integer grid walks off the
// correlation peak inside a single frame.
//
// Default admission is deliberately tighter: 1.5 ppm = 0.23 samples of drift,
// with 2.3x margin over the best clock measured on hardware (0.66 ppm).  The
// 256-peak period estimator quantises samples_per_chip in ~2 ppm steps, so a
// gate at the physical ceiling would admit a clock whose *measured* offset is
// 3-6 ppm away from zero without any real evidence that the clock is good.
// Callers can widen or tighten it per run through Qm35825Profile::fused_gate_ppm
// without a rebuild.
//
// Measured on hardware: a 27 ppm clock drifts 4.2 samples across the payload
// and is rejected at every setting; a 0.66 ppm clock drifts 0.10 samples and is
// accepted.  The previous 1e-9 (0.0005 ppm) threshold was three orders of
// magnitude BELOW the estimator noise, so real data could never satisfy it and
// the fused path was unreachable on hardware.
// ---------------------------------------------------------------------------
inline constexpr double kMaxFramePayloadSamples = 154624.0; // 127 B BPRF
// Hard ceiling: one sample of drift across the longest frame.
inline constexpr double kIntegerGridCeilingPpm =
    1.0 / kMaxFramePayloadSamples * 1e6;
// Default admission (deliberately tighter than the ceiling).
inline constexpr double kIntegerGridMaxClockPpm = 1.5;
inline constexpr double kIntegerGridRelTolerance =
    kIntegerGridMaxClockPpm * 1e-6;
inline constexpr double kIntegerGridMaxDriftSamples =
    kIntegerGridRelTolerance * kMaxFramePayloadSamples;

/** True when `samples_per_chip` sits within `rel_tolerance` of `target`. */
inline bool integer_chip_grid_at(double samples_per_chip, size_t target,
                                 double rel_tolerance)
{
    if (!(samples_per_chip > 0.0) || target == 0 ||
        !(rel_tolerance > 0.0) || !std::isfinite(rel_tolerance))
        return false;
    return std::abs(samples_per_chip - static_cast<double>(target)) /
               samples_per_chip <=
           rel_tolerance;
}

/** Default-tolerance overload (see kIntegerGridMaxClockPpm). */
inline bool integer_chip_grid_at(double samples_per_chip, size_t target)
{
    return integer_chip_grid_at(samples_per_chip, target,
                                kIntegerGridRelTolerance);
}

/** True when `samples_per_chip` sits within tolerance of its nearest integer. */
inline bool integer_chip_grid(double samples_per_chip, double rel_tolerance)
{
    if (!(samples_per_chip > 0.0))
        return false;
    const double nearest = std::round(samples_per_chip);
    if (nearest <= 0.0)
        return false;
    return integer_chip_grid_at(samples_per_chip,
                                static_cast<size_t>(nearest), rel_tolerance);
}

/** Default-tolerance overload (see kIntegerGridMaxClockPpm). */
inline bool integer_chip_grid(double samples_per_chip)
{
    return integer_chip_grid(samples_per_chip, kIntegerGridRelTolerance);
}

// ---------------------------------------------------------------------------
// R1 implementations: Stage 1 (timing), Stage 2 (CFO), Stage 3 (SFD refine).
// Coordinate convention: 0-based ABSOLUTE sample indices (matching detector /
// PDU metadata and the R0 golden reference, which is stored in absolute
// coordinates).  The template `template_wf` is the 1016-sample L2-normalized
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
    // (see the note above).  Track the argmax of that central sub-window; the
    // full window is still searched so the fold and the correlation share one
    // absolute grid and the seed's own error is measurable.
    const int64_t half_period = P / 2;
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
        for (int64_t lag = 0; lag <= 2 * refine; ++lag) {
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

// ---------------------------------------------------------------------------
// Worker scratch: one per worker thread.  Pre-allocated at construction and
// reused across jobs to avoid per-packet allocation in the hot path.
// ---------------------------------------------------------------------------
struct DemodScratch {
    std::vector<std::complex<float>> work;     // cropped frame workspace
    std::vector<std::complex<float>> derotated; // CFO-derotated frame
    std::vector<float> energy;                  // per-sample |x|^2
    std::vector<float> metric;                  // correlation metric buffer
    std::vector<std::complex<float>> corr;      // matched-filter output
    std::vector<float> soft_chips;              // soft-chip stream
    std::vector<float> cir_taps;                // CIR estimate
    std::vector<int64_t> peaks;                 // SYNC peak positions
    std::vector<float> peak_values;             // per-peak complex correlation
    // On-demand soft-chip FIR output.  Kept separate from `corr` because the
    // extension call re-runs the (cheap) CIR estimate, which temporarily uses
    // `corr` for the coherent SYNC average while the prefix FIR values must
    // survive.  Unused by the legacy path.
    std::vector<std::complex<float>> softchip_corr;

    // Diagnostic-only replay of the legacy soft-chip FIR, segmented at the
    // decoded SFD/PHR/payload boundaries.  Never part of a formal timing run
    // and NOT part of the public Qm35825Profile production configuration:
    // only the experiment wrappers set this on their private scratch.
    bool diagnose_fir_segments = false;

    // ---- Experimental RX fast-path state ---------------------------------
    // Cached normalized SFD waveform (kron(sfd_sequence, template_wf)).
    // The key covers the template + SFD content, so the cache does not depend
    // on the temporary contents of work/corr surviving between calls.
    std::vector<std::complex<float>> sfd_template;
    uint64_t sfd_template_key = 0;

    // Cached sparse ternary CIR template (Task A2).
    rxopt::SparseCirTemplate cir_template;

    // Cached pre-duplicated real SYNC template for AVX2 fast dot.
    rxopt::RealSyncTemplate real_sync_template;

    // Cached sequence data for fixed preamble code and SFD mode (Task B2).
    struct CachedSequenceState {
        size_t code_index = std::numeric_limits<size_t>::max();
        std::string sfd_mode;
        std::vector<int8_t> pcode;
        std::vector<int8_t> sfd_seq;
        std::vector<int8_t> sampled_pcode;

        struct SparseSfdEntry {
            uint32_t offset;
            int8_t sign;
        };
        std::vector<SparseSfdEntry> ns_sfd_sparse;
        size_t ns_sfd_total_chips = 0;
        float ns_sfd_norm = 0.0f;

        void update(size_t new_code_index, const char* new_sfd_mode)
        {
            const char* cur = new_sfd_mode ? new_sfd_mode : "";
            const bool code_changed = (code_index != new_code_index);
            const bool mode_changed = (sfd_mode != cur);
            if (code_changed) {
                code_index = new_code_index;
                const int8_t* pc = GetPreambleCode(code_index);
                pcode.assign(pc, pc + kQm35CodeLength);
                sampled_pcode = BuildSampledCode(pcode.data(), pcode.size());
            }
            if (mode_changed) {
                sfd_mode = cur;
                sfd_seq = gr::uwb::demod::GetSfdSequence(new_sfd_mode);
            }
            if (code_changed || mode_changed || ns_sfd_sparse.empty()) {
                ns_sfd_sparse.clear();
                ns_sfd_total_chips = sfd_seq.size() * sampled_pcode.size();
                for (size_t i = 0; i < sfd_seq.size(); ++i) {
                    const int8_t si = sfd_seq[i];
                    if (si == 0) continue;
                    for (size_t j = 0; j < sampled_pcode.size(); ++j) {
                        const int8_t pj = sampled_pcode[j];
                        if (pj == 0) continue;
                        const int8_t val = static_cast<int8_t>(si * pj);
                        ns_sfd_sparse.push_back({ static_cast<uint32_t>(i * sampled_pcode.size() + j), val });
                    }
                }
                ns_sfd_norm = std::sqrt(static_cast<float>(ns_sfd_sparse.size()));
            }
        }

        void invalidate()
        {
            code_index = std::numeric_limits<size_t>::max();
            sfd_mode.clear();
            pcode.clear();
            sfd_seq.clear();
            sampled_pcode.clear();
            ns_sfd_sparse.clear();
            ns_sfd_total_chips = 0;
            ns_sfd_norm = 0.0f;
        }
    } cached_seq;

    // Buffer for SFD window pre-folding.
    std::vector<std::complex<float>> sfd_folded;

    // Retained initial timing for tail re-track conditional equivalence.
    TimingResult initial_timing;

    // Two-phase soft-chip state.  Only populated when
    // profile.rx_softchip_on_demand is enabled; the legacy full-stream path
    // never reads it.  The extension re-runs the cheap CIR estimate and
    // deterministically rebuilds the same taps/grid, so only the prefix
    // boundary and its normalization maximum must persist.
    struct SoftChipState {
        bool prepared = false;
        size_t generated = 0;
        float prefix_max = 0.0f;
        // Phase rotation frozen by the prefix pass: the phase alignment is
        // estimated once per packet and reused by the extension.
        std::complex<float> rot{ 1.0f, 0.0f };

        void reset() { *this = SoftChipState{}; }
    } softchip;

    // Experimental fused payload CMF/despread state.  PN depends only on the
    // selected code and the decoded payload length, so it survives packets;
    // the per-packet channel templates live in kernel.templates and are
    // rebuilt for every packet (counted in stage_fused_template_us).
    struct FusedCmfState {
        size_t code_index = 0;
        size_t symbols = 0;
        std::vector<int8_t> pn;
        rxfusion::Metrics metrics;
        rxfusion::Scratch kernel;

        void reset_metrics()
        {
            rxfusion::Metrics empty;
            metrics = std::move(empty);
        }
    } fused;

    // Experimental fused PHR CMF/despread state.  PN depends only on the
    // selected code (fixed 21 symbols, scrambler offset 0); the per-packet
    // channel templates live in kernel.templates.  The 21 raw g0/g1 decisions
    // are cached for the joint payload Viterbi, which must not re-run the
    // soft-chip PHR demod when the PHR was produced by the fused kernel.
    struct FusedPhrState {
        size_t code_index = 0;
        std::vector<int8_t> pn;
        std::vector<int8_t> g0;
        std::vector<int8_t> g1;
        bool decisions_valid = false;
        rxfusion::Metrics metrics;
        rxfusion::Scratch kernel;

        void reset_decisions()
        {
            decisions_valid = false;
            g0.clear();
            g1.clear();
        }
    } fused_phr;

    void reserve(size_t n)
    {
        work.reserve(n);
        derotated.reserve(n);
        energy.reserve(n);
        metric.reserve(n);
        corr.reserve(n);
        softchip_corr.reserve(n);
        soft_chips.reserve(n);
        cir_taps.reserve(64);
        peaks.reserve(256);
        peak_values.reserve(256);
        sfd_template.reserve(8 * kQm35SamplesPerSymbol);
        cached_seq.pcode.reserve(kQm35CodeLength);
        cached_seq.sfd_seq.reserve(64);
        cached_seq.sampled_pcode.reserve(2 * kQm35ChipsPerSymbol);
        fused.pn.reserve(8 * 2048);
        fused.metrics.complex_value.reserve(
            rxfusion::kCandidateCount * 2048);
        fused.metrics.signed_value.reserve(
            rxfusion::kCandidateCount * 2048);
        fused.kernel.templates.reserve(
            rxfusion::kBlockSymbols * (rxfusion::kMaxTapCount +
                                       rxfusion::kSamplesPerChip *
                                           (rxfusion::kChipsPerBurst - 1)));
        fused_phr.pn.reserve(rxfusion::kPhrSymbols *
                             rxfusion::kPhrChipsPerBurst);
        fused_phr.g0.reserve(rxfusion::kPhrSymbols);
        fused_phr.g1.reserve(rxfusion::kPhrSymbols);
        fused_phr.metrics.complex_value.reserve(
            rxfusion::kCandidateCount * rxfusion::kPhrSymbols);
        fused_phr.metrics.signed_value.reserve(
            rxfusion::kCandidateCount * rxfusion::kPhrSymbols);
        fused_phr.kernel.templates.reserve(
            rxfusion::kBlockSymbols *
            (rxfusion::kMaxTapCount +
             rxfusion::kSamplesPerChip *
                 (rxfusion::kPhrChipsPerBurst - 1)));
    }
};

// ---------------------------------------------------------------------------
// Stage 1 — seeded timing (R1).
// Uses the detector-supplied absolute seed_start (or, if -1, scans the whole
// buffer) to run a full-rate matched filter over a narrow ROI, then tracks
// the repeated SYNC peaks spaced ~samples_per_symbol apart, and fits a linear
// model to get measured_period.  Peak positions are symbol-END convention
// (first peak = start + (L-1)), matching the detector / MATLAB.
// ---------------------------------------------------------------------------
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
                          bool allow_par_fallback = false) -> bool {
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
        float pw = 0.0f;
        for (size_t k = 0; k < L; ++k)
            pw += std::norm(rx[w0 + static_cast<int64_t>(k)]);

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
                float pw = 0.0f;
                for (size_t k = 0; k < L; ++k)
                    pw += std::norm(rx[pos + static_cast<int64_t>(k)]);
                const std::complex<float> acc =
                    scratch.real_sync_template.is_real
                        ? rxopt::real_template_dot_single(
                              rx + pos,
                              scratch.real_sync_template.taps_dup.data(), L)
                        : detail::timing_conjugate_dot(rx + pos,
                                                       scratch.work.data(), L);
                return std::norm(acc) / (pw * Et + 1e-12f);
            };
            float best_score = -1e30f;
            float max_here = 0.0f;
            for (int64_t j = -max_j; j <= max_j; ++j) {
                const float m = ss_metric(fold.start + j * P);
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
            for (int64_t j = -max_j; j <= max_j; ++j) {
                const int64_t c = fold.start + j * P;
                const float here = ss_metric(c);
                if (here < 0.0f)
                    continue;
                if (here >= edge_thr) {
                    fold_start = c;
                    best_score = here;
                    break;
                }
            }
            (void)best_score;
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
        for (size_t m = 0; m <= inner; ++m) {
            const int64_t want =
                static_cast<int64_t>(seed_start) +
                static_cast<int64_t>(m) * static_cast<int64_t>(kQm35SamplesPerSymbol);
            if (want + static_cast<int64_t>(L) >
                static_cast<int64_t>(n))
                break;
            if (local_best(want, roi, thr, &pos, &met, &corr,
                           /*allow_par_fallback=*/true)) {
                used_inner = m;
                found = true;
                break;
            }
        }
        if (!found && profile.locked_tail_probe) {
            // Failure path only.  The leading SYNCs are all below threshold --
            // typically a burst interferer parked on the first symbols while
            // the tail of the preamble is clean (measured on the 5.2 m
            // capture: first_5_max 0.029..0.053, med_tail_32_255 0.14..0.20).
            // The old code declared a tracking failure without ever looking
            // past `locked_inner_symbols`.  Keep the bounded ROI (the grid is
            // known, so only the residual is unknown) and probe sparse tail
            // anchors on the symbol grid.
            //
            // The threshold is the TRACKING threshold, not the acquisition
            // one: a low-SNR tail sits at 0.14..0.20, well under
            // radar_verification_threshold (0.25) but far above the per-SYNC
            // tracking floor.  The neighbour gate below is what keeps a
            // +/-roi false alarm from being accepted at this lower threshold.
            const float tail_thr =
                std::min(profile.radar_verification_threshold, 0.08f);
            const size_t reps = profile.preamble_repetitions;
            const size_t step =
                std::max<size_t>(1, profile.locked_tail_probe_step);
            const size_t need_confirm =
                std::max<size_t>(1, profile.locked_tail_probe_confirm);
            // Two tiers: the locked residual is a few samples, so a narrow
            // probe finds the clean tail cheaply; only when the whole narrow
            // sweep fails do we pay for the full bounded ROI.
            const int64_t tail_radii[2] = {std::min<int64_t>(roi, 32), roi};
            for (int tier = 0; tier < 2 && !found; ++tier) {
                if (tier == 1 && tail_radii[1] <= tail_radii[0])
                    break;
                const int64_t trad = tail_radii[tier];
                for (size_t m = inner + 1; m < reps; m += step) {
                    const int64_t want = static_cast<int64_t>(seed_start) +
                        static_cast<int64_t>(m) *
                            static_cast<int64_t>(kQm35SamplesPerSymbol);
                    if (want + static_cast<int64_t>(L) >
                        static_cast<int64_t>(n))
                        break;
                    ++out.tail_probe_attempts;
                    int64_t cpos = -1;
                    float cmet = 0.0f;
                    std::complex<float> ccorr(0.0f, 0.0f);
                    if (!local_best(want, trad, tail_thr, &cpos, &cmet, &ccorr,
                                    /*allow_par_fallback=*/true))
                        continue;
                    if (cmet > out.tail_probe_best_metric)
                        out.tail_probe_best_metric = cmet;
                    // Consistency gate: a real grid hit is confirmed by its
                    // neighbouring SYMBOLS.  A false alarm (wrong lag inside
                    // the ROI) is not, so probing continues instead of
                    // locking onto a bogus origin.
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
                        if (local_best(nb_want, trad, tail_thr, &np, &nm, &nc))
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
                    break;
                }
            }
        }
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
            if (!profile.locked_tail_probe ||
                !try_fold_acquire(
                    static_cast<int64_t>(seed_start),
                    std::max<int64_t>(1, static_cast<int64_t>(
                                             profile.locked_roi_samples)))) {
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
            float pw = 0.0f;
            for (size_t k = 0; k < L; ++k)
                pw += std::norm(rx[j_lo + static_cast<int64_t>(k)]);
            float best_coarse_m = -1.0f;
            int64_t best_coarse_j = j_lo;
            std::complex<float> best_coarse_c(0.0f, 0.0f);
            for (int64_t j = j_lo; j <= j_hi; ++j) {
                if (((j - j_lo) % stride) == 0) {
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
                }
                if (j + 1 <= j_hi)
                    pw += std::norm(rx[j + static_cast<int64_t>(L)]) -
                          std::norm(rx[j]);
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
    // Windowed |x|^2 with eight independent accumulators.  A single-accumulator
    // float reduction cannot be vectorised without -ffast-math, and the power
    // turned out to be HALF the cost of the single-point stage (measured 2x
    // faster with eight accumulators).  The summation order differs from the
    // scalar loop, but this value only normalises the grid metric; the legacy
    // path keeps its own O(1) power update untouched.
    auto window_power = [](const std::complex<float>* x, size_t count) -> float {
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
    };
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
            float pw = window_power(rx + base, L);
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
    if (!grid_applied && !fold_used) {
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
    if (!bounded && !fold_used && !grid_applied &&
        out.peak_samples.size() <
            std::max<size_t>(4, profile.min_valid_peaks)) {
        out.peak_samples.clear();
        out.peak_metrics.clear();
        out.peak_corr.clear();
        try_fold_acquire(
            acq_seed,
            std::min<int64_t>(static_cast<int64_t>(profile.timing_search_margin),
                              8192));
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

// ---------------------------------------------------------------------------
// Stage 2 — CFO estimation + compensation (R1).
// Uses the stable SYNC peaks after the startup transient to fit phase vs time
// linearly; the slope / 2pi is the CFO in Hz.  MATLAB skips up to its first
// 24 peaks while retaining at least 32, controlled here by the separate CFO
// profile fields (not the CIR skip policy). The frame is then derotated by
// exp(-j2pi*f*n/fs) and phase-resolved against the preamble waveform. Golden
// reference: CFO = 0 Hz for the clean baseband signal.
// ---------------------------------------------------------------------------
// Decoupled CFO estimation (Task A1).
// Fits the CFO frequency from SYNC peaks without derotating samples.
inline bool fit_cfo(const std::complex<float>* rx,
                    size_t n,
                    const Qm35825Profile& profile,
                    const TimingResult& timing,
                    CfoResult& out,
                    DemodScratch& scratch)
{
    out = CfoResult{};
    const size_t np = timing.peak_samples.size();
    if (np < 4)
        return false;

    // Match MATLAB compensateCarrierOffset: discard up to the first 24 SYNC
    // correlation phases, but retain at least 32 for the least-squares fit.
    // For shorter preambles, keep all available peaks rather than failing.
    const size_t min_fit = std::max<size_t>(2, profile.cfo_min_fit_repetitions);
    const size_t available_to_skip = np > min_fit ? np - min_fit : 0;
    const size_t skip =
        std::min(profile.cfo_skip_initial_repetitions, available_to_skip);
    const size_t nfit = np - skip;
    if (nfit < 2)
        return false;
    out.skipped_peaks = skip;
    out.fit_first_peak_sample = timing.peak_samples[skip];
    out.fit_last_peak_sample = timing.peak_samples[np - 1];

    // Gather per-peak carrier phases from the SYNC matched-filter values (the
    // noise-averaged correlation phase, NOT the raw rx[peak] sample).  Falls
    // back to the raw sample if the timing stage did not populate peak_corr.
    const auto cfo_fit_begin = std::chrono::steady_clock::now();
    const bool have_corr = timing.peak_corr.size() >= np;
    scratch.metric.resize(nfit);
    for (size_t k = 0; k < nfit; ++k) {
        const size_t idx = skip + k;
        scratch.metric[k] =
            have_corr ? std::arg(timing.peak_corr[idx])
                      : std::arg(rx[timing.peak_samples[idx]]);
    }
    // Unwrap: adjacent SYNC phases must stay within ±π.  When LockedBounded
    // bridges a multi-symbol interference gap, estimate the per-sample phase
    // slope from adjacent 1-symbol pairs so multi-symbol gaps unwrap cleanly.
    double unwrap_slope_rad_per_sample = 0.0;
    {
        std::complex<double> adj_sum(0.0, 0.0);
        const double max_adj_dt = 1.5 * static_cast<double>(kQm35SamplesPerSymbol);
        for (size_t k = 1; k < nfit; ++k) {
            const size_t idx0 = skip + k - 1;
            const size_t idx1 = skip + k;
            const double dt = static_cast<double>(
                timing.peak_samples[idx1] - timing.peak_samples[idx0]);
            if (dt > 0.0 && dt <= max_adj_dt) {
                if (have_corr) {
                    const std::complex<double> c0(timing.peak_corr[idx0]);
                    const std::complex<double> c1(timing.peak_corr[idx1]);
                    adj_sum += c1 * std::conj(c0);
                } else {
                    const double dphi = static_cast<double>(scratch.metric[k] - scratch.metric[k - 1]);
                    adj_sum += std::complex<double>(std::cos(dphi), std::sin(dphi));
                }
            }
        }
        if (std::norm(adj_sum) > 1e-24) {
            unwrap_slope_rad_per_sample =
                std::arg(adj_sum) / static_cast<double>(kQm35SamplesPerSymbol);
        }
    }
    for (size_t k = 1; k < nfit; ++k) {
        const size_t idx0 = skip + k - 1;
        const size_t idx1 = skip + k;
        const double dt = static_cast<double>(
            timing.peak_samples[idx1] - timing.peak_samples[idx0]);
        const float pred = scratch.metric[k - 1] +
                           static_cast<float>(unwrap_slope_rad_per_sample * dt);
        while (scratch.metric[k] - pred > (float)M_PI)
            scratch.metric[k] -= 2.0f * (float)M_PI;
        while (scratch.metric[k] - pred < -(float)M_PI)
            scratch.metric[k] += 2.0f * (float)M_PI;
    }

    // Linear fit of phase vs RELATIVE time (the slope is offset-invariant, so
    // subtracting t0 only improves conditioning).
    const double t0 = (double)timing.peak_samples[skip] / profile.sample_rate;
    double sx = 0.0, sy = 0.0, sxx = 0.0, sxy = 0.0;
    for (size_t k = 0; k < nfit; ++k) {
        const size_t idx = skip + k;
        const double t =
            (double)timing.peak_samples[idx] / profile.sample_rate - t0;
        const double ph = (double)scratch.metric[k];
        sx += t;
        sy += ph;
        sxx += t * t;
        sxy += t * ph;
    }
    const double denom = (double)nfit * sxx - sx * sx;
    if (std::abs(denom) < 1e-30)
        return false;
    const double slope = ((double)nfit * sxy - sx * sy) / denom;
    out.cfo_hz = slope / (2.0 * M_PI);
    out.peaks_used = nfit;
    out.ok = true;
    out.fit_us = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - cfo_fit_begin).count());
    return true;
}

// Decoupled CFO range rotation (Task A1).
// Compensates [lo, hi) where phase origin is strictly sample 0.
inline void derotate_cfo_range(const std::complex<float>* rx,
                               size_t n,
                               const Qm35825Profile& profile,
                               double cfo_hz,
                               size_t lo,
                               size_t hi,
                               DemodScratch& scratch,
                               uint64_t& rotate_us,
                               uint32_t& rotation_kernel)
{
    const auto cfo_rotate_begin = std::chrono::steady_clock::now();
    if (scratch.derotated.size() < n)
        scratch.derotated.resize(n);
    hi = std::min(hi, n);
    const double w = 2.0 * M_PI * cfo_hz / profile.sample_rate;
    if (lo < hi) {
        if (profile.rx_fast_cfo_rotation) {
            rxopt::derotate_range_block(scratch.derotated.data(), rx, n, lo, hi, w);
            rotation_kernel = 1;
        } else {
            rxopt::derotate_range_recurrence(scratch.derotated.data(), rx, n, lo, hi, w);
            rotation_kernel = 0;
        }
    } else {
        rotation_kernel = profile.rx_fast_cfo_rotation ? 1 : 0;
    }
    rotate_us = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - cfo_rotate_begin).count());
}

inline bool stage_cfo(const std::complex<float>* rx,
                      size_t n,
                      const Qm35825Profile& profile,
                      const TimingResult& timing,
                      CfoResult& out,
                      DemodScratch& scratch)
{
    if (!fit_cfo(rx, n, profile, timing, out, scratch))
        return false;
    uint32_t rk = 0;
    derotate_cfo_range(rx, n, profile, out.cfo_hz, 0, n, scratch, out.rotate_us, rk);
    out.rotation_kernel = rk;
    return true;
}

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
        const bool use_real_sfd_fast =
            profile.rx_fast_sfd && scratch.real_sync_template.is_real;
        const float* taps_dup = use_real_sfd_fast ? scratch.real_sync_template.taps_dup.data() : nullptr;

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
                    } else if (window_best_j + len * sym <= n) {
                        // Low-floor fallback (0.02 <= pk < 0.08): verify that
                        // all SFD symbols coherently match sfd_sequence signs
                        // to reject shifted-SFD sidelobes (e.g. 0.0625).
                        std::complex<float> sym_corr[16];
                        const size_t n_check = std::min<size_t>(len, 16);
                        std::complex<double> coh_ref(0.0, 0.0);
                        for (size_t si = 0; si < n_check; ++si) {
                            const size_t pos = window_best_j + si * sym;
                            std::complex<float> c(0.0f, 0.0f);
                            if (use_real_sfd_fast) {
                                c = rxopt::real_template_dot_single(
                                    rx + pos, taps_dup, sym);
                            } else {
                                for (size_t k = 0; k < sym; ++k)
                                    c += rx[pos + k] * std::conj(template_wf[k]);
                            }
                            sym_corr[si] = c;
                            coh_ref += static_cast<double>(sfd_sequence[si]) *
                                       std::complex<double>(c);
                        }
                        if (std::norm(coh_ref) > 1e-24) {
                            size_t sign_ok = 0;
                            for (size_t si = 0; si < n_check; ++si) {
                                const double proj =
                                    static_cast<double>(sym_corr[si].real()) * coh_ref.real() +
                                    static_cast<double>(sym_corr[si].imag()) * coh_ref.imag();
                                if (proj * static_cast<double>(sfd_sequence[si]) > 0.0)
                                    ++sign_ok;
                            }
                            if (sign_ok == n_check)
                                window_passed = true;
                        }
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
    for (int k = 1; k <= kSfdSymbolOffsets && !sfd_passed; ++k) {
        const int64_t step = static_cast<int64_t>(k) * static_cast<int64_t>(sym);
        sfd_passed = scan_window(expected + step, kSfdNarrowHalf, k);
        if (sfd_passed)
            break;
        sfd_passed = scan_window(expected - step, kSfdNarrowHalf, k);
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

// ---------------------------------------------------------------------------
// Stage 4 — CIR estimation + soft-chip generation (R2).
// Mirrors MATLAB estimateCir + estimateCirAndSoftChips:
//
//   * CIR: coherently average the aligned raw SYNC windows of the LAST
//     cir_repetitions repetitions (skipping the first
//     cir_skip_initial_repetitions), then correlate each of the
//     (pre+post) taps against the code in NATURAL order:
//       values[n] = sum_m avg[n+m] * conj(sampled_code[m]) / code_energy
//     and L2-normalize.  (The template is the sparse sampled code, NOT the
//     pulse-shaped preamble waveform — forward order, not time-reversed.)
//
//   * Soft chips: causal FIR with cirMf = conj(flip(values)) evaluated on the
//     chip grid chipStart = start + (pre+post) - pre - 1, spaced
//     measured_period/chips_per_symbol apart, up to the frame-span budget
//     bounded by the buffer length.  The complex chips are then phase-aligned
//     against the last min(32, preamble_repetitions) SYNCs' spread code, and
//     the real part is normalized by max|real|.
//
// Coordinate convention: rx uses 0-based ABSOLUTE sample indices (the full
// scheduled window / captured PDU), so chipStart and the CIR first-path are
// absolute.  Verified against the MATLAB golden reference: CIR max diff
// ~6e-8, soft-chip stream max diff ~6e-7 (stage_cir.mat / stage_softchips.mat).
// ---------------------------------------------------------------------------
inline bool stage_cir_softchips(const std::complex<float>* rx,
                                size_t n,
                                const Qm35825Profile& profile,
                                const TimingResult& timing,
                                const std::vector<int8_t>& preamble_code,
                                CirResult& out,
                                DemodScratch& scratch,
                                size_t max_chips = 0,
                                size_t begin_chip = 0)
{
    out = CirResult{};
    // On-demand experiment: max_chips != 0 limits this call to a prefix or an
    // extension.  begin_chip > 0 means the prefix [0, begin_chip) was already
    // produced and normalized by an earlier call; only [begin_chip, num_chips)
    // is computed here.  Defaults (0, 0) preserve the legacy full-stream path.
    const bool on_demand =
        (profile.rx_softchip_on_demand || profile.rx_fused_payload_cmf ||
         profile.rx_fused_phr_cmf) &&
        max_chips != 0;
    const bool extending = on_demand && begin_chip != 0;
    if (extending && !scratch.softchip.prepared)
        return false;
    if (on_demand && !extending)
        scratch.softchip.reset();
    auto t_cir0 = std::chrono::steady_clock::now();
    const size_t pre = profile.cir_pre_samples;
    const size_t post = profile.cir_post_samples;
    const size_t tap_count = pre + post;
    const double period = timing.measured_period;
    const int64_t start = timing.preamble_start_sample;
    if (period <= 0.0 || start < 0 || preamble_code.empty() || tap_count == 0)
        return false;

    // ---- Build (or reuse) the sparse sampled code (508 spread x2 -> 1016, zeros on the
    //      odd samples), matching MATLAB buildUwbReference.sampled_code. ----
    if (scratch.cached_seq.code_index != profile.code_index ||
        scratch.cached_seq.sampled_pcode.empty()) {
        scratch.cached_seq.update(profile.code_index, profile.sfd_mode);
    }
    const std::vector<int8_t>& spread = scratch.cached_seq.sampled_pcode;
    const uint64_t ckey =
        rxopt::cir_template_key(preamble_code.data(), preamble_code.size());
    if (scratch.cir_template.key != ckey) {
        rxopt::build_sparse_cir_template(
            preamble_code.data(), preamble_code.size(), scratch.cir_template);
    }
    const size_t code_len = 2 * kQm35ChipsPerSymbol; // 1016

    // ---- Coherently average the aligned SYNC windows. ----
    const size_t available =
        std::min(timing.detected_peaks, profile.preamble_repetitions);
    if (available == 0 || profile.cir_skip_initial_repetitions >= available)
        return false;
    const size_t first_rep = profile.cir_skip_initial_repetitions;
    const size_t rep_count = std::min(profile.cir_repetitions,
                                      available - first_rep);
    if (rep_count == 0)
        return false;
    // Match acceleration/estimateCir.m's explicit-skip branch: begin after
    // cir_skip_initial_repetitions and consume at most cir_repetitions.
    // The runtime profile sets count=preamble-skip when all remaining SYNCs
    // should participate.

    const size_t wlen = code_len + tap_count - 1; // 1053
    scratch.corr.assign(wlen, std::complex<float>(0.0f, 0.0f));
    size_t valid = 0;
    for (size_t k = first_rep; k < first_rep + rep_count; ++k) {
        const int64_t rs = start + static_cast<int64_t>(std::llround((double)k * period));
        const int64_t lo = rs - static_cast<int64_t>(pre);
        const int64_t hi = lo + static_cast<int64_t>(wlen);
        if (lo < 0 || hi > static_cast<int64_t>(n))
            continue; // drop SYNCs clipped by the buffer edge (like MATLAB)
        rxopt::vector_accumulate_complex(
            scratch.corr.data(),
            rx + static_cast<size_t>(lo),
            wlen);
        ++valid;
    }
    if (valid == 0)
        return false;
    const float inv = 1.0f / static_cast<float>(valid);
    rxopt::vector_scale_complex(scratch.corr.data(), inv, wlen);

    // ---- CIR taps: sparse ternary template correlation (Task A2). ----
    std::vector<std::complex<float>> values;
    rxopt::correlate_cir_sparse(scratch.corr.data(), scratch.cir_template,
                                tap_count, values);
    float nrm = 0.0f;
    for (size_t i = 0; i < tap_count; ++i)
        nrm += std::norm(values[i]);
    nrm = std::sqrt(nrm);
    if (!(nrm > 0.0f))
        return false;
    for (size_t i = 0; i < tap_count; ++i)
        values[i] /= (nrm + 1e-12f);

    size_t peak_idx = 0;
    float peak_mag = -1.0f;
    for (size_t i = 0; i < tap_count; ++i) {
        const float m = std::abs(values[i]);
        if (m > peak_mag) {
            peak_mag = m;
            peak_idx = i;
        }
    }
    out.first_path_sample =
        static_cast<size_t>(std::max<int64_t>(0, start + (int64_t)peak_idx - (int64_t)pre));
    out.pre_samples = pre;
    out.post_samples = post;
    out.cir_peak_metric = peak_mag;
    out.cir_values.resize(tap_count);
    out.cir_filter_taps.resize(tap_count);
    for (size_t i = 0; i < tap_count; ++i) {
        out.cir_values[i] = values[i].real(); // golden CIR is real (Q=0 channel)
        out.cir_filter_taps[i] = std::conj(values[i]);
    }
    out.cir_estimate_us = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - t_cir0)
            .count());
    auto t_fir0 = std::chrono::steady_clock::now();

    // ---- Soft-chip matched filter on the chip grid. ----
    const double spc = period / static_cast<double>(kQm35ChipsPerSymbol);
    // Frame-span chip budget (estimateFrameSampleSpan): 4z2 SFD = 8 symbols.
    const size_t sfd_symbols = 8;
    const size_t n_shr = (profile.preamble_repetitions + sfd_symbols) * kQm35ChipsPerSymbol;
    const size_t n_phr_head = 40 * kQm35ChipsPerSymbol;
    const size_t extra_bytes =
        (profile.max_psdu_bytes > 12) ? (profile.max_psdu_bytes - 12) : 0;
    const size_t n_extra = static_cast<size_t>(std::ceil((double)extra_bytes * 1000.0));
    const size_t n_guard = 8 * kQm35ChipsPerSymbol;
    size_t n_chips = n_shr + n_phr_head + n_extra + n_guard;
    n_chips = static_cast<size_t>(std::ceil((double)n_chips * 1.05));
    if (on_demand) {
        if (max_chips < 2)
            return false;
        if (n_chips > max_chips)
            n_chips = max_chips;
    }

    const int64_t chip_start = start + static_cast<int64_t>(tap_count) -
                               static_cast<int64_t>(pre) - 1; // = start + post - 1
    const int64_t last_by_budget =
        chip_start + static_cast<int64_t>(std::ceil((double)(n_chips - 1) * spc));
    const int64_t last_chip_sample = std::min<int64_t>(static_cast<int64_t>(n), last_by_budget);
    const int64_t last_pos = last_chip_sample - 1; // inclusive, 0-based
    if (last_pos < chip_start)
        return false;
    const int64_t spc_i = std::max<int64_t>(1, std::llround(spc));
    // Integer grid unless the clock offset would drift the grid past the
    // correlation tolerance across the longest frame (see integer_chip_grid).
    const bool fractional_grid =
        !integer_chip_grid(spc, profile.fused_gate_ppm * 1e-6);
    const size_t num_chips =
        static_cast<size_t>(std::floor(
            static_cast<double>(last_pos - chip_start) / spc)) + 1;
    if (on_demand &&
        num_chips > static_cast<size_t>(std::numeric_limits<uint32_t>::max()))
        return false; // result chip counters are uint32_t

    if (extending && num_chips <= begin_chip) {
        // The PHR-derived range already fits in the prefix.  Nothing to do;
        // the existing normalized prefix remains the final stream.
        out.soft_chip_count = begin_chip;
        out.initial_chip_count = static_cast<uint32_t>(begin_chip);
        out.extended_chip_count = static_cast<uint32_t>(begin_chip);
        out.on_demand_extended = 0;
        out.samples_per_chip = spc;
        out.ok = true;
        return true;
    }

    // Causal FIR on the chip grid, rewritten for contiguous forward access:
    //   chip = sum_k conj(values[T-1-k]) * rx[base-k]
    //        = sum_q conj(values[q])     * rx[base-(T-1)+q]
    // Pre-conjugated taps h[q]=conj(values[q]) once outside the chip loop.
    // Kernels: multi-acc / VOLK / fixed-38 AVX2 (see uwb_cir_fir_simd.h).
    //
    // On-demand uses a separate output buffer: the prefix must survive the
    // extension call's CIR re-estimate (which temporarily uses scratch.corr).
    std::vector<std::complex<float>>& fir_out =
        on_demand ? scratch.softchip_corr : scratch.corr;
    fir_out.resize(num_chips);
    const int64_t fwd = static_cast<int64_t>(tap_count) - 1;

    // Stack buffers — no heap alloc in the FIR hot path.
    // h_conj[q] = conj(values[q]) for multi-acc / AVX2; values_nat for VOLK.
    alignas(32) std::complex<float> h_conj[64];
    alignas(32) std::complex<float> values_nat[64];
    if (tap_count > 64)
        return false;
    for (size_t q = 0; q < tap_count; ++q) {
        values_nat[q] = values[q];
        h_conj[q] = std::conj(values[q]);
    }

    // Optional sparse RAKE: choose the strongest complex CIR taps once per
    // packet. Keep indices in ascending delay order for predictable loads;
    // weights retain their estimated complex phases for coherent combining.
    alignas(32) uint8_t rake_indices[64];
    alignas(32) std::complex<float> rake_weights[64];
    const bool bypass_filter =
        profile.cir_soft_chip_mode == CirSoftChipMode::Bypass;
    const bool use_rake =
        profile.cir_soft_chip_mode == CirSoftChipMode::Rake ||
        (profile.cir_soft_chip_mode == CirSoftChipMode::Auto &&
         profile.cir_rake_top_k > 0);
    const size_t rake_k = use_rake
        ? std::min(profile.cir_rake_top_k, tap_count)
        : size_t(0);
    if (rake_k > 0 && rake_k < tap_count) {
        std::array<uint8_t, 64> order{};
        for (size_t q = 0; q < tap_count; ++q)
            order[q] = static_cast<uint8_t>(q);
        std::partial_sort(order.begin(), order.begin() + rake_k,
                          order.begin() + tap_count,
                          [&](uint8_t a, uint8_t b) {
                              return std::norm(values[a]) > std::norm(values[b]);
                          });
        std::sort(order.begin(), order.begin() + rake_k);
        for (size_t i = 0; i < rake_k; ++i) {
            rake_indices[i] = order[i];
            rake_weights[i] = h_conj[order[i]];
        }
    }
    // The on-demand experiment is limited to the production integer-grid,
    // full-tap kernel.  Fractional SFO grids, sparse RAKE and bypass keep the
    // complete legacy path, so their semantics are not changed or rounded.
    if (on_demand && (fractional_grid || bypass_filter || rake_k > 0))
        return false;

    // Only NS-SFD still consumes a soft-chip stream on the fused PHR path.
    // Keep its -8-chip search halo, not a filtered copy of every SYNC.
    const bool skip_sync_fir = on_demand && !extending &&
        profile.rx_fused_phr_cmf && profile.rx_fused_payload_cmf && spc_i == 2;
    const size_t sync_end = profile.preamble_repetitions * kQm35ChipsPerSymbol;
    const size_t fir_first = skip_sync_fir && sync_end > 8 ? sync_end - 8 : 0;
    if (fir_first >= num_chips)
        return false;
    size_t checked = 0;
    if (fractional_grid) {
        // MATLAB keeps measured_period/chips_per_symbol as a fractional
        // position and linearly interpolates the CIR-filtered stream.  Keep
        // the optimized integer-grid kernels below for the common exact-2
        // case; use the scalar interpolation path only when SFO is present.
        auto dot_at = [&](int64_t p) {
            if (bypass_filter) {
                const int64_t idx =
                    p - fwd + static_cast<int64_t>(peak_idx);
                return idx >= 0 && idx < static_cast<int64_t>(n)
                    ? rx[static_cast<size_t>(idx)]
                    : std::complex<float>(0.0f, 0.0f);
            }
            std::complex<float> acc(0.0f, 0.0f);
            const size_t count =
                (rake_k > 0 && rake_k < tap_count) ? rake_k : tap_count;
            for (size_t j = 0; j < count; ++j) {
                const size_t q = (count == tap_count) ? j : rake_indices[j];
                const int64_t idx = p - fwd + static_cast<int64_t>(q);
                if (idx >= 0 && idx < static_cast<int64_t>(n))
                    acc += h_conj[q] * rx[static_cast<size_t>(idx)];
            }
            return acc;
        };
        for (size_t i = 0; i < num_chips; ++i) {
            const double pd = static_cast<double>(chip_start) +
                              static_cast<double>(i) * spc;
            const int64_t p0 = static_cast<int64_t>(std::floor(pd));
            const float frac = static_cast<float>(pd - std::floor(pd));
            const std::complex<float> y0 = dot_at(p0);
            const std::complex<float> y1 = dot_at(p0 + 1);
            fir_out[i] = y0 + frac * (y1 - y0);
        }
        checked = num_chips; // fractional path completed the whole stream
    } else if (chip_start < fwd) {
        const int64_t gap = fwd - chip_start;
        checked = std::min(num_chips,
                           static_cast<size_t>((gap + spc_i - 1) / spc_i));
    }
    if (extending && begin_chip < checked)
        return false; // [begin_chip, checked) would never be generated
    if (extending)
        checked = std::min(num_chips, begin_chip);
    // Edge chips: FIR window may clip below sample 0 (bounds-checked scalar).
    // A prefix call handles them once; an extension starts after them.
    for (size_t i = extending ? checked : 0;
         !fractional_grid && !skip_sync_fir && i < checked; ++i) {
        const int64_t p = chip_start + static_cast<int64_t>(i) * spc_i;
        if (bypass_filter) {
            const int64_t idx =
                p - fwd + static_cast<int64_t>(peak_idx);
            fir_out[i] = idx >= 0 && idx < static_cast<int64_t>(n)
                ? rx[static_cast<size_t>(idx)]
                : std::complex<float>(0.0f, 0.0f);
            continue;
        }
        std::complex<float> acc(0.0f, 0.0f);
        const size_t edge_k =
            (rake_k > 0 && rake_k < tap_count) ? rake_k : tap_count;
        for (size_t j = 0; j < edge_k; ++j) {
            const size_t q = (edge_k == tap_count) ? j : rake_indices[j];
            const int64_t idx = p - fwd + static_cast<int64_t>(q);
            if (idx < 0)
                continue;
            acc += h_conj[q] * rx[static_cast<size_t>(idx)];
        }
        fir_out[i] = acc;
    }
    if (skip_sync_fir)
        checked = std::max(checked, fir_first);
    out.soft_fir_first_chip = static_cast<uint32_t>(checked);
    out.soft_fir_computed_chips = static_cast<uint32_t>(
        num_chips - (skip_sync_fir ? fir_first : (extending ? begin_chip : 0)));
    // Main body: full tap window is in-bounds.  Call the selected kernel
    // directly (no per-chip switch) so the fixed-38 AVX2 path fully inlines.
    {
        const cir_fir::Kernel fir_k = cir_fir::kDefaultKernel;
        const std::complex<float>* rx0 = rx;
        if (bypass_filter) {
            for (size_t i = checked; i < num_chips; ++i) {
                const int64_t p =
                    chip_start + static_cast<int64_t>(i) * spc_i;
                const size_t idx = static_cast<size_t>(
                    p - fwd + static_cast<int64_t>(peak_idx));
                fir_out[i] = rx0[idx];
            }
        } else if (rake_k > 0 && rake_k < tap_count) {
            size_t i = checked;
#if UWB_CIR_FIR_HAVE_AVX2
            // Fixed Top-4/Top-8 fast path: vectorize across four outputs. The
            // load helper reads through max_q+7; keep the final boundary on
            // the scalar path when that speculative contiguous load is unsafe.
            if ((rake_k == 4 || rake_k == 8) && spc_i == 2) {
                const size_t max_q = rake_indices[rake_k - 1];
                for (; i + 4 <= num_chips; i += 4) {
                    const int64_t p =
                        chip_start + static_cast<int64_t>(i) * spc_i;
                    const size_t win = static_cast<size_t>(p - fwd);
                    if (win + max_q + 7 >= n)
                        break;
                    if (rake_k == 4)
                        cir_fir::dot_topk_x4_avx2<4>(
                            rx0 + win, rake_indices, rake_weights,
                            fir_out.data() + i);
                    else
                        cir_fir::dot_topk_x4_avx2<8>(
                            rx0 + win, rake_indices, rake_weights,
                            fir_out.data() + i);
                }
            }
#endif
            for (; i < num_chips; ++i) {
                const int64_t p = chip_start + static_cast<int64_t>(i) * spc_i;
                const size_t win = static_cast<size_t>(p - fwd);
                fir_out[i] = cir_fir::dot_topk(
                    rx0 + win, rake_indices, rake_weights, rake_k);
            }
        } else if (fir_k == cir_fir::Kernel::Avx2Fixed &&
            tap_count == cir_fir::kDefaultTapCount) {
            for (size_t i = checked; i < num_chips; ++i) {
                const int64_t p = chip_start + static_cast<int64_t>(i) * spc_i;
                const size_t win = static_cast<size_t>(p - fwd);
                fir_out[i] =
                    cir_fir::dot_avx2_fixed38(rx0 + win, h_conj, tap_count);
            }
        } else if (fir_k == cir_fir::Kernel::Volk) {
            for (size_t i = checked; i < num_chips; ++i) {
                const int64_t p = chip_start + static_cast<int64_t>(i) * spc_i;
                const size_t win = static_cast<size_t>(p - fwd);
                fir_out[i] =
                    cir_fir::dot_volk(rx0 + win, values_nat, tap_count);
            }
        } else {
            for (size_t i = checked; i < num_chips; ++i) {
                const int64_t p = chip_start + static_cast<int64_t>(i) * spc_i;
                const size_t win = static_cast<size_t>(p - fwd);
                fir_out[i] =
                    cir_fir::dot_multi_acc8(rx0 + win, h_conj, tap_count);
            }
        }
    }
    out.soft_fir_us = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - t_fir0)
            .count());
    auto t_post0 = std::chrono::steady_clock::now();

    // ---- Phase-align against the last min(32, preamble) SYNCs' spread code. ----
    const size_t phase_reps = std::min<size_t>(32, profile.preamble_repetitions);
    const size_t phase_first =
        (profile.preamble_repetitions - phase_reps) * kQm35ChipsPerSymbol;
    const size_t phase_len = phase_reps * kQm35ChipsPerSymbol;
    if (phase_first + phase_len > num_chips)
        return false; // chip stream shorter than the configured preamble
    // spread[] is real ±1/0, so conj(spread)*corr = spread*corr.  Avoid % by
    // walking the 508-chip period with a running index.
    std::complex<float> rot(1.0f, 0.0f);
    if (extending) {
        // Frozen by the prefix pass: phase alignment runs once per packet.
        rot = scratch.softchip.rot;
    } else if (skip_sync_fir) {
        const auto phase_begin = std::chrono::steady_clock::now();
        std::complex<float> gain{};
        if (!rxopt::sync_phase_gain(rx, n,
                chip_start - fwd + static_cast<int64_t>(2 * phase_first),
                h_conj, tap_count, spread, phase_reps, scratch.work, gain))
            return false;
        const float ang = std::arg(gain);
        rot = std::complex<float>(std::cos(-ang), std::sin(-ang));
        scratch.softchip.rot = rot;
        out.sync_phase_us = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::steady_clock::now() - phase_begin).count());
    } else {
        std::complex<float> gain(0.0f, 0.0f);
        size_t s = 0;
        for (size_t i = 0; i < phase_len; ++i) {
            gain += static_cast<float>(spread[s]) *
                    fir_out[phase_first + i];
            if (++s == kQm35ChipsPerSymbol)
                s = 0;
        }
        const float ang = std::arg(gain);
        rot = std::complex<float>(std::cos(-ang), std::sin(-ang));
        if (on_demand)
            scratch.softchip.rot = rot;
    }
    const float rr = rot.real();
    const float ri = rot.imag();

    // Merged postprocess: compute only the rotated real component and track
    // max|real|, then normalize into soft_chips. No later stage consumes the
    // rotated complex corr stream, so writing it back wastes ~1.2 MB/packet.
    scratch.soft_chips.resize(num_chips);
    if (!extending) {
        // Preserve absolute chip coordinates; the skipped prefix is not data.
        // NS-SFD uses normalized correlation, invariant to this common scale.
        const size_t post_first = skip_sync_fir ? fir_first : 0;
        std::fill(scratch.soft_chips.begin(),
                  scratch.soft_chips.begin() + post_first, 0.0f);
        float mx = 0.0f;
        for (size_t i = post_first; i < num_chips; ++i) {
            // (cr+j ci)*(rr+j ri) → real = cr*rr - ci*ri
            const float cr = fir_out[i].real();
            const float ci = fir_out[i].imag();
            const float r = cr * rr - ci * ri;
            scratch.soft_chips[i] = r;                // temp; normalized below
            const float ar = (r >= 0.0f) ? r : -r;
            if (ar > mx)
                mx = ar;
        }
        const float inv_denom = 1.0f / (mx + 1e-12f);
        for (size_t i = post_first; i < num_chips; ++i)
            scratch.soft_chips[i] *= inv_denom;
        if (on_demand) {
            scratch.softchip.prefix_max = mx;
            scratch.softchip.generated = num_chips;
            scratch.softchip.prepared = true;
            out.initial_chip_count = static_cast<uint32_t>(num_chips);
            out.extended_chip_count = 0;
            out.on_demand_extended = 0;
        }
    } else {
        // Extension: rotate only the new chips, then put the whole stream on
        // one denominator.  rescale_prefix() multiplies the already-normalized
        // prefix by (prefix_max+eps)/(final_max+eps); the new tail is scaled by
        // 1/(final_max+eps).  This equals the one-shot raw/final_max result up
        // to one float rounding on the prefix.
        float ext_mx = 0.0f;
        for (size_t i = begin_chip; i < num_chips; ++i) {
            const float cr = fir_out[i].real();
            const float ci = fir_out[i].imag();
            const float r = cr * rr - ci * ri;
            scratch.soft_chips[i] = r;
            const float ar = (r >= 0.0f) ? r : -r;
            if (ar > ext_mx)
                ext_mx = ar;
        }
        const float final_max = std::max(scratch.softchip.prefix_max, ext_mx);
        rxopt::rescale_prefix(scratch.soft_chips, begin_chip,
                              scratch.softchip.prefix_max, final_max);
        const float inv_denom = 1.0f / (final_max + 1e-12f);
        for (size_t i = begin_chip; i < num_chips; ++i)
            scratch.soft_chips[i] *= inv_denom;
        scratch.softchip.prefix_max = final_max;
        scratch.softchip.generated = num_chips;
        out.initial_chip_count = static_cast<uint32_t>(begin_chip);
        out.extended_chip_count = static_cast<uint32_t>(num_chips);
        out.on_demand_extended = 1;
    }
    out.postprocess_us = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - t_post0)
            .count());

    out.soft_chip_count = num_chips;
    out.samples_per_chip = spc;
    out.ok = true;
    return true;
}

// ---------------------------------------------------------------------------
// R3 helpers: BPRF spreading LFSR, CL-3 Viterbi, and SECDED parity.
// All verified against the MATLAB golden reference (phr_ref2.mat / decode_uwb).
// ---------------------------------------------------------------------------
namespace detail {

// ---------------------------------------------------------------------------
// Payload sub-stage timing helpers (diagnostic only).
//
// `PayloadSubTimer` is an RAII scope guard over the whole payload stage: it
// snapshots the four named child buckets on construction and, on destruction,
// adds whatever is left of the enclosing interval to `other_us`.  The five
// buckets are therefore exactly additive whatever early return the enclosing
// function takes, and the guard costs two steady_clock reads per payload stage
// with no allocation and no change to any algorithm path.
//
// `PayloadSubBucket` attributes the elapsed time of a nested scope to one named
// bucket (e.g. the fused CMF plan + correlation).  Nested buckets are children
// of the same struct, so the outer guard's remainder automatically shrinks by
// the same amount: no double counting.
// ---------------------------------------------------------------------------
inline uint64_t payload_sub_total(const PayloadSubTimers& s)
{
    // Includes `other_us` so nested guards stay additive: an inner guard adds
    // its remainder to `other_us`, and the outer guard subtracts everything
    // already attributed (including that remainder) from its own interval.
    return s.soft_us + s.viterbi_us + s.rs_us + s.crc_us + s.other_us;
}

// Elapsed microseconds since `t0`, truncated exactly like the parent laps.
inline uint64_t payload_elapsed_us(std::chrono::steady_clock::time_point t0)
{
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - t0)
            .count());
}

class PayloadSubTimer
{
public:
    explicit PayloadSubTimer(PayloadSubTimers* s)
        : s_(s),
          t0_(s ? std::chrono::steady_clock::now()
                : std::chrono::steady_clock::time_point()),
          base_(s ? payload_sub_total(*s) : 0)
    {
    }
    ~PayloadSubTimer()
    {
        if (!s_)
            return;
        const uint64_t total = payload_elapsed_us(t0_);
        const uint64_t spent = payload_sub_total(*s_) - base_;
        if (total > spent)
            s_->other_us += total - spent;
    }
    PayloadSubTimer(const PayloadSubTimer&) = delete;
    PayloadSubTimer& operator=(const PayloadSubTimer&) = delete;

private:
    PayloadSubTimers* s_;
    std::chrono::steady_clock::time_point t0_;
    uint64_t base_;
};

class PayloadSubBucket
{
public:
    PayloadSubBucket(PayloadSubTimers* s, uint64_t PayloadSubTimers::*member)
        : s_(s),
          member_(member),
          t0_(s ? std::chrono::steady_clock::now()
                : std::chrono::steady_clock::time_point())
    {
    }
    ~PayloadSubBucket()
    {
        if (s_)
            s_->*member_ += payload_elapsed_us(t0_);
    }
    PayloadSubBucket(const PayloadSubBucket&) = delete;
    PayloadSubBucket& operator=(const PayloadSubBucket&) = delete;

private:
    PayloadSubTimers* s_;
    uint64_t PayloadSubTimers::*member_;
    std::chrono::steady_clock::time_point t0_;
};

// IEEE 802.15.4a BPRF data-scrambler spreading (lrwpan.internal.createScrambler):
// a 15-bit LFSR with s[i] = s[i-14] ^ s[i-15].  The initial state depends
// on the preamble code index; values below were exported from MATLAB R2025b
// lrwpan.internal.createScrambler for codes 9–12. This distinction is required
// for DW1000 PHR/payload data.
// Returns the +/-1 spreading stream over [start_bit, start_bit+nbits).
// The PHR field uses offset 0; the payload field is offset by the number of
// PHR active chips (21 symbols x 64 chips/burst = 1344).
inline bool bprf_spreading(std::vector<int8_t>& out, size_t code_index,
                           size_t start_bit, size_t nbits)
{
    static const int8_t kInit9[15] = {
        0, 1, 0, 0, 0, 0, 1, 0, 0, 1, 1, 1, 1, 0, 1
    };
    static const int8_t kInit10[15] = {
        0, 0, 1, 1, 0, 0, 1, 0, 0, 0, 0, 1, 1, 1, 1
    };
    static const int8_t kInit11[15] = {
        1, 1, 1, 1, 0, 0, 1, 0, 0, 1, 1, 0, 0, 1, 1
    };
    static const int8_t kInit12[15] = {
        1, 0, 0, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 0, 0
    };
    const int8_t* init = nullptr;
    if (code_index == 9)
        init = kInit9;
    else if (code_index == 10)
        init = kInit10;
    else if (code_index == 11)
        init = kInit11;
    else if (code_index == 12)
        init = kInit12;
    else
        return false;
    std::vector<int8_t> s(start_bit + nbits);
    for (size_t i = 0; i < 15 && i < s.size(); ++i)
        s[i] = init[i];
    for (size_t i = 15; i < s.size(); ++i)
        s[i] = static_cast<int8_t>(s[i - 14] ^ s[i - 15]);
    out.resize(nbits);
    for (size_t i = 0; i < nbits; ++i)
        out[i] = s[start_bit + i] ? int8_t(-1) : int8_t(1); // bit 1 -> -1
    return true;
}

// Rate-1/2 constraint-length-3 Viterbi decoder for poly2trellis(3,[2 5])
// (IEEE 802.15.4a PHR).  Hard decision, 'trunc' mode.  `rx` is the interleaved
// received code bits (length 2*N); output is the N decoded systematic bits.
inline void vitdec_cl3(const int8_t* rx, size_t N, std::vector<int8_t>& out)
{
    // octal 2 = 010 -> g0 = [0 1 0]; octal 5 = 101 -> g1 = [1 0 1].
    int o0[4][2], o1[4][2], nxt[4][2];
    for (int st = 0; st < 4; ++st) {
        const int s1 = st >> 1, s0 = st & 1;
        for (int u = 0; u < 2; ++u) {
            o0[st][u] = s1;
            o1[st][u] = u ^ s0;
            nxt[st][u] = (u << 1) | s1;
        }
    }
    const int INF = 1000000;
    int surf[4] = { 0, INF, INF, INF };
    std::vector<int8_t> back(N * 4);
    for (size_t t = 0; t < N; ++t) {
        int ns[4] = { INF, INF, INF, INF };
        for (int st = 0; st < 4; ++st) {
            for (int u = 0; u < 2; ++u) {
                const int d = (rx[2 * t] ^ o0[st][u]) + (rx[2 * t + 1] ^ o1[st][u]);
                const int m = surf[st] + d;
                const int nst = nxt[st][u];
                if (m < ns[nst]) {
                    ns[nst] = m;
                    back[t * 4 + nst] = static_cast<int8_t>((u << 2) | st);
                }
            }
        }
        for (int i = 0; i < 4; ++i)
            surf[i] = ns[i];
    }
    int best = 0;
    for (int i = 1; i < 4; ++i)
        if (surf[i] < surf[best])
            best = i;
    out.resize(N);
    int cur = best;
    for (size_t t = N; t-- > 0;) {
        const int v = back[t * 4 + cur];
        out[t] = static_cast<int8_t>(v >> 2);
        cur = v & 3;
    }
}

// SECDED encode: 13 systematic bits -> 19-bit codeword (13 data + 6 parity).
// Parity matrix rows from lrwpan.internal.hrpSECDED (verified against golden).
inline void hrp_secded(const std::vector<int8_t>& sys13,
                       std::vector<int8_t>& phr19)
{
    static const uint16_t kParity[6] = {
        0b1110110100111, // p1
        0b0000000000011, // p2
        0b0000111111100, // p3
        0b0111000111100, // p4
        0b1011011001101, // p5
        0b1101101010110, // p6
    };
    phr19.resize(19);
    for (int i = 0; i < 13; ++i)
        phr19[i] = sys13[i];
    for (int r = 0; r < 6; ++r) {
        int p = 0;
        for (int i = 0; i < 13; ++i)
            if (kParity[r] & (1u << (12 - i)))
                p ^= sys13[i];
        phr19[13 + r] = static_cast<int8_t>(p);
    }
}

// ---------------------------------------------------------------------------
// Shared BPM-BPSK BPRF demod kernel (used by both the PHR and payload fields).
// soft: chip-rate soft chips (0-based absolute).  start: field start chip.
// nsym: number of BPM-BPSK symbols.  cpb: chips per burst.  cps: chips per
// symbol (BPM half-symbol offset = cps/2).  scram_offset: LFSR start bit
// (PHR=0, payload=1344).  pol: sfd polarity.  Fills g0 (BPM position bit) and
// g1 (BPSK polarity bit), each nsym entries.  Mirrors helperUWBBPRFDemodKernel.
// ---------------------------------------------------------------------------
inline bool bprf_demod(const std::vector<float>& soft, int64_t start,
                       size_t code_index, size_t nsym, size_t cpb, size_t cps,
                       size_t scram_offset, int8_t pol, std::vector<int8_t>& g0,
                       std::vector<int8_t>& g1)
{
    if (start < 0 ||
        static_cast<size_t>(start + static_cast<int64_t>(cps * nsym)) >
            soft.size())
        return false;
    std::vector<int8_t> spread;
    if (!bprf_spreading(spread, code_index, scram_offset, cpb * nsym))
        return false;
    g0.resize(nsym);
    g1.resize(nsym);
    const size_t third = cps / 2;
    for (size_t s = 0; s < nsym; ++s) {
        const size_t base = static_cast<size_t>(start) + s * cps;
        const int8_t* sp = &spread[s * cpb];
        float b0 = -1e30f, b1 = -1e30f, m0 = 0.0f, m1 = 0.0f;
        for (size_t hop = 0; hop < 2; ++hop) {
            const size_t f0 = hop * cpb;
            float metric0 = 0.0f, metric1 = 0.0f;
            for (size_t c = 0; c < cpb; ++c) {
                metric0 += pol * soft[base + f0 + c] * static_cast<float>(sp[c]);
                metric1 += pol * soft[base + third + f0 + c] *
                           static_cast<float>(sp[c]);
            }
            if (std::abs(metric0) > b0) {
                b0 = std::abs(metric0);
                m0 = metric0;
            }
            if (std::abs(metric1) > b1) {
                b1 = std::abs(metric1);
                m1 = metric1;
            }
        }
        float best;
        if (b0 >= b1) {
            g0[s] = 0;
            best = m0;
        } else {
            g0[s] = 1;
            best = m1;
        }
        g1[s] = (best < 0.0f) ? 1 : 0;
    }
    return true;
}

// Soft-output variant of bprf_demod.  llr_g0 / llr_g1 > 0 favour bit value 0.
//   llr_g0 = |metric_half0| - |metric_half1|   (BPM position)
//   llr_g1 = chosen_half_metric                 (BPSK polarity; + => bit 0)
// Hard decisions still written to g0/g1 for callers that need them.
inline bool bprf_demod_soft(const std::vector<float>& soft, int64_t start,
                            size_t code_index, size_t nsym, size_t cpb, size_t cps,
                            size_t scram_offset, int8_t pol,
                            std::vector<int8_t>& g0, std::vector<int8_t>& g1,
                            std::vector<float>& llr_g0, std::vector<float>& llr_g1)
{
    if (start < 0 ||
        static_cast<size_t>(start + static_cast<int64_t>(cps * nsym)) >
            soft.size())
        return false;
    std::vector<int8_t> spread;
    if (!bprf_spreading(spread, code_index, scram_offset, cpb * nsym))
        return false;
    g0.resize(nsym);
    g1.resize(nsym);
    llr_g0.resize(nsym);
    llr_g1.resize(nsym);
    const size_t third = cps / 2;
    for (size_t s = 0; s < nsym; ++s) {
        const size_t base = static_cast<size_t>(start) + s * cps;
        const int8_t* sp = &spread[s * cpb];
        float b0 = -1e30f, b1 = -1e30f, m0 = 0.0f, m1 = 0.0f;
        for (size_t hop = 0; hop < 2; ++hop) {
            const size_t f0 = hop * cpb;
            float metric0 = 0.0f, metric1 = 0.0f;
            for (size_t c = 0; c < cpb; ++c) {
                metric0 += pol * soft[base + f0 + c] * static_cast<float>(sp[c]);
                metric1 += pol * soft[base + third + f0 + c] *
                           static_cast<float>(sp[c]);
            }
            if (std::abs(metric0) > b0) {
                b0 = std::abs(metric0);
                m0 = metric0;
            }
            if (std::abs(metric1) > b1) {
                b1 = std::abs(metric1);
                m1 = metric1;
            }
        }
        float best;
        if (b0 >= b1) {
            g0[s] = 0;
            best = m0;
        } else {
            g0[s] = 1;
            best = m1;
        }
        g1[s] = (best < 0.0f) ? 1 : 0;
        llr_g0[s] = b0 - b1; // >0 prefers g0=0
        llr_g1[s] = best;    // >0 prefers g1=0
    }
    return true;
}

// Soft-decision rate-1/2 CL-3 Viterbi (same trellis as vitdec_cl3).
// llr0/llr1: per-symbol LLRs for the two coded bits (positive favours bit 0).
inline void vitdec_cl3_soft(const float* llr0, const float* llr1, size_t N,
                            std::vector<int8_t>& out)
{
    int o0[4][2], o1[4][2], nxt[4][2];
    for (int st = 0; st < 4; ++st) {
        const int s1 = st >> 1, s0 = st & 1;
        for (int u = 0; u < 2; ++u) {
            o0[st][u] = s1;
            o1[st][u] = u ^ s0;
            nxt[st][u] = (u << 1) | s1;
        }
    }
    const float INF = 1e30f;
    float surf[4] = { 0.0f, INF, INF, INF };
    std::vector<int8_t> back(N * 4);
    for (size_t t = 0; t < N; ++t) {
        float ns[4] = { INF, INF, INF, INF };
        for (int st = 0; st < 4; ++st) {
            for (int u = 0; u < 2; ++u) {
                // cost = (2*o-1)*llr : o=0 gets -llr, o=1 gets +llr
                const float c0 =
                    (2.0f * static_cast<float>(o0[st][u]) - 1.0f) * llr0[t];
                const float c1 =
                    (2.0f * static_cast<float>(o1[st][u]) - 1.0f) * llr1[t];
                const float m = surf[st] + c0 + c1;
                const int nst = nxt[st][u];
                if (m < ns[nst]) {
                    ns[nst] = m;
                    back[t * 4 + nst] = static_cast<int8_t>((u << 2) | st);
                }
            }
        }
        for (int i = 0; i < 4; ++i)
            surf[i] = ns[i];
    }
    int best = 0;
    for (int i = 1; i < 4; ++i)
        if (surf[i] < surf[best])
            best = i;
    out.resize(N);
    int cur = best;
    for (size_t t = N; t-- > 0;) {
        const int v = back[t * 4 + cur];
        out[t] = static_cast<int8_t>(v >> 2);
        cur = v & 3;
    }
}

// IEEE 802.15.4 CRC-16 (reflected polynomial 0x8408, init 0).  FCS algorithm
// used by DW1000 / QM35 frames (matches uwbdecoder.ieee802154CRC16).
inline uint16_t crc16_802154(const uint8_t* data, size_t len)
{
    uint16_t crc = 0;
    const uint16_t poly = 0x8408;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit) {
            if (crc & 1)
                crc = static_cast<uint16_t>((crc >> 1) ^ poly);
            else
                crc = static_cast<uint16_t>(crc >> 1);
        }
    }
    return crc;
}

// ---------------------------------------------------------------------------
// IEEE 802.15.4a RS(63,55) over GF(2^6) — matches lrwpan.internal.hrpRS.
// Field primitive poly: x^6 + x^5 + 1 (0x61).  Generator roots alpha^1..alpha^8
// with alpha = 2 (the element x).  Symbols are 6-bit, packed MSB-first.
// Systematic codeword layout (low-first poly): [55 data][8 parity] symbols =
// 330 data bits + 48 parity bits = 378 bits.  Partial blocks are formed by
// LEADING zero-padding the data field to 330 bits (verified on golden vectors).
// ---------------------------------------------------------------------------

// GF(2^6) multiply (poly 0x61).
inline int rs_gf_mul(int a, int b)
{
    int p = 0;
    a &= 0x3f;
    b &= 0x3f;
    for (int i = 0; i < 6; ++i) {
        if (b & 1)
            p ^= a;
        const bool carry = (a & 0x20) != 0;
        a = (a << 1) & 0x3f;
        if (carry)
            a ^= 0x21; // lower 6 bits of 0x61 (x^6 = x^5 + 1)
        b >>= 1;
    }
    return p & 0x3f;
}

// GF(2^6) power a^n (n may be negative via modular reduction mod 63).
inline int rs_gf_pow(int a, int n)
{
    int r = 1;
    a &= 0x3f;
    n %= 63;
    if (n < 0)
        n += 63;
    while (n) {
        if (n & 1)
            r = rs_gf_mul(r, a);
        a = rs_gf_mul(a, a);
        n >>= 1;
    }
    return r;
}

inline int rs_gf_inv(int a)
{
    return a ? rs_gf_pow(a, 62) : 0;
}

// Pack 378 MSB-first bits -> 63 GF(2^6) symbols.
inline void rs_bits_to_syms(const int8_t* bits378, int* syms63)
{
    for (int i = 0; i < 63; ++i) {
        int s = 0;
        for (int b = 0; b < 6; ++b)
            s = (s << 1) | (bits378[i * 6 + b] & 1);
        syms63[i] = s;
    }
}

// Unpack nsym symbols -> 6*nsym MSB-first bits.
inline void rs_syms_to_bits(const int* syms, int nsym, int8_t* bits)
{
    for (int i = 0; i < nsym; ++i) {
        for (int b = 0; b < 6; ++b)
            bits[i * 6 + b] =
                static_cast<int8_t>((syms[i] >> (5 - b)) & 1);
    }
}

// Syndromes S_i = r(alpha^i), i=1..8.  Low-first: r(x) = sum_j r[j] x^j.
inline void rs_syndromes(const int* r63, int* S8)
{
    for (int i = 1; i <= 8; ++i) {
        const int ai = rs_gf_pow(2, i);
        int s = 0;
        int ap = 1;
        for (int j = 0; j < 63; ++j) {
            s ^= rs_gf_mul(r63[j], ap);
            ap = rs_gf_mul(ap, ai);
        }
        S8[i - 1] = s;
    }
}

// Berlekamp-Massey on S1..S8.  Returns L; Lambda[0..L] with Lambda[0]=1.
inline int rs_berlekamp_massey(const int* S8, int* Lambda /* len >= 9 */)
{
    int C[9] = { 1, 0, 0, 0, 0, 0, 0, 0, 0 };
    int B[9] = { 1, 0, 0, 0, 0, 0, 0, 0, 0 };
    int L = 0;
    int m = 1;
    int b = 1;
    for (int n = 0; n < 8; ++n) {
        int delta = S8[n];
        for (int i = 1; i <= L; ++i)
            delta ^= rs_gf_mul(C[i], S8[n - i]);
        if (delta == 0) {
            ++m;
        } else {
            int T[9];
            for (int i = 0; i < 9; ++i)
                T[i] = C[i];
            const int scale = rs_gf_mul(delta, rs_gf_inv(b));
            for (int i = 0; i < 9 - m; ++i)
                C[i + m] ^= rs_gf_mul(scale, B[i]);
            if (2 * L <= n) {
                L = n + 1 - L;
                for (int i = 0; i < 9; ++i)
                    B[i] = T[i];
                b = delta;
                m = 1;
            } else {
                ++m;
            }
        }
    }
    for (int i = 0; i < 9; ++i)
        Lambda[i] = C[i];
    return L;
}

// Chien search: roots of Lambda at alpha^{-j} => error at position j.
// Returns number of roots (may exceed L if uncorrectable).
inline int rs_chien_search(const int* Lambda, int L, int* err_pos /* <=4 */)
{
    int nerr = 0;
    for (int j = 0; j < 63; ++j) {
        const int xinv = rs_gf_pow(2, (63 - j) % 63); // alpha^{-j}
        int v = 0;
        int xp = 1;
        for (int i = 0; i <= L; ++i) {
            v ^= rs_gf_mul(Lambda[i], xp);
            xp = rs_gf_mul(xp, xinv);
        }
        if (v == 0) {
            if (nerr < 4)
                err_pos[nerr] = j;
            ++nerr;
        }
    }
    return nerr;
}

// Forney (b=1, roots alpha^1..): e_j = Omega(X^{-1}) / Lambda'(X^{-1}).
inline void rs_forney(const int* S8, const int* Lambda, int L,
                      const int* err_pos, int nerr, int* err_val)
{
    int Omega[8] = {};
    for (int i = 0; i < 8; ++i) {
        int o = 0;
        for (int j = 0; j <= L && j <= i; ++j)
            o ^= rs_gf_mul(Lambda[j], S8[i - j]);
        Omega[i] = o;
    }
    for (int k = 0; k < nerr; ++k) {
        const int j = err_pos[k];
        const int Xinv = rs_gf_pow(2, (63 - j) % 63);
        int num = 0;
        int xp = 1;
        for (int i = 0; i < 8; ++i) {
            num ^= rs_gf_mul(Omega[i], xp);
            xp = rs_gf_mul(xp, Xinv);
        }
        // Lambda' in char 2: only odd powers, Lambda_i * x^{i-1}
        int den = 0;
        xp = 1; // Xinv^0 for i=1
        for (int i = 1; i <= L; i += 2) {
            den ^= rs_gf_mul(Lambda[i], xp);
            xp = rs_gf_mul(xp, rs_gf_mul(Xinv, Xinv));
        }
        err_val[k] = den ? rs_gf_mul(num, rs_gf_inv(den)) : 0;
    }
}

// Decode one RS(63,55) block: 378 coded bits -> 330 data bits (MSB-first).
// Returns false if the block is uncorrectable (more than t=4 symbol errors).
inline bool rs_decode_block(const int8_t* coded378, int8_t* data330)
{
    int r[63];
    rs_bits_to_syms(coded378, r);

    int S[8];
    rs_syndromes(r, S);
    bool all0 = true;
    for (int i = 0; i < 8; ++i)
        if (S[i])
            all0 = false;

    if (!all0) {
        int Lambda[9] = {};
        const int L = rs_berlekamp_massey(S, Lambda);
        if (L <= 0 || L > 4)
            return false;
        int err_pos[4] = {};
        const int nerr = rs_chien_search(Lambda, L, err_pos);
        if (nerr != L)
            return false;
        int err_val[4] = {};
        rs_forney(S, Lambda, L, err_pos, nerr, err_val);
        for (int k = 0; k < nerr; ++k)
            r[err_pos[k]] ^= err_val[k];
        // Verify correction
        rs_syndromes(r, S);
        for (int i = 0; i < 8; ++i)
            if (S[i])
                return false;
    }

    rs_syms_to_bits(r, 55, data330);
    return true;
}

// Full-stream RS decoder for a PSDU of `data_bits` bits (e.g. 8*L).
// Splits the coded bit stream into num_blocks = ceil(data_bits/330) blocks,
// each carrying min(330, remaining) data bits + 48 parity bits.  Partial
// blocks are leading-zero-padded to 330 data bits before decode.
inline bool rs_decode_stream(const std::vector<int8_t>& coded,
                             size_t data_bits,
                             std::vector<int8_t>& data_out)
{
    data_out.clear();
    data_out.reserve(data_bits);
    if (data_bits == 0)
        return true;

    const size_t num_blocks = (data_bits + 329) / 330;
    size_t coded_off = 0;
    size_t data_off = 0;
    for (size_t b = 0; b < num_blocks; ++b) {
        const size_t this_data =
            std::min<size_t>(330, data_bits - data_off);
        const size_t this_coded = this_data + 48;
        if (coded_off + this_coded > coded.size())
            return false;

        int8_t block378[378];
        for (int i = 0; i < 378; ++i)
            block378[i] = 0;
        // Leading-zero pad: real data occupies the last this_data bits of
        // the 330-bit systematic data field (MATLAB hrpRS convention).
        const size_t data_start = 330 - this_data;
        for (size_t i = 0; i < this_data; ++i)
            block378[data_start + i] =
                static_cast<int8_t>(coded[coded_off + i] & 1);
        for (size_t i = 0; i < 48; ++i)
            block378[330 + i] = static_cast<int8_t>(
                coded[coded_off + this_data + i] & 1);

        int8_t data330[330];
        if (!rs_decode_block(block378, data330))
            return false;

        for (size_t i = 0; i < this_data; ++i)
            data_out.push_back(data330[data_start + i]);

        coded_off += this_coded;
        data_off += this_data;
    }
    return data_out.size() == data_bits;
}

// Shared hard-decision payload backend: concatenate the 21 PHR symbols and the
// payload symbols, run the truncated CL-3 Viterbi, remove the 19 systematic PHR
// bits and the two tail bits, RS-decode, pack LSB-first and compute the FCS.
// This is the exact first attempt of stage_payload_fcs(); the fused payload
// path calls it with g0/g1 produced by rxfusion, so both paths share the same
// FEC/FCS implementation and length conventions.
inline bool decode_payload_hard(const std::vector<int8_t>& g0p,
                                const std::vector<int8_t>& g1p,
                                const std::vector<int8_t>& g0,
                                const std::vector<int8_t>& g1,
                                size_t psdu_bits,
                                PayloadResult& out,
                                PayloadSubTimers* sub = nullptr)
{
    PayloadSubTimer sub_guard(sub);
    out = PayloadResult{};
    if (g0p.size() != 21 || g1p.size() != 21 || g0.empty() ||
        g0.size() != g1.size())
        return false;

    const size_t total = 21 + g0.size();
    std::vector<int8_t> rx_bits(2 * total);
    for (size_t s = 0; s < 21; ++s) {
        rx_bits[2 * s] = g0p[s];
        rx_bits[2 * s + 1] = g1p[s];
    }
    for (size_t s = 0; s < g0.size(); ++s) {
        rx_bits[2 * (s + 21)] = g0[s];
        rx_bits[2 * (s + 21) + 1] = g1[s];
    }
    std::vector<int8_t> decoded;
    const auto t_vit = std::chrono::steady_clock::now();
    vitdec_cl3(rx_bits.data(), total, decoded);
    if (sub)
        sub->viterbi_us += payload_elapsed_us(t_vit);
    if (decoded.size() < total)
        return false;

    std::vector<int8_t> rs_cw(decoded.begin() + 19,
                              decoded.begin() + (total - 2));
    if (rs_cw.size() != g0.size())
        return false;
    std::vector<int8_t> psdu_bits_out;
    const auto t_rs = std::chrono::steady_clock::now();
    const bool rs_ok = rs_decode_stream(rs_cw, psdu_bits, psdu_bits_out);
    if (sub)
        sub->rs_us += payload_elapsed_us(t_rs);
    if (!rs_ok)
        return false;

    // PSDU assembly + FCS: bit packing, CRC-16 and the comparison.
    const auto t_crc = std::chrono::steady_clock::now();
    out.bits.assign(psdu_bits_out.begin(), psdu_bits_out.end());
    out.bytes.resize(psdu_bits_out.size() / 8);
    for (size_t i = 0; i < out.bytes.size(); ++i) {
        uint8_t b = 0;
        for (int bit = 0; bit < 8; ++bit)
            b |= static_cast<uint8_t>(psdu_bits_out[8 * i + bit]) << bit;
        out.bytes[i] = b;
    }
    if (out.bytes.size() >= 2) {
        out.calculated_fcs =
            crc16_802154(out.bytes.data(), out.bytes.size() - 2);
        out.received_fcs = static_cast<uint16_t>(
                               out.bytes[out.bytes.size() - 2]) |
                           (static_cast<uint16_t>(
                                out.bytes[out.bytes.size() - 1])
                            << 8);
        out.fcs_pass = (out.received_fcs == out.calculated_fcs);
    }
    if (sub)
        sub->crc_us += payload_elapsed_us(t_crc);
    out.ok = true;
    return true;
}

} // namespace detail

// ---------------------------------------------------------------------------
// Stage 5 — NS-SFD location in the soft-chip stream (R3).
// Matches MATLAB locateNsSfd: spread = kron(sfd_sequence, spread_code), search
// a +/-8-chip window around expected = preamble_repetitions*chips_per_symbol
// for the best normalized correlation.  Golden: start_chip=32512,
// end_chip=36575, polarity=1, correlation=1.0 (chip coordinates, 0-based).
// ---------------------------------------------------------------------------
inline bool stage_ns_sfd(const std::vector<float>& soft_chips,
                         const Qm35825Profile& profile,
                         const std::vector<int8_t>& sfd_sequence,
                         size_t chips_per_symbol,
                         NsSfdResult& out,
                         DemodScratch& scratch)
{
    out = NsSfdResult{};
    if (soft_chips.empty() || sfd_sequence.empty() || chips_per_symbol == 0)
        return false;

    // spread = kron(sfd_sequence, spread_code); both real ternary -> real.
    const char* cur_mode = profile.sfd_mode ? profile.sfd_mode : "";
    if (scratch.cached_seq.code_index != profile.code_index ||
        scratch.cached_seq.sfd_mode != cur_mode ||
        scratch.cached_seq.sampled_pcode.empty()) {
        scratch.cached_seq.update(profile.code_index, profile.sfd_mode);
    }
    if (scratch.cached_seq.sfd_seq != sfd_sequence ||
        scratch.cached_seq.ns_sfd_sparse.empty()) {
        scratch.cached_seq.sfd_seq = sfd_sequence;
        scratch.cached_seq.ns_sfd_sparse.clear();
        const std::vector<int8_t>& sc = scratch.cached_seq.sampled_pcode;
        scratch.cached_seq.ns_sfd_total_chips = sfd_sequence.size() * sc.size();
        for (size_t i = 0; i < sfd_sequence.size(); ++i) {
            const int8_t si = sfd_sequence[i];
            if (si == 0) continue;
            for (size_t j = 0; j < sc.size(); ++j) {
                const int8_t pj = sc[j];
                if (pj == 0) continue;
                const int8_t val = static_cast<int8_t>(si * pj);
                scratch.cached_seq.ns_sfd_sparse.push_back({ static_cast<uint32_t>(i * sc.size() + j), val });
            }
        }
        scratch.cached_seq.ns_sfd_norm = std::sqrt(static_cast<float>(scratch.cached_seq.ns_sfd_sparse.size()));
    }

    const size_t sfd_len = sfd_sequence.size() * chips_per_symbol;
    const int64_t expected =
        static_cast<int64_t>(profile.preamble_repetitions) *
        static_cast<int64_t>(chips_per_symbol);
    const int64_t half = 8;
    const int64_t lo = std::max<int64_t>(0, expected - half);
    const int64_t hi = std::min<int64_t>(
        static_cast<int64_t>(soft_chips.size()) - static_cast<int64_t>(sfd_len),
        expected + half);
    if (hi < lo)
        return false;

    float best = -1.0f;
    int64_t best_k = lo;
    int pol = 1;

    if (chips_per_symbol == scratch.cached_seq.sampled_pcode.size() &&
        scratch.cached_seq.ns_sfd_total_chips == sfd_len &&
        !scratch.cached_seq.ns_sfd_sparse.empty()) {
        const float spread_norm = scratch.cached_seq.ns_sfd_norm;
        double win_energy = 0.0;
        for (size_t i = 0; i < sfd_len; ++i) {
            const double v = static_cast<double>(
                soft_chips[static_cast<size_t>(lo + static_cast<int64_t>(i))]);
            win_energy += v * v;
        }
        if (!std::isfinite(win_energy) || win_energy < 0.0)
            return false;

        for (int64_t k = lo; k <= hi; ++k) {
            if (k > lo) {
                const double v_in = static_cast<double>(
                    soft_chips[static_cast<size_t>(k + static_cast<int64_t>(sfd_len) - 1)]);
                const double v_out = static_cast<double>(
                    soft_chips[static_cast<size_t>(k - 1)]);
                win_energy += v_in * v_in - v_out * v_out;
                if (!std::isfinite(win_energy))
                    return false;
            }
            const float seg_norm = std::sqrt(std::max(0.0, win_energy));
            float corr = 0.0f;
            for (const auto& entry : scratch.cached_seq.ns_sfd_sparse) {
                const float v = soft_chips[static_cast<size_t>(k + static_cast<int64_t>(entry.offset))];
                corr += (entry.sign > 0) ? v : -v;
            }
            const float m = std::abs(corr) / (spread_norm * seg_norm + 1e-12f);
            if (m > best) {
                best = m;
                best_k = k;
                pol = (corr >= 0.0f) ? 1 : -1;
            }
        }
    } else {
        const std::vector<int8_t>& spread_code = scratch.cached_seq.sampled_pcode;
        scratch.work.assign(sfd_len, std::complex<float>(0.0f, 0.0f));
        for (size_t i = 0; i < sfd_sequence.size(); ++i)
            for (size_t j = 0; j < spread_code.size(); ++j)
                scratch.work[i * spread_code.size() + j] = std::complex<float>(
                    static_cast<float>(sfd_sequence[i] * spread_code[j]), 0.0f);
        float spread_norm = 0.0f;
        for (size_t i = 0; i < sfd_len; ++i)
            spread_norm += std::norm(scratch.work[i]);
        spread_norm = std::sqrt(spread_norm);

        for (int64_t k = lo; k <= hi; ++k) {
            float seg_norm = 0.0f, corr = 0.0f;
            for (size_t i = 0; i < sfd_len; ++i) {
                const float v = soft_chips[static_cast<size_t>(k + (int64_t)i)];
                seg_norm += v * v;
                corr += scratch.work[i].real() * v;
            }
            seg_norm = std::sqrt(seg_norm);
            const float m = std::abs(corr) / (spread_norm * seg_norm + 1e-12f);
            if (m > best) {
                best = m;
                best_k = k;
                pol = (corr >= 0.0f) ? 1 : -1;
            }
        }
    }

    if (best < profile.sfd_detection_threshold)
        return false;

    out.ok = true;
    out.sfd_start_chip = best_k;
    out.sfd_end_chip = best_k + static_cast<int64_t>(sfd_len) - 1;
    out.polarity = pol;
    out.metric = best;
    return true;
}

// ---------------------------------------------------------------------------
// Stage 6 — BPRF PHR demod + convolutional decode + SECDED (R3).
// Matches helperUWBBPRFDemod + helperUWBConvDec + helperUWBPHRDecode:
//   * 21 BPM-BPSK symbols (0.85 Mbps PHR: 64 chips/burst, 512 chips/symbol),
//     de-spread with the scrambler LFSR (offset 0).
//   * rate-1/2 CL-3 Viterbi (trunc), keep the 19 PHR bits.
//   * SECDED single-bit correction / double-bit detection -> PSDU length.
// Golden: coded/decoded PHR match MATLAB, psdu_length=127, secded_pass=1.
// ---------------------------------------------------------------------------
// Shared PHR Viterbi/SECDED parser.  The legacy stage_phr() and the fused PHR
// kernel both feed their 21 raw g0/g1 pairs here, so the FEC convention and the
// SECDED tail-energy guard are identical.  `soft_chips` is only read by the
// guard when a corrected length grows; callers with a short prefix therefore
// keep the same conservative "return the corrected result and let the caller
// retry the full stream once" behaviour as the existing on-demand path.
inline bool decode_phr_decisions(const std::vector<int8_t>& g0,
                                 const std::vector<int8_t>& g1,
                                 const std::vector<float>& soft_chips,
                                 const NsSfdResult& ns_sfd,
                                 PhrResult& out)
{
    out = PhrResult{};
    const size_t nsym = 21;
    if (g0.size() != nsym || g1.size() != nsym)
        return false;

    std::vector<int8_t> rx(2 * nsym);
    for (size_t s = 0; s < nsym; ++s) {
        rx[2 * s] = g0[s];
        rx[2 * s + 1] = g1[s];
    }
    std::vector<int8_t> decoded;
    detail::vitdec_cl3(rx.data(), nsym, decoded);
    if (decoded.size() < 19)
        return false;

    std::vector<int8_t> sys13(decoded.begin(), decoded.begin() + 13);
    std::vector<int8_t> recv_par(decoded.begin() + 13, decoded.begin() + 19);
    std::vector<int8_t> phr19;
    detail::hrp_secded(sys13, phr19);
    int8_t synd[6];
    for (int i = 0; i < 6; ++i)
        synd[i] = static_cast<int8_t>(recv_par[i] ^ phr19[13 + i]);

    bool secded_pass = true;
    int idx = 0;
    for (int i = 1; i < 6; ++i)
        idx |= (synd[i] << (i - 1));
    if (idx > 0) {
        static const int kAddresses[13] = { 3, 5, 6, 7, 9, 10, 11,
                                            12, 13, 14, 15, 17, 18 };
        for (int j = 0; j < 13; ++j) {
            if (kAddresses[j] == idx) {
                phr19[j] ^= 1;
                out.secded_corrected = true;
                break;
            }
        }
        if (!synd[0])
            secded_pass = false; // DED: second error cannot be corrected
    }
    out.secded_uncorrectable = !secded_pass;

    // MATLAB bit2int default is MSB-first (bit2int(x,n)); phr(1:2) -> data
    // rate, phr(3:9) -> PSDU length.
    //
    // Guard against SECDED mis-correction on real RF.  A false single-bit
    // flip can inflate PSDU length (e.g. 13 -> 45 on DW1000 captures) past
    // the RF energy while the soft-chip *buffer* is still long enough.  If
    // the corrected length needs a long tail whose soft-chip energy is near
    // the noise floor relative to the uncorrected body, keep uncorrected.
    auto length_of = [](const std::vector<int8_t>& b) -> uint32_t {
        uint32_t psdu = 0;
        for (int i = 0; i < 7; ++i)
            psdu |= (static_cast<uint32_t>(b[2 + i]) << (6 - i));
        return psdu;
    };
    auto chips_needed = [](uint32_t psdu_len) -> size_t {
        const size_t psdu_bits = static_cast<size_t>(psdu_len) * 8;
        const size_t num_blocks = (psdu_bits + 329) / 330;
        const size_t nsym = psdu_bits + 48 * num_blocks;
        // PHR 21*512 + payload nsym*64 (6.81 Mb/s) chips after SFD end.
        return 21 * 512 + nsym * 64;
    };
    const uint32_t len_uncorr = length_of(sys13); // sys13 is never flipped
    const uint32_t len_corr = length_of(phr19);
    if (out.secded_corrected && len_corr > len_uncorr &&
        ns_sfd.sfd_end_chip >= 0) {
        const int64_t phr_start = ns_sfd.sfd_end_chip + 1;
        const size_t need_c = chips_needed(len_corr);
        const size_t need_u = chips_needed(len_uncorr);
        if (phr_start >= 0 && need_c > need_u + 64) {
            const size_t base = static_cast<size_t>(phr_start);
            const size_t end_u = base + need_u;
            const size_t end_c = base + need_c;
            if (end_c <= soft_chips.size() && end_u < soft_chips.size()) {
                double e_body = 0.0, e_tail = 0.0;
                for (size_t i = base; i < end_u; ++i) {
                    const double v = soft_chips[i];
                    e_body += v * v;
                }
                for (size_t i = end_u; i < end_c; ++i) {
                    const double v = soft_chips[i];
                    e_tail += v * v;
                }
                const double n_body = static_cast<double>(end_u - base);
                const double n_tail = static_cast<double>(end_c - end_u);
                const double mean_body = e_body / (n_body + 1e-12);
                const double mean_tail = e_tail / (n_tail + 1e-12);
                // Tail is noise-like if its mean energy is << body.
                if (mean_tail < 0.15 * mean_body) {
                    for (int i = 0; i < 13; ++i)
                        phr19[i] = sys13[i];
                    out.secded_corrected = false;
                }
            }
        }
    }

    const int dr = (phr19[0] << 1) | phr19[1];
    static const float kDataRates[4] = { 0.11f, 0.85f, 6.81f, 27.24f };
    out.data_rate_mbps = kDataRates[dr & 3];
    uint32_t psdu = 0;
    for (int i = 0; i < 7; ++i)
        psdu |= (static_cast<uint32_t>(phr19[2 + i]) << (6 - i)); // MSB-first
    out.psdu_length = psdu;
    out.phr_bits.reserve(19);
    for (int i = 0; i < 19; ++i)
        out.phr_bits.push_back(static_cast<uint8_t>(phr19[i]));
    out.ok = true;
    return true;
}

inline bool stage_phr(const std::vector<float>& soft_chips,
                      const Qm35825Profile& profile,
                      const NsSfdResult& ns_sfd,
                      size_t chips_per_symbol,
                      PhrResult& out,
                      DemodScratch& scratch)
{
    out = PhrResult{};
    if (soft_chips.empty() || !ns_sfd.ok || ns_sfd.sfd_end_chip < 0)
        return false;
    const size_t cpb = 64, cps = 512, nsym = 21;
    const int64_t phr_start = ns_sfd.sfd_end_chip + 1;
    if (phr_start < 0 ||
        static_cast<size_t>(phr_start + static_cast<int64_t>(cps * nsym)) >
            soft_chips.size())
        return false;
    const int8_t pol = (ns_sfd.polarity < 0) ? -1 : 1;

    std::vector<int8_t> g0, g1;
    if (!detail::bprf_demod(soft_chips, phr_start, profile.code_index, nsym,
                            cpb, cps, 0, pol, g0, g1))
        return false;
    return decode_phr_decisions(g0, g1, soft_chips, ns_sfd, out);
}

// Build (or reuse) the PHR spreading stream.  The PN depends only on the code
// index and the fixed 21-symbol geometry; the channel template is rebuilt per
// packet by stage_phr_fused_cmf().
inline bool prepare_fused_phr_pn(DemodScratch& scratch, size_t code_index)
{
    const size_t nbits =
        rxfusion::kPhrSymbols * rxfusion::kPhrChipsPerBurst;
    if (scratch.fused_phr.code_index == code_index &&
        scratch.fused_phr.pn.size() == nbits)
        return true;
    if (!detail::bprf_spreading(scratch.fused_phr.pn, code_index,
                                rxfusion::kPhrScramOffset, nbits)) {
        scratch.fused_phr.code_index = 0;
        scratch.fused_phr.pn.clear();
        scratch.fused_phr.reset_decisions();
        return false;
    }
    scratch.fused_phr.code_index = code_index;
    scratch.fused_phr.reset_decisions();
    return true;
}

// Experimental fused PHR metrics + the shared PHR FEC/SECDED parser.  The PHR
// field is generated directly from CFO-compensated IQ with 164-point templates
// (38 taps), so no PHR soft-chip FIR is required.  The 21 raw g0/g1 decisions
// are cached for the payload joint Viterbi.  Returns false on unsupported
// geometry, range violations, non-finite metrics or decode failure; the caller
// retries the complete legacy stream once.  Timings are always reported when
// the corresponding work was performed.
inline bool stage_phr_fused_cmf(const std::complex<float>* rx,
                                size_t n,
                                const Qm35825Profile& profile,
                                const TimingResult& timing,
                                const CirResult& cir,
                                const std::vector<float>& prefix_soft_chips,
                                const NsSfdResult& ns_sfd,
                                PhrResult& out,
                                DemodScratch& scratch,
                                uint64_t& template_us,
                                uint64_t& correlate_us,
                                uint64_t& decode_us,
                                uint64_t& decode_ns,
                                size_t& symbols_out)
{
    out = PhrResult{};
    template_us = 0;
    correlate_us = 0;
    decode_us = 0;
    decode_ns = 0;
    symbols_out = 0;
    if (!rx || !ns_sfd.ok || ns_sfd.sfd_end_chip < 0)
        return false;
    const int64_t phr_start = ns_sfd.sfd_end_chip + 1;
    if (phr_start < 0 ||
        static_cast<size_t>(phr_start) > prefix_soft_chips.size())
        return false;

    const size_t tap_count = cir.cir_filter_taps.size();
    if (tap_count == 0 || tap_count > rxfusion::kMaxTapCount)
        return false;
    if (tap_count != profile.cir_pre_samples + profile.cir_post_samples)
        return false;
    if (!integer_chip_grid_at(cir.samples_per_chip,
                              rxfusion::kSamplesPerChip,
                              profile.fused_gate_ppm * 1e-6))
        return false;
    if (profile.cir_pre_samples > tap_count)
        return false;
    if (!scratch.softchip.prepared || !std::isfinite(scratch.softchip.rot.real()) ||
        !std::isfinite(scratch.softchip.rot.imag()))
        return false;

    const int64_t chip_start =
        timing.preamble_start_sample + static_cast<int64_t>(tap_count) -
        static_cast<int64_t>(profile.cir_pre_samples) - 1;
    if (chip_start < 0 || static_cast<int64_t>(n) <= chip_start)
        return false;

    const auto t_plan0 = std::chrono::steady_clock::now();
    if (!prepare_fused_phr_pn(scratch, profile.code_index))
        return false;
    const auto t_plan1 = std::chrono::steady_clock::now();
    const uint64_t plan_us = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(t_plan1 - t_plan0)
            .count());

    const int8_t pol = (ns_sfd.polarity < 0) ? -1 : 1;
    rxfusion::Timings timings;
    if (!rxfusion::correlate_phr(
            scratch.derotated.data(), n, cir.cir_filter_taps.data(), tap_count,
            chip_start, rxfusion::kSamplesPerChip, phr_start,
            scratch.fused_phr.pn.data(), pol, scratch.softchip.rot,
            scratch.fused_phr.metrics, scratch.fused_phr.kernel, &timings))
        return false;
    template_us = plan_us + timings.template_update_us;
    correlate_us = timings.correlate_us;
    for (float v : scratch.fused_phr.metrics.signed_value) {
        if (!std::isfinite(v))
            return false;
    }

    const auto t_decode0 = std::chrono::steady_clock::now();
    rxfusion::decide_bpm(scratch.fused_phr.metrics, scratch.fused_phr.g0,
                         scratch.fused_phr.g1);
    scratch.fused_phr.decisions_valid = true;
    const bool decoded = decode_phr_decisions(
        scratch.fused_phr.g0, scratch.fused_phr.g1, prefix_soft_chips,
        ns_sfd, out);
    decode_ns = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now() - t_decode0)
            .count());
    decode_us = decode_ns / 1000;
    symbols_out = rxfusion::kPhrSymbols;
    if (!decoded) {
        scratch.fused_phr.reset_decisions();
        return false;
    }
    return true;
}

// ---------------------------------------------------------------------------
// Stage 7 — payload BPM-BPSK + RS + FCS (R4).
// Mirrors helperUWBPayloadDecode + lrwpan.internal.hrpRS + ieee802154CRC16:
//   * payload field at 6.81 Mbps (8 chips/burst, 64 chips/symbol), scrambler
//     LFSR offset = 21 PHR symbols * 64 = 1344.
//   * joint rate-1/2 CL-3 Viterbi decode of [PHR | payload] codewords, then
//     slice out the RS-coded stream (drop the first 19 PHR bits + 2 tails).
//   * RS(63,55) decode (detail::rs_decode_stream) -> PSDU bits.
//   * pack bits to bytes LSB-first, compute IEEE 802.15.4 CRC-16 FCS.
// Golden: 127 payload bytes, fcs_pass=1, received==calculated==0x584b.
// ---------------------------------------------------------------------------
inline bool stage_payload_fcs(const std::vector<float>& soft_chips,
                              const Qm35825Profile& profile,
                              const PhrResult& phr,
                              const NsSfdResult& ns_sfd,
                              size_t chips_per_symbol,
                              PayloadResult& out,
                              DemodScratch& scratch,
                              PayloadSubTimers* sub = nullptr)
{
    // Diagnostic child timing of `stage_payload_us`.  `sub_guard` attributes the
    // remainder of this function to `other_us`; the named buckets are filled
    // around the exact calls that produce them.  Passing `sub == nullptr`
    // (e.g. the full-stream fallback retry, which is accounted in its own
    // bucket) disables all of it.
    detail::PayloadSubTimer sub_guard(sub);
    if (sub)
        ++sub->legacy_calls;
    out = PayloadResult{};
    if (soft_chips.empty() || !phr.ok || !ns_sfd.ok || phr.psdu_length == 0)
        return false;
    const int64_t phr_start = ns_sfd.sfd_end_chip + 1;
    if (phr_start < 0)
        return false;
    // The payload field begins right after the 21 PHR symbols (512 chips each).
    const int64_t payload_start = phr_start + static_cast<int64_t>(512 * 21);

    bool any_ok = false;

    // First run the MATLAB-equivalent production path: reported PHR length,
    // NS-SFD polarity, hard BPM decisions and hard truncated Viterbi.  The
    // RF experiments below (length neighborhood, opposite polarity and soft
    // Viterbi) are fallbacks; they must not hide a valid fixed-length decode
    // or replace it with a shorter accidental RS codeword.
    {
        const int8_t pol = (ns_sfd.polarity < 0) ? -1 : 1;
        const size_t psdu_bits = static_cast<size_t>(phr.psdu_length) * 8;
        const size_t num_blocks = (psdu_bits + 329) / 330;
        const size_t nsym = psdu_bits + 48 * num_blocks;
        std::vector<int8_t> g0p, g1p, g0, g1;
        const auto t_soft = std::chrono::steady_clock::now();
        const bool demod_ok =
            detail::bprf_demod(soft_chips, phr_start, profile.code_index, 21,
                               64, 512, 0, pol, g0p, g1p) &&
            detail::bprf_demod(soft_chips, payload_start, profile.code_index,
                               nsym, 8, 64, 1344, pol, g0, g1);
        if (sub)
            sub->soft_us += detail::payload_elapsed_us(t_soft);
        if (demod_ok) {
            if (detail::decode_payload_hard(g0p, g1p, g0, g1, psdu_bits, out,
                                            sub)) {
                any_ok = true;
                if (out.fcs_pass)
                    return true;
            }
        }
    }

    // Candidate PSDU lengths: reported PHR length, then a small neighborhood
    // (±2) that covers common SECDED single-bit length slips on real RF.
    uint32_t lens[5];
    size_t n_lens = 0;
    auto add_len = [&](uint32_t L) {
        if (L == 0 || L > profile.max_psdu_bytes)
            return;
        for (size_t i = 0; i < n_lens; ++i)
            if (lens[i] == L)
                return;
        if (n_lens < 5)
            lens[n_lens++] = L;
    };
    add_len(phr.psdu_length);
    if (phr.psdu_length > 2)
        add_len(phr.psdu_length - 1);
    add_len(phr.psdu_length + 1);
    if (phr.psdu_length > 3)
        add_len(phr.psdu_length - 2);
    add_len(phr.psdu_length + 2);

    // Try reported SFD polarity first, then the opposite.  On real RF the
    // NS-SFD sign can flip relative to the payload field even when the
    // magnitude metric is high; one polarity yields RS+FCS, the other fails.
    const int8_t pol0 = (ns_sfd.polarity < 0) ? -1 : 1;
    const int8_t pols[2] = { pol0, static_cast<int8_t>(-pol0) };
    for (size_t li = 0; li < n_lens; ++li) {
        const uint32_t L = lens[li];
        const size_t psdu_bits = static_cast<size_t>(L) * 8;
        const size_t num_blocks = (psdu_bits + 329) / 330;
        const size_t nsym = psdu_bits + 48 * num_blocks;
        for (int8_t pol : pols) {
            std::vector<int8_t> g0p, g1p, g0, g1;
            std::vector<float> llr0p, llr1p, llr0, llr1;
            const auto t_soft = std::chrono::steady_clock::now();
            const bool soft_ok =
                detail::bprf_demod_soft(soft_chips, phr_start,
                                        profile.code_index, 21, 64, 512, 0,
                                        pol, g0p, g1p, llr0p, llr1p) &&
                detail::bprf_demod_soft(soft_chips, payload_start,
                                        profile.code_index, nsym, 8, 64,
                                        1344, pol, g0, g1, llr0, llr1);
            if (sub)
                sub->soft_us += detail::payload_elapsed_us(t_soft);
            if (!soft_ok)
                continue;

            const size_t total = 21 + nsym;
            std::vector<float> llr0j(total), llr1j(total);
            for (size_t s = 0; s < 21; ++s) {
                llr0j[s] = llr0p[s];
                llr1j[s] = llr1p[s];
            }
            for (size_t s = 0; s < nsym; ++s) {
                llr0j[s + 21] = llr0[s];
                llr1j[s + 21] = llr1[s];
            }
            std::vector<int8_t> decoded;
            const auto t_vit = std::chrono::steady_clock::now();
            detail::vitdec_cl3_soft(llr0j.data(), llr1j.data(), total, decoded);
            if (sub)
                sub->viterbi_us += detail::payload_elapsed_us(t_vit);
            if (decoded.size() < total)
                continue;

            std::vector<int8_t> rs_cw(decoded.begin() + 19,
                                      decoded.begin() + (total - 2));
            if (rs_cw.size() != nsym)
                continue;

            std::vector<int8_t> psdu_bits_out;
            const auto t_rs = std::chrono::steady_clock::now();
            const bool rs_ok =
                detail::rs_decode_stream(rs_cw, psdu_bits, psdu_bits_out);
            if (sub)
                sub->rs_us += detail::payload_elapsed_us(t_rs);
            if (!rs_ok)
                continue;

            // PSDU assembly + FCS: bit packing, CRC-16 and the comparison.
            const auto t_crc = std::chrono::steady_clock::now();
            const size_t nbytes = psdu_bits_out.size() / 8;
            PayloadResult cand;
            cand.bytes.resize(nbytes);
            for (size_t i = 0; i < nbytes; ++i) {
                uint8_t b = 0;
                for (int bit = 0; bit < 8; ++bit)
                    b |= static_cast<uint8_t>(psdu_bits_out[8 * i + bit])
                         << bit;
                cand.bytes[i] = b;
            }
            cand.bits.assign(psdu_bits_out.begin(), psdu_bits_out.end());
            if (nbytes >= 2) {
                cand.calculated_fcs =
                    detail::crc16_802154(cand.bytes.data(), nbytes - 2);
                cand.received_fcs =
                    static_cast<uint16_t>(cand.bytes[nbytes - 2]) |
                    (static_cast<uint16_t>(cand.bytes[nbytes - 1]) << 8);
                cand.fcs_pass =
                    (cand.received_fcs == cand.calculated_fcs);
            }
            cand.ok = true;
            if (sub)
                sub->crc_us += detail::payload_elapsed_us(t_crc);
            if (!any_ok || cand.fcs_pass) {
                out = cand;
                any_ok = true;
            }
            if (out.fcs_pass)
                return true;
        }
    }
    return any_ok;
}

// Build (or reuse) the BPRF payload spreading stream for the fused kernel.
// The PN sequence depends only on the code index and the decoded symbol count,
// so it is cached across packets; the channel template is still rebuilt for
// every packet and counted separately.
inline bool prepare_fused_payload_pn(DemodScratch& scratch,
                                     size_t code_index,
                                     size_t symbols)
{
    const size_t nbits = symbols * rxfusion::kChipsPerBurst;
    if (scratch.fused.code_index == code_index &&
        scratch.fused.symbols == symbols &&
        scratch.fused.pn.size() == nbits)
        return true;
    if (!detail::bprf_spreading(scratch.fused.pn, code_index, 1344, nbits)) {
        scratch.fused.code_index = 0;
        scratch.fused.symbols = 0;
        scratch.fused.pn.clear();
        return false;
    }
    scratch.fused.code_index = code_index;
    scratch.fused.symbols = symbols;
    return true;
}

// Experimental fused payload metrics + hard FEC/FCS decode.  Reuses the PHR
// decisions and the CIR/phase frozen by the prefix pass.  When `phr_g0`/
// `phr_g1` are supplied by the fused PHR stage, the joint Viterbi consumes
// those raw 21 decisions directly and the prefix does not need PHR soft chips;
// otherwise it re-runs bprf_demod() from the legacy prefix.  Returns false on any unsupported
// configuration, range/short-window violation, fused-decode failure or FCS
// failure; the caller then retries the complete legacy stream once.
// `template_us`, `correlate_us` and `decode_us` are populated even when a later
// stage fails so the attempted work is not hidden.
inline bool stage_payload_fused_cmf(const std::complex<float>* rx,
                                    size_t n,
                                    const Qm35825Profile& profile,
                                    const TimingResult& timing,
                                    const CirResult& cir,
                                    const std::vector<float>& prefix_soft_chips,
                                    const NsSfdResult& ns_sfd,
                                    const PhrResult& phr,
                                    PayloadResult& out,
                                    DemodScratch& scratch,
                                    uint64_t& template_us,
                                    uint64_t& correlate_us,
                                    uint64_t& decode_us,
                                    uint64_t& decode_ns,
                                    size_t& symbols_out,
                                    const std::vector<int8_t>* phr_g0 = nullptr,
                                    const std::vector<int8_t>* phr_g1 = nullptr,
                                    PayloadSubTimers* sub = nullptr)
{
    // Diagnostic child timing of `stage_payload_us`; see stage_payload_fcs().
    detail::PayloadSubTimer sub_guard(sub);
    if (sub)
        ++sub->fused_calls;
    out = PayloadResult{};
    template_us = 0;
    correlate_us = 0;
    decode_us = 0;
    decode_ns = 0;
    symbols_out = 0;
    if (!rx || !phr.ok || !ns_sfd.ok || phr.psdu_length == 0 ||
        phr.psdu_length > profile.max_psdu_bytes)
        return false;

    const int64_t phr_start = ns_sfd.sfd_end_chip + 1;
    const int64_t payload_start =
        phr_start + static_cast<int64_t>(rxopt::kPhrSymbols) *
                       static_cast<int64_t>(rxopt::kPhrChipsPerSymbol);
    if (phr_start < 0 || payload_start < 0)
        return false;
    // When the fused PHR stage supplies the raw 21 g0/g1 decisions, the
    // payload path does not need the PHR soft chips in the prefix.  Otherwise
    // it must reproduce the legacy bprf_demod() admission exactly.
    const bool have_phr_decisions = phr_g0 && phr_g1;
    if (!have_phr_decisions &&
        static_cast<size_t>(phr_start) +
                rxopt::kPhrSymbols * rxopt::kPhrChipsPerSymbol >
            prefix_soft_chips.size())
        return false;

    const size_t psdu_bits = static_cast<size_t>(phr.psdu_length) * 8;
    const size_t num_blocks = (psdu_bits + 329) / 330;
    const size_t symbols = psdu_bits + 48 * num_blocks;
    if (symbols == 0)
        return false;
    symbols_out = symbols;

    const size_t tap_count = cir.cir_filter_taps.size();
    if (tap_count == 0 || tap_count > rxfusion::kMaxTapCount)
        return false;
    if (tap_count != profile.cir_pre_samples + profile.cir_post_samples)
        return false;
    if (!integer_chip_grid_at(cir.samples_per_chip,
                              rxfusion::kSamplesPerChip,
                              profile.fused_gate_ppm * 1e-6))
        return false;
    if (profile.cir_pre_samples > tap_count)
        return false;
    if (!scratch.softchip.prepared || std::norm(scratch.softchip.rot) <= 0.0f ||
        !std::isfinite(scratch.softchip.rot.real()) ||
        !std::isfinite(scratch.softchip.rot.imag()))
        return false;

    const int64_t chip_start =
        timing.preamble_start_sample + static_cast<int64_t>(tap_count) -
        static_cast<int64_t>(profile.cir_pre_samples) - 1;
    if (chip_start < 0 || static_cast<int64_t>(n) <= chip_start)
        return false;
    const int64_t max_chip =
        (static_cast<int64_t>(n) - 1 - chip_start) /
        static_cast<int64_t>(rxfusion::kSamplesPerChip);
    if (max_chip < 0)
        return false;

    // Reproduce the exact length of the soft-chip stream that the legacy
    // on-demand path would have produced (prefix + optional extension).  The
    // legacy bprf_demod() admission is `payload_start + 64*nsym <= stream`,
    // which is 16 chips stricter than the last actually used chip.  Using only
    // the raw-IQ length here would be a different contract if the chip budget
    // ever became the limiting factor.
    const size_t prefix_chips = cir.soft_chip_count;
    size_t stream_chips = prefix_chips;
    const size_t final_budget = rxopt::final_chip_budget(
        profile, ns_sfd.sfd_end_chip, phr.psdu_length);
    if (final_budget > prefix_chips) {
        const double spc = cir.samples_per_chip;
        const int64_t last_by_budget =
            chip_start + static_cast<int64_t>(std::ceil(
                             static_cast<double>(final_budget - 1) * spc));
        const int64_t last_chip_sample =
            std::min<int64_t>(static_cast<int64_t>(n), last_by_budget);
        const int64_t last_pos = last_chip_sample - 1;
        if (last_pos < chip_start)
            return false;
        stream_chips = static_cast<size_t>(
                           std::floor(static_cast<double>(last_pos - chip_start) /
                                      spc)) +
                       1;
    }
    if (payload_start + static_cast<int64_t>(symbols) *
                            static_cast<int64_t>(rxopt::kPayloadChipsPerSymbol) >
        static_cast<int64_t>(stream_chips))
        return false;

    const int8_t pol = (ns_sfd.polarity < 0) ? -1 : 1;
    rxfusion::Timings timings;
    {
        // PN build + four-candidate correlation is the fused "soft bit"
        // generation; it is the soft bucket of this path.
        detail::PayloadSubBucket soft_bucket(sub, &PayloadSubTimers::soft_us);
        const auto t_plan0 = std::chrono::steady_clock::now();
        if (!prepare_fused_payload_pn(scratch, profile.code_index, symbols))
            return false;
        const auto t_plan1 = std::chrono::steady_clock::now();
        const uint64_t plan_us = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::microseconds>(t_plan1 -
                                                                  t_plan0)
                .count());

        if (!rxfusion::correlate_payload(
                scratch.derotated.data(), n, cir.cir_filter_taps.data(),
                tap_count, chip_start, rxfusion::kSamplesPerChip, payload_start,
                scratch.fused.pn.data(), symbols, pol, scratch.softchip.rot,
                scratch.fused.metrics, scratch.fused.kernel, &timings))
            return false;
        template_us = plan_us + timings.template_update_us;
        correlate_us = timings.correlate_us;
        for (float v : scratch.fused.metrics.signed_value) {
            if (!std::isfinite(v))
                return false;
        }
    }

    const auto t_decode0 = std::chrono::steady_clock::now();
    std::vector<int8_t> local_g0p, local_g1p, g0, g1;
    const std::vector<int8_t>* g0p = phr_g0;
    const std::vector<int8_t>* g1p = phr_g1;
    {
        // PHR re-demod (only when the fused PHR stage supplied no decisions) and
        // the BPM decisions belong to the metric-generation bucket.
        detail::PayloadSubBucket soft_bucket(sub, &PayloadSubTimers::soft_us);
        if (g0p || g1p) {
            if (!g0p || !g1p || g0p->size() != 21 || g1p->size() != 21) {
                decode_ns = static_cast<uint64_t>(
                    std::chrono::duration_cast<std::chrono::nanoseconds>(
                        std::chrono::steady_clock::now() - t_decode0)
                        .count());
                decode_us = decode_ns / 1000;
                return false;
            }
        } else {
            if (!detail::bprf_demod(prefix_soft_chips, phr_start,
                                    profile.code_index, 21, 64, 512, 0, pol,
                                    local_g0p, local_g1p)) {
                decode_ns = static_cast<uint64_t>(
                    std::chrono::duration_cast<std::chrono::nanoseconds>(
                        std::chrono::steady_clock::now() - t_decode0)
                        .count());
                decode_us = decode_ns / 1000;
                return false;
            }
            g0p = &local_g0p;
            g1p = &local_g1p;
        }
        rxfusion::decide_bpm(scratch.fused.metrics, g0, g1);
    }
    const bool decoded = detail::decode_payload_hard(*g0p, *g1p, g0, g1,
                                                     psdu_bits, out, sub);
    decode_ns = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now() - t_decode0)
            .count());
    decode_us = decode_ns / 1000;
    if (!decoded)
        return false;
    return out.ok && out.fcs_pass;
}

// ---------------------------------------------------------------------------
// Full pipeline (R1-R4): timing + CFO + SFD + CIR/soft chips + NS-SFD + PHR +
// payload/FCS.
// ---------------------------------------------------------------------------
inline DemodResult demodulate_one(const std::complex<float>* rx,
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
    scratch.fused_phr.reset_decisions();

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
    auto rebase = [&]() {
        res.stage_payload_soft_us = payload_sub.soft_us;
        res.stage_payload_viterbi_us = payload_sub.viterbi_us;
        res.stage_payload_rs_us = payload_sub.rs_us;
        res.stage_payload_crc_us = payload_sub.crc_us;
        res.stage_payload_other_us = payload_sub.other_us;
        res.stage_payload_derotate_us = payload_sub.other_derotate_us;
        res.stage_payload_fused_calls = payload_sub.fused_calls;
        res.stage_payload_legacy_calls = payload_sub.legacy_calls;
        if (window_start == 0)
            return;
        if (res.timing.ok) {
            res.timing.preamble_start_sample += window_start;
            res.timing.preamble_start_uncropped += window_start;
            for (auto& p : res.timing.peak_samples)
                p += window_start;
        }
        if (res.cfo.ok) {
            res.cfo.fit_first_peak_sample += window_start;
            res.cfo.fit_last_peak_sample += window_start;
        }
        if (res.sfd.ok) {
            res.sfd.sfd_start_sample += window_start;
            res.sfd.sfd_end_sample += window_start;
        }
        if (res.sfd_initial_predicted_sample >= 0)
            res.sfd_initial_predicted_sample += window_start;
        if (res.sfd_bootstrap_detected_sample >= 0)
            res.sfd_bootstrap_detected_sample += window_start;
        if (res.cir.ok)
            res.cir.first_path_sample += window_start;
    };

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
    bool integer_grid_ok = true;
    // Per-packet effective profile.  Only the soft-chip mode can differ from
    // the caller's profile (see the degradation note below).
    Qm35825Profile eff = profile;
    if (profile.rx_softchip_on_demand || profile.rx_fused_payload_cmf ||
        profile.rx_fused_phr_cmf) {
        const double spc_hat =
            (res.timing.measured_period > 0.0)
                ? res.timing.measured_period /
                      static_cast<double>(kQm35ChipsPerSymbol)
                : 0.0;
        integer_grid_ok =
            integer_chip_grid(spc_hat, profile.fused_gate_ppm * 1e-6);
        if (!integer_grid_ok) {
            ++res.stage_grid_rejects;
            // Graceful degradation.  `Full` soft chips exist only to feed the
            // fused CMF; once that is refused they are a 38-tap matched filter
            // over the whole payload (~15 ms).  Bypass (strongest-path chip
            // sampling) is the cheap legacy path (~0.5 ms of CIR) and still
            // decodes.  Without this, an untrimmed clock would turn the new
            // default from ~1.5 ms/packet into ~15 ms/packet.
            if (eff.cir_soft_chip_mode == CirSoftChipMode::Full)
                eff.cir_soft_chip_mode = CirSoftChipMode::Bypass;
        }
    }

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
    // MATLAB's per-SYNC floor is 0.20 of the candidate energy.
    cfo_anchor_profile.radar_verification_threshold = 0.20f;

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
                const bool fused_ok = stage_payload_fused_cmf(
                    scratch.derotated.data(), n, profile, res.timing, res.cir,
                    scratch.soft_chips, res.ns_sfd, res.phr, res.payload,
                    scratch, fused_template_us, fused_correlate_us,
                    fused_decode_us, fused_decode_ns, fused_symbols,
                    cached_g0, cached_g1, &payload_sub);
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

// ---------------------------------------------------------------------------
// Stage 1 — seeded timing: detect + track SYNC preamble repetitions.
// Returns false if fewer than min_valid_peaks SYNC peaks are confirmed.
// ---------------------------------------------------------------------------
bool stage_timing(const std::complex<float>* rx,
                  size_t n,
                  const Qm35825Profile& profile,
                  const std::vector<std::complex<float>>& template_wf,
                  int64_t seed_start, // absolute predicted start, or -1
                  TimingResult& out,
                  DemodScratch& scratch);

// ---------------------------------------------------------------------------
// Stage 2 — CFO estimation + compensation.
// Uses stable SYNC peaks after the configured CFO startup skip to fit phase
// vs time, then derotates the frame.
// ---------------------------------------------------------------------------
bool stage_cfo(const std::complex<float>* rx,
               size_t n,
               const Qm35825Profile& profile,
               const TimingResult& timing,
               CfoResult& out,
               DemodScratch& scratch);

// ---------------------------------------------------------------------------
// Stage 3 — SFD template selection + full-rate timing refinement.
// Picks the best-matching SFD template (4z2 for QM35825) and refines the
// preamble start at full sample rate.
// ---------------------------------------------------------------------------
bool stage_sfd(const std::complex<float>* rx,
               size_t n,
               const Qm35825Profile& profile,
               const TimingResult& timing,
               const std::vector<int8_t>& sfd_sequence,
               const std::vector<std::complex<float>>& template_wf,
               SfdResult& out,
               DemodScratch& scratch);

// ---------------------------------------------------------------------------
// Stage 4 — CIR estimation + soft-chip generation.
// Estimates CIR from the last cir_repetitions SYNC symbols, then matched-
// filters the frame to produce a soft-chip stream.
// ---------------------------------------------------------------------------
bool stage_cir_softchips(const std::complex<float>* rx,
                         size_t n,
                         const Qm35825Profile& profile,
                         const TimingResult& timing,
                         const std::vector<int8_t>& preamble_code,
                         CirResult& out,
                         DemodScratch& scratch,
                         size_t max_chips,
                         size_t begin_chip);

// ---------------------------------------------------------------------------
// Stage 5 — NS-SFD location in the soft-chip stream.
// ---------------------------------------------------------------------------
bool stage_ns_sfd(const std::vector<float>& soft_chips,
                  const Qm35825Profile& profile,
                  const std::vector<int8_t>& sfd_sequence,
                  size_t chips_per_symbol,
                  NsSfdResult& out,
                  DemodScratch& scratch);

// ---------------------------------------------------------------------------
// Stage 6 — BPRF PHR demod + convolutional decode + SECDED.
// ---------------------------------------------------------------------------
bool stage_phr(const std::vector<float>& soft_chips,
               const Qm35825Profile& profile,
               const NsSfdResult& ns_sfd,
               size_t chips_per_symbol,
               PhrResult& out,
               DemodScratch& scratch);

// ---------------------------------------------------------------------------
// Stage 7 — payload BPM-BPSK demod + convolutional + RS + FCS.
// ---------------------------------------------------------------------------
bool stage_payload_fcs(const std::vector<float>& soft_chips,
                       const Qm35825Profile& profile,
                       const PhrResult& phr,
                       const NsSfdResult& ns_sfd,
                       size_t chips_per_symbol,
                       PayloadResult& out,
                       DemodScratch& scratch,
                       PayloadSubTimers* sub);

// ---------------------------------------------------------------------------
// Full pipeline: runs all stages in order, stopping at the first failure.
// Used by the worker thread for each job.
// ---------------------------------------------------------------------------
DemodResult demodulate_one(const std::complex<float>* rx,
                           size_t n,
                           const Qm35825Profile& profile,
                           uint64_t packet_id,
                           int64_t predicted_start,
                           int64_t window_start,
                           const std::vector<std::complex<float>>& template_wf,
                           DemodScratch& scratch);

} // namespace core
} // namespace demod
} // namespace uwb
} // namespace gr
