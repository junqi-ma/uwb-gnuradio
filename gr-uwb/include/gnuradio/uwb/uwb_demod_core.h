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

    void reserve(size_t n)
    {
        work.reserve(n);
        derotated.reserve(n);
        energy.reserve(n);
        metric.reserve(n);
        corr.reserve(n);
        soft_chips.reserve(n);
        cir_taps.reserve(64);
        peaks.reserve(256);
        peak_values.reserve(256);
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
                          std::complex<float>* corr_out) -> bool {
        const int64_t w0 = std::max<int64_t>(0, want - radius);
        const int64_t w1 =
            std::min<int64_t>(static_cast<int64_t>(n) - static_cast<int64_t>(L),
                              want + radius);
        if (w1 <= w0)
            return false;
        float bm = -1.0f;
        int64_t bp = w0;
        std::complex<float> best_acc(0.0f, 0.0f);
        // Adjacent candidate windows overlap by L-1 samples.  Compute power
        // once, then update it in O(1); correlation remains an exact long dot
        // product handled by VOLK.  No allocation occurs in this hot loop.
        float pw = 0.0f;
        for (size_t k = 0; k < L; ++k)
            pw += std::norm(rx[w0 + static_cast<int64_t>(k)]);
        for (int64_t j = w0; j < w1; ++j) {
            const std::complex<float> acc =
                detail::timing_conjugate_dot(rx + j, scratch.work.data(), L);
            const float m = std::norm(acc) / (pw * Et + 1e-12f);
            if (m > bm) {
                bm = m;
                bp = j;
                best_acc = acc;
            }
            if (j + 1 < w1)
                pw += std::norm(rx[j + static_cast<int64_t>(L)]) -
                      std::norm(rx[j]);
        }
        if (bm < min_metric)
            return false;
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
    float first_metric = 0.0f;
    std::complex<float> first_corr(0.0f, 0.0f);
    const float thr = profile.radar_verification_threshold;
    {
        const int64_t seed =
            (seed_start >= 0) ? seed_start
                              : (int64_t)(roi_start + (roi_end - roi_start) / 2);
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
        {
            int64_t pos = -1;
            float met = 0.0f;
            std::complex<float> corr(0.0f, 0.0f);
            if (local_best(seed, std::max<int64_t>(256, stride), thr, &pos, &met,
                           &corr)) {
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
                        detail::timing_conjugate_dot(
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
            finish_timing_profile();
            return false;
        }
    }
    // A real QM35825 has a repeatable startup transient: the first few SYNCs
    // can score below the normal verification threshold, so the detector seeds
    // the first strong SYNC.  Once locked, walk backwards on the known grid
    // with a relaxed threshold, stopping at the first missing predecessor.
    size_t backtracked_symbols = 0;
    for (size_t back = 0; back < profile.timing_max_backtrack_symbols; ++back) {
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
    const int64_t start = first_start;
    if (start < 0) {
        finish_timing_profile();
        return false;
    }
    const int64_t period = static_cast<int64_t>(kQm35SamplesPerSymbol);

    // Track all SYNC starts: expected = start + k*period, then convert each to
    // symbol-END convention (start + L - 1) to match the detector / MATLAB.
    out.peak_samples.clear();
    out.peak_metrics.clear();
    out.peak_corr.clear();
    out.peak_samples.push_back(start + static_cast<int64_t>(L - 1));
    out.peak_metrics.push_back(first_metric);
    out.peak_corr.push_back(first_corr);
    for (size_t k = 1; k < profile.preamble_repetitions; ++k) {
        const int64_t want = start + static_cast<int64_t>(k * period);
        int64_t pk = -1;
        float pm = 0.0f;
        std::complex<float> pk_corr(0.0f, 0.0f);
        const float tracking_threshold =
            (k < backtracked_symbols)
                ? 0.7f * profile.radar_verification_threshold
                : profile.radar_verification_threshold;
        if (!local_best(want, 8, tracking_threshold,
                        &pk, &pm, &pk_corr))
            break; // lost the train
        out.peak_samples.push_back(pk + static_cast<int64_t>(L - 1));
        out.peak_metrics.push_back(pm);
        out.peak_corr.push_back(pk_corr);
    }

    out.detected_peaks = out.peak_samples.size();
    out.expected_peaks = profile.preamble_repetitions;
    out.metric = first_metric;

    // Linear fit of peak positions vs repetition index -> measured period and
    // the preamble start.  peak(k) = start + k*period  =>  least squares
    // slope = period, intercept = the SYNC-start at index 0.  This matches
    // MATLAB trackCandidatePreamble: startSample = round(polyfit(...,1).coef
    // (2)), i.e. the start is derived from the WHOLE tracked train rather than
    // the first peak, so a slightly-off first peak does not bias the origin.
    const size_t np = out.peak_samples.size();
    if (np >= 2) {
        double sx = 0.0, sy = 0.0, sxx = 0.0, sxy = 0.0;
        for (size_t k = 0; k < np; ++k) {
            const double x = static_cast<double>(k);
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
inline bool stage_cfo(const std::complex<float>* rx,
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
    const bool have_corr = timing.peak_corr.size() >= np;
    scratch.metric.resize(nfit);
    for (size_t k = 0; k < nfit; ++k) {
        const size_t idx = skip + k;
        scratch.metric[k] =
            have_corr ? std::arg(timing.peak_corr[idx])
                      : std::arg(rx[timing.peak_samples[idx]]);
    }
    // Unwrap: adjacent SYNC phases must stay within ±π.  The per-symbol CFO
    // advance (2π·f·period/fs) stays well under π for |f| ≲ 150 kHz.
    for (size_t k = 1; k < nfit; ++k) {
        while (scratch.metric[k] - scratch.metric[k - 1] > (float)M_PI)
            scratch.metric[k] -= 2.0f * (float)M_PI;
        while (scratch.metric[k] - scratch.metric[k - 1] < -(float)M_PI)
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

    // Derotate a copy: out[i] = rx[i] * e^{-j w i}.  A naive per-sample sin/cos
    // (2 transcendentals / sample), a recursion rot *= e^{-j w}, or an
    // elementwise table multiply all leave the loop scalar (serial chain, or
    // std::complex mul that GCC won't SIMD).  Instead use 4 INDEPENDENT float
    // rotation chains, each advancing by step^4 — the chains have no data
    // dependency, so the compiler interleaves them (4x ILP) instead of stalling
    // on one serial accumulator.  Re-anchor every kB samples to the exact
    // absolute phase, so float precision drift never accumulates.
    scratch.derotated.resize(n);
    const double w = 2.0 * M_PI * out.cfo_hz / profile.sample_rate;
    const float cw = static_cast<float>(w);
    constexpr size_t kB = 1024;
    const float s4r = std::cos(-4.0f * cw); // e^{-j w*4}: chain advance
    const float s4i = std::sin(-4.0f * cw);
    for (size_t b0 = 0; b0 < n; b0 += kB) {
        const size_t b1 = std::min(n, b0 + kB);
        const float ph0 = cw * static_cast<float>(b0);
        float cr[4], ci[4];
        for (int c = 0; c < 4; ++c) {
            const float ph = -ph0 - cw * static_cast<float>(c);
            cr[c] = std::cos(ph);
            ci[c] = std::sin(ph);
        }
        size_t i = b0;
        for (; i + 4 <= b1; i += 4) {
            for (int c = 0; c < 4; ++c) {
                scratch.derotated[i + c] =
                    rx[i + c] * std::complex<float>(cr[c], ci[c]);
                const float nr = cr[c] * s4r - ci[c] * s4i;
                const float ni = cr[c] * s4i + ci[c] * s4r;
                cr[c] = nr;
                ci[c] = ni;
            }
        }
        for (; i < b1; ++i) { // tail (1..3 samples): exact phase
            const float ph = -ph0 - cw * static_cast<float>(i - b0);
            scratch.derotated[i] =
                rx[i] * std::complex<float>(std::cos(ph), std::sin(ph));
        }
    }
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

    // Build the SFD template: kron(sfd_sequence, preamble_waveform), then
    // L2-normalize — same as MATLAB refineTimingWithNsSfd.
    const size_t len = sfd_sequence.size();
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
    const size_t sfd_len = scratch.corr.size();

    // Search center = one measured period after the last tracked SYNC START
    // (NOT the fixed preamble_repetitions * period extrapolation), so SFO and
    // a partial SYNC train are handled.  Half-width scales with how trustworthy
    // MATLAB strategy (refineTimingWithNsSfd): search a symmetric
    // expected ± samples_per_symbol window and take the maximum.  A full
    // samples_per_symbol half-width covers an SFD that sits up to one symbol
    // earlier or later than the nominal position (e.g. a partial preamble
    // train, or a preamble start a few SYNCs into the frame).
    const int64_t half = static_cast<int64_t>(sym);
    // MATLAB strategy (refineTimingWithNsSfd): SFD start = preamble start +
    // preamble_repetitions × measured_period.  This is physically correct even
    // when the trailing preamble SYNC(s) score below the acquisition threshold
    // and are not tracked (partial train) — exactly the case where the old
    // "last_tracked_peak + period" extrapolation landed one symbol early.
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
    constexpr size_t kSfdStride = 8;
    float best = -1.0f;
    size_t best_j = 0;

    auto scan_window = [&](int64_t center, int64_t backtrack_symbols) {
        if (center < 0 || n < sfd_len)
            return false;
        const int64_t lo = std::max<int64_t>(0, center - half);
        const int64_t hi = std::min<int64_t>(
            static_cast<int64_t>(n - sfd_len), center + half);
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
        // Coarse pass.
        for (size_t j = j_lo; j <= j_end; j += kSfdStride) {
            if (j > j_lo) {
                for (size_t k = 0; k < kSfdStride; ++k)
                    pwr += std::norm(rx[j + sfd_len - 1 - k]) -
                           std::norm(rx[j - 1 - k]);
            }
            std::complex<float> acc(0.0f, 0.0f);
            ++out.coarse_correlations;
            for (size_t k = 0; k < sfd_len; ++k)
                acc += rx[j + k] * std::conj(scratch.corr[k]);
            const float m = std::norm(acc) / (pwr + 1e-12f);
            if (m > window_best) {
                window_best = m;
                window_best_j = j;
            }
        }
        // Full-rate refinement around the coarse peak.
        const size_t r_lo =
            (window_best_j >= j_lo + kSfdStride)
                ? window_best_j - kSfdStride + 1
                : j_lo;
        const size_t r_hi =
            std::min(j_end, window_best_j + kSfdStride - 1);
        pwr = 0.0f;
        for (size_t k = 0; k < sfd_len; ++k)
            pwr += std::norm(rx[r_lo + k]);
        for (size_t j = r_lo; j <= r_hi; ++j) {
            if (j > r_lo)
                pwr += std::norm(rx[j + sfd_len - 1]) -
                       std::norm(rx[j - 1]);
            std::complex<float> acc(0.0f, 0.0f);
            ++out.fine_correlations;
            for (size_t k = 0; k < sfd_len; ++k)
                acc += rx[j + k] * std::conj(scratch.corr[k]);
            const float m = std::norm(acc) / (pwr + 1e-12f);
            if (m > window_best) {
                window_best = m;
                window_best_j = j;
            }
        }
        if (window_best > best) {
            best = window_best;
            best_j = window_best_j;
        }
        if (out.first_threshold_backtrack_symbols < 0 &&
            window_best >= profile.sfd_detection_threshold) {
            out.first_threshold_backtrack_symbols = backtrack_symbols;
        }
        return window_best >= profile.sfd_detection_threshold;
    };

    // MATLAB-style symmetric ±1-symbol search: one scan_window over
    // [expected - samples_per_symbol, expected + samples_per_symbol], taking
    // the maximum correlation (no symbol stepping in either direction).
    scan_window(expected, 0);
    if (best < profile.sfd_detection_threshold)
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
                                DemodScratch& scratch)
{
    out = CirResult{};
    auto t_cir0 = std::chrono::steady_clock::now();
    const size_t pre = profile.cir_pre_samples;
    const size_t post = profile.cir_post_samples;
    const size_t tap_count = pre + post;
    const double period = timing.measured_period;
    const int64_t start = timing.preamble_start_sample;
    if (period <= 0.0 || start < 0 || preamble_code.empty() || tap_count == 0)
        return false;

    // ---- Build the sparse sampled code (508 spread x2 -> 1016, zeros on the
    //      odd samples), matching MATLAB buildUwbReference.sampled_code. ----
    const std::vector<int8_t> spread =
        BuildSampledCode(preamble_code.data(), preamble_code.size());
    scratch.work.assign(2 * kQm35ChipsPerSymbol, std::complex<float>(0.0f, 0.0f));
    for (size_t j = 0; j < spread.size(); ++j)
        scratch.work[2 * j] = std::complex<float>((float)spread[j], 0.0f);
    const size_t code_len = scratch.work.size(); // 1016
    float code_energy = 0.0f;
    for (size_t m = 0; m < code_len; ++m)
        code_energy += std::norm(scratch.work[m]);

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
        for (size_t m = 0; m < wlen; ++m)
            scratch.corr[m] += rx[static_cast<size_t>(lo + (int64_t)m)];
        ++valid;
    }
    if (valid == 0)
        return false;
    const float inv = 1.0f / static_cast<float>(valid);
    for (size_t m = 0; m < wlen; ++m)
        scratch.corr[m] *= inv;

    // ---- CIR taps: forward-order code correlation (NOT time-reversed). ----
    std::vector<std::complex<float>> values(tap_count);
    for (size_t nn = 0; nn < tap_count; ++nn) {
        std::complex<float> acc(0.0f, 0.0f);
        for (size_t m = 0; m < code_len; ++m)
            acc += std::conj(scratch.work[m]) * scratch.corr[nn + m];
        values[nn] = acc / code_energy;
    }
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
    for (size_t i = 0; i < tap_count; ++i)
        out.cir_values[i] = values[i].real(); // golden CIR is real (Q=0 channel)
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

    const int64_t chip_start = start + static_cast<int64_t>(tap_count) -
                               static_cast<int64_t>(pre) - 1; // = start + post - 1
    const int64_t last_by_budget =
        chip_start + static_cast<int64_t>(std::ceil((double)(n_chips - 1) * spc));
    const int64_t last_chip_sample = std::min<int64_t>(static_cast<int64_t>(n), last_by_budget);
    const int64_t last_pos = last_chip_sample - 1; // inclusive, 0-based
    if (last_pos < chip_start)
        return false;
    const int64_t spc_i = std::max<int64_t>(1, std::llround(spc));
    const bool fractional_grid =
        std::abs(spc - static_cast<double>(spc_i)) > 1e-9;
    const size_t num_chips =
        static_cast<size_t>(std::floor(
            static_cast<double>(last_pos - chip_start) / spc)) + 1;

    // Causal FIR on the chip grid, rewritten for contiguous forward access:
    //   chip = sum_k conj(values[T-1-k]) * rx[base-k]
    //        = sum_q conj(values[q])     * rx[base-(T-1)+q]
    // Pre-conjugated taps h[q]=conj(values[q]) once outside the chip loop.
    // Kernels: multi-acc / VOLK / fixed-38 AVX2 (see uwb_cir_fir_simd.h).
    scratch.corr.resize(num_chips);
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
            scratch.corr[i] = y0 + frac * (y1 - y0);
        }
        checked = num_chips; // fractional path completed the whole stream
    } else if (chip_start < fwd) {
        const int64_t gap = fwd - chip_start;
        checked = std::min(num_chips,
                           static_cast<size_t>((gap + spc_i - 1) / spc_i));
    }
    // Edge chips: FIR window may clip below sample 0 (bounds-checked scalar).
    for (size_t i = 0; !fractional_grid && i < checked; ++i) {
        const int64_t p = chip_start + static_cast<int64_t>(i) * spc_i;
        if (bypass_filter) {
            const int64_t idx =
                p - fwd + static_cast<int64_t>(peak_idx);
            scratch.corr[i] = idx >= 0 && idx < static_cast<int64_t>(n)
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
        scratch.corr[i] = acc;
    }
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
                scratch.corr[i] = rx0[idx];
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
                            scratch.corr.data() + i);
                    else
                        cir_fir::dot_topk_x4_avx2<8>(
                            rx0 + win, rake_indices, rake_weights,
                            scratch.corr.data() + i);
                }
            }
#endif
            for (; i < num_chips; ++i) {
                const int64_t p = chip_start + static_cast<int64_t>(i) * spc_i;
                const size_t win = static_cast<size_t>(p - fwd);
                scratch.corr[i] = cir_fir::dot_topk(
                    rx0 + win, rake_indices, rake_weights, rake_k);
            }
        } else if (fir_k == cir_fir::Kernel::Avx2Fixed &&
            tap_count == cir_fir::kDefaultTapCount) {
            for (size_t i = checked; i < num_chips; ++i) {
                const int64_t p = chip_start + static_cast<int64_t>(i) * spc_i;
                const size_t win = static_cast<size_t>(p - fwd);
                scratch.corr[i] =
                    cir_fir::dot_avx2_fixed38(rx0 + win, h_conj, tap_count);
            }
        } else if (fir_k == cir_fir::Kernel::Volk) {
            for (size_t i = checked; i < num_chips; ++i) {
                const int64_t p = chip_start + static_cast<int64_t>(i) * spc_i;
                const size_t win = static_cast<size_t>(p - fwd);
                scratch.corr[i] =
                    cir_fir::dot_volk(rx0 + win, values_nat, tap_count);
            }
        } else {
            for (size_t i = checked; i < num_chips; ++i) {
                const int64_t p = chip_start + static_cast<int64_t>(i) * spc_i;
                const size_t win = static_cast<size_t>(p - fwd);
                scratch.corr[i] =
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
    std::complex<float> gain(0.0f, 0.0f);
    {
        size_t s = 0;
        for (size_t i = 0; i < phase_len; ++i) {
            gain += static_cast<float>(spread[s]) *
                    scratch.corr[phase_first + i];
            if (++s == kQm35ChipsPerSymbol)
                s = 0;
        }
    }
    const float ang = std::arg(gain);
    const float rr = std::cos(-ang);
    const float ri = std::sin(-ang);
    const std::complex<float> rot(rr, ri);

    // Merged postprocess: compute only the rotated real component and track
    // max|real|, then normalize into soft_chips. No later stage consumes the
    // rotated complex corr stream, so writing it back wastes ~1.2 MB/packet.
    scratch.soft_chips.resize(num_chips);
    float mx = 0.0f;
    for (size_t i = 0; i < num_chips; ++i) {
        // (cr+j ci)*(rr+j ri) → real = cr*rr - ci*ri
        const float cr = scratch.corr[i].real();
        const float ci = scratch.corr[i].imag();
        const float r = cr * rr - ci * ri;
        scratch.soft_chips[i] = r;                    // temp; normalized below
        const float ar = (r >= 0.0f) ? r : -r;
        if (ar > mx)
            mx = ar;
    }
    const float inv_denom = 1.0f / (mx + 1e-12f);
    for (size_t i = 0; i < num_chips; ++i)
        scratch.soft_chips[i] *= inv_denom;
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
    const int8_t* pc = GetPreambleCode(profile.code_index);
    const std::vector<int8_t> spread_code = BuildSampledCode(pc, kQm35CodeLength);
    const size_t sfd_len = sfd_sequence.size() * spread_code.size();
    scratch.work.assign(sfd_len, std::complex<float>(0.0f, 0.0f));
    for (size_t i = 0; i < sfd_sequence.size(); ++i)
        for (size_t j = 0; j < spread_code.size(); ++j)
            scratch.work[i * spread_code.size() + j] = std::complex<float>(
                static_cast<float>(sfd_sequence[i] * spread_code[j]), 0.0f);
    float spread_norm = 0.0f;
    for (size_t i = 0; i < sfd_len; ++i)
        spread_norm += std::norm(scratch.work[i]);
    spread_norm = std::sqrt(spread_norm);

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
    // flip can inflate PSDU length (e.g. 13 → 45 on DW1000 captures) past
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
    if (out.secded_corrected && len_corr > len_uncorr) {
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
                              DemodScratch& scratch)
{
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
        if (detail::bprf_demod(soft_chips, phr_start, profile.code_index, 21,
                               64, 512, 0, pol, g0p, g1p) &&
            detail::bprf_demod(soft_chips, payload_start, profile.code_index,
                               nsym, 8, 64, 1344, pol, g0, g1)) {
            const size_t total = 21 + nsym;
            std::vector<int8_t> rx_bits(2 * total);
            for (size_t s = 0; s < 21; ++s) {
                rx_bits[2 * s] = g0p[s];
                rx_bits[2 * s + 1] = g1p[s];
            }
            for (size_t s = 0; s < nsym; ++s) {
                rx_bits[2 * (s + 21)] = g0[s];
                rx_bits[2 * (s + 21) + 1] = g1[s];
            }
            std::vector<int8_t> decoded;
            detail::vitdec_cl3(rx_bits.data(), total, decoded);
            if (decoded.size() >= total) {
                std::vector<int8_t> rs_cw(decoded.begin() + 19,
                                          decoded.begin() + (total - 2));
                std::vector<int8_t> psdu_bits_out;
                if (rs_cw.size() == nsym &&
                    detail::rs_decode_stream(rs_cw, psdu_bits,
                                             psdu_bits_out)) {
                    out.bits.assign(psdu_bits_out.begin(), psdu_bits_out.end());
                    out.bytes.resize(psdu_bits_out.size() / 8);
                    for (size_t i = 0; i < out.bytes.size(); ++i) {
                        uint8_t b = 0;
                        for (int bit = 0; bit < 8; ++bit)
                            b |= static_cast<uint8_t>(
                                     psdu_bits_out[8 * i + bit])
                                 << bit;
                        out.bytes[i] = b;
                    }
                    if (out.bytes.size() >= 2) {
                        out.calculated_fcs = detail::crc16_802154(
                            out.bytes.data(), out.bytes.size() - 2);
                        out.received_fcs =
                            static_cast<uint16_t>(
                                out.bytes[out.bytes.size() - 2]) |
                            (static_cast<uint16_t>(
                                 out.bytes[out.bytes.size() - 1])
                             << 8);
                        out.fcs_pass =
                            (out.received_fcs == out.calculated_fcs);
                    }
                    out.ok = true;
                    any_ok = true;
                    if (out.fcs_pass)
                        return true;
                }
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
            if (!detail::bprf_demod_soft(soft_chips, phr_start,
                                         profile.code_index, 21, 64, 512, 0,
                                         pol, g0p, g1p, llr0p, llr1p))
                continue;
            if (!detail::bprf_demod_soft(soft_chips, payload_start,
                                         profile.code_index, nsym, 8, 64,
                                         1344, pol, g0, g1, llr0, llr1))
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
            detail::vitdec_cl3_soft(llr0j.data(), llr1j.data(), total, decoded);
            if (decoded.size() < total)
                continue;

            std::vector<int8_t> rs_cw(decoded.begin() + 19,
                                      decoded.begin() + (total - 2));
            if (rs_cw.size() != nsym)
                continue;

            std::vector<int8_t> psdu_bits_out;
            if (!detail::rs_decode_stream(rs_cw, psdu_bits, psdu_bits_out))
                continue;

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

    // predicted_start is ABSOLUTE.  The core stages index rx as a 0-based
    // window buffer, so seed with the window-relative position.  All stage
    // outputs are window-relative; `rebase` re-bases the absolute sample
    // fields (timing / SFD sample positions / CIR first path) by window_start
    // before each return, so result coordinates match the schema (absolute).
    const int64_t rel_seed =
        (predicted_start >= 0) ? predicted_start - window_start : -1;
    auto rebase = [&]() {
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
    // Stage 2: CFO
    if (!stage_cfo(rx, n, profile, res.timing, res.cfo, scratch)) {
        lap(res.stage_cfo_us);
        res.status = DemodStatus::CfoFailed;
        rebase();
        return res;
    }
    lap(res.stage_cfo_us);
    // Stage 3: bootstrap SFD timing.  Timing acquisition can start several
    // SYNCs late when a weak leading repetition falls below its tracking
    // threshold.  Its 64-point phase train would then spill into the SFD and
    // corrupt a CFO fit, even though the SFD search can recover the true frame
    // origin by testing earlier hypotheses.  Use this first SFD only to anchor
    // the preamble; the final reported CFO is re-fitted below from that anchor.
    const auto sfd_seq = gr::uwb::demod::GetSfdSequence(profile.sfd_mode);
    const std::complex<float>* sfd_rx =
        scratch.derotated.empty() ? rx : scratch.derotated.data();
    const bool bootstrap_sfd_ok = stage_sfd(
        sfd_rx, n, profile, res.timing, sfd_seq, template_wf, res.sfd, scratch);
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

    // Match MATLAB's CFO input: retrack exactly the SYNC train implied by the
    // SFD-refined frame origin, then discard the first 24 of those peaks.  A
    // tight one-SYNC search prevents the broad acquisition scan from selecting
    // a later high-metric repetition again; no backwards probing is needed
    // because the SFD has already supplied the preamble origin.
    const int64_t refined_start =
        res.sfd.sfd_start_sample - static_cast<int64_t>(std::llround(
            static_cast<double>(profile.preamble_repetitions) *
            res.timing.measured_period));
    Qm35825Profile cfo_anchor_profile = profile;
    // `stage_timing` always performs a local ±8 refinement around the seed;
    // set the coarse/grid margin to zero so it cannot re-acquire another
    // symbol when the SFD anchor itself is already sample-accurate.
    cfo_anchor_profile.timing_search_margin = 0;
    cfo_anchor_profile.timing_coarse_stride = 1;
    cfo_anchor_profile.timing_max_backtrack_symbols = 0;
    // MATLAB's per-SYNC floor is 0.20 of the candidate energy.  The ordinary
    // acquisition threshold is intentionally stricter (0.30), but must not
    // reject a known, SFD-anchored leading SYNC during CFO retracking.
    cfo_anchor_profile.radar_verification_threshold = 0.20f;
    TimingResult cfo_timing;
    if (!stage_timing(rx, n, cfo_anchor_profile, template_wf, refined_start,
                      cfo_timing, scratch) ||
        !stage_cfo(rx, n, profile, cfo_timing, res.cfo, scratch)) {
        lap(res.stage_sfd_us);
        res.status = DemodStatus::CfoFailed;
        rebase();
        return res;
    }
    res.timing = std::move(cfo_timing);

    // Re-run SFD on the final MATLAB-aligned CFO compensation.  This removes
    // any bootstrap-CFO bias from the SFD/CIR path while retaining the same
    // full-rate refinement contract as MATLAB refineTimingWithNsSfd.
    sfd_rx = scratch.derotated.empty() ? rx : scratch.derotated.data();
    const bool final_sfd_ok = stage_sfd(
        sfd_rx, n, cfo_anchor_profile, res.timing, sfd_seq, template_wf,
        res.sfd, scratch);
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
    // Stage 4: CIR + soft chips on the CFO-compensated frame.
    const int8_t* pc = GetPreambleCode(profile.code_index);
    std::vector<int8_t> pcode(pc, pc + kQm35CodeLength);
    if (!stage_cir_softchips(scratch.derotated.data(), n, profile, res.timing,
                             pcode, res.cir, scratch)) {
        lap(res.stage_cir_us);
        res.status = DemodStatus::CirFailed;
        rebase();
        return res;
    }
    lap(res.stage_cir_us);
    // Stage 5: NS-SFD location in the soft-chip stream.
    const auto nsfd_seq = gr::uwb::demod::GetSfdSequence(profile.sfd_mode);
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
                           kQm35ChipsPerSymbol, res.payload, scratch)) {
        lap(res.stage_payload_us);
        res.status = DemodStatus::PayloadFailed;
        rebase();
        return res;
    }
    lap(res.stage_payload_us);
    res.status = (res.payload.ok && res.payload.fcs_pass)
                     ? DemodStatus::Success
                     : DemodStatus::FcsFailed;
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
                         DemodScratch& scratch);

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
                       DemodScratch& scratch);

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
