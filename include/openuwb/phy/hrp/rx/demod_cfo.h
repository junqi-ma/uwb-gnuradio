// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <openuwb/phy/hrp/rx/demod_sync.h>
#include <openuwb/phy/hrp/rx/rx_cfo_opt.h>

namespace openuwb::demod::core {
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
        auto collect_adj = [&](double max_dt, double& mean_dt) -> std::complex<double> {
            std::complex<double> s(0.0, 0.0);
            double dt_sum = 0.0;
            size_t cnt = 0;
            for (size_t k = 1; k < nfit; ++k) {
                const size_t idx0 = skip + k - 1;
                const size_t idx1 = skip + k;
                const double dt = static_cast<double>(
                    timing.peak_samples[idx1] - timing.peak_samples[idx0]);
                if (dt > 0.0 && dt <= max_dt) {
                    if (have_corr) {
                        const std::complex<double> c0(timing.peak_corr[idx0]);
                        const std::complex<double> c1(timing.peak_corr[idx1]);
                        s += c1 * std::conj(c0);
                    } else {
                        const double dphi = static_cast<double>(scratch.metric[k] - scratch.metric[k - 1]);
                        s += std::complex<double>(std::cos(dphi), std::sin(dphi));
                    }
                    dt_sum += dt;
                    ++cnt;
                }
            }
            mean_dt = (cnt > 0) ? (dt_sum / static_cast<double>(cnt)) : 0.0;
            return s;
        };
        double mean_dt = 0.0;
        std::complex<double> adj_sum = collect_adj(
            1.5 * static_cast<double>(kQm35SamplesPerSymbol), mean_dt);
        if (std::norm(adj_sum) > 1e-24) {
            unwrap_slope_rad_per_sample =
                std::arg(adj_sum) / static_cast<double>(kQm35SamplesPerSymbol);
        } else if (timing.fold_phase_per_symbol != 0.0) {
            unwrap_slope_rad_per_sample = timing.fold_phase_per_symbol /
                static_cast<double>(kQm35SamplesPerSymbol);
        } else {
            // Fallback for sparse trains (e.g. 16 super-anchors spaced
            // 16*1016): recover the average per-sample slope across intervals up
            // to 20 symbols so bridging gaps still unwraps cleanly.
            adj_sum = collect_adj(
                20.0 * static_cast<double>(kQm35SamplesPerSymbol), mean_dt);
            if (std::norm(adj_sum) > 1e-24 && mean_dt > 0.0) {
                unwrap_slope_rad_per_sample = std::arg(adj_sum) / mean_dt;
            }
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
    detail::LinearFit fit;
    for (size_t k = 0; k < nfit; ++k) {
        const size_t idx = skip + k;
        const double t =
            (double)timing.peak_samples[idx] / profile.sample_rate - t0;
        const double ph = (double)scratch.metric[k];
        fit.add(t, ph);
    }
    const double denom = fit.determinant();
    if (std::abs(denom) < 1e-30)
        return false;
    const double slope = fit.slope(denom);
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

} // namespace openuwb::demod::core
