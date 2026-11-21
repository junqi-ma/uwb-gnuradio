/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Out-of-line implementations of the standalone UWB detection core
 * (declarations in include/openuwb/dsp/detector_core.h):
 *
 *   - the sliding-window power / energy and normalized-correlation helpers,
 *   - the candidate-range energy gates (full-rate and decimated),
 *   - the coarse decimated preamble scans, FC32 and SC16, plus the SC16
 *     conjugate dot product they share.
 *
 * Everything a consumer must see stays in the header: the PHY constants,
 * Sc16DotProduct, RingBufferT / SlidingEnergy / DetectorEnergyGate /
 * UwbDetectorStateMachineT (templates plus the SC16 gate specialisation) and
 * the short power/energy helpers.  This TU includes only support/ and dsp/
 * headers, so openuwb_dsp keeps its one-way dependency (no phy_hrp, no
 * runtime).
 */

#include <openuwb/dsp/detector_core.h>

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

namespace openuwb {
namespace core {

void uwb_window_power(const std::complex<float>* in,
                      size_t n,
                      size_t L,
                      float* pow)
{
    if (n == 0 || L == 0)
        return;
    float acc = sample_power_sum(in, L);
    for (size_t j = 0; j < n; ++j) {
        pow[j] = acc;
        if (j + 1 < n) {
            acc -= std::norm(in[j]);
            acc += std::norm(in[j + L]);
        }
    }
}

void uwb_window_energy(const std::complex<float>* in,
                       size_t n,
                       size_t window,
                       float* out)
{
    if (n == 0 || window == 0)
        return;
    uwb_window_power(in, n, window, out);
    for (size_t j = 0; j < n; ++j)
        out[j] /= static_cast<float>(window);
}

void uwb_normalized_score(const std::complex<float>* corr,
                          const float* winpow,
                          size_t n,
                          float template_energy,
                          float* metric)
{
    for (size_t j = 0; j < n; ++j) {
        const float m = normalized_correlation_power(corr[j], winpow[j], template_energy);
        metric[j] = (m < 0.0f) ? 0.0f : ((m > 1.0f) ? 1.0f : m);
    }
}

bool uwb_full_rate_peak(const std::complex<float>* in,
                        size_t n,
                        const std::complex<float>* tmpl,
                        size_t L,
                        float template_energy,
                        size_t j0,
                        size_t j1,
                        std::complex<float>* corr,
                        float* winpow,
                        float* metric,
                        size_t* best_start,
                        float* best_metric)
{
    if (L == 0 || n < L || j0 > j1 || j1 > n - L)
        return false;
    const size_t len = j1 - j0 + 1;
    uwb_window_power(in + j0, len, L, winpow);
    for (size_t k = 0; k < len; ++k)
        conjugate_dot_into(corr[k], in + j0 + k, tmpl, L);
    uwb_normalized_score(corr, winpow, len, template_energy, metric);
    size_t best = 0;
    for (size_t k = 1; k < len; ++k) {
        if (metric[k] > metric[best])
            best = k;
    }
    *best_start = j0 + best;
    *best_metric = metric[best];
    return true;
}

void uwb_candidate_ranges(const float* energy,
                          size_t n,
                          float threshold,
                          size_t margin,
                          std::vector<std::pair<size_t, size_t>>& ranges)
{
    ranges.clear();
    size_t i = 0;
    while (i < n) {
        if (energy[i] > threshold) {
            size_t start = i;
            while (i < n && energy[i] > threshold)
                ++i;
            size_t end = i;
            const size_t a = (start > margin) ? start - margin : 0;
            const size_t b = std::min(end + margin, n);
            if (ranges.empty() || ranges.back().second < a)
                ranges.emplace_back(a, b);
            else if (b > ranges.back().second)
                ranges.back().second = b; // merge adjacent/overlapping
        } else {
            ++i;
        }
    }
}

void uwb_energy_gate_strided(const std::complex<float>* in,
                             size_t n,
                             size_t D,
                             size_t win,
                             float threshold,
                             size_t margin,
                             std::vector<std::pair<size_t, size_t>>& ranges)
{
    ranges.clear();
    if (n == 0 || D == 0)
        return;
    const size_t nd = n / D;
    if (nd == 0)
        return;

    std::vector<float> ring(win, 0.0f); // tiny; one allocation per call
    size_t pos = 0;
    size_t cnt = 0;
    float sum = 0.0f;
    size_t start = 0;
    bool in_run = false;

    auto flush = [&](size_t last) {
        const size_t a = (start * D > margin) ? start * D - margin : 0;
        const size_t b = std::min(n, (last + 1) * D + margin);
        if (b > a)
            ranges.emplace_back(a, b);
    };

    for (size_t k = 0; k < nd; ++k) {
        const float p = std::norm(in[k * D]);
        sum -= ring[pos];
        ring[pos] = p;
        sum += p;
        pos = (pos + 1) % win;
        if (cnt < win)
            ++cnt;
        const bool on = sum / static_cast<float>(cnt) > threshold;
        if (on && !in_run) {
            in_run = true;
            start = k;
        } else if (!on && in_run) {
            in_run = false;
            flush(k > 0 ? k - 1 : 0);
        }
    }
    if (in_run)
        flush(nd - 1);

    std::vector<std::pair<size_t, size_t>> merged;
    for (auto& r : ranges) {
        if (!merged.empty() && r.first <= merged.back().second)
            merged.back().second = std::max(merged.back().second, r.second);
        else
            merged.push_back(r);
    }
    ranges.swap(merged);
}

void uwb_coarse_peaks(const std::complex<float>* in,
                      size_t scan_start,
                      size_t range_end,
                      const std::complex<float>* tmpl_ds,
                      size_t Ld,
                      size_t D,
                      size_t R,
                      size_t sym_ds,
                      float peak_rel,   // relative peak threshold (x max)
                             float exist_frac, // absolute exist floor (x R)
                             size_t stride,    // correlation stride (R=1 only)
                             std::vector<std::complex<float>>& sig_ds,
                             std::vector<float>& pow_ds,
                             std::vector<float>& score_ds,
                             std::vector<float>& metric_ds,
                             std::vector<size_t>& peaks,
                             float* max_metric)
{
    peaks.clear();
    *max_metric = 0.0f;
    if (D == 0 || R == 0 || Ld == 0)
        return;
    const size_t scan_len = range_end - scan_start;
    const size_t nd = scan_len / D;
    if (nd < Ld)
        return;

    sig_ds.resize(nd);
    pow_ds.resize(nd);
    for (size_t j = 0; j < nd; ++j)
        sig_ds[j] = in[scan_start + j * D];

    const float etd = sample_power_sum(tmpl_ds, Ld);
    if (etd <= 0.0f)
        return;

    // trailing window power on the decimated grid
    float psum = sample_power_sum(sig_ds.data(), Ld);
    pow_ds[0] = psum;
    for (size_t j = 1; j + Ld <= nd; ++j) {
        psum += std::norm(sig_ds[j + Ld - 1]) - std::norm(sig_ds[j - 1]);
        pow_ds[j] = psum;
    }

    const size_t nscore = nd - Ld + 1;
    score_ds.resize(nscore);

    // The repetition accumulation needs R whole SYNC symbols; in a streaming
    // chunk there may be fewer.  The accumulation also shortens the covered
    // region by (R-1)*sym_ds, so only use it when the chunk holds at least R
    // symbols; otherwise fall back to R=1 which covers the whole chunk and
    // still yields one peak per symbol alignment.
    const size_t avail = (sym_ds > 0) ? nscore / sym_ds : 0;
    const size_t R_eff = (avail >= R && R > 0) ? R : 1;

    // VOLK conjugate dot product helper: score[j] = |Σ sig_ds[j+k]·conj(tmpl_ds[k])|
    // / sqrt(pow·etd).  (VOLK vectorizes the decimated matched filter.)
    auto compute_score = [&](size_t j) {
        const auto acc = conjugate_dot(sig_ds.data() + j, tmpl_ds, Ld);
        score_ds[j] = std::abs(acc) / (std::sqrt(pow_ds[j] * etd + kUwbEpsilon));
    };

    if (R_eff > 1) {
        // Full correlation + repetition accumulation.
        for (size_t j = 0; j < nscore; ++j)
            compute_score(j);
    } else {
        // R=1: stride the correlation.  The SYNC peaks are ~sym_ds apart, so a
        // stride of a few decimated positions costs ~stride× less.  Each
        // strided candidate peak is then refined by a dense local scan
        // (±stride positions), restoring the precision of a full scan.
        std::fill(score_ds.begin(), score_ds.end(), 0.0f);
        const size_t S = (stride > 0) ? stride : 1;
        if (S <= 1) {
            for (size_t j = 0; j < nscore; ++j)
                compute_score(j);
        } else {
            // NOTE: a stride > 1 is experimentally UNRELIABLE for this signal —
            // the decimated matched-filter peak is only ~1 decimated position
            // wide (score 1.0 at alignment, 0.28 at ±1, ~0.05 at ±2, ~0 at ±4),
            // so a strided scan misses most symbol peaks entirely.  Kept as an
            // option, but stride must stay 1 for correct detection.
            for (size_t j = 0; j < nscore; j += S)
                compute_score(j);
            for (size_t j = 0; j < nscore; j += S)
                if (score_ds[j] > *max_metric)
                    *max_metric = score_ds[j];
            if (*max_metric < exist_frac * static_cast<float>(R_eff))
                return;
            const float thr = std::max(peak_rel * (*max_metric), 0.0f);
            for (size_t j = S; j + S < nscore; j += S) {
                if (score_ds[j] > thr && score_ds[j] > score_ds[j - S] &&
                    score_ds[j] >= score_ds[j + S]) {
                    // Dense local refinement around the strided peak.
                    const size_t lo = (j > S) ? j - S : 0;
                    const size_t hi = std::min(j + S, nscore - 1);
                    size_t best = j;
                    for (size_t jj = lo; jj <= hi; ++jj) {
                        if (score_ds[jj] == 0.0f)
                            compute_score(jj);
                        if (score_ds[jj] > score_ds[best])
                            best = jj;
                    }
                    if (peaks.empty() || best >= peaks.back() + sym_ds / 2)
                        peaks.push_back(best);
                    else if (best > peaks.back())
                        peaks.back() = best;
                }
            }
            for (auto& j : peaks)
                j = scan_start + j * D;
            return;
        }
    }

    const size_t nmet =
        (nscore > (R_eff - 1) * sym_ds) ? nscore - (R_eff - 1) * sym_ds : 0;
    if (nmet == 0)
        return;
    metric_ds.assign(nmet, 0.0f);
    for (size_t r = 0; r < R_eff; ++r)
        for (size_t j = 0; j < nmet; ++j)
            metric_ds[j] += score_ds[j + r * sym_ds];
    for (size_t j = 0; j < nmet; ++j)
        if (metric_ds[j] > *max_metric)
            *max_metric = metric_ds[j];

    // existence check: the accumulated peak must be a meaningful fraction of
    // the number of accumulated repetitions.
    if (*max_metric < exist_frac * static_cast<float>(R_eff))
        return;

    const float thr = std::max(peak_rel * (*max_metric), 0.0f);
    for (size_t j = 1; j + 1 < nmet; ++j) {
        if (metric_ds[j] > metric_ds[j - 1] &&
            metric_ds[j] >= metric_ds[j + 1] && metric_ds[j] > thr &&
            (peaks.empty() || j - peaks.back() >= sym_ds / 2)) {
            peaks.push_back(j);
        }
    }
    // convert decimated peak index -> original sample position (symbol start)
    for (auto& j : peaks)
        j = scan_start + j * D;
}

Sc16DotProduct uwb_sc16_conjugate_dot_product(
    const std::complex<int16_t>* input,
    const std::complex<int16_t>* taps,
    const std::complex<int16_t>* imag_taps,
    size_t count)
{
    Sc16DotProduct result;
    size_t k = 0;
#if defined(__AVX2__)
    __m256i acc_real_lo = _mm256_setzero_si256();
    __m256i acc_real_hi = _mm256_setzero_si256();
    __m256i acc_imag_lo = _mm256_setzero_si256();
    __m256i acc_imag_hi = _mm256_setzero_si256();
    for (; k + 8 <= count; k += 8) {
        const __m256i x = _mm256_loadu_si256(
            reinterpret_cast<const __m256i*>(input + k));
        const __m256i t = _mm256_loadu_si256(
            reinterpret_cast<const __m256i*>(taps + k));
        const __m256i ti = _mm256_loadu_si256(
            reinterpret_cast<const __m256i*>(imag_taps + k));
        const __m256i real32 = _mm256_madd_epi16(x, t);
        const __m256i imag32 = _mm256_madd_epi16(x, ti);
        const __m128i real32_lo = _mm256_castsi256_si128(real32);
        const __m128i real32_hi = _mm256_extracti128_si256(real32, 1);
        const __m128i imag32_lo = _mm256_castsi256_si128(imag32);
        const __m128i imag32_hi = _mm256_extracti128_si256(imag32, 1);
        acc_real_lo = _mm256_add_epi64(
            acc_real_lo, _mm256_cvtepi32_epi64(real32_lo));
        acc_real_hi = _mm256_add_epi64(
            acc_real_hi, _mm256_cvtepi32_epi64(real32_hi));
        acc_imag_lo = _mm256_add_epi64(
            acc_imag_lo, _mm256_cvtepi32_epi64(imag32_lo));
        acc_imag_hi = _mm256_add_epi64(
            acc_imag_hi, _mm256_cvtepi32_epi64(imag32_hi));
    }
    alignas(32) int64_t lanes[4];
    _mm256_store_si256(reinterpret_cast<__m256i*>(lanes),
                       _mm256_add_epi64(acc_real_lo, acc_real_hi));
    result.real = lanes[0] + lanes[1] + lanes[2] + lanes[3];
    _mm256_store_si256(reinterpret_cast<__m256i*>(lanes),
                       _mm256_add_epi64(acc_imag_lo, acc_imag_hi));
    result.imag = lanes[0] + lanes[1] + lanes[2] + lanes[3];
#endif
    for (; k < count; ++k) {
        const int64_t xr = input[k].real();
        const int64_t xi = input[k].imag();
        const int64_t tr = taps[k].real();
        const int64_t ti = taps[k].imag();
        result.real += xr * tr + xi * ti;
        result.imag += xi * tr - xr * ti;
    }
    return result;
}

void uwb_coarse_peaks_sc16(
                           const std::complex<int16_t>* in,
                           size_t scan_start,
                           size_t range_end,
                           const std::complex<int16_t>* tmpl_ds,
                           const std::complex<int16_t>* tmpl_imag_ds,
                           size_t Ld,
                           size_t D,
                           size_t R,
                           size_t sym_ds,
                           float peak_rel,
                           float exist_frac,
                           size_t stride,
                           std::vector<std::complex<int16_t>>& sig_ds,
                           std::vector<uint64_t>& pow_ds,
                           std::vector<float>& score_ds,
                           std::vector<float>& metric_ds,
                           std::vector<size_t>& peaks,
                           float* max_metric)
{
    peaks.clear();
    *max_metric = 0.0f;
    if (D == 0 || R == 0 || Ld == 0 || range_end <= scan_start)
        return;
    const size_t nd = (range_end - scan_start) / D;
    if (nd < Ld)
        return;

    sig_ds.resize(nd);
    pow_ds.resize(nd);
    for (size_t j = 0; j < nd; ++j)
        sig_ds[j] = in[scan_start + j * D];

    uint64_t template_energy = 0;
    for (size_t k = 0; k < Ld; ++k) {
        const int64_t re = tmpl_ds[k].real();
        const int64_t im = tmpl_ds[k].imag();
        template_energy += static_cast<uint64_t>(re * re + im * im);
    }
    if (template_energy == 0)
        return;

    const auto power = [](const std::complex<int16_t>& sample) {
        const int64_t re = sample.real();
        const int64_t im = sample.imag();
        return static_cast<uint64_t>(re * re + im * im);
    };
    uint64_t window_power = 0;
    for (size_t k = 0; k < Ld; ++k)
        window_power += power(sig_ds[k]);
    pow_ds[0] = window_power;
    for (size_t j = 1; j + Ld <= nd; ++j) {
        window_power -= power(sig_ds[j - 1]);
        window_power += power(sig_ds[j + Ld - 1]);
        pow_ds[j] = window_power;
    }

    const size_t nscore = nd - Ld + 1;
    score_ds.resize(nscore);
    const size_t avail = (sym_ds > 0) ? nscore / sym_ds : 0;
    const size_t R_eff = (avail >= R && R > 0) ? R : 1;
    const bool squared_metric = R_eff == 1;
    const auto compute_score = [&](size_t j) {
        const Sc16DotProduct corr = uwb_sc16_conjugate_dot_product(
            sig_ds.data() + j, tmpl_ds, tmpl_imag_ds, Ld);
        const double corr_power = static_cast<double>(corr.real) * corr.real +
                                  static_cast<double>(corr.imag) * corr.imag;
        const double denom = static_cast<double>(pow_ds[j]) * template_energy;
        if (denom <= 0.0) {
            score_ds[j] = 0.0f;
        } else {
            const double normalized_power = corr_power / denom;
            score_ds[j] = static_cast<float>(
                squared_metric ? normalized_power : std::sqrt(normalized_power));
        }
    };

    if (R_eff > 1) {
        for (size_t j = 0; j < nscore; ++j)
            compute_score(j);
    } else {
        std::fill(score_ds.begin(), score_ds.end(), 0.0f);
        const size_t S = stride > 0 ? stride : 1;
        if (S <= 1) {
            for (size_t j = 0; j < nscore; ++j)
                compute_score(j);
        } else {
            for (size_t j = 0; j < nscore; j += S)
                compute_score(j);
            for (size_t j = 0; j < nscore; j += S)
                *max_metric = std::max(*max_metric, score_ds[j]);
            const float exist_threshold = squared_metric
                                              ? exist_frac * exist_frac
                                              : exist_frac * R_eff;
            if (*max_metric < exist_threshold)
                return;
            const float threshold = (squared_metric ? peak_rel * peak_rel : peak_rel) *
                                    (*max_metric);
            for (size_t j = S; j + S < nscore; j += S) {
                if (score_ds[j] > threshold && score_ds[j] > score_ds[j - S] &&
                    score_ds[j] >= score_ds[j + S]) {
                    const size_t lo = j > S ? j - S : 0;
                    const size_t hi = std::min(j + S, nscore - 1);
                    size_t best = j;
                    for (size_t jj = lo; jj <= hi; ++jj) {
                        if (score_ds[jj] == 0.0f)
                            compute_score(jj);
                        if (score_ds[jj] > score_ds[best])
                            best = jj;
                    }
                    if (peaks.empty() || best >= peaks.back() + sym_ds / 2)
                        peaks.push_back(best);
                    else if (best > peaks.back())
                        peaks.back() = best;
                }
            }
            for (auto& peak : peaks)
                peak = scan_start + peak * D;
            return;
        }
    }

    const size_t nmet = nscore > (R_eff - 1) * sym_ds
                            ? nscore - (R_eff - 1) * sym_ds
                            : 0;
    if (nmet == 0)
        return;
    metric_ds.assign(nmet, 0.0f);
    for (size_t r = 0; r < R_eff; ++r)
        for (size_t j = 0; j < nmet; ++j)
            metric_ds[j] += score_ds[j + r * sym_ds];
    for (float metric : metric_ds)
        *max_metric = std::max(*max_metric, metric);
    const float exist_threshold = squared_metric
                                      ? exist_frac * exist_frac
                                      : exist_frac * static_cast<float>(R_eff);
    if (*max_metric < exist_threshold)
        return;
    const float threshold = (squared_metric ? peak_rel * peak_rel : peak_rel) *
                            (*max_metric);
    for (size_t j = 1; j + 1 < nmet; ++j) {
        if (metric_ds[j] > metric_ds[j - 1] &&
            metric_ds[j] >= metric_ds[j + 1] && metric_ds[j] > threshold &&
            (peaks.empty() || j - peaks.back() >= sym_ds / 2))
            peaks.push_back(j);
    }
    for (auto& peak : peaks)
        peak = scan_start + peak * D;
}

} // namespace core
} // namespace openuwb
