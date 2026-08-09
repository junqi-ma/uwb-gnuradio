/* -*- c++ -*- */
/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#ifndef INCLUDED_UWB_SIC_CORE_H
#define INCLUDED_UWB_SIC_CORE_H

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstddef>
#include <limits>
#include <vector>

namespace gr {
namespace uwb {
namespace sic {

enum class CancelStatus {
    Applied,
    FcsFailed,
    InvalidInput,
    AlignmentFailed,
    CfoFailed,
    GainFailed,
    SuppressionFailed
};

struct CancelOptions {
    double sample_rate = 998.4e6;
    size_t search_radius = 32;
    size_t period_samples = 1016;
    size_t alignment_first_repetition = 24;
    size_t alignment_repetitions = 4;
    size_t cfo_first_repetition = 24;
    size_t cfo_last_repetition = 128;
    size_t gain_first_repetition = 24;
    size_t gain_last_repetition = 128;
    float min_alignment_correlation = 0.70f;
    float min_suppression_db = 0.20f;
    double max_abs_cfo_hz = 100000.0;
};

struct CancelResult {
    CancelStatus status = CancelStatus::InvalidInput;
    bool sic_applied = false;
    size_t fitted_start = 0;
    size_t samples_subtracted = 0;
    float alignment_correlation = 0.0f;
    double fitted_cfo_hz = 0.0;
    std::complex<float> global_gain = { 0.0f, 0.0f };
    float suppression_db = -std::numeric_limits<float>::infinity();
};

struct CancelScratch {
    std::vector<std::complex<float>> model;
    std::vector<float> phases;
    std::vector<double> phase_times;

    void reserve(size_t max_replica_samples, size_t max_repetitions)
    {
        model.reserve(max_replica_samples);
        phases.reserve(max_repetitions);
        phase_times.reserve(max_repetitions);
    }
};

inline float normalized_correlation(const std::complex<float>* a,
                                    const std::complex<float>* b,
                                    size_t n)
{
    std::complex<double> cross(0.0, 0.0);
    double pa = 0.0;
    double pb = 0.0;
    for (size_t i = 0; i < n; ++i) {
        cross += std::conj(static_cast<std::complex<double>>(a[i])) *
                 static_cast<std::complex<double>>(b[i]);
        pa += std::norm(a[i]);
        pb += std::norm(b[i]);
    }
    const double denom = std::sqrt(pa * pb);
    return denom > 1e-30 ? static_cast<float>(std::abs(cross) / denom) : 0.0f;
}

// Trial/commit cancellation primitive for one already reconstructed packet.
// `work` is never modified unless all safety gates pass.  Scratch vectors are
// reusable by a worker; reserve them at worker construction to avoid hot-path
// allocations.  Fractional delay, PLL/CIR-slow phase and SFO are deliberately
// outside this Phase-2 first-version primitive.
inline CancelResult trial_cancel(std::vector<std::complex<float>>& work,
                                 const std::complex<float>* replica,
                                 size_t replica_samples,
                                 size_t nominal_start,
                                 bool fcs_pass,
                                 const CancelOptions& options,
                                 CancelScratch& scratch)
{
    CancelResult result;
    if (!fcs_pass) {
        result.status = CancelStatus::FcsFailed;
        return result;
    }
    if (!replica || replica_samples == 0 || work.empty() ||
        options.sample_rate <= 0.0 || options.period_samples == 0 ||
        options.alignment_repetitions == 0) {
        result.status = CancelStatus::InvalidInput;
        return result;
    }

    const size_t template_offset =
        options.alignment_first_repetition * options.period_samples;
    if (template_offset >= replica_samples) {
        result.status = CancelStatus::InvalidInput;
        return result;
    }
    const size_t template_samples = std::min(
        options.alignment_repetitions * options.period_samples,
        replica_samples - template_offset);
    if (template_samples == 0) {
        result.status = CancelStatus::InvalidInput;
        return result;
    }
    if (work.size() < template_offset + template_samples) {
        result.status = CancelStatus::AlignmentFailed;
        return result;
    }

    const size_t first_candidate = nominal_start > options.search_radius
                                       ? nominal_start - options.search_radius
                                       : 0;
    const size_t last_candidate = std::min(
        nominal_start + options.search_radius,
        work.size() - template_offset - template_samples);
    if (first_candidate > last_candidate) {
        result.status = CancelStatus::AlignmentFailed;
        return result;
    }

    float best = -1.0f;
    size_t fitted_start = first_candidate;
    for (size_t candidate = first_candidate; candidate <= last_candidate;
         ++candidate) {
        const float score = normalized_correlation(
            replica + template_offset,
            work.data() + candidate + template_offset,
            template_samples);
        if (score > best) {
            best = score;
            fitted_start = candidate;
        }
    }
    result.fitted_start = fitted_start;
    result.alignment_correlation = std::max(0.0f, best);
    if (best < options.min_alignment_correlation) {
        result.status = CancelStatus::AlignmentFailed;
        return result;
    }

    const size_t available = std::min(replica_samples, work.size() - fitted_start);
    const size_t available_repetitions = available / options.period_samples;
    const size_t cfo_first = std::min(options.cfo_first_repetition,
                                      available_repetitions);
    const size_t cfo_last = std::min(options.cfo_last_repetition,
                                     available_repetitions);
    if (cfo_first >= cfo_last) {
        result.status = CancelStatus::CfoFailed;
        return result;
    }

    scratch.phases.resize(cfo_last - cfo_first);
    scratch.phase_times.resize(cfo_last - cfo_first);
    float previous = 0.0f;
    double unwrap_offset = 0.0;
    for (size_t repetition = cfo_first; repetition < cfo_last; ++repetition) {
        const size_t first = repetition * options.period_samples;
        const size_t last = std::min(first + options.period_samples, available);
        std::complex<double> cross(0.0, 0.0);
        for (size_t i = first; i < last; ++i)
            cross += std::conj(static_cast<std::complex<double>>(replica[i])) *
                     static_cast<std::complex<double>>(work[fitted_start + i]);
        float phase = static_cast<float>(std::arg(cross));
        const size_t q = repetition - cfo_first;
        if (q != 0) {
            const double delta = static_cast<double>(phase - previous);
            if (delta > M_PI)
                unwrap_offset -= 2.0 * M_PI;
            else if (delta < -M_PI)
                unwrap_offset += 2.0 * M_PI;
        }
        previous = phase;
        scratch.phases[q] = static_cast<float>(phase + unwrap_offset);
        scratch.phase_times[q] =
            (static_cast<double>(first + last - 1) * 0.5) / options.sample_rate;
    }

    const size_t nfit = scratch.phases.size();
    double sx = 0.0, sy = 0.0, sxx = 0.0, sxy = 0.0;
    for (size_t i = 0; i < nfit; ++i) {
        const double x = scratch.phase_times[i];
        const double y = scratch.phases[i];
        sx += x;
        sy += y;
        sxx += x * x;
        sxy += x * y;
    }
    const double denom = static_cast<double>(nfit) * sxx - sx * sx;
    if (nfit < 2 || std::abs(denom) < 1e-30) {
        result.status = CancelStatus::CfoFailed;
        return result;
    }
    const double slope = (static_cast<double>(nfit) * sxy - sx * sy) / denom;
    result.fitted_cfo_hz = slope / (2.0 * M_PI);
    if (!std::isfinite(result.fitted_cfo_hz) ||
        std::abs(result.fitted_cfo_hz) > options.max_abs_cfo_hz) {
        result.status = CancelStatus::CfoFailed;
        return result;
    }

    scratch.model.resize(available);
    const double omega = 2.0 * M_PI * result.fitted_cfo_hz / options.sample_rate;
    for (size_t i = 0; i < available; ++i) {
        const double phase = omega * static_cast<double>(i);
        scratch.model[i] = replica[i] * std::complex<float>(
            static_cast<float>(std::cos(phase)), static_cast<float>(std::sin(phase)));
    }

    const size_t gain_first = std::min(
        options.gain_first_repetition * options.period_samples, available);
    const size_t gain_last = std::min(
        options.gain_last_repetition * options.period_samples, available);
    if (gain_first >= gain_last) {
        result.status = CancelStatus::GainFailed;
        return result;
    }
    std::complex<double> numerator(0.0, 0.0);
    double denominator = 0.0;
    for (size_t i = gain_first; i < gain_last; ++i) {
        numerator += std::conj(static_cast<std::complex<double>>(scratch.model[i])) *
                     static_cast<std::complex<double>>(work[fitted_start + i]);
        denominator += std::norm(scratch.model[i]);
    }
    if (denominator <= 1e-30) {
        result.status = CancelStatus::GainFailed;
        return result;
    }
    result.global_gain = static_cast<std::complex<float>>(numerator / denominator);

    double before = 0.0;
    double after = 0.0;
    for (size_t i = 0; i < available; ++i) {
        const std::complex<float> observed = work[fitted_start + i];
        const std::complex<float> residual =
            observed - result.global_gain * scratch.model[i];
        before += std::norm(observed);
        after += std::norm(residual);
    }
    if (before <= 1e-30) {
        result.status = CancelStatus::SuppressionFailed;
        return result;
    }
    result.suppression_db = static_cast<float>(
        10.0 * std::log10(before / std::max(after, 1e-30)));
    if (!std::isfinite(result.suppression_db) ||
        result.suppression_db < options.min_suppression_db) {
        result.status = CancelStatus::SuppressionFailed;
        return result;
    }

    for (size_t i = 0; i < available; ++i)
        work[fitted_start + i] -= result.global_gain * scratch.model[i];
    result.samples_subtracted = available;
    result.sic_applied = true;
    result.status = CancelStatus::Applied;
    return result;
}

} // namespace sic
} // namespace uwb
} // namespace gr

#endif /* INCLUDED_UWB_SIC_CORE_H */
