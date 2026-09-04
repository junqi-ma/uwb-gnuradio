/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Header-only monostatic-radar SFD search at 998.4 MS/s.
 *
 * Unlike demod::core::stage_sfd this does not take TimingResult, does not
 * hunt ±1..±4 extra symbols, and does not realloc a correlation buffer.
 * The caller supplies a predicted SFD start (known TX time) and a margin;
 * only integer starts in the clipped [predicted-margin, predicted+margin]
 * window are scored.
 */

#pragma once

#include <gnuradio/uwb/uwb_detector_core.h>
#include <gnuradio/uwb/uwb_radar_checked_math.h>

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdint>
#include <limits>
#include <vector>

namespace gr {
namespace uwb {
namespace radar {

enum class SfdStatus : uint8_t { Ok = 0, SfdFailed = 1, InvalidInput = 2 };

struct RadarSfdScratch {
    std::vector<std::complex<float>> sfd_template; // L2-normalized

    // Only used by prepare_sfd_template, never by search_sfd.
    void reserve(size_t sfd_len) { sfd_template.reserve(sfd_len); }
};

struct RadarSfdResult {
    SfdStatus status = SfdStatus::SfdFailed;
    int64_t sfd_start_sample = -1; // 0-based; -1 on failure
    int64_t predicted_start_sample = -1;
    int64_t search_lo = 0;
    int64_t search_hi = -1;
    float metric = 0.f;
    uint32_t coarse_correlations = 0;
    uint32_t fine_correlations = 0;
};

// May allocate. Call once at init / when the SFD template changes.
inline bool prepare_sfd_template(const int8_t* sfd_sequence,
                                 size_t n_sfd_symbols,
                                 const std::complex<float>* sync_template,
                                 size_t sync_len,
                                 RadarSfdScratch& scratch)
{
    if (!sfd_sequence || n_sfd_symbols == 0 || !sync_template ||
        sync_len == 0) {
        scratch.sfd_template.clear();
        return false;
    }
    if (n_sfd_symbols > std::numeric_limits<size_t>::max() / sync_len) {
        scratch.sfd_template.clear();
        return false;
    }
    const size_t sfd_len = n_sfd_symbols * sync_len;
    scratch.reserve(sfd_len);
    scratch.sfd_template.resize(sfd_len);
    double energy = 0.0;
    for (size_t i = 0; i < n_sfd_symbols; ++i) {
        const float s = static_cast<float>(sfd_sequence[i]);
        for (size_t k = 0; k < sync_len; ++k) {
            const float re = s * sync_template[k].real();
            const float im = s * sync_template[k].imag();
            if (!std::isfinite(re) || !std::isfinite(im)) {
                scratch.sfd_template.clear();
                return false;
            }
            scratch.sfd_template[i * sync_len + k] =
                std::complex<float>(re, im);
            energy += static_cast<double>(re) * static_cast<double>(re) +
                      static_cast<double>(im) * static_cast<double>(im);
        }
    }
    if (!(energy > 0.0) || !std::isfinite(energy)) {
        scratch.sfd_template.clear();
        return false;
    }
    gr::uwb::core::uwb_l2_normalize(scratch.sfd_template);
    return true;
}

// HOT PATH: no heap allocation / no vector growth.
// Success only if best metric >= threshold; then sfd_start_sample = best j.
// On failure never writes predicted_start into sfd_start_sample.
inline bool search_sfd(const std::complex<float>* rx,
                       size_t n,
                       int64_t predicted_start,
                       int64_t margin_samples,
                       float threshold,
                       RadarSfdResult& out,
                       const RadarSfdScratch& scratch)
{
    out = RadarSfdResult{};
    out.predicted_start_sample = predicted_start;

    if (!rx || n == 0 || predicted_start < 0 || margin_samples < 0 ||
        scratch.sfd_template.empty() || !std::isfinite(threshold) ||
        !(threshold > 0.0f)) {
        out.status = SfdStatus::InvalidInput;
        return false;
    }

    const size_t sfd_len = scratch.sfd_template.size();
    int64_t n64 = 0;
    int64_t sfd_len64 = 0;
    int64_t max_start = 0;
    int64_t lo_unclip = 0;
    int64_t hi_unclip = 0;
    const bool coords_ok = radar_i64_from_size(n, n64) &&
                           radar_i64_from_size(sfd_len, sfd_len64) &&
                           radar_i64_sub(n64, sfd_len64, max_start) &&
                           radar_i64_sub(predicted_start, margin_samples,
                                         lo_unclip) &&
                           radar_i64_add(predicted_start, margin_samples,
                                         hi_unclip);
    if (!coords_ok) {
        out.status = SfdStatus::InvalidInput;
        return false;
    }
    const int64_t lo = std::max<int64_t>(0, lo_unclip);
    const int64_t hi = std::min(max_start, hi_unclip);
    out.search_lo = lo;
    out.search_hi = hi;

    // Empty after clip (SFD does not fit at any legal j) is a search miss,
    // not a caller-contract error.
    if (max_start < 0 || hi < lo) {
        out.status = SfdStatus::SfdFailed;
        return false;
    }

    // First version scores every integer start in the clipped window.
    // A stride-8 coarse peak can sit on a near-zero sidelobe (metric at
    // lag 2 is ~0) and the subsequent ±7 refine never reaches the true
    // start; Codex R1 forbids that blind spot.
    const std::complex<float>* tmpl = scratch.sfd_template.data();

    float best = -1.0f;
    int64_t best_j = lo;
    float pwr = 0.0f;
    {
        const size_t js = static_cast<size_t>(lo);
        for (size_t k = 0; k < sfd_len; ++k)
            pwr += std::norm(rx[js + k]);
    }
    for (int64_t j = lo;; ++j) {
        if (j > lo) {
            const size_t js = static_cast<size_t>(j);
            pwr += std::norm(rx[js + sfd_len - 1]) - std::norm(rx[js - 1]);
        }
        std::complex<float> acc(0.0f, 0.0f);
        const size_t js = static_cast<size_t>(j);
        for (size_t k = 0; k < sfd_len; ++k)
            acc += rx[js + k] * std::conj(tmpl[k]);
        if (out.fine_correlations < std::numeric_limits<uint32_t>::max())
            ++out.fine_correlations;
        const float m = std::norm(acc) / (pwr + gr::uwb::core::kUwbEpsilon);
        if (m > best) {
            best = m;
            best_j = j;
        }
        // Do not increment INT64_MAX after scoring the final legal start.
        if (j == hi)
            break;
    }

    out.metric = best;
    if (best < threshold) {
        out.status = SfdStatus::SfdFailed;
        out.sfd_start_sample = -1;
        return false;
    }

    out.status = SfdStatus::Ok;
    out.sfd_start_sample = best_j;
    return true;
}

} // namespace radar
} // namespace uwb
} // namespace gr
