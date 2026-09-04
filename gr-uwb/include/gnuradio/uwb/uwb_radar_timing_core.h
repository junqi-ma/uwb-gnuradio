/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Header-only monostatic-radar SYNC origin backtrack at 998.4 MS/s.
 *
 * Nominal origin is sfd_start - sync_repetitions * samples_per_symbol.
 * For 4z1/2/3/4 the SFD still follows the last SYNC, so SFD mode /
 * n_sfd_symbols does not change this formula; callers may keep n_sfd_symbols
 * only to assert that the SFD length is known and nonzero.
 *
 * Unlike demod timing backtrack / stage_sfd this does not hunt extra SYNC
 * symbols. Refine is strictly [nominal-margin, nominal+margin], clipped to
 * legal SYNC-template starts, scoring every integer start at full rate.
 */

#pragma once

#include <gnuradio/uwb/uwb_detector_core.h>
#include <gnuradio/uwb/uwb_radar_checked_math.h>

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdint>
#include <limits>

namespace gr {
namespace uwb {
namespace radar {

enum class TimingStatus : uint8_t { Ok = 0, TimingFailed = 1, InvalidInput = 2 };

struct RadarTimingResult {
    TimingStatus status = TimingStatus::TimingFailed;
    int64_t preamble_start_sample = -1; // 0-based; -1 on failure
    int64_t nominal_preamble_start = -1;
    int64_t sfd_start_sample = -1;
    int64_t refine_lo = 0;
    int64_t refine_hi = -1;
    float metric = 0.f;
    uint32_t refine_correlations = 0;
};

inline int64_t nominal_preamble_start(int64_t sfd_start,
                                      size_t sync_repetitions,
                                      size_t samples_per_symbol)
{
    if (sfd_start < 0 || sync_repetitions == 0 || samples_per_symbol == 0)
        return -1;
    int64_t reps = 0;
    int64_t sps = 0;
    int64_t span = 0;
    int64_t origin = 0;
    if (!radar_i64_from_size(sync_repetitions, reps) ||
        !radar_i64_from_size(samples_per_symbol, sps) ||
        !radar_i64_mul(reps, sps, span) ||
        !radar_i64_sub(sfd_start, span, origin))
        return -1;
    return origin;
}

// rx: 998.4 CF32 window. sync_template: L2-normalized, length == samples_per_symbol.
// HOT PATH: no allocation.
inline bool refine_sync_origin(const std::complex<float>* rx,
                               size_t n,
                               int64_t sfd_start,
                               size_t sync_repetitions,
                               size_t samples_per_symbol,
                               int64_t margin_samples,
                               float threshold,
                               const std::complex<float>* sync_template,
                               size_t sync_len,
                               RadarTimingResult& out)
{
    out = RadarTimingResult{};
    out.sfd_start_sample = sfd_start;

    bool tmpl_ok = sync_template != nullptr && sync_len != 0 &&
                   samples_per_symbol != 0 && sync_len == samples_per_symbol;
    if (tmpl_ok) {
        for (size_t k = 0; k < sync_len; ++k) {
            if (!std::isfinite(sync_template[k].real()) ||
                !std::isfinite(sync_template[k].imag())) {
                tmpl_ok = false;
                break;
            }
        }
    }

    int64_t n64 = 0;
    int64_t L = 0;
    int64_t reps = 0;
    int64_t sps = 0;
    int64_t span = 0;
    int64_t nominal = 0;
    int64_t max_start = 0;
    int64_t lo_unclip = 0;
    int64_t hi_unclip = 0;
    const bool coords_ok =
        radar_i64_from_size(n, n64) &&
        radar_i64_from_size(samples_per_symbol, L) &&
        radar_i64_from_size(sync_repetitions, reps) &&
        radar_i64_from_size(samples_per_symbol, sps) &&
        radar_i64_mul(reps, sps, span) &&
        sfd_start >= 0 && radar_i64_sub(sfd_start, span, nominal) &&
        radar_i64_sub(n64, L, max_start) &&
        margin_samples >= 0 &&
        radar_i64_sub(nominal, margin_samples, lo_unclip) &&
        radar_i64_add(nominal, margin_samples, hi_unclip);

    if (!rx || n == 0 || sfd_start < 0 || sync_repetitions == 0 ||
        samples_per_symbol == 0 || margin_samples < 0 || !tmpl_ok ||
        !std::isfinite(threshold) || !(threshold > 0.0f) || !coords_ok) {
        out.status = TimingStatus::InvalidInput;
        out.nominal_preamble_start =
            coords_ok ? nominal
                      : nominal_preamble_start(sfd_start, sync_repetitions,
                                               samples_per_symbol);
        return false;
    }
    out.nominal_preamble_start = nominal;
    const int64_t lo = std::max<int64_t>(0, lo_unclip);
    const int64_t hi = std::min(max_start, hi_unclip);
    out.refine_lo = lo;
    out.refine_hi = hi;

    // Empty after clip (SYNC template does not fit at any legal j) is a
    // refine miss, not a caller-contract error. Never write the prediction.
    if (max_start < 0 || hi < lo) {
        out.status = TimingStatus::TimingFailed;
        out.preamble_start_sample = -1;
        return false;
    }

    float best = -1.0f;
    int64_t best_j = lo;
    float pwr = 0.0f;
    {
        const size_t js = static_cast<size_t>(lo);
        for (size_t k = 0; k < sync_len; ++k)
            pwr += std::norm(rx[js + k]);
    }
    for (int64_t j = lo;; ++j) {
        if (j > lo) {
            const size_t js = static_cast<size_t>(j);
            pwr += std::norm(rx[js + sync_len - 1]) - std::norm(rx[js - 1]);
        }
        std::complex<float> acc(0.0f, 0.0f);
        const size_t js = static_cast<size_t>(j);
        for (size_t k = 0; k < sync_len; ++k)
            acc += rx[js + k] * std::conj(sync_template[k]);
        if (out.refine_correlations <
            std::numeric_limits<uint32_t>::max())
            ++out.refine_correlations;
        const float m =
            std::norm(acc) / (pwr + gr::uwb::core::kUwbEpsilon);
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
        out.status = TimingStatus::TimingFailed;
        out.preamble_start_sample = -1;
        return false;
    }

    out.status = TimingStatus::Ok;
    out.preamble_start_sample = best_j;
    return true;
}

} // namespace radar
} // namespace uwb
} // namespace gr
