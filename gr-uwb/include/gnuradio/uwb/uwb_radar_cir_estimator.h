/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Header-only monostatic-radar CIR estimator at 998.4 MS/s.
 *
 * Ports MATLAB +uwbdecoder/estimateCir.m and testdata/uwb_radar
 * local_estimate_cir (0-based).  Input is a known preamble origin; output
 * is complex raw CIR plus L2-normalized CIR.  Not a GNU Radio block.
 */

#pragma once

#include <gnuradio/uwb/uwb_phy_profile.h>
#include <gnuradio/uwb/uwb_detector_core.h>
#include <gnuradio/uwb/uwb_radar_timing_core.h>

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdint>
#include <limits>
#include <vector>

namespace gr {
namespace uwb {
namespace radar {

enum class CirStatus : uint8_t { Ok = 0, CirFailed = 1, InvalidInput = 2 };

struct RadarCirEstimate {
    CirStatus status = CirStatus::CirFailed;
    size_t tap_count = 0;
    size_t peak_tap = 0;     // 0-based on CIR grid
    float peak_abs = 0.f;    // |raw| at peak
    float raw_l2_norm = 0.f;
    size_t valid_repetitions = 0;
    size_t first_repetition = 0;
};

struct RadarCirScratch {
    std::vector<std::complex<float>> sampled_code; // 1016
    float code_energy = 0.f;
    std::vector<std::complex<float>> avg;          // capacity >= wlen
    std::vector<std::complex<float>> raw_taps;     // capacity >= tap_count
    std::vector<std::complex<float>> norm_taps;    // capacity >= tap_count

    // May allocate. Call from prepare, never from estimate_radar_cir.
    void reserve(size_t code_len, size_t max_taps)
    {
        sampled_code.reserve(code_len);
        const size_t max_wlen = code_len + max_taps - 1;
        avg.reserve(max_wlen);
        raw_taps.reserve(max_taps);
        norm_taps.reserve(max_taps);
    }
};

namespace detail {

inline bool cir_add_overflow(size_t a, size_t b)
{
    return b > std::numeric_limits<size_t>::max() - a;
}

inline void cir_fail(RadarCirEstimate& out, CirStatus status)
{
    out = RadarCirEstimate{};
    out.status = status;
}

inline bool cir_fail_status(RadarCirEstimate& out, CirStatus status)
{
    cir_fail(out, status);
    return false;
}

} // namespace detail

// May allocate. Builds 1016 sampled_code + code_energy; sizes avg/raw/norm for max_pre+max_post.
inline bool prepare_radar_cir_code(const int8_t* hrp_code,
                                   size_t code_len,
                                   size_t max_pre,
                                   size_t max_post,
                                   RadarCirScratch& scratch)
{
    scratch.sampled_code.clear();
    scratch.code_energy = 0.f;

    if (!hrp_code || code_len == 0)
        return false;
    if (demod::kQm35SpreadingFactor > 0 &&
        code_len > std::numeric_limits<size_t>::max() / demod::kQm35SpreadingFactor)
        return false;
    if (detail::cir_add_overflow(max_pre, max_post))
        return false;

    const std::vector<int8_t> spread =
        demod::BuildSampledCode(hrp_code, code_len);
    if (spread.empty())
        return false;
    if (demod::kQm35SamplesPerChip > 0 &&
        spread.size() >
            std::numeric_limits<size_t>::max() / demod::kQm35SamplesPerChip)
        return false;

    const size_t sampled_len = spread.size() * demod::kQm35SamplesPerChip;
    const size_t max_taps = max_pre + max_post;
    size_t max_wlen = 0;
    if (max_taps == 0) {
        if (sampled_len == 0)
            return false;
        max_wlen = sampled_len - 1;
    } else {
        if (sampled_len > std::numeric_limits<size_t>::max() - (max_taps - 1))
            return false;
        max_wlen = sampled_len + max_taps - 1;
    }

    scratch.reserve(sampled_len, max_taps);
    scratch.sampled_code.assign(sampled_len, std::complex<float>(0.f, 0.f));
    for (size_t j = 0; j < spread.size(); ++j) {
        const float v = static_cast<float>(spread[j]);
        if (!std::isfinite(v)) {
            scratch.sampled_code.clear();
            scratch.code_energy = 0.f;
            return false;
        }
        scratch.sampled_code[j * demod::kQm35SamplesPerChip] =
            std::complex<float>(v, 0.f);
    }

    float energy = 0.f;
    for (size_t m = 0; m < sampled_len; ++m)
        energy += std::norm(scratch.sampled_code[m]);
    if (!(energy > 0.f) || !std::isfinite(energy)) {
        scratch.sampled_code.clear();
        scratch.code_energy = 0.f;
        return false;
    }
    scratch.code_energy = energy;

    scratch.avg.assign(max_wlen, std::complex<float>(0.f, 0.f));
    scratch.raw_taps.assign(max_taps, std::complex<float>(0.f, 0.f));
    scratch.norm_taps.assign(max_taps, std::complex<float>(0.f, 0.f));
    return true;
}

// HOT PATH: no alloc. Taps written to scratch.raw_taps/norm_taps [0, tap_count).
// On failure: out.tap_count=0; do not present leftover taps as a result.
inline bool estimate_radar_cir(const std::complex<float>* rx,
                               size_t n,
                               int64_t preamble_start,
                               size_t samples_per_symbol,
                               size_t pre,
                               size_t post,
                               size_t skip_initial,
                               size_t max_repetitions,
                               size_t preamble_repetitions,
                               RadarCirEstimate& out,
                               RadarCirScratch& scratch)
{
    detail::cir_fail(out, CirStatus::InvalidInput);

    if (!rx || n == 0 || preamble_start < 0 || samples_per_symbol == 0 ||
        scratch.sampled_code.empty() || !(scratch.code_energy > 0.f) ||
        !std::isfinite(scratch.code_energy))
        return false;
    if (detail::cir_add_overflow(pre, post))
        return false;

    const size_t tap_count = pre + post;
    if (tap_count == 0)
        return false;
    if (tap_count > scratch.raw_taps.size() ||
        tap_count > scratch.norm_taps.size() ||
        tap_count > scratch.raw_taps.capacity() ||
        tap_count > scratch.norm_taps.capacity())
        return false;

    const size_t code_len = scratch.sampled_code.size();
    if (code_len > std::numeric_limits<size_t>::max() - (tap_count - 1))
        return false;
    const size_t wlen = code_len + tap_count - 1;
    if (wlen > scratch.avg.size() || wlen > scratch.avg.capacity())
        return false;

    const size_t available = preamble_repetitions;
    if (skip_initial >= available)
        return false;

    const size_t count =
        std::min(max_repetitions, available - skip_initial);

    std::fill(scratch.avg.begin(),
              scratch.avg.begin() + static_cast<std::ptrdiff_t>(wlen),
              std::complex<float>(0.f, 0.f));

    int64_t n64 = 0;
    int64_t pre64 = 0;
    int64_t wlen64 = 0;
    int64_t period64 = 0;
    int64_t last_ok = 0;
    if (!radar_i64_from_size(n, n64) || !radar_i64_from_size(pre, pre64) ||
        !radar_i64_from_size(wlen, wlen64) ||
        !radar_i64_from_size(samples_per_symbol, period64) ||
        !radar_i64_sub(n64, wlen64, last_ok))
        return false;

    size_t valid = 0;
    for (size_t k = skip_initial; k < skip_initial + count; ++k) {
        int64_t k64 = 0;
        int64_t offset = 0;
        int64_t rs = 0;
        int64_t lo = 0;
        if (!radar_i64_from_size(k, k64) ||
            !radar_i64_mul(k64, period64, offset) ||
            !radar_i64_add(preamble_start, offset, rs) ||
            !radar_i64_sub(rs, pre64, lo))
            return false;
        if (lo < 0 || lo > last_ok)
            continue;
        const std::complex<float>* src = rx + static_cast<size_t>(lo);
        for (size_t m = 0; m < wlen; ++m)
            scratch.avg[m] += src[m];
        ++valid;
    }
    if (valid == 0)
        return detail::cir_fail_status(out, CirStatus::CirFailed);

    const float inv_valid = 1.f / static_cast<float>(valid);
    for (size_t m = 0; m < wlen; ++m)
        scratch.avg[m] *= inv_valid;

    const float energy = scratch.code_energy;
    size_t peak_tap = 0;
    float peak_abs = -1.f;
    double nrm2 = 0.0;
    for (size_t nn = 0; nn < tap_count; ++nn) {
        std::complex<double> acc(0.0, 0.0);
        for (size_t m = 0; m < code_len; ++m) {
            const std::complex<float> c = scratch.sampled_code[m];
            const std::complex<float> a = scratch.avg[nn + m];
            acc += std::complex<double>(static_cast<double>(a.real()),
                                        static_cast<double>(a.imag())) *
                   std::complex<double>(static_cast<double>(c.real()),
                                        -static_cast<double>(c.imag()));
        }
        const std::complex<float> raw(
            static_cast<float>(acc.real() / static_cast<double>(energy)),
            static_cast<float>(acc.imag() / static_cast<double>(energy)));
        scratch.raw_taps[nn] = raw;
        nrm2 += static_cast<double>(std::norm(raw));
        const float mag = std::abs(raw);
        if (mag > peak_abs) {
            peak_abs = mag;
            peak_tap = nn;
        }
    }

    const float nrm = static_cast<float>(std::sqrt(nrm2));
    if (!std::isfinite(nrm) || !(nrm > 0.f) || !std::isfinite(peak_abs))
        return detail::cir_fail_status(out, CirStatus::CirFailed);

    const float inv_n = 1.f / (nrm + 1e-12f);
    for (size_t i = 0; i < tap_count; ++i)
        scratch.norm_taps[i] = scratch.raw_taps[i] * inv_n;

    out.status = CirStatus::Ok;
    out.tap_count = tap_count;
    out.peak_tap = peak_tap;
    out.peak_abs = peak_abs;
    out.raw_l2_norm = nrm;
    out.valid_repetitions = valid;
    out.first_repetition = skip_initial;
    return true;
}

} // namespace radar
} // namespace uwb
} // namespace gr
