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

// QA-only FROZEN snapshot of the phase-2 scalar mainline (commit 5260299,
// pre-phase-4a), used by test_radar_cir_avx2_matches_scalar to compare the
// AVX2 kernels against the untouched scalar kernels bit-for-bit.
// Generated with: git show 5260299:gr-uwb/include/gnuradio/uwb/
//                   uwb_radar_cir_estimator.h
// All names carry a Ref suffix and live in gr::uwb::scalar_ref so both
// headers can be included in one TU.  Do NOT update this file with new
// kernel work; it is a fixed baseline on purpose.

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
namespace scalar_ref {

using namespace gr::uwb::radar; // radar_i64_* helpers

enum class CirStatusRef : uint8_t { Ok = 0, CirFailed = 1, InvalidInput = 2 };

struct RadarCirEstimateRef {
    CirStatusRef status = CirStatusRef::CirFailed;
    size_t tap_count = 0;
    size_t peak_tap = 0;     // 0-based on CIR grid
    float peak_abs = 0.f;    // |raw| at peak
    float raw_l2_norm = 0.f;
    size_t valid_repetitions = 0;
    size_t first_repetition = 0;
};

struct RadarCirScratchRef {
    std::vector<std::complex<float>> sampled_code; // 1016
    // Non-zero sampled-code entries in ascending sample order.  HRP codes
    // contain one non-zero chip sample followed by seven exact zeros, so
    // per-repetition CIR must not rescan all 1016 entries for every tap.
    // Prepared once; the hot path only reads these vectors.
    std::vector<size_t> active_code_indices;
    std::vector<std::complex<float>> active_code_values;
    float code_energy = 0.f;
    std::vector<std::complex<float>> avg;          // capacity >= wlen
    // Double-precision accumulator for estimate_radar_cir_ref_from_repetitions
    // (adds each valid repetition window, then scales by 1/N once).
    std::vector<std::complex<double>> avg_rep_sum; // capacity >= wlen
    std::vector<std::complex<float>> raw_taps;     // capacity >= tap_count
    std::vector<std::complex<float>> norm_taps;    // capacity >= tap_count

    // May allocate. Call from prepare, never from estimate_radar_cir_ref.
    void reserve(size_t code_len, size_t max_taps)
    {
        sampled_code.reserve(code_len);
        active_code_indices.reserve(code_len);
        active_code_values.reserve(code_len);
        const size_t max_wlen = code_len + max_taps - 1;
        avg.reserve(max_wlen);
        avg_rep_sum.reserve(max_wlen);
        raw_taps.reserve(max_taps);
        norm_taps.reserve(max_taps);
    }
};

namespace detail_ref {

inline bool cir_add_overflow(size_t a, size_t b)
{
    return b > std::numeric_limits<size_t>::max() - a;
}

inline void cir_fail(RadarCirEstimateRef& out, CirStatusRef status)
{
    out = RadarCirEstimateRef{};
    out.status = status;
}

inline bool cir_fail_status(RadarCirEstimateRef& out, CirStatusRef status)
{
    cir_fail(out, status);
    return false;
}

} // namespace detail_ref

// May allocate. Builds 1016 sampled_code + code_energy; sizes avg/raw/norm for max_pre+max_post.
inline bool prepare_radar_cir_code_ref(const int8_t* hrp_code,
                                   size_t code_len,
                                   size_t max_pre,
                                   size_t max_post,
                                   RadarCirScratchRef& scratch)
{
    scratch.sampled_code.clear();
    scratch.active_code_indices.clear();
    scratch.active_code_values.clear();
    scratch.code_energy = 0.f;

    if (!hrp_code || code_len == 0)
        return false;
    if (demod::kQm35SpreadingFactor > 0 &&
        code_len > std::numeric_limits<size_t>::max() / demod::kQm35SpreadingFactor)
        return false;
    if (detail_ref::cir_add_overflow(max_pre, max_post))
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
    for (size_t m = 0; m < sampled_len; ++m) {
        energy += std::norm(scratch.sampled_code[m]);
        if (scratch.sampled_code[m] != std::complex<float>(0.f, 0.f)) {
            scratch.active_code_indices.push_back(m);
            scratch.active_code_values.push_back(scratch.sampled_code[m]);
        }
    }
    if (!(energy > 0.f) || !std::isfinite(energy)) {
        scratch.sampled_code.clear();
        scratch.active_code_indices.clear();
        scratch.active_code_values.clear();
        scratch.avg_rep_sum.clear();
        scratch.code_energy = 0.f;
        return false;
    }
    scratch.code_energy = energy;

    scratch.avg.assign(max_wlen, std::complex<float>(0.f, 0.f));
    scratch.avg_rep_sum.assign(max_wlen, std::complex<double>(0.0, 0.0));
    scratch.raw_taps.assign(max_taps, std::complex<float>(0.f, 0.f));
    scratch.norm_taps.assign(max_taps, std::complex<float>(0.f, 0.f));
    return true;
}

// HOT PATH: no alloc. Taps written to scratch.raw_taps/norm_taps [0, tap_count).
// On failure: out.tap_count=0; do not present leftover taps as a result.
inline bool estimate_radar_cir_ref(const std::complex<float>* rx,
                               size_t n,
                               int64_t preamble_start,
                               size_t samples_per_symbol,
                               size_t pre,
                               size_t post,
                               size_t skip_initial,
                               size_t max_repetitions,
                               size_t preamble_repetitions,
                               RadarCirEstimateRef& out,
                               RadarCirScratchRef& scratch)
{
    detail_ref::cir_fail(out, CirStatusRef::InvalidInput);

    if (!rx || n == 0 || preamble_start < 0 || samples_per_symbol == 0 ||
        scratch.sampled_code.empty() || !(scratch.code_energy > 0.f) ||
        !std::isfinite(scratch.code_energy))
        return false;
    if (detail_ref::cir_add_overflow(pre, post))
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
        return detail_ref::cir_fail_status(out, CirStatusRef::CirFailed);

    const float inv_valid = 1.f / static_cast<float>(valid);
    for (size_t m = 0; m < wlen; ++m)
        scratch.avg[m] *= inv_valid;

    // Correlate on the prepared non-zero code chips (active_code_indices/
    // values) instead of scanning all code_len sampled-code entries.  HRP
    // sampled codes hold one non-zero chip per 8 samples (64 active of
    // 1016), so the dropped entries are exact-zero terms whose removal
    // does not change the mathematical value; the double-summation order
    // changes, so results may differ from the dense scan by float rounding
    // only (golden tolerance covers this; see Phase-1 point 3).
    const double energy_d = static_cast<double>(scratch.code_energy);
    size_t peak_tap = 0;
    float peak_abs = -1.f;
    double nrm2 = 0.0;
    const size_t n_active = scratch.active_code_indices.size();
    for (size_t nn = 0; nn < tap_count; ++nn) {
        double acc_re = 0.0;
        double acc_im = 0.0;
        for (size_t j = 0; j < n_active; ++j) {
            const size_t m = scratch.active_code_indices[j];
            const auto c = scratch.active_code_values[j];
            const auto a = scratch.avg[nn + m];
            const double cr = static_cast<double>(c.real());
            const double ci = static_cast<double>(c.imag());
            acc_re += static_cast<double>(a.real()) * cr +
                      static_cast<double>(a.imag()) * ci;
            acc_im += static_cast<double>(a.imag()) * cr -
                      static_cast<double>(a.real()) * ci;
        }
        const std::complex<float> raw(
            static_cast<float>(acc_re / energy_d),
            static_cast<float>(acc_im / energy_d));
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
        return detail_ref::cir_fail_status(out, CirStatusRef::CirFailed);

    const float inv_n = 1.f / (nrm + 1e-12f);
    for (size_t i = 0; i < tap_count; ++i)
        scratch.norm_taps[i] = scratch.raw_taps[i] * inv_n;

    out.status = CirStatusRef::Ok;
    out.tap_count = tap_count;
    out.peak_tap = peak_tap;
    out.peak_abs = peak_abs;
    out.raw_l2_norm = nrm;
    out.valid_repetitions = valid;
    out.first_repetition = skip_initial;
    return true;
}

// HOT PATH: average-mode CIR assembled from the per-repetition kernel.
// Numerically this is mean(rep taps): the coherent sum over the same
// repetitions is only scaled by 1/N at the end, and the correlation already
// runs on the sparse non-zero code chips (active_code_indices/values).
// Compared with estimate_radar_cir_ref the floating-point order differs (N
// per-repetition accumulations instead of one correlated accumulation over
// the coherent mean window); exact equality with estimate_radar_cir_ref is NOT
// required or provided.
//
// Repetition loop is identical to estimate_radar_cir_ref: absolute repetition
// k = skip..skip+count-1, out-of-window / negative repetitions are skipped,
// and out.valid_repetitions counts the ones used.  raw_taps of the sum stay
// in double precision; scratch.raw_taps is not touched until the final
// result (single-rep callers must copy their taps before calling this).
inline bool estimate_radar_cir_ref_from_repetitions(
    const std::complex<float>* rx,
    size_t n,
    int64_t preamble_start,
    size_t samples_per_symbol,
    size_t pre,
    size_t post,
    size_t skip_initial,
    size_t max_repetitions,
    size_t preamble_repetitions,
    RadarCirEstimateRef& out,
    RadarCirScratchRef& scratch)
{
    detail_ref::cir_fail(out, CirStatusRef::InvalidInput);
    if (!rx || n == 0 || preamble_start < 0 || samples_per_symbol == 0 ||
        scratch.sampled_code.empty() || !(scratch.code_energy > 0.f) ||
        !std::isfinite(scratch.code_energy) ||
        scratch.avg_rep_sum.empty() ||
        scratch.active_code_indices.empty() ||
        scratch.active_code_indices.size() !=
            scratch.active_code_values.size() ||
        detail_ref::cir_add_overflow(pre, post))
        return false;

    const size_t tap_count = pre + post;
    if (tap_count == 0 || tap_count > scratch.raw_taps.size() ||
        tap_count > scratch.norm_taps.size())
        return false;
    const size_t code_len = scratch.sampled_code.size();
    if (code_len > std::numeric_limits<size_t>::max() - (tap_count - 1))
        return false;
    const size_t wlen = code_len + tap_count - 1;
    if (wlen > scratch.avg_rep_sum.size() ||
        wlen > scratch.avg_rep_sum.capacity())
        return false;

    const size_t available = preamble_repetitions;
    if (skip_initial >= available)
        return false;
    const size_t count = std::min(max_repetitions, available - skip_initial);

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

    // One fixed accumulation buffer, zeroed per call (fill only wlen).
    std::fill(scratch.avg_rep_sum.begin(),
              scratch.avg_rep_sum.begin() + static_cast<std::ptrdiff_t>(wlen),
              std::complex<double>(0.0, 0.0));

    // Add each repetition's window into the double accumulator.  The
    // ordering matches estimate_radar_cir_ref (repetitions ascending, samples
    // ascending); sparse code chips are applied in the final correlation.
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
        for (size_t m = 0; m < wlen; ++m) {
            const std::complex<float> s = src[m];
            scratch.avg_rep_sum[m] +=
                std::complex<double>(static_cast<double>(s.real()),
                                     static_cast<double>(s.imag()));
        }
        ++valid;
    }
    if (valid == 0)
        return detail_ref::cir_fail_status(out, CirStatusRef::CirFailed);

    const double inv_valid = 1.0 / static_cast<double>(valid);
    const double energy = static_cast<double>(scratch.code_energy);
    size_t peak_tap = 0;
    float peak_abs = -1.f;
    double nrm2 = 0.0;
    for (size_t nn = 0; nn < tap_count; ++nn) {
        double acc_re = 0.0;
        double acc_im = 0.0;
        for (size_t j = 0; j < scratch.active_code_indices.size(); ++j) {
            const size_t m = scratch.active_code_indices[j];
            const auto c = scratch.active_code_values[j];
            const auto a = scratch.avg_rep_sum[nn + m];
            const double cr = static_cast<double>(c.real());
            const double ci = static_cast<double>(c.imag());
            acc_re += (static_cast<double>(a.real()) * cr +
                       static_cast<double>(a.imag()) * ci) * inv_valid;
            acc_im += (static_cast<double>(a.imag()) * cr -
                       static_cast<double>(a.real()) * ci) * inv_valid;
        }
        const std::complex<float> raw(
            static_cast<float>(acc_re / energy),
            static_cast<float>(acc_im / energy));
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
        return detail_ref::cir_fail_status(out, CirStatusRef::CirFailed);

    const float inv_n = 1.f / (nrm + 1e-12f);
    for (size_t i = 0; i < tap_count; ++i)
        scratch.norm_taps[i] = scratch.raw_taps[i] * inv_n;

    out.status = CirStatusRef::Ok;
    out.tap_count = tap_count;
    out.peak_tap = peak_tap;
    out.peak_abs = peak_abs;
    out.raw_l2_norm = nrm;
    out.valid_repetitions = valid;
    out.first_repetition = skip_initial;
    return true;
}

// HOT PATH: estimate one absolute SYNC repetition without averaging.  This
// shares the prepared code and fixed scratch buffers with estimate_radar_cir_ref;
// callers must consume/copy the taps before the next call.  No allocation or
// vector growth occurs here.
inline bool estimate_radar_cir_ref_repetition(
    const std::complex<float>* rx,
    size_t n,
    int64_t preamble_start,
    size_t samples_per_symbol,
    size_t pre,
    size_t post,
    size_t repetition_index,
    size_t preamble_repetitions,
    RadarCirEstimateRef& out,
    RadarCirScratchRef& scratch)
{
    detail_ref::cir_fail(out, CirStatusRef::InvalidInput);
    if (!rx || n == 0 || preamble_start < 0 || samples_per_symbol == 0 ||
        repetition_index >= preamble_repetitions ||
        scratch.sampled_code.empty() || !(scratch.code_energy > 0.f) ||
        !std::isfinite(scratch.code_energy) ||
        scratch.active_code_indices.empty() ||
        scratch.active_code_indices.size() != scratch.active_code_values.size() ||
        detail_ref::cir_add_overflow(pre, post))
        return false;

    const size_t tap_count = pre + post;
    const size_t code_len = scratch.sampled_code.size();
    if (tap_count == 0 || tap_count > scratch.raw_taps.size() ||
        tap_count > scratch.norm_taps.size() ||
        code_len > std::numeric_limits<size_t>::max() - (tap_count - 1))
        return false;
    const size_t wlen = code_len + tap_count - 1;

    int64_t rep64 = 0, period64 = 0, offset = 0, rep_start = 0, pre64 = 0;
    int64_t lo = 0, n64 = 0, wlen64 = 0, last_ok = 0;
    if (!radar_i64_from_size(repetition_index, rep64) ||
        !radar_i64_from_size(samples_per_symbol, period64) ||
        !radar_i64_mul(rep64, period64, offset) ||
        !radar_i64_add(preamble_start, offset, rep_start) ||
        !radar_i64_from_size(pre, pre64) ||
        !radar_i64_sub(rep_start, pre64, lo) ||
        !radar_i64_from_size(n, n64) || !radar_i64_from_size(wlen, wlen64) ||
        !radar_i64_sub(n64, wlen64, last_ok) || lo < 0 || lo > last_ok)
        return detail_ref::cir_fail_status(out, CirStatusRef::CirFailed);

    const std::complex<float>* src = rx + static_cast<size_t>(lo);
    const double energy = static_cast<double>(scratch.code_energy);
    size_t peak_tap = 0;
    float peak_abs = -1.f;
    double nrm2 = 0.0;
    for (size_t nn = 0; nn < tap_count; ++nn) {
        double acc_re = 0.0;
        double acc_im = 0.0;
        for (size_t j = 0; j < scratch.active_code_indices.size(); ++j) {
            const size_t m = scratch.active_code_indices[j];
            const auto c = scratch.active_code_values[j];
            const auto a = src[nn + m];
            // HRP sampled_code is real {-1,0,+1}; keep the general complex
            // form for correctness, but avoid constructing complex<double>
            // temporaries in the 118 x tap_count inner loop.
            const double cr = static_cast<double>(c.real());
            const double ci = static_cast<double>(c.imag());
            acc_re += static_cast<double>(a.real()) * cr +
                      static_cast<double>(a.imag()) * ci;
            acc_im += static_cast<double>(a.imag()) * cr -
                      static_cast<double>(a.real()) * ci;
        }
        const std::complex<float> raw(
            static_cast<float>(acc_re / energy),
            static_cast<float>(acc_im / energy));
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
        return detail_ref::cir_fail_status(out, CirStatusRef::CirFailed);
    const float inv_n = 1.f / (nrm + 1e-12f);
    for (size_t i = 0; i < tap_count; ++i)
        scratch.norm_taps[i] = scratch.raw_taps[i] * inv_n;

    out.status = CirStatusRef::Ok;
    out.tap_count = tap_count;
    out.peak_tap = peak_tap;
    out.peak_abs = peak_abs;
    out.raw_l2_norm = nrm;
    out.valid_repetitions = 1;
    out.first_repetition = repetition_index;
    return true;
}

} // namespace scalar_ref
} // namespace uwb
} // namespace gr
