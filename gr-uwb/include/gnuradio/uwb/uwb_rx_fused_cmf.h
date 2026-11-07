/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Experimental payload CMF + PN-despreading fusion.
 *
 * The legacy payload metric first computes one complex CMF output on every
 * chip of the payload field and then correlates the eight chips of each BPM
 * burst with the BPRF spreading sequence:
 *
 *   y[k]        = sum_q h[q] * rx[win(k) + q]          (h = conj(CIR taps))
 *   metric(s,p) = sum_c a[s,c] * Re(rot * y[base(s)+p+c])
 *
 * where p selects one of the four BPM candidates:
 *
 *   p = 0  -> hop 0, BPM half 0   (chip offset  0)
 *   p = 1  -> hop 0, BPM half 1   (chip offset 32)
 *   p = 2  -> hop 1, BPM half 0   (chip offset  8)
 *   p = 3  -> hop 1, BPM half 1   (chip offset 40)
 *
 * Both hops and both BPM halves use the SAME eight-chip spreading segment
 * a[s, 0..7] (bprf_demod() uses `sp[c]` for all four candidates).  Exchanging
 * the two sums gives a finite template
 *
 *   t[m] = sum_{q + 2c = m} h[q] * a[s,c],   m = 0 .. tap_count + 14
 *   M(s,p) = sum_m t[m] * rx[base(s) + p_offset + m]
 *
 * so one 8-chip template (52 complex values for the production 38-tap CMF)
 * replaces the per-chip FIR plus the eight-chip despread.  A common positive
 * normalization scale (the legacy global max|soft|) cancels in the hard BPM
 * decision, so this kernel deliberately returns unnormalized metrics; the
 * caller may apply any positive scale.  Phase rotation and NS-SFD polarity are
 * applied here because the sign of g1 depends on them.
 *
 * This header is pure math: no GNU Radio block, no allocation inside the
 * block loop (Scratch owns the template buffer), and no fake/known packet
 * metadata.  Fractional chip grids, RAKE and bypass are rejected by the core
 * integration and keep the legacy path.
 */

#pragma once

#include <gnuradio/uwb/uwb_cir_fir_simd.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace gr {
namespace uwb {
namespace demod {
namespace rxfusion {

// BPRF 6.81 Mb/s payload geometry (same constants as bprf_demod()).
inline constexpr size_t kChipsPerBurst = 8;
inline constexpr size_t kChipsPerSymbol = 64;
inline constexpr size_t kBpmHalfOffset = 32; // cps / 2
inline constexpr size_t kCandidateCount = 4;
inline constexpr size_t kSamplesPerChip = 2;
inline constexpr size_t kMaxTapCount = 64;
inline constexpr size_t kBlockSymbols = 64;

// Candidate order is fixed and is also the layout of Metrics below:
//   0: hop 0, BPM half 0
//   1: hop 0, BPM half 1
//   2: hop 1, BPM half 0
//   3: hop 1, BPM half 1
inline constexpr size_t kCandidateChipOffset[kCandidateCount] = { 0, 32, 8, 40 };

struct Metrics {
    size_t symbols = 0;
    // Raw complex correlation M(s,p), before rotation/polarity.  Kept for
    // independent numerical verification and diagnostics.
    std::vector<std::complex<float>> complex_value;
    // signed_value = polarity * Re(rot * complex_value).  This is the exact
    // quantity compared by bprf_demod() up to a common positive scale.
    std::vector<float> signed_value;

    void reset(size_t count)
    {
        symbols = count;
        complex_value.resize(kCandidateCount * count);
        signed_value.resize(kCandidateCount * count);
    }
};

struct Timings {
    uint64_t template_update_us = 0;
    uint64_t correlate_us = 0;
};

struct Scratch {
    // One block of templates: kBlockSymbols * template_len complex values.
    // The block loop keeps this buffer L1/L2-resident for long PSDU lengths.
    std::vector<std::complex<float>> templates;
    size_t template_len = 0;

    void ensure(size_t block_symbols, size_t len)
    {
        if (template_len != len || templates.size() < block_symbols * len) {
            template_len = len;
            templates.resize(block_symbols * len);
        }
    }
};

// Template construction for one symbol: t[q + 2c] += a[c] * h[q].
// `pn` points at the eight spreading chips of this symbol.  h is the
// pre-conjugated CMF vector (the same `h_conj` used by stage_cir_softchips).
inline void build_template(std::complex<float>* out,
                           const std::complex<float>* h,
                           size_t tap_count,
                           const int8_t* pn)
{
    const size_t len = tap_count +
                       kSamplesPerChip * (kChipsPerBurst - 1);
    std::fill(out, out + len, std::complex<float>(0.0f, 0.0f));
#if UWB_CIR_FIR_HAVE_AVX2
    const float* hp = reinterpret_cast<const float*>(h);
    const size_t floats = 2 * tap_count;
    for (size_t c = 0; c < kChipsPerBurst; ++c) {
        const float sign = (pn[c] < 0) ? -1.0f : 1.0f;
        float* dp = reinterpret_cast<float*>(
            out + kSamplesPerChip * c);
        const __m256 vsign = _mm256_set1_ps(sign);
        size_t k = 0;
        for (; k + 8 <= floats; k += 8) {
            const __m256 hv = _mm256_loadu_ps(hp + k);
            const __m256 dv = _mm256_loadu_ps(dp + k);
            _mm256_storeu_ps(dp + k, _mm256_fmadd_ps(vsign, hv, dv));
        }
        for (; k < floats; ++k)
            dp[k] += sign * hp[k];
    }
#else
    for (size_t c = 0; c < kChipsPerBurst; ++c) {
        const float sign = (pn[c] < 0) ? -1.0f : 1.0f;
        std::complex<float>* dst = out + kSamplesPerChip * c;
        for (size_t q = 0; q < tap_count; ++q)
            dst[q] += sign * h[q];
    }
#endif
}

// Fixed 52-tap complex dot for the production 38-tap CMF (38 + 7*2).
// The template starts at an arbitrary (possibly unaligned) address.
inline std::complex<float>
dot_fixed52(const std::complex<float>* rx_win,
            const std::complex<float>* taps)
{
#if UWB_CIR_FIR_HAVE_AVX2
    const float* xp = reinterpret_cast<const float*>(rx_win);
    const float* hp = reinterpret_cast<const float*>(taps);
    __m256 acc0 = cir_fir::cir_fir_avx2_group4(xp, hp, 0);
    __m256 acc1 = cir_fir::cir_fir_avx2_group4(xp, hp, 8);
    acc0 = _mm256_add_ps(acc0, cir_fir::cir_fir_avx2_group4(xp, hp, 16));
    acc1 = _mm256_add_ps(acc1, cir_fir::cir_fir_avx2_group4(xp, hp, 24));
    acc0 = _mm256_add_ps(acc0, cir_fir::cir_fir_avx2_group4(xp, hp, 32));
    acc1 = _mm256_add_ps(acc1, cir_fir::cir_fir_avx2_group4(xp, hp, 40));
    acc0 = _mm256_add_ps(acc0, cir_fir::cir_fir_avx2_group4(xp, hp, 48));
    acc1 = _mm256_add_ps(acc1, cir_fir::cir_fir_avx2_group4(xp, hp, 56));
    acc0 = _mm256_add_ps(acc0, cir_fir::cir_fir_avx2_group4(xp, hp, 64));
    acc1 = _mm256_add_ps(acc1, cir_fir::cir_fir_avx2_group4(xp, hp, 72));
    acc0 = _mm256_add_ps(acc0, cir_fir::cir_fir_avx2_group4(xp, hp, 80));
    acc1 = _mm256_add_ps(acc1, cir_fir::cir_fir_avx2_group4(xp, hp, 88));
    acc0 = _mm256_add_ps(acc0, cir_fir::cir_fir_avx2_group4(xp, hp, 96));
    const __m256 acc = _mm256_add_ps(acc0, acc1);
    const __m128 lo = _mm256_castps256_ps128(acc);
    const __m128 hi = _mm256_extractf128_ps(acc, 1);
    const __m128 s = _mm_add_ps(lo, hi);
    const __m128 s2 = _mm_add_ps(s, _mm_movehl_ps(s, s));
    return std::complex<float>(_mm_cvtss_f32(s2),
                               _mm_cvtss_f32(_mm_shuffle_ps(s2, s2, 1)));
#else
    return cir_fir::dot_multi_acc8(rx_win, taps, 52);
#endif
}

inline std::complex<float>
dot_template(const std::complex<float>* rx_win,
             const std::complex<float>* taps,
             size_t len)
{
    if (len == 52)
        return dot_fixed52(rx_win, taps);
    return cir_fir::dot_multi_acc8(rx_win, taps, len);
}

// Correlate all payload symbols against the fused templates.
//
// Inputs:
//   rx             CFO-compensated IQ, 0-based window coordinates
//   fir_taps       h[q] = conj(normalized CIR values), tap_count entries
//   chip_start     chip-stream origin: sample of chip 0 (preamble_start +
//                  tap_count - cir_pre_samples - 1)
//   payload_start  chip index of the first payload symbol
//   pn             BPRF spreading stream at scrambler offset 1344, indexed
//                  pn[s*8 + c] for c = 0..7
//   symbols        number of BPRF payload symbols (nsym)
//   polarity       NS-SFD polarity (+1/-1)
//   rot            frozen prefix phase rotation (exp(-j*arg(gain)))
//
// Returns false, leaving Metrics unspecified, if any candidate window is not
// fully readable.  The caller retries the legacy full stream once; the fused
// path must not silently truncate a payload or zero-pad its tail.
inline bool correlate_payload(const std::complex<float>* rx,
                              size_t n,
                              const std::complex<float>* fir_taps,
                              size_t tap_count,
                              int64_t chip_start,
                              size_t samples_per_chip,
                              int64_t payload_start,
                              const int8_t* pn,
                              size_t symbols,
                              int8_t polarity,
                              std::complex<float> rot,
                              Metrics& out,
                              Scratch& scratch,
                              Timings* timings = nullptr)
{
    if (!rx || !fir_taps || !pn || symbols == 0 || tap_count == 0 ||
        tap_count > kMaxTapCount)
        return false;
    if (samples_per_chip != kSamplesPerChip)
        return false;
    if (chip_start < 0 || payload_start < 0)
        return false;
    if (polarity != 1 && polarity != -1)
        return false;

    const size_t len = tap_count +
                       samples_per_chip * (kChipsPerBurst - 1);
    // Window for symbol 0, hop 0, half 0, chip c=0.  The last candidate is
    // symbol symbols-1, hop 1, half 1, chip c=7.
    const int64_t base0 = chip_start +
                          payload_start * static_cast<int64_t>(samples_per_chip) -
                          static_cast<int64_t>(tap_count) + 1;
    const int64_t last_chip = static_cast<int64_t>(symbols - 1) *
                                  static_cast<int64_t>(kChipsPerSymbol) +
                              static_cast<int64_t>(kBpmHalfOffset) +
                              static_cast<int64_t>(kChipsPerBurst) - 1;
    const int64_t last = base0 +
                         last_chip * static_cast<int64_t>(samples_per_chip) +
                         static_cast<int64_t>(len) - 1;
    if (base0 < 0 || last >= static_cast<int64_t>(n))
        return false;
    // Cheap overflow guard before any multiplication below.
    if (symbols > (static_cast<size_t>(1) << 20))
        return false;

    out.reset(symbols);
    const size_t block_symbols = std::min(kBlockSymbols, symbols);
    scratch.ensure(block_symbols, len);

    auto t_template = std::chrono::steady_clock::duration::zero();
    auto t_correlate = std::chrono::steady_clock::duration::zero();

    for (size_t start = 0; start < symbols; start += kBlockSymbols) {
        const size_t count = std::min(kBlockSymbols, symbols - start);
        auto t0 = std::chrono::steady_clock::now();
        for (size_t i = 0; i < count; ++i) {
            build_template(scratch.templates.data() + i * len,
                           fir_taps, tap_count, pn + (start + i) * kChipsPerBurst);
        }
        auto t1 = std::chrono::steady_clock::now();
        t_template += t1 - t0;

        for (size_t i = 0; i < count; ++i) {
            const size_t s = start + i;
            const std::complex<float>* taps =
                scratch.templates.data() + i * len;
            const int64_t win0 = base0 +
                                 static_cast<int64_t>(s) *
                                     static_cast<int64_t>(kChipsPerSymbol) *
                                     static_cast<int64_t>(samples_per_chip);
            for (size_t p = 0; p < kCandidateCount; ++p) {
                const int64_t off = static_cast<int64_t>(
                    kCandidateChipOffset[p]) *
                    static_cast<int64_t>(samples_per_chip);
                const std::complex<float> m =
                    dot_template(rx + win0 + off, taps, len);
                out.complex_value[kCandidateCount * s + p] = m;
                const float re = rot.real() * m.real() -
                                 rot.imag() * m.imag();
                out.signed_value[kCandidateCount * s + p] =
                    (polarity < 0) ? -re : re;
            }
        }
        auto t2 = std::chrono::steady_clock::now();
        t_correlate += t2 - t1;
    }

    if (timings) {
        timings->template_update_us = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::microseconds>(t_template)
                .count());
        timings->correlate_us = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::microseconds>(t_correlate)
                .count());
    }
    return true;
}

// Reproduce bprf_demod()'s hard decision exactly from the four signed
// metrics.  Hop 0 wins a tie against hop 1; BPM half 0 wins a tie against
// half 1.  g1 is the sign of the selected metric (negative -> 1).
inline void decide_bpm(const Metrics& metrics,
                       std::vector<int8_t>& g0,
                       std::vector<int8_t>& g1)
{
    g0.resize(metrics.symbols);
    g1.resize(metrics.symbols);
    for (size_t s = 0; s < metrics.symbols; ++s) {
        const float* m = &metrics.signed_value[kCandidateCount * s];
        // Half 0: candidates 0 (hop 0) and 2 (hop 1).
        float b0 = std::abs(m[0]);
        float best0 = m[0];
        if (std::abs(m[2]) > b0) {
            b0 = std::abs(m[2]);
            best0 = m[2];
        }
        // Half 1: candidates 1 (hop 0) and 3 (hop 1).
        float b1 = std::abs(m[1]);
        float best1 = m[1];
        if (std::abs(m[3]) > b1) {
            b1 = std::abs(m[3]);
            best1 = m[3];
        }
        float best;
        if (b0 >= b1) {
            g0[s] = 0;
            best = best0;
        } else {
            g0[s] = 1;
            best = best1;
        }
        g1[s] = (best < 0.0f) ? 1 : 0;
    }
}

} // namespace rxfusion
} // namespace demod
} // namespace uwb
} // namespace gr
