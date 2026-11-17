/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Experimental CMF + PN-despreading fusion for the HRP payload and PHR
 * fields.
 *
 * The legacy metric first computes one complex CMF output on every chip of
 * the field and then correlates the chips of each BPM burst with the BPRF
 * spreading sequence:
 *
 *   y[k]        = sum_q h[q] * rx[win(k) + q]          (h = conj(CIR taps))
 *   metric(s,p) = sum_c a[s,c] * Re(rot * y[base(s)+p+c])
 *
 * where p selects one of the four BPM candidates (two hops x two BPM halves):
 *
 *   p = 0  -> hop 0, BPM half 0
 *   p = 1  -> hop 0, BPM half 1
 *   p = 2  -> hop 1, BPM half 0
 *   p = 3  -> hop 1, BPM half 1
 *
 * Both hops and both BPM halves use the SAME spreading segment
 * a[s, 0..burst-1] (bprf_demod() uses `sp[c]` for all four candidates).
 * Exchanging the two sums gives a finite per-symbol template
 *
 *   t[m] = sum_{q + spc*c = m} h[q] * a[s,c],   m = 0 .. len-1
 *   M(s,p) = sum_m t[m] * rx[base(s) + p_offset + m]
 *
 * with len = tap_count + spc * (chips_per_burst - 1).  One template replaces
 * the per-chip FIR plus the burst despread.  Two geometries are provided:
 *
 *   payload   8 chips/burst, 64 chips/symbol, PN scrambler offset 1344,
 *             candidates at chip offsets {0, 32, 8, 40}; 38 taps give the
 *             52-point template of the verified payload fast path.
 *   PHR      64 chips/burst, 512 chips/symbol, PN scrambler offset 0,
 *             candidates at chip offsets {0, 256, 64, 320}; 38 taps give a
 *             164-point template.
 *
 * correlate_payload() and correlate_phr() are thin wrappers over the generic
 * correlate_field().  The payload wrapper keeps the original 52-point fixed
 * kernel and arithmetic order bit-for-bit; other template lengths use a
 * bounds-safe AVX2/FMA dot with a scalar tail.  A common positive
 * normalization scale (the legacy global max|soft|) cancels in the hard BPM
 * decision, so this kernel deliberately returns unnormalized metrics; the
 * caller may apply any positive scale.  Phase rotation and NS-SFD polarity
 * are applied here because the sign of g1 depends on them.
 *
 * This header is pure math: no GNU Radio block, no allocation inside the
 * symbol loop (Scratch owns the template buffer), and no fake/known packet
 * metadata.  Fractional chip grids, RAKE and bypass are rejected by the core
 * integration and keep the legacy path.  Any candidate window that is not
 * fully readable is rejected; the caller retries the legacy stream once and
 * the fused path must not silently truncate a field or zero-pad its tail.
 */

#pragma once

#include <gnuradio/uwb/uwb_cir_fir_simd.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <limits>
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

// PHR geometry (850 kb/s).  bprf_demod() again reuses one 64-chip spreading
// segment for both hops and both BPM halves, so the candidate order stays
// (hop 0, half 0), (hop 0, half 1), (hop 1, half 0), (hop 1, half 1) with
// offsets 0, cps/2 = 256, cpb = 64 and cps/2 + cpb = 320 chips.
inline constexpr size_t kPhrChipsPerBurst = 64;
inline constexpr size_t kPhrChipsPerSymbol = 512;
inline constexpr size_t kPhrSymbols = 21;
inline constexpr size_t kPhrScramOffset = 0;
inline constexpr size_t kPhrCandidateChipOffset[kCandidateCount] = { 0, 256, 64, 320 };

// Field geometry consumed by correlate_field().  `candidate_chip_offset`
// points at kCandidateCount entries in the fixed candidate order above.
struct Geometry {
    size_t chips_per_burst;
    size_t chips_per_symbol;
    size_t candidate_count;
    const size_t* candidate_chip_offset;
};

inline constexpr Geometry kPayloadGeometry = { kChipsPerBurst,
                                               kChipsPerSymbol,
                                               kCandidateCount,
                                               kCandidateChipOffset };
inline constexpr Geometry kPhrGeometry = { kPhrChipsPerBurst,
                                           kPhrChipsPerSymbol,
                                           kCandidateCount,
                                           kPhrCandidateChipOffset };

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

// Length in complex samples of one fused template:
//   tap_count + samples_per_chip * (chips_per_burst - 1).
inline size_t template_length(size_t tap_count,
                              size_t samples_per_chip,
                              size_t chips_per_burst)
{
    return tap_count + samples_per_chip * (chips_per_burst - 1);
}

// Template construction for one symbol:
//   t[q + samples_per_chip * c] += pn[c] * h[q].
// `pn` points at chips_per_burst spreading chips.  h is the pre-conjugated
// CMF vector (the same `h_conj` used by stage_cir_softchips).
inline void build_template_geometry(std::complex<float>* out,
                                    const std::complex<float>* h,
                                    size_t tap_count,
                                    const int8_t* pn,
                                    size_t chips_per_burst,
                                    size_t samples_per_chip)
{
    const size_t len =
        template_length(tap_count, samples_per_chip, chips_per_burst);
    std::fill(out, out + len, std::complex<float>(0.0f, 0.0f));
#if UWB_CIR_FIR_HAVE_AVX2
    const float* hp = reinterpret_cast<const float*>(h);
    const size_t floats = 2 * tap_count;
    for (size_t c = 0; c < chips_per_burst; ++c) {
        const float sign = (pn[c] < 0) ? -1.0f : 1.0f;
        float* dp = reinterpret_cast<float*>(out + samples_per_chip * c);
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
    for (size_t c = 0; c < chips_per_burst; ++c) {
        const float sign = (pn[c] < 0) ? -1.0f : 1.0f;
        std::complex<float>* dst = out + samples_per_chip * c;
        for (size_t q = 0; q < tap_count; ++q)
            dst[q] += sign * h[q];
    }
#endif
}

// Payload wrapper: eight spreading chips per burst, spc = 2.
inline void build_template(std::complex<float>* out,
                           const std::complex<float>* h,
                           size_t tap_count,
                           const int8_t* pn)
{
    build_template_geometry(out, h, tap_count, pn, kChipsPerBurst,
                            kSamplesPerChip);
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

// Generic complex dot for a template of `len` complex values.  Full groups of
// four complex samples use the AVX2/FMA kernel with two accumulators; the
// remaining 0..3 complex samples use a scalar tail.  Only indices below `len`
// are ever loaded, so a template tail never overreads the IQ window.
inline std::complex<float>
dot_generic(const std::complex<float>* rx_win,
            const std::complex<float>* taps,
            size_t len)
{
#if UWB_CIR_FIR_HAVE_AVX2
    const float* xp = reinterpret_cast<const float*>(rx_win);
    const float* hp = reinterpret_cast<const float*>(taps);
    const size_t groups = len / 4;
    __m256 acc0 = _mm256_setzero_ps();
    __m256 acc1 = _mm256_setzero_ps();
    size_t g = 0;
    // Two groups (8 complex) per iteration; both offsets stay below `groups`.
    for (; g + 2 <= groups; g += 2) {
        acc0 = _mm256_add_ps(
            acc0, cir_fir::cir_fir_avx2_group4(
                      xp, hp, static_cast<int>(8 * g)));
        acc1 = _mm256_add_ps(
            acc1, cir_fir::cir_fir_avx2_group4(
                      xp, hp, static_cast<int>(8 * (g + 1))));
    }
    for (; g < groups; ++g)
        acc0 = _mm256_add_ps(
            acc0, cir_fir::cir_fir_avx2_group4(
                      xp, hp, static_cast<int>(8 * g)));

    const __m256 acc = _mm256_add_ps(acc0, acc1);
    const __m128 lo = _mm256_castps256_ps128(acc);
    const __m128 hi = _mm256_extractf128_ps(acc, 1);
    const __m128 s = _mm_add_ps(lo, hi);
    const __m128 s2 = _mm_add_ps(s, _mm_movehl_ps(s, s));
    float re = _mm_cvtss_f32(s2);
    float im = _mm_cvtss_f32(_mm_shuffle_ps(s2, s2, 1));
    // Scalar tail: no speculative load past len.
    for (size_t i = 4 * groups; i < len; ++i) {
        re += rx_win[i].real() * taps[i].real() -
              rx_win[i].imag() * taps[i].imag();
        im += rx_win[i].real() * taps[i].imag() +
              rx_win[i].imag() * taps[i].real();
    }
    return std::complex<float>(re, im);
#else
    return cir_fir::dot_multi_acc8(rx_win, taps, len);
#endif
}

inline std::complex<float>
dot_template(const std::complex<float>* rx_win,
             const std::complex<float>* taps,
             size_t len)
{
    if (len == 52)
        return dot_fixed52(rx_win, taps);
    return dot_generic(rx_win, taps, len);
}

// Correlate all `symbols` symbols of one field against the fused templates.
//
// Inputs:
//   rx                 CFO-compensated IQ, 0-based window coordinates
//   n                  number of readable samples in rx
//   fir_taps           h[q] = conj(normalized CIR values), tap_count entries
//   chip_start         chip-stream origin: sample of chip 0 (preamble_start +
//                      tap_count - cir_pre_samples - 1)
//   samples_per_chip   must be kSamplesPerChip (integer grid only)
//   field_start        chip index of the first symbol of this field
//   pn                 spreading stream, indexed pn[s*chips_per_burst + c]
//   symbols            number of symbols to correlate
//   geometry           burst/symbol lengths and the four candidate offsets
//   polarity           NS-SFD polarity (+1/-1)
//   rot                frozen prefix phase rotation (exp(-j*arg(gain)))
//
// Returns false, leaving Metrics unspecified, if the geometry is unsupported
// or any candidate window is not fully readable.  The caller retries the
// legacy full stream once; the fused path must not silently truncate a field
// or zero-pad its tail.
inline bool correlate_field(const std::complex<float>* rx,
                            size_t n,
                            const std::complex<float>* fir_taps,
                            size_t tap_count,
                            int64_t chip_start,
                            size_t samples_per_chip,
                            int64_t field_start,
                            const int8_t* pn,
                            size_t symbols,
                            const Geometry& geometry,
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
    if (chip_start < 0 || field_start < 0)
        return false;
    if (polarity != 1 && polarity != -1)
        return false;
    if (geometry.candidate_count != kCandidateCount ||
        !geometry.candidate_chip_offset)
        return false;
    if (geometry.chips_per_burst == 0 || geometry.chips_per_symbol == 0)
        return false;
    // Every candidate burst must lie inside its symbol (offset + chips_per_burst <= chips_per_symbol).
    // Compare with subtraction so the addition cannot wrap.
    if (geometry.chips_per_burst > geometry.chips_per_symbol)
        return false;
    const size_t max_allowed_offset =
        geometry.chips_per_symbol - geometry.chips_per_burst;
    size_t max_candidate_offset = 0;
    for (size_t p = 0; p < kCandidateCount; ++p) {
        const size_t off = geometry.candidate_chip_offset[p];
        if (off > max_allowed_offset)
            return false;
        if (off > max_candidate_offset)
            max_candidate_offset = off;
    }
    // Bound the template length without overflowing size_t.  The support
    // limit is the 64-chip PHR burst: 38 taps give 164 complex samples and
    // the absolute maximum is 64 + 2*63 = 190.
    const size_t max_len =
        kMaxTapCount + kSamplesPerChip * (kPhrChipsPerBurst - 1);
    if (geometry.chips_per_burst - 1 >
        (max_len - tap_count) / samples_per_chip)
        return false;
    if (geometry.chips_per_symbol > (static_cast<size_t>(1) << 30))
        return false;
    if (symbols > (static_cast<size_t>(1) << 20))
        return false;
    // The PN contract is a +/-1 sequence.  A zero would silently be treated
    // as +1 by build_template_geometry(), so reject it before producing
    // metrics.
    for (size_t i = 0; i < symbols * geometry.chips_per_burst; ++i) {
        if (pn[i] != 1 && pn[i] != -1)
            return false;
    }

    const size_t len =
        template_length(tap_count, samples_per_chip, geometry.chips_per_burst);
    const int64_t spc = static_cast<int64_t>(samples_per_chip);
    // Guard the sample-coordinate product before it enters int64.  Real
    // fields use small chip indices; this only rejects absurd offsets.
    if (field_start >
        (std::numeric_limits<int64_t>::max() - chip_start) / spc)
        return false;
    // Window for symbol 0, hop 0, half 0, chip c=0.  The furthest candidate is
    // symbol symbols-1 with the maximum geometry offset across all candidates;
    // its template then spans len samples.
    const int64_t base0 = chip_start + field_start * spc -
                          static_cast<int64_t>(tap_count) + 1;
    if (base0 < 0)
        return false;
    const int64_t last_start_chip =
        static_cast<int64_t>(symbols - 1) *
            static_cast<int64_t>(geometry.chips_per_symbol) +
        static_cast<int64_t>(max_candidate_offset);
    // Compare as unsigned int64/u64 indices; base0 is non-negative and the
    // geometry bounds above keep this addition far from wrapping.
    const uint64_t add = static_cast<uint64_t>(last_start_chip) *
                             static_cast<uint64_t>(spc) +
                         static_cast<uint64_t>(len) - 1;
    const uint64_t last = static_cast<uint64_t>(base0) + add;
    if (last < static_cast<uint64_t>(base0) ||
        last >= static_cast<uint64_t>(n))
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
            build_template_geometry(
                scratch.templates.data() + i * len, fir_taps, tap_count,
                pn + (start + i) * geometry.chips_per_burst,
                geometry.chips_per_burst, samples_per_chip);
        }
        auto t1 = std::chrono::steady_clock::now();
        t_template += t1 - t0;

        for (size_t i = 0; i < count; ++i) {
            const size_t s = start + i;
            const std::complex<float>* taps =
                scratch.templates.data() + i * len;
            const int64_t win0 =
                base0 + static_cast<int64_t>(s) *
                            static_cast<int64_t>(geometry.chips_per_symbol) *
                            spc;
            for (size_t p = 0; p < kCandidateCount; ++p) {
                const int64_t off =
                    static_cast<int64_t>(
                        geometry.candidate_chip_offset[p]) *
                    spc;
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

// Payload wrapper.  Signature and arithmetic are unchanged from the verified
// payload fast path: 8 chips/burst, 64 chips/symbol, PN scrambler offset
// 1344, four candidates {0, 32, 8, 40} and the 52-point fixed dot for 38
// taps.
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
    return correlate_field(rx, n, fir_taps, tap_count, chip_start,
                           samples_per_chip, payload_start, pn, symbols,
                           kPayloadGeometry, polarity, rot, out, scratch,
                           timings);
}

// PHR wrapper: 21 symbols of 64 chips/burst and 512 chips/symbol starting at
// `phr_start` (sfd_end_chip + 1), PN scrambler offset 0.
inline bool correlate_phr(const std::complex<float>* rx,
                          size_t n,
                          const std::complex<float>* fir_taps,
                          size_t tap_count,
                          int64_t chip_start,
                          size_t samples_per_chip,
                          int64_t phr_start,
                          const int8_t* pn,
                          int8_t polarity,
                          std::complex<float> rot,
                          Metrics& out,
                          Scratch& scratch,
                          Timings* timings = nullptr)
{
    return correlate_field(rx, n, fir_taps, tap_count, chip_start,
                           samples_per_chip, phr_start, pn, kPhrSymbols,
                           kPhrGeometry, polarity, rot, out, scratch, timings);
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

// Soft-output twin of decide_bpm().  The hard decisions written to g0/g1 are
// bit-identical to decide_bpm() (same tie rules: hop 0 wins a hop tie, BPM
// half 0 wins a half tie, zero is non-negative), and the LLRs reproduce
// bprf_demod_soft()'s convention exactly on the same four signed metrics:
//
//   llr0[s] = |half0| - |half1|   > 0 favours g0 = 0
//   llr1[s] = selected half metric > 0 favours g1 = 0
//
// Because the fused kernel returns unnormalized metrics (the legacy global
// max|soft| scale cancels in the hard decision), the LLRs are on the same
// arbitrary positive scale as the metrics.  A soft-decision Viterbi only
// needs a consistent scale per packet, so no renormalization is applied here.
inline void decide_bpm_soft(const Metrics& metrics,
                            std::vector<int8_t>& g0, std::vector<int8_t>& g1,
                            std::vector<float>& llr0,
                            std::vector<float>& llr1)
{
    g0.resize(metrics.symbols);
    g1.resize(metrics.symbols);
    llr0.resize(metrics.symbols);
    llr1.resize(metrics.symbols);
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
        llr0[s] = b0 - b1;
        llr1[s] = best;
    }
}

} // namespace rxfusion
} // namespace demod
} // namespace uwb
} // namespace gr
