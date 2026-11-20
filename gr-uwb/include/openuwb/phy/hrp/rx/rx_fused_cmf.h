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

#include <openuwb/phy/hrp/rx/rx_bpm.h>

#include <openuwb/dsp/cir_fir_simd.h>

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

// Exact-CFO fused payload kernel (correlate_payload_fused_cfo below).
//
// The production CMF is 38 taps, which gives the verified 52-point payload
// template.  Half of an 8-chip burst (4 chips, 2 samples/chip) spans
// 2*(4-1) = 6 extra samples, so one half template covers 38 + 6 = 44 complex
// values.  Only 2^3 = 8 base patterns exist because the first chip of each
// half is factored out as a sign: pattern bit (c-1) flips chip c relative to
// chip 0.  The upper half is the same 8 patterns multiplied by
// exp(-j*w*8) and shifted by 8 samples.
inline constexpr size_t kFusedFastTaps = 38;
inline constexpr size_t kFusedFastLen = kFusedFastTaps + 14; // 52
inline constexpr size_t kFusedHalfTemplates = 8;
inline constexpr size_t kFusedHalfSpan =
    kFusedFastTaps + 2 * (kChipsPerBurst / 2 - 1); // 44
// Phase re-anchor interval for the exact-CFO symbol ramp (symbols).
inline constexpr size_t kFusedPhaseAnchor = 32;

// Known-hop SFO path: half-template values stored per fractional phase bin
// (8 lo patterns followed by 8 hi patterns).
inline constexpr size_t kKhLutPerBin = 2 * kFusedHalfTemplates * kFusedHalfSpan;
inline constexpr size_t kKhMaxPhaseBins = 64;

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

    // `want_complex == false` keeps complex_value empty (capacity retained) so
    // the production path never touches the 16 bytes/symbol diagnostic buffer.
    // signed_value is always sized and written in full.
    void reset(size_t count, bool want_complex = true)
    {
        symbols = count;
        if (want_complex)
            complex_value.resize(kCandidateCount * count);
        else
            complex_value.clear();
        signed_value.resize(kCandidateCount * count);
    }
};

// Known-hop 2-candidate metrics (payload/D-PHY hop and BPM half are known to
// be `hop = (pn[8*s] < 0) ? 1 : 0`, so only two of the four candidates above
// are evaluated).  `m0` is BPM half 0 of the known hop and `m1` half 1.
//
// `z0/z1` are the raw complex correlations on the same phase basis as
// `Metrics::complex_value`: the exact-CFO fusion already removed the modelled
// CFO (window ramp + per-chip ramp), but neither `rot`, `polarity` nor the
// DD-PLL tracking rotation is applied.  Wave 2 can therefore feed them
// directly to the PHR pilot refinement and biorthogonal decoder.
//
// `m0/m1` are the final signed metrics and match the corresponding
// `signed_value` entries of the 4-candidate path:
//   m_half = polarity * Re(rot_cur * z_half)
// with rot_cur = rot when the DD-PLL is disabled; with the loop enabled
// rot_cur is the per-symbol tracked rotation (rot times the loop phase).
// `z0/z1` are only written when `write_complex == true` (always for the PHR
// known-hop helper).
struct Metrics2 {
    size_t symbols = 0;
    std::vector<float> m0, m1;
    std::vector<std::complex<float>> z0, z1;

    // DD-PLL diagnostics (all zero when the loop is disabled):
    //   pll_freq_rad  final per-symbol frequency estimate (rad/symbol)
    //   pll_theta     accumulated phase applied to rot_cur (rad)
    //   pll_updates   number of symbols whose phase error was accepted
    float pll_freq_rad = 0.0f;
    float pll_theta = 0.0f;
    size_t pll_updates = 0;

    void reset(size_t count, bool want_complex = false)
    {
        symbols = count;
        m0.resize(count);
        m1.resize(count);
        if (want_complex) {
            z0.resize(count);
            z1.resize(count);
        } else {
            z0.clear();
            z1.clear();
        }
        pll_freq_rad = 0.0f;
        pll_theta = 0.0f;
        pll_updates = 0;
    }
};

// Symbol-level decision-directed PLL for the known-hop payload kernel.
//
// The payload symbol correlation is already complex, so a residual CFO can be
// tracked without touching the chips: rotate both candidate values by the
// current tracking phase, pick the stronger half, and drive a second-order PI
// loop with the normalized BPSK phase error
//   err = sign(Re(z_win)) * Im(z_win) / (|z_win| + eps).
// `min_mag_ratio` gates the update against deep fades; `max_freq_step` clamps
// the per-symbol frequency state to roughly +/- 60 kHz at the production 128
// samples/symbol (2*pi*62170/998.4e6 = 0.05 rad/symbol).
struct DdPllConfig {
    bool enable = false;
    float alpha = 0.08f;         // proportional loop gain (rad)
    float beta = 0.002f;         // residual CFO integrator gain (rad/symbol)
    float max_freq_step = 0.05f; // per-symbol frequency clamp (rad/symbol)
    float min_mag_ratio = 0.25f; // skip updates below this fraction of mag EMA
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

    // Exact-CFO payload kernel state.  `fused_lut_lo` holds the 8 four-chip
    // half templates (kFusedHalfTemplates * kFusedHalfSpan) and
    // `fused_lut_hi` the same patterns advanced by 8 samples and multiplied by
    // exp(-j*w*8).  `fused_h_w` is h[q]*exp(-j*w*q) and `fused_tpl` is the one
    // per-symbol template (52 for 38 taps, up to 78 for the generic path).
    // All are grown by ensure_fused_cfo(), never inside the symbol loop.
    std::vector<std::complex<float>> fused_lut_lo;
    std::vector<std::complex<float>> fused_lut_hi;
    std::vector<std::complex<float>> fused_h_w;
    std::vector<std::complex<float>> fused_tpl;

    // Known-hop kernel state.  `kh_lut` holds 2*8 half templates (lo block
    // followed by hi block, kKhLutPerBin values) for each fractional SFO phase
    // bin; `kh_h_w` holds the per-bin h[q]*exp(-j*w*q) used to build the
    // generic (non-38-tap) fractional template on the fly.  The integer grid
    // path keeps using fused_lut_lo/fused_lut_hi and never touches these.
    std::vector<std::complex<float>> kh_lut;
    std::vector<std::complex<float>> kh_h_w;

    void ensure(size_t block_symbols, size_t len)
    {
        if (template_len != len || templates.size() < block_symbols * len) {
            template_len = len;
            templates.resize(block_symbols * len);
        }
    }

    void ensure_fused_cfo(size_t tap_count)
    {
        const size_t lut = kFusedHalfTemplates * kFusedHalfSpan;
        if (fused_lut_lo.size() < lut)
            fused_lut_lo.resize(lut);
        if (fused_lut_hi.size() < lut)
            fused_lut_hi.resize(lut);
        if (fused_h_w.size() < tap_count)
            fused_h_w.resize(tap_count);
        const size_t tpl_len =
            tap_count + kSamplesPerChip * (kChipsPerBurst - 1);
        if (fused_tpl.size() < tpl_len)
            fused_tpl.resize(tpl_len);
    }

    // Integer-grid known-hop kernel: identical template state to the
    // 4-candidate exact-CFO kernel, only the dot kernel differs.
    void ensure_known_hop(size_t tap_count) { ensure_fused_cfo(tap_count); }

    // Fractional-phase (SFO) known-hop kernel: one lo/hi half-template LUT and
    // one h*exp(-j*w*q) ramp per phase bin.  Allocated once per packet.
    void ensure_known_hop_sfo(size_t tap_count, size_t phase_bins)
    {
        ensure_fused_cfo(tap_count);
        const size_t lut = phase_bins * kKhLutPerBin;
        if (kh_lut.size() < lut)
            kh_lut.resize(lut);
        const size_t hw = phase_bins * tap_count;
        if (kh_h_w.size() < hw)
            kh_h_w.resize(hw);
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

// ---------------------------------------------------------------------------
// Exact-CFO fused payload helpers.
// ---------------------------------------------------------------------------

// Reject any spreading chip outside {+1,-1}.  A zero must never be silently
// treated as +1, so this check is kept even though production callers derive
// the stream from prepare_fused_payload_pn().  Vectorised over 32 chips; the
// scan is O(symbols*8) bytes and negligible next to the correlation.
inline bool pn_stream_is_pm1(const int8_t* pn, size_t count)
{
#if UWB_CIR_FIR_HAVE_AVX2
    const __m256i one = _mm256_set1_epi8(1);
    const __m256i minus_one = _mm256_set1_epi8(-1);
    size_t i = 0;
    for (; i + 32 <= count; i += 32) {
        const __m256i v =
            _mm256_loadu_si256(reinterpret_cast<const __m256i*>(pn + i));
        const __m256i ok = _mm256_or_si256(_mm256_cmpeq_epi8(v, one),
                                           _mm256_cmpeq_epi8(v, minus_one));
        if (_mm256_movemask_epi8(ok) != -1)
            return false;
    }
    for (; i < count; ++i) {
        if (pn[i] != 1 && pn[i] != -1)
            return false;
    }
    return true;
#else
    for (size_t i = 0; i < count; ++i) {
        if (pn[i] != 1 && pn[i] != -1)
            return false;
    }
    return true;
#endif
}

// dst[q] += (wr + j*wi) * src[q] for q < count.
inline void accumulate_scaled_complex(std::complex<float>* dst,
                                      const std::complex<float>* src,
                                      size_t count,
                                      float wr,
                                      float wi)
{
#if UWB_CIR_FIR_HAVE_AVX2
    const float* sp = reinterpret_cast<const float*>(src);
    float* dp = reinterpret_cast<float*>(dst);
    const __m256 wre = _mm256_set1_ps(wr);
    const __m256 wim = _mm256_set1_ps(wi);
    size_t q = 0;
    for (; q + 4 <= count; q += 4) {
        const __m256 h = _mm256_loadu_ps(sp + 2 * q);
        // [wr*hr - wi*hi, wr*hi + wi*hr, ...]
        const __m256 prod =
            _mm256_fmaddsub_ps(wre, h,
                               _mm256_mul_ps(wim, _mm256_permute_ps(h, 0xB1)));
        _mm256_storeu_ps(dp + 2 * q,
                         _mm256_add_ps(_mm256_loadu_ps(dp + 2 * q), prod));
    }
    for (; q < count; ++q) {
        const float sr = src[q].real();
        const float si = src[q].imag();
        dst[q] += std::complex<float>(wr * sr - wi * si, wr * si + wi * sr);
    }
#else
    for (size_t q = 0; q < count; ++q) {
        const float sr = src[q].real();
        const float si = src[q].imag();
        dst[q] += std::complex<float>(wr * sr - wi * si, wr * si + wi * sr);
    }
#endif
}

// dst[q] = (wr + j*wi) * src[q] for q < count.  dst may not alias src.
inline void scale_complex_copy(std::complex<float>* dst,
                               const std::complex<float>* src,
                               size_t count,
                               float wr,
                               float wi)
{
#if UWB_CIR_FIR_HAVE_AVX2
    const float* sp = reinterpret_cast<const float*>(src);
    float* dp = reinterpret_cast<float*>(dst);
    const __m256 wre = _mm256_set1_ps(wr);
    const __m256 wim = _mm256_set1_ps(wi);
    size_t q = 0;
    for (; q + 4 <= count; q += 4) {
        const __m256 h = _mm256_loadu_ps(sp + 2 * q);
        _mm256_storeu_ps(dp + 2 * q,
                         _mm256_fmaddsub_ps(
                             wre, h,
                             _mm256_mul_ps(wim, _mm256_permute_ps(h, 0xB1))));
    }
    for (; q < count; ++q) {
        const float sr = src[q].real();
        const float si = src[q].imag();
        dst[q] = std::complex<float>(wr * sr - wi * si, wr * si + wi * sr);
    }
#else
    for (size_t q = 0; q < count; ++q) {
        const float sr = src[q].real();
        const float si = src[q].imag();
        dst[q] = std::complex<float>(wr * sr - wi * si, wr * si + wi * sr);
    }
#endif
}

// h_w[q] = h[q] * exp(-j*w*q) and chip_phase[c] = exp(-j*w*2c), c = 0..7.
// Both ramps are accumulated in double and rounded once to float.
inline void build_fused_tap_ramp(std::complex<float>* h_w,
                                 const std::complex<float>* h,
                                 size_t tap_count,
                                 double w,
                                 std::complex<float>* chip_phase)
{
    if (w == 0.0) {
        for (size_t q = 0; q < tap_count; ++q)
            h_w[q] = h[q];
        for (size_t c = 0; c < 8; ++c)
            chip_phase[c] = std::complex<float>(1.0f, 0.0f);
        return;
    }
    const std::complex<double> h_step(std::cos(w), -std::sin(w));
    std::complex<double> hp(1.0, 0.0);
    for (size_t q = 0; q < tap_count; ++q) {
        const double re = static_cast<double>(h[q].real());
        const double im = static_cast<double>(h[q].imag());
        h_w[q] = std::complex<float>(
            static_cast<float>(re * hp.real() - im * hp.imag()),
            static_cast<float>(re * hp.imag() + im * hp.real()));
        hp *= h_step;
    }
    const std::complex<double> c_step(std::cos(2.0 * w), -std::sin(2.0 * w));
    std::complex<double> cp(1.0, 0.0);
    for (size_t c = 0; c < 8; ++c) {
        chip_phase[c] = std::complex<float>(static_cast<float>(cp.real()),
                                            static_cast<float>(cp.imag()));
        cp *= c_step;
    }
}

// Build the 8 lo / 8 hi half templates.  Requires tap_count == kFusedFastTaps
// and h_w/chip_phase from build_fused_tap_ramp().
inline void build_fused_half_lut(std::complex<float>* lut_lo,
                                 std::complex<float>* lut_hi,
                                 const std::complex<float>* h_w,
                                 const std::complex<float>* chip_phase)
{
    constexpr size_t kHalfChips = kChipsPerBurst / 2; // 4
    for (size_t k = 0; k < kFusedHalfTemplates; ++k) {
        std::complex<float>* bl = lut_lo + k * kFusedHalfSpan;
        std::fill(bl, bl + kFusedHalfSpan, std::complex<float>(0.0f, 0.0f));
        for (size_t c = 0; c < kHalfChips; ++c) {
            // Pattern k: chip 0 is +1, bit (c-1) flips chip c.
            const bool flip = (c != 0) && (((k >> (c - 1)) & 1u) != 0);
            const float s = flip ? -1.0f : 1.0f;
            accumulate_scaled_complex(bl + 2 * c, h_w, kFusedFastTaps,
                                      s * chip_phase[c].real(),
                                      s * chip_phase[c].imag());
        }
    }
    // Upper half: same patterns, advanced 8 samples, phase exp(-j*w*8).
    const std::complex<float> hi_phase = chip_phase[kHalfChips];
    for (size_t k = 0; k < kFusedHalfTemplates; ++k) {
        scale_complex_copy(lut_hi + k * kFusedHalfSpan,
                           lut_lo + k * kFusedHalfSpan, kFusedHalfSpan,
                           hi_phase.real(), hi_phase.imag());
    }
}

// tpl[0..51] for one payload symbol from the two half-template indices.
// Signs are exactly +/-1, so they are applied as a sign-bit XOR.
inline void build_fused_symbol_template(std::complex<float>* tpl,
                                        const std::complex<float>* lut_lo,
                                        const std::complex<float>* lut_hi,
                                        size_t idx_lo,
                                        float sign_lo,
                                        size_t idx_hi,
                                        float sign_hi)
{
    const std::complex<float>* lo = lut_lo + idx_lo * kFusedHalfSpan;
    const std::complex<float>* hi = lut_hi + idx_hi * kFusedHalfSpan;
#if UWB_CIR_FIR_HAVE_AVX2
    const __m256 mlo = (sign_lo < 0.0f) ? _mm256_set1_ps(-0.0f)
                                        : _mm256_setzero_ps();
    const __m256 mhi = (sign_hi < 0.0f) ? _mm256_set1_ps(-0.0f)
                                        : _mm256_setzero_ps();
    const float* lop = reinterpret_cast<const float*>(lo);
    const float* hip = reinterpret_cast<const float*>(hi);
    float* tp = reinterpret_cast<float*>(tpl);
    // 0..7 from lo (2 vectors of 4 complex).
    for (size_t v = 0; v < 2; ++v)
        _mm256_storeu_ps(tp + 8 * v,
                         _mm256_xor_ps(_mm256_loadu_ps(lop + 8 * v), mlo));
    // 8..43: lo[8..43] + hi[0..35] (9 vectors).
    for (size_t v = 0; v < 9; ++v)
        _mm256_storeu_ps(
            tp + 16 + 8 * v,
            _mm256_add_ps(_mm256_xor_ps(_mm256_loadu_ps(lop + 16 + 8 * v), mlo),
                          _mm256_xor_ps(_mm256_loadu_ps(hip + 8 * v), mhi)));
    // 44..51 from hi[36..43] (2 vectors).
    for (size_t v = 0; v < 2; ++v)
        _mm256_storeu_ps(tp + 88 + 8 * v,
                         _mm256_xor_ps(_mm256_loadu_ps(hip + 72 + 8 * v), mhi));
#else
    for (size_t u = 0; u < 8; ++u)
        tpl[u] = lo[u] * sign_lo;
    for (size_t u = 8; u < kFusedHalfSpan; ++u)
        tpl[u] = lo[u] * sign_lo + hi[u - 8] * sign_hi;
    for (size_t u = kFusedHalfSpan; u < kFusedFastLen; ++u)
        tpl[u] = hi[u - 8] * sign_hi;
#endif
}

#if UWB_CIR_FIR_HAVE_AVX2
// One complex MAC of four samples against a shared template group.
inline void fused_mac_group(const float* xp,
                            int off,
                            __m256 ht,
                            __m256 hsw,
                            __m256& acc)
{
    const __m256 x = _mm256_loadu_ps(xp + off);
    const __m256 t = _mm256_mul_ps(_mm256_movehdup_ps(x), hsw);
    acc = _mm256_add_ps(acc, _mm256_fmaddsub_ps(_mm256_moveldup_ps(x), ht, t));
}

inline std::complex<float> fused_hsum4(__m256 acc)
{
    const __m128 lo = _mm256_castps256_ps128(acc);
    const __m128 hi = _mm256_extractf128_ps(acc, 1);
    const __m128 s = _mm_add_ps(lo, hi);
    const __m128 s2 = _mm_add_ps(s, _mm_movehl_ps(s, s));
    return std::complex<float>(_mm_cvtss_f32(s2),
                               _mm_cvtss_f32(_mm_shuffle_ps(s2, s2, 1)));
}
#endif

// Four candidates sharing one 52-point template.  52 complex = 13 groups of
// four with no scalar tail; the template group is loaded once per iteration
// and reused by all four windows (two accumulators each to hide FMA latency).
inline void dot4_shared_fixed52(const std::complex<float>* x0,
                                const std::complex<float>* x1,
                                const std::complex<float>* x2,
                                const std::complex<float>* x3,
                                const std::complex<float>* tpl,
                                std::complex<float>* m)
{
#if UWB_CIR_FIR_HAVE_AVX2
    const float* hp = reinterpret_cast<const float*>(tpl);
    const float* xp0 = reinterpret_cast<const float*>(x0);
    const float* xp1 = reinterpret_cast<const float*>(x1);
    const float* xp2 = reinterpret_cast<const float*>(x2);
    const float* xp3 = reinterpret_cast<const float*>(x3);
    const __m256 zero = _mm256_setzero_ps();
    __m256 a0 = zero, a1 = zero, a2 = zero, a3 = zero;
    __m256 b0 = zero, b1 = zero, b2 = zero, b3 = zero;
    for (int g = 0; g < 12; g += 2) {
        const int o0 = 8 * g;
        const int o1 = o0 + 8;
        const __m256 ht0 = _mm256_loadu_ps(hp + o0);
        const __m256 hs0 = _mm256_permute_ps(ht0, 0xB1);
        fused_mac_group(xp0, o0, ht0, hs0, a0);
        fused_mac_group(xp1, o0, ht0, hs0, a1);
        fused_mac_group(xp2, o0, ht0, hs0, a2);
        fused_mac_group(xp3, o0, ht0, hs0, a3);
        const __m256 ht1 = _mm256_loadu_ps(hp + o1);
        const __m256 hs1 = _mm256_permute_ps(ht1, 0xB1);
        fused_mac_group(xp0, o1, ht1, hs1, b0);
        fused_mac_group(xp1, o1, ht1, hs1, b1);
        fused_mac_group(xp2, o1, ht1, hs1, b2);
        fused_mac_group(xp3, o1, ht1, hs1, b3);
    }
    {
        const int o = 96; // group 12 of 13
        const __m256 ht = _mm256_loadu_ps(hp + o);
        const __m256 hs = _mm256_permute_ps(ht, 0xB1);
        fused_mac_group(xp0, o, ht, hs, a0);
        fused_mac_group(xp1, o, ht, hs, a1);
        fused_mac_group(xp2, o, ht, hs, a2);
        fused_mac_group(xp3, o, ht, hs, a3);
    }
    m[0] = fused_hsum4(_mm256_add_ps(a0, b0));
    m[1] = fused_hsum4(_mm256_add_ps(a1, b1));
    m[2] = fused_hsum4(_mm256_add_ps(a2, b2));
    m[3] = fused_hsum4(_mm256_add_ps(a3, b3));
#else
    m[0] = cir_fir::dot_multi_acc8(x0, tpl, kFusedFastLen);
    m[1] = cir_fir::dot_multi_acc8(x1, tpl, kFusedFastLen);
    m[2] = cir_fir::dot_multi_acc8(x2, tpl, kFusedFastLen);
    m[3] = cir_fir::dot_multi_acc8(x3, tpl, kFusedFastLen);
#endif
}

// Two candidates sharing one 52-point template (known-hop payload kernel).
// Same 13 groups of four complex and the same even/odd two-chain reduction as
// dot4_shared_fixed52(), so each output follows the arithmetic path of the
// corresponding column in the 4-candidate kernel and only the two windows
// actually needed are read.  The template group is loaded once per iteration
// and feeds both candidates.  Non-AVX2 platforms fall back to the scalar
// multi-accumulator dot.
inline void dot2_shared_fixed52(const std::complex<float>* x0,
                                const std::complex<float>* x1,
                                const std::complex<float>* tpl,
                                std::complex<float> out[2])
{
#if UWB_CIR_FIR_HAVE_AVX2
    const float* hp = reinterpret_cast<const float*>(tpl);
    const float* xp0 = reinterpret_cast<const float*>(x0);
    const float* xp1 = reinterpret_cast<const float*>(x1);
    const __m256 zero = _mm256_setzero_ps();
    __m256 a0 = zero, a1 = zero;
    __m256 b0 = zero, b1 = zero;
    for (int g = 0; g < 12; g += 2) {
        const int o0 = 8 * g;
        const int o1 = o0 + 8;
        const __m256 ht0 = _mm256_loadu_ps(hp + o0);
        const __m256 hs0 = _mm256_permute_ps(ht0, 0xB1);
        fused_mac_group(xp0, o0, ht0, hs0, a0);
        fused_mac_group(xp1, o0, ht0, hs0, a1);
        const __m256 ht1 = _mm256_loadu_ps(hp + o1);
        const __m256 hs1 = _mm256_permute_ps(ht1, 0xB1);
        fused_mac_group(xp0, o1, ht1, hs1, b0);
        fused_mac_group(xp1, o1, ht1, hs1, b1);
    }
    {
        const int o = 96; // group 12 of 13
        const __m256 ht = _mm256_loadu_ps(hp + o);
        const __m256 hs = _mm256_permute_ps(ht, 0xB1);
        fused_mac_group(xp0, o, ht, hs, a0);
        fused_mac_group(xp1, o, ht, hs, a1);
    }
    out[0] = fused_hsum4(_mm256_add_ps(a0, b0));
    out[1] = fused_hsum4(_mm256_add_ps(a1, b1));
#else
    out[0] = cir_fir::dot_multi_acc8(x0, tpl, kFusedFastLen);
    out[1] = cir_fir::dot_multi_acc8(x1, tpl, kFusedFastLen);
#endif
}

// Window-origin and length validation shared by the payload kernels.  Mirrors
// correlate_field()'s uint64 bounds logic exactly: symbol 0 starts at `base0`,
// the furthest candidate window must end at or before sample n-1.
inline bool payload_window_bounds(size_t n,
                                  size_t tap_count,
                                  int64_t chip_start,
                                  size_t samples_per_chip,
                                  int64_t payload_start,
                                  size_t symbols,
                                  int64_t& base0,
                                  size_t& len)
{
    if (tap_count == 0 || tap_count > kMaxTapCount)
        return false;
    if (samples_per_chip != kSamplesPerChip)
        return false;
    if (chip_start < 0 || payload_start < 0 || symbols == 0)
        return false;
    if (symbols > (static_cast<size_t>(1) << 20))
        return false;
    const int64_t spc = static_cast<int64_t>(samples_per_chip);
    if (payload_start >
        (std::numeric_limits<int64_t>::max() - chip_start) / spc)
        return false;
    const int64_t window0 = chip_start + payload_start * spc;
    base0 = window0 - static_cast<int64_t>(tap_count) + 1;
    if (base0 < 0)
        return false;
    len = template_length(tap_count, samples_per_chip, kChipsPerBurst);
    int64_t max_candidate_offset = 0;
    for (size_t p = 0; p < kCandidateCount; ++p) {
        if (static_cast<int64_t>(kCandidateChipOffset[p]) > max_candidate_offset)
            max_candidate_offset =
                static_cast<int64_t>(kCandidateChipOffset[p]);
    }
    const int64_t last_start_chip =
        static_cast<int64_t>(symbols - 1) *
            static_cast<int64_t>(kChipsPerSymbol) +
        max_candidate_offset;
    const uint64_t add = static_cast<uint64_t>(last_start_chip) *
                             static_cast<uint64_t>(spc) +
                         static_cast<uint64_t>(len) - 1;
    const uint64_t last = static_cast<uint64_t>(base0) + add;
    if (last < static_cast<uint64_t>(base0) || last >= static_cast<uint64_t>(n))
        return false;
    return true;
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
    if (!pn_stream_is_pm1(pn, symbols * geometry.chips_per_burst))
        return false;

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
    // No clock calls at all when the caller does not ask for timings.
    const bool want_timings = (timings != nullptr);
    std::chrono::steady_clock::time_point t0, t1, t2;

    for (size_t start = 0; start < symbols; start += kBlockSymbols) {
        const size_t count = std::min(kBlockSymbols, symbols - start);
        if (want_timings)
            t0 = std::chrono::steady_clock::now();
        for (size_t i = 0; i < count; ++i) {
            build_template_geometry(
                scratch.templates.data() + i * len, fir_taps, tap_count,
                pn + (start + i) * geometry.chips_per_burst,
                geometry.chips_per_burst, samples_per_chip);
        }
        if (want_timings) {
            t1 = std::chrono::steady_clock::now();
            t_template += t1 - t0;
        }

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
        if (want_timings) {
            t2 = std::chrono::steady_clock::now();
            t_correlate += t2 - t1;
        }
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

// Exact-CFO fused payload correlation.
//
// Mathematical contract (no approximation): the result is identical to
// derotating the raw buffer first,
//     rx_derot[i] = rx_raw[i] * exp(-j*cfo_w_rad*i),
// and then calling correlate_payload(rx_derot, ...).  Factoring the phase out
// of each window,
//     M(s,p) = exp(-j*w*win(s,p)) * sum_m tpl_w[m] * rx_raw[win(s,p) + m],
// with tpl_w[m] = exp(-j*w*m) * t[m] = sum_c pn[c] * exp(-j*w*2c) * h_w[m-2c]
// and h_w[q] = h[q]*exp(-j*w*q).  The window phase uses a double-precision
// symbol ramp re-anchored every kFusedPhaseAnchor symbols; the four candidate
// offsets contribute four per-packet constants.  38 taps take the 8+8
// half-template LUT and the four-way shared-template AVX2 kernel; any other
// tap count builds the exact per-symbol template and uses dot_generic().
//
// `cfo_w_rad == 0.0` means no derotation (no trig calls at all).  `write_complex
// == false` skips the diagnostic complex buffer entirely; signed_value is
// always written in full.  `validate_pn` keeps the explicit +/-1 check for QA
// and debug callers; production may set it false because
// prepare_fused_payload_pn() only ever emits +/-1.
inline bool correlate_payload_fused_cfo_impl(
    const std::complex<float>* rx_raw,
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
    double cfo_w_rad,
    Metrics& out,
    Scratch& scratch,
    Timings* timings,
    bool write_complex,
    bool validate_pn)
{
    if (!rx_raw || !fir_taps || !pn)
        return false;
    if (polarity != 1 && polarity != -1)
        return false;
    if (!std::isfinite(cfo_w_rad) || !std::isfinite(rot.real()) ||
        !std::isfinite(rot.imag()))
        return false;
    int64_t base0 = 0;
    size_t len = 0;
    if (!payload_window_bounds(n, tap_count, chip_start, samples_per_chip,
                               payload_start, symbols, base0, len))
        return false;
    if (validate_pn && !pn_stream_is_pm1(pn, symbols * kChipsPerBurst))
        return false;

    const double w = cfo_w_rad;
    const bool has_cfo = (w != 0.0);
    const bool fast = (tap_count == kFusedFastTaps);

    scratch.ensure_fused_cfo(tap_count);
    std::complex<float>* h_w = scratch.fused_h_w.data();
    std::complex<float>* tpl = scratch.fused_tpl.data();

    using clock = std::chrono::steady_clock;
    clock::time_point t_begin;
    if (timings)
        t_begin = clock::now();

    std::complex<float> chip_phase[kChipsPerBurst];
    build_fused_tap_ramp(h_w, fir_taps, tap_count, w, chip_phase);
    if (fast) {
        build_fused_half_lut(scratch.fused_lut_lo.data(),
                             scratch.fused_lut_hi.data(), h_w, chip_phase);
    }
    clock::time_point t_templ;
    if (timings)
        t_templ = clock::now();

    // Per-packet phase of each candidate window offset (chips -> samples).
    std::complex<float> cand_phase[kCandidateCount];
    for (size_t p = 0; p < kCandidateCount; ++p) {
        if (!has_cfo) {
            cand_phase[p] = std::complex<float>(1.0f, 0.0f);
            continue;
        }
        const double ang = -w * static_cast<double>(
                                   kCandidateChipOffset[p] * samples_per_chip);
        cand_phase[p] = std::complex<float>(static_cast<float>(std::cos(ang)),
                                           static_cast<float>(std::sin(ang)));
    }

    out.reset(symbols, write_complex);

    const int64_t symbol_stride =
        static_cast<int64_t>(kChipsPerSymbol) *
        static_cast<int64_t>(samples_per_chip);
    const double dphi = -w * static_cast<double>(symbol_stride);
    const std::complex<double> step(std::cos(dphi), std::sin(dphi));
    std::complex<double> ramp(1.0, 0.0);

    const std::complex<float>* lut_lo = scratch.fused_lut_lo.data();
    const std::complex<float>* lut_hi = scratch.fused_lut_hi.data();
    const int64_t off_sample[kCandidateCount] = {
        static_cast<int64_t>(kCandidateChipOffset[0]) *
            static_cast<int64_t>(samples_per_chip),
        static_cast<int64_t>(kCandidateChipOffset[1]) *
            static_cast<int64_t>(samples_per_chip),
        static_cast<int64_t>(kCandidateChipOffset[2]) *
            static_cast<int64_t>(samples_per_chip),
        static_cast<int64_t>(kCandidateChipOffset[3]) *
            static_cast<int64_t>(samples_per_chip)
    };

    for (size_t s = 0; s < symbols; ++s) {
        std::complex<float> phase(1.0f, 0.0f);
        if (has_cfo) {
            if ((s % kFusedPhaseAnchor) == 0) {
                const double ang =
                    -w * static_cast<double>(
                             base0 + static_cast<int64_t>(s) * symbol_stride);
                ramp = std::complex<double>(std::cos(ang), std::sin(ang));
            }
            phase = std::complex<float>(static_cast<float>(ramp.real()),
                                        static_cast<float>(ramp.imag()));
            ramp *= step;
        }

        if (fast) {
            const int8_t* ps = pn + s * kChipsPerBurst;
            const float sign_lo = (ps[0] < 0) ? -1.0f : 1.0f;
            const float sign_hi = (ps[4] < 0) ? -1.0f : 1.0f;
            size_t idx_lo = 0;
            size_t idx_hi = 0;
            for (size_t c = 1; c < 4; ++c) {
                if (ps[c] != ps[0])
                    idx_lo |= (static_cast<size_t>(1) << (c - 1));
                if (ps[c + 4] != ps[4])
                    idx_hi |= (static_cast<size_t>(1) << (c - 1));
            }
            build_fused_symbol_template(tpl, lut_lo, lut_hi, idx_lo, sign_lo,
                                        idx_hi, sign_hi);
        } else {
            std::fill(tpl, tpl + len, std::complex<float>(0.0f, 0.0f));
            for (size_t c = 0; c < kChipsPerBurst; ++c) {
                const float sgn =
                    (pn[s * kChipsPerBurst + c] < 0) ? -1.0f : 1.0f;
                accumulate_scaled_complex(tpl + samples_per_chip * c, h_w,
                                          tap_count,
                                          sgn * chip_phase[c].real(),
                                          sgn * chip_phase[c].imag());
            }
        }

        const int64_t win0 =
            base0 + static_cast<int64_t>(s) * symbol_stride;
        std::complex<float> m[kCandidateCount];
        if (fast) {
            dot4_shared_fixed52(rx_raw + win0 + off_sample[0],
                                rx_raw + win0 + off_sample[1],
                                rx_raw + win0 + off_sample[2],
                                rx_raw + win0 + off_sample[3], tpl, m);
        } else {
            for (size_t p = 0; p < kCandidateCount; ++p)
                m[p] = dot_generic(rx_raw + win0 + off_sample[p], tpl, len);
        }

        for (size_t p = 0; p < kCandidateCount; ++p) {
            std::complex<float> value = m[p];
            if (has_cfo)
                value = (phase * cand_phase[p]) * m[p];
            if (write_complex)
                out.complex_value[kCandidateCount * s + p] = value;
            const float re = rot.real() * value.real() -
                             rot.imag() * value.imag();
            out.signed_value[kCandidateCount * s + p] =
                (polarity < 0) ? -re : re;
        }
    }

    if (timings) {
        const clock::time_point t_end = clock::now();
        timings->template_update_us = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::microseconds>(t_templ -
                                                                 t_begin)
                .count());
        timings->correlate_us = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::microseconds>(t_end -
                                                                 t_templ)
                .count());
    }
    return true;
}

// Production entry: always validates the spreading stream.
inline bool correlate_payload_fused_cfo(
    const std::complex<float>* rx_raw,
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
    double cfo_w_rad,
    Metrics& out,
    Scratch& scratch,
    Timings* timings = nullptr,
    bool write_complex = false)
{
    return correlate_payload_fused_cfo_impl(
        rx_raw, n, fir_taps, tap_count, chip_start, samples_per_chip,
        payload_start, pn, symbols, polarity, rot, cfo_w_rad, out, scratch,
        timings, write_complex, true);
}

// Reproduce bprf_demod()'s hard decision exactly from the four signed
// metrics.  Hop 0 wins a tie against hop 1; BPM half 0 wins a tie against
// half 1.  g1 is the sign of the selected metric (negative -> 1).
inline void decide_bpm_impl(const Metrics& metrics,
                            std::vector<int8_t>& g0, std::vector<int8_t>& g1,
                            std::vector<float>* llr0, std::vector<float>* llr1)
{
    g0.resize(metrics.symbols);
    g1.resize(metrics.symbols);
    if (llr0) {
        llr0->resize(metrics.symbols);
        llr1->resize(metrics.symbols);
    }
    for (size_t s = 0; s < metrics.symbols; ++s) {
        const float* m = &metrics.signed_value[kCandidateCount * s];
        const auto decision = rxopt::decide_bpm_symbol(m, false);
        g0[s] = decision.position;
        g1[s] = decision.polarity;
        if (llr0) {
            (*llr0)[s] = decision.position_llr;
            (*llr1)[s] = rxopt::bpm_polarity_llr(m);
        }
    }
}

inline void decide_bpm(const Metrics& metrics,
                       std::vector<int8_t>& g0,
                       std::vector<int8_t>& g1)
{
    decide_bpm_impl(metrics, g0, g1, nullptr, nullptr);
}

// Soft-output twin of decide_bpm().  The hard decisions written to g0/g1 are
// bit-identical to decide_bpm() (same tie rules: hop 0 wins a hop tie, BPM
// half 0 wins a half tie, zero is non-negative), and the LLRs reproduce
// bprf_demod_soft()'s convention exactly on the same four signed metrics:
//
//   llr0[s] = |half0| - |half1|                  > 0 favours g0 = 0
//   llr1[s] = max_c(+m[c]) - max_c(-m[c])        > 0 favours g1 = 0
//
// `llr1` is the exact max-log LLR over all four BPM/hop candidates (rather than
// only the winning half's metric): when the two halves have similar magnitude
// and a noisy half flips sign, taking the global +/- max prevents feeding a
// high-confidence wrong polarity into the CL-3 Viterbi.
//
// Because the fused kernel returns unnormalized metrics (the legacy global
// max|soft| scale cancels in the hard decision), the LLRs are on the same
// arbitrary positive scale as the metrics.  A soft-decision Viterbi only
// needs a consistent scale per packet, so no renormalization is applied here.
inline void decide_bpm_soft(const Metrics& metrics,
                            std::vector<int8_t>& g0, std::vector<int8_t>& g1,
                            std::vector<float>& llr0, std::vector<float>& llr1)
{
    decide_bpm_impl(metrics, g0, g1, &llr0, &llr1);
}

// ---------------------------------------------------------------------------
// Known-hop 2-candidate kernel + symbol-level DD-PLL.
// ---------------------------------------------------------------------------

// Nearest fractional-phase-bin decomposition with correct floor semantics for
// positions on either side of zero:
//   int_pos + bin/phase_bins  ==  round(pos_samples * phase_bins) / phase_bins
// and the residual is always within +/-0.5/phase_bins of pos_samples.
inline bool decompose_frac_position(double pos_samples,
                                    size_t phase_bins,
                                    int64_t& int_pos,
                                    size_t& bin)
{
    if (phase_bins == 0 || !std::isfinite(pos_samples))
        return false;
    const double bins = static_cast<double>(phase_bins);
    const double scaled = pos_samples * bins;
    if (!std::isfinite(scaled) || std::fabs(scaled) > 9.0e15)
        return false;
    const double rounded = std::floor(scaled + 0.5);
    const int64_t n = static_cast<int64_t>(rounded);
    const int64_t ibins = static_cast<int64_t>(phase_bins);
    int64_t q = n / ibins;
    int64_t r = n % ibins;
    if (r < 0) {
        r += ibins;
        --q;
    }
    int_pos = q;
    bin = static_cast<size_t>(r);
    return true;
}

// exp(-j*theta) for one DD-PLL correction step.  With the default gains
// |theta| <= alpha*1 + max_freq_step ~ 0.13 rad, where the truncated series is
// accurate to better than 1e-8; angles that large never happen on the
// production path but still fall back to libm instead of degrading.
inline std::complex<float> ddpll_rot_neg(float theta)
{
    if (std::fabs(theta) <= 0.25f) {
        const float t2 = theta * theta;
        const float c = 1.0f - t2 * (0.5f - t2 * (1.0f / 24.0f));
        const float s =
            theta * (1.0f - t2 * (1.0f / 6.0f - t2 * (1.0f / 120.0f)));
        return std::complex<float>(c, -s);
    }
    return std::complex<float>(std::cos(theta), -std::sin(theta));
}

// Known-hop exact-CFO payload correlation (D-PHY BPM-BPSK, 6.81 Mb/s).
//
// The hop is fully determined by the spreading stream -- the IEEE 802.15.4-2020
// BPRF BPM hop index is `hop = (pn[8*s] < 0) ? 1 : 0`, the same rule the TX
// uses -- so only the two BPM-half candidates of that hop are correlated:
//
//   half 0: rx_raw[win0 + hop*16]        (8 chips at hop*8)
//   half 1: rx_raw[win0 + hop*16 + 64]   (8 chips at hop*8 + 32)
//
// Both windows share one 52-point template built from the same 8+8 half-LUT as
// correlate_payload_fused_cfo(), and dot2_shared_fixed52() reads each template
// group once, so exactly half the IQ traffic of the 4-candidate kernel is
// issued.  The CFO phase uses the same double symbol ramp re-anchored every
// kFusedPhaseAnchor symbols; the per-symbol hop constant exp(-j*w*hop*16)
// anchors it at the true known-hop window origin `win0 + hop*16`, and half 1
// adds exp(-j*w*64).
//
// `samples_per_chip_actual == 2.0 && frac_cir_bank == nullptr` selects the
// integer fast path.  Otherwise `frac_cir_bank[bin*tap_count + q]` supplies a
// per-fractional-phase CIR bank (`frac_phase_bins` entries): the symbol
// position is `base_d + s*64*spc_actual`, split into an integer sample start
// and a nearest phase bin, and the template is drawn from that bin.  The CFO
// phase anchors to the integer window start on this path.  `bin == 0` must be
// the undistorted CIR so an integer grid with a bank reproduces the fast path.
//
// With `pll_cfg.enable == false` the output is the 4-candidate signed metric of
// the known hop: m0 == signed_value[4*s + 2*hop], m1 ==
// signed_value[4*s + 2*hop + 1] (up to float reduction order).  With the PLL
// enabled, rot_cur starts at `rot` and is driven by the normalized BPSK phase
// error of the stronger half; m0/m1 are the real parts after tracking, which
// is the same quantity a genie phase would produce.  In both cases z0/z1 are
// the raw correlations before rot/polarity/PLL (same basis as
// Metrics::complex_value), and polarity is applied only to m0/m1.
//
// Returns false (without reading outside rx_raw) for any null pointer, illegal
// polarity/CFO/rotation/tap count, non-positive or non-finite sample spacing,
// unsupported fractional request, PN chip outside {+1,-1}, zero symbols, or a
// candidate window that is not fully readable.  The caller keeps the legacy /
// 4-candidate fallback; this entry never truncates or zero-pads a field.
inline bool correlate_payload_known_hop_fused_cfo(
    const std::complex<float>* rx_raw,
    size_t n,
    const std::complex<float>* fir_taps,
    size_t tap_count,
    int64_t chip_start,
    double samples_per_chip_actual,
    int64_t payload_start,
    const int8_t* pn,
    size_t symbols,
    int8_t polarity,
    std::complex<float> rot,
    double cfo_w_rad,
    const DdPllConfig& pll_cfg,
    Metrics2& out,
    Scratch& scratch,
    Timings* timings,
    bool write_complex,
    const std::complex<float>* frac_cir_bank,
    size_t frac_phase_bins)
{
    if (!rx_raw || !fir_taps || !pn)
        return false;
    if (polarity != 1 && polarity != -1)
        return false;
    if (!std::isfinite(cfo_w_rad) || !std::isfinite(rot.real()) ||
        !std::isfinite(rot.imag()))
        return false;
    if (tap_count == 0 || tap_count > kMaxTapCount)
        return false;
    if (symbols == 0 || symbols > (static_cast<size_t>(1) << 20))
        return false;
    if (chip_start < 0 || payload_start < 0)
        return false;
    if (!std::isfinite(samples_per_chip_actual) ||
        !(samples_per_chip_actual > 0.0))
        return false;
    if (pll_cfg.enable) {
        if (!std::isfinite(pll_cfg.alpha) || !std::isfinite(pll_cfg.beta) ||
            !std::isfinite(pll_cfg.max_freq_step) ||
            !std::isfinite(pll_cfg.min_mag_ratio) || pll_cfg.beta < 0.0f ||
            pll_cfg.max_freq_step < 0.0f || pll_cfg.min_mag_ratio < 0.0f)
            return false;
    }

    const bool sfo = (frac_cir_bank != nullptr);
    if (sfo) {
        if (frac_phase_bins == 0 || frac_phase_bins > kKhMaxPhaseBins)
            return false;
    } else if (samples_per_chip_actual !=
               static_cast<double>(kSamplesPerChip)) {
        // Non-integer spacing without a phase bank is unsupported.
        return false;
    }

    // The hop is taken from the same chip as the first half template sign, so
    // the +/-1 scan also protects the hop rule from a zero chip.
    if (!pn_stream_is_pm1(pn, symbols * kChipsPerBurst))
        return false;

    const size_t len =
        template_length(tap_count, kSamplesPerChip, kChipsPerBurst);
    const double w = cfo_w_rad;
    const bool has_cfo = (w != 0.0);
    const bool fast = (tap_count == kFusedFastTaps);
    const bool sfo_path = sfo;
    const double spc_d =
        sfo_path ? samples_per_chip_actual
                 : static_cast<double>(kSamplesPerChip);
    const size_t bins = sfo_path ? frac_phase_bins : 0;

    int64_t base0 = 0;
    double base_d = 0.0;
    const int64_t stride_i =
        static_cast<int64_t>(kChipsPerSymbol) *
        static_cast<int64_t>(kSamplesPerChip); // 128
    const double stride_d = static_cast<double>(kChipsPerSymbol) * spc_d;

    if (!sfo_path) {
        const int64_t spc = static_cast<int64_t>(kSamplesPerChip);
        if (payload_start >
            (std::numeric_limits<int64_t>::max() - chip_start) / spc)
            return false;
        const int64_t window0 = chip_start + payload_start * spc;
        base0 = window0 - static_cast<int64_t>(tap_count) + 1;
        if (base0 < 0)
            return false;
        // Furthest window: known hop 1, BPM half 1 = 16 + 64 samples.
        const int64_t max_off = 16 + 64;
        const uint64_t add =
            static_cast<uint64_t>(symbols - 1) *
                static_cast<uint64_t>(stride_i) +
            static_cast<uint64_t>(max_off) + static_cast<uint64_t>(len) - 1;
        const uint64_t last = static_cast<uint64_t>(base0) + add;
        if (last < static_cast<uint64_t>(base0) ||
            last >= static_cast<uint64_t>(n))
            return false;
        base_d = static_cast<double>(base0);
    } else {
        base_d = static_cast<double>(chip_start) +
                 static_cast<double>(payload_start) * spc_d -
                 static_cast<double>(tap_count) + 1.0;
        if (base_d < 0.0 || !std::isfinite(base_d))
            return false;
        const double last_d = base_d +
                              static_cast<double>(symbols - 1) * stride_d +
                              40.0 * spc_d; // hop 1 + half 1 = 40 chips
        if (!std::isfinite(last_d) || last_d > 9.0e15)
            return false;
        int64_t last_int = 0;
        size_t last_bin = 0;
        if (!decompose_frac_position(last_d, bins, last_int, last_bin))
            return false;
        if (last_int < 0)
            return false;
        const uint64_t last =
            static_cast<uint64_t>(last_int) + static_cast<uint64_t>(len) - 1;
        if (last >= static_cast<uint64_t>(n))
            return false;
        (void)last_bin;
    }

    scratch.ensure_known_hop(tap_count);
    if (sfo_path)
        scratch.ensure_known_hop_sfo(tap_count, bins);

    using clock = std::chrono::steady_clock;
    clock::time_point t_begin{};
    clock::time_point t_templ{};
    if (timings)
        t_begin = clock::now();

    std::complex<float> chip_phase[kChipsPerBurst];
    std::complex<float>* tpl = scratch.fused_tpl.data();
    if (!sfo_path) {
        build_fused_tap_ramp(scratch.fused_h_w.data(), fir_taps, tap_count, w,
                             chip_phase);
        if (fast) {
            build_fused_half_lut(scratch.fused_lut_lo.data(),
                                 scratch.fused_lut_hi.data(),
                                 scratch.fused_h_w.data(), chip_phase);
        }
    } else {
        for (size_t b = 0; b < bins; ++b) {
            build_fused_tap_ramp(scratch.kh_h_w.data() + b * tap_count,
                                 frac_cir_bank + b * tap_count, tap_count, w,
                                 chip_phase);
        }
        if (fast) {
            for (size_t b = 0; b < bins; ++b) {
                std::complex<float>* lo =
                    scratch.kh_lut.data() + b * kKhLutPerBin;
                std::complex<float>* hi =
                    lo + kFusedHalfTemplates * kFusedHalfSpan;
                build_fused_half_lut(lo, hi,
                                     scratch.kh_h_w.data() + b * tap_count,
                                     chip_phase);
            }
        }
    }
    if (timings)
        t_templ = clock::now();

    // Per-packet constants.  hop_phase[1] and half_phase are the same
    // single-rounded values as the 4-candidate cand_phase[] entries for chip
    // offsets 8 and 32, so the integer path stays close to bit-identical.
    std::complex<float> hop_phase[2] = {
        std::complex<float>(1.0f, 0.0f), std::complex<float>(1.0f, 0.0f)
    };
    std::complex<float> half_phase(1.0f, 0.0f);
    if (!sfo_path && has_cfo) {
        hop_phase[1] = std::complex<float>(
            static_cast<float>(std::cos(-w * 16.0)),
            static_cast<float>(std::sin(-w * 16.0)));
        half_phase = std::complex<float>(
            static_cast<float>(std::cos(-w * 64.0)),
            static_cast<float>(std::sin(-w * 64.0)));
    }
    const double dphi = -w * static_cast<double>(stride_i);
    const std::complex<double> ramp_step(std::cos(dphi), std::sin(dphi));
    std::complex<double> ramp(1.0, 0.0);

    out.reset(symbols, write_complex);

    const bool pll_on = pll_cfg.enable;
    // rot_cur is the per-symbol tracking rotation.  Polarity is applied only
    // to the final real metric, never folded into z0/z1 or the phase detector.
    std::complex<float> rot_cur = rot;
    float pll_freq = 0.0f;
    float pll_theta = 0.0f;
    float mag_ema = 0.0f;
    bool mag_init = false;
    size_t pll_updates = 0;

    for (size_t s = 0; s < symbols; ++s) {
        const int8_t* ps = pn + s * kChipsPerBurst;
        const int hop = (ps[0] < 0) ? 1 : 0;
        const float sign_lo = (ps[0] < 0) ? -1.0f : 1.0f;
        const float sign_hi = (ps[4] < 0) ? -1.0f : 1.0f;

        int64_t win0 = 0;
        int64_t win1 = 0;
        size_t bin = 0;
        std::complex<float> phase0(1.0f, 0.0f);
        std::complex<float> phase1(1.0f, 0.0f);

        if (!sfo_path) {
            if (has_cfo) {
                if ((s % kFusedPhaseAnchor) == 0) {
                    const double ang =
                        -w * static_cast<double>(
                                 base0 + static_cast<int64_t>(s) * stride_i);
                    ramp = std::complex<double>(std::cos(ang), std::sin(ang));
                }
                // Anchor at win0 + hop*16, i.e. the true half-0 window.
                phase0 = std::complex<float>(static_cast<float>(ramp.real()),
                                             static_cast<float>(ramp.imag())) *
                         hop_phase[hop];
                phase1 = phase0 * half_phase;
                ramp *= ramp_step;
            }
            win0 = base0 + static_cast<int64_t>(s) * stride_i + hop * 16;
            win1 = win0 + 64;
        } else {
            // Half 0 starts at the known-hop offset (hop*8 chips); half 1 is
            // another 32 chips later, matching the integer path.
            const double pos0 =
                base_d + static_cast<double>(s) * stride_d +
                static_cast<double>(hop * 8) * spc_d;
            int64_t int0 = 0;
            int64_t int1 = 0;
            size_t bin0 = 0;
            size_t bin1 = 0;
            if (!decompose_frac_position(pos0, bins, int0, bin0))
                return false;
            if (!decompose_frac_position(
                    pos0 + static_cast<double>(kChipsPerSymbol / 2) * spc_d,
                    bins, int1, bin1))
                return false;
            win0 = int0;
            win1 = int1;
            bin = bin0;
            (void)bin1;
            if (has_cfo) {
                const double ang0 = -w * static_cast<double>(int0);
                const double ang1 = -w * static_cast<double>(int1);
                phase0 = std::complex<float>(
                    static_cast<float>(std::cos(ang0)),
                    static_cast<float>(std::sin(ang0)));
                phase1 = std::complex<float>(
                    static_cast<float>(std::cos(ang1)),
                    static_cast<float>(std::sin(ang1)));
            }
        }

        if (fast) {
            size_t idx_lo = 0;
            size_t idx_hi = 0;
            for (size_t c = 1; c < 4; ++c) {
                if (ps[c] != ps[0])
                    idx_lo |= (static_cast<size_t>(1) << (c - 1));
                if (ps[c + 4] != ps[4])
                    idx_hi |= (static_cast<size_t>(1) << (c - 1));
            }
            const std::complex<float>* lut_lo;
            const std::complex<float>* lut_hi;
            if (sfo_path) {
                lut_lo = scratch.kh_lut.data() + bin * kKhLutPerBin;
                lut_hi = lut_lo + kFusedHalfTemplates * kFusedHalfSpan;
            } else {
                lut_lo = scratch.fused_lut_lo.data();
                lut_hi = scratch.fused_lut_hi.data();
            }
            build_fused_symbol_template(tpl, lut_lo, lut_hi, idx_lo, sign_lo,
                                        idx_hi, sign_hi);
        } else {
            const std::complex<float>* hw =
                sfo_path ? scratch.kh_h_w.data() + bin * tap_count
                         : scratch.fused_h_w.data();
            std::fill(tpl, tpl + len, std::complex<float>(0.0f, 0.0f));
            for (size_t c = 0; c < kChipsPerBurst; ++c) {
                const float sgn = (ps[c] < 0) ? -1.0f : 1.0f;
                accumulate_scaled_complex(
                    tpl + kSamplesPerChip * c, hw, tap_count,
                    sgn * chip_phase[c].real(), sgn * chip_phase[c].imag());
            }
        }

        std::complex<float> m[2];
        if (fast) {
            dot2_shared_fixed52(rx_raw + win0, rx_raw + win1, tpl, m);
        } else {
            m[0] = dot_generic(rx_raw + win0, tpl, len);
            m[1] = dot_generic(rx_raw + win1, tpl, len);
        }
        // Raw complex correlation on the same phase basis as
        // Metrics::complex_value: CFO fusion applied, rot/polarity/PLL not.
        const std::complex<float> v0 = has_cfo ? phase0 * m[0] : m[0];
        const std::complex<float> v1 = has_cfo ? phase1 * m[1] : m[1];
        if (write_complex) {
            out.z0[s] = v0;
            out.z1[s] = v1;
        }

        if (pll_on) {
            // rot_cur is the per-symbol tracked rotation (starts at rot).
            const std::complex<float> r0 = rot_cur * v0;
            const std::complex<float> r1 = rot_cur * v1;
            const float re0 = r0.real();
            const float re1 = r1.real();
            out.m0[s] = (polarity < 0) ? -re0 : re0;
            out.m1[s] = (polarity < 0) ? -re1 : re1;

            const float norm0 = std::norm(r0);
            const float norm1 = std::norm(r1);
            const std::complex<float> z_win =
                (norm0 >= norm1) ? r0 : r1;
            const float mag = std::sqrt(std::norm(z_win));
            if (!std::isfinite(mag)) {
                // Never propagate a non-finite phase error into the loop.
                continue;
            }
            if (!mag_init) {
                mag_ema = mag;
                mag_init = true;
            } else {
                mag_ema += 0.05f * (mag - mag_ema);
            }
            if (!(mag_ema > 0.0f) ||
                mag < pll_cfg.min_mag_ratio * mag_ema)
                continue;
            const float sgn = (z_win.real() >= 0.0f) ? 1.0f : -1.0f;
            const float err = (sgn * z_win.imag()) / (mag + 1e-12f);
            pll_freq += pll_cfg.beta * err;
            pll_freq = std::max(-pll_cfg.max_freq_step,
                                std::min(pll_cfg.max_freq_step, pll_freq));
            const float dtheta = pll_cfg.alpha * err + pll_freq;
            pll_theta -= dtheta;
            rot_cur *= ddpll_rot_neg(dtheta);
            ++pll_updates;
            if ((pll_updates % 64) == 0) {
                const float rmag = std::sqrt(std::norm(rot_cur));
                if (rmag > 0.0f && std::isfinite(rmag))
                    rot_cur /= rmag;
            }
        } else {
            // PLL disabled: rot_cur == rot, and the arithmetic matches the
            // 4-candidate signed_value expression exactly.
            const float re0 = rot.real() * v0.real() -
                              rot.imag() * v0.imag();
            const float re1 = rot.real() * v1.real() -
                              rot.imag() * v1.imag();
            out.m0[s] = (polarity < 0) ? -re0 : re0;
            out.m1[s] = (polarity < 0) ? -re1 : re1;
        }
    }

    out.pll_freq_rad = pll_freq;
    out.pll_theta = pll_theta;
    out.pll_updates = pll_updates;

    if (timings) {
        const clock::time_point t_end = clock::now();
        timings->template_update_us = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::microseconds>(t_templ -
                                                                 t_begin)
                .count());
        timings->correlate_us = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::microseconds>(t_end -
                                                                 t_templ)
                .count());
    }
    return true;
}

// Known-hop 2-candidate PHR correlation (64 chips/burst, 512 chips/symbol).
//
// The PHR hop follows the same rule as the payload (`hop = pn[64*s] < 0`).
// The two BPM-half windows are 64 chips (hop offset) and 64 + 256 chips from
// the symbol start, matching candidates 0/1/2/3 of correlate_phr() for the
// known hop.  Only 21 symbols are processed, so the template is built per
// symbol instead of being LUT-cached; m0/m1 are the signed real parts, and z0/
// z1 are ALWAYS filled because Wave 2 needs the 21 complex pilot values for
// PHR frequency/phase refinement and biorthogonal decoding.
inline bool correlate_phr_known_hop(const std::complex<float>* rx,
                                    size_t n,
                                    const std::complex<float>* fir_taps,
                                    size_t tap_count,
                                    int64_t chip_start,
                                    size_t samples_per_chip,
                                    int64_t phr_start,
                                    const int8_t* pn,
                                    int8_t polarity,
                                    std::complex<float> rot,
                                    Metrics2& out,
                                    Scratch& scratch,
                                    Timings* timings = nullptr)
{
    if (!rx || !fir_taps || !pn)
        return false;
    if (polarity != 1 && polarity != -1)
        return false;
    if (tap_count == 0 || tap_count > kMaxTapCount)
        return false;
    if (samples_per_chip != kSamplesPerChip)
        return false;
    if (chip_start < 0 || phr_start < 0)
        return false;
    if (!std::isfinite(rot.real()) || !std::isfinite(rot.imag()))
        return false;
    if (!pn_stream_is_pm1(pn, kPhrSymbols * kPhrChipsPerBurst))
        return false;

    const int64_t spc = static_cast<int64_t>(samples_per_chip);
    if (phr_start > (std::numeric_limits<int64_t>::max() - chip_start) / spc)
        return false;
    const size_t len = template_length(tap_count, kSamplesPerChip,
                                       kPhrChipsPerBurst);
    const int64_t base0 = chip_start + phr_start * spc -
                          static_cast<int64_t>(tap_count) + 1;
    if (base0 < 0)
        return false;
    // Furthest window: known hop 1, BPM half 1 = (64 + 256) chips.
    const int64_t max_off =
        static_cast<int64_t>(kPhrChipsPerBurst + kPhrChipsPerSymbol / 2) * spc;
    const int64_t stride_i =
        static_cast<int64_t>(kPhrChipsPerSymbol) * spc;
    const uint64_t add = static_cast<uint64_t>(kPhrSymbols - 1) *
                             static_cast<uint64_t>(stride_i) +
                         static_cast<uint64_t>(max_off) +
                         static_cast<uint64_t>(len) - 1;
    const uint64_t last = static_cast<uint64_t>(base0) + add;
    if (last < static_cast<uint64_t>(base0) || last >= static_cast<uint64_t>(n))
        return false;

    using clock = std::chrono::steady_clock;
    clock::time_point t_begin{};
    clock::time_point t_templ{};
    if (timings)
        t_begin = clock::now();

    scratch.ensure(kPhrSymbols, len);
    std::complex<float>* templates = scratch.templates.data();
    for (size_t s = 0; s < kPhrSymbols; ++s) {
        build_template_geometry(templates + s * len, fir_taps, tap_count,
                                pn + s * kPhrChipsPerBurst,
                                kPhrChipsPerBurst, kSamplesPerChip);
    }
    if (timings)
        t_templ = clock::now();

    out.reset(kPhrSymbols, true);
    for (size_t s = 0; s < kPhrSymbols; ++s) {
        const int hop =
            (pn[s * kPhrChipsPerBurst] < 0) ? 1 : 0;
        const int64_t win0 = base0 + static_cast<int64_t>(s) * stride_i +
                             static_cast<int64_t>(hop * kPhrChipsPerBurst) *
                                 spc;
        const int64_t win1 =
            win0 + static_cast<int64_t>(kPhrChipsPerSymbol / 2) * spc;
        const std::complex<float>* tpl = templates + s * len;
        // Raw complex correlation on the Metrics::complex_value phase basis:
        // no rot, no polarity.  Wave 2 consumes these 21 pilot values
        // directly, so they are always written.
        const std::complex<float> z0 = dot_generic(rx + win0, tpl, len);
        const std::complex<float> z1 = dot_generic(rx + win1, tpl, len);
        out.z0[s] = z0;
        out.z1[s] = z1;
        const float re0 = rot.real() * z0.real() -
                          rot.imag() * z0.imag();
        const float re1 = rot.real() * z1.real() -
                          rot.imag() * z1.imag();
        out.m0[s] = (polarity < 0) ? -re0 : re0;
        out.m1[s] = (polarity < 0) ? -re1 : re1;
    }

    if (timings) {
        const clock::time_point t_end = clock::now();
        timings->template_update_us = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::microseconds>(t_templ -
                                                                 t_begin)
                .count());
        timings->correlate_us = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::microseconds>(t_end -
                                                                 t_templ)
                .count());
    }
    return true;
}

} // namespace rxfusion
} // namespace demod
} // namespace uwb
} // namespace gr
