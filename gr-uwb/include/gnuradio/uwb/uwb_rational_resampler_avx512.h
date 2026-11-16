/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * AVX-512 SC16 FIR kernels for the rational resampler core.
 *
 * These are the ONLY translation units that may contain AVX-512
 * instructions.  They are compiled with function target attributes
 * ("avx512f,avx512bw,avx512dq,avx512vl"), never with a global -mavx512 flag,
 * so the rest of the program stays on the AVX2/FMA baseline and cannot
 * execute a wide instruction before the runtime capability check.
 *
 * Numerical contract (why this is safe):
 *
 *   The AVX-512 kernel keeps the EXACT per-lane accumulation order of the
 *   AVX2 kernel.  One 16-float accumulator covers 8 complex taps.  Lane j
 *   (j = 0..3) accumulates the same taps as the AVX2 acc_lo lane j, and lane
 *   j+4 the same taps as acc_hi lane j.  The final reduction is
 *   lo256 + hi256 (which reproduces the AVX2 acc_lo + acc_hi step), then
 *   low128 + high128, then the same hadd chain.  For every shipped H_pad
 *   (16/24/32/48, all multiples of 8) the result is therefore BIT-IDENTICAL
 *   to the AVX2/FMA kernel, not merely within tolerance.  The QA gate
 *   qa_uwb_resampler_avx512.cc enforces this sample by sample.
 *
 *   If a future kernel changes the summation tree, that QA must be replaced
 *   by an explicit tolerance contract plus an independent numeric reference,
 *   not silently relaxed.
 */
#pragma once

#include <complex>
#include <cstddef>
#include <cstdint>

namespace gr {
namespace uwb {
namespace core {
namespace simd {
namespace avx512 {

using gr_complex = std::complex<float>;
using sc16 = std::complex<int16_t>;

/**
 * Single-output SC16 FIR.
 *   taps_scaled : pre-scaled complex tap vector, 2*hpad floats
 *                 (arms_rev_dup_scaled_ + arm * 2 * H_pad)
 *   win         : oldest-first SC16 window, hpad complex samples
 *   hpad        : tap count; must be a multiple of 8 for the fast path
 */
gr_complex fir_sc16(const float* taps_scaled, const sc16* win, size_t hpad);

/**
 * Single-output FC32 FIR.  Bit-identical to fir_avx2(); needed so the SC16
 * and FC32 paths keep their bit-exact agreement when the SC16 path switches
 * to AVX-512.
 */
gr_complex fir_cf32(const float* taps_dup, const gr_complex* win, size_t hpad);

/**
 * Four-output FC32 FIR, pointer/arm form.  Bit-identical to fir_avx2_4x().
 */
void fir_cf32_4x(const float* h_base,
                 size_t h_stride,
                 uint8_t arm0, const gr_complex* win0,
                 uint8_t arm1, const gr_complex* win1,
                 uint8_t arm2, const gr_complex* win2,
                 uint8_t arm3, const gr_complex* win3,
                 size_t hpad,
                 gr_complex* dest4);

/**
 * Four-output SC16 FIR, pointer/arm form used by the default (non-C1)
 * macroblock loop.  Bit-identical to fir_avx2_sc16_4x().
 */
void fir_sc16_4x(const float* h_base,
                 size_t h_stride,
                 uint8_t arm0, const sc16* win0,
                 uint8_t arm1, const sc16* win1,
                 uint8_t arm2, const sc16* win2,
                 uint8_t arm3, const sc16* win3,
                 size_t hpad,
                 gr_complex* dest4);

/**
 * Four-output SC16 FIR, C1 descriptor form.  h0..h3 point at one pre-scaled
 * tap vector each; win0..win3 are absolute window pointers; `reuse` bit 1/2/3
 * means win1==win0 / win2==win1 / win3==win2.  Bit-identical to
 * fir_avx2_sc16_4x_hp<H_PAD>().
 *
 * The hp16/hp24/hp32/hp48 entry points have a compile-time tap bound; the
 * generic one uses hpad_rt.  A profile whose H_pad is not one of the four
 * shipped values takes the generic entry point.
 */
void fir_sc16_4x_hp16(const float* h0, const float* h1,
                      const float* h2, const float* h3,
                      const sc16* win0, const sc16* win1,
                      const sc16* win2, const sc16* win3,
                      size_t hpad_rt, uint8_t reuse, gr_complex* dest4);
void fir_sc16_4x_hp24(const float* h0, const float* h1,
                      const float* h2, const float* h3,
                      const sc16* win0, const sc16* win1,
                      const sc16* win2, const sc16* win3,
                      size_t hpad_rt, uint8_t reuse, gr_complex* dest4);
void fir_sc16_4x_hp32(const float* h0, const float* h1,
                      const float* h2, const float* h3,
                      const sc16* win0, const sc16* win1,
                      const sc16* win2, const sc16* win3,
                      size_t hpad_rt, uint8_t reuse, gr_complex* dest4);
void fir_sc16_4x_hp48(const float* h0, const float* h1,
                      const float* h2, const float* h3,
                      const sc16* win0, const sc16* win1,
                      const sc16* win2, const sc16* win3,
                      size_t hpad_rt, uint8_t reuse, gr_complex* dest4);
void fir_sc16_4x_generic(const float* h0, const float* h1,
                         const float* h2, const float* h3,
                         const sc16* win0, const sc16* win1,
                         const sc16* win2, const sc16* win3,
                         size_t hpad_rt, uint8_t reuse, gr_complex* dest4);

/**
 * One C1 4-output descriptor, byte-for-byte the layout of
 * RationalResamplerLmCore::FixedPhaseGroup.  The core passes its table
 * directly (with a static_assert on the size) so the batch entry point below
 * can walk it without copying.
 */
struct C1GroupDesc {
    uint16_t win[4];
    uint16_t tap[4];
    uint8_t reuse;
    uint8_t pad[3];
};
static_assert(sizeof(C1GroupDesc) == 20, "C1GroupDesc layout changed");

/**
 * Batched C1 macroblock: compute `ngroups` 4-output groups in ONE call.
 *
 * This is the hot production entry point.  Calling the 4-output kernel once
 * per group from the header-only core cost ~10 cycles of call overhead per
 * group, which dominated the short H_pad=16 loop and made AVX-512 slower than
 * the inlined AVX2 kernel.  Batching moves the group loop into the AVX-512
 * translation unit, so the wide kernel only pays one call per macroblock.
 *
 *   h_base    : arms_rev_dup_scaled_.data()
 *   groups    : fp_groups_.data() + ctr * kMacroblockGroups
 *   base_win  : work_sc16_.data() + (hist_n + post_i + 1 - H_)
 *   dest      : out + produced (4 * ngroups complex outputs)
 */
void fir_sc16_macroblock_c1_hp16(const float* h_base,
                                 const C1GroupDesc* groups, size_t ngroups,
                                 const sc16* base_win, gr_complex* dest);
void fir_sc16_macroblock_c1_hp24(const float* h_base,
                                 const C1GroupDesc* groups, size_t ngroups,
                                 const sc16* base_win, gr_complex* dest);
void fir_sc16_macroblock_c1_hp32(const float* h_base,
                                 const C1GroupDesc* groups, size_t ngroups,
                                 const sc16* base_win, gr_complex* dest);
void fir_sc16_macroblock_c1_hp48(const float* h_base,
                                 const C1GroupDesc* groups, size_t ngroups,
                                 const sc16* base_win, gr_complex* dest);
void fir_sc16_macroblock_c1_generic(const float* h_base,
                                    const C1GroupDesc* groups, size_t ngroups,
                                    const sc16* base_win, size_t hpad,
                                    gr_complex* dest);

/**
 * Batched DEFAULT (non-C1) macroblock: walk the production schedule arrays
 * (arm index and input advance per output) inside the AVX-512 TU and compute
 * all `nout` outputs of one macroblock in one call.  Bit-identical to the
 * per-output/per-group AVX2 loop; batching removes the out-of-line call
 * overhead that made the FC32 path ~20% slower when called per group.
 */
void fir_sc16_macroblock(const float* h_base, size_t h_stride,
                         const uint8_t* arms, const uint8_t* advs,
                         const sc16* base_win, size_t decim,
                         size_t n_macroblocks, size_t nout, size_t hpad,
                         gr_complex* dest);
void fir_cf32_macroblock(const float* h_base, size_t h_stride,
                         const uint8_t* arms, const uint8_t* advs,
                         const gr_complex* base_win, size_t decim,
                         size_t n_macroblocks, size_t nout, size_t hpad,
                         gr_complex* dest);

} // namespace avx512
} // namespace simd
} // namespace core
} // namespace uwb
} // namespace gr
