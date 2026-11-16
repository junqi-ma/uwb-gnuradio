/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Phase-2 AVX-512 kernels for the demodulator and the SIC front end.
 *
 * Plan: docs/performance/解调与SIC_AVX512内核扩展开发规划_2026-11-16.md (P1/P2/P3).
 *
 * Only the kernels that MEASURED a gain and stayed bit-identical to AVX2 are
 * here.  The measured losers stay on AVX2 and are documented in
 * uwb_kernel_backend.cc:
 *
 *   in : real-template 8-lag dot, element-wise complex add/sub/sub3/scale,
 *        batched SFD fold
 *   out: fixed-38 CIR FIR and fused-52 payload CMF dots (the 512-bit complex
 *        MAC is slower on this host: +95% / +29%)
 *   out: 20-tap SIC pulse accumulation (store-forwarding bound: +1%)
 *
 * Numerical contract (why this is safe)
 * -------------------------------------
 * Every kernel here keeps the AVX2 lane mapping, FMA use, accumulation order
 * and reduction tree, so the output is BIT-IDENTICAL to the AVX2 reference,
 * not merely within tolerance.  Two mechanisms make that possible:
 *
 *   - Element-wise ops (add/sub/sub3/scale/fold) have no reduction at all, so
 *     widening the vector is transparent.
 *   - The 8-lag dot pairs lag m with lag m+4 inside one 512-bit register: a
 *     single unaligned 512-bit load at rx + (k+m) yields complex samples
 *     (k+m .. k+m+3) in the low half and (k+m+4 .. k+m+7) in the high half, so
 *     lane j of each half accumulates exactly the taps that AVX2 accumulator
 *     lane j accumulates for that lag.  The final reduction reuses the AVX2
 *     scalar tree tmp[0]+tmp[2]+tmp[4]+tmp[6].
 *
 * qa_uwb_demod_sic_avx512.cc enforces the identity sample by sample.  If a
 * future kernel has to change the summation tree, that QA must be replaced by
 * an explicit tolerance contract plus an independent numeric reference -- it
 * must not be silently relaxed.
 *
 * All functions are out-of-line and take plain (non-vector) types, so the
 * header-only cores can call them without any ODR hazard.  The callers batch
 * (one call per 8 lags / per whole SFD fold / per whole vector op), never per
 * tap, chip or sample.
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

/** True when this translation unit was compiled into the build. */
bool available() noexcept;

/**
 * Real-template dot product for eight consecutive lags.
 *
 *   rx       : rx[j .. j+L+6] must be readable (lags j..j+7 overlap by L)
 *   taps_dup : [t0,t0,t1,t1,...] duplicated real template, 2*L floats
 *   out[0..7]: sum_{k<L} rx[j+m+k] * t[k] for m = 0..7
 *
 * Bit-identical to two real_template_dot_4lag() calls.  L == 0 or L > 65536
 * yields zeros, matching the AVX2 guard.
 */
void real_template_dot_8lag(const gr_complex* rx, const float* taps_dup,
                            size_t L, gr_complex out[8]);

/** dst[i] += src[i] for count complex samples.  Bit-identical to AVX2. */
void accumulate_cf32(gr_complex* dst, const gr_complex* src, size_t count);

/** dst[i] -= src[i] for count complex samples.  Bit-identical to AVX2. */
void sub_cf32(gr_complex* dst, const gr_complex* src, size_t count);

/** dst[i] = a[i] - b[i] for count complex samples.  Bit-identical to AVX2. */
void sub3_cf32(gr_complex* dst, const gr_complex* a, const gr_complex* b,
               size_t count);

/** dst[i] *= scale for count complex samples.  Bit-identical to AVX2. */
void scale_cf32(gr_complex* dst, float scale, size_t count);

/**
 * Whole SFD fold: out[k] = sum_i sfd_seq[i] * rx_base[i*sym + k] for
 * k < fold_len, skipping sfd_seq[i] == 0.  The symbol loop stays inside this
 * translation unit so the batched call pays one call, not one per symbol.
 *
 * Semantics match rxopt::fold_sfd_window() exactly, including the
 * first-nonzero-symbol memcpy/fill shortcut.
 */
void fold_sfd_window(const gr_complex* rx_base, size_t sym,
                     const int8_t* sfd_seq, size_t sfd_len, size_t fold_len,
                     gr_complex* out);

} // namespace avx512
} // namespace simd
} // namespace core
} // namespace uwb
} // namespace gr
