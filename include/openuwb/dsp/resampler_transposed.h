/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * RS-5: cross-macroblock transposed rational-resampler kernel.
 *
 * The production macroblock kernels compute one output at a time and reduce
 * the complex accumulator down to a scalar.  This kernel instead batches G
 * consecutive macroblocks (G = 4 on AVX2, 8 on AVX-512) and keeps the
 * accumulator in transposed form:
 *
 *   X_tile[u] = [ x(mb0+0, u), x(mb0+1, u), ..., x(mb0+G-1, u) ]
 *
 * where x(m, u) is the complex sample at input offset
 *     (mb0+m)*Decim + u - (H-1)
 * (zero outside the window).  One vector lane therefore belongs to one
 * macroblock, and an output is
 *
 *   y(m, j) = sum_{k=0..H-1} tap[arm_j][k] * X_tile[w_j + k][m]
 *
 * with arm_j = (j*Decim) mod Interp and w_j = floor(j*Decim / Interp).
 * There is no horizontal reduction and no reuse branch: only broadcast-scalar
 * FMAs over the tap axis.  The tap table stores exactly H coefficients per
 * arm (H = ceil(T/Interp), NOT padded to 8), so the realtime/quality arms do
 * not pay the padding FMA the macroblock kernels do.
 *
 * Numerical contract: this is a deliberately DIFFERENT accumulation order
 * (tap axis, one accumulator) from the bit-exact macroblock kernels.  It is an
 * explicitly selected path and is validated against an independent float64
 * upfirdn model with |error| < 1e-5 (qa_uwb_resampler_transposed.cc).  The
 * existing memcmp gates of the production kernels are untouched.
 *
 * ISA isolation: the AVX2 microkernel implementations now live in
 * lib/dsp/uwb_resampler_transposed.cc (baseline AVX2/FMA build); the
 * AVX-512 8-macroblock microkernel lives in
 * uwb_resampler_transposed_avx512.cc behind function target
 * attributes, and is dispatched only when
 * UWB_SIMD_HAVE_AVX512 && simd::runtime_supported().
 * Implementations of TransposedResampler live in
 * lib/dsp/uwb_resampler_transposed.cc; this header keeps the class
 * declaration and the public types only.
 */
#pragma once

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <new>
#include <stdexcept>
#include <string>
#include <vector>

#include <openuwb/support/simd.h>
#include <openuwb/support/simd_config.h>

#if UWB_SIMD_LEVEL >= 1
// The AVX2 method declarations below name `__m256` types; the
// implementations live in lib/dsp/uwb_resampler_transposed.cc.
#include <immintrin.h>
#endif

namespace openuwb {
namespace core {
namespace simd {
namespace transposed {

using gr_complex = std::complex<float>;
using sc16 = std::complex<int16_t>;

/** 64-byte aligned allocator (the tap table and the X_tile scratch). */
template <class T, size_t Align>
struct AlignedAllocator {
    using value_type = T;

    AlignedAllocator() noexcept = default;
    template <class U>
    AlignedAllocator(const AlignedAllocator<U, Align>&) noexcept
    {
    }

    T* allocate(size_t n)
    {
        return static_cast<T*>(
            ::operator new(n * sizeof(T), std::align_val_t(Align)));
    }
    void deallocate(T* p, size_t) noexcept
    {
        ::operator delete(p, std::align_val_t(Align));
    }

    template <class U>
    struct rebind {
        using other = AlignedAllocator<U, Align>;
    };
};

template <class T, size_t Align>
bool operator==(const AlignedAllocator<T, Align>&,
                const AlignedAllocator<T, Align>&) noexcept
{
    return true;
}
template <class T, size_t Align>
bool operator!=(const AlignedAllocator<T, Align>&,
                const AlignedAllocator<T, Align>&) noexcept
{
    return false;
}

using AlignedFloatVector = std::vector<float, AlignedAllocator<float, 64>>;

#if UWB_SIMD_HAVE_AVX512
namespace avx512 {

/**
 * AVX-512 8-macroblock microkernel.  Computes all `n_mb_full / 8` complete
 * 8-macroblock groups of the one-shot window; the caller handles the
 * leftover macroblocks and the final partial macroblock.
 *
 *   taps      : pre-scaled [Interp][H] reversed tap table (64-byte aligned)
 *   tile      : scratch of at least (Decim + H - 1) * 16 floats (64-aligned)
 *   out       : CF32 destination, macroblock m at out + m*Interp
 */
void transposed_process_window(const float* taps, size_t H, size_t Interp,
                               size_t Decim, const sc16* in, size_t ninput,
                               size_t n_mb_full, float* tile, gr_complex* out);

} // namespace avx512
#endif

/**
 * Standalone one-shot transposed resampler for one (Interp, Decim) direction.
 * It owns only the tap table and scratch; it is stateless between windows, so
 * a burst worker can reset a window by simply calling process_window_sc16().
 */
template <uint32_t Interp, uint32_t Decim>
class TransposedResampler
{

public:
    static constexpr uint32_t kInterp = Interp;
    static constexpr uint32_t kDecim = Decim;

    enum class Kernel { Auto, Avx2, Avx512, Scalar };

    TransposedResampler(const float* taps, size_t T);

    explicit TransposedResampler(const std::vector<float>& taps);

    static size_t expected_output_length(uint64_t N, size_t T);

    size_t expected_output_length(uint64_t N) const;

    size_t tap_count() const;

    size_t arm_length() const;

    /** "auto" (default), "avx2", "avx512", "scalar". */
    void set_kernel(const std::string& name);

    const char* kernel_name() const;

    /**
     * One-shot full window.  `out` must hold expected_output_length(ninput)
     * samples; a smaller `max_out` truncates the stream (the production
     * caller always sizes the buffer to the expected length).
     */
    size_t process_window_sc16(const sc16* in,
                               size_t ninput,
                               float scale,
                               gr_complex* out,
                               size_t max_out);

private:
    size_t T_;
    size_t H_;
    AlignedFloatVector taps_;   // unscaled [Interp][H], reversed
    AlignedFloatVector scaled_; // pre-scaled copy
    float last_scale_ = 1.0f;
    Kernel kernel_ = Kernel::Auto;
    AlignedFloatVector tile_;

    Kernel resolve_kernel() const;

    void ensure_scaled(float scale);

    void scalar_output(const sc16* in, size_t ninput, size_t o,
                       gr_complex* out) const;

    void scalar_macroblock(const sc16* in, size_t ninput, size_t mb,
                           gr_complex* out) const;

#if UWB_SIMD_LEVEL >= 1

    static int32_t load_sc16_bits(const sc16* p);

    /**
     * Same-loop 4-phase tap accumulation over X_tile:
     * breaks the 4-cycle FMA dependency chain across 4 independent accumulators
     * and reuses L1d tile lines across phases j..j+3.
     */
    template <size_t H_VAL>
    static inline void phase_accum_4x_avx2(
        const float* tp0, const float* tp1, const float* tp2, const float* tp3,
        const float* x0, const float* x1, const float* x2, const float* x3,
        size_t h_rt,
        __m256& c0, __m256& c1, __m256& c2, __m256& c3) noexcept;

    /**
     * Same-loop 8-phase tap accumulation: 8 independent FMA chains saturate
     * dual 256-bit FMA ports and reuse adjacent X_tile slots.
     */
    template <size_t H_VAL>
    static inline void phase_accum_8x_avx2(
        const float* const* tp8, const float* const* x8, size_t h_rt,
        __m256* out8) noexcept;

    static inline void transpose_4x4_store_avx2(
        __m256 c0, __m256 c1, __m256 c2, __m256 c3,
        gr_complex* out, size_t mb0, size_t j) noexcept;

    /** 4 macroblocks, all Interp outputs, compile-time H specialization. */
    template <size_t H_VAL>
    void avx2_group_hp(const sc16* in, size_t ninput, size_t mb0,
                       gr_complex* out) noexcept;

    void avx2_group(const sc16* in, size_t ninput, size_t mb0,
                    gr_complex* out);

#endif

};
extern template class TransposedResampler<65, 48>;
extern template class TransposedResampler<65, 32>;
extern template class TransposedResampler<48, 65>;
extern template class TransposedResampler<32, 65>;

using TransposedResampler65_48 = TransposedResampler<65, 48>;
using TransposedResampler65_32 = TransposedResampler<65, 32>;
using TransposedResampler48_65 = TransposedResampler<48, 65>;
using TransposedResampler32_65 = TransposedResampler<32, 65>;

} // namespace transposed
} // namespace simd
} // namespace core
} // namespace openuwb
