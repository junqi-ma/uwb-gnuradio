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
 * ISA isolation: the AVX2 microkernel lives in this header (baseline AVX2/FMA
 * build); the AVX-512 8-macroblock microkernel lives in
 * uwb_resampler_transposed_avx512.cc behind function target attributes, and is
 * dispatched only when UWB_SIMD_HAVE_AVX512 && simd::runtime_supported().
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

#include <gnuradio/uwb/uwb_simd.h>
#include <gnuradio/uwb/uwb_simd_config.h>

#if UWB_SIMD_LEVEL >= 1
#include <immintrin.h>
#endif

namespace gr {
namespace uwb {
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

    TransposedResampler(const float* taps, size_t T)
        : T_(T),
          H_((T + Interp - 1) / Interp),
          taps_(static_cast<size_t>(Interp) * ((T + Interp - 1) / Interp), 0.0f)
    {
        simd::require_simd_backend();
        if (taps == nullptr || T == 0)
            throw std::invalid_argument(
                "TransposedResampler: taps must be non-empty");
        if (H_ == 0)
            throw std::invalid_argument("TransposedResampler: empty arm");

        // Reversed polyphase arms, [arm][k] with
        //   taps_[arm*H + k] = taps[arm + Interp*(H-1-k)]
        // i.e. exactly the production arms_rev_ table, but H entries per arm
        // instead of H_pad entries.
        for (size_t i = 0; i < T; ++i) {
            const size_t arm = i % Interp;
            const size_t k = i / Interp;
            taps_[arm * H_ + (H_ - 1 - k)] = taps[i];
        }
        scaled_ = taps_;
        last_scale_ = 1.0f;

        // Max G = 8 lanes: (Decim + H - 1) slots * 2 floats per complex * 8.
        tile_.assign((static_cast<size_t>(Decim) + H_ - 1) * 16, 0.0f);
    }

    explicit TransposedResampler(const std::vector<float>& taps)
        : TransposedResampler(taps.data(), taps.size())
    {
    }

    static size_t expected_output_length(uint64_t N, size_t T)
    {
        const int64_t num = (static_cast<int64_t>(N) - 1) *
                                static_cast<int64_t>(Interp) +
                            static_cast<int64_t>(T);
        if (num <= 0)
            return 0;
        return static_cast<size_t>(
            (num + static_cast<int64_t>(Decim) - 1) /
            static_cast<int64_t>(Decim));
    }

    size_t expected_output_length(uint64_t N) const
    {
        return expected_output_length(N, T_);
    }

    size_t tap_count() const { return T_; }
    size_t arm_length() const { return H_; }

    /** "auto" (default), "avx2", "avx512", "scalar". */
    void set_kernel(const std::string& name)
    {
        if (name == "auto" || name == "default" || name == "transposed") {
            kernel_ = Kernel::Auto;
        } else if (name == "avx2" || name == "transposed_avx2") {
#if UWB_SIMD_LEVEL >= 1
            kernel_ = Kernel::Avx2;
#else
            throw std::invalid_argument(
                "TransposedResampler: avx2 requested in a SCALAR build");
#endif
        } else if (name == "avx512" || name == "transposed_avx512") {
#if UWB_SIMD_HAVE_AVX512
            kernel_ = Kernel::Avx512;
#else
            throw std::invalid_argument(
                "TransposedResampler: avx512 requested but this build has no "
                "AVX-512 backend");
#endif
        } else if (name == "scalar" || name == "transposed_scalar") {
            kernel_ = Kernel::Scalar;
        } else {
            throw std::invalid_argument("TransposedResampler: unknown kernel " +
                                        name);
        }
    }

    const char* kernel_name() const
    {
        switch (resolve_kernel()) {
        case Kernel::Avx512:
            return "transposed_avx512";
        case Kernel::Avx2:
            return "transposed_avx2";
        default:
            return "transposed_scalar";
        }
    }

    /**
     * One-shot full window.  `out` must hold expected_output_length(ninput)
     * samples; a smaller `max_out` truncates the stream (the production
     * caller always sizes the buffer to the expected length).
     */
    size_t process_window_sc16(const sc16* in,
                               size_t ninput,
                               float scale,
                               gr_complex* out,
                               size_t max_out)
    {
        size_t lout = expected_output_length(ninput, T_);
        if (lout > max_out)
            lout = max_out;
        if (lout == 0)
            return 0;

        ensure_scaled(scale);

        const Kernel k = resolve_kernel();
        const size_t n_mb = lout / Interp;
        size_t done_mb = 0;

        if (k == Kernel::Avx512) {
#if UWB_SIMD_HAVE_AVX512
            const size_t groups8 = n_mb / 8;
            if (groups8 > 0) {
                avx512::transposed_process_window(
                    scaled_.data(), H_, Interp, Decim, in, ninput, groups8 * 8,
                    tile_.data(), out);
                done_mb = groups8 * 8;
            }
#endif
        } else if (k == Kernel::Avx2) {
#if UWB_SIMD_LEVEL >= 1
            const size_t groups4 = n_mb / 4;
            for (size_t g = 0; g < groups4; ++g)
                avx2_group(in, ninput, g * 4, out);
            done_mb = groups4 * 4;
#endif
        }

        // Leftover complete macroblocks (fewer than G) and the final partial
        // macroblock: scalar, same tap table and same H (no padding).
        for (; done_mb < n_mb; ++done_mb)
            scalar_macroblock(in, ninput, done_mb, out);

        const size_t produced_mb = n_mb * Interp;
        for (size_t o = produced_mb; o < lout; ++o)
            scalar_output(in, ninput, o, out);

        return lout;
    }

private:
    size_t T_;
    size_t H_;
    AlignedFloatVector taps_;   // unscaled [Interp][H], reversed
    AlignedFloatVector scaled_; // pre-scaled copy
    float last_scale_ = 1.0f;
    Kernel kernel_ = Kernel::Auto;
    AlignedFloatVector tile_;

    Kernel resolve_kernel() const
    {
        if (kernel_ != Kernel::Auto)
            return kernel_;
#if UWB_SIMD_HAVE_AVX512
        if (simd::runtime_supported())
            return Kernel::Avx512;
#endif
#if UWB_SIMD_LEVEL >= 1
        return Kernel::Avx2;
#else
        return Kernel::Scalar;
#endif
    }

    void ensure_scaled(float scale)
    {
        if (scale == last_scale_)
            return;
        const size_t n = taps_.size();
        if (scale == 1.0f) {
            std::memcpy(scaled_.data(), taps_.data(), n * sizeof(float));
        } else {
            for (size_t i = 0; i < n; ++i)
                scaled_[i] = taps_[i] * scale;
        }
        last_scale_ = scale;
    }

    void scalar_output(const sc16* in, size_t ninput, size_t o,
                       gr_complex* out) const
    {
        const size_t mb = o / Interp;
        const size_t j = o % Interp;
        const size_t arm = (j * Decim) % Interp;
        const size_t w = (j * Decim) / Interp;
        const int64_t base = static_cast<int64_t>(mb) * Decim -
                             static_cast<int64_t>(H_ - 1) +
                             static_cast<int64_t>(w);
        const float* const tp = scaled_.data() + arm * H_;
        float re = 0.0f;
        float im = 0.0f;
        for (size_t k = 0; k < H_; ++k) {
            const int64_t idx = base + static_cast<int64_t>(k);
            if (idx < 0 || idx >= static_cast<int64_t>(ninput))
                continue;
            const sc16 x = in[static_cast<size_t>(idx)];
            re += tp[k] * static_cast<float>(x.real());
            im += tp[k] * static_cast<float>(x.imag());
        }
        out[o] = gr_complex(re, im);
    }

    void scalar_macroblock(const sc16* in, size_t ninput, size_t mb,
                           gr_complex* out) const
    {
        const size_t base = mb * Interp;
        for (size_t j = 0; j < Interp; ++j)
            scalar_output(in, ninput, base + j, out);
    }

#if UWB_SIMD_LEVEL >= 1
    static int32_t load_sc16_bits(const sc16* p)
    {
        int32_t v;
        std::memcpy(&v, p, sizeof(v));
        return v;
    }

    /** FMA over the tap axis for one output phase; accumulator in lanes. */
    static inline __m256 phase_accum_avx2(const float* tp, const float* tile,
                                          size_t H)
    {
        __m256 acc = _mm256_setzero_ps();
        for (size_t k = 0; k < H; ++k) {
            acc = _mm256_fmadd_ps(_mm256_set1_ps(tp[k]),
                                  _mm256_load_ps(tile + k * 8), acc);
        }
        return acc;
    }

    /** 4 macroblocks, all Interp outputs. */
    void avx2_group(const sc16* in, size_t ninput, size_t mb0,
                    gr_complex* out)
    {
        const size_t slots = static_cast<size_t>(Decim) + H_ - 1;
        float* const tile = tile_.data();

        for (size_t u = 0; u < slots; ++u) {
            int32_t v[4];
            for (size_t m = 0; m < 4; ++m) {
                const int64_t idx = static_cast<int64_t>(mb0 + m) * Decim +
                                    static_cast<int64_t>(u) -
                                    static_cast<int64_t>(H_ - 1);
                v[m] = (idx >= 0 && idx < static_cast<int64_t>(ninput))
                           ? load_sc16_bits(in + idx)
                           : 0;
            }
            const __m128i ab =
                _mm_unpacklo_epi32(_mm_cvtsi32_si128(v[0]),
                                   _mm_cvtsi32_si128(v[1]));
            const __m128i cd =
                _mm_unpacklo_epi32(_mm_cvtsi32_si128(v[2]),
                                   _mm_cvtsi32_si128(v[3]));
            const __m128i q = _mm_unpacklo_epi64(ab, cd);
            const __m256 f = _mm256_cvtepi32_ps(_mm256_cvtepi16_epi32(q));
            _mm256_store_ps(tile + u * 8, f);
        }

        size_t j = 0;
        for (; j + 4 <= Interp; j += 4) {
            const uint32_t a0 = (j * Decim) % Interp;
            const uint32_t a1 = ((j + 1) * Decim) % Interp;
            const uint32_t a2 = ((j + 2) * Decim) % Interp;
            const uint32_t a3 = ((j + 3) * Decim) % Interp;
            const size_t w0 = (j * Decim) / Interp;
            const size_t w1 = ((j + 1) * Decim) / Interp;
            const size_t w2 = ((j + 2) * Decim) / Interp;
            const size_t w3 = ((j + 3) * Decim) / Interp;

            const __m256 c0 =
                phase_accum_avx2(scaled_.data() + a0 * H_, tile + w0 * 8, H_);
            const __m256 c1 =
                phase_accum_avx2(scaled_.data() + a1 * H_, tile + w1 * 8, H_);
            const __m256 c2 =
                phase_accum_avx2(scaled_.data() + a2 * H_, tile + w2 * 8, H_);
            const __m256 c3 =
                phase_accum_avx2(scaled_.data() + a3 * H_, tile + w3 * 8, H_);

            // 4x4 transpose of 64-bit complex elements: out[m] gets the four
            // phases j..j+3 of macroblock mb0+m.
            const __m256 t0 = _mm256_castpd_ps(
                _mm256_unpacklo_pd(_mm256_castps_pd(c0), _mm256_castps_pd(c1)));
            const __m256 t1 = _mm256_castpd_ps(
                _mm256_unpackhi_pd(_mm256_castps_pd(c0), _mm256_castps_pd(c1)));
            const __m256 t2 = _mm256_castpd_ps(
                _mm256_unpacklo_pd(_mm256_castps_pd(c2), _mm256_castps_pd(c3)));
            const __m256 t3 = _mm256_castpd_ps(
                _mm256_unpackhi_pd(_mm256_castps_pd(c2), _mm256_castps_pd(c3)));

            const __m256 o0 = _mm256_permute2f128_ps(t0, t2, 0x20);
            const __m256 o1 = _mm256_permute2f128_ps(t1, t3, 0x20);
            const __m256 o2 = _mm256_permute2f128_ps(t0, t2, 0x31);
            const __m256 o3 = _mm256_permute2f128_ps(t1, t3, 0x31);

            _mm256_storeu_ps(
                reinterpret_cast<float*>(out + (mb0 + 0) * Interp + j), o0);
            _mm256_storeu_ps(
                reinterpret_cast<float*>(out + (mb0 + 1) * Interp + j), o1);
            _mm256_storeu_ps(
                reinterpret_cast<float*>(out + (mb0 + 2) * Interp + j), o2);
            _mm256_storeu_ps(
                reinterpret_cast<float*>(out + (mb0 + 3) * Interp + j), o3);
        }

        // Interp % 4 leftover phases (1 for both 65/48 and 65/32).  Directions
        // whose Interp is a multiple of 4 compile the loop out entirely.
        if constexpr (Interp % 4 != 0) {
            for (; j < Interp; ++j) {
                const size_t arm = (j * Decim) % Interp;
                const size_t w = (j * Decim) / Interp;
                const __m256 acc = phase_accum_avx2(scaled_.data() + arm * H_,
                                                    tile + w * 8, H_);
                alignas(32) float lanes[8];
                _mm256_store_ps(lanes, acc);
                for (size_t m = 0; m < 4; ++m) {
                    out[(mb0 + m) * Interp + j] =
                        gr_complex(lanes[2 * m], lanes[2 * m + 1]);
                }
            }
        }
    }
#endif
};

using TransposedResampler65_48 = TransposedResampler<65, 48>;
using TransposedResampler65_32 = TransposedResampler<65, 32>;
using TransposedResampler48_65 = TransposedResampler<48, 65>;
using TransposedResampler32_65 = TransposedResampler<32, 65>;

} // namespace transposed
} // namespace simd
} // namespace core
} // namespace uwb
} // namespace gr
