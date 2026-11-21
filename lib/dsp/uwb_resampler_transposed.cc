/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Out-of-line implementations of
 * TransposedResampler<Interp, Decim> (class declaration in
 * include/openuwb/dsp/resampler_transposed.h).
 *
 * Explicit instantiations below define the link-time symbols
 * for the four aliases; the header marks them `extern
 * template`.  The AVX-512 8-macroblock microkernel stays in
 * lib/dsp/uwb_resampler_transposed_avx512.cc behind function
 * target attributes.
 */
#include <openuwb/dsp/resampler_transposed.h>

#if UWB_SIMD_LEVEL >= 1
#include <immintrin.h>
#endif

namespace openuwb {
namespace core {
namespace simd {
namespace transposed {

template <uint32_t Interp, uint32_t Decim>
TransposedResampler<Interp, Decim>::TransposedResampler(const float* taps, size_t T)
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
template <uint32_t Interp, uint32_t Decim>
TransposedResampler<Interp, Decim>::TransposedResampler(const std::vector<float>& taps)
    : TransposedResampler(taps.data(), taps.size())
{
}
template <uint32_t Interp, uint32_t Decim>
size_t TransposedResampler<Interp, Decim>::expected_output_length(uint64_t N, size_t T)
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
template <uint32_t Interp, uint32_t Decim>
size_t TransposedResampler<Interp, Decim>::expected_output_length(uint64_t N) const
{
    return expected_output_length(N, T_);
}
template <uint32_t Interp, uint32_t Decim>
size_t TransposedResampler<Interp, Decim>::tap_count() const { return T_; }
template <uint32_t Interp, uint32_t Decim>
size_t TransposedResampler<Interp, Decim>::arm_length() const { return H_; }

/** "auto" (default), "avx2", "avx512", "scalar". */
template <uint32_t Interp, uint32_t Decim>
void TransposedResampler<Interp, Decim>::set_kernel(const std::string& name)
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
template <uint32_t Interp, uint32_t Decim>
const char* TransposedResampler<Interp, Decim>::kernel_name() const
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
template <uint32_t Interp, uint32_t Decim>
size_t TransposedResampler<Interp, Decim>::process_window_sc16(const sc16* in,
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
template <uint32_t Interp, uint32_t Decim>
typename TransposedResampler<Interp, Decim>::Kernel TransposedResampler<Interp, Decim>::resolve_kernel() const
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
template <uint32_t Interp, uint32_t Decim>
void TransposedResampler<Interp, Decim>::ensure_scaled(float scale)
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
template <uint32_t Interp, uint32_t Decim>
void TransposedResampler<Interp, Decim>::scalar_output(const sc16* in, size_t ninput, size_t o,
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
template <uint32_t Interp, uint32_t Decim>
void TransposedResampler<Interp, Decim>::scalar_macroblock(const sc16* in, size_t ninput, size_t mb,
                       gr_complex* out) const
{
    const size_t base = mb * Interp;
    for (size_t j = 0; j < Interp; ++j)
        scalar_output(in, ninput, base + j, out);
}
#if UWB_SIMD_LEVEL >= 1
template <uint32_t Interp, uint32_t Decim>
int32_t TransposedResampler<Interp, Decim>::load_sc16_bits(const sc16* p)
{
    int32_t v;
    std::memcpy(&v, p, sizeof(v));
    return v;
}

/**
 * Same-loop 4-phase tap accumulation over X_tile:
 * breaks the 4-cycle FMA dependency chain across 4 independent accumulators
 * and reuses L1d tile lines across phases j..j+3.
 */
template <uint32_t Interp, uint32_t Decim>
template <size_t H_VAL>
void TransposedResampler<Interp, Decim>::phase_accum_4x_avx2(
    const float* tp0, const float* tp1, const float* tp2, const float* tp3,
    const float* x0, const float* x1, const float* x2, const float* x3,
    size_t h_rt,
    __m256& c0, __m256& c1, __m256& c2, __m256& c3) noexcept
{
    const size_t H = (H_VAL != 0) ? H_VAL : h_rt;
    __m256 a0 = _mm256_setzero_ps();
    __m256 a1 = _mm256_setzero_ps();
    __m256 a2 = _mm256_setzero_ps();
    __m256 a3 = _mm256_setzero_ps();
    for (size_t k = 0; k < H; ++k) {
        const size_t off = k * 8;
        a0 = _mm256_fmadd_ps(_mm256_set1_ps(tp0[k]), _mm256_load_ps(x0 + off), a0);
        a1 = _mm256_fmadd_ps(_mm256_set1_ps(tp1[k]), _mm256_load_ps(x1 + off), a1);
        a2 = _mm256_fmadd_ps(_mm256_set1_ps(tp2[k]), _mm256_load_ps(x2 + off), a2);
        a3 = _mm256_fmadd_ps(_mm256_set1_ps(tp3[k]), _mm256_load_ps(x3 + off), a3);
    }
    c0 = a0;
    c1 = a1;
    c2 = a2;
    c3 = a3;
}

/**
 * Same-loop 8-phase tap accumulation: 8 independent FMA chains saturate
 * dual 256-bit FMA ports and reuse adjacent X_tile slots.
 */
template <uint32_t Interp, uint32_t Decim>
template <size_t H_VAL>
void TransposedResampler<Interp, Decim>::phase_accum_8x_avx2(
    const float* const* tp8, const float* const* x8, size_t h_rt,
    __m256* out8) noexcept
{
    const size_t H = (H_VAL != 0) ? H_VAL : h_rt;
    __m256 a0 = _mm256_setzero_ps(), a1 = _mm256_setzero_ps();
    __m256 a2 = _mm256_setzero_ps(), a3 = _mm256_setzero_ps();
    __m256 a4 = _mm256_setzero_ps(), a5 = _mm256_setzero_ps();
    __m256 a6 = _mm256_setzero_ps(), a7 = _mm256_setzero_ps();
    const float* const tp0 = tp8[0]; const float* const tp1 = tp8[1];
    const float* const tp2 = tp8[2]; const float* const tp3 = tp8[3];
    const float* const tp4 = tp8[4]; const float* const tp5 = tp8[5];
    const float* const tp6 = tp8[6]; const float* const tp7 = tp8[7];
    const float* const x0 = x8[0]; const float* const x1 = x8[1];
    const float* const x2 = x8[2]; const float* const x3 = x8[3];
    const float* const x4 = x8[4]; const float* const x5 = x8[5];
    const float* const x6 = x8[6]; const float* const x7 = x8[7];
    for (size_t k = 0; k < H; ++k) {
        const size_t off = k * 8;
        a0 = _mm256_fmadd_ps(_mm256_set1_ps(tp0[k]), _mm256_load_ps(x0 + off), a0);
        a1 = _mm256_fmadd_ps(_mm256_set1_ps(tp1[k]), _mm256_load_ps(x1 + off), a1);
        a2 = _mm256_fmadd_ps(_mm256_set1_ps(tp2[k]), _mm256_load_ps(x2 + off), a2);
        a3 = _mm256_fmadd_ps(_mm256_set1_ps(tp3[k]), _mm256_load_ps(x3 + off), a3);
        a4 = _mm256_fmadd_ps(_mm256_set1_ps(tp4[k]), _mm256_load_ps(x4 + off), a4);
        a5 = _mm256_fmadd_ps(_mm256_set1_ps(tp5[k]), _mm256_load_ps(x5 + off), a5);
        a6 = _mm256_fmadd_ps(_mm256_set1_ps(tp6[k]), _mm256_load_ps(x6 + off), a6);
        a7 = _mm256_fmadd_ps(_mm256_set1_ps(tp7[k]), _mm256_load_ps(x7 + off), a7);
    }
    out8[0] = a0; out8[1] = a1; out8[2] = a2; out8[3] = a3;
    out8[4] = a4; out8[5] = a5; out8[6] = a6; out8[7] = a7;
}
template <uint32_t Interp, uint32_t Decim>
void TransposedResampler<Interp, Decim>::transpose_4x4_store_avx2(
    __m256 c0, __m256 c1, __m256 c2, __m256 c3,
    gr_complex* out, size_t mb0, size_t j) noexcept
{
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

    _mm256_storeu_ps(reinterpret_cast<float*>(out + (mb0 + 0) * Interp + j), o0);
    _mm256_storeu_ps(reinterpret_cast<float*>(out + (mb0 + 1) * Interp + j), o1);
    _mm256_storeu_ps(reinterpret_cast<float*>(out + (mb0 + 2) * Interp + j), o2);
    _mm256_storeu_ps(reinterpret_cast<float*>(out + (mb0 + 3) * Interp + j), o3);
}

/** 4 macroblocks, all Interp outputs, compile-time H specialization. */
template <uint32_t Interp, uint32_t Decim>
template <size_t H_VAL>
void TransposedResampler<Interp, Decim>::avx2_group_hp(const sc16* in, size_t ninput, size_t mb0,
                   gr_complex* out) noexcept
{
    const size_t H = (H_VAL != 0) ? H_VAL : H_;
    const size_t slots = static_cast<size_t>(Decim) + H - 1;
    float* const tile = tile_.data();

    const int64_t base0 = static_cast<int64_t>(mb0) * Decim -
                          static_cast<int64_t>(H - 1);
    const int64_t max_idx = base0 + 3 * static_cast<int64_t>(Decim) +
                            static_cast<int64_t>(slots);
    if (base0 >= 0 && max_idx <= static_cast<int64_t>(ninput)) {
        const sc16* const p0 = in + base0;
        const sc16* const p1 = p0 + Decim;
        const sc16* const p2 = p0 + 2 * Decim;
        const sc16* const p3 = p0 + 3 * Decim;
        const size_t slots_vec = slots & ~size_t(3);
        for (size_t u = 0; u < slots_vec; u += 4) {
            const __m128i r0 = _mm_loadu_si128(reinterpret_cast<const __m128i*>(p0 + u));
            const __m128i r1 = _mm_loadu_si128(reinterpret_cast<const __m128i*>(p1 + u));
            const __m128i r2 = _mm_loadu_si128(reinterpret_cast<const __m128i*>(p2 + u));
            const __m128i r3 = _mm_loadu_si128(reinterpret_cast<const __m128i*>(p3 + u));

            const __m128i t01_lo = _mm_unpacklo_epi32(r0, r1);
            const __m128i t01_hi = _mm_unpackhi_epi32(r0, r1);
            const __m128i t23_lo = _mm_unpacklo_epi32(r2, r3);
            const __m128i t23_hi = _mm_unpackhi_epi32(r2, r3);

            const __m128i q0 = _mm_unpacklo_epi64(t01_lo, t23_lo);
            const __m128i q1 = _mm_unpackhi_epi64(t01_lo, t23_lo);
            const __m128i q2 = _mm_unpacklo_epi64(t01_hi, t23_hi);
            const __m128i q3 = _mm_unpackhi_epi64(t01_hi, t23_hi);

            _mm256_store_ps(tile + (u + 0) * 8, _mm256_cvtepi32_ps(_mm256_cvtepi16_epi32(q0)));
            _mm256_store_ps(tile + (u + 1) * 8, _mm256_cvtepi32_ps(_mm256_cvtepi16_epi32(q1)));
            _mm256_store_ps(tile + (u + 2) * 8, _mm256_cvtepi32_ps(_mm256_cvtepi16_epi32(q2)));
            _mm256_store_ps(tile + (u + 3) * 8, _mm256_cvtepi32_ps(_mm256_cvtepi16_epi32(q3)));
        }
        for (size_t u = slots_vec; u < slots; ++u) {
            const __m128i q = _mm_setr_epi32(
                load_sc16_bits(p0 + u), load_sc16_bits(p1 + u),
                load_sc16_bits(p2 + u), load_sc16_bits(p3 + u));
            _mm256_store_ps(tile + u * 8, _mm256_cvtepi32_ps(_mm256_cvtepi16_epi32(q)));
        }
    } else {
        for (size_t u = 0; u < slots; ++u) {
            int32_t v[4];
            for (size_t m = 0; m < 4; ++m) {
                const int64_t idx = base0 + static_cast<int64_t>(m) * Decim +
                                    static_cast<int64_t>(u);
                v[m] = (idx >= 0 && idx < static_cast<int64_t>(ninput))
                           ? load_sc16_bits(in + idx)
                           : 0;
            }
            const __m128i q = _mm_setr_epi32(v[0], v[1], v[2], v[3]);
            _mm256_store_ps(tile + u * 8,
                            _mm256_cvtepi32_ps(_mm256_cvtepi16_epi32(q)));
        }
    }

    const float* const taps_base = scaled_.data();
    size_t j = 0;
    for (; j + 8 <= Interp; j += 8) {
        const float* tp8[8];
        const float* x8[8];
        for (size_t p = 0; p < 8; ++p) {
            const size_t prod = (j + p) * Decim;
            tp8[p] = taps_base + (prod % Interp) * H;
            x8[p] = tile + (prod / Interp) * 8;
        }
        __m256 out8[8];
        phase_accum_8x_avx2<H_VAL>(tp8, x8, H, out8);
        transpose_4x4_store_avx2(out8[0], out8[1], out8[2], out8[3], out, mb0, j);
        transpose_4x4_store_avx2(out8[4], out8[5], out8[6], out8[7], out, mb0, j + 4);
    }
    for (; j + 4 <= Interp; j += 4) {
        const uint32_t a0 = (j * Decim) % Interp;
        const uint32_t a1 = ((j + 1) * Decim) % Interp;
        const uint32_t a2 = ((j + 2) * Decim) % Interp;
        const uint32_t a3 = ((j + 3) * Decim) % Interp;
        const size_t w0 = (j * Decim) / Interp;
        const size_t w1 = ((j + 1) * Decim) / Interp;
        const size_t w2 = ((j + 2) * Decim) / Interp;
        const size_t w3 = ((j + 3) * Decim) / Interp;

        __m256 c0, c1, c2, c3;
        phase_accum_4x_avx2<H_VAL>(
            taps_base + a0 * H, taps_base + a1 * H,
            taps_base + a2 * H, taps_base + a3 * H,
            tile + w0 * 8, tile + w1 * 8, tile + w2 * 8, tile + w3 * 8,
            H, c0, c1, c2, c3);
        transpose_4x4_store_avx2(c0, c1, c2, c3, out, mb0, j);
    }

    if constexpr (Interp % 4 != 0) {
        for (; j < Interp; ++j) {
            const size_t arm = (j * Decim) % Interp;
            const size_t w = (j * Decim) / Interp;
            const float* const tp = taps_base + arm * H;
            const float* const xp = tile + w * 8;
            __m256 acc = _mm256_setzero_ps();
            for (size_t k = 0; k < H; ++k) {
                acc = _mm256_fmadd_ps(_mm256_set1_ps(tp[k]),
                                      _mm256_load_ps(xp + k * 8), acc);
            }
            alignas(32) float lanes[8];
            _mm256_store_ps(lanes, acc);
            for (size_t m = 0; m < 4; ++m) {
                out[(mb0 + m) * Interp + j] =
                    gr_complex(lanes[2 * m], lanes[2 * m + 1]);
            }
        }
    }
}
template <uint32_t Interp, uint32_t Decim>
void TransposedResampler<Interp, Decim>::avx2_group(const sc16* in, size_t ninput, size_t mb0,
                gr_complex* out)
{
    if (H_ == 16) avx2_group_hp<16>(in, ninput, mb0, out);
    else if (H_ == 21) avx2_group_hp<21>(in, ninput, mb0, out);
    else if (H_ == 32) avx2_group_hp<32>(in, ninput, mb0, out);
    else if (H_ == 42) avx2_group_hp<42>(in, ninput, mb0, out);
    else avx2_group_hp<0>(in, ninput, mb0, out);
}
#endif

template class TransposedResampler<65, 48>;
template class TransposedResampler<65, 32>;
template class TransposedResampler<48, 65>;
template class TransposedResampler<32, 65>;

} // namespace transposed
} // namespace simd
} // namespace core
} // namespace openuwb
