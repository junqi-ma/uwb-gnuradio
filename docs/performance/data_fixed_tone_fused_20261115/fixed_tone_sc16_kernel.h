#pragma once
// Standalone prototype for the OpenCode handoff. No pipeline/thread ownership.
// Packed IQ input: [I0,Q0,I1,Q1,...]. Exact alias (in-place) or non-overlapping
// buffers are supported; partial overlap is not supported.
#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#if defined(__AVX2__)
#include <immintrin.h>
#endif
namespace fixed_tone_prototype {
class Kernel {
public:
    static constexpr size_t period = 288;
    static constexpr double omega = -2.0 * 3.14159265358979323846 * 91.0 / 288.0;
    Kernel() {
        for (size_t k = 0; k < period; ++k) {
            cos_[k] = std::cos(omega * k);
            sin_[k] = std::sin(omega * k);
        }
        iq_.fill(0);
    }
    // Output coefficient uses the first training sample as its phase origin.
    bool estimate(const int16_t* src, size_t n, std::complex<double>& out) const {
        if (!src || n == 0 || n > period * size_t(65535)) return false;
        alignas(32) std::array<int32_t, 2 * period> fold{};
        size_t j = 0;
        for (; j + period <= n; j += period)
            for (size_t k = 0; k < 2 * period; ++k) fold[k] += src[2 * j + k];
        for (size_t k = 0; k < 2 * (n - j); ++k) fold[k] += src[2 * j + k];
        double re = 0, im = 0;
        for (size_t k = 0; k < period; ++k) {
            re += fold[2*k] * cos_[k] + fold[2*k+1] * sin_[k];
            im += fold[2*k+1] * cos_[k] - fold[2*k] * sin_[k];
        }
        out = {re / double(n), im / double(n)};
        return true;
    }
    // Failed update leaves the previous LUT untouched. Round ties to even,
    // independently of the process rounding mode; no clipped tone model.
    bool set_coefficient(std::complex<double> c) {
        if (!std::isfinite(c.real()) || !std::isfinite(c.imag())) return false;
        alignas(32) std::array<int16_t, 2 * period> next;
        for (size_t k = 0; k < period; ++k) {
            double re = c.real()*cos_[k] - c.imag()*sin_[k];
            double im = c.real()*sin_[k] + c.imag()*cos_[k];
            if (!quantize(re, next[2*k]) || !quantize(im, next[2*k+1])) return false;
        }
        iq_ = next;
        return true;
    }
    // Single load/subtract/store pass. When a raw queue must be copied into
    // pipeline storage, use copy_cancel instead of memcpy + cancel_in_place.
    void copy_cancel(const int16_t* src, int16_t* dst, size_t n,
                     size_t phase = 0) const {
        phase %= period;
        size_t a = 0;
        while (a < n) {
            const size_t count = std::min(n-a, period-phase);
            size_t j = 0;
#if defined(__AVX2__)
            for (; j + 8 <= count; j += 8) {
                const __m256i x = _mm256_loadu_si256(
                    reinterpret_cast<const __m256i*>(src + 2*(a+j)));
                const __m256i tone = _mm256_loadu_si256(
                    reinterpret_cast<const __m256i*>(iq_.data() + 2*(phase+j)));
                _mm256_storeu_si256(reinterpret_cast<__m256i*>(dst + 2*(a+j)),
                                   _mm256_subs_epi16(x, tone));
            }
#endif
            for (; j < count; ++j)
                for (size_t q = 0; q < 2; ++q) {
                    const int v = int(src[2*(a+j)+q]) - int(iq_[2*(phase+j)+q]);
                    dst[2*(a+j)+q] = int16_t(std::clamp(v, -32768, 32767));
                }
            a += count;
            phase = 0;
        }
    }
    void cancel_in_place(int16_t* data, size_t n, size_t phase = 0) const {
        copy_cancel(data, data, n, phase);
    }
private:
    static bool quantize(double v, int16_t& out) {
        if (!std::isfinite(v) || v < -32769.0 || v > 32768.0) return false;
        const double lo = std::floor(v), frac = v-lo;
        const double z = lo + ((frac > 0.5 ||
                               (frac == 0.5 && std::fmod(lo, 2.0) != 0.0)) ? 1.0 : 0.0);
        if (z < -32768.0 || z > 32767.0) return false;
        out = int16_t(z);
        return true;
    }
    std::array<double, period> cos_, sin_;
    alignas(32) std::array<int16_t, 2 * period> iq_;
};
} // namespace fixed_tone_prototype
