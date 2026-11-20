/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Per-window CW estimate / subtract.  Aligns with
 * testdata/cancel_capture_tone.py:
 *
 *   coarse: Hanning FFT peak in [f_lo, f_hi], parabolic interpolate
 *   refine: |<x, exp(j 2π f n / fs)>| grid search
 *   subtract: c = <x, s> / N, y = x − c s, n from the window start
 *
 * capture.iq is concatenated non-contiguous windows, so n is never the
 * absolute window_start unless a later path proves phase continuity.
 */

#pragma once

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <vector>

namespace gr {
namespace uwb {
namespace core {

struct ToneEstimate {
    double baseband_hz = 0.0;
    double peak_mag = 0.0;
    double coarse_hz = 0.0;
};

struct ToneSubtractResult {
    std::complex<double> coef{ 0.0, 0.0 };
    double power_before = 0.0;
    double power_after = 0.0;
    double bin_before = 0.0;
    double bin_after = 0.0;
    uint64_t clip_count = 0;
};

inline size_t next_pow2_at_least(size_t n)
{
    size_t p = 1;
    while (p < n)
        p <<= 1;
    return p;
}

// s[n+1] = s[n] * exp(j w).  Refresh every 1024 samples so |s| cannot
// walk off the unit circle on a 590 µs (≈4.3e5) window.
constexpr size_t kTonePhasorRefresh = 1024;

inline std::complex<double> tone_phasor(double w, size_t k)
{
    const double ph = w * static_cast<double>(k);
    return { std::cos(ph), std::sin(ph) };
}

inline std::complex<double> tone_step(double w)
{
    return { std::cos(w), std::sin(w) };
}

template <typename Sample>
inline std::complex<double> correlate_cis(const Sample* x, size_t n, double w)
{
    const auto step = tone_step(w);
    auto s = tone_phasor(w, 0);
    std::complex<double> acc(0.0, 0.0);
    for (size_t k = 0; k < n; ++k) {
        if ((k & (kTonePhasorRefresh - 1)) == 0)
            s = tone_phasor(w, k);
        acc += std::complex<double>(static_cast<double>(x[k].real()),
                                    static_cast<double>(x[k].imag())) *
               s;
        s *= step;
    }
    return acc;
}

inline std::complex<double> correlate_cis_sc16(const int16_t* x,
                                              size_t n,
                                              double w,
                                              double* power)
{
    const auto step = tone_step(w);
    auto s = tone_phasor(w, 0);
    std::complex<double> acc(0.0, 0.0);
    double p0 = 0.0;
    for (size_t k = 0; k < n; ++k) {
        if ((k & (kTonePhasorRefresh - 1)) == 0)
            s = tone_phasor(w, k);
        const double re = static_cast<double>(x[2 * k]);
        const double im = static_cast<double>(x[2 * k + 1]);
        acc += std::complex<double>(re, im) * s;
        p0 += re * re + im * im;
        s *= step;
    }
    if (power)
        *power = p0;
    return acc;
}

inline void fft_radix2(std::vector<std::complex<double>>& a, bool inverse)
{
    const size_t n = a.size();
    if (n == 0 || (n & (n - 1)) != 0)
        throw std::invalid_argument("fft_radix2: length must be a power of 2");
    for (size_t i = 1, j = 0; i < n; ++i) {
        size_t bit = n >> 1;
        for (; j & bit; bit >>= 1)
            j ^= bit;
        j ^= bit;
        if (i < j)
            std::swap(a[i], a[j]);
    }
    const double sign = inverse ? 1.0 : -1.0;
    for (size_t len = 2; len <= n; len <<= 1) {
        const double ang = sign * 2.0 * M_PI / static_cast<double>(len);
        const std::complex<double> wlen(std::cos(ang), std::sin(ang));
        for (size_t i = 0; i < n; i += len) {
            std::complex<double> w(1.0, 0.0);
            for (size_t j = 0; j < len / 2; ++j) {
                const auto u = a[i + j];
                const auto v = a[i + j + len / 2] * w;
                a[i + j] = u + v;
                a[i + j + len / 2] = u - v;
                w *= wlen;
            }
        }
    }
    if (inverse) {
        const double s = 1.0 / static_cast<double>(n);
        for (auto& z : a)
            z *= s;
    }
}

template <typename Sample>
inline double fft_peak_hz(const Sample* x,
                          size_t n,
                          double fs,
                          double f_lo,
                          double f_hi,
                          double* peak_mag)
{
    if (x == nullptr || n == 0 || fs <= 0.0)
        throw std::invalid_argument("fft_peak_hz: empty input");
    const size_t nfft = next_pow2_at_least(std::max(n, size_t(4096)));
    std::vector<std::complex<double>> spec(nfft, { 0.0, 0.0 });
    const double denom = (n > 1) ? static_cast<double>(n - 1) : 1.0;
    for (size_t i = 0; i < n; ++i) {
        const double w =
            0.5 * (1.0 - std::cos(2.0 * M_PI * static_cast<double>(i) / denom));
        spec[i] = std::complex<double>(static_cast<double>(x[i].real()),
                                       static_cast<double>(x[i].imag())) *
                  w;
    }
    fft_radix2(spec, false);

    // fftshift into a linear frequency axis: k=0 → −fs/2.
    std::vector<double> mag(nfft);
    for (size_t k = 0; k < nfft; ++k) {
        const size_t src = (k + nfft / 2) % nfft;
        mag[k] = std::abs(spec[src]);
    }
    const double df = fs / static_cast<double>(nfft);
    const auto freq_of = [nfft, df](size_t k) {
        return (static_cast<double>(k) - 0.5 * static_cast<double>(nfft)) * df;
    };

    const bool band_limited = (f_lo < f_hi);
    size_t best = 0;
    double best_m = -1.0;
    for (size_t k = 0; k < nfft; ++k) {
        if (band_limited) {
            const double f = freq_of(k);
            if (f < f_lo || f > f_hi)
                continue;
        }
        if (mag[k] > best_m) {
            best_m = mag[k];
            best = k;
        }
    }
    if (best_m < 0.0) {
        for (size_t k = 0; k < nfft; ++k) {
            if (mag[k] > best_m) {
                best_m = mag[k];
                best = k;
            }
        }
    }
    double f = freq_of(best);
    if (best > 0 && best + 1 < nfft && mag[best - 1] > 0.0 &&
        mag[best + 1] > 0.0) {
        const double a = mag[best - 1];
        const double b = mag[best];
        const double c = mag[best + 1];
        const double den = a - 2.0 * b + c;
        const double delta = (den != 0.0) ? 0.5 * (a - c) / den : 0.0;
        f += delta * df;
    }
    if (peak_mag)
        *peak_mag = best_m;
    return f;
}

template <typename Sample>
inline double refine_tone_freq(const Sample* x,
                               size_t n,
                               double fs,
                               double f0,
                               double span_hz,
                               int steps = 5)
{
    if (x == nullptr || n == 0 || fs <= 0.0)
        return f0;
    double best_f = f0;
    double best = 0.0;
    double span = span_hz;
    for (int s = 0; s < steps; ++s) {
        for (int i = 0; i < 21; ++i) {
            const double f =
                best_f - span + (2.0 * span) * (static_cast<double>(i) / 20.0);
            const double w = -2.0 * M_PI * f / fs;
            const double m = std::abs(correlate_cis(x, n, w));
            if (m > best) {
                best = m;
                best_f = f;
            }
        }
        span *= 0.25;
    }
    return best_f;
}

template <typename Sample>
inline ToneEstimate estimate_tone(const Sample* x,
                                  size_t n,
                                  double fs,
                                  double f_lo,
                                  double f_hi)
{
    ToneEstimate e;
    e.coarse_hz = fft_peak_hz(x, n, fs, f_lo, f_hi, &e.peak_mag);
    const double span = std::max(200.0, fs / static_cast<double>(n) * 8.0);
    e.baseband_hz = refine_tone_freq(x, n, fs, e.coarse_hz, span);
    return e;
}

template <typename Sample>
inline ToneSubtractResult fit_tone(const Sample* x, size_t n, double fs,
                                   double f_hz)
{
    ToneSubtractResult r;
    if (x == nullptr || n == 0)
        return r;
    double p0 = 0.0;
    const double w = 2.0 * M_PI * f_hz / fs;
    // ⟨x, exp(jωn)⟩ = correlate with exp(−jωn)
    const auto acc = correlate_cis(x, n, -w);
    for (size_t k = 0; k < n; ++k) {
        const double re = static_cast<double>(x[k].real());
        const double im = static_cast<double>(x[k].imag());
        p0 += re * re + im * im;
    }
    r.coef = acc / static_cast<double>(n);
    r.power_before = p0 / static_cast<double>(n);
    r.bin_before = std::abs(r.coef);
    return r;
}

template <typename Sample>
inline ToneSubtractResult subtract_tone(Sample* y,
                                        const Sample* x,
                                        size_t n,
                                        double fs,
                                        double f_hz)
{
    ToneSubtractResult r = fit_tone(x, n, fs, f_hz);
    if (x == nullptr || y == nullptr || n == 0)
        return r;
    double p1 = 0.0;
    std::complex<double> resid(0.0, 0.0);
    const double w = 2.0 * M_PI * f_hz / fs;
    const auto step = tone_step(w);
    auto s = tone_phasor(w, 0);
    for (size_t k = 0; k < n; ++k) {
        if ((k & (kTonePhasorRefresh - 1)) == 0)
            s = tone_phasor(w, k);
        const std::complex<double> z(static_cast<double>(x[k].real()),
                                     static_cast<double>(x[k].imag()));
        const std::complex<double> yn = z - r.coef * s;
        y[k] = Sample(static_cast<decltype(x[0].real())>(yn.real()),
                      static_cast<decltype(x[0].imag())>(yn.imag()));
        p1 += std::norm(yn);
        resid += yn * std::conj(s);
        s *= step;
    }
    r.power_after = p1 / static_cast<double>(n);
    r.bin_after = std::abs(resid / static_cast<double>(n));
    return r;
}

inline int16_t clip_i16(double v, uint64_t* clips)
{
    const double r = std::rint(v);
    if (r > 32767.0) {
        if (clips)
            ++(*clips);
        return 32767;
    }
    if (r < -32768.0) {
        if (clips)
            ++(*clips);
        return -32768;
    }
    return static_cast<int16_t>(r);
}

inline ToneSubtractResult subtract_tone_sc16(int16_t* y_interleaved,
                                             const int16_t* x_interleaved,
                                             size_t n,
                                             double fs,
                                             double f_hz)
{
    ToneSubtractResult r;
    if (x_interleaved == nullptr || y_interleaved == nullptr || n == 0)
        return r;
    const double w = 2.0 * M_PI * f_hz / fs;
    double p0 = 0.0;
    const auto acc = correlate_cis_sc16(x_interleaved, n, -w, &p0);
    r.coef = acc / static_cast<double>(n);
    r.power_before = p0 / static_cast<double>(n);
    r.bin_before = std::abs(r.coef);

    double p1 = 0.0;
    std::complex<double> resid(0.0, 0.0);
    r.clip_count = 0;
    const auto step = tone_step(w);
    auto s = tone_phasor(w, 0);
    for (size_t k = 0; k < n; ++k) {
        if ((k & (kTonePhasorRefresh - 1)) == 0)
            s = tone_phasor(w, k);
        const std::complex<double> z(
            static_cast<double>(x_interleaved[2 * k]),
            static_cast<double>(x_interleaved[2 * k + 1]));
        const std::complex<double> yn = z - r.coef * s;
        y_interleaved[2 * k] = clip_i16(yn.real(), &r.clip_count);
        y_interleaved[2 * k + 1] = clip_i16(yn.imag(), &r.clip_count);
        p1 += std::norm(yn);
        resid += yn * std::conj(s);
        s *= step;
    }
    r.power_after = p1 / static_cast<double>(n);
    r.bin_after = std::abs(resid / static_cast<double>(n));
    return r;
}

inline void sc16_to_cf32(const int16_t* interleaved,
                         size_t n,
                         std::complex<float>* out)
{
    for (size_t i = 0; i < n; ++i) {
        out[i] = std::complex<float>(
            static_cast<float>(interleaved[2 * i]),
            static_cast<float>(interleaved[2 * i + 1]));
    }
}

} // namespace core
} // namespace uwb
} // namespace gr
