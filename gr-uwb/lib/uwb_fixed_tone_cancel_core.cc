/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Fixed-frequency CW cancellation core implementation.
 */

#include <gnuradio/uwb/uwb_fixed_tone_cancel_core.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <sstream>

#if defined(__x86_64__) || defined(__i386__)
#include <immintrin.h>
#define UWB_FIXED_TONE_X86 1
#else
#define UWB_FIXED_TONE_X86 0
#endif

namespace gr {
namespace uwb {
namespace core {

namespace {

constexpr size_t kMaxPeriodSamples = 4096;
// int32 fold safety: 65535 * 32768 < INT32_MAX.
constexpr size_t kMaxFoldPerPosition = 65535;
// Relative tolerance when testing f0/fs * P for an integer number of cycles.
constexpr double kPeriodTolerance = 1e-9;

// out = saturate_int16(in - tone).  `tone` points at n contiguous interleaved
// I/Q components (a LUT segment or a cross-fade segment).  src and dst may be
// the same buffer (in-place mode), so no restrict qualifier is used: every
// 32-byte block is loaded before it is stored.
void subtract_sat_scalar(const int16_t* src, int16_t* dst, size_t n, const int16_t* tone)
{
    for (size_t k = 0; k < 2 * n; ++k) {
        const int v = int(src[k]) - int(tone[k]);
        dst[k] = static_cast<int16_t>(v < -32768 ? -32768 : (v > 32767 ? 32767 : v));
    }
}

#if UWB_FIXED_TONE_X86
__attribute__((target("avx2"))) void subtract_sat_avx2(const int16_t* src,
                                                       int16_t* dst,
                                                       size_t n,
                                                       const int16_t* tone)
{
    size_t k = 0;
    for (; k + 16 <= 2 * n; k += 16) {
        const __m256i x = _mm256_loadu_si256(reinterpret_cast<const __m256i*>(src + k));
        const __m256i t = _mm256_loadu_si256(reinterpret_cast<const __m256i*>(tone + k));
        _mm256_storeu_si256(reinterpret_cast<__m256i*>(dst + k), _mm256_subs_epi16(x, t));
    }
    for (; k < 2 * n; ++k) {
        const int v = int(src[k]) - int(tone[k]);
        dst[k] = static_cast<int16_t>(v < -32768 ? -32768 : (v > 32767 ? 32767 : v));
    }
}
#endif

using SubtractSatFn = void (*)(const int16_t*, int16_t*, size_t, const int16_t*);

SubtractSatFn scalar_kernel()
{
    return &subtract_sat_scalar;
}

SubtractSatFn avx2_kernel()
{
#if UWB_FIXED_TONE_X86
    return &subtract_sat_avx2;
#else
    return &subtract_sat_scalar;
#endif
}

bool runtime_has_avx2()
{
#if UWB_FIXED_TONE_X86 && (defined(__GNUC__) || defined(__clang__))
    __builtin_cpu_init();
    return __builtin_cpu_supports("avx2") != 0;
#else
    return false;
#endif
}

} // namespace

// ---------------------------------------------------------------------------
// Construction / configuration
// ---------------------------------------------------------------------------
FixedToneCancelCore::FixedToneCancelCore() = default;
FixedToneCancelCore::~FixedToneCancelCore() = default;

bool FixedToneCancelCore::find_exact_period(double sample_rate,
                                            double tone_hz,
                                            size_t max_period,
                                            size_t* period,
                                            int* cycles,
                                            double* omega,
                                            std::string* error)
{
    auto fail = [&](const std::string& why) {
        if (error) {
            *error = why;
        }
        return false;
    };
    if (!(sample_rate > 0.0) || !std::isfinite(sample_rate)) {
        return fail("sample rate must be finite and positive");
    }
    if (!std::isfinite(tone_hz)) {
        return fail("tone frequency must be finite");
    }
    if (std::abs(tone_hz) >= 0.5 * sample_rate) {
        return fail("tone frequency must satisfy |f0| < fs/2");
    }
    if (max_period < 1) {
        return fail("max period must be >= 1");
    }
    // f0/fs * P must be an integer number of cycles.  P == 288 with
    // f0/fs == -91/288 is the validated configuration; anything without a
    // short exact period is rejected rather than approximated.
    for (size_t p = 1; p <= max_period; ++p) {
        const double x = tone_hz * static_cast<double>(p) / sample_rate;
        const double nearest = std::round(x);
        const double tol = kPeriodTolerance * std::max(1.0, std::abs(x));
        if (nearest == 0.0 && std::abs(x) > 0.0) {
            continue; // 0 cycles only matches the degenerate DC case
        }
        if (std::abs(x - nearest) <= tol) {
            if (period) {
                *period = p;
            }
            if (cycles) {
                *cycles = static_cast<int>(nearest);
            }
            if (omega) {
                *omega = 2.0 * M_PI * nearest / static_cast<double>(p);
            }
            if (error) {
                error->clear();
            }
            return true;
        }
    }
    std::ostringstream os;
    os << "no exact tone period <= " << max_period << " samples for fs=" << sample_rate
       << " Hz and f0=" << tone_hz << " Hz (f0/fs="
       << (tone_hz / sample_rate)
       << "); this build supports only configurations with a short exact period";
    return fail(os.str());
}

bool FixedToneCancelCore::prepare(const FixedToneCancelConfig& cfg, std::string* error)
{
    prepared_ = false;
    clear_state();
    cfg_ = cfg;
    avx2_available_ = runtime_has_avx2();

    if (!cfg_.enable) {
        // Pass-through mode: no table is needed, but the configuration echo
        // still carries the requested values so a diagnostic can print them.
        prepared_ = true;
        if (error) {
            error->clear();
        }
        return true;
    }

    std::string why;
    size_t period = 0;
    int cycles = 0;
    double omega = 0.0;
    if (!find_exact_period(cfg_.sample_rate,
                           cfg_.tone_frequency_hz,
                           kMaxPeriodSamples,
                           &period,
                           &cycles,
                           &omega,
                           &why)) {
        if (error) {
            *error = why;
        }
        return false;
    }

    if (!(cfg_.period_s > 0.0) || !std::isfinite(cfg_.period_s)) {
        if (error) {
            *error = "period_s must be finite and positive";
        }
        return false;
    }
    if (!(cfg_.training_s > 0.0) || !std::isfinite(cfg_.training_s)) {
        if (error) {
            *error = "training_s must be finite and positive";
        }
        return false;
    }
    if (!(cfg_.training_s < cfg_.period_s)) {
        if (error) {
            *error = "training_s must be shorter than period_s";
        }
        return false;
    }

    const double fs = cfg_.sample_rate;
    const int64_t period_length =
        static_cast<int64_t>(std::llround(cfg_.period_s * fs));
    const int64_t training_length =
        static_cast<int64_t>(std::llround(cfg_.training_s * fs));
    if (period_length < static_cast<int64_t>(period)) {
        if (error) {
            *error = "period_s is shorter than one tone period";
        }
        return false;
    }
    if (training_length <= 0) {
        if (error) {
            *error = "training_s rounds to zero samples";
        }
        return false;
    }
    if (training_length > period_length) {
        if (error) {
            *error = "training window must fit inside one period";
        }
        return false;
    }
    const size_t fold_per_position =
        static_cast<size_t>((training_length + static_cast<int64_t>(period) - 1) /
                            static_cast<int64_t>(period));
    if (fold_per_position > kMaxFoldPerPosition) {
        std::ostringstream os;
        os << "training window needs " << fold_per_position
           << " samples per folded position (int32 fold limit is " << kMaxFoldPerPosition
           << ")";
        if (error) {
            *error = os.str();
        }
        return false;
    }
    if (cfg_.crossfade_samples > static_cast<size_t>(period_length)) {
        if (error) {
            *error = "crossfade_samples must not exceed one period";
        }
        return false;
    }

    period_ = period;
    period_cycles_ = cycles;
    omega_ = omega;
    period_length_ = period_length;
    training_length_ = static_cast<size_t>(training_length);
    crossfade_length_ = cfg_.crossfade_samples;
    max_fold_per_position_ = fold_per_position;

    // One-time unit phasor base.  Every model update only rotates/scales these
    // cached values; no sin/cos runs on the periodic update path.
    base_cos_.resize(period_);
    base_sin_.resize(period_);
    for (size_t k = 0; k < period_; ++k) {
        const double ph = omega_ * static_cast<double>(k);
        base_cos_[k] = std::cos(ph);
        base_sin_[k] = std::sin(ph);
    }

    fold_.assign(2 * period_, 0);
    fold_scratch_.clear();
    lut_.assign(2 * period_, 0);
    crossfade_tone_.assign(2 * crossfade_length_, 0);
    crossfade_active_ = 0;
    crossfade_epoch_ = 0;
    model_ = FixedToneCancelModel{};

    prepared_ = true;
    if (error) {
        error->clear();
    }
    return true;
}

void FixedToneCancelCore::clear_state()
{
    training_active_ = false;
    training_count_ = 0;
    fold_pos_ = 0;
    training_epoch_ = 0;
    std::fill(fold_.begin(), fold_.end(), 0);
    std::fill(lut_.begin(), lut_.end(), 0);
    std::fill(crossfade_tone_.begin(), crossfade_tone_.end(), 0);
    crossfade_active_ = 0;
    crossfade_epoch_ = 0;
    model_ = FixedToneCancelModel{};
}

void FixedToneCancelCore::reset()
{
    clear_state();
}

void FixedToneCancelCore::invalidate_model()
{
    model_.valid = false;
    crossfade_active_ = 0;
}

// ---------------------------------------------------------------------------
// Training
// ---------------------------------------------------------------------------
bool FixedToneCancelCore::begin_training(int64_t epoch, std::string* error)
{
    if (!enabled()) {
        if (error) {
            *error = "core is not prepared/enabled";
        }
        return false;
    }
    std::fill(fold_.begin(), fold_.end(), 0);
    training_count_ = 0;
    fold_pos_ = 0;
    training_epoch_ = epoch;
    training_active_ = true;
    if (error) {
        error->clear();
    }
    return true;
}

bool FixedToneCancelCore::add_training_samples(const int16_t* iq,
                                               size_t n,
                                               std::string* error)
{
    if (!training_active_) {
        if (error) {
            *error = "no training window is open";
        }
        return false;
    }
    if (n == 0) {
        return true;
    }
    if (iq == nullptr) {
        if (error) {
            *error = "null training samples";
        }
        return false;
    }
    if (training_count_ + n > training_length_) {
        if (error) {
            *error = "training window overflow (more than training_s worth of samples)";
        }
        return false;
    }
    // Exact int32 fold: same residue position means the same unit phasor, so
    // the per-position sum is algebraically the full-window projection.
    // Whole periods accumulate contiguously (simd friendly); only the partial
    // period at the start/end of a chunk uses the rotating index.
    const size_t period = period_;
    int32_t* fold = fold_.data();
    size_t pos = fold_pos_;
    size_t k = 0;
    if (pos != 0) {
        const size_t cnt = std::min(n, period - pos);
        for (size_t j = 0; j < cnt; ++j) {
            fold[2 * (pos + j)] += static_cast<int32_t>(iq[2 * j]);
            fold[2 * (pos + j) + 1] += static_cast<int32_t>(iq[2 * j + 1]);
        }
        k = cnt;
        pos += cnt;
        if (pos == period) {
            pos = 0;
        }
    }
    for (; k + period <= n; k += period) {
        const int16_t* src = iq + 2 * k;
        for (size_t j = 0; j < 2 * period; ++j) {
            fold[j] += static_cast<int32_t>(src[j]);
        }
    }
    for (; k < n; ++k, ++pos) {
        fold[2 * pos] += static_cast<int32_t>(iq[2 * k]);
        fold[2 * pos + 1] += static_cast<int32_t>(iq[2 * k + 1]);
    }
    fold_pos_ = pos;
    training_count_ += n;
    if (error) {
        error->clear();
    }
    return true;
}

bool FixedToneCancelCore::quantize_tone(double value, int16_t* out)
{
    if (!out || !std::isfinite(value)) {
        return false;
    }
    // Reject before rounding: a model component that does not fit int16 must
    // never be clipped and then called a correct cancellation.
    if (value < -32769.0 || value > 32768.0) {
        return false;
    }
    const double lo = std::floor(value);
    const double frac = value - lo;
    // Nearest-even, independent of the process rounding mode (ties at .5 go to
    // the even neighbour), so the LUT does not follow fesetround().
    const double z =
        lo + ((frac > 0.5 || (frac == 0.5 && std::fmod(lo, 2.0) != 0.0)) ? 1.0 : 0.0);
    if (z < -32768.0 || z > 32767.0) {
        return false;
    }
    *out = static_cast<int16_t>(z);
    return true;
}

FixedToneTrainingStatus FixedToneCancelCore::finish_training(std::string* error)
{
    if (!training_active_) {
        if (error) {
            *error = "no training window is open";
        }
        return FixedToneTrainingStatus::Incomplete;
    }
    if (training_count_ < training_length_) {
        if (error) {
            *error = "training window incomplete";
        }
        return FixedToneTrainingStatus::Incomplete;
    }

    double re = 0.0;
    double im = 0.0;
    for (size_t k = 0; k < period_; ++k) {
        const double fi = static_cast<double>(fold_[2 * k]);
        const double fq = static_cast<double>(fold_[2 * k + 1]);
        re += fi * base_cos_[k] + fq * base_sin_[k];
        im += fq * base_cos_[k] - fi * base_sin_[k];
    }
    const double inv_n = 1.0 / static_cast<double>(training_count_);
    const std::complex<double> c(re * inv_n, im * inv_n);
    if (!std::isfinite(c.real()) || !std::isfinite(c.imag())) {
        if (error) {
            *error = "projected coefficient is not finite";
        }
        return FixedToneTrainingStatus::InvalidModel;
    }

    // Build the new tone table first; the committed LUT is only replaced once
    // every component is representable.
    std::vector<int16_t> next(2 * period_);
    for (size_t k = 0; k < period_; ++k) {
        const double tr = c.real() * base_cos_[k] - c.imag() * base_sin_[k];
        const double ti = c.real() * base_sin_[k] + c.imag() * base_cos_[k];
        if (!quantize_tone(tr, &next[2 * k]) || !quantize_tone(ti, &next[2 * k + 1])) {
            if (error) {
                *error = "tone model component does not fit int16";
            }
            return FixedToneTrainingStatus::InvalidModel;
        }
    }

    // Cross-fade table from the still-valid previous model to the new one.
    // Both tones are evaluated at the same absolute sample index, so the blend
    // never mixes different phase origins.
    size_t blend_n = 0;
    std::vector<int16_t> blend;
    if (model_.valid && crossfade_length_ > 0) {
        blend_n = crossfade_length_;
        blend.assign(2 * blend_n, 0);
        for (size_t j = 0; j < blend_n; ++j) {
            const double a = static_cast<double>(j + 1) / static_cast<double>(blend_n);
            int64_t rel = training_epoch_ + static_cast<int64_t>(j) - model_.epoch;
            const int64_t m = static_cast<int64_t>(period_);
            rel = ((rel % m) + m) % m;
            const size_t idx_old = static_cast<size_t>(rel);
            const size_t idx_new = j % period_;
            const double br = (1.0 - a) * static_cast<double>(lut_[2 * idx_old]) +
                              a * static_cast<double>(next[2 * idx_new]);
            const double bi = (1.0 - a) * static_cast<double>(lut_[2 * idx_old + 1]) +
                              a * static_cast<double>(next[2 * idx_new + 1]);
            if (!quantize_tone(br, &blend[2 * j]) ||
                !quantize_tone(bi, &blend[2 * j + 1])) {
                // Not reachable for representable endpoints (a convex
                // combination), but a blend must never be clipped: degrade to
                // a hard switch instead.
                blend_n = 0;
                blend.clear();
                break;
            }
        }
    }

    lut_.swap(next);
    crossfade_tone_.swap(blend);
    crossfade_active_ = blend_n;
    crossfade_epoch_ = training_epoch_;

    model_.valid = true;
    model_.generation += 1;
    model_.epoch = training_epoch_;
    model_.coefficient = c;
    model_.amplitude = std::abs(c);
    model_.phase_radians = std::arg(c);
    model_.training_samples = training_count_;

    training_active_ = false;
    training_count_ = 0;
    std::fill(fold_.begin(), fold_.end(), 0);
    fold_pos_ = 0;
    if (error) {
        error->clear();
    }
    return FixedToneTrainingStatus::Ok;
}

// ---------------------------------------------------------------------------
// Processing
// ---------------------------------------------------------------------------
int FixedToneCancelCore::active_kernel() const
{
    if (kernel_mode_ == 1) {
        return 1;
    }
    if (kernel_mode_ == 2) {
        return avx2_available_ ? 2 : 1;
    }
    return avx2_available_ ? 2 : 1;
}

FixedToneProcessCounts FixedToneCancelCore::copy_cancel(const int16_t* src,
                                                        int16_t* dst,
                                                        size_t n,
                                                        int64_t start_abs) const
{
    FixedToneProcessCounts counts;
    counts.consumed = n;
    if (n == 0) {
        return counts;
    }
    if (!enabled() || !model_.valid) {
        if (dst != src) {
            std::memcpy(dst, src, n * 2 * sizeof(int16_t));
        }
        counts.bypassed = n;
        return counts;
    }
    const int64_t rel = start_abs - model_.epoch;
    if (rel < 0) {
        // A block that predates the model's phase origin cannot be corrected
        // with it; bypass and count instead of extrapolating backwards.
        if (dst != src) {
            std::memcpy(dst, src, n * 2 * sizeof(int16_t));
        }
        counts.bypassed = n;
        return counts;
    }

    const SubtractSatFn kernel =
        (active_kernel() == 2) ? avx2_kernel() : scalar_kernel();

    size_t pos = 0;
    size_t lut_pos = static_cast<size_t>(rel % static_cast<int64_t>(period_));
    if (static_cast<uint64_t>(rel) < static_cast<uint64_t>(crossfade_active_)) {
        const size_t cnt =
            std::min(n, crossfade_active_ - static_cast<size_t>(rel));
        kernel(src, dst, cnt, crossfade_tone_.data() + 2 * static_cast<size_t>(rel));
        pos = cnt;
        counts.cancelled += cnt;
        lut_pos = (lut_pos + cnt) % period_;
    }
    while (pos < n) {
        const size_t cnt = std::min(n - pos, period_ - lut_pos);
        kernel(src + 2 * pos, dst + 2 * pos, cnt, lut_.data() + 2 * lut_pos);
        pos += cnt;
        counts.cancelled += cnt;
        lut_pos = 0;
    }
    return counts;
}

FixedToneProcessCounts FixedToneCancelCore::cancel_in_place(int16_t* iq,
                                                            size_t n,
                                                            int64_t start_abs) const
{
    return copy_cancel(iq, iq, n, start_abs);
}

} // namespace core
} // namespace uwb
} // namespace gr
