/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * M2-A native-domain channel model (TEST HARNESS ONLY -- G0 appendix A.3).
 *
 * This file implements the ONE declaration
 *
 *     twr::m2a_apply_impairment(iq, native_rate_hz, imp, why)
 *
 * from `include/gnuradio/uwb/uwb_twr_phy.h`.  It is NOT part of the production
 * pipeline: the harness applies it on the NATIVE CF32 grid between TX
 * resampling and the (optional) RX SC16 quantisation, so the decoder never
 * receives an impairment "truth".
 *
 * ---------------------------------------------------------------------------
 * Frozen application order (G0 A.3 / task §5)
 * ---------------------------------------------------------------------------
 *
 *     CFO phase  ->  AWGN  ->  (integer + fractional) delay  ->  multipath
 *
 * Each stage is skipped exactly when it is a no-op, so a degenerate impairment
 * is bitwise identical to the input.
 *
 * ---------------------------------------------------------------------------
 * Conventions chosen here (all documented so the QA can reproduce them)
 * ---------------------------------------------------------------------------
 *
 * CFO.  The phase advances with the NATIVE SAMPLE INDEX `n` of the buffer:
 *
 *     y[n] = x[n] * exp(j * 2*pi * cfo_hz * n / native_rate_hz)
 *
 * `n` is 0-based within the buffer handed in.  The caller MUST apply the model
 * to the WHOLE buffer at once (or pass an explicit global sample offset); if a
 * caller splits the buffer into chunks and calls once per chunk, the phase
 * restarts at 0 in every chunk -- that is a caller error, not a model feature.
 * `cfo_hz == 0` is a bitwise no-op.  Non-finite `cfo_hz` is refused, and a
 * non-zero `cfo_hz` requires a finite, positive `native_rate_hz`.
 *
 * AWGN.  Complex Gaussian, added to the samples of the SIGNAL VALID REGION.
 * SNR is defined over that region's MEAN power:
 *
 *     P_signal_valid = mean_{n in region} |x[n]|^2
 *     sigma^2        = P_signal_valid / (2 * 10^(SNR_dB/10))   [per component]
 *
 * so that SNR_dB = 10*log10(P_signal_valid / (2*sigma^2)).  The frozen G0 A.3
 * definition forbids leading zero padding from changing the noise level of the
 * same signal.  The public API exposes NO explicit region parameter, so this
 * model DERIVES the region as the NONZERO SUPPORT of the (CFO-rotated) buffer:
 *
 *     lo = first index with x != 0,  hi = last index with x != 0, region = [lo, hi)
 *
 * (An all-zero input has no signal, so P_signal_valid == 0 and no noise is
 * added -- the formula is applied literally.)  Noise is added only inside the
 * region, which makes the added noise EXACTLY independent of any leading or
 * trailing zero padding: the same signal plus zeros produces the identical
 * noise samples.  If the API ever gains an explicit region argument, pass it
 * here and drop the derivation.
 *
 * PRNG.  `std::mt19937_64` seeded with `awgn_seed` (seed 0 is a perfectly
 * ordinary seed for this generator -- documented, not special-cased), and a
 * FIXED Box-Muller transform built from two uniforms so the stream does not
 * depend on a library's `std::normal_distribution` implementation:
 *
 *     u1 = ((raw >> 11) + 0.5) / 2^53        # uniform in (0, 1), never 0
 *     u2 = ((raw >> 11) + 0.5) / 2^53
 *     r  = sqrt(-2 ln u1),  theta = 2*pi*u2
 *     z0 = r cos(theta), z1 = r sin(theta)   # two independent N(0,1)
 *
 * One (z0, z1) pair is consumed per region sample: real += sigma*z0,
 * imag += sigma*z1.
 *
 * Delay.  Integer `delay_int_samples` plus a fractional `delay_frac_num /
 * delay_frac_den`.  The integer part is a front zero-padded right shift; the
 * output length is PRESERVED (the tail is truncated).  The fractional part is
 * the CAUSAL fractional-delay FIR
 *
 *     h[k] = sinc(k - f),  k = 0 .. 15   (16 taps, order 15),  f = num/den
 *     normalized so sum_k h[k] == 1      (unity DC gain)
 *     y[n] = sum_{k=0}^{15} h[k] * x[n-k],  x[j] = 0 for j < 0
 *
 * The normalized first moment is exactly f:  sum_k k*h[k] == f, so the digital
 * group delay is `f` native samples (recorded; NOT a hardware calibration
 * constant).  Edge convention: samples outside [0, N) are zero; the output
 * length is N.  `num == 0` skips the FIR entirely, so `delay_int == 0` with
 * `num == 0` is a bitwise no-op.
 *
 * Multipath.  Full convolution with the complex taps:
 *
 *     y[n] = sum_{k=0}^{M-1} tap[k] * x[n-k],   n = 0 .. N+M-2
 *
 * `tap[0]` is the FIRST path (offset 0); tap[k] sits at +k native samples.  The
 * added tail length is `M-1` native samples and is RETAINED: the output grows
 * from N to N+M-1.  A single tap equal to 1 is a bitwise no-op.  Growth past
 * `kM2aMaxSamples` is refused (`CapacityExceeded`-style named reason).
 *
 * ---------------------------------------------------------------------------
 * Failure policy
 * ---------------------------------------------------------------------------
 *
 * Domain check first (the M0.1 N07 rule), then the identity short-circuit.
 * Every stage works on a COPY; `iq` is committed only after the result is
 * verified finite.  An invalid impairment, an unusable rate, an extreme SNR or
 * a non-finite result returns false with a NAMED reason and leaves `iq`
 * UNCHANGED.  NaN/Inf is never passed through silently.
 */

#include <gnuradio/uwb/uwb_twr_phy.h>

#include <array>
#include <cmath>
#include <complex>
#include <cstdint>
#include <random>
#include <string>
#include <vector>

namespace gr {
namespace uwb {
namespace twr {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr size_t kFracDelayTaps = 16; // order 15 (see the file header)

// ---------------------------------------------------------------------------
// Fractional-delay FIR (causal, normalized, group delay == f)
// ---------------------------------------------------------------------------

double sinc(double x)
{
    if (x == 0.0)
        return 1.0;
    const double px = kPi * x;
    return std::sin(px) / px;
}

std::array<double, kFracDelayTaps> frac_delay_taps(double f)
{
    std::array<double, kFracDelayTaps> h{};
    double sum = 0.0;
    for (size_t k = 0; k < kFracDelayTaps; ++k) {
        h[k] = sinc(static_cast<double>(k) - f);
        sum += h[k];
    }
    if (sum != 0.0) {
        for (double& v : h)
            v /= sum;
    }
    return h;
}

// ---------------------------------------------------------------------------
// Deterministic complex Gaussian stream (fixed Box-Muller, see file header)
// ---------------------------------------------------------------------------

class AwgnStream
{
public:
    explicit AwgnStream(uint64_t seed) : gen_(seed) {}

    // Two independent standard normals.
    void next(double& z0, double& z1)
    {
        const double u1 = uniform_open();
        const double u2 = uniform_open();
        const double r = std::sqrt(-2.0 * std::log(u1));
        const double theta = 2.0 * kPi * u2;
        z0 = r * std::cos(theta);
        z1 = r * std::sin(theta);
    }

private:
    // Uniform in (0, 1): 53 mantissa bits plus a half, so log() is always safe.
    double uniform_open()
    {
        const uint64_t raw = gen_();
        return (static_cast<double>(raw >> 11) + 0.5) * (1.0 / 9007199254740992.0);
    }

    std::mt19937_64 gen_;
};

bool all_finite(const std::vector<std::complex<float>>& v)
{
    for (const std::complex<float>& x : v) {
        if (!std::isfinite(x.real()) || !std::isfinite(x.imag()))
            return false;
    }
    return true;
}

constexpr std::complex<float> kZero(0.0f, 0.0f);

} // namespace

bool m2a_apply_impairment(std::vector<std::complex<float>>& iq, double native_rate_hz,
                          const M2aImpairment& imp, std::string& why)
{
    // 1. Domain check FIRST (N07 rule), then the identity short-circuit.
    if (!imp.is_valid(why))
        return false;

    // Disabled, or every sub-impairment is zero: bitwise no-op.
    if (imp.is_identity())
        return true;

    // The CFO phase increment is the only user of the rate.
    if (imp.cfo_hz != 0.0 &&
        (!std::isfinite(native_rate_hz) || !(native_rate_hz > 0.0))) {
        why = "native_rate_hz must be finite and > 0 for a non-zero cfo_hz";
        return false;
    }

    std::vector<std::complex<float>> work = iq;

    // 2. CFO phase: advances with the native sample index n.
    if (imp.cfo_hz != 0.0) {
        const double w = 2.0 * kPi * imp.cfo_hz / native_rate_hz;
        for (size_t n = 0; n < work.size(); ++n) {
            const double ph = w * static_cast<double>(n);
            work[n] *= std::complex<float>(static_cast<float>(std::cos(ph)),
                                           static_cast<float>(std::sin(ph)));
        }
    }

    // 3. AWGN over the derived valid region (nonzero support).
    if (imp.awgn_enabled) {
        size_t lo = 0;
        size_t hi = work.size();
        while (lo < hi && work[lo] == kZero)
            ++lo;
        while (hi > lo && work[hi - 1] == kZero)
            --hi;
        const size_t nsig = hi - lo;
        if (nsig == 0) {
            // No signal -> P_signal_valid == 0 -> sigma^2 == 0 -> no noise.
            // The formula is applied literally; nothing is invented.
        } else {
            double p = 0.0;
            for (size_t n = lo; n < hi; ++n) {
                const double re = static_cast<double>(work[n].real());
                const double im = static_cast<double>(work[n].imag());
                p += re * re + im * im;
            }
            p /= static_cast<double>(nsig);
            const double denom = 2.0 * std::pow(10.0, imp.awgn_snr_db / 10.0);
            const double sigma2 = p / denom;
            if (!std::isfinite(sigma2) || sigma2 < 0.0) {
                why = "awgn sigma^2 is not finite (awgn_snr_db is too extreme)";
                return false;
            }
            const double sigma = std::sqrt(sigma2);
            AwgnStream stream(imp.awgn_seed);
            for (size_t n = lo; n < hi; ++n) {
                double z0 = 0.0;
                double z1 = 0.0;
                stream.next(z0, z1);
                work[n] += std::complex<float>(static_cast<float>(sigma * z0),
                                               static_cast<float>(sigma * z1));
            }
        }
    }

    // 4. Delay: integer shift, then the causal fractional FIR.
    if (imp.delay_int_samples > 0 || imp.delay_frac_num > 0) {
        const size_t n0 = work.size();

        if (imp.delay_int_samples > 0) {
            const size_t d = static_cast<size_t>(imp.delay_int_samples);
            for (size_t n = n0; n-- > 0;) {
                work[n] = (n >= d) ? work[n - d] : kZero;
            }
        }

        if (imp.delay_frac_num > 0) {
            const double f = static_cast<double>(imp.delay_frac_num) /
                             static_cast<double>(imp.delay_frac_den);
            const std::array<double, kFracDelayTaps> h = frac_delay_taps(f);
            std::vector<std::complex<float>> filtered(n0, kZero);
            for (size_t n = 0; n < n0; ++n) {
                double ar = 0.0;
                double ai = 0.0;
                const size_t kmax = (n < kFracDelayTaps) ? n : (kFracDelayTaps - 1);
                for (size_t k = 0; k <= kmax; ++k) {
                    const double hr = h[k];
                    const std::complex<float>& x = work[n - k];
                    ar += hr * static_cast<double>(x.real());
                    ai += hr * static_cast<double>(x.imag());
                }
                filtered[n] = std::complex<float>(static_cast<float>(ar),
                                                  static_cast<float>(ai));
            }
            work.swap(filtered);
        }
    }

    // 5. Multipath: full convolution; the M-1 tail is retained.
    if (!imp.multipath.empty()) {
        const size_t m = imp.multipath.size();
        if (work.size() + m - 1 > kM2aMaxSamples) {
            why = "multipath output would exceed kM2aMaxSamples";
            return false;
        }
        std::vector<std::complex<float>> out(work.size() + m - 1, kZero);
        for (size_t n = 0; n < work.size(); ++n) {
            const std::complex<float>& x = work[n];
            if (x == kZero)
                continue;
            for (size_t k = 0; k < m; ++k)
                out[n + k] += imp.multipath[k] * x;
        }
        work.swap(out);
    }

    if (!all_finite(work)) {
        why = "impairment produced a non-finite sample";
        return false;
    }

    iq.swap(work);
    return true;
}

} // namespace twr
} // namespace uwb
} // namespace gr
