// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <volk/volk.h>
#include <complex>
#include <cstddef>

namespace gr::uwb::core {

// A lag points to the first sample aligned with taps[0]. Keep the scalar
// reference and VOLK reductions separate because their accumulation differs.
inline std::complex<float> conjugate_dot_scalar(const std::complex<float>* input,
                                                const std::complex<float>* taps,
                                                size_t count)
{
    std::complex<float> acc{};
    for (size_t k = 0; k < count; ++k)
        acc += input[k] * std::conj(taps[k]);
    return acc;
}

inline void conjugate_dot_into(std::complex<float>& out,
                               const std::complex<float>* input,
                               const std::complex<float>* taps,
                               size_t count)
{
    if (count != 0)
        volk_32fc_x2_conjugate_dot_prod_32fc(
            &out, input, taps, static_cast<unsigned int>(count));
    else
        out = {};
}

inline std::complex<float> conjugate_dot(const std::complex<float>* input,
                                         const std::complex<float>* taps,
                                         size_t count)
{
    std::complex<float> acc{};
    conjugate_dot_into(acc, input, taps, count);
    return acc;
}

inline float sample_power_sum(const std::complex<float>* input, size_t count)
{
    float power = 0.0f;
    for (size_t k = 0; k < count; ++k)
        power += std::norm(input[k]);
    return power;
}

inline constexpr float kUwbEpsilon = 1e-12f;

inline float normalized_correlation_power(std::complex<float> corr,
                                           float power,
                                           float template_energy = 1.0f)
{
    return std::norm(corr) / (power * template_energy + kUwbEpsilon);
}

} // namespace gr::uwb::core
