/* SPDX-License-Identifier: GPL-3.0-or-later */
#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

namespace openuwb { namespace experiment { namespace cfo_diag {
using cf = std::complex<float>;
using cd = std::complex<double>;

// Compact replay of a real reconstruction interval. absolute_lo and the
// original phase reference preserve the production kernel's absolute grid.
struct Input {
    const cd* accum = nullptr;
    const cf* iq = nullptr;
    size_t n = 0;
    int64_t absolute_lo = 0, phase_reference = 0;
    double cfo_hz = 0, sample_rate = 998.4e6;
};
enum class Variant { FusedChecked, FusedModelChecked, FusedUnchecked, SplitChecked };
struct Timings {
    double total_us = 0, base_us = 0, ramp_us = 0, convert_us = 0, multiply_us = 0;
    double model_check_us = 0, trial_us = 0, trial_check_us = 0;
};
struct Scratch {
    std::vector<cf> model, rotation, converted, trial;
    void reserve(size_t n)
    {
        model.resize(n);
        rotation.resize(n);
        converted.resize(n);
        trial.resize(n);
    }
};

namespace detail {
using Clock = std::chrono::steady_clock;
constexpr double pi = 3.14159265358979323846;
constexpr size_t block_size = 1024;
inline double elapsed(Clock::time_point start)
{
    return std::chrono::duration<double, std::micro>(Clock::now() - start).count();
}
inline bool finite(cf x)
{
    return std::isfinite(x.real()) && std::isfinite(x.imag());
}
inline int64_t floordiv(int64_t a, int64_t b)
{
    return a / b - ((a % b) < 0 ? 1 : 0);
}
inline void build_base(double omega, std::array<cf, block_size>& base)
{
    const cd step = std::exp(cd(0, omega));
    cd cur = 1;
    for (size_t j = 0; j < block_size; ++j) {
        base[j] = cf(cur);
        cur *= step;
    }
}

// Each instantiation removes the diagnostic selection branch from the sample
// loop. CheckLevel2 is the production expression; CheckLevel1 removes the
// trial subtraction AND its finite predicate, because the trial has no other
// side effect in the original kernel. CheckLevel0 removes both predicates.
// The unchecked variants are NOT online alternatives: callers must replay
// prevalidated finite snapshots and post-validate every result outside timers.
template <unsigned CheckLevel>
inline bool fused(const Input& in, double omega, Scratch& work)
{
    std::array<cf, block_size> base;
    build_base(omega, base);
    for (size_t i = 0; i < in.n;) {
        const int64_t p = in.absolute_lo + int64_t(i) - in.phase_reference;
        const int64_t b = floordiv(p, int64_t(block_size));
        const size_t j0 = size_t(p - b * int64_t(block_size));
        const size_t count = std::min(block_size - j0, in.n - i);
        const cf block(std::exp(cd(0, omega * double(b * int64_t(block_size)))));
        for (size_t z = 0; z < count; ++z) {
            work.model[i + z] = cf(in.accum[i + z]) * (base[j0 + z] * block);
            if constexpr (CheckLevel == 2) {
                if (!finite(work.model[i + z]) ||
                    !finite(in.iq[i + z] - work.model[i + z]))
                    return false;
            } else if constexpr (CheckLevel == 1) {
                if (!finite(work.model[i + z]))
                    return false;
            }
        }
        i += count;
    }
    return true;
}

// Separating the loops materializes extra arrays and changes memory traffic,
// compiler vectorization and cache behavior. These sub-times diagnose the
// staged implementation; they are not additive attribution of fused time.
inline bool split(const Input& in, double omega, Scratch& work, Timings& times)
{
    std::array<cf, block_size> base;
    auto start = Clock::now();
    build_base(omega, base);
    times.base_us = elapsed(start);

    start = Clock::now();
    for (size_t i = 0; i < in.n;) {
        const int64_t p = in.absolute_lo + int64_t(i) - in.phase_reference;
        const int64_t b = floordiv(p, int64_t(block_size));
        const size_t j0 = size_t(p - b * int64_t(block_size));
        const size_t count = std::min(block_size - j0, in.n - i);
        const cf block(std::exp(cd(0, omega * double(b * int64_t(block_size)))));
        for (size_t z = 0; z < count; ++z)
            work.rotation[i + z] = base[j0 + z] * block;
        i += count;
    }
    times.ramp_us = elapsed(start);

    start = Clock::now();
    for (size_t i = 0; i < in.n; ++i)
        work.converted[i] = cf(in.accum[i]);
    times.convert_us = elapsed(start);

    start = Clock::now();
    for (size_t i = 0; i < in.n; ++i)
        work.model[i] = work.converted[i] * work.rotation[i];
    times.multiply_us = elapsed(start);

    start = Clock::now();
    for (size_t i = 0; i < in.n; ++i) {
        if (!finite(work.model[i])) {
            times.model_check_us = elapsed(start);
            return false;
        }
    }
    times.model_check_us = elapsed(start);

    start = Clock::now();
    for (size_t i = 0; i < in.n; ++i)
        work.trial[i] = in.iq[i] - work.model[i];
    times.trial_us = elapsed(start);

    start = Clock::now();
    for (size_t i = 0; i < in.n; ++i) {
        if (!finite(work.trial[i])) {
            times.trial_check_us = elapsed(start);
            return false;
        }
    }
    times.trial_check_us = elapsed(start);
    return true;
}
} // namespace detail

// Writes scratch only. No input IQ is modified or cancellation committed.
// No allocation occurs here; reserve() must have prepared all work arrays.
inline bool run(const Input& input, Variant variant, Scratch& scratch, Timings& times)
{
    times = Timings{};
    const auto start = detail::Clock::now();
    const auto finish = [&](bool ok) {
        times.total_us = detail::elapsed(start);
        return ok;
    };
    const auto max_index = std::numeric_limits<int64_t>::max();
    if (!input.accum || !input.iq || !input.n || input.absolute_lo < 0 ||
        input.phase_reference < 0 || uint64_t(input.n) > uint64_t(max_index - input.absolute_lo) ||
        !std::isfinite(input.cfo_hz) || !std::isfinite(input.sample_rate) ||
        !(input.sample_rate > 0) ||
        scratch.model.size() < input.n || scratch.rotation.size() < input.n ||
        scratch.converted.size() < input.n || scratch.trial.size() < input.n ||
        static_cast<unsigned>(variant) > static_cast<unsigned>(Variant::SplitChecked))
        return finish(false);
    const double omega = 2 * detail::pi * input.cfo_hz / input.sample_rate;
    if (!std::isfinite(omega))
        return finish(false);
    switch (variant) {
    case Variant::FusedChecked:
        return finish(detail::fused<2>(input, omega, scratch));
    case Variant::FusedModelChecked:
        return finish(detail::fused<1>(input, omega, scratch));
    case Variant::FusedUnchecked:
        return finish(detail::fused<0>(input, omega, scratch));
    case Variant::SplitChecked:
        return finish(detail::split(input, omega, scratch, times));
    }
    return finish(false);
}
}}}
