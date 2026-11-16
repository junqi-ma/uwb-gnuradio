/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Per-kernel-group backend selection for the demod / SIC DSP kernels.
 *
 * This translation unit is compiled on the baseline ISA (it is part of
 * uwb_simd_probe) so the selector itself can never execute a wide
 * instruction, and so a diagnostic override can be installed before any
 * AVX-512 kernel is reachable.
 */

#include <gnuradio/uwb/uwb_kernel_backend.h>

#include <gnuradio/uwb/uwb_simd.h>
#include <gnuradio/uwb/uwb_simd_config.h>

#include <array>
#include <atomic>
#include <stdexcept>
#include <string>

namespace gr {
namespace uwb {
namespace core {
namespace simd {

namespace {

constexpr int kGroupCount = static_cast<int>(KernelGroup::Count);

constexpr int group_index(KernelGroup g) noexcept
{
    return static_cast<int>(g);
}

/** Per-group default AVX-512 admission.
 *
 *  Set from the phase-2 ABBA measurements, NOT from CPU capability:
 *
 *   Sync           yes  real-template 8-lag dot -36%/lag, SFD fold -41%,
 *                       element-wise accumulate/sub -62% (small n) and
 *                       -11..-12% (135k samples), scale -23..-35%.
 *   Cfo            no   the rotation is VOLK-owned (volk_32fc_x2_multiply /
 *                       volk_32fc_s32fc_multiply); not converted this round.
 *   Cir            no   the 512-bit complex MAC is SLOWER here: fixed-52
 *                       payload dot +29%, fixed-38 CIR FIR +95%.
 *   SicReconstruct no   the 20-tap pulse accumulation is store-forwarding
 *                       bound: +1% (i.e. no gain).
 *   SicEnergy      no   not converted this round; no measured candidate.
 *
 *  A group that is "no" stays on AVX2 in an AVX-512 build and is reachable
 *  only through an explicit diagnostic override.
 */
constexpr std::array<bool, kGroupCount> kAvx512Default = {
    true,  // Sync
    false, // Cfo
    false, // Cir
    false, // SicReconstruct
    false, // SicEnergy
};

constexpr std::array<const char*, kGroupCount> kGroupNames = {
    "sync", "cfo", "cir", "sic_reconstruct", "sic_energy"
};

std::array<std::atomic<int>, kGroupCount> g_override = {
    std::atomic<int>(static_cast<int>(Backend::Default)),
    std::atomic<int>(static_cast<int>(Backend::Default)),
    std::atomic<int>(static_cast<int>(Backend::Default)),
    std::atomic<int>(static_cast<int>(Backend::Default)),
    std::atomic<int>(static_cast<int>(Backend::Default)),
};

bool valid_group(KernelGroup g) noexcept
{
    const int i = group_index(g);
    return i >= 0 && i < kGroupCount;
}

bool can_run_avx512() noexcept
{
#if UWB_SIMD_HAVE_AVX512
    return runtime_supported();
#else
    return false;
#endif
}

} // namespace

const char* kernel_group_name(KernelGroup g) noexcept
{
    return valid_group(g) ? kGroupNames[group_index(g)] : "invalid";
}

bool parse_backend(const std::string& name, Backend* out) noexcept
{
    Backend b = Backend::Default;
    if (name == "default")
        b = Backend::Default;
    else if (name == "scalar")
        b = Backend::Scalar;
    else if (name == "avx2")
        b = Backend::Avx2;
    else if (name == "avx512")
        b = Backend::Avx512;
    else
        return false;
    if (out)
        *out = b;
    return true;
}

void set_kernel_backend(KernelGroup g, Backend b)
{
    if (!valid_group(g))
        throw std::invalid_argument("set_kernel_backend: invalid kernel group");
    switch (b) {
    case Backend::Default:
    case Backend::Scalar:
        break;
    case Backend::Avx2:
        if (!UWB_SIMD_HAVE_AVX2)
            throw std::runtime_error(
                std::string("set_kernel_backend(") + kernel_group_name(g) +
                ", avx2): this build has no AVX2 backend (UWB_SIMD=" +
                UWB_SIMD_NAME + ")");
        break;
    case Backend::Avx512:
        if (!UWB_SIMD_HAVE_AVX512)
            throw std::runtime_error(
                std::string("set_kernel_backend(") + kernel_group_name(g) +
                ", avx512): this build has no AVX-512 backend (UWB_SIMD=" +
                UWB_SIMD_NAME + ")");
        if (!can_run_avx512())
            throw std::runtime_error(
                std::string("set_kernel_backend(") + kernel_group_name(g) +
                ", avx512): this CPU/OS cannot execute " +
                UWB_SIMD_REQUIRED_ISA);
        break;
    }
    g_override[group_index(g)].store(static_cast<int>(b),
                                     std::memory_order_relaxed);
}

void set_kernel_backend(KernelGroup g, const std::string& name)
{
    Backend b = Backend::Default;
    if (!parse_backend(name, &b))
        throw std::invalid_argument("set_kernel_backend: unknown backend '" +
                                    name + "'");
    set_kernel_backend(g, b);
}

Backend kernel_backend_override(KernelGroup g) noexcept
{
    if (!valid_group(g))
        return Backend::Default;
    return static_cast<Backend>(
        g_override[group_index(g)].load(std::memory_order_relaxed));
}

Backend resolve_backend(KernelGroup g) noexcept
{
    if (!valid_group(g))
        return Backend::Scalar;
    const Backend forced = kernel_backend_override(g);
    if (forced != Backend::Default)
        return forced;

    if (UWB_SIMD_HAVE_AVX512 && kAvx512Default[group_index(g)] &&
        can_run_avx512())
        return Backend::Avx512;
    if (UWB_SIMD_HAVE_AVX2 && runtime_supported())
        return Backend::Avx2;
    return Backend::Scalar;
}

const char* resolved_backend_name(KernelGroup g) noexcept
{
    switch (resolve_backend(g)) {
    case Backend::Avx512:
        return "avx512";
    case Backend::Avx2:
        return "avx2";
    case Backend::Scalar:
        return "scalar";
    case Backend::Default:
        break;
    }
    return "default";
}

bool avx512_build() noexcept
{
    return UWB_SIMD_HAVE_AVX512 != 0;
}

bool avx2_build() noexcept
{
    return UWB_SIMD_HAVE_AVX2 != 0;
}

void reset_kernel_backends() noexcept
{
    for (int i = 0; i < kGroupCount; ++i)
        g_override[i].store(static_cast<int>(Backend::Default),
                            std::memory_order_relaxed);
}

} // namespace simd
} // namespace core
} // namespace uwb
} // namespace gr
