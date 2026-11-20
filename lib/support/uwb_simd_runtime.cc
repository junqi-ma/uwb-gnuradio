/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Runtime capability check for the compile-time SIMD backend.
 *
 * Keep this translation unit free of wide intrinsics: it is the entry point
 * that must run (and refuse) before any AVX-512 kernel can be reached.  The
 * check uses the compiler's CPU feature builtins, which consult CPUID and the
 * OS XSAVE state (so an OS that does not enable the vector state is rejected
 * even on capable silicon).
 */

#include <openuwb/support/simd.h>

#include <stdexcept>
#include <string>

#if defined(__x86_64__) || defined(__i386__)
#define UWB_SIMD_X86 1
#else
#define UWB_SIMD_X86 0
#endif

namespace gr {
namespace uwb {
namespace core {
namespace simd {

const char* backend_name() noexcept
{
    return UWB_SIMD_NAME;
}

const char* required_isa() noexcept
{
    return UWB_SIMD_REQUIRED_ISA;
}

bool runtime_supported() noexcept
{
#if UWB_SIMD_LEVEL == 0
    return true;
#elif UWB_SIMD_X86 && (defined(__GNUC__) || defined(__clang__))
    __builtin_cpu_init();
#if UWB_SIMD_LEVEL >= 2
    return __builtin_cpu_supports("avx2") && __builtin_cpu_supports("fma") &&
           __builtin_cpu_supports("avx512f") &&
           __builtin_cpu_supports("avx512bw") &&
           __builtin_cpu_supports("avx512dq") &&
           __builtin_cpu_supports("avx512vl");
#else
    return __builtin_cpu_supports("avx2") && __builtin_cpu_supports("fma");
#endif
#else
    // A non-x86 build cannot have selected AVX2/AVX-512 in UwbSimd.cmake.
    return true;
#endif
}

void require_simd_backend()
{
    if (runtime_supported())
        return;
    throw std::runtime_error(
        std::string("UWB SIMD backend '") + backend_name() +
        "' requires ISA " + required_isa() +
        ", which this CPU/OS does not provide. Rebuild with "
        "-DUWB_SIMD=AVX2, -DUWB_SIMD=SCALAR, or rebuild on the target "
        "machine with -DUWB_SIMD=AUTO.");
}

} // namespace simd
} // namespace core
} // namespace uwb
} // namespace gr
