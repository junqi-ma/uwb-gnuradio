/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Per-kernel-group backend selection for the demod / SIC DSP kernels.
 *
 * Plan: docs/performance/解调与SIC_AVX512内核扩展开发规划_2026-11-16.md (P1).
 *
 * Two separate things live here:
 *
 *   1. A DEFAULT policy per kernel group.  It is a compile-time table decided
 *      by measured ABBA results, not by "the CPU has AVX-512".  A group whose
 *      AVX-512 candidate did not clear the admission bar keeps AVX2 even in an
 *      AVX-512 build.
 *
 *   2. A DIAGNOSTIC override (avx2|avx512|scalar|default) used by QA and the
 *      A/B/C/D benchmark matrix.  It is deliberately not part of any
 *      production PHY profile: nothing in the receiver sets it, and the
 *      default is "no override".
 *
 * The registry is process-global and lock-free (relaxed atomics) because a
 * worker thread may resolve a group while a test harness sets another one.
 * Overrides are expected to be installed before a measurement run starts.
 */
#pragma once

#include <string>

namespace gr {
namespace uwb {
namespace core {
namespace simd {

/**
 * Kernel groups that can be switched independently.  The split follows the
 * phase-2 hotspot map so a measured gain (or regression) can be attributed to
 * one family of kernels instead of the whole chain.
 */
enum class KernelGroup : int {
    /** D1: real-template dots, SFD fold, element-wise complex ops.
     *  Shared by the demodulator and by every SIC path that reuses comm_rx. */
    Sync = 0,
    /** D3: CFO table rotation (VOLK-owned today; kept for attribution). */
    Cfo = 1,
    /** D2: fixed-38 CIR FIR and the fused PHR/payload CMF dots. */
    Cir = 2,
    /** S1: discrete-CIR pulse reconstruction (SIC-specific). */
    SicReconstruct = 3,
    /** S2: ROI energy / estimator scans (SIC-specific). */
    SicEnergy = 4,
    Count = 5
};

enum class Backend : int {
    Default = 0, ///< follow the measured per-group policy
    Scalar = 1,  ///< portable reference
    Avx2 = 2,    ///< AVX2/FMA reference kernels
    Avx512 = 3   ///< wide kernels (only in a UWB_SIMD=AVX512 build)
};

/** Stable name of a group ("sync", "cfo", "cir", "sic_reconstruct", ...). */
const char* kernel_group_name(KernelGroup g) noexcept;

/** Parse "default"|"scalar"|"avx2"|"avx512".  False on an unknown name. */
bool parse_backend(const std::string& name, Backend* out) noexcept;

/**
 * Install a diagnostic override.
 *
 * Throws std::invalid_argument for an unknown group index or backend name, and
 * std::runtime_error when AVX-512 is requested from a build without an
 * AVX-512 backend or on a CPU/OS that cannot execute it.  A forced mode never
 * silently degrades: a request that cannot be honoured is an error, not a
 * quiet fallback that would still be reported as "avx512".
 */
void set_kernel_backend(KernelGroup g, Backend b);
void set_kernel_backend(KernelGroup g, const std::string& name);

/** Currently installed override (Backend::Default when none). */
Backend kernel_backend_override(KernelGroup g) noexcept;

/**
 * The backend a kernel group will actually use for the next call.
 *
 * Override first, then the measured per-group default policy, then the best
 * backend the build/CPU can run.  Never returns Avx512 unless
 * UWB_SIMD_HAVE_AVX512 and the runtime capability check both say yes.
 */
Backend resolve_backend(KernelGroup g) noexcept;

/** Name of resolve_backend(g): "scalar", "avx2" or "avx512". */
const char* resolved_backend_name(KernelGroup g) noexcept;

/** True when this binary was built with UWB_SIMD=AVX512. */
bool avx512_build() noexcept;

/** True when this binary was built with AVX2/FMA (level >= 1). */
bool avx2_build() noexcept;

/** Restore every group to Backend::Default (test teardown). */
void reset_kernel_backends() noexcept;

} // namespace simd
} // namespace core
} // namespace uwb
} // namespace gr
