/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Compile-time SIMD backend selection helpers.
 *
 * The backend is chosen by CMake (UWB_SIMD=AUTO|AVX2|AVX512|SCALAR) and
 * frozen into uwb_simd_config.h.  This header exposes:
 *
 *   - the resolved backend name / required ISA as compile-time strings,
 *   - a runtime capability check that the produced binary can actually run
 *     on the current machine.
 *
 * The check is implemented in its own baseline-ISA translation unit
 * (uwb_simd_runtime.cc) so it cannot itself execute an AVX-512 instruction.
 * A non-SCALAR build has a target-CPU requirement: it is not a portable
 * single binary.  Callers that must survive an unsupported machine can test
 * simd_runtime_supported() first; the resampler constructor uses
 * require_simd_backend() to refuse loudly instead of dying on SIGILL.
 */
#pragma once

#include <string>

#include <gnuradio/uwb/uwb_simd_config.h>

namespace gr {
namespace uwb {
namespace core {
namespace simd {

/** Resolved backend name: "scalar", "avx2" or "avx512". */
const char* backend_name() noexcept;

/** Required ISA string, e.g. "none", "avx2+fma", "avx512f+...". */
const char* required_isa() noexcept;

/**
 * True iff the current CPU/OS can execute every instruction the compiled
 * backend may use.  Always true for UWB_SIMD_LEVEL == 0.
 */
bool runtime_supported() noexcept;

/**
 * Throw std::runtime_error with a precise message when runtime_supported()
 * is false.  Used at the entry points of the header-only cores so an
 * explicitly targeted binary fails loudly on an unsupported machine.
 */
void require_simd_backend();

} // namespace simd
} // namespace core
} // namespace uwb
} // namespace gr
