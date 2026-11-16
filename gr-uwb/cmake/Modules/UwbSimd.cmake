# UwbSimd.cmake -- centralised compile-time ISA selection for the
# hand-written UWB DSP kernels.
#
# Plan: docs/performance/AVX512内核与编译期CPU选择开发规划_2026-11-16.md
#
# The option is a *compile-time* choice, not a runtime dispatcher:
#
#   UWB_SIMD=AUTO    detect the build machine's executable ISA (only when not
#                    cross-compiling) and the compiler's capability, then pick
#                    the fastest backend that has a validated kernel.  Today
#                    that means AVX-512 when the machine + compiler support it,
#                    otherwise AVX2/FMA, otherwise SCALAR.
#   UWB_SIMD=AVX2    generate the AVX2/FMA self-written kernels explicitly.
#                    Never pass -march=native, so AVX-512 cannot sneak in.
#   UWB_SIMD=AVX512  enable the AVX-512 kernel translation unit.  The rest of
#                    the build stays AVX2/FMA; the wide kernels live in a
#                    separate file with function target attributes, so no
#                    AVX-512 instruction can execute before the runtime check.
#   UWB_SIMD=SCALAR  disable the self-written SIMD kernels (reference /
#                    compatibility builds).  External VOLK runtime dispatch is
#                    NOT controlled by this option and is documented as such.
#
# The generated uwb_simd_config.h is the single source of truth every
# translation unit sees, which is what keeps the header-only resampler core
# from being compiled into different backends in different TUs (ODR risk).
#
# Cross-compiling: AUTO refuses to guess.  Pass AVX2 / AVX512 / SCALAR
# explicitly; the build then carries the target-CPU requirement and the
# runtime check refuses a machine that cannot execute it.

include(CheckCXXCompilerFlag)
include(CheckCXXSourceCompiles)
include(CMakeParseArguments)

set(UWB_SIMD "AUTO" CACHE STRING
    "Compile-time ISA for the self-written DSP kernels: AUTO|AVX2|AVX512|SCALAR")
set_property(CACHE UWB_SIMD PROPERTY STRINGS AUTO AVX2 AVX512 SCALAR)
string(TOUPPER "${UWB_SIMD}" UWB_SIMD_REQUESTED)
if(NOT UWB_SIMD_REQUESTED MATCHES "^(AUTO|AVX2|AVX512|SCALAR)$")
    message(FATAL_ERROR
        "UWB_SIMD must be AUTO, AVX2, AVX512 or SCALAR "
        "(got '${UWB_SIMD}')")
endif()

# ---------------------------------------------------------------------------
# Compiler capability probes (never run target code).
# ---------------------------------------------------------------------------
check_cxx_compiler_flag("-mavx2" UWB_CC_HAS_MAVX2)
check_cxx_compiler_flag("-mfma" UWB_CC_HAS_MFMA)

set(_uwb_avx512_probe "
#include <immintrin.h>
__attribute__((target(\"avx512f,avx512bw,avx512dq,avx512vl\")))
static __m512 uwb_probe(__m512 a, __m512 b)
{
    __m256i v = _mm256_setzero_si256();
    __m512i i = _mm512_cvtepi16_epi32(v);
    return _mm512_add_ps(_mm512_cvtepi32_ps(i), _mm512_fmadd_ps(a, b, a));
}
int main() { return 0; }
")
check_cxx_source_compiles("${_uwb_avx512_probe}" UWB_CC_HAS_AVX512_TARGET)
if(UWB_CC_HAS_AVX512_TARGET)
    # Also require the plain flag so the AVX-512 TU can be built with the
    # declared subset even when target attributes are the mechanism.
    check_cxx_compiler_flag("-mavx512f" UWB_CC_HAS_MAVX512F)
    check_cxx_compiler_flag("-mavx512bw" UWB_CC_HAS_MAVX512BW)
    check_cxx_compiler_flag("-mavx512dq" UWB_CC_HAS_MAVX512DQ)
    check_cxx_compiler_flag("-mavx512vl" UWB_CC_HAS_MAVX512VL)
    if(NOT (UWB_CC_HAS_MAVX512F AND UWB_CC_HAS_MAVX512BW AND
            UWB_CC_HAS_MAVX512DQ AND UWB_CC_HAS_MAVX512VL))
        set(UWB_CC_HAS_AVX512_TARGET FALSE)
    endif()
endif()

# ---------------------------------------------------------------------------
# Build-machine CPU probe for AUTO.  This only reads /proc/cpuinfo; it never
# executes a target binary, so it is safe for a native (non-cross) build.
# ---------------------------------------------------------------------------
set(UWB_BUILD_CPU_HAS_AVX2 FALSE)
set(UWB_BUILD_CPU_HAS_AVX512 FALSE)
if(NOT CMAKE_CROSSCOMPILING AND CMAKE_SYSTEM_NAME STREQUAL "Linux")
    if(EXISTS "/proc/cpuinfo")
        file(READ "/proc/cpuinfo" _uwb_cpuinfo)
        if(_uwb_cpuinfo MATCHES " avx2 " AND _uwb_cpuinfo MATCHES " fma ")
            set(UWB_BUILD_CPU_HAS_AVX2 TRUE)
        endif()
        if(_uwb_cpuinfo MATCHES " avx512f " AND
           _uwb_cpuinfo MATCHES " avx512bw " AND
           _uwb_cpuinfo MATCHES " avx512dq " AND
           _uwb_cpuinfo MATCHES " avx512vl ")
            set(UWB_BUILD_CPU_HAS_AVX512 TRUE)
        endif()
    endif()
endif()

# ---------------------------------------------------------------------------
# Resolve the requested mode into a concrete backend.
# ---------------------------------------------------------------------------
set(UWB_SIMD_LEVEL 0)
set(UWB_SIMD_NAME "scalar")
set(UWB_SIMD_REQUIRED_ISA "none")
set(UWB_SIMD_HAVE_AVX2 0)
set(UWB_SIMD_HAVE_AVX512 0)
set(UWB_SIMD_SELECTED "SCALAR")

if(UWB_SIMD_REQUESTED STREQUAL "AVX2")
    if(NOT (UWB_CC_HAS_MAVX2 AND UWB_CC_HAS_MFMA))
        message(FATAL_ERROR
            "UWB_SIMD=AVX2 but this compiler cannot generate AVX2/FMA "
            "(-mavx2 / -mfma probes failed)")
    endif()
    set(UWB_SIMD_SELECTED "AVX2")
elseif(UWB_SIMD_REQUESTED STREQUAL "AVX512")
    if(NOT UWB_CC_HAS_AVX512_TARGET)
        message(FATAL_ERROR
            "UWB_SIMD=AVX512 but this compiler cannot generate the required "
            "AVX-512 subsets (avx512f/bw/dq/vl target-attribute probe failed)")
    endif()
    if(NOT (UWB_CC_HAS_MAVX2 AND UWB_CC_HAS_MFMA))
        message(FATAL_ERROR
            "UWB_SIMD=AVX512 requires the AVX2/FMA baseline flags")
    endif()
    set(UWB_SIMD_SELECTED "AVX512")
elseif(UWB_SIMD_REQUESTED STREQUAL "SCALAR")
    set(UWB_SIMD_SELECTED "SCALAR")
else() # AUTO
    if(CMAKE_CROSSCOMPILING)
        message(FATAL_ERROR
            "UWB_SIMD=AUTO cannot probe a cross-compilation target. "
            "Pass -DUWB_SIMD=AVX2, -DUWB_SIMD=AVX512 or -DUWB_SIMD=SCALAR "
            "explicitly.")
    endif()
    if(UWB_CC_HAS_AVX512_TARGET AND UWB_BUILD_CPU_HAS_AVX512)
        set(UWB_SIMD_SELECTED "AVX512")
    elseif(UWB_CC_HAS_MAVX2 AND UWB_CC_HAS_MFMA AND UWB_BUILD_CPU_HAS_AVX2)
        set(UWB_SIMD_SELECTED "AVX2")
    else()
        set(UWB_SIMD_SELECTED "SCALAR")
    endif()
endif()

if(UWB_SIMD_SELECTED STREQUAL "AVX512")
    set(UWB_SIMD_LEVEL 2)
    set(UWB_SIMD_NAME "avx512")
    set(UWB_SIMD_REQUIRED_ISA "avx512f+avx512bw+avx512dq+avx512vl")
    set(UWB_SIMD_HAVE_AVX2 1)
    set(UWB_SIMD_HAVE_AVX512 1)
elseif(UWB_SIMD_SELECTED STREQUAL "AVX2")
    set(UWB_SIMD_LEVEL 1)
    set(UWB_SIMD_NAME "avx2")
    set(UWB_SIMD_REQUIRED_ISA "avx2+fma")
    set(UWB_SIMD_HAVE_AVX2 1)
else()
    set(UWB_SIMD_LEVEL 0)
    set(UWB_SIMD_NAME "scalar")
    set(UWB_SIMD_REQUIRED_ISA "none")
endif()

# Baseline flags for every target that includes the DSP headers.  AVX-512 is
# deliberately NOT a global flag: the wide kernels use function target
# attributes in their own translation unit, so the rest of the program cannot
# execute an AVX-512 instruction before the runtime capability check.
#
# -ffp-contract=off keeps the ORIGINAL scalar floating-point semantics: -mfma
# would otherwise let the compiler fuse a*b+c in ordinary code, which changed
# bit-exact results in QAs that were written before FMA was enabled (e.g. the
# CFO diagnostic reference).  The explicit _mm*_fmadd_ps intrinsics in the
# kernels are not affected by this flag, so the AVX2/AVX-512 bit-identity
# contract still holds.
set(UWB_SIMD_COMPILE_OPTIONS -O3)
if(UWB_SIMD_LEVEL GREATER 0)
    list(APPEND UWB_SIMD_COMPILE_OPTIONS -mavx2 -mfma -ffp-contract=off)
endif()

# Apply the central flags to a target.  PUBLIC so an installed consumer of
# uwb_core/uwb_native compiles the header-only cores with the SAME backend
# (otherwise the template instantiations would differ between the library and
# the consumer -- the ODR hazard the plan calls out).
function(uwb_simd_apply target)
    if(NOT TARGET ${target})
        message(FATAL_ERROR "uwb_simd_apply: no such target '${target}'")
    endif()
    target_compile_options(${target} PUBLIC ${UWB_SIMD_COMPILE_OPTIONS})
endfunction()

message(STATUS
    "UWB SIMD: requested=${UWB_SIMD_REQUESTED} selected=${UWB_SIMD_SELECTED} "
    "level=${UWB_SIMD_LEVEL} required_isa=${UWB_SIMD_REQUIRED_ISA}")
message(STATUS
    "UWB SIMD: compiler avx2/fma=${UWB_CC_HAS_MAVX2}/${UWB_CC_HAS_MFMA} "
    "avx512-target=${UWB_CC_HAS_AVX512_TARGET} "
    "build-cpu avx2=${UWB_BUILD_CPU_HAS_AVX2} avx512=${UWB_BUILD_CPU_HAS_AVX512}")
