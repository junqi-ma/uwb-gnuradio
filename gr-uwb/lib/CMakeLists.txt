# Copyright 2011-2020 Free Software Foundation, Inc.
# SPDX-License-Identifier: GPL-3.0-or-later

set(uwb_core_sources
    uwb_demod_pipeline.cc
    uwb_discrete_cir_estimator.cc
    uwb_discrete_cir_reconstruct.cc
    uwb_fixed_tone_cancel_core.cc
    uwb_fullpacket_sic_core.cc
    uwb_preamble_sic_core.cc
    uwb_sic_unified.cc
)

# The runtime capability check MUST be compiled on the baseline ISA: it is the
# entry point that has to run (and refuse) before any AVX-512 kernel can be
# reached.  A separate OBJECT library keeps it out of uwb_simd_apply().
# uwb_kernel_backend.cc lives here too: the per-group selector must be able to
# answer before any wide instruction is reachable, and its override is what QA
# uses to pin a backend.
add_library(uwb_simd_probe OBJECT uwb_simd_runtime.cc uwb_kernel_backend.cc)
target_include_directories(uwb_simd_probe PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
    ${PROJECT_BINARY_DIR}/include)
target_compile_options(uwb_simd_probe PRIVATE -O3)
set_target_properties(uwb_simd_probe PROPERTIES POSITION_INDEPENDENT_CODE ON)

# The AVX-512 kernels live in their own translation unit and are built only
# when CMake selected the AVX-512 backend.  The rest of the build stays on the
# AVX2/FMA baseline; the wide functions carry target attributes.
if(UWB_SIMD_HAVE_AVX512)
    list(APPEND uwb_core_sources uwb_rational_resampler_avx512.cc
                                uwb_resampler_transposed_avx512.cc
                                uwb_demod_sic_avx512.cc)
endif()

########################################################################
# Native runtime sources (pipeline, sources, tone decorator; UHD-gated
# device backend)
########################################################################
set(uwb_native_sources
    uwb_uhd_sc16_pipeline.cc
    uwb_fixed_tone_cancel_source.cc
)
if(UWB_HAVE_UHD)
    list(APPEND uwb_native_sources uwb_uhd_burst_backend.cc)
endif()

########################################################################
# uwb_core: pure C++ core archive
########################################################################
add_library(uwb_core STATIC ${uwb_core_sources}
    $<TARGET_OBJECTS:uwb_simd_probe>)
target_link_libraries(uwb_core PUBLIC Volk::volk Threads::Threads)
target_compile_features(uwb_core PUBLIC cxx_std_17)
target_include_directories(uwb_core
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
    PUBLIC $<INSTALL_INTERFACE:include>
)
set_target_properties(uwb_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
# Central ISA flags (PUBLIC so an installed consumer compiles the header-only
# cores with the same backend).  See cmake/Modules/UwbSimd.cmake.
uwb_simd_apply(uwb_core)

########################################################################
# uwb_native: native RX runtime archive (pipeline + sources + decorator)
########################################################################
add_library(uwb_native STATIC ${uwb_native_sources})
target_link_libraries(uwb_native PUBLIC uwb_core Threads::Threads)
target_include_directories(uwb_native
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
    PUBLIC $<INSTALL_INTERFACE:include>
)
set_target_properties(uwb_native PROPERTIES POSITION_INDEPENDENT_CODE ON)
uwb_simd_apply(uwb_native)
if(UWB_HAVE_UHD)
    target_link_libraries(uwb_native PUBLIC UHD::UHD)
    # Public so consumers including uwb_uhd_burst_backend.h see the guard.
    target_compile_definitions(uwb_native PUBLIC UWB_HAVE_UHD=1)
endif()

########################################################################
# M1-B: the standalone, GNU-Radio-free / UHD-free TWR protocol core.
#
# `uwb_twr_core.h` + `uwb_twr_protocol_time.h` are the PUBLIC installed M1-B
# contract.  The core is its OWN archive so an isolated install consumer can
# link it and PROVE (ldd/readelf) that it pulls in neither GNU Radio nor UHD
# (M1-B §8).  The fake link is transport/QA support and is NOT installed.
########################################################################
add_library(uwb_twr_core STATIC uwb_twr_core.cc)
target_include_directories(uwb_twr_core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
    $<INSTALL_INTERFACE:include>)
set_target_properties(uwb_twr_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_library(uwb_twr_fake_link STATIC uwb_twr_fake_link.cc)
target_include_directories(uwb_twr_fake_link PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>)
target_link_libraries(uwb_twr_fake_link PUBLIC uwb_twr_core)

########################################################################
# M2-A: the offline native-rate PHY closed loop (G0 §2).
#
# `uwb_twr_phy.h` is the PUBLIC M2-A entry.  It reuses the header-only
# hrp_mod / rational_resampler / demod cores under ../include, so this archive
# links VOLK + Threads and needs NO GNU Radio runtime.  The install consumer
# proves that with ldd/readelf (G0 §14.1); if libgnuradio-* ever appears here,
# the fallback is header-only + a consumer-only compile check.
########################################################################
add_library(uwb_twr_phy STATIC uwb_twr_phy.cc uwb_twr_phy_impairment.cc)
target_include_directories(uwb_twr_phy PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
    $<INSTALL_INTERFACE:include>)
target_link_libraries(uwb_twr_phy PUBLIC Volk::volk Threads::Threads uwb_core)
set_target_properties(uwb_twr_phy PROPERTIES POSITION_INDEPENDENT_CODE ON)
# uwb_twr_phy.h includes uwb_rational_resampler_core.h, so it must compile the
# header-only core with the same ISA as the rest of the build.
uwb_simd_apply(uwb_twr_phy)

########################################################################
# Install and export the native libraries.
install(TARGETS uwb_core uwb_native uwb_twr_core uwb_twr_phy
    EXPORT uwb-export
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(EXPORT uwb-export FILE uwbTargets.cmake NAMESPACE uwb::
    DESTINATION ${UWB_CMAKE_DIR})

########################################################################
# UHD backend dry-run CLI (Radar Step 11)
#
# Pure-contract printer: includes no UHD header and opens no device. Registered
# in CTest in both build configurations (UHD ON and OFF).
########################################################################
add_executable(uwb_uhd_burst_dry_run uwb_uhd_dry_run.cc)
target_compile_definitions(uwb_uhd_burst_dry_run PRIVATE UWB_NO_UHD_LINK=1)
target_link_libraries(uwb_uhd_burst_dry_run PRIVATE uwb_core)

add_test(NAME uwb_uhd_burst_dry_run
    COMMAND uwb_uhd_burst_dry_run
        --device-args addr=192.168.10.2
        --rate 737280000
        --tx-channel 0 --rx-channel 1
        --clock-source internal --time-source internal
        --pri-num 3686400 --pri-den 1
        --pre-guard-ticks 1475
        --tx-samples 140982 --rx-samples 55591
        --max-fragment-size 65536
        --t0-ticks 36864000
        --schedule-index 0)

add_test(NAME uwb_uhd_burst_dry_run_rejects_invalid_config
    COMMAND uwb_uhd_burst_dry_run --pri-num 0)
set_tests_properties(uwb_uhd_burst_dry_run_rejects_invalid_config
    PROPERTIES WILL_FAIL TRUE)
