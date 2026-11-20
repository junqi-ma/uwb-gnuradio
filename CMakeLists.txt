# Copyright 2011-2020 Free Software Foundation, Inc.
# SPDX-License-Identifier: GPL-3.0-or-later

cmake_minimum_required(VERSION 3.8)
# Phase 3: the repository root is the project.  `OpenUWB` is the package name
# consumers find; targets keep the `openuwb_` prefix (see lib/CMakeLists.txt).
project(OpenUWB LANGUAGES CXX)
enable_testing()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Catch an old build command that explicitly asks for the retired adapter.
if(UWB_BUILD_GNURADIO)
    message(FATAL_ERROR "The GNU Radio adapter is archived on archive/gnuradio-adapter. Configure this branch in a fresh native build directory.")
endif()

include(GNUInstallDirs)
# Runtime resources (taps, preamble template) live under share/openuwb; the
# lowercase directory is deliberate, `share/OpenUWB` would look like a casing
# accident on case-sensitive filesystems.
set(UWB_DATA_DIR ${CMAKE_INSTALL_DATADIR}/openuwb)
# Primary package: find_package(OpenUWB CONFIG REQUIRED) + openuwb::* targets.
set(OPENUWB_CMAKE_DIR ${CMAKE_INSTALL_LIBDIR}/cmake/OpenUWB)
# Legacy package kept for existing consumers.
# TODO(openuwb-compat-removal): drop the uwb package once downstream migrated.
set(UWB_CMAKE_DIR ${CMAKE_INSTALL_LIBDIR}/cmake/uwb)

find_package(Volk REQUIRED)
find_package(Threads REQUIRED)
find_package(Boost REQUIRED COMPONENTS unit_test_framework)
find_package(Python3 COMPONENTS Interpreter QUIET)

# Select the release build type by default to get optimization flags
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
    message(STATUS "Build type not specified: defaulting to release.")
endif(NOT CMAKE_BUILD_TYPE)
set(CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE} CACHE STRING "")

option(UWB_BUILD_EXPERIMENTS
    "Build historical benchmark/diagnostic experiment apps" ON)

# Make sure our local CMake Modules path comes first
list(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules)

########################################################################
# Compile-time ISA selection for the self-written DSP kernels
# (UWB_SIMD=AUTO|AVX2|AVX512|SCALAR).  See cmake/Modules/UwbSimd.cmake and
# docs/performance/AVX512内核与编译期CPU选择开发规划_2026-11-16.md.
########################################################################
include(UwbSimd)
set(_uwb_sparse_mf_avx512_default OFF)
if(UWB_SIMD_HAVE_AVX512)
    set(_uwb_sparse_mf_avx512_default ON)
endif()
option(UWB_SPARSE_MF_AVX512_16LAG
    "Use AVX-512 16-lag batching for sparse ternary matched filtering when AVX-512 is enabled"
    ${_uwb_sparse_mf_avx512_default})
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/include/openuwb/support/simd_config.h.in
    ${CMAKE_CURRENT_BINARY_DIR}/include/openuwb/support/simd_config.h
    @ONLY)
# The generated header must be visible to every subdirectory (and to the
# installed consumer, which gets the same file from the install rule).
include_directories(${CMAKE_CURRENT_BINARY_DIR}/include)

if(APPLE)
    if(NOT CMAKE_INSTALL_NAME_DIR)
        set(CMAKE_INSTALL_NAME_DIR
            ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR} CACHE
            PATH "Library Install Name Destination Directory" FORCE)
    endif(NOT CMAKE_INSTALL_NAME_DIR)
    if(NOT CMAKE_INSTALL_RPATH)
        set(CMAKE_INSTALL_RPATH
            ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR} CACHE
            PATH "Library Install RPath" FORCE)
    endif(NOT CMAKE_INSTALL_RPATH)
    if(NOT CMAKE_BUILD_WITH_INSTALL_RPATH)
        set(CMAKE_BUILD_WITH_INSTALL_RPATH ON CACHE
            BOOL "Do Build Using Library Install RPath" FORCE)
    endif(NOT CMAKE_BUILD_WITH_INSTALL_RPATH)
endif(APPLE)

########################################################################
# Optional UHD backend discovery (AUTO/ON/OFF
# semantics).  Sets UWB_HAVE_UHD and makes the UHD::UHD imported target.
########################################################################
set(ENABLE_UHD_BACKEND "AUTO" CACHE STRING
    "Build the UHD device backends: AUTO (build when UHD found), ON (require UHD), OFF (never)")
set_property(CACHE ENABLE_UHD_BACKEND PROPERTY STRINGS AUTO ON OFF)
string(TOUPPER "${ENABLE_UHD_BACKEND}" ENABLE_UHD_BACKEND_UC)
if(NOT ENABLE_UHD_BACKEND_UC MATCHES "^(AUTO|ON|OFF)$")
    message(FATAL_ERROR
        "ENABLE_UHD_BACKEND must be AUTO, ON or OFF "
        "(got '${ENABLE_UHD_BACKEND}')")
endif()

set(UWB_HAVE_UHD 0)
if(NOT ENABLE_UHD_BACKEND_UC STREQUAL "OFF")
    find_package(UHD CONFIG QUIET)
    if(NOT UHD_FOUND)
        find_package(PkgConfig QUIET)
        if(PkgConfig_FOUND)
            pkg_check_modules(UHD_PC QUIET uhd)
        endif()
    endif()
    if(UHD_FOUND OR UHD_PC_FOUND)
        set(UWB_HAVE_UHD 1)
    elseif(ENABLE_UHD_BACKEND_UC STREQUAL "ON")
        message(FATAL_ERROR
            "ENABLE_UHD_BACKEND=ON but UHD was not found "
            "(install libuhd-dev / UHD CMake or pkg-config support). "
            "Use ENABLE_UHD_BACKEND=AUTO to skip the backend instead.")
    else()
        message(STATUS
            "UHD not found: UHD device backends disabled; the native replay "
            "chain, library and QA build and test without UHD.")
    endif()
endif()
if(UWB_HAVE_UHD AND NOT TARGET UHD::UHD)
    add_library(UHD::UHD INTERFACE IMPORTED GLOBAL)
    if(UHD_INCLUDE_DIRS)
        set_target_properties(UHD::UHD PROPERTIES
            INTERFACE_INCLUDE_DIRECTORIES "${UHD_INCLUDE_DIRS}")
    endif()
    set_target_properties(UHD::UHD PROPERTIES
        INTERFACE_LINK_LIBRARIES "${UHD_LIBRARIES};${UHD_PC_LIBRARIES}")
endif()
message(STATUS "UHD backends (ENABLE_UHD_BACKEND=${ENABLE_UHD_BACKEND_UC}): ${UWB_HAVE_UHD}")

if(UWB_HAVE_UHD)
    # The exported openuwb::uwb_native references UHD::UHD; the installed config
    # must define that target even when the consumer's UHD package does not
    # (some UHD packages only set UHD_LIBRARIES/UHD_INCLUDE_DIRS or pkg-config).
    # \${...} escapes keep the consumer-time ${CMAKE_CURRENT_LIST_DIR} literal.
    set(OPENUWB_CONFIG_UHD_DEP
        "include(\"\${CMAKE_CURRENT_LIST_DIR}/OpenUWBConfigUhdDep.cmake\")")
else()
    set(OPENUWB_CONFIG_UHD_DEP "# this build has no UHD dependency")
endif()


configure_file(cmake/cmake_uninstall.cmake.in cmake_uninstall.cmake @ONLY)
add_custom_target(uninstall
    ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)

add_subdirectory(include/gnuradio/uwb)
add_subdirectory(lib)
add_subdirectory(apps)
# Phase 1: the unit QA, benchmarks and integration harness live in tests/.
# Registered last so it can reference the app targets it drives
# (twr_fake_demo, twr_native_phy_demo, uwb_uhd_sc16_pipeline_runner).
add_subdirectory(tests)

include(CMakePackageConfigHelpers)
# Primary package: find_package(OpenUWB CONFIG REQUIRED) + openuwb::* targets.
configure_package_config_file(
    cmake/Modules/OpenUWBConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/OpenUWBConfig.cmake
    INSTALL_DESTINATION ${OPENUWB_CMAKE_DIR})
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/OpenUWBConfig.cmake
    cmake/Modules/OpenUWBConfigUhdDep.cmake
    DESTINATION ${OPENUWB_CMAKE_DIR})

# Legacy package name.  Same exported targets, with the old uwb:: spellings
# re-created on top.
# TODO(openuwb-compat-removal): remove the uwb package and its config.
configure_package_config_file(
    cmake/Modules/uwbConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/uwbConfig.cmake
    INSTALL_DESTINATION ${UWB_CMAKE_DIR})
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/uwbConfig.cmake
    DESTINATION ${UWB_CMAKE_DIR})

# Install the runtime resources the native live receiver needs (resampler
# taps + preamble template), so an INSTALLED uwb_dw3000_live does not depend
# on a source checkout, an old build tree, RPATH or PYTHONPATH.
########################################################################
install(FILES
    ${CMAKE_SOURCE_DIR}/testdata/resampler_65_48/taps_quality_minorder.txt
    ${CMAKE_SOURCE_DIR}/testdata/resampler_65_48/taps_quality_eq32.txt
    ${CMAKE_SOURCE_DIR}/testdata/resampler_65_48/taps_realtime_minorder.txt
    ${CMAKE_SOURCE_DIR}/testdata/resampler_65_48/taps_realtime_eq16.txt
    DESTINATION ${UWB_DATA_DIR}/resampler_65_48
)
install(FILES
    ${CMAKE_SOURCE_DIR}/testdata/reference_preamble_code10_998p4.cf32
    DESTINATION ${UWB_DATA_DIR}
)
