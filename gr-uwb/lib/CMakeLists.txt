# Copyright 2011-2020 Free Software Foundation, Inc.
# SPDX-License-Identifier: GPL-3.0-or-later

########################################################################
# Phase 2: one static library per domain, with a strictly one-way DAG.
#
#   openuwb_support      Threads, Volk            (baseline ISA, no UHD)
#   openuwb_dsp       -> support
#   openuwb_phy_hrp   -> dsp
#   openuwb_sic       -> phy_hrp
#   openuwb_sensing   -> phy_hrp                  (INTERFACE, header-only)
#   openuwb_twr_core  -> phy_hrp
#   openuwb_twr_fake_link -> twr_core             (QA transport, not installed)
#   openuwb_twr_phy   -> twr_core, dsp
#   openuwb_radio     -> dsp                      (+ UHD when available)
#   openuwb_runtime   -> phy_hrp, sic, sensing, radio
#
# Nothing below may depend on UHD; only openuwb_radio does, and only when
# ENABLE_UHD_BACKEND did not turn it off.
#
# uwb_core / uwb_native are compatibility aggregates kept for downstream and
# for the install export.
# TODO(openuwb-compat-removal): drop the aggregates once consumers link the
# domain targets directly.
########################################################################

add_subdirectory(support)
add_subdirectory(dsp)
add_subdirectory(phy/hrp)
add_subdirectory(sic)
add_subdirectory(sensing)
add_subdirectory(twr)
add_subdirectory(radio)
add_subdirectory(runtime)

########################################################################
# Compatibility aggregates.
#
# A static archive needs a source, so both aggregates compile one empty TU: all
# of their content is the PUBLIC link interface, which also carries the SIMD
# compile flags so a header-only consumer instantiates the same kernels.
########################################################################
set(OPENUWB_COMPAT_TU ${CMAKE_CURRENT_BINARY_DIR}/openuwb_compat_empty.cc)
file(WRITE ${OPENUWB_COMPAT_TU}
    "// Phase 2 compatibility aggregate: see lib/CMakeLists.txt.\n")

add_library(uwb_core STATIC ${OPENUWB_COMPAT_TU})
target_link_libraries(uwb_core PUBLIC
    openuwb_support openuwb_dsp openuwb_phy_hrp openuwb_sic openuwb_sensing)
set_target_properties(uwb_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
uwb_simd_apply(uwb_core)

add_library(uwb_native STATIC ${OPENUWB_COMPAT_TU})
target_link_libraries(uwb_native PUBLIC openuwb_runtime openuwb_radio)
set_target_properties(uwb_native PROPERTIES POSITION_INDEPENDENT_CODE ON)
uwb_simd_apply(uwb_native)

########################################################################
# Install and export the native libraries.
########################################################################
install(TARGETS openuwb_support openuwb_dsp openuwb_phy_hrp openuwb_sic
                openuwb_twr_core openuwb_twr_phy openuwb_radio openuwb_runtime
                uwb_core uwb_native
    EXPORT uwb-export
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
# INTERFACE libraries take no destination.
install(TARGETS openuwb_sensing EXPORT uwb-export)
install(EXPORT uwb-export FILE uwbTargets.cmake NAMESPACE uwb::
    DESTINATION ${UWB_CMAKE_DIR})

########################################################################
# UHD backend dry-run CLI (Radar Step 11)
#
# Pure-contract printer: includes no UHD header and opens no device. Registered
# in CTest in both build configurations (UHD ON and OFF).
########################################################################
add_executable(uwb_uhd_burst_dry_run radio/uwb_uhd_dry_run.cc)
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
