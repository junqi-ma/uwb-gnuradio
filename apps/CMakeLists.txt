# Copyright 2011-2020 Free Software Foundation, Inc.
# SPDX-License-Identifier: GPL-3.0-or-later

########################################################################
# Native application entry points and operator tools.
#
# Phase 1 moved the benchmarks and the integration harness to tests/; what
# stays here is what ships (or what is a reusable operator tool next to the
# app it drives): the installed binaries, the two TWR demos, the replay driver,
# and the two experiment executables whose sources live in tools/sic/.
#
# Phase 3 registers the official `openuwb_*` targets.  The pre-3 names survive
# as ALIAS targets (so $<TARGET_FILE:old> in tests still resolves) and as
# post-build symlinks (so tools/ and documented commands keep finding a file
# named uwb_dw3000_live in the build tree).
########################################################################

# TODO(openuwb-compat-removal): drop the legacy target names and symlinks once
# tools/, tests/ and the docs use the openuwb_* names.
function(openuwb_compat_symlink target legacy_name)
    if(NOT CMAKE_HOST_UNIX)
        return()
    endif()
    add_custom_command(TARGET ${target} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E remove -f
                $<TARGET_FILE_DIR:${target}>/${legacy_name}
        COMMAND ${CMAKE_COMMAND} -E create_symlink
                $<TARGET_FILE_NAME:${target}>
                $<TARGET_FILE_DIR:${target}>/${legacy_name}
        VERBATIM
        COMMENT "compat symlink ${legacy_name} -> $<TARGET_FILE_NAME:${target}>")
    # Same name in the install prefix, for scripts that still call it.
    install(CODE "execute_process(COMMAND \"${CMAKE_COMMAND}\" -E create_symlink \"${legacy_name}\" \"\$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_BINDIR}/${legacy_name}\")")
endfunction()

########################################################################
# M1-B: the offline two-endpoint simulation demo.
#
# `twr_fake_demo` links ONLY the standalone `uwb_twr_core` archive and the
# QA-only fake link, so the demo works without the radio runtime (M1-B §8).
# Its Python driver and the independent verifier live in tests/integration/.
########################################################################
add_executable(openuwb_twr_fake_demo twr_fake_demo.cc)
target_include_directories(openuwb_twr_fake_demo PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../include)
target_link_libraries(openuwb_twr_fake_demo PRIVATE uwb_twr_core uwb_twr_fake_link)
# TODO(openuwb-compat-removal): legacy target/binary name used by tests/ scripts.
add_executable(twr_fake_demo ALIAS openuwb_twr_fake_demo)
openuwb_compat_symlink(openuwb_twr_fake_demo twr_fake_demo)

########################################################################
# M2-A: offline native-rate PHY demo (G0 §10).
#
# `twr_native_phy_demo` links ONLY the static `uwb_twr_phy` archive (+ its
# PUBLIC Volk/Threads), so a green run is itself evidence that the closed loop
# is radio-free.  The independent verifier lives in tests/integration/.
########################################################################
add_executable(openuwb_twr_native_phy_demo twr_native_phy_demo.cc)
target_include_directories(openuwb_twr_native_phy_demo PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../include)
target_link_libraries(openuwb_twr_native_phy_demo PRIVATE
    uwb_twr_phy Volk::volk Threads::Threads)
target_compile_definitions(openuwb_twr_native_phy_demo PRIVATE
    "UWB_TESTDATA_DIR=\"${CMAKE_SOURCE_DIR}/testdata\"")
# TODO(openuwb-compat-removal): legacy target/binary name used by tests/ scripts.
add_executable(twr_native_phy_demo ALIAS openuwb_twr_native_phy_demo)
openuwb_compat_symlink(openuwb_twr_native_phy_demo twr_native_phy_demo)

########################################################################
# Offline dump postprocess: per-window notch + 65/48 (no UHD, no work())
########################################################################
add_executable(uwb_offline_postprocess_dump uwb_offline_postprocess_dump.cc)
target_link_libraries(uwb_offline_postprocess_dump
    uwb_native
    Threads::Threads
)
install(TARGETS uwb_offline_postprocess_dump DESTINATION ${CMAKE_INSTALL_BINDIR})

########################################################################
# Historical benchmark / diagnostic EXPERIMENT apps whose sources live in
# tools/sic/.  The measurement rigs themselves moved to tests/benchmarks/.
########################################################################
if(UWB_BUILD_EXPERIMENTS)
    # Offline paired recovery experiment: desired complete packet + other-code
    # SYNC/SFD-only interference. Does not change the production pipeline.
    add_executable(experiment_preamble_recovery
        ${CMAKE_SOURCE_DIR}/tools/sic/experiment_preamble_recovery.cc)
    target_link_libraries(experiment_preamble_recovery uwb_native)
    uwb_simd_apply(experiment_preamble_recovery)

    add_executable(benchmark_chain_workers
        ${CMAKE_SOURCE_DIR}/tools/sic/benchmark_chain_workers.cc)
    target_link_libraries(benchmark_chain_workers uwb_native)
    uwb_simd_apply(benchmark_chain_workers)

    # Self-checks of the tools/sic comparison scripts: they test the scripts,
    # not a moved source, so they stay next to the experiment targets.
    if(Python3_Interpreter_FOUND)
        add_test(NAME uwb_qa_fullpacket_compare_py
            COMMAND ${Python3_EXECUTABLE} ${CMAKE_SOURCE_DIR}/tools/sic/qa_fullpacket_compare.py)
        set_tests_properties(uwb_qa_fullpacket_compare_py PROPERTIES TIMEOUT 60 RUN_SERIAL TRUE)

        add_test(NAME uwb_qa_discrete_cir_compare_py
            COMMAND ${Python3_EXECUTABLE} ${CMAKE_SOURCE_DIR}/tools/sic/verify_discrete_cir_compare.py --self-test)
        set_tests_properties(uwb_qa_discrete_cir_compare_py PROPERTIES TIMEOUT 60 RUN_SERIAL TRUE)
    endif()
else()
    message(STATUS "UWB_BUILD_EXPERIMENTS=OFF: skipping historical benchmark/"
                   "diagnostic experiment apps")
endif()

########################################################################
# UHD SC16 Direct RX and Multi-Core Stream Pipeline Runner
########################################################################
add_executable(openuwb_pipeline_runner uwb_uhd_sc16_pipeline_runner.cc)
target_link_libraries(openuwb_pipeline_runner uwb_native Threads::Threads)
uwb_simd_apply(openuwb_pipeline_runner)
target_compile_definitions(openuwb_pipeline_runner PRIVATE
    "UWB_TESTDATA_DIR=\"${CMAKE_SOURCE_DIR}/testdata\"")
if(UWB_HAVE_UHD)
    target_link_libraries(openuwb_pipeline_runner UHD::UHD)
endif()
# TODO(openuwb-compat-removal): legacy target/binary name used by tools/ and QA.
add_executable(uwb_uhd_sc16_pipeline_runner ALIAS openuwb_pipeline_runner)
openuwb_compat_symlink(openuwb_pipeline_runner uwb_uhd_sc16_pipeline_runner)

########################################################################
# UHD SC16 direct capture + native-grid packet analysis.
# Diagnostic counterpart of the GNU Radio capture flow: same UhdDeviceSc16Source
# the pipeline uses, then the native detector cores over the captured buffer.
########################################################################
add_executable(openuwb_capture uwb_uhd_sc16_capture.cc)
target_link_libraries(openuwb_capture uwb_native Threads::Threads)
uwb_simd_apply(openuwb_capture)
if(UWB_HAVE_UHD)
    target_link_libraries(openuwb_capture UHD::UHD)
endif()
# TODO(openuwb-compat-removal): legacy target/binary name.
add_executable(uwb_uhd_sc16_capture ALIAS openuwb_capture)
openuwb_compat_symlink(openuwb_capture uwb_uhd_sc16_capture)

# 2026-11-15: fixed-frequency CW cancellation replay driver.  It runs the real
# source decorator over a raw SC16 capture and writes the clean stream for the
# independent Python reference (its paired benchmark lives in tests/benchmarks).
add_executable(fixed_tone_cancel_replay fixed_tone_cancel_replay.cc)
target_link_libraries(fixed_tone_cancel_replay uwb_native Threads::Threads)
uwb_simd_apply(fixed_tone_cancel_replay)

# Live DW3000 packet console (native UHD, bounded workers, no GR scheduler).
#
# Runtime resources (resampler taps, preamble template) are resolved at run
# time: --res-dir > $UWB_RESOURCE_DIR > $UWB_TESTDATA_DIR > the installed
# resource dir (UWB_INSTALL_RESOURCE_DIR) > the build-tree testdata fallback,
# so an installed binary never depends on a source checkout / old build tree.
# Offline decoder shares the compiled packet entry point and ISA configuration.
add_executable(uwb_dw3000_decode ${CMAKE_SOURCE_DIR}/tools/rx/decode_dw3000_packets.cc)
target_link_libraries(uwb_dw3000_decode uwb_core Threads::Threads)
uwb_simd_apply(uwb_dw3000_decode)

# Phase 1 split (1E): CLI/config, stdout telemetry and the timed replay source
# are separate translation units; this target is still one binary.
add_executable(openuwb_rx
    uwb_dw3000_live.cc
    live_rx_config.cc
    console_reporter.cc
    live_rx_session.cc
)
target_link_libraries(openuwb_rx uwb_native Threads::Threads)
uwb_simd_apply(openuwb_rx)
target_compile_definitions(openuwb_rx PRIVATE
    "UWB_TESTDATA_DIR=\"${CMAKE_SOURCE_DIR}/testdata\""
    "UWB_INSTALL_RESOURCE_DIR=\"${CMAKE_INSTALL_PREFIX}/${UWB_DATA_DIR}\"")
if(UWB_HAVE_UHD)
    target_link_libraries(openuwb_rx UHD::UHD)
endif()
# TODO(openuwb-compat-removal): legacy target/binary name used by the console
# QA, tools/rx scripts and the documented command line.
add_executable(uwb_dw3000_live ALIAS openuwb_rx)
openuwb_compat_symlink(openuwb_rx uwb_dw3000_live)
# The console contract of the installed receiver stays next to the binary it
# pins; the script itself lives in tools/rx/ (unchanged in Phase 1).
if(Python3_Interpreter_FOUND)
    add_test(NAME uwb_qa_dw3000_live_console
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_SOURCE_DIR}/tools/rx/qa_live_dw3000_console.py
                $<TARGET_FILE:uwb_dw3000_live>)
    set_tests_properties(uwb_qa_dw3000_live_console PROPERTIES TIMEOUT 90)
endif()

########################################################################
# Install the native application entry points.
########################################################################
install(TARGETS openuwb_pipeline_runner openuwb_capture openuwb_rx
    DESTINATION ${CMAKE_INSTALL_BINDIR})
