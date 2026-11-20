# Copyright 2011-2020 Free Software Foundation, Inc.
# SPDX-License-Identifier: GPL-3.0-or-later

if(Python3_Interpreter_FOUND)
    add_test(NAME uwb_qa_twr_config_py
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_twr_config.py
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

    set_property(TEST uwb_qa_twr_config_py APPEND PROPERTY
        ENVIRONMENT
            "PYTHONPATH="
            "PYTHONDONTWRITEBYTECODE=1"
            "UWB_TWR_PYTHON_QA=ctest")
    set_tests_properties(uwb_qa_twr_config_py PROPERTIES
        TIMEOUT 300
        # A hand-edited checkout or a stale installed uwb package must not make
        # this test's result depend on run order, so it never runs alongside
        # anything else.
        RUN_SERIAL TRUE)

    message(STATUS "UWB CTest: registered uwb_qa_twr_config_py "
                   "(apps/test_twr_config.py)")
endif()

########################################################################
# M1-B: the offline two-endpoint simulation demo and its independent checks.
#
# `twr_fake_demo` links ONLY the standalone `uwb_twr_core` archive and the
# QA-only fake link, so the demo works without the radio runtime (M1-B §8).
#
# The Python demo/QA and the independent verifier are registered here so a
# green `ctest` covers B17 rather than only a hand run.  The demo never
# bypasses a capability gate: it runs with an explicit simulation envelope.
########################################################################
add_executable(twr_fake_demo twr_fake_demo.cc)
target_include_directories(twr_fake_demo PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../include)
target_link_libraries(twr_fake_demo PRIVATE uwb_twr_core uwb_twr_fake_link)

if(Python3_Interpreter_FOUND)
    add_test(NAME uwb_qa_twr_fake_demo_py
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_twr_fake_demo.py
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
    set_property(TEST uwb_qa_twr_fake_demo_py APPEND PROPERTY
        ENVIRONMENT
            "PYTHONPATH="
            "PYTHONDONTWRITEBYTECODE=1"
            "UWB_TWR_FAKE_DEMO=$<TARGET_FILE:twr_fake_demo>")
    set_tests_properties(uwb_qa_twr_fake_demo_py PROPERTIES TIMEOUT 300 RUN_SERIAL TRUE)

    # The independent verifier re-derives the ToF from the frame fields and the
    # transport model without calling the C++ at all.  It needs a real
    # request/output pair, generated in a scratch dir on the fly.
    add_test(NAME uwb_qa_twr_m1b_verify_py
        COMMAND sh -c
            "d=$(mktemp -d ${CMAKE_CURRENT_BINARY_DIR}/m1b_verify.XXXXXX) && \
${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/twr_fake_demo.py --scenario ds_unity \
    --request-out $d/req.json --output $d/out.json && \
${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/../../tools/twr/verify_m1_b_protocol.py \
    --request $d/req.json --output $d/out.json; rc=$?; rm -rf $d; exit $rc")
    set_property(TEST uwb_qa_twr_m1b_verify_py APPEND PROPERTY
        ENVIRONMENT
            "PYTHONPATH="
            "PYTHONDONTWRITEBYTECODE=1"
            "UWB_TWR_FAKE_DEMO=$<TARGET_FILE:twr_fake_demo>")
    set_tests_properties(uwb_qa_twr_m1b_verify_py PROPERTIES TIMEOUT 300 RUN_SERIAL TRUE)

    message(STATUS "UWB CTest: registered uwb_qa_twr_fake_demo_py + "
                   "uwb_qa_twr_m1b_verify_py (M1-B demo + verifier)")
endif()

set(UWB_NATIVE_INSTALL_PREFIX "" CACHE PATH
    "Installed UWB prefix for the consumer test (empty skips the test)")

########################################################################
# Native install-consumer gate (2026-11-15 native cleanup).
#
# run_native_install_consumer.sh compiles uwb_native_consumer.cc against an
# install prefix ONLY (no source tree, no build tree, no GR headers), links
# the installed uwb_native/uwb_core archives, runs the consumer, and proves
# with readelf/ldd that the result has NO libgnuradio-*/PMT dependency.
# Works after an install; empty prefix => exit 77 => SKIPPED.
########################################################################
if(UNIX)
    add_test(NAME uwb_qa_native_install_consumer
        COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/install_consumer/run_native_install_consumer.sh
                --prefix "${UWB_NATIVE_INSTALL_PREFIX}"
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
    set_tests_properties(uwb_qa_native_install_consumer PROPERTIES
        TIMEOUT 300
        SKIP_RETURN_CODE 77
        RUN_SERIAL TRUE)
    message(STATUS "UWB CTest: registered uwb_qa_native_install_consumer "
                   "(prefix='${UWB_NATIVE_INSTALL_PREFIX}'; empty => SKIP)")
endif()

########################################################################
# Native apps: link uwb_native/uwb_core only -- their transitive dynamic
# dependencies contain no libgnuradio-*/PMT (proven by the install consumer
# and by readelf on the built binaries).
########################################################################

# HRP TX modulator timing (P3: 64-SYNC PSDU<=127 B >= 1000 pkt/s)
add_executable(benchmark_hrp_mod benchmark_hrp_mod.cc)
target_link_libraries(benchmark_hrp_mod
    uwb_core
    Volk::volk
)
add_test(NAME uwb_benchmark_hrp_mod
    COMMAND benchmark_hrp_mod 4 32)
set_tests_properties(uwb_benchmark_hrp_mod PROPERTIES TIMEOUT 120)

add_executable(benchmark_energy_gate_formats benchmark_energy_gate_formats.cc)
target_link_libraries(benchmark_energy_gate_formats
    uwb_core
)

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
# M2-A: offline native-rate PHY demo + independent verifier (G0 §10).
#
# `twr_native_phy_demo` links ONLY the static `uwb_twr_phy` archive (+ its
# PUBLIC Volk/Threads), so a green run is itself
# evidence that the closed loop is radio-free.
########################################################################
add_executable(twr_native_phy_demo twr_native_phy_demo.cc)
target_include_directories(twr_native_phy_demo PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../include)
target_link_libraries(twr_native_phy_demo PRIVATE
    uwb_twr_phy Volk::volk Threads::Threads)
target_compile_definitions(twr_native_phy_demo PRIVATE
    "UWB_TESTDATA_DIR=\"${CMAKE_SOURCE_DIR}/testdata\"")

# The verifier re-derives the frame bytes / CRC / lengths / decode verdict from
# the manifest WITHOUT calling the C++, so it needs no build tree.  It runs the
# demo on the fly to get a real manifest and then checks it, plus --self-test.
if(Python3_Interpreter_FOUND)
    add_test(NAME uwb_qa_twr_m2a_native_phy_verify_py
        COMMAND sh -c
            "d=$(mktemp -d ${CMAKE_CURRENT_BINARY_DIR}/m2a_verify.XXXXXX) && \
$<TARGET_FILE:twr_native_phy_demo> --frame poll --native-rate 737280000 --iq cf32 \
    --output $d/out.json && \
${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/../../tools/twr/verify_m2_a_native_phy.py \
    --output $d/out.json --self-test; rc=$?; rm -rf $d; exit $rc")
    set_property(TEST uwb_qa_twr_m2a_native_phy_verify_py APPEND PROPERTY
        ENVIRONMENT
            "PYTHONPATH="
            "PYTHONDONTWRITEBYTECODE=1")
    set_tests_properties(uwb_qa_twr_m2a_native_phy_verify_py PROPERTIES
        TIMEOUT 300 RUN_SERIAL TRUE)
endif()

########################################################################
# M2-A closeout: the A13 allocation/latency observation tool (G0 A.4).
# An observation tool, not a timing gate; links only uwb_twr_phy.
########################################################################
add_executable(twr_m2a_bench twr_m2a_bench.cc)
target_include_directories(twr_m2a_bench PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../include)
target_link_libraries(twr_m2a_bench PRIVATE
    uwb_twr_phy Volk::volk Threads::Threads)
target_compile_definitions(twr_m2a_bench PRIVATE
    "UWB_TESTDATA_DIR=\"${CMAKE_SOURCE_DIR}/testdata\"")

########################################################################
# Historical benchmark / diagnostic EXPERIMENT apps (UWB_BUILD_EXPERIMENTS).
# Native-linked; they exist for A/B measurement, not production.
########################################################################
if(UWB_BUILD_EXPERIMENTS)
    add_executable(benchmark_fullpacket_compare benchmark_fullpacket_compare.cc)
    target_link_libraries(benchmark_fullpacket_compare uwb_native)

    if(Python3_Interpreter_FOUND)
        add_test(NAME uwb_qa_fullpacket_compare_py
            COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/../../tools/sic/qa_fullpacket_compare.py)
        set_tests_properties(uwb_qa_fullpacket_compare_py PROPERTIES TIMEOUT 60 RUN_SERIAL TRUE)

        add_test(NAME uwb_qa_discrete_cir_compare_py
            COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/../../tools/sic/verify_discrete_cir_compare.py --self-test)
        set_tests_properties(uwb_qa_discrete_cir_compare_py PROPERTIES TIMEOUT 60 RUN_SERIAL TRUE)
    endif()

    # Mature RX uses the existing AVX2/FMA FIR and compiler vectorization.
    # Keep the portable executable for same-source, same-library paired measurement.
    add_executable(benchmark_fullpacket_compare_fast_rx benchmark_fullpacket_compare.cc)
    target_link_libraries(benchmark_fullpacket_compare_fast_rx uwb_native)
    uwb_simd_apply(benchmark_fullpacket_compare_fast_rx)

    # Isolated generic-ISA CFO replay: diagnostic only, production kernel unchanged.
    add_executable(benchmark_fullpacket_cfo_diag benchmark_fullpacket_cfo_diag.cc)
    target_link_libraries(benchmark_fullpacket_cfo_diag uwb_native)
    target_compile_options(benchmark_fullpacket_cfo_diag PRIVATE -fPIC)

    # Offline-only copy of reconstruction, split by waveform fields for timing.
    add_executable(benchmark_fullpacket_segment_diag
        benchmark_fullpacket_segment_diag.cc fullpacket_segment_diagnostics.cc)
    target_link_libraries(benchmark_fullpacket_segment_diag uwb_native)
    target_compile_options(benchmark_fullpacket_segment_diag PRIVATE -fPIC)
    add_test(NAME uwb_qa_fullpacket_segment_diag
        COMMAND benchmark_fullpacket_segment_diag --self-test)
    set_tests_properties(uwb_qa_fullpacket_segment_diag PROPERTIES TIMEOUT 60 RUN_SERIAL TRUE)

    # Controlled tap-count sweep using the unchanged production discrete backend.
    add_executable(benchmark_discrete_cir_taps benchmark_discrete_cir_taps.cc)
    target_link_libraries(benchmark_discrete_cir_taps uwb_native)
    target_compile_options(benchmark_discrete_cir_taps PRIVATE -fPIC)

    # Unified SIC timing benchmark.
    add_executable(bench_unified_timing bench_unified_timing.cc)
    target_link_libraries(bench_unified_timing uwb_native)
    uwb_simd_apply(bench_unified_timing)

    # Phase-2 operator-level benchmark: one demod/SIC kernel at a time, with a
    # bit-identity gate against the AVX2 reference for every AVX-512 candidate.
    # Plan: docs/performance/解调与SIC_AVX512内核扩展开发规划_2026-11-16.md
    add_executable(bench_demod_sic_kernels bench_demod_sic_kernels.cc)
    target_link_libraries(bench_demod_sic_kernels uwb_native)
    uwb_simd_apply(bench_demod_sic_kernels)

    # Offline paired recovery experiment: desired complete packet + other-code
    # SYNC/SFD-only interference. Does not change the production pipeline.
    add_executable(experiment_preamble_recovery
        ../../tools/sic/experiment_preamble_recovery.cc)
    target_link_libraries(experiment_preamble_recovery uwb_native)
    uwb_simd_apply(experiment_preamble_recovery)

    add_executable(benchmark_chain_workers ../../tools/sic/benchmark_chain_workers.cc)
    target_link_libraries(benchmark_chain_workers uwb_native)
    uwb_simd_apply(benchmark_chain_workers)
else()
    message(STATUS "UWB_BUILD_EXPERIMENTS=OFF: skipping historical benchmark/"
                   "diagnostic experiment apps")
endif()

########################################################################
# UHD SC16 Direct RX and Multi-Core Stream Pipeline Runner
########################################################################
add_executable(uwb_uhd_sc16_pipeline_runner uwb_uhd_sc16_pipeline_runner.cc)
target_link_libraries(uwb_uhd_sc16_pipeline_runner uwb_native Threads::Threads)
uwb_simd_apply(uwb_uhd_sc16_pipeline_runner)
target_compile_definitions(uwb_uhd_sc16_pipeline_runner PRIVATE
    "UWB_TESTDATA_DIR=\"${CMAKE_SOURCE_DIR}/testdata\"")
if(UWB_HAVE_UHD)
    target_link_libraries(uwb_uhd_sc16_pipeline_runner UHD::UHD)
endif()

########################################################################
# UHD SC16 direct capture + native-grid packet analysis.
# Diagnostic counterpart of the GNU Radio capture flow: same UhdDeviceSc16Source
# the pipeline uses, then the native detector cores over the captured buffer.
########################################################################
add_executable(uwb_uhd_sc16_capture uwb_uhd_sc16_capture.cc)
target_link_libraries(uwb_uhd_sc16_capture uwb_native Threads::Threads)
uwb_simd_apply(uwb_uhd_sc16_capture)
if(UWB_HAVE_UHD)
    target_link_libraries(uwb_uhd_sc16_capture UHD::UHD)
endif()

########################################################################
# Round-2 review R2: the runner used to `return 0` unconditionally, so a
# failed run (no source, source start failure, broken ledger) looked like
# success to every caller.  It now exits non-zero when runtime_ok is false and
# keeps 0 for a normal EOF / stop after a successful start.
#
# The shell case pins the promise end to end:
#   1. nonexistent --file  -> non-zero (initialize failure)
#   2. one valid 56k window from the replay fixture -> 0, with
#      source_started=true, source_failed=false, ledger_invariants_ok=true.
# The start_stream() == false variant cannot be produced by a file source and
# is covered by the library QA (fake source with set_start_stream_result(false)).
#
add_test(NAME uwb_qa_sc16_pipeline_runner_exit
    COMMAND sh ${CMAKE_CURRENT_SOURCE_DIR}/qa_uwb_uhd_sc16_pipeline_runner_exit.sh
            --runner $<TARGET_FILE:uwb_uhd_sc16_pipeline_runner>
            --fixture ${CMAKE_SOURCE_DIR}/testdata/uhd_sc16/sim_preamble_sc16_737m28_rate.bin
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
set_tests_properties(uwb_qa_sc16_pipeline_runner_exit PROPERTIES TIMEOUT 120)

# 2026-11-11 round 2: pure-core resampler benchmark (owner: subagent A).
add_executable(benchmark_sc16_burst_core benchmark_sc16_burst_core.cc)
target_link_libraries(benchmark_sc16_burst_core uwb_native Threads::Threads)
uwb_simd_apply(benchmark_sc16_burst_core)
target_compile_definitions(benchmark_sc16_burst_core PRIVATE
    "UWB_TESTDATA_DIR=\"${CMAKE_SOURCE_DIR}/testdata\"")

# 2026-11-15: fixed-frequency CW cancellation.  The benchmark measures the
# production core (two-pass vs fused, ABBA); the replay driver runs the real
# source decorator over a raw SC16 capture and writes the clean stream for the
# independent Python reference.
add_executable(benchmark_fixed_tone_cancel benchmark_fixed_tone_cancel.cc)
target_link_libraries(benchmark_fixed_tone_cancel uwb_native Threads::Threads)
uwb_simd_apply(benchmark_fixed_tone_cancel)

add_executable(fixed_tone_cancel_replay fixed_tone_cancel_replay.cc)
target_link_libraries(fixed_tone_cancel_replay uwb_native Threads::Threads)
uwb_simd_apply(fixed_tone_cancel_replay)

# 2026-11-11 round 2: steady-state pipeline benchmark driver (owner: main agent).
add_executable(benchmark_sc16_pipeline_steady benchmark_sc16_pipeline_steady.cc)
target_link_libraries(benchmark_sc16_pipeline_steady uwb_native Threads::Threads)
uwb_simd_apply(benchmark_sc16_pipeline_steady)
target_compile_definitions(benchmark_sc16_pipeline_steady PRIVATE
    "UWB_TESTDATA_DIR=\"${CMAKE_SOURCE_DIR}/testdata\"")
if(UWB_HAVE_UHD)
    target_link_libraries(benchmark_sc16_pipeline_steady UHD::UHD)
endif()

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

add_executable(uwb_dw3000_live uwb_dw3000_live.cc)
target_link_libraries(uwb_dw3000_live uwb_native Threads::Threads)
uwb_simd_apply(uwb_dw3000_live)
target_compile_definitions(uwb_dw3000_live PRIVATE
    "UWB_TESTDATA_DIR=\"${CMAKE_SOURCE_DIR}/testdata\""
    "UWB_INSTALL_RESOURCE_DIR=\"${CMAKE_INSTALL_PREFIX}/${UWB_DATA_DIR}\"")
if(UWB_HAVE_UHD)
    target_link_libraries(uwb_dw3000_live UHD::UHD)
endif()
if(Python3_Interpreter_FOUND)
    add_test(NAME uwb_qa_dw3000_live_console
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_SOURCE_DIR}/tools/rx/qa_live_dw3000_console.py
                $<TARGET_FILE:uwb_dw3000_live>)
    set_tests_properties(uwb_qa_dw3000_live_console PROPERTIES TIMEOUT 90)
endif()

########################################################################
# Install the native application entry points.
########################################################################
install(TARGETS uwb_uhd_sc16_pipeline_runner uwb_uhd_sc16_capture
              uwb_dw3000_live DESTINATION ${CMAKE_INSTALL_BINDIR})
