# Copyright 2011,2012 Free Software Foundation, Inc.
#
# This file is part of gr-uwb
#
# SPDX-License-Identifier: GPL-3.0-or-later
#

########################################################################
# Install public header files
########################################################################
install(FILES
    api.h
    uwb_gr_complex.h
    uwb_radar_meta_core.h
    uwb_sc16_stream_source.h
    uwb_defaults.h
    uwb_detector_core.h
    uwb_scheduled_extractor_core.h
    uwb_preamble_verifier_sc16.h
    uwb_qm35_acquisition_tracker.h
    uwb_rational_resampler_core.h
    uwb_rational_resampler_avx512.h
    uwb_simd.h
    uwb_kernel_backend.h
    uwb_demod_sic_avx512.h
    uwb_sc16_scale.h
    uwb_discrete_cir_sic.h
    uwb_window_crop_core.h
    uwb_tone_canceller.h
    uwb_scheduled_dump_io.h
    uwb_radar_checked_math.h
    uwb_radar_sfd_core.h
    uwb_radar_cir_estimator.h
    uwb_radar_timing_core.h
    uwb_radar_cir_core.h
    uwb_echo_scheduler_core.h
    uwb_echo_burst_backend.h
    uwb_fake_burst_backend.h
    uwb_uhd_backend_config.h
    uwb_uhd_burst_backend.h
    uwb_hrp_mod_core.h
    uwb_hrp_codec.h
    uwb_resample_window.h
    uwb_demod_scratch.h
    uwb_demod_grid.h
    uwb_demod_sync.h
    uwb_demod_timing.h
    uwb_demod_cfo.h
    uwb_demod_sfd.h
    uwb_demod_softchips.h
    uwb_demod_fec.h
    uwb_demod_phr.h
    uwb_demod_payload.h
    uwb_demod_pipeline.h
    uwb_demod_execution.h
    uwb_demod_finalize.h
    # TWR M0 contract layer.  These six headers ARE the public API for
    # configuring a TWR exchange from C++ (frame profile, timestamp/ToF
    # arithmetic, typed configuration + capability whitelist) AND for
    # admitting a raw measurement into a ranging input (the M0.1 R6 gate).
    # They were never installed, so the installed module could not include
    # them and an installed consumer had no way to reach the M0 contract at
    # all (M0 review R8 / P2).
    #
    # uwb_twr_capability_evidence.h (M0.1) grades capability claims;
    # uwb_twr_tof_input.h (M0.1, R6) is the ranging-admission entry point
    # (`admit_range_capable_time` / `admit_ranging_interval`) that M1-A will
    # call.  It was accidentally left out of this list even though the other
    # five TWR headers were installed, so an isolated install could not
    # compile a consumer that used the publicly documented gate at all
    # (review defect N05).  It is installed here.
    #
    # uwb_twr_test_output.h is deliberately NOT installed, and this is a
    # decision, not an omission.  It is QA-only output-directory policy (it
    # picks where a QA may write generated CSV), not part of the TWR
    # contract a consumer links against.  More concretely, it expands
    # `UWB_TESTDATA_DIR` with no default (see `source_twr_dir()`), so a
    # consumer that includes it from an isolated prefix WITHOUT the project's
    # `target_compile_definitions()` fails to compile.  Shipping a public
    # header that cannot compile standalone is worse than not shipping it;
    # the TWR QAs still include it from the source tree.  The installed
    # consumer's header check was updated to match.
    uwb_twr_types.h
    uwb_twr_frame.h
    uwb_twr_timestamp.h
    uwb_twr_config.h
    uwb_twr_capability_evidence.h
    uwb_twr_tof_input.h
    # M1-A: the SS/DS ToF mathematics (ClockRatio, compute_ss_tof,
    # compute_ds_tof).  Header-only, UHD-free and config-free -- it depends
    # only on the TWR contract headers above, so it installs cleanly and an
    # isolated consumer can compute a ToF from admitted intervals.
    uwb_twr_math.h
    # M1-B: the wire-claim boundary and the per-endpoint protocol core.  Both
    # are pure C++ (no GNU Radio, no UHD, no config business rules) and are the
    # PUBLIC M1-B contract, so they are installed and exercised by the isolated
    # install consumer.  `uwb_twr_protocol_time.h` is a strict superset of the
    # M0.1 ranging gate contract; `uwb_twr_core.h` declares the endpoint FSM
    # whose implementation ships in the standalone `uwb_twr_core` archive.
    #
    # uwb_twr_fake_link.h is deliberately NOT installed: it is QA/demo support
    # only (M1-B §5/§8).
    uwb_twr_protocol_time.h
    uwb_twr_core.h
    # Public result/profile/SIMD dependencies of the packet API and legacy
    # stage aggregate. The compiled entry point is supplied by uwb_core.
    uwb_demod_core.h
    uwb_demod_result.h
    uwb_phy_profile.h
    uwb_cir_fir_simd.h
    # Experimental RX fast-path helpers included by uwb_demod_core.h.
    # They are header-only and must ship with the core header so an isolated
    # installed consumer still compiles.
    uwb_rx_sync_opt.h
    uwb_rx_cfo_opt.h
    uwb_rx_softchip_opt.h
    uwb_rx_fused_cmf.h
    uwb_softchip_fir_diag.h
    # M2-A: the offline native-rate PHY closed loop.  Header-only entry that
    # reuses the hrp_mod / resampler / demod cores above, so it installs and an
    # isolated consumer can drive a full-frame native round trip.
    uwb_twr_phy.h
    uwb_fullpacket_sic_core.h
    uwb_preamble_sic_core.h
    uwb_sic_unified.h
    # Native RX runtime headers (2026-11-15 cleanup): the pipeline, its
    # minimal SC16 source contract, the fixed-tone decorator + core, and the
    # target-lock / timed-plan cores are installed so an isolated native
    # consumer can build against the prefix alone.
    uwb_uhd_sc16_pipeline.h
    uwb_fixed_tone_cancel_core.h
    uwb_fixed_tone_cancel_source.h
    uwb_target_lock_core.h
    uwb_timed_rx_plan.h
    DESTINATION include/gnuradio/uwb
)

# The compile-time SIMD backend header is GENERATED by CMake (UwbSimd.cmake)
# into the build tree.  Install that generated copy, not the .in template, so
# an installed consumer sees the same backend the library was built with.
install(FILES
    ${PROJECT_BINARY_DIR}/include/gnuradio/uwb/uwb_simd_config.h
    DESTINATION include/gnuradio/uwb
)
