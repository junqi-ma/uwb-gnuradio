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
    uwb_defaults.h
    uwb_detector_core.h
    uwb_detector.h
    uwb_detector_sc16.h
    uwb_energy_detector.h
    uwb_packet_writer.h
    uwb_preamble_detector.h
    uwb_scheduled_extractor_core.h
    uwb_scheduled_extractor.h
    uwb_scheduled_extractor_sc16.h
    uwb_preamble_verifier_sc16.h
    uwb_qm35_acquisition_tracker.h
    uwb_auto_scheduled_extractor_sc16.h
    uwb_rational_resampler_core.h
    uwb_rational_resampler_ccf_65_48.h
    uwb_pdu_rational_resampler_ccf_65_48.h
    uwb_radar_pdu_meta.h
    uwb_window_crop_core.h
    uwb_pdu_window_crop.h
    uwb_tone_canceller.h
    uwb_scheduled_dump_io.h
    uwb_radar_checked_math.h
    uwb_radar_sfd_core.h
    uwb_radar_cir_estimator.h
    uwb_radar_timing_core.h
    uwb_radar_cir_core.h
    uwb_radar_packet_source.h
    uwb_loopback_echo.h
    uwb_radar_cir_estimator_block.h
    uwb_cir_writer.h
    uwb_echo_scheduler_core.h
    uwb_echo_burst_backend.h
    uwb_fake_burst_backend.h
    uwb_realtime_echo_timer.h
    uwb_uhd_backend_config.h
    uwb_uhd_burst_backend.h
    DESTINATION include/gnuradio/uwb
)
