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
    uwb_detector_core.h
    uwb_detector.h
    uwb_detector_sc16.h
    uwb_energy_detector.h
    uwb_packet_writer.h
    uwb_preamble_detector.h
    uwb_scheduled_extractor_core.h
    uwb_scheduled_extractor.h
    uwb_sic_core.h
    DESTINATION include/gnuradio/uwb
)
