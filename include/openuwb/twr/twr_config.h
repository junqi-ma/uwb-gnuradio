/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * TWR configuration, capability whitelist, validator, frozen effective config
 * and JSON import/export (M0 / REQ-API-01..05).
 *
 * Phase 2 split: this is the stable aggregate entry point.  The content lives
 * in eight ordered sub-headers, one per responsibility:
 *
 *   twr_config_util.h         small utilities (Opt, rate check)
 *   twr_config_reasons.h      rejection reasons + validation report
 *   twr_config_timed.h        timed fields (event/marker/unit)
 *   twr_config_capabilities.h default-deny capability whitelist
 *   twr_config_types.h        the typed configuration structs
 *   twr_config_fields.h       canonical field listing + enum-domain checks
 *   twr_config_validator.h    the validator
 *   twr_config_effective.h    frozen effective config + JSON round trip
 *
 * The dependency is still ONE WAY: this file is header-only, stdlib-only and
 * includes no GNU Radio, UHD or pmt header.
 */
#ifndef INCLUDED_GNURADIO_UWB_UWB_TWR_CONFIG_H
#define INCLUDED_GNURADIO_UWB_UWB_TWR_CONFIG_H

#include <openuwb/twr/twr_config_util.h>
#include <openuwb/twr/twr_config_reasons.h>
#include <openuwb/twr/twr_config_timed.h>
#include <openuwb/twr/twr_config_capabilities.h>
#include <openuwb/twr/twr_config_types.h>
#include <openuwb/twr/twr_config_fields.h>
#include <openuwb/twr/twr_config_validator.h>
#include <openuwb/twr/twr_config_effective.h>

#endif /* INCLUDED_GNURADIO_UWB_UWB_TWR_CONFIG_H */
