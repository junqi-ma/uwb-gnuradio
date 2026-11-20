/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * One live-receive session: source -> acquisition -> bounded job pool ->
 * demod -> console reporter.  Returns the process exit status.
 */
#ifndef GR_UWB_LIVE_RX_SESSION_H
#define GR_UWB_LIVE_RX_SESSION_H

#include "live_rx_config.h"

namespace openuwb {
namespace live {

int run_live_rx(const LiveRxConfig& cfg);

} // namespace live
} // namespace openuwb

#endif /* GR_UWB_LIVE_RX_SESSION_H */
