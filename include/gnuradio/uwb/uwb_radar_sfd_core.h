/* -*- c++ -*- */
/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Legacy forwarding header. The implementation lives at
 * <openuwb/sensing/radar_sfd_core.h>; see Phase 2 of the openUWB architecture refactor.
 */
#pragma once

// Keep the legacy gr::uwb spelling resolvable (Phase 3 namespace switch).
#include <gnuradio/uwb/compat.h>
// TODO(openuwb-compat-removal): remove legacy forwarding header
#include <openuwb/sensing/radar_sfd_core.h>
