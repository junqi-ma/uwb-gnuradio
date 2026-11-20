// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// Compatibility aggregate for callers of individual stages and QA.
// Full-packet consumers should include uwb_demod_pipeline.h instead.
// Stages index their input window locally; demodulate_one rebases results.
#include <openuwb/phy/hrp/rx/demod_pipeline.h>
#include <openuwb/phy/hrp/rx/demod_timing.h>
#include <openuwb/phy/hrp/rx/demod_cfo.h>
#include <openuwb/phy/hrp/rx/demod_sfd.h>
#include <openuwb/phy/hrp/rx/demod_softchips.h>
#include <openuwb/phy/hrp/rx/demod_payload.h>
