// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// Compatibility aggregate for callers of individual stages and QA.
// Full-packet consumers should include uwb_demod_pipeline.h instead.
// Stages index their input window locally; demodulate_one rebases results.
#include <gnuradio/uwb/uwb_demod_pipeline.h>
#include <gnuradio/uwb/uwb_demod_timing.h>
#include <gnuradio/uwb/uwb_demod_cfo.h>
#include <gnuradio/uwb/uwb_demod_sfd.h>
#include <gnuradio/uwb/uwb_demod_softchips.h>
#include <gnuradio/uwb/uwb_demod_payload.h>
