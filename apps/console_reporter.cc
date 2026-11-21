/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "console_reporter.h"

namespace openuwb {
namespace live {

void print_live_help()
{
        std::cout << "Native DW3000 live receiver. Defaults: X410 ch3/RX1, code10/256/4z2/6M8, STS off.\n"
                "--args DEVICE_ARGS --channel 3 --antenna RX1 --subdev SPEC --freq HZ --gain DB\n"
                "--seconds 0 (until Ctrl+C) --workers 2 --queue 24 --raw-blocks 128\n"
                "--summary 50 (report every N packets; N=100 is 0.1 s at a 1 ms period)\n"
                "--print packets|summary (default packets; summary suppresses per-packet lines)\n"
                "--cir full|auto|bypass (default full: the fused CMF needs it)\n"
                "--replay FILE (paced diagnostic, no hardware)\n"
                "--mode continuous|timed (default continuous; timed needs --replay)\n"
                "--sync acquire|locked (default acquire; locked uses a bounded target ROI)\n"
                "--par-only off|sync|sfd|both|guarded|window3|window3-sfd\n"
                "  (experimental raw correlation PAR gates)\n"
                "--period-ms 5 --pre-guard N --post-guard N (native samples)\n"
                "--locked-pre-guard N (pre-guard while LOCKED, default 8192; the\n"
                "  acquire/holdover/reacquire states keep the wide --pre-guard)\n"
                "--locked-roi N (bounded first-SYNC half-width, default 256)\n"
                "--locked-timing grid|legacy (LockedBounded SYNC train: sparse anchors +\n"
                "  single-point correlation on the fitted grid, or the per-SYNC +/-8\n"
                "  search; default grid.  measured_period always comes from the anchor fit)\n"
                "--lock-observations 3 --holdover 3 --reacquire 8\n"
                "--lock-residual-limit N (grid residual outlier limit in native samples;\n"
                "  0 = auto max(512, locked_roi_native))\n"
                "--lock-fcs on|off (default on; off drops the FCS requirement from\n"
                "  target identity, for low-SNR runs where the payload never decodes.\n"
                "  The lock then discriminates by grid residual)\n"
                "--stop-after-sfd on|off (default off; on stops demod after the SFD,\n"
                "  skipping PHR/payload, the --cir auto Full retry and the\n"
                "  FCS-based interference diagnostic)\n"
                "--stop-after-cir on|off (default off; on stops demod after\n"
                "  standalone CIR estimation, skipping CMF/PHR/payload)\n"
                "--timing-sparse on|off (default on; uses 64-entry sparse\n"
                "  ternary code for timing matched filtering; off uses dense template)\n"
                "--timing-shaped on|off (default off; requires --timing-sparse off.\n"
                "  Replaces the 1016-tap dense template with the exact\n"
                "  (r*code)*p factorization: dense metric and lag convention at\n"
                "  ~3-6x lower correlation cost; falls back if the template is\n"
                "  not code*p)\n"
                "--interference on|off (default on)\n"
                "--autocorr-when-locked scan|suspend|auto (default scan;\n"
                "  suspend pauses the autocorrelation scan while the lock is\n"
                "  tracking and requires --interference off;\n"
                "  auto = suspend when --interference off, else scan)\n"
                "--autocorr-resume-lookback N (default -1 = acq_span+window;\n"
                "  0 = resume at current end; N is native samples)\n"
                "--tone on|off (fixed-tone cancellation, default on)\n"
                "--tone-period-ms 20 --tone-training-ms 0.5 (model cadence)\n"
                "--fused on|off (fused payload+PHR CMF, default on)\n"
                "--sfo-from-cfo on|off (derive SFO/period from Stage-2 CFO and --freq, default on)\n"
                "--fused-gate-ppm 6.468 (integer chip-grid admission; ceiling = 1 sample of drift)\n"
                "--ondemand auto|on|off (on-demand soft-chip prefix, default auto = follow --fused)\n"
                "--cfo-refit legacy|auto|skip (SFD-anchored CFO refit + second full-frame\n"
                "  derotation; default legacy = always.  auto keeps the Stage-2 estimate\n"
                "  when the two fits differ by <= --cfo-refit-skip-rad of accumulated\n"
                "  phase and the single-pass range covers the packet; skip forces it)\n"
                "--cfo-refit-skip-rad 0.10 (auto threshold, rad over the rotated span)\n"
                "--cfo-refit-min-peaks 64 (auto: minimum Stage-2 peaks)\n"
                "--acq-span 192000 --acq-step 64 --acq-persist 12000 (autocorrelation window, block stride & plateau span)\n"
                "--acq-low 0.00015 --acq-peak 0.00030 (autocorrelation plateau & peak gates)\n"
                "--timed-lead-us 300 --timed-lookahead 4 --timed-alignment 4\n"
                "--timed-t0-ms 0 (first expected target time; timed_burst has no blind acquisition)\n"
                "--taps eq16|quality|eq32|realtime (resampler Hpad 16|48|32|24, default eq16)\n"
                "--sic off|preamble|full|auto (default off: no SIC allocation/work)\n"
                "  preamble = subtract the known interferer SYNC+SFD; full = subtract\n"
                "  the decoded full interferer packet (strict); auto = full, falling\n"
                "  back to preamble when the interferer payload cannot be decoded\n"
                "--sic-backend ipatov|discrete|path (default ipatov)\n"
                "--sic-code 9..12 (interferer HRP code, default 11)\n"
                "--sic-reps N (interferer SYNC repetitions, default 128)\n"
                "--sic-sfd dw-8|4z1|4z2|4z3|4z4|ieee (default dw-8)\n"
                "--res-dir DIR (runtime resources: resampler taps + preamble template;\n"
                "  default: $UWB_RESOURCE_DIR or $UWB_TESTDATA_DIR or the installed\n"
                "  resource dir, then the build-tree testdata fallback)\n";

}

} // namespace live
} // namespace openuwb
