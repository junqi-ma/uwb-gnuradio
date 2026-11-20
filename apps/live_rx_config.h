/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Live receiver configuration: the complete CLI surface of uwb_dw3000_live,
 * plus the runtime resource-root resolution.
 *
 * EXTENSION POINT (file-based configuration): a future loader must fill this
 * same LiveRxConfig and call validate_live_rx_config(cfg) -- never a second,
 * weaker validator -- so every fail-closed check still runs before any source
 * is opened.
 */
#ifndef GR_UWB_LIVE_RX_CONFIG_H
#define GR_UWB_LIVE_RX_CONFIG_H

#include <chrono>
#include <cstddef>
#include <string>

#include <gnuradio/uwb/uwb_defaults.h>
#include <gnuradio/uwb/uwb_demod_grid.h>

namespace openuwb {
namespace live {

using Clock = std::chrono::steady_clock;
constexpr double fs = 737.28e6;
constexpr double work_fs = 998.4e6;
constexpr std::size_t pre = 36864, window = 450000, block = 65536;
constexpr std::size_t history_size = 1 << 21;

struct LiveRxConfig {
    std::string args = "addr=192.168.10.2,recv_frame_size=8000,num_recv_frames=32768";
    std::string channel = "3", antenna = "RX1", subdev, replay;
    // Default is the measured-fastest pairing: the fused CMF requires non-Bypass
    // soft chips, so `full` is its prerequisite.  With `auto` (Bypass first) the
    // fused path is never admitted and demod is 1556 us instead of ~1100 us.
    // A refused integer grid uses the fractional full-stream matched filter.
    std::string cir = "full";
    // Resampler tap set.  The kernel cost is proportional to
    // Hpad = 8*ceil(ceil(T/65)/8), not to T, so only Hpad changes matter:
    //   quality  T=2707 Hpad=48  ripple 0.0050 dB  stopband -70.03 dB  (Kaiser)
    //   eq32     T=2079 Hpad=32  ripple 0.0849 dB  stopband -76.03 dB  (equiripple)
    //   realtime T=1319 Hpad=24  ripple 0.0053 dB  stopband -70.07 dB  (Kaiser)
    //   eq16     T=1039 Hpad=16  ripple 0.0868 dB  stopband -74.56 dB  (equiripple)
    // Default is `eq16`: 3x fewer multiply-adds than `quality` (Hpad 48 -> 16).
    // Measured on the X410 1 s capture (5 ms, W=1): resample 1925.9 -> 958.9 us
    // (-50.2%), demod 1024.5 -> 1011.1 us (unchanged), FCS 203/204 identical,
    // and the demod correlation metrics are within noise
    // (tmetric 0.7323 vs 0.7320, cirmetric 0.6359 vs 0.6365, CFO 4.26 vs
    // 4.25 kHz).  At 1 ms / W=4 it is the difference between working and not:
    // quality drops 182-206 of 990 slots, eq16 drops 0.
    // Trade-off to keep in mind: eq16 has 0.0868 dB ripple (17x the Kaiser
    // baseline) and a 290 MHz passband instead of 330 MHz, and the SIC path has
    // not been validated with it.  `--taps quality` restores the old set.
    std::string taps = "eq16";
    // `packets` prints one line per packet plus the periodic summary;
    // `summary` prints only the periodic line (much less console I/O).
    std::string print = "packets";
    std::string mode = "continuous";   // continuous | timed
    std::string sync = "acquire";      // acquire | locked
    std::string par_only = "off";     // raw correlation PAR experiment
    double freq = 6489.6e6, gain = 60, seconds = 0;
    double period_ms = 5.0;
    size_t workers = 2, capacity = 24, raw_blocks = 128, summary = 50;
    size_t lock_observations = defaults::kLockObservations;
    size_t holdover = defaults::kHoldoverMissCount;
    size_t reacquire = defaults::kReacquireMissCount;
    double lock_residual_limit = 0.0; // 0 = auto: max(512, locked_roi_native)
    size_t locked_roi = 256;
    // LockedBounded SYNC-train tracking: grid (sparse anchors + single-point
    // correlation on the fitted grid) or legacy (per-SYNC +/-8 search).
    std::string locked_timing = "grid";
    // Target identity rule for the lock.  `on` (default) keeps the historical
    // behaviour: an observation only confirms the target when the payload CRC
    // passes, so a preamble+SFD success without a valid FCS is interference.
    // `off` is for low-SNR work where the payload cannot be decoded at all:
    // the lock then discriminates by grid residual instead of dropping every
    // observation.  It is a lock-only switch; decoding is unaffected.
    std::string lock_fcs = "on";
    // Stop the demodulator after the SFD.  `off` (default) keeps the full
    // PHR/payload path.  `on` is the low-SNR capture-only mode: the packet is
    // reported as SFD-only, so the `--cir auto` 15 ms Full retry must not run
    // and an FCS-less SFD success is not an interference event.
    std::string stop_after_sfd = "off";
    // Stop the demodulator right after standalone CIR estimation (after SFD).
    std::string stop_after_cir = "off";
    // Use 64-entry sparse ternary code for timing/SFD/fold matched filtering.
    // TODO(matlab-verify): Live/sensing defaults to "on" (~7x faster on wide
    // ROIs); UWB_demodulation/+uwbdecoder/detectRepeatedPreamble.m uses the
    // dense pulse-shaped template ("off") for low-SNR (-12/-14 dB) sensitivity.
    // Verify low-SNR crossover point in MATLAB before removing the dense path.
    std::string timing_sparse = "on";
    // MF-4: factorize the dense pulse-shaped SYNC template into
    // (r * code) * p for the timing/fold correlation. Recovers the dense
    // pulse-matched sensitivity at sparse-kernel throughput. Opt-in: it is not
    // yet cross-checked against MATLAB detectRepeatedPreamble.m, and the
    // shipped profile default stays on the dense path.
    std::string timing_shaped = "auto";
    size_t post_guard = 8192;
    // Pre-guard used while LOCKED.  The wide 36864 guard exists for the blind
    // autocorrelation seed (Acquisition::scan needs a 36864-sample platform
    // before the true packet start); once the lock predicts the origin the
    // residual is a few samples, so only the bounded ROI plus the resampler
    // transient has to fit in front of the packet.  See --locked-pre-guard.
    size_t locked_pre_guard = 8192;
    bool interference = true;
    bool tone = true;        // fixed-tone cancellation (continuous mode only)
    double tone_period_ms = 20.0;   // model update cadence (diagnostic sweep)
    double tone_training_ms = 0.5;  // training window at each period start
    bool fused = true;       // fused payload + PHR CMF/despread
    bool sfo_from_cfo = true; // derive chip/symbol period from Stage-2 CFO (-cfo/freq)
    // Integer chip-grid admission tolerance in ppm.  Defaults to the physical
    // ceiling (1 sample of drift over the 127 B payload); see
    // Qm35825Profile::fused_gate_ppm for why a tighter gate is counterproductive.
    double fused_gate_ppm = demod::core::kIntegerGridCeilingPpm;
    // On-demand soft-chip prefix.  Tri-state: "auto" follows --fused, because
    // the short prefix only pays off together with the fused CMF.  Measured on
    // the X410 1 s capture with fused=off, the on-demand prefix always failed
    // and fell back to the full stream, costing +7.3% demod for nothing.
    std::string ondemand = "auto";
    bool ondemand_resolved = false; // set from --ondemand/--fused in main
    // SFD-anchored CFO refit policy: legacy | auto | skip.  See
    // Qm35825Profile::cfo_refit_mode.  Default legacy keeps the pre-existing
    // two-pass behaviour bit-identical.
    std::string cfo_refit = "legacy";
    double cfo_refit_skip_rad = 0.10;
    size_t cfo_refit_min_peaks = 64;
    // Streaming autocorrelation acquisition window & thresholds.
    // The window spans 192000 native samples (~256 SYNC symbols = 260.4 us)
    // and is evaluated at full rate: `step` is only the ring-table block
    // granularity (3000 block sums, 96 KB), not a decimation.  The AVX2 block
    // kernel keeps the frontend at ~25% of one core instead of the ~63% the
    // scalar double version needed for the same span.  Thresholds sit ~2.9x
    // (low) and ~5.7x (peak) above the measured pure-noise maxima over 0.5 s,
    // which is what makes -12 dB in-band SNR a 100% capture and -14 dB a
    // recorded boundary.
    size_t acq_span = 192000;
    size_t acq_step = 64;
    size_t acq_persist = 12000;
    double acq_low = 0.00015;
    double acq_peak = 0.00030;
    // Locked-state autocorrelation policy.  "scan" (default) keeps the legacy
    // behaviour: every RX block is scanned.  "suspend" stops the scan while
    // the lock is tracking (Verify/Locked/Holdover) and requires
    // --interference off; "auto" resolves to suspend only when interference
    // monitoring is off.  A lookback of -1 means acq_span + window (one
    // autocorrelation platform plus one packet window); 0 resumes at `end`.
    std::string autocorr_when_locked = "scan";
    long long autocorr_resume_lookback = -1;
    double timed_lead_us = 300.0;
    double timed_t0_ms = 0.0;
    size_t timed_lookahead = 4, timed_alignment = 4;
    std::string res_dir;   // --res-dir: runtime resource root (see resolve_res_dir)
    // Resolved by validate_live_rx_config(): true when the lock is tracking and
    // the autocorrelation scan must be suspended (requires --interference off).
    bool autocorr_suspend = false;
};

struct LiveRxCli {
    LiveRxConfig config;
    bool help = false;
};

/// Parse argv into a config; sets help=true and stops without validating when
/// --help is present (the caller prints the help text and exits 0).
LiveRxCli parse_live_rx_cli(int argc, char** argv);

/// All fail-closed validation, including "timed needs --replay".
void validate_live_rx_config(LiveRxConfig& cfg);

/// Runtime resource root (taps + preamble template); throws when not found.
std::string resolve_res_dir(const LiveRxConfig& cfg);

} // namespace live
} // namespace openuwb

#endif /* GR_UWB_LIVE_RX_CONFIG_H */
