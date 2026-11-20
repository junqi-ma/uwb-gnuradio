/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * CLI parsing and fail-closed validation for uwb_dw3000_live.
 */
#include "live_rx_config.h"

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <vector>

#include <gnuradio/uwb/uwb_autocorr_gate.h>

namespace gr {
namespace uwb {
namespace live {

// Runtime resource root for the resampler taps + preamble template.
// Resolution order (first existing directory wins):
//   1. --res-dir <dir>
//   2. $UWB_RESOURCE_DIR
//   3. $UWB_TESTDATA_DIR
//   4. the installed resource dir (UWB_INSTALL_RESOURCE_DIR, compiled in)
//   5. the build-tree testdata fallback (UWB_TESTDATA_DIR, compiled in)
// An installed binary therefore never depends on a source checkout, an old
// build tree, RPATH or PYTHONPATH; the source-tree path is only a build-tree
// convenience and never refers to another worktree.
std::string resolve_res_dir(const LiveRxConfig& o)
{
    namespace fs = std::filesystem;
    const char* env_res = std::getenv("UWB_RESOURCE_DIR");
    const char* env_td  = std::getenv("UWB_TESTDATA_DIR");
    std::vector<std::string> candidates;
    if (!o.res_dir.empty()) candidates.push_back(o.res_dir);
    if (env_res && *env_res) candidates.push_back(env_res);
    if (env_td && *env_td) candidates.push_back(env_td);
#ifdef UWB_INSTALL_RESOURCE_DIR
    candidates.push_back(UWB_INSTALL_RESOURCE_DIR);
#endif
#ifdef UWB_TESTDATA_DIR
    candidates.push_back(UWB_TESTDATA_DIR);
#endif
    for (const auto& c : candidates) {
        std::error_code ec;
        if (fs::is_directory(c, ec))
            return c;
    }
    throw std::runtime_error(
        "resource directory not found; pass --res-dir <dir> or set "
        "UWB_RESOURCE_DIR (looked in: " +
        [&]{ std::string all; for (const auto& c : candidates) { all += c; all += "; "; } return all; }() +
        ")");
}


LiveRxCli parse_live_rx_cli(int argc, char** argv)
{
    LiveRxCli cli;
    LiveRxConfig& o = cli.config;
    for (int i=1; i<argc; ++i) {
        const std::string key=argv[i];
        auto value=[&] { if (++i>=argc) throw std::runtime_error("missing value for "+key); return std::string(argv[i]); };
        if (key=="--help") { cli.help = true; return cli; }
        else if (key=="--args") o.args=value();
        else if (key=="--channel") o.channel=value();
        else if (key=="--antenna") o.antenna=value();
        else if (key=="--subdev") o.subdev=value();
        else if (key=="--freq") o.freq=std::stod(value());
        else if (key=="--gain") o.gain=std::stod(value());
        else if (key=="--seconds") o.seconds=std::stod(value());
        else if (key=="--workers") o.workers=std::stoul(value());
        else if (key=="--queue") o.capacity=std::stoul(value());
        else if (key=="--raw-blocks") o.raw_blocks=std::stoul(value());
        else if (key=="--summary") o.summary=std::stoul(value());
        else if (key=="--cir") o.cir=value();
        else if (key=="--taps") o.taps=value();
        else if (key=="--print") o.print=value();
        else if (key=="--replay") o.replay=value();
        else if (key=="--mode") o.mode=value();
        else if (key=="--sync") o.sync=value();
        else if (key=="--par-only") o.par_only=value();
        else if (key=="--period-ms") o.period_ms=std::stod(value());
        else if (key=="--post-guard") o.post_guard=std::stoul(value());
        else if (key=="--locked-pre-guard") o.locked_pre_guard=std::stoul(value());
        else if (key=="--locked-roi") o.locked_roi=std::stoul(value());
        else if (key=="--locked-timing") o.locked_timing=value();
        else if (key=="--lock-fcs") o.lock_fcs=value();
        else if (key=="--stop-after-sfd") o.stop_after_sfd=value();
        else if (key=="--stop-after-cir") o.stop_after_cir=value();
        else if (key=="--timing-sparse") o.timing_sparse=value();
        else if (key=="--timing-shaped") o.timing_shaped=value();
        else if (key=="--lock-observations") o.lock_observations=std::stoul(value());
        else if (key=="--holdover") o.holdover=std::stoul(value());
        else if (key=="--reacquire") o.reacquire=std::stoul(value());
        else if (key=="--lock-residual-limit") o.lock_residual_limit=std::stod(value());
        else if (key=="--interference") o.interference=(value()!="off");
        else if (key=="--tone") o.tone=(value()!="off");
        else if (key=="--tone-period-ms") o.tone_period_ms=std::stod(value());
        else if (key=="--tone-training-ms") o.tone_training_ms=std::stod(value());
        else if (key=="--fused") o.fused=(value()!="off");
        else if (key=="--sfo-from-cfo") o.sfo_from_cfo=(value()!="off");
        else if (key=="--fused-gate-ppm") o.fused_gate_ppm=std::stod(value());
        else if (key=="--ondemand") o.ondemand=value();
        else if (key=="--cfo-refit") o.cfo_refit=value();
        else if (key=="--cfo-refit-skip-rad") o.cfo_refit_skip_rad=std::stod(value());
        else if (key=="--cfo-refit-min-peaks") o.cfo_refit_min_peaks=std::stoul(value());
        else if (key=="--acq-span") o.acq_span=std::stoul(value());
        else if (key=="--acq-step") o.acq_step=std::stoul(value());
        else if (key=="--acq-persist") o.acq_persist=std::stoul(value());
        else if (key=="--acq-low") o.acq_low=std::stod(value());
        else if (key=="--acq-peak") o.acq_peak=std::stod(value());
        else if (key=="--autocorr-when-locked") o.autocorr_when_locked=value();
        else if (key=="--autocorr-resume-lookback") o.autocorr_resume_lookback=std::stoll(value());
        else if (key=="--timed-lead-us") o.timed_lead_us=std::stod(value());
        else if (key=="--timed-lookahead") o.timed_lookahead=std::stoul(value());
        else if (key=="--timed-alignment") o.timed_alignment=std::stoul(value());
        else if (key=="--timed-t0-ms") o.timed_t0_ms=std::stod(value());
        else if (key=="--res-dir") o.res_dir=value();
        else throw std::runtime_error("unknown option "+key);
    }
    return cli;
}

void validate_live_rx_config(LiveRxConfig& o)
{
    if (!o.workers || o.workers>32 || !o.capacity || o.capacity>1024 ||
        o.raw_blocks<6 || o.raw_blocks>4096 || !o.summary || o.seconds<0 ||
        !std::isfinite(o.seconds) || (o.cir!="full" && o.cir!="bypass" && o.cir!="auto"))
        throw std::runtime_error("invalid workers/queue/summary/duration/CIR configuration");
    if (o.mode!="continuous" && o.mode!="timed")
        throw std::runtime_error("--mode must be continuous or timed");
    if (o.print!="packets" && o.print!="summary")
        throw std::runtime_error("--print must be packets or summary");
    if (o.taps!="quality" && o.taps!="eq32" && o.taps!="realtime" &&
        o.taps!="eq16")
        throw std::runtime_error("--taps must be quality, eq32, realtime or eq16");
    if (o.sync!="acquire" && o.sync!="locked")
        throw std::runtime_error("--sync must be acquire or locked");
    if (o.par_only!="off" && o.par_only!="sync" && o.par_only!="sfd" &&
        o.par_only!="both" && o.par_only!="guarded" && o.par_only!="window3" &&
        o.par_only!="window3-sfd")
        throw std::runtime_error(
            "--par-only must be off, sync, sfd, both, guarded, window3, or window3-sfd");
    if (o.locked_timing!="grid" && o.locked_timing!="legacy")
        throw std::runtime_error("--locked-timing must be grid or legacy");
    // Strict on|off: unlike --tone/--fused (which treat anything but "off" as
    // on) these two change what the lock and the demodulator consider a
    // success, so a typo must fail closed instead of silently picking a mode.
    if (o.lock_fcs!="on" && o.lock_fcs!="off")
        throw std::runtime_error("--lock-fcs must be on or off");
    if (o.stop_after_sfd!="on" && o.stop_after_sfd!="off")
        throw std::runtime_error("--stop-after-sfd must be on or off");
    if (o.stop_after_cir!="on" && o.stop_after_cir!="off")
        throw std::runtime_error("--stop-after-cir must be on or off");
    if (o.timing_sparse!="on" && o.timing_sparse!="off")
        throw std::runtime_error("--timing-sparse must be on or off");
    if (o.timing_shaped!="auto" && o.timing_shaped!="on" && o.timing_shaped!="off")
        throw std::runtime_error("--timing-shaped must be auto, on or off");
    // The shaped path reproduces the DENSE metric scale and lag convention; it
    // is not a drop-in for the sparse peak-sample convention, so combining the
    // two would silently mix conventions. Make the intended mode explicit.
    if (o.timing_shaped=="on" && o.timing_sparse=="on")
        throw std::runtime_error("--timing-shaped on requires --timing-sparse off");
    if (!(o.tone_period_ms > 0.0) || !(o.tone_training_ms > 0.0) ||
        o.tone_training_ms >= o.tone_period_ms)
        throw std::runtime_error("invalid --tone-period-ms/--tone-training-ms");
    if (o.ondemand!="auto" && o.ondemand!="on" && o.ondemand!="off")
        throw std::runtime_error("--ondemand must be auto, on or off");
    if (o.cfo_refit!="legacy" && o.cfo_refit!="auto" && o.cfo_refit!="skip")
        throw std::runtime_error("--cfo-refit must be legacy, auto or skip");
    if (!(o.cfo_refit_skip_rad > 0.0) || !std::isfinite(o.cfo_refit_skip_rad))
        throw std::runtime_error("--cfo-refit-skip-rad must be finite and > 0");
    if (o.acq_step == 0 || o.acq_span < o.acq_step || (o.acq_span % o.acq_step) != 0)
        throw std::runtime_error("--acq-span must be a positive multiple of --acq-step");
    if (o.acq_persist == 0)
        throw std::runtime_error("--acq-persist must be > 0");
    if (!(o.acq_low > 0.0) || o.acq_low >= 1.0 || !std::isfinite(o.acq_low) ||
        !(o.acq_peak >= o.acq_low) || o.acq_peak >= 1.0 || !std::isfinite(o.acq_peak))
        throw std::runtime_error("--acq-low/--acq-peak must satisfy 0 < low <= peak < 1");
    // Locked-state scan policy.  Resolve it here, BEFORE the device/replay
    // source is opened: `suspend` with --interference on must fail closed at
    // startup instead of silently disabling interference monitoring.
    if (o.autocorr_resume_lookback < -1)
        throw std::runtime_error("--autocorr-resume-lookback must be >= -1 "
                                 "(-1 = acq_span + window)");
    o.autocorr_suspend = live::resolve_autocorr_suspend(
        live::parse_autocorr_when_locked(o.autocorr_when_locked),
        o.interference);
    // The gate is a drift budget: above the physical ceiling (1 sample over the
    // 127 B payload = 6.5 ppm) the integer grid cannot stay on the correlation
    // peak, so accepting it would produce silently wrong soft chips.
    if (!(o.fused_gate_ppm > 0.0) || !std::isfinite(o.fused_gate_ppm) ||
        o.fused_gate_ppm > demod::core::kIntegerGridCeilingPpm)
        throw std::runtime_error("--fused-gate-ppm must be in (0, 6.468] "
                                 "(1 sample of drift over the 127 B payload)");
    o.ondemand_resolved =
        (o.ondemand == "auto") ? o.fused : (o.ondemand == "on");
    if (!(o.period_ms>0.0) || !std::isfinite(o.period_ms))
        throw std::runtime_error("invalid --period-ms");
    if (o.locked_roi==0 || o.lock_observations<2 || o.holdover==0 ||
        o.reacquire<=o.holdover || o.lock_residual_limit<0.0 ||
        !std::isfinite(o.lock_residual_limit) || o.timed_lookahead==0 ||
        o.timed_alignment==0 || o.timed_t0_ms<0.0 ||
        !std::isfinite(o.timed_t0_ms))
        throw std::runtime_error("invalid lock/timed configuration");
    if (o.locked_pre_guard == 0)
        throw std::runtime_error("--locked-pre-guard must be > 0");
    // The timed UHD command path is NOT enabled: it must be validated on a
    // board before it can be claimed.  Failing closed here is deliberate.
    if (o.mode=="timed" && o.replay.empty())
        throw std::runtime_error("--mode timed requires --replay: the device-timed "
                                 "UHD command path is not validated on hardware yet");
}

} // namespace live
} // namespace uwb
} // namespace gr
