/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Header-only monostatic-radar integrator at 998.4 MS/s.
 *
 * Workstream C of
 * docs/performance/OpenCode_Sensing与Comm统一前端和CIR_开发方案_2026-11-12.md:
 * sensing no longer owns a second preamble receiver or a second CIR kernel.
 * The two production paths are
 *
 *   RX window + predicted SFD
 *     → if use_predicted_timing: skip detection (timed echo)
 *     → else demod::acquire_preamble(PreamblePurpose::CirOnly)   [shared with comm]
 *     → sync_cir::estimate_sync_cir at the TX-time axis           [shared with comm]
 *     → CIR success ? Ok (raw+norm taps) : CirFailed
 *
 * ---------------------------------------------------------------------------
 * Target CFO (the defect this file closes)
 * ---------------------------------------------------------------------------
 * The legacy sensing path coherently averaged raw SYNC windows and never
 * estimated or compensated the TARGET carrier offset; interference removal by
 * the SIC does not remove it either.  The shared front-end now estimates the
 * final CFO from the target code and the shared CIR kernel consumes IQ that is
 * already derotated with it.
 *
 *   use_predicted_timing == false
 *       acquire_preamble() always estimates + compensates; `final_cfo_hz`,
 *       `cfo_epoch` and `phase_reference_sample` describe exactly what the CIR
 *       kernel consumed.  With `compensate_target_cfo == false` the detection
 *       and the CFO estimate still run (that is the control arm), but the CIR
 *       kernel consumes the RAW window and `cfo_epoch == 0` records that the
 *       estimate was NOT applied.
 *   use_predicted_timing == true, target_cfo_known == true
 *       the caller already knows the offset; the CIR union is derotated from
 *       the raw input with `target_cfo_hz` and `cfo_epoch` is published as 2
 *       ("this buffer is final-CFO compensated").
 *   use_predicted_timing == true, target_cfo_known == false
 *       the timed path has no detection of its own, so it runs the SAME shared
 *       front-end (seeded with the TX-time axis, `period_hint =
 *       samples_per_symbol`) purely to estimate the target CFO and to build
 *       the compensated buffer.  The published detection coordinates stay the
 *       predicted ones (the timed contract is unchanged); `cfo_source ==
 *       "estimated_timed"` and `cfo_epoch == 2` record what really happened.
 *       If the front-end cannot acquire the echo (weak echo, destroyed SFD,
 *       SFD outside the public coarse ROI, coverage too small) the call FAILS
 *       CLOSED with `status_reason` naming the reason.  It never returns Ok
 *       with an uncompensated CIR.
 *   use_predicted_timing == true, compensate_target_cfo == false
 *       the explicit uncompensated control arm: no front-end pass, the CIR
 *       kernel consumes the RAW window, `cfo_epoch == 0` and `cfo_source ==
 *       "none"`.  This is the only way to get a timed CIR without CFO
 *       compensation, and it is opt-in.
 *
 * ---------------------------------------------------------------------------
 * Three distinct origins (never conflated)
 * ---------------------------------------------------------------------------
 *   detected_sync_origin    RX front-end SYNC origin (detection / CFO anchor).
 *                           Equals `preamble_start_sample`.
 *   cir_axis_origin         tap-0 delay axis.  Sensing uses the PUBLIC TX-time
 *                           prediction `nominal_preamble_start(predicted_sfd,
 *                           sync_reps, sps)`, so a physical delay D appears as
 *                           a tap shift.  It is NEVER re-centred on the detected
 *                           peak (that would erase the propagation delay).
 *                           `cir_origin_sample` is a legacy alias of this field.
 *   phase_reference_sample  absolute sample whose phase is the derotation
 *                           anchor (0 today: dst[i] = src[i]*exp(-j*w*i)).
 *                           It is WINDOW-relative: the absolute phase anchor is
 *                           `window_absolute_origin + phase_reference_sample`,
 *                           which the block publishes as
 *                           `phase_reference_absolute_sample`.  A single
 *                           window's sample 0 is NOT global sample 0, so raw
 *                           tap phase is only comparable across windows after
 *                           accounting for the window origin.
 *
 * ---------------------------------------------------------------------------
 * Per-consumer final-CFO coverage (R03)
 * ---------------------------------------------------------------------------
 * The front-end plans the final rotation over the union of every derotated
 * reader, not just the main CIR: `extra_cir_skip_initial` /
 * `extra_cir_repetitions` describe a second consumer (the block's alongside
 * average, whose skip/count differ from the main CIR's).  The union is frozen
 * into the prepared profile and validated on every call.  A consumer whose
 * windows fall outside `[covered_lo, covered_hi)` must fail closed
 * (`radar_cir_coverage_ok()`); holding a pointer into `demod.derotated` is not
 * proof of coverage.
 *
 * Failure: tap_count=0, raw/norm taps zeroed. Do not return seed-only CIR.
 * Failed-stage sample indices stay -1. Do not copy predicted SFD/origin into
 * success fields.
 *
 * sfd_search_margin=64 is a software default, not a hardware freeze.
 * peak_tap is argmax(|raw|) on the CIR grid, not physical zero range.
 *
 * HOT PATH (radar_cir_one): no heap allocation / no vector growth.
 */

#pragma once

#include <gnuradio/uwb/uwb_demod_core.h> // shared preamble front-end + DemodScratch
#include <gnuradio/uwb/uwb_phy_profile.h>
#include <gnuradio/uwb/uwb_radar_cir_estimator.h>
#include <gnuradio/uwb/uwb_radar_sfd_core.h>
#include <gnuradio/uwb/uwb_radar_timing_core.h>
#include <gnuradio/uwb/uwb_sync_cir_core.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstdint>
#include <cstring>
#include <limits>
#include <vector>

namespace gr {
namespace uwb {
namespace radar {

enum class RadarCirStatus : uint8_t {
    Ok = 0,
    SfdFailed = 1,
    TimingFailed = 2,
    CirFailed = 3,
    InvalidInput = 4
};

struct RadarCirConfig {
    size_t code_index = 9;
    size_t sync_repetitions = 64;
    size_t samples_per_symbol = 1016;
    const char* sfd_mode = "4z2";
    uint32_t tx_profile_id = 0;     // 0 = unspecified; nonzero must match prepare
    int64_t sfd_search_margin = 64; // software default; NOT a hardware freeze
    int64_t sync_refine_margin = 8;
    float sfd_threshold = 0.3f;
    float sync_refine_threshold = 0.3f;
    size_t cir_pre = 16;
    size_t cir_post = 100;
    size_t cir_skip_initial = 10;
    size_t cir_repetitions = 54; // used as min(this, reps-skip)
    // Timed monostatic TX/RX: SFD is a constant offset from the scheduled
    // echo.  When true, do not search SFD or refine SYNC; CIR uses the
    // predicted origin.  Default false keeps the communication-style gate.
    bool use_predicted_timing = false;

    // ---- target CFO (plan §2 / contract §3) ------------------------------
    // Estimate and compensate the TARGET carrier offset before the coherent
    // SYNC average.  Default true: the legacy path silently averaged a
    // CFO-rotated preamble, which is the defect workstream C closes.
    bool compensate_target_cfo = true;
    // The caller already knows the target CFO (e.g. a calibrated link).  Only
    // consulted when `use_predicted_timing` is true; the detected path always
    // estimates it from the target code.
    bool target_cfo_known = false;
    // Used only when `target_cfo_known` is true (Hz, at 998.4 MS/s).
    double target_cfo_hz = 0.0;

    // ---- extra derotated consumer (R03 item 4) ---------------------------
    // A second reader of the same final-CFO buffer whose repetition window
    // differs from the main CIR's (the estimator block's alongside average:
    // skip 0, count = sync_repetitions, versus the main CIR's skip 10 /
    // count 54).  The final rotation is planned over the UNION of the main
    // CIR windows and this range, so the extra consumer can never read
    // samples that were not compensated with the final CFO.  `0` repetitions
    // means "no extra consumer".  Both fields are part of the prepared
    // identity: changing them after prepare() is rejected.
    size_t extra_cir_skip_initial = 0;
    size_t extra_cir_repetitions = 0;
};

// Immutable identity captured at prepare(). radar_cir_one rejects any
// runtime config that disagrees with this profile.
struct PreparedRadarProfile {
    bool ready = false;
    size_t code_index = 0;
    char sfd_mode[16] = {};
    size_t n_sfd_symbols = 0;
    size_t sync_repetitions = 0;
    size_t samples_per_symbol = 0;
    uint32_t tx_profile_id = 0;
    size_t max_pre = 0;
    size_t max_post = 0;
    // Union of every derotated consumer's repetition window, frozen at
    // prepare().  The front-end plans the final rotation over exactly
    // [coverage_skip_initial, coverage_skip_initial + coverage_repetitions).
    size_t coverage_skip_initial = 0;
    size_t coverage_repetitions = 0;
};

struct RadarCirResult {
    RadarCirStatus status = RadarCirStatus::InvalidInput;
    int64_t sfd_start_sample = -1;         // RX-detected SFD; -1 if SFD failed
    int64_t preamble_start_sample = -1;    // RX-refined SYNC origin; -1 if timing failed
    int64_t cir_origin_sample = -1;        // alias of cir_axis_origin (legacy name)
    int64_t predicted_sfd_start = -1;
    float sfd_metric = 0.f;
    float sync_metric = 0.f;
    size_t tap_count = 0;
    size_t peak_tap = 0;
    float peak_abs = 0.f;
    float raw_l2_norm = 0.f;
    size_t valid_repetitions = 0;

    // ---- three origins + CFO provenance (contract §3) --------------------
    // RX-detected SYNC origin (== preamble_start_sample).  Detection and CFO
    // anchor only; never the CIR delay axis.
    int64_t detected_sync_origin = -1;
    // Tap-0 delay axis: the public TX-time prediction.  Keeps the physical
    // delay in the taps.  `cir_origin_sample` carries the same value.
    int64_t cir_axis_origin = -1;
    // Absolute sample the final derotation is anchored at (0 = buffer origin).
    int64_t phase_reference_sample = 0;
    // Final CFO the CIR kernel consumed.  0 with cfo_epoch == 0 means the
    // explicit uncompensated control arm of the timed path.
    double final_cfo_hz = 0.0;
    // Bootstrap (epoch 1) CFO estimate, published for provenance only: it is
    // NEVER consumed (the final rotation is always recomputed from the raw
    // input with `final_cfo_hz`).  A consumer can audit that the CIR used the
    // final epoch, and how far the bootstrap estimate was from it.
    double bootstrap_cfo_hz = 0.0;
    // 0 = no compensation applied, 2 = the CIR input is final-CFO compensated
    // (the demod front-end uses 1 for its bootstrap epoch; that epoch is never
    // published here because it is never consumed).
    uint32_t cfo_epoch = 0;
    // Unified repetition period P actually used by the CIR kernel (measured
    // period when the front-end ran, configured sps otherwise).
    double cir_samples_per_symbol = 0.0;

    // Sub-timings of the shared front-end (diagnostics only; they are children
    // of the caller's sensing stage and must never be added to a parent timer).
    uint64_t frontend_timing_us = 0;
    uint64_t frontend_cfo_us = 0;
    uint64_t frontend_sfd_us = 0;
    uint64_t coverage_us = 0;
    uint64_t cir_us = 0;

    // ---- final-CFO coverage actually advertised (R03) --------------------
    // [covered_lo, covered_hi) is the interval of `scratch.demod.derotated`
    // that holds FINAL-CFO compensated samples.  -1/-1 means "no derotated
    // buffer is advertised" (raw arm or a failed call).  Every derotated
    // consumer must validate its own window range against this interval.
    int64_t covered_lo = -1;
    int64_t covered_hi = -1;

    // Provenance of the CFO the kernel consumed:
    //   "estimated_detected"       detected path, front-end estimate applied
    //   "estimated_not_applied"    detected path, control arm (raw consumed)
    //   "estimated_timed"          timed path, front-end estimate applied
    //   "caller_known"             timed path, caller-supplied value applied
    //   "none"                     no compensation (raw consumed)
    const char* cfo_source = "none";

    // Static string literal naming why the call failed ("" on success).  It is
    // never allocated and never contains data-derived text.
    const char* status_reason = "";
};

struct RadarCirCoreScratch {
    RadarSfdScratch sfd; // legacy SFD template; kept for identity/prepare checks
    // Legacy scalar scratch.  Still used by estimate_radar_cir_repetition()
    // (per-repetition emission) and by the legacy reference math in QA.
    RadarCirScratch cir;
    std::vector<std::complex<float>> sync_template; // L2-normalized, length sps
    PreparedRadarProfile prepared;

    // ---- shared preamble front-end + shared CIR kernel -------------------
    // Raw pulse-shaped SYNC symbol (not normalized): the demod front-end
    // normalizes its own copy, exactly as it does for the comm path.
    std::vector<std::complex<float>> sync_pulse;
    // Qm35825Profile built from RadarCirConfig at prepare time.  Its
    // `sfd_mode` points at `prepared.sfd_mode`, so the scratch must not be
    // copied/moved after prepare (detail::rebind_demod_profile() re-binds it
    // defensively on every call).
    demod::Qm35825Profile demod_profile;
    demod::core::DemodScratch demod;      // front-end scratch (reserve() is the caller's job)
    demod::sync_cir::PreparedSyncCode code;
    demod::sync_cir::SyncCirScratch cir_scratch;
};

namespace detail {

inline bool radar_sfd_mode_known(const char* sfd_mode)
{
    if (!sfd_mode || sfd_mode[0] == '\0')
        return false;
    return std::strcmp(sfd_mode, "4z2") == 0 ||
           std::strcmp(sfd_mode, "4z1") == 0 ||
           std::strcmp(sfd_mode, "4z3") == 0 ||
           std::strcmp(sfd_mode, "4z4") == 0 ||
           std::strcmp(sfd_mode, "decawave") == 0 ||
           std::strcmp(sfd_mode, "ieee") == 0;
}

inline void reset_radar_cir_core_scratch(RadarCirCoreScratch& scratch)
{
    scratch.sfd = RadarSfdScratch{};
    scratch.cir = RadarCirScratch{};
    scratch.sync_template.clear();
    scratch.sync_pulse.clear();
    scratch.code = demod::sync_cir::PreparedSyncCode{};
    scratch.cir_scratch.avg.clear();
    scratch.cir_scratch.raw.clear();
    scratch.cir_scratch.norm.clear();
    // `demod` is pure per-worker cache; dropping it would only cost a
    // re-allocation on the next packet, so leave it alone.
    scratch.demod_profile = demod::Qm35825Profile{};
    scratch.prepared = PreparedRadarProfile{};
}

// Re-bind the profile's borrowed `sfd_mode` pointer to the scratch-owned copy.
inline void rebind_demod_profile(RadarCirCoreScratch& scratch)
{
    scratch.demod_profile.sfd_mode = scratch.prepared.sfd_mode;
}

inline bool finite_positive(float x)
{
    return std::isfinite(x) && (x > 0.0f);
}

inline bool copy_sfd_mode(const char* src, char* dst, size_t dst_len)
{
    if (!src || !dst || dst_len == 0)
        return false;
    size_t n = 0;
    while (src[n] != '\0') {
        if (n + 1 >= dst_len)
            return false;
        ++n;
    }
    for (size_t i = 0; i < n; ++i)
        dst[i] = src[i];
    dst[n] = '\0';
    return true;
}

// Union of every derotated consumer's repetition window, expressed as
// (skip_initial, repetitions) on the SYNC grid.  The main CIR is
// (cfg.cir_skip_initial, cfg.cir_repetitions) and the optional extra consumer
// is (cfg.extra_cir_skip_initial, cfg.extra_cir_repetitions); the union is the
// envelope of both, clamped to the preamble.  `repetitions == 0` means the
// union is empty (an invalid CIR request, rejected later).
inline void coverage_union(const RadarCirConfig& cfg, size_t& skip, size_t& count)
{
    const size_t total = cfg.sync_repetitions;
    skip = 0;
    count = 0;
    if (total == 0)
        return;

    size_t main_skip = cfg.cir_skip_initial;
    if (main_skip >= total)
        main_skip = total; // empty main window
    size_t main_count = 0;
    if (main_skip < total) {
        main_count = (cfg.cir_repetitions > 0) ? cfg.cir_repetitions
                                               : total - main_skip;
        if (main_count > total - main_skip)
            main_count = total - main_skip;
    }

    size_t extra_skip = total;
    size_t extra_count = 0;
    if (cfg.extra_cir_repetitions > 0 && cfg.extra_cir_skip_initial < total) {
        extra_skip = cfg.extra_cir_skip_initial;
        extra_count = cfg.extra_cir_repetitions;
        if (extra_count > total - extra_skip)
            extra_count = total - extra_skip;
    }

    if (main_count == 0 && extra_count == 0) {
        skip = (main_skip < total) ? main_skip : 0;
        count = 0;
        return;
    }
    if (main_count == 0) {
        skip = extra_skip;
        count = extra_count;
        return;
    }
    if (extra_count == 0) {
        skip = main_skip;
        count = main_count;
        return;
    }
    const size_t lo = std::min(main_skip, extra_skip);
    const size_t hi = std::max(main_skip + main_count, extra_skip + extra_count);
    skip = lo;
    count = hi - lo;
}

inline bool profile_matches(const PreparedRadarProfile& p,
                            const RadarCirConfig& cfg)
{
    if (!p.ready || !cfg.sfd_mode)
        return false;
    if (p.code_index != cfg.code_index)
        return false;
    if (p.sync_repetitions != cfg.sync_repetitions)
        return false;
    if (p.samples_per_symbol != cfg.samples_per_symbol)
        return false;
    if (std::strcmp(p.sfd_mode, cfg.sfd_mode) != 0)
        return false;
    if (p.tx_profile_id != cfg.tx_profile_id)
        return false;
    if (cfg.cir_pre > p.max_pre || cfg.cir_post > p.max_post)
        return false;
    // The final-CFO coverage was planned for the prepared union only; a
    // runtime config that widened (or moved) any consumer's window would read
    // samples outside it, so it is rejected instead of silently truncated.
    // An EMPTY union means the request itself is invalid (skip >= repetitions
    // or zero repetitions); that is reported by the dedicated skip/count check
    // with its own status, so it is not treated as an identity mismatch here.
    size_t skip = 0;
    size_t count = 0;
    coverage_union(cfg, skip, count);
    if (count > 0 &&
        (skip != p.coverage_skip_initial || count != p.coverage_repetitions))
        return false;
    return true;
}

inline void zero_cir_taps(RadarCirScratch& cir)
{
    if (!cir.raw_taps.empty())
        std::fill(cir.raw_taps.begin(), cir.raw_taps.end(),
                  std::complex<float>(0.f, 0.f));
    if (!cir.norm_taps.empty())
        std::fill(cir.norm_taps.begin(), cir.norm_taps.end(),
                  std::complex<float>(0.f, 0.f));
}

// True when radar_cir_one reads `scratch.demod.derotated` (final-CFO
// compensated) instead of the caller's raw window.
//
// Every compensated arm consumes the derotated buffer: the detected path
// always, and the timed path both with a caller-supplied CFO and with the
// front-end estimate.  Only the explicit uncompensated control arm
// (`compensate_target_cfo == false`) reads raw.
inline bool radar_cir_uses_derotated(const RadarCirConfig& cfg)
{
    return cfg.compensate_target_cfo;
}

// True when every window the kernel would read for (skip_initial,
// max_repetitions) fits inside the advertised final-CFO coverage
// [covered_lo, covered_hi).  `need_lo`/`need_hi` receive the unclipped
// envelope of the accepted windows so a caller can report the shortfall.
// A pointer into `scratch.demod.derotated` proves nothing on its own: samples
// outside the coverage were written by an earlier epoch (bootstrap CFO) or are
// stale, so this check is the only proof of compensation.
inline bool coverage_contains(const demod::sync_cir::SyncCirRequest& req,
                              size_t n,
                              int64_t covered_lo,
                              int64_t covered_hi,
                              int64_t& need_lo,
                              int64_t& need_hi)
{
    need_lo = -1;
    need_hi = -1;
    if (covered_lo < 0 || covered_hi <= covered_lo)
        return false;
    if (!demod::sync_cir::required_input_range(
            req, demod::sync_cir::kSyncCirCodeLength, n, need_lo, need_hi))
        return false;
    return need_lo >= covered_lo && need_hi <= covered_hi;
}

// Convenience wrapper for a caller that only knows its own (skip, count) --
// the estimator block's alongside average uses this for its own window range.
inline bool coverage_ok_for_range(const RadarCirConfig& cfg,
                                  int64_t cir_axis_origin,
                                  double samples_per_symbol,
                                  size_t skip_initial,
                                  size_t max_repetitions,
                                  size_t total_repetitions,
                                  size_t n,
                                  int64_t covered_lo,
                                  int64_t covered_hi,
                                  int64_t& need_lo,
                                  int64_t& need_hi)
{
    demod::sync_cir::SyncCirRequest req;
    req.cir_axis_origin = cir_axis_origin;
    req.samples_per_symbol = samples_per_symbol;
    req.total_repetitions = total_repetitions;
    req.skip_initial = skip_initial;
    req.max_repetitions = max_repetitions;
    req.pre = cfg.cir_pre;
    req.post = cfg.cir_post;
    return coverage_contains(req, n, covered_lo, covered_hi, need_lo, need_hi);
}

// Map the shared front-end status onto the radar status vocabulary.  There is
// no separate radar CFO status, so a CFO/coverage failure is reported as
// CirFailed (no CIR could be produced) rather than as a detection failure.
inline RadarCirStatus map_demod_status(demod::DemodStatus s)
{
    switch (s) {
    case demod::DemodStatus::Success:
        return RadarCirStatus::Ok;
    case demod::DemodStatus::InvalidInput:
        return RadarCirStatus::InvalidInput;
    case demod::DemodStatus::TimingFailed:
        return RadarCirStatus::TimingFailed;
    case demod::DemodStatus::SfdFailed:
        return RadarCirStatus::SfdFailed;
    case demod::DemodStatus::CfoFailed:
    case demod::DemodStatus::CirFailed:
    case demod::DemodStatus::PhrFailed:
    case demod::DemodStatus::PayloadFailed:
    case demod::DemodStatus::FcsFailed:
    case demod::DemodStatus::QueueFull:
    case demod::DemodStatus::InternalError:
        return RadarCirStatus::CirFailed;
    }
    return RadarCirStatus::CirFailed;
}

} // namespace detail

// Public: true when every window the kernel would read for
// (skip_initial, max_repetitions) fits inside the final-CFO coverage
// [covered_lo, covered_hi) advertised by radar_cir_one().  Every derotated
// consumer that is not the main CIR (the estimator block's alongside average)
// must call this before reading `scratch.demod.derotated`; a pointer into that
// buffer is not proof that the samples were compensated with the final CFO.
inline bool radar_cir_coverage_ok(const RadarCirConfig& cfg,
                                  int64_t cir_axis_origin,
                                  double samples_per_symbol,
                                  size_t skip_initial,
                                  size_t max_repetitions,
                                  size_t total_repetitions,
                                  size_t n,
                                  int64_t covered_lo,
                                  int64_t covered_hi,
                                  int64_t& need_lo,
                                  int64_t& need_hi)
{
    return detail::coverage_ok_for_range(
        cfg, cir_axis_origin, samples_per_symbol, skip_initial,
        max_repetitions, total_repetitions, n, covered_lo, covered_hi, need_lo,
        need_hi);
}

// May allocate. Call at init / when the TX packet profile changes.
// Identity (code, SFD, SYNC length, sps, tx_profile_id) is frozen here;
// radar_cir_one rejects any runtime config that disagrees.
//
// Besides the legacy scalar scratch this prepares:
//   * the shared sparse SYNC code (`sync_cir::prepare_sync_code`) sized for
//     max_pre/max_post,
//   * the raw pulse-shaped SYNC symbol the shared front-end needs,
//   * the Qm35825Profile built from `cfg` (see the mapping below).
//
// Config → profile mapping (documented once, used everywhere):
//   code_index                     → code_index
//   sync_repetitions               → preamble_repetitions
//   sfd_mode                       → sfd_mode (scratch-owned copy)
//   samples_per_symbol             → (not a profile field; 998.4 MS/s grid)
//   cir_pre/cir_post               → cir_pre_samples / cir_post_samples
//   cir_skip_initial + cir_repetitions
//     + extra_cir_skip_initial + extra_cir_repetitions
//                                  → cir_skip_initial_repetitions /
//                                    cir_repetitions = the UNION of every
//                                    derotated consumer's window (the final
//                                    rotation coverage plan)
//   sfd_threshold                  → sfd_detection_threshold
//   sync_refine_threshold          → radar_verification_threshold
//   sfd_search_margin              → timing_search_margin
//   sync_refine_margin             → timing_track_radius
//   (no radar equivalent)          → timing_coarse_stride = 1 (full rate, as
//                                    the legacy radar refine scored every
//                                    integer start), timing_max_backtrack_symbols
//                                    = profile default (40)
//   sample_rate                    → 998.4e6 (kQm35SampleRate)
//   rx_stop_after_sfd              → false (CirOnly must reach the CIR end)
//   rx_fast_sfd / rx_fast_cfo_rotation / rx_seed_refine_tail
//                                  → true (R04: reuse the existing optimised
//                                    comm front-end kernels; the wide search
//                                    stays as an automatic fallback)
inline bool prepare_radar_cir_core(const RadarCirConfig& cfg,
                                   const std::complex<float>* sync_pulse_shaped,
                                   size_t sync_len,
                                   size_t max_pre,
                                   size_t max_post,
                                   RadarCirCoreScratch& scratch)
{
    detail::reset_radar_cir_core_scratch(scratch);

    if (!detail::radar_sfd_mode_known(cfg.sfd_mode) ||
        cfg.sync_repetitions == 0 || cfg.samples_per_symbol == 0 ||
        cfg.code_index < 9 || cfg.code_index > 12 || !sync_pulse_shaped ||
        sync_len == 0) {
        return false;
    }
    if (sync_len != cfg.samples_per_symbol)
        return false;
    if (max_pre == 0 && max_post == 0) {
        max_pre = cfg.cir_pre;
        max_post = cfg.cir_post;
    }
    if (max_post > std::numeric_limits<size_t>::max() - max_pre)
        return false;
    if (cfg.cir_pre > max_pre || cfg.cir_post > max_post)
        return false;

    const auto sfd_seq = demod::GetSfdSequence(cfg.sfd_mode);
    if (sfd_seq.empty())
        return false;
    const int8_t* hrp_code = demod::GetPreambleCode(cfg.code_index);
    if (!hrp_code)
        return false;

    if (!prepare_sfd_template(sfd_seq.data(), sfd_seq.size(),
                              sync_pulse_shaped, sync_len, scratch.sfd)) {
        detail::reset_radar_cir_core_scratch(scratch);
        return false;
    }

    scratch.sync_template.assign(sync_pulse_shaped,
                                 sync_pulse_shaped + sync_len);
    double energy = 0.0;
    for (size_t i = 0; i < sync_len; ++i) {
        const float re = scratch.sync_template[i].real();
        const float im = scratch.sync_template[i].imag();
        if (!std::isfinite(re) || !std::isfinite(im)) {
            detail::reset_radar_cir_core_scratch(scratch);
            return false;
        }
        energy += static_cast<double>(re) * static_cast<double>(re) +
                  static_cast<double>(im) * static_cast<double>(im);
    }
    if (!(energy > 0.0) || !std::isfinite(energy)) {
        detail::reset_radar_cir_core_scratch(scratch);
        return false;
    }
    gr::uwb::core::uwb_l2_normalize(scratch.sync_template);

    if (!prepare_radar_cir_code(hrp_code, demod::kQm35CodeLength, max_pre,
                                max_post, scratch.cir)) {
        detail::reset_radar_cir_core_scratch(scratch);
        return false;
    }

    // Shared CIR kernel: sparse ternary code + preallocated avg/raw/norm.
    if (!demod::sync_cir::prepare_sync_code(hrp_code,
                                            demod::sync_cir::kSyncCirCodeLength,
                                            cfg.code_index, max_pre, max_post,
                                            scratch.code, scratch.cir_scratch)) {
        detail::reset_radar_cir_core_scratch(scratch);
        return false;
    }

    // Raw pulse-shaped SYNC symbol for the shared front-end.
    scratch.sync_pulse.assign(sync_pulse_shaped,
                              sync_pulse_shaped + sync_len);

    scratch.prepared.ready = true;
    scratch.prepared.code_index = cfg.code_index;
    if (!detail::copy_sfd_mode(cfg.sfd_mode, scratch.prepared.sfd_mode,
                               sizeof(scratch.prepared.sfd_mode))) {
        detail::reset_radar_cir_core_scratch(scratch);
        return false;
    }
    scratch.prepared.n_sfd_symbols = sfd_seq.size();
    scratch.prepared.sync_repetitions = cfg.sync_repetitions;
    scratch.prepared.samples_per_symbol = cfg.samples_per_symbol;
    scratch.prepared.tx_profile_id = cfg.tx_profile_id;
    scratch.prepared.max_pre = max_pre;
    scratch.prepared.max_post = max_post;

    // Final-CFO coverage union (R03 item 4).  The front-end plans the final
    // rotation from the profile's two repetition fields, so they must describe
    // every derotated consumer: the main CIR AND the optional alongside
    // average, whose skip/count differ (default skip 0 / 64 reps versus the
    // main 10 / 54).  Using the main window alone would leave the extra
    // consumer reading samples compensated with the bootstrap CFO.
    size_t cov_skip = 0;
    size_t cov_count = 0;
    detail::coverage_union(cfg, cov_skip, cov_count);
    scratch.prepared.coverage_skip_initial = cov_skip;
    scratch.prepared.coverage_repetitions = cov_count;

    // ---- build the shared front-end profile ------------------------------
    demod::Qm35825Profile& p = scratch.demod_profile;
    p = demod::Qm35825Profile{};
    p.sample_rate = demod::kQm35SampleRate; // 998.4e6 work grid
    p.code_index = cfg.code_index;
    p.preamble_repetitions = cfg.sync_repetitions;
    p.cir_pre_samples = cfg.cir_pre;
    p.cir_post_samples = cfg.cir_post;
    p.cir_skip_initial_repetitions = cov_skip;
    p.cir_repetitions = cov_count;
    p.sfd_detection_threshold = cfg.sfd_threshold;
    p.radar_verification_threshold = cfg.sync_refine_threshold;
    p.timing_search_margin =
        (cfg.sfd_search_margin > 0)
            ? static_cast<size_t>(cfg.sfd_search_margin)
            : size_t(0);
    p.timing_coarse_stride = 1; // score every integer start, as radar did
    // The caller supplies a trusted predicted SYNC origin (the public coarse
    // ROI), so the seed search is a local refinement, not an acquisition: the
    // legacy radar path refined within `sync_refine_margin` and never
    // backtracked across symbols.  A narrow seed refine keeps that cost, and a
    // failed narrow search still falls back to the wide search inside
    // stage_timing, so recovery is unchanged.
    p.timing_seed_refine_radius =
        (cfg.sync_refine_margin > 0)
            ? static_cast<size_t>(cfg.sync_refine_margin)
            : size_t(0);
    p.timing_max_backtrack_symbols = 0;
    p.timing_track_radius =
        (cfg.sync_refine_margin > 0)
            ? static_cast<size_t>(cfg.sync_refine_margin)
            : size_t(0);
    // CirOnly must reach the end of the CIR union, never stop at the SFD.
    p.rx_stop_after_sfd = false;
    // R04: reuse the existing optimised comm front-end kernels instead of
    // adding a second detection algorithm.  These three are the same switches
    // the comm fast path uses; the sensing path was still paying the scalar
    // SFD search, the four-chain CFO rotation and the wide tail re-track.
    //   rx_fast_sfd            -> VOLK/AVX2 SFD correlation + template cache
    //   rx_fast_cfo_rotation   -> 1024-sample block rotation kernel
    //   rx_seed_refine_tail    -> SFD-anchored +/-8 tail seed with the legacy
    //                             wide search kept as an automatic fallback
    // No threshold, ROI, coefficient or output coordinate changes with them;
    // the fallback path and its counters stay observable.
    p.rx_fast_sfd = true;
    p.rx_fast_cfo_rotation = true;
    p.rx_seed_refine_tail = true;
    detail::rebind_demod_profile(scratch);
    return true;
}

// May allocate (vector reserve only).  Optional: gives the shared front-end
// scratch room for an `n`-sample window so the first packet does not grow
// vectors inside the hot path.
inline void reserve_radar_cir_core(size_t n, RadarCirCoreScratch& scratch)
{
    if (n > 0)
        scratch.demod.reserve(n);
}

// HOT PATH: no heap alloc / no vector growth.
inline bool radar_cir_one(const std::complex<float>* rx,
                          size_t n,
                          int64_t predicted_sfd_start,
                          const RadarCirConfig& cfg,
                          RadarCirCoreScratch& scratch,
                          RadarCirResult& out)
{
    out = RadarCirResult{};
    out.predicted_sfd_start = predicted_sfd_start;

    // The profile borrows `prepared.sfd_mode`; re-bind before every use.
    detail::rebind_demod_profile(scratch);

    const bool identity_ok = detail::profile_matches(scratch.prepared, cfg);
    const bool thr_ok = cfg.use_predicted_timing ||
                        (detail::finite_positive(cfg.sfd_threshold) &&
                         detail::finite_positive(cfg.sync_refine_threshold));
    const bool taps_ok =
        cfg.cir_post <= std::numeric_limits<size_t>::max() - cfg.cir_pre &&
        (cfg.cir_pre + cfg.cir_post) > 0;
    const size_t want_taps = taps_ok ? (cfg.cir_pre + cfg.cir_post) : 0;
    const bool scratch_ok =
        identity_ok && !scratch.sfd.sfd_template.empty() &&
        scratch.sync_template.size() == scratch.prepared.samples_per_symbol &&
        scratch.sync_pulse.size() == scratch.prepared.samples_per_symbol &&
        scratch.code.ready && !scratch.cir.sampled_code.empty() &&
        scratch.cir.raw_taps.size() >= want_taps &&
        scratch.cir.norm_taps.size() >= want_taps &&
        scratch.cir_scratch.raw.size() >= want_taps &&
        scratch.cir_scratch.norm.size() >= want_taps;
    const bool margin_ok =
        cfg.use_predicted_timing ||
        (cfg.sfd_search_margin >= 0 && cfg.sync_refine_margin >= 0);
    // A known CFO must be a usable number; a NaN/Inf would poison the whole
    // derotated buffer, so it is a caller-contract error, not a detection miss.
    const bool cfo_ok =
        !(cfg.use_predicted_timing && cfg.compensate_target_cfo &&
          cfg.target_cfo_known) ||
        std::isfinite(cfg.target_cfo_hz);

    if (!rx || n == 0 || predicted_sfd_start < 0 || !identity_ok || !thr_ok ||
        !scratch_ok || !taps_ok || !margin_ok || !cfo_ok) {
        out.status = RadarCirStatus::InvalidInput;
        out.status_reason = !rx || n == 0            ? "null_input"
                            : predicted_sfd_start < 0 ? "invalid_prediction"
                            : !identity_ok            ? "profile_mismatch"
                            : !thr_ok                 ? "invalid_threshold"
                            : !scratch_ok             ? "scratch_unprepared"
                            : !taps_ok                ? "invalid_tap_window"
                            : !margin_ok              ? "invalid_margin"
                                                      : "invalid_known_cfo";
        detail::zero_cir_taps(scratch.cir);
        return false;
    }

    // Identity comes from the prepared TX profile, not a second runtime copy.
    const size_t sync_reps = scratch.prepared.sync_repetitions;
    const size_t sps = scratch.prepared.samples_per_symbol;

    // CIR delay axis = public TX-time prediction.  Computed before any
    // detection so the physical delay can never be re-zeroed.
    const int64_t cir_axis =
        nominal_preamble_start(predicted_sfd_start, sync_reps, sps);
    if (cir_axis < 0) {
        out.status = RadarCirStatus::CirFailed;
        out.status_reason = "cir_axis_out_of_range";
        detail::zero_cir_taps(scratch.cir);
        return false;
    }

    // Skip/count is a pure configuration check and runs before any detection:
    // an impossible CIR request must not cost a front-end pass, and it must not
    // publish partial coordinates either (fail closed, nothing leaked).
    if (cfg.cir_skip_initial >= sync_reps || cfg.cir_repetitions == 0) {
        out.status = RadarCirStatus::CirFailed;
        out.status_reason = "invalid_skip_count";
        detail::zero_cir_taps(scratch.cir);
        return false;
    }

    demod::sync_cir::SyncCirRequest req;
    req.cir_axis_origin = cir_axis;
    req.samples_per_symbol = static_cast<double>(sps);
    req.total_repetitions = sync_reps;
    req.skip_initial = cfg.cir_skip_initial;
    req.max_repetitions = cfg.cir_repetitions;
    req.pre = cfg.cir_pre;
    req.post = cfg.cir_post;

    // Coverage request: the UNION of every derotated consumer (main CIR plus
    // the optional alongside average).  The front-end plans the final rotation
    // from the request it is handed, so the union -- not just the main CIR --
    // must be passed in, otherwise the extra consumer would read samples left
    // on the bootstrap-CFO epoch.  The CIR kernel itself still uses `req`.
    demod::sync_cir::SyncCirRequest cov_req = req;
    cov_req.skip_initial = scratch.prepared.coverage_skip_initial;
    cov_req.max_repetitions = scratch.prepared.coverage_repetitions;

    const std::complex<float>* cir_rx = rx;

    if (cfg.use_predicted_timing) {
        // Timed echo: arrival is the schedule, not a detected SFD, so the
        // published detection coordinates stay the predicted ones.  The shared
        // front-end is used ONLY to estimate the target CFO (and to build the
        // compensated buffer) when the caller did not supply one.
        out.sfd_start_sample = predicted_sfd_start;
        out.sfd_metric = 0.f;
        out.preamble_start_sample = cir_axis;
        out.detected_sync_origin = cir_axis;
        out.sync_metric = 0.f;
        out.cir_axis_origin = cir_axis;
        out.cir_origin_sample = cir_axis;
        out.phase_reference_sample = 0;
        out.cir_samples_per_symbol = req.samples_per_symbol;

        if (!cfg.compensate_target_cfo) {
            // Explicit uncompensated control arm: no front-end pass, the
            // kernel consumes the RAW window.  `cfo_epoch == 0` and
            // `cfo_source == "none"` make that unmistakable.
            out.cfo_source = "none";
        } else if (cfg.target_cfo_known) {
            // Caller-supplied target CFO: derotate the CIR union from the raw
            // input (never from an already rotated buffer).  The range is
            // planned from the UNION request, so every derotated consumer
            // (main CIR and the alongside average) is covered.
            int64_t lo = -1;
            int64_t hi = -1;
            if (!demod::sync_cir::required_input_range(
                    cov_req, demod::sync_cir::kSyncCirCodeLength, n, lo, hi)) {
                out.status = RadarCirStatus::CirFailed;
                out.status_reason = "coverage_plan_failed";
                detail::zero_cir_taps(scratch.cir);
                return false;
            }
            lo = std::max<int64_t>(0, lo);
            hi = std::min<int64_t>(static_cast<int64_t>(n), hi);
            if (hi <= lo) {
                out.status = RadarCirStatus::CirFailed;
                out.status_reason = "coverage_empty";
                detail::zero_cir_taps(scratch.cir);
                return false;
            }
            uint64_t rot_us = 0;
            uint32_t rot_kernel = 0;
            demod::core::derotate_cfo_range(
                rx, n, scratch.demod_profile, cfg.target_cfo_hz,
                static_cast<size_t>(lo), static_cast<size_t>(hi), scratch.demod,
                rot_us, rot_kernel);
            // A CFO rotation is not coverage planning: report it under the CFO
            // sub-timer so coverage_us keeps one meaning everywhere.
            out.frontend_cfo_us = rot_us;
            out.final_cfo_hz = cfg.target_cfo_hz;
            out.cfo_epoch = 2; // final-CFO compensated buffer
            out.cfo_source = "caller_known";
            out.covered_lo = lo;
            out.covered_hi = hi;
            cir_rx = scratch.demod.derotated.data();
        } else {
            // ---- timed mode with an UNKNOWN target CFO (R03 item 2) -------
            // There is no detection of its own, so run the shared front-end
            // seeded with the known TX-time axis and the configured period.
            // It is used for the CFO estimate and the compensated buffer only:
            // the CIR axis stays the TX prediction (never re-zeroed on the
            // detected peak).  If the echo cannot be acquired the call fails
            // closed with a reason; it never returns Ok with an uncompensated
            // CIR.
            demod::PreambleRxContext ctx;
            const bool pre_ok = demod::acquire_preamble(
                rx, n, scratch.demod_profile, scratch.sync_pulse, cir_axis,
                demod::PreamblePurpose::CirOnly, &cov_req, scratch.demod, ctx,
                static_cast<double>(sps));

            out.frontend_timing_us = ctx.timing_us;
            out.frontend_cfo_us = ctx.cfo_us;
            out.frontend_sfd_us = ctx.sfd_us;
            out.coverage_us = ctx.coverage_us;

            if (!pre_ok) {
                out.status = detail::map_demod_status(ctx.status);
                out.status_reason = "timed_cfo_estimate_failed";
                detail::zero_cir_taps(scratch.cir);
                return false;
            }
            // Same public coarse-ROI gate as the detected path: an SFD far
            // outside the requested window is a different hypothesis, and its
            // CFO estimate must not be applied to this echo.
            if (cfg.sfd_search_margin >= 0 &&
                std::llabs(ctx.sfd.sfd_start_sample - predicted_sfd_start) >
                    cfg.sfd_search_margin) {
                out.status = RadarCirStatus::SfdFailed;
                out.status_reason = "timed_cfo_sfd_outside_roi";
                detail::zero_cir_taps(scratch.cir);
                return false;
            }
            if (ctx.cfo_epoch != 2u || !ctx.derotated_valid) {
                out.status = RadarCirStatus::CirFailed;
                out.status_reason = "timed_cfo_unavailable";
                detail::zero_cir_taps(scratch.cir);
                return false;
            }
            // Plan the read with the period that will actually be used (the
            // front-end's measured P, not the nominal one) and prove the CIR
            // union is inside the advertised final-CFO coverage.
            req.samples_per_symbol = (ctx.samples_per_symbol > 0.0)
                                         ? ctx.samples_per_symbol
                                         : static_cast<double>(sps);
            out.cir_samples_per_symbol = req.samples_per_symbol;
            int64_t need_lo = -1;
            int64_t need_hi = -1;
            if (!detail::coverage_contains(req, n, ctx.covered_lo,
                                           ctx.covered_hi, need_lo, need_hi)) {
                out.status = RadarCirStatus::CirFailed;
                out.status_reason = "coverage_insufficient";
                detail::zero_cir_taps(scratch.cir);
                return false;
            }
            out.final_cfo_hz = ctx.final_cfo_hz;
            out.bootstrap_cfo_hz = ctx.bootstrap_cfo_hz;
            out.cfo_epoch = ctx.cfo_epoch;
            out.cfo_source = "estimated_timed";
            out.phase_reference_sample = ctx.phase_reference_sample;
            out.covered_lo = ctx.covered_lo;
            out.covered_hi = ctx.covered_hi;
            cir_rx = scratch.demod.derotated.data();
        }
    } else {
        // Shared preamble front-end (timing -> CFO -> SFD -> final CFO
        // coverage).  The seed is the predicted SYNC origin, not the SFD.
        demod::PreambleRxContext ctx;
        const bool pre_ok = demod::acquire_preamble(
            rx, n, scratch.demod_profile, scratch.sync_pulse, cir_axis,
            demod::PreamblePurpose::CirOnly, &cov_req, scratch.demod, ctx,
            static_cast<double>(sps));

        out.frontend_timing_us = ctx.timing_us;
        out.frontend_cfo_us = ctx.cfo_us;
        out.frontend_sfd_us = ctx.sfd_us;
        out.coverage_us = ctx.coverage_us;

        if (!pre_ok) {
            out.status = detail::map_demod_status(ctx.status);
            out.status_reason = "frontend_acquire_failed";
            detail::zero_cir_taps(scratch.cir);
            return false;
        }

        // ---- public coarse-ROI gate (plan §5) ----------------------------
        // The caller asked for an SFD inside
        // [predicted_sfd_start - sfd_search_margin, +margin] (that is exactly
        // what the legacy radar SFD search scored).  The shared front-end also
        // probes +/-4 SYMBOLS for comm robustness, so a hit outside the
        // requested ROI is a different hypothesis, not the echo the caller
        // asked about: reject it instead of silently accepting a far-away
        // false peak.
        if (std::llabs(ctx.sfd.sfd_start_sample - predicted_sfd_start) >
            cfg.sfd_search_margin) {
            out.status = RadarCirStatus::SfdFailed;
            out.status_reason = "sfd_outside_roi";
            detail::zero_cir_taps(scratch.cir);
            return false;
        }
        // A negative SYNC origin cannot be a detected absolute coordinate.
        if (ctx.detected_sync_origin < 0) {
            out.status = RadarCirStatus::TimingFailed;
            out.status_reason = "negative_sync_origin";
            detail::zero_cir_taps(scratch.cir);
            return false;
        }

        // Detected origin / SFD / metrics are published only on success, so a
        // failed call can never leak a seed-only or stale coordinate.
        out.sfd_start_sample = ctx.sfd.sfd_start_sample;
        out.sfd_metric = ctx.sfd.metric;
        out.preamble_start_sample = ctx.detected_sync_origin;
        out.detected_sync_origin = ctx.detected_sync_origin;
        out.sync_metric = ctx.timing.metric;
        out.final_cfo_hz = ctx.final_cfo_hz;
        out.bootstrap_cfo_hz = ctx.bootstrap_cfo_hz;
        out.cfo_epoch = ctx.cfo_epoch;
        out.phase_reference_sample = ctx.phase_reference_sample;
        out.cir_axis_origin = cir_axis;
        out.cir_origin_sample = cir_axis;

        // Unified period: the front-end's measured period, falling back to the
        // configured integer grid.  Used as a double, never rounded twice.  The
        // coverage plan inside the front-end used this same measured period, so
        // planning and reading cannot disagree.
        req.samples_per_symbol = (ctx.samples_per_symbol > 0.0)
                                     ? ctx.samples_per_symbol
                                     : static_cast<double>(sps);
        out.cir_samples_per_symbol = req.samples_per_symbol;

        if (!cfg.compensate_target_cfo) {
            // Explicit control arm: detection + CFO estimate ran, but the CIR
            // kernel consumes the RAW window.  `final_cfo_hz` reports what was
            // estimated and `cfo_epoch == 0` says it was NOT applied, so this
            // can never be mistaken for a compensated result.
            out.cfo_epoch = 0;
            out.cfo_source = "estimated_not_applied";
            cir_rx = rx;
        } else {
            // ---- fail-closed coverage / epoch validation ------------------
            // The derotated buffer is only valid on [covered_lo, covered_hi)
            // at cfo_epoch == 2.  Everything the kernel will read must be
            // inside it, otherwise the old epoch (bootstrap CFO) or a stale
            // window would be consumed silently.
            int64_t need_lo = -1;
            int64_t need_hi = -1;
            const bool need_ok = detail::coverage_contains(
                req, n, static_cast<int64_t>(ctx.covered_lo),
                static_cast<int64_t>(ctx.covered_hi), need_lo, need_hi);
            if (!need_ok || ctx.cfo_epoch != 2u || !ctx.derotated_valid) {
                out.status = RadarCirStatus::CirFailed;
                out.status_reason =
                    (ctx.cfo_epoch != 2u || !ctx.derotated_valid)
                        ? "final_cfo_unavailable"
                        : "coverage_insufficient";
                detail::zero_cir_taps(scratch.cir);
                return false;
            }
            out.cfo_source = "estimated_detected";
            out.covered_lo = static_cast<int64_t>(ctx.covered_lo);
            out.covered_hi = static_cast<int64_t>(ctx.covered_hi);
            cir_rx = scratch.demod.derotated.data();
        }
    }

    // Skip/count was validated before detection (see above); this second copy
    // is unreachable for a well-formed config and exists only so the CIR call
    // below cannot be reached with an empty repetition set.
    if (cfg.cir_skip_initial >= sync_reps || cfg.cir_repetitions == 0) {
        out.status = RadarCirStatus::CirFailed;
        out.status_reason = "invalid_skip_count";
        detail::zero_cir_taps(scratch.cir);
        return false;
    }

    const size_t tap_count = cfg.cir_pre + cfg.cir_post;
    demod::sync_cir::SyncCirResult cir;
    const auto cir_t0 = std::chrono::steady_clock::now();
    const bool cir_ok = demod::sync_cir::estimate_sync_cir(
        cir_rx, n, req, scratch.code, scratch.cir_scratch, cir);
    out.cir_us = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - cir_t0).count());
    if (!cir_ok || cir.status != demod::sync_cir::SyncCirStatus::Ok ||
        cir.tap_count != tap_count || tap_count == 0) {
        out.status = RadarCirStatus::CirFailed;
        out.status_reason = "cir_kernel_failed";
        detail::zero_cir_taps(scratch.cir);
        return false;
    }

    // Publish through the legacy scratch so every existing consumer keeps
    // working.  `raw` carries the true amplitude (never the normalized taps).
    std::copy_n(scratch.cir_scratch.raw.data(), tap_count,
                scratch.cir.raw_taps.data());
    std::copy_n(scratch.cir_scratch.norm.data(), tap_count,
                scratch.cir.norm_taps.data());

    out.status = RadarCirStatus::Ok;
    out.tap_count = tap_count;
    out.peak_tap = cir.peak_tap;
    out.peak_abs = cir.peak_abs;
    out.raw_l2_norm = cir.raw_l2_norm;
    out.valid_repetitions = cir.valid_repetitions;
    return true;
}

// Diagnostic helper: coherent average of an explicit repetition range through
// the SAME shared kernel radar_cir_one uses.  `rx` must be the buffer
// radar_cir_one consumed (i.e. scratch.demod.derotated when
// detail::radar_cir_uses_derotated(cfg) is true).  No allocation; taps are
// published into scratch.cir exactly like radar_cir_one does.
//
// `covered_lo`/`covered_hi` are the final-CFO coverage advertised by the
// radar_cir_one call this average belongs to.  When they are >= 0 the helper
// validates that THIS consumer's windows are inside the coverage and fails
// closed with `status_reason == "coverage_insufficient"` otherwise; a pointer
// into `demod.derotated` is not proof that the samples were compensated with
// the final CFO.  Pass -1/-1 only for a raw (uncompensated) buffer.
inline bool radar_cir_average_range(const std::complex<float>* rx,
                                    size_t n,
                                    const RadarCirConfig& cfg,
                                    int64_t cir_axis_origin,
                                    double samples_per_symbol,
                                    size_t skip_initial,
                                    size_t max_repetitions,
                                    RadarCirCoreScratch& scratch,
                                    RadarCirResult& out,
                                    int64_t covered_lo = -1,
                                    int64_t covered_hi = -1)
{
    out = RadarCirResult{};
    out.cir_axis_origin = cir_axis_origin;
    out.cir_origin_sample = cir_axis_origin;
    out.cir_samples_per_symbol = samples_per_symbol;

    const size_t tap_count = cfg.cir_pre + cfg.cir_post;
    if (!rx || n == 0 || cir_axis_origin < 0 || tap_count == 0 ||
        !(samples_per_symbol > 0.0) || !std::isfinite(samples_per_symbol) ||
        !scratch.code.ready || scratch.cir.raw_taps.size() < tap_count ||
        scratch.cir.norm_taps.size() < tap_count) {
        out.status = RadarCirStatus::InvalidInput;
        out.status_reason = "invalid_average_request";
        detail::zero_cir_taps(scratch.cir);
        return false;
    }

    demod::sync_cir::SyncCirRequest req;
    req.cir_axis_origin = cir_axis_origin;
    req.samples_per_symbol = samples_per_symbol;
    req.total_repetitions = scratch.prepared.sync_repetitions;
    req.skip_initial = skip_initial;
    req.max_repetitions = max_repetitions;
    req.pre = cfg.cir_pre;
    req.post = cfg.cir_post;

    if (covered_lo >= 0 || covered_hi >= 0) {
        int64_t need_lo = -1;
        int64_t need_hi = -1;
        if (!detail::coverage_contains(req, n, covered_lo, covered_hi, need_lo,
                                       need_hi)) {
            out.status = RadarCirStatus::CirFailed;
            out.status_reason = "coverage_insufficient";
            detail::zero_cir_taps(scratch.cir);
            return false;
        }
        out.covered_lo = covered_lo;
        out.covered_hi = covered_hi;
    }

    demod::sync_cir::SyncCirResult cir;
    if (!demod::sync_cir::estimate_sync_cir(rx, n, req, scratch.code,
                                            scratch.cir_scratch, cir) ||
        cir.status != demod::sync_cir::SyncCirStatus::Ok ||
        cir.tap_count != tap_count) {
        out.status = RadarCirStatus::CirFailed;
        out.status_reason = "cir_kernel_failed";
        detail::zero_cir_taps(scratch.cir);
        return false;
    }
    std::copy_n(scratch.cir_scratch.raw.data(), tap_count,
                scratch.cir.raw_taps.data());
    std::copy_n(scratch.cir_scratch.norm.data(), tap_count,
                scratch.cir.norm_taps.data());

    out.status = RadarCirStatus::Ok;
    out.tap_count = tap_count;
    out.peak_tap = cir.peak_tap;
    out.peak_abs = cir.peak_abs;
    out.raw_l2_norm = cir.raw_l2_norm;
    out.valid_repetitions = cir.valid_repetitions;
    return true;
}

} // namespace radar
} // namespace uwb
} // namespace gr
