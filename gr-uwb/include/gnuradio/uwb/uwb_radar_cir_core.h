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
 *       there is no detection to estimate a CFO from, so the path keeps the
 *       historical uncompensated behaviour.  This is an EXPLICIT adaptation:
 *       `final_cfo_hz == 0` and `cfo_epoch == 0` are published so a caller can
 *       tell "assumed zero" from "estimated zero".  Do not read this as
 *       "skipping SFD implies the target CFO is zero".
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
    // explicit "assumed zero target CFO" adaptation of the timed path.
    double final_cfo_hz = 0.0;
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
inline bool radar_cir_uses_derotated(const RadarCirConfig& cfg)
{
    if (!cfg.compensate_target_cfo)
        return false;
    if (cfg.use_predicted_timing)
        return cfg.target_cfo_known;
    return true;
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
//   cir_skip_initial               → cir_skip_initial_repetitions
//   cir_repetitions                → cir_repetitions
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
//   rx_fast_* / rx_seed_refine_tail→ profile defaults (off): the sensing path
//                                    inherits the comm front-end's default
//                                    numerics rather than a benchmark profile.
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

    // ---- build the shared front-end profile ------------------------------
    demod::Qm35825Profile& p = scratch.demod_profile;
    p = demod::Qm35825Profile{};
    p.sample_rate = demod::kQm35SampleRate; // 998.4e6 work grid
    p.code_index = cfg.code_index;
    p.preamble_repetitions = cfg.sync_repetitions;
    p.cir_pre_samples = cfg.cir_pre;
    p.cir_post_samples = cfg.cir_post;
    p.cir_skip_initial_repetitions = cfg.cir_skip_initial;
    p.cir_repetitions = cfg.cir_repetitions;
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
        detail::zero_cir_taps(scratch.cir);
        return false;
    }

    // Skip/count is a pure configuration check and runs before any detection:
    // an impossible CIR request must not cost a front-end pass, and it must not
    // publish partial coordinates either (fail closed, nothing leaked).
    if (cfg.cir_skip_initial >= sync_reps || cfg.cir_repetitions == 0) {
        out.status = RadarCirStatus::CirFailed;
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

    const std::complex<float>* cir_rx = rx;

    if (cfg.use_predicted_timing) {
        // Timed echo: arrival is the schedule, not a detected SFD.
        out.sfd_start_sample = predicted_sfd_start;
        out.sfd_metric = 0.f;
        out.preamble_start_sample = cir_axis;
        out.detected_sync_origin = cir_axis;
        out.sync_metric = 0.f;
        out.cir_axis_origin = cir_axis;
        out.cir_origin_sample = cir_axis;
        out.phase_reference_sample = 0;
        out.cir_samples_per_symbol = req.samples_per_symbol;

        if (cfg.compensate_target_cfo && cfg.target_cfo_known) {
            // Caller-supplied target CFO: derotate the CIR union from the raw
            // input (never from an already rotated buffer).
            int64_t lo = -1;
            int64_t hi = -1;
            if (!demod::sync_cir::required_input_range(
                    req, demod::sync_cir::kSyncCirCodeLength, n, lo, hi)) {
                out.status = RadarCirStatus::CirFailed;
                detail::zero_cir_taps(scratch.cir);
                return false;
            }
            lo = std::max<int64_t>(0, lo);
            hi = std::min<int64_t>(static_cast<int64_t>(n), hi);
            if (hi <= lo) {
                out.status = RadarCirStatus::CirFailed;
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
            cir_rx = scratch.demod.derotated.data();
        }
        // else: explicit adaptation.  No detection is possible, so the target
        // CFO is ASSUMED zero; final_cfo_hz stays 0 and cfo_epoch stays 0 so a
        // consumer can tell this apart from an estimated zero.
    } else {
        // Shared preamble front-end (timing -> CFO -> SFD -> final CFO
        // coverage).  The seed is the predicted SYNC origin, not the SFD.
        demod::PreambleRxContext ctx;
        const bool pre_ok = demod::acquire_preamble(
            rx, n, scratch.demod_profile, scratch.sync_pulse, cir_axis,
            demod::PreamblePurpose::CirOnly, &req, scratch.demod, ctx,
            static_cast<double>(sps));

        out.frontend_timing_us = ctx.timing_us;
        out.frontend_cfo_us = ctx.cfo_us;
        out.frontend_sfd_us = ctx.sfd_us;
        out.coverage_us = ctx.coverage_us;

        if (!pre_ok) {
            out.status = detail::map_demod_status(ctx.status);
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
            detail::zero_cir_taps(scratch.cir);
            return false;
        }
        // A negative SYNC origin cannot be a detected absolute coordinate.
        if (ctx.detected_sync_origin < 0) {
            out.status = RadarCirStatus::TimingFailed;
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
        out.cfo_epoch = ctx.cfo_epoch;
        out.phase_reference_sample = ctx.phase_reference_sample;
        out.cir_axis_origin = cir_axis;
        out.cir_origin_sample = cir_axis;

        // Unified period: the front-end's measured period, falling back to the
        // configured integer grid.  Used as a double, never rounded twice.
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
            cir_rx = rx;
        } else {
            // ---- fail-closed coverage / epoch validation ------------------
            // The derotated buffer is only valid on [covered_lo, covered_hi)
            // at cfo_epoch == 2.  Everything the kernel will read must be
            // inside it, otherwise the old epoch (bootstrap CFO) or a stale
            // window would be consumed silently.
            int64_t need_lo = -1;
            int64_t need_hi = -1;
            const bool need_ok = demod::sync_cir::required_input_range(
                req, demod::sync_cir::kSyncCirCodeLength, n, need_lo, need_hi);
            if (!need_ok || ctx.cfo_epoch != 2u || !ctx.derotated_valid ||
                need_lo < static_cast<int64_t>(ctx.covered_lo) ||
                need_hi > static_cast<int64_t>(ctx.covered_hi)) {
                out.status = RadarCirStatus::CirFailed;
                detail::zero_cir_taps(scratch.cir);
                return false;
            }
            cir_rx = scratch.demod.derotated.data();
        }
    }

    // Skip/count was validated before detection (see above); this second copy
    // is unreachable for a well-formed config and exists only so the CIR call
    // below cannot be reached with an empty repetition set.
    if (cfg.cir_skip_initial >= sync_reps || cfg.cir_repetitions == 0) {
        out.status = RadarCirStatus::CirFailed;
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
inline bool radar_cir_average_range(const std::complex<float>* rx,
                                    size_t n,
                                    const RadarCirConfig& cfg,
                                    int64_t cir_axis_origin,
                                    double samples_per_symbol,
                                    size_t skip_initial,
                                    size_t max_repetitions,
                                    RadarCirCoreScratch& scratch,
                                    RadarCirResult& out)
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

    demod::sync_cir::SyncCirResult cir;
    if (!demod::sync_cir::estimate_sync_cir(rx, n, req, scratch.code,
                                            scratch.cir_scratch, cir) ||
        cir.status != demod::sync_cir::SyncCirStatus::Ok ||
        cir.tap_count != tap_count) {
        out.status = RadarCirStatus::CirFailed;
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
