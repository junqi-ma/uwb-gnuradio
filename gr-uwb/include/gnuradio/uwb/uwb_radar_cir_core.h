/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Header-only monostatic-radar integrator at 998.4 MS/s:
 *   RX window + predicted SFD
 *     → search_sfd
 *     → SFD success ? refine_sync_origin : SfdFailed
 *     → timing success ? estimate_radar_cir : TimingFailed
 *     → CIR success ? Ok (raw+norm taps) : CirFailed
 *
 * Failure: tap_count=0. Do not return seed-only CIR. Failed-stage sample
 * indices stay -1. Do not copy predicted SFD/origin into success fields.
 *
 * sfd_search_margin=64 is a software default, not a hardware freeze.
 * peak_tap is argmax(|raw|) on the estimateCir grid, not physical zero range.
 *
 * HOT PATH (radar_cir_one): no heap allocation / no vector growth.
 */

#pragma once

#include <gnuradio/uwb/uwb_radar_sfd_core.h>
#include <gnuradio/uwb/uwb_radar_timing_core.h>
#include <gnuradio/uwb/uwb_radar_cir_estimator.h>
#include <gnuradio/uwb/uwb_phy_profile.h>

#include <algorithm>
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
    int64_t cir_origin_sample = -1;        // TX-time origin used as CIR delay axis
    int64_t predicted_sfd_start = -1;
    float sfd_metric = 0.f;
    float sync_metric = 0.f;
    size_t tap_count = 0;
    size_t peak_tap = 0;
    float peak_abs = 0.f;
    float raw_l2_norm = 0.f;
    size_t valid_repetitions = 0;
};

struct RadarCirCoreScratch {
    RadarSfdScratch sfd;
    RadarCirScratch cir;
    std::vector<std::complex<float>> sync_template; // L2-normalized, length sps
    PreparedRadarProfile prepared;
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
    scratch.prepared = PreparedRadarProfile{};
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

} // namespace detail

// May allocate. Call at init / when the TX packet profile changes.
// Identity (code, SFD, SYNC length, sps, tx_profile_id) is frozen here;
// radar_cir_one rejects any runtime config that disagrees.
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
    return true;
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

    const bool identity_ok = detail::profile_matches(scratch.prepared, cfg);
    const bool thr_ok = detail::finite_positive(cfg.sfd_threshold) &&
                        detail::finite_positive(cfg.sync_refine_threshold);
    const bool taps_ok =
        cfg.cir_post <= std::numeric_limits<size_t>::max() - cfg.cir_pre &&
        (cfg.cir_pre + cfg.cir_post) > 0;
    const size_t want_taps = taps_ok ? (cfg.cir_pre + cfg.cir_post) : 0;
    const bool scratch_ok =
        identity_ok && !scratch.sfd.sfd_template.empty() &&
        scratch.sync_template.size() == scratch.prepared.samples_per_symbol &&
        !scratch.cir.sampled_code.empty() &&
        scratch.cir.raw_taps.size() >= want_taps &&
        scratch.cir.norm_taps.size() >= want_taps;
    const bool margin_ok =
        cfg.sfd_search_margin >= 0 && cfg.sync_refine_margin >= 0;

    if (!rx || n == 0 || predicted_sfd_start < 0 || !identity_ok || !thr_ok ||
        !scratch_ok || !taps_ok || !margin_ok) {
        out.status = RadarCirStatus::InvalidInput;
        detail::zero_cir_taps(scratch.cir);
        return false;
    }

    // Identity comes from the prepared TX profile, not a second runtime copy.
    const size_t sync_reps = scratch.prepared.sync_repetitions;
    const size_t sps = scratch.prepared.samples_per_symbol;

    RadarSfdResult sfd;
    if (!search_sfd(rx, n, predicted_sfd_start, cfg.sfd_search_margin,
                    cfg.sfd_threshold, sfd, scratch.sfd)) {
        out.sfd_metric = sfd.metric;
        if (sfd.status == SfdStatus::InvalidInput)
            out.status = RadarCirStatus::InvalidInput;
        else
            out.status = RadarCirStatus::SfdFailed;
        detail::zero_cir_taps(scratch.cir);
        return false;
    }
    out.sfd_start_sample = sfd.sfd_start_sample;
    out.sfd_metric = sfd.metric;

    RadarTimingResult timing;
    if (!refine_sync_origin(rx, n, sfd.sfd_start_sample, sync_reps, sps,
                            cfg.sync_refine_margin, cfg.sync_refine_threshold,
                            scratch.sync_template.data(),
                            scratch.sync_template.size(), timing)) {
        out.sync_metric = timing.metric;
        out.status = (timing.status == TimingStatus::InvalidInput)
                         ? RadarCirStatus::InvalidInput
                         : RadarCirStatus::TimingFailed;
        detail::zero_cir_taps(scratch.cir);
        return false;
    }
    out.preamble_start_sample = timing.preamble_start_sample;
    out.sync_metric = timing.metric;

    // CIR delay axis is the TX-time / predicted SYNC origin so a channel
    // delay D appears as a tap shift. Timing refine is a validity gate and
    // fills preamble_start_sample (RX origin); it is not the CIR axis.
    const int64_t cir_origin =
        nominal_preamble_start(predicted_sfd_start, sync_reps, sps);
    out.cir_origin_sample = cir_origin;
    if (cir_origin < 0) {
        out.status = RadarCirStatus::CirFailed;
        detail::zero_cir_taps(scratch.cir);
        return false;
    }

    // Skip/count is computed here so CIR is not invoked when no repetition
    // remains. Estimator also uses min(cir_repetitions, reps-skip).
    if (cfg.cir_skip_initial >= sync_reps || cfg.cir_repetitions == 0) {
        out.status = RadarCirStatus::CirFailed;
        detail::zero_cir_taps(scratch.cir);
        return false;
    }

    RadarCirEstimate cir;
    if (!estimate_radar_cir(rx, n, cir_origin, sps, cfg.cir_pre, cfg.cir_post,
                            cfg.cir_skip_initial, cfg.cir_repetitions,
                            sync_reps, cir, scratch.cir)) {
        out.status = RadarCirStatus::CirFailed;
        detail::zero_cir_taps(scratch.cir);
        return false;
    }

    const size_t tap_count = cfg.cir_pre + cfg.cir_post;
    if (cir.tap_count != tap_count ||
        scratch.cir.raw_taps.size() < tap_count ||
        scratch.cir.norm_taps.size() < tap_count || tap_count == 0) {
        out.status = RadarCirStatus::CirFailed;
        detail::zero_cir_taps(scratch.cir);
        return false;
    }

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
