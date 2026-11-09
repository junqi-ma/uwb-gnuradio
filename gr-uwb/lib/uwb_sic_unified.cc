/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Unified UWB SIC Implementation (Stage M2 Contract).
 * Namespace gr::uwb::sic.
 */

#include <gnuradio/uwb/uwb_sic_unified.h>

#include <chrono>
#include <cmath>
#include <cstring>
#include <limits>

namespace gr {
namespace uwb {
namespace sic {

namespace {

using Clock = std::chrono::steady_clock;

double us(Clock::time_point t)
{
    return std::chrono::duration<double, std::micro>(Clock::now() - t).count();
}

} // namespace

// ---------------------------------------------------------------------------
// SicUnifiedChannel Conversions
// ---------------------------------------------------------------------------

void SicUnifiedChannel::from_discrete_cir(const DiscreteCirModel& m)
{
    valid = m.valid;
    backend = m.impulse_template ? SicEstimatorBackend::Ipatov : SicEstimatorBackend::DiscreteCir;
    status = m.status;
    reason = m.reason;
    model_id = m.model_id;
    q = m.q;
    cfo_hz = m.cfo_hz;
    phase_reference_sample = m.phase_reference_sample;
    observed_end_sample = m.observed_end_sample;
    training_lo = m.training_lo;
    training_hi = m.training_hi;
    fit_residual = m.fit_residual;
    gain_condition = m.gain_condition;
    repeats_confirmed = m.repeats_confirmed;
    lag_lo = m.lag_lo;
    tap_count = m.tap_count;
    active_count = m.active_count;
    active_lags = m.active_lags;
    taps = m.taps;
    impulse_template = m.impulse_template;
    cir_repetitions = m.cir_repetitions;
    n_paths = 0;
}

void SicUnifiedChannel::from_parametric(const SicModel& m)
{
    valid = m.valid;
    backend = SicEstimatorBackend::PathFractional;
    status = m.status;
    reason = m.reason;
    model_id = m.model_id;
    q = m.origin;
    cfo_hz = m.cfo_hz;
    phase_reference_sample = m.phase_reference_sample;
    observed_end_sample = m.observed_end_sample;
    training_lo = m.training_lo;
    training_hi = m.training_hi;
    fit_residual = m.fit_residual;
    gain_condition = m.gain_condition;
    repeats_confirmed = m.training_reps_confirmed;
    lag_lo = 0;
    tap_count = 0;
    active_count = 0;
    impulse_template = false;
    cir_repetitions = 0;
    n_paths = m.n_paths;
    for (size_t k = 0; k < kSicMaxPaths; ++k) {
        tau_abs[k] = m.tau_abs[k];
        gain[k] = m.gain[k];
    }
}

void SicUnifiedChannel::to_discrete_cir(DiscreteCirModel& m) const
{
    m.valid = valid;
    m.status = status;
    m.reason = reason;
    m.model_id = model_id;
    m.q = q;
    m.cfo_hz = cfo_hz;
    m.phase_reference_sample = phase_reference_sample;
    m.observed_end_sample = observed_end_sample;
    m.training_lo = training_lo;
    m.training_hi = training_hi;
    m.fit_residual = fit_residual;
    m.gain_condition = gain_condition;
    m.repeats_confirmed = repeats_confirmed;
    m.lag_lo = lag_lo;
    m.tap_count = tap_count;
    m.active_count = active_count;
    m.active_lags = active_lags;
    m.taps = taps;
    m.impulse_template = impulse_template;
    m.cir_repetitions = cir_repetitions;
}

void SicUnifiedChannel::to_parametric(SicModel& m) const
{
    m.valid = valid;
    m.state = valid ? SicState::Committed : SicState::Failed;
    m.status = status;
    m.reason = reason;
    m.model_id = model_id;
    m.origin = q;
    m.cfo_hz = cfo_hz;
    m.phase_reference_sample = phase_reference_sample;
    m.observed_end_sample = observed_end_sample;
    m.training_lo = training_lo;
    m.training_hi = training_hi;
    m.fit_residual = fit_residual;
    m.gain_condition = gain_condition;
    m.training_reps_confirmed = repeats_confirmed;
    m.n_paths = n_paths;
    for (size_t k = 0; k < kSicMaxPaths; ++k) {
        m.tau_abs[k] = tau_abs[k];
        m.gain[k] = gain[k];
    }
}

// ---------------------------------------------------------------------------
// SicUnifiedConfig Conversions
// ---------------------------------------------------------------------------

SicConfig SicUnifiedConfig::to_sic_config() const
{
    SicConfig c;
    c.mode = SicMode::Estimate;
    c.comm_code = comm_code;
    c.sync_reps = sync_reps;
    c.samples_per_symbol = samples_per_symbol;
    c.sample_rate = sample_rate;
    c.k_paths = k_paths;
    c.fit_reps = fit_reps;
    c.roi_lo = roi_lo;
    c.roi_hi = roi_hi;
    c.sub_roi_lo = sub_roi_lo;
    c.sub_roi_hi = sub_roi_hi;
    c.sub_min_overlap = sub_min_overlap;
    c.coarse_stride = coarse_stride;
    c.post_tail = post_tail;
    c.enable_fractional = enable_fractional;
    c.enable_cfo = enable_cfo;
    c.detect_threshold = detect_threshold;
    c.observed_end_sample = observed_end_sample;
    c.fit_residual_max = fit_residual_max;
    return c;
}

DiscreteCirConfig SicUnifiedConfig::to_discrete_cir_config() const
{
    DiscreteCirConfig c;
    c.ipatov_correlation = (backend == SicEstimatorBackend::Ipatov);
    c.sparse_pulse_reconstruct = (backend == SicEstimatorBackend::Ipatov && sparse_pulse_reconstruct);
    c.comm_code = comm_code;
    c.sync_reps = sync_reps;
    c.samples_per_symbol = samples_per_symbol;
    c.sample_rate = sample_rate;
    c.fit_reps = fit_reps;
    c.roi_lo = roi_lo;
    c.roi_hi = roi_hi;
    c.lag_lo = lag_lo;
    c.tap_count = tap_count;
    c.peak_floor = peak_floor;
    c.peak_min_spacing = peak_min_spacing;
    c.active_radius = active_radius;
    c.lambda = lambda;
    c.coarse_stride = coarse_stride;
    c.detect_threshold = detect_threshold;
    c.fit_residual_max = fit_residual_max;
    c.gain_condition_max = gain_condition_max;
    c.max_window_samples = max_window_samples;
    c.max_packet_samples = max_packet_samples;
    return c;
}

mod::HrpModConfig SicUnifiedConfig::to_hrp_mod_config() const
{
    mod::HrpModConfig c;
    c.code_index = comm_code;
    c.sync_repetitions = sync_reps;
    c.sfd_mode = sfd_mode;
    c.peak_amplitude = 0.8f;
    return c;
}

// ---------------------------------------------------------------------------
// SicUnifiedTemplates Conversions
// ---------------------------------------------------------------------------

SicTemplates SicUnifiedTemplates::to_sic_templates() const
{
    SicTemplates t;
    t.sync = sync;
    t.sync_len = sync_len;
    t.preamble = preamble;
    t.preamble_len = preamble_len;
    return t;
}

DiscreteCirTemplates SicUnifiedTemplates::to_discrete_cir_templates() const
{
    DiscreteCirTemplates t;
    t.sync = sync;
    t.sync_len = sync_len;
    t.train = preamble;
    t.train_len = preamble_len;
    return t;
}

// ---------------------------------------------------------------------------
// SicUnifiedScratch Reservation
// ---------------------------------------------------------------------------

void SicUnifiedScratch::reserve(size_t max_window_samples, size_t max_packet_samples)
{
    if (max_packet_samples == 0) {
        max_packet_samples = max_window_samples;
    }
    capacity_window_samples = max_window_samples;
    capacity_packet_samples = max_packet_samples;

    discrete.reserve(max_window_samples);
    fullpacket.reserve(max_window_samples);
    modulator.reserve(127, max_packet_samples);
    demod.reserve(max_window_samples);

    work_waveform.resize(max_packet_samples);
    work_accum.resize(max_window_samples);
    work_rot.resize(max_window_samples);
    work_phr_bits.resize(19);
    work_wire_psdu.resize(radar_meta::kMaxPsduBytes);
}

// ---------------------------------------------------------------------------
// prepare_sic_unified
// ---------------------------------------------------------------------------

bool prepare_sic_unified(
    const SicUnifiedConfig& cfg,
    const SicUnifiedTemplates& templates,
    SicUnifiedScratch& scratch,
    std::string* err)
{
    if (!templates.sync || templates.sync_len != cfg.samples_per_symbol) {
        if (err) *err = "invalid_sync_template";
        return false;
    }
    if (!templates.preamble || templates.preamble_len == 0) {
        if (err) *err = "invalid_preamble_template";
        return false;
    }

    scratch.reserve(cfg.max_window_samples, cfg.max_packet_samples);

    if (cfg.backend == SicEstimatorBackend::PathFractional) {
        SicConfig sc = cfg.to_sic_config();
        if (!prepare_sic_core(sc, templates.sync, templates.sync_len,
                              templates.preamble, templates.preamble_len,
                              scratch.fractional, err)) {
            return false;
        }
    } else {
        DiscreteCirConfig dc = cfg.to_discrete_cir_config();
        DiscreteCirTemplates dt = templates.to_discrete_cir_templates();
        if (!prepare_discrete_cir(dc, dt, scratch.discrete, err)) {
            return false;
        }
    }

    scratch.prepared = true;
    return true;
}

// ---------------------------------------------------------------------------
// estimate_channel_unified
// ---------------------------------------------------------------------------

bool estimate_channel_unified(
    const std::complex<float>* y,
    size_t n,
    int64_t observed_end,
    const SicUnifiedTemplates& templates,
    const SicUnifiedConfig& cfg,
    SicUnifiedScratch& scratch,
    SicUnifiedChannel& channel,
    SicUnifiedTimings* timings)
{
    channel.valid = false;
    channel.backend = cfg.backend;
    if (!y || n == 0 || !scratch.prepared) {
        channel.status = "invalid_input";
        return false;
    }
    const int64_t obs = (observed_end <= 0) ? int64_t(n) : std::min(int64_t(n), observed_end);

    const auto t_start = Clock::now();

    if (cfg.backend == SicEstimatorBackend::PathFractional) {
        SicModel model;
        SicTimings st;
        SicConfig sc = cfg.to_sic_config();
        sc.observed_end_sample = obs;
        SicTemplates stemp = templates.to_sic_templates();
        bool ok = estimate_model(y, n, obs, stemp, sc, scratch.fractional, model, &st);
        channel.from_parametric(model);
        if (timings) {
            timings->estimate_us = us(t_start);
            timings->est_detect_us = static_cast<double>(st.detect_coarse_us + st.detect_fine_us);
            timings->est_cfo_us = static_cast<double>(st.cfo_us);
            timings->est_cir_us = static_cast<double>(st.cir_us + st.path_select_us);
            timings->est_solve_us = static_cast<double>(st.matrix_build_us + st.gain_fit_us);
            timings->est_fit_check_us = static_cast<double>(st.model_prepare_us);
        }
        return ok;
    } else {
        DiscreteCirModel dm;
        DiscreteCirTimings dt;
        DiscreteCirConfig dc = cfg.to_discrete_cir_config();
        DiscreteCirTemplates dtemp = templates.to_discrete_cir_templates();
        bool ok = estimate_discrete_cir(y, n, obs, dtemp, dc, scratch.discrete, dm, &dt);
        channel.from_discrete_cir(dm);
        if (timings) {
            timings->estimate_us = us(t_start);
            timings->est_detect_us = dt.detect_us;
            timings->est_cfo_us = dt.cfo_us;
            timings->est_cir_us = dt.support_us;
            timings->est_solve_us = dt.projection_us + dt.solve_us;
            timings->est_fit_check_us = dt.fit_check_us;
        }
        return ok;
    }
}

// ---------------------------------------------------------------------------
// reconstruct_and_subtract_unified
// ---------------------------------------------------------------------------

bool reconstruct_and_subtract_unified(
    const std::complex<float>* y,
    size_t n,
    const SicUnifiedChannel& channel,
    SicCancellationLevel level,
    SicKnownFields known_fields,
    const std::complex<float>* s_known,
    size_t s_known_len,
    const SicUnifiedTemplates& templates,
    const SicUnifiedConfig& cfg,
    SicUnifiedScratch& scratch,
    std::complex<float>* out,
    std::complex<float>* i_hat,
    SicUnifiedResult& result)
{
    auto fail_closed = [&](const char* reason) {
        if (out && y && n > 0 && out != y) {
            std::copy(y, y + n, out);
        }
        if (i_hat && i_hat != y && i_hat != out) {
            std::fill(i_hat, i_hat + n, std::complex<float>(0.f, 0.f));
        }
        result.committed = false;
        result.reason = reason;
        result.level = SicCancellationLevel::L0_Bypass;
        result.cancelled_fields = SicKnownFields::None;
        return false;
    };

    if (!y || !out || n == 0 || !channel.valid) {
        return fail_closed("invalid_input_or_channel");
    }

    const auto t_recon = Clock::now();

    if (channel.backend == SicEstimatorBackend::PathFractional) {
        SicModel model;
        channel.to_parametric(model);
        SicConfig sc = cfg.to_sic_config();
        SicTemplates stemp = templates.to_sic_templates();
        SicTimings st;

        if (level == SicCancellationLevel::L1_Preamble) {
            bool ok = apply_model(model, y, n, 0, stemp, sc, scratch.fractional, out, i_hat, &st);
            result.timings.reconstruct_us = static_cast<double>(st.reconstruct_us);
            result.timings.subtract_us = std::max(0.0, us(t_recon) - result.timings.reconstruct_us);
            if (!ok) {
                return fail_closed("apply_model_failed");
            }
            result.committed = true;
            result.reason = "committed_preamble_fractional";
            result.level = SicCancellationLevel::L1_Preamble;
            result.cancelled_fields = SicKnownFields::Preamble;
            result.support_lo = model.predicted_support_lo;
            result.support_hi = model.predicted_support_hi;
            result.sub_lo = model.sub_lo;
            result.sub_hi = model.sub_hi;
            result.sub_overlap = (result.sub_hi > result.sub_lo) ? (result.sub_hi - result.sub_lo) : 0;
            return true;
        } else {
            // FullPacket with PathFractional
            mod::HrpModConfig modcfg = cfg.to_hrp_mod_config();
            FullPacketResult f = fullpacket_cancel(
                y, n, scratch.work_phr_bits, scratch.work_wire_psdu,
                true, modcfg, model, templates.sync, templates.sync_len,
                cfg.sub_roi_lo, cfg.sub_roi_hi, scratch.fullpacket, out, i_hat);
            result.timings.reconstruct_us = f.channel_us;
            result.timings.subtract_us = f.subtract_us;
            result.waveform_scale = f.waveform_scale;
            if (!f.committed) {
                return fail_closed(f.reason);
            }
            result.committed = true;
            result.reason = "committed_full_packet_fractional";
            result.level = SicCancellationLevel::L2_FullPacket;
            result.cancelled_fields = SicKnownFields::FullPacket;
            result.sub_lo = f.sub_lo;
            result.sub_hi = f.sub_hi;
            result.sub_overlap = (result.sub_hi > result.sub_lo) ? (result.sub_hi - result.sub_lo) : 0;
            return true;
        }
    } else {
        // Discrete CIR / Ipatov backend
        DiscreteCirModel dm;
        channel.to_discrete_cir(dm);
        DiscreteCirResult dres;

        if (level == SicCancellationLevel::L1_Preamble) {
            const std::complex<float>* wf = s_known ? s_known : templates.preamble;
            size_t wf_n = s_known ? s_known_len : templates.preamble_len;

            bool ok = apply_discrete_cir(
                y, n, wf, wf_n, templates.sync, templates.sync_len, dm,
                cfg.sub_roi_lo, cfg.sub_roi_hi, scratch.discrete, out, i_hat, &dres);
            result.timings.reconstruct_us = dres.channel_us;
            result.timings.subtract_us = dres.subtract_us;
            result.waveform_scale = dres.waveform_scale;
            if (!ok || !dres.committed) {
                return fail_closed(dres.reason ? dres.reason : "apply_discrete_failed");
            }
            result.committed = true;
            result.reason = "committed_preamble_discrete";
            result.level = SicCancellationLevel::L1_Preamble;
            result.cancelled_fields = known_fields;
            result.support_lo = dres.support_lo;
            result.support_hi = dres.support_hi;
            result.sub_lo = dres.sub_lo;
            result.sub_hi = dres.sub_hi;
            result.sub_overlap = (result.sub_hi > result.sub_lo) ? (result.sub_hi - result.sub_lo) : 0;
            return true;
        } else {
            // FullPacket with DiscreteCir / Ipatov
            mod::HrpModConfig modcfg = cfg.to_hrp_mod_config();
            dres = fullpacket_cancel_discrete(
                y, n, scratch.work_phr_bits, scratch.work_wire_psdu,
                true, modcfg, dm, templates.sync, templates.sync_len,
                cfg.sub_roi_lo, cfg.sub_roi_hi, scratch.discrete, out, i_hat);
            result.timings.tx_remod_us = dres.tx_us;
            result.timings.reconstruct_us = dres.channel_us;
            result.timings.subtract_us = dres.subtract_us;
            result.waveform_scale = dres.waveform_scale;
            if (!dres.committed) {
                return fail_closed(dres.reason);
            }
            result.committed = true;
            result.reason = "committed_full_packet_discrete";
            result.level = SicCancellationLevel::L2_FullPacket;
            result.cancelled_fields = SicKnownFields::FullPacket;
            result.support_lo = dres.support_lo;
            result.support_hi = dres.support_hi;
            result.sub_lo = dres.sub_lo;
            result.sub_hi = dres.sub_hi;
            result.sub_overlap = (result.sub_hi > result.sub_lo) ? (result.sub_hi - result.sub_lo) : 0;
            return true;
        }
    }
}

// ---------------------------------------------------------------------------
// sic_process_unified
// ---------------------------------------------------------------------------

bool sic_process_unified(
    const std::complex<float>* y,
    size_t n,
    const SicUnifiedTemplates& templates,
    const SicUnifiedConfig& cfg,
    SicUnifiedScratch& scratch,
    SicUnifiedResult& result)
{
    const auto t_core_start = Clock::now();
    result.policy = cfg.policy;
    result.committed = false;
    result.level = SicCancellationLevel::L0_Bypass;
    result.cancelled_fields = SicKnownFields::None;
    result.suppression_db = 0.0;
    result.fallback_reason = nullptr;

    if (!y || !result.out || n == 0 || !scratch.prepared) {
        result.reason = "invalid_buffers_or_scratch";
        if (result.out && y && n > 0 && result.out != y) {
            std::copy(y, y + n, result.out);
        }
        if (result.i_hat) {
            std::fill(result.i_hat, result.i_hat + n, std::complex<float>(0.f, 0.f));
        }
        return false;
    }

    // 1. Strict-prefix Channel Estimation
    SicUnifiedChannel channel;
    const bool est_ok = estimate_channel_unified(
        y, n, cfg.observed_end_sample, templates, cfg, scratch,
        channel, &result.timings);
    result.channel = channel;

    if (!est_ok || !channel.valid) {
        if (result.out != y) {
            std::copy(y, y + n, result.out);
        }
        if (result.i_hat) {
            std::fill(result.i_hat, result.i_hat + n, std::complex<float>(0.f, 0.f));
        }
        result.reason = channel.reason ? channel.reason : channel.status;
        result.timings.core_total_us = us(t_core_start);
        return true;
    }

    // 2. Cancellation Policy & Comm RX Demodulation (SYNC + SFD for both PreambleOnly and FullPacket)
    SicCancellationLevel target_level = SicCancellationLevel::L0_Bypass;
    SicKnownFields target_fields = SicKnownFields::None;
    const std::complex<float>* s_known_ptr = templates.preamble;
    size_t s_known_len = templates.preamble_len;

    std::vector<std::complex<float>> sync_vec(templates.sync, templates.sync + templates.sync_len);

    auto demod_prof = demod::Qm35825Profile::Default();
    demod_prof.code_index = cfg.comm_code;
    demod_prof.timing_search_margin = 256;
    demod_prof.timing_coarse_stride = 1;
    demod_prof.sfd_mode = cfg.sfd_mode;
    demod_prof.rx_fast_sfd = cfg.rx_fast_sfd;
    demod_prof.rx_fast_cfo_rotation = cfg.rx_fast_cfo;
    demod_prof.rx_seed_refine_tail = cfg.rx_seed_refine_tail;
    demod_prof.rx_softchip_on_demand = cfg.rx_softchip_on_demand;
    demod_prof.rx_fused_payload_cmf = cfg.rx_fused_payload_cmf;
    demod_prof.rx_fused_phr_cmf = cfg.rx_fused_phr_cmf;

    if (cfg.policy == SicPolicy::PreambleOnly) {
        // Preamble-only: run the exact same SYNC + SFD detection flow as full packet!
        // Early terminate right after Final SFD verification to avoid CIR/PHR/payload overhead.
        demod_prof.rx_stop_after_sfd = true;
        const auto t_rx = Clock::now();
        demod::DemodResult decoded = demod::core::demodulate_one(
            y, n, demod_prof, 0, channel.q, 0, sync_vec, scratch.demod);
        result.timings.comm_rx_us = us(t_rx);
        result.timings.rx_timing_us = static_cast<double>(decoded.stage_timing_us);
        result.timings.rx_cfo_us = static_cast<double>(decoded.stage_cfo_us);
        result.timings.rx_sfd_us = static_cast<double>(decoded.stage_sfd_us);

        result.comm_detected = (decoded.status != demod::DemodStatus::TimingFailed &&
                                decoded.status != demod::DemodStatus::InvalidInput);

        const bool sfd_ok = (decoded.status == demod::DemodStatus::Success && decoded.sfd.ok);
        if (sfd_ok) {
            // High-precision SYNC+SFD preamble model
            target_level = SicCancellationLevel::L1_Preamble;
            target_fields = SicKnownFields::PreambleSfd;

            // Generate the exact SYNC + SFD known waveform
            mod::HrpModConfig modcfg = cfg.to_hrp_mod_config();
            size_t n_prefix = 0;
            if (mod::modulate_prefix_waveform(modcfg, scratch.modulator,
                                              scratch.work_waveform.data(),
                                              scratch.work_waveform.size(),
                                              n_prefix) && n_prefix > 0) {
                s_known_ptr = scratch.work_waveform.data();
                s_known_len = n_prefix;
            }
        } else {
            // SFD detection failed on preamble-only
            if (result.out != y) {
                std::copy(y, y + n, result.out);
            }
            if (result.i_hat) {
                std::fill(result.i_hat, result.i_hat + n, std::complex<float>(0.f, 0.f));
            }
            result.reason = "sfd_detect_failed";
            result.level = SicCancellationLevel::L0_Bypass;
            result.cancelled_fields = SicKnownFields::None;
            result.timings.core_total_us = us(t_core_start);
            return true;
        }
    } else {
        // FullPacket or AutoFallback: attempt fast comm demodulation
        demod_prof.rx_stop_after_sfd = false;
        const auto t_rx = Clock::now();
        demod::DemodResult decoded = demod::core::demodulate_one(
            y, n, demod_prof, 0, channel.q, 0, sync_vec, scratch.demod);
        result.timings.comm_rx_us = us(t_rx);
        result.timings.rx_timing_us = static_cast<double>(decoded.stage_timing_us);
        result.timings.rx_cfo_us = static_cast<double>(decoded.stage_cfo_us);
        result.timings.rx_sfd_us = static_cast<double>(decoded.stage_sfd_us);
        result.timings.rx_cir_us = static_cast<double>(decoded.stage_cir_us);
        result.timings.rx_phr_us = static_cast<double>(decoded.stage_phr_us);
        result.timings.rx_payload_us = static_cast<double>(decoded.stage_payload_us);

        result.comm_detected = (decoded.status != demod::DemodStatus::TimingFailed &&
                                decoded.status != demod::DemodStatus::InvalidInput);
        result.comm_phr_pass = (decoded.phr.psdu_length > 0 && !decoded.phr.secded_uncorrectable);
        result.comm_fcs_pass = decoded.payload.fcs_pass;
        result.phr_secded_corrected = decoded.phr.secded_corrected;
        result.phr_secded_uncorrectable = decoded.phr.secded_uncorrectable;
        result.comm_phr_length = decoded.phr.psdu_length;
        result.comm_phr_rate_mbps = decoded.phr.data_rate_mbps;

        if (decoded.payload.bytes.size() <= 127) {
            result.wire_psdu_len = decoded.payload.bytes.size();
            std::copy(decoded.payload.bytes.begin(), decoded.payload.bytes.end(), result.wire_psdu);
            scratch.work_wire_psdu = decoded.payload.bytes;
        }
        scratch.work_phr_bits = decoded.phr.phr_bits;

        const bool demod_success = (decoded.status == demod::DemodStatus::Success &&
                                    decoded.payload.fcs_pass &&
                                    decoded.phr.phr_bits.size() == 19);

        if (demod_success) {
            target_level = SicCancellationLevel::L2_FullPacket;
            target_fields = SicKnownFields::FullPacket;
        } else {
            const char* fb = !result.comm_detected ? "comm_detect_failed"
                           : (!result.comm_phr_pass ? "comm_phr_failed"
                           : (!result.comm_fcs_pass ? "comm_fcs_failed"
                           : "comm_demod_failed"));

            if (cfg.policy == SicPolicy::FullPacket) {
                // Strict full-packet: abort to L0 Bypass
                if (result.out != y) {
                    std::copy(y, y + n, result.out);
                }
                if (result.i_hat) {
                    std::fill(result.i_hat, result.i_hat + n, std::complex<float>(0.f, 0.f));
                }
                result.reason = fb;
                result.level = SicCancellationLevel::L0_Bypass;
                result.cancelled_fields = SicKnownFields::None;
                result.timings.core_total_us = us(t_core_start);
                return true;
            } else {
                // AutoFallback: gracefully degrade to L1 Preamble!
                result.fallback_reason = fb;
                target_level = SicCancellationLevel::L1_Preamble;
                target_fields = SicKnownFields::PreambleSfd;

                mod::HrpModConfig modcfg = cfg.to_hrp_mod_config();
                size_t n_prefix = 0;
                if (mod::modulate_prefix_waveform(modcfg, scratch.modulator,
                                                  scratch.work_waveform.data(),
                                                  scratch.work_waveform.size(),
                                                  n_prefix) && n_prefix > 0) {
                    s_known_ptr = scratch.work_waveform.data();
                    s_known_len = n_prefix;
                }
            }
        }
    }

    // 3. Reconstruction and Subtraction
    bool ok = reconstruct_and_subtract_unified(
        y, n, channel, target_level, target_fields,
        s_known_ptr, s_known_len,
        templates, cfg, scratch, result.out, result.i_hat, result);

    if (!ok) {
        result.timings.core_total_us = us(t_core_start);
        return true;
    }

    // 4. Calculate actual suppression dB in subtraction ROI: 10 * log10(||y||^2 / ||out||^2)
    if (result.committed && result.sub_hi > result.sub_lo &&
        result.sub_lo >= 0 && result.sub_hi <= static_cast<int64_t>(n)) {
        double e_y = 0.0;
        double e_out = 0.0;
        for (int64_t i = result.sub_lo; i < result.sub_hi; ++i) {
            e_y += std::norm(y[i]);
            e_out += std::norm(result.out[i]);
        }
        if (e_y > 0.0 && e_out > 0.0) {
            result.suppression_db = 10.0 * std::log10(e_y / e_out);
        }
    }

    result.timings.core_total_us = us(t_core_start);
    return true;
}

} // namespace sic
} // namespace uwb
} // namespace gr
