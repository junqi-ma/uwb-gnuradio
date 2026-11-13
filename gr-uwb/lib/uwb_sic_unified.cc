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

#if defined(__x86_64__) || defined(_M_X64)
#include <immintrin.h>
#endif

namespace gr {
namespace uwb {
namespace sic {

namespace {

using Clock = std::chrono::steady_clock;

double us(Clock::time_point t)
{
    return std::chrono::duration<double, std::micro>(Clock::now() - t).count();
}

constexpr uint64_t kFnvOffset = 14695981039346656037ull;
constexpr uint64_t kFnvPrime  = 1099511628211ull;

inline void hash_u64(uint64_t& h, uint64_t v)
{
    h = (h ^ v) * kFnvPrime;
}

inline void hash_bytes(uint64_t& h, const void* data, size_t n)
{
    const auto* p = static_cast<const uint8_t*>(data);
    for (size_t i = 0; i < n; ++i) {
        h = (h ^ static_cast<uint64_t>(p[i])) * kFnvPrime;
    }
}

inline void hash_words64(uint64_t& h, const void* data, size_t n)
{
    const auto* p64 = static_cast<const uint64_t*>(data);
    const size_t n64 = n / 8;
    for (size_t i = 0; i < n64; ++i) {
        h = (h ^ p64[i]) * kFnvPrime;
    }
    const auto* p8 = static_cast<const uint8_t*>(data) + n64 * 8;
    for (size_t i = 0; i < (n & 7); ++i) {
        h = (h ^ static_cast<uint64_t>(p8[i])) * kFnvPrime;
    }
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
    c.coarse_stride = (coarse_stride == 0)
                          ? 0
                          : ((coarse_stride > 1) ? coarse_stride
                                                 : (full_window_search ? 8 : coarse_stride));
    c.detect_threshold = detect_threshold;
    c.fit_residual_max = fit_residual_max;
    c.gain_condition_max = gain_condition_max;
    c.autocorr_prefilter = autocorr_prefilter;
    c.autocorr_threshold = autocorr_threshold;
    c.autocorr_search_inside_target = autocorr_search_inside_target;
    c.target_preamble_lo = target_preamble_lo;
    c.target_preamble_hi = target_preamble_hi;
    c.sfd_mode = (sfd_mode && sfd_mode[0]) ? sfd_mode : "4z2";
    c.pulse = pulse;
    c.max_window_samples = max_window_samples;
    c.max_packet_samples = max_packet_samples;
    return c;
}

mod::HrpModConfig SicUnifiedConfig::to_hrp_mod_config() const
{
    mod::HrpModConfig c;
    c.code_index = comm_code;
    c.sync_repetitions = sync_reps;
    c.sfd_mode = (sfd_mode && sfd_mode[0]) ? sfd_mode : "4z2";
    c.peak_amplitude = 0.8f;
    c.pulse = pulse;
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
// Hash Identity Computations
// ---------------------------------------------------------------------------

uint64_t compute_unified_config_hash(const SicUnifiedConfig& cfg)
{
    uint64_t h = kFnvOffset;
    hash_u64(h, 0x534943554e494631ull); // "SICUNIF1"
    hash_u64(h, static_cast<uint64_t>(cfg.comm_code));
    hash_u64(h, static_cast<uint64_t>(cfg.sync_reps));
    hash_u64(h, static_cast<uint64_t>(cfg.samples_per_symbol));
    uint64_t fs_bits = 0;
    std::memcpy(&fs_bits, &cfg.sample_rate, sizeof(double));
    hash_u64(h, fs_bits);
    const char* eff_sfd = (cfg.sfd_mode && cfg.sfd_mode[0]) ? cfg.sfd_mode : "4z2";
    hash_bytes(h, eff_sfd, std::strlen(eff_sfd));
    hash_u64(h, static_cast<uint64_t>(cfg.pulse.shape));
    uint32_t f_bits = 0;
    std::memcpy(&f_bits, &cfg.pulse.gaussian_sigma_ns, sizeof(float));
    hash_u64(h, static_cast<uint64_t>(f_bits));
    std::memcpy(&f_bits, &cfg.pulse.blackman_bw_mhz, sizeof(float));
    hash_u64(h, static_cast<uint64_t>(f_bits));
    hash_u64(h, static_cast<uint64_t>(cfg.pulse.external_n_taps));
    hash_u64(h, static_cast<uint64_t>(cfg.pulse.external_center));
    hash_u64(h, static_cast<uint64_t>(cfg.backend));
    hash_u64(h, cfg.sparse_pulse_reconstruct ? 1ull : 0ull);
    hash_u64(h, static_cast<uint64_t>(cfg.fit_reps));
    hash_u64(h, static_cast<uint64_t>(cfg.lag_lo));
    hash_u64(h, static_cast<uint64_t>(cfg.tap_count));
    hash_u64(h, static_cast<uint64_t>(cfg.k_paths));
    hash_u64(h, static_cast<uint64_t>(cfg.post_tail));
    hash_u64(h, cfg.full_window_search ? 1ull : 0ull);
    hash_u64(h, static_cast<uint64_t>(cfg.coarse_stride));
    hash_u64(h, static_cast<uint64_t>(cfg.comm_timing_search_margin));
    hash_u64(h, static_cast<uint64_t>(cfg.comm_timing_coarse_stride));
    hash_u64(h, cfg.autocorr_prefilter ? 1ull : 0ull);
    uint64_t ac_bits = 0;
    std::memcpy(&ac_bits, &cfg.autocorr_threshold, sizeof(double));
    hash_u64(h, ac_bits);
    return h;
}

uint64_t compute_unified_template_hash(const SicUnifiedTemplates& tmpl)
{
    uint64_t h = kFnvOffset;
    hash_u64(h, 0x54454d504c415431ull); // "TEMPLAT1"
    hash_u64(h, static_cast<uint64_t>(tmpl.sync_len));
    if (tmpl.sync && tmpl.sync_len > 0) {
        hash_words64(h, tmpl.sync, tmpl.sync_len * sizeof(std::complex<float>));
    }
    hash_u64(h, static_cast<uint64_t>(tmpl.preamble_len));
    hash_u64(h, reinterpret_cast<uintptr_t>(tmpl.preamble));
    hash_u64(h, static_cast<uint64_t>(tmpl.impulse_sync_len));
    if (tmpl.impulse_sync && tmpl.impulse_sync_len > 0) {
        hash_words64(h, tmpl.impulse_sync, tmpl.impulse_sync_len * sizeof(std::complex<float>));
    }
    return h;
}

// ---------------------------------------------------------------------------
// SicUnifiedScratch Reservation & Cache Management
// ---------------------------------------------------------------------------

void SicUnifiedScratch::reserve(size_t max_window_samples, size_t max_packet_samples)
{
    if (max_packet_samples == 0) {
        max_packet_samples = max_window_samples;
    }
    if (max_window_samples > capacity_window_samples) {
        capacity_window_samples = max_window_samples;
    }
    if (max_packet_samples > capacity_packet_samples) {
        capacity_packet_samples = max_packet_samples;
    }

    discrete.reserve(capacity_window_samples);
    fullpacket.reserve(capacity_window_samples);
    modulator.reserve(127, capacity_packet_samples);
    demod.reserve(capacity_window_samples);

    if (work_waveform.capacity() < capacity_packet_samples) {
        work_waveform.reserve(capacity_packet_samples);
    }
    work_waveform.resize(capacity_packet_samples);

    if (work_accum.capacity() < capacity_window_samples) {
        work_accum.reserve(capacity_window_samples);
    }
    work_accum.resize(capacity_window_samples);

    if (work_rot.capacity() < capacity_window_samples) {
        work_rot.reserve(capacity_window_samples);
    }
    work_rot.resize(capacity_window_samples);

    if (work_phr_bits.capacity() < 19) {
        work_phr_bits.reserve(19);
    }
    work_phr_bits.resize(19);

    if (work_wire_psdu.capacity() < radar_meta::kMaxPsduBytes) {
        work_wire_psdu.reserve(radar_meta::kMaxPsduBytes);
    }
    work_wire_psdu.resize(radar_meta::kMaxPsduBytes);
}

bool SicUnifiedScratch::cache_matches(const SicUnifiedConfig& cfg,
                                      const SicUnifiedTemplates& tmpl,
                                      bool force_full_hash) const
{
    if (!prepared || !preamble_cache_valid) {
        return false;
    }
    // The config identity is cheap (a few dozen 64-bit mixes) and is NEVER
    // weakened: it covers every field that changes the cached preamble.
    const uint64_t ch = compute_unified_config_hash(cfg);
    if (config_hash != ch || preamble_cache_config_hash != ch) {
        return false;
    }

    // Template identity.  The full content digest (~8 KB of SYNC samples per
    // window) is only recomputed when the (pointer, length) tuple changes or
    // the caller bumped template_epoch.  Residual aliasing risk: identical
    // pointer + identical length + mutated content is NOT detectable here; such
    // a caller must call invalidate_template_identity().
    const bool tuple_same =
        template_identity.valid && template_identity.epoch == template_epoch &&
        template_identity.sync == static_cast<const void*>(tmpl.sync) &&
        template_identity.sync_len == tmpl.sync_len &&
        template_identity.preamble == static_cast<const void*>(tmpl.preamble) &&
        template_identity.preamble_len == tmpl.preamble_len &&
        template_identity.impulse_sync == static_cast<const void*>(tmpl.impulse_sync) &&
        template_identity.impulse_sync_len == tmpl.impulse_sync_len;

    if (!force_full_hash && tuple_same) {
        return template_hash == template_identity.digest &&
               preamble_cache_template_hash == template_identity.digest;
    }

    const uint64_t th = compute_unified_template_hash(tmpl);
    template_identity.valid = true;
    template_identity.epoch = template_epoch;
    template_identity.sync = tmpl.sync;
    template_identity.sync_len = tmpl.sync_len;
    template_identity.preamble = tmpl.preamble;
    template_identity.preamble_len = tmpl.preamble_len;
    template_identity.impulse_sync = tmpl.impulse_sync;
    template_identity.impulse_sync_len = tmpl.impulse_sync_len;
    template_identity.digest = th;
    return template_hash == th && preamble_cache_template_hash == th;
}

void SicUnifiedScratch::invalidate_cache()
{
    preamble_cache_valid = false;
    preamble_cache_config_hash = 0;
    preamble_cache_template_hash = 0;
    cached_preamble_samples = 0;
    cached_sync_vec.clear();
    cached_preamble_waveform.clear();
    cached_preamble_events.clear();
    cached_preamble_unit_waveform.clear();
    config_hash = 0;
    template_hash = 0;
    prepared = false;
    ++template_epoch;
    template_identity.valid = false;
}

void SicUnifiedScratch::invalidate_template_identity()
{
    ++template_epoch;
    template_identity.valid = false;
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
    scratch.invalidate_cache();

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

    // 1. Cached sync_vec for receiver demod
    scratch.cached_sync_vec.assign(templates.sync, templates.sync + templates.sync_len);

    // 2. Materialized prefix complex waveform (SYNC + SFD)
    mod::HrpModConfig modcfg = cfg.to_hrp_mod_config();
    if (scratch.cached_preamble_waveform.capacity() < scratch.capacity_packet_samples) {
        scratch.cached_preamble_waveform.reserve(scratch.capacity_packet_samples);
    }
    scratch.cached_preamble_waveform.resize(scratch.capacity_packet_samples);
    size_t n_prefix = 0;
    if (mod::modulate_prefix_waveform(modcfg, scratch.modulator,
                                      scratch.cached_preamble_waveform.data(),
                                      scratch.cached_preamble_waveform.size(),
                                      n_prefix) && n_prefix > 0) {
        scratch.cached_preamble_waveform.resize(n_prefix);
    } else {
        scratch.cached_preamble_waveform.clear();
    }

    // 3. Dedicated cached preamble pulse events for sparse pulse reconstruction
    mod::HrpModConfig imp_cfg = modcfg;
    imp_cfg.peak_amplitude = 1.f;
    imp_cfg.pulse.shape = mod::PulseShape::External;
    imp_cfg.external_taps = {1.f, 0.f};
    imp_cfg.pulse.external_n_taps = 2;

    const mod::HrpModConfig& prof = (cfg.backend == SicEstimatorBackend::Ipatov &&
                                     scratch.discrete.impulse_profile.code_index != 0)
        ? scratch.discrete.impulse_profile
        : imp_cfg;
    auto& mod_scratch = (cfg.backend == SicEstimatorBackend::Ipatov)
        ? scratch.discrete.modulator
        : scratch.modulator;

    mod::PulseList pulses;
    if (mod::modulate_prefix_pulse_list(prof, mod_scratch,
                                        scratch.capacity_packet_samples, pulses) && pulses.count > 0) {
        scratch.cached_preamble_events.assign(pulses.events, pulses.events + pulses.count);
        scratch.cached_preamble_samples = pulses.packet_samples;
    } else {
        scratch.cached_preamble_events.clear();
        scratch.cached_preamble_samples = 0;
    }

    // 4. Ipatov unit-impulse prefix waveform (SYNC + SFD)
    // The Ipatov model is fitted against the unit impulse reference and its
    // taps already contain the pulse shaping, so the L1 reconstruction input
    // is the unit impulse prefix, not the shaped one.  Materializing it here
    // keeps the hot path allocation-free and lets the non-sparse apply path
    // run the same transaction as the shaped discrete backend.
    scratch.cached_preamble_unit_waveform.clear();
    if (cfg.backend == SicEstimatorBackend::Ipatov &&
        scratch.cached_preamble_samples > 0 &&
        !scratch.cached_preamble_events.empty()) {
        scratch.cached_preamble_unit_waveform.assign(
            scratch.cached_preamble_samples, std::complex<float>(0.f, 0.f));
        bool ok = true;
        for (const mod::PulseEvent& e : scratch.cached_preamble_events) {
            if (e.sample_offset >= scratch.cached_preamble_samples) {
                ok = false;
                break;
            }
            scratch.cached_preamble_unit_waveform[e.sample_offset] =
                std::complex<float>(static_cast<float>(e.sign), 0.f);
        }
        if (!ok)
            scratch.cached_preamble_unit_waveform.clear();
    }

    scratch.config_hash = compute_unified_config_hash(cfg);
    scratch.template_hash = compute_unified_template_hash(templates);
    scratch.preamble_cache_config_hash = scratch.config_hash;
    scratch.preamble_cache_template_hash = scratch.template_hash;
    // Seed the cheap template identity cache so the first window after prepare
    // does not have to re-hash the SYNC content.
    scratch.template_identity.valid = true;
    scratch.template_identity.epoch = scratch.template_epoch;
    scratch.template_identity.sync = templates.sync;
    scratch.template_identity.sync_len = templates.sync_len;
    scratch.template_identity.preamble = templates.preamble;
    scratch.template_identity.preamble_len = templates.preamble_len;
    scratch.template_identity.impulse_sync = templates.impulse_sync;
    scratch.template_identity.impulse_sync_len = templates.impulse_sync_len;
    scratch.template_identity.digest = scratch.template_hash;
    scratch.preamble_cache_valid = true;
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
        // copy_input_on_bypass == false leaves `out` untouched: the caller reads
        // its own input because `committed` stays false.
        if (cfg.copy_input_on_bypass && out && y && n > 0 && out != y) {
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
    const int64_t eff_sub_lo = (cfg.sub_roi_lo == 0 && cfg.sub_roi_hi == 0)
                                   ? 0
                                   : cfg.sub_roi_lo;
    const int64_t eff_sub_hi = (cfg.sub_roi_lo == 0 && cfg.sub_roi_hi == 0)
                                   ? static_cast<int64_t>(n)
                                   : cfg.sub_roi_hi;

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
                eff_sub_lo, eff_sub_hi, scratch.fullpacket, out, i_hat);
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
            // Check if sparse pulse reconstruction is enabled for Ipatov backend
            if (dm.impulse_template && scratch.discrete.sparse_pulse_reconstruct) {
                mod::PulseList pulses;
                if (scratch.preamble_cache_valid && !scratch.cached_preamble_events.empty()) {
                    pulses.events = scratch.cached_preamble_events.data();
                    pulses.count = scratch.cached_preamble_events.size();
                    pulses.packet_samples = scratch.cached_preamble_samples;
                } else {
                    const bool tx_ok = mod::modulate_prefix_pulse_list(
                        scratch.discrete.impulse_profile, scratch.discrete.modulator,
                        scratch.discrete.waveform.size(), pulses);
                    if (!tx_ok || pulses.count == 0) {
                        pulses = mod::PulseList{};
                    }
                }
                if (pulses.count > 0) {
                    bool ok = apply_discrete_cir_pulses(
                        y, n, pulses, templates.sync, templates.sync_len, dm,
                        eff_sub_lo, eff_sub_hi, scratch.discrete, out, i_hat, &dres);
                    result.timings.reconstruct_us = dres.channel_us;
                    result.timings.subtract_us = dres.subtract_us;
                    result.timings.recon_scale_us = dres.scale_us;
                    result.timings.recon_conv_us = dres.convolution_us;
                    result.timings.recon_cfo_rot_us = dres.phase_us + dres.cfo_us;
                    result.waveform_scale = dres.waveform_scale;
                    if (!ok || !dres.committed) {
                        return fail_closed(dres.reason ? dres.reason : "apply_discrete_pulses_failed");
                    }
                    result.committed = true;
                    result.reason = "committed_preamble_sparse_pulse";
                    result.level = SicCancellationLevel::L1_Preamble;
                    result.cancelled_fields = known_fields;
                    result.support_lo = dres.support_lo;
                    result.support_hi = dres.support_hi;
                    result.sub_lo = dres.sub_lo;
                    result.sub_hi = dres.sub_hi;
                    result.sub_overlap = (result.sub_hi > result.sub_lo) ? (result.sub_hi - result.sub_lo) : 0;
                    return true;
                }
            }

            // Ipatov: the reconstruction input must be the UNIT impulse prefix
            // (the taps already contain the shaping).  Using the shaped prefix
            // here would fail the waveform-scale consistency check against the
            // unit reference SYNC, so it is never a valid fallback.
            const std::complex<float>* wf = s_known ? s_known : templates.preamble;
            size_t wf_n = s_known ? s_known_len : templates.preamble_len;
            const char* commit_reason = "committed_preamble_discrete";
            if (dm.impulse_template) {
                if (scratch.cached_preamble_unit_waveform.empty()) {
                    return fail_closed("impulse_prefix_unavailable");
                }
                wf = scratch.cached_preamble_unit_waveform.data();
                wf_n = scratch.cached_preamble_unit_waveform.size();
                commit_reason = "committed_preamble_ipatov";
            }

            bool ok = apply_discrete_cir(
                y, n, wf, wf_n, templates.sync, templates.sync_len, dm,
                eff_sub_lo, eff_sub_hi, scratch.discrete, out, i_hat, &dres);
            result.timings.reconstruct_us = dres.channel_us;
            result.timings.subtract_us = dres.subtract_us;
            result.timings.recon_scale_us = dres.scale_us;
            result.timings.recon_conv_us = dres.convolution_us;
            result.timings.recon_cfo_rot_us = dres.phase_us + dres.cfo_us;
            result.waveform_scale = dres.waveform_scale;
            if (!ok || !dres.committed) {
                return fail_closed(dres.reason ? dres.reason : "apply_discrete_failed");
            }
            result.committed = true;
            result.reason = commit_reason;
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
                eff_sub_lo, eff_sub_hi, scratch.discrete, out, i_hat);
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
// compute_roi_energies (SIMD float load + double accumulation)
// ---------------------------------------------------------------------------

void compute_roi_energies(
    const std::complex<float>* y,
    const std::complex<float>* out,
    int64_t start,
    int64_t end,
    double& e_y,
    double& e_out)
{
    e_y = 0.0;
    e_out = 0.0;
    if (!y || !out || end <= start || start < 0) {
        return;
    }
    const size_t count = static_cast<size_t>(end - start);

#if defined(__AVX2__)
    const float* py = reinterpret_cast<const float*>(y + start);
    const float* pout = reinterpret_cast<const float*>(out + start);

    __m256d y_acc0 = _mm256_setzero_pd();
    __m256d y_acc1 = _mm256_setzero_pd();
    __m256d y_acc2 = _mm256_setzero_pd();
    __m256d y_acc3 = _mm256_setzero_pd();

    __m256d out_acc0 = _mm256_setzero_pd();
    __m256d out_acc1 = _mm256_setzero_pd();
    __m256d out_acc2 = _mm256_setzero_pd();
    __m256d out_acc3 = _mm256_setzero_pd();

    size_t i = 0;
    const size_t unroll8 = count & ~size_t(7);
    for (; i < unroll8; i += 8) {
        __m256 vy0 = _mm256_loadu_ps(py + 2 * i);
        __m256 vy1 = _mm256_loadu_ps(py + 2 * i + 8);
        __m256 vout0 = _mm256_loadu_ps(pout + 2 * i);
        __m256 vout1 = _mm256_loadu_ps(pout + 2 * i + 8);

        __m256 vy0_sq = _mm256_mul_ps(vy0, vy0);
        __m256 vy1_sq = _mm256_mul_ps(vy1, vy1);
        __m256 vout0_sq = _mm256_mul_ps(vout0, vout0);
        __m256 vout1_sq = _mm256_mul_ps(vout1, vout1);

        y_acc0 = _mm256_add_pd(y_acc0, _mm256_cvtps_pd(_mm256_castps256_ps128(vy0_sq)));
        y_acc1 = _mm256_add_pd(y_acc1, _mm256_cvtps_pd(_mm256_extractf128_ps(vy0_sq, 1)));
        y_acc2 = _mm256_add_pd(y_acc2, _mm256_cvtps_pd(_mm256_castps256_ps128(vy1_sq)));
        y_acc3 = _mm256_add_pd(y_acc3, _mm256_cvtps_pd(_mm256_extractf128_ps(vy1_sq, 1)));

        out_acc0 = _mm256_add_pd(out_acc0, _mm256_cvtps_pd(_mm256_castps256_ps128(vout0_sq)));
        out_acc1 = _mm256_add_pd(out_acc1, _mm256_cvtps_pd(_mm256_extractf128_ps(vout0_sq, 1)));
        out_acc2 = _mm256_add_pd(out_acc2, _mm256_cvtps_pd(_mm256_castps256_ps128(vout1_sq)));
        out_acc3 = _mm256_add_pd(out_acc3, _mm256_cvtps_pd(_mm256_extractf128_ps(vout1_sq, 1)));
    }

    if (i + 4 <= count) {
        __m256 vy0 = _mm256_loadu_ps(py + 2 * i);
        __m256 vout0 = _mm256_loadu_ps(pout + 2 * i);

        __m256 vy0_sq = _mm256_mul_ps(vy0, vy0);
        __m256 vout0_sq = _mm256_mul_ps(vout0, vout0);

        y_acc0 = _mm256_add_pd(y_acc0, _mm256_cvtps_pd(_mm256_castps256_ps128(vy0_sq)));
        y_acc1 = _mm256_add_pd(y_acc1, _mm256_cvtps_pd(_mm256_extractf128_ps(vy0_sq, 1)));

        out_acc0 = _mm256_add_pd(out_acc0, _mm256_cvtps_pd(_mm256_castps256_ps128(vout0_sq)));
        out_acc1 = _mm256_add_pd(out_acc1, _mm256_cvtps_pd(_mm256_extractf128_ps(vout0_sq, 1)));
        i += 4;
    }

    __m256d y_sum = _mm256_add_pd(_mm256_add_pd(y_acc0, y_acc1),
                                  _mm256_add_pd(y_acc2, y_acc3));
    alignas(32) double y_buf[4];
    _mm256_storeu_pd(y_buf, y_sum);
    double sum_y = y_buf[0] + y_buf[1] + y_buf[2] + y_buf[3];

    __m256d out_sum = _mm256_add_pd(_mm256_add_pd(out_acc0, out_acc1),
                                    _mm256_add_pd(out_acc2, out_acc3));
    alignas(32) double out_buf[4];
    _mm256_storeu_pd(out_buf, out_sum);
    double sum_out = out_buf[0] + out_buf[1] + out_buf[2] + out_buf[3];

    for (; i < count; ++i) {
        sum_y += std::norm(y[start + i]);
        sum_out += std::norm(out[start + i]);
    }
    e_y = sum_y;
    e_out = sum_out;
#else
    double sum_y = 0.0;
    double sum_out = 0.0;
    for (size_t i = 0; i < count; ++i) {
        sum_y += std::norm(y[start + i]);
        sum_out += std::norm(out[start + i]);
    }
    e_y = sum_y;
    e_out = sum_out;
#endif
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

    // L0 Bypass / fail-closed output handling.  `result.committed` is already
    // false here, which is what tells the caller to read its own input.
    //
    // cfg.copy_input_on_bypass == false (UHD SC16 pipeline worker) skips the
    // O(n) std::copy: on a 100% clean window that copy alone is ~122 us and the
    // i_hat zero-fill another ~116 us, both pure waste because `out` is never
    // read unless `committed` is true.
    auto bypass_outputs = [&]() {
        if (cfg.copy_input_on_bypass && result.out && y && n > 0 && result.out != y) {
            std::copy(y, y + n, result.out);
        }
        if (result.i_hat) {
            std::fill(result.i_hat, result.i_hat + n, std::complex<float>(0.f, 0.f));
        }
    };

    if (!y || !result.out || n == 0 || !scratch.prepared ||
        !scratch.cache_matches(cfg, templates)) {
        result.reason = (!y || !result.out || n == 0 || !scratch.prepared)
                            ? "invalid_buffers_or_scratch"
                            : "config_or_template_mismatch";
        bypass_outputs();
        return false;
    }

    // 1. Strict-prefix Channel Estimation
    SicUnifiedChannel channel;
    const bool est_ok = estimate_channel_unified(
        y, n, cfg.observed_end_sample, templates, cfg, scratch,
        channel, &result.timings);
    result.channel = channel;

    if (!est_ok || !channel.valid) {
        bypass_outputs();
        result.reason = channel.reason ? channel.reason : channel.status;
        result.timings.core_total_us = us(t_core_start);
        return true;
    }

    // 2. Cancellation Policy & Comm RX Demodulation (SYNC + SFD for both PreambleOnly and FullPacket)
    SicCancellationLevel target_level = SicCancellationLevel::L0_Bypass;
    SicKnownFields target_fields = SicKnownFields::None;
    const std::complex<float>* s_known_ptr = templates.preamble;
    size_t s_known_len = templates.preamble_len;

    const std::vector<std::complex<float>>& sync_vec = scratch.cached_sync_vec;

    auto demod_prof = demod::Qm35825Profile::Default();
    demod_prof.code_index = cfg.comm_code;
    demod_prof.preamble_repetitions = cfg.sync_reps;
    demod_prof.pulse_center_sample = mod::pulse_center_tap(cfg.pulse);
    if (cfg.sync_reps < 64) {
        const size_t skip = std::max<size_t>(1, cfg.sync_reps / 4);
        demod_prof.cir_skip_initial_repetitions =
            std::min<size_t>(demod_prof.cir_skip_initial_repetitions, skip);
        demod_prof.cir_repetitions =
            (cfg.sync_reps > demod_prof.cir_skip_initial_repetitions)
                ? (cfg.sync_reps - demod_prof.cir_skip_initial_repetitions)
                : 1;
        demod_prof.cfo_skip_initial_repetitions =
            std::min<size_t>(demod_prof.cfo_skip_initial_repetitions, skip);
        const size_t avail_cfo =
            (cfg.sync_reps > demod_prof.cfo_skip_initial_repetitions)
                ? (cfg.sync_reps - demod_prof.cfo_skip_initial_repetitions)
                : 1;
        demod_prof.cfo_min_fit_repetitions = std::max<size_t>(
            1, std::min<size_t>(demod_prof.cfo_min_fit_repetitions, avail_cfo));
        demod_prof.min_valid_peaks = std::min<size_t>(
            demod_prof.min_valid_peaks, std::max<size_t>(2, cfg.sync_reps / 2));
        demod_prof.timing_max_backtrack_symbols = std::min<size_t>(
            demod_prof.timing_max_backtrack_symbols, cfg.sync_reps);
    } else if (cfg.sync_reps > demod_prof.cir_skip_initial_repetitions) {
        demod_prof.cir_repetitions = cfg.sync_reps - demod_prof.cir_skip_initial_repetitions;
    }
    demod_prof.timing_search_margin =
        (cfg.full_window_search && cfg.comm_timing_search_margin == 256)
            ? 2048
            : cfg.comm_timing_search_margin;
    demod_prof.timing_coarse_stride =
        (cfg.comm_timing_coarse_stride > 1)
            ? cfg.comm_timing_coarse_stride
            : (cfg.full_window_search ? 8 : cfg.comm_timing_coarse_stride);
    demod_prof.sfd_mode = (cfg.sfd_mode && cfg.sfd_mode[0]) ? cfg.sfd_mode : "4z2";
    demod_prof.rx_fast_sfd = cfg.rx_fast_sfd;
    demod_prof.rx_fast_cfo_rotation = cfg.rx_fast_cfo;
    demod_prof.rx_seed_refine_tail = cfg.rx_seed_refine_tail;
    demod_prof.rx_softchip_on_demand = cfg.rx_softchip_on_demand;
    demod_prof.rx_fused_payload_cmf = cfg.rx_fused_payload_cmf;
    demod_prof.rx_fused_phr_cmf = cfg.rx_fused_phr_cmf;
    demod_prof.timing_seed_refine_radius = 16;

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

            if (scratch.preamble_cache_valid && !scratch.cached_preamble_waveform.empty()) {
                s_known_ptr = scratch.cached_preamble_waveform.data();
                s_known_len = scratch.cached_preamble_waveform.size();
            } else {
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
        } else {
            // SFD detection failed on preamble-only
            bypass_outputs();
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
                bypass_outputs();
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

                if (scratch.preamble_cache_valid && !scratch.cached_preamble_waveform.empty()) {
                    s_known_ptr = scratch.cached_preamble_waveform.data();
                    s_known_len = scratch.cached_preamble_waveform.size();
                } else {
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
        compute_roi_energies(y, result.out, result.sub_lo, result.sub_hi, e_y, e_out);
        if (std::isfinite(e_y) && std::isfinite(e_out) && e_y > 0.0 && e_out > 0.0) {
            result.suppression_db = 10.0 * std::log10(e_y / e_out);
        }
    }

    result.timings.core_total_us = us(t_core_start);
    return true;
}

} // namespace sic
} // namespace uwb
} // namespace gr
