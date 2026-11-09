/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * UwbPreambleSic — thin GNU Radio PDU adapter for the S0 preamble-only SIC
 * math core.  See uwb_preamble_sic.h (scheduler semantics) and
 * docs/sic/S0_sic_adapter_api.md (frozen contract).
 *
 * This file contains NO SIC algorithm: it validates PDUs, applies admission
 * control, runs the core on a single worker, wraps PMTs and counts.  All
 * reconstruction logic lives in uwb_preamble_sic_core.cc.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gnuradio/io_signature.h>
#include <gnuradio/uwb/uwb_preamble_sic.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <stdexcept>
#include <utility>

namespace gr {
namespace uwb {

namespace {

inline uint64_t elapsed_us(std::chrono::steady_clock::time_point t0,
                           std::chrono::steady_clock::time_point t1)
{
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count());
}

struct SicPmtSymbols {
    // Ports
    pmt::pmt_t port_in;
    pmt::pmt_t port_out;
    pmt::pmt_t port_status;
    pmt::pmt_t port_truth;
    pmt::pmt_t port_i_hat;

    // Common keys and values
    pmt::pmt_t key_window_id;
    pmt::pmt_t key_event;
    pmt::pmt_t key_reason;
    pmt::pmt_t key_sample_count;
    pmt::pmt_t key_planned_publish_ns;
    pmt::pmt_t key_sensing_roi_lo;
    pmt::pmt_t key_sensing_roi_hi;
    pmt::pmt_t key_sensing_min_overlap;
    pmt::pmt_t key_observed_end_sample;
    pmt::pmt_t key_visible_end_sample;
    pmt::pmt_t key_start_sample;
    pmt::pmt_t key_cfo_hz;
    pmt::pmt_t key_tau;
    pmt::pmt_t key_gain_re;
    pmt::pmt_t key_gain_im;
    pmt::pmt_t key_n_paths;

    // Status event names & values
    pmt::pmt_t sym_empty;
    pmt::pmt_t sym_none;
    pmt::pmt_t sym_window;
    pmt::pmt_t sym_stopped;
    pmt::pmt_t sym_invalid_input;
    pmt::pmt_t sym_invalid_truth;
    pmt::pmt_t sym_truth_ready;
    pmt::pmt_t sym_queue_overflow;
    pmt::pmt_t sym_bypass_fifo_full;
    pmt::pmt_t sym_in_flight_limit;
    pmt::pmt_t sym_iq_budget;
    pmt::pmt_t sym_queue_full;
    pmt::pmt_t sym_pool_capacity_exceeded;
    pmt::pmt_t sym_sic_estimate_failed;
    pmt::pmt_t sym_sic_one_failed;
    pmt::pmt_t sym_whole_window_default;
    pmt::pmt_t sym_fingerprint_zero;

    // States
    pmt::pmt_t sym_bypassed;
    pmt::pmt_t sym_committed;
    pmt::pmt_t sym_failed;
    pmt::pmt_t sym_ambiguous;

    // SIC output metadata keys
    pmt::pmt_t key_sic_state;
    pmt::pmt_t key_sic_detected;
    pmt::pmt_t key_sic_committed;
    pmt::pmt_t key_sic_partial;
    pmt::pmt_t key_sic_ambiguous;
    pmt::pmt_t key_sic_bypass_reason;
    pmt::pmt_t key_sic_uses_truth;
    pmt::pmt_t key_sic_missed_admission;
    pmt::pmt_t key_sic_detected_start;
    pmt::pmt_t key_sic_cfo_hz;
    pmt::pmt_t key_sic_n_paths;
    pmt::pmt_t key_sic_support_lo;
    pmt::pmt_t key_sic_support_hi;
    pmt::pmt_t key_sic_sub_overlap;
    pmt::pmt_t key_sic_residual_corr;
    pmt::pmt_t key_sic_model_mode;
    pmt::pmt_t key_sic_model_id;
    pmt::pmt_t key_sic_model_fingerprint;
    pmt::pmt_t key_sic_observed_end_sample;
    pmt::pmt_t key_sic_observed_end_source;
    pmt::pmt_t key_sic_visible_end_sample;
    pmt::pmt_t key_sic_training_lo;
    pmt::pmt_t key_sic_training_hi;
    pmt::pmt_t key_sic_estimation_ready_sample;
    pmt::pmt_t key_sic_support_source;
    pmt::pmt_t key_sic_support_coverage;
    pmt::pmt_t key_sic_sub_lo;
    pmt::pmt_t key_sic_sub_hi;
    pmt::pmt_t key_sic_phase_reference;
    pmt::pmt_t key_sic_fit_residual;
    pmt::pmt_t key_sic_cond2;
    pmt::pmt_t key_sic_tau;
    pmt::pmt_t key_sic_gain_re;
    pmt::pmt_t key_sic_gain_im;
    pmt::pmt_t key_sic_path_count;
    pmt::pmt_t key_sic_gain_cond;
    pmt::pmt_t key_sic_apply_refit_count;
    pmt::pmt_t key_sic_apply_param_deltas;
    pmt::pmt_t key_sic_nominal_sync_reps;
    pmt::pmt_t key_sic_samples_per_symbol;
    pmt::pmt_t key_sic_sample_rate;

    // Timer & Diagnostic keys
    pmt::pmt_t key_handler_us;
    pmt::pmt_t key_queue_wait_us;
    pmt::pmt_t key_core_total_us;
    pmt::pmt_t key_sic_detect_coarse_us;
    pmt::pmt_t key_sic_detect_fine_us;
    pmt::pmt_t key_sic_cfo_us;
    pmt::pmt_t key_sic_cir_us;
    pmt::pmt_t key_sic_path_select_us;
    pmt::pmt_t key_sic_fractional_refine_us;
    pmt::pmt_t key_sic_matrix_build_us;
    pmt::pmt_t key_sic_gain_fit_us;
    pmt::pmt_t key_sic_reconstruct_us;
    pmt::pmt_t key_sic_validate_commit_us;
    pmt::pmt_t key_sic_estimate_us;
    pmt::pmt_t key_sic_model_prepare_us;
    pmt::pmt_t key_sic_apply_us;
    pmt::pmt_t key_sic_backfill_us;
    pmt::pmt_t key_sic_apply_forward_us;
    pmt::pmt_t key_sic_refine_init_rot_us;
    pmt::pmt_t key_sic_refine_init_columns_us;
    pmt::pmt_t key_sic_refine_init_qr_us;
    pmt::pmt_t key_sic_refine_resid_us;
    pmt::pmt_t key_sic_refine_derot_us;
    pmt::pmt_t key_sic_refine_candidate_us;
    pmt::pmt_t key_sic_refine_project_us;
    pmt::pmt_t key_sic_refine_self_energy_us;
    pmt::pmt_t key_sic_refine_k;
    pmt::pmt_t key_sic_refine_m;
    pmt::pmt_t key_sic_refine_candidates;
    pmt::pmt_t key_sic_refine_rebuilds;
    pmt::pmt_t key_sic_refine_cache_hits;
    pmt::pmt_t key_sic_refine_cache_misses;
    pmt::pmt_t key_sic_refine_self_energy_hits;
    pmt::pmt_t key_sic_refine_self_energy_misses;
    pmt::pmt_t key_sic_apply_block_builds;
    pmt::pmt_t key_sic_apply_block_hits;
    pmt::pmt_t key_sic_apply_rot_base_hits;
    pmt::pmt_t key_sic_apply_rot_base_misses;
    pmt::pmt_t key_output_wrap_publish_us;
    pmt::pmt_t key_completion_latency_us;
    pmt::pmt_t key_sic_pool_capacity_exceeded;

    // Status counters
    pmt::pmt_t key_windows_offered;
    pmt::pmt_t key_windows_admitted;
    pmt::pmt_t key_windows_completed;
    pmt::pmt_t key_windows_failed;
    pmt::pmt_t key_windows_bypassed;
    pmt::pmt_t key_windows_dropped;
    pmt::pmt_t key_windows_invalid;
    pmt::pmt_t key_windows_missed_admission;
    pmt::pmt_t key_missed_admission;

    // Mode name symbols
    pmt::pmt_t sym_estimate8_frozen;
    pmt::pmt_t sym_confirmed_legacy;
    pmt::pmt_t sym_configured_nominal;

    SicPmtSymbols()
        : port_in(pmt::string_to_symbol("in")),
          port_out(pmt::string_to_symbol("out")),
          port_status(pmt::string_to_symbol("status")),
          port_truth(pmt::string_to_symbol("truth")),
          port_i_hat(pmt::string_to_symbol("i_hat")),
          key_window_id(pmt::string_to_symbol("window_id")),
          key_event(pmt::string_to_symbol("event")),
          key_reason(pmt::string_to_symbol("reason")),
          key_sample_count(pmt::string_to_symbol("sample_count")),
          key_planned_publish_ns(pmt::string_to_symbol("planned_publish_ns")),
          key_sensing_roi_lo(pmt::string_to_symbol("sensing_roi_lo")),
          key_sensing_roi_hi(pmt::string_to_symbol("sensing_roi_hi")),
          key_sensing_min_overlap(pmt::string_to_symbol("sensing_min_overlap")),
          key_observed_end_sample(pmt::string_to_symbol("observed_end_sample")),
          key_visible_end_sample(pmt::string_to_symbol("visible_end_sample")),
          key_start_sample(pmt::string_to_symbol("start_sample")),
          key_cfo_hz(pmt::string_to_symbol("cfo_hz")),
          key_tau(pmt::string_to_symbol("tau")),
          key_gain_re(pmt::string_to_symbol("gain_re")),
          key_gain_im(pmt::string_to_symbol("gain_im")),
          key_n_paths(pmt::string_to_symbol("n_paths")),
          sym_empty(pmt::string_to_symbol("")),
          sym_none(pmt::string_to_symbol("none")),
          sym_window(pmt::string_to_symbol("window")),
          sym_stopped(pmt::string_to_symbol("stopped")),
          sym_invalid_input(pmt::string_to_symbol("invalid_input")),
          sym_invalid_truth(pmt::string_to_symbol("invalid_truth")),
          sym_truth_ready(pmt::string_to_symbol("truth_ready")),
          sym_queue_overflow(pmt::string_to_symbol("queue_overflow")),
          sym_bypass_fifo_full(pmt::string_to_symbol("bypass_fifo_full")),
          sym_in_flight_limit(pmt::string_to_symbol("in_flight_limit")),
          sym_iq_budget(pmt::string_to_symbol("iq_budget")),
          sym_queue_full(pmt::string_to_symbol("queue_full")),
          sym_pool_capacity_exceeded(pmt::string_to_symbol("pool_capacity_exceeded")),
          sym_sic_estimate_failed(pmt::string_to_symbol("sic_estimate_failed")),
          sym_sic_one_failed(pmt::string_to_symbol("sic_one_failed")),
          sym_whole_window_default(pmt::string_to_symbol("whole_window_default")),
          sym_fingerprint_zero(pmt::string_to_symbol("0x0000000000000000")),
          sym_bypassed(pmt::string_to_symbol("bypassed")),
          sym_committed(pmt::string_to_symbol("committed")),
          sym_failed(pmt::string_to_symbol("failed")),
          sym_ambiguous(pmt::string_to_symbol("ambiguous")),
          key_sic_state(pmt::string_to_symbol("sic_state")),
          key_sic_detected(pmt::string_to_symbol("sic_detected")),
          key_sic_committed(pmt::string_to_symbol("sic_committed")),
          key_sic_partial(pmt::string_to_symbol("sic_partial")),
          key_sic_ambiguous(pmt::string_to_symbol("sic_ambiguous")),
          key_sic_bypass_reason(pmt::string_to_symbol("sic_bypass_reason")),
          key_sic_uses_truth(pmt::string_to_symbol("sic_uses_truth")),
          key_sic_missed_admission(pmt::string_to_symbol("sic_missed_admission")),
          key_sic_detected_start(pmt::string_to_symbol("sic_detected_start")),
          key_sic_cfo_hz(pmt::string_to_symbol("sic_cfo_hz")),
          key_sic_n_paths(pmt::string_to_symbol("sic_n_paths")),
          key_sic_support_lo(pmt::string_to_symbol("sic_support_lo")),
          key_sic_support_hi(pmt::string_to_symbol("sic_support_hi")),
          key_sic_sub_overlap(pmt::string_to_symbol("sic_sub_overlap")),
          key_sic_residual_corr(pmt::string_to_symbol("sic_residual_corr")),
          key_sic_model_mode(pmt::string_to_symbol("sic_model_mode")),
          key_sic_model_id(pmt::string_to_symbol("sic_model_id")),
          key_sic_model_fingerprint(pmt::string_to_symbol("sic_model_fingerprint")),
          key_sic_observed_end_sample(pmt::string_to_symbol("sic_observed_end_sample")),
          key_sic_observed_end_source(pmt::string_to_symbol("sic_observed_end_source")),
          key_sic_visible_end_sample(pmt::string_to_symbol("sic_visible_end_sample")),
          key_sic_training_lo(pmt::string_to_symbol("sic_training_lo")),
          key_sic_training_hi(pmt::string_to_symbol("sic_training_hi")),
          key_sic_estimation_ready_sample(pmt::string_to_symbol("sic_estimation_ready_sample")),
          key_sic_support_source(pmt::string_to_symbol("sic_support_source")),
          key_sic_support_coverage(pmt::string_to_symbol("sic_support_coverage")),
          key_sic_sub_lo(pmt::string_to_symbol("sic_sub_lo")),
          key_sic_sub_hi(pmt::string_to_symbol("sic_sub_hi")),
          key_sic_phase_reference(pmt::string_to_symbol("sic_phase_reference")),
          key_sic_fit_residual(pmt::string_to_symbol("sic_fit_residual")),
          key_sic_cond2(pmt::string_to_symbol("sic_cond2")),
          key_sic_tau(pmt::string_to_symbol("sic_tau")),
          key_sic_gain_re(pmt::string_to_symbol("sic_gain_re")),
          key_sic_gain_im(pmt::string_to_symbol("sic_gain_im")),
          key_sic_path_count(pmt::string_to_symbol("sic_path_count")),
          key_sic_gain_cond(pmt::string_to_symbol("sic_gain_cond")),
          key_sic_apply_refit_count(pmt::string_to_symbol("sic_apply_refit_count")),
          key_sic_apply_param_deltas(pmt::string_to_symbol("sic_apply_param_deltas")),
          key_sic_nominal_sync_reps(pmt::string_to_symbol("sic_nominal_sync_reps")),
          key_sic_samples_per_symbol(pmt::string_to_symbol("sic_samples_per_symbol")),
          key_sic_sample_rate(pmt::string_to_symbol("sic_sample_rate")),
          key_handler_us(pmt::string_to_symbol("handler_us")),
          key_queue_wait_us(pmt::string_to_symbol("queue_wait_us")),
          key_core_total_us(pmt::string_to_symbol("core_total_us")),
          key_sic_detect_coarse_us(pmt::string_to_symbol("sic_detect_coarse_us")),
          key_sic_detect_fine_us(pmt::string_to_symbol("sic_detect_fine_us")),
          key_sic_cfo_us(pmt::string_to_symbol("sic_cfo_us")),
          key_sic_cir_us(pmt::string_to_symbol("sic_cir_us")),
          key_sic_path_select_us(pmt::string_to_symbol("sic_path_select_us")),
          key_sic_fractional_refine_us(pmt::string_to_symbol("sic_fractional_refine_us")),
          key_sic_matrix_build_us(pmt::string_to_symbol("sic_matrix_build_us")),
          key_sic_gain_fit_us(pmt::string_to_symbol("sic_gain_fit_us")),
          key_sic_reconstruct_us(pmt::string_to_symbol("sic_reconstruct_us")),
          key_sic_validate_commit_us(pmt::string_to_symbol("sic_validate_commit_us")),
          key_sic_estimate_us(pmt::string_to_symbol("sic_estimate_us")),
          key_sic_model_prepare_us(pmt::string_to_symbol("sic_model_prepare_us")),
          key_sic_apply_us(pmt::string_to_symbol("sic_apply_us")),
          key_sic_backfill_us(pmt::string_to_symbol("sic_backfill_us")),
          key_sic_apply_forward_us(pmt::string_to_symbol("sic_apply_forward_us")),
          key_sic_refine_init_rot_us(pmt::string_to_symbol("sic_refine_init_rot_us")),
          key_sic_refine_init_columns_us(pmt::string_to_symbol("sic_refine_init_columns_us")),
          key_sic_refine_init_qr_us(pmt::string_to_symbol("sic_refine_init_qr_us")),
          key_sic_refine_resid_us(pmt::string_to_symbol("sic_refine_resid_us")),
          key_sic_refine_derot_us(pmt::string_to_symbol("sic_refine_derot_us")),
          key_sic_refine_candidate_us(pmt::string_to_symbol("sic_refine_candidate_us")),
          key_sic_refine_project_us(pmt::string_to_symbol("sic_refine_project_us")),
          key_sic_refine_self_energy_us(pmt::string_to_symbol("sic_refine_self_energy_us")),
          key_sic_refine_k(pmt::string_to_symbol("sic_refine_k")),
          key_sic_refine_m(pmt::string_to_symbol("sic_refine_m")),
          key_sic_refine_candidates(pmt::string_to_symbol("sic_refine_candidates")),
          key_sic_refine_rebuilds(pmt::string_to_symbol("sic_refine_rebuilds")),
          key_sic_refine_cache_hits(pmt::string_to_symbol("sic_refine_cache_hits")),
          key_sic_refine_cache_misses(pmt::string_to_symbol("sic_refine_cache_misses")),
          key_sic_refine_self_energy_hits(pmt::string_to_symbol("sic_refine_self_energy_hits")),
          key_sic_refine_self_energy_misses(pmt::string_to_symbol("sic_refine_self_energy_misses")),
          key_sic_apply_block_builds(pmt::string_to_symbol("sic_apply_block_builds")),
          key_sic_apply_block_hits(pmt::string_to_symbol("sic_apply_block_hits")),
          key_sic_apply_rot_base_hits(pmt::string_to_symbol("sic_apply_rot_base_hits")),
          key_sic_apply_rot_base_misses(pmt::string_to_symbol("sic_apply_rot_base_misses")),
          key_output_wrap_publish_us(pmt::string_to_symbol("output_wrap_publish_us")),
          key_completion_latency_us(pmt::string_to_symbol("completion_latency_us")),
          key_sic_pool_capacity_exceeded(pmt::string_to_symbol("sic_pool_capacity_exceeded")),
          key_windows_offered(pmt::string_to_symbol("windows_offered")),
          key_windows_admitted(pmt::string_to_symbol("windows_admitted")),
          key_windows_completed(pmt::string_to_symbol("windows_completed")),
          key_windows_failed(pmt::string_to_symbol("windows_failed")),
          key_windows_bypassed(pmt::string_to_symbol("windows_bypassed")),
          key_windows_dropped(pmt::string_to_symbol("windows_dropped")),
          key_windows_invalid(pmt::string_to_symbol("windows_invalid")),
          key_windows_missed_admission(pmt::string_to_symbol("windows_missed_admission")),
          key_missed_admission(pmt::string_to_symbol("missed_admission")),
          sym_estimate8_frozen(pmt::string_to_symbol("estimate8_frozen")),
          sym_confirmed_legacy(pmt::string_to_symbol("confirmed_legacy")),
          sym_configured_nominal(pmt::string_to_symbol("configured_nominal"))
    {
    }

    static const SicPmtSymbols& get()
    {
        static const SicPmtSymbols instance;
        return instance;
    }

    pmt::pmt_t event_symbol(const std::string& event) const
    {
        if (event == "window") return sym_window;
        if (event == "stopped") return sym_stopped;
        if (event == "invalid_input") return sym_invalid_input;
        if (event == "invalid_truth") return sym_invalid_truth;
        if (event == "truth_ready") return sym_truth_ready;
        if (event == "queue_overflow") return sym_queue_overflow;
        return pmt::string_to_symbol(event);
    }

    pmt::pmt_t reason_symbol(const char* reason) const
    {
        if (reason == nullptr || *reason == '\0') return sym_empty;
        if (std::strcmp(reason, "queue_overflow") == 0) return sym_queue_overflow;
        if (std::strcmp(reason, "bypass_fifo_full") == 0) return sym_bypass_fifo_full;
        if (std::strcmp(reason, "in_flight_limit") == 0) return sym_in_flight_limit;
        if (std::strcmp(reason, "iq_budget") == 0) return sym_iq_budget;
        if (std::strcmp(reason, "queue_full") == 0) return sym_queue_full;
        if (std::strcmp(reason, "pool_capacity_exceeded") == 0) return sym_pool_capacity_exceeded;
        if (std::strcmp(reason, "sic_estimate_failed") == 0) return sym_sic_estimate_failed;
        if (std::strcmp(reason, "sic_one_failed") == 0) return sym_sic_one_failed;
        return pmt::string_to_symbol(reason);
    }

    pmt::pmt_t state_symbol(sic::SicState s) const
    {
        switch (s) {
        case sic::SicState::Bypassed: return sym_bypassed;
        case sic::SicState::Committed: return sym_committed;
        case sic::SicState::Failed: return sym_failed;
        case sic::SicState::Ambiguous: return sym_ambiguous;
        }
        return sym_failed;
    }

    pmt::pmt_t mode_symbol(const std::string& mode) const
    {
        if (mode == "estimate8_frozen") return sym_estimate8_frozen;
        if (mode == "confirmed_legacy") return sym_confirmed_legacy;
        return pmt::string_to_symbol(mode);
    }

    pmt::pmt_t support_source_symbol(const char* s) const
    {
        if (s == nullptr || *s == '\0' || std::strcmp(s, "none") == 0) return sym_none;
        if (std::strcmp(s, "configured_nominal") == 0) return sym_configured_nominal;
        if (std::strcmp(s, "confirmed_legacy") == 0) return sym_confirmed_legacy;
        return pmt::string_to_symbol(s);
    }

    pmt::pmt_t observed_end_source_symbol(const char* s) const
    {
        if (s == nullptr || *s == '\0') return sym_empty;
        if (std::strcmp(s, "whole_window_default") == 0) return sym_whole_window_default;
        if (std::strcmp(s, "observed_end_sample") == 0) return key_observed_end_sample;
        if (std::strcmp(s, "visible_end_sample") == 0) return key_visible_end_sample;
        return pmt::string_to_symbol(s);
    }
};

int64_t dict_i64(pmt::pmt_t dict, const pmt::pmt_t& key, int64_t def)
{
    if (!pmt::is_dict(dict))
        return def;
    pmt::pmt_t v = pmt::dict_ref(dict, key, pmt::from_long(def));
    if (pmt::is_uint64(v))
        return static_cast<int64_t>(pmt::to_uint64(v));
    if (pmt::is_integer(v))
        return static_cast<int64_t>(pmt::to_long(v));
    return def;
}

uint64_t dict_u64(pmt::pmt_t dict, const pmt::pmt_t& key, uint64_t def)
{
    if (!pmt::is_dict(dict))
        return def;
    pmt::pmt_t v = pmt::dict_ref(dict, key, pmt::from_uint64(def));
    if (pmt::is_uint64(v))
        return pmt::to_uint64(v);
    if (pmt::is_integer(v))
        return static_cast<uint64_t>(pmt::to_long(v));
    return def;
}

double dict_f64(pmt::pmt_t dict, const pmt::pmt_t& key, double def)
{
    if (!pmt::is_dict(dict))
        return def;
    pmt::pmt_t v = pmt::dict_ref(dict, key, pmt::from_double(def));
    if (pmt::is_real(v) || pmt::is_integer(v) || pmt::is_uint64(v))
        return pmt::to_double(v);
    return def;
}

// S1 cancellation-support mode label -> core enumerator.  Explicit: an unknown
// string is rejected (fail-closed), never silently mapped to a default.
bool parse_support_mode(const std::string& s, sic::SicSupportMode& out)
{
    if (s == "configured_nominal") {
        out = sic::SicSupportMode::ConfiguredNominal;
        return true;
    }
    if (s == "confirmed_legacy") {
        out = sic::SicSupportMode::ConfirmedLegacy;
        return true;
    }
    return false;
}

// The per-window estimation visibility limit declared in the PDU meta:
// `observed_end_sample` (preferred) or `visible_end_sample`.  Absent => 0, which
// the core reads as "the whole window is visible" (legacy-visible contract).
// A declared prefix is passed through unchanged and never widened.  `source`
// records which input produced the value (adapter meta key, or the default).
int64_t declared_observed_end(pmt::pmt_t meta, const char** source)
{
    const auto& sym = SicPmtSymbols::get();
    if (pmt::dict_has_key(meta, sym.key_observed_end_sample)) {
        *source = "observed_end_sample";
        return dict_i64(meta, sym.key_observed_end_sample, 0);
    }
    if (pmt::dict_has_key(meta, sym.key_visible_end_sample)) {
        *source = "visible_end_sample";
        return dict_i64(meta, sym.key_visible_end_sample, 0);
    }
    *source = "whole_window_default";
    return 0;
}

// Lower-case hex of a 64-bit model fingerprint ("0x%016llx").
std::string hex_u64(uint64_t v)
{
    char buf[19];
    std::snprintf(buf, sizeof(buf), "0x%016llx",
                  static_cast<unsigned long long>(v));
    return std::string(buf);
}

} // namespace

// ---------------------------------------------------------------------------
// Construction / factory
// ---------------------------------------------------------------------------
std::vector<gr_complex>
UwbPreambleSic::load_cf32_file(const std::string& path)
{
    std::ifstream f(path, std::ios::binary);
    if (!f) {
        throw std::invalid_argument(
            "UwbPreambleSic: cannot open template file " + path);
    }
    f.seekg(0, std::ios::end);
    const std::streamoff bytes = f.tellg();
    f.seekg(0, std::ios::beg);
    if (bytes <= 0 ||
        bytes % static_cast<std::streamoff>(sizeof(gr_complex)) != 0) {
        throw std::invalid_argument(
            "UwbPreambleSic: template file empty or invalid size: " + path);
    }
    std::vector<gr_complex> tmpl(static_cast<size_t>(bytes) /
                                 sizeof(gr_complex));
    f.read(reinterpret_cast<char*>(tmpl.data()), bytes);
    if (!f && !f.eof()) {
        throw std::invalid_argument(
            "UwbPreambleSic: failed reading template file " + path);
    }
    if (tmpl.empty()) {
        throw std::invalid_argument(
            "UwbPreambleSic: template file is empty: " + path);
    }
    return tmpl;
}

bool UwbPreambleSic::parse_mode(const std::string& mode, sic::SicMode& out)
{
    if (mode == "off") {
        out = sic::SicMode::Off;
        return true;
    }
    if (mode == "estimate") {
        out = sic::SicMode::Estimate;
        return true;
    }
    if (mode == "oracle") {
        out = sic::SicMode::Oracle;
        return true;
    }
    return false;
}

UwbPreambleSic::UwbPreambleSic(const std::vector<gr_complex>& sync,
                               const std::vector<gr_complex>& preamble,
                               const std::string& mode_name,
                               size_t comm_code,
                               size_t sync_reps,
                               size_t samples_per_symbol,
                               double sample_rate,
                               size_t k_paths,
                               size_t fit_reps,
                               size_t validation_reps,
                               int64_t roi_lo,
                               int64_t roi_hi,
                               size_t coarse_stride,
                               double detect_threshold,
                               double validate_threshold,
                               size_t in_flight_max,
                               size_t iq_budget_bytes,
                               size_t queue_capacity,
                               bool publish_reconstruction,
                               bool enable_fractional,
                               bool enable_cfo,
                               size_t path_min_spacing,
                               size_t post_tail,
                               size_t max_window_samples,
                               size_t pending_bypass_max,
                               const std::string& support_mode,
                               size_t online_validation_reps)
    : gr::block("uwb_preamble_sic",
                gr::io_signature::make(0, 0, 0),
                gr::io_signature::make(0, 0, 0)),
      d_sync_(sync),
      d_preamble_(preamble),
      d_mode_name_(mode_name),
      d_in_flight_max_(in_flight_max),
      d_iq_budget_bytes_(iq_budget_bytes),
      d_queue_capacity_(queue_capacity),
      d_publish_reconstruction_(publish_reconstruction),
      d_max_window_samples_(max_window_samples)
{
    if (d_sync_.empty()) {
        throw std::invalid_argument("UwbPreambleSic: sync template is empty");
    }
    if (d_preamble_.empty()) {
        throw std::invalid_argument(
            "UwbPreambleSic: preamble template is empty");
    }
    if (d_in_flight_max_ == 0) {
        throw std::invalid_argument(
            "UwbPreambleSic: in_flight_max must be > 0");
    }
    if (d_iq_budget_bytes_ == 0) {
        throw std::invalid_argument(
            "UwbPreambleSic: iq_budget_bytes must be > 0");
    }
    if (d_queue_capacity_ == 0) {
        throw std::invalid_argument(
            "UwbPreambleSic: queue_capacity must be > 0");
    }
    // Default bound on the pending-bypass FIFO: one in-flight round of work
    // plus the admitted work queue.  0 => computed default.
    d_pending_bypass_max_ = (pending_bypass_max > 0)
        ? pending_bypass_max
        : (d_in_flight_max_ + d_queue_capacity_);

    sic::SicMode m;
    if (!parse_mode(mode_name, m)) {
        throw std::invalid_argument("UwbPreambleSic: unknown mode '" +
                                    mode_name + "' (off|estimate|oracle)");
    }
    d_truth_oracle_ = (m == sic::SicMode::Oracle);
    d_cfg_.mode = m;
    d_cfg_.comm_code = comm_code;
    d_cfg_.sync_reps = sync_reps;
    d_cfg_.samples_per_symbol = samples_per_symbol;
    d_cfg_.sample_rate = sample_rate;
    d_cfg_.k_paths = k_paths;
    d_cfg_.fit_reps = fit_reps;
    d_cfg_.validation_reps = validation_reps;
    d_cfg_.roi_lo = roi_lo;
    d_cfg_.roi_hi = roi_hi;
    d_cfg_.coarse_stride = coarse_stride;
    d_cfg_.detect_threshold = detect_threshold;
    d_cfg_.validate_threshold = validate_threshold;
    d_cfg_.enable_fractional = enable_fractional;
    d_cfg_.enable_cfo = enable_cfo;
    d_cfg_.path_min_spacing = path_min_spacing;
    d_cfg_.post_tail = post_tail;

    // S1 support/visibility plumbing.  support_mode is validated explicitly (an
    // unknown label is a hard construction error, not a silent default).
    sic::SicSupportMode sm;
    if (!parse_support_mode(support_mode, sm)) {
        throw std::invalid_argument(
            "UwbPreambleSic: unknown support_mode '" + support_mode +
            "' (configured_nominal|confirmed_legacy)");
    }
    d_cfg_.support_mode = sm;
    d_support_mode_name_ = (sm == sic::SicSupportMode::ConfiguredNominal)
        ? "configured_nominal"
        : "confirmed_legacy";
    d_model_mode_name_ = (sm == sic::SicSupportMode::ConfiguredNominal)
        ? "estimate8_frozen"
        : "confirmed_legacy";
    // The explicit holdout is bounded by the PREPARED holdout capacity; reject
    // an impossible request at construction instead of failing every window.
    if (online_validation_reps > validation_reps) {
        throw std::invalid_argument(
            "UwbPreambleSic: online_validation_reps must be <= validation_reps");
    }
    d_cfg_.online_validation_reps = online_validation_reps;

    // Pre-size the output pool when the caller declared a maximum window.
    if (d_max_window_samples_ > 0) {
        d_out_.assign(d_max_window_samples_, gr_complex(0.0f, 0.0f));
        if (d_publish_reconstruction_)
            d_ihat_.assign(d_max_window_samples_, gr_complex(0.0f, 0.0f));
        d_pool_capacity_ = d_max_window_samples_;
    }

    std::string err;
    if (!sic::prepare_sic_core(d_cfg_,
                               d_sync_.data(),
                               d_sync_.size(),
                               d_preamble_.data(),
                               d_preamble_.size(),
                               d_scratch_,
                               &err)) {
        throw std::invalid_argument("UwbPreambleSic: " + err);
    }
    // Estimate mode is the only mode usable for effect/real-time acceptance; it
    // needs fixed, non-zero thresholds or the core fails closed at run time.
    if (m == sic::SicMode::Estimate &&
        (!(detect_threshold > 0.0) || !(validate_threshold > 0.0))) {
        throw std::invalid_argument(
            "UwbPreambleSic: estimate mode requires non-zero detect/validate "
            "thresholds");
    }

    const auto& sym = SicPmtSymbols::get();
    message_port_register_in(sym.port_in);
    // The truth port is oracle-only: in estimate/off it does not exist and no
    // truth object is ever stored on the block.
    if (d_truth_oracle_) {
        message_port_register_in(sym.port_truth);
        set_msg_handler(sym.port_truth,
                        [this](pmt::pmt_t msg) { handle_truth(msg); });
    }
    message_port_register_out(sym.port_out);
    message_port_register_out(sym.port_status);
    if (d_publish_reconstruction_)
        message_port_register_out(sym.port_i_hat);

    set_msg_handler(sym.port_in,
                    [this](pmt::pmt_t msg) { handle_in(msg); });
}

UwbPreambleSic::~UwbPreambleSic()
{
    {
        std::lock_guard<std::mutex> lock(d_queue_mutex_);
        d_stop_ = true;
    }
    d_queue_cv_.notify_all();
    if (d_worker_.joinable())
        d_worker_.join();
}

UwbPreambleSic::sptr
UwbPreambleSic::make(const std::string& sync_template_path,
                     const std::string& preamble_template_path,
                     size_t comm_code,
                     size_t sync_reps,
                     size_t samples_per_symbol,
                     double sample_rate,
                     size_t k_paths,
                     size_t fit_reps,
                     size_t validation_reps,
                     int64_t roi_lo,
                     int64_t roi_hi,
                     size_t coarse_stride,
                     const std::string& mode,
                     double detect_threshold,
                     double validate_threshold,
                     size_t in_flight_max,
                     size_t iq_budget_bytes,
                     size_t queue_capacity,
                     bool publish_reconstruction,
                     bool enable_fractional,
                     bool enable_cfo,
                     size_t path_min_spacing,
                     size_t post_tail,
                     size_t max_window_samples,
                     size_t pending_bypass_max,
                     const std::string& support_mode,
                     size_t online_validation_reps)
{
    std::vector<gr_complex> sync = load_cf32_file(sync_template_path);
    std::vector<gr_complex> preamble = load_cf32_file(preamble_template_path);
    return make_from_templates(sync,
                               preamble,
                               comm_code,
                               sync_reps,
                               samples_per_symbol,
                               sample_rate,
                               k_paths,
                               fit_reps,
                               validation_reps,
                               roi_lo,
                               roi_hi,
                               coarse_stride,
                               mode,
                               detect_threshold,
                               validate_threshold,
                               in_flight_max,
                               iq_budget_bytes,
                               queue_capacity,
                               publish_reconstruction,
                               enable_fractional,
                               enable_cfo,
                               path_min_spacing,
                               post_tail,
                               max_window_samples,
                               pending_bypass_max,
                               support_mode,
                               online_validation_reps);
}

UwbPreambleSic::sptr
UwbPreambleSic::make_from_templates(const std::vector<gr_complex>& sync,
                                    const std::vector<gr_complex>& preamble,
                                    size_t comm_code,
                                    size_t sync_reps,
                                    size_t samples_per_symbol,
                                    double sample_rate,
                                    size_t k_paths,
                                    size_t fit_reps,
                                    size_t validation_reps,
                                    int64_t roi_lo,
                                    int64_t roi_hi,
                                    size_t coarse_stride,
                                    const std::string& mode,
                                    double detect_threshold,
                                    double validate_threshold,
                                    size_t in_flight_max,
                                    size_t iq_budget_bytes,
                                    size_t queue_capacity,
                                    bool publish_reconstruction,
                                    bool enable_fractional,
                                    bool enable_cfo,
                                    size_t path_min_spacing,
                                    size_t post_tail,
                                    size_t max_window_samples,
                                    size_t pending_bypass_max,
                                    const std::string& support_mode,
                                    size_t online_validation_reps)
{
    return gnuradio::get_initial_sptr(new UwbPreambleSic(sync,
                                                         preamble,
                                                         mode,
                                                         comm_code,
                                                         sync_reps,
                                                         samples_per_symbol,
                                                         sample_rate,
                                                         k_paths,
                                                         fit_reps,
                                                         validation_reps,
                                                         roi_lo,
                                                         roi_hi,
                                                         coarse_stride,
                                                         detect_threshold,
                                                         validate_threshold,
                                                         in_flight_max,
                                                         iq_budget_bytes,
                                                         queue_capacity,
                                                         publish_reconstruction,
                                                         enable_fractional,
                                                         enable_cfo,
                                                         path_min_spacing,
                                                         post_tail,
                                                         max_window_samples,
                                                         pending_bypass_max,
                                                         support_mode,
                                                         online_validation_reps));
}

// ---------------------------------------------------------------------------
// Lifecycle / stats
// ---------------------------------------------------------------------------

size_t
UwbPreambleSic::queue_depth() const
{
    std::lock_guard<std::mutex> lock(d_queue_mutex_);
    return d_queue_.size();
}

bool
UwbPreambleSic::drained() const
{
    std::lock_guard<std::mutex> lock(d_queue_mutex_);
    return d_queue_.empty() && d_worker_idle_;
}

void
UwbPreambleSic::drain()
{
    std::unique_lock<std::mutex> lock(d_queue_mutex_);
    d_queue_cv_.wait(
        lock, [this] { return d_queue_.empty() && d_worker_idle_; });
}

bool
UwbPreambleSic::start()
{
    // Join any leftover worker without holding the queue mutex (join under the
    // lock would deadlock a worker blocked on d_queue_mutex_).
    {
        std::lock_guard<std::mutex> lock(d_queue_mutex_);
        if (d_worker_.joinable())
            d_stop_ = true;
    }
    d_queue_cv_.notify_all();
    if (d_worker_.joinable())
        d_worker_.join();
    {
        std::lock_guard<std::mutex> lock(d_queue_mutex_);
        d_stop_ = false;
        d_worker_idle_ = true;
    }
    d_worker_ = std::thread(&UwbPreambleSic::worker_loop, this);
    return true;
}

bool
UwbPreambleSic::stop()
{
    {
        std::lock_guard<std::mutex> lock(d_queue_mutex_);
        d_stop_ = true;
    }
    d_queue_cv_.notify_all();
    if (d_worker_.joinable())
        d_worker_.join();
    publish_status("stopped");
    return true;
}

// ---------------------------------------------------------------------------
// Message handlers
// ---------------------------------------------------------------------------

void
UwbPreambleSic::handle_in(pmt::pmt_t msg)
{
    const auto t0 = std::chrono::steady_clock::now();

    // ---- validate --------------------------------------------------------
    if (!pmt::is_pair(msg)) {
        d_invalid_.fetch_add(1, std::memory_order_relaxed);
        publish_status("invalid_input");
        return;
    }
    pmt::pmt_t meta_in = pmt::car(msg);
    pmt::pmt_t data = pmt::cdr(msg);
    if (!pmt::is_dict(meta_in))
        meta_in = pmt::make_dict();
    if (!pmt::is_c32vector(data)) {
        d_invalid_.fetch_add(1, std::memory_order_relaxed);
        publish_status("invalid_input");
        return;
    }
    const size_t n = pmt::length(data);
    if (n == 0) {
        d_invalid_.fetch_add(1, std::memory_order_relaxed);
        publish_status("invalid_input");
        return;
    }
    const auto& sym = SicPmtSymbols::get();
    // "wrong length": a declared sample_count must agree with the vector.
    if (pmt::dict_has_key(meta_in, sym.key_sample_count)) {
        const int64_t sc = dict_i64(meta_in, sym.key_sample_count, -1);
        if (sc < 0 || static_cast<uint64_t>(sc) != static_cast<uint64_t>(n)) {
            d_invalid_.fetch_add(1, std::memory_order_relaxed);
            publish_status("invalid_input");
            return;
        }
    }
    size_t nelem = 0;
    const gr_complex* y = pmt::c32vector_elements(data, nelem);
    if (y == nullptr || nelem != n) {
        d_invalid_.fetch_add(1, std::memory_order_relaxed);
        publish_status("invalid_input");
        return;
    }

    Pending p;
    p.meta_in = meta_in;
    p.iq_in = data;
    p.n = n;
    p.bytes = n * sizeof(gr_complex);
    if (pmt::dict_has_key(meta_in, sym.key_planned_publish_ns)) {
        const int64_t plan_ns = dict_i64(meta_in, sym.key_planned_publish_ns, 0);
        p.offered_at = (plan_ns > 0)
            ? std::chrono::steady_clock::time_point(
                  std::chrono::nanoseconds(plan_ns))
            : t0;
    } else {
        p.offered_at = t0;
    }

    d_offered_.fetch_add(1, std::memory_order_relaxed);

    // ---- admission -------------------------------------------------------
    const char* reject_reason = nullptr;
    bool overflow = false;
    {
        std::lock_guard<std::mutex> lock(d_queue_mutex_);
        const bool room_inflight = d_in_flight_ < d_in_flight_max_;
        const bool room_bytes =
            (d_pending_bytes_ + p.bytes) <= d_iq_budget_bytes_;
        const bool room_queue = d_queue_admitted_ < d_queue_capacity_;
        // "Stop admitting" once the ordered-bypass FIFO is at its cap.
        const bool room_bypass = d_pending_bypass_ < d_pending_bypass_max_;
        if (room_inflight && room_bytes && room_queue && room_bypass) {
            p.admitted = true;
            d_admitted_.fetch_add(1, std::memory_order_relaxed);
            ++d_in_flight_;
            ++d_queue_admitted_;
            d_pending_bytes_ += p.bytes;
            d_inflight_hwm_ = std::max(d_inflight_hwm_, d_in_flight_);
        } else {
            p.admitted = false;
            if (!room_inflight)
                reject_reason = "in_flight_limit";
            else if (!room_bytes)
                reject_reason = "iq_budget";
            else if (!room_queue)
                reject_reason = "queue_full";
            else
                reject_reason = "bypass_fifo_full";
            p.bypass_reason = reject_reason;
            d_missed_admission_.fetch_add(1, std::memory_order_relaxed);
            // Honest bounded bypass FIFO: if parking this ordered bypass would
            // exceed the cap, terminate it as queue_overflow instead (recorded,
            // never a silent drop).  Upstream credit keeps this off the normal
            // path (offered == admitted + missed_admission).
            if (d_pending_bypass_ >= d_pending_bypass_max_) {
                overflow = true;
            } else {
                ++d_pending_bypass_;
                if (d_pending_bypass_ > d_bypass_fifo_hwm_)
                    d_bypass_fifo_hwm_ = d_pending_bypass_;
            }
        }
        if (!overflow) {
            // Enqueue in offered order.  Bypass entries carry no scratch and
            // reuse the original IQ; this is what keeps `out` ordered.
            p.enqueued_at = std::chrono::steady_clock::now();
            p.handler_us = elapsed_us(t0, p.enqueued_at);
            d_queue_.push_back(std::move(p));
            if (d_queue_.size() > d_queue_hwm_)
                d_queue_hwm_ = d_queue_.size();
        }
    }
    if (overflow) {
        d_queue_overflow_.fetch_add(1, std::memory_order_relaxed);
        d_dropped_.fetch_add(1, std::memory_order_relaxed);
        pmt::pmt_t extra = pmt::make_dict();
        extra = pmt::dcons(
            pmt::cons(sym.key_window_id,
                      pmt::from_uint64(dict_u64(meta_in, sym.key_window_id, 0))),
            extra);
        extra = pmt::dcons(pmt::cons(sym.key_sic_state, sym.sym_queue_overflow), extra);
        extra = pmt::dcons(pmt::cons(sym.key_reason, sym.sym_bypass_fifo_full), extra);
        extra = pmt::dcons(
            pmt::cons(sym.key_handler_us,
                      pmt::from_uint64(elapsed_us(t0, std::chrono::steady_clock::now()))),
            extra);
        publish_status("queue_overflow", extra);
        return;
    }
    d_queue_cv_.notify_one();
}

void
UwbPreambleSic::handle_truth(pmt::pmt_t msg)
{
    const auto& sym = SicPmtSymbols::get();
    // Oracle-only diagnostic sidecar.  Never consulted in Estimate mode.
    if (!pmt::is_dict(msg) || !pmt::dict_has_key(msg, sym.key_window_id)) {
        publish_status("invalid_truth");
        return;
    }
    sic::SicTruth t;
    t.valid = true;
    t.start_sample = dict_f64(msg, sym.key_start_sample, 0.0);
    t.cfo_hz = dict_f64(msg, sym.key_cfo_hz, 0.0);

    size_t nt = 0;
    const double* tau = nullptr;
    if (pmt::dict_has_key(msg, sym.key_tau)) {
        pmt::pmt_t v = pmt::dict_ref(msg, sym.key_tau, pmt::PMT_NIL);
        if (pmt::is_f64vector(v))
            tau = pmt::f64vector_elements(v, nt);
    }
    size_t ng = 0, ni = 0;
    const double* gr = nullptr;
    const double* gi = nullptr;
    if (pmt::dict_has_key(msg, sym.key_gain_re)) {
        pmt::pmt_t v = pmt::dict_ref(msg, sym.key_gain_re, pmt::PMT_NIL);
        if (pmt::is_f64vector(v))
            gr = pmt::f64vector_elements(v, ng);
    }
    if (pmt::dict_has_key(msg, sym.key_gain_im)) {
        pmt::pmt_t v = pmt::dict_ref(msg, sym.key_gain_im, pmt::PMT_NIL);
        if (pmt::is_f64vector(v))
            gi = pmt::f64vector_elements(v, ni);
    }
    const size_t avail =
        std::min({ nt, ng, ni, static_cast<size_t>(sic::kSicMaxPaths) });
    size_t np = static_cast<size_t>(
        std::max<int64_t>(0, dict_i64(msg, sym.key_n_paths, (int64_t)avail)));
    np = std::min(np, avail);
    t.n_paths = np;
    for (size_t k = 0; k < np; ++k) {
        t.tau[k] = (tau != nullptr) ? tau[k] : 0.0;
        const double re = (gr != nullptr) ? gr[k] : 0.0;
        const double im = (gi != nullptr) ? gi[k] : 0.0;
        t.gain[k] = std::complex<float>(static_cast<float>(re),
                                        static_cast<float>(im));
    }

    const uint64_t wid = dict_u64(msg, sym.key_window_id, 0);
    {
        std::lock_guard<std::mutex> lock(d_truth_mutex_);
        d_truth_[wid] = t;
    }
    // Deterministic "stored" acknowledgement.  GNU Radio keeps a SEPARATE queue
    // per input port (basic_block::msg_queue_map_t), so posting `truth` and then
    // `in` gives no cross-port ordering guarantee.  A producer that needs the
    // sidecar to be visible before the window can wait for this event.
    pmt::pmt_t extra = pmt::make_dict();
    extra = pmt::dcons(pmt::cons(sym.key_window_id, pmt::from_uint64(wid)), extra);
    publish_status("truth_ready", extra);
}

// ---------------------------------------------------------------------------
// Worker
// ---------------------------------------------------------------------------

void
UwbPreambleSic::worker_loop()
{
    for (;;) {
        Pending p;
        {
            std::unique_lock<std::mutex> lock(d_queue_mutex_);
            d_worker_idle_ = true;
            d_queue_cv_.notify_all();
            d_queue_cv_.wait(lock, [this] {
                return d_stop_ || !d_queue_.empty();
            });
            if (d_queue_.empty() && d_stop_)
                return;
            d_worker_idle_ = false;
            p = std::move(d_queue_.front());
            d_queue_.pop_front();
            if (p.admitted && d_queue_admitted_ > 0)
                --d_queue_admitted_;
            if (!p.admitted && d_pending_bypass_ > 0)
                --d_pending_bypass_;
        }
        process(p);
        d_queue_cv_.notify_all();
    }
}

bool
UwbPreambleSic::ensure_pool(size_t n)
{
    if (n <= d_pool_capacity_)
        return true;
    // Unsized (max_window_samples == 0): the first window sizes the pool.  That
    // initial sizing is not a "grow" event.
    if (d_pool_capacity_ == 0) {
        d_pool_capacity_ = n;
        d_out_.assign(n, gr_complex(0.0f, 0.0f));
        if (d_publish_reconstruction_)
            d_ihat_.assign(n, gr_complex(0.0f, 0.0f));
        return true;
    }
    // One-time grow only, counted.  After that, a larger window is fail-closed
    // (never grow per window on the hot path).
    if (d_window_grow_count_.load(std::memory_order_relaxed) == 0) {
        d_pool_capacity_ = n;
        d_out_.assign(n, gr_complex(0.0f, 0.0f));
        if (d_publish_reconstruction_)
            d_ihat_.assign(n, gr_complex(0.0f, 0.0f));
        d_window_grow_count_.fetch_add(1, std::memory_order_relaxed);
        return true;
    }
    return false;
}

const char*
UwbPreambleSic::state_name(sic::SicState s)
{
    switch (s) {
    case sic::SicState::Bypassed:
        return "bypassed";
    case sic::SicState::Committed:
        return "committed";
    case sic::SicState::Failed:
        return "failed";
    case sic::SicState::Ambiguous:
        return "ambiguous";
    }
    return "failed";
}

void
UwbPreambleSic::process(Pending& p)
{
    const auto& sym = SicPmtSymbols::get();
    const auto t_worker0 = std::chrono::steady_clock::now();
    const uint64_t queue_wait_us = elapsed_us(p.enqueued_at, t_worker0);

    // Size/hold the output pool.  Sized from max_window_samples at
    // construction, else from the first window; at most ONE counted grow, then
    // a larger window is fail-closed (never per-window growth on the hot path).
    const bool pool_ok = ensure_pool(p.n);
    bool pool_exceeded = false;

    sic::SicResult r;
    sic::SicModel model;      // frozen S1 model (estimate mode only)
    bool have_model = false;  // model struct filled by estimate_model
    bool sic_ran = false;
    bool failed = false;
    const char* reason = p.bypass_reason;
    // Declared estimation visibility limit (0 = whole window) and the meta key
    // it came from.  Recorded verbatim; never widened to the whole window.
    int64_t observed_end_declared = 0;
    const char* observed_end_source = "whole_window_default";
    if (p.admitted && !pool_ok) {
        pool_exceeded = true;
        d_pool_capacity_exceeded_.fetch_add(1, std::memory_order_relaxed);
        reason = "pool_capacity_exceeded";
    }

    if (p.admitted && pool_ok) {
        r.out = d_out_.data();
        r.i_hat = d_publish_reconstruction_ ? d_ihat_.data() : nullptr;

        sic::SicTemplates tpl;
        tpl.sync = d_sync_.data();
        tpl.sync_len = d_sync_.size();
        tpl.preamble = d_preamble_.data();
        tpl.preamble_len = d_preamble_.size();

        // R10: the SENSING processing ROI (the subtraction / commit region) is a
        // sensing property carried in the input PDU meta and kept SEPARATE from
        // the wide candidate search ROI in the block config.  It gates the commit
        // so a real comm detected outside the sensing preamble/CIR window is
        // bypassed instead of committed (out == y).  Absent keys leave the gate
        // disabled (legacy behaviour: subtraction = support ∩ search ROI).
        sic::SicConfig cfg = d_cfg_;
        if (pmt::dict_has_key(p.meta_in, sym.key_sensing_roi_lo) ||
            pmt::dict_has_key(p.meta_in, sym.key_sensing_roi_hi)) {
            cfg.sub_roi_lo = dict_i64(p.meta_in, sym.key_sensing_roi_lo, 0);
            cfg.sub_roi_hi = dict_i64(p.meta_in, sym.key_sensing_roi_hi, 0);
            cfg.sub_min_overlap =
                dict_u64(p.meta_in, sym.key_sensing_min_overlap, d_cfg_.sub_min_overlap);
        }
        // S1: per-window estimation visibility.  `observed_end_sample` (or the
        // `visible_end_sample` alias) is taken from the meta when the caller
        // declares a prefix; absent => 0 = whole window (legacy-visible).
        observed_end_declared = declared_observed_end(p.meta_in, &observed_end_source);
        cfg.observed_end_sample = observed_end_declared;

        size_t nelem = 0;
        const gr_complex* y = pmt::c32vector_elements(p.iq_in, nelem);
        const size_t n = nelem;

        if (d_cfg_.mode == sic::SicMode::Estimate) {
            // S1: estimate a FROZEN 8-SYNC model over the declared prefix, then
            // apply it to the WHOLE window (backfill the observed prefix, then
            // clean forward) -- the same composition sic_one uses, but with the
            // explicit model in hand so its frozen fields can be published.
            const auto t_core0 = std::chrono::steady_clock::now();
            // Byte-exact bypass default: `out` starts as a copy of `y`, the
            // diagnostic reconstruction as zeros.  Every failure/bypass path
            // leaves this untouched (out == y bit-for-bit).
            std::copy(y, y + n, d_out_.data());
            if (r.i_hat)
                std::fill(d_ihat_.begin(),
                          d_ihat_.begin() + static_cast<std::ptrdiff_t>(n),
                          gr_complex(0.0f, 0.0f));

            const auto t_est0 = std::chrono::steady_clock::now();
            bool ok = false;
            try {
                ok = sic::estimate_model(y, n, observed_end_declared, tpl, cfg,
                                         d_scratch_, model, &r.timings, &r);
            } catch (...) {
                ok = false;
            }
            r.timings.estimate_total_us =
                elapsed_us(t_est0, std::chrono::steady_clock::now());
            sic_ran = true;
            have_model = true;
            if (!ok) {
                failed = true;
                reason = (r.reject_reason != nullptr)
                    ? r.reject_reason
                    : (model.reason != nullptr ? model.reason
                                               : "sic_estimate_failed");
            } else {
                // model.state is the authoritative terminal estimation state
                // (estimate_model does not back-fill diag on its early returns).
                r.state = model.state;
                reason = (r.bypass_reason != nullptr) ? r.bypass_reason
                                                      : model.reason;
                if (model.valid) {
                    const int64_t visible_end = model.observed_end_sample;
                    const size_t backfill_end = static_cast<size_t>(
                        std::max<int64_t>(0,
                                          std::min<int64_t>(visible_end,
                                                            static_cast<int64_t>(n))));
                    const auto t_bf0 = std::chrono::steady_clock::now();
                    if (backfill_end > 0)
                        sic::apply_model(model, y, backfill_end, 0, tpl, cfg,
                                         d_scratch_, d_out_.data(), r.i_hat,
                                         &r.timings, true);
                    r.timings.backfill_us =
                        elapsed_us(t_bf0, std::chrono::steady_clock::now());
                    const auto t_fw0 = std::chrono::steady_clock::now();
                    const size_t fwd0 = backfill_end;
                    const size_t fwd = n - fwd0;
                    if (fwd > 0)
                        sic::apply_model(model, y + fwd0, fwd,
                                         static_cast<int64_t>(fwd0), tpl, cfg,
                                         d_scratch_, d_out_.data() + fwd0,
                                         r.i_hat ? r.i_hat + fwd0 : nullptr,
                                         &r.timings, true);
                    r.timings.apply_forward_us =
                        elapsed_us(t_fw0, std::chrono::steady_clock::now());
                }
            }
            // Outer total over the estimate+apply composition (single authority,
            // exactly the role core_total_us plays inside sic_one).
            r.timings.core_total_us =
                elapsed_us(t_core0, std::chrono::steady_clock::now());
        } else {
            // off / oracle: the core's single-entry sic_one path is unchanged
            // (it owns the out-copy and core_total_us; oracle consumes an
            // explicit, window-matched truth struct; estimate never reaches this
            // branch and reads no truth).
            sic::SicTruth truth;
            const sic::SicTruth* truth_ptr = nullptr;
            if (d_cfg_.mode == sic::SicMode::Oracle) {
                const uint64_t wid = dict_u64(p.meta_in, sym.key_window_id, 0);
                std::lock_guard<std::mutex> lock(d_truth_mutex_);
                auto it = d_truth_.find(wid);
                if (it != d_truth_.end()) {
                    truth = it->second;
                    truth_ptr = &truth;
                }
            }
            bool ok = false;
            try {
                ok = sic::sic_one(y, n, tpl, cfg, d_scratch_, r, truth_ptr);
            } catch (...) {
                ok = false;
            }
            sic_ran = true;
            if (!ok) {
                failed = true;
                reason = (r.reject_reason != nullptr) ? r.reject_reason
                                                      : "sic_one_failed";
            } else {
                reason = r.bypass_reason;
            }
        }
    }

    const bool committed = p.admitted && sic_ran && !failed && r.committed;
    const bool uses_truth = p.admitted && sic_ran && !failed && r.uses_truth;

    // Diagnostic reconstruction is exactly zero on every bypass path.  It can
    // only be materialised when the pool holds the window (a window larger than
    // the one-time-grown pool is fail-closed and skips the diagnostic i_hat).
    const bool can_publish_ihat =
        d_publish_reconstruction_ && (committed || pool_ok);
    if (can_publish_ihat && !committed) {
        std::fill(d_ihat_.begin(),
                  d_ihat_.begin() + static_cast<std::ptrdiff_t>(p.n),
                  gr_complex(0.0f, 0.0f));
    }

    // ---- wrap + publish ---------------------------------------------------
    const auto t_wrap0 = std::chrono::steady_clock::now();
    pmt::pmt_t meta = pmt::is_dict(p.meta_in) ? p.meta_in : pmt::make_dict();
    auto put_u64 = [&meta](const pmt::pmt_t& k, uint64_t v) {
        meta = pmt::dcons(pmt::cons(k, pmt::from_uint64(v)), meta);
    };
    auto put_i64 = [&meta](const pmt::pmt_t& k, int64_t v) {
        meta = pmt::dcons(pmt::cons(k, pmt::from_long(v)), meta);
    };
    auto put_f64 = [&meta](const pmt::pmt_t& k, double v) {
        meta = pmt::dcons(pmt::cons(k, pmt::from_double(v)), meta);
    };
    auto put_bool = [&meta](const pmt::pmt_t& k, bool v) {
        meta = pmt::dcons(pmt::cons(k, pmt::from_bool(v)), meta);
    };

    const sic::SicState st =
        (p.admitted && sic_ran) ? r.state : sic::SicState::Bypassed;

    meta = pmt::dcons(pmt::cons(sym.key_sic_state, sym.state_symbol(st)), meta);
    put_bool(sym.key_sic_detected, p.admitted && sic_ran && r.detected);
    put_bool(sym.key_sic_committed, committed);
    put_bool(sym.key_sic_partial, p.admitted && sic_ran && r.partial);
    put_bool(sym.key_sic_ambiguous, p.admitted && sic_ran && r.ambiguous);
    meta = pmt::dcons(
        pmt::cons(sym.key_sic_bypass_reason, sym.reason_symbol(reason)), meta);
    put_bool(sym.key_sic_uses_truth, uses_truth);
    // Additive diagnostic: true when this window was rejected by admission and
    // passed through unchanged (the reason is in sic_bypass_reason).
    put_bool(sym.key_sic_missed_admission, !p.admitted);
    put_i64(sym.key_sic_detected_start,
            (p.admitted && sic_ran) ? r.detected_start : int64_t(-1));
    put_f64(sym.key_sic_cfo_hz, (p.admitted && sic_ran) ? r.cfo_hz : 0.0);
    put_u64(sym.key_sic_n_paths,
            (p.admitted && sic_ran) ? static_cast<uint64_t>(r.n_paths) : 0);
    put_i64(sym.key_sic_support_lo,
            (p.admitted && sic_ran) ? r.support_lo : int64_t(-1));
    put_i64(sym.key_sic_support_hi,
            (p.admitted && sic_ran) ? r.support_hi : int64_t(-1));
    // Actionable overlap of the detected support with the sensing processing ROI
    // (the subtraction/commit gate).  -1 when no sensing ROI was supplied.
    put_i64(sym.key_sic_sub_overlap,
            (p.admitted && sic_ran) ? r.sub_overlap : int64_t(-1));
    put_f64(sym.key_sic_residual_corr,
            (p.admitted && sic_ran) ? r.residual_corr : 0.0);

    const bool have_core = p.admitted && sic_ran;

    // ---- S1 frozen-model metadata (additive) ------------------------------
    // Published from the FROZEN SicModel when estimation ran (estimate mode);
    // otherwise neutral defaults so every `out` carries the same key set.  The
    // declared visibility limit is recorded verbatim (0 = whole window) together
    // with the meta key it came from; the RESOLVED limit is separate.
    const int64_t declared_end = observed_end_declared;
    const int64_t resolved_end =
        (have_model && model.observed_end_sample > 0)
        ? model.observed_end_sample
        : (declared_end == 0
               ? static_cast<int64_t>(p.n)
               : std::min<int64_t>(declared_end, static_cast<int64_t>(p.n)));
    const char* const support_source_pub =
        have_core ? (r.support_source != nullptr
                         ? r.support_source
                         : (have_model ? model.support_source : "none"))
                  : "none";
    const int64_t support_lo_pub = have_core ? r.support_lo : int64_t(-1);
    const int64_t support_hi_pub = have_core ? r.support_hi : int64_t(-1);
    double support_coverage = 0.0;
    if (have_core && support_hi_pub > support_lo_pub) {
        const double nominal =
            static_cast<double>(d_cfg_.sync_reps) *
                static_cast<double>(d_cfg_.samples_per_symbol) +
            static_cast<double>(d_cfg_.post_tail);
        if (nominal > 0.0)
            support_coverage =
                static_cast<double>(support_hi_pub - support_lo_pub) / nominal;
        if (support_coverage > 1.0)
            support_coverage = 1.0;
        if (support_coverage < 0.0)
            support_coverage = 0.0;
    }
    meta = pmt::dcons(
        pmt::cons(sym.key_sic_model_mode, sym.mode_symbol(d_model_mode_name_)),
        meta);
    put_u64(sym.key_sic_model_id, have_model ? model.model_id : uint64_t(0));
    meta = pmt::dcons(
        pmt::cons(
            sym.key_sic_model_fingerprint,
            (have_model && model.model_id != 0)
                ? pmt::string_to_symbol(hex_u64(model.model_id))
                : sym.sym_fingerprint_zero),
        meta);
    put_i64(sym.key_sic_observed_end_sample, declared_end);
    meta = pmt::dcons(
        pmt::cons(sym.key_sic_observed_end_source,
                  sym.observed_end_source_symbol(observed_end_source)),
        meta);
    put_i64(sym.key_sic_visible_end_sample, resolved_end);
    put_i64(sym.key_sic_training_lo, have_model ? model.training_lo : int64_t(-1));
    put_i64(sym.key_sic_training_hi, have_model ? model.training_hi : int64_t(-1));
    put_i64(sym.key_sic_estimation_ready_sample,
            have_model ? model.estimation_ready_sample : int64_t(-1));
    meta = pmt::dcons(
        pmt::cons(sym.key_sic_support_source,
                  sym.support_source_symbol(support_source_pub)),
        meta);
    put_f64(sym.key_sic_support_coverage, support_coverage);
    put_i64(sym.key_sic_sub_lo, have_model ? model.sub_lo : int64_t(-1));
    put_i64(sym.key_sic_sub_hi, have_model ? model.sub_hi : int64_t(-1));
    put_i64(sym.key_sic_phase_reference,
            have_model ? model.phase_reference_sample : int64_t(-1));
    put_f64(sym.key_sic_fit_residual,
            have_model ? model.fit_residual
                       : (have_core ? r.fit_residual : 0.0));
    put_f64(sym.key_sic_cond2,
            have_model ? model.gain_condition
                       : (have_core ? r.gain_condition : 0.0));

    // ---- S1 per-path frozen-model tables (additive) ----------------------
    // Published from the AUTHORITATIVE frozen SicModel (not SicResult::taus,
    // which is only an inspection mirror).  Index order is the model's own
    // selection order, i.e. 0 .. n_paths-1 of SicModel::tau_abs[], and the gain
    // tables use EXACTLY the same index order, so gain[k] belongs to tau[k].
    //   sic_tau       f64vector, absolute work-sample delays (tau_abs)
    //   sic_gain_re   f64vector, real part of the complex gain
    //   sic_gain_im   f64vector, imag part of the complex gain
    // A window with no APPLIED frozen model (off/oracle, bypass, failure) still
    // publishes all three keys, as EMPTY vectors: the consumer can distinguish
    // "no paths" (present, length 0) from "table not published" (absent).
    // `sic_path_count` mirrors the table length on `out` and `status`.
    const bool have_frozen = have_model && model.valid;
    const size_t n_tbl = have_frozen
        ? std::min<size_t>(model.n_paths,
                           static_cast<size_t>(sic::kSicMaxPaths))
        : 0;
    double tau_tbl[sic::kSicMaxPaths] = {};
    double gre_tbl[sic::kSicMaxPaths] = {};
    double gim_tbl[sic::kSicMaxPaths] = {};
    for (size_t k = 0; k < n_tbl; ++k) {
        tau_tbl[k] = model.tau_abs[k];
        gre_tbl[k] = static_cast<double>(model.gain[k].real());
        gim_tbl[k] = static_cast<double>(model.gain[k].imag());
    }
    meta = pmt::dcons(
        pmt::cons(sym.key_sic_tau, pmt::init_f64vector(n_tbl, tau_tbl)), meta);
    meta = pmt::dcons(
        pmt::cons(sym.key_sic_gain_re, pmt::init_f64vector(n_tbl, gre_tbl)), meta);
    meta = pmt::dcons(
        pmt::cons(sym.key_sic_gain_im, pmt::init_f64vector(n_tbl, gim_tbl)), meta);
    put_u64(sym.key_sic_path_count, static_cast<uint64_t>(n_tbl));
    put_f64(sym.key_sic_gain_cond,
            have_model ? model.gain_condition
                       : (have_core ? r.gain_condition : 0.0));
    // apply_model takes the model by const reference and has no re-fit path, so
    // the frozen parameters cannot change during apply.  These are the contract
    // witnesses the QA asserts on every committed window.
    put_u64(sym.key_sic_apply_refit_count, uint64_t(0));
    put_u64(sym.key_sic_apply_param_deltas, uint64_t(0));
    put_u64(sym.key_sic_nominal_sync_reps,
            have_model ? static_cast<uint64_t>(model.nominal_sync_reps)
                       : static_cast<uint64_t>(d_cfg_.sync_reps));
    put_u64(sym.key_sic_samples_per_symbol,
            have_model ? static_cast<uint64_t>(model.samples_per_symbol)
                       : static_cast<uint64_t>(d_cfg_.samples_per_symbol));
    put_f64(sym.key_sic_sample_rate,
            have_model ? model.sample_rate : d_cfg_.sample_rate);

    // Adapter timers.  Values written here are measured through CONSTRUCTION:
    // a message cannot carry the duration of its own enqueue.  The
    // pub-inclusive mirrors are published on the `status` event below.
    put_u64(sym.key_handler_us, p.handler_us);
    put_u64(sym.key_queue_wait_us, queue_wait_us);
    put_u64(sym.key_core_total_us, have_core ? r.timings.core_total_us : 0);

    // Core stage timers (children of core_total_us; never double counted).
    const sic::SicTimings& tt = r.timings;
    put_u64(sym.key_sic_detect_coarse_us, have_core ? tt.detect_coarse_us : 0);
    put_u64(sym.key_sic_detect_fine_us, have_core ? tt.detect_fine_us : 0);
    put_u64(sym.key_sic_cfo_us, have_core ? tt.cfo_us : 0);
    put_u64(sym.key_sic_cir_us, have_core ? tt.cir_us : 0);
    put_u64(sym.key_sic_path_select_us, have_core ? tt.path_select_us : 0);
    put_u64(sym.key_sic_fractional_refine_us,
            have_core ? tt.fractional_refine_us : 0);
    put_u64(sym.key_sic_matrix_build_us, have_core ? tt.matrix_build_us : 0);
    put_u64(sym.key_sic_gain_fit_us, have_core ? tt.gain_fit_us : 0);
    put_u64(sym.key_sic_reconstruct_us, have_core ? tt.reconstruct_us : 0);
    put_u64(sym.key_sic_validate_commit_us, have_core ? tt.validate_commit_us : 0);

    // S1 decomposition.  These describe the estimate/apply split and are NOT
    // summed into core_total_us (estimate_total_us is itself the parent of the
    // stage children above; model_prepare_us is inside estimate_total_us;
    // apply_us == backfill_us + apply_forward_us).  No parent/child is added
    // twice anywhere.
    put_u64(sym.key_sic_estimate_us, have_core ? tt.estimate_total_us : 0);
    put_u64(sym.key_sic_model_prepare_us, have_core ? tt.model_prepare_us : 0);
    put_u64(sym.key_sic_apply_us, have_core ? tt.apply_us : 0);
    put_u64(sym.key_sic_backfill_us, have_core ? tt.backfill_us : 0);
    put_u64(sym.key_sic_apply_forward_us, have_core ? tt.apply_forward_us : 0);
    // Refine children and cache counts.  Not members of SIC_CORE_STAGES: the
    // sim must not copy them into scope=sic stage rows (that sum is checked
    // against core_total_us).  They are diagnostic only.
    put_u64(sym.key_sic_refine_init_rot_us, have_core ? tt.refine_init_rot_us : 0);
    put_u64(sym.key_sic_refine_init_columns_us,
            have_core ? tt.refine_init_columns_us : 0);
    put_u64(sym.key_sic_refine_init_qr_us, have_core ? tt.refine_init_qr_us : 0);
    put_u64(sym.key_sic_refine_resid_us, have_core ? tt.refine_resid_us : 0);
    put_u64(sym.key_sic_refine_derot_us, have_core ? tt.refine_derot_us : 0);
    put_u64(sym.key_sic_refine_candidate_us, have_core ? tt.refine_candidate_us : 0);
    put_u64(sym.key_sic_refine_project_us, have_core ? tt.refine_project_us : 0);
    put_u64(sym.key_sic_refine_self_energy_us,
            have_core ? tt.refine_self_energy_us : 0);
    put_u64(sym.key_sic_refine_k, have_core ? tt.refine_k : 0);
    put_u64(sym.key_sic_refine_m, have_core ? tt.refine_m : 0);
    put_u64(sym.key_sic_refine_candidates, have_core ? tt.refine_candidates : 0);
    put_u64(sym.key_sic_refine_rebuilds, have_core ? tt.refine_rebuilds : 0);
    put_u64(sym.key_sic_refine_cache_hits, have_core ? tt.refine_cache_hits : 0);
    put_u64(sym.key_sic_refine_cache_misses, have_core ? tt.refine_cache_misses : 0);
    put_u64(sym.key_sic_refine_self_energy_hits,
            have_core ? tt.refine_self_energy_hits : 0);
    put_u64(sym.key_sic_refine_self_energy_misses,
            have_core ? tt.refine_self_energy_misses : 0);
    put_u64(sym.key_sic_apply_block_builds, have_core ? tt.apply_block_builds : 0);
    put_u64(sym.key_sic_apply_block_hits, have_core ? tt.apply_block_hits : 0);
    put_u64(sym.key_sic_apply_rot_base_hits, have_core ? tt.apply_rot_base_hits : 0);
    put_u64(sym.key_sic_apply_rot_base_misses,
            have_core ? tt.apply_rot_base_misses : 0);

    // Byte-exact bypass: reuse the producer's c32vector PMT unchanged.  A
    // committed window is a fresh vector copied from the cleaned output buffer.
    pmt::pmt_t out_vec =
        committed ? pmt::init_c32vector(p.n, d_out_.data()) : p.iq_in;
    pmt::pmt_t ihat_vec;
    if (can_publish_ihat)
        ihat_vec = pmt::init_c32vector(p.n, d_ihat_.data());

    // Pre-publish timers go into the out meta (see note above).
    {
        const auto t_now = std::chrono::steady_clock::now();
        put_u64(sym.key_output_wrap_publish_us, elapsed_us(t_wrap0, t_now));
        put_u64(sym.key_completion_latency_us, elapsed_us(p.offered_at, t_now));
        put_bool(sym.key_sic_pool_capacity_exceeded, pool_exceeded);
    }

    message_port_pub(sym.port_out, pmt::cons(meta, out_vec));
    if (can_publish_ihat)
        message_port_pub(sym.port_i_hat, pmt::cons(meta, ihat_vec));
    const auto t_pub_done = std::chrono::steady_clock::now();
    // Pub-inclusive adapter timers: output_wrap_publish_us spans PMT
    // construction AND the actual message_port_pub calls; completion_latency_us
    // spans the window's planned publish to the terminal publish.
    const uint64_t wrap_full_us = elapsed_us(t_wrap0, t_pub_done);
    const uint64_t latency_full_us = elapsed_us(p.offered_at, t_pub_done);

    // ---- status + counters ----------------------------------------------
    {
        pmt::pmt_t extra = pmt::make_dict();
        auto put_extra_u64 = [&extra](const pmt::pmt_t& k, uint64_t v) {
            extra = pmt::dcons(pmt::cons(k, pmt::from_uint64(v)), extra);
        };
        auto put_extra_i64 = [&extra](const pmt::pmt_t& k, int64_t v) {
            extra = pmt::dcons(pmt::cons(k, pmt::from_long(v)), extra);
        };
        auto put_extra_f64 = [&extra](const pmt::pmt_t& k, double v) {
            extra = pmt::dcons(pmt::cons(k, pmt::from_double(v)), extra);
        };
        auto put_extra_bool = [&extra](const pmt::pmt_t& k, bool v) {
            extra = pmt::dcons(pmt::cons(k, pmt::from_bool(v)), extra);
        };

        put_extra_u64(sym.key_window_id, dict_u64(p.meta_in, sym.key_window_id, 0));
        extra = pmt::dcons(pmt::cons(sym.key_sic_state, sym.state_symbol(st)), extra);
        extra = pmt::dcons(
            pmt::cons(sym.key_sic_bypass_reason, sym.reason_symbol(reason)), extra);
        put_extra_bool(sym.key_missed_admission, !p.admitted);
        put_extra_u64(sym.key_handler_us, p.handler_us);
        put_extra_u64(sym.key_queue_wait_us, queue_wait_us);
        put_extra_u64(sym.key_core_total_us, have_core ? tt.core_total_us : 0);
        put_extra_u64(sym.key_output_wrap_publish_us, wrap_full_us);
        put_extra_u64(sym.key_completion_latency_us, latency_full_us);
        // S1 frozen-model essentials mirrored on status.
        extra = pmt::dcons(
            pmt::cons(sym.key_sic_model_mode, sym.mode_symbol(d_model_mode_name_)),
            extra);
        put_extra_u64(sym.key_sic_model_id, have_model ? model.model_id : uint64_t(0));
        put_extra_i64(sym.key_sic_observed_end_sample, declared_end);
        extra = pmt::dcons(
            pmt::cons(sym.key_sic_support_source,
                      sym.support_source_symbol(support_source_pub)),
            extra);
        put_extra_i64(sym.key_sic_support_lo, support_lo_pub);
        put_extra_i64(sym.key_sic_support_hi, support_hi_pub);
        put_extra_i64(sym.key_sic_sub_lo, have_model ? model.sub_lo : int64_t(-1));
        put_extra_i64(sym.key_sic_sub_hi, have_model ? model.sub_hi : int64_t(-1));
        put_extra_u64(sym.key_sic_apply_refit_count, 0);
        put_extra_u64(sym.key_sic_apply_param_deltas, 0);
        // S1 per-path table counts (mirrors of the `out` meta; the tables
        // themselves stay on `out`).  sic_path_count is the number of PUBLISHED
        // per-path entries (len(sic_tau)); sic_n_paths is the core's detected
        // selection count.  They are equal on a committed window and both 0 on
        // a window where no frozen model was applied.
        put_extra_u64(sym.key_sic_path_count, static_cast<uint64_t>(n_tbl));
        put_extra_u64(sym.key_sic_n_paths,
                      have_core ? static_cast<uint64_t>(r.n_paths) : uint64_t(0));
        put_extra_f64(sym.key_sic_gain_cond,
                      have_model ? model.gain_condition
                                 : (have_core ? r.gain_condition : 0.0));
        put_extra_u64(sym.key_sic_estimate_us, have_core ? tt.estimate_total_us : 0);
        put_extra_u64(sym.key_sic_model_prepare_us,
                      have_core ? tt.model_prepare_us : 0);
        put_extra_u64(sym.key_sic_backfill_us, have_core ? tt.backfill_us : 0);
        put_extra_u64(sym.key_sic_apply_forward_us,
                      have_core ? tt.apply_forward_us : 0);
        publish_status("window", extra);
    }

    if (p.admitted) {
        d_completed_.fetch_add(1, std::memory_order_relaxed);
        if (failed || (sic_ran && r.state == sic::SicState::Failed))
            d_failed_.fetch_add(1, std::memory_order_relaxed);
        if (!committed)
            d_bypassed_.fetch_add(1, std::memory_order_relaxed);
        std::lock_guard<std::mutex> lock(d_queue_mutex_);
        if (d_in_flight_ > 0)
            --d_in_flight_;
        d_pending_bytes_ =
            (d_pending_bytes_ >= p.bytes) ? d_pending_bytes_ - p.bytes : 0;
    } else {
        d_bypassed_.fetch_add(1, std::memory_order_relaxed);
    }
}

void
UwbPreambleSic::publish_status(const std::string& event, pmt::pmt_t extra)
{
    const auto& sym = SicPmtSymbols::get();
    pmt::pmt_t meta = pmt::is_dict(extra) ? extra : pmt::PMT_NIL;
    auto put_u64 = [&meta](const pmt::pmt_t& k, uint64_t v) {
        meta = pmt::dcons(pmt::cons(k, pmt::from_uint64(v)), meta);
    };
    put_u64(sym.key_windows_missed_admission, windows_missed_admission());
    put_u64(sym.key_windows_invalid, windows_invalid());
    put_u64(sym.key_windows_dropped, windows_dropped());
    put_u64(sym.key_windows_bypassed, windows_bypassed());
    put_u64(sym.key_windows_failed, windows_failed());
    put_u64(sym.key_windows_completed, windows_completed());
    put_u64(sym.key_windows_admitted, windows_admitted());
    put_u64(sym.key_windows_offered, windows_offered());
    meta = pmt::dcons(pmt::cons(sym.key_event, sym.event_symbol(event)), meta);

    message_port_pub(sym.port_status, meta);
}

} // namespace uwb
} // namespace gr
