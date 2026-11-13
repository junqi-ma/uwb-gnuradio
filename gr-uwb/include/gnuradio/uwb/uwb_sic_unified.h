/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Unified UWB SIC Architecture (Stage M2 Contract).
 * Namespace gr::uwb::sic.
 *
 * Core Principle:
 *   "Preamble-only SIC is a special case of Full-packet SIC where
 *    known_fields == SYNC (or SYNC | SFD)."
 *
 * Mathematical Foundations:
 *   Both Preamble-Only and Full-Packet SIC share the identical physical
 *   reconstruction model:
 *
 *     i_hat[n] = exp(j * w * (n - r)) * (h * s_known)[n - q]
 *     out[n]   = y[n] - i_hat[n]   (evaluated only inside [sub_lo, sub_hi))
 *     out[n]   = y[n]              (byte-identical everywhere else)
 *
 *   where:
 *     - q is the integer packet origin (detected start sample on work grid),
 *     - w = 2*pi*cfo_hz/sample_rate is the carrier frequency offset in rad/sample,
 *     - r is the absolute phase reference anchor (r == q),
 *     - h is the estimated channel impulse response (discrete FIR taps h[l] or
 *       parametric multi-path delays/gains {tau_k, gain_k}),
 *     - s_known is the known emitted signal:
 *         * For L1 (Preamble): known_fields = SYNC (or SYNC|SFD), derived
 *           directly from public templates without requiring packet demodulation.
 *         * For L2 (FullPacket): known_fields = SYNC|SFD|PHR|PSDU|FCS, generated
 *           by re-modulating the decoded, SECDED-corrected, FCS-verified wire PSDU.
 *
 * Policies and Graceful Degradation:
 *   - SicPolicy::PreambleOnly:
 *       Cancels only the preamble region (SYNC repetitions). Packet demodulation
 *       is never attempted. Target cancellation level: L1_Preamble.
 *   - SicPolicy::FullPacket:
 *       Strict full-packet SIC. Decodes the comm packet; requires PHR and FCS to
 *       pass. If decoding or FCS validation fails, the transaction aborts to
 *       L0_Bypass (strict full-packet contract). Target cancellation level: L2_FullPacket.
 *   - SicPolicy::AutoFallback:
 *       Adaptive robust SIC. Attempts full-packet demodulation and cancellation
 *       (L2_FullPacket). If comm packet demodulation fails (e.g. payload SNR too low,
 *       FCS error, SECDED uncorrectable, unsupported PHY header), the engine
 *       DOES NOT abort to L0 Bypass; instead, it automatically and gracefully
 *       degrades to L1_Preamble, eliminating the interference across the known
 *       preamble symbols.
 *
 * Estimator Backends:
 *   - SicEstimatorBackend::DiscreteCir:
 *       Ridge-regularized discrete FIR channel estimation (h[l]). Highest
 *       suppression depth (~35.6 dB) on SYNC collisions.
 *   - SicEstimatorBackend::PathFractional:
 *       S1 parametric multi-path fractional delay ({tau_k, gain_k}) + QR fit.
 *   - SicEstimatorBackend::Ipatov:
 *       128-point correlation with sparse Ipatov code + unit impulse pulses.
 *       Fastest execution (~1.40 ms) for non-colliding preambles.
 *
 * Zero-Allocation Hot Path Guarantee:
 *   All scratch buffers (DiscreteCirScratch, SicScratch, FullPacketScratch,
 *   modulator, demodulator scratch, and working vectors) are preallocated once
 *   via prepare_sic_unified() or reserve(). The hot processing functions
 *   (estimate_channel_unified, reconstruct_and_subtract_unified,
 *   sic_process_unified) perform NO heap allocation, NO vector resizing, and NO
 *   dynamic reallocation.
 *
 * Strict Prefix Validation:
 *   All estimation stages read strictly within y[0, min(n, observed_end_sample)).
 *   Every correlation row, CIR tap, and CFO estimation pair lies entirely
 *   within the visible prefix.
 *
 * Non-Breaking Compatibility:
 *   Existing APIs in uwb_preamble_sic_core.h, uwb_discrete_cir_sic.h, and
 *   uwb_fullpacket_sic_core.h remain fully preserved and unchanged.
 */

#pragma once

#include <gnuradio/uwb/api.h>
#include <gnuradio/uwb/uwb_preamble_sic_core.h>
#include <gnuradio/uwb/uwb_discrete_cir_sic.h>
#include <gnuradio/uwb/uwb_fullpacket_sic_core.h>
#include <gnuradio/uwb/uwb_demod_core.h>
#include <gnuradio/uwb/uwb_hrp_mod_core.h>

#include <algorithm>
#include <array>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace gr {
namespace uwb {
namespace sic {

// ---------------------------------------------------------------------------
// 1. Enumerations and Bitmasks
// ---------------------------------------------------------------------------

/**
 * @brief High-level cancellation policy governing operation mode and fallback.
 */
enum class SicPolicy : uint8_t {
    PreambleOnly = 0, ///< Always cancel only the preamble (known a priori). Never demod comm.
    FullPacket   = 1, ///< Strict full-packet cancel. If demod/FCS fails, abort to L0 Bypass.
    AutoFallback = 2  ///< Attempt full-packet cancel; if demod/FCS fails, gracefully fall back to L1 Preamble.
};

/**
 * @brief Estimator algorithmic backend.
 */
enum class SicEstimatorBackend : uint8_t {
    DiscreteCir    = 0, ///< Ridge-regularized discrete FIR channel estimation (h[l]).
    PathFractional = 1, ///< Parametric multi-path fractional delay + QR solve (S1 baseline).
    Ipatov         = 2  ///< 128-point correlation with sparse Ipatov code + unit impulse pulses.
};

/**
 * @brief Achieved cancellation level for a processed window.
 */
enum class SicCancellationLevel : uint8_t {
    L0_Bypass     = 0, ///< No cancellation performed: out == y bit-identically, i_hat == 0.
    L1_Preamble   = 1, ///< Preamble-only cancellation (known SYNC / SFD subtracted).
    L2_FullPacket = 2  ///< Full-packet cancellation (remodulated complete frame subtracted).
};

/**
 * @brief Bitmask representing the fields of the comm frame known/reconstructed for cancellation.
 *
 * Preamble-only SIC is formally represented by known_fields == SicKnownFields::Sync
 * (or Sync | Sfd).
 */
enum class SicKnownFields : uint32_t {
    None        = 0,
    Sync        = 1 << 0, ///< SYNC symbols (known a priori from comm code).
    Sfd         = 1 << 1, ///< SFD symbols (known a priori from standard/profile).
    Phr         = 1 << 2, ///< PHR header (decoded & SECDED verified).
    Payload     = 1 << 3, ///< PSDU payload data (decoded & FCS verified).
    Fcs         = 1 << 4, ///< 16-bit CRC / FCS.

    Preamble    = Sync,
    PreambleSfd = Sync | Sfd,
    FullPacket  = Sync | Sfd | Phr | Payload | Fcs
};

inline constexpr SicKnownFields operator|(SicKnownFields a, SicKnownFields b) {
    return static_cast<SicKnownFields>(static_cast<uint32_t>(a) | static_cast<uint32_t>(b));
}
inline constexpr SicKnownFields operator&(SicKnownFields a, SicKnownFields b) {
    return static_cast<SicKnownFields>(static_cast<uint32_t>(a) & static_cast<uint32_t>(b));
}
inline constexpr SicKnownFields operator^(SicKnownFields a, SicKnownFields b) {
    return static_cast<SicKnownFields>(static_cast<uint32_t>(a) ^ static_cast<uint32_t>(b));
}
inline constexpr SicKnownFields operator~(SicKnownFields a) {
    return static_cast<SicKnownFields>(~static_cast<uint32_t>(a));
}
inline constexpr bool has_field(SicKnownFields mask, SicKnownFields field) {
    return (static_cast<uint32_t>(mask) & static_cast<uint32_t>(field)) == static_cast<uint32_t>(field);
}

// ---------------------------------------------------------------------------
// 2. Unified Channel Model
// ---------------------------------------------------------------------------

/**
 * @brief Unified channel model representation bridging discrete FIR taps and
 *        parametric fractional paths.
 */
struct SicUnifiedChannel {
    bool valid = false;
    SicEstimatorBackend backend = SicEstimatorBackend::DiscreteCir;
    const char* status = "not_estimated"; ///< Stable literal
    const char* reason = nullptr;         ///< Stable literal or nullptr
    uint64_t model_id = 0;                ///< Deterministic hash of frozen fields

    // Common packet geometry and synchronization anchors
    int64_t q = -1;                       ///< Detected packet origin (work samples)
    double cfo_hz = 0.0;                  ///< Carrier frequency offset in Hz
    int64_t phase_reference_sample = 0;   ///< CFO anchor sample (typically q)
    int64_t observed_end_sample = 0;      ///< Strict-prefix visibility limit used
    int64_t training_lo = -1;             ///< Visible training span start [training_lo, training_hi)
    int64_t training_hi = -1;             ///< Visible training span end

    // Estimation quality diagnostics
    double fit_residual = 0.0;            ///< Normalized residual ||y - i_hat||^2 / ||y||^2 on training
    double gain_condition = 0.0;          ///< Condition number of design matrix or factor
    size_t repeats_confirmed = 0;         ///< Repetitions confirmed inside visible prefix

    // Discrete CIR representation (used when backend == DiscreteCir or Ipatov)
    int64_t lag_lo = 0;                   ///< Lowest integer lag
    size_t tap_count = 0;                 ///< Continuous hull tap count (<= kDiscreteCirMaxTaps)
    size_t active_count = 0;              ///< Nonzero active tap count
    std::array<int16_t, kDiscreteCirMaxActive> active_lags{};
    std::array<std::complex<float>, kDiscreteCirMaxTaps> taps{}; ///< Dense hull, 0 on inactive
    bool impulse_template = false;        ///< True if taps include pulse shaping (Ipatov)
    size_t cir_repetitions = 0;           ///< Correlation repetitions averaged (Ipatov)

    // Parametric multi-path representation (used when backend == PathFractional)
    size_t n_paths = 0;                   ///< Number of fitted paths (<= kSicMaxPaths)
    double tau_abs[kSicMaxPaths] = {};    ///< Absolute fractional delays
    std::complex<float> gain[kSicMaxPaths] = {}; ///< Complex gains

    // Conversion helpers from/to legacy structs
    UWB_API void from_discrete_cir(const DiscreteCirModel& m);
    UWB_API void from_parametric(const SicModel& m);
    UWB_API void to_discrete_cir(DiscreteCirModel& m) const;
    UWB_API void to_parametric(SicModel& m) const;
};

// ---------------------------------------------------------------------------
// 3. Unified Configuration
// ---------------------------------------------------------------------------

/**
 * @brief Configuration parameters for unified SIC processing.
 */
struct SicUnifiedConfig {
    // Policy and Backend (Fixed to historical baseline: Ipatov backend + AutoFallback)
    SicPolicy policy = SicPolicy::AutoFallback;
    SicEstimatorBackend backend = SicEstimatorBackend::Ipatov;

    // Physical Layer Profile
    size_t comm_code = 10;
    size_t sync_reps = 64;
    size_t samples_per_symbol = 1016;
    double sample_rate = 998.4e6;
    const char* sfd_mode = "4z2";

    // ROI and Visibility (Work sample coordinates)
    int64_t roi_lo = 0;                   ///< Public coarse search ROI [roi_lo, roi_hi)
    int64_t roi_hi = 0;
    int64_t sub_roi_lo = 0;               ///< Sensing protected subtraction ROI [sub_roi_lo, sub_roi_hi)
    int64_t sub_roi_hi = 0;
    size_t sub_min_overlap = 0;           ///< Minimum overlap between comm support and sub_roi (0 => >0)
    int64_t observed_end_sample = 0;      ///< Strict-prefix visibility limit (0 => whole window)

    // Estimation Thresholds and Strides
    size_t fit_reps = 8;                  ///< SYNC symbols used for training (strictly inside prefix)
    size_t coarse_stride = 1;             ///< Acquisition coarse stride
    double detect_threshold = 0.40;       ///< Detection metric threshold
    double fit_residual_max = 0.80;       ///< Commit ceiling on normalized training residual
    double gain_condition_max = 1e8;      ///< Maximum acceptable condition number

    // Discrete CIR / Ipatov Backend Specific Options (Fixed to 20-tap historical baseline)
    int64_t lag_lo = -1;                  ///< Compact lag origin (-1)
    size_t tap_count = 20;                ///< 20-tap compact support
    double peak_floor = 0.15;
    size_t peak_min_spacing = 5;
    size_t active_radius = 1;
    double lambda = 1e-6;                 ///< Ridge regularization parameter
    bool sparse_pulse_reconstruct = true; ///< Fast single-pass AVX2 pulse accumulator (historical baseline)

    // Path Fractional Backend Specific Options
    size_t k_paths = 4;
    size_t post_tail = 100;
    bool enable_fractional = true;
    bool enable_cfo = true;

    // Receiver (Comm Demod) Optimization Options
    bool rx_fast_sfd = true;
    bool rx_fast_cfo = true;
    bool rx_seed_refine_tail = true;
    bool rx_softchip_on_demand = true;
    bool rx_fused_payload_cmf = true;
    bool rx_fused_phr_cmf = true;

    mod::PulseSpec pulse;
    bool full_window_search = false;
    size_t comm_timing_search_margin = 256;
    size_t comm_timing_coarse_stride = 1;
    bool autocorr_prefilter = true;
    double autocorr_threshold = 0.20;
    bool autocorr_search_inside_target = false;
    int64_t target_preamble_lo = -1;
    int64_t target_preamble_hi = -1;

    // Kernel SIMD controls
    bool enable_b3_simd = true;

    // Bypass output policy
    //
    // When true (the historical default, kept for every existing caller and
    // test) every L0_Bypass / fail-closed exit copies the whole window
    // y[0, n) into result.out, so `out` is always a complete, valid window on
    // return regardless of `committed`.
    //
    // When false the O(n) copy is skipped and `out` is left untouched on those
    // exits.  `result.committed` is still false, so the caller MUST read its
    // own input buffer instead of `out`.  This is the contract the UHD SC16
    // pipeline worker uses: it keeps feeding the resampled input downstream
    // unless `committed` is true, and the per-window 122 us copy plus the
    // 116 us i_hat zero-fill disappear from the clean-window path.
    //
    // Precise contract with the flag false:
    //   * committed == true  => `out` holds the complete cleaned window.
    //   * committed == false => `out` is UNSPECIFIED and must not be read.
    // The five top-level L0 bypass exits (invalid_buffers_or_scratch,
    // config_or_template_mismatch, estimation failed, PreambleOnly
    // sfd_detect_failed, strict-FullPacket abort) additionally leave `out`
    // byte-untouched.  A reconstruction-level fail-closed may still leave `y`
    // in `out`: the frozen discrete-CIR apply API (uwb_discrete_cir_sic.h)
    // performs its own fail-closed copy and is not parameterised by this flag.
    // That is harmless because `committed` stays false.
    //
    // The committed path is identical under both settings: `out` is fully
    // written (subtraction ROI plus byte-identical y outside it).
    //
    // NOTE: this flag is deliberately NOT part of compute_unified_config_hash()
    // because it does not change any cached preamble/template content, so a
    // caller may prepare scratch with the default and then run with either
    // value without tripping the config_or_template_mismatch guard.
    bool copy_input_on_bypass = true;

    // Buffer Capacities
    size_t max_window_samples = 0;        ///< Maximum supported y / out window length
    size_t max_packet_samples = 0;        ///< Maximum remodulated packet length

    // Helper conversion methods to backend-specific configs
    UWB_API SicConfig to_sic_config() const;
    UWB_API DiscreteCirConfig to_discrete_cir_config() const;
    UWB_API mod::HrpModConfig to_hrp_mod_config() const;
};

// ---------------------------------------------------------------------------
// 4. Unified Templates
// ---------------------------------------------------------------------------

/**
 * @brief Public waveform templates supplied by the caller.
 */
struct SicUnifiedTemplates {
    const std::complex<float>* sync = nullptr;        ///< One shaped SYNC waveform (length == samples_per_symbol)
    size_t sync_len = 0;
    const std::complex<float>* preamble = nullptr;    ///< Full comm preamble waveform (real repetition pattern)
    size_t preamble_len = 0;
    const std::complex<float>* impulse_sync = nullptr;///< Unit impulse SYNC (used for Ipatov backend)
    size_t impulse_sync_len = 0;

    // Helper conversion methods
    UWB_API SicTemplates to_sic_templates() const;
    UWB_API DiscreteCirTemplates to_discrete_cir_templates() const;
};

// ---------------------------------------------------------------------------
// 5. Unified Timings
// ---------------------------------------------------------------------------

/**
 * @brief Detailed stage and sub-stage timings in microseconds.
 */
struct SicUnifiedTimings {
    // Primary Stage Totals (Sequential non-overlapping breakdown)
    double core_total_us = 0.0;           ///< Total steady_clock wall time of unified SIC execution
    double estimate_us = 0.0;             ///< Prefix estimation (detect, CFO, CIR/paths, solve, fit check)
    double comm_rx_us = 0.0;              ///< Fast communication demodulation (timing, SFD, CIR, PHR, payload)
    double tx_remod_us = 0.0;             ///< Remodulation of wire PSDU or pulse list
    double reconstruct_us = 0.0;          ///< Channel synthesis (FIR / sparse pulses / polyphase + CFO rot)
    double subtract_us = 0.0;             ///< Transactional subtraction inside sub_roi & bypass copies

    // Estimation Sub-timings (Children of estimate_us)
    double est_detect_us = 0.0;
    double est_cfo_us = 0.0;
    double est_cir_us = 0.0;
    double est_solve_us = 0.0;
    double est_fit_check_us = 0.0;

    // Comm Demod Sub-timings (Children of comm_rx_us)
    double rx_timing_us = 0.0;
    double rx_cfo_us = 0.0;
    double rx_sfd_us = 0.0;
    double rx_cir_us = 0.0;
    double rx_phr_us = 0.0;
    double rx_payload_us = 0.0;

    // Reconstruction Sub-timings (Children of reconstruct_us)
    double recon_scale_us = 0.0;
    double recon_conv_us = 0.0;
    double recon_cfo_rot_us = 0.0;
};

// ---------------------------------------------------------------------------
// 6. Unified Result
// ---------------------------------------------------------------------------

/**
 * @brief Comprehensive outcome, cancellation metrics, demodulation status,
 *        and execution diagnostics for a processed window.
 */
struct SicUnifiedResult {
    // Policy, Level, and Outcome
    SicPolicy policy = SicPolicy::AutoFallback;
    SicCancellationLevel level = SicCancellationLevel::L0_Bypass;
    SicKnownFields cancelled_fields = SicKnownFields::None;
    bool committed = false;               ///< True if interference was actively subtracted in sub_roi
    const char* reason = "not_run";       ///< Stable literal: "committed_full_packet", "fallback_preamble", "bypassed_detect_failed", etc.
    const char* fallback_reason = nullptr;///< Explains why fallback occurred (e.g. "comm_fcs_failed", "phr_secded_uncorrectable")

    // Quantified Cancellation Metrics
    double suppression_db = 0.0;          ///< Measured residual suppression: 10 * log10(||y||^2 / ||out||^2) inside [sub_lo, sub_hi)
    double fit_residual = 0.0;            ///< Training residual ||y - i_hat||^2 / ||y||^2
    double waveform_scale = 0.0;          ///< Applied waveform scale relative to reference SYNC

    // Communication Packet Demodulation Info (when attempted)
    bool comm_detected = false;
    bool comm_phr_pass = false;
    bool comm_fcs_pass = false;
    bool phr_secded_corrected = false;
    bool phr_secded_uncorrectable = false;
    uint16_t comm_phr_length = 0;
    double comm_phr_rate_mbps = 0.0;
    size_t wire_psdu_len = 0;
    uint8_t wire_psdu[127] = {};          ///< Decoded wire PSDU (including 2-byte FCS)

    // Cancellation Support and ROI Geometry (Work samples)
    int64_t support_lo = 0;               ///< Union of reconstructed packet support [support_lo, support_hi)
    int64_t support_hi = 0;
    int64_t sub_lo = 0;                   ///< Active subtraction range [sub_lo, sub_hi) = support ∩ sub_roi
    int64_t sub_hi = 0;
    int64_t sub_overlap = 0;              ///< Overlap length in samples

    // Frozen Channel Snapshot
    SicUnifiedChannel channel;

    // Execution Timings
    SicUnifiedTimings timings;

    // Caller-owned Output Buffers
    // out: cleaned window of length n. Bit-identical to y on L0 Bypass and outside [sub_lo, sub_hi).
    //      With cfg.copy_input_on_bypass == false the L0 Bypass copy is skipped:
    //      `committed` is then the only signal the caller needs (false => read
    //      your own input), and `out` must not be read.
    // i_hat: optional reconstruction buffer of length n. Zero outside support.
    //        May be nullptr: the reconstruction then uses internal scratch and no
    //        i_hat write (including the outside-support zero fill) happens.
    std::complex<float>* out = nullptr;
    std::complex<float>* i_hat = nullptr;
};

// ---------------------------------------------------------------------------
// 7. Zero-Allocation Hot Path Scratch Structures
// ---------------------------------------------------------------------------

/**
 * @brief Preallocated scratch memory holding all working buffers and sub-scratches.
 *
 * Sized once via reserve() or prepare_sic_unified(). No vector growth or heap
 * allocation occurs on the hot processing path.
 */
struct SicUnifiedScratch {
    bool prepared = false;
    size_t capacity_window_samples = 0;
    size_t capacity_packet_samples = 0;
    uint64_t config_hash = 0;
    uint64_t template_hash = 0;

    // Subordinate component scratches
    DiscreteCirScratch discrete;
    SicScratch fractional;
    FullPacketScratch fullpacket;
    mod::HrpModScratch modulator;
    demod::core::DemodScratch demod;

    // Unified working vectors (pre-reserved to capacity)
    std::vector<std::complex<float>> work_waveform;
    std::vector<std::complex<float>> work_accum;
    std::vector<std::complex<float>> work_rot;
    std::vector<uint8_t> work_phr_bits;
    std::vector<uint8_t> work_wire_psdu;

    // Fixed preamble and pulse event caches (C1 optimization)
    bool preamble_cache_valid = false;
    uint64_t preamble_cache_config_hash = 0;
    uint64_t preamble_cache_template_hash = 0;
    size_t cached_preamble_samples = 0;
    std::vector<std::complex<float>> cached_sync_vec;
    std::vector<std::complex<float>> cached_preamble_waveform;
    std::vector<mod::PulseEvent> cached_preamble_events;
    // Ipatov backend only: the SYNC + SFD prefix as UNIT impulses (+-1 at the
    // pulse positions, 0 elsewhere).  The Ipatov model is fitted against the
    // unit reference SYNC and its taps already contain the pulse shaping, so
    // the reconstruction input must be this waveform, never the shaped one.
    std::vector<std::complex<float>> cached_preamble_unit_waveform;

    /**
     * @brief Preallocates all scratch buffers to handle windows up to max_window_samples
     *        and packet waveforms up to max_packet_samples.
     */
    UWB_API void reserve(size_t max_window_samples, size_t max_packet_samples = 0);

    /**
     * @brief Cheap template identity cache backing cache_matches().
     *
     * The full template digest (FNV over the SYNC / impulse-SYNC content, i.e.
     * ~8 KB per SYNC) is only recomputed when the (pointer, length) tuple
     * changes or when the caller bumped template_epoch.  Steady-state windows
     * therefore cost one short config hash and a tuple compare instead of an
     * 8 KB hash.
     *
     * RESIDUAL ALIASING RISK (documented, not detectable): the same pointer
     * with the same length and mutated CONTENT is invisible to this cache.  A
     * caller that mutates template content in place MUST call
     * invalidate_template_identity() (or invalidate_cache() followed by
     * prepare_sic_unified()) before the next window.  cache_matches(cfg, tmpl,
     * true) always recomputes the full digest and is what QA uses.
     */
    struct TemplateIdentity {
        const void* sync = nullptr;
        size_t sync_len = 0;
        const void* preamble = nullptr;
        size_t preamble_len = 0;
        const void* impulse_sync = nullptr;
        size_t impulse_sync_len = 0;
        uint64_t epoch = 0;
        uint64_t digest = 0;
        bool valid = false;
    };

    /// Bumped by invalidate_cache() and invalidate_template_identity().
    uint64_t template_epoch = 0;
    /// Mutable so the const cache_matches() can memoise the digest.
    mutable TemplateIdentity template_identity;

    /**
     * @brief Checks if cached preamble matches current config and templates.
     *
     * The config hash is always recomputed in full (never weakened).  The
     * template digest is reused while the (pointer, length) tuple and
     * template_epoch are unchanged.
     *
     * @param force_full_hash Recompute the complete template content digest
     *        even if the cached tuple matches.  QA / in-place-mutation callers.
     */
    UWB_API bool cache_matches(const SicUnifiedConfig& cfg,
                               const SicUnifiedTemplates& tmpl,
                               bool force_full_hash = false) const;

    /**
     * @brief Invalidates the preamble and pulse event cache.
     */
    UWB_API void invalidate_cache();

    /**
     * @brief Forces the next cache_matches() to recompute the template digest.
     *
     * Use this after mutating template CONTENT in place (same pointer and
     * length).  It does not clear the prepared preamble cache, so it is much
     * cheaper than invalidate_cache() + prepare_sic_unified().
     */
    UWB_API void invalidate_template_identity();
};

// ---------------------------------------------------------------------------
// 8. Unified Core API
// ---------------------------------------------------------------------------

/**
 * @brief Computes configuration identity hash for preamble and estimation caching.
 */
UWB_API uint64_t compute_unified_config_hash(const SicUnifiedConfig& cfg);

/**
 * @brief Computes template identity hash for preamble caching.
 */
UWB_API uint64_t compute_unified_template_hash(const SicUnifiedTemplates& tmpl);

/**
 * @brief Computes squared energy of complex float signals over [start, end)
 *        using SIMD (AVX2/FMA when available) with double-precision accumulation.
 *
 * @param y First complex vector (typically input signal).
 * @param out Second complex vector (typically cleaned output signal).
 * @param start Start index in y and out.
 * @param end End index (exclusive) in y and out.
 * @param e_y Output accumulated double-precision energy for y.
 * @param e_out Output accumulated double-precision energy for out.
 */
UWB_API void compute_roi_energies(
    const std::complex<float>* y,
    const std::complex<float>* out,
    int64_t start,
    int64_t end,
    double& e_y,
    double& e_out);

/**
 * @brief Prepares and pre-allocates unified SIC scratch structures.
 *
 * May allocate memory; must be called during initialization or when the
 * PHY configuration or templates change. Safe to call multiple times.
 *
 * @param cfg Unified configuration.
 * @param templates Waveform templates.
 * @param scratch Scratch structure to prepare and size.
 * @param err Optional diagnostic error message on failure.
 * @return true on success, false on failure (with err populated).
 */
UWB_API bool prepare_sic_unified(
    const SicUnifiedConfig& cfg,
    const SicUnifiedTemplates& templates,
    SicUnifiedScratch& scratch,
    std::string* err = nullptr);

/**
 * @brief Performs strict-prefix channel estimation using the configured backend.
 *
 * ZERO heap allocation: scratch must be prepared.
 * Reads STRICTLY within y[0, min(n, observed_end)).
 *
 * @param y Input IQ window.
 * @param n Total window sample count.
 * @param observed_end Strict prefix visibility limit.
 * @param templates Waveform templates.
 * @param cfg Unified configuration.
 * @param scratch Pre-allocated scratch.
 * @param channel Output frozen channel representation.
 * @param timings Optional timing diagnostics.
 * @return true if an estimation attempt was validly made, false on unrecoverable input bug.
 */
UWB_API bool estimate_channel_unified(
    const std::complex<float>* y,
    size_t n,
    int64_t observed_end,
    const SicUnifiedTemplates& templates,
    const SicUnifiedConfig& cfg,
    SicUnifiedScratch& scratch,
    SicUnifiedChannel& channel,
    SicUnifiedTimings* timings = nullptr);

/**
 * @brief Reconstructs the interference waveform from known fields and subtracts
 *        it from y inside the permitted subtraction ROI.
 *
 * ZERO heap allocation: scratch must be prepared.
 *
 * When level == L1_Preamble, s_known is the known shaped preamble waveform (or SYNC).
 * When level == L2_FullPacket, s_known is the full remodulated packet waveform.
 *
 * Fail-closed guarantee: any failure copies y to out bit-identically and zeroes i_hat.
 * Exception: when cfg.copy_input_on_bypass == false the copy is skipped and `out` is
 * left untouched; result.committed is still false, so the caller reads its own input.
 * i_hat may be nullptr (no write, no outside-support zero fill).
 *
 * @param y Input IQ window.
 * @param n Total window sample count.
 * @param channel Frozen estimated channel.
 * @param level Achieved cancellation level (L1_Preamble or L2_FullPacket).
 * @param known_fields Bitmask of known fields contained in s_known.
 * @param s_known Emitted reference waveform for known fields.
 * @param s_known_len Length of s_known in samples.
 * @param templates Waveform templates.
 * @param cfg Unified configuration.
 * @param scratch Pre-allocated scratch.
 * @param out Output cleaned buffer (length n).
 * @param i_hat Optional reconstruction buffer (length n).
 * @param result Output outcome and metrics.
 * @return true on success, false on fail-closed error.
 */
UWB_API bool reconstruct_and_subtract_unified(
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
    SicUnifiedResult& result);

/**
 * @brief Unified end-to-end hot path processing function.
 *
 * ZERO heap allocation: scratch must be prepared.
 *
 * Complete Processing Pipeline:
 *   1. Prefix Validation: Enforces strict visibility limit y[0, min(n, observed_end)).
 *   2. Channel Estimation: Invokes selected backend (DiscreteCir, PathFractional, Ipatov).
 *      If estimation fails -> L0_Bypass (out == y, i_hat == 0).
 *   3. Policy Dispatch & Comm RX:
 *      - If policy == PreambleOnly:
 *          Skip comm demodulation. known_fields = SYNC.
 *          Target level = L1_Preamble.
 *      - If policy == FullPacket or AutoFallback:
 *          Run fast receiver demodulation on mixed IQ seeded at channel.q.
 *          Check PHR SECDED and CRC16 FCS:
 *          * If pass: remodulate complete packet; target level = L2_FullPacket.
 *          * If fail & policy == FullPacket: abort to L0_Bypass.
 *          * If fail & policy == AutoFallback: degrade gracefully to L1_Preamble!
 *            known_fields = SYNC. Reconstruct using known preamble.
 *   4. Channel Reconstruction & CFO Rotation:
 *      Synthesize interference i_hat across packet support.
 *   5. Transactional Subtraction & Gate Validation:
 *      Subtract only inside [sub_lo, sub_hi) = support ∩ sub_roi.
 *      Verify non-finite bounds and commit gates.
 *      Calculate residual suppression dB and timing metrics.
 *
 * @param y Input IQ window.
 * @param n Sample count in y.
 * @param templates Public waveform templates.
 * @param cfg Runtime configuration.
 * @param scratch Pre-allocated scratch.
 * @param result Execution result, metrics, and caller-owned buffer pointers.
 * @return true on execution, false on fatal input bug (null pointers, 0 samples).
 */
UWB_API bool sic_process_unified(
    const std::complex<float>* y,
    size_t n,
    const SicUnifiedTemplates& templates,
    const SicUnifiedConfig& cfg,
    SicUnifiedScratch& scratch,
    SicUnifiedResult& result);

} // namespace sic
} // namespace uwb
} // namespace gr
