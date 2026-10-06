/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * M2-A: the offline native-rate PHY closed loop for TWR frames.
 *
 * ---------------------------------------------------------------------------
 * What this is
 * ---------------------------------------------------------------------------
 *
 * A bounded, radio-free helper that takes a REAL frame v1 (Poll / Response /
 * Final), runs it through the repository's existing modulator, resampler and
 * demodulator cores at a NATIVE rate, and reports whether the bytes survived:
 *
 *   Frame v1 -> codec MAC bytes (no FCS)
 *            -> HRP append_ieee_fcs (exactly once) -> modulate_one
 *            -> work IQ @ 998.4 MS/s
 *            -> TX resampler  (work -> native: <48,65> or <32,65>)
 *            -> native CF32, optionally quantised CF32->SC16->CF32
 *            -> RX resampler  (native -> work: <65,48> or <65,32>)
 *            -> demodulate_one (real PHR / payload / FCS)
 *            -> codec decode -> byte-exact comparison with the input
 *
 * It reuses the ONE existing core for each stage; it is not a second
 * implementation.  See docs/twr/M2-A_G0接口与数字坐标.md (the frozen G0) and
 * docs/twr/OpenCode开发指示_M2-A_native_PHY闭环.md.
 *
 * ---------------------------------------------------------------------------
 * What it deliberately does NOT do
 * ---------------------------------------------------------------------------
 *
 *   * No first-path, no ToA, no RMARKER, no ranging.  The decode coordinates
 *     this helper records (`sfd_start_sample`, `packet_start_sample`) are
 *     SAMPLE-GRID BOOKKEEPING for diagnostics, NOT a protocol RMARKER and not
 *     an antenna reference plane.  Nothing here produces a `HardwareMeasured`
 *     RX time, a first-path verdict, or a distance.
 *   * No GNU Radio block, no UHD, no controller, no real-time deadline, no
 *     pybind.  A native round trip succeeding is `native_roundtrip_verified`
 *     evidence for the exact rate/profile/taps/format measured, and NOTHING
 *     more: `allows(NativeRoundtripVerified, Ranging)` stays false.
 *   * No CFO/SFO estimation, no multipath model.  Impairments belong to the
 *     test harness, never to the decoder's inputs as a "truth".
 *
 * ---------------------------------------------------------------------------
 * Numeric contract (G0 §3, §5, §6, §7)
 * ---------------------------------------------------------------------------
 *
 *   * Resampling is CAUSAL FULL CONVOLUTION (`upfirdn`), never zero-phase
 *     centre-crop.  `Lout = ceil(((N-1)*L + T)/M)`; N == 0 is defined here as
 *     ZERO output with no state change (the raw core would emit a transient).
 *   * Taps are caller-supplied and pre-scaled: DC sum == Interp so the
 *     effective gain is 1.  The core never scales.
 *   * Exact integers and rationals for coordinates; NO integer-nanosecond
 *     truncation anywhere.
 *   * SC16 is `clamp(llround(x*S), -32768, 32767)` with
 *     `S = 32767 / peak_amplitude`, saturation counted, NaN/Inf refused.
 *   * SUCCESS (G0 §6, review B3) is exactly:
 *         demod status == Success AND fcs_pass AND
 *         decoded PSDU length == expected AND bytes byte-exact.
 *     Any demod payload FALLBACK candidate, any FcsFailed, any length
 *     mismatch is a FAILURE and is counted, never reported as success.
 *
 * Enum handling follows the M0.1 N07 rule: every enum has an `xxx_is_known()`
 * written as a switch with NO `default`, and a domain test runs before the
 * switch that consumes the value.
 */

#ifndef INCLUDED_GNURADIO_UWB_UWB_TWR_PHY_H
#define INCLUDED_GNURADIO_UWB_UWB_TWR_PHY_H

#include <gnuradio/uwb/uwb_demod_core.h>
#include <gnuradio/uwb/uwb_demod_result.h>
#include <gnuradio/uwb/uwb_hrp_mod_core.h>
#include <gnuradio/uwb/uwb_rational_resampler_core.h>
#include <gnuradio/uwb/uwb_twr_frame.h>

#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace gr {
namespace uwb {
namespace twr {

// ===========================================================================
// Enums (each with a fail-closed domain test; no `default` switches)
// ===========================================================================

// The two native rates M2-A must cover (G0 §1).  Both are mandatory paths.
enum class M2aNativeRate : uint8_t {
    Uc200_737280000 = 0, // 998.4e6 * 48/65
    Cg400_491520000 = 1  // 998.4e6 * 32/65
};

inline const char* m2a_native_rate_to_string(M2aNativeRate r)
{
    switch (r) {
    case M2aNativeRate::Uc200_737280000:
        return "737280000";
    case M2aNativeRate::Cg400_491520000:
        return "491520000";
    }
    return "invalid";
}

inline bool m2a_native_rate_is_known(M2aNativeRate r)
{
    switch (r) {
    case M2aNativeRate::Uc200_737280000:
    case M2aNativeRate::Cg400_491520000:
        return true;
    }
    return false;
}

// TX interpolation / RX decimation for each rate (G0 §3.3).
inline bool m2a_rate_tx_lm(M2aNativeRate r, uint32_t& l, uint32_t& m)
{
    if (!m2a_native_rate_is_known(r))
        return false;
    switch (r) {
    case M2aNativeRate::Uc200_737280000:
        l = 48;
        m = 65;
        return true;
    case M2aNativeRate::Cg400_491520000:
        l = 32;
        m = 65;
        return true;
    }
    return false;
}

inline double m2a_rate_hz(M2aNativeRate r)
{
    switch (r) {
    case M2aNativeRate::Uc200_737280000:
        return 737280000.0;
    case M2aNativeRate::Cg400_491520000:
        return 491520000.0;
    }
    return 0.0;
}

// Native sample format (G0 §7).
enum class M2aIqFormat : uint8_t {
    Cf32 = 0,
    Sc16 = 1
};

inline const char* m2a_iq_format_to_string(M2aIqFormat f)
{
    switch (f) {
    case M2aIqFormat::Cf32:
        return "cf32";
    case M2aIqFormat::Sc16:
        return "sc16";
    }
    return "invalid";
}

inline bool m2a_iq_format_is_known(M2aIqFormat f)
{
    switch (f) {
    case M2aIqFormat::Cf32:
    case M2aIqFormat::Sc16:
        return true;
    }
    return false;
}

// Why the closed loop stopped.  Append-only schema.
enum class M2aStatus : uint8_t {
    Ok = 0,
    InvalidConfig = 1,   // a config field is out of domain / inconsistent
    InvalidFrame = 2,    // codec refused the frame
    ModulateFailed = 3,  // modulate_one refused
    TxResampleFailed = 4,
    QuantiseFailed = 5,  // NaN/Inf input or an SC16 contract violation
    RxResampleFailed = 6,
    DemodFailed = 7,     // demod returned a non-Success status
    FcsFailed = 8,       // decoded but FCS mismatch
    BytesMismatch = 9,   // decoded bytes differ from the input
    LengthMismatch = 10, // decoded PSDU length differs from expected
    CapacityExceeded = 11,
    InternalError = 12
};

inline const char* m2a_status_to_string(M2aStatus s)
{
    switch (s) {
    case M2aStatus::Ok:
        return "ok";
    case M2aStatus::InvalidConfig:
        return "invalid_config";
    case M2aStatus::InvalidFrame:
        return "invalid_frame";
    case M2aStatus::ModulateFailed:
        return "modulate_failed";
    case M2aStatus::TxResampleFailed:
        return "tx_resample_failed";
    case M2aStatus::QuantiseFailed:
        return "quantise_failed";
    case M2aStatus::RxResampleFailed:
        return "rx_resample_failed";
    case M2aStatus::DemodFailed:
        return "demod_failed";
    case M2aStatus::FcsFailed:
        return "fcs_failed";
    case M2aStatus::BytesMismatch:
        return "bytes_mismatch";
    case M2aStatus::LengthMismatch:
        return "length_mismatch";
    case M2aStatus::CapacityExceeded:
        return "capacity_exceeded";
    case M2aStatus::InternalError:
        return "internal_error";
    }
    return "invalid";
}

inline bool m2a_status_is_known(M2aStatus s)
{
    switch (s) {
    case M2aStatus::Ok:
    case M2aStatus::InvalidConfig:
    case M2aStatus::InvalidFrame:
    case M2aStatus::ModulateFailed:
    case M2aStatus::TxResampleFailed:
    case M2aStatus::QuantiseFailed:
    case M2aStatus::RxResampleFailed:
    case M2aStatus::DemodFailed:
    case M2aStatus::FcsFailed:
    case M2aStatus::BytesMismatch:
    case M2aStatus::LengthMismatch:
    case M2aStatus::CapacityExceeded:
    case M2aStatus::InternalError:
        return true;
    }
    return false;
}

// Frozen per-frame capacity bound (G0 §3.5 / review B5): N <= 2^20 on both the
// work and the native grid, which keeps the resampler core from re-reserving.
inline constexpr size_t kM2aMaxSamples = size_t(1) << 20;

// ===========================================================================
// The frozen profile + coefficients (G0 §1, §3.3)
// ===========================================================================
//
// Everything a run needs, in ONE immutable snapshot.  The frame bytes, the FCS
// owner, the modulation profile and the RX profile all come from here; no
// stage may invent its own constant.
struct M2aConfig {
    M2aNativeRate native_rate = M2aNativeRate::Uc200_737280000;
    M2aIqFormat iq_format = M2aIqFormat::Cf32;

    // Modulation profile (G0 §1).  Fixed for M2-A's minimal profile.
    size_t code_index = 9;
    size_t sync_repetitions = 64;
    std::string sfd_mode = "ieee";
    bool insert_sts = false; // STS is out of scope for M2-A
    bool ranging = true;     // sets the PHR ranging bit (this DOES change IQ)
    float peak_amplitude = 0.8f;

    // Resampler coefficients.  `tx_taps` for <L,65> (DC sum == L) and
    // `rx_taps` for <65,L> (DC sum == 65); the core never scales.
    std::vector<float> tx_taps;
    std::vector<float> rx_taps;

    // SC16 scale (G0 §7).  Defaults to 32767 / peak_amplitude.
    float sc16_scale = 0.0f;

    double tx_rate_hz() const { return demod::kQm35SampleRate; }
    double native_rate_hz() const { return m2a_rate_hz(native_rate); }

    // The ONE validity question.  Reports the first problem by name; never
    // throws.  A config that is wrong in several places still reports one
    // actionable reason, and every caller can rely on it being total.
    bool is_valid(std::string& why) const
    {
        if (!m2a_native_rate_is_known(native_rate)) {
            why = "native_rate is not a member of M2aNativeRate";
            return false;
        }
        if (!m2a_iq_format_is_known(iq_format)) {
            why = "iq_format is not a member of M2aIqFormat";
            return false;
        }
        if (insert_sts) {
            why = "insert_sts is out of scope for M2-A (no STS)";
            return false;
        }
        if (!(peak_amplitude > 0.0f) || !std::isfinite(peak_amplitude)) {
            why = "peak_amplitude must be finite and > 0";
            return false;
        }
        if (!radar_meta::code_index_supported(code_index)) {
            why = "code_index is not supported by this build";
            return false;
        }
        if (!radar_meta::sync_reps_supported(sync_repetitions)) {
            why = "sync_repetitions is not supported by this build";
            return false;
        }
        uint32_t l = 0;
        uint32_t m = 0;
        if (!m2a_rate_tx_lm(native_rate, l, m)) {
            why = "no TX ratio for this native rate";
            return false;
        }
        if (tx_taps.size() < 2u) {
            why = "tx_taps must hold at least 2 coefficients";
            return false;
        }
        if (rx_taps.size() < 2u) {
            why = "rx_taps must hold at least 2 coefficients";
            return false;
        }
        // DC sum contract: taps sum to Interp (effective gain 1).  A tolerance
        // is allowed because the coefficients are float32; it is tight enough
        // that a wrong-length or unscaled table is refused.
        const double tx_sum = m2a_sum(tx_taps);
        if (std::fabs(tx_sum - static_cast<double>(l)) > 1e-2 * static_cast<double>(l)) {
            why = "tx_taps DC sum is not Interp (" + std::to_string(tx_sum) + " vs " +
                  std::to_string(l) + ")";
            return false;
        }
        const double rx_sum = m2a_sum(rx_taps);
        if (std::fabs(rx_sum - 65.0) > 1e-2 * 65.0) {
            why = "rx_taps DC sum is not 65 (" + std::to_string(rx_sum) + ")";
            return false;
        }
        for (float t : tx_taps)
            if (!std::isfinite(t)) {
                why = "tx_taps contains a non-finite coefficient";
                return false;
            }
        for (float t : rx_taps)
            if (!std::isfinite(t)) {
                why = "rx_taps contains a non-finite coefficient";
                return false;
            }
        return true;
    }

    // Effective SC16 scale (G0 §7).
    float effective_sc16_scale() const
    {
        return sc16_scale > 0.0f ? sc16_scale : (32767.0f / peak_amplitude);
    }

    // The HRP modulation config this snapshot implies.
    mod::HrpModConfig hrp_config() const
    {
        mod::HrpModConfig c;
        c.code_index = code_index;
        c.sync_repetitions = sync_repetitions;
        c.sfd_mode = sfd_mode.c_str();
        c.insert_sts = insert_sts;
        c.peak_amplitude = peak_amplitude;
        c.ranging = ranging;
        return c;
    }

    // The demod profile this snapshot implies (code/sync/SFD must match TX).
    demod::Qm35825Profile demod_profile() const
    {
        demod::Qm35825Profile p;
        p.sample_rate = demod::kQm35SampleRate;
        p.code_index = code_index;
        p.preamble_repetitions = sync_repetitions;
        p.sfd_mode = sfd_mode.c_str();
        return p;
    }

private:
    static double m2a_sum(const std::vector<float>& v)
    {
        double s = 0.0;
        for (float x : v)
            s += static_cast<double>(x);
        return s;
    }
};

// ===========================================================================
// Per-stage sample-grid bookkeeping (G0 §5)
// ===========================================================================
//
// These are DIGITAL coordinates, not protocol RMARKERs and not hardware
// calibration constants.  Every stage records where its input began, how many
// samples it consumed and produced, and the valid output range.
struct M2aStageTrace {
    std::string name;
    // G0 A.2: every stage states its own UNIT, so a byte count is never read as
    // a sample count (or vice versa).
    std::string unit = "samples"; // "samples" | "bytes"
    double rate_hz = 0.0;
    uint32_t interp = 1;
    uint32_t decim = 1;
    int64_t origin = 0;      // 0-based input index of this stage, in `unit`
    size_t in_count = 0;
    size_t out_count = 0;
    uint32_t phase = 0;      // resampler phase at entry
    int64_t trim = 0;        // 0: causal full convolution, no crop
    // G0 A.2: the two padding components are recorded SEPARATELY; the sum must
    // not be used to guess the front offset.
    int64_t pad_front = 0;
    int64_t pad_back = 0;
    int64_t padding = 0;     // == pad_front + pad_back (convenience only)
    // Search window (demod stage): the guards it added and the ROI it searched.
    // Diagnostic coordinates must be rebased out of the guards.
    int64_t search_guard_front = 0;
    int64_t search_guard_back = 0;
    size_t search_roi_from = 0;
    size_t search_roi_to = 0;
    double filter_delay = 0.0; // 0.5*(T-1) in the INTERPOLATED domain
    size_t valid_from = 0;
    size_t valid_to = 0;     // exclusive

    std::string to_string() const;
};

// ===========================================================================
// Impairment (TEST HARNESS ONLY -- G0 A.3)
// ===========================================================================
//
// The channel model lives in the harness, never in the decoder's inputs as a
// truth.  It is applied on the NATIVE CF32 grid, between TX resampling and the
// (optional) RX SC16 quantisation.  `m2a_apply_impairment` is implemented in
// lib/uwb_twr_phy_impairment.cc and is NOT part of the production pipeline.
struct M2aImpairment {
    bool enabled = false;
    double cfo_hz = 0.0;            // phase advances with native sample index
    bool awgn_enabled = false;
    double awgn_snr_db = 0.0;       // over the SIGNAL VALID REGION (G0 A.3)
    uint64_t awgn_seed = 0;
    int64_t delay_int_samples = 0;  // whole native samples
    int32_t delay_frac_num = 0;     // sub-sample delay = num/den, 0 <= num < den
    uint32_t delay_frac_den = 1;
    // Complex taps; index 0 is the first path.  Empty == no multipath.
    std::vector<std::complex<float>> multipath;

    bool is_valid(std::string& why) const
    {
        if (!std::isfinite(cfo_hz))
            return why = "cfo_hz must be finite", false;
        if (awgn_enabled && !std::isfinite(awgn_snr_db))
            return why = "awgn_snr_db must be finite", false;
        if (delay_int_samples < 0)
            return why = "delay_int_samples must be >= 0", false;
        if (delay_frac_den == 0u)
            return why = "delay_frac_den must be > 0", false;
        if (delay_frac_num < 0 ||
            static_cast<uint32_t>(delay_frac_num) >= delay_frac_den)
            return why = "delay_frac_num must be in [0, den)", false;
        if (!multipath.empty() && multipath[0] == std::complex<float>(0.0f, 0.0f))
            return why = "multipath[0] (the first path) must not be zero", false;
        for (const std::complex<float>& t : multipath)
            if (!std::isfinite(t.real()) || !std::isfinite(t.imag()))
                return why = "multipath has a non-finite tap", false;
        return true;
    }

    // True when this is a no-op (all sub-impairments disabled).
    bool is_identity() const
    {
        return !enabled || (cfo_hz == 0.0 && !awgn_enabled && delay_int_samples == 0 &&
                            delay_frac_num == 0 && multipath.empty());
    }
};

// Apply the model in place on the native grid.  `native_rate_hz` sets the CFO
// phase increment.  Deterministic given `awgn_seed`.  Returns false with a
// reason on an invalid impairment or a non-finite result.
bool m2a_apply_impairment(std::vector<std::complex<float>>& iq, double native_rate_hz,
                          const M2aImpairment& imp, std::string& why);

// ===========================================================================
// The result (mirrors the `twr-m2a-native/1` manifest schema, G0 §8)
// ===========================================================================
struct M2aResult {
    bool ok = false;
    M2aStatus status = M2aStatus::Ok;

    M2aNativeRate native_rate = M2aNativeRate::Uc200_737280000;
    M2aIqFormat iq_format = M2aIqFormat::Cf32;

    // Frame bytes.  `mac_bytes` has NO FCS; `psdu_bytes` is mac + the single
    // 2-byte FCS the HRP layer appends.
    uint8_t function_code = 0;
    std::vector<uint8_t> mac_bytes;
    std::vector<uint8_t> psdu_bytes;
    uint16_t fcs = 0;

    // Sample counts.
    size_t work_tx_samples = 0;
    size_t native_samples = 0;
    size_t work_rx_samples = 0;
    size_t sc16_saturated = 0;

    // Decode outcome.
    demod::DemodStatus demod_status = demod::DemodStatus::InvalidInput;
    bool fcs_pass = false;
    size_t expected_psdu_length = 0;
    std::vector<uint8_t> decoded_bytes;
    // DIAGNOSTIC ONLY: decode coordinates, never a RMARKER / first path.
    int64_t sfd_start_sample = -1;
    int64_t packet_start_sample = -1;

    // Comparison against the input.
    bool bytes_exact = false;
    double max_abs_error = 0.0;
    double relative_l2 = 0.0;

    std::vector<M2aStageTrace> stages;

    // Evidence boundary.  `measurement_valid` is ALWAYS false in M2-A: a native
    // round trip is a protocol/waveform estimate, not a hardware measurement.
    bool measurement_valid = false;
    std::string detail;

    std::string to_string() const;
};

// ===========================================================================
// Inline helpers (total, no I/O, no allocation surprises)
// ===========================================================================

// SFD symbol count for a mode string; 0 for an unknown mode.
inline size_t m2a_sfd_symbols(const std::string& sfd_mode)
{
    return demod::GetSfdSequence(sfd_mode.c_str()).size();
}

// Exact work-grid waveform length of one modulated frame (G0 §1.3).
// Returns 0 when the config or the mode is unusable.
inline size_t m2a_expected_work_samples(size_t psdu_bytes, const M2aConfig& cfg)
{
    const size_t n_sfd = m2a_sfd_symbols(cfg.sfd_mode);
    if (n_sfd == 0)
        return 0;
    if (psdu_bytes > radar_meta::kMaxPsduBytes)
        return 0;
    return mod::packet_samples_998p4(cfg.sync_repetitions, n_sfd, psdu_bytes,
                                     cfg.insert_sts, nullptr);
}

// Exact resampler output length for the CAUSAL FULL CONVOLUTION contract
// (G0 §3.1): Lout = ceil(((N-1)*L + T)/M), with N == 0 defined as 0 here.
inline size_t m2a_resampled_length(size_t n, size_t taps, uint32_t l, uint32_t m)
{
    if (l == 0 || m == 0 || taps == 0)
        return 0;
    if (n == 0)
        return 0; // G0 §3.2: no input -> no output, no transient
    const int64_t num = (static_cast<int64_t>(n) - 1) * static_cast<int64_t>(l) +
                        static_cast<int64_t>(taps);
    if (num <= 0)
        return 0;
    return static_cast<size_t>((num + static_cast<int64_t>(m) - 1) /
                               static_cast<int64_t>(m));
}

// ===========================================================================
// Stages
// ===========================================================================
//
// Each stage is bounded and total: it never throws, never reads or writes
// outside its buffers, and reports the first problem through `why`.  A stage
// that fails leaves `out` unspecified; the caller must not use it.
//
// The stages exist as public entry points so the QA can drive them
// individually (G0 A03/A04/A05) and so an independent verifier can recompute
// the expected values WITHOUT calling the closed loop.

// codec MAC bytes (no FCS) -> append_ieee_fcs once -> modulate_one -> work IQ.
bool m2a_modulate_to_work(const uint8_t* mac_bytes, size_t mac_len,
                          const M2aConfig& cfg, std::vector<std::complex<float>>& out,
                          M2aStageTrace& trace, std::string& why);

// work IQ -> native IQ (TX: <L,65>).
bool m2a_tx_resample(const std::complex<float>* work, size_t n, const M2aConfig& cfg,
                     std::vector<std::complex<float>>& native, M2aStageTrace& trace,
                     std::string& why);

// native IQ -> work IQ (RX: <65,L>).
bool m2a_rx_resample(const std::complex<float>* native, size_t n, const M2aConfig& cfg,
                     std::vector<std::complex<float>>& work, M2aStageTrace& trace,
                     std::string& why);

// CF32 -> SC16 -> CF32 with the frozen scale (G0 §7).  NaN/Inf input is
// refused; saturation is counted, never silently accepted as high quality.
bool m2a_sc16_roundtrip(const std::complex<float>* in, size_t n, float scale,
                        std::vector<std::complex<float>>& out, size_t& saturated,
                        std::string& why);

// work IQ -> demodulate_one -> DemodResult (real PHR / payload / FCS).
bool m2a_demod_work(const std::complex<float>* work, size_t n, const M2aConfig& cfg,
                    demod::DemodResult& out, M2aStageTrace& trace, std::string& why);

// ===========================================================================
// Prepared context (G0 A.1)
// ===========================================================================
//
// The one-shot `m2a_native_roundtrip` below is a COLD wrapper: it builds every
// core/scratch and runs one frame.  A benchmark or a soak must instead prepare
// once and reuse, so `M2aContext` freezes the profile/taps/capacity/format and
// owns the TX/RX resamplers, the modulation scratch, the demod scratch, the
// demod template and a reusable search workspace.
//
// Rules (G0 A.1): not a concurrent shared object (one per worker/endpoint);
// `prepare()` is atomic; capacity changes happen only in `prepare()`; the
// per-frame path must not reserve/resize; every guard/tail/impairment length
// counts toward kM2aMaxSamples; a failed frame leaves the context usable and
// `reset()` clears filter history and the previous frame's bytes.
class M2aContext {
public:
    M2aContext();
    ~M2aContext();
    M2aContext(const M2aContext&) = delete;
    M2aContext& operator=(const M2aContext&) = delete;

    // Atomic: on failure the context stays unusable and is not half-updated.
    bool prepare(const M2aConfig& cfg, std::string& why);
    bool prepared() const;

    // Per-frame.  No reallocation of the owned cores/scratch.  A failure leaves
    // the context usable for the next frame.
    bool run(const Frame& frame, const FrameProfile& profile, M2aResult& out,
             std::string& why);

    // Between independent bursts: clears filter history, phase and the previous
    // frame's bytes WITHOUT rebuilding the cores.
    void reset();

    const M2aConfig& config() const;

private:
    struct Impl;
    Impl* d_impl;
};

// ===========================================================================
// The closed loop
// ===========================================================================
//
// Encodes `frame` with `profile`, runs the full chain above, and reports
// whether the bytes survived.  `ok` is true ONLY under the G0 §6 / review B3
// criterion:
//
//     demod status == Success AND fcs_pass AND
//     decoded PSDU length == expected AND bytes byte-exact
//
// Any payload FALLBACK candidate, FcsFailed, or length mismatch yields
// ok == false with the corresponding M2aStatus.  The result never claims a
// distance, a RMARKER, a first path, or hardware evidence.
bool m2a_native_roundtrip(const Frame& frame, const FrameProfile& profile,
                          const M2aConfig& cfg, M2aResult& out, std::string& why);

} // namespace twr
} // namespace uwb
} // namespace gr

#endif /* INCLUDED_GNURADIO_UWB_UWB_TWR_PHY_H */
