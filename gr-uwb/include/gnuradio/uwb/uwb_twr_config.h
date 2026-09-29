/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * TWR configuration, capability whitelist, validator, frozen effective
 * config and JSON import/export (M0 / REQ-API-01..05).
 *
 * This header is header-only and dependency-free: it includes NO GNU Radio
 * header, NO UHD header and NO pmt header, so the whole configuration
 * surface can be unit tested, fuzzed and imported from Python without a
 * radio, without a device and without the OOT module being built.  The
 * only project dependencies are the frozen vocabulary header
 * <gnuradio/uwb/uwb_twr_types.h> and the frame codec
 * <gnuradio/uwb/uwb_twr_frame.h>.  Both are themselves stdlib-only, and the
 * dependency runs ONE WAY: the codec does not know this file exists.
 *
 * ---------------------------------------------------------------------------
 * THE FRAME CODEC IS THE GEOMETRY AUTHORITY (M0.1 / R3)
 * ---------------------------------------------------------------------------
 * M0 review R3 found TWO geometries for one format: this schema carried a
 * pre-codec "7-byte header" FrameGeometry with 9/19/24-byte budgets, while
 * uwb_twr_frame.h has emitted a 14-byte header and 16/26/31-byte on-air frames
 * since it was frozen.  Both test suites passed, because nothing ever asked
 * whether an ACCEPTED configuration can losslessly build the real frame.
 *
 * The dependency now points in the only direction that can be wrong in one
 * place instead of two:
 *
 *   - `FrameFormatConfig::geometry` is a CLAIM -- "this is what I believe the
 *     layout to be".  It is checked field by field through
 *     `frame_geometry_check()` / `frame_geometry_for()`, and every
 *     disagreement becomes a rejection naming that exact field.  It never
 *     decides anything.
 *   - Every byte count the config reports -- the PSDU budget, the on-air
 *     lengths in `effective_config()`, `frame_psdu_bytes()` -- is a delegation
 *     to the codec.  There is no second `header + n * timestamp` sum anywhere
 *     in this file, and no test fixture keeps its own copy of the numbers.
 *   - `FrameFormatConfig::frame_profile` is a `FrameProfileId`, not a
 *     free-form string, so "an arbitrary profile name is an executable
 *     geometry" is unrepresentable rather than merely discouraged.
 *
 * ---------------------------------------------------------------------------
 * THE ONE RULE THIS FILE EXISTS TO ENFORCE
 * ---------------------------------------------------------------------------
 * REQ-SCOPE-01: an unknown or unsupported combination must be REJECTED
 * with a specific, machine-readable reason.  It must never be silently
 * replaced by the radar / QM35 default.  Concretely:
 *
 *   - there is no "fill in the blanks" code path anywhere in this file;
 *   - `validate()` returns a LIST of violations, each carrying a field
 *     path, a `ConfigReason`, an `ExchangeStatus` classification and a
 *     human message -- never a bare bool;
 *   - the capability whitelist (`capabilities()`) is DEFAULT DENY: a
 *     combination that is not present in it is rejected with
 *     `ExchangeStatus::Unsupported`;
 *   - TX gain in dB, digital IQ amplitude and calibrated TX power in dBm
 *     are three DIFFERENT named fields.  A vendor "power word" is a
 *     fourth, separately named field that may only be set together with
 *     the backend capability id that defines its meaning;
 *   - application payload, MAC PSDU and "does the MAC PSDU already
 *     include the FCS" are three different concepts, and exactly one
 *     layer may append the FCS.
 *
 * ---------------------------------------------------------------------------
 * WHAT IS *NOT* CLAIMED HERE
 * ---------------------------------------------------------------------------
 * The shipped `capabilities()` whitelist contains ONLY facts that are
 * already established elsewhere in this repository or in the frozen TWR
 * requirements:
 *
 *   MEASURED (from existing code)
 *     - native sample rate: 737280000 or 491520000
 *       (mirrors uhd::is_allowed_uhd_native_rate(); qa_uwb_twr_config
 *       cross-checks the two implementations stay identical)
 *     - work rate 998.4e6, 1016 work samples per symbol
 *       (twr::kTwrWorkSampleRateHz / kTwrWorkSamplesPerSymbol)
 *     - preamble code indices 9..12
 *       (mirrors radar_meta::code_index_supported())
 *     - max PSDU 127 bytes (mirrors radar_meta::kMaxPsduBytes)
 *
 *   MEASURED-PENDING (see testdata/twr/phy_matrix_*.csv)
 *     - preamble SYNC repetitions, SFD mode, data rate per (native rate,
 *       code) combination.  The corresponding whitelist rows are EMPTY,
 *       so the validator rejects them until the measured matrix lands.
 *     - qa_uwb_twr_config proves this default-deny behaviour, and also
 *       proves a config DOES validate against a whitelist the caller
 *       installs through the public `Capabilities` builder.  That is the
 *       mechanism the measured matrix will be wired into.
 *
 *   BY DEFINITION (from the frozen TWR requirement, not a measurement)
 *     - STS is out of scope for phase 1 (REQ-SCOPE-04, explicit user
 *       decision): `sts_mode != Off` is always rejected as Unsupported.
 *     - a TWR MAC frame must set the ranging bit; ranging_bit == false
 *       is not a valid TWR profile.
 *
 * No vendor part, no DW UUS value, no module capability and no measured
 * timing budget is asserted anywhere in this file.
 *
 * Requirements traceability (docs/twr/需求_UWB_SS_DS_TWR.md):
 *   REQ-SCOPE-01  unsupported combinations are rejected, never defaulted
 *   REQ-SCOPE-04  no STS in phase 1
 *   REQ-PHY-01    common profile is ch5 / 6489.6 MHz / 64 MHz PRF class;
 *                 the SYNC lengths, SFD modes, PHR rate and data rate are
 *                 limited by what THIS software implements, and a refusal
 *                 says so rather than claiming a chip cannot do it
 *   REQ-PHY-02    native rate, work rate, symbol rate, PHR rate and RF
 *                 bandwidth are five different quantities
 *   REQ-PHY-03    channel 5 first; other channels are per-channel claims
 *   REQ-PROTO-01  a session id that does not fit the 16-bit wire field is
 *                 refused, never narrowed
 *   REQ-PROTO-06  one frame profile, one geometry authority, one set of
 *                 numbers; a claim that disagrees is rejected per field
 *   REQ-TIME-04   the timestamp width is a profile parameter, not a
 *                 project-wide constant
 *   REQ-API-01    typed config, one validator, capabilities() /
 *                 validate() / effective_config(), versions, requested
 *                 vs effective, readback, startup rejection
 *   REQ-API-02    reply delay / post-TX RX enable / RX timeout are three
 *                 distinct things, each with a reference event
 *   REQ-API-03    per-message overrides take effect at exchange
 *                 boundaries on an immutable snapshot
 *   REQ-ERR-01    every rejection maps onto the frozen status taxonomy
 *   REQ-OUT-01    64-bit fields keep integer precision through JSON
 *   REQ-GR-03     bounded queues / diagnostic buffers
 *   REQ-GR-04     reply-delay budget feasibility is checked, not assumed
 *   REQ-CAL-01    calibration is versioned and applicable exactly once
 *   REQ-QA-01/02  targeted QA, no self-generated ground truth
 */

#ifndef INCLUDED_GNURADIO_UWB_UWB_TWR_CONFIG_H
#define INCLUDED_GNURADIO_UWB_UWB_TWR_CONFIG_H

// The frame codec: this file's geometry/session authority (M0.1 / R3).  It is
// stdlib-only, so including it keeps the "no GNU Radio, no UHD" property.
#include <gnuradio/uwb/uwb_twr_frame.h>
#include <gnuradio/uwb/uwb_twr_types.h>
// M0.1 R7: the per-row evidence ladder.  uwb_twr_capability_evidence.h does
// NOT include this file, so the dependency is one-way.
#include <gnuradio/uwb/uwb_twr_capability_evidence.h>

// File-scope alias so the evidence vocabulary reads as `evidence::...` in the
// capability tables below.  C++ forbids a namespace alias as a *member*, so it
// must live at namespace scope.
namespace evidence = ::gr::uwb::twr::evidence;

#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace gr {
namespace uwb {
namespace twr {

// ===========================================================================
// 0. Small self-contained utilities
// ===========================================================================

// Presence marker.  Used for every field whose "unset" state must survive a
// JSON round trip and must NOT be confused with a sentinel value (REQ-API-01
// "unknown or unsupported must be rejected", so "absent" can never mean
// "0 dB" or "no gain change").
template <typename T>
class Opt
{
public:
    Opt() = default;
    Opt(T v) : d_value(v), d_has(true) {}

    static Opt none() { return Opt(); }
    static Opt some(T v) { return Opt(v); }

    bool has() const { return d_has; }
    explicit operator bool() const { return d_has; }

    // Throws when absent: reading an absent field is a programming error,
    // never a silent default.  Config-time only, never on a work thread.
    T value() const
    {
        if (!d_has)
            throw std::logic_error("gr::uwb::twr::Opt: no value");
        return d_value;
    }
    T value_or(T fallback) const { return d_has ? d_value : fallback; }

    Opt& operator=(T v)
    {
        d_value = v;
        d_has = true;
        return *this;
    }
    void reset()
    {
        d_value = T();
        d_has = false;
    }

    bool operator==(const Opt& o) const
    {
        return d_has == o.d_has && (!d_has || d_value == o.d_value);
    }
    bool operator!=(const Opt& o) const { return !(*this == o); }

private:
    T d_value{};
    bool d_has = false;
};

// IEEE 754 JSON-safe integer limit (2^53 - 1).  Larger integers are written
// as decimal STRINGS (REQ-OUT-01).
inline constexpr int64_t kTwrJsonMaxSafeInteger = 9007199254740991LL;

inline std::string twr_double_to_text(double v)
{
    // %.17g round-trips every finite IEEE 754 double exactly.
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%.17g", v);
    return std::string(buf);
}

inline std::string twr_int_to_text(int64_t v) { return std::to_string(v); }

inline std::string twr_bool_to_text(bool v) { return v ? "true" : "false"; }

inline std::string twr_duration_to_text(const Duration& d)
{
    return twr_int_to_text(d.nanos()) + "ns";
}

// Mirrors uhd::is_allowed_uhd_native_rate() (gr-uwb/include/gnuradio/uwb/
// uwb_uhd_backend_config.h).  Re-implemented here because that header pulls
// in GNU Radio; qa_uwb_twr_config cross-checks the two agree on a table of
// values, so the duplication cannot silently drift.
inline constexpr double kTwrNativeRateUc200Hz = 737280000.0; // X410/UC200
inline constexpr double kTwrNativeRateCg400Hz = 491520000.0; // CG400
inline constexpr double kTwrNativeRateToleranceRel = 1e-9;

inline bool twr_rate_matches_strict(double requested, double readback, double rel_tol)
{
    if (!(rel_tol > 0.0) || !std::isfinite(requested) || !std::isfinite(readback))
        return false;
    const double scale = requested > readback ? requested : readback;
    return std::fabs(readback - requested) <= rel_tol * scale;
}

// REQ-PHY-02: the native device rate is a DIFFERENT quantity from the
// 998.4 MS/s work rate, the symbol rate and the RF bandwidth.
inline bool is_allowed_native_rate(double hz)
{
    return twr_rate_matches_strict(kTwrNativeRateUc200Hz, hz,
        kTwrNativeRateToleranceRel) ||
           twr_rate_matches_strict(kTwrNativeRateCg400Hz, hz,
               kTwrNativeRateToleranceRel);
}

// Mirrors radar_meta::code_index_supported() (gr-uwb/include/gnuradio/uwb/
// uwb_radar_pdu_meta.h).  Cross-checked by qa_uwb_twr_config.
inline constexpr uint8_t kTwrCodeIndexMin = 9;
inline constexpr uint8_t kTwrCodeIndexMax = 12;
inline bool code_index_supported(uint8_t n)
{
    return n >= kTwrCodeIndexMin && n <= kTwrCodeIndexMax;
}

// Mirrors radar_meta::kMaxPsduBytes.
inline constexpr uint16_t kTwrMaxPsduBytes = 127;

// Phase-1 hard limit (REQ-SCOPE-01 "同端初版一次仅一个在途 exchange").
inline constexpr uint32_t kTwrMaxInFlightExchanges = 1;

// Diagnostics memory bound (REQ-GR-03: bounded buffers).
inline constexpr uint64_t kTwrMaxDiagnosticBytes = 64ull * 1024ull * 1024ull;
inline constexpr uint32_t kTwrMaxQueueEntries = 1u << 20;
inline constexpr uint32_t kTwrMaxMeasurementCount = 1000000u;
inline constexpr uint32_t kTwrMaxAttemptsPerExchange = 16u;
inline constexpr uint32_t kTwrMaxPhysicalChannels = 16u;
inline constexpr size_t kTwrMaxPeersPerConfig = 4u;

// Quantisation rates a `TimedField` may name.  Anything else (in particular
// any rate finer than 500 MHz, i.e. finer than half a nanosecond) is
// rejected: the device tick cannot express it (REQ-API-01 计时精度不足).
inline bool is_allowed_quantisation_hz(double hz)
{
    if (!std::isfinite(hz) || hz <= 0.0)
        return false;
    if (hz == 1.0e9) // integer nanoseconds
        return true;
    if (hz == kTwrWorkSampleRateHz)
        return true;
    return is_allowed_native_rate(hz);
}

// ---------------------------------------------------------------------------
// Structural enums (string round-trippable, appended-only values)
// ---------------------------------------------------------------------------

// PRF CLASS, not a rate: 802.15.4a pairs a class with a channel.  The
// existing code base measures 62.4 MHz mean PRF for its BPRF geometry
// (kTwrExistingMeanPrfHz); the nominal HRP figure is 64 MHz
// (kTwrNominalMeanPrfHz).  Neither may be silently substituted for the other.
enum class PrfClass : uint8_t {
    Bprf64 = 0, // <= 500 MHz channels, 64 MHz nominal mean PRF
    Hprf64 = 1, // > 500 MHz channels, 64 MHz nominal mean PRF
    Hprf400 = 2 // > 500 MHz channels, 400 MHz nominal mean PRF
};
inline const char* prf_class_to_string(PrfClass v)
{
    switch (v) {
    case PrfClass::Bprf64:
        return "bprf64";
    case PrfClass::Hprf64:
        return "hprf64";
    case PrfClass::Hprf400:
        return "hprf400";
    }
    return "invalid";
}
inline bool prf_class_from_string(const std::string& s, PrfClass& out)
{
    if (s == "bprf64") {
        out = PrfClass::Bprf64;
        return true;
    }
    if (s == "hprf64") {
        out = PrfClass::Hprf64;
        return true;
    }
    if (s == "hprf400") {
        out = PrfClass::Hprf400;
        return true;
    }
    return false;
}

// Data-rate CLASS.  Whether any of these actually round-trips on this build
// is a MEASURED-PENDING question; the shipped whitelist is empty.
enum class DataRate : uint8_t {
    R850k = 0,
    R6p8M = 1,
    R27M = 2,
    R7p8M = 3,
    R27p2M = 4,
    R6p8M_hprf = 5
};
inline const char* data_rate_to_string(DataRate v)
{
    switch (v) {
    case DataRate::R850k:
        return "850k";
    case DataRate::R6p8M:
        return "6p8m";
    case DataRate::R27M:
        return "27m";
    case DataRate::R7p8M:
        return "7p8m";
    case DataRate::R27p2M:
        return "27p2m";
    case DataRate::R6p8M_hprf:
        return "6p8m_hprf";
    }
    return "invalid";
}
inline bool data_rate_from_string(const std::string& s, DataRate& out)
{
    if (s == "850k") {
        out = DataRate::R850k;
        return true;
    }
    if (s == "6p8m") {
        out = DataRate::R6p8M;
        return true;
    }
    if (s == "27m") {
        out = DataRate::R27M;
        return true;
    }
    if (s == "7p8m") {
        out = DataRate::R7p8M;
        return true;
    }
    if (s == "27p2m") {
        out = DataRate::R27p2M;
        return true;
    }
    if (s == "6p8m_hprf") {
        out = DataRate::R6p8M_hprf;
        return true;
    }
    return false;
}

// ---------------------------------------------------------------------------
// PHR RATE -- a SEPARATE quantity from the payload data rate (M0.1 / R2)
// ---------------------------------------------------------------------------
// WHY THIS IS NOT A DataRate ANY MORE
// ------------------------------------
// M0 typed `PhyConfig::phr_rate` as a DataRate and then required it to EQUAL
// `PhyConfig::data_rate`.  That rule was the exact inverse of what this
// repository transmits:
//
//   uwb_hrp_mod_core.h:8-12   "BPM-BPSK 0.85 Mb/s PHR + 6.81 Mb/s payload"
//   kPhrSymbols = 21, kPhrChipsPerSymbol = 512              (the PHR)
//   kPayloadChipsPerBurst = 8, kPayloadChipsPerSymbol = 64  (the payload)
//
// The PHR is a different modulation at a different rate.  The 2-bit data-rate
// field that lives INSIDE the PHR describes the PAYLOAD that follows it; it
// says nothing about how the PHR itself is transmitted.  Qorvo's API guides
// keep the two as separate fields for the same reason.
//
// So the M0 rule accepted a 6.81 Mb/s PHR -- which this modulator cannot
// produce -- and rejected the 0.85 Mb/s PHR it does produce.  A dedicated enum
// is the fix: the two axes are separately typed, separately named and
// separately validated, and the value domain is the set of PHR rates the
// modulator actually implements rather than the set of payload rates.
enum class PhrRate : uint8_t {
    // 851.2 kb/s, the fixed PHR rate of an HRP BPRF profile.  THIS IS THE ONE
    // uwb_hrp_mod_core.h modulates: 21 SEC-DED-coded symbols of 512 chips.
    Standard850k = 0,
    // "Transmit the PHR at the payload data rate."  A legal option in IEEE
    // 802.15.4 and an explicit enum in the Qorvo DW3xxx API guide.
    //
    // IT IS REJECTED HERE, and the reason is a limit of THIS IMPLEMENTATION,
    // not of the standard and not of any chip: the modulator has exactly one
    // PHR code path and it is the 512-chips-per-symbol one.  There is no
    // second path to select.  When one is added and measured this member
    // becomes supported -- it is an enum member precisely so the config
    // surface can NAME the option it does not implement, instead of encoding
    // the wish as a payload rate.
    SameAsData = 1
};

inline const char* phr_rate_to_string(PhrRate v)
{
    switch (v) {
    case PhrRate::Standard850k:
        return "850k";
    case PhrRate::SameAsData:
        return "same_as_data";
    }
    return "invalid";
}

// Strict: one accepted spelling per member, nothing else.  A payload-rate name
// such as "6p8m" is NOT accepted here; that string belongs to DataRate, and
// asking for it as a PHR rate is exactly the confusion this enum removes.
inline bool phr_rate_from_string(const std::string& s, PhrRate& out)
{
    if (s == "850k") {
        out = PhrRate::Standard850k;
        return true;
    }
    if (s == "same_as_data") {
        out = PhrRate::SameAsData;
        return true;
    }
    return false;
}

// The complete set, so a validator can enumerate what it may offer.
inline constexpr PhrRate kAllPhrRates[] = { PhrRate::Standard850k, PhrRate::SameAsData };
inline constexpr size_t kPhrRateCount = sizeof(kAllPhrRates) / sizeof(kAllPhrRates[0]);

// True only for a PHR rate this build can actually transmit.  A member that is
// a legitimate protocol option with no implementation is refused.
inline bool phr_rate_is_implemented(PhrRate v) { return v == PhrRate::Standard850k; }

// PHR symbols per frame of the implemented PHR: 21 SEC-DED(13,13)-coded
// symbols carrying 13 information bits (mod::kPhrSymbols).  0 for a rate with
// no implementation, so it can never be mistaken for a real budget.
inline uint16_t phr_rate_symbols(PhrRate v)
{
    return phr_rate_is_implemented(v) ? 21u : 0u;
}

// Why a PHR rate member is refused, in its own words.  Every member that is
// not implemented has its own reason here, so no rejection of this field is
// ever a bare "unsupported" with nothing behind it.
inline std::string phr_rate_unsupported_reason(PhrRate v)
{
    switch (v) {
    case PhrRate::Standard850k:
        return "the 851.2 kb/s PHR is the rate uwb_hrp_mod_core.h modulates";
    case PhrRate::SameAsData:
        return "phr_rate=same_as_data is a legal 802.15.4 / DW3xxx API option, but this build "
               "has exactly ONE PHR code path and it is the fixed 0.85 Mb/s one (21 symbols of "
               "512 chips, uwb_hrp_mod_core.h kPhrSymbols / kPhrChipsPerSymbol). There is no "
               "same-as-data PHR implementation to select, so the request is refused rather than "
               "silently transmitted as a 0.85 Mb/s PHR. This is a limit of the software, not a "
               "claim about any Qorvo part.";
    }
    return "phr_rate " + std::to_string(static_cast<unsigned>(v)) +
           " is not a member of the PHR-rate enumeration";
}

// ---------------------------------------------------------------------------
// PREAMBLE (SYNC) LENGTH -- the lengths THIS decoder / CFO path implements
// ---------------------------------------------------------------------------
// A SYNC repetition count, named.  `PreambleLength` is deliberately an
// enumeration of what the current software can RUN, not of what the standard
// or any vendor part can do:
//
//   * Qorvo's own API accepts 64 / 128 / 256 / 512 / 1024 / 2048, and the
//     DW3xxx guide lists 128 / 256 / 512 as non-standard-but-supported
//     preamble lengths.  None of that is in dispute.
//   * What limits THIS build is the demodulator.  The CFO re-measurement in
//     uwb_demod_core.h re-fits only the last max(cfo_min_fit_repetitions, 40)
//     SYNCs and synthesises the earlier peaks at phase 0, so the number of
//     zero-phase points that leak into the least-squares fit is
//     max(0, reps - 64) and 64 is the first clean length.  Separately,
//     mod::encode_phr19 maps 128 / 256 / 512 to preamble-duration index 1
//     (= 64), so the PHR does not self-describe those lengths.
//   * 16 is the other measured-clean length (its tail covers the whole
//     preamble, so no zero-phase point reaches the fit) and it is
//     self-describing (index 0 = 16) -- but its first-path / ToA accuracy is
//     still unmeasured, which is a separate caveat carried by
//     kTwrSyncRepsNeedingToaValidation.
//   * 128 and everything longer are therefore refused HERE, with a reason
//     that names this decoder/CFO path.  Writing "the chip cannot do 128" would
//     be a false statement about hardware this project has never spoken to.
//
// 128 is NOT a member of this enumeration, deliberately, for this task: adding
// it means the CFO-fit and PHR self-description work has been done AND
// measured, and then the member is added with that evidence attached.
enum class PreambleLength : uint8_t {
    Sym16 = 0,
    Sym64 = 1
};

inline const char* preamble_length_to_string(PreambleLength v)
{
    switch (v) {
    case PreambleLength::Sym16:
        return "16";
    case PreambleLength::Sym64:
        return "64";
    }
    return "invalid";
}

inline bool preamble_length_from_string(const std::string& s, PreambleLength& out)
{
    if (s == "16") {
        out = PreambleLength::Sym16;
        return true;
    }
    if (s == "64") {
        out = PreambleLength::Sym64;
        return true;
    }
    return false;
}

inline constexpr PreambleLength kAllPreambleLengths[] = { PreambleLength::Sym16,
                                                          PreambleLength::Sym64 };
inline constexpr size_t kPreambleLengthCount =
    sizeof(kAllPreambleLengths) / sizeof(kAllPreambleLengths[0]);

// The symbols() accessor.  Named `preamble_length_symbols()` to match this
// file's own `sfd_mode_symbols()` convention, which is what the raw
// `phy.preamble_symbols` integer is checked against.
inline uint16_t preamble_length_symbols(PreambleLength v)
{
    switch (v) {
    case PreambleLength::Sym16:
        return 16;
    case PreambleLength::Sym64:
        return 64;
    }
    return 0;
}

// The reverse mapping, for the raw `phy.preamble_symbols` field.  False means
// "not a length this build implements" -- never "round to the nearest".
inline bool preamble_length_from_symbols(uint16_t symbols, PreambleLength& out)
{
    for (size_t i = 0; i < kPreambleLengthCount; ++i) {
        const PreambleLength v = kAllPreambleLengths[i];
        if (preamble_length_symbols(v) == symbols) {
            out = v;
            return true;
        }
    }
    return false;
}

// Why a SYNC repetition count is refused.  Each refused length gets its own
// measured reason, and every one of them names a limit of THIS SOFTWARE
// rather than of a chip: the Qorvo API accepts 64 through 2048, so "the
// hardware cannot do 128" would be a false statement.
inline std::string preamble_length_unsupported_reason(uint16_t symbols)
{
    const std::string n = std::to_string(symbols);
    if (symbols == 32)
        return "preamble length " + n +
               " SYNC is refused: mod::encode_phr19 advertises preamble-duration index 1 (= 64) "
               "for it, so the PHR does not self-describe a 32-symbol preamble";
    if (symbols == 128 || symbols == 256 || symbols == 512)
        return "preamble length " + n +
               " SYNC is refused by the CURRENT DEMODULATOR, not by any hardware: "
               "uwb_demod_core.h re-measures only the last max(cfo_min_fit_repetitions, 40) SYNCs "
               "and synthesises the earlier peaks at phase 0, so the zero-phase points that leak "
               "into the least-squares CFO fit number max(0, reps - 64) -- measured at 128 SYNC as "
               "an injected 20 kHz offset coming back as 3791 Hz; and mod::encode_phr19 maps " + n +
               " to preamble-duration index 1 (= 64), so the PHR does not self-describe it. "
               "Qorvo's API accepts these lengths (DW3xxx lists 128/256/512 as non-standard "
               "preamble lengths), so this is a limit of this software path, not a statement about "
               "what a DW1000 or DW3000 can transmit";
    if (symbols == 1024)
        return "preamble length " + n +
               " SYNC is PHR-legal (preamble-duration index 2) but refused by the current "
               "demodulator for the same CFO-fit bias as 128/256/512; a software limit, not a "
               "hardware one";
    if (symbols == 2048)
        return "preamble length " + n +
               " SYNC exceeds the work-grid TX buffer this build sizes for one frame "
               "(mod::kMaxHrpTxSamples); a length this large is a buffer and scheduling "
               "decision, not a hardware capability statement";
    if (symbols == 1 || symbols == 2)
        return "preamble length " + n +
               " SYNC leaves fewer than the 4 measured peaks uwb_demod_core.h stage_cfo needs";
    if (symbols == 4 || symbols == 8)
        return "preamble length " + n +
               " SYNC is shorter than the 10 initial repetitions uwb_demod_core.h skips before "
               "the CIR search";
    return "preamble length " + n +
           " is not a SYNC repetition count this build implements: the enumeration is 16 or 64, "
           "bounded by this repository's demodulator/CFO path (see the PreambleLength comment). "
           "That is a software limit; the Qorvo parts' own API accepts 64 through 2048";
}

enum class SfdMode : uint8_t {
    R4z1 = 0,
    R4z2 = 1,
    R4z3 = 2,
    R4z4 = 3,
    Dwt8 = 4,
    Ieee8 = 5
};
inline const char* sfd_mode_to_string(SfdMode v)
{
    switch (v) {
    case SfdMode::R4z1:
        return "4z1";
    case SfdMode::R4z2:
        return "4z2";
    case SfdMode::R4z3:
        return "4z3";
    case SfdMode::R4z4:
        return "4z4";
    case SfdMode::Dwt8:
        return "decawave";
    case SfdMode::Ieee8:
        return "ieee";
    }
    return "invalid";
}
inline bool sfd_mode_from_string(const std::string& s, SfdMode& out)
{
    for (int i = 0; i <= static_cast<int>(SfdMode::Ieee8); ++i) {
        SfdMode c = static_cast<SfdMode>(i);
        if (s == sfd_mode_to_string(c)) {
            out = c;
            return true;
        }
    }
    return false;
}
// SYMBOL count of each SFD (the sequence lengths in demod::GetSfdSequence).
inline uint16_t sfd_mode_symbols(SfdMode v)
{
    switch (v) {
    case SfdMode::R4z1:
        return 4;
    case SfdMode::R4z2:
        return 8;
    case SfdMode::R4z3:
        return 16;
    case SfdMode::R4z4:
        return 32;
    case SfdMode::Dwt8:
    case SfdMode::Ieee8:
        return 8;
    }
    return 0;
}

enum class PhrMode : uint8_t {
    Standard = 0,
    Extended = 1,
    None = 2
};
inline const char* phr_mode_to_string(PhrMode v)
{
    switch (v) {
    case PhrMode::Standard:
        return "standard";
    case PhrMode::Extended:
        return "extended";
    case PhrMode::None:
        return "none";
    }
    return "invalid";
}
inline bool phr_mode_from_string(const std::string& s, PhrMode& out)
{
    if (s == "standard") {
        out = PhrMode::Standard;
        return true;
    }
    if (s == "extended") {
        out = PhrMode::Extended;
        return true;
    }
    if (s == "none") {
        out = PhrMode::None;
        return true;
    }
    return false;
}

// The PHR PRESENCE/FORM is a different axis from the PHR RATE above: this says
// whether a PHR is transmitted at all and which PHR layout carries it, while
// PhrRate says at what rate.  Both axes were present in M0; what was missing
// is that each member gets its own reason, instead of only `none` being caught
// while `extended` passed validation silently.
//
// Only `standard` is implemented.  `extended` is a vendor PHR layout (a
// header longer than the 13 information bits frame v1 encodes) that this codec
// and this modulator have no layout for; `none` is not a UWB frame at all,
// because the PHR is what carries the RANGING bit and the PSDU length a
// receiver needs before it can decode the payload.
inline std::string phr_mode_unsupported_reason(PhrMode v)
{
    switch (v) {
    case PhrMode::Standard:
        return "the standard 13-bit (2 octet) HRP PHR is the one frame v1 encodes";
    case PhrMode::Extended:
        return "phr_mode=extended asks for a vendor PHR layout longer than the 13 information "
               "bits frame v1 encodes (2 octets, SEC-DED coded to 19 bits over 21 symbols). No "
               "extended-PHR layout exists in this codec or in uwb_hrp_mod_core.h, so the request "
               "is refused rather than transmitted as a standard PHR. This is a limit of this "
               "software, not a claim about any Qorvo part";
    case PhrMode::None:
        return "a TWR frame without a PHR is not a valid TWR profile; the PHR carries the RANGING "
               "bit and the PSDU length the receiver needs before it can decode the payload";
    }
    return "phr_mode " + std::to_string(static_cast<unsigned>(v)) +
           " is not a member of the PHR-mode enumeration";
}

// STS is a phase-2 capability (REQ-SCOPE-04).  The values exist so a future
// profile can claim them; the shipped capabilities() whitelist has none, so
// any non-Off value is rejected as Unsupported.
enum class StsMode : uint8_t {
    Off = 0,
    Sp64 = 1,
    Sp128 = 2,
    Sp256 = 3,
    Sp512 = 4,
    Sp1024 = 5
};
inline const char* sts_mode_to_string(StsMode v)
{
    switch (v) {
    case StsMode::Off:
        return "off";
    case StsMode::Sp64:
        return "sp64";
    case StsMode::Sp128:
        return "sp128";
    case StsMode::Sp256:
        return "sp256";
    case StsMode::Sp512:
        return "sp512";
    case StsMode::Sp1024:
        return "sp1024";
    }
    return "invalid";
}
inline bool sts_mode_from_string(const std::string& s, StsMode& out)
{
    for (int i = 0; i <= static_cast<int>(StsMode::Sp1024); ++i) {
        StsMode c = static_cast<StsMode>(i);
        if (s == sts_mode_to_string(c)) {
            out = c;
            return true;
        }
    }
    return false;
}

// WHO appends the FCS.  Exactly one layer may do it (REQ-API-01 帧格式).
enum class FcsAppender : uint8_t {
    None = 0,    // invalid: the profile must append somewhere
    MacLayer = 1,// the MAC builds the CRC and reports mac_psdu_includes_fcs
    PhyLayer = 2 // the PHY appends it; mac_psdu_includes_fcs is then false
};
inline const char* fcs_appender_to_string(FcsAppender v)
{
    switch (v) {
    case FcsAppender::None:
        return "none";
    case FcsAppender::MacLayer:
        return "mac";
    case FcsAppender::PhyLayer:
        return "phy";
    }
    return "invalid";
}
inline bool fcs_appender_from_string(const std::string& s, FcsAppender& out)
{
    if (s == "none") {
        out = FcsAppender::None;
        return true;
    }
    if (s == "mac") {
        out = FcsAppender::MacLayer;
        return true;
    }
    if (s == "phy") {
        out = FcsAppender::PhyLayer;
        return true;
    }
    return false;
}

// The three TX power concepts, kept apart (REQ-API-01 发射 / AGENTS rule 7).
enum class TxPowerPolicy : uint8_t {
    LeaveUntouched = 0, // do not program the power at all
    ManualGainDb = 1,   // authority: tx.gain_db
    IqAmplitude = 2,    // authority: tx.iq_amplitude (digital scaling)
    CalibratedDbm = 3   // authority: tx.calibrated_tx_power_dbm
};
inline const char* tx_power_policy_to_string(TxPowerPolicy v)
{
    switch (v) {
    case TxPowerPolicy::LeaveUntouched:
        return "leave_untouched";
    case TxPowerPolicy::ManualGainDb:
        return "manual_gain_db";
    case TxPowerPolicy::IqAmplitude:
        return "iq_amplitude";
    case TxPowerPolicy::CalibratedDbm:
        return "calibrated_dbm";
    }
    return "invalid";
}
inline bool tx_power_policy_from_string(const std::string& s, TxPowerPolicy& out)
{
    if (s == "leave_untouched") {
        out = TxPowerPolicy::LeaveUntouched;
        return true;
    }
    if (s == "manual_gain_db") {
        out = TxPowerPolicy::ManualGainDb;
        return true;
    }
    if (s == "iq_amplitude") {
        out = TxPowerPolicy::IqAmplitude;
        return true;
    }
    if (s == "calibrated_dbm") {
        out = TxPowerPolicy::CalibratedDbm;
        return true;
    }
    return false;
}

enum class PulseShaping : uint8_t {
    ExistingHrP = 0,     // the pulse-shaped SYNC field already in lib/uwb_hrp_*
    Rectangular = 1,
    RootRaisedCosine = 2
};
inline const char* pulse_shaping_to_string(PulseShaping v)
{
    switch (v) {
    case PulseShaping::ExistingHrP:
        return "existing_hrp";
    case PulseShaping::Rectangular:
        return "rectangular";
    case PulseShaping::RootRaisedCosine:
        return "root_raised_cosine";
    }
    return "invalid";
}
inline bool pulse_shaping_from_string(const std::string& s, PulseShaping& out)
{
    if (s == "existing_hrp") {
        out = PulseShaping::ExistingHrP;
        return true;
    }
    if (s == "rectangular") {
        out = PulseShaping::Rectangular;
        return true;
    }
    if (s == "root_raised_cosine") {
        out = PulseShaping::RootRaisedCosine;
        return true;
    }
    return false;
}

enum class AgcMode : uint8_t {
    Manual = 0,
    Disabled = 1,
    VendorDefault = 2
};
inline const char* agc_mode_to_string(AgcMode v)
{
    switch (v) {
    case AgcMode::Manual:
        return "manual";
    case AgcMode::Disabled:
        return "disabled";
    case AgcMode::VendorDefault:
        return "vendor_default";
    }
    return "invalid";
}
inline bool agc_mode_from_string(const std::string& s, AgcMode& out)
{
    if (s == "manual") {
        out = AgcMode::Manual;
        return true;
    }
    if (s == "disabled") {
        out = AgcMode::Disabled;
        return true;
    }
    if (s == "vendor_default") {
        out = AgcMode::VendorDefault;
        return true;
    }
    return false;
}

enum class FirstPathAlgorithm : uint8_t {
    LeadingEdge = 0,
    Peak = 1,
    EnergyCentroid = 2,
    InterpolatedPeak = 3,
    StrongestCluster = 4
};
inline const char* first_path_algorithm_to_string(FirstPathAlgorithm v)
{
    switch (v) {
    case FirstPathAlgorithm::LeadingEdge:
        return "leading_edge";
    case FirstPathAlgorithm::Peak:
        return "peak";
    case FirstPathAlgorithm::EnergyCentroid:
        return "energy_centroid";
    case FirstPathAlgorithm::InterpolatedPeak:
        return "interpolated_peak";
    case FirstPathAlgorithm::StrongestCluster:
        return "strongest_cluster";
    }
    return "invalid";
}
inline bool first_path_algorithm_from_string(const std::string& s,
    FirstPathAlgorithm& out)
{
    if (s == "leading_edge") {
        out = FirstPathAlgorithm::LeadingEdge;
        return true;
    }
    if (s == "peak") {
        out = FirstPathAlgorithm::Peak;
        return true;
    }
    if (s == "energy_centroid") {
        out = FirstPathAlgorithm::EnergyCentroid;
        return true;
    }
    if (s == "interpolated_peak") {
        out = FirstPathAlgorithm::InterpolatedPeak;
        return true;
    }
    if (s == "strongest_cluster") {
        out = FirstPathAlgorithm::StrongestCluster;
        return true;
    }
    return false;
}

// Off / On / Required.  `Required` means a measurement without the
// compensation is a failure, not a lower-quality success (REQ-TIME-05).
enum class CompensationFlag : uint8_t {
    Off = 0,
    On = 1,
    Required = 2
};
inline const char* compensation_flag_to_string(CompensationFlag v)
{
    switch (v) {
    case CompensationFlag::Off:
        return "off";
    case CompensationFlag::On:
        return "on";
    case CompensationFlag::Required:
        return "required";
    }
    return "invalid";
}
inline bool compensation_flag_from_string(const std::string& s, CompensationFlag& out)
{
    if (s == "off") {
        out = CompensationFlag::Off;
        return true;
    }
    if (s == "on") {
        out = CompensationFlag::On;
        return true;
    }
    if (s == "required") {
        out = CompensationFlag::Required;
        return true;
    }
    return false;
}

// The event a timed field is measured FROM (REQ-API-02).  This is what makes
// "the protocol reply delay", "the post-TX RX enable delay" and "the timeout
// waiting for an expected RX frame" three different, non-interchangeable
// numbers.
enum class TimeReferenceEvent : uint8_t {
    PollTransmitRmarker = 0,
    PollReceiveRmarker = 1,
    ResponseTransmitRmarker = 2,
    ResponseReceiveRmarker = 3,
    FinalTransmitRmarker = 4,
    FinalReceiveRmarker = 5,
    ReportTransmitRmarker = 6,
    ReportReceiveRmarker = 7,
    FrameTail = 8,     // last transmitted / received sample of the frame
    RxEnable = 9,       // the timed RX stream command was armed
    HostMonotonic = 10 // std::chrono::steady_clock, never the RF plane
};
inline const char* time_reference_event_to_string(TimeReferenceEvent v)
{
    switch (v) {
    case TimeReferenceEvent::PollTransmitRmarker:
        return "poll_tx_rmarker";
    case TimeReferenceEvent::PollReceiveRmarker:
        return "poll_rx_rmarker";
    case TimeReferenceEvent::ResponseTransmitRmarker:
        return "response_tx_rmarker";
    case TimeReferenceEvent::ResponseReceiveRmarker:
        return "response_rx_rmarker";
    case TimeReferenceEvent::FinalTransmitRmarker:
        return "final_tx_rmarker";
    case TimeReferenceEvent::FinalReceiveRmarker:
        return "final_rx_rmarker";
    case TimeReferenceEvent::ReportTransmitRmarker:
        return "report_tx_rmarker";
    case TimeReferenceEvent::ReportReceiveRmarker:
        return "report_rx_rmarker";
    case TimeReferenceEvent::FrameTail:
        return "frame_tail";
    case TimeReferenceEvent::RxEnable:
        return "rx_enable";
    case TimeReferenceEvent::HostMonotonic:
        return "host_monotonic";
    }
    return "invalid";
}
inline bool time_reference_event_from_string(const std::string& s,
    TimeReferenceEvent& out)
{
    for (int i = 0; i <= static_cast<int>(TimeReferenceEvent::HostMonotonic); ++i) {
        TimeReferenceEvent c = static_cast<TimeReferenceEvent>(i);
        if (s == time_reference_event_to_string(c)) {
            out = c;
            return true;
        }
    }
    return false;
}

// Which clock the number lives in (REQ-TIME-01).  `Unspecified` is never
// acceptable in a validated config.
enum class TimeDomain : uint8_t {
    Unspecified = 0,
    DeviceTicks = 1,
    MonotonicHost = 2
};
inline const char* time_domain_to_string(TimeDomain v)
{
    switch (v) {
    case TimeDomain::Unspecified:
        return "unspecified";
    case TimeDomain::DeviceTicks:
        return "device_ticks";
    case TimeDomain::MonotonicHost:
        return "monotonic_host";
    }
    return "invalid";
}
inline bool time_domain_from_string(const std::string& s, TimeDomain& out)
{
    if (s == "unspecified") {
        out = TimeDomain::Unspecified;
        return true;
    }
    if (s == "device_ticks") {
        out = TimeDomain::DeviceTicks;
        return true;
    }
    if (s == "monotonic_host") {
        out = TimeDomain::MonotonicHost;
        return true;
    }
    return false;
}

// The unit a calibration field is expressed in.  Present so that ns,
// seconds and native ticks can never be added together by accident
// (REQ-TIME-01 / REQ-CAL-01).
enum class TimeUnit : uint8_t {
    Nanoseconds = 0,
    Seconds = 1,
    NativeTicks = 2
};
inline const char* time_unit_to_string(TimeUnit v)
{
    switch (v) {
    case TimeUnit::Nanoseconds:
        return "ns";
    case TimeUnit::Seconds:
        return "s";
    case TimeUnit::NativeTicks:
        return "native_ticks";
    }
    return "invalid";
}
inline bool time_unit_from_string(const std::string& s, TimeUnit& out)
{
    if (s == "ns") {
        out = TimeUnit::Nanoseconds;
        return true;
    }
    if (s == "s") {
        out = TimeUnit::Seconds;
        return true;
    }
    if (s == "native_ticks") {
        out = TimeUnit::NativeTicks;
        return true;
    }
    return false;
}

// ===========================================================================
// 1. Machine-readable rejection reasons and the validation report
// ===========================================================================

// Every rejection carries one of these.  Values are appended-only (they are
// part of the JSON/CSV schema, REQ-OUT-01).
enum class ConfigReason : uint16_t {
    None = 0,

    // --- value level ------------------------------------------------------
    NotFinite = 1,          // NaN / +Inf / -Inf in a float field
    OutOfRange = 2,         // outside the documented numeric range
    IndexOutOfRange = 3,    // array / code / channel index out of range
    NegativeValue = 4,      // a delay or window may not be negative
    EmptyValue = 5,         // required string / identifier is empty
    OverCapacity = 6,       // queue or diagnostic buffer above its bound
    ZeroValue = 7,          // a field that must be non-zero is zero

    // --- cross-field ------------------------------------------------------
    FieldConflict = 20,     // two fields cannot both be true
    ChannelFrequencyMismatch = 21,
    DuplicateResource = 22, // two endpoints claim one physical channel
    TimingOrderViolation = 23,
    TimingBudgetInfeasible = 24,
    TimingPrecisionInsufficient = 25,
    QuantisationUnsupported = 26,
    UnitMismatch = 27,
    FrameLengthOverflow = 28,

    // --- capability -------------------------------------------------------
    Unsupported = 40,      // not in the capability whitelist
    OutOfScope = 41,       // real, but deliberately out of phase-1 scope
    InFlightNotSupported = 42,

    // --- calibration ------------------------------------------------------
    CalibrationMissing = 60,
    CalibrationExpired = 61,
    CalibrationAlreadyApplied = 62,
    CalibrationMismatch = 63,

    // --- lifecycle / mutation ---------------------------------------------
    MidExchangeMutation = 80,
    ExchangeAlreadyInFlight = 81,

    // --- JSON import ------------------------------------------------------
    MalformedJson = 100,
    UnknownKey = 101,
    MissingKey = 102,
    TypeMismatch = 103,
    UnknownEnumValue = 104,
    IntegerPrecisionLoss = 105
};

inline const char* config_reason_to_string(ConfigReason r)
{
    switch (r) {
    case ConfigReason::None:
        return "none";
    case ConfigReason::NotFinite:
        return "not_finite";
    case ConfigReason::OutOfRange:
        return "out_of_range";
    case ConfigReason::IndexOutOfRange:
        return "index_out_of_range";
    case ConfigReason::NegativeValue:
        return "negative_value";
    case ConfigReason::EmptyValue:
        return "empty_value";
    case ConfigReason::OverCapacity:
        return "over_capacity";
    case ConfigReason::ZeroValue:
        return "zero_value";
    case ConfigReason::FieldConflict:
        return "field_conflict";
    case ConfigReason::ChannelFrequencyMismatch:
        return "channel_frequency_mismatch";
    case ConfigReason::DuplicateResource:
        return "duplicate_resource";
    case ConfigReason::TimingOrderViolation:
        return "timing_order_violation";
    case ConfigReason::TimingBudgetInfeasible:
        return "timing_budget_infeasible";
    case ConfigReason::TimingPrecisionInsufficient:
        return "timing_precision_insufficient";
    case ConfigReason::QuantisationUnsupported:
        return "quantisation_unsupported";
    case ConfigReason::UnitMismatch:
        return "unit_mismatch";
    case ConfigReason::FrameLengthOverflow:
        return "frame_length_overflow";
    case ConfigReason::Unsupported:
        return "unsupported";
    case ConfigReason::OutOfScope:
        return "out_of_scope";
    case ConfigReason::InFlightNotSupported:
        return "in_flight_not_supported";
    case ConfigReason::CalibrationMissing:
        return "calibration_missing";
    case ConfigReason::CalibrationExpired:
        return "calibration_expired";
    case ConfigReason::CalibrationAlreadyApplied:
        return "calibration_already_applied";
    case ConfigReason::CalibrationMismatch:
        return "calibration_mismatch";
    case ConfigReason::MidExchangeMutation:
        return "mid_exchange_mutation";
    case ConfigReason::ExchangeAlreadyInFlight:
        return "exchange_already_in_flight";
    case ConfigReason::MalformedJson:
        return "malformed_json";
    case ConfigReason::UnknownKey:
        return "unknown_key";
    case ConfigReason::MissingKey:
        return "missing_key";
    case ConfigReason::TypeMismatch:
        return "type_mismatch";
    case ConfigReason::UnknownEnumValue:
        return "unknown_enum_value";
    case ConfigReason::IntegerPrecisionLoss:
        return "integer_precision_loss";
    }
    return "invalid";
}

inline bool config_reason_from_string(const std::string& s, ConfigReason& out)
{
    for (int i = 0; i <= static_cast<int>(ConfigReason::IntegerPrecisionLoss); ++i) {
        ConfigReason c = static_cast<ConfigReason>(i);
        if (s == config_reason_to_string(c)) {
            out = c;
            return true;
        }
    }
    return false;
}

// One rejection.  `status` is the frozen ExchangeStatus the caller will see,
// so a configuration failure and a runtime failure of the same family are
// reported with the same machine-readable code (REQ-ERR-01).
struct ConfigViolation {
    std::string field;   // dotted path, e.g. "phy.code_index"
    ConfigReason reason = ConfigReason::None;
    ExchangeStatus status = ExchangeStatus::ConfigRejected;
    std::string message;  // human-readable detail
    std::string requirement; // "REQ-API-01", ...

    bool operator==(const ConfigViolation& o) const
    {
        return field == o.field && reason == o.reason && status == o.status &&
               message == o.message && requirement == o.requirement;
    }
};

inline std::string config_violation_to_string(const ConfigViolation& v)
{
    std::string s = v.field.empty() ? std::string("<config>") : v.field;
    s += ": ";
    s += config_reason_to_string(v.reason);
    s += " [";
    s += exchange_status_to_string(v.status);
    s += "]";
    if (!v.message.empty()) {
        s += " ";
        s += v.message;
    }
    if (!v.requirement.empty()) {
        s += " (";
        s += v.requirement;
        s += ")";
    }
    return s;
}

// The full report.  `ok()` is never the only information a caller gets.
struct ValidationReport {
    bool valid = true;
    std::vector<ConfigViolation> violations;

    bool ok() const { return valid; }
    bool has(ConfigReason r) const
    {
        for (const auto& v : violations)
            if (v.reason == r)
                return true;
        return false;
    }
    bool has(const std::string& field, ConfigReason r) const
    {
        return find(field, r) != nullptr;
    }
    bool has_field(const std::string& path) const
    {
        for (const auto& v : violations)
            if (v.field == path)
                return true;
        return false;
    }
    const ConfigViolation* find(const std::string& path, ConfigReason r) const
    {
        for (const auto& v : violations)
            if (v.field == path && v.reason == r)
                return &v;
        return nullptr;
    }
    const ConfigViolation* first_for_field(const std::string& path) const
    {
        for (const auto& v : violations)
            if (v.field == path)
                return &v;
        return nullptr;
    }
    // First non-Ok status, in report order.  Never a bare bool.
    ExchangeStatus first_status() const
    {
        if (violations.empty())
            return ExchangeStatus::Ok;
        return violations.front().status;
    }
    std::vector<std::string> messages() const
    {
        std::vector<std::string> out;
        out.reserve(violations.size());
        for (const auto& v : violations)
            out.push_back(config_violation_to_string(v));
        return out;
    }
    std::string to_string() const
    {
        if (violations.empty())
            return "ok";
        std::string s;
        for (size_t i = 0; i < violations.size(); ++i) {
            if (i)
                s += "; ";
            s += config_violation_to_string(violations[i]);
        }
        return s;
    }
    void add(const ConfigViolation& v)
    {
        valid = false;
        violations.push_back(v);
    }
    void add(const std::string& field,
             ConfigReason reason,
             ExchangeStatus status,
             const std::string& message,
             const char* requirement)
    {
        ConfigViolation v;
        v.field = field;
        v.reason = reason;
        v.status = status;
        v.message = message;
        v.requirement = requirement;
        add(v);
    }
};

// ===========================================================================
// 2. Timed fields (reference event + marker + unit + quantisation)
// ===========================================================================

// A duration that knows what it is measured FROM, in WHICH clock, with which
// RF marker, and to WHAT resolution the device must express it (REQ-API-02).
struct TimedField {
    Duration value{0};
    TimeDomain domain = TimeDomain::Unspecified;
    TimeReferenceEvent reference = TimeReferenceEvent::HostMonotonic;
    // Required iff domain == DeviceTicks; must be ABSENT for MonotonicHost
    // (there is no RF marker on a host monotonic clock).
    Opt<TimestampMarker> marker;
    // Device tick rate the value is quantised to, or 1e9 for integer ns.
    double required_quantisation_hz = 0.0;
    // Largest quantisation error the caller tolerates, in ns.  1 is exact to
    // the nearest tick for every rate this build supports; 0 demands an exact
    // nanosecond and is therefore rejected (TimingPrecisionInsufficient).
    int64_t max_quantisation_error_ns = 1;
    // Free text: how the number was obtained / converted.  REQ-API-02
    // requires the conversion result to be recorded when a delay is
    // expressed "after the end of the frame".
    std::string note;

    bool operator==(const TimedField& o) const
    {
        return value == o.value && domain == o.domain && reference == o.reference &&
               marker == o.marker &&
                   required_quantisation_hz == o.required_quantisation_hz &&
               max_quantisation_error_ns == o.max_quantisation_error_ns &&
                   note == o.note;
    }
    bool operator!=(const TimedField& o) const { return !(*this == o); }

    std::string to_text() const
    {
        std::string s = twr_duration_to_text(value);
        s += "@";
        s += time_domain_to_string(domain);
        s += "/";
        s += time_reference_event_to_string(reference);
        s += "/";
        s += marker.has() ? timestamp_marker_to_string(marker.value()) : "(no marker)";
        s += "/q=";
        s += twr_double_to_text(required_quantisation_hz);
        s += "/e=";
        s += twr_int_to_text(max_quantisation_error_ns);
        s += "ns";
        return s;
    }
};

inline std::string timed_field_to_text(const TimedField& f) { return f.to_text(); }

// Round to the nearest device tick and report whether the value survived the
// trip.  This is the single place a Duration becomes device ticks; adapters
// convert from here, never at the call site (REQ-API-02).
inline int64_t quantise_duration(const Duration& d, double rate_hz, bool& ok)
{
    int64_t ticks = d.ticks_at(rate_hz, ok);
    if (!ok)
        return 0;
    // Guard the reverse direction too: the tick-quantised value must still be
    // a finite, representable Duration.
    Duration back;
    if (!Duration::from_ticks(ticks, rate_hz, back)) {
        ok = false;
        return 0;
    }
    return ticks;
}

// ===========================================================================
// 3. Capability whitelist (default deny)
// ===========================================================================

enum class CapabilityStatus : uint8_t {
    Measured = 0,        // established in this repo / frozen requirements
    MeasuredPending = 1, // the measured matrix has not landed yet
    ByDefinition = 2,    // fixed by the TWR requirement, no measurement needed
    OutOfScope = 3       // deliberately excluded from phase 1
};
inline const char* capability_status_to_string(CapabilityStatus v)
{
    switch (v) {
    case CapabilityStatus::Measured:
        return "measured";
    case CapabilityStatus::MeasuredPending:
        return "measured_pending";
    case CapabilityStatus::ByDefinition:
        return "by_definition";
    case CapabilityStatus::OutOfScope:
        return "out_of_scope";
    }
    return "invalid";
}

// One row of testdata/twr/phy_matrix_*.csv, expressed as a whitelist entry.
// `result` in the CSV is what a measurement produced; a row only enters the
// whitelist once its `result` is a pass AND the code+SYNC+SFD combination
// round-trips with a correct FCS at both endpoints (REQ-PHY-01: "不能仅据
// API 枚举认定兼容").
struct PhyCapabilityRow {
    double native_rate_hz = 0.0;
    uint8_t code_index = 0;
    uint16_t sync_repetitions = 0;
    SfdMode sfd_mode = SfdMode::R4z2;
    uint16_t max_psdu_bytes = 0;
    bool ranging = true;
    CapabilityStatus status = CapabilityStatus::MeasuredPending;
    std::string reason;
    // M0.1 R7: what this row was ACTUALLY measured to establish.  A row may
    // be work-decode verified and nothing more -- which is exactly the M0
    // state, and the reason it must not be read as ranging capability.
    evidence::Level evidence_level = evidence::Level::None;
    std::string evidence_source;
    std::string evidence_provenance_id;
    std::string evidence_not_established; // the levels explicitly NOT claimed
};

// Outcome of a whitelist lookup.  `allowed == false` with a
// `MeasuredPending` status means "not yet known", which is still a rejection
// (REQ-SCOPE-01) and never a silent fallback.
struct CapabilityLookup {
    bool allowed = false;
    CapabilityStatus status = CapabilityStatus::MeasuredPending;
    std::string reason;
    // M0.1 R7: the evidence behind this answer.  A refusal states what is
    // missing and at which level it would have to be measured, so an operator
    // can tell "not looked at" from "looked at and failed".
    evidence::Level established = evidence::Level::None;
    evidence::Use requested_use = evidence::Use::WorkDecode;
    std::string evidence_not_established;
    std::string provenance_id;
};

// The whitelist.  Everything not listed here is rejected.
//
// The shipped instance is built by `build_default_capabilities()`.  When the
// measured PHY matrix lands, the measured rows are added through
// `CapabilitiesBuilder` (see the MEASURED-PENDING comment blocks) and the
// `MeasuredPending` marker moves to `Measured`.
class Capabilities
{
public:
    // ---- MEASURED lists -------------------------------------------------
    std::vector<double> native_rates_hz;
    std::vector<uint8_t> code_indices;
    std::vector<uint16_t> max_psdu_bytes_seen;
    std::vector<PrfClass> prf_classes;
    std::vector<uint8_t> channels;
    double work_sample_rate_hz = 0.0;
    int64_t work_samples_per_symbol = 0;
    double mean_prf_hz = 0.0;
    bool ranging_bit_required = true;

    // ---- MEASURED-PENDING lists (empty until the matrix lands) -----------
    std::vector<DataRate> data_rates;
    std::vector<PhyCapabilityRow> phy_matrix;
    std::vector<uint16_t> sync_repetitions;
    std::vector<SfdMode> sfd_modes;

    // ---- explicitly out of scope ---------------------------------------
    bool sts_supported = false;
    std::vector<StsMode> sts_modes;

    // ---- quantisation ---------------------------------------------------
    std::vector<double> quantisation_rates_hz;

    // ---- provenance -----------------------------------------------------
    std::string schema_version = "twr-config/2";
    std::string profile_version = "unfrozen";
    std::string build_id = "m0";
    std::string pending_reason =
        "PHY capability matrix not measured yet; see "
        "testdata/twr/phy_matrix_737280000.csv";
    std::string unsupported_sts_reason =
        "STS is out of scope for phase 1 (REQ-SCOPE-04, explicit user decision)";

    // ---- queries --------------------------------------------------------

    bool native_rate_supported(double hz) const
    {
        for (double v : native_rates_hz)
            if (twr_rate_matches_strict(v, hz, kTwrNativeRateToleranceRel))
                return true;
        return false;
    }
    bool code_index_supported(uint8_t n) const
    {
        for (uint8_t v : code_indices)
            if (v == n)
                return true;
        return false;
    }
    bool prf_class_supported(PrfClass c) const
    {
        for (PrfClass v : prf_classes)
            if (v == c)
                return true;
        return false;
    }
    bool channel_supported(uint8_t ch) const
    {
        for (uint8_t v : channels)
            if (v == ch)
                return true;
        return false;
    }
    bool data_rate_supported(DataRate r) const
    {
        for (DataRate v : data_rates)
            if (v == r)
                return true;
        return false;
    }
    bool sync_repetitions_supported(uint16_t n) const
    {
        for (uint16_t v : sync_repetitions)
            if (v == n)
                return true;
        return false;
    }
    bool sfd_mode_supported(SfdMode m) const
    {
        for (SfdMode v : sfd_modes)
            if (v == m)
                return true;
        return false;
    }
    uint16_t max_psdu_bytes() const
    {
        uint16_t m = 0;
        for (uint16_t v : max_psdu_bytes_seen)
            if (v > m)
                m = v;
        return m;
    }
    bool quantisation_supported(double hz) const
    {
        for (double v : quantisation_rates_hz)
            if (twr_rate_matches_strict(v, hz, 1e-9))
                return true;
        return false;
    }

    // The full (native rate, code, SYNC, SFD, PSDU, ranging) joint lookup.
    // Every one of them is cross-validated together: "code supported" alone
    // never implies "this preamble length transmits and decodes both ways"
    // (REQ-BASE-02).
    CapabilityLookup lookup_phy(double native_rate_hz,
                                uint8_t code_index,
                                uint16_t sync_repetitions_in,
                                SfdMode sfd_mode,
                                uint16_t psdu_bytes,
                                bool ranging) const
    {
        CapabilityLookup out;
        if (phy_matrix.empty()) {
            out.allowed = false;
            out.status = CapabilityStatus::MeasuredPending;
            out.reason = pending_reason;
            return out;
        }
        for (const auto& row : phy_matrix) {
            if (!twr_rate_matches_strict(row.native_rate_hz, native_rate_hz, 1e-9))
                continue;
            if (row.code_index != code_index)
                continue;
            if (row.sync_repetitions != sync_repetitions_in)
                continue;
            if (row.sfd_mode != sfd_mode)
                continue;
            if (row.ranging != ranging)
                continue;
            if (psdu_bytes > row.max_psdu_bytes)
                continue;
            out.allowed = true;
            out.status = row.status;
            out.reason = row.reason;
            out.established = row.evidence_level;
            out.evidence_not_established = row.evidence_not_established;
            out.provenance_id = row.evidence_provenance_id;
            return out;
        }
        out.allowed = false;
        out.status = CapabilityStatus::MeasuredPending;
        out.established = evidence::Level::None;
        out.reason = "no measured row for native_rate=" + twr_double_to_text(native_rate_hz) +
                     " code=" + twr_int_to_text(code_index) +
                     " sync=" + twr_int_to_text(sync_repetitions_in) +
                     std::string(" sfd=") + sfd_mode_to_string(sfd_mode) +
                     " psdu<=" + twr_int_to_text(psdu_bytes) +
                     "; see testdata/twr/phy_matrix_*.csv";
        return out;
    }

    std::string to_string() const
    {
        std::string s;
        s += "schema=" + schema_version;
        s += " profile=" + profile_version;
        s += " build=" + build_id + "\n";
        s += "native_rates_hz=";
        for (size_t i = 0; i < native_rates_hz.size(); ++i) {
            if (i)
                s += ",";
            s += twr_double_to_text(native_rates_hz[i]);
        }
        s += "\ncode_indices=";
        for (size_t i = 0; i < code_indices.size(); ++i) {
            if (i)
                s += ",";
            s += twr_int_to_text(static_cast<int64_t>(code_indices[i]));
        }
        s += "\nchannels=";
        for (size_t i = 0; i < channels.size(); ++i) {
            if (i)
                s += ",";
            s += twr_int_to_text(static_cast<int64_t>(channels[i]));
        }
        s += "\nprf_classes=";
        for (size_t i = 0; i < prf_classes.size(); ++i) {
            if (i)
                s += ",";
            s += prf_class_to_string(prf_classes[i]);
        }
        s += "\nwork_sample_rate_hz=" + twr_double_to_text(work_sample_rate_hz);
        s += " work_samples_per_symbol=" + twr_int_to_text(work_samples_per_symbol);
        s += " mean_prf_hz=" + twr_double_to_text(mean_prf_hz) + "\n";
        s += "max_psdu_bytes=" + twr_int_to_text(static_cast<int64_t>(max_psdu_bytes())) + "\n";
        s += "data_rates=";
        for (size_t i = 0; i < data_rates.size(); ++i) {
            if (i)
                s += ",";
            s += data_rate_to_string(data_rates[i]);
        }
        s += "\nsync_repetitions=";
        for (size_t i = 0; i < sync_repetitions.size(); ++i) {
            if (i)
                s += ",";
            s += twr_int_to_text(static_cast<int64_t>(sync_repetitions[i]));
        }
        s += "\nsfd_modes=";
        for (size_t i = 0; i < sfd_modes.size(); ++i) {
            if (i)
                s += ",";
            s += sfd_mode_to_string(sfd_modes[i]);
        }
        s += "\nranging_bit_required=" + twr_bool_to_text(ranging_bit_required) + "\n";
        s += "sts_supported=" + twr_bool_to_text(sts_supported) +
             (sts_supported ? "" : " (" + unsupported_sts_reason + ")") + "\n";
        s += "quantisation_rates_hz=";
        for (size_t i = 0; i < quantisation_rates_hz.size(); ++i) {
            if (i)
                s += ",";
            s += twr_double_to_text(quantisation_rates_hz[i]);
        }
        s += "\nphy_matrix_rows=" + twr_int_to_text(static_cast<int64_t>(phy_matrix.size()));
        if (phy_matrix.empty())
            s += " [MEASURED-PENDING] " + pending_reason;
        s += "\n";
        return s;
    }

};

// Public builder so a measured matrix can be installed without touching the
// shipped default.  It is a TOP-LEVEL class (a nested class could not hold
// its enclosing class by value: the class is still incomplete inside its own
// body).  Nothing here relaxes a default: adding a row only ever turns a
// rejection into an acceptance.
class CapabilitiesBuilder
{
public:
    explicit CapabilitiesBuilder(Capabilities base) : d_caps(std::move(base)) {}

    CapabilitiesBuilder& add_native_rate(double hz)
    {
        d_caps.native_rates_hz.push_back(hz);
        return *this;
    }
    CapabilitiesBuilder& add_code_index(uint8_t n)
    {
        d_caps.code_indices.push_back(n);
        return *this;
    }
    CapabilitiesBuilder& add_channel(uint8_t ch)
    {
        d_caps.channels.push_back(ch);
        return *this;
    }
    CapabilitiesBuilder& add_prf_class(PrfClass c)
    {
        d_caps.prf_classes.push_back(c);
        return *this;
    }
    CapabilitiesBuilder& add_data_rate(DataRate r)
    {
        d_caps.data_rates.push_back(r);
        return *this;
    }
    CapabilitiesBuilder& add_sync_repetitions(uint16_t n)
    {
        d_caps.sync_repetitions.push_back(n);
        return *this;
    }
    CapabilitiesBuilder& add_sfd_mode(SfdMode m)
    {
        d_caps.sfd_modes.push_back(m);
        return *this;
    }
    CapabilitiesBuilder& add_max_psdu_bytes(uint16_t n)
    {
        d_caps.max_psdu_bytes_seen.push_back(n);
        return *this;
    }
    CapabilitiesBuilder& add_quantisation_rate(double hz)
    {
        d_caps.quantisation_rates_hz.push_back(hz);
        return *this;
    }
    // Install one measured row of the PHY matrix.
    CapabilitiesBuilder& add_phy_row(const PhyCapabilityRow& row)
    {
        d_caps.phy_matrix.push_back(row);
        return *this;
    }
    CapabilitiesBuilder& set_profile_version(const std::string& v)
    {
        d_caps.profile_version = v;
        return *this;
    }
    CapabilitiesBuilder& set_build_id(const std::string& v)
    {
        d_caps.build_id = v;
        return *this;
    }
    CapabilitiesBuilder& set_pending(bool pending)
    {
        d_caps.pending_reason = pending ? d_caps.pending_reason : std::string();
        if (!pending)
            d_caps.phy_matrix.clear();
        return *this;
    }
    Capabilities build() const { return d_caps; }

private:
    Capabilities d_caps;
};

// IEEE 802.15.4a UWB channel plan (normative centre frequencies).  This is a
// STANDARD table, not a capability claim: the capability whitelist decides
// which of these channels this build may actually use.
inline double uwb_channel_center_frequency_hz(uint8_t channel)
{
    static const double kTable[17] = {
        4835.2e6, 4953.6e6, 5066.4e6, 5179.2e6, 5292.0e6, 6489.6e6, 6614.4e6, 6739.2e6,
        6863.9e6, 6988.8e6, 7112.6e6, 7236.4e6, 7360.1e6, 7483.8e6, 7606.6e6, 7729.4e6,
        7852.2e6
    };
    if (channel > 16u)
        return 0.0;
    return kTable[channel];
}

// Absolute tolerance when checking a configured centre frequency against the
// channel plan.  The X410 NCO resolves far below 1 Hz, so this only catches a
// genuinely wrong channel, not quantisation (mirrors the reasoning behind
// kUhdTxFreqReadbackTolHz in uwb_uhd_backend_config.h).
inline constexpr double kTwrChannelFrequencyToleranceHz = 100.0;

// The SHIPPED whitelist.  MEASURED-PENDING rows are deliberately absent so
// the validator rejects them until the measured matrix lands.
//
// MEASURED-PENDING (see testdata/twr/phy_matrix_*.csv).  The blocks below
// are the only place that has to change once A2's matrix is available: add
// the measured rows with
//     CapabilitiesBuilder(build_default_capabilities())
//         .add_data_rate(DataRate::R6p8M)
//         .add_sync_repetitions(128)
//         .add_sfd_mode(SfdMode::R4z2)
//         .add_phy_row({ 737280000.0, 9, 128, SfdMode::R4z2, 127, true,
//                        CapabilityStatus::Measured, "phy_matrix row 3 pass" })
// SYNC lengths that are decode-verified (byte-exact FCS round trip) but whose
// FIRST-PATH / ToA ACCURACY has not been measured yet.  Ranging accuracy is a
// function of first-path ToA, so a length may appear in the capability
// whitelist while still being unfit as a ranging profile.  16 SYNC is in this
// set because its integration window is 16.3 us against 64 SYNC's 65.1 us.
// M2 must close this before 16 SYNC is used to produce a range.
inline constexpr uint16_t kTwrSyncRepsNeedingToaValidation[] = { 16 };

inline bool twr_sync_reps_needs_toa_validation(uint16_t n)
{
    for (uint16_t v : kTwrSyncRepsNeedingToaValidation)
        if (v == n)
            return true;
    return false;
}

inline Capabilities build_default_capabilities()
{
    Capabilities c;
    c.schema_version = "twr-config/2";
    c.profile_version = "unfrozen-m0";
    c.build_id = "m0";

    // ---- MEASURED (existing code) ----
    c.native_rates_hz.push_back(kTwrNativeRateUc200Hz); // is_allowed_uhd_native_rate()
    c.native_rates_hz.push_back(kTwrNativeRateCg400Hz);
    for (uint8_t i = kTwrCodeIndexMin; i <= kTwrCodeIndexMax; ++i)
        c.code_indices.push_back(i); // radar_meta::code_index_supported()
    c.max_psdu_bytes_seen.push_back(kTwrMaxPsduBytes); // radar_meta::kMaxPsduBytes
    c.work_sample_rate_hz = kTwrWorkSampleRateHz;
    c.work_samples_per_symbol = kTwrWorkSamplesPerSymbol;
    c.mean_prf_hz = kTwrExistingMeanPrfHz; // 62.4e6 as MEASURED by this code base
    c.quantisation_rates_hz.push_back(1.0e9);
    c.quantisation_rates_hz.push_back(kTwrWorkSampleRateHz);
    c.quantisation_rates_hz.push_back(kTwrNativeRateUc200Hz);
    c.quantisation_rates_hz.push_back(kTwrNativeRateCg400Hz);

    // ---- MEASURED (testdata/twr/phy_matrix_737280000.csv,
    //      generated by gr-uwb/lib/qa_uwb_twr_phy_matrix.cc) ----
    //
    // Every row below was produced by a full
    //   modulate -> UwbLoopbackEcho -> demodulate -> FCS-pass, byte-exact
    // round trip at native 737.28 MS/s.  That proves THIS repo's TX and RX
    // agree with each other and nothing more: no DW1000 / DW3000, no SDK
    // frame profile and no firmware hash was measured (REQ-PHY-01, M5).
    //
    // PAYLOAD data rate: 6.81 Mb/s is the only rate that can round trip.
    //   uwb_demod_core.h:stage_payload_fcs always uses the 6.81 geometry
    //   (64 chips/burst, 64 chips/symbol, scrambler offset 1344).  Every
    //   other rate stays rejected.
    //
    //   This is the PAYLOAD axis only.  The PHR is a separate modulation at a
    //   separate rate (0.85 Mb/s, PhrRate::Standard850k); the 2-bit data-rate
    //   field the PHR carries describes THIS rate, not the PHR's own.  M0
    //   conflated the two and required them to be equal, which was backwards
    //   (M0 review R2).
    c.data_rates.push_back(DataRate::R6p8M);

    // SYNC length: 16 and 64 are both measured-supported; 64 is the phase-1
    // profile.  Every rejection below is a MEASURED result, not an assumption.
    //
    //  16  supported.  Its re-measured tail covers the WHOLE preamble
    //      (tail_first = 0), so cfo_zero_peak_corr = 0 and the hardcoded
    //      cfo_skip_initial_repetitions = 24 degenerates to an effective 0
    //      because np(16) <= cfo_min_fit_repetitions(32).  Measured CFO error
    //      0.000 Hz at 0 / +-1 / +-5 / +-20 kHz and 0.007 Hz at +-100 kHz, over
    //      codes 9..12 x 6 SFD modes and PSDU 16/26/31/127 B, with byte-exact
    //      FCS.  It is also SELF-DESCRIBING: encode_phr19 maps <= 16 to
    //      preamble-duration index 0 (= 16), which is a legal IEEE 802.15.4a
    //      BPRF duration, and the measured phr_preamble_idx is 0 on all 16-SYNC
    //      rows.  CAVEAT carried from the measurement: decode-verified ONLY --
    //      first-path / ToA accuracy is UNVERIFIED for the shorter 16.3 us
    //      integration window (vs 65.1 us at 64 SYNC).  Ranging accuracy
    //      depends on exactly that quantity, so 16 must not be used as a
    //      ranging profile until M2 measures it.  See
    //      kTwrSyncRepsNeedingToaValidation below.
    //
    //  32  rejected.  CFO-exact and decodes, but encode_phr19 advertises
    //      preamble-duration index 1 (= 64), so the PHR does not self-describe.
    //
    //  128 / 256 / 512  rejected for TWO independent measured reasons:
    //      (1) uwb_demod_core.h:2203 re-measures only the last
    //          max(cfo_min_fit_repetitions, 40) SYNCs, so earlier peaks are
    //          synthesised with peak_corr = (0,0) i.e. phase 0.  At 128 SYNC
    //          tail_first = 88 while cfo_skip_initial_repetitions stays 24, so
    //          64 zero-phase points leak into the 104-point least-squares fit.
    //          Measured: 20 kHz injected came back as 3791 Hz (error -16209 Hz).
    //          The generalised rule is synthesised - skipped = max(0, reps-64),
    //          so 64 is clean and everything longer is not.
    //      (2) encode_phr19 maps 128/256/512 to preamble-duration index 1
    //          (= 64) and 2048 to index 2 (= 1024), so the PHR does not
    //          self-describe.  128/256/512/2048 are also not legal IEEE
    //          802.15.4a BPRF preamble durations (legal: 16/64/1024/4096).
    //
    //  1024  rejected.  PHR-legal (index 2 = 1024) but the CFO-fit bias
    //      applies; measured error -19901 Hz at 20 kHz.
    //
    //  1 / 2  rejected: stage_cfo needs >= 4 measured peaks -> CfoFailed.
    //  4 / 8  rejected: cir_skip_initial_repetitions = 10 exceeds the
    //           available repetitions -> CirFailed.
    //  Non-power-of-two values are rejected by the API.
    //
    //  EVERY one of these rejections is a limit of THIS repository's
    //  demodulator / modulator, and the Qorvo parts' own API accepts 64
    //  through 2048 (DW3xxx lists 128/256/512 as non-standard preamble
    //  lengths).  `PreambleLength` is therefore the enumeration of what this
    //  software can run, and every reason it emits says so.
    c.sync_repetitions.push_back(16);
    c.sync_repetitions.push_back(64);

    // SFD modes: all six measured combinations round trip.  NOTE that 4z1
    // (4 symbols) and 4z4 (32 symbols) are NOT IEEE 802.15.4a HRP SFD
    // lengths -- the standard defines 8 and 16 symbol forms.  The CSV flags
    // this in its sfd_len_ieee_802154a_standard column (4z1=0, 4z4=0,
    // 4z2=1, 4z3=1, decawave=1, ieee=1).  They are self-consistent on this
    // link but MUST be confirmed against the target module before M5; the
    // phase-1 common profile uses 4z2.
    c.sfd_modes.push_back(SfdMode::R4z1);
    c.sfd_modes.push_back(SfdMode::R4z2);
    c.sfd_modes.push_back(SfdMode::R4z3);
    c.sfd_modes.push_back(SfdMode::R4z4);
    c.sfd_modes.push_back(SfdMode::Dwt8);
    c.sfd_modes.push_back(SfdMode::Ieee8);

    // Joint whitelist: 64 SYNC x {9,10,11,12} x {4z1,4z2,4z3,4z4,decawave,
    // ieee} = 24 measured rows.  A lookup outside this set is rejected with
    // the reason the CSV recorded, never defaulted (REQ-SCOPE-01).
    {
        static const SfdMode kSfd[] = { SfdMode::R4z1,
                                        SfdMode::R4z2,
                                        SfdMode::R4z3,
                                        SfdMode::R4z4,
                                        SfdMode::Dwt8,
                                        SfdMode::Ieee8 };
        // Both measured SYNC lengths get a full row set: 2 x 4 x 6 = 48.
        // 16 SYNC carries the extra caveat token because its first-path / ToA
        // accuracy is unverified (kTwrSyncRepsNeedingToaValidation above).
        const uint16_t kSyncs[] = { 16, 64 };
        for (const uint16_t sync : kSyncs) {
            for (uint8_t code = kTwrCodeIndexMin; code <= kTwrCodeIndexMax;
                 ++code) {
                for (const SfdMode sfd : kSfd) {
                    PhyCapabilityRow row;
                    row.native_rate_hz = kTwrNativeRateUc200Hz;
                    row.code_index = code;
                    row.sync_repetitions = sync;
                    row.sfd_mode = sfd;
                    row.max_psdu_bytes = kTwrMaxPsduBytes;
                    row.ranging = true;
                    row.status = CapabilityStatus::Measured;
                    row.reason =
                        twr_sync_reps_needs_toa_validation(sync)
                            ? "measured_fcs_pass_round_trip_but_first_path_toa_"
                              "accuracy_unverified_see_"
                              "testdata/twr/phy_matrix_737280000.csv"
                            : "measured_fcs_pass_round_trip_see_"
                              "testdata/twr/phy_matrix_737280000.csv";
                    // M0.1 R7: these 48 rows were measured ONLY on the
                    // 998.4 MS/s work grid.  The established level is therefore
                    // exactly WorkDecodeVerified and nothing above it; the
                    // absent levels are recorded as data so nothing downstream
                    // can read a decode-verified row as ranging, hardware or
                    // vendor capability.
                    row.evidence_level = evidence::Level::WorkDecodeVerified;
                    row.evidence_source =
                        "work_direct_998p4_modulate_loopback_demodulate_fcs";
                    row.evidence_not_established =
                        std::string(evidence::level_name(
                            evidence::Level::NativeRoundtripVerified)) +
                        ";" + evidence::level_name(evidence::Level::ToaVerified) +
                        ";" +
                        evidence::level_name(evidence::Level::HardwareVerified) +
                        ";" +
                        evidence::level_name(evidence::Level::VendorInteropVerified);
                    c.phy_matrix.push_back(row);
                }
            }
        }
    }
    c.profile_version = "m0-ch5-64sync-4z2";

    // ---- from the frozen TWR requirements ----
    // REQ-PHY-01 / REQ-PHY-03: channel 5 at 6489.6 MHz with the 64 MHz PRF
    // class is the common phase-1 baseline.  Other channels are per-channel
    // acceptance items and are NOT claimed here.
    c.channels.push_back(5);
    c.prf_classes.push_back(PrfClass::Bprf64);
    // A TWR MAC frame must set the ranging bit (REQ-PROTO-05).
    c.ranging_bit_required = true;

    // ---- out of scope for phase 1 ----
    c.sts_supported = false;
    c.sts_modes.clear();

    return c;
}

// The single process-wide whitelist instance (header-only, function-local
// static; initialised on first use, never mutated after).
inline const Capabilities& capabilities()
{
    static const Capabilities kCaps = build_default_capabilities();
    return kCaps;
}

// ===========================================================================
// 4. The typed configuration
// ===========================================================================

// --- 帧几何 (frame geometry) ------------------------------------------------

// A CLAIM about the versioned TWR MAC frame layout (REQ-PROTO-06), not a
// second source of truth for it.
//
// M0 review R3: this struct used to carry a pre-codec "7-byte header" geometry
// (FCF(2) + sequence(1) + PAN(2) + address(2)) with 9/19/24-byte budgets while
// uwb_twr_frame.h emitted a 14-byte header and 16/26/31-byte on-air frames.
// Both suites passed because nothing compared the two.
//
// Now the codec owns every one of these numbers (`FrameProfileGeometry`) and
// this struct is only what a caller BELIEVES.  `frame_geometry_claim()` turns
// it into the codec's `FrameGeometryClaim`, `frame_geometry_check()` reports
// every field that disagrees, and the validator turns each disagreement into
// its own rejection naming that exact field.  The fields stay here so the
// claim is visible in the canonical field listing, in the config hash and in
// the JSON -- an operator must be able to see what was claimed -- but they
// never decide anything.
struct FrameGeometry {
    // MAC header bytes the caller believes the frame has.  frame v1: 14
    // (version / function code / 16-bit session / seq / PAN / src / dst /
    // flags), from kFrameHeaderBytes.
    uint16_t mac_header_bytes = 0;
    // bytes per timestamp field; 5 for the 40-bit frame-v1 timestamp.
    uint16_t timestamp_bytes = 0;
    // trailing address / profile trailer; frame v1 has none.
    uint16_t mac_footer_bytes = 0;
    // FCS bytes reserved INSIDE the MAC PSDU; 0 when the PHY appends it.
    uint16_t mac_fcs_bytes = 0;
    // PHR information bytes the caller believes are carried; frame v1: 2
    // (13 information bits, SEC-DED coded to 19 bits over 21 symbols).  This is
    // the PHR's INFORMATION size, not its on-air duration.
    uint16_t phr_bytes = 0;

    bool operator==(const FrameGeometry& o) const
    {
        return mac_header_bytes == o.mac_header_bytes &&
               timestamp_bytes == o.timestamp_bytes &&
               mac_footer_bytes == o.mac_footer_bytes &&
                   mac_fcs_bytes == o.mac_fcs_bytes &&
               phr_bytes == o.phr_bytes;
    }
    bool operator!=(const FrameGeometry& o) const { return !(*this == o); }
    bool any_set() const
    {
        return mac_header_bytes != 0 || timestamp_bytes != 0 || mac_footer_bytes != 0 ||
               mac_fcs_bytes != 0 || phr_bytes != 0;
    }
    // One line for a rejection or a diff, with the field order of the codec's
    // own report so the two can be read side by side.
    std::string to_text() const
    {
        return "mac_header_bytes=" + std::to_string(mac_header_bytes) +
               " timestamp_bytes=" + std::to_string(timestamp_bytes) +
               " mac_footer_bytes=" + std::to_string(mac_footer_bytes) +
               " mac_fcs_bytes=" + std::to_string(mac_fcs_bytes) +
               " phr_bytes=" + std::to_string(phr_bytes);
    }
};

// The claim, in the codec's own claim type.  Same field names on purpose: the
// config states what it believes and the codec checks it.  A DISTINCT type
// from FrameProfileGeometry is also deliberate, so an authority and a claim can
// never be swapped for one another by accident.
inline FrameGeometryClaim frame_geometry_claim(const FrameGeometry& g)
{
    FrameGeometryClaim c;
    c.mac_header_bytes = g.mac_header_bytes;
    c.timestamp_bytes = g.timestamp_bytes;
    c.mac_footer_bytes = g.mac_footer_bytes;
    c.mac_fcs_bytes = g.mac_fcs_bytes;
    c.phr_bytes = g.phr_bytes;
    return c;
}

// The geometry the CODEC says, in the config's own struct.  This is how a
// caller (and every test fixture) states a claim without keeping a private
// copy of the numbers: fill the config from the authority, then let the
// validator prove the two agree.
inline FrameGeometry frame_geometry_from_authority(const FrameProfileGeometry& g,
                                                   const FrameProfile& p)
{
    FrameGeometry out;
    out.mac_header_bytes = static_cast<uint16_t>(g.mac_header_bytes());
    out.timestamp_bytes = static_cast<uint16_t>(g.timestamp_bytes(p));
    out.mac_footer_bytes = static_cast<uint16_t>(g.mac_footer_bytes());
    out.mac_fcs_bytes = static_cast<uint16_t>(g.mac_fcs_bytes());
    out.phr_bytes = static_cast<uint16_t>(g.phr_bytes());
    return out;
}

// The claim of the executable authority for a profile id, or false when the id
// names no implemented layout.  Never a silent fallback to frame v1.
inline bool frame_geometry_of_profile(FrameProfileId id, FrameGeometry& out)
{
    FrameProfileGeometry g;
    if (!frame_geometry_for(id, g))
        return false;
    FrameProfile p;
    if (!frame_profile_for(id, p))
        return false;
    out = frame_geometry_from_authority(g, p);
    return true;
}

// Number of MAC PSDU bytes a frame of `type` occupies for a profile, INCLUDING
// the FCS when the MAC appends it and EXCLUDING it when the PHY does (exactly
// one layer appends the FCS, REQ-API-01 帧格式).
//
// This is a DELEGATION to the codec, which is the only place a frame length is
// computed: `FrameProfileGeometry::mac_payload_bytes()` calls
// `frame_length_for()`.  M0 re-summed `header + n * timestamp + footer` here,
// which is how a 7-byte header produced a 9-byte Poll and understated the
// frame-duration budget.
inline uint32_t frame_psdu_bytes(FrameType type, FcsAppender fcs)
{
    const FcsOwner owner = (fcs == FcsAppender::MacLayer) ? FcsOwner::MacLayer
                                                          : FcsOwner::PhyLayer;
    FrameProfileGeometry g;
    g.id = FrameProfileId::TwrV1;
    g.fcs_owner = owner;
    FrameProfile p;
    if (!frame_profile_for(FrameProfileId::TwrV1, p))
        return 0;
    return static_cast<uint32_t>(g.mac_payload_bytes(type, p));
}

// Retained call shape.  `g` is DELIBERATELY IGNORED: a claim does not decide
// a length, and keeping the parameter means an existing call site keeps
// compiling while losing the ability to pass a number of its own.  New code
// should use the two-argument form above.
inline uint32_t frame_psdu_bytes(const FrameGeometry& g, FrameType type, FcsAppender fcs)
{
    (void)g;
    return frame_psdu_bytes(type, fcs);
}

struct SessionConfig {
    Protocol protocol = Protocol::Ss;
    Role role = Role::Initiator;
    uint16_t local_address = 0;
    uint16_t peer_address = 0;
    uint16_t pan_id = 0;
    // LOCAL session id.  The wire field at kOffSessionId is 16 bits, so the
    // mapping is identity-or-refuse (session_id_to_wire()), never a fold or a
    // mask.  The type stays uint32_t DELIBERATELY rather than narrowing to
    // uint16_t: narrowing would make the local id space and the wire id space
    // indistinguishable in the type system, so an operator could no longer
    // express the id they intended and would never learn it does not fit.
    // Keeping the wider type makes the refusal a VISIBLE, TESTABLE, ATTRIBUTABLE
    // configuration error instead of an unreachable code path.
    //
    // The cost is stated here rather than left to be discovered: the wire
    // session space is 2^16, so ~256 concurrently live sessions reach a 50%
    // chance of one collision (kSessionIdBirthdaySessions50pct).  No local ->
    // wire mapping can raise that; only a different frame version with a wider
    // or keyed session field could, and the peer would have to understand it.
    // 0 is the frame profile's "no session" marker.
    uint32_t session_id = 0;
    uint32_t exchange_id = 0;
    // Frame sequence number and its own wrap modulus, INDEPENDENT of the
    // session id (REQ-API-01 会话: 帧序号回绕与会话 ID 独立).
    uint8_t sequence = 0;
    uint16_t sequence_modulus = 256;
    uint32_t measurement_count = 1;
    Duration measurement_interval{0};
    uint32_t max_attempts_per_exchange = 1;
    Duration retry_backoff{0};
    // Phase 1 allows exactly one in-flight exchange per endpoint.
    uint32_t max_in_flight_exchanges = kTwrMaxInFlightExchanges;
    // A peer frame with a mismatching PAN/address is WrongPeer, not a match.
    bool require_pan_match = true;
    bool require_address_match = true;
};

// --- PHY -------------------------------------------------------------------

struct PhyConfig {
    uint8_t channel = 5;
    double center_frequency_hz = 0.0; // 0 = not stated; the plan decides
    uint8_t tx_preamble_code = 0;     // 0 = "not stated" -> rejected
    uint8_t rx_preamble_code = 0;
    // SYNC repetitions.  The raw count is kept because it is what the
    // capability whitelist and the measured PHY matrix key on, but it must be
    // a member of `PreambleLength` -- a length this decoder/CFO path can
    // actually run -- and the validator separately consults the whitelist.
    uint16_t preamble_symbols = 0;
    PrfClass prf_class = PrfClass::Bprf64;
    // PAYLOAD data rate, measured: 6.81 Mb/s is the only rate that round
    // trips through this modulator/demodulator pair.
    DataRate data_rate = DataRate::R6p8M;
    // PHR rate -- a SEPARATE quantity from `data_rate` (M0.1 / R2).  It used
    // to be a DataRate required to EQUAL `data_rate`, which had the rule
    // exactly backwards: this modulator sends a 0.85 Mb/s PHR (21 symbols of
    // 512 chips) ahead of a 6.81 Mb/s payload (8 chips per burst of 64 per
    // symbol).  See the PhrRate comment for the full argument.
    PhrRate phr_rate = PhrRate::Standard850k;
};

// --- 帧格式 (frame format) --------------------------------------------------

struct FrameFormatConfig {
    // WHICH MAC LAYOUT.  A FrameProfileId, not a free-form string: an
    // arbitrary non-empty profile name must not be able to act as an
    // executable geometry (M0 review R3).  It selects the authority that the
    // `geometry` claim below is checked against.
    FrameProfileId frame_profile = FrameProfileId::TwrV1;
    SfdMode sfd_mode = SfdMode::R4z2;
    // Raw SFD length.  It must equal sfd_mode_symbols(sfd_mode): the mode
    // names the sequence, the count is a restatement of it, and a caller may
    // not say "4z2" while stating 4.
    uint16_t sfd_symbols = 0;
    // Timeout after which a received frame is declared SFD-missing.
    // Zero means "disabled"; a non-zero value must be in a device-tick domain.
    TimedField sfd_timeout;
    // PHR presence/form.  The PHR RATE is phy.phr_rate; the two are separate
    // axes and neither is derived from the other.
    PhrMode phr_mode = PhrMode::Standard;
    bool ranging_bit = true;

    // THREE DISTINCT CONCEPTS, never conflated:
    //  (1) upper-layer bytes handed to the MAC, stated as a CLAIM about the
    //      layout that the codec then checks field by field,
    FrameGeometry geometry;
    //  (2) bytes the MAC places in the PSDU field,
    uint16_t mac_psdu_bytes = 0;
    //  (3) whether (2) already contains the FCS,
    bool mac_psdu_includes_fcs = false;
    FcsAppender fcs_append = FcsAppender::PhyLayer;
    // 16-bit CRC.  Checked against the profile's own FCS size; the default
    // value here is only a starting point the validator will refuse.
    uint16_t fcs_bytes = 2;
    // 1 = the application payload the caller wants to carry; the MAC header,
    // the timestamps and the FCS are NOT counted here.
    uint16_t application_payload_bytes = 0;

    // STS: phase-2 only.  Present so the interface can grow, rejected now.
    StsMode sts_mode = StsMode::Off;
    uint16_t sts_length_symbols = 0;
};

// --- 发射 (transmit) -------------------------------------------------------

struct TransmitConfig {
    uint8_t port = 0; // physical TX channel index inside the device
    // The THREE distinct power concepts (REQ-API-01 发射, AGENTS rule 7).
    // `gain_db`  : the UHD/USRP TX gain setting, in dB.
    Opt<double> gain_db;
    // `iq_amplitude` : digital scaling of the SC16 waveform, dimensionless,
    //                  (0, 1] for a full-scale signed 16-bit stream.
    Opt<double> iq_amplitude;
    // `calibrated_tx_power_dbm` : the power MEASURED at the antenna plane for
    //                  this gain/frequency/bandwidth, in dBm.
    Opt<double> calibrated_tx_power_dbm;
    TxPowerPolicy power_policy = TxPowerPolicy::LeaveUntouched;
    PulseShaping pulse_shaping = PulseShaping::ExistingHrP;
    // A vendor "power word" is a FOURTH, separately named field.  It may only
    // be set together with the backend capability id that defines its bits;
    // there is no software-side default for it.
    Opt<uint32_t> vendor_power_word;
    std::string vendor_power_word_backend;
};

// --- 接收 (receive) --------------------------------------------------------

struct ReceiveConfig {
    uint8_t port = 0; // physical RX channel index inside the device
    Opt<double> gain_db; // RX gain in dB
    AgcMode agc = AgcMode::Manual;
    Opt<double> bandwidth_hz; // RF bandwidth, distinct from the sample rate
    double detection_threshold = 0.0;    // energy/coarse gate, linear
    double correlation_threshold = 0.0;  // fine preamble correlation gate
    double first_path_threshold = 0.0;   // first-path quality gate
    uint16_t first_path_index = 0;       // first usable CIR index
    uint16_t first_path_window = 0;      // CIR taps kept for the search
    // A vendor PAC-like value is ONLY meaningful together with the backend
    // capability that defines it AND the software step the backend actually
    // applies.  Mapping a PAC value onto an arbitrary software detection step
    // is forbidden (REQ-API-01 接收).
    Opt<uint32_t> vendor_pac_value;
    std::string vendor_pac_backend;
    Opt<double> vendor_pac_applied_step;
};

// --- 无线设备 (radio) ------------------------------------------------------

struct RadioReadback {
    bool present = false;
    double sample_rate_hz = 0.0;
    double center_freq_hz = 0.0;
    uint8_t tx_channel = 0;
    uint8_t rx_channel = 0;
    std::string mpm_string;    // e.g. "CG400" / "X410"
    std::string fpga_image;    // e.g. "CG400-...-R1"
    std::string uhd_version;
    std::string clock_source;
    std::string time_source;
};

// One logical endpoint sharing the physical device.  Phase 1: two of these
// (A = TX0/RX0, B = TX1/RX1) on one X410.
struct EndpointBinding {
    std::string id; // "A" / "B"
    Role role = Role::Initiator;
    uint8_t tx_channel = 0;
    uint8_t rx_channel = 0;
    double native_sample_rate_hz = 0.0;
    bool occupies_resources = true;
};

struct RadioConfig {
    std::string device_args;
    uint8_t tx_channel = 0;
    uint8_t rx_channel = 0;
    double native_sample_rate_hz = 0.0;
    std::string clock_source; // "" = leave untouched
    std::string time_source;  // "" = leave untouched
    std::string fpga_image;   // "" = build default
    std::string dpdk_config;  // "" = no DPDK
    // The other endpoint(s) of the same session, so the config is
    // self-describing and the validator can reject channel conflicts.
    std::vector<EndpointBinding> peers;
    // Filled at startup.  A readback that differs from the request is a
    // STARTUP FAILURE (REQ-API-01 无线设备: 读回不符启动失败).
    RadioReadback readback;
    // Require the readback block to be filled in.  A dry-run/offline session
    // may set it false; a real radio must not.
    bool require_readback = true;
};

// --- 每包时间 (per-message timing) -----------------------------------------

struct PerMessageTiming {
    // When the first Poll of a measurement is transmitted.  Absolute in the
    // device clock domain when domain == DeviceTicks.
    TimedField poll_start;
    // Responder: RX Poll RMARKER -> Response TX RMARKER.  THE PROTOCOL REPLY
    // DELAY.
    TimedField poll_to_response;
    // Initiator: RX Response RMARKER -> Final TX RMARKER (DS only).  A
    // different thing from poll_to_response.
    TimedField response_to_final;
    // Report: phase 1 has no Report frame (FrameType::Report is reserved),
    // so this must stay zero.
    TimedField final_to_report;
    // After the END OF OUR OWN TRANSMITTED FRAME, delay before RX is armed.
    // THE POST-TX RX-ENABLE DELAY -- a completely different number from the
    // reply delay and from the RX timeout.
    TimedField post_tx_rx_enable;
    // Measured minimum UHD timed-command lead time.  The operator must supply
    // it; there is no default (REQ-GR-04: 未经验证的短时序不得承诺).
    TimedField min_tx_lead_time;
};

// --- 超时 (timeouts) -------------------------------------------------------

struct TimeoutConfig {
    // Per-frame RX windows.  Each is armed at RX ENABLE and measured on the
    // device clock.
    TimedField poll_rx_window;
    TimedField response_rx_window;
    TimedField final_rx_window;
    TimedField report_rx_window; // must stay zero in phase 1
    // How long to wait for the expected RX frame, measured FROM RX ENABLE.
    TimedField rx_timeout;
    // Whole-exchange deadline, measured on the HOST MONOTONIC CLOCK.
    TimedField exchange_timeout;
    // Gap between a failed attempt and its retry, host monotonic.
    TimedField retry_interval;
};

// --- 时戳与校准 (timestamps and calibration) -------------------------------

// A calibration record, versioned by device / channel / rate / gain /
// profile.  `valid_until_monotonic_ns` is 0 = no expiry.
struct CalibrationRecord {
    std::string calibration_id;
    std::string device_serial;
    uint8_t channel = 0;
    double native_sample_rate_hz = 0.0;
    std::string profile_version;
    Opt<double> gain_db; // the gain the constants were taken at
    int64_t valid_until_monotonic_ns = 0;
};

struct TimestampCalibrationConfig {
    // The link / antenna / cable delays, in integer NANOSECONDS (named type,
    // not a vendor tick).
    Duration tx_link_delay{0};
    Duration rx_link_delay{0};
    Duration antenna_delay{0};
    Duration cable_delay{0};
    // The SAME quantity expressed in DEVICE TICKS -- a separate field, never
    // added to the nanosecond fields above.
    int64_t tx_link_delay_native_ticks = 0;
    int64_t rx_link_delay_native_ticks = 0;
    double native_sample_rate_hz = 0.0;
    // Which unit the *_native_ticks fields and the Durations refer to.
    TimeUnit link_delay_unit = TimeUnit::Nanoseconds;
    FirstPathAlgorithm first_path_algorithm = FirstPathAlgorithm::LeadingEdge;
    CompensationFlag cfo_compensation = CompensationFlag::Off;
    CompensationFlag sfo_compensation = CompensationFlag::Off;
    std::string calibration_id;
    CalibrationRecord record;
    // REQ-CAL-01: a calibration is applicable EXACTLY ONCE.  `applied_count`
    // is bumped by apply_calibration_once(); a second call is rejected.
    uint32_t applied_count = 0;
    // A calibration is mandatory for a validated absolute range claim.
    bool calibration_required = true;
};

// --- 诊断 (diagnostics) ----------------------------------------------------

struct DiagnosticsConfig {
    bool cir_capture_enabled = false;
    uint64_t cir_capture_max_bytes = 0;
    uint32_t cir_capture_stride = 0;
    bool short_iq_enabled = false;
    uint64_t short_iq_max_bytes = 0;
    uint32_t short_iq_stride = 0;
    bool raw_frame_dump = false;
    uint64_t raw_frame_max_bytes = 0;
    std::string result_output_path; // "" = no file output
    uint32_t result_queue_capacity = 0;
    uint32_t event_queue_capacity = 0;
    TimedField stats_cadence; // host monotonic
    // Diagnostic I/O must never run on the realtime thread (REQ-API-01
    // 诊断).  This flag exists so that violation is a CONFIG error, not a
    // surprise discovered during a soak.
    bool io_on_realtime_thread = false;
};

// --- meta ------------------------------------------------------------------

struct ConfigMeta {
    std::string schema_version = "twr-config/2";
    std::string profile_version;
    std::string calibration_version;
    // Optional operator note carried into every result.
    std::string label;
};

// --- the config ------------------------------------------------------------

struct TwrConfig {
    ConfigMeta meta;
    SessionConfig session;
    PhyConfig phy;
    FrameFormatConfig frame;
    TransmitConfig tx;
    ReceiveConfig rx;
    RadioConfig radio;
    PerMessageTiming timing;
    TimeoutConfig timeouts;
    TimestampCalibrationConfig calibration;
    DiagnosticsConfig diagnostics;
};

// ===========================================================================
// 5. Canonical field listing (requested vs effective diff + config hash)
// ===========================================================================

// A flattened "path=value" record.  Used to diff requested against effective
// and to compute a stable config hash (REQ-OUT-01: config hash).
struct FieldRecord {
    std::string path;
    std::string value;
};

class FieldSink
{
public:
    explicit FieldSink(std::vector<FieldRecord>& out) : d_out(out) {}

    void add(const std::string& path, const std::string& v) { d_out.push_back({ path,
        v }); }
    void add_i64(const std::string& path, int64_t v) { add(path, twr_int_to_text(v)); }
    void add_u64(const std::string& path, uint64_t v)
    {
        add(path, twr_int_to_text(static_cast<int64_t>(v)));
    }
    void add_u32(const std::string& path, uint32_t v)
    {
        add(path, twr_int_to_text(static_cast<int64_t>(v)));
    }
    void add_u16(const std::string& path, uint16_t v)
    {
        add(path, twr_int_to_text(static_cast<int64_t>(v)));
    }
    void add_u8(const std::string& path, uint8_t v)
    {
        add(path, twr_int_to_text(static_cast<int64_t>(v)));
    }
    void add_bool(const std::string& path, bool v) { add(path, twr_bool_to_text(v)); }
    void add_double(const std::string& path, double v) { add(path,
        twr_double_to_text(v)); }
    void add_dur(const std::string& path, const Duration& d) { add(path,
        twr_duration_to_text(d)); }
    void add_enum(const std::string& path, const char* name) { add(path,
        name ? name : "invalid"); }

    void add_opt_i64(const std::string& path, const Opt<int64_t>& o)
    {
        add(path, o.has() ? twr_int_to_text(o.value()) : std::string("null"));
    }
    void add_opt_u32(const std::string& path, const Opt<uint32_t>& o)
    {
        add(path, o.has() ? twr_int_to_text(static_cast<int64_t>(o.value()))
                          : std::string("null"));
    }
    void add_opt_double(const std::string& path, const Opt<double>& o)
    {
        add(path, o.has() ? twr_double_to_text(o.value()) : std::string("null"));
    }
    void add_opt_str(const std::string& path, const Opt<std::string>& o)
    {
        add(path, o.has() ? o.value() : std::string("null"));
    }
    void add_opt_marker(const std::string& path, const Opt<TimestampMarker>& o)
    {
        add(path,
            o.has() ? std::string(timestamp_marker_to_string(o.value())) : std::string("null"));
    }
    void add_timed(const std::string& path, const TimedField& f) { add(path,
        f.to_text()); }
    void add_record(const std::string& path, const std::string& v) { add(path, v); }

private:
    std::vector<FieldRecord>& d_out;
};

inline void collect_timed_fields(FieldSink& s, const std::string& p,
    const TimedField& f)
{
    s.add(p + ".ns", twr_int_to_text(f.value.nanos()));
    s.add_enum(p + ".domain", time_domain_to_string(f.domain));
    s.add_enum(p + ".reference", time_reference_event_to_string(f.reference));
    s.add_opt_marker(p + ".marker", f.marker);
    s.add_double(p + ".quantisation_hz", f.required_quantisation_hz);
    s.add_i64(p + ".max_quantisation_error_ns", f.max_quantisation_error_ns);
    s.add(p + ".note", f.note);
}

inline void collect_fields(FieldSink& s, const ConfigMeta& v, const std::string& p)
{
    s.add(p + ".schema_version", v.schema_version);
    s.add(p + ".profile_version", v.profile_version);
    s.add(p + ".calibration_version", v.calibration_version);
    s.add(p + ".label", v.label);
}

inline void collect_fields(FieldSink& s, const SessionConfig& v, const std::string& p)
{
    s.add_enum(p + ".protocol", protocol_to_string(v.protocol));
    s.add_enum(p + ".role", role_to_string(v.role));
    s.add_u16(p + ".local_address", v.local_address);
    s.add_u16(p + ".peer_address", v.peer_address);
    s.add_u16(p + ".pan_id", v.pan_id);
    s.add_u32(p + ".session_id", v.session_id);
    s.add_u32(p + ".exchange_id", v.exchange_id);
    s.add_u8(p + ".sequence", v.sequence);
    s.add_u16(p + ".sequence_modulus", v.sequence_modulus);
    s.add_u32(p + ".measurement_count", v.measurement_count);
    s.add_dur(p + ".measurement_interval", v.measurement_interval);
    s.add_u32(p + ".max_attempts_per_exchange", v.max_attempts_per_exchange);
    s.add_dur(p + ".retry_backoff", v.retry_backoff);
    s.add_u32(p + ".max_in_flight_exchanges", v.max_in_flight_exchanges);
    s.add_bool(p + ".require_pan_match", v.require_pan_match);
    s.add_bool(p + ".require_address_match", v.require_address_match);
}

inline void collect_fields(FieldSink& s, const PhyConfig& v, const std::string& p)
{
    s.add_u8(p + ".channel", v.channel);
    s.add_double(p + ".center_frequency_hz", v.center_frequency_hz);
    s.add_u8(p + ".tx_preamble_code", v.tx_preamble_code);
    s.add_u8(p + ".rx_preamble_code", v.rx_preamble_code);
    s.add_u16(p + ".preamble_symbols", v.preamble_symbols);
    s.add_enum(p + ".prf_class", prf_class_to_string(v.prf_class));
    s.add_enum(p + ".data_rate", data_rate_to_string(v.data_rate));
    s.add_enum(p + ".phr_rate", phr_rate_to_string(v.phr_rate));
}

inline void collect_fields(FieldSink& s, const FrameGeometry& v, const std::string& p)
{
    s.add_u16(p + ".mac_header_bytes", v.mac_header_bytes);
    s.add_u16(p + ".timestamp_bytes", v.timestamp_bytes);
    s.add_u16(p + ".mac_footer_bytes", v.mac_footer_bytes);
    s.add_u16(p + ".mac_fcs_bytes", v.mac_fcs_bytes);
    s.add_u16(p + ".phr_bytes", v.phr_bytes);
}

inline void collect_fields(FieldSink& s, const FrameFormatConfig& v,
    const std::string& p)
{
    s.add_enum(p + ".frame_profile", frame_profile_id_to_string(v.frame_profile));
    s.add_enum(p + ".sfd_mode", sfd_mode_to_string(v.sfd_mode));
    s.add_u16(p + ".sfd_symbols", v.sfd_symbols);
    collect_timed_fields(s, p + ".sfd_timeout", v.sfd_timeout);
    s.add_enum(p + ".phr_mode", phr_mode_to_string(v.phr_mode));
    s.add_bool(p + ".ranging_bit", v.ranging_bit);
    // The geometry CLAIM goes into the hash verbatim: two configs that claim
    // different layouts are different configs, even though the codec will
    // reject one of them.
    collect_fields(s, v.geometry, p + ".geometry");
    s.add_u16(p + ".mac_psdu_bytes", v.mac_psdu_bytes);
    s.add_bool(p + ".mac_psdu_includes_fcs", v.mac_psdu_includes_fcs);
    s.add_enum(p + ".fcs_append", fcs_appender_to_string(v.fcs_append));
    s.add_u16(p + ".fcs_bytes", v.fcs_bytes);
    s.add_u16(p + ".application_payload_bytes", v.application_payload_bytes);
    s.add_enum(p + ".sts_mode", sts_mode_to_string(v.sts_mode));
    s.add_u16(p + ".sts_length_symbols", v.sts_length_symbols);
}

inline void collect_fields(FieldSink& s, const TransmitConfig& v, const std::string& p)
{
    s.add_u8(p + ".port", v.port);
    s.add_opt_double(p + ".gain_db", v.gain_db);
    s.add_opt_double(p + ".iq_amplitude", v.iq_amplitude);
    s.add_opt_double(p + ".calibrated_tx_power_dbm", v.calibrated_tx_power_dbm);
    s.add_enum(p + ".power_policy", tx_power_policy_to_string(v.power_policy));
    s.add_enum(p + ".pulse_shaping", pulse_shaping_to_string(v.pulse_shaping));
    s.add_opt_u32(p + ".vendor_power_word", v.vendor_power_word);
    s.add(p + ".vendor_power_word_backend", v.vendor_power_word_backend);
}

inline void collect_fields(FieldSink& s, const ReceiveConfig& v, const std::string& p)
{
    s.add_u8(p + ".port", v.port);
    s.add_opt_double(p + ".gain_db", v.gain_db);
    s.add_enum(p + ".agc", agc_mode_to_string(v.agc));
    s.add_opt_double(p + ".bandwidth_hz", v.bandwidth_hz);
    s.add_double(p + ".detection_threshold", v.detection_threshold);
    s.add_double(p + ".correlation_threshold", v.correlation_threshold);
    s.add_double(p + ".first_path_threshold", v.first_path_threshold);
    s.add_u16(p + ".first_path_index", v.first_path_index);
    s.add_u16(p + ".first_path_window", v.first_path_window);
    s.add_opt_u32(p + ".vendor_pac_value", v.vendor_pac_value);
    s.add(p + ".vendor_pac_backend", v.vendor_pac_backend);
    s.add_opt_double(p + ".vendor_pac_applied_step", v.vendor_pac_applied_step);
}

inline void collect_fields(FieldSink& s, const EndpointBinding& v, const std::string& p)
{
    s.add(p + ".id", v.id);
    s.add_enum(p + ".role", role_to_string(v.role));
    s.add_u8(p + ".tx_channel", v.tx_channel);
    s.add_u8(p + ".rx_channel", v.rx_channel);
    s.add_double(p + ".native_sample_rate_hz", v.native_sample_rate_hz);
    s.add_bool(p + ".occupies_resources", v.occupies_resources);
}

inline void collect_fields(FieldSink& s, const RadioReadback& v, const std::string& p)
{
    s.add_bool(p + ".present", v.present);
    s.add_double(p + ".sample_rate_hz", v.sample_rate_hz);
    s.add_double(p + ".center_freq_hz", v.center_freq_hz);
    s.add_u8(p + ".tx_channel", v.tx_channel);
    s.add_u8(p + ".rx_channel", v.rx_channel);
    s.add(p + ".mpm_string", v.mpm_string);
    s.add(p + ".fpga_image", v.fpga_image);
    s.add(p + ".uhd_version", v.uhd_version);
    s.add(p + ".clock_source", v.clock_source);
    s.add(p + ".time_source", v.time_source);
}

inline void collect_fields(FieldSink& s, const RadioConfig& v, const std::string& p)
{
    s.add(p + ".device_args", v.device_args);
    s.add_u8(p + ".tx_channel", v.tx_channel);
    s.add_u8(p + ".rx_channel", v.rx_channel);
    s.add_double(p + ".native_sample_rate_hz", v.native_sample_rate_hz);
    s.add(p + ".clock_source", v.clock_source);
    s.add(p + ".time_source", v.time_source);
    s.add(p + ".fpga_image", v.fpga_image);
    s.add(p + ".dpdk_config", v.dpdk_config);
    for (size_t i = 0; i < v.peers.size(); ++i)
        collect_fields(s, v.peers[i], p + ".peers[" +
            twr_int_to_text(static_cast<int64_t>(i)) + "]");
    collect_fields(s, v.readback, p + ".readback");
    s.add_bool(p + ".require_readback", v.require_readback);
}

inline void collect_fields(FieldSink& s, const PerMessageTiming& v,
    const std::string& p)
{
    collect_timed_fields(s, p + ".poll_start", v.poll_start);
    collect_timed_fields(s, p + ".poll_to_response", v.poll_to_response);
    collect_timed_fields(s, p + ".response_to_final", v.response_to_final);
    collect_timed_fields(s, p + ".final_to_report", v.final_to_report);
    collect_timed_fields(s, p + ".post_tx_rx_enable", v.post_tx_rx_enable);
    collect_timed_fields(s, p + ".min_tx_lead_time", v.min_tx_lead_time);
}

inline void collect_fields(FieldSink& s, const TimeoutConfig& v, const std::string& p)
{
    collect_timed_fields(s, p + ".poll_rx_window", v.poll_rx_window);
    collect_timed_fields(s, p + ".response_rx_window", v.response_rx_window);
    collect_timed_fields(s, p + ".final_rx_window", v.final_rx_window);
    collect_timed_fields(s, p + ".report_rx_window", v.report_rx_window);
    collect_timed_fields(s, p + ".rx_timeout", v.rx_timeout);
    collect_timed_fields(s, p + ".exchange_timeout", v.exchange_timeout);
    collect_timed_fields(s, p + ".retry_interval", v.retry_interval);
}

inline void collect_fields(FieldSink& s, const CalibrationRecord& v,
    const std::string& p)
{
    s.add(p + ".calibration_id", v.calibration_id);
    s.add(p + ".device_serial", v.device_serial);
    s.add_u8(p + ".channel", v.channel);
    s.add_double(p + ".native_sample_rate_hz", v.native_sample_rate_hz);
    s.add(p + ".profile_version", v.profile_version);
    s.add_opt_double(p + ".gain_db", v.gain_db);
    s.add_i64(p + ".valid_until_monotonic_ns", v.valid_until_monotonic_ns);
}

inline void collect_fields(FieldSink& s, const TimestampCalibrationConfig& v,
    const std::string& p)
{
    s.add_dur(p + ".tx_link_delay", v.tx_link_delay);
    s.add_dur(p + ".rx_link_delay", v.rx_link_delay);
    s.add_dur(p + ".antenna_delay", v.antenna_delay);
    s.add_dur(p + ".cable_delay", v.cable_delay);
    s.add_i64(p + ".tx_link_delay_native_ticks", v.tx_link_delay_native_ticks);
    s.add_i64(p + ".rx_link_delay_native_ticks", v.rx_link_delay_native_ticks);
    s.add_double(p + ".native_sample_rate_hz", v.native_sample_rate_hz);
    s.add_enum(p + ".link_delay_unit", time_unit_to_string(v.link_delay_unit));
    s.add_enum(p + ".first_path_algorithm",
        first_path_algorithm_to_string(v.first_path_algorithm));
    s.add_enum(p + ".cfo_compensation",
        compensation_flag_to_string(v.cfo_compensation));
    s.add_enum(p + ".sfo_compensation",
        compensation_flag_to_string(v.sfo_compensation));
    s.add(p + ".calibration_id", v.calibration_id);
    collect_fields(s, v.record, p + ".record");
    s.add_u32(p + ".applied_count", v.applied_count);
    s.add_bool(p + ".calibration_required", v.calibration_required);
}

inline void collect_fields(FieldSink& s, const DiagnosticsConfig& v,
    const std::string& p)
{
    s.add_bool(p + ".cir_capture_enabled", v.cir_capture_enabled);
    s.add_u64(p + ".cir_capture_max_bytes", v.cir_capture_max_bytes);
    s.add_u32(p + ".cir_capture_stride", v.cir_capture_stride);
    s.add_bool(p + ".short_iq_enabled", v.short_iq_enabled);
    s.add_u64(p + ".short_iq_max_bytes", v.short_iq_max_bytes);
    s.add_u32(p + ".short_iq_stride", v.short_iq_stride);
    s.add_bool(p + ".raw_frame_dump", v.raw_frame_dump);
    s.add_u64(p + ".raw_frame_max_bytes", v.raw_frame_max_bytes);
    s.add(p + ".result_output_path", v.result_output_path);
    s.add_u32(p + ".result_queue_capacity", v.result_queue_capacity);
    s.add_u32(p + ".event_queue_capacity", v.event_queue_capacity);
    collect_timed_fields(s, p + ".stats_cadence", v.stats_cadence);
    s.add_bool(p + ".io_on_realtime_thread", v.io_on_realtime_thread);
}

inline void collect_fields(FieldSink& s, const TwrConfig& v, const std::string& p)
{
    collect_fields(s, v.meta, p + "meta");
    collect_fields(s, v.session, p + "session");
    collect_fields(s, v.phy, p + "phy");
    collect_fields(s, v.frame, p + "frame");
    collect_fields(s, v.tx, p + "tx");
    collect_fields(s, v.rx, p + "rx");
    collect_fields(s, v.radio, p + "radio");
    collect_fields(s, v.timing, p + "timing");
    collect_fields(s, v.timeouts, p + "timeouts");
    collect_fields(s, v.calibration, p + "calibration");
    collect_fields(s, v.diagnostics, p + "diagnostics");
}

// Ordered "path=value" listing.  Stable across runs and across builds, so it
// can be hashed and diffed.
inline std::vector<FieldRecord> flatten_fields(const TwrConfig& cfg)
{
    std::vector<FieldRecord> out;
    out.reserve(256);
    FieldSink s(out);
    collect_fields(s, cfg, std::string());
    return out;
}

inline std::string canonical_text(const std::vector<FieldRecord>& fields)
{
    std::string out;
    out.reserve(fields.size() * 24);
    for (const auto& f : fields) {
        out += f.path;
        out += '=';
        out += f.value;
        out += '\n';
    }
    return out;
}

// FNV-1a 64 over the canonical text.  Not a security hash: an identifier that
// appears in every result so a config can be tied to the run that used it.
inline std::string config_hash(const TwrConfig& cfg)
{
    const std::string text = canonical_text(flatten_fields(cfg));
    uint64_t h = 1469598103934665603ull;
    for (unsigned char c : text) {
        h ^= static_cast<uint64_t>(c);
        h *= 1099511628211ull;
    }
    char buf[32];
    std::snprintf(buf, sizeof(buf), "fnv1a64:%016llx",
        static_cast<unsigned long long>(h));
    return std::string(buf);
}

// One requested-vs-effective difference.
struct FieldChange {
    std::string path;
    std::string requested;
    std::string effective;
    std::string note;
    bool adjusted() const { return requested != effective; }
};

inline std::vector<FieldChange>
diff_fields(const TwrConfig& requested, const TwrConfig& effective)
{
    const std::vector<FieldRecord> a = flatten_fields(requested);
    const std::vector<FieldRecord> b = flatten_fields(effective);
    std::vector<FieldChange> out;
    const size_t n = a.size() < b.size() ? a.size() : b.size();
    for (size_t i = 0; i < n; ++i) {
        if (a[i].path != b[i].path || a[i].value == b[i].value)
            continue;
        FieldChange c;
        c.path = a[i].path;
        c.requested = a[i].value;
        c.effective = b[i].value;
        out.push_back(c);
    }
    return out;
}

// ===========================================================================
// 6. The validator
// ===========================================================================

// Every check appends a specific violation.  There is no early return: a
// config that is wrong in many places is reported in full so the operator
// fixes them in one pass.
class ConfigValidator
{
public:
    explicit ConfigValidator(const Capabilities& caps) : d_caps(caps) {}

    ValidationReport run(const TwrConfig& cfg) const
    {
        d_report = ValidationReport{};
        check_meta(cfg);
        check_session(cfg);
        check_phy(cfg);
        check_frame(cfg);
        check_tx(cfg);
        check_rx(cfg);
        check_radio(cfg);
        check_timing(cfg);
        check_timeouts(cfg);
        check_calibration(cfg);
        check_diagnostics(cfg);
        check_frame_lengths(cfg);
        return d_report;
    }

private:
    // -- helpers ---------------------------------------------------------
    void rej(const std::string& f,
             ConfigReason r,
             ExchangeStatus s,
             const std::string& m,
             const char* req) const
    {
        d_report.add(f, r, s, m, req);
    }
    void cfg_rej(const std::string& f,
                 ConfigReason r,
                 const std::string& m,
                 const char* req = "REQ-API-01") const
    {
        rej(f, r, ExchangeStatus::ConfigRejected, m, req);
    }
    void unsup(const std::string& f,
               const std::string& m,
               ConfigReason r = ConfigReason::Unsupported,
               const char* req = "REQ-SCOPE-01") const
    {
        rej(f, r, ExchangeStatus::Unsupported, m, req);
    }
    // A float field that must be finite.
    void finite(const std::string& f, double v, const char* req = "REQ-API-01") const
    {
        if (!std::isfinite(v))
            cfg_rej(f, ConfigReason::NotFinite, "must be finite (NaN/Inf rejected)",
                req);
    }
    void check_finite_opt(const std::string& f, const Opt<double>& o) const
    {
        if (o.has() && !std::isfinite(o.value()))
            cfg_rej(f, ConfigReason::NotFinite,
                "must be finite when present (NaN/Inf rejected)");
    }
    void check_finite_timed(const std::string& f, const TimedField& t) const
    {
        finite(f + ".quantisation_hz", t.required_quantisation_hz);
        if (t.value.negative())
            cfg_rej(f, ConfigReason::NegativeValue,
                "a delay or window may not be negative");
    }

    // -- sections --------------------------------------------------------
    void check_meta(const TwrConfig& c) const
    {
        if (c.meta.schema_version.empty())
            cfg_rej("meta.schema_version", ConfigReason::EmptyValue,
                "schema_version is required");
        if (c.meta.schema_version != d_caps.schema_version) {
            // M0.1: twr-config/2 is a BREAKING change -- frame.frame_profile
            // became required and phy.phr_rate changed value domain.  A v1
            // document is refused with a migration message, never silently
            // misread as a v2 one.
            if (c.meta.schema_version == "twr-config/1")
                unsup("meta.schema_version",
                      "twr-config/1 is superseded by twr-config/2: "
                      "frame.frame_profile is now REQUIRED (frame_v1) and "
                      "phy.phr_rate now takes 850k (the PHR rate the modulator "
                      "actually uses) rather than the payload rate; the frame "
                      "geometry moved from the old 7-byte header to the frozen "
                      "14-byte header, so on-air sizes are 16/26/31 B and "
                      "session.session_id must fit 16 bits. This document is "
                      "refused rather than reinterpreted");
            else
                unsup("meta.schema_version",
                      "this build implements " + d_caps.schema_version +
                          ", config asks for '" + c.meta.schema_version + "'");
        }
        if (c.meta.profile_version.empty())
            cfg_rej("meta.profile_version",
                    ConfigReason::EmptyValue,
                    "profile_version is required (REQ-OUT-01 calibration/profile/config hash)");
    }

    void check_session(const TwrConfig& c) const
    {
        const SessionConfig& s = c.session;
        if (s.local_address == s.peer_address)
            cfg_rej("session.peer_address",
                    ConfigReason::FieldConflict,
                    "local and peer address are identical (would make every frame a match)");
        if (s.local_address == 0x0000 || s.local_address == 0xffff)
            cfg_rej("session.local_address",
                    ConfigReason::FieldConflict,
                    "0x0000 and 0xffff are reserved/invalid local addresses");
        if (s.peer_address == 0x0000 || s.peer_address == 0xffff)
            cfg_rej("session.peer_address",
                    ConfigReason::FieldConflict,
                    "0x0000 and 0xffff are reserved/invalid peer addresses");
        if (s.pan_id == 0x0000 || s.pan_id == 0xffff)
            cfg_rej("session.pan_id", ConfigReason::EmptyValue,
                "PAN 0x0000/0xffff is not usable");
        if (s.session_id == 0)
            cfg_rej("session.session_id",
                    ConfigReason::ZeroValue,
                    "0 is the frame profile's 'no session' marker");
        else {
            // The wire field at kOffSessionId is 16 bits.  session_id_to_wire()
            // is identity-or-refuse: in range -> exact, out of range -> an
            // explicit error.  It is NOT folded, masked or hashed, because a
            // lossy map makes two different local sessions produce a
            // byte-identical session field and frame_match() would then accept
            // the wrong session's reply as this one's.
            uint16_t wire = 0;
            std::string sid_err_text;
            SessionIdError sid_err = SessionIdError::None;
            if (!session_id_to_wire(s.session_id, wire, sid_err_text, &sid_err)) {
                rej("session.session_id",
                    ConfigReason::OutOfRange,
                    session_id_error_to_exchange_status(sid_err),
                    sid_err_text,
                    "REQ-PROTO-01");
            }
        }
        if (s.sequence_modulus == 0)
            cfg_rej("session.sequence_modulus", ConfigReason::ZeroValue,
                "wrap modulus is zero");
        else if (s.sequence_modulus != 4 && s.sequence_modulus != 16 &&
                 s.sequence_modulus != 64 && s.sequence_modulus != 256)
            cfg_rej("session.sequence_modulus",
                    ConfigReason::OutOfRange,
                    "must be 4, 16, 64 or 256 (2/4/6/8 sequence bits)");
        if (static_cast<uint16_t>(s.sequence) >= s.sequence_modulus)
            cfg_rej("session.sequence",
                    ConfigReason::IndexOutOfRange,
                    "sequence does not fit the wrap modulus");
        if (s.measurement_count == 0 || s.measurement_count > kTwrMaxMeasurementCount)
            cfg_rej("session.measurement_count",
                    ConfigReason::OutOfRange,
                    "must be 1.." + twr_int_to_text(kTwrMaxMeasurementCount));
        if (s.measurement_interval.negative())
            cfg_rej("session.measurement_interval", ConfigReason::NegativeValue,
                "negative interval");
        if (s.max_attempts_per_exchange == 0 ||
            s.max_attempts_per_exchange > kTwrMaxAttemptsPerExchange)
            cfg_rej("session.max_attempts_per_exchange",
                    ConfigReason::OutOfRange,
                    "must be 1.." + twr_int_to_text(kTwrMaxAttemptsPerExchange));
        if (s.retry_backoff.negative())
            cfg_rej("session.retry_backoff", ConfigReason::NegativeValue,
                "negative backoff");
        if (s.max_attempts_per_exchange > 1) {
            if (s.retry_backoff.is_zero())
                cfg_rej("session.retry_backoff",
                        ConfigReason::FieldConflict,
                        "a retry policy with more than one attempt needs a non-zero backoff");
        } else if (!s.retry_backoff.is_zero()) {
            cfg_rej("session.retry_backoff",
                    ConfigReason::FieldConflict,
                    "retry_backoff is set but max_attempts_per_exchange is 1");
        }
        if (s.max_in_flight_exchanges != kTwrMaxInFlightExchanges)
            unsup("session.max_in_flight_exchanges",
                  "phase 1 allows at most " + twr_int_to_text(kTwrMaxInFlightExchanges) +
                      " in-flight exchange per endpoint; requested " +
                      twr_int_to_text(s.max_in_flight_exchanges),
                  ConfigReason::InFlightNotSupported,
                  "REQ-API-01");
    }

    void check_phy(const TwrConfig& c) const
    {
        const PhyConfig& p = c.phy;
        if (p.channel > 16u)
            cfg_rej("phy.channel", ConfigReason::IndexOutOfRange,
                "channel must be 0..16");
        else if (!d_caps.channel_supported(p.channel))
            unsup("phy.channel",
                  "channel " + twr_int_to_text(p.channel) +
                      " is not in this build's capability whitelist; the phase-1 common profile "
                      "is channel 5 (REQ-PHY-01/03) and other channels are per-channel "
                      "acceptance items");

        // Channel vs centre frequency: a conflict is rejected, never ignored.
        finite("phy.center_frequency_hz", p.center_frequency_hz, "REQ-PHY-01");
        if (std::isfinite(p.center_frequency_hz) && p.center_frequency_hz > 0.0) {
            const double plan = uwb_channel_center_frequency_hz(p.channel);
            if (plan <= 0.0) {
                cfg_rej("phy.center_frequency_hz",
                        ConfigReason::IndexOutOfRange,
                        "no channel plan entry for this channel");
            } else if (std::fabs(p.center_frequency_hz - plan) > kTwrChannelFrequencyToleranceHz) {
                cfg_rej("phy.center_frequency_hz",
                        ConfigReason::ChannelFrequencyMismatch,
                        "channel " + twr_int_to_text(p.channel) + " is " +
                            twr_double_to_text(plan) + " Hz, config states " +
                            twr_double_to_text(p.center_frequency_hz) + " Hz",
                        "REQ-PHY-01");
            }
        }

        // code / PRF / channel / profile are cross-validated together.
        for (int dir = 0; dir < 2; ++dir) {
            const std::string f =
                dir == 0 ? "phy.tx_preamble_code" : "phy.rx_preamble_code";
            const uint8_t code = dir == 0 ? p.tx_preamble_code : p.rx_preamble_code;
            if (code == 0)
                cfg_rej(f, ConfigReason::ZeroValue, "preamble code is required");
            else if (!d_caps.code_index_supported(code))
                unsup(f,
                      "preamble code " + twr_int_to_text(code) +
                          " is not in the capability whitelist (" +
                          twr_int_to_text(kTwrCodeIndexMin) + ".." +
                          twr_int_to_text(kTwrCodeIndexMax) + ")");
        }
        if (p.tx_preamble_code != 0 && p.rx_preamble_code != 0 &&
            p.tx_preamble_code != p.rx_preamble_code)
            cfg_rej("phy.rx_preamble_code",
                    ConfigReason::FieldConflict,
                    "TX and RX preamble codes differ (" + twr_int_to_text(p.tx_preamble_code) +
                        " vs " + twr_int_to_text(p.rx_preamble_code) +
                        "); one profile uses one code",
                    "REQ-PHY-01");
        if (!d_caps.prf_class_supported(p.prf_class))
            unsup("phy.prf_class",
                  std::string("PRF class ") + prf_class_to_string(p.prf_class) +
                      " is not in the capability whitelist");
        // The PHR rate is validated on its OWN axis.  It is NOT compared to
        // the payload data rate: the PHR is a different modulation at a
        // different rate, and the data-rate field INSIDE the PHR describes
        // the payload, not the PHR's own transmission.  M0's `phr_rate ==
        // data_rate` rule therefore accepted the 6.81 Mb/s PHR this modulator
        // cannot produce and rejected the 0.85 Mb/s PHR it does produce.
        if (!phr_rate_is_implemented(p.phr_rate)) {
            unsup("phy.phr_rate",
                  "PHR rate " + std::string(phr_rate_to_string(p.phr_rate)) +
                      " is not implemented: " + phr_rate_unsupported_reason(p.phr_rate) +
                      " It is NOT compared with phy.data_rate (" +
                      data_rate_to_string(p.data_rate) +
                      "): the PHR and the payload are separate modulations, and the "
                      "data-rate field carried inside the PHR describes the payload",
                  ConfigReason::Unsupported,
                  "REQ-PHY-02");
        }

        // MEASURED-PENDING: sync repetitions / SFD mode / data rate.
        if (!d_caps.data_rate_supported(p.data_rate))
            unsup("phy.data_rate",
                  std::string("data rate ") + data_rate_to_string(p.data_rate) +
                      " is not in the capability whitelist: " + d_caps.pending_reason,
                  ConfigReason::Unsupported,
                  "REQ-PHY-01");
        if (p.preamble_symbols == 0)
            cfg_rej("phy.preamble_symbols", ConfigReason::ZeroValue,
                "preamble length is required");
        else {
            // Gate 1: the length must be a member of PreambleLength, i.e. one
            // this repository's demodulator / CFO path can actually run.  The
            // reason names THAT limit and never a chip's capability: the
            // Qorvo parts' own API accepts 64 through 2048, so "the hardware
            // cannot do 128" would be a false statement about hardware this
            // project has never spoken to.
            PreambleLength plen = PreambleLength::Sym64;
            if (!preamble_length_from_symbols(p.preamble_symbols, plen)) {
                unsup("phy.preamble_symbols",
                      preamble_length_unsupported_reason(p.preamble_symbols),
                      ConfigReason::Unsupported,
                      "REQ-PHY-01");
            }
            // Gate 2: the length must also be MEASURED for this (native rate,
            // code, SFD, PSDU, ranging) combination.  Separate from gate 1 on
            // purpose: a length can be implemented and still unmeasured.
            if (!d_caps.sync_repetitions_supported(p.preamble_symbols)) {
                // M0.1 R7: state the SCOPE of the refusal.  For the lengths
                // Qorvo parts do support, this build's decoder / CFO path is
                // the limit -- that is not a hardware claim, and the note
                // carries the vendor citation so C++ and the CSV agree.
                std::string note = evidence::preamble_length_supported_by_vendor(
                                       p.preamble_symbols)
                                       ? std::string(" [") +
                                             evidence::preamble_length_reject_note(
                                                 p.preamble_symbols) +
                                             "]"
                                       : std::string();
                unsup("phy.preamble_symbols",
                      "preamble length " + twr_int_to_text(p.preamble_symbols) +
                          " is not in the capability whitelist: " +
                          d_caps.pending_reason + note,
                      ConfigReason::Unsupported,
                      "REQ-PHY-01");
            }
        }
    }

    void check_frame(const TwrConfig& c) const
    {
        const FrameFormatConfig& f = c.frame;

        // ---- which layout? -------------------------------------------------
        // The profile is an ENUM, so "an arbitrary non-empty profile string"
        // is not expressible.  An id this build has no implementation for is
        // refused with its name, never defaulted to frame v1.
        if (!frame_profile_id_is_supported(f.frame_profile)) {
            unsup("frame.frame_profile",
                  "frame profile " + std::string(frame_profile_id_to_string(f.frame_profile)) +
                      " is not implemented: only the ids named by "
                      "frame_profile_id_to_string() have a codec, a geometry authority and an "
                      "encoder in this build. A free-form profile name is never an executable "
                      "geometry",
                  ConfigReason::Unsupported,
                  "REQ-PROTO-06");
        }
        // The authority this config is checked against, and the codec profile
        // whose timestamp width it reports.  Both come from the enum; neither
        // is read out of the config.
        FrameProfileGeometry authority;
        FrameProfile profile;
        const bool have_profile =
            frame_profile_id_is_supported(f.frame_profile) &&
            frame_geometry_for(f.frame_profile, authority) &&
            frame_profile_for(f.frame_profile, profile);

        if (!d_caps.sfd_mode_supported(f.sfd_mode))
            unsup("frame.sfd_mode",
                  std::string("SFD mode ") + sfd_mode_to_string(f.sfd_mode) +
                      " is not in the capability whitelist: " + d_caps.pending_reason,
                  ConfigReason::Unsupported,
                  "REQ-PHY-01");
        // The raw SFD length is a RESTATEMENT of the mode, never an
        // independent number: "4z2" and 4 symbols cannot both be stated.
        if (f.sfd_symbols == 0)
            cfg_rej("frame.sfd_symbols", ConfigReason::ZeroValue,
                "SFD length is required");
        else if (f.sfd_symbols != sfd_mode_symbols(f.sfd_mode))
            cfg_rej("frame.sfd_symbols",
                    ConfigReason::FieldConflict,
                    std::string("SFD mode ") + sfd_mode_to_string(f.sfd_mode) + " is " +
                        twr_int_to_text(sfd_mode_symbols(f.sfd_mode)) + " symbols, config states " +
                        twr_int_to_text(f.sfd_symbols) +
                        "; the length is derived from the mode, not chosen beside it",
                    "REQ-PHY-01");

        // PHR presence/form.  Each unimplemented member carries its own reason
        // (phr_mode_unsupported_reason); M0 caught only `none` and let
        // `extended` through as a valid profile.
        if (f.phr_mode != PhrMode::Standard) {
            unsup("frame.phr_mode",
                  std::string("PHR mode ") + phr_mode_to_string(f.phr_mode) +
                      " is not implemented: " + phr_mode_unsupported_reason(f.phr_mode),
                  f.phr_mode == PhrMode::None ? ConfigReason::OutOfScope
                                              : ConfigReason::Unsupported,
                  "REQ-PHY-01");
        }
        if (d_caps.ranging_bit_required && !f.ranging_bit)
            cfg_rej("frame.ranging_bit",
                    ConfigReason::FieldConflict,
                    "a TWR MAC frame must set the ranging bit; ranging_bit=false is not a valid "
                    "TWR profile",
                    "REQ-PROTO-05");
        if (!d_caps.ranging_bit_required && f.ranging_bit)
            cfg_rej("frame.ranging_bit",
                    ConfigReason::FieldConflict,
                    "this build's whitelist carries no ranging frame",
                    "REQ-PROTO-05");

        check_finite_timed("frame.sfd_timeout", f.sfd_timeout);
        validate_timed_field("frame.sfd_timeout", f.sfd_timeout, /*required=*/false,
                             TimeReferenceEvent::RxEnable, /*require_marker=*/true);

        // Exactly one layer appends the FCS, and WHICH one is not free.
        // A config that names the MAC as the appender is DESCRIBABLE -- the
        // geometry authority has a variant for it, frame_geometry_check()
        // reports the mac_fcs_bytes that layout requires, and the on-air
        // length does not change -- but it is not ENCODABLE here:
        // FrameProfile::fcs_appended_by_modulation_layer is true, so
        // frame_profile_validate() rejects it, and
        // FrameProfileGeometry::executable() is false for it.  M0 accepted
        // such a configuration and then reported a "MAC PSDU" no encoder can
        // produce, which is the R3 class of defect one layer down.
        if (f.fcs_append == FcsAppender::None)
            cfg_rej("frame.fcs_append",
                    ConfigReason::FieldConflict,
                    "no layer appends the FCS; exactly one of mac/phy must");
        else if (f.fcs_append == FcsAppender::MacLayer && have_profile) {
            unsup("frame.fcs_append",
                  "fcs_append=mac asks for a MAC that builds and appends the FCS itself. The "
                  "geometry is DESCRIBABLE and the on-air length is unchanged, but this codec "
                  "only encodes the PHY-appends form: FrameProfileGeometry::executable() is false "
                  "for the MAC-appends variant and frame_profile_validate() rejects "
                  "fcs_appended_by_modulation_layer=false",
                  ConfigReason::Unsupported,
                  "REQ-API-01");
        }
        if (f.fcs_bytes == 0)
            cfg_rej("frame.fcs_bytes", ConfigReason::ZeroValue,
                "FCS length is required");
        else if (have_profile &&
                 f.fcs_bytes != static_cast<uint16_t>(authority.fcs_bytes_on_air()))
            unsup("frame.fcs_bytes",
                  "frame profile " + std::string(frame_profile_id_to_string(f.frame_profile)) +
                      " has a " + twr_int_to_text(authority.fcs_bytes_on_air()) +
                      "-byte FCS (IEEE 802.15.4), the config states " +
                      twr_int_to_text(f.fcs_bytes) + " bytes",
                  ConfigReason::Unsupported,
                  "REQ-API-01");
        const bool declared_includes = (f.fcs_append == FcsAppender::MacLayer);
        if (f.mac_psdu_includes_fcs != declared_includes)
            cfg_rej("frame.mac_psdu_includes_fcs",
                    ConfigReason::FieldConflict,
                    std::string("fcs_append=") + fcs_appender_to_string(f.fcs_append) +
                        " but mac_psdu_includes_fcs=" + twr_bool_to_text(f.mac_psdu_includes_fcs) +
                        "; exactly one layer appends the FCS",
                    "REQ-API-01");

        // ---- the geometry CLAIM, field by field, against the codec ---------
        if (!f.geometry.any_set()) {
            cfg_rej("frame.geometry",
                    ConfigReason::EmptyValue,
                    "the frame profile geometry must be stated explicitly; it is never assumed. "
                    "Fill it with frame_geometry_of_profile(frame.frame_profile) so the claim "
                    "comes from the codec rather than from a literal");
        } else if (have_profile) {
            // A claim that disagrees with the codec is a REJECTION PER FIELD,
            // not a silently-corrected value: an operator who believes the
            // header is 7 bytes must be told it is 14, not handed a working
            // config that says something else.
            //
            // The authority the claim is checked against follows the FCS owner
            // the config NAMES, not the one it should name: a MAC-appends
            // layout is described by frame_geometry_mac_appends_fcs(), which
            // expects the 2 FCS bytes inside the MAC PSDU.  That way a
            // MAC-appends claim that forgets to reserve them is caught as the
            // field disagreement it is, and the separate `fcs_append`
            // rejection above says the layout is not encodable here at all.
            const FrameProfileGeometry& named_authority =
                (f.fcs_append == FcsAppender::MacLayer)
                    ? frame_geometry_mac_appends_fcs(f.frame_profile)
                    : authority;
            const GeometryCheckResult geo = frame_geometry_check(
                frame_geometry_claim(f.geometry), named_authority, profile);
            for (size_t i = 0; i < geo.fields.count; ++i) {
                const GeometryMismatch& m = geo.fields.items[i];
                cfg_rej(std::string("frame.geometry.") + geometry_field_to_string(m.field),
                        ConfigReason::FieldConflict,
                        std::string("the frame codec is the geometry authority: profile ") +
                            frame_profile_id_to_string(f.frame_profile) + " with FCS appended by " +
                            fcs_owner_to_string(named_authority.fcs_owner) + " has " +
                            geometry_field_to_string(m.field) + " = " +
                            twr_int_to_text(static_cast<int64_t>(m.expected)) +
                            ", the config claims " +
                            twr_int_to_text(static_cast<int64_t>(m.actual)) +
                            ". A claim does not decide the layout: fill the geometry from "
                            "frame_geometry_of_profile() so there is one source",
                        "REQ-PROTO-06");
            }
        }

        // STS: out of scope for phase 1, explicitly.
        if (f.sts_mode != StsMode::Off)
            unsup("frame.sts_mode",
                  std::string("STS mode ") + sts_mode_to_string(f.sts_mode) + ": " +
                      d_caps.unsupported_sts_reason,
                  ConfigReason::OutOfScope,
                  "REQ-SCOPE-04");
        if (f.sts_length_symbols != 0)
            cfg_rej("frame.sts_length_symbols",
                    ConfigReason::FieldConflict,
                    "sts_mode is off but an STS length is requested");
    }

    void check_tx(const TwrConfig& c) const
    {
        const TransmitConfig& t = c.tx;
        if (static_cast<uint32_t>(t.port) >= kTwrMaxPhysicalChannels)
            cfg_rej("tx.port", ConfigReason::IndexOutOfRange,
                "physical TX channel index too large");
        check_finite_opt("tx.gain_db", t.gain_db);
        check_finite_opt("tx.iq_amplitude", t.iq_amplitude);
        check_finite_opt("tx.calibrated_tx_power_dbm", t.calibrated_tx_power_dbm);

        if (t.gain_db.has() && (t.gain_db.value() < 0.0 || t.gain_db.value() > 120.0))
            cfg_rej("tx.gain_db", ConfigReason::OutOfRange,
                "TX gain must be within 0..120 dB");
        if (t.iq_amplitude.has() && (t.iq_amplitude.value() <= 0.0 || t.iq_amplitude.value() > 1.0))
            cfg_rej("tx.iq_amplitude",
                    ConfigReason::OutOfRange,
                    "digital IQ amplitude must be within (0, 1] for a full-scale SC16 stream");
        if (t.calibrated_tx_power_dbm.has() &&
            (t.calibrated_tx_power_dbm.value() < -30.0 || t.calibrated_tx_power_dbm.value() > 30.0))
            cfg_rej("tx.calibrated_tx_power_dbm",
                    ConfigReason::OutOfRange,
                    "calibrated antenna-plane power must be within -30..30 dBm");

        // Exactly one power authority, and the authority must be the field that
        // is actually present.  The three quantities are never conflated.
        int present = 0;
        if (t.gain_db.has())
            present++;
        if (t.iq_amplitude.has())
            present++;
        if (t.calibrated_tx_power_dbm.has())
            present++;
        const bool authority_present =
            (t.power_policy == TxPowerPolicy::LeaveUntouched) ||
            (t.power_policy == TxPowerPolicy::ManualGainDb && t.gain_db.has()) ||
            (t.power_policy == TxPowerPolicy::IqAmplitude && t.iq_amplitude.has()) ||
            (t.power_policy == TxPowerPolicy::CalibratedDbm && t.calibrated_tx_power_dbm.has());
        if (!authority_present)
            cfg_rej("tx.power_policy",
                    ConfigReason::FieldConflict,
                    std::string("power_policy=") + tx_power_policy_to_string(t.power_policy) +
                        " but the field it names is not set (gain dB, IQ amplitude and calibrated "
                        "dBm are three separate fields)");
        if (t.power_policy == TxPowerPolicy::LeaveUntouched && present != 0)
            cfg_rej("tx.power_policy",
                    ConfigReason::FieldConflict,
                    "power_policy is leave_untouched but " + twr_int_to_text(present) +
                        " power field(s) are set");
        if (t.power_policy == TxPowerPolicy::CalibratedDbm && t.calibrated_tx_power_dbm.has() &&
            !t.gain_db.has())
            cfg_rej("tx.gain_db",
                    ConfigReason::FieldConflict,
                    "a calibrated dBm is only meaningful together with the gain it was "
                    "measured at");

        // A vendor power word is a fourth, separately named field and may only
        // exist with the backend capability that defines its bits.
        if (t.vendor_power_word.has()) {
            if (t.vendor_power_word_backend.empty())
                cfg_rej("tx.vendor_power_word",
                        ConfigReason::CalibrationMismatch,
                        "a vendor power word is meaningless without the backend capability id that "
                        "defines its bits");
        } else if (!t.vendor_power_word_backend.empty()) {
            cfg_rej("tx.vendor_power_word_backend",
                    ConfigReason::FieldConflict,
                    "a backend capability id is set but no vendor power word is");
        }
    }

    void check_rx(const TwrConfig& c) const
    {
        const ReceiveConfig& r = c.rx;
        if (static_cast<uint32_t>(r.port) >= kTwrMaxPhysicalChannels)
            cfg_rej("rx.port", ConfigReason::IndexOutOfRange,
                "physical RX channel index too large");
        check_finite_opt("rx.gain_db", r.gain_db);
        check_finite_opt("rx.bandwidth_hz", r.bandwidth_hz);
        finite("rx.detection_threshold", r.detection_threshold);
        finite("rx.correlation_threshold", r.correlation_threshold);
        finite("rx.first_path_threshold", r.first_path_threshold);
        check_finite_opt("rx.vendor_pac_applied_step", r.vendor_pac_applied_step);

        if (r.gain_db.has() && (r.gain_db.value() < 0.0 || r.gain_db.value() > 120.0))
            cfg_rej("rx.gain_db", ConfigReason::OutOfRange,
                "RX gain must be within 0..120 dB");
        if (r.agc == AgcMode::Manual && !r.gain_db.has())
            cfg_rej("rx.gain_db",
                    ConfigReason::FieldConflict,
                    "agc=manual needs an explicit RX gain in dB");
        if (r.bandwidth_hz.has() && r.bandwidth_hz.value() <= 0.0)
            cfg_rej("rx.bandwidth_hz",
                    ConfigReason::OutOfRange,
                    "RF bandwidth must be > 0; it is a different quantity from the sample rate "
                    "(REQ-PHY-02)");
        if (r.bandwidth_hz.has() && r.bandwidth_hz.value() > 1.0e9)
            cfg_rej("rx.bandwidth_hz", ConfigReason::OutOfRange,
                "RF bandwidth is implausible");
        if (r.detection_threshold < 0.0 || r.detection_threshold > 1.0)
            cfg_rej("rx.detection_threshold", ConfigReason::OutOfRange,
                "must be within 0..1");
        if (r.correlation_threshold < 0.0 || r.correlation_threshold > 1.0)
            cfg_rej("rx.correlation_threshold", ConfigReason::OutOfRange,
                "must be within 0..1");
        if (r.first_path_threshold < 0.0 || r.first_path_threshold > 1.0)
            cfg_rej("rx.first_path_threshold", ConfigReason::OutOfRange,
                "must be within 0..1");
        if (r.correlation_threshold < r.detection_threshold)
            cfg_rej("rx.correlation_threshold",
                    ConfigReason::FieldConflict,
                    "the fine correlation gate must not be looser than the coarse detection gate");
        if (r.first_path_window == 0)
            cfg_rej("rx.first_path_window",
                    ConfigReason::ZeroValue,
                    "the first-path search window must be at least one tap");
        else if (r.first_path_index >= r.first_path_window)
            cfg_rej("rx.first_path_index",
                    ConfigReason::IndexOutOfRange,
                    "first usable CIR index lies outside the search window");

        // A vendor PAC value must come from a backend capability query AND
        // carry the software step that is actually applied.  Mapping a PAC
        // value onto an arbitrary software detection step is forbidden.
        if (r.vendor_pac_value.has()) {
            if (r.vendor_pac_backend.empty())
                cfg_rej("rx.vendor_pac_value",
                        ConfigReason::CalibrationMismatch,
                        "a vendor PAC value is meaningless without the backend capability id that "
                        "defines it");
            if (!r.vendor_pac_applied_step.has())
                cfg_rej("rx.vendor_pac_applied_step",
                        ConfigReason::CalibrationMismatch,
                        "a vendor PAC value may not be mapped onto an arbitrary software "
                        "detection step; the backend must state the step it applies");
            else if (r.vendor_pac_applied_step.value() <= 0.0)
                cfg_rej("rx.vendor_pac_applied_step",
                        ConfigReason::OutOfRange,
                        "the applied software step must be > 0");
        } else {
            if (!r.vendor_pac_backend.empty())
                cfg_rej("rx.vendor_pac_backend",
                        ConfigReason::FieldConflict,
                        "a backend capability id is set but no vendor PAC value is");
            if (r.vendor_pac_applied_step.has())
                cfg_rej("rx.vendor_pac_applied_step",
                        ConfigReason::FieldConflict,
                        "a software step is declared without a vendor PAC value");
        }
    }

    void check_radio(const TwrConfig& c) const
    {
        const RadioConfig& r = c.radio;
        if (static_cast<uint32_t>(r.tx_channel) >= kTwrMaxPhysicalChannels)
            cfg_rej("radio.tx_channel", ConfigReason::IndexOutOfRange,
                "TX channel index too large");
        if (static_cast<uint32_t>(r.rx_channel) >= kTwrMaxPhysicalChannels)
            cfg_rej("radio.rx_channel", ConfigReason::IndexOutOfRange,
                "RX channel index too large");
        finite("radio.native_sample_rate_hz", r.native_sample_rate_hz, "REQ-PHY-02");
        if (std::isfinite(r.native_sample_rate_hz) && r.native_sample_rate_hz > 0.0 &&
            !d_caps.native_rate_supported(r.native_sample_rate_hz))
            unsup("radio.native_sample_rate_hz",
                  "native sample rate " + twr_double_to_text(r.native_sample_rate_hz) +
                      " is not supported; this build accepts 737280000 or 491520000",
                  ConfigReason::Unsupported,
                  "REQ-PHY-02");

        if (r.peers.size() > kTwrMaxPeersPerConfig)
            cfg_rej("radio.peers",
                    ConfigReason::OverCapacity,
                    "at most " + twr_int_to_text(static_cast<int64_t>(kTwrMaxPeersPerConfig)) +
                        " peer endpoints per config");

        // Resource conflicts on the same physical channel.
        for (size_t i = 0; i < r.peers.size(); ++i) {
            const EndpointBinding& p = r.peers[i];
            const std::string pf = "radio.peers[" + twr_int_to_text(static_cast<int64_t>(i)) + "]";
            if (p.id.empty())
                cfg_rej(pf + ".id", ConfigReason::EmptyValue,
                    "endpoint id is required");
            if (static_cast<uint32_t>(p.tx_channel) >= kTwrMaxPhysicalChannels)
                cfg_rej(pf + ".tx_channel", ConfigReason::IndexOutOfRange,
                    "TX channel too large");
            if (static_cast<uint32_t>(p.rx_channel) >= kTwrMaxPhysicalChannels)
                cfg_rej(pf + ".rx_channel", ConfigReason::IndexOutOfRange,
                    "RX channel too large");
            finite(pf + ".native_sample_rate_hz", p.native_sample_rate_hz);
            if (!p.occupies_resources)
                continue;
            if (p.tx_channel == r.tx_channel)
                cfg_rej(pf + ".tx_channel",
                        ConfigReason::DuplicateResource,
                        "physical TX channel " + twr_int_to_text(p.tx_channel) +
                            " is already claimed by this endpoint; one device resource, one owner",
                        "REQ-BASE-01");
            if (p.rx_channel == r.rx_channel)
                cfg_rej(pf + ".rx_channel",
                        ConfigReason::DuplicateResource,
                        "physical RX channel " + twr_int_to_text(p.rx_channel) +
                            " is already claimed by this endpoint; one device resource, one owner",
                        "REQ-BASE-01");
            if (p.role == c.session.role)
                cfg_rej(pf + ".role",
                        ConfigReason::FieldConflict,
                        std::string("both endpoints claim role ") + role_to_string(p.role) +
                            "; a two-endpoint session needs one initiator and one responder",
                        "REQ-PROTO-01");
            if (p.native_sample_rate_hz > 0.0 && r.native_sample_rate_hz > 0.0 &&
                !twr_rate_matches_strict(p.native_sample_rate_hz,
                    r.native_sample_rate_hz, 1e-9))
                cfg_rej(pf + ".native_sample_rate_hz",
                        ConfigReason::FieldConflict,
                        "endpoints sharing one device must share the native sample rate (" +
                            twr_double_to_text(r.native_sample_rate_hz) + " vs " +
                            twr_double_to_text(p.native_sample_rate_hz) + ")",
                        "REQ-PHY-02");
            for (size_t j = 0; j < i; ++j) {
                const EndpointBinding& q = r.peers[j];
                if (!q.occupies_resources)
                    continue;
                if (q.id == p.id)
                    cfg_rej(pf + ".id", ConfigReason::DuplicateResource,
                        "duplicate endpoint id");
                if (q.tx_channel == p.tx_channel)
                    cfg_rej(pf + ".tx_channel",
                            ConfigReason::DuplicateResource,
                            "physical TX channel " + twr_int_to_text(p.tx_channel) +
                                " claimed by two endpoints",
                            "REQ-BASE-01");
                if (q.rx_channel == p.rx_channel)
                    cfg_rej(pf + ".rx_channel",
                            ConfigReason::DuplicateResource,
                            "physical RX channel " + twr_int_to_text(p.rx_channel) +
                                " claimed by two endpoints",
                            "REQ-BASE-01");
                if (q.role == p.role)
                    cfg_rej(pf + ".role",
                            ConfigReason::FieldConflict,
                            std::string("two peers claim the same role ") + role_to_string(p.role),
                            "REQ-PROTO-01");
            }
        }

        // Readback: a mismatch is a STARTUP FAILURE, never a soft warning.
        if (r.require_readback && !r.readback.present)
            rej("radio.readback.present",
                ConfigReason::CalibrationMissing,
                ExchangeStatus::CalibrationMissing,
                "require_readback is set but no hardware readback was recorded",
                "REQ-PHY-02");
        if (r.readback.present) {
            finite("radio.readback.sample_rate_hz", r.readback.sample_rate_hz,
                "REQ-PHY-02");
            finite("radio.readback.center_freq_hz", r.readback.center_freq_hz,
                "REQ-PHY-01");
            if (!d_caps.native_rate_supported(r.readback.sample_rate_hz))
                cfg_rej("radio.readback.sample_rate_hz",
                        ConfigReason::OutOfRange,
                        "readback native rate " + twr_double_to_text(r.readback.sample_rate_hz) +
                            " Hz is not a supported rate");
            else if (r.native_sample_rate_hz > 0.0 &&
                     !twr_rate_matches_strict(r.native_sample_rate_hz,
                                              r.readback.sample_rate_hz,
                                              1e-9))
                cfg_rej("radio.readback.sample_rate_hz",
                        ConfigReason::FieldConflict,
                        "readback native rate " + twr_double_to_text(r.readback.sample_rate_hz) +
                            " Hz differs from the requested " +
                            twr_double_to_text(r.native_sample_rate_hz) + " Hz",
                        "REQ-PHY-02");
            if (r.readback.tx_channel != r.tx_channel)
                cfg_rej("radio.readback.tx_channel",
                        ConfigReason::FieldConflict,
                        "readback TX channel differs from the requested one",
                        "REQ-PHY-02");
            if (r.readback.rx_channel != r.rx_channel)
                cfg_rej("radio.readback.rx_channel",
                        ConfigReason::FieldConflict,
                        "readback RX channel differs from the requested one",
                        "REQ-PHY-02");
            if (c.phy.center_frequency_hz > 0.0 && r.readback.center_freq_hz > 0.0 &&
                std::fabs(c.phy.center_frequency_hz - r.readback.center_freq_hz) >
                    kTwrChannelFrequencyToleranceHz)
                cfg_rej("radio.readback.center_freq_hz",
                        ConfigReason::ChannelFrequencyMismatch,
                        "readback centre frequency " +
                        twr_double_to_text(r.readback.center_freq_hz) +
                            " Hz differs from the requested " +
                            twr_double_to_text(c.phy.center_frequency_hz) + " Hz",
                        "REQ-PHY-01");
            if (!r.clock_source.empty() && !r.readback.clock_source.empty() &&
                r.clock_source != r.readback.clock_source)
                cfg_rej("radio.readback.clock_source",
                        ConfigReason::FieldConflict,
                        "readback clock source differs from the requested one",
                        "REQ-PHY-02");
            if (!r.time_source.empty() && !r.readback.time_source.empty() &&
                r.time_source != r.readback.time_source)
                cfg_rej("radio.readback.time_source",
                        ConfigReason::FieldConflict,
                        "readback time source differs from the requested one",
                        "REQ-PHY-02");
        }
    }

    // The generic timed-field contract.  Returns nothing; appends violations.
    void validate_timed_field(const std::string& f,
                              const TimedField& t,
                              bool required,
                              TimeReferenceEvent expect_ref,
                              bool require_marker) const
    {
        if (t.domain == TimeDomain::Unspecified) {
            cfg_rej(f + ".domain",
                    ConfigReason::EmptyValue,
                    "the clock domain must be stated (device_ticks or monotonic_host)",
                    "REQ-API-02");
            return;
        }
        if (t.value.negative())
            cfg_rej(f + ".ns", ConfigReason::NegativeValue, "may not be negative",
                "REQ-API-02");
        if (required && t.value.is_zero())
            cfg_rej(f + ".ns", ConfigReason::ZeroValue, "must be > 0", "REQ-API-02");
        if (expect_ref != TimeReferenceEvent::HostMonotonic && t.reference != expect_ref)
            cfg_rej(f + ".reference",
                    ConfigReason::FieldConflict,
                    std::string("must be ") + time_reference_event_to_string(expect_ref) +
                        std::string(", is ") + time_reference_event_to_string(t.reference),
                    "REQ-API-02");
        if (t.domain == TimeDomain::DeviceTicks) {
            if (require_marker && !t.marker.has())
                cfg_rej(f + ".marker",
                        ConfigReason::EmptyValue,
                        "a device-tick field must name the RF marker it is measured at "
                        "(REQ-TIME-02)",
                        "REQ-API-02");
            if (t.marker.has() && t.marker.value() == TimestampMarker::UhdRxFirstIqSample &&
                expect_ref == TimeReferenceEvent::FrameTail)
                cfg_rej(f + ".marker",
                        ConfigReason::FieldConflict,
                        "the first RX IQ sample is not the end of a frame",
                        "REQ-TIME-02");
        } else { // MonotonicHost
            if (t.marker.has())
                cfg_rej(f + ".marker",
                        ConfigReason::FieldConflict,
                        "a host monotonic field has no RF marker; a marker here would invite a "
                        "cross-domain subtraction",
                        "REQ-TIME-01");
            if (t.required_quantisation_hz != 1.0e9)
                cfg_rej(f + ".quantisation_hz",
                        ConfigReason::QuantisationUnsupported,
                        "a host monotonic field must be quantised at 1e9 (integer nanoseconds)",
                        "REQ-TIME-01");
        }
        if (!std::isfinite(t.required_quantisation_hz) || t.required_quantisation_hz <= 0.0) {
            cfg_rej(f + ".quantisation_hz",
                    ConfigReason::QuantisationUnsupported,
                    "the required quantisation rate must be finite and > 0",
                    "REQ-API-01");
            return;
        }
        if (!d_caps.quantisation_supported(t.required_quantisation_hz)) {
            cfg_rej(f + ".quantisation_hz",
                    ConfigReason::QuantisationUnsupported,
                    "quantisation " + twr_double_to_text(t.required_quantisation_hz) +
                        " Hz is not a rate this build can express (allowed: 1e9 ns, " +
                        twr_double_to_text(d_caps.work_sample_rate_hz) + " work rate, the native "
                        "device rates)",
                    "REQ-API-01");
            return;
        }
        if (t.max_quantisation_error_ns < 0)
            cfg_rej(f + ".max_quantisation_error_ns",
                    ConfigReason::OutOfRange,
                    "the tolerated quantisation error may not be negative");
        // The two distinct precision gates.
        if (!t.value.representable_at(t.required_quantisation_hz,
            t.max_quantisation_error_ns))
            cfg_rej(f + ".ns",
                    ConfigReason::TimingPrecisionInsufficient,
                    "a device tick at " + twr_double_to_text(t.required_quantisation_hz) +
                        " Hz cannot express this value with <= " +
                        twr_int_to_text(t.max_quantisation_error_ns) + " ns of error",
                    "REQ-API-01");
        bool ok = false;
        (void)quantise_duration(t.value, t.required_quantisation_hz, ok);
        if (!ok)
            cfg_rej(f + ".ns",
                    ConfigReason::TimingPrecisionInsufficient,
                    "the value does not fit the device tick range at " +
                        twr_double_to_text(t.required_quantisation_hz) + " Hz",
                    "REQ-API-01");
    }

    void check_timing(const TwrConfig& c) const
    {
        const PerMessageTiming& t = c.timing;
        const bool ss = c.session.protocol == Protocol::Ss;
        const bool initiator = c.session.role == Role::Initiator;

        check_finite_timed("timing.poll_start", t.poll_start);
        check_finite_timed("timing.poll_to_response", t.poll_to_response);
        check_finite_timed("timing.response_to_final", t.response_to_final);
        check_finite_timed("timing.final_to_report", t.final_to_report);
        check_finite_timed("timing.post_tx_rx_enable", t.post_tx_rx_enable);
        check_finite_timed("timing.min_tx_lead_time", t.min_tx_lead_time);

        // poll_start: absolute, on the device clock, at the Poll TX RMARKER.
        // Only the initiator transmits a Poll, so a responder's poll_start must
        // be zero rather than silently reusing the initiator's value.
        validate_timed_field("timing.poll_start",
                             t.poll_start,
                             initiator,
                             TimeReferenceEvent::PollTransmitRmarker,
                             true);
        if (!initiator && !t.poll_start.value.is_zero())
            cfg_rej("timing.poll_start.ns",
                    ConfigReason::FieldConflict,
                    "the responder transmits no Poll, so the Poll start time must be zero",
                    "REQ-API-02");

        // The three distinct quantities.
        if (initiator) {
            // The initiator does not send a Response, so it owns no reply delay.
            if (!t.poll_to_response.value.is_zero())
                cfg_rej("timing.poll_to_response",
                        ConfigReason::FieldConflict,
                        "the initiator never sends a Response, so this reply delay must be zero",
                        "REQ-API-02");
            if (t.poll_to_response.reference != TimeReferenceEvent::PollReceiveRmarker)
                cfg_rej("timing.poll_to_response.reference",
                        ConfigReason::FieldConflict,
                        "must be poll_rx_rmarker",
                        "REQ-API-02");
            if (ss) {
                if (!t.response_to_final.value.is_zero())
                    cfg_rej("timing.response_to_final",
                            ConfigReason::FieldConflict,
                            "SS-TWR has no Final message, so this delay must be zero",
                            "REQ-API-02");
            } else {
                validate_timed_field("timing.response_to_final",
                                     t.response_to_final,
                                     true,
                                     TimeReferenceEvent::ResponseReceiveRmarker,
                                     true);
            }
        } else {
            if (!t.response_to_final.value.is_zero())
                cfg_rej("timing.response_to_final",
                        ConfigReason::FieldConflict,
                        "the responder never sends a Final, so this delay must be zero",
                        "REQ-API-02");
            if (t.response_to_final.reference != TimeReferenceEvent::FinalTransmitRmarker)
                cfg_rej("timing.response_to_final.reference",
                        ConfigReason::FieldConflict,
                        "must be final_tx_rmarker",
                        "REQ-API-02");
            validate_timed_field("timing.poll_to_response",
                                 t.poll_to_response,
                                 true,
                                 TimeReferenceEvent::PollReceiveRmarker,
                                 true);
        }

        // post-TX RX enable: a device-tick delay from the end of OUR OWN frame.
        validate_timed_field("timing.post_tx_rx_enable",
                             t.post_tx_rx_enable,
                             true,
                             TimeReferenceEvent::FrameTail,
                             true);
        validate_timed_field("timing.min_tx_lead_time",
                             t.min_tx_lead_time,
                             true,
                             TimeReferenceEvent::ResponseTransmitRmarker,
                             true);

        // Report is reserved in phase 1.
        if (!t.final_to_report.value.is_zero())
            unsup("timing.final_to_report",
                  "FrameType::Report is reserved in phase 1; a Report message would invalidate the "
                  "three-message interop claim (REQ-PROTO-05)",
                  ConfigReason::OutOfScope,
                  "REQ-PROTO-05");

        // REQ-GR-04 feasibility: the reply delay must cover the device lead
        // time.  There is no default lead time -- the operator must supply the
        // measured value, and a delay shorter than it is rejected instead of
        // being sent late.
        const TimedField& reply = initiator ? t.response_to_final : t.poll_to_response;
        if (reply.value.negative() || t.min_tx_lead_time.value.negative() || reply.value.is_zero())
            return; // the structural violation was already reported above
        if (reply.value > t.min_tx_lead_time.value)
            return; // the budget is feasible
        rej(initiator ? "timing.response_to_final.ns" : "timing.poll_to_response.ns",
            ConfigReason::TimingBudgetInfeasible,
            ExchangeStatus::DeadlineMissed,
            "reply delay " + twr_duration_to_text(reply.value) +
                " does not cover the measured minimum TX lead time " +
                twr_duration_to_text(t.min_tx_lead_time.value) +
                " (a scheduled TX whose deadline cannot be met must fail, not be sent late)",
            "REQ-GR-04");
    }

    void check_timeouts(const TwrConfig& c) const
    {
        const TimeoutConfig& t = c.timeouts;
        const bool ss = c.session.protocol == Protocol::Ss;
        const bool initiator = c.session.role == Role::Initiator;

        check_finite_timed("timeouts.poll_rx_window", t.poll_rx_window);
        check_finite_timed("timeouts.response_rx_window", t.response_rx_window);
        check_finite_timed("timeouts.final_rx_window", t.final_rx_window);
        check_finite_timed("timeouts.report_rx_window", t.report_rx_window);
        check_finite_timed("timeouts.rx_timeout", t.rx_timeout);
        check_finite_timed("timeouts.exchange_timeout", t.exchange_timeout);
        check_finite_timed("timeouts.retry_interval", t.retry_interval);

        // The RX timeout is measured FROM RX ENABLE, on the device clock.
        validate_timed_field("timeouts.rx_timeout", t.rx_timeout, true,
                             TimeReferenceEvent::RxEnable, true);

        // The whole-exchange deadline and the retry gap live on the host
        // monotonic clock and must carry NO RF marker (REQ-TIME-01).
        validate_timed_field("timeouts.exchange_timeout", t.exchange_timeout, true,
                             TimeReferenceEvent::HostMonotonic, false);
        validate_timed_field("timeouts.retry_interval", t.retry_interval, false,
                             TimeReferenceEvent::HostMonotonic, false);

        // Per-frame RX windows, armed at RX enable.  Only the frames this role
        // actually waits for may be non-zero.
        validate_timed_field("timeouts.poll_rx_window", t.poll_rx_window, !initiator,
                             TimeReferenceEvent::RxEnable, true);
        validate_timed_field("timeouts.response_rx_window", t.response_rx_window,
            initiator,
                             TimeReferenceEvent::RxEnable, true);
        if (initiator && !t.poll_rx_window.value.is_zero())
            cfg_rej("timeouts.poll_rx_window.ns",
                    ConfigReason::FieldConflict,
                    "the initiator never receives a Poll, so its Poll RX window must be zero",
                    "REQ-API-02");
        if (!initiator && !t.response_rx_window.value.is_zero())
            cfg_rej("timeouts.response_rx_window.ns",
                    ConfigReason::FieldConflict,
                    "the responder never receives a Response, so its Response RX window must be "
                    "zero",
                    "REQ-API-02");
        if (ss) {
            if (!t.final_rx_window.value.is_zero())
                cfg_rej("timeouts.final_rx_window.ns",
                        ConfigReason::FieldConflict,
                        "SS-TWR has no Final message, so the Final RX window must be zero",
                        "REQ-API-02");
        } else {
            validate_timed_field("timeouts.final_rx_window", t.final_rx_window,
                !initiator,
                                 TimeReferenceEvent::RxEnable, true);
        }
        if (!t.report_rx_window.value.is_zero())
            unsup("timeouts.report_rx_window",
                  "FrameType::Report is reserved in phase 1 (REQ-PROTO-05)",
                  ConfigReason::OutOfScope,
                  "REQ-PROTO-05");

        // Every active window must fit inside the RX timeout measured from the
        // same RX enable event, otherwise the timeout can fire before the
        // window closes.
        const struct {
            const char* path;
            const TimedField& f;
        } windows[] = {
            { "timeouts.poll_rx_window", t.poll_rx_window },
            { "timeouts.response_rx_window", t.response_rx_window },
            { "timeouts.final_rx_window", t.final_rx_window },
        };
        for (const auto& w : windows) {
            if (w.f.value.is_zero())
                continue;
            if (w.f.value > t.rx_timeout.value)
                cfg_rej(std::string(w.path) + ".ns",
                        ConfigReason::TimingOrderViolation,
                        "the RX window " + twr_duration_to_text(w.f.value) +
                            " is longer than the RX timeout " +
                            twr_duration_to_text(t.rx_timeout.value) +
                            " measured from the same RX enable",
                        "REQ-API-02");
        }

        // The whole-exchange deadline must cover the chain this endpoint runs.
        int64_t chain_ns = t.rx_timeout.value.nanos();
        if (!initiator)
            chain_ns += c.timing.poll_to_response.value.nanos();
        if (!ss && initiator)
            chain_ns += c.timing.response_to_final.value.nanos();
        if (chain_ns > 0 && t.exchange_timeout.value.nanos() < chain_ns)
            cfg_rej("timeouts.exchange_timeout.ns",
                    ConfigReason::TimingOrderViolation,
                    "the whole-exchange deadline " +
                        twr_duration_to_text(t.exchange_timeout.value) +
                        " is shorter than the chain this endpoint must run (" +
                        twr_duration_to_text(Duration::from_nanos(chain_ns)) + ")",
                    "REQ-API-02");

        // Consecutive measurements may not overlap: at most one exchange is in
        // flight, so the interval must be at least one whole exchange.
        if (c.session.measurement_count > 1 &&
            c.session.measurement_interval.nanos() < t.exchange_timeout.value.nanos())
            cfg_rej("session.measurement_interval.ns",
                    ConfigReason::TimingOrderViolation,
                    "at most one exchange is in flight, so the measurement interval must be at "
                    "least one whole-exchange timeout",
                    "REQ-API-01");
    }

    void check_calibration(const TwrConfig& c) const
    {
        const TimestampCalibrationConfig& k = c.calibration;
        if (k.tx_link_delay.negative() || k.rx_link_delay.negative() ||
            k.antenna_delay.negative() ||
            k.cable_delay.negative())
            cfg_rej("calibration.tx_link_delay",
                    ConfigReason::NegativeValue,
                    "a link/antenna/cable delay may not be negative",
                    "REQ-CAL-01");
        finite("calibration.native_sample_rate_hz", k.native_sample_rate_hz,
            "REQ-PHY-02");
        check_finite_opt("calibration.record.gain_db", k.record.gain_db);
        if (k.native_sample_rate_hz > 0.0 && !is_allowed_native_rate(k.native_sample_rate_hz))
            cfg_rej("calibration.native_sample_rate_hz",
                    ConfigReason::OutOfRange,
                    "calibration is versioned by sample rate; this build only calibrates "
                    "at 737280000 or 491520000",
                    "REQ-CAL-01");
        if (k.tx_link_delay_native_ticks < 0 || k.rx_link_delay_native_ticks < 0)
            cfg_rej("calibration.tx_link_delay_native_ticks",
                    ConfigReason::NegativeValue,
                    "a native-tick delay may not be negative",
                    "REQ-CAL-01");
        if (k.link_delay_unit == TimeUnit::Seconds)
            cfg_rej("calibration.link_delay_unit",
                    ConfigReason::UnitMismatch,
                    "the delay fields are integer nanoseconds; use link_delay_unit=ns or "
                    "native_ticks and state the seconds value elsewhere",
                    "REQ-TIME-01");
        if (k.link_delay_unit == TimeUnit::NativeTicks && k.native_sample_rate_hz <= 0.0)
            cfg_rej("calibration.native_sample_rate_hz",
                    ConfigReason::UnitMismatch,
                    "native_ticks needs the native sample rate it refers to",
                    "REQ-TIME-01");
        if (k.link_delay_unit == TimeUnit::NativeTicks &&
            (k.tx_link_delay_native_ticks == 0 && k.rx_link_delay_native_ticks == 0))
            cfg_rej("calibration.link_delay_unit",
                    ConfigReason::FieldConflict,
                    "link_delay_unit=native_ticks but no native-tick delay is stated",
                    "REQ-TIME-01");

        if (k.applied_count != 0)
            rej("calibration.applied_count",
                ConfigReason::CalibrationAlreadyApplied,
                ExchangeStatus::CalibrationMissing,
                "a calibration is applicable exactly once; this one has already been applied " +
                    twr_int_to_text(k.applied_count) + " time(s)",
                "REQ-CAL-01");

        if (!k.calibration_required)
            return;
        if (k.calibration_id.empty()) {
            rej("calibration.calibration_id",
                ConfigReason::CalibrationMissing,
                ExchangeStatus::CalibrationMissing,
                "a calibration id is required; an uncalibrated absolute range may not be claimed",
                "REQ-CAL-01");
            return;
        }
        if (k.record.calibration_id != k.calibration_id)
            cfg_rej("calibration.record.calibration_id",
                    ConfigReason::FieldConflict,
                    "the applicability record names a different calibration id",
                    "REQ-CAL-01");
        if (k.record.channel != c.phy.channel)
            cfg_rej("calibration.record.channel",
                    ConfigReason::CalibrationMismatch,
                    "calibration channel " + twr_int_to_text(k.record.channel) +
                        " does not match phy.channel " + twr_int_to_text(c.phy.channel),
                    "REQ-CAL-01");
        // M0.1: the applicability rule below is guarded by `> 0.0`, and a NaN
        // fails that comparison, so a NaN here used to sail through the whole
        // validator.  The record is compared against a rate, so it must be
        // finite whether or not the match ends up being attempted.
        // (record.gain_db is already covered by check_finite_opt above.)
        finite("calibration.record.native_sample_rate_hz",
               k.record.native_sample_rate_hz, "REQ-CAL-01");
        if (k.record.native_sample_rate_hz > 0.0 && c.radio.native_sample_rate_hz > 0.0 &&
            !twr_rate_matches_strict(k.record.native_sample_rate_hz,
                                     c.radio.native_sample_rate_hz,
                                     1e-9))
            cfg_rej("calibration.record.native_sample_rate_hz",
                    ConfigReason::CalibrationMismatch,
                    "calibration sample rate does not match the radio native rate",
                    "REQ-CAL-01");
        if (!k.record.profile_version.empty() && !c.meta.profile_version.empty() &&
            k.record.profile_version != c.meta.profile_version)
            cfg_rej("calibration.record.profile_version",
                    ConfigReason::CalibrationMismatch,
                    "calibration profile_version '" + k.record.profile_version +
                        "' does not match the session profile_version '" + c.meta.profile_version +
                        "'",
                    "REQ-CAL-01");
        if (!c.meta.calibration_version.empty() && k.calibration_id != c.meta.calibration_version)
            cfg_rej("meta.calibration_version",
                    ConfigReason::CalibrationMismatch,
                    "meta.calibration_version does not name the calibration that is loaded",
                    "REQ-OUT-01");
        if (c.tx.gain_db.has() && k.record.gain_db.has() &&
            k.record.gain_db.value() != c.tx.gain_db.value())
            cfg_rej("calibration.record.gain_db",
                    ConfigReason::CalibrationMismatch,
                    "the calibration was taken at a different TX gain",
                    "REQ-CAL-01");
    }

    void check_diagnostics(const TwrConfig& c) const
    {
        const DiagnosticsConfig& d = c.diagnostics;
        const struct {
            const char* on;
            const char* bytes;
            const char* stride;
            bool enabled;
            uint64_t bytes_v;
            uint32_t stride_v;
        } buffers[] = {
            { "diagnostics.cir_capture_enabled", "diagnostics.cir_capture_max_bytes",
              "diagnostics.cir_capture_stride", d.cir_capture_enabled, d.cir_capture_max_bytes,
              d.cir_capture_stride },
            { "diagnostics.short_iq_enabled", "diagnostics.short_iq_max_bytes",
              "diagnostics.short_iq_stride", d.short_iq_enabled, d.short_iq_max_bytes,
              d.short_iq_stride },
            { "diagnostics.raw_frame_dump", "diagnostics.raw_frame_max_bytes", nullptr,
              d.raw_frame_dump, d.raw_frame_max_bytes, 0u },
        };
        for (const auto& b : buffers) {
            if (b.bytes_v > kTwrMaxDiagnosticBytes)
                cfg_rej(b.bytes, ConfigReason::OverCapacity,
                        "diagnostic buffer above the " +
                            twr_int_to_text(static_cast<int64_t>(kTwrMaxDiagnosticBytes)) +
                            " byte bound (bounded memory, REQ-GR-03)");
            if (b.enabled && b.bytes_v == 0)
                cfg_rej(b.bytes, ConfigReason::ZeroValue,
                        "the diagnostic is enabled but no byte budget was stated");
            if (!b.enabled && b.bytes_v != 0)
                cfg_rej(b.bytes, ConfigReason::FieldConflict,
                        "a byte budget is stated but the diagnostic is disabled");
            if (b.stride != nullptr && b.enabled && b.stride_v == 0)
                cfg_rej(b.stride, ConfigReason::ZeroValue,
                        "a diagnostic stride of 0 would capture nothing");
            if (b.stride != nullptr && !b.enabled && b.stride_v != 0)
                cfg_rej(b.stride, ConfigReason::FieldConflict,
                        "a diagnostic stride is stated but the diagnostic is disabled");
        }
        if (d.result_queue_capacity == 0)
            cfg_rej("diagnostics.result_queue_capacity", ConfigReason::ZeroValue,
                    "the result queue must hold at least one entry");
        else if (d.result_queue_capacity > kTwrMaxQueueEntries)
            cfg_rej("diagnostics.result_queue_capacity", ConfigReason::OverCapacity,
                    "above the bounded queue limit");
        if (d.event_queue_capacity == 0)
            cfg_rej("diagnostics.event_queue_capacity", ConfigReason::ZeroValue,
                    "the event queue must hold at least one entry");
        else if (d.event_queue_capacity > kTwrMaxQueueEntries)
            cfg_rej("diagnostics.event_queue_capacity", ConfigReason::OverCapacity,
                    "above the bounded queue limit");
        if (!d.result_output_path.empty() && d.result_output_path.size() > 4096)
            cfg_rej("diagnostics.result_output_path", ConfigReason::OverCapacity,
                    "absurdly long output path");
        if (d.io_on_realtime_thread)
            cfg_rej("diagnostics.io_on_realtime_thread",
                    ConfigReason::FieldConflict,
                    "diagnostic I/O must not run on the realtime processing thread",
                    "REQ-GR-02");
        check_finite_timed("diagnostics.stats_cadence", d.stats_cadence);
        validate_timed_field("diagnostics.stats_cadence", d.stats_cadence, false,
                             TimeReferenceEvent::HostMonotonic, false);
    }

    // Frame lengths: Poll / Response / Final must each fit the negotiated PSDU
    // (REQ-API-01 帧格式; REQ-PHY-01 standard-length PSDU).
    void check_frame_lengths(const TwrConfig& c) const
    {
        const FrameFormatConfig& f = c.frame;
        if (f.mac_psdu_bytes == 0)
            return; // already reported as a missing value elsewhere
        const uint16_t cap = d_caps.max_psdu_bytes();
        if (cap != 0 && f.mac_psdu_bytes > cap)
            rej("frame.mac_psdu_bytes",
                ConfigReason::FrameLengthOverflow,
                ExchangeStatus::Unsupported,
                "negotiated PSDU " + twr_int_to_text(f.mac_psdu_bytes) +
                    " bytes exceeds the capability maximum " + twr_int_to_text(cap) + " bytes",
                "REQ-PHY-01");

        // Every length below comes from the codec, through
        // frame_psdu_bytes() -> FrameProfileGeometry::mac_payload_bytes() ->
        // frame_length_for().  The config's geometry is a claim, already
        // checked field by field in check_frame(), and is used here ONLY to
        // describe the arithmetic back to the operator.
        FrameProfileGeometry authority;
        FrameProfile profile;
        const bool have_profile =
            frame_profile_id_is_supported(f.frame_profile) &&
            frame_geometry_for(f.frame_profile, authority) &&
            frame_profile_for(f.frame_profile, profile);

        const FrameType types[] = { FrameType::Poll, FrameType::Response,
            FrameType::Final };
        for (FrameType t : types) {
            // `mac_psdu_bytes` is the negotiated MAC PSDU and, when the PHY
            // appends the FCS, EXCLUDES it (see the fcs_append /
            // mac_psdu_includes_fcs cross-check above).  So the comparison
            // below must be against the MAC bytes only.  The 127-byte IEEE
            // limit is separately checked against the on-air length.
            const uint32_t need = frame_psdu_bytes(t, f.fcs_append);
            const uint32_t on_air = have_profile
                                        ? static_cast<uint32_t>(
                                              authority.on_air_bytes(t, profile))
                                        : need + ((f.fcs_append == FcsAppender::PhyLayer)
                                                      ? f.fcs_bytes
                                                      : 0u);
            const std::string path =
                std::string("frame.mac_psdu_bytes[") + frame_type_to_string(t) + "]";
            if (need == 0)
                continue; // the profile itself was already reported as unknown
            if (need > static_cast<uint32_t>(f.mac_psdu_bytes))
                rej(path,
                    ConfigReason::FrameLengthOverflow,
                    ExchangeStatus::Unsupported,
                    std::string("the ") + frame_type_to_string(t) + " frame needs " +
                        twr_int_to_text(static_cast<int64_t>(need)) +
                        " MAC PSDU bytes (as frame profile " +
                        frame_profile_id_to_string(f.frame_profile) + " builds it: header " +
                        twr_int_to_text(authority.mac_header_bytes()) + " + " +
                        twr_int_to_text(frame_type_timestamp_count(t)) + "x" +
                        twr_int_to_text(authority.timestamp_bytes(profile)) +
                        " timestamps + footer " +
                        twr_int_to_text(authority.mac_footer_bytes()) +
                        (f.fcs_append == FcsAppender::MacLayer
                             ? std::string(" + FCS ") + twr_int_to_text(f.fcs_bytes)
                             : std::string("; the PHY appends the FCS separately")) +
                        ") but only " +
                        twr_int_to_text(f.mac_psdu_bytes) + " are negotiated",
                    "REQ-PHY-01");
            // The IEEE 127-byte limit applies to the ON-AIR PSDU, i.e. with
            // the FCS the PHY appends (UwbHrpPacketSource appends and then
            // checks 127), so this is the check that uses `on_air`.
            if (cap != 0 && on_air > static_cast<uint32_t>(cap))
                rej(path,
                    ConfigReason::FrameLengthOverflow,
                    ExchangeStatus::Unsupported,
                    std::string("the ") + frame_type_to_string(t) +
                        " frame needs " + twr_int_to_text(static_cast<int64_t>(on_air)) +
                        " on-air PSDU bytes (MAC " + twr_int_to_text(static_cast<int64_t>(need)) +
                        " + FCS " + twr_int_to_text(authority.fcs_bytes_on_air()) +
                        ") and does not fit the " + twr_int_to_text(cap) +
                        "-byte PSDU maximum",
                    "REQ-PHY-01");
        }
        // The application payload is a separate concept and must fit inside
        // the MAC PSDU after the header/timestamps/FCS are removed.
        if (f.geometry.any_set() && f.application_payload_bytes > f.mac_psdu_bytes)
            rej("frame.application_payload_bytes",
                ConfigReason::FrameLengthOverflow,
                ExchangeStatus::Unsupported,
                "the application payload (" + twr_int_to_text(f.application_payload_bytes) +
                    " bytes) does not fit the MAC PSDU (" +
                    twr_int_to_text(f.mac_psdu_bytes) + " bytes)",
                "REQ-API-01");

        // The joint PHY capability lookup, now that every axis is known.
        const CapabilityLookup look = d_caps.lookup_phy(c.radio.native_sample_rate_hz,
                                                       c.phy.tx_preamble_code,
                                                       c.phy.preamble_symbols,
                                                       f.sfd_mode,
                                                       f.mac_psdu_bytes,
                                                       f.ranging_bit);
        if (!look.allowed) {
            rej("phy.preamble_symbols",
                ConfigReason::Unsupported,
                ExchangeStatus::Unsupported,
                std::string("the (native_rate, code, sync, sfd, psdu, ranging) combination is not "
                            "in the capability whitelist [") +
                    capability_status_to_string(look.status) + "]: " + look.reason,
                "REQ-PHY-01");
        }
    }

    const Capabilities& d_caps;
    mutable ValidationReport d_report;
};

inline ValidationReport validate(const TwrConfig& cfg, const Capabilities& caps)
{
    ConfigValidator v(caps);
    return v.run(cfg);
}

inline ValidationReport validate(const TwrConfig& cfg) { return validate(cfg,
    capabilities()); }

// ===========================================================================
// 7. The frozen effective config
// ===========================================================================

// A frozen, self-describing snapshot (REQ-API-01: 完整保存 requested、effective、
// 硬件 readback).  It is a VALUE: an exchange in flight holds a copy, so a
// later change to the requested config cannot reach it (REQ-API-03).
struct EffectiveConfig {
    bool ok = false;
    ValidationReport validation;

    std::string schema_version;
    std::string profile_version;
    std::string calibration_version;
    std::string config_hash;

    TwrConfig requested;
    TwrConfig effective;
    std::vector<FieldChange> changes; // requested vs effective, per field

    // The hardware readback exactly as recorded at startup.
    RadioReadback readback;

    // Quantised timing actually handed to the device.  The REQUESTED delay and
    // the EFFECTIVE (tick-quantised) delay are both available so REQ-GR-04 can
    // report the margin.
    int64_t poll_to_response_ticks = 0;
    int64_t response_to_final_ticks = 0;
    int64_t post_tx_rx_enable_ticks = 0;
    int64_t poll_start_ticks = 0;
    Duration poll_to_response_effective{0};
    Duration response_to_final_effective{0};
    Duration post_tx_rx_enable_effective{0};
    Duration poll_start_effective{0};
    double tick_rate_hz = 0.0;

    // Frame budget, per frame type, as transmitted (including the FCS exactly
    // once).  These are the CODEC's numbers, not a re-summation of the
    // config's geometry claim: see effective_config().
    uint16_t poll_bytes = 0;
    uint16_t response_bytes = 0;
    uint16_t final_bytes = 0;
    uint16_t max_psdu_bytes = 0;
    uint32_t max_timestamp_count = 0;
    // The PHR of this profile, recorded so no consumer recomputes it: the
    // number of INFORMATION bytes and the number of transmitted coded bits.
    uint16_t phr_bytes = 0;
    uint16_t phr_coded_bits = 0;
};

// Build the frozen snapshot.  On failure `ok == false`, `validation` holds
// every reason, and `effective` is a value-initialised (NOT a default
// substituted) config: there is no way to accidentally use it.
inline EffectiveConfig
effective_config(const TwrConfig& cfg, const Capabilities& caps)
{
    EffectiveConfig out;
    out.requested = cfg;
    out.validation = validate(cfg, caps);
    out.ok = out.validation.ok();
    out.schema_version = caps.schema_version;
    out.profile_version = cfg.meta.profile_version;
    out.calibration_version = cfg.meta.calibration_version;
    if (!out.ok) {
        // A rejected config produces no effective values at all.  The hash of
        // the REQUESTED config is still recorded so a failure can be tied to
        // the exact input that caused it (REQ-OUT-01).
        out.effective = TwrConfig{};
        out.config_hash = config_hash(cfg);
        out.changes.clear();
        return out;
    }

    // Effective == requested plus the materialisations the validator proved
    // legal: nothing is invented, everything is recorded.
    out.effective = cfg;
    out.config_hash = config_hash(out.effective);
    out.readback = cfg.radio.readback;
    out.changes = diff_fields(out.requested, out.effective);
    out.max_psdu_bytes = caps.max_psdu_bytes();
    out.max_timestamp_count = frame_type_timestamp_count(FrameType::Final);

    const double rate = caps.native_rate_supported(cfg.radio.native_sample_rate_hz)
                            ? cfg.radio.native_sample_rate_hz
                            : caps.native_rates_hz.front();
    out.tick_rate_hz = rate;

    struct Q {
        const TimedField* in;
        int64_t* ticks;
        Duration* eff;
    } q[] = {
        { &cfg.timing.poll_start, &out.poll_start_ticks, &out.poll_start_effective },
        { &cfg.timing.poll_to_response, &out.poll_to_response_ticks,
          &out.poll_to_response_effective },
        { &cfg.timing.response_to_final, &out.response_to_final_ticks,
          &out.response_to_final_effective },
        { &cfg.timing.post_tx_rx_enable, &out.post_tx_rx_enable_ticks,
          &out.post_tx_rx_enable_effective },
    };
    for (const auto& item : q) {
        if (item.in->domain != TimeDomain::DeviceTicks)
            continue;
        bool ok = false;
        const int64_t ticks =
            quantise_duration(item.in->value, item.in->required_quantisation_hz, ok);
        *item.ticks = ok ? ticks : 0;
        Duration back;
        *item.eff = (ok && Duration::from_ticks(ticks, item.in->required_quantisation_hz, back))
                        ? back
                        : Duration{0};
        if (back != item.in->value) {
            // Record the quantisation as an explicit requested/effective
            // difference; the device gets the tick value, the operator sees both.
            FieldChange ch;
            ch.path = item.in->value.nanos() != 0 ? "timing.quantised_delay" : "timing.delay";
            ch.requested = twr_duration_to_text(item.in->value);
            ch.effective = twr_duration_to_text(back);
            ch.note = "quantised at " + twr_double_to_text(item.in->required_quantisation_hz) +
                      " Hz (ticks=" + twr_int_to_text(ticks) + ")";
            out.changes.push_back(ch);
        }
    }

    // The frame budget is the CODEC's, computed once through the authority.
    // M0 re-summed the config's own geometry claim here, which is how a 7-byte
    // header produced a 9-byte Poll and understated the on-air length; and it
    // added the FCS from a raw config field, so a wrong fcs_bytes silently
    // changed the answer.  `on_air_bytes()` is invariant under which layer
    // appends the FCS, which is exactly the property the budget should have.
    FrameProfileGeometry authority;
    FrameProfile profile;
    if (frame_geometry_for(cfg.frame.frame_profile, authority) &&
        frame_profile_for(cfg.frame.frame_profile, profile)) {
        out.poll_bytes = static_cast<uint16_t>(authority.on_air_bytes(FrameType::Poll, profile));
        out.response_bytes =
            static_cast<uint16_t>(authority.on_air_bytes(FrameType::Response, profile));
        out.final_bytes =
            static_cast<uint16_t>(authority.on_air_bytes(FrameType::Final, profile));
        // The PHR is present in every frame of this profile; recording its
        // information size here means a consumer never has to recompute it.
        out.phr_bytes = static_cast<uint16_t>(authority.phr_bytes());
        out.phr_coded_bits = authority.phr_coded_bits();
    }
    return out;
}

inline EffectiveConfig effective_config(const TwrConfig& cfg)
{
    return effective_config(cfg, capabilities());
}

// ---------------------------------------------------------------------------
// In-flight gate (phase 1: at most one exchange per endpoint)
// ---------------------------------------------------------------------------

// Enforces "同端初版一次仅一个在途 exchange" at RUN time, next to the config
// that promised it.  A second concurrent request is QueueFull, never a
// silently dropped or queued-forever exchange.
class ExchangeGate
{
public:
    explicit ExchangeGate(uint32_t max_in_flight = kTwrMaxInFlightExchanges)
        : d_max(max_in_flight == 0 ? 1u : max_in_flight)
    {
    }

    ValidationReport try_begin()
    {
        ValidationReport r;
        if (d_in_flight >= d_max) {
            ConfigViolation v;
            v.field = "session.max_in_flight_exchanges";
            v.reason = ConfigReason::ExchangeAlreadyInFlight;
            v.status = ExchangeStatus::QueueFull;
            v.message = "an exchange is already in flight; at most " + twr_int_to_text(d_max) +
                        " may be outstanding";
            v.requirement = "REQ-API-01";
            r.add(v);
            // A REFUSED request must not consume a slot, or a run of refusals
            // would deadlock the endpoint.
            return r;
        }
        ++d_in_flight;
        return r;
    }
    void end()
    {
        if (d_in_flight > 0)
            --d_in_flight;
    }
    uint32_t in_flight() const { return d_in_flight; }

private:
    uint32_t d_max;
    uint32_t d_in_flight = 0;
};

// ---------------------------------------------------------------------------
// REQ-API-03: per-message / per-exchange overrides on an immutable snapshot
// ---------------------------------------------------------------------------

// Only the values a single exchange may re-negotiate.  Anything that affects
// synchronisation (PHY, frequency, gain, addresses, calibration) is
// deliberately absent: it may only change at a session boundary, and the
// snapshot an exchange already holds is immutable (REQ-API-03).
struct PerMessageOverrides {
    // 0 = "not overridden"; the validator then requires the base config value.
    int64_t measurement_count = 0;
    int64_t measurement_interval_ns = 0;
    int64_t poll_to_response_ns = 0;
    int64_t response_to_final_ns = 0;
    int64_t post_tx_rx_enable_ns = 0;
    int64_t rx_timeout_ns = 0;
    int64_t exchange_timeout_ns = 0;

    bool any() const
    {
        return measurement_count != 0 || measurement_interval_ns != 0 ||
               poll_to_response_ns != 0 || response_to_final_ns != 0 ||
               post_tx_rx_enable_ns != 0 || rx_timeout_ns != 0 ||
                   exchange_timeout_ns != 0;
    }
};

// The frozen snapshot plus the exchange gate.  Copying it is the only way to
// hand an immutable config to a worker; there is no mutable accessor.
class TwrConfigSnapshot
{
public:
    TwrConfigSnapshot() : d_effective(), d_caps(&capabilities()) {}
    TwrConfigSnapshot(const EffectiveConfig& e, const Capabilities& caps)
        : d_effective(e), d_caps(&caps)
    {
    }

    const EffectiveConfig& get() const { return d_effective; }
    bool ok() const { return d_effective.ok; }

    uint32_t in_flight() const { return d_gate.in_flight(); }

    // Begin an exchange.  Returns Ok when the gate admitted it.
    ValidationReport begin_exchange() { return d_gate.try_begin(); }
    void end_exchange() { d_gate.end(); }

    // REQ-API-03: overrides take effect at an exchange BOUNDARY.  A call
    // while an exchange is in flight is rejected -- the running exchange keeps
    // the snapshot it started with.
    ValidationReport
    set_overrides(const PerMessageOverrides& o, TwrConfigSnapshot& out) const
    {
        ValidationReport r;
        if (d_gate.in_flight() > 0) {
            ConfigViolation v;
            v.field = "session";
            v.reason = ConfigReason::MidExchangeMutation;
            v.status = ExchangeStatus::ConfigRejected;
            v.message = "an exchange is in flight; overrides may only be applied at an exchange "
                        "boundary and the running exchange keeps its immutable snapshot";
            v.requirement = "REQ-API-03";
            r.add(v);
            return r;
        }
        if (!d_effective.ok) {
            ConfigViolation v;
            v.field = "";
            v.reason = ConfigReason::OutOfScope;
            v.status = ExchangeStatus::ConfigRejected;
            v.message = "the base config is not valid; overrides cannot be applied to it";
            v.requirement = "REQ-API-03";
            r.add(v);
            return r;
        }
        TwrConfig next = d_effective.effective;
        if (o.measurement_count != 0) {
            if (o.measurement_count < 0)
                next.session.measurement_count = 0; // let the validator reject
            else
                next.session.measurement_count = static_cast<uint32_t>(o.measurement_count);
        }
        if (o.measurement_interval_ns != 0)
            next.session.measurement_interval = Duration::from_nanos(o.measurement_interval_ns);
        if (o.poll_to_response_ns != 0)
            next.timing.poll_to_response.value = Duration::from_nanos(o.poll_to_response_ns);
        if (o.response_to_final_ns != 0)
            next.timing.response_to_final.value = Duration::from_nanos(o.response_to_final_ns);
        if (o.post_tx_rx_enable_ns != 0)
            next.timing.post_tx_rx_enable.value = Duration::from_nanos(o.post_tx_rx_enable_ns);
        if (o.rx_timeout_ns != 0)
            next.timeouts.rx_timeout.value = Duration::from_nanos(o.rx_timeout_ns);
        if (o.exchange_timeout_ns != 0)
            next.timeouts.exchange_timeout.value = Duration::from_nanos(o.exchange_timeout_ns);

        out = TwrConfigSnapshot(effective_config(next, *d_caps), *d_caps);
        if (!out.d_effective.ok) {
            r = out.d_effective.validation;
            return r;
        }
        out.d_effective.requested = d_effective.requested; // keep the original
        out.d_effective.changes = diff_fields(d_effective.requested,
            out.d_effective.effective);
        return r;
    }

private:
    EffectiveConfig d_effective;
    const Capabilities* d_caps;
    ExchangeGate d_gate;
};

// ---------------------------------------------------------------------------
// REQ-CAL-01: a calibration is applicable exactly once
// ---------------------------------------------------------------------------

// Apply the configured calibration to a measurement.  Idempotence is
// enforced, not assumed: a second application is rejected so a range can never
// be corrected twice.  `now_monotonic_ns` comes from the host monotonic clock
// and is only used for the expiry check.
inline ExchangeStatus
apply_calibration_once(TimestampCalibrationConfig& cal, int64_t now_monotonic_ns,
    std::string* why)
{
    if (cal.calibration_id.empty()) {
        if (why)
            *why = "no calibration id: an uncalibrated absolute range may not be claimed";
        return ExchangeStatus::CalibrationMissing;
    }
    if (cal.applied_count != 0) {
        if (why)
            *why = "calibration already applied " + twr_int_to_text(cal.applied_count) +
                   " time(s); a calibration is applicable exactly once";
        return ExchangeStatus::CalibrationMissing;
    }
    if (cal.record.valid_until_monotonic_ns != 0 && now_monotonic_ns != 0 &&
        now_monotonic_ns > cal.record.valid_until_monotonic_ns) {
        if (why)
            *why = "calibration " + cal.calibration_id + " expired at " +
                   twr_int_to_text(cal.record.valid_until_monotonic_ns);
        return ExchangeStatus::CalibrationExpired;
    }
    cal.applied_count = 1;
    if (why)
        *why = "calibration " + cal.calibration_id + " applied once";
    return ExchangeStatus::Ok;
}

// ===========================================================================
// 8. JSON import / export (dependency-free)
// ===========================================================================
//
// The repository has NO JSON library dependency (verified: nothing under
// gr-uwb/include or gr-uwb/lib provides one) and this milestone must not add
// one, so a minimal reader/writer for exactly this schema lives here.
//
// SCHEMA NOTES (REQ-OUT-01)
//   * Every value is emitted, including `null` for an absent optional field,
//     so `from_json(to_json(c)) == c` is exact.
//   * 64-bit integers are emitted UNQUOTED when |v| <= 2^53-1 and as a
//     QUOTED DECIMAL STRING otherwise.  The reader accepts both forms for any
//     integer field, so a timestamp beyond the JSON safe range survives a
//     round trip.  A decimal fraction or exponent on an integer field is
//     REJECTED (`type_mismatch`) rather than silently truncated.
//   * Doubles are emitted with %.17g, which round-trips every finite IEEE 754
//     double.  NaN / +Inf / -Inf have no JSON representation and are
//     REJECTED on export, so a corrupt config cannot be serialised into a
//     file that a later run would parse as a valid one.
//   * Durations are emitted as a plain integer number of NANOSECONDS under a
//     `_ns` key.  Native-tick fields carry a `_native_ticks` key and the
//     calibration additionally carries an explicit `link_delay_unit`, so ns,
//     seconds and native ticks can never be confused.
//   * Number parsing uses strtod and therefore expects the C numeric locale,
//     which is what GNU Radio and the Python host run with.
namespace json {

enum class Type { Null, Bool, Int, Double, String, Array, Object };

class Value
{
public:
    Type type = Type::Null;
    bool boolean = false;
    int64_t integer = 0;
    double number = 0.0;
    std::string text;
    std::vector<Value> items;                            // Array
    std::vector<std::pair<std::string, Value>> members;  // Object, ordered

    static Value make_null() { return Value(); }
    static Value make_bool(bool v)
    {
        Value x;
        x.type = Type::Bool;
        x.boolean = v;
        return x;
    }
    static Value make_int(int64_t v)
    {
        Value x;
        x.type = Type::Int;
        x.integer = v;
        return x;
    }
    static Value make_double(double v)
    {
        Value x;
        x.type = Type::Double;
        x.number = v;
        return x;
    }
    static Value make_string(std::string v)
    {
        Value x;
        x.type = Type::String;
        x.text = std::move(v);
        return x;
    }
    static Value make_array()
    {
        Value x;
        x.type = Type::Array;
        return x;
    }
    static Value make_object()
    {
        Value x;
        x.type = Type::Object;
        return x;
    }

    const Value* find(const std::string& key) const
    {
        for (const auto& m : members)
            if (m.first == key)
                return &m.second;
        return nullptr;
    }
    Value* find(const std::string& key)
    {
        for (auto& m : members)
            if (m.first == key)
                return &m.second;
        return nullptr;
    }
    void set(const std::string& key, Value v)
    {
        if (type == Type::Null)
            type = Type::Object;
        for (auto& m : members) {
            if (m.first == key) {
                m.second = std::move(v);
                return;
            }
        }
        members.emplace_back(key, std::move(v));
    }
    void push(Value v)
    {
        if (type == Type::Null)
            type = Type::Array;
        items.push_back(std::move(v));
    }

    bool is_number() const { return type == Type::Int || type == Type::Double; }
    double as_double() const
    {
        return type == Type::Int ? static_cast<double>(integer) : number;
    }
    const char* type_name() const
    {
        switch (type) {
        case Type::Null:
            return "null";
        case Type::Bool:
            return "bool";
        case Type::Int:
            return "integer";
        case Type::Double:
            return "number";
        case Type::String:
            return "string";
        case Type::Array:
            return "array";
        case Type::Object:
            return "object";
        }
        return "invalid";
    }
};

inline constexpr size_t kJsonMaxDepth = 32;
inline constexpr size_t kJsonMaxInputBytes = 4u * 1024u * 1024u;

class Parser
{
public:
    explicit Parser(const std::string& text) : s(text) {}

    bool parse(Value& out, std::string& err)
    {
        if (s.size() > kJsonMaxInputBytes) {
            err = "json input above " + twr_int_to_text(static_cast<int64_t>(kJsonMaxInputBytes)) +
                  " bytes";
            return false;
        }
        skip_ws();
        if (!parse_value(out, err))
            return false;
        skip_ws();
        if (p != s.size()) {
            err = "trailing characters at offset " + twr_int_to_text(static_cast<int64_t>(p));
            return false;
        }
        return true;
    }

private:
    bool fail(const std::string& m, std::string& err)
    {
        err = m + " at offset " + twr_int_to_text(static_cast<int64_t>(p));
        return false;
    }
    void skip_ws()
    {
        while (p < s.size() &&
               (s[p] == ' ' || s[p] == '\t' || s[p] == '\n' || s[p] == '\r'))
            ++p;
    }
    bool parse_value(Value& out, std::string& err)
    {
        if (d_depth >= kJsonMaxDepth)
            return fail("json nesting too deep", err);
        if (p >= s.size())
            return fail("unexpected end of input", err);
        switch (s[p]) {
        case '{':
            return parse_object(out, err);
        case '[':
            return parse_array(out, err);
        case '"': {
            std::string t;
            if (!parse_string(t, err))
                return false;
            out = Value::make_string(std::move(t));
            return true;
        }
        case 't':
            if (s.compare(p, 4, "true") != 0)
                return fail("bad literal", err);
            p += 4;
            out = Value::make_bool(true);
            return true;
        case 'f':
            if (s.compare(p, 5, "false") != 0)
                return fail("bad literal", err);
            p += 5;
            out = Value::make_bool(false);
            return true;
        case 'n':
            if (s.compare(p, 4, "null") != 0)
                return fail("bad literal", err);
            p += 4;
            out = Value::make_null();
            return true;
        default:
            return parse_number(out, err);
        }
    }
    bool parse_object(Value& out, std::string& err)
    {
        ++p; // '{'
        ++d_depth;
        out = Value::make_object();
        skip_ws();
        if (p < s.size() && s[p] == '}') {
            ++p;
            --d_depth;
            return true;
        }
        while (true) {
            skip_ws();
            if (p >= s.size() || s[p] != '"')
                return fail("expected object key", err);
            std::string key;
            if (!parse_string(key, err))
                return false;
            skip_ws();
            if (p >= s.size() || s[p] != ':')
                return fail("expected ':'", err);
            ++p;
            skip_ws();
            Value v;
            if (!parse_value(v, err))
                return false;
            out.set(key, std::move(v));
            skip_ws();
            if (p < s.size() && s[p] == ',') {
                ++p;
                continue;
            }
            if (p < s.size() && s[p] == '}') {
                ++p;
                --d_depth;
                return true;
            }
            return fail("expected ',' or '}'", err);
        }
    }
    bool parse_array(Value& out, std::string& err)
    {
        ++p; // '['
        ++d_depth;
        out = Value::make_array();
        skip_ws();
        if (p < s.size() && s[p] == ']') {
            ++p;
            --d_depth;
            return true;
        }
        while (true) {
            skip_ws();
            Value v;
            if (!parse_value(v, err))
                return false;
            out.push(std::move(v));
            skip_ws();
            if (p < s.size() && s[p] == ',') {
                ++p;
                continue;
            }
            if (p < s.size() && s[p] == ']') {
                ++p;
                --d_depth;
                return true;
            }
            return fail("expected ',' or ']'", err);
        }
    }
    bool parse_string(std::string& out, std::string& err)
    {
        ++p; // opening quote
        out.clear();
        while (p < s.size()) {
            const char c = s[p++];
            if (c == '"')
                return true;
            if (c == '\\') {
                if (p >= s.size())
                    return fail("unterminated escape", err);
                const char e = s[p++];
                switch (e) {
                case '"':
                    out.push_back('"');
                    break;
                case '\\':
                    out.push_back('\\');
                    break;
                case '/':
                    out.push_back('/');
                    break;
                case 'b':
                    out.push_back('\b');
                    break;
                case 'f':
                    out.push_back('\f');
                    break;
                case 'n':
                    out.push_back('\n');
                    break;
                case 'r':
                    out.push_back('\r');
                    break;
                case 't':
                    out.push_back('\t');
                    break;
                case 'u': {
                    if (p + 4 > s.size())
                        return fail("short \\u escape", err);
                    unsigned cp = 0;
                    for (int i = 0; i < 4; ++i) {
                        const char h = s[p + static_cast<size_t>(i)];
                        cp <<= 4;
                        if (h >= '0' && h <= '9')
                            cp |= static_cast<unsigned>(h - '0');
                        else if (h >= 'a' && h <= 'f')
                            cp |= static_cast<unsigned>(h - 'a' + 10);
                        else if (h >= 'A' && h <= 'F')
                            cp |= static_cast<unsigned>(h - 'A' + 10);
                        else
                            return fail("bad \\u escape", err);
                    }
                    p += 4;
                    // UTF-8 encode (surrogate pairs are passed through as-is;
                    // this config only ever carries ASCII identifiers).
                    if (cp < 0x80) {
                        out.push_back(static_cast<char>(cp));
                    } else if (cp < 0x800) {
                        out.push_back(static_cast<char>(0xC0 | (cp >> 6)));
                        out.push_back(static_cast<char>(0x80 | (cp & 0x3F)));
                    } else {
                        out.push_back(static_cast<char>(0xE0 | (cp >> 12)));
                        out.push_back(static_cast<char>(0x80 | ((cp >> 6) & 0x3F)));
                        out.push_back(static_cast<char>(0x80 | (cp & 0x3F)));
                    }
                    break;
                }
                default:
                    return fail("unknown escape", err);
                }
                continue;
            }
            out.push_back(c);
        }
        return fail("unterminated string", err);
    }
    bool parse_number(Value& out, std::string& err)
    {
        const size_t start = p;
        if (p < s.size() && s[p] == '-')
            ++p;
        if (p >= s.size() || s[p] < '0' || s[p] > '9')
            return fail("expected a number", err);
        if (s[p] == '0') {
            ++p;
        } else {
            while (p < s.size() && s[p] >= '0' && s[p] <= '9')
                ++p;
        }
        bool integral = true;
        if (p < s.size() && s[p] == '.') {
            integral = false;
            ++p;
            if (p >= s.size() || s[p] < '0' || s[p] > '9')
                return fail("expected digits after '.'", err);
            while (p < s.size() && s[p] >= '0' && s[p] <= '9')
                ++p;
        }
        if (p < s.size() && (s[p] == 'e' || s[p] == 'E')) {
            integral = false;
            ++p;
            if (p < s.size() && (s[p] == '+' || s[p] == '-'))
                ++p;
            if (p >= s.size() || s[p] < '0' || s[p] > '9')
                return fail("expected digits in exponent", err);
            while (p < s.size() && s[p] >= '0' && s[p] <= '9')
                ++p;
        }
        const std::string tok = s.substr(start, p - start);
        if (integral) {
            errno = 0;
            char* end = nullptr;
            const long long v = std::strtoll(tok.c_str(), &end, 10);
            if (errno == 0 && end != nullptr && *end == '\0') {
                out = Value::make_int(static_cast<int64_t>(v));
                return true;
            }
            // An integer literal outside int64 is kept as a Double so the
            // reader for an integer field can reject it explicitly.
        }
        errno = 0;
        char* end = nullptr;
        const double d = std::strtod(tok.c_str(), &end);
        if (end == nullptr || *end != '\0')
            return fail("malformed number", err);
        out = Value::make_double(d);
        return true;
    }

    const std::string& s;
    size_t p = 0;
    size_t d_depth = 0;
};

inline bool parse(const std::string& text, Value& out, std::string& err)
{
    Parser parser(text);
    return parser.parse(out, err);
}

inline void escape_into(const std::string& in, std::string& out)
{
    out.push_back('"');
    for (unsigned char c : in) {
        switch (c) {
        case '"':
            out += "\\\"";
            break;
        case '\\':
            out += "\\\\";
            break;
        case '\n':
            out += "\\n";
            break;
        case '\r':
            out += "\\r";
            break;
        case '\t':
            out += "\\t";
            break;
        default:
            if (c < 0x20) {
                char buf[8];
                std::snprintf(buf, sizeof(buf), "\\u%04x", static_cast<unsigned>(c));
                out += buf;
            } else {
                out.push_back(static_cast<char>(c));
            }
        }
    }
    out.push_back('"');
}

// 64-bit integer as a JSON number when safe, else as a decimal string.
inline void put_int(int64_t v, std::string& out)
{
    if (v <= kTwrJsonMaxSafeInteger && v >= -kTwrJsonMaxSafeInteger) {
        out += twr_int_to_text(v);
        return;
    }
    escape_into(twr_int_to_text(v), out);
}

// -- dump ------------------------------------------------------------------

class Dumper
{
public:
    Dumper(const Value& v, bool pretty) : d_v(v), d_pretty(pretty) {}

    bool run(std::string& out, std::string& err)
    {
        m_buf.clear();
        if (!dump(d_v, 0, err))
            return false;
        out = m_buf;
        if (d_pretty)
            out += "\n";
        return true;
    }

private:
    void indent(int depth)
    {
        if (!d_pretty)
            return;
        m_buf += "\n";
        m_buf.append(static_cast<size_t>(depth) * 2, ' ');
    }
    bool dump(const Value& v, int depth, std::string& err)
    {
        switch (v.type) {
        case Type::Null:
            m_buf += "null";
            return true;
        case Type::Bool:
            m_buf += v.boolean ? "true" : "false";
            return true;
        case Type::Int:
            put_int(v.integer, m_buf);
            return true;
        case Type::Double:
            if (!std::isfinite(v.number)) {
                err = "non-finite double cannot be represented in JSON";
                return false;
            }
            m_buf += twr_double_to_text(v.number);
            return true;
        case Type::String:
            escape_into(v.text, m_buf);
            return true;
        case Type::Array: {
            if (v.items.empty()) {
                m_buf += "[]";
                return true;
            }
            m_buf += "[";
            for (size_t i = 0; i < v.items.size(); ++i) {
                if (i)
                    m_buf += ",";
                indent(depth + 1);
                if (!dump(v.items[i], depth + 1, err))
                    return false;
            }
            indent(depth);
            m_buf += "]";
            return true;
        }
        case Type::Object: {
            if (v.members.empty()) {
                m_buf += "{}";
                return true;
            }
            m_buf += "{";
            for (size_t i = 0; i < v.members.size(); ++i) {
                if (i)
                    m_buf += ",";
                indent(depth + 1);
                escape_into(v.members[i].first, m_buf);
                m_buf += ":";
                if (d_pretty)
                    m_buf += " ";
                if (!dump(v.members[i].second, depth + 1, err))
                    return false;
            }
            indent(depth);
            m_buf += "}";
            return true;
        }
        }
        err = "unreachable json type";
        return false;
    }

    const Value& d_v;
    bool d_pretty;
    std::string m_buf;
};

inline bool dump(const Value& v, std::string& out, std::string& err, bool pretty = true)
{
    Dumper d(v, pretty);
    return d.run(out, err);
}

} // namespace json

// ===========================================================================
// 9. TwrConfig <-> JSON
// ===========================================================================

// Small ordered object writer.  Emits every field, so the round trip is
// exact and a config file documents itself.
class ConfigJsonWriter
{
public:
    ConfigJsonWriter() { m_stack.push_back(&m_root); }

    bool ok() const { return m_err.empty(); }
    const std::string& error() const { return m_err; }
    json::Value value() const { return m_root; }
    std::string to_string(bool pretty = true) const
    {
        std::string out;
        std::string err;
        if (!json::dump(m_root, out, err, pretty)) {
            return std::string();
        }
        return out;
    }

    // The root is already an object; this exists so the document shape reads
    // the same as the reader.
    void current_object() {}

    void begin_group(const std::string& key)
    {
        m_stack.back()->set(key, json::Value::make_object());
        m_stack.push_back(m_stack.back()->find(key));
    }
    void begin_array(const std::string& key)
    {
        m_stack.back()->set(key, json::Value::make_array());
        m_stack.push_back(m_stack.back()->find(key));
    }
    void end() { m_stack.pop_back(); }

    void push_object()
    {
        json::Value* parent = m_stack.back();
        parent->push(json::Value::make_object());
        m_stack.push_back(&parent->items.back());
    }
    void pop_object() { m_stack.pop_back(); }

    void put(const std::string& key, const std::string& v)
    {
        current()->set(key, json::Value::make_string(v));
    }
    void put(const std::string& key, int64_t v)
    {
        current()->set(key, json::Value::make_int(v));
    }
    void put(const std::string& key, uint32_t v)
    {
        current()->set(key, json::Value::make_int(static_cast<int64_t>(v)));
    }
    void put(const std::string& key, uint16_t v)
    {
        current()->set(key, json::Value::make_int(static_cast<int64_t>(v)));
    }
    void put(const std::string& key, uint8_t v)
    {
        current()->set(key, json::Value::make_int(static_cast<int64_t>(v)));
    }
    void put(const std::string& key, int v)
    {
        current()->set(key, json::Value::make_int(static_cast<int64_t>(v)));
    }
    void put(const std::string& key, uint64_t v)
    {
        // uint64 above int64 max is not representable; the schema has no such
        // field, but keep the check explicit rather than wrapping silently.
        if (v > static_cast<uint64_t>(INT64_MAX)) {
            fail("field " + key + " exceeds the int64 range of the JSON schema");
            current()->set(key, json::Value::make_null());
            return;
        }
        current()->set(key, json::Value::make_int(static_cast<int64_t>(v)));
    }
    void put(const std::string& key, bool v)
    {
        current()->set(key, json::Value::make_bool(v));
    }
    void put(const std::string& key, double v)
    {
        if (!std::isfinite(v)) {
            fail("field " + key + " is not finite and cannot be written as JSON");
            current()->set(key, json::Value::make_null());
            return;
        }
        current()->set(key, json::Value::make_double(v));
    }
    void put(const std::string& key, const char* v)
    {
        current()->set(key, json::Value::make_string(v ? v : ""));
    }
    void put_ns(const std::string& key, const Duration& d)
    {
        current()->set(key, json::Value::make_int(d.nanos()));
    }
    void put_opt(const std::string& key, const Opt<int64_t>& o)
    {
        current()->set(key,
                       o.has() ? json::Value::make_int(o.value()) : json::Value::make_null());
    }
    void put_opt(const std::string& key, const Opt<uint32_t>& o)
    {
        current()->set(key,
                       o.has() ? json::Value::make_int(static_cast<int64_t>(o.value()))
                               : json::Value::make_null());
    }
    void put_opt(const std::string& key, const Opt<double>& o)
    {
        if (!o.has()) {
            current()->set(key, json::Value::make_null());
            return;
        }
        if (!std::isfinite(o.value())) {
            fail("field " + key + " is not finite and cannot be written as JSON");
            current()->set(key, json::Value::make_null());
            return;
        }
        current()->set(key, json::Value::make_double(o.value()));
    }
    void put_timed(const std::string& key, const TimedField& t)
    {
        json::Value o = json::Value::make_object();
        o.set("ns", json::Value::make_int(t.value.nanos()));
        o.set("domain", json::Value::make_string(time_domain_to_string(t.domain)));
        o.set("reference",
            json::Value::make_string(time_reference_event_to_string(t.reference)));
        if (t.marker.has())
            o.set("marker",
                json::Value::make_string(timestamp_marker_to_string(t.marker.value())));
        else
            o.set("marker", json::Value::make_null());
        o.set("quantisation_hz", json::Value::make_double(t.required_quantisation_hz));
        o.set("max_quantisation_error_ns",
              json::Value::make_int(t.max_quantisation_error_ns));
        o.set("note", json::Value::make_string(t.note));
        current()->set(key, std::move(o));
    }

private:
    json::Value* current() { return m_stack.back(); }
    void fail(const std::string& m)
    {
        if (m_err.empty())
            m_err = m;
    }

    json::Value m_root;
    std::vector<json::Value*> m_stack;
    std::string m_err;
};

inline void write_config_json(const TwrConfig& c, ConfigJsonWriter& w)
{
    w.current_object();
    w.begin_group("meta");
    w.put("schema_version", c.meta.schema_version);
    w.put("profile_version", c.meta.profile_version);
    w.put("calibration_version", c.meta.calibration_version);
    w.put("label", c.meta.label);
    w.end();

    w.begin_group("session");
    w.put("protocol", protocol_to_string(c.session.protocol));
    w.put("role", role_to_string(c.session.role));
    w.put("local_address", c.session.local_address);
    w.put("peer_address", c.session.peer_address);
    w.put("pan_id", c.session.pan_id);
    w.put("session_id", c.session.session_id);
    w.put("exchange_id", c.session.exchange_id);
    w.put("sequence", c.session.sequence);
    w.put("sequence_modulus", c.session.sequence_modulus);
    w.put("measurement_count", c.session.measurement_count);
    w.put_ns("measurement_interval_ns", c.session.measurement_interval);
    w.put("max_attempts_per_exchange", c.session.max_attempts_per_exchange);
    w.put_ns("retry_backoff_ns", c.session.retry_backoff);
    w.put("max_in_flight_exchanges", c.session.max_in_flight_exchanges);
    w.put("require_pan_match", c.session.require_pan_match);
    w.put("require_address_match", c.session.require_address_match);
    w.end();

    w.begin_group("phy");
    w.put("channel", c.phy.channel);
    w.put("center_frequency_hz", c.phy.center_frequency_hz);
    w.put("tx_preamble_code", c.phy.tx_preamble_code);
    w.put("rx_preamble_code", c.phy.rx_preamble_code);
    w.put("preamble_symbols", c.phy.preamble_symbols);
    w.put("prf_class", prf_class_to_string(c.phy.prf_class));
    w.put("data_rate", data_rate_to_string(c.phy.data_rate));
    // PHR rate, in the PHR-rate spelling ("850k" / "same_as_data"), NOT the
    // payload-rate spelling: the two axes have separate value domains and a
    // document that used "6p8m" here was describing a PHR this build cannot
    // transmit.
    w.put("phr_rate", phr_rate_to_string(c.phy.phr_rate));
    w.end();

    w.begin_group("frame");
    w.put("frame_profile", frame_profile_id_to_string(c.frame.frame_profile));
    w.put("sfd_mode", sfd_mode_to_string(c.frame.sfd_mode));
    w.put("sfd_symbols", c.frame.sfd_symbols);
    w.put_timed("sfd_timeout", c.frame.sfd_timeout);
    w.put("phr_mode", phr_mode_to_string(c.frame.phr_mode));
    w.put("ranging_bit", c.frame.ranging_bit);
    // The geometry CLAIM is written out verbatim, so a reader can see what the
    // operator asserted; the validator is what decides whether it agrees with
    // the codec.
    w.begin_group("geometry");
    w.put("mac_header_bytes", c.frame.geometry.mac_header_bytes);
    w.put("timestamp_bytes", c.frame.geometry.timestamp_bytes);
    w.put("mac_footer_bytes", c.frame.geometry.mac_footer_bytes);
    w.put("mac_fcs_bytes", c.frame.geometry.mac_fcs_bytes);
    w.put("phr_bytes", c.frame.geometry.phr_bytes);
    w.end();
    w.put("mac_psdu_bytes", c.frame.mac_psdu_bytes);
    w.put("mac_psdu_includes_fcs", c.frame.mac_psdu_includes_fcs);
    w.put("fcs_append", fcs_appender_to_string(c.frame.fcs_append));
    w.put("fcs_bytes", c.frame.fcs_bytes);
    w.put("application_payload_bytes", c.frame.application_payload_bytes);
    w.put("sts_mode", sts_mode_to_string(c.frame.sts_mode));
    w.put("sts_length_symbols", c.frame.sts_length_symbols);
    w.end();

    w.begin_group("tx");
    w.put("port", c.tx.port);
    w.put_opt("gain_db", c.tx.gain_db);
    w.put_opt("iq_amplitude", c.tx.iq_amplitude);
    w.put_opt("calibrated_tx_power_dbm", c.tx.calibrated_tx_power_dbm);
    w.put("power_policy", tx_power_policy_to_string(c.tx.power_policy));
    w.put("pulse_shaping", pulse_shaping_to_string(c.tx.pulse_shaping));
    w.put_opt("vendor_power_word", c.tx.vendor_power_word);
    w.put("vendor_power_word_backend", c.tx.vendor_power_word_backend);
    w.end();

    w.begin_group("rx");
    w.put("port", c.rx.port);
    w.put_opt("gain_db", c.rx.gain_db);
    w.put("agc", agc_mode_to_string(c.rx.agc));
    w.put_opt("bandwidth_hz", c.rx.bandwidth_hz);
    w.put("detection_threshold", c.rx.detection_threshold);
    w.put("correlation_threshold", c.rx.correlation_threshold);
    w.put("first_path_threshold", c.rx.first_path_threshold);
    w.put("first_path_index", c.rx.first_path_index);
    w.put("first_path_window", c.rx.first_path_window);
    w.put_opt("vendor_pac_value", c.rx.vendor_pac_value);
    w.put("vendor_pac_backend", c.rx.vendor_pac_backend);
    w.put_opt("vendor_pac_applied_step", c.rx.vendor_pac_applied_step);
    w.end();

    w.begin_group("radio");
    w.put("device_args", c.radio.device_args);
    w.put("tx_channel", c.radio.tx_channel);
    w.put("rx_channel", c.radio.rx_channel);
    w.put("native_sample_rate_hz", c.radio.native_sample_rate_hz);
    w.put("clock_source", c.radio.clock_source);
    w.put("time_source", c.radio.time_source);
    w.put("fpga_image", c.radio.fpga_image);
    w.put("dpdk_config", c.radio.dpdk_config);
    w.begin_array("peers");
    for (const auto& p : c.radio.peers) {
        w.push_object();
        w.put("id", p.id);
        w.put("role", role_to_string(p.role));
        w.put("tx_channel", p.tx_channel);
        w.put("rx_channel", p.rx_channel);
        w.put("native_sample_rate_hz", p.native_sample_rate_hz);
        w.put("occupies_resources", p.occupies_resources);
        w.pop_object();
    }
    w.end();
    w.begin_group("readback");
    w.put("present", c.radio.readback.present);
    w.put("sample_rate_hz", c.radio.readback.sample_rate_hz);
    w.put("center_freq_hz", c.radio.readback.center_freq_hz);
    w.put("tx_channel", c.radio.readback.tx_channel);
    w.put("rx_channel", c.radio.readback.rx_channel);
    w.put("mpm_string", c.radio.readback.mpm_string);
    w.put("fpga_image", c.radio.readback.fpga_image);
    w.put("uhd_version", c.radio.readback.uhd_version);
    w.put("clock_source", c.radio.readback.clock_source);
    w.put("time_source", c.radio.readback.time_source);
    w.end();
    w.put("require_readback", c.radio.require_readback);
    w.end();

    w.begin_group("timing");
    w.put_timed("poll_start", c.timing.poll_start);
    w.put_timed("poll_to_response", c.timing.poll_to_response);
    w.put_timed("response_to_final", c.timing.response_to_final);
    w.put_timed("final_to_report", c.timing.final_to_report);
    w.put_timed("post_tx_rx_enable", c.timing.post_tx_rx_enable);
    w.put_timed("min_tx_lead_time", c.timing.min_tx_lead_time);
    w.end();

    w.begin_group("timeouts");
    w.put_timed("poll_rx_window", c.timeouts.poll_rx_window);
    w.put_timed("response_rx_window", c.timeouts.response_rx_window);
    w.put_timed("final_rx_window", c.timeouts.final_rx_window);
    w.put_timed("report_rx_window", c.timeouts.report_rx_window);
    w.put_timed("rx_timeout", c.timeouts.rx_timeout);
    w.put_timed("exchange_timeout", c.timeouts.exchange_timeout);
    w.put_timed("retry_interval", c.timeouts.retry_interval);
    w.end();

    w.begin_group("calibration");
    w.put_ns("tx_link_delay_ns", c.calibration.tx_link_delay);
    w.put_ns("rx_link_delay_ns", c.calibration.rx_link_delay);
    w.put_ns("antenna_delay_ns", c.calibration.antenna_delay);
    w.put_ns("cable_delay_ns", c.calibration.cable_delay);
    w.put("tx_link_delay_native_ticks", c.calibration.tx_link_delay_native_ticks);
    w.put("rx_link_delay_native_ticks", c.calibration.rx_link_delay_native_ticks);
    w.put("native_sample_rate_hz", c.calibration.native_sample_rate_hz);
    w.put("link_delay_unit", time_unit_to_string(c.calibration.link_delay_unit));
    w.put("first_path_algorithm",
          first_path_algorithm_to_string(c.calibration.first_path_algorithm));
    w.put("cfo_compensation",
        compensation_flag_to_string(c.calibration.cfo_compensation));
    w.put("sfo_compensation",
        compensation_flag_to_string(c.calibration.sfo_compensation));
    w.put("calibration_id", c.calibration.calibration_id);
    w.begin_group("record");
    w.put("calibration_id", c.calibration.record.calibration_id);
    w.put("device_serial", c.calibration.record.device_serial);
    w.put("channel", c.calibration.record.channel);
    w.put("native_sample_rate_hz", c.calibration.record.native_sample_rate_hz);
    w.put("profile_version", c.calibration.record.profile_version);
    w.put_opt("gain_db", c.calibration.record.gain_db);
    w.put("valid_until_monotonic_ns", c.calibration.record.valid_until_monotonic_ns);
    w.end();
    w.put("applied_count", c.calibration.applied_count);
    w.put("calibration_required", c.calibration.calibration_required);
    w.end();

    w.begin_group("diagnostics");
    w.put("cir_capture_enabled", c.diagnostics.cir_capture_enabled);
    w.put("cir_capture_max_bytes", c.diagnostics.cir_capture_max_bytes);
    w.put("cir_capture_stride", c.diagnostics.cir_capture_stride);
    w.put("short_iq_enabled", c.diagnostics.short_iq_enabled);
    w.put("short_iq_max_bytes", c.diagnostics.short_iq_max_bytes);
    w.put("short_iq_stride", c.diagnostics.short_iq_stride);
    w.put("raw_frame_dump", c.diagnostics.raw_frame_dump);
    w.put("raw_frame_max_bytes", c.diagnostics.raw_frame_max_bytes);
    w.put("result_output_path", c.diagnostics.result_output_path);
    w.put("result_queue_capacity", c.diagnostics.result_queue_capacity);
    w.put("event_queue_capacity", c.diagnostics.event_queue_capacity);
    w.put_timed("stats_cadence", c.diagnostics.stats_cadence);
    w.put("io_on_realtime_thread", c.diagnostics.io_on_realtime_thread);
    w.end();
}

// Strict reader: every failure is a machine-readable violation, never a
// silently skipped key and never a default.
class ConfigJsonReader
{
public:
    void bad(const std::string& field,
             ConfigReason r,
             const std::string& msg,
             ExchangeStatus st = ExchangeStatus::ConfigRejected)
    {
        m_report.add(field, r, st, msg, "REQ-API-01");
    }

    ValidationReport report() const { return m_report; }
    bool ok() const { return m_report.ok(); }

    // -- object navigation ------------------------------------------------
    const json::Value* group(const std::string& path, const json::Value* parent)
    {
        const json::Value* g = parent ? parent->find(path) : nullptr;
        if (!g) {
            bad(path, ConfigReason::MissingKey, "missing required group");
            return nullptr;
        }
        if (g->type != json::Type::Object) {
            bad(path, ConfigReason::TypeMismatch, "expected an object, got " +
                                                         std::string(g->type_name()));
            return nullptr;
        }
        return g;
    }

    // -- typed getters ----------------------------------------------------
    std::string get_str(const json::Value* o, const std::string& path, const char* key)
    {
        const json::Value* v = key_of(o, path, key);
        if (!v)
            return std::string();
        if (v->type == json::Type::Null)
            return std::string();
        if (v->type != json::Type::String) {
            bad(path + "." + key, ConfigReason::TypeMismatch,
                "expected a string, got " + std::string(v->type_name()));
            return std::string();
        }
        return v->text;
    }
    bool get_bool(const json::Value* o, const std::string& path, const char* key)
    {
        const json::Value* v = key_of(o, path, key);
        if (!v)
            return false;
        if (v->type != json::Type::Bool) {
            bad(path + "." + key, ConfigReason::TypeMismatch,
                "expected a bool, got " + std::string(v->type_name()));
            return false;
        }
        return v->boolean;
    }
    int64_t get_i64(const json::Value* o, const std::string& path, const char* key)
    {
        const json::Value* v = key_of(o, path, key);
        if (!v)
            return 0;
        if (v->type == json::Type::String) {
            // The decimal-string form used for values beyond 2^53.
            int64_t out = 0;
            if (parse_int64_strict(v->text, out))
                return out;
            bad(path + "." + key, ConfigReason::IntegerPrecisionLoss,
                "string is not a decimal int64: '" + v->text + "'");
            return 0;
        }
        if (v->type != json::Type::Int) {
            bad(path + "." + key, ConfigReason::TypeMismatch,
                "expected an integer, got " + std::string(v->type_name()) +
                    " (a decimal fraction or exponent is never truncated silently)");
            return 0;
        }
        return v->integer;
    }

    // -- range-checked unsigned narrowing (M0.1 / N01) --------------------
    //
    // The reader used to assign through `static_cast<uint16_t>(get_i64(...))`,
    // which WRAPS an out-of-range JSON number instead of refusing it: channel
    // 261 became channel 5, session 2^32+1 became session 1, local_address
    // 65537 became 1.  The validator then never saw the value the document
    // actually carried, and two different documents could produce the same
    // runtime configuration -- exactly the silent fallback REQ-CFG forbids and
    // the reason `session_id_to_wire()` exists.
    //
    // A C++ cast is not a specification.  These getters are: the value must be
    // in [0, max] or it is refused with `out_of_range` and stored as 0.  They
    // keep `get_i64`'s shape (same arguments) so the call sites read the same
    // way, but the range check happens BEFORE any narrowing.
    bool check_unsigned(const std::string& path, const char* key, int64_t v,
                        uint64_t max_value, uint64_t& out)
    {
        // Formatted unsigned: `twr_int_to_text` takes an int64_t, so passing a
        // uint64_t max would print a negative bound.
        const std::string bound = std::to_string(
            static_cast<unsigned long long>(max_value));
        if (v < 0) {
            bad(path + "." + key, ConfigReason::OutOfRange,
                "must be in [0, " + bound + "], got " + twr_int_to_text(v) +
                    " (a negative value is refused, never wrapped to fit the field)");
            out = 0u;
            return false;
        }
        const uint64_t u = static_cast<uint64_t>(v);
        if (u > max_value) {
            bad(path + "." + key, ConfigReason::OutOfRange,
                "must be in [0, " + bound + "], got " + twr_int_to_text(v) +
                    " (out of range for this field; refused, not masked)");
            out = 0u;
            return false;
        }
        out = u;
        return true;
    }

    uint64_t get_u64(const json::Value* o, const std::string& path, const char* key,
                     uint64_t max_value)
    {
        const json::Value* v = key_of(o, path, key);
        if (!v)
            return 0u;
        int64_t raw = 0;
        if (v->type == json::Type::String) {
            if (!parse_int64_strict(v->text, raw)) {
                bad(path + "." + key, ConfigReason::IntegerPrecisionLoss,
                    "string is not a decimal int64: '" + v->text + "'");
                return 0u;
            }
        } else if (v->type != json::Type::Int) {
            bad(path + "." + key, ConfigReason::TypeMismatch,
                "expected an integer, got " + std::string(v->type_name()) +
                    " (a decimal fraction or exponent is never truncated silently)");
            return 0u;
        } else {
            raw = v->integer;
        }
        uint64_t out = 0u;
        check_unsigned(path, key, raw, max_value, out);
        return out;
    }

    uint8_t get_u8(const json::Value* o, const std::string& path, const char* key)
    {
        return static_cast<uint8_t>(get_u64(o, path, key, 0xFFu));
    }
    uint16_t get_u16(const json::Value* o, const std::string& path, const char* key)
    {
        return static_cast<uint16_t>(get_u64(o, path, key, 0xFFFFu));
    }
    uint32_t get_u32(const json::Value* o, const std::string& path, const char* key)
    {
        return static_cast<uint32_t>(get_u64(o, path, key, 0xFFFFFFFFu));
    }
    double get_double(const json::Value* o, const std::string& path, const char* key)
    {
        const json::Value* v = key_of(o, path, key);
        if (!v)
            return 0.0;
        if (!v->is_number()) {
            bad(path + "." + key, ConfigReason::TypeMismatch,
                "expected a number, got " + std::string(v->type_name()));
            return 0.0;
        }
        return v->as_double();
    }
    Duration get_dur(const json::Value* o, const std::string& path, const char* key)
    {
        return Duration::from_nanos(get_i64(o, path, key));
    }
    // -- optional getters ------------------------------------------------
    Opt<int64_t> get_opt_i64(const json::Value* o, const std::string& path,
        const char* key)
    {
        const json::Value* v = key_of(o, path, key, /*required=*/false);
        if (!v || v->type == json::Type::Null)
            return Opt<int64_t>::none();
        const std::string p = path + "." + key;
        Opt<int64_t> out;
        if (v->type == json::Type::String) {
            int64_t parsed = 0;
            if (!parse_int64_strict(v->text, parsed)) {
                bad(p, ConfigReason::IntegerPrecisionLoss,
                    "string is not a decimal int64: '" + v->text + "'");
                return Opt<int64_t>::none();
            }
            out = parsed;
        } else if (v->type == json::Type::Int) {
            out = v->integer;
        } else {
            bad(p, ConfigReason::TypeMismatch,
                "expected an integer or null, got " + std::string(v->type_name()));
        }
        return out;
    }
    Opt<uint32_t> get_opt_u32(const json::Value* o, const std::string& path,
        const char* key)
    {
        const Opt<int64_t> v = get_opt_i64(o, path, key);
        if (!v.has())
            return Opt<uint32_t>::none();
        if (v.value() < 0 || v.value() > 0xffffffffLL) {
            bad(path + "." + key, ConfigReason::OutOfRange, "outside the uint32 range");
            return Opt<uint32_t>::none();
        }
        return Opt<uint32_t>(static_cast<uint32_t>(v.value()));
    }
    Opt<double> get_opt_double(const json::Value* o, const std::string& path,
        const char* key)
    {
        const json::Value* v = key_of(o, path, key, /*required=*/false);
        if (!v || v->type == json::Type::Null)
            return Opt<double>::none();
        if (!v->is_number()) {
            bad(path + "." + key, ConfigReason::TypeMismatch,
                "expected a number or null, got " + std::string(v->type_name()));
            return Opt<double>::none();
        }
        return Opt<double>(v->as_double());
    }
    // -- enum getters ----------------------------------------------------
    template <typename E, typename F>
    E get_enum(const json::Value* o, const std::string& path, const char* key,
        F from_string)
    {
        const json::Value* v = key_of(o, path, key);
        if (!v)
            return static_cast<E>(0);
        if (v->type != json::Type::String) {
            bad(path + "." + key, ConfigReason::TypeMismatch,
                "expected a string enum, got " + std::string(v->type_name()));
            return static_cast<E>(0);
        }
        E out = static_cast<E>(0);
        if (!from_string(v->text, out)) {
            bad(path + "." + key, ConfigReason::UnknownEnumValue,
                "'" + v->text + "' is not a known value of this enum");
            return static_cast<E>(0);
        }
        return out;
    }
    TimedField get_timed(const json::Value* o, const std::string& path, const char* key)
    {
        TimedField t;
        const std::string p = path + "." + key;
        const json::Value* v = key_of(o, path, key);
        if (!v)
            return t;
        if (v->type != json::Type::Object) {
            bad(p, ConfigReason::TypeMismatch,
                "expected an object, got " + std::string(v->type_name()));
            return t;
        }
        t.value = Duration::from_nanos(get_i64(v, p, "ns"));
        t.domain = get_enum<TimeDomain>(v, p, "domain", time_domain_from_string);
        t.reference = get_enum<TimeReferenceEvent>(v, p, "reference",
                                                   time_reference_event_from_string);
        const json::Value* mk = key_of(v, p, "marker", /*required=*/false);
        if (mk && mk->type == json::Type::String) {
            bool found = false;
            for (int i = 0; i <= static_cast<int>(TimestampMarker::AntennaPlane); ++i) {
                const TimestampMarker m = static_cast<TimestampMarker>(i);
                if (mk->text == timestamp_marker_to_string(m)) {
                    t.marker = m;
                    found = true;
                    break;
                }
            }
            if (!found)
                bad(p + ".marker", ConfigReason::UnknownEnumValue,
                    "'" + mk->text + "' is not a known timestamp marker");
        } else if (mk && mk->type != json::Type::Null) {
            bad(p + ".marker", ConfigReason::TypeMismatch,
                "expected a marker name or null, got " + std::string(mk->type_name()));
        }
        t.required_quantisation_hz = get_double(v, p, "quantisation_hz");
        t.max_quantisation_error_ns = get_i64(v, p, "max_quantisation_error_ns");
        t.note = get_str(v, p, "note");
        return t;
    }

    static bool parse_int64_strict(const std::string& s, int64_t& out)
    {
        if (s.empty())
            return false;
        size_t i = 0;
        if (s[0] == '-')
            i = 1;
        if (i >= s.size())
            return false;
        for (size_t k = i; k < s.size(); ++k)
            if (s[k] < '0' || s[k] > '9')
                return false;
        errno = 0;
        char* end = nullptr;
        const long long v = std::strtoll(s.c_str(), &end, 10);
        if (errno != 0 || end == nullptr || *end != '\0')
            return false;
        out = static_cast<int64_t>(v);
        return true;
    }

    // Reject keys the schema does not define: an unknown key is a typo or an
    // unsupported feature, and either way it must not be ignored.
    void reject_unknown_keys(const json::Value* o, const std::string& path,
        const char* const* known)
    {
        if (!o || o->type != json::Type::Object)
            return;
        for (const auto& m : o->members) {
            bool found = false;
            for (size_t i = 0; known[i] != nullptr; ++i) {
                if (m.first == known[i]) {
                    found = true;
                    break;
                }
            }
            if (!found)
                bad(path + "." + m.first, ConfigReason::UnknownKey,
                    "key is not part of schema twr-config/2");
        }
    }

private:
    const json::Value* key_of(const json::Value* o,
                              const std::string& path,
                              const char* key,
                              bool required = true)
    {
        if (!o)
            return nullptr;
        const json::Value* v = o->find(key);
        if (!v) {
            if (required)
                bad(path + "." + key, ConfigReason::MissingKey, "missing required key");
            return nullptr;
        }
        return v;
    }
    ValidationReport m_report;
};

// ---------------------------------------------------------------------------

inline bool to_json_string(const TwrConfig& c, std::string& out, std::string& err)
{
    ConfigJsonWriter w;
    write_config_json(c, w);
    if (!w.ok()) {
        err = w.error();
        return false;
    }
    return json::dump(w.value(), out, err, /*pretty=*/true);
}

// Convenience: the frozen snapshot plus its report, as one JSON document.
inline bool to_json_string(const EffectiveConfig& e, std::string& out, std::string& err)
{
    ConfigJsonWriter w;
    w.current_object();
    w.put("ok", e.ok);
    w.put("schema_version", e.schema_version);
    w.put("profile_version", e.profile_version);
    w.put("calibration_version", e.calibration_version);
    w.put("config_hash", e.config_hash);
    write_config_json(e.effective, w);
    w.begin_array("changes");
    for (const auto& ch : e.changes) {
        w.push_object();
        w.put("path", ch.path);
        w.put("requested", ch.requested);
        w.put("effective", ch.effective);
        w.put("note", ch.note);
        w.pop_object();
    }
    w.end();
    w.begin_array("violations");
    for (const auto& v : e.validation.violations) {
        w.push_object();
        w.put("field", v.field);
        w.put("reason", config_reason_to_string(v.reason));
        w.put("status", exchange_status_to_string(v.status));
        w.put("message", v.message);
        w.put("requirement", v.requirement);
        w.pop_object();
    }
    w.end();
    w.begin_group("quantised");
    w.put("tick_rate_hz", e.tick_rate_hz);
    w.put("poll_start_ticks", e.poll_start_ticks);
    w.put("poll_to_response_ticks", e.poll_to_response_ticks);
    w.put("response_to_final_ticks", e.response_to_final_ticks);
    w.put("post_tx_rx_enable_ticks", e.post_tx_rx_enable_ticks);
    w.put_ns("poll_to_response_effective_ns", e.poll_to_response_effective);
    w.put_ns("response_to_final_effective_ns", e.response_to_final_effective);
    w.put_ns("post_tx_rx_enable_effective_ns", e.post_tx_rx_enable_effective);
    w.end();
    w.begin_group("frame_budget");
    w.put("poll_bytes", e.poll_bytes);
    w.put("response_bytes", e.response_bytes);
    w.put("final_bytes", e.final_bytes);
    w.put("max_psdu_bytes", e.max_psdu_bytes);
    w.end();
    if (!w.ok()) {
        err = w.error();
        return false;
    }
    return json::dump(w.value(), out, err, /*pretty=*/true);
}

inline ValidationReport from_json_string(const std::string& text, TwrConfig& out)
{
    ConfigJsonReader rd;
    json::Value root;
    std::string err;
    if (!json::parse(text, root, err)) {
        rd.bad("", ConfigReason::MalformedJson, err);
        return rd.report();
    }
    if (root.type != json::Type::Object) {
        rd.bad("", ConfigReason::TypeMismatch,
            "the config document must be a JSON object");
        return rd.report();
    }
    TwrConfig c;

    {
        const std::string p = "meta";
        const json::Value* g = rd.group(p, &root);
        if (g) {
            static const char* const kKeys[] = { "schema_version", "profile_version",
                                                 "calibration_version", "label", nullptr };
            rd.reject_unknown_keys(g, p, kKeys);
            c.meta.schema_version = rd.get_str(g, p, "schema_version");
            c.meta.profile_version = rd.get_str(g, p, "profile_version");
            c.meta.calibration_version = rd.get_str(g, p, "calibration_version");
            c.meta.label = rd.get_str(g, p, "label");
        }
    }
    {
        const std::string p = "session";
        const json::Value* g = rd.group(p, &root);
        if (g) {
            static const char* const kKeys[] = {
                "protocol", "role", "local_address", "peer_address", "pan_id", "session_id",
                "exchange_id", "sequence", "sequence_modulus", "measurement_count",
                "measurement_interval_ns", "max_attempts_per_exchange", "retry_backoff_ns",
                "max_in_flight_exchanges", "require_pan_match", "require_address_match", nullptr
            };
            rd.reject_unknown_keys(g, p, kKeys);
            c.session.protocol = rd.get_enum<Protocol>(g, p, "protocol",
                protocol_from_string);
            c.session.role = rd.get_enum<Role>(g, p, "role", role_from_string);
            c.session.local_address =
                static_cast<uint16_t>(rd.get_u16(g, p, "local_address"));
            c.session.peer_address = static_cast<uint16_t>(rd.get_u16(g, p,
                "peer_address"));
            c.session.pan_id = static_cast<uint16_t>(rd.get_u16(g, p, "pan_id"));
            c.session.session_id = static_cast<uint32_t>(rd.get_u32(g, p,
                "session_id"));
            c.session.exchange_id = static_cast<uint32_t>(rd.get_u32(g, p,
                "exchange_id"));
            c.session.sequence = static_cast<uint8_t>(rd.get_u8(g, p, "sequence"));
            c.session.sequence_modulus =
                static_cast<uint16_t>(rd.get_u16(g, p, "sequence_modulus"));
            c.session.measurement_count =
                static_cast<uint32_t>(rd.get_u32(g, p, "measurement_count"));
            c.session.measurement_interval = rd.get_dur(g, p,
                "measurement_interval_ns");
            c.session.max_attempts_per_exchange =
                static_cast<uint32_t>(rd.get_u32(g, p, "max_attempts_per_exchange"));
            c.session.retry_backoff = rd.get_dur(g, p, "retry_backoff_ns");
            c.session.max_in_flight_exchanges =
                static_cast<uint32_t>(rd.get_u32(g, p, "max_in_flight_exchanges"));
            c.session.require_pan_match = rd.get_bool(g, p, "require_pan_match");
            c.session.require_address_match = rd.get_bool(g, p,
                "require_address_match");
        }
    }
    {
        const std::string p = "phy";
        const json::Value* g = rd.group(p, &root);
        if (g) {
            static const char* const kKeys[] = {
                "channel", "center_frequency_hz", "tx_preamble_code", "rx_preamble_code",
                "preamble_symbols", "prf_class", "data_rate", "phr_rate", nullptr
            };
            rd.reject_unknown_keys(g, p, kKeys);
            c.phy.channel = static_cast<uint8_t>(rd.get_u8(g, p, "channel"));
            c.phy.center_frequency_hz = rd.get_double(g, p, "center_frequency_hz");
            c.phy.tx_preamble_code = static_cast<uint8_t>(rd.get_u8(g, p,
                "tx_preamble_code"));
            c.phy.rx_preamble_code = static_cast<uint8_t>(rd.get_u8(g, p,
                "rx_preamble_code"));
            c.phy.preamble_symbols = static_cast<uint16_t>(rd.get_u16(g, p,
                "preamble_symbols"));
            c.phy.prf_class = rd.get_enum<PrfClass>(g, p, "prf_class",
                prf_class_from_string);
            c.phy.data_rate = rd.get_enum<DataRate>(g, p, "data_rate",
                data_rate_from_string);
            // PHR rate, read in the PHR-rate domain.  A payload-rate name such
            // as "6p8m" is now an UnknownEnumValue here, which is the point:
            // the two axes no longer share a value domain.
            c.phy.phr_rate = rd.get_enum<PhrRate>(g, p, "phr_rate",
                phr_rate_from_string);
        }
    }
    {
        const std::string p = "frame";
        const json::Value* g = rd.group(p, &root);
        if (g) {
            static const char* const kKeys[] = {
                "frame_profile",
                "sfd_mode", "sfd_symbols", "sfd_timeout", "phr_mode", "ranging_bit", "geometry",
                "mac_psdu_bytes", "mac_psdu_includes_fcs", "fcs_append", "fcs_bytes",
                "application_payload_bytes", "sts_mode", "sts_length_symbols", nullptr
            };
            rd.reject_unknown_keys(g, p, kKeys);
            c.frame.frame_profile = rd.get_enum<FrameProfileId>(g, p, "frame_profile",
                frame_profile_id_from_string);
            c.frame.sfd_mode = rd.get_enum<SfdMode>(g, p, "sfd_mode",
                sfd_mode_from_string);
            c.frame.sfd_symbols = static_cast<uint16_t>(rd.get_u16(g, p,
                "sfd_symbols"));
            c.frame.sfd_timeout = rd.get_timed(g, p, "sfd_timeout");
            c.frame.phr_mode = rd.get_enum<PhrMode>(g, p, "phr_mode",
                phr_mode_from_string);
            c.frame.ranging_bit = rd.get_bool(g, p, "ranging_bit");
            const json::Value* geo = g->find("geometry");
            if (!geo)
                rd.bad(p + ".geometry", ConfigReason::MissingKey,
                    "missing required group");
            else if (geo->type != json::Type::Object)
                rd.bad(p + ".geometry", ConfigReason::TypeMismatch,
                    "expected an object");
            else {
                static const char* const kGeo[] = { "mac_header_bytes",
                    "timestamp_bytes",
                                                    "mac_footer_bytes", "mac_fcs_bytes",
                                                    "phr_bytes", nullptr };
                rd.reject_unknown_keys(geo, p + ".geometry", kGeo);
                c.frame.geometry.mac_header_bytes =
                    static_cast<uint16_t>(rd.get_u16(geo, p + ".geometry",
                        "mac_header_bytes"));
                c.frame.geometry.timestamp_bytes =
                    static_cast<uint16_t>(rd.get_u16(geo, p + ".geometry",
                        "timestamp_bytes"));
                c.frame.geometry.mac_footer_bytes =
                    static_cast<uint16_t>(rd.get_u16(geo, p + ".geometry",
                        "mac_footer_bytes"));
                c.frame.geometry.mac_fcs_bytes =
                    static_cast<uint16_t>(rd.get_u16(geo, p + ".geometry",
                        "mac_fcs_bytes"));
                c.frame.geometry.phr_bytes =
                    static_cast<uint16_t>(rd.get_u16(geo, p + ".geometry",
                        "phr_bytes"));
            }
            c.frame.mac_psdu_bytes = static_cast<uint16_t>(rd.get_u16(g, p,
                "mac_psdu_bytes"));
            c.frame.mac_psdu_includes_fcs = rd.get_bool(g, p, "mac_psdu_includes_fcs");
            c.frame.fcs_append =
                rd.get_enum<FcsAppender>(g, p, "fcs_append", fcs_appender_from_string);
            c.frame.fcs_bytes = static_cast<uint16_t>(rd.get_u16(g, p, "fcs_bytes"));
            c.frame.application_payload_bytes =
                static_cast<uint16_t>(rd.get_u16(g, p, "application_payload_bytes"));
            c.frame.sts_mode = rd.get_enum<StsMode>(g, p, "sts_mode",
                sts_mode_from_string);
            c.frame.sts_length_symbols =
                static_cast<uint16_t>(rd.get_u16(g, p, "sts_length_symbols"));
        }
    }
    {
        const std::string p = "tx";
        const json::Value* g = rd.group(p, &root);
        if (g) {
            static const char* const kKeys[] = {
                "port", "gain_db", "iq_amplitude", "calibrated_tx_power_dbm", "power_policy",
                "pulse_shaping", "vendor_power_word", "vendor_power_word_backend", nullptr
            };
            rd.reject_unknown_keys(g, p, kKeys);
            c.tx.port = static_cast<uint8_t>(rd.get_u8(g, p, "port"));
            c.tx.gain_db = rd.get_opt_double(g, p, "gain_db");
            c.tx.iq_amplitude = rd.get_opt_double(g, p, "iq_amplitude");
            c.tx.calibrated_tx_power_dbm = rd.get_opt_double(g, p,
                "calibrated_tx_power_dbm");
            c.tx.power_policy =
                rd.get_enum<TxPowerPolicy>(g, p, "power_policy",
                    tx_power_policy_from_string);
            c.tx.pulse_shaping =
                rd.get_enum<PulseShaping>(g, p, "pulse_shaping",
                    pulse_shaping_from_string);
            c.tx.vendor_power_word = rd.get_opt_u32(g, p, "vendor_power_word");
            c.tx.vendor_power_word_backend = rd.get_str(g, p,
                "vendor_power_word_backend");
        }
    }
    {
        const std::string p = "rx";
        const json::Value* g = rd.group(p, &root);
        if (g) {
            static const char* const kKeys[] = {
                "port", "gain_db", "agc", "bandwidth_hz", "detection_threshold",
                "correlation_threshold", "first_path_threshold", "first_path_index",
                "first_path_window", "vendor_pac_value", "vendor_pac_backend",
                "vendor_pac_applied_step", nullptr
            };
            rd.reject_unknown_keys(g, p, kKeys);
            c.rx.port = static_cast<uint8_t>(rd.get_u8(g, p, "port"));
            c.rx.gain_db = rd.get_opt_double(g, p, "gain_db");
            c.rx.agc = rd.get_enum<AgcMode>(g, p, "agc", agc_mode_from_string);
            c.rx.bandwidth_hz = rd.get_opt_double(g, p, "bandwidth_hz");
            c.rx.detection_threshold = rd.get_double(g, p, "detection_threshold");
            c.rx.correlation_threshold = rd.get_double(g, p, "correlation_threshold");
            c.rx.first_path_threshold = rd.get_double(g, p, "first_path_threshold");
            c.rx.first_path_index = static_cast<uint16_t>(rd.get_u16(g, p,
                "first_path_index"));
            c.rx.first_path_window = static_cast<uint16_t>(rd.get_u16(g, p,
                "first_path_window"));
            c.rx.vendor_pac_value = rd.get_opt_u32(g, p, "vendor_pac_value");
            c.rx.vendor_pac_backend = rd.get_str(g, p, "vendor_pac_backend");
            c.rx.vendor_pac_applied_step = rd.get_opt_double(g, p,
                "vendor_pac_applied_step");
        }
    }
    {
        const std::string p = "radio";
        const json::Value* g = rd.group(p, &root);
        if (g) {
            static const char* const kKeys[] = {
                "device_args", "tx_channel", "rx_channel", "native_sample_rate_hz",
                "clock_source", "time_source", "fpga_image", "dpdk_config", "peers", "readback",
                "require_readback", nullptr
            };
            rd.reject_unknown_keys(g, p, kKeys);
            c.radio.device_args = rd.get_str(g, p, "device_args");
            c.radio.tx_channel = static_cast<uint8_t>(rd.get_u8(g, p, "tx_channel"));
            c.radio.rx_channel = static_cast<uint8_t>(rd.get_u8(g, p, "rx_channel"));
            c.radio.native_sample_rate_hz = rd.get_double(g, p,
                "native_sample_rate_hz");
            c.radio.clock_source = rd.get_str(g, p, "clock_source");
            c.radio.time_source = rd.get_str(g, p, "time_source");
            c.radio.fpga_image = rd.get_str(g, p, "fpga_image");
            c.radio.dpdk_config = rd.get_str(g, p, "dpdk_config");
            const json::Value* peers = g->find("peers");
            if (!peers)
                rd.bad(p + ".peers", ConfigReason::MissingKey, "missing required key");
            else if (peers->type != json::Type::Array) {
                rd.bad(p + ".peers", ConfigReason::TypeMismatch, "expected an array");
            } else {
                for (size_t i = 0; i < peers->items.size(); ++i) {
                    const std::string pp = p + ".peers[" + twr_int_to_text(
                                                        static_cast<int64_t>(i)) + "]";
                    const json::Value& e = peers->items[i];
                    if (e.type != json::Type::Object) {
                        rd.bad(pp, ConfigReason::TypeMismatch, "expected an object");
                        continue;
                    }
                    static const char* const kPeer[] = { "id", "role", "tx_channel",
                        "rx_channel",
                                                         "native_sample_rate_hz",
                                                         "occupies_resources", nullptr };
                    rd.reject_unknown_keys(&e, pp, kPeer);
                    EndpointBinding b;
                    b.id = rd.get_str(&e, pp, "id");
                    b.role = rd.get_enum<Role>(&e, pp, "role", role_from_string);
                    b.tx_channel = static_cast<uint8_t>(rd.get_u8(&e, pp,
                        "tx_channel"));
                    b.rx_channel = static_cast<uint8_t>(rd.get_u8(&e, pp,
                        "rx_channel"));
                    b.native_sample_rate_hz = rd.get_double(&e, pp,
                        "native_sample_rate_hz");
                    b.occupies_resources = rd.get_bool(&e, pp, "occupies_resources");
                    c.radio.peers.push_back(b);
                }
            }
            const json::Value* rb = g->find("readback");
            if (!rb)
                rd.bad(p + ".readback", ConfigReason::MissingKey,
                    "missing required group");
            else if (rb->type != json::Type::Object)
                rd.bad(p + ".readback", ConfigReason::TypeMismatch,
                    "expected an object");
            else {
                const std::string rp = p + ".readback";
                static const char* const kRb[] = { "present", "sample_rate_hz",
                    "center_freq_hz",
                                                    "tx_channel", "rx_channel", "mpm_string",
                                                    "fpga_image", "uhd_version", "clock_source",
                                                    "time_source", nullptr };
                rd.reject_unknown_keys(rb, rp, kRb);
                c.radio.readback.present = rd.get_bool(rb, rp, "present");
                c.radio.readback.sample_rate_hz = rd.get_double(rb, rp,
                    "sample_rate_hz");
                c.radio.readback.center_freq_hz = rd.get_double(rb, rp,
                    "center_freq_hz");
                c.radio.readback.tx_channel = static_cast<uint8_t>(rd.get_u8(rb, rp,
                    "tx_channel"));
                c.radio.readback.rx_channel = static_cast<uint8_t>(rd.get_u8(rb, rp,
                    "rx_channel"));
                c.radio.readback.mpm_string = rd.get_str(rb, rp, "mpm_string");
                c.radio.readback.fpga_image = rd.get_str(rb, rp, "fpga_image");
                c.radio.readback.uhd_version = rd.get_str(rb, rp, "uhd_version");
                c.radio.readback.clock_source = rd.get_str(rb, rp, "clock_source");
                c.radio.readback.time_source = rd.get_str(rb, rp, "time_source");
            }
            c.radio.require_readback = rd.get_bool(g, p, "require_readback");
        }
    }
    {
        const std::string p = "timing";
        const json::Value* g = rd.group(p, &root);
        if (g) {
            static const char* const kKeys[] = { "poll_start", "poll_to_response",
                                                 "response_to_final", "final_to_report",
                                                 "post_tx_rx_enable", "min_tx_lead_time", nullptr };
            rd.reject_unknown_keys(g, p, kKeys);
            c.timing.poll_start = rd.get_timed(g, p, "poll_start");
            c.timing.poll_to_response = rd.get_timed(g, p, "poll_to_response");
            c.timing.response_to_final = rd.get_timed(g, p, "response_to_final");
            c.timing.final_to_report = rd.get_timed(g, p, "final_to_report");
            c.timing.post_tx_rx_enable = rd.get_timed(g, p, "post_tx_rx_enable");
            c.timing.min_tx_lead_time = rd.get_timed(g, p, "min_tx_lead_time");
        }
    }
    {
        const std::string p = "timeouts";
        const json::Value* g = rd.group(p, &root);
        if (g) {
            static const char* const kKeys[] = { "poll_rx_window", "response_rx_window",
                                                 "final_rx_window", "report_rx_window",
                                                 "rx_timeout", "exchange_timeout", "retry_interval",
                                                 nullptr };
            rd.reject_unknown_keys(g, p, kKeys);
            c.timeouts.poll_rx_window = rd.get_timed(g, p, "poll_rx_window");
            c.timeouts.response_rx_window = rd.get_timed(g, p, "response_rx_window");
            c.timeouts.final_rx_window = rd.get_timed(g, p, "final_rx_window");
            c.timeouts.report_rx_window = rd.get_timed(g, p, "report_rx_window");
            c.timeouts.rx_timeout = rd.get_timed(g, p, "rx_timeout");
            c.timeouts.exchange_timeout = rd.get_timed(g, p, "exchange_timeout");
            c.timeouts.retry_interval = rd.get_timed(g, p, "retry_interval");
        }
    }
    {
        const std::string p = "calibration";
        const json::Value* g = rd.group(p, &root);
        if (g) {
            static const char* const kKeys[] = {
                "tx_link_delay_ns", "rx_link_delay_ns", "antenna_delay_ns", "cable_delay_ns",
                "tx_link_delay_native_ticks", "rx_link_delay_native_ticks",
                "native_sample_rate_hz", "link_delay_unit", "first_path_algorithm",
                "cfo_compensation", "sfo_compensation", "calibration_id", "record",
                "applied_count", "calibration_required", nullptr
            };
            rd.reject_unknown_keys(g, p, kKeys);
            c.calibration.tx_link_delay = rd.get_dur(g, p, "tx_link_delay_ns");
            c.calibration.rx_link_delay = rd.get_dur(g, p, "rx_link_delay_ns");
            c.calibration.antenna_delay = rd.get_dur(g, p, "antenna_delay_ns");
            c.calibration.cable_delay = rd.get_dur(g, p, "cable_delay_ns");
            c.calibration.tx_link_delay_native_ticks =
                rd.get_i64(g, p, "tx_link_delay_native_ticks");
            c.calibration.rx_link_delay_native_ticks =
                rd.get_i64(g, p, "rx_link_delay_native_ticks");
            c.calibration.native_sample_rate_hz = rd.get_double(g, p,
                "native_sample_rate_hz");
            c.calibration.link_delay_unit =
                rd.get_enum<TimeUnit>(g, p, "link_delay_unit", time_unit_from_string);
            c.calibration.first_path_algorithm = rd.get_enum<FirstPathAlgorithm>(
                g, p, "first_path_algorithm", first_path_algorithm_from_string);
            c.calibration.cfo_compensation =
                rd.get_enum<CompensationFlag>(g, p, "cfo_compensation",
                    compensation_flag_from_string);
            c.calibration.sfo_compensation =
                rd.get_enum<CompensationFlag>(g, p, "sfo_compensation",
                    compensation_flag_from_string);
            c.calibration.calibration_id = rd.get_str(g, p, "calibration_id");
            const json::Value* rec = g->find("record");
            if (!rec)
                rd.bad(p + ".record", ConfigReason::MissingKey,
                    "missing required group");
            else if (rec->type != json::Type::Object)
                rd.bad(p + ".record", ConfigReason::TypeMismatch, "expected an object");
            else {
                const std::string rp = p + ".record";
                static const char* const kRec[] = { "calibration_id", "device_serial",
                    "channel",
                                                    "native_sample_rate_hz", "profile_version",
                                                    "gain_db", "valid_until_monotonic_ns",
                                                    nullptr };
                rd.reject_unknown_keys(rec, rp, kRec);
                c.calibration.record.calibration_id = rd.get_str(rec, rp,
                    "calibration_id");
                c.calibration.record.device_serial = rd.get_str(rec, rp,
                    "device_serial");
                c.calibration.record.channel = static_cast<uint8_t>(rd.get_u8(rec, rp,
                    "channel"));
                c.calibration.record.native_sample_rate_hz =
                    rd.get_double(rec, rp, "native_sample_rate_hz");
                c.calibration.record.profile_version = rd.get_str(rec, rp,
                    "profile_version");
                c.calibration.record.gain_db = rd.get_opt_double(rec, rp, "gain_db");
                c.calibration.record.valid_until_monotonic_ns =
                    rd.get_i64(rec, rp, "valid_until_monotonic_ns");
            }
            c.calibration.applied_count = static_cast<uint32_t>(rd.get_u32(g, p,
                "applied_count"));
            c.calibration.calibration_required = rd.get_bool(g, p,
                "calibration_required");
        }
    }
    {
        const std::string p = "diagnostics";
        const json::Value* g = rd.group(p, &root);
        if (g) {
            static const char* const kKeys[] = { "cir_capture_enabled",
                "cir_capture_max_bytes",
                                                 "cir_capture_stride", "short_iq_enabled",
                                                 "short_iq_max_bytes", "short_iq_stride",
                                                 "raw_frame_dump", "raw_frame_max_bytes",
                                                 "result_output_path", "result_queue_capacity",
                                                 "event_queue_capacity", "stats_cadence",
                                                 "io_on_realtime_thread", nullptr };
            rd.reject_unknown_keys(g, p, kKeys);
            c.diagnostics.cir_capture_enabled = rd.get_bool(g, p,
                "cir_capture_enabled");
            c.diagnostics.cir_capture_max_bytes =
                static_cast<uint64_t>(rd.get_u64(g, p, "cir_capture_max_bytes", 0xFFFFFFFFFFFFFFFFull));
            c.diagnostics.cir_capture_stride =
                static_cast<uint32_t>(rd.get_u32(g, p, "cir_capture_stride"));
            c.diagnostics.short_iq_enabled = rd.get_bool(g, p, "short_iq_enabled");
            c.diagnostics.short_iq_max_bytes =
                static_cast<uint64_t>(rd.get_u64(g, p, "short_iq_max_bytes", 0xFFFFFFFFFFFFFFFFull));
            c.diagnostics.short_iq_stride =
                static_cast<uint32_t>(rd.get_u32(g, p, "short_iq_stride"));
            c.diagnostics.raw_frame_dump = rd.get_bool(g, p, "raw_frame_dump");
            c.diagnostics.raw_frame_max_bytes =
                static_cast<uint64_t>(rd.get_u64(g, p, "raw_frame_max_bytes", 0xFFFFFFFFFFFFFFFFull));
            c.diagnostics.result_output_path = rd.get_str(g, p, "result_output_path");
            c.diagnostics.result_queue_capacity =
                static_cast<uint32_t>(rd.get_u32(g, p, "result_queue_capacity"));
            c.diagnostics.event_queue_capacity =
                static_cast<uint32_t>(rd.get_u32(g, p, "event_queue_capacity"));
            c.diagnostics.stats_cadence = rd.get_timed(g, p, "stats_cadence");
            c.diagnostics.io_on_realtime_thread = rd.get_bool(g, p,
                "io_on_realtime_thread");
        }
    }

    // Even a document that failed to import is handed back so the caller can
    // see which groups were readable; the report is the authority.
    out = c;
    return rd.report();
}

} // namespace twr
} // namespace uwb
} // namespace gr

#endif /* INCLUDED_GNURADIO_UWB_UWB_TWR_CONFIG_H */
