/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * TWR configuration utilities (Phase 2 split; aggregate entry point is
 * <openuwb/twr/twr_config.h>).  This first piece carries the original
 * prologue: the stdlib includes and the `evidence::` file-scope alias.
 */
#ifndef INCLUDED_OPENUWB_TWR_CONFIG_UTIL_H
#define INCLUDED_OPENUWB_TWR_CONFIG_UTIL_H

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
 * <openuwb/twr/twr_types.h> and the frame codec
 * <openuwb/twr/twr_frame.h>.  Both are themselves stdlib-only, and the
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


// The frame codec: this file's geometry/session authority (M0.1 / R3).  It is
// stdlib-only, so including it keeps the "no GNU Radio, no UHD" property.
#include <openuwb/twr/twr_frame.h>
#include <openuwb/twr/twr_timestamp.h>
#include <openuwb/twr/twr_types.h>
// M0.1 R7: the per-row evidence ladder.  uwb_twr_capability_evidence.h does
// NOT include this file, so the dependency is one-way.
#include <openuwb/twr/twr_capability_evidence.h>

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

} // namespace twr
} // namespace uwb
} // namespace gr

#endif /* INCLUDED_OPENUWB_TWR_CONFIG_UTIL_H */
