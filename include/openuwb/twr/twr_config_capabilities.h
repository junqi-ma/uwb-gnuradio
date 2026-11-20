/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * TWR configuration, split by responsibility (Phase 2).
 * The aggregate entry point is <openuwb/twr/twr_config.h>; these pieces are
 * ordered, not independent: each one includes its predecessor.
 */
#ifndef INCLUDED_OPENUWB_TWR_CONFIG_CAPABILITIES_H
#define INCLUDED_OPENUWB_TWR_CONFIG_CAPABILITIES_H

#include <openuwb/twr/twr_config_timed.h>
namespace gr {
namespace uwb {
namespace twr {

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

} // namespace twr
} // namespace uwb
} // namespace gr

#endif /* INCLUDED_OPENUWB_TWR_CONFIG_CAPABILITIES_H */
