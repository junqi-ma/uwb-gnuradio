/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * uwb_twr_capability_evidence.h — evidence grading for TWR capability claims.
 *
 * Why this file exists
 * --------------------
 * M0 review R7 / P2: the M0 capability whitelist overstates its evidence.
 * `testdata/twr/phy_matrix_737280000.csv` has 318 rows and EVERY one of them
 * carries `path = work_direct_998p4*`: a full
 * modulate -> UwbLoopbackEcho -> demodulate -> byte-exact-FCS round trip on
 * THIS repo's 998.4 MS/s work grid.  That is a real, useful result -- and it
 * is exactly ONE level of evidence.  The 48 supported (code, SYNC, SFD)
 * combinations therefore establish
 *
 *     work_decode_verified
 *
 * and nothing else.  In particular:
 *
 *   * the `native_rate_hz` column is the rate the capability row is KEYED on
 *     (the UHD device rate the profile targets), NOT proof that a frame was
 *     ever resampled down to that grid and back;
 *   * nothing was measured for first-path / ToA accuracy, so a row may say
 *     "work decode verified" while explicitly NOT claiming ToA;
 *   * nothing was measured on hardware, and nothing was measured against a
 *     DW1000 / DW3000 module.
 *
 * This header makes those five levels explicit, monotonic and checkable, and
 * records the provenance (revision, working-tree hash, RESOLVED library path,
 * compiler, VOLK setting, test command, seed, output hash) of every run that
 * claims one.  A CSV produced against the wrong libgnuradio-uwb.so is a silent,
 * dangerous artifact, so the resolved path is part of the record.
 *
 * Rejection wording
 * -----------------
 * R7 also asks that the 128 / 256 / 512 / 1024 / 2048 rejections be worded as
 * limits of the CURRENT SOFTWARE, not as claims about the hardware.  Qorvo
 * parts DO support those lengths (DW1000 API Guide v2.7 §5.12, `txPreambLength`
 * 64/128/256/512/1024/...; DW3xxx API Guide PDF pp. 32-34, whose preamble
 * enumeration includes 128/256/512 as "non-standard preamble lengths").  What
 * fails here is THIS decoder: `uwb_demod_core.h` re-measures only the last
 * max(cfo_min_fit_repetitions, 40) SYNCs while `cfo_skip_initial_repetitions`
 * stays 24, and `mod::encode_phr19` maps the length onto the 4-value
 * preamble-duration index.  `preamble_length_reject_note()` emits that
 * distinction together with the citation, and the word "structurally
 * impossible" is not used anywhere in this header.
 *
 * Relationship to uwb_twr_config.h (owned by another agent in M0.1)
 * ---------------------------------------------------------------
 * This header deliberately does NOT include or edit `uwb_twr_config.h`.  It
 * is standalone, header-only and depends on nothing but the C++ standard
 * library, so the QA can include it without pulling in GNU Radio.  See
 * docs/twr/M0.1_证据分级与交付清单.md §4 for the exact wiring the orchestrator
 * must apply during the merge.
 *
 * Fail-closed (REQ-SCOPE-01)
 * -------------------------
 * `allows()` and `explain()` never widen a claim.  A row whose established
 * level is below the level its use requires is REJECTED with a reason naming
 * both the missing level and where it would have to be measured.  There is no
 * default, no silent fallback to the radar / QM35 profile, and no
 * "assume-it-probably-works".
 */

#ifndef INCLUDED_GNURADIO_UWB_UWB_TWR_CAPABILITY_EVIDENCE_H
#define INCLUDED_GNURADIO_UWB_UWB_TWR_CAPABILITY_EVIDENCE_H

// <cstddef> MUST come before <array>/<vector>: libstdc++'s <array> uses the
// unqualified `size_t` from <initializer_list>, which in turn needs
// <cstddef> to have been included first.  Inside the OOT build some other
// header happens to pull it in, so the omission only shows up for a consumer
// that includes THIS header first -- which is exactly what the install test
// does.  Found by the external-consumer compile, not by the in-tree build.
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include <array>

namespace gr {
namespace uwb {
namespace twr {
namespace evidence {

// ===========================================================================
// 1. The five evidence levels
// ===========================================================================

// The ladder is MONOTONIC and CUMULATIVE: establishing a higher level
// establishes every lower one, because each stage strictly contains the
// previous one as a precondition.
//
//   WorkDecodeVerified    a TWR-sized PSDU was modulated, looped back and
//                         demodulated byte-exactly on THIS repo's work grid
//                         (998.4 MS/s).  Proves TX and RX agree with each
//                         other.  Proves nothing about a radio.
//   NativeRoundtripVerified
//                         additionally: the frame actually traversed the
//                         native sample rate (work -> native -> work) with
//                         the resampler and its time mapping under test.
//                         Not established by any M0 row.
//   ToaVerified           additionally: first-path / time-of-arrival accuracy
//                         measured against an independent oracle.  Ranging
//                         accuracy is a function of exactly this quantity,
//                         which is why a decode-verified row may NOT be used
//                         to produce a range.
//   HardwareVerified      additionally: measured on real hardware, with the
//                         FPGA/UHD versions and the clock source recorded.
//   VendorInteropVerified additionally: measured against a NAMED module with
//                         its SDK / firmware hash and frame bytes recorded.
enum class Level : uint8_t {
    None = 0,
    WorkDecodeVerified = 1,
    NativeRoundtripVerified = 2,
    ToaVerified = 3,
    HardwareVerified = 4,
    VendorInteropVerified = 5
};

inline constexpr size_t kLevelCount = 6;

// Canonical spelling used in CSV cells, JSON, log lines and reason strings.
// These strings are part of the on-disk contract: do not shorten or re-case
// them without regenerating every artifact that carries them.
inline const char*
level_name(Level l)
{
    switch (l) {
    case Level::None:
        return "none";
    case Level::WorkDecodeVerified:
        return "work_decode_verified";
    case Level::NativeRoundtripVerified:
        return "native_roundtrip_verified";
    case Level::ToaVerified:
        return "toa_verified";
    case Level::HardwareVerified:
        return "hardware_verified";
    case Level::VendorInteropVerified:
        return "vendor_interop_verified";
    }
    return "invalid";
}

inline bool
level_from_name(const std::string& s, Level& out)
{
    for (size_t i = 0; i < kLevelCount; ++i) {
        const Level l = static_cast<Level>(i);
        if (s == level_name(l)) {
            out = l;
            return true;
        }
    }
    return false;
}

inline bool
level_valid(Level l)
{
    return static_cast<size_t>(l) < kLevelCount;
}

// `a` is at least as strong as `b`.
inline bool
level_implies(Level a, Level b)
{
    return static_cast<uint8_t>(a) >= static_cast<uint8_t>(b);
}

inline Level
level_min(Level a, Level b)
{
    return level_implies(a, b) ? b : a;
}

inline Level
level_max(Level a, Level b)
{
    return level_implies(a, b) ? a : b;
}

// The levels a given USE requires before it may be attempted.  A decode-only
// row is usable for offline codec work; producing a range needs ToA; putting
// it on a radio needs hardware; talking to a DW1000 needs a named module.
enum class Use : uint8_t {
    WorkDecode = 0,     // offline codec / loopback self-consistency
    NativeRoundtrip = 1,// frame must survive the native grid
    Ranging = 2,        // must yield a first path / ToA (i.e. a range)
    Hardware = 3,       // must run on a real radio
    VendorInterop = 4   // must interop with a named vendor module
};

inline constexpr size_t kUseCount = 5;

inline Level
required_level(Use u)
{
    switch (u) {
    case Use::WorkDecode:
        return Level::WorkDecodeVerified;
    case Use::NativeRoundtrip:
        return Level::NativeRoundtripVerified;
    case Use::Ranging:
        return Level::ToaVerified;
    case Use::Hardware:
        return Level::HardwareVerified;
    case Use::VendorInterop:
        return Level::VendorInteropVerified;
    }
    return Level::None;
}

inline const char*
use_name(Use u)
{
    switch (u) {
    case Use::WorkDecode:
        return "work_decode";
    case Use::NativeRoundtrip:
        return "native_roundtrip";
    case Use::Ranging:
        return "ranging";
    case Use::Hardware:
        return "hardware";
    case Use::VendorInterop:
        return "vendor_interop";
    }
    return "invalid";
}

inline bool
use_valid(Use u)
{
    return static_cast<size_t>(u) < kUseCount;
}

inline bool
allows(Level established, Use u)
{
    return level_implies(established, required_level(u));
}

// The fail-closed explanation.  `established` below the requirement is a
// REJECTION, phrased so an operator can see exactly which measurement is
// missing and where it would have to come from.
inline std::string
explain(Level established, Use u)
{
    const Level need = required_level(u);
    if (allows(established, u))
        return std::string("evidence_ok:") + level_name(established) +
               " covers use=" + use_name(u);
    return std::string("evidence_insufficient:use=") + use_name(u) +
           "_requires_" + level_name(need) + "_but_only_" +
           level_name(established) +
           "_is_established_unverified_stays_rejected_REQ_SCOPE_01";
}

// ===========================================================================
// 2. Per-row evidence record
// ===========================================================================

// Why a given level is or is not claimed.  The three sources are kept
// distinct so "we measured it", "the requirement fixes it" and "we decided
// not to look" can never be confused -- which is precisely the confusion R7
// found in the M0 CSV.
enum class SourceKind : uint8_t {
    None = 0,
    Measured = 1,   // a run in this repo produced it; `provenance_id` applies
    ByDefinition = 2,// fixed by the TWR requirement, no measurement needed
    NotMeasured = 3 // deliberately not measured; the reason says why
};

inline const char*
source_kind_name(SourceKind s)
{
    switch (s) {
    case SourceKind::None:
        return "none";
    case SourceKind::Measured:
        return "measured";
    case SourceKind::ByDefinition:
        return "by_definition";
    case SourceKind::NotMeasured:
        return "not_measured";
    }
    return "invalid";
}

inline bool
source_kind_valid(SourceKind s)
{
    return static_cast<size_t>(s) <= static_cast<size_t>(SourceKind::NotMeasured);
}

// One capability row's evidence, as written to the CSV.
//
// `established` is the HIGHEST level this row claims.  Because the ladder is
// cumulative it also implies every lower level, and
// `not_established` spells the remaining ones out in full so a reader can
// never mistake a work-decode row for a ranging or vendor claim.
struct RowEvidence {
    Level established = Level::None;
    SourceKind kind = SourceKind::None;
    std::string source;   // artifact that establishes it, e.g. the CSV + path
    std::string reason;   // why this level, or why not
    std::string provenance_id; // RunProvenance::provenance_id, when measured

    // Rejection scope, orthogonal to the evidence level.  A row can be
    // rejected because of a limit in THIS build's software (the normal case
    // for 128 / 256 / 512 / 1024 / 2048) without any statement about the
    // hardware, or because the capability is out of scope for phase 1, or
    // because it was simply never measured.
    std::string reject_scope;
    std::string reject_scope_note;
};

inline bool
row_evidence_allows(const RowEvidence& e, Use u)
{
    return e.kind == SourceKind::Measured && allows(e.established, u);
}

inline std::string
row_evidence_explain(const RowEvidence& e, Use u)
{
    if (e.kind == SourceKind::ByDefinition)
        return std::string("evidence_by_definition:use=") + use_name(u) +
               "_source=" + e.source;
    if (!row_evidence_allows(e, u))
        return explain(e.established, u);
    return std::string("evidence_ok:") + level_name(e.established) +
           "_source=" + level_name(e.established == Level::None ? Level::None
                                                                : e.established) +
           "_artifact=" + e.source + "_kind=" + source_kind_name(e.kind);
}

// The levels a row explicitly does NOT claim, ';'-separated, in ladder order.
// Written to the CSV so the absence is DATA rather than a comment.
//
// The separator is ';' and not ',' because the CSV field sanitiser maps ','
// to ';' on the way out: a comma-separated list inside a cell would be
// rewritten anyway, and the reader could not tell a list from a column break.
inline std::string
not_established_list(Level established)
{
    std::string out;
    for (size_t i = static_cast<size_t>(Level::None) + 1; i < kLevelCount; ++i) {
        const Level l = static_cast<Level>(i);
        if (level_implies(established, l))
            continue;
        if (!out.empty())
            out += ";";
        out += level_name(l);
    }
    return out; // empty when every level is established
}

// ===========================================================================
// 3. Rejection scope
// ===========================================================================

// A rejection is always attributed to something.  Naming the scope is what
// stops "our decoder can't" from hardening into "the chip can't".
inline const char*
reject_scope_current_software() { return "current_software_implementation_limit"; }
inline const char* reject_scope_out_of_scope() { return "out_of_scope_phase1"; }
inline const char* reject_scope_not_measured() { return "not_measured"; }
inline const char* reject_scope_api_contract() { return "this_build_api_contract"; }

// Vendor citation for the preamble lengths this build refuses.  Kept as data
// so the claim is greppable and can be re-verified against the PDFs.
inline const char* vendor_preamble_length_citation()
{
    return "qorvo_supports_these_lengths_see_dw1000_api_guide_v2.7_sec5.12_txPreambLength"
           "_and_dw3xxx_api_guide_pdf_pp32-34_preamble_enumeration_including_128_256_512"
           "_listed_as_non_standard_preamble_lengths";
}

// True for the lengths this build refuses for decoder/encoder reasons.  It is
// emphatically NOT a statement that the hardware lacks them.
inline bool
preamble_length_supported_by_vendor(uint16_t n)
{
    switch (n) {
    case 64:
    case 128:
    case 256:
    case 512:
    case 1024:
    case 2048:
        return true;
    default:
        return false;
    }
}

// The note carried alongside a 128/256/512/1024/2048 rejection.
inline std::string
preamble_length_reject_note(uint16_t n)
{
    std::string s = reject_scope_current_software();
    s += "_preamble_symbols_" + std::to_string(n) +
         "_refused_by_this_build_only_";
    s += (n == 1024u)
             ? "uwb_demod_core_stage_cfo_refits_only_the_last_40_reps_while"
               "_cfo_skip_initial_repetitions_stays_24_so_synthesised_zero_phase"
               "_peaks_enter_the_phase_fit_measured_error_19901_Hz_at_20_kHz_"
             : "uwb_demod_core_stage_cfo_refits_only_the_last_40_reps_while"
               "_cfo_skip_initial_repetitions_stays_24_so_synthesised_zero_phase"
               "_peaks_enter_the_phase_fit_and_mod_encode_phr19_maps_the_length"
               "_onto_the_4_value_preamble_duration_index_so_the_phr_does_not_"
               "self_describe_";
    if (preamble_length_supported_by_vendor(n))
        s += std::string("_not_a_hardware_claim_") + vendor_preamble_length_citation();
    return s;
}

// ===========================================================================
// 4. Deriving a row's evidence from a measured CSV record
// ===========================================================================
//
// The committed 318-row CSV predates the evidence columns.  Because the level
// is a pure function of fields that CSV ALREADY has (`path`, `result`,
// `whitelist`, `sync_repetitions`), the existing file can be annotated
// without re-running a single measurement -- see
// qa_uwb_twr_phy_matrix.cc::twr_phy_matrix_evidence_annotation_is_pure.
struct SourceRecord {
    double native_rate_hz = 0.0;
    std::string path;            // "work_direct_998p4", "native_...", ...
    std::string result;          // PASS | FAIL | ERROR | NOT_RUN
    std::string whitelist;       // supported | unsupported | unverified
    std::string reason;
    uint16_t sync_repetitions = 0;
};

// Every measured path in the M0 matrix goes through the 998.4 MS/s work grid.
// Naming one that would imply a native round trip must fail loudly rather
// than quietly upgrade a row.
inline bool
path_claims_native_roundtrip(const std::string& path)
{
    return path.rfind("native_", 0) == 0 ||
           path.find("native_roundtrip") != std::string::npos ||
           path.find("65_48") != std::string::npos ||
           path.find("65_32") != std::string::npos;
}

inline RowEvidence
derive_row_evidence(const SourceRecord& rec)
{
    RowEvidence e;

    // Scope of the rejection is decided first: it is independent of whether
    // anything was measured.  A deliberate scope exclusion outranks "not
    // measured", because "we decided not to look" and "we did not look" are
    // different statements and STS is the first.
    if (rec.reason.find("deliberately_excluded_REQ_SCOPE_04") !=
        std::string::npos) {
        e.reject_scope = reject_scope_out_of_scope();
        e.reject_scope_note = rec.reason;
    } else if (rec.result == "NOT_RUN" || rec.whitelist == "unverified") {
        e.reject_scope = reject_scope_not_measured();
        e.reject_scope_note = rec.reason.empty() ? "not_measured" : rec.reason;
    } else if (rec.result == "PASS" && rec.whitelist == "supported") {
        e.reject_scope.clear();
    } else if (preamble_length_supported_by_vendor(rec.sync_repetitions) &&
               rec.sync_repetitions > 64) {
        e.reject_scope = reject_scope_current_software();
        e.reject_scope_note = preamble_length_reject_note(rec.sync_repetitions);
    } else if (rec.reason.find("outside_api_range") != std::string::npos ||
               rec.reason.find("outside_api_list") != std::string::npos ||
               rec.reason.find("not_in_phy_profile_table") != std::string::npos) {
        e.reject_scope = reject_scope_api_contract();
        e.reject_scope_note = rec.reason;
    } else if (rec.result == "PASS" || rec.result == "FAIL") {
        // 1 / 2 / 4 / 8 SYNC and the rate/profile refusals: measured decoder
        // or profile limits in THIS build, not hardware claims.
        e.reject_scope = reject_scope_current_software();
        e.reject_scope_note = rec.reason;
    } else {
        e.reject_scope = reject_scope_not_measured();
        e.reject_scope_note = rec.reason;
    }

    // Then the evidence level itself.
    if (rec.result == "NOT_RUN" || rec.whitelist == "unverified") {
        e.established = Level::None;
        e.kind = SourceKind::NotMeasured;
        e.source = "none";
        e.reason = rec.reason.empty() ? "not_measured" : rec.reason;
        return e;
    }
    if (rec.result == "ERROR" || rec.whitelist != "supported") {
        e.established = Level::None;
        e.kind = SourceKind::Measured;
        e.source = "testdata/twr/phy_matrix_737280000.csv#" + rec.path;
        e.reason = rec.reason;
        return e;
    }
    // whitelist == supported.  The ONLY level the M0 sweep can establish.
    e.established = Level::WorkDecodeVerified;
    e.kind = SourceKind::Measured;
    e.source = "testdata/twr/phy_matrix_737280000.csv#" + rec.path;
    if (path_claims_native_roundtrip(rec.path)) {
        // A `native_*` path would be the only thing that could upgrade this
        // row, and no such row exists.  Refuse to guess.
        e.established = Level::None;
        e.reason = "native_path_present_but_no_native_roundtrip_was_measured";
    } else {
        e.reason = "measured_fcs_pass_byte_exact_on_work_grid_998p4_only_"
                   "native_toa_hardware_and_vendor_remain_unverified";
    }
    e.provenance_id = "see_" + std::string("phy_matrix_provenance_sidecar");
    return e;
}

// ===========================================================================
// 5. CSV column contract
// ===========================================================================

// The columns appended to testdata/twr/phy_matrix_*.csv and
// testdata/twr/phy_matrix_whitelist_*.csv.  Kept in one place so the writer
// and the annotator can never disagree about the order.
inline const std::vector<std::string>&
evidence_csv_columns()
{
    static const std::vector<std::string> kCols = {
        "evidence_level",     // highest level established (level_name)
        "evidence_source",    // artifact, or "none"
        "evidence_not_established", // ';' list, empty when all established
        "evidence_kind",      // measured | by_definition | not_measured | none
        "reject_scope",       // see section 3; empty when supported
        "reject_scope_note"   // why, incl. the vendor citation
    };
    return kCols;
}

// The evidence fields of one row, ONE ENTRY PER COLUMN, in the order of
// evidence_csv_columns().  They are returned SEPARATELY on purpose: a CSV
// writer must sanitise each field on its own and then join with commas.
// Rendering a pre-joined string and sanitising that would turn the column
// separators into whatever the sanitiser maps commas to, silently collapsing
// six columns into one.
inline std::vector<std::string>
evidence_csv_fields(const RowEvidence& e)
{
    return {
        level_name(e.established),
        e.source.empty() ? std::string("none") : e.source,
        not_established_list(e.established),
        source_kind_name(e.kind),
        e.reject_scope,
        e.reject_scope_note.empty() ? e.reason : e.reject_scope_note
    };
}

// Convenience for a writer that already has a per-field sanitiser `san`:
// returns the six sanitised fields joined by commas, ready to append.
template <typename Sanitise>
inline std::string
evidence_csv_joined(const RowEvidence& e, Sanitise san)
{
    std::string out;
    bool first = true;
    for (const auto& f : evidence_csv_fields(e)) {
        if (!first)
            out += ",";
        first = false;
        out += san(f);
    }
    return out;
}

// ===========================================================================
// 6. SHA-256 (for the output hash in the provenance record)
// ===========================================================================
//
// Self-contained so the provenance sidecar has no dependency on a crypto
// library being present in the QA's link line.  Verified against the
// FIPS-180-4 test vectors in
// qa_uwb_twr_phy_matrix.cc::twr_phy_matrix_sha256_matches_fips_vectors.

namespace detail {

inline uint32_t
rotr32(uint32_t x, unsigned n)
{
    return (x >> n) | (x << (32u - n));
}

} // namespace detail

class Sha256
{
public:
    Sha256() { reset(); }

    void reset()
    {
        d_state_[0] = 0x6a09e667u;
        d_state_[1] = 0xbb67ae85u;
        d_state_[2] = 0x3c6ef372u;
        d_state_[3] = 0xa54ff53au;
        d_state_[4] = 0x510e527fu;
        d_state_[5] = 0x9b05688cu;
        d_state_[6] = 0x1f83d9abu;
        d_state_[7] = 0x5be0cd19u;
        d_len_ = 0;
        d_buf_len_ = 0;
    }

    void update(const void* data, size_t n)
    {
        const uint8_t* p = static_cast<const uint8_t*>(data);
        d_len_ += static_cast<uint64_t>(n);
        while (n > 0) {
            const size_t take = (64 - d_buf_len_) < n ? (64 - d_buf_len_) : n;
            std::memcpy(d_buf_ + d_buf_len_, p, take);
            d_buf_len_ += take;
            p += take;
            n -= take;
            if (d_buf_len_ == 64) {
                block(d_buf_);
                d_buf_len_ = 0;
            }
        }
    }

    void update(const std::string& s) { update(s.data(), s.size()); }

    std::string hex()
    {
        // Padding: 0x80, then zeros, then the 64-bit big-endian bit length.
        const uint64_t bits = d_len_ * 8u;
        uint8_t pad = 0x80u;
        update(&pad, 1);
        pad = 0x00u;
        while (d_buf_len_ != 56) {
            update(&pad, 1);
        }
        uint8_t len_be[8];
        for (int i = 0; i < 8; ++i)
            len_be[i] = static_cast<uint8_t>((bits >> (56 - 8 * i)) & 0xffu);
        update(len_be, 8);

        static const char* hexd = "0123456789abcdef";
        std::string out;
        out.reserve(64);
        for (int i = 0; i < 8; ++i) {
            for (int b = 3; b >= 0; --b) {
                const uint8_t v = static_cast<uint8_t>((d_state_[i] >> (8 * b)) & 0xffu);
                out += hexd[(v >> 4) & 0xfu];
                out += hexd[v & 0xfu];
            }
        }
        return out;
    }

    static std::string hex_of(const std::string& s)
    {
        Sha256 h;
        h.update(s);
        return h.hex();
    }

    // Hash a whole file.  Returns "" when the file cannot be read, so a
    // missing artifact is visible instead of silently hashed as empty.
    static std::string hex_of_file(const std::string& path)
    {
        std::FILE* f = std::fopen(path.c_str(), "rb");
        if (!f)
            return std::string();
        Sha256 h;
        std::array<uint8_t, 65536> buf{};
        while (true) {
            const size_t n = std::fread(buf.data(), 1, buf.size(), f);
            if (n > 0)
                h.update(buf.data(), n);
            if (n < buf.size())
                break;
        }
        std::fclose(f);
        return h.hex();
    }

private:
    void block(const uint8_t* p)
    {
        static const uint32_t K[64] = {
            0x428a2f98u, 0x71374491u, 0xb5c0fbcfu, 0xe9b5dba5u, 0x3956c25bu,
            0x59f111f1u, 0x923f82a4u, 0xab1c5ed5u, 0xd807aa98u, 0x12835b01u,
            0x243185beu, 0x550c7dc3u, 0x72be5d74u, 0x80deb1feu, 0x9bdc06a7u,
            0xc19bf174u, 0xe49b69c1u, 0xefbe4786u, 0x0fc19dc6u, 0x240ca1ccu,
            0x2de92c6fu, 0x4a7484aau, 0x5cb0a9dcu, 0x76f988dau, 0x983e5152u,
            0xa831c66du, 0xb00327c8u, 0xbf597fc7u, 0xc6e00bf3u, 0xd5a79147u,
            0x06ca6351u, 0x14292967u, 0x27b70a85u, 0x2e1b2138u, 0x4d2c6dfcu,
            0x53380d13u, 0x650a7354u, 0x766a0abbu, 0x81c2c92eu, 0x92722c85u,
            0xa2bfe8a1u, 0xa81a664bu, 0xc24b8b70u, 0xc76c51a3u, 0xd192e819u,
            0xd6990624u, 0xf40e3585u, 0x106aa070u, 0x19a4c116u, 0x1e376c08u,
            0x2748774cu, 0x34b0bcb5u, 0x391c0cb3u, 0x4ed8aa4au, 0x5b9cca4fu,
            0x682e6ff3u, 0x748f82eeu, 0x78a5636fu, 0x84c87814u, 0x8cc70208u,
            0x90befffau, 0xa4506cebu, 0xbef9a3f7u, 0xc67178f2u
        };
        uint32_t w[64];
        for (int i = 0; i < 16; ++i) {
            w[i] = (static_cast<uint32_t>(p[4 * i]) << 24) |
                   (static_cast<uint32_t>(p[4 * i + 1]) << 16) |
                   (static_cast<uint32_t>(p[4 * i + 2]) << 8) |
                   static_cast<uint32_t>(p[4 * i + 3]);
        }
        for (int i = 16; i < 64; ++i) {
            const uint32_t s0 = detail::rotr32(w[i - 15], 7) ^
                                detail::rotr32(w[i - 15], 18) ^ (w[i - 15] >> 3);
            const uint32_t s1 = detail::rotr32(w[i - 2], 17) ^
                                detail::rotr32(w[i - 2], 19) ^ (w[i - 2] >> 10);
            w[i] = w[i - 16] + s0 + w[i - 7] + s1;
        }
        uint32_t a = d_state_[0], b = d_state_[1], c = d_state_[2],
                 d = d_state_[3], e = d_state_[4], f = d_state_[5],
                 g = d_state_[6], h = d_state_[7];
        for (int i = 0; i < 64; ++i) {
            const uint32_t S1 = detail::rotr32(e, 6) ^ detail::rotr32(e, 11) ^
                                detail::rotr32(e, 25);
            const uint32_t ch = (e & f) ^ ((~e) & g);
            const uint32_t t1 = h + S1 + ch + K[i] + w[i];
            const uint32_t S0 = detail::rotr32(a, 2) ^ detail::rotr32(a, 13) ^
                                detail::rotr32(a, 22);
            const uint32_t maj = (a & b) ^ (a & c) ^ (b & c);
            const uint32_t t2 = S0 + maj;
            h = g;
            g = f;
            f = e;
            e = d + t1;
            d = c;
            c = b;
            b = a;
            a = t1 + t2;
        }
        d_state_[0] += a;
        d_state_[1] += b;
        d_state_[2] += c;
        d_state_[3] += d;
        d_state_[4] += e;
        d_state_[5] += f;
        d_state_[6] += g;
        d_state_[7] += h;
    }

    uint32_t d_state_[8];
    uint64_t d_len_;
    uint8_t d_buf_[64];
    size_t d_buf_len_;
};

// ===========================================================================
// 7. Run provenance
// ===========================================================================

// Every claim of `Measured` evidence must name the run that produced it.  The
// fields below are the ones R7 asked for.  A CSV written against the wrong
// libgnuradio-uwb.so looks identical to a correct one unless the resolved
// library path is recorded, which is why it is not optional.
struct RunProvenance {
    std::string source_revision;   // git rev-parse HEAD
    std::string working_tree_hash; // sha256 over tracked diff + status
    std::string resolved_library;  // ABSOLUTE path of the lib actually loaded
    std::string compiler;          // __VERSION__ / compiler id
    std::string build_type;
    std::string volk_setting;      // VOLK_GENERIC, as seen by this process
    std::string test_command;      // argv + the filters that shaped the run
    std::string seed;              // "deterministic" or the seed actually used
    std::string output_sha256;     // sha256 of the artifact this run wrote
    std::string generated_by;      // source file + test case
    std::string notes;

    // Stable, short, greppable id written into every row's
    // `evidence_source` provenance slot.
    std::string provenance_id() const
    {
        std::string s = "prov-";
        s += Sha256::hex_of(source_revision + "|" + working_tree_hash + "|" +
                            resolved_library + "|" + compiler + "|" +
                            build_type + "|" + volk_setting + "|" +
                            test_command + "|" + seed)
                 .substr(0, 16);
        return s;
    }

    bool complete() const
    {
        return !source_revision.empty() && !resolved_library.empty() &&
               !compiler.empty() && !test_command.empty() && !output_sha256.empty();
    }

    std::vector<std::string> missing_fields() const
    {
        std::vector<std::string> m;
        if (source_revision.empty())
            m.push_back("source_revision");
        if (working_tree_hash.empty())
            m.push_back("working_tree_hash");
        if (resolved_library.empty())
            m.push_back("resolved_library");
        if (compiler.empty())
            m.push_back("compiler");
        if (build_type.empty())
            m.push_back("build_type");
        if (volk_setting.empty())
            m.push_back("volk_setting");
        if (test_command.empty())
            m.push_back("test_command");
        if (seed.empty())
            m.push_back("seed");
        if (output_sha256.empty())
            m.push_back("output_sha256");
        return m;
    }

    // Key/value lines, sorted in a fixed order so two runs of the same
    // configuration produce byte-identical sidecars.
    std::string to_text(const std::string& csv_name = std::string()) const
    {
        std::string s;
        s += "provenance_id=" + provenance_id() + "\n";
        s += "source_revision=" + source_revision + "\n";
        s += "working_tree_hash=" + working_tree_hash + "\n";
        s += "resolved_library=" + resolved_library + "\n";
        s += "compiler=" + compiler + "\n";
        s += "build_type=" + build_type + "\n";
        s += "volk_setting=" + volk_setting + "\n";
        s += "test_command=" + test_command + "\n";
        s += "seed=" + seed + "\n";
        s += "output_sha256=" + output_sha256 + "\n";
        s += "output_file=" + csv_name + "\n";
        s += "generated_by=" + generated_by + "\n";
        s += "notes=" + notes + "\n";
        s += "evidence_levels=" + std::string("work_decode_verified,") +
             "native_roundtrip_verified,toa_verified,hardware_verified,"
             "vendor_interop_verified\n";
        return s;
    }
};

// The sidecar's own hash must cover the payload it describes, so a truncated
// or edited sidecar is detectable.  `csv_sha256` is deliberately excluded
// from the id (it is a property of the output, not of the run).
inline std::string
provenance_sidecar_sha256(const RunProvenance& p)
{
    return Sha256::hex_of(p.to_text());
}

// ===========================================================================
// 8. Runtime capture helpers
// ===========================================================================

// The ABSOLUTE path of the libgnuradio-uwb the running process actually
// loaded, read from /proc/self/maps.  Returns "" when it cannot be
// determined -- never a guess, because a wrong "resolved_library" would make
// the provenance worse than none at all.
inline std::string
resolved_library_path()
{
    std::FILE* f = std::fopen("/proc/self/maps", "r");
    if (!f)
        return std::string();
    std::string best;
    char line[4096];
    while (std::fgets(line, sizeof(line), f)) {
        const char* hit = std::strstr(line, "libgnuradio-uwb.so");
        if (!hit)
            continue;
        // A maps line is
        //   <addr> <perms> <off> <dev> <inode> <path> [(deleted)]
        // and the PATH may itself contain spaces, so walk back to the start
        // of the field containing the hit rather than splitting on the first
        // space after it.
        const char* start = hit;
        while (start > line && *(start - 1) != '\n' && *(start - 1) != ' ')
            --start;
        std::string cand(start);
        const size_t nl = cand.find_first_of("\r\n");
        if (nl != std::string::npos)
            cand.erase(nl);
        const size_t del = cand.find(" (deleted)");
        if (del != std::string::npos)
            cand.erase(del);
        // An absolute path is the only thing worth recording; a bare SONAME
        // means the loader had nothing better, which is itself the finding.
        if (!cand.empty() && cand[0] == '/' && cand.size() > best.size())
            best = cand;
    }
    std::fclose(f);
    return best;
}

// True when `resolved` lives in `build_dir` (i.e. it is the freshly built
// library) rather than in a system prefix such as /usr/local/lib.  This is
// the check that catches a CSV generated against a stale installed library.
inline bool
library_is_from_build(const std::string& resolved, const std::string& build_dir)
{
    if (resolved.empty() || build_dir.empty())
        return false;
    if (resolved.rfind(build_dir, 0) != 0)
        return false;
    return resolved.size() > build_dir.size() &&
           resolved.compare(build_dir.size(), 5, "/lib/") == 0;
}

// A short, human-scannable statement of WHERE the library came from, so a
// reviewer can see at a glance whether a run used the build tree or a system
// install.
inline std::string
library_origin(const std::string& resolved, const std::string& build_dir)
{
    if (resolved.empty())
        return "unresolved";
    if (library_is_from_build(resolved, build_dir))
        return "build_tree";
    return "system_or_other_prefix";
}

inline std::string
env_or(const char* name, const char* fallback)
{
    const char* v = std::getenv(name);
    return (v && *v) ? std::string(v) : std::string(fallback);
}

// VOLK_GENERIC=1 is what GrTest's generated script exports; it forces VOLK to
// the generic (non-SIMD) kernels.  A throughput number is only meaningful
// next to this string, so it is recorded verbatim.
inline std::string
volk_setting()
{
    const char* v = std::getenv("VOLK_GENERIC");
    if (!v)
        return "unset";
    return std::string("VOLK_GENERIC=") + v;
}

inline std::string
compiler_string()
{
#if defined(__clang__)
    return std::string("clang ") + __clang_version__;
#elif defined(__GNUC__)
    return std::string("gcc ") + __VERSION__;
#else
    return "unknown";
#endif
}

// Run a command and capture stdout, trimmed.  Returns "" on any failure.
inline std::string
capture_command(const std::string& cmd)
{
    std::FILE* p = popen((cmd + " 2>/dev/null").c_str(), "r");
    if (!p)
        return std::string();
    std::string out;
    char buf[512];
    while (std::fgets(buf, sizeof(buf), p))
        out += buf;
    const int rc = pclose(p);
    if (rc != 0)
        return std::string();
    while (!out.empty() && (out.back() == '\n' || out.back() == '\r'))
        out.pop_back();
    return out;
}

// The source revision and a hash of the working tree.  `repo_dir` is the
// checkout root.  Deliberately best-effort: outside a git checkout both come
// back empty and `RunProvenance::complete()` reports it, rather than the run
// inventing a revision.
inline void
capture_git(const std::string& repo_dir, std::string& revision, std::string& tree)
{
    revision.clear();
    tree.clear();
    if (repo_dir.empty())
        return;
    revision = capture_command("git -C '" + repo_dir + "' rev-parse HEAD");
    if (revision.empty())
        return;
    const std::string status = capture_command("git -C '" + repo_dir +
                                              "' status --porcelain");
    const std::string diff = capture_command("git -C '" + repo_dir +
                                            "' diff HEAD -- .");
    Sha256 h;
    h.update(revision);
    h.update("\n");
    h.update(status);
    h.update("\n");
    h.update(diff);
    tree = h.hex();
}

} // namespace evidence
} // namespace twr
} // namespace uwb
} // namespace gr

#endif /* INCLUDED_GNURADIO_UWB_UWB_TWR_CAPABILITY_EVIDENCE_H */
