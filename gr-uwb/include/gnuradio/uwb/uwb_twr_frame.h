/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * UWB SS-TWR / DS-TWR MAC PSDU frame codec (milestone M0, phase 1).
 *
 * WHAT THIS IS
 * ------------
 * A dependency-free (no GNU Radio, no UHD, no volk, no pmt) encode/decode of
 * the versioned TWR MAC PSDU.  It knows nothing about frames, radiOs, streams
 * or schedulers: it turns a `Frame` into bytes and bytes into a `Frame`, and
 * refuses anything it cannot represent without ambiguity.  M1's FSM, the
 * Python schema and the CLI all sit on top of this file.
 *
 * WIRE FORMAT (frozen, little-endian throughout)
 * ----------------------------------------------
 *   off  size  field        notes
 *     0    1    version     0x01
 *     1    1    function    Poll 0x00 / Response 0x01 / Final 0x02
 *                           Report 0x03 is RESERVED (see below)
 *     2    2    session_id  u16, wraps independently of `seq`.  The local ->
 *                           wire mapping is identity-or-refuse: see
 *                           `session_id_to_wire()` and the collision note.
 *     4    2    seq         u16
 *     6    2    pan_id      u16
 *     8    2    src_addr    u16
 *    10    2    dst_addr    u16
 *    12    2    flags       bit0 ranging, bit1 sts_present, bits 2..15
 *                           reserved and MUST be zero
 *    14   5*n  timestamps  40-bit little-endian by default, n per
 *                           function code: Poll 0, Response 2, Final 3
 *
 *   MAC payload size: Poll 14 B, Response 24 B, Final 29 B.
 *
 * Timestamp payloads follow the Decawave three-message asymmetric DS-TWR
 * exchange (REQ-PROTO-03) and the SS formula of REQ-PROTO-02:
 *   Poll     carries no timestamps.  t1A is local to the initiator.
 *   Response carries t2B, t3B -- the responder's RX and TX instants.  The
 *            initiator needs them for  ToF_A = (RA - kAB*DB)/2  with
 *            RA = t4A - t1A and DB = t3B - t2B.
 *   Final    carries t1A, t4A, t5A -- the initiator's TX-Poll, RX-Response
 *            and TX-Final instants.  The responder adds its own local
 *            t2B, t3B, t6B to obtain RA, DA, RB, DB and evaluates
 *            ToF = (RA*RB - DA*DB) / (RA + RB + DA + DB).
 *            Final deliberately does NOT echo t2B/t3B: that matches the
 *            Decawave reference exchange and keeps the frame at 29 B.
 *
 * FCS IS NOT PART OF THIS CODEC
 * -----------------------------
 * REQ-API-01 ("分清应用 payload、MAC PSDU 和是否包含 FCS；只由一层追加 FCS"):
 * exactly one layer appends the FCS, and it is the HRP modulation layer.
 * `encode()` returns the MAC payload only (14 / 24 / 29 B).  The single FCS
 * producer is `UwbHrpPacketSource` constructed with `append_fcs = true`,
 * which calls `mod::append_ieee_fcs()` on the PSDU it is given, producing
 * 16 / 26 / 31 B.  Consequently `decode()` is strict about its input length:
 * a buffer that still carries the two FCS bytes is REJECTED
 * (`FrameError::LengthMismatch`) rather than silently reinterpreted, so a
 * double-FCS or FCS-stripped-at-the-wrong-layer bug is caught at the codec
 * boundary and reported as a failed measurement.
 *
 * REJECTION, NOT SILENT DEFAULTS (REQ-PROTO-01, REQ-PROTO-06, REQ-TIME-04)
 * -----------------------------------------------------------------------
 * - `Report` (0x03) is refused with `FrameError::NotImplemented`.  Phase 1
 *   emits no fourth message: the off-the-shelf three-message reference
 *   firmware does not emit one, and adding one would invalidate the interop
 *   claim of REQ-PROTO-05.
 * - An unknown `version`, an unknown/reserved `function_code`, or any of the
 *   reserved flag bits 2..15 being set is refused.  A frame whose meaning
 *   depends on an unknown extension is not a frame this codec may measure
 *   with.
 * - `sts_present` is refused with `FrameError::StsUnsupported`.  STS is out
 *   of scope; an explicit failure is mandatory, a silent ignore is not.
 * - Timestamp width and tick rate are PROFILE parameters, never a
 *   project-wide constant (REQ-TIME-04 forbids one global multiplier mixing
 *   DW ticks, UUS and USRP ticks).  A value that does not fit the declared
 *   field is refused by `encode()`; a profile whose width the codec cannot
 *   map onto whole bytes is refused by `frame_profile_validate()`.
 *
 * ALLOCATION POLICY
 * -----------------
 * `Frame` is a fixed-capacity value type: no vector, no string, no heap.
 * `encode_into()` writes into caller-provided storage and only ever writes to
 * the caller's `std::string& error`, so it is usable from an emit handler /
 * delayed-TX path that must not allocate (AGENTS.md dev rule 4).  The
 * `encode()` overload taking a `std::vector<uint8_t>&` exists for setup,
 * logging and tests; `out` is resized once to the exact frame length.
 * `FrameScratch` bundles the 38-byte worst-case buffer.
 *
 * GEOMETRY AUTHORITY (M0.1, fixes R3)
 * -----------------------------------
 * A frame's length is computed in exactly ONE function, `frame_length_for()`.
 * Everything else that needs a size -- `psdu_length_for()`, `FrameProfileGeometry`,
 * the configuration layer, any budget calculation -- delegates to it, and this
 * header contains no second `header + n*timestamp` sum.  `FrameProfileGeometry`
 * is the authority for every byte count of the frozen layout, and
 * `frame_geometry_check()` compares a caller's claim against it field by
 * field, so a configuration cannot assert a 7-byte header and a 9/19/24-byte
 * budget while the codec emits 14 bytes and 16/26/31.  The frame profile is
 * an enumeration (`FrameProfileId`), never a free-form name, and the local ->
 * wire session id mapping is identity-or-refuse -- see the sections below.
 *
 * Requirements traceability (docs/twr/需求_UWB_SS_DS_TWR.md):
 *   REQ-API-01  one single FCS layer, application payload vs MAC PSDU split
 *   REQ-PROTO-01 only a matching frame may advance the FSM (frame_match)
 *   REQ-PROTO-02 Response carries the responder's t2B/t3B
 *   REQ-PROTO-03 three-message asymmetric DS-TWR, Final carries t1A/t4A/t5A
 *   REQ-PROTO-05 no Report message in phase 1
 *   REQ-PROTO-06 independent frame codec, explicit FCF/PAN/addr/seq/FCF/
 *               byte order/timestamp width+unit/FCS scope
 *   REQ-TIME-04 timestamp width and unit are per-profile
 *   REQ-QA-02   byte order, malformed length, sequence/address/type matching
 *               are all covered by qa_uwb_twr_frame.cc
 */

#ifndef INCLUDED_GNURADIO_UWB_UWB_TWR_FRAME_H
#define INCLUDED_GNURADIO_UWB_UWB_TWR_FRAME_H

#include <gnuradio/uwb/uwb_twr_types.h>

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

namespace gr::uwb::twr {

// ---------------------------------------------------------------------------
// Frozen layout constants
// ---------------------------------------------------------------------------

// Wire version.  A frame carrying anything else is rejected, so this value is
// the compatibility switch for the whole layout (REQ-PROTO-06).
inline constexpr uint8_t kFrameVersion = 0x01;

// Byte offset of every header field.  These ARE the specification; the QA
// asserts the byte at each one of them.
inline constexpr size_t kOffVersion = 0;
inline constexpr size_t kOffFunctionCode = 1;
inline constexpr size_t kOffSessionId = 2;
inline constexpr size_t kOffSeq = 4;
inline constexpr size_t kOffPanId = 6;
inline constexpr size_t kOffSrcAddr = 8;
inline constexpr size_t kOffDstAddr = 10;
inline constexpr size_t kOffFlags = 12;
inline constexpr size_t kOffTimestamps = 14;

// The header is exactly everything ahead of the first timestamp, so its size IS
// the first timestamp's offset: one definition, not two that can drift apart
// (a moved offset with a stale size would silently corrupt every frame length).
inline constexpr size_t kFrameHeaderBytes = kOffTimestamps;

// Max timestamps any frame type carries (Final).  Fixed capacity, no heap.
inline constexpr size_t kMaxTimestamps = 3;

// Worst-case encoded size with the widest supported timestamp field (64 bit):
// 14 + 3*8.
inline constexpr size_t kMaxFrameBytes =
    kFrameHeaderBytes + kMaxTimestamps * 8u;

// IEEE 802.15.4 FCS size.  NOT written by this codec -- see the FCS section
// at the top of this file.  Only the modulation layer appends it.
inline constexpr size_t kFrameFcsBytes = 2;

// PHY limit for one PSDU, mirroring gr::uwb::radar_meta::kMaxPsduBytes
// (uwb_radar_pdu_meta.h).  Duplicated rather than included because this
// header must stay GNU-Radio-free; qa_uwb_twr_frame.cc static_asserts that
// the two constants still agree, so the mirror cannot silently rot.
inline constexpr size_t kPhyMaxPsduBytes = 127;

// flags bit assignments.  bits 2..15 are reserved and must be zero.
inline constexpr uint16_t kFlagRanging = 0x0001;
inline constexpr uint16_t kFlagStsPresent = 0x0002;
inline constexpr uint16_t kFlagReservedMask = 0xFFFC;

inline bool flags_ranging(uint16_t flags) { return (flags & kFlagRanging) != 0; }
inline bool flags_sts_present(uint16_t flags) { return (flags & kFlagStsPresent) != 0; }
inline bool flags_has_reserved_bits(uint16_t flags)
{
    return (flags & kFlagReservedMask) != 0;
}
inline uint16_t make_flags(bool ranging, bool sts_present = false)
{
    return static_cast<uint16_t>((ranging ? kFlagRanging : 0u) |
                                 (sts_present ? kFlagStsPresent : 0u));
}

// ---------------------------------------------------------------------------
// Timestamp fields
// ---------------------------------------------------------------------------

// The six protocol instants the exchange needs.  Naming them instead of
// indexing an array is what makes a wrong field order hard to write: a
// `Frame` only ever exposes the fields its own function code carries, and
// `get()`/`set()` say so with a bool.
enum class TimestampField : uint8_t {
    T1A = 0, // Final    : initiator TX of Poll       (REMarkerTx on A)
    T2B = 1, // Response : responder RX of Poll      (RmarkerRx on B)
    T3B = 2, // Response : responder TX of Response  (RmarkerTx on B)
    T4A = 3, // Final    : initiator RX of Response (RmarkerRx on A)
    T5A = 4, // Final    : initiator TX of Final     (RmarkerTx on A)
    Count = 5
};
// t6B (responder RX of Final) is local to the responder and is never on the
// wire.  It is listed here only to make the omission explicit.

inline const char* timestamp_field_name(TimestampField f)
{
    switch (f) {
    case TimestampField::T1A:
        return "t1A";
    case TimestampField::T2B:
        return "t2B";
    case TimestampField::T3B:
        return "t3B";
    case TimestampField::T4A:
        return "t4A";
    case TimestampField::T5A:
        return "t5A";
    case TimestampField::Count:
        break;
    }
    return "invalid";
}

// Wire order of the timestamp block for one function code.  This table is the
// single source of truth for both encode order and the accessors.
struct FrameLayout {
    FrameType type;
    size_t timestamp_count;              // == frame_type_timestamp_count(type)
    TimestampField order[kMaxTimestamps]; // wire order; unused slots are
                                           // TimestampField::Count
};

inline const FrameLayout* frame_layout(FrameType t)
{
    static const FrameLayout kPoll = { FrameType::Poll, 0,
                                       { TimestampField::Count,
                                         TimestampField::Count,
                                         TimestampField::Count } };
    static const FrameLayout kResponse = { FrameType::Response, 2,
                                           { TimestampField::T2B,
                                             TimestampField::T3B,
                                             TimestampField::Count } };
    static const FrameLayout kFinal = { FrameType::Final, 3,
                                        { TimestampField::T1A,
                                          TimestampField::T4A,
                                          TimestampField::T5A } };
    // Report is reserved: it carries no defined layout in phase 1 and is
    // rejected by the codec.  A null layout is the signal for that.
    switch (t) {
    case FrameType::Poll:
        return &kPoll;
    case FrameType::Response:
        return &kResponse;
    case FrameType::Final:
        return &kFinal;
    case FrameType::Report:
        return nullptr;
    }
    return nullptr;
}

// True for the three frame types phase 1 actually puts on the air.
inline bool frame_type_is_implemented(FrameType t)
{
    return frame_layout(t) != nullptr;
}

// Decodes a wire function code.  `Report` (0x03) IS a known code point and
// parses successfully here; the codec -- not this function -- is what refuses
// to process it.
inline bool frame_type_from_wire(uint8_t wire, FrameType& out)
{
    switch (wire) {
    case 0x00:
        out = FrameType::Poll;
        return true;
    case 0x01:
        out = FrameType::Response;
        return true;
    case 0x02:
        out = FrameType::Final;
        return true;
    case 0x03:
        out = FrameType::Report;
        return true;
    default:
        return false;
    }
}

// ---------------------------------------------------------------------------
// Frame profile
// ---------------------------------------------------------------------------

// Everything about the encoding that is NOT frozen across all TWR
// deployments.  REQ-TIME-04: the timestamp width and its unit belong to a
// versioned adapter/profile, never to a project-wide constant, because DW
// ticks (~63.8976 GHz), UUS and USRP device ticks must never be mixed behind
// one multiplier.
struct FrameProfile {
    uint8_t version = kFrameVersion;
    // Width of ONE timestamp field in bits.  Must be a whole number of bytes
    // in [8, 64]; 40 is the phase-1 default.  A width that is not a whole
    // number of bytes would need an explicit bit order inside a byte, which
    // this little-endian layout does not define, so such a profile is
    // rejected rather than guessed at.
    uint8_t timestamp_bits = 40;
    // Nominal ticks per second of one timestamp tick.  Phase 1 default is the
    // X410 device tick rate.  Recorded so a result can state the unit its
    // timestamps were in; never used to convert inside the codec.
    double timestamp_unit_hz = 737.28e6;
    // Upper bound on one PSDU for this profile.  May be tightened below the
    // PHY limit, never raised above it.
    size_t max_psdu_bytes = kPhyMaxPsduBytes;
    // Declaration only: the codec itself never appends or strips an FCS.
    // See the FCS section at the top of this file.
    bool fcs_appended_by_modulation_layer = true;
};

inline size_t timestamp_bytes(const FrameProfile& p)
{
    return static_cast<size_t>(p.timestamp_bits) / 8u;
}

// Largest value representable in one timestamp field of this profile.
inline uint64_t timestamp_max_value(const FrameProfile& p)
{
    const unsigned bits = p.timestamp_bits;
    if (bits == 0)
        return 0;
    if (bits >= 64)
        return ~static_cast<uint64_t>(0);
    return (static_cast<uint64_t>(1) << bits) - 1u;
}

inline bool timestamp_fits(const FrameProfile& p, uint64_t v)
{
    return v <= timestamp_max_value(p);
}

// ---------------------------------------------------------------------------
// Error taxonomy
// ---------------------------------------------------------------------------

// The codec's own error set.  M1 maps these onto `ExchangeStatus`; the mapping
// is provided so the FSM does not have to parse strings.
enum class FrameError : uint8_t {
    None = 0,
    BadProfile,                // the profile itself is not usable
    BadVersion,                // unknown frame version
    BadFunctionCode,           // unknown / reserved function code
    NotImplemented,            // known code point refused in phase 1 (Report)
    ReservedFlags,             // flags bits 2..15 set
    StsUnsupported,            // sts_present set; STS is out of scope
    TimestampOutOfRange,       // value does not fit the declared field width
    ReservedTimestampNonZero,  // unused timestamp slot is not zero
    BufferTooSmall,            // caller storage cannot hold the frame
    NullPointer,               // null data pointer with non-zero length
    Truncated,                 // fewer bytes than the function code requires
    LengthMismatch,            // more bytes than required (e.g. FCS not stripped)
    PsduTooLarge               // would exceed the profile's max PSDU
};

inline const char* frame_error_to_string(FrameError e)
{
    switch (e) {
    case FrameError::None:
        return "none";
    case FrameError::BadProfile:
        return "bad_profile";
    case FrameError::BadVersion:
        return "bad_version";
    case FrameError::BadFunctionCode:
        return "bad_function_code";
    case FrameError::NotImplemented:
        return "not_implemented";
    case FrameError::ReservedFlags:
        return "reserved_flags_set";
    case FrameError::StsUnsupported:
        return "sts_unsupported";
    case FrameError::TimestampOutOfRange:
        return "timestamp_out_of_range";
    case FrameError::ReservedTimestampNonZero:
        return "reserved_timestamp_nonzero";
    case FrameError::BufferTooSmall:
        return "buffer_too_small";
    case FrameError::NullPointer:
        return "null_pointer";
    case FrameError::Truncated:
        return "truncated";
    case FrameError::LengthMismatch:
        return "length_mismatch";
    case FrameError::PsduTooLarge:
        return "psdu_too_large";
    }
    return "invalid";
}

inline ExchangeStatus frame_error_to_exchange_status(FrameError e)
{
    switch (e) {
    case FrameError::None:
        return ExchangeStatus::Ok;
    case FrameError::BadProfile:
    case FrameError::PsduTooLarge:
    case FrameError::BufferTooSmall:
        return ExchangeStatus::ConfigRejected;
    case FrameError::NotImplemented:
    case FrameError::StsUnsupported:
        return ExchangeStatus::Unsupported;
    case FrameError::BadFunctionCode:
        return ExchangeStatus::PhyDecodeFailed;
    case FrameError::BadVersion:
    case FrameError::ReservedFlags:
    case FrameError::TimestampOutOfRange:
    case FrameError::ReservedTimestampNonZero:
        return ExchangeStatus::WrongPeer;
    case FrameError::NullPointer:
    case FrameError::Truncated:
    case FrameError::LengthMismatch:
        return ExchangeStatus::PhyDecodeFailed;
    }
    return ExchangeStatus::InternalError;
}

// ---------------------------------------------------------------------------
// Frame
// ---------------------------------------------------------------------------

// A decoded / to-be-encoded frame.  Trivially copyable and heap-free so it
// can live in a bounded ring, be snapshotted into a non-retrying exchange
// (REQ-API-03: an in-flight exchange uses an immutable snapshot), and be
// copied into an event record with no allocator traffic.
struct Frame {
    uint8_t version = kFrameVersion;
    FrameType function_code = FrameType::Poll;
    uint16_t session_id = 0;
    uint16_t seq = 0;
    uint16_t pan_id = 0;
    uint16_t src_addr = 0;
    uint16_t dst_addr = 0;
    uint16_t flags = 0;
    // Wire order; see frame_layout().  Entries at or beyond
    // frame_type_timestamp_count(function_code) are ignored on read and must
    // be zero on encode.
    uint64_t timestamps[kMaxTimestamps] = { 0, 0, 0 };

    size_t timestamp_count() const
    {
        return frame_type_timestamp_count(function_code);
    }

    bool carries(TimestampField f) const
    {
        const FrameLayout* l = frame_layout(function_code);
        if (l == nullptr)
            return false;
        for (size_t i = 0; i < l->timestamp_count; ++i) {
            if (l->order[i] == f)
                return true;
        }
        return false;
    }

    // Wire index of a field in THIS frame, or -1 when it is not carried.
    int index_of(TimestampField f) const
    {
        const FrameLayout* l = frame_layout(function_code);
        if (l == nullptr)
            return -1;
        for (size_t i = 0; i < l->timestamp_count; ++i) {
            if (l->order[i] == f)
                return static_cast<int>(i);
        }
        return -1;
    }

    // Checked read.  Returns false and leaves `out` untouched when this frame
    // does not carry the field -- the caller must not substitute 0 for a
    // measurement instant.
    bool get(TimestampField f, uint64_t& out) const
    {
        const int i = index_of(f);
        if (i < 0)
            return false;
        out = timestamps[static_cast<size_t>(i)];
        return true;
    }

    // Checked write.  Returns false when this frame does not carry the field.
    bool set(TimestampField f, uint64_t v)
    {
        const int i = index_of(f);
        if (i < 0)
            return false;
        timestamps[static_cast<size_t>(i)] = v;
        return true;
    }

    // Convenience readers.  These return 0 for a frame that does not carry
    // the field, so reading t4A() off a Poll yields 0 rather than some other
    // frame's instant.  Code that acts on a measurement instant should
    // prefer get()/has() and reject the frame explicitly.
    uint64_t t1A() const
    {
        uint64_t v = 0;
        get(TimestampField::T1A, v);
        return v;
    }
    uint64_t t2B() const
    {
        uint64_t v = 0;
        get(TimestampField::T2B, v);
        return v;
    }
    uint64_t t3B() const
    {
        uint64_t v = 0;
        get(TimestampField::T3B, v);
        return v;
    }
    uint64_t t4A() const
    {
        uint64_t v = 0;
        get(TimestampField::T4A, v);
        return v;
    }
    uint64_t t5A() const
    {
        uint64_t v = 0;
        get(TimestampField::T5A, v);
        return v;
    }

    bool has_t1A() const { return carries(TimestampField::T1A); }
    bool has_t2B() const { return carries(TimestampField::T2B); }
    bool has_t3B() const { return carries(TimestampField::T3B); }
    bool has_t4A() const { return carries(TimestampField::T4A); }
    bool has_t5A() const { return carries(TimestampField::T5A); }

    bool ranging() const { return flags_ranging(flags); }
    bool sts_present() const { return flags_sts_present(flags); }
};

inline bool operator==(const Frame& a, const Frame& b)
{
    if (a.version != b.version || a.function_code != b.function_code ||
        a.session_id != b.session_id || a.seq != b.seq || a.pan_id != b.pan_id ||
        a.src_addr != b.src_addr || a.dst_addr != b.dst_addr ||
        a.flags != b.flags) {
        return false;
    }
    const size_t n = a.timestamp_count();
    for (size_t i = 0; i < n; ++i) {
        if (a.timestamps[i] != b.timestamps[i])
            return false;
    }
    return true;
}

inline bool operator!=(const Frame& a, const Frame& b) { return !(a == b); }

// ---------------------------------------------------------------------------
// Caller-provided storage for the no-allocation path
// ---------------------------------------------------------------------------

struct FrameScratch {
    uint8_t bytes[kMaxFrameBytes] = {};
    static constexpr size_t capacity = kMaxFrameBytes;
};

// ---------------------------------------------------------------------------
// Validation
// ---------------------------------------------------------------------------

inline void set_error(std::string& error, FrameError* code, FrameError c, const char* detail)
{
    if (code != nullptr)
        *code = c;
    error = frame_error_to_string(c);
    if (detail != nullptr && detail[0] != '\0') {
        error += ": ";
        error += detail;
    }
}

// Rejects a profile the codec cannot map without ambiguity.  Callers may also
// call this before `create_session` (REQ-API-01: every entry point uses the
// same validator and rejects before the radio starts).
inline bool frame_profile_validate(const FrameProfile& p, std::string& error, FrameError* code = nullptr)
{
    if (p.version != kFrameVersion) {
        set_error(error, code, FrameError::BadProfile, "profile version is not 0x01");
        return false;
    }
    if (p.timestamp_bits == 0 || p.timestamp_bits > 64 || (p.timestamp_bits % 8u) != 0) {
        set_error(error, code, FrameError::BadProfile,
                  "timestamp_bits must be a multiple of 8 in [8, 64]");
        return false;
    }
    // A NaN tick rate fails the `> 0.0` test, so no <cmath> is needed.
    if (!(p.timestamp_unit_hz > 0.0) || p.timestamp_unit_hz > 1.0e15) {
        set_error(error, code, FrameError::BadProfile,
                  "timestamp_unit_hz must be finite and in (0, 1e15]");
        return false;
    }
    if (p.max_psdu_bytes < kFrameHeaderBytes || p.max_psdu_bytes > kPhyMaxPsduBytes) {
        set_error(error, code, FrameError::BadProfile,
                  "max_psdu_bytes must be in [14, 127]");
        return false;
    }
    if (!p.fcs_appended_by_modulation_layer) {
        set_error(error, code, FrameError::BadProfile,
                  "this codec never emits an FCS; it is appended by the HRP "
                  "modulation layer (UwbHrpPacketSource append_fcs=true)");
        return false;
    }
    if (code != nullptr)
        *code = FrameError::None;
    error.clear();
    return true;
}

// Frame sizes.  `frame_length_for` is the MAC payload size (no FCS):
// 14 / 24 / 29 B for the default 40-bit profile.  `psdu_length_for` adds the
// two FCS bytes the modulation layer will append.
inline size_t frame_length_for(FrameType t, const FrameProfile& p = FrameProfile())
{
    return kFrameHeaderBytes +
           static_cast<size_t>(frame_type_timestamp_count(t)) * timestamp_bytes(p);
}

inline size_t psdu_length_for(FrameType t, const FrameProfile& p = FrameProfile())
{
    return frame_length_for(t, p) + kFrameFcsBytes;
}

// Size-checked form.  Returns 0 and reports the reason for a frame type the
// codec refuses or a profile it cannot use, so a caller cannot schedule a
// frame that can never be built.
inline size_t frame_length_for_checked(FrameType t,
                                       const FrameProfile& p,
                                       std::string& error,
                                       FrameError* code = nullptr)
{
    if (!frame_profile_validate(p, error, code))
        return 0;
    if (!frame_type_is_implemented(t)) {
        set_error(error, code, FrameError::NotImplemented,
                  "report frame type is reserved and not emitted in phase 1");
        return 0;
    }
    const size_t n = frame_length_for(t, p);
    if (n + kFrameFcsBytes > p.max_psdu_bytes) {
        set_error(error, code, FrameError::PsduTooLarge,
                  "frame plus FCS exceeds the profile max PSDU");
        return 0;
    }
    if (code != nullptr)
        *code = FrameError::None;
    error.clear();
    return n;
}

// ===========================================================================
// FRAME GEOMETRY AUTHORITY  (M0.1 / R3)
//
// WHY THIS SECTION EXISTS
// -----------------------
// The M0 review (docs/twr/M0_复核报告.md R3) found TWO geometries for one
// format.  The configuration schema carried a pre-codec "7-byte header"
// FrameGeometry (FCF(2)+seq(1)+PAN(2)+addr(2)) with 9/19/24-byte budgets,
// while this codec has emitted a 14-byte header and 16/26/31-byte on-air
// frames since it was frozen.  Both test suites passed, because nothing ever
// asked whether an ACCEPTED configuration can losslessly build the real
// frame.  A wrong 7-byte claim does not merely describe a slightly different
// budget: it understates the buffer and frame-duration budget, and a session
// id narrowed from 32 to 16 bits collides with a probability no test stated.
//
// THE CONTRACT RESTATED HERE
// -------------------------
//   * This file is the ONLY authority for frame v1's byte geometry.  The
//     configuration layer does not recompute a single one of these numbers;
//     it asks, and `frame_geometry_check()` proves the two agree.
//   * A frame's length is computed in exactly one function, `frame_length_for()`.
//     Every other size in this header, in the geometry authority, and in any
//     caller is a delegation to it.  There is no second `header + n*ts` sum.
//   * A frame profile is a NAMED, IMPLEMENTED layout taken from an
//     enumeration.  A free-form profile string is never accepted as an
//     executable geometry.
//   * The local -> wire session id mapping is identity-or-refuse.  A lossy
//     mapping would fabricate session collisions silently, so it does not
//     exist.
//
// It is deliberately NOT a config/JSON/dictionary type: the configuration
// layer builds a `FrameGeometryClaim` from whatever it has (schema, JSON,
// a test fixture) and checks the claim against the authority.  Three parallel
// geometry types would be the defect, not the fix.
// ===========================================================================

// ---------------------------------------------------------------------------
// Profile identity
// ---------------------------------------------------------------------------

// A frame profile is a versioned MAC PSDU layout.  Phase 1 implements exactly
// one.  Selecting it by enum is what makes "an arbitrary non-empty profile
// name" unrepresentable instead of merely discouraged (R3).
//
// NOT the same concept as the PHY capability profile string
// ("m0-ch5-64sync-4z2" in capabilities()), which names channel / SYNC length /
// SFD, not the MAC PSDU layout.  The two must not be conflated or derived
// from one another.
enum class FrameProfileId : uint8_t {
    // frame v1: 14 B header (version, function code, 16-bit session/seq/PAN/
    // src/dst/flags), 40-bit little-endian timestamps, standard HRP PHR,
    // FCS appended by the modulation layer.  Frozen; see the wire-format
    // table at the top of this file.
    TwrV1 = 0
};

inline const char* frame_profile_id_to_string(FrameProfileId id)
{
    switch (id) {
    case FrameProfileId::TwrV1:
        return "frame_v1";
    }
    return "invalid";
}

// Strict: exactly one accepted spelling, and nothing else.  An unknown name is
// a rejection with a reason, never a default to whatever happens to exist.
inline bool frame_profile_id_from_string(const std::string& name, FrameProfileId& out)
{
    if (name == "frame_v1") {
        out = FrameProfileId::TwrV1;
        return true;
    }
    return false;
}

// True only for an implemented layout.  A value cast in from an int, or a
// future enumerator added without a case here, is refused.
inline bool frame_profile_id_is_supported(FrameProfileId id)
{
    return id == FrameProfileId::TwrV1;
}

// The complete set, so a validator can enumerate what it may offer.
inline constexpr FrameProfileId kAllFrameProfileIds[] = { FrameProfileId::TwrV1 };
inline constexpr size_t kFrameProfileIdCount =
    sizeof(kAllFrameProfileIds) / sizeof(kAllFrameProfileIds[0]);

// The `FrameProfile` of a profile id, or false when the id names no
// implemented layout.  A default-constructed `FrameProfile` IS frame v1, so
// selecting the profile explicitly does not change a single byte on the air;
// it only makes the choice checkable.
inline bool frame_profile_for(FrameProfileId id, FrameProfile& out)
{
    if (!frame_profile_id_is_supported(id))
        return false;
    out = FrameProfile();
    return true;
}

// Who appends the FCS.  Exactly one layer may (REQ-API-01).  Distinct from
// the configuration layer's own FcsAppender enumeration, which this header
// does not depend on; a configuration maps onto this one.
enum class FcsOwner : uint8_t {
    PhyLayer = 0, // the modulation layer appends it (UwbHrpPacketSource)
    MacLayer = 1  // a MAC that builds and appends the CRC itself
};

inline const char* fcs_owner_to_string(FcsOwner o)
{
    switch (o) {
    case FcsOwner::PhyLayer:
        return "phy";
    case FcsOwner::MacLayer:
        return "mac";
    }
    return "invalid";
}

// ---------------------------------------------------------------------------
// PHR: the standard HRP BPRF PHY header
// ---------------------------------------------------------------------------

// `phr_bytes()` answers "how many information bytes does the PHR carry", not
// "how long is the PHR on the air" -- those differ and conflating them is how
// a frame-duration budget gets understated.  The 13 information bits are
// exactly the standard 2-octet PHY header: 2 data-rate + 7 PSDU-length + 1
// ranging + 1 reserved + 2 preamble-duration index.  mod::encode_phr19()
// fills those 13 bits and SEC-DED(13,13)-codes them to 19 bits, which the
// modulator spreads over 21 PHR symbols (19 coded bits + 2 trailing bits).
// qa_uwb_twr_frame.cc asserts the 19 against the modulator's own constant.
inline constexpr size_t kPhrStandardInfoBytes = 2;
inline constexpr uint16_t kPhrStandardCodedBits = 19;

// ---------------------------------------------------------------------------
// A caller's claim about the layout
// ---------------------------------------------------------------------------

// A claim is a plain POD with the same field names as the configuration
// layer's `FrameGeometry`, on purpose: the config builds one of these, the
// frame layer checks it, and no third geometry type exists.  A distinct type
// from FrameProfileGeometry is also deliberate -- the authority and the claim
// can never be swapped for one another by accident.
struct FrameGeometryClaim {
    size_t mac_header_bytes = 0;
    size_t timestamp_bytes = 0;
    size_t mac_footer_bytes = 0;
    size_t mac_fcs_bytes = 0;
    size_t phr_bytes = 0;

    bool operator==(const FrameGeometryClaim& o) const
    {
        return mac_header_bytes == o.mac_header_bytes &&
               timestamp_bytes == o.timestamp_bytes &&
               mac_footer_bytes == o.mac_footer_bytes &&
               mac_fcs_bytes == o.mac_fcs_bytes && phr_bytes == o.phr_bytes;
    }
    bool operator!=(const FrameGeometryClaim& o) const { return !(*this == o); }
    bool any_set() const
    {
        return mac_header_bytes != 0 || timestamp_bytes != 0 ||
               mac_footer_bytes != 0 || mac_fcs_bytes != 0 || phr_bytes != 0;
    }
};

// ---------------------------------------------------------------------------
// The authority
// ---------------------------------------------------------------------------

struct FrameProfileGeometry {
    // Which layout this describes, and which layer owns the FCS.  Both are
    // part of the profile's identity, not per-call options: a caller states a
    // profile and gets exactly one geometry.
    FrameProfileId id = FrameProfileId::TwrV1;
    FcsOwner fcs_owner = FcsOwner::PhyLayer;

    // MAC header size.  Returns kFrameHeaderBytes -- the codec's own constant
    // -- so a 14 can never exist anywhere else in the system (R3).
    size_t mac_header_bytes() const { return kFrameHeaderBytes; }

    // Bytes per timestamp field.  Delegates to the profile's declared width
    // (a profile parameter, REQ-TIME-04): 5 B at the 40-bit frame-v1 default.
    size_t timestamp_bytes(const FrameProfile& p) const
    {
        // Qualified: an unqualified call would find this member and recurse.
        return ::gr::uwb::twr::timestamp_bytes(p);
    }

    // No trailing address/profile trailer in frame v1.  It is a method rather
    // than a constant so a future profile with a footer has one place to say
    // so instead of a caller inventing a third geometry.
    size_t mac_footer_bytes() const { return 0; }

    // FCS bytes reserved INSIDE the MAC PSDU.  Frame v1 is 0: the modulation
    // layer appends the FCS and the codec never emits one.
    //
    // The MAC-appends variant is a DESCRIPTION, not an executable path: see
    // frame_geometry_mac_appends_fcs() and executable().  This codec cannot
    // produce a MAC-appended PSDU -- frame_profile_validate() rejects
    // `fcs_appended_by_modulation_layer == false` -- so advertising that
    // layout as encodable would be a lie the validator then has to undo.
    size_t mac_fcs_bytes() const
    {
        return fcs_owner == FcsOwner::MacLayer ? kFrameFcsBytes : 0u;
    }

    // The FCS the PHY appends after the MAC payload.  2 B, IEEE 802.15.4.
    size_t fcs_bytes_on_air() const { return kFrameFcsBytes; }

    // PHR information size / coded size.  See kPhrStandardInfoBytes above.
    size_t phr_bytes() const { return kPhrStandardInfoBytes; }
    uint16_t phr_coded_bits() const { return kPhrStandardCodedBits; }

    // MAC PSDU of one frame as THIS profile builds it -- the single place, a
    // delegation to frame_length_for() plus the FCS bytes this profile
    // reserves inside the MAC PSDU.  For the executable frame-v1 profile that
    // reservation is 0, so this is exactly 14 / 24 / 29 B; for the
    // MAC-appends description it is the 2 FCS bytes the MAC itself adds.
    size_t mac_payload_bytes(FrameType t, const FrameProfile& p) const
    {
        return frame_length_for(t, p) + mac_fcs_bytes();
    }

    // Size-checked form: 0 with a reason for a reserved frame type (Report),
    // an unusable profile, or a frame that would not fit the profile's PSDU
    // once the FCS is on it.  Delegates to frame_length_for_checked().
    size_t mac_payload_bytes_checked(FrameType t,
                                     const FrameProfile& p,
                                     std::string& error,
                                     FrameError* code = nullptr) const
    {
        const size_t n = frame_length_for_checked(t, p, error, code);
        if (n == 0)
            return 0;
        return n + mac_fcs_bytes();
    }

    // On-air PSDU: the MAC payload plus the 2 FCS bytes, whichever layer
    // appends them -- 16 / 26 / 31 B.  Invariant under fcs_owner: moving who
    // appends the FCS does not change a single sample on the air, it only
    // changes which layer is responsible for it.
    size_t on_air_bytes(FrameType t, const FrameProfile& p) const
    {
        return psdu_length_for(t, p);
    }

    // True only for a layout this codec can actually build.  A MAC-appends
    // geometry is describeable but not encodable here.
    bool executable() const { return fcs_owner == FcsOwner::PhyLayer; }

    // The claim that agrees with this authority, for a caller filling a
    // FrameGeometryClaim in.  Checking it back must report no mismatch; that
    // is the self-consistency the R3 regression test relies on.
    FrameGeometryClaim claim(const FrameProfile& p) const
    {
        FrameGeometryClaim c;
        c.mac_header_bytes = mac_header_bytes();
        c.timestamp_bytes = timestamp_bytes(p);
        c.mac_footer_bytes = mac_footer_bytes();
        c.mac_fcs_bytes = mac_fcs_bytes();
        c.phr_bytes = phr_bytes();
        return c;
    }
};

inline bool operator==(const FrameProfileGeometry& a, const FrameProfileGeometry& b)
{
    return a.id == b.id && a.fcs_owner == b.fcs_owner;
}
inline bool operator!=(const FrameProfileGeometry& a, const FrameProfileGeometry& b)
{
    return !(a == b);
}

// The description-only variant: same layout, but the MAC reserves and appends
// the 2 FCS bytes itself instead of the modulation layer.  Provided so a
// configuration can STATE that layout and be told it is not executable here,
// rather than having the number silently accepted (R3).
inline FrameProfileGeometry frame_geometry_mac_appends_fcs(FrameProfileId id = FrameProfileId::TwrV1)
{
    FrameProfileGeometry g;
    g.id = id;
    g.fcs_owner = FcsOwner::MacLayer;
    return g;
}

// The executable authority for a profile id, or false when the id names no
// implemented layout (never a silent fallback to frame v1).
inline bool frame_geometry_for(FrameProfileId id, FrameProfileGeometry& out)
{
    if (!frame_profile_id_is_supported(id))
        return false;
    out = FrameProfileGeometry();
    out.id = id;
    out.fcs_owner = FcsOwner::PhyLayer;
    return true;
}

// ---------------------------------------------------------------------------
// Cross-checking a caller's claim
// ---------------------------------------------------------------------------

// The five comparable fields, in a fixed report order.
enum class GeometryField : uint8_t {
    MacHeaderBytes = 0,
    TimestampBytes = 1,
    MacFooterBytes = 2,
    MacFcsBytes = 3,
    PhrBytes = 4,
    Count = 5
};

inline constexpr size_t kGeometryFieldCount =
    static_cast<size_t>(GeometryField::Count);

inline const char* geometry_field_to_string(GeometryField f)
{
    switch (f) {
    case GeometryField::MacHeaderBytes:
        return "mac_header_bytes";
    case GeometryField::TimestampBytes:
        return "timestamp_bytes";
    case GeometryField::MacFooterBytes:
        return "mac_footer_bytes";
    case GeometryField::MacFcsBytes:
        return "mac_fcs_bytes";
    case GeometryField::PhrBytes:
        return "phr_bytes";
    case GeometryField::Count:
        break;
    }
    return "invalid";
}

struct GeometryMismatch {
    GeometryField field = GeometryField::MacHeaderBytes;
    size_t expected = 0;
    size_t actual = 0;
};

// Fixed capacity, trivially copyable: five fields, so the report is a value
// type with no allocation.  A validator can hand the whole thing to a log.
struct GeometryMismatchReport {
    static constexpr size_t capacity = kGeometryFieldCount;
    GeometryMismatch items[capacity] = {};
    size_t count = 0;

    bool ok() const { return count == 0; }
    const GeometryMismatch* at(size_t index) const
    {
        if (index >= count)
            return nullptr;
        return &items[index];
    }
    const GeometryMismatch* find(GeometryField f) const
    {
        for (size_t i = 0; i < count; ++i) {
            if (items[i].field == f)
                return &items[i];
        }
        return nullptr;
    }
};

struct GeometryCheckResult {
    FrameProfileId id = FrameProfileId::TwrV1;
    FrameProfileGeometry authority;
    FrameGeometryClaim claimed;
    // False when the authority is describeable but not encodable by this codec
    // (the MAC-appends variant).
    bool fcs_owner_ok = true;
    GeometryMismatchReport fields;

    bool ok() const { return fcs_owner_ok && fields.ok(); }
    // The first mismatching field, or null when `ok()`.
    const GeometryMismatch* first_mismatch() const { return fields.at(0); }

    // One line naming every field that disagrees, for a rejection message:
    //   "mac_header_bytes: expected 14, got 7; phr_bytes: expected 2, got 12"
    std::string summary() const;
};

// Checks a caller-supplied claim against the authority, per field.  This is
// the function a configuration validator calls: it never decides geometry, it
// only reports where the claim diverges from the one source of truth, so a
// 7-byte header is reported as `mac_header_bytes: expected 14, got 7` rather
// than quietly accepted with a 9-byte budget.
inline GeometryCheckResult
frame_geometry_check(const FrameGeometryClaim& claimed,
                     const FrameProfileGeometry& authority,
                     const FrameProfile& p)
{
    GeometryCheckResult r;
    r.id = authority.id;
    r.authority = authority;
    r.claimed = claimed;
    r.fcs_owner_ok = authority.executable();

    const size_t want[kGeometryFieldCount] = {
        authority.mac_header_bytes(), authority.timestamp_bytes(p),
        authority.mac_footer_bytes(), authority.mac_fcs_bytes(),
        authority.phr_bytes()
    };
    const size_t got[kGeometryFieldCount] = {
        claimed.mac_header_bytes, claimed.timestamp_bytes,
        claimed.mac_footer_bytes, claimed.mac_fcs_bytes, claimed.phr_bytes
    };
    for (size_t i = 0; i < kGeometryFieldCount; ++i) {
        if (want[i] == got[i])
            continue;
        GeometryMismatch& m = r.fields.items[r.fields.count++];
        m.field = static_cast<GeometryField>(i);
        m.expected = want[i];
        m.actual = got[i];
    }
    return r;
}

inline std::string GeometryCheckResult::summary() const
{
    std::string s;
    for (size_t i = 0; i < fields.count; ++i) {
        if (!s.empty())
            s += "; ";
        s += geometry_field_to_string(fields.items[i].field);
        s += ": expected ";
        s += std::to_string(fields.items[i].expected);
        s += ", got ";
        s += std::to_string(fields.items[i].actual);
    }
    if (!fcs_owner_ok) {
        if (!s.empty())
            s += "; ";
        s += "fcs_owner: expected ";
        s += fcs_owner_to_string(FcsOwner::PhyLayer);
        s += ", got ";
        s += fcs_owner_to_string(authority.fcs_owner);
        s += " (this codec only encodes the PHY-appends form)";
    }
    return s;
}

// ---------------------------------------------------------------------------
// Session id: the local -> wire contract
// ---------------------------------------------------------------------------

// `kOffSessionId` is 2 bytes, so the wire session field is 16 bits wide and
// that is the ENTIRE session space.  Everything below follows from those two
// facts; none of it is a policy choice the codec could have made differently.
inline constexpr unsigned kSessionIdWireBits = 16;
inline constexpr uint32_t kSessionIdWireMax = 0xFFFFu;

// A locally chosen session id of 0 must not reach the air: the configuration
// layer uses it as its "not stated" marker.  The codec itself does not refuse
// it -- a `Frame` is a `Frame` -- so this is a caller rule, stated here so both
// sides quote the same constant instead of inventing one.
inline constexpr uint32_t kSessionIdReservedLocal = 0;

enum class SessionIdError : uint8_t {
    None = 0,
    OutOfWireRange // local id > kSessionIdWireMax: refused, never truncated
};

inline const char* session_id_error_to_string(SessionIdError e)
{
    switch (e) {
    case SessionIdError::None:
        return "none";
    case SessionIdError::OutOfWireRange:
        return "session_id_out_of_wire_range";
    }
    return "invalid";
}

inline ExchangeStatus session_id_error_to_exchange_status(SessionIdError e)
{
    switch (e) {
    case SessionIdError::None:
        return ExchangeStatus::Ok;
    case SessionIdError::OutOfWireRange:
        // A configuration that cannot be represented is rejected before the
        // radio starts; it is not a peer or a decode problem.
        return ExchangeStatus::ConfigRejected;
    }
    return ExchangeStatus::InternalError;
}

// THE MAPPING RULE (frame v1):
//
//     wire_session_id == local_session_id, bit for bit, 16 bits,
//                       little-endian at kOffSessionId
//     a local id outside [0, 0xFFFF] is REFUSED
//
// There is deliberately no folding, hashing, low-word extraction or derived
// value.  A lossy local->wire map would make two different local sessions
// produce byte-identical session fields, and frame_match() would then accept
// the wrong session's reply as this one's (REQ-PROTO-01) -- a wrong-peer frame
// silently folded into a range.  A refusal is visible and attributable; a
// collision is neither.  So the rule is: in range -> exact, out of range ->
// explicit error.  There is no third case.
inline bool session_id_fits_wire(uint32_t local_session_id)
{
    return local_session_id <= kSessionIdWireMax;
}

inline bool session_id_to_wire(uint32_t local_session_id,
                               uint16_t& wire,
                               std::string& error,
                               SessionIdError* code = nullptr)
{
    if (!session_id_fits_wire(local_session_id)) {
        if (code != nullptr)
            *code = SessionIdError::OutOfWireRange;
        error = std::string(session_id_error_to_string(SessionIdError::OutOfWireRange)) +
                ": local session id " + std::to_string(local_session_id) +
                " does not fit the " + std::to_string(kSessionIdWireBits) +
                "-bit wire field at kOffSessionId; refused, not truncated to " +
                std::to_string(static_cast<uint32_t>(local_session_id & kSessionIdWireMax));
        wire = 0; // never leave a plausible-looking id behind on failure
        return false;
    }
    wire = static_cast<uint16_t>(local_session_id);
    if (code != nullptr)
        *code = SessionIdError::None;
    error.clear();
    return true;
}

// THE COLLISION PROPERTY, as a predicate so it is tested rather than merely
// asserted in prose:
//
//   Two ACCEPTED local session ids are indistinguishable on the wire if and
//   only if they are the same integer.  Because an out-of-range id is refused
//   instead of narrowed, the mapping is injective over everything that can
//   reach the air; the `fits` tests are what make that statement true rather
//   than false for a rejected id (which collides with nothing, because it
//   never goes on the air).
//
// THE CONSEQUENCE, which any session-space sizing must state: the wire field is
// 16 bits, so the space is 2^16 sessions and the birthday bound is ~2^8 = 256
// concurrently live sessions at a 50% chance of one collision.  That density
// is a consequence of the 16-bit field at kOffSessionId.  The codec cannot fix
// it, and NO local->wire mapping can raise it: a wider local id folded into 16
// wire bits can only lose distinctions, never create room.  Raising it needs a
// different frame version with a wider (or keyed) session field, which the
// peer must also understand -- i.e. an interoperability change, not a config
// option.
inline constexpr double kSessionIdBirthdaySessions50pct = 256.0; // ~sqrt(2^16)

inline bool wire_session_id_collides(uint32_t a, uint32_t b)
{
    return session_id_fits_wire(a) && session_id_fits_wire(b) && a == b;
}

// ---------------------------------------------------------------------------
// Little-endian primitives (the wire order, in one place)
// ---------------------------------------------------------------------------

inline void put_u16_le(uint8_t* p, uint16_t v)
{
    p[0] = static_cast<uint8_t>(v & 0xffu);
    p[1] = static_cast<uint8_t>((v >> 8) & 0xffu);
}

inline uint16_t get_u16_le(const uint8_t* p)
{
    return static_cast<uint16_t>(static_cast<uint16_t>(p[0]) |
                                 static_cast<uint16_t>(static_cast<uint16_t>(p[1]) << 8));
}

// Writes the low `n` bytes of `v`, least significant first.  `n` is at most 8.
inline void put_uint_le(uint8_t* p, uint64_t v, size_t n)
{
    for (size_t i = 0; i < n; ++i) {
        p[i] = static_cast<uint8_t>((v >> (8u * i)) & 0xffu);
    }
}

inline uint64_t get_uint_le(const uint8_t* p, size_t n)
{
    uint64_t v = 0;
    for (size_t i = 0; i < n; ++i) {
        v |= static_cast<uint64_t>(p[i]) << (8u * i);
    }
    return v;
}

// ---------------------------------------------------------------------------
// encode
// ---------------------------------------------------------------------------

// Validates the frame against the profile and writes the MAC payload -- and
// NOTHING ELSE, in particular no FCS -- into caller-provided storage.
//
// `out` must have room for frame_length_for(f.function_code, p) bytes; on
// success `written` is exactly that.  Allocation-free apart from assigning to
// the caller's `error` string, so this is the form the FSM and the delayed-TX
// path use.
inline bool encode_into(const Frame& f,
                        const FrameProfile& p,
                        uint8_t* out,
                        size_t out_capacity,
                        size_t& written,
                        std::string& error,
                        FrameError* code = nullptr)
{
    written = 0;

    if (out == nullptr) {
        set_error(error, code, FrameError::NullPointer, "null output buffer");
        return false;
    }
    if (!frame_profile_validate(p, error, code))
        return false;

    // Version must match the profile; an unknown layout is never guessed at.
    if (f.version != p.version) {
        set_error(error, code, FrameError::BadVersion, "frame version != profile version");
        return false;
    }

    const FrameLayout* layout = frame_layout(f.function_code);
    if (layout == nullptr) {
        // Report, or an out-of-range enum value.
        if (f.function_code == FrameType::Report) {
            set_error(error, code, FrameError::NotImplemented,
                      "report frame type is reserved; phase 1 emits the "
                      "three-message exchange only (REQ-PROTO-05)");
        } else {
            set_error(error, code, FrameError::BadFunctionCode,
                      "unknown function code");
        }
        return false;
    }

    if (flags_has_reserved_bits(f.flags)) {
        set_error(error, code, FrameError::ReservedFlags,
                  "flags bits 2..15 are reserved and must be zero");
        return false;
    }
    if (flags_sts_present(f.flags)) {
        set_error(error, code, FrameError::StsUnsupported,
                  "sts_present is set; STS is out of scope for phase 1");
        return false;
    }

    const size_t tsb = timestamp_bytes(p);
    const size_t n_ts = layout->timestamp_count;
    // frame_length_for() is the ONE place a frame's length is computed; this
    // function must not restate the arithmetic (R3: no second geometry).
    const size_t need = frame_length_for(f.function_code, p);

    // A value that does not fit the declared field is refused, never
    // truncated: truncation would silently change a measurement instant.
    const uint64_t tmax = timestamp_max_value(p);
    for (size_t i = 0; i < n_ts; ++i) {
        if (f.timestamps[i] > tmax) {
            set_error(error, code, FrameError::TimestampOutOfRange,
                      "timestamp does not fit the profile field width");
            return false;
        }
    }
    // An unused slot must be zero so a frame's bytes are a function of its
    // declared content alone and a stale instant can never ride along.
    for (size_t i = n_ts; i < kMaxTimestamps; ++i) {
        if (f.timestamps[i] != 0) {
            set_error(error, code, FrameError::ReservedTimestampNonZero,
                      "unused timestamp slot is non-zero");
            return false;
        }
    }

    if (need > out_capacity) {
        set_error(error, code, FrameError::BufferTooSmall,
                  "caller buffer smaller than the frame");
        return false;
    }
    if (need + kFrameFcsBytes > p.max_psdu_bytes) {
        set_error(error, code, FrameError::PsduTooLarge,
                  "frame plus FCS exceeds the profile max PSDU");
        return false;
    }

    std::memset(out, 0, need);
    out[kOffVersion] = f.version;
    out[kOffFunctionCode] = static_cast<uint8_t>(f.function_code);
    put_u16_le(out + kOffSessionId, f.session_id);
    put_u16_le(out + kOffSeq, f.seq);
    put_u16_le(out + kOffPanId, f.pan_id);
    put_u16_le(out + kOffSrcAddr, f.src_addr);
    put_u16_le(out + kOffDstAddr, f.dst_addr);
    put_u16_le(out + kOffFlags, f.flags);
    for (size_t i = 0; i < n_ts; ++i) {
        put_uint_le(out + kOffTimestamps + i * tsb, f.timestamps[i], tsb);
    }

    written = need;
    if (code != nullptr)
        *code = FrameError::None;
    error.clear();
    return true;
}

// Convenience form for setup, logging and tests.  `out` is resized to the
// exact MAC payload length; no FCS is appended (see the header comment).
inline bool encode(const Frame& f,
                   const FrameProfile& p,
                   std::vector<uint8_t>& out,
                   std::string& error,
                   FrameError* code = nullptr)
{
    out.clear();
    std::string ignored;
    if (!frame_profile_validate(p, ignored, code)) {
        error = ignored;
        return false;
    }
    const FrameType t = f.function_code;
    if (!frame_type_is_implemented(t)) {
        set_error(error, code,
                  t == FrameType::Report ? FrameError::NotImplemented
                                         : FrameError::BadFunctionCode,
                  t == FrameType::Report
                      ? "report frame type is reserved; phase 1 emits the "
                        "three-message exchange only (REQ-PROTO-05)"
                      : "unknown function code");
        return false;
    }
    const size_t need = frame_length_for(t, p);
    out.resize(need);
    size_t written = 0;
    if (!encode_into(f, p, out.data(), out.size(), written, error, code)) {
        out.clear();
        return false;
    }
    out.resize(written);
    return true;
}

// ---------------------------------------------------------------------------
// decode
// ---------------------------------------------------------------------------

// Strict decode of one MAC payload.
//
// The input MUST be exactly the MAC payload: `frame_length_for(function_code,
// profile)` bytes with the two FCS bytes already removed by the receive path.
// A short buffer is `Truncated`; a long one -- which is what a PSDU that still
// carries its FCS looks like -- is `LengthMismatch`.  Nothing is inferred.
//
// On failure `out` is left in a state no valid profile accepts (`version` 0),
// so a caller that ignores the return value still cannot mistake a failed
// decode for a frame that may advance the FSM (REQ-PROTO-01).
inline bool decode(const uint8_t* data,
                   size_t n,
                   const FrameProfile& p,
                   Frame& out,
                   std::string& error,
                   FrameError* code = nullptr)
{
    out = Frame();
    out.version = 0; // invalid on purpose; see the comment above

    if (data == nullptr && n != 0) {
        set_error(error, code, FrameError::NullPointer, "null payload pointer");
        return false;
    }
    if (!frame_profile_validate(p, error, code))
        return false;

    if (n < kFrameHeaderBytes) {
        set_error(error, code, FrameError::Truncated, "shorter than the 14-byte header");
        return false;
    }

    if (data[kOffVersion] != p.version) {
        set_error(error, code, FrameError::BadVersion, "unknown frame version");
        return false;
    }

    FrameType type = FrameType::Poll;
    if (!frame_type_from_wire(data[kOffFunctionCode], type)) {
        set_error(error, code, FrameError::BadFunctionCode, "unknown function code");
        return false;
    }
    const FrameLayout* layout = frame_layout(type);
    if (layout == nullptr) {
        set_error(error, code, FrameError::NotImplemented,
                  "report frame type is reserved; phase 1 decodes the "
                  "three-message exchange only (REQ-PROTO-05)");
        return false;
    }

    const uint16_t flags = get_u16_le(data + kOffFlags);
    if (flags_has_reserved_bits(flags)) {
        set_error(error, code, FrameError::ReservedFlags,
                  "flags bits 2..15 are reserved and must be zero");
        return false;
    }
    if (flags_sts_present(flags)) {
        set_error(error, code, FrameError::StsUnsupported,
                  "sts_present is set; STS is out of scope for phase 1");
        return false;
    }

    const size_t tsb = timestamp_bytes(p);
    // Same single place as encode_into(): a frame's length is computed once.
    const size_t need = frame_length_for(type, p);
    if (n < need) {
        set_error(error, code, FrameError::Truncated,
                  "timestamp block is shorter than the function code requires");
        return false;
    }
    if (n > need) {
        set_error(error, code, FrameError::LengthMismatch,
                  "buffer is longer than the function code requires; a PSDU "
                  "whose FCS was not stripped is the usual cause");
        return false;
    }
    if (need + kFrameFcsBytes > p.max_psdu_bytes) {
        set_error(error, code, FrameError::PsduTooLarge,
                  "frame plus FCS exceeds the profile max PSDU");
        return false;
    }

    Frame f;
    f.version = data[kOffVersion];
    f.function_code = type;
    f.session_id = get_u16_le(data + kOffSessionId);
    f.seq = get_u16_le(data + kOffSeq);
    f.pan_id = get_u16_le(data + kOffPanId);
    f.src_addr = get_u16_le(data + kOffSrcAddr);
    f.dst_addr = get_u16_le(data + kOffDstAddr);
    f.flags = flags;
    for (size_t i = 0; i < kMaxTimestamps; ++i) {
        f.timestamps[i] = 0;
    }
    for (size_t i = 0; i < layout->timestamp_count; ++i) {
        const uint64_t v =
            get_uint_le(data + kOffTimestamps + i * tsb, tsb);
        if (!timestamp_fits(p, v)) {
            // Unreachable for a whole-byte field, kept as a standing
            // invariant check: a value that cannot be represented
            // unambiguously in this profile must never leave the codec.
            set_error(error, code, FrameError::TimestampOutOfRange,
                      "decoded timestamp is not representable in this profile");
            return false;
        }
        f.timestamps[i] = v;
    }

    out = f;
    if (code != nullptr)
        *code = FrameError::None;
    error.clear();
    return true;
}

inline bool decode(const std::vector<uint8_t>& mac_payload,
                   const FrameProfile& p,
                   Frame& out,
                   std::string& error,
                   FrameError* code = nullptr)
{
    return decode(mac_payload.data(), mac_payload.size(), p, out, error, code);
}

// Splits a received PSDU into its MAC payload, dropping exactly the two FCS
// bytes the modulation layer appended.  The FCS itself is verified by the
// demodulator (it is what produced the PDU), so this is a framing step only;
// a PSDU too short to hold a header plus an FCS is rejected.
inline bool mac_payload_from_psdu(const uint8_t* psdu,
                                  size_t n,
                                  const FrameProfile& p,
                                  const uint8_t*& payload,
                                  size_t& payload_bytes,
                                  std::string& error,
                                  FrameError* code = nullptr)
{
    payload = nullptr;
    payload_bytes = 0;
    if (psdu == nullptr && n != 0) {
        set_error(error, code, FrameError::NullPointer, "null psdu pointer");
        return false;
    }
    if (n < kFrameHeaderBytes + kFrameFcsBytes) {
        set_error(error, code, FrameError::Truncated, "psdu shorter than header + FCS");
        return false;
    }
    const size_t m = n - kFrameFcsBytes;
    if (m + kFrameFcsBytes > p.max_psdu_bytes) {
        set_error(error, code, FrameError::PsduTooLarge,
                  "psdu exceeds the profile max PSDU");
        return false;
    }
    payload = psdu;
    payload_bytes = m;
    if (code != nullptr)
        *code = FrameError::None;
    error.clear();
    return true;
}

// ---------------------------------------------------------------------------
// Session / sequence helpers and frame matching
// ---------------------------------------------------------------------------

// session_id and seq wrap INDEPENDENTLY (REQ-API-01): a 16-bit session id is
// not "the same counter as" seq, and neither is derived from the other.
inline uint16_t next_seq(uint16_t seq)
{
    return static_cast<uint16_t>(seq + 1u); // 0xFFFF -> 0x0000
}

inline uint16_t next_session_id(uint16_t session_id)
{
    return static_cast<uint16_t>(session_id + 1u); // 0xFFFF -> 0x0000
}

// Exact equality.  Deliberately not a tolerance and not a modulo compare: a
// frame belongs to an exchange if and only if its seq is the expected one.
inline bool seq_matches(uint16_t expected, uint16_t received) { return expected == received; }

// True when `candidate` is strictly newer than `previous` in the 16-bit ring,
// i.e. within +/- 32768.  Used only for "is this a duplicate of what I
// already answered" bookkeeping -- never to accept a frame as a measurement.
inline bool seq_is_newer(uint16_t previous, uint16_t candidate)
{
    return static_cast<int16_t>(static_cast<uint16_t>(candidate - previous)) > 0;
}

enum class FrameMatch : uint8_t {
    Match = 0,
    TypeMismatch,    // wrong message for the current FSM state
    PanMismatch,     // different PAN
    AddressMismatch, // not addressed to this endpoint
    SelfAddressed,   // our own address echoed on both fields (TX loopback)
    SessionMismatch, // different/expired session
    SeqMismatch      // different exchange within the same session
};

inline const char* frame_match_to_string(FrameMatch m)
{
    switch (m) {
    case FrameMatch::Match:
        return "match";
    case FrameMatch::TypeMismatch:
        return "type_mismatch";
    case FrameMatch::PanMismatch:
        return "pan_mismatch";
    case FrameMatch::AddressMismatch:
        return "address_mismatch";
    case FrameMatch::SelfAddressed:
        return "self_addressed";
    case FrameMatch::SessionMismatch:
        return "session_mismatch";
    case FrameMatch::SeqMismatch:
        return "seq_mismatch";
    }
    return "invalid";
}

inline ExchangeStatus frame_match_to_exchange_status(FrameMatch m)
{
    switch (m) {
    case FrameMatch::Match:
        return ExchangeStatus::Ok;
    case FrameMatch::TypeMismatch:
        return ExchangeStatus::UnexpectedFrameType;
    case FrameMatch::PanMismatch:
    case FrameMatch::AddressMismatch:
    case FrameMatch::SelfAddressed:
        return ExchangeStatus::WrongPeer;
    case FrameMatch::SessionMismatch:
        return ExchangeStatus::StaleSession;
    case FrameMatch::SeqMismatch:
        return ExchangeStatus::StaleSession;
    }
    return ExchangeStatus::InternalError;
}

struct PeerExpectation {
    FrameType expected_type = FrameType::Poll;
    uint16_t pan_id = 0;
    uint16_t local_addr = 0; // this endpoint's own address
    uint16_t session_id = 0;
    uint16_t seq = 0;
};

// REQ-PROTO-01: only a frame whose address, session, sequence and message type
// all match the current state may advance the FSM.  The checks are reported
// SEPARATELY so a late frame, a duplicate and a stale session are distinct
// statistics rather than one bucket -- and so a failure can never be
// "recovered" by skipping a number and continuing.
//
// Check order is fixed and documented: type, then PAN, then address, then
// session, then seq.  Type comes first because for a wrong message type the
// remaining fields have no defined meaning.
inline FrameMatch frame_match(const Frame& f, const PeerExpectation& e)
{
    if (f.function_code != e.expected_type)
        return FrameMatch::TypeMismatch;
    if (f.pan_id != e.pan_id)
        return FrameMatch::PanMismatch;
    if (f.dst_addr != e.local_addr)
        return FrameMatch::AddressMismatch;
    // A frame whose src and dst are both us is our own transmission coming
    // back.  Counting that as a peer measurement would fabricate a round trip
    // (REQ-PROTO-01/03), so it is refused rather than accepted.
    if (f.src_addr == e.local_addr)
        return FrameMatch::SelfAddressed;
    if (f.session_id != e.session_id)
        return FrameMatch::SessionMismatch;
    if (!seq_matches(e.seq, f.seq))
        return FrameMatch::SeqMismatch;
    return FrameMatch::Match;
}

// ---------------------------------------------------------------------------
// Hex helpers (logging, golden files, failure reports)
// ---------------------------------------------------------------------------

inline std::string bytes_to_hex(const uint8_t* data, size_t n)
{
    static const char* kDigits = "0123456789abcdef";
    std::string out;
    out.reserve(n * 2);
    for (size_t i = 0; i < n; ++i) {
        out.push_back(kDigits[(data[i] >> 4) & 0xf]);
        out.push_back(kDigits[data[i] & 0xf]);
    }
    return out;
}

inline std::string bytes_to_hex(const std::vector<uint8_t>& v)
{
    return bytes_to_hex(v.data(), v.size());
}

// Parses lowercase or uppercase hex into bytes.  Returns false on an odd
// length or a non-hex character; `out` is cleared in that case.
inline bool hex_to_bytes(const std::string& hex, std::vector<uint8_t>& out)
{
    out.clear();
    if (hex.size() % 2 != 0)
        return false;
    out.reserve(hex.size() / 2);
    for (size_t i = 0; i < hex.size(); i += 2) {
        int hi = -1;
        int lo = -1;
        for (int pass = 0; pass < 2; ++pass) {
            const char c = hex[i + static_cast<size_t>(pass)];
            int v = -1;
            if (c >= '0' && c <= '9')
                v = c - '0';
            else if (c >= 'a' && c <= 'f')
                v = c - 'a' + 10;
            else if (c >= 'A' && c <= 'F')
                v = c - 'A' + 10;
            if (v < 0)
                return false;
            if (pass == 0)
                hi = v;
            else
                lo = v;
        }
        out.push_back(static_cast<uint8_t>((hi << 4) | lo));
    }
    return true;
}

} // namespace gr::uwb::twr

#endif /* INCLUDED_GNURADIO_UWB_UWB_TWR_FRAME_H */
