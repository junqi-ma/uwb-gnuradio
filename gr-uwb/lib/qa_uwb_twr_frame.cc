/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * QA for gr::uwb::twr frame codec (gr-uwb/include/gnuradio/uwb/uwb_twr_frame.h),
 * milestone M0 / REQ-QA-02 "帧" row.
 *
 * WHAT THIS TEST PROVES
 * ---------------------
 * 1. BYTE-EXACT GOLDENS.  testdata/twr/frame_golden_v1.json is TEST DATA, not
 *    documentation.  The expected bytes appear a third time in this file as a
 *    hand-written table transcribed from the frozen field specification.  This
 *    QA asserts all three agree: golden JSON == hand table == codec output,
 *    byte for byte, at every offset.  Any one of them drifting fails the QA.
 * 2. BYTE ORDER.  The byte at each named offset is asserted individually
 *    (version, function code, session_id, seq, pan_id, src, dst, flags, then
 *    every timestamp byte), not merely that a round trip is self-consistent.
 * 3. REFUSAL, NOT DEFAULTS.  Report (0x03), unknown versions, unknown
 *    function codes, reserved flag bits 2..15, sts_present, out-of-range
 *    timestamps, malformed and truncated input are all rejected with a
 *    specific FrameError, and every rejection maps onto the right
 *    ExchangeStatus.  Nothing is silently reinterpreted or ignored.
 * 4. FCS HAS EXACTLY ONE PRODUCER.  The codec output is 14 / 24 / 29 bytes; the
 *    two FCS bytes are added by UwbHrpPacketSource(append_fcs=true) through
 *    mod::append_ieee_fcs, giving 16 / 26 / 31.  A tree scan fails the QA if a
 *    second production layer ever appears, and decoding a buffer that still
 *    carries its FCS is rejected instead of being reinterpreted.
 * 5. PROFILE-PARAMETRIC TIMESTAMPS.  Width and tick rate are profile
 *    parameters (REQ-TIME-04), so 32- and 64-bit profiles are exercised too,
 *    including that a frame written under one width is not accepted under
 *    another.
 * 6. INDEPENDENT WRAP.  session_id and seq wrap independently at 0xFFFF and
 *    are compared independently, and the 40-bit timestamp field round trips
 *    exactly at 0xFFFFFFFFFF while 2^40 is refused.
 *
 * The bytes in the JSON and in the hand table were produced from the field
 * table by an independent serialiser, NOT by running the codec; otherwise the
 * golden would only prove the codec agrees with itself.
 *
 * Requires UWB_TESTDATA_DIR (the project's CMake defines it).  No hardware, no
 * GNU Radio scheduler, no network.
 */

#include <boost/test/unit_test.hpp>

// The one layer that appends the FCS (see uwb_hrp_packet_source.cc).
#include <gnuradio/uwb/uwb_hrp_mod_core.h>
#include <gnuradio/uwb/uwb_radar_pdu_meta.h>
#include <gnuradio/uwb/uwb_twr_frame.h>
#include <gnuradio/uwb/uwb_twr_types.h>

#include <dirent.h>

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <ostream>
#include <sstream>
#include <string>
#include <vector>

// Boost's print helper cannot render a scoped enum, so give it the codec's
// own stable names.  Declared in the enum's namespace so ADL finds them.
namespace gr {
namespace uwb {
namespace twr {

inline std::ostream& operator<<(std::ostream& os, FrameError e)
{
    return os << frame_error_to_string(e);
}

inline std::ostream& operator<<(std::ostream& os, FrameMatch m)
{
    return os << frame_match_to_string(m);
}

inline std::ostream& operator<<(std::ostream& os, TimestampField f)
{
    return os << timestamp_field_name(f);
}

inline std::ostream& operator<<(std::ostream& os, ExchangeStatus s)
{
    return os << exchange_status_to_string(s);
}

} // namespace twr
} // namespace uwb
} // namespace gr

using gr::uwb::twr::Frame;
using gr::uwb::twr::FrameError;
using gr::uwb::twr::FrameLayout;
using gr::uwb::twr::FrameMatch;
using gr::uwb::twr::FrameProfile;
using gr::uwb::twr::TimestampField;

#ifndef UWB_TESTDATA_DIR
#define UWB_TESTDATA_DIR "../../../testdata"
#endif

namespace {

// ---------------------------------------------------------------------------
// Hand-transcribed goldens
// ---------------------------------------------------------------------------
//
// Transcribed by hand from the frozen field table:
//
//   off size field        Poll  Response  Final
//    0   1   version     01
//    1   1   function    00   01        02
//    2   2   session_id  efbe (LE for 0xBEEF)
//    4   2   seq         0201 (LE for 0x0102)
//    6   2   pan_id      3412 (LE for 0x1234)
//    8   2   src_addr    0100 / 0200
//   10   2   dst_addr    0200 / 0100
//   12   2   flags       0100 (LE for 0x0001, ranging)
//   14   5*n timestamps
//
// Fixed values: version 0x01, session_id 0xBEEF, seq 0x0102, pan_id 0x1234,
// src_addr 0x0001 (endpoint A), dst_addr 0x0002 (endpoint B), flags 0x0001
// (ranging set, sts clear, bits 2..15 clear).

constexpr uint16_t kGoldenSession = 0xBEEF;
constexpr uint16_t kGoldenSeq = 0x0102;
constexpr uint16_t kGoldenPan = 0x1234;
constexpr uint16_t kAddrA = 0x0001;
constexpr uint16_t kAddrB = 0x0002;
constexpr uint16_t kGoldenFlags = 0x0001; // ranging

struct Golden {
    const char* name;
    gr::uwb::twr::FrameType type;
    uint16_t session_id;
    uint16_t seq;
    uint16_t src_addr;
    uint16_t dst_addr;
    uint64_t timestamps[3];
    size_t timestamp_count;
    size_t payload_bytes;
    const char* mac_payload_hex;
};

const Golden kGoldens[] = {
    { "poll_v1",
      gr::uwb::twr::FrameType::Poll,
      kGoldenSession,
      kGoldenSeq,
      kAddrA,
      kAddrB,
      { 0, 0, 0 },
      0,
      14,
      "0100efbe02013412010002000100" },
    { "response_v1",
      gr::uwb::twr::FrameType::Response,
      kGoldenSession,
      kGoldenSeq,
      kAddrB,
      kAddrA,
      { 0x0123456789ull, 0x00FEDCBAull, 0 },
      2,
      24,
      "0101efbe020134120200010001008967452301badcfe0000" },
    { "final_v1",
      gr::uwb::twr::FrameType::Final,
      kGoldenSession,
      kGoldenSeq,
      kAddrA,
      kAddrB,
      { 0x1122334455ull, 0x9988776655ull, 0x0102030405ull },
      3,
      29,
      "0102efbe02013412010002000100554433221155667788990504030201" },
    { "poll_wrap_v1",
      gr::uwb::twr::FrameType::Poll,
      0xFFFF,
      0xFFFF,
      kAddrA,
      kAddrB,
      { 0, 0, 0 },
      0,
      14,
      "0100ffffffff3412010002000100" },
    { "response_wrap_v1",
      gr::uwb::twr::FrameType::Response,
      kGoldenSession,
      kGoldenSeq,
      kAddrB,
      kAddrA,
      { 0xFFFFFFFFFFull, 0x0000000000ull, 0 },
      2,
      24,
      "0101efbe02013412020001000100ffffffffff0000000000" },
    { "final_wrap_v1",
      gr::uwb::twr::FrameType::Final,
      kGoldenSession,
      kGoldenSeq,
      kAddrA,
      kAddrB,
      { 0xFFFFFFFFFFull, 0x0000000000ull, 0x8000000000ull },
      3,
      29,
      "0102efbe02013412010002000100ffffffffff00000000000000000080" },
};

constexpr size_t kNumGoldens = sizeof(kGoldens) / sizeof(kGoldens[0]);

FrameProfile default_profile() { return FrameProfile(); }

Frame make_golden_frame(const Golden& g)
{
    Frame f;
    f.version = gr::uwb::twr::kFrameVersion;
    f.function_code = g.type;
    f.session_id = g.session_id;
    f.seq = g.seq;
    f.pan_id = kGoldenPan;
    f.src_addr = g.src_addr;
    f.dst_addr = g.dst_addr;
    f.flags = kGoldenFlags;
    for (size_t i = 0; i < gr::uwb::twr::kMaxTimestamps; ++i)
        f.timestamps[i] = (i < g.timestamp_count) ? g.timestamps[i] : 0;
    return f;
}

std::vector<uint8_t> golden_bytes(const Golden& g)
{
    std::vector<uint8_t> v;
    BOOST_REQUIRE(gr::uwb::twr::hex_to_bytes(g.mac_payload_hex, v));
    BOOST_REQUIRE_EQUAL(v.size(), g.payload_bytes);
    return v;
}

// ---------------------------------------------------------------------------
// testdata location
// ---------------------------------------------------------------------------

const char* kGoldenRel = "twr/frame_golden_v1.json";

std::string testdata_path(const std::string& rel)
{
    const std::string from_def = std::string(UWB_TESTDATA_DIR) + "/" + rel;
    {
        std::ifstream f(from_def, std::ios::binary);
        if (f)
            return from_def;
    }
    const char* prefixes[] = {
        "../../../testdata/", "../../testdata/", "../testdata/", "testdata/",
    };
    for (const char* p : prefixes) {
        const std::string path = std::string(p) + rel;
        std::ifstream f(path, std::ios::binary);
        if (f)
            return path;
    }
    return from_def;
}

bool read_file(const std::string& path, std::string& out)
{
    std::ifstream f(path, std::ios::binary);
    if (!f)
        return false;
    std::ostringstream ss;
    ss << f.rdbuf();
    out = ss.str();
    return true;
}

// Repository root, derived from UWB_TESTDATA_DIR ("<root>/testdata").
std::string repo_root()
{
    std::string td(UWB_TESTDATA_DIR);
    while (td.size() > 1 && td.back() == '/')
        td.pop_back();
    const std::string suffix = "/testdata";
    if (td.size() >= suffix.size() &&
        td.compare(td.size() - suffix.size(), suffix.size(), suffix) == 0) {
        return td.substr(0, td.size() - suffix.size());
    }
    if (!td.empty() && td[0] == '.' && td.find('/') == std::string::npos) {
        return std::string("..") + td.substr(1);
    }
    BOOST_TEST_MESSAGE("cannot derive repo root from UWB_TESTDATA_DIR");
    return std::string();
}

// ---------------------------------------------------------------------------
// Minimal JSON reader
// ---------------------------------------------------------------------------
//
// The golden file is a machine artifact in a fixed, simple shape, so the QA
// carries its own ~120-line reader instead of taking a dependency.  It only
// understands what the golden uses: objects, flat arrays of scalars, strings
// (no escapes) and non-negative integers.  Anything unexpected makes a lookup
// FAIL -- the point is to catch a stale or hand-edited golden, not to
// tolerate a rewrite.

const size_t kNpos = static_cast<size_t>(-1);

void skip_ws(const std::string& s, size_t& i)
{
    while (i < s.size() && (s[i] == ' ' || s[i] == '\t' || s[i] == '\n' || s[i] == '\r'))
        ++i;
}

// Index of the opening quote of `"key"`, or kNpos.
size_t key_pos(const std::string& doc, const std::string& key, size_t from = 0)
{
    const std::string token = "\"" + key + "\"";
    return doc.find(token, from);
}

// Index of the first character of the value of `key`, or kNpos.
size_t value_pos(const std::string& doc, const std::string& key, size_t from = 0)
{
    size_t i = key_pos(doc, key, from);
    if (i == kNpos)
        return kNpos;
    i += key.size() + 2;
    skip_ws(doc, i);
    if (i >= doc.size() || doc[i] != ':')
        return kNpos;
    ++i;
    skip_ws(doc, i);
    return i < doc.size() ? i : kNpos;
}

std::string read_string(const std::string& doc, size_t& i, bool& ok)
{
    std::string out;
    if (i >= doc.size() || doc[i] != '"') {
        ok = false;
        return out;
    }
    ++i;
    while (i < doc.size() && doc[i] != '"') {
        if (doc[i] == '\\') {
            ok = false; // no escapes in the golden
            return std::string();
        }
        out.push_back(doc[i]);
        ++i;
    }
    if (i >= doc.size()) {
        ok = false;
        return out;
    }
    ++i;
    ok = true;
    return out;
}

std::string json_string(const std::string& doc, const std::string& key, bool& ok)
{
    const size_t p = value_pos(doc, key);
    if (p == kNpos) {
        ok = false;
        return std::string();
    }
    size_t i = p;
    return read_string(doc, i, ok);
}

uint64_t json_uint(const std::string& doc, const std::string& key, bool& ok)
{
    const size_t p = value_pos(doc, key);
    if (p == kNpos) {
        ok = false;
        return 0;
    }
    if (p + 3u < doc.size() && doc.compare(p, 4, "true") == 0) {
        return 1u;
    }
    if (p + 4u < doc.size() && doc.compare(p, 5, "false") == 0) {
        return 0u;
    }
    if (p < doc.size() && doc[p] == '"') {
        // hex string, as used for timestamps
        size_t i = p;
        const std::string hx = read_string(doc, i, ok);
        if (!ok)
            return 0;
        uint64_t v = 0;
        if (hx.empty())
            return 0;
        for (char c : hx) {
            int d;
            if (c >= '0' && c <= '9')
                d = c - '0';
            else if (c >= 'a' && c <= 'f')
                d = c - 'a' + 10;
            else if (c >= 'A' && c <= 'F')
                d = c - 'A' + 10;
            else {
                ok = false;
                return 0;
            }
            v = (v << 4) | static_cast<uint64_t>(d);
        }
        return v;
    }
    size_t i = p;
    uint64_t v = 0;
    bool any = false;
    while (i < doc.size() && doc[i] >= '0' && doc[i] <= '9') {
        v = v * 10u + static_cast<uint64_t>(doc[i] - '0');
        any = true;
        ++i;
    }
    if (!any) {
        ok = false;
        return 0;
    }
    return v;
}


// Reads `[ "a", "b" ]` into `out`.  An empty array yields an empty vector.
bool json_string_array(const std::string& doc,
                       const std::string& key,
                       std::vector<std::string>& out)
{
    out.clear();
    size_t i = value_pos(doc, key);
    if (i == kNpos || doc[i] != '[')
        return false;
    ++i;
    skip_ws(doc, i);
    if (i < doc.size() && doc[i] == ']')
        return true;
    for (;;) {
        skip_ws(doc, i);
        bool ok = false;
        const std::string s = read_string(doc, i, ok);
        if (!ok)
            return false;
        out.push_back(s);
        skip_ws(doc, i);
        if (i < doc.size() && doc[i] == ',') {
            ++i;
            continue;
        }
        return i < doc.size() && doc[i] == ']';
    }
}

// Reads `[ 1, 2 ]` into `out`, honouring hex strings.
bool json_uint_array(const std::string& doc,
                     const std::string& key,
                     std::vector<uint64_t>& out)
{
    out.clear();
    size_t i = value_pos(doc, key);
    if (i == kNpos || doc[i] != '[')
        return false;
    ++i;
    skip_ws(doc, i);
    if (i < doc.size() && doc[i] == ']')
        return true;
    for (;;) {
        skip_ws(doc, i);
        if (i >= doc.size())
            return false;
        if (doc[i] == '"') {
            bool ok = false;
            const std::string hx = read_string(doc, i, ok);
            if (!ok || hx.empty())
                return false;
            uint64_t v = 0;
            for (char c : hx) {
                int d;
                if (c >= '0' && c <= '9')
                    d = c - '0';
                else if (c >= 'a' && c <= 'f')
                    d = c - 'a' + 10;
                else if (c >= 'A' && c <= 'F')
                    d = c - 'A' + 10;
                else
                    return false;
                v = (v << 4) | static_cast<uint64_t>(d);
            }
            out.push_back(v);
        } else {
            uint64_t v = 0;
            bool any = false;
            while (i < doc.size() && doc[i] >= '0' && doc[i] <= '9') {
                v = v * 10u + static_cast<uint64_t>(doc[i] - '0');
                any = true;
                ++i;
            }
            if (!any)
                return false;
            out.push_back(v);
        }
        skip_ws(doc, i);
        if (i < doc.size() && doc[i] == ',') {
            ++i;
            continue;
        }
        return i < doc.size() && doc[i] == ']';
    }
}

// Parses "00:01 01:00 ..." into offset -> byte.
bool parse_byte_map(const std::string& s, std::vector<std::pair<size_t, uint8_t>>& out)
{
    out.clear();
    std::istringstream ss(s);
    std::string tok;
    while (ss >> tok) {
        const size_t colon = tok.find(':');
        if (colon == std::string::npos || colon + 3 != tok.size())
            return false;
        const std::string off = tok.substr(0, colon);
        const std::string val = tok.substr(colon + 1);
        size_t o = 0;
        for (char c : off) {
            if (c < '0' || c > '9')
                return false;
            o = o * 10u + static_cast<size_t>(c - '0');
        }
        unsigned v = 0;
        for (char c : val) {
            unsigned d;
            if (c >= '0' && c <= '9')
                d = static_cast<unsigned>(c - '0');
            else if (c >= 'a' && c <= 'f')
                d = static_cast<unsigned>(c - 'a' + 10);
            else if (c >= 'A' && c <= 'F')
                d = static_cast<unsigned>(c - 'A' + 10);
            else
                return false;
            v = v * 16u + d;
        }
        if (v > 0xffu)
            return false;
        out.emplace_back(o, static_cast<uint8_t>(v));
    }
    return true;
}

// Scopes the `"cases"` array into one substring per case, keyed by `"name"`.
std::vector<std::pair<std::string, std::string>>
split_cases(const std::string& doc, size_t& cases_begin)
{
    std::vector<std::pair<std::string, std::string>> out;
    cases_begin = value_pos(doc, "cases");
    if (cases_begin == kNpos)
        return out;
    const size_t end = doc.size();
    size_t i = cases_begin;
    for (;;) {
        const size_t kp = key_pos(doc, "name", i);
        if (kp == kNpos || kp >= end)
            break;
        const size_t vp = value_pos(doc, "name", kp);
        if (vp == kNpos)
            break;
        size_t ni = vp;
        bool ok = false;
        const std::string name = read_string(doc, ni, ok);
        if (!ok)
            break;
        const size_t next_key = key_pos(doc, "name", ni);
        const size_t stop = (next_key == kNpos) ? end : next_key;
        out.emplace_back(name, doc.substr(kp, stop - kp));
        i = stop;
    }
    return out;
}


// ---------------------------------------------------------------------------
// Whole-file / whole-tree scans
// ---------------------------------------------------------------------------

// Blanks out // and /* */ comments and string/char literal bodies while
// preserving every byte offset and every newline, so line numbers stay valid
// and a token mentioned in prose cannot be mistaken for code.
std::string strip_comments(const std::string& in)
{
    std::string out(in);
    enum State { Code, Line, Block, Str, Chr } st = Code;
    for (size_t i = 0; i < in.size(); ++i) {
        const char c = in[i];
        const char nx = (i + 1 < in.size()) ? in[i + 1] : '\0';
        switch (st) {
        case Code:
            if (c == '/' && nx == '/') {
                out[i] = ' ';
                out[i + 1] = ' ';
                ++i;
                st = Line;
            } else if (c == '/' && nx == '*') {
                out[i] = ' ';
                out[i + 1] = ' ';
                ++i;
                st = Block;
            } else if (c == '"') {
                out[i] = ' ';
                st = Str;
            } else if (c == '\'') {
                out[i] = ' ';
                st = Chr;
            }
            break;
        case Line:
            if (c == '\n')
                st = Code;
            else
                out[i] = ' ';
            break;
        case Block:
            if (c == '*' && nx == '/') {
                out[i] = ' ';
                out[i + 1] = ' ';
                ++i;
                st = Code;
            } else if (c != '\n') {
                out[i] = ' ';
            }
            break;
        case Str:
        case Chr:
            if (c == '\\' && nx != '\0') {
                out[i] = ' ';
                if (nx != '\n')
                    out[i + 1] = ' ';
                ++i;
            } else if ((st == Str && c == '"') || (st == Chr && c == '\'')) {
                out[i] = ' ';
                st = Code;
            } else if (c != '\n') {
                out[i] = ' ';
            }
            break;
        }
    }
    return out;
}

std::vector<std::string> list_files(const std::string& dir, const char* suffix)
{
    std::vector<std::string> out;
    DIR* d = opendir(dir.c_str());
    if (d == nullptr)
        return out;
    const size_t slen = std::strlen(suffix);
    for (struct dirent* e = readdir(d); e != nullptr; e = readdir(d)) {
        const std::string n(e->d_name);
        if (n.size() <= slen)
            continue;
        if (n.compare(n.size() - slen, slen, suffix) != 0)
            continue;
        out.push_back(dir + "/" + n);
    }
    closedir(d);
    return out;
}

} // namespace

// ===========================================================================
// 1. Layout constants and the frozen table
// ===========================================================================

BOOST_AUTO_TEST_CASE(uwb_twr_frame_layout_constants)
{
    using namespace gr::uwb::twr;
    BOOST_CHECK_EQUAL(kFrameVersion, 0x01);
    BOOST_CHECK_EQUAL(kOffVersion, 0u);
    BOOST_CHECK_EQUAL(kOffFunctionCode, 1u);
    BOOST_CHECK_EQUAL(kOffSessionId, 2u);
    BOOST_CHECK_EQUAL(kOffSeq, 4u);
    BOOST_CHECK_EQUAL(kOffPanId, 6u);
    BOOST_CHECK_EQUAL(kOffSrcAddr, 8u);
    BOOST_CHECK_EQUAL(kOffDstAddr, 10u);
    BOOST_CHECK_EQUAL(kOffFlags, 12u);
    BOOST_CHECK_EQUAL(kOffTimestamps, 14u);
    BOOST_CHECK_EQUAL(kFrameHeaderBytes, 14u);
    BOOST_CHECK_EQUAL(kMaxTimestamps, 3u);
    BOOST_CHECK_EQUAL(kMaxFrameBytes, 38u);
    BOOST_CHECK_EQUAL(kFrameFcsBytes, 2u);
    BOOST_CHECK_EQUAL(kPhyMaxPsduBytes, 127u);
    // The mirror above must not drift from the PHY constant it copies.
    static_assert(kPhyMaxPsduBytes == 127,
                  "UWBR_FK3WS: PHY max PSDU is 127 bytes today; if the PHY "
                  "changed, update this mirror and the golden file together");
    BOOST_CHECK_EQUAL(gr::uwb::radar_meta::kMaxPsduBytes, kPhyMaxPsduBytes);

    // The frozen function codes and timestamp counts.
    BOOST_CHECK_EQUAL(static_cast<int>(FrameType::Poll), 0x00);
    BOOST_CHECK_EQUAL(static_cast<int>(FrameType::Response), 0x01);
    BOOST_CHECK_EQUAL(static_cast<int>(FrameType::Final), 0x02);
    BOOST_CHECK_EQUAL(static_cast<int>(FrameType::Report), 0x03);
    BOOST_CHECK_EQUAL(frame_type_timestamp_count(FrameType::Poll), 0u);
    BOOST_CHECK_EQUAL(frame_type_timestamp_count(FrameType::Response), 2u);
    BOOST_CHECK_EQUAL(frame_type_timestamp_count(FrameType::Final), 3u);

    // Report is reserved in phase 1: known code point, no layout.
    BOOST_CHECK(frame_type_is_implemented(FrameType::Poll));
    BOOST_CHECK(frame_type_is_implemented(FrameType::Response));
    BOOST_CHECK(frame_type_is_implemented(FrameType::Final));
    BOOST_CHECK(!frame_type_is_implemented(FrameType::Report));
    BOOST_CHECK(frame_layout(FrameType::Report) == nullptr);

    FrameType parsed = FrameType::Report;
    BOOST_CHECK(frame_type_from_wire(0x03, parsed));
    BOOST_CHECK(parsed == FrameType::Report);
    BOOST_CHECK(!frame_type_from_wire(0x04, parsed));
    BOOST_CHECK(!frame_type_from_wire(0xFF, parsed));

    // Flags: bit0 ranging, bit1 sts_present, 2..15 reserved.
    BOOST_CHECK_EQUAL(kFlagRanging, 0x0001);
    BOOST_CHECK_EQUAL(kFlagStsPresent, 0x0002);
    BOOST_CHECK_EQUAL(kFlagReservedMask, 0xFFFC);
    BOOST_CHECK(flags_ranging(make_flags(true, false)));
    BOOST_CHECK(!flags_ranging(make_flags(false, false)));
    BOOST_CHECK(flags_sts_present(make_flags(true, true)));
    BOOST_CHECK(!flags_has_reserved_bits(make_flags(true, true)));
    for (unsigned b = 2; b < 16; ++b) {
        BOOST_CHECK(flags_has_reserved_bits(static_cast<uint16_t>(1u << b)));
    }
}

// ===========================================================================
// 2. Byte-exact goldens, byte order asserted per offset
// ===========================================================================

BOOST_AUTO_TEST_CASE(uwb_twr_frame_golden_bytes_and_offsets)
{
    using namespace gr::uwb::twr;
    const FrameProfile p = default_profile();

    for (size_t gi = 0; gi < kNumGoldens; ++gi) {
        const Golden& g = kGoldens[gi];
        const std::vector<uint8_t> want = golden_bytes(g);
        const Frame f = make_golden_frame(g);

        std::vector<uint8_t> got;
        std::string err;
        FrameError code = FrameError::None;
        BOOST_TEST_MESSAGE("golden " << g.name << " expected " << bytes_to_hex(want));
        BOOST_REQUIRE_MESSAGE(encode(f, p, got, err, &code), err);
        BOOST_CHECK_EQUAL(code, FrameError::None);
        BOOST_REQUIRE_EQUAL(got.size(), g.payload_bytes);

        // Byte-exact, and the byte order is little-endian at every offset.
        BOOST_REQUIRE_EQUAL(got.size(), want.size());
        for (size_t i = 0; i < want.size(); ++i) {
            BOOST_CHECK_MESSAGE(got[i] == want[i],
                                g.name << " byte " << i << ": got 0x"
                                       << std::hex << static_cast<int>(got[i])
                                       << " want 0x" << static_cast<int>(want[i]));
        }

        // Individual header fields at their frozen offsets.
        BOOST_CHECK_EQUAL(got[kOffVersion], 0x01);
        BOOST_CHECK_EQUAL(got[kOffFunctionCode], static_cast<uint8_t>(g.type));
        BOOST_CHECK_EQUAL(got[kOffSessionId], static_cast<uint8_t>(g.session_id & 0xFF));
        BOOST_CHECK_EQUAL(got[kOffSessionId + 1],
                          static_cast<uint8_t>((g.session_id >> 8) & 0xFF));
        BOOST_CHECK_EQUAL(got[kOffSeq], static_cast<uint8_t>(g.seq & 0xFF));
        BOOST_CHECK_EQUAL(got[kOffSeq + 1], static_cast<uint8_t>((g.seq >> 8) & 0xFF));
        BOOST_CHECK_EQUAL(got[kOffPanId], static_cast<uint8_t>(kGoldenPan & 0xFF));
        BOOST_CHECK_EQUAL(got[kOffPanId + 1],
                          static_cast<uint8_t>((kGoldenPan >> 8) & 0xFF));
        BOOST_CHECK_EQUAL(got[kOffSrcAddr], static_cast<uint8_t>(g.src_addr & 0xFF));
        BOOST_CHECK_EQUAL(got[kOffSrcAddr + 1],
                          static_cast<uint8_t>((g.src_addr >> 8) & 0xFF));
        BOOST_CHECK_EQUAL(got[kOffDstAddr], static_cast<uint8_t>(g.dst_addr & 0xFF));
        BOOST_CHECK_EQUAL(got[kOffDstAddr + 1],
                          static_cast<uint8_t>((g.dst_addr >> 8) & 0xFF));
        BOOST_CHECK_EQUAL(got[kOffFlags], 0x01);
        BOOST_CHECK_EQUAL(got[kOffFlags + 1], 0x00);

        // Timestamp bytes: least significant first, 5 B per field, wire order.
        const FrameLayout* l = frame_layout(g.type);
        BOOST_REQUIRE(l != nullptr);
        for (size_t k = 0; k < g.timestamp_count; ++k) {
            for (size_t b = 0; b < 5; ++b) {
                const uint8_t expect =
                    static_cast<uint8_t>((g.timestamps[k] >> (8 * b)) & 0xFF);
                BOOST_CHECK_MESSAGE(
                    got[kOffTimestamps + k * 5 + b] == expect,
                    g.name << " timestamp " << l->order[k] << " byte " << b);
            }
        }
        // The frozen sizes.
        BOOST_CHECK_EQUAL(got.size(), frame_length_for(g.type, p));
    }

    // The three frozen frames spelled out, so a reader of the failure log
    // sees the actual expected bytes.
    BOOST_CHECK_EQUAL(std::string(kGoldens[0].mac_payload_hex),
                      "0100efbe02013412010002000100");
    BOOST_CHECK_EQUAL(std::string(kGoldens[1].mac_payload_hex),
                      "0101efbe020134120200010001008967452301badcfe0000");
    BOOST_CHECK_EQUAL(std::string(kGoldens[2].mac_payload_hex),
                      "0102efbe02013412010002000100554433221155667788990504030201");
    BOOST_CHECK_EQUAL(kGoldens[0].payload_bytes, 14u);
    BOOST_CHECK_EQUAL(kGoldens[1].payload_bytes, 24u);
    BOOST_CHECK_EQUAL(kGoldens[2].payload_bytes, 29u);
}

// ===========================================================================
// 3. The golden JSON is data this QA actually reads and asserts against
// ===========================================================================

BOOST_AUTO_TEST_CASE(uwb_twr_frame_golden_json_is_live_test_data)
{
    using namespace gr::uwb::twr;
    const std::string path = testdata_path(kGoldenRel);
    std::string doc;
    BOOST_REQUIRE_MESSAGE(read_file(path, doc),
                          "golden file not found: " << path
                                                     << " (set UWB_TESTDATA_DIR)");
    BOOST_TEST_MESSAGE("golden file: " << path << " (" << doc.size() << " bytes)");

    // Header.
    bool ok = false;
    BOOST_CHECK_EQUAL(json_string(doc, "schema", ok), std::string("uwb.twr.frame_golden"));
    BOOST_REQUIRE(ok);
    BOOST_CHECK_EQUAL(json_uint(doc, "schema_version", ok), 1u);
    BOOST_REQUIRE(ok);

    // Profile block: everything before the first case.
    size_t cases_begin = 0;
    const auto cases = split_cases(doc, cases_begin);
    BOOST_REQUIRE(cases_begin != kNpos);
    const std::string profile = doc.substr(0, cases_begin);

    BOOST_CHECK_EQUAL(json_uint(profile, "version", ok), 1u);
    BOOST_REQUIRE(ok);
    BOOST_CHECK_EQUAL(json_uint(profile, "timestamp_bits", ok), 40u);
    BOOST_REQUIRE(ok);
    BOOST_CHECK_EQUAL(json_uint(profile, "timestamp_unit_hz", ok), 737280000u);
    BOOST_REQUIRE(ok);
    BOOST_CHECK_EQUAL(json_uint(profile, "max_psdu_bytes", ok), 127u);
    BOOST_REQUIRE(ok);
    BOOST_CHECK_EQUAL(json_uint(profile, "fcs_bytes", ok), 2u);
    BOOST_REQUIRE(ok);
    BOOST_CHECK_EQUAL(json_string(profile, "fcs_layer", ok), std::string("modulation_layer"));
    BOOST_REQUIRE(ok);
    BOOST_CHECK_EQUAL(json_uint(profile, "fcs_in_codec", ok), 0u);
    BOOST_REQUIRE(ok);

    const FrameProfile p = default_profile();
    BOOST_CHECK_EQUAL(p.timestamp_bits, 40u);
    BOOST_CHECK_EQUAL(static_cast<uint64_t>(p.timestamp_unit_hz), 737280000ull);
    BOOST_CHECK_EQUAL(p.max_psdu_bytes, 127u);
    BOOST_REQUIRE(p.fcs_appended_by_modulation_layer);

    // Same number of cases on both sides: a stale or extra golden fails.
    BOOST_REQUIRE_EQUAL(cases.size(), kNumGoldens);

    for (size_t gi = 0; gi < kNumGoldens; ++gi) {
        const Golden& g = kGoldens[gi];
        const std::string& blk = cases[gi].second;
        BOOST_CHECK_EQUAL(cases[gi].first, std::string(g.name));
        BOOST_TEST_MESSAGE("golden json case " << cases[gi].first);

        // --- code point and declared size -------------------------------
        FrameType fc = FrameType::Report;
        BOOST_REQUIRE(frame_type_from_wire(
            static_cast<uint8_t>(json_uint(blk, "function_code", ok)), fc));
        BOOST_REQUIRE(ok);
        BOOST_CHECK(fc == g.type);
        BOOST_CHECK_EQUAL(json_string(blk, "function_code_name", ok),
                          std::string(frame_type_to_string(g.type)));
        BOOST_REQUIRE(ok);
        BOOST_CHECK_EQUAL(json_uint(blk, "timestamp_count", ok),
                          static_cast<uint64_t>(g.timestamp_count));
        BOOST_REQUIRE(ok);
        BOOST_CHECK_EQUAL(json_uint(blk, "mac_payload_length", ok),
                          static_cast<uint64_t>(g.payload_bytes));
        BOOST_REQUIRE(ok);
        // Exactly one FCS layer: +2, never more, never zero.
        BOOST_CHECK_EQUAL(json_uint(blk, "psdu_with_fcs_length", ok),
                          static_cast<uint64_t>(g.payload_bytes + kFrameFcsBytes));
        BOOST_REQUIRE(ok);

        // --- header fields -----------------------------------------------
        const Frame f = make_golden_frame(g);
        BOOST_CHECK_EQUAL(json_uint(blk, "version", ok), static_cast<uint64_t>(f.version));
        BOOST_REQUIRE(ok);
        BOOST_CHECK_EQUAL(json_uint(blk, "session_id", ok), f.session_id);
        BOOST_REQUIRE(ok);
        BOOST_CHECK_EQUAL(json_uint(blk, "seq", ok), f.seq);
        BOOST_REQUIRE(ok);
        BOOST_CHECK_EQUAL(json_uint(blk, "pan_id", ok), f.pan_id);
        BOOST_REQUIRE(ok);
        BOOST_CHECK_EQUAL(json_uint(blk, "src_addr", ok), f.src_addr);
        BOOST_REQUIRE(ok);
        BOOST_CHECK_EQUAL(json_uint(blk, "dst_addr", ok), f.dst_addr);
        BOOST_REQUIRE(ok);
        BOOST_CHECK_EQUAL(json_uint(blk, "flags", ok), f.flags);
        BOOST_REQUIRE(ok);

        // --- timestamps: names, decimal and hex must all agree ----------
        std::vector<std::string> names;
        std::vector<uint64_t> dec;
        std::vector<uint64_t> hx;
        BOOST_REQUIRE(json_string_array(blk, "timestamp_names", names));
        BOOST_REQUIRE(json_uint_array(blk, "timestamp_values_dec", dec));
        BOOST_REQUIRE(json_uint_array(blk, "timestamp_values_hex", hx));
        BOOST_REQUIRE_EQUAL(names.size(), g.timestamp_count);
        BOOST_REQUIRE_EQUAL(dec.size(), g.timestamp_count);
        BOOST_REQUIRE_EQUAL(hx.size(), g.timestamp_count);
        const FrameLayout* l = frame_layout(g.type);
        BOOST_REQUIRE(l != nullptr);
        for (size_t k = 0; k < g.timestamp_count; ++k) {
            BOOST_CHECK_EQUAL(names[k], std::string(timestamp_field_name(l->order[k])));
            BOOST_CHECK_MESSAGE(dec[k] == hx[k],
                                g.name << " timestamp " << k
                                       << ": dec " << dec[k] << " != hex " << hx[k]);
            BOOST_CHECK_EQUAL(dec[k], g.timestamps[k]);
            BOOST_CHECK(timestamp_fits(p, dec[k]));
        }

        // --- the bytes: JSON == hand table == codec ----------------------
        std::vector<uint8_t> json_bytes;
        BOOST_REQUIRE_MESSAGE(hex_to_bytes(json_string(blk, "mac_payload_hex", ok), json_bytes),
                              g.name << ": bad mac_payload_hex");
        BOOST_REQUIRE(ok);
        const std::vector<uint8_t> hand_bytes = golden_bytes(g);
        std::vector<uint8_t> codec_bytes;
        std::string err;
        BOOST_REQUIRE_MESSAGE(encode(f, p, codec_bytes, err), err);

        BOOST_CHECK_MESSAGE(json_bytes == hand_bytes,
                            g.name << ": golden JSON bytes differ from the hand table");
        BOOST_CHECK_MESSAGE(codec_bytes == hand_bytes,
                            g.name << ": codec bytes differ from the hand table");
        BOOST_CHECK_EQUAL(bytes_to_hex(codec_bytes), bytes_to_hex(json_bytes));

        // --- per-offset map, asserted byte by byte -----------------------
        std::vector<std::pair<size_t, uint8_t>> bmap;
        BOOST_REQUIRE_MESSAGE(
            parse_byte_map(json_string(blk, "byte_map", ok), bmap), g.name << ": bad byte_map");
        BOOST_REQUIRE(ok);
        BOOST_REQUIRE_EQUAL(bmap.size(), codec_bytes.size());
        for (size_t k = 0; k < bmap.size(); ++k) {
            BOOST_REQUIRE_EQUAL(bmap[k].first, k); // dense, offset-ordered
            BOOST_CHECK_MESSAGE(codec_bytes[bmap[k].first] == bmap[k].second,
                                g.name << " offset " << bmap[k].first << ": json 0x"
                                       << std::hex
                                       << static_cast<int>(bmap[k].second) << " codec 0x"
                                       << static_cast<int>(codec_bytes[bmap[k].first]));
        }
    }
}

// ===========================================================================
// 4. Round trip
// ===========================================================================

BOOST_AUTO_TEST_CASE(uwb_twr_frame_round_trip)
{
    using namespace gr::uwb::twr;
    const FrameProfile p = default_profile();

    for (size_t gi = 0; gi < kNumGoldens; ++gi) {
        const Golden& g = kGoldens[gi];
        const Frame f = make_golden_frame(g);

        std::vector<uint8_t> bytes;
        std::string err;
        BOOST_REQUIRE(encode(f, p, bytes, err));

        Frame back;
        BOOST_REQUIRE_MESSAGE(decode(bytes, p, back, err), err);
        BOOST_CHECK(back == f);
        BOOST_CHECK_EQUAL(back.version, f.version);
        BOOST_CHECK(back.function_code == f.function_code);
        BOOST_CHECK_EQUAL(back.session_id, f.session_id);
        BOOST_CHECK_EQUAL(back.seq, f.seq);
        BOOST_CHECK_EQUAL(back.pan_id, f.pan_id);
        BOOST_CHECK_EQUAL(back.src_addr, f.src_addr);
        BOOST_CHECK_EQUAL(back.dst_addr, f.dst_addr);
        BOOST_CHECK_EQUAL(back.flags, f.flags);
        BOOST_CHECK_EQUAL(back.timestamp_count(), g.timestamp_count);
        for (size_t k = 0; k < g.timestamp_count; ++k)
            BOOST_CHECK_EQUAL(back.timestamps[k], g.timestamps[k]);

        // decode -> encode reproduces the identical bytes.
        std::vector<uint8_t> again;
        BOOST_REQUIRE(encode(back, p, again, err));
        BOOST_CHECK(again == bytes);
    }

    // Every 16-bit / 40-bit bit pattern in the header region survives.
    for (uint32_t v = 0; v < 0x10000u; v += 0x37u) {
        Frame f;
        f.function_code = FrameType::Response;
        f.session_id = static_cast<uint16_t>(v);
        f.seq = static_cast<uint16_t>(0xFFFFu - v);
        f.pan_id = 0x1234;
        f.src_addr = kAddrA;
        f.dst_addr = kAddrB;
        f.flags = kGoldenFlags;
        f.timestamps[0] = v;
        f.timestamps[1] = 0xFFFFFFFFFFull - v;
        std::vector<uint8_t> bytes;
        std::string err;
        BOOST_REQUIRE(encode(f, p, bytes, err));
        Frame back;
        BOOST_REQUIRE(decode(bytes, p, back, err));
        BOOST_CHECK(back == f);
    }
}

// ===========================================================================
// 5. Refusals
// ===========================================================================

BOOST_AUTO_TEST_CASE(uwb_twr_frame_report_is_rejected_not_implemented)
{
    using namespace gr::uwb::twr;
    const FrameProfile p = default_profile();

    // Encode side.
    Frame f = make_golden_frame(kGoldens[0]);
    f.function_code = FrameType::Report;
    std::vector<uint8_t> out;
    std::string err;
    FrameError code = FrameError::None;
    BOOST_CHECK(!encode(f, p, out, err, &code));
    BOOST_CHECK_EQUAL(code, FrameError::NotImplemented);
    BOOST_CHECK(err.find("not_implemented") == 0);
    BOOST_CHECK(out.empty());
    BOOST_CHECK_EQUAL(frame_error_to_exchange_status(code), ExchangeStatus::Unsupported);
    BOOST_CHECK(!exchange_status_yields_range(frame_error_to_exchange_status(code)));

    // Decode side: a Report on the wire is refused even though it parses as a
    // known code point.
    std::vector<uint8_t> bytes = golden_bytes(kGoldens[0]);
    bytes[kOffFunctionCode] = 0x03;
    Frame back;
    BOOST_CHECK(!decode(bytes, p, back, err, &code));
    BOOST_CHECK_EQUAL(code, FrameError::NotImplemented);
    BOOST_CHECK_EQUAL(frame_error_to_exchange_status(code), ExchangeStatus::Unsupported);

    // The reserved slot has no layout and therefore no length.
    std::string len_err;
    BOOST_CHECK_EQUAL(frame_length_for_checked(FrameType::Report, p, len_err, &code), 0u);
    BOOST_CHECK_EQUAL(code, FrameError::NotImplemented);
}

BOOST_AUTO_TEST_CASE(uwb_twr_frame_unknown_version_rejected)
{
    using namespace gr::uwb::twr;
    const FrameProfile p = default_profile();

    for (int v = 0; v < 256; ++v) {
        if (v == 0x01)
            continue;
        std::vector<uint8_t> bytes = golden_bytes(kGoldens[1]);
        bytes[kOffVersion] = static_cast<uint8_t>(v);
        Frame back;
        std::string err;
        FrameError code = FrameError::None;
        BOOST_CHECK(!decode(bytes, p, back, err, &code));
        BOOST_CHECK_MESSAGE(code == FrameError::BadVersion,
                            "version 0x" << std::hex << v << " -> "
                                         << frame_error_to_string(code));
        // A failed decode must not leave something that looks usable.
        BOOST_CHECK_EQUAL(back.version, 0);
    }

    Frame f = make_golden_frame(kGoldens[0]);
    f.version = 0x02;
    std::vector<uint8_t> out;
    std::string err;
    FrameError code = FrameError::None;
    BOOST_CHECK(!encode(f, p, out, err, &code));
    BOOST_CHECK_EQUAL(code, FrameError::BadVersion);

    // A profile that declares a version other than 0x01 is itself rejected.
    FrameProfile bad = default_profile();
    bad.version = 0x02;
    BOOST_CHECK(!frame_profile_validate(bad, err, &code));
    BOOST_CHECK_EQUAL(code, FrameError::BadProfile);
}

BOOST_AUTO_TEST_CASE(uwb_twr_frame_unknown_function_code_rejected)
{
    using namespace gr::uwb::twr;
    const FrameProfile p = default_profile();

    const uint8_t bad_codes[] = { 0x04, 0x05, 0x10, 0x7F, 0x80, 0xFE, 0xFF };
    for (uint8_t c : bad_codes) {
        std::vector<uint8_t> bytes = golden_bytes(kGoldens[0]);
        bytes[kOffFunctionCode] = c;
        Frame back;
        std::string err;
        FrameError code = FrameError::None;
        BOOST_CHECK(!decode(bytes, p, back, err, &code));
        BOOST_CHECK_MESSAGE(code == FrameError::BadFunctionCode,
                            "function code 0x" << std::hex << static_cast<int>(c)
                                                 << " -> "
                                                 << frame_error_to_string(code));
        BOOST_CHECK_EQUAL(frame_error_to_exchange_status(code),
                          ExchangeStatus::PhyDecodeFailed);
    }

    // Encode side: an out-of-range enum value is refused, not truncated to a
    // valid code point.
    Frame f = make_golden_frame(kGoldens[0]);
    for (int c : { 4, 5, 42, 200, 255 }) {
        f.function_code = static_cast<FrameType>(c);
        std::vector<uint8_t> out;
        std::string err;
        FrameError code = FrameError::None;
        BOOST_CHECK(!encode(f, p, out, err, &code));
        BOOST_CHECK_EQUAL(code, FrameError::BadFunctionCode);
    }
}

BOOST_AUTO_TEST_CASE(uwb_twr_frame_reserved_flag_bits_rejected)
{
    using namespace gr::uwb::twr;
    const FrameProfile p = default_profile();

    for (unsigned b = 2; b < 16; ++b) {
        const uint16_t flags = static_cast<uint16_t>(kGoldenFlags | (1u << b));
        // Decode side.
        std::vector<uint8_t> bytes = golden_bytes(kGoldens[0]);
        bytes[kOffFlags] = static_cast<uint8_t>(flags & 0xFF);
        bytes[kOffFlags + 1] = static_cast<uint8_t>((flags >> 8) & 0xFF);
        Frame back;
        std::string err;
        FrameError code = FrameError::None;
        BOOST_CHECK(!decode(bytes, p, back, err, &code));
        BOOST_CHECK_MESSAGE(code == FrameError::ReservedFlags,
                            "flags bit " << b << " -> " << frame_error_to_string(code));
        // Encode side.
        Frame f = make_golden_frame(kGoldens[0]);
        f.flags = flags;
        std::vector<uint8_t> out;
        BOOST_CHECK(!encode(f, p, out, err, &code));
        BOOST_CHECK_EQUAL(code, FrameError::ReservedFlags);
    }

    // Bits 0 and 1 alone are not "reserved".
    std::vector<uint8_t> bytes = golden_bytes(kGoldens[0]);
    bytes[kOffFlags] = 0x00; // ranging clear, sts clear
    bytes[kOffFlags + 1] = 0x00;
    Frame back;
    std::string err;
    FrameError code = FrameError::None;
    BOOST_REQUIRE(decode(bytes, p, back, err, &code));
    BOOST_CHECK_EQUAL(back.flags, 0u);
    BOOST_CHECK(!back.ranging());
    BOOST_CHECK(!back.sts_present());
}

BOOST_AUTO_TEST_CASE(uwb_twr_frame_sts_present_rejected)
{
    using namespace gr::uwb::twr;
    const FrameProfile p = default_profile();

    std::vector<uint8_t> bytes = golden_bytes(kGoldens[1]);
    bytes[kOffFlags] = static_cast<uint8_t>(kGoldenFlags | kFlagStsPresent); // 0x03
    Frame back;
    std::string err;
    FrameError code = FrameError::None;
    BOOST_CHECK(!decode(bytes, p, back, err, &code));
    BOOST_CHECK_EQUAL(code, FrameError::StsUnsupported);
    // An explicit refusal, never a silent ignore of the bit.
    BOOST_CHECK_EQUAL(frame_error_to_exchange_status(code), ExchangeStatus::Unsupported);
    BOOST_CHECK(!exchange_status_yields_range(frame_error_to_exchange_status(code)));
    BOOST_CHECK(back.version == 0);

    Frame f = make_golden_frame(kGoldens[1]);
    f.flags = make_flags(true, true);
    std::vector<uint8_t> out;
    BOOST_CHECK(!encode(f, p, out, err, &code));
    BOOST_CHECK_EQUAL(code, FrameError::StsUnsupported);

    // sts_present with ranging clear is still refused.
    f.flags = make_flags(false, true);
    BOOST_CHECK(!encode(f, p, out, err, &code));
    BOOST_CHECK_EQUAL(code, FrameError::StsUnsupported);
}

// ===========================================================================
// 6. Timestamp width, wrap boundary, out of range
// ===========================================================================

BOOST_AUTO_TEST_CASE(uwb_twr_frame_timestamp_out_of_range_rejected)
{
    using namespace gr::uwb::twr;
    const FrameProfile p = default_profile();

    const uint64_t ts40_max = (static_cast<uint64_t>(1) << 40) - 1;
    BOOST_CHECK_EQUAL(timestamp_max_value(p), ts40_max);
    BOOST_CHECK_EQUAL(timestamp_bytes(p), 5u);
    BOOST_CHECK(timestamp_fits(p, ts40_max));
    BOOST_CHECK(!timestamp_fits(p, ts40_max + 1)); // 2^40

    // Encode: the first value that does not fit is refused, in every slot.
    for (size_t k = 0; k < 3; ++k) {
        Frame f = make_golden_frame(kGoldens[2]); // Final carries 3
        f.timestamps[k] = ts40_max + 1;
        std::vector<uint8_t> out;
        std::string err;
        FrameError code = FrameError::None;
        BOOST_CHECK(!encode(f, p, out, err, &code));
        BOOST_CHECK_MESSAGE(code == FrameError::TimestampOutOfRange,
                            "slot " << k << " -> " << frame_error_to_string(code));
        BOOST_CHECK(out.empty());
        BOOST_CHECK_EQUAL(frame_error_to_exchange_status(code),
                          ExchangeStatus::WrongPeer);
    }

    // A 32-bit profile refuses anything above 2^32-1 -- proving the width is a
    // profile parameter and not a hard-wired constant (REQ-TIME-04).
    FrameProfile p32 = default_profile();
    p32.timestamp_bits = 32;
    BOOST_CHECK_EQUAL(timestamp_max_value(p32), 0xFFFFFFFFull);
    Frame f = make_golden_frame(kGoldens[1]);
    f.timestamps[0] = 0x100000000ull;
    std::vector<uint8_t> out;
    std::string err;
    FrameError code = FrameError::None;
    BOOST_CHECK(!encode(f, p32, out, err, &code));
    BOOST_CHECK_EQUAL(code, FrameError::TimestampOutOfRange);

    // A 64-bit profile has no such limit, and 0xFFFFFFFFFF still fits.
    FrameProfile p64 = default_profile();
    p64.timestamp_bits = 64;
    BOOST_CHECK_EQUAL(timestamp_max_value(p64), ~static_cast<uint64_t>(0));
    f.timestamps[0] = 0xFFFFFFFFFFull;
    BOOST_REQUIRE(encode(f, p64, out, err, &code));
    Frame back;
    BOOST_REQUIRE(decode(out, p64, back, err));
    BOOST_CHECK_EQUAL(back.t2B(), 0xFFFFFFFFFFull);

    // An unused slot must be zero: a stale instant can never ride along.
    Frame poll = make_golden_frame(kGoldens[0]);
    poll.timestamps[0] = 1;
    BOOST_CHECK(!encode(poll, p, out, err, &code));
    BOOST_CHECK_EQUAL(code, FrameError::ReservedTimestampNonZero);
    poll.timestamps[0] = 0;
    poll.timestamps[2] = 1;
    BOOST_CHECK(!encode(poll, p, out, err, &code));
    BOOST_CHECK_EQUAL(code, FrameError::ReservedTimestampNonZero);
}

BOOST_AUTO_TEST_CASE(uwb_twr_frame_timestamp_wrap_boundary)
{
    using namespace gr::uwb::twr;
    const FrameProfile p = default_profile();

    // Exact values at both ends of the 40-bit range round trip.  The bounds
    // are written as shifts so there is no chance of miscounting hex digits.
    const uint64_t ts40_max = (static_cast<uint64_t>(1) << 40) - 1;
    BOOST_CHECK_EQUAL(ts40_max, 0xFFFFFFFFFFull);
    const uint64_t vals[] = {
        0,
        1,
        (static_cast<uint64_t>(1) << 39) - 1, // 2^39-1
        (static_cast<uint64_t>(1) << 39),     // 2^39, the top bit of the field
        ts40_max,
        ts40_max - 1
    };
    for (uint64_t v : vals) {
        Frame f = make_golden_frame(kGoldens[2]);
        f.timestamps[0] = v;
        f.timestamps[1] = ts40_max;
        f.timestamps[2] = ts40_max - 1;
        std::vector<uint8_t> out;
        std::string err;
        BOOST_REQUIRE_MESSAGE(encode(f, p, out, err), err);
        BOOST_CHECK_EQUAL(out.size(), 29u);
        Frame back;
        BOOST_REQUIRE_MESSAGE(decode(out, p, back, err), err);
        // Unambiguous: the exact value comes back, not a truncated or
        // wrapped one.
        BOOST_CHECK_EQUAL(back.t1A(), v);
        BOOST_CHECK_EQUAL(back.t4A(), ts40_max);
        BOOST_CHECK_EQUAL(back.t5A(), ts40_max - 1);
    }

    // One past the end is refused in every direction.
    const uint64_t overs[] = { ts40_max + 1,
                               ts40_max + 2,
                               ~static_cast<uint64_t>(0) };
    for (uint64_t over : overs) {
        Frame f = make_golden_frame(kGoldens[1]);
        f.timestamps[1] = over;
        std::vector<uint8_t> out;
        std::string err;
        FrameError code = FrameError::None;
        BOOST_CHECK(!encode(f, p, out, err, &code));
        BOOST_CHECK_EQUAL(code, FrameError::TimestampOutOfRange);
    }
}

BOOST_AUTO_TEST_CASE(uwb_twr_frame_alternate_profile_widths)
{
    using namespace gr::uwb::twr;

    struct Width {
        uint8_t bits;
        size_t poll;
        size_t response;
        size_t final_frame;
    };
    const Width widths[] = { { 8, 14, 16, 17 },
                             { 16, 14, 18, 20 },
                             { 32, 14, 22, 26 },
                             { 40, 14, 24, 29 },
                             { 64, 14, 30, 38 } };

    for (const Width& w : widths) {
        FrameProfile p = default_profile();
        p.timestamp_bits = w.bits;
        std::string err;
        BOOST_REQUIRE_MESSAGE(frame_profile_validate(p, err), err);
        BOOST_CHECK_EQUAL(frame_length_for(FrameType::Poll, p), w.poll);
        BOOST_CHECK_EQUAL(frame_length_for(FrameType::Response, p), w.response);
        BOOST_CHECK_EQUAL(frame_length_for(FrameType::Final, p), w.final_frame);
        BOOST_CHECK_EQUAL(psdu_length_for(FrameType::Final, p), w.final_frame + 2);

        Frame f = make_golden_frame(kGoldens[2]);
        for (size_t k = 0; k < 3; ++k)
            f.timestamps[k] &= timestamp_max_value(p);
        std::vector<uint8_t> out;
        FrameError code = FrameError::None;
        BOOST_REQUIRE_MESSAGE(encode(f, p, out, err, &code), err);
        BOOST_CHECK_EQUAL(out.size(), w.final_frame);
        Frame back;
        BOOST_REQUIRE(decode(out, p, back, err));
        BOOST_CHECK(back == f);
    }

    // A frame written under one width must not be accepted under another.
    const FrameProfile p40 = default_profile();
    const FrameProfile p32 = [] {
        FrameProfile q = FrameProfile();
        q.timestamp_bits = 32;
        return q;
    }();
    Frame fin = make_golden_frame(kGoldens[2]);
    for (size_t k = 0; k < 3; ++k)
        fin.timestamps[k] &= timestamp_max_value(p32); // truncate on purpose
    std::vector<uint8_t> out40;
    std::string err;
    BOOST_REQUIRE(encode(fin, p40, out40, err));
    Frame back;
    FrameError code = FrameError::None;
    BOOST_CHECK(!decode(out40, p32, back, err, &code));
    BOOST_CHECK_EQUAL(code, FrameError::LengthMismatch);
    // ... and the other way round (26 read as 29 bytes).
    std::vector<uint8_t> out32;
    BOOST_REQUIRE(encode(fin, p32, out32, err));
    BOOST_CHECK(!decode(out32, p40, back, err, &code));
    BOOST_CHECK_EQUAL(code, FrameError::Truncated);
}

BOOST_AUTO_TEST_CASE(uwb_twr_frame_profile_validation)
{
    using namespace gr::uwb::twr;
    std::string err;
    FrameError code = FrameError::None;

    // The phase-1 defaults.
    const FrameProfile p = default_profile();
    BOOST_REQUIRE(frame_profile_validate(p, err, &code));
    BOOST_CHECK_EQUAL(code, FrameError::None);
    BOOST_CHECK_EQUAL(p.timestamp_bits, 40);
    BOOST_CHECK(p.timestamp_unit_hz > 0.0);

    // Non-whole-byte widths have no defined bit order in this layout.
    for (uint8_t bits : { 0, 1, 7, 9, 12, 20, 44, 63, 65 }) {
        FrameProfile q = p;
        q.timestamp_bits = bits;
        BOOST_CHECK_MESSAGE(!frame_profile_validate(q, err, &code),
                            "timestamp_bits " << static_cast<int>(bits)
                                               << " must be rejected");
        BOOST_CHECK_EQUAL(code, FrameError::BadProfile);
    }

    // Tick rate must be finite and positive (a NaN fails the comparison).
    for (double hz : { 0.0, -1.0, -737.28e6, 1.0e16, std::nan(""), std::nan("1") }) {
        FrameProfile q = p;
        q.timestamp_unit_hz = hz;
        BOOST_CHECK_MESSAGE(!frame_profile_validate(q, err, &code),
                            "timestamp_unit_hz " << hz << " must be rejected");
        BOOST_CHECK_EQUAL(code, FrameError::BadProfile);
    }

    // max_psdu_bytes may be tightened but not raised above the PHY limit.
    for (size_t n : { size_t(0), size_t(13), size_t(128), size_t(1000) }) {
        FrameProfile q = p;
        q.max_psdu_bytes = n;
        BOOST_CHECK(!frame_profile_validate(q, err, &code));
        BOOST_CHECK_EQUAL(code, FrameError::BadProfile);
    }
    FrameProfile q = p;
    q.max_psdu_bytes = 31; // exactly Final + FCS
    BOOST_REQUIRE(frame_profile_validate(q, err, &code));

    // A profile that claims the codec emits the FCS is a contradiction.
    FrameProfile r = p;
    r.fcs_appended_by_modulation_layer = false;
    BOOST_CHECK(!frame_profile_validate(r, err, &code));
    BOOST_CHECK_EQUAL(code, FrameError::BadProfile);
}

// ===========================================================================
// 7. session_id / seq wrap independently
// ===========================================================================

BOOST_AUTO_TEST_CASE(uwb_twr_frame_session_and_seq_wrap_independently)
{
    using namespace gr::uwb::twr;
    const FrameProfile p = default_profile();

    // Independent 16-bit wraps.
    BOOST_CHECK_EQUAL(next_seq(0xFFFE), 0xFFFF);
    BOOST_CHECK_EQUAL(next_seq(0xFFFF), 0x0000);
    BOOST_CHECK_EQUAL(next_seq(0x0000), 0x0001);
    BOOST_CHECK_EQUAL(next_session_id(0xFFFE), 0xFFFF);
    BOOST_CHECK_EQUAL(next_session_id(0xFFFF), 0x0000);
    BOOST_CHECK_EQUAL(next_session_id(0x0000), 0x0001);

    // The two counters are not the same counter: one may be at its maximum
    // while the other is near zero.
    Frame a = make_golden_frame(kGoldens[0]);
    a.session_id = 0xFFFF;
    a.seq = 0x0000;
    std::vector<uint8_t> out;
    std::string err;
    BOOST_REQUIRE(encode(a, p, out, err));
    BOOST_CHECK_EQUAL(out[kOffSessionId], 0xFF);
    BOOST_CHECK_EQUAL(out[kOffSessionId + 1], 0xFF);
    BOOST_CHECK_EQUAL(out[kOffSeq], 0x00);
    BOOST_CHECK_EQUAL(out[kOffSeq + 1], 0x00);
    Frame back;
    BOOST_REQUIRE(decode(out, p, back, err));
    BOOST_CHECK_EQUAL(back.session_id, 0xFFFF);
    BOOST_CHECK_EQUAL(back.seq, 0x0000);

    // Both wrap together and stay distinguishable after the wrap.
    Frame b = make_golden_frame(kGoldens[0]);
    b.session_id = next_session_id(0xFFFF);
    b.seq = next_seq(0xFFFF);
    BOOST_REQUIRE(encode(b, p, out, err));
    BOOST_CHECK_EQUAL(out[kOffSessionId], 0x00);
    BOOST_CHECK_EQUAL(out[kOffSeq], 0x00);
    BOOST_REQUIRE(decode(out, p, back, err));
    BOOST_CHECK_EQUAL(back.session_id, 0);
    BOOST_CHECK_EQUAL(back.seq, 0);

    // seq comparison is exact, never a tolerance or a modulo compare.
    BOOST_CHECK(seq_matches(0xFFFF, 0xFFFF));
    BOOST_CHECK(!seq_matches(0xFFFF, 0x0000));
    BOOST_CHECK(!seq_matches(0x0000, 0xFFFF));
    // "newer" is ring-relative and is only ever used for duplicate bookkeeping.
    BOOST_CHECK(seq_is_newer(0xFFFE, 0xFFFF));
    BOOST_CHECK(seq_is_newer(0xFFFF, 0x0000)); // wrapped
    BOOST_CHECK(!seq_is_newer(0x0000, 0xFFFF));
    BOOST_CHECK(!seq_is_newer(0x0007, 0x0007));

    // Matching is decided independently: a right seq with a wrong session is
    // a session failure, and vice versa.  Neither is silently accepted.
    PeerExpectation e;
    e.expected_type = FrameType::Poll;
    e.pan_id = kGoldenPan;
    e.local_addr = kAddrB;
    e.session_id = 0xBEEF;
    e.seq = 0x0102;
    BOOST_CHECK(frame_match(make_golden_frame(kGoldens[0]), e) == FrameMatch::Match);

    Frame wrong_seq = make_golden_frame(kGoldens[0]);
    wrong_seq.seq = 0x0103;
    BOOST_CHECK(frame_match(wrong_seq, e) == FrameMatch::SeqMismatch);

    Frame wrong_session = make_golden_frame(kGoldens[0]);
    wrong_session.session_id = 0xBE00;
    BOOST_CHECK(frame_match(wrong_session, e) == FrameMatch::SessionMismatch);

    // The wrap case that a naive "seq > last" check would break on.
    Frame wrapped = make_golden_frame(kGoldens[0]);
    wrapped.session_id = 0xBEEF;
    wrapped.seq = 0x0000;
    PeerExpectation ew = e;
    ew.seq = 0x0000;
    BOOST_CHECK(frame_match(wrapped, ew) == FrameMatch::Match);
    ew.seq = 0xFFFF; // the previous seq, i.e. a duplicate before the wrap
    BOOST_CHECK(frame_match(wrapped, ew) == FrameMatch::SeqMismatch);
}

// ===========================================================================
// 8. Malformed / truncated input
// ===========================================================================

BOOST_AUTO_TEST_CASE(uwb_twr_frame_malformed_input_rejected)
{
    using namespace gr::uwb::twr;
    const FrameProfile p = default_profile();
    std::string err;
    FrameError code = FrameError::None;
    Frame back;

    // Empty and every truncation of the header.
    for (size_t n = 0; n < kFrameHeaderBytes; ++n) {
        std::vector<uint8_t> bytes = golden_bytes(kGoldens[0]);
        bytes.resize(n);
        BOOST_CHECK_MESSAGE(!decode(bytes, p, back, err, &code),
                            "header truncated to " << n << " bytes");
        BOOST_CHECK_EQUAL(code, FrameError::Truncated);
        BOOST_CHECK_EQUAL(back.version, 0);
    }

    // Short timestamp block: a Response with 24 -> 23 bytes, a Final with
    // 29 -> 28 bytes, and every shorter length too.
    struct Case {
        size_t gi;
        size_t full;
    };
    const Case cases[] = { { 1, 24 }, { 2, 29 } };
    for (const Case& c : cases) {
        for (size_t n = kFrameHeaderBytes; n < c.full; ++n) {
            std::vector<uint8_t> bytes = golden_bytes(kGoldens[c.gi]);
            bytes.resize(n);
            BOOST_CHECK_MESSAGE(!decode(bytes, p, back, err, &code),
                                "case " << c.gi << " truncated to " << n << " bytes");
            BOOST_CHECK_EQUAL(code, FrameError::Truncated);
        }
    }

    // A Poll carrying two timestamps' worth of bytes: rejected, not read as
    // a Response.
    std::vector<uint8_t> poll_plus = golden_bytes(kGoldens[0]);
    poll_plus.resize(24, 0x00);
    BOOST_CHECK(!decode(poll_plus, p, back, err, &code));
    BOOST_CHECK_EQUAL(code, FrameError::LengthMismatch);

    // Random garbage of every length 0..64 is rejected, never accepted.  The
    // function code is chosen so that NO length in the sweep is the length
    // that code requires: below 29 the frame is truncated, at 29 and above it
    // is a Poll (14 B) with trailing bytes.  A 14-byte Poll is a perfectly
    // legal frame -- there is nothing in a Poll's header to invalidate -- so
    // accepting random bytes of exactly that length would be correct, not a
    // hole; the sweep avoids that length on purpose.
    uint32_t seed = 0x12345678u;
    for (size_t n = 0; n <= 64; ++n) {
        std::vector<uint8_t> bytes(n);
        for (size_t i = 0; i < n; ++i) {
            seed = seed * 1664525u + 1013904223u;
            bytes[i] = static_cast<uint8_t>(seed >> 16);
        }
        // Force the header to be plausible so the rejection reason is
        // exercised rather than a lucky version mismatch.
        const bool as_final = (n < 29u);
        if (n >= 2) {
            bytes[kOffVersion] = 0x01;
            bytes[kOffFunctionCode] = as_final ? 0x02 : 0x00;
            bytes[kOffFlags] = 0x01;
            bytes[kOffFlags + 1] = 0x00;
        }
        BOOST_CHECK_MESSAGE(!decode(bytes, p, back, err, &code),
                            "random " << n << " bytes was accepted");
        BOOST_CHECK_EQUAL(code,
                          as_final ? FrameError::Truncated
                                   : FrameError::LengthMismatch);
        BOOST_CHECK_EQUAL(back.version, 0);
    }

    // Null pointer with a non-zero length.
    BOOST_CHECK(!decode(static_cast<const uint8_t*>(nullptr), 14, p, back, err, &code));
    BOOST_CHECK_EQUAL(code, FrameError::NullPointer);
}

// ===========================================================================
// 9. Frame length vs max PSDU
// ===========================================================================

BOOST_AUTO_TEST_CASE(uwb_twr_frame_length_vs_max_psdu)
{
    using namespace gr::uwb::twr;
    const FrameProfile p = default_profile();
    std::string err;
    FrameError code = FrameError::None;

    // MAC payload sizes for the phase-1 profile.
    BOOST_CHECK_EQUAL(frame_length_for(FrameType::Poll, p), 14u);
    BOOST_CHECK_EQUAL(frame_length_for(FrameType::Response, p), 24u);
    BOOST_CHECK_EQUAL(frame_length_for(FrameType::Final, p), 29u);
    // The default-argument form uses the same 40-bit profile.
    BOOST_CHECK_EQUAL(frame_length_for(FrameType::Poll), 14u);
    BOOST_CHECK_EQUAL(frame_length_for(FrameType::Response), 24u);
    BOOST_CHECK_EQUAL(frame_length_for(FrameType::Final), 29u);
    // With the FCS the modulation layer appends.
    BOOST_CHECK_EQUAL(psdu_length_for(FrameType::Poll, p), 16u);
    BOOST_CHECK_EQUAL(psdu_length_for(FrameType::Response, p), 26u);
    BOOST_CHECK_EQUAL(psdu_length_for(FrameType::Final, p), 31u);

    // All three fit the PHY's 127-byte PSDU with room to spare.
    BOOST_CHECK(frame_length_for(FrameType::Final, p) + kFrameFcsBytes <=
                kPhyMaxPsduBytes);
    BOOST_CHECK_EQUAL(gr::uwb::radar_meta::kMaxPsduBytes, kPhyMaxPsduBytes);

    // A profile that cannot hold Final + FCS refuses to build one up front
    // (REQ-API-01: reject before the radio starts).
    FrameProfile tight = p;
    tight.max_psdu_bytes = 20;
    BOOST_REQUIRE(frame_profile_validate(tight, err, &code));
    BOOST_CHECK_EQUAL(frame_length_for_checked(FrameType::Poll, tight, err, &code), 14u);
    BOOST_CHECK_EQUAL(frame_length_for_checked(FrameType::Response, tight, err, &code),
                      0u);
    BOOST_CHECK_EQUAL(code, FrameError::PsduTooLarge);
    BOOST_CHECK_EQUAL(frame_length_for_checked(FrameType::Final, tight, err, &code), 0u);
    BOOST_CHECK_EQUAL(code, FrameError::PsduTooLarge);
    BOOST_CHECK_EQUAL(frame_error_to_exchange_status(code), ExchangeStatus::ConfigRejected);

    Frame f = make_golden_frame(kGoldens[2]);
    std::vector<uint8_t> out;
    BOOST_CHECK(!encode(f, tight, out, err, &code));
    BOOST_CHECK_EQUAL(code, FrameError::PsduTooLarge);

    // 16 is exactly Poll + FCS, 30 exactly Response + FCS.
    FrameProfile t16 = p;
    t16.max_psdu_bytes = 16;
    BOOST_CHECK_EQUAL(frame_length_for_checked(FrameType::Poll, t16, err, &code), 14u);
    t16.max_psdu_bytes = 25;
    BOOST_CHECK_EQUAL(frame_length_for_checked(FrameType::Response, t16, err, &code), 0u);
    t16.max_psdu_bytes = 26;
    BOOST_CHECK_EQUAL(frame_length_for_checked(FrameType::Response, t16, err, &code), 24u);
    t16.max_psdu_bytes = 30;
    BOOST_CHECK_EQUAL(frame_length_for_checked(FrameType::Final, t16, err, &code), 0u);
    t16.max_psdu_bytes = 31;
    BOOST_CHECK_EQUAL(frame_length_for_checked(FrameType::Final, t16, err, &code), 29u);

    // 64-bit timestamps: Final is 38 B, still inside 127.
    FrameProfile p64 = p;
    p64.timestamp_bits = 64;
    BOOST_CHECK_EQUAL(frame_length_for(FrameType::Final, p64), 38u);
    BOOST_CHECK_EQUAL(psdu_length_for(FrameType::Final, p64), 40u);
    BOOST_REQUIRE(encode(f, p64, out, err, &code));
    BOOST_CHECK_EQUAL(out.size(), 38u);
}

// ===========================================================================
// 10. Exactly one layer appends the FCS
// ===========================================================================

BOOST_AUTO_TEST_CASE(uwb_twr_frame_fcs_has_exactly_one_producer)
{
    using namespace gr::uwb::twr;
    const FrameProfile p = default_profile();
    std::string err;
    FrameError code = FrameError::None;

    // 1. The codec never appends one: 14 / 24 / 29, never +2.
    const size_t want[] = { 14, 24, 29 };
    for (size_t gi = 0; gi < 3; ++gi) {
        std::vector<uint8_t> out;
        BOOST_REQUIRE(encode(make_golden_frame(kGoldens[gi]), p, out, err, &code));
        BOOST_CHECK_EQUAL(out.size(), want[gi]);
        BOOST_CHECK(out.size() != want[gi] + kFrameFcsBytes);
        // The encode path cannot be told to add one either.
        std::vector<uint8_t> other;
        BOOST_CHECK(encode(make_golden_frame(kGoldens[gi]), p, other, err, &code));
        BOOST_CHECK(other == out);
    }

    // 2. The modulation layer adds exactly two bytes: 16 / 26 / 31.
    for (size_t gi = 0; gi < 3; ++gi) {
        std::vector<uint8_t> out;
        BOOST_REQUIRE(encode(make_golden_frame(kGoldens[gi]), p, out, err, &code));
        const std::vector<uint8_t> codec_only = out;
        gr::uwb::mod::append_ieee_fcs(out); // what UwbHrpPacketSource does
        BOOST_CHECK_EQUAL(out.size(), want[gi] + kFrameFcsBytes);
        // The FCS is appended, nothing is rewritten.
        BOOST_REQUIRE(out.size() > codec_only.size());
        BOOST_CHECK(std::equal(codec_only.begin(), codec_only.end(), out.begin()));

        // 3. A PSDU that still carries its FCS must NOT decode.  This is the
        //    regression guard for a double-FCS or a wrong-layer strip.
        Frame back;
        BOOST_CHECK_MESSAGE(!decode(out, p, back, err, &code),
                            "a PSDU with the FCS still attached decoded as a frame");
        BOOST_CHECK_EQUAL(code, FrameError::LengthMismatch);

        // 4. Stripping exactly two bytes gives back a decodable MAC payload.
        const uint8_t* mac = nullptr;
        size_t mac_bytes = 0;
        BOOST_REQUIRE(mac_payload_from_psdu(out.data(), out.size(), p, mac, mac_bytes, err,
                                             &code));
        BOOST_CHECK_EQUAL(mac_bytes, want[gi]);
        BOOST_REQUIRE(decode(mac, mac_bytes, p, back, err, &code));
        BOOST_CHECK(back == make_golden_frame(kGoldens[gi]));
    }

    // 5. A PSDU too short to hold a header plus an FCS is rejected.
    {
        std::vector<uint8_t> tiny(10, 0x00);
        const uint8_t* mac = nullptr;
        size_t mac_bytes = 0;
        BOOST_CHECK(!mac_payload_from_psdu(tiny.data(), tiny.size(), p, mac, mac_bytes, err,
                                          &code));
        BOOST_CHECK_EQUAL(code, FrameError::Truncated);
    }
    {
        // 128 bytes: over the PHY max PSDU.
        std::vector<uint8_t> big(128, 0x00);
        const uint8_t* mac = nullptr;
        size_t mac_bytes = 0;
        BOOST_CHECK(!mac_payload_from_psdu(big.data(), big.size(), p, mac, mac_bytes, err,
                                          &code));
        BOOST_CHECK_EQUAL(code, FrameError::PsduTooLarge);
    }
}

BOOST_AUTO_TEST_CASE(uwb_twr_frame_only_one_source_file_appends_the_fcs)
{
    // Structural guard for "只由一层追加 FCS" (REQ-API-01).  Walk every
    // production source in gr-uwb and collect the lines that append an FCS.
    // The invariant is about PRODUCTION code, so the codec's own QA -- which
    // legitimately calls mod::append_ieee_fcs to check the resulting length --
    // is excluded.  Everything else must be the single modulation-layer site.
    const std::string root = repo_root();
    BOOST_REQUIRE_MESSAGE(!root.empty(),
                          "cannot locate the repository root from UWB_TESTDATA_DIR");
    const std::string hdr_dir = root + "/gr-uwb/include/gnuradio/uwb";
    const std::string lib_dir = root + "/gr-uwb/lib";
    BOOST_REQUIRE_MESSAGE(!list_files(hdr_dir, ".h").empty(), "no headers in " << hdr_dir);
    BOOST_REQUIRE_MESSAGE(!list_files(lib_dir, ".cc").empty(), "no sources in " << lib_dir);

    // Comment-stripped lines, so a mention in prose cannot be mistaken for a
    // call site (this file, for one, mentions the function in its header).
    struct Hit {
        std::string file;
        int line;
        std::string text;
    };
    std::vector<Hit> hits;
    std::vector<std::string> files = list_files(hdr_dir, ".h");
    for (const std::string& s : list_files(lib_dir, ".cc"))
        files.push_back(s);

    for (const std::string& f : files) {
        if (f.size() >= 3 && f.compare(f.size() - 3, 3, ".cc") == 0 &&
            f.find("/qa_") != std::string::npos) {
            continue; // QA: not a production layer
        }
        std::string text;
        if (!read_file(f, text))
            continue;
        if (text.find("append_ieee_fcs") == std::string::npos)
            continue;
        int line_no = 0;
        std::istringstream ss(strip_comments(text));
        std::string line;
        while (std::getline(ss, line)) {
            ++line_no;
            if (line.find("append_ieee_fcs") == std::string::npos)
                continue;
            hits.push_back(Hit{ f, line_no, line });
        }
    }

    // Exactly one production append site, and it must be the modulation
    // layer, gated on the append_fcs parameter.  The definition itself and the
    // core's own random-PSDU test helper are not append layers.
    int defs = 0;
    int core_helper = 0;
    int production = 0;
    for (const Hit& h : hits) {
        const bool in_core = h.file.find("uwb_hrp_mod_core.h") != std::string::npos;
        const bool in_pkt = h.file.find("uwb_hrp_packet_source.cc") != std::string::npos;
        if (in_core && h.text.find("void append_ieee_fcs") != std::string::npos) {
            ++defs;
        } else if (in_core) {
            ++core_helper;
        } else if (in_pkt && h.text.find("mod::append_ieee_fcs(") != std::string::npos) {
            ++production;
            // ... and it must be gated on the append_fcs parameter, so the FCS
            // is never appended unless the layer was explicitly asked to.
            std::string raw;
            BOOST_REQUIRE(read_file(h.file, raw));
            std::istringstream rs(raw);
            std::string line;
            int line_no = 0;
            bool gated = false;
            while (std::getline(rs, line)) {
                ++line_no;
                if (line_no < h.line && line_no + 8 >= h.line &&
                    line.find("if (append_fcs)") != std::string::npos) {
                    gated = true;
                }
            }
            BOOST_CHECK_MESSAGE(gated,
                                "the FCS append in uwb_hrp_packet_source.cc "
                                "must be inside `if (append_fcs)`");
        } else {
            BOOST_FAIL("unexpected FCS production site " << h.file << ":" << h.line
                                                          << "  " << h.text);
        }
    }
    BOOST_CHECK_EQUAL(defs, 1);       // the definition
    BOOST_CHECK_EQUAL(core_helper, 1); // make_random_psdu, a test helper
    BOOST_CHECK_EQUAL(production, 1);  // UwbHrpPacketSource, the only layer
}

// ===========================================================================
// 11. Semantic accessors make a wrong field order hard to write
// ===========================================================================

BOOST_AUTO_TEST_CASE(uwb_twr_frame_semantic_timestamp_accessors)
{
    using namespace gr::uwb::twr;
    const FrameProfile p = default_profile();
    std::string err;
    FrameError code = FrameError::None;

    const Frame resp = make_golden_frame(kGoldens[1]);
    const Frame fin = make_golden_frame(kGoldens[2]);
    const Frame poll = make_golden_frame(kGoldens[0]);

    // Response carries t2B at index 0 and t3B at index 1 -- in that order.
    BOOST_REQUIRE(resp.carries(TimestampField::T2B));
    BOOST_REQUIRE(resp.carries(TimestampField::T3B));
    BOOST_CHECK(!resp.carries(TimestampField::T1A));
    BOOST_CHECK(!resp.carries(TimestampField::T4A));
    BOOST_CHECK(!resp.carries(TimestampField::T5A));
    BOOST_CHECK_EQUAL(resp.index_of(TimestampField::T2B), 0);
    BOOST_CHECK_EQUAL(resp.index_of(TimestampField::T3B), 1);
    BOOST_CHECK_EQUAL(resp.index_of(TimestampField::T1A), -1);
    BOOST_CHECK_EQUAL(resp.t2B(), 0x0123456789ull);
    BOOST_CHECK_EQUAL(resp.t3B(), 0x00FEDCBAull);
    BOOST_CHECK_EQUAL(resp.t1A(), 0u); // not carried
    BOOST_CHECK_EQUAL(resp.t4A(), 0u);

    // Final carries t1A, t4A, t5A at 0, 1, 2 and does NOT carry t2B/t3B.
    BOOST_REQUIRE(fin.carries(TimestampField::T1A));
    BOOST_REQUIRE(fin.carries(TimestampField::T4A));
    BOOST_REQUIRE(fin.carries(TimestampField::T5A));
    BOOST_CHECK(!fin.carries(TimestampField::T2B));
    BOOST_CHECK(!fin.carries(TimestampField::T3B));
    BOOST_CHECK_EQUAL(fin.index_of(TimestampField::T1A), 0);
    BOOST_CHECK_EQUAL(fin.index_of(TimestampField::T4A), 1);
    BOOST_CHECK_EQUAL(fin.index_of(TimestampField::T5A), 2);
    BOOST_CHECK_EQUAL(fin.t1A(), 0x1122334455ull);
    BOOST_CHECK_EQUAL(fin.t4A(), 0x9988776655ull);
    BOOST_CHECK_EQUAL(fin.t5A(), 0x0102030405ull);
    BOOST_CHECK_EQUAL(fin.t2B(), 0u); // not carried, NOT another instant
    BOOST_CHECK_EQUAL(fin.t3B(), 0u);

    // Poll carries nothing.
    BOOST_CHECK_EQUAL(poll.timestamp_count(), 0u);
    for (unsigned i = 0; i < static_cast<unsigned>(TimestampField::Count); ++i) {
        BOOST_CHECK(!poll.carries(static_cast<TimestampField>(i)));
    }
    uint64_t v = 0xDEAD;
    BOOST_CHECK(!poll.get(TimestampField::T1A, v));
    BOOST_CHECK_EQUAL(v, 0xDEAD); // untouched on failure
    BOOST_CHECK(!poll.get(TimestampField::T2B, v));
    Frame poll_mutable = poll;
    BOOST_CHECK(!poll_mutable.set(TimestampField::T3B, 1));
    BOOST_CHECK_EQUAL(poll_mutable.timestamps[0], 0u);

    // Setting a carried field lands in the right slot and survives encode.
    Frame f = fin;
    BOOST_REQUIRE(f.set(TimestampField::T5A, 0x00000000A5ull));
    BOOST_CHECK_EQUAL(f.timestamps[2], 0xA5ull);
    BOOST_CHECK_EQUAL(f.t1A(), 0x1122334455ull);
    BOOST_CHECK(!f.set(TimestampField::T2B, 1));
    std::vector<uint8_t> out;
    BOOST_REQUIRE(encode(f, p, out, err, &code));
    BOOST_CHECK_EQUAL(out[24], 0xA5); // first byte of t5A, little-endian
    Frame back;
    BOOST_REQUIRE(decode(out, p, back, err, &code));
    BOOST_CHECK_EQUAL(back.t5A(), 0xA5ull);

    // The named strings are the ones M1's logging and the golden file use.
    BOOST_CHECK_EQUAL(std::string(timestamp_field_name(TimestampField::T1A)), "t1A");
    BOOST_CHECK_EQUAL(std::string(timestamp_field_name(TimestampField::T2B)), "t2B");
    BOOST_CHECK_EQUAL(std::string(timestamp_field_name(TimestampField::T3B)), "t3B");
    BOOST_CHECK_EQUAL(std::string(timestamp_field_name(TimestampField::T4A)), "t4A");
    BOOST_CHECK_EQUAL(std::string(timestamp_field_name(TimestampField::T5A)), "t5A");
}

// ===========================================================================
// 12. encode_into: caller-provided storage, no allocation
// ===========================================================================

BOOST_AUTO_TEST_CASE(uwb_twr_frame_encode_into_caller_storage)
{
    using namespace gr::uwb::twr;
    const FrameProfile p = default_profile();
    std::string err;
    FrameError code = FrameError::None;

    FrameScratch scratch;
    BOOST_CHECK_EQUAL(FrameScratch::capacity, kMaxFrameBytes);
    BOOST_CHECK(FrameScratch::capacity >= frame_length_for(FrameType::Final, p));

    for (size_t gi = 0; gi < kNumGoldens; ++gi) {
        const Golden& g = kGoldens[gi];
        size_t written = 0;
        BOOST_REQUIRE_MESSAGE(
            encode_into(make_golden_frame(g), p, scratch.bytes, scratch.capacity, written,
                        err, &code),
            err);
        BOOST_CHECK_EQUAL(code, FrameError::None);
        BOOST_CHECK_EQUAL(written, g.payload_bytes);
        BOOST_CHECK_EQUAL(bytes_to_hex(scratch.bytes, written), std::string(g.mac_payload_hex));
        BOOST_CHECK(std::string(err).empty());
    }

    // A too-small buffer is refused, and nothing is written.
    uint8_t small[4] = { 0xAA, 0xAA, 0xAA, 0xAA };
    size_t written = 0;
    BOOST_CHECK(!encode_into(make_golden_frame(kGoldens[0]), p, small, sizeof(small),
                             written, err, &code));
    BOOST_CHECK_EQUAL(code, FrameError::BufferTooSmall);
    BOOST_CHECK_EQUAL(written, 0u);
    BOOST_CHECK_EQUAL(small[0], 0xAA);

    BOOST_CHECK(!encode_into(make_golden_frame(kGoldens[0]), p, nullptr, 14, written, err,
                             &code));
    BOOST_CHECK_EQUAL(code, FrameError::NullPointer);

    // The 64-bit worst case still fits the scratch buffer.
    FrameProfile p64 = p;
    p64.timestamp_bits = 64;
    size_t w64 = 0;
    BOOST_REQUIRE(encode_into(make_golden_frame(kGoldens[2]), p64, scratch.bytes,
                              scratch.capacity, w64, err, &code));
    BOOST_CHECK_EQUAL(w64, 38u);
}

// ===========================================================================
// 13. Frame matching (REQ-PROTO-01) reports each failure separately
// ===========================================================================

BOOST_AUTO_TEST_CASE(uwb_twr_frame_match_reports_each_failure_separately)
{
    using namespace gr::uwb::twr;

    PeerExpectation e;
    e.expected_type = FrameType::Response;
    e.pan_id = kGoldenPan;
    e.local_addr = kAddrA;
    e.session_id = 0xBEEF;
    e.seq = 0x0102;

    BOOST_CHECK(frame_match(make_golden_frame(kGoldens[1]), e) == FrameMatch::Match);
    BOOST_CHECK_EQUAL(frame_error_to_exchange_status(FrameError::None),
                      ExchangeStatus::Ok);

    // Wrong message type is reported first: for a wrong type the remaining
    // fields have no defined meaning.
    Frame f = make_golden_frame(kGoldens[1]);
    f.function_code = FrameType::Final;
    BOOST_CHECK(frame_match(f, e) == FrameMatch::TypeMismatch);
    BOOST_CHECK_EQUAL(frame_match_to_exchange_status(FrameMatch::TypeMismatch),
                      ExchangeStatus::UnexpectedFrameType);
    // Even with a wrong PAN too, the type is what gets reported.
    f.pan_id = 0x9999;
    BOOST_CHECK(frame_match(f, e) == FrameMatch::TypeMismatch);

    f = make_golden_frame(kGoldens[1]);
    f.pan_id = 0x9999;
    BOOST_CHECK(frame_match(f, e) == FrameMatch::PanMismatch);
    BOOST_CHECK_EQUAL(frame_match_to_exchange_status(FrameMatch::PanMismatch),
                      ExchangeStatus::WrongPeer);

    f = make_golden_frame(kGoldens[1]);
    f.dst_addr = 0x00AB;
    BOOST_CHECK(frame_match(f, e) == FrameMatch::AddressMismatch);

    // Our own transmission coming back is not a peer measurement.
    f = make_golden_frame(kGoldens[1]);
    f.src_addr = kAddrA;
    BOOST_CHECK(frame_match(f, e) == FrameMatch::SelfAddressed);
    BOOST_CHECK_EQUAL(frame_match_to_exchange_status(FrameMatch::SelfAddressed),
                      ExchangeStatus::WrongPeer);

    f = make_golden_frame(kGoldens[1]);
    f.session_id = 0xBEE0;
    BOOST_CHECK(frame_match(f, e) == FrameMatch::SessionMismatch);
    BOOST_CHECK_EQUAL(frame_match_to_exchange_status(FrameMatch::SessionMismatch),
                      ExchangeStatus::StaleSession);

    f = make_golden_frame(kGoldens[1]);
    f.seq = 0x0103;
    BOOST_CHECK(frame_match(f, e) == FrameMatch::SeqMismatch);
    BOOST_CHECK_EQUAL(frame_match_to_exchange_status(FrameMatch::SeqMismatch),
                      ExchangeStatus::StaleSession);

    // A late/duplicate frame is NOT rescued by relaxing a check: every
    // non-Match result is a failure that cannot yield a range.
    BOOST_CHECK(!exchange_status_yields_range(
        frame_match_to_exchange_status(FrameMatch::TypeMismatch)));
    BOOST_CHECK(!exchange_status_yields_range(
        frame_match_to_exchange_status(FrameMatch::SeqMismatch)));

    // Every name round trips to a distinct non-"invalid" string.
    const FrameMatch all[] = { FrameMatch::Match,
                               FrameMatch::TypeMismatch,
                               FrameMatch::PanMismatch,
                               FrameMatch::AddressMismatch,
                               FrameMatch::SelfAddressed,
                               FrameMatch::SessionMismatch,
                               FrameMatch::SeqMismatch };
    for (FrameMatch m : all)
        BOOST_CHECK(std::string(frame_match_to_string(m)) != "invalid");
    BOOST_CHECK(std::string(frame_match_to_string(static_cast<FrameMatch>(99))) == "invalid");
}

// ===========================================================================
// 14. Hex helpers used by the golden file and the failure reports
// ===========================================================================

BOOST_AUTO_TEST_CASE(uwb_twr_frame_hex_helpers)
{
    using namespace gr::uwb::twr;
    std::vector<uint8_t> v;
    BOOST_REQUIRE(hex_to_bytes("00ff10aB", v));
    BOOST_REQUIRE_EQUAL(v.size(), 4u);
    BOOST_CHECK_EQUAL(v[0], 0x00);
    BOOST_CHECK_EQUAL(v[1], 0xFF);
    BOOST_CHECK_EQUAL(v[2], 0x10);
    BOOST_CHECK_EQUAL(v[3], 0xAB);
    BOOST_CHECK_EQUAL(bytes_to_hex(v), std::string("00ff10ab"));
    BOOST_CHECK_EQUAL(bytes_to_hex(std::vector<uint8_t>()), std::string(""));

    BOOST_CHECK(!hex_to_bytes("abc", v)); // odd length
    BOOST_CHECK(v.empty());
    BOOST_CHECK(!hex_to_bytes("zz", v)); // not hex
    BOOST_CHECK(v.empty());
    BOOST_CHECK(hex_to_bytes("", v));
    BOOST_CHECK(v.empty());

    for (size_t gi = 0; gi < kNumGoldens; ++gi) {
        const std::vector<uint8_t> b = golden_bytes(kGoldens[gi]);
        std::vector<uint8_t> back;
        BOOST_REQUIRE(hex_to_bytes(bytes_to_hex(b), back));
        BOOST_CHECK(back == b);
    }
}
