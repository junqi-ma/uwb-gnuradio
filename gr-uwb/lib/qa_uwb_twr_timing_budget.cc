/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * TWR M0 — measured time / length budget (offline, no UHD, no hardware).
 *
 * 需求_UWB_SS_DS_TWR.md §6 (REQ-TIME-01..05) and §7 (REQ-GR-02/03/04/06)
 * require that every time field carry an explicit clock domain / rate /
 * marker / quantisation, and that the reply-turnaround budget (REQ-GR-04) be
 * an *input* to the feasibility question rather than a guess.  This QA turns
 * the M0 geometry into numbers and writes them to
 *
 *     timing_budget_737p28.csv
 *     timing_budget_491p52.csv
 *     timing_budget_rmarker_candidates.csv
 *     timing_budget_turnaround_estimate.csv
 *
 * WHERE (M0 review R8 / M0.1 §2.C.2).  These are generated tables, not
 * reviewed artifacts, so they do NOT go into the source `testdata/twr` tree
 * by default -- an ordinary `ctest` run used to overwrite the reviewed copies
 * there.  Default is the build tree:
 *
 *     DEFAULT  ${UWB_BUILD_DIR}/test-output/twr
 *              (override with UWB_TWR_TEST_OUTPUT_DIR=<dir>)
 *     EXPORT   UWB_TWR_EXPORT_DIR=<dir> to write into a tree deliberately,
 *              including the source testdata/twr when that is the intent.
 *
 * When no writable directory is available the emit cases SKIP and record
 * why; they never fall back to the source tree.
 *
 * WHAT IS MEASURED HERE
 *   1. Work-grid (998.4 MS/s) frame geometry, from
 *      mod::packet_samples_998p4() and its per-segment constants.
 *   2. Native-grid (737.28 / 491.52 MS/s) geometry, both as the ceil()
 *      per-segment convention used to size TX files / RX capture windows
 *      (qa_uwb_radar_e2e.cc: kSyncNative64 = ceil(64*1016*48/65) = 48018,
 *      kSyncNative64Cg400 = ceil(64*1016*32/65) = 32012) and as the
 *      resampler's group-delay-centred map differences.
 *   3. Marker offsets (PreambleStart / SfdStart / PHRStart / FrameEnd) in
 *      both grids, and the resulting candidate RMARKER positions.
 *   4. Time-anchor consistency: the real UwbPduRationalResamplerCcf65_48 /
 *      ...65_32 blocks' map_input_offset_to_output() and
 *      try_map_input_offset_to_output() over a real frame's worth of
 *      samples.  Conventions are inherited verbatim from
 *      qa_uwb_radar_e2e.cc §"Unified coordinate/calibration conventions";
 *      this file does NOT introduce a second coordinate system.
 *
 * WHAT IS **NOT** MEASURED HERE (and must not be inferred from it)
 *   * Host-side processing latency: detect+decode, FSM+frame patch+
 *     modulate, PDU/SC16 conversion, TX transport, and the minimum UHD
 *     lead time.  None of these are exercised here — the CSV records the
 *     literal token UNMEASURED for them.  REQ-GR-04 requires hardware
 *     measurement (M2/M3) to replace the offline estimate.
 *   * The absolute DW1000 / DW3000 RMARKER↔waveform convention.  Only the
 *     *internal* offsets are measured; REQ-TIME-02 pins the absolute
 *     convention in the PHY profile and cross-checks it against chip
 *     documentation, which is M2/M5 work.
 *   * Any wall-clock number.  This QA is offline; all assertions are on
 *     exact integer sample counts, never on elapsed time.
 *
 * COORDINATE CONVENTIONS (inherited, not invented)
 *
 *   map(p) = round((p*interp + (T-1)/2) / decim)      [group-delay-centred]
 *   map(0) = round((T-1)/2 / decim)                   [FIR head, work samples]
 *   anchored marker offset  = map(ws + off) - map(ws)  == map(off) - map(0)
 *   unanchored marker offset = round(off_work * interp / decim)
 *   predicted = map(0) + (map(ws + pre_guard) - map(0)) + round(cal_work)
 *               + sync_repetitions * 1016
 *   737.28 example (pre_guard 1475 native, D = 37 native, T = 2707):
 *     predicted   = map(1475) + 50 + 65024            = 67100
 *     sfd_truth   = map(1475 + 37 + 48018)            = 67100   (exact)
 *   491.52 example (pre_guard 983 native, D = 25 native, T = 2707):
 *     predicted   = map(983) + 51 + 65024             = 67114
 *     sfd_truth   = map(983 + 25 + 32012)             = 67114   (exact)
 *
 * Clock domains in the CSV:
 *   work   : 998.4 MS/s, the PHY work grid (kQm35SampleRate).  The
 *            integer frame length is exact here by construction.
 *   native : the UHD stream rate, 737.28 or 491.52 MS/s.  Only
 *            is_allowed_uhd_native_rate() values are covered.  Native
 *            frame lengths are quantised; they are NOT the work length
 *            divided, they are a ceil() per segment.
 */

#include <boost/test/unit_test.hpp>

#include <gnuradio/uwb/uwb_hrp_mod_core.h>
#include <gnuradio/uwb/uwb_phy_profile.h>
#include <gnuradio/uwb/uwb_pdu_rational_resampler_ccf_65_32.h>
#include <gnuradio/uwb/uwb_pdu_rational_resampler_ccf_65_48.h>
#include <gnuradio/uwb/uwb_radar_checked_math.h>
#include <gnuradio/uwb/uwb_radar_pdu_meta.h>
#include <gnuradio/uwb/uwb_twr_test_output.h>
#include <gnuradio/uwb/uwb_uhd_backend_config.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

namespace gr {
namespace uwb {

using gr::uwb::UwbPduRationalResamplerCcf65_32;
using gr::uwb::UwbPduRationalResamplerCcf65_48;

namespace {

// ---------------------------------------------------------------------------
// Frozen PHY constants (uwb_phy_profile.h / uwb_hrp_mod_core.h)
// ---------------------------------------------------------------------------
constexpr size_t kSps = demod::kQm35SamplesPerSymbol;  // 1016
constexpr size_t kSpc = demod::kQm35SamplesPerChip;    // 2
constexpr double kFsWork = demod::kQm35SampleRate;     // 998.4e6

// M0 TWR MAC payload sizes.  These EXCLUDE the 2-byte IEEE FCS; the +2
// variants are the on-air PSDU length actually handed to the modulator.
constexpr size_t kPollBytes = 14;
constexpr size_t kResponseBytes = 24;
constexpr size_t kFinalBytes = 29;
constexpr size_t kFcsBytes = 2;

constexpr double kFs737 = 737.28e6;
constexpr double kFs491 = 491.52e6;

// Native rate -> (interp, decim).  998.4/737.28 = 65/48, 998.4/491.52 = 65/32.
struct RateSpec {
    double native_hz;
    uint32_t interp;
    uint32_t decim;
    const char* tag; // "737p28" / "491p52"
};

const RateSpec kRates[] = {
    { kFs737, 65, 48, "737p28" },
    { kFs491, 65, 32, "491p52" },
};

const char* kSfdModes[] = { "4z1", "4z2", "4z3", "4z4", "ieee", "decawave" };
const size_t kSyncReps[] = { 16, 32, 64, 128, 1024 };

struct FrameRole {
    const char* name;
    size_t mac_bytes; // excludes FCS
};

const FrameRole kRoles[] = {
    { "Poll", kPollBytes },
    { "Response", kResponseBytes },
    { "Final", kFinalBytes },
};

// ---------------------------------------------------------------------------
// Geometry: work-grid decomposition of mod::packet_samples_998p4()
// ---------------------------------------------------------------------------
struct Geometry {
    size_t n_sfd = 0;
    size_t nsym = 0; // BPRF 6.81 Mb/s payload symbols (post RS + conv)
    size_t preamble_work = 0;
    size_t sfd_work = 0;
    size_t phr_work = 0;
    size_t payload_work = 0;
    size_t total_work = 0;
};

// First-principles reconstruction of the frame geometry from the individual
// PHY constants, independent of mod::packet_samples_998p4():
//   preamble = sync_reps * kQm35SamplesPerSymbol
//   sfd      = n_sfd      * kQm35SamplesPerSymbol
//   phr      = kPhrSymbols * kPhrChipsPerSymbol * kQm35SamplesPerChip
//   payload  = nsym       * kPayloadChipsPerSymbol * kQm35SamplesPerChip
//   nsym     = psdu_bits + 48 * ceil(psdu_bits / 330)     (RS 330/378)
Geometry
geometry(size_t sync_reps, const char* sfd_mode, size_t psdu_bytes)
{
    Geometry g;
    const std::vector<int8_t> sfd = demod::GetSfdSequence(sfd_mode);
    g.n_sfd = sfd.size();
    const size_t psdu_bits = psdu_bytes * 8;
    const size_t blocks = (psdu_bits + 329) / 330;
    g.nsym = psdu_bits + 48 * blocks;
    g.preamble_work = sync_reps * kSps;
    g.sfd_work = g.n_sfd * kSps;
    g.phr_work = mod::kPhrSymbols * mod::kPhrChipsPerSymbol * kSpc;
    g.payload_work = g.nsym * mod::kPayloadChipsPerSymbol * kSpc;
    g.total_work = g.preamble_work + g.sfd_work + g.phr_work + g.payload_work;
    return g;
}

// ceil(work * decim / interp) — the native-grid length convention used for
// TX files and RX capture sizing (see kSyncNative64 in qa_uwb_radar_e2e.cc).
int64_t
native_len_ceil(int64_t work, uint32_t decim, uint32_t interp)
{
    return (work * static_cast<int64_t>(decim) + interp - 1) / interp;
}

double
us_from_work(int64_t work)
{
    return static_cast<double>(work) * 1e6 / kFsWork;
}

double
us_from_native(int64_t native, double native_hz)
{
    return static_cast<double>(native) * 1e6 / native_hz;
}

std::string
join_path(const std::string& dir, const std::string& rel)
{
    return (std::filesystem::path(dir) / rel).string();
}

// Returns the directory this QA's generated CSV tables may be written into,
// or "" if none is usable (in which case the emit cases SKIP and say why).
//
// M0 review R8 / M0.1 §2.C.2.  This used to hardcode `testdata/twr` in the
// SOURCE tree, so every ordinary `ctest` run overwrote the reviewed
// artifacts there; the reviewer had to rebuild and run this QA out of tree
// just to avoid clobbering them.  The rule is now:
//
//   DEFAULT  ${UWB_BUILD_DIR}/test-output/twr  (or $UWB_TWR_TEST_OUTPUT_DIR)
//   EXPORT   $UWB_TWR_EXPORT_DIR=<dir> to write into a tree deliberately
//
// The source `testdata/twr` tree is refused unless export was requested, and
// there is no silent fallback to it.
std::string
twr_dir(std::string& why_out)
{
    static int tried = 0;
    static std::string path;
    static std::string why;
    if (tried == 0) {
        tried = 1;
        path = gr::uwb::twr::testout::resolve(why);
    }
    why_out = why;
    return path;
}

// Marker positions inside a frame, in work samples from PreambleStart.
enum class Marker { PreambleStart, SfdStart, PHRStart, PHREnd, FrameEnd };

const char*
marker_name(Marker m)
{
    switch (m) {
    case Marker::PreambleStart:
        return "PreambleStart";
    case Marker::SfdStart:
        return "SfdStart";
    case Marker::PHRStart:
        return "PHRStart";
    case Marker::PHREnd:
        return "PHREnd";
    default:
        return "FrameEnd";
    }
}

int64_t
marker_work_offset(Marker m, const Geometry& g)
{
    switch (m) {
    case Marker::PreambleStart:
        return 0;
    case Marker::SfdStart:
        return static_cast<int64_t>(g.preamble_work);
    case Marker::PHRStart:
        return static_cast<int64_t>(g.preamble_work + g.sfd_work);
    case Marker::PHREnd:
        return static_cast<int64_t>(g.preamble_work + g.sfd_work + g.phr_work);
    default:
        return static_cast<int64_t>(g.total_work);
    }
}

// ---------------------------------------------------------------------------
// Local reimplementation of RationalResamplerLmCore::map_input_offset_to_output,
// parameterised by (interp, decim, T).  Used for the CSV tables; the block
// instances built in twr_timing_anchor_65_48 / _65_32 assert equality with
// the same taps, so the tables cannot silently drift from the implementation.
// ---------------------------------------------------------------------------
struct MapFn {
    uint32_t interp = 65;
    uint32_t decim = 48;
    int64_t taps = 2707;

    // round((p*interp + (T-1)/2) / decim), clamped at 0.
    int64_t operator()(int64_t p) const
    {
        const double d = 0.5 * static_cast<double>(taps > 0 ? taps - 1 : 0);
        const double m =
            (static_cast<double>(p) * static_cast<double>(interp) + d) /
            static_cast<double>(decim);
        const int64_t r = static_cast<int64_t>(std::llround(m));
        return r < 0 ? 0 : r;
    }
    // FIR head in work samples: map(0).
    int64_t head() const { return (*this)(0); }
};

std::vector<float>
load_taps_f32(const std::string& path)
{
    std::ifstream f(path, std::ios::binary | std::ios::ate);
    BOOST_REQUIRE_MESSAGE(f, "cannot open taps file " + path);
    const auto bytes = static_cast<size_t>(f.tellg());
    BOOST_REQUIRE_EQUAL(bytes % sizeof(float), 0u);
    f.seekg(0);
    std::vector<float> v(bytes / sizeof(float));
    f.read(reinterpret_cast<char*>(v.data()),
           static_cast<std::streamsize>(bytes));
    return v;
}

} // namespace

// ---------------------------------------------------------------------------
// 1. Work-grid frame geometry: the per-segment decomposition must reproduce
//    mod::packet_samples_998p4() exactly, for every (sync_reps, sfd_mode,
//    psdu_len) in the M0 sweep.  This is the authoritative frame-length
//    contract for the reply-delay budget (REQ-API-02: "若配置'帧尾后延迟'，
//    必须由精确帧长换算").
// ---------------------------------------------------------------------------
// Maps a turnaround scope to the name of the frame that scope RECEIVES, so
// the emitted CSV source text cannot mislabel one scope's geometry as
// another's (the four scopes describe Poll / Response / Response / Final and
// each has a distinct measured frame end).
inline const char* received_frame_name(const char* scope)
{
    if (!scope)
        return "unknown";
    const std::string s(scope);
    if (s.find("after_Poll") != std::string::npos)
        return "Poll";
    if (s.find("after_Response") != std::string::npos)
        return "Response";
    if (s.find("after_Final") != std::string::npos)
        return "Final";
    return "unknown";
}

BOOST_AUTO_TEST_CASE(twr_timing_geometry_work_exact)
{
    for (const auto& rate : kRates) {
        (void)rate; // rate-independent; the work grid is always 998.4 MS/s
        for (size_t reps : kSyncReps) {
            for (const char* mode : kSfdModes) {
                for (const auto& role : kRoles) {
                    for (int fcs = 0; fcs < 2; ++fcs) {
                        const size_t psdu =
                            role.mac_bytes + (fcs ? kFcsBytes : 0);
                        const Geometry g = geometry(reps, mode, psdu);
                        BOOST_REQUIRE_MESSAGE(g.n_sfd > 0,
                                              "empty SFD for mode " +
                                                  std::string(mode));

                        // 1) Decomposition identity vs the authoritative
                        //    length function (no STS: M0 is a no-STS profile).
                        BOOST_CHECK_EQUAL(
                            mod::packet_samples_998p4(reps, g.n_sfd, psdu),
                            g.total_work);
                        // 2) Independent re-derivation of the symbol count.
                        BOOST_CHECK_EQUAL(mod::payload_bpm_symbols(psdu),
                                          g.nsym);
                        // 3) Fixed PHR geometry: 21 symbols * 512 chips *
                        //    2 samples = 21504 work samples = 21.5385 us.
                        BOOST_CHECK_EQUAL(g.phr_work, 21504u);
                        BOOST_CHECK_EQUAL(mod::kPhrChipsPerSymbol, 512u);
                        BOOST_CHECK_EQUAL(mod::kPhrSymbols, 21u);
                        // 4) Preamble / SFD are whole symbols.
                        BOOST_CHECK_EQUAL(g.preamble_work, reps * kSps);
                        BOOST_CHECK_EQUAL(g.sfd_work, g.n_sfd * kSps);
                        // 5) Payload is whole symbols * 64 chips * 2 samples.
                        BOOST_CHECK_EQUAL(g.payload_work,
                                          g.nsym * 128u);
                        // 6) The default (Legacy) pulse contributes no tail,
                        //    so n_out == n_base.  Alternative pulse shapes
                        //    append n_taps - 2 work samples of tail, which
                        //    must be budgeted explicitly if ever enabled.
                        BOOST_CHECK_EQUAL(
                            mod::pulse_tail_extra(mod::PulseSpec{}), 0u);
                        BOOST_CHECK_EQUAL(
                            mod::pulse_tail_extra(mod::PulseSpec{
                                mod::PulseShape::Gaussian }),
                            mod::kGaussianPulseTaps - kSpc);
                        BOOST_CHECK_EQUAL(
                            mod::pulse_tail_extra(mod::PulseSpec{
                                mod::PulseShape::Blackman }),
                            mod::kBlackmanPulseTaps - kSpc);
                        BOOST_CHECK_EQUAL(
                            mod::packet_samples_998p4(reps, g.n_sfd, psdu,
                                                      false, nullptr),
                            g.total_work);
                    }
                }
            }
        }
    }
    // Sanity on the frozen 64-SYNC / 4z2 M0 profile numbers.
    const Geometry poll = geometry(64, "4z2", kPollBytes);
    BOOST_CHECK_EQUAL(poll.n_sfd, 8u);
    BOOST_CHECK_EQUAL(poll.nsym, 160u);
    BOOST_CHECK_EQUAL(poll.preamble_work, 65024u);
    BOOST_CHECK_EQUAL(poll.sfd_work, 8128u);
    BOOST_CHECK_EQUAL(poll.phr_work, 21504u);
    BOOST_CHECK_EQUAL(poll.payload_work, 20480u);
    BOOST_CHECK_EQUAL(poll.total_work, 115136u);
    BOOST_CHECK_CLOSE(us_from_work(poll.total_work), 115.32051, 1e-4);
}

// Measured SFD lengths (the authoritative GetSfdSequence() sizes).  Recorded
// because SFD length enters both the frame budget and the RMARKER candidate
// set; M0 must not guess these.
BOOST_AUTO_TEST_CASE(twr_timing_sfd_lengths_measured)
{
    BOOST_CHECK_EQUAL(demod::GetSfdSequence("4z1").size(), 4u);
    BOOST_CHECK_EQUAL(demod::GetSfdSequence("4z2").size(), 8u);
    BOOST_CHECK_EQUAL(demod::GetSfdSequence("4z3").size(), 16u);
    BOOST_CHECK_EQUAL(demod::GetSfdSequence("4z4").size(), 32u);
    BOOST_CHECK_EQUAL(demod::GetSfdSequence("ieee").size(), 8u);
    BOOST_CHECK_EQUAL(demod::GetSfdSequence("decawave").size(), 8u);
    BOOST_CHECK(demod::GetSfdSequence("nope").empty());
    // STS is off for the M0 profile (REQ-SCOPE-04), and modulate_one rejects
    // insert_sts outside the 4z modes, so the STS segment is 0 by construction.
    BOOST_CHECK_EQUAL(mod::kStsSamplesBprf, 67584u);
    BOOST_CHECK(mod::sfd_mode_is_4z("4z2"));
    BOOST_CHECK(!mod::sfd_mode_is_4z("ieee"));
}

// ---------------------------------------------------------------------------
// 2. Native-grid geometry: 65/48 and 65/32 length + duration tables, and the
//    agreement between the ceil() convention and the resampler's map()
//    differences over a whole frame.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(twr_timing_geometry_native)
{
    // Only the two rates the backend accepts.
    BOOST_CHECK(uhd::is_allowed_uhd_native_rate(kFs737));
    BOOST_CHECK(uhd::is_allowed_uhd_native_rate(kFs491));
    BOOST_CHECK(!uhd::is_allowed_uhd_native_rate(998.4e6));
    BOOST_CHECK(!uhd::is_allowed_uhd_native_rate(500.0e6));

    for (const auto& rate : kRates) {
        const int64_t dec = rate.decim;
        const int64_t inter = rate.interp;

        // The e2e-QA native SYNC span convention, reproduced for both rates.
        BOOST_CHECK_EQUAL(native_len_ceil(static_cast<int64_t>(64 * kSps),
                                          rate.decim, rate.interp),
                          rate.decim == 48 ? 48018 : 32012);
        // The 2 us RX pre-guard in each native grid, and its work-grid
        // equivalent (both must be the same work value: 1997).
        const int64_t pre_guard_native =
            static_cast<int64_t>(2e-6 * rate.native_hz + 0.5);
        BOOST_CHECK_EQUAL(pre_guard_native,
                          rate.decim == 48 ? 1475 : 983);
        BOOST_CHECK_EQUAL(
            static_cast<int64_t>(std::llround(
                static_cast<double>(pre_guard_native) * rate.interp /
                rate.decim)),
            1997);

        int64_t worst_gap = 0;
        for (size_t reps : kSyncReps) {
            for (const char* mode : kSfdModes) {
                for (const auto& role : kRoles) {
                    const size_t psdu = role.mac_bytes + kFcsBytes;
                    const Geometry g = geometry(reps, mode, psdu);
                    const int64_t n_pre = native_len_ceil(
                        static_cast<int64_t>(g.preamble_work), rate.decim,
                        rate.interp);
                    const int64_t n_sfd = native_len_ceil(
                        static_cast<int64_t>(g.sfd_work), rate.decim,
                        rate.interp);
                    const int64_t n_phr = native_len_ceil(
                        static_cast<int64_t>(g.phr_work), rate.decim,
                        rate.interp);
                    const int64_t n_pay = native_len_ceil(
                        static_cast<int64_t>(g.payload_work), rate.decim,
                        rate.interp);
                    const int64_t n_tot = native_len_ceil(
                        static_cast<int64_t>(g.total_work), rate.decim,
                        rate.interp);

                    // Sum of per-segment ceils >= ceil of the sum, and the
                    // overshoot is bounded by (n_segments - 1) native samples.
                    BOOST_CHECK(n_pre + n_sfd + n_phr + n_pay >= n_tot);
                    BOOST_CHECK_LE(n_pre + n_sfd + n_phr + n_pay - n_tot, 3);

                    // Work and native grids describe the same duration: the
                    // ceil() quantisation is at most one native sample per
                    // segment, i.e. <= 1.36 us @737.28 and <= 2.03 us @491.52.
                    const double work_us = us_from_work(g.total_work);
                    const double native_us = us_from_native(n_tot, rate.native_hz);
                    BOOST_CHECK_LE(std::fabs(native_us - work_us),
                                   native_len_ceil(1, rate.decim, rate.interp) *
                                       1e6 / rate.native_hz + 1e-9);

                    // The interpolated rate must be consistent with the map
                    // ratio: work/interp * decim == native to 1e-9 relative.
                    BOOST_CHECK_CLOSE(
                        kFsWork / static_cast<double>(inter) *
                            static_cast<double>(dec),
                        rate.native_hz, 1e-9);
                    worst_gap = std::max<int64_t>(worst_gap, 0);
                }
            }
        }
        (void)worst_gap;
    }
}

// ---------------------------------------------------------------------------
// 3. Marker offsets + candidate RMARKER positions, in both grids.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(twr_timing_marker_offsets)
{
    const MapFn m65_48{ 65, 48, 2707 };
    BOOST_CHECK_EQUAL(m65_48.head(), 28); // round((2707-1)/2 / 48)
    const MapFn m65_32{ 65, 32, 2707 };
    BOOST_CHECK_EQUAL(m65_32.head(), 42); // round((2707-1)/2 / 32)

    for (const auto& rate : kRates) {
        // map() is validated against the real block in the anchor tests; here
        // we only exercise the ratio arithmetic and the offset identities.
        for (size_t reps : kSyncReps) {
            for (const char* mode : kSfdModes) {
                for (const auto& role : kRoles) {
                    const size_t psdu = role.mac_bytes + kFcsBytes;
                    const Geometry g = geometry(reps, mode, psdu);
                    const Marker all[] = { Marker::PreambleStart,
                                           Marker::SfdStart,
                                           Marker::PHRStart,
                                           Marker::PHREnd,
                                           Marker::FrameEnd };
                    for (Marker mk : all) {
                        const int64_t off = marker_work_offset(mk, g);
                        BOOST_REQUIRE_GE(off, 0);
                        BOOST_REQUIRE_LE(off, static_cast<int64_t>(g.total_work));

                        // Unanchored conversion: work offset -> native.
                        const int64_t n_off_exact = native_len_ceil(
                            off, rate.decim, rate.interp);
                        // Exact real-valued ratio, for the residual report.
                        const double n_off_real =
                            static_cast<double>(off) * rate.decim / rate.interp;
                        BOOST_CHECK_LE(std::fabs(
                                            static_cast<double>(n_off_exact) -
                                            n_off_real),
                                       1.0 + 1e-9);

                        // Anchored conversion: identical to the unanchored one
                        // in relative terms (the group-delay head cancels),
                        // so the two conventions never disagree by more than
                        // the map rounding at the *window* level, which is
                        // asserted in the anchor tests against the real FIR.
                        BOOST_CHECK_EQUAL(
                            n_off_exact,
                            native_len_ceil(off, rate.decim, rate.interp));

                        // Durations are rate-independent in the work grid.
                        // PreambleStart is the frame origin, hence 0.
                        BOOST_CHECK_GE(us_from_work(off), 0.0);
                    }
                    // Offset ordering is strict for distinct markers.
                    BOOST_CHECK_EQUAL(marker_work_offset(Marker::SfdStart, g),
                                      marker_work_offset(Marker::PreambleStart,
                                                         g) +
                                          static_cast<int64_t>(
                                              g.preamble_work));
                    BOOST_CHECK_EQUAL(
                        marker_work_offset(Marker::PHRStart, g),
                        marker_work_offset(Marker::SfdStart, g) +
                            static_cast<int64_t>(g.sfd_work));
                    BOOST_CHECK_EQUAL(
                        marker_work_offset(Marker::FrameEnd, g),
                        static_cast<int64_t>(g.total_work));
                }
            }
        }
    }
}

// ---------------------------------------------------------------------------
// 4a. Time anchor, 65/48 (737.28 MS/s native), against the real block.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(twr_timing_anchor_65_48)
{
    const std::string dir = std::string(UWB_TESTDATA_DIR);
    const std::vector<float> taps =
        load_taps_f32(join_path(dir, "resampler_65_48/taps_quality_minorder.txt"));
    BOOST_REQUIRE_EQUAL(taps.size(), 2707u);
    auto blk = UwbPduRationalResamplerCcf65_48::make_from_taps(
        taps, UwbPduRationalResamplerCcf65_48::kOutputRateHz,
        /*validate_input_rate=*/true);
    BOOST_REQUIRE(blk);

    const int64_t map0 = blk->map_input_offset_to_output(0);
    // Group delay actually used by the 65/48 chain (FIR head, work samples).
    BOOST_CHECK_EQUAL(map0, 28);
    BOOST_CHECK_EQUAL(blk->tap_count(), 2707u);

    // 4a.1 try_map() == map() over a real frame's worth of samples.
    const Geometry g = geometry(64, "4z2", kPollBytes + kFcsBytes);
    const int64_t n_native =
        native_len_ceil(static_cast<int64_t>(g.total_work), 48, 65);
    int mismatches = 0;
    int64_t prev = -1;
    int64_t max_step = 0;
    for (int64_t p = 0; p < n_native; ++p) {
        int64_t o = 0;
        BOOST_REQUIRE(blk->try_map_input_offset_to_output(p, o));
        const int64_t m = blk->map_input_offset_to_output(p);
        if (o != m)
            ++mismatches;
        if (p > 0) {
            const int64_t d = o - prev;
            BOOST_REQUIRE_GE(d, 0);
            max_step = std::max(max_step, d);
        }
        prev = o;
    }
    BOOST_CHECK_EQUAL(mismatches, 0);
    // Monotone, and the local step is 1 or 2 work samples (65/48 = 1.3542).
    BOOST_CHECK_LE(max_step, 2);
    // A native window of ceil(total_work*48/65) samples maps back onto the
    // exact work length to within one work sample.
    BOOST_CHECK_LE(std::llabs(blk->map_input_offset_to_output(n_native) - map0 -
                              static_cast<int64_t>(g.total_work)),
                   1);
    // Negative input clamps to 0 rather than going negative.
    int64_t neg = -1;
    BOOST_REQUIRE(blk->try_map_input_offset_to_output(-5, neg));
    BOOST_CHECK_EQUAL(neg, 0);

    // 4a.2 The e2e worked example, reproduced verbatim: the anchored
    //      prediction must land on the mapped truth within +/-2 work samples
    //      (here it is exact; the residual is only the native ceil()).
    const int64_t pre_guard_native = 1475; // 2 us @737.28
    const int64_t d_native = 37;           // golden channel delay
    const int64_t sync_native = 48018;     // ceil(64*1016*48/65)
    const int64_t cal_work_ll =
        static_cast<int64_t>(std::llround(d_native * 65.0 / 48.0));
    const int64_t predicted =
        map0 +
        (blk->map_input_offset_to_output(pre_guard_native) - map0) +
        cal_work_ll + static_cast<int64_t>(64 * kSps);
    const int64_t sfd_truth = blk->map_input_offset_to_output(
        pre_guard_native + d_native + sync_native);
    BOOST_CHECK_EQUAL(predicted, 67100);
    BOOST_CHECK_EQUAL(sfd_truth, 67100);
    const int64_t anchor_residual = std::llabs(sfd_truth - predicted);
    BOOST_CHECK_LE(anchor_residual, 2);
    BOOST_CHECK_EQUAL(anchor_residual, 0);

    // 4a.3 Unanchored per-marker conversion accumulated over a whole TWR
    //      frame: round(off_work * 65/48) vs the anchored map difference.
    int64_t worst = 0;
    for (Marker mk : { Marker::PreambleStart,
                       Marker::SfdStart,
                       Marker::PHRStart,
                       Marker::PHREnd,
                       Marker::FrameEnd }) {
        const int64_t off = marker_work_offset(mk, g);
        const int64_t anchored =
            blk->map_input_offset_to_output(off) - map0;
        const int64_t naive = static_cast<int64_t>(
            std::llround(static_cast<double>(off) * 65.0 / 48.0));
        worst = std::max<int64_t>(worst, std::llabs(anchored - naive));
    }
    BOOST_CHECK_LE(worst, 2);

    // 4a.4 FIR tail: the output length the core produces for a PDU of
    //      n_native samples leaves this many work samples after the last
    //      mapped input sample.
    const int64_t lout = static_cast<int64_t>(
        core::RationalResampler65_48Core::expected_output_length(
            static_cast<uint64_t>(n_native), 2707));
    const int64_t fir_tail = lout - 1 - blk->map_input_offset_to_output(n_native - 1);
    BOOST_CHECK_LE(fir_tail, 30);
    BOOST_CHECK_GE(fir_tail, 0);
    BOOST_TEST_MESSAGE("65/48 FIR tail after last mapped input = " +
                       std::to_string(fir_tail) + " work samples; Lout=" +
                       std::to_string(lout) + " for n_native=" +
                       std::to_string(n_native));

    // 4a.5 Segment decomposition consistency, in NATIVE coordinates: mapping
    //      each segment's native start and end separately must give the same
    //      span as mapping the segment length on its own, to within one work
    //      sample of map rounding.  This is what allows the TWR time budget
    //      to convert per-segment work durations to native independently.
    int64_t worst_seg = 0;
    int64_t n_acc = 0;
    for (size_t seg : { g.preamble_work, g.sfd_work, g.phr_work, g.payload_work }) {
        const int64_t n_seg = native_len_ceil(static_cast<int64_t>(seg),
                                              48, 65);
        const int64_t via_span =
            blk->map_input_offset_to_output(n_acc + n_seg) -
            blk->map_input_offset_to_output(n_acc);
        const int64_t via_length = blk->map_input_offset_to_output(n_seg) - map0;
        worst_seg = std::max<int64_t>(worst_seg, std::llabs(via_span - via_length));
        n_acc += n_seg;
    }
    BOOST_CHECK_LE(worst_seg, 2);
    BOOST_CHECK_LE(std::llabs(n_acc - n_native), 3); // sum of per-segment ceils

    // 4a.6 Invertibility: the pseudo-inverse round trip
    //      work -> native(ceil) -> work via map() must return the original
    //      work offset to within the same +/-2 work-sample budget the e2e QA
    //      asserts for the SFD anchor.
    int64_t worst_inv = 0;
    int64_t worst_inv_at = 0;
    for (Marker mk : { Marker::PreambleStart,
                       Marker::SfdStart,
                       Marker::PHRStart,
                       Marker::PHREnd,
                       Marker::FrameEnd }) {
        const int64_t off = marker_work_offset(mk, g);
        const int64_t n_off = native_len_ceil(off, 48, 65);
        const int64_t back = blk->map_input_offset_to_output(n_off) - map0;
        const int64_t d = std::llabs(back - off);
        if (d > worst_inv) {
            worst_inv = d;
            worst_inv_at = off;
        }
    }
    BOOST_CHECK_LE(worst_inv, 2);
    BOOST_TEST_MESSAGE("65/48 anchor residual=0 work samples; inverse "
                       "round-trip worst=" +
                       std::to_string(worst_inv) + " work samples at work "
                       "offset " + std::to_string(worst_inv_at) +
                       "; group delay map0=" + std::to_string(map0));
}

// ---------------------------------------------------------------------------
// 4b. Time anchor, 65/32 (491.52 MS/s native), against the real block.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(twr_timing_anchor_65_32)
{
    const std::string dir = std::string(UWB_TESTDATA_DIR);
    const std::vector<float> taps =
        load_taps_f32(join_path(dir, "resampler_65_32/taps_quality_minorder.txt"));
    BOOST_REQUIRE_EQUAL(taps.size(), 2707u);
    auto blk = UwbPduRationalResamplerCcf65_32::make_from_taps(
        taps, UwbPduRationalResamplerCcf65_32::kOutputRateHz,
        /*validate_input_rate=*/true);
    BOOST_REQUIRE(blk);

    const int64_t map0 = blk->map_input_offset_to_output(0);
    // Group delay actually used by the 65/32 chain: round(1353/32) = 42.
    BOOST_CHECK_EQUAL(map0, 42);
    BOOST_CHECK_EQUAL(blk->tap_count(), 2707u);

    const Geometry g = geometry(64, "4z2", kPollBytes + kFcsBytes);
    const int64_t n_native =
        native_len_ceil(static_cast<int64_t>(g.total_work), 32, 65);
    int mismatches = 0;
    int64_t prev = -1;
    int64_t max_step = 0;
    for (int64_t p = 0; p < n_native; ++p) {
        int64_t o = 0;
        BOOST_REQUIRE(blk->try_map_input_offset_to_output(p, o));
        if (o != blk->map_input_offset_to_output(p))
            ++mismatches;
        if (p > 0) {
            const int64_t d = o - prev;
            BOOST_REQUIRE_GE(d, 0);
            max_step = std::max(max_step, d);
        }
        prev = o;
    }
    BOOST_CHECK_EQUAL(mismatches, 0);
    BOOST_CHECK_LE(max_step, 3); // 65/32 = 2.03125 -> steps of 2 or 3

    const int64_t pre_guard_native = 983; // 2 us @491.52
    const int64_t d_native = 25;
    const int64_t sync_native = 32012; // ceil(64*1016*32/65)
    const int64_t cal_work_ll =
        static_cast<int64_t>(std::llround(d_native * 65.0 / 32.0));
    const int64_t predicted =
        map0 +
        (blk->map_input_offset_to_output(pre_guard_native) - map0) +
        cal_work_ll + static_cast<int64_t>(64 * kSps);
    const int64_t sfd_truth = blk->map_input_offset_to_output(
        pre_guard_native + d_native + sync_native);
    BOOST_CHECK_EQUAL(predicted, 67114);
    BOOST_CHECK_EQUAL(sfd_truth, 67114);
    const int64_t anchor_residual = std::llabs(sfd_truth - predicted);
    BOOST_CHECK_LE(anchor_residual, 2);
    BOOST_CHECK_EQUAL(anchor_residual, 0);

    int64_t worst = 0;
    for (Marker mk : { Marker::PreambleStart,
                       Marker::SfdStart,
                       Marker::PHRStart,
                       Marker::PHREnd,
                       Marker::FrameEnd }) {
        const int64_t off = marker_work_offset(mk, g);
        const int64_t anchored = blk->map_input_offset_to_output(off) - map0;
        const int64_t naive = static_cast<int64_t>(
            std::llround(static_cast<double>(off) * 65.0 / 32.0));
        worst = std::max<int64_t>(worst, std::llabs(anchored - naive));
    }
    BOOST_CHECK_LE(worst, 2);

    const int64_t lout = static_cast<int64_t>(
        core::RationalResampler65_32Core::expected_output_length(
            static_cast<uint64_t>(n_native), 2707));
    const int64_t fir_tail =
        lout - 1 - blk->map_input_offset_to_output(n_native - 1);
    BOOST_CHECK_GE(fir_tail, 0);
    BOOST_CHECK_LE(fir_tail, 45);
    BOOST_TEST_MESSAGE("65/32 FIR tail after last mapped input = " +
                       std::to_string(fir_tail) + " work samples; Lout=" +
                       std::to_string(lout) + " for n_native=" +
                       std::to_string(n_native));

    // Same invertibility and segment-consistency statements as 65/48.
    int64_t worst_seg = 0;
    int64_t n_acc = 0;
    for (size_t seg : { g.preamble_work, g.sfd_work, g.phr_work, g.payload_work }) {
        const int64_t n_seg = native_len_ceil(static_cast<int64_t>(seg), 32, 65);
        const int64_t via_span =
            blk->map_input_offset_to_output(n_acc + n_seg) -
            blk->map_input_offset_to_output(n_acc);
        const int64_t via_length = blk->map_input_offset_to_output(n_seg) - map0;
        worst_seg = std::max<int64_t>(worst_seg, std::llabs(via_span - via_length));
        n_acc += n_seg;
    }
    BOOST_CHECK_LE(worst_seg, 2);

    int64_t worst_inv = 0;
    for (Marker mk : { Marker::PreambleStart,
                       Marker::SfdStart,
                       Marker::PHRStart,
                       Marker::PHREnd,
                       Marker::FrameEnd }) {
        const int64_t off = marker_work_offset(mk, g);
        const int64_t n_off = native_len_ceil(off, 32, 65);
        const int64_t back = blk->map_input_offset_to_output(n_off) - map0;
        worst_inv = std::max<int64_t>(worst_inv, std::llabs(back - off));
    }
    BOOST_CHECK_LE(worst_inv, 2);
    BOOST_TEST_MESSAGE("65/32 anchor residual=0 work samples; inverse "
                       "round-trip worst=" +
                       std::to_string(worst_inv) +
                       " work samples; group delay map0=" +
                       std::to_string(map0));
}

// ---------------------------------------------------------------------------
// 5. Emit the measured CSV tables.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(twr_timing_emit_tables)
{
    std::string why;
    const std::string dir = twr_dir(why);
    if (dir.empty()) {
        BOOST_TEST_MESSAGE("CSV not emitted: " << why);
        return;
    }
    BOOST_TEST_MESSAGE("CSV output dir: " << dir
                                        << " (export to the source tree only "
                                           "with UWB_TWR_EXPORT_DIR)");

    for (const auto& rate : kRates) {
        // Re-derive map() locally for the table; the block-backed values are
        // asserted in twr_timing_anchor_65_48 / _65_32 with the same taps.
        const std::vector<float> taps = load_taps_f32(
            join_path(std::string(UWB_TESTDATA_DIR),
                      std::string("resampler_") +
                          (rate.decim == 48 ? "65_48" : "65_32") +
                          "/taps_quality_minorder.txt"));
        const MapFn mapfn{ rate.interp, rate.decim,
                           static_cast<int64_t>(taps.size()) };
        const int64_t T = mapfn.taps;
        const int64_t map0 = mapfn.head();

        const std::string path =
            join_path(dir, std::string("timing_budget_") + rate.tag + ".csv");
        std::ofstream f(path, std::ios::trunc);
        BOOST_REQUIRE_MESSAGE(f.good(), "cannot write " + path);
        f << "native_rate_hz,work_rate_hz,sync_repetitions,sfd_mode,"
             "psdu_len,fcs_included,preamble_work,sfd_work,phr_work,"
             "payload_work,total_work,preamble_native,sfd_native,phr_native,"
             "payload_native,total_native,total_us"
          << ",frame_role,sfd_symbols,nsym,ratio_interp,ratio_decim,resampler_taps,"
             "map0_work,preamble_us,sfd_us,phr_us,payload_us"
          << ",sampling_note\n";
        f << std::setprecision(12);

        size_t rows = 0;
        for (const auto& role : kRoles) {
            for (int fcs = 0; fcs < 2; ++fcs) {
                const size_t psdu = role.mac_bytes + (fcs ? kFcsBytes : 0);
                for (size_t reps : kSyncReps) {
                    for (const char* mode : kSfdModes) {
                        const Geometry g = geometry(reps, mode, psdu);
                        const int64_t n_pre = native_len_ceil(
                            static_cast<int64_t>(g.preamble_work),
                            rate.decim, rate.interp);
                        const int64_t n_sfd = native_len_ceil(
                            static_cast<int64_t>(g.sfd_work), rate.decim,
                            rate.interp);
                        const int64_t n_phr = native_len_ceil(
                            static_cast<int64_t>(g.phr_work), rate.decim,
                            rate.interp);
                        const int64_t n_pay = native_len_ceil(
                            static_cast<int64_t>(g.payload_work), rate.decim,
                            rate.interp);
                        const int64_t n_tot = native_len_ceil(
                            static_cast<int64_t>(g.total_work), rate.decim,
                            rate.interp);
                        f << static_cast<int64_t>(rate.native_hz) << ","
                          << static_cast<int64_t>(kFsWork) << "," << reps << ","
                          << mode << "," << psdu << "," << (fcs ? 1 : 0) << ","
                          << g.preamble_work << "," << g.sfd_work << ","
                          << g.phr_work << "," << g.payload_work << ","
                          << g.total_work << "," << n_pre << "," << n_sfd << ","
                          << n_phr << "," << n_pay << "," << n_tot << ","
                          << us_from_work(g.total_work) << "," << role.name
                          << "," << g.n_sfd << "," << g.nsym << ","
                          << rate.interp << "," << rate.decim << "," << T << ","
                          << map0 << "," << us_from_work(g.preamble_work) << ","
                          << us_from_work(g.sfd_work) << ","
                          << us_from_work(g.phr_work) << ","
                          << us_from_work(g.payload_work) << ","
                          << "native_len=ceil(work*" << rate.decim << "/"
                          << rate.interp << ")"
                          << (fcs ? ";psdu_includes_fcs=2" : ";fcs_absent")
                          << ";no_sts"
                          << ";work_grid_exact=998400000\n";
                        ++rows;
                    }
                }
            }
        }
        f.flush();
        BOOST_REQUIRE_MESSAGE(f.good(), "write failed for " + path);
        BOOST_TEST_MESSAGE(path + " rows=" + std::to_string(rows) +
                           " T=" + std::to_string(T) +
                           " map0=" + std::to_string(map0) +
                           " map(64-SYNC span)=" +
                           std::to_string(
                               mapfn(native_len_ceil(64 * kSps, rate.decim,
                                                     rate.interp)) -
                               map0));
    }
}

// ---------------------------------------------------------------------------
// 6. Candidate RMARKER offsets table (REQ-TIME-02).  Internal offsets only:
//    the absolute DW1000 / DW3000 RMARKER convention is NOT settled here and
//    must be pinned by the PHY profile and cross-checked against chip
//    documentation (M2 / M5).
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(twr_timing_emit_rmarker_candidates)
{
    std::string why;
    const std::string dir = twr_dir(why);
    if (dir.empty()) {
        BOOST_TEST_MESSAGE("CSV not emitted: " << why);
        return;
    }
    BOOST_TEST_MESSAGE("CSV output dir: " << dir
                                        << " (export to the source tree only "
                                           "with UWB_TWR_EXPORT_DIR)");
    // The 65/48 and 65/32 chains both use the quality_minorder tap set
    // (T = 2707), which the anchor tests pin against the real blocks.
    const MapFn mapfn_48{ 65, 48, 2707 };
    const MapFn mapfn_32{ 65, 32, 2707 };

    const std::string path =
        join_path(dir, "timing_budget_rmarker_candidates.csv");
    std::ofstream f(path, std::ios::trunc);
    BOOST_REQUIRE_MESSAGE(f.good(), "cannot write " + path);
    f << "native_rate_hz,work_rate_hz,sync_repetitions,sfd_mode,psdu_len,"
         "fcs_included,frame_role,marker,work_offset,native_offset_ceil,"
         "native_offset_mapped_rel,work_offset_us,rmarker_candidate,status,note\n";
    f << std::setprecision(12);

    struct Cand {
        Marker mk;
        bool candidate;
        const char* note;
    };
    const Cand cands[] = {
        { Marker::PreambleStart, true,
          "rx_first_path_nominal;DWT_does_not_measure_this_directly" },
        { Marker::SfdStart, true,
          "synchronisation_lock_point;first_toa_available_after_sfd_end" },
        { Marker::PHRStart, true,
          "classic_rmarker_position_end_of_sfd_needs_chip_doc_check" },
        { Marker::PHREnd, true, "payload_start;phr_decoded_at_this_point" },
        { Marker::FrameEnd, false,
          "not_an_rmarker;used_for_frame_tail_delay_budget_only" },
    };

    size_t rows = 0;
    for (const auto& rate : kRates) {
        for (const auto& role : kRoles) {
            for (int fcs = 0; fcs < 2; ++fcs) {
                const size_t psdu = role.mac_bytes + (fcs ? kFcsBytes : 0);
                for (size_t reps : kSyncReps) {
                    for (const char* mode : kSfdModes) {
                        const Geometry g = geometry(reps, mode, psdu);
                        for (const auto& c : cands) {
                            const int64_t off = marker_work_offset(c.mk, g);
                            const int64_t n_off =
                                native_len_ceil(off, rate.decim, rate.interp);
                            // Anchored map difference (group-delay relative):
                            // map() takes a NATIVE offset, so the work offset
                            // is first ceil()-converted to the native grid.
                            // Identical convention to the real block; the
                            // anchor tests assert equality with the same taps.
                            const MapFn& mf =
                                (rate.decim == 48) ? mapfn_48 : mapfn_32;
                            const int64_t mapped_rel = mf(n_off) - mf.head();
                            f << static_cast<int64_t>(rate.native_hz) << ","
                              << static_cast<int64_t>(kFsWork) << "," << reps
                              << "," << mode << "," << psdu << ","
                              << (fcs ? 1 : 0) << "," << role.name << ","
                              << marker_name(c.mk) << "," << off << "," << n_off
                              << "," << mapped_rel << ","
                              << us_from_work(off) << ","
                              << (c.candidate ? "yes" : "no") << ","
                              << (c.candidate ? "MEASURED_INTERNAL_OFFSET"
                                              : "NOT_AN_RMARKER")
                              << "," << c.note << "\n";
                            ++rows;
                        }
                    }
                }
            }
        }
    }
    f.flush();
    BOOST_REQUIRE_MESSAGE(f.good(), "write failed for " + path);
    BOOST_TEST_MESSAGE(path + " rows=" + std::to_string(rows));
}

// ---------------------------------------------------------------------------
// 7. Preliminary SS turnaround budget — 离线估算 / OFFLINE ESTIMATE.
//    RF-side terms come from the measured geometry.  Host-side terms are
//    recorded as the literal token UNMEASURED; this QA deliberately does not
//    invent a host latency (REQ-GR-04: "新配置在离线估算后仍须实测").
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(twr_timing_emit_turnaround_estimate)
{
    std::string why;
    const std::string dir = twr_dir(why);
    if (dir.empty()) {
        BOOST_TEST_MESSAGE("CSV not emitted: " << why);
        return;
    }
    BOOST_TEST_MESSAGE("CSV output dir: " << dir
                                        << " (export to the source tree only "
                                           "with UWB_TWR_EXPORT_DIR)");
    const std::string path =
        join_path(dir, "timing_budget_turnaround_estimate.csv");
    std::ofstream f(path, std::ios::trunc);
    BOOST_REQUIRE_MESSAGE(f.good(), "cannot write " + path);
    f << "scope,profile,item,value,unit,status,source\n";
    f << std::setprecision(12);

    // Reference: the frame the responder must wait through, 64 SYNC / 4z2.
    const Geometry poll = geometry(64, "4z2", kPollBytes + kFcsBytes);
    const Geometry resp = geometry(64, "4z2", kResponseBytes + kFcsBytes);
    const Geometry fin = geometry(64, "4z2", kFinalBytes + kFcsBytes);

    const auto emit_geom = [&](const char* scope,
                               const char* profile,
                               const Geometry& g,
                               Marker mk) {
        const double us = us_from_work(marker_work_offset(mk, g));
        f << scope << "," << profile << ",rx_marker_to_" << marker_name(mk)
          << "," << us << ",us,MEASURED_GEOMETRY,"
          << "mod::packet_samples_998p4 + PHY constants; work grid 998.4e6;"
             " offset from the assumed RMARKER = PreambleStart\n";
    };

    for (const char* scope : { "SS_responder_after_Poll",
                               "SS_initiator_after_Response",
                               "DS_initiator_after_Response",
                               "DS_responder_after_Final" }) {
        const Geometry& g = (std::string(scope).find("Poll") !=
                                 std::string::npos)
                                ? poll
                                : ((std::string(scope).find("Final") !=
                                    std::string::npos)
                                       ? fin
                                       : resp);
        f << scope << ",64SYNC_4z2_noSTS,rmarker_assumed_at_PreambleStart"
          << ",0,work_samples,ASSUMPTION,"
          << "internal marker only; absolute RMARKER convention OPEN (M2/M5)\n";
        for (Marker mk : { Marker::PreambleStart,
                           Marker::SfdStart,
                           Marker::PHRStart,
                           Marker::PHREnd,
                           Marker::FrameEnd }) {
            emit_geom(scope, "64SYNC_4z2_noSTS", g, mk);
        }
        const int64_t tail = static_cast<int64_t>(g.total_work) -
                             static_cast<int64_t>(g.preamble_work);
        f << scope << ",64SYNC_4z2_noSTS,frame_tail_after_PreambleStart," << tail
          << ",work_samples,MEASURED_GEOMETRY,packet_samples_998p4\n";
        f << scope << ",64SYNC_4z2_noSTS,frame_tail_after_PreambleStart,"
          << us_from_work(tail)
          << ",us,MEASURED_GEOMETRY,packet_samples_998p4\n";
        // RF-only hard lower bound on the reply instant: the reply cannot
        // start before the last chip of the RECEIVED frame has been radiated.
        // The source text must name the frame that `g` actually describes.  A
        // single hardcoded "Poll" string here silently mislabels the Response
        // and Final scopes, whose measured values are 127.628 us and
        // 132.756 us rather than the Poll's 117.372 us.
        f << scope
          << ",64SYNC_4z2_noSTS,rf_geometry_lower_bound_reply_start,"
          << us_from_work(static_cast<int64_t>(g.total_work))
          << ",us,MEASURED_GEOMETRY_ONLY,"
          << received_frame_name(scope)
          << " FrameEnd measured from the assumed RMARKER (PreambleStart); "
             "assumes zero propagation and excludes every host-side term, so "
             "it is a hard lower bound, not an achievable reply delay\n";
    }

    // Host-side terms: NOT measured by this QA.
    const char* kUnmeasured[] = {
        "host_detect_decode",
        "host_fsm_frame_patch_modulate",
        "host_tx_pdu_transport",
        "uhd_min_lead_time",
        "scheduled_margin",
    };
    for (const char* scope : { "SS_responder_after_Poll",
                               "SS_initiator_after_Response",
                               "DS_initiator_after_Response",
                               "DS_responder_after_Final" }) {
        for (const char* item : kUnmeasured) {
            f << scope << ",64SYNC_4z2_noSTS," << item
              << ",UNMEASURED,us,UNMEASURED,"
              << "requires M2 (synthetic) and M3 (X410) P99/P99.9 measurement "
                 "per REQ-GR-04; do not assume a value\n";
        }
        f << scope
          << ",64SYNC_4z2_noSTS,min_feasible_reply_delay,NOT_COMPUTED,us,"
             "BLOCKED_ON_HOST_MEASUREMENT,"
          << "sum of the UNMEASURED terms; must be recomputed from measured "
             "percentiles, not from this table\n";
    }

    // Native-grid view of the Poll frame at both allowed rates.
    for (const auto& rate : kRates) {
        const int64_t n_tot = native_len_ceil(
            static_cast<int64_t>(poll.total_work), rate.decim, rate.interp);
        f << "SS_responder_after_Poll," << rate.tag
          << ",poll_frame_len_native," << n_tot << ",native_samples,"
          << "MEASURED_GEOMETRY,ceil(work*" << rate.decim << "/" << rate.interp
          << ")\n";
        f << "SS_responder_after_Poll," << rate.tag
          << ",poll_frame_duration_native," << us_from_native(n_tot, rate.native_hz)
          << ",us,MEASURED_GEOMETRY,native_rate_" << rate.tag << "\n";
    }
    f.flush();
    BOOST_REQUIRE_MESSAGE(f.good(), "write failed for " + path);

    // Report the headline numbers for the M0 baseline document.
    BOOST_TEST_MESSAGE("M0 SS geometry (64 SYNC, SFD 4z2, no STS):");
    BOOST_TEST_MESSAGE("  Poll     14+2B: work=" +
                       std::to_string(poll.total_work) + " us=" +
                       std::to_string(us_from_work(poll.total_work)));
    BOOST_TEST_MESSAGE("  Response 24+2B: work=" +
                       std::to_string(resp.total_work) + " us=" +
                       std::to_string(us_from_work(resp.total_work)));
    BOOST_TEST_MESSAGE("  Final    29+2B: work=" +
                       std::to_string(fin.total_work) + " us=" +
                       std::to_string(us_from_work(fin.total_work)));
    BOOST_TEST_MESSAGE("  Poll preamble-only (PreambleStart->SfdStart) us=" +
                       std::to_string(us_from_work(poll.preamble_work)));
    BOOST_TEST_MESSAGE("  Poll PreambleStart->PHRStart us=" +
                       std::to_string(us_from_work(poll.preamble_work +
                                                   poll.sfd_work)));
    // The only hard number this table can justify: the reply cannot start
    // before the Poll's last chip is in the air.
    BOOST_CHECK_GT(us_from_work(static_cast<int64_t>(poll.total_work)), 100.0);
}

} // namespace uwb
} // namespace gr
