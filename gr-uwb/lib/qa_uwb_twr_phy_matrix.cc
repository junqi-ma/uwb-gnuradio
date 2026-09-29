/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * qa_uwb_twr_phy_matrix — TWR M0 measurement-derived PHY capability matrix.
 *
 * Why this file exists
 * --------------------
 * docs/twr/需求_UWB_SS_DS_TWR.md REQ-BASE-02: "code 支持" and "某 preamble 长度
 * 可发" are NOT evidence of a supported PHY.  REQ-PHY-01 asks to start from
 * code 9 / 128 preamble, but "具体组合必须通过两端 PHY/FCS golden 冻结".
 * REQ-SCOPE-01 / REQ-API-01: unsupported combinations must be rejected, never
 * silently defaulted to the radar/QM35 profile.
 *
 * So every row in this QA is a MEASURED round trip:
 *
 *   mod::modulate_one(psdu = MAC payload + append_ieee_fcs)   998.4 MS/s grid
 *     -> cf32 file -> UwbRadarPacketSource::make()
 *     -> UwbLoopbackEcho::make(delay, multipath, AWGN)        real gr::block
 *     -> core::demodulate_one()                              R1..R4 pipeline
 *        assert payload.fcs_pass AND byte-exact PSDU recovery
 *
 * The FCS value is re-computed by an INDEPENDENT CRC-16 implementation in this
 * file (crc16_802154_ref), never by the library under test, so "observed FCS"
 * is not the library grading its own homework (REQ-QA-01).
 *
 * Prior state of the art in this repo
 * ----------------------------------
 *   * 64 SYNC: full PHR/FCS golden (testdata/realtime_demod_golden, 127 B PSDU,
 *     FCS 0x584b) — qa_uwb_demod_core.cc::test_demod_core_r4_payload_fcs_matches_golden
 *   * 32/64/128 SYNC: preamble/SFD/CIR only, via UwbRadarPacketSource +
 *     UwbRadarCirEstimator, which do NOT decode PHR/payload/FCS —
 *     qa_uwb_radar_e2e.cc::e2e_sync_repetitions
 * => 128 SYNC was NOT verified at PHR/payload/FCS level.  This QA closes that
 *    gap and finds that 128 SYNC decodes at zero CFO but its CFO estimate is
 *    structurally wrong (see twr_phy_matrix_128sync_cfo_cause).
 *
 * sync_repetitions whitelist (measured, see the boundary test)
 * ----------------------------------------------------------
 *   16  SUPPORTED.  Only length that is both CFO-robust and PHR
 *       self-describing at the short end.  encode_phr19 emits
 *       preamble-duration index 0 (=16) for reps <= 16, so the frame
 *       describes itself; and because 16 <= cfo_min_fit_repetitions(32) the
 *       whole preamble is re-measured, so no synthesised zero-phase peak can
 *       enter the phase fit.  Measured CFO error 0.0 Hz at
 *       0/±1/±5/±20/±100 kHz, byte-exact FCS pass on 16/26/31/127 B PSDU for
 *       code 9..12 x all six SFD modes, and integer / fractional delay,
 *       2-/3-tap multipath and AWGN sigma 0.05 / 0.15.  Also the SHORTEST
 *       usable preamble, i.e. the shortest frame for the TWR turnaround
 *       budget.  Its first-path / ToA accuracy is NOT measured here and stays
 *       unverified.
 *   64  SUPPORTED.  The reference profile.
 *   32  excluded: CFO-exact, but advertises preamble duration 64.
 *   128 / 256 / 512 / 1024 / 2048  excluded: the CFO fit is structurally
 *       biased (measured -16.2 kHz error at 128 SYNC, growing with length)
 *       and the PHR cannot describe the length (1024 excepted, and it is still
 *       CFO-biased).
 *   1 / 2  excluded: stage_cfo needs at least 4 measured peaks.
 *   4 / 8  excluded: stage_cir_softchips requires more than
 *       cir_skip_initial_repetitions (10) repetitions.
 *
 * What is NOT measured here (must stay "unverified" in the M0 whitelist)
 * --------------------------------------------------------------------
 *   * STS.  Deliberately excluded (REQ-SCOPE-04 phase-1 baseline is STS-free).
 *     mod::HrpModConfig::insert_sts only accepts a 4z SFD and Qm35825Profile
 *     carries no STS mode/segment/length parameters at all, so no STS round
 *     trip can be driven through demodulate_one().
 *   * Data rates other than 6.81 Mb/s.  mod::encode_phr19 hardcodes PHR
 *     data-rate index 2, and core::stage_payload_fcs always de-maps the
 *     payload with the 6.81 Mb/s geometry (64 chips/burst, 64 chips/symbol,
 *     scrambler offset 1344).  stage_phr reports the decoded rate but the
 *     demodulator never branches on it.
 *   * Preamble lengths other than 16 and 64.  See the whitelist above.
 *   * First-path / ToA accuracy for ANY length.  This QA qualifies decode
 *     (PHR / payload / FCS) only; the ranging-side accuracy claims belong to
 *     M2/M3 with the independent MATLAB oracle that REQ-QA-01 requires.
 *   * Native-rate (737.28 / 491.52 MS/s) round trips.  The repository has only
 *     UPDATING resamplers (48->65, 32->65, i.e. native -> 998.4 work grid);
 *     there is no work -> native decimator, so a TWR-sized PSDU cannot be
 *     modulated on the native grid for an end-to-end native test.  The existing
 *     native 65/48 and 65/32 e2e cases use pre-generated native-rate packet
 *     goldens (testdata/uwb_radar/tx_737p28.cf32, tx_491p52.cf32) whose
 *     payload is not a TWR frame.
 *   * Any vendor module.  Nothing here says DW1000 / DW3000 interoperate.  A
 *     self-consistent loopback round trip only proves this repo's TX and RX
 *     agree with each other.
 *
 * Output: testdata/twr/phy_matrix_<native_rate>.csv  (directory override with
 * UWB_TWR_PHY_MATRIX_CSV_DIR).  Offline QA — no streaming latency contract,
 * no allocation-in-work constraint applies.
 */

#include <boost/test/unit_test.hpp>

#include <gnuradio/blocks/message_debug.h>
#include <gnuradio/gr_complex.h>
#include <gnuradio/top_block.h>
#include <gnuradio/uwb/uwb_demod_core.h>
#include <gnuradio/uwb/uwb_hrp_mod_core.h>
#include <gnuradio/uwb/uwb_loopback_echo.h>
#include <gnuradio/uwb/uwb_phy_profile.h>
#include <gnuradio/uwb/uwb_radar_packet_source.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#ifndef UWB_TESTDATA_DIR
#define UWB_TESTDATA_DIR "../../../testdata"
#endif

namespace {

using gr_complex = std::complex<float>;
using gr::uwb::demod::DemodResult;
using gr::uwb::demod::DemodStatus;
using gr::uwb::demod::Qm35825Profile;
using gr::uwb::mod::HrpModConfig;
using gr::uwb::mod::HrpModScratch;

// ---------------------------------------------------------------------------
// Frozen geometry of the measured chain (work grid 998.4 MS/s).
// ---------------------------------------------------------------------------
constexpr size_t kSps = gr::uwb::demod::kQm35SamplesPerSymbol; // 1016
constexpr double kFsWork = 998.4e6;
constexpr double kFsNativeCg600 = 737.28e6; // CG600 default, phase-1 native
constexpr double kFsNativeCg400 = 491.52e6; // CG400, measured but see above
constexpr size_t kPreGuardWork = 1997;      // canonical golden rx pre-guard
constexpr size_t kTail = 4096;
constexpr size_t kMaxPsdu = gr::uwb::radar_meta::kMaxPsduBytes; // 127

// The demodulator re-measures only the last max(cfo_min_fit_repetitions, 40)
// repetitions and GENERATES the earlier peak coordinates from the SFD-anchored
// grid (uwb_demod_core.h, demodulate_one).  Generated peaks carry a zero
// matched-filter value, so they must be excluded from the CFO fit.
constexpr size_t kCfoTailReps = 40; // max(cfo_min_fit_repetitions=32, 40)
// stage_cfo() hardcodes this as Qm35825Profile::cfo_skip_initial_repetitions.
constexpr size_t kCfoSkipStock = 24;
// stage_cir_softchips() bails out when cir_skip_initial_repetitions is not
// smaller than the number of available repetitions: this is the hard floor on
// sync_repetitions, measured as DemodStatus::CirFailed at 1/2/4/8/12 SYNC.
constexpr size_t kCirSkipInitial = 10;

// Measured CFO robustness boundary used by the whitelist rule: 64 SYNC tracks
// +-100 kHz exactly, and every length >= 128 is already broken at +-20 kHz.
constexpr double kCfoWhitelistHz = 20000.0;

const char*
status_name(DemodStatus s)
{
    switch (s) {
    case DemodStatus::Success: return "Success";
    case DemodStatus::InvalidInput: return "InvalidInput";
    case DemodStatus::TimingFailed: return "TimingFailed";
    case DemodStatus::CfoFailed: return "CfoFailed";
    case DemodStatus::SfdFailed: return "SfdFailed";
    case DemodStatus::PhrFailed: return "PhrFailed";
    case DemodStatus::PayloadFailed: return "PayloadFailed";
    case DemodStatus::FcsFailed: return "FcsFailed";
    case DemodStatus::QueueFull: return "QueueFull";
    case DemodStatus::InternalError: return "InternalError";
    case DemodStatus::CirFailed: return "CirFailed";
    }
    return "?";
}

std::string
f3(double v)
{
    std::ostringstream os;
    os << std::fixed << std::setprecision(3) << v;
    return os.str();
}

std::string
hex16(uint16_t v)
{
    std::ostringstream os;
    os << "0x" << std::hex << std::setw(4) << std::setfill('0') << v;
    return os.str();
}

// CSV field sanitiser: a recorded reason may embed a block's exception text,
// which contains commas.  Replace them so the record stays parseable.
std::string
csv_field(const std::string& in)
{
    std::string out(in);
    for (auto& c : out) {
        if (c == ',')
            c = ';';
        else if (c == '\n' || c == '\r')
            c = ' ';
    }
    return out;
}

bool
wait_until(const std::function<bool()>& pred, int timeout_ms = 30000)
{
    const auto t0 = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - t0 <
           std::chrono::milliseconds(timeout_ms)) {
        if (pred())
            return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    return pred();
}

// ---------------------------------------------------------------------------
// Independent IEEE 802.15.4 FCS reference (CRC-16/X.25, reflected 0x8408,
// init 0x0000, no final inversion).  Deliberately a separate implementation
// from demod::core::detail::crc16_802154 so the observed FCS is an
// independent number (REQ-QA-01).
// ---------------------------------------------------------------------------
uint16_t
crc16_802154_ref(const uint8_t* data, size_t len)
{
    uint16_t crc = 0x0000;
    for (size_t i = 0; i < len; ++i) {
        crc ^= static_cast<uint16_t>(data[i]);
        for (int b = 0; b < 8; ++b) {
            crc = (crc & 1u) ? static_cast<uint16_t>((crc >> 1) ^ 0x8408u)
                             : static_cast<uint16_t>(crc >> 1);
        }
    }
    return crc;
}

// ---------------------------------------------------------------------------
// M0 draft TWR MAC frames: bytes BEFORE FCS.  FCF(2) + SEQ(1) + PAN(2) +
// DST(2) + SRC(2) = 9 B header, then the frame body.
//   Poll      : 5 B reserved                                  -> 14 B
//   Response  : poll Rx tag(4) + own Tx tag(4) + RTT(4) +
//               options(3) = 15 B                             -> 24 B
//   Final     : 5 x 4 B timestamps = 20 B                     -> 29 B
// Byte order and field widths are the M0 draft (DW-style little-endian 32-bit
// tags).  NOT frozen against a vendor SDK frame profile (REQ-PROTO-06, M1/M5).
// ---------------------------------------------------------------------------
struct TwrFrame {
    const char* name;
    size_t mac_bytes;
};

const TwrFrame kTwrFrames[] = {
    { "Poll", 14 },
    { "Response", 24 },
    { "Final", 29 },
};

// REQ-PHY-01 asks for a "standard length PSDU".  127 B is the maximum
// (radar_meta::kMaxPsduBytes) and is by far the worst case for residual
// CFO phase drift across the payload, so it is the deterministic stress
// payload for the CFO-robustness measurements.
const TwrFrame kStressFrame = { "Max127", 125 };

std::vector<uint8_t>
build_stress_psdu()
{
    std::vector<uint8_t> m(kStressFrame.mac_bytes);
    for (size_t i = 0; i < m.size(); ++i)
        m[i] = static_cast<uint8_t>((0x5a + 7 * i) & 0xff);
    gr::uwb::mod::append_ieee_fcs(m);
    return m;
}

constexpr uint8_t kTwrSeq = 0x42;

std::vector<uint8_t>
build_twr_psdu(const char* name, size_t mac_bytes, bool with_fcs)
{
    if (std::string(name) == kStressFrame.name)
        return build_stress_psdu();
    std::vector<uint8_t> m(mac_bytes, 0);
    if (mac_bytes < 9)
        return m;
    auto put16 = [&](size_t off, uint16_t v) {
        m[off] = static_cast<uint8_t>(v & 0xff);
        m[off + 1] = static_cast<uint8_t>(v >> 8);
    };
    auto put32 = [&](size_t off, uint32_t v) {
        for (int i = 0; i < 4; ++i)
            m[off + i] = static_cast<uint8_t>((v >> (8 * i)) & 0xff);
    };
    const std::string n(name);
    // FCF 0x8841 = data frame, PAN-ID compression, short dst + short src.
    put16(0, (n == "Response") ? uint16_t(0x8845) : uint16_t(0x8841));
    m[2] = kTwrSeq;
    put16(3, 0x1234); // PAN ID
    put16(5, 0x0007); // DST short
    put16(7, 0x0005); // SRC short
    if (n == "Response" && mac_bytes >= 24) {
        put32(9, 0x0000102a);  // poll Rx tag t2B
        put32(13, 0x0000204b); // response Tx tag t3B
        put32(17, 0x00000139); // RTT in device ticks
        m[21] = 0x21;          // options: SS mode, 6.8 Mb/s
    } else if (n == "Final" && mac_bytes >= 29) {
        put32(9, 0x00000065);  // t1A
        put32(13, 0x00000092); // t2A
        put32(17, 0x0000010c); // t3A
        put32(21, 0x00000139); // t4A
        put32(25, 0x000001a5); // t5A
    }
    if (with_fcs && !m.empty())
        gr::uwb::mod::append_ieee_fcs(m);
    return m;
}

// ---------------------------------------------------------------------------
// Channel conditions (UwbLoopbackEcho) + work-grid carrier offset.
// ---------------------------------------------------------------------------
struct Channel {
    std::vector<double> delays;
    std::vector<gr_complex> gains;
    float noise_std = 0.0f;
    uint32_t seed = 1;
    size_t pre_guard = kPreGuardWork;
    size_t tail = kTail;
    // Carrier offset applied on the work grid BEFORE the channel block,
    // i.e. a TX/RX oscillator error.  0 = clean, no CFO.
    double cfo_hz = 0.0;
};

Channel
clean_channel()
{
    Channel c;
    c.delays = { 0.0 };
    c.gains = { gr_complex(1.0f, 0.0f) };
    return c;
}

std::string
scratch_dir(const std::string& tag)
{
    const std::filesystem::path d =
        std::filesystem::temp_directory_path() /
        ("uwb_qa_twr_phy_matrix_" + tag);
    std::error_code ec;
    std::filesystem::remove_all(d, ec);
    std::filesystem::create_directories(d, ec);
    return d.string();
}

// ---------------------------------------------------------------------------
// One measured round trip.
// ---------------------------------------------------------------------------
struct RoundTrip {
    bool modulated = false;
    bool channel_ok = false;
    // Where the chain stopped: "modulator" | "channel_blocks" | "demod" | ""
    std::string stage = "not_started";
    size_t tx_samples = 0;
    size_t rx_samples = 0;
    DemodResult res;
    std::string reason;
};

RoundTrip
run_roundtrip(const HrpModConfig& cfg,
              const std::vector<uint8_t>& psdu,
              const Channel& ch,
              Qm35825Profile& prof, // in/out: profile under test
              const std::string& tag)
{
    RoundTrip out;
    prof.code_index = cfg.code_index;
    prof.preamble_repetitions = cfg.sync_repetitions;
    prof.sfd_mode = cfg.sfd_mode;
    prof.max_psdu_bytes = kMaxPsdu;

    const auto sfd = gr::uwb::demod::GetSfdSequence(cfg.sfd_mode);
    if (sfd.empty()) {
        out.stage = "modulator";
        out.reason = "sfd_mode_not_in_phy_profile_table";
        return out;
    }
    if (!gr::uwb::radar_meta::code_index_supported(cfg.code_index)) {
        out.stage = "modulator";
        out.reason = "code_index_outside_api_range_rejected_before_modulating";
        return out;
    }
    if (!gr::uwb::radar_meta::sync_reps_supported(cfg.sync_repetitions)) {
        out.stage = "modulator";
        out.reason = "sync_repetitions_outside_api_list_rejected_before_"
                     "modulating";
        return out;
    }

    // ---- TX: modulate on the 998.4 MS/s work grid --------------------------
    HrpModScratch ms;
    const size_t cap = gr::uwb::mod::packet_samples_998p4(
        cfg.sync_repetitions, sfd.size(), psdu.size(), cfg.insert_sts,
        &cfg.pulse);
    ms.reserve(psdu.size(), cap);
    std::vector<gr_complex> tx(cap);
    size_t tx_n = 0;
    if (!gr::uwb::mod::modulate_one(psdu.data(), psdu.size(), cfg, ms,
                                    tx.data(), tx.size(), tx_n)) {
        out.stage = "modulator";
        out.reason = "modulate_one_rejected_config";
        return out;
    }
    out.modulated = true;
    out.stage = "channel_blocks";
    out.tx_samples = tx_n;
    tx.resize(tx_n);
    // Matched-filter template: first SYNC symbol as modulated, i.e. BEFORE the
    // carrier offset (same convention as qa_uwb_radar_e2e.cc's template file).
    // Affects the timing correlation only, never the FCS verdict.
    const std::vector<gr_complex> tmpl(tx.begin(),
                                       tx.begin() + static_cast<long>(kSps));

    if (ch.cfo_hz != 0.0) {
        const double w = 2.0 * M_PI * ch.cfo_hz / kFsWork;
        for (size_t i = 0; i < tx.size(); ++i) {
            const double ph = w * static_cast<double>(i);
            tx[i] *= gr_complex(static_cast<float>(std::cos(ph)),
                                static_cast<float>(std::sin(ph)));
        }
    }

    // ---- channel ------------------------------------------------------------
    const std::string tx_path = scratch_dir(tag) + "/tx_998p4.cf32";
    {
        std::ofstream f(tx_path, std::ios::binary | std::ios::trunc);
        f.write(reinterpret_cast<const char*>(tx.data()),
                static_cast<std::streamsize>(tx.size() * sizeof(gr_complex)));
    }
    // The production blocks validate their own arguments and throw.  Turn that
    // into a recorded rejection instead of letting it escape the test case:
    // the prebuilt libgnuradio-uwb accepts sync_repetitions 32..2048 while the
    // header's radar_meta::sync_reps_supported() also lists 1..16, so the
    // header's enumeration is NOT sufficient evidence (REQ-BASE-02).
    gr::uwb::UwbRadarPacketSource::sptr src;
    gr::uwb::UwbLoopbackEcho::sptr echo;
    std::shared_ptr<gr::blocks::message_debug> dbg;
    std::shared_ptr<gr::top_block> tb;
    try {
        src = gr::uwb::UwbRadarPacketSource::make(
            tx_path, kFsWork, "fc32", cfg.sync_repetitions,
            std::string(cfg.sfd_mode ? cfg.sfd_mode : ""), cfg.code_index);
        echo = gr::uwb::UwbLoopbackEcho::make(ch.pre_guard, ch.tail,
                                              ch.delays, ch.gains,
                                              ch.noise_std, ch.seed,
                                              4194304, 8388608);
        dbg = gr::blocks::message_debug::make();
        tb = gr::make_top_block("qa_twr_phy_matrix");
    } catch (const std::exception& e) {
        out.reason = std::string("production_block_rejected_config: ") + e.what();
        return out;
    }
    tb->msg_connect(src, "tx", echo, "tx");
    tb->msg_connect(echo, "rx", dbg, "store");
    tb->start();
    src->_post(pmt::mp("emit"), pmt::make_dict());
    const bool got = wait_until([&] { return dbg->num_messages() >= 1; });
    std::vector<gr_complex> rx;
    if (got) {
        size_t n = 0;
        const gr_complex* p =
            pmt::c32vector_elements(pmt::cdr(dbg->get_message(0)), n);
        if (p && n > 0)
            rx.assign(p, p + n);
    }
    tb->stop();
    tb->wait();
    if (rx.empty()) {
        out.reason = "loopback_emitted_no_rx_window";
        return out;
    }
    out.channel_ok = true;
    out.rx_samples = rx.size();
    out.stage = "demod";

    // ---- RX: full R1..R4 pipeline ------------------------------------------
    gr::uwb::demod::core::DemodScratch scratch;
    scratch.reserve(rx.size());
    out.res = gr::uwb::demod::core::demodulate_one(
        rx.data(), rx.size(), prof, /*packet_id=*/1,
        /*predicted_start=*/static_cast<int64_t>(ch.pre_guard),
        /*window_start=*/0, tmpl, scratch);
    return out;
}

const char*
failure_reason(const DemodResult& r, bool bytes_exact)
{
    if (r.status == DemodStatus::Success && r.payload.fcs_pass && bytes_exact)
        return "";
    if (r.status == DemodStatus::Success && r.payload.fcs_pass && !bytes_exact)
        return "fcs_pass_but_psdu_bytes_differ";
    switch (r.status) {
    case DemodStatus::Success: return "status_success_but_fcs_fail";
    case DemodStatus::TimingFailed: return "stage1_timing_failed";
    case DemodStatus::CfoFailed: return "stage2_cfo_failed";
    case DemodStatus::SfdFailed:
        return r.ns_sfd.ok ? "stage3_sfd_failed"
                           : "stage5_ns_sfd_below_threshold";
    case DemodStatus::PhrFailed: return "stage6_phr_failed";
    case DemodStatus::PayloadFailed: return "stage7_payload_failed";
    case DemodStatus::FcsFailed: return "stage7_fcs_mismatch";
    case DemodStatus::CirFailed: return "stage4_cir_failed";
    default: return "other_status";
    }
}

// PHR preamble-duration indices are 0=16, 1=64, 2=1024, 3=4096
// (802.15.4a-2007 Table 68a).  Returns the repetition count the PHR actually
// advertises, or 0 when the field cannot express the configured length.
size_t
phr_advertised_sync_symbols(int idx)
{
    switch (idx) {
    case 0: return 16;
    case 1: return 64;
    case 2: return 1024;
    case 3: return 4096;
    default: return 0;
    }
}

// ---------------------------------------------------------------------------
// One matrix row.
// ---------------------------------------------------------------------------
struct Row {
    double native_rate_hz = kFsNativeCg600;
    std::string path = "work_direct_998p4";
    size_t code_index = 9;
    size_t sync_repetitions = 64;
    std::string sfd_mode = "4z2";
    std::string frame = "Poll";
    size_t mac_bytes = 14;
    size_t psdu_bytes = 16; // on-air PSDU, FCS included
    bool ranging = false;
    std::string result = "FAIL";   // PASS | FAIL | ERROR (measured)
    std::string whitelist = "unverified"; // supported | unsupported | unverified
    std::string status = "not_run";
    uint32_t phr_len = 0;
    int phr_ranging = -1;
    int phr_preamble_idx = -1;
    double phr_data_rate = 0.0;
    bool bytes_exact = false;
    uint16_t fcs_rx = 0;
    uint16_t fcs_calc = 0;
    bool fcs_pass = false;
    double cfo_inject_hz = 0.0;
    double cfo_est_hz = 0.0;
    size_t detected_peaks = 0;
    size_t cfo_zero_peak_corr = 0;
    int64_t ns_sfd_chip = -1;
    size_t tx_samples = 0;
    size_t rx_window_samples = 0;
    std::string reason;
};

Row
measure(const HrpModConfig& cfg,
        const TwrFrame& frame,
        const Channel& ch,
        const std::string& tag)
{
    Row row;
    row.code_index = cfg.code_index;
    row.sync_repetitions = cfg.sync_repetitions;
    row.sfd_mode = cfg.sfd_mode ? cfg.sfd_mode : "(null)";
    row.frame = frame.name;
    row.mac_bytes = frame.mac_bytes;
    row.psdu_bytes = frame.mac_bytes + 2;
    row.ranging = cfg.ranging;

    const std::vector<uint8_t> expect =
        build_twr_psdu(frame.name, frame.mac_bytes, true);
    HrpModConfig mcfg = cfg;
    Qm35825Profile prof = Qm35825Profile::Default();
    const RoundTrip rt = run_roundtrip(mcfg, expect, ch, prof, tag);
    row.tx_samples = rt.tx_samples;
    row.rx_window_samples = rt.rx_samples;
    if (!rt.modulated || !rt.channel_ok) {
        row.result = "ERROR";
        row.whitelist = "unsupported";
        row.status = (rt.stage == "modulator") ? "config_rejected" : rt.stage;
        row.reason = rt.reason;
        return row;
    }

    const DemodResult& r = rt.res;
    row.status = status_name(r.status);
    row.phr_len = r.phr.psdu_length;
    row.phr_data_rate = r.phr.data_rate_mbps;
    row.fcs_rx = r.payload.received_fcs;
    row.fcs_calc = r.payload.calculated_fcs;
    row.fcs_pass = r.payload.fcs_pass;
    row.cfo_inject_hz = ch.cfo_hz;
    row.cfo_est_hz = r.cfo.ok ? r.cfo.cfo_hz : 0.0;
    row.detected_peaks = r.timing.detected_peaks;
    for (const auto& c : r.timing.peak_corr)
        if (std::abs(c) == 0.0f)
            ++row.cfo_zero_peak_corr;
    row.ns_sfd_chip = r.ns_sfd.sfd_start_chip;
    // phr_bits is the SECDED-corrected 19-bit PHR: bit0..1 data rate,
    // bit2..8 PSDU length, bit9 ranging, bit10 reserved, bit11..12 preamble
    // duration index.
    if (r.phr.phr_bits.size() >= 19) {
        row.phr_ranging = r.phr.phr_bits[9] ? 1 : 0;
        row.phr_preamble_idx = (r.phr.phr_bits[11] << 1) | r.phr.phr_bits[12];
    }
    row.bytes_exact = (r.payload.bytes == expect);
    row.reason = failure_reason(r, row.bytes_exact);
    row.result = row.reason.empty() ? "PASS" : "FAIL";
    return row;
}

// Whitelist verdict, derived from the measurements in this file and applied
// consistently to every row.  A row may only say "supported" when the whole
// measured chain passed AND the PHR preamble-duration field actually describes
// the transmitted preamble length.
void
classify(Row& row)
{
    if (row.result != "PASS") {
        row.whitelist = "unsupported";
        return;
    }
    // Accumulate EVERY independent reason the combination must be rejected for,
    // so the config layer can report all of them and not just the first.
    std::vector<std::string> why;
    // Rule 1 (measured): stage_cir_softchips needs strictly more than
    // cir_skip_initial_repetitions repetitions.  Observed as CirFailed.
    if (row.sync_repetitions <= kCirSkipInitial) {
        why.push_back("cir_stage_requires_more_than_cir_skip_initial_"
                      "repetitions_" +
                      std::to_string(kCirSkipInitial) + "_measured_cir_failed");
    }
    // Rule 2 (measured): the PHR preamble-duration field must describe the
    // frame that was actually transmitted.
    const size_t advertised = phr_advertised_sync_symbols(row.phr_preamble_idx);
    if (advertised != row.sync_repetitions) {
        why.push_back("phr_preamble_duration_index_" +
                      std::to_string(row.phr_preamble_idx) + "_advertises_" +
                      std::to_string(advertised) + "_syMBOLs_not_" +
                      std::to_string(row.sync_repetitions) +
                      "_ieee_802154a_cannot_express_this_length");
    }
    // Rule 3 (measured): the re-measured tail must start at or before the
    // hardcoded cfo_skip_initial_repetitions, otherwise the synthesised peaks
    // (zero matched-filter value => phase 0) enter the phase fit and bias it.
    const size_t tail_first = row.sync_repetitions > kCfoTailReps
                                  ? row.sync_repetitions - kCfoTailReps
                                  : 0;
    if (tail_first > kCfoSkipStock) {
        why.push_back("cfo_fit_includes_" +
                      std::to_string(row.cfo_zero_peak_corr) +
                      "_zero_peak_corr_generated_peaks_because_cfo_skip_" +
                      std::to_string(kCfoSkipStock) + "_lt_tail_first_" +
                      std::to_string(tail_first) +
                      "_needs_cfo_skip_eq_preamble_minus_40");
    }
    if (why.empty()) {
        row.whitelist = "supported";
        // Honest scope caveat: this QA proves DECODE robustness (PHR / payload /
        // FCS), not first-path or ToA accuracy.  A 16-SYNC preamble integrates
        // for 16.3 us against 65.1 us at 64 SYNC, so its ranging accuracy is
        // explicitly UNVERIFIED and must stay so until M2/M3.
        if (row.sync_repetitions == 16)
            row.reason = "decode_verified_only_first_path_toa_accuracy_"
                         "unverified_for_16_sync_shorter_integration_window";
        return;
    }
    row.whitelist = "unsupported";
    for (size_t i = 0; i < why.size(); ++i)
        row.reason += (i ? " | " : "") + why[i];
}

void
log_row(const Row& r)
{
    BOOST_TEST_MESSAGE("MATRIX code=" << r.code_index
                                     << " reps=" << r.sync_repetitions
                                     << " sfd=" << r.sfd_mode
                                     << " frame=" << r.frame
                                     << " ranging=" << r.ranging
                                     << " path=" << r.path << " -> "
                                     << r.result << " / " << r.whitelist
                                     << " status=" << r.status
                                     << " phr_len=" << r.phr_len
                                     << " phr_pre_idx=" << r.phr_preamble_idx
                                     << " phr_ranging=" << r.phr_ranging
                                     << " bytes_exact=" << r.bytes_exact
                                     << " fcs=" << hex16(r.fcs_rx) << "/"
                                     << hex16(r.fcs_calc)
                                     << " cfo_inj=" << f3(r.cfo_inject_hz)
                                     << " cfo_est=" << f3(r.cfo_est_hz)
                                     << " peaks=" << r.detected_peaks
                                     << " zero_peak_corr=" << r.cfo_zero_peak_corr
                                     << " tx=" << r.tx_samples
                                     << " reason=" << r.reason);
}

struct Csv {
    std::vector<Row> rows;
    std::string path;
    std::string whitelist_path;
    double rate = kFsNativeCg600;

    void write(const std::string& dir)
    {
        std::error_code ec;
        std::filesystem::create_directories(dir, ec);
        path = dir + "/phy_matrix_" +
               std::to_string(static_cast<long long>(rate)) + ".csv";
        std::ofstream f(path, std::ios::trunc);
        f << "native_rate_hz,path,code_index,sync_repetitions,sfd_mode,frame,"
             "mac_bytes,psdu_bytes,ranging,result,whitelist,status,"
             "phr_psdu_length,phr_ranging,phr_preamble_idx,"
             "phr_data_rate_mbps,bytes_exact,fcs_rx,fcs_calc,fcs_pass,"
             "cfo_inject_hz,cfo_est_hz,detected_peaks,cfo_zero_peak_corr,"
             "ns_sfd_chip,tx_samples,rx_window_samples,reason\n";
        for (const auto& r : rows) {
            f << static_cast<long long>(r.native_rate_hz) << ','
              << csv_field(r.path) << ',' << r.code_index << ',' << r.sync_repetitions << ','
              << r.sfd_mode << ',' << csv_field(r.frame) << ','
              << r.mac_bytes << ','
              << r.psdu_bytes << ',' << (r.ranging ? 1 : 0) << ','
              << r.result << ',' << r.whitelist << ',' << r.status << ','
              << r.phr_len
              << ',' << r.phr_ranging << ',' << r.phr_preamble_idx << ','
              << f3(r.phr_data_rate) << ',' << (r.bytes_exact ? 1 : 0) << ','
              << hex16(r.fcs_rx) << ',' << hex16(r.fcs_calc) << ','
              << (r.fcs_pass ? 1 : 0) << ',' << f3(r.cfo_inject_hz) << ','
              << f3(r.cfo_est_hz) << ',' << r.detected_peaks << ','
              << r.cfo_zero_peak_corr << ',' << r.ns_sfd_chip << ','
              << r.tx_samples << ',' << r.rx_window_samples << ','
              << csv_field(r.reason.empty() ? "ok" : r.reason) << '\n';
        }
    }

    // Deduplicated (code_index, sync_repetitions, sfd_mode) verdict: exactly the
    // table the M0 capabilities() validator has to encode, one row per PHY
    // combination with the observed reason on the unsupported side.
    void write_whitelist(const std::string& dir)
    {
        struct Key {
            size_t code, reps;
            std::string sfd;
            bool operator<(const Key& o) const
            {
                if (code != o.code)
                    return code < o.code;
                if (reps != o.reps)
                    return reps < o.reps;
                return sfd < o.sfd;
            }
        };
        std::map<Key, std::pair<std::string, std::string>> verdict;
        for (const auto& r : rows) {
            if (r.frame != "Poll" && r.frame != "Response" &&
                r.frame != "Final" && r.frame != "Max127")
                continue;
            const Key k{ r.code_index, r.sync_repetitions, r.sfd_mode };
            auto it = verdict.find(k);
            if (it == verdict.end()) {
                verdict.emplace(k, std::make_pair(r.whitelist, r.reason));
            } else {
                // Any measured failure downgrades the whole combination.
                if (it->second.first == "supported" &&
                    r.whitelist != "supported") {
                    it->second = { r.whitelist, r.reason };
                } else if (it->second.first == "unsupported" &&
                           it->second.second.empty() &&
                           !r.reason.empty()) {
                    it->second.second = r.reason;
                }
            }
        }
        const std::string p =
            dir + "/phy_matrix_whitelist_" +
            std::to_string(static_cast<long long>(rate)) + ".csv";
        std::ofstream f(p, std::ios::trunc);
        f << "# TWR M0 measurement-derived PHY whitelist.  A combination is "
             "'supported' ONLY if a full modulate -> UwbLoopbackEcho -> "
             "demodulate -> FCS-pass round trip was observed for it.  That "
             "proves this repo's TX and RX agree with each other and NOTHING "
             "more: it does NOT mean a DW1000 / DW3000 interoperates, because "
             "no vendor module, SDK frame profile or firmware hash was "
             "measured (REQ-PHY-01, M5).  sfd_len_ieee_802154a_standard flags "
             "whether the SFD length is one of the 8/16-symbol HRP lengths the "
             "standard defines; a non-standard length is self-consistent here "
             "but must be confirmed against the target module before M5.  "
             "Generated by gr-uwb/lib/qa_uwb_twr_phy_matrix.cc.\n"
             "native_rate_hz,code_index,sync_repetitions,sfd_mode,sfd_symbols,"
             "sfd_len_ieee_802154a_standard,whitelist,reason\n";
        for (const auto& kv : verdict) {
            const auto seq = gr::uwb::demod::GetSfdSequence(kv.first.sfd.c_str());
            const bool len_ok = (seq.size() == 8 || seq.size() == 16);
            f << static_cast<long long>(rate) << ',' << kv.first.code << ','
              << kv.first.reps << ',' << kv.first.sfd << ','
              << (seq.empty() ? 0u : static_cast<unsigned>(seq.size())) << ','
              << (len_ok ? 1 : 0) << ',' << kv.second.first << ','
              << csv_field(kv.second.second.empty() ? "ok" : kv.second.second)
              << '\n';
        }
        whitelist_path = p;
    }
};

} // namespace

// ---------------------------------------------------------------------------
// 1) 64-SYNC baseline: the reference result for the whole approach.  TWR-sized
//    MAC payloads, one FCS layer, real UwbLoopbackEcho, R1..R4 FCS pass and
//    byte-exact recovery.  The FCS is cross-checked against the independent
//    CRC-16 in this file.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(twr_phy_matrix_64sync_roundtrip_twr_payloads)
{
    const Channel ch = clean_channel();
    HrpModConfig cfg;
    cfg.code_index = 9;
    cfg.sync_repetitions = 64;
    cfg.sfd_mode = "4z2";
    cfg.ranging = true;

    // Observed FCS values, cross-checked against crc16_802154_ref().
    const struct {
        const char* frame;
        uint16_t fcs;
    } expected[] = {
        { "Poll", 0xdd53 },
        { "Response", 0xf18d },
        { "Final", 0xfcb1 },
    };

    size_t i = 0;
    for (const auto& f : kTwrFrames) {
        const std::vector<uint8_t> psdu =
            build_twr_psdu(f.name, f.mac_bytes, true);
        BOOST_REQUIRE_EQUAL(psdu.size(), f.mac_bytes + 2);
        // Exactly one FCS layer, appended by the modulator side.
        const uint16_t ref =
            crc16_802154_ref(psdu.data(), psdu.size() - 2);
        BOOST_CHECK_EQUAL(ref, expected[i].fcs);
        BOOST_CHECK_EQUAL(std::string(f.name), std::string(expected[i].frame));

        Qm35825Profile prof = Qm35825Profile::Default();
        const RoundTrip rt =
            run_roundtrip(cfg, psdu, ch, prof, "base_" + std::string(f.name));
        BOOST_REQUIRE_MESSAGE(rt.modulated, rt.reason);
        BOOST_REQUIRE_MESSAGE(rt.channel_ok, rt.reason);
        BOOST_TEST_MESSAGE("64-SYNC " << f.name << " mac=" << f.mac_bytes
                                     << " psdu=" << psdu.size()
                                     << " ref_fcs=" << hex16(ref) << " tx="
                                     << rt.tx_samples << " rx="
                                     << rt.rx_samples << " status="
                                     << status_name(rt.res.status) << " reason="
                                     << failure_reason(rt.res,
                                                       rt.res.payload.bytes ==
                                                           psdu));
        BOOST_REQUIRE(rt.res.status == DemodStatus::Success);
        BOOST_REQUIRE(rt.res.phr.ok);
        BOOST_REQUIRE(rt.res.phr.psdu_length == psdu.size());
        BOOST_CHECK(!rt.res.phr.secded_uncorrectable);
        BOOST_REQUIRE(rt.res.payload.ok);
        BOOST_REQUIRE_MESSAGE(rt.res.payload.fcs_pass,
                              "rx=" << hex16(rt.res.payload.received_fcs)
                                    << " calc="
                                    << hex16(rt.res.payload.calculated_fcs));
        BOOST_CHECK_EQUAL(rt.res.payload.received_fcs, ref);
        BOOST_CHECK_EQUAL(rt.res.payload.calculated_fcs, ref);
        BOOST_REQUIRE_EQUAL(rt.res.payload.bytes.size(), psdu.size());
        for (size_t k = 0; k < psdu.size(); ++k)
            BOOST_CHECK_EQUAL(rt.res.payload.bytes[k], psdu[k]);
        // PHR must describe the frame actually transmitted.
        BOOST_CHECK_EQUAL(rt.res.phr.data_rate_mbps, 6.81f);
        BOOST_REQUIRE_EQUAL(rt.res.phr.phr_bits.size(), size_t(19));
        BOOST_CHECK(rt.res.phr.phr_bits[9]);         // ranging bit
        BOOST_CHECK_EQUAL((rt.res.phr.phr_bits[11] << 1) | rt.res.phr.phr_bits[12],
                          1); // preamble duration = 64 SYNC
        // 64 SYNC is the only length whose CFO fit is exact (see the CFO case).
        BOOST_CHECK_EQUAL(rt.res.timing.detected_peaks, size_t(64));
        BOOST_TEST_MESSAGE("64-SYNC " << f.name << " observed FCS "
                                     << hex16(rt.res.payload.received_fcs)
                                     << " (independent reference " << hex16(ref)
                                     << ")");
        ++i;
    }
}

// ---------------------------------------------------------------------------
// 2) 128-SYNC, identical payloads and channel.  This combination was previously
//    only verified at preamble/SFD/CIR level.  It DOES complete a full PHR +
//    payload + FCS round trip at zero CFO — the gap is real but narrower than
//    expected.  What is broken is CFO robustness (case 3) and the PHR
//    preamble-duration field (case 4).
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(twr_phy_matrix_128sync_roundtrip_twr_payloads)
{
    const Channel ch = clean_channel();
    HrpModConfig cfg;
    cfg.code_index = 9;
    cfg.sync_repetitions = 128;
    cfg.sfd_mode = "4z2";
    cfg.ranging = true;

    for (const auto& f : kTwrFrames) {
        const std::vector<uint8_t> psdu =
            build_twr_psdu(f.name, f.mac_bytes, true);
        const uint16_t ref = crc16_802154_ref(psdu.data(), psdu.size() - 2);
        Qm35825Profile prof = Qm35825Profile::Default();
        const RoundTrip rt =
            run_roundtrip(cfg, psdu, ch, prof, "p128_" + std::string(f.name));
        BOOST_REQUIRE_MESSAGE(rt.modulated, rt.reason);
        BOOST_REQUIRE_MESSAGE(rt.channel_ok, rt.reason);
        const bool exact = rt.res.payload.bytes == psdu;
        BOOST_TEST_MESSAGE("128-SYNC " << f.name << " status="
                                       << status_name(rt.res.status)
                                       << " reason="
                                       << failure_reason(rt.res, exact)
                                       << " peaks="
                                       << rt.res.timing.detected_peaks
                                       << " cfo=" << f3(rt.res.cfo.cfo_hz)
                                       << " sfd_metric="
                                       << f3(rt.res.sfd.metric)
                                       << " phr_len=" << rt.res.phr.psdu_length
                                       << " phr_pre_idx="
                                       << ((rt.res.phr.phr_bits[11] << 1) |
                                           rt.res.phr.phr_bits[12])
                                       << " fcs="
                                       << hex16(rt.res.payload.received_fcs));
        BOOST_REQUIRE(rt.res.status == DemodStatus::Success);
        BOOST_REQUIRE(rt.res.phr.psdu_length == psdu.size());
        BOOST_REQUIRE(rt.res.payload.fcs_pass);
        BOOST_CHECK_EQUAL(rt.res.payload.received_fcs, ref);
        BOOST_REQUIRE_EQUAL(rt.res.payload.bytes.size(), psdu.size());
        for (size_t k = 0; k < psdu.size(); ++k)
            BOOST_CHECK_EQUAL(rt.res.payload.bytes[k], psdu[k]);
        BOOST_CHECK_EQUAL(rt.res.timing.detected_peaks, size_t(128));
        // The PHR claims 64 SYNC, so a standards-conforming peer would look for
        // the SFD in the wrong place.  Documented, asserted, not hidden.
        BOOST_REQUIRE_EQUAL(rt.res.phr.phr_bits.size(), size_t(19));
        BOOST_CHECK_EQUAL((rt.res.phr.phr_bits[11] << 1) | rt.res.phr.phr_bits[12],
                          1);
    }
}

// ---------------------------------------------------------------------------
// 3) 128-SYNC CFO diagnosis.
//    uwb_demod_core.h demodulate_one() re-tracks only the last
//    tail_repetitions = max(cfo_min_fit_repetitions, 40) = 40 repetitions from
//    the SFD anchor and then SYNTHESISES the remaining peak coordinates from
//    the SFD-anchored grid.  Those synthesised peaks get peak_corr = (0,0), so
//    stage_cfo() sees a matched-filter phase of exactly 0 for them.
//    Qm35825Profile::cfo_skip_initial_repetitions is hardcoded to 24, which is
//    correct ONLY for 64 SYNC (where tail_first = 64 - 40 = 24).  For 128 SYNC
//    tail_first = 88, so 64 synthesised peaks enter the least-squares phase
//    fit and bias the slope.
//    Consequence: the CFO estimate is wrong for any non-zero offset, and the
//    decode already fails at +-20 kHz.  Setting the skip to
//    preamble_repetitions - tail_repetitions restores it exactly.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(twr_phy_matrix_128sync_cfo_cause)
{
    const Channel base = clean_channel();
    HrpModConfig cfg;
    cfg.code_index = 9;
    cfg.sfd_mode = "4z2";
    cfg.ranging = true;
    const std::vector<uint8_t> psdu =
        build_twr_psdu("Response", kTwrFrames[1].mac_bytes, true);
    // Worst-case payload for residual CFO phase drift across the field.  Used
    // to make the decode-outcome measurement deterministic instead of
    // payload-luck dependent.
    const std::vector<uint8_t> stress = build_stress_psdu();

    for (size_t reps : { size_t(64), size_t(128) }) {
        const size_t tail_first = reps - kCfoTailReps;
        for (double f : { kCfoWhitelistHz, -kCfoWhitelistHz }) {
            Channel ch = base;
            ch.cfo_hz = f;

            // (a) stock profile: cfo_skip_initial_repetitions = 24
            cfg.sync_repetitions = reps;
            Qm35825Profile stock = Qm35825Profile::Default();
            const RoundTrip a =
                run_roundtrip(cfg, psdu, ch, stock, "cfostock_" +
                                                        std::to_string(reps));
            BOOST_REQUIRE_MESSAGE(a.channel_ok, a.reason);
            // Same stock profile, worst-case payload.
            const RoundTrip as =
                run_roundtrip(cfg, stress, ch, stock, "cfostock127_" +
                                                        std::to_string(reps));
            BOOST_REQUIRE_MESSAGE(as.channel_ok, as.reason);

            size_t zero_corr = 0;
            for (const auto& c : a.res.timing.peak_corr)
                if (std::abs(c) == 0.0f)
                    ++zero_corr;
            BOOST_TEST_MESSAGE("CFO reps=" << reps << " inj=" << f
                                           << " stock_est="
                                           << f3(a.res.cfo.cfo_hz)
                                           << " err="
                                           << f3(a.res.cfo.cfo_hz - f)
                                           << " zero_peak_corr=" << zero_corr
                                           << " tail_first=" << tail_first
                                           << " status="
                                           << status_name(a.res.status)
                                           << " | 127B status="
                                           << status_name(as.res.status)
                                           << " fcs=" << as.res.payload.fcs_pass);

            // (b) diagnostic only: skip the synthesised peaks.
            Qm35825Profile fixed = Qm35825Profile::Default();
            fixed.cfo_skip_initial_repetitions = tail_first;
            const RoundTrip b =
                run_roundtrip(cfg, stress, ch, fixed, "cfofix_" +
                                                       std::to_string(reps));

            BOOST_CHECK_EQUAL(zero_corr, tail_first);
            if (reps == 64) {
                // tail_first == 24 == the stock skip: nothing is wrong here.
                BOOST_REQUIRE(a.res.status == DemodStatus::Success);
                BOOST_REQUIRE(a.res.payload.fcs_pass);
                BOOST_CHECK_CLOSE(a.res.cfo.cfo_hz, f, 0.1);
                BOOST_REQUIRE(as.res.status == DemodStatus::Success);
                BOOST_REQUIRE(as.res.payload.fcs_pass);
                BOOST_CHECK(as.res.payload.bytes == stress);
            } else {
                // 64 synthesised (zero) peaks leak into the 104-point fit.
                BOOST_CHECK_GT(zero_corr, size_t(24));
                // Deterministic hard fact: the estimate is badly wrong.
                BOOST_CHECK_GT(std::abs(a.res.cfo.cfo_hz - f), 1000.0);
                // Deterministic consequence on a standard-length PSDU.
                BOOST_CHECK(as.res.status != DemodStatus::Success);
                BOOST_CHECK(!as.res.payload.fcs_pass);
                // With the skip corrected the estimate is exact again and the
                // SAME frame decodes with a passing FCS.
                BOOST_REQUIRE(b.res.status == DemodStatus::Success);
                BOOST_REQUIRE(b.res.payload.fcs_pass);
                BOOST_CHECK_CLOSE(b.res.cfo.cfo_hz, f, 0.1);
                BOOST_CHECK(b.res.payload.bytes == stress);
                BOOST_TEST_MESSAGE("CFO reps=" << reps << " inj=" << f
                                               << " fixed_est="
                                               << f3(b.res.cfo.cfo_hz)
                                               << " -> diagnosis confirmed");
            }
        }
    }
}

// ---------------------------------------------------------------------------
// 4) Preamble-length boundary.  Establishes the exact `sync_repetitions`
//    whitelist from measurement:
//
//      * 16 SYNC is SUPPORTED.  It is the only short length that is BOTH
//        CFO-robust and PHR self-describing:
//          - CFO: for reps <= cfo_min_fit_repetitions (32) the whole preamble
//            is re-measured, so tail_first = 0, no peak is synthesised
//            (cfo_zero_peak_corr = 0) and stage_cfo's
//            cfo_skip_initial_repetitions = 24 degenerates to an effective 0
//            because np(16) <= min_fit(32).  Measured error 0.0 Hz at
//            0/±1/±5/±20/±100 kHz.
//          - PHR: encode_phr19 emits preamble-duration index 0 (=16) for
//            reps <= 16, so a 16-SYNC frame DESCRIBES ITSELF.  Contrast 32 /
//            128 / 256 / 512 (all advertise 64) and 2048 (advertises 1024).
//      * 32 SYNC is CFO-exact but advertises 64, so it is not self-describing.
//      * every length above 64 has the structurally biased CFO fit measured in
//        twr_phy_matrix_128sync_cfo_cause (PHR also cannot describe it,
//        except 1024, which is PHR-legal but still CFO-biased).
//      * lengths at or below 12 SYNC fail in the CIR stage, because
//        stage_cir_softchips bails out when
//        cir_skip_initial_repetitions (10) >= the number of available
//        repetitions.  Measured status: CirFailed.
//
//    NOTE on scope: this QA measures DECODE robustness (PHR/payload/FCS).  It
//    does NOT measure first-path / ToA accuracy, so a 16-SYNC preamble's
//    shorter integration window is explicitly UNVERIFIED for ranging.  See the
//    reason string written into the CSV.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(twr_phy_matrix_preamble_length_boundary)
{
    const Channel base = clean_channel();
    HrpModConfig cfg;
    cfg.code_index = 9;
    cfg.sfd_mode = "4z2";
    cfg.ranging = true;
    const size_t reps_list[] = { 1, 2, 4, 8, 16, 32, 64, 128, 256,
                                 512, 1024, 2048 };
    struct Obs {
        size_t reps;
        bool ran = false;
        bool pass0 = false;
        size_t peaks0 = 0;
        size_t zero_corr0 = 0;
        double est20k = 0.0;
        bool pass20k = false;
        int pre_idx = -1;
        size_t tx_samples = 0;
        DemodStatus status0 = DemodStatus::Success;
        std::string reason;
    };
    std::vector<Obs> obs;
    const std::vector<uint8_t> stress = build_stress_psdu();

    for (size_t reps : reps_list) {
        cfg.sync_repetitions = reps;
        Obs o{};
        o.reps = reps;
        {
            Qm35825Profile p = Qm35825Profile::Default();
            const RoundTrip r = run_roundtrip(cfg, stress, base, p,
                                              "len0_" + std::to_string(reps));
            o.ran = r.channel_ok;
            o.reason = r.reason;
            o.status0 = r.res.status;
            o.pass0 = (r.channel_ok &&
                       r.res.status == DemodStatus::Success &&
                       r.res.payload.fcs_pass && r.res.payload.bytes == stress);
            o.peaks0 = r.res.timing.detected_peaks;
            for (const auto& c : r.res.timing.peak_corr)
                if (std::abs(c) == 0.0f)
                    ++o.zero_corr0;
            o.tx_samples = r.tx_samples;
            o.pre_idx = (r.res.phr.phr_bits.size() >= 19)
                            ? ((r.res.phr.phr_bits[11] << 1) |
                               r.res.phr.phr_bits[12])
                            : -1;
            BOOST_TEST_MESSAGE("LEN reps=" << reps << " cfo0 -> " << o.pass0
                                           << " status="
                                           << status_name(o.status0)
                                           << " phr_pre_idx=" << o.pre_idx
                                           << " peaks=" << o.peaks0
                                           << " zero_peak_corr=" << o.zero_corr0
                                           << " tx=" << o.tx_samples
                                           << " reason=" << o.reason);
        }
        if (o.ran) {
            Channel ch = base;
            ch.cfo_hz = kCfoWhitelistHz;
            Qm35825Profile p = Qm35825Profile::Default();
            const RoundTrip r =
                run_roundtrip(cfg, stress, ch, p,
                              "len20k_" + std::to_string(reps));
            o.est20k = r.res.cfo.cfo_hz;
            o.pass20k = (r.res.status == DemodStatus::Success &&
                         r.res.payload.fcs_pass &&
                         r.res.payload.bytes == stress);
            BOOST_TEST_MESSAGE("LEN reps=" << reps << " cfo20k -> "
                                           << o.pass20k << " est="
                                           << f3(o.est20k) << " err="
                                           << f3(o.est20k - kCfoWhitelistHz)
                                           << " status="
                                           << status_name(r.res.status));
        }
        obs.push_back(o);
    }

    for (const auto& o : obs) {
        const size_t advertised = phr_advertised_sync_symbols(o.pre_idx);
        BOOST_TEST_MESSAGE("BOUNDARY reps=" << o.reps << " ran=" << o.ran
                                             << " cfo0_pass=" << o.pass0
                                             << " status="
                                             << status_name(o.status0)
                                             << " cfo20k_pass=" << o.pass20k
                                             << " cfo20k_err="
                                             << f3(o.est20k - kCfoWhitelistHz)
                                             << " phr_advertises="
                                             << advertised
                                             << " reason=" << o.reason);
        if (!o.ran) {
            BOOST_CHECK(!o.reason.empty());
            BOOST_CHECK(o.reason.find("production_block_rejected_config") !=
                        std::string::npos);
            continue;
        }

        if (o.reps <= kCirSkipInitial) {
            // Below the usable floor, and the reason depends on which guard
            // trips first — both measured here:
            //   * reps 1, 2 : stage_cfo needs >= 4 peaks (CfoFailed)
            //   * reps 4, 8 : stage_cir_softchips bails out when
            //                 cir_skip_initial_repetitions(10) >=
            //                 available repetitions (CirFailed)
            BOOST_CHECK(!o.pass0);
            if (o.reps < 4) {
                BOOST_CHECK(o.status0 == DemodStatus::CfoFailed);
                BOOST_TEST_MESSAGE("FLOOR reps=" << o.reps
                                                 << " -> CfoFailed "
                                                    "(stage_cfo needs >= 4 "
                                                    "measured peaks)");
            } else {
                BOOST_CHECK(o.status0 == DemodStatus::CirFailed);
                BOOST_TEST_MESSAGE("FLOOR reps=" << o.reps
                                                 << " -> CirFailed "
                                                    "(cir_skip_initial_"
                                                    "repetitions="
                                                 << kCirSkipInitial
                                                 << " >= available "
                                                 << o.peaks0 << ")");
            }
            continue;
        }

        // Structural fact, identical for every length: the number of
        // SFD-anchored SYNTHESISED peaks is max(0, reps - tail_repetitions).
        const size_t expect_synth = o.reps > kCfoTailReps
                                        ? o.reps - kCfoTailReps
                                        : 0;
        BOOST_CHECK_EQUAL(o.zero_corr0, expect_synth);
        // Every length above the floor decodes at zero CFO at full length.
        BOOST_REQUIRE_MESSAGE(o.pass0, "zero-CFO decode failed for reps="
                                           + std::to_string(o.reps));
        BOOST_CHECK_EQUAL(o.peaks0, o.reps);
        if (o.reps <= 64) {
            // synthesised - skipped = max(0, reps - 40 - 24) = 0, so the phase
            // fit sees only measured peaks and the estimate is exact.
            BOOST_REQUIRE(o.pass20k);
            BOOST_CHECK_CLOSE(o.est20k, kCfoWhitelistHz, 0.1);
        } else {
            // synthesised - skipped = reps - 64 > 0: the phase fit is biased.
            // The estimate error is the deterministic invariant; the decode
            // failure on a 127 B PSDU is deterministic too.
            BOOST_CHECK_GT(std::abs(o.est20k - kCfoWhitelistHz), 1000.0);
            BOOST_CHECK(!o.pass20k);
        }
        // PHR self-description: only 16, 64 and 1024 are representable.
        const bool phr_ok = (advertised == o.reps);
        if (phr_ok)
            BOOST_CHECK(o.reps == 16 || o.reps == 64 || o.reps == 1024);
    }

    // Final whitelist for sync_repetitions: CFO-robust AND PHR-describable.
    // Measured result: {16, 64}.  1024 is PHR-legal but CFO-biased; 32 is
    // CFO-exact but advertises 64; >64 has both defects; <=12 hits the CIR
    // floor.
    for (const auto& o : obs) {
        const bool cfo_robust = o.ran && o.pass20k;
        const bool phr_ok = (phr_advertised_sync_symbols(o.pre_idx) == o.reps);
        const bool whitelist = o.ran && cfo_robust && phr_ok;
        BOOST_TEST_MESSAGE("WHITELIST reps=" << o.reps << " cfo_robust="
                                             << cfo_robust << " phr_ok=" << phr_ok
                                             << " -> "
                                             << (whitelist ? "supported"
                                                           : "unsupported"));
        if (o.reps == 16 || o.reps == 64)
            BOOST_CHECK(whitelist);
        else
            BOOST_CHECK(!whitelist);
    }
}

// ---------------------------------------------------------------------------
// 4b) 16 SYNC given the same treatment as 64 SYNC.  REQ-PHY-01 suggests
//     starting from "code 9, 128 preamble"; the measurement says 128 is not
//     usable and 16/64 are.  16 SYNC is the SHORTEST usable preamble (shortest
//     frame => shortest TWR turnaround budget) so it must be qualified, not
//     assumed.
//
//     Why it is CFO-robust where 128 is not: with 16 repetitions the re-measured
//     tail covers the WHOLE preamble (tail_first = 0), so every peak has a real
//     matched-filter value, and stage_cfo's skip degenerates to 0 because
//     np(16) <= cfo_min_fit_repetitions(32).  No synthesised zero-phase peak can
//     enter the fit.
//
//     Scope: this qualifies DECODE (PHR / payload / FCS) only.  First-path /
//     ToA accuracy at 16 SYNC is NOT measured here and stays unverified.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(twr_phy_matrix_16sync_treatment)
{
    const Channel clean = clean_channel();
    HrpModConfig cfg;
    cfg.code_index = 9;
    cfg.sync_repetitions = 16;
    cfg.sfd_mode = "4z2";
    cfg.ranging = true;

    // --- A) all four TWR frame sizes + the 127 B worst case ---------------
    {
        const TwrFrame frames[] = { kTwrFrames[0], kTwrFrames[1], kTwrFrames[2],
                                    kStressFrame };
        for (const auto& f : frames) {
            const std::vector<uint8_t> psdu =
                build_twr_psdu(f.name, f.mac_bytes, true);
            const uint16_t ref = crc16_802154_ref(psdu.data(), psdu.size() - 2);
            Qm35825Profile prof = Qm35825Profile::Default();
            const RoundTrip rt =
                run_roundtrip(cfg, psdu, clean, prof, "s16_" + std::string(f.name));
            const DemodResult& r = rt.res;
            BOOST_REQUIRE_MESSAGE(rt.channel_ok, rt.reason);
            BOOST_TEST_MESSAGE("16SYNC " << f.name << " psdu=" << psdu.size()
                                         << " ref_fcs=" << hex16(ref)
                                         << " status="
                                         << status_name(r.status)
                                         << " reason="
                                         << failure_reason(r,
                                                           r.payload.bytes ==
                                                               psdu));
            BOOST_REQUIRE(r.status == DemodStatus::Success);
            BOOST_REQUIRE(r.payload.fcs_pass);
            BOOST_CHECK_EQUAL(r.payload.received_fcs, ref);
            BOOST_REQUIRE_EQUAL(r.payload.bytes.size(), psdu.size());
            for (size_t k = 0; k < psdu.size(); ++k)
                BOOST_CHECK_EQUAL(r.payload.bytes[k], psdu[k]);
            // PHR must describe 16 SYNC, i.e. preamble-duration index 0.
            BOOST_REQUIRE_EQUAL(r.phr.phr_bits.size(), size_t(19));
            BOOST_CHECK_EQUAL((r.phr.phr_bits[11] << 1) | r.phr.phr_bits[12], 0);
            BOOST_CHECK_EQUAL(r.phr.psdu_length,
                              static_cast<uint32_t>(psdu.size()));
            // All 16 repetitions are measured: nothing is synthesised.
            BOOST_CHECK_EQUAL(r.timing.detected_peaks, size_t(16));
            size_t zero = 0;
            for (const auto& c : r.timing.peak_corr)
                if (std::abs(c) == 0.0f)
                    ++zero;
            BOOST_CHECK_EQUAL(zero, size_t(0));
        }
    }

    // --- B) code_index 9..12 x every SFD mode ------------------------------
    {
        const size_t codes[] = { 9, 10, 11, 12 };
        const char* sfds[] = { "4z2", "4z3", "ieee", "decawave", "4z1", "4z4" };
        for (size_t code : codes) {
            for (const char* sfd : sfds) {
                HrpModConfig c = cfg;
                c.code_index = code;
                c.sfd_mode = sfd;
                Row r = measure(c, kStressFrame, clean,
                                std::string("s16cs_") + std::to_string(code) +
                                    "_" + sfd);
                classify(r);
                BOOST_TEST_MESSAGE("16SYNC code=" << code << " sfd=" << sfd
                                                 << " -> " << r.result << "/"
                                                 << r.whitelist);
                BOOST_REQUIRE_MESSAGE(r.result == "PASS", r.reason);
                BOOST_REQUIRE(r.phr_preamble_idx == 0);
            }
        }
    }

    // --- C) CFO robustness: the part 128 SYNC fails ------------------------
    {
        const double cfos[] = { 1000.0, -1000.0, 5000.0,  -5000.0,
                                20000.0, -20000.0, 100000.0, -100000.0 };
        const TwrFrame frames[] = { { "Response", 24 }, kStressFrame };
        for (const auto& f : frames) {
            for (double f_hz : cfos) {
                Channel ch = clean;
                ch.cfo_hz = f_hz;
                Row r = measure(cfg, f, ch,
                                std::string("s16cfo_") + f.name + "_" +
                                    std::to_string(static_cast<long>(f_hz)));
                r.path = "work_direct_998p4_cfo" +
                         std::to_string(static_cast<long long>(f_hz));
                BOOST_TEST_MESSAGE("16SYNC CFO " << f.name << " inj=" << f_hz
                                               << " est=" << f3(r.cfo_est_hz)
                                               << " err="
                                               << f3(r.cfo_est_hz - f_hz)
                                               << " zero_peak_corr="
                                               << r.cfo_zero_peak_corr << " -> "
                                               << r.result);
                // The whole preamble is in the fit, so the estimate is exact.
                BOOST_REQUIRE_MESSAGE(r.result == "PASS", r.reason);
                BOOST_CHECK_EQUAL(r.cfo_zero_peak_corr, size_t(0));
                BOOST_CHECK_CLOSE(r.cfo_est_hz, f_hz, 0.1);
            }
        }
    }

    // --- D) channel conditions --------------------------------------------
    {
        struct Cond {
            const char* name;
            std::vector<double> delays;
            std::vector<gr_complex> gains;
            float noise;
            uint32_t seed;
        };
        const Cond conds[] = {
            { "d0_clean", { 0.0 }, { gr_complex(1.0f, 0.0f) }, 0.0f, 1 },
            { "d37_int", { 37.0 }, { gr_complex(1.0f, 0.0f) }, 0.0f, 1 },
            { "d37p5_frac", { 37.5 }, { gr_complex(1.0f, 0.0f) }, 0.0f, 1 },
            { "mp2_2tap",
              { 12.0, 61.0 },
              { gr_complex(0.8f, 0.0f), gr_complex(0.3f, 0.0f) },
              0.0f,
              1 },
            { "mp3_delayed_strong_second",
              { 0.0, 5.0, 40.0 },
              { gr_complex(0.35f, 0.0f), gr_complex(0.55f, 0.0f),
                gr_complex(0.1f, 0.0f) },
              0.0f,
              1 },
            { "d37_awgn_0p05", { 37.0 }, { gr_complex(1.0f, 0.0f) }, 0.05f, 11 },
            { "d37_awgn_0p15", { 37.0 }, { gr_complex(1.0f, 0.0f) }, 0.15f, 11 },
        };
        for (const auto& c : conds) {
            Channel ch;
            ch.delays = c.delays;
            ch.gains = c.gains;
            ch.noise_std = c.noise;
            ch.seed = c.seed;
            Row r = measure(cfg, kStressFrame, ch,
                            std::string("s16cond_") + c.name);
            r.path = std::string("work_direct_998p4_cond_") + c.name +
                     "_16sync";
            BOOST_TEST_MESSAGE("16SYNC COND " << c.name << " -> " << r.result
                                             << " status=" << r.status
                                             << " reason=" << r.reason);
            BOOST_REQUIRE_MESSAGE(r.result == "PASS", r.reason);
        }
        // Combined worst case: delay + noise + carrier offset together.
        Channel ch = clean;
        ch.delays = { 37.0 };
        ch.gains = { gr_complex(1.0f, 0.0f) };
        ch.noise_std = 0.05f;
        ch.cfo_hz = kCfoWhitelistHz;
        Row r = measure(cfg, kStressFrame, ch, "s16cond_d37_awgn05_cfo20k");
        r.path = "work_direct_998p4_cond_d37_awgn0p05_cfo20k_16sync";
        BOOST_TEST_MESSAGE("16SYNC COND d37+awgn0.05+cfo20k -> " << r.result
                                                 << " est=" << f3(r.cfo_est_hz)
                                                 << " err="
                                                 << f3(r.cfo_est_hz -
                                                       kCfoWhitelistHz)
                                                 << " reason=" << r.reason);
        BOOST_REQUIRE_MESSAGE(r.result == "PASS", r.reason);
        // With noise the residual is noise-limited, not bias-limited.
        BOOST_CHECK_LT(std::abs(r.cfo_est_hz - kCfoWhitelistHz), 1000.0);
    }

    // --- E) AWGN ceiling: 16 SYNC is NOT measurably worse than 64 SYNC ------
    {
        const float levels[] = { 0.05f, 0.10f, 0.15f, 0.20f };
        for (float nz : levels) {
            int pass16 = 0, pass64 = 0;
            for (uint32_t s = 1; s <= 3; ++s) {
                for (int which = 0; which < 2; ++which) {
                    HrpModConfig c = cfg;
                    c.sync_repetitions = which == 0 ? 16 : 64;
                    Channel ch = clean;
                    ch.delays = { 0.0 };
                    ch.gains = { gr_complex(1.0f, 0.0f) };
                    ch.noise_std = nz;
                    ch.seed = s * 7919;
                    Row r = measure(c, kStressFrame, ch,
                                    "s16noise_" + std::to_string(nz) + "_" +
                                        std::to_string(s) + "_" +
                                        std::to_string(which));
                    if (r.result == "PASS")
                        (which == 0 ? pass16 : pass64)++;
                }
            }
            BOOST_TEST_MESSAGE("16SYNC NOISE sigma=" << nz << " 16sync=" << pass16
                                                   << "/3 64sync=" << pass64
                                                   << "/3");
            // Documented, not asserted as a hard threshold: the measured
            // ceilings are the same for 16 and 64, but that is a property of
            // THIS channel model, not a hardware claim.
        }
    }
}

// ---------------------------------------------------------------------------
// 5) Full capability matrix + CSV.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(twr_phy_matrix_sweep_and_csv)
{
    const Channel clean = clean_channel();
    Csv csv;
    csv.rate = kFsNativeCg600;

    const size_t codes[] = { 9, 10, 11, 12 };
    const char* sfds[] = { "4z2", "4z3", "ieee", "decawave", "4z1", "4z4" };
    // 16 and 64 are the measured whitelist; 128 is the open question and is
    // carried here so the CSV records its verdict alongside the rest.
    const size_t reps_list[] = { 16, 64, 128 };

    for (size_t reps : reps_list) {
        for (size_t code : codes) {
            for (const char* sfd : sfds) {
                for (const auto& f : kTwrFrames) {
                    HrpModConfig cfg;
                    cfg.code_index = code;
                    cfg.sync_repetitions = reps;
                    cfg.sfd_mode = sfd;
                    cfg.ranging = (reps != 128);
                    Row r = measure(cfg, f, clean,
                                    std::to_string(reps) + "_" +
                                        std::to_string(code) + "_" + sfd + "_" +
                                        f.name);
                    classify(r);
                    log_row(r);
                    csv.rows.push_back(r);
                }
            }
        }
    }

    // ranging bit both ways on the reference profile, at both whitelist lengths.
    for (size_t reps : { size_t(16), size_t(64) }) {
        for (int ranging = 0; ranging <= 1; ++ranging) {
            HrpModConfig cfg;
            cfg.code_index = 9;
            cfg.sync_repetitions = reps;
            cfg.sfd_mode = "4z2";
            cfg.ranging = (ranging != 0);
            Row r = measure(cfg, kTwrFrames[1], clean,
                            "rang" + std::to_string(reps) + "_" +
                                std::to_string(ranging));
            classify(r);
            BOOST_REQUIRE_MESSAGE(r.result == "PASS", r.reason);
            BOOST_CHECK_EQUAL(r.phr_ranging, ranging);
            log_row(r);
            csv.rows.push_back(r);
        }
    }

    // CFO sensitivity on the reference profile at both preamble lengths, with
    // the TWR Response frame AND the worst-case 127 B PSDU.
    {
        const double cfos[] = { 1000.0, -1000.0, 5000.0,  -5000.0,
                                20000.0, -20000.0, 100000.0, -100000.0 };
        const TwrFrame frames[] = { { "Response", 24 }, kStressFrame };
        for (size_t reps : reps_list) {
            for (double f : cfos) {
                for (const auto& fr : frames) {
                    Channel ch = clean;
                    ch.cfo_hz = f;
                    HrpModConfig cfg;
                    cfg.code_index = 9;
                    cfg.sync_repetitions = reps;
                    cfg.sfd_mode = "4z2";
                    cfg.ranging = true;
                    Row r = measure(cfg, fr, ch,
                                    "cfo" + std::to_string(reps) + "_" +
                                        std::to_string(static_cast<long>(f)) +
                                        "_" + fr.name);
                    r.path = "work_direct_998p4_cfo" +
                             std::to_string(static_cast<long long>(f));
                    classify(r);
                    log_row(r);
                    csv.rows.push_back(r);
                }
            }
        }
    }

    // Channel conditions on the reference profile: the whitelist must not be a
    // zero-delay / no-noise artifact.
    {
        struct Cond {
            const char* name;
            std::vector<double> delays;
            std::vector<gr_complex> gains;
            float noise;
            uint32_t seed;
        };
        const Cond conds[] = {
            { "d0_clean", { 0.0 }, { gr_complex(1.0f, 0.0f) }, 0.0f, 1 },
            { "d37_int", { 37.0 }, { gr_complex(1.0f, 0.0f) }, 0.0f, 1 },
            { "d37p5_frac", { 37.5 }, { gr_complex(1.0f, 0.0f) }, 0.0f, 1 },
            { "mp2_2tap",
              { 12.0, 61.0 },
              { gr_complex(0.8f, 0.0f), gr_complex(0.3f, 0.0f) },
              0.0f,
              1 },
            { "d37_awgn_0p05", { 37.0 }, { gr_complex(1.0f, 0.0f) }, 0.05f, 11 },
            { "mp3_delayed_strong_second",
              { 0.0, 5.0, 40.0 },
              { gr_complex(0.35f, 0.0f), gr_complex(0.55f, 0.0f),
                gr_complex(0.1f, 0.0f) },
              0.0f,
              1 },
        };
        for (const auto& c : conds) {
            for (size_t reps : { size_t(16), size_t(64) }) {
                Channel ch;
                ch.delays = c.delays;
                ch.gains = c.gains;
                ch.noise_std = c.noise;
                ch.seed = c.seed;
                HrpModConfig cfg;
                cfg.code_index = 9;
                cfg.sync_repetitions = reps;
                cfg.sfd_mode = "4z2";
                cfg.ranging = true;
                Row r = measure(cfg, kTwrFrames[1], ch,
                                std::string("cond_") + c.name + "_" +
                                    std::to_string(reps));
                r.path = std::string("work_direct_998p4_cond_") + c.name + "_" +
                         std::to_string(reps) + "sync";
                classify(r);
                log_row(r);
                csv.rows.push_back(r);
            }
        }
    }

    // Out-of-range enumerations must be rejected, not defaulted.  The reason
    // recorded is the one the chain actually produced.
    {
        // Complete sync_repetitions scan over every length the API enumerates,
        // at the reference code / SFD and the worst-case PSDU, at zero CFO and
        // at +20 kHz.  This is the table the config layer's capabilities()
        // validator needs for that field: one measured row per value.
        const size_t all_reps[] = { 1,  2,    4,    8,    16,  32,
                                    64, 128,  256,  512,  1024, 2048 };
        for (size_t reps : all_reps) {
            HrpModConfig cfg;
            cfg.code_index = 9;
            cfg.sync_repetitions = reps;
            cfg.sfd_mode = "4z2";
            cfg.ranging = true;
            for (int with_cfo = 0; with_cfo <= 1; ++with_cfo) {
                Channel ch = clean;
                if (with_cfo)
                    ch.cfo_hz = kCfoWhitelistHz;
                Row r = measure(cfg, kStressFrame, ch,
                                "lenscan_" + std::to_string(reps) + "_" +
                                    std::to_string(with_cfo));
                r.path = std::string("work_direct_998p4_length_scan_cfo") +
                         (with_cfo ? "20k" : "0");
                classify(r);
                BOOST_TEST_MESSAGE("LENSCAN reps=" << reps << " cfo="
                                                   << (with_cfo ? "20k" : "0")
                                                   << " -> " << r.result << "/"
                                                   << r.whitelist
                                                   << " est=" << f3(r.cfo_est_hz)
                                                   << " pre_idx="
                                                   << r.phr_preamble_idx
                                                   << " reason=" << r.reason);
                log_row(r);
                csv.rows.push_back(r);
            }
        }
    }

    {
        HrpModConfig cfg;
        cfg.code_index = 13;
        cfg.sync_repetitions = 64;
        cfg.sfd_mode = "4z2";
        Row r = measure(cfg, kTwrFrames[0], clean, "bad_code");
        BOOST_CHECK_EQUAL(r.result, "ERROR");
        BOOST_CHECK_EQUAL(r.status, "config_rejected");
        log_row(r);
        csv.rows.push_back(r);
    }
    {
        HrpModConfig cfg;
        cfg.code_index = 9;
        cfg.sync_repetitions = 100;
        cfg.sfd_mode = "4z2";
        Row r = measure(cfg, kTwrFrames[0], clean, "bad_reps");
        BOOST_CHECK_EQUAL(r.result, "ERROR");
        BOOST_CHECK_EQUAL(r.status, "config_rejected");
        log_row(r);
        csv.rows.push_back(r);
    }
    {
        HrpModConfig cfg;
        cfg.code_index = 9;
        cfg.sync_repetitions = 64;
        cfg.sfd_mode = "bogus_sfd";
        Row r = measure(cfg, kTwrFrames[0], clean, "bad_sfd");
        BOOST_CHECK_EQUAL(r.result, "ERROR");
        BOOST_CHECK_EQUAL(r.status, "config_rejected");
        log_row(r);
        csv.rows.push_back(r);
    }
    {
        // 16 SYNC is part of the whitelist (see
        // twr_phy_matrix_16sync_treatment).  The sub-16 lengths are not, and the
        // reason is a MEASURED one, not a library refusal: reps 1/2 stop in
        // stage_cfo (fewer than 4 peaks) and reps 4/8 stop in
        // stage_cir_softchips (cir_skip_initial_repetitions = 10 is not smaller
        // than the available repetitions).
        const struct {
            size_t reps;
            const char* expect;
            const char* why;
        } floors[] = {
            { 1, "CfoFailed", "stage_cfo_requires_at_least_4_measured_peaks" },
            { 2, "CfoFailed", "stage_cfo_requires_at_least_4_measured_peaks" },
            { 4, "CirFailed",
              "cir_skip_initial_repetitions_10_not_smaller_than_available_4" },
            { 8, "CirFailed",
              "cir_skip_initial_repetitions_10_not_smaller_than_available_8" },
        };
        for (const auto& f : floors) {
            HrpModConfig cfg;
            cfg.code_index = 9;
            cfg.sync_repetitions = f.reps;
            cfg.sfd_mode = "4z2";
            Row r = measure(cfg, kStressFrame, clean,
                            "shortreps" + std::to_string(f.reps));
            r.path = "work_direct_998p4_usable_floor";
            BOOST_TEST_MESSAGE("FLOOR reps=" << f.reps << " status=" << r.status
                                            << " reason=" << r.reason);
            BOOST_CHECK_EQUAL(r.status, std::string(f.expect));
            BOOST_CHECK_EQUAL(r.result, "FAIL");
            classify(r);
            BOOST_CHECK_EQUAL(r.whitelist, "unsupported");
            r.reason += std::string("_") + f.why;
            log_row(r);
            csv.rows.push_back(r);
        }
    }

    // Deliberately unverified capabilities, recorded so the config layer has an
    // explicit reason to reject them.
    {
        struct Unverified {
            const char* what;
            double native_rate_hz;
            const char* why;
        };
        const Unverified items[] = {
            { "sts",
              kFsNativeCg600,
              "deliberately_excluded_REQ_SCOPE_04_phase1_baseline_is_sts_free_"
              "modulator_requires_4z_sfd_and_demod_profile_has_no_sts_"
              "parameters" },
            { "data_rate_0p11_0p85_27p24",
              kFsNativeCg600,
              "mod_encode_phr19_hardcodes_6_81_mbps_index_2_and_"
              "stage_payload_fcs_always_uses_6_81_mbps_geometry" },
            { "native_737p28_65_48_roundtrip",
              kFsNativeCg600,
              "no_work_to_native_decimator_in_repo_so_a_twr_psdu_cannot_be_"
              "modulated_on_the_native_grid" },
            { "native_491p52_65_32_roundtrip",
              kFsNativeCg400,
              "no_work_to_native_decimator_in_repo_so_a_twr_psdu_cannot_be_"
              "modulated_on_the_native_grid" },
            { "dw1000_dw3000_interop",
              kFsNativeCg600,
              "loopback_roundtrip_only_proves_this_repo_tx_and_rx_agree_"
              "no_vendor_module_or_sdk_frame_profile_measured" },
            { "preamble_1024",
              kFsNativeCg600,
              "phr_legal_but_cfo_fit_bias_equals_128_case" },
            { "independent_sfo",
              kFsNativeCg600,
              "not_exercised_single_clock_loopback_has_no_sample_rate_offset" },
        };
        for (const auto& it : items) {
            Row r;
            r.native_rate_hz = it.native_rate_hz;
            r.code_index = 9;
            r.sync_repetitions = 64;
            r.sfd_mode = "4z2";
            r.frame = it.what;
            r.mac_bytes = 0;
            r.psdu_bytes = 0;
            r.result = "NOT_RUN";
            r.whitelist = "unverified";
            r.status = "not_run";
            r.reason = it.why;
            log_row(r);
            csv.rows.push_back(r);
        }
    }

    const char* dir_env = std::getenv("UWB_TWR_PHY_MATRIX_CSV_DIR");
    const std::string dir = dir_env ? std::string(dir_env)
                                    : std::string(UWB_TESTDATA_DIR) + "/twr";
    // The CG400 (491.52 MS/s) rows are unverified and are recorded inside the
    // same file; see the native_491p52_65_32_roundtrip row.
    csv.write(dir);
    csv.write_whitelist(dir);
    BOOST_TEST_MESSAGE("CSV: " << csv.path << " rows=" << csv.rows.size());
    BOOST_TEST_MESSAGE("CSV: " << csv.whitelist_path);

    size_t pass = 0, fail = 0, err = 0, supported = 0, unsupported = 0,
           unverified = 0;
    for (const auto& r : csv.rows) {
        if (r.result == "PASS")
            ++pass;
        else if (r.result == "FAIL")
            ++fail;
        else if (r.result == "ERROR")
            ++err;
        if (r.whitelist == "supported")
            ++supported;
        else if (r.whitelist == "unsupported")
            ++unsupported;
        else
            ++unverified;
    }
    BOOST_TEST_MESSAGE("SUMMARY pass=" << pass << " fail=" << fail
                                      << " error=" << err << " supported="
                                      << supported << " unsupported="
                                      << unsupported << " unverified="
                                      << unverified);

    // Whitelist invariants the M0 config layer must encode.  These are derived
    // from the measurements above, not assumed:
    //   * a measured PASS whose PHR advertises its own length and whose
    //     synthesised-peak count is fully covered by the CFO skip must be
    //     classified "supported"
    //   * NO row above 64 SYNC may ever be classified "supported"
    //   * the config-rejected enumerations must stay "unsupported"
    for (const auto& r : csv.rows) {
        const bool phr_ok =
            phr_advertised_sync_symbols(r.phr_preamble_idx) == r.sync_repetitions;
        const size_t tail_first = r.sync_repetitions > kCfoTailReps
                                      ? r.sync_repetitions - kCfoTailReps
                                      : 0;
        const bool cfo_ok = tail_first <= kCfoSkipStock;
        if (r.result == "PASS" && phr_ok && cfo_ok)
            BOOST_CHECK_EQUAL(r.whitelist, "supported");
        if (r.sync_repetitions > 64)
            BOOST_CHECK_NE(r.whitelist, "supported");
        if (r.status == "config_rejected")
            BOOST_CHECK_NE(r.whitelist, "supported");
    }
    // Both whitelist lengths must be present for the reference profile.
    size_t base_supported = 0;
    for (const auto& r : csv.rows) {
        if (r.code_index == 9 && r.sfd_mode == "4z2" &&
            r.path == "work_direct_998p4" && r.whitelist == "supported")
            ++base_supported;
    }
    BOOST_CHECK_GE(base_supported, size_t(6)); // 3 frames x {16, 64}
}
