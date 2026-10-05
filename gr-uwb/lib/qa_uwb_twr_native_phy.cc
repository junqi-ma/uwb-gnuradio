/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * M2-A native PHY closed-loop QA (G0 docs/twr/M2-A_G0接口与数字坐标.md §12,
 * acceptance IDs A01/A02/A04/A06/A07/A08/A09).
 *
 * WHAT THIS FILE IS
 * -----------------
 * The independent acceptance QA for the frozen M2-A interface
 * `gr-uwb/include/gnuradio/uwb/uwb_twr_phy.h`.  It drives ONLY the frozen
 * public entry points:
 *
 *     twr::encode()                     -- real frame codec (no FCS)
 *     mod::append_ieee_fcs()            -- the ONE FCS layer
 *     twr::m2a_modulate_to_work()       -- MAC -> work IQ (998.4 MS/s)
 *     twr::m2a_tx_resample()            -- work -> native
 *     twr::m2a_rx_resample()            -- native -> work
 *     twr::m2a_sc16_roundtrip()         -- CF32 -> SC16 -> CF32
 *     twr::m2a_demod_work()             -- work -> PHR/PSDU/FCS
 *     twr::m2a_native_roundtrip()       -- the whole closed loop
 *
 * Every expectation below is HAND-DERIVED or independently recomputed in this
 * file.  In particular:
 *   * the PSDU is built from the codec's own MAC bytes plus an INDEPENDENT
 *     CRC-16 (reflected poly 0x8408, init 0, little-endian) -- never from the
 *     library's FCS output;
 *   * the expected work/native/RX sample counts come from the frozen integer
 *     formula and a hard-coded numeric table (G0 §1.3), not from the helper;
 *   * the SC16 quantiser expectations are literal values for literal inputs.
 * This QA never asserts that two of the project's own functions agree as a
 * substitute for an independent oracle.
 *
 * IT IS EXPECTED TO FAIL TO LINK UNTIL `lib/uwb_twr_phy.cc` LANDS.  The
 * coordinator registers it (see the report); until then a syntax-only compile
 * (`c++ -fsyntax-only`) is the strongest check available.
 *
 * SCOPE / HONESTY
 * ---------------
 * Nothing here is a hardware measurement.  `measurement_valid` must stay
 * false.  No MATLAB was run.  A native round trip is `native_roundtrip_verified`
 * evidence for exactly the measured rate/profile/taps/format and NOTHING more;
 * `allows(NativeRoundtripVerified, Ranging)` stays false.
 */

#include <boost/test/unit_test.hpp>

#include <gnuradio/uwb/uwb_rational_resampler_core.h>
#include <gnuradio/uwb/uwb_twr_frame.h>
#include <gnuradio/uwb/uwb_twr_phy.h>

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdint>
#include <fstream>
#include <limits>
#include <random>
#include <string>
#include <vector>

#ifndef UWB_TESTDATA_DIR
#define UWB_TESTDATA_DIR "../../../testdata"
#endif

namespace twr = gr::uwb::twr;
namespace mod = gr::uwb::mod;
namespace demod = gr::uwb::demod;

using gr_complex = std::complex<float>;

namespace {

// ---------------------------------------------------------------------------
// Tap loading (float32 LE, raw).  The M2-A config carries these; the helper
// never invents a coefficient.  Paths per G0 §3.3.
// ---------------------------------------------------------------------------

std::string find_path(const std::string& rel_to_testdata)
{
    std::vector<std::string> cands;
#ifdef UWB_TESTDATA_DIR
    cands.push_back(std::string(UWB_TESTDATA_DIR) + "/" + rel_to_testdata);
#endif
    const char* prefixes[] = { "", "../", "../../", "../../../", "../../../../" };
    for (const char* p : prefixes)
        cands.push_back(std::string(p) + "testdata/" + rel_to_testdata);
    for (const auto& c : cands) {
        std::ifstream f(c, std::ios::binary);
        if (f)
            return c;
    }
    return cands.empty() ? rel_to_testdata : cands.front();
}

std::vector<float> load_f32(const std::string& path)
{
    std::ifstream f(path, std::ios::binary | std::ios::ate);
    BOOST_REQUIRE_MESSAGE(f, "cannot open " + path);
    const auto bytes = static_cast<size_t>(f.tellg());
    BOOST_REQUIRE_EQUAL(bytes % sizeof(float), 0u);
    f.seekg(0);
    std::vector<float> v(bytes / sizeof(float));
    f.read(reinterpret_cast<char*>(v.data()),
           static_cast<std::streamsize>(bytes));
    BOOST_REQUIRE(f);
    return v;
}

const std::vector<float>& tx_taps_for(twr::M2aNativeRate r)
{
    static const std::vector<float> t48 =
        load_f32(find_path("twr/m2a/taps/tx_48_65.f32"));
    static const std::vector<float> t32 =
        load_f32(find_path("twr/m2a/taps/tx_32_65.f32"));
    return (r == twr::M2aNativeRate::Uc200_737280000) ? t48 : t32;
}

const std::vector<float>& rx_taps_for(twr::M2aNativeRate r)
{
    static const std::vector<float> t65_48 =
        load_f32(find_path("resampler_65_48/taps_quality_minorder.txt"));
    static const std::vector<float> t65_32 =
        load_f32(find_path("resampler_65_32/taps_quality_minorder.txt"));
    return (r == twr::M2aNativeRate::Uc200_737280000) ? t65_48 : t65_32;
}

// ---------------------------------------------------------------------------
// Independent IEEE 802.15.4 FCS (CRC-16/X.25, reflected 0x8408, init 0, no
// final inversion).  A SECOND implementation, deliberately not the library's
// crc16_802154, so the observed FCS is an independent number.
// ---------------------------------------------------------------------------
uint16_t crc16_ref(const uint8_t* data, size_t len)
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
// Frozen integer contracts (G0 §1.3, §3.1).  Implemented HERE, independently
// of the helper, so a helper that drifts is caught.
// ---------------------------------------------------------------------------

// Causal full-convolution length: Lout = ceil(((N-1)*L + T)/M).  N == 0 is
// defined as 0 at the M2-A helper layer (G0 §3.2).
int64_t frozen_len(uint64_t N, uint32_t L, uint32_t M, size_t T)
{
    if (N == 0 || L == 0 || M == 0 || T == 0)
        return 0;
    const int64_t num =
        (static_cast<int64_t>(N) - 1) * static_cast<int64_t>(L) +
        static_cast<int64_t>(T);
    if (num <= 0)
        return 0;
    return (num + static_cast<int64_t>(M) - 1) / static_cast<int64_t>(M);
}

// n_work = 64*1016 + 8*1016 + 21*512*2 + nsym*128 with
// nsym = 8*psdu + 48*ceil(8*psdu/330).  Hard-coded numeric table in G0 §1.3.
size_t expected_work_samples(size_t psdu_bytes)
{
    const size_t bits = psdu_bytes * 8;
    const size_t blocks = (bits + 329) / 330;
    const size_t nsym = bits + 48 * blocks;
    return 64 * demod::kQm35SamplesPerSymbol +
           8 * demod::kQm35SamplesPerSymbol +
           21 * 512 * demod::kQm35SamplesPerChip + nsym * 128;
}

// ---------------------------------------------------------------------------
// Frames and configs
// ---------------------------------------------------------------------------

// A REAL codec frame.  `variant` changes (session, seq, addr, timestamps) so
// two variants of the same type are different bytes: reusing a stale payload
// is then visible.
twr::Frame make_frame(twr::FrameType t, int variant)
{
    twr::Frame f;
    f.version = twr::kFrameVersion;
    f.function_code = t;
    f.session_id = static_cast<uint16_t>(0x1000 + variant);
    f.seq = static_cast<uint16_t>(0x0042 + 7 * variant);
    f.pan_id = 0x1234;
    f.src_addr = static_cast<uint16_t>(0x0005 + variant);
    f.dst_addr = static_cast<uint16_t>(0x0007 + variant);
    f.flags = twr::make_flags(true);
    switch (t) {
    case twr::FrameType::Poll:
        break;
    case twr::FrameType::Response:
        f.set(twr::TimestampField::T2B, 0x1122334455ULL + variant);
        f.set(twr::TimestampField::T3B, 0x2233445566ULL + variant);
        break;
    case twr::FrameType::Final:
        f.set(twr::TimestampField::T1A, 0x00000000A1ULL + variant);
        f.set(twr::TimestampField::T4A, 0x00000000A2ULL + variant);
        f.set(twr::TimestampField::T5A, 0x00000000A3ULL + variant);
        break;
    default:
        break;
    }
    return f;
}

twr::M2aConfig make_cfg(twr::M2aNativeRate rate, twr::M2aIqFormat fmt)
{
    twr::M2aConfig c;
    c.native_rate = rate;
    c.iq_format = fmt;
    c.code_index = 9;
    c.sync_repetitions = 64;
    c.sfd_mode = "ieee";
    c.insert_sts = false;
    c.ranging = true;
    c.peak_amplitude = 0.8f;
    c.tx_taps = tx_taps_for(rate);
    c.rx_taps = rx_taps_for(rate);
    return c;
}

// Modulate an arbitrary, already-complete PSDU (FCS included) with the frozen
// profile.  Used for the wrong-FCS case, where the helper's own FCS append
// would undo the deliberate corruption.
bool modulate_psdu(const std::vector<uint8_t>& psdu, const twr::M2aConfig& cfg,
                   std::vector<gr_complex>& work, std::string& why)
{
    mod::HrpModScratch ms;
    const size_t cap = expected_work_samples(psdu.size());
    ms.reserve(psdu.size(), cap);
    work.assign(cap, gr_complex(0, 0));
    size_t n = 0;
    mod::HrpModConfig mc = cfg.hrp_config();
    if (!mod::modulate_one(psdu.data(), psdu.size(), mc, ms, work.data(),
                           work.size(), n)) {
        why = "modulate_one refused the PSDU";
        work.clear();
        return false;
    }
    work.resize(n);
    why.clear();
    return true;
}

// The waveform the demod stage actually consumes: MAC -> work -> native ->
// work (band-limited).  The raw modulator output is NOT a demod input in the
// M2-A chain, so A09 embeds this, not the raw work waveform.
bool make_bandlimited_work(const std::vector<uint8_t>& mac, const twr::M2aConfig& cfg,
                           std::vector<gr_complex>& out, std::string& why)
{
    twr::M2aStageTrace tr;
    std::vector<gr_complex> w, nat;
    if (!twr::m2a_modulate_to_work(mac.data(), mac.size(), cfg, w, tr, why))
        return false;
    if (!twr::m2a_tx_resample(w.data(), w.size(), cfg, nat, tr, why))
        return false;
    if (!twr::m2a_rx_resample(nat.data(), nat.size(), cfg, out, tr, why))
        return false;
    return true;
}

// ---------------------------------------------------------------------------
// A01 helper: one clean native round trip, checked against independent
// expectations.
// ---------------------------------------------------------------------------
void check_clean_roundtrip(const twr::Frame& f, const twr::FrameProfile& prof,
                           const twr::M2aConfig& cfg, const std::string& label)
{
    std::string err;
    std::vector<uint8_t> mac;
    BOOST_REQUIRE_MESSAGE(twr::encode(f, prof, mac, err),
                          label + ": codec encode refused: " + err);

    const uint16_t ref_fcs = crc16_ref(mac.data(), mac.size());
    std::vector<uint8_t> expect = mac;
    mod::append_ieee_fcs(expect);
    BOOST_REQUIRE_EQUAL(expect.size(), mac.size() + 2);
    BOOST_CHECK_EQUAL(expect[mac.size()], static_cast<uint8_t>(ref_fcs & 0xff));
    BOOST_CHECK_EQUAL(expect[mac.size() + 1], static_cast<uint8_t>(ref_fcs >> 8));

    const size_t exp_work = expected_work_samples(expect.size());
    BOOST_CHECK_EQUAL(twr::m2a_expected_work_samples(expect.size(), cfg),
                      exp_work);

    twr::M2aResult out;
    std::string why;
    const bool ok = twr::m2a_native_roundtrip(f, prof, cfg, out, why);
    // Non-fatal so every unit of the 12-cell matrix is reported, not just the
    // first failure; the field checks below are meaningless when !ok.
    BOOST_CHECK_MESSAGE(ok, label + ": closed loop not ok: status=" +
                                  std::string(twr::m2a_status_to_string(out.status)) +
                                  " why=" + why);
    if (!ok)
        return;

    BOOST_CHECK_MESSAGE(out.status == twr::M2aStatus::Ok,
                        label + ": status=" +
                            std::string(twr::m2a_status_to_string(out.status)));
    BOOST_CHECK_MESSAGE(out.function_code == static_cast<uint8_t>(f.function_code),
                        label + ": function_code mismatch");
    BOOST_CHECK_MESSAGE(out.mac_bytes == mac, label + ": mac_bytes != codec bytes");
    BOOST_CHECK_MESSAGE(out.psdu_bytes == expect, label + ": psdu_bytes != mac+FCS");
    BOOST_CHECK_EQUAL(out.fcs, ref_fcs);
    BOOST_CHECK_EQUAL(out.expected_psdu_length, expect.size());
    BOOST_CHECK_MESSAGE(out.decoded_bytes == expect,
                        label + ": decoded bytes differ from the input PSDU");
    BOOST_CHECK_MESSAGE(out.fcs_pass, label + ": fcs_pass false");
    BOOST_CHECK_MESSAGE(out.bytes_exact, label + ": bytes_exact false");
    BOOST_CHECK_MESSAGE(out.demod_status == demod::DemodStatus::Success,
                        label + ": demod_status not Success");
    BOOST_CHECK_MESSAGE(!out.measurement_valid,
                        label + ": measurement_valid must be false");
    BOOST_CHECK_EQUAL(out.work_tx_samples, exp_work);

    uint32_t l = 0, m = 0;
    BOOST_REQUIRE(twr::m2a_rate_tx_lm(cfg.native_rate, l, m));
    BOOST_CHECK_EQUAL(
        out.native_samples,
        static_cast<size_t>(frozen_len(exp_work, l, m, cfg.tx_taps.size())));
    BOOST_CHECK_EQUAL(
        out.work_rx_samples,
        static_cast<size_t>(
            frozen_len(out.native_samples, 65, l, cfg.rx_taps.size())));

    // CF32 has no quantiser, so nothing may be reported as saturated.
    if (cfg.iq_format == twr::M2aIqFormat::Cf32)
        BOOST_CHECK_EQUAL(out.sc16_saturated, size_t(0));

    BOOST_CHECK_EQUAL(out.stages.size(), size_t(4));
    BOOST_TEST_MESSAGE(label << ": ok work=" << out.work_tx_samples
                            << " native=" << out.native_samples
                            << " rx=" << out.work_rx_samples
                            << " sc16_sat=" << out.sc16_saturated
                            << " detail=" << out.detail);
}

// ---------------------------------------------------------------------------
// A04 core helpers (chunk invariance + short output buffer).  These drive the
// reused resampler core DIRECTLY, so the helper's own chunking cannot mask a
// core defect.
// ---------------------------------------------------------------------------
template <typename Core>
std::vector<gr_complex> core_run_full(Core& core, const std::vector<gr_complex>& x)
{
    const size_t Lout = Core::expected_output_length(x.size(), core.tap_count());
    std::vector<gr_complex> y(Lout + 256, gr_complex(0, 0));
    size_t produced = 0;
    if (!x.empty()) {
        auto r = core.process(x.data(), x.size(), y.data(), y.size());
        produced = r.produced;
        BOOST_REQUIRE_EQUAL(r.consumed, x.size());
    }
    while (true) {
        if (produced >= y.size())
            y.resize(produced + 256);
        const size_t n = core.flush(y.data() + produced, y.size() - produced);
        if (n == 0)
            break;
        produced += n;
    }
    y.resize(produced);
    return y;
}

// Feed `x` as one chunk, but with a SHORT output buffer: process() must return
// fewer outputs and consume fewer inputs per call, then flush the tail.
template <typename Core>
std::vector<gr_complex> core_run_short(Core& core, const std::vector<gr_complex>& x,
                                       size_t out_cap)
{
    std::vector<gr_complex> y;
    std::vector<gr_complex> tmp(out_cap);
    size_t fed = 0;
    size_t guard = 0;
    while (fed < x.size()) {
        auto r = core.process(x.data() + fed, x.size() - fed, tmp.data(), out_cap);
        for (size_t i = 0; i < r.produced; ++i)
            y.push_back(tmp[i]);
        fed += r.consumed;
        if (r.consumed == 0 && r.produced == 0)
            BOOST_FAIL("resampler made no progress with a short output buffer");
        if (++guard > 10000000)
            BOOST_FAIL("resampler short-buffer drain did not terminate");
    }
    std::vector<gr_complex> tail(out_cap);
    while (true) {
        const size_t n = core.flush(tail.data(), tail.size());
        if (n == 0)
            break;
        for (size_t i = 0; i < n; ++i)
            y.push_back(tail[i]);
    }
    return y;
}

float max_abs_diff(const std::vector<gr_complex>& a,
                   const std::vector<gr_complex>& b)
{
    BOOST_REQUIRE_EQUAL(a.size(), b.size());
    float m = 0.f;
    for (size_t i = 0; i < a.size(); ++i)
        m = std::max(m, std::abs(a[i] - b[i]));
    return m;
}

template <typename Core>
void check_core_chunk_invariance(const char* name, uint32_t L, uint32_t M,
                                 const std::vector<float>& taps)
{
    const size_t N = 4096;
    std::vector<gr_complex> x(N);
    for (size_t i = 0; i < N; ++i)
        x[i] = gr_complex(std::sin(0.01f * i), std::cos(0.013f * i));

    Core one(taps);
    auto y_ref = core_run_full(one, x);
    BOOST_CHECK_EQUAL(y_ref.size(),
                      static_cast<size_t>(frozen_len(N, L, M, taps.size())));

    Core shortbuf(taps);
    auto y_short = core_run_short(shortbuf, x, 16);
    BOOST_CHECK_EQUAL(y_short.size(), y_ref.size());
    const float e = max_abs_diff(y_short, y_ref);
    BOOST_TEST_MESSAGE("A04 core[" << name << "] ref=" << y_ref.size()
                                   << " shortbuf max_abs=" << e);
    BOOST_CHECK_SMALL(e, 1e-6f);
}

// Helper statelessness: a call on burst A, then B, then A again must give the
// SAME A both times (no cross-burst filter-history leak), and the length must
// equal the frozen formula.
void check_helper_stateless_bursts(twr::M2aNativeRate rate, const char* name)
{
    const twr::M2aConfig cfg = make_cfg(rate, twr::M2aIqFormat::Cf32);
    uint32_t l = 0, m = 0;
    BOOST_REQUIRE(twr::m2a_rate_tx_lm(rate, l, m));

    const size_t N = 4096;
    std::vector<gr_complex> A(N), B(N);
    std::mt19937 rng(0x5EED);
    std::uniform_real_distribution<float> d(-0.5f, 0.5f);
    for (size_t i = 0; i < N; ++i) {
        A[i] = gr_complex(d(rng), d(rng));
        B[i] = gr_complex(d(rng), d(rng));
    }

    std::vector<gr_complex> a1, a2, b1, a_rx, a_rx2;
    twr::M2aStageTrace tr;
    std::string why;
    BOOST_REQUIRE_MESSAGE(twr::m2a_tx_resample(A.data(), A.size(), cfg, a1, tr, why),
                          std::string(name) + " tx A: " + why);
    BOOST_REQUIRE_MESSAGE(twr::m2a_tx_resample(B.data(), B.size(), cfg, b1, tr, why),
                          std::string(name) + " tx B: " + why);
    BOOST_REQUIRE_MESSAGE(twr::m2a_tx_resample(A.data(), A.size(), cfg, a2, tr, why),
                          std::string(name) + " tx A again: " + why);

    BOOST_CHECK_EQUAL(a1.size(),
                      static_cast<size_t>(frozen_len(N, l, m, cfg.tx_taps.size())));
    BOOST_REQUIRE_EQUAL(a1.size(), a2.size());
    // No cross-burst state leak: A,B,A must reproduce A.  The reused core is
    // "prefer bitwise" but this build's VOLK/AVX2 kernels reproduce only to
    // ~1e-7 (a core-level uninitialized-read finding reported separately), so
    // a 1e-5 bound is used; a real state leak would be O(0.1+) and is caught.
    BOOST_CHECK_MESSAGE(max_abs_diff(a1, a2) <= 1e-5f,
                        std::string(name) + " burst A leaked into the next call");

    // RX direction: same statelessness property.
    BOOST_REQUIRE_MESSAGE(
        twr::m2a_rx_resample(a1.data(), a1.size(), cfg, a_rx, tr, why),
        std::string(name) + " rx A: " + why);
    BOOST_REQUIRE_MESSAGE(
        twr::m2a_rx_resample(a1.data(), a1.size(), cfg, a_rx2, tr, why),
        std::string(name) + " rx A again: " + why);
    BOOST_CHECK_EQUAL(
        a_rx.size(),
        static_cast<size_t>(frozen_len(a1.size(), 65, l, cfg.rx_taps.size())));
    BOOST_REQUIRE_EQUAL(a_rx.size(), a_rx2.size());
    BOOST_CHECK_MESSAGE(max_abs_diff(a_rx, a_rx2) <= 1e-5f,
                        std::string(name) + " RX burst A leaked into the next call");

    BOOST_TEST_MESSAGE("A04 helper[" << name << "] tx=" << a1.size()
                                     << " rx=" << a_rx.size() << " stateless");
}

} // namespace

// ===========================================================================
// A01 -- 2 native rates x Poll/Response/Final x CF32/SC16 = 12 clean units,
//        each run with TWO distinct (timestamp, addr, seq) sets.  The real
//        codec produces the bytes; the FCS and PSDU are independent.
// ===========================================================================
BOOST_AUTO_TEST_CASE(m2a_a01_two_rates_three_frames_two_formats)
{
    const twr::FrameProfile prof;
    const twr::M2aNativeRate rates[] = { twr::M2aNativeRate::Uc200_737280000,
                                         twr::M2aNativeRate::Cg400_491520000 };
    const twr::FrameType frames[] = { twr::FrameType::Poll,
                                      twr::FrameType::Response,
                                      twr::FrameType::Final };
    const twr::M2aIqFormat formats[] = { twr::M2aIqFormat::Cf32,
                                         twr::M2aIqFormat::Sc16 };

    // Independent numeric table for the three TWR frames (G0 §1.3).
    BOOST_CHECK_EQUAL(expected_work_samples(16), size_t(117184));
    BOOST_CHECK_EQUAL(expected_work_samples(26), size_t(127424));
    BOOST_CHECK_EQUAL(expected_work_samples(31), size_t(132544));
    BOOST_CHECK_EQUAL(expected_work_samples(127), size_t(249280));

    size_t units = 0;
    for (auto rate : rates) {
        for (auto ft : frames) {
            for (auto fmt : formats) {
                const twr::M2aConfig cfg = make_cfg(rate, fmt);
                for (int variant = 0; variant < 2; ++variant) {
                    const twr::Frame f = make_frame(ft, variant);
                    const std::string label =
                        std::string(twr::m2a_native_rate_to_string(rate)) + "/" +
                        twr::frame_type_to_string(ft) + "/" +
                        twr::m2a_iq_format_to_string(fmt) + "/v" +
                        std::to_string(variant);
                    check_clean_roundtrip(f, prof, cfg, label);
                }
                ++units;
            }
        }
    }
    BOOST_CHECK_EQUAL(units, size_t(12));
}

// ===========================================================================
// A02 -- FCS ownership: exactly one append, independent CRC-16, and a PSDU
//        that still carries its FCS (or is malformed) is rejected.
// ===========================================================================
BOOST_AUTO_TEST_CASE(m2a_a02_fcs_ownership_and_malformed_rejection)
{
    const twr::FrameProfile prof;
    const twr::Frame f = make_frame(twr::FrameType::Poll, 0);

    std::string err;
    std::vector<uint8_t> mac;
    BOOST_REQUIRE_MESSAGE(twr::encode(f, prof, mac, err), err);
    BOOST_REQUIRE_EQUAL(mac.size(), size_t(14));

    // Independent CRC and LE append order.
    const uint16_t ref = crc16_ref(mac.data(), mac.size());
    std::vector<uint8_t> psdu = mac;
    mod::append_ieee_fcs(psdu);
    BOOST_CHECK_MESSAGE(psdu.size() == mac.size() + 2,
                        "append_ieee_fcs must add exactly 2 bytes");
    BOOST_CHECK_EQUAL(psdu[mac.size()], static_cast<uint8_t>(ref & 0xff));
    BOOST_CHECK_EQUAL(psdu[mac.size() + 1], static_cast<uint8_t>(ref >> 8));

    // POSITIVE CONTROL: the MAC payload (no FCS) decodes.
    {
        twr::Frame out;
        std::string e;
        twr::FrameError code = twr::FrameError::None;
        BOOST_CHECK_MESSAGE(twr::decode(mac.data(), mac.size(), prof, out, e, &code),
                            "codec refused a valid MAC payload: " + e);
        BOOST_CHECK(code == twr::FrameError::None);
        BOOST_CHECK(out == f);
    }

    // A buffer still carrying the FCS is LengthMismatch, never reinterpreted.
    {
        twr::Frame out;
        std::string e;
        twr::FrameError code = twr::FrameError::None;
        BOOST_CHECK_MESSAGE(!twr::decode(psdu.data(), psdu.size(), prof, out, e, &code),
                            "codec accepted a PSDU with the FCS still attached");
        BOOST_CHECK_MESSAGE(code == twr::FrameError::LengthMismatch,
                            "expected LengthMismatch, got " +
                                std::string(twr::frame_error_to_string(code)));
    }

    // Double FCS is also rejected.
    {
        std::vector<uint8_t> twice = psdu;
        mod::append_ieee_fcs(twice);
        twr::Frame out;
        std::string e;
        twr::FrameError code = twr::FrameError::None;
        BOOST_CHECK(!twr::decode(twice.data(), twice.size(), prof, out, e, &code));
        BOOST_CHECK(code == twr::FrameError::LengthMismatch);
    }

    // Short / truncated input is refused, not guessed.
    {
        twr::Frame out;
        std::string e;
        twr::FrameError code = twr::FrameError::None;
        BOOST_CHECK_MESSAGE(!twr::decode(mac.data(), mac.size() - 1, prof, out, e, &code),
                            "codec accepted a truncated payload");
        BOOST_CHECK(code == twr::FrameError::Truncated);
    }

    // Malformed (14 zero bytes: version 0) is refused.
    {
        std::vector<uint8_t> junk(14, 0);
        twr::Frame out;
        std::string e;
        twr::FrameError code = twr::FrameError::None;
        BOOST_CHECK_MESSAGE(!twr::decode(junk.data(), junk.size(), prof, out, e, &code),
                            "codec accepted 14 zero bytes as a frame");
        BOOST_CHECK(code == twr::FrameError::BadVersion);
    }

    // The PSDU->MAC framing step strips exactly two bytes and the result decodes.
    {
        const uint8_t* payload = nullptr;
        size_t n = 0;
        std::string e;
        twr::FrameError code = twr::FrameError::None;
        BOOST_REQUIRE(twr::mac_payload_from_psdu(psdu.data(), psdu.size(), prof,
                                                 payload, n, e, &code));
        BOOST_CHECK_EQUAL(n, mac.size());
        twr::Frame out;
        BOOST_CHECK(twr::decode(payload, n, prof, out, e, &code));
        BOOST_CHECK(out == f);
    }
}

// ===========================================================================
// A04 -- chunking / short output buffer / reset semantics.  The core is
//        driven directly for chunk invariance; the helper is checked for
//        statelessness and the frozen length.
// ===========================================================================
BOOST_AUTO_TEST_CASE(m2a_a04_chunk_capacity_reset)
{
    check_core_chunk_invariance<gr::uwb::core::RationalResampler48_65Core>(
        "tx_48_65", 48, 65, tx_taps_for(twr::M2aNativeRate::Uc200_737280000));
    check_core_chunk_invariance<gr::uwb::core::RationalResampler32_65Core>(
        "tx_32_65", 32, 65, tx_taps_for(twr::M2aNativeRate::Cg400_491520000));
    check_core_chunk_invariance<gr::uwb::core::RationalResampler65_48Core>(
        "rx_65_48", 65, 48, rx_taps_for(twr::M2aNativeRate::Uc200_737280000));
    check_core_chunk_invariance<gr::uwb::core::RationalResampler65_32Core>(
        "rx_65_32", 65, 32, rx_taps_for(twr::M2aNativeRate::Cg400_491520000));

    check_helper_stateless_bursts(twr::M2aNativeRate::Uc200_737280000, "uc200");
    check_helper_stateless_bursts(twr::M2aNativeRate::Cg400_491520000, "cg400");

    // N == 0 is defined as ZERO output with no transient (G0 §3.2).
    {
        const twr::M2aConfig cfg = make_cfg(twr::M2aNativeRate::Uc200_737280000,
                                            twr::M2aIqFormat::Cf32);
        std::vector<gr_complex> empty, out;
        twr::M2aStageTrace tr;
        std::string why;
        (void)twr::m2a_tx_resample(empty.data(), 0, cfg, out, tr, why);
        BOOST_CHECK_MESSAGE(out.empty(), "tx_resample N=0 produced a transient");
        (void)twr::m2a_rx_resample(empty.data(), 0, cfg, out, tr, why);
        BOOST_CHECK_MESSAGE(out.empty(), "rx_resample N=0 produced a transient");
    }
}

// ===========================================================================
// A06 -- SC16 quantisation: 0, +-full scale, round-half-away-from-zero,
//        saturation counting, NaN/Inf refused.
// ===========================================================================
BOOST_AUTO_TEST_CASE(m2a_a06_sc16_quantisation)
{
    twr::M2aConfig cfg;
    cfg.peak_amplitude = 0.8f;
    const float S = cfg.effective_sc16_scale();
    BOOST_CHECK_CLOSE(S, 32767.0f / 0.8f, 0.001);
    cfg.sc16_scale = 100.0f;
    BOOST_CHECK_EQUAL(cfg.effective_sc16_scale(), 100.0f);

    // Round-half-away-from-zero, tested with scale == 1 so the products are
    // exact half-integers.
    {
        std::vector<gr_complex> in = { { 2.5f, 0.f },  { 1.5f, 0.f },
                                       { 0.5f, 0.f },  { -0.5f, 0.f },
                                       { -1.5f, 0.f }, { -2.5f, 0.f },
                                       { 0.f, 0.f } };
        std::vector<gr_complex> out;
        size_t sat = 99;
        std::string why;
        BOOST_REQUIRE_MESSAGE(
            twr::m2a_sc16_roundtrip(in.data(), in.size(), 1.0f, out, sat, why),
            "sc16 scale=1 refused: " + why);
        const float want[] = { 3.f, 2.f, 1.f, -1.f, -2.f, -3.f, 0.f };
        BOOST_REQUIRE_EQUAL(out.size(), in.size());
        for (size_t i = 0; i < out.size(); ++i) {
            BOOST_CHECK_EQUAL(out[i].real(), want[i]);
            BOOST_CHECK_EQUAL(out[i].imag(), 0.f);
        }
        BOOST_CHECK_EQUAL(sat, size_t(0));
    }

    // Zero is exactly zero (no dither, no offset).
    {
        std::vector<gr_complex> in(4, gr_complex(0, 0)), out;
        size_t sat = 7;
        std::string why;
        BOOST_REQUIRE(twr::m2a_sc16_roundtrip(in.data(), in.size(), S, out, sat, why));
        BOOST_REQUIRE_EQUAL(out.size(), in.size());
        for (const auto& c : out)
            BOOST_CHECK(c == gr_complex(0, 0));
        BOOST_CHECK_EQUAL(sat, size_t(0));
    }

    // +-full scale maps to +-32767 and back, with no saturation.
    {
        std::vector<gr_complex> in = { { 0.8f, 0.f }, { -0.8f, 0.f } };
        std::vector<gr_complex> out;
        size_t sat = 7;
        std::string why;
        BOOST_REQUIRE(twr::m2a_sc16_roundtrip(in.data(), in.size(), S, out, sat, why));
        BOOST_REQUIRE_EQUAL(out.size(), size_t(2));
        BOOST_CHECK_CLOSE(out[0].real(), 32767.0f / S, 0.001);
        BOOST_CHECK_CLOSE(out[1].real(), -32767.0f / S, 0.001);
        BOOST_CHECK_EQUAL(sat, size_t(0));
    }

    // Saturation is clamped AND counted: +1.0 -> +32767, -1.0 -> -32768.
    {
        std::vector<gr_complex> in = { { 1.0f, 0.f }, { -1.0f, 0.f }, { 2.0f, 0.f } };
        std::vector<gr_complex> out;
        size_t sat = 0;
        std::string why;
        BOOST_REQUIRE(twr::m2a_sc16_roundtrip(in.data(), in.size(), S, out, sat, why));
        BOOST_REQUIRE_EQUAL(out.size(), size_t(3));
        BOOST_CHECK_CLOSE(out[0].real(), 32767.0f / S, 0.001);
        BOOST_CHECK_CLOSE(out[1].real(), -32768.0f / S, 0.001);
        BOOST_CHECK_CLOSE(out[2].real(), 32767.0f / S, 0.001);
        BOOST_CHECK_EQUAL(sat, size_t(3));
    }

    // NaN / Inf are refused, never silently coerced to 0.
    {
        std::vector<gr_complex> in = { { std::numeric_limits<float>::quiet_NaN(), 0.f } };
        std::vector<gr_complex> out;
        size_t sat = 0;
        std::string why;
        BOOST_CHECK_MESSAGE(!twr::m2a_sc16_roundtrip(in.data(), in.size(), S, out, sat, why),
                            "sc16 accepted a NaN real part");
        BOOST_CHECK(!why.empty());
    }
    {
        std::vector<gr_complex> in = { { 0.f, std::numeric_limits<float>::infinity() } };
        std::vector<gr_complex> out;
        size_t sat = 0;
        std::string why;
        BOOST_CHECK_MESSAGE(!twr::m2a_sc16_roundtrip(in.data(), in.size(), S, out, sat, why),
                            "sc16 accepted an Inf imaginary part");
        BOOST_CHECK(!why.empty());
    }

    // Empty input: zero output.
    {
        std::vector<gr_complex> empty, out;
        size_t sat = 0;
        std::string why;
        (void)twr::m2a_sc16_roundtrip(empty.data(), 0, S, out, sat, why);
        BOOST_CHECK_MESSAGE(out.empty(), "sc16 N=0 produced samples");
        BOOST_CHECK_EQUAL(sat, size_t(0));
    }
}

// ===========================================================================
// A07 -- fail-closed config/entry: out-of-domain enums, STS, bad taps,
//        oversized input, all with a named reason.
// ===========================================================================
BOOST_AUTO_TEST_CASE(m2a_a07_fail_closed_config_and_entry)
{
    const twr::M2aConfig good =
        make_cfg(twr::M2aNativeRate::Uc200_737280000, twr::M2aIqFormat::Cf32);
    std::string why;
    BOOST_CHECK_MESSAGE(good.is_valid(why), "valid config rejected: " + why);

    // Out-of-domain enums: a value the enum does not have must fail closed and
    // be named BEFORE any dependent check (M0.1 N07).
    {
        twr::M2aConfig c = good;
        c.native_rate = static_cast<twr::M2aNativeRate>(2);
        why.clear();
        BOOST_CHECK_MESSAGE(!c.is_valid(why), "native_rate=2 accepted");
        BOOST_CHECK_MESSAGE(why.find("native_rate") != std::string::npos,
                            "reason must name native_rate: " + why);
        BOOST_CHECK(!twr::m2a_native_rate_is_known(c.native_rate));
        uint32_t l = 0, m = 0;
        BOOST_CHECK(!twr::m2a_rate_tx_lm(c.native_rate, l, m));
    }
    {
        twr::M2aConfig c = good;
        c.iq_format = static_cast<twr::M2aIqFormat>(2);
        why.clear();
        BOOST_CHECK_MESSAGE(!c.is_valid(why), "iq_format=2 accepted");
        BOOST_CHECK_MESSAGE(why.find("iq_format") != std::string::npos,
                            "reason must name iq_format: " + why);
        BOOST_CHECK(!twr::m2a_iq_format_is_known(c.iq_format));
    }

    // STS is out of scope.
    {
        twr::M2aConfig c = good;
        c.insert_sts = true;
        why.clear();
        BOOST_CHECK_MESSAGE(!c.is_valid(why), "insert_sts=true accepted");
        BOOST_CHECK_MESSAGE(why.find("sts") != std::string::npos,
                            "reason must mention STS: " + why);
    }

    // Bad taps: wrong DC sum, too few, non-finite.
    {
        twr::M2aConfig c = good;
        c.tx_taps = { 1.0f, 1.0f };
        why.clear();
        BOOST_CHECK_MESSAGE(!c.is_valid(why), "wrong-DC tx_taps accepted");
        BOOST_CHECK_MESSAGE(why.find("tx_taps") != std::string::npos,
                            "reason must name tx_taps: " + why);
    }
    {
        twr::M2aConfig c = good;
        c.tx_taps = { 48.0f };
        why.clear();
        BOOST_CHECK_MESSAGE(!c.is_valid(why), "1-tap tx_taps accepted");
        BOOST_CHECK_MESSAGE(why.find("tx_taps") != std::string::npos, why);
    }
    {
        twr::M2aConfig c = good;
        c.rx_taps = { 65.0f, 1.0f };
        why.clear();
        BOOST_CHECK_MESSAGE(!c.is_valid(why), "wrong-DC rx_taps accepted");
        BOOST_CHECK_MESSAGE(why.find("rx_taps") != std::string::npos, why);
    }
    {
        // Any non-finite coefficient makes the DC sum non-finite, so the
        // refusal is guaranteed; the exact reason word is not pinned.
        twr::M2aConfig c = good;
        c.tx_taps[0] = std::numeric_limits<float>::quiet_NaN();
        why.clear();
        BOOST_CHECK_MESSAGE(!c.is_valid(why), "non-finite tx_taps accepted");
        BOOST_CHECK(!why.empty());
    }

    // Non-positive peak amplitude.
    {
        twr::M2aConfig c = good;
        c.peak_amplitude = 0.0f;
        why.clear();
        BOOST_CHECK_MESSAGE(!c.is_valid(why), "peak_amplitude=0 accepted");
    }

    // The closed loop refuses an out-of-domain config and names the status.
    {
        const twr::FrameProfile prof;
        twr::M2aConfig c = good;
        c.native_rate = static_cast<twr::M2aNativeRate>(9);
        twr::M2aResult out;
        why.clear();
        const bool ok = twr::m2a_native_roundtrip(make_frame(twr::FrameType::Poll, 0),
                                                  prof, c, out, why);
        BOOST_CHECK_MESSAGE(!ok, "closed loop accepted out-of-domain native_rate");
        BOOST_CHECK_EQUAL(static_cast<int>(out.status),
                          static_cast<int>(twr::M2aStatus::InvalidConfig));
        BOOST_CHECK_MESSAGE(!why.empty(), "InvalidConfig must carry a reason");
        BOOST_CHECK(!out.measurement_valid);
    }

    // Oversized input is refused by both resampler stages with a reason.
    {
        std::vector<gr_complex> big(twr::kM2aMaxSamples + 1,
                                    gr_complex(0.1f, 0.1f));
        std::vector<gr_complex> out;
        twr::M2aStageTrace tr;
        why.clear();
        BOOST_CHECK_MESSAGE(
            !twr::m2a_tx_resample(big.data(), big.size(), good, out, tr, why),
            "tx_resample accepted N > 2^20");
        BOOST_CHECK_MESSAGE(!why.empty(), "oversized tx refusal must name a reason");
        why.clear();
        BOOST_CHECK_MESSAGE(
            !twr::m2a_rx_resample(big.data(), big.size(), good, out, tr, why),
            "rx_resample accepted N > 2^20");
        BOOST_CHECK_MESSAGE(!why.empty(), "oversized rx refusal must name a reason");
    }

    // POSITIVE CONTROL: a modest input is accepted and has the frozen length.
    {
        std::vector<gr_complex> x(1024, gr_complex(0.25f, -0.1f));
        std::vector<gr_complex> out;
        twr::M2aStageTrace tr;
        why.clear();
        BOOST_REQUIRE_MESSAGE(
            twr::m2a_tx_resample(x.data(), x.size(), good, out, tr, why),
            "valid tx_resample refused: " + why);
        uint32_t l = 0, m = 0;
        BOOST_REQUIRE(twr::m2a_rate_tx_lm(good.native_rate, l, m));
        BOOST_CHECK_EQUAL(out.size(),
                          static_cast<size_t>(frozen_len(1024, l, m, good.tx_taps.size())));
    }
}

// ===========================================================================
// A08 -- boundaries: 127 B PSDU, minimal PSDU, short/empty input, and a
//        distinguishable bad-FCS vs no-packet outcome.  No stale result.
// ===========================================================================
BOOST_AUTO_TEST_CASE(m2a_a08_boundaries_and_error_distinction)
{
    const twr::FrameProfile prof;
    const twr::M2aConfig cfg =
        make_cfg(twr::M2aNativeRate::Uc200_737280000, twr::M2aIqFormat::Cf32);

    // ---- 127 B PSDU (the PHY capacity boundary; NOT a TWR frame type) ----
    std::vector<uint8_t> mac127(125);
    for (size_t i = 0; i < mac127.size(); ++i)
        mac127[i] = static_cast<uint8_t>((0x5a + 7 * i) & 0xff);
    std::vector<uint8_t> expect127 = mac127;
    const uint16_t ref127 = crc16_ref(mac127.data(), mac127.size());
    mod::append_ieee_fcs(expect127);
    BOOST_REQUIRE_EQUAL(expect127.size(), size_t(127));
    BOOST_CHECK_EQUAL(expect127[125], static_cast<uint8_t>(ref127 & 0xff));

    std::vector<gr_complex> work127;
    twr::M2aStageTrace tr;
    std::string why;
    BOOST_REQUIRE_MESSAGE(
        twr::m2a_modulate_to_work(mac127.data(), mac127.size(), cfg, work127, tr, why),
        "127 B modulate refused: " + why);
    BOOST_CHECK_EQUAL(work127.size(), expected_work_samples(127));

    std::vector<gr_complex> native127, rx127;
    BOOST_REQUIRE_MESSAGE(
        twr::m2a_tx_resample(work127.data(), work127.size(), cfg, native127, tr, why),
        "127 B tx resample: " + why);
    BOOST_REQUIRE_MESSAGE(
        twr::m2a_rx_resample(native127.data(), native127.size(), cfg, rx127, tr, why),
        "127 B rx resample: " + why);
    {
        demod::DemodResult res;
        BOOST_REQUIRE_MESSAGE(
            twr::m2a_demod_work(rx127.data(), rx127.size(), cfg, res, tr, why),
            "127 B demod: " + why);
        BOOST_CHECK_MESSAGE(res.status == demod::DemodStatus::Success,
                            "127 B status=" +
                                std::to_string(static_cast<int>(res.status)));
        BOOST_CHECK_MESSAGE(res.payload.fcs_pass, "127 B fcs_pass false");
        BOOST_CHECK_MESSAGE(res.payload.bytes == expect127,
                            "127 B decoded bytes differ from the input PSDU");
    }

    // ---- Minimal PSDU: a Poll (14 B MAC -> 16 B PSDU) via the same stages ----
    {
        std::vector<uint8_t> mac;
        std::string err;
        BOOST_REQUIRE(twr::encode(make_frame(twr::FrameType::Poll, 0), prof, mac, err));
        std::vector<uint8_t> expect = mac;
        mod::append_ieee_fcs(expect);
        std::vector<gr_complex> w, nat, rwx;
        BOOST_REQUIRE(twr::m2a_modulate_to_work(mac.data(), mac.size(), cfg, w, tr, why));
        BOOST_CHECK_EQUAL(w.size(), expected_work_samples(16));
        BOOST_REQUIRE(twr::m2a_tx_resample(w.data(), w.size(), cfg, nat, tr, why));
        BOOST_REQUIRE(twr::m2a_rx_resample(nat.data(), nat.size(), cfg, rwx, tr, why));
        demod::DemodResult res;
        BOOST_REQUIRE(twr::m2a_demod_work(rwx.data(), rwx.size(), cfg, res, tr, why));
        BOOST_CHECK_MESSAGE(res.status == demod::DemodStatus::Success,
                            "minimal PSDU status=" +
                                std::to_string(static_cast<int>(res.status)));
        BOOST_CHECK(res.payload.bytes == expect);
    }

    // ---- Truncated input: must not decode ----
    {
        demod::DemodResult res;
        std::string e;
        const bool ok = twr::m2a_demod_work(work127.data(), 64, cfg, res, tr, e);
        // The frozen stage contract leaves `out` unspecified when the stage
        // fails, so only constrain it when the call claims success.
        BOOST_CHECK_MESSAGE(!ok || res.status != demod::DemodStatus::Success,
                            "demod reported Success for a 64-sample truncated buffer");
    }

    // ---- Empty input: no transient, no decode ----
    {
        std::vector<gr_complex> empty, out;
        std::string e;
        (void)twr::m2a_tx_resample(empty.data(), 0, cfg, out, tr, e);
        BOOST_CHECK_MESSAGE(out.empty(), "tx_resample N=0 produced a transient");
        demod::DemodResult res;
        const bool ok = twr::m2a_demod_work(empty.data(), 0, cfg, res, tr, e);
        BOOST_CHECK_MESSAGE(!ok || res.status != demod::DemodStatus::Success,
                            "demod reported Success for an empty buffer");
    }

    // ---- Bad FCS (a valid PSDU with a wrong CRC) vs no packet ----
    demod::DemodStatus bad_fcs_status = demod::DemodStatus::Success;
    {
        std::vector<uint8_t> mac;
        std::string err;
        BOOST_REQUIRE(twr::encode(make_frame(twr::FrameType::Poll, 0), prof, mac, err));
        std::vector<uint8_t> bad = mac;
        const uint16_t wrong =
            static_cast<uint16_t>(crc16_ref(mac.data(), mac.size()) ^ 0xffffu);
        bad.push_back(static_cast<uint8_t>(wrong & 0xff));
        bad.push_back(static_cast<uint8_t>(wrong >> 8));

        std::vector<gr_complex> w, nat, rwx;
        BOOST_REQUIRE_MESSAGE(modulate_psdu(bad, cfg, w, why),
                              "bad-FCS modulate refused: " + why);
        BOOST_REQUIRE(twr::m2a_tx_resample(w.data(), w.size(), cfg, nat, tr, why));
        BOOST_REQUIRE(twr::m2a_rx_resample(nat.data(), nat.size(), cfg, rwx, tr, why));
        demod::DemodResult res;
        (void)twr::m2a_demod_work(rwx.data(), rwx.size(), cfg, res, tr, why);
        BOOST_CHECK_MESSAGE(!res.payload.fcs_pass,
                            "a deliberately wrong FCS was reported as passing");
        bad_fcs_status = res.status;
        BOOST_CHECK_MESSAGE(res.status == demod::DemodStatus::FcsFailed,
                            "bad-FCS status must be FcsFailed, got " +
                                std::to_string(static_cast<int>(res.status)));
        BOOST_CHECK_MESSAGE(res.payload.bytes == bad,
                            "bad-FCS decoded bytes differ from the input PSDU");
    }
    {
        // A window containing only noise must be a DEMOD failure, never an FCS
        // failure -- that is what distinguishes the two negative classes.
        std::vector<gr_complex> noise(work127.size());
        std::mt19937 rng(0xBADC0DE);
        std::uniform_real_distribution<float> d(-0.3f, 0.3f);
        for (auto& c : noise)
            c = gr_complex(d(rng), d(rng));
        demod::DemodResult res;
        std::string e;
        (void)twr::m2a_demod_work(noise.data(), noise.size(), cfg, res, tr, e);
        BOOST_CHECK_MESSAGE(res.status != demod::DemodStatus::Success,
                            "a noise-only window decoded as Success");
        BOOST_CHECK_MESSAGE(res.status != demod::DemodStatus::FcsFailed,
                            "noise reported FcsFailed; must be a demod failure");
        BOOST_CHECK_MESSAGE(res.status != bad_fcs_status,
                            "bad-FCS and no-packet are not distinguishable");
    }

    // ---- Never silently return a previous result ----
    {
        twr::M2aResult out;
        std::string e;
        BOOST_REQUIRE(twr::m2a_native_roundtrip(make_frame(twr::FrameType::Poll, 0),
                                                prof, cfg, out, e));
        BOOST_REQUIRE(out.ok);
        twr::M2aConfig bad = cfg;
        bad.native_rate = static_cast<twr::M2aNativeRate>(3);
        e.clear();
        BOOST_CHECK(!twr::m2a_native_roundtrip(make_frame(twr::FrameType::Poll, 0),
                                               prof, bad, out, e));
        BOOST_CHECK_MESSAGE(!out.ok, "a stale ok=true survived a refused config");
        BOOST_CHECK_EQUAL(static_cast<int>(out.status),
                          static_cast<int>(twr::M2aStatus::InvalidConfig));
    }
    {
        // Same property for the demod stage.  Use the band-limited waveform the
        // demod actually consumes (rx127), not the raw modulator output.
        demod::DemodResult res;
        std::string e;
        BOOST_REQUIRE(twr::m2a_demod_work(rx127.data(), rx127.size(), cfg, res, tr, e));
        BOOST_REQUIRE(res.status == demod::DemodStatus::Success);
        std::vector<gr_complex> noise(work127.size(), gr_complex(0.2f, -0.2f));
        (void)twr::m2a_demod_work(noise.data(), noise.size(), cfg, res, tr, e);
        BOOST_CHECK_MESSAGE(res.status != demod::DemodStatus::Success,
                            "a stale demod Success survived a failed decode");
    }
}

// ===========================================================================
// A09 -- random leading blank.  The decoder is NEVER told where the frame is,
//        so this fails if the QA (or the helper) leaks the truth.
// ===========================================================================
BOOST_AUTO_TEST_CASE(m2a_a09_random_leading_blank_search)
{
    const twr::FrameProfile prof;
    const twr::M2aConfig cfg =
        make_cfg(twr::M2aNativeRate::Uc200_737280000, twr::M2aIqFormat::Cf32);

    std::vector<uint8_t> mac_a;
    std::string err;
    BOOST_REQUIRE(twr::encode(make_frame(twr::FrameType::Poll, 0), prof, mac_a, err));
    std::vector<uint8_t> expect_a = mac_a;
    mod::append_ieee_fcs(expect_a);

    // The demod's input in the M2-A chain is the band-limited waveform that
    // comes out of the RX resampler, so that is what is embedded here.
    std::vector<gr_complex> frame_a;
    twr::M2aStageTrace tr;
    std::string why;
    BOOST_REQUIRE_MESSAGE(
        make_bandlimited_work(mac_a, cfg, frame_a, why),
        "A09 band-limited frame build: " + why);

    // POSITIVE CONTROL: no leading blank decodes.
    {
        demod::DemodResult res;
        (void)twr::m2a_demod_work(frame_a.data(), frame_a.size(), cfg, res, tr, why);
        BOOST_REQUIRE_MESSAGE(res.status == demod::DemodStatus::Success,
                              "clean frame did not decode: " + why);
        BOOST_REQUIRE(res.payload.bytes == expect_a);
    }

    // Random leading blanks; the demod is not given the start.
    const size_t blanks[] = { 0, 137, 2048 };
    for (size_t N : blanks) {
        std::vector<gr_complex> buf;
        if (N > 0) {
            buf.resize(N);
            std::mt19937 rng(static_cast<uint32_t>(0xC0FFEEu + N));
            std::uniform_real_distribution<float> d(-0.25f, 0.25f);
            for (auto& c : buf)
                c = gr_complex(d(rng), d(rng));
        }
        buf.insert(buf.end(), frame_a.begin(), frame_a.end());
        demod::DemodResult res;
        std::string e;
        (void)twr::m2a_demod_work(buf.data(), buf.size(), cfg, res, tr, e);
        BOOST_CHECK_MESSAGE(
            res.status == demod::DemodStatus::Success,
            "leading blank N=" + std::to_string(N) +
                " was not found: status=" +
                std::to_string(static_cast<int>(res.status)) + " why=" + e);
        BOOST_CHECK_MESSAGE(res.payload.bytes == expect_a,
                            "leading blank N=" + std::to_string(N) +
                                " decoded the wrong bytes");
        BOOST_CHECK_MESSAGE(res.payload.fcs_pass,
                            "leading blank N=" + std::to_string(N) + " fcs_pass false");
    }

    // NEGATIVE CONTROL: a window with no packet must not decode.
    {
        std::vector<gr_complex> noise(frame_a.size());
        std::mt19937 rng(4242u);
        std::uniform_real_distribution<float> d(-0.3f, 0.3f);
        for (auto& c : noise)
            c = gr_complex(d(rng), d(rng));
        demod::DemodResult res;
        std::string e;
        (void)twr::m2a_demod_work(noise.data(), noise.size(), cfg, res, tr, e);
        BOOST_CHECK_MESSAGE(res.status != demod::DemodStatus::Success,
                            "a window with no packet decoded as Success");
    }

    // Consecutive DIFFERENT frames: each must decode to its own bytes.
    {
        std::vector<uint8_t> mac_b;
        BOOST_REQUIRE(twr::encode(make_frame(twr::FrameType::Poll, 1), prof, mac_b, err));
        std::vector<uint8_t> expect_b = mac_b;
        mod::append_ieee_fcs(expect_b);
        BOOST_CHECK_MESSAGE(expect_a != expect_b,
                            "the two variants must actually differ");
        std::vector<gr_complex> frame_b;
        BOOST_REQUIRE_MESSAGE(make_bandlimited_work(mac_b, cfg, frame_b, why),
                              "A09 frame B build: " + why);
        demod::DemodResult ra, rb;
        (void)twr::m2a_demod_work(frame_a.data(), frame_a.size(), cfg, ra, tr, why);
        (void)twr::m2a_demod_work(frame_b.data(), frame_b.size(), cfg, rb, tr, why);
        BOOST_CHECK_MESSAGE(ra.payload.bytes == expect_a,
                            "consecutive frame A decoded to the wrong bytes");
        BOOST_CHECK_MESSAGE(rb.payload.bytes == expect_b,
                            "consecutive frame B decoded to the wrong bytes");
    }
}
