/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * QA for the HRP BPRF modulator against testdata/realtime_demod_golden.
 */

#include <boost/test/unit_test.hpp>

#include <gnuradio/gr_complex.h>
#include <gnuradio/uwb/uwb_demod_core.h>
#include <gnuradio/uwb/uwb_hrp_mod_core.h>
#include <gnuradio/uwb/uwb_phy_profile.h>
#include <gnuradio/uwb/uwb_radar_sfd_core.h>

#include <cmath>
#include <cstdint>
#include <fstream>
#include <string>
#include <vector>

namespace {

using gr::uwb::demod::Qm35825Profile;
using gr::uwb::demod::core::demodulate_one;
using gr::uwb::mod::HrpModConfig;
using gr::uwb::mod::HrpModScratch;
using gr::uwb::mod::modulate_one;
using gr::uwb::mod::packet_samples_998p4;

#ifdef UWB_TESTDATA_DIR
const char* kTestdata = UWB_TESTDATA_DIR;
#else
const char* kTestdata = "../../../testdata";
#endif

std::string golden_dir()
{
    return std::string(kTestdata) + "/realtime_demod_golden";
}

bool load_cf32(const std::string& path, std::vector<gr_complex>& out)
{
    std::ifstream f(path, std::ios::binary);
    if (!f)
        return false;
    f.seekg(0, std::ios::end);
    const auto bytes = static_cast<size_t>(f.tellg());
    f.seekg(0);
    if (bytes == 0 || bytes % sizeof(gr_complex) != 0)
        return false;
    out.resize(bytes / sizeof(gr_complex));
    f.read(reinterpret_cast<char*>(out.data()),
           static_cast<std::streamsize>(bytes));
    return static_cast<bool>(f);
}

bool load_u8(const std::string& path, std::vector<uint8_t>& out)
{
    std::ifstream f(path, std::ios::binary);
    if (!f)
        return false;
    f.seekg(0, std::ios::end);
    const auto bytes = static_cast<size_t>(f.tellg());
    f.seekg(0);
    out.resize(bytes);
    f.read(reinterpret_cast<char*>(out.data()),
           static_cast<std::streamsize>(bytes));
    return static_cast<bool>(f);
}

bool load_i8(const std::string& path, std::vector<int8_t>& out)
{
    std::ifstream f(path, std::ios::binary);
    if (!f)
        return false;
    f.seekg(0, std::ios::end);
    const auto bytes = static_cast<size_t>(f.tellg());
    f.seekg(0);
    out.resize(bytes);
    f.read(reinterpret_cast<char*>(out.data()),
           static_cast<std::streamsize>(bytes));
    return static_cast<bool>(f);
}

std::vector<gr_complex> load_template()
{
    std::vector<gr_complex> tmpl;
    BOOST_REQUIRE(load_cf32(std::string(kTestdata) + "/reference_preamble.bin",
                            tmpl));
    return tmpl;
}

double l2_rel(const std::vector<gr_complex>& a, const gr_complex* b, size_t n)
{
    double num = 0.0, den = 0.0;
    for (size_t i = 0; i < n; ++i) {
        const double dr = static_cast<double>(a[i].real()) - b[i].real();
        const double di = static_cast<double>(a[i].imag()) - b[i].imag();
        num += dr * dr + di * di;
        den += static_cast<double>(b[i].real()) * b[i].real() +
               static_cast<double>(b[i].imag()) * b[i].imag();
    }
    return std::sqrt(num / (den + 1e-30));
}

} // namespace

BOOST_AUTO_TEST_CASE(test_hrp_mod_packet_length_p1)
{
    BOOST_CHECK_EQUAL(packet_samples_998p4(64, 8, 127), size_t(249280));
}

BOOST_AUTO_TEST_CASE(test_hrp_mod_rs_matches_golden)
{
    std::vector<uint8_t> bytes;
    BOOST_REQUIRE(load_u8(golden_dir() + "/stage_payload_bytes.bin", bytes));
    BOOST_REQUIRE_EQUAL(bytes.size(), size_t(127));

    HrpModScratch scratch;
    scratch.reserve(127, 249280);
    gr::uwb::mod::detail::bytes_to_lsb_bits(bytes.data(), bytes.size(),
                                            scratch.psdu_bits);
    BOOST_REQUIRE(gr::uwb::mod::detail::rs_encode_stream(
        scratch.psdu_bits.data(), scratch.psdu_bits.size(), scratch.rs_cw));
    std::vector<int8_t> golden;
    BOOST_REQUIRE(load_i8(golden_dir() + "/stage_rs_coded.bin", golden));
    BOOST_REQUIRE_EQUAL(scratch.rs_cw.size(), golden.size());
    BOOST_CHECK_EQUAL_COLLECTIONS(scratch.rs_cw.begin(), scratch.rs_cw.end(),
                                  golden.begin(), golden.end());

    std::vector<int8_t> decoded;
    BOOST_REQUIRE(gr::uwb::demod::core::detail::rs_decode_stream(
        scratch.rs_cw, scratch.psdu_bits.size(), decoded));
    BOOST_CHECK_EQUAL_COLLECTIONS(decoded.begin(), decoded.end(),
                                  scratch.psdu_bits.begin(),
                                  scratch.psdu_bits.end());
}

BOOST_AUTO_TEST_CASE(test_hrp_mod_iq_matches_matlab_window)
{
    std::vector<uint8_t> bytes;
    BOOST_REQUIRE(load_u8(golden_dir() + "/stage_payload_bytes.bin", bytes));
    std::vector<gr_complex> window;
    BOOST_REQUIRE(load_cf32(golden_dir() + "/window.cfile", window));
    BOOST_REQUIRE_GE(window.size(), size_t(9984 + 249280));

    HrpModConfig cfg;
    HrpModScratch scratch;
    scratch.reserve(127, 249280);
    std::vector<gr_complex> tx(249280);
    size_t n = 0;
    BOOST_REQUIRE(modulate_one(bytes.data(), bytes.size(), cfg, scratch,
                               tx.data(), tx.size(), n));
    BOOST_REQUIRE_EQUAL(n, size_t(249280));
    const double rel = l2_rel(tx, window.data() + 9984, n);
    BOOST_CHECK_LT(rel, 1e-6);
}

BOOST_AUTO_TEST_CASE(test_hrp_mod_demodulate_one_fcs)
{
    std::vector<uint8_t> bytes;
    BOOST_REQUIRE(load_u8(golden_dir() + "/stage_payload_bytes.bin", bytes));

    HrpModConfig cfg;
    HrpModScratch scratch;
    scratch.reserve(127, 249280);
    std::vector<gr_complex> tx(249280);
    size_t n = 0;
    BOOST_REQUIRE(modulate_one(bytes.data(), bytes.size(), cfg, scratch,
                               tx.data(), tx.size(), n));

    const size_t t0 = 9984;
    std::vector<gr_complex> iq(t0 + n + 4096, gr_complex(0.f, 0.f));
    std::copy(tx.begin(), tx.end(), iq.begin() + static_cast<std::ptrdiff_t>(t0));

    auto prof = Qm35825Profile::Default();
    prof.sfd_mode = "ieee";
    gr::uwb::demod::core::DemodScratch dscratch;
    dscratch.reserve(iq.size());
    const auto res = demodulate_one(iq.data(), iq.size(), prof, 1,
                                    static_cast<int64_t>(t0), 0,
                                    load_template(), dscratch);
    BOOST_REQUIRE(res.status == gr::uwb::demod::DemodStatus::Success);
    BOOST_REQUIRE(res.payload.ok);
    BOOST_CHECK(res.payload.fcs_pass);
    BOOST_REQUIRE_EQUAL(res.payload.bytes.size(), bytes.size());
    BOOST_CHECK_EQUAL_COLLECTIONS(res.payload.bytes.begin(),
                                  res.payload.bytes.end(),
                                  bytes.begin(), bytes.end());
    BOOST_CHECK_EQUAL(res.phr.psdu_length, 127u);
}

BOOST_AUTO_TEST_CASE(test_hrp_mod_rejects_sts_and_bad_len)
{
    HrpModConfig cfg;
    cfg.insert_sts = true; // default sfd ieee — not 4z
    HrpModScratch scratch;
    scratch.reserve(4, 4096);
    std::vector<gr_complex> tx(4096);
    size_t n = 0;
    uint8_t b[2] = { 1, 2 };
    BOOST_CHECK(!modulate_one(b, 2, cfg, scratch, tx.data(), tx.size(), n));
    cfg.insert_sts = false;
    BOOST_CHECK(!modulate_one(b, 128, cfg, scratch, tx.data(), tx.size(), n));
    cfg.sync_repetitions = 16;
    BOOST_CHECK(!modulate_one(b, 2, cfg, scratch, tx.data(), tx.size(), n));
}

BOOST_AUTO_TEST_CASE(test_hrp_mod_psdu0_length_and_phr)
{
    HrpModConfig cfg;
    HrpModScratch scratch;
    const size_t want = packet_samples_998p4(64, 8, 0);
    scratch.reserve(0, want);
    std::vector<gr_complex> tx(want);
    size_t n = 0;
    BOOST_REQUIRE(modulate_one(nullptr, 0, cfg, scratch, tx.data(), tx.size(),
                               n));
    BOOST_REQUIRE_EQUAL(n, want);
    // 0-byte packets end on PHR; do not require payload FCS.  SFD must lock.
    auto tmpl = load_template();
    const auto seq = gr::uwb::demod::GetSfdSequence("ieee");
    gr::uwb::radar::RadarSfdScratch ss;
    BOOST_REQUIRE(gr::uwb::radar::prepare_sfd_template(
        seq.data(), seq.size(), tmpl.data(), tmpl.size(), ss));
    gr::uwb::radar::RadarSfdResult sr;
    BOOST_REQUIRE(gr::uwb::radar::search_sfd(
        tx.data(), tx.size(),
        static_cast<int64_t>(64 * 1016), 64, 0.3f, sr, ss));
    BOOST_CHECK(sr.status == gr::uwb::radar::SfdStatus::Ok);
}

BOOST_AUTO_TEST_CASE(test_hrp_mod_4z2_no_sts_demod_fcs)
{
    auto bytes = gr::uwb::mod::make_random_psdu(8, 20260904u, true);
    BOOST_REQUIRE_EQUAL(bytes.size(), size_t(10));
    HrpModConfig cfg;
    cfg.sfd_mode = "4z2";
    HrpModScratch scratch;
    const size_t want = packet_samples_998p4(64, 8, bytes.size());
    scratch.reserve(bytes.size(), want);
    std::vector<gr_complex> tx(want);
    size_t n = 0;
    BOOST_REQUIRE(modulate_one(bytes.data(), bytes.size(), cfg, scratch,
                               tx.data(), tx.size(), n));
    const size_t t0 = 9984;
    std::vector<gr_complex> iq(t0 + n + 8192, gr_complex(0.f, 0.f));
    std::copy(tx.begin(), tx.end(), iq.begin() + static_cast<std::ptrdiff_t>(t0));
    auto prof = Qm35825Profile::Default();
    prof.sfd_mode = "4z2";
    gr::uwb::demod::core::DemodScratch dscratch;
    dscratch.reserve(iq.size());
    const auto res = demodulate_one(iq.data(), iq.size(), prof, 1,
                                    static_cast<int64_t>(t0), 0,
                                    load_template(), dscratch);
    BOOST_REQUIRE_MESSAGE(res.status == gr::uwb::demod::DemodStatus::Success,
                          "4z2 demod status=" +
                              std::to_string(static_cast<int>(res.status)));
    BOOST_CHECK(res.payload.fcs_pass);
    BOOST_REQUIRE_EQUAL(res.payload.bytes.size(), bytes.size());
    BOOST_CHECK_EQUAL_COLLECTIONS(res.payload.bytes.begin(),
                                  res.payload.bytes.end(),
                                  bytes.begin(), bytes.end());
}

BOOST_AUTO_TEST_CASE(test_hrp_mod_4z2_sts_matches_radar_length_and_sfd)
{
    const uint8_t psdu22[] = {
        0x47, 0x26, 0x1D, 0xF6, 0x6F, 0x4C, 0x1B, 0xEF, 0x45, 0xC8, 0xF7,
        0x7C, 0xE7, 0x7B, 0xD7, 0xD8, 0xC4, 0xD1, 0x80, 0xFB, 0x12, 0x21
    };
    HrpModConfig cfg;
    cfg.sfd_mode = "4z2";
    cfg.insert_sts = true;
    HrpModScratch scratch;
    scratch.reserve(22, 190912);
    std::vector<gr_complex> tx(190912);
    size_t n = 0;
    BOOST_REQUIRE(modulate_one(psdu22, 22, cfg, scratch, tx.data(), tx.size(),
                               n));
    BOOST_REQUIRE_EQUAL(n, size_t(190912));

    std::vector<gr_complex> golden;
    BOOST_REQUIRE(load_cf32(std::string(kTestdata) + "/uwb_radar/tx_998p4.cf32",
                            golden));
    BOOST_REQUIRE_EQUAL(golden.size(), n);
    const double rel = l2_rel(tx, golden.data(), n);
    BOOST_CHECK_LT(rel, 5e-3);
    double num = 0.0, da = 0.0, db = 0.0;
    for (size_t i = 0; i < n; ++i) {
        num += static_cast<double>(tx[i].real()) * golden[i].real();
        da += static_cast<double>(tx[i].real()) * tx[i].real();
        db += static_cast<double>(golden[i].real()) * golden[i].real();
    }
    BOOST_CHECK_GT(num / (std::sqrt(da * db) + 1e-18), 0.999);

    auto tmpl = load_template();
    const auto seq = gr::uwb::demod::GetSfdSequence("4z2");
    gr::uwb::radar::RadarSfdScratch ss;
    BOOST_REQUIRE(gr::uwb::radar::prepare_sfd_template(
        seq.data(), seq.size(), tmpl.data(), tmpl.size(), ss));
    gr::uwb::radar::RadarSfdResult sr;
    BOOST_REQUIRE(gr::uwb::radar::search_sfd(
        tx.data(), tx.size(), /*predicted=*/65024, /*margin=*/64,
        0.3f, sr, ss));
    BOOST_CHECK(sr.status == gr::uwb::radar::SfdStatus::Ok);
    BOOST_CHECK_EQUAL(sr.sfd_start_sample, int64_t(65024));
}

BOOST_AUTO_TEST_CASE(test_hrp_mod_sync_lengths_ieee_psdu0)
{
    HrpModConfig cfg;
    HrpModScratch scratch;
    scratch.reserve(0, gr::uwb::mod::kMaxHrpTxSamples);
    std::vector<gr_complex> tx(gr::uwb::mod::kMaxHrpTxSamples);
    const size_t lens[] = { 32, 128, 256, 512, 1024, 2048 };
    for (size_t reps : lens) {
        cfg.sync_repetitions = reps;
        const size_t want = packet_samples_998p4(reps, 8, 0);
        size_t n = 0;
        BOOST_REQUIRE_MESSAGE(
            modulate_one(nullptr, 0, cfg, scratch, tx.data(), tx.size(), n),
            "modulate failed for SYNC " + std::to_string(reps));
        BOOST_CHECK_EQUAL(n, want);
    }
}

BOOST_AUTO_TEST_CASE(test_hrp_mod_prefix_cache_preserves_p1)
{
    std::vector<uint8_t> bytes;
    BOOST_REQUIRE(load_u8(golden_dir() + "/stage_payload_bytes.bin", bytes));
    std::vector<gr_complex> window;
    BOOST_REQUIRE(load_cf32(golden_dir() + "/window.cfile", window));
    BOOST_REQUIRE_GE(window.size(), size_t(9984 + 249280));

    HrpModConfig cfg;
    HrpModScratch scratch;
    scratch.reserve(127, 249280);
    std::vector<gr_complex> tx(249280);
    size_t n = 0;
    BOOST_REQUIRE(modulate_one(bytes.data(), bytes.size(), cfg, scratch,
                               tx.data(), tx.size(), n));
    BOOST_REQUIRE_EQUAL(n, size_t(249280));
    BOOST_CHECK_LT(l2_rel(tx, window.data() + 9984, n), 1e-6);

    auto other = gr::uwb::mod::make_random_psdu(8, 99u, true);
    const size_t n_other = packet_samples_998p4(64, 8, other.size());
    std::vector<gr_complex> tx2(n_other);
    size_t n2 = 0;
    BOOST_REQUIRE(modulate_one(other.data(), other.size(), cfg, scratch,
                               tx2.data(), tx2.size(), n2));
    BOOST_REQUIRE_EQUAL(n2, n_other);

    size_t n3 = 0;
    BOOST_REQUIRE(modulate_one(bytes.data(), bytes.size(), cfg, scratch,
                               tx.data(), tx.size(), n3));
    BOOST_REQUIRE_EQUAL(n3, size_t(249280));
    BOOST_CHECK_LT(l2_rel(tx, window.data() + 9984, n3), 1e-6);
}

BOOST_AUTO_TEST_CASE(test_hrp_mod_code10_ieee_fcs)
{
    auto bytes = gr::uwb::mod::make_random_psdu(8, 7u, true);
    HrpModConfig cfg;
    cfg.code_index = 10;
    HrpModScratch scratch;
    const size_t want = packet_samples_998p4(64, 8, bytes.size());
    scratch.reserve(bytes.size(), want);
    std::vector<gr_complex> tx(want);
    size_t n = 0;
    BOOST_REQUIRE(modulate_one(bytes.data(), bytes.size(), cfg, scratch,
                               tx.data(), tx.size(), n));
    auto prof = Qm35825Profile::Default();
    prof.sfd_mode = "ieee";
    prof.code_index = 10;
    std::vector<gr_complex> tmpl;
    BOOST_REQUIRE(load_cf32(std::string(kTestdata) +
                                "/reference_preamble.bin",
                            tmpl));
    // code-10 needs its own SYNC template; first generated symbol, L2-norm.
    tmpl.assign(tx.begin(), tx.begin() + 1016);
    double e = 0.0;
    for (const auto& z : tmpl)
        e += static_cast<double>(z.real()) * z.real() +
             static_cast<double>(z.imag()) * z.imag();
    const float inv = (e > 0.0) ? static_cast<float>(1.0 / std::sqrt(e)) : 1.f;
    for (auto& z : tmpl)
        z *= inv;
    const size_t t0 = 9984;
    std::vector<gr_complex> iq(t0 + n + 8192, gr_complex(0.f, 0.f));
    std::copy(tx.begin(), tx.end(), iq.begin() + static_cast<std::ptrdiff_t>(t0));
    gr::uwb::demod::core::DemodScratch dscratch;
    dscratch.reserve(iq.size());
    const auto res = demodulate_one(iq.data(), iq.size(), prof, 1,
                                    static_cast<int64_t>(t0), 0, tmpl, dscratch);
    BOOST_REQUIRE_MESSAGE(res.status == gr::uwb::demod::DemodStatus::Success,
                          "code10 demod status=" +
                              std::to_string(static_cast<int>(res.status)));
    BOOST_CHECK(res.payload.fcs_pass);
}
