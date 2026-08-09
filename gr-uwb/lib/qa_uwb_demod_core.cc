/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * QA for the UWB realtime-demodulator core R1 stages (timing + CFO + SFD)
 * against the MATLAB golden reference (开发方案_UWB实时解调.md R1).
 *
 * Golden reference: testdata/realtime_demod_golden/
 *   window.cfile          full signal IQ (64-SYNC code-9, IEEE legacy SFD)
 *   stage_timing.mat      start=9984 period=1016 metric=1.0 peaks=64 (absolute)
 *   stage_cfo.mat         cfo=0 Hz
 * Template: testdata/reference_preamble.bin (code-9 SYNC, 1016 samples)
 *
 * Coordinate convention: 0-based ABSOLUTE sample indices.
 */

#include <boost/test/unit_test.hpp>

#include <gnuradio/gr_complex.h>
#include <gnuradio/uwb/uwb_demod_core.h>
#include <gnuradio/uwb/uwb_phy_profile.h>

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdint>
#include <fstream>
#include <random>
#include <string>
#include <vector>

namespace {

using namespace gr::uwb::demod;

// Load a CF32 (interleaved float I/Q) file into gr_complex.
bool load_cf32(const std::string& path, std::vector<gr_complex>& out)
{
    std::ifstream f(path, std::ios::binary);
    if (!f)
        return false;
    f.seekg(0, std::ios::end);
    const size_t bytes = static_cast<size_t>(f.tellg());
    f.seekg(0, std::ios::beg);
    if (bytes == 0 || bytes % 8 != 0)
        return false;
    out.resize(bytes / 8);
    f.read(reinterpret_cast<char*>(out.data()), static_cast<std::streamsize>(bytes));
    return true;
}

// Path to the golden directory (repo-root/testdata/realtime_demod_golden).
std::string golden_dir()
{
    // QA runs with CWD = gr-uwb/build; golden lives at repo-root/testdata/...
    return "../../../testdata/realtime_demod_golden";
}

// Load a raw float32 (f32) file.
bool load_f32(const std::string& path, std::vector<float>& out)
{
    std::ifstream f(path, std::ios::binary);
    if (!f)
        return false;
    f.seekg(0, std::ios::end);
    const size_t bytes = static_cast<size_t>(f.tellg());
    f.seekg(0, std::ios::beg);
    if (bytes == 0 || bytes % 4 != 0)
        return false;
    out.resize(bytes / 4);
    f.read(reinterpret_cast<char*>(out.data()), static_cast<std::streamsize>(bytes));
    return true;
}

// Load a raw int8 file.
bool load_i8(const std::string& path, std::vector<int8_t>& out)
{
    std::ifstream f(path, std::ios::binary);
    if (!f)
        return false;
    f.seekg(0, std::ios::end);
    const size_t bytes = static_cast<size_t>(f.tellg());
    f.seekg(0, std::ios::beg);
    if (bytes == 0)
        return false;
    out.resize(bytes);
    f.read(reinterpret_cast<char*>(out.data()), static_cast<std::streamsize>(bytes));
    return true;
}

// Load a raw uint8 file.
bool load_u8(const std::string& path, std::vector<uint8_t>& out)
{
    std::ifstream f(path, std::ios::binary);
    if (!f)
        return false;
    f.seekg(0, std::ios::end);
    const size_t bytes = static_cast<size_t>(f.tellg());
    f.seekg(0, std::ios::beg);
    if (bytes == 0)
        return false;
    out.resize(bytes);
    f.read(reinterpret_cast<char*>(out.data()), static_cast<std::streamsize>(bytes));
    return true;
}

// The 1016-sample code-9 SYNC waveform template (reference_preamble.bin).
std::vector<gr_complex> load_reference_template()
{
    std::vector<gr_complex> tmpl;
    load_cf32("../../../testdata/reference_preamble.bin", tmpl);
    return tmpl;
}

} // namespace

BOOST_AUTO_TEST_CASE(test_demod_core_r1_timing_matches_golden)
{
    std::vector<gr_complex> iq, tmpl;
    BOOST_REQUIRE(load_cf32(golden_dir() + "/window.cfile", iq));
    BOOST_REQUIRE(load_cf32("../../../testdata/reference_preamble.bin", tmpl));
    BOOST_REQUIRE_EQUAL(tmpl.size(), size_t(1016));

    core::DemodScratch scratch;
    scratch.reserve(iq.size());
    auto prof = Qm35825Profile::Default();

    TimingResult tr;
    // seed = first SYNC start (9984) as the detector would supply.
    BOOST_REQUIRE(core::stage_timing(iq.data(), iq.size(), prof, tmpl, 9984, tr,
                                     scratch));
    // Absolute coordinate start (golden: 9984).
    BOOST_CHECK_CLOSE(static_cast<double>(tr.preamble_start_sample), 9984.0,
                      0.05);
    // Period (golden: 1016).
    BOOST_CHECK_CLOSE(tr.measured_period, 1016.0, 0.1);
    // Metric (golden: 1.0).
    BOOST_CHECK_GE(tr.metric, 0.95f);
    // Peaks (golden: 64).
    BOOST_CHECK_EQUAL(tr.detected_peaks, size_t(64));
    // First peak is symbol-END = start + L - 1 (golden: 10999).
    if (!tr.peak_samples.empty())
        BOOST_CHECK_CLOSE(static_cast<double>(tr.peak_samples.front()), 10999.0,
                          0.1);
}

BOOST_AUTO_TEST_CASE(test_demod_core_r1_cfo_matches_golden)
{
    std::vector<gr_complex> iq, tmpl;
    BOOST_REQUIRE(load_cf32(golden_dir() + "/window.cfile", iq));
    BOOST_REQUIRE(load_cf32("../../../testdata/reference_preamble.bin", tmpl));

    core::DemodScratch scratch;
    scratch.reserve(iq.size());
    auto prof = Qm35825Profile::Default();

    TimingResult tr;
    BOOST_REQUIRE(core::stage_timing(iq.data(), iq.size(), prof, tmpl, 9984, tr,
                                     scratch));
    CfoResult cf;
    BOOST_REQUIRE(core::stage_cfo(iq.data(), iq.size(), prof, tr, cf, scratch));
    // Golden: 0 Hz.  Allow tight tolerance.
    BOOST_CHECK_LT(std::abs(cf.cfo_hz), 50.0);
}

BOOST_AUTO_TEST_CASE(test_demod_core_r1_sfd_matches_golden)
{
    std::vector<gr_complex> iq, tmpl;
    BOOST_REQUIRE(load_cf32(golden_dir() + "/window.cfile", iq));
    BOOST_REQUIRE(load_cf32("../../../testdata/reference_preamble.bin", tmpl));

    core::DemodScratch scratch;
    scratch.reserve(iq.size());
    auto prof = Qm35825Profile::Default();

    TimingResult tr;
    BOOST_REQUIRE(core::stage_timing(iq.data(), iq.size(), prof, tmpl, 9984, tr,
                                     scratch));

    // Golden uses IEEE legacy SFD (the lrwpan generator's default SFD).
    const auto sfd_seq = GetSfdSequence("ieee");
    SfdResult sr;
    BOOST_REQUIRE(core::stage_sfd(iq.data(), iq.size(), prof, tr, sfd_seq, tmpl,
                                  sr, scratch));
    // SFD begins right after the 64 SYNC symbols: start + 64*1016 = 75008.
    BOOST_CHECK_CLOSE(static_cast<double>(sr.sfd_start_sample), 75008.0, 0.1);
    BOOST_CHECK_GE(sr.metric, 0.95f);
}

BOOST_AUTO_TEST_CASE(test_demod_core_r1_bad_input_fails_cleanly)
{
    std::vector<gr_complex> noise(4000); // too short for a full SYNC
    for (size_t i = 0; i < noise.size(); ++i)
        noise[i] = gr_complex((float)(i % 17) / 17.0f, 0.0f);
    std::vector<gr_complex> tmpl(1016, gr_complex(0.0f, 0.0f));
    core::DemodScratch scratch;
    scratch.reserve(4096);
    auto prof = Qm35825Profile::Default();
    TimingResult tr;
    BOOST_CHECK(!core::stage_timing(noise.data(), noise.size(), prof, tmpl, 0,
                                    tr, scratch));
}

// ---------------------------------------------------------------------------
// R2: CIR estimation + soft-chip generation (MATLAB estimateCirAndSoftChips).
// ---------------------------------------------------------------------------

// Run timing + CFO + CIR/soft chips on `iq` and return the CIR + soft chips.
static bool run_cir_softchips(const std::vector<gr_complex>& iq,
                              core::DemodScratch& scratch,
                              Qm35825Profile& prof,
                              TimingResult& tr,
                              CfoResult& cf,
                              CirResult& cir)
{
    std::vector<gr_complex> tmpl;
    if (!load_cf32("../../../testdata/reference_preamble.bin", tmpl))
        return false;
    if (!core::stage_timing(iq.data(), iq.size(), prof, tmpl, 9984, tr, scratch))
        return false;
    if (!core::stage_cfo(iq.data(), iq.size(), prof, tr, cf, scratch))
        return false;
    std::vector<int8_t> code9(kPreambleCode9.begin(), kPreambleCode9.end());
    return core::stage_cir_softchips(scratch.derotated.data(), iq.size(), prof,
                                     tr, code9, cir, scratch);
}

BOOST_AUTO_TEST_CASE(test_demod_core_r2_cir_matches_golden)
{
    std::vector<gr_complex> iq;
    BOOST_REQUIRE(load_cf32(golden_dir() + "/window.cfile", iq));
    core::DemodScratch scratch;
    scratch.reserve(iq.size());
    auto prof = Qm35825Profile::Default();
    TimingResult tr;
    CfoResult cf;
    CirResult cir;
    BOOST_REQUIRE(run_cir_softchips(iq, scratch, prof, tr, cf, cir));

    // Golden CIR: 38 normalized taps, pre=8, post=30, first path = 9986.
    std::vector<float> gcir;
    BOOST_REQUIRE(load_f32(golden_dir() + "/stage_cir.f32", gcir));
    BOOST_REQUIRE_EQUAL(gcir.size(), size_t(38));
    BOOST_REQUIRE_EQUAL(cir.cir_values.size(), gcir.size());
    BOOST_CHECK_EQUAL(cir.pre_samples, size_t(8));
    BOOST_CHECK_EQUAL(cir.post_samples, size_t(30));
    BOOST_CHECK_EQUAL(cir.first_path_sample, size_t(9986));

    size_t pk = 0;
    float pm = -1.0f;
    for (size_t i = 0; i < cir.cir_values.size(); ++i) {
        const float a = std::abs(cir.cir_values[i]);
        if (a > pm) {
            pm = a;
            pk = i;
        }
    }
    BOOST_CHECK_EQUAL(pk, size_t(10)); // offset +2
    BOOST_CHECK_CLOSE(cir.cir_peak_metric, 0.732856f, 1.0);

    float l2 = 0.0f;
    for (size_t i = 0; i < gcir.size(); ++i) {
        const float d = cir.cir_values[i] - gcir[i];
        l2 += d * d;
    }
    BOOST_CHECK_SMALL(std::sqrt(l2), 0.01f); // actual ~6e-8
}

BOOST_AUTO_TEST_CASE(test_demod_core_r2_softchips_match_golden)
{
    std::vector<gr_complex> iq;
    BOOST_REQUIRE(load_cf32(golden_dir() + "/window.cfile", iq));
    core::DemodScratch scratch;
    scratch.reserve(iq.size());
    auto prof = Qm35825Profile::Default();
    TimingResult tr;
    CfoResult cf;
    CirResult cir;
    BOOST_REQUIRE(run_cir_softchips(iq, scratch, prof, tr, cf, cir));

    // Golden soft-chip stream: 154578 chips, samples_per_chip = 2, first chip
    // at absolute sample 10013 (= start + post - 1).
    std::vector<float> gsoft;
    BOOST_REQUIRE(load_f32(golden_dir() + "/stage_softchips.f32", gsoft));
    BOOST_REQUIRE_EQUAL(gsoft.size(), size_t(154578));
    BOOST_REQUIRE_EQUAL(scratch.soft_chips.size(), gsoft.size());
    BOOST_CHECK_EQUAL(cir.soft_chip_count, size_t(154578));
    BOOST_CHECK_CLOSE(cir.samples_per_chip, 2.0, 0.5);
    BOOST_CHECK_EQUAL(tr.preamble_start_sample + (int64_t)cir.post_samples - 1,
                      int64_t(10013));

    float mx = 0.0f;
    for (size_t i = 0; i < gsoft.size(); ++i)
        mx = std::max(mx, std::abs(scratch.soft_chips[i] - gsoft[i]));
    BOOST_CHECK_LT(mx, 1e-3f); // actual ~6e-7
    // First soft chip is near the normalized max (preamble present).
    BOOST_CHECK_GT(std::abs(scratch.soft_chips[0]), 0.9f);
}

BOOST_AUTO_TEST_CASE(test_demod_core_r2_cfo_compensated_matches_golden)
{
    // Inject +1 kHz CFO into the golden window; after stage_cfo derotation the
    // CIR and soft chips must still match the (CFO-free) golden reference.
    std::vector<gr_complex> iq;
    BOOST_REQUIRE(load_cf32(golden_dir() + "/window.cfile", iq));
    auto prof = Qm35825Profile::Default();
    const double w = 2.0 * M_PI * 1000.0 / prof.sample_rate;
    std::vector<gr_complex> iqc(iq.size());
    for (size_t i = 0; i < iq.size(); ++i) {
        const double ph = w * static_cast<double>(i);
        iqc[i] = iq[i] * gr_complex(static_cast<float>(std::cos(ph)),
                                    static_cast<float>(std::sin(ph)));
    }

    core::DemodScratch scratch;
    scratch.reserve(iq.size());
    TimingResult tr;
    CfoResult cf;
    CirResult cir;
    BOOST_REQUIRE(run_cir_softchips(iqc, scratch, prof, tr, cf, cir));

    // Estimated CFO should be near the injected +1 kHz.
    BOOST_CHECK_CLOSE(cf.cfo_hz, 1000.0, 10.0);

    std::vector<float> gcir;
    BOOST_REQUIRE(load_f32(golden_dir() + "/stage_cir.f32", gcir));
    float l2 = 0.0f;
    for (size_t i = 0; i < gcir.size(); ++i) {
        const float d = cir.cir_values[i] - gcir[i];
        l2 += d * d;
    }
    BOOST_CHECK_SMALL(std::sqrt(l2), 0.05f);

    std::vector<float> gsoft;
    BOOST_REQUIRE(load_f32(golden_dir() + "/stage_softchips.f32", gsoft));
    float mx = 0.0f;
    for (size_t i = 0; i < gsoft.size(); ++i)
        mx = std::max(mx, std::abs(scratch.soft_chips[i] - gsoft[i]));
    BOOST_CHECK_LT(mx, 0.05f);
}

BOOST_AUTO_TEST_CASE(test_demod_core_r2_bad_input_fails_cleanly)
{
    std::vector<gr_complex> noise(4000);
    for (size_t i = 0; i < noise.size(); ++i)
        noise[i] = gr_complex(static_cast<float>(i % 17) / 17.0f, 0.0f);
    core::DemodScratch scratch;
    scratch.reserve(4096);
    auto prof = Qm35825Profile::Default();
    // A too-short / silent buffer must fail cleanly (no CIR / no soft chips).
    std::vector<int8_t> code9(kPreambleCode9.begin(), kPreambleCode9.end());
    TimingResult tr;
    tr.preamble_start_sample = 0;
    tr.measured_period = 1016.0;
    tr.detected_peaks = 1; // too few repetitions
    CirResult cir;
    BOOST_CHECK(!core::stage_cir_softchips(noise.data(), noise.size(), prof, tr,
                                           code9, cir, scratch));
}

// ---------------------------------------------------------------------------
// R3: NS-SFD location + BPRF PHR (locateNsSfd / helperUWBBPRFDemod /
// helperUWBConvDec / helperUWBPHRDecode).
// ---------------------------------------------------------------------------

BOOST_AUTO_TEST_CASE(test_demod_core_r3_ns_sfd_matches_golden)
{
    std::vector<gr_complex> iq;
    BOOST_REQUIRE(load_cf32(golden_dir() + "/window.cfile", iq));
    core::DemodScratch scratch;
    scratch.reserve(iq.size());
    auto prof = Qm35825Profile::Default();
    TimingResult tr;
    CfoResult cf;
    CirResult cir;
    BOOST_REQUIRE(run_cir_softchips(iq, scratch, prof, tr, cf, cir));

    // Golden window uses the IEEE legacy SFD.
    const auto sfd_seq = GetSfdSequence("ieee");
    NsSfdResult ns;
    BOOST_REQUIRE(core::stage_ns_sfd(scratch.soft_chips, prof, sfd_seq,
                                     kQm35ChipsPerSymbol, ns, scratch));
    // Golden: start_chip=32512, end_chip=36575, polarity=1, correlation=1.0.
    BOOST_CHECK_EQUAL(ns.sfd_start_chip, int64_t(32512));
    BOOST_CHECK_EQUAL(ns.sfd_end_chip, int64_t(36575));
    BOOST_CHECK_EQUAL(ns.polarity, 1);
    BOOST_CHECK_GE(ns.metric, 0.95f);
}

BOOST_AUTO_TEST_CASE(test_demod_core_r3_phr_matches_golden)
{
    std::vector<gr_complex> iq;
    BOOST_REQUIRE(load_cf32(golden_dir() + "/window.cfile", iq));
    core::DemodScratch scratch;
    scratch.reserve(iq.size());
    auto prof = Qm35825Profile::Default();
    TimingResult tr;
    CfoResult cf;
    CirResult cir;
    BOOST_REQUIRE(run_cir_softchips(iq, scratch, prof, tr, cf, cir));

    const auto sfd_seq = GetSfdSequence("ieee");
    NsSfdResult ns;
    BOOST_REQUIRE(core::stage_ns_sfd(scratch.soft_chips, prof, sfd_seq,
                                     kQm35ChipsPerSymbol, ns, scratch));
    PhrResult phr;
    BOOST_REQUIRE(core::stage_phr(scratch.soft_chips, prof, ns,
                                  kQm35ChipsPerSymbol, phr, scratch));
    // Golden: psdu_length=127, secded_pass=1, data rate 6.81.
    BOOST_CHECK_EQUAL(phr.psdu_length, uint32_t(127));
    BOOST_CHECK(!phr.secded_uncorrectable);
    BOOST_CHECK_CLOSE(phr.data_rate_mbps, 6.81f, 0.5);
    // Decoded 19 PHR bits match golden (stage_phr_decoded.bin).
    std::vector<int8_t> gdec;
    BOOST_REQUIRE(load_i8(golden_dir() + "/stage_phr_decoded.bin", gdec));
    BOOST_REQUIRE_EQUAL(gdec.size(), size_t(19));
    BOOST_REQUIRE_EQUAL(phr.phr_bits.size(), gdec.size());
    for (size_t i = 0; i < 19; ++i)
        BOOST_CHECK_EQUAL(phr.phr_bits[i], static_cast<uint8_t>(gdec[i]));
}

BOOST_AUTO_TEST_CASE(test_demod_core_r3_full_pipeline_reaches_phr)
{
    // Run the whole demodulate_one chain on the golden window with the SFD
    // switched to the IEEE legacy SFD the window was generated with.
    std::vector<gr_complex> iq, tmpl;
    BOOST_REQUIRE(load_cf32(golden_dir() + "/window.cfile", iq));
    BOOST_REQUIRE(load_cf32("../../../testdata/reference_preamble.bin", tmpl));
    core::DemodScratch scratch;
    scratch.reserve(iq.size());
    auto prof = Qm35825Profile::Default();
    prof.sfd_mode = "ieee";
    auto res = core::demodulate_one(iq.data(), iq.size(), prof, 1, 9984, 0,
                                    tmpl, scratch);
    BOOST_CHECK(res.status == DemodStatus::Success);
    BOOST_CHECK_EQUAL(res.ns_sfd.sfd_start_chip, int64_t(32512));
    BOOST_CHECK_EQUAL(res.phr.psdu_length, uint32_t(127));
    BOOST_CHECK(!res.phr.secded_uncorrectable);
}

BOOST_AUTO_TEST_CASE(test_demod_core_r3_bad_input_fails_cleanly)
{
    std::vector<float> short_soft(5000, 0.0f); // no SFD / no PHR
    auto prof = Qm35825Profile::Default();
    core::DemodScratch scratch;
    scratch.reserve(8192);
    const auto sfd_seq = GetSfdSequence("ieee");
    NsSfdResult ns;
    BOOST_CHECK(!core::stage_ns_sfd(short_soft, prof, sfd_seq,
                                    kQm35ChipsPerSymbol, ns, scratch));
    // A fabricated NS-SFD beyond the buffer must fail the PHR stage.
    NsSfdResult bad_ns;
    bad_ns.ok = true;
    bad_ns.sfd_end_chip = 100000;
    bad_ns.polarity = 1;
    PhrResult phr;
    BOOST_CHECK(!core::stage_phr(short_soft, prof, bad_ns, kQm35ChipsPerSymbol,
                                 phr, scratch));
}

// ---------------------------------------------------------------------------
// R4: payload BPM-BPSK + RS + FCS (helperUWBPayloadDecode / hrpRS / CRC16).
// ---------------------------------------------------------------------------

BOOST_AUTO_TEST_CASE(test_demod_core_r4_crc16_matches_golden)
{
    // IEEE 802.15.4 CRC-16 (reflected 0x8408): golden FCS of the 125 data
    // bytes = 0x584b.
    std::vector<uint8_t> data;
    BOOST_REQUIRE(load_u8(golden_dir() + "/stage_payload_bytes.bin", data));
    BOOST_REQUIRE_EQUAL(data.size(), size_t(127));
    const uint16_t crc = core::detail::crc16_802154(data.data(), 125);
    BOOST_CHECK_EQUAL(crc, uint16_t(0x584b));
    // The received FCS in the last two bytes (little-endian) must match.
    const uint16_t recv = static_cast<uint16_t>(data[125]) |
                          (static_cast<uint16_t>(data[126]) << 8);
    BOOST_CHECK_EQUAL(recv, uint16_t(0x584b));
}

BOOST_AUTO_TEST_CASE(test_demod_core_r4_payload_fcs_matches_golden)
{
    std::vector<gr_complex> iq;
    BOOST_REQUIRE(load_cf32(golden_dir() + "/window.cfile", iq));
    core::DemodScratch scratch;
    scratch.reserve(iq.size());
    auto prof = Qm35825Profile::Default();
    prof.sfd_mode = "ieee";
    const auto res = core::demodulate_one(iq.data(), iq.size(), prof, 1, 9984, 0,
                                          load_reference_template(), scratch);
    BOOST_REQUIRE(res.status == DemodStatus::Success);
    BOOST_REQUIRE(res.phr.ok);
    BOOST_REQUIRE(res.payload.ok);
    // Golden: 127 payload bytes, FCS pass, received == calculated == 0x584b.
    std::vector<uint8_t> gbytes;
    BOOST_REQUIRE(load_u8(golden_dir() + "/stage_payload_bytes.bin", gbytes));
    BOOST_REQUIRE_EQUAL(res.payload.bytes.size(), gbytes.size());
    for (size_t i = 0; i < gbytes.size(); ++i)
        BOOST_CHECK_EQUAL(res.payload.bytes[i], gbytes[i]);
    BOOST_CHECK(res.payload.fcs_pass);
    BOOST_CHECK_EQUAL(res.payload.received_fcs, uint16_t(0x584b));
    BOOST_CHECK_EQUAL(res.payload.calculated_fcs, uint16_t(0x584b));
}

BOOST_AUTO_TEST_CASE(test_demod_core_r4_bad_input_fails_cleanly)
{
    std::vector<float> short_soft(5000, 0.0f);
    auto prof = Qm35825Profile::Default();
    core::DemodScratch scratch;
    scratch.reserve(8192);
    PhrResult phr;
    phr.ok = true;
    phr.psdu_length = 127;
    NsSfdResult ns;
    ns.ok = true;
    ns.sfd_end_chip = 100000; // PHR end beyond the buffer
    PayloadResult pay;
    BOOST_CHECK(!core::stage_payload_fcs(short_soft, prof, phr, ns,
                                         kQm35ChipsPerSymbol, pay, scratch));
}

// ---------------------------------------------------------------------------
// P0: CFO sweep — estimate + full demod across 0/±1/±5/±10/±25/±50 kHz.
// The full chain (timing → cfo → sfd(derotated) → cir → ns_sfd → phr →
// payload) must still decode the golden payload at every CFO.
// ---------------------------------------------------------------------------

namespace {

// Inject a carrier rotation of +f Hz (e^{+j2π f n/fs}) into the golden window
// and run the full demod chain (IEEE SFD, seeded at the golden preamble).
DemodResult demod_window_with_cfo(const std::vector<gr_complex>& iq, double f_hz)
{
    auto prof = Qm35825Profile::Default();
    prof.sfd_mode = "ieee";
    const double w = 2.0 * M_PI * f_hz / prof.sample_rate;
    std::vector<gr_complex> iqc(iq.size());
    for (size_t i = 0; i < iq.size(); ++i) {
        const double ph = w * static_cast<double>(i);
        iqc[i] = iq[i] * gr_complex(static_cast<float>(std::cos(ph)),
                                    static_cast<float>(std::sin(ph)));
    }
    core::DemodScratch scratch;
    scratch.reserve(iq.size());
    auto tmpl = load_reference_template();
    return core::demodulate_one(iqc.data(), iqc.size(), prof, 1, 9984, 0, tmpl,
                                scratch);
}

} // namespace

BOOST_AUTO_TEST_CASE(test_demod_core_p0_cfo_sweep_clean)
{
    std::vector<gr_complex> iq;
    BOOST_REQUIRE(load_cf32(golden_dir() + "/window.cfile", iq));
    const double kfreqs[] = { -50000.0, -25000.0, -10000.0, -5000.0, -1000.0,
                              0.0,      1000.0,   5000.0,   10000.0, 25000.0,
                              50000.0 };
    for (double f : kfreqs) {
        auto res = demod_window_with_cfo(iq, f);
        BOOST_TEST_MESSAGE("CFO sweep f=" << f << " est=" << res.cfo.cfo_hz
                                          << " status=" << (int)res.status
                                          << " sfd_metric=" << res.sfd.metric
                                          << " fcs=" << res.payload.fcs_pass);
        // CFO estimate within 5% + 50 Hz of the injected value.
        if (f == 0.0)
            BOOST_CHECK_LT(std::abs(res.cfo.cfo_hz), 50.0);
        else
            BOOST_CHECK_CLOSE(res.cfo.cfo_hz, f, 5.0);
        // The full chain must still decode the golden payload.
        BOOST_CHECK(res.status == DemodStatus::Success);
        BOOST_CHECK(res.payload.fcs_pass);
        BOOST_CHECK_EQUAL(res.payload.received_fcs, uint16_t(0x584b));
        // SFD position must be stable across the sweep (golden: 75008).
        BOOST_CHECK_CLOSE(static_cast<double>(res.sfd.sfd_start_sample), 75008.0,
                          0.2);
    }
}

BOOST_AUTO_TEST_CASE(test_demod_core_p0_cfo_sweep_awgn)
{
    // Same sweep on a 20 dB-SNR AWGN version of the golden window: CFO
    // estimate must stay accurate and the payload must still decode.
    std::vector<gr_complex> iq;
    BOOST_REQUIRE(load_cf32(golden_dir() + "/window.cfile", iq));
    const double snr_db = 20.0;
    double sp = 0.0;
    for (size_t i = 0; i < iq.size(); ++i)
        sp += static_cast<double>(std::norm(iq[i]));
    sp /= static_cast<double>(iq.size());
    const double sigma =
        std::sqrt(sp / std::pow(10.0, snr_db / 10.0) / 2.0);
    std::mt19937 rng(12345);
    std::normal_distribution<float> g(0.0f, static_cast<float>(sigma));
    std::vector<gr_complex> iqn(iq.size());
    for (size_t i = 0; i < iq.size(); ++i)
        iqn[i] = iq[i] + gr_complex(g(rng), g(rng));

    const double kfreqs[] = { -10000.0, 0.0, 10000.0 };
    for (double f : kfreqs) {
        auto res = demod_window_with_cfo(iqn, f);
        BOOST_TEST_MESSAGE("CFO sweep AWGN f=" << f
                                               << " est=" << res.cfo.cfo_hz
                                               << " status=" << (int)res.status
                                               << " fcs=" << res.payload.fcs_pass);
        // Wider tolerance under noise: 10% + 200 Hz.
        if (f == 0.0)
            BOOST_CHECK_LT(std::abs(res.cfo.cfo_hz), 200.0);
        else
            BOOST_CHECK_CLOSE(res.cfo.cfo_hz, f, 10.0);
        BOOST_CHECK(res.status == DemodStatus::Success);
        BOOST_CHECK(res.payload.fcs_pass);
        BOOST_CHECK_EQUAL(res.payload.received_fcs, uint16_t(0x584b));
    }
}

// ---------------------------------------------------------------------------
// P3: SFD ROI compression robustness — the search center adapts to the last
// tracked SYNC + measured_period with a narrow ±half-width window, so SFD must
// still be located under timing offset, SFO, AWGN and a multipath echo.
// ---------------------------------------------------------------------------

BOOST_AUTO_TEST_CASE(test_demod_core_p3_sfd_offset)
{
    // Shift the whole golden window right by +7 samples; through the full chain
    // the SFD must be found at 75008 + 7.
    std::vector<gr_complex> iq;
    BOOST_REQUIRE(load_cf32(golden_dir() + "/window.cfile", iq));
    auto prof = Qm35825Profile::Default();
    prof.sfd_mode = "ieee";
    std::vector<gr_complex> iq2(iq.size() + 7, gr_complex(0.0f, 0.0f));
    std::copy(iq.begin(), iq.end(), iq2.begin() + 7);
    core::DemodScratch scratch;
    scratch.reserve(iq2.size());
    auto tmpl = load_reference_template();
    auto res = core::demodulate_one(iq2.data(), iq2.size(), prof, 1, 9984 + 7, 0,
                                    tmpl, scratch);
    BOOST_CHECK(res.status == DemodStatus::Success);
    BOOST_CHECK_CLOSE(static_cast<double>(res.sfd.sfd_start_sample), 75015.0,
                      0.1);
}

BOOST_AUTO_TEST_CASE(test_demod_core_p3_sfd_sfo_adaptation)
{
    // measured_period 200 ppm high (1016.2): the search center uses
    // last_start + round(measured_period), still inside the ±16-sample window.
    std::vector<gr_complex> iq, tmpl;
    BOOST_REQUIRE(load_cf32(golden_dir() + "/window.cfile", iq));
    BOOST_REQUIRE(load_cf32("../../../testdata/reference_preamble.bin", tmpl));
    core::DemodScratch scratch;
    scratch.reserve(iq.size());
    auto prof = Qm35825Profile::Default();
    TimingResult tr;
    BOOST_REQUIRE(core::stage_timing(iq.data(), iq.size(), prof, tmpl, 9984, tr,
                                     scratch));
    tr.measured_period = 1016.0 * (1.0 + 200e-6); // ~13-sample drift over 64 SYNCs
    const auto sfd_seq = GetSfdSequence("ieee");
    SfdResult sr;
    const std::complex<float>* sfd_rx =
        scratch.derotated.empty() ? iq.data() : scratch.derotated.data();
    BOOST_REQUIRE(core::stage_sfd(sfd_rx, iq.size(), prof, tr, sfd_seq, tmpl, sr,
                                  scratch));
    BOOST_CHECK_CLOSE(static_cast<double>(sr.sfd_start_sample), 75008.0, 0.5);
    BOOST_CHECK_GE(sr.metric, 0.9f);
}

BOOST_AUTO_TEST_CASE(test_demod_core_p3_sfd_awgn)
{
    std::vector<gr_complex> iq;
    BOOST_REQUIRE(load_cf32(golden_dir() + "/window.cfile", iq));
    const double snr_db = 20.0;
    double sp = 0.0;
    for (size_t i = 0; i < iq.size(); ++i)
        sp += static_cast<double>(std::norm(iq[i]));
    sp /= static_cast<double>(iq.size());
    const double sigma = std::sqrt(sp / std::pow(10.0, snr_db / 10.0) / 2.0);
    std::mt19937 rng(7);
    std::normal_distribution<float> g(0.0f, static_cast<float>(sigma));
    std::vector<gr_complex> iqn(iq.size());
    for (size_t i = 0; i < iq.size(); ++i)
        iqn[i] = iq[i] + gr_complex(g(rng), g(rng));

    core::DemodScratch scratch;
    scratch.reserve(iqn.size());
    auto prof = Qm35825Profile::Default();
    TimingResult tr;
    SfdResult sr;
    std::vector<gr_complex> tmpl;
    BOOST_REQUIRE(load_cf32("../../../testdata/reference_preamble.bin", tmpl));
    BOOST_REQUIRE(core::stage_timing(iqn.data(), iqn.size(), prof, tmpl, 9984,
                                     tr, scratch));
    const auto sfd_seq = GetSfdSequence("ieee");
    const std::complex<float>* sfd_rx =
        scratch.derotated.empty() ? iqn.data() : scratch.derotated.data();
    BOOST_REQUIRE(core::stage_sfd(sfd_rx, iqn.size(), prof, tr, sfd_seq, tmpl,
                                  sr, scratch));
    BOOST_CHECK_CLOSE(static_cast<double>(sr.sfd_start_sample), 75008.0, 2.0);
    BOOST_CHECK_GE(sr.metric, 0.5f);
}

BOOST_AUTO_TEST_CASE(test_demod_core_p3_sfd_multipath)
{
    // First path + a 0.4x echo delayed by 150 samples (≈1.5 chips): the SFD
    // must still lock onto the first-path peak.
    std::vector<gr_complex> iq;
    BOOST_REQUIRE(load_cf32(golden_dir() + "/window.cfile", iq));
    std::vector<gr_complex> m(iq.size() + 150, gr_complex(0.0f, 0.0f));
    std::copy(iq.begin(), iq.end(), m.begin());
    for (size_t i = 0; i + 150 < m.size() && i < iq.size(); ++i)
        m[i + 150] += 0.4f * iq[i];

    core::DemodScratch scratch;
    scratch.reserve(m.size());
    auto prof = Qm35825Profile::Default();
    TimingResult tr;
    SfdResult sr;
    std::vector<gr_complex> tmpl;
    BOOST_REQUIRE(load_cf32("../../../testdata/reference_preamble.bin", tmpl));
    BOOST_REQUIRE(core::stage_timing(m.data(), m.size(), prof, tmpl, 9984, tr,
                                     scratch));
    const auto sfd_seq = GetSfdSequence("ieee");
    const std::complex<float>* sfd_rx =
        scratch.derotated.empty() ? m.data() : scratch.derotated.data();
    BOOST_REQUIRE(core::stage_sfd(sfd_rx, m.size(), prof, tr, sfd_seq, tmpl, sr,
                                  scratch));
    BOOST_CHECK_CLOSE(static_cast<double>(sr.sfd_start_sample), 75008.0, 2.0);
    BOOST_CHECK_GE(sr.metric, 0.5f);
}

// ---------------------------------------------------------------------------
// P2: CIR soft-chip FIR SIMD — kernel consistency vs multi-acc reference,
// golden soft-chip / CIR tolerances, and a small microbench of the three
// candidates (a=MultiAcc8, b=VOLK, c=AVX2 fixed-38).
// ---------------------------------------------------------------------------

#include <gnuradio/uwb/uwb_cir_fir_simd.h>
#include <chrono>
#include <iostream>

namespace {

// Serial scalar reference: sum_q conj(values[q]) * rx[win+q]  (forward form).
std::complex<float> cir_fir_serial_ref(const std::complex<float>* rx_win,
                                       const std::complex<float>* values,
                                       size_t tap_count)
{
    std::complex<float> acc(0.0f, 0.0f);
    for (size_t q = 0; q < tap_count; ++q)
        acc += std::conj(values[q]) * rx_win[q];
    return acc;
}

} // namespace

BOOST_AUTO_TEST_CASE(test_demod_core_p2_cir_fir_kernels_agree)
{
    // Synthetic random taps + window: all three kernels must match the serial
    // forward-form reference within a tight abs tolerance (FP reassociation).
    constexpr size_t T = 38;
    std::mt19937 rng(0xC1F12u);
    std::uniform_real_distribution<float> dist(-1.0f, 1.0f);
    std::vector<std::complex<float>> values(T), rx(T + 8);
    for (size_t i = 0; i < T; ++i)
        values[i] = { dist(rng), dist(rng) };
    for (size_t i = 0; i < rx.size(); ++i)
        rx[i] = { dist(rng), dist(rng) };

    alignas(32) std::complex<float> h_conj[64];
    alignas(32) std::complex<float> values_nat[64];
    for (size_t q = 0; q < T; ++q) {
        values_nat[q] = values[q];
        h_conj[q] = std::conj(values[q]);
    }

    // Probe several base offsets so unaligned AVX loads are exercised.
    for (size_t off = 0; off < 4; ++off) {
        const std::complex<float>* win = rx.data() + off;
        const auto ref = cir_fir_serial_ref(win, values.data(), T);

        const auto a = cir_fir::dot_multi_acc8(win, h_conj, T);
        const auto b = cir_fir::dot_volk(win, values_nat, T);
        const auto c = cir_fir::dot_avx2_fixed38(win, h_conj, T);

        auto check_close = [&](std::complex<float> x, const char* name) {
            const float d = std::abs(x - ref);
            BOOST_CHECK_MESSAGE(d < 1e-4f,
                                name << " vs serial ref abs-diff=" << d
                                     << " off=" << off);
        };
        check_close(a, "MultiAcc8");
        check_close(b, "VOLK");
        check_close(c, "Avx2Fixed38");
    }
}

BOOST_AUTO_TEST_CASE(test_demod_core_p2_cir_softchips_still_golden)
{
    // Full CIR stage must keep the existing golden tolerances after the FIR
    // rewrite (L2 CIR < 0.01, soft max-diff < 1e-3, first_path / peak / count).
    std::vector<gr_complex> iq;
    BOOST_REQUIRE(load_cf32(golden_dir() + "/window.cfile", iq));
    BOOST_REQUIRE_EQUAL(iq.size(), size_t(319168));

    core::DemodScratch scratch;
    scratch.reserve(iq.size());
    auto prof = Qm35825Profile::Default();
    TimingResult tr;
    CfoResult cf;
    CirResult cir;
    BOOST_REQUIRE(run_cir_softchips(iq, scratch, prof, tr, cf, cir));

    BOOST_CHECK_EQUAL(cir.first_path_sample, size_t(9986));
    BOOST_CHECK_CLOSE(cir.cir_peak_metric, 0.732856f, 1.0);
    BOOST_CHECK_EQUAL(cir.soft_chip_count, size_t(154578));
    BOOST_CHECK_CLOSE(cir.samples_per_chip, 2.0, 0.5);
    BOOST_CHECK_EQUAL(tr.preamble_start_sample + (int64_t)cir.post_samples - 1,
                      int64_t(10013));

    std::vector<float> gcir;
    BOOST_REQUIRE(load_f32(golden_dir() + "/stage_cir.f32", gcir));
    float l2 = 0.0f;
    for (size_t i = 0; i < gcir.size(); ++i) {
        const float d = cir.cir_values[i] - gcir[i];
        l2 += d * d;
    }
    BOOST_CHECK_SMALL(std::sqrt(l2), 0.01f);

    std::vector<float> gsoft;
    BOOST_REQUIRE(load_f32(golden_dir() + "/stage_softchips.f32", gsoft));
    BOOST_REQUIRE_EQUAL(scratch.soft_chips.size(), gsoft.size());
    float mx = 0.0f;
    for (size_t i = 0; i < gsoft.size(); ++i)
        mx = std::max(mx, std::abs(scratch.soft_chips[i] - gsoft[i]));
    BOOST_CHECK_LT(mx, 1e-3f);

    // Full pipeline still yields PHR/payload FCS 0x584b.
    auto prof2 = Qm35825Profile::Default();
    prof2.sfd_mode = "ieee";
    core::DemodScratch scratch2;
    scratch2.reserve(iq.size());
    const auto res = core::demodulate_one(iq.data(), iq.size(), prof2, 1, 9984,
                                          0, load_reference_template(),
                                          scratch2);
    BOOST_REQUIRE(res.status == DemodStatus::Success);
    BOOST_CHECK_EQUAL(res.payload.received_fcs, uint16_t(0x584b));
    BOOST_CHECK_EQUAL(res.payload.calculated_fcs, uint16_t(0x584b));
    BOOST_CHECK(res.payload.fcs_pass);
}

BOOST_AUTO_TEST_CASE(test_demod_core_p2_cir_fir_microbench)
{
    // Microbench the three FIR candidates over the golden chip grid using the
    // real CIR taps.  Prints wall µs so the P2 report can compare a/b/c
    // without touching benchmark_detector.cc.
    std::vector<gr_complex> iq;
    BOOST_REQUIRE(load_cf32(golden_dir() + "/window.cfile", iq));
    core::DemodScratch scratch;
    scratch.reserve(iq.size());
    auto prof = Qm35825Profile::Default();
    TimingResult tr;
    CfoResult cf;
    CirResult cir;
    BOOST_REQUIRE(run_cir_softchips(iq, scratch, prof, tr, cf, cir));
    BOOST_REQUIRE_EQUAL(cir.cir_values.size(), size_t(38));

    // Rebuild complex CIR taps by re-running the estimate portion is heavy;
    // soft chips already passed golden, so for the microbench re-derive h
    // from the real-valued CIR taps (Q≈0 golden channel) as complex.
    constexpr size_t T = 38;
    alignas(32) std::complex<float> values_nat[64];
    alignas(32) std::complex<float> h_conj[64];
    for (size_t q = 0; q < T; ++q) {
        values_nat[q] = std::complex<float>(cir.cir_values[q], 0.0f);
        h_conj[q] = std::conj(values_nat[q]);
    }

    // Use the CFO-compensated buffer the stage already filled.
    const std::complex<float>* rx = scratch.derotated.data();
    const size_t n = iq.size();
    const int64_t chip_start =
        tr.preamble_start_sample + static_cast<int64_t>(cir.post_samples) - 1;
    const int64_t spc_i = 2;
    const int64_t fwd = static_cast<int64_t>(T) - 1;
    const size_t num_chips = cir.soft_chip_count;
    BOOST_REQUIRE(chip_start >= fwd);
    BOOST_REQUIRE(static_cast<size_t>(chip_start + (int64_t)(num_chips - 1) * spc_i) <
                  n);

    auto time_kernel = [&](cir_fir::Kernel k, const char* name) {
        std::vector<std::complex<float>> out(num_chips);
        // Warmup
        for (size_t i = 0; i < num_chips; i += 1024) {
            const size_t win =
                static_cast<size_t>(chip_start + (int64_t)i * spc_i - fwd);
            out[i] = cir_fir::dot(k, rx + win, h_conj, values_nat, T);
        }
        auto t0 = std::chrono::steady_clock::now();
        for (size_t i = 0; i < num_chips; ++i) {
            const size_t win =
                static_cast<size_t>(chip_start + (int64_t)i * spc_i - fwd);
            out[i] = cir_fir::dot(k, rx + win, h_conj, values_nat, T);
        }
        auto t1 = std::chrono::steady_clock::now();
        const double us =
            std::chrono::duration<double, std::micro>(t1 - t0).count();
        std::cout << "[P2 FIR microbench] " << name << " chips=" << num_chips
                  << " fir_us=" << us << "  ("
                  << (1e-6 * num_chips / (us * 1e-6)) << " Mchips/s)\n";

        // Spot-check vs serial on a few chips (reassociation ok within 1e-3).
        for (size_t i : { size_t(0), size_t(1000), num_chips / 2,
                          num_chips - 1 }) {
            const size_t win =
                static_cast<size_t>(chip_start + (int64_t)i * spc_i - fwd);
            const auto ref = cir_fir_serial_ref(rx + win, values_nat, T);
            BOOST_CHECK_LT(std::abs(out[i] - ref), 1e-3f);
        }
        return us;
    };

    const double ua = time_kernel(cir_fir::Kernel::MultiAcc8, "a_MultiAcc8");
    const double ub = time_kernel(cir_fir::Kernel::Volk, "b_VOLK");
    const double uc = time_kernel(cir_fir::Kernel::Avx2Fixed, "c_Avx2Fixed38");
    std::cout << "[P2 FIR microbench] summary a=" << ua << " b=" << ub
              << " c=" << uc << " us (default kernel="
              << static_cast<int>(cir_fir::kDefaultKernel) << ")\n";
    BOOST_CHECK_GT(ua, 0.0);
    BOOST_CHECK_GT(ub, 0.0);
    BOOST_CHECK_GT(uc, 0.0);

    // 203776-sample scheduling window: re-run full CIR stage on a truncated
    // view of the golden buffer and report estimate/fir/post (O(num_chips)).
    {
        constexpr size_t n_sched = 203776;
        BOOST_REQUIRE(iq.size() >= n_sched);
        core::DemodScratch scr2;
        scr2.reserve(n_sched);
        // Reuse timing/cfo from the full window where possible, but CIR stage
        // needs derotated of length n_sched: re-run stages on the prefix.
        TimingResult tr2;
        CfoResult cf2;
        CirResult cir2;
        std::vector<gr_complex> prefix(iq.begin(), iq.begin() + (long)n_sched);
        BOOST_REQUIRE(run_cir_softchips(prefix, scr2, prof, tr2, cf2, cir2));
        std::cout << "[P2 CIR window] n=319168 chips=" << cir.soft_chip_count
                  << " estimate=" << cir.cir_estimate_us
                  << " fir=" << cir.soft_fir_us
                  << " post=" << cir.postprocess_us
                  << " total="
                  << (cir.cir_estimate_us + cir.soft_fir_us + cir.postprocess_us)
                  << " us\n";
        std::cout << "[P2 CIR window] n=" << n_sched
                  << " chips=" << cir2.soft_chip_count
                  << " estimate=" << cir2.cir_estimate_us
                  << " fir=" << cir2.soft_fir_us
                  << " post=" << cir2.postprocess_us
                  << " total="
                  << (cir2.cir_estimate_us + cir2.soft_fir_us +
                      cir2.postprocess_us)
                  << " us\n";
        BOOST_CHECK_LT(cir2.soft_chip_count, cir.soft_chip_count);
    }
}
