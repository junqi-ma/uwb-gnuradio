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

#include <cmath>
#include <complex>
#include <cstdint>
#include <fstream>
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
