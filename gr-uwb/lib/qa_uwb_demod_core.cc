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

