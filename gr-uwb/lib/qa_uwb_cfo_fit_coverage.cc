/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * QA for the CFO refit's measured-peak coverage.
 *
 * `acquire_preamble()` builds the SFD-anchored full-repetition grid with the
 * leading `peak_corr` entries left as zero placeholders: only the tail
 * `[tail_first, total)` carries a measured SYNC matched-filter value.  The CFO
 * refit must consume only measured peaks, because `std::arg(0) == 0` is not a
 * phase observation.
 *
 * Before the fix a 256-SYNC frame with a +50 kHz target CFO was estimated as
 * ~3.9 kHz (216 of 232 fitted peaks were placeholders), the CIR peak collapsed
 * from 0.463 to 0.0127 and the payload failed.  64 SYNC was unaffected because
 * its global skip and tail start coincide at 24.
 */

#include <boost/test/unit_test.hpp>

#include <gnuradio/uwb/uwb_demod_core.h>
#include <gnuradio/uwb/uwb_hrp_mod_core.h>

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdint>
#include <vector>

namespace {

using cf = std::complex<float>;
using gr::uwb::demod::Qm35825Profile;
using gr::uwb::demod::PreamblePurpose;
using gr::uwb::demod::PreambleRxContext;
using gr::uwb::demod::acquire_preamble;
using gr::uwb::demod::core::DemodScratch;
using gr::uwb::demod::sync_cir::SyncCirRequest;

constexpr double kSampleRate = 998.4e6;
constexpr double kTwoPi = 6.283185307179586476925286766559;
constexpr size_t kSps = gr::uwb::demod::kQm35SamplesPerSymbol;
constexpr int64_t kLead = 1997;

struct Frame {
    std::vector<cf> iq;
    std::vector<cf> tmpl;
    size_t reps = 0;
};

Frame make_frame(size_t reps, size_t psdu_bytes)
{
    gr::uwb::mod::HrpModConfig cfg;
    cfg.code_index = 9;
    cfg.sync_repetitions = reps;
    cfg.sfd_mode = "4z2";
    cfg.peak_amplitude = 0.8f;
    const std::vector<uint8_t> psdu =
        gr::uwb::mod::make_random_psdu(psdu_bytes - 2, 4242, true);
    const size_t cap =
        gr::uwb::mod::packet_samples_998p4(reps, 8, psdu.size(), false,
                                           &cfg.pulse) +
        1024;
    gr::uwb::mod::HrpModScratch scratch;
    scratch.reserve(127, cap);
    std::vector<cf> packet(cap);
    size_t len = 0;
    BOOST_REQUIRE(gr::uwb::mod::modulate_one(psdu.data(), psdu.size(), cfg,
                                             scratch, packet.data(), cap, len));
    packet.resize(len);

    Frame f;
    f.reps = reps;
    f.iq.assign(static_cast<size_t>(kLead) + packet.size() + 4096,
                cf(0.f, 0.f));
    std::copy(packet.begin(), packet.end(), f.iq.begin() + kLead);
    f.tmpl.assign(packet.begin(), packet.begin() + kSps);
    return f;
}

void apply_cfo(std::vector<cf>& iq, double cfo_hz)
{
    const double w = kTwoPi * cfo_hz / kSampleRate;
    for (size_t i = 0; i < iq.size(); ++i) {
        const double ph = w * static_cast<double>(i);
        iq[i] *= cf(static_cast<float>(std::cos(ph)),
                    static_cast<float>(std::sin(ph)));
    }
}

struct Estimate {
    bool ok = false;
    double cfo_hz = 0.0;
    size_t peaks_used = 0;
    size_t skipped = 0;
    int64_t fit_first = -1;
    int64_t fit_last = -1;
};

Estimate estimate(const Frame& f)
{
    Qm35825Profile profile = Qm35825Profile::Default();
    profile.code_index = 9;
    profile.preamble_repetitions = f.reps;
    profile.cir_repetitions = f.reps - profile.cir_skip_initial_repetitions;
    profile.timing_coarse_stride = 1;

    SyncCirRequest req;
    req.cir_axis_origin = kLead;
    req.samples_per_symbol = static_cast<double>(kSps);
    req.total_repetitions = f.reps;
    req.skip_initial = profile.cir_skip_initial_repetitions;
    req.max_repetitions = profile.cir_repetitions;
    req.pre = 16;
    req.post = 100;

    DemodScratch scratch;
    scratch.reserve(f.iq.size());
    PreambleRxContext ctx;
    const bool ok = acquire_preamble(f.iq.data(), f.iq.size(), profile, f.tmpl,
                                     kLead, PreamblePurpose::CirOnly, &req,
                                     scratch, ctx);
    Estimate e;
    e.ok = ok && ctx.ok;
    e.cfo_hz = ctx.final_cfo_hz;
    e.peaks_used = ctx.cfo.peaks_used;
    e.skipped = ctx.cfo.skipped_peaks;
    e.fit_first = ctx.cfo.fit_first_peak_sample;
    e.fit_last = ctx.cfo.fit_last_peak_sample;
    return e;
}

} // namespace

BOOST_AUTO_TEST_CASE(cfo_fit_uses_only_measured_peaks_64_sync)
{
    const Frame f = make_frame(64, 32);
    for (const double cfo : { 0.0, 50000.0, -50000.0 }) {
        Frame g = f;
        if (cfo != 0.0)
            apply_cfo(g.iq, cfo);
        const Estimate e = estimate(g);
        BOOST_REQUIRE(e.ok);
        BOOST_TEST_MESSAGE("64 sync cfo=" << cfo << " est=" << e.cfo_hz
                                          << " peaks=" << e.peaks_used);
        BOOST_CHECK_SMALL(e.cfo_hz - cfo, 500.0);
        // 64 SYNC: global skip == tail start == 24, so the fit consumes the
        // 40 measured tail peaks and nothing else.
        BOOST_CHECK_EQUAL(e.peaks_used, size_t(40));
        BOOST_CHECK_EQUAL(e.skipped, size_t(24));
        BOOST_CHECK(e.fit_first >= 0);
        BOOST_CHECK(e.fit_last > e.fit_first);
    }
}

BOOST_AUTO_TEST_CASE(cfo_fit_uses_only_measured_peaks_256_sync)
{
    const Frame f = make_frame(256, 127);
    for (const double cfo : { 0.0, 50000.0, -50000.0 }) {
        Frame g = f;
        if (cfo != 0.0)
            apply_cfo(g.iq, cfo);
        const Estimate e = estimate(g);
        BOOST_REQUIRE(e.ok);
        BOOST_TEST_MESSAGE("256 sync cfo=" << cfo << " est=" << e.cfo_hz
                                           << " peaks=" << e.peaks_used
                                           << " skipped=" << e.skipped);
        // The defect produced ~3.9 kHz here; 1% is far tighter than that and
        // still loose enough for the 40-peak least-squares fit.
        BOOST_CHECK_SMALL(e.cfo_hz - cfo, std::max(500.0, std::abs(cfo) * 0.01));
        // 256 SYNC: tail_first = 216 > global skip 24, so the fit must start at
        // the tail and use exactly the 40 measured repetitions.
        BOOST_CHECK_EQUAL(e.peaks_used, size_t(40));
        BOOST_CHECK_EQUAL(e.skipped, size_t(216));
        BOOST_CHECK(e.fit_first >= 0);
        BOOST_CHECK(e.fit_last > e.fit_first);
    }
}

BOOST_AUTO_TEST_CASE(cfo_fit_rejects_placeholder_phases)
{
    // The fit must never consume more peaks than were measured.  For every
    // supported preamble length the measured tail is `cfo_min_fit`-bounded, so
    // peaks_used can never exceed the tail length.
    for (const size_t reps : { size_t(64), size_t(128), size_t(256) }) {
        const Frame f = make_frame(reps, 32);
        const Estimate e = estimate(f);
        BOOST_REQUIRE(e.ok);
        const size_t tail = std::min(reps, std::max<size_t>(
            Qm35825Profile::Default().cfo_min_fit_repetitions, 40));
        BOOST_TEST_MESSAGE("reps=" << reps << " peaks=" << e.peaks_used
                                   << " tail=" << tail);
        BOOST_CHECK_LE(e.peaks_used, tail);
        BOOST_CHECK_GE(e.peaks_used, size_t(2));
    }
}
