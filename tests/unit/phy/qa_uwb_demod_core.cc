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

#include <gnuradio/uwb/uwb_gr_complex.h>
#include <gnuradio/uwb/uwb_demod_core.h>
#include <gnuradio/uwb/uwb_phy_profile.h>

#include <algorithm>
#include <limits>
#include <cmath>
#include <complex>
#include <cstdint>
#include <fstream>
#include <random>
#include <string>
#include <vector>

namespace {

using namespace openuwb::demod;

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

// ---------------------------------------------------------------------------
// Integer chip-grid admission (fused CMF / AVX2 soft-chip kernel).
//
// The grid tolerance is derived from the worst-case payload drift, not from an
// arbitrary epsilon.  These cases pin the two ends that matter on real
// hardware: a 27 ppm clock must still be rejected, a sub-ppm clock accepted,
// and the boundary must sit at the documented drift budget.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(integer_chip_grid_tolerance_matches_the_drift_budget)
{
    using namespace openuwb::demod::core;

    // Default admission is 1.5 ppm = 0.23 samples of drift over a 127 B
    // payload.  The hard ceiling is 1 sample = 6.468 ppm.
    BOOST_CHECK_CLOSE(kIntegerGridMaxClockPpm, 1.5, 1e-9);
    BOOST_CHECK_CLOSE(kIntegerGridCeilingPpm, 6.468, 0.1);
    BOOST_CHECK_CLOSE(kIntegerGridRelTolerance, 1.5e-6, 1e-9);
    BOOST_CHECK_CLOSE(kIntegerGridMaxDriftSamples, 0.232, 0.5);

    // Exactly 2 samples/chip is always accepted.
    BOOST_CHECK(integer_chip_grid(2.0));
    BOOST_CHECK(integer_chip_grid_at(2.0, 2));

    // 0.66 ppm (the best clock measured on hardware) -> 0.10 samples of
    // drift -> accepted with 2.3x margin.
    const double spc_066 = 2.0 * (1.0 - 0.66e-6);
    BOOST_CHECK(integer_chip_grid(spc_066));
    BOOST_CHECK(integer_chip_grid_at(spc_066, 2));

    // 1.24 ppm (an earlier trim step) -> 0.19 samples -> still accepted.
    BOOST_CHECK(integer_chip_grid_at(2.0 * (1.0 - 1.24e-6), 2));

    // Just inside / just outside the default gate.
    BOOST_CHECK(integer_chip_grid_at(2.0 * (1.0 - 1.4e-6), 2));
    BOOST_CHECK(!integer_chip_grid_at(2.0 * (1.0 - 1.6e-6), 2));

    // 27 ppm (the untrimmed clock) -> 4.2 samples of drift -> rejected at
    // every tolerance up to the physical ceiling.
    BOOST_CHECK(!integer_chip_grid_at(2.0 * (1.0 - 27.0e-6), 2));
    BOOST_CHECK(!integer_chip_grid(2.0 * (1.0 - 27.0e-6)));
    BOOST_CHECK(!integer_chip_grid_at(2.0 * (1.0 - 27.0e-6), 2,
                                      kIntegerGridCeilingPpm * 1e-6));

    // An explicit tolerance is honoured: the physical ceiling admits a 6 ppm
    // clock that the default gate refuses.
    const double spc_6 = 2.0 * (1.0 - 6.0e-6);
    BOOST_CHECK(!integer_chip_grid(spc_6));
    BOOST_CHECK(integer_chip_grid(spc_6, kIntegerGridCeilingPpm * 1e-6));

    // Degenerate inputs fail closed.
    BOOST_CHECK(!integer_chip_grid(0.0));
    BOOST_CHECK(!integer_chip_grid(-2.0));
    BOOST_CHECK(!integer_chip_grid_at(2.0, 0));
    BOOST_CHECK(!integer_chip_grid_at(2.0, 2, 0.0));
    BOOST_CHECK(!integer_chip_grid_at(2.0, 2, -1e-6));
    BOOST_CHECK(!integer_chip_grid_at(2.0, 2,
                                      std::numeric_limits<double>::quiet_NaN()));

    // A different integer grid is not silently accepted as 2 samples/chip.
    BOOST_CHECK(!integer_chip_grid_at(1.0, 2));
    BOOST_CHECK(!integer_chip_grid_at(4.0, 2));
    BOOST_CHECK(integer_chip_grid(1.0)); // its own nearest integer, though
}


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

BOOST_AUTO_TEST_CASE(test_demod_core_tail_seed_refine_radius_fallback)
{
    // SFD-anchored tail profile: the first tail SYNC prediction is within a
    // few samples, so the experiment tries +/-8 before the legacy wide search.
    // A miss at +12 samples must fall back and still recover all 40 peaks.
    std::vector<gr_complex> iq, tmpl;
    BOOST_REQUIRE(load_cf32(golden_dir() + "/window.cfile", iq));
    BOOST_REQUIRE(load_cf32("../../../testdata/reference_preamble.bin", tmpl));
    BOOST_REQUIRE_EQUAL(tmpl.size(), size_t(1016));

    auto prof = Qm35825Profile::Default();
    prof.preamble_repetitions = 40;
    prof.timing_search_margin = 0;
    prof.timing_coarse_stride = 1;
    prof.timing_max_backtrack_symbols = 0;
    prof.timing_track_radius = 8;
    prof.radar_verification_threshold = 0.20f;
    prof.timing_seed_refine_radius = 8;
    const int64_t tail_start =
        9984 + 24 * static_cast<int64_t>(kQm35SamplesPerSymbol);

    core::DemodScratch scratch;
    scratch.reserve(iq.size());
    TimingResult narrow;
    BOOST_REQUIRE(core::stage_timing(iq.data(), iq.size(), prof, tmpl,
                                     tail_start, narrow, scratch));
    BOOST_CHECK_EQUAL(narrow.detected_peaks, size_t(40));
    BOOST_CHECK_EQUAL(narrow.seed_refine_narrow_attempts, 1u);
    BOOST_CHECK_EQUAL(narrow.seed_refine_narrow_success, 1u);
    BOOST_CHECK_EQUAL(narrow.seed_refine_fallbacks, 0u);

    TimingResult fallback;
    BOOST_REQUIRE(core::stage_timing(iq.data(), iq.size(), prof, tmpl,
                                     tail_start + 12, fallback, scratch));
    BOOST_CHECK_EQUAL(fallback.detected_peaks, size_t(40));
    BOOST_CHECK_EQUAL(fallback.seed_refine_narrow_attempts, 1u);
    BOOST_CHECK_EQUAL(fallback.seed_refine_narrow_success, 0u);
    BOOST_CHECK_EQUAL(fallback.seed_refine_fallbacks, 1u);

    prof.timing_seed_refine_radius = 0;
    TimingResult legacy;
    BOOST_REQUIRE(core::stage_timing(iq.data(), iq.size(), prof, tmpl,
                                     tail_start, legacy, scratch));
    BOOST_CHECK_EQUAL(legacy.detected_peaks, size_t(40));
    BOOST_CHECK_EQUAL(legacy.seed_refine_narrow_attempts, 0u);
}

BOOST_AUTO_TEST_CASE(test_locked_grid_timing_matches_legacy_train)
{
    // LockedBounded grid tracking replaces the per-SYNC +/-8 search with a
    // sparse anchor fit plus one correlation per SYNC.  On a clean train it
    // must reproduce the legacy peak train and the fitted period, because CFO
    // (peak_corr phases) and CIR (measured_period) are downstream consumers.
    std::vector<gr_complex> iq, tmpl;
    BOOST_REQUIRE(load_cf32(golden_dir() + "/window.cfile", iq));
    BOOST_REQUIRE(load_cf32("../../../testdata/reference_preamble.bin", tmpl));

    auto prof = Qm35825Profile::Default();
    prof.sync_strategy = SyncStrategy::LockedBounded;
    prof.locked_roi_samples = 256;
    prof.locked_inner_symbols = 4;
    prof.locked_allow_backtrack = false;

    core::DemodScratch scratch_a;
    scratch_a.reserve(iq.size());
    prof.locked_grid_timing = false;
    TimingResult legacy;
    BOOST_REQUIRE(core::stage_timing(iq.data(), iq.size(), prof, tmpl, 9984,
                                     legacy, scratch_a));
    BOOST_CHECK_EQUAL(legacy.grid_anchor_hits, 0u);
    BOOST_CHECK_EQUAL(legacy.grid_single_points, 0u);

    core::DemodScratch scratch_b;
    scratch_b.reserve(iq.size());
    prof.locked_grid_timing = true;
    TimingResult grid;
    BOOST_REQUIRE(core::stage_timing(iq.data(), iq.size(), prof, tmpl, 9984,
                                     grid, scratch_b));
    // The grid path ran (no fallback) and used real anchor searches.
    BOOST_CHECK_EQUAL(grid.grid_fallbacks, 0u);
    BOOST_CHECK_GE(grid.grid_anchor_hits, 8u);
    BOOST_CHECK_EQUAL(grid.grid_single_points, uint32_t(prof.preamble_repetitions));
    BOOST_CHECK_EQUAL(grid.grid_single_valid, uint32_t(prof.preamble_repetitions));
    // A clean train never needs the failure-path tail probe.
    BOOST_CHECK_EQUAL(grid.tail_probe_attempts, 0u);
    BOOST_CHECK_EQUAL(grid.tail_probe_hits, 0u);
    // Same train, same origin, same period as the legacy per-SYNC search.
    BOOST_CHECK_EQUAL(grid.detected_peaks, legacy.detected_peaks);
    BOOST_CHECK_EQUAL(grid.detected_peaks, prof.preamble_repetitions);
    BOOST_CHECK_SMALL(std::abs(grid.measured_period - legacy.measured_period),
                      0.02);
    BOOST_CHECK_SMALL(std::abs(static_cast<double>(grid.preamble_start_sample) -
                               static_cast<double>(legacy.preamble_start_sample)),
                      2.0);
    // Locked contract still holds: no backtrack, no legacy scan.
    BOOST_CHECK_EQUAL(grid.backtrack_symbols, 0u);
    BOOST_CHECK_EQUAL(grid.bounded_legacy_scans, 0u);
    BOOST_CHECK_EQUAL(grid.bounded_success, 1u);

    // End to end: the grid path must still decode the golden packet, including
    // the SFD-anchored tail re-track (which runs stage_timing a second time on
    // the 40-SYNC tail with the same bounded profile).
    auto p_legacy = prof;
    p_legacy.locked_grid_timing = false;
    p_legacy.sfd_mode = "ieee";
    auto p_grid = prof;
    p_grid.locked_grid_timing = true;
    p_grid.sfd_mode = "ieee";
    core::DemodScratch s_legacy, s_grid;
    s_legacy.reserve(iq.size());
    s_grid.reserve(iq.size());
    auto r_legacy = core::demodulate_one(iq.data(), iq.size(), p_legacy, 1,
                                         9984, 0, tmpl, s_legacy);
    auto r_grid = core::demodulate_one(iq.data(), iq.size(), p_grid, 1, 9984, 0,
                                       tmpl, s_grid);
    BOOST_CHECK(r_grid.status == DemodStatus::Success);
    BOOST_CHECK(r_legacy.status == r_grid.status);
    BOOST_CHECK(r_grid.payload.bytes == r_legacy.payload.bytes);
    BOOST_CHECK(r_grid.payload.fcs_pass == r_legacy.payload.fcs_pass);
    BOOST_CHECK_EQUAL(r_grid.timing.grid_fallbacks, 0u);
    BOOST_CHECK_SMALL(std::abs(r_grid.timing.measured_period -
                               r_legacy.timing.measured_period), 0.02);
    BOOST_CHECK_SMALL(std::abs(r_grid.cfo.cfo_hz - r_legacy.cfo.cfo_hz), 15.0);
    BOOST_CHECK_SMALL(std::abs(static_cast<double>(r_grid.sfd.sfd_start_sample) -
                               static_cast<double>(r_legacy.sfd.sfd_start_sample)),
                      2.0);
}

BOOST_AUTO_TEST_CASE(test_locked_grid_timing_period_is_measured_not_nominal)
{
    // The grid path must MEASURE the period (anchors bootstrap the grid, then a
    // least-squares fit over every refined peak), never report the nominal 1016
    // grid: measured_period feeds integer_chip_grid()/spc_ppm and the fused-CMF
    // admission, so a degenerate constant would silently disable the
    // Full->Bypass (grid_reject) protection for an untrimmed clock.
    std::vector<gr_complex> iq, tmpl;
    BOOST_REQUIRE(load_cf32(golden_dir() + "/window.cfile", iq));
    BOOST_REQUIRE(load_cf32("../../../testdata/reference_preamble.bin", tmpl));

    // Compress the sample clock by 50 ppm: out[n] = in[n*rate] maps a signal
    // feature at input index m to output index m/rate, so the SYNC period
    // becomes 1016/rate = 1015.949 samples (drift over 63 symbols is 3.2
    // samples, inside the +/-8 anchor radius) and the integer chip grid must
    // be rejected.
    const double ppm = 50.0;
    const double rate = 1.0 + ppm * 1e-6;
    std::vector<gr_complex> stretched;
    stretched.resize(static_cast<size_t>(
        std::floor(static_cast<double>(iq.size() - 1) / rate)));
    for (size_t n = 0; n < stretched.size(); ++n) {
        const double x = static_cast<double>(n) * rate;
        const size_t i0 = static_cast<size_t>(x);
        const double f = x - static_cast<double>(i0);
        const size_t i1 = std::min(i0 + 1, iq.size() - 1);
        stretched[n] = iq[i0] * static_cast<float>(1.0 - f) +
                       iq[i1] * static_cast<float>(f);
    }

    auto prof = Qm35825Profile::Default();
    prof.sync_strategy = SyncStrategy::LockedBounded;
    prof.locked_roi_samples = 256;
    prof.locked_inner_symbols = 4;
    prof.locked_grid_timing = true;

    core::DemodScratch scratch;
    scratch.reserve(stretched.size());
    TimingResult tr;
    BOOST_REQUIRE(core::stage_timing(stretched.data(), stretched.size(), prof,
                                     tmpl, 9984, tr, scratch));
    BOOST_CHECK_EQUAL(tr.grid_fallbacks, 0u);
    BOOST_CHECK_GE(tr.grid_anchor_hits, 8u);
    const double expected = 1016.0 / rate;
    BOOST_CHECK_SMALL(std::abs(tr.measured_period - expected), 0.05);
    // Not the nominal grid.
    BOOST_CHECK_GT(std::abs(tr.measured_period - 1016.0), 0.02);
    // 50 ppm is far outside the integer chip-grid drift budget, so the fused
    // path must still be refused (this is the Full->Bypass protection).
    BOOST_CHECK(!core::integer_chip_grid(tr.measured_period /
                                         double(kQm35ChipsPerSymbol)));
}

BOOST_AUTO_TEST_CASE(test_locked_tail_probe_rescues_leading_interference)
{
    // A burst interferer can cover only the first few SYNCs while the tail is
    // clean (5.2 m capture: first_5_max 0.029..0.053, med_tail 0.14..0.20).
    // The bounded probe must then keep probing sparse tail anchors instead of
    // declaring a tracking failure, and the rescued origin must still be the
    // true first SYNC start.
    std::vector<gr_complex> iq, tmpl;
    BOOST_REQUIRE(load_cf32(golden_dir() + "/window.cfile", iq));
    BOOST_REQUIRE(load_cf32("../../../testdata/reference_preamble.bin", tmpl));
    constexpr size_t kStart = 9984;
    constexpr size_t kSym = kQm35SamplesPerSymbol;
    constexpr size_t kAtten = 20;

    std::vector<gr_complex> hit = iq;
    // Replace the leading SYNCs with uncorrelated noise of similar power: the
    // normalised metric is scale-invariant, so attenuating them would NOT
    // lower it.  Real burst interference decorrelates the SYNC, which is what
    // the 5.2 m capture shows (first_5_max 0.029..0.053).
    std::mt19937 rng(20261116u);
    std::normal_distribution<float> nd(0.0f, 0.35f);
    float rms = 0.0f;
    for (size_t i = 0; i < kSym; ++i)
        rms += std::norm(iq[kStart + i]);
    rms = std::sqrt(rms / static_cast<float>(kSym));
    for (size_t k = 0; k < kAtten; ++k) {
        const size_t base = kStart + k * kSym;
        BOOST_REQUIRE_LT(base + kSym, hit.size());
        for (size_t i = 0; i < kSym; ++i)
            hit[base + i] = gr_complex(nd(rng), nd(rng)) * rms;
    }

    auto prof = Qm35825Profile::Default();
    prof.sync_strategy = SyncStrategy::LockedBounded;
    prof.locked_roi_samples = 256;
    prof.locked_inner_symbols = 4;
    prof.locked_grid_timing = true;

    // Without the tail probe the packet is lost: m = 0..4 are all attenuated.
    core::DemodScratch scratch_a;
    scratch_a.reserve(hit.size());
    prof.locked_tail_probe = false;
    TimingResult no_tail;
    BOOST_CHECK(!core::stage_timing(hit.data(), hit.size(), prof, tmpl, kStart,
                                    no_tail, scratch_a));
    BOOST_CHECK_EQUAL(no_tail.bounded_failures, 1u);
    BOOST_CHECK_EQUAL(no_tail.tail_probe_attempts, 0u);

    // With the tail probe the clean tail rescues the packet.
    core::DemodScratch scratch_b;
    scratch_b.reserve(hit.size());
    prof.locked_tail_probe = true;
    prof.locked_tail_probe_step = 8;
    prof.locked_tail_probe_confirm = 2;
    TimingResult res;
    BOOST_REQUIRE(core::stage_timing(hit.data(), hit.size(), prof, tmpl, kStart,
                                     res, scratch_b));
    BOOST_CHECK_EQUAL(res.bounded_failures, 0u);
    BOOST_CHECK_EQUAL(res.tail_probe_hits, 1u);
    BOOST_CHECK_GT(res.tail_probe_k, int64_t(kAtten));
    BOOST_CHECK_GE(res.grid_anchor_hits, 8u);
    // Attenuated head symbols are below threshold; the clean tail is kept.
    BOOST_CHECK_GE(res.detected_peaks, prof.preamble_repetitions - kAtten);
    BOOST_CHECK_LT(res.detected_peaks, prof.preamble_repetitions);
    // The anchor fit extrapolates back to the true first SYNC start.
    BOOST_CHECK_SMALL(std::abs(static_cast<double>(res.preamble_start_sample) -
                               static_cast<double>(kStart)),
                      4.0);
    BOOST_CHECK_SMALL(std::abs(res.measured_period - 1016.0), 0.05);
    BOOST_CHECK_EQUAL(res.backtrack_symbols, 0u);
    BOOST_CHECK_EQUAL(res.bounded_legacy_scans, 0u);
}

BOOST_AUTO_TEST_CASE(test_demod_core_rxopt_on_demand_rejects_fractional_grid)
{
    // The on-demand experiment must not round a fractional SFO chip grid or
    // change RAKE/bypass semantics; it declines and leaves the full legacy
    // path available.
    std::vector<gr_complex> iq, tmpl;
    BOOST_REQUIRE(load_cf32(golden_dir() + "/window.cfile", iq));
    BOOST_REQUIRE(load_cf32("../../../testdata/reference_preamble.bin", tmpl));
    auto prof = Qm35825Profile::Default();
    core::DemodScratch scratch;
    scratch.reserve(iq.size());
    TimingResult tr;
    BOOST_REQUIRE(core::stage_timing(iq.data(), iq.size(), prof, tmpl, 9984, tr,
                                     scratch));
    TimingResult fractional = tr;
    fractional.measured_period += 0.5;
    const int8_t* pc = GetPreambleCode(prof.code_index);
    std::vector<int8_t> pcode(pc, pc + kQm35CodeLength);
    prof.rx_softchip_on_demand = true;
    CirResult prefix;
    BOOST_CHECK(!core::stage_cir_softchips(
        iq.data(), iq.size(), prof, fractional, pcode, prefix, scratch,
        rxopt::initial_chip_budget(prof), 0));
    BOOST_CHECK(!scratch.softchip.prepared);

    prof.rx_softchip_on_demand = false;
    CirResult full;
    BOOST_REQUIRE(core::stage_cir_softchips(
        iq.data(), iq.size(), prof, fractional, pcode, full, scratch));
    BOOST_CHECK_GT(full.soft_chip_count, 0u);
    BOOST_CHECK_GT(full.samples_per_chip, 0.0);
}

BOOST_AUTO_TEST_CASE(test_demod_core_r1_backtracks_weak_qm35_first_sync)
{
    std::vector<gr_complex> tmpl;
    BOOST_REQUIRE(load_cf32("../../../testdata/reference_preamble.bin", tmpl));
    BOOST_REQUIRE_EQUAL(tmpl.size(), size_t(1016));

    auto prof = Qm35825Profile::Default();
    constexpr int64_t start = 2048;
    const size_t sym = tmpl.size();
    const auto sfd = GetSfdSequence("4z2");
    std::vector<gr_complex> iq(
        static_cast<size_t>(start) + (prof.preamble_repetitions + sfd.size()) * sym,
        gr_complex(0.0f, 0.0f));

    // The real QM35 startup transient can leave the first few SYNCs below the
    // normal 0.3 threshold.  Make SYNC #1/#2 metric 0.25 and seed at strong
    // SYNC #3; timing must lock strong first, then recover both predecessors.
    std::vector<gr_complex> orth(sym);
    for (size_t i = 0; i < sym; ++i)
        orth[i] = gr_complex(std::sin(0.173f * static_cast<float>(i)),
                             std::cos(0.119f * static_cast<float>(i)));
    gr_complex projection(0.0f, 0.0f);
    float et = 0.0f;
    for (size_t i = 0; i < sym; ++i) {
        projection += std::conj(tmpl[i]) * orth[i];
        et += std::norm(tmpl[i]);
    }
    for (size_t i = 0; i < sym; ++i)
        orth[i] -= (projection / et) * tmpl[i];
    float eo = 0.0f;
    for (const auto& v : orth)
        eo += std::norm(v);
    const float orth_scale = std::sqrt(et / eo);
    for (size_t rep = 0; rep < 2; ++rep)
        for (size_t i = 0; i < sym; ++i)
            iq[static_cast<size_t>(start) + rep * sym + i] =
                0.5f * tmpl[i] + std::sqrt(0.75f) * orth_scale * orth[i];
    for (size_t rep = 2; rep < prof.preamble_repetitions; ++rep)
        std::copy(tmpl.begin(), tmpl.end(),
                  iq.begin() + start + static_cast<int64_t>(rep * sym));
    for (size_t k = 0; k < sfd.size(); ++k)
        for (size_t i = 0; i < sym; ++i)
            iq[static_cast<size_t>(start) +
               (prof.preamble_repetitions + k) * sym + i] =
                static_cast<float>(sfd[k]) * tmpl[i];

    core::DemodScratch scratch;
    scratch.reserve(iq.size());
    TimingResult tr;
    BOOST_REQUIRE(core::stage_timing(iq.data(), iq.size(), prof, tmpl,
                                     start + static_cast<int64_t>(2 * sym), tr,
                                     scratch));
    BOOST_CHECK_EQUAL(tr.preamble_start_sample, start);
    BOOST_CHECK_EQUAL(tr.detected_peaks, prof.preamble_repetitions);
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

BOOST_AUTO_TEST_CASE(test_demod_core_r1_cfo_skips_24_startup_syncs)
{
    // Mirror MATLAB compensateCarrierOffset: the first 24 matched-filter
    // phases can be a front-end transient; retain the later 40 clean peaks.
    auto prof = Qm35825Profile::Default();
    constexpr size_t kPeaks = 64;
    constexpr size_t kTransient = 24;
    constexpr double kCfoHz = 1900.0;

    TimingResult tr;
    tr.peak_samples.reserve(kPeaks);
    tr.peak_corr.reserve(kPeaks);
    for (size_t k = 0; k < kPeaks; ++k) {
        const int64_t sample = 4096 + static_cast<int64_t>(k * kQm35SamplesPerSymbol);
        tr.peak_samples.push_back(sample);
        // Deliberately flat/invalid startup phase.  The clean segment starts
        // at zero phase so phase unwrap across the discarded boundary is safe.
        const double phase = k < kTransient
                                 ? 0.0
                                 : 2.0 * M_PI * kCfoHz *
                                       static_cast<double>(sample -
                                                           tr.peak_samples[kTransient]) /
                                       prof.sample_rate;
        tr.peak_corr.emplace_back(static_cast<float>(std::cos(phase)),
                                  static_cast<float>(std::sin(phase)));
    }

    std::vector<gr_complex> iq(
        static_cast<size_t>(tr.peak_samples.back()) + 1, gr_complex(0.0f, 0.0f));
    core::DemodScratch scratch;
    scratch.reserve(iq.size());
    CfoResult cf;
    BOOST_REQUIRE(core::stage_cfo(iq.data(), iq.size(), prof, tr, cf, scratch));
    BOOST_CHECK_EQUAL(cf.peaks_used, kPeaks - kTransient);
    BOOST_CHECK_EQUAL(cf.skipped_peaks, kTransient);
    BOOST_CHECK_EQUAL(cf.fit_first_peak_sample, tr.peak_samples[kTransient]);
    BOOST_CHECK_EQUAL(cf.fit_last_peak_sample, tr.peak_samples.back());
    BOOST_CHECK_CLOSE(cf.cfo_hz, kCfoHz, 0.1);
}

BOOST_AUTO_TEST_CASE(test_demod_core_low_snr_sparse_anchor_cfo)
{
    // The low-SNR fold path reports 16 super-anchors spaced 16 SYNC periods
    // (16 * 1016 samples).  The legacy unwrap gate only accumulated pairs with
    // dt <= 1.5 symbols, so it collected ZERO pairs and the slope silently
    // stayed 0; the LS fit then ran on wrapped phases and happened to work
    // only because |CFO| * 256 symbols stays inside +-2*pi for small offsets.
    // The sparse gate (dt <= 20 symbols) must recover the true slope.
    auto prof = Qm35825Profile::Default();
    constexpr size_t kAnchors = 16;
    constexpr size_t kGroup = 16;
    const int64_t start = 4096;

    for (double cfo : { 1900.0, 4000.0, -2500.0 }) {
        TimingResult tr;
        for (size_t g = 0; g < kAnchors; ++g) {
            const int64_t sample =
                start + static_cast<int64_t>(g * kGroup) *
                            kQm35SamplesPerSymbol;
            tr.peak_samples.push_back(sample);
            const double phase =
                2.0 * M_PI * cfo *
                static_cast<double>(sample - start) / prof.sample_rate;
            tr.peak_corr.emplace_back(static_cast<float>(std::cos(phase)),
                                      static_cast<float>(std::sin(phase)));
            tr.peak_metrics.push_back(0.56f);
        }
        tr.detected_peaks = prof.preamble_repetitions;
        tr.measured_period = static_cast<double>(kQm35SamplesPerSymbol);

        std::vector<gr_complex> iq(
            static_cast<size_t>(tr.peak_samples.back()) + 1,
            gr_complex(0.0f, 0.0f));
        core::DemodScratch scratch;
        scratch.reserve(iq.size());
        CfoResult cf;
        BOOST_REQUIRE(core::fit_cfo(iq.data(), iq.size(), prof, tr, cf,
                                    scratch));
        BOOST_CHECK_EQUAL(cf.peaks_used, kAnchors);
        BOOST_CHECK_CLOSE(cf.cfo_hz, cfo, 0.1);
    }

    // With one missing super-anchor (32-symbol gap) at 18 kHz CFO, the gap
    // rotates by > pi (~3.68 rad), so zero-slope unwrap fails while the
    // dt <= 20*1016 slope estimator recovers the exact 18 kHz frequency.
    {
        constexpr double kGapCfo = 18000.0;
        TimingResult tr;
        for (size_t g = 0; g < kAnchors; ++g) {
            if (g == 5)
                continue;
            const int64_t sample =
                start + static_cast<int64_t>(g * kGroup) *
                            kQm35SamplesPerSymbol;
            tr.peak_samples.push_back(sample);
            const double phase =
                2.0 * M_PI * kGapCfo *
                static_cast<double>(sample - start) / prof.sample_rate;
            tr.peak_corr.emplace_back(static_cast<float>(std::cos(phase)),
                                      static_cast<float>(std::sin(phase)));
            tr.peak_metrics.push_back(0.56f);
        }
        tr.detected_peaks = prof.preamble_repetitions;
        tr.measured_period = static_cast<double>(kQm35SamplesPerSymbol);
        std::vector<gr_complex> iq(
            static_cast<size_t>(tr.peak_samples.back()) + 1,
            gr_complex(0.0f, 0.0f));
        core::DemodScratch scratch;
        scratch.reserve(iq.size());
        CfoResult cf;
        BOOST_REQUIRE(core::fit_cfo(iq.data(), iq.size(), prof, tr, cf,
                                    scratch));
        BOOST_CHECK_EQUAL(cf.peaks_used, kAnchors - 1);
        BOOST_CHECK_CLOSE(cf.cfo_hz, kGapCfo, 0.1);
    }
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
    BOOST_CHECK_CLOSE(static_cast<double>(sr.expected_start_sample), 75008.0, 0.1);
    BOOST_CHECK_CLOSE(static_cast<double>(sr.sfd_start_sample), 75008.0, 0.1);
    BOOST_CHECK_GE(sr.metric, 0.95f);
    BOOST_CHECK_EQUAL(sr.search_windows, 1u);
    BOOST_CHECK_GT(sr.coarse_correlations, 0u);
    BOOST_CHECK_GT(sr.fine_correlations, 0u);
}

BOOST_AUTO_TEST_CASE(test_demod_core_r1_sfd_symmetric_one_symbol_forward)
{
    // MATLAB-style symmetric +-1-symbol search: an SFD one symbol LATER than
    // the nominal position (preamble start one SYNC early) is recovered inside
    // the expected +- samples_per_symbol window.
    std::vector<gr_complex> iq, tmpl;
    BOOST_REQUIRE(load_cf32(golden_dir() + "/window.cfile", iq));
    BOOST_REQUIRE(load_cf32("../../../testdata/reference_preamble.bin", tmpl));

    core::DemodScratch scratch;
    scratch.reserve(iq.size());
    auto prof = Qm35825Profile::Default();

    TimingResult tr;
    BOOST_REQUIRE(core::stage_timing(iq.data(), iq.size(), prof, tmpl, 9984, tr,
                                     scratch));
    for (auto& peak : tr.peak_samples)
        peak -= static_cast<int64_t>(kQm35SamplesPerSymbol);
    tr.preamble_start_sample -= static_cast<int64_t>(kQm35SamplesPerSymbol);

    const auto sfd_seq = GetSfdSequence("ieee");
    SfdResult sr;
    BOOST_REQUIRE(core::stage_sfd(iq.data(), iq.size(), prof, tr, sfd_seq, tmpl,
                                  sr, scratch));
    BOOST_CHECK_EQUAL(sr.sfd_start_sample, int64_t(75008));
    BOOST_CHECK_GE(sr.metric, 0.95f);
    // One symbol later: ±64 at nominal misses, then +1 symbol hits (2 windows).
    BOOST_CHECK_EQUAL(sr.search_windows, 2u);
}

BOOST_AUTO_TEST_CASE(test_demod_core_r1_sfd_symmetric_partial_train_backward)
{
    // An SFD one symbol EARLIER than the nominal position (preamble start one
    // SYNC late) is recovered by the same symmetric window, even from a
    // 63/64 tracked train (np no longer narrows the half-width).
    std::vector<gr_complex> iq, tmpl;
    BOOST_REQUIRE(load_cf32(golden_dir() + "/window.cfile", iq));
    BOOST_REQUIRE(load_cf32("../../../testdata/reference_preamble.bin", tmpl));

    core::DemodScratch scratch;
    scratch.reserve(iq.size());
    auto prof = Qm35825Profile::Default();
    TimingResult tr;
    BOOST_REQUIRE(core::stage_timing(iq.data(), iq.size(), prof, tmpl, 9984, tr,
                                     scratch));
    for (auto& peak : tr.peak_samples)
        peak += static_cast<int64_t>(kQm35SamplesPerSymbol);
    tr.preamble_start_sample += static_cast<int64_t>(kQm35SamplesPerSymbol);
    BOOST_REQUIRE_EQUAL(tr.peak_samples.size(), size_t(64));
    tr.peak_samples.pop_back();
    tr.detected_peaks = tr.peak_samples.size();

    const auto sfd_seq = GetSfdSequence("ieee");
    SfdResult sr;
    BOOST_REQUIRE(core::stage_sfd(iq.data(), iq.size(), prof, tr, sfd_seq, tmpl,
                                  sr, scratch));
    BOOST_CHECK_EQUAL(sr.sfd_start_sample, int64_t(75008));
    BOOST_CHECK_GE(sr.metric, 0.95f);
    // One symbol earlier: nominal miss, +1 miss, then -1 hits (3 windows).
    BOOST_CHECK_EQUAL(sr.search_windows, 3u);
}

BOOST_AUTO_TEST_CASE(test_demod_core_low_snr_sfd_discrete_grid)
{
    // Low-SNR SFD acquisition.  The fast path (+-4 symbols, 0.02 floor) cannot
    // reach -10..-14 dB: the SFD peak is 0.008..0.05 and the origin can be
    // several SYMBOLS off after the low-SNR timing fold.  The discrete
    // integer-symbol grid (+-36 symbols, 0.008 floor, PAR >= 1.5, shared
    // 8-symbol polarity lock) must recover it without any pure-noise hit.
    std::vector<gr_complex> sync;
    BOOST_REQUIRE(load_cf32("../../../testdata/reference_preamble.bin", sync));
    BOOST_REQUIRE_EQUAL(sync.size(), kQm35SamplesPerSymbol);
    const auto sfd_seq = GetSfdSequence("4z2");
    const size_t sym = kQm35SamplesPerSymbol;
    const size_t reps = 256;
    const int64_t pre = 8192;
    std::vector<gr_complex> sfd_wf(sfd_seq.size() * sym);
    for (size_t i = 0; i < sfd_seq.size(); ++i)
        for (size_t k = 0; k < sym; ++k)
            sfd_wf[i * sym + k] =
                static_cast<float>(sfd_seq[i]) * sync[k];
    const int64_t true_start = pre;
    const int64_t true_sfd = true_start + static_cast<int64_t>(reps * sym);
    const size_t n = static_cast<size_t>(true_sfd) + sfd_wf.size() + 4096;

    double p_sig = 0.0;
    for (size_t i = 0; i < sym; ++i)
        p_sig += std::norm(sync[i]);
    p_sig /= static_cast<double>(sym);

    auto prof = Qm35825Profile::Default();
    prof.preamble_repetitions = reps;
    prof.sfd_mode = "4z2";
    // Both SFD backends must reach the same answer.  rx_fast_sfd = false uses
    // the scalar kron template; true uses the cached real-template kernels,
    // which also enables the per-SYMBOL decomposition of the discrete grid.
    for (bool fast_sfd : { false, true }) {
    prof.rx_fast_sfd = fast_sfd;

    auto make_tr = [&](int64_t start) {
        TimingResult tr;
        tr.preamble_start_sample = start;
        tr.measured_period = static_cast<double>(sym);
        tr.detected_peaks = reps;
        tr.peak_samples.push_back(start + static_cast<int64_t>(sym - 1));
        tr.peak_metrics.push_back(0.5f);
        tr.peak_corr.push_back(gr_complex(1.0f, 0.0f));
        tr.ok = true;
        return tr;
    };

    const double snrs[] = { -10.0, -12.0, -14.0 };
    constexpr int kFrames = 50;
    for (double snr : snrs) {
        int passed = 0;
        int within2 = 0;
        int64_t worst = 0;
        for (int f = 0; f < kFrames; ++f) {
            std::vector<gr_complex> rx(n, gr_complex(0.0f, 0.0f));
            for (size_t i = 0; i < reps; ++i)
                std::copy(sync.begin(), sync.end(),
                          rx.begin() + true_start +
                              static_cast<int64_t>(i * sym));
            std::copy(sfd_wf.begin(), sfd_wf.end(), rx.begin() + true_sfd);
            const double snr_lin = std::pow(10.0, snr / 10.0);
            const double p_noise = p_sig * (998.4 / 500.0) / snr_lin;
            const float sigma = static_cast<float>(std::sqrt(p_noise / 2.0));
            std::mt19937 rng(9000u + static_cast<uint32_t>(f));
            std::normal_distribution<float> g(0.0f, 1.0f);
            for (auto& v : rx)
                v += gr_complex(sigma * g(rng), sigma * g(rng));
            if (snr == -12.0 && f == 0) {
                auto shifted = make_tr(true_start - 4 * static_cast<int64_t>(sym) - 20);
                shifted.metric = 0.04f;
                core::DemodScratch shifted_scratch;
                SfdResult shifted_sfd;
                BOOST_REQUIRE(core::stage_sfd(rx.data(), rx.size(), prof, shifted,
                                             sfd_seq, sync, shifted_sfd, shifted_scratch));
                BOOST_CHECK_EQUAL(shifted_sfd.sfd_start_sample, true_sfd);
                BOOST_CHECK_EQUAL(shifted_sfd.first_threshold_backtrack_symbols, 4);
            }
            // +-10 SYMBOL origin error: outside the fast path's +-4 window.
            const int64_t off =
                static_cast<int64_t>(static_cast<int>(f % 21) - 10) *
                static_cast<int64_t>(sym);
            TimingResult tr = make_tr(true_start + off);
            core::DemodScratch scratch;
            scratch.reserve(rx.size());
            SfdResult sr;
            if (!core::stage_sfd(rx.data(), rx.size(), prof, tr, sfd_seq, sync,
                                 sr, scratch))
                continue;
            ++passed;
            const int64_t err = std::llabs(sr.sfd_start_sample - true_sfd);
            worst = std::max(worst, err);
            if (err <= 2)
                ++within2;
        }
        std::printf("sfd low-snr: fast=%d snr=%.0f passed=%d/%d within2=%d "
                    "worst=%lld\n",
                    fast_sfd ? 1 : 0, snr, passed, kFrames, within2,
                    static_cast<long long>(worst));
        BOOST_CHECK_GE(passed, 48);
        BOOST_CHECK_GE(within2, 48);
    }

    int false_alarms = 0;
    for (int w = 0; w < 200; ++w) {
        std::vector<gr_complex> rx(n, gr_complex(0.0f, 0.0f));
        std::mt19937 rng(70000u + static_cast<uint32_t>(w));
        std::normal_distribution<float> g(0.0f, 1.0f);
        for (auto& v : rx)
            v = gr_complex(g(rng), g(rng));
        TimingResult tr = make_tr(true_start);
        core::DemodScratch scratch;
        scratch.reserve(rx.size());
        SfdResult sr;
        if (core::stage_sfd(rx.data(), rx.size(), prof, tr, sfd_seq, sync, sr,
                            scratch))
            ++false_alarms;
    }
    std::printf("sfd low-snr noise: fast=%d %d/200 false alarms\n",
                fast_sfd ? 1 : 0, false_alarms);
    BOOST_CHECK_EQUAL(false_alarms, 0);
    }
}

BOOST_AUTO_TEST_CASE(test_demod_core_r1_sfd_recovers_four_symbol_forward)
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
    for (auto& peak : tr.peak_samples)
        peak -= static_cast<int64_t>(2 * kQm35SamplesPerSymbol);
    tr.preamble_start_sample -=
        static_cast<int64_t>(2 * kQm35SamplesPerSymbol);

    const auto sfd_seq = GetSfdSequence("ieee");
    SfdResult sr;
    BOOST_REQUIRE(core::stage_sfd(iq.data(), iq.size(), prof, tr, sfd_seq, tmpl,
                                  sr, scratch));
    BOOST_CHECK_EQUAL(sr.sfd_start_sample, int64_t(75008));
    BOOST_CHECK_GE(sr.metric, 0.95f);
}

BOOST_AUTO_TEST_CASE(test_demod_core_r1_sfd_recovers_four_symbol_backward)
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
    for (auto& peak : tr.peak_samples)
        peak += static_cast<int64_t>(2 * kQm35SamplesPerSymbol);
    tr.preamble_start_sample +=
        static_cast<int64_t>(2 * kQm35SamplesPerSymbol);

    const auto sfd_seq = GetSfdSequence("ieee");
    SfdResult sr;
    BOOST_REQUIRE(core::stage_sfd(iq.data(), iq.size(), prof, tr, sfd_seq, tmpl,
                                  sr, scratch));
    BOOST_CHECK_EQUAL(sr.sfd_start_sample, int64_t(75008));
    BOOST_CHECK_GE(sr.metric, 0.95f);
}

BOOST_AUTO_TEST_CASE(test_demod_core_r1_sfd_grid_rescues_five_symbol_forward)
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
    for (auto& peak : tr.peak_samples)
        peak -= static_cast<int64_t>(5 * kQm35SamplesPerSymbol);
    tr.preamble_start_sample -=
        static_cast<int64_t>(5 * kQm35SamplesPerSymbol);

    const auto sfd_seq = GetSfdSequence("ieee");
    SfdResult sr;
    // The +/-4-symbol direct windows miss, but the +/-36-symbol discrete grid
    // is the documented low-SNR rescue and must find the true SFD.  (The old
    // `par_only &&` guard in polarity_ok made every ieee/decawave SFD fail
    // that check, which is why this test used to expect a miss.)
    BOOST_REQUIRE(core::stage_sfd(iq.data(), iq.size(), prof, tr, sfd_seq, tmpl,
                                  sr, scratch));
    BOOST_CHECK_EQUAL(sr.sfd_start_sample, int64_t(75008));
    BOOST_CHECK_GE(sr.metric, 0.95f);
}

BOOST_AUTO_TEST_CASE(test_demod_core_r1_sfd_grid_rescues_five_symbol_backward)
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
    for (auto& peak : tr.peak_samples)
        peak += static_cast<int64_t>(5 * kQm35SamplesPerSymbol);
    tr.preamble_start_sample +=
        static_cast<int64_t>(5 * kQm35SamplesPerSymbol);

    const auto sfd_seq = GetSfdSequence("ieee");
    SfdResult sr;
    BOOST_REQUIRE(core::stage_sfd(iq.data(), iq.size(), prof, tr, sfd_seq, tmpl,
                                  sr, scratch));
    BOOST_CHECK_EQUAL(sr.sfd_start_sample, int64_t(75008));
    BOOST_CHECK_GE(sr.metric, 0.95f);
}

BOOST_AUTO_TEST_CASE(test_demod_core_sfd_sfo_grid_cache_equivalence)
{
    // The per-delta symbol cache must reproduce the direct per-symbol grid on
    // an untrimmed clock (+27 ppm), where round(k*period) - k*sym is not zero
    // and the old code disabled the decomposition entirely.
    std::vector<gr_complex> iq, tmpl;
    BOOST_REQUIRE(load_cf32(golden_dir() + "/window.cfile", iq));
    BOOST_REQUIRE(load_cf32("../../../testdata/reference_preamble.bin", tmpl));

    core::DemodScratch scratch;
    scratch.reserve(iq.size());
    auto prof = Qm35825Profile::Default();
    TimingResult tr;
    BOOST_REQUIRE(core::stage_timing(iq.data(), iq.size(), prof, tmpl, 9984, tr,
                                     scratch));
    tr.measured_period += 0.027; // +27 ppm on the 1016-sample SYNC period
    const auto sfd_seq = GetSfdSequence("4z2");

    Qm35825Profile dense = prof;
    dense.rx_fast_sfd = false;
    dense.rx_timing_sparse = false;
    SfdResult direct;
    BOOST_REQUIRE(core::stage_sfd(iq.data(), iq.size(), dense, tr, sfd_seq,
                                  tmpl, direct, scratch));

    Qm35825Profile cached = prof;
    cached.rx_fast_sfd = true;
    cached.rx_timing_sparse = false;
    SfdResult fast;
    BOOST_REQUIRE(core::stage_sfd(iq.data(), iq.size(), cached, tr, sfd_seq,
                                  tmpl, fast, scratch));

    BOOST_CHECK_EQUAL(fast.sfd_start_sample, direct.sfd_start_sample);
    BOOST_CHECK_SMALL(std::abs(fast.metric - direct.metric), 1e-4f);
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

BOOST_AUTO_TEST_CASE(test_demod_core_cir_fused_rotation_matches_derotated)
{
    // CIR-1: folding the CFO rotation into the SYNC average (raw rx + per-SYNC
    // scalar + one per-sample ramp) must reproduce the legacy "derotate the
    // whole preamble, then average" result, and the tail average must give the
    // same sync_phase_gain statistic without re-reading 32 SYNC windows.
    std::vector<gr_complex> iq;
    BOOST_REQUIRE(load_cf32(golden_dir() + "/window.cfile", iq));
    const double cfo_hz = 25000.0;
    const double w = 2.0 * M_PI * cfo_hz / kQm35SampleRate;
    std::vector<gr_complex> iqc(iq.size());
    for (size_t i = 0; i < iq.size(); ++i) {
        const double ph = w * static_cast<double>(i);
        iqc[i] = iq[i] * gr_complex(static_cast<float>(std::cos(ph)),
                                    static_cast<float>(std::sin(ph)));
    }

    core::DemodScratch scratch_a;
    core::DemodScratch scratch_b;
    scratch_a.reserve(iqc.size());
    scratch_b.reserve(iqc.size());
    auto prof = Qm35825Profile::Default();
    std::vector<gr_complex> tmpl;
    BOOST_REQUIRE(load_cf32("../../../testdata/reference_preamble.bin", tmpl));
    TimingResult tr;
    CfoResult cf;
    BOOST_REQUIRE(core::stage_timing(iqc.data(), iqc.size(), prof, tmpl, 9984,
                                     tr, scratch_a));
    BOOST_REQUIRE(core::stage_cfo(iqc.data(), iqc.size(), prof, tr, cf, scratch_a));
    BOOST_CHECK_CLOSE(cf.cfo_hz, cfo_hz, 10.0);
    std::vector<int8_t> code9(kPreambleCode9.begin(), kPreambleCode9.end());

    CirResult legacy;
    CirResult fused;
    BOOST_REQUIRE(core::estimate_cir(scratch_a.derotated.data(), iqc.size(), prof,
                                     tr, code9, legacy, scratch_a));
    BOOST_REQUIRE(core::estimate_cir(iqc.data(), iqc.size(), prof, tr, code9,
                                     fused, scratch_b, cf.cfo_hz));
    BOOST_REQUIRE_EQUAL(fused.cir_values.size(), legacy.cir_values.size());
    BOOST_CHECK_EQUAL(fused.first_path_sample, legacy.first_path_sample);
    BOOST_CHECK_SMALL(std::abs(fused.cir_peak_metric - legacy.cir_peak_metric),
                      1e-4f);
    double l2 = 0.0;
    for (size_t i = 0; i < legacy.cir_values.size(); ++i) {
        const double d = static_cast<double>(fused.cir_values[i] -
                                             legacy.cir_values[i]);
        l2 += d * d;
    }
    BOOST_CHECK_SMALL(std::sqrt(l2), 1e-4);

    BOOST_REQUIRE(scratch_b.cir_avg_tail_valid);
    const auto spread = BuildSampledCode(code9.data(), code9.size());
    const size_t taps = fused.cir_filter_taps.size();
    const int64_t chip_start = tr.preamble_start_sample +
        static_cast<int64_t>(taps) -
        static_cast<int64_t>(prof.cir_pre_samples) - 1;
    const size_t phase_reps =
        std::min<size_t>(32, prof.preamble_repetitions);
    const size_t phase_first = (prof.preamble_repetitions - phase_reps) *
                               kQm35ChipsPerSymbol;
    const int64_t base = chip_start - static_cast<int64_t>(taps - 1) +
                         static_cast<int64_t>(2 * phase_first);
    std::vector<gr_complex> sum;
    gr_complex gain_legacy{};
    gr_complex gain_tail{};
    BOOST_REQUIRE(rxopt::sync_phase_gain(
        scratch_a.derotated.data(), iqc.size(), base,
        fused.cir_filter_taps.data(), taps, spread, phase_reps, sum,
        gain_legacy));
    BOOST_REQUIRE(rxopt::sync_phase_gain_from_tail(
        scratch_b.cir_avg_tail.data(), fused.cir_filter_taps.data(), taps,
        spread, gain_tail));
    BOOST_CHECK_SMALL(std::abs(gain_tail - gain_legacy) /
                          std::max(1e-6f, std::abs(gain_legacy)), 1e-3f);
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
    // The complete path (including SFD-anchored CFO retracking) must keep
    // MATLAB's 24-SYNC startup exclusion and 40-point LS fit.
    BOOST_CHECK_EQUAL(res.cfo.skipped_peaks, size_t(24));
    BOOST_CHECK_EQUAL(res.cfo.peaks_used, size_t(40));
    BOOST_CHECK_EQUAL(res.cfo.fit_first_peak_sample,
                      res.timing.peak_samples[24]);
    BOOST_CHECK_EQUAL(res.cfo.fit_last_peak_sample,
                      res.timing.peak_samples.back());
    BOOST_CHECK_EQUAL(res.ns_sfd.sfd_start_chip, int64_t(32512));
    BOOST_CHECK_EQUAL(res.phr.psdu_length, uint32_t(127));
    BOOST_CHECK(!res.phr.secded_uncorrectable);
}

BOOST_AUTO_TEST_CASE(test_demod_core_r3_sfd_child_subtimers_are_consistent)
{
    // S0-A: the SFD parent stage publishes diagnostic child sub-timers.  They
    // must partition `stage_sfd_us` and must NOT be double-counted into
    // `stage_total_us`.  Decoding behaviour itself must be unchanged.
    std::vector<gr_complex> iq;
    BOOST_REQUIRE(load_cf32(golden_dir() + "/window.cfile", iq));
    core::DemodScratch scratch;
    scratch.reserve(iq.size());
    auto prof = Qm35825Profile::Default();
    prof.sfd_mode = "ieee";
    const auto res = core::demodulate_one(
        iq.data(), iq.size(), prof, 1, 9984, 0, load_reference_template(),
        scratch);

    // (c) the decode verdict is unchanged (same fixture as the r3 pipeline
    // test above).
    BOOST_REQUIRE(res.status == DemodStatus::Success);
    BOOST_REQUIRE(res.payload.ok);
    BOOST_CHECK(res.payload.fcs_pass);

    // (a) children partition the parent.  `stage_sfd_anchor_other_us` is the
    // clamped remainder, so the sum telescopes back to `stage_sfd_us`; an
    // independent-clock overshoot would clamp to 0 and break the equality.
    const uint64_t children_us = res.stage_sfd_bootstrap_us +
        res.stage_sfd_tail_retrack_us + res.stage_sfd_cfo_refit_us +
        res.stage_sfd_final_us;
    BOOST_TEST_MESSAGE("stage_sfd_us=" << res.stage_sfd_us
                       << " bootstrap=" << res.stage_sfd_bootstrap_us
                       << " tail_retrack=" << res.stage_sfd_tail_retrack_us
                       << " cfo_refit=" << res.stage_sfd_cfo_refit_us
                       << " final=" << res.stage_sfd_final_us
                       << " anchor_other=" << res.stage_sfd_anchor_other_us);
    BOOST_CHECK_LE(children_us, res.stage_sfd_us);
    BOOST_CHECK_EQUAL(children_us + res.stage_sfd_anchor_other_us,
                      res.stage_sfd_us);
    BOOST_CHECK_GT(res.stage_sfd_bootstrap_us, uint64_t(0));
    // The final SFD is skipped when the refit reused the Stage-2 rotation or
    // moved the 8-symbol phase by < 0.05 rad; otherwise it must have run.
    const double drift8_rad = std::abs(
        2.0 * M_PI * (res.cfo.cfo_hz - res.cfo_initial_hz) *
        (8.0 * static_cast<double>(kQm35SamplesPerSymbol)) / prof.sample_rate);
    BOOST_CHECK(res.stage_sfd_final_us > 0 || res.cfo_refit_action != 0 ||
                drift8_rad < 0.05);

    // (b) `stage_total_us` is the sum of exactly the fields written by the
    // `lap()` lambda (timing, cfo, sfd, cir, ns_sfd, phr, payload) plus the
    // separately timed soft-chip fallback bucket.  The SFD children are NOT
    // separately accumulated, so this equality would fail if they leaked into
    // the total.
    const uint64_t parent_sum = res.stage_timing_us + res.stage_cfo_us +
        res.stage_sfd_us + res.stage_cir_us + res.stage_ns_sfd_us +
        res.stage_phr_us + res.stage_payload_us +
        res.stage_softchip_fallback_us;
    BOOST_CHECK_EQUAL(res.stage_total_us, parent_sum);
}

// ---------------------------------------------------------------------------
// SFD-anchored CFO refit policy (single-pass rotation).
//
// `demodulate_one` fits the carrier offset twice and derotates the downstream
// span twice.  The second pass is the largest single cost inside `stage_sfd`.
// `Qm35825Profile::cfo_refit_mode` can replace it with one rotation that uses
// the Stage-2 estimate.  The contract that must hold:
//   * legacy (default) still runs both fits and both rotations,
//   * skip keeps the Stage-2 estimate and drops one rotation pass,
//   * the decode verdict (status / PSDU / FCS) is unchanged,
//   * auto falls back to the legacy path when the drift budget is exceeded.
// ---------------------------------------------------------------------------
namespace {

struct CfoRefitRun {
    DemodResult res;
    std::vector<uint8_t> psdu;
};

CfoRefitRun run_cfo_refit_case(const std::vector<gr_complex>& iq,
                               const std::vector<gr_complex>& tmpl,
                               int mode,
                               double skip_rad,
                               size_t min_peaks)
{
    core::DemodScratch scratch;
    scratch.reserve(iq.size());
    auto prof = Qm35825Profile::Default();
    prof.sfd_mode = "ieee";
    prof.cfo_refit_mode = mode;
    prof.cfo_refit_skip_rad = skip_rad;
    prof.cfo_refit_min_peaks = min_peaks;
    CfoRefitRun out;
    out.res = core::demodulate_one(iq.data(), iq.size(), prof, 1, 9984, 0,
                                   tmpl, scratch);
    out.psdu = out.res.payload.bytes;
    return out;
}

} // namespace

BOOST_AUTO_TEST_CASE(test_demod_core_cfo_refit_skip_matches_legacy_decode)
{
    std::vector<gr_complex> iq;
    BOOST_REQUIRE(load_cf32(golden_dir() + "/window.cfile", iq));
    const auto tmpl = load_reference_template();
    BOOST_REQUIRE(!tmpl.empty());

    // Legacy: both fits, both rotations.  The Stage-2 fit is published, the
    // refit replaces it, and the second rotation covers the whole window.
    const auto legacy = run_cfo_refit_case(iq, tmpl, 0, 0.10, 64);
    BOOST_REQUIRE(legacy.res.status == DemodStatus::Success);
    BOOST_CHECK(legacy.res.payload.fcs_pass);
    BOOST_CHECK_EQUAL(legacy.res.cfo_refit_action, uint32_t(0));
    BOOST_CHECK_EQUAL(legacy.res.stage_cfo_rotation_passes, uint32_t(2));
    BOOST_CHECK_GT(legacy.res.cfo_refit_span_samples, uint64_t(0));
    BOOST_CHECK_GT(legacy.res.cfo_initial_peaks, uint32_t(0));
    BOOST_CHECK_GT(legacy.res.cfo_refit_peaks, uint32_t(0));
    BOOST_CHECK_EQUAL(legacy.res.cfo.cfo_hz, legacy.res.cfo_refit_hz);
    BOOST_CHECK(std::abs(legacy.res.cfo_initial_hz - legacy.res.cfo_refit_hz) <
                1e4);

    // Forced skip: the Stage-2 estimate survives, one rotation pass is gone,
    // and the payload/FCS verdict is identical.
    const auto skipped = run_cfo_refit_case(iq, tmpl, 2, 0.10, 64);
    BOOST_REQUIRE(skipped.res.status == DemodStatus::Success);
    BOOST_CHECK(skipped.res.payload.fcs_pass);
    BOOST_CHECK_EQUAL(skipped.res.cfo_refit_action, uint32_t(2));
    BOOST_CHECK_EQUAL(skipped.res.stage_cfo_rotation_passes, uint32_t(1));
    BOOST_CHECK_EQUAL(skipped.res.cfo.cfo_hz, skipped.res.cfo_initial_hz);
    BOOST_CHECK_EQUAL(skipped.res.cfo_refit_rotate_us, uint64_t(0));
    // The refit is still measured (it is what the decision is based on) but it
    // must not be applied.
    BOOST_CHECK_GT(skipped.res.cfo_refit_peaks, uint32_t(0));
    BOOST_CHECK_GT(skipped.res.cfo_refit_span_samples, uint64_t(0));
    // Same PSDU and same SFD/PHR coordinates: the rotation only changes the
    // carrier phase reference, not the frame geometry.
    BOOST_CHECK(skipped.psdu == legacy.psdu);
    BOOST_CHECK_EQUAL(skipped.res.ns_sfd.sfd_start_chip,
                      legacy.res.ns_sfd.sfd_start_chip);
    BOOST_CHECK_EQUAL(skipped.res.phr.psdu_length, legacy.res.phr.psdu_length);
    BOOST_CHECK_EQUAL(skipped.res.timing.preamble_start_sample,
                      legacy.res.timing.preamble_start_sample);
    BOOST_CHECK_EQUAL(skipped.res.sfd.sfd_start_sample,
                      legacy.res.sfd.sfd_start_sample);
}

BOOST_AUTO_TEST_CASE(test_demod_core_cfo_refit_auto_falls_back_on_drift)
{
    std::vector<gr_complex> iq;
    BOOST_REQUIRE(load_cf32(golden_dir() + "/window.cfile", iq));
    const auto tmpl = load_reference_template();
    BOOST_REQUIRE(!tmpl.empty());

    // A zero drift budget can never be met, so auto must keep the legacy path
    // and report the fallback instead of silently skipping.
    const auto strict = run_cfo_refit_case(iq, tmpl, 1, 1e-12, 64);
    BOOST_REQUIRE(strict.res.status == DemodStatus::Success);
    BOOST_CHECK(strict.res.payload.fcs_pass);
    BOOST_CHECK_EQUAL(strict.res.cfo_refit_action, uint32_t(3));
    BOOST_CHECK_EQUAL(strict.res.stage_cfo_rotation_passes, uint32_t(2));
    BOOST_CHECK_EQUAL(strict.res.cfo.cfo_hz, strict.res.cfo_refit_hz);

    // A peak-count requirement the Stage-2 fit cannot satisfy must also fall
    // back (the fit only sees preamble_repetitions - 24 peaks).
    const auto too_few = run_cfo_refit_case(iq, tmpl, 1, 1e9, 100000);
    BOOST_REQUIRE(too_few.res.status == DemodStatus::Success);
    BOOST_CHECK_EQUAL(too_few.res.cfo_refit_action, uint32_t(3));
    BOOST_CHECK_EQUAL(too_few.res.stage_cfo_rotation_passes, uint32_t(2));

    // A generous budget and a reachable peak count must skip.
    const auto loose = run_cfo_refit_case(iq, tmpl, 1, 1e9, 1);
    BOOST_REQUIRE(loose.res.status == DemodStatus::Success);
    BOOST_CHECK(loose.res.payload.fcs_pass);
    BOOST_CHECK_EQUAL(loose.res.cfo_refit_action, uint32_t(1));
    BOOST_CHECK_EQUAL(loose.res.stage_cfo_rotation_passes, uint32_t(1));
    BOOST_CHECK_EQUAL(loose.res.cfo.cfo_hz, loose.res.cfo_initial_hz);
    BOOST_CHECK(loose.psdu == strict.psdu);
}

BOOST_AUTO_TEST_CASE(test_demod_core_cfo_refit_legacy_default_is_unchanged)
{
    // The default profile must keep the pre-existing two-fit / two-rotation
    // behaviour so every historical measurement stays comparable.
    std::vector<gr_complex> iq;
    BOOST_REQUIRE(load_cf32(golden_dir() + "/window.cfile", iq));
    const auto tmpl = load_reference_template();
    BOOST_REQUIRE(!tmpl.empty());

    const auto dflt = run_cfo_refit_case(iq, tmpl, 0, 0.10, 64);
    BOOST_REQUIRE(dflt.res.status == DemodStatus::Success);
    BOOST_CHECK_EQUAL(dflt.res.cfo.skipped_peaks, size_t(24));
    BOOST_CHECK_EQUAL(dflt.res.cfo.peaks_used, size_t(40));
    BOOST_CHECK_EQUAL(dflt.res.cfo_refit_action, uint32_t(0));
    BOOST_CHECK_EQUAL(dflt.res.stage_cfo_rotation_passes, uint32_t(2));
    BOOST_CHECK(dflt.res.cfo_refit_rotate_us <= dflt.res.stage_sfd_cfo_refit_us);
}

BOOST_AUTO_TEST_CASE(test_demod_core_second_rotation_stops_at_payload_start)
{
    // The payload pass derotates from min(sec_hi, base0) onwards.  The second
    // rotation used to run to the legacy 40-PHR-head budget, which sits PAST
    // the payload start, so ~35k samples were derotated twice with the same
    // CFO.  The bound must now stop at the real requirement (soft-chip prefix
    // and fused-PHR reads) and the decode must be unchanged.
    std::vector<gr_complex> iq;
    BOOST_REQUIRE(load_cf32(golden_dir() + "/window.cfile", iq));
    const auto tmpl = load_reference_template();
    BOOST_REQUIRE(!tmpl.empty());

    auto prof = Qm35825Profile::Default();
    prof.sfd_mode = "ieee";
    prof.rx_fast_sfd = true;
    prof.rx_fast_cfo_rotation = true;
    prof.rx_softchip_on_demand = true;
    prof.rx_fused_payload_cmf = true;
    prof.rx_fused_phr_cmf = true;

    core::DemodScratch scratch;
    scratch.reserve(iq.size());
    const auto res = core::demodulate_one(iq.data(), iq.size(), prof, 1, 9984, 0,
                                          tmpl, scratch);
    BOOST_REQUIRE(res.status == DemodStatus::Success);
    BOOST_CHECK(res.payload.fcs_pass);
    BOOST_CHECK_EQUAL(res.payload.received_fcs, uint16_t(0x584b));

    // The legacy budget would need 2 * initial_chip_budget samples; the real
    // requirement is smaller because the fused PHR reads raw IQ.
    const size_t legacy_span = 2 * rxopt::initial_chip_budget(prof);
    BOOST_CHECK_LT(res.cfo_refit_span_samples, legacy_span);
    BOOST_CHECK_GT(res.cfo_refit_span_samples, uint64_t(0));
}

BOOST_AUTO_TEST_CASE(test_demod_core_single_pass_rotation_reduces_rotated_samples)
{
    // Single-pass mode must derotate strictly fewer samples than the legacy
    // two-pass chain while producing the same PSDU.
    std::vector<gr_complex> iq;
    BOOST_REQUIRE(load_cf32(golden_dir() + "/window.cfile", iq));
    const auto tmpl = load_reference_template();
    BOOST_REQUIRE(!tmpl.empty());

    auto make = [&](int mode) {
        auto prof = Qm35825Profile::Default();
        prof.sfd_mode = "ieee";
        prof.rx_fast_sfd = true;
        prof.rx_fast_cfo_rotation = true;
        prof.rx_softchip_on_demand = true;
        prof.rx_fused_payload_cmf = true;
        prof.rx_fused_phr_cmf = true;
        prof.cfo_refit_mode = mode;
        return prof;
    };
    auto run = [&](int mode) {
        core::DemodScratch scratch;
        scratch.reserve(iq.size());
        return core::demodulate_one(iq.data(), iq.size(), make(mode), 1, 9984, 0,
                                    tmpl, scratch);
    };

    const auto legacy = run(0);
    const auto single = run(2);
    BOOST_REQUIRE(legacy.status == DemodStatus::Success);
    BOOST_REQUIRE(single.status == DemodStatus::Success);
    BOOST_CHECK(legacy.payload.fcs_pass);
    BOOST_CHECK(single.payload.fcs_pass);
    BOOST_CHECK(single.payload.bytes == legacy.payload.bytes);
    BOOST_CHECK_EQUAL(single.cfo_refit_action, uint32_t(2));
    BOOST_CHECK_EQUAL(single.stage_cfo_rotation_passes + 1,
                      legacy.stage_cfo_rotation_passes);
    BOOST_CHECK_LT(single.stage_cfo_rotated_samples,
                   legacy.stage_cfo_rotated_samples);
    BOOST_CHECK_EQUAL(single.cfo.cfo_hz, single.cfo_initial_hz);
    // Same frame geometry: the derotation only changes the phase reference.
    BOOST_CHECK_EQUAL(single.ns_sfd.sfd_start_chip, legacy.ns_sfd.sfd_start_chip);
    BOOST_CHECK_EQUAL(single.phr.psdu_length, legacy.phr.psdu_length);
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

// Apply co-sourced crystal CFO and matching SFO (rate = 1 + f_hz / carrier_freq_hz)
// to a baseband signal at sample_rate. Preserves buffer length (zero-padded).
std::vector<gr_complex> apply_cfo_sfo_constraint(
    const std::vector<gr_complex>& iq,
    double f_hz,
    double sample_rate,
    double carrier_freq_hz)
{
    std::vector<gr_complex> out(iq.size(), gr_complex(0.0f, 0.0f));
    if (iq.empty())
        return out;
    const double rate = (carrier_freq_hz > 0.0)
        ? (1.0 + f_hz / carrier_freq_hz)
        : 1.0;
    const double w = 2.0 * M_PI * f_hz / sample_rate;
    for (size_t i = 0; i < iq.size(); ++i) {
        gr_complex s(0.0f, 0.0f);
        if (rate == 1.0) {
            s = iq[i];
        } else {
            const double x = static_cast<double>(i) * rate;
            const size_t i0 = static_cast<size_t>(x);
            if (i0 + 1 < iq.size()) {
                const double frac = x - static_cast<double>(i0);
                s = iq[i0] * static_cast<float>(1.0 - frac) +
                    iq[i0 + 1] * static_cast<float>(frac);
            } else if (i0 < iq.size()) {
                s = iq[i0];
            }
        }
        if (f_hz != 0.0) {
            const double ph = w * static_cast<double>(i);
            s *= gr_complex(static_cast<float>(std::cos(ph)),
                            static_cast<float>(std::sin(ph)));
        }
        out[i] = s;
    }
    return out;
}

// Inject a carrier rotation of +f Hz (e^{+j2π f n/fs}) together with its
// co-sourced crystal SFO (1 + f/fc) into the golden window and run the full
// demod chain (IEEE SFD, seeded at the golden preamble).
DemodResult demod_window_with_cfo(const std::vector<gr_complex>& iq, double f_hz)
{
    auto prof = Qm35825Profile::Default();
    prof.sfd_mode = "ieee";
    auto iqc = apply_cfo_sfo_constraint(
        iq, f_hz, prof.sample_rate, prof.carrier_freq_hz);
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

BOOST_AUTO_TEST_CASE(test_demod_core_sfo_from_cfo_default_and_override)
{
    const auto def = Qm35825Profile::Default();
    BOOST_CHECK(def.rx_sfo_from_cfo);
    BOOST_CHECK_CLOSE(def.carrier_freq_hz, 6489.6e6, 1e-12);

    std::vector<gr_complex> iq;
    BOOST_REQUIRE(load_cf32(golden_dir() + "/window.cfile", iq));
    const auto tmpl = load_reference_template();

    // Coupled 27 ppm crystal offset: CFO = +175 kHz at 6489.6 MHz, SFO = +27 ppm.
    constexpr double kCfoHz = 175000.0;
    auto coupled = apply_cfo_sfo_constraint(
        iq, kCfoHz, def.sample_rate, def.carrier_freq_hz);
    auto p_on = Qm35825Profile::Default();
    p_on.sfd_mode = "ieee";
    core::DemodScratch scratch;
    scratch.reserve(coupled.size());
    const auto res_on = core::demodulate_one(
        coupled.data(), coupled.size(), p_on, 1, 9984, 0, tmpl, scratch);
    BOOST_CHECK(res_on.status == DemodStatus::Success);
    BOOST_CHECK(res_on.payload.fcs_pass);
    const double expected_period =
        static_cast<double>(kQm35SamplesPerSymbol) /
        (1.0 + res_on.cfo.cfo_hz / p_on.carrier_freq_hz);
    BOOST_CHECK_CLOSE(res_on.timing.measured_period, expected_period, 1e-9);

    // Explicit opt-out: decoupled CFO-only signal (0 ppm SFO) decodes cleanly
    // when rx_sfo_from_cfo = false, using Stage-1 timing OLS period (~1016).
    auto decoupled = apply_cfo_sfo_constraint(iq, kCfoHz, def.sample_rate, 0.0);
    auto p_off = p_on;
    p_off.rx_sfo_from_cfo = false;
    const auto res_off = core::demodulate_one(
        decoupled.data(), decoupled.size(), p_off, 2, 9984, 0, tmpl, scratch);
    BOOST_CHECK(res_off.status == DemodStatus::Success);
    BOOST_CHECK(res_off.payload.fcs_pass);
    BOOST_CHECK_SMALL(
        std::abs(res_off.timing.measured_period -
                 static_cast<double>(kQm35SamplesPerSymbol)),
        0.05);
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
    BOOST_CHECK_EQUAL(
        res.timing.preamble_start_sample,
        res.sfd.sfd_start_sample - static_cast<int64_t>(std::llround(
            static_cast<double>(prof.preamble_repetitions) *
            res.timing.measured_period)));
}

BOOST_AUTO_TEST_CASE(test_demod_core_p3_sfd_sfo_adaptation)
{
    // Apply the full preamble drift before rounding the SFD search centre.
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
    BOOST_CHECK_EQUAL(sr.expected_start_sample, tr.preamble_start_sample +
        std::llround(prof.preamble_repetitions * tr.measured_period));
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

BOOST_AUTO_TEST_CASE(test_demod_core_timing_volk_dot_agrees_with_scalar)
{
    constexpr size_t N = 1016;
    std::mt19937 rng(0x71A11u);
    std::uniform_real_distribution<float> dist(-1.0f, 1.0f);
    std::vector<std::complex<float>> input(N + 3), taps(N);
    for (auto& x : input)
        x = { dist(rng), dist(rng) };
    for (auto& x : taps)
        x = { dist(rng), dist(rng) };

    for (size_t off = 0; off < 3; ++off) {
        std::complex<float> ref(0.0f, 0.0f);
        for (size_t k = 0; k < N; ++k)
            ref += input[off + k] * std::conj(taps[k]);
        const auto got = core::detail::timing_conjugate_dot(
            input.data() + off, taps.data(), N);
        BOOST_CHECK_LT(std::abs(got - ref), 2e-3f);
    }
}

BOOST_AUTO_TEST_CASE(test_sync_matched_filter_exact_window_edges)
{
    const std::vector<std::complex<float>> taps{ { 1.0f, 0.0f }, { -1.0f, 0.0f } };
    rxopt::RealSyncTemplate real;
    rxopt::build_real_sync_template(taps, real);
    const rxopt::SyncMatchedFilter filter(taps.data(), taps.size(),
                                          real.taps_dup.data(), nullptr);
    const std::vector<std::complex<float>> single{ { 1.0f, 0.0f }, { -1.0f, 0.0f } };
    filter.for_each_lag(single.data(), 1, [](size_t lag, std::complex<float> value,
                                             bool batch) {
        BOOST_CHECK_EQUAL(lag, 0u);
        BOOST_CHECK_EQUAL(value, std::complex<float>(2.0f, 0.0f));
        BOOST_CHECK(!batch);
    });

    // A complete batch followed by the last legal lag in an exact-size buffer.
    std::vector<std::complex<float>> samples(10);
    samples[8] = single[0];
    samples[9] = single[1];
    const auto peak = rxopt::scan_normalized_sync(
        filter, samples.data(), 9, 0.0f, real.energy);
    BOOST_CHECK_EQUAL(peak.lag, 8u);
    BOOST_CHECK_EQUAL(peak.metric, 1.0f);
    BOOST_CHECK_EQUAL(peak.corr, std::complex<float>(2.0f, 0.0f));
}

BOOST_AUTO_TEST_CASE(test_demod_core_topk_rake_kernel_agrees_with_scalar)
{
    constexpr size_t N = 48;
    constexpr size_t K = 8;
    std::mt19937 rng(0x70A4u);
    std::uniform_real_distribution<float> dist(-1.0f, 1.0f);
    std::vector<std::complex<float>> input(N), weights(K);
    const uint8_t indices[K] = { 1, 4, 9, 15, 23, 31, 36, 37 };
    for (auto& x : input)
        x = { dist(rng), dist(rng) };
    for (auto& x : weights)
        x = { dist(rng), dist(rng) };

    std::complex<float> ref(0.0f, 0.0f);
    for (size_t i = 0; i < K; ++i)
        ref += weights[i] * input[indices[i]];
    const auto got = cir_fir::dot_topk(
        input.data(), indices, weights.data(), K);
    BOOST_CHECK_LT(std::abs(got - ref), 1e-5f);

#if UWB_CIR_FIR_HAVE_AVX2
    // Four outputs at decimation 2, including deliberately unaligned input.
    for (size_t k : { size_t(4), size_t(8) }) {
        std::complex<float> out[4];
        if (k == 4)
            cir_fir::dot_topk_x4_avx2<4>(
                input.data() + 1, indices, weights.data(), out);
        else
            cir_fir::dot_topk_x4_avx2<8>(
                input.data() + 1, indices, weights.data(), out);
        for (size_t o = 0; o < 4; ++o) {
            const auto scalar = cir_fir::dot_topk(
                input.data() + 1 + 2 * o, indices, weights.data(), k);
            BOOST_CHECK_LT(std::abs(out[o] - scalar), 1e-5f);
        }
    }
#endif
}

BOOST_AUTO_TEST_CASE(test_demod_core_topk_rake_golden_fcs)
{
    std::vector<gr_complex> iq;
    BOOST_REQUIRE(load_cf32(golden_dir() + "/window.cfile", iq));
    const auto tmpl = load_reference_template();
    for (size_t k : { size_t(4), size_t(8) }) {
        auto prof = Qm35825Profile::Default();
        prof.sfd_mode = "ieee";
        prof.cir_rake_top_k = k;
        core::DemodScratch scratch;
        scratch.reserve(iq.size());
        const auto res = core::demodulate_one(
            iq.data(), iq.size(), prof, 1, 9984, 0, tmpl, scratch);
        BOOST_TEST_CONTEXT("Top-K=" << k) {
            BOOST_REQUIRE(res.status == DemodStatus::Success);
            BOOST_CHECK(res.payload.fcs_pass);
            BOOST_CHECK_EQUAL(res.payload.received_fcs, uint16_t(0x584b));
        }
    }
}

BOOST_AUTO_TEST_CASE(test_demod_core_bypass_filter_golden_fcs)
{
    std::vector<gr_complex> iq;
    BOOST_REQUIRE(load_cf32(golden_dir() + "/window.cfile", iq));
    const auto tmpl = load_reference_template();
    auto prof = Qm35825Profile::Default();
    prof.sfd_mode = "ieee";
    prof.cir_soft_chip_mode = CirSoftChipMode::Bypass;
    core::DemodScratch scratch;
    scratch.reserve(iq.size());
    const auto res = core::demodulate_one(
        iq.data(), iq.size(), prof, 1, 9984, 0, tmpl, scratch);
    BOOST_REQUIRE(res.status == DemodStatus::Success);
    BOOST_CHECK(res.payload.fcs_pass);
    BOOST_CHECK_EQUAL(res.payload.received_fcs, uint16_t(0x584b));
}

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

// ---------------------------------------------------------------------------
// R7: code-10 / DW1000 profile self-consistency + representative robustness.
// Cyclic origin of kPreambleCode10 is locked to the MATLAB-generated
// testdata/uwb_code10_preamble16_payload8.cfile single-symbol template.
// ---------------------------------------------------------------------------

BOOST_AUTO_TEST_CASE(test_demod_core_r7_code10_self_consistency)
{
    // code-10: ternary {-1,0,+1}, length 127, 64 non-zeros, energy 64.
    // code-9 path must be UNCHANGED (same pointer content as before R7).
    const int8_t* c9 = GetPreambleCode(9);
    const int8_t* c10 = GetPreambleCode(10);
    BOOST_REQUIRE(c9 != nullptr);
    BOOST_REQUIRE(c10 != nullptr);
    BOOST_CHECK(c9 != c10);

    size_t nnz9 = 0, nnz10 = 0, e9 = 0, e10 = 0;
    for (size_t i = 0; i < kQm35CodeLength; ++i) {
        BOOST_CHECK(c9[i] == -1 || c9[i] == 0 || c9[i] == 1);
        BOOST_CHECK(c10[i] == -1 || c10[i] == 0 || c10[i] == 1);
        if (c9[i] != 0)
            ++nnz9;
        if (c10[i] != 0)
            ++nnz10;
        e9 += static_cast<size_t>(std::abs(c9[i]));
        e10 += static_cast<size_t>(std::abs(c10[i]));
    }
    BOOST_CHECK_EQUAL(nnz9, size_t(64));
    BOOST_CHECK_EQUAL(e9, size_t(64));
    BOOST_CHECK_EQUAL(nnz10, size_t(64));
    BOOST_CHECK_EQUAL(e10, size_t(64));

    // MATLAB R2025b lrwpan.internal.HRPCodes(9:12) lock.  FNV-1a over the
    // exact ternary sequence catches a cyclic origin error as well as a wrong
    // chip value, while keeping this QA compact.
    const std::array<size_t, 4> code_indices = { 9, 10, 11, 12 };
    const std::array<uint64_t, 4> matlab_hashes = {
        UINT64_C(0xcba009f2c3a1660a), UINT64_C(0xb90c5ccecde2762c),
        UINT64_C(0x5ca2e4c081cae608), UINT64_C(0xb95c6c21c2aecc56)
    };
    for (size_t code_i = 0; code_i < code_indices.size(); ++code_i) {
        const int8_t* code = GetPreambleCode(code_indices[code_i]);
        BOOST_REQUIRE(code != nullptr);
        uint64_t hash = UINT64_C(1469598103934665603);
        size_t nnz = 0;
        for (size_t chip = 0; chip < kQm35CodeLength; ++chip) {
            BOOST_CHECK(code[chip] == -1 || code[chip] == 0 || code[chip] == 1);
            if (code[chip] != 0)
                ++nnz;
            hash ^= static_cast<uint8_t>(code[chip] + 1);
            hash *= UINT64_C(1099511628211);
        }
        BOOST_CHECK_EQUAL(nnz, size_t(64));
        BOOST_CHECK_EQUAL(hash, matlab_hashes[code_i]);
    }

    // First 30 output bits of MATLAB R2025b
    // lrwpan.internal.createScrambler(code, 30, 0), mapped 0 -> +1 and
    // 1 -> -1 by bprf_spreading. This validates code-11/12 data de-spreading,
    // not just their preamble sequences.
    const std::array<std::array<int8_t, 30>, 4> matlab_spreading = {{
        {{ 1, -1, 1, 1, 1, 1, -1, 1, 1, -1, -1, -1, -1, 1, -1,
           -1, -1, 1, 1, 1, -1, -1, 1, -1, 1, 1, 1, -1, -1, 1 }},
        {{ 1, 1, -1, -1, 1, 1, -1, 1, 1, 1, 1, -1, -1, -1, -1,
           1, -1, 1, -1, 1, -1, -1, 1, 1, 1, -1, 1, 1, 1, -1 }},
        {{ -1, -1, -1, -1, 1, 1, -1, 1, 1, -1, -1, 1, 1, -1, -1,
           1, 1, 1, -1, 1, -1, -1, 1, -1, 1, -1, 1, -1, 1, -1 }},
        {{ -1, 1, 1, -1, -1, -1, -1, -1, 1, 1, -1, -1, -1, 1, 1,
           -1, 1, -1, 1, 1, 1, 1, -1, 1, -1, 1, 1, -1, 1, -1 }}
    }};
    for (size_t code_i = 0; code_i < code_indices.size(); ++code_i) {
        std::vector<int8_t> spreading;
        BOOST_REQUIRE(core::detail::bprf_spreading(
            spreading, code_indices[code_i], 0, matlab_spreading[code_i].size()));
        BOOST_CHECK_EQUAL_COLLECTIONS(spreading.begin(), spreading.end(),
                                      matlab_spreading[code_i].begin(),
                                      matlab_spreading[code_i].end());
    }

    // Sampled code for code-10: non-zeros placed every SpreadingFactor chips.
    auto samp10 = BuildSampledCode(c10, kQm35CodeLength);
    BOOST_REQUIRE_EQUAL(samp10.size(), kQm35ChipsPerSymbol);
    size_t snz = 0, se = 0;
    for (int8_t v : samp10) {
        if (v != 0)
            ++snz;
        se += static_cast<size_t>(std::abs(v));
    }
    BOOST_CHECK_EQUAL(snz, size_t(64));
    BOOST_CHECK_EQUAL(se, size_t(64));

    // code-9 sampled path unchanged: first non-zero is +1 at index 0.
    auto samp9 = BuildSampledCode(c9, kQm35CodeLength);
    BOOST_CHECK_EQUAL(samp9[0], int8_t(1));
    BOOST_CHECK_EQUAL(static_cast<int>(c9[0]), 1);
    BOOST_CHECK_EQUAL(static_cast<int>(c9[7]), -1);

    // Cyclic origin lock: sparse-grid despread of the MATLAB code-10 SYNC
    // template must match kPreambleCode10 (roll-23 origin).  Sample phase 2
    // is the pulse peak of the lrwpan waveform at 998.4 MS/s.
    {
        std::vector<gr_complex> ref;
        BOOST_REQUIRE(
            load_cf32("../../../testdata/uwb_code10_preamble16_payload8.cfile",
                      ref));
        BOOST_REQUIRE_GE(ref.size(), kQm35SamplesPerSymbol);
        // DC-remove first symbol (same as the capture driver).
        gr_complex mean(0.0f, 0.0f);
        for (size_t i = 0; i < kQm35SamplesPerSymbol; ++i)
            mean += ref[i];
        mean /= static_cast<float>(kQm35SamplesPerSymbol);
        std::vector<float> chips(kQm35CodeLength, 0.0f);
        float e_chips = 0.0f;
        for (size_t c = 0; c < kQm35CodeLength; ++c) {
            const size_t idx = 2 + c * 8; // sample phase 2, SF=4, 2 samp/chip
            BOOST_REQUIRE_LT(idx, kQm35SamplesPerSymbol);
            const gr_complex v = ref[idx] - mean;
            chips[c] = v.real(); // waveform is essentially real after align
            e_chips += chips[c] * chips[c];
        }
        // Project to real after a single global phase from the complex grid.
        gr_complex gain(0.0f, 0.0f);
        for (size_t c = 0; c < kQm35CodeLength; ++c) {
            const size_t idx = 2 + c * 8;
            gain += (ref[idx] - mean) * static_cast<float>(c10[c]);
        }
        // Normalized correlation of phase-aligned chips vs code.
        float corr = 0.0f, e_code = 0.0f, e_rx = 0.0f;
        const float ang = std::arg(gain);
        const gr_complex rot(std::cos(-ang), std::sin(-ang));
        for (size_t c = 0; c < kQm35CodeLength; ++c) {
            const size_t idx = 2 + c * 8;
            const gr_complex v = (ref[idx] - mean) * rot;
            const float r = v.real();
            corr += r * static_cast<float>(c10[c]);
            e_rx += r * r;
            e_code += static_cast<float>(c10[c] * c10[c]);
        }
        const float norm_corr =
            corr / (std::sqrt(e_rx * e_code) + 1e-12f);
        // Wrong cyclic origin (pre-fix draft) scored ~0.64; correct is ~0.999.
        BOOST_CHECK_GT(norm_corr, 0.95f);
        BOOST_CHECK_EQUAL(static_cast<int>(c10[0]), 1);
        BOOST_CHECK_EQUAL(static_cast<int>(c10[1]), 1);
    }

    // Dw1000Profile factory: code_index=10, 64 SYNC, converts to Qm35825 layout.
    auto dw = Dw1000Profile::Default();
    BOOST_CHECK_EQUAL(dw.code_index, size_t(10));
    BOOST_CHECK_EQUAL(dw.preamble_repetitions, size_t(64));
    auto qm = dw.as_qm35825();
    BOOST_CHECK_EQUAL(qm.code_index, size_t(10));
    BOOST_CHECK_EQUAL(qm.preamble_repetitions, size_t(64));
    BOOST_CHECK_CLOSE(qm.sample_rate, kQm35SampleRate, 1e-9);
}

BOOST_AUTO_TEST_CASE(test_demod_core_r7_code10_matlab_waveform_phr)
{
    // Full frame generated by testdata/generate_uwb_comm_sample.m with
    // lrwpanWaveformGenerator: code 10, 16 SYNC, IEEE legacy SFD and an
    // 8-byte PSDU.  This locks the code-index-dependent BPM scrambler, not
    // merely the code-10 preamble cyclic origin.
    std::vector<gr_complex> iq;
    BOOST_REQUIRE(load_cf32(
        "../../../testdata/uwb_code10_preamble16_payload8.cfile", iq));
    BOOST_REQUIRE_GT(iq.size(), size_t(16 * kQm35SamplesPerSymbol));

    std::vector<gr_complex> tmpl(
        iq.begin(), iq.begin() + kQm35SamplesPerSymbol);
    // The generator ends exactly on the final payload sample.  Supply the
    // receive guard needed by the CIR matched filter, as a live PDU does.
    iq.resize(iq.size() + 128, gr_complex(0.0f, 0.0f));
    auto prof = Qm35825Profile::Default();
    prof.code_index = 10;
    prof.preamble_repetitions = 16;
    prof.sfd_mode = "ieee";
    prof.cir_skip_initial_repetitions = 0;
    prof.cir_repetitions = 16;

    core::DemodScratch scratch;
    scratch.reserve(iq.size());
    const auto result = core::demodulate_one(
        iq.data(), iq.size(), prof, 10, 0, 0, tmpl, scratch);
    BOOST_REQUIRE(result.timing.ok);
    BOOST_REQUIRE(result.sfd.ok);
    BOOST_REQUIRE(result.ns_sfd.ok);
    BOOST_REQUIRE(result.phr.ok);
    BOOST_CHECK(!result.phr.secded_uncorrectable);
    BOOST_CHECK_EQUAL(result.phr.psdu_length, uint32_t(8));
    BOOST_CHECK_CLOSE(result.phr.data_rate_mbps, 6.81f, 0.5);
    BOOST_REQUIRE(result.payload.ok);
    BOOST_REQUIRE_EQUAL(result.payload.bytes.size(), size_t(8));
    for (size_t i = 0; i < result.payload.bytes.size(); ++i)
        BOOST_CHECK_EQUAL(result.payload.bytes[i], static_cast<uint8_t>(i + 1));
}

BOOST_AUTO_TEST_CASE(test_demod_core_r7_long_preamble_cir_honors_explicit_skip)
{
    // MATLAB acceleration estimateCir uses [skip, skip+count) when an
    // explicit cir_skip_initial_repetitions is configured.  For a 256-SYNC
    // DW1000 frame this must not silently change to the final `count` SYNCs.
    auto prof = Qm35825Profile::Default();
    prof.code_index = 10;
    prof.preamble_repetitions = 256;
    prof.cir_skip_initial_repetitions = 10;
    prof.cir_repetitions = 64;
    prof.max_psdu_bytes = 1;

    constexpr int64_t start = 128;
    constexpr size_t late_delay = 12;
    const size_t n = static_cast<size_t>(start) +
                     prof.preamble_repetitions * kQm35SamplesPerSymbol +
                     48 * kQm35SamplesPerSymbol;
    std::vector<gr_complex> iq(n, gr_complex(0.0f, 0.0f));

    const int8_t* code = GetPreambleCode(10);
    const auto spread = BuildSampledCode(code, kQm35CodeLength);
    for (size_t rep = 0; rep < prof.preamble_repetitions; ++rep) {
        const size_t delay = (rep >= 10 && rep < 74) ? late_delay : 0;
        const size_t base = static_cast<size_t>(start) +
                            rep * kQm35SamplesPerSymbol + delay;
        for (size_t chip = 0; chip < spread.size(); ++chip) {
            const size_t sample = base + 2 * chip;
            BOOST_REQUIRE_LT(sample, iq.size());
            iq[sample] += gr_complex(static_cast<float>(spread[chip]), 0.0f);
        }
    }

    TimingResult timing;
    timing.ok = true;
    timing.preamble_start_sample = start;
    timing.measured_period = static_cast<double>(kQm35SamplesPerSymbol);
    timing.detected_peaks = prof.preamble_repetitions;

    core::DemodScratch scratch;
    scratch.reserve(iq.size());
    CirResult cir;
    std::vector<int8_t> pcode(code, code + kQm35CodeLength);
    BOOST_REQUIRE(core::stage_cir_softchips(iq.data(), iq.size(), prof, timing,
                                            pcode, cir, scratch));
    BOOST_CHECK_EQUAL(cir.first_path_sample,
                      static_cast<size_t>(start) + late_delay);
}

BOOST_AUTO_TEST_CASE(test_demod_core_r7_fractional_chip_grid_tracks_sfo)
{
    auto prof = Qm35825Profile::Default();
    prof.code_index = 10;
    prof.preamble_repetitions = 64;
    prof.sfd_mode = "decawave";
    prof.max_psdu_bytes = 1;

    constexpr int64_t start = 128;
    constexpr double period = 1016.25;
    const auto sfd = GetSfdSequence("decawave");
    const int8_t* code = GetPreambleCode(10);
    const auto spread = BuildSampledCode(code, kQm35CodeLength);
    const size_t symbol_count = prof.preamble_repetitions + sfd.size();
    const size_t n = static_cast<size_t>(start +
        std::ceil((symbol_count + 16) * period));
    std::vector<gr_complex> iq(n, gr_complex(0.0f, 0.0f));

    auto add_symbol = [&](size_t symbol, float sign) {
        const double base = static_cast<double>(start) + symbol * period;
        for (size_t chip = 0; chip < spread.size(); ++chip) {
            const float value = sign * static_cast<float>(spread[chip]);
            if (value == 0.0f)
                continue;
            const double pos = base + 2.0 * static_cast<double>(chip);
            const size_t i0 = static_cast<size_t>(std::floor(pos));
            const float frac = static_cast<float>(pos - std::floor(pos));
            BOOST_REQUIRE_LT(i0 + 1, iq.size());
            iq[i0] += gr_complex(value * (1.0f - frac), 0.0f);
            iq[i0 + 1] += gr_complex(value * frac, 0.0f);
        }
    };
    for (size_t rep = 0; rep < prof.preamble_repetitions; ++rep)
        add_symbol(rep, 1.0f);
    for (size_t k = 0; k < sfd.size(); ++k)
        add_symbol(prof.preamble_repetitions + k,
                   static_cast<float>(sfd[k]));

    TimingResult timing;
    timing.ok = true;
    timing.preamble_start_sample = start;
    timing.measured_period = period;
    timing.detected_peaks = prof.preamble_repetitions;

    core::DemodScratch scratch;
    scratch.reserve(iq.size());
    CirResult cir;
    std::vector<int8_t> pcode(code, code + kQm35CodeLength);
    BOOST_REQUIRE(core::stage_cir_softchips(iq.data(), iq.size(), prof, timing,
                                            pcode, cir, scratch));
    NsSfdResult ns;
    BOOST_REQUIRE(core::stage_ns_sfd(scratch.soft_chips, prof, sfd,
                                     kQm35ChipsPerSymbol, ns, scratch));
    BOOST_CHECK_GE(ns.metric, 0.75f);
}

BOOST_AUTO_TEST_CASE(test_demod_core_r7_robust_awgn_representative)
{
    // One representative AWGN point (20 dB, N=3 seeds) — full sweep lives in
    // benchmark_detector demod-robust.  Pass = Success + FCS 0x584b.
    std::vector<gr_complex> iq;
    BOOST_REQUIRE(load_cf32(golden_dir() + "/window.cfile", iq));
    double sp = 0.0;
    for (size_t i = 0; i < iq.size(); ++i)
        sp += static_cast<double>(std::norm(iq[i]));
    sp /= static_cast<double>(iq.size());
    const double snr_db = 20.0;
    const double sigma = std::sqrt(sp / std::pow(10.0, snr_db / 10.0) / 2.0);

    int pass = 0;
    for (unsigned seed = 0; seed < 3; ++seed) {
        std::mt19937 rng(1000u + seed);
        std::normal_distribution<float> g(0.0f, static_cast<float>(sigma));
        std::vector<gr_complex> iqn(iq.size());
        for (size_t i = 0; i < iq.size(); ++i)
            iqn[i] = iq[i] + gr_complex(g(rng), g(rng));
        auto res = demod_window_with_cfo(iqn, 0.0);
        if (res.status == DemodStatus::Success && res.payload.fcs_pass &&
            res.payload.received_fcs == uint16_t(0x584b))
            ++pass;
    }
    BOOST_CHECK_GE(pass, 2); // at least 2/3 at 20 dB
}

BOOST_AUTO_TEST_CASE(test_demod_core_rxopt_cfo_block_rotation_boundaries)
{
    // Experimental CFO kernel: 1024-sample blocks, non-multiple lengths and
    // positive/negative CFO.  Check against the independent double reference.
    std::vector<gr_complex> src(4097);
    for (size_t i = 0; i < src.size(); ++i) {
        src[i] = gr_complex(std::sin(0.013f * static_cast<float>(i)) + 0.2f,
                            std::cos(0.021f * static_cast<float>(i)) - 0.1f);
    }
    for (size_t n : { size_t(1000), size_t(1024), size_t(1025),
                      size_t(4097) }) {
        for (double cfo_hz : { 50000.0, -50000.0, 300000.0 }) {
            const double w = 2.0 * M_PI * cfo_hz / kQm35SampleRate;
            std::vector<gr_complex> got, ref(n);
            rxopt::derotate_block(got, src.data(), n, w);
            rxopt::derotate_reference(ref.data(), src.data(), n, w);
            double max_err = 0.0;
            for (size_t i = 0; i < n; ++i)
                max_err = std::max(max_err,
                                   static_cast<double>(std::abs(got[i] - ref[i])));
            BOOST_CHECK_LT(max_err, 2e-5);
            BOOST_CHECK_LT(std::abs(got[0] - src[0]), 1e-7f);
        }
    }
}

BOOST_AUTO_TEST_CASE(test_demod_core_r7_robust_multipath_representative)
{
    // One multipath point: gain 0.4, delay 120 samples — first packet must
    // still decode (FCS 0x584b).
    std::vector<gr_complex> iq;
    BOOST_REQUIRE(load_cf32(golden_dir() + "/window.cfile", iq));
    const float gain = 0.4f;
    const size_t delay = 120;
    std::vector<gr_complex> m(iq.size() + delay, gr_complex(0.0f, 0.0f));
    std::copy(iq.begin(), iq.end(), m.begin());
    for (size_t i = 0; i < iq.size(); ++i)
        m[i + delay] += gain * iq[i];

    auto prof = Qm35825Profile::Default();
    prof.sfd_mode = "ieee";
    core::DemodScratch scratch;
    scratch.reserve(m.size());
    auto res = core::demodulate_one(m.data(), m.size(), prof, 1, 9984, 0,
                                    load_reference_template(), scratch);
    BOOST_CHECK(res.status == DemodStatus::Success);
    BOOST_CHECK(res.payload.fcs_pass);
    BOOST_CHECK_EQUAL(res.payload.received_fcs, uint16_t(0x584b));
}

BOOST_AUTO_TEST_CASE(test_demod_core_r7_robust_collision_representative)
{
    // Late packet B preamble lands on A's SFD start (offset 0 symbols).
    // Demod of A must not crash; report FCS of A (may pass or fail).
    std::vector<gr_complex> iq;
    BOOST_REQUIRE(load_cf32(golden_dir() + "/window.cfile", iq));
    const size_t sfd_start = 75008;
    const size_t total = sfd_start + iq.size();
    std::vector<gr_complex> mix(total, gr_complex(0.0f, 0.0f));
    std::copy(iq.begin(), iq.end(), mix.begin());
    for (size_t i = 0; i < iq.size(); ++i)
        mix[sfd_start + i] += iq[i];

    auto prof = Qm35825Profile::Default();
    prof.sfd_mode = "ieee";
    core::DemodScratch scratch;
    scratch.reserve(mix.size());
    auto t0 = std::chrono::steady_clock::now();
    auto res = core::demodulate_one(mix.data(), mix.size(), prof, 1, 9984, 0,
                                    load_reference_template(), scratch);
    auto t1 = std::chrono::steady_clock::now();
    const double ms =
        std::chrono::duration<double, std::milli>(t1 - t0).count();
    // Graceful: no exception (we got here), bounded latency (< 100 ms).
    BOOST_CHECK_LT(ms, 100.0);
    // Status is one of the defined enum values (no crash / no hang).
    BOOST_CHECK(static_cast<int>(res.status) >= 0);
    BOOST_TEST_MESSAGE("collision offset0: status="
                       << static_cast<int>(res.status)
                       << " fcs=" << res.payload.fcs_pass << " latency_ms="
                       << ms);
}

BOOST_AUTO_TEST_CASE(test_demod_core_task_a1_cfo_range_absolute_phase_origin)
{
    // Task A1 direct numerical QA:
    // Decoupled CFO range rotation [lo, hi) where phase origin is strictly
    // sample 0 (i.e. sample index i has phase -w * i, not treating lo as 0).
    // Test non-aligned block boundaries and positive/negative CFOs.
    const size_t n = 8192;
    std::vector<gr_complex> src(n);
    for (size_t i = 0; i < n; ++i) {
        src[i] = gr_complex(std::sin(0.017f * static_cast<float>(i)) + 0.3f,
                            std::cos(0.023f * static_cast<float>(i)) - 0.2f);
    }

    const gr_complex sentinel(-999.0f, -999.0f);

    struct Interval {
        size_t lo;
        size_t hi;
    };
    const std::vector<Interval> intervals = {
        { 0, 1024 },
        { 137, 3519 },    // non-multiple start and end, spanning 4 blocks
        { 1024, 2048 },   // exactly 1 block
        { 1025, 1026 },   // single sample
        { 512, 7168 },    // large multi-block range
        { 2000, 2000 }    // empty interval
    };

    for (double cfo_hz : { 50000.0, -50000.0, 150000.0 }) {
        const double w = 2.0 * M_PI * cfo_hz / kQm35SampleRate;
        for (const auto& iv : intervals) {
            std::vector<gr_complex> dst_block(n, sentinel);
            std::vector<gr_complex> dst_recurrence(n, sentinel);
            std::vector<gr_complex> dst_scalar(n, sentinel);
            std::vector<gr_complex> ref(n, sentinel);

            rxopt::derotate_range_block(dst_block.data(), src.data(), n, iv.lo, iv.hi, w);
            rxopt::derotate_range_recurrence(dst_recurrence.data(), src.data(), n, iv.lo, iv.hi, w);
            rxopt::derotate_range_scalar(dst_scalar.data(), src.data(), n, iv.lo, iv.hi, w);
            rxopt::derotate_range_reference(ref.data(), src.data(), n, iv.lo, iv.hi, w);

            // 1. Outside [lo, hi), sentinel must be strictly preserved
            for (size_t i = 0; i < iv.lo; ++i) {
                BOOST_CHECK_EQUAL(dst_block[i], sentinel);
                BOOST_CHECK_EQUAL(dst_recurrence[i], sentinel);
                BOOST_CHECK_EQUAL(dst_scalar[i], sentinel);
            }
            for (size_t i = iv.hi; i < n; ++i) {
                BOOST_CHECK_EQUAL(dst_block[i], sentinel);
                BOOST_CHECK_EQUAL(dst_recurrence[i], sentinel);
                BOOST_CHECK_EQUAL(dst_scalar[i], sentinel);
            }

            // 2. Inside [lo, hi), verify against independent reference
            double max_err_block = 0.0;
            double max_err_recurrence = 0.0;
            double max_err_scalar = 0.0;
            for (size_t i = iv.lo; i < iv.hi; ++i) {
                max_err_block = std::max(max_err_block,
                    static_cast<double>(std::abs(dst_block[i] - ref[i])));
                max_err_recurrence = std::max(max_err_recurrence,
                    static_cast<double>(std::abs(dst_recurrence[i] - ref[i])));
                max_err_scalar = std::max(max_err_scalar,
                    static_cast<double>(std::abs(dst_scalar[i] - ref[i])));
            }
            BOOST_CHECK_LT(max_err_block, 2e-5);
            BOOST_CHECK_LT(max_err_recurrence, 2e-5);
            BOOST_CHECK_LT(max_err_scalar, 1e-6);

            // 3. Strict verification of phase origin = sample 0 (not lo!):
            // Sample lo must have phase -w * lo.
            if (iv.lo < iv.hi && iv.lo > 0 && std::abs(w * static_cast<double>(iv.lo)) > 0.1) {
                const double expected_phase = -w * static_cast<double>(iv.lo);
                const gr_complex expected_lo = src[iv.lo] *
                    gr_complex(static_cast<float>(std::cos(expected_phase)),
                               static_cast<float>(std::sin(expected_phase)));
                BOOST_CHECK_LT(std::abs(dst_block[iv.lo] - expected_lo), 2e-5f);
                BOOST_CHECK_LT(std::abs(dst_recurrence[iv.lo] - expected_lo), 2e-5f);
                // If lo were wrongly treated as phase 0, dst[lo] would equal src[lo]
                BOOST_CHECK_GT(std::abs(dst_block[iv.lo] - src[iv.lo]), 1e-3f);
                BOOST_CHECK_GT(std::abs(dst_recurrence[iv.lo] - src[iv.lo]), 1e-3f);
            }
        }
    }
}

BOOST_AUTO_TEST_CASE(test_demod_core_task_a2_cir_sparse_correlation_against_dense)
{
    // Task A2: CIR ternary template sparse correlation.
    // Verify that rxopt::correlate_cir_sparse matches dense correlation
    // across all HRP codes (9, 10, 11, 12).
    const size_t tap_count = 38;
    const size_t code_len = 2 * kQm35ChipsPerSymbol; // 1016
    const size_t wlen = code_len + tap_count - 1;

    std::vector<gr_complex> corr(wlen);
    for (size_t i = 0; i < wlen; ++i) {
        corr[i] = gr_complex(std::cos(0.031f * static_cast<float>(i)) + 0.1f,
                             std::sin(0.047f * static_cast<float>(i)) - 0.15f);
    }

    for (size_t code_idx : { 9, 10, 11, 12 }) {
        const int8_t* pc = GetPreambleCode(code_idx);
        const auto spread = BuildSampledCode(pc, kQm35CodeLength);

        // Dense reference implementation
        std::vector<gr_complex> dense_work(code_len, gr_complex(0.0f, 0.0f));
        float dense_energy = 0.0f;
        for (size_t j = 0; j < spread.size(); ++j) {
            dense_work[2 * j] = gr_complex(static_cast<float>(spread[j]), 0.0f);
            dense_energy += std::norm(dense_work[2 * j]);
        }
        std::vector<gr_complex> dense_values(tap_count);
        for (size_t nn = 0; nn < tap_count; ++nn) {
            gr_complex acc(0.0f, 0.0f);
            for (size_t m = 0; m < code_len; ++m) {
                acc += std::conj(dense_work[m]) * corr[nn + m];
            }
            dense_values[nn] = acc / dense_energy;
        }

        // Sparse implementation
        rxopt::SparseCirTemplate sparse_tmpl;
        rxopt::build_sparse_cir_template(pc, kQm35CodeLength, sparse_tmpl);
        BOOST_CHECK_EQUAL(sparse_tmpl.code_energy, dense_energy);
        BOOST_CHECK_GT(sparse_tmpl.entries.size(), 0u);
        BOOST_CHECK_LE(sparse_tmpl.entries.size(), 127u); // ternary code length <= 127 (e.g. 64 non-zeros)

        std::vector<gr_complex> sparse_values;
        rxopt::correlate_cir_sparse(corr.data(), sparse_tmpl, tap_count, sparse_values);

        BOOST_REQUIRE_EQUAL(sparse_values.size(), dense_values.size());
        double max_diff = 0.0;
        for (size_t nn = 0; nn < tap_count; ++nn) {
            max_diff = std::max(max_diff,
                static_cast<double>(std::abs(sparse_values[nn] - dense_values[nn])));
        }
        // Arithmetic is identical additions/subtractions in identical order
        BOOST_CHECK_LT(max_diff, 1e-6);
    }
}

BOOST_AUTO_TEST_CASE(test_demod_core_task_a1_a2_full_pipeline_range_cfo)
{
    // End-to-end full packet decode with range CFO compensation and sparse CIR:
    // Tests fast CFO rotation, on-demand soft-chips, and fused payload CMF.
    std::vector<gr_complex> iq;
    BOOST_REQUIRE(load_cf32(golden_dir() + "/window.cfile", iq));

    for (double cfo_hz : { 0.0, 25000.0, -25000.0 }) {
        std::vector<gr_complex> rx(iq.size());
        const double w = 2.0 * M_PI * cfo_hz / kQm35SampleRate;
        for (size_t i = 0; i < iq.size(); ++i) {
            const double phase = w * static_cast<double>(i);
            const float c = static_cast<float>(std::cos(phase));
            const float s = static_cast<float>(std::sin(phase));
            rx[i] = iq[i] * gr_complex(c, s);
        }

        auto prof = Qm35825Profile::Default();
        prof.sfd_mode = "ieee";
        prof.rx_fast_sfd = true;
        prof.rx_fast_cfo_rotation = true;
        prof.rx_softchip_on_demand = true;
        prof.rx_fused_payload_cmf = true;
        prof.rx_fused_phr_cmf = true;

        core::DemodScratch scratch;
        scratch.reserve(rx.size());
        auto res = core::demodulate_one(rx.data(), rx.size(), prof, 1, 9984, 0,
                                        load_reference_template(), scratch);
        BOOST_CHECK(res.status == DemodStatus::Success);
        BOOST_CHECK(res.payload.fcs_pass);
        BOOST_CHECK_EQUAL(res.payload.received_fcs, uint16_t(0x584b));
        BOOST_CHECK_CLOSE(res.cfo.cfo_hz, cfo_hz, 200.0); // within 200 Hz
        BOOST_CHECK_GT(res.stage_cfo_rotate_us, 0u);
        BOOST_CHECK_GT(res.stage_cfo_rotated_samples, 0u);
        BOOST_CHECK_GE(res.stage_cfo_rotation_passes, 2u);
    }
}

BOOST_AUTO_TEST_CASE(test_demod_core_default_rx_recurrence)
{
    // Verify that default RX configuration (rx_fast_cfo_rotation = false)
    // succeeds using the fast 4-chain recurrence kernel.
    std::vector<gr_complex> iq;
    BOOST_REQUIRE(load_cf32(golden_dir() + "/window.cfile", iq));

    const double cfo_hz = 30000.0;
    std::vector<gr_complex> rx(iq.size());
    const double w = 2.0 * M_PI * cfo_hz / kQm35SampleRate;
    for (size_t i = 0; i < iq.size(); ++i) {
        const double phase = w * static_cast<double>(i);
        rx[i] = iq[i] * gr_complex(static_cast<float>(std::cos(phase)),
                                   static_cast<float>(std::sin(phase)));
    }

    auto prof = Qm35825Profile::Default();
    prof.sfd_mode = "ieee";
    prof.rx_fast_cfo_rotation = false; // test recurrence path
    prof.rx_fast_sfd = true;

    core::DemodScratch scratch;
    scratch.reserve(rx.size());
    auto res = core::demodulate_one(rx.data(), rx.size(), prof, 1, 9984, 0,
                                    load_reference_template(), scratch);
    BOOST_CHECK(res.status == DemodStatus::Success);
    BOOST_CHECK(res.payload.fcs_pass);
    BOOST_CHECK_EQUAL(res.cfo.rotation_kernel, 0u);
    BOOST_CHECK_CLOSE(res.cfo.cfo_hz, cfo_hz, 200.0);
    BOOST_CHECK_GT(res.stage_cfo_rotated_samples, 0u);
}

BOOST_AUTO_TEST_CASE(test_demod_core_softchip_fallback_timing_accounting)
{
    // Test that on softchip fallback retry:
    // 1. Fallback rotation is included in stage_softchip_fallback_us and stage_total_us
    // 2. Full window is derotated and counted
    std::vector<gr_complex> iq;
    BOOST_REQUIRE(load_cf32(golden_dir() + "/window.cfile", iq));

    // Corrupt payload region so fused CMF fails FCS and forces fallback retry
    std::vector<gr_complex> rx = iq;
    // Preamble + SFD + PHR is ~104000 samples, corrupt payload after sample 110000
    for (size_t i = 110000; i < rx.size(); ++i) {
        rx[i] = gr_complex(0.0f, 0.0f);
    }

    auto prof = Qm35825Profile::Default();
    prof.sfd_mode = "ieee";
    prof.rx_fast_sfd = true;
    prof.rx_fast_cfo_rotation = true;
    prof.rx_softchip_on_demand = true;
    prof.rx_fused_payload_cmf = true;
    prof.rx_fused_phr_cmf = true;

    core::DemodScratch scratch;
    scratch.reserve(rx.size());
    auto res = core::demodulate_one(rx.data(), rx.size(), prof, 1, 9984, 0,
                                    load_reference_template(), scratch);
    // Fallback must have been triggered
    BOOST_CHECK_EQUAL(res.stage_softchip_fallbacks, 1u);
    BOOST_CHECK_GT(res.stage_softchip_fallback_us, 0u);
    // stage_total_us must be at least stage_softchip_fallback_us
    BOOST_CHECK_GE(res.stage_total_us, res.stage_softchip_fallback_us);
    // Full window was rotated in fallback
    BOOST_CHECK_GE(res.stage_cfo_rotated_samples, rx.size());
}

BOOST_AUTO_TEST_CASE(test_demod_core_payload_substage_timers_partition_parent)
{
    // The payload sub-stage timers (soft / viterbi / rs / crc / other) are
    // diagnostic CHILDREN of stage_payload_us.  `other` is the exact remainder,
    // so the five buckets must sum to the in-function payload interval and can
    // never exceed the parent lap.  The only allowed difference is the
    // call/lap overhead (a few µs at 1 µs clock resolution).
    std::vector<gr_complex> iq;
    BOOST_REQUIRE(load_cf32(golden_dir() + "/window.cfile", iq));

    auto check_partition = [&](const DemodResult& res, const char* label) {
        BOOST_TEST_CONTEXT(label) {
            const uint64_t sum = res.stage_payload_soft_us +
                                 res.stage_payload_viterbi_us +
                                 res.stage_payload_rs_us +
                                 res.stage_payload_crc_us +
                                 res.stage_payload_other_us;
            BOOST_TEST_MESSAGE(label << ": payload=" << res.stage_payload_us
                                     << " soft=" << res.stage_payload_soft_us
                                     << " vit=" << res.stage_payload_viterbi_us
                                     << " rs=" << res.stage_payload_rs_us
                                     << " crc=" << res.stage_payload_crc_us
                                     << " other=" << res.stage_payload_other_us
                                     << " sum=" << sum
                                     << " fused_calls="
                                     << res.stage_payload_fused_calls
                                     << " legacy_calls="
                                     << res.stage_payload_legacy_calls);
            // Decoded successfully, so the payload stage really ran.
            BOOST_REQUIRE(res.status == DemodStatus::Success);
            BOOST_REQUIRE(res.payload.fcs_pass);
            BOOST_REQUIRE_GE(res.stage_payload_fused_calls +
                                 res.stage_payload_legacy_calls,
                             1u);
            // Children never exceed the parent lap.
            BOOST_CHECK_LE(sum, res.stage_payload_us);
            // ... and the gap is only call overhead, not a missing bucket.
            BOOST_CHECK_LE(res.stage_payload_us - sum, 20u);
            BOOST_CHECK_GT(sum, 0u);
            // The three dominant buckets must be attributed, not dumped into
            // `other` (µs truncation keeps them strictly positive here).
            BOOST_CHECK_GT(res.stage_payload_soft_us, 0u);
            BOOST_CHECK_GT(res.stage_payload_viterbi_us, 0u);
            BOOST_CHECK_GT(res.stage_payload_rs_us + res.stage_payload_crc_us,
                           0u);
        }
    };

    // Legacy payload backend.
    {
        auto prof = Qm35825Profile::Default();
        prof.sfd_mode = "ieee";
        prof.rx_fast_sfd = true;
        prof.rx_fused_payload_cmf = false;
        prof.rx_fused_phr_cmf = false;
        prof.rx_softchip_on_demand = false;
        core::DemodScratch scratch;
        scratch.reserve(iq.size());
        auto res = core::demodulate_one(iq.data(), iq.size(), prof, 1, 9984, 0,
                                        load_reference_template(), scratch);
        check_partition(res, "legacy");
        BOOST_CHECK_GE(res.stage_payload_legacy_calls, 1u);
        BOOST_CHECK_EQUAL(res.stage_payload_fused_calls, 0u);
    }

    // Fused CMF payload backend (the live default).
    {
        auto prof = Qm35825Profile::Default();
        prof.sfd_mode = "ieee";
        prof.rx_fast_sfd = true;
        prof.rx_fast_cfo_rotation = true;
        prof.rx_softchip_on_demand = true;
        prof.rx_fused_payload_cmf = true;
        prof.rx_fused_phr_cmf = true;
        core::DemodScratch scratch;
        scratch.reserve(iq.size());
        auto res = core::demodulate_one(iq.data(), iq.size(), prof, 1, 9984, 0,
                                        load_reference_template(), scratch);
        check_partition(res, "fused");
        BOOST_CHECK_EQUAL(res.stage_softchip_fallbacks, 0u);
        BOOST_CHECK_GE(res.stage_payload_fused_calls, 1u);
    }
}

BOOST_AUTO_TEST_CASE(test_demod_core_real_sync_dot_numeric_equivalence)
{
    const size_t L = 1016;
    std::vector<gr_complex> tmpl = load_reference_template();
    BOOST_REQUIRE_EQUAL(tmpl.size(), L);

    rxopt::RealSyncTemplate real_tmpl;
    rxopt::build_real_sync_template(tmpl, real_tmpl);
    BOOST_CHECK(real_tmpl.is_real);
    BOOST_CHECK_EQUAL(real_tmpl.length, L);
    BOOST_CHECK_GT(real_tmpl.energy, 0.0f);

    // Generate test rx signal with synthetic multipath + noise
    std::vector<gr_complex> rx(L + 128);
    for (size_t i = 0; i < rx.size(); ++i) {
        rx[i] = gr_complex(std::sin(static_cast<float>(i) * 0.13f) + 0.1f * std::cos(static_cast<float>(i) * 0.37f),
                           std::cos(static_cast<float>(i) * 0.17f) - 0.2f * std::sin(static_cast<float>(i) * 0.29f));
    }

    // Verify 4-lag AVX2 dot vs VOLK across 64 candidate offsets
    for (size_t lag = 0; lag < 64; lag += 4) {
        gr_complex v_out[4];
        for (size_t j = 0; j < 4; ++j) {
            v_out[j] = rxopt::conjugate_dot(rx.data() + lag + j, tmpl.data(), L);
        }

        gr_complex a_out[4];
        rxopt::real_template_dot_4lag(rx.data() + lag, real_tmpl.taps_dup.data(), L, a_out);

        for (size_t j = 0; j < 4; ++j) {
            const float diff = std::abs(v_out[j] - a_out[j]);
            BOOST_CHECK_LT(diff, 1e-5f);

            // Also test single lag dot
            const gr_complex s_out = rxopt::real_template_dot_single(
                rx.data() + lag + j, real_tmpl.taps_dup.data(), L);
            const float s_diff = std::abs(v_out[j] - s_out);
            BOOST_CHECK_LT(s_diff, 1e-5f);
        }
    }
}

BOOST_AUTO_TEST_CASE(test_demod_core_sfd_dot_numeric_equivalence)
{
    const size_t sym = kQm35SamplesPerSymbol; // 1016
    std::vector<gr_complex> tmpl = load_reference_template();
    BOOST_REQUIRE_EQUAL(tmpl.size(), sym);

    rxopt::RealSyncTemplate real_tmpl;
    rxopt::build_real_sync_template(tmpl, real_tmpl);
    BOOST_REQUIRE(real_tmpl.is_real);

    // Test with IEEE SFD and SFD 4z2
    const char* modes[] = { "ieee", "4z2" };
    for (const char* mode : modes) {
        const std::vector<int8_t> sfd_seq = openuwb::demod::GetSfdSequence(mode);
        const size_t len = sfd_seq.size();
        BOOST_REQUIRE_GT(len, 0);

        float sfd_norm_sq = 0.0f;
        for (int8_t s : sfd_seq)
            sfd_norm_sq += static_cast<float>(s * s);
        BOOST_REQUIRE_GT(sfd_norm_sq, 0.0f);
        const float sfd_scale = 1.0f / std::sqrt(sfd_norm_sq * real_tmpl.energy);

        // Generate synthetic test rx signal covering full SFD plus margin
        const size_t total_sfd_samples = len * sym;
        std::vector<gr_complex> rx(total_sfd_samples + 256);
        for (size_t i = 0; i < rx.size(); ++i) {
            rx[i] = gr_complex(
                std::sin(static_cast<float>(i) * 0.07f) + 0.3f * std::cos(static_cast<float>(i) * 0.23f),
                std::cos(static_cast<float>(i) * 0.11f) - 0.2f * std::sin(static_cast<float>(i) * 0.19f));
        }

        // Test over 64 candidate offsets, testing both 4-lag batch and single-lag
        for (size_t lag = 0; lag < 64; lag += 4) {
            gr_complex a4[4];
            rxopt::sfd_real_dot_4lag(rx.data() + lag,
                                     real_tmpl.taps_dup.data(),
                                     sym,
                                     sfd_seq.data(),
                                     len,
                                     sfd_scale,
                                     a4);

            for (size_t j = 0; j < 4; ++j) {
                const gr_complex s_out = rxopt::sfd_real_dot_single(
                    rx.data() + lag + j,
                    real_tmpl.taps_dup.data(),
                    sym,
                    sfd_seq.data(),
                    len,
                    sfd_scale);

                const float diff = std::abs(a4[j] - s_out);
                const float mag = std::abs(s_out);
                const float rel_err = (mag > 1e-12f) ? (diff / mag) : diff;
                BOOST_CHECK_LT(rel_err, 2e-5f);
                BOOST_CHECK_LT(diff, 2e-5f);
            }
        }
    }
}

BOOST_AUTO_TEST_CASE(test_demod_core_cached_sequence_invalidation)
{
    core::DemodScratch scratch;
    // Initially invalid
    BOOST_CHECK_EQUAL(scratch.cached_seq.code_index, std::numeric_limits<size_t>::max());
    BOOST_CHECK(scratch.cached_seq.pcode.empty());
    BOOST_CHECK(scratch.cached_seq.sampled_pcode.empty());
    BOOST_CHECK(scratch.cached_seq.sfd_seq.empty());

    // Update with code 9, "4z2"
    scratch.cached_seq.update(9, "4z2");
    BOOST_CHECK_EQUAL(scratch.cached_seq.code_index, 9);
    BOOST_CHECK_EQUAL(scratch.cached_seq.sfd_mode, "4z2");
    BOOST_CHECK_EQUAL(scratch.cached_seq.pcode.size(), kQm35CodeLength);
    const int8_t* pc9 = GetPreambleCode(9);
    for (size_t i = 0; i < kQm35CodeLength; ++i) {
        BOOST_CHECK_EQUAL(scratch.cached_seq.pcode[i], pc9[i]);
    }
    const std::vector<int8_t> expected_sampled9 = BuildSampledCode(pc9, kQm35CodeLength);
    BOOST_CHECK_EQUAL(scratch.cached_seq.sampled_pcode.size(), expected_sampled9.size());
    for (size_t i = 0; i < expected_sampled9.size(); ++i) {
        BOOST_CHECK_EQUAL(scratch.cached_seq.sampled_pcode[i], expected_sampled9[i]);
    }
    const std::vector<int8_t> expected_sfd4z2 = openuwb::demod::GetSfdSequence("4z2");
    BOOST_CHECK_EQUAL(scratch.cached_seq.sfd_seq.size(), expected_sfd4z2.size());
    for (size_t i = 0; i < expected_sfd4z2.size(); ++i) {
        BOOST_CHECK_EQUAL(scratch.cached_seq.sfd_seq[i], expected_sfd4z2[i]);
    }

    // Repeated update with same parameters should not reallocate/change
    const int8_t* pcode_ptr = scratch.cached_seq.pcode.data();
    scratch.cached_seq.update(9, "4z2");
    BOOST_CHECK_EQUAL(scratch.cached_seq.pcode.data(), pcode_ptr);

    // Update with code 10 (code change, SFD mode unchanged)
    scratch.cached_seq.update(10, "4z2");
    BOOST_CHECK_EQUAL(scratch.cached_seq.code_index, 10);
    const int8_t* pc10 = GetPreambleCode(10);
    BOOST_CHECK_EQUAL(scratch.cached_seq.pcode[0], pc10[0]);
    const std::vector<int8_t> expected_sampled10 = BuildSampledCode(pc10, kQm35CodeLength);
    BOOST_CHECK_EQUAL(scratch.cached_seq.sampled_pcode[0], expected_sampled10[0]);

    // Update with "ieee" (SFD mode change, code unchanged)
    scratch.cached_seq.update(10, "ieee");
    BOOST_CHECK_EQUAL(scratch.cached_seq.sfd_mode, "ieee");
    const std::vector<int8_t> expected_ieee = openuwb::demod::GetSfdSequence("ieee");
    BOOST_CHECK_EQUAL(scratch.cached_seq.sfd_seq.size(), expected_ieee.size());
    for (size_t i = 0; i < expected_ieee.size(); ++i) {
        BOOST_CHECK_EQUAL(scratch.cached_seq.sfd_seq[i], expected_ieee[i]);
    }

    // Explicit invalidate
    scratch.cached_seq.invalidate();
    BOOST_CHECK_EQUAL(scratch.cached_seq.code_index, std::numeric_limits<size_t>::max());
    BOOST_CHECK(scratch.cached_seq.pcode.empty());
    BOOST_CHECK(scratch.cached_seq.sampled_pcode.empty());
    BOOST_CHECK(scratch.cached_seq.sfd_seq.empty());
}

BOOST_AUTO_TEST_CASE(test_demod_core_vector_accumulate_scale)
{
    const size_t N = 1053;
    std::vector<gr_complex> dst_simd(N, gr_complex(0.0f, 0.0f));
    std::vector<gr_complex> dst_scalar(N, gr_complex(0.0f, 0.0f));

    // Test accumulation of 16 blocks of random-ish data
    for (size_t b = 0; b < 16; ++b) {
        std::vector<gr_complex> src(N);
        for (size_t i = 0; i < N; ++i) {
            src[i] = gr_complex(std::sin(static_cast<float>(b * N + i) * 0.1f),
                                std::cos(static_cast<float>(b * N + i) * 0.17f));
            dst_scalar[i] += src[i];
        }
        rxopt::vector_accumulate_complex(dst_simd.data(), src.data(), N);
    }

    for (size_t i = 0; i < N; ++i) {
        BOOST_CHECK_CLOSE(dst_simd[i].real(), dst_scalar[i].real(), 1e-4f);
        BOOST_CHECK_CLOSE(dst_simd[i].imag(), dst_scalar[i].imag(), 1e-4f);
    }

    const float scale = 1.0f / 16.0f;
    for (size_t i = 0; i < N; ++i)
        dst_scalar[i] *= scale;
    rxopt::vector_scale_complex(dst_simd.data(), scale, N);

    for (size_t i = 0; i < N; ++i) {
        BOOST_CHECK_CLOSE(dst_simd[i].real(), dst_scalar[i].real(), 1e-4f);
        BOOST_CHECK_CLOSE(dst_simd[i].imag(), dst_scalar[i].imag(), 1e-4f);
    }
}

BOOST_AUTO_TEST_CASE(test_demod_core_sfd_fold_window_equivalence)
{
    const size_t sym = kQm35SamplesPerSymbol; // 1016
    const auto sfd_seq = openuwb::demod::GetSfdSequence("4z2");
    const size_t len = sfd_seq.size(); // 8

    std::vector<gr_complex> tmpl(sym);
    for (size_t i = 0; i < sym; ++i) {
        tmpl[i] = gr_complex(std::cos(static_cast<float>(i) * 0.12f), 0.0f);
    }
    rxopt::RealSyncTemplate real_tmpl;
    rxopt::build_real_sync_template(tmpl, real_tmpl);

    float sfd_norm_sq = 0.0f;
    for (int8_t s : sfd_seq) sfd_norm_sq += static_cast<float>(s * s);
    const float sfd_scale = 1.0f / std::sqrt(sfd_norm_sq * real_tmpl.energy);

    // Synthetic rx of length 16 * sym
    const size_t rx_len = 16 * sym;
    std::vector<gr_complex> rx(rx_len);
    for (size_t i = 0; i < rx_len; ++i) {
        rx[i] = gr_complex(std::sin(static_cast<float>(i) * 0.05f),
                           std::cos(static_cast<float>(i) * 0.07f));
    }

    const size_t j_lo = 100;
    const size_t j_end = 228;
    const size_t fold_len = (j_end - j_lo) + sym;
    std::vector<gr_complex> folded(fold_len);
    rxopt::fold_sfd_window(rx.data() + j_lo, sym, sfd_seq.data(), len, fold_len, folded.data());

    for (size_t j = j_lo; j <= j_end; j += 7) {
        const gr_complex dot_direct = rxopt::sfd_real_dot_single(
            rx.data() + j, real_tmpl.taps_dup.data(), sym, sfd_seq.data(), len, sfd_scale);
        const gr_complex dot_folded = rxopt::real_template_dot_single(
            folded.data() + (j - j_lo), real_tmpl.taps_dup.data(), sym) * sfd_scale;

        BOOST_CHECK_SMALL(std::abs(dot_direct.real() - dot_folded.real()), 1e-5f);
        BOOST_CHECK_SMALL(std::abs(dot_direct.imag() - dot_folded.imag()), 1e-5f);
    }
}

BOOST_AUTO_TEST_CASE(test_demod_core_ns_sfd_sparse_sliding_equivalence)
{
    auto prof = Qm35825Profile::Default();
    prof.code_index = 10;
    prof.sfd_mode = "4z2";

    const auto sfd_seq = openuwb::demod::GetSfdSequence(prof.sfd_mode);
    const size_t total_chips = 80 * kQm35ChipsPerSymbol;

    std::vector<float> soft_chips(total_chips, 0.0f);
    const int64_t target_start = 64 * kQm35ChipsPerSymbol + 2;

    core::DemodScratch scratch;
    scratch.cached_seq.update(prof.code_index, prof.sfd_mode);
    const auto& sc = scratch.cached_seq.sampled_pcode;

    // Inject target NS-SFD pattern with polarity +1
    for (size_t i = 0; i < sfd_seq.size(); ++i) {
        const int8_t s = sfd_seq[i];
        for (size_t j = 0; j < sc.size(); ++j) {
            soft_chips[target_start + i * sc.size() + j] = static_cast<float>(s * sc[j]);
        }
    }

    NsSfdResult res;
    BOOST_REQUIRE(core::stage_ns_sfd(soft_chips, prof, sfd_seq, kQm35ChipsPerSymbol, res, scratch));
    BOOST_CHECK(res.ok);
    BOOST_CHECK_EQUAL(res.sfd_start_chip, target_start);
    BOOST_CHECK_EQUAL(res.polarity, 1);
    BOOST_CHECK_GE(res.metric, 0.95f);
}

BOOST_AUTO_TEST_CASE(test_demod_core_tail_reuse_equivalence_and_fallback)
{
    std::vector<gr_complex> iq, tmpl;
    BOOST_REQUIRE(load_cf32(golden_dir() + "/window.cfile", iq));
    BOOST_REQUIRE(load_cf32("../../../testdata/reference_preamble.bin", tmpl));

    // 1. Nominal reuse run: default profile with rx_seed_refine_tail = true (narrow search = 8).
    // All 5 admission conditions are satisfied, so run_tail_and_cfo_refit executes tail reuse.
    core::DemodScratch scratch_reuse;
    scratch_reuse.reserve(iq.size());
    auto prof_reuse = Qm35825Profile::Default();
    prof_reuse.sfd_mode = "ieee";
    prof_reuse.rx_seed_refine_tail = true;
    auto res_reuse = core::demodulate_one(iq.data(), iq.size(), prof_reuse, 1, 9984, 0, tmpl, scratch_reuse);
    BOOST_CHECK(res_reuse.status == DemodStatus::Success);

    // 2. Explicit full re-search reference run: set rx_seed_refine_tail = false (seed_radius = 0).
    // This forces run_tail_and_cfo_refit to bypass can_reuse and execute full stage_timing.
    core::DemodScratch scratch_legacy;
    scratch_legacy.reserve(iq.size());
    auto prof_legacy = Qm35825Profile::Default();
    prof_legacy.sfd_mode = "ieee";
    prof_legacy.rx_seed_refine_tail = false;
    auto res_legacy = core::demodulate_one(iq.data(), iq.size(), prof_legacy, 1, 9984, 0, tmpl, scratch_legacy);
    BOOST_CHECK(res_legacy.status == DemodStatus::Success);

    // Equivalence checks between reuse and explicit re-search
    BOOST_CHECK_EQUAL(res_reuse.timing.detected_peaks, res_legacy.timing.detected_peaks);
    BOOST_CHECK_EQUAL(res_reuse.timing.peak_samples.size(), res_legacy.timing.peak_samples.size());
    BOOST_REQUIRE_GE(res_reuse.timing.peak_samples.size(), size_t(64));
    // Verify that the tail 40 peaks in reuse exactly match the explicit tracking
    for (size_t k = 24; k < 64; ++k) {
        BOOST_CHECK_EQUAL(res_reuse.timing.peak_samples[k], res_legacy.timing.peak_samples[k]);
    }
    // Estimated CFO must match closely (within < 15 Hz)
    BOOST_CHECK_SMALL(std::abs(res_reuse.cfo.cfo_hz - res_legacy.cfo.cfo_hz), 15.0);
    BOOST_CHECK_EQUAL(res_reuse.phr.psdu_length, res_legacy.phr.psdu_length);

    // 3. Fallback test: violate an admission condition.
    // Setting timing_track_radius = 12 (not 8) causes can_reuse == false.
    core::DemodScratch scratch_fb;
    scratch_fb.reserve(iq.size());
    auto prof_fb = Qm35825Profile::Default();
    prof_fb.sfd_mode = "ieee";
    prof_fb.rx_seed_refine_tail = true;
    prof_fb.timing_track_radius = 12; // Incompatible with radius 8 reuse condition
    auto res_fb = core::demodulate_one(iq.data(), iq.size(), prof_fb, 1, 9984, 0, tmpl, scratch_fb);
    BOOST_CHECK(res_fb.status == DemodStatus::Success);
    BOOST_CHECK_EQUAL(res_fb.timing.detected_peaks, size_t(64));
    BOOST_CHECK_SMALL(std::abs(res_fb.cfo.cfo_hz - res_legacy.cfo.cfo_hz), 25.0);
}

BOOST_AUTO_TEST_CASE(test_demod_core_sfd_tail_cfo_uses_only_measured_peaks)
{
    // Keep the golden SFD/PHR/payload and vary only the SYNC train length.
    // The full-frame timing grid deliberately still contains zero placeholders;
    // those placeholders must never participate in the final CFO phase fit.
    std::vector<gr_complex> golden;
    BOOST_REQUIRE(load_cf32(golden_dir() + "/window.cfile", golden));
    const auto tmpl = load_reference_template();
    constexpr size_t start = 9984;
    constexpr size_t sym = kQm35SamplesPerSymbol;
    constexpr size_t original_reps = 64;
    const size_t sfd = start + original_reps * sym;
    BOOST_REQUIRE_GT(golden.size(), sfd);
    const std::vector<gr_complex> stable_symbol(
        golden.begin() + sfd - sym, golden.begin() + sfd);

    for (size_t reps : {size_t(16), size_t(32), size_t(64), size_t(128), size_t(256)}) {
        std::vector<gr_complex> base(golden.begin(), golden.begin() + start);
        if (reps > original_reps) {
            for (size_t k = original_reps; k < reps; ++k)
                base.insert(base.end(), stable_symbol.begin(), stable_symbol.end());
            base.insert(base.end(), golden.begin() + start, golden.end());
        } else {
            base.insert(base.end(), golden.begin() + sfd - reps * sym, golden.end());
        }
        for (double hz : {0.0, -25000.0, 25000.0, -175000.0, 175000.0}) {
            const double rate = 1.0 + hz / Qm35825Profile::Default().carrier_freq_hz;
            std::vector<gr_complex> rx(base.size(), gr_complex(0.0f, 0.0f));
            for (size_t k = 0; k < base.size(); ++k) {
                const double x = static_cast<double>(k) * rate;
                const size_t i0 = static_cast<size_t>(x);
                gr_complex s(0.0f, 0.0f);
                if (i0 + 1 < base.size()) {
                    const double frac = x - static_cast<double>(i0);
                    s = base[i0] * static_cast<float>(1.0 - frac) +
                        base[i0 + 1] * static_cast<float>(frac);
                } else if (i0 < base.size()) {
                    s = base[i0];
                }
                const double phase = 2.0 * M_PI * hz * k / kQm35SampleRate;
                rx[k] = s * gr_complex(std::cos(phase), std::sin(phase));
            }
            // Exercise both explicit tail re-tracking and the peak-reuse path.
            for (bool reuse : {false, true}) {
                BOOST_TEST_CONTEXT("reps=" << reps << " hz=" << hz << " reuse=" << reuse) {
                    auto p = Qm35825Profile::Default();
                    p.preamble_repetitions = reps;
                    p.cir_repetitions = reps - p.cir_skip_initial_repetitions;
                    p.sfd_mode = "ieee";
                    p.rx_seed_refine_tail = reuse;
                    core::DemodScratch scratch;
                    auto r = core::demodulate_one(rx.data(), rx.size(), p, 0,
                                                  start, 0, tmpl, scratch);
                    BOOST_REQUIRE(r.status == DemodStatus::Success);
                    BOOST_REQUIRE(r.payload.fcs_pass);
                    BOOST_CHECK_EQUAL(r.payload.received_fcs, uint16_t(0x584b));
                    const size_t measured = std::min<size_t>(reps, 40);
                    const size_t first = reps - measured;
                    BOOST_CHECK_EQUAL(r.cfo.peaks_used, measured);
                    BOOST_CHECK_EQUAL(r.cfo.skipped_peaks, first);
                    BOOST_CHECK_EQUAL(r.cfo.fit_first_peak_sample, r.timing.peak_samples[first]);
                    BOOST_CHECK_EQUAL(r.cfo.fit_last_peak_sample, r.timing.peak_samples.back());
                    BOOST_CHECK_SMALL(r.cfo.cfo_hz - hz, 50.0);
                    for (size_t k = 0; k < first; ++k)
                        BOOST_CHECK_EQUAL(std::abs(r.timing.peak_corr[k]), 0.0f);
                    for (size_t k = first; k < reps; ++k)
                        BOOST_CHECK_GT(std::abs(r.timing.peak_corr[k]), 0.0f);
                }
            }
        }
    }
}

BOOST_AUTO_TEST_CASE(test_demod_core_soft_trace_viterbi_matches_plain)
{
    // decode_payload_soft() now runs ONE tracing soft Viterbi and reuses its
    // bits for the first RS/CRC attempt, instead of running the plain Viterbi
    // and then recomputing the whole thing with margins inside the rescue.
    // That is only sound if vitdec_cl3_soft_trace()'s surviving path is
    // bit-identical to vitdec_cl3_soft()'s, so pin it over random LLRs.
    std::mt19937 rng(20261117u);
    std::normal_distribution<float> nd(0.0f, 1.0f);
    const size_t lengths[] = { 64, 257, 1229 };
    for (size_t n : lengths) {
        std::vector<float> l0(n), l1(n);
        for (size_t i = 0; i < n; ++i) {
            l0[i] = nd(rng);
            l1[i] = nd(rng);
        }
        std::vector<int8_t> plain;
        openuwb::demod::core::detail::vitdec_cl3_soft(l0.data(), l1.data(), n, plain);
        openuwb::demod::core::detail::Cl3TrellisTrace tr;
        openuwb::demod::core::detail::vitdec_cl3_soft_trace(l0.data(), l1.data(), n, tr);
        BOOST_REQUIRE_EQUAL(tr.bits.size(), n);
        BOOST_REQUIRE_EQUAL(plain.size(), n);
        BOOST_CHECK(std::equal(plain.begin(), plain.end(), tr.bits.begin()));
        // The trace must also carry usable per-step evidence for the rescue.
        BOOST_REQUIRE_EQUAL(tr.margins.size(), n);
        BOOST_REQUIRE_EQUAL(tr.states.size(), n + 1);
        BOOST_REQUIRE_EQUAL(tr.lose_u.size(), n);
        for (size_t i = 0; i < n; ++i) {
            BOOST_CHECK(tr.margins[i] >= 0.0f);
            BOOST_CHECK(tr.lose_u[i] <= 1);
        }
    }
}

// ---------------------------------------------------------------------------
// FEC-OPT (perf/payload-fec-opt): bit-exactness of the table-driven GF(2^6)
// RS(63,55) decoder and the unrolled 4-state CL-3 Viterbi.
//
// The reference implementation below is an independent copy of the code that
// ran before this change (bit-serial GF multiply, alpha-power syndrome loop,
// `for (st) for (u)` ACS).  The optimised headers are always compared against
// this copy -- never against themselves.
// ---------------------------------------------------------------------------
namespace fec_opt_ref {

// --- Original GF(2^6) arithmetic (poly 0x61, alpha = 2) -------------------
int gf_mul(int a, int b)
{
    int p = 0;
    a &= 0x3f;
    b &= 0x3f;
    for (int i = 0; i < 6; ++i) {
        if (b & 1)
            p ^= a;
        const bool carry = (a & 0x20) != 0;
        a = (a << 1) & 0x3f;
        if (carry)
            a ^= 0x21;
        b >>= 1;
    }
    return p & 0x3f;
}

int gf_pow(int a, int n)
{
    int r = 1;
    a &= 0x3f;
    n %= 63;
    if (n < 0)
        n += 63;
    while (n) {
        if (n & 1)
            r = gf_mul(r, a);
        a = gf_mul(a, a);
        n >>= 1;
    }
    return r;
}

int gf_inv(int a)
{
    return a ? gf_pow(a, 62) : 0;
}

// --- Original RS decoder --------------------------------------------------
void bits_to_syms(const int8_t* bits, int count, int* symbols)
{
    for (int i = 0; i < count; ++i) {
        int s = 0;
        for (int b = 0; b < 6; ++b)
            s = (s << 1) | (bits[i * 6 + b] & 1);
        symbols[i] = s;
    }
}

void syms_to_bits(const int* syms, int nsym, int8_t* bits)
{
    for (int i = 0; i < nsym; ++i)
        for (int b = 0; b < 6; ++b)
            bits[i * 6 + b] = static_cast<int8_t>((syms[i] >> (5 - b)) & 1);
}

void syndromes(const int* r63, int* S8)
{
    for (int i = 1; i <= 8; ++i) {
        const int ai = gf_pow(2, i);
        int s = 0;
        int ap = 1;
        for (int j = 0; j < 63; ++j) {
            s ^= gf_mul(r63[j], ap);
            ap = gf_mul(ap, ai);
        }
        S8[i - 1] = s;
    }
}

int bm_len(const int* S, int count, int* Lambda)
{
    int C[9] = { 1, 0, 0, 0, 0, 0, 0, 0, 0 };
    int B[9] = { 1, 0, 0, 0, 0, 0, 0, 0, 0 };
    int L = 0;
    int m = 1;
    int b = 1;
    for (int n = 0; n < count; ++n) {
        int delta = S[n];
        for (int i = 1; i <= L; ++i)
            delta ^= gf_mul(C[i], S[n - i]);
        if (delta == 0) {
            ++m;
        } else {
            int T[9];
            for (int i = 0; i < 9; ++i)
                T[i] = C[i];
            const int scale = gf_mul(delta, gf_inv(b));
            for (int i = 0; i < 9 - m; ++i)
                C[i + m] ^= gf_mul(scale, B[i]);
            if (2 * L <= n) {
                L = n + 1 - L;
                for (int i = 0; i < 9; ++i)
                    B[i] = T[i];
                b = delta;
                m = 1;
            } else {
                ++m;
            }
        }
    }
    for (int i = 0; i < 9; ++i)
        Lambda[i] = C[i];
    return L;
}

int chien(const int* Lambda, int L, int* err_pos)
{
    int nerr = 0;
    for (int j = 0; j < 63; ++j) {
        const int xinv = gf_pow(2, (63 - j) % 63);
        int v = 0;
        int xp = 1;
        for (int i = 0; i <= L; ++i) {
            v ^= gf_mul(Lambda[i], xp);
            xp = gf_mul(xp, xinv);
        }
        if (v == 0) {
            if (nerr < 4)
                err_pos[nerr] = j;
            ++nerr;
        }
    }
    return nerr;
}

void forney(const int* S8, const int* Lambda, int L, const int* err_pos,
            int nerr, int* err_val)
{
    int Omega[8] = {};
    for (int i = 0; i < 8; ++i) {
        int o = 0;
        for (int j = 0; j <= L && j <= i; ++j)
            o ^= gf_mul(Lambda[j], S8[i - j]);
        Omega[i] = o;
    }
    for (int k = 0; k < nerr; ++k) {
        const int j = err_pos[k];
        const int Xinv = gf_pow(2, (63 - j) % 63);
        int num = 0;
        int xp = 1;
        for (int i = 0; i < 8; ++i) {
            num ^= gf_mul(Omega[i], xp);
            xp = gf_mul(xp, Xinv);
        }
        int den = 0;
        xp = 1;
        for (int i = 1; i <= L; i += 2) {
            den ^= gf_mul(Lambda[i], xp);
            xp = gf_mul(xp, gf_mul(Xinv, Xinv));
        }
        err_val[k] = den ? gf_mul(num, gf_inv(den)) : 0;
    }
}

bool zero_syndromes(const int* S)
{
    for (int i = 0; i < 8; ++i)
        if (S[i] != 0)
            return false;
    return true;
}

bool decode_block(const int8_t* coded378, int8_t* data330)
{
    int r[63];
    bits_to_syms(coded378, 63, r);
    int S[8];
    syndromes(r, S);
    if (!zero_syndromes(S)) {
        int Lambda[9] = {};
        const int L = bm_len(S, 8, Lambda);
        if (L <= 0 || L > 4)
            return false;
        int err_pos[4] = {};
        const int nerr = chien(Lambda, L, err_pos);
        if (nerr != L)
            return false;
        int corrections[8] = {};
        forney(S, Lambda, L, err_pos, nerr, corrections);
        for (int i = 0; i < nerr; ++i)
            r[err_pos[i]] ^= corrections[i];
        int check[8];
        syndromes(r, check);
        if (!zero_syndromes(check))
            return false;
    }
    syms_to_bits(r, 55, data330);
    return true;
}

void pack_psdu_bytes(const std::vector<int8_t>& bits, std::vector<uint8_t>& bytes)
{
    bytes.resize(bits.size() / 8);
    for (size_t i = 0; i < bytes.size(); ++i) {
        uint8_t byte = 0;
        for (int bit = 0; bit < 8; ++bit)
            byte |= static_cast<uint8_t>(bits[8 * i + bit]) << bit;
        bytes[i] = byte;
    }
}

// --- Original CL-3 Viterbi ------------------------------------------------
struct Cl3Transition {
    int g0;
    int g1;
    int next_state;
};

Cl3Transition cl3_transition(int state, int input)
{
    const int s1 = state >> 1;
    const int s0 = state & 1;
    return { s1, input ^ s0, (input << 1) | s1 };
}

float soft_metric(float path, int o0, int o1, float llr0, float llr1)
{
    const float c0 = (2.0f * static_cast<float>(o0) - 1.0f) * llr0;
    const float c1 = (2.0f * static_cast<float>(o1) - 1.0f) * llr1;
    return path + c0 + c1;
}

template <typename Metric, typename ExtendPath>
void vit_truncated(size_t N, Metric inf, ExtendPath extend,
                   std::vector<int8_t>& out)
{
    int o0[4][2], o1[4][2], nxt[4][2];
    for (int st = 0; st < 4; ++st) {
        for (int u = 0; u < 2; ++u) {
            const auto step = cl3_transition(st, u);
            o0[st][u] = step.g0;
            o1[st][u] = step.g1;
            nxt[st][u] = step.next_state;
        }
    }
    Metric surf[4] = { Metric{0}, inf, inf, inf };
    std::vector<int8_t> back(N * 4);
    for (size_t t = 0; t < N; ++t) {
        Metric ns[4] = { inf, inf, inf, inf };
        for (int st = 0; st < 4; ++st) {
            for (int u = 0; u < 2; ++u) {
                const Metric m = extend(t, surf[st], o0[st][u], o1[st][u]);
                const int nst = nxt[st][u];
                if (m < ns[nst]) {
                    ns[nst] = m;
                    back[t * 4 + nst] = static_cast<int8_t>((u << 2) | st);
                }
            }
        }
        for (int i = 0; i < 4; ++i)
            surf[i] = ns[i];
    }
    int best = 0;
    for (int i = 1; i < 4; ++i)
        if (surf[i] < surf[best])
            best = i;
    out.resize(N);
    int cur = best;
    for (size_t t = N; t-- > 0;) {
        const int v = back[t * 4 + cur];
        out[t] = static_cast<int8_t>(v >> 2);
        cur = v & 3;
    }
}

void vit_hard(const int8_t* rx, size_t N, std::vector<int8_t>& out)
{
    vit_truncated(N, 1000000, [&](size_t t, int path, int g0, int g1) {
        const int distance = (rx[2 * t] ^ g0) + (rx[2 * t + 1] ^ g1);
        return path + distance;
    }, out);
}

// Count the ACS comparisons where a candidate metric exactly equals the
// running best (and both are finite): those are the steps where the strict
// `if (m < ns[nst])` tie rule decides the survivor, so the equivalence test
// below is only meaningful if this counter is non-zero.
template <typename Metric, typename ExtendPath>
size_t count_ties(size_t N, Metric inf, ExtendPath extend)
{
    int o0[4][2], o1[4][2], nxt[4][2];
    for (int st = 0; st < 4; ++st) {
        for (int u = 0; u < 2; ++u) {
            const auto step = cl3_transition(st, u);
            o0[st][u] = step.g0;
            o1[st][u] = step.g1;
            nxt[st][u] = step.next_state;
        }
    }
    Metric surf[4] = { Metric{0}, inf, inf, inf };
    size_t ties = 0;
    for (size_t t = 0; t < N; ++t) {
        Metric ns[4] = { inf, inf, inf, inf };
        for (int st = 0; st < 4; ++st) {
            for (int u = 0; u < 2; ++u) {
                const Metric m = extend(t, surf[st], o0[st][u], o1[st][u]);
                const int nst = nxt[st][u];
                if (m < ns[nst]) {
                    ns[nst] = m;
                } else if (m == ns[nst] && m != inf) {
                    ++ties;
                }
            }
        }
        for (int i = 0; i < 4; ++i)
            surf[i] = ns[i];
    }
    return ties;
}

size_t count_hard_ties(const int8_t* rx, size_t N)
{
    return count_ties(N, 1000000, [&](size_t t, int path, int g0, int g1) {
        const int distance = (rx[2 * t] ^ g0) + (rx[2 * t + 1] ^ g1);
        return path + distance;
    });
}

void vit_soft(const float* l0, const float* l1, size_t N,
              std::vector<int8_t>& out)
{
    vit_truncated(N, 1e30f, [&](size_t t, float path, int g0, int g1) {
        return soft_metric(path, g0, g1, l0[t], l1[t]);
    }, out);
}

struct Trace {
    std::vector<int8_t> bits;
    std::vector<int8_t> states;
    std::vector<float> margins;
    std::vector<uint8_t> lose_u;
};

void vit_soft_trace(const float* llr0, const float* llr1, size_t N, Trace& tr)
{
    int o0[4][2], o1[4][2], nxt[4][2];
    for (int st = 0; st < 4; ++st) {
        for (int u = 0; u < 2; ++u) {
            const auto step = cl3_transition(st, u);
            o0[st][u] = step.g0;
            o1[st][u] = step.g1;
            nxt[st][u] = step.next_state;
        }
    }
    const float INF = 1e30f;
    float surf[4] = { 0.0f, INF, INF, INF };
    std::vector<int8_t> back(N * 4);
    std::vector<float> diff(N * 4, INF);
    std::vector<uint8_t> alt(N * 4, 0);
    for (size_t t = 0; t < N; ++t) {
        float ns[4] = { INF, INF, INF, INF };
        float ns2[4] = { INF, INF, INF, INF };
        uint8_t nalt[4] = { 0, 0, 0, 0 };
        for (int st = 0; st < 4; ++st) {
            for (int u = 0; u < 2; ++u) {
                const float m = soft_metric(surf[st], o0[st][u], o1[st][u],
                                            llr0[t], llr1[t]);
                const int nst = nxt[st][u];
                const uint8_t code = static_cast<uint8_t>((u << 2) | st);
                if (m < ns[nst]) {
                    ns2[nst] = ns[nst];
                    nalt[nst] = static_cast<uint8_t>(back[t * 4 + nst]);
                    ns[nst] = m;
                    back[t * 4 + nst] = static_cast<int8_t>(code);
                } else if (m < ns2[nst]) {
                    ns2[nst] = m;
                    nalt[nst] = code;
                }
            }
        }
        for (int i = 0; i < 4; ++i) {
            surf[i] = ns[i];
            diff[t * 4 + i] = ns2[i] - ns[i];
            alt[t * 4 + i] = nalt[i];
        }
    }
    int best = 0;
    for (int i = 1; i < 4; ++i)
        if (surf[i] < surf[best])
            best = i;
    tr.bits.resize(N);
    tr.states.resize(N + 1);
    tr.margins.resize(N);
    tr.lose_u.resize(N);
    int cur = best;
    tr.states[N] = static_cast<int8_t>(cur);
    for (size_t t = N; t-- > 0;) {
        const int v = back[t * 4 + cur];
        tr.bits[t] = static_cast<int8_t>(v >> 2);
        tr.margins[t] = diff[t * 4 + cur];
        tr.lose_u[t] = static_cast<uint8_t>(alt[t * 4 + cur] >> 2);
        cur = v & 3;
        tr.states[t] = static_cast<int8_t>(cur);
    }
}

// --- Systematic RS(63,55) test encoder ------------------------------------
// Solves A p = b over GF(2^6) for the 8 parity symbols so that
// r(x) = sum_{j<55} d_j x^j + sum_k p_k x^{55+k} has roots alpha^1..alpha^8.
bool encode_systematic(const int* data55, int* out63)
{
    for (int j = 0; j < 55; ++j)
        out63[j] = data55[j] & 0x3f;
    int A[8][8];
    int b[8];
    for (int i = 1; i <= 8; ++i) {
        int s = 0;
        for (int j = 0; j < 55; ++j)
            s ^= gf_mul(out63[j], gf_pow(2, i * j));
        b[i - 1] = s;
        for (int k = 0; k < 8; ++k)
            A[i - 1][k] = gf_pow(2, i * (55 + k));
    }
    for (int c = 0; c < 8; ++c) {
        int piv = -1;
        for (int r = c; r < 8; ++r)
            if (A[r][c]) {
                piv = r;
                break;
            }
        if (piv < 0)
            return false;
        if (piv != c) {
            for (int k = 0; k < 8; ++k)
                std::swap(A[c][k], A[piv][k]);
            std::swap(b[c], b[piv]);
        }
        const int invp = gf_inv(A[c][c]);
        for (int k = c; k < 8; ++k)
            A[c][k] = gf_mul(A[c][k], invp);
        b[c] = gf_mul(b[c], invp);
        for (int r = 0; r < 8; ++r) {
            if (r == c)
                continue;
            const int f = A[r][c];
            if (!f)
                continue;
            for (int k = c; k < 8; ++k)
                A[r][k] ^= gf_mul(f, A[c][k]);
            b[r] ^= gf_mul(f, b[c]);
        }
    }
    for (int k = 0; k < 8; ++k)
        out63[55 + k] = b[k];
    return true;
}

} // namespace fec_opt_ref

BOOST_AUTO_TEST_CASE(test_fec_opt_gf64_tables_match_bitserial_reference)
{
    using namespace openuwb::codec;
    // 64 x 64 exhaustive multiply.
    for (int a = 0; a < 64; ++a) {
        for (int b = 0; b < 64; ++b) {
            const int want = fec_opt_ref::gf_mul(a, b);
            BOOST_REQUIRE_EQUAL(rs_gf_mul(a, b), want);
            // Input masking must keep the old `a &= 0x3f` / `b &= 0x3f`.
            BOOST_REQUIRE_EQUAL(rs_gf_mul(a | 0x40, b), want);
            BOOST_REQUIRE_EQUAL(rs_gf_mul(a, b | 0x80), want);
            BOOST_REQUIRE_EQUAL(rs_gf_mul(a | 0xc0, b | 0xc0), want);
        }
    }
    // Powers, including the negative / >62 reduction paths.
    const int exponents[] = { -64, -1, 0, 1, 61, 62, 63, 64, 126 };
    for (int a = 0; a < 64; ++a)
        for (int n : exponents)
            BOOST_REQUIRE_EQUAL(rs_gf_pow(a, n), fec_opt_ref::gf_pow(a, n));
    // Inverses (inv(0) == 0).
    for (int a = 0; a < 64; ++a) {
        BOOST_REQUIRE_EQUAL(rs_gf_inv(a), fec_opt_ref::gf_inv(a));
        BOOST_REQUIRE_EQUAL(rs_gf_inv(a | 0x40), fec_opt_ref::gf_inv(a));
    }
    BOOST_REQUIRE_EQUAL(rs_gf_inv(0), 0);
    // The alpha power lookups must equal pow(2, k) for the whole cycle.
    for (int k = -63; k <= 126; ++k) {
        BOOST_REQUIRE_EQUAL(rs_gf_alpha_pow(k), fec_opt_ref::gf_pow(2, k));
        BOOST_REQUIRE_EQUAL(rs_gf_alpha_inv_pow(k), fec_opt_ref::gf_pow(2, -k));
    }
    // Broadcast primitive check: 2 generates the whole multiplicative group.
    int seen[64] = {};
    for (int k = 0; k < 63; ++k)
        ++seen[rs_gf_alpha_pow(k)];
    for (int v = 1; v < 64; ++v)
        BOOST_REQUIRE_EQUAL(seen[v], 1);
    BOOST_REQUIRE_EQUAL(seen[0], 0);
}

BOOST_AUTO_TEST_CASE(test_fec_opt_bit_packing_matches_reference)
{
    using namespace openuwb::codec;
    std::mt19937 rng(20261120u);
    std::uniform_int_distribution<int> bit(0, 1);
    for (int rep = 0; rep < 8; ++rep) {
        const int nsym = 63;
        std::vector<int8_t> bits(static_cast<size_t>(nsym) * 6);
        for (auto& b : bits)
            b = static_cast<int8_t>(bit(rng));
        std::vector<int> syms_new(nsym), syms_ref(nsym);
        rs_bits_to_syms(bits.data(), nsym, syms_new.data());
        fec_opt_ref::bits_to_syms(bits.data(), nsym, syms_ref.data());
        BOOST_REQUIRE(syms_new == syms_ref);

        std::vector<int8_t> from_new(static_cast<size_t>(nsym) * 6, 0);
        std::vector<int8_t> from_ref(static_cast<size_t>(nsym) * 6, 0);
        rs_syms_to_bits(syms_ref.data(), nsym, from_new.data());
        fec_opt_ref::syms_to_bits(syms_ref.data(), nsym, from_ref.data());
        BOOST_REQUIRE(from_new == from_ref);
        BOOST_REQUIRE(from_new == bits);

        // LSB-first byte packing.
        std::vector<int8_t> psdu(8 * 37);
        for (auto& b : psdu)
            b = static_cast<int8_t>(bit(rng));
        std::vector<uint8_t> new_bytes, ref_bytes;
        openuwb::demod::core::detail::pack_psdu_bytes(psdu, new_bytes);
        fec_opt_ref::pack_psdu_bytes(psdu, ref_bytes);
        BOOST_REQUIRE(new_bytes == ref_bytes);
        BOOST_REQUIRE_EQUAL(new_bytes.size(), 37u);

        // Non-binary inputs must truncate exactly like the reference (each
        // shifted term keeps only its low 8 bits).
        for (auto& b : psdu)
            b = static_cast<int8_t>(static_cast<int>(rng() % 8) - 2);
        openuwb::demod::core::detail::pack_psdu_bytes(psdu, new_bytes);
        fec_opt_ref::pack_psdu_bytes(psdu, ref_bytes);
        BOOST_REQUIRE(new_bytes == ref_bytes);
    }
}

BOOST_AUTO_TEST_CASE(test_fec_opt_rs_decode_block_matches_reference)
{
    using namespace openuwb::demod::core::detail;
    std::mt19937 rng(20261121u);
    std::uniform_int_distribution<int> sym(0, 63);
    std::uniform_int_distribution<int> pos(0, 62);
    std::uniform_int_distribution<int> errval(1, 63);

    size_t uncorrectable = 0;
    size_t miscorrected = 0;
    for (int trial = 0; trial < 240; ++trial) {
        int data55[55], cw[63];
        for (int j = 0; j < 55; ++j)
            data55[j] = sym(rng);
        BOOST_REQUIRE(fec_opt_ref::encode_systematic(data55, cw));
        int check[8];
        fec_opt_ref::syndromes(cw, check);
        BOOST_REQUIRE(fec_opt_ref::zero_syndromes(check));

        int8_t coded[378];
        fec_opt_ref::syms_to_bits(cw, 63, coded);

        const int nerr = trial % 6; // 0..5 injected symbol errors
        bool used[63] = {};
        for (int e = 0; e < nerr; ++e) {
            int p = pos(rng);
            while (used[p])
                p = (p + 1) % 63;
            used[p] = true;
            const int v = errval(rng); // nonzero GF(2^6) error value
            for (int b = 0; b < 6; ++b)
                if ((v >> b) & 1)
                    coded[p * 6 + 5 - b] ^= 1;
        }

        int8_t new_data[330] = {};
        int8_t ref_data[330] = {};
        const bool new_ok = rs_decode_block(coded, new_data);
        const bool ref_ok = fec_opt_ref::decode_block(coded, ref_data);
        BOOST_REQUIRE_EQUAL(new_ok, ref_ok);
        BOOST_REQUIRE(std::equal(new_data, new_data + 330, ref_data));
        if (nerr <= 4) {
            BOOST_REQUIRE(new_ok);
            int8_t expect[330];
            fec_opt_ref::syms_to_bits(data55, 55, expect);
            BOOST_REQUIRE(std::equal(new_data, new_data + 330, expect));
        } else {
            if (new_ok)
                ++miscorrected;
            else
                ++uncorrectable;
        }
    }
    // Five symbol errors exceed t = 4.  RS(63,55) has d_min = 9, so a 5-error
    // word can (rarely) land exactly 4 symbols from a *different* codeword and
    // be "miscorrected"; the invariant that must hold is that the optimised
    // decoder agrees with the reference on every single trial (checked above).
    BOOST_TEST_MESSAGE("RS 5-error trials: uncorrectable="
                       << uncorrectable << " miscorrected=" << miscorrected);
    BOOST_REQUIRE_EQUAL(uncorrectable + miscorrected, 40u);
    BOOST_REQUIRE_GE(uncorrectable, 39u);
}

BOOST_AUTO_TEST_CASE(test_fec_opt_viterbi_hard_matches_reference)
{
    using namespace openuwb::demod::core::detail;
    std::mt19937 rng(20261122u);
    std::uniform_int_distribution<int> bit(0, 1);
    const size_t lengths[] = { 1, 2, 3, 7, 64, 1229, 2048, 2049 };
    size_t total_ties = 0;
    for (size_t n : lengths) {
        for (int pattern = 0; pattern < 5; ++pattern) {
            std::vector<int8_t> rx(2 * n);
            for (size_t i = 0; i < n; ++i) {
                int a = bit(rng) & 1;
                int b = bit(rng) & 1;
                if (pattern == 1) { // all-zero input: metric ties everywhere
                    a = 0;
                    b = 0;
                } else if (pattern == 2) { // all-one input
                    a = 1;
                    b = 1;
                } else if (pattern == 3) { // alternating hard ties
                    a = static_cast<int>(i & 1);
                    b = static_cast<int>((i >> 1) & 1);
                } else if (pattern == 4) { // repeated 2-symbol tie motif
                    a = 0;
                    b = static_cast<int>(i & 1);
                }
                rx[2 * i] = static_cast<int8_t>(a);
                rx[2 * i + 1] = static_cast<int8_t>(b);
            }
            total_ties += fec_opt_ref::count_hard_ties(rx.data(), n);
            std::vector<int8_t> got, want;
            vitdec_cl3(rx.data(), n, got);
            fec_opt_ref::vit_hard(rx.data(), n, want);
            BOOST_REQUIRE_EQUAL(got.size(), n);
            BOOST_REQUIRE_EQUAL(want.size(), n);
            BOOST_REQUIRE(std::equal(got.begin(), got.end(), want.begin()));
        }
    }
    // The equivalence above only covers tie-breaking if exact ties occur.
    BOOST_TEST_MESSAGE("hard CL-3 ACS exact ties exercised: " << total_ties);
    BOOST_REQUIRE_GT(total_ties, 0u);
}

BOOST_AUTO_TEST_CASE(test_fec_opt_viterbi_soft_matches_reference)
{
    using namespace openuwb::demod::core::detail;
    std::mt19937 rng(20261123u);
    std::normal_distribution<float> nd(0.0f, 2.0f);
    const size_t lengths[] = { 1, 2, 3, 7, 64, 1229, 2048, 2049 };
    size_t total_ties = 0;
    for (size_t n : lengths) {
        for (int pattern = 0; pattern < 6; ++pattern) {
            std::vector<float> l0(n), l1(n);
            for (size_t i = 0; i < n; ++i) {
                switch (pattern) {
                case 1: // all-zero LLRs: every edge weight is an exact tie
                    l0[i] = 0.0f;
                    l1[i] = 0.0f;
                    break;
                case 2: // identical constant LLRs
                    l0[i] = 0.5f;
                    l1[i] = 0.5f;
                    break;
                case 3: // constant-magnitude sign ties
                    l0[i] = (i & 1) ? 1.0f : -1.0f;
                    l1[i] = (i & 2) ? 1.0f : -1.0f;
                    break;
                case 4: // large magnitudes
                    l0[i] = (nd(rng) > 0.0f) ? 1.0e6f : -1.0e6f;
                    l1[i] = (nd(rng) > 0.0f) ? 1.0e6f : -1.0e6f;
                    break;
                case 5: // tiny magnitudes (near-tie decisions)
                    l0[i] = 1.0e-8f * nd(rng);
                    l1[i] = 1.0e-8f * nd(rng);
                    break;
                default:
                    l0[i] = nd(rng);
                    l1[i] = nd(rng);
                    break;
                }
            }
            total_ties += fec_opt_ref::count_ties(
                n, 1e30f, [&](size_t t, float path, int g0, int g1) {
                    return fec_opt_ref::soft_metric(path, g0, g1, l0[t], l1[t]);
                });
            std::vector<int8_t> got, want;
            vitdec_cl3_soft(l0.data(), l1.data(), n, got);
            fec_opt_ref::vit_soft(l0.data(), l1.data(), n, want);
            BOOST_REQUIRE_EQUAL(got.size(), n);
            BOOST_REQUIRE_EQUAL(want.size(), n);
            BOOST_REQUIRE(std::equal(got.begin(), got.end(), want.begin()));
        }
    }
    // All-zero / constant LLRs force exact metric ties on every step, so the
    // strict-comparison tie rule is definitely exercised.
    BOOST_TEST_MESSAGE("soft CL-3 ACS exact ties exercised: " << total_ties);
    BOOST_REQUIRE_GT(total_ties, 100u);
}

BOOST_AUTO_TEST_CASE(test_fec_opt_viterbi_soft_trace_matches_reference)
{
    using namespace openuwb::demod::core::detail;
    std::mt19937 rng(20261124u);
    std::normal_distribution<float> nd(0.0f, 1.5f);
    const size_t lengths[] = { 1, 2, 3, 7, 64, 257, 1229, 2048, 2049 };
    for (size_t n : lengths) {
        for (int pattern = 0; pattern < 3; ++pattern) {
            std::vector<float> l0(n), l1(n);
            for (size_t i = 0; i < n; ++i) {
                if (pattern == 1) {
                    l0[i] = 0.0f;
                    l1[i] = 0.0f;
                } else if (pattern == 2) {
                    l0[i] = (i & 1) ? 2.0f : -2.0f;
                    l1[i] = (i & 2) ? 2.0f : -2.0f;
                } else {
                    l0[i] = nd(rng);
                    l1[i] = nd(rng);
                }
            }
            Cl3TrellisTrace got;
            fec_opt_ref::Trace want;
            vitdec_cl3_soft_trace(l0.data(), l1.data(), n, got);
            fec_opt_ref::vit_soft_trace(l0.data(), l1.data(), n, want);
            BOOST_REQUIRE_EQUAL(got.bits.size(), n);
            BOOST_REQUIRE_EQUAL(got.states.size(), n + 1);
            BOOST_REQUIRE_EQUAL(got.margins.size(), n);
            BOOST_REQUIRE_EQUAL(got.lose_u.size(), n);
            BOOST_REQUIRE(std::equal(got.bits.begin(), got.bits.end(), want.bits.begin()));
            BOOST_REQUIRE(std::equal(got.states.begin(), got.states.end(), want.states.begin()));
            BOOST_REQUIRE(std::equal(got.lose_u.begin(), got.lose_u.end(), want.lose_u.begin()));
            for (size_t i = 0; i < n; ++i)
                BOOST_REQUIRE_EQUAL(got.margins[i], want.margins[i]);
        }
    }
}

BOOST_AUTO_TEST_CASE(test_fec_opt_rs_kernels_match_reference)
{
    // Direct bit-exactness of the individually rewritten RS kernels, including
    // the degrees the errors-and-erasures rescue path reaches (Forney with
    // L = Le + v up to 8), which rs_decode_block alone never exercises.
    using namespace openuwb::demod::core::detail;
    std::mt19937 rng(20261125u);
    std::uniform_int_distribution<int> sym(0, 63);
    std::uniform_int_distribution<int> pos(0, 62);
    std::uniform_int_distribution<int> degree(0, 8);
    std::uniform_int_distribution<int> count(0, 4);

    for (int trial = 0; trial < 2000; ++trial) {
        int r63[63];
        for (int j = 0; j < 63; ++j)
            r63[j] = sym(rng);
        int S_new[8], S_ref[8];
        rs_syndromes(r63, S_new);
        fec_opt_ref::syndromes(r63, S_ref);
        for (int i = 0; i < 8; ++i)
            BOOST_REQUIRE_EQUAL(S_new[i], S_ref[i]);

        int L_new[9] = {}, L_ref[9] = {};
        const int deg_new = rs_berlekamp_massey(S_new, L_new);
        const int deg_ref = fec_opt_ref::bm_len(S_ref, 8, L_ref);
        BOOST_REQUIRE_EQUAL(deg_new, deg_ref);
        for (int i = 0; i < 9; ++i)
            BOOST_REQUIRE_EQUAL(L_new[i], L_ref[i]);

        // Fuzz the locator/Forney inputs too (any 0..8 degree, any roots).
        const int L = degree(rng);
        int Lambda[9] = {};
        Lambda[0] = 1;
        for (int i = 1; i <= L; ++i)
            Lambda[i] = sym(rng);
        int p_new[4] = {}, p_ref[4] = {};
        const int n_new = rs_chien_search(Lambda, L, p_new);
        const int n_ref = fec_opt_ref::chien(Lambda, L, p_ref);
        BOOST_REQUIRE_EQUAL(n_new, n_ref);
        for (int i = 0; i < 4; ++i)
            BOOST_REQUIRE_EQUAL(p_new[i], p_ref[i]);

        int pos_in[4];
        const int nerr = count(rng);
        for (int i = 0; i < nerr; ++i)
            pos_in[i] = pos(rng);
        int v_new[8] = {}, v_ref[8] = {};
        rs_forney(S_new, Lambda, L, pos_in, nerr, v_new);
        fec_opt_ref::forney(S_ref, Lambda, L, pos_in, nerr, v_ref);
        for (int i = 0; i < 8; ++i)
            BOOST_REQUIRE_EQUAL(v_new[i], v_ref[i]);
    }
}

// ---------------------------------------------------------------------------
// Wave 1 / JOINT-ML-VITERBI QA.
//
// Everything below compares the new joint bi-orthogonal ML kernels against
// independent references written in this anonymous namespace: a nested
// `for (st) for (u)` DP (the pre-unrolling loop shape), a brute-force
// enumeration of every input sequence for the tiny exact-ML case, and a local
// CL-3 encoder built from openuwb::codec::cl3_transition.  The new production
// code is never used as its own oracle.
// ---------------------------------------------------------------------------
namespace {

using openuwb::codec::Cl3Transition;
using openuwb::codec::cl3_transition;
using openuwb::codec::crc16_802154;
using openuwb::codec::hrp_secded;
using openuwb::codec::rs_bits_to_syms;
using openuwb::codec::rs_syms_to_bits;

// Joint ML branch metric, written straight from the task definition.
float qa_branch(float m0, float m1, int o0, int o1)
{
    return (o0 == 0) ? (o1 ? m0 : -m0) : (o1 ? m1 : -m1);
}

void qa_cl3_encode(const std::vector<int8_t>& u, std::vector<int8_t>& g0,
                   std::vector<int8_t>& g1)
{
    g0.assign(u.size(), int8_t(0));
    g1.assign(u.size(), int8_t(0));
    int state = 0;
    for (size_t t = 0; t < u.size(); ++t) {
        const Cl3Transition step = cl3_transition(state, u[t] & 1);
        g0[t] = static_cast<int8_t>(step.g0);
        g1[t] = static_cast<int8_t>(step.g1);
        state = step.next_state;
    }
}

// BPM-BPSK constellation: (g0,g1) -> (m0,m1) at amplitude A.
void qa_map_bpm(const std::vector<int8_t>& g0, const std::vector<int8_t>& g1,
                float A, std::vector<float>& m0, std::vector<float>& m1)
{
    m0.assign(g0.size(), 0.0f);
    m1.assign(g0.size(), 0.0f);
    for (size_t t = 0; t < g0.size(); ++t) {
        const float s = g1[t] ? -A : A;
        if (g0[t] == 0)
            m0[t] = s;
        else
            m1[t] = s;
    }
}

// Split-LLR convention of decide_bpm_soft(): positive favours bit 0.
void qa_split_llrs(const std::vector<float>& m0, const std::vector<float>& m1,
                   std::vector<float>& l0, std::vector<float>& l1)
{
    l0.resize(m0.size());
    l1.resize(m0.size());
    for (size_t t = 0; t < m0.size(); ++t) {
        l0[t] = std::abs(m0[t]) - std::abs(m1[t]);
        l1[t] = std::max(m0[t], m1[t]) - std::max(-m0[t], -m1[t]);
    }
}

float qa_path_metric(const std::vector<int8_t>& bits,
                     const std::vector<float>& m0,
                     const std::vector<float>& m1)
{
    int state = 0;
    float metric = 0.0f;
    for (size_t t = 0; t < bits.size(); ++t) {
        const Cl3Transition step = cl3_transition(state, bits[t] & 1);
        metric += qa_branch(m0[t], m1[t], step.g0, step.g1);
        state = step.next_state;
    }
    return metric;
}

// Independent nested-loop DP with traceback and final metric.
struct QaPath {
    std::vector<int8_t> bits;
    float metric = 0.0f;
};

QaPath qa_ref_dp(const std::vector<float>& m0, const std::vector<float>& m1,
                 bool terminated)
{
    const float INF = 1e30f;
    const size_t N = m0.size();
    float surf[4] = { 0.0f, INF, INF, INF };
    std::vector<int8_t> back(N * 4, int8_t(0));
    for (size_t t = 0; t < N; ++t) {
        float ns[4] = { INF, INF, INF, INF };
        for (int st = 0; st < 4; ++st) {
            for (int u = 0; u < 2; ++u) {
                const Cl3Transition step = cl3_transition(st, u);
                const float m = surf[st] + qa_branch(m0[t], m1[t], step.g0,
                                                     step.g1);
                if (m < ns[step.next_state]) {
                    ns[step.next_state] = m;
                    back[t * 4 + step.next_state] =
                        static_cast<int8_t>((u << 2) | st);
                }
            }
        }
        for (int i = 0; i < 4; ++i)
            surf[i] = ns[i];
    }
    int best = 0;
    if (!terminated) {
        for (int i = 1; i < 4; ++i)
            if (surf[i] < surf[best])
                best = i;
    }
    QaPath p;
    p.bits.resize(N);
    p.metric = surf[best];
    int cur = best;
    for (size_t t = N; t-- > 0;) {
        const int v = back[t * 4 + cur];
        p.bits[t] = static_cast<int8_t>(v >> 2);
        cur = v & 3;
    }
    return p;
}

// Split-LLR Viterbi, again from a nested loop (production vitdec_cl3_soft() is
// intentionally not called here).
std::vector<int8_t> qa_ref_split(const std::vector<float>& m0,
                                 const std::vector<float>& m1)
{
    std::vector<float> l0, l1;
    qa_split_llrs(m0, m1, l0, l1);
    const float INF = 1e30f;
    const size_t N = m0.size();
    float surf[4] = { 0.0f, INF, INF, INF };
    std::vector<int8_t> back(N * 4, int8_t(0));
    for (size_t t = 0; t < N; ++t) {
        float ns[4] = { INF, INF, INF, INF };
        for (int st = 0; st < 4; ++st) {
            for (int u = 0; u < 2; ++u) {
                const Cl3Transition step = cl3_transition(st, u);
                const float c0 = (2.0f * step.g0 - 1.0f) * l0[t];
                const float c1 = (2.0f * step.g1 - 1.0f) * l1[t];
                const float m = surf[st] + c0 + c1;
                if (m < ns[step.next_state]) {
                    ns[step.next_state] = m;
                    back[t * 4 + step.next_state] =
                        static_cast<int8_t>((u << 2) | st);
                }
            }
        }
        for (int i = 0; i < 4; ++i)
            surf[i] = ns[i];
    }
    int best = 0;
    for (int i = 1; i < 4; ++i)
        if (surf[i] < surf[best])
            best = i;
    std::vector<int8_t> out(N);
    int cur = best;
    for (size_t t = N; t-- > 0;) {
        const int v = back[t * 4 + cur];
        out[t] = static_cast<int8_t>(v >> 2);
        cur = v & 3;
    }
    return out;
}

// Exhaustive ML over all 2^N input sequences (small N only).
bool qa_brute_ml(const std::vector<float>& m0, const std::vector<float>& m1,
                 QaPath& out, size_t& min_count)
{
    const size_t N = m0.size();
    if (N > 20)
        return false;
    const float INF = 1e30f;
    float best = INF;
    std::vector<int8_t> bits(N, int8_t(0));
    std::vector<int8_t> best_bits;
    min_count = 0;
    const uint32_t total = static_cast<uint32_t>(1u) << N;
    for (uint32_t mask = 0; mask < total; ++mask) {
        for (size_t t = 0; t < N; ++t)
            bits[t] = static_cast<int8_t>((mask >> t) & 1u);
        const float v = qa_path_metric(bits, m0, m1);
        if (v < best - 1e-6f) {
            best = v;
            best_bits = bits;
            min_count = 1;
        } else if (std::abs(v - best) <= 1e-6f) {
            ++min_count;
        }
    }
    out.bits = best_bits;
    out.metric = best;
    return !best_bits.empty();
}

// Exact tie counter over the nested-loop ACS (does not change survivors).
size_t qa_count_biorth_ties(const std::vector<float>& m0,
                            const std::vector<float>& m1)
{
    const float INF = 1e30f;
    float surf[4] = { 0.0f, INF, INF, INF };
    size_t ties = 0;
    for (size_t t = 0; t < m0.size(); ++t) {
        float ns[4] = { INF, INF, INF, INF };
        for (int st = 0; st < 4; ++st) {
            for (int u = 0; u < 2; ++u) {
                const Cl3Transition step = cl3_transition(st, u);
                const float m = surf[st] + qa_branch(m0[t], m1[t], step.g0,
                                                     step.g1);
                if (m < ns[step.next_state]) {
                    ns[step.next_state] = m;
                } else if (m == ns[step.next_state] && m != INF) {
                    ++ties;
                }
            }
        }
        for (int i = 0; i < 4; ++i)
            surf[i] = ns[i];
    }
    return ties;
}

// Independent nested-loop tracing ML Viterbi.
struct QaTrace {
    std::vector<int8_t> bits;
    std::vector<int8_t> states;
    std::vector<float> margins;
    std::vector<uint8_t> lose_u;
};

QaTrace qa_ref_trace(const std::vector<float>& m0, const std::vector<float>& m1,
                     bool terminated)
{
    const float INF = 1e30f;
    const size_t N = m0.size();
    float surf[4] = { 0.0f, INF, INF, INF };
    std::vector<int8_t> back(N * 4, int8_t(0));
    std::vector<float> diff(N * 4, INF);
    std::vector<uint8_t> alt(N * 4, 0);
    for (size_t t = 0; t < N; ++t) {
        float ns[4] = { INF, INF, INF, INF };
        float ns2[4] = { INF, INF, INF, INF };
        uint8_t nalt[4] = { 0, 0, 0, 0 };
        for (int st = 0; st < 4; ++st) {
            for (int u = 0; u < 2; ++u) {
                const Cl3Transition step = cl3_transition(st, u);
                const float m = surf[st] + qa_branch(m0[t], m1[t], step.g0,
                                                     step.g1);
                const int nst = step.next_state;
                const uint8_t code = static_cast<uint8_t>((u << 2) | st);
                if (m < ns[nst]) {
                    ns2[nst] = ns[nst];
                    nalt[nst] = static_cast<uint8_t>(back[t * 4 + nst]);
                    ns[nst] = m;
                    back[t * 4 + nst] = static_cast<int8_t>(code);
                } else if (m < ns2[nst]) {
                    ns2[nst] = m;
                    nalt[nst] = code;
                }
            }
        }
        for (int i = 0; i < 4; ++i) {
            surf[i] = ns[i];
            diff[t * 4 + i] = ns2[i] - ns[i];
            alt[t * 4 + i] = nalt[i];
        }
    }
    int best = 0;
    if (!terminated) {
        for (int i = 1; i < 4; ++i)
            if (surf[i] < surf[best])
                best = i;
    }
    QaTrace tr;
    tr.bits.resize(N);
    tr.states.resize(N + 1);
    tr.margins.resize(N);
    tr.lose_u.resize(N);
    int cur = best;
    tr.states[N] = static_cast<int8_t>(cur);
    for (size_t t = N; t-- > 0;) {
        const int v = back[t * 4 + cur];
        tr.bits[t] = static_cast<int8_t>(v >> 2);
        tr.margins[t] = diff[t * 4 + cur];
        tr.lose_u[t] = static_cast<uint8_t>(alt[t * 4 + cur] >> 2);
        cur = v & 3;
        tr.states[t] = static_cast<int8_t>(cur);
    }
    return tr;
}

// ---- Synthetic end-to-end payload frames ---------------------------------
struct QaFrame {
    size_t psdu_bits = 0;
    size_t payload_symbols = 0;
    std::vector<uint8_t> bytes;    // expected PSDU (with FCS)
    std::vector<int8_t> u;         // joint trellis info bits
    std::vector<float> m0, m1;     // noiseless joint metrics
};

// Systematic RS(63,55) encode of one shortened block: data bits + 48 parity
// bits, matching expand_rs_codeword()'s leading-zero padding convention.
bool qa_rs_encode_block(const std::vector<int8_t>& data,
                        std::vector<int8_t>& coded)
{
    if (data.empty() || data.size() > 330)
        return false;
    int8_t data330[330] = {};
    const size_t pad = 330 - data.size();
    for (size_t i = 0; i < data.size(); ++i)
        data330[pad + i] = data[i] & 1;
    int syms55[55];
    rs_bits_to_syms(data330, 55, syms55);
    int cw63[63];
    if (!fec_opt_ref::encode_systematic(syms55, cw63))
        return false;
    int8_t par48[48];
    rs_syms_to_bits(cw63 + 55, 8, par48);
    coded = data;
    coded.insert(coded.end(), par48, par48 + 48);
    return true;
}

bool qa_build_frame(size_t psdu_bytes, uint32_t seed, QaFrame& f)
{
    if (psdu_bytes < 3)
        return false;
    std::mt19937 rng(seed);
    std::vector<uint8_t> bytes(psdu_bytes);
    for (size_t i = 0; i + 2 < psdu_bytes; ++i)
        bytes[i] = static_cast<uint8_t>(rng() & 0xffu);
    const uint16_t fcs = crc16_802154(bytes.data(), psdu_bytes - 2);
    bytes[psdu_bytes - 2] = static_cast<uint8_t>(fcs & 0xffu);
    bytes[psdu_bytes - 1] = static_cast<uint8_t>(fcs >> 8);

    std::vector<int8_t> psdu_bits(psdu_bytes * 8);
    for (size_t i = 0; i < psdu_bytes; ++i)
        for (int k = 0; k < 8; ++k)
            psdu_bits[8 * i + k] = static_cast<int8_t>((bytes[i] >> k) & 1u);

    std::vector<int8_t> rs_cw;
    size_t off = 0;
    while (off < psdu_bits.size()) {
        const size_t n = std::min<size_t>(330, psdu_bits.size() - off);
        std::vector<int8_t> block(psdu_bits.begin() + off,
                                  psdu_bits.begin() + off + n);
        std::vector<int8_t> coded;
        if (!qa_rs_encode_block(block, coded))
            return false;
        rs_cw.insert(rs_cw.end(), coded.begin(), coded.end());
        off += n;
    }

    f.psdu_bits = psdu_bits.size();
    f.payload_symbols = rs_cw.size();
    f.bytes = bytes;
    // The first 19 trellis bits are the systematic PHR bits.  The payload
    // decoder discards them, so any fixed pattern works for this frame.
    f.u.assign(19, int8_t(0));
    f.u.insert(f.u.end(), rs_cw.begin(), rs_cw.end());
    f.u.push_back(0);
    f.u.push_back(0);

    std::vector<int8_t> g0, g1;
    qa_cl3_encode(f.u, g0, g1);
    qa_map_bpm(g0, g1, 1.0f, f.m0, f.m1);
    return true;
}

void qa_add_noise(std::vector<float>& m0, std::vector<float>& m1, float sigma,
                  std::mt19937& rng)
{
    std::normal_distribution<float> nd(0.0f, sigma);
    for (size_t i = 0; i < m0.size(); ++i) {
        m0[i] += nd(rng);
        m1[i] += nd(rng);
    }
}

} // namespace

BOOST_AUTO_TEST_CASE(test_w1_joint_ml_exact_counterexample)
{
    using namespace openuwb::demod::core::detail;

    // The documented coupling: a +1.0 half and a -0.8 half make (1,1) the true
    // runner-up (cost -0.8), while the split LLRs score (1,1) as the worst
    // branch (+0.4).
    const std::vector<float> m0 = { 1.0f, 0.6f };
    const std::vector<float> m1 = { -0.8f, 0.9f };

    std::vector<int8_t> biorth;
    vitdec_cl3_biorth(m0.data(), m1.data(), m0.size(), biorth);
    BOOST_REQUIRE_EQUAL(biorth.size(), m0.size());

    // Exhaustive ML over all 2^2 = 4 input sequences.
    QaPath brute;
    size_t min_count = 0;
    BOOST_REQUIRE(qa_brute_ml(m0, m1, brute, min_count));
    BOOST_REQUIRE_EQUAL(min_count, size_t(1)); // unique ML path
    BOOST_CHECK_EQUAL_COLLECTIONS(biorth.begin(), biorth.end(),
                                  brute.bits.begin(), brute.bits.end());
    BOOST_CHECK_SMALL(qa_path_metric(biorth, m0, m1) - brute.metric, 1e-6f);

    // The split path must genuinely lose on this sequence.
    const std::vector<int8_t> split = qa_ref_split(m0, m1);
    const float split_metric = qa_path_metric(split, m0, m1);
    BOOST_CHECK(split != biorth);
    BOOST_CHECK_GT(split_metric, brute.metric + 1e-6f);
    BOOST_TEST_MESSAGE("joint-ML counterexample: biorth metric "
                       << brute.metric << ", split path metric " << split_metric
                       << ", split bits [" << int(split[0]) << int(split[1])
                       << "], tied ML paths " << min_count);

    // Trace format: bits/states/margins/lose_u must match the independent
    // nested-loop reference exactly, including the strict `<` tie order.
    Cl3TrellisTrace tr;
    vitdec_cl3_biorth_trace(m0.data(), m1.data(), m0.size(), tr);
    const QaTrace ref = qa_ref_trace(m0, m1, false);
    BOOST_REQUIRE_EQUAL(tr.bits.size(), m0.size());
    BOOST_REQUIRE_EQUAL(tr.states.size(), m0.size() + 1);
    BOOST_CHECK(std::equal(tr.bits.begin(), tr.bits.end(), ref.bits.begin()));
    BOOST_CHECK(std::equal(tr.states.begin(), tr.states.end(),
                           ref.states.begin()));
    BOOST_CHECK(std::equal(tr.lose_u.begin(), tr.lose_u.end(),
                           ref.lose_u.begin()));
    for (size_t t = 0; t < m0.size(); ++t)
        BOOST_CHECK_EQUAL(tr.margins[t], ref.margins[t]);

    // Deterministic tie stress: zero metrics make every edge cost 0, so all
    // ACS comparisons tie and only the strict `<` / first-edge rule decides.
    std::vector<float> z0(6, 0.0f), z1(6, 0.0f);
    std::vector<int8_t> tie_bits;
    vitdec_cl3_biorth(z0.data(), z1.data(), z0.size(), tie_bits);
    const QaPath tie_ref = qa_ref_dp(z0, z1, false);
    BOOST_CHECK_EQUAL_COLLECTIONS(tie_bits.begin(), tie_bits.end(),
                                  tie_ref.bits.begin(), tie_ref.bits.end());
    const size_t tie_count = qa_count_biorth_ties(z0, z1);
    BOOST_CHECK_GT(tie_count, size_t(0));
    BOOST_TEST_MESSAGE("joint-ML tie stress: " << tie_count
                                               << " exact ACS ties");
}

BOOST_AUTO_TEST_CASE(test_w1_joint_ml_awgn_gain_over_split_llr)
{
    using namespace openuwb::demod::core::detail;

    // 600 frames x 128 coded bits at sigma = 0.4 places the split-LLR FER in
    // the requested 20-40% band while the joint ML decoder is far below it.
    constexpr size_t kFrames = 600;
    constexpr size_t kBits = 128;
    constexpr float kSigma = 0.4f;
    std::mt19937 rng(20261120u);
    std::normal_distribution<float> nd(0.0f, kSigma);
    std::mt19937 bit_rng(0x5eed1234u);

    size_t split_fer = 0, biorth_fer = 0;
    size_t split_err = 0, biorth_err = 0, total_bits = 0, ties = 0;
    for (size_t f = 0; f < kFrames; ++f) {
        std::vector<int8_t> u(kBits);
        for (size_t t = 0; t < kBits; ++t)
            u[t] = static_cast<int8_t>(bit_rng() & 1u);
        std::vector<int8_t> g0, g1;
        qa_cl3_encode(u, g0, g1);
        std::vector<float> m0, m1;
        qa_map_bpm(g0, g1, 1.0f, m0, m1);
        for (size_t t = 0; t < kBits; ++t) {
            m0[t] += nd(rng);
            m1[t] += nd(rng);
        }

        std::vector<float> l0, l1;
        qa_split_llrs(m0, m1, l0, l1);
        std::vector<int8_t> split_bits;
        vitdec_cl3_soft(l0.data(), l1.data(), kBits, split_bits);
        std::vector<int8_t> biorth_bits;
        vitdec_cl3_biorth(m0.data(), m1.data(), kBits, biorth_bits);
        BOOST_REQUIRE_EQUAL(split_bits.size(), kBits);
        BOOST_REQUIRE_EQUAL(biorth_bits.size(), kBits);

        size_t se = 0, be = 0;
        for (size_t t = 0; t < kBits; ++t) {
            se += (split_bits[t] != u[t]);
            be += (biorth_bits[t] != u[t]);
        }
        split_err += se;
        biorth_err += be;
        split_fer += (se != 0);
        biorth_fer += (be != 0);
        total_bits += kBits;
        ties += qa_count_biorth_ties(m0, m1);
    }

    const double split_fer_pct = 100.0 * split_fer / kFrames;
    const double biorth_fer_pct = 100.0 * biorth_fer / kFrames;
    const double split_ber = static_cast<double>(split_err) / total_bits;
    const double biorth_ber = static_cast<double>(biorth_err) / total_bits;
    BOOST_TEST_MESSAGE("AWGN joint-ML gain (" << kFrames << " frames x "
                       << kBits << " bits, sigma=" << kSigma
                       << "): split FER=" << split_fer_pct
                       << "% BER=" << split_ber << "; biorth FER="
                       << biorth_fer_pct << "% BER=" << biorth_ber
                       << "; exact biorth ACS ties=" << ties);

    // The test is only informative if the split baseline sits in the target
    // band; then both BER and FER must be strictly lower for joint ML.
    BOOST_REQUIRE_GE(split_fer_pct, 20.0);
    BOOST_REQUIRE_LE(split_fer_pct, 40.0);
    BOOST_CHECK_LT(biorth_err, split_err);
    BOOST_CHECK_LT(biorth_fer, split_fer);
    BOOST_CHECK_LT(biorth_err * 4, split_err);
    BOOST_CHECK_LT(biorth_fer * 4, split_fer);
}

BOOST_AUTO_TEST_CASE(test_w1_joint_ml_terminated_tail_beats_truncated)
{
    using namespace openuwb::demod::core::detail;

    // 400 frames with the standard two zero tail bits.  Only the last eight
    // symbols carry noise, so the endpoint decision is what matters.
    constexpr size_t kFrames = 400;
    constexpr size_t kBits = 64;
    constexpr float kTailSigma = 1.0f;
    std::mt19937 rng(20261121u);
    std::normal_distribution<float> nd(0.0f, kTailSigma);
    std::mt19937 bit_rng(0x7151245u);

    size_t term_tail_err = 0, trunc_tail_err = 0;
    size_t clean_mismatch = 0, term_frame_err = 0, trunc_frame_err = 0;
    for (size_t f = 0; f < kFrames; ++f) {
        std::vector<int8_t> u(kBits, int8_t(0));
        for (size_t t = 0; t + 2 < kBits; ++t)
            u[t] = static_cast<int8_t>(bit_rng() & 1u);
        std::vector<int8_t> g0, g1;
        qa_cl3_encode(u, g0, g1);
        std::vector<float> clean0, clean1;
        qa_map_bpm(g0, g1, 1.0f, clean0, clean1);

        // Noiseless: terminated and truncated must agree bit for bit.
        std::vector<int8_t> clean_t, clean_u2;
        vitdec_cl3_biorth(clean0.data(), clean1.data(), kBits, clean_t, true);
        vitdec_cl3_biorth(clean0.data(), clean1.data(), kBits, clean_u2, false);
        if (clean_t != clean_u2 || clean_t != u)
            ++clean_mismatch;

        std::vector<float> m0 = clean0, m1 = clean1;
        for (size_t t = kBits - 8; t < kBits; ++t) {
            m0[t] += nd(rng);
            m1[t] += nd(rng);
        }
        std::vector<int8_t> term, trunc;
        vitdec_cl3_biorth(m0.data(), m1.data(), kBits, term, true);
        vitdec_cl3_biorth(m0.data(), m1.data(), kBits, trunc, false);
        for (size_t t = kBits - 16; t < kBits; ++t) {
            term_tail_err += (term[t] != u[t]);
            trunc_tail_err += (trunc[t] != u[t]);
        }
        term_frame_err += (term != u);
        trunc_frame_err += (trunc != u);
    }

    BOOST_TEST_MESSAGE("tail-noise test: last-16 errors terminated="
                       << term_tail_err << " truncated=" << trunc_tail_err
                       << "; frame errors terminated=" << term_frame_err
                       << " truncated=" << trunc_frame_err);
    BOOST_CHECK_EQUAL(clean_mismatch, size_t(0));
    BOOST_CHECK_LT(term_tail_err * 4, trunc_tail_err * 3);
    BOOST_CHECK_LT(term_frame_err, trunc_frame_err);
}

BOOST_AUTO_TEST_CASE(test_w1_joint_ml_pulse_clipping_recovers_fcs)
{
    using namespace openuwb::demod::core::detail;

    // Direct clipping contract: median one, 50x spikes clamped to 4x.
    {
        std::vector<float> a(32, 1.0f), b(32, 0.0f);
        a[4] = 50.0f;
        a[13] = -50.0f;
        b[22] = 50.0f;
        clip_biorth_metrics(a.data(), b.data(), a.size(), 4.0f);
        for (size_t i = 0; i < a.size(); ++i) {
            BOOST_CHECK_LE(std::abs(a[i]), 4.0f);
            BOOST_CHECK_LE(std::abs(b[i]), 4.0f);
        }
        BOOST_CHECK_CLOSE(a[4], 4.0f, 1e-6);
        BOOST_CHECK_CLOSE(a[13], -4.0f, 1e-6);
        BOOST_CHECK_CLOSE(b[22], 4.0f, 1e-6);
        BOOST_CHECK_CLOSE(a[0], 1.0f, 1e-6);
    }
    // median <= 0 and N < 16 skip safely (buffer untouched).
    {
        std::vector<float> a(32, 0.0f), b(32, 0.0f);
        clip_biorth_metrics(a.data(), b.data(), a.size(), 4.0f);
        for (size_t i = 0; i < a.size(); ++i)
            BOOST_CHECK_EQUAL(a[i], 0.0f);
        std::vector<float> c(8, 50.0f), d(8, -50.0f);
        clip_biorth_metrics(c.data(), d.data(), c.size(), 4.0f);
        BOOST_CHECK_EQUAL(c[0], 50.0f);
        BOOST_CHECK_EQUAL(d[7], -50.0f);
    }

    // 24-byte PSDU (one RS block, 240 coded payload symbols).  Five 50x
    // pulses on spaced symbols destroy the unclipped path metric ordering;
    // after clipping the residual errors stay within RS t = 4.
    size_t clipped_pass = 0, unclipped_fail = 0;
    constexpr size_t kFrames = 64;
    for (size_t f = 0; f < kFrames; ++f) {
        QaFrame frame;
        BOOST_REQUIRE(qa_build_frame(24, 1000u + static_cast<uint32_t>(f), frame));
        BOOST_REQUIRE_EQUAL(frame.payload_symbols, size_t(240));

        std::vector<float> m0 = frame.m0, m1 = frame.m1;
        const size_t pulse_pos[5] = { 60, 100, 140, 180, 220 };
        for (size_t i = 0; i < 5; ++i) {
            const float amp = (i % 2 == 0) ? 50.0f : -50.0f;
            m0[21 + pulse_pos[i]] += amp;
        }

        PayloadDecodeWorkspace ws;
        PayloadResult clipped;
        const bool clipped_ok = decode_payload_biorth_soft(
            frame.m0.data(), frame.m1.data(), m0.data() + 21, m1.data() + 21,
            frame.payload_symbols, frame.psdu_bits, clipped, nullptr, &ws,
            false, true);
        if (clipped_ok && clipped.fcs_pass && clipped.bytes == frame.bytes)
            ++clipped_pass;

        PayloadResult plain;
        decode_payload_biorth_soft(
            frame.m0.data(), frame.m1.data(), m0.data() + 21, m1.data() + 21,
            frame.payload_symbols, frame.psdu_bits, plain, nullptr, &ws,
            false, false);
        if (!(plain.ok && plain.fcs_pass))
            ++unclipped_fail;
    }
    BOOST_TEST_MESSAGE("pulse clipping: clipped FCS " << clipped_pass << "/"
                       << kFrames << ", unclipped failures " << unclipped_fail
                       << "/" << kFrames);
    BOOST_CHECK_EQUAL(clipped_pass, kFrames);
}

BOOST_AUTO_TEST_CASE(test_w1_joint_ml_end_to_end_and_boundaries)
{
    using namespace openuwb::demod::core::detail;
    using openuwb::demod::core::decode_phr_decisions;

    // ---- Payload: biorth must agree with decode_payload_soft() ------------
    QaFrame frame;
    BOOST_REQUIRE(qa_build_frame(24, 777u, frame));
    std::vector<float> phr0(frame.m0.begin(), frame.m0.begin() + 21);
    std::vector<float> phr1(frame.m1.begin(), frame.m1.begin() + 21);
    std::vector<float> pay0(frame.m0.begin() + 21, frame.m0.end());
    std::vector<float> pay1(frame.m1.begin() + 21, frame.m1.end());
    std::vector<float> l0, l1;
    qa_split_llrs(pay0, pay1, l0, l1);
    std::vector<float> l0p, l1p;
    qa_split_llrs(phr0, phr1, l0p, l1p);

    PayloadResult soft_out, biorth_out, term_out;
    PayloadDecodeWorkspace ws;
    BOOST_REQUIRE(decode_payload_soft(l0p, l1p, l0, l1, frame.psdu_bits,
                                      soft_out, nullptr, &ws));
    BOOST_REQUIRE(soft_out.fcs_pass);
    BOOST_CHECK(soft_out.bytes == frame.bytes);
    BOOST_REQUIRE(decode_payload_biorth_soft(
        phr0.data(), phr1.data(), pay0.data(), pay1.data(),
        frame.payload_symbols, frame.psdu_bits, biorth_out, nullptr, &ws));
    BOOST_CHECK(biorth_out.fcs_pass);
    BOOST_CHECK(biorth_out.bytes == soft_out.bytes);
    BOOST_CHECK_EQUAL(biorth_out.received_fcs, soft_out.received_fcs);
    BOOST_CHECK_EQUAL(biorth_out.calculated_fcs, soft_out.calculated_fcs);
    BOOST_REQUIRE(decode_payload_biorth_soft(
        phr0.data(), phr1.data(), pay0.data(), pay1.data(),
        frame.payload_symbols, frame.psdu_bits, term_out, nullptr, &ws, true));
    BOOST_CHECK(term_out.fcs_pass);
    BOOST_CHECK(term_out.bytes == frame.bytes);

    // ---- PHR: biorth vs decode_phr_decisions() on the same 21 symbols -----
    const std::vector<int8_t> sys13 = { 1, 0, 0, 1, 0, 0, 0, 0,
                                        0, 0, 0, 0, 1 };
    std::vector<int8_t> phr19;
    hrp_secded(sys13, phr19);
    std::vector<int8_t> phr_u(phr19.begin(), phr19.end());
    phr_u.push_back(0);
    phr_u.push_back(0);
    std::vector<int8_t> pg0, pg1;
    qa_cl3_encode(phr_u, pg0, pg1);
    std::vector<float> pm0, pm1;
    qa_map_bpm(pg0, pg1, 1.0f, pm0, pm1);
    const std::vector<float> empty_soft;
    NsSfdResult ns;
    ns.ok = true;
    ns.sfd_start_chip = -1;
    ns.sfd_end_chip = -1;
    ns.polarity = 1;

    PhrResult want, got;
    BOOST_REQUIRE(decode_phr_decisions(pg0, pg1, empty_soft, ns, want));
    BOOST_REQUIRE(decode_phr_biorth_soft(pm0.data(), pm1.data(), empty_soft,
                                         ns, got));
    BOOST_CHECK_EQUAL(got.ok, want.ok);
    BOOST_CHECK_EQUAL(got.psdu_length, want.psdu_length);
    BOOST_CHECK_EQUAL(got.data_rate_mbps, want.data_rate_mbps);
    BOOST_CHECK(got.phr_bits == want.phr_bits);
    BOOST_CHECK_EQUAL(got.psdu_length, uint32_t(32));
    BOOST_CHECK_EQUAL(got.data_rate_mbps, 6.81f);

    // Light noise: derive the hard decisions exactly like decide_bpm_symbol()
    // and require both PHR entries to keep agreeing.
    std::mt19937 rng(20261122u);
    std::normal_distribution<float> light(0.0f, 0.15f);
    std::vector<float> nm0 = pm0, nm1 = pm1;
    for (size_t t = 0; t < nm0.size(); ++t) {
        nm0[t] += light(rng);
        nm1[t] += light(rng);
    }
    std::vector<int8_t> hg0(21), hg1(21);
    for (size_t t = 0; t < 21; ++t) {
        const bool use1 = std::abs(nm1[t]) > std::abs(nm0[t]);
        hg0[t] = use1 ? 1 : 0;
        const float v = use1 ? nm1[t] : nm0[t];
        hg1[t] = (v < 0.0f) ? 1 : 0;
    }
    PhrResult hw, hb;
    BOOST_REQUIRE(decode_phr_decisions(hg0, hg1, empty_soft, ns, hw));
    BOOST_REQUIRE(decode_phr_biorth_soft(nm0.data(), nm1.data(), empty_soft,
                                         ns, hb));
    BOOST_CHECK_EQUAL(hb.ok, hw.ok);
    BOOST_CHECK_EQUAL(hb.psdu_length, hw.psdu_length);
    BOOST_CHECK_EQUAL(hb.data_rate_mbps, hw.data_rate_mbps);
    BOOST_CHECK(hb.phr_bits == hw.phr_bits);

    // ---- Boundary handling: no crashes, safe false on unusable input ------
    PayloadResult bad;
    BOOST_CHECK(!decode_payload_biorth_soft(nullptr, phr1.data(), pay0.data(),
                                            pay1.data(), 240, 192, bad));
    BOOST_CHECK(!decode_payload_biorth_soft(phr0.data(), nullptr, pay0.data(),
                                            pay1.data(), 240, 192, bad));
    BOOST_CHECK(!decode_payload_biorth_soft(phr0.data(), phr1.data(), nullptr,
                                            pay1.data(), 240, 192, bad));
    BOOST_CHECK(!decode_payload_biorth_soft(phr0.data(), phr1.data(),
                                            pay0.data(), pay1.data(), 0, 192,
                                            bad));
    // psdu_bits far beyond what 8 payload symbols can carry -> RS fails.
    BOOST_CHECK(!decode_payload_biorth_soft(phr0.data(), phr1.data(),
                                            pay0.data(), pay1.data(), 8,
                                            8 * 400, bad));

    PhrResult bad_phr;
    BOOST_CHECK(!decode_phr_biorth_soft(nullptr, pm1.data(), empty_soft, ns,
                                        bad_phr));
    BOOST_CHECK(!decode_phr_biorth_soft(pm0.data(), nullptr, empty_soft, ns,
                                        bad_phr));
}

namespace {

// Independent nested-loop local CL-3 re-solve with the joint ML metric.
void qa_ref_local_flip(const std::vector<float>& m0,
                       const std::vector<float>& m1,
                       size_t lo,
                       size_t hi,
                       int start_st,
                       int end_st,
                       const size_t* force_t,
                       const uint8_t* force_u,
                       size_t n_force,
                       std::vector<int8_t>& out_bits)
{
    const float INF = 1e30f;
    float surf[4] = { INF, INF, INF, INF };
    surf[start_st] = 0.0f;
    std::vector<int8_t> back((hi - lo) * 4, int8_t(0));
    for (size_t t = lo; t < hi; ++t) {
        int forced = -1;
        for (size_t f = 0; f < n_force; ++f)
            if (force_t[f] == t)
                forced = static_cast<int>(force_u[f]);
        float ns[4] = { INF, INF, INF, INF };
        for (int st = 0; st < 4; ++st) {
            if (surf[st] > 1e20f)
                continue;
            for (int u = 0; u < 2; ++u) {
                if (forced >= 0 && u != forced)
                    continue;
                const Cl3Transition step = cl3_transition(st, u);
                const float m = surf[st] + qa_branch(m0[t], m1[t], step.g0,
                                                     step.g1);
                if (m < ns[step.next_state]) {
                    ns[step.next_state] = m;
                    back[(t - lo) * 4 + step.next_state] =
                        static_cast<int8_t>((u << 2) | st);
                }
            }
        }
        for (int i = 0; i < 4; ++i)
            surf[i] = ns[i];
    }
    int cur = end_st;
    if (surf[cur] > 1e20f) {
        for (int s = 0; s < 4; ++s)
            if (surf[s] < surf[cur])
                cur = s;
    }
    for (size_t t = hi; t-- > lo;) {
        const int v = back[(t - lo) * 4 + cur];
        out_bits[t] = static_cast<int8_t>(v >> 2);
        cur = v & 3;
    }
}

} // namespace

BOOST_AUTO_TEST_CASE(test_w1_joint_ml_local_flip_and_rescue_phase1)
{
    using namespace openuwb::demod::core::detail;

    QaFrame frame;
    BOOST_REQUIRE(qa_build_frame(24, 4242u, frame));
    const size_t total = frame.u.size();
    // Mild noise so the trace has both usable margins and a real ACS order.
    std::mt19937 rng(20261123u);
    std::vector<float> m0 = frame.m0, m1 = frame.m1;
    qa_add_noise(m0, m1, 0.3f, rng);
    Cl3TrellisTrace tr;
    vitdec_cl3_biorth_trace(m0.data(), m1.data(), total, tr);
    BOOST_REQUIRE_EQUAL(tr.bits.size(), total);

    // local_cl3_trellis_flip_biorth must match the independent nested-loop
    // re-solve for unforced, single-forced and pair-forced windows.
    struct Window {
        size_t lo, hi;
        size_t ft[2];
        uint8_t fu[2];
        size_t n;
    };
    const Window wins[3] = {
        { 30, 70, { 0, 0 }, { 0, 0 }, 0 },
        { 100, 140, { 118, 0 }, { 1, 0 }, 1 },
        { 190, 235, { 200, 225 }, { 0, 1 }, 2 },
    };
    for (const auto& w : wins) {
        std::vector<int8_t> got(total, int8_t(7));
        std::vector<int8_t> ref(total, int8_t(7));
        local_cl3_trellis_flip_biorth(m0.data(), m1.data(), w.lo, w.hi,
                                      tr.states[w.lo], tr.states[w.hi],
                                      w.ft, w.fu, w.n, got);
        qa_ref_local_flip(m0, m1, w.lo, w.hi, tr.states[w.lo],
                          tr.states[w.hi], w.ft, w.fu, w.n, ref);
        for (size_t t = w.lo; t < w.hi; ++t)
            BOOST_CHECK_EQUAL(int(got[t]), int(ref[t]));
        for (size_t t = 0; t < total; ++t) {
            if (t < w.lo || t >= w.hi)
                BOOST_CHECK_EQUAL(int(got[t]), 7);
        }
    }

    // Phase 1 of rescue_payload_biorth: a trace whose codeword has one or two
    // bit errors inside different RS symbols must be recovered by RS alone.
    for (size_t nerr : { size_t(1), size_t(2) }) {
        Cl3TrellisTrace bad = tr;
        for (size_t k = 0; k < nerr; ++k)
            bad.bits[19 + 6 * (3 * k + 1) + 1] ^= 1;
        PayloadResult out;
        const bool ok = rescue_payload_biorth(m0.data(), m1.data(), total,
                                              frame.psdu_bits, bad, out);
        BOOST_REQUIRE(ok);
        BOOST_CHECK(out.fcs_pass);
        BOOST_CHECK(out.bytes == frame.bytes);
        BOOST_CHECK_EQUAL(out.received_fcs, out.calculated_fcs);
    }

    // Heavily corrupted trace must never be accepted without a CRC match;
    // failing safely is equally valid (the rescue is allowed to give up).
    Cl3TrellisTrace garbage = tr;
    for (size_t t = 19; t + 2 < total; t += 7)
        garbage.bits[t] ^= 1;
    PayloadResult bad_out;
    if (rescue_payload_biorth(m0.data(), m1.data(), total, frame.psdu_bits,
                              garbage, bad_out)) {
        BOOST_CHECK(bad_out.fcs_pass);
        BOOST_CHECK_EQUAL(bad_out.received_fcs, bad_out.calculated_fcs);
    }
    // Contract guards mirror rescue_payload_soft().
    PayloadResult guard;
    BOOST_CHECK(!rescue_payload_biorth(m0.data(), m1.data(), total, 0, tr, guard));
    BOOST_CHECK(!rescue_payload_biorth(m0.data(), m1.data(), total, 191, tr, guard));
}
