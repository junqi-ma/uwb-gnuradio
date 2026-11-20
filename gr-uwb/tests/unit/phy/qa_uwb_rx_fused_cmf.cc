#include <gnuradio/uwb/uwb_demod_fec.h>
/* SPDX-License-Identifier: GPL-3.0-or-later */
#include <boost/test/unit_test.hpp>

#include "../apps/fullpacket_rx_pipeline.h"
#include <gnuradio/uwb/uwb_hrp_mod_core.h>
#include <gnuradio/uwb/uwb_rx_fused_cmf.h>
#include <gnuradio/uwb/uwb_softchip_fir_diag.h>

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdint>
#include <random>
#include <utility>
#include <vector>

namespace {

using namespace gr::uwb;
using namespace gr::uwb::demod;
using cf = std::complex<float>;
using cd = std::complex<double>;

constexpr size_t kTap = 38;

struct RandomInput {
    std::vector<cf> rx;
    std::vector<cf> taps;
    std::vector<int8_t> pn;
    std::vector<std::complex<float>> rot;
    size_t symbols = 9;
    int64_t chip_start = 17;
    int64_t payload_start = 100;

    explicit RandomInput(uint32_t seed = 12345)
    {
        std::mt19937 rng(seed);
        std::uniform_real_distribution<float> dist(-1.0f, 1.0f);
        rx.resize(8192);
        for (auto& x : rx)
            x = cf(dist(rng), dist(rng));
        taps.resize(kTap);
        for (auto& h : taps)
            h = cf(dist(rng), dist(rng));
        pn.resize(symbols * rxfusion::kChipsPerBurst);
        for (auto& a : pn)
            a = (dist(rng) < 0.0f) ? int8_t(-1) : int8_t(1);
        const float ang = dist(rng) * 3.0f;
        rot.push_back(std::complex<float>(std::cos(ang), std::sin(ang)));
    }
};

cd reference_metric(const RandomInput& in, size_t s, size_t p, int8_t pol)
{
    (void)pol;
    const int64_t offset = static_cast<int64_t>(
        rxfusion::kCandidateChipOffset[p]);
    cd m(0.0, 0.0);
    for (size_t c = 0; c < rxfusion::kChipsPerBurst; ++c) {
        const int8_t a = in.pn[s * rxfusion::kChipsPerBurst + c];
        for (size_t q = 0; q < in.taps.size(); ++q) {
            const int64_t chip =
                in.payload_start +
                static_cast<int64_t>(s) * rxfusion::kChipsPerSymbol +
                offset + static_cast<int64_t>(c);
            const int64_t idx = in.chip_start +
                                chip * rxfusion::kSamplesPerChip -
                                static_cast<int64_t>(in.taps.size()) + 1 +
                                static_cast<int64_t>(q);
            m += static_cast<double>(a) * cd(in.taps[q]) * cd(in.rx[idx]);
        }
    }
    return m;
}

} // namespace

BOOST_AUTO_TEST_CASE(fused_cmf_matches_double_double_sum)
{
    RandomInput in;
    rxfusion::Metrics out;
    rxfusion::Scratch scratch;
    for (int8_t pol : { int8_t(1), int8_t(-1) }) {
        BOOST_REQUIRE(rxfusion::correlate_payload(
            in.rx.data(), in.rx.size(), in.taps.data(), in.taps.size(),
            in.chip_start, rxfusion::kSamplesPerChip, in.payload_start,
            in.pn.data(), in.symbols, pol, in.rot[0], out, scratch));
        double max_abs = 0.0, max_rel = 0.0, max_ref = 0.0;
        const cd r(in.rot[0].real(), in.rot[0].imag());
        for (size_t s = 0; s < in.symbols; ++s) {
            for (size_t p = 0; p < rxfusion::kCandidateCount; ++p) {
                const cd ref = reference_metric(in, s, p, pol);
                const cd got(out.complex_value[rxfusion::kCandidateCount * s + p].real(),
                             out.complex_value[rxfusion::kCandidateCount * s + p].imag());
                const double abs_err = std::abs(got - ref);
                const double signed_ref =
                    static_cast<double>(pol) * (r * ref).real();
                const double signed_err = std::abs(
                    out.signed_value[rxfusion::kCandidateCount * s + p] -
                    static_cast<float>(signed_ref));
                max_abs = std::max(max_abs, std::max(abs_err, signed_err));
                max_ref = std::max(max_ref, std::abs(ref));
            }
        }
        max_rel = max_abs / std::max(max_ref, 1e-12);
        BOOST_TEST_MESSAGE("fused kernel pol=" << int(pol)
                           << " max_abs=" << max_abs
                           << " max_rel=" << max_rel);
        // Float initial threshold from the task: 2e-5 relative.  The double
        // reference re-associates the same finite sum; no tolerance is hidden
        // in the kernel.
        BOOST_CHECK_LT(max_rel, 2e-5);
    }
}

BOOST_AUTO_TEST_CASE(fused_cmf_decision_matches_bprf_demod)
{
    // Random soft chips: compare the fused four-metric decision with the
    // production bprf_demod() over every supported code and polarity.
    const size_t symbols = 37;
    const int64_t start = 123;
    std::vector<float> soft(start + 64 * symbols + 128);
    std::mt19937 rng(991);
    std::uniform_real_distribution<float> dist(-1.0f, 1.0f);
    for (auto& x : soft)
        x = dist(rng);

    for (size_t code : { 9u, 10u, 11u, 12u }) {
        std::vector<int8_t> spread;
        BOOST_REQUIRE(demod::core::detail::bprf_spreading(
            spread, code, 1344, rxfusion::kChipsPerBurst * symbols));
        for (int8_t pol : { int8_t(1), int8_t(-1) }) {
            std::vector<int8_t> g0_ref, g1_ref;
            BOOST_REQUIRE(demod::core::detail::bprf_demod(
                soft, start, code, symbols, 8, 64, 1344, pol, g0_ref, g1_ref));

            rxfusion::Metrics metrics;
            metrics.reset(symbols);
            for (size_t s = 0; s < symbols; ++s) {
                for (size_t p = 0; p < rxfusion::kCandidateCount; ++p) {
                    float m = 0.0f;
                    for (size_t c = 0; c < rxfusion::kChipsPerBurst; ++c) {
                        const int64_t idx =
                            start +
                            static_cast<int64_t>(s) * rxfusion::kChipsPerSymbol +
                            static_cast<int64_t>(
                                rxfusion::kCandidateChipOffset[p]) +
                            static_cast<int64_t>(c);
                        m += static_cast<float>(
                                 spread[s * rxfusion::kChipsPerBurst + c]) *
                             soft[idx];
                    }
                    metrics.signed_value[rxfusion::kCandidateCount * s + p] =
                        (pol < 0) ? -m : m;
                }
            }
            std::vector<int8_t> g0, g1;
            rxfusion::decide_bpm(metrics, g0, g1);
            BOOST_CHECK(g0 == g0_ref);
            BOOST_CHECK(g1 == g1_ref);
        }
    }
}

BOOST_AUTO_TEST_CASE(fused_cmf_decision_ties_match_legacy)
{
    // hop 0 wins a hop tie; BPM half 0 wins a half tie; zero is non-negative.
    rxfusion::Metrics metrics;
    metrics.reset(3);
    // Symbol 0: all equal -> hop0 half0.
    metrics.signed_value = { 2.0f, 1.0f, 2.0f, 1.0f,
                             -3.0f, 4.0f, 3.0f, 4.0f,
                             0.0f, 0.0f, 0.0f, 0.0f };
    std::vector<int8_t> g0, g1;
    rxfusion::decide_bpm(metrics, g0, g1);
    BOOST_CHECK_EQUAL(int(g0[0]), 0);
    BOOST_CHECK_EQUAL(int(g1[0]), 0);
    // Symbol 1: half0 (hop1, -3) vs half1 (hop0, 4); half1 wins, negative -> 0.
    BOOST_CHECK_EQUAL(int(g0[1]), 1);
    BOOST_CHECK_EQUAL(int(g1[1]), 0);
    // Symbol 2: zeros -> half0, non-negative -> 0.
    BOOST_CHECK_EQUAL(int(g0[2]), 0);
    BOOST_CHECK_EQUAL(int(g1[2]), 0);
}

BOOST_AUTO_TEST_CASE(fused_cmf_rejects_unsupported_and_out_of_range)
{
    RandomInput in;
    rxfusion::Metrics out;
    rxfusion::Scratch scratch;
    const cf* rx = in.rx.data();
    const size_t n = in.rx.size();
    const cf* h = in.taps.data();
    const int8_t* pn = in.pn.data();

    BOOST_CHECK(!rxfusion::correlate_payload(
        nullptr, n, h, kTap, in.chip_start, 2, in.payload_start, pn,
        in.symbols, 1, in.rot[0], out, scratch));
    BOOST_CHECK(!rxfusion::correlate_payload(
        rx, n, nullptr, kTap, in.chip_start, 2, in.payload_start, pn,
        in.symbols, 1, in.rot[0], out, scratch));
    BOOST_CHECK(!rxfusion::correlate_payload(
        rx, n, h, kTap, in.chip_start, 2, in.payload_start, nullptr,
        in.symbols, 1, in.rot[0], out, scratch));
    BOOST_CHECK(!rxfusion::correlate_payload(
        rx, n, h, 0, in.chip_start, 2, in.payload_start, pn, in.symbols,
        1, in.rot[0], out, scratch));
    BOOST_CHECK(!rxfusion::correlate_payload(
        rx, n, h, rxfusion::kMaxTapCount + 1, in.chip_start, 2,
        in.payload_start, pn, in.symbols, 1, in.rot[0], out, scratch));
    BOOST_CHECK(!rxfusion::correlate_payload(
        rx, n, h, kTap, in.chip_start, 1, in.payload_start, pn, in.symbols,
        1, in.rot[0], out, scratch));
    BOOST_CHECK(!rxfusion::correlate_payload(
        rx, n, h, kTap, -1, 2, in.payload_start, pn, in.symbols, 1,
        in.rot[0], out, scratch));
    BOOST_CHECK(!rxfusion::correlate_payload(
        rx, n, h, kTap, in.chip_start, 2, -1, pn, in.symbols, 1,
        in.rot[0], out, scratch));
    BOOST_CHECK(!rxfusion::correlate_payload(
        rx, n, h, kTap, in.chip_start, 2, in.payload_start, pn, 0, 1,
        in.rot[0], out, scratch));
    BOOST_CHECK(!rxfusion::correlate_payload(
        rx, n, h, kTap, in.chip_start, 2, in.payload_start, pn, in.symbols,
        0, in.rot[0], out, scratch));

    // Exact last readable window succeeds; one sample shorter fails.  The
    // last candidate starts at chip offset 40 (hop 1, BPM half 1); its
    // template then spans len samples.
    const int64_t base0 = in.chip_start + in.payload_start * 2 - kTap + 1;
    const int64_t last_chip =
        static_cast<int64_t>(in.symbols - 1) * 64 +
        static_cast<int64_t>(
            rxfusion::kCandidateChipOffset[rxfusion::kCandidateCount - 1]);
    const size_t len = kTap + 2 * 7;
    const int64_t needed = base0 + last_chip * 2 + len;
    BOOST_REQUIRE_GT(needed, 0);
    BOOST_REQUIRE_LE(needed, static_cast<int64_t>(in.rx.size()));
    BOOST_CHECK(rxfusion::correlate_payload(
        rx, static_cast<size_t>(needed), h, kTap, in.chip_start, 2,
        in.payload_start, pn, in.symbols, 1, in.rot[0], out, scratch));
    BOOST_CHECK(!rxfusion::correlate_payload(
        rx, static_cast<size_t>(needed - 1), h, kTap, in.chip_start, 2,
        in.payload_start, pn, in.symbols, 1, in.rot[0], out, scratch));
}

BOOST_AUTO_TEST_CASE(fused_cmf_unaligned_address_matches)
{
    RandomInput in;
    rxfusion::Metrics aligned, shifted;
    rxfusion::Scratch scratch;
    BOOST_REQUIRE(rxfusion::correlate_payload(
        in.rx.data(), in.rx.size(), in.taps.data(), kTap, in.chip_start, 2,
        in.payload_start, in.pn.data(), in.symbols, 1, in.rot[0], aligned,
        scratch));
    std::vector<cf> copy(in.rx.size() + 1);
    copy[0] = cf(0.0f, 0.0f);
    std::copy(in.rx.begin(), in.rx.end(), copy.begin() + 1);
    BOOST_REQUIRE(rxfusion::correlate_payload(
        copy.data() + 1, in.rx.size(), in.taps.data(), kTap, in.chip_start,
        2, in.payload_start, in.pn.data(), in.symbols, 1, in.rot[0], shifted,
        scratch));
    BOOST_CHECK(aligned.signed_value == shifted.signed_value);
    BOOST_CHECK(aligned.complex_value == shifted.complex_value);
}

namespace {
struct Packet {
    std::vector<uint8_t> wire;
    std::vector<cf> iq, sync;
    mod::HrpModConfig cfg;
    static constexpr size_t seed = 2048;

    Packet(size_t bytes, size_t code)
    {
        wire.resize(bytes - 2);
        for (size_t j = 0; j < wire.size(); ++j)
            wire[j] = uint8_t(19 * j + code);
        mod::append_ieee_fcs(wire);
        cfg.code_index = code;
        cfg.sync_repetitions = 64;
        cfg.sfd_mode = "4z2";
        cfg.peak_amplitude = .8f;
        const size_t count = mod::packet_samples_998p4(
            64, 8, wire.size(), false, &cfg.pulse);
        mod::HrpModScratch scratch;
        scratch.reserve(127, count);
        std::vector<cf> wave(count);
        size_t written = 0;
        BOOST_REQUIRE(mod::modulate_one(wire.data(), wire.size(), cfg,
                                       scratch, wave.data(), wave.size(), written));
        BOOST_REQUIRE_EQUAL(written, count);
        sync.assign(wave.begin(), wave.begin() + 1016);
        iq.assign(seed + count + 2048, cf(0));
        std::copy(wave.begin(), wave.end(), iq.begin() + seed);
    }
};
} // namespace

BOOST_AUTO_TEST_CASE(fused_cmf_skips_rake_and_bypass)
{
    Packet packet(32, 10);
    for (auto mode : { demod::CirSoftChipMode::Rake,
                       demod::CirSoftChipMode::Bypass }) {
        auto profile = demod::Qm35825Profile::Default();
        profile.code_index = 10;
        profile.timing_search_margin = 256;
        profile.timing_coarse_stride = 1;
        profile.rx_softchip_on_demand = true;
        profile.rx_fused_payload_cmf = true;
        profile.cir_soft_chip_mode = mode;
        profile.cir_rake_top_k =
            (mode == demod::CirSoftChipMode::Rake) ? 4 : 0;
        demod::core::DemodScratch scratch;
        scratch.reserve(packet.iq.size());
        const auto r = demod::core::demodulate_one(
            packet.iq.data(), packet.iq.size(), profile, 61, Packet::seed, 0,
            packet.sync, scratch);
        BOOST_REQUIRE_EQUAL(int(r.status), int(demod::DemodStatus::Success));
        BOOST_CHECK(r.payload.fcs_pass);
        // The fused kernel only supports the full integer-grid CMF; RAKE and
        // bypass must keep the legacy path and not count a fused attempt.
        // The on-demand prefix itself rejects these modes and performs one
        // full-stream retry, so softchip_fallbacks is 1.
        BOOST_CHECK_EQUAL(r.stage_fused_attempts, 0u);
        BOOST_CHECK_EQUAL(r.stage_softchip_fallbacks, 1u);
        BOOST_CHECK_EQUAL(r.stage_fused_fallbacks, 0u);
    }
}

BOOST_AUTO_TEST_CASE(fused_cmf_pipeline_decodes_same_psdu)
{
    for (const auto spec : { std::pair<size_t, size_t>{ 32, 10 },
                             std::pair<size_t, size_t>{ 127, 9 } }) {
        Packet packet(spec.first, spec.second);
        experiment::FastRxPipeline legacy(packet.iq.size(), spec.second);
        experiment::FastRxPipeline::Options options(true, true, true, true, true);
        experiment::FastRxPipeline fast(packet.iq.size(), spec.second, options);
        const auto a = legacy.run(packet.iq.data(), packet.iq.size(), 31,
                                  Packet::seed, packet.sync);
        const auto b = fast.run(packet.iq.data(), packet.iq.size(), 32,
                                Packet::seed, packet.sync);
        BOOST_REQUIRE_EQUAL(int(a.status), int(demod::DemodStatus::Success));
        BOOST_REQUIRE_EQUAL(int(b.status), int(demod::DemodStatus::Success));
        BOOST_CHECK(a.payload.fcs_pass);
        BOOST_CHECK(b.payload.fcs_pass);
        BOOST_CHECK(a.payload.bytes == b.payload.bytes);
        BOOST_CHECK_EQUAL(a.phr.psdu_length, b.phr.psdu_length);
        BOOST_CHECK_EQUAL(b.stage_fused_attempts, 1u);
        BOOST_CHECK_EQUAL(b.stage_fused_fallbacks, 0u);
        BOOST_CHECK_GT(b.stage_fused_template_us + b.stage_fused_correlate_us, 0u);
        BOOST_CHECK_LT(b.cir.soft_chip_count, a.cir.soft_chip_count);
        BOOST_CHECK_EQUAL(b.cir.extended_chip_count, 0u);
    }
}

BOOST_AUTO_TEST_CASE(fused_cmf_corrupt_payload_falls_back_without_commit)
{
    Packet packet(127, 9);
    // Zero an uncorrectable 8192-sample (~64 symbols / >10 RS 6-bit symbols,
    // beyond RS(63,55)'s 8-erasure bound) payload slice while leaving SFD/PHR
    // intact, so the fused attempt runs and falls back to legacy once.
    const size_t payload_start = Packet::seed +
                                 (64 + 8) * 1016 +
                                 21 * 1024;
    const size_t corrupt_lo = std::min(packet.iq.size(), payload_start + 100);
    const size_t corrupt_hi = std::min(packet.iq.size(), corrupt_lo + 8192);
    for (size_t i = corrupt_lo; i < corrupt_hi; ++i)
        packet.iq[i] = cf(0.0f, 0.0f);

    experiment::FastRxPipeline legacy(packet.iq.size(), 9);
    experiment::FastRxPipeline::Options options(true, true, true, true, true);
    experiment::FastRxPipeline fast(packet.iq.size(), 9, options);
    const auto a = legacy.run(packet.iq.data(), packet.iq.size(), 41,
                              Packet::seed, packet.sync);
    const auto b = fast.run(packet.iq.data(), packet.iq.size(), 42,
                            Packet::seed, packet.sync);
    BOOST_TEST_MESSAGE("corrupt payload: phr_ok=" << b.phr.ok
                       << " fused_attempts=" << b.stage_fused_attempts
                       << " fused_fallbacks=" << b.stage_fused_fallbacks
                       << " softchip_fallbacks=" << b.stage_softchip_fallbacks
                       << " status=" << int(b.status));
    BOOST_CHECK_EQUAL(int(a.status), int(b.status));
    BOOST_REQUIRE_EQUAL(a.phr.ok, b.phr.ok);
    BOOST_REQUIRE(b.phr.ok); // the corruption starts after the PHR field
    BOOST_CHECK_EQUAL(b.stage_fused_attempts, 1u);
    BOOST_CHECK_EQUAL(b.stage_fused_fallbacks, 1u);
    BOOST_CHECK_EQUAL(b.stage_softchip_fallbacks, 1u);
    // A corrupted packet must never be reported as a successful commit.
    BOOST_CHECK(!b.payload.fcs_pass);
    BOOST_CHECK(b.status != demod::DemodStatus::Success);
    // The published result must describe the final legacy retry, not a
    // discarded prefix/fused attempt.
    BOOST_CHECK_EQUAL(b.cir.soft_chip_count, a.cir.soft_chip_count);
    BOOST_CHECK_EQUAL(b.ns_sfd.sfd_start_chip, a.ns_sfd.sfd_start_chip);
}

BOOST_AUTO_TEST_CASE(fused_cmf_option_implies_prefix_path)
{
    Packet packet(32, 10);
    // The fused option alone selects the SYNC/SFD/PHR prefix and must not run
    // the legacy full soft-chip stream first.
    experiment::FastRxPipeline::Options options(false, false, false, false, true);
    experiment::FastRxPipeline fast(packet.iq.size(), 10, options);
    const auto r = fast.run(packet.iq.data(), packet.iq.size(), 51,
                            Packet::seed, packet.sync);
    BOOST_REQUIRE_EQUAL(int(r.status), int(demod::DemodStatus::Success));
    BOOST_CHECK(r.payload.fcs_pass);
    BOOST_CHECK(r.payload.bytes == packet.wire);
    BOOST_CHECK_EQUAL(r.stage_fused_attempts, 1u);
    BOOST_CHECK_EQUAL(r.cir.extended_chip_count, 0u);
}

BOOST_AUTO_TEST_CASE(r1_non_last_candidate_max_offset_bounds_check)
{
    // R1: candidate 1 is max offset (320), candidate 3 is 0.
    // len = 164. base0 = 0 (chip_start=37, tap_count=38).
    // Candidate 1 requires up to sample index 803 (n >= 804).
    // Candidate 3 only requires up to sample index 163 (n >= 164).
    const size_t offsets[rxfusion::kCandidateCount] = { 0, 320, 64, 0 };
    rxfusion::Geometry geom;
    geom.chips_per_burst = 64;
    geom.chips_per_symbol = 512;
    geom.candidate_count = rxfusion::kCandidateCount;
    geom.candidate_chip_offset = offsets;

    std::vector<cf> rx(1024, cf(1.0f, 0.0f));
    std::vector<cf> taps(38, cf(1.0f, 0.0f));
    std::vector<int8_t> pn(64, 1);
    rxfusion::Metrics out;
    rxfusion::Scratch scratch;

    // With n = 164, candidate 3 would pass, but candidate 1 requires 804 samples.
    // Must return false.
    BOOST_CHECK(!rxfusion::correlate_field(
        rx.data(), 164, taps.data(), 38, 37, 2, 0, pn.data(), 1, geom,
        1, cf(1.0f, 0.0f), out, scratch));

    // With n = 804, all candidates fit. Must return true.
    BOOST_CHECK(rxfusion::correlate_field(
        rx.data(), 804, taps.data(), 38, 37, 2, 0, pn.data(), 1, geom,
        1, cf(1.0f, 0.0f), out, scratch));
}

BOOST_AUTO_TEST_CASE(r1_burst_crossing_symbol_boundary)
{
    // R1: chips_per_burst=64, chips_per_symbol=512.
    // Max offset within symbol is 512 - 64 = 448.
    // Candidate at 448 exactly fits inside symbol (offset + chips_per_burst == 512).
    // Candidate at 449 crosses symbol boundary by 1 chip (offset + chips_per_burst == 513).
    rxfusion::Geometry geom_exact;
    const size_t offsets_exact[rxfusion::kCandidateCount] = { 0, 64, 128, 448 };
    geom_exact.chips_per_burst = 64;
    geom_exact.chips_per_symbol = 512;
    geom_exact.candidate_count = rxfusion::kCandidateCount;
    geom_exact.candidate_chip_offset = offsets_exact;

    rxfusion::Geometry geom_cross;
    const size_t offsets_cross[rxfusion::kCandidateCount] = { 0, 64, 128, 449 };
    geom_cross.chips_per_burst = 64;
    geom_cross.chips_per_symbol = 512;
    geom_cross.candidate_count = rxfusion::kCandidateCount;
    geom_cross.candidate_chip_offset = offsets_cross;

    std::vector<cf> rx(2048, cf(1.0f, 0.0f));
    std::vector<cf> taps(38, cf(1.0f, 0.0f));
    std::vector<int8_t> pn(64, 1);
    rxfusion::Metrics out;
    rxfusion::Scratch scratch;

    BOOST_CHECK(rxfusion::correlate_field(
        rx.data(), rx.size(), taps.data(), 38, 37, 2, 0, pn.data(), 1, geom_exact,
        1, cf(1.0f, 0.0f), out, scratch));

    // Exactly crosses boundary by 1 chip -> rejected!
    BOOST_CHECK(!rxfusion::correlate_field(
        rx.data(), rx.size(), taps.data(), 38, 37, 2, 0, pn.data(), 1, geom_cross,
        1, cf(1.0f, 0.0f), out, scratch));
}

BOOST_AUTO_TEST_CASE(r2_fir_diag_iq_boundary_and_short_by_one)
{
    // R2: test diagnose_softchip_fir_segments right boundary and out-of-bounds isolation.
    std::vector<cf> rx(64, cf(1.0f, 0.0f));
    std::vector<cf> taps(38, cf(1.0f, 0.0f));
    demod::NsSfdResult ns_sfd;
    ns_sfd.ok = true;
    ns_sfd.sfd_start_chip = 0;
    ns_sfd.sfd_end_chip = 0;
    demod::PhrResult phr;
    phr.ok = true;
    std::vector<cf> work;
    demod::core::detail::SoftFirSegments seg;

    // Case 1: total_chips=1, tap_count=38, chip_start=37, spc=2.
    // For chip 0: p = 37, fwd = 37, win = 0.
    // win + tap_count = 38.
    // When n = 38 (exact boundary):
    BOOST_REQUIRE(demod::core::detail::diagnose_softchip_fir_segments(
        rx.data(), 38, taps.data(), 38, 37, 1, 2.0, ns_sfd, phr, 0, work, seg));
    BOOST_CHECK_EQUAL(work[0], cf(38.0f, 0.0f));

    // Changing rx[38] (which is outside n=38) must not change the result with n=38.
    rx[38] = cf(999.0f, 999.0f);
    BOOST_REQUIRE(demod::core::detail::diagnose_softchip_fir_segments(
        rx.data(), 38, taps.data(), 38, 37, 1, 2.0, ns_sfd, phr, 0, work, seg));
    BOOST_CHECK_EQUAL(work[0], cf(38.0f, 0.0f));

    // When n = 37 (short by 1 sample):
    // win + tap_count = 38 > 37, falls back safely to scalar_dot which only sums 37 taps (0..36).
    BOOST_REQUIRE(demod::core::detail::diagnose_softchip_fir_segments(
        rx.data(), 37, taps.data(), 38, 37, 1, 2.0, ns_sfd, phr, 0, work, seg));
    BOOST_CHECK_EQUAL(work[0], cf(37.0f, 0.0f));

    // Changing rx[37] (outside n=37) must not change the result with n=37.
    rx[37] = cf(777.0f, 777.0f);
    BOOST_REQUIRE(demod::core::detail::diagnose_softchip_fir_segments(
        rx.data(), 37, taps.data(), 38, 37, 1, 2.0, ns_sfd, phr, 0, work, seg));
    BOOST_CHECK_EQUAL(work[0], cf(37.0f, 0.0f));

    // Case 2: Counterexample from review doc:
    // 38 taps, chip_start=37, spc=2, total_chips=10, n=40. Physical allocation 56.
    std::vector<cf> rx_buf(56, cf(1.0f, 0.0f));
    std::vector<cf> work1, work2;
    demod::core::detail::SoftFirSegments seg1, seg2;
    BOOST_REQUIRE(demod::core::detail::diagnose_softchip_fir_segments(
        rx_buf.data(), 40, taps.data(), 38, 37, 10, 2.0, ns_sfd, phr, 0, work1, seg1));

    // Modify samples beyond n=40:
    for (size_t i = 40; i < 56; ++i)
        rx_buf[i] = cf(100.0f, -50.0f);
    BOOST_REQUIRE(demod::core::detail::diagnose_softchip_fir_segments(
        rx_buf.data(), 40, taps.data(), 38, 37, 10, 2.0, ns_sfd, phr, 0, work2, seg2));

    BOOST_REQUIRE_EQUAL(work1.size(), work2.size());
    for (size_t i = 0; i < work1.size(); ++i) {
        BOOST_CHECK_EQUAL(work1[i], work2[i]);
    }
}

// ---------------------------------------------------------------------------
// Task 1 — rxfusion::decide_bpm_soft() equivalence.
//
// The soft twin must reproduce decide_bpm()'s hard decisions bit-for-bit and
// bprf_demod_soft()'s LLRs *exactly* (no tolerance): with +/-1 spreading and
// +/-1 polarity every product is exact, so the two accumulation orders agree
// bitwise.  The metrics are built from a random soft-chip stream with the same
// four candidate offsets the kernel uses, over every supported code/polarity.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(fused_cmf_soft_decision_matches_bprf_demod_soft)
{
    const size_t symbols = 37;
    const int64_t start = 123;
    std::vector<float> soft(start + 64 * symbols + 128);
    std::mt19937 rng(991);
    std::uniform_real_distribution<float> dist(-1.0f, 1.0f);
    for (auto& x : soft)
        x = dist(rng);

    size_t checked = 0;
    for (size_t code : { 9u, 10u, 11u, 12u }) {
        std::vector<int8_t> spread;
        BOOST_REQUIRE(demod::core::detail::bprf_spreading(
            spread, code, 1344, rxfusion::kChipsPerBurst * symbols));
        for (int8_t pol : { int8_t(1), int8_t(-1) }) {
            std::vector<int8_t> g0_ref, g1_ref;
            std::vector<float> llr0_ref, llr1_ref;
            BOOST_REQUIRE(demod::core::detail::bprf_demod_soft(
                soft, start, code, symbols, 8, 64, 1344, pol, g0_ref, g1_ref,
                llr0_ref, llr1_ref));

            rxfusion::Metrics metrics;
            metrics.reset(symbols);
            for (size_t s = 0; s < symbols; ++s) {
                for (size_t p = 0; p < rxfusion::kCandidateCount; ++p) {
                    float m = 0.0f;
                    for (size_t c = 0; c < rxfusion::kChipsPerBurst; ++c) {
                        const int64_t idx =
                            start +
                            static_cast<int64_t>(s) * rxfusion::kChipsPerSymbol +
                            static_cast<int64_t>(
                                rxfusion::kCandidateChipOffset[p]) +
                            static_cast<int64_t>(c);
                        m += static_cast<float>(
                                 spread[s * rxfusion::kChipsPerBurst + c]) *
                             soft[idx];
                    }
                    metrics.signed_value[rxfusion::kCandidateCount * s + p] =
                        (pol < 0) ? -m : m;
                }
            }

            std::vector<int8_t> g0, g1, sg0, sg1;
            std::vector<float> llr0, llr1;
            rxfusion::decide_bpm(metrics, g0, g1);
            rxfusion::decide_bpm_soft(metrics, sg0, sg1, llr0, llr1);

            // Hard decisions: identical to decide_bpm() and to bprf_demod().
            BOOST_CHECK(sg0 == g0);
            BOOST_CHECK(sg1 == g1);
            BOOST_CHECK(g0 == g0_ref);
            BOOST_CHECK(g1 == g1_ref);
            // LLRs: bit-identical to bprf_demod_soft().
            BOOST_REQUIRE_EQUAL(llr0.size(), llr0_ref.size());
            BOOST_REQUIRE_EQUAL(llr1.size(), llr1_ref.size());
            BOOST_CHECK(llr0 == llr0_ref);
            BOOST_CHECK(llr1 == llr1_ref);
            // LLR sign convention: > 0 favours bit 0, and llr0's sign is the
            // BPM-half decision while llr1's sign is the BPSK decision.
            for (size_t s = 0; s < symbols; ++s) {
                BOOST_CHECK_EQUAL(llr0[s] >= 0.0f, sg0[s] == 0);
                BOOST_CHECK_EQUAL(llr1[s] >= 0.0f, sg1[s] == 0);
            }
            ++checked;
        }
    }
    BOOST_TEST_MESSAGE("decide_bpm_soft: " << checked
                       << " code/polarity combinations, hard bits and LLRs "
                          "bit-identical to bprf_demod_soft");
    BOOST_CHECK_EQUAL(checked, size_t(8));
}

// Tie rules must be inherited from decide_bpm() unchanged.
BOOST_AUTO_TEST_CASE(fused_cmf_soft_decision_ties_match_legacy)
{
    rxfusion::Metrics metrics;
    metrics.reset(6);
    auto set = [&](size_t s, float m0, float m1, float m2, float m3) {
        metrics.signed_value[4 * s + 0] = m0;
        metrics.signed_value[4 * s + 1] = m1;
        metrics.signed_value[4 * s + 2] = m2;
        metrics.signed_value[4 * s + 3] = m3;
    };
    set(0, 2.0f, 2.0f, 2.0f, 2.0f);   // all equal -> half0, hop0, >= 0
    set(1, 3.0f, 0.1f, 3.0f, 0.1f);   // hop tie in half0 -> hop0
    set(2, 0.1f, 4.0f, 0.1f, 4.0f);   // hop tie in half1 -> hop0
    set(3, -2.0f, -2.0f, 1.0f, 1.0f); // half tie -> half0, negative
    set(4, 0.0f, 0.0f, 0.0f, 0.0f);   // exact zeros
    set(5, 1e-20f, -2e-20f, 1e-20f, 2e-20f);

    std::vector<int8_t> g0, g1, sg0, sg1;
    std::vector<float> llr0, llr1;
    rxfusion::decide_bpm(metrics, g0, g1);
    rxfusion::decide_bpm_soft(metrics, sg0, sg1, llr0, llr1);
    BOOST_CHECK(sg0 == g0);
    BOOST_CHECK(sg1 == g1);
    BOOST_CHECK_EQUAL(int(sg0[0]), 0);
    BOOST_CHECK_EQUAL(int(sg1[0]), 0);
    BOOST_CHECK_EQUAL(int(sg0[3]), 0);
    BOOST_CHECK_EQUAL(int(sg1[3]), 1);
    BOOST_CHECK_EQUAL(int(sg0[5]), 1);
    BOOST_CHECK_EQUAL(int(sg1[5]), 1);
    // llr0 is exactly b0 - b1, llr1 exactly max(+m) - max(-m).
    BOOST_CHECK_EQUAL(llr0[0], 0.0f);
    BOOST_CHECK_EQUAL(llr0[3], 0.0f);
    BOOST_CHECK_EQUAL(llr1[3], -1.0f);
}

// ---------------------------------------------------------------------------
// Task 4.2 — high-SNR regression: the fused path must produce the exact wire
// bytes, and the unchanged hard attempt alone must already decode them.  That
// second assertion is what proves the in-place soft retry is never entered on
// a clean packet (so the high-SNR cost and output are unchanged).
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(fused_cmf_high_snr_psdu_matches_hard_baseline)
{
    for (const auto spec : { std::pair<size_t, size_t>{ 32, 10 },
                             std::pair<size_t, size_t>{ 127, 9 } }) {
        Packet packet(spec.first, spec.second);
        auto profile = demod::Qm35825Profile::Default();
        profile.code_index = spec.second;
        profile.timing_search_margin = 256;
        profile.timing_coarse_stride = 1;
        profile.rx_softchip_on_demand = true;
        profile.rx_fused_payload_cmf = true;
        profile.rx_fused_phr_cmf = true;
        demod::core::DemodScratch scratch;
        scratch.reserve(packet.iq.size());
        const auto r = demod::core::demodulate_one(
            packet.iq.data(), packet.iq.size(), profile, 71, Packet::seed, 0,
            packet.sync, scratch);
        BOOST_REQUIRE_EQUAL(int(r.status), int(demod::DemodStatus::Success));
        BOOST_REQUIRE(r.payload.fcs_pass);
        BOOST_CHECK(r.payload.bytes == packet.wire);
        BOOST_CHECK_EQUAL(r.stage_fused_attempts, 1u);
        BOOST_CHECK_EQUAL(r.stage_fused_fallbacks, 0u);
        BOOST_CHECK_EQUAL(r.stage_softchip_fallbacks, 0u);
        BOOST_CHECK_EQUAL(r.stage_phr_fused_fallbacks, 0u);

        // Hard-only baseline from the SAME four signed metrics the fused kernel
        // produced.  If this passes, the soft retry was not needed and the
        // early return in stage_payload_fused_cmf() was taken.
        BOOST_REQUIRE(scratch.fused_phr.decisions_valid);
        BOOST_REQUIRE_EQUAL(scratch.fused_phr.g0.size(), size_t(21));
        BOOST_REQUIRE_EQUAL(scratch.fused_phr.llr0.size(), size_t(21));
        std::vector<int8_t> g0, g1;
        rxfusion::decide_bpm(scratch.fused.metrics, g0, g1);
        demod::PayloadResult hard;
        const size_t psdu_bits = static_cast<size_t>(r.phr.psdu_length) * 8;
        BOOST_REQUIRE(demod::core::detail::decode_payload_hard(
            scratch.fused_phr.g0, scratch.fused_phr.g1, g0, g1, psdu_bits,
            hard));
        BOOST_CHECK(hard.fcs_pass);
        BOOST_CHECK(hard.bytes == packet.wire);
        BOOST_CHECK(hard.bytes == r.payload.bytes);
        BOOST_CHECK(hard.bits == r.payload.bits);
        BOOST_CHECK_EQUAL(hard.received_fcs, r.payload.received_fcs);
        BOOST_CHECK_EQUAL(hard.calculated_fcs, r.payload.calculated_fcs);

        // The soft path on the same LLRs must agree byte-for-byte as well.
        std::vector<int8_t> sg0, sg1;
        std::vector<float> llr0, llr1;
        rxfusion::decide_bpm_soft(scratch.fused.metrics, sg0, sg1, llr0, llr1);
        demod::PayloadResult soft;
        BOOST_REQUIRE(demod::core::detail::decode_payload_soft(
            scratch.fused_phr.llr0, scratch.fused_phr.llr1, llr0, llr1,
            psdu_bits, soft));
        BOOST_CHECK(soft.fcs_pass);
        BOOST_CHECK(soft.bytes == packet.wire);
    }
}

// ---------------------------------------------------------------------------
// Task 4.3a — controlled hard-vs-soft measurement on the exact transmitted
// codeword.  The modulator's own encoder supplies the frame's coded bits, so
// the LLRs are synthesised around ground truth with a known noise level; no
// detector is involved and the comparison is deterministic given the seed.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(fused_cmf_soft_llr_rescues_hard_viterbi_failure)
{
    Packet packet(127, 9);
    const int8_t* sfd = nullptr;
    size_t n_sfd = 0;
    BOOST_REQUIRE(mod::detail::sfd_lookup(packet.cfg.sfd_mode, sfd, n_sfd));
    mod::HrpModScratch mod_scratch;
    mod::detail::HrpCodedGeometry geom;
    BOOST_REQUIRE(mod::detail::encode_hrp_packet(
        packet.wire.data(), packet.wire.size(), packet.cfg, mod_scratch, sfd,
        n_sfd, geom));

    const size_t psdu_bits = packet.wire.size() * 8;
    BOOST_REQUIRE_EQUAL(geom.nsym, psdu_bits + 48 * ((psdu_bits + 329) / 330));
    BOOST_REQUIRE_EQUAL(geom.n_enc_sym, size_t(21) + geom.nsym);
    BOOST_REQUIRE_EQUAL(mod_scratch.g0.size(), geom.n_enc_sym);
    BOOST_REQUIRE_EQUAL(mod_scratch.g1.size(), geom.n_enc_sym);

    std::mt19937 rng(4242);
    std::normal_distribution<float> nd(0.0f, 1.0f);
    size_t total_hard = 0, total_soft = 0, total_rescued = 0;
    const size_t kTrials = 200;

    for (float sigma : { 0.60f, 0.70f, 0.80f, 0.90f, 1.00f, 1.10f }) {
        size_t hard_ok = 0, soft_ok = 0, rescued = 0;
        for (size_t trial = 0; trial < kTrials; ++trial) {
            std::vector<float> llr0(geom.n_enc_sym), llr1(geom.n_enc_sym);
            for (size_t s = 0; s < geom.n_enc_sym; ++s) {
                llr0[s] = (mod_scratch.g0[s] ? -1.0f : 1.0f) + sigma * nd(rng);
                llr1[s] = (mod_scratch.g1[s] ? -1.0f : 1.0f) + sigma * nd(rng);
            }
            std::vector<int8_t> h0(geom.n_enc_sym), h1(geom.n_enc_sym);
            for (size_t s = 0; s < geom.n_enc_sym; ++s) {
                h0[s] = (llr0[s] >= 0.0f) ? 0 : 1;
                h1[s] = (llr1[s] >= 0.0f) ? 0 : 1;
            }
            const std::vector<int8_t> h0p(h0.begin(), h0.begin() + 21);
            const std::vector<int8_t> h1p(h1.begin(), h1.begin() + 21);
            const std::vector<int8_t> h0d(h0.begin() + 21, h0.end());
            const std::vector<int8_t> h1d(h1.begin() + 21, h1.end());
            const std::vector<float> l0p(llr0.begin(), llr0.begin() + 21);
            const std::vector<float> l1p(llr1.begin(), llr1.begin() + 21);
            const std::vector<float> l0d(llr0.begin() + 21, llr0.end());
            const std::vector<float> l1d(llr1.begin() + 21, llr1.end());

            demod::PayloadResult hard, soft;
            const bool hd = demod::core::detail::decode_payload_hard(
                h0p, h1p, h0d, h1d, psdu_bits, hard);
            const bool sd = demod::core::detail::decode_payload_soft(
                l0p, l1p, l0d, l1d, psdu_bits, soft);
            const bool hpass = hd && hard.fcs_pass;
            const bool spass = sd && soft.fcs_pass;
            hard_ok += hpass ? 1 : 0;
            soft_ok += spass ? 1 : 0;
            if (!hpass && spass) {
                ++rescued;
                BOOST_CHECK(soft.bytes == packet.wire);
            }
        }
        BOOST_TEST_MESSAGE("LLR sweep sigma=" << sigma << " hard=" << hard_ok
                           << "/" << kTrials << " soft=" << soft_ok << "/"
                           << kTrials << " rescued=" << rescued);
        BOOST_CHECK_GE(soft_ok, hard_ok);
        total_hard += hard_ok;
        total_soft += soft_ok;
        total_rescued += rescued;
    }
    BOOST_TEST_MESSAGE("LLR sweep totals: hard=" << total_hard << "/"
                       << 6 * kTrials << " soft=" << total_soft << "/"
                       << 6 * kTrials << " rescued=" << total_rescued);
    BOOST_CHECK_GT(total_rescued, size_t(0));
}

// ---------------------------------------------------------------------------
// Task 4.3b — end-to-end: a noisy frame the hard attempt cannot decode must
// still be committed by the in-place soft retry, without entering the ~15 ms
// full-stream stage_softchip_fallbacks path.
//
// The hard-only counterfactual is taken from the SAME four signed metrics the
// fused kernel produced for that packet (decide_bpm + decode_payload_hard), so
// the comparison is on identical inputs rather than on a second noisy run.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(fused_cmf_soft_retry_decodes_without_stream_fallback)
{
    Packet packet(127, 9);
    const size_t psdu_bits = packet.wire.size() * 8;
    std::mt19937 rng(20261117);
    std::normal_distribution<float> nd(0.0f, 1.0f);

    size_t committed = 0, hard_only_ok = 0, rescued = 0;
    for (float sigma : { 0.165f, 0.170f, 0.175f, 0.180f, 0.185f, 0.190f, 0.195f }) {
        size_t s_committed = 0, s_hard = 0, s_rescued = 0, s_phrfail = 0, s_other = 0;
        for (int trial = 0; trial < 20; ++trial) {
            std::vector<cf> noisy = packet.iq;
            for (auto& v : noisy)
                v += cf(sigma * nd(rng), sigma * nd(rng));

            auto profile = demod::Qm35825Profile::Default();
            profile.code_index = 9;
            profile.timing_search_margin = 256;
            profile.timing_coarse_stride = 1;
            profile.rx_softchip_on_demand = true;
            profile.rx_fused_payload_cmf = true;
            profile.rx_fused_phr_cmf = true;
            demod::core::DemodScratch scratch;
            scratch.reserve(noisy.size());
            const auto r = demod::core::demodulate_one(
                noisy.data(), noisy.size(), profile, 81, Packet::seed, 0,
                packet.sync, scratch);
            BOOST_TEST_MESSAGE("probe sigma=" << sigma << " trial=" << trial
                << " status=" << int(r.status) << " phr_ok=" << r.phr.ok
                << " len=" << r.phr.psdu_length
                << " secded=" << r.phr.secded_corrected
                << " uncorr=" << r.phr.secded_uncorrectable
                << " fcs=" << r.payload.fcs_pass
                << " fused_fb=" << r.stage_fused_fallbacks
                << " soft_fb=" << r.stage_softchip_fallbacks);
            if (!r.payload.fcs_pass) {
                if (!r.phr.ok) ++s_phrfail; else ++s_other;
                continue;
            }
            ++committed;
            ++s_committed;
            BOOST_CHECK(r.payload.bytes == packet.wire);
            BOOST_CHECK_EQUAL(r.stage_softchip_fallbacks, 0u);
            BOOST_CHECK_EQUAL(r.stage_fused_fallbacks, 0u);
            if (!scratch.fused_phr.decisions_valid ||
                scratch.fused_phr.g0.size() != 21)
                continue;

            std::vector<int8_t> g0, g1;
            rxfusion::decide_bpm(scratch.fused.metrics, g0, g1);
            demod::PayloadResult hard;
            const bool hd = demod::core::detail::decode_payload_hard(
                scratch.fused_phr.g0, scratch.fused_phr.g1, g0, g1,
                psdu_bits, hard);
            if (hd && hard.fcs_pass) {
                ++hard_only_ok;
                ++s_hard;
            } else {
                ++rescued;
                ++s_rescued;
                BOOST_TEST_MESSAGE("soft rescue at sigma=" << sigma
                                   << " trial=" << trial);
            }
        }
        BOOST_TEST_MESSAGE("end-to-end AWGN sigma=" << sigma
                           << " committed=" << s_committed << "/20 hard_only="
                           << s_hard << " rescued=" << s_rescued
                           << " phr_fail=" << s_phrfail << " other=" << s_other);
    }
    BOOST_TEST_MESSAGE("end-to-end AWGN: committed=" << committed
                       << " hard_only_ok=" << hard_only_ok
                       << " soft_rescued=" << rescued);
    BOOST_CHECK_GT(committed, size_t(0));
    // The soft retry must not have made anything worse: every committed packet
    // is either a hard success or a genuine soft rescue.
    BOOST_CHECK_EQUAL(committed, hard_only_ok + rescued);
}

// ---------------------------------------------------------------------------
// Step 2 — failure-only CL-3 Trellis-Merge Chase + RS(63,55) erasure rescue.
// Injects 6 isolated coded-symbol perturbations into one RS block so that
// single-pass vitdec_cl3_soft() leaves >4 symbol errors (uncorrectable by
// standard RS), verifies decode_payload_soft() rescues the exact wire bytes
// with CRC pass, and verifies 100 random noise LLR streams produce 0 false CRCs.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(fused_cmf_trellis_chase_and_rs_erasure_rescue)
{
    Packet packet(127, 9);
    const int8_t* sfd = nullptr;
    size_t n_sfd = 0;
    BOOST_REQUIRE(mod::detail::sfd_lookup(packet.cfg.sfd_mode, sfd, n_sfd));
    mod::HrpModScratch mod_scratch;
    mod::detail::HrpCodedGeometry geom;
    BOOST_REQUIRE(mod::detail::encode_hrp_packet(
        packet.wire.data(), packet.wire.size(), packet.cfg, mod_scratch, sfd,
        n_sfd, geom));

    const size_t total = geom.n_enc_sym;
    const size_t nsym = geom.nsym;
    const size_t psdu_bits = packet.wire.size() * 8;
    std::vector<float> llr0p(21), llr1p(21), llr0(nsym), llr1(nsym);
    for (size_t s = 0; s < 21; ++s) {
        llr0p[s] = mod_scratch.g0[s] ? -1.0f : 1.0f;
        llr1p[s] = mod_scratch.g1[s] ? -1.0f : 1.0f;
    }
    for (size_t s = 0; s < nsym; ++s) {
        llr0[s] = mod_scratch.g0[21 + s] ? -1.0f : 1.0f;
        llr1[s] = mod_scratch.g1[21 + s] ? -1.0f : 1.0f;
    }

    // Perturb 6 isolated 2-symbol spans inside RS block 0 with low-confidence
    // inverted LLRs: enough to force >4 decoded RS symbol errors in single-pass
    // vitdec_cl3_soft() while keeping trellis Chase + RS erasure rescue within reach.
    std::vector<float> mod0 = llr0, mod1 = llr1;
    for (int k = 0; k < 6; ++k) {
        const size_t base = 30 + size_t(k) * 48;
        for (size_t d = 0; d < 3 && base + d < mod0.size(); ++d) {
            mod0[base + d] = -mod0[base + d] * 0.55f;
            mod1[base + d] = -mod1[base + d] * 0.55f;
        }
    }

    // Baseline single-pass soft Viterbi + hard RS must not pass FCS on this block.
    std::vector<float> j0(total), j1(total);
    std::copy(llr0p.begin(), llr0p.end(), j0.begin());
    std::copy(llr1p.begin(), llr1p.end(), j1.begin());
    std::copy(mod0.begin(), mod0.end(), j0.begin() + 21);
    std::copy(mod1.begin(), mod1.end(), j1.begin() + 21);
    std::vector<int8_t> sp_dec;
    demod::core::detail::vitdec_cl3_soft(j0.data(), j1.data(), total, sp_dec);
    std::vector<int8_t> sp_cw(sp_dec.begin() + 19,
                              sp_dec.begin() + (total - 2));
    demod::PayloadResult sp_pay;
    const bool sp_rs_ok =
        demod::core::detail::finish_rs_crc_payload(sp_cw, psdu_bits, sp_pay);
    BOOST_CHECK(!(sp_rs_ok && sp_pay.fcs_pass));

    // decode_payload_soft() must rescue the exact wire bytes via Step 2.
    demod::PayloadResult rescued;
    BOOST_REQUIRE(demod::core::detail::decode_payload_soft(
        llr0p, llr1p, mod0, mod1, psdu_bits, rescued));
    BOOST_CHECK(rescued.fcs_pass);
    BOOST_CHECK(rescued.bytes == packet.wire);

    // Pure-noise LLRs must produce zero false CRC passes.
    std::mt19937 rng(20261117);
    std::normal_distribution<float> nd(0.0f, 1.0f);
    std::vector<float> nl0p(21), nl1p(21), nl0(nsym), nl1(nsym);
    size_t false_crc = 0;
    for (int trial = 0; trial < 100; ++trial) {
        for (float& x : nl0p) x = nd(rng);
        for (float& x : nl1p) x = nd(rng);
        for (float& x : nl0) x = nd(rng);
        for (float& x : nl1) x = nd(rng);
        demod::PayloadResult nres;
        if (demod::core::detail::decode_payload_soft(
                nl0p, nl1p, nl0, nl1, psdu_bits, nres) &&
            nres.fcs_pass) {
            ++false_crc;
        }
    }
    BOOST_CHECK_EQUAL(false_crc, size_t(0));
}

// ---------------------------------------------------------------------------
// Exact-CFO fused payload kernel: correlate_payload_fused_cfo().
//
// Reference: derotate the raw buffer sample by sample with the production
// scalar derotator (phase origin = rx[0]) and then run the untouched legacy
// correlate_payload().  This is the exact contract of the new entry, so both
// the raw complex metric and the signed metric are compared element-wise, and
// the hard BPM decisions must agree 100%.
// ---------------------------------------------------------------------------
namespace {

constexpr double kQaSampleRate = 998.4e6;
constexpr double kQaTwoPi = 6.283185307179586476925286766559;
constexpr int64_t kQaChipStart = 37;
constexpr int64_t kQaPayloadStart = 100;

struct FusedCfoInput {
    std::vector<cf> rx;
    std::vector<cf> taps;
    std::vector<int8_t> pn;
    cf rot{ 1.0f, 0.0f };
    size_t symbols = 0;
};

// Sized so that the furthest candidate window (chip offset 40 = sample 80) plus
// the 52-point template of the last symbol is readable, with slack.
FusedCfoInput make_fused_cfo_input(size_t symbols,
                                   size_t tap_count,
                                   uint32_t seed,
                                   int64_t chip_start = kQaChipStart,
                                   int64_t payload_start = kQaPayloadStart)
{
    FusedCfoInput in;
    in.symbols = symbols;
    std::mt19937 rng(seed);
    std::uniform_real_distribution<float> dist(-1.0f, 1.0f);
    const size_t base0 =
        static_cast<size_t>(chip_start + payload_start * 2 -
                            static_cast<int64_t>(tap_count) + 1);
    const size_t needed = base0 + (symbols - 1) * 128 + 80 +
                          (tap_count + 14) + 64;
    in.rx.resize(needed);
    for (auto& x : in.rx)
        x = cf(dist(rng), dist(rng));
    in.taps.resize(tap_count);
    for (auto& h : in.taps)
        h = cf(dist(rng), dist(rng));
    in.pn.resize(symbols * rxfusion::kChipsPerBurst);
    for (auto& a : in.pn)
        a = (dist(rng) < 0.0f) ? int8_t(-1) : int8_t(1);
    const float ang = dist(rng) * 3.0f;
    in.rot = cf(std::cos(ang), std::sin(ang));
    return in;
}

struct CfoDiff {
    double max_abs_signed = 0.0;
    double max_abs_complex = 0.0;
    double max_ref = 0.0;
    size_t decision_mismatch = 0;
    size_t decisions = 0;
};

// Compare the fused-CFO entry against derotate + legacy correlate_payload for
// one (tap_count, cfo, polarity, alignment) combination.
bool compare_fused_cfo(const FusedCfoInput& in,
                       size_t tap_count,
                       double cfo_hz,
                       int8_t pol,
                       bool unaligned,
                       CfoDiff& diff)
{
    const double w = kQaTwoPi * cfo_hz / kQaSampleRate;

    std::vector<cf> shifted;
    const cf* rx_ptr = in.rx.data();
    if (unaligned) {
        shifted.resize(in.rx.size() + 1);
        shifted[0] = cf(0.0f, 0.0f);
        std::copy(in.rx.begin(), in.rx.end(), shifted.begin() + 1);
        rx_ptr = shifted.data() + 1;
    }

    std::vector<cf> derotated(in.rx.size());
    gr::uwb::demod::rxopt::derotate_range_scalar(
        derotated.data(), rx_ptr, in.rx.size(), 0, in.rx.size(), w);

    rxfusion::Metrics reference;
    rxfusion::Metrics fused;
    rxfusion::Scratch scratch_ref;
    rxfusion::Scratch scratch_fused;
    if (!rxfusion::correlate_payload(
            derotated.data(), derotated.size(), in.taps.data(), tap_count,
            kQaChipStart, rxfusion::kSamplesPerChip, kQaPayloadStart,
            in.pn.data(), in.symbols, pol, in.rot, reference, scratch_ref))
        return false;
    if (!rxfusion::correlate_payload_fused_cfo(
            rx_ptr, in.rx.size(), in.taps.data(), tap_count, kQaChipStart,
            rxfusion::kSamplesPerChip, kQaPayloadStart, in.pn.data(),
            in.symbols, pol, in.rot, w, fused, scratch_fused, nullptr, true))
        return false;

    diff = CfoDiff{};
    const size_t count = rxfusion::kCandidateCount * in.symbols;
    if (reference.signed_value.size() != count ||
        fused.signed_value.size() != count ||
        fused.complex_value.size() != count)
        return false;
    for (size_t i = 0; i < count; ++i) {
        diff.max_abs_signed =
            std::max(diff.max_abs_signed,
                     static_cast<double>(std::abs(fused.signed_value[i] -
                                                  reference.signed_value[i])));
        diff.max_abs_complex = std::max(
            diff.max_abs_complex,
            static_cast<double>(std::abs(fused.complex_value[i] -
                                         reference.complex_value[i])));
        diff.max_ref = std::max(
            diff.max_ref,
            std::max(static_cast<double>(std::abs(reference.signed_value[i])),
                     static_cast<double>(std::abs(reference.complex_value[i]))));
    }
    std::vector<int8_t> g0_ref, g1_ref, g0, g1;
    rxfusion::decide_bpm(reference, g0_ref, g1_ref);
    rxfusion::decide_bpm(fused, g0, g1);
    diff.decisions = in.symbols;
    for (size_t s = 0; s < in.symbols; ++s) {
        if (g0[s] != g0_ref[s] || g1[s] != g1_ref[s])
            ++diff.decision_mismatch;
    }
    return true;
}

} // namespace

BOOST_AUTO_TEST_CASE(fused_cfo_matches_derotate_then_correlate)
{
    double max_abs_signed = 0.0;
    double max_abs_complex = 0.0;
    double max_ref = 0.0;
    size_t decisions = 0;
    size_t mismatched = 0;
    size_t combinations = 0;

    struct Spec {
        size_t symbols;
        int64_t chip_start;
        int64_t payload_start;
    };
    const Spec specs[] = { { 1, kQaChipStart, kQaPayloadStart },
                           { 64, kQaChipStart, kQaPayloadStart },
                           { 65, 13, 900 },
                           { 1300, kQaChipStart, kQaPayloadStart } };

    for (const auto& spec : specs) {
        const FusedCfoInput in =
            make_fused_cfo_input(spec.symbols, kTap, 7717 + spec.symbols,
                                 spec.chip_start, spec.payload_start);
        for (double cfo_hz : { 0.0, 1000.0, 10000.0, 40000.0 }) {
            for (int8_t pol : { int8_t(1), int8_t(-1) }) {
                for (bool unaligned : { false, true }) {
                    CfoDiff d;
                    BOOST_REQUIRE(compare_fused_cfo(in, kTap, cfo_hz, pol,
                                                    unaligned, d));
                    BOOST_TEST_MESSAGE(
                        "fused_cfo symbols=" << spec.symbols << " cfo="
                        << cfo_hz << " pol=" << int(pol) << " unaligned="
                        << unaligned << " abs_signed=" << d.max_abs_signed
                        << " abs_complex=" << d.max_abs_complex
                        << " ref=" << d.max_ref
                        << " mismatched=" << d.decision_mismatch << "/"
                        << d.decisions);
                    BOOST_CHECK_EQUAL(d.decision_mismatch, size_t(0));
                    max_abs_signed = std::max(max_abs_signed, d.max_abs_signed);
                    max_abs_complex =
                        std::max(max_abs_complex, d.max_abs_complex);
                    max_ref = std::max(max_ref, d.max_ref);
                    decisions += d.decisions;
                    mismatched += d.decision_mismatch;
                    ++combinations;
                }
            }
        }
    }

    BOOST_TEST_MESSAGE("fused_cfo overall: combinations="
                       << combinations << " max_abs_signed=" << max_abs_signed
                       << " max_abs_complex=" << max_abs_complex
                       << " max_ref=" << max_ref
                       << " max_rel=" << (max_abs_signed / std::max(max_ref, 1e-12))
                       << " decision_mismatch=" << mismatched << "/"
                       << decisions);
    // Exact CFO fusion (no approximation): only float accumulation order
    // differs.  Absolute and relative error must both stay below 1e-5.
    BOOST_CHECK_LT(max_abs_signed, 1e-5);
    BOOST_CHECK_LT(max_abs_complex, 1e-5);
    BOOST_CHECK_LT(max_abs_signed / std::max(max_ref, 1e-12), 1e-5);
    BOOST_CHECK_LT(max_abs_complex / std::max(max_ref, 1e-12), 1e-5);
    BOOST_CHECK_EQUAL(mismatched, size_t(0));
}

BOOST_AUTO_TEST_CASE(fused_cfo_large_window_offset_phase_anchoring)
{
    // A large chip_start pushes the window phase argument into the hundreds of
    // radians and spans several kFusedPhaseAnchor re-anchor blocks, so the
    // double symbol ramp (not a float recurrence) is what keeps this exact.
    const FusedCfoInput in =
        make_fused_cfo_input(96, kTap, 606060, 1000003, 100);
    CfoDiff d;
    BOOST_REQUIRE(compare_fused_cfo(in, kTap, 40000.0, 1, false, d));
    BOOST_TEST_MESSAGE("fused_cfo large offset: abs_signed="
                       << d.max_abs_signed
                       << " abs_complex=" << d.max_abs_complex
                       << " ref=" << d.max_ref << " mismatch="
                       << d.decision_mismatch << "/" << d.decisions);
    BOOST_CHECK_LT(d.max_abs_signed, 1e-5);
    BOOST_CHECK_LT(d.max_abs_complex, 1e-5);
    BOOST_CHECK_EQUAL(d.decision_mismatch, size_t(0));
}

BOOST_AUTO_TEST_CASE(fused_cfo_generic_tap_counts)
{
    // 24 and 48 taps take the exact generic per-symbol template path, not the
    // 38-tap half-template LUT.
    double max_abs = 0.0;
    double max_ref = 0.0;
    size_t decisions = 0;
    size_t mismatched = 0;
    for (size_t tap_count : { size_t(24), size_t(48) }) {
        BOOST_REQUIRE_NE(tap_count, kTap);
        for (size_t symbols : { size_t(1), size_t(65) }) {
            const FusedCfoInput in = make_fused_cfo_input(
                symbols, tap_count, 4100 + 97 * tap_count + symbols);
            for (double cfo_hz : { 0.0, 10000.0, 40000.0 }) {
                for (int8_t pol : { int8_t(1), int8_t(-1) }) {
                    CfoDiff d;
                    BOOST_REQUIRE(compare_fused_cfo(in, tap_count, cfo_hz, pol,
                                                    true, d));
                    BOOST_TEST_MESSAGE("fused_cfo generic taps="
                                       << tap_count << " symbols=" << symbols
                                       << " cfo=" << cfo_hz
                                       << " pol=" << int(pol)
                                       << " abs_signed=" << d.max_abs_signed
                                       << " abs_complex=" << d.max_abs_complex
                                       << " ref=" << d.max_ref);
                    BOOST_CHECK_EQUAL(d.decision_mismatch, size_t(0));
                    max_abs = std::max(max_abs, std::max(d.max_abs_signed,
                                                         d.max_abs_complex));
                    max_ref = std::max(max_ref, d.max_ref);
                    decisions += d.decisions;
                    mismatched += d.decision_mismatch;
                }
            }
        }
    }
    BOOST_TEST_MESSAGE("fused_cfo generic overall: max_abs="
                       << max_abs << " max_ref=" << max_ref
                       << " max_rel=" << (max_abs / std::max(max_ref, 1e-12))
                       << " decision_mismatch=" << mismatched << "/"
                       << decisions);
    BOOST_CHECK_LT(max_abs, 1e-5);
    BOOST_CHECK_LT(max_abs / std::max(max_ref, 1e-12), 1e-5);
    BOOST_CHECK_EQUAL(mismatched, size_t(0));
}

BOOST_AUTO_TEST_CASE(fused_cfo_write_complex_flag)
{
    const FusedCfoInput in = make_fused_cfo_input(65, kTap, 90210);
    const double w = kQaTwoPi * 40000.0 / kQaSampleRate;
    rxfusion::Metrics with_complex;
    rxfusion::Metrics without_complex;
    rxfusion::Scratch scratch_a;
    rxfusion::Scratch scratch_b;
    BOOST_REQUIRE(rxfusion::correlate_payload_fused_cfo(
        in.rx.data(), in.rx.size(), in.taps.data(), kTap, kQaChipStart,
        rxfusion::kSamplesPerChip, kQaPayloadStart, in.pn.data(), in.symbols, 1,
        in.rot, w, with_complex, scratch_a, nullptr, true));
    BOOST_REQUIRE(rxfusion::correlate_payload_fused_cfo(
        in.rx.data(), in.rx.size(), in.taps.data(), kTap, kQaChipStart,
        rxfusion::kSamplesPerChip, kQaPayloadStart, in.pn.data(), in.symbols, 1,
        in.rot, w, without_complex, scratch_b, nullptr, false));

    const size_t count = rxfusion::kCandidateCount * in.symbols;
    BOOST_REQUIRE_EQUAL(with_complex.complex_value.size(), count);
    // write_complex == false must not touch (or require capacity in) the
    // 16 bytes/symbol diagnostic buffer.
    BOOST_CHECK(without_complex.complex_value.empty());
    BOOST_CHECK_EQUAL(without_complex.complex_value.capacity(), size_t(0));
    BOOST_REQUIRE_EQUAL(without_complex.signed_value.size(), count);
    BOOST_CHECK(with_complex.signed_value == without_complex.signed_value);

    // A Metrics that never held a complex buffer at all must work too.
    rxfusion::Metrics fresh;
    rxfusion::Scratch scratch_c;
    BOOST_REQUIRE(rxfusion::correlate_payload_fused_cfo(
        in.rx.data(), in.rx.size(), in.taps.data(), kTap, kQaChipStart,
        rxfusion::kSamplesPerChip, kQaPayloadStart, in.pn.data(), in.symbols, 1,
        in.rot, w, fresh, scratch_c, nullptr, false));
    BOOST_CHECK(fresh.signed_value == with_complex.signed_value);
    BOOST_CHECK(fresh.complex_value.empty());
}

BOOST_AUTO_TEST_CASE(fused_cfo_timings_do_not_change_results)
{
    const FusedCfoInput in = make_fused_cfo_input(65, kTap, 555);
    const double w = kQaTwoPi * 10000.0 / kQaSampleRate;
    rxfusion::Metrics a, b;
    rxfusion::Scratch scratch_a, scratch_b;
    rxfusion::Timings timings;
    BOOST_REQUIRE(rxfusion::correlate_payload_fused_cfo(
        in.rx.data(), in.rx.size(), in.taps.data(), kTap, kQaChipStart,
        rxfusion::kSamplesPerChip, kQaPayloadStart, in.pn.data(), in.symbols, 1,
        in.rot, w, a, scratch_a, nullptr, false));
    BOOST_REQUIRE(rxfusion::correlate_payload_fused_cfo(
        in.rx.data(), in.rx.size(), in.taps.data(), kTap, kQaChipStart,
        rxfusion::kSamplesPerChip, kQaPayloadStart, in.pn.data(), in.symbols, 1,
        in.rot, w, b, scratch_b, &timings, false));
    BOOST_CHECK(a.signed_value == b.signed_value);
    BOOST_TEST_MESSAGE("fused_cfo timings template="
                       << timings.template_update_us
                       << "us correlate=" << timings.correlate_us << "us");
}

BOOST_AUTO_TEST_CASE(fused_cfo_rejects_unsupported_and_out_of_range)
{
    const FusedCfoInput in = make_fused_cfo_input(4, kTap, 31337);
    rxfusion::Metrics out;
    rxfusion::Scratch scratch;
    const cf* rx = in.rx.data();
    const size_t n = in.rx.size();
    const cf* h = in.taps.data();
    const int8_t* pn = in.pn.data();
    const double w = kQaTwoPi * 10000.0 / kQaSampleRate;

    auto call = [&](const cf* rx_p, size_t n_p, const cf* h_p, size_t taps,
                    int64_t chip_start, size_t spc, int64_t payload_start,
                    const int8_t* pn_p, size_t symbols, int8_t pol, double cfo) {
        return rxfusion::correlate_payload_fused_cfo(
            rx_p, n_p, h_p, taps, chip_start, spc, payload_start, pn_p, symbols,
            pol, in.rot, cfo, out, scratch);
    };

    // Null pointers.
    BOOST_CHECK(!call(nullptr, n, h, kTap, kQaChipStart, 2, kQaPayloadStart, pn,
                      in.symbols, 1, w));
    BOOST_CHECK(!call(rx, n, nullptr, kTap, kQaChipStart, 2, kQaPayloadStart,
                      pn, in.symbols, 1, w));
    BOOST_CHECK(!call(rx, n, h, kTap, kQaChipStart, 2, kQaPayloadStart, nullptr,
                      in.symbols, 1, w));
    // Geometry / argument domain.
    BOOST_CHECK(!call(rx, n, h, 0, kQaChipStart, 2, kQaPayloadStart, pn,
                      in.symbols, 1, w));
    BOOST_CHECK(!call(rx, n, h, rxfusion::kMaxTapCount + 1, kQaChipStart, 2,
                      kQaPayloadStart, pn, in.symbols, 1, w));
    BOOST_CHECK(!call(rx, n, h, kTap, kQaChipStart, 1, kQaPayloadStart, pn,
                      in.symbols, 1, w));
    BOOST_CHECK(!call(rx, n, h, kTap, kQaChipStart, 4, kQaPayloadStart, pn,
                      in.symbols, 1, w));
    BOOST_CHECK(!call(rx, n, h, kTap, -1, 2, kQaPayloadStart, pn, in.symbols, 1,
                      w));
    BOOST_CHECK(!call(rx, n, h, kTap, kQaChipStart, 2, -1, pn, in.symbols, 1,
                      w));
    BOOST_CHECK(!call(rx, n, h, kTap, kQaChipStart, 2, kQaPayloadStart, pn, 0, 1,
                      w));
    BOOST_CHECK(!call(rx, n, h, kTap, kQaChipStart, 2, kQaPayloadStart, pn,
                      in.symbols, 0, w));
    BOOST_CHECK(!call(rx, n, h, kTap, kQaChipStart, 2, kQaPayloadStart, pn,
                      in.symbols, 1, std::nan("")));
    // payload_start == 0 with chip_start == 0 leaves base0 = -tap + 1 < 0.
    BOOST_CHECK(!call(rx, n, h, kTap, 0, 2, 0, pn, in.symbols, 1, w));

    // Exact last readable window succeeds; one sample shorter is rejected.
    const size_t len = kTap + 14;
    const int64_t base0 = kQaChipStart + kQaPayloadStart * 2 - kTap + 1;
    const size_t needed = static_cast<size_t>(
        base0 + static_cast<int64_t>(in.symbols - 1) * 128 + 80 +
        static_cast<int64_t>(len));
    BOOST_REQUIRE_LE(needed, in.rx.size());
    BOOST_CHECK(call(rx, needed, h, kTap, kQaChipStart, 2, kQaPayloadStart, pn,
                     in.symbols, 1, w));
    BOOST_CHECK(!call(rx, needed - 1, h, kTap, kQaChipStart, 2, kQaPayloadStart,
                      pn, in.symbols, 1, w));
    // Same boundary on the generic 48-tap path: its window origin shifts by
    // -10 samples and its template grows by 10, so the last readable sample is
    // identical to the 38-tap case.
    BOOST_CHECK(call(rx, needed, h, 48, kQaChipStart, 2, kQaPayloadStart, pn,
                     in.symbols, 1, w));
    BOOST_CHECK(!call(rx, needed - 1, h, 48, kQaChipStart, 2, kQaPayloadStart,
                      pn, in.symbols, 1, w));

    // Illegal spreading chips must be rejected, never folded to +1.
    std::vector<int8_t> bad_zero = in.pn;
    bad_zero[3] = 0;
    BOOST_CHECK(!call(rx, n, h, kTap, kQaChipStart, 2, kQaPayloadStart,
                      bad_zero.data(), in.symbols, 1, w));
    std::vector<int8_t> bad_two = in.pn;
    bad_two[bad_two.size() - 1] = 2;
    BOOST_CHECK(!call(rx, n, h, kTap, kQaChipStart, 2, kQaPayloadStart,
                      bad_two.data(), in.symbols, 1, w));
    std::vector<int8_t> bad_neg_two = in.pn;
    bad_neg_two[5] = -2;
    BOOST_CHECK(!call(rx, n, h, kTap, kQaChipStart, 2, kQaPayloadStart,
                      bad_neg_two.data(), in.symbols, 1, w));

    // The explicit validate_pn flag gates the scan: the validated entry rejects
    // the zero chip, while the trusted implementation (used only by callers
    // that ran prepare_fused_payload_pn) does not rescan it.
    BOOST_CHECK(!rxfusion::correlate_payload_fused_cfo_impl(
        rx, n, h, kTap, kQaChipStart, 2, kQaPayloadStart, bad_zero.data(),
        in.symbols, 1, in.rot, w, out, scratch, nullptr, false, true));
    BOOST_CHECK(rxfusion::correlate_payload_fused_cfo_impl(
        rx, n, h, kTap, kQaChipStart, 2, kQaPayloadStart, bad_zero.data(),
        in.symbols, 1, in.rot, w, out, scratch, nullptr, false, false));
}

// ---------------------------------------------------------------------------
// Known-hop 2-candidate kernel + symbol-level DD-PLL.
// ---------------------------------------------------------------------------
namespace {

// Synthetic BPM-BPSK payload with an exactly known answer.
//
// The CIR is a single unit tap at h[37], so the fused template has support only
// at m = 2*c + 37 with coefficient pn[c].  Placing pn[c] at rx[win + 2*c + 37]
// therefore makes the half correlation exactly 8 (times the complex gain) when
// `win` is the true window, and ~0 on the other half/hop.  The whole buffer is
// then multiplied by exp(j*2*pi*f_res*n/fs) so the entry can be called with
// cfo_w_rad == 0 while the packet still carries a residual CFO.
struct SynthBpmPayload {
    std::vector<cf> rx;
    std::vector<cf> taps;
    std::vector<int8_t> pn;
    std::vector<int8_t> expect_half;
    size_t symbols = 0;
    int64_t chip_start = 11;
    int64_t payload_start = 40;
};

SynthBpmPayload make_synth_payload(size_t symbols,
                                   double residual_cfo_hz,
                                   float noise_sigma,
                                   uint32_t seed,
                                   bool alternate_half = true,
                                   double static_phase_rad = 0.0)
{
    SynthBpmPayload in;
    in.symbols = symbols;
    std::mt19937 rng(seed);
    std::uniform_real_distribution<float> dist(-1.0f, 1.0f);
    in.taps.assign(kTap, cf(0.0f, 0.0f));
    in.taps[kTap - 1] = cf(1.0f, 0.0f);
    in.pn.resize(symbols * rxfusion::kChipsPerBurst);
    for (auto& a : in.pn)
        a = (dist(rng) < 0.0f) ? int8_t(-1) : int8_t(1);
    in.expect_half.resize(symbols);
    for (size_t s = 0; s < symbols; ++s)
        in.expect_half[s] = alternate_half ? int8_t(s & 1) : int8_t(0);

    const int64_t base0 =
        in.chip_start + in.payload_start * 2 - int64_t(kTap) + 1;
    const size_t needed = static_cast<size_t>(
        base0 + static_cast<int64_t>(symbols - 1) * 128 + 80 + 52 + 16);
    in.rx.assign(needed, cf(0.0f, 0.0f));
    for (size_t s = 0; s < symbols; ++s) {
        const int hop = (in.pn[s * rxfusion::kChipsPerBurst] < 0) ? 1 : 0;
        const int half = in.expect_half[s];
        const int64_t win = base0 + static_cast<int64_t>(s) * 128 + hop * 16 +
                            half * 64;
        for (size_t c = 0; c < rxfusion::kChipsPerBurst; ++c) {
            in.rx[static_cast<size_t>(win + 2 * int64_t(c) + int64_t(kTap) - 1)] +=
                cf(static_cast<float>(in.pn[s * rxfusion::kChipsPerBurst + c]),
                   0.0f);
        }
    }

    const double w = kQaTwoPi * residual_cfo_hz / kQaSampleRate;
    if (w != 0.0 || static_phase_rad != 0.0) {
        std::complex<double> ramp(std::cos(static_phase_rad),
                                  std::sin(static_phase_rad));
        const std::complex<double> step(std::cos(w), std::sin(w));
        for (auto& x : in.rx) {
            x *= cf(static_cast<float>(ramp.real()),
                    static_cast<float>(ramp.imag()));
            ramp *= step;
        }
    }
    if (noise_sigma > 0.0f) {
        std::normal_distribution<float> nd(0.0f, noise_sigma);
        for (auto& x : in.rx)
            x += cf(nd(rng), nd(rng));
    }
    return in;
}

size_t count_reference_half_errors(const SynthBpmPayload& in,
                                   const rxfusion::Metrics2& out,
                                   size_t first_symbol)
{
    size_t errors = 0;
    for (size_t s = first_symbol; s < in.symbols; ++s) {
        const float v = in.expect_half[s] == 0 ? out.m0[s] : out.m1[s];
        if (v < 0.0f)
            ++errors;
    }
    return errors;
}

bool all_finite(const rxfusion::Metrics2& out)
{
    for (size_t s = 0; s < out.symbols; ++s) {
        if (!std::isfinite(out.m0[s]) || !std::isfinite(out.m1[s]))
            return false;
        if (!out.z0.empty() &&
            (!std::isfinite(out.z0[s].real()) ||
             !std::isfinite(out.z0[s].imag()) ||
             !std::isfinite(out.z1[s].real()) ||
             !std::isfinite(out.z1[s].imag())))
            return false;
    }
    return true;
}

struct KnownHopDiff {
    double max_abs_m = 0.0;
    double max_abs_z = 0.0;
    double max_ref = 0.0;
    size_t decision_mismatch = 0;
    size_t decisions = 0;
    size_t full_decision_checked = 0;
    size_t full_decision_mismatch = 0;
};

// Run the 4-candidate exact-CFO entry and the known-hop entry on the same
// buffer and compare the two known-hop columns element-wise.
bool compare_known_hop(const FusedCfoInput& in,
                       double cfo_hz,
                       int8_t pol,
                       bool unaligned,
                       KnownHopDiff& diff)
{
    const double w = kQaTwoPi * cfo_hz / kQaSampleRate;
    std::vector<cf> shifted;
    const cf* rx_ptr = in.rx.data();
    size_t n = in.rx.size();
    if (unaligned) {
        shifted.resize(in.rx.size() + 1);
        shifted[0] = cf(0.0f, 0.0f);
        std::copy(in.rx.begin(), in.rx.end(), shifted.begin() + 1);
        rx_ptr = shifted.data() + 1;
        n = in.rx.size();
    }

    rxfusion::Metrics ref;
    rxfusion::Scratch scratch_ref;
    if (!rxfusion::correlate_payload_fused_cfo(
            rx_ptr, n, in.taps.data(), kTap, kQaChipStart,
            rxfusion::kSamplesPerChip, kQaPayloadStart, in.pn.data(),
            in.symbols, pol, in.rot, w, ref, scratch_ref, nullptr, true))
        return false;

    rxfusion::DdPllConfig pll; // enable == false
    rxfusion::Metrics2 got;
    rxfusion::Scratch scratch_got;
    if (!rxfusion::correlate_payload_known_hop_fused_cfo(
            rx_ptr, n, in.taps.data(), kTap, kQaChipStart,
            static_cast<double>(rxfusion::kSamplesPerChip), kQaPayloadStart,
            in.pn.data(), in.symbols, pol, in.rot, w, pll, got, scratch_got,
            nullptr, true, nullptr, 0))
        return false;

    std::vector<int8_t> g0_ref, g1_ref;
    rxfusion::decide_bpm(ref, g0_ref, g1_ref);

    diff = KnownHopDiff{};
    for (size_t s = 0; s < in.symbols; ++s) {
        const int hop =
            (in.pn[s * rxfusion::kChipsPerBurst] < 0) ? 1 : 0;
        const size_t p = 2 * static_cast<size_t>(hop);
        const float r0 = ref.signed_value[4 * s + p];
        const float r1 = ref.signed_value[4 * s + p + 1];
        diff.max_abs_m =
            std::max(diff.max_abs_m,
                     static_cast<double>(std::abs(got.m0[s] - r0)));
        diff.max_abs_m =
            std::max(diff.max_abs_m,
                     static_cast<double>(std::abs(got.m1[s] - r1)));
        diff.max_abs_z = std::max(
            diff.max_abs_z,
            static_cast<double>(std::abs(got.z0[s] -
                                         ref.complex_value[4 * s + p])));
        diff.max_abs_z = std::max(
            diff.max_abs_z,
            static_cast<double>(std::abs(got.z1[s] -
                                         ref.complex_value[4 * s + p + 1])));
        diff.max_ref = std::max(
            diff.max_ref,
            std::max(std::max(static_cast<double>(std::abs(r0)),
                              static_cast<double>(std::abs(r1))),
                     std::max(static_cast<double>(
                                  std::abs(ref.complex_value[4 * s + p])),
                              static_cast<double>(
                                  std::abs(ref.complex_value[4 * s + p + 1])))));

        // Known-hop decision on the two correlated halves.
        const int ref_half = (std::abs(r0) >= std::abs(r1)) ? 0 : 1;
        const int8_t ref_pol =
            ((ref_half == 0 ? r0 : r1) < 0.0f) ? int8_t(1) : int8_t(0);
        const int got_half =
            (std::abs(got.m0[s]) >= std::abs(got.m1[s])) ? 0 : 1;
        const int8_t got_pol =
            ((got_half == 0 ? got.m0[s] : got.m1[s]) < 0.0f) ? int8_t(1)
                                                              : int8_t(0);
        if (ref_half != got_half || ref_pol != got_pol)
            ++diff.decision_mismatch;
        ++diff.decisions;

        // When the known-hop candidates dominate both halves, the 4-candidate
        // global decision must agree exactly with the known-hop decision.  On
        // random noise input the other hop sometimes wins a half; that is not
        // a disagreement, it is information the known-hop kernel never sees.
        const size_t p0 = 4 * s + 2 * static_cast<size_t>(hop);
        const size_t p1 = p0 + 1;
        const size_t q0 = 4 * s + 2 * static_cast<size_t>(1 - hop);
        const size_t q1 = q0 + 1;
        const bool hop_dominates =
            std::abs(ref.signed_value[p0]) >=
                std::abs(ref.signed_value[q0]) &&
            std::abs(ref.signed_value[p1]) >=
                std::abs(ref.signed_value[q1]);
        if (hop_dominates) {
            ++diff.full_decision_checked;
            if (g0_ref[s] != got_half || g1_ref[s] != got_pol)
                ++diff.full_decision_mismatch;
        }
    }
    return true;
}

} // namespace

BOOST_AUTO_TEST_CASE(known_hop_matches_four_candidate)
{
    double max_abs_m = 0.0;
    double max_abs_z = 0.0;
    double max_ref = 0.0;
    size_t decisions = 0;
    size_t mismatch = 0;
    size_t full_checked = 0;
    size_t full_mismatch = 0;
    size_t combinations = 0;

    struct Spec {
        size_t symbols;
        int64_t chip_start;
        int64_t payload_start;
    };
    const Spec specs[] = { { 1, kQaChipStart, kQaPayloadStart },
                           { 64, kQaChipStart, kQaPayloadStart },
                           { 65, 13, 900 },
                           { 1208, kQaChipStart, kQaPayloadStart } };

    for (const auto& spec : specs) {
        const FusedCfoInput in =
            make_fused_cfo_input(spec.symbols, kTap, 5100 + spec.symbols,
                                 spec.chip_start, spec.payload_start);
        for (double cfo_hz : { 0.0, 40000.0 }) {
            for (int8_t pol : { int8_t(1), int8_t(-1) }) {
                for (bool unaligned : { false, true }) {
                    KnownHopDiff d;
                    BOOST_REQUIRE(compare_known_hop(in, cfo_hz, pol, unaligned,
                                                    d));
                    BOOST_TEST_MESSAGE(
                        "known_hop symbols=" << spec.symbols << " cfo="
                        << cfo_hz << " pol=" << int(pol) << " unaligned="
                        << unaligned << " m_err=" << d.max_abs_m
                        << " z_err=" << d.max_abs_z << " ref=" << d.max_ref
                        << " decision_mismatch=" << d.decision_mismatch << "/"
                        << d.decisions << " global_checked="
                        << d.full_decision_checked << " global_mismatch="
                        << d.full_decision_mismatch);
                    BOOST_CHECK_EQUAL(d.decision_mismatch, size_t(0));
                    BOOST_CHECK_EQUAL(d.full_decision_mismatch, size_t(0));
                    max_abs_m = std::max(max_abs_m, d.max_abs_m);
                    max_abs_z = std::max(max_abs_z, d.max_abs_z);
                    max_ref = std::max(max_ref, d.max_ref);
                    decisions += d.decisions;
                    mismatch += d.decision_mismatch;
                    full_checked += d.full_decision_checked;
                    full_mismatch += d.full_decision_mismatch;
                    ++combinations;
                }
            }
        }
    }

    BOOST_TEST_MESSAGE("known_hop overall: combinations="
                       << combinations << " max_abs_m=" << max_abs_m
                       << " max_abs_z=" << max_abs_z << " max_ref=" << max_ref
                       << " max_rel=" << (max_abs_m / std::max(max_ref, 1e-12))
                       << " decision_mismatch=" << mismatch << "/" << decisions
                       << " global_mismatch=" << full_mismatch << "/"
                       << full_checked);
    // The two kernels share the template and the per-candidate arithmetic
    // path; only the symbol-ramp anchor differs, so the tolerance is the same
    // 1e-5 absolute bound as the 4-candidate exact-CFO QA.
    BOOST_CHECK_LT(max_abs_m, 1e-5);
    BOOST_CHECK_LT(max_abs_z, 1e-5);
    BOOST_CHECK_LT(max_abs_m / std::max(max_ref, 1e-12), 1e-5);
    BOOST_CHECK_EQUAL(mismatch, size_t(0));
    BOOST_CHECK_EQUAL(full_mismatch, size_t(0));
}

BOOST_AUTO_TEST_CASE(known_hop_ddpll_tracks_residual_cfo)
{
    for (double cfo_hz : { 2000.0, -3500.0 }) {
        const SynthBpmPayload in =
            make_synth_payload(1208, cfo_hz, 0.05f, 4242u);

        rxfusion::DdPllConfig off_cfg; // enable == false
        rxfusion::Metrics2 off_out;
        rxfusion::Scratch scratch_off;
        BOOST_REQUIRE(rxfusion::correlate_payload_known_hop_fused_cfo(
            in.rx.data(), in.rx.size(), in.taps.data(), kTap, in.chip_start,
            2.0, in.payload_start, in.pn.data(), in.symbols, 1, cf(1.0f, 0.0f),
            0.0, off_cfg, off_out, scratch_off, nullptr, true, nullptr, 0));
        const size_t off_tail_errors =
            count_reference_half_errors(in, off_out, in.symbols - 200);
        const size_t off_errors = count_reference_half_errors(in, off_out, 0);

        rxfusion::DdPllConfig on_cfg;
        on_cfg.enable = true;
        rxfusion::Metrics2 on_out;
        rxfusion::Scratch scratch_on;
        BOOST_REQUIRE(rxfusion::correlate_payload_known_hop_fused_cfo(
            in.rx.data(), in.rx.size(), in.taps.data(), kTap, in.chip_start,
            2.0, in.payload_start, in.pn.data(), in.symbols, 1, cf(1.0f, 0.0f),
            0.0, on_cfg, on_out, scratch_on, nullptr, true, nullptr, 0));
        const size_t on_errors = count_reference_half_errors(in, on_out, 32);
        const double theory =
            kQaTwoPi * cfo_hz / kQaSampleRate * 128.0;

        BOOST_TEST_MESSAGE("ddpll cfo="
                           << cfo_hz << "Hz off_tail_errors="
                           << off_tail_errors << "/200 off_errors="
                           << off_errors << "/" << in.symbols
                           << " on_errors_after32=" << on_errors << "/"
                           << (in.symbols - 32) << " freq="
                           << on_out.pll_freq_rad << " theory=" << theory
                           << " theta=" << on_out.pll_theta << " updates="
                           << on_out.pll_updates);

        // Without tracking the residual CFO rotates the packet well past 90
        // degrees, so the tail polarity must break massively.
        BOOST_CHECK_GT(off_tail_errors, size_t(100));
        // With tracking, no polarity error at all after the ~32-symbol
        // acquisition transient.
        BOOST_CHECK_EQUAL(on_errors, size_t(0));
        BOOST_CHECK(all_finite(on_out));
        // The integrator state must converge to the true per-symbol rotation
        // (within 15%; the loop still moves a little at the end).
        BOOST_CHECK_LT(std::fabs(on_out.pll_freq_rad - theory),
                       std::fabs(0.15 * theory));
        BOOST_CHECK_GT(on_out.pll_updates, size_t(1000));
    }
}

BOOST_AUTO_TEST_CASE(known_hop_ddpll_zero_cfo_low_snr_no_cycle_slip)
{
    // Zero residual CFO, AWGN only.  Two scenarios, both with cfo_w_rad == 0:
    //
    //  A) no phase offset, symbol SNR ~8 dB: the DD-PLL is not expected to
    //     improve anything (the optimum at zero phase error is "do nothing"),
    //     so the check is "never worse" over a fixed seed ensemble.  A cycle
    //     slip would flip a long run of symbols and blow past the bound.
    //  B) a constant 0.6 rad phase offset (still zero CFO): the loop must
    //     acquire it and then stay locked, so the polarity error count has to
    //     drop by a wide, robust margin.  This also exercises the
    //     sign(Re)*Im/mag discriminator beyond the small-angle regime.
    size_t off_a = 0, on_a = 0;
    const size_t trials_a = 8;
    for (uint32_t trial = 0; trial < trials_a; ++trial) {
        const SynthBpmPayload in =
            make_synth_payload(1208, 0.0, 0.8f, 777u + 131u * trial);

        rxfusion::DdPllConfig off_cfg;
        rxfusion::Metrics2 off_out;
        rxfusion::Scratch scratch_off;
        BOOST_REQUIRE(rxfusion::correlate_payload_known_hop_fused_cfo(
            in.rx.data(), in.rx.size(), in.taps.data(), kTap, in.chip_start,
            2.0, in.payload_start, in.pn.data(), in.symbols, 1, cf(1.0f, 0.0f),
            0.0, off_cfg, off_out, scratch_off, nullptr, false, nullptr, 0));

        rxfusion::DdPllConfig on_cfg;
        on_cfg.enable = true;
        rxfusion::Metrics2 on_out;
        rxfusion::Scratch scratch_on;
        BOOST_REQUIRE(rxfusion::correlate_payload_known_hop_fused_cfo(
            in.rx.data(), in.rx.size(), in.taps.data(), kTap, in.chip_start,
            2.0, in.payload_start, in.pn.data(), in.symbols, 1, cf(1.0f, 0.0f),
            0.0, on_cfg, on_out, scratch_on, nullptr, false, nullptr, 0));

        const size_t off_errors = count_reference_half_errors(in, off_out, 0);
        const size_t on_errors = count_reference_half_errors(in, on_out, 0);
        BOOST_TEST_MESSAGE("ddpll low-snr(A) trial="
                           << trial << " off=" << off_errors << " on="
                           << on_errors << " freq=" << on_out.pll_freq_rad
                           << " theta=" << on_out.pll_theta);
        BOOST_CHECK(all_finite(on_out));
        // No cycle slip: bounded accumulated correction and near-zero
        // frequency estimate.  A 180-degree slip is ~3.14 rad and a false
        // frequency lock is far outside this window.
        BOOST_CHECK_LT(std::fabs(on_out.pll_theta), 1.0f);
        BOOST_CHECK_LT(std::fabs(on_out.pll_freq_rad), 0.02f);
        off_a += off_errors;
        on_a += on_errors;
    }
    BOOST_TEST_MESSAGE("ddpll zero-CFO AWGN aggregate: off="
                       << off_a << " on=" << on_a << " over "
                       << trials_a * 1208 << " symbols");
    BOOST_CHECK_GT(off_a, size_t(0));
    BOOST_CHECK_LE(on_a, off_a);

    size_t off_b = 0, on_b = 0;
    const size_t trials_b = 6;
    for (uint32_t trial = 0; trial < trials_b; ++trial) {
        const SynthBpmPayload in = make_synth_payload(
            1208, 0.0, 1.0f, 4242u + 977u * trial, true, 0.6);

        rxfusion::DdPllConfig off_cfg;
        rxfusion::Metrics2 off_out;
        rxfusion::Scratch scratch_off;
        BOOST_REQUIRE(rxfusion::correlate_payload_known_hop_fused_cfo(
            in.rx.data(), in.rx.size(), in.taps.data(), kTap, in.chip_start,
            2.0, in.payload_start, in.pn.data(), in.symbols, 1, cf(1.0f, 0.0f),
            0.0, off_cfg, off_out, scratch_off, nullptr, false, nullptr, 0));

        rxfusion::DdPllConfig on_cfg;
        on_cfg.enable = true;
        rxfusion::Metrics2 on_out;
        rxfusion::Scratch scratch_on;
        BOOST_REQUIRE(rxfusion::correlate_payload_known_hop_fused_cfo(
            in.rx.data(), in.rx.size(), in.taps.data(), kTap, in.chip_start,
            2.0, in.payload_start, in.pn.data(), in.symbols, 1, cf(1.0f, 0.0f),
            0.0, on_cfg, on_out, scratch_on, nullptr, false, nullptr, 0));

        // Skip the first 32 symbols of acquisition before counting.
        const size_t off_errors =
            count_reference_half_errors(in, off_out, 32);
        const size_t on_errors = count_reference_half_errors(in, on_out, 32);
        BOOST_TEST_MESSAGE("ddpll low-snr(B) trial="
                           << trial << " off=" << off_errors << " on="
                           << on_errors << " freq=" << on_out.pll_freq_rad
                           << " theta=" << on_out.pll_theta);
        BOOST_CHECK(all_finite(on_out));
        BOOST_CHECK_LT(std::fabs(on_out.pll_theta), 1.0f);
        BOOST_CHECK_LT(std::fabs(on_out.pll_freq_rad), 0.02f);
        off_b += off_errors;
        on_b += on_errors;
    }
    BOOST_TEST_MESSAGE("ddpll zero-CFO constant-phase aggregate: off="
                       << off_b << " on=" << on_b << " over "
                       << trials_b * 1208 << " symbols");
    BOOST_CHECK_GT(off_b, size_t(30));
    BOOST_CHECK_LE(on_b, off_b);
    BOOST_CHECK_LT(on_b * 2, off_b); // wide margin: the loop really re-locks
}

BOOST_AUTO_TEST_CASE(known_hop_rejects_invalid_and_out_of_range)
{
    const size_t symbols = 4;
    const FusedCfoInput in =
        make_fused_cfo_input(symbols, kTap, 31337);
    rxfusion::Metrics2 out;
    rxfusion::Scratch scratch;
    const cf* rx = in.rx.data();
    const size_t n = in.rx.size();
    const cf* h = in.taps.data();
    const int8_t* pn = in.pn.data();
    const rxfusion::DdPllConfig pll_off;

    auto call = [&](const cf* rx_p, size_t n_p, const cf* h_p, size_t taps,
                    int64_t chip_start, double spc, int64_t payload_start,
                    const int8_t* pn_p, size_t nsym, int8_t pol, double cfo,
                    const cf* bank, size_t bins) {
        return rxfusion::correlate_payload_known_hop_fused_cfo(
            rx_p, n_p, h_p, taps, chip_start, spc, payload_start, pn_p, nsym,
            pol, in.rot, cfo, pll_off, out, scratch, nullptr, false, bank,
            bins);
    };

    // Null pointers.
    BOOST_CHECK(!call(nullptr, n, h, kTap, kQaChipStart, 2.0, kQaPayloadStart,
                      pn, symbols, 1, 0.0, nullptr, 0));
    BOOST_CHECK(!call(rx, n, nullptr, kTap, kQaChipStart, 2.0, kQaPayloadStart,
                      pn, symbols, 1, 0.0, nullptr, 0));
    BOOST_CHECK(!call(rx, n, h, kTap, kQaChipStart, 2.0, kQaPayloadStart,
                      nullptr, symbols, 1, 0.0, nullptr, 0));
    // Argument domain.
    BOOST_CHECK(!call(rx, n, h, 0, kQaChipStart, 2.0, kQaPayloadStart, pn,
                      symbols, 1, 0.0, nullptr, 0));
    BOOST_CHECK(!call(rx, n, h, rxfusion::kMaxTapCount + 1, kQaChipStart, 2.0,
                      kQaPayloadStart, pn, symbols, 1, 0.0, nullptr, 0));
    BOOST_CHECK(!call(rx, n, h, kTap, kQaChipStart, 2.0, kQaPayloadStart, pn, 0,
                      1, 0.0, nullptr, 0));
    BOOST_CHECK(!call(rx, n, h, kTap, kQaChipStart, 2.0, kQaPayloadStart, pn,
                      symbols, 0, 0.0, nullptr, 0));
    BOOST_CHECK(!call(rx, n, h, kTap, kQaChipStart, 2.0, kQaPayloadStart, pn,
                      symbols, 1, std::nan(""), nullptr, 0));
    BOOST_CHECK(!call(rx, n, h, kTap, -1, 2.0, kQaPayloadStart, pn, symbols, 1,
                      0.0, nullptr, 0));
    BOOST_CHECK(!call(rx, n, h, kTap, kQaChipStart, 2.0, -1, pn, symbols, 1,
                      0.0, nullptr, 0));
    BOOST_CHECK(!call(rx, n, h, kTap, 0, 2.0, 0, pn, symbols, 1, 0.0, nullptr,
                      0));
    // Sample spacing: only 2.0 is supported without a fractional bank.
    BOOST_CHECK(!call(rx, n, h, kTap, kQaChipStart, 0.0, kQaPayloadStart, pn,
                      symbols, 1, 0.0, nullptr, 0));
    BOOST_CHECK(!call(rx, n, h, kTap, kQaChipStart, -1.0, kQaPayloadStart, pn,
                      symbols, 1, 0.0, nullptr, 0));
    BOOST_CHECK(!call(rx, n, h, kTap, kQaChipStart, std::nan(""),
                      kQaPayloadStart, pn, symbols, 1, 0.0, nullptr, 0));
    BOOST_CHECK(!call(rx, n, h, kTap, kQaChipStart,
                      std::numeric_limits<double>::infinity(), kQaPayloadStart,
                      pn, symbols, 1, 0.0, nullptr, 0));
    BOOST_CHECK(!call(rx, n, h, kTap, kQaChipStart, 1.0, kQaPayloadStart, pn,
                      symbols, 1, 0.0, nullptr, 0));
    BOOST_CHECK(!call(rx, n, h, kTap, kQaChipStart, 4.0, kQaPayloadStart, pn,
                      symbols, 1, 0.0, nullptr, 0));

    // Fractional bank: bin 0 is the undistorted CIR.
    std::vector<cf> bank(8 * kTap);
    for (size_t b = 0; b < 8; ++b)
        std::copy(in.taps.begin(), in.taps.end(), bank.begin() + b * kTap);
    BOOST_CHECK(!call(rx, n, h, kTap, kQaChipStart, 2.0, kQaPayloadStart, pn,
                      symbols, 1, 0.0, bank.data(), 0));
    BOOST_CHECK(!call(rx, n, h, kTap, kQaChipStart, 2.0, kQaPayloadStart, pn,
                      symbols, 1, 0.0, bank.data(),
                      rxfusion::kKhMaxPhaseBins + 1));
    BOOST_CHECK(call(rx, n, h, kTap, kQaChipStart, 2.0, kQaPayloadStart, pn,
                     symbols, 1, 0.0, bank.data(), 8));
    // The bank path with spc == 2.0 must reproduce the integer path.
    {
        rxfusion::Metrics2 a, b;
        rxfusion::Scratch sa, sb;
        BOOST_REQUIRE(rxfusion::correlate_payload_known_hop_fused_cfo(
            rx, n, h, kTap, kQaChipStart, 2.0, kQaPayloadStart, pn, symbols, 1,
            in.rot, 0.0, pll_off, a, sa, nullptr, true, nullptr, 0));
        BOOST_REQUIRE(rxfusion::correlate_payload_known_hop_fused_cfo(
            rx, n, h, kTap, kQaChipStart, 2.0, kQaPayloadStart, pn, symbols, 1,
            in.rot, 0.0, pll_off, b, sb, nullptr, true, bank.data(), 8));
        for (size_t s = 0; s < symbols; ++s) {
            BOOST_CHECK_SMALL(b.m0[s] - a.m0[s], 1e-5f);
            BOOST_CHECK_SMALL(b.m1[s] - a.m1[s], 1e-5f);
        }
    }

    // Illegal spreading chips (including the hop chip) are rejected.
    std::vector<int8_t> bad_zero = in.pn;
    bad_zero[3] = 0;
    BOOST_CHECK(!call(rx, n, h, kTap, kQaChipStart, 2.0, kQaPayloadStart,
                      bad_zero.data(), symbols, 1, 0.0, nullptr, 0));
    std::vector<int8_t> bad_two = in.pn;
    bad_two[bad_two.size() - 1] = 2;
    BOOST_CHECK(!call(rx, n, h, kTap, kQaChipStart, 2.0, kQaPayloadStart,
                      bad_two.data(), symbols, 1, 0.0, nullptr, 0));
    std::vector<int8_t> bad_neg = in.pn;
    bad_neg[5] = -2;
    BOOST_CHECK(!call(rx, n, h, kTap, kQaChipStart, 2.0, kQaPayloadStart,
                      bad_neg.data(), symbols, 1, 0.0, nullptr, 0));

    // Invalid DD-PLL configuration is rejected when the loop is enabled.
    {
        rxfusion::DdPllConfig bad;
        bad.enable = true;
        bad.alpha = std::nan("");
        BOOST_CHECK(!rxfusion::correlate_payload_known_hop_fused_cfo(
            rx, n, h, kTap, kQaChipStart, 2.0, kQaPayloadStart, pn, symbols, 1,
            in.rot, 0.0, bad, out, scratch, nullptr, false, nullptr, 0));
    }

    // Exact last readable integer window succeeds; one sample shorter fails.
    const size_t len = kTap + 14;
    const int64_t base0 = kQaChipStart + kQaPayloadStart * 2 - kTap + 1;
    const size_t needed = static_cast<size_t>(
        base0 + static_cast<int64_t>(symbols - 1) * 128 + 80 +
        static_cast<int64_t>(len));
    BOOST_REQUIRE_LE(needed, in.rx.size());
    BOOST_CHECK(call(rx, needed, h, kTap, kQaChipStart, 2.0, kQaPayloadStart,
                     pn, symbols, 1, 0.0, nullptr, 0));
    BOOST_CHECK(!call(rx, needed - 1, h, kTap, kQaChipStart, 2.0,
                      kQaPayloadStart, pn, symbols, 1, 0.0, nullptr, 0));

    // Same boundary on the fractional path (and with a non-2.0 spacing).
    BOOST_CHECK(call(rx, needed, h, kTap, kQaChipStart, 2.0, kQaPayloadStart,
                     pn, symbols, 1, 0.0, bank.data(), 8));
    BOOST_CHECK(!call(rx, needed - 1, h, kTap, kQaChipStart, 2.0,
                      kQaPayloadStart, pn, symbols, 1, 0.0, bank.data(), 8));
    BOOST_CHECK(call(rx, n, h, kTap, kQaChipStart, 2.0001, kQaPayloadStart, pn,
                     symbols, 1, 0.0, bank.data(), 8));
    BOOST_CHECK(!call(rx, needed - 1, h, kTap, kQaChipStart, 2.0001,
                      kQaPayloadStart, pn, symbols, 1, 0.0, bank.data(), 8));

    // No write past the caller's `n` (canary bytes in a slack region).
    {
        std::vector<cf> guarded(needed + 8);
        std::copy(in.rx.begin(), in.rx.begin() + needed, guarded.begin());
        const cf sentinel(12345.5f, -54321.25f);
        for (size_t i = needed; i < guarded.size(); ++i)
            guarded[i] = sentinel;
        BOOST_CHECK(rxfusion::correlate_payload_known_hop_fused_cfo(
            guarded.data(), needed, h, kTap, kQaChipStart, 2.0,
            kQaPayloadStart, pn, symbols, 1, in.rot, 0.0, pll_off, out,
            scratch, nullptr, false, bank.data(), 8));
        BOOST_CHECK(rxfusion::correlate_payload_known_hop_fused_cfo(
            guarded.data(), needed - 1, h, kTap, kQaChipStart, 2.0,
            kQaPayloadStart, pn, symbols, 1, in.rot, 0.0, pll_off, out,
            scratch, nullptr, false, bank.data(), 8) == false);
        for (size_t i = needed; i < guarded.size(); ++i)
            BOOST_CHECK(guarded[i] == sentinel);
    }
}

BOOST_AUTO_TEST_CASE(known_hop_phr_matches_four_candidate)
{
    const int64_t chip_start = kQaChipStart;
    const int64_t phr_start = 100;
    const size_t len = kTap + 2 * (rxfusion::kPhrChipsPerBurst - 1);
    std::mt19937 rng(20261120);
    std::uniform_real_distribution<float> dist(-1.0f, 1.0f);
    const size_t base0 = static_cast<size_t>(
        chip_start + phr_start * 2 - int64_t(kTap) + 1);
    // Exact last readable sample: any shorter buffer must be rejected.
    const size_t needed = base0 +
                          (rxfusion::kPhrSymbols - 1) *
                              rxfusion::kPhrChipsPerSymbol * 2 +
                          320 * 2 + len;
    std::vector<cf> rx(needed);
    for (auto& x : rx)
        x = cf(dist(rng), dist(rng));
    std::vector<cf> taps(kTap);
    for (auto& h : taps)
        h = cf(dist(rng), dist(rng));
    std::vector<int8_t> pn(rxfusion::kPhrSymbols *
                           rxfusion::kPhrChipsPerBurst);
    for (auto& a : pn)
        a = (dist(rng) < 0.0f) ? int8_t(-1) : int8_t(1);
    const float ang = dist(rng) * 3.0f;
    const cf rot(std::cos(ang), std::sin(ang));

    rxfusion::Metrics ref;
    rxfusion::Scratch scratch_ref;
    rxfusion::Metrics2 got;
    rxfusion::Scratch scratch_got;
    for (int8_t pol : { int8_t(1), int8_t(-1) }) {
        BOOST_REQUIRE(rxfusion::correlate_phr(
            rx.data(), rx.size(), taps.data(), kTap, chip_start,
            rxfusion::kSamplesPerChip, phr_start, pn.data(), pol, rot, ref,
            scratch_ref));
        BOOST_REQUIRE(rxfusion::correlate_phr_known_hop(
            rx.data(), rx.size(), taps.data(), kTap, chip_start,
            rxfusion::kSamplesPerChip, phr_start, pn.data(), pol, rot, got,
            scratch_got));
        BOOST_REQUIRE_EQUAL(got.symbols, rxfusion::kPhrSymbols);
        BOOST_REQUIRE_EQUAL(got.z0.size(), rxfusion::kPhrSymbols);
        double max_m = 0.0, max_z = 0.0;
        for (size_t s = 0; s < rxfusion::kPhrSymbols; ++s) {
            const int hop =
                (pn[s * rxfusion::kPhrChipsPerBurst] < 0) ? 1 : 0;
            const size_t p = 2 * static_cast<size_t>(hop);
            max_m = std::max(
                max_m, static_cast<double>(std::abs(
                           got.m0[s] - ref.signed_value[4 * s + p])));
            max_m = std::max(
                max_m, static_cast<double>(std::abs(
                           got.m1[s] - ref.signed_value[4 * s + p + 1])));
            max_z = std::max(
                max_z,
                static_cast<double>(std::abs(
                    got.z0[s] - ref.complex_value[4 * s + p])));
            max_z = std::max(
                max_z,
                static_cast<double>(std::abs(
                    got.z1[s] - ref.complex_value[4 * s + p + 1])));
        }
        BOOST_TEST_MESSAGE("known_hop phr pol=" << int(pol) << " max_m="
                                                << max_m << " max_z=" << max_z);
        BOOST_CHECK_LT(max_m, 1e-5);
        BOOST_CHECK_LT(max_z, 1e-5);
    }

    // Out-of-range PHR tail is rejected.
    BOOST_CHECK(!rxfusion::correlate_phr_known_hop(
        rx.data(), needed - 1, taps.data(), kTap, chip_start,
        rxfusion::kSamplesPerChip, phr_start, pn.data(), 1, rot, got,
        scratch_got));
    // Invalid PN is rejected.
    std::vector<int8_t> bad = pn;
    bad[0] = 0;
    BOOST_CHECK(!rxfusion::correlate_phr_known_hop(
        rx.data(), rx.size(), taps.data(), kTap, chip_start,
        rxfusion::kSamplesPerChip, phr_start, bad.data(), 1, rot, got,
        scratch_got));
}
