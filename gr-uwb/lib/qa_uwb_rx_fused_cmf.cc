/* SPDX-License-Identifier: GPL-3.0-or-later */
#include <boost/test/unit_test.hpp>

#include "../apps/fullpacket_rx_pipeline.h"
#include <gnuradio/uwb/uwb_hrp_mod_core.h>
#include <gnuradio/uwb/uwb_rx_fused_cmf.h>

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
    static constexpr size_t seed = 2048;

    Packet(size_t bytes, size_t code)
    {
        wire.resize(bytes - 2);
        for (size_t j = 0; j < wire.size(); ++j)
            wire[j] = uint8_t(19 * j + code);
        mod::append_ieee_fcs(wire);
        mod::HrpModConfig cfg;
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
    // Payload starts after 64 SYNC + 8 SFD symbols and 21 PHR symbols.  Zero a
    // long deterministic payload slice while leaving SFD/PHR intact, so the
    // fused attempt runs and fails rather than failing earlier.
    const size_t payload_start = Packet::seed +
                                 (64 + 8) * 1016 +
                                 21 * 1024;
    const size_t corrupt_lo = std::min(packet.iq.size(), payload_start + 100);
    const size_t corrupt_hi = std::min(packet.iq.size(), corrupt_lo + 4096);
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
