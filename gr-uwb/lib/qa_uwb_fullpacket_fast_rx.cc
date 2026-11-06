/* SPDX-License-Identifier: GPL-3.0-or-later */
#include <boost/test/unit_test.hpp>
#include "../apps/fullpacket_rx_pipeline.h"
#include <gnuradio/uwb/uwb_hrp_mod_core.h>
#include <algorithm>
#include <cmath>
#include <utility>

namespace {
using namespace gr::uwb;
using cf = std::complex<float>;

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

BOOST_AUTO_TEST_CASE(fullpacket_fast_rx_decodes_complete_wire_psdu)
{
    // Two production-experiment roles and a payload-length boundary. Existing
    // demod/mod golden QA owns the sample-level algorithm equivalence tests.
    for (const auto spec : {std::pair<size_t, size_t>{32, 10}, {127, 9}}) {
        Packet packet(spec.first, spec.second);
        experiment::FastRxPipeline receiver(packet.iq.size(), spec.second);
        const auto result = receiver.run(packet.iq.data(), packet.iq.size(),
                                         7, Packet::seed, packet.sync);
        BOOST_REQUIRE_EQUAL(int(result.status), int(demod::DemodStatus::Success));
        BOOST_REQUIRE(result.payload.fcs_pass);
        BOOST_CHECK(result.payload.bytes == packet.wire);
        BOOST_CHECK_EQUAL(result.phr.psdu_length, packet.wire.size());
    }
}

BOOST_AUTO_TEST_CASE(fullpacket_fast_rx_rejects_invalid_bounds_and_recovers)
{
    Packet packet(32, 10);
    experiment::FastRxPipeline receiver(packet.iq.size(), 10);
    const auto null_input = receiver.run(nullptr, packet.iq.size(),
                                         1, Packet::seed, packet.sync);
    BOOST_CHECK_EQUAL(int(null_input.status), int(demod::DemodStatus::InvalidInput));
    const auto oversized = receiver.run(packet.iq.data(), packet.iq.size() + 1,
                                        2, Packet::seed, packet.sync);
    BOOST_CHECK_EQUAL(int(oversized.status), int(demod::DemodStatus::InvalidInput));
    const auto valid = receiver.run(packet.iq.data(), packet.iq.size(),
                                    3, Packet::seed, packet.sync);
    BOOST_REQUIRE_EQUAL(int(valid.status), int(demod::DemodStatus::Success));
    BOOST_CHECK(valid.payload.fcs_pass);
    BOOST_CHECK(valid.payload.bytes == packet.wire);
}

BOOST_AUTO_TEST_CASE(fullpacket_fast_rx_experiment_options_match_legacy)
{
    // Fast SFD/CFO, narrow tail seed and on-demand soft chips are explicit
    // opt-ins.  They may change intermediate ranges/normalization, but the
    // decoded PSDU, FCS, field positions and CIR first path must match.
    for (const auto& spec : { std::pair<size_t, size_t>{ 32, 10 },
                              std::pair<size_t, size_t>{ 127, 9 } }) {
        Packet packet(spec.first, spec.second);
        experiment::FastRxPipeline legacy(packet.iq.size(), spec.second);
        experiment::FastRxPipeline::Options options(true, true, true, true);
        experiment::FastRxPipeline fast(packet.iq.size(), spec.second, options);
        const auto a = legacy.run(packet.iq.data(), packet.iq.size(),
                                  11, Packet::seed, packet.sync);
        const auto b = fast.run(packet.iq.data(), packet.iq.size(),
                                12, Packet::seed, packet.sync);
        BOOST_REQUIRE_EQUAL(int(a.status), int(demod::DemodStatus::Success));
        BOOST_REQUIRE_EQUAL(int(b.status), int(demod::DemodStatus::Success));
        BOOST_CHECK(a.payload.fcs_pass);
        BOOST_CHECK(b.payload.fcs_pass);
        BOOST_CHECK(a.payload.bytes == b.payload.bytes);
        BOOST_CHECK_EQUAL(a.phr.psdu_length, b.phr.psdu_length);
        BOOST_CHECK_EQUAL(a.ns_sfd.sfd_start_chip, b.ns_sfd.sfd_start_chip);
        BOOST_CHECK_EQUAL(a.cir.first_path_sample, b.cir.first_path_sample);
        BOOST_CHECK_EQUAL(a.cir.samples_per_chip, b.cir.samples_per_chip);
        if (spec.first == 32) {
            BOOST_CHECK_GT(b.cir.initial_chip_count, 0u);
            BOOST_CHECK_LT(b.cir.initial_chip_count, a.cir.soft_chip_count);
            BOOST_CHECK_LE(b.cir.soft_chip_count, a.cir.soft_chip_count);
        }
    }
}

BOOST_AUTO_TEST_CASE(fullpacket_fast_rx_on_demand_falls_back_on_phr_failure)
{
    Packet packet(127, 9);
    // Destroy the SFD/PHR region after the preamble.  Both paths must report
    // the same failure status; the on-demand path retries the full stream once.
    const size_t corrupt_lo = Packet::seed + 64 * 1016;
    const size_t corrupt_hi =
        std::min(packet.iq.size(), corrupt_lo + 40 * 1016);
    for (size_t i = corrupt_lo; i < corrupt_hi; ++i)
        packet.iq[i] = cf(0.0f, 0.0f);
    experiment::FastRxPipeline legacy(packet.iq.size(), 9);
    experiment::FastRxPipeline::Options options(true, true, true, true);
    experiment::FastRxPipeline fast(packet.iq.size(), 9, options);
    const auto a = legacy.run(packet.iq.data(), packet.iq.size(),
                              21, Packet::seed, packet.sync);
    const auto b = fast.run(packet.iq.data(), packet.iq.size(),
                            22, Packet::seed, packet.sync);
    BOOST_CHECK_EQUAL(int(a.status), int(b.status));
    BOOST_CHECK_EQUAL(b.stage_softchip_fallbacks, 1u);
}

BOOST_AUTO_TEST_CASE(fullpacket_fast_rx_on_demand_chips_match_legacy_prefix)
{
    // On a long mixed window the legacy budget produces more chips than a 32 B
    // frame needs.  The on-demand stream must match the legacy chip values over
    // the produced prefix (up to the global positive normalization scale) and
    // decode the same PSDU.
    Packet packet(32, 10);
    const size_t n = 236416;
    std::vector<cf> iq(n, cf(0.0f, 0.0f));
    std::copy(packet.iq.begin(), packet.iq.end(), iq.begin());

    auto profile = demod::Qm35825Profile::Default();
    profile.code_index = 10;
    profile.timing_search_margin = 256;
    profile.timing_coarse_stride = 1;
    demod::core::DemodScratch legacy_scratch;
    demod::core::DemodScratch fast_scratch;
    legacy_scratch.reserve(n);
    fast_scratch.reserve(n);
    const auto a = demod::core::demodulate_one(
        iq.data(), n, profile, 31, Packet::seed, 0, packet.sync, legacy_scratch);
    auto fast_profile = profile;
    fast_profile.rx_softchip_on_demand = true;
    const auto b = demod::core::demodulate_one(
        iq.data(), n, fast_profile, 32, Packet::seed, 0, packet.sync,
        fast_scratch);
    BOOST_REQUIRE_EQUAL(int(a.status), int(demod::DemodStatus::Success));
    BOOST_REQUIRE_EQUAL(int(b.status), int(demod::DemodStatus::Success));
    BOOST_CHECK(a.payload.bytes == b.payload.bytes);
    BOOST_REQUIRE_GT(b.cir.soft_chip_count, 0u);
    BOOST_REQUIRE_LE(b.cir.soft_chip_count, legacy_scratch.soft_chips.size());
    BOOST_CHECK_LT(b.cir.soft_chip_count, a.cir.soft_chip_count);

    double num = 0.0, den = 0.0;
    for (size_t i = 0; i < b.cir.soft_chip_count; ++i) {
        const double x = legacy_scratch.soft_chips[i];
        const double y = fast_scratch.soft_chips[i];
        num += x * y;
        den += x * x;
    }
    const double scale = num / den;
    BOOST_CHECK_GT(scale, 0.0);
    double max_err = 0.0;
    for (size_t i = 0; i < b.cir.soft_chip_count; ++i) {
        const double d = std::abs(legacy_scratch.soft_chips[i] * scale -
                                  fast_scratch.soft_chips[i]);
        max_err = std::max(max_err, d);
    }
    BOOST_TEST_MESSAGE("on-demand prefix scale=" << scale
                       << " max_err=" << max_err
                       << " prefix_chips=" << b.cir.soft_chip_count
                       << " legacy_chips=" << a.cir.soft_chip_count);
    BOOST_CHECK_LT(max_err, 1e-5);
}
