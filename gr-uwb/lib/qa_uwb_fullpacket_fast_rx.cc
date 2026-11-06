/* SPDX-License-Identifier: GPL-3.0-or-later */
#include <boost/test/unit_test.hpp>
#include "../apps/fullpacket_rx_pipeline.h"
#include <gnuradio/uwb/uwb_hrp_mod_core.h>
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
