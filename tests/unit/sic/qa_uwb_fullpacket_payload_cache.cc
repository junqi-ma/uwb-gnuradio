/* SPDX-License-Identifier: GPL-3.0-or-later */
#include <boost/test/unit_test.hpp>
#include <gnuradio/uwb/uwb_fullpacket_sic_core.h>

#include <cstring>
#include <limits>
#include <random>

namespace {
using namespace openuwb;
using cf = std::complex<float>;
constexpr size_t payload_start = 64 * 1016 + 8 * 1016 + 21 * 1024;
// The encoder concatenates19 PHR input bits with RS bits; the emitted
// last2 PHR symbols therefore depend on payload, before nominal payload_start.
constexpr size_t stable_phr_end = 64 * 1016 + 8 * 1016 + 19 * 1024;
constexpr size_t prefix_input = stable_phr_end - 64;
constexpr size_t prefix_output = prefix_input - 16;

bool byte_equal(const std::vector<cf>& a, const std::vector<cf>& b)
{
    return a.size() == b.size() &&
           std::memcmp(a.data(), b.data(), a.size() * sizeof(cf)) == 0;
}

struct CacheFixture {
    mod::HrpModConfig profile;
    mod::HrpModScratch tx;
    sic::SicModel model;
    sic::FullPacketScratch reference, cached;
    std::vector<uint8_t> wire, phr;
    std::vector<cf> waveform, sync, input, reference_out, reference_hat, out, hat;

    CacheFixture()
    {
        profile.code_index = 10;
        profile.sync_repetitions = 64;
        profile.sfd_mode = "4z2";
        profile.peak_amplitude = .8f;
        const size_t n = mod::packet_samples_998p4(64, 8, 127, false,
                                                  &profile.pulse) + 1024;
        tx.reserve(127, n);
        reference.reserve(n);
        cached.reserve(n);
        BOOST_REQUIRE(cached.prepare_prefix_cache());
        BOOST_REQUIRE_EQUAL(cached.prefix_input_len, prefix_input);
        BOOST_REQUIRE_EQUAL(cached.prefix_output_len, prefix_output);
        input.resize(n);
        for (size_t j = 0; j < n; ++j)
            input[j] = cf(float(int(j % 17) - 8) * .002f,
                          float(int(j % 13) - 6) * .003f);
        reference_out.resize(n); reference_hat.resize(n);
        out.resize(n); hat.resize(n);
        model.valid = true;
        model.state = sic::SicState::Committed;
        model.n_paths = 3;
        model.tau_abs[0] = 128.4;
        model.tau_abs[1] = 141.99; // phase31 blends wrapped phase0 at index-1
        model.tau_abs[2] = 157.0;
        model.gain[0] = cf(.8f, -.2f);
        model.gain[1] = cf(.2f, .1f);
        model.gain[2] = cf(-.075f, .03f);
        model.cfo_hz = 13217.25;
        model.phase_reference_sample = 128;
        model.sample_rate = 998.4e6;
        model.samples_per_symbol = 1016;
        model.nominal_sync_reps = 64;
        packet(32, 1);
    }

    void packet(size_t bytes, unsigned seed, float amplitude = .8f)
    {
        profile.peak_amplitude = amplitude;
        std::mt19937 rng(seed);
        wire.resize(bytes - 2);
        for (auto& b : wire) b = static_cast<uint8_t>(rng());
        mod::append_ieee_fcs(wire);
        std::vector<int8_t> p;
        mod::detail::encode_phr19(wire.size(), 64, false, p);
        phr.assign(p.begin(), p.end());
        waveform.resize(input.size());
        size_t written = 0;
        BOOST_REQUIRE(mod::modulate_one(wire.data(), wire.size(), profile,
                                        tx, waveform.data(), waveform.size(), written));
        waveform.resize(written);
        // Keep estimator-template normalization fixed while packet amplitude,
        // length and data vary; fullpacket_cancel must derive its own scale.
        if (sync.empty()) {
            sync.assign(waveform.begin(), waveform.begin() + 1016);
            for (auto& v : sync) v *= .37f;
        }
    }

    sic::FullPacketResult compare(bool fcs = true)
    {
        const auto r = sic::fullpacket_cancel(
            input.data(), input.size(), phr, wire, fcs, profile, model,
            sync.data(), sync.size(), 0, input.size(), reference,
            reference_out.data(), reference_hat.data());
        const auto c = sic::fullpacket_cancel(
            input.data(), input.size(), phr, wire, fcs, profile, model,
            sync.data(), sync.size(), 0, input.size(), cached,
            out.data(), hat.data());
        BOOST_REQUIRE_EQUAL(c.committed, r.committed);
        BOOST_CHECK_EQUAL(std::string(c.reason), std::string(r.reason));
        BOOST_CHECK_EQUAL(c.sub_lo, r.sub_lo);
        BOOST_CHECK_EQUAL(c.sub_hi, r.sub_hi);
        // Whole-window equality includes the cached/dynamic split, TX field
        // boundary, fractional-filter halo, multipath wrap and packet tail.
        BOOST_CHECK(byte_equal(reference_out, out));
        BOOST_CHECK(byte_equal(reference_hat, hat));
        return c;
    }

    void require_bypass() const
    {
        BOOST_CHECK(byte_equal(input, out));
        BOOST_CHECK(std::all_of(hat.begin(), hat.end(),
                               [](cf v) { return v == cf(0); }));
    }
};
} // namespace

BOOST_AUTO_TEST_CASE(payload_cache_exact_cold_warm_and_distinct_payloads)
{
    CacheFixture f;
    const auto cold = f.compare();
    BOOST_REQUIRE_MESSAGE(cold.committed, cold.reason);
    BOOST_CHECK(cold.prefix_cache_used);
    BOOST_CHECK(cold.prefix_key_changed);
    BOOST_CHECK_GT(cold.prefix_cache_misses, size_t(0));
    BOOST_CHECK_EQUAL(cold.prefix_cached_samples, prefix_output);

    const auto warm = f.compare();
    BOOST_REQUIRE(warm.committed);
    BOOST_CHECK(!warm.prefix_key_changed);
    BOOST_CHECK_GT(warm.prefix_cache_hits, size_t(0));
    BOOST_CHECK_EQUAL(warm.prefix_cache_misses, size_t(0));
    BOOST_CHECK_LT(warm.fir_output_samples, warm.reference_fir_output_samples);
    BOOST_CHECK_EQUAL(warm.fir_output_samples,
                     warm.nonzero_phase_calls * (f.waveform.size() - prefix_output));

    bool saw_payload_dependent_phr_tail = false;
    for (unsigned seed : {2U, 3U}) {
        const auto previous_waveform = f.waveform;
        const auto previous_phr = f.phr;
        const auto previous_hat = f.hat;
        f.packet(32, seed);
        BOOST_CHECK(f.phr == previous_phr);
        // Remove the global TX normalization ratio before comparing shape:
        // identical logical PHR bits do not make its emitted final2 symbols
        // payload-independent in the current concatenated encoder stream.
        const auto peak = std::max_element(previous_waveform.begin(),
                                           previous_waveform.begin() + 1016,
                                           [](cf a, cf b) { return std::abs(a) < std::abs(b); });
        const size_t peak_index = size_t(peak - previous_waveform.begin());
        BOOST_REQUIRE_GT(std::abs(*peak), 0.f);
        const float ratio = std::abs(f.waveform[peak_index]) / std::abs(*peak);
        for (size_t j = stable_phr_end; j < payload_start; ++j)
            saw_payload_dependent_phr_tail |=
                std::abs(f.waveform[j] - previous_waveform[j] * ratio) > 1e-5f;
        BOOST_CHECK(std::memcmp(previous_waveform.data() + payload_start,
                                f.waveform.data() + payload_start,
                                (f.waveform.size() - payload_start) * sizeof(cf)) != 0);
        const auto changed = f.compare();
        BOOST_REQUIRE_MESSAGE(changed.committed, changed.reason);
        BOOST_CHECK(!byte_equal(previous_hat, f.hat));
        // Normalization can change prefix bytes with payload, so only a repeat
        // of this exact packet is guaranteed to be a cache hit.
        const auto repeat = f.compare();
        BOOST_REQUIRE(repeat.committed);
        BOOST_CHECK(!repeat.prefix_key_changed);
        BOOST_CHECK_EQUAL(repeat.prefix_cache_misses, size_t(0));
        BOOST_CHECK_GT(repeat.prefix_cache_hits, size_t(0));
    }
    BOOST_CHECK(saw_payload_dependent_phr_tail);
}

BOOST_AUTO_TEST_CASE(payload_cache_invalidates_on_phr_and_actual_prefix_scale)
{
    CacheFixture f;
    BOOST_REQUIRE(f.compare().committed);
    for (size_t bytes : {size_t(16), size_t(127)}) {
        const std::vector<cf> old_prefix(f.waveform.begin(),
                                       f.waveform.begin() + prefix_input);
        f.packet(bytes, unsigned(bytes));
        BOOST_REQUIRE(std::memcmp(old_prefix.data(), f.waveform.data(),
                                  prefix_input * sizeof(cf)) != 0);
        const auto changed = f.compare();
        BOOST_REQUIRE_MESSAGE(changed.committed, changed.reason);
        BOOST_CHECK(changed.prefix_key_changed);
        BOOST_CHECK_GT(changed.prefix_cache_misses, size_t(0));
        const auto repeated = f.compare();
        BOOST_REQUIRE(repeated.committed);
        BOOST_CHECK_EQUAL(repeated.prefix_cache_misses, size_t(0));
    }
    const std::vector<cf> old_prefix(f.waveform.begin(),
                                   f.waveform.begin() + prefix_input);
    f.packet(127, 127, .61f);
    BOOST_REQUIRE(std::memcmp(old_prefix.data(), f.waveform.data(),
                              prefix_input * sizeof(cf)) != 0);
    const auto changed_scale = f.compare();
    BOOST_REQUIRE_MESSAGE(changed_scale.committed, changed_scale.reason);
    BOOST_CHECK(changed_scale.prefix_key_changed);
    BOOST_CHECK_GT(changed_scale.prefix_cache_misses, size_t(0));
}

BOOST_AUTO_TEST_CASE(payload_cache_failure_is_exact_bypass_then_valid_reuse)
{
    CacheFixture f;
    BOOST_REQUIRE(f.compare().committed);
    const uint32_t warm_mask = f.cached.prefix_phase_mask;
    BOOST_CHECK(!f.compare(false).committed);
    f.require_bypass();
    BOOST_CHECK_EQUAL(f.cached.prefix_phase_mask, warm_mask);

    f.model.valid = false;
    BOOST_CHECK(!f.compare().committed);
    f.require_bypass();
    f.model.valid = true;

    const auto saved = f.input.back();
    f.input.back() = cf(std::numeric_limits<float>::quiet_NaN(), 0);
    BOOST_CHECK(!f.compare().committed);
    f.require_bypass();
    f.input.back() = saved;

    const double delay = f.model.tau_abs[0];
    f.model.tau_abs[0] = double(f.input.size() - 64);
    BOOST_CHECK(!f.compare().committed);
    f.require_bypass();
    f.model.tau_abs[0] = delay;

    const auto resumed = f.compare();
    BOOST_REQUIRE_MESSAGE(resumed.committed, resumed.reason);
}
