/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Layout QA for the contiguous multi-TX window bank (X410 dual-TX
 * random-delay remediation).  Header-only helpers, UHD-free.
 */

#include <boost/test/unit_test.hpp>

#include <gnuradio/uwb/uwb_echo_multitx.h>

#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace echo = gr::uwb::echo;

namespace {

std::vector<int16_t>
make_wave(uint64_t n, int16_t seed)
{
    std::vector<int16_t> v(static_cast<size_t>(n) * 2);
    for (size_t i = 0; i < v.size(); ++i)
        v[i] = static_cast<int16_t>(seed + static_cast<int16_t>(i * 17));
    return v;
}

bool
rows_equal(const int16_t* a, const int16_t* b, uint64_t pairs)
{
    if (a == nullptr || b == nullptr)
        return false;
    for (uint64_t i = 0; i < pairs * 2; ++i) {
        if (a[i] != b[i])
            return false;
    }
    return true;
}

} // namespace

BOOST_AUTO_TEST_CASE(test_helpers_d_zero)
{
    uint64_t backing = 0, wave_begin = 0, win = 0;
    std::string err;
    BOOST_REQUIRE(echo::jam_backing_length(100, 0, backing, &err));
    BOOST_CHECK_EQUAL(backing, 100u);
    BOOST_REQUIRE(echo::jam_wave_begin(0, wave_begin, &err));
    BOOST_CHECK_EQUAL(wave_begin, 0u);
    BOOST_REQUIRE(echo::jam_window_begin(0, 0, win, &err));
    BOOST_CHECK_EQUAL(win, 0u);
}

BOOST_AUTO_TEST_CASE(test_helpers_delta_extremes)
{
    const uint64_t D = 7;
    const uint64_t L = 40;
    uint64_t backing = 0, wave_begin = 0, win = 0;
    std::string err;
    BOOST_REQUIRE(echo::jam_backing_length(L, D, backing, &err));
    BOOST_CHECK_EQUAL(backing, L + 2 * D);
    BOOST_REQUIRE(echo::jam_wave_begin(D, wave_begin, &err));
    BOOST_CHECK_EQUAL(wave_begin, 2 * D);
    BOOST_REQUIRE(echo::jam_window_begin(D, static_cast<int64_t>(D), win, &err));
    BOOST_CHECK_EQUAL(win, 0u); // D - (+D)
    BOOST_REQUIRE(echo::jam_window_begin(D, 0, win, &err));
    BOOST_CHECK_EQUAL(win, D);
    BOOST_REQUIRE(echo::jam_window_begin(D, -static_cast<int64_t>(D), win,
                                         &err));
    BOOST_CHECK_EQUAL(win, 2 * D);
    BOOST_CHECK(!echo::jam_window_begin(D, static_cast<int64_t>(D) + 1, win,
                                        &err));
    BOOST_CHECK(!echo::jam_window_begin(D, -static_cast<int64_t>(D) - 1, win,
                                        &err));
}

BOOST_AUTO_TEST_CASE(test_helpers_overflow)
{
    uint64_t out = 0;
    std::string err;
    const uint64_t half = std::numeric_limits<uint64_t>::max() / 2 + 1;
    BOOST_CHECK(!echo::jam_wave_begin(half, out, &err));
    BOOST_CHECK_EQUAL(err, "geometry overflow");
    BOOST_CHECK(!echo::jam_backing_length(1, half, out, &err));
    BOOST_CHECK(!echo::jam_backing_length(std::numeric_limits<uint64_t>::max(),
                                          1, out, &err));
    BOOST_CHECK(!echo::jam_window_begin(
        static_cast<uint64_t>(std::numeric_limits<int64_t>::max()) + 1u, 0,
        out, &err));
}

BOOST_AUTO_TEST_CASE(test_materialize_uniform_all_delays)
{
    const uint64_t Ns = 9;
    const uint64_t Nj = 5;
    const uint64_t D = 4;
    echo::MultiTxGeometry g;
    std::string err;
    BOOST_REQUIRE(echo::prepare_multitx_geometry(Ns, Nj, D, g, &err));
    const uint64_t L = g.phys_len_L;
    auto sense = make_wave(Ns, 1000);
    auto jam = make_wave(Nj, -4000);
    const int16_t* waves[echo::kEchoMaxTxChannels] = { sense.data(),
                                                       jam.data() };
    const uint64_t lens[echo::kEchoMaxTxChannels] = { Ns, Nj };
    const uint64_t bases[echo::kEchoMaxTxChannels] = { D, D };
    echo::MultiTxWindowBank bank;
    BOOST_REQUIRE(echo::materialize_multitx_window_bank(
        waves, lens, bases, 2, 1, L, echo::JamDelayMode::Uniform,
        1u << 21, bank, &err));
    BOOST_CHECK_EQUAL(bank.tx_len, L);
    BOOST_CHECK_EQUAL(bank.sense_offset, D);
    BOOST_CHECK_EQUAL(bank.jam_backing_wave_begin, 2 * D);
    BOOST_CHECK_EQUAL(bank.dense_rows[0].size(), L * 2);
    BOOST_CHECK_EQUAL(bank.jam_backing.size(), (L + 2 * D) * 2);

    // Guard zeros around the parked sense waveform; TX0 identical for
    // every delay (byte-for-byte the same buffer).
    const int16_t* tx0 = bank.dense_rows[0].data();
    for (uint64_t s = 0; s < D; ++s) {
        BOOST_CHECK_EQUAL(tx0[s * 2], 0);
        BOOST_CHECK_EQUAL(tx0[s * 2 + 1], 0);
    }
    BOOST_CHECK(rows_equal(tx0 + D * 2, sense.data(), Ns));
    for (uint64_t s = D + Ns; s < L; ++s)
        BOOST_CHECK_EQUAL(tx0[s * 2], 0);

    std::vector<int16_t> expect(static_cast<size_t>(L) * 2);
    for (int64_t delta = -static_cast<int64_t>(D);
         delta <= static_cast<int64_t>(D); ++delta) {
        const int16_t* ptr = echo::multitx_jam_ptr(bank, delta, &err);
        BOOST_REQUIRE(ptr != nullptr);
        BOOST_REQUIRE(echo::compose_jam_window(
            jam.data(), Nj, L, D, delta, expect.data(), &err));
        BOOST_CHECK_MESSAGE(rows_equal(ptr, expect.data(), L),
                            "TX1 window mismatch at delta=" +
                                std::to_string(delta));
        // Sliding window stays inside backing.
        uint64_t win = 0;
        BOOST_REQUIRE(echo::jam_window_begin(D, delta, win, &err));
        BOOST_CHECK_LE(win + L, L + 2 * D);
        // TX0 address never moves.
        BOOST_CHECK_EQUAL(bank.dense_rows[0].data(), tx0);
    }
}

BOOST_AUTO_TEST_CASE(test_materialize_unequal_lengths_and_silent_jam)
{
    const uint64_t Ns = 11;
    const uint64_t Nj = 3;
    const uint64_t D = 6;
    echo::MultiTxGeometry g;
    std::string err;
    BOOST_REQUIRE(echo::prepare_multitx_geometry(Ns, Nj, D, g, &err));
    auto sense = make_wave(Ns, 50);
    auto jam = make_wave(Nj, 9);
    const int16_t* waves[echo::kEchoMaxTxChannels] = { sense.data(),
                                                       jam.data() };
    uint64_t lens[echo::kEchoMaxTxChannels] = { Ns, Nj };
    uint64_t bases[echo::kEchoMaxTxChannels] = { D, D };
    echo::MultiTxWindowBank bank;
    BOOST_REQUIRE(echo::materialize_multitx_window_bank(
        waves, lens, bases, 2, 1, g.phys_len_L, echo::JamDelayMode::Uniform,
        1u << 21, bank, &err));

    // Silent jammer of non-zero length: all-zero waveform still occupies Nj.
    std::vector<int16_t> zeros(static_cast<size_t>(Nj) * 2, 0);
    waves[1] = zeros.data();
    echo::MultiTxWindowBank silent;
    BOOST_REQUIRE(echo::materialize_multitx_window_bank(
        waves, lens, bases, 2, 1, g.phys_len_L, echo::JamDelayMode::Uniform,
        1u << 21, silent, &err));
    for (int64_t delta = -static_cast<int64_t>(D);
         delta <= static_cast<int64_t>(D); ++delta) {
        const int16_t* ptr = echo::multitx_jam_ptr(silent, delta, &err);
        BOOST_REQUIRE(ptr != nullptr);
        for (uint64_t i = 0; i < g.phys_len_L * 2; ++i)
            BOOST_CHECK_EQUAL(ptr[i], 0);
    }
}

BOOST_AUTO_TEST_CASE(test_materialize_fixed_places_jam_at_delay)
{
    const uint64_t Ns = 8;
    const uint64_t Nj = 4;
    const uint64_t delay = 3;
    const uint64_t L = Ns > Nj + delay ? Ns : Nj + delay;
    auto sense = make_wave(Ns, 200);
    auto jam = make_wave(Nj, -90);
    const int16_t* waves[echo::kEchoMaxTxChannels] = { sense.data(),
                                                       jam.data() };
    const uint64_t lens[echo::kEchoMaxTxChannels] = { Ns, Nj };
    const uint64_t bases[echo::kEchoMaxTxChannels] = { 0, delay };
    echo::MultiTxWindowBank bank;
    std::string err;
    BOOST_REQUIRE(echo::materialize_multitx_window_bank(
        waves, lens, bases, 2, 1, L, echo::JamDelayMode::Fixed, 1u << 21,
        bank, &err));
    BOOST_CHECK(bank.jam_backing.empty());
    BOOST_CHECK(rows_equal(bank.dense_rows[0].data(), sense.data(), Ns));
    for (uint64_t s = Ns; s < L; ++s)
        BOOST_CHECK_EQUAL(bank.dense_rows[0][s * 2], 0);
    const int16_t* tx1 = echo::multitx_jam_ptr(bank, 0, &err);
    BOOST_REQUIRE(tx1 != nullptr);
    for (uint64_t s = 0; s < delay; ++s)
        BOOST_CHECK_EQUAL(tx1[s * 2], 0);
    BOOST_CHECK(rows_equal(tx1 + delay * 2, jam.data(), Nj));
}

BOOST_AUTO_TEST_CASE(test_tx0_identical_across_delays_bytewise)
{
    const uint64_t Ns = 7, Nj = 7, D = 3;
    echo::MultiTxGeometry g;
    std::string err;
    BOOST_REQUIRE(echo::prepare_multitx_geometry(Ns, Nj, D, g, &err));
    auto sense = make_wave(Ns, 1);
    auto jam = make_wave(Nj, 2);
    const int16_t* waves[echo::kEchoMaxTxChannels] = { sense.data(),
                                                       jam.data() };
    const uint64_t lens[echo::kEchoMaxTxChannels] = { Ns, Nj };
    const uint64_t bases[echo::kEchoMaxTxChannels] = { D, D };
    echo::MultiTxWindowBank bank;
    BOOST_REQUIRE(echo::materialize_multitx_window_bank(
        waves, lens, bases, 2, 1, g.phys_len_L, echo::JamDelayMode::Uniform,
        1u << 21, bank, &err));
    const auto snap = bank.dense_rows[0];
    const int16_t* tx0 = bank.dense_rows[0].data();
    for (int64_t delta = -static_cast<int64_t>(D);
         delta <= static_cast<int64_t>(D); ++delta) {
        (void)echo::multitx_jam_ptr(bank, delta, &err);
        BOOST_CHECK(bank.dense_rows[0] == snap);
        BOOST_CHECK_EQUAL(bank.dense_rows[0].data(), tx0);
    }
}

BOOST_AUTO_TEST_CASE(test_old_bounds_planner_still_compiles)
{
    // Production no longer calls this; keep the helper for existing QA.
    echo::MultiTxGeometry g;
    std::string err;
    BOOST_REQUIRE(echo::prepare_multitx_geometry(10, 4, 2, g, &err));
    uint64_t bounds[5] = {};
    const size_t n = echo::multitx_burst_bounds(g, 1, bounds, 5);
    BOOST_CHECK_GE(n, 2u);
    BOOST_CHECK_EQUAL(bounds[0], 0u);
    BOOST_CHECK_EQUAL(bounds[n - 1], g.phys_len_L);
}
