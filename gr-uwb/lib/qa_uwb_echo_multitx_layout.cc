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

// ---------------------------------------------------------------------------
// Preamble-overlap asymmetric geometry + span bank (real packet collision).
// ---------------------------------------------------------------------------

BOOST_AUTO_TEST_CASE(test_overlap_geometry_degenerates_to_uniform)
{
    echo::MultiTxSpanGeometry g;
    echo::MultiTxGeometry u;
    std::string err;
    BOOST_REQUIRE(echo::prepare_overlap_geometry(100, 50, -20, 20, g, &err));
    BOOST_REQUIRE(echo::prepare_multitx_geometry(100, 50, 20, u, &err));
    BOOST_CHECK_EQUAL(g.P, 20u);
    BOOST_CHECK_EQUAL(g.R, 40u);
    BOOST_CHECK_EQUAL(g.phys_len_L, u.phys_len_L);
    BOOST_CHECK_EQUAL(g.jam_wave_begin, 40u);
    BOOST_CHECK_EQUAL(g.backing_len, g.phys_len_L + 40u);
}

BOOST_AUTO_TEST_CASE(test_overlap_geometry_one_sided)
{
    echo::MultiTxSpanGeometry g;
    std::string err;
    // All-positive (lag-only with Ns>Nj): sense parks at 0.
    BOOST_REQUIRE(echo::prepare_overlap_geometry(100, 50, 10, 30, g, &err));
    BOOST_CHECK_EQUAL(g.P, 0u);
    BOOST_CHECK_EQUAL(g.R, 20u);
    BOOST_CHECK_EQUAL(g.jam_wave_begin, 30u);
    BOOST_CHECK_EQUAL(g.phys_len_L, std::max<uint64_t>(100, 30 + 50));
    // All-negative (lead-only with Nj>Ns): sense parks at |dmin|.
    BOOST_REQUIRE(echo::prepare_overlap_geometry(100, 50, -30, -10, g, &err));
    BOOST_CHECK_EQUAL(g.P, 30u);
    BOOST_CHECK_EQUAL(g.R, 20u);
    BOOST_CHECK_EQUAL(g.jam_wave_begin, 20u);
    BOOST_CHECK_EQUAL(g.phys_len_L,
                      std::max<uint64_t>(30 + 100, 20 + 50));
    // Degenerate table (fixed aligned).
    BOOST_REQUIRE(echo::prepare_overlap_geometry(100, 50, 0, 0, g, &err));
    BOOST_CHECK_EQUAL(g.P, 0u);
    BOOST_CHECK_EQUAL(g.R, 0u);
    BOOST_CHECK_EQUAL(g.backing_len, g.phys_len_L);
    // Reversed / empty rejected.
    BOOST_CHECK(!echo::prepare_overlap_geometry(100, 50, 5, 4, g, &err));
    BOOST_CHECK(!echo::prepare_overlap_geometry(0, 50, 0, 0, g, &err));
    BOOST_CHECK(!echo::prepare_overlap_geometry(100, 0, 0, 0, g, &err));
}

BOOST_AUTO_TEST_CASE(test_overlap_span_materialize_all_delays)
{
    const uint64_t Ns = 10;
    const uint64_t Nj = 6;
    const int64_t dmin = -12;
    const int64_t dmax = 20;
    echo::MultiTxSpanGeometry g;
    std::string err;
    BOOST_REQUIRE(echo::prepare_overlap_geometry(Ns, Nj, dmin, dmax, g, &err));
    auto sense = make_wave(Ns, 700);
    auto jam = make_wave(Nj, -3000);
    const int16_t* waves[echo::kEchoMaxTxChannels] = { sense.data(),
                                                       jam.data() };
    const uint64_t lens[echo::kEchoMaxTxChannels] = { Ns, Nj };
    const uint64_t bases[echo::kEchoMaxTxChannels] = { g.P,
                                                       g.jam_wave_begin };
    echo::MultiTxWindowBank bank;
    BOOST_REQUIRE(echo::materialize_multitx_span_bank(
        waves, lens, bases, 2, 1, g, 1u << 21, bank, &err));
    BOOST_CHECK_EQUAL(bank.tx_len, g.phys_len_L);
    BOOST_CHECK_EQUAL(bank.sense_offset, g.P);
    BOOST_CHECK_EQUAL(bank.jam_backing_wave_begin, g.jam_wave_begin);
    BOOST_CHECK_EQUAL(bank.jam_dmin, dmin);
    BOOST_CHECK_EQUAL(bank.jam_dmax, dmax);
    BOOST_CHECK(bank.delay_mode == echo::JamDelayMode::PreambleOverlap);

    // TX0 pointer/content/positions are delay-independent.
    const int16_t* tx0 = bank.dense_rows[0].data();
    for (uint64_t s = 0; s < g.P; ++s) {
        BOOST_CHECK_EQUAL(tx0[s * 2], 0);
        BOOST_CHECK_EQUAL(tx0[s * 2 + 1], 0);
    }
    BOOST_CHECK(rows_equal(tx0 + g.P * 2, sense.data(), Ns));
    for (uint64_t s = g.P + Ns; s < g.phys_len_L; ++s)
        BOOST_CHECK_EQUAL(tx0[s * 2], 0);

    for (int64_t d = dmin; d <= dmax; ++d) {
        const int16_t* ptr = echo::multitx_jam_ptr(bank, d, &err);
        BOOST_REQUIRE(ptr != nullptr);
        std::vector<int16_t> win(ptr, ptr + g.phys_len_L * 2);
        const size_t at = static_cast<size_t>(g.P + d);
        BOOST_CHECK_MESSAGE(rows_equal(win.data() + at * 2, jam.data(), Nj),
                            "jammer packet truncated at delay=" +
                                std::to_string(d));
        for (size_t s = 0; s < at; ++s)
            BOOST_CHECK_EQUAL(win[s * 2], 0);
        for (size_t s = at + Nj; s < g.phys_len_L; ++s)
            BOOST_CHECK_EQUAL(win[s * 2], 0);
        BOOST_CHECK_EQUAL(bank.dense_rows[0].data(), tx0);
    }
    BOOST_CHECK(echo::multitx_jam_ptr(bank, dmax + 1, &err) == nullptr);
    BOOST_CHECK(echo::multitx_jam_ptr(bank, dmin - 1, &err) == nullptr);
}

BOOST_AUTO_TEST_CASE(test_overlap_rng_contract)
{
    // Same seed → bit-exact; different seed → different.
    auto draw_seq = [](uint64_t seed, int n) {
        echo::JamDelayRng rng(seed);
        std::vector<std::pair<int64_t, int>> out;
        out.reserve(n);
        for (int i = 0; i < n; ++i) {
            int64_t reps = 0;
            int side = 0;
            echo::draw_overlap_reps(rng, 1, 128, true, reps, side);
            out.emplace_back(reps, side);
        }
        return out;
    };
    const auto a = draw_seq(1234, 512);
    const auto b = draw_seq(1234, 512);
    const auto c = draw_seq(4321, 512);
    BOOST_CHECK(a == b);
    BOOST_CHECK(a != c);
    for (const auto& p : a) {
        BOOST_CHECK(p.first >= 1 && p.first <= 128);
        BOOST_CHECK(p.second == 0 || p.second == 1);
    }
    // A fixed side still consumes the second draw; a degenerate L still
    // consumes the first.
    {
        echo::JamDelayRng p(9), q(9);
        int64_t reps = 0;
        int side = 0;
        echo::draw_overlap_reps(p, 5, 5, false, reps, side);
        BOOST_CHECK_EQUAL(reps, 5);
        BOOST_CHECK_EQUAL(side, 0);
        const int64_t qr = q.next_range_consume(5, 5);
        (void)q.next_range_consume(0, 1);
        BOOST_CHECK_EQUAL(qr, 5);
        BOOST_CHECK_EQUAL(p.next_u32(), q.next_u32());
    }
    // Golden sequence: locks the PCG-XSH-RR + Lemire contract so a future
    // STL/compiler change cannot silently alter the random experiment.
    {
        echo::JamDelayRng rng(1234);
        const std::vector<std::pair<int64_t, int>> golden = {
            { 80, 0 }, { 36, 1 }, { 3, 1 },
            { 75, 0 }, { 63, 1 }, { 30, 1 },
        };
        for (const auto& g : golden) {
            int64_t reps = 0;
            int side = 0;
            echo::draw_overlap_reps(rng, 1, 128, true, reps, side);
            BOOST_CHECK_EQUAL(reps, g.first);
            BOOST_CHECK_EQUAL(side, g.second);
        }
    }
}
