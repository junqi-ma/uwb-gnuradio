/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Host-side timing of IEEE 802.15.4a HRP BPRF packet synthesis
 * (modulate_one at 998.4 MS/s).
 *
 * P3 gate: 64-SYNC, PSDU 127 B, mean <= 1000 us/packet for both a
 * cached (identical) PSDU and a per-packet unique PSDU.  Prefix
 * SYNC/SFD/STS is cached after warmup.  2048-SYNC is informational.
 *
 * Usage:
 *   benchmark_hrp_mod [warmup] [rounds]
 *     warmup  untimed calls per profile (default 4)
 *     rounds  timed repetitions          (default 64)
 *
 * Exit 0 if the 64-SYNC 127 B gates pass.
 */

#include <gnuradio/uwb/uwb_hrp_mod_core.h>
#include <gnuradio/uwb/uwb_radar_pdu_meta.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <numeric>
#include <stdexcept>
#include <string>
#include <vector>

using gr::uwb::mod::HrpModConfig;
using gr::uwb::mod::HrpModScratch;
using gr::uwb::mod::kMaxHrpTxSamples;
using gr::uwb::mod::make_random_psdu;
using gr::uwb::mod::modulate_one;
using gr::uwb::mod::packet_samples_998p4;
using gr_complex = std::complex<float>;

namespace {

constexpr double kGateUs = 1000.0;
constexpr size_t kSync64 = 64;
constexpr size_t kPsdu127 = 127;

struct Stats {
    size_t n = 0;
    double min_us = 0;
    double max_us = 0;
    double mean_us = 0;
    double p50_us = 0;
    double p95_us = 0;
};

Stats
summarize(std::vector<double> xs)
{
    Stats s;
    if (xs.empty())
        return s;
    std::sort(xs.begin(), xs.end());
    s.n = xs.size();
    s.min_us = xs.front();
    s.max_us = xs.back();
    s.mean_us = std::accumulate(xs.begin(), xs.end(), 0.0) /
                static_cast<double>(xs.size());
    auto at = [&](double p) {
        const double idx = p * static_cast<double>(xs.size() - 1);
        return xs[static_cast<size_t>(std::llround(idx))];
    };
    s.p50_us = at(0.50);
    s.p95_us = at(0.95);
    return s;
}

void
print_stats(const char* name, const Stats& s, bool gated)
{
    const double pps = (s.mean_us > 0.0) ? (1e6 / s.mean_us) : 0.0;
    std::printf("  %-28s n=%3zu  mean=%8.1f  p50=%8.1f  p95=%8.1f  "
                "min=%8.1f  max=%8.1f us  (%.0f pkt/s)%s\n",
                name, s.n, s.mean_us, s.p50_us, s.p95_us, s.min_us, s.max_us,
                pps, gated ? "  [GATE]" : "");
}

template <typename F>
Stats
time_loop(size_t warmup, size_t rounds, F&& fn)
{
    for (size_t i = 0; i < warmup; ++i)
        fn();
    std::vector<double> xs;
    xs.reserve(rounds);
    for (size_t i = 0; i < rounds; ++i) {
        const auto t0 = std::chrono::steady_clock::now();
        fn();
        const auto t1 = std::chrono::steady_clock::now();
        xs.push_back(1e-3 * static_cast<double>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0)
                .count()));
    }
    return summarize(std::move(xs));
}

} // namespace

int
main(int argc, char** argv)
{
    const size_t warmup = (argc > 1) ? std::strtoul(argv[1], nullptr, 10) : 4;
    const size_t rounds = (argc > 2) ? std::strtoul(argv[2], nullptr, 10) : 64;
    if (warmup == 0 || rounds == 0) {
        std::fprintf(stderr, "warmup and rounds must be > 0\n");
        return 2;
    }

    HrpModScratch scratch;
    scratch.reserve(gr::uwb::radar_meta::kMaxPsduBytes, kMaxHrpTxSamples);
    std::vector<gr_complex> tx(kMaxHrpTxSamples);

    const auto psdu_fixed = make_random_psdu(125, 20260909u, true);
    if (psdu_fixed.size() != kPsdu127) {
        std::fprintf(stderr, "expected 127-byte PSDU, got %zu\n",
                     psdu_fixed.size());
        return 2;
    }

    HrpModConfig cfg;
    cfg.code_index = 9;
    cfg.sync_repetitions = kSync64;
    cfg.sfd_mode = "ieee";
    cfg.insert_sts = false;

    const size_t n64 = packet_samples_998p4(kSync64, 8, kPsdu127);
    size_t n = 0;
    if (!modulate_one(psdu_fixed.data(), psdu_fixed.size(), cfg, scratch,
                      tx.data(), tx.size(), n) ||
        n != n64) {
        std::fprintf(stderr, "modulate_one P1 profile failed (n=%zu want=%zu)\n",
                     n, n64);
        return 2;
    }

    std::printf("benchmark_hrp_mod  warmup=%zu  rounds=%zu\n", warmup, rounds);
    std::printf("  P3 gate: 64-SYNC PSDU=127 B mean <= %.0f us  (host-only)\n",
                kGateUs);
    std::printf("  packet 64-SYNC 127 B = %zu samples @ 998.4 MS/s\n", n64);

    const Stats cached = time_loop(warmup, rounds, [&]() {
        size_t nn = 0;
        if (!modulate_one(psdu_fixed.data(), psdu_fixed.size(), cfg, scratch,
                          tx.data(), tx.size(), nn) ||
            nn != n64)
            throw std::runtime_error("modulate_one cached failed");
    });

    std::vector<uint8_t> psdu_var = psdu_fixed;
    uint32_t seed = 1;
    const Stats unique = time_loop(warmup, rounds, [&]() {
        psdu_var = make_random_psdu(125, seed++, true);
        size_t nn = 0;
        if (!modulate_one(psdu_var.data(), psdu_var.size(), cfg, scratch,
                          tx.data(), tx.size(), nn) ||
            nn != n64)
            throw std::runtime_error("modulate_one unique failed");
    });

    HrpModConfig cfg0 = cfg;
    const size_t n0 = packet_samples_998p4(kSync64, 8, 0);
    const Stats psdu0 = time_loop(warmup, rounds, [&]() {
        size_t nn = 0;
        if (!modulate_one(nullptr, 0, cfg0, scratch, tx.data(), tx.size(),
                          nn) ||
            nn != n0)
            throw std::runtime_error("modulate_one psdu0 failed");
    });

    HrpModConfig cfg2k = cfg;
    cfg2k.sync_repetitions = 2048;
    const size_t n2k = packet_samples_998p4(2048, 8, 0);
    const Stats sync2k = time_loop(2, std::min<size_t>(rounds, 8), [&]() {
        size_t nn = 0;
        if (!modulate_one(nullptr, 0, cfg2k, scratch, tx.data(), tx.size(),
                          nn) ||
            nn != n2k)
            throw std::runtime_error("modulate_one 2048-SYNC failed");
    });

    HrpModConfig cfg_sts = cfg;
    cfg_sts.sfd_mode = "4z2";
    cfg_sts.insert_sts = true;
    const auto psdu22 = make_random_psdu(20, 9u, true);
    const size_t nsts = packet_samples_998p4(kSync64, 8, psdu22.size(), true);
    const Stats sts = time_loop(warmup, rounds, [&]() {
        size_t nn = 0;
        if (!modulate_one(psdu22.data(), psdu22.size(), cfg_sts, scratch,
                          tx.data(), tx.size(), nn) ||
            nn != nsts)
            throw std::runtime_error("modulate_one sts failed");
    });

    print_stats("64-SYNC 127B cached", cached, true);
    print_stats("64-SYNC 127B unique PSDU", unique, true);
    print_stats("64-SYNC 0B", psdu0, false);
    print_stats("2048-SYNC 0B", sync2k, false);
    print_stats("64-SYNC 4z2+STS 22B", sts, false);

    bool pass = true;
    if (!(cached.mean_us > 0.0) || cached.mean_us > kGateUs) {
        std::printf("FAIL: cached 64-SYNC 127 B mean %.1f us > %.0f us\n",
                    cached.mean_us, kGateUs);
        pass = false;
    }
    if (!(unique.mean_us > 0.0) || unique.mean_us > kGateUs) {
        std::printf("FAIL: unique 64-SYNC 127 B mean %.1f us > %.0f us\n",
                    unique.mean_us, kGateUs);
        pass = false;
    }
    if (pass) {
        std::printf("PASS: 64-SYNC PSDU<=127 B mean <= %.0f us "
                    "(cached %.0f pkt/s, unique %.0f pkt/s)\n",
                    kGateUs, 1e6 / cached.mean_us, 1e6 / unique.mean_us);
        return 0;
    }
    return 1;
}
