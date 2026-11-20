/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * uwb_uhd_burst_dry_run — pure-contract dry-run CLI for the UWB radar
 * UhdBurstBackend (Radar Step 11).
 *
 * Prints the complete TX/RX/rate/window/PRI contract the backend would
 * execute — device-time tick grid, RX-before-TX geometry, fragment
 * planning, streamer/rate-strictness policy and the error mappings —
 * WITHOUT opening a device: this TU contains no UHD include and links
 * neither UHD nor gnuradio-uwb, so "no device opened" holds by
 * construction, not by policy.  Exit code 0 = contract printed;
 * nonzero = invalid arguments (nothing was opened either).
 *
 * Usage (defaults mirror the project radar contract):
 *   uwb_uhd_burst_dry_run
 *       [--device-args STR]      multi_usrp args, e.g. addr=192.168.10.2
 *       [--rate HZ]              radio rate (default 737280000)
 *       [--tx-channel N]         (default 0)
 *       [--rx-channel N]         (default 1)
 *       [--clock-source STR]     (default internal)
 *       [--time-source STR]      (default internal)
 *       [--pri-num N]            PRI ticks numerator (default 3686400 = 5 ms)
 *       [--pri-den N]            PRI ticks denominator (default 1)
 *       [--pre-guard-ticks N]    RX before TX (default 1475 = 2 us)
 *       [--tx-samples N]         TX burst length (default 140982 = full
 *                                64-SYNC normal packet @ 737.28 MS/s,
 *                                testdata/uwb_radar metadata)
 *       [--rx-samples N]         RX capture length (default 55591 ~ 75.4 us
 *                                = pre_guard + SYNC + SFD + range guard +
 *                                resampler tail, 开发需求 §5.3/§8.2)
 *       [--max-fragment-size N]  (default 65536)
 *       [--t0-ticks N]           first TX slot (default 36864000 = 0.05 s
 *                                arm delay at 737.28 MS/s)
 *       [--schedule-index N]     (default 0)
 *       [--tx-channels "0,1"]    multi-TX physical channels, comma
 *                                separated, 1..4 unique (exclusive with
 *                                --tx-channel; default = single
 *                                --tx-channel)
 *       [--tx-gains "g0,g1"]     per-channel gains dB, empty or N entries
 *                                (negative = leave; needs --tx-channels)
 *       [--tx-antennas "a0,a1"]  per-channel antennas, empty or N entries
 *                                (empty token = leave; needs --tx-channels)
 *       [--tx-freqs "f0,f1"]     per-channel freqs Hz, empty or N entries
 *                                (0 = leave; needs --tx-channels)
 *
 * The [multitx] trailer prints the logical→physical map, per-channel and
 * aggregate wire bytes, the active-waveform placeholder and the worst-case
 * fragment count (§6.3).
 */

#include <gnuradio/uwb/uwb_echo_multitx.h>
#include <gnuradio/uwb/uwb_echo_scheduler_core.h>
#include <gnuradio/uwb/uwb_uhd_backend_config.h>

#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

namespace {

bool parse_u64(const char* s, uint64_t& out)
{
    if (s == nullptr || *s == '\0')
        return false;
    char* end = nullptr;
    const unsigned long long v = std::strtoull(s, &end, 10);
    if (end == nullptr || *end != '\0')
        return false;
    out = static_cast<uint64_t>(v);
    return true;
}

bool parse_i64(const char* s, int64_t& out)
{
    if (s == nullptr || *s == '\0')
        return false;
    char* end = nullptr;
    const long long v = std::strtoll(s, &end, 10);
    if (end == nullptr || *end != '\0')
        return false;
    out = static_cast<int64_t>(v);
    return true;
}

bool parse_double(const char* s, double& out)
{
    if (s == nullptr || *s == '\0')
        return false;
    char* end = nullptr;
    const double v = std::strtod(s, &end);
    if (end == nullptr || *end != '\0' || !std::isfinite(v))
        return false;
    out = v;
    return true;
}

// Split "a,b,c" on commas, trimming ASCII blanks.  Empty tokens are kept
// (antennas use them as "leave untouched"; numerics reject them).
std::vector<std::string> split_csv(const char* s)
{
    std::vector<std::string> out;
    std::string cur;
    for (const char* p = s;; ++p) {
        if (*p == ',' || *p == '\0') {
            const size_t b = cur.find_first_not_of(" \t");
            const size_t e = cur.find_last_not_of(" \t");
            out.push_back(b == std::string::npos
                              ? std::string()
                              : cur.substr(b, e - b + 1));
            cur.clear();
            if (*p == '\0')
                break;
        } else {
            cur.push_back(*p);
        }
    }
    return out;
}

// M2 transitional mirror of uhd::multitx_detail::has_tx_channels (see
// uwb_uhd_burst_backend.h): this TU must also build UHD-free
// (ENABLE_UHD_BACKEND=OFF), so it cannot include that UHD-gated header.
template <typename C, typename = void>
struct HasTxChannels : std::false_type {};
template <typename C>
struct HasTxChannels<
    C,
    std::void_t<decltype(std::declval<const C&>().tx_channels)>>
    : std::true_type {};

std::string to_string_u128(unsigned __int128 v)
{
    if (v == 0)
        return "0";
    std::string s;
    while (v != 0) {
        s.push_back(static_cast<char>('0' + v % 10));
        v /= 10;
    }
    return std::string(s.rbegin(), s.rend());
}

} // namespace

int main(int argc, char** argv)
{
    gr::uwb::uhd::UhdBurstBackendConfig cfg;
    cfg.clock_source = "internal";
    cfg.time_source = "internal";

    gr::uwb::echo::EchoSchedulerConfig sched;
    uint64_t tx_samples = 140982;
    uint64_t rx_samples = 55591;
    int64_t t0_ticks = 36864000; // 0.05 s arm delay at 737.28 MS/s
    uint64_t schedule_index = 0;

    // Multi-TX CLI state (M2 §6.3).  --tx-channel stays the single-channel
    // spelling; --tx-channels "0,1" selects N channels with optional
    // per-channel gains/antennas/freqs (each empty or exactly N entries).
    bool tx_channel_single_given = false;
    std::string tx_channels_arg;
    std::string tx_gains_arg;
    std::string tx_antennas_arg;
    std::string tx_freqs_arg;

    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        const bool has_next = (i + 1 < argc);
        uint64_t u = 0;
        int64_t v = 0;
        if (a == "--device-args" && has_next) {
            cfg.device_args = argv[++i];
        } else if (a == "--rate" && has_next && parse_u64(argv[i + 1], u)) {
            cfg.sample_rate_hz = static_cast<double>(u);
            ++i;
        } else if (a == "--tx-channel" && has_next &&
                   parse_u64(argv[i + 1], u)) {
            cfg.tx_channel = static_cast<size_t>(u);
            tx_channel_single_given = true;
            ++i;
        } else if (a == "--tx-channels" && has_next) {
            tx_channels_arg = argv[++i];
        } else if (a == "--tx-gains" && has_next) {
            tx_gains_arg = argv[++i];
        } else if (a == "--tx-antennas" && has_next) {
            tx_antennas_arg = argv[++i];
        } else if (a == "--tx-freqs" && has_next) {
            tx_freqs_arg = argv[++i];
        } else if (a == "--rx-channel" && has_next &&
                   parse_u64(argv[i + 1], u)) {
            cfg.rx_channel = static_cast<size_t>(u);
            ++i;
        } else if (a == "--clock-source" && has_next) {
            cfg.clock_source = argv[++i];
        } else if (a == "--time-source" && has_next) {
            cfg.time_source = argv[++i];
        } else if (a == "--pri-num" && has_next && parse_i64(argv[i + 1], v)) {
            sched.pri_num = v;
            ++i;
        } else if (a == "--pri-den" && has_next && parse_i64(argv[i + 1], v)) {
            sched.pri_den = v;
            ++i;
        } else if (a == "--pre-guard-ticks" && has_next &&
                   parse_i64(argv[i + 1], v)) {
            sched.pre_guard_ticks = v;
            ++i;
        } else if (a == "--tx-samples" && has_next &&
                   parse_u64(argv[i + 1], u)) {
            tx_samples = u;
            ++i;
        } else if (a == "--rx-samples" && has_next &&
                   parse_u64(argv[i + 1], u)) {
            rx_samples = u;
            ++i;
        } else if (a == "--max-fragment-size" && has_next &&
                   parse_u64(argv[i + 1], u)) {
            sched.max_fragment_size = u;
            ++i;
        } else if (a == "--t0-ticks" && has_next && parse_i64(argv[i + 1], v)) {
            t0_ticks = v;
            ++i;
        } else if (a == "--schedule-index" && has_next &&
                   parse_u64(argv[i + 1], u)) {
            schedule_index = u;
            ++i;
        } else if (a == "--help" || a == "-h") {
            std::cout << "usage: uwb_uhd_burst_dry_run "
                         "[--device-args S] [--rate HZ] [--tx-channel N] "
                         "[--tx-channels \"0,1\"] [--tx-gains \"g0,g1\"] "
                         "[--tx-antennas \"a0,a1\"] [--tx-freqs \"f0,f1\"] "
                         "[--rx-channel N] [--clock-source S] "
                         "[--time-source S] [--pri-num N] [--pri-den N] "
                         "[--pre-guard-ticks N] [--tx-samples N] "
                         "[--rx-samples N] [--max-fragment-size N] "
                         "[--t0-ticks N] [--schedule-index N]\n"
                      << "  --tx-channel and --tx-channels are exclusive; "
                         "per-channel --tx-gains/--tx-antennas/--tx-freqs "
                         "need --tx-channels and take empty or exactly N "
                         "entries (gain<0 / empty antenna / freq 0 = leave "
                         "that channel untouched)\n";
            return 0;
        } else {
            std::cerr << "unknown or incomplete argument: " << a << "\n";
            return 2;
        }
    }

    // Resolve the TX channel list: legacy single --tx-channel vs multi
    // --tx-channels "0,1" (1..kEchoMaxTxChannels, no duplicates).
    std::vector<size_t> tx_chs;
    if (!tx_channels_arg.empty()) {
        if (tx_channel_single_given) {
            std::cerr << "dry-run: --tx-channel and --tx-channels are "
                         "exclusive\n";
            return 2;
        }
        for (const auto& tok : split_csv(tx_channels_arg.c_str())) {
            uint64_t u = 0;
            if (tok.empty() || !parse_u64(tok.c_str(), u)) {
                std::cerr << "dry-run: bad --tx-channels entry: '" << tok
                          << "'\n";
                return 2;
            }
            tx_chs.push_back(static_cast<size_t>(u));
        }
        if (tx_chs.empty() ||
            tx_chs.size() > gr::uwb::echo::kEchoMaxTxChannels) {
            std::cerr << "dry-run: --tx-channels needs 1.."
                      << gr::uwb::echo::kEchoMaxTxChannels << " entries\n";
            return 2;
        }
        for (size_t i = 0; i < tx_chs.size(); ++i) {
            for (size_t j = i + 1; j < tx_chs.size(); ++j) {
                if (tx_chs[i] == tx_chs[j]) {
                    std::cerr << "dry-run: duplicate TX channel "
                              << tx_chs[i] << "\n";
                    return 2;
                }
            }
        }
    } else {
        tx_chs.push_back(cfg.tx_channel);
    }
    const size_t ntx = tx_chs.size();

    // Per-channel options (defaults = leave that channel untouched).
    std::vector<std::string> tx_ants(ntx);
    std::vector<double> tx_gains(ntx, -1.0);
    std::vector<double> tx_freqs(ntx, 0.0);
    if (!tx_gains_arg.empty() || !tx_antennas_arg.empty() ||
        !tx_freqs_arg.empty()) {
        if (tx_channels_arg.empty()) {
            std::cerr << "dry-run: --tx-gains/--tx-antennas/--tx-freqs "
                         "need --tx-channels\n";
            return 2;
        }
        if (!tx_gains_arg.empty()) {
            const auto toks = split_csv(tx_gains_arg.c_str());
            if (toks.size() != ntx) {
                std::cerr << "dry-run: --tx-gains needs exactly " << ntx
                          << " entries\n";
                return 2;
            }
            for (size_t i = 0; i < ntx; ++i) {
                double g = 0.0;
                if (toks[i].empty() || !parse_double(toks[i].c_str(), g) ||
                    (g >= 0.0 && g > 120.0)) {
                    std::cerr << "dry-run: bad --tx-gains entry: '"
                              << toks[i] << "'\n";
                    return 2;
                }
                tx_gains[i] = g;
            }
        }
        if (!tx_antennas_arg.empty()) {
            const auto toks = split_csv(tx_antennas_arg.c_str());
            if (toks.size() != ntx) {
                std::cerr << "dry-run: --tx-antennas needs exactly " << ntx
                          << " entries\n";
                return 2;
            }
            tx_ants = toks; // empty token = leave untouched
        }
        if (!tx_freqs_arg.empty()) {
            const auto toks = split_csv(tx_freqs_arg.c_str());
            if (toks.size() != ntx) {
                std::cerr << "dry-run: --tx-freqs needs exactly " << ntx
                          << " entries\n";
                return 2;
            }
            for (size_t i = 0; i < ntx; ++i) {
                double f = 0.0;
                if (toks[i].empty() || !parse_double(toks[i].c_str(), f) ||
                    f < 0.0) {
                    std::cerr << "dry-run: bad --tx-freqs entry: '"
                              << toks[i] << "'\n";
                    return 2;
                }
                tx_freqs[i] = f;
            }
        }
    }

    // Legacy scalar compat: logical-0 drives the single-channel fields
    // (what today's device section prints pre-M1).
    cfg.tx_channel = tx_chs[0];
    if (!tx_ants[0].empty())
        cfg.tx_antenna = tx_ants[0];
    cfg.tx_gain_db = tx_gains[0];
    if (tx_freqs[0] > 0.0)
        cfg.center_freq_hz = tx_freqs[0];
    // M1 vector fill when the member exists (same field names as
    // uhd::assign_tx_channels; this TU mirrors it to stay UHD-free).
    if constexpr (HasTxChannels<
                      gr::uwb::uhd::UhdBurstBackendConfig>::value) {
        cfg.tx_channels.clear();
        for (size_t i = 0; i < ntx; ++i) {
            typename std::decay_t<decltype(cfg.tx_channels)>::value_type e;
            e.channel = tx_chs[i];
            e.antenna = tx_ants[i];
            e.gain_db = tx_gains[i];
            e.center_freq_hz = tx_freqs[i];
            cfg.tx_channels.push_back(e);
        }
    }

    gr::uwb::echo::EchoSchedulerPrepared grid;
    std::string err;
    if (!gr::uwb::echo::prepare_echo_scheduler(sched, grid, &err)) {
        std::cerr << "dry-run: invalid scheduler config: " << err << "\n";
        return 2;
    }

    gr::uwb::uhd::UhdDryRunPlan plan;
    if (!gr::uwb::uhd::build_uhd_dry_run_plan(
            cfg, grid, t0_ticks, schedule_index, tx_samples, rx_samples,
            plan, &err)) {
        std::cerr << "dry-run: invalid contract: " << err << "\n";
        return 2;
    }

    std::cout << gr::uwb::uhd::print_uhd_dry_run_plan(plan);

    // [multitx] section (M2 §6.3): logical→physical map, per-channel and
    // aggregate (wire) SC16 bytes, active-waveform placeholder, worst-case
    // fragment count.  The M3 schedule PDU refines the full-window
    // placeholder into per-channel sense/jam intervals; the fragment bound
    // below already uses the same planner (plan_fragments) as the worker.
    {
        auto d = [](double v) {
            std::ostringstream s;
            s << std::setprecision(17) << v;
            return s.str();
        };
        const unsigned __int128 per_ch_bytes =
            static_cast<unsigned __int128>(tx_samples) * 4;
        const unsigned __int128 wire_bytes = per_ch_bytes * ntx;
        std::cout << "[multitx]\n";
        std::cout << "tx_channel_count           = " << ntx << "\n";
        std::cout << "tx_map_logical_to_phys     = ";
        for (size_t i = 0; i < ntx; ++i)
            std::cout << (i ? ", " : "") << i << "->" << tx_chs[i];
        std::cout << "\n";
        for (size_t i = 0; i < ntx; ++i) {
            std::cout << "ch" << i << ": phys=" << tx_chs[i] << " antenna="
                      << (tx_ants[i].empty() ? "(untouched)" : tx_ants[i])
                      << " gain_db="
                      << (tx_gains[i] < 0.0 ? "(untouched)"
                                            : d(tx_gains[i]))
                      << " freq_hz="
                      << (tx_freqs[i] <= 0.0 ? "(untouched)"
                                             : d(tx_freqs[i]))
                      << " bytes=" << to_string_u128(per_ch_bytes) << "\n";
        }
        std::cout << "tx_bytes_per_channel       = "
                  << to_string_u128(per_ch_bytes) << " (L=" << tx_samples
                  << " * 4 B sc16)\n";
        std::cout << "tx_wire_bytes              = "
                  << to_string_u128(wire_bytes)
                  << " (channels * per-channel)\n";
        std::cout << "active_waveform            = placeholder full-window "
                     "[0, "
                  << tx_samples
                  << ") per channel (M3 schedule PDU refines sense/jam "
                     "intervals)\n";
        gr::uwb::echo::EchoFragmentSpan spans
            [gr::uwb::echo::kEchoMaxFragmentsPerBurst];
        size_t nfr = 0;
        if (gr::uwb::echo::plan_fragments(tx_samples,
                                          grid.max_fragment_size, spans,
                                          gr::uwb::echo::
                                              kEchoMaxFragmentsPerBurst,
                                          nfr)) {
            std::cout << "worst_tx_fragments         = " << nfr
                      << " (max_fragment_size=" << grid.max_fragment_size
                      << ", cap="
                      << gr::uwb::echo::kEchoMaxFragmentsPerBurst << ": ok)\n";
        } else {
            std::cout << "worst_tx_fragments         = EXCEEDS cap "
                      << gr::uwb::echo::kEchoMaxFragmentsPerBurst
                      << " (max_fragment_size=" << grid.max_fragment_size
                      << ")\n";
        }
    }
    return 0;
}
