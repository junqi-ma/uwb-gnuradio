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
 */

#include <gnuradio/uwb/uwb_echo_scheduler_core.h>
#include <gnuradio/uwb/uwb_uhd_backend_config.h>

#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>

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
            ++i;
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
                         "[--rx-channel N] [--clock-source S] "
                         "[--time-source S] [--pri-num N] [--pri-den N] "
                         "[--pre-guard-ticks N] [--tx-samples N] "
                         "[--rx-samples N] [--max-fragment-size N] "
                         "[--t0-ticks N] [--schedule-index N]\n";
            return 0;
        } else {
            std::cerr << "unknown or incomplete argument: " << a << "\n";
            return 2;
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
    return 0;
}
