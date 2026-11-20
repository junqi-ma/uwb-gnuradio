// Native UHD -> production fixed-tone source -> streaming autocorrelation
// acquisition -> bounded packet jobs -> existing C++ demod -> console reporter.
//
// Phase 1 split: the CLI surface lives in live_rx_config.{h,cc}, the stdout
// telemetry in console_reporter.{h,cc}, the device-timed replay source in
// timed_replay_source.h, and the session itself in live_rx_session.{h,cc}.
// This file is only the process entry point.
#include "console_reporter.h"
#include "live_rx_config.h"
#include "live_rx_session.h"

#include <gnuradio/uwb/uwb_log.h>

#include <exception>

int main(int argc, char** argv)
try {
    openuwb::live::LiveRxCli cli =
        openuwb::live::parse_live_rx_cli(argc, argv);
    if (cli.help) {
        openuwb::live::print_live_help();
        return 0;
    }
    openuwb::live::validate_live_rx_config(cli.config);
    return openuwb::live::run_live_rx(cli.config);
} catch (const std::exception& e) {
    // Top-level diagnostics go to the shared logger (stderr).  The console
    // script only matches substrings and the exit status, and the prefix is
    // deliberately kept so existing operator habits survive.
    UWB_LOG_ERR("live", "ERROR " << e.what());
    return 2;
}
