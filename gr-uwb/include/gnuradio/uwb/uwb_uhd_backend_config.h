/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * UhdBurstBackend configuration, contract mapping and dry-run planning
 * (Radar Step 11) — UHD-FREE header.
 *
 * This header deliberately does NOT include any UHD header so that the
 * full contract surface of the real backend — configuration validation,
 * the UHD error-code → BurstStatus mappings, device-time tick conversions
 * and the no-device dry-run plan — is compiled and QA-ed on machines
 * without UHD.  The UHD-dependent implementation lives in
 * uwb_uhd_burst_backend.{h,cc} and is built only when CMake finds UHD
 * (ENABLE_UHD_BACKEND); that TU cross-checks the ABI constants below with
 * static_assert against the real UHD enums.
 *
 * UHD error/event ABI: rx_metadata_t::error_code and
 * async_metadata_t::event_code are part of the UHD ABI and have been
 * stable since UHD 3.x (values below); gr-uhd maps the same named
 * constants (gr-uhd/lib/usrp_source_impl.cc ERROR_CODE_* handling,
 * usrp_sink_impl.cc async_event_loop EVENT_CODE_* bit tests).
 */

#ifndef INCLUDED_GNURADIO_UWB_UWB_UHD_BACKEND_CONFIG_H
#define INCLUDED_GNURADIO_UWB_UWB_UHD_BACKEND_CONFIG_H

// Self-containment regression guard (same pattern as qa_uwb_echo_timer):
// this shared header must pull in its own size_t.
#include <gnuradio/uwb/uwb_radar_checked_math.h>
#include <gnuradio/uwb/uwb_echo_burst_backend.h>
#include <gnuradio/uwb/uwb_echo_scheduler_core.h>

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

namespace gr {
namespace uwb {
namespace uhd {

// ---------------------------------------------------------------------------
// UHD ABI constants (integer values; cross-checked with static_assert in
// the UHD-linked TU).
// ---------------------------------------------------------------------------

// uhd::rx_metadata_t::error_code_t (REAL UHD 4.1 ABI — NOT contiguous:
// late=0x2, broken_chain=0x4, overflow=0x8, alignment=0xc, bad_packet=0xf;
// cross-checked with static_assert in the UHD-linked TU).
inline constexpr int32_t kUhdRxErrorNone = 0x0;
inline constexpr int32_t kUhdRxErrorTimeout = 0x1;
inline constexpr int32_t kUhdRxErrorLateCommand = 0x2;
inline constexpr int32_t kUhdRxErrorBrokenChain = 0x4;
inline constexpr int32_t kUhdRxErrorOverflow = 0x8;
inline constexpr int32_t kUhdRxErrorAlignment = 0xc;
inline constexpr int32_t kUhdRxErrorBadPacket = 0xf;

// uhd::async_metadata_t::event_code_t (bitmask; USER_PAYLOAD = 0x40 is
// informational, not an error).
inline constexpr uint32_t kUhdAsyncBurstAck = 0x1;
inline constexpr uint32_t kUhdAsyncUnderflow = 0x2;
inline constexpr uint32_t kUhdAsyncSeqError = 0x4;
inline constexpr uint32_t kUhdAsyncTimeError = 0x8;
inline constexpr uint32_t kUhdAsyncUnderflowInPacket = 0x10;
inline constexpr uint32_t kUhdAsyncSeqErrorInBurst = 0x20;
inline constexpr uint32_t kUhdAsyncUserPayload = 0x40;

// Map one UHD RX stream error code (rx_metadata_t::error_code as int) to
// the backend BurstStatus the EchoTimer worker understands.  ERROR_CODE_NONE
// is not an error: it returns Ok (the caller continues its recv loop).
inline echo::BurstStatus map_uhd_rx_error_code(int32_t code)
{
    switch (code) {
    case kUhdRxErrorNone:
        return echo::BurstStatus::Ok;
    case kUhdRxErrorTimeout:
        return echo::BurstStatus::Timeout;
    case kUhdRxErrorOverflow:
        return echo::BurstStatus::Overflow;
    case kUhdRxErrorLateCommand:
        return echo::BurstStatus::LateCommand;
    case kUhdRxErrorBrokenChain:
        return echo::BurstStatus::BrokenChain;
    default:
        // alignment / bad_packet / anything unknown: transport-level
        // failure needing intervention.
        return echo::BurstStatus::BackendError;
    }
}

// Map one drained TX async metadata event (async_metadata_t::event_code,
// bitmask) to a BurstStatus.  Multiple bits can be set in one event; the
// most severe mapping wins.  BURST_ACK / no bits are normal (Ok); any
// non-ack event is appended to `note` for the per-burst uhd_error string.
inline echo::BurstStatus map_uhd_tx_async_event(uint32_t event_code,
                                                std::string& note)
{
    if (event_code == 0 || event_code == kUhdAsyncBurstAck)
        return echo::BurstStatus::Ok;

    echo::BurstStatus worst = echo::BurstStatus::Ok;
    auto bump = [&](echo::BurstStatus s, const char* what) {
        note += note.empty() ? what : std::string(", ") + what;
        // Severity: LateCommand beats transport errors; any event keeps
        // the burst non-ok.
        if (s == echo::BurstStatus::LateCommand)
            worst = s;
        else if (worst == echo::BurstStatus::Ok)
            worst = s;
    };
    if (event_code & kUhdAsyncTimeError)
        bump(echo::BurstStatus::LateCommand, "tx_time_error");
    if (event_code & (kUhdAsyncUnderflow | kUhdAsyncUnderflowInPacket))
        bump(echo::BurstStatus::BackendError, "tx_underflow");
    if (event_code & (kUhdAsyncSeqError | kUhdAsyncSeqErrorInBurst))
        bump(echo::BurstStatus::BackendError, "tx_seq_error");
    // USER_PAYLOAD (0x40) is informational; unknown bits above it are
    // surfaced rather than silently dropped.
    if (event_code & ~uint32_t(0x7F))
        bump(echo::BurstStatus::BackendError, "tx_unknown_event");
    return worst;
}

// The prepare()-failure message for a missing/misconfigured device.  The
// exact prefix is a contract: it is how the operator (and QA) distinguish
// "no USRP present / unreachable" from any other backend failure, and the
// block surfaces it verbatim in the backend_prepare_failed status event.
inline std::string make_device_unavailable_error(const std::string& what)
{
    return "device unavailable: " + what;
}

// ---------------------------------------------------------------------------
// Planned-burst fragment flag contract (shared by the UhdBurstBackend; the
// FakeBurstBackend keeps its own private copy of the same rules)
// ---------------------------------------------------------------------------

// The FIRST fragment of a burst carries kFlagTimeSpec + kFlagStartOfBurst,
// the LAST fragment carries kFlagEndOfBurst, intermediates carry neither,
// and every fragment is non-empty.  Violations are broken chains.
inline bool validate_fragment_flags(const echo::BurstFragment* frags,
                                    size_t count,
                                    std::string* error = nullptr)
{
    auto fail = [&](const char* what) {
        if (error)
            *error = what;
        return false;
    };
    if (frags == nullptr || count == 0 ||
        count > echo::kEchoMaxFragmentsPerBurst)
        return fail("empty or oversized fragment list");
    for (size_t i = 0; i < count; ++i) {
        const unsigned f = frags[i].flags;
        if (frags[i].count == 0)
            return fail("fragment with zero samples");
        if (i == 0) {
            if (!(f & echo::kFlagTimeSpec) || !(f & echo::kFlagStartOfBurst))
                return fail("first fragment lacks time spec + SOB");
        } else if (f & (echo::kFlagTimeSpec | echo::kFlagStartOfBurst)) {
            return fail("SOB / time spec outside the first fragment");
        }
        if (i + 1 == count) {
            if (!(f & echo::kFlagEndOfBurst))
                return fail("last fragment lacks EOB");
        } else if (f & echo::kFlagEndOfBurst) {
            return fail("EOB outside the last fragment");
        }
    }
    return true;
}

// ---------------------------------------------------------------------------
// Strict sample-rate readback policy
// ---------------------------------------------------------------------------

// The required native rate is 737.28 MS/s exactly (double-representable);
// a get_*_rate() readback that differs by more than rel_tol is a hard
// prepare() failure — silent UHD coercion is rejected, never accepted.
inline constexpr double kUhdRequiredRateHz = 737280000.0;
inline constexpr double kUhdDefaultRateTolRel = 1e-9;

inline bool rate_matches_strict(double requested, double readback,
                                double rel_tol)
{
    if (!(rel_tol > 0.0))
        return false;
    const double scale = requested > readback ? requested : readback;
    return std::fabs(readback - requested) <= rel_tol * scale;
}

// ---------------------------------------------------------------------------
// Device-time tick <-> (full, frac) conversions
// ---------------------------------------------------------------------------

// Device ticks → UHD time parts: full = floor(ticks / rate),
// frac in [0, 1) with full double precision on the fraction (sub-tick
// error ~1e-16 s at 737.28 MS/s).  Deterministic round trip with
// ticks_from_time_parts.
inline bool time_parts_from_ticks(int64_t ticks, double rate_hz,
                                  int64_t& full, double& frac)
{
    if (!(rate_hz > 0.0) || ticks < 0)
        return false;
    const double secs = static_cast<double>(ticks) / rate_hz;
    if (!(secs < 9.0e18))
        return false; // full seconds would not fit int64
    full = static_cast<int64_t>(std::floor(secs));
    double rem_ticks = static_cast<double>(ticks) -
                       static_cast<double>(full) * rate_hz;
    if (rem_ticks < 0.0)
        rem_ticks = 0.0;
    if (rem_ticks >= rate_hz)
        rem_ticks = rate_hz - 1.0; // clamp double rounding into [0, rate)
    frac = rem_ticks / rate_hz;
    return true;
}

// UHD time parts → device ticks: llround(full * rate) + llround(frac * rate),
// checked against the int64 range (documented accuracy: ±~64 ticks at
// GPS-scale full seconds because of double spacing; irrelevant for the
// PRI grid — the geometry itself stays in exact integer tick space).
inline bool ticks_from_time_parts(int64_t full, double frac, double rate_hz,
                                  int64_t& ticks)
{
    if (!(rate_hz > 0.0) || full < 0)
        return false;
    if (!(frac >= 0.0 && frac < 1.0))
        return false;
    const double whole_d = std::floor(static_cast<double>(full) * rate_hz);
    if (!(whole_d <= 9.0e18))
        return false;
    const int64_t whole = static_cast<int64_t>(whole_d);
    const int64_t frac_ticks = static_cast<int64_t>(
        std::llround(frac * rate_hz));
    return radar::radar_i64_add(whole, frac_ticks, ticks);
}

// ---------------------------------------------------------------------------
// Backend configuration (validated once by validate_uhd_burst_backend_config;
// the UHD-linked implementation re-freezes it in prepare())
// ---------------------------------------------------------------------------

struct UhdBurstBackendConfig {
    // Device args for multi_usrp::make (e.g. "addr=192.168.10.2");
    // empty = UHD default discovery.
    std::string device_args;
    // Required radio rate for BOTH directions, read back strictly.
    double sample_rate_hz = kUhdRequiredRateHz;
    double rate_tolerance_rel = kUhdDefaultRateTolRel;
    // Channels inside the SAME multi_usrp device (SC16 streamers).
    size_t tx_channel = 0;
    size_t rx_channel = 1;
    // Optional clock/time source ("internal" for single-device radar).
    // Empty = leave the device configuration untouched.
    std::string clock_source;
    std::string time_source;
    // Optional antennas ("TX/RX", "RX2", ...); empty = leave untouched.
    std::string tx_antenna;
    std::string rx_antenna;
    // Optional center frequency; 0 = leave untouched.
    double center_freq_hz = 0.0;
    // Optional gains; NEGATIVE = leave untouched (an explicit 0 dB is a
    // real setting and must not collide with the sentinel).
    double tx_gain_db = -1.0;
    double rx_gain_db = -1.0;
    // Per-call UHD buffer timeouts (seconds): send() waits for device
    // buffer space, recv() waits for one packet; burst-level bounds are
    // the EchoTimer's rx_collect_wait_ms.
    double send_timeout_s = 0.1;
    double recv_timeout_s = 0.1;
};

inline bool validate_uhd_burst_backend_config(
    const UhdBurstBackendConfig& cfg, std::string* error = nullptr)
{
    auto fail = [&](const char* what) {
        if (error)
            *error = what;
        return false;
    };
    if (!(cfg.sample_rate_hz > 0.0) || !std::isfinite(cfg.sample_rate_hz))
        return fail("sample_rate_hz must be > 0 and finite");
    if (!(cfg.rate_tolerance_rel > 0.0) ||
        !(cfg.rate_tolerance_rel < 1e-3) ||
        !std::isfinite(cfg.rate_tolerance_rel))
        return fail("rate_tolerance_rel must be in (0, 1e-3)");
    if (!(cfg.send_timeout_s > 0.0) || !(cfg.send_timeout_s <= 10.0) ||
        !std::isfinite(cfg.send_timeout_s))
        return fail("send_timeout_s must be in (0, 10]");
    if (!(cfg.recv_timeout_s > 0.0) || !(cfg.recv_timeout_s <= 10.0) ||
        !std::isfinite(cfg.recv_timeout_s))
        return fail("recv_timeout_s must be in (0, 10]");
    if (!std::isfinite(cfg.center_freq_hz) || cfg.center_freq_hz < 0.0)
        return fail("center_freq_hz must be >= 0 and finite");
    if (!std::isfinite(cfg.tx_gain_db))
        return fail("tx_gain_db must be finite (negative = not set)");
    if (!std::isfinite(cfg.rx_gain_db))
        return fail("rx_gain_db must be finite (negative = not set)");
    if (cfg.tx_gain_db >= 0.0 && cfg.tx_gain_db > 120.0)
        return fail("tx_gain_db must be <= 120 when set");
    if (cfg.rx_gain_db >= 0.0 && cfg.rx_gain_db > 120.0)
        return fail("rx_gain_db must be <= 120 when set");
    return true;
}

// ---------------------------------------------------------------------------
// Dry-run plan (no device: pure contract, printable and QA-able)
// ---------------------------------------------------------------------------

struct UhdDryRunPlan {
    // Echoed configuration.
    std::string device_args;
    double sample_rate_hz = 0.0;
    double rate_tolerance_rel = 0.0;
    size_t tx_channel = 0;
    size_t rx_channel = 0;
    std::string clock_source;
    std::string time_source;
    std::string tx_antenna;
    std::string rx_antenna;
    double center_freq_hz = 0.0;
    double tx_gain_db = 0.0;
    double rx_gain_db = 0.0;
    double send_timeout_s = 0.0;
    double recv_timeout_s = 0.0;

    // Frozen scheduler contract.
    int64_t pri_num = 0;
    int64_t pri_den = 0;
    int64_t pre_guard_ticks = 0;
    uint64_t max_fragment_size = 0;

    // Burst geometry (first grid slot of the planned schedule).
    uint64_t schedule_index = 0;
    int64_t t0_ticks = 0;
    uint64_t tx_samples = 0;
    uint64_t rx_samples = 0;
    int64_t tx_ticks = 0; // = t0_ticks
    int64_t rx_ticks = 0; // = t0_ticks - pre_guard (BEFORE tx)

    // Derived display values.
    double pri_s = 0.0;
    double pre_guard_s = 0.0;
    double tx_window_us = 0.0;
    double rx_window_us = 0.0;
    uint64_t tx_bytes = 0; // SC16: 4 bytes per sample pair
    uint64_t rx_bytes = 0;
    uint64_t tx_fragments = 0;
    uint64_t rx_fragments = 0;
};

// Build the dry-run plan from a validated config + prepared grid + burst
// geometry.  Pure integer/derived math on the SAME core the real backend
// is driven by (echo::plan_fragments), so the printed contract is the
// executed contract.  Fails on invalid config/geometry.
inline bool build_uhd_dry_run_plan(const UhdBurstBackendConfig& cfg,
                                   const echo::EchoSchedulerPrepared& grid,
                                   int64_t t0_ticks,
                                   uint64_t schedule_index,
                                   uint64_t tx_samples,
                                   uint64_t rx_samples,
                                   UhdDryRunPlan& plan,
                                   std::string* error = nullptr)
{
    auto fail = [&](const char* what) {
        if (error)
            *error = what;
        return false;
    };
    plan = UhdDryRunPlan{};
    if (!validate_uhd_burst_backend_config(cfg, error))
        return false;

    // Burst geometry: lengths positive and fragmentable by the shared
    // planner (fixed fragment arrays — the same limit the live path has).
    if (tx_samples == 0 || rx_samples == 0)
        return fail("tx_samples and rx_samples must be > 0");
    echo::EchoFragmentSpan spans[echo::kEchoMaxFragmentsPerBurst];
    size_t ntx = 0;
    size_t nrx = 0;
    if (!echo::plan_fragments(tx_samples, grid.max_fragment_size, spans,
                              echo::kEchoMaxFragmentsPerBurst, ntx) ||
        !echo::plan_fragments(rx_samples, grid.max_fragment_size, spans,
                              echo::kEchoMaxFragmentsPerBurst, nrx))
        return fail("burst geometry needs too many fragments");

    int64_t rx_ticks = 0;
    if (!radar::radar_i64_sub(t0_ticks, grid.pre_guard_ticks, rx_ticks) ||
        rx_ticks < 0)
        return fail("t0_ticks must leave room for pre_guard (rx_ticks >= 0)");

    // Echo configuration.
    plan.device_args = cfg.device_args;
    plan.sample_rate_hz = cfg.sample_rate_hz;
    plan.rate_tolerance_rel = cfg.rate_tolerance_rel;
    plan.tx_channel = cfg.tx_channel;
    plan.rx_channel = cfg.rx_channel;
    plan.clock_source = cfg.clock_source;
    plan.time_source = cfg.time_source;
    plan.tx_antenna = cfg.tx_antenna;
    plan.rx_antenna = cfg.rx_antenna;
    plan.center_freq_hz = cfg.center_freq_hz;
    plan.tx_gain_db = cfg.tx_gain_db;
    plan.rx_gain_db = cfg.rx_gain_db;
    plan.send_timeout_s = cfg.send_timeout_s;
    plan.recv_timeout_s = cfg.recv_timeout_s;

    // Scheduler contract + geometry.
    plan.pri_num = grid.pri_num;
    plan.pri_den = grid.pri_den;
    plan.pre_guard_ticks = grid.pre_guard_ticks;
    plan.max_fragment_size = grid.max_fragment_size;
    plan.schedule_index = schedule_index;
    plan.t0_ticks = t0_ticks;
    plan.tx_samples = tx_samples;
    plan.rx_samples = rx_samples;
    plan.tx_ticks = t0_ticks;
    plan.rx_ticks = rx_ticks;

    // Derived display values (double precision, display only — all
    // scheduling stays in exact integer tick space).
    plan.pri_s = static_cast<double>(grid.pri_num) /
                 (static_cast<double>(grid.pri_den) * cfg.sample_rate_hz);
    plan.pre_guard_s = static_cast<double>(grid.pre_guard_ticks) /
                       cfg.sample_rate_hz;
    plan.tx_window_us = static_cast<double>(tx_samples) /
                        cfg.sample_rate_hz * 1e6;
    plan.rx_window_us = static_cast<double>(rx_samples) /
                        cfg.sample_rate_hz * 1e6;
    plan.tx_bytes = tx_samples * 4;
    plan.rx_bytes = rx_samples * 4;
    plan.tx_fragments = static_cast<uint64_t>(ntx);
    plan.rx_fragments = static_cast<uint64_t>(nrx);
    return true;
}

inline std::string print_uhd_dry_run_plan(const UhdDryRunPlan& p)
{
    // Full-precision doubles (exact rate line; no silent rounding of the
    // printed contract).
    auto d = [](double v) {
        std::ostringstream s;
        s << std::setprecision(17) << v;
        return s.str();
    };
    std::ostringstream os;
    os << "uwb_uhd_burst_backend DRY-RUN (no device opened)\n";
    os << "[device]\n";
    os << "device_args              = " << p.device_args << "\n";
    os << "clock_source             = "
       << (p.clock_source.empty() ? "(untouched)" : p.clock_source) << "\n";
    os << "time_source              = "
       << (p.time_source.empty() ? "(untouched)" : p.time_source) << "\n";
    os << "tx_channel               = " << p.tx_channel << "\n";
    os << "rx_channel               = " << p.rx_channel << "\n";
    os << "tx_antenna               = "
       << (p.tx_antenna.empty() ? "(untouched)" : p.tx_antenna) << "\n";
    os << "rx_antenna               = "
       << (p.rx_antenna.empty() ? "(untouched)" : p.rx_antenna) << "\n";
    os << "center_freq_hz           = "
       << (p.center_freq_hz > 0.0 ? d(p.center_freq_hz)
                                  : std::string("(untouched)"))
       << "\n";
    os << "tx_gain_db               = "
       << (p.tx_gain_db >= 0.0 ? d(p.tx_gain_db)
                               : std::string("(untouched)"))
       << "\n";
    os << "rx_gain_db               = "
       << (p.rx_gain_db >= 0.0 ? d(p.rx_gain_db)
                               : std::string("(untouched)"))
       << "\n";
    os << "[streamers]\n";
    os << "otw_format               = sc16\n";
    os << "cpu_format               = s16\n";
    os << "sample_rate_hz           = " << d(p.sample_rate_hz) << "\n";
    os << "rate_readback            = strict rel_tol=" << d(p.rate_tolerance_rel)
       << " (silent coercion rejected)\n";
    os << "[scheduler contract]\n";
    os << "pri_ticks                = " << p.pri_num << "/" << p.pri_den
       << "\n";
    os << "pri_s                    = " << d(p.pri_s) << "\n";
    os << "pre_guard_ticks          = " << p.pre_guard_ticks << " ("
       << d(p.pre_guard_s) << " s)\n";
    os << "[burst geometry]\n";
    os << "schedule_index           = " << p.schedule_index << "\n";
    os << "t0_ticks                 = " << p.t0_ticks << "\n";
    os << "tx_ticks / rx_ticks      = " << p.tx_ticks << " / "
       << p.rx_ticks << " (rx BEFORE tx)\n";
    os << "tx_samples / rx_samples  = " << p.tx_samples << " / "
       << p.rx_samples << "\n";
    os << "tx_window_us / rx_window_us = " << d(p.tx_window_us) << " / "
       << d(p.rx_window_us) << "\n";
    os << "tx_fragments / rx_fragments = " << p.tx_fragments << " / "
       << p.rx_fragments << " (max_fragment_size=" << p.max_fragment_size
       << ")\n";
    os << "tx_bytes / rx_bytes      = " << p.tx_bytes << " / "
       << p.rx_bytes << " (sc16 = 4 B/sample)\n";
    os << "[io contract]\n";
    os << "command_order            = issue_rx(NUM_SAMPS_AND_DONE, "
          "timed) BEFORE issue_tx(send loop)\n";
    os << "tx_metadata              = SOB+time_spec on first send call; "
          "EOB on a dedicated zero-length call after all data\n";
    os << "rx_stream_mode           = NUM_SAMPS_AND_DONE, stream_now=false, "
          "time_spec=rx_ticks\n";
    os << "rx_error_map             = 0x0:none 0x1:timeout 0x2:late_command "
          "0x4:broken_chain 0x8:overflow else:backend_error\n";
    os << "tx_async_map             = time_error:late_command "
          "underflow/seq_error:backend_error burst_ack:normal\n";
    os << "send/recv_timeout_s      = " << d(p.send_timeout_s) << " / "
       << d(p.recv_timeout_s) << " (per call; partial re-issued)\n";
    os << "stop_safety              = request_stop wakes collect_result; "
          "abort_rx issues STOP_CONTINUOUS\n";
    os << "dry-run complete: no device was opened\n";
    return os.str();
}

} // namespace uhd
} // namespace uwb
} // namespace gr

#endif /* INCLUDED_GNURADIO_UWB_UWB_UHD_BACKEND_CONFIG_H */
