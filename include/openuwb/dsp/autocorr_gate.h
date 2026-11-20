/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Pure gating logic for "pause the autocorrelation scan while the target is
 * locked" (docs/performance/开发规划_无干扰锁定后暂停自相关_2026-11-20.md §3).
 *
 * Header-only and UHD-free so lib/ QA can include it directly.  The live app
 * calls update() once per RX block, after the block is copied into the ring
 * and before the acquisition scan:
 *
 *   - enabled == false      : the historical behaviour, always scan.
 *   - enabled && scheduled  : the target is tracked (Verify/Locked/Holdover);
 *                             skip the scan and remember how much was skipped.
 *   - enabled && !scheduled : Acquire/Reacquire, scan again.  The first block
 *                             after a suspension reports reset=true so the
 *                             caller re-seeds the probe at reset_at and
 *                             catches up on the backlog still in the ring.
 *
 * Threading: single-threaded, owned by the RX/scheduler thread.
 */

#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>

namespace openuwb::live {

// CLI values of --autocorr-when-locked.
enum class AutocorrWhenLocked { Scan, Suspend, Auto };

// Parse exactly "scan", "suspend" or "auto"; anything else is a configuration
// error and throws std::invalid_argument (fail-closed, no silent default).
inline AutocorrWhenLocked parse_autocorr_when_locked(const std::string& value)
{
    if (value == "scan") {
        return AutocorrWhenLocked::Scan;
    }
    if (value == "suspend") {
        return AutocorrWhenLocked::Suspend;
    }
    if (value == "auto") {
        return AutocorrWhenLocked::Auto;
    }
    throw std::invalid_argument("invalid --autocorr-when-locked value: '" + value +
                                "' (expected scan|suspend|auto)");
}

// Resolve a mode into "suspend while tracked?".
//
//   Scan                    -> false
//   Auto                    -> !interference
//   Suspend + interference  -> throw: suspending the scan would silently
//                              disable interference monitoring
//   Suspend + no interference -> true
inline bool resolve_autocorr_suspend(AutocorrWhenLocked mode, bool interference)
{
    switch (mode) {
    case AutocorrWhenLocked::Scan:
        return false;
    case AutocorrWhenLocked::Auto:
        return !interference;
    case AutocorrWhenLocked::Suspend:
        if (interference) {
            throw std::invalid_argument(
                "--autocorr-when-locked suspend requires --interference off");
        }
        return true;
    }
    throw std::invalid_argument("unhandled --autocorr-when-locked value");
}

struct AutocorrGate {
    // Resolved suspend mode (see resolve_autocorr_suspend()).
    bool enabled = false;
    // True between entering the tracked state and the next !scheduled block.
    bool suspended = false;
    // Suspend/resume transition counts.
    uint64_t suspend_events = 0;
    uint64_t resume_events = 0;
    // Native samples whose scan was skipped while suspended.
    uint64_t suspended_samples = 0;
    // Samples re-scanned from the ring at each resume (end - reset_at).
    uint64_t resume_backlog_samples = 0;

    // Per-block decision for the caller.
    struct Action {
        bool scan = false;      // false: skip acq.scan for this block
        bool reset = false;     // true: acq.reset(reset_at) before scanning
        int64_t reset_at = 0;   // probe restart position (end unless reset)
    };

    // Call once per RX block with n = number of newly copied samples.
    //
    //   !enabled                 -> {scan=true,  reset=false, reset_at=end}
    //   enabled &&  scheduled    -> {scan=false, reset=false, reset_at=end},
    //                               ++suspend_events on entry only,
    //                               suspended_samples += n
    //   enabled && !scheduled && suspended
    //                            -> {scan=true,  reset=true,  reset_at=r},
    //                               r = max(0, valid_begin, end - max(0, lookback)),
    //                               ++resume_events,
    //                               resume_backlog_samples += end - r
    //   enabled && !scheduled && !suspended
    //                            -> {scan=true,  reset=false, reset_at=end}
    Action update(bool scheduled, int64_t end, int64_t valid_begin,
                  int64_t lookback, size_t n)
    {
        if (!enabled) {
            // Historical behaviour: always scan, never touch the probe.
            suspended = false;
            return Action{ true, false, end };
        }

        if (scheduled) {
            if (!suspended) {
                suspended = true;
                ++suspend_events;
            }
            suspended_samples += static_cast<uint64_t>(n);
            return Action{ false, false, end };
        }

        if (suspended) {
            suspended = false;
            ++resume_events;
            // end is a non-negative absolute sample index, so even a
            // caller-supplied LLONG_MAX lookback cannot underflow here
            // (end - INT64_MAX > INT64_MIN for end >= 0).
            const int64_t lookback_samples = std::max<int64_t>(0, lookback);
            // The live caller always has valid_begin >= 0; clamp defensively
            // anyway because Acquisition::scan casts its probe to size_t.
            const int64_t resume_at = std::max<int64_t>(
                0, std::max(valid_begin, end - lookback_samples));
            // valid_begin is contractually <= end; clamp defensively so an
            // inconsistent caller cannot wrap the unsigned backlog counter.
            resume_backlog_samples +=
                static_cast<uint64_t>(std::max<int64_t>(0, end - resume_at));
            return Action{ true, true, resume_at };
        }

        // Not tracked, not resuming: keep scanning from the current probe.
        return Action{ true, false, end };
    }
};

} // namespace openuwb::live
