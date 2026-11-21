/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * GNU Radio-independent device-timed RX plan for the native X410 live receiver.
 *
 * Design contract (docs/performance/X410_目标周期锁定与随机干扰双路径修改规划_2026-11-15.md §7/§8):
 *
 *   - Exactly one RX owner issues stream commands and calls recv.  DSP workers
 *     never touch the USRP.
 *   - Windows are queued in the future with an explicit device time and a
 *     finite sample count (STREAM_MODE_NUM_SAMPS_AND_DONE semantics).  Host
 *     timers are never used as a substitute for device-timed capture.
 *   - The lookahead is bounded.  A window already issued to the device is
 *     immutable: late feedback may only change windows not yet issued.
 *   - Start/length are expanded outward to the hardware alignment (SPC)
 *     multiple, and the EXPANDED start is what the demod coordinate mapping
 *     uses.
 *   - recv may return in several pieces; they are assembled by device time and
 *     sample count.  Partial recv, timeout, late command, overflow, device time
 *     jump and complete windows are distinct terminal states.
 *   - A gap between planned windows is `planned_gap`, not a UHD error and not
 *     fabricated continuous samples.  Missing samples inside a planned window
 *     are `unexpected_gap` and invalidate detection statistics / the target
 *     lock.
 *   - "Fewer samples uploaded" is never described as the ADC/RF stopping; only
 *     the host-visible data window and host load change.
 *
 * Coordinates: absolute 64-bit device sample indices at `sample_rate`.  The
 * device sample index is derived from the UHD time spec, never from a running
 * "how many samples did I receive" counter, so a planned stop does not shift
 * the timeline.
 *
 * This header keeps the enums, the configuration/spec/statistics structs,
 * PhysicalRxInterval and the trivial accessors.  The window expansion, the
 * request/issue queue, the chunk assembly state machine and the tone/target
 * interval merge are defined in lib/runtime/uwb_timed_rx_plan.cc (target
 * openuwb_runtime).
 */

#pragma once

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdint>
#include <deque>
#include <limits>
#include <mutex>
#include <vector>

namespace openuwb {
namespace core {

// ---------------------------------------------------------------------------
// Terminal states
// ---------------------------------------------------------------------------
enum class TimedRxTerminal : int {
    Complete = 0,      // full planned window received
    Partial = 1,       // burst ended early / short read at EOS
    Timeout = 2,       // recv timed out
    LateCommand = 3,   // device rejected the command time
    Overflow = 4,      // device overflow
    DeviceTimeJump = 5,// time spec went backwards or jumped
    PlannedGap = 6,    // gap between two planned windows (not an error)
    UnexpectedGap = 7, // missing samples inside a planned window
    Cancelled = 8      // dropped before issue (queue full / superseded)
};

const char* timed_rx_terminal_name(TimedRxTerminal t);

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------
struct TimedRxPlanConfig {
    double sample_rate = 737280000.0;
    // Hardware samples-per-clock / alignment granularity.  Start and length are
    // expanded outward to this multiple.
    size_t alignment = 4;
    // Maximum number of windows issued but not yet assembled.
    size_t max_lookahead = 4;
    // Minimum lead between command issue and the window start.
    double command_lead_s = 200e-6;
    // Device per-command sample cap (0 = unlimited).
    size_t max_samples_per_command = 0;
    // Bounded request queue (scheduler -> issuer).
    size_t request_capacity = 32;
};

// ---------------------------------------------------------------------------
// Window specification
// ---------------------------------------------------------------------------
struct TimedRxWindowSpec {
    uint64_t window_id = 0;
    uint64_t generation = 0;
    // Logical (unexpanded) request.
    int64_t logical_start = 0;
    size_t logical_samples = 0;
    // Expanded, hardware-aligned request; this is the demod coordinate origin.
    int64_t device_start = 0;
    size_t device_samples = 0;
    double device_time_s = 0.0; // device_start / sample_rate
};

struct TimedRxRequest {
    int64_t logical_start = 0;
    size_t logical_samples = 0;
    uint64_t generation = 0;
};

struct TimedRxChunk {
    int64_t device_start = 0;
    size_t count = 0;
    // Samples for this chunk (count entries).  May be null only for
    // error/timeout chunks with count == 0.
    const std::complex<int16_t>* data = nullptr;
    bool has_time_spec = false;
    double time_spec_s = 0.0;
    int error_code = 0; // 0 = none; caller maps UHD codes
    bool overflow = false;
    bool late_command = false;
    bool timeout = false;
};

struct TimedRxAssembled {
    TimedRxWindowSpec spec;
    TimedRxTerminal terminal = TimedRxTerminal::Complete;
    // Samples actually assembled (may be shorter than spec.device_samples).
    std::vector<std::complex<int16_t>> samples;
    size_t received = 0;
    // First missing sample offset inside the window, or -1.
    int64_t first_missing = -1;
    // Gap between the previous window end and this window start.
    int64_t planned_gap_samples = 0;
    uint64_t issue_delay_us = 0;
};

struct TimedRxPlanStats {
    uint64_t requested = 0;
    uint64_t issued = 0;
    uint64_t completed = 0;
    uint64_t partial = 0;
    uint64_t timeouts = 0;
    uint64_t late_commands = 0;
    uint64_t overflows = 0;
    uint64_t device_time_jumps = 0;
    uint64_t planned_gaps = 0;
    uint64_t unexpected_gaps = 0;
    uint64_t cancelled = 0;
    uint64_t request_queue_full = 0;
    uint64_t lookahead_blocked = 0;
    uint64_t alignment_expanded_samples = 0;
    int64_t planned_gap_samples = 0;
    int64_t unexpected_gap_samples = 0;
};

// ---------------------------------------------------------------------------
// Plan core
// ---------------------------------------------------------------------------
/**
 * Scheduler-side timed RX plan.
 *
 * Typical loop (single RX thread):
 *   queue_request(...)            // from the target slot scheduler
 *   while (next_command(now_s, spec)) issue_stream_cmd(spec)   // device
 *   chunk = recv(...); on_chunk(chunk)
 *   while (take_assembled(w)) deliver(w)
 */
class TimedRxPlanCore {
public:
    explicit TimedRxPlanCore(
        const TimedRxPlanConfig& cfg = TimedRxPlanConfig());

    void configure(const TimedRxPlanConfig& cfg);

    const TimedRxPlanConfig& config() const { return cfg_; }

    void reset();

    TimedRxPlanStats stats() const { return stats_; }
    size_t outstanding() const { return outstanding_.size(); }
    size_t queued_requests() const { return requests_.size(); }

    TimedRxWindowSpec expand(const TimedRxRequest& r,
                            uint64_t window_id) const;

    /**
     * Queue a future logical window.  Returns false (counted) when the bounded
     * request queue is full, so a target slot is never silently lost.
     */
    bool queue_request(const TimedRxRequest& r);

    /**
     * Pop the next request that can be issued now.
     *
     * `device_now_s` is the current device time.  A request is issuable when
     * its expanded start is at least `command_lead_s` in the future and the
     * lookahead is not exhausted.  Requests whose window has already passed are
     * dropped as Cancelled (counted) rather than issued late.
     */
    bool next_command(double device_now_s, TimedRxWindowSpec& out);

    /**
     * Feed one recv result.  Chunks may be partial and out of order; they are
     * matched to the outstanding window that contains them.
     */
    void on_chunk(const TimedRxChunk& chunk);

    /**
     * End of stream / explicit burst end for the oldest outstanding window.
     * A window that is not full becomes Partial.
     */
    void flush_partial();

    bool take_assembled(TimedRxAssembled& out);

    size_t assembled_ready() const { return assembled_.size(); }

private:
    struct IssuedWindow {
        TimedRxWindowSpec spec;
        std::vector<std::complex<int16_t>> samples;
        size_t filled = 0;
        int64_t next_expected = 0;
        int64_t first_missing = -1;
        bool gap = false;
    };

    void sanitize();

    void push_assembled(TimedRxAssembled&& a);

    void finish(IssuedWindow& iw, TimedRxTerminal terminal);

    void fail_window_at(int64_t device_start, TimedRxTerminal terminal);

    void fail_all(TimedRxTerminal terminal);

    TimedRxPlanConfig cfg_{};
    std::deque<TimedRxRequest> requests_;
    std::deque<IssuedWindow> outstanding_;
    std::deque<TimedRxAssembled> assembled_;
    TimedRxPlanStats stats_{};
    uint64_t next_window_id_ = 1;
    int64_t last_device_end_ = -1;
    double last_time_s_ = -1.0;
};

// ---------------------------------------------------------------------------
// Fixed-tone schedule compatibility (§8)
// ---------------------------------------------------------------------------
/**
 * One physical RX interval that may serve several logical windows.
 *
 * The fixed-tone module needs a 0.5 ms training window every 20 ms anchored to
 * device time.  When a target window overlaps a training window the physical
 * capture is merged once and each logical window references its slice, so the
 * tone estimate is never computed twice and the same samples are never
 * subtracted twice.
 */
struct PhysicalRxInterval {
    int64_t start = 0;
    size_t samples = 0;
    bool has_tone_training = false;
    bool has_target = false;
    // Logical slices inside this interval (relative offsets).
    struct Slice {
        int64_t offset = 0;
        size_t samples = 0;
        bool tone_training = false;
        bool target = false;
        uint64_t slot_index = 0;
    };
    std::vector<Slice> slices;

    int64_t end() const { return start + static_cast<int64_t>(samples); }
};

/**
 * Merge tone-training and target windows into physical RX intervals.
 *
 * `tone_period_samples` / `tone_window_samples` describe the periodic training
 * grid; `targets` are (start, samples, slot) triples.  Overlapping windows are
 * merged and keep one slice per logical purpose.  Intervals are returned sorted
 * by start and never overlap.
 */
std::vector<PhysicalRxInterval> merge_physical_rx_intervals(
    const std::vector<std::pair<int64_t, size_t>>& targets,
    const std::vector<uint64_t>& target_slots,
    int64_t tone_origin,
    int64_t tone_period_samples,
    size_t tone_window_samples,
    int64_t range_start,
    int64_t range_end);

} // namespace core
} // namespace openuwb
