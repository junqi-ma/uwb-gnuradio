/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Out-of-line implementations of the device-timed RX plan
 * (declarations in include/openuwb/runtime/timed_rx_plan.h).
 *
 * Moved bodies: the terminal-state name table, the hardware-alignment
 * expansion, the bounded request queue / issuable-command selection, the
 * out-of-order chunk assembly and its terminal states, plus the
 * tone-training/target window merge used by the fixed-tone schedule.
 *
 * The header keeps the enums, structs, PhysicalRxInterval and the trivial
 * accessors (config, stats, outstanding, queued_requests, assembled_ready).
 */

#include <openuwb/runtime/timed_rx_plan.h>

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdint>
#include <utility>
#include <vector>

namespace openuwb {
namespace core {

const char* timed_rx_terminal_name(TimedRxTerminal t)
{
    switch (t) {
    case TimedRxTerminal::Complete:
        return "complete";
    case TimedRxTerminal::Partial:
        return "partial";
    case TimedRxTerminal::Timeout:
        return "timeout";
    case TimedRxTerminal::LateCommand:
        return "late_command";
    case TimedRxTerminal::Overflow:
        return "overflow";
    case TimedRxTerminal::DeviceTimeJump:
        return "device_time_jump";
    case TimedRxTerminal::PlannedGap:
        return "planned_gap";
    case TimedRxTerminal::UnexpectedGap:
        return "unexpected_gap";
    case TimedRxTerminal::Cancelled:
        return "cancelled";
    }
    return "unknown";
}

TimedRxPlanCore::TimedRxPlanCore(const TimedRxPlanConfig& cfg)
    : cfg_(cfg)
{
    sanitize();
}

void TimedRxPlanCore::configure(const TimedRxPlanConfig& cfg)
{
    cfg_ = cfg;
    sanitize();
    reset();
}

void TimedRxPlanCore::reset()
{
    requests_.clear();
    outstanding_.clear();
    assembled_.clear();
    stats_ = TimedRxPlanStats{};
    next_window_id_ = 1;
    last_device_end_ = -1;
    last_time_s_ = -1.0;
}

/** Expand a logical request to the hardware-aligned device window. */
TimedRxWindowSpec TimedRxPlanCore::expand(const TimedRxRequest& r,
                                          uint64_t window_id) const
{
    TimedRxWindowSpec s;
    s.window_id = window_id;
    s.generation = r.generation;
    s.logical_start = r.logical_start;
    s.logical_samples = r.logical_samples;
    const int64_t a = static_cast<int64_t>(cfg_.alignment);
    int64_t start = r.logical_start;
    int64_t end = r.logical_start + static_cast<int64_t>(r.logical_samples);
    if (a > 1) {
        start = (start >= 0) ? (start / a) * a
                             : -(((-start) + a - 1) / a) * a;
        end = ((end + a - 1) / a) * a;
    }
    size_t n = static_cast<size_t>(std::max<int64_t>(0, end - start));
    if (cfg_.max_samples_per_command && n > cfg_.max_samples_per_command)
        n = cfg_.max_samples_per_command;
    s.device_start = start;
    s.device_samples = n;
    s.device_time_s =
        cfg_.sample_rate > 0.0
            ? static_cast<double>(start) / cfg_.sample_rate
            : 0.0;
    return s;
}

bool TimedRxPlanCore::queue_request(const TimedRxRequest& r)
{
    if (r.logical_samples == 0)
        return false;
    ++stats_.requested;
    if (requests_.size() >= cfg_.request_capacity) {
        ++stats_.request_queue_full;
        return false;
    }
    requests_.push_back(r);
    return true;
}

bool TimedRxPlanCore::next_command(double device_now_s, TimedRxWindowSpec& out)
{
    while (!requests_.empty()) {
        if (outstanding_.size() >= cfg_.max_lookahead) {
            ++stats_.lookahead_blocked;
            return false;
        }
        TimedRxRequest r = requests_.front();
        TimedRxWindowSpec spec = expand(r, next_window_id_);
        const double start_s = spec.device_time_s;
        if (start_s < device_now_s + cfg_.command_lead_s) {
            // Too late to schedule this window; never issue a late command.
            requests_.pop_front();
            TimedRxAssembled a;
            a.spec = spec;
            a.terminal = TimedRxTerminal::Cancelled;
            a.first_missing = 0;
            push_assembled(std::move(a));
            continue;
        }
        requests_.pop_front();
        ++next_window_id_;
        ++stats_.issued;
        if (spec.device_samples >= r.logical_samples)
            stats_.alignment_expanded_samples +=
                spec.device_samples - r.logical_samples;
        IssuedWindow iw;
        iw.spec = spec;
        iw.next_expected = spec.device_start;
        iw.samples.assign(spec.device_samples, std::complex<int16_t>{ 0, 0 });
        outstanding_.push_back(std::move(iw));
        out = spec;
        return true;
    }
    return false;
}

void TimedRxPlanCore::on_chunk(const TimedRxChunk& chunk)
{
    if (chunk.device_start < 0) {
        fail_all(TimedRxTerminal::DeviceTimeJump);
        return;
    }
    if (chunk.has_time_spec) {
        if (last_time_s_ >= 0.0 &&
            chunk.time_spec_s + 1e-9 < last_time_s_) {
            fail_all(TimedRxTerminal::DeviceTimeJump);
            last_time_s_ = chunk.time_spec_s;
            return;
        }
        last_time_s_ = chunk.time_spec_s;
    }
    if (chunk.overflow) {
        fail_window_at(chunk.device_start, TimedRxTerminal::Overflow);
        return;
    }
    if (chunk.late_command) {
        fail_window_at(chunk.device_start, TimedRxTerminal::LateCommand);
        return;
    }
    if (chunk.timeout) {
        fail_window_at(chunk.device_start, TimedRxTerminal::Timeout);
        return;
    }
    if (chunk.count == 0)
        return;

    // Find the outstanding window containing this chunk start.
    for (auto& iw : outstanding_) {
        const int64_t ws = iw.spec.device_start;
        const int64_t we = ws + static_cast<int64_t>(iw.spec.device_samples);
        if (chunk.device_start + static_cast<int64_t>(chunk.count) <= ws)
            continue;
        if (chunk.device_start >= we)
            continue;
        const int64_t copy_from =
            std::max<int64_t>(chunk.device_start, ws);
        const int64_t copy_to =
            std::min<int64_t>(chunk.device_start +
                                  static_cast<int64_t>(chunk.count),
                              we);
        if (copy_to <= copy_from)
            continue;
        const size_t dst = static_cast<size_t>(copy_from - ws);
        const size_t n = static_cast<size_t>(copy_to - copy_from);
        // Fill the hole in front of this chunk with zeros and record it.
        if (copy_from > iw.next_expected) {
            if (iw.first_missing < 0)
                iw.first_missing = iw.next_expected - ws;
            iw.gap = true;
        }
        if (chunk.data && dst + n <= iw.samples.size()) {
            const size_t src =
                static_cast<size_t>(copy_from - chunk.device_start);
            std::copy_n(chunk.data + src, n, iw.samples.begin() + dst);
        }
        iw.next_expected = std::max(iw.next_expected, copy_to);
        // Contiguous coverage from the window start; overlapping chunks are
        // not counted twice.
        iw.filled = static_cast<size_t>(iw.next_expected - ws);
        if (iw.next_expected >= we)
            finish(iw, iw.gap ? TimedRxTerminal::UnexpectedGap
                              : TimedRxTerminal::Complete);
        return;
    }
    // A chunk that belongs to no outstanding window is an unexpected gap
    // relative to the plan.
    ++stats_.unexpected_gaps;
    stats_.unexpected_gap_samples += static_cast<int64_t>(chunk.count);
}

void TimedRxPlanCore::flush_partial()
{
    if (outstanding_.empty())
        return;
    auto& iw = outstanding_.front();
    finish(iw, iw.filled >= iw.spec.device_samples
                   ? TimedRxTerminal::Complete
                   : TimedRxTerminal::Partial);
}

/** Pop one assembled window.  Returns false when none is ready. */
bool TimedRxPlanCore::take_assembled(TimedRxAssembled& out)
{
    if (assembled_.empty())
        return false;
    out = std::move(assembled_.front());
    assembled_.pop_front();
    return true;
}

void TimedRxPlanCore::sanitize()
{
    if (cfg_.sample_rate <= 0.0)
        cfg_.sample_rate = 737280000.0;
    if (cfg_.alignment == 0)
        cfg_.alignment = 1;
    if (cfg_.max_lookahead == 0)
        cfg_.max_lookahead = 1;
    if (cfg_.command_lead_s < 0.0)
        cfg_.command_lead_s = 0.0;
    if (cfg_.request_capacity == 0)
        cfg_.request_capacity = 1;
}

void TimedRxPlanCore::push_assembled(TimedRxAssembled&& a)
{
    if (a.spec.device_start >= 0) {
        if (last_device_end_ >= 0 && a.spec.device_start > last_device_end_) {
            a.planned_gap_samples = a.spec.device_start - last_device_end_;
            ++stats_.planned_gaps;
            stats_.planned_gap_samples += a.planned_gap_samples;
        }
        last_device_end_ =
            a.spec.device_start +
            static_cast<int64_t>(a.spec.device_samples);
    }
    switch (a.terminal) {
    case TimedRxTerminal::Complete:
        ++stats_.completed;
        break;
    case TimedRxTerminal::Partial:
        ++stats_.partial;
        break;
    case TimedRxTerminal::Timeout:
        ++stats_.timeouts;
        break;
    case TimedRxTerminal::LateCommand:
        ++stats_.late_commands;
        break;
    case TimedRxTerminal::Overflow:
        ++stats_.overflows;
        break;
    case TimedRxTerminal::UnexpectedGap:
        ++stats_.unexpected_gaps;
        if (a.first_missing >= 0)
            stats_.unexpected_gap_samples += a.first_missing;
        break;
    case TimedRxTerminal::DeviceTimeJump:
        ++stats_.device_time_jumps;
        break;
    case TimedRxTerminal::Cancelled:
        ++stats_.cancelled;
        break;
    default:
        break;
    }
    assembled_.push_back(std::move(a));
}

void TimedRxPlanCore::finish(IssuedWindow& iw,
                             TimedRxTerminal terminal)
{
    TimedRxAssembled a;
    a.spec = iw.spec;
    a.terminal = terminal;
    a.received = iw.filled;
    a.first_missing = iw.first_missing;
    a.samples = std::move(iw.samples);
    a.samples.resize(iw.filled);
    for (auto it = outstanding_.begin(); it != outstanding_.end(); ++it) {
        if (it->spec.window_id == iw.spec.window_id) {
            outstanding_.erase(it);
            break;
        }
    }
    push_assembled(std::move(a));
}

void TimedRxPlanCore::fail_window_at(int64_t device_start,
                                     TimedRxTerminal terminal)
{
    for (auto& iw : outstanding_) {
        const int64_t ws = iw.spec.device_start;
        const int64_t we = ws + static_cast<int64_t>(iw.spec.device_samples);
        if (device_start >= ws && device_start < we) {
            finish(iw, terminal);
            return;
        }
    }
    // No matching outstanding window: treat as a global device fault.
    fail_all(terminal);
}

void TimedRxPlanCore::fail_all(TimedRxTerminal terminal)
{
    while (!outstanding_.empty()) {
        auto& iw = outstanding_.front();
        finish(iw, terminal);
    }
}

std::vector<PhysicalRxInterval> merge_physical_rx_intervals(
    const std::vector<std::pair<int64_t, size_t>>& targets,
    const std::vector<uint64_t>& target_slots,
    int64_t tone_origin,
    int64_t tone_period_samples,
    size_t tone_window_samples,
    int64_t range_start,
    int64_t range_end)
{
    struct Raw {
        int64_t start;
        size_t samples;
        bool tone;
        bool target;
        uint64_t slot;
    };
    std::vector<Raw> raw;
    if (tone_period_samples > 0 && tone_window_samples > 0) {
        // First tone window at or after range_start, using independent
        // rounding per index so the grid never drifts.
        const double k0 = std::ceil(
            static_cast<double>(range_start - tone_origin) /
            static_cast<double>(tone_period_samples));
        for (int64_t k = static_cast<int64_t>(std::max(0.0, k0));; ++k) {
            const int64_t s =
                tone_origin + static_cast<int64_t>(k) * tone_period_samples;
            if (s >= range_end)
                break;
            if (s + static_cast<int64_t>(tone_window_samples) <= range_start)
                continue;
            raw.push_back({ s, tone_window_samples, true, false, 0 });
        }
    }
    for (size_t i = 0; i < targets.size(); ++i) {
        const uint64_t slot = i < target_slots.size() ? target_slots[i] : 0;
        raw.push_back({ targets[i].first, targets[i].second, false, true,
                        slot });
    }
    std::sort(raw.begin(), raw.end(),
              [](const Raw& a, const Raw& b) { return a.start < b.start; });

    std::vector<PhysicalRxInterval> out;
    for (const auto& r : raw) {
        if (r.samples == 0)
            continue;
        if (!out.empty()) {
            PhysicalRxInterval& cur = out.back();
            const int64_t r_end = r.start + static_cast<int64_t>(r.samples);
            if (r.start <= cur.end()) {
                // Overlap or adjacency: merge.
                if (r_end > cur.end())
                    cur.samples = static_cast<size_t>(r_end - cur.start);
                cur.has_tone_training = cur.has_tone_training || r.tone;
                cur.has_target = cur.has_target || r.target;
                PhysicalRxInterval::Slice sl;
                sl.offset = r.start - cur.start;
                sl.samples = r.samples;
                sl.tone_training = r.tone;
                sl.target = r.target;
                sl.slot_index = r.slot;
                cur.slices.push_back(sl);
                continue;
            }
        }
        PhysicalRxInterval iv;
        iv.start = r.start;
        iv.samples = r.samples;
        iv.has_tone_training = r.tone;
        iv.has_target = r.target;
        PhysicalRxInterval::Slice sl;
        sl.offset = 0;
        sl.samples = r.samples;
        sl.tone_training = r.tone;
        sl.target = r.target;
        sl.slot_index = r.slot;
        iv.slices.push_back(sl);
        out.push_back(std::move(iv));
    }
    return out;
}

} // namespace core
} // namespace openuwb
