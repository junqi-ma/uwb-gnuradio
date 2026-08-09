/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * GNU Radio-independent fixed-interval radar-slot IQ extraction core.
 *
 * Matches MATLAB UWB_demodulation/decode_uwb_all.m::predictFixedIntervalCandidates:
 *
 *   predicted_start(k) = round(t0 + k * T)   // sample domain, independent per k
 *   window = [predicted - pre_guard, predicted + capture + post_guard)
 *
 * Coordinate rule: absolute 0-based sample indices (document any 1-based
 * conversion when comparing to MATLAB file indexes).  Do NOT accumulate a
 * pre-rounded period_samples — that drifts over long sequences.
 *
 * Hot path (process_chunk): bulk skip outside windows; bulk memcpy into a
 * preallocated pool inside windows.  No correlation, PMT, or alloc in process.
 */

#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <complex>
#include <cstdint>
#include <cstring>
#include <limits>
#include <mutex>
#include <vector>

namespace gr {
namespace uwb {
namespace core {

// ---------------------------------------------------------------------------
// Schedule arithmetic (pure functions)
// ---------------------------------------------------------------------------

/**
 * Independent per-slot predicted radar start (0-based absolute sample).
 *
 *   predicted = round(first_packet_sample_exact + k * period_samples_exact)
 *
 * first_packet_sample_exact may be non-integer if derived from time*fs;
 * period_samples_exact = packet_interval_s * sample_rate must stay double.
 */
inline int64_t predicted_start_sample(double first_packet_sample_exact,
                                      double period_samples_exact,
                                      uint64_t schedule_index)
{
    // Match MATLAB: round((t0 + k*T)*fs) using double then llround.
    // Do not accumulate a pre-rounded integer period (that drifts).
    const double t = first_packet_sample_exact +
                     static_cast<double>(schedule_index) * period_samples_exact;
    return static_cast<int64_t>(std::llround(t));
}

/**
 * Cumulative-integer baseline (WRONG — drifts).  Exposed only for QA to
 * prove independent round diverges from this over long non-integer periods.
 */
inline int64_t predicted_start_cumulative_integer(int64_t first_packet_sample,
                                                  int64_t period_samples_rounded,
                                                  uint64_t schedule_index)
{
    return first_packet_sample +
           static_cast<int64_t>(schedule_index) * period_samples_rounded;
}

struct WindowBounds {
    int64_t predicted_start = 0;
    int64_t window_start = 0; // inclusive, absolute
    int64_t window_end = 0;   // exclusive, absolute
    size_t capacity = 0;
};

/**
 * Window for schedule index k.
 * Scheme §2.1:
 *   [predicted_start - pre_guard, predicted_start + capture + post_guard)
 * If predicted_start < pre_guard, window_start clamps to 0 and window_end
 * keeps the full capacity so IQ length stays fixed when possible.
 */
inline WindowBounds window_bounds_for_slot(double first_packet_sample_exact,
                                           double period_samples_exact,
                                           uint64_t schedule_index,
                                           size_t pre_guard,
                                           size_t capture,
                                           size_t post_guard)
{
    WindowBounds b;
    b.predicted_start = predicted_start_sample(
        first_packet_sample_exact, period_samples_exact, schedule_index);
    b.capacity = pre_guard + capture + post_guard;
    const int64_t ideal_start =
        b.predicted_start - static_cast<int64_t>(pre_guard);
    if (ideal_start >= 0) {
        b.window_start = ideal_start;
        b.window_end = b.predicted_start + static_cast<int64_t>(capture) +
                       static_cast<int64_t>(post_guard);
    } else {
        b.window_start = 0;
        b.window_end = static_cast<int64_t>(b.capacity);
    }
    return b;
}

enum class EmitPolicy : int { EverySlot = 0, VerifiedOnly = 1 };
enum class PartialEosPolicy : int { Drop = 0, EmitPartial = 1 };
enum class ScheduleMachineState : int {
    Unlocked = 0,
    WaitWindow = 1,
    CopyWindow = 2,
    Paused = 3
};

struct ScheduleConfig {
    double sample_rate = 998400000.0;
    double packet_interval_s = 0.01; // 100 radar/s
    double first_packet_sample_exact = 0.0;
    size_t pre_guard_samples = 9984;   // ~10 us @ 998.4e6
    size_t capture_samples = 189696;   // ~190 us
    size_t post_guard_samples = 4096;
    EmitPolicy emit_policy = EmitPolicy::EverySlot;
    PartialEosPolicy partial_eos_policy = PartialEosPolicy::Drop;
    size_t pool_size = 8;

    size_t window_capacity() const
    {
        return pre_guard_samples + capture_samples + post_guard_samples;
    }

    double period_samples_exact() const
    {
        return packet_interval_s * sample_rate;
    }
};

struct WindowMeta {
    uint64_t schedule_index = 0;
    int64_t predicted_start_sample = 0;
    int64_t window_start_sample = 0;
    int64_t window_end_sample = 0; // exclusive
    size_t sample_count = 0;
    bool partial = false;
    bool radar_verified = false;
    float radar_metric = 0.0f;
    bool comm_present = false;
    float comm_metric = 0.0f;
    bool collision = false;
    int64_t detected_start_sample = -1;
    int64_t timing_error_samples = 0;
    uint64_t schedule_generation = 0;
};

struct WindowSlot {
    WindowMeta meta;
    std::vector<std::complex<float>> samples;
    size_t filled = 0; // samples written so far toward capacity
    bool active = false;
};

/**
 * Fixed-interval window schedule stream processor.
 *
 * Thread model (matches UwbDetector pool pattern):
 * - work thread: process_chunk / finish → ready queue / take_window
 * - worker thread: exclusive owner of a taken handle until release_window
 * - free-list acquire/release is mutex-protected (begin_window vs release)
 * - counters that both threads update are atomic
 * - configure()/reset() require zero checked-out handles; the GR block must
 *   wait_for_worker_idle() before calling them mid-stream
 */
class ScheduledWindowCore {
public:
    using WindowHandle = size_t;
    static constexpr WindowHandle kInvalid = std::numeric_limits<size_t>::max();
    static constexpr size_t kMaxPoolSize = 16;

    explicit ScheduledWindowCore(const ScheduleConfig& cfg = ScheduleConfig())
    {
        configure(cfg);
    }

    /**
     * Full reconfigure.  Caller must ensure checked_out()==0 (no worker-held
     * handles).  Ready and active windows are returned to the free list first.
     */
    void configure(const ScheduleConfig& cfg)
    {
        abort_active_window();
        drain_ready_to_free();
        {
            std::lock_guard<std::mutex> lock(pool_mutex_);
            // Never wipe slots still checked out to the worker.
            if (checked_out_ != 0) {
                // Soft recovery: only rebuild free list from non-checked-out
                // handles; leave worker-owned samples intact until release.
                rebuild_free_excluding_checked_out_locked();
                cfg_ = cfg;
                if (cfg_.pool_size == 0)
                    cfg_.pool_size = 1;
                if (cfg_.pool_size > kMaxPoolSize)
                    cfg_.pool_size = kMaxPoolSize;
                // Do not clear checked-out pool entries.
                for (size_t i = 0; i < kMaxPoolSize; ++i) {
                    if (checked_out_flags_[i])
                        continue;
                    const size_t cap = cfg_.window_capacity();
                    pool_[i].samples.clear();
                    pool_[i].samples.reserve(cap);
                    pool_[i].filled = 0;
                    pool_[i].active = false;
                    pool_[i].meta = WindowMeta{};
                }
            } else {
                cfg_ = cfg;
                if (cfg_.pool_size == 0)
                    cfg_.pool_size = 1;
                if (cfg_.pool_size > kMaxPoolSize)
                    cfg_.pool_size = kMaxPoolSize;
                const size_t cap = cfg_.window_capacity();
                for (size_t i = 0; i < kMaxPoolSize; ++i) {
                    pool_[i].samples.clear();
                    pool_[i].samples.reserve(cap);
                    pool_[i].filled = 0;
                    pool_[i].active = false;
                    pool_[i].meta = WindowMeta{};
                    checked_out_flags_[i] = false;
                }
                free_count_ = cfg_.pool_size;
                for (size_t i = 0; i < cfg_.pool_size; ++i)
                    free_[i] = cfg_.pool_size - 1 - i;
                checked_out_ = 0;
            }
        }
        {
            std::lock_guard<std::mutex> lock(pool_mutex_);
            ready_head_ = ready_tail_ = ready_count_ = 0;
        }
        active_ = kInvalid;
        next_k_ = 0;
        abs_cursor_ = 0;
        state_ = ScheduleMachineState::Unlocked;
        clear_stats();
        paused_ = false;
        ++generation_;
    }

    /** Seed schedule and enter WAIT_WINDOW (Phase-1 external seed). */
    void set_schedule(double first_packet_sample_exact,
                      double packet_interval_s,
                      double sample_rate = 0.0)
    {
        if (sample_rate > 0.0)
            cfg_.sample_rate = sample_rate;
        if (packet_interval_s > 0.0)
            cfg_.packet_interval_s = packet_interval_s;
        cfg_.first_packet_sample_exact = first_packet_sample_exact;
        abort_active_window();
        next_k_ = 0;
        advance_k_to_abs(abs_cursor_);
        state_ = paused_ ? ScheduleMachineState::Paused
                         : ScheduleMachineState::WaitWindow;
        ++generation_;
        schedule_updates_.fetch_add(1, std::memory_order_relaxed);
    }

    void set_abs_cursor(uint64_t abs) { abs_cursor_ = abs; }

    void set_guards(size_t pre_guard, size_t capture, size_t post_guard)
    {
        cfg_.pre_guard_samples = pre_guard;
        cfg_.capture_samples = capture;
        cfg_.post_guard_samples = post_guard;
        const size_t cap = cfg_.window_capacity();
        {
            std::lock_guard<std::mutex> lock(pool_mutex_);
            for (size_t i = 0; i < kMaxPoolSize; ++i) {
                if (checked_out_flags_[i])
                    continue;
                if (pool_[i].samples.capacity() < cap)
                    pool_[i].samples.reserve(cap);
            }
        }
        if (state_ != ScheduleMachineState::Unlocked) {
            abort_active_window();
            advance_k_to_abs(abs_cursor_);
            state_ = paused_ ? ScheduleMachineState::Paused
                             : ScheduleMachineState::WaitWindow;
            ++generation_;
        }
    }

    void pause()
    {
        paused_ = true;
        abort_active_window();
        state_ = ScheduleMachineState::Paused;
    }

    void resume()
    {
        paused_ = false;
        if (state_ == ScheduleMachineState::Paused ||
            state_ == ScheduleMachineState::Unlocked) {
            if (cfg_.packet_interval_s > 0.0 && cfg_.sample_rate > 0.0) {
                advance_k_to_abs(abs_cursor_);
                state_ = ScheduleMachineState::WaitWindow;
            }
        }
    }

    /**
     * Reset schedule state.  Caller should ensure checked_out()==0 for a full
     * pool wipe; otherwise checked-out slots are left for release_window.
     */
    void reset()
    {
        abort_active_window();
        drain_ready_to_free();
        next_k_ = 0;
        abs_cursor_ = 0;
        state_ = ScheduleMachineState::Unlocked;
        paused_ = false;
        clear_stats();
        {
            std::lock_guard<std::mutex> lock(pool_mutex_);
            if (checked_out_ == 0) {
                free_count_ = cfg_.pool_size;
                for (size_t i = 0; i < cfg_.pool_size; ++i) {
                    free_[i] = cfg_.pool_size - 1 - i;
                    pool_[i].samples.clear();
                    pool_[i].filled = 0;
                    pool_[i].active = false;
                    pool_[i].meta = WindowMeta{};
                    checked_out_flags_[i] = false;
                }
            } else {
                rebuild_free_excluding_checked_out_locked();
            }
        }
        ++generation_;
    }

    ScheduleConfig config() const { return cfg_; }
    ScheduleMachineState state() const { return state_; }
    uint64_t abs_cursor() const { return abs_cursor_; }
    uint64_t next_schedule_index() const { return next_k_; }
    uint64_t schedule_generation() const { return generation_; }

    /** Handles currently owned by take_window() consumers (worker). */
    size_t checked_out() const
    {
        std::lock_guard<std::mutex> lock(pool_mutex_);
        return checked_out_;
    }

    /**
     * Process n input samples starting at absolute abs_start.
     * work-thread only.  Returns n.
     */
    size_t process_chunk(const std::complex<float>* in,
                         size_t n,
                         uint64_t abs_start)
    {
        if (n == 0)
            return 0;
        if (abs_cursor_ == 0 && abs_start > 0 &&
            state_ == ScheduleMachineState::Unlocked) {
            abs_cursor_ = abs_start;
        }
        if (abs_start != abs_cursor_) {
            abort_active_window();
            abs_cursor_ = abs_start;
            if (state_ == ScheduleMachineState::WaitWindow ||
                state_ == ScheduleMachineState::CopyWindow) {
                advance_k_to_abs(abs_cursor_);
                state_ = ScheduleMachineState::WaitWindow;
            }
        }

        size_t offset = 0;
        while (offset < n) {
            if (state_ == ScheduleMachineState::Unlocked ||
                state_ == ScheduleMachineState::Paused) {
                const size_t take = n - offset;
                abs_cursor_ += take;
                offset += take;
                continue;
            }

            const WindowBounds bounds = bounds_for(next_k_);
            const int64_t abs = static_cast<int64_t>(abs_cursor_);

            if (state_ == ScheduleMachineState::WaitWindow) {
                if (abs >= bounds.window_end) {
                    dropped_windows_.fetch_add(1, std::memory_order_relaxed);
                    scheduled_windows_.fetch_add(1, std::memory_order_relaxed);
                    ++next_k_;
                    continue;
                }
                if (abs < bounds.window_start) {
                    const int64_t gap = bounds.window_start - abs;
                    const size_t take = static_cast<size_t>(std::min(
                        gap, static_cast<int64_t>(n - offset)));
                    abs_cursor_ += take;
                    offset += take;
                    continue;
                }
                if (!begin_window(bounds, abs)) {
                    const int64_t remain = bounds.window_end - abs;
                    const size_t take = static_cast<size_t>(std::min(
                        remain, static_cast<int64_t>(n - offset)));
                    abs_cursor_ += take;
                    offset += take;
                    if (static_cast<int64_t>(abs_cursor_) >= bounds.window_end) {
                        scheduled_windows_.fetch_add(1,
                                                     std::memory_order_relaxed);
                        dropped_windows_.fetch_add(1, std::memory_order_relaxed);
                        ++next_k_;
                    }
                    continue;
                }
                state_ = ScheduleMachineState::CopyWindow;
            }

            WindowSlot& slot = pool_[active_];
            const int64_t win_end = slot.meta.window_end_sample;
            const int64_t remain_abs =
                win_end - static_cast<int64_t>(abs_cursor_);
            const size_t need = slot.meta.sample_count > slot.filled
                                    ? slot.meta.sample_count - slot.filled
                                    : 0;
            const size_t take = std::min(
                { static_cast<size_t>(std::max(remain_abs, int64_t{ 0 })),
                  need,
                  n - offset });
            if (take > 0) {
                std::memcpy(slot.samples.data() + slot.filled,
                            in + offset,
                            take * sizeof(std::complex<float>));
                slot.filled += take;
                abs_cursor_ += take;
                offset += take;
            }
            if (slot.filled >= slot.meta.sample_count ||
                static_cast<int64_t>(abs_cursor_) >= win_end) {
                finish_active_window(/*partial=*/slot.filled <
                                     slot.meta.sample_count);
            } else if (take == 0) {
                abs_cursor_ += 1;
                offset += 1;
            }
        }
        return n;
    }

    void flush_eos()
    {
        if (active_ == kInvalid)
            return;
        WindowSlot& slot = pool_[active_];
        const bool partial = slot.filled < slot.meta.sample_count;
        if (partial && cfg_.partial_eos_policy == PartialEosPolicy::Drop) {
            partial_windows_.fetch_add(1, std::memory_order_relaxed);
            dropped_windows_.fetch_add(1, std::memory_order_relaxed);
            release_to_free(active_);
            active_ = kInvalid;
            state_ = ScheduleMachineState::WaitWindow;
            return;
        }
        slot.meta.partial = partial;
        slot.meta.sample_count = slot.filled;
        finish_active_window(partial);
    }

    bool window_ready() const
    {
        std::lock_guard<std::mutex> lock(pool_mutex_);
        return ready_count_ > 0;
    }

    /**
     * Take a completed window for exclusive consumer use (worker).
     * Marks handle checked-out; free/ready mutations are mutex-protected so
     * concurrent release_window from a worker is safe.
     */
    WindowHandle take_window()
    {
        std::lock_guard<std::mutex> lock(pool_mutex_);
        if (ready_count_ == 0)
            return kInvalid;
        const WindowHandle h = ready_[ready_head_];
        ready_head_ = (ready_head_ + 1) % kMaxPoolSize;
        --ready_count_;
        if (h < kMaxPoolSize) {
            checked_out_flags_[h] = true;
            ++checked_out_;
        }
        return h;
    }

    /**
     * Access slot for a handle.  Only the owner (work while ACTIVE/READY, or
     * worker while checked-out) may read/write it.
     */
    const WindowSlot& window(WindowHandle h) const { return pool_[h]; }
    WindowSlot& window(WindowHandle h) { return pool_[h]; }

    /** Worker-thread safe: return handle to free list. */
    void release_window(WindowHandle h)
    {
        if (h == kInvalid || h >= kMaxPoolSize)
            return;
        release_to_free(h);
    }

    struct Stats {
        uint64_t scheduled_windows = 0;
        uint64_t completed_windows = 0;
        uint64_t emitted_windows = 0;
        uint64_t partial_windows = 0;
        uint64_t dropped_windows = 0;
        uint64_t queue_high_watermark = 0;
        uint64_t schedule_updates = 0;
        uint64_t verification_failures = 0;
        uint64_t collisions = 0;
    };

    Stats stats() const
    {
        Stats s;
        s.scheduled_windows =
            scheduled_windows_.load(std::memory_order_relaxed);
        s.completed_windows =
            completed_windows_.load(std::memory_order_relaxed);
        s.emitted_windows = emitted_windows_.load(std::memory_order_relaxed);
        s.partial_windows = partial_windows_.load(std::memory_order_relaxed);
        s.dropped_windows = dropped_windows_.load(std::memory_order_relaxed);
        s.queue_high_watermark =
            queue_high_watermark_.load(std::memory_order_relaxed);
        s.schedule_updates =
            schedule_updates_.load(std::memory_order_relaxed);
        s.verification_failures =
            verification_failures_.load(std::memory_order_relaxed);
        s.collisions = collisions_.load(std::memory_order_relaxed);
        return s;
    }

    void note_emitted()
    {
        emitted_windows_.fetch_add(1, std::memory_order_relaxed);
    }
    void note_verification_failure()
    {
        verification_failures_.fetch_add(1, std::memory_order_relaxed);
    }
    void note_collision()
    {
        collisions_.fetch_add(1, std::memory_order_relaxed);
    }

    void note_queue_depth(uint64_t depth)
    {
        uint64_t prev =
            queue_high_watermark_.load(std::memory_order_relaxed);
        while (depth > prev &&
               !queue_high_watermark_.compare_exchange_weak(
                   prev, depth, std::memory_order_relaxed)) {
        }
    }

private:
    WindowBounds bounds_for(uint64_t k) const
    {
        return window_bounds_for_slot(cfg_.first_packet_sample_exact,
                                      cfg_.period_samples_exact(),
                                      k,
                                      cfg_.pre_guard_samples,
                                      cfg_.capture_samples,
                                      cfg_.post_guard_samples);
    }

    void clear_stats()
    {
        scheduled_windows_.store(0, std::memory_order_relaxed);
        completed_windows_.store(0, std::memory_order_relaxed);
        emitted_windows_.store(0, std::memory_order_relaxed);
        partial_windows_.store(0, std::memory_order_relaxed);
        dropped_windows_.store(0, std::memory_order_relaxed);
        queue_high_watermark_.store(0, std::memory_order_relaxed);
        schedule_updates_.store(0, std::memory_order_relaxed);
        verification_failures_.store(0, std::memory_order_relaxed);
        collisions_.store(0, std::memory_order_relaxed);
    }

    void advance_k_to_abs(uint64_t abs)
    {
        const double period = cfg_.period_samples_exact();
        if (period <= 0.0)
            return;
        const double first = cfg_.first_packet_sample_exact;
        const size_t pre = cfg_.pre_guard_samples;
        if (static_cast<double>(abs) + static_cast<double>(pre) < first) {
            next_k_ = 0;
            return;
        }
        const long double k_est =
            (static_cast<long double>(abs) + static_cast<long double>(pre) -
             static_cast<long double>(first)) /
            static_cast<long double>(period);
        uint64_t k = k_est > 0 ? static_cast<uint64_t>(k_est) : 0;
        while (true) {
            const WindowBounds b = bounds_for(k);
            if (b.window_end > static_cast<int64_t>(abs)) {
                next_k_ = k;
                return;
            }
            ++k;
            if (k > next_k_ + 100000000ULL) {
                next_k_ = k;
                return;
            }
        }
    }

    bool begin_window(const WindowBounds& bounds, int64_t abs_now)
    {
        const int64_t actual_start = std::max(abs_now, bounds.window_start);
        if (actual_start >= bounds.window_end)
            return false;
        const size_t remaining =
            static_cast<size_t>(bounds.window_end - actual_start);
        const bool partial_start = actual_start > bounds.window_start;

        WindowHandle h = kInvalid;
        {
            std::lock_guard<std::mutex> lock(pool_mutex_);
            if (free_count_ == 0)
                return false;
            h = free_[--free_count_];
        }

        WindowSlot& slot = pool_[h];
        slot.active = true;
        slot.filled = 0;
        slot.meta = WindowMeta{};
        slot.meta.schedule_index = next_k_;
        slot.meta.predicted_start_sample = bounds.predicted_start;
        slot.meta.window_start_sample = actual_start;
        slot.meta.window_end_sample = bounds.window_end;
        slot.meta.sample_count = remaining;
        slot.meta.partial = partial_start || remaining < bounds.capacity;
        slot.meta.schedule_generation = generation_;
        if (slot.samples.size() < remaining)
            slot.samples.resize(remaining);
        active_ = h;
        scheduled_windows_.fetch_add(1, std::memory_order_relaxed);
        if (slot.meta.partial)
            partial_windows_.fetch_add(1, std::memory_order_relaxed);
        return true;
    }

    void finish_active_window(bool partial)
    {
        if (active_ == kInvalid)
            return;
        WindowSlot& slot = pool_[active_];
        slot.meta.partial = slot.meta.partial || partial;
        slot.meta.sample_count = slot.filled;
        const WindowHandle h = active_;
        active_ = kInvalid;
        ++next_k_;
        state_ = paused_ ? ScheduleMachineState::Paused
                         : ScheduleMachineState::WaitWindow;

        {
            std::lock_guard<std::mutex> lock(pool_mutex_);
            if (ready_count_ < kMaxPoolSize) {
                ready_[ready_tail_] = h;
                ready_tail_ = (ready_tail_ + 1) % kMaxPoolSize;
                ++ready_count_;
                completed_windows_.fetch_add(1, std::memory_order_relaxed);
                const uint64_t depth = ready_count_;
                // Atomic watermark update (also used from worker path).
                uint64_t prev =
                    queue_high_watermark_.load(std::memory_order_relaxed);
                while (depth > prev &&
                       !queue_high_watermark_.compare_exchange_weak(
                           prev, depth, std::memory_order_relaxed)) {
                }
                return;
            }
        }
        // Ready full — return slot under free-list lock path.
        release_to_free(h);
        dropped_windows_.fetch_add(1, std::memory_order_relaxed);
    }

    void abort_active_window()
    {
        if (active_ == kInvalid)
            return;
        release_to_free(active_);
        active_ = kInvalid;
    }

    void drain_ready_to_free()
    {
        for (;;) {
            WindowHandle h = kInvalid;
            {
                std::lock_guard<std::mutex> lock(pool_mutex_);
                if (ready_count_ == 0)
                    break;
                h = ready_[ready_head_];
                ready_head_ = (ready_head_ + 1) % kMaxPoolSize;
                --ready_count_;
            }
            release_to_free(h);
        }
    }

    void release_to_free(WindowHandle h)
    {
        if (h >= kMaxPoolSize)
            return;
        // Clear meta/samples before returning to free so a subsequent
        // begin_window does not observe stale IQ.  Worker finishes publish
        // before calling release_window.
        pool_[h].active = false;
        pool_[h].filled = 0;
        pool_[h].meta = WindowMeta{};
        // Keep vector capacity; do not free memory.
        {
            std::lock_guard<std::mutex> lock(pool_mutex_);
            if (checked_out_flags_[h]) {
                checked_out_flags_[h] = false;
                if (checked_out_ > 0)
                    --checked_out_;
            }
            // Avoid double-free: only push if not already present.
            // free list is small; scan is acceptable.
            for (size_t i = 0; i < free_count_; ++i) {
                if (free_[i] == h)
                    return;
            }
            if (free_count_ < cfg_.pool_size)
                free_[free_count_++] = h;
        }
    }

    void rebuild_free_excluding_checked_out_locked()
    {
        free_count_ = 0;
        for (size_t i = 0; i < cfg_.pool_size; ++i) {
            if (!checked_out_flags_[i])
                free_[free_count_++] = i;
        }
    }

    ScheduleConfig cfg_;
    std::array<WindowSlot, kMaxPoolSize> pool_{};
    std::array<WindowHandle, kMaxPoolSize> free_{};
    std::array<WindowHandle, kMaxPoolSize> ready_{};
    std::array<bool, kMaxPoolSize> checked_out_flags_{};
    mutable std::mutex pool_mutex_;
    size_t free_count_ = 0;
    size_t checked_out_ = 0;
    size_t ready_head_ = 0;
    size_t ready_tail_ = 0;
    size_t ready_count_ = 0;
    WindowHandle active_ = kInvalid;
    uint64_t next_k_ = 0;
    uint64_t abs_cursor_ = 0;
    ScheduleMachineState state_ = ScheduleMachineState::Unlocked;
    bool paused_ = false;
    uint64_t generation_ = 0;

    std::atomic<uint64_t> scheduled_windows_{ 0 };
    std::atomic<uint64_t> completed_windows_{ 0 };
    std::atomic<uint64_t> emitted_windows_{ 0 };
    std::atomic<uint64_t> partial_windows_{ 0 };
    std::atomic<uint64_t> dropped_windows_{ 0 };
    std::atomic<uint64_t> queue_high_watermark_{ 0 };
    std::atomic<uint64_t> schedule_updates_{ 0 };
    std::atomic<uint64_t> verification_failures_{ 0 };
    std::atomic<uint64_t> collisions_{ 0 };
};

/**
 * Optional lightweight radar verification inside a completed window.
 * Searches normalized |corr|^2 metric near predicted offset for a peak.
 * Does not allocate; uses caller-provided scratch of size >= search_len.
 *
 * @param window_iq     captured window samples
 * @param n             sample count
 * @param template_iq   L2-normalized or raw template (normalized here if needed)
 * @param L             template length
 * @param search_start  offset in window to begin search
 * @param search_len    number of alignments to try
 * @param out_metric    peak metric [0,1]
 * @param out_offset    offset of peak in window
 * @return true if peak metric >= threshold
 */
inline bool verify_template_in_window(const std::complex<float>* window_iq,
                                      size_t n,
                                      const std::complex<float>* template_iq,
                                      size_t L,
                                      size_t search_start,
                                      size_t search_len,
                                      float threshold,
                                      float* out_metric,
                                      size_t* out_offset)
{
    if (out_metric)
        *out_metric = 0.0f;
    if (out_offset)
        *out_offset = 0;
    if (!window_iq || !template_iq || L == 0 || n < L)
        return false;

    // Template energy
    float t_energy = 0.0f;
    for (size_t i = 0; i < L; ++i)
        t_energy += std::norm(template_iq[i]);
    if (t_energy < 1e-20f)
        return false;

    const size_t max_align = n - L + 1;
    if (search_start >= max_align)
        return false;
    const size_t end =
        std::min(search_start + (search_len > 0 ? search_len : max_align),
                 max_align);

    float best = 0.0f;
    size_t best_j = search_start;
    for (size_t j = search_start; j < end; ++j) {
        std::complex<float> corr(0.0f, 0.0f);
        float pwr = 0.0f;
        for (size_t k = 0; k < L; ++k) {
            corr += window_iq[j + k] * std::conj(template_iq[k]);
            pwr += std::norm(window_iq[j + k]);
        }
        const float denom = pwr * t_energy + 1e-12f;
        const float metric = std::norm(corr) / denom;
        if (metric > best) {
            best = metric;
            best_j = j;
        }
    }
    if (out_metric)
        *out_metric = best;
    if (out_offset)
        *out_offset = best_j;
    return best >= threshold;
}

} // namespace core
} // namespace uwb
} // namespace gr
