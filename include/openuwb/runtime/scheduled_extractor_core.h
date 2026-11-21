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
 *
 * This header keeps the schedule arithmetic (predicted_start_sample and the
 * carry-free window bounds), the config/meta/slot structs, the pool accessors
 * and the atomic counters.  The ScheduledWindowCore state machine (CF32 and
 * native SC16), the free-list/ready bookkeeping, the ScheduleLockTracker
 * learn/hold loop and verify_template_in_window are defined in
 * lib/runtime/uwb_scheduled_extractor_core.cc (target openuwb_runtime).
 */

#pragma once

#include <openuwb/support/defaults.h>
#include <openuwb/dsp/matched_filter.h>

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

namespace openuwb {
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
WindowBounds window_bounds_for_slot(double first_packet_sample_exact,
                                    double period_samples_exact,
                                    uint64_t schedule_index,
                                    size_t pre_guard,
                                    size_t capture,
                                    size_t post_guard);

enum class EmitPolicy : int { EverySlot = 0, VerifiedOnly = 1 };
enum class PartialEosPolicy : int { Drop = 0, EmitPartial = 1 };
enum class ScheduleMachineState : int {
    Unlocked = 0,
    WaitWindow = 1,
    CopyWindow = 2,
    Paused = 3
};

struct ScheduleConfig {
    double sample_rate = defaults::kSampleRateHz;
    double packet_interval_s = defaults::kScheduledPacketIntervalS;
    double first_packet_sample_exact = 0.0;
    size_t pre_guard_samples = defaults::kScheduledPreGuard;
    size_t capture_samples = defaults::kScheduledCapture;
    size_t post_guard_samples = defaults::kScheduledPostGuard;
    EmitPolicy emit_policy = EmitPolicy::EverySlot;
    PartialEosPolicy partial_eos_policy = PartialEosPolicy::Drop;
    size_t pool_size = defaults::kScheduledPoolSize;

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
    std::vector<std::complex<int16_t>> samples_sc16;
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
    void configure(const ScheduleConfig& cfg);

    /** Seed schedule and enter WAIT_WINDOW (Phase-1 external seed). */
    void set_schedule(double first_packet_sample_exact,
                      double packet_interval_s,
                      double sample_rate = 0.0);

    void set_abs_cursor(uint64_t abs) { abs_cursor_ = abs; }

    /** Use the SC16 pool (native X410 path).  Call before configure(). */
    void set_sc16_windows(bool en) { sc16_mode_ = en; }
    bool sc16_windows() const { return sc16_mode_; }

    /**
     * Change guards without bumping schedule_generation (holdover widen /
     * lock restore).  Aborts the active window and re-advances k.
     */
    void update_guards_keep_generation(size_t pre_guard,
                                       size_t capture,
                                       size_t post_guard);

    /**
     * Soft-update (t0, T) from schedule lock after learn-then-freeze.
     *
     * force_t0=true (preferred on first Hold): apply absolute learned t0 when
     * the next window is still safely in the future; else fall back to
     * continuity at next_k so the imminent window does not jump:
     *   new_t0 = old_pred(next_k) - next_k * new_period
     *
     * Continuity keeps δ in the period while absorbing residual phase into
     * demod wide search when an absolute re-anchor would be too late.
     */
    bool update_locked_params(double first_packet_sample_exact,
                              double packet_interval_s,
                              bool force_t0 = false);

    void set_guards(size_t pre_guard, size_t capture, size_t post_guard);

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
    void reset();

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
                         uint64_t abs_start);

    /**
     * Native SC16 process_chunk.  Same skip/copy state machine as the CF32
     * path; memcpy stays packed complex<int16_t> (no host conversion).
     */
    size_t process_chunk_sc16(const std::complex<int16_t>* in,
                              size_t n,
                              uint64_t abs_start);

    void flush_eos();

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
    WindowHandle take_window();

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

    Stats stats() const;

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

    void advance_k_to_abs(uint64_t abs);

    bool begin_window(const WindowBounds& bounds, int64_t abs_now);

    void finish_active_window(bool partial);

    void abort_active_window()
    {
        if (active_ == kInvalid)
            return;
        release_to_free(active_);
        active_ = kInvalid;
    }

    void drain_ready_to_free();

    void release_to_free(WindowHandle h);

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
    bool sc16_mode_ = false;
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
 * Schedule lock: learn a fixed interval deviation, then apply slow first-order
 * phase (t0) tracking in Hold with the period T frozen.
 *
 * Operator seeds a nominal radar schedule (t0_seed, T_nom) — for QM35 this is
 * typically T_nom = 5000 us.  The true slot grid is approximately
 *
 *   predicted(k) = (t0_seed + b) + k * (T_nom + δ)
 *
 * where bias b and period deviation δ are nearly constant (SFO ≈ fixed ppm over
 * a capture).  Wide demod search covers residual error during ACQUIRE/LEARN;
 * once (b, δ) converge we enter HOLD and apply a slow first-order phase (t0)
 * PLL to absorb residual clock drift.  The period T stays frozen at the learned
 * value — SFO is a fixed ppm over the capture, so the single learned T
 * suffices; updating T in Hold is ill-conditioned at large schedule index k.
 *
 * State machine:
 *   Searching  — no observations yet
 *   Learning   — buffer (k, detected_start); linear LS for t0 and T
 *   Hold       — apply slow first-order phase (t0) tracking with T frozen;
 *                only re-enter Learning on sustained residual outliers
 *                (maintenance, not a fast loop)
 *
 * Preferred observation: SYNC/preamble timing from the demod (or radar
 * verification peak).  FCS is NOT used as a fast lock loop.
 *
 * All arithmetic is in the extractor sample domain (e.g. 737.28 MS/s).
 */
struct ScheduleLockTracker {
    enum class State : int {
        Searching = 0,
        Learning = 1, // was "Acquiring"
        Hold = 2      // was "Locked" — freeze after converge
    };

    bool enabled = true;
    // Need enough distinct slots for the learned period T to be accurate even
    // with preamble-timing jitter (~±20 samples on QM35 vs ~±1 on DW1000).
    // With min_observations=20 the OLS slope (T) uncertainty drops to ~0.2 ppm,
    // so the frozen-T residual over the remaining ~80 slots stays well under
    // unlock_residual_samples.  (≥3 was only enough to make the 2-param fit
    // residual meaningful when jitter was ~±1 sample.)
    size_t min_observations = 20;
    size_t max_buffer = 48;
    double max_period_rel_err = 0.02; // reject T more than 2% from nominal
    // Max |det - pred| over the learning buffer to accept Hold (samples).
    double converge_residual_samples = 128.0;
    // Hold maintenance: re-learn only after this many consecutive large residuals.
    double unlock_residual_samples = 512.0;
    size_t unlock_outlier_count = 5;
    // Continuous tracking gain in Hold (replaces full freeze): small enough
    // that a single ±1-sample noisy observation barely moves the estimate, but
    // fast enough to absorb residual phase drift.  First-order phase (t0)
    // tracking only — the period T stays frozen at the learned value (SFO is a
    // fixed ppm, so the single learned T suffices).
    double hold_track_phase_gain = 0.10; // first-order t0 correction per residual

    State state = State::Searching;
    double nominal_t0 = 0.0;     // operator seed (immutable until reset)
    double nominal_period = 0.0; // T_nom in samples
    double t0_exact = 0.0;       // learned = nominal_t0 + bias
    double period_samples = 0.0; // learned = nominal_period + delta
    double delta_period = 0.0;   // period_samples - nominal_period
    double bias_t0 = 0.0;        // t0_exact - nominal_t0

    static constexpr size_t kMaxBuf = 64;
    std::array<uint64_t, kMaxBuf> buf_k_{};
    std::array<int64_t, kMaxBuf> buf_det_{};
    size_t n_buf = 0;
    size_t n_obs = 0;
    uint64_t last_k = 0;
    int64_t last_det = -1;
    size_t consecutive_outliers = 0;
    // Times we applied a new (t0,T): one Hold entry plus every continuous
    // Hold tracking nudge (so a live capture grows this well past 1).
    uint64_t lock_updates = 0;

    void reset(double seed_t0, double seed_period_samples)
    {
        state = State::Searching;
        nominal_t0 = seed_t0;
        nominal_period = seed_period_samples;
        t0_exact = seed_t0;
        period_samples = seed_period_samples;
        delta_period = 0.0;
        bias_t0 = 0.0;
        n_buf = 0;
        n_obs = 0;
        last_k = 0;
        last_det = -1;
        consecutive_outliers = 0;
        lock_updates = 0;
    }

    /**
     * Ingest one SYNC-quality detection.
     * @return true only when (t0, period) should be applied to the schedule
     *         (transition into Hold after converge / re-converge).
     */
    bool observe(uint64_t k, int64_t detected_start,
                 double /*nominal_period_arg*/);

private:
    bool fit_linear(size_t n, double& out_t0, double& out_T) const;
};

bool verify_template_in_window(const std::complex<float>* window_iq,
                               size_t n,
                               const std::complex<float>* template_iq,
                               size_t L,
                               size_t search_start,
                               size_t search_len,
                               float threshold,
                               float* out_metric,
                               size_t* out_offset,
                               size_t coarse_stride = 1);

/**
 * Optional lightweight radar verification inside a completed window.
 * Searches normalized |corr|^2 metric near predicted offset for a peak.
 * coarse_stride>1 does a strided coarse pass + fine refine (see above).
 */

} // namespace core
} // namespace openuwb
