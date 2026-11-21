/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * GNU Radio-independent periodic-target lock core for the native X410 live
 * receiver (uwb_dw3000_live).
 *
 * Design contract (docs/performance/X410_目标周期锁定与随机干扰双路径修改规划_2026-11-15.md):
 *
 *   ACQUIRE -> VERIFY -> LOCKED -> HOLDOVER -> REACQUIRE
 *
 *   - ACQUIRE   : no schedule.  Wide legacy acquisition + autocorrelation
 *                 candidates confirmed by the target code template.
 *   - VERIFY    : the first identity-confirmed target packet seeds the slot
 *                 grid; `verify_observations` independent valid packets are
 *                 needed before LOCKED.  Slot indices are inferred from the
 *                 observed interval, so a single missed packet (2 slots of
 *                 gap) is still the same grid and is never mistaken for a new
 *                 period.
 *   - LOCKED    : every expected slot is planned from the learned
 *                 (t0, period) model.  Prediction is
 *                     n_pred(k) = round(t0_exact + k * period_samples_exact)
 *                 evaluated independently per k -- a rounded integer period is
 *                 never accumulated (that drifts on a 4.999864 ms capture).
 *   - HOLDOVER  : a target failure keeps the phase/period prediction, does not
 *                 update the lock from failed/unknown observations, and widens
 *                 the guards with the time since the last valid observation.
 *   - REACQUIRE : sustained target failures, residual out of bounds or an
 *                 unexpected RX gap invalidates the grid and returns to
 *                 ACQUIRE.  Global reacquisition happens only here, on the
 *                 scheduler thread, never inside a DSP worker.
 *
 * Threading contract:
 *   - Only the scheduler (RX) thread mutates lock state.
 *   - DSP workers push TargetObservation records into a bounded queue with
 *     push_observation(); they never read or write the lock itself.
 *   - Out-of-order completion is expected.  Duplicate, stale-generation and
 *     already-applied slots are ignored, so worker completion order is never
 *     mistaken for air order.
 *
 * Coordinates: absolute 0-based NATIVE samples (737.28 MS/s for X410 without
 * RFNoC).  A demod observation at 998.4 MS/s must be mapped to native before
 * it reaches this core (see uwb_qm35_acquisition_tracker.h).
 *
 * This header keeps the state/outcome enums, the configuration/observation/
 * plan/stats structs and the small accessors and predicates.  The state
 * machine, the bounded feedback queue, the (t0, period) fit, slot planning and
 * candidate classification are defined in
 * lib/runtime/uwb_target_lock_core.cc (target openuwb_runtime).
 */

#pragma once

#include <openuwb/support/defaults.h>
#include <openuwb/runtime/scheduled_extractor_core.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <map>
#include <mutex>
#include <string>
#include <vector>

namespace openuwb {
namespace core {

// ---------------------------------------------------------------------------
// State machine
// ---------------------------------------------------------------------------
enum class TargetLockState : int {
    Acquire = 0,
    Verify = 1,
    Locked = 2,
    Holdover = 3,
    Reacquire = 4
};

const char* target_lock_state_name(TargetLockState s);

// ---------------------------------------------------------------------------
// Window geometry
// ---------------------------------------------------------------------------
/**
 * Fixed-position target window around a predicted packet origin.
 *
 * The packet start sits at `pre_guard` inside the window (not centred), so the
 * whole packet plus halo always fits.  `capture` is the maximum packet span
 * derived from the PHY (256 SYNC + SFD + PHR + max PSDU + pulse/halo), never
 * from the PHR of the packet being decoded.
 */
struct TargetWindowGeometry {
    size_t pre_guard = 0;
    size_t capture = 0;
    size_t post_guard = 0;

    size_t capacity() const { return pre_guard + capture + post_guard; }
    bool valid() const { return capture > 0; }
};

/**
 * Derive the maximum packet span in samples at `sample_rate` for a BPRF HRP
 * frame.  All arguments are sample counts in the SAME domain as the window:
 *
 *   sync_span        sync_repetitions * samples_per_symbol
 *   sfd_span         sfd_symbols      * samples_per_symbol
 *   phr_span         PHR symbols * chips/symbol * samples/chip
 *   payload_span     max PSDU BPM symbols * chips/symbol * samples/chip
 *   pulse_halo       pulse-shaping tail + resampler filter halo
 *
 * Callers derive the four spans from the active PHY profile (see
 * mod::packet_samples_998p4 for the 998.4 MS/s work grid) and pass them here so
 * this core stays rate- and PHY-independent and unit-testable.
 */
inline TargetWindowGeometry target_window_geometry(size_t sync_span,
                                                   size_t sfd_span,
                                                   size_t phr_span,
                                                   size_t payload_span,
                                                   size_t pulse_halo,
                                                   size_t pre_guard,
                                                   size_t post_guard)
{
    TargetWindowGeometry g;
    g.pre_guard = pre_guard;
    g.capture = sync_span + sfd_span + phr_span + payload_span + pulse_halo;
    g.post_guard = post_guard;
    return g;
}

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------
struct TargetLockConfig {
    double sample_rate = defaults::kNativeSampleRateHz;
    // Nominal period is only an admission gate; the learned period replaces it.
    double nominal_period_s = defaults::kQm35PacketIntervalS;

    // VERIFY needs this many independent identity-valid target packets.
    size_t verify_observations = defaults::kLockObservations;
    // Observations used by the least-squares (t0, period) fit.  §5 asks for
    // 8-16 valid observations to stabilise the period estimate.
    size_t period_observations = 16;
    double max_period_rel_err = 0.02;

    // LOCKED -> HOLDOVER after this many consecutive target failures.
    size_t holdover_miss_count = defaults::kHoldoverMissCount;
    // HOLDOVER/VERIFY -> REACQUIRE after this many consecutive failures.
    size_t reacquire_miss_count = defaults::kReacquireMissCount;

    // Residual gate: a valid observation whose |observed - predicted| exceeds
    // this is an outlier.  Consecutive outliers force REACQUIRE.
    double residual_limit_samples = 512.0;
    size_t residual_outlier_count = 3;
    // While VERIFYing, an observation whose inferred slot is more than this many
    // slots away from the last valid one is not the same grid (a same-code
    // emitter, or a re-tuned device).  It is ignored, never used to re-anchor.
    size_t max_slot_gap = 8;

    // Holdover guard widening: extra pre/post guard per consecutive miss,
    // capped.  Both are native samples.
    size_t holdover_guard_growth = 2048;
    size_t holdover_guard_cap = 32768;

    // Bounded feedback queue (worker -> scheduler).
    size_t feedback_queue_capacity = 64;

    // Interference association: a candidate within this distance of the
    // predicted target origin is a candidate for the target's own response.
    // Fixed at construction (0 = use geometry.pre_guard) on purpose: it is a
    // distance from the prediction, not a window budget, so a runtime guard
    // shrink must never narrow it.
    int64_t target_association_window = 0; // 0 = use geometry.pre_guard

    // Samples available in one job buffer.  set_pre_guard() rejects a geometry
    // that does not fit; the buffer itself is allocated for the widest
    // geometry, so a runtime update may only shrink the window.  0 = runtime
    // guard updates are refused (fail closed).
    size_t window_capacity = 0;

    // Target identity rules.  A code mismatch is interference, never a target.
    int code_index = 10;
    size_t preamble_repetitions = 256;
    std::string sfd_mode = "4z2";
    bool require_fcs = true;

    TargetWindowGeometry geometry;

    double period_samples() const
    {
        return nominal_period_s * sample_rate;
    }
};

// ---------------------------------------------------------------------------
// Observations and plans
// ---------------------------------------------------------------------------
struct TargetObservation {
    uint64_t packet_id = 0;
    uint64_t schedule_generation = 0;
    uint64_t slot_index = 0;
    int64_t window_origin = 0;    // absolute native window start
    int64_t observed_origin = -1; // absolute native preamble start
    int64_t predicted_origin = 0; // absolute native predicted origin
    int64_t residual = 0;         // observed - predicted
    bool timing_ok = false;
    bool sfd_ok = false;
    bool phr_ok = false;
    bool fcs_pass = false;
    int code_index = -1;   // -1 = not reported
    size_t preamble_repetitions = 0;
    std::string sfd_mode;
    float quality = 0.0f;
    bool from_target_window = false;
};

enum class TargetSlotOutcome : int {
    Planned = 0,
    Submitted = 1,
    DroppedQueue = 2,
    Expired = 3,
    NotDetected = 4,
    DecodeFailed = 5,
    FcsFailed = 6,
    Success = 7
};

const char* target_slot_outcome_name(TargetSlotOutcome o);

struct TargetSlotPlan {
    uint64_t slot_index = 0;
    uint64_t generation = 0;
    int64_t predicted_origin = 0;
    int64_t window_start = 0;
    int64_t window_end = 0; // exclusive
    size_t pre_guard = 0;
    size_t capture = 0;
    size_t post_guard = 0;
    bool widened = false; // holdover guard expansion active
};

// ---------------------------------------------------------------------------
// Interference path
// ---------------------------------------------------------------------------
struct InterferenceCandidate {
    int64_t start_sample = -1;
    double metric = 0.0;
    int code_index = -1; // template confirmation result, -1 = not confirmed
    bool template_confirmed = false;
};

enum class CandidateClass : int {
    TargetAssociated = 0, // the locked target's own autocorrelation response
    Interference = 1,     // a different emitter (code/time says so)
    Unknown = 2           // not enough evidence; never silently dropped
};

const char* candidate_class_name(CandidateClass c);

struct CandidateVerdict {
    CandidateClass cls = CandidateClass::Unknown;
    uint64_t slot_index = std::numeric_limits<uint64_t>::max();
    int64_t offset_from_target = 0;
};

// ---------------------------------------------------------------------------
// Statistics (§10)
// ---------------------------------------------------------------------------
struct TargetLockStats {
    uint64_t expected_slots = 0;
    uint64_t submitted = 0;
    uint64_t dropped_queue = 0;
    uint64_t expired = 0;
    uint64_t not_detected = 0;
    uint64_t decode_failed = 0;
    uint64_t fcs_ok = 0;
    uint64_t fcs_failed = 0;
    uint64_t observations = 0;
    uint64_t stale_observations = 0;
    uint64_t duplicate_observations = 0;
    uint64_t identity_rejects = 0;
    uint64_t out_of_grid = 0;
    uint64_t queue_full = 0;
    uint64_t residual_outliers = 0;
    uint64_t holdover_entries = 0;
    uint64_t reacquire_entries = 0;
    uint64_t lock_entries = 0;
    uint64_t discontinuity_events = 0;
    uint64_t candidates = 0;
    uint64_t candidates_target = 0;
    uint64_t candidates_interference = 0;
    uint64_t candidates_unknown = 0;
    uint64_t period_updates = 0;
    uint64_t pre_guard_updates = 0;
    // Lock acquisition cost: slots between the first confirmed target packet
    // and entering LOCKED (0 while still verifying).
    uint64_t lock_slots = 0;
    bool lock_slots_known = false;
    double period_samples = 0.0;
    double period_ppm = 0.0;
    int64_t last_valid_origin = -1;
    uint64_t holdover_length = 0; // consecutive misses while in HOLDOVER
};

// ---------------------------------------------------------------------------
// Core
// ---------------------------------------------------------------------------
class TargetLockCore {
public:
    explicit TargetLockCore(const TargetLockConfig& cfg = TargetLockConfig())
        : cfg_(cfg)
    {
        sanitize_config();
        reset_acquire(/*new_generation=*/true);
    }

    void configure(const TargetLockConfig& cfg)
    {
        cfg_ = cfg;
        sanitize_config();
        reset_acquire(/*new_generation=*/true);
    }

    const TargetLockConfig& config() const { return cfg_; }

    TargetLockState state() const { return state_; }
    uint64_t generation() const { return generation_; }
    uint64_t epoch() const { return epoch_; }
    bool locked() const { return state_ == TargetLockState::Locked; }
    bool tracking() const
    {
        return state_ == TargetLockState::Verify ||
               state_ == TargetLockState::Locked ||
               state_ == TargetLockState::Holdover;
    }
    /** True when the scheduler should cut predicted windows. */
    bool scheduled() const { return tracking(); }
    double t0_exact() const { return t0_exact_; }
    double period_samples() const { return period_samples_; }
    double period_s() const
    {
        return cfg_.sample_rate > 0.0 ? period_samples_ / cfg_.sample_rate
                                      : cfg_.nominal_period_s;
    }
    int64_t last_valid_origin() const { return last_valid_origin_; }
    TargetLockStats stats() const;

    /** Predicted native origin of slot k (independent rounding per k). */
    int64_t predicted(uint64_t k) const
    {
        return predicted_start_sample(t0_exact_, period_samples_, k);
    }

    /**
     * Plan the first slot whose window is not yet fully in the past.
     * Scheduler thread only.  Returns false when no schedule is active.
     */
    bool next_plan(int64_t abs_cursor, TargetSlotPlan& out) const
    {
        if (!scheduled() || period_samples_ <= 0.0)
            return false;
        uint64_t k = next_future_slot(abs_cursor);
        out = make_plan(k);
        return true;
    }

    /** Recompute the plan for a specific slot under the CURRENT model. */
    TargetSlotPlan plan_for_slot(uint64_t k) const { return make_plan(k); }

    /**
     * Runtime pre-guard update (scheduler thread only).
     *
     * The pre-guard is the only geometry field a running receiver needs to
     * change: ACQUIRE/HOLDOVER/REACQUIRE must cover the whole autocorrelation
     * platform (the seed can precede the true packet start), while LOCKED only
     * has to cover the bounded first-SYNC ROI plus the resampler transient.
     *
     * The job buffer is allocated once for the widest geometry, so an update
     * may only move the window inside that budget.  Returns false and changes
     * nothing when `pre_guard` is zero, when the geometry is not valid, or when
     * `pre_guard + capture + post_guard` would exceed `window_capacity`.  The
     * caller must treat false as fatal: keeping the old guard would cut a
     * window whose start no longer matches the plan it reports.
     */
    bool set_pre_guard(size_t pre_guard)
    {
        if (pre_guard == 0 || !cfg_.geometry.valid() || cfg_.window_capacity == 0)
            return false;
        if (pre_guard + cfg_.geometry.capture + cfg_.geometry.post_guard >
            cfg_.window_capacity)
            return false;
        if (pre_guard == cfg_.geometry.pre_guard)
            return true;
        cfg_.geometry.pre_guard = pre_guard;
        ++stats_.pre_guard_updates;
        return true;
    }

    /** Currently effective pre-guard (native samples). */
    size_t pre_guard() const { return cfg_.geometry.pre_guard; }

    /** Largest window this core will ever cut (`window_capacity`, 0 = n/a). */
    size_t window_capacity() const { return cfg_.window_capacity; }

    /** Mark a plan as handed to the DSP queue. */
    void commit_plan(const TargetSlotPlan& plan);

    /** Record a terminal outcome for a slot (scheduler thread). */
    void note_slot_outcome(uint64_t slot_index, TargetSlotOutcome outcome);

    /**
     * Give up on a slot that can no longer be cut (history overrun, queue
     * full).  Records Expired and raises the scheduling floor so the same slot
     * is never planned twice.
     */
    void abandon_slot(uint64_t slot_index)
    {
        note_slot_outcome(slot_index, TargetSlotOutcome::Expired);
        if (slot_index + 1 > floor_slot_)
            floor_slot_ = slot_index + 1;
    }

    /**
     * Worker-thread safe: enqueue one observation.  Returns false when the
     * bounded queue is full (counted, never silently dropped).
     */
    bool push_observation(const TargetObservation& obs);

    /**
     * Scheduler thread: apply every queued observation in arrival order.
     * Returns the number applied.  Stale/duplicate records are counted and
     * ignored without touching the lock.
     */
    size_t drain_observations();

    /** Unexpected RX gap / time-base change invalidates the grid. */
    void note_time_discontinuity();

    /** Classify one autocorrelation candidate against the current lock. */
    CandidateVerdict classify_candidate(const InterferenceCandidate& c);

    /** Diagnostics: the last plan actually committed. */
    const TargetSlotPlan& last_plan() const { return last_plan_; }
    bool has_last_plan() const { return has_last_plan_; }

    /** True when the slot has a terminal (non-Planned/Submitted) record. */
    bool slot_terminal(uint64_t slot) const
    {
        auto it = slots_.find(slot);
        if (it == slots_.end())
            return false;
        switch (it->second.outcome) {
        case TargetSlotOutcome::Planned:
        case TargetSlotOutcome::Submitted:
            return false;
        default:
            return true;
        }
    }

    /** True when the slot's target decode succeeded (identity-valid). */
    bool slot_succeeded(uint64_t slot) const
    {
        auto it = slots_.find(slot);
        return it != slots_.end() && it->second.observed &&
               it->second.identity_ok;
    }

    /** Slot index inferred for an absolute native origin (nearest grid slot). */
    uint64_t slot_for(int64_t origin) const;

private:
    struct SlotRecord {
        TargetSlotOutcome outcome = TargetSlotOutcome::Planned;
        bool observed = false;
        bool identity_ok = false;
        int64_t observed_origin = -1;
        int code_index = -1;
    };

    void sanitize_config()
    {
        if (cfg_.sample_rate <= 0.0)
            cfg_.sample_rate = defaults::kNativeSampleRateHz;
        if (cfg_.nominal_period_s <= 0.0)
            cfg_.nominal_period_s = defaults::kQm35PacketIntervalS;
        if (cfg_.verify_observations < 2)
            cfg_.verify_observations = 2;
        if (cfg_.period_observations < 2)
            cfg_.period_observations = 2;
        if (cfg_.holdover_miss_count == 0)
            cfg_.holdover_miss_count = 1;
        if (cfg_.reacquire_miss_count <= cfg_.holdover_miss_count)
            cfg_.reacquire_miss_count = cfg_.holdover_miss_count + 1;
        if (cfg_.residual_outlier_count == 0)
            cfg_.residual_outlier_count = 1;
        if (cfg_.feedback_queue_capacity == 0)
            cfg_.feedback_queue_capacity = 1;
        if (cfg_.target_association_window <= 0)
            cfg_.target_association_window =
                static_cast<int64_t>(cfg_.geometry.pre_guard);
    }

    void reset_acquire(bool new_generation);
    void enter_verify(int64_t origin, uint64_t slot);
    void enter_locked();
    void enter_holdover();
    void enter_reacquire();
    void prune_slots();

    uint64_t next_future_slot(int64_t abs_cursor) const;
    TargetSlotPlan make_plan(uint64_t k) const;
    size_t current_guard_growth() const;

    bool identity_matches(const TargetObservation& obs) const;
    bool apply_observation(const TargetObservation& obs);
    bool fit_period_ok(double& out_t0, double& out_period) const;

    TargetLockConfig cfg_{};
    TargetLockState state_ = TargetLockState::Acquire;
    uint64_t generation_ = 0;
    uint64_t epoch_ = 0;

    double t0_exact_ = 0.0;
    double period_samples_ = 0.0;
    int64_t last_valid_origin_ = -1;
    uint64_t last_valid_slot_ = 0;
    bool have_slot_anchor_ = false;

    size_t verify_count_ = 0;
    size_t consecutive_misses_ = 0;
    size_t consecutive_residual_outliers_ = 0;
    uint64_t holdover_length_ = 0;
    uint64_t lock_slots_ = 0;
    bool lock_slots_known_ = false;
    uint64_t floor_slot_ = 0;

    // (slot, origin) learning buffer for the least-squares period fit.
    std::vector<std::pair<uint64_t, int64_t>> learn_;

    // Per-slot records, pruned to a bounded window behind the cursor.
    std::map<uint64_t, SlotRecord> slots_;

    TargetSlotPlan last_plan_{};
    bool has_last_plan_ = false;

    mutable std::mutex queue_mutex_;
    std::deque<TargetObservation> queue_;

    TargetLockStats stats_{};
};

} // namespace core
} // namespace openuwb
