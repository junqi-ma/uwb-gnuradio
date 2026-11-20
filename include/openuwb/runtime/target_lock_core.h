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

namespace gr {
namespace uwb {
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

inline const char* target_lock_state_name(TargetLockState s)
{
    switch (s) {
    case TargetLockState::Acquire:
        return "acquire";
    case TargetLockState::Verify:
        return "verify";
    case TargetLockState::Locked:
        return "locked";
    case TargetLockState::Holdover:
        return "holdover";
    case TargetLockState::Reacquire:
        return "reacquire";
    }
    return "unknown";
}

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

inline const char* target_slot_outcome_name(TargetSlotOutcome o)
{
    switch (o) {
    case TargetSlotOutcome::Planned:
        return "planned";
    case TargetSlotOutcome::Submitted:
        return "submitted";
    case TargetSlotOutcome::DroppedQueue:
        return "dropped_queue";
    case TargetSlotOutcome::Expired:
        return "expired";
    case TargetSlotOutcome::NotDetected:
        return "not_detected";
    case TargetSlotOutcome::DecodeFailed:
        return "decode_failed";
    case TargetSlotOutcome::FcsFailed:
        return "fcs_failed";
    case TargetSlotOutcome::Success:
        return "success";
    }
    return "unknown";
}

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

inline const char* candidate_class_name(CandidateClass c)
{
    switch (c) {
    case CandidateClass::TargetAssociated:
        return "target_associated";
    case CandidateClass::Interference:
        return "interference";
    case CandidateClass::Unknown:
        return "unknown";
    }
    return "unknown";
}

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

// ===========================================================================
// Implementation
// ===========================================================================

inline void TargetLockCore::reset_acquire(bool new_generation)
{
    state_ = TargetLockState::Acquire;
    if (new_generation)
        ++generation_;
    ++epoch_;
    t0_exact_ = 0.0;
    period_samples_ = cfg_.period_samples();
    last_valid_origin_ = -1;
    last_valid_slot_ = 0;
    have_slot_anchor_ = false;
    verify_count_ = 0;
    consecutive_misses_ = 0;
    consecutive_residual_outliers_ = 0;
    holdover_length_ = 0;
    lock_slots_ = 0;
    lock_slots_known_ = false;
    floor_slot_ = 0;
    learn_.clear();
    slots_.clear();
    has_last_plan_ = false;
    last_plan_ = TargetSlotPlan{};
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        queue_.clear();
    }
}

inline bool TargetLockCore::identity_matches(const TargetObservation& obs) const
{
    if (!obs.timing_ok || !obs.sfd_ok)
        return false;
    if (cfg_.require_fcs && !obs.fcs_pass)
        return false;
    if (obs.code_index >= 0 && obs.code_index != cfg_.code_index)
        return false;
    if (obs.preamble_repetitions != 0 &&
        obs.preamble_repetitions != cfg_.preamble_repetitions)
        return false;
    if (!obs.sfd_mode.empty() && obs.sfd_mode != cfg_.sfd_mode)
        return false;
    return true;
}

inline size_t TargetLockCore::current_guard_growth() const
{
    if (state_ != TargetLockState::Holdover || consecutive_misses_ == 0)
        return 0;
    const size_t growth = cfg_.holdover_guard_growth * consecutive_misses_;
    return std::min(growth, cfg_.holdover_guard_cap);
}

inline TargetSlotPlan TargetLockCore::make_plan(uint64_t k) const
{
    TargetSlotPlan p;
    p.slot_index = k;
    p.generation = generation_;
    p.predicted_origin = predicted(k);
    const size_t growth = current_guard_growth();
    p.pre_guard = cfg_.geometry.pre_guard + growth;
    p.capture = cfg_.geometry.capture;
    p.post_guard = cfg_.geometry.post_guard + growth;
    p.widened = growth != 0;
    p.window_start = p.predicted_origin - static_cast<int64_t>(p.pre_guard);
    if (p.window_start < 0)
        p.window_start = 0;
    p.window_end = p.predicted_origin + static_cast<int64_t>(p.capture) +
                   static_cast<int64_t>(p.post_guard);
    return p;
}

inline uint64_t TargetLockCore::slot_for(int64_t origin) const
{
    if (period_samples_ <= 0.0)
        return 0;
    const double k = (static_cast<double>(origin) - t0_exact_) / period_samples_;
    if (!(k > 0.0))
        return 0;
    return static_cast<uint64_t>(std::llround(k));
}

inline uint64_t TargetLockCore::next_future_slot(int64_t abs_cursor) const
{
    if (period_samples_ <= 0.0)
        return 0;
    const int64_t pre = static_cast<int64_t>(cfg_.geometry.pre_guard);
    const double num =
        static_cast<double>(abs_cursor) + static_cast<double>(pre) - t0_exact_;
    if (num <= 0.0)
        return 0;
    uint64_t k = static_cast<uint64_t>(std::ceil(num / period_samples_));
    if (k < floor_slot_)
        k = floor_slot_;
    // Skip slots whose window has already closed (they are counted as expired
    // by the caller through note_slot_outcome).
    for (int guard = 0; guard < 8; ++guard) {
        const TargetSlotPlan p = make_plan(k);
        if (p.window_end > abs_cursor)
            break;
        ++k;
    }
    return k;
}

inline void TargetLockCore::commit_plan(const TargetSlotPlan& plan)
{
    last_plan_ = plan;
    has_last_plan_ = true;
    ++stats_.expected_slots;
    ++stats_.submitted;
    slots_[plan.slot_index].outcome = TargetSlotOutcome::Submitted;
    prune_slots();
}

inline void TargetLockCore::prune_slots()
{
    // Keep a bounded history behind the scheduling floor; the lock only ever
    // reads records for slots it is still planning or classifying.
    const uint64_t keep_from =
        floor_slot_ > 64 ? floor_slot_ - 64 : 0;
    while (!slots_.empty() && slots_.begin()->first < keep_from)
        slots_.erase(slots_.begin());
}

inline void TargetLockCore::note_slot_outcome(uint64_t slot_index,
                                              TargetSlotOutcome outcome)
{
    SlotRecord& rec = slots_[slot_index];
    // Never downgrade a terminal result to a weaker one.
    if (rec.outcome == TargetSlotOutcome::Success)
        return;
    rec.outcome = outcome;
    switch (outcome) {
    case TargetSlotOutcome::DroppedQueue:
        ++stats_.dropped_queue;
        break;
    case TargetSlotOutcome::Expired:
        ++stats_.expired;
        break;
    case TargetSlotOutcome::NotDetected:
        ++stats_.not_detected;
        break;
    case TargetSlotOutcome::DecodeFailed:
        ++stats_.decode_failed;
        break;
    case TargetSlotOutcome::FcsFailed:
        ++stats_.fcs_failed;
        break;
    case TargetSlotOutcome::Success:
        ++stats_.fcs_ok;
        break;
    default:
        break;
    }
}

inline bool TargetLockCore::push_observation(const TargetObservation& obs)
{
    std::lock_guard<std::mutex> lock(queue_mutex_);
    if (queue_.size() >= cfg_.feedback_queue_capacity) {
        ++stats_.queue_full;
        return false;
    }
    queue_.push_back(obs);
    return true;
}

inline size_t TargetLockCore::drain_observations()
{
    std::deque<TargetObservation> local;
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        local.swap(queue_);
    }
    size_t applied = 0;
    for (const auto& obs : local) {
        if (apply_observation(obs))
            ++applied;
    }
    return applied;
}

inline bool TargetLockCore::apply_observation(const TargetObservation& obs)
{
    ++stats_.observations;
    // Provenance: only the current generation may move the lock.
    if (obs.schedule_generation != generation_) {
        ++stats_.stale_observations;
        return false;
    }
    // An acquisition-path window has no slot index yet.  Once the grid exists
    // the slot is inferred from the observed origin; before that it defines
    // slot 0.
    uint64_t slot = 0;
    if (obs.slot_index == std::numeric_limits<uint64_t>::max()) {
        if (state_ == TargetLockState::Acquire ||
            state_ == TargetLockState::Reacquire || obs.observed_origin < 0)
            slot = 0;
        else
            slot = slot_for(obs.observed_origin);
    } else {
        slot = obs.slot_index;
    }
    if (obs.observed_origin < 0) {
        // A failed/absent detection is a miss, never a timing update.
        if (scheduled()) {
            ++consecutive_misses_;
            if (state_ == TargetLockState::Locked &&
                consecutive_misses_ >= cfg_.holdover_miss_count) {
                enter_holdover();
            } else if (consecutive_misses_ >= cfg_.reacquire_miss_count) {
                enter_reacquire();
            }
        }
        note_slot_outcome(slot, TargetSlotOutcome::NotDetected);
        return false;
    }

    if (!identity_matches(obs)) {
        ++stats_.identity_rejects;
        // An identity mismatch inside a target slot is a decode failure of the
        // target, not evidence for a different grid.
        if (scheduled()) {
            ++consecutive_misses_;
            if (state_ == TargetLockState::Locked &&
                consecutive_misses_ >= cfg_.holdover_miss_count) {
                enter_holdover();
            } else if (consecutive_misses_ >= cfg_.reacquire_miss_count) {
                enter_reacquire();
            }
        }
        note_slot_outcome(slot, obs.fcs_pass ? TargetSlotOutcome::DecodeFailed
                                             : TargetSlotOutcome::FcsFailed);
        return false;
    }

    // Duplicate slot: keep the first applied result so worker completion order
    // cannot re-order the air timeline.
    auto it = slots_.find(slot);
    if (it != slots_.end() && it->second.observed) {
        ++stats_.duplicate_observations;
        return false;
    }

    const int64_t origin = obs.observed_origin;
    if (state_ == TargetLockState::Acquire ||
        state_ == TargetLockState::Reacquire) {
        // First identity-valid target packet (re)seeds the slot grid.
        enter_verify(origin, slot);
        return true;
    }

    // Same-grid guard: a slot further than the allowed gap from the last valid
    // one is not this target's grid.  Never let it re-anchor the lock.
    const size_t allowed_gap = std::max(cfg_.max_slot_gap,
                                       cfg_.reacquire_miss_count);
    if (have_slot_anchor_) {
        const uint64_t gap = slot > last_valid_slot_
                                 ? slot - last_valid_slot_
                                 : last_valid_slot_ - slot;
        if (gap > allowed_gap) {
            ++stats_.out_of_grid;
            return false;
        }
    }

    // Residual against the current model.
    const double pred =
        t0_exact_ + static_cast<double>(slot) * period_samples_;
    const double resid = static_cast<double>(origin) - pred;
    if (state_ == TargetLockState::Locked ||
        state_ == TargetLockState::Holdover) {
        if (std::abs(resid) > cfg_.residual_limit_samples) {
            ++stats_.residual_outliers;
            ++consecutive_residual_outliers_;
            if (consecutive_residual_outliers_ >=
                cfg_.residual_outlier_count) {
                enter_reacquire();
                return false;
            }
        } else {
            consecutive_residual_outliers_ = 0;
        }
    }

    // Accept: refresh the learning buffer and the fit.
    consecutive_misses_ = 0;
    holdover_length_ = 0;
    ++verify_count_;
    last_valid_origin_ = origin;
    last_valid_slot_ = slot;
    have_slot_anchor_ = true;
    {
        SlotRecord& rec = slots_[slot];
        rec.observed = true;
        rec.identity_ok = true;
        rec.observed_origin = origin;
        rec.code_index = obs.code_index;
        rec.outcome = TargetSlotOutcome::Success;
    }
    ++stats_.fcs_ok;

    // Replace a duplicate learning entry for the same slot, then bound the
    // buffer to the most recent `period_observations`.
    bool replaced = false;
    for (auto& e : learn_) {
        if (e.first == slot) {
            e.second = origin;
            replaced = true;
            break;
        }
    }
    if (!replaced)
        learn_.emplace_back(slot, origin);
    if (learn_.size() > cfg_.period_observations)
        learn_.erase(learn_.begin());

    double fit_t0 = 0.0;
    double fit_period = 0.0;
    if (fit_period_ok(fit_t0, fit_period)) {
        const double rel =
            std::abs(fit_period - cfg_.period_samples()) /
            std::max(1.0, cfg_.period_samples());
        if (rel <= cfg_.max_period_rel_err) {
            t0_exact_ = fit_t0;
            period_samples_ = fit_period;
            ++stats_.period_updates;
        }
    }

    if (state_ == TargetLockState::Verify &&
        verify_count_ >= cfg_.verify_observations) {
        enter_locked();
    } else if (state_ == TargetLockState::Holdover) {
        state_ = TargetLockState::Locked;
    }
    return true;
}

inline bool TargetLockCore::fit_period_ok(double& out_t0,
                                          double& out_period) const
{
    const size_t n = learn_.size();
    if (n < 2)
        return false;
    long double sk = 0.0L, sd = 0.0L, skk = 0.0L, skd = 0.0L;
    for (const auto& e : learn_) {
        const long double k = static_cast<long double>(e.first);
        const long double d = static_cast<long double>(e.second);
        sk += k;
        sd += d;
        skk += k * k;
        skd += k * d;
    }
    const long double nn = static_cast<long double>(n);
    const long double den = nn * skk - sk * sk;
    if (std::abs(den) < 1.0e-18L)
        return false;
    const long double period = (nn * skd - sk * sd) / den;
    if (!(period > 0.0L))
        return false;
    const long double t0 = (sd - period * sk) / nn;
    out_period = static_cast<double>(period);
    out_t0 = static_cast<double>(t0);
    return std::isfinite(out_period) && std::isfinite(out_t0);
}

inline void TargetLockCore::enter_verify(int64_t origin, uint64_t slot)
{
    state_ = TargetLockState::Verify;
    // Deliberately NOT bumping generation_: VERIFY does not invalidate any
    // plan that was already issued (there were none before the seed), and the
    // acquisition windows that seeded the grid must stay usable so the
    // VERIFY -> LOCKED convergence can complete from the packets already in
    // flight.  REACQUIRE and a time discontinuity DO bump it.
    period_samples_ = cfg_.period_samples();
    // Anchor the grid so that `slot` sits at `origin`; slot 0 is then the
    // extrapolated origin of the first slot.  This keeps `predicted(k)` exact
    // even when the first accepted observation arrives out of order.
    t0_exact_ = static_cast<double>(origin) -
                static_cast<double>(slot) * period_samples_;
    last_valid_origin_ = origin;
    last_valid_slot_ = slot;
    have_slot_anchor_ = true;
    verify_count_ = 1;
    consecutive_misses_ = 0;
    consecutive_residual_outliers_ = 0;
    holdover_length_ = 0;
    lock_slots_ = 0;
    lock_slots_known_ = false;
    floor_slot_ = 0;
    learn_.clear();
    learn_.emplace_back(slot, origin);
    slots_.clear();
    SlotRecord rec;
    rec.observed = true;
    rec.identity_ok = true;
    rec.observed_origin = origin;
    rec.code_index = cfg_.code_index;
    rec.outcome = TargetSlotOutcome::Success;
    slots_[slot] = rec;
    ++stats_.fcs_ok;
}

inline void TargetLockCore::enter_locked()
{
    const bool was_tracking = state_ == TargetLockState::Verify;
    state_ = TargetLockState::Locked;
    consecutive_misses_ = 0;
    consecutive_residual_outliers_ = 0;
    holdover_length_ = 0;
    ++stats_.lock_entries;
    if (was_tracking && !lock_slots_known_) {
        lock_slots_ = verify_count_;
        lock_slots_known_ = true;
    }
}

inline void TargetLockCore::enter_holdover()
{
    if (state_ != TargetLockState::Holdover) {
        state_ = TargetLockState::Holdover;
        holdover_length_ = 0;
        ++stats_.holdover_entries;
    }
    ++holdover_length_;
    stats_.holdover_length = std::max(stats_.holdover_length, holdover_length_);
}

inline void TargetLockCore::enter_reacquire()
{
    ++stats_.reacquire_entries;
    reset_acquire(/*new_generation=*/true);
    state_ = TargetLockState::Reacquire;
}

inline void TargetLockCore::note_time_discontinuity()
{
    ++stats_.discontinuity_events;
    reset_acquire(/*new_generation=*/true);
    state_ = TargetLockState::Reacquire;
}

inline CandidateVerdict
TargetLockCore::classify_candidate(const InterferenceCandidate& c)
{
    CandidateVerdict v;
    ++stats_.candidates;
    if (c.start_sample < 0) {
        v.cls = CandidateClass::Unknown;
        ++stats_.candidates_unknown;
        return v;
    }
    if (!scheduled() || period_samples_ <= 0.0) {
        // No grid: a confirmed different code is still interference.
        if (c.template_confirmed && c.code_index >= 0 &&
            c.code_index != cfg_.code_index) {
            v.cls = CandidateClass::Interference;
            ++stats_.candidates_interference;
        } else {
            v.cls = CandidateClass::Unknown;
            ++stats_.candidates_unknown;
        }
        return v;
    }

    const uint64_t k = slot_for(c.start_sample);
    v.slot_index = k;
    v.offset_from_target = c.start_sample - predicted(k);
    const int64_t assoc = cfg_.target_association_window;

    const bool near_target = std::abs(v.offset_from_target) <= assoc;
    // A confirmed different code is interference even inside the target
    // window: overlapping interference must not be masked by the target.
    if (c.template_confirmed && c.code_index >= 0 &&
        c.code_index != cfg_.code_index) {
        v.cls = CandidateClass::Interference;
        ++stats_.candidates_interference;
        return v;
    }
    if (!near_target) {
        v.cls = c.template_confirmed ? CandidateClass::Interference
                                     : CandidateClass::Unknown;
        if (v.cls == CandidateClass::Interference)
            ++stats_.candidates_interference;
        else
            ++stats_.candidates_unknown;
        return v;
    }

    // Near the predicted time: it is the target's own response when that slot's
    // target window is already scheduled (result still pending) or when the
    // target decode actually succeeded.  A slot whose decode already FAILED
    // stays Unknown so an overlapping interferer is never silently discarded.
    auto it = slots_.find(k);
    if (it != slots_.end()) {
        if (it->second.observed && it->second.identity_ok) {
            v.cls = CandidateClass::TargetAssociated;
            ++stats_.candidates_target;
            return v;
        }
        if (it->second.outcome == TargetSlotOutcome::Submitted) {
            // Planned target window covers this response; association is
            // confirmed later from the slot's decode result.
            v.cls = CandidateClass::TargetAssociated;
            ++stats_.candidates_target;
            return v;
        }
    }
    v.cls = CandidateClass::Unknown;
    ++stats_.candidates_unknown;
    return v;
}

inline TargetLockStats TargetLockCore::stats() const
{
    TargetLockStats s = stats_;
    s.period_samples = period_samples_;
    const double nominal = cfg_.period_samples();
    s.period_ppm = nominal > 0.0
                       ? (period_samples_ - nominal) / nominal * 1.0e6
                       : 0.0;
    s.last_valid_origin = last_valid_origin_;
    s.lock_slots_known = lock_slots_known_;
    s.lock_slots = lock_slots_;
    s.holdover_length = holdover_length_;
    return s;
}

} // namespace core
} // namespace uwb
} // namespace gr
