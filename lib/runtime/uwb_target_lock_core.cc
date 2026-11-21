/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Out-of-line implementations of the periodic-target lock core
 * (declarations in include/openuwb/runtime/target_lock_core.h).
 *
 * Moved bodies: the state-name tables, the ACQUIRE/VERIFY/LOCKED/HOLDOVER/
 * REACQUIRE transitions, the observation feedback queue (worker push /
 * scheduler drain), the least-squares (t0, period) fit, slot planning and
 * candidate classification, plus the aggregate statistics read-out.
 *
 * The header keeps the enums, structs, constants and the small accessors /
 * predicates (locked, tracking, scheduled, predicted, slot_terminal,
 * slot_succeeded, abandon_slot ...).
 */

#include <openuwb/runtime/target_lock_core.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <mutex>
#include <utility>
#include <vector>

namespace openuwb {
namespace core {

const char* target_lock_state_name(TargetLockState s)
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

const char* target_slot_outcome_name(TargetSlotOutcome o)
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

const char* candidate_class_name(CandidateClass c)
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

void TargetLockCore::reset_acquire(bool new_generation)
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

bool TargetLockCore::identity_matches(const TargetObservation& obs) const
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

size_t TargetLockCore::current_guard_growth() const
{
    if (state_ != TargetLockState::Holdover || consecutive_misses_ == 0)
        return 0;
    const size_t growth = cfg_.holdover_guard_growth * consecutive_misses_;
    return std::min(growth, cfg_.holdover_guard_cap);
}

TargetSlotPlan TargetLockCore::make_plan(uint64_t k) const
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

uint64_t TargetLockCore::slot_for(int64_t origin) const
{
    if (period_samples_ <= 0.0)
        return 0;
    const double k = (static_cast<double>(origin) - t0_exact_) / period_samples_;
    if (!(k > 0.0))
        return 0;
    return static_cast<uint64_t>(std::llround(k));
}

uint64_t TargetLockCore::next_future_slot(int64_t abs_cursor) const
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

void TargetLockCore::commit_plan(const TargetSlotPlan& plan)
{
    last_plan_ = plan;
    has_last_plan_ = true;
    ++stats_.expected_slots;
    ++stats_.submitted;
    slots_[plan.slot_index].outcome = TargetSlotOutcome::Submitted;
    prune_slots();
}

void TargetLockCore::prune_slots()
{
    // Keep a bounded history behind the scheduling floor; the lock only ever
    // reads records for slots it is still planning or classifying.
    const uint64_t keep_from =
        floor_slot_ > 64 ? floor_slot_ - 64 : 0;
    while (!slots_.empty() && slots_.begin()->first < keep_from)
        slots_.erase(slots_.begin());
}

void TargetLockCore::note_slot_outcome(uint64_t slot_index,
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

bool TargetLockCore::push_observation(const TargetObservation& obs)
{
    std::lock_guard<std::mutex> lock(queue_mutex_);
    if (queue_.size() >= cfg_.feedback_queue_capacity) {
        ++stats_.queue_full;
        return false;
    }
    queue_.push_back(obs);
    return true;
}

size_t TargetLockCore::drain_observations()
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

bool TargetLockCore::apply_observation(const TargetObservation& obs)
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

bool TargetLockCore::fit_period_ok(double& out_t0,
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

void TargetLockCore::enter_verify(int64_t origin, uint64_t slot)
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

void TargetLockCore::enter_locked()
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

void TargetLockCore::enter_holdover()
{
    if (state_ != TargetLockState::Holdover) {
        state_ = TargetLockState::Holdover;
        holdover_length_ = 0;
        ++stats_.holdover_entries;
    }
    ++holdover_length_;
    stats_.holdover_length = std::max(stats_.holdover_length, holdover_length_);
}

void TargetLockCore::enter_reacquire()
{
    ++stats_.reacquire_entries;
    reset_acquire(/*new_generation=*/true);
    state_ = TargetLockState::Reacquire;
}

void TargetLockCore::note_time_discontinuity()
{
    ++stats_.discontinuity_events;
    reset_acquire(/*new_generation=*/true);
    state_ = TargetLockState::Reacquire;
}

CandidateVerdict
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

TargetLockStats TargetLockCore::stats() const
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
} // namespace openuwb
