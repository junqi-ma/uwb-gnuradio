/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * QM35 acquisition / lock / holdover / reacquire tracker.
 *
 * Spec v1 (盲 t0, known T=5 ms):
 *   UNLOCKED_ACQUIRE → CANDIDATE_VERIFY → PROVISIONAL_TRACK → LOCKED
 *                   ↘ HOLDOVER → REACQUIRE (neighborhood, then global energy)
 *
 * Identity is configurable PHY + FCS (no hard-coded MAC).  A code-9 fine
 * peak or energy-only burst is never enough to enter LOCKED.
 *
 * The live schedule lives only in the native 737.28 MHz sample domain.
 * Demod observations at 998.4 MHz must be mapped with the inverse of
 *   map_737_to_998(p) = round((p*65 + (taps-1)/2) / 48)
 * before they update t0/T.  A 998.4 index presented as native is rejected.
 */

#pragma once

#include <gnuradio/uwb/uwb_defaults.h>
#include <gnuradio/uwb/uwb_scheduled_extractor_core.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>

namespace gr {
namespace uwb {
namespace core {

inline constexpr double kNativeRateHz = defaults::kNativeSampleRateHz;
inline constexpr double kHostRate998Hz = defaults::kSampleRateHz;

/**
 * Inverse of the PDU 65/48 group-delay map
 *   out = round((in * 65 + gd) / 48)
 *   in  = round((out * 48 - gd) / 65)
 * when obs is 998.4e6 and native is 737.28e6.
 */
inline int64_t map_obs_sample_to_native(int64_t obs_sample,
                                        double obs_rate,
                                        double native_rate,
                                        double filter_delay)
{
    if (obs_sample < 0 || obs_rate <= 0.0 || native_rate <= 0.0)
        return obs_sample;
    if (std::abs(obs_rate - native_rate) <=
        1e-6 * std::max(obs_rate, native_rate))
        return obs_sample;

    const bool obs_998 =
        std::abs(obs_rate - kHostRate998Hz) <= 1e-6 * kHostRate998Hz;
    const bool nat_737 =
        std::abs(native_rate - kNativeRateHz) <= 1e-6 * kNativeRateHz;
    if (obs_998 && nat_737) {
        return static_cast<int64_t>(std::llround(
            (static_cast<double>(obs_sample) * 48.0 - filter_delay) / 65.0));
    }
    return static_cast<int64_t>(
        std::llround(static_cast<double>(obs_sample) * native_rate / obs_rate));
}

enum class Qm35LockState : int {
    UnlockedAcquire = 0,
    CandidateVerify = 1,
    ProvisionalTrack = 2,
    Locked = 3,
    Holdover = 4,
    Reacquire = 5
};

inline const char* qm35_lock_state_name(Qm35LockState s)
{
    switch (s) {
    case Qm35LockState::UnlockedAcquire:
        return "unlocked_acquire";
    case Qm35LockState::CandidateVerify:
        return "candidate_verify";
    case Qm35LockState::ProvisionalTrack:
        return "provisional";
    case Qm35LockState::Locked:
        return "locked";
    case Qm35LockState::Holdover:
        return "holdover";
    case Qm35LockState::Reacquire:
        return "reacquire";
    }
    return "unknown";
}

struct Qm35IdentitySpec {
    int code_index = 9;
    int preamble_repetitions = 64;
    std::string sfd_mode = "4z2";
    bool require_fcs_pass = true;
    bool require_status_success = true;
};

struct Qm35LockObservation {
    uint64_t packet_id = 0;
    uint64_t acquisition_epoch = 0;
    uint64_t schedule_generation = 0;
    uint64_t schedule_index = std::numeric_limits<uint64_t>::max();
    int64_t detected_start_sample = -1;
    double sample_rate = 0.0;
    double native_sample_rate = 0.0;
    double resample_filter_delay = 0.0;
    bool timing_ok = false;
    bool fcs_pass = false;
    std::string status;
    int code_index = -1;
    int preamble_repetitions = -1;
    std::string sfd_mode;
    bool has_generation = false;
    bool has_epoch = false;
    bool has_timing_ok = false;
    bool has_sample_rate = false;
    bool has_native_sample_rate = false;
    bool has_filter_delay = false;
};

enum class Qm35ObsAction : int {
    IgnoredStale = 0,
    RejectedIdentity = 1,
    RejectedUnmapped = 2,
    ConfirmedIdentity = 3,
    TimingUpdate = 4,
    TimingMiss = 5,
    Ignored = 6
};

struct Qm35AcquisitionConfig {
    double sample_rate = defaults::kNativeSampleRateHz;
    double nominal_interval_s = defaults::kQm35PacketIntervalS;
    size_t lock_observations = defaults::kLockObservations;
    size_t holdover_miss_count = defaults::kHoldoverMissCount;
    size_t reacquire_miss_count = defaults::kReacquireMissCount;
    double provisional_guard_us = defaults::kProvisionalGuardUs;
    double max_period_rel_err = 0.02;
    size_t neighborhood_slots = 4;
    Qm35IdentitySpec identity;
};

class Qm35AcquisitionTracker {
public:
    using State = Qm35LockState;
    using Action = Qm35ObsAction;

    explicit Qm35AcquisitionTracker(const Qm35AcquisitionConfig& cfg =
                                        Qm35AcquisitionConfig())
        : cfg_(cfg)
    {
        reset_acquire(/*new_epoch=*/true);
    }

    void configure(const Qm35AcquisitionConfig& cfg)
    {
        cfg_ = cfg;
        if (cfg_.lock_observations < 2)
            cfg_.lock_observations = 2;
        if (cfg_.holdover_miss_count == 0)
            cfg_.holdover_miss_count = 1;
        if (cfg_.reacquire_miss_count <= cfg_.holdover_miss_count)
            cfg_.reacquire_miss_count = cfg_.holdover_miss_count + 1;
        reset_acquire(/*new_epoch=*/true);
    }

    const Qm35AcquisitionConfig& config() const { return cfg_; }
    State state() const { return state_; }
    uint64_t generation() const { return generation_; }
    uint64_t epoch() const { return epoch_; }
    uint64_t stale_feedback() const { return stale_feedback_; }
    uint64_t unmapped_rejected() const { return unmapped_rejected_; }
    uint64_t identity_rejects() const { return identity_rejects_; }
    uint64_t timing_ok_count() const { return timing_ok_count_; }
    uint64_t consecutive_misses() const { return consecutive_misses_; }
    uint64_t neighborhood_tries() const { return neighborhood_tries_; }
    bool global_energy() const { return global_energy_; }
    bool identity_confirmed() const { return identity_confirmed_; }
    int64_t first_start_sample() const { return first_start_sample_; }
    double t0_exact() const { return t0_exact_; }
    double period_samples() const { return period_samples_; }
    double period_s() const
    {
        return cfg_.sample_rate > 0.0 ? period_samples_ / cfg_.sample_rate
                                      : cfg_.nominal_interval_s;
    }

    bool energy_path_active() const
    {
        return state_ == State::UnlockedAcquire ||
               state_ == State::CandidateVerify ||
               (state_ == State::Reacquire && global_energy_);
    }

    bool scheduled_path_active() const
    {
        return state_ == State::ProvisionalTrack ||
               state_ == State::Locked ||
               state_ == State::Holdover ||
               (state_ == State::Reacquire && !global_energy_);
    }

    const char* capture_mode_name() const
    {
        if (state_ == State::UnlockedAcquire ||
            state_ == State::CandidateVerify)
            return "acquisition";
        if (state_ == State::ProvisionalTrack)
            return "provisional";
        return "scheduled";
    }

    void note_candidate_emitted(int64_t native_start)
    {
        if (state_ == State::UnlockedAcquire ||
            state_ == State::CandidateVerify ||
            (state_ == State::Reacquire && global_energy_)) {
            state_ = State::CandidateVerify;
            pending_start_ = native_start;
        }
    }

    /**
     * Ingest one demod-style lock_obs.  `mapped_native` is the observation
     * already translated into the native 737.28 domain (or -1 if rejected
     * before mapping).  `used_mapping` is true when the 998.4 inverse map
     * (or a rate-ratio map) was applied.
     */
    Action on_lock_obs(const Qm35LockObservation& obs, int64_t mapped_native)
    {
        if (mapped_native < 0) {
            ++unmapped_rejected_;
            return Action::RejectedUnmapped;
        }
        // After a discontinuity (or any published generation that we have
        // invalidated), missing epoch/generation is stale — demod
        // schedule_feedback used to omit both, which would otherwise
        // re-confirm identity on the old native t0.
        if (require_obs_provenance_) {
            if (!obs.has_epoch || obs.acquisition_epoch != epoch_) {
                ++stale_feedback_;
                return Action::IgnoredStale;
            }
            if (!obs.has_generation || obs.schedule_generation != generation_) {
                ++stale_feedback_;
                return Action::IgnoredStale;
            }
        } else {
            if (obs.has_epoch && obs.acquisition_epoch != epoch_) {
                ++stale_feedback_;
                return Action::IgnoredStale;
            }
            if (obs.has_generation && obs.schedule_generation != generation_) {
                ++stale_feedback_;
                return Action::IgnoredStale;
            }
        }

        if (!identity_confirmed_) {
            if (!identity_matches(obs)) {
                ++identity_rejects_;
                // Keep scanning; a code-9 / FCS-fail candidate never locks.
                if (state_ == State::CandidateVerify)
                    state_ = State::UnlockedAcquire;
                pending_start_ = -1;
                return Action::RejectedIdentity;
            }
            confirm_identity(mapped_native);
            return Action::ConfirmedIdentity;
        }

        const bool timing_ok = !obs.has_timing_ok || obs.timing_ok;
        if (!timing_ok) {
            return note_miss();
        }
        return note_timing(obs.schedule_index, mapped_native);
    }

    Action note_miss()
    {
        if (!scheduled_path_active() && state_ != State::Locked &&
            state_ != State::Holdover && state_ != State::ProvisionalTrack)
            return Action::Ignored;
        ++consecutive_misses_;
        if (state_ == State::Locked &&
            consecutive_misses_ >= cfg_.holdover_miss_count) {
            state_ = State::Holdover;
        }
        if ((state_ == State::Holdover || state_ == State::Locked ||
             state_ == State::ProvisionalTrack) &&
            consecutive_misses_ >= cfg_.reacquire_miss_count) {
            enter_reacquire(/*neighborhood=*/true);
        } else if (state_ == State::Reacquire && !global_energy_) {
            ++neighborhood_tries_;
            if (neighborhood_tries_ >= cfg_.neighborhood_slots)
                enter_reacquire(/*neighborhood=*/false);
        }
        return Action::TimingMiss;
    }

    void note_discontinuity()
    {
        ++generation_;
        enter_reacquire(/*neighborhood=*/false);
        // Old sample-domain t0/T must not keep cutting windows.
        t0_exact_ = 0.0;
        period_samples_ = cfg_.nominal_interval_s * cfg_.sample_rate;
        first_start_sample_ = -1;
        identity_confirmed_ = false;
        pending_start_ = -1;
        timing_ok_count_ = 0;
        consecutive_misses_ = 0;
        require_obs_provenance_ = true;
    }

    void bump_generation() { ++generation_; }

    /** Next still-future schedule index from the current native cursor. */
    uint64_t next_future_k(uint64_t current_sample, size_t pre_guard) const
    {
        const double T = period_samples_;
        if (T <= 0.0)
            return 0;
        const double num = static_cast<double>(current_sample) +
                           static_cast<double>(pre_guard) - t0_exact_;
        if (num <= 0.0)
            return 0;
        const double k = std::ceil(num / T);
        return k > 0.0 ? static_cast<uint64_t>(k) : 0;
    }

    int64_t predicted(uint64_t k) const
    {
        return predicted_start_sample(t0_exact_, period_samples_, k);
    }

private:
    bool identity_matches(const Qm35LockObservation& obs) const
    {
        const auto& id = cfg_.identity;
        if (id.require_status_success) {
            if (obs.status != "success")
                return false;
        }
        if (id.require_fcs_pass && !obs.fcs_pass)
            return false;
        if (obs.code_index >= 0 && obs.code_index != id.code_index)
            return false;
        if (obs.preamble_repetitions >= 0 &&
            obs.preamble_repetitions != id.preamble_repetitions)
            return false;
        if (!obs.sfd_mode.empty() && obs.sfd_mode != id.sfd_mode)
            return false;
        // Require the identity fields that the demod actually sends.
        if (obs.status.empty() && id.require_status_success)
            return false;
        return true;
    }

    void confirm_identity(int64_t native_start)
    {
        identity_confirmed_ = true;
        first_start_sample_ = native_start;
        t0_exact_ = static_cast<double>(native_start);
        period_samples_ = cfg_.nominal_interval_s * cfg_.sample_rate;
        timing_ok_count_ = 1;
        consecutive_misses_ = 0;
        state_ = State::ProvisionalTrack;
        global_energy_ = false;
        neighborhood_tries_ = 0;
        ++generation_;
        lock_.enabled = true;
        lock_.min_observations = cfg_.lock_observations;
        lock_.max_period_rel_err = cfg_.max_period_rel_err;
        lock_.reset(t0_exact_, period_samples_);
        lock_.observe(0, native_start, period_samples_);
    }

    Action note_timing(uint64_t schedule_index, int64_t native_start)
    {
        consecutive_misses_ = 0;
        ++timing_ok_count_;
        if (lock_.enabled) {
            lock_.observe(schedule_index == std::numeric_limits<uint64_t>::max()
                              ? timing_ok_count_ - 1
                              : schedule_index,
                          native_start, period_samples_);
            if (lock_.period_samples > 0.0) {
                const double rel = std::abs(lock_.period_samples -
                                            period_samples_) /
                                   std::max(period_samples_, 1.0);
                if (rel <= cfg_.max_period_rel_err) {
                    t0_exact_ = lock_.t0_exact;
                    period_samples_ = lock_.period_samples;
                }
            }
        }
        if (state_ == State::Holdover ||
            (state_ == State::Reacquire && !global_energy_)) {
            state_ = State::Locked;
        }
        if (state_ == State::ProvisionalTrack &&
            timing_ok_count_ >= cfg_.lock_observations) {
            state_ = State::Locked;
        }
        return Action::TimingUpdate;
    }

    void enter_reacquire(bool neighborhood)
    {
        state_ = State::Reacquire;
        global_energy_ = !neighborhood;
        neighborhood_tries_ = neighborhood ? 0 : cfg_.neighborhood_slots;
        if (!neighborhood) {
            identity_confirmed_ = false;
            ++epoch_;
            ++generation_;
        } else {
            ++generation_;
        }
    }

    void reset_acquire(bool new_epoch)
    {
        state_ = State::UnlockedAcquire;
        identity_confirmed_ = false;
        global_energy_ = true;
        pending_start_ = -1;
        first_start_sample_ = -1;
        t0_exact_ = 0.0;
        period_samples_ = cfg_.nominal_interval_s * cfg_.sample_rate;
        timing_ok_count_ = 0;
        consecutive_misses_ = 0;
        neighborhood_tries_ = 0;
        if (new_epoch)
            ++epoch_;
        ++generation_;
        lock_.enabled = false;
        require_obs_provenance_ = false;
    }

    Qm35AcquisitionConfig cfg_{};
    State state_ = State::UnlockedAcquire;
    bool identity_confirmed_ = false;
    bool global_energy_ = true;
    int64_t pending_start_ = -1;
    int64_t first_start_sample_ = -1;
    double t0_exact_ = 0.0;
    double period_samples_ = 0.0;
    uint64_t generation_ = 0;
    uint64_t epoch_ = 0;
    uint64_t stale_feedback_ = 0;
    uint64_t unmapped_rejected_ = 0;
    uint64_t identity_rejects_ = 0;
    uint64_t timing_ok_count_ = 0;
    uint64_t consecutive_misses_ = 0;
    uint64_t neighborhood_tries_ = 0;
    bool require_obs_provenance_ = false;
    ScheduleLockTracker lock_;
};

/**
 * Decide whether a lock_obs must be mapped, used as native, or rejected.
 *
 * A 998.4-domain index presented as native (sample_rate == native but a
 * non-zero resample_filter_delay is attached) is rejected so the 998.4
 * number can never be written into the 737.28 schedule.
 */
inline bool classify_lock_obs_mapping(const Qm35LockObservation& obs,
                                      double native_rate,
                                      int64_t* out_native,
                                      bool* out_mapped)
{
    *out_native = -1;
    *out_mapped = false;
    if (obs.detected_start_sample < 0)
        return false;

    const double delay = obs.has_filter_delay ? obs.resample_filter_delay : 0.0;
    const double sample_rate =
        obs.has_sample_rate ? obs.sample_rate : 0.0;

    // Same-domain claim with a resample group delay: this is the 998.4
    // index being smuggled in as native.  Reject.
    if (sample_rate > 0.0 &&
        std::abs(sample_rate - native_rate) <=
            1e-6 * std::max(sample_rate, native_rate) &&
        delay != 0.0) {
        return false;
    }

    if (sample_rate > 0.0 &&
        std::abs(sample_rate - native_rate) >
            1e-6 * std::max(sample_rate, native_rate)) {
        *out_native = map_obs_sample_to_native(
            obs.detected_start_sample, sample_rate, native_rate, delay);
        *out_mapped = true;
        return *out_native >= 0;
    }

    // No sample_rate: if a native rate + delay are present, assume 998.4.
    if (sample_rate <= 0.0 && obs.has_native_sample_rate &&
        obs.has_filter_delay && delay != 0.0 &&
        std::abs(obs.native_sample_rate - native_rate) <=
            1e-6 * native_rate) {
        *out_native = map_obs_sample_to_native(
            obs.detected_start_sample, kHostRate998Hz, native_rate, delay);
        *out_mapped = true;
        return *out_native >= 0;
    }

    *out_native = obs.detected_start_sample;
    *out_mapped = false;
    return true;
}

} // namespace core
} // namespace uwb
} // namespace gr
