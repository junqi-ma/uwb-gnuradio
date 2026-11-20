/*
 * QA for the native periodic-target lock core and the LockedBounded
 * synchronization strategy.
 *
 * These are the "math/state" acceptance items of
 * docs/performance/X410_目标周期锁定与随机干扰双路径修改规划_2026-11-15.md §10.1:
 *   - non-integer period and long-horizon drift
 *   - missed slots, out-of-order feedback, stale generations
 *   - same-code false target, low-correlation start SYNC
 *   - integer coordinate and resample group-delay mapping
 *   - LockedBounded really does not fall back to a wide scan
 *   - holdover widening, reacquire, interference classification
 */
#define BOOST_TEST_MODULE qa_uwb_target_lock_core
#include <boost/test/unit_test.hpp>

#include <gnuradio/uwb/uwb_demod_core.h>
#include <gnuradio/uwb/uwb_qm35_acquisition_tracker.h>
#include <gnuradio/uwb/uwb_target_lock_core.h>
#include <gnuradio/uwb/uwb_timed_rx_plan.h>

#include <cmath>
#include <complex>
#include <cstdint>
#include <limits>
#include <vector>

namespace {

using openuwb::core::CandidateClass;
using openuwb::core::InterferenceCandidate;
using openuwb::core::TargetLockConfig;
using openuwb::core::TargetLockCore;
using openuwb::core::TargetLockState;
using openuwb::core::TargetObservation;
using openuwb::core::TargetSlotOutcome;
using openuwb::core::TargetSlotPlan;

constexpr double kFs = 737280000.0;
constexpr double kNominalPeriodS = 5.0e-3;
// The measured live period from the planning document: 4.999864 ms.
constexpr double kTruePeriodS = 4.999864e-3;
constexpr double kTruePeriodSamples = kTruePeriodS * kFs;

TargetLockConfig make_config()
{
    TargetLockConfig cfg;
    cfg.sample_rate = kFs;
    cfg.nominal_period_s = kNominalPeriodS;
    cfg.verify_observations = 3;
    cfg.holdover_miss_count = 3;
    cfg.reacquire_miss_count = 8;
    cfg.period_observations = 16;
    cfg.residual_limit_samples = 512.0;
    cfg.residual_outlier_count = 3;
    cfg.holdover_guard_growth = 2048;
    cfg.holdover_guard_cap = 32768;
    cfg.feedback_queue_capacity = 64;
    cfg.code_index = 10;
    cfg.preamble_repetitions = 256;
    cfg.sfd_mode = "4z2";
    cfg.require_fcs = true;
    cfg.geometry = openuwb::core::target_window_geometry(
        260096, 8128, 21504, 154624, 1024, 36864, 8192);
    // Job buffer for the widest geometry plus the holdover cap.  The QA spans
    // are the raw work-grid numbers, so this is larger than the live app's
    // 450000-sample buffer; only the capacity contract matters here.
    cfg.window_capacity = 600000;
    return cfg;
}

int64_t true_origin(uint64_t k, int64_t t0 = 1000000)
{
    return static_cast<int64_t>(
        std::llround(static_cast<double>(t0) +
                     static_cast<double>(k) * kTruePeriodSamples));
}

TargetObservation good_obs(TargetLockCore& lock, uint64_t slot, int64_t origin)
{
    TargetObservation o;
    o.packet_id = slot + 1;
    o.schedule_generation = lock.generation();
    o.slot_index = slot;
    o.window_origin = origin - 36864;
    o.predicted_origin = lock.predicted(slot);
    o.observed_origin = origin;
    o.residual = origin - o.predicted_origin;
    o.timing_ok = true;
    o.sfd_ok = true;
    o.phr_ok = true;
    o.fcs_pass = true;
    o.code_index = 10;
    o.preamble_repetitions = 256;
    o.sfd_mode = "4z2";
    o.quality = 0.9f;
    o.from_target_window = true;
    return o;
}

void feed(TargetLockCore& lock, const TargetObservation& o)
{
    BOOST_REQUIRE(lock.push_observation(o));
    lock.drain_observations();
}

// ---------------------------------------------------------------------------
// Synthetic preamble for the LockedBounded strategy tests.
// ---------------------------------------------------------------------------
std::vector<std::complex<float>> make_real_template(size_t L)
{
    std::vector<std::complex<float>> t(L);
    uint32_t prng = 0x6d2b79f5U;
    for (size_t k = 0; k < L; ++k) {
        prng ^= prng << 13;
        prng ^= prng >> 17;
        prng ^= prng << 5;
        t[k] = std::complex<float>((prng & 1U) ? 1.0f : -1.0f, 0.0f);
    }
    return t;
}

std::vector<std::complex<float>> make_preamble_window(
    const std::vector<std::complex<float>>& tmpl,
    size_t start,
    size_t reps,
    size_t n)
{
    std::vector<std::complex<float>> rx(n, std::complex<float>(0.0f, 0.0f));
    const size_t L = tmpl.size();
    for (size_t r = 0; r < reps; ++r) {
        for (size_t k = 0; k < L; ++k) {
            const size_t i = start + r * L + k;
            if (i < n)
                rx[i] = tmpl[k];
        }
    }
    return rx;
}

} // namespace

// ===========================================================================
// 1. Non-integer period + long-horizon drift
// ===========================================================================
BOOST_AUTO_TEST_CASE(non_integer_period_learns_and_does_not_drift)
{
    auto cfg = make_config();
    cfg.period_observations = 32;
    TargetLockCore lock(cfg);
    BOOST_REQUIRE_EQUAL(static_cast<int>(lock.state()),
                        static_cast<int>(TargetLockState::Acquire));
    BOOST_REQUIRE(!lock.scheduled());

    for (uint64_t k = 0; k < 32; ++k)
        feed(lock, good_obs(lock, k, true_origin(k)));

    BOOST_REQUIRE_EQUAL(static_cast<int>(lock.state()),
                        static_cast<int>(TargetLockState::Locked));
    // The learned period must match the true non-integer period.
    BOOST_CHECK_SMALL(lock.period_samples() - kTruePeriodSamples, 0.05);
    // And the nominal 5 ms must be visibly rejected.
    BOOST_CHECK_GT(std::abs(lock.period_samples() - kNominalPeriodS * kFs), 50.0);

    // Prediction stays usable over a moderate horizon (inside the ±256 ROI).
    BOOST_CHECK_LE(std::abs(lock.predicted(1000) - true_origin(1000)), 64);

    // Over a very long horizon the residual is bounded by the integer
    // quantisation of the observations, and is far better than a cumulative
    // rounded-integer grid (which is what the contract forbids).
    const int64_t learned_err =
        std::abs(lock.predicted(10000) - true_origin(10000));
    const int64_t cumulative =
        1000000 + static_cast<int64_t>(10000) *
                      static_cast<int64_t>(std::llround(kTruePeriodSamples));
    const int64_t cumulative_err = std::abs(cumulative - true_origin(10000));
    const int64_t nominal_err = std::abs(
        1000000 + static_cast<int64_t>(10000) *
                      static_cast<int64_t>(std::llround(kNominalPeriodS * kFs)) -
        true_origin(10000));
    BOOST_CHECK_LT(learned_err, 500);
    BOOST_CHECK_GT(cumulative_err, 2000);
    BOOST_CHECK_GT(nominal_err, 900000);
    BOOST_CHECK_LT(learned_err * 5, cumulative_err);

    const auto st = lock.stats();
    BOOST_CHECK_EQUAL(st.lock_entries, 1u);
    BOOST_CHECK_EQUAL(st.reacquire_entries, 0u);
    BOOST_CHECK_GT(st.period_updates, 0u);
    BOOST_CHECK_GT(st.fcs_ok, 0u);
}

// ===========================================================================
// 2. A single missed slot is still the same grid
// ===========================================================================
BOOST_AUTO_TEST_CASE(missing_slot_does_not_restart_the_period)
{
    TargetLockCore lock(make_config());
    for (uint64_t k = 0; k < 12; ++k) {
        if (k == 4 || k == 5)
            continue; // two consecutive missing packets = one missed slot pair
        feed(lock, good_obs(lock, k, true_origin(k)));
    }
    BOOST_REQUIRE_EQUAL(static_cast<int>(lock.state()),
                        static_cast<int>(TargetLockState::Locked));
    BOOST_CHECK_SMALL(lock.period_samples() - kTruePeriodSamples, 1.0);
    BOOST_CHECK_EQUAL(lock.stats().reacquire_entries, 0u);
    BOOST_CHECK_LE(std::abs(lock.predicted(12) - true_origin(12)), 1);
}

// ===========================================================================
// 3. Out-of-order feedback and duplicate slots
// ===========================================================================
BOOST_AUTO_TEST_CASE(out_of_order_feedback_is_order_independent)
{
    TargetLockCore ordered(make_config());
    TargetLockCore reversed(make_config());
    const uint64_t n = 12;
    for (uint64_t k = 0; k < n; ++k)
        feed(ordered, good_obs(ordered, k, true_origin(k)));
    // Reverse arrival order: worker completion order is NOT air order.
    for (uint64_t i = 0; i < n; ++i) {
        const uint64_t k = n - 1 - i;
        feed(reversed, good_obs(reversed, k, true_origin(k)));
    }
    BOOST_REQUIRE_EQUAL(static_cast<int>(ordered.state()),
                        static_cast<int>(reversed.state()));
    BOOST_CHECK_SMALL(reversed.period_samples() - kTruePeriodSamples, 1.0);
    BOOST_CHECK_LE(std::abs(reversed.predicted(50) - ordered.predicted(50)), 1);

    // A duplicate slot must not move the lock a second time.
    const double before = reversed.period_samples();
    const uint64_t updates_before = reversed.stats().period_updates;
    feed(reversed, good_obs(reversed, 3, true_origin(3)));
    BOOST_CHECK_EQUAL(reversed.stats().period_updates, updates_before);
    BOOST_CHECK_EQUAL(reversed.period_samples(), before);
    BOOST_CHECK_GT(reversed.stats().duplicate_observations, 0u);
}

// ===========================================================================
// 4. Stale generation feedback is ignored
// ===========================================================================
BOOST_AUTO_TEST_CASE(stale_generation_is_rejected)
{
    TargetLockCore lock(make_config());
    for (uint64_t k = 0; k < 6; ++k)
        feed(lock, good_obs(lock, k, true_origin(k)));
    const double period = lock.period_samples();
    const int64_t t0 = std::llround(lock.t0_exact());
    const uint64_t stale_before = lock.stats().stale_observations;

    auto o = good_obs(lock, 6, true_origin(6) + 100000);
    o.schedule_generation = lock.generation() - 1; // old lock
    feed(lock, o);

    BOOST_CHECK_EQUAL(lock.stats().stale_observations, stale_before + 1);
    BOOST_CHECK_EQUAL(lock.period_samples(), period);
    BOOST_CHECK_EQUAL(std::llround(lock.t0_exact()), t0);
}

// ===========================================================================
// 5. Same-code / different-code false target
// ===========================================================================
BOOST_AUTO_TEST_CASE(identity_mismatch_never_moves_the_lock)
{
    TargetLockCore lock(make_config());
    for (uint64_t k = 0; k < 6; ++k)
        feed(lock, good_obs(lock, k, true_origin(k)));
    const double period = lock.period_samples();
    const int64_t t0 = std::llround(lock.t0_exact());
    const uint64_t rejects_before = lock.stats().identity_rejects;

    auto o = good_obs(lock, 6, true_origin(6) + 5000);
    o.code_index = 9; // a different preamble code
    feed(lock, o);

    BOOST_CHECK_EQUAL(lock.stats().identity_rejects, rejects_before + 1);
    BOOST_CHECK_EQUAL(lock.period_samples(), period);
    BOOST_CHECK_EQUAL(std::llround(lock.t0_exact()), t0);

    // FCS failure is also not identity.
    auto f = good_obs(lock, 7, true_origin(7));
    f.fcs_pass = false;
    feed(lock, f);
    BOOST_CHECK_EQUAL(lock.stats().identity_rejects, rejects_before + 2);
    BOOST_CHECK_EQUAL(std::llround(lock.t0_exact()), t0);
}

// ===========================================================================
// 6. Low-correlation start SYNC is a miss, not a timing update
// ===========================================================================
BOOST_AUTO_TEST_CASE(timing_failure_is_a_miss)
{
    TargetLockCore lock(make_config());
    for (uint64_t k = 0; k < 6; ++k)
        feed(lock, good_obs(lock, k, true_origin(k)));
    const int64_t t0 = std::llround(lock.t0_exact());

    auto o = good_obs(lock, 6, true_origin(6));
    o.timing_ok = false;   // startup SYNC below the verification threshold
    o.sfd_ok = false;
    o.observed_origin = -1; // no detection
    feed(lock, o);

    BOOST_CHECK_EQUAL(std::llround(lock.t0_exact()), t0);
    BOOST_CHECK_GT(lock.stats().not_detected, 0u);
}

// ===========================================================================
// 7. Integer coordinate + resample group-delay mapping
// ===========================================================================
BOOST_AUTO_TEST_CASE(native_work_coordinate_mapping_round_trips)
{
    const double gd = 1039.0; // (T-1)/2 for the quality_minorder 65/48 filter
    for (int64_t p = 0; p < 200000; p += 977) {
        const int64_t work = static_cast<int64_t>(
            std::llround((static_cast<double>(p) * 65.0 + gd) / 48.0));
        const int64_t native = openuwb::core::map_obs_sample_to_native(
            work, 998400000.0, kFs, gd);
        BOOST_CHECK_LE(std::abs(native - p), 1);
    }
    // A 998.4-domain index presented as native is not silently accepted by the
    // tracker's classifier when a group delay is attached.
    openuwb::core::Qm35LockObservation obs;
    obs.detected_start_sample = 123456;
    obs.has_sample_rate = true;
    obs.sample_rate = kFs;
    obs.has_filter_delay = true;
    obs.resample_filter_delay = gd;
    int64_t mapped = -1;
    bool used = false;
    BOOST_CHECK(!openuwb::core::classify_lock_obs_mapping(obs, kFs, &mapped, &used));
}

// ===========================================================================
// 8. LockedBounded never escalates to a wide scan
// ===========================================================================
BOOST_AUTO_TEST_CASE(locked_bounded_never_uses_a_wide_scan)
{
    const size_t L = 1016;
    const size_t reps = 256;
    const size_t start = 5000;
    const size_t n = start + reps * L + 4096;
    const auto tmpl = make_real_template(L);
    const auto rx = make_preamble_window(tmpl, start, reps, n);

    openuwb::demod::Qm35825Profile p;
    p.code_index = 10;
    p.preamble_repetitions = reps;
    p.sfd_mode = "4z2";
    p.sync_strategy = openuwb::demod::SyncStrategy::LockedBounded;
    p.locked_roi_samples = 256;
    p.locked_allow_backtrack = false;

    openuwb::demod::core::DemodScratch scratch;
    scratch.reserve(n);

    openuwb::demod::TimingResult ok;
    // Seed 200 samples off the true origin: inside the ±256 ROI.
    BOOST_REQUIRE(openuwb::demod::core::stage_timing(
        rx.data(), n, p, tmpl, int64_t(start) + 200, ok, scratch));
    BOOST_CHECK_EQUAL(ok.bounded_attempts, 1u);
    BOOST_CHECK_EQUAL(ok.bounded_success, 1u);
    BOOST_CHECK_EQUAL(ok.bounded_failures, 0u);
    // The contract: no legacy scan, no backtrack, and the ROI is bounded.
    BOOST_CHECK_EQUAL(ok.bounded_legacy_scans, 0u);
    BOOST_CHECK_EQUAL(ok.backtrack_symbols, 0u);
    BOOST_CHECK_EQUAL(ok.bounded_roi_samples, 256u);
    BOOST_CHECK_LE(std::abs(ok.preamble_start_sample - int64_t(start)), 2);

    // Seed half a SYNC symbol away: no SYNC start lies inside the ±256 ROI, so
    // this is an explicit tracking failure, still with no wide scan.
    openuwb::demod::TimingResult miss;
    BOOST_CHECK(!openuwb::demod::core::stage_timing(
        rx.data(), n, p, tmpl, int64_t(start) + 508, miss, scratch));
    BOOST_CHECK_EQUAL(miss.bounded_attempts, 1u);
    BOOST_CHECK_EQUAL(miss.bounded_success, 0u);
    BOOST_CHECK_EQUAL(miss.bounded_failures, 1u);
    BOOST_CHECK_EQUAL(miss.bounded_legacy_scans, 0u);

    // The same far seed DOES succeed under the legacy acquisition strategy,
    // which proves the bounded path is genuinely more restrictive rather than
    // broken.
    auto legacy = p;
    legacy.sync_strategy = openuwb::demod::SyncStrategy::AcquireLegacy;
    legacy.timing_search_margin = 40960;
    legacy.timing_max_backtrack_symbols = 40;
    openuwb::demod::TimingResult acq;
    BOOST_REQUIRE(openuwb::demod::core::stage_timing(
        rx.data(), n, legacy, tmpl, int64_t(start) + 508, acq, scratch));
    BOOST_CHECK_EQUAL(acq.bounded_attempts, 0u);
    BOOST_CHECK_LE(std::abs(acq.preamble_start_sample - int64_t(start)), 2);
}

// ===========================================================================
// 9. Holdover widens the guards and keeps the prediction
// ===========================================================================
BOOST_AUTO_TEST_CASE(holdover_widens_guards_and_keeps_phase)
{
    TargetLockCore lock(make_config());
    for (uint64_t k = 0; k < 8; ++k)
        feed(lock, good_obs(lock, k, true_origin(k)));
    const int64_t t0 = std::llround(lock.t0_exact());
    const double period = lock.period_samples();

    TargetSlotPlan before;
    BOOST_REQUIRE(lock.next_plan(true_origin(8) - 100000, before));
    BOOST_CHECK(!before.widened);

    // Three consecutive target failures -> HOLDOVER.
    for (uint64_t k = 8; k < 8 + 3; ++k) {
        auto o = good_obs(lock, k, true_origin(k));
        o.observed_origin = -1;
        feed(lock, o);
    }
    BOOST_REQUIRE_EQUAL(static_cast<int>(lock.state()),
                        static_cast<int>(TargetLockState::Holdover));
    // The phase/period prediction is untouched by failures.
    BOOST_CHECK_EQUAL(std::llround(lock.t0_exact()), t0);
    BOOST_CHECK_EQUAL(lock.period_samples(), period);

    TargetSlotPlan after;
    BOOST_REQUIRE(lock.next_plan(true_origin(11) - 100000, after));
    BOOST_CHECK(after.widened);
    BOOST_CHECK_GT(after.pre_guard, before.pre_guard);
    BOOST_CHECK_GT(after.post_guard, before.post_guard);
    BOOST_CHECK_EQUAL(after.capture, before.capture);
    BOOST_CHECK_GT(lock.stats().holdover_entries, 0u);
    BOOST_CHECK_GT(lock.stats().holdover_length, 0u);
}

// ===========================================================================
// 10. Sustained failures force REACQUIRE
// ===========================================================================
BOOST_AUTO_TEST_CASE(sustained_failures_reacquire)
{
    TargetLockCore lock(make_config());
    for (uint64_t k = 0; k < 8; ++k)
        feed(lock, good_obs(lock, k, true_origin(k)));
    const uint64_t gen = lock.generation();
    for (uint64_t k = 8; k < 8 + 8; ++k) {
        auto o = good_obs(lock, k, true_origin(k));
        o.observed_origin = -1;
        feed(lock, o);
    }
    BOOST_CHECK_EQUAL(static_cast<int>(lock.state()),
                      static_cast<int>(TargetLockState::Reacquire));
    BOOST_CHECK_GT(lock.generation(), gen);
    BOOST_CHECK_EQUAL(lock.stats().reacquire_entries, 1u);
    BOOST_CHECK(!lock.scheduled());

    // A new identity-valid target packet re-seeds from REACQUIRE.
    feed(lock, good_obs(lock, 0, true_origin(40)));
    BOOST_CHECK(lock.scheduled());
}

// ===========================================================================
// 11. Residual outliers force REACQUIRE
// ===========================================================================
BOOST_AUTO_TEST_CASE(residual_outliers_reacquire)
{
    TargetLockCore lock(make_config());
    for (uint64_t k = 0; k < 8; ++k)
        feed(lock, good_obs(lock, k, true_origin(k)));
    for (uint64_t k = 8; k < 8 + 3; ++k)
        feed(lock, good_obs(lock, k, true_origin(k) + 20000)); // way off grid
    BOOST_CHECK_EQUAL(static_cast<int>(lock.state()),
                      static_cast<int>(TargetLockState::Reacquire));
    BOOST_CHECK_GT(lock.stats().residual_outliers, 0u);
}

// ===========================================================================
// 12. Unexpected gap invalidates the grid
// ===========================================================================
BOOST_AUTO_TEST_CASE(discontinuity_invalidates_the_grid)
{
    TargetLockCore lock(make_config());
    for (uint64_t k = 0; k < 8; ++k)
        feed(lock, good_obs(lock, k, true_origin(k)));
    const uint64_t gen = lock.generation();
    lock.note_time_discontinuity();
    BOOST_CHECK_EQUAL(static_cast<int>(lock.state()),
                      static_cast<int>(TargetLockState::Reacquire));
    BOOST_CHECK_GT(lock.generation(), gen);
    BOOST_CHECK_EQUAL(lock.stats().discontinuity_events, 1u);
    BOOST_CHECK(!lock.scheduled());
}

// ===========================================================================
// 13. Interference classification and target-response dedup
// ===========================================================================
BOOST_AUTO_TEST_CASE(candidate_classification)
{
    TargetLockCore lock(make_config());
    // Before the lock nothing can be called the target's own response.
    InterferenceCandidate c0;
    c0.start_sample = true_origin(0);
    c0.metric = 0.8;
    BOOST_CHECK_EQUAL(static_cast<int>(lock.classify_candidate(c0).cls),
                      static_cast<int>(CandidateClass::Unknown));

    for (uint64_t k = 0; k < 8; ++k)
        feed(lock, good_obs(lock, k, true_origin(k)));

    // Near a slot whose target decode succeeded -> the target's own response.
    InterferenceCandidate near;
    near.start_sample = true_origin(5) + 500;
    near.metric = 0.9;
    BOOST_CHECK_EQUAL(static_cast<int>(lock.classify_candidate(near).cls),
                      static_cast<int>(CandidateClass::TargetAssociated));

    // A confirmed different code is interference even inside the target window.
    InterferenceCandidate other = near;
    other.template_confirmed = true;
    other.code_index = 9;
    BOOST_CHECK_EQUAL(static_cast<int>(lock.classify_candidate(other).cls),
                      static_cast<int>(CandidateClass::Interference));

    // Far from the predicted grid -> interference / unknown, never the target.
    InterferenceCandidate far;
    far.start_sample = true_origin(5) + 200000;
    far.metric = 0.9;
    BOOST_CHECK_EQUAL(static_cast<int>(lock.classify_candidate(far).cls),
                      static_cast<int>(CandidateClass::Unknown));

    // Near a slot whose target decode FAILED stays Unknown: overlapping
    // interference must not be silently discarded.
    auto miss = good_obs(lock, 9, true_origin(9));
    miss.observed_origin = -1;
    feed(lock, miss);
    InterferenceCandidate near_failed;
    near_failed.start_sample = true_origin(9) + 300;
    near_failed.metric = 0.9;
    BOOST_CHECK_EQUAL(static_cast<int>(lock.classify_candidate(near_failed).cls),
                      static_cast<int>(CandidateClass::Unknown));

    const auto st = lock.stats();
    BOOST_CHECK_EQUAL(st.candidates, 5ull);
    BOOST_CHECK_EQUAL(st.candidates_target, 1ull);
    BOOST_CHECK_EQUAL(st.candidates_interference, 1ull);
    BOOST_CHECK_EQUAL(st.candidates_unknown, 3ull);
}

// ===========================================================================
// 14. Bounded feedback queue
// ===========================================================================
BOOST_AUTO_TEST_CASE(feedback_queue_is_bounded)
{
    auto cfg = make_config();
    cfg.feedback_queue_capacity = 4;
    TargetLockCore lock(cfg);
    for (int i = 0; i < 4; ++i)
        BOOST_CHECK(lock.push_observation(good_obs(lock, 0, true_origin(0))));
    BOOST_CHECK(!lock.push_observation(good_obs(lock, 0, true_origin(0))));
    BOOST_CHECK_EQUAL(lock.stats().queue_full, 1u);
    BOOST_CHECK_EQUAL(lock.drain_observations(), 1u); // only one can seed
}

// ===========================================================================
// 15. Slot accounting: expected / submitted / dropped / expired
// ===========================================================================
BOOST_AUTO_TEST_CASE(slot_accounting)
{
    TargetLockCore lock(make_config());
    for (uint64_t k = 0; k < 6; ++k)
        feed(lock, good_obs(lock, k, true_origin(k)));

    TargetSlotPlan plan;
    BOOST_REQUIRE(lock.next_plan(true_origin(6) - 200000, plan));
    BOOST_CHECK_EQUAL(plan.generation, lock.generation());
    BOOST_CHECK_EQUAL(plan.predicted_origin, lock.predicted(plan.slot_index));
    BOOST_CHECK_EQUAL(plan.window_start,
                      plan.predicted_origin - int64_t(plan.pre_guard));
    BOOST_CHECK_EQUAL(plan.window_end,
                      plan.predicted_origin + int64_t(plan.capture) +
                          int64_t(plan.post_guard));

    lock.commit_plan(plan);
    BOOST_CHECK_EQUAL(lock.stats().expected_slots, 1u);
    BOOST_CHECK_EQUAL(lock.stats().submitted, 1u);

    lock.note_slot_outcome(plan.slot_index, TargetSlotOutcome::DroppedQueue);
    BOOST_CHECK_EQUAL(lock.stats().dropped_queue, 1u);

    // The same slot must never be planned twice after an abandon.
    lock.abandon_slot(plan.slot_index);
    TargetSlotPlan next;
    BOOST_REQUIRE(lock.next_plan(true_origin(6) - 200000, next));
    BOOST_CHECK_GT(next.slot_index, plan.slot_index);
}

// ===========================================================================
// 16. Runtime pre-guard update (LOCKED shrinks the window, the rest keeps it)
// ===========================================================================
BOOST_AUTO_TEST_CASE(runtime_pre_guard_update_keeps_plan_and_window_consistent)
{
    auto cfg = make_config();
    TargetLockCore lock(cfg);
    const size_t wide = lock.pre_guard();
    BOOST_CHECK_EQUAL(wide, 36864u);
    BOOST_CHECK_EQUAL(lock.window_capacity(), 600000u);

    // Lock on a real grid first: the guard change happens while LOCKED.
    for (uint64_t k = 0; k < 6; ++k)
        feed(lock, good_obs(lock, k, true_origin(k)));
    BOOST_REQUIRE_EQUAL(static_cast<int>(lock.state()),
                        static_cast<int>(TargetLockState::Locked));

    const size_t narrow = 8192;
    BOOST_REQUIRE(lock.set_pre_guard(narrow));
    BOOST_CHECK_EQUAL(lock.pre_guard(), narrow);
    BOOST_CHECK_EQUAL(lock.stats().pre_guard_updates, 1u);

    TargetSlotPlan plan;
    BOOST_REQUIRE(lock.next_plan(true_origin(6) - 200000, plan));
    // The window start, the plan's own guard and the predicted origin must
    // agree: this is the invariant the live scheduler relies on.
    BOOST_CHECK_EQUAL(plan.pre_guard, narrow);
    BOOST_CHECK_EQUAL(plan.window_start,
                      plan.predicted_origin - int64_t(narrow));
    BOOST_CHECK_EQUAL(plan.window_start + int64_t(plan.pre_guard),
                      plan.predicted_origin);
    // The capture/post-guard side is untouched by a guard change.
    BOOST_CHECK_EQUAL(plan.capture, cfg.geometry.capture);
    BOOST_CHECK_EQUAL(plan.post_guard, cfg.geometry.post_guard);
    BOOST_CHECK_EQUAL(plan.window_end,
                      plan.predicted_origin + int64_t(plan.capture) +
                          int64_t(plan.post_guard));
    BOOST_CHECK_LE(static_cast<size_t>(plan.window_end - plan.window_start),
                   lock.window_capacity());
    // plan_for_slot must be the same geometry as next_plan.
    const TargetSlotPlan again = lock.plan_for_slot(plan.slot_index);
    BOOST_CHECK_EQUAL(again.window_start, plan.window_start);
    BOOST_CHECK_EQUAL(again.window_end, plan.window_end);

    // Shrinking moves the window start LATER, so a cursor that already passed
    // the wide start still yields a window that begins at or after it.
    const int64_t cursor = true_origin(6) - 20000;
    TargetSlotPlan near;
    BOOST_REQUIRE(lock.next_plan(cursor, near));
    BOOST_CHECK_GE(near.window_start, cursor);

    // A state change back to the wide guard restores the exact old geometry.
    BOOST_REQUIRE(lock.set_pre_guard(wide));
    BOOST_CHECK_EQUAL(lock.pre_guard(), wide);
    BOOST_CHECK_EQUAL(lock.stats().pre_guard_updates, 2u);
    const TargetSlotPlan restored = lock.plan_for_slot(plan.slot_index);
    BOOST_CHECK_EQUAL(restored.window_start, plan.predicted_origin - int64_t(wide));
    BOOST_CHECK_EQUAL(restored.window_end, plan.window_end);

    // A no-op update is accepted but must not be counted as a change.
    BOOST_CHECK(lock.set_pre_guard(wide));
    BOOST_CHECK_EQUAL(lock.stats().pre_guard_updates, 2u);

    // Holdover widening is added on top of whatever guard is effective, and
    // the widened geometry must still fit the job buffer.
    BOOST_REQUIRE(lock.set_pre_guard(narrow));
    for (uint64_t k = 6; k < 9; ++k) {
        auto miss = good_obs(lock, k, true_origin(k));
        miss.observed_origin = -1;
        feed(lock, miss);
    }
    BOOST_REQUIRE_EQUAL(static_cast<int>(lock.state()),
                        static_cast<int>(TargetLockState::Holdover));
    TargetSlotPlan held;
    BOOST_REQUIRE(lock.next_plan(true_origin(9) - 100000, held));
    BOOST_CHECK(held.widened);
    // Growth is per consecutive miss (3 misses -> 3 * 2048, under the cap).
    BOOST_CHECK_EQUAL(held.pre_guard, narrow + 3 * cfg.holdover_guard_growth);
    BOOST_CHECK_EQUAL(held.window_start,
                      held.predicted_origin - int64_t(held.pre_guard));
    BOOST_CHECK_LE(static_cast<size_t>(held.window_end - held.window_start),
                   lock.window_capacity());
}

// ===========================================================================
// 17. Runtime pre-guard update fails closed outside the job buffer
// ===========================================================================
BOOST_AUTO_TEST_CASE(runtime_pre_guard_update_fails_closed)
{
    auto cfg = make_config();
    TargetLockCore lock(cfg);
    const size_t wide = lock.pre_guard();
    const size_t capture = cfg.geometry.capture;
    const size_t post = cfg.geometry.post_guard;

    // Zero is never a valid guard.
    BOOST_CHECK(!lock.set_pre_guard(0));
    BOOST_CHECK_EQUAL(lock.pre_guard(), wide);
    BOOST_CHECK_EQUAL(lock.stats().pre_guard_updates, 0u);

    // One sample over the job buffer must be refused, not clamped.
    const size_t fits = cfg.window_capacity - capture - post;
    BOOST_REQUIRE(lock.set_pre_guard(fits));
    BOOST_CHECK_EQUAL(lock.pre_guard(), fits);
    BOOST_CHECK(!lock.set_pre_guard(fits + 1));
    BOOST_CHECK_EQUAL(lock.pre_guard(), fits); // unchanged after the refusal
    BOOST_CHECK_EQUAL(lock.stats().pre_guard_updates, 1u);

    // A core without a declared job buffer refuses every update.
    auto unbounded = make_config();
    unbounded.window_capacity = 0;
    TargetLockCore raw(unbounded);
    BOOST_CHECK(!raw.set_pre_guard(8192));
    BOOST_CHECK_EQUAL(raw.pre_guard(), 36864u);

    // A geometry without a capture span is not a window at all.
    auto broken = make_config();
    broken.geometry.capture = 0;
    TargetLockCore invalid(broken);
    BOOST_CHECK(!invalid.set_pre_guard(8192));
}

// ===========================================================================
// 18. The association window does not follow a guard shrink
// ===========================================================================
BOOST_AUTO_TEST_CASE(association_window_is_not_narrowed_by_guard_shrink)
{
    auto cfg = make_config();
    TargetLockCore lock(cfg);
    for (uint64_t k = 0; k < 6; ++k)
        feed(lock, good_obs(lock, k, true_origin(k)));
    BOOST_REQUIRE(lock.set_pre_guard(8192));

    // Schedule slot 6 (outcome Submitted) so association can resolve.
    TargetSlotPlan plan;
    BOOST_REQUIRE(lock.next_plan(true_origin(6) - 200000, plan));
    lock.commit_plan(plan);

    // 20000 samples from the prediction is outside the shrunken 8192 guard but
    // inside the association window captured at construction (36864).  The
    // guard shrink must not reclassify the target's own response.
    InterferenceCandidate near;
    near.start_sample = plan.predicted_origin + 20000;
    near.metric = 0.9;
    const auto v = lock.classify_candidate(near);
    BOOST_CHECK_EQUAL(static_cast<int>(v.cls),
                      static_cast<int>(CandidateClass::TargetAssociated));
    BOOST_CHECK_EQUAL(v.slot_index, plan.slot_index);
    BOOST_CHECK_EQUAL(v.offset_from_target, 20000);

    // Beyond the association window the verdict is still Unknown, not target.
    InterferenceCandidate far;
    far.start_sample = plan.predicted_origin + 40000;
    far.metric = 0.9;
    BOOST_CHECK_EQUAL(static_cast<int>(lock.classify_candidate(far).cls),
                      static_cast<int>(CandidateClass::Unknown));
}
