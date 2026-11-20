/*
 * QA for the header-only autocorrelation gate used by uwb_dw3000_live
 * (docs/performance/开发规划_无干扰锁定后暂停自相关_2026-11-20.md §3.2, §4.4).
 *
 * Covers the pure state machine only:
 *   - disabled gate is the historical always-scan behaviour;
 *   - Acquire -> Verify/Locked/Holdover suspends exactly once;
 *   - Reacquire resumes with reset_at truncated by valid_begin and lookback;
 *   - repeated lock cycles keep the counters and sample sums exact;
 *   - CLI parsing / resolution fail-closed rules and Action invariants.
 *
 * All expectations are exact integers; no fixtures and no floating point.
 */
#define BOOST_TEST_MODULE qa_uwb_autocorr_gate
#include <boost/test/unit_test.hpp>

#include <gnuradio/uwb/uwb_autocorr_gate.h>

#include <cstdint>
#include <stdexcept>
#include <string>

namespace {

using gr::uwb::live::AutocorrGate;
using gr::uwb::live::AutocorrWhenLocked;
using gr::uwb::live::parse_autocorr_when_locked;
using gr::uwb::live::resolve_autocorr_suspend;

AutocorrGate make_gate(bool enabled)
{
    AutocorrGate gate;
    gate.enabled = enabled;
    return gate;
}

// A resume must always restart the probe: scan implies reset.
void check_resume_action(const AutocorrGate::Action& action, int64_t reset_at)
{
    BOOST_CHECK(action.scan);
    BOOST_CHECK(action.reset);
    BOOST_CHECK_EQUAL(action.reset_at, reset_at);
}

void check_plain_scan(const AutocorrGate::Action& action, int64_t end)
{
    BOOST_CHECK(action.scan);
    BOOST_CHECK(!action.reset);
    BOOST_CHECK_EQUAL(action.reset_at, end);
}

void check_suspended_action(const AutocorrGate::Action& action, int64_t end)
{
    BOOST_CHECK(!action.scan);
    BOOST_CHECK(!action.reset);
    BOOST_CHECK_EQUAL(action.reset_at, end);
}

// 1. enabled=false: always scan, never reset, never count, across mixed
//    scheduled/unscheduled blocks and different block sizes.
BOOST_AUTO_TEST_CASE(disabled_gate_always_scans)
{
    AutocorrGate gate = make_gate(false);

    struct Step {
        bool scheduled;
        int64_t end;
        size_t n;
    };
    const Step steps[] = {
        { false, 1280, 0 },
        { false, 2560, 128 },
        { true, 5120, 1024 },
        { true, 6144, 512 },
        { false, 7168, 1 },
        { true, 8192, 4096 },
    };

    for (const auto& step : steps) {
        const auto action = gate.update(step.scheduled, step.end, 0, 0, step.n);
        check_plain_scan(action, step.end);
        BOOST_CHECK(!gate.suspended);
        BOOST_CHECK_EQUAL(gate.suspend_events, uint64_t{ 0 });
        BOOST_CHECK_EQUAL(gate.resume_events, uint64_t{ 0 });
        BOOST_CHECK_EQUAL(gate.suspended_samples, uint64_t{ 0 });
        BOOST_CHECK_EQUAL(gate.resume_backlog_samples, uint64_t{ 0 });
    }
}

// 1b. enabled but never tracked: every block is a plain scan and no counter
//     moves (the gate must not treat ordinary Acquire blocks as a resume).
BOOST_AUTO_TEST_CASE(enabled_untracked_never_counts)
{
    AutocorrGate gate = make_gate(true);
    for (int i = 0; i < 50; ++i) {
        const auto action =
            gate.update(false, 1000 + 64 * i, 0, 642000, 64);
        check_plain_scan(action, 1000 + 64 * i);
    }
    BOOST_CHECK(!gate.suspended);
    BOOST_CHECK_EQUAL(gate.suspend_events, uint64_t{ 0 });
    BOOST_CHECK_EQUAL(gate.resume_events, uint64_t{ 0 });
    BOOST_CHECK_EQUAL(gate.suspended_samples, uint64_t{ 0 });
    BOOST_CHECK_EQUAL(gate.resume_backlog_samples, uint64_t{ 0 });
}

// 1c. A zero-length block can enter/keep the suspension but adds no skipped
//     samples; the app wraps update() in `if (n)`, this pins the contract.
BOOST_AUTO_TEST_CASE(zero_length_block)
{
    AutocorrGate gate = make_gate(true);
    check_suspended_action(gate.update(true, 1000, 0, 0, 0), 1000);
    BOOST_CHECK_EQUAL(gate.suspend_events, uint64_t{ 1 });
    BOOST_CHECK_EQUAL(gate.suspended_samples, uint64_t{ 0 });
    check_suspended_action(gate.update(true, 1000, 0, 0, 0), 1000);
    BOOST_CHECK_EQUAL(gate.suspend_events, uint64_t{ 1 });
    BOOST_CHECK_EQUAL(gate.suspended_samples, uint64_t{ 0 });
}

// 2. Acquire (not tracked) -> Verify/Locked/Holdover (tracked) suspends on the
//    first tracked block only and accumulates suspended samples.
BOOST_AUTO_TEST_CASE(tracking_suspends_once)
{
    AutocorrGate gate = make_gate(true);

    // Acquire: still scanning.
    auto action = gate.update(false, 1000, 0, 0, 128);
    check_plain_scan(action, 1000);
    BOOST_CHECK(!gate.suspended);
    BOOST_CHECK_EQUAL(gate.suspend_events, uint64_t{ 0 });
    BOOST_CHECK_EQUAL(gate.suspended_samples, uint64_t{ 0 });

    // Verify: enter suspension.
    action = gate.update(true, 2000, 0, 0, 256);
    check_suspended_action(action, 2000);
    BOOST_CHECK(gate.suspended);
    BOOST_CHECK_EQUAL(gate.suspend_events, uint64_t{ 1 });
    BOOST_CHECK_EQUAL(gate.suspended_samples, uint64_t{ 256 });

    // Locked: stays suspended, no extra suspend event.
    action = gate.update(true, 3000, 0, 0, 300);
    check_suspended_action(action, 3000);
    BOOST_CHECK_EQUAL(gate.suspend_events, uint64_t{ 1 });
    BOOST_CHECK_EQUAL(gate.suspended_samples, uint64_t{ 556 });

    // Holdover: still tracked, still suspended.
    action = gate.update(true, 4000, 0, 0, 100);
    check_suspended_action(action, 4000);
    BOOST_CHECK_EQUAL(gate.suspend_events, uint64_t{ 1 });
    BOOST_CHECK_EQUAL(gate.suspended_samples, uint64_t{ 656 });

    // Nothing has resumed yet.
    BOOST_CHECK_EQUAL(gate.resume_events, uint64_t{ 0 });
    BOOST_CHECK_EQUAL(gate.resume_backlog_samples, uint64_t{ 0 });
}

// 3a. Reacquire after tracking: reset_at is truncated by valid_begin.
BOOST_AUTO_TEST_CASE(resume_truncated_by_valid_begin)
{
    AutocorrGate gate = make_gate(true);

    gate.update(false, 1000, 0, 0, 64);
    gate.update(true, 5000, 0, 0, 100);
    BOOST_CHECK(gate.suspended);

    // end - lookback = 6000 - 1500 = 4500, valid_begin = 5200 wins.
    const auto action = gate.update(false, 6000, 5200, 1500, 50);
    check_resume_action(action, 5200);
    BOOST_CHECK(!gate.suspended);
    BOOST_CHECK_EQUAL(gate.resume_events, uint64_t{ 1 });
    BOOST_CHECK_EQUAL(gate.resume_backlog_samples, uint64_t{ 800 });
    // The resume block itself is scanned, not counted as suspended.
    BOOST_CHECK_EQUAL(gate.suspended_samples, uint64_t{ 100 });
    BOOST_CHECK_EQUAL(gate.suspend_events, uint64_t{ 1 });

    // The next untracked block is a plain scan with no reset.
    check_plain_scan(gate.update(false, 7000, 6000, 1500, 50), 7000);
    BOOST_CHECK_EQUAL(gate.resume_events, uint64_t{ 1 });
}

// 3b. lookback wins when it is newer than valid_begin.
BOOST_AUTO_TEST_CASE(resume_uses_lookback_window)
{
    AutocorrGate gate = make_gate(true);
    gate.update(true, 5000, 0, 0, 100);

    // end - lookback = 7000 - 2000 = 5000 > valid_begin = 100.
    const auto action = gate.update(false, 7000, 100, 2000, 40);
    check_resume_action(action, 5000);
    BOOST_CHECK_EQUAL(gate.resume_backlog_samples, uint64_t{ 2000 });
}

// 3c. lookback == 0 resumes at end with no backlog; negative lookback is
//     treated as 0.
BOOST_AUTO_TEST_CASE(resume_zero_and_negative_lookback)
{
    {
        AutocorrGate gate = make_gate(true);
        gate.update(true, 5000, 0, 0, 100);
        const auto action = gate.update(false, 7000, 100, 0, 40);
        check_resume_action(action, 7000);
        BOOST_CHECK_EQUAL(gate.resume_backlog_samples, uint64_t{ 0 });
    }
    {
        AutocorrGate gate = make_gate(true);
        gate.update(true, 5000, 0, 0, 100);
        const auto action = gate.update(false, 7000, 100, -1234, 40);
        check_resume_action(action, 7000);
        BOOST_CHECK_EQUAL(gate.resume_backlog_samples, uint64_t{ 0 });
    }
}

// 3d. lookback beyond the ring start: reset_at never goes before sample 0.
//     The live caller never passes a negative valid_begin, but a bad caller
//     must not make Acquisition::scan cast a negative probe to size_t.
BOOST_AUTO_TEST_CASE(resume_clamped_by_ring_start)
{
    AutocorrGate gate = make_gate(true);
    gate.update(true, 500, 0, 0, 100);

    // raw end - lookback = 1000 - 4000 = -3000 and valid_begin = -500; both
    // are clamped to the absolute stream start 0.
    const auto action = gate.update(false, 1000, -500, 4000, 20);
    check_resume_action(action, 0);
    BOOST_CHECK_EQUAL(gate.resume_backlog_samples, uint64_t{ 1000 });
}

// 3e. lookback larger than the current end with valid_begin == 0 saturates at
//     0 instead of producing a negative probe.
BOOST_AUTO_TEST_CASE(resume_lookback_beyond_end)
{
    AutocorrGate gate = make_gate(true);
    gate.update(true, 500, 0, 0, 100);
    const auto action = gate.update(false, 1000, 0, 4000, 20);
    check_resume_action(action, 0);
    BOOST_CHECK_EQUAL(gate.resume_backlog_samples, uint64_t{ 1000 });
}

// 4. Several enter/leave cycles: every counter and sample sum is exact,
//    including an immediate re-lock on the block after a resume.
BOOST_AUTO_TEST_CASE(multiple_lock_cycles)
{
    AutocorrGate gate = make_gate(true);

    // Cycle 1: Acquire, Verify, Locked, Reacquire.
    check_plain_scan(gate.update(false, 1000, 0, 0, 10), 1000);
    check_suspended_action(gate.update(true, 2000, 0, 0, 100), 2000);
    check_suspended_action(gate.update(true, 3000, 0, 0, 110), 3000);
    // end - lookback = 4000 - 800 = 3200 > valid_begin 0.
    check_resume_action(gate.update(false, 4000, 0, 800, 5), 3200);

    // Cycle 2: immediate re-lock (Verify) on the very next block.
    check_suspended_action(gate.update(true, 5000, 0, 0, 210), 5000);
    // Reacquire at end == valid_begin: empty backlog.
    check_resume_action(gate.update(false, 5100, 5100, 800, 7), 5100);

    // Cycle 3: two tracked blocks then a resume truncated by valid_begin.
    check_suspended_action(gate.update(true, 6000, 0, 0, 30), 6000);
    check_suspended_action(gate.update(true, 7000, 0, 0, 20), 7000);
    // end - lookback = 8000 - 3000 = 5000 < valid_begin 6500.
    check_resume_action(gate.update(false, 8000, 6500, 3000, 11), 6500);

    BOOST_CHECK_EQUAL(gate.suspend_events, uint64_t{ 3 });
    BOOST_CHECK_EQUAL(gate.resume_events, uint64_t{ 3 });
    BOOST_CHECK_EQUAL(gate.suspended_samples, uint64_t{ 470 });
    // 800 + 0 + 1500.
    BOOST_CHECK_EQUAL(gate.resume_backlog_samples, uint64_t{ 2300 });
    BOOST_CHECK(!gate.suspended);
}

// 5a. parse_autocorr_when_locked accepts exactly the three values and rejects
//     everything else, including case/whitespace variants.
BOOST_AUTO_TEST_CASE(parse_accepts_only_documented_values)
{
    BOOST_CHECK(parse_autocorr_when_locked("scan") == AutocorrWhenLocked::Scan);
    BOOST_CHECK(parse_autocorr_when_locked("suspend") == AutocorrWhenLocked::Suspend);
    BOOST_CHECK(parse_autocorr_when_locked("auto") == AutocorrWhenLocked::Auto);

    const std::string bad[] = { "on", "off", "", "SUSPEND", "Scan", "scan ",
                                " suspend", "auto\n", "1" };
    for (const auto& value : bad) {
        BOOST_CHECK_THROW(parse_autocorr_when_locked(value), std::invalid_argument);
    }
}

// 5b. resolve_autocorr_suspend contract, including the fail-closed
//     Suspend+interference combination.
BOOST_AUTO_TEST_CASE(resolve_suspend_contract)
{
    // Scan never suspends.
    BOOST_CHECK(!resolve_autocorr_suspend(AutocorrWhenLocked::Scan, false));
    BOOST_CHECK(!resolve_autocorr_suspend(AutocorrWhenLocked::Scan, true));

    // Auto follows the interference flag.
    BOOST_CHECK(!resolve_autocorr_suspend(AutocorrWhenLocked::Auto, true));
    BOOST_CHECK(resolve_autocorr_suspend(AutocorrWhenLocked::Auto, false));

    // Suspend requires interference off.
    BOOST_CHECK(resolve_autocorr_suspend(AutocorrWhenLocked::Suspend, false));
    BOOST_CHECK_THROW(resolve_autocorr_suspend(AutocorrWhenLocked::Suspend, true),
                      std::invalid_argument);
}

// 5c. Action invariants: a resume always resets; a disabled gate never resets;
//     a suspended block never resets.
BOOST_AUTO_TEST_CASE(action_invariants)
{
    AutocorrGate gate = make_gate(true);
    gate.update(false, 1000, 0, 0, 10);
    gate.update(true, 2000, 0, 0, 20);
    const auto resume = gate.update(false, 3000, 0, 500, 10);
    BOOST_CHECK(resume.scan);
    BOOST_CHECK(resume.reset);
    BOOST_CHECK_EQUAL(resume.reset_at, 2500);

    // Every disabled-gate action is reset-free.
    AutocorrGate off = make_gate(false);
    for (int i = 0; i < 4; ++i) {
        const auto action = off.update(i % 2 == 0, 1000 + i, 900, 500, 16);
        BOOST_CHECK(action.scan);
        BOOST_CHECK(!action.reset);
    }

    // A suspended block never resets.
    AutocorrGate on = make_gate(true);
    const auto suspended = on.update(true, 4000, 0, 500, 32);
    BOOST_CHECK(!suspended.scan);
    BOOST_CHECK(!suspended.reset);
}

} // namespace
