/*
 * QA for the device-timed RX plan (uwb_timed_rx_plan.h).
 *
 * Acceptance items from
 * docs/performance/X410_目标周期锁定与随机干扰双路径修改规划_2026-11-15.md §10.4:
 *   - a planned gap is not an error and is not fabricated continuous samples
 *   - an unplanned gap is an unexpected_gap
 *   - late command / partial recv / timeout / overflow / device time jump all
 *     have a distinct terminal result
 *   - hardware alignment expansion, bounded lookahead, bounded request queue
 *   - absolute tone phase under a merged physical RX interval
 */
#define BOOST_TEST_MODULE qa_uwb_timed_rx_plan
#include <boost/test/unit_test.hpp>

#include <gnuradio/uwb/uwb_timed_rx_plan.h>

#include <cmath>
#include <complex>
#include <cstdint>
#include <vector>

namespace {

using openuwb::core::merge_physical_rx_intervals;
using openuwb::core::PhysicalRxInterval;
using openuwb::core::TimedRxAssembled;
using openuwb::core::TimedRxChunk;
using openuwb::core::TimedRxPlanConfig;
using openuwb::core::TimedRxPlanCore;
using openuwb::core::TimedRxRequest;
using openuwb::core::TimedRxTerminal;
using openuwb::core::TimedRxWindowSpec;

constexpr double kFs = 737280000.0;

TimedRxPlanConfig make_config()
{
    TimedRxPlanConfig c;
    c.sample_rate = kFs;
    c.alignment = 4;
    c.max_lookahead = 4;
    c.command_lead_s = 300e-6;
    c.max_samples_per_command = 0;
    c.request_capacity = 32;
    return c;
}

std::vector<std::complex<int16_t>> ramp(int64_t start, size_t n)
{
    std::vector<std::complex<int16_t>> v(n);
    for (size_t i = 0; i < n; ++i) {
        const int64_t p = start + static_cast<int64_t>(i);
        v[i] = std::complex<int16_t>(
            static_cast<int16_t>(p & 0x7fff),
            static_cast<int16_t>((p * 3) & 0x7fff));
    }
    return v;
}

void feed_all(TimedRxPlanCore& plan, int64_t start, size_t n)
{
    const auto src = ramp(start, n);
    TimedRxChunk c;
    c.device_start = start;
    c.count = n;
    c.data = src.data();
    c.has_time_spec = true;
    c.time_spec_s = double(start) / kFs;
    plan.on_chunk(c);
}

} // namespace

// ===========================================================================
// 1. Hardware alignment expansion
// ===========================================================================
BOOST_AUTO_TEST_CASE(alignment_expands_outward)
{
    TimedRxPlanCore plan(make_config());
    TimedRxRequest r;
    r.logical_start = 1001;
    r.logical_samples = 5001;
    const auto s = plan.expand(r, 1);
    BOOST_CHECK_EQUAL(s.device_start, 1000);          // floor to a multiple of 4
    BOOST_CHECK_EQUAL(s.device_samples, 5004u);       // ceil(6002/4)*4 - 1000
    BOOST_CHECK_LE(s.device_start, r.logical_start);
    BOOST_CHECK_GE(s.device_start + int64_t(s.device_samples),
                   r.logical_start + int64_t(r.logical_samples));
    BOOST_CHECK_EQUAL(s.device_samples % 4, 0u);
    BOOST_CHECK_CLOSE(s.device_time_s, 1000.0 / kFs, 1e-9);
}

// ===========================================================================
// 2. A too-late request is cancelled, never issued late
// ===========================================================================
BOOST_AUTO_TEST_CASE(late_request_is_cancelled_not_issued)
{
    TimedRxPlanCore plan(make_config());
    TimedRxRequest r;
    r.logical_start = 100000;
    r.logical_samples = 4000;
    BOOST_REQUIRE(plan.queue_request(r));

    TimedRxWindowSpec spec;
    // "Now" is already past the window start minus the lead time.
    const double now = double(100000) / kFs;
    BOOST_CHECK(!plan.next_command(now, spec));
    BOOST_CHECK_EQUAL(plan.stats().cancelled, 1u);
    BOOST_CHECK_EQUAL(plan.stats().issued, 0u);

    TimedRxAssembled a;
    BOOST_REQUIRE(plan.take_assembled(a));
    BOOST_CHECK_EQUAL(static_cast<int>(a.terminal),
                      static_cast<int>(TimedRxTerminal::Cancelled));
}

// ===========================================================================
// 3. Bounded lookahead and bounded request queue
// ===========================================================================
BOOST_AUTO_TEST_CASE(bounded_lookahead_and_request_queue)
{
    auto cfg = make_config();
    cfg.max_lookahead = 2;
    cfg.request_capacity = 3;
    TimedRxPlanCore plan(cfg);
    for (int i = 0; i < 3; ++i) {
        TimedRxRequest r;
        r.logical_start = 1000000 + int64_t(i) * 4000000;
        r.logical_samples = 100000;
        BOOST_REQUIRE(plan.queue_request(r));
    }
    TimedRxRequest extra;
    extra.logical_start = 99000000;
    extra.logical_samples = 1000;
    BOOST_CHECK(!plan.queue_request(extra));
    BOOST_CHECK_EQUAL(plan.stats().request_queue_full, 1u);

    TimedRxWindowSpec spec;
    BOOST_REQUIRE(plan.next_command(0.0, spec));
    BOOST_REQUIRE(plan.next_command(0.0, spec));
    // Lookahead exhausted: the third command must wait for an assembly.
    BOOST_CHECK(!plan.next_command(0.0, spec));
    BOOST_CHECK_EQUAL(plan.stats().lookahead_blocked, 1u);
    BOOST_CHECK_EQUAL(plan.outstanding(), 2u);
}

// ===========================================================================
// 4. Multi-chunk assembly reproduces the source and reports Complete
// ===========================================================================
BOOST_AUTO_TEST_CASE(multi_chunk_assembly_is_byte_exact)
{
    TimedRxPlanCore plan(make_config());
    TimedRxRequest r;
    r.logical_start = 400000;
    r.logical_samples = 200000;
    r.generation = 7;
    BOOST_REQUIRE(plan.queue_request(r));
    TimedRxWindowSpec spec;
    BOOST_REQUIRE(plan.next_command(0.0, spec));

    const auto src = ramp(spec.device_start, spec.device_samples);
    // Deliver in three pieces, like three recv() calls.
    const size_t a = spec.device_samples / 3;
    const size_t b = spec.device_samples / 3;
    const size_t c = spec.device_samples - a - b;
    size_t off = 0;
    for (size_t n : { a, b, c }) {
        TimedRxChunk ch;
        ch.device_start = spec.device_start + int64_t(off);
        ch.count = n;
        ch.data = src.data() + off;
        ch.has_time_spec = true;
        ch.time_spec_s = double(ch.device_start) / kFs;
        plan.on_chunk(ch);
        off += n;
    }
    TimedRxAssembled w;
    BOOST_REQUIRE(plan.take_assembled(w));
    BOOST_CHECK_EQUAL(static_cast<int>(w.terminal),
                      static_cast<int>(TimedRxTerminal::Complete));
    BOOST_CHECK_EQUAL(w.received, spec.device_samples);
    BOOST_CHECK_EQUAL(w.spec.generation, 7u);
    BOOST_REQUIRE_EQUAL(w.samples.size(), src.size());
    BOOST_CHECK(std::equal(w.samples.begin(), w.samples.end(), src.begin()));
    BOOST_CHECK_EQUAL(plan.stats().completed, 1u);
}

// ===========================================================================
// 5. A missing chunk inside a planned window is an unexpected gap
// ===========================================================================
BOOST_AUTO_TEST_CASE(missing_chunk_is_unexpected_gap)
{
    TimedRxPlanCore plan(make_config());
    TimedRxRequest r;
    r.logical_start = 400000;
    r.logical_samples = 40000;
    BOOST_REQUIRE(plan.queue_request(r));
    TimedRxWindowSpec spec;
    BOOST_REQUIRE(plan.next_command(0.0, spec));

    // Skip the first 1000 samples, then deliver the rest.
    const int64_t skip = 1000;
    feed_all(plan, spec.device_start + skip,
             spec.device_samples - size_t(skip));

    TimedRxAssembled w;
    BOOST_REQUIRE(plan.take_assembled(w));
    BOOST_CHECK_EQUAL(static_cast<int>(w.terminal),
                      static_cast<int>(TimedRxTerminal::UnexpectedGap));
    BOOST_CHECK_EQUAL(w.first_missing, 0);
    BOOST_CHECK_GT(plan.stats().unexpected_gaps, 0u);
}

// ===========================================================================
// 6. Overflow / late command / timeout are distinct terminals
// ===========================================================================
BOOST_AUTO_TEST_CASE(device_faults_are_distinct)
{
    const auto run = [](bool overflow, bool late, bool timeout) {
        TimedRxPlanCore plan(make_config());
        TimedRxRequest r;
        r.logical_start = 400000;
        r.logical_samples = 40000;
        plan.queue_request(r);
        TimedRxWindowSpec spec;
        BOOST_REQUIRE(plan.next_command(0.0, spec));
        TimedRxChunk ch;
        ch.device_start = spec.device_start;
        ch.count = 0;
        ch.overflow = overflow;
        ch.late_command = late;
        ch.timeout = timeout;
        plan.on_chunk(ch);
        TimedRxAssembled w;
        BOOST_REQUIRE(plan.take_assembled(w));
        return w.terminal;
    };
    BOOST_CHECK_EQUAL(static_cast<int>(run(true, false, false)),
                      static_cast<int>(TimedRxTerminal::Overflow));
    BOOST_CHECK_EQUAL(static_cast<int>(run(false, true, false)),
                      static_cast<int>(TimedRxTerminal::LateCommand));
    BOOST_CHECK_EQUAL(static_cast<int>(run(false, false, true)),
                      static_cast<int>(TimedRxTerminal::Timeout));
}

// ===========================================================================
// 7. Device time going backwards invalidates every outstanding window
// ===========================================================================
BOOST_AUTO_TEST_CASE(device_time_jump_invalidates)
{
    TimedRxPlanCore plan(make_config());
    for (int i = 0; i < 2; ++i) {
        TimedRxRequest r;
        r.logical_start = 400000 + int64_t(i) * 4000000;
        r.logical_samples = 40000;
        plan.queue_request(r);
    }
    TimedRxWindowSpec spec;
    BOOST_REQUIRE(plan.next_command(0.0, spec));
    BOOST_REQUIRE(plan.next_command(0.0, spec));

    TimedRxChunk ok;
    ok.device_start = spec.device_start;
    ok.count = 10;
    ok.has_time_spec = true;
    ok.time_spec_s = 1.0;
    plan.on_chunk(ok);

    TimedRxChunk back;
    back.device_start = spec.device_start + 10;
    back.count = 10;
    back.has_time_spec = true;
    back.time_spec_s = 0.5; // backwards
    plan.on_chunk(back);

    // Both outstanding windows are invalidated by the single jump.
    BOOST_CHECK_EQUAL(plan.stats().device_time_jumps, 2u);
    BOOST_CHECK_EQUAL(plan.outstanding(), 0u);
}

// ===========================================================================
// 8. Planned gap between windows is not an error
// ===========================================================================
BOOST_AUTO_TEST_CASE(planned_gap_is_not_an_error)
{
    TimedRxPlanCore plan(make_config());
    const int64_t gap = 500000;
    for (int i = 0; i < 2; ++i) {
        TimedRxRequest r;
        r.logical_start = 400000 + int64_t(i) * (100000 + gap);
        r.logical_samples = 100000;
        plan.queue_request(r);
    }
    for (int i = 0; i < 2; ++i) {
        TimedRxWindowSpec spec;
        BOOST_REQUIRE(plan.next_command(0.0, spec));
        feed_all(plan, spec.device_start, spec.device_samples);
    }
    TimedRxAssembled a, b;
    BOOST_REQUIRE(plan.take_assembled(a));
    BOOST_REQUIRE(plan.take_assembled(b));
    BOOST_CHECK_EQUAL(static_cast<int>(a.terminal),
                      static_cast<int>(TimedRxTerminal::Complete));
    BOOST_CHECK_EQUAL(static_cast<int>(b.terminal),
                      static_cast<int>(TimedRxTerminal::Complete));
    BOOST_CHECK_EQUAL(a.planned_gap_samples, 0);
    BOOST_CHECK_GT(b.planned_gap_samples, 0);
    BOOST_CHECK_EQUAL(plan.stats().planned_gaps, 1u);
    BOOST_CHECK_EQUAL(plan.stats().unexpected_gaps, 0u);
    BOOST_CHECK_EQUAL(plan.stats().completed, 2u);
}

// ===========================================================================
// 9. Partial recv at end of burst
// ===========================================================================
BOOST_AUTO_TEST_CASE(partial_recv_terminal)
{
    TimedRxPlanCore plan(make_config());
    TimedRxRequest r;
    r.logical_start = 400000;
    r.logical_samples = 40000;
    BOOST_REQUIRE(plan.queue_request(r));
    TimedRxWindowSpec spec;
    BOOST_REQUIRE(plan.next_command(0.0, spec));
    feed_all(plan, spec.device_start, spec.device_samples / 2);
    plan.flush_partial();
    TimedRxAssembled w;
    BOOST_REQUIRE(plan.take_assembled(w));
    BOOST_CHECK_EQUAL(static_cast<int>(w.terminal),
                      static_cast<int>(TimedRxTerminal::Partial));
    BOOST_CHECK_EQUAL(w.received, spec.device_samples / 2);
    BOOST_CHECK_EQUAL(plan.stats().partial, 1u);
}

// ===========================================================================
// 10. Fixed-tone training and target windows merge into one physical RX
// ===========================================================================
BOOST_AUTO_TEST_CASE(tone_and_target_windows_merge)
{
    // Tone training: 0.5 ms every 20 ms, origin 0.
    const int64_t tone_period = std::llround(0.020 * kFs);
    const size_t tone_window = size_t(std::llround(0.0005 * kFs));
    // Two targets: one overlapping the first tone window, one far away.
    std::vector<std::pair<int64_t, size_t>> targets = {
        { tone_window / 2, 400000 },          // overlaps tone window 0
        { 5 * tone_period, 400000 }           // overlaps tone window 5
    };
    std::vector<uint64_t> slots = { 3, 4 };
    const auto iv = merge_physical_rx_intervals(
        targets, slots, 0, tone_period, tone_window, 0, 6 * tone_period);

    BOOST_REQUIRE_GE(iv.size(), 2u);
    // Intervals never overlap and are sorted.
    for (size_t i = 1; i < iv.size(); ++i)
        BOOST_CHECK_LE(iv[i - 1].end(), iv[i].start);

    // The first interval carries BOTH a tone-training slice and a target slice.
    const PhysicalRxInterval* merged = nullptr;
    for (const auto& v : iv)
        if (v.has_tone_training && v.has_target)
            merged = &v;
    BOOST_REQUIRE(merged != nullptr);
    BOOST_CHECK_EQUAL(merged->slices.size(), 2u);
    size_t tone_slices = 0, target_slices = 0;
    for (const auto& s : merged->slices) {
        tone_slices += s.tone_training ? 1 : 0;
        target_slices += s.target ? 1 : 0;
        BOOST_CHECK_GE(s.offset, 0);
        BOOST_CHECK_LE(s.offset + int64_t(s.samples), int64_t(merged->samples));
    }
    BOOST_CHECK_EQUAL(tone_slices, 1u);
    BOOST_CHECK_EQUAL(target_slices, 1u);
    // The target slices still know their slot, in every merged interval.
    bool slot3 = false, slot4 = false;
    for (const auto& v : iv)
        for (const auto& s : v.slices) {
            if (s.target && s.slot_index == 3)
                slot3 = true;
            if (s.target && s.slot_index == 4)
                slot4 = true;
        }
    BOOST_CHECK(slot3);
    BOOST_CHECK(slot4);

    // Tone windows do not drift: every tone window start is an exact multiple
    // of the period, independent of rounding.
    for (const auto& v : iv) {
        for (const auto& s : v.slices) {
            if (!s.tone_training)
                continue;
            const int64_t abs_start = v.start + s.offset;
            BOOST_CHECK_EQUAL(abs_start % tone_period, 0);
        }
    }
}
