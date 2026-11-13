/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * QA for the shared preamble reception front-end (`acquire_preamble`) and for
 * the `demodulate_one()` call site that now delegates to it.
 *
 * Contract: docs/performance/接口契约_统一SYNC_CIR核与公共前导前端_2026-11-12.md §2
 * Plan:     docs/performance/OpenCode_Sensing与Comm统一前端和CIR_开发方案_2026-11-12.md §4
 *
 * The cases below are the CFO-coverage gap regression (plan §4): the final
 * rotation must cover every legal CIR input window with the FINAL CFO, must
 * never leave a window on the bootstrap epoch, and must never rotate an
 * already-rotated buffer (a 2x phase slope).
 */

#include <boost/test/unit_test.hpp>

#include <gnuradio/uwb/uwb_demod_core.h>
#include <gnuradio/uwb/uwb_hrp_mod_core.h>

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdint>
#include <limits>
#include <vector>

namespace {

using namespace gr::uwb;
using namespace gr::uwb::demod;
using cf = std::complex<float>;

constexpr size_t kSeed = 2048;          // frame origin inside the window
constexpr size_t kSyncSamples = 1016;   // one SYNC symbol on the work grid
constexpr double kSampleRate = 998.4e6;
constexpr double kTwoPi = 6.283185307179586476925286766559;

struct Frame {
    std::vector<cf> iq;                  // seeded receive window
    std::vector<cf> tmpl;                // clean 1016-sample SYNC template
    std::vector<uint8_t> wire;           // PSDU incl. FCS
};

// Same construction as qa_uwb_fullpacket_fast_rx.cc: a complete 64-SYNC code-10
// frame with an IEEE FCS, placed at kSeed inside a zero-padded window.
Frame make_frame(size_t bytes, size_t code)
{
    Frame f;
    f.wire.resize(bytes - 2);
    for (size_t j = 0; j < f.wire.size(); ++j)
        f.wire[j] = static_cast<uint8_t>(19 * j + code);
    mod::append_ieee_fcs(f.wire);
    mod::HrpModConfig cfg;
    cfg.code_index = code;
    cfg.sync_repetitions = 64;
    cfg.sfd_mode = "4z2";
    cfg.peak_amplitude = .8f;
    const size_t count =
        mod::packet_samples_998p4(64, 8, f.wire.size(), false, &cfg.pulse);
    mod::HrpModScratch scratch;
    scratch.reserve(127, count);
    std::vector<cf> wave(count);
    size_t written = 0;
    BOOST_REQUIRE(mod::modulate_one(f.wire.data(), f.wire.size(), cfg, scratch,
                                    wave.data(), wave.size(), written));
    BOOST_REQUIRE_EQUAL(written, count);
    f.tmpl.assign(wave.begin(), wave.begin() + kSyncSamples);
    f.iq.assign(kSeed + count + 2048, cf(0));
    std::copy(wave.begin(), wave.end(), f.iq.begin() + kSeed);
    return f;
}

// Rotate the received signal by +w so the receiver's exp(-j w i) undoes it.
void apply_cfo(std::vector<cf>& iq, double cfo_hz)
{
    if (cfo_hz == 0.0)
        return;
    const double w = kTwoPi * cfo_hz / kSampleRate;
    for (size_t i = 0; i < iq.size(); ++i) {
        const double ph = w * static_cast<double>(i);
        iq[i] *= cf(static_cast<float>(std::cos(ph)),
                    static_cast<float>(std::sin(ph)));
    }
}

// Independent double-precision reference: dst[i] = src[i] * exp(-j w i).
std::vector<cf> reference_derotate(const std::vector<cf>& src, double cfo_hz)
{
    std::vector<cf> out(src.size());
    const double w = kTwoPi * cfo_hz / kSampleRate;
    for (size_t i = 0; i < src.size(); ++i) {
        const double ph = -w * static_cast<double>(i);
        const double c = std::cos(ph);
        const double s = std::sin(ph);
        out[i] = cf(static_cast<float>(src[i].real() * c - src[i].imag() * s),
                    static_cast<float>(src[i].real() * s + src[i].imag() * c));
    }
    return out;
}

// Contract §1.3 window arithmetic, re-derived here so the test does not trust
// the implementation it is checking.
void expected_union(int64_t origin, double P, size_t total, size_t skip,
                    size_t max_rep, size_t pre, size_t post, int64_t& lo,
                    int64_t& hi)
{
    size_t count = total - skip;
    if (max_rep > 0)
        count = std::min(count, max_rep);
    const int64_t wlen =
        static_cast<int64_t>(kSyncSamples) + static_cast<int64_t>(pre) +
        static_cast<int64_t>(post) - 1;
    lo = std::numeric_limits<int64_t>::max();
    hi = std::numeric_limits<int64_t>::min();
    for (size_t i = 0; i < count; ++i) {
        const size_t k = skip + i;
        const int64_t rs = origin + static_cast<int64_t>(std::llround(
                                        static_cast<double>(k) * P));
        const int64_t l = rs - static_cast<int64_t>(pre);
        lo = std::min(lo, l);
        hi = std::max(hi, l + wlen);
    }
}

// Contract §1.3 window arithmetic with the CLIP RULE applied: windows with
// lo < 0 or lo + wlen > n are dropped, and the result is the envelope of the
// survivors.  Returns false when no window survives.  This is the semantics of
// sync_cir::required_input_range, re-derived here so the test does not trust
// the implementation it is checking.
bool expected_clipped_union(int64_t origin,
                            double P,
                            size_t total,
                            size_t skip,
                            size_t max_rep,
                            size_t pre,
                            size_t post,
                            int64_t n,
                            int64_t& lo,
                            int64_t& hi)
{
    size_t count = total - skip;
    if (max_rep > 0)
        count = std::min(count, max_rep);
    const int64_t wlen =
        static_cast<int64_t>(kSyncSamples) + static_cast<int64_t>(pre) +
        static_cast<int64_t>(post) - 1;
    lo = std::numeric_limits<int64_t>::max();
    hi = std::numeric_limits<int64_t>::min();
    for (size_t i = 0; i < count; ++i) {
        const size_t k = skip + i;
        const int64_t rs = origin + static_cast<int64_t>(std::llround(
                                        static_cast<double>(k) * P));
        const int64_t l = rs - static_cast<int64_t>(pre);
        if (l < 0 || l + wlen > n)
            continue;
        lo = std::min(lo, l);
        hi = std::max(hi, l + wlen);
    }
    return hi > lo;
}

double relative_l2(const cf* a, const cf* b, size_t n)
{
    double num = 0.0;
    double den = 0.0;
    for (size_t i = 0; i < n; ++i) {
        const double dr = double(a[i].real()) - double(b[i].real());
        const double di = double(a[i].imag()) - double(b[i].imag());
        num += dr * dr + di * di;
        den += double(b[i].real()) * double(b[i].real()) +
               double(b[i].imag()) * double(b[i].imag());
    }
    return (den > 0.0) ? std::sqrt(num / den) : std::sqrt(num);
}

double wrap_pi(double x)
{
    while (x > M_PI)
        x -= kTwoPi;
    while (x < -M_PI)
        x += kTwoPi;
    return x;
}

// Cumulative rotation phase between two samples of the compensated buffer,
// measured against the raw input.  A single rotation gives -w*span, a double
// rotation gives -2w*span.
double measured_rotation_phase(const std::vector<cf>& raw,
                               const cf* derotated,
                               size_t lo,
                               size_t hi)
{
    const cf a = derotated[lo] * std::conj(raw[lo]);
    const cf b = derotated[hi] * std::conj(raw[hi]);
    return wrap_pi(double(std::arg(b)) - double(std::arg(a)));
}

demod::core::DemodScratch make_scratch(size_t n)
{
    demod::core::DemodScratch scratch;
    scratch.reserve(n);
    return scratch;
}

} // namespace

// ---------------------------------------------------------------------------
// 1. CFO coverage regression: with CirOnly, every first/middle/last SYNC window
//    inside [covered_lo, covered_hi) is compensated with the FINAL CFO, and no
//    sample is double-rotated.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(preamble_frontend_cir_coverage_uses_final_cfo)
{
    const Frame frame = make_frame(32, 10);
    const size_t pre = 16;
    const size_t post = 100;
    const size_t wlen = kSyncSamples + pre + post - 1;
    int64_t union_lo = 0;
    int64_t union_hi = 0;
    expected_union(static_cast<int64_t>(kSeed), double(kSyncSamples), 64, 0, 64,
                   pre, post, union_lo, union_hi);
    BOOST_REQUIRE(union_lo >= 0);
    BOOST_REQUIRE(union_hi <= static_cast<int64_t>(frame.iq.size()));

    for (const double cfo_hz : {0.0, 50000.0, -50000.0}) {
        // Both rotation kernels must satisfy the same coverage contract.
        for (const bool fast_rotation : {false, true}) {
            std::vector<cf> iq = frame.iq;
            apply_cfo(iq, cfo_hz);

            auto profile = Qm35825Profile::Default();
            profile.code_index = 10;
            profile.rx_fast_cfo_rotation = fast_rotation;

            sync_cir::SyncCirRequest req;
            req.cir_axis_origin = static_cast<int64_t>(kSeed);
            req.samples_per_symbol = double(kSyncSamples);
            req.total_repetitions = 64;
            req.skip_initial = 0;
            req.max_repetitions = 64;
            req.pre = pre;
            req.post = post;

            auto scratch = make_scratch(iq.size());
            PreambleRxContext ctx;
            const bool ok = acquire_preamble(iq.data(), iq.size(), profile,
                                             frame.tmpl, static_cast<int64_t>(kSeed),
                                             PreamblePurpose::CirOnly, &req,
                                             scratch, ctx);
            BOOST_TEST_MESSAGE("cfo=" << cfo_hz << " fast=" << fast_rotation
                               << " ok=" << ok << " epoch=" << ctx.cfo_epoch
                               << " final=" << ctx.final_cfo_hz
                               << " covered=[" << ctx.covered_lo << ","
                               << ctx.covered_hi << ") union=[" << ctx.cir_union_lo
                               << "," << ctx.cir_union_hi << ")");
            BOOST_REQUIRE(ok);
            BOOST_REQUIRE_EQUAL(int(ctx.status), int(DemodStatus::Success));
            BOOST_CHECK(ctx.ok);
            // Epoch 2 == the buffer is compensated with the FINAL CFO.
            BOOST_CHECK_EQUAL(ctx.cfo_epoch, 2u);
            BOOST_CHECK(ctx.derotated_valid);
            BOOST_CHECK_EQUAL(ctx.phase_reference_sample, int64_t(0));

            // The planner's union is the contract arithmetic, and the coverage
            // brackets it (plus the documented halo).
            BOOST_CHECK_EQUAL(ctx.cir_union_lo, union_lo);
            BOOST_CHECK_EQUAL(ctx.cir_union_hi, union_hi);
            BOOST_CHECK_LE(ctx.covered_lo, size_t(union_lo));
            BOOST_CHECK_GE(ctx.covered_hi, size_t(union_hi));
            // Lower edge: the union minus at most the halo.
            BOOST_CHECK_GE(int64_t(ctx.covered_lo), union_lo - int64_t(128));
            // The coverage is the union of the CIR windows AND the final SFD
            // search range, so it may extend past the CIR union.  It must stay
            // inside the buffer and must still contain the whole SFD.
            BOOST_CHECK_LE(ctx.covered_hi, frame.iq.size());
            BOOST_CHECK_LE(int64_t(ctx.covered_lo), int64_t(ctx.sfd.sfd_start_sample));
            BOOST_CHECK_GE(int64_t(ctx.covered_hi), int64_t(ctx.sfd.sfd_end_sample));

            // The final CFO is the injected one (this is what makes the
            // comparison below meaningful).
            BOOST_CHECK_SMALL(ctx.final_cfo_hz - cfo_hz, 2000.0);
            BOOST_CHECK_EQUAL(ctx.samples_per_symbol, double(kSyncSamples));

            const std::vector<cf> ref = reference_derotate(iq, ctx.final_cfo_hz);
            const cf* dst = scratch.derotated.data();

            for (const size_t k : {size_t(0), size_t(32), size_t(63)}) {
                const int64_t lo = static_cast<int64_t>(kSeed) +
                    static_cast<int64_t>(std::llround(double(k) *
                                                      double(kSyncSamples))) -
                    static_cast<int64_t>(pre);
                BOOST_REQUIRE_GE(lo, int64_t(ctx.covered_lo));
                BOOST_REQUIRE_LE(lo + int64_t(wlen), int64_t(ctx.covered_hi));
                const double err =
                    relative_l2(dst + lo, ref.data() + lo, wlen);
                BOOST_TEST_MESSAGE("  sync " << k << " lo=" << lo
                                   << " rel_l2=" << err);
                BOOST_CHECK_LT(err, 2e-5);
            }

            // No double rotation: the cumulative rotation phase over a 4096
            // sample span inside the coverage is -w*span, not -2w*span.  The
            // span starts one SYNC into the frame so it is never zero-padding
            // (arg(0) carries no phase).
            if (cfo_hz != 0.0) {
                const size_t span_lo = kSeed + kSyncSamples;
                const size_t span_hi = span_lo + 4096;
                BOOST_REQUIRE_GE(span_lo, ctx.covered_lo);
                BOOST_REQUIRE_LE(span_hi, ctx.covered_hi);
                const double w = kTwoPi * ctx.final_cfo_hz / kSampleRate;
                const double span = double(span_hi - span_lo);
                const double measured =
                    measured_rotation_phase(iq, dst, span_lo, span_hi);
                const double single = wrap_pi(-w * span);
                const double dbl = wrap_pi(-2.0 * w * span);
                BOOST_TEST_MESSAGE("  rotation phase measured=" << measured
                                   << " single=" << single << " double=" << dbl);
                BOOST_CHECK_LT(std::abs(wrap_pi(measured - single)), 0.05);
                BOOST_CHECK_GT(std::abs(wrap_pi(measured - dbl)), 0.5);
            }
        }
    }
}

// ---------------------------------------------------------------------------
// 2. Epoch invalidation: when the bootstrap CFO differs from the final CFO the
//    buffer must be re-rotated from raw, not left on epoch 1.
//
//    The two fits normally see the same tail peaks, so this case makes them
//    differ deterministically: with cfo_skip_initial_repetitions = 0 the final
//    SFD-anchored grid feeds arg(0) = 0 for the leading peaks it did not
//    measure, while the bootstrap fit uses their real phases.  That is an
//    artificial profile, chosen only to separate the two epochs.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(preamble_frontend_final_epoch_replaces_bootstrap_coverage)
{
    const Frame frame = make_frame(32, 10);
    const double cfo_hz = 50000.0;
    std::vector<cf> iq = frame.iq;
    apply_cfo(iq, cfo_hz);
    // Make the bootstrap fit genuinely differ from the final fit.  The final
    // SFD-anchored refit only ever sees the MEASURED tail `[tail_first, 64)`
    // (tail_first = 24); the bootstrap fit sees every repetition the tracker
    // acquired with `cfo_skip_initial_repetitions = 0`.  Giving the leading 24
    // repetitions an extra carrier offset therefore separates the two epochs
    // for real, instead of relying on the zero-filled `peak_corr` placeholders
    // that the measured-only CFO fit now (correctly) refuses to read.
    {
        const double extra_hz = 20000.0;
        const double w = kTwoPi * extra_hz / kSampleRate;
        const size_t lead = 24 * kSyncSamples;
        for (size_t i = 0; i < lead && kSeed + i < iq.size(); ++i) {
            const double ph = w * static_cast<double>(i);
            iq[kSeed + i] *= cf(static_cast<float>(std::cos(ph)),
                                static_cast<float>(std::sin(ph)));
        }
    }

    auto profile = Qm35825Profile::Default();
    profile.code_index = 10;
    profile.cfo_skip_initial_repetitions = 0;
    profile.cfo_min_fit_repetitions = 2;

    sync_cir::SyncCirRequest req;
    req.cir_axis_origin = static_cast<int64_t>(kSeed);
    req.samples_per_symbol = double(kSyncSamples);
    req.total_repetitions = 64;
    req.skip_initial = 0;
    req.max_repetitions = 64;
    req.pre = 16;
    req.post = 100;

    auto scratch = make_scratch(iq.size());
    PreambleRxContext ctx;
    const bool ok = acquire_preamble(iq.data(), iq.size(), profile, frame.tmpl,
                                     static_cast<int64_t>(kSeed),
                                     PreamblePurpose::CirOnly, &req, scratch, ctx);
    BOOST_TEST_MESSAGE("bootstrap=" << ctx.bootstrap_cfo_hz
                       << " final=" << ctx.final_cfo_hz
                       << " epoch=" << ctx.cfo_epoch);
    BOOST_REQUIRE(ok);
    BOOST_CHECK_EQUAL(ctx.cfo_epoch, 2u);
    BOOST_CHECK(ctx.derotated_valid);
    // The final SFD-anchored refit must land on the true target CFO; the
    // bootstrap fit saw the extra leading offset and must disagree with it.
    BOOST_CHECK_SMALL(ctx.final_cfo_hz - cfo_hz, 2000.0);
    BOOST_CHECK_GT(std::abs(ctx.final_cfo_hz - ctx.bootstrap_cfo_hz), 500.0);

    int64_t union_lo = 0;
    int64_t union_hi = 0;
    expected_union(static_cast<int64_t>(kSeed), double(kSyncSamples), 64, 0, 64,
                   16, 100, union_lo, union_hi);
    BOOST_CHECK_EQUAL(ctx.cir_union_lo, union_lo);
    BOOST_CHECK_EQUAL(ctx.cir_union_hi, union_hi);
    BOOST_CHECK_LE(ctx.covered_lo, size_t(union_lo));
    BOOST_CHECK_GE(ctx.covered_hi, size_t(union_hi));

    // Every window must match the FINAL-CFO reference and must NOT match the
    // bootstrap-CFO one: that is what "no stale epoch" means numerically.
    const std::vector<cf> ref_final = reference_derotate(iq, ctx.final_cfo_hz);
    const std::vector<cf> ref_bootstrap =
        reference_derotate(iq, ctx.bootstrap_cfo_hz);
    const cf* dst = scratch.derotated.data();
    const size_t wlen = kSyncSamples + 16 + 100 - 1;
    for (const size_t k : {size_t(0), size_t(32), size_t(63)}) {
        const int64_t lo = static_cast<int64_t>(kSeed) +
            static_cast<int64_t>(std::llround(double(k) * double(kSyncSamples))) -
            16;
        BOOST_REQUIRE_GE(lo, int64_t(ctx.covered_lo));
        BOOST_REQUIRE_LE(lo + int64_t(wlen), int64_t(ctx.covered_hi));
        const double err_final = relative_l2(dst + lo, ref_final.data() + lo, wlen);
        const double err_bootstrap =
            relative_l2(dst + lo, ref_bootstrap.data() + lo, wlen);
        BOOST_TEST_MESSAGE("  sync " << k << " rel_l2(final)=" << err_final
                           << " rel_l2(bootstrap)=" << err_bootstrap);
        BOOST_CHECK_LT(err_final, 2e-5);
        BOOST_CHECK_GT(err_bootstrap, 1e-3);
    }
}

// ---------------------------------------------------------------------------
// 3. Coverage insufficiency: a union that cannot be covered fails closed and
//    never advertises stale data.
//
// The clip rule is part of the contract: `required_input_range` returns the
// envelope of the windows that SURVIVE the buffer-edge clip, and reports
// failure only when no window survives.  A partially clipped request therefore
// succeeds on the surviving subset (case b2); that is not a relaxation, it is
// the same rule `estimate_sync_cir` applies to the samples it accumulates.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(preamble_frontend_cir_coverage_insufficiency_fails_closed)
{
    const Frame frame = make_frame(32, 10);
    auto profile = Qm35825Profile::Default();
    profile.code_index = 10;

    auto run = [&](const sync_cir::SyncCirRequest* req, PreambleRxContext& ctx,
                   demod::core::DemodScratch& scratch) {
        return acquire_preamble(frame.iq.data(), frame.iq.size(), profile,
                                frame.tmpl, static_cast<int64_t>(kSeed),
                                PreamblePurpose::CirOnly, req, scratch, ctx);
    };

    // (a) every window runs past the buffer end -> nothing survives.
    {
        sync_cir::SyncCirRequest req;
        req.cir_axis_origin = static_cast<int64_t>(frame.iq.size()) - 200;
        req.samples_per_symbol = double(kSyncSamples);
        req.total_repetitions = 64;
        req.skip_initial = 0;
        req.max_repetitions = 64;
        req.pre = 16;
        req.post = 100;
        auto scratch = make_scratch(frame.iq.size());
        PreambleRxContext ctx;
        BOOST_CHECK(!run(&req, ctx, scratch));
        BOOST_CHECK_EQUAL(int(ctx.status), int(DemodStatus::CirFailed));
        BOOST_CHECK(!ctx.derotated_valid);
        BOOST_CHECK_EQUAL(ctx.covered_lo, size_t(0));
        BOOST_CHECK_EQUAL(ctx.covered_hi, size_t(0));
        // No union is advertised at all, not even a clipped one.
        BOOST_CHECK_EQUAL(ctx.cir_union_lo, int64_t(-1));
        BOOST_CHECK_EQUAL(ctx.cir_union_hi, int64_t(-1));
    }

    // (b) the only requested window starts before the buffer origin -> the
    //     request is entirely clipped, so it still fails closed.
    {
        sync_cir::SyncCirRequest req;
        req.cir_axis_origin = 4;
        req.samples_per_symbol = double(kSyncSamples);
        req.total_repetitions = 1;
        req.skip_initial = 0;
        req.max_repetitions = 1;
        req.pre = 16;
        req.post = 100;
        auto scratch = make_scratch(frame.iq.size());
        PreambleRxContext ctx;
        BOOST_CHECK(!run(&req, ctx, scratch));
        BOOST_CHECK_EQUAL(int(ctx.status), int(DemodStatus::CirFailed));
        BOOST_CHECK(!ctx.derotated_valid);
        BOOST_CHECK_EQUAL(ctx.covered_lo, size_t(0));
        BOOST_CHECK_EQUAL(ctx.covered_hi, size_t(0));
        BOOST_CHECK_EQUAL(ctx.cir_union_lo, int64_t(-1));
        BOOST_CHECK_EQUAL(ctx.cir_union_hi, int64_t(-1));
    }

    // (b2) partially clipped: the first repetition falls before the origin, the
    //      remaining 63 are usable, so the coverage is planned on exactly those
    //      survivors and the reported union must match the clipped envelope.
    {
        sync_cir::SyncCirRequest req;
        req.cir_axis_origin = 4;
        req.samples_per_symbol = double(kSyncSamples);
        req.total_repetitions = 64;
        req.skip_initial = 0;
        req.max_repetitions = 64;
        req.pre = 16;
        req.post = 100;
        auto scratch = make_scratch(frame.iq.size());
        PreambleRxContext ctx;
        int64_t want_lo = 0;
        int64_t want_hi = 0;
        BOOST_REQUIRE(expected_clipped_union(
            4, double(kSyncSamples), 64, 0, 64, 16, 100,
            static_cast<int64_t>(frame.iq.size()), want_lo, want_hi));
        BOOST_REQUIRE(run(&req, ctx, scratch));
        BOOST_CHECK_EQUAL(int(ctx.status), int(DemodStatus::Success));
        BOOST_CHECK(ctx.derotated_valid);
        BOOST_CHECK_EQUAL(ctx.cir_union_lo, want_lo);
        BOOST_CHECK_EQUAL(ctx.cir_union_hi, want_hi);
        // The halo-expanded coverage contains the whole surviving union.
        BOOST_CHECK_LE(ctx.covered_lo, size_t(want_lo));
        BOOST_CHECK_GE(ctx.covered_hi, size_t(want_hi));
        BOOST_CHECK_EQUAL(ctx.covered_lo,
                          size_t(std::max<int64_t>(0, want_lo - 128)));
    }

    // (c) CirOnly without a request is a planning failure, not a silent
    //     "no CIR" success.
    {
        auto scratch = make_scratch(frame.iq.size());
        PreambleRxContext ctx;
        BOOST_CHECK(!run(nullptr, ctx, scratch));
        BOOST_CHECK_EQUAL(int(ctx.status), int(DemodStatus::CirFailed));
        BOOST_CHECK(!ctx.derotated_valid);
        BOOST_CHECK_EQUAL(ctx.covered_lo, size_t(0));
        BOOST_CHECK_EQUAL(ctx.covered_hi, size_t(0));
    }

    // (d) a malformed request (zero period) is rejected the same way.
    {
        sync_cir::SyncCirRequest req;
        req.cir_axis_origin = static_cast<int64_t>(kSeed);
        req.samples_per_symbol = 0.0;
        req.total_repetitions = 64;
        req.skip_initial = 0;
        req.pre = 16;
        req.post = 100;
        auto scratch = make_scratch(frame.iq.size());
        PreambleRxContext ctx;
        BOOST_CHECK(!run(&req, ctx, scratch));
        BOOST_CHECK_EQUAL(int(ctx.status), int(DemodStatus::CirFailed));
        BOOST_CHECK(!ctx.derotated_valid);
    }
}

// ---------------------------------------------------------------------------
// 4. SyncSfdOnly early stop: no CIR / NS-SFD / PHR / payload work, and
//    stage_total_us is exactly timing + cfo + sfd.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(preamble_frontend_sync_sfd_only_stops_after_sfd)
{
    const Frame frame = make_frame(32, 10);
    auto profile = Qm35825Profile::Default();
    profile.code_index = 10;
    profile.rx_stop_after_sfd = true;

    auto scratch = make_scratch(frame.iq.size());
    const auto res = demod::core::demodulate_one(
        frame.iq.data(), frame.iq.size(), profile, 1, static_cast<int64_t>(kSeed),
        0, frame.tmpl, scratch);

    BOOST_REQUIRE_EQUAL(int(res.status), int(DemodStatus::Success));
    BOOST_CHECK(res.timing.ok);
    BOOST_CHECK(res.cfo.ok);
    BOOST_CHECK(res.sfd.ok);
    BOOST_CHECK_EQUAL(res.stage_total_us,
                      res.stage_timing_us + res.stage_cfo_us + res.stage_sfd_us);
    // Nothing past the final SFD ran.
    BOOST_CHECK_EQUAL(res.stage_cir_us, uint64_t(0));
    BOOST_CHECK_EQUAL(res.stage_ns_sfd_us, uint64_t(0));
    BOOST_CHECK_EQUAL(res.stage_phr_us, uint64_t(0));
    BOOST_CHECK_EQUAL(res.stage_payload_us, uint64_t(0));
    BOOST_CHECK(!res.cir.ok);
    BOOST_CHECK_EQUAL(res.cir.soft_chip_count, size_t(0));
    BOOST_CHECK(res.cir.cir_values.empty());
    BOOST_CHECK(!res.ns_sfd.ok);
    BOOST_CHECK_EQUAL(res.ns_sfd.sfd_start_chip, int64_t(-1));
    BOOST_CHECK(!res.phr.ok);
    BOOST_CHECK_EQUAL(res.phr.psdu_length, uint32_t(0));
    BOOST_CHECK(!res.payload.ok);
    BOOST_CHECK(res.payload.bytes.empty());
    // The preamble itself is fully anchored.
    BOOST_CHECK_EQUAL(res.timing.preamble_start_sample, int64_t(kSeed));
    BOOST_CHECK_EQUAL(res.sfd.sfd_start_sample, int64_t(67072));
}

// ---------------------------------------------------------------------------
// 5. FullPacket unchanged: the published fields of a clean frame still match
//    the pre-refactor baseline captured once for this fixture (64-SYNC code-10,
//    32 B PSDU, seed 2048, 4z2 SFD).
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(preamble_frontend_full_packet_fields_match_baseline)
{
    const Frame frame = make_frame(32, 10);
    auto profile = Qm35825Profile::Default();
    profile.code_index = 10;

    auto scratch = make_scratch(frame.iq.size());
    const auto res = demod::core::demodulate_one(
        frame.iq.data(), frame.iq.size(), profile, 7, static_cast<int64_t>(kSeed),
        0, frame.tmpl, scratch);

    BOOST_REQUIRE_EQUAL(int(res.status), int(DemodStatus::Success));
    BOOST_CHECK(res.payload.fcs_pass);
    BOOST_CHECK(res.payload.bytes == frame.wire);

    // Captured from the pre-refactor path (see the wave-B report).
    BOOST_CHECK_EQUAL(res.timing.preamble_start_sample, int64_t(2048));
    BOOST_CHECK_EQUAL(res.timing.measured_period, 1016.0);
    BOOST_CHECK_EQUAL(res.timing.detected_peaks, size_t(64));
    BOOST_CHECK_EQUAL(res.cfo.cfo_hz, 0.0);
    BOOST_CHECK_EQUAL(res.cfo.skipped_peaks, size_t(24));
    BOOST_CHECK_EQUAL(res.cfo.peaks_used, size_t(40));
    BOOST_CHECK_EQUAL(res.cfo.fit_first_peak_sample, int64_t(27447));
    BOOST_CHECK_EQUAL(res.cfo.fit_last_peak_sample, int64_t(67071));
    BOOST_CHECK_EQUAL(res.sfd.sfd_start_sample, int64_t(67072));
    BOOST_CHECK_EQUAL(res.sfd.sfd_end_sample, int64_t(75199));
    BOOST_CHECK_EQUAL(res.sfd.expected_start_sample, int64_t(67072));
    BOOST_CHECK_EQUAL(res.sfd.polarity, 1);
    BOOST_CHECK_EQUAL(res.sfd.search_windows, uint64_t(1));
    BOOST_CHECK_EQUAL(res.cir.first_path_sample, size_t(2050));
    BOOST_CHECK_EQUAL(res.cir.pre_samples, size_t(8));
    BOOST_CHECK_EQUAL(res.cir.post_samples, size_t(30));
    BOOST_CHECK_EQUAL(res.cir.cir_values.size(), size_t(38));
    BOOST_CHECK_EQUAL(res.cir.soft_chip_count, size_t(67794));
    BOOST_CHECK_EQUAL(res.cir.samples_per_chip, 2.0);
    BOOST_CHECK_EQUAL(res.ns_sfd.sfd_start_chip, int64_t(32512));
    BOOST_CHECK_EQUAL(res.ns_sfd.sfd_end_chip, int64_t(36575));
    BOOST_CHECK_EQUAL(res.phr.psdu_length, uint32_t(32));
    BOOST_CHECK_EQUAL(res.payload.received_fcs, uint16_t(0xf1b3));
    BOOST_CHECK_EQUAL(res.payload.calculated_fcs, uint16_t(0xf1b3));
    BOOST_CHECK_EQUAL(res.sfd_bootstrap_windows, uint64_t(1));
    BOOST_CHECK_EQUAL(res.sfd_final_windows, uint64_t(1));
    BOOST_CHECK_EQUAL(res.sfd_initial_predicted_sample, int64_t(67072));
    BOOST_CHECK_EQUAL(res.sfd_bootstrap_detected_sample, int64_t(67072));
    BOOST_CHECK_EQUAL(res.stage_tail_retrack_fallbacks, uint32_t(0));
    // Both CFO passes ran and the second one covered the whole frame (the
    // legacy non-on-demand rule), exactly as before the refactor.
    BOOST_CHECK_EQUAL(res.stage_cfo_rotation_passes, uint32_t(2));
    BOOST_CHECK_GE(res.stage_cfo_rotated_samples, uint64_t(frame.iq.size()));
    // Child sub-timers still partition the SFD parent and are never added to
    // the total a second time.
    const uint64_t children = res.stage_sfd_bootstrap_us +
        res.stage_sfd_tail_retrack_us + res.stage_sfd_cfo_refit_us +
        res.stage_sfd_final_us;
    BOOST_CHECK_LE(children, res.stage_sfd_us);
    BOOST_CHECK_EQUAL(children + res.stage_sfd_anchor_other_us,
                      res.stage_sfd_us);
    BOOST_CHECK_LE(res.stage_timing_us + res.stage_cfo_us + res.stage_sfd_us +
                       res.stage_cir_us + res.stage_ns_sfd_us +
                       res.stage_phr_us + res.stage_payload_us +
                       res.stage_softchip_fallback_us,
                   res.stage_total_us);

    // A payload digest keeps the whole PSDU pinned without listing 32 bytes.
    unsigned long long h = 1469598103934665603ull;
    for (uint8_t b : res.payload.bytes) {
        h ^= b;
        h *= 1099511628211ull;
    }
    BOOST_CHECK_EQUAL(h, 0x038fcbba6660e4baull);
}
