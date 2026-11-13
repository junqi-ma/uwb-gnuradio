/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * R01 QA — the shared SYNC-CIR kernel cache must be invalidated from the
 * shared object's OWN identity, never from a scratch field that the same call
 * wrote.
 *
 * The defect this file pins down (reviewer-reproduced):
 *   `stage_cir_softchips()` used `scratch.cir_template.key != ckey` as the
 *   "is the shared kernel stale?" test, but `rxopt::build_sparse_cir_template()`
 *   a few lines above had already written `scratch.cir_template.key = ckey`.
 *   The predicate therefore always looked fresh, `prepare_sync_code()` was
 *   skipped, and a scratch reused for code 9 then code 10 kept code 9's
 *   ternary template while reporting success.
 *
 * Every case below drives the REAL call site (`demodulate_one()` ->
 * `stage_cir_softchips()`) on one long-lived `DemodScratch` and compares the
 * complete observable result against the same packet on a FRESH scratch.  The
 * comparison is exact (bit-level): the kernel is deterministic for identical
 * input, so any difference is a cache-identity bug, not numerics.
 *
 * Covered:
 *   1. sequential codes 9 -> 10 -> 11 -> 12 -> 9 on one scratch
 *   2. the exact failing case: same tap count, different code
 *   3. 38 -> 116 -> 38 taps on one scratch (capacity growth then shrink)
 *   4. same total tap count, different pre/post split (16/100 -> 100/16)
 *   5. failure then recovery on the same scratch
 *   6. N identical packets must prepare the kernel exactly once
 */

#include <boost/test/unit_test.hpp>

#include <gnuradio/gr_complex.h>
#include <gnuradio/uwb/uwb_demod_core.h>
#include <gnuradio/uwb/uwb_hrp_mod_core.h>
#include <gnuradio/uwb/uwb_phy_profile.h>
#include <gnuradio/uwb/uwb_sync_cir_core.h>

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

namespace {

using gr::uwb::demod::CirResult;
using gr::uwb::demod::DemodResult;
using gr::uwb::demod::Qm35825Profile;
using gr::uwb::demod::core::DemodScratch;
using gr::uwb::demod::core::demodulate_one;
using gr::uwb::demod::core::stage_cir_softchips;
using gr::uwb::mod::HrpModConfig;
using gr::uwb::mod::HrpModScratch;
using gr::uwb::mod::modulate_one;
using gr::uwb::mod::packet_samples_998p4;

constexpr size_t kT0 = 9984;        // absolute packet origin inside the window
constexpr size_t kSyncReps = 64;    // 64-SYNC preamble
constexpr size_t kSfdSymbols = 8;   // 4z2 SFD is 8 symbols
constexpr size_t kSyncSamples = 1016;
constexpr size_t kTail = 8192;      // trailing guard so the payload fits

// ---------------------------------------------------------------------------
// TX side: one modulated packet plus its own 1016-sample SYNC template.
// The template must belong to the same code, otherwise the timing stage would
// not lock and the CIR stage would never run.
// ---------------------------------------------------------------------------
struct TxPacket {
    size_t code_index = 9;
    std::vector<gr_complex> iq;   // window buffer, packet at kT0
    std::vector<gr_complex> tmpl; // 1016-sample SYNC waveform of this code
    std::vector<uint8_t> psdu;
};

TxPacket make_packet(size_t code_index, size_t psdu_len, uint32_t seed)
{
    TxPacket p;
    p.code_index = code_index;
    p.psdu = gr::uwb::mod::make_random_psdu(psdu_len, seed, true);

    HrpModConfig cfg;
    cfg.code_index = code_index;
    cfg.sync_repetitions = kSyncReps;
    cfg.sfd_mode = "4z2";
    HrpModScratch mod_scratch;
    const size_t want = packet_samples_998p4(kSyncReps, kSfdSymbols,
                                             p.psdu.size(), false, &cfg.pulse);
    mod_scratch.reserve(p.psdu.size(), want);
    std::vector<gr_complex> tx(want);
    size_t n = 0;
    BOOST_REQUIRE_MESSAGE(modulate_one(p.psdu.data(), p.psdu.size(), cfg,
                                       mod_scratch, tx.data(), tx.size(), n),
                          "modulate_one failed for code " << code_index);
    BOOST_REQUIRE_EQUAL(n, want);

    p.tmpl.assign(tx.begin(), tx.begin() + kSyncSamples);
    p.iq.assign(kT0 + n + kTail, gr_complex(0.0f, 0.0f));
    std::copy(tx.begin(), tx.end(),
              p.iq.begin() + static_cast<std::ptrdiff_t>(kT0));
    return p;
}

Qm35825Profile make_profile(size_t code_index, size_t pre, size_t post,
                            size_t reps = kSyncReps)
{
    Qm35825Profile prof = Qm35825Profile::Default();
    prof.code_index = code_index;
    prof.sfd_mode = "4z2";
    prof.cir_pre_samples = pre;
    prof.cir_post_samples = post;
    prof.preamble_repetitions = reps;
    return prof;
}

DemodResult run_packet(const TxPacket& p, size_t pre, size_t post,
                       DemodScratch& scratch, size_t reps = kSyncReps)
{
    const Qm35825Profile prof = make_profile(p.code_index, pre, post, reps);
    return demodulate_one(p.iq.data(), p.iq.size(), prof, 1,
                          static_cast<int64_t>(kT0), 0, p.tmpl, scratch);
}

DemodScratch fresh_scratch(const TxPacket& p)
{
    DemodScratch s;
    s.reserve(p.iq.size());
    return s;
}

// ---------------------------------------------------------------------------
// Exact comparison helpers.  `rel_l2` is reported only to make a failure
// legible; the assertion itself is bit-exact (== 0).
// ---------------------------------------------------------------------------
double rel_l2(const std::vector<float>& a, const std::vector<float>& b)
{
    if (a.size() != b.size())
        return std::numeric_limits<double>::infinity();
    double num = 0.0;
    double den = 0.0;
    for (size_t i = 0; i < a.size(); ++i) {
        const double d = static_cast<double>(a[i]) - static_cast<double>(b[i]);
        num += d * d;
        den += static_cast<double>(b[i]) * static_cast<double>(b[i]);
    }
    return std::sqrt(num / (den + 1e-30));
}

double rel_l2(const std::vector<std::complex<float>>& a,
              const std::vector<std::complex<float>>& b)
{
    if (a.size() != b.size())
        return std::numeric_limits<double>::infinity();
    double num = 0.0;
    double den = 0.0;
    for (size_t i = 0; i < a.size(); ++i) {
        const double dr =
            static_cast<double>(a[i].real()) - static_cast<double>(b[i].real());
        const double di =
            static_cast<double>(a[i].imag()) - static_cast<double>(b[i].imag());
        num += dr * dr + di * di;
        den += static_cast<double>(b[i].real()) * b[i].real() +
               static_cast<double>(b[i].imag()) * b[i].imag();
    }
    return std::sqrt(num / (den + 1e-30));
}

double max_abs_diff(const std::vector<float>& a, const std::vector<float>& b)
{
    if (a.size() != b.size())
        return std::numeric_limits<double>::infinity();
    double m = 0.0;
    for (size_t i = 0; i < a.size(); ++i)
        m = std::max(m, std::abs(static_cast<double>(a[i]) -
                                 static_cast<double>(b[i])));
    return m;
}

double max_abs_diff(const std::vector<std::complex<float>>& a,
                    const std::vector<std::complex<float>>& b)
{
    if (a.size() != b.size())
        return std::numeric_limits<double>::infinity();
    double m = 0.0;
    for (size_t i = 0; i < a.size(); ++i)
        m = std::max(m, static_cast<double>(std::abs(a[i] - b[i])));
    return m;
}

std::string num(double v)
{
    std::ostringstream os;
    os << std::setprecision(17) << v;
    return os.str();
}

// Every observable field the CIR cache can corrupt, plus the decoded frame.
void check_matches_fresh(const std::string& tag,
                         const DemodResult& got,
                         const DemodResult& want)
{
    BOOST_CHECK_MESSAGE(
        static_cast<int>(got.status) == static_cast<int>(want.status),
        tag << ": status " << static_cast<int>(got.status) << " != fresh "
            << static_cast<int>(want.status));

    // ---- CIR taps (the field the bug corrupted) ----
    BOOST_REQUIRE_MESSAGE(got.cir.cir_values.size() ==
                              want.cir.cir_values.size(),
                          tag << ": cir_values size "
                              << got.cir.cir_values.size() << " != fresh "
                              << want.cir.cir_values.size());
    const double l2 = rel_l2(got.cir.cir_values, want.cir.cir_values);
    const double linf = max_abs_diff(got.cir.cir_values, want.cir.cir_values);
    BOOST_CHECK_MESSAGE(
        l2 == 0.0,
        tag << ": cir_values differs from fresh scratch: relative L2 = "
            << num(l2) << ", max|diff| = " << num(linf));

    BOOST_REQUIRE_MESSAGE(got.cir.cir_filter_taps.size() ==
                              want.cir.cir_filter_taps.size(),
                          tag << ": cir_filter_taps size "
                              << got.cir.cir_filter_taps.size() << " != fresh "
                              << want.cir.cir_filter_taps.size());
    const double tap_l2 =
        rel_l2(got.cir.cir_filter_taps, want.cir.cir_filter_taps);
    BOOST_CHECK_MESSAGE(
        tap_l2 == 0.0,
        tag << ": cir_filter_taps differs from fresh scratch: relative L2 = "
            << num(tap_l2));

    BOOST_CHECK_MESSAGE(got.cir.first_path_sample == want.cir.first_path_sample,
                        tag << ": first_path_sample "
                            << got.cir.first_path_sample << " != fresh "
                            << want.cir.first_path_sample);
    BOOST_CHECK_MESSAGE(
        got.cir.cir_peak_metric == want.cir.cir_peak_metric,
        tag << ": cir_peak_metric " << num(got.cir.cir_peak_metric)
            << " != fresh " << num(want.cir.cir_peak_metric));
    BOOST_CHECK_MESSAGE(got.cir.soft_chip_count == want.cir.soft_chip_count,
                        tag << ": soft_chip_count " << got.cir.soft_chip_count
                            << " != fresh " << want.cir.soft_chip_count);

    // ---- decoded frame ----
    BOOST_CHECK_MESSAGE(got.payload.fcs_pass == want.payload.fcs_pass,
                        tag << ": fcs_pass " << got.payload.fcs_pass
                            << " != fresh " << want.payload.fcs_pass);
    BOOST_CHECK_MESSAGE(
        got.payload.received_fcs == want.payload.received_fcs,
        tag << ": received_fcs " << got.payload.received_fcs << " != fresh "
            << want.payload.received_fcs);
    BOOST_CHECK_MESSAGE(
        got.payload.bytes == want.payload.bytes,
        tag << ": decoded PSDU differs from fresh scratch ("
            << got.payload.bytes.size() << " vs " << want.payload.bytes.size()
            << " bytes)");
}

// Convenience: run on the shared scratch and on a fresh scratch, then compare.
void check_against_fresh(const std::string& tag, const TxPacket& p, size_t pre,
                         size_t post, DemodScratch& shared)
{
    DemodScratch fresh = fresh_scratch(p);
    const DemodResult want = run_packet(p, pre, post, fresh);
    const DemodResult got = run_packet(p, pre, post, shared);
    check_matches_fresh(tag, got, want);
}

} // namespace

// ---------------------------------------------------------------------------
// 1. Sequential codes 9 -> 10 -> 11 -> 12 -> 9 on ONE scratch.
//    Every step must equal a fresh-scratch run of the same packet.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_sync_cir_cache_sequential_codes)
{
    const size_t pre = 8;
    const size_t post = 30; // production 38-tap window
    TxPacket p9 = make_packet(9, 16, 9001u);
    TxPacket p10 = make_packet(10, 16, 10001u);
    TxPacket p11 = make_packet(11, 16, 11001u);
    TxPacket p12 = make_packet(12, 16, 12001u);

    DemodScratch shared = fresh_scratch(p9);

    check_against_fresh("code9 (first use)", p9, pre, post, shared);
    check_against_fresh("code10 after code9", p10, pre, post, shared);
    check_against_fresh("code11 after code10", p11, pre, post, shared);
    check_against_fresh("code12 after code11", p12, pre, post, shared);
    check_against_fresh("code9 after code12", p9, pre, post, shared);

    // 5 distinct code transitions => 5 genuine cold preparations, never one
    // per packet beyond that.
    BOOST_CHECK_EQUAL(shared.sync_cir_prepares, 5u);
    BOOST_CHECK(shared.sync_cir_code.ready);
}

// ---------------------------------------------------------------------------
// 2. The exact failing case: identical geometry, different ternary code.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_sync_cir_cache_same_tap_count_different_code)
{
    const size_t pre = 8;
    const size_t post = 30;
    TxPacket p9 = make_packet(9, 16, 4242u);
    TxPacket p10 = make_packet(10, 16, 4243u);

    DemodScratch shared = fresh_scratch(p9);
    const DemodResult r9 = run_packet(p9, pre, post, shared);
    BOOST_REQUIRE_MESSAGE(r9.status == gr::uwb::demod::DemodStatus::Success,
                          "code9 reference packet did not decode, status="
                              << static_cast<int>(r9.status));
    BOOST_REQUIRE(r9.payload.fcs_pass);
    const uint32_t prepares_after_9 = shared.sync_cir_prepares;

    DemodScratch fresh10 = fresh_scratch(p10);
    const DemodResult want = run_packet(p10, pre, post, fresh10);
    BOOST_REQUIRE_MESSAGE(want.status == gr::uwb::demod::DemodStatus::Success,
                          "code10 reference packet did not decode, status="
                              << static_cast<int>(want.status));
    BOOST_REQUIRE(want.payload.fcs_pass);

    const DemodResult got = run_packet(p10, pre, post, shared);
    check_matches_fresh("code10 reusing a code9 scratch", got, want);

    // A different ternary code is a genuine identity miss: exactly one new
    // preparation, not zero (stale template) and not two.
    BOOST_CHECK_EQUAL(shared.sync_cir_prepares, prepares_after_9 + 1);
}

// ---------------------------------------------------------------------------
// 3. 38 -> 116 -> 38 taps on one scratch.
//    The 116-tap geometry exceeds the soft-chip FIR's 64-tap stack window, so
//    the packet fails at the FIR; the CIR taps are still produced and are what
//    the cache governs.  Both runs must agree exactly at every step.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_sync_cir_cache_tap_count_growth_and_shrink)
{
    TxPacket p = make_packet(10, 16, 5555u);
    DemodScratch shared = fresh_scratch(p);

    // 38 taps
    check_against_fresh("38 taps (8/30)", p, 8, 30, shared);
    BOOST_CHECK_EQUAL(shared.sync_cir_prepares, 1u);
    BOOST_CHECK_EQUAL(shared.sync_cir_code.fingerprint,
                      gr::uwb::demod::rxopt::cir_template_key(
                          gr::uwb::demod::GetPreambleCode(10),
                          gr::uwb::demod::kQm35CodeLength));

    // 116 taps: capacity must grow, and the CIR must match a fresh scratch.
    {
        DemodScratch fresh = fresh_scratch(p);
        const DemodResult want = run_packet(p, 8, 108, fresh);
        const DemodResult got = run_packet(p, 8, 108, shared);
        check_matches_fresh("116 taps (8/108)", got, want);
        BOOST_CHECK_EQUAL(want.cir.cir_values.size(), 116u);
    }
    // Same code, wider window => one capacity-driven preparation.
    BOOST_CHECK_EQUAL(shared.sync_cir_prepares, 2u);

    // Back to 38 taps: the reservation already covers it, so no new prepare,
    // and the result must still equal a fresh scratch.
    check_against_fresh("38 taps after 116", p, 8, 30, shared);
    BOOST_CHECK_EQUAL(shared.sync_cir_prepares, 2u);
}

// ---------------------------------------------------------------------------
// 4. Same total tap count, different pre/post split (16/100 -> 100/16).
//    The template is unchanged, so the only thing that may differ is the
//    geometry; both splits must equal their fresh-scratch references and the
//    second must not silently reuse the first split's buffers.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_sync_cir_cache_same_taps_different_split)
{
    TxPacket p = make_packet(11, 16, 7777u);
    DemodScratch shared = fresh_scratch(p);

    check_against_fresh("split 16/100", p, 16, 100, shared);
    const uint32_t prepares_after_first = shared.sync_cir_prepares;
    check_against_fresh("split 100/16", p, 100, 16, shared);

    // Both splits need 116 taps; the second one fits the existing reservation.
    BOOST_CHECK_EQUAL(shared.sync_cir_prepares, prepares_after_first);
    BOOST_CHECK_EQUAL(prepares_after_first, 1u);
}

// ---------------------------------------------------------------------------
// 5. Failure then recovery on the same scratch.
//    (a) zero repetitions: the stage fails before touching the kernel.
//    (b) a truncated buffer: the kernel is prepared for code 10 and then
//        fails with NoValidRepetitions.  The next valid packet uses code 9, so
//        a cache keyed on the wrong object would keep code 10's template.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_sync_cir_cache_failure_then_recovery)
{
    const size_t pre = 8;
    const size_t post = 30;
    TxPacket p9 = make_packet(9, 16, 3131u);
    TxPacket p10 = make_packet(10, 16, 3132u);

    DemodScratch shared = fresh_scratch(p9);

    // (a) zero repetitions on the shared scratch.
    {
        DemodScratch fresh = fresh_scratch(p9);
        const DemodResult want = run_packet(p9, pre, post, fresh);
        const DemodResult failed = run_packet(p9, pre, post, shared, /*reps=*/0);
        BOOST_CHECK_MESSAGE(failed.status != gr::uwb::demod::DemodStatus::Success,
                            "zero-repetition run must fail");
        const DemodResult got = run_packet(p9, pre, post, shared);
        check_matches_fresh("after zero-repetition failure", got, want);
    }

    // (b) kernel-level failure after preparation, then recovery with a
    //     DIFFERENT code at a NARROWER window.  The wide preparation reserves
    //     more capacity than the recovery needs, so a capacity-only staleness
    //     test cannot notice the code change -- only a real identity test can.
    {
        DemodScratch recover = fresh_scratch(p9);

        // Prepare code 10 with a 116-tap reservation.  The packet itself fails
        // at the soft-chip FIR (64-tap stack window), which is irrelevant here:
        // the preparation is what this case needs.
        const DemodResult wide = run_packet(p10, 8, 108, recover);
        BOOST_CHECK(wide.cir.cir_values.size() == 116u);
        BOOST_REQUIRE(recover.sync_cir_code.ready);
        BOOST_CHECK_EQUAL(recover.sync_cir_code.code_index, 10u);
        const uint32_t prepares_before = recover.sync_cir_prepares;

        // Same code, clipped buffer: every CIR window is rejected, so the
        // kernel fails closed AFTER the identity was already satisfied.
        const std::vector<int8_t> code10(
            gr::uwb::demod::GetPreambleCode(10),
            gr::uwb::demod::GetPreambleCode(10) + gr::uwb::demod::kQm35CodeLength);
        const size_t truncated = kT0 + 11 * kSyncSamples;
        CirResult failed_cir;
        const bool ok = stage_cir_softchips(
            p10.iq.data(), truncated, make_profile(10, pre, post), wide.timing,
            code10, failed_cir, recover);
        BOOST_CHECK_MESSAGE(!ok, "truncated CIR window must fail closed");
        BOOST_CHECK(!failed_cir.ok);
        BOOST_CHECK_EQUAL(recover.sync_cir_prepares, prepares_before);

        // Recovery: code 9 at the production 38-tap window.
        DemodScratch fresh9 = fresh_scratch(p9);
        const DemodResult want = run_packet(p9, pre, post, fresh9);
        BOOST_REQUIRE(want.status == gr::uwb::demod::DemodStatus::Success);
        const DemodResult got = run_packet(p9, pre, post, recover);
        check_matches_fresh("after kernel failure with another code", got, want);
        BOOST_CHECK_EQUAL(recover.sync_cir_prepares, prepares_before + 1);
    }
}

// ---------------------------------------------------------------------------
// 6. N identical packets on one scratch must prepare the kernel exactly once.
//    This is the guard against "fixing" the staleness test by rebuilding the
//    template on every packet.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_sync_cir_cache_prepares_once_per_identity)
{
    const size_t pre = 8;
    const size_t post = 30;
    TxPacket p = make_packet(12, 16, 6060u);
    DemodScratch shared = fresh_scratch(p);
    DemodScratch fresh = fresh_scratch(p);
    const DemodResult want = run_packet(p, pre, post, fresh);

    constexpr size_t kPackets = 8;
    for (size_t i = 0; i < kPackets; ++i) {
        const DemodResult got = run_packet(p, pre, post, shared);
        check_matches_fresh("identical packet " + std::to_string(i), got, want);
    }
    BOOST_CHECK_MESSAGE(shared.sync_cir_prepares == 1u,
                        "prepare_sync_code ran " << shared.sync_cir_prepares
                                                 << " times for "
                                                 << kPackets
                                                 << " identical packets");
    BOOST_CHECK(shared.sync_cir_code.ready);
    BOOST_CHECK_EQUAL(shared.sync_cir_code.code_index, 12u);
}
