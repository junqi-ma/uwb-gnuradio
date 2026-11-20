#include <gnuradio/uwb/uwb_demod_phr.h>
/* SPDX-License-Identifier: GPL-3.0-or-later */
/*
 * Independent QA for the experimental fused PHR CMF/PN-despread path.
 *
 * The PHR kernel is checked against a double-precision two-level reference
 * (per-chip CMF followed by PN despread), against the production
 * bprf_demod() decisions, against the payload wrapper's shared geometry, and
 * around every readable-window edge.  The SECDED cases use a tiny CL-3
 * convolutional encoder (poly2trellis(3, [2 5])) so the decoded codeword is
 * deterministic and the corrupted codeword is exactly representable as hard
 * decisions.
 *
 * Integration cases use the same self-contained HRP packet generator as
 * qa_uwb_rx_fused_cmf.cc.  They compare a legacy FastRxPipeline with one that
 * has only fused_phr_cmf enabled, and exercise the one-shot full-stream
 * fallback for a corrupted PHR/payload field.
 *
 * Case 11 (diagnostic FIR replay) is intentionally a skip: the replay helper
 * named diag_fir_phr_us is not declared at this integration point.  When the
 * helper lands, replace the pending case with a segment-chip-count check
 * (sum == cir.soft_chip_count, PHR == 21*512 when covered).
 */
#include <boost/test/unit_test.hpp>

#include <gnuradio/uwb/fullpacket_rx_pipeline.h>
#include <gnuradio/uwb/uwb_hrp_mod_core.h>
#include <gnuradio/uwb/uwb_rx_fused_cmf.h>

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdint>
#include <random>
#include <utility>
#include <vector>

namespace {

using namespace openuwb;
using namespace openuwb::demod;
using cf = std::complex<float>;
using cd = std::complex<double>;

constexpr size_t kTap = 38;

// ---------------------------------------------------------------------------
// Random input for the PHR kernel: 38-tap complex CIR, 21 symbols of PN at
// scrambler offset 0, a random unit phase rotation, and enough IQ for the last
// candidate (symbol 20, offset 320) plus the 164-point template.
// ---------------------------------------------------------------------------
struct PhrInput {
    std::vector<cf> rx;
    std::vector<cf> taps;
    std::vector<int8_t> pn;
    std::complex<float> rot;
    int64_t chip_start = 17;
    int64_t phr_start = 100;

    PhrInput(size_t code, uint32_t seed)
    {
        std::mt19937 rng(seed);
        std::uniform_real_distribution<float> dist(-1.0f, 1.0f);
        rx.resize(32768);
        for (auto& x : rx)
            x = cf(dist(rng), dist(rng));
        taps.resize(kTap);
        for (auto& h : taps)
            h = cf(dist(rng), dist(rng));
        BOOST_REQUIRE(demod::core::detail::bprf_spreading(
            pn, code, rxfusion::kPhrScramOffset,
            rxfusion::kPhrSymbols * rxfusion::kPhrChipsPerBurst));
        const float ang = dist(rng) * 3.0f;
        rot = std::complex<float>(std::cos(ang), std::sin(ang));
    }
};

// Double-precision reference:
//   M(s,p) = sum_c a[c] * sum_q h[q] * rx[chip_start +
//              (phr_start + s*512 + offset[p] + c)*2 - 38 + 1 + q]
cd reference_metric(const PhrInput& in, size_t s, size_t p)
{
    const int64_t offset =
        static_cast<int64_t>(rxfusion::kPhrCandidateChipOffset[p]);
    cd m(0.0, 0.0);
    for (size_t c = 0; c < rxfusion::kPhrChipsPerBurst; ++c) {
        const int8_t a =
            in.pn[s * rxfusion::kPhrChipsPerBurst + c];
        cd inner(0.0, 0.0);
        for (size_t q = 0; q < in.taps.size(); ++q) {
            const int64_t idx =
                in.chip_start +
                (in.phr_start +
                 static_cast<int64_t>(s) * rxfusion::kPhrChipsPerSymbol +
                 offset + static_cast<int64_t>(c)) *
                    static_cast<int64_t>(rxfusion::kSamplesPerChip) -
                static_cast<int64_t>(in.taps.size()) + 1 +
                static_cast<int64_t>(q);
            inner += cd(in.taps[q]) * cd(in.rx[idx]);
        }
        m += static_cast<double>(a) * inner;
    }
    return m;
}

// CL-3 convolutional encoder matching poly2trellis(3, [2 5]) and the trellis
// used by detail::vitdec_cl3(): out0 = state>>1, out1 = u^(state&1),
// next = (u<<1)|(state>>1).  The state starts at zero, as in vitdec_cl3().
void encode_cl3(const std::vector<int8_t>& u,
                std::vector<int8_t>& g0,
                std::vector<int8_t>& g1)
{
    g0.assign(u.size(), int8_t(0));
    g1.assign(u.size(), int8_t(0));
    int state = 0;
    for (size_t t = 0; t < u.size(); ++t) {
        const int s1 = state >> 1;
        const int s0 = state & 1;
        const int bit = u[t] & 1;
        g0[t] = static_cast<int8_t>(s1);
        g1[t] = static_cast<int8_t>(bit ^ s0);
        state = (bit << 1) | s1;
    }
}

// ---------------------------------------------------------------------------
// Self-contained HRP packet generator (same style as qa_uwb_rx_fused_cmf.cc).
// ---------------------------------------------------------------------------
struct Packet {
    std::vector<uint8_t> wire;
    std::vector<cf> iq, sync;
    static constexpr size_t seed = 2048;

    Packet(size_t bytes, size_t code)
    {
        wire.resize(bytes - 2);
        for (size_t j = 0; j < wire.size(); ++j)
            wire[j] = uint8_t(19 * j + code);
        mod::append_ieee_fcs(wire);
        mod::HrpModConfig cfg;
        cfg.code_index = code;
        cfg.sync_repetitions = 64;
        cfg.sfd_mode = "4z2";
        cfg.peak_amplitude = .8f;
        const size_t count = mod::packet_samples_998p4(
            64, 8, wire.size(), false, &cfg.pulse);
        mod::HrpModScratch scratch;
        scratch.reserve(127, count);
        std::vector<cf> wave(count);
        size_t written = 0;
        BOOST_REQUIRE(mod::modulate_one(wire.data(), wire.size(), cfg,
                                       scratch, wave.data(), wave.size(),
                                       written));
        BOOST_REQUIRE_EQUAL(written, count);
        sync.assign(wave.begin(), wave.begin() + 1016);
        iq.assign(seed + count + 2048, cf(0));
        std::copy(wave.begin(), wave.end(), iq.begin() + seed);
    }
};

} // namespace

// ---------------------------------------------------------------------------
// 1. Kernel vs the double-precision two-level sum.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(phr_fused_kernel_matches_double_double_sum)
{
    double worst = 0.0;
    for (size_t code : { 9u, 10u, 11u, 12u }) {
        PhrInput in(code, 7000 + static_cast<uint32_t>(code));
        for (int8_t pol : { int8_t(1), int8_t(-1) }) {
            rxfusion::Metrics out;
            rxfusion::Scratch scratch;
            BOOST_REQUIRE(rxfusion::correlate_phr(
                in.rx.data(), in.rx.size(), in.taps.data(), in.taps.size(),
                in.chip_start, rxfusion::kSamplesPerChip, in.phr_start,
                in.pn.data(), pol, in.rot, out, scratch));
            BOOST_REQUIRE_EQUAL(out.symbols, rxfusion::kPhrSymbols);

            const cd r(in.rot.real(), in.rot.imag());
            double max_abs = 0.0, max_ref = 0.0;
            for (size_t s = 0; s < rxfusion::kPhrSymbols; ++s) {
                for (size_t p = 0; p < rxfusion::kCandidateCount; ++p) {
                    const cd ref = reference_metric(in, s, p);
                    const auto& got_c = out.complex_value[
                        rxfusion::kCandidateCount * s + p];
                    const cd got(got_c.real(), got_c.imag());
                    const double complex_err = std::abs(got - ref);
                    const double signed_ref =
                        static_cast<double>(pol) * (r * ref).real();
                    const double signed_err = std::abs(
                        out.signed_value[rxfusion::kCandidateCount * s + p] -
                        static_cast<float>(signed_ref));
                    max_abs = std::max(
                        max_abs, std::max(complex_err, signed_err));
                    max_ref = std::max(max_ref, std::abs(ref));
                }
            }
            const double rel = max_abs / std::max(max_ref, 1e-12);
            worst = std::max(worst, rel);
            BOOST_TEST_MESSAGE("PHR kernel code=" << code
                               << " pol=" << int(pol)
                               << " max_abs=" << max_abs
                               << " max_rel=" << rel);
            BOOST_CHECK_LT(rel, 2e-5);
        }
    }
    BOOST_TEST_MESSAGE("PHR kernel worst relative scale error=" << worst);
    BOOST_CHECK_LT(worst, 2e-5);
}

// ---------------------------------------------------------------------------
// 2. Kernel metric decision equivalence with production bprf_demod().
//
// With h = delta at tap 0 and rx[2k] = soft[k], rx[2k+1] = 0, the fused
// template reproduces exactly the four real metrics that bprf_demod()
// computes from the same soft chips.  chip_start is chosen so that
// base0 = 2*phr_start.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(phr_fused_decision_matches_bprf_demod)
{
    const int64_t phr_start = 5;
    // With a one-tap CIR: base0 = chip_start + 2*phr_start.  chip_start = 0
    // makes the first template window start at rx[2*phr_start], so the kernel
    // reads exactly the even samples that carry the soft chips.
    const int64_t chip_start = 0;
    const size_t soft_len =
        static_cast<size_t>(phr_start) +
        rxfusion::kPhrSymbols * rxfusion::kPhrChipsPerSymbol;
    std::vector<float> soft(soft_len);
    std::mt19937 rng(991);
    std::uniform_real_distribution<float> dist(-1.0f, 1.0f);
    for (auto& x : soft)
        x = dist(rng);

    std::vector<cf> rx(2 * soft_len + 2, cf(0.0f, 0.0f));
    for (size_t k = 0; k < soft_len; ++k)
        rx[2 * k] = cf(soft[k], 0.0f);

    std::vector<cf> h(1, cf(1.0f, 0.0f));

    for (size_t code : { 9u, 10u, 11u, 12u }) {
        std::vector<int8_t> pn;
        BOOST_REQUIRE(demod::core::detail::bprf_spreading(
            pn, code, rxfusion::kPhrScramOffset,
            rxfusion::kPhrSymbols * rxfusion::kPhrChipsPerBurst));
        for (int8_t pol : { int8_t(1), int8_t(-1) }) {
            std::vector<int8_t> g0_ref, g1_ref;
            BOOST_REQUIRE(demod::core::detail::bprf_demod(
                soft, phr_start, code, rxfusion::kPhrSymbols,
                rxfusion::kPhrChipsPerBurst,
                rxfusion::kPhrChipsPerSymbol, rxfusion::kPhrScramOffset, pol,
                g0_ref, g1_ref));

            rxfusion::Metrics metrics;
            rxfusion::Scratch scratch;
            BOOST_REQUIRE(rxfusion::correlate_phr(
                rx.data(), rx.size(), h.data(), h.size(), chip_start,
                rxfusion::kSamplesPerChip, phr_start, pn.data(), pol,
                cf(1.0f, 0.0f), metrics, scratch));

            std::vector<int8_t> g0, g1;
            rxfusion::decide_bpm(metrics, g0, g1);
            BOOST_CHECK(g0 == g0_ref);
            BOOST_CHECK(g1 == g1_ref);

            // Report the metric scale difference on the same soft chips.
            float max_diff = 0.0f, max_ref = 0.0f;
            for (size_t s = 0; s < rxfusion::kPhrSymbols; ++s) {
                for (size_t p = 0; p < rxfusion::kCandidateCount; ++p) {
                    const int64_t base =
                        phr_start +
                        static_cast<int64_t>(s) *
                            rxfusion::kPhrChipsPerSymbol +
                        static_cast<int64_t>(
                            rxfusion::kPhrCandidateChipOffset[p]);
                    float ref = 0.0f;
                    for (size_t c = 0;
                         c < rxfusion::kPhrChipsPerBurst; ++c) {
                        ref += static_cast<float>(
                                   pn[s * rxfusion::kPhrChipsPerBurst + c]) *
                               soft[static_cast<size_t>(base +
                                                        static_cast<int64_t>(
                                                            c))];
                    }
                    ref = (pol < 0) ? -ref : ref;
                    const float got =
                        metrics.signed_value[
                            rxfusion::kCandidateCount * s + p];
                    max_diff = std::max(max_diff, std::abs(got - ref));
                    max_ref = std::max(max_ref, std::abs(ref));
                }
            }
            BOOST_TEST_MESSAGE("PHR decision code=" << code
                               << " pol=" << int(pol)
                               << " max_metric_diff=" << max_diff
                               << " max_metric_ref=" << max_ref);
        }
    }
}

// ---------------------------------------------------------------------------
// 3. Tie rules and near-zero values.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(phr_fused_decision_ties_and_near_zero)
{
    rxfusion::Metrics m;
    m.reset(6);
    auto set = [&](size_t s, float m0, float m1, float m2, float m3) {
        m.signed_value[4 * s + 0] = m0;
        m.signed_value[4 * s + 1] = m1;
        m.signed_value[4 * s + 2] = m2;
        m.signed_value[4 * s + 3] = m3;
    };
    // 0: all equal -> half0, hop0, non-negative.
    set(0, 2.0f, 2.0f, 2.0f, 2.0f);
    // 1: hop tie inside half0 (candidates 0 and 2) -> hop0 wins.
    set(1, 3.0f, 0.1f, 3.0f, 0.1f);
    // 2: hop tie inside half1 (candidates 1 and 3), half1 larger -> hop0
    //    within half1 (candidate 1) wins.
    set(2, 0.1f, 4.0f, 0.1f, 4.0f);
    // 3: half tie, half0 is negative -> half0 wins and g1=1.
    set(3, -2.0f, -2.0f, 1.0f, 1.0f);
    // 4: exact zeros -> half0, non-negative.
    set(4, 0.0f, 0.0f, 0.0f, 0.0f);
    // 5: tiny values; half1 has the larger magnitude and is negative.
    set(5, 1e-20f, -2e-20f, 1e-20f, 2e-20f);

    std::vector<int8_t> g0, g1;
    rxfusion::decide_bpm(m, g0, g1);
    BOOST_REQUIRE_EQUAL(g0.size(), size_t(6));
    BOOST_REQUIRE_EQUAL(g1.size(), size_t(6));

    BOOST_CHECK_EQUAL(int(g0[0]), 0);
    BOOST_CHECK_EQUAL(int(g1[0]), 0);
    BOOST_CHECK_EQUAL(int(g0[1]), 0);
    BOOST_CHECK_EQUAL(int(g1[1]), 0);
    BOOST_CHECK_EQUAL(int(g0[2]), 1);
    BOOST_CHECK_EQUAL(int(g1[2]), 0);
    BOOST_CHECK_EQUAL(int(g0[3]), 0);
    BOOST_CHECK_EQUAL(int(g1[3]), 1);
    BOOST_CHECK_EQUAL(int(g0[4]), 0);
    BOOST_CHECK_EQUAL(int(g1[4]), 0);
    BOOST_CHECK_EQUAL(int(g0[5]), 1);
    BOOST_CHECK_EQUAL(int(g1[5]), 1);
}

// ---------------------------------------------------------------------------
// 4. Unaligned base pointer must produce identical metrics.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(phr_fused_unaligned_address_matches)
{
    PhrInput in(10, 8123);
    rxfusion::Metrics aligned, shifted;
    rxfusion::Scratch scratch;
    BOOST_REQUIRE(rxfusion::correlate_phr(
        in.rx.data(), in.rx.size(), in.taps.data(), in.taps.size(),
        in.chip_start, rxfusion::kSamplesPerChip, in.phr_start, in.pn.data(),
        1, in.rot, aligned, scratch));

    std::vector<cf> copy(in.rx.size() + 1);
    copy[0] = cf(0.0f, 0.0f);
    std::copy(in.rx.begin(), in.rx.end(), copy.begin() + 1);
    BOOST_REQUIRE(rxfusion::correlate_phr(
        copy.data() + 1, in.rx.size(), in.taps.data(), in.taps.size(),
        in.chip_start, rxfusion::kSamplesPerChip, in.phr_start, in.pn.data(),
        1, in.rot, shifted, scratch));

    BOOST_CHECK(aligned.complex_value == shifted.complex_value);
    BOOST_CHECK(aligned.signed_value == shifted.signed_value);

    // The generic entry point must agree with the PHR wrapper too.
    rxfusion::Metrics generic;
    BOOST_REQUIRE(rxfusion::correlate_field(
        in.rx.data(), in.rx.size(), in.taps.data(), in.taps.size(),
        in.chip_start, rxfusion::kSamplesPerChip, in.phr_start, in.pn.data(),
        rxfusion::kPhrSymbols, rxfusion::kPhrGeometry, 1, in.rot, generic,
        scratch));
    BOOST_CHECK(aligned.complex_value == generic.complex_value);
    BOOST_CHECK(aligned.signed_value == generic.signed_value);
}

// ---------------------------------------------------------------------------
// 5. Exact readable tail boundary for PHR (last candidate offset 320).
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(phr_fused_tail_boundary_exact)
{
    PhrInput in(10, 9311);
    const size_t len = rxfusion::template_length(
        in.taps.size(), rxfusion::kSamplesPerChip,
        rxfusion::kPhrChipsPerBurst);
    BOOST_CHECK_EQUAL(len, size_t(164));

    const int64_t base0 =
        in.chip_start +
        in.phr_start * static_cast<int64_t>(rxfusion::kSamplesPerChip) -
        static_cast<int64_t>(in.taps.size()) + 1;
    const int64_t last_start =
        static_cast<int64_t>(rxfusion::kPhrSymbols - 1) *
            static_cast<int64_t>(rxfusion::kPhrChipsPerSymbol) +
        static_cast<int64_t>(
            rxfusion::kPhrCandidateChipOffset[
                rxfusion::kCandidateCount - 1]);
    const int64_t needed =
        base0 + last_start * static_cast<int64_t>(rxfusion::kSamplesPerChip) +
        static_cast<int64_t>(len) - 1;
    BOOST_REQUIRE_GT(needed, 0);
    BOOST_REQUIRE_LT(needed + 1, static_cast<int64_t>(in.rx.size()));

    rxfusion::Metrics out;
    rxfusion::Scratch scratch;
    BOOST_CHECK(rxfusion::correlate_phr(
        in.rx.data(), static_cast<size_t>(needed + 1), in.taps.data(),
        in.taps.size(), in.chip_start, rxfusion::kSamplesPerChip,
        in.phr_start, in.pn.data(), 1, in.rot, out, scratch));
    BOOST_CHECK(!rxfusion::correlate_phr(
        in.rx.data(), static_cast<size_t>(needed), in.taps.data(),
        in.taps.size(), in.chip_start, rxfusion::kSamplesPerChip,
        in.phr_start, in.pn.data(), 1, in.rot, out, scratch));
}

// ---------------------------------------------------------------------------
// 6. Invalid geometry, pointers and PN contract.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(phr_fused_rejects_invalid_geometry)
{
    PhrInput in(10, 10417);
    rxfusion::Metrics out;
    rxfusion::Scratch scratch;
    const cf* rx = in.rx.data();
    const size_t n = in.rx.size();
    const cf* h = in.taps.data();
    const int8_t* pn = in.pn.data();
    const size_t tap = in.taps.size();

    // Null pointers.
    BOOST_CHECK(!rxfusion::correlate_phr(
        nullptr, n, h, tap, in.chip_start, 2, in.phr_start, pn, 1, in.rot,
        out, scratch));
    BOOST_CHECK(!rxfusion::correlate_phr(
        rx, n, nullptr, tap, in.chip_start, 2, in.phr_start, pn, 1, in.rot,
        out, scratch));
    BOOST_CHECK(!rxfusion::correlate_phr(
        rx, n, h, tap, in.chip_start, 2, in.phr_start, nullptr, 1, in.rot,
        out, scratch));

    // Unsupported tap count / grid / polarity / indices.
    BOOST_CHECK(!rxfusion::correlate_phr(
        rx, n, h, 0, in.chip_start, 2, in.phr_start, pn, 1, in.rot, out,
        scratch));
    BOOST_CHECK(!rxfusion::correlate_phr(
        rx, n, h, rxfusion::kMaxTapCount + 1, in.chip_start, 2, in.phr_start,
        pn, 1, in.rot, out, scratch));
    BOOST_CHECK(!rxfusion::correlate_phr(
        rx, n, h, tap, in.chip_start, 1, in.phr_start, pn, 1, in.rot, out,
        scratch));
    BOOST_CHECK(!rxfusion::correlate_phr(
        rx, n, h, tap, -1, 2, in.phr_start, pn, 1, in.rot, out, scratch));
    BOOST_CHECK(!rxfusion::correlate_phr(
        rx, n, h, tap, in.chip_start, 2, -1, pn, 1, in.rot, out, scratch));
    BOOST_CHECK(!rxfusion::correlate_phr(
        rx, n, h, tap, in.chip_start, 2, in.phr_start, pn, 0, in.rot, out,
        scratch));
    BOOST_CHECK(!rxfusion::correlate_phr(
        rx, 0, h, tap, in.chip_start, 2, in.phr_start, pn, 1, in.rot, out,
        scratch));

    // PN must be strictly +/-1.
    std::vector<int8_t> bad_pn = in.pn;
    bad_pn[7] = 0;
    BOOST_CHECK(!rxfusion::correlate_phr(
        rx, n, h, tap, in.chip_start, 2, in.phr_start, bad_pn.data(), 1,
        in.rot, out, scratch));

    // Symbols bound is checked by the generic entry point.
    BOOST_CHECK(!rxfusion::correlate_field(
        rx, n, h, tap, in.chip_start, 2, in.phr_start, pn, 0,
        rxfusion::kPhrGeometry, 1, in.rot, out, scratch));
    BOOST_CHECK(!rxfusion::correlate_field(
        rx, n, h, tap, in.chip_start, 2, in.phr_start, pn,
        (static_cast<size_t>(1) << 20) + 1, rxfusion::kPhrGeometry, 1,
        in.rot, out, scratch));
    BOOST_CHECK(!rxfusion::correlate_field(
        rx, n, h, tap, in.chip_start, 2, -1, pn, rxfusion::kPhrSymbols,
        rxfusion::kPhrGeometry, 1, in.rot, out, scratch));

    // Candidate offsets / counts / zero geometry lengths.
    const size_t bad_offsets[rxfusion::kCandidateCount] = { 0, 256, 64, 512 };
    const rxfusion::Geometry bad_offset{
        rxfusion::kPhrChipsPerBurst, rxfusion::kPhrChipsPerSymbol,
        rxfusion::kCandidateCount, bad_offsets
    };
    BOOST_CHECK(!rxfusion::correlate_field(
        rx, n, h, tap, in.chip_start, 2, in.phr_start, pn,
        rxfusion::kPhrSymbols, bad_offset, 1, in.rot, out, scratch));

    const rxfusion::Geometry null_offsets{
        rxfusion::kPhrChipsPerBurst, rxfusion::kPhrChipsPerSymbol,
        rxfusion::kCandidateCount, nullptr
    };
    BOOST_CHECK(!rxfusion::correlate_field(
        rx, n, h, tap, in.chip_start, 2, in.phr_start, pn,
        rxfusion::kPhrSymbols, null_offsets, 1, in.rot, out, scratch));

    const rxfusion::Geometry bad_count{
        rxfusion::kPhrChipsPerBurst, rxfusion::kPhrChipsPerSymbol,
        3, rxfusion::kPhrCandidateChipOffset
    };
    BOOST_CHECK(!rxfusion::correlate_field(
        rx, n, h, tap, in.chip_start, 2, in.phr_start, pn,
        rxfusion::kPhrSymbols, bad_count, 1, in.rot, out, scratch));

    const rxfusion::Geometry zero_burst{
        0, rxfusion::kPhrChipsPerSymbol, rxfusion::kCandidateCount,
        rxfusion::kPhrCandidateChipOffset
    };
    BOOST_CHECK(!rxfusion::correlate_field(
        rx, n, h, tap, in.chip_start, 2, in.phr_start, pn,
        rxfusion::kPhrSymbols, zero_burst, 1, in.rot, out, scratch));

    const rxfusion::Geometry zero_symbol{
        rxfusion::kPhrChipsPerBurst, 0, rxfusion::kCandidateCount,
        rxfusion::kPhrCandidateChipOffset
    };
    BOOST_CHECK(!rxfusion::correlate_field(
        rx, n, h, tap, in.chip_start, 2, in.phr_start, pn,
        rxfusion::kPhrSymbols, zero_symbol, 1, in.rot, out, scratch));

    const rxfusion::Geometry huge_symbol{
        rxfusion::kPhrChipsPerBurst,
        (static_cast<size_t>(1) << 30) + 1, rxfusion::kCandidateCount,
        rxfusion::kPhrCandidateChipOffset
    };
    BOOST_CHECK(!rxfusion::correlate_field(
        rx, n, h, tap, in.chip_start, 2, in.phr_start, pn,
        rxfusion::kPhrSymbols, huge_symbol, 1, in.rot, out, scratch));

    // Geometry constants and template lengths.
    BOOST_CHECK_EQUAL(rxfusion::kPhrGeometry.chips_per_burst, size_t(64));
    BOOST_CHECK_EQUAL(rxfusion::kPhrGeometry.chips_per_symbol, size_t(512));
    BOOST_CHECK_EQUAL(
        rxfusion::template_length(38, rxfusion::kSamplesPerChip,
                                  rxfusion::kPhrChipsPerBurst),
        size_t(164));
    BOOST_CHECK_EQUAL(
        rxfusion::template_length(38, rxfusion::kSamplesPerChip,
                                  rxfusion::kChipsPerBurst),
        size_t(52));
}

// ---------------------------------------------------------------------------
// 7. Shared PHR decision decoder: valid codeword, one-bit SECDED correction,
//    two-bit uncorrectable detection.  soft_chips is empty and
//    sfd_end_chip = -1, so the corrected-length tail-energy guard is skipped.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(phr_fused_decode_phr_decisions_secded)
{
    // data rate 6.81 Mb/s, PSDU length 32, not ranging, 64 SYNC.
    const std::vector<int8_t> sys13 = {
        1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1
    };
    std::vector<int8_t> phr19;
    demod::core::detail::hrp_secded(sys13, phr19);
    BOOST_REQUIRE_EQUAL(phr19.size(), size_t(19));

    std::vector<int8_t> u(phr19.begin(), phr19.end());
    u.push_back(int8_t(0));
    u.push_back(int8_t(0));
    BOOST_REQUIRE_EQUAL(u.size(), size_t(21));

    NsSfdResult ns;
    ns.ok = true;
    ns.sfd_start_chip = -1;
    ns.sfd_end_chip = -1;
    ns.polarity = 1;
    const std::vector<float> empty_soft;

    auto decode = [&](const std::vector<int8_t>& bits, PhrResult& out,
                      std::vector<int8_t>& g0, std::vector<int8_t>& g1) {
        encode_cl3(bits, g0, g1);
        BOOST_REQUIRE_EQUAL(g0.size(), size_t(21));
        BOOST_REQUIRE_EQUAL(g1.size(), size_t(21));
        return demod::core::decode_phr_decisions(
            g0, g1, empty_soft, ns, out);
    };

    // Valid codeword: exact bits, no correction.
    {
        PhrResult out;
        std::vector<int8_t> g0, g1;
        BOOST_REQUIRE(decode(u, out, g0, g1));
        BOOST_REQUIRE(out.ok);
        BOOST_CHECK_EQUAL(out.psdu_length, uint32_t(32));
        BOOST_CHECK_EQUAL(out.data_rate_mbps, 6.81f);
        BOOST_CHECK(!out.secded_corrected);
        BOOST_CHECK(!out.secded_uncorrectable);
        BOOST_REQUIRE_EQUAL(out.phr_bits.size(), size_t(19));
        for (size_t i = 0; i < 19; ++i)
            BOOST_CHECK_EQUAL(out.phr_bits[i],
                              static_cast<uint8_t>(phr19[i]));
    }

    // One codeword bit flipped: bit 11 (preamble-duration field) is one of the
    // data columns that the production hrpSECDED address table can correct,
    // and it does not change the PSDU length, so the tail-energy guard cannot
    // replace the correction.  The shared decoder deliberately preserves the
    // legacy stage_phr convention: the 13 systematic bits are corrected, while
    // the 6-bit parity tail is the parity regenerated from the uncorrected
    // systematic word before the flip (production has always done this).
    {
        std::vector<int8_t> u1 = u;
        u1[11] ^= 1;
        PhrResult out;
        std::vector<int8_t> g0, g1;
        BOOST_REQUIRE(decode(u1, out, g0, g1));
        BOOST_REQUIRE(out.ok);
        BOOST_CHECK(out.secded_corrected);
        BOOST_CHECK(!out.secded_uncorrectable);
        BOOST_CHECK_EQUAL(out.psdu_length, uint32_t(32));
        BOOST_REQUIRE_EQUAL(out.phr_bits.size(), size_t(19));
        // The 13 systematic bits carry all PHR fields and must be restored.
        for (size_t i = 0; i < 13; ++i)
            BOOST_CHECK_EQUAL(out.phr_bits[i],
                              static_cast<uint8_t>(sys13[i]));
        // Preserve the production parity-tail convention exactly: the helper
        // regenerates parity from the corrupted systematic word, then flips
        // the addressed data bit without recomputing the tail.
        std::vector<int8_t> sys_corrupt = sys13;
        sys_corrupt[11] ^= 1;
        std::vector<int8_t> legacy19;
        demod::core::detail::hrp_secded(sys_corrupt, legacy19);
        for (size_t i = 13; i < 19; ++i)
            BOOST_CHECK_EQUAL(out.phr_bits[i],
                              static_cast<uint8_t>(legacy19[i]));
    }

    // Two codeword bits flipped (length bit 6 and ranging bit 9): their parity
    // columns have even combined weight, so the decoder must report an
    // uncorrectable error.
    {
        std::vector<int8_t> u2 = u;
        u2[6] ^= 1;
        u2[9] ^= 1;
        PhrResult out;
        std::vector<int8_t> g0, g1;
        BOOST_REQUIRE(decode(u2, out, g0, g1));
        BOOST_REQUIRE(out.ok);
        BOOST_CHECK(out.secded_uncorrectable);
        BOOST_CHECK_EQUAL(out.phr_bits.size(), size_t(19));
    }
}

// ---------------------------------------------------------------------------
// 8. Fused PHR + fused payload pipeline decodes the same PSDU and shortens
//    the prefix relative to the payload-fusion-only baseline.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(phr_fused_pipeline_decodes_same_psdu)
{
    for (const auto spec : { std::pair<size_t, size_t>{ 32, 10 },
                             std::pair<size_t, size_t>{ 127, 9 } }) {
        Packet packet(spec.first, spec.second);
        experiment::FastRxPipeline legacy(packet.iq.size(), spec.second);
        experiment::FastRxPipeline::Options payload_only(
            false, false, false, false, true);
        experiment::FastRxPipeline baseline(packet.iq.size(), spec.second,
                                            payload_only);
        experiment::FastRxPipeline::Options phr_only(
            false, false, false, false, false, true);
        experiment::FastRxPipeline fast(packet.iq.size(), spec.second,
                                        phr_only);

        const auto a = legacy.run(packet.iq.data(), packet.iq.size(), 31,
                                  Packet::seed, packet.sync);
        const auto baseline_res = baseline.run(
            packet.iq.data(), packet.iq.size(), 32, Packet::seed,
            packet.sync);
        const auto b = fast.run(packet.iq.data(), packet.iq.size(), 33,
                                Packet::seed, packet.sync);

        BOOST_REQUIRE_EQUAL(int(a.status), int(demod::DemodStatus::Success));
        BOOST_REQUIRE_EQUAL(int(b.status), int(demod::DemodStatus::Success));
        BOOST_REQUIRE_EQUAL(int(baseline_res.status),
                            int(demod::DemodStatus::Success));
        BOOST_CHECK(a.payload.fcs_pass);
        BOOST_CHECK(b.payload.fcs_pass);
        BOOST_CHECK(baseline_res.payload.fcs_pass);
        BOOST_CHECK(a.payload.bytes == b.payload.bytes);
        BOOST_CHECK(packet.wire == b.payload.bytes);
        BOOST_CHECK_EQUAL(a.phr.psdu_length, b.phr.psdu_length);
        BOOST_CHECK(a.phr.phr_bits == b.phr.phr_bits);
        BOOST_CHECK_EQUAL(a.phr.secded_corrected, b.phr.secded_corrected);
        BOOST_CHECK_EQUAL(a.phr.secded_uncorrectable,
                          b.phr.secded_uncorrectable);
        BOOST_CHECK_EQUAL(a.phr.data_rate_mbps, b.phr.data_rate_mbps);

        BOOST_CHECK_EQUAL(b.stage_phr_fused_attempts, 1u);
        BOOST_CHECK_EQUAL(b.stage_phr_fused_fallbacks, 0u);
        BOOST_CHECK_EQUAL(b.stage_phr_fused_symbols,
                          static_cast<uint32_t>(rxfusion::kPhrSymbols));
        BOOST_CHECK_EQUAL(b.stage_fused_attempts, 1u);
        BOOST_CHECK_EQUAL(b.stage_fused_fallbacks, 0u);
        BOOST_CHECK_GT(b.stage_phr_fused_decode_ns, 0u);
        BOOST_CHECK_GT(b.stage_fused_decode_ns, 0u);

        // PHR fusion shrinks the on-demand prefix; the payload fusion path
        // must not extend it.
        const size_t budget =
            rxopt::fused_phr_prefix_budget(Qm35825Profile::Default());
        BOOST_TEST_MESSAGE("PHR prefix: legacy=" << a.cir.soft_chip_count
                           << " payload_only=" << baseline_res.cir.soft_chip_count
                           << " fused_phr=" << b.cir.soft_chip_count
                           << " budget=" << budget);
        BOOST_CHECK_LT(b.cir.soft_chip_count, baseline_res.cir.soft_chip_count);
        BOOST_CHECK_LE(b.cir.soft_chip_count, budget);
        // The PHR soft-chip range starts at sfd_end_chip + 1.  NS-SFD still
        // searches +8 chips past the nominal boundary, so the necessary
        // prefix retains at most that 8-chip halo plus one budget-mapping
        // entry.  It must not contain a full PHR symbol (512 chips).
        const size_t phr_start =
            static_cast<size_t>(b.ns_sfd.sfd_end_chip + 1);
        const size_t retained_phr_halo =
            b.cir.soft_chip_count > phr_start
                ? b.cir.soft_chip_count - phr_start
                : 0;
        BOOST_TEST_MESSAGE("PHR retained soft-chip halo=" << retained_phr_halo
                           << " chips");
        BOOST_CHECK_LE(retained_phr_halo, size_t(9));
        BOOST_CHECK_LT(b.cir.soft_chip_count,
                       phr_start + rxfusion::kPhrChipsPerSymbol);
        BOOST_CHECK_EQUAL(b.cir.extended_chip_count, 0u);
        BOOST_CHECK_EQUAL(b.cir.on_demand_extended, 0u);
        BOOST_CHECK_EQUAL(b.cir.on_demand_fallbacks, 0u);
        BOOST_CHECK_EQUAL(b.stage_softchip_fallbacks, 0u);
        BOOST_CHECK_EQUAL(b.cir.soft_fir_first_chip, 32504u);
        BOOST_CHECK_EQUAL(b.cir.soft_fir_computed_chips, 4081u);
        BOOST_CHECK_EQUAL(a.ns_sfd.sfd_start_chip, b.ns_sfd.sfd_start_chip);
        BOOST_CHECK_EQUAL(a.ns_sfd.polarity, b.ns_sfd.polarity);
    }
}

// ---------------------------------------------------------------------------
// 9a. Corrupted payload after an intact PHR: PHR fusion succeeds, payload
//     fusion fails, and exactly one full-stream retry publishes the legacy
//     verdict.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(phr_fused_payload_corruption_falls_back_once)
{
    Packet packet(127, 9);
    const size_t payload_start =
        Packet::seed + (64 + 8) * 1016 + 21 * 1024;
    const size_t corrupt_lo =
        std::min(packet.iq.size(), payload_start + 100);
    // 8192 samples (~10.7 RS 6-bit symbols) exceeds RS(63,55)'s 8-erasure bound.
    const size_t corrupt_hi = std::min(packet.iq.size(), corrupt_lo + 8192);
    for (size_t i = corrupt_lo; i < corrupt_hi; ++i)
        packet.iq[i] = cf(0.0f, 0.0f);

    experiment::FastRxPipeline legacy(packet.iq.size(), 9);
    experiment::FastRxPipeline::Options options(
        false, false, false, false, false, true);
    experiment::FastRxPipeline fast(packet.iq.size(), 9, options);
    const auto a = legacy.run(packet.iq.data(), packet.iq.size(), 41,
                              Packet::seed, packet.sync);
    const auto b = fast.run(packet.iq.data(), packet.iq.size(), 42,
                            Packet::seed, packet.sync);

    BOOST_TEST_MESSAGE("corrupt payload: phr_ok=" << b.phr.ok
                       << " phr_fused_attempts="
                       << b.stage_phr_fused_attempts
                       << " payload_fused_attempts="
                       << b.stage_fused_attempts
                       << " payload_fused_fallbacks="
                       << b.stage_fused_fallbacks
                       << " softchip_fallbacks="
                       << b.stage_softchip_fallbacks
                       << " status=" << int(b.status));
    BOOST_CHECK_EQUAL(int(a.status), int(b.status));
    BOOST_CHECK(b.status != demod::DemodStatus::Success);
    BOOST_REQUIRE(b.phr.ok); // corruption starts after the PHR field
    BOOST_CHECK(!b.payload.fcs_pass);

    BOOST_CHECK_EQUAL(b.stage_phr_fused_attempts, 1u);
    BOOST_CHECK_EQUAL(b.stage_phr_fused_fallbacks, 0u);
    BOOST_CHECK_EQUAL(b.stage_fused_attempts, 1u);
    BOOST_CHECK_EQUAL(b.stage_fused_fallbacks, 1u);
    BOOST_CHECK_EQUAL(b.stage_softchip_fallbacks, 1u);

    // The published result must describe the final legacy retry.
    BOOST_CHECK_EQUAL(b.cir.soft_chip_count, a.cir.soft_chip_count);
    BOOST_CHECK_EQUAL(b.ns_sfd.sfd_start_chip, a.ns_sfd.sfd_start_chip);
}

// ---------------------------------------------------------------------------
// 9b. Corrupted PHR field: the fused attempt cannot be trusted, and the
//     published CIR/NS-SFD/PHR must belong to the one full-stream retry.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(phr_fused_phr_corruption_publishes_legacy_retry)
{
    Packet packet(127, 9);
    const size_t phr_lo = Packet::seed + (64 + 8) * 1016;
    const size_t phr_hi = std::min(packet.iq.size(), phr_lo + 21 * 1024);
    for (size_t i = phr_lo; i < phr_hi; ++i)
        packet.iq[i] = cf(0.0f, 0.0f);

    experiment::FastRxPipeline legacy(packet.iq.size(), 9);
    experiment::FastRxPipeline::Options options(
        false, false, false, false, false, true);
    experiment::FastRxPipeline fast(packet.iq.size(), 9, options);
    const auto a = legacy.run(packet.iq.data(), packet.iq.size(), 43,
                              Packet::seed, packet.sync);
    const auto b = fast.run(packet.iq.data(), packet.iq.size(), 44,
                            Packet::seed, packet.sync);

    BOOST_TEST_MESSAGE("corrupt PHR: status legacy=" << int(a.status)
                       << " fused=" << int(b.status)
                       << " phr_fused_attempts="
                       << b.stage_phr_fused_attempts
                       << " softchip_fallbacks="
                       << b.stage_softchip_fallbacks);
    BOOST_CHECK_EQUAL(int(a.status), int(b.status));
    BOOST_CHECK_EQUAL(b.stage_phr_fused_attempts, 1u);
    BOOST_CHECK_EQUAL(b.stage_softchip_fallbacks, 1u);

    // Final published state must be the full-stream retry, not the discarded
    // short prefix or the fused attempt.
    BOOST_CHECK_EQUAL(b.cir.soft_chip_count, a.cir.soft_chip_count);
    BOOST_CHECK_EQUAL(b.ns_sfd.sfd_start_chip, a.ns_sfd.sfd_start_chip);
    BOOST_CHECK_EQUAL(b.ns_sfd.sfd_end_chip, a.ns_sfd.sfd_end_chip);
    BOOST_CHECK_EQUAL(b.phr.psdu_length, a.phr.psdu_length);
    BOOST_CHECK(b.phr.phr_bits == a.phr.phr_bits);
    BOOST_CHECK_EQUAL(b.phr.secded_corrected, a.phr.secded_corrected);
    BOOST_CHECK_EQUAL(b.phr.secded_uncorrectable,
                      a.phr.secded_uncorrectable);
}

// ---------------------------------------------------------------------------
// 9c. RAKE / bypass keep the legacy stream: no PHR-fused attempt, one
//     full-stream retry, and the packet still decodes.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(phr_fused_skips_rake_and_bypass)
{
    Packet packet(32, 10);
    for (auto mode : { demod::CirSoftChipMode::Rake,
                       demod::CirSoftChipMode::Bypass }) {
        auto profile = demod::Qm35825Profile::Default();
        profile.code_index = 10;
        profile.timing_search_margin = 256;
        profile.timing_coarse_stride = 1;
        profile.rx_softchip_on_demand = true;
        profile.rx_fused_payload_cmf = true;
        profile.rx_fused_phr_cmf = true;
        profile.cir_soft_chip_mode = mode;
        profile.cir_rake_top_k =
            (mode == demod::CirSoftChipMode::Rake) ? 4 : 0;
        demod::core::DemodScratch scratch;
        scratch.reserve(packet.iq.size());
        const auto r = demod::core::demodulate_one(
            packet.iq.data(), packet.iq.size(), profile, 61, Packet::seed, 0,
            packet.sync, scratch);
        BOOST_REQUIRE_EQUAL(int(r.status), int(demod::DemodStatus::Success));
        BOOST_CHECK(r.payload.fcs_pass);
        BOOST_CHECK_EQUAL(r.stage_phr_fused_attempts, 0u);
        BOOST_CHECK_EQUAL(r.stage_phr_fused_fallbacks, 0u);
        BOOST_CHECK_EQUAL(r.stage_softchip_fallbacks, 1u);
    }
}

// ---------------------------------------------------------------------------
// 10. Option off: neither the default pipeline nor payload-fusion-only may
//     run the PHR-fused kernel.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(phr_fused_option_off_keeps_legacy)
{
    Packet packet(32, 10);
    {
        experiment::FastRxPipeline legacy(packet.iq.size(), 10);
        const auto r = legacy.run(packet.iq.data(), packet.iq.size(), 51,
                                  Packet::seed, packet.sync);
        BOOST_REQUIRE_EQUAL(int(r.status), int(demod::DemodStatus::Success));
        BOOST_CHECK_EQUAL(r.stage_phr_fused_attempts, 0u);
    }
    {
        experiment::FastRxPipeline::Options payload_only(
            false, false, false, false, true);
        experiment::FastRxPipeline payload(packet.iq.size(), 10, payload_only);
        const auto r = payload.run(packet.iq.data(), packet.iq.size(), 52,
                                   Packet::seed, packet.sync);
        BOOST_REQUIRE_EQUAL(int(r.status), int(demod::DemodStatus::Success));
        BOOST_CHECK_EQUAL(r.stage_phr_fused_attempts, 0u);
        BOOST_CHECK_EQUAL(r.stage_fused_attempts, 1u);
    }

    // The configuration layer must make fused_phr imply both dependencies.
    const auto effective = experiment::FastRxPipeline::Options(
                               false, false, false, false, false, true)
                               .normalized();
    BOOST_CHECK(effective.fused_phr_cmf);
    BOOST_CHECK(effective.fused_payload_cmf);
    BOOST_CHECK(effective.softchip_on_demand);
    BOOST_CHECK(!effective.fast_sfd);
    BOOST_CHECK(!effective.fast_cfo_rotation);
    BOOST_CHECK(!effective.seed_refine_tail);
}

// ---------------------------------------------------------------------------
// 11. Diagnostic FIR replay: the payload-fusion prefix is replayed at the
//     decoded boundaries.  Verify the ranges partition the produced stream and
//     that the PHR segment is the full 21*512 chips.  This run is diagnostic
//     only and is never used for formal timing.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(phr_fused_diag_replay_partitions_prefix)
{
    Packet packet(32, 10);
    experiment::FastRxPipeline::Options options(
        false, false, false, false, true, false, true);
    experiment::FastRxPipeline payload(packet.iq.size(), 10, options);
    const auto r = payload.run(packet.iq.data(), packet.iq.size(), 71,
                               Packet::seed, packet.sync);
    BOOST_REQUIRE_EQUAL(int(r.status), int(demod::DemodStatus::Success));
    BOOST_REQUIRE_EQUAL(r.diag_fir_valid, 1u);
    const uint64_t sum = uint64_t(r.diag_fir_preamble_chips) +
                         r.diag_fir_sfd_chips + r.diag_fir_phr_chips +
                         r.diag_fir_payload_chips + r.diag_fir_extra_chips;
    BOOST_CHECK_EQUAL(sum, uint64_t(r.cir.soft_chip_count));
    BOOST_CHECK_EQUAL(r.diag_fir_phr_chips,
                      uint32_t(rxfusion::kPhrSymbols *
                               rxfusion::kPhrChipsPerSymbol));
    BOOST_CHECK_GT(r.diag_fir_phr_us, 0u);
    BOOST_TEST_MESSAGE("FIR diag: preamble=" << r.diag_fir_preamble_us
                       << "us/" << r.diag_fir_preamble_chips
                       << " sfd=" << r.diag_fir_sfd_us << "us/"
                       << r.diag_fir_sfd_chips
                       << " phr=" << r.diag_fir_phr_us << "us/"
                       << r.diag_fir_phr_chips
                       << " payload=" << r.diag_fir_payload_us << "us/"
                       << r.diag_fir_payload_chips
                       << " extra=" << r.diag_fir_extra_us << "us/"
                       << r.diag_fir_extra_chips);
}

// Independent double reference for the reordered SYNC phase statistic.
BOOST_AUTO_TEST_CASE(sync_phase_without_chip_fir_matches_reference)
{
    std::mt19937 rng(8204);
    std::uniform_real_distribution<float> d(-1.0f, 1.0f);
    std::vector<cf> iq(34000), taps(38), sum;
    std::vector<int8_t> spread(508);
    for (auto& v : iq) v = cf(d(rng), d(rng));
    for (auto& v : taps) v = cf(d(rng), d(rng));
    for (size_t i = 0; i < spread.size(); ++i)
        spread[i] = i % 16 == 0 ? (i % 32 == 0 ? 1 : -1) : 0;
    for (int64_t base : {int64_t(-8), int64_t(17)}) {
        cf gain;
        BOOST_REQUIRE(rxopt::sync_phase_gain(iq.data(), iq.size(), base,
            taps.data(), taps.size(), spread, 32, sum, gain));
        std::complex<double> ref{};
        double scale = 0;
        for (size_t r = 0; r < 32; ++r)
            for (size_t c = 0; c < spread.size(); ++c) {
                std::complex<double> v{};
                for (size_t q = 0; q < taps.size(); ++q) {
                    const int64_t k = base + r * 1016 + c * 2 + q;
                    if (k >= 0)
                        v += std::complex<double>(taps[q]) *
                             std::complex<double>(iq[k]);
                }
                ref += double(spread[c]) * v;
                scale += std::abs(double(spread[c]) * v);
            }
        BOOST_CHECK_LT(std::abs(std::complex<double>(gain) - ref) /
                           std::max(std::abs(ref), 1.0), 2e-5);
        BOOST_CHECK_GT(scale, 0.0);
    }
    cf gain;
    const size_t exact = 17 + 31 * 1016 + 2 * 507 + 38;
    BOOST_CHECK(rxopt::sync_phase_gain(iq.data(), exact, 17,
        taps.data(), taps.size(), spread, 32, sum, gain));
    BOOST_CHECK(!rxopt::sync_phase_gain(iq.data(), exact-1, 17,
        taps.data(), taps.size(), spread, 32, sum, gain));
}
