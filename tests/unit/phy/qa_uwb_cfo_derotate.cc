/*
 * QA for the experimental communication-RX CFO derotation kernel
 * (gr-uwb/include/gnuradio/uwb/uwb_rx_cfo_opt.h).
 *
 * Contract under test
 * -------------------
 * `derotate_range_block` computes
 *
 *     dst[i] = src[i] * exp(-j*w*i)      for absolute i in [lo, hi)
 *
 * with phase origin strictly at sample 0, using one fused pass per 1024-sample
 * block: the block anchor exp(-j*w*b0) is folded into the table entry in
 * registers instead of being applied to the data in a second pass.
 *
 * The acceptance items are:
 *   1. error contract against the frozen two-pass implementation (584c1bb):
 *      max |dst_new - dst_old| <= 1e-6 and relative L2 <= 1e-7 on 500k samples
 *      for typical and extreme CFOs, including w = 0 and +-200 kHz;
 *   2. absolute accuracy against an independent double-precision reference;
 *   3. phase origin: the same sample must get the same phase whether it is
 *      reached with lo = 0 or with lo > 0 (the anchor is per block, not per
 *      call, so this is a real regression risk);
 *   4. hi truncation, lo >= hi, n = 0 and null buffers are no-ops;
 *   5. w = 0 is a bit-exact identity;
 *   6. block boundaries: lengths that are exact multiples of 1024, 1024 +- 1,
 *      and lengths shorter than one block (scalar tail);
 *   7. the untouched legacy recurrence kernel still agrees with the reference,
 *      so the rx_fast_cfo_rotation = false path cannot silently rot.
 *
 * Nothing here is a performance test: the micro-benchmark numbers live in the
 * development report, not in CTest.
 */
#define BOOST_TEST_MODULE qa_uwb_cfo_derotate
#include <boost/test/unit_test.hpp>

#include <gnuradio/uwb/uwb_phy_profile.h>
#include <gnuradio/uwb/uwb_rx_cfo_opt.h>

#include <volk/volk.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <cstddef>
#include <random>
#include <vector>

namespace {

using gr_complex = std::complex<float>;
using gr::uwb::demod::rxopt::kCfoDerotateBlock;

constexpr double kFs = 998400000.0; // QM35825 work-grid sample rate

// Error-contract thresholds (task requirement).
constexpr double kMaxAbsDiff = 1e-6;
constexpr double kMaxRelL2 = 1e-7;
// Absolute accuracy bound used by the pre-existing demod-core QA for the same
// kernel family (test_demod_core_rxopt_cfo_block_rotation_boundaries).
constexpr double kMaxAbsVsDoubleRef = 2e-5;

double cfo_to_w(double cfo_hz)
{
    return 2.0 * M_PI * cfo_hz / kFs;
}

// Deterministic, non-trivially-structured test signal (unit-ish amplitude so
// the absolute error bound is directly comparable to the float32 epsilon).
std::vector<gr_complex> make_signal(size_t n, unsigned seed = 12345u)
{
    std::vector<gr_complex> v(n);
    std::mt19937 rng(seed);
    std::normal_distribution<float> g(0.0f, 0.3f);
    for (size_t i = 0; i < n; ++i) {
        const float re = g(rng) + 0.2f * std::sin(0.013f * static_cast<float>(i));
        const float im = g(rng) + 0.2f * std::cos(0.021f * static_cast<float>(i));
        v[i] = gr_complex(re, im);
    }
    return v;
}

// ---------------------------------------------------------------------------
// Frozen reference: the two-pass implementation that shipped up to 584c1bb.
// Copied verbatim (modulo the vector wrapper) so the error contract is measured
// against the real predecessor and not against a re-derived formula.
// ---------------------------------------------------------------------------
void derotate_range_two_pass_legacy(gr_complex* dst,
                                    const gr_complex* src,
                                    size_t n,
                                    size_t lo,
                                    size_t hi,
                                    double w)
{
    using namespace gr::uwb::demod::rxopt;
    if (n == 0 || src == nullptr || dst == nullptr)
        return;
    hi = std::min(hi, n);
    if (lo >= hi)
        return;

    std::array<gr_complex, kCfoDerotateBlock> table;
    detail::build_cfo_table(table, w);

    const size_t b_start = (lo / kCfoDerotateBlock) * kCfoDerotateBlock;
    for (size_t b0 = b_start; b0 < hi; b0 += kCfoDerotateBlock) {
        const size_t k_start = (b0 < lo) ? (lo - b0) : 0;
        const size_t k_end =
            std::min(hi - b0, std::min(kCfoDerotateBlock, n - b0));
        if (k_start >= k_end)
            continue;
        const size_t len = k_end - k_start;

        gr_complex* dst_ptr = dst + b0 + k_start;
        const gr_complex* src_ptr = src + b0 + k_start;
        const gr_complex* table_ptr = table.data() + k_start;

        volk_32fc_x2_multiply_32fc(reinterpret_cast<lv_32fc_t*>(dst_ptr),
                                   reinterpret_cast<const lv_32fc_t*>(src_ptr),
                                   reinterpret_cast<const lv_32fc_t*>(table_ptr),
                                   static_cast<unsigned>(len));

        if (b0 != 0) {
            const double phase = -w * static_cast<double>(b0);
            const float ar = static_cast<float>(std::cos(phase));
            const float ai = static_cast<float>(std::sin(phase));
            volk_32fc_s32fc_multiply_32fc(
                reinterpret_cast<lv_32fc_t*>(dst_ptr),
                reinterpret_cast<const lv_32fc_t*>(dst_ptr),
                lv_cmake(ar, ai),
                static_cast<unsigned>(len));
        }
    }
}

struct DiffStats {
    double max_abs = 0.0;
    double rel_l2 = 0.0;
};

// max |a - b| and ||a - b|| / ||b|| over [lo, hi).
DiffStats diff_stats(const std::vector<gr_complex>& a,
                     const std::vector<gr_complex>& b,
                     size_t lo,
                     size_t hi)
{
    DiffStats st;
    double num = 0.0;
    double den = 0.0;
    for (size_t i = lo; i < hi; ++i) {
        const double dr = double(a[i].real()) - double(b[i].real());
        const double di = double(a[i].imag()) - double(b[i].imag());
        st.max_abs = std::max(st.max_abs, std::sqrt(dr * dr + di * di));
        num += dr * dr + di * di;
        den += double(b[i].real()) * b[i].real() +
               double(b[i].imag()) * b[i].imag();
    }
    st.rel_l2 = (den > 0.0) ? std::sqrt(num / den) : 0.0;
    return st;
}

} // namespace

// ---------------------------------------------------------------------------
// 1. Error contract: fused (new) vs two-pass (frozen legacy) on 500k samples.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_cfo_derotate_error_contract_vs_legacy_two_pass)
{
    const size_t n = 500000;
    const std::vector<gr_complex> src = make_signal(n);
    std::vector<gr_complex> got(n);
    std::vector<gr_complex> old(n);

    // 0 Hz (identity), the small offsets seen on a trimmed X410 clock, the
    // +-20 kHz representative and a deliberately extreme 200 kHz.
    const double cfos[] = { 0.0, 500.0, 2000.0, 5000.0, 20000.0, -20000.0,
                            200000.0, -200000.0 };
    for (double cfo : cfos) {
        const double w = cfo_to_w(cfo);
        std::fill(got.begin(), got.end(), gr_complex(0.0f, 0.0f));
        std::fill(old.begin(), old.end(), gr_complex(0.0f, 0.0f));
        gr::uwb::demod::rxopt::derotate_range_block(got.data(), src.data(), n, 0,
                                                    n, w);
        derotate_range_two_pass_legacy(old.data(), src.data(), n, 0, n, w);
        const DiffStats st = diff_stats(got, old, 0, n);
        BOOST_TEST_MESSAGE("cfo_hz=" << cfo << " max_abs=" << st.max_abs
                                     << " rel_l2=" << st.rel_l2);
        BOOST_CHECK_LE(st.max_abs, kMaxAbsDiff);
        BOOST_CHECK_LE(st.rel_l2, kMaxRelL2);
    }
}

// ---------------------------------------------------------------------------
// 2. Absolute accuracy against the independent double-precision reference.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_cfo_derotate_absolute_accuracy_vs_double_reference)
{
    const size_t n = 200000;
    const std::vector<gr_complex> src = make_signal(n, 777u);
    std::vector<gr_complex> got(n);
    std::vector<gr_complex> ref(n);
    std::vector<gr_complex> old(n);

    for (double cfo : { 0.0, 2000.0, 20000.0, -200000.0 }) {
        const double w = cfo_to_w(cfo);
        gr::uwb::demod::rxopt::derotate_range_block(got.data(), src.data(), n, 0,
                                                    n, w);
        derotate_range_two_pass_legacy(old.data(), src.data(), n, 0, n, w);
        gr::uwb::demod::rxopt::derotate_range_reference(ref.data(), src.data(),
                                                        n, 0, n, w);
        const DiffStats st_new = diff_stats(got, ref, 0, n);
        const DiffStats st_old = diff_stats(old, ref, 0, n);
        BOOST_TEST_MESSAGE("cfo_hz=" << cfo << " new_vs_double=" << st_new.max_abs
                                     << " legacy_vs_double=" << st_old.max_abs);
        // Same absolute bound the pre-existing demod-core QA uses for this
        // kernel family; the fused form must not be worse than the two-pass
        // form it replaces (with a small allowance for rounding luck).
        BOOST_CHECK_LE(st_new.max_abs, kMaxAbsVsDoubleRef);
        BOOST_CHECK_LE(st_new.max_abs, st_old.max_abs + 1e-7);
    }
}

// ---------------------------------------------------------------------------
// 3. Phase origin: lo = 0 and lo > 0 must give the same phase for the same
//    absolute sample index (the anchor is per block, not per call).
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_cfo_derotate_phase_origin_independent_of_lo)
{
    const size_t n = 20000;
    const std::vector<gr_complex> src = make_signal(n, 4242u);
    std::vector<gr_complex> full(n);
    std::vector<gr_complex> part(n, gr_complex(-999.0f, -999.0f));

    const double w = cfo_to_w(20000.0);
    gr::uwb::demod::rxopt::derotate_range_block(full.data(), src.data(), n, 0, n,
                                                w);

    // lo inside the first block, on a block boundary, and mid-block in a later
    // block; hi also non-aligned.
    struct Interval {
        size_t lo;
        size_t hi;
    };
    const Interval intervals[] = { { 137, 9000 },   { 1024, 12000 },
                                   { 1025, 1026 },  { 5120, 15361 },
                                   { 0, 1024 } };
    for (const auto& iv : intervals) {
        std::fill(part.begin(), part.end(), gr_complex(-999.0f, -999.0f));
        gr::uwb::demod::rxopt::derotate_range_block(part.data(), src.data(), n,
                                                    iv.lo, iv.hi, w);
        // Outside the requested range nothing may be written.
        for (size_t i = 0; i < iv.lo; ++i)
            BOOST_CHECK_EQUAL(part[i], gr_complex(-999.0f, -999.0f));
        for (size_t i = iv.hi; i < n; ++i)
            BOOST_CHECK_EQUAL(part[i], gr_complex(-999.0f, -999.0f));
        // Inside, the value must match the lo = 0 call sample by sample.  The
        // fused kernel evaluates the same anchor for the same block, so this is
        // bit-exact, not tolerance-based.
        size_t mismatches = 0;
        double max_abs = 0.0;
        for (size_t i = iv.lo; i < iv.hi; ++i) {
            if (part[i] != full[i])
                ++mismatches;
            max_abs = std::max(max_abs,
                               static_cast<double>(std::abs(part[i] - full[i])));
        }
        BOOST_TEST_MESSAGE("lo=" << iv.lo << " hi=" << iv.hi
                                 << " bit_mismatches=" << mismatches
                                 << " max_abs=" << max_abs);
        BOOST_CHECK_EQUAL(mismatches, size_t(0));
    }

    // And the phase origin is really sample 0: sample lo must not be src[lo].
    std::vector<gr_complex> lo_only(n, gr_complex(-999.0f, -999.0f));
    gr::uwb::demod::rxopt::derotate_range_block(lo_only.data(), src.data(), n,
                                                4096, 4097, w);
    BOOST_CHECK_GT(std::abs(lo_only[4096] - src[4096]), 1e-3f);
    const double phase = -w * 4096.0;
    const gr_complex expected =
        src[4096] * gr_complex(static_cast<float>(std::cos(phase)),
                               static_cast<float>(std::sin(phase)));
    BOOST_CHECK_LT(std::abs(lo_only[4096] - expected), 1e-5f);
}

// ---------------------------------------------------------------------------
// 4. hi truncation, empty range, degenerate inputs.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_cfo_derotate_range_guards)
{
    const size_t n = 4096;
    const std::vector<gr_complex> src = make_signal(n, 99u);
    const gr_complex sentinel(-999.0f, -999.0f);
    const double w = cfo_to_w(-5000.0);

    // hi > n must clamp to n and still fill [lo, n).
    {
        std::vector<gr_complex> d(n, sentinel);
        std::vector<gr_complex> ref(n, sentinel);
        gr::uwb::demod::rxopt::derotate_range_block(d.data(), src.data(), n,
                                                    1024, n + 5000, w);
        gr::uwb::demod::rxopt::derotate_range_reference(ref.data(), src.data(),
                                                        n, 1024, n, w);
        for (size_t i = 0; i < 1024; ++i)
            BOOST_CHECK_EQUAL(d[i], sentinel);
        const DiffStats st = diff_stats(d, ref, 1024, n);
        BOOST_CHECK_LE(st.max_abs, kMaxAbsVsDoubleRef);
    }
    // lo == hi and lo > hi are no-ops.
    for (auto iv : { std::pair<size_t, size_t>{ 2000, 2000 },
                     std::pair<size_t, size_t>{ 3000, 2000 } }) {
        std::vector<gr_complex> d(n, sentinel);
        gr::uwb::demod::rxopt::derotate_range_block(d.data(), src.data(), n,
                                                    iv.first, iv.second, w);
        for (size_t i = 0; i < n; ++i)
            BOOST_CHECK_EQUAL(d[i], sentinel);
    }
    // n = 0 and null pointers are no-ops (must not crash).
    {
        std::vector<gr_complex> d(4, sentinel);
        gr::uwb::demod::rxopt::derotate_range_block(d.data(), src.data(), 0, 0,
                                                    0, w);
        gr::uwb::demod::rxopt::derotate_range_block(nullptr, src.data(), n, 0, 8,
                                                    w);
        gr::uwb::demod::rxopt::derotate_range_block(d.data(), nullptr, n, 0, 8,
                                                    w);
        for (size_t i = 0; i < d.size(); ++i)
            BOOST_CHECK_EQUAL(d[i], sentinel);
    }
}

// ---------------------------------------------------------------------------
// 5. w = 0 is a bit-exact identity.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_cfo_derotate_zero_cfo_is_identity)
{
    const size_t n = 5000;
    const std::vector<gr_complex> src = make_signal(n, 5u);
    std::vector<gr_complex> d(n, gr_complex(-999.0f, -999.0f));
    gr::uwb::demod::rxopt::derotate_range_block(d.data(), src.data(), n, 0, n,
                                                0.0);
    for (size_t i = 0; i < n; ++i)
        BOOST_CHECK_EQUAL(d[i], src[i]);
}

// ---------------------------------------------------------------------------
// 6. Block boundaries: exact multiples of 1024, 1024 +- 1, sub-block lengths.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_cfo_derotate_block_boundaries)
{
    const size_t max_n = 5 * kCfoDerotateBlock + 3;
    const std::vector<gr_complex> src = make_signal(max_n, 31337u);
    const double w = cfo_to_w(20000.0);

    const size_t lengths[] = { 1,
                               3,
                               4,
                               5,
                               kCfoDerotateBlock - 1,
                               kCfoDerotateBlock,
                               kCfoDerotateBlock + 1,
                               2 * kCfoDerotateBlock,
                               2 * kCfoDerotateBlock + 1,
                               4 * kCfoDerotateBlock,
                               max_n };
    for (size_t n : lengths) {
        std::vector<gr_complex> got(n);
        std::vector<gr_complex> old(n);
        std::vector<gr_complex> ref(n);
        gr::uwb::demod::rxopt::derotate_range_block(got.data(), src.data(), n, 0,
                                                    n, w);
        derotate_range_two_pass_legacy(old.data(), src.data(), n, 0, n, w);
        gr::uwb::demod::rxopt::derotate_range_reference(ref.data(), src.data(),
                                                        n, 0, n, w);
        const DiffStats st_old = diff_stats(got, old, 0, n);
        const DiffStats st_ref = diff_stats(got, ref, 0, n);
        BOOST_TEST_MESSAGE("n=" << n << " vs_legacy_max_abs=" << st_old.max_abs
                                << " rel_l2=" << st_old.rel_l2
                                << " vs_double_max_abs=" << st_ref.max_abs);
        BOOST_CHECK_LE(st_old.max_abs, kMaxAbsDiff);
        BOOST_CHECK_LE(st_old.rel_l2, kMaxRelL2);
        BOOST_CHECK_LE(st_ref.max_abs, kMaxAbsVsDoubleRef);
    }
}

// ---------------------------------------------------------------------------
// 7. The legacy recurrence kernel (rx_fast_cfo_rotation = false) is untouched
//    and must still track the same reference.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_cfo_derotate_legacy_recurrence_unchanged)
{
    const size_t n = 8192;
    const std::vector<gr_complex> src = make_signal(n, 2024u);
    for (double cfo : { 0.0, 20000.0, -50000.0 }) {
        const double w = cfo_to_w(cfo);
        std::vector<gr_complex> got(n);
        std::vector<gr_complex> ref(n);
        gr::uwb::demod::rxopt::derotate_range_recurrence(got.data(), src.data(),
                                                         n, 0, n, w);
        gr::uwb::demod::rxopt::derotate_range_reference(ref.data(), src.data(),
                                                        n, 0, n, w);
        const DiffStats st = diff_stats(got, ref, 0, n);
        BOOST_CHECK_LE(st.max_abs, kMaxAbsVsDoubleRef);
    }
}
