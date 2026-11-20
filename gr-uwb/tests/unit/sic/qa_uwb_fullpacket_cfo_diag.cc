/* SPDX-License-Identifier: GPL-3.0-or-later */
#include <boost/test/unit_test.hpp>
#include "../../benchmarks/fullpacket_cfo_diagnostics.h"

#include <array>
#include <cmath>
#include <cstring>
#include <limits>

namespace {
namespace diag = gr::uwb::experiment::cfo_diag;
using diag::cf;
using diag::cd;
constexpr double pi = 3.14159265358979323846;
constexpr int64_t block_size = 1024;

std::vector<cf> scalar_reference(const diag::Input& in)
{
    // Compute block selection independently for EVERY sample, so this catches
    // wrong edge partitioning or an incorrectly truncated negative division.
    std::array<cf, block_size> base;
    const double omega = 2 * pi * in.cfo_hz / in.sample_rate;
    const cd step = std::exp(cd(0, omega));
    cd value = 1;
    for (auto& sample : base) {
        sample = cf(value);
        value *= step;
    }
    std::vector<cf> expected(in.n);
    for (size_t j = 0; j < in.n; ++j) {
        const int64_t p = in.absolute_lo + int64_t(j) - in.phase_reference;
        int64_t b = p / block_size;
        if (p % block_size < 0)
            --b;
        const size_t index = size_t(p - b * block_size);
        const cf phase(std::exp(cd(0, omega * double(b * block_size))));
        expected[j] = cf(in.accum[j]) * (base[index] * phase);
    }
    return expected;
}

bool same(const std::vector<cf>& actual, const std::vector<cf>& expected)
{
    return actual.size() >= expected.size() &&
           std::memcmp(actual.data(), expected.data(),
                       expected.size() * sizeof(cf)) == 0;
}

struct Fixture {
    std::vector<cd> accum;
    std::vector<cf> iq;
    diag::Input input;
    diag::Scratch scratch;

    explicit Fixture(size_t n = 4103)
        : accum(n), iq(n)
    {
        for (size_t j = 0; j < n; ++j) {
            accum[j] = cd(double(int(j % 37) - 18) * .0031,
                          double(int(j % 23) - 11) * .0053);
            iq[j] = cf(float(int(j % 31) - 15) * .0017f,
                       float(int(j % 19) - 9) * .0029f);
        }
        input.accum = accum.data();
        input.iq = iq.data();
        input.n = n;
        input.absolute_lo = 2043;
        input.phase_reference = 2048;
        scratch.reserve(n);
    }

    bool run(diag::Variant variant)
    {
        diag::Timings times;
        const bool result = diag::run(input, variant, scratch, times);
        BOOST_CHECK(std::isfinite(times.total_us));
        BOOST_CHECK_GE(times.total_us, 0.);
        return result;
    }
};
} // namespace

BOOST_AUTO_TEST_CASE(cfo_diagnostic_variants_match_absolute_grid_reference)
{
    Fixture f;
    for (double cfo : {0., -50000., 50000.}) {
        f.input.cfo_hz = cfo;
        const auto expected = scalar_reference(f.input);
        for (auto variant : {diag::Variant::FusedChecked,
                             diag::Variant::FusedModelChecked,
                             diag::Variant::FusedUnchecked,
                             diag::Variant::SplitChecked}) {
            BOOST_REQUIRE(f.run(variant));
            BOOST_CHECK_MESSAGE(same(f.scratch.model, expected),
                                "CFO output differs from original arithmetic");
        }
    }
}

BOOST_AUTO_TEST_CASE(cfo_diagnostic_checked_paths_reject_nonfinite_outputs)
{
    Fixture f(8);
    const auto checked = {diag::Variant::FusedChecked,
                          diag::Variant::FusedModelChecked,
                          diag::Variant::SplitChecked};
    // Finite double can overflow when narrowed to float; explicit Inf/NaN
    // probes exercise rejection after conversion and complex multiplication.
    for (double bad : {std::numeric_limits<double>::max(),
                       std::numeric_limits<double>::infinity(),
                       std::numeric_limits<double>::quiet_NaN()}) {
        f.accum[1] = cd(bad, 0);
        for (auto variant : checked)
            BOOST_CHECK(!f.run(variant));
    }

    const double max_float = std::numeric_limits<float>::max();
    std::fill(f.accum.begin(), f.accum.end(), cd(0));
    f.input.absolute_lo = 2496; // 45 degrees at 50kHz and 998.4MS/s
    f.input.phase_reference = 0;
    f.input.cfo_hz = 50000;
    f.accum[0] = cd(.9 * max_float, -.9 * max_float);
    for (auto variant : checked)
        BOOST_CHECK(!f.run(variant));

    // Finite model and finite IQ can still overflow during trial subtraction.
    // The model-only variant deliberately omits this gate for offline cost
    // isolation; it is not a production cancellation path.
    std::fill(f.accum.begin(), f.accum.end(), cd(0));
    std::fill(f.iq.begin(), f.iq.end(), cf(0));
    f.input.absolute_lo = 0;
    f.input.cfo_hz = 0;
    f.accum[0] = cd(.75 * max_float, 0);
    f.iq[0] = cf(-std::numeric_limits<float>::max(), 0);
    BOOST_CHECK(!f.run(diag::Variant::FusedChecked));
    BOOST_CHECK(!f.run(diag::Variant::SplitChecked));
    BOOST_CHECK(f.run(diag::Variant::FusedModelChecked));
    BOOST_CHECK(f.run(diag::Variant::FusedUnchecked));
}

BOOST_AUTO_TEST_CASE(cfo_diagnostic_rejects_invalid_parameters_and_capacity)
{
    Fixture f(16);
    auto invalid = f.input;
    auto rejects = [&](const diag::Input& in) {
        diag::Timings times;
        BOOST_CHECK(!diag::run(in, diag::Variant::SplitChecked,
                              f.scratch, times));
    };
    invalid.accum = nullptr; rejects(invalid);
    invalid = f.input; invalid.iq = nullptr; rejects(invalid);
    invalid = f.input; invalid.n = f.accum.size() + 1; rejects(invalid);
    invalid = f.input; invalid.absolute_lo = -1; rejects(invalid);
    invalid = f.input; invalid.absolute_lo = std::numeric_limits<int64_t>::max();
    rejects(invalid);
    for (double rate : {0., -1., std::numeric_limits<double>::quiet_NaN()}) {
        invalid = f.input; invalid.sample_rate = rate; rejects(invalid);
    }
    invalid = f.input;
    invalid.cfo_hz = std::numeric_limits<double>::infinity(); rejects(invalid);
    diag::Timings times;
    BOOST_CHECK(!diag::run(f.input, static_cast<diag::Variant>(99),
                          f.scratch, times));
    BOOST_REQUIRE(f.run(diag::Variant::FusedChecked));
}
