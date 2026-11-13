/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * QA for the discrete equivalent CIR estimator (active peak groups).
 *
 * The QA never feeds sidecar truth into the online estimator.  Truth is used
 * only to synthesize y or to assert on the frozen model afterwards.
 */
#include <boost/test/unit_test.hpp>

#include <gnuradio/uwb/uwb_discrete_cir_sic.h>
#include <gnuradio/uwb/uwb_hrp_mod_core.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <random>
#include <string>
#include <utility>
#include <vector>

namespace {

using namespace gr::uwb;
using cf = std::complex<float>;
using cd = std::complex<double>;

constexpr double kPi = 3.14159265358979323846;
constexpr double kFs = 998.4e6;
constexpr size_t kSps = 1016;
constexpr size_t kReps = 8;
constexpr size_t kTrainLen = kReps * kSps;

std::string reason_of(const sic::DiscreteCirModel& m)
{
    return m.reason ? std::string(m.reason) : std::string();
}

// Public shaped packet.  The first `kReps` SYNC symbols are the finite train
// (start transient + inter-symbol tail), not a tiled single symbol.
struct Packet {
    mod::HrpModConfig profile;
    mod::HrpModScratch mod_scratch;
    std::vector<cf> wave;
    std::vector<cf> train;
    std::vector<cf> sync;

    explicit Packet(size_t code = 10, size_t psdu_len = 32, size_t reps = 64)
    {
        profile.code_index = code;
        profile.sync_repetitions = reps;
        profile.sfd_mode = "4z2";
        profile.peak_amplitude = 0.8f;
        std::vector<uint8_t> psdu = mod::make_random_psdu(
            psdu_len - 2, static_cast<uint32_t>(1000 + code), true);
        const size_t cap = mod::packet_samples_998p4(
                               reps, 8, psdu.size(), false, &profile.pulse) +
                           512;
        mod_scratch.reserve(127, cap);
        wave.resize(cap);
        size_t plen = 0;
        BOOST_REQUIRE(mod::modulate_one(psdu.data(), psdu.size(), profile,
                                        mod_scratch, wave.data(), cap, plen));
        wave.resize(plen);
        BOOST_REQUIRE(wave.size() >= kTrainLen);
        train.assign(wave.begin(), wave.begin() + kTrainLen);
        sync.assign(wave.begin(), wave.begin() + kSps);
    }
};

sic::DiscreteCirTemplates templates_of(const Packet& p)
{
    sic::DiscreteCirTemplates t;
    t.sync = p.sync.data();
    t.sync_len = p.sync.size();
    t.train = p.train.data();
    t.train_len = p.train.size();
    return t;
}

sic::DiscreteCirConfig make_cfg(size_t n, size_t q, size_t tap_count = 40)
{
    sic::DiscreteCirConfig c;
    c.comm_code = 10;
    c.sync_reps = 64;
    c.samples_per_symbol = kSps;
    c.sample_rate = kFs;
    c.fit_reps = kReps;
    c.roi_lo = static_cast<int64_t>(q) - 64;
    c.roi_hi = static_cast<int64_t>(q) + 64;
    c.lag_lo = -8;
    c.tap_count = tap_count;
    c.peak_floor = 0.15;
    c.peak_min_spacing = 5;
    c.active_radius = 1;
    c.lambda = 1e-6;
    c.coarse_stride = 1;
    c.detect_threshold = 0.40;
    c.fit_residual_max = 0.8;
    c.gain_condition_max = 1e8;
    c.max_window_samples = n;
    c.max_packet_samples = 0;
    return c;
}

std::vector<cf> make_integer_trace(
    size_t n,
    size_t q,
    const std::vector<cf>& train,
    const std::vector<std::pair<int64_t, cf>>& paths,
    double cfo_hz = 0.0)
{
    std::vector<cf> y(n, cf(0.f, 0.f));
    for (size_t m = 0; m < kTrainLen; ++m) {
        cd acc(0.0, 0.0);
        for (const auto& p : paths) {
            const int64_t xi = static_cast<int64_t>(m) - p.first;
            if (xi >= 0 && xi < static_cast<int64_t>(train.size()))
                acc += cd(p.second) *
                       cd(train[static_cast<size_t>(xi)]);
        }
        const cd rot = std::polar(
            1.0, 2.0 * kPi * cfo_hz * static_cast<double>(m) / kFs);
        y[q + m] = cf(acc * rot);
    }
    return y;
}

// Independent cubic (Catmull-Rom) fractional-delay operator.  It is NOT the
// frozen preamble fractional path; it is a local 4-tap interpolation used only
// to synthesize the QA receive waveform.
void add_fractional_path(std::vector<cf>& y,
                         size_t q,
                         const std::vector<cf>& train,
                         double tau,
                         cf gain,
                         double cfo_hz)
{
    const double base = std::floor(tau);
    const double t = tau - base;
    const double w[4] = {
        -0.5 * t * t * t + t * t - 0.5 * t,
        1.5 * t * t * t - 2.5 * t * t + 1.0,
        -1.5 * t * t * t + 2.0 * t * t + 0.5 * t,
        0.5 * t * t * t - 0.5 * t * t
    };
    for (size_t m = 0; m < kTrainLen; ++m) {
        cd acc(0.0, 0.0);
        for (int k = -1; k <= 2; ++k) {
            const int64_t xi =
                static_cast<int64_t>(m) -
                (static_cast<int64_t>(base) + static_cast<int64_t>(k));
            if (xi >= 0 && xi < static_cast<int64_t>(train.size()))
                acc += w[k + 1] * cd(train[static_cast<size_t>(xi)]);
        }
        const cd rot = std::polar(
            1.0, 2.0 * kPi * cfo_hz * static_cast<double>(m) / kFs);
        y[q + m] += cf(acc * rot * cd(gain));
    }
}

const cf* dense_tap_at(const sic::DiscreteCirModel& m, int64_t lag)
{
    if (lag < m.lag_lo ||
        lag >= m.lag_lo + static_cast<int64_t>(m.tap_count))
        return nullptr;
    return &m.taps[static_cast<size_t>(lag - m.lag_lo)];
}

bool active_has_lag(const sic::DiscreteCirModel& m, int64_t lag)
{
    for (size_t i = 0; i < m.active_count; ++i)
        if (static_cast<int64_t>(m.active_lags[i]) == lag)
            return true;
    return false;
}

// Reconstruct the first 8 SYNC with the frozen dense hull taps and the frozen
// CFO, then compare with the independently synthesized receive waveform.
double dense_normalized_residual(const sic::DiscreteCirModel& m,
                                 const std::vector<cf>& y,
                                 const std::vector<cf>& train)
{
    if (m.q < 0 ||
        m.q + static_cast<int64_t>(kTrainLen) >
            static_cast<int64_t>(y.size()))
        return std::numeric_limits<double>::infinity();
    cd num(0.0, 0.0);
    cd den(0.0, 0.0);
    for (size_t mm = 0; mm < kTrainLen; ++mm) {
        cd yhat(0.0, 0.0);
        for (size_t d = 0; d < m.tap_count; ++d) {
            const int64_t xi =
                static_cast<int64_t>(mm) -
                (m.lag_lo + static_cast<int64_t>(d));
            if (xi >= 0 && xi < static_cast<int64_t>(train.size()))
                yhat += cd(m.taps[d]) *
                        cd(train[static_cast<size_t>(xi)]);
        }
        const cd rot = std::polar(
            1.0, 2.0 * kPi * m.cfo_hz * static_cast<double>(mm) / kFs);
        yhat *= rot;
        const cd obs = cd(y[static_cast<size_t>(m.q + static_cast<int64_t>(mm))]);
        num += std::norm(obs - yhat);
        den += std::norm(obs);
    }
    if (!(den.real() > 0.0))
        return std::numeric_limits<double>::infinity();
    return (num / den).real();
}

bool prepare_ok(const Packet& p,
                const sic::DiscreteCirConfig& cfg,
                sic::DiscreteCirScratch& scratch,
                std::string& err)
{
    return sic::prepare_discrete_cir(cfg, templates_of(p), scratch, &err);
}

bool estimate_ok(const Packet& p,
                 const sic::DiscreteCirConfig& cfg,
                 const std::vector<cf>& y,
                 int64_t observed_end,
                 sic::DiscreteCirScratch& scratch,
                 sic::DiscreteCirModel& model,
                 sic::DiscreteCirTimings* timings = nullptr)
{
    return sic::estimate_discrete_cir(y.data(), y.size(), observed_end,
                                      templates_of(p), cfg, scratch, model,
                                      timings);
}

} // namespace

// ---------------------------------------------------------------------------
// Known integer FIR: the peak groups must find the three integer paths and the
// active LS must recover them exactly.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(discrete_cir_integer_fir_peak_groups)
{
    Packet p(10);
    const size_t q = 300;
    const size_t n = q + kTrainLen + 512;
    const std::vector<std::pair<int64_t, cf>> paths = {
        { 0, cf( 0.90f,  0.10f) },
        { 7, cf( 0.35f, -0.20f) },
        {23, cf(-0.25f,  0.15f) }
    };
    std::vector<cf> y = make_integer_trace(n, q, p.train, paths);
    sic::DiscreteCirConfig cfg = make_cfg(n, q);

    sic::DiscreteCirScratch s;
    std::string err;
    BOOST_REQUIRE_MESSAGE(prepare_ok(p, cfg, s, err), err);

    sic::DiscreteCirModel m;
    sic::DiscreteCirTimings timings;
    const int64_t observed_end =
        static_cast<int64_t>(q + kTrainLen + 64);
    BOOST_REQUIRE_MESSAGE(
        estimate_ok(p, cfg, y, observed_end, s, m, &timings),
        "status=" << m.status << " reason=" << reason_of(m));
    BOOST_REQUIRE(m.valid);
    BOOST_CHECK_EQUAL(std::string(m.status), std::string("model_ready"));
    BOOST_CHECK_EQUAL(std::string(m.support_source),
                      std::string("coarse_cir_peak_groups"));
    BOOST_CHECK(std::llabs(m.q - static_cast<int64_t>(q)) <= 1);
    BOOST_CHECK_LE(m.tap_count, cfg.tap_count);
    BOOST_CHECK_LT(m.tap_count, kSps);
    BOOST_CHECK_GE(m.active_count, size_t(3));
    BOOST_CHECK_LE(m.active_count, m.tap_count);
    BOOST_CHECK(!m.active_count || m.active_count <= sic::kDiscreteCirMaxActive);

    const int64_t shift = m.q - static_cast<int64_t>(q);
    for (const auto& path : paths) {
        const int64_t lag = path.first - shift;
        const cf* est = dense_tap_at(m, lag);
        BOOST_REQUIRE_MESSAGE(est != nullptr,
                              "truth lag " << lag << " outside hull ["
                                           << m.lag_lo << ","
                                           << (m.lag_lo + int64_t(m.tap_count))
                                           << ")");
        BOOST_CHECK_MESSAGE(active_has_lag(m, lag),
                            "truth lag " << lag << " is not active");
        const double denom =
            std::max(1e-12, static_cast<double>(std::abs(cd(path.second))));
        const double rel =
            static_cast<double>(std::abs(cd(*est) - cd(path.second))) / denom;
        BOOST_CHECK_LT(rel, 1e-4);
    }
    BOOST_CHECK_LT(m.fit_residual, 1e-8);
    BOOST_CHECK(std::isfinite(m.gain_condition));
    BOOST_CHECK_GE(m.gain_condition, 1.0);
    BOOST_CHECK_GE(m.effective_rank, size_t(1));
    BOOST_CHECK_LE(m.effective_rank, m.active_count);
    BOOST_CHECK_GT(timings.estimate_total_us, 0.0);
}

// ---------------------------------------------------------------------------
// Independent fractional multipath.  The receive waveform comes from a local
// cubic operator, not from this estimator and not from the frozen SIC path.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(discrete_cir_fractional_multipath_independent_operator)
{
    Packet p(10);
    const size_t q = 300;
    const size_t n = q + kTrainLen + 512;
    std::vector<cf> y(n, cf(0.f, 0.f));
    const double cfo_hz = 20000.0;
    add_fractional_path(y, q, p.train, 0.4, cf(1.00f, 0.10f), cfo_hz);
    add_fractional_path(y, q, p.train, 7.6, cf(0.60f, -0.20f), cfo_hz);

    sic::DiscreteCirConfig cfg = make_cfg(n, q);
    sic::DiscreteCirScratch s;
    std::string err;
    BOOST_REQUIRE_MESSAGE(prepare_ok(p, cfg, s, err), err);

    sic::DiscreteCirModel m;
    BOOST_REQUIRE_MESSAGE(
        estimate_ok(p, cfg, y, static_cast<int64_t>(q + kTrainLen + 64), s, m),
        "status=" << m.status << " reason=" << reason_of(m));
    BOOST_REQUIRE(m.valid);
    BOOST_CHECK_LE(m.tap_count, cfg.tap_count);
    BOOST_CHECK_GE(m.active_count, size_t(3));

    const double residual = dense_normalized_residual(m, y, p.train);
    BOOST_CHECK_MESSAGE(residual < 1e-2,
                        "fractional dense residual=" << residual);
    BOOST_CHECK_LT(std::abs(m.cfo_hz - cfo_hz), 300.0);
}

// ---------------------------------------------------------------------------
// Non-zero CFO plus a negative absolute lag.  The strong +3 path dominates
// acquisition, so the -2 path appears as a negative lag relative to q.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(discrete_cir_negative_lag_and_cfo)
{
    Packet p(10);
    const size_t q = 300;
    const size_t n = q + kTrainLen + 512;
    const std::vector<std::pair<int64_t, cf>> paths = {
        {-2, cf(0.40f,  0.10f)},
        { 3, cf(1.00f, -0.10f)}
    };
    const double cfo_hz = -50000.0;
    std::vector<cf> y = make_integer_trace(n, q, p.train, paths, cfo_hz);

    sic::DiscreteCirConfig cfg = make_cfg(n, q);
    sic::DiscreteCirScratch s;
    std::string err;
    BOOST_REQUIRE_MESSAGE(prepare_ok(p, cfg, s, err), err);

    sic::DiscreteCirModel m;
    BOOST_REQUIRE_MESSAGE(
        estimate_ok(p, cfg, y, static_cast<int64_t>(q + kTrainLen + 64), s, m),
        "status=" << m.status << " reason=" << reason_of(m));
    BOOST_REQUIRE(m.valid);
    BOOST_CHECK_LT(std::abs(m.cfo_hz - cfo_hz), 300.0);

    const int64_t shift = m.q - static_cast<int64_t>(q);
    const int64_t neg_lag = -2 - shift;
    const int64_t pos_lag = 3 - shift;
    BOOST_REQUIRE(dense_tap_at(m, neg_lag) != nullptr);
    BOOST_REQUIRE(dense_tap_at(m, pos_lag) != nullptr);
    BOOST_CHECK_MESSAGE(active_has_lag(m, neg_lag),
                        "negative lag " << neg_lag << " not active");
    BOOST_CHECK_MESSAGE(active_has_lag(m, pos_lag),
                        "positive lag " << pos_lag << " not active");
    BOOST_CHECK_LT(neg_lag, int64_t(0));
    BOOST_CHECK_LT(m.lag_lo, int64_t(0));
    BOOST_CHECK_GE(m.lag_lo, cfg.lag_lo);

    for (const auto& path : paths) {
        const int64_t lag = path.first - shift;
        const cf* est = dense_tap_at(m, lag);
        BOOST_REQUIRE(est != nullptr);
        const double denom =
            std::max(1e-12, static_cast<double>(std::abs(cd(path.second))));
        const double rel =
            static_cast<double>(std::abs(cd(*est) - cd(path.second))) / denom;
        // The CFO is estimated from 8 repetitions (a few Hz residual error),
        // so a non-zero-CFO tap has a small phase/amplitude bias.  Zero-CFO
        // integer recovery is checked at 1e-4 in the first case.
        BOOST_CHECK_LT(rel, 5e-3);
    }
}

// ---------------------------------------------------------------------------
// Strict prefix isolation: mutating after observed_end (including NaN) must
// not change the frozen model, while an actually truncated training span must
// fail with the documented stable reason.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(discrete_cir_strict_prefix_isolation)
{
    Packet p(10);
    const size_t q = 300;
    const size_t n = q + kTrainLen + 1024;
    const std::vector<std::pair<int64_t, cf>> paths = {
        {0, cf(0.90f, 0.10f)},
        {7, cf(0.35f, -0.20f)},
        {23, cf(-0.25f, 0.15f)}
    };
    std::vector<cf> y = make_integer_trace(n, q, p.train, paths);
    sic::DiscreteCirConfig cfg = make_cfg(n, q);

    sic::DiscreteCirScratch s;
    std::string err;
    BOOST_REQUIRE_MESSAGE(prepare_ok(p, cfg, s, err), err);

    const int64_t observed_end =
        static_cast<int64_t>(q + kTrainLen + 64);
    sic::DiscreteCirModel first;
    BOOST_REQUIRE_MESSAGE(
        estimate_ok(p, cfg, y, observed_end, s, first),
        "status=" << first.status << " reason=" << reason_of(first));
    BOOST_REQUIRE(first.valid);

    // Everything at/after observed_end is outside the strict prefix.  Use
    // finite garbage and a NaN to prove the estimator never looks there.
    for (size_t i = static_cast<size_t>(observed_end); i < y.size(); ++i)
        y[i] = cf(7.5f, -3.25f);
    y[static_cast<size_t>(observed_end) + 10] =
        cf(std::numeric_limits<float>::quiet_NaN(), 0.f);

    sic::DiscreteCirModel second;
    BOOST_REQUIRE_MESSAGE(
        estimate_ok(p, cfg, y, observed_end, s, second),
        "status=" << second.status << " reason=" << reason_of(second));
    BOOST_REQUIRE(second.valid);
    BOOST_CHECK_EQUAL(second.model_id, first.model_id);
    BOOST_CHECK_EQUAL(second.q, first.q);
    BOOST_CHECK_EQUAL(second.cfo_hz, first.cfo_hz);
    BOOST_CHECK_EQUAL(second.tap_count, first.tap_count);
    BOOST_CHECK_EQUAL(second.active_count, first.active_count);
    for (size_t i = 0; i < first.tap_count; ++i) {
        BOOST_CHECK_EQUAL(second.taps[i].real(), first.taps[i].real());
        BOOST_CHECK_EQUAL(second.taps[i].imag(), first.taps[i].imag());
    }

    // q + 4*sps is below the full 8-SYMBOL training span.
    const int64_t truncated =
        static_cast<int64_t>(q + 4 * kSps);
    sic::DiscreteCirModel short_model;
    const bool ok = estimate_ok(p, cfg, y, truncated, s, short_model);
    BOOST_CHECK(!ok);
    BOOST_CHECK(!short_model.valid);
    BOOST_CHECK_EQUAL(reason_of(short_model),
                      std::string("short_training_prefix"));
}

// ---------------------------------------------------------------------------
// Non-finite inputs, zero input, bad geometry, unprepared scratch and invalid
// runtime config all fail closed without committing a model.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(discrete_cir_invalid_inputs_fail_closed)
{
    Packet p(10);
    const size_t q = 300;
    const size_t n = q + kTrainLen + 512;
    std::vector<cf> y = make_integer_trace(
        n, q, p.train, {{0, cf(1.0f, 0.0f)}});
    sic::DiscreteCirConfig cfg = make_cfg(n, q);

    sic::DiscreteCirScratch s;
    std::string err;
    BOOST_REQUIRE_MESSAGE(prepare_ok(p, cfg, s, err), err);

    // NaN inside the visible prefix.
    std::vector<cf> nan_y = y;
    nan_y[10] = cf(std::numeric_limits<float>::quiet_NaN(), 0.f);
    sic::DiscreteCirModel m;
    BOOST_CHECK(!estimate_ok(p, cfg, nan_y,
                             static_cast<int64_t>(q + kTrainLen), s, m));
    BOOST_CHECK(!m.valid);
    BOOST_CHECK(!reason_of(m).empty());

    // Inf inside the visible prefix.
    nan_y = y;
    nan_y[11] = cf(std::numeric_limits<float>::infinity(), 0.f);
    BOOST_CHECK(!estimate_ok(p, cfg, nan_y,
                             static_cast<int64_t>(q + kTrainLen), s, m));
    BOOST_CHECK(!m.valid);

    // Zero input.
    std::vector<cf> zero(y.size(), cf(0.f, 0.f));
    BOOST_CHECK(!estimate_ok(p, cfg, zero,
                             static_cast<int64_t>(q + kTrainLen), s, m));
    BOOST_CHECK(!m.valid);

    // Runtime config domain error (after a successful prepare).
    sic::DiscreteCirConfig bad_runtime = cfg;
    bad_runtime.detect_threshold =
        std::numeric_limits<double>::quiet_NaN();
    BOOST_CHECK(!estimate_ok(p, bad_runtime, y,
                             static_cast<int64_t>(q + kTrainLen), s, m));
    BOOST_CHECK(!m.valid);

    // Unprepared scratch: no geometry, no cache, no prepared identity.
    sic::DiscreteCirScratch raw;
    BOOST_CHECK(!estimate_ok(p, cfg, y, static_cast<int64_t>(q + kTrainLen),
                             raw, m));
    BOOST_CHECK(!m.valid);
    BOOST_CHECK(!reason_of(m).empty());

    // Bad preparation geometry / calibrated parameters.
    const auto bad_prepare = [&](const sic::DiscreteCirConfig& bad) {
        sic::DiscreteCirScratch bs;
        std::string be;
        return !prepare_ok(p, bad, bs, be) && !be.empty();
    };
    {
        sic::DiscreteCirConfig c = cfg;
        c.tap_count = 0;
        BOOST_CHECK(bad_prepare(c));
    }
    {
        sic::DiscreteCirConfig c = cfg;
        c.tap_count = 1016;
        BOOST_CHECK(bad_prepare(c));
    }
    {
        sic::DiscreteCirConfig c = cfg;
        c.lag_lo = 1;
        BOOST_CHECK(bad_prepare(c));
    }
    {
        sic::DiscreteCirConfig c = cfg;
        c.lambda = 0.0;
        BOOST_CHECK(bad_prepare(c));
    }
    {
        sic::DiscreteCirConfig c = cfg;
        c.peak_floor = 1.5;
        BOOST_CHECK(bad_prepare(c));
    }
    {
        sic::DiscreteCirConfig c = cfg;
        c.peak_min_spacing = 0;
        BOOST_CHECK(bad_prepare(c));
    }
    {
        sic::DiscreteCirConfig c = cfg;
        c.max_window_samples = 0;
        BOOST_CHECK(bad_prepare(c));
    }
}

// ---------------------------------------------------------------------------
// A wrong preamble code must not commit.  This exercises both the detection
// gate and the fit gate; only fail-closed is required.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(discrete_cir_wrong_template_gate)
{
    Packet p10(10);
    Packet p9(9);
    const size_t q = 300;
    const size_t n = q + kTrainLen + 512;
    std::vector<cf> y9 = make_integer_trace(
        n, q, p9.train, {{0, cf(1.0f, 0.0f)}});
    sic::DiscreteCirConfig cfg = make_cfg(n, q);

    sic::DiscreteCirScratch s;
    std::string err;
    BOOST_REQUIRE_MESSAGE(prepare_ok(p10, cfg, s, err), err);

    sic::DiscreteCirModel m;
    BOOST_CHECK(!estimate_ok(p10, cfg, y9,
                             static_cast<int64_t>(q + kTrainLen), s, m));
    BOOST_CHECK(!m.valid);
    BOOST_CHECK(!reason_of(m).empty());
}

// ---------------------------------------------------------------------------
// Peak groups vs a wide continuous hull.  A delayed copy of the same preamble
// is exactly representable by a dense hull LS, but its coarse peak is below
// peak_floor and therefore MUST stay inactive.  The dense hull retains the
// zero tap, so the model keeps only the comm peaks.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(discrete_cir_peak_groups_reject_second_preamble)
{
    Packet p(10);
    const size_t q = 300;
    const size_t n = q + kTrainLen + 512;
    const std::vector<std::pair<int64_t, cf>> paths = {
        { 0, cf(1.00f, 0.00f)},
        { 7, cf(0.50f, 0.00f)},
        {23, cf(0.30f, 0.00f)}
    };
    std::vector<cf> y = make_integer_trace(n, q, p.train, paths);

    // Second comm preamble 12 samples later at -26 dB relative amplitude: well
    // below peak_floor=0.15.  A continuous hull LS could fit it exactly with a
    // tap at lag 12; the peak-group estimator must leave that tap zero.
    const int64_t second_lag = 12;
    const float second_gain = 0.05f;
    for (size_t m = 0; m < kTrainLen; ++m)
        y[q + static_cast<size_t>(second_lag) + m] +=
            cf(second_gain, 0.f) * p.train[m];

    sic::DiscreteCirConfig cfg = make_cfg(n, q);
    sic::DiscreteCirScratch s;
    std::string err;
    BOOST_REQUIRE_MESSAGE(prepare_ok(p, cfg, s, err), err);

    sic::DiscreteCirModel m;
    BOOST_REQUIRE_MESSAGE(
        estimate_ok(p, cfg, y, static_cast<int64_t>(q + kTrainLen + 64), s, m),
        "status=" << m.status << " reason=" << reason_of(m));
    BOOST_REQUIRE(m.valid);
    BOOST_CHECK_LE(m.tap_count, cfg.tap_count);
    BOOST_CHECK_LT(m.tap_count, kSps);

    const int64_t shift = m.q - static_cast<int64_t>(q);
    for (const auto& path : paths) {
        const int64_t lag = path.first - shift;
        BOOST_REQUIRE(dense_tap_at(m, lag) != nullptr);
        BOOST_CHECK_MESSAGE(active_has_lag(m, lag),
                            "comm peak lag " << lag << " missing");
    }

    // The delayed preamble must lie inside the dense hull but outside the
    // active set, and its dense tap must be EXACTLY zero.
    const int64_t delayed = second_lag - shift;
    const cf* delayed_tap = dense_tap_at(m, delayed);
    BOOST_REQUIRE_MESSAGE(delayed_tap != nullptr,
                          "second preamble lag " << delayed
                                                 << " outside hull ["
                                                 << m.lag_lo << ","
                                                 << (m.lag_lo +
                                                     int64_t(m.tap_count))
                                                 << ")");
    BOOST_CHECK(!active_has_lag(m, delayed));
    BOOST_CHECK_EQUAL(delayed_tap->real(), 0.0f);
    BOOST_CHECK_EQUAL(delayed_tap->imag(), 0.0f);
    BOOST_CHECK_LT(m.active_count, m.tap_count);
    // Active taps fit the comm peaks, not the delayed copy: a nonzero residual
    // is expected because the second preamble is deliberately excluded.
    BOOST_CHECK_GT(m.fit_residual, 1e-6);
}

// Ipatov correlation must retain pulse shaping in the response.  Compare
// against an independent forward correlation, then apply to UNIT impulses.
BOOST_AUTO_TEST_CASE(ipatov_cir_response_and_impulse_reconstruction)
{
    Packet p;
    const size_t q = 256, n = p.wave.size() + 1024;
    auto cfg = make_cfg(n, q, 128);
    cfg.ipatov_correlation = true;
    const cf gain(0.7f, 0.3f);
    std::vector<cf> y(n, cf{});
    for (size_t i = 0; i < p.wave.size(); ++i) y[q+i] = gain*p.wave[i];
    sic::DiscreteCirScratch s;
    sic::DiscreteCirModel m;
    sic::DiscreteCirTimings dt;
    const int64_t observed = q + kTrainLen;
    std::string err;
    BOOST_REQUIRE_MESSAGE(prepare_ok(p,cfg,s,err),err);
    BOOST_REQUIRE_MESSAGE(estimate_ok(p, cfg, y, observed, s, m, &dt),
                          m.status << " " << reason_of(m));
    BOOST_REQUIRE(m.impulse_template);
    BOOST_CHECK_EQUAL(m.tap_count, 128u);
    BOOST_CHECK_EQUAL(dt.projection_us, 0.0);
    BOOST_CHECK_EQUAL(dt.solve_us, 0.0);
    const int8_t* code = demod::GetPreambleCode(10);
    double energy = 0;
    for (size_t c=0;c<127;++c) energy += code[c]*code[c];
    for (size_t d=0;d<m.tap_count;++d) {
        cd ref{}; size_t reps=0;
        for (size_t k=1;k<kReps;++k) {
            const int64_t lo=m.q+int64_t(k*kSps)+cfg.lag_lo;
            if (lo<0 || lo+int64_t(kSps+cfg.tap_count-1)>observed) continue;
            cd one{};
            for (size_t c=0;c<127;++c)
                one += double(code[c])*cd(y[size_t(lo+int64_t(d+8*c))]);
            ref += one; ++reps;
        }
        ref /= energy*reps;
        BOOST_CHECK_LT(std::abs(cd(m.taps[d])-ref), 2e-5);
    }
    auto altered=y;
    for(size_t i=size_t(observed);i<n;++i) altered[i]=cf(123,-37);
    sic::DiscreteCirModel other;
    BOOST_REQUIRE(sic::estimate_discrete_cir(altered.data(),n,observed,
                       templates_of(p),cfg,s,other));
    BOOST_CHECK_EQUAL(m.model_id,other.model_id);
    const size_t original_z_size=s.z.size();
    s.z.resize(kTrainLen-8); // satisfies legacy rows, too short for raw fit check
    BOOST_CHECK(!sic::estimate_discrete_cir(y.data(),n,observed,
                       templates_of(p),cfg,s,other));
    BOOST_CHECK_EQUAL(reason_of(other),"scratch_capacity");
    s.z.resize(original_z_size);
    std::vector<cf> out(n),ihat(n);
    // A complete decoded packet exercises the impulse remodulator too.
    auto wire=mod::make_random_psdu(30,1010,true);
    mod::detail::encode_phr19(wire.size(),64,false,s.expected_phr);
    const std::vector<uint8_t> phr(s.expected_phr.begin(),s.expected_phr.end());
    // The unit-real wrapper must not read/write obsolete complex packet
    // intermediates. Leave them poisoned to catch accidental reuse.
    const cf poison(std::numeric_limits<float>::quiet_NaN(),0.f);
    std::fill(s.waveform.begin(),s.waveform.end(),poison);
    std::fill(s.scaled_waveform.begin(),s.scaled_waveform.end(),poison);
    auto result=sic::fullpacket_cancel_discrete(y.data(),n,phr,wire,true,
                 p.profile,m,p.sync.data(),p.sync.size(),0,n,s,out.data(),ihat.data());
    BOOST_REQUIRE_MESSAGE(result.committed,result.reason);
    double num=0,den=0;
    for(size_t i=0;i<n;++i) {num+=std::norm(out[i]);den+=std::norm(y[i]);}
    BOOST_CHECK_LT(num/den,0.01); // >=20 dB, including finite-code sidelobes
    BOOST_CHECK_CLOSE(result.waveform_scale,1.0,0.001);
    BOOST_CHECK(std::all_of(s.waveform.begin(),s.waveform.end(),
                           [](cf x){return std::isnan(x.real());}));
    BOOST_CHECK(std::all_of(s.scaled_waveform.begin(),s.scaled_waveform.end(),
                           [](cf x){return std::isnan(x.real());}));

    // Sparse pulse-list path with the same public observation/model geometry.
    // The zero-CFO case must match the unit-real reconstruction exactly.
    {
        auto sparse_cfg=cfg; sparse_cfg.sparse_pulse_reconstruct=true;
        sic::DiscreteCirScratch ss; sic::DiscreteCirModel sm;
        BOOST_REQUIRE_MESSAGE(prepare_ok(p,sparse_cfg,ss,err),err);
        BOOST_REQUIRE_MESSAGE(estimate_ok(p,sparse_cfg,y,observed,ss,sm),
                              sm.status << " " << reason_of(sm));
        BOOST_REQUIRE(sm.impulse_template);
        BOOST_CHECK_EQUAL(sm.q,m.q);
        BOOST_CHECK_EQUAL(sm.cfo_hz,m.cfo_hz);
        BOOST_CHECK_EQUAL(sm.tap_count,m.tap_count);
        BOOST_CHECK_EQUAL(std::memcmp(sm.taps.data(),m.taps.data(),
                                     m.tap_count*sizeof(cf)),0);
        std::fill(ss.waveform.begin(),ss.waveform.end(),poison);
        std::fill(ss.scaled_waveform.begin(),ss.scaled_waveform.end(),poison);
        std::vector<cf> out2(n),ihat2(n);
        auto result2=sic::fullpacket_cancel_discrete(y.data(),n,phr,wire,true,
                     p.profile,sm,p.sync.data(),p.sync.size(),0,n,ss,
                     out2.data(),ihat2.data());
        BOOST_REQUIRE_MESSAGE(result2.committed,result2.reason);
        BOOST_CHECK_GT(result2.pulse_event_count,0u);
        BOOST_CHECK_EQUAL(result2.pulse_packet_samples,p.wave.size());
        BOOST_CHECK_EQUAL(std::memcmp(ihat.data(),ihat2.data(),n*sizeof(cf)),0);
        // Transaction reuse must also work without ihat and preserve bypass
        // on early frame/input rejection, even with dirty destination buffers.
        std::vector<cf> out_only(n,cf(7.f,8.f));
        auto ro=sic::fullpacket_cancel_discrete(y.data(),n,phr,wire,true,
                     p.profile,sm,p.sync.data(),p.sync.size(),0,n,ss,
                     out_only.data(),nullptr);
        BOOST_REQUIRE_MESSAGE(ro.committed,ro.reason);
        BOOST_CHECK_EQUAL(std::memcmp(out2.data(),out_only.data(),n*sizeof(cf)),0);
        // The sparse transaction intentionally skips a whole-window finite
        // scan. A nonfinite sample outside reconstruction support is passed
        // through; it must not trigger the former nonfinite_input rejection.
        auto unchecked_y=y;
        unchecked_y.back()=poison;
        auto ru=sic::fullpacket_cancel_discrete(unchecked_y.data(),n,phr,wire,true,
                     p.profile,sm,p.sync.data(),p.sync.size(),0,n,ss,
                     out2.data(),ihat2.data());
        BOOST_REQUIRE_MESSAGE(ru.committed,ru.reason);
        BOOST_CHECK(std::isnan(out2.back().real()));
        std::fill(ihat2.begin(),ihat2.end(),cf(7.f,8.f));
        auto rf=sic::fullpacket_cancel_discrete(y.data(),n,phr,wire,false,
                     p.profile,sm,p.sync.data(),p.sync.size(),0,n,ss,
                     out2.data(),ihat2.data());
        BOOST_CHECK_EQUAL(std::string(rf.reason),"fcs_failed");
        BOOST_CHECK_EQUAL(std::memcmp(y.data(),out2.data(),n*sizeof(cf)),0);
        BOOST_CHECK(std::all_of(ihat2.begin(),ihat2.end(),[](cf x){return x==cf{};}));

        auto truncated=sm;
        truncated.q=int64_t(n)-1;
        truncated.phase_reference_sample=truncated.q;
        std::fill(out2.begin(),out2.end(),cf(7.f,8.f));
        std::fill(ihat2.begin(),ihat2.end(),cf(7.f,8.f));
        auto rt=sic::fullpacket_cancel_discrete(y.data(),n,phr,wire,true,
                     p.profile,truncated,p.sync.data(),p.sync.size(),0,n,ss,
                     out2.data(),ihat2.data());
        BOOST_CHECK_EQUAL(std::string(rt.reason),"context_truncated");
        BOOST_CHECK_EQUAL(std::memcmp(y.data(),out2.data(),n*sizeof(cf)),0);
        BOOST_CHECK(std::all_of(ihat2.begin(),ihat2.end(),[](cf x){return x==cf{};}));
        BOOST_CHECK(std::all_of(ss.waveform.begin(),ss.waveform.end(),
                               [](cf x){return std::isnan(x.real());}));
        BOOST_CHECK(std::all_of(ss.scaled_waveform.begin(),
                               ss.scaled_waveform.end(),
                               [](cf x){return std::isnan(x.real());}));
    }

    auto impossible=cfg; impossible.fit_residual_max=1e-14;
    sic::DiscreteCirScratch rejected; sic::DiscreteCirModel bad;
    BOOST_REQUIRE_MESSAGE(prepare_ok(p,impossible,rejected,err),err);
    auto damaged=y; damaged[q+17]+=cf(0.1f,-0.1f);
    BOOST_CHECK(!estimate_ok(p,impossible,damaged,observed,rejected,bad));
    BOOST_CHECK(!bad.valid);
}

BOOST_AUTO_TEST_CASE(test_discrete_cir_c1_c2_c4_optimizations)
{
    Packet p(10, 32);
    const size_t q = 1200;
    const size_t n = q + p.wave.size() + 200;
    auto cfg = make_cfg(n, q, 20);
    cfg.lag_lo = -1;
    cfg.ipatov_correlation = true;
    cfg.sparse_pulse_reconstruct = true;

    // C4: verify prepare_discrete_cir does NOT re-reserve if already sized
    sic::DiscreteCirScratch scratch;
    scratch.reserve(cfg.max_window_samples);
    const auto* factor_cache_ptr = scratch.factor_cache.data();
    const auto* z_ptr = scratch.z.data();
    std::string err;
    auto tmpl = templates_of(p);
    BOOST_REQUIRE_MESSAGE(sic::prepare_discrete_cir(cfg, tmpl, scratch, &err), err);
    // Capacity check must not trigger re-reserve
    BOOST_CHECK_EQUAL(scratch.factor_cache.data(), factor_cache_ptr);
    BOOST_CHECK_EQUAL(scratch.z.data(), z_ptr);

    // Create a clean received signal with packet placed at q
    std::vector<cf> y(n, cf(0.f, 0.f));
    std::copy(p.wave.begin(), p.wave.end(), y.begin() + q);

    const int64_t observed_end = static_cast<int64_t>(q + 8 * kSps);

    // C1: NaN inside visible_end must fail with nonfinite_input
    {
        auto bad_y = y;
        bad_y[q / 2] = cf(std::numeric_limits<float>::quiet_NaN(), 0.f);
        sic::DiscreteCirModel m;
        BOOST_CHECK(!sic::estimate_discrete_cir(bad_y.data(), n, observed_end, tmpl, cfg, scratch, m));
        BOOST_CHECK_EQUAL(reason_of(m), "nonfinite_input");
    }

    // C1: NaN outside visible_end must NOT trigger nonfinite_input
    {
        auto pass_y = y;
        pass_y[static_cast<size_t>(observed_end) + 10] = cf(std::numeric_limits<float>::quiet_NaN(), 0.f);
        sic::DiscreteCirModel m;
        BOOST_CHECK(sic::estimate_discrete_cir(pass_y.data(), n, observed_end, tmpl, cfg, scratch, m));
        BOOST_CHECK(m.valid);
    }

    // C2: Periodic residual prediction
    {
        sic::DiscreteCirModel m;
        BOOST_REQUIRE(sic::estimate_discrete_cir(y.data(), n, observed_end, tmpl, cfg, scratch, m));
        BOOST_CHECK(m.valid);
        BOOST_CHECK_EQUAL(std::string(m.status), "model_ready");
        BOOST_CHECK_LE(m.fit_residual, cfg.fit_residual_max);
        BOOST_CHECK_GT(m.fit_residual, 0.0);
    }
}

// ---------------------------------------------------------------------------
// Track A QA: Real vs Complex Template Equivalence
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_discrete_cir_real_vs_complex_template_equivalence)
{
    Packet p(10, 32);
    const size_t q = 400;
    const size_t n = q + p.wave.size() + 512;
    const int64_t observed_end = static_cast<int64_t>(q + kTrainLen + 64);

    std::vector<cf> y(n, cf(0.f, 0.f));
    std::copy(p.wave.begin(), p.wave.end(), y.begin() + q);

    auto cfg = make_cfg(n, q, 30);
    sic::DiscreteCirScratch scratch_real;
    std::string err;
    BOOST_REQUIRE_MESSAGE(prepare_ok(p, cfg, scratch_real, err), err);
    BOOST_REQUIRE(scratch_real.sync_real.is_real);

    // Direct comparison of real_template_dot_single / 4lag vs VOLK complex dot
    const size_t Ls = scratch_real.sync_len;
    for (size_t lag = 0; lag + 4 + Ls <= y.size(); lag += 7) {
        cf dot_c[4];
        for (size_t j = 0; j < 4; ++j) {
            volk_32fc_x2_conjugate_dot_prod_32fc_u(
                &dot_c[j], y.data() + lag + j, scratch_real.sync_norm.data(), static_cast<unsigned>(Ls));
        }

        cf dot_single[4];
        for (size_t j = 0; j < 4; ++j) {
            dot_single[j] = demod::rxopt::real_template_dot_single(
                y.data() + lag + j, scratch_real.sync_real.taps_dup.data(), Ls);
        }

        cf dot_4lag[4];
        demod::rxopt::real_template_dot_4lag(
            y.data() + lag, scratch_real.sync_real.taps_dup.data(), Ls, dot_4lag);

        for (size_t j = 0; j < 4; ++j) {
            const double err_single = std::abs(dot_single[j] - dot_c[j]);
            const double norm_c = std::abs(dot_c[j]);
            if (norm_c > 0.05) {
                BOOST_CHECK_LT(err_single / norm_c, 2e-5);
            }
            const double err_4lag = std::abs(dot_4lag[j] - dot_c[j]);
            if (norm_c > 0.05) {
                BOOST_CHECK_LT(err_4lag / norm_c, 2e-5);
            }
            BOOST_CHECK_LT(err_single, 1e-5);
            BOOST_CHECK_LT(err_4lag, 1e-5);
            BOOST_CHECK_LT(std::abs(dot_4lag[j] - dot_single[j]), 1e-5);
        }
    }

    // Now verify end-to-end model estimation
    sic::DiscreteCirModel m_real;
    BOOST_REQUIRE(sic::estimate_discrete_cir(
        y.data(), n, observed_end, templates_of(p), cfg, scratch_real, m_real));
    BOOST_REQUIRE(m_real.valid);

    // Forced complex template path (non-zero imaginary part > 1e-6f)
    std::vector<cf> comp_sync = p.sync;
    for (size_t i = 0; i < comp_sync.size(); ++i) {
        comp_sync[i] += cf(0.f, 0.01f * comp_sync[i].real());
    }
    sic::DiscreteCirTemplates tmpl_comp;
    tmpl_comp.sync = comp_sync.data();
    tmpl_comp.sync_len = comp_sync.size();
    tmpl_comp.train = p.train.data();
    tmpl_comp.train_len = p.train.size();

    sic::DiscreteCirScratch scratch_comp;
    BOOST_REQUIRE(sic::prepare_discrete_cir(cfg, tmpl_comp, scratch_comp, &err));
    BOOST_REQUIRE(!scratch_comp.sync_real.is_real);

    sic::DiscreteCirModel m_comp;
    BOOST_REQUIRE(sic::estimate_discrete_cir(
        y.data(), n, observed_end, tmpl_comp, cfg, scratch_comp, m_comp));
    BOOST_REQUIRE(m_comp.valid);
    BOOST_CHECK_EQUAL(m_comp.q, m_real.q);
    BOOST_CHECK_LT(std::abs(m_comp.detection_metric - m_real.detection_metric), 1e-2);
}

// ---------------------------------------------------------------------------
// Track A QA: Localized Energy Prefix Sum & Scratch Isolation
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_discrete_cir_localized_energy_scratch_isolation)
{
    Packet p(10, 32);
    const size_t q = 500;
    const size_t n = q + p.wave.size() + 512;
    const int64_t observed_end = static_cast<int64_t>(q + kTrainLen + 64);

    std::vector<cf> y(n, cf(0.f, 0.f));
    std::copy(p.wave.begin(), p.wave.end(), y.begin() + q);

    auto cfg = make_cfg(n, q, 25);

    // 1. Clean scratch run
    sic::DiscreteCirScratch s_clean;
    std::string err;
    BOOST_REQUIRE(sic::prepare_discrete_cir(cfg, templates_of(p), s_clean, &err));
    sic::DiscreteCirModel m_clean;
    BOOST_REQUIRE(sic::estimate_discrete_cir(
        y.data(), n, observed_end, templates_of(p), cfg, s_clean, m_clean));
    BOOST_REQUIRE(m_clean.valid);

    // 2. Poisoned scratch run (simulate previous dirty packet)
    sic::DiscreteCirScratch s_dirty;
    BOOST_REQUIRE(sic::prepare_discrete_cir(cfg, templates_of(p), s_dirty, &err));
    std::fill(s_dirty.energy_prefix.begin(), s_dirty.energy_prefix.end(),
              std::numeric_limits<double>::quiet_NaN());
    std::fill(s_dirty.cfo_rot.begin(), s_dirty.cfo_rot.end(),
              cd(std::numeric_limits<double>::quiet_NaN(), 0.0));

    sic::DiscreteCirModel m_dirty;
    BOOST_REQUIRE(sic::estimate_discrete_cir(
        y.data(), n, observed_end, templates_of(p), cfg, s_dirty, m_dirty));
    BOOST_REQUIRE(m_dirty.valid);

    // Models must match identically, not reading contaminated scratch
    BOOST_CHECK_EQUAL(m_dirty.model_id, m_clean.model_id);
    BOOST_CHECK_EQUAL(m_dirty.q, m_clean.q);
    BOOST_CHECK_LT(std::abs(m_dirty.detection_metric - m_clean.detection_metric), 1e-9);
    BOOST_CHECK_LT(std::abs(m_dirty.fit_residual - m_clean.fit_residual), 1e-9);
}

// ---------------------------------------------------------------------------
// Track A QA: Visible Prefix NaN/Inf Detection
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_discrete_cir_visible_prefix_nan_inf_detection)
{
    Packet p(10, 32);
    const size_t q = 300;
    const size_t n = q + p.wave.size() + 512;
    const int64_t observed_end = static_cast<int64_t>(q + kTrainLen + 64);

    std::vector<cf> y(n, cf(0.f, 0.f));
    std::copy(p.wave.begin(), p.wave.end(), y.begin() + q);

    auto cfg = make_cfg(n, q, 25);
    sic::DiscreteCirScratch s;
    std::string err;
    BOOST_REQUIRE(sic::prepare_discrete_cir(cfg, templates_of(p), s, &err));

    const float qnan = std::numeric_limits<float>::quiet_NaN();
    const float pinf = std::numeric_limits<float>::infinity();
    const float ninf = -std::numeric_limits<float>::infinity();

    const std::vector<std::pair<size_t, cf>> test_cases = {
        {0, cf(qnan, 0.f)},
        {0, cf(0.f, qnan)},
        {q / 2, cf(pinf, 0.f)},
        {q + 10, cf(0.f, ninf)},
        {static_cast<size_t>(observed_end - 1), cf(qnan, 0.f)},
        {static_cast<size_t>(observed_end - 1), cf(0.f, pinf)}
    };

    for (const auto& tc : test_cases) {
        auto bad_y = y;
        bad_y[tc.first] = tc.second;
        sic::DiscreteCirModel m;
        BOOST_CHECK(!sic::estimate_discrete_cir(
            bad_y.data(), n, observed_end, templates_of(p), cfg, s, m));
        BOOST_CHECK_EQUAL(reason_of(m), "nonfinite_input");
    }

    // Outside visible prefix must be ignored
    auto pass_y = y;
    pass_y[static_cast<size_t>(observed_end)] = cf(qnan, 0.f);
    pass_y[static_cast<size_t>(observed_end + 10)] = cf(pinf, 0.f);
    sic::DiscreteCirModel m_pass;
    BOOST_CHECK(sic::estimate_discrete_cir(
        pass_y.data(), n, observed_end, templates_of(p), cfg, s, m_pass));
    BOOST_CHECK(m_pass.valid);
}

// ---------------------------------------------------------------------------
// Track A QA: Non-Zero CFO Model & Residual
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_discrete_cir_nonzero_cfo_model_and_residual)
{
    Packet p(10, 32);
    const size_t q = 350;
    const size_t n = q + kTrainLen + 512;
    const int64_t observed_end = static_cast<int64_t>(q + kTrainLen + 64);

    const double cfo_hz = 35000.0;
    const std::vector<std::pair<int64_t, cf>> paths = {
        {0, cf(1.00f, 0.00f)},
        {7, cf(0.50f, 0.20f)}
    };
    std::vector<cf> y = make_integer_trace(n, q, p.train, paths, cfo_hz);

    auto cfg = make_cfg(n, q, 30);
    sic::DiscreteCirScratch s;
    std::string err;
    BOOST_REQUIRE(sic::prepare_discrete_cir(cfg, templates_of(p), s, &err));

    sic::DiscreteCirModel m;
    BOOST_REQUIRE(sic::estimate_discrete_cir(
        y.data(), n, observed_end, templates_of(p), cfg, s, m));
    BOOST_REQUIRE(m.valid);
    BOOST_CHECK_LT(std::abs(m.cfo_hz - cfo_hz), 300.0);
    BOOST_CHECK_LE(m.fit_residual, cfg.fit_residual_max);
    BOOST_CHECK_GT(m.fit_residual, 0.0);

    const double residual = dense_normalized_residual(m, y, p.train);
    BOOST_CHECK_LT(residual, 1e-2);

    const int64_t shift = m.q - static_cast<int64_t>(q);
    BOOST_REQUIRE(dense_tap_at(m, 0 - shift) != nullptr);
    BOOST_REQUIRE(dense_tap_at(m, 7 - shift) != nullptr);
}

BOOST_AUTO_TEST_CASE(test_discrete_cir_zero_cfo_bypass)
{
    Packet p(10, 32);
    const size_t q = 350;
    const size_t n = q + p.wave.size() + 512;
    const int64_t observed_end = static_cast<int64_t>(q + kTrainLen);

    const double cfo_hz = 0.0;
    std::vector<cf> y(n, cf{});
    for (size_t i = 0; i < p.wave.size(); ++i) {
        y[q + i] = p.wave[i];
    }

    auto cfg = make_cfg(n, q, 40);
    cfg.ipatov_correlation = true;
    sic::DiscreteCirScratch s;
    std::string err;
    BOOST_REQUIRE_MESSAGE(sic::prepare_discrete_cir(cfg, templates_of(p), s, &err), err);

    sic::DiscreteCirModel m;
    BOOST_REQUIRE(sic::estimate_discrete_cir(
        y.data(), n, observed_end, templates_of(p), cfg, s, m));
    BOOST_REQUIRE(m.valid);
    BOOST_CHECK_EQUAL(m.support_source, "ipatov_correlation");
    BOOST_CHECK(m.impulse_template);
    BOOST_CHECK_SMALL(m.cfo_hz, 50.0);
    BOOST_CHECK_LE(m.fit_residual, cfg.fit_residual_max);

    // Explicit scalar reference calculation without vectorization/bypass
    const double omega = 2.0 * kPi * m.cfo_hz / kFs;
    const size_t taps = cfg.tap_count;
    const size_t width = kSps + taps - 1;
    std::vector<cd> ref_avg(width, cd{});
    size_t ref_valid = 0;
    for (size_t k = 1; k < cfg.fit_reps; ++k) {
        const int64_t lo = m.q + int64_t(k * kSps) + cfg.lag_lo;
        if (lo < 0 || lo + int64_t(width) > observed_end) continue;
        const int64_t rel_lo = lo - m.q;
        for (size_t j = 0; j < width; ++j) {
            const cd phase = (cfo_hz == 0.0) ? cd(1.0, 0.0) : std::polar(1.0, -omega * double(rel_lo + int64_t(j)));
            ref_avg[j] += cd(y[size_t(lo) + j]) * phase;
        }
        ++ref_valid;
    }
    BOOST_REQUIRE_GT(ref_valid, 0u);
    const int8_t* code = demod::GetPreambleCode(cfg.comm_code);
    double code_energy = 0;
    for (size_t c = 0; c < demod::kQm35CodeLength; ++c) code_energy += code[c] * code[c];
    for (size_t d = 0; d < taps; ++d) {
        cd acc{};
        for (size_t c = 0; c < demod::kQm35CodeLength; ++c) {
            if (code[c] != 0) acc += double(code[c]) * ref_avg[d + 8 * c];
        }
        const cd ref_tap = acc / (code_energy * ref_valid);
        BOOST_CHECK_LT(std::abs(cd(m.taps[d]) - ref_tap), 2e-5);
    }
}

BOOST_AUTO_TEST_CASE(test_discrete_cir_negative_cfo)
{
    Packet p(10, 32);
    const size_t q = 350;
    const size_t n = q + p.wave.size() + 512;
    const int64_t observed_end = static_cast<int64_t>(q + kTrainLen);

    const double cfo_hz = -35000.0;
    std::vector<cf> y(n, cf{});
    for (size_t i = 0; i < p.wave.size(); ++i) {
        const cd rot = std::polar(1.0, 2.0 * kPi * cfo_hz * double(i) / kFs);
        y[q + i] = cf(cd(p.wave[i]) * rot);
    }

    auto cfg = make_cfg(n, q, 40);
    cfg.ipatov_correlation = true;
    sic::DiscreteCirScratch s;
    std::string err;
    BOOST_REQUIRE_MESSAGE(sic::prepare_discrete_cir(cfg, templates_of(p), s, &err), err);

    sic::DiscreteCirModel m;
    BOOST_REQUIRE(sic::estimate_discrete_cir(
        y.data(), n, observed_end, templates_of(p), cfg, s, m));
    BOOST_REQUIRE(m.valid);
    BOOST_CHECK_EQUAL(m.support_source, "ipatov_correlation");
    BOOST_CHECK(m.impulse_template);
    BOOST_CHECK_LT(std::abs(m.cfo_hz - cfo_hz), 300.0);
    BOOST_CHECK_LE(m.fit_residual, cfg.fit_residual_max);

    // Explicit scalar reference calculation without vectorization/bypass
    const double omega = 2.0 * kPi * m.cfo_hz / kFs;
    const size_t taps = cfg.tap_count;
    const size_t width = kSps + taps - 1;
    std::vector<cd> ref_avg(width, cd{});
    size_t ref_valid = 0;
    for (size_t k = 1; k < cfg.fit_reps; ++k) {
        const int64_t lo = m.q + int64_t(k * kSps) + cfg.lag_lo;
        if (lo < 0 || lo + int64_t(width) > observed_end) continue;
        const int64_t rel_lo = lo - m.q;
        for (size_t j = 0; j < width; ++j) {
            const cd phase = std::polar(1.0, -omega * double(rel_lo + int64_t(j)));
            ref_avg[j] += cd(y[size_t(lo) + j]) * phase;
        }
        ++ref_valid;
    }
    BOOST_REQUIRE_GT(ref_valid, 0u);
    const int8_t* code = demod::GetPreambleCode(cfg.comm_code);
    double code_energy = 0;
    for (size_t c = 0; c < demod::kQm35CodeLength; ++c) code_energy += code[c] * code[c];
    for (size_t d = 0; d < taps; ++d) {
        cd acc{};
        for (size_t c = 0; c < demod::kQm35CodeLength; ++c) {
            if (code[c] != 0) acc += double(code[c]) * ref_avg[d + 8 * c];
        }
        const cd ref_tap = acc / (code_energy * ref_valid);
        BOOST_CHECK_LT(std::abs(cd(m.taps[d]) - ref_tap), 2e-5);
    }
}


// ===========================================================================
// Track B QA: autocorr prefilter attribution (B1) and the lazy / vectorised
// energy prefix (B2).
//
// Scenario convention (matches the SIC pipeline): the estimator is configured
// with the COMM template (code 10) and must locate the comm preamble, while
// `target_preamble_lo/hi` is the PUBLIC band of the sensing TARGET (code 9)
// whose own preamble must not be mistaken for outside interference.
//
// CONFIGURATION CAVEAT (read this before reading any "fast reject" result as a
// pipeline property): the reject can only fire when the caller SETS
// `target_preamble_lo/hi` AND leaves `autocorr_search_inside_target` false.
// The current Fixed-schedule pipeline path does NOT set the band
// (`slot->use_dynamic_slot == false`), so it takes the legacy `else` branch
// that puts every hit into `outside_ivs` and never rejects.  These cases
// therefore prove the ATTRIBUTION RULE, not that the production pipeline
// fast-rejects clean windows.
//
// The band the pipeline computes for a DYNAMIC slot (uwb_uhd_sc16_pipeline.cc)
// is
//   [seed - (search_margin + 1024),
//    seed + (sync_reps + sfd_syms)*sps + (search_margin + 3*sps))
// i.e. it covers the target preamble, its SFD and three extra symbols.  The
// "tight" band below is exactly that formula with search_margin = 0; the
// "loose" band widens both edges.  Nothing is tuned: autocorr_threshold stays
// at the calibrated 0.20 and no detection threshold is touched.
// ===========================================================================
namespace {

// Add `wave` at `q` (gain applied) on top of an existing window.
void place_wave(std::vector<cf>& y, const std::vector<cf>& wave, size_t q, float gain)
{
    for (size_t i = 0; i < wave.size() && q + i < y.size(); ++i)
        y[q + i] += gain * wave[i];
}

// Deterministic complex noise, `sigma` per component.
void add_noise(std::vector<cf>& y, float sigma, uint32_t seed)
{
    uint32_t rng = seed;
    for (size_t i = 0; i < y.size(); ++i) {
        rng = rng * 1664525U + 1013904223U;
        const float re = (static_cast<float>(rng >> 16) / 32768.0f - 1.0f) * sigma;
        rng = rng * 1664525U + 1013904223U;
        const float im = (static_cast<float>(rng >> 16) / 32768.0f - 1.0f) * sigma;
        y[i] += cf(re, im);
    }
}

// Wide-ROI config: the segmented prefilter only runs when
// roi_hi - roi_lo > 8*sps, so the ROI must be wide.
sic::DiscreteCirConfig make_wide_cfg(size_t n,
                                     int64_t roi_lo,
                                     int64_t roi_hi,
                                     int64_t t_lo,
                                     int64_t t_hi)
{
    sic::DiscreteCirConfig c = make_cfg(n, static_cast<size_t>(roi_lo), 40);
    c.roi_lo = roi_lo;
    c.roi_hi = roi_hi;
    c.target_preamble_lo = t_lo;
    c.target_preamble_hi = t_hi;
    c.autocorr_prefilter = true;
    c.autocorr_threshold = 0.20;
    c.autocorr_search_inside_target = false;
    return c;
}

struct ModelSnapshot {
    std::string status;
    std::string reason;
    bool valid = false;
    int64_t q = -1;
    int64_t lag_lo = 0;
    size_t tap_count = 0;
    size_t active_count = 0;
    std::vector<cf> taps;
};

ModelSnapshot snapshot_of(const sic::DiscreteCirModel& m)
{
    ModelSnapshot s;
    s.status = m.status ? m.status : "";
    s.reason = reason_of(m);
    s.valid = m.valid;
    s.q = m.q;
    s.lag_lo = m.lag_lo;
    s.tap_count = m.tap_count;
    s.active_count = m.active_count;
    s.taps.assign(m.taps.begin(), m.taps.begin() + m.tap_count);
    return s;
}

} // namespace

// ---------------------------------------------------------------------------
// B1 (a): a PURE CLEAN target packet (no comm interferer) must take the fast
// reject `no_outside_autocorr_preamble` on 100% of trials, at tight and loose
// public ROI margins and at more than one SNR.
//
// Before the fix the target's own padded hits always spilled into the
// "outside" intervals, so `outside_count != 0` and the reject never fired for
// the tight band (measured on the pre-change code: 12/24 overall — the loose
// band rejected, the tight band did not).  Those trials paid the eager prefix
// build plus the full-window cross-correlation before failing with
// `below_detect_threshold`.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_discrete_cir_clean_target_fast_reject)
{
    Packet p_comm(10, 32);      // estimator template: comm code 10
    Packet p_target(9, 16, 16); // clean sensing target: code 9, 16 SYNC

    constexpr size_t n = 160000;
    constexpr size_t kSfdSyms = 8; // "4z2" = 8 SFD symbols
    const size_t target_q = 12000;
    const int64_t preamble_end =
        static_cast<int64_t>(target_q) + 16 * static_cast<int64_t>(kSps);
    const int64_t band_body =
        preamble_end + static_cast<int64_t>(kSfdSyms) * static_cast<int64_t>(kSps);

    struct Band {
        int64_t lo;
        int64_t hi;
        const char* name;
    };
    const Band bands[] = {
        { static_cast<int64_t>(target_q) - 1024, band_body + 3 * static_cast<int64_t>(kSps), "tight" },
        { static_cast<int64_t>(target_q) - 4096, band_body + 8 * static_cast<int64_t>(kSps), "loose" }
    };
    // sigma is per-component AWGN on top of a 0.25-scaled packet (peak 0.2).
    const float sigmas[] = { 0.0f, 0.01f, 0.03f };

    size_t trials = 0;
    size_t rejected = 0;
    size_t prefix_untouched = 0;
    for (const Band& band : bands) {
        // One prepare per band; the config is identical across sigma/seed.
        sic::DiscreteCirConfig cfg =
            make_wide_cfg(n, 10000, 150000, band.lo, band.hi);
        sic::DiscreteCirScratch s;
        std::string err;
        BOOST_REQUIRE_MESSAGE(prepare_ok(p_comm, cfg, s, err), err);
        for (float sigma : sigmas) {
            for (uint32_t seed = 1; seed <= 4; ++seed) {
                std::vector<cf> y(n, cf(0.f, 0.f));
                place_wave(y, p_target.wave, target_q, 0.25f);
                if (sigma > 0.0f)
                    add_noise(y, sigma, seed * 7919u + 13u);

                // Poison the prefix: a fast reject must NOT build it (B2).
                std::fill(s.energy_prefix.begin(), s.energy_prefix.end(), -12345.0);

                sic::DiscreteCirModel m;
                estimate_ok(p_comm, cfg, y, static_cast<int64_t>(n), s, m);
                ++trials;
                const bool took_reject =
                    !m.valid && reason_of(m) == "no_outside_autocorr_preamble";
                if (took_reject)
                    ++rejected;
                if (s.energy_prefix[1] == -12345.0)
                    ++prefix_untouched;
                BOOST_TEST_MESSAGE("band=" << band.name << " sigma=" << sigma
                                           << " seed=" << seed << " status="
                                           << (m.status ? m.status : "")
                                           << " reason=" << reason_of(m));
            }
        }
    }
    BOOST_TEST_MESSAGE("clean-target fast reject " << rejected << "/" << trials);
    BOOST_CHECK_EQUAL(rejected, trials);
    // The lazy prefix must not be built on a rejected window.
    BOOST_CHECK_EQUAL(prefix_untouched, trials);

    // INDICATIVE ONLY (no assertion, not a benchmark): cost of the reject path
    // vs the pre-change path on the same clean window.  The coordinator owns
    // all performance measurement; this only shows the direction.
    {
        std::vector<cf> y(n, cf(0.f, 0.f));
        place_wave(y, p_target.wave, target_q, 0.25f);
        sic::DiscreteCirConfig cfg_on =
            make_wide_cfg(n, 10000, 150000, bands[0].lo, bands[0].hi);
        sic::DiscreteCirConfig cfg_off = cfg_on;
        cfg_off.autocorr_prefilter = false;
        sic::DiscreteCirScratch s_on;
        sic::DiscreteCirScratch s_off;
        std::string err;
        BOOST_REQUIRE_MESSAGE(prepare_ok(p_comm, cfg_on, s_on, err), err);
        BOOST_REQUIRE_MESSAGE(prepare_ok(p_comm, cfg_off, s_off, err), err);
        constexpr int kIters = 5;
        const auto t_on0 = std::chrono::steady_clock::now();
        for (int i = 0; i < kIters; ++i) {
            sic::DiscreteCirModel m;
            estimate_ok(p_comm, cfg_on, y, static_cast<int64_t>(n), s_on, m);
        }
        const double on_us = std::chrono::duration<double, std::micro>(
                                 std::chrono::steady_clock::now() - t_on0)
                                 .count() /
                             kIters;
        const auto t_off0 = std::chrono::steady_clock::now();
        for (int i = 0; i < kIters; ++i) {
            sic::DiscreteCirModel m;
            estimate_ok(p_comm, cfg_off, y, static_cast<int64_t>(n), s_off, m);
        }
        const double off_us = std::chrono::duration<double, std::micro>(
                                  std::chrono::steady_clock::now() - t_off0)
                                  .count() /
                              kIters;
        BOOST_TEST_MESSAGE("INDICATIVE (n=" << n << ", roi=140k): reject path "
                                            << on_us << " us, pre-change path "
                                            << off_us << " us");
    }

    // DOCUMENTED BOUNDARY (unchanged by this fix): if the public band stops at
    // the PREAMBLE end, the target's own SFD (8 "4z2" symbols with repeated
    // -1 runs) is genuine periodic content beyond the band, so neither the old
    // rule (`outside_count`) nor the new rule (`outside_evidence`) can reject.
    // The pipeline never uses such a band; it always covers preamble + SFD.
    {
        std::vector<cf> y(n, cf(0.f, 0.f));
        place_wave(y, p_target.wave, target_q, 0.25f);
        sic::DiscreteCirConfig cfg =
            make_wide_cfg(n, 10000, 150000,
                          static_cast<int64_t>(target_q) - 64, preamble_end + 64);
        sic::DiscreteCirScratch s;
        std::string err;
        BOOST_REQUIRE_MESSAGE(prepare_ok(p_comm, cfg, s, err), err);
        sic::DiscreteCirModel m;
        estimate_ok(p_comm, cfg, y, static_cast<int64_t>(n), s, m);
        BOOST_TEST_MESSAGE("preamble-only band: reason=" << reason_of(m));
        BOOST_CHECK_NE(reason_of(m), "no_outside_autocorr_preamble");
    }
}

// ---------------------------------------------------------------------------
// B1 (b): an interferer fully LEFT, fully RIGHT, or PARTIALLY OVERLAPPING the
// target preamble by 20/50/80% must NEVER be fast-rejected, and the resulting
// model must be identical to the pre-change path (`autocorr_prefilter=false`).
//
// The control below proves the test is not vacuous: the SAME window without
// the interferer does take the fast reject, so the interferer is exactly what
// flips the decision.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_discrete_cir_outside_evidence_never_misses_interferer)
{
    Packet p_comm(10, 32);      // interferer: 64 SYNC = 65024 samples
    Packet p_target(9, 16, 16); // target: 16 SYNC = 16256 samples

    constexpr size_t n = 200000;
    constexpr size_t kSfdSyms = 8;
    const size_t target_q = 80000;
    const int64_t preamble_end =
        static_cast<int64_t>(target_q) + 16 * static_cast<int64_t>(kSps);
    const int64_t t_lo = static_cast<int64_t>(target_q) - 1024;
    const int64_t t_hi = preamble_end +
                         static_cast<int64_t>(kSfdSyms) * static_cast<int64_t>(kSps) +
                         3 * static_cast<int64_t>(kSps);
    const size_t int_len = 64 * kSps;
    const size_t left_edge = target_q - int_len; // interferer end == target_q

    struct Case {
        const char* name;
        size_t int_q;
    };
    const Case cases[] = {
        { "left", 6000 },
        { "right", 105000 },
        { "overlap20", left_edge + (16 * kSps * 20) / 100 },
        { "overlap50", left_edge + (16 * kSps * 50) / 100 },
        { "overlap80", left_edge + (16 * kSps * 80) / 100 }
    };

    // Control: the clean target alone must be rejected with this exact band.
    {
        std::vector<cf> y(n, cf(0.f, 0.f));
        place_wave(y, p_target.wave, target_q, 0.25f);
        sic::DiscreteCirConfig cfg = make_wide_cfg(n, 0, 198000, t_lo, t_hi);
        sic::DiscreteCirScratch s;
        std::string err;
        BOOST_REQUIRE_MESSAGE(prepare_ok(p_comm, cfg, s, err), err);
        sic::DiscreteCirModel m;
        estimate_ok(p_comm, cfg, y, static_cast<int64_t>(n), s, m);
        BOOST_TEST_MESSAGE("control (target only): reason=" << reason_of(m));
        BOOST_CHECK_EQUAL(reason_of(m), "no_outside_autocorr_preamble");
    }

    size_t missed = 0;
    size_t equivalent = 0;
    for (const Case& c : cases) {
        std::vector<cf> y(n, cf(0.f, 0.f));
        place_wave(y, p_target.wave, target_q, 0.25f);
        place_wave(y, p_comm.wave, c.int_q, 1.0f);

        sic::DiscreteCirConfig cfg_on = make_wide_cfg(n, 0, 198000, t_lo, t_hi);
        sic::DiscreteCirConfig cfg_off = cfg_on;
        cfg_off.autocorr_prefilter = false;

        sic::DiscreteCirScratch s_on;
        sic::DiscreteCirScratch s_off;
        std::string err;
        BOOST_REQUIRE_MESSAGE(prepare_ok(p_comm, cfg_on, s_on, err), err);
        BOOST_REQUIRE_MESSAGE(prepare_ok(p_comm, cfg_off, s_off, err), err);

        sic::DiscreteCirModel m_on;
        sic::DiscreteCirModel m_off;
        estimate_ok(p_comm, cfg_on, y, static_cast<int64_t>(n), s_on, m_on);
        estimate_ok(p_comm, cfg_off, y, static_cast<int64_t>(n), s_off, m_off);

        BOOST_TEST_MESSAGE(c.name << ": on status="
                                   << (m_on.status ? m_on.status : "")
                                   << " reason=" << reason_of(m_on)
                                   << " q=" << m_on.q << " valid=" << m_on.valid
                                   << " | off status="
                                   << (m_off.status ? m_off.status : "")
                                   << " reason=" << reason_of(m_off)
                                   << " q=" << m_off.q
                                   << " valid=" << m_off.valid);

        // (1) never fast-rejected
        if (reason_of(m_on) == "no_outside_autocorr_preamble") {
            ++missed;
            BOOST_ERROR(c.name << " was fast-rejected");
            continue;
        }
        // (2) the interferer is actually detected
        BOOST_CHECK_MESSAGE(m_on.valid,
                            c.name << " not detected: " << reason_of(m_on));
        BOOST_CHECK_LE(std::abs(m_on.q - static_cast<int64_t>(c.int_q)), 8);
        BOOST_CHECK(m_off.valid);

        // (3) equivalent to the pre-change path
        const ModelSnapshot a = snapshot_of(m_on);
        const ModelSnapshot b = snapshot_of(m_off);
        bool same = a.status == b.status && a.reason == b.reason &&
                    a.valid == b.valid && a.q == b.q && a.lag_lo == b.lag_lo &&
                    a.tap_count == b.tap_count && a.active_count == b.active_count &&
                    a.taps.size() == b.taps.size();
        if (same) {
            // Exact comparison: both runs are deterministic and must produce
            // bit-identical dense taps.
            for (size_t i = 0; i < a.taps.size(); ++i) {
                if (a.taps[i] != b.taps[i]) {
                    same = false;
                    break;
                }
            }
        }
        if (same)
            ++equivalent;
        BOOST_CHECK_MESSAGE(same, c.name << " differs from the prefilter-off path:"
                                         << " q " << a.q << "/" << b.q
                                         << " taps " << a.tap_count << "/"
                                         << b.tap_count << " active "
                                         << a.active_count << "/" << b.active_count);
    }
    BOOST_TEST_MESSAGE("interferer cases: missed=" << missed
                                                   << " equivalent=" << equivalent
                                                   << "/" << (sizeof(cases) / sizeof(cases[0])));
    BOOST_CHECK_EQUAL(missed, 0u);
    BOOST_CHECK_EQUAL(equivalent, sizeof(cases) / sizeof(cases[0]));
}

// ---------------------------------------------------------------------------
// B1 (c): IN-BAND INTERFERER.  An interferer whose entire periodic region lies
// inside the attribution band cannot be told apart from the target by a
// lag-sps autocorrelation alone.
//
// Measured difference against the pre-change rule (do not read this as "no
// behaviour change"): the OLD rule did NOT reject this window (the target's
// own preamble hit one segment before t_lo spilled into `outside_ivs`, so
// `outside_count != 0`) and it ended in `below_detect_threshold`; the NEW rule
// rejects it as `no_outside_autocorr_preamble`.
//
// The OUTCOME is nevertheless unchanged, and that is asserted below: with a
// band set and `autocorr_search_inside_target == false` the cross-correlation
// only scans `outside_ivs`, which excludes the band, so an in-band interferer
// was never detected in this configuration either.  The prefilter only avoids
// the wasted scan and reports a more accurate reason.  Callers that must catch
// an in-band (co-located) interferer have to opt in with
// `autocorr_search_inside_target`, which is exercised at the end.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_discrete_cir_inside_band_interferer_is_documented_reject)
{
    Packet p_comm(10, 32, 8);   // 8 SYNC = 8128 samples: fits inside the band
    Packet p_target(9, 16, 16);

    constexpr size_t n = 200000;
    constexpr size_t kSfdSyms = 8;
    const size_t target_q = 80000;
    const int64_t preamble_end =
        static_cast<int64_t>(target_q) + 16 * static_cast<int64_t>(kSps);
    const int64_t t_lo = static_cast<int64_t>(target_q) - 1024;
    const int64_t t_hi = preamble_end +
                         static_cast<int64_t>(kSfdSyms) * static_cast<int64_t>(kSps) +
                         3 * static_cast<int64_t>(kSps);
    const size_t int_q = target_q + 2000; // entirely inside [t_lo, t_hi)

    std::vector<cf> y(n, cf(0.f, 0.f));
    place_wave(y, p_target.wave, target_q, 0.25f);
    place_wave(y, p_comm.wave, int_q, 1.0f);

    sic::DiscreteCirConfig cfg = make_wide_cfg(n, 0, 198000, t_lo, t_hi);
    sic::DiscreteCirScratch s;
    std::string err;
    BOOST_REQUIRE_MESSAGE(prepare_ok(p_comm, cfg, s, err), err);
    sic::DiscreteCirModel m;
    estimate_ok(p_comm, cfg, y, static_cast<int64_t>(n), s, m);
    BOOST_TEST_MESSAGE("in-band interferer: reason=" << reason_of(m));
    BOOST_CHECK_EQUAL(reason_of(m), "no_outside_autocorr_preamble");

    // Outcome equivalence with the pre-change search path: turning the
    // prefilter off (the old code path) must ALSO fail to detect, because the
    // search excludes the band.  This is what makes the reject safe here.
    sic::DiscreteCirConfig cfg_off = cfg;
    cfg_off.autocorr_prefilter = false;
    sic::DiscreteCirScratch s_off;
    BOOST_REQUIRE_MESSAGE(prepare_ok(p_comm, cfg_off, s_off, err), err);
    sic::DiscreteCirModel m_off;
    estimate_ok(p_comm, cfg_off, y, static_cast<int64_t>(n), s_off, m_off);
    BOOST_TEST_MESSAGE("in-band interferer, prefilter off: status="
                       << (m_off.status ? m_off.status : "")
                       << " reason=" << reason_of(m_off) << " q=" << m_off.q);
    BOOST_CHECK(!m_off.valid);

    // Explicit opt-in recovers the in-band interferer.
    sic::DiscreteCirConfig cfg_in = cfg;
    cfg_in.autocorr_search_inside_target = true;
    sic::DiscreteCirScratch s_in;
    BOOST_REQUIRE_MESSAGE(prepare_ok(p_comm, cfg_in, s_in, err), err);
    sic::DiscreteCirModel m_in;
    estimate_ok(p_comm, cfg_in, y, static_cast<int64_t>(n), s_in, m_in);
    BOOST_TEST_MESSAGE("in-band opt-in: status="
                       << (m_in.status ? m_in.status : "")
                       << " reason=" << reason_of(m_in) << " q=" << m_in.q);
    BOOST_CHECK_MESSAGE(m_in.valid, "in-band opt-in: " << reason_of(m_in));
    BOOST_CHECK_LE(std::abs(m_in.q - static_cast<int64_t>(int_q)), 8);

    // Plan §4.3 "Mode A" red line: a FULL-LENGTH code-10 interferer preamble
    // sitting exactly ON TOP of a full-length code-9 target preamble must
    // still be detected when the caller opts in.  The reject cannot fire at
    // all with `autocorr_search_inside_target == true`, so this is the
    // supported configuration for the co-located case.
    {
        Packet p_comm64(10, 32);      // 64 SYNC
        Packet p_target64(9, 16, 64); // 64 SYNC
        const size_t co_q = 40000;
        std::vector<cf> yc(n, cf(0.f, 0.f));
        place_wave(yc, p_target64.wave, co_q, 0.25f);
        place_wave(yc, p_comm64.wave, co_q, 1.0f);

        const int64_t co_lo = static_cast<int64_t>(co_q) - 1024;
        const int64_t co_hi =
            static_cast<int64_t>(co_q) +
            (64 + static_cast<int64_t>(kSfdSyms)) * static_cast<int64_t>(kSps) +
            3 * static_cast<int64_t>(kSps);
        sic::DiscreteCirConfig cfg_co = make_wide_cfg(n, 0, 198000, co_lo, co_hi);
        cfg_co.autocorr_search_inside_target = true;
        sic::DiscreteCirScratch s_co;
        BOOST_REQUIRE_MESSAGE(prepare_ok(p_comm64, cfg_co, s_co, err), err);
        sic::DiscreteCirModel m_co;
        estimate_ok(p_comm64, cfg_co, yc, static_cast<int64_t>(n), s_co, m_co);
        BOOST_TEST_MESSAGE("co-located 64-SYNC opt-in: status="
                           << (m_co.status ? m_co.status : "")
                           << " reason=" << reason_of(m_co) << " q=" << m_co.q);
        BOOST_CHECK_MESSAGE(m_co.valid, "co-located opt-in: " << reason_of(m_co));
        BOOST_CHECK_LE(std::abs(m_co.q - static_cast<int64_t>(co_q)), 8);
    }
}

// ---------------------------------------------------------------------------
// B2: the AVX2 prefix must match an independent scalar DOUBLE reference
// point-by-point over >= 5e5 samples.  The window carries a large-amplitude
// packet (prefix ~5e9) followed by a tiny tail (|v|^2 ~ 1e-4): a float
// accumulator has ~512 ulp at 5e9 and would erase the tail entirely, so this
// case also proves the double accumulation.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_discrete_cir_energy_prefix_avx2_matches_scalar)
{
    Packet p_comm(10, 32);
    constexpr size_t n = 560000;
    const size_t q = 1000;

    std::vector<cf> y(n, cf(0.f, 0.f));
    place_wave(y, p_comm.wave, q, 300.0f);
    uint32_t rng = 0x1234567u;
    for (size_t i = 100000; i < n; ++i) {
        rng = rng * 1664525u + 1013904223u;
        const float re = (static_cast<float>(rng >> 16) / 32768.0f - 1.0f) * 0.003f;
        rng = rng * 1664525u + 1013904223u;
        const float im = (static_cast<float>(rng >> 16) / 32768.0f - 1.0f) * 0.003f;
        y[i] += cf(re, im);
    }

    // target band disabled -> the prefilter never rejects, so the lazy prefix
    // is built and can be inspected.
    sic::DiscreteCirConfig cfg = make_wide_cfg(n, 0, 544000, -1, -1);
    sic::DiscreteCirScratch s;
    std::string err;
    BOOST_REQUIRE_MESSAGE(prepare_ok(p_comm, cfg, s, err), err);
    sic::DiscreteCirModel m;
    const bool ok = estimate_ok(p_comm, cfg, y, static_cast<int64_t>(n), s, m);
    BOOST_REQUIRE_MESSAGE(ok, "status=" << (m.status ? m.status : "")
                                        << " reason=" << reason_of(m));

    // Reproduce the estimator's prefix window [energy_base, energy_end).
    const int64_t visible_end = static_cast<int64_t>(n);
    const int64_t roi_hi_clipped =
        std::min<int64_t>(cfg.roi_hi, visible_end - static_cast<int64_t>(kSps) + 1);
    const int64_t energy_base = cfg.roi_lo;
    const int64_t energy_end = std::max<int64_t>(
        energy_base,
        std::min<int64_t>(visible_end,
                          roi_hi_clipped +
                              static_cast<int64_t>(kReps - 1) * static_cast<int64_t>(kSps) +
                              static_cast<int64_t>(kSps)));
    const size_t len = static_cast<size_t>(energy_end - energy_base);
    BOOST_REQUIRE(len >= 500000u);
    BOOST_REQUIRE(s.energy_prefix.size() > len);

    // Independent scalar reference: the compiled fallback kernel's algorithm.
    std::vector<double> ref(len + 1, 0.0);
    double acc = 0.0;
    for (size_t i = 0; i < len; ++i) {
        const cf v = y[static_cast<size_t>(energy_base) + i];
        acc += static_cast<double>(v.real()) * static_cast<double>(v.real()) +
               static_cast<double>(v.imag()) * static_cast<double>(v.imag());
        ref[i + 1] = acc;
    }

    double max_rel = 0.0;
    size_t monotone_breaks = 0;
    for (size_t i = 0; i <= len; ++i) {
        BOOST_REQUIRE(std::isfinite(s.energy_prefix[i]));
        const double d = std::abs(s.energy_prefix[i] - ref[i]);
        const double denom = std::max(1.0, std::abs(ref[i]));
        max_rel = std::max(max_rel, d / denom);
        if (i > 0 && s.energy_prefix[i] < s.energy_prefix[i - 1])
            ++monotone_breaks;
    }
    BOOST_CHECK_EQUAL(monotone_breaks, 0u);
    BOOST_CHECK_LT(max_rel, 1e-7);

    // The tiny tail must survive the large prefix.  The packet ends near
    // sample 250k, so [300k, len) is pure tiny noise.  A float accumulator
    // would quantise this contribution away completely (it is far below one
    // float ulp of the total, see the sensitivity check below); the double
    // accumulator keeps it, and both kernels must agree on it.
    const size_t tail_lo = 300000;
    const double sim_delta = s.energy_prefix[len] - s.energy_prefix[tail_lo];
    const double ref_delta = ref[len] - ref[tail_lo];
    BOOST_REQUIRE_GT(ref_delta, 0.0);
    BOOST_CHECK_GT(sim_delta, 0.99 * ref_delta);
    BOOST_CHECK_LT(sim_delta, 1.01 * ref_delta);
    // Sensitivity of the check above: the tail is far below one float ulp of
    // the accumulated total (~6e-8 relative), so a float accumulator would
    // lose it completely.
    BOOST_CHECK_LT(ref_delta / ref[len], 1e-8);

    BOOST_TEST_MESSAGE("energy prefix: len=" << len << " max_rel=" << max_rel
                                             << " tail_delta=" << ref_delta
                                             << " tail_sim=" << sim_delta
                                             << " tail/total=" << ref_delta / ref[len]);
}
