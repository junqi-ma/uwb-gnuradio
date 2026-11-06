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

    explicit Packet(size_t code = 10, size_t psdu_len = 32)
    {
        profile.code_index = code;
        profile.sync_repetitions = 64;
        profile.sfd_mode = "4z2";
        profile.peak_amplitude = 0.8f;
        std::vector<uint8_t> psdu = mod::make_random_psdu(
            psdu_len - 2, static_cast<uint32_t>(1000 + code), true);
        const size_t cap = mod::packet_samples_998p4(
                               64, 8, psdu.size(), false, &profile.pulse) +
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
