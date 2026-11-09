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
