/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * QA for the shared SYNC-CIR kernel (uwb_sync_cir_core.h, contract §1 of
 * docs/performance/接口契约_统一SYNC_CIR核与公共前导前端_2026-11-12.md).
 *
 * Everything is synthesised from GetPreambleCode() / BuildSampledCode(): no
 * testdata files are needed.  The kernel is checked against
 *   * an independent float64 reference (double accumulation, double
 *     correlation, double normalisation, relative L2 <= 2e-5, identical peak),
 *   * a local transcription of the legacy comm `stage_cir_softchips` float
 *     sequence (memcmp-exact `norm`),
 *   * structural properties: raw amplitude/phase preservation, unit-L2 norm,
 *     integer delay axis, clip rule, required_input_range envelope, and every
 *     fail-closed path (tap_count == 0, taps zeroed).
 */

#ifndef BOOST_TEST_MODULE
#define BOOST_TEST_MODULE qa_uwb_sync_cir_core
#endif
// GR_ADD_CPP_TEST already passes -DBOOST_TEST_DYN_LINK; the guard keeps a plain
// standalone `g++ qa_uwb_sync_cir_core.cc -lboost_unit_test_framework` working
// (Boost.Test only emits main() for the dyn-link variant).
#ifndef BOOST_TEST_DYN_LINK
#define BOOST_TEST_DYN_LINK
#endif

#include <boost/test/unit_test.hpp>

#include <gnuradio/uwb/uwb_phy_profile.h>
#include <gnuradio/uwb/uwb_rx_cfo_opt.h>
#include <gnuradio/uwb/uwb_rx_sync_opt.h>
#include <gnuradio/uwb/uwb_sync_cir_core.h>

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

namespace {

using gr::uwb::demod::BuildSampledCode;
using gr::uwb::demod::GetPreambleCode;
using gr::uwb::demod::kQm35CodeLength;
using gr::uwb::demod::kQm35ChipsPerSymbol;
using gr::uwb::demod::rxopt::build_sparse_cir_template;
using gr::uwb::demod::rxopt::cir_template_key;
using gr::uwb::demod::rxopt::correlate_cir_sparse;
using gr::uwb::demod::rxopt::SparseCirTemplate;
using gr::uwb::demod::rxopt::vector_accumulate_complex;
using gr::uwb::demod::rxopt::vector_scale_complex;
using gr::uwb::demod::sync_cir::estimate_sync_cir;
using gr::uwb::demod::sync_cir::kSyncCirCodeLength;
using gr::uwb::demod::sync_cir::prepare_sync_code;
using gr::uwb::demod::sync_cir::PreparedSyncCode;
using gr::uwb::demod::sync_cir::required_input_range;
using gr::uwb::demod::sync_cir::SyncCirRequest;
using gr::uwb::demod::sync_cir::SyncCirResult;
using gr::uwb::demod::sync_cir::SyncCirScratch;
using gr::uwb::demod::sync_cir::SyncCirStatus;

constexpr double kRelL2Tolerance = 2e-5;

// ---------------------------------------------------------------------------
// Synthetic signal construction
// ---------------------------------------------------------------------------

// The sampled ternary code as an RX waveform: spread[j] at sample 2*j, exact
// zeros in between.  This is the waveform the CIR correlation is matched to,
// so a clean single-path input produces a peak at tap == pre (delay 0).
std::vector<std::complex<float>> sampled_code_wave(size_t code_index)
{
    const std::vector<int8_t> spread =
        BuildSampledCode(GetPreambleCode(code_index), kQm35CodeLength);
    BOOST_REQUIRE_EQUAL(spread.size(), kQm35ChipsPerSymbol);
    std::vector<std::complex<float>> wave(2 * spread.size(),
                                          std::complex<float>(0.f, 0.f));
    for (size_t j = 0; j < spread.size(); ++j)
        wave[2 * j] = std::complex<float>(static_cast<float>(spread[j]), 0.f);
    return wave;
}

struct Packet {
    std::vector<std::complex<float>> rx;
    int64_t origin = 0;
    size_t total_repetitions = 0;
    double period = 0.0;
};

// Places `total` copies of the sampled code on the repetition grid
// origin + llround(k*P) + delay.  The tail keeps every window inside the buffer
// for post <= tail.
Packet make_packet(size_t code_index,
                   size_t total,
                   double period,
                   int64_t origin,
                   int64_t delay = 0,
                   int64_t tail = 256)
{
    BOOST_REQUIRE(origin >= 0);
    BOOST_REQUIRE(total > 0);
    const std::vector<std::complex<float>> wave = sampled_code_wave(code_index);
    const int64_t last_rs =
        origin + static_cast<int64_t>(
                     std::llround(static_cast<double>(total - 1) * period));
    Packet p;
    p.origin = origin;
    p.total_repetitions = total;
    p.period = period;
    p.rx.assign(static_cast<size_t>(last_rs + delay +
                                    static_cast<int64_t>(wave.size()) + tail),
                std::complex<float>(0.f, 0.f));
    for (size_t k = 0; k < total; ++k) {
        const int64_t rs =
            origin + static_cast<int64_t>(
                         std::llround(static_cast<double>(k) * period)) + delay;
        BOOST_REQUIRE(rs >= 0);
        BOOST_REQUIRE(static_cast<size_t>(rs) + wave.size() <= p.rx.size());
        for (size_t i = 0; i < wave.size(); ++i)
            p.rx[static_cast<size_t>(rs) + i] += wave[i];
    }
    return p;
}

// Dense deterministic pseudo-random IQ: every tap has energy, so amplitude and
// phase relations can be checked on all of them.
struct Lcg {
    uint64_t state;
    explicit Lcg(uint64_t seed) : state(seed) {}
    uint32_t next()
    {
        state = state * 6364136223846793005ULL + 1442695040888963407ULL;
        return static_cast<uint32_t>(state >> 33);
    }
    float unit() { return static_cast<float>(next() % 2001) / 1000.0f - 1.0f; }
};

std::vector<std::complex<float>> random_packet(size_t n, uint64_t seed)
{
    Lcg rng(seed);
    std::vector<std::complex<float>> rx(n);
    for (size_t i = 0; i < n; ++i)
        rx[i] = std::complex<float>(rng.unit(), rng.unit());
    return rx;
}

SyncCirRequest make_req(int64_t origin,
                        double period,
                        size_t total,
                        size_t skip,
                        size_t max_reps,
                        size_t pre,
                        size_t post)
{
    SyncCirRequest req;
    req.cir_axis_origin = origin;
    req.samples_per_symbol = period;
    req.total_repetitions = total;
    req.skip_initial = skip;
    req.max_repetitions = max_reps;
    req.pre = pre;
    req.post = post;
    return req;
}

std::string label(size_t code,
                  size_t pre,
                  size_t post,
                  size_t total,
                  double period,
                  int64_t origin)
{
    std::ostringstream oss;
    oss << "code=" << code << " pre=" << pre << " post=" << post
        << " total=" << total << " P=" << period << " origin=" << origin;
    return oss.str();
}

// ---------------------------------------------------------------------------
// Independent references
// ---------------------------------------------------------------------------

// Documented clip rule, implemented independently of the kernel.
std::vector<int64_t> accepted_windows(const SyncCirRequest& req, size_t n)
{
    const size_t tap_count = req.pre + req.post;
    const size_t wlen = kSyncCirCodeLength + tap_count - 1;
    size_t count = req.total_repetitions - req.skip_initial;
    if (req.max_repetitions > 0)
        count = std::min(count, req.max_repetitions);
    std::vector<int64_t> accepted;
    for (size_t k = req.skip_initial; k < req.skip_initial + count; ++k) {
        const int64_t rs =
            req.cir_axis_origin +
            static_cast<int64_t>(
                std::llround(static_cast<double>(k) * req.samples_per_symbol));
        const int64_t lo = rs - static_cast<int64_t>(req.pre);
        const int64_t hi = lo + static_cast<int64_t>(wlen);
        if (lo < 0 || hi > static_cast<int64_t>(n))
            continue;
        accepted.push_back(lo);
    }
    return accepted;
}

struct Float64Reference {
    bool ok = false;
    size_t valid = 0;
    size_t peak_tap = 0;
    double peak_abs = 0.0;
    double nrm = 0.0;
    int64_t first_lo = -1;
    int64_t last_lo = -1;
    std::vector<std::complex<double>> raw;
    std::vector<std::complex<double>> norm;
};

// Fully independent float64 model: double accumulation, double correlation with
// the sampled code, double normalisation.
Float64Reference float64_reference(const std::complex<float>* rx,
                                   size_t n,
                                   const SyncCirRequest& req,
                                   size_t code_index)
{
    Float64Reference ref;
    const size_t code_len = kSyncCirCodeLength;
    const size_t tap_count = req.pre + req.post;
    const size_t wlen = code_len + tap_count - 1;

    const std::vector<int8_t> spread =
        BuildSampledCode(GetPreambleCode(code_index), kQm35CodeLength);
    std::vector<double> code(code_len, 0.0);
    double energy = 0.0;
    for (size_t j = 0; j < spread.size(); ++j) {
        code[2 * j] = static_cast<double>(spread[j]);
        energy += static_cast<double>(spread[j]) * static_cast<double>(spread[j]);
    }
    BOOST_REQUIRE(energy > 0.0);

    const std::vector<int64_t> accepted = accepted_windows(req, n);
    ref.valid = accepted.size();
    if (accepted.empty())
        return ref;
    ref.first_lo = accepted.front();
    ref.last_lo = accepted.back();

    std::vector<std::complex<double>> avg(wlen, std::complex<double>(0.0, 0.0));
    for (int64_t lo : accepted) {
        for (size_t m = 0; m < wlen; ++m) {
            const std::complex<float> v = rx[static_cast<size_t>(lo) + m];
            avg[m] += std::complex<double>(static_cast<double>(v.real()),
                                          static_cast<double>(v.imag()));
        }
    }
    for (size_t m = 0; m < wlen; ++m)
        avg[m] /= static_cast<double>(ref.valid);

    ref.raw.assign(tap_count, std::complex<double>(0.0, 0.0));
    for (size_t nn = 0; nn < tap_count; ++nn) {
        std::complex<double> acc(0.0, 0.0);
        for (size_t m = 0; m < code_len; ++m) {
            if (code[m] == 0.0)
                continue;
            acc += avg[nn + m] * std::conj(std::complex<double>(code[m], 0.0));
        }
        ref.raw[nn] = acc / energy;
    }

    double nrm2 = 0.0;
    for (size_t i = 0; i < tap_count; ++i)
        nrm2 += std::norm(ref.raw[i]);
    ref.nrm = std::sqrt(nrm2);
    if (!(ref.nrm > 0.0))
        return ref;

    ref.norm.assign(tap_count, std::complex<double>(0.0, 0.0));
    size_t peak = 0;
    double peak_abs = -1.0;
    for (size_t i = 0; i < tap_count; ++i) {
        ref.norm[i] = ref.raw[i] / ref.nrm;
        const double mag = std::abs(ref.raw[i]);
        if (mag > peak_abs) {
            peak_abs = mag;
            peak = i;
        }
    }
    ref.peak_tap = peak;
    ref.peak_abs = peak_abs;
    ref.ok = true;
    return ref;
}

// Local transcription of the legacy comm `stage_cir_softchips` float sequence:
// zero the window, vector accumulate, float 1/valid scale, sparse correlate
// (float code_energy divide), float L2 norm, /(nrm + 1e-12f).
std::vector<std::complex<float>> comm_stage_cir_reference(
    const std::complex<float>* rx,
    size_t n,
    const SyncCirRequest& req,
    const SparseCirTemplate& tmpl)
{
    const size_t code_len = kSyncCirCodeLength;
    const size_t tap_count = req.pre + req.post;
    const size_t wlen = code_len + tap_count - 1;

    std::vector<std::complex<float>> corr(wlen, std::complex<float>(0.f, 0.f));
    size_t valid = 0;
    for (int64_t lo : accepted_windows(req, n)) {
        vector_accumulate_complex(corr.data(), rx + static_cast<size_t>(lo), wlen);
        ++valid;
    }
    BOOST_REQUIRE(valid > 0);
    const float inv = 1.0f / static_cast<float>(valid);
    vector_scale_complex(corr.data(), inv, wlen);

    std::vector<std::complex<float>> values;
    correlate_cir_sparse(corr.data(), tmpl, tap_count, values);
    float nrm = 0.0f;
    for (size_t i = 0; i < tap_count; ++i)
        nrm += std::norm(values[i]);
    nrm = std::sqrt(nrm);
    BOOST_REQUIRE(nrm > 0.0f);
    for (size_t i = 0; i < tap_count; ++i)
        values[i] /= (nrm + 1e-12f);
    return values;
}

double rel_l2(const std::complex<float>* a,
              size_t n,
              const std::vector<std::complex<double>>& b)
{
    BOOST_REQUIRE_EQUAL(b.size(), n);
    double num = 0.0;
    double den = 0.0;
    for (size_t i = 0; i < n; ++i) {
        const std::complex<double> d =
            std::complex<double>(static_cast<double>(a[i].real()),
                                 static_cast<double>(a[i].imag())) -
            b[i];
        num += std::norm(d);
        den += std::norm(b[i]);
    }
    if (den == 0.0)
        return num == 0.0 ? 0.0 : std::numeric_limits<double>::infinity();
    return std::sqrt(num / den);
}

double rel_l2(const std::complex<float>* a,
              const std::complex<float>* b,
              size_t n)
{
    double num = 0.0;
    double den = 0.0;
    for (size_t i = 0; i < n; ++i) {
        const std::complex<double> d =
            std::complex<double>(static_cast<double>(a[i].real()),
                                 static_cast<double>(a[i].imag())) -
            std::complex<double>(static_cast<double>(b[i].real()),
                                 static_cast<double>(b[i].imag()));
        num += std::norm(d);
        den += std::norm(std::complex<double>(static_cast<double>(b[i].real()),
                                              static_cast<double>(b[i].imag())));
    }
    if (den == 0.0)
        return num == 0.0 ? 0.0 : std::numeric_limits<double>::infinity();
    return std::sqrt(num / den);
}

double l2_norm_sq(const std::complex<float>* a, size_t n)
{
    double acc = 0.0;
    for (size_t i = 0; i < n; ++i)
        acc += std::norm(std::complex<double>(static_cast<double>(a[i].real()),
                                             static_cast<double>(a[i].imag())));
    return acc;
}

void check_taps_zeroed(const SyncCirScratch& scratch)
{
    for (const std::complex<float>& v : scratch.raw)
        BOOST_CHECK_EQUAL(v, std::complex<float>(0.f, 0.f));
    for (const std::complex<float>& v : scratch.norm)
        BOOST_CHECK_EQUAL(v, std::complex<float>(0.f, 0.f));
}

void check_failed_result_zeroed(const SyncCirResult& out)
{
    BOOST_CHECK_EQUAL(out.tap_count, size_t(0));
    BOOST_CHECK_EQUAL(out.peak_tap, size_t(0));
    BOOST_CHECK_EQUAL(out.peak_abs, 0.f);
    BOOST_CHECK_EQUAL(out.raw_l2_norm, 0.f);
    BOOST_CHECK_EQUAL(out.valid_repetitions, size_t(0));
    BOOST_CHECK_EQUAL(out.first_window_sample, int64_t(-1));
    BOOST_CHECK_EQUAL(out.last_window_sample, int64_t(-1));
}

bool prepare(const SyncCirRequest& req,
             size_t code_index,
             PreparedSyncCode& prepared,
             SyncCirScratch& scratch)
{
    return prepare_sync_code(GetPreambleCode(code_index), kSyncCirCodeLength,
                             code_index, req.pre, req.post, prepared, scratch);
}

} // namespace

// ---------------------------------------------------------------------------
// 1. Full matrix against the independent float64 reference.
//
// codes 9-12 x (pre,post) in {(8,30),(16,100)} x 64/256 SYNC x skip 10, with
// integer origins, a low-edge clipped origin (0) and non-integer periods
// (1015.7 / 1016.3) so round(k*P) lands on fractional positions.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_sync_cir_matrix_matches_float64_reference)
{
    const size_t codes[] = { 9, 10, 11, 12 };
    const size_t tap_cfgs[][2] = { { 8, 30 }, { 16, 100 } };
    const size_t totals[] = { 64, 256 };
    const double periods[] = { 1016.0, 1015.7, 1016.3 };
    const int64_t origins[] = { 0, 1997 };
    const size_t skip = 10;

    for (size_t code : codes) {
        for (size_t total : totals) {
            for (double period : periods) {
                for (int64_t origin : origins) {
                    const Packet pkt =
                        make_packet(code, total, period, origin);
                    for (const auto& taps : tap_cfgs) {
                        const size_t pre = taps[0];
                        const size_t post = taps[1];
                        const SyncCirRequest req =
                            make_req(origin, period, total, skip, 0, pre, post);
                        const std::string tag =
                            label(code, pre, post, total, period, origin);

                        PreparedSyncCode prepared;
                        SyncCirScratch scratch;
                        BOOST_REQUIRE_MESSAGE(
                            prepare(req, code, prepared, scratch), tag);

                        SyncCirResult out;
                        const bool ok = estimate_sync_cir(
                            pkt.rx.data(), pkt.rx.size(), req, prepared,
                            scratch, out);
                        BOOST_REQUIRE_MESSAGE(ok, tag);
                        BOOST_REQUIRE_MESSAGE(out.status == SyncCirStatus::Ok,
                                              tag);

                        const Float64Reference ref = float64_reference(
                            pkt.rx.data(), pkt.rx.size(), req, code);
                        BOOST_REQUIRE_MESSAGE(ref.ok, tag);

                        BOOST_CHECK_MESSAGE(out.tap_count == pre + post, tag);
                        BOOST_CHECK_MESSAGE(
                            out.valid_repetitions == ref.valid,
                            tag + " valid=" +
                                std::to_string(out.valid_repetitions) +
                                " expected=" + std::to_string(ref.valid));
                        BOOST_CHECK_MESSAGE(out.first_repetition == skip, tag);
                        BOOST_CHECK_MESSAGE(
                            out.first_window_sample == ref.first_lo,
                            tag + " first_window=" +
                                std::to_string(out.first_window_sample));
                        BOOST_CHECK_MESSAGE(
                            out.last_window_sample == ref.last_lo,
                            tag + " last_window=" +
                                std::to_string(out.last_window_sample));
                        BOOST_CHECK_MESSAGE(
                            out.code_energy == prepared.tmpl.code_energy &&
                                out.code_energy ==
                                    static_cast<float>(
                                        prepared.tmpl.entries.size()),
                            tag + " code_energy=" +
                                std::to_string(out.code_energy));

                        // Clean single path on the axis: the peak is exactly
                        // at tap == pre for every code/period/origin.
                        BOOST_CHECK_MESSAGE(out.peak_tap == pre, tag);
                        BOOST_CHECK_MESSAGE(out.peak_tap == ref.peak_tap,
                                            tag + " peak_tap=" +
                                                std::to_string(out.peak_tap) +
                                                " ref=" +
                                                std::to_string(ref.peak_tap));

                        const double e_raw =
                            rel_l2(scratch.raw.data(), out.tap_count, ref.raw);
                        const double e_norm =
                            rel_l2(scratch.norm.data(), out.tap_count, ref.norm);
                        BOOST_CHECK_MESSAGE(e_raw <= kRelL2Tolerance,
                                            tag + " raw rel L2=" +
                                                std::to_string(e_raw));
                        BOOST_CHECK_MESSAGE(e_norm <= kRelL2Tolerance,
                                            tag + " norm rel L2=" +
                                                std::to_string(e_norm));

                        const double e_peak =
                            std::abs(out.peak_abs - ref.peak_abs) /
                            (ref.peak_abs + 1e-30);
                        BOOST_CHECK_MESSAGE(e_peak < 1e-5, tag);
                        const double e_nrm =
                            std::abs(out.raw_l2_norm - ref.nrm) /
                            (ref.nrm + 1e-30);
                        BOOST_CHECK_MESSAGE(e_nrm < 1e-5, tag);
                    }
                }
            }
        }
    }
}

// ---------------------------------------------------------------------------
// 2. Bit-identity guard for the comm path.
//
// The legacy `stage_cir_softchips` float sequence, transcribed locally, must
// produce a `norm` that is memcmp-equal to the kernel's.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_sync_cir_comm_bit_identity)
{
    const size_t codes[] = { 9, 10 };
    const size_t tap_cfgs[][2] = { { 8, 30 }, { 16, 100 } };
    const double periods[] = { 1016.0, 1015.7 };
    const size_t total = 64;
    const size_t skip = 10;
    const int64_t origin = 1997;

    for (size_t code : codes) {
        for (const auto& taps : tap_cfgs) {
            for (double period : periods) {
                const size_t pre = taps[0];
                const size_t post = taps[1];
                const Packet pkt = make_packet(code, total, period, origin);
                const SyncCirRequest req =
                    make_req(origin, period, total, skip, 0, pre, post);
                const std::string tag =
                    label(code, pre, post, total, period, origin);

                PreparedSyncCode prepared;
                SyncCirScratch scratch;
                BOOST_REQUIRE_MESSAGE(prepare(req, code, prepared, scratch),
                                      tag);

                SyncCirResult out;
                BOOST_REQUIRE_MESSAGE(
                    estimate_sync_cir(pkt.rx.data(), pkt.rx.size(), req,
                                      prepared, scratch, out),
                    tag);
                BOOST_REQUIRE_MESSAGE(out.status == SyncCirStatus::Ok, tag);

                // The prepared template must be the legacy one.
                SparseCirTemplate legacy;
                build_sparse_cir_template(GetPreambleCode(code), kQm35CodeLength,
                                          legacy);
                BOOST_REQUIRE_EQUAL(legacy.key, prepared.tmpl.key);
                BOOST_REQUIRE_EQUAL(legacy.code_energy,
                                    prepared.tmpl.code_energy);
                BOOST_REQUIRE_EQUAL(legacy.entries.size(),
                                    prepared.tmpl.entries.size());

                const std::vector<std::complex<float>> comm =
                    comm_stage_cir_reference(pkt.rx.data(), pkt.rx.size(), req,
                                             legacy);
                BOOST_REQUIRE_EQUAL(comm.size(), out.tap_count);
                BOOST_CHECK_MESSAGE(
                    std::memcmp(comm.data(), scratch.norm.data(),
                                out.tap_count * sizeof(std::complex<float>)) ==
                        0,
                    tag + " norm is not bit-identical to stage_cir_softchips");
            }
        }
    }
}

// ---------------------------------------------------------------------------
// 3. Raw amplitude / phase, unit-L2 norm, no overwrite by normalisation.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_sync_cir_raw_amplitude_and_phase)
{
    const size_t code = 9;
    const size_t pre = 16;
    const size_t post = 100;
    const size_t total = 64;
    const size_t skip = 10;
    const int64_t origin = 1997;
    const SyncCirRequest req =
        make_req(origin, 1016.0, total, skip, 0, pre, post);
    const size_t n = 70000;
    const std::vector<std::complex<float>> rx = random_packet(n, 0x1234abcdULL);

    PreparedSyncCode prepared;
    SyncCirScratch scratch;
    BOOST_REQUIRE(prepare(req, code, prepared, scratch));

    SyncCirResult base;
    BOOST_REQUIRE(estimate_sync_cir(rx.data(), rx.size(), req, prepared, scratch,
                                    base));
    BOOST_REQUIRE(base.status == SyncCirStatus::Ok);
    const size_t tap_count = base.tap_count;
    std::vector<std::complex<float>> raw0(scratch.raw.begin(),
                                          scratch.raw.begin() + tap_count);
    std::vector<std::complex<float>> norm0(scratch.norm.begin(),
                                           scratch.norm.begin() + tap_count);

    // `norm` is unit L2, `raw` keeps its amplitude (not overwritten).
    const double norm_energy = l2_norm_sq(norm0.data(), tap_count);
    BOOST_CHECK_SMALL(norm_energy - 1.0, 1e-4);
    const double raw_energy = l2_norm_sq(raw0.data(), tap_count);
    BOOST_CHECK(std::abs(std::sqrt(raw_energy) - base.raw_l2_norm) <
                1e-5 * base.raw_l2_norm);
    BOOST_CHECK(std::memcmp(raw0.data(), norm0.data(),
                            tap_count * sizeof(std::complex<float>)) != 0);

    // Scaling the input by a known complex gain scales `raw` by exactly that
    // gain; `norm` is unit L2 and only picks up the gain's phase.
    const std::complex<float> gain(0.75f, -0.5f);
    const float gain_abs = std::abs(gain);
    const std::complex<float> gain_phase(gain.real() / gain_abs,
                                         gain.imag() / gain_abs);
    std::vector<std::complex<float>> scaled(rx.size());
    for (size_t i = 0; i < rx.size(); ++i)
        scaled[i] = rx[i] * gain;
    SyncCirResult scaled_out;
    BOOST_REQUIRE(estimate_sync_cir(scaled.data(), scaled.size(), req, prepared,
                                    scratch, scaled_out));
    BOOST_REQUIRE(scaled_out.status == SyncCirStatus::Ok);
    BOOST_CHECK_EQUAL(scaled_out.peak_tap, base.peak_tap);
    std::vector<std::complex<float>> raw_g(tap_count);
    std::vector<std::complex<float>> norm_g(tap_count);
    for (size_t i = 0; i < tap_count; ++i) {
        raw_g[i] = gain * raw0[i];
        norm_g[i] = gain_phase * norm0[i];
    }
    BOOST_CHECK_LT(rel_l2(scratch.raw.data(), raw_g.data(), tap_count), 1e-6);
    BOOST_CHECK_LT(rel_l2(scratch.norm.data(), norm_g.data(), tap_count), 1e-6);
    BOOST_CHECK(std::abs(l2_norm_sq(scratch.norm.data(), tap_count) - 1.0) <
                1e-4);

    // A real positive gain leaves `norm` untouched.
    const float real_gain = 2.5f;
    std::vector<std::complex<float>> scaled_real(rx.size());
    for (size_t i = 0; i < rx.size(); ++i)
        scaled_real[i] = rx[i] * real_gain;
    SyncCirResult real_out;
    BOOST_REQUIRE(estimate_sync_cir(scaled_real.data(), scaled_real.size(), req,
                                    prepared, scratch, real_out));
    BOOST_REQUIRE(real_out.status == SyncCirStatus::Ok);
    BOOST_CHECK_EQUAL(real_out.peak_tap, base.peak_tap);
    BOOST_CHECK_LT(rel_l2(scratch.norm.data(), norm0.data(), tap_count), 1e-6);
    BOOST_CHECK(std::abs(real_out.raw_l2_norm - real_gain * base.raw_l2_norm) <
                1e-5 * real_gain * base.raw_l2_norm);

    // Phase reference: multiplying the input by exp(j*theta) rotates every raw
    // tap by theta.
    const double theta = 0.7;
    const std::complex<float> rot(static_cast<float>(std::cos(theta)),
                                  static_cast<float>(std::sin(theta)));
    std::vector<std::complex<float>> rotated(rx.size());
    for (size_t i = 0; i < rx.size(); ++i)
        rotated[i] = rx[i] * rot;
    SyncCirResult rot_out;
    BOOST_REQUIRE(estimate_sync_cir(rotated.data(), rotated.size(), req,
                                    prepared, scratch, rot_out));
    BOOST_REQUIRE(rot_out.status == SyncCirStatus::Ok);
    BOOST_CHECK_EQUAL(rot_out.peak_tap, base.peak_tap);
    std::vector<std::complex<float>> raw_r(tap_count);
    for (size_t i = 0; i < tap_count; ++i)
        raw_r[i] = rot * raw0[i];
    BOOST_CHECK_LT(rel_l2(scratch.raw.data(), raw_r.data(), tap_count), 1e-5);
    for (size_t i = 0; i < tap_count; ++i) {
        if (std::abs(raw0[i]) < 0.05f * base.peak_abs)
            continue;
        const std::complex<double> expected =
            std::complex<double>(static_cast<double>(rot.real()),
                                 static_cast<double>(rot.imag())) *
            std::complex<double>(static_cast<double>(raw0[i].real()),
                                 static_cast<double>(raw0[i].imag()));
        const std::complex<double> got(
            static_cast<double>(scratch.raw[i].real()),
            static_cast<double>(scratch.raw[i].imag()));
        const double dphi = std::abs(std::arg(got * std::conj(expected)));
        BOOST_CHECK_SMALL(dphi, 1e-5);
    }
}

// ---------------------------------------------------------------------------
// 4. Delay axis: a known integer delay shifts peak_tap by exactly that delay
//    for both raw and norm, and the window coordinates stay on the axis.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_sync_cir_delay_axis)
{
    const size_t code = 11;
    const size_t pre = 8;
    const size_t post = 30;
    const size_t total = 64;
    const size_t skip = 10;
    const int64_t origin = 1997;
    const double period = 1016.0;
    const int64_t delays[] = { 0, 5, 17 };

    for (int64_t delay : delays) {
        const Packet pkt = make_packet(code, total, period, origin, delay);
        const SyncCirRequest req =
            make_req(origin, period, total, skip, 0, pre, post);
        const std::string tag = "delay=" + std::to_string(delay);

        PreparedSyncCode prepared;
        SyncCirScratch scratch;
        BOOST_REQUIRE_MESSAGE(prepare(req, code, prepared, scratch), tag);

        SyncCirResult out;
        BOOST_REQUIRE_MESSAGE(
            estimate_sync_cir(pkt.rx.data(), pkt.rx.size(), req, prepared,
                              scratch, out),
            tag);
        BOOST_REQUIRE_MESSAGE(out.status == SyncCirStatus::Ok, tag);
        BOOST_CHECK_MESSAGE(out.peak_tap == pre + static_cast<size_t>(delay),
                            tag + " peak_tap=" + std::to_string(out.peak_tap));
        BOOST_CHECK_EQUAL(out.valid_repetitions, total - skip);
        BOOST_CHECK_EQUAL(out.first_window_sample,
                          origin + static_cast<int64_t>(skip) * 1016 -
                              static_cast<int64_t>(pre));
        BOOST_CHECK_EQUAL(
            out.last_window_sample,
            origin + static_cast<int64_t>(total - 1) * 1016 -
                static_cast<int64_t>(pre));

        // The normalised taps peak at the same index.
        size_t norm_peak = 0;
        float norm_peak_abs = -1.0f;
        for (size_t i = 0; i < out.tap_count; ++i) {
            const float m = std::abs(scratch.norm[i]);
            if (m > norm_peak_abs) {
                norm_peak_abs = m;
                norm_peak = i;
            }
        }
        BOOST_CHECK_MESSAGE(norm_peak == out.peak_tap, tag);
        BOOST_CHECK(std::abs(l2_norm_sq(scratch.norm.data(), out.tap_count) -
                             1.0) < 1e-4);
        // Only the delayed tap carries energy for a clean single path.
        BOOST_CHECK_GT(out.peak_abs, 0.0f);
    }
}

// ---------------------------------------------------------------------------
// 5. Windows clipped at the low edge, the high edge, and both.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_sync_cir_clipped_windows)
{
    const size_t code = 10;
    const size_t pre = 16;
    const size_t post = 100;
    const size_t total = 64;
    const size_t skip = 0;
    const double period = 1016.0;

    struct Case {
        const char* name;
        int64_t origin;
        size_t truncate; // drop this many samples from the high end
    };
    const Case cases[] = {
        { "low-edge", 0, 0 },
        { "high-edge", 1997, 3 * 1016 + 200 },
        { "both-edges", 0, 2 * 1016 + 500 },
    };

    for (const Case& c : cases) {
        Packet pkt = make_packet(code, total, period, c.origin);
        const size_t n = pkt.rx.size() - c.truncate;
        BOOST_REQUIRE(n > 0);
        const SyncCirRequest req =
            make_req(c.origin, period, total, skip, 0, pre, post);
        const std::string tag = c.name;

        const std::vector<int64_t> accepted = accepted_windows(req, n);
        BOOST_REQUIRE_MESSAGE(!accepted.empty(), tag);
        BOOST_REQUIRE_MESSAGE(accepted.size() < total - skip,
                              tag + ": the case must actually clip");

        PreparedSyncCode prepared;
        SyncCirScratch scratch;
        BOOST_REQUIRE_MESSAGE(prepare(req, code, prepared, scratch), tag);

        SyncCirResult out;
        BOOST_REQUIRE_MESSAGE(
            estimate_sync_cir(pkt.rx.data(), n, req, prepared, scratch, out),
            tag);
        BOOST_REQUIRE_MESSAGE(out.status == SyncCirStatus::Ok, tag);
        BOOST_CHECK_MESSAGE(out.valid_repetitions == accepted.size(),
                            tag + " valid=" +
                                std::to_string(out.valid_repetitions) +
                                " expected=" + std::to_string(accepted.size()));
        BOOST_CHECK_MESSAGE(out.first_window_sample == accepted.front(), tag);
        BOOST_CHECK_MESSAGE(out.last_window_sample == accepted.back(), tag);

        const Float64Reference ref =
            float64_reference(pkt.rx.data(), n, req, code);
        BOOST_REQUIRE_MESSAGE(ref.ok, tag);
        BOOST_CHECK_MESSAGE(out.peak_tap == ref.peak_tap, tag);
        BOOST_CHECK_MESSAGE(
            rel_l2(scratch.raw.data(), out.tap_count, ref.raw) <=
                kRelL2Tolerance,
            tag);
        BOOST_CHECK_MESSAGE(
            rel_l2(scratch.norm.data(), out.tap_count, ref.norm) <=
                kRelL2Tolerance,
            tag);
    }
}

// ---------------------------------------------------------------------------
// 6. No valid repetition -> NoValidRepetitions, taps zeroed.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_sync_cir_no_valid_repetitions)
{
    const size_t code = 9;
    const Packet pkt = make_packet(code, 1, 1016.0, 0);

    // pre = 16 pushes the only window's lo to -16.
    const SyncCirRequest req = make_req(0, 1016.0, 1, 0, 0, 16, 100);
    PreparedSyncCode prepared;
    SyncCirScratch scratch;
    BOOST_REQUIRE(prepare(req, code, prepared, scratch));

    SyncCirResult out;
    BOOST_REQUIRE(!estimate_sync_cir(pkt.rx.data(), pkt.rx.size(), req,
                                     prepared, scratch, out));
    BOOST_CHECK(out.status == SyncCirStatus::NoValidRepetitions);
    check_failed_result_zeroed(out);
    check_taps_zeroed(scratch);

    // Buffer shorter than one window: every repetition is clipped.
    const SyncCirRequest short_req = make_req(1997, 1016.0, 64, 10, 0, 16, 100);
    PreparedSyncCode short_prepared;
    SyncCirScratch short_scratch;
    BOOST_REQUIRE(prepare(short_req, code, short_prepared, short_scratch));
    BOOST_REQUIRE(!estimate_sync_cir(pkt.rx.data(), 512, short_req,
                                     short_prepared, short_scratch, out));
    BOOST_CHECK(out.status == SyncCirStatus::NoValidRepetitions);
    check_failed_result_zeroed(out);
    check_taps_zeroed(short_scratch);

    // required_input_range agrees: no accepted window -> false.
    int64_t lo = 0;
    int64_t hi = 0;
    BOOST_CHECK(!required_input_range(short_req, kSyncCirCodeLength, 512, lo,
                                      hi));
    BOOST_CHECK_EQUAL(lo, int64_t(-1));
    BOOST_CHECK_EQUAL(hi, int64_t(-1));
}

// ---------------------------------------------------------------------------
// 7. Capacity insufficient -> CapacityInsufficient, output zeroed.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_sync_cir_capacity_insufficient)
{
    const size_t code = 9;
    const size_t total = 64;
    const size_t skip = 10;
    const int64_t origin = 1997;
    const Packet pkt = make_packet(code, total, 1016.0, origin);

    // Scratch prepared for 38 taps, request for 116 taps.
    const SyncCirRequest small = make_req(origin, 1016.0, total, skip, 0, 8, 30);
    const SyncCirRequest big = make_req(origin, 1016.0, total, skip, 0, 16, 100);
    PreparedSyncCode prepared;
    SyncCirScratch scratch;
    BOOST_REQUIRE(prepare(small, code, prepared, scratch));

    SyncCirResult ok_out;
    BOOST_REQUIRE(estimate_sync_cir(pkt.rx.data(), pkt.rx.size(), small,
                                    prepared, scratch, ok_out));
    BOOST_REQUIRE(ok_out.status == SyncCirStatus::Ok);

    SyncCirResult out;
    BOOST_REQUIRE(!estimate_sync_cir(pkt.rx.data(), pkt.rx.size(), big,
                                     prepared, scratch, out));
    BOOST_CHECK(out.status == SyncCirStatus::CapacityInsufficient);
    check_failed_result_zeroed(out);
    check_taps_zeroed(scratch);

    // avg capacity is checked independently of the tap capacity.
    PreparedSyncCode wide_prepared;
    SyncCirScratch wide_scratch;
    BOOST_REQUIRE(prepare(big, code, wide_prepared, wide_scratch));
    BOOST_REQUIRE_GE(wide_scratch.avg.size(),
                     kSyncCirCodeLength + big.pre + big.post - 1);
    wide_scratch.avg = std::vector<std::complex<float>>(
        wide_scratch.avg.begin(), wide_scratch.avg.begin() + 1000);
    BOOST_REQUIRE_LT(wide_scratch.avg.capacity(),
                     kSyncCirCodeLength + big.pre + big.post - 1);
    BOOST_REQUIRE(!estimate_sync_cir(pkt.rx.data(), pkt.rx.size(), big,
                                     wide_prepared, wide_scratch, out));
    BOOST_CHECK(out.status == SyncCirStatus::CapacityInsufficient);
    check_failed_result_zeroed(out);
    check_taps_zeroed(wide_scratch);
}

// ---------------------------------------------------------------------------
// 8. Non-finite input -> NonFinite, output zeroed.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_sync_cir_non_finite_input)
{
    const size_t code = 9;
    const size_t pre = 16;
    const size_t post = 100;
    const size_t total = 64;
    const size_t skip = 10;
    const int64_t origin = 1997;
    const SyncCirRequest req =
        make_req(origin, 1016.0, total, skip, 0, pre, post);
    Packet pkt = make_packet(code, total, 1016.0, origin);

    PreparedSyncCode prepared;
    SyncCirScratch scratch;
    BOOST_REQUIRE(prepare(req, code, prepared, scratch));

    SyncCirResult out;
    for (int kind = 0; kind < 2; ++kind) {
        Packet bad = pkt;
        // Inside the first accepted window (k = skip).
        const size_t idx = static_cast<size_t>(
            origin + static_cast<int64_t>(skip) * 1016 - 5);
        bad.rx[idx] = (kind == 0)
                          ? std::complex<float>(
                                std::numeric_limits<float>::quiet_NaN(), 0.f)
                          : std::complex<float>(
                                std::numeric_limits<float>::infinity(), 0.f);
        BOOST_REQUIRE(!estimate_sync_cir(bad.rx.data(), bad.rx.size(), req,
                                         prepared, scratch, out));
        BOOST_CHECK(out.status == SyncCirStatus::NonFinite);
        check_failed_result_zeroed(out);
        check_taps_zeroed(scratch);
    }

    // A NaN in a clipped window must NOT poison the estimate.
    Packet clipped = pkt;
    clipped.rx[0] = std::complex<float>(
        std::numeric_limits<float>::quiet_NaN(), 0.f);
    BOOST_REQUIRE(estimate_sync_cir(clipped.rx.data(), clipped.rx.size(), req,
                                    prepared, scratch, out));
    BOOST_CHECK(out.status == SyncCirStatus::Ok);
    BOOST_CHECK_EQUAL(out.peak_tap, pre);
}

// ---------------------------------------------------------------------------
// 9. Invalid input.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_sync_cir_invalid_input)
{
    const size_t code = 9;
    const size_t total = 64;
    const size_t skip = 10;
    const int64_t origin = 1997;
    const Packet pkt = make_packet(code, total, 1016.0, origin);
    const SyncCirRequest good = make_req(origin, 1016.0, total, skip, 0, 16, 100);

    PreparedSyncCode prepared;
    SyncCirScratch scratch;
    BOOST_REQUIRE(prepare(good, code, prepared, scratch));

    SyncCirResult out;
    auto expect_invalid = [&](const SyncCirRequest& req,
                              const std::complex<float>* rx,
                              size_t n,
                              const char* what) {
        BOOST_REQUIRE_MESSAGE(
            !estimate_sync_cir(rx, n, req, prepared, scratch, out), what);
        BOOST_CHECK_MESSAGE(out.status == SyncCirStatus::InvalidInput, what);
        check_failed_result_zeroed(out);
    };

    expect_invalid(good, nullptr, pkt.rx.size(), "null rx");
    expect_invalid(good, pkt.rx.data(), 0, "n == 0");

    SyncCirRequest bad = good;
    bad.pre = 0;
    bad.post = 0;
    expect_invalid(bad, pkt.rx.data(), pkt.rx.size(), "pre+post == 0");

    bad = good;
    bad.samples_per_symbol = 0.0;
    expect_invalid(bad, pkt.rx.data(), pkt.rx.size(), "P == 0");

    bad = good;
    bad.samples_per_symbol = -1016.0;
    expect_invalid(bad, pkt.rx.data(), pkt.rx.size(), "P < 0");

    bad = good;
    bad.samples_per_symbol = std::numeric_limits<double>::quiet_NaN();
    expect_invalid(bad, pkt.rx.data(), pkt.rx.size(), "P NaN");

    bad = good;
    bad.samples_per_symbol = std::numeric_limits<double>::infinity();
    expect_invalid(bad, pkt.rx.data(), pkt.rx.size(), "P Inf");

    bad = good;
    bad.cir_axis_origin = -1;
    expect_invalid(bad, pkt.rx.data(), pkt.rx.size(), "origin < 0");

    bad = good;
    bad.skip_initial = total;
    expect_invalid(bad, pkt.rx.data(), pkt.rx.size(),
                   "skip_initial == total");

    bad = good;
    bad.skip_initial = total + 5;
    expect_invalid(bad, pkt.rx.data(), pkt.rx.size(),
                   "skip_initial > total");

    bad = good;
    bad.total_repetitions = 0;
    expect_invalid(bad, pkt.rx.data(), pkt.rx.size(),
                   "total_repetitions == 0");

    // Unprepared code -> InvalidInput (this is the "code_len == 0" case: the
    // canonical entry point refuses to prepare a zero-length code).
    PreparedSyncCode unprepared;
    SyncCirScratch unprepared_scratch;
    BOOST_REQUIRE(!estimate_sync_cir(pkt.rx.data(), pkt.rx.size(), good,
                                     unprepared, unprepared_scratch, out));
    BOOST_CHECK(out.status == SyncCirStatus::InvalidInput);
    check_failed_result_zeroed(out);

    // prepare_sync_code rejects a zero-length / null / ternary-length code.
    PreparedSyncCode p;
    SyncCirScratch s;
    BOOST_REQUIRE(!prepare_sync_code(GetPreambleCode(code), 0, code, 16, 100, p,
                                     s));
    BOOST_CHECK(!p.ready);
    BOOST_CHECK(s.avg.empty());
    BOOST_CHECK(s.raw.empty());
    BOOST_CHECK(s.norm.empty());

    BOOST_REQUIRE(!prepare_sync_code(nullptr, kSyncCirCodeLength, code, 16, 100,
                                     p, s));
    BOOST_CHECK(!p.ready);

    // The ternary code length (127) is not the window length: fail closed
    // instead of over-reading the code array.
    BOOST_REQUIRE(!prepare_sync_code(GetPreambleCode(code), kQm35CodeLength, code,
                                     16, 100, p, s));
    BOOST_CHECK(!p.ready);

    BOOST_REQUIRE(!prepare_sync_code(GetPreambleCode(code), kSyncCirCodeLength,
                                     code, 0, 0, p, s));
    BOOST_CHECK(!p.ready);

    // required_input_range mirrors the same validation.
    int64_t lo = 0;
    int64_t hi = 0;
    BOOST_CHECK(!required_input_range(good, 0, pkt.rx.size(), lo, hi));
    BOOST_CHECK(!required_input_range(good, kSyncCirCodeLength, 0, lo, hi));
    bad = good;
    bad.pre = 0;
    bad.post = 0;
    BOOST_CHECK(!required_input_range(bad, kSyncCirCodeLength, pkt.rx.size(),
                                      lo, hi));
    bad = good;
    bad.cir_axis_origin = -1;
    BOOST_CHECK(!required_input_range(bad, kSyncCirCodeLength, pkt.rx.size(),
                                      lo, hi));
    bad = good;
    bad.skip_initial = total;
    BOOST_CHECK(!required_input_range(bad, kSyncCirCodeLength, pkt.rx.size(),
                                      lo, hi));
}

// ---------------------------------------------------------------------------
// 10. required_input_range is the union envelope of the accepted windows.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_sync_cir_required_input_range_matches_accepted)
{
    const size_t code = 12;
    const size_t total = 64;
    const double period = 1015.7;

    struct Case {
        int64_t origin;
        size_t skip;
        size_t max_reps;
        size_t pre;
        size_t post;
        size_t n;
    };
    const Case cases[] = {
        { 1997, 10, 0, 16, 100, 70000 },
        { 1997, 10, 0, 8, 30, 70000 },
        { 0, 0, 0, 16, 100, 70000 },
        { 1997, 0, 0, 16, 100, 40000 },
        { 0, 0, 0, 8, 30, 30000 },
        { 1997, 10, 7, 16, 100, 70000 },
        { 1997, 55, 0, 8, 30, 70000 },
    };

    for (const Case& c : cases) {
        const Packet pkt = make_packet(code, total, period, c.origin);
        const size_t n = std::min(c.n, pkt.rx.size());
        const SyncCirRequest req = make_req(c.origin, period, total, c.skip,
                                            c.max_reps, c.pre, c.post);

        const std::vector<int64_t> accepted = accepted_windows(req, n);
        int64_t lo = 0;
        int64_t hi = 0;
        const bool ok = required_input_range(req, kSyncCirCodeLength, n, lo, hi);
        BOOST_REQUIRE_EQUAL(ok, !accepted.empty());
        if (accepted.empty()) {
            BOOST_CHECK_EQUAL(lo, int64_t(-1));
            BOOST_CHECK_EQUAL(hi, int64_t(-1));
            continue;
        }
        const size_t wlen = kSyncCirCodeLength + req.pre + req.post - 1;
        const int64_t expected_lo = *std::min_element(accepted.begin(),
                                                     accepted.end());
        const int64_t expected_hi = *std::max_element(accepted.begin(),
                                                      accepted.end()) +
                                    static_cast<int64_t>(wlen);
        BOOST_CHECK_EQUAL(lo, expected_lo);
        BOOST_CHECK_EQUAL(hi, expected_hi);

        // The kernel must accept exactly that set.
        PreparedSyncCode prepared;
        SyncCirScratch scratch;
        BOOST_REQUIRE(prepare(req, code, prepared, scratch));
        SyncCirResult out;
        BOOST_REQUIRE(estimate_sync_cir(pkt.rx.data(), n, req, prepared, scratch,
                                        out));
        BOOST_REQUIRE(out.status == SyncCirStatus::Ok);
        BOOST_CHECK_EQUAL(out.valid_repetitions, accepted.size());
        BOOST_CHECK_EQUAL(out.first_window_sample, expected_lo);
        BOOST_CHECK_EQUAL(out.last_window_sample,
                          *std::max_element(accepted.begin(), accepted.end()));
        BOOST_CHECK_GE(out.first_window_sample, int64_t(0));
        BOOST_CHECK_LE(out.last_window_sample + static_cast<int64_t>(wlen),
                       static_cast<int64_t>(n));
        BOOST_CHECK_EQUAL(lo, out.first_window_sample);
        BOOST_CHECK_EQUAL(hi, out.last_window_sample + static_cast<int64_t>(wlen));
    }
}

// ---------------------------------------------------------------------------
// 11. prepare_sync_code contract: sizes, identity, template, no reallocation
//     on the hot path.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_sync_cir_prepare_contract)
{
    const size_t code = 10;
    const size_t max_pre = 16;
    const size_t max_post = 100;
    PreparedSyncCode prepared;
    SyncCirScratch scratch;
    BOOST_REQUIRE(prepare_sync_code(GetPreambleCode(code), kSyncCirCodeLength,
                                    code, max_pre, max_post, prepared, scratch));

    BOOST_CHECK(prepared.ready);
    BOOST_CHECK_EQUAL(prepared.code_index, code);
    BOOST_CHECK_EQUAL(prepared.fingerprint,
                      cir_template_key(GetPreambleCode(code), kQm35CodeLength));
    BOOST_CHECK_EQUAL(prepared.fingerprint, prepared.tmpl.key);
    // Ternary Ipatov codes contain zeros, so code_energy is the number of
    // non-zero sampled entries, not the ternary code length.
    BOOST_CHECK_EQUAL(prepared.tmpl.code_energy,
                      static_cast<float>(prepared.tmpl.entries.size()));
    BOOST_CHECK_GT(prepared.tmpl.code_energy, 0.0f);
    BOOST_CHECK_LE(prepared.tmpl.code_energy,
                   static_cast<float>(kQm35CodeLength));

    SparseCirTemplate legacy;
    build_sparse_cir_template(GetPreambleCode(code), kQm35CodeLength, legacy);
    BOOST_REQUIRE_EQUAL(prepared.tmpl.entries.size(), legacy.entries.size());
    for (size_t i = 0; i < legacy.entries.size(); ++i) {
        BOOST_CHECK_EQUAL(prepared.tmpl.entries[i].pos, legacy.entries[i].pos);
        BOOST_CHECK_EQUAL(prepared.tmpl.entries[i].sign,
                          legacy.entries[i].sign);
    }

    BOOST_CHECK_EQUAL(scratch.avg.size(),
                      kSyncCirCodeLength + max_pre + max_post - 1);
    BOOST_CHECK_EQUAL(scratch.raw.size(), max_pre + max_post);
    BOOST_CHECK_EQUAL(scratch.norm.size(), max_pre + max_post);
    BOOST_CHECK_GE(scratch.avg.capacity(), scratch.avg.size());
    BOOST_CHECK_GE(scratch.raw.capacity(), scratch.raw.size());
    BOOST_CHECK_GE(scratch.norm.capacity(), scratch.norm.size());

    // The hot path must not reallocate.
    const std::complex<float>* avg_ptr = scratch.avg.data();
    const std::complex<float>* raw_ptr = scratch.raw.data();
    const std::complex<float>* norm_ptr = scratch.norm.data();
    const size_t avg_cap = scratch.avg.capacity();
    const size_t raw_cap = scratch.raw.capacity();
    const size_t norm_cap = scratch.norm.capacity();

    const Packet pkt = make_packet(code, 64, 1016.0, 1997);
    const SyncCirRequest req = make_req(1997, 1016.0, 64, 10, 0, max_pre,
                                        max_post);
    SyncCirResult out;
    BOOST_REQUIRE(estimate_sync_cir(pkt.rx.data(), pkt.rx.size(), req, prepared,
                                    scratch, out));
    BOOST_CHECK(scratch.avg.data() == avg_ptr);
    BOOST_CHECK(scratch.raw.data() == raw_ptr);
    BOOST_CHECK(scratch.norm.data() == norm_ptr);
    BOOST_CHECK_EQUAL(scratch.avg.capacity(), avg_cap);
    BOOST_CHECK_EQUAL(scratch.raw.capacity(), raw_cap);
    BOOST_CHECK_EQUAL(scratch.norm.capacity(), norm_cap);
    BOOST_CHECK_EQUAL(scratch.norm.size(), max_pre + max_post);

    // Re-preparing with a smaller window shrinks the buffers (no stale state).
    PreparedSyncCode small;
    SyncCirScratch small_scratch;
    BOOST_REQUIRE(prepare_sync_code(GetPreambleCode(code), kSyncCirCodeLength,
                                    code, 8, 30, small, small_scratch));
    BOOST_CHECK_EQUAL(small_scratch.avg.size(), kSyncCirCodeLength + 8 + 30 - 1);
    BOOST_CHECK_EQUAL(small_scratch.raw.size(), size_t(38));
    BOOST_CHECK_EQUAL(small_scratch.norm.size(), size_t(38));

    // A failed prepare leaves nothing that looks ready.
    PreparedSyncCode failed;
    SyncCirScratch failed_scratch;
    BOOST_REQUIRE(!prepare_sync_code(GetPreambleCode(code), kSyncCirCodeLength,
                                     code, 0, 0, failed, failed_scratch));
    BOOST_CHECK(!failed.ready);
    BOOST_CHECK_EQUAL(failed.fingerprint, uint64_t(0));
    BOOST_CHECK(failed_scratch.avg.empty());
    BOOST_CHECK(failed_scratch.raw.empty());
    BOOST_CHECK(failed_scratch.norm.empty());
}

// ---------------------------------------------------------------------------
// 12. skip_initial / max_repetitions window selection.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_sync_cir_repetition_selection)
{
    const size_t code = 9;
    const size_t pre = 8;
    const size_t post = 30;
    const size_t total = 64;
    const int64_t origin = 1997;
    const double period = 1016.0;
    const Packet pkt = make_packet(code, total, period, origin);

    struct Case {
        size_t skip;
        size_t max_reps;
        size_t expected;
    };
    const Case cases[] = {
        { 10, 0, 54 },  // 0 = all remaining
        { 10, 10, 10 }, // explicit limit
        { 10, 100, 54 },// clamped to the remaining count
        { 0, 0, 64 },
        { 63, 0, 1 },
    };

    for (const Case& c : cases) {
        const SyncCirRequest req =
            make_req(origin, period, total, c.skip, c.max_reps, pre, post);
        PreparedSyncCode prepared;
        SyncCirScratch scratch;
        BOOST_REQUIRE(prepare(req, code, prepared, scratch));
        SyncCirResult out;
        BOOST_REQUIRE(estimate_sync_cir(pkt.rx.data(), pkt.rx.size(), req,
                                        prepared, scratch, out));
        BOOST_REQUIRE(out.status == SyncCirStatus::Ok);
        BOOST_CHECK_EQUAL(out.valid_repetitions, c.expected);
        BOOST_CHECK_EQUAL(out.first_repetition, c.skip);
        BOOST_CHECK_EQUAL(out.first_window_sample,
                          origin + static_cast<int64_t>(c.skip) * 1016 -
                              static_cast<int64_t>(pre));
        BOOST_CHECK_EQUAL(out.last_window_sample,
                          origin + static_cast<int64_t>(c.skip + c.expected - 1) *
                                       1016 -
                              static_cast<int64_t>(pre));
    }
}

// ---------------------------------------------------------------------------
// 13. Non-integer samples_per_symbol: the axis uses llround(k*P) exactly and
//     the physical peak does not move.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_sync_cir_fractional_period_axis)
{
    const size_t code = 12;
    const size_t pre = 16;
    const size_t post = 100;
    const size_t total = 256;
    const size_t skip = 10;
    const int64_t origin = 1997;
    const double periods[] = { 1015.7, 1016.3 };

    for (double period : periods) {
        const Packet pkt = make_packet(code, total, period, origin);
        const SyncCirRequest req =
            make_req(origin, period, total, skip, 0, pre, post);
        const std::string tag = "P=" + std::to_string(period);

        PreparedSyncCode prepared;
        SyncCirScratch scratch;
        BOOST_REQUIRE_MESSAGE(prepare(req, code, prepared, scratch), tag);
        SyncCirResult out;
        BOOST_REQUIRE_MESSAGE(
            estimate_sync_cir(pkt.rx.data(), pkt.rx.size(), req, prepared,
                              scratch, out),
            tag);
        BOOST_REQUIRE_MESSAGE(out.status == SyncCirStatus::Ok, tag);
        BOOST_CHECK_MESSAGE(out.valid_repetitions == total - skip, tag);
        BOOST_CHECK_MESSAGE(out.peak_tap == pre, tag);

        // The first/last window origins follow llround(k*P), not k*round(P).
        BOOST_CHECK_EQUAL(
            out.first_window_sample,
            origin + static_cast<int64_t>(
                         std::llround(static_cast<double>(skip) * period)) -
                static_cast<int64_t>(pre));
        BOOST_CHECK_EQUAL(
            out.last_window_sample,
            origin + static_cast<int64_t>(
                         std::llround(static_cast<double>(total - 1) * period)) -
                static_cast<int64_t>(pre));
        // ... and differ from the integer grid, otherwise the case is vacuous.
        BOOST_CHECK(out.last_window_sample !=
                    origin + static_cast<int64_t>(total - 1) * 1016 -
                        static_cast<int64_t>(pre));

        const Float64Reference ref =
            float64_reference(pkt.rx.data(), pkt.rx.size(), req, code);
        BOOST_REQUIRE_MESSAGE(ref.ok, tag);
        BOOST_CHECK_MESSAGE(out.peak_tap == ref.peak_tap, tag);
        BOOST_CHECK_MESSAGE(
            rel_l2(scratch.raw.data(), out.tap_count, ref.raw) <=
                kRelL2Tolerance,
            tag);
        BOOST_CHECK_MESSAGE(
            rel_l2(scratch.norm.data(), out.tap_count, ref.norm) <=
                kRelL2Tolerance,
            tag);
    }
}
