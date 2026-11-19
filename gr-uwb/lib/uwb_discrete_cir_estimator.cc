/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Discrete equivalent CIR estimator for the full-packet SIC experiment.
 *
 * This TU implements ONLY the estimator half of uwb_discrete_cir_sic.h:
 *   - DiscreteCirScratch::reserve()
 *   - prepare_discrete_cir()
 *   - estimate_discrete_cir()
 *
 * apply_discrete_cir() / fullpacket_cancel_discrete() live in the
 * reconstruction TU.  The frozen preamble core is not called and not changed.
 *
 * Support model (calibrated S0 revision):
 *   * coarse matched-filter CIR over every lag in [lag_lo, lag_lo+tap_count);
 *   * local maxima above peak_floor * global_peak, greedily spaced by at
 *     least peak_min_spacing samples;
 *   * each kept peak activates [peak-active_radius, peak+active_radius];
 *   * the LS columns are ONLY the active lags, while the frozen model stores
 *     a dense hull [min(active),max(active)] with exactly zero inactive taps.
 * A wide continuous hull LS would fit a second (sensing) preamble into h; the
 * peak groups deliberately do not.
 *
 * The active-set normal equations are factored lazily into a fixed-size
 * factor cache.  prepare() sizes every cache slot and reserves all work
 * buffers; estimate() never allocates or grows a vector.
 *
 * Strict prefix: every read of y[] is bounded by visible_end.  Detection and
 * repetition rows must be fully visible; the training target and the folded
 * coarse CIR use only visible samples.  No sidecar/truth parameter exists.
 */
#include <gnuradio/uwb/uwb_discrete_cir_sic.h>

#include <volk/volk.h>
#if defined(__x86_64__) || defined(_M_X64)
#include <immintrin.h>
#endif

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace gr {
namespace uwb {
namespace sic {
namespace {

using cf = std::complex<float>;
using cd = std::complex<double>;
using Clock = std::chrono::steady_clock;

constexpr double kPi = 3.14159265358979323846;
constexpr size_t kMinSps = 64;
constexpr size_t kMaxSps = 4096;
constexpr size_t kMaxFitReps = 64;
constexpr size_t kMaxTaps = kDiscreteCirMaxTaps;
constexpr size_t kMaxActive = 64; // unchanged legacy LS solve bound
constexpr size_t kFactorCacheSlots = 16;
constexpr int kJacobiSweeps = 24;
constexpr uint64_t kFnvOffset = 14695981039346656037ull;
constexpr uint64_t kFnvPrime = 1099511628211ull;
constexpr uint64_t kHashDomain = 0x44435231ull; // "DCR1"

double us_since(Clock::time_point t)
{
    return std::chrono::duration<double, std::micro>(Clock::now() - t).count();
}

bool finite_cf(const cf& v)
{
    return std::isfinite(v.real()) && std::isfinite(v.imag());
}

bool finite_cd(const cd& v)
{
    return std::isfinite(v.real()) && std::isfinite(v.imag());
}

#if (defined(__x86_64__) || defined(_M_X64)) && (defined(__GNUC__) || defined(__clang__))
__attribute__((target("avx2")))
bool scan_finite_avx2(const float* data, size_t count)
{
    const __m256i exp_mask = _mm256_set1_epi32(0x7F800000);
    __m256i or_acc = _mm256_setzero_si256();
    size_t i = 0;
    const size_t limit32 = (count >= 32) ? (count - 31) : 0;
    for (; i < limit32; i += 32) {
        __m256i v0 = _mm256_loadu_si256(reinterpret_cast<const __m256i*>(data + i));
        __m256i v1 = _mm256_loadu_si256(reinterpret_cast<const __m256i*>(data + i + 8));
        __m256i v2 = _mm256_loadu_si256(reinterpret_cast<const __m256i*>(data + i + 16));
        __m256i v3 = _mm256_loadu_si256(reinterpret_cast<const __m256i*>(data + i + 24));
        __m256i m0 = _mm256_cmpeq_epi32(_mm256_and_si256(v0, exp_mask), exp_mask);
        __m256i m1 = _mm256_cmpeq_epi32(_mm256_and_si256(v1, exp_mask), exp_mask);
        __m256i m2 = _mm256_cmpeq_epi32(_mm256_and_si256(v2, exp_mask), exp_mask);
        __m256i m3 = _mm256_cmpeq_epi32(_mm256_and_si256(v3, exp_mask), exp_mask);
        or_acc = _mm256_or_si256(or_acc, _mm256_or_si256(_mm256_or_si256(m0, m1), _mm256_or_si256(m2, m3)));
    }
    const size_t limit8 = (count >= 8) ? (count - 7) : 0;
    for (; i < limit8; i += 8) {
        __m256i v = _mm256_loadu_si256(reinterpret_cast<const __m256i*>(data + i));
        __m256i m = _mm256_cmpeq_epi32(_mm256_and_si256(v, exp_mask), exp_mask);
        or_acc = _mm256_or_si256(or_acc, m);
    }
    if (!_mm256_testz_si256(or_acc, or_acc))
        return false;
    for (; i < count; ++i) {
        if (!std::isfinite(data[i]))
            return false;
    }
    return true;
}
#endif

bool scan_finite(const cf* y, size_t count)
{
    const float* data = reinterpret_cast<const float*>(y);
    const size_t total_floats = count * 2;
#if (defined(__x86_64__) || defined(_M_X64)) && (defined(__GNUC__) || defined(__clang__))
    if (__builtin_cpu_supports("avx2")) {
        return scan_finite_avx2(data, total_floats);
    }
#endif
    for (size_t i = 0; i < total_floats; ++i) {
        if (!std::isfinite(data[i]))
            return false;
    }
    return true;
}

void build_cfo_rotation(double cfo_hz, double fs, size_t count, std::vector<cd>& cfo_rot)
{
    if (cfo_rot.size() < count)
        cfo_rot.resize(count);
    const double w = 2.0 * kPi * cfo_hz / fs;
    const cd step1 = std::polar(1.0, -w);
    const cd step2 = step1 * step1;
    const cd step3 = step2 * step1;
    const cd step4 = step2 * step2;

    for (size_t block = 0; block < count; block += 1024) {
        const size_t blk_len = std::min<size_t>(1024, count - block);
        cd cur0 = std::polar(1.0, -w * static_cast<double>(block));
        cd cur1 = cur0 * step1;
        cd cur2 = cur0 * step2;
        cd cur3 = cur0 * step3;
        size_t i = 0;
        const size_t limit = (blk_len >= 4) ? (blk_len - 3) : 0;
        for (; i < limit; i += 4) {
            cfo_rot[block + i + 0] = cur0;
            cfo_rot[block + i + 1] = cur1;
            cfo_rot[block + i + 2] = cur2;
            cfo_rot[block + i + 3] = cur3;
            cur0 *= step4;
            cur1 *= step4;
            cur2 *= step4;
            cur3 *= step4;
        }
        for (; i < blk_len; ++i) {
            cfo_rot[block + i] = cur0;
            cur0 *= step1;
        }
    }
}

void hash_bytes(uint64_t& h, const void* data, size_t bytes)
{
    const auto* p = static_cast<const unsigned char*>(data);
    for (size_t i = 0; i < bytes; ++i) {
        h ^= static_cast<uint64_t>(p[i]);
        h *= kFnvPrime;
    }
}

void hash_u64(uint64_t& h, uint64_t v)
{
    hash_bytes(h, &v, sizeof(v));
}

void hash_i64(uint64_t& h, int64_t v)
{
    hash_u64(h, static_cast<uint64_t>(v));
}

void hash_double(uint64_t& h, double v)
{
    static_assert(sizeof(double) == sizeof(uint64_t), "unexpected double width");
    uint64_t bits = 0;
    std::memcpy(&bits, &v, sizeof(bits));
    hash_u64(h, bits);
}

void hash_float(uint64_t& h, float v)
{
    static_assert(sizeof(float) == sizeof(uint32_t), "unexpected float width");
    uint32_t bits = 0;
    std::memcpy(&bits, &v, sizeof(bits));
    hash_u64(h, static_cast<uint64_t>(bits));
}

// Deterministic template identity: lengths plus raw float bytes.  No pointer
// value is hashed, so a model fingerprint is reproducible across processes.
uint64_t template_identity(const cf* sync, size_t sync_len, const cf* train,
                           size_t train_len)
{
    uint64_t h = kFnvOffset;
    hash_u64(h, kHashDomain);
    hash_u64(h, static_cast<uint64_t>(sync_len));
    hash_bytes(h, sync, sync_len * sizeof(cf));
    hash_u64(h, static_cast<uint64_t>(train_len));
    hash_bytes(h, train, train_len * sizeof(cf));
    return h;
}

// Algorithmic preparation key.  The public per-call ROI and capacity fields
// are excluded: they do not change any cached factor.
uint64_t config_identity(const DiscreteCirConfig& cfg)
{
    uint64_t h = kFnvOffset;
    hash_u64(h, kHashDomain ^ 0x43464731ull); // "CFG1"
    hash_u64(h, cfg.ipatov_correlation);
    hash_u64(h, cfg.sparse_pulse_reconstruct);
    hash_u64(h, static_cast<uint64_t>(cfg.comm_code));
    hash_u64(h, static_cast<uint64_t>(cfg.sync_reps));
    hash_u64(h, static_cast<uint64_t>(cfg.samples_per_symbol));
    hash_double(h, cfg.sample_rate);
    hash_u64(h, static_cast<uint64_t>(cfg.fit_reps));
    hash_i64(h, cfg.lag_lo);
    hash_u64(h, static_cast<uint64_t>(cfg.tap_count));
    hash_double(h, cfg.peak_floor);
    hash_u64(h, static_cast<uint64_t>(cfg.peak_min_spacing));
    hash_u64(h, static_cast<uint64_t>(cfg.active_radius));
    hash_double(h, cfg.lambda);
    hash_u64(h, static_cast<uint64_t>(cfg.coarse_stride));
    hash_double(h, cfg.detect_threshold);
    hash_double(h, cfg.fit_residual_max);
    hash_double(h, cfg.gain_condition_max);
    return h;
}

// Cache key for one active lag set.  The exact lag check on lookup protects
// against a (negligible but not impossible) 64-bit collision.
uint64_t active_factor_key(uint64_t template_hash,
                           uint64_t config_hash,
                           const int16_t* lags,
                           size_t count,
                           size_t training_rows,
                           double lambda_rel)
{
    uint64_t h = kFnvOffset;
    hash_u64(h, kHashDomain ^ 0x46435431ull); // "FCT1"
    hash_u64(h, template_hash);
    hash_u64(h, config_hash);
    hash_u64(h, static_cast<uint64_t>(training_rows));
    hash_double(h, lambda_rel);
    hash_u64(h, static_cast<uint64_t>(count));
    for (size_t i = 0; i < count; ++i)
        hash_i64(h, static_cast<int64_t>(lags[i]));
    return h;
}

// One-sided cyclic Hermitian Jacobi eigensolver.  Eigenvalues only: no
// eigenvectors are needed.  `a` is row-major K x K Hermitian and is destroyed.
// The matrix is X^H X + lambda I, so the singular values of the regularised
// design are sqrt(eigenvalues); the caller derives cond2(X) and the effective
// rank from them without ever inverting a normal matrix.
bool hermitian_eigenvalues(cd* a, size_t k, double* ev)
{
    if (!a || !ev || k == 0)
        return false;

    for (int sweep = 0; sweep < kJacobiSweeps; ++sweep) {
        double off2 = 0.0;
        double diag2 = 0.0;
        for (size_t r = 0; r < k; ++r) {
            const double d = a[r * k + r].real();
            diag2 += d * d;
            for (size_t c = r + 1; c < k; ++c)
                off2 += std::norm(a[r * k + c]);
        }
        const double scale = std::sqrt(diag2) + 1.0;
        if (std::sqrt(off2) <= 1e-15 * scale)
            break;

        for (size_t p = 0; p < k; ++p) {
            for (size_t q = p + 1; q < k; ++q) {
                const cd apq = a[p * k + q];
                const double ab = std::abs(apq);
                if (ab <= 1e-18 * scale)
                    continue;

                // U = P J0 with P = diag(1, exp(-i phi)) on the (p,q) block.
                // P^H A P makes the off-diagonal real; the real Jacobi
                // rotation then annihilates it.  Only columns p/q change.
                const double phi = std::arg(apq);
                const cd phase = std::polar(1.0, phi);
                const cd bpp = a[p * k + p];
                const cd bqq = a[q * k + q];
                const double tau = (bqq.real() - bpp.real()) / (2.0 * ab);
                double t;
                if (tau >= 0.0)
                    t = 1.0 / (tau + std::sqrt(1.0 + tau * tau));
                else
                    t = -1.0 / (-tau + std::sqrt(1.0 + tau * tau));
                const double c = 1.0 / std::sqrt(1.0 + t * t);
                const double s = t * c;

                for (size_t r = 0; r < k; ++r) {
                    if (r == p || r == q)
                        continue;
                    const cd bpk = a[p * k + r];
                    const cd bqk = phase * a[q * k + r];
                    const cd cpk = c * bpk - s * bqk;
                    const cd cqk = s * bpk + c * bqk;
                    a[p * k + r] = cpk;
                    a[r * k + p] = std::conj(cpk);
                    a[q * k + r] = cqk;
                    a[r * k + q] = std::conj(cqk);
                }
                const double app = bpp.real();
                const double aqq = bqq.real();
                a[p * k + p] = cd(c * c * app - 2.0 * c * s * ab + s * s * aqq, 0.0);
                a[q * k + q] = cd(s * s * app + 2.0 * c * s * ab + c * c * aqq, 0.0);
                a[p * k + q] = cd(0.0, 0.0);
                a[q * k + p] = cd(0.0, 0.0);
            }
        }
    }

    for (size_t i = 0; i < k; ++i) {
        const double v = a[i * k + i].real();
        if (!std::isfinite(v))
            return false;
        ev[i] = v;
    }
    return true;
}

// Lower Cholesky factor A = L L^H, row-major.  A is not modified and L is a
// caller-provided pre-sized buffer (no allocation on the hot path).
bool cholesky_lower(const cd* a, size_t k, cd* l)
{
    if (!a || !l || k == 0)
        return false;
    std::fill(l, l + k * k, cd(0.0, 0.0));
    for (size_t r = 0; r < k; ++r) {
        for (size_t c = 0; c <= r; ++c) {
            cd sum = a[r * k + c];
            for (size_t p = 0; p < c; ++p)
                sum -= l[r * k + p] * std::conj(l[c * k + p]);
            if (r == c) {
                const double d = sum.real();
                if (!(d > 0.0) || !std::isfinite(d))
                    return false;
                l[r * k + r] = cd(std::sqrt(d), 0.0);
            } else {
                const cd pivot = l[c * k + c];
                if (!(std::abs(pivot) > 0.0) || !finite_cd(pivot))
                    return false;
                l[r * k + c] = sum / pivot;
                if (!finite_cd(l[r * k + c]))
                    return false;
            }
        }
    }
    return true;
}

} // namespace

// ---------------------------------------------------------------------------
// reserve: cold capacity setup.  Never called from estimate_discrete_cir().
// ---------------------------------------------------------------------------
void DiscreteCirScratch::reserve(size_t max_window_samples_)
{
    prepared = false;
    max_window_samples = max_window_samples_;
    max_packet_samples = 0;

    template_hash = 0;
    config_hash = 0;
    orig_sync = nullptr;
    orig_train = nullptr;
    sync_len = 0;
    train_len = 0;
    fit_reps = 0;
    sps = 0;
    max_taps = 0;
    max_active = 0;
    lag_lo = 0;
    lag_hi = 0;
    training_rows = 0;
    lambda_rel = 0.0;
    peak_floor = 0.15;
    peak_min_spacing = 5;
    active_radius = 1;

    ipatov_correlation = false;
    sparse_pulse_reconstruct = false;
    impulse_sync.clear();
    sync_norm.clear();
    sync_real.reset();
    train.clear();
    gram.clear();
    factor_cache.clear();
    factor_cache_next = 0;
    energy_prefix.clear();
    cfo_rot.clear();
    z.clear();
    b.clear();
    work.clear();
    waveform.clear();
    scaled_waveform.clear();
    expected_phr.clear();

    if (max_window_samples == 0)
        return;

    factor_cache.resize(kFactorCacheSlots);
    energy_prefix.resize(max_window_samples + 1);
    cfo_rot.resize(max_window_samples);
    z.resize(max_window_samples);
    b.resize(kMaxTaps);
    work.resize(kMaxTaps);
    waveform.resize(max_window_samples);
    scaled_waveform.resize(max_window_samples);
    accumulator.resize(max_window_samples);
    fir_tmp.resize(max_window_samples);
    expected_phr.resize(19);

    modulator.reserve(127, max_window_samples);
}

// ---------------------------------------------------------------------------
// prepare_discrete_cir: may allocate.  Validates the public template/profile,
// copies the finite train, builds the full max-support Gram and pre-sizes the
// fixed factor cache.  Active-set factors are filled lazily by estimate().
// ---------------------------------------------------------------------------
bool prepare_discrete_cir(const DiscreteCirConfig& cfg,
                          const DiscreteCirTemplates& templates,
                          DiscreteCirScratch& scratch,
                          std::string* err)
{
    if (err)
        err->clear();
    scratch.prepared = false;
    scratch.factor_cache_next = 0;
    for (auto& slot : scratch.factor_cache) {
        slot.key = 0;
        slot.count = 0;
        slot.lags.fill(0);
        slot.condition = 0.0;
        slot.effective_rank = 0;
        slot.lambda_abs = 0.0;
    }

    auto fail = [&](const char* msg) {
        if (err)
            *err = msg;
        scratch.prepared = false;
        scratch.factor_cache_next = 0;
        return false;
    };

    // ---- profile / template domain ----------------------------------------
    if (!templates.sync || templates.sync_len == 0 || !templates.train ||
        templates.train_len == 0)
        return fail("null/empty template");
    if (cfg.samples_per_symbol < kMinSps || cfg.samples_per_symbol > kMaxSps)
        return fail("samples_per_symbol outside [64,4096]");
    if (templates.sync_len != cfg.samples_per_symbol)
        return fail("sync_len != samples_per_symbol");
    if (cfg.fit_reps == 0 || cfg.fit_reps > kMaxFitReps)
        return fail("fit_reps outside [1,64]");
    if (cfg.sync_reps == 0)
        return fail("sync_reps == 0");
    if (!(cfg.sample_rate > 0.0) || !std::isfinite(cfg.sample_rate))
        return fail("sample_rate must be finite and > 0");
    if (cfg.tap_count == 0 || cfg.tap_count > kMaxTaps)
        return fail("tap_count outside [1,kDiscreteCirMaxTaps]");
    if (cfg.tap_count >= cfg.samples_per_symbol)
        return fail("tap_count must be < samples_per_symbol");
    if (cfg.lag_lo > 0 ||
        cfg.lag_lo + static_cast<int64_t>(cfg.tap_count) >
            static_cast<int64_t>(cfg.samples_per_symbol))
        return fail("lag geometry outside [-sps,sps)");
    if (cfg.lag_lo < static_cast<int64_t>(std::numeric_limits<int16_t>::min()) ||
        cfg.lag_lo + static_cast<int64_t>(cfg.tap_count) - 1 >
            static_cast<int64_t>(std::numeric_limits<int16_t>::max()))
        return fail("lag geometry does not fit the int16 model");
    if (!std::isfinite(cfg.peak_floor) || cfg.peak_floor < 0.0 ||
        cfg.peak_floor > 1.0)
        return fail("peak_floor outside [0,1]");
    if (cfg.peak_min_spacing == 0 || cfg.peak_min_spacing > cfg.tap_count)
        return fail("peak_min_spacing outside [1,tap_count]");
    if (cfg.active_radius > cfg.tap_count)
        return fail("active_radius exceeds tap_count");
    if (cfg.fit_reps > std::numeric_limits<size_t>::max() / cfg.samples_per_symbol)
        return fail("fit_reps*samples_per_symbol overflow");
    const int64_t fit_span =
        static_cast<int64_t>(cfg.fit_reps * cfg.samples_per_symbol);
    if (cfg.lag_lo <= -fit_span)
        return fail("lag_lo consumes the whole training span");
    if (templates.train_len < static_cast<size_t>(fit_span))
        return fail("train_len < fit_reps*samples_per_symbol");
    if (!std::isfinite(cfg.lambda) || !(cfg.lambda > 0.0))
        return fail("lambda must be finite and > 0");
    if (cfg.coarse_stride == 0)
        return fail("coarse_stride == 0");
    if (!std::isfinite(cfg.detect_threshold) || cfg.detect_threshold < 0.0 ||
        cfg.detect_threshold > 1.0)
        return fail("detect_threshold outside [0,1]");
    if (!std::isfinite(cfg.fit_residual_max) || !(cfg.fit_residual_max > 0.0) ||
        cfg.fit_residual_max > 1.0)
        return fail("fit_residual_max outside (0,1]");
    if (!std::isfinite(cfg.gain_condition_max) ||
        !(cfg.gain_condition_max > 0.0))
        return fail("gain_condition_max must be finite and > 0");
    if (cfg.max_window_samples == 0)
        return fail("max_window_samples == 0");

    // ---- capacity ---------------------------------------------------------
    if (scratch.max_window_samples < cfg.max_window_samples ||
        scratch.energy_prefix.size() < cfg.max_window_samples + 1 ||
        scratch.cfo_rot.size() < cfg.max_window_samples ||
        scratch.z.size() < cfg.max_window_samples ||
        scratch.waveform.size() < cfg.max_window_samples ||
        scratch.scaled_waveform.size() < cfg.max_window_samples ||
        scratch.accumulator.size() < cfg.max_window_samples ||
        scratch.fir_tmp.size() < cfg.max_window_samples ||
        scratch.b.size() < kMaxActive || scratch.work.size() < kMaxActive ||
        scratch.factor_cache.size() < kFactorCacheSlots)
        scratch.reserve(cfg.max_window_samples);

    const size_t sps = cfg.samples_per_symbol;
    const size_t max_taps = cfg.tap_count;
    const size_t max_active = std::min(max_taps, kMaxActive);
    const size_t train_len = templates.train_len;
    const int64_t lag_lo = cfg.lag_lo;
    const int64_t lag_hi = lag_lo + static_cast<int64_t>(max_taps);
    const size_t training_rows =
        static_cast<size_t>(fit_span + lag_lo); // lag_lo <= 0
    if (training_rows == 0)
        return fail("training_rows == 0");

    // ---- copy / normalise the public templates ----------------------------
    scratch.sync_norm.assign(templates.sync, templates.sync + sps);
    double sync_energy = 0.0;
    for (size_t i = 0; i < sps; ++i) {
        if (!finite_cf(scratch.sync_norm[i]))
            return fail("sync contains non-finite samples");
        sync_energy += std::norm(scratch.sync_norm[i]);
    }
    if (!(sync_energy > 0.0) || !std::isfinite(sync_energy))
        return fail("sync energy == 0");
    const float sync_inv = static_cast<float>(1.0 / std::sqrt(sync_energy));
    for (size_t i = 0; i < sps; ++i)
        scratch.sync_norm[i] *= sync_inv;

    // ---- prepare real-valued sync template if imaginary part is zero ------
    scratch.sync_real.reset();
    bool is_real = true;
    for (size_t i = 0; i < sps; ++i) {
        if (std::abs(scratch.sync_norm[i].imag()) > 1e-6f) {
            is_real = false;
            break;
        }
    }
    scratch.sync_real.is_real = is_real;
    scratch.sync_real.length = sps;
    if (is_real) {
        scratch.sync_real.taps_dup.resize(2 * sps);
        float e = 0.0f;
        for (size_t i = 0; i < sps; ++i) {
            const float r = scratch.sync_norm[i].real();
            scratch.sync_real.taps_dup[2 * i] = r;
            scratch.sync_real.taps_dup[2 * i + 1] = r;
            e += r * r;
        }
        scratch.sync_real.energy = e;
    }

    scratch.train.assign(templates.train, templates.train + train_len);
    for (size_t i = 0; i < train_len; ++i) {
        if (!finite_cf(scratch.train[i]))
            return fail("train contains non-finite samples");
    }

    // ---- metadata ---------------------------------------------------------
    scratch.template_hash =
        template_identity(templates.sync, sps, templates.train, train_len);
    scratch.config_hash = config_identity(cfg);
    scratch.orig_sync = static_cast<const void*>(templates.sync);
    scratch.orig_train = static_cast<const void*>(templates.train);
    scratch.sync_len = sps;
    scratch.train_len = train_len;
    scratch.fit_reps = cfg.fit_reps;
    scratch.sps = sps;
    scratch.max_taps = max_taps;
    scratch.max_active = max_active;
    scratch.lag_lo = lag_lo;
    scratch.lag_hi = lag_hi;
    scratch.training_rows = training_rows;
    scratch.lambda_rel = cfg.lambda;
    scratch.peak_floor = cfg.peak_floor;
    scratch.peak_min_spacing = cfg.peak_min_spacing;
    scratch.active_radius = cfg.active_radius;

    // Capacity for the training target.  z is sized independently of the
    // window because fit_reps*sps can be smaller than the benchmark window.
    if (scratch.z.size() < training_rows)
        scratch.z.resize(training_rows);
    if (scratch.b.size() < max_active)
        scratch.b.resize(max_active);
    if (scratch.work.size() < max_active * max_active)
        scratch.work.resize(max_active * max_active);
    if (scratch.work.size() < max_taps)
        scratch.work.resize(max_taps);

    // Fixed-size factor cache.  Every slot owns a max_active^2 buffer so the
    // hot path can fill a round-robin slot without allocating.
    if (scratch.factor_cache.size() < kFactorCacheSlots)
        scratch.factor_cache.resize(kFactorCacheSlots);
    for (auto& slot : scratch.factor_cache) {
        slot.key = 0;
        slot.count = 0;
        slot.lags.fill(0);
        slot.condition = 0.0;
        slot.effective_rank = 0;
        slot.lambda_abs = 0.0;
        if (slot.chol.size() < max_active * max_active)
            slot.chol.resize(max_active * max_active);
        std::fill(slot.chol.begin(), slot.chol.end(), cd(0.0, 0.0));
    }
    scratch.factor_cache_next = 0;

    // Safe upper bound for the remodulated packet.  The frozen experiment
    // profile uses configured SYNC / 8 SFD / 127 B / legacy pulse; +64 covers shaping
    // and the modulator's convention.  A public capacity never exceeds the
    // prepared window.
    const mod::PulseSpec default_pulse{};
    size_t max_packet = cfg.max_packet_samples;
    if (max_packet == 0) {
        max_packet = mod::packet_samples_998p4(cfg.sync_reps, 8, 127, false,
                                               &default_pulse) +
                     64;
    }
    max_packet = std::min(max_packet, cfg.max_window_samples);
    if (max_packet > scratch.waveform.size())
        scratch.waveform.resize(max_packet);
    if (max_packet > scratch.scaled_waveform.size())
        scratch.scaled_waveform.resize(max_packet);
    scratch.max_packet_samples = max_packet;

    scratch.ipatov_correlation = cfg.ipatov_correlation;
    scratch.sparse_pulse_reconstruct = false;
    if (cfg.sparse_pulse_reconstruct && !cfg.ipatov_correlation)
        return fail("sparse_pulse_requires_ipatov");
    if (cfg.ipatov_correlation) {
        if (sps != demod::kQm35SamplesPerSymbol || cfg.fit_reps < 3 ||
            cfg.sample_rate != radar_meta::kWorkRateHz ||
            (cfg.sync_reps != 64 && cfg.sync_reps != 256))
            return fail("unsupported Ipatov geometry");
        if (!radar_meta::code_index_supported(cfg.comm_code))
            return fail("unsupported Ipatov code");
        const int8_t* code = demod::GetPreambleCode(cfg.comm_code);
        if (!code) return fail("unsupported Ipatov code");
        scratch.impulse_sync.assign(sps, cf{});
        for (size_t c = 0; c < demod::kQm35CodeLength; ++c)
            scratch.impulse_sync[8*c] = cf(float(code[c]), 0.f);
        demod::rxopt::build_sparse_cir_template(
            code, demod::kQm35CodeLength, scratch.cir_sparse_tmpl);
        scratch.cir_avg_cf.assign(sps + max_taps, cf{});
        scratch.cir_corr_vals.assign(max_taps, cf{});
        scratch.impulse_profile = mod::HrpModConfig{};
        auto& raw = scratch.impulse_profile;
        raw.code_index = cfg.comm_code;
        raw.sync_repetitions = cfg.sync_reps;
        raw.sfd_mode = "4z2";
        raw.peak_amplitude = 1.f;
        raw.pulse.shape = mod::PulseShape::External;
        raw.external_taps = {1.f, 0.f}; // unit impulses, 2 samples/chip
        raw.pulse.external_n_taps = 2;
        // Ensure pulse storage/cache allocation happens only in preparation.
        if (!mod::ensure_pulse(raw, scratch.modulator))
            return fail("impulse pulse preparation failed");
        scratch.sparse_pulse_reconstruct = cfg.sparse_pulse_reconstruct;
        if (cfg.sparse_pulse_reconstruct) {
            const size_t max_events =
                max_packet / demod::kQm35SamplesPerChip + 2;
            if (scratch.modulator.pulse_events.capacity() < max_events ||
                scratch.modulator.prefix.events.capacity() < max_events)
                return fail("sparse_pulse_event_capacity");
        }
        scratch.prepared = true;
        return true;
    }

    // ---- full max-support Gram over the legal training rows --------------
    scratch.gram.assign(max_taps * max_taps, cd(0.0, 0.0));
    const int64_t train_len_i = static_cast<int64_t>(train_len);
    const int64_t rows_i = static_cast<int64_t>(training_rows);
    for (size_t i = 0; i < max_taps; ++i) {
        const int64_t li = lag_lo + static_cast<int64_t>(i);
        for (size_t j = i; j < max_taps; ++j) {
            const int64_t lj = lag_lo + static_cast<int64_t>(j);
            cd acc(0.0, 0.0);
            for (int64_t m = 0; m < rows_i; ++m) {
                const int64_t xi = m - li;
                const int64_t xj = m - lj;
                if (xi < 0 || xi >= train_len_i || xj < 0 ||
                    xj >= train_len_i)
                    continue;
                acc += std::conj(cd(scratch.train[static_cast<size_t>(xi)])) *
                       cd(scratch.train[static_cast<size_t>(xj)]);
            }
            if (i == j)
                acc = cd(acc.real(), 0.0);
            scratch.gram[i * max_taps + j] = acc;
            scratch.gram[j * max_taps + i] = std::conj(acc);
        }
    }

    double trace = 0.0;
    for (size_t i = 0; i < max_taps; ++i)
        trace += scratch.gram[i * max_taps + i].real();
    if (!(trace > 0.0) || !std::isfinite(trace))
        return fail("template training Gram has no energy");
    const double lambda_abs =
        cfg.lambda * (trace / static_cast<double>(max_taps));
    if (!(lambda_abs > 0.0) || !std::isfinite(lambda_abs))
        return fail("lambda_abs is not finite/positive");

    scratch.prepared = true;
    return true;
}

// ---------------------------------------------------------------------------
// estimate_discrete_cir: strict-prefix, no allocation/growth.
// ---------------------------------------------------------------------------
bool estimate_discrete_cir(const std::complex<float>* y,
                           size_t n,
                           int64_t observed_end,
                           const DiscreteCirTemplates& templates,
                           const DiscreteCirConfig& cfg,
                           DiscreteCirScratch& scratch,
                           DiscreteCirModel& model,
                           DiscreteCirTimings* timings)
{
    const auto t_outer = Clock::now();
    if (timings)
        *timings = DiscreteCirTimings{};
    model = DiscreteCirModel{};

    auto finish = [&](const char* status, const char* reason) {
        model.valid = false;
        model.status = status;
        model.reason = reason;
        if (timings)
            timings->estimate_total_us = us_since(t_outer);
        return false;
    };
    auto succeed = [&]() {
        model.valid = true;
        model.status = "model_ready";
        model.reason = nullptr;
        if (timings)
            timings->estimate_total_us = us_since(t_outer);
        return true;
    };

    // ---- 0. prepared identity and input domain ----------------------------
    if (!scratch.prepared)
        return finish("failed", "unprepared_scratch");
    if (!y || n == 0)
        return finish("failed", "invalid_input");
    if (!templates.sync || !templates.train)
        return finish("failed", "null_template");
    if (templates.sync_len != scratch.sync_len ||
        templates.sync != scratch.orig_sync)
        return finish("failed", "prepared_sync_mismatch");
    if (templates.train_len != scratch.train_len ||
        templates.train != scratch.orig_train)
        return finish("failed", "prepared_train_mismatch");
    if (cfg.samples_per_symbol != scratch.sps ||
        cfg.fit_reps != scratch.fit_reps || cfg.lag_lo != scratch.lag_lo ||
        cfg.tap_count != scratch.max_taps ||
        cfg.peak_floor != scratch.peak_floor ||
        cfg.peak_min_spacing != scratch.peak_min_spacing ||
        cfg.active_radius != scratch.active_radius)
        return finish("failed", "prepared_geometry_mismatch");
    if (!(cfg.lambda > 0.0) || !std::isfinite(cfg.lambda) ||
        cfg.lambda != scratch.lambda_rel)
        return finish("failed", "prepared_lambda_mismatch");
    if (!std::isfinite(cfg.detect_threshold) || cfg.detect_threshold < 0.0 ||
        cfg.detect_threshold > 1.0)
        return finish("failed", "invalid_detect_threshold");
    if (!std::isfinite(cfg.fit_residual_max) || !(cfg.fit_residual_max > 0.0) ||
        cfg.fit_residual_max > 1.0)
        return finish("failed", "invalid_fit_residual_max");
    if (!std::isfinite(cfg.gain_condition_max) ||
        !(cfg.gain_condition_max > 0.0))
        return finish("failed", "invalid_gain_condition_max");
    if (cfg.coarse_stride == 0)
        return finish("failed", "invalid_coarse_stride");
    if (!std::isfinite(cfg.sample_rate) || !(cfg.sample_rate > 0.0))
        return finish("failed", "invalid_sample_rate");
    if (!std::isfinite(cfg.peak_floor) || cfg.peak_floor < 0.0 ||
        cfg.peak_floor > 1.0)
        return finish("failed", "invalid_peak_floor");
    if (cfg.peak_min_spacing == 0)
        return finish("failed", "invalid_peak_min_spacing");
    if (config_identity(cfg) != scratch.config_hash)
        return finish("failed", "prepared_config_mismatch");
    if (n > scratch.max_window_samples)
        return finish("failed", "scratch_capacity");
    if (scratch.sync_norm.size() < scratch.sps ||
        scratch.train.size() < scratch.train_len ||
        scratch.energy_prefix.size() < static_cast<size_t>(n) + 1 ||
        scratch.z.size() < scratch.training_rows ||
        scratch.b.size() < scratch.max_active ||
        scratch.work.size() < scratch.max_active * scratch.max_active ||
        scratch.factor_cache.empty() ||
        scratch.factor_cache.size() < kFactorCacheSlots)
        return finish("failed", "scratch_capacity");

    if (observed_end < 0)
        return finish("failed", "invalid_observed_end");
    const int64_t n_i64 = static_cast<int64_t>(n);
    const int64_t visible_end =
        (observed_end == 0) ? n_i64 : std::min(n_i64, observed_end);
    model.observed_end_sample = visible_end;
    model.template_hash = scratch.template_hash;
    model.config_hash = scratch.config_hash;
    if (visible_end <= 0)
        return finish("failed", "invalid_observed_end");

    const size_t Ls = scratch.sync_len;
    const size_t sps = scratch.sps;
    const size_t fit_reps = scratch.fit_reps;
    const size_t confirm_reps = fit_reps;
    const double fs = cfg.sample_rate;
    const double detect_thr = cfg.detect_threshold;
    const cf* const syncn = scratch.sync_norm.data();
    double* const energy = scratch.energy_prefix.data();

    // ---- finite prefix check with AVX2 scan ------------------------------
    if (!scan_finite(y, static_cast<size_t>(visible_end)))
        return finish("failed", "nonfinite_input");

    int64_t q = -1;
    double cfo = 0.0;
    const bool use_known_origin =
        (cfg.ipatov_correlation && cfg.known_q >= 0 && cfg.has_known_cfo);

    if (use_known_origin) {
        if (cfg.known_q + static_cast<int64_t>(Ls) > visible_end ||
            !std::isfinite(cfg.known_cfo_hz)) {
            return finish("failed", "invalid_known_origin_or_cfo");
        }
        q = cfg.known_q;
        cfo = cfg.known_cfo_hz;
        model.q = q;
        model.phase_reference_sample = q;
        model.cfo_hz = cfo;
        model.coarse_peak = 1.0;
        model.detection_metric = 1.0;
        const int64_t avail_reps =
            (visible_end > q) ? ((visible_end - q) / static_cast<int64_t>(sps)) : 0;
        model.repeats_confirmed =
            static_cast<size_t>(std::min<int64_t>(confirm_reps, avail_reps));
    } else {
        // ---- 1. public ROI clipped to the visible prefix ------------------
        int64_t roi_lo = cfg.roi_lo;
        int64_t roi_hi = cfg.roi_hi;
        if (roi_hi <= roi_lo) {
            roi_lo = 0;
            roi_hi = visible_end;
        }
        roi_hi = std::min<int64_t>(
            roi_hi, visible_end - static_cast<int64_t>(Ls) + 1);
        roi_lo = std::max<int64_t>(roi_lo, 0);
        if (roi_hi <= roi_lo)
            return finish("failed", "invalid_roi");
        const int64_t stride =
            std::max<int64_t>(1, static_cast<int64_t>(cfg.coarse_stride));

        // ---- localized self-energy prefix sum covering ROI and confirmation
        const int64_t energy_base = roi_lo;
        const int64_t max_confirm_end =
            roi_hi + static_cast<int64_t>((confirm_reps - 1) * sps + Ls);
        const int64_t energy_end =
            std::max(energy_base, std::min<int64_t>(visible_end, max_confirm_end));
        const size_t local_energy_len =
            static_cast<size_t>(energy_end - energy_base);

        double acc = 0.0;
        energy[0] = 0.0;
        for (size_t i = 0; i < local_energy_len; ++i) {
            const cf v = y[static_cast<size_t>(energy_base) + i];
            acc += static_cast<double>(v.real()) * static_cast<double>(v.real()) +
                   static_cast<double>(v.imag()) * static_cast<double>(v.imag());
            energy[i + 1] = acc;
        }
        if (!std::isfinite(acc))
            return finish("failed", "nonfinite_energy");

        auto get_energy = [&](int64_t d, size_t L) -> double {
            if (d >= energy_base &&
                d + static_cast<int64_t>(L) <= energy_end) {
                const size_t lo = static_cast<size_t>(d - energy_base);
                const size_t hi =
                    static_cast<size_t>(d + static_cast<int64_t>(L) - energy_base);
                return std::sqrt(std::max(0.0, energy[hi] - energy[lo]));
            }
            if (d < 0 || d + static_cast<int64_t>(L) > visible_end)
                return 0.0;
            double sum = 0.0;
            for (size_t k = 0; k < L; ++k) {
                const cf v = y[static_cast<size_t>(d) + k];
                sum += static_cast<double>(v.real()) * static_cast<double>(v.real()) +
                       static_cast<double>(v.imag()) * static_cast<double>(v.imag());
            }
            return std::sqrt(std::max(0.0, sum));
        };

        int64_t last_norm_d = -1;
        double last_norm_val = 0.0;
        cf last_norm_corr(0.f, 0.f);
        bool last_norm_valid = false;

        auto norm_at = [&](int64_t d, cf* corr_out) -> double {
            if (last_norm_valid && d == last_norm_d) {
                if (corr_out)
                    *corr_out = last_norm_corr;
                return last_norm_val;
            }
            // STRICT PREFIX: the whole correlation row must be visible.
            if (d < 0 || d + static_cast<int64_t>(Ls) > visible_end) {
                if (corr_out)
                    *corr_out = cf(0.f, 0.f);
                return 0.0;
            }
            const cf* const seg = y + d;
            cf dot(0.f, 0.f);
            if (scratch.sync_real.is_real) {
                dot = demod::rxopt::real_template_dot_single(
                    seg, scratch.sync_real.taps_dup.data(), Ls);
            } else {
                volk_32fc_x2_conjugate_dot_prod_32fc_u(
                    &dot, seg, syncn, static_cast<unsigned>(Ls));
            }
            const double e = get_energy(d, Ls);
            const double nrm = (e > 0.0) ? static_cast<double>(std::abs(dot)) / e : 0.0;
            if (corr_out)
                *corr_out = dot;
            last_norm_d = d;
            last_norm_val = nrm;
            last_norm_corr = dot;
            last_norm_valid = true;
            return nrm;
        };

        // ---- 2. detection (independent copy of the frozen profile semantics)
        int64_t coarse_pos = -1;
        double coarse_nrm = 0.0;
        int64_t detected_start = -1;
        double peak_norm = 0.0;
        {
            const auto t0 = Clock::now();
            if (stride == 1 && scratch.sync_real.is_real) {
                int64_t d = roi_lo;
                const float* const taps_dup = scratch.sync_real.taps_dup.data();
                for (; d + 4 <= roi_hi; d += 4) {
                    if (d < 0 || d + 3 + static_cast<int64_t>(Ls) > visible_end) {
                        break;
                    }
                    cf dots[4];
                    demod::rxopt::real_template_dot_4lag(y + d, taps_dup, Ls, dots);
                    for (int j = 0; j < 4; ++j) {
                        const int64_t cur_d = d + j;
                        const double e = get_energy(cur_d, Ls);
                        const double nrm =
                            (e > 0.0) ? static_cast<double>(std::abs(dots[j])) / e : 0.0;
                        if (nrm > coarse_nrm) {
                            coarse_nrm = nrm;
                            coarse_pos = cur_d;
                        }
                    }
                }
                for (; d < roi_hi; ++d) {
                    const double nrm = norm_at(d, nullptr);
                    if (nrm > coarse_nrm) {
                        coarse_nrm = nrm;
                        coarse_pos = d;
                    }
                }
            } else {
                for (int64_t d = roi_lo; d < roi_hi; d += stride) {
                    const double nrm = norm_at(d, nullptr);
                    if (nrm > coarse_nrm) {
                        coarse_nrm = nrm;
                        coarse_pos = d;
                    }
                }
            }

            detected_start = coarse_pos;
            peak_norm = coarse_nrm;
            if (coarse_pos >= 0) {
                const int64_t lo = std::max(roi_lo, coarse_pos - stride);
                const int64_t hi = std::min(roi_hi, coarse_pos + stride + 1);
                for (int64_t d = lo; d < hi; ++d) {
                    const double nrm = norm_at(d, nullptr);
                    if (nrm > peak_norm) {
                        peak_norm = nrm;
                        detected_start = d;
                    }
                }
            }

            if (detected_start >= 0) {
                int64_t origin = detected_start;
                const int64_t floor_lo = std::max<int64_t>(roi_lo, 0);
                while (origin - static_cast<int64_t>(sps) >= floor_lo) {
                    const int64_t d = origin - static_cast<int64_t>(sps);
                    const double nrm = norm_at(d, nullptr);
                    if (nrm >= detect_thr)
                        origin = d;
                    else
                        break;
                }
                const int64_t lo = std::max(roi_lo, origin - stride);
                const int64_t hi = std::min(roi_hi, origin + stride + 1);
                for (int64_t d = lo; d < hi; ++d) {
                    const double nrm = norm_at(d, nullptr);
                    if (nrm > peak_norm) {
                        origin = d;
                        peak_norm = nrm;
                    }
                }
                detected_start = origin;
            }

            model.coarse_peak = coarse_nrm;
            model.detection_metric = peak_norm;
            if (timings)
                timings->detect_us = us_since(t0);
        }

        if (detected_start < 0 || peak_norm < detect_thr)
            return finish("bypassed", "below_detect_threshold");

        q = detected_start;
        model.q = q;
        model.phase_reference_sample = q;

        // ---- 3. repetition confirmation ----------------------------------
        cf rep_corr[kMaxFitReps];
        double rep_norms[kMaxFitReps];
        size_t confirmed = 0;
        for (size_t k = 0; k < confirm_reps; ++k) {
            const int64_t d = q + static_cast<int64_t>(k * sps);
            cf c(0.f, 0.f);
            const double nrm =
                (d + static_cast<int64_t>(Ls) <= visible_end)
                    ? norm_at(d, &c)
                    : 0.0;
            rep_corr[k] = c;
            rep_norms[k] = nrm;
            if (nrm >= detect_thr)
                ++confirmed;
        }
        model.repeats_confirmed = confirmed;
        if (confirmed < 2)
            return finish("bypassed", "repetition_not_confirmed");

        // ---- 4. CFO from adjacent training-peak phases -------------------
        {
            const auto t0 = Clock::now();
            cd sum(0.0, 0.0);
            size_t cnt = 0;
            const size_t k_lo = (fit_reps >= 3) ? 1 : 0;
            for (size_t k = k_lo; k + 1 < fit_reps; ++k) {
                if (rep_norms[k] >= detect_thr &&
                    rep_norms[k + 1] >= detect_thr) {
                    cd t = cd(rep_corr[k + 1]) * std::conj(cd(rep_corr[k]));
                    const double a = std::abs(t);
                    if (a > 0.0) {
                        sum += t / a;
                        ++cnt;
                    }
                }
            }
            if (cnt > 0) {
                const double dphi = std::arg(sum);
                cfo = dphi / (2.0 * kPi * static_cast<double>(sps) / fs);
            }
            if (timings)
                timings->cfo_us = us_since(t0);
        }
        if (!std::isfinite(cfo))
            return finish("failed", "nonfinite_cfo");
        model.cfo_hz = cfo;
    }

    if (cfg.ipatov_correlation) {
        const auto t_support = Clock::now();
        if (!scratch.ipatov_correlation || scratch.impulse_sync.size() != sps)
            return finish("failed", "impulse_template_unprepared");
        const size_t taps = cfg.tap_count;
        const size_t width = sps + taps - 1;
        std::array<cd, kMaxSps + kMaxTaps> avg{};
        size_t valid = 0;
        const double omega = 2.0*kPi*cfo/fs;
        // Skip the first finite-train transient. Every accepted window is
        // wholly inside both the first fit_reps and the visible IQ prefix.
        const int64_t training_end = std::min(visible_end,
                              q + int64_t(cfg.fit_reps*sps));
        const size_t rows_train = (training_end > q) ? size_t(training_end - q) : 0;
        const size_t max_rot_len = std::max(rows_train, cfg.fit_reps * sps);
        if (cfo != 0.0) {
            build_cfo_rotation(cfo, fs, max_rot_len, scratch.cfo_rot);
        }

        for (size_t k = 1; k < cfg.fit_reps; ++k) {
            const int64_t lo = q + int64_t(k * sps) + cfg.lag_lo;
            if (lo < 0 || lo + int64_t(width) > training_end) continue;
            const int64_t rel_lo = lo - q;
            if (cfo == 0.0) {
                for (size_t j = 0; j < width; ++j) {
                    avg[j] += cd(y[size_t(lo) + j]);
                }
            } else if (rel_lo >= 0 && size_t(rel_lo + width) <= scratch.cfo_rot.size()) {
                const cd* const rot_base = scratch.cfo_rot.data() + rel_lo;
                size_t j = 0;
                const size_t lim = (width >= 4) ? (width - 3) : 0;
                for (; j < lim; j += 4) {
                    avg[j + 0] += cd(y[size_t(lo) + j + 0]) * rot_base[j + 0];
                    avg[j + 1] += cd(y[size_t(lo) + j + 1]) * rot_base[j + 1];
                    avg[j + 2] += cd(y[size_t(lo) + j + 2]) * rot_base[j + 2];
                    avg[j + 3] += cd(y[size_t(lo) + j + 3]) * rot_base[j + 3];
                }
                for (; j < width; ++j) {
                    avg[j] += cd(y[size_t(lo) + j]) * rot_base[j];
                }
            } else {
                cd phase = std::polar(1.0, -omega * double(rel_lo));
                const cd step = std::polar(1.0, -omega);
                for (size_t j = 0; j < width; ++j) {
                    if ((j & 1023u) == 0)
                        phase = std::polar(1.0, -omega * double(rel_lo + int64_t(j)));
                    avg[j] += cd(y[size_t(lo) + j]) * phase;
                    phase *= step;
                }
            }
            ++valid;
        }
        if (!valid) return finish("failed", "short_correlation_prefix");
        if (scratch.cir_sparse_tmpl.code_energy <= 0.0f)
            return finish("failed", "empty_impulse_code");
        const float inv_valid = 1.0f / static_cast<float>(valid);
        for (size_t j = 0; j < width; ++j)
            scratch.cir_avg_cf[j] = cf(avg[j]);
        demod::rxopt::correlate_cir_sparse(
            scratch.cir_avg_cf.data(), scratch.cir_sparse_tmpl, taps, scratch.cir_corr_vals);
        for (size_t d = 0; d < taps; ++d) {
            model.taps[d] = scratch.cir_corr_vals[d] * inv_valid;
            if (!finite_cf(model.taps[d]))
                return finish("failed", "nonfinite_correlation");
            model.active_lags[d] = int16_t(cfg.lag_lo + int64_t(d));
        }
        model.impulse_template=true;
        model.cir_repetitions=valid;
        model.lag_lo=cfg.lag_lo;
        model.tap_count=model.active_count=taps;
        model.support_source="ipatov_correlation";
        model.training_lo=q;
        model.training_hi=training_end;
        if (timings) timings->support_us=us_since(t_support);

        // Validate the frozen float response against UNIT impulse training.
        // No projection, Gram solve or re-fit. Keep physical amplitude/phase.
        const auto t_fit=Clock::now();
        const size_t rows=size_t(training_end-q);
        if (scratch.z.size()<rows) return finish("failed","scratch_capacity");

        bool use_periodic = (sps == 1016 && rows == cfg.fit_reps * sps && cfg.fit_reps >= 2);
        int64_t p_min = std::numeric_limits<int64_t>::max();
        int64_t p_max = std::numeric_limits<int64_t>::min();
        if (use_periodic) {
            for (size_t c = 0; c < demod::kQm35CodeLength; ++c) {
                const float s = scratch.impulse_sync[8 * c].real();
                if (s != 0.f) {
                    const int64_t p = int64_t(8 * c) + cfg.lag_lo;
                    p_min = std::min(p_min, p);
                    p_max = std::max(p_max, p + int64_t(taps) - 1);
                }
            }
            if (p_min > p_max || p_min < -int64_t(sps) || p_max >= 2 * int64_t(sps)) {
                use_periodic = false;
            }
        }

        if (use_periodic) {
            const int64_t spill_head_hi = std::max<int64_t>(0, p_max - int64_t(sps) + 1);
            const int64_t spill_tail_lo = (p_min < 0) ? (int64_t(sps) + p_min) : int64_t(sps);
            alignas(64) std::array<cd, 1016> z_per{};
            alignas(64) std::array<cd, 1016> head_spill{};
            alignas(64) std::array<cd, 1016> tail_spill{};
            for (size_t c = 0; c < demod::kQm35CodeLength; ++c) {
                const double sign = scratch.impulse_sync[8 * c].real();
                if (sign == 0.0) continue;
                const int64_t pos = int64_t(8 * c) + cfg.lag_lo;
                for (size_t d = 0; d < taps; ++d) {
                    const int64_t p = pos + int64_t(d);
                    const cd term = sign * cd(model.taps[d]);
                    if (p >= 0 && p < int64_t(sps)) {
                        z_per[size_t(p)] += term;
                    } else if (p < 0) {
                        const size_t t = size_t(p + int64_t(sps));
                        z_per[t] += term;
                        tail_spill[t] += term;
                    } else {
                        const size_t t = size_t(p - int64_t(sps));
                        z_per[t] += term;
                        head_spill[t] += term;
                    }
                }
            }
            for (size_t k = 0; k < cfg.fit_reps; ++k) {
                std::memcpy(scratch.z.data() + k * sps, z_per.data(), sps * sizeof(cd));
            }
            for (int64_t t = 0; t < spill_head_hi; ++t) {
                scratch.z[size_t(t)] -= head_spill[size_t(t)];
            }
            const size_t last_sym_base = (cfg.fit_reps - 1) * sps;
            for (int64_t t = spill_tail_lo; t < int64_t(sps); ++t) {
                scratch.z[last_sym_base + size_t(t)] -= tail_spill[size_t(t)];
            }
        } else {
            std::fill(scratch.z.begin(), scratch.z.begin() + rows, cd{});
            for (size_t k = 0; k < cfg.fit_reps; ++k) {
                for (size_t c = 0; c < demod::kQm35CodeLength; ++c) {
                    const double sign = scratch.impulse_sync[8 * c].real();
                    if (sign == 0) continue;
                    const int64_t pos = int64_t(k * sps + 8 * c) + cfg.lag_lo;
                    for (size_t d = 0; d < taps; ++d) {
                        const int64_t j = pos + int64_t(d);
                        if (j >= 0 && j < int64_t(rows))
                            scratch.z[size_t(j)] += sign * cd(model.taps[d]);
                    }
                }
            }
        }
        if (cfo == 0.0) {
            double num0 = 0.0, num1 = 0.0, num2 = 0.0, num3 = 0.0;
            double den0 = 0.0, den1 = 0.0, den2 = 0.0, den3 = 0.0;
            size_t m = 0;
            const size_t limit = (rows >= 4) ? (rows - 3) : 0;
            for (; m < limit; m += 4) {
                const cd obs0(y[size_t(q) + m]);
                const cd obs1(y[size_t(q) + m + 1]);
                const cd obs2(y[size_t(q) + m + 2]);
                const cd obs3(y[size_t(q) + m + 3]);
                const cd diff0 = obs0 - scratch.z[m];
                const cd diff1 = obs1 - scratch.z[m + 1];
                const cd diff2 = obs2 - scratch.z[m + 2];
                const cd diff3 = obs3 - scratch.z[m + 3];
                num0 += std::norm(diff0);
                num1 += std::norm(diff1);
                num2 += std::norm(diff2);
                num3 += std::norm(diff3);
                den0 += std::norm(obs0);
                den1 += std::norm(obs1);
                den2 += std::norm(obs2);
                den3 += std::norm(obs3);
            }
            double num = (num0 + num1) + (num2 + num3);
            double den = (den0 + den1) + (den2 + den3);
            for (; m < rows; ++m) {
                const cd obs(y[size_t(q) + m]);
                num += std::norm(obs - scratch.z[m]);
                den += std::norm(obs);
            }
            model.fit_residual = den > 0 ? num / den : std::numeric_limits<double>::infinity();
        } else {
            const cd* const rot = scratch.cfo_rot.data();
            double num0 = 0.0, num1 = 0.0, num2 = 0.0, num3 = 0.0;
            double den0 = 0.0, den1 = 0.0, den2 = 0.0, den3 = 0.0;
            size_t m = 0;
            const size_t limit = (rows >= 4) ? (rows - 3) : 0;
            for (; m < limit; m += 4) {
                const cd obs0(y[size_t(q) + m]);
                const cd obs1(y[size_t(q) + m + 1]);
                const cd obs2(y[size_t(q) + m + 2]);
                const cd obs3(y[size_t(q) + m + 3]);
                const cd diff0 = obs0 - scratch.z[m] * std::conj(rot[m]);
                const cd diff1 = obs1 - scratch.z[m + 1] * std::conj(rot[m + 1]);
                const cd diff2 = obs2 - scratch.z[m + 2] * std::conj(rot[m + 2]);
                const cd diff3 = obs3 - scratch.z[m + 3] * std::conj(rot[m + 3]);
                num0 += std::norm(diff0);
                num1 += std::norm(diff1);
                num2 += std::norm(diff2);
                num3 += std::norm(diff3);
                den0 += std::norm(obs0);
                den1 += std::norm(obs1);
                den2 += std::norm(obs2);
                den3 += std::norm(obs3);
            }
            double num = (num0 + num1) + (num2 + num3);
            double den = (den0 + den1) + (den2 + den3);
            for (; m < rows; ++m) {
                const cd obs(y[size_t(q) + m]);
                num += std::norm(obs - scratch.z[m] * std::conj(rot[m]));
                den += std::norm(obs);
            }
            model.fit_residual = den > 0 ? num / den : std::numeric_limits<double>::infinity();
        }
        if(timings) timings->fit_check_us=us_since(t_fit);
        if(!std::isfinite(model.fit_residual))
            return finish("failed","nonfinite_fit_residual");
        if(model.fit_residual>cfg.fit_residual_max)
            return finish("rejected","fit_residual_exceeded");
    uint64_t id = kFnvOffset;
    hash_u64(id, kHashDomain ^ 0x4d4f4431ull); // "MOD1"
    hash_u64(id, model.impulse_template);
    hash_u64(id, model.cir_repetitions);
    hash_i64(id, model.q);
    hash_double(id, model.cfo_hz);
    hash_i64(id, model.observed_end_sample);
    hash_i64(id, model.lag_lo);
    hash_u64(id, static_cast<uint64_t>(model.tap_count));
    hash_double(id, model.lambda);
    for (size_t i = 0; i < model.tap_count; ++i) {
        hash_float(id, model.taps[i].real());
        hash_float(id, model.taps[i].imag());
    }
    hash_double(id, model.fit_residual);
    hash_double(id, model.gain_condition);
    hash_u64(id, static_cast<uint64_t>(model.active_count));
    for (size_t i = 0; i < model.active_count; ++i)
        hash_i64(id, static_cast<int64_t>(model.active_lags[i]));
    hash_u64(id, model.template_hash);
    hash_u64(id, model.config_hash);
    model.model_id = id;

        return succeed();
    }

    // ---- 5. de-CFO training prefix, folded coarse CIR, peak groups -------
    const size_t M = scratch.training_rows;
    const int16_t* active_lags = nullptr;
    size_t active_count = 0;
    size_t hull_width = 0;
    int64_t hull_start = 0;
    {
        const auto t0 = Clock::now();
        if (q < 0 || M == 0 || q + static_cast<int64_t>(M) > visible_end) {
            if (timings)
                timings->support_us = us_since(t0);
            return finish("failed", "short_training_prefix");
        }
        // Folding all fit_reps periods needs the full visible training span.
        // With a negative lag_lo, M is slightly shorter than fit_reps*sps;
        // never read beyond visible_end to complete a period.
        if (q + static_cast<int64_t>(fit_reps * sps) > visible_end) {
            if (timings)
                timings->support_us = us_since(t0);
            return finish("failed", "short_training_prefix");
        }
        if (scratch.train_len < fit_reps * sps)
            return finish("failed", "short_training_prefix");

        const size_t fold_len = fit_reps * sps;
        build_cfo_rotation(cfo, fs, fold_len, scratch.cfo_rot);

        // folded[t] = sum_k z[k*sps + t]; z is stored only for the LS rows.
        cd folded[kMaxSps];
        for (size_t t = 0; t < sps; ++t)
            folded[t] = cd(0.0, 0.0);
        cd* const zdata = scratch.z.data();
        const cd* const rot = scratch.cfo_rot.data();
        for (size_t m = 0; m < fold_len; ++m) {
            const cd val =
                cd(y[static_cast<size_t>(q + static_cast<int64_t>(m))]) *
                rot[m];
            folded[m % sps] += val;
            if (m < M)
                zdata[m] = val;
        }

        double cir_mag[kMaxTaps];
        double global_peak = 0.0;
        size_t global_idx = 0;
        for (size_t i = 0; i < scratch.max_taps; ++i) {
            const int64_t l = scratch.lag_lo + static_cast<int64_t>(i);
            cd acc(0.0, 0.0);
            for (size_t t = 0; t < sps; ++t) {
                int64_t idx = (static_cast<int64_t>(t) - l) %
                              static_cast<int64_t>(sps);
                if (idx < 0)
                    idx += static_cast<int64_t>(sps);
                acc += folded[t] *
                       std::conj(cd(syncn[static_cast<size_t>(idx)]));
            }
            cir_mag[i] = std::abs(acc);
            if (cir_mag[i] > global_peak) {
                global_peak = cir_mag[i];
                global_idx = i;
            }
        }
        if (!(global_peak > 0.0))
            return finish("failed", "support_unavailable");

        // Local maxima above the relative floor, magnitude-descending.
        std::array<size_t, kMaxTaps> peaks{};
        size_t npeaks = 0;
        const double floor =
            cfg.peak_floor * global_peak;
        for (size_t i = 0; i < scratch.max_taps; ++i) {
            const bool left = (i == 0) || (cir_mag[i] >= cir_mag[i - 1]);
            const bool right = (i + 1 == scratch.max_taps) ||
                               (cir_mag[i] >= cir_mag[i + 1]);
            if (left && right && cir_mag[i] >= floor)
                peaks[npeaks++] = i;
        }
        std::sort(peaks.begin(), peaks.begin() + static_cast<long>(npeaks),
                  [&](size_t a, size_t b) {
                      if (cir_mag[a] != cir_mag[b])
                          return cir_mag[a] > cir_mag[b];
                      return a < b;
                  });

        // Greedy minimum-spacing selection, deterministic in magnitude order.
        std::array<size_t, kMaxTaps> kept{};
        size_t nkept = 0;
        for (size_t p = 0; p < npeaks; ++p) {
            const size_t idx = peaks[p];
            bool ok = true;
            for (size_t k = 0; k < nkept; ++k) {
                const int64_t d = std::llabs(
                    static_cast<int64_t>(idx) -
                    static_cast<int64_t>(kept[k]));
                if (d < static_cast<int64_t>(cfg.peak_min_spacing)) {
                    ok = false;
                    break;
                }
            }
            if (ok)
                kept[nkept++] = idx;
        }
        if (nkept == 0)
            kept[nkept++] = global_idx;

        // Union of +/-active_radius groups, clipped to [lag_lo,lag_hi).
        std::array<bool, kMaxTaps> active_mask{};
        size_t first_idx = scratch.max_taps;
        size_t last_idx = 0;
        for (size_t k = 0; k < nkept; ++k) {
            const int64_t c = static_cast<int64_t>(kept[k]);
            const int64_t rad = static_cast<int64_t>(cfg.active_radius);
            const int64_t lo = std::max<int64_t>(c - rad, 0);
            const int64_t hi = std::min<int64_t>(
                c + rad, static_cast<int64_t>(scratch.max_taps) - 1);
            for (int64_t i = lo; i <= hi; ++i) {
                active_mask[static_cast<size_t>(i)] = true;
                const size_t ui = static_cast<size_t>(i);
                first_idx = std::min(first_idx, ui);
                last_idx = std::max(last_idx, ui);
            }
        }
        if (first_idx == scratch.max_taps)
            return finish("failed", "support_unavailable");

        active_count = 0;
        for (size_t i = first_idx; i <= last_idx; ++i) {
            if (active_mask[i]) {
                if (active_count >= kMaxActive)
                    return finish("failed", "active_set_too_large");
                ++active_count;
            }
        }
        if (active_count == 0)
            return finish("failed", "support_unavailable");
        hull_start = scratch.lag_lo + static_cast<int64_t>(first_idx);
        hull_width = last_idx - first_idx + 1;
        if (hull_width > cfg.tap_count ||
            hull_width >= cfg.samples_per_symbol)
            return finish("failed", "support_too_wide");

        // Keep the sorted absolute active lags in the cached model entry.
        static thread_local std::array<int16_t, kMaxActive> active_cache{};
        size_t ai = 0;
        for (size_t i = first_idx; i <= last_idx && ai < active_count; ++i) {
            if (active_mask[i]) {
                active_cache[ai++] = static_cast<int16_t>(
                    scratch.lag_lo + static_cast<int64_t>(i));
            }
        }
        active_lags = active_cache.data();
        if (timings)
            timings->support_us = us_since(t0);
    }

    // ---- 6. projection X^H z over the ACTIVE columns ----------------------
    cd* const b = scratch.b.data();
    {
        const auto t0 = Clock::now();
        const cd* const zdata = scratch.z.data();
        const cf* const train = scratch.train.data();
        const int64_t train_len_i = static_cast<int64_t>(scratch.train_len);
        for (size_t i = 0; i < active_count; ++i) {
            const int64_t l = active_lags[i];
            const int64_t m0 = std::max<int64_t>(0, l);
            cd acc(0.0, 0.0);
            for (int64_t m = m0; m < static_cast<int64_t>(M); ++m) {
                const int64_t xi = m - l;
                if (xi < 0 || xi >= train_len_i)
                    continue;
                acc += std::conj(cd(train[static_cast<size_t>(xi)])) *
                       zdata[static_cast<size_t>(m)];
            }
            b[i] = acc;
        }
        if (timings)
            timings->projection_us = us_since(t0);
    }

    // ---- 7. active-set factor cache and triangular solves ----------------
    const DiscreteCirActiveFactor* factor = nullptr;
    cd h[kMaxActive];
    {
        const auto t0 = Clock::now();
        const size_t K = active_count;
        const uint64_t key = active_factor_key(
            scratch.template_hash, scratch.config_hash, active_lags, K,
            scratch.training_rows, scratch.lambda_rel);

        for (const auto& slot : scratch.factor_cache) {
            if (slot.count != K || slot.key != key)
                continue;
            bool same = true;
            for (size_t i = 0; i < K; ++i) {
                if (slot.lags[i] != active_lags[i]) {
                    same = false;
                    break;
                }
            }
            if (same && slot.chol.size() >= K * K) {
                factor = &slot;
                break;
            }
        }

        if (!factor) {
            const size_t slot_idx =
                scratch.factor_cache_next++ % scratch.factor_cache.size();
            DiscreteCirActiveFactor& slot = scratch.factor_cache[slot_idx];
            slot.count = 0;
            slot.key = 0;
            if (slot.chol.size() < K * K ||
                scratch.work.size() < K * K)
                return finish("failed", "scratch_capacity");

            // A = X_active^H X_active + lambda I from the public Gram.
            cd* const a = scratch.work.data();
            for (size_t i = 0; i < K; ++i) {
                const size_t gi = static_cast<size_t>(
                    static_cast<int64_t>(active_lags[i]) -
                    scratch.lag_lo);
                for (size_t j = 0; j < K; ++j) {
                    const size_t gj = static_cast<size_t>(
                        static_cast<int64_t>(active_lags[j]) -
                        scratch.lag_lo);
                    a[i * K + j] =
                        scratch.gram[gi * scratch.max_taps + gj];
                    if (i == j)
                        a[i * K + j] =
                            cd(a[i * K + j].real() +
                                   scratch.factor_cache[0].lambda_abs,
                               0.0);
                }
            }
            // lambda_abs is recomputed below from the fixed Gram; the slot
            // value above is only a placeholder until then.
            double trace = 0.0;
            for (size_t i = 0; i < scratch.max_taps; ++i)
                trace += scratch.gram[i * scratch.max_taps + i].real();
            const double lambda_abs =
                scratch.lambda_rel * (trace /
                                     static_cast<double>(scratch.max_taps));
            if (!(lambda_abs > 0.0) || !std::isfinite(lambda_abs))
                return finish("failed", "invalid_lambda_abs");
            for (size_t i = 0; i < K; ++i)
                a[i * K + i] = cd(a[i * K + i].real() + lambda_abs -
                                      scratch.factor_cache[0].lambda_abs,
                                 0.0);

            // Eigenvalues for cond2(X) and effective rank; the copy in the
            // cache slot is immediately overwritten by the Cholesky factor.
            std::copy(a, a + K * K, slot.chol.begin());
            double ev[kMaxActive];
            if (!hermitian_eigenvalues(slot.chol.data(), K, ev))
                return finish("failed", "nonfinite_active_gram");
            double max_ev = 0.0;
            double min_ev = std::numeric_limits<double>::infinity();
            for (size_t i = 0; i < K; ++i) {
                if (!std::isfinite(ev[i]))
                    return finish("failed", "nonfinite_active_eigenvalue");
                max_ev = std::max(max_ev, ev[i]);
                min_ev = std::min(min_ev, ev[i]);
            }
            if (!(max_ev > 0.0))
                return finish("failed", "non_positive_definite_factor");
            if (!(min_ev > 0.0)) {
                if (min_ev < -1e-10 * max_ev)
                    return finish("failed", "non_positive_definite_factor");
                min_ev = max_ev * 1e-300;
            }
            const double cond = std::sqrt(max_ev / min_ev);
            if (!std::isfinite(cond) || cond < 1.0)
                return finish("failed", "non_positive_definite_factor");
            size_t rank = 0;
            const double rank_floor = max_ev * 1e-20; // (1e-10)^2
            for (size_t i = 0; i < K; ++i)
                if (ev[i] > rank_floor)
                    ++rank;
            if (rank == 0)
                return finish("failed", "non_positive_definite_factor");

            if (!cholesky_lower(a, K, slot.chol.data()))
                return finish("failed", "non_positive_definite_factor");
            slot.key = key;
            slot.count = K;
            slot.lags.fill(0);
            for (size_t i = 0; i < K; ++i)
                slot.lags[i] = active_lags[i];
            slot.condition = std::max(1.0, cond);
            slot.effective_rank = rank;
            slot.lambda_abs = lambda_abs;
            factor = &slot;
        }

        if (!factor || factor->count != K ||
            factor->chol.size() < K * K)
            return finish("failed", "factor_cache_invalid");

        const cd* const L = factor->chol.data();
        cd* const ytmp = scratch.work.data();
        for (size_t r = 0; r < K; ++r) {
            cd acc = b[r];
            for (size_t c = 0; c < r; ++c)
                acc -= L[r * K + c] * ytmp[c];
            const cd piv = L[r * K + r];
            if (!(std::abs(piv) > 0.0) || !finite_cd(piv))
                return finish("failed", "nonfinite_solve");
            ytmp[r] = acc / piv;
            if (!finite_cd(ytmp[r]))
                return finish("failed", "nonfinite_solve");
        }
        for (size_t rr = K; rr-- > 0;) {
            cd acc = ytmp[rr];
            for (size_t c = rr + 1; c < K; ++c)
                acc -= std::conj(L[c * K + rr]) * h[c];
            const cd piv = std::conj(L[rr * K + rr]);
            if (!(std::abs(piv) > 0.0) || !finite_cd(piv))
                return finish("failed", "nonfinite_solve");
            h[rr] = acc / piv;
            if (!finite_cd(h[rr]))
                return finish("failed", "nonfinite_solve");
        }
        if (timings)
            timings->solve_us = us_since(t0);
    }

    // ---- 8. dense hull taps (inactive exactly zero) ----------------------
    cf fitted[kMaxTaps];
    for (size_t i = 0; i < hull_width; ++i)
        fitted[i] = cf(0.f, 0.f);
    for (size_t i = 0; i < active_count; ++i) {
        const int64_t l = active_lags[i];
        const int64_t dense = l - hull_start;
        if (dense < 0 || dense >= static_cast<int64_t>(hull_width))
            return finish("failed", "support_unavailable");
        fitted[static_cast<size_t>(dense)] =
            cf(static_cast<float>(h[i].real()),
               static_cast<float>(h[i].imag()));
        if (!finite_cf(fitted[static_cast<size_t>(dense)]))
            return finish("failed", "nonfinite_solve");
    }

    // ---- 9. frozen-float fit residual over the dense hull ----------------
    double fit_residual = 0.0;
    {
        const auto t0 = Clock::now();
        const cf* const train = scratch.train.data();
        const int64_t train_len_i = static_cast<int64_t>(scratch.train_len);
        const cd* const rot = scratch.cfo_rot.data();
        double num0 = 0.0, num1 = 0.0, num2 = 0.0, num3 = 0.0;
        double den0 = 0.0, den1 = 0.0, den2 = 0.0, den3 = 0.0;
        int64_t m = 0;
        const int64_t limit = (static_cast<int64_t>(M) >= 4) ? (static_cast<int64_t>(M) - 3) : 0;
        for (; m < limit; m += 4) {
            cd yhat0(0.0, 0.0), yhat1(0.0, 0.0), yhat2(0.0, 0.0), yhat3(0.0, 0.0);
            for (size_t d = 0; d < hull_width; ++d) {
                const int64_t lag = hull_start + static_cast<int64_t>(d);
                const cd fd = cd(fitted[d]);
                const int64_t xi0 = m - lag;
                const int64_t xi1 = m + 1 - lag;
                const int64_t xi2 = m + 2 - lag;
                const int64_t xi3 = m + 3 - lag;
                if (xi0 >= 0 && xi0 < train_len_i) yhat0 += fd * cd(train[static_cast<size_t>(xi0)]);
                if (xi1 >= 0 && xi1 < train_len_i) yhat1 += fd * cd(train[static_cast<size_t>(xi1)]);
                if (xi2 >= 0 && xi2 < train_len_i) yhat2 += fd * cd(train[static_cast<size_t>(xi2)]);
                if (xi3 >= 0 && xi3 < train_len_i) yhat3 += fd * cd(train[static_cast<size_t>(xi3)]);
            }
            yhat0 *= std::conj(rot[static_cast<size_t>(m)]);
            yhat1 *= std::conj(rot[static_cast<size_t>(m + 1)]);
            yhat2 *= std::conj(rot[static_cast<size_t>(m + 2)]);
            yhat3 *= std::conj(rot[static_cast<size_t>(m + 3)]);
            const cd obs0 = cd(y[static_cast<size_t>(q + m)]);
            const cd obs1 = cd(y[static_cast<size_t>(q + m + 1)]);
            const cd obs2 = cd(y[static_cast<size_t>(q + m + 2)]);
            const cd obs3 = cd(y[static_cast<size_t>(q + m + 3)]);
            num0 += std::norm(obs0 - yhat0);
            num1 += std::norm(obs1 - yhat1);
            num2 += std::norm(obs2 - yhat2);
            num3 += std::norm(obs3 - yhat3);
            den0 += std::norm(obs0);
            den1 += std::norm(obs1);
            den2 += std::norm(obs2);
            den3 += std::norm(obs3);
        }
        double num = (num0 + num1) + (num2 + num3);
        double den = (den0 + den1) + (den2 + den3);
        for (; m < static_cast<int64_t>(M); ++m) {
            cd yhat(0.0, 0.0);
            for (size_t d = 0; d < hull_width; ++d) {
                const int64_t xi =
                    m - (hull_start + static_cast<int64_t>(d));
                if (xi < 0 || xi >= train_len_i)
                    continue;
                yhat += cd(fitted[d]) *
                        cd(train[static_cast<size_t>(xi)]);
            }
            yhat *= std::conj(rot[static_cast<size_t>(m)]);
            const cd obs = cd(y[static_cast<size_t>(q + m)]);
            num += std::norm(obs - yhat);
            den += std::norm(obs);
        }
        if (!(den > 0.0) || !std::isfinite(num) || !std::isfinite(den)) {
            if (timings)
                timings->fit_check_us = us_since(t0);
            return finish("failed", "nonfinite_fit_residual");
        }
        fit_residual = num / den;
        if (!std::isfinite(fit_residual)) {
            if (timings)
                timings->fit_check_us = us_since(t0);
            return finish("failed", "nonfinite_fit_residual");
        }
        if (fit_residual > cfg.fit_residual_max) {
            if (timings)
                timings->fit_check_us = us_since(t0);
            return finish("rejected", "fit_residual_exceeded");
        }
        if (!(factor->condition < cfg.gain_condition_max)) {
            if (timings)
                timings->fit_check_us = us_since(t0);
            return finish("rejected", "condition_exceeded");
        }
        if (timings)
            timings->fit_check_us = us_since(t0);
    }

    // ---- 10. freeze the model and its fingerprint ------------------------
    model.fit_residual = fit_residual;
    model.lag_lo = hull_start;
    model.tap_count = hull_width;
    model.active_count = active_count;
    for (size_t i = 0; i < active_count; ++i)
        model.active_lags[i] = active_lags[i];
    for (size_t d = 0; d < hull_width; ++d)
        model.taps[d] = fitted[d];
    model.lambda = factor->lambda_abs;
    model.gain_condition = factor->condition;
    model.effective_rank = factor->effective_rank;
    model.training_lo = q;
    model.training_hi = q + static_cast<int64_t>(M);
    model.support_source = "coarse_cir_peak_groups";

    uint64_t id = kFnvOffset;
    hash_u64(id, kHashDomain ^ 0x4d4f4431ull); // "MOD1"
    hash_u64(id, model.impulse_template);
    hash_u64(id, model.cir_repetitions);
    hash_i64(id, model.q);
    hash_double(id, model.cfo_hz);
    hash_i64(id, model.observed_end_sample);
    hash_i64(id, model.lag_lo);
    hash_u64(id, static_cast<uint64_t>(model.tap_count));
    hash_double(id, model.lambda);
    for (size_t i = 0; i < model.tap_count; ++i) {
        hash_float(id, model.taps[i].real());
        hash_float(id, model.taps[i].imag());
    }
    hash_double(id, model.fit_residual);
    hash_double(id, model.gain_condition);
    hash_u64(id, static_cast<uint64_t>(model.active_count));
    for (size_t i = 0; i < model.active_count; ++i)
        hash_i64(id, static_cast<int64_t>(model.active_lags[i]));
    hash_u64(id, model.template_hash);
    hash_u64(id, model.config_hash);
    model.model_id = id;

    return succeed();
}

} // namespace sic
} // namespace uwb
} // namespace gr
