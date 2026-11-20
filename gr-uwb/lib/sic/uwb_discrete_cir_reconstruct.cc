/* SPDX-License-Identifier: GPL-3.0-or-later */
/*
 * Discrete equivalent CIR full-packet reconstruction.
 *
 * Implements the reconstruction half of uwb_discrete_cir_sic.h:
 *
 *   ihat[j] = exp(j*w*(j-r)) * sum_{l in active} h[l] * (s*x)[j-q-l]
 *
 * r == q, x is the COMPLETE packet waveform produced by the public modulator,
 * and h is the frozen finite FIR from estimate_discrete_cir (h already
 * contains the packet amplitude because it was fitted against the public
 * shaped SYNC train).  Only the ACTIVE lags of the dense hull are multiplied;
 * the inactive hull taps are exactly zero by contract.
 *
 * The ONE waveform scale s is recomputed here from the public reference SYNC
 * and applied to the complete waveform in a single pass before the FIR; the
 * taps are never scaled again.  The direct O(N * active_count) FIR accumulates
 * in std::complex<float> and the CFO rotation uses a 1024-sample block-anchored
 * ABSOLUTE phase grid (double block base, float rotation), so no float phase
 * drift can accumulate.
 *
 * Fail-closed: every failure leaves out == y byte-for-byte and ihat == 0.
 * The only code that modifies out is the final ROI subtraction of a committed
 * result.  Nothing here reads sidecar/truth data and there is no fallback to
 * the fractional-path backend.
 */
#include <gnuradio/uwb/uwb_discrete_cir_sic.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <gnuradio/uwb/uwb_hrp_codec.h>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstdlib>
#include <cstring>
#include <volk/volk.h>

#if defined(__x86_64__) || defined(_M_X64)
#include <immintrin.h>
#endif

namespace gr {
namespace uwb {
namespace sic {

static std::atomic<int> s_force_disable_b3(-1);

void set_b3_simd_disabled(int disabled)
{
    s_force_disable_b3.store(disabled, std::memory_order_relaxed);
}

bool is_b3_simd_disabled()
{
    const int forced = s_force_disable_b3.load(std::memory_order_relaxed);
    if (forced >= 0)
        return forced != 0;
    const char* env = std::getenv("UWB_SIC_DISABLE_B3");
    return (env != nullptr && *env != '\0' && *env != '0');
}

namespace {

using cf = std::complex<float>;
using cd = std::complex<double>;
using Clock = std::chrono::steady_clock;

constexpr double kPi = 3.14159265358979323846;
// Full-packet coordinates are frozen on the 998.4 MS/s work grid.  The frozen
// model carries a CFO in Hz but no rate, and the scratch carries only sps, so
// this public constant is the only admissible Hz -> rad/sample conversion.
// A model CFO at or beyond Nyquist is rejected instead of silently aliasing.
constexpr double kWorkRateHz = radar_meta::kWorkRateHz;
constexpr size_t kCfoBlock = 1024;
constexpr double kScaleImagMaxRel = 1e-6;
constexpr double kTemplateErrorMaxRel = 1e-10;

double us(Clock::time_point t)
{
    return std::chrono::duration<double, std::micro>(Clock::now() - t).count();
}

bool finite(cf v)
{
    return std::isfinite(v.real()) && std::isfinite(v.imag());
}

int64_t floordiv(int64_t a, int64_t b)
{
    return a / b - ((a % b) < 0 ? 1 : 0);
}

// Fused complex AXPY for one tap: dst[i] += h * src[i].
//
// This replaces multiply-into-tmp followed by add (two full passes over the
// destination) with a single read/modify/write pass, removing one vector
// write and one vector read per tap.  The scalar float form is deliberate:
// GCC lowers std::complex operator* through a NaN-preserving __mulsc3 call,
// which defeats vectorization, while this component form compiles to packed
// SSE with the project's baseline flags.  The helper stays out of line
// because inlining it into the large apply transaction suppressed the loop
// vectorizer; one call per active tap is negligible against O(N) samples.
// `dst` and `src` live in distinct preallocated scratch buffers, hence
// __restrict; per-sample tap order is unchanged because the caller still
// runs one tap at a time in ascending active order.
void __attribute__((noinline))
fused_axpy(cf* __restrict dst, const cf* __restrict src, cf h, size_t count)
{
    float* __restrict d = reinterpret_cast<float*>(dst);
    const float* __restrict s = reinterpret_cast<const float*>(src);
    const float hr = h.real();
    const float hi = h.imag();
    // No loop-carried dependence: each sample updates only its own two floats.
#pragma GCC ivdep
    for (size_t i = 0; i < count; ++i) {
        const float sr = s[2 * i];
        const float si = s[2 * i + 1];
        d[2 * i] += hr * sr - hi * si;
        d[2 * i + 1] += hi * sr + hr * si;
    }
}

// Zero-CFO dedicated accumulation.  A unit sign does not need a complex
// multiply; this preserves the old unit-real path's per-tap accumulation
// order while removing the two zero multiplies in fused_axpy.
void __attribute__((noinline))
fused_addsub(cf* __restrict dst, const cf* __restrict src, int sign,
             size_t count)
{
    float* __restrict d = reinterpret_cast<float*>(dst);
    const float* __restrict s = reinterpret_cast<const float*>(src);
    if (sign > 0) {
#pragma GCC ivdep
        for (size_t i = 0; i < count; ++i) {
            d[2 * i] += s[2 * i];
            d[2 * i + 1] += s[2 * i + 1];
        }
    } else {
#pragma GCC ivdep
        for (size_t i = 0; i < count; ++i) {
            d[2 * i] -= s[2 * i];
            d[2 * i + 1] -= s[2 * i + 1];
        }
    }
}

#if defined(__x86_64__) || defined(_M_X64)
#if defined(__GNUC__) || defined(__clang__)
inline bool has_avx2_fma()
{
    static const bool s_has =
        __builtin_cpu_supports("avx2") && __builtin_cpu_supports("fma");
    return s_has;
}

__attribute__((target("avx2")))
void accumulate_pulses_zero_cfo_20tap_avx2(
    cf* __restrict recon_base,
    const cf* __restrict taps_ptr,
    const mod::PulseEvent* __restrict events,
    size_t count)
{
    const float* t_flt = reinterpret_cast<const float*>(taps_ptr);
    const __m256 t0 = _mm256_loadu_ps(t_flt);
    const __m256 t1 = _mm256_loadu_ps(t_flt + 8);
    const __m256 t2 = _mm256_loadu_ps(t_flt + 16);
    const __m256 t3 = _mm256_loadu_ps(t_flt + 24);
    const __m256 t4 = _mm256_loadu_ps(t_flt + 32);

    for (size_t i = 0; i < count; ++i) {
        const mod::PulseEvent& e = events[i];
        float* d = reinterpret_cast<float*>(recon_base + e.sample_offset);
        if (e.sign > 0) {
            _mm256_storeu_ps(d, _mm256_add_ps(_mm256_loadu_ps(d), t0));
            _mm256_storeu_ps(d + 8, _mm256_add_ps(_mm256_loadu_ps(d + 8), t1));
            _mm256_storeu_ps(d + 16, _mm256_add_ps(_mm256_loadu_ps(d + 16), t2));
            _mm256_storeu_ps(d + 24, _mm256_add_ps(_mm256_loadu_ps(d + 24), t3));
            _mm256_storeu_ps(d + 32, _mm256_add_ps(_mm256_loadu_ps(d + 32), t4));
        } else {
            _mm256_storeu_ps(d, _mm256_sub_ps(_mm256_loadu_ps(d), t0));
            _mm256_storeu_ps(d + 8, _mm256_sub_ps(_mm256_loadu_ps(d + 8), t1));
            _mm256_storeu_ps(d + 16, _mm256_sub_ps(_mm256_loadu_ps(d + 16), t2));
            _mm256_storeu_ps(d + 24, _mm256_sub_ps(_mm256_loadu_ps(d + 24), t3));
            _mm256_storeu_ps(d + 32, _mm256_sub_ps(_mm256_loadu_ps(d + 32), t4));
        }
    }
}

__attribute__((target("avx2,fma")))
void accumulate_pulses_cfo_20tap_avx2(
    cf* __restrict recon_base,
    const cf* __restrict taps_ptr,
    const mod::PulseEvent* __restrict events,
    size_t count,
    const std::array<cf, kCfoBlock>& rotation,
    double omega)
{
    const float* t_flt = reinterpret_cast<const float*>(taps_ptr);
    const __m256 h0 = _mm256_loadu_ps(t_flt);
    const __m256 h1 = _mm256_loadu_ps(t_flt + 8);
    const __m256 h2 = _mm256_loadu_ps(t_flt + 16);
    const __m256 h3 = _mm256_loadu_ps(t_flt + 24);
    const __m256 h4 = _mm256_loadu_ps(t_flt + 32);

    const __m256 hs0 = _mm256_permute_ps(h0, 0xB1);
    const __m256 hs1 = _mm256_permute_ps(h1, 0xB1);
    const __m256 hs2 = _mm256_permute_ps(h2, 0xB1);
    const __m256 hs3 = _mm256_permute_ps(h3, 0xB1);
    const __m256 hs4 = _mm256_permute_ps(h4, 0xB1);

    const __m256 sign_mask =
        _mm256_setr_ps(-1.f, 1.f, -1.f, 1.f, -1.f, 1.f, -1.f, 1.f);

    int64_t block = -1;
    cf block_base(1.f, 0.f);

    for (size_t i = 0; i < count; ++i) {
        const mod::PulseEvent& e = events[i];
        const int64_t b = int64_t(e.sample_offset) / int64_t(kCfoBlock);
        if (b != block) {
            block = b;
            const cd base = std::exp(
                cd(0.0, omega * static_cast<double>(b * int64_t(kCfoBlock))));
            block_base = cf(static_cast<float>(base.real()),
                            static_cast<float>(base.imag()));
        }
        const cf phase =
            block_base *
            rotation[static_cast<size_t>(e.sample_offset) % kCfoBlock];
        const cf c = phase * static_cast<float>(e.sign);

        const __m256 v_cr = _mm256_set1_ps(c.real());
        const __m256 v_ci = _mm256_set1_ps(c.imag());
        const __m256 v_ci_sign = _mm256_mul_ps(v_ci, sign_mask);

        float* d = reinterpret_cast<float*>(recon_base + e.sample_offset);

        __m256 d0 = _mm256_loadu_ps(d);
        d0 = _mm256_fmadd_ps(v_cr, h0, d0);
        d0 = _mm256_fmadd_ps(v_ci_sign, hs0, d0);
        _mm256_storeu_ps(d, d0);

        __m256 d1 = _mm256_loadu_ps(d + 8);
        d1 = _mm256_fmadd_ps(v_cr, h1, d1);
        d1 = _mm256_fmadd_ps(v_ci_sign, hs1, d1);
        _mm256_storeu_ps(d + 8, d1);

        __m256 d2 = _mm256_loadu_ps(d + 16);
        d2 = _mm256_fmadd_ps(v_cr, h2, d2);
        d2 = _mm256_fmadd_ps(v_ci_sign, hs2, d2);
        _mm256_storeu_ps(d + 16, d2);

        __m256 d3 = _mm256_loadu_ps(d + 24);
        d3 = _mm256_fmadd_ps(v_cr, h3, d3);
        d3 = _mm256_fmadd_ps(v_ci_sign, hs3, d3);
        _mm256_storeu_ps(d + 24, d3);

        __m256 d4 = _mm256_loadu_ps(d + 32);
        d4 = _mm256_fmadd_ps(v_cr, h4, d4);
        d4 = _mm256_fmadd_ps(v_ci_sign, hs4, d4);
        _mm256_storeu_ps(d + 32, d4);
    }
}
#else
inline bool has_avx2_fma() { return false; }
#endif
#else
inline bool has_avx2_fma() { return false; }
#endif

// Fail-closed model gate shared by apply and the transactional wrapper.  A
// frozen model is usable only with an integer origin, the matching phase
// anchor, a finite hull strictly shorter than one SYNC period, a finite CFO
// inside Nyquist, a non-empty ACTIVE set that is sorted, inside the hull and
// backed by finite dense taps.  This reads no estimator state.
const char* model_failure(const DiscreteCirModel& m, size_t sps, size_t n)
{
    if (!m.valid || sps < 2)
        return "invalid_model";
    if (m.q < 0 || m.q >= int64_t(n))
        return "invalid_model";
    if (m.phase_reference_sample != m.q)
        return "invalid_model";
    if (m.tap_count == 0 || m.tap_count > kDiscreteCirMaxTaps)
        return "invalid_model";
    if (m.tap_count >= sps)
        return "invalid_model";
    if (m.lag_lo < -int64_t(sps) || m.lag_lo > int64_t(sps))
        return "invalid_model";
    if (m.lag_lo + int64_t(m.tap_count) > int64_t(sps))
        return "invalid_model";
    if (!std::isfinite(m.cfo_hz) || !(std::fabs(m.cfo_hz) < 0.5 * kWorkRateHz))
        return "invalid_model";
    // The dense hull is zero outside the active groups, so every hull entry
    // must be finite even though only the active ones are multiplied.
    for (size_t i = 0; i < m.tap_count; ++i)
        if (!finite(m.taps[i]))
            return "invalid_model";
    if (m.active_count == 0 || m.active_count > kDiscreteCirMaxActive ||
        m.active_count > m.tap_count)
        return "invalid_model";
    for (size_t a = 0; a < m.active_count; ++a) {
        const int64_t lag = int64_t(m.active_lags[a]);
        if (lag < m.lag_lo || lag >= m.lag_lo + int64_t(m.tap_count))
            return "invalid_model";
        if (a && lag <= int64_t(m.active_lags[a - 1]))
            return "invalid_model"; // must be strictly increasing
        if (!finite(m.taps[static_cast<size_t>(lag - m.lag_lo)]))
            return "invalid_model";
    }
    return nullptr;
}

// Capacity contract.  prepare_discrete_cir/reserve own every allocation on
// this path, so an undersized buffer is a hard failure, never a hidden resize.
// `max_packet_samples` is only the estimator's planning bound: the hot call is
// sized by the ACTUAL waveform_n/packet_n, so a stale or zero bound cannot
// reject a waveform that fits the real buffers.
const char* apply_scratch_failure(const DiscreteCirScratch& s,
                                  size_t n,
                                  size_t waveform_n)
{
    if (!s.prepared)
        return "scratch_unprepared";
    if (s.sps == 0 || s.max_window_samples < n)
        return "scratch_capacity";
    if (s.scaled_waveform.size() < waveform_n ||
        s.accumulator.size() < n || s.fir_tmp.size() < n)
        return "scratch_capacity";
    return nullptr;
}

const char* transaction_scratch_failure(const DiscreteCirScratch& s, size_t n)
{
    if (!s.prepared)
        return "scratch_unprepared";
    if (s.sps == 0 || s.max_window_samples < n)
        return "scratch_capacity";
    // modulate_one() is sized by the real buffer length; an empty buffer can
    // never hold a packet.
    if (s.waveform.empty())
        return "scratch_capacity";
    // encode_phr19() resizes its output vector to 19 entries.  Requiring the
    // capacity up front keeps the transactional path allocation-free.
    if (s.expected_phr.capacity() < 19)
        return "scratch_capacity";
    return nullptr;
}

// Sparse event-list gate.  It validates the representation itself (strictly
// increasing unit signs, complete packet length) and checks the first SYNC
// against the prepared unit template.  It never regenerates the full packet
// waveform.  The linear scan is over events, not samples.
// For internal just-generated pulse list transactions (transaction_prepared),
// the full event order/sign scan is skipped while prefix SYNC match and bounds
// are verified.
const char* pulse_list_failure(const mod::PulseList& p,
                               size_t n,
                               const DiscreteCirScratch& s,
                               size_t sync_n,
                               bool transaction_prepared = false)
{
    if (!s.prepared || s.sps == 0 ||
        sync_n != s.sps || s.impulse_sync.size() != sync_n ||
        s.max_window_samples < n)
        return "scratch_capacity";
    if (!s.sparse_pulse_reconstruct || !s.ipatov_correlation)
        return "sparse_pulse_unprepared";
    if (!p.events || p.count == 0 || p.packet_samples == 0 ||
        p.packet_samples > mod::kMaxHrpTxSamples)
        return "invalid_pulse_list";

    if (transaction_prepared) {
        // Trusted internal transaction: skip full event order/sign scan.
        // Still verify the prefix SYNC match and basic packet bounds.
        size_t first_sync_events = 0;
        for (size_t i = 0; i < p.count; ++i) {
            const mod::PulseEvent& e = p.events[i];
            if (e.sample_offset >= sync_n)
                break;
            const cf expected = s.impulse_sync[e.sample_offset];
            if (expected.imag() != 0.f ||
                expected.real() != static_cast<float>(e.sign))
                return "pulse_template_mismatch";
            ++first_sync_events;
        }
        size_t first_sync_nonzero = 0;
        for (size_t i = 0; i < sync_n; ++i)
            if (s.impulse_sync[i] != cf(0.f, 0.f))
                ++first_sync_nonzero;
        if (first_sync_events != first_sync_nonzero)
            return "pulse_template_mismatch";
        return nullptr;
    }

    uint32_t last = 0;
    size_t first_sync_events = 0;
    for (size_t i = 0; i < p.count; ++i) {
        const mod::PulseEvent& e = p.events[i];
        if (e.sign != 1 && e.sign != -1)
            return "invalid_pulse_list";
        if (e.sample_offset >= p.packet_samples)
            return "invalid_pulse_list";
        if (i && e.sample_offset <= last)
            return "invalid_pulse_list";
        last = e.sample_offset;
        if (e.sample_offset < sync_n) {
            const cf expected = s.impulse_sync[e.sample_offset];
            if (expected.imag() != 0.f ||
                expected.real() != static_cast<float>(e.sign))
                return "pulse_template_mismatch";
            ++first_sync_events;
        }
    }
    size_t first_sync_nonzero = 0;
    for (size_t i = 0; i < sync_n; ++i)
        if (s.impulse_sync[i] != cf(0.f, 0.f))
            ++first_sync_nonzero;
    if (first_sync_events != first_sync_nonzero)
        return "pulse_template_mismatch";
    return nullptr;
}

} // namespace

static bool apply_discrete_cir_impl(const std::complex<float>* y,
                        size_t n,
                        const std::complex<float>* waveform,
                        size_t waveform_n,
                        const std::complex<float>* reference_sync,
                        size_t sync_n,
                        const DiscreteCirModel& model,
                        int64_t roi_lo,
                        int64_t roi_hi,
                        DiscreteCirScratch& scratch,
                        std::complex<float>* out,
                        std::complex<float>* ihat,
                        DiscreteCirResult* result,
                        const float* real_impulses)
{
    const auto t_apply = Clock::now();
    DiscreteCirResult local;
    DiscreteCirResult& r = result ? *result : local;
    r = DiscreteCirResult();
    auto fail = [&](const char* reason) {
        if (out && y && n > 0 && out != y)
            std::copy(y, y + n, out);
        if (ihat && ihat != y && ihat != out)
            std::fill(ihat, ihat + n, cf(0.f, 0.f));
        r.committed = false;
        r.reason = reason;
        r.apply_total_us = us(t_apply);
        return false;
    };

    if (!y || !out || n == 0 || (ihat && (ihat == y || ihat == out)))
        return fail("invalid_buffers");
    if ((!waveform && !real_impulses) || !reference_sync || waveform_n == 0 || sync_n == 0 ||
        waveform_n < sync_n)
        return fail("invalid_buffers");

    const char* sf = apply_scratch_failure(scratch, n, waveform_n);
    if (sf)
        return fail(sf);
    const char* mf = model_failure(model, scratch.sps, n);
    if (mf)
        return fail(mf);
    if (roi_lo < 0 || roi_hi > int64_t(n) || roi_hi <= roi_lo)
        return fail("invalid_roi");
    if (sync_n != scratch.sps)
        return fail("invalid_template");
    for (size_t i = 0; i < n; ++i)
        if (!finite(y[i]))
            return fail("nonfinite_input");

    if (model.impulse_template != scratch.ipatov_correlation ||
        (real_impulses && !model.impulse_template))
        return fail("template_kind_mismatch");
    if (model.impulse_template) {
        if (scratch.impulse_sync.size() != sync_n)
            return fail("impulse_template_unprepared");
        reference_sync = scratch.impulse_sync.data();
    }

    r.model_id = model.model_id;
    r.q = model.q;
    r.cfo_hz = model.cfo_hz;
    r.tap_count = model.tap_count;
    r.fit_residual = model.fit_residual;
    r.gain_condition = model.gain_condition;

    const int64_t q = model.q;
    const int64_t lo = q + model.lag_lo;
    const int64_t hi = lo + int64_t(model.tap_count) + int64_t(waveform_n);
    r.support_lo = lo;
    r.support_hi = hi;

    const auto t_channel = Clock::now();

    // --- the ONE waveform scale, against the public reference SYNC ----------
    const auto t_scale = Clock::now();
    cf* const scaled = scratch.scaled_waveform.data();
    if (real_impulses) {
        // Private input from modulate_unit_impulses: validated unit pulse
        // profile + integer BPRF symbols guarantee finite {-1,0,+1} samples.
        // Match the public unit reference without constructing/scaling a
        // complete complex waveform. Absolute amplitude remains in the CIR.
        for (size_t i=0; i<sync_n; ++i)
            if (real_impulses[i] != reference_sync[i].real() ||
                reference_sync[i].imag() != 0.f)
                return fail("template_scale_mismatch");
        r.waveform_scale = 1.0;
    } else {
        cd cross(0.0, 0.0);
        double energy = 0.0, ref_energy = 0.0;
        for (size_t i = 0; i < sync_n; ++i) {
            if (!finite(waveform[i]) || !finite(reference_sync[i]))
                return fail("nonfinite_input");
            const cd w = cd(waveform[i]);
            const cd ref = cd(reference_sync[i]);
            cross += std::conj(w) * ref;
            energy += std::norm(w);
            ref_energy += std::norm(ref);
        }
        const cd scale = energy > 0.0 ? cross / energy : cd(0.0, 0.0);
        const double scale_real = scale.real();
        if (!(scale_real > 0.0) || !std::isfinite(scale_real) ||
            !std::isfinite(scale.imag()) ||
            std::fabs(scale.imag()) > kScaleImagMaxRel * scale_real ||
            !(ref_energy > 0.0))
            return fail("template_scale_mismatch");
        double template_error = 0.0;
        for (size_t i = 0; i < sync_n; ++i)
            template_error += std::norm(cd(waveform[i]) * scale - cd(reference_sync[i]));
        if (!std::isfinite(template_error) ||
            template_error > kTemplateErrorMaxRel * ref_energy)
            return fail("template_scale_mismatch");
        // Pre-scale the COMPLETE waveform exactly once.  h already contains the
        // packet amplitude, so the taps are never scaled again.  Reading through
        // `waveform` and writing `scaled_waveform` element by element also stays
        // correct if the caller handed in the same scratch buffer.
        const float scale_f = static_cast<float>(scale_real);
        for (size_t i = 0; i < waveform_n; ++i) {
            const cf v = waveform[i];
            if (!finite(v))
                return fail("nonfinite_input");
            scaled[i] = v * scale_f;
        }
        r.waveform_scale = scale_real;
    }
    r.scale_us = us(t_scale);

    // --- support and consumer ROI ------------------------------------------
    // A complete observed packet is mandatory; truncation never clips.
    if (lo < 0 || hi > int64_t(n)) {
        r.channel_us = us(t_channel);
        return fail("context_truncated");
    }
    const int64_t sub_lo = std::max(lo, roi_lo);
    const int64_t sub_hi = std::min(hi, roi_hi);
    if (sub_hi <= sub_lo) {
        r.channel_us = us(t_channel);
        return fail("no_overlap");
    }
    r.sub_lo = sub_lo;
    r.sub_hi = sub_hi;

    // --- direct O(N * active_count) FIR, tap-major contiguous passes ------
    // Each active tap is one contiguous run over j.  The inner loop is a
    // vectorized fused complex multiply-add with no per-sample bounds branch;
    // the accumulator lives in preallocated scratch, so the hot path never
    // allocates.  Only the active lags contribute (inactive hull taps are 0).
    // fir_tmp is still used by the CFO stage below.
    const auto t_conv = Clock::now();
    cf* const recon = ihat ? ihat : scratch.fir_tmp.data();
    const int64_t x_n = int64_t(waveform_n);
    const int64_t len = hi - lo;
    cf* const acc = scratch.accumulator.data();
    std::fill(acc, acc + len, cf(0.f, 0.f));
    const size_t active = model.active_count;
    if (model.impulse_template) {
        // The unit impulse packet is sparse. Add a contiguous response per
        // nonzero impulse, avoiding N * response_width zero multiplications.
        for (int64_t i=0; i<x_n; ++i) {
            const cf impulse = real_impulses ? cf(real_impulses[i],0.f) : scaled[i];
            if (impulse==cf{}) continue;
            const int64_t base=q+i+model.lag_lo;
            fused_axpy(acc+(base-lo),model.taps.data(),impulse,model.tap_count);
        }
    } else for (size_t a = 0; a < active; ++a) {
        const int64_t lag = int64_t(model.active_lags[a]);
        const cf h =
            model.taps[static_cast<size_t>(lag - model.lag_lo)];
        const int64_t j0 = std::max<int64_t>(lo, q + lag);
        const int64_t j1 = std::min<int64_t>(hi, q + lag + x_n);
        if (j1 <= j0)
            continue;
        const cf* const src = scaled + (j0 - q - lag);
        cf* const dst = acc + (j0 - lo);
        fused_axpy(dst, src, h, static_cast<size_t>(j1 - j0));
    }
    r.convolution_us = us(t_conv);

    // --- block-anchored absolute CFO grid and finite checks -----------------
    const auto t_cfo = Clock::now();
    alignas(32) std::array<cf, kCfoBlock> tmp_block;
    cf* const tmp = tmp_block.data(); // shared scalar-multiply temp
    const double omega = 2.0 * kPi * model.cfo_hz / kWorkRateHz;
    std::array<cf, kCfoBlock> rotation;
    {
        cd cur(1.0, 0.0);
        const cd step = std::exp(cd(0.0, omega));
        for (size_t k = 0; k < kCfoBlock; ++k) {
            rotation[k] = cf(static_cast<float>(cur.real()),
                             static_cast<float>(cur.imag()));
            cur *= step;
        }
    }
    const int64_t anchor = model.phase_reference_sample; // == q by the gate
    for (int64_t j = lo; j < hi;) {
        const int64_t p = j - anchor;
        const int64_t b = floordiv(p, int64_t(kCfoBlock));
        const size_t j0 = static_cast<size_t>(p - b * int64_t(kCfoBlock));
        const size_t count =
            std::min<size_t>(kCfoBlock - j0, static_cast<size_t>(hi - j));
        const cd base_phase =
            std::exp(cd(0.0, omega * static_cast<double>(b * int64_t(kCfoBlock))));
        const cf block(static_cast<float>(base_phase.real()),
                       static_cast<float>(base_phase.imag()));
        const cf* const src = acc + (j - lo);
        const unsigned nrot = static_cast<unsigned>(count);
        volk_32fc_s32fc_multiply_32fc(tmp, rotation.data() + j0, block, nrot);
        volk_32fc_x2_multiply_32fc(recon + j, src, tmp, nrot);
        j += static_cast<int64_t>(count);
    }
    for (int64_t j = lo; j < hi; ++j) {
        if (!finite(recon[j]) || !finite(y[j] - recon[j])) {
            r.cfo_us = us(t_cfo);
            r.channel_us = us(t_channel);
            return fail("nonfinite_reconstruction");
        }
    }
    if (ihat) {
        if (lo > 0)
            std::fill(ihat, ihat + lo, cf(0.f, 0.f));
        if (hi < int64_t(n))
            std::fill(ihat + hi, ihat + n, cf(0.f, 0.f));
    }
    r.cfo_us = us(t_cfo);
    r.channel_us = us(t_channel);

    // --- subtract only the consumer ROI ------------------------------------
    const auto t_subtract = Clock::now();
    if (out != y) {
        if (sub_lo > 0)
            std::copy(y, y + sub_lo, out);
        if (sub_hi < int64_t(n))
            std::copy(y + sub_hi, y + n, out + sub_hi);
    }
    // Element-wise out = y - recon.  Routed through the shared D1 element-wise
    // kernel (bit-identical; AVX-512 when the Sync group selects it) instead of
    // a bare loop the compiler only widens to 256 bits.
    demod::rxopt::vector_sub3_complex(out + sub_lo, y + sub_lo, recon + sub_lo,
                                      static_cast<size_t>(sub_hi - sub_lo));
    r.subtract_us = us(t_subtract);

    r.packet_samples = waveform_n;
    r.committed = true;
    r.reason = "committed";
    r.apply_total_us = us(t_apply);
    return true;
}


bool apply_discrete_cir(const std::complex<float>* y, size_t n,
                        const std::complex<float>* waveform, size_t waveform_n,
                        const std::complex<float>* reference_sync, size_t sync_n,
                        const DiscreteCirModel& model, int64_t roi_lo, int64_t roi_hi,
                        DiscreteCirScratch& scratch, std::complex<float>* out,
                        std::complex<float>* ihat, DiscreteCirResult* result)
{
    return apply_discrete_cir_impl(y,n,waveform,waveform_n,reference_sync,sync_n,
                                  model,roi_lo,roi_hi,scratch,out,ihat,result,nullptr);
}

static bool apply_discrete_cir_pulses_impl(
    const std::complex<float>* y,
    size_t n,
    const mod::PulseList& pulses,
    const std::complex<float>* reference_sync,
    size_t sync_n,
    const DiscreteCirModel& model,
    int64_t roi_lo,
    int64_t roi_hi,
    DiscreteCirScratch& scratch,
    std::complex<float>* out,
    std::complex<float>* ihat,
    DiscreteCirResult* result,
    bool transaction_prepared)
{
    const auto t_apply = Clock::now();
    DiscreteCirResult local;
    DiscreteCirResult& r = result ? *result : local;
    r = DiscreteCirResult();
    auto fail = [&](const char* reason) {
        if (out && y && n > 0 && out != y)
            std::copy(y, y + n, out);
        if (ihat && ihat != y && ihat != out)
            std::fill(ihat, ihat + n, cf(0.f, 0.f));
        r.committed = false;
        r.reason = reason;
        r.apply_total_us = us(t_apply);
        return false;
    };

    if (!y || !out || n == 0 || (ihat && (ihat == y || ihat == out)))
        return fail("invalid_buffers");

    const char* pf = pulse_list_failure(pulses, n, scratch, sync_n, transaction_prepared);
    if (pf)
        return fail(pf);
    if (!ihat && scratch.accumulator.size() < n)
        return fail("scratch_capacity");
    const char* mf = model_failure(model, scratch.sps, n);
    if (mf)
        return fail(mf);
    if (!reference_sync || sync_n != scratch.sps ||
        scratch.impulse_sync.size() != sync_n)
        return fail("invalid_template");
    if (!model.impulse_template || !scratch.ipatov_correlation)
        return fail("template_kind_mismatch");
    if (roi_lo < 0 || roi_hi > int64_t(n) || roi_hi <= roi_lo)
        return fail("invalid_roi");
    if (!transaction_prepared)
        for (size_t i = 0; i < n; ++i)
            if (!finite(y[i]))
                return fail("nonfinite_input");

    r.model_id = model.model_id;
    r.q = model.q;
    r.cfo_hz = model.cfo_hz;
    r.tap_count = model.tap_count;
    r.fit_residual = model.fit_residual;
    r.gain_condition = model.gain_condition;
    r.pulse_event_count = pulses.count;
    r.pulse_packet_samples = pulses.packet_samples;

    const int64_t q = model.q;
    const int64_t lo = q + model.lag_lo;
    const int64_t hi = lo + int64_t(model.tap_count) +
                       int64_t(pulses.packet_samples);
    r.support_lo = lo;
    r.support_hi = hi;

    const auto t_channel = Clock::now();

    // --- unit template identity and zero-filled reconstruction buffer ------
    const auto t_scale = Clock::now();
    // The pulse list's first SYNC was checked against impulse_sync above; the
    // public reference is intentionally bypassed here, exactly like the unit
    // real path.  Absolute amplitude lives in the frozen CIR.
    r.waveform_scale = 1.0;
    r.scale_us = us(t_scale);

    if (lo < 0 || hi > int64_t(n)) {
        r.channel_us = us(t_channel);
        return fail("context_truncated");
    }
    const int64_t sub_lo = std::max(lo, roi_lo);
    const int64_t sub_hi = std::min(hi, roi_hi);
    if (sub_hi <= sub_lo) {
        r.channel_us = us(t_channel);
        return fail("no_overlap");
    }
    r.sub_lo = sub_lo;
    r.sub_hi = sub_hi;

    const auto t_zero = Clock::now();
    cf* const recon = ihat ? ihat : scratch.accumulator.data();
    std::fill(recon + lo, recon + hi, cf(0.f, 0.f));
    if (ihat) {
        if (lo > 0)
            std::fill(ihat, ihat + lo, cf(0.f, 0.f));
        if (hi < int64_t(n))
            std::fill(ihat + hi, ihat + n, cf(0.f, 0.f));
    }
    r.zero_us = us(t_zero);

    // --- one absolute-phase multiply per event, no whole-packet rotation ----
    const auto t_phase = Clock::now();
    const double omega = 2.0 * kPi * model.cfo_hz / kWorkRateHz;
    std::array<cf, kCfoBlock> rotation;
    const cf* const taps = model.taps.data();
    if (model.cfo_hz != 0.0) {
        cd cur(1.0, 0.0);
        const cd step = std::exp(cd(0.0, omega));
        for (size_t k = 0; k < kCfoBlock; ++k) {
            rotation[k] = cf(static_cast<float>(cur.real()),
                             static_cast<float>(cur.imag()));
            cur *= step;
        }
    }
    r.phase_us = us(t_phase);

    const auto t_conv = Clock::now();
    const bool disable_b3 = is_b3_simd_disabled();
#if (defined(__x86_64__) || defined(_M_X64)) && (defined(__GNUC__) || defined(__clang__))
    if (!disable_b3 && has_avx2_fma() && model.tap_count == 20) {
        if (model.cfo_hz == 0.0) {
            accumulate_pulses_zero_cfo_20tap_avx2(
                recon + lo, taps, pulses.events, pulses.count);
        } else {
            accumulate_pulses_cfo_20tap_avx2(
                recon + lo, taps, pulses.events, pulses.count, rotation, omega);
        }
    } else
#endif
    {
        if (model.cfo_hz == 0.0) {
            // Zero CFO: sign_i is real; the dedicated add/sub keeps the old
            // unit-real accumulation order and is bit-comparable to it.
            for (size_t i = 0; i < pulses.count; ++i) {
                const mod::PulseEvent& e = pulses.events[i];
                fused_addsub(recon + lo + int64_t(e.sample_offset), taps,
                             int(e.sign), model.tap_count);
            }
        } else {
            int64_t block = -1;
            cf block_base(1.f, 0.f);
            for (size_t i = 0; i < pulses.count; ++i) {
                const mod::PulseEvent& e = pulses.events[i];
                const int64_t b = int64_t(e.sample_offset) / int64_t(kCfoBlock);
                if (b != block) {
                    block = b;
                    const cd base = std::exp(
                        cd(0.0, omega * static_cast<double>(b *
                                                            int64_t(kCfoBlock))));
                    block_base = cf(static_cast<float>(base.real()),
                                    static_cast<float>(base.imag()));
                }
                const cf phase =
                    block_base *
                    rotation[static_cast<size_t>(e.sample_offset) % kCfoBlock];
                const cf c = phase * static_cast<float>(e.sign);
                fused_axpy(recon + lo + int64_t(e.sample_offset), taps, c,
                           model.tap_count);
            }
        }
    }
    r.convolution_us = us(t_conv);
    r.cfo_us = 0.0; // CFO was folded into each pulse; no packet rotation.

    // Sparse experiment: skip the full-packet finite scan. Input/model
    // validation above remains enabled; finite-value overflow during
    // accumulation/subtraction is intentionally not detected on this path.
    r.check_us = 0.0;
    r.channel_us = us(t_channel);

    // --- subtract only the consumer ROI ------------------------------------
    const auto t_subtract = Clock::now();
    if (out != y) {
        if (sub_lo > 0)
            std::copy(y, y + sub_lo, out);
        if (sub_hi < int64_t(n))
            std::copy(y + sub_hi, y + n, out + sub_hi);
    }
    // Element-wise out = y - recon.  Routed through the shared D1 element-wise
    // kernel (bit-identical; AVX-512 when the Sync group selects it) instead of
    // a bare loop the compiler only widens to 256 bits.
    demod::rxopt::vector_sub3_complex(out + sub_lo, y + sub_lo, recon + sub_lo,
                                      static_cast<size_t>(sub_hi - sub_lo));
    r.subtract_us = us(t_subtract);

    r.packet_samples = pulses.packet_samples;
    r.committed = true;
    r.reason = "committed";
    r.apply_total_us = us(t_apply);
    return true;
}

// Public standalone entry retains its own bypass preparation and IQ scan.
bool apply_discrete_cir_pulses(
    const std::complex<float>* y, size_t n, const mod::PulseList& pulses,
    const std::complex<float>* reference_sync, size_t sync_n,
    const DiscreteCirModel& model, int64_t roi_lo, int64_t roi_hi,
    DiscreteCirScratch& scratch, std::complex<float>* out,
    std::complex<float>* ihat, DiscreteCirResult* result)
{
    return apply_discrete_cir_pulses_impl(y, n, pulses, reference_sync, sync_n,
        model, roi_lo, roi_hi, scratch, out, ihat, result, false);
}

DiscreteCirResult fullpacket_cancel_discrete(
    const std::complex<float>* y,
    size_t n,
    const std::vector<uint8_t>& phr_bits,
    const std::vector<uint8_t>& wire_psdu,
    bool fcs_pass,
    const mod::HrpModConfig& profile,
    const DiscreteCirModel& model,
    const std::complex<float>* reference_sync,
    size_t sync_n,
    int64_t roi_lo,
    int64_t roi_hi,
    DiscreteCirScratch& scratch,
    std::complex<float>* out,
    std::complex<float>* ihat)
{
    DiscreteCirResult r;
    const auto t_validate = Clock::now();
    auto fail = [&](const char* reason) {
        if (out && y && n > 0 && out != y)
            std::copy(y, y + n, out);
        if (ihat && ihat != y && ihat != out)
            std::fill(ihat, ihat + n, cf(0.f, 0.f));
        r.committed = false;
        r.reason = reason;
        r.validate_us = us(t_validate);
        return r;
    };

    if (!y || !out || n == 0 || (ihat && (ihat == y || ihat == out)))
        return fail("invalid_buffers");

    const char* sf = transaction_scratch_failure(scratch, n);
    if (sf)
        return fail(sf);
    if (!reference_sync || sync_n != scratch.sps)
        return fail("invalid_template");
    if (!fcs_pass)
        return fail("fcs_failed");
    if (profile.code_index != 10 || profile.sync_repetitions != 64 ||
        profile.insert_sts || !profile.sfd_mode ||
        std::strcmp(profile.sfd_mode, "4z2") != 0 || profile.ranging ||
        profile.pulse.shape != mod::PulseShape::Legacy)
        return fail("unsupported_profile");
    if (phr_bits.size() != 19 || wire_psdu.size() < 2 ||
        wire_psdu.size() > radar_meta::kMaxPsduBytes)
        return fail("invalid_frame_length");
    const size_t crc_len = wire_psdu.size() - 2;
    const uint16_t crc_expected =
        static_cast<uint16_t>(wire_psdu[crc_len]) |
        static_cast<uint16_t>(static_cast<uint16_t>(wire_psdu[crc_len + 1]) << 8);
    if (codec::crc16_802154(wire_psdu.data(), crc_len) != crc_expected)
        return fail("fcs_failed");
    mod::detail::encode_phr19(wire_psdu.size(),
                              profile.sync_repetitions,
                              profile.ranging,
                              scratch.expected_phr);
    for (size_t i = 0; i < 19; ++i)
        if (phr_bits[i] > 1 ||
            phr_bits[i] != static_cast<uint8_t>(scratch.expected_phr[i]))
            return fail("unsupported_phr");

    const char* mf = model_failure(model, scratch.sps, n);
    if (mf)
        return fail(mf);
    if (roi_lo < 0 || roi_hi > int64_t(n) || roi_hi <= roi_lo)
        return fail("invalid_roi");
    // Sparse SIC experiment trusts the receiver IQ: do not scan the entire
    // window for NaN/Inf. Keep frame/model/bounds validation and the standalone
    // public apply input check. Legacy waveform transactions retain the scan.
    if (!(model.impulse_template && scratch.sparse_pulse_reconstruct))
        for (size_t i = 0; i < n; ++i)
            if (!finite(y[i]))
                return fail("nonfinite_input");
    r.validate_us = us(t_validate);

    // Remodulate the decoded wire PSDU once into the scratch packet buffer.
    // The capacity is the REAL buffer length, not the estimator's planning
    // bound, so a stale max_packet_samples cannot reject a fitting packet.
    const auto t_tx = Clock::now();
    if (model.impulse_template != scratch.ipatov_correlation) {
        return fail("template_kind_mismatch");
    }
    const bool real_only = model.impulse_template;
    if (real_only && scratch.sparse_pulse_reconstruct) {
        // Sparse experiment: the decoded packet is generated as positions and
        // signs only.  No real/complex waveform is materialized.
        mod::PulseList pulses;
        const bool tx_ok = mod::modulate_pulse_list(
            wire_psdu.data(), wire_psdu.size(), scratch.impulse_profile,
            scratch.modulator, scratch.waveform.size(), pulses);
        if (!tx_ok || pulses.count == 0) {
            r.tx_us = us(t_tx);
            return fail("tx_failed");
        }
        const double tx_us = us(t_tx);

        DiscreteCirResult applied;
        apply_discrete_cir_pulses_impl(y, n, pulses, reference_sync, sync_n, model,
                                  roi_lo, roi_hi, scratch, out, ihat,
                                  &applied, true);
        applied.validate_us = r.validate_us;
        applied.tx_us = tx_us;
        return applied;
    }

    size_t packet_n = 0;
    const bool tx_ok = real_only
        ? mod::modulate_unit_impulses(wire_psdu.data(), wire_psdu.size(),
                                    scratch.impulse_profile, scratch.modulator,
                                    scratch.waveform.size(), packet_n)
        : mod::modulate_one(wire_psdu.data(), wire_psdu.size(), profile,
                            scratch.modulator, scratch.waveform.data(),
                            scratch.waveform.size(), packet_n);
    if (!tx_ok || packet_n == 0 || packet_n > scratch.waveform.size()) {
        r.tx_us = us(t_tx);
        return fail("tx_failed");
    }
    const double tx_us = us(t_tx);

    DiscreteCirResult applied;
    apply_discrete_cir_impl(y,
                       n,
                       real_only ? nullptr : scratch.waveform.data(),
                       packet_n,
                       reference_sync,
                       sync_n,
                       model,
                       roi_lo,
                       roi_hi,
                       scratch,
                       out,
                       ihat,
                       &applied,
                       real_only ? scratch.modulator.iq_re.data() : nullptr);
    // apply() owns channel_us/subtract_us/apply_total_us; the wrapper owns the
    // validation and remodulation stages (never double-counted as children).
    applied.validate_us = r.validate_us;
    applied.tx_us = tx_us;
    return applied;
}

} // namespace sic
} // namespace uwb
} // namespace gr
