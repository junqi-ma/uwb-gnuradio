/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * S0 preamble-only SIC math core — implementation.
 * See uwb_preamble_sic_core.h and docs/sic/S0_sic_core_api.md.
 *
 * Pure C++ core.  No scheduler, no file I/O.  sic_one() never allocates and
 * never grows a vector: all buffers are sized by prepare_sic_core().  The
 * complex least squares is a self-contained double-precision K x K Gram matrix
 * + Cholesky (no Eigen/LAPACK wiring), and no explicit matrix inverse is ever
 * formed.
 *
 * The Estimate path mirrors the frozen NumPy reference
 * (tools/sic/sic_reference.py) stage for stage: linear normalized-correlation
 * detection, a backward repetition walk, steady-state CFO, a local matched-
 * filter CIR, mainlobe-spaced path selection, an index-windowed bandlimited
 * fractional refine, a Gram-matrix gain fit, masked reconstruction and a
 * disjoint-rep holdout commit gate.  Matching the reference is what makes the
 * golden cross-check meaningful.
 */

#include <gnuradio/uwb/uwb_preamble_sic_core.h>

#include <volk/volk.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

namespace gr {
namespace uwb {
namespace sic {

namespace {

using cplx = std::complex<float>;
using dcp = std::complex<double>;

constexpr double kPi = 3.14159265358979323846;
constexpr int kInterpHalf = 16;        // core fractional-delay filter half-span
constexpr size_t kInterpTaps = static_cast<size_t>(2 * kInterpHalf + 1);
// Polyphase-upsampled preamble phases.  P >= 20 keeps the delay grid at least
// as fine as the old 0.05-sample frac_refine search grid.  The fractional-delay
// operator reads at most two neighbouring rows and blends them (linear in the
// phase), so nothing in the hot path runs a per-output 33-tap dot product.
constexpr size_t kPreUpPhases = 32;
constexpr double kMaxGainCond = 1e6;   // commit ceiling on cond2(A), not a pivot ratio
constexpr double kPathPeakFloor = 0.15; // candidate floor vs strongest peak
constexpr size_t kMaxReps = 512;       // confirmed-rep scratch ceiling
constexpr int64_t kCirPre = 4;         // CIR delay range: [-4, post_tail+4)

inline uint64_t us_since(const std::chrono::steady_clock::time_point& t0)
{
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - t0)
            .count());
}

inline double sinc_pi(double t)
{
    if (std::abs(t) < 1e-12)
        return 1.0;
    const double p = kPi * t;
    return std::sin(p) / p;
}

// Windowed-sinc taps for sub-sample phase ph/kPreUpPhases (DC-normalized, so
// the integer phase is an exact delta).  These are the phase-specific rows of
// the ONE windowed-sinc kernel; at prepare time they build the P-phase
// polyphase table, and the fractional-delay operator LATER blends two
// neighbouring rows linearly in the sub-sample phase.  So the operator is a
// piecewise-linear interpolation in phase over the same kernel's phase samples
// -- not "the same kernel" evaluated exactly.  (The frozen reference and the
// cross-model generator use their own windowed-sinc operators; the golden NMSE
// bounds what this approximation costs.)
void build_phase_taps(double* taps, size_t ph)
{
    const size_t M = kInterpTaps;
    const double tau = static_cast<double>(ph) / static_cast<double>(kPreUpPhases);
    double sum = 0.0;
    for (size_t nn = 0; nn < M; ++nn) {
        const double j = static_cast<double>(nn) - static_cast<double>(kInterpHalf);
        const double w =
            0.54 - 0.46 * std::cos(2.0 * kPi * static_cast<double>(nn) /
                                   static_cast<double>(M - 1));
        const double tap = sinc_pi(j - tau) * w;
        taps[nn] = tap;
        sum += tap;
    }
    if (std::abs(sum) > 1e-12) {
        const double inv = 1.0 / sum;
        for (size_t nn = 0; nn < M; ++nn)
            taps[nn] *= inv;
    }
}

// Polyphase delay selection for output position `pos`.  Row g holds the
// template DELAYED by g/kPreUpPhases of a sample, so with tau_abs = n_int + fr,
//   preamble(pos - tau_abs) ~= (1-alpha)*pre_up[g][pos-n_int]
//                           +  alpha  *pre_up[g+1][pos-n_int],
// where g = floor(fr*P), alpha = fr*P - g (row 0 at pos-n_int-1 when g+1 === P).
struct PhaseSel {
    size_t g = 0;       // first table row
    double alpha = 0.0; // blend weight towards the next phase
    int64_t n0 = 0;     // integer base index into a row
    bool wrap = false;  // g+1 wraps to row 0 evaluated at n0-1
};

inline PhaseSel select_phase(double tau_abs, int64_t pos)
{
    const double ni = std::floor(tau_abs);
    const int64_t n_int = static_cast<int64_t>(ni);
    double fr = tau_abs - ni; // [0,1)
    if (!(fr >= 0.0) || fr >= 1.0)
        fr = 0.0;
    double beta = fr * static_cast<double>(kPreUpPhases);
    if (!(beta >= 0.0))
        beta = 0.0;
    size_t g = static_cast<size_t>(beta);
    if (g >= kPreUpPhases)
        g = kPreUpPhases - 1;
    double alpha = beta - static_cast<double>(g);
    if (alpha < 0.0)
        alpha = 0.0;
    if (alpha > 1.0)
        alpha = 1.0;
    PhaseSel s;
    s.g = g;
    s.alpha = alpha;
    s.n0 = pos - n_int;
    s.wrap = (g + 1 >= kPreUpPhases);
    return s;
}

// dst[m] += coeff * row[base + m] for m in [0,M), skipping out-of-range taps.
// Real-or-complex coefficient scaling of a complex<double> destination; the
// interleaved storage is walked as one contiguous double AXPY (float source ->
// double accumulator).
inline void axpy_row(dcp* dst, size_t M, const cplx* row, size_t plen,
                     int64_t base, dcp coeff)
{
    if (M == 0 || !row)
        return;
    int64_t m_lo = (base < 0) ? -base : 0;
    int64_t m_hi = static_cast<int64_t>(M);
    const int64_t hi = static_cast<int64_t>(plen) - base;
    if (hi < m_hi)
        m_hi = hi;
    if (m_hi <= m_lo)
        return;
    const cplx* x = row + m_lo + base;
    double* d = reinterpret_cast<double*>(dst + m_lo);
    const float* xf = reinterpret_cast<const float*>(x);
    const double cr = coeff.real();
    const double ci = coeff.imag();
    const int64_t cnt = m_hi - m_lo;
    if (ci == 0.0) {
        const int64_t n2 = 2 * cnt;
        for (int64_t i = 0; i < n2; ++i)
            d[i] += cr * static_cast<double>(xf[i]);
    } else {
        for (int64_t i = 0; i < cnt; ++i) {
            const double xr = static_cast<double>(xf[2 * i]);
            const double xi = static_cast<double>(xf[2 * i + 1]);
            d[2 * i] += cr * xr - ci * xi;
            d[2 * i + 1] += cr * xi + ci * xr;
        }
    }
}

// dst[m] += gain * preamble((w0 + m) - tau_abs), evaluated on the polyphase
// table.  At most two contiguous row AXPYs per delay -- the replacement for the
// old per-output 33-tap windowed-sinc dot.
inline void accum_shifted(dcp* dst, size_t M, const cplx* up, size_t plen,
                          int64_t w0, double tau_abs, dcp gain)
{
    if (M == 0 || !up)
        return;
    const PhaseSel s = select_phase(tau_abs, w0);
    axpy_row(dst, M, up + s.g * plen, plen, s.n0, gain * dcp(1.0 - s.alpha, 0.0));
    if (s.alpha != 0.0) {
        if (s.wrap)
            axpy_row(dst, M, up, plen, s.n0 - 1, gain * dcp(s.alpha, 0.0));
        else
            axpy_row(dst, M, up + (s.g + 1) * plen, plen, s.n0,
                     gain * dcp(s.alpha, 0.0));
    }
}

// Column over the absolute window [w0, w0+M): preamble delayed by tau_abs,
// CFO-rotated by the precomputed ramp.  This is the matrix_build hotspot; it is
// a couple of contiguous polyphase row AXPYs instead of 33 tap-major passes.
inline void build_column(dcp* col, size_t M, const cplx* up, size_t plen,
                         int64_t w0, double tau_abs, const dcp* rot)
{
    if (M == 0)
        return;
    for (size_t m = 0; m < M; ++m)
        col[m] = dcp(0.0, 0.0);
    accum_shifted(col, M, up, plen, w0, tau_abs, dcp(1.0, 0.0));
    if (rot)
        for (size_t m = 0; m < M; ++m)
            col[m] *= rot[m];
}

// dst[m] = preamble((w0 + m) - tau_abs) as single-precision complex, WITHOUT the
// CFO rotation.  This is the un-rotated shifted-template column, the exact
// float32 twin of `build_column(.., rot=nullptr)`.  It exists so the
// fractional-refine projection score -- the single hottest inner loop of the
// core -- can run on VOLK 32fc kernels instead of rebuilding a double column
// per candidate offset.  The caller folds the unit-modulus ramp into the
// residual (q = conj(rot) * resid), which makes the projection numerator
// <u, q> and the denominator ||u||^2 identical to the double-rotated form while
// keeping this loop rotation-free.  Out-of-range taps are exactly zero, matching
// axpy_row's clamping.
inline void build_shifted_f(cplx* dst, size_t M, const cplx* up, size_t plen,
                            int64_t w0, double tau_abs)
{
    if (M == 0 || !up)
        return;
    const PhaseSel s = select_phase(tau_abs, w0);
    const cplx* const r0 = up + s.g * plen;
    const int64_t n0 = s.n0;
    const int64_t lo0 = std::max<int64_t>(0, -n0);
    const int64_t hi0 = std::min<int64_t>(static_cast<int64_t>(M),
                                          static_cast<int64_t>(plen) - n0);
    const float a0 = static_cast<float>(1.0 - s.alpha);
    const float a1 = static_cast<float>(s.alpha);
    const bool second = (s.alpha != 0.0);
    const cplx* const r1 = s.wrap ? up : up + (s.g + 1) * plen;
    const int64_t b1 = s.wrap ? n0 - 1 : n0;
    const int64_t lo1 = second ? std::max<int64_t>(0, -b1) : 0;
    const int64_t hi1 = second
        ? std::min<int64_t>(static_cast<int64_t>(M),
                            static_cast<int64_t>(plen) - b1)
        : 0;
    const int64_t lo = second ? std::min(lo0, lo1) : lo0;
    int64_t hi = second ? std::max(hi0, hi1) : hi0;
    if (hi < lo)
        hi = lo; // defensive: both rows fully out of range -> all-zero column
    hi = std::min<int64_t>(hi, static_cast<int64_t>(M));
    // Zero the (usually tiny) out-of-range head/tail, then fill [lo,hi) in
    // branch-free segments: the overlap of the two rows is the common case and
    // vectorizes as a straight two-row blend; at most one one-row shoulder.
    for (int64_t m = 0; m < lo; ++m)
        dst[m] = cplx(0.f, 0.f);
    for (int64_t m = hi; m < static_cast<int64_t>(M); ++m)
        dst[m] = cplx(0.f, 0.f);
    if (!second) {
        for (int64_t m = lo0; m < hi0; ++m) {
            const cplx v = r0[n0 + m];
            dst[m] = cplx(a0 * v.real(), a0 * v.imag());
        }
        return;
    }
    const int64_t lb = std::max(lo0, lo1);
    const int64_t hb = std::min(hi0, hi1);
    for (int64_t m = lo; m < lb; ++m) {
        if (m >= lo0 && m < hi0) {
            const cplx v = r0[n0 + m];
            dst[m] = cplx(a0 * v.real(), a0 * v.imag());
        } else if (m >= lo1 && m < hi1) {
            const cplx v = r1[b1 + m];
            dst[m] = cplx(a1 * v.real(), a1 * v.imag());
        } else {
            dst[m] = cplx(0.f, 0.f);
        }
    }
    for (int64_t m = lb; m < hb; ++m) {
        const cplx v0 = r0[n0 + m];
        const cplx v1 = r1[b1 + m];
        dst[m] = cplx(a0 * v0.real() + a1 * v1.real(),
                      a0 * v0.imag() + a1 * v1.imag());
    }
    for (int64_t m = hb; m < hi; ++m) {
        if (m >= lo0 && m < hi0) {
            const cplx v = r0[n0 + m];
            dst[m] = cplx(a0 * v.real(), a0 * v.imag());
        } else if (m >= lo1 && m < hi1) {
            const cplx v = r1[b1 + m];
            dst[m] = cplx(a1 * v.real(), a1 * v.imag());
        } else {
            dst[m] = cplx(0.f, 0.f);
        }
    }
}

// rot[i] = exp(j*w*(idx0+i)); block recurrence with periodic absolute re-anchor.
inline void fill_rot(dcp* rot, size_t count, int64_t idx0, double w)
{
    constexpr size_t kB = 256;
    for (size_t b0 = 0; b0 < count; b0 += kB) {
        const size_t b1 = std::min(count, b0 + kB);
        const double ph0 = w * static_cast<double>(idx0 + static_cast<int64_t>(b0));
        dcp cur(std::cos(ph0), std::sin(ph0));
        const dcp step(std::cos(w), std::sin(w));
        for (size_t i = b0; i < b1; ++i) {
            rot[i] = cur;
            cur *= step;
        }
    }
}

// One block of the float32 CFO ramp base.  Kept a power of two; the phase is
// re-anchored absolutely every block (see fill_rot_f), so this only trades the
// number of VOLK calls against the sincos count and does not affect accuracy.
constexpr size_t kRotBlock = 1024;

// out[i] = exp(j*w*(idx0+i)) as complex<float>, VOLK-vectorized.
//
// rot[0..count) = blockphase_b * base[0..B) with base[j] = exp(j*w*j) and
// blockphase_b = exp(j*w*(idx0 + b*B)).  base is computed in double by a short
// recurrence over one block, and each block phase is re-anchored ABSOLUTELY
// from std::exp(dcp(0,w*(idx0+b*B))) -- no cross-block recurrence, so the block
// phase carries no accumulated drift.  Only the per-sample write is float32.
// Each block is then a single VOLK 32fc scalar multiply, so the old serial
// 65k-step double recurrence -- the single hottest loop in reconstruct -- is
// gone.  `base` must hold at least min(count, kRotBlock) entries.
inline void fill_rot_f(cplx* out, size_t count, int64_t idx0, double w, cplx* base)
{
    if (count == 0 || !out)
        return;
    const size_t nb = std::min(count, kRotBlock);
    const dcp step(std::cos(w), std::sin(w));
    dcp cur(1.0, 0.0);
    for (size_t j = 0; j < nb; ++j) {
        base[j] = cplx(static_cast<float>(cur.real()), static_cast<float>(cur.imag()));
        cur *= step;
    }
    for (size_t b0 = 0; b0 < count; b0 += kRotBlock) {
        const size_t n = std::min(kRotBlock, count - b0);
        const dcp blk = std::exp(
            dcp(0.0, w * static_cast<double>(idx0 + static_cast<int64_t>(b0))));
        const cplx bf(static_cast<float>(blk.real()), static_cast<float>(blk.imag()));
        volk_32fc_s32fc_multiply_32fc(reinterpret_cast<lv_32fc_t*>(out + b0),
                                      reinterpret_cast<const lv_32fc_t*>(base), bf,
                                      static_cast<unsigned>(n));
    }
}

// out[i] = acc[i] * exp(j*w*(idx0+i)) for i in [0,count): narrow the double
// path-major accumulation to float32 (VOLK 64f->32f over the interleaved
// components) and multiply by the VOLK-built float CFO ramp.  This is the
// reconstruction combine; it replaces a scalar double per-sample complex
// multiply plus scalar rotation.  The mask/region semantics are untouched -- the
// caller still zeroes i_hat outside the mask.
inline void reconstruct_combine(cplx* out, const dcp* acc, size_t count,
                                int64_t idx0, double w, cplx* rotf, cplx* base)
{
    if (count == 0)
        return;
    fill_rot_f(rotf, count, idx0, w, base);
    volk_64f_convert_32f(reinterpret_cast<float*>(out),
                         reinterpret_cast<const double*>(acc),
                         static_cast<unsigned>(2 * count));
    volk_32fc_x2_multiply_32fc(reinterpret_cast<lv_32fc_t*>(out),
                               reinterpret_cast<const lv_32fc_t*>(out),
                               reinterpret_cast<const lv_32fc_t*>(rotf),
                               static_cast<unsigned>(count));
}

// cond2 (2-norm condition number) of a K x K complex matrix given column-major,
// by a self-contained one-sided complex Jacobi SVD (K <= kSicMaxPaths).  The
// singular values are the column 2-norms at convergence, so cond2 = smax/smin.
//
// This is the definition the commit gate uses.  It deliberately factors the QR
// factor R itself and NEVER forms R^H R, so it does not square the condition
// number: forming the normal-equation Gram loses log10(cond) digits of the
// small singular value.  A matrix whose |R_kk| diagonal ratio is small but whose
// true cond2 exceeds the ceiling is therefore still rejected.  Returns false for
// a non-finite or (numerically) rank-deficient factor so the caller fails closed.
bool jacobi_cond2(const dcp* Rin, size_t K, double* cond_out)
{
    dcp M[kSicMaxPaths][kSicMaxPaths];
    for (size_t j = 0; j < K; ++j)
        for (size_t i = 0; i < K; ++i)
            M[i][j] = Rin[j * K + i];
    for (size_t i = 0; i < K; ++i)
        for (size_t j = 0; j < K; ++j)
            if (!std::isfinite(M[i][j].real()) || !std::isfinite(M[i][j].imag()))
                return false;

    constexpr int kMaxSweeps = 50;
    bool converged = false;
    for (int sweep = 0; sweep < kMaxSweeps && !converged; ++sweep) {
        double worst = 0.0;
        for (size_t p = 0; p + 1 < K; ++p) {
            for (size_t q = p + 1; q < K; ++q) {
                double alpha = 0.0, beta = 0.0;
                dcp gamma(0.0, 0.0);
                for (size_t i = 0; i < K; ++i) {
                    alpha += std::norm(M[i][p]);
                    beta += std::norm(M[i][q]);
                    gamma += std::conj(M[i][p]) * M[i][q];
                }
                const double mu = std::abs(gamma);
                const double scale = std::sqrt(alpha * beta);
                if (scale > 0.0)
                    worst = std::max(worst, mu / scale);
                // Already (numerically) orthogonal, or a zero column: no rotation.
                if (!(mu > 1e-15 * scale))
                    continue;
                // Orthogonalize columns p,q with the eigenvectors of the 2x2
                // Hermitian Gram [[alpha,gamma],[conj(gamma),beta]].  For
                // theta = (beta-alpha)/2, r = hypot(theta,|gamma|), the
                // eigenvectors are [gamma, theta +/- r]; V = [v+ v-] normalized
                // is unitary and diagonalizes the block, so the rotated column
                // norms are its eigenvalues (the pair singular values squared).
                const double theta = 0.5 * (beta - alpha);
                const double r = std::hypot(theta, mu);
                const double np = std::hypot(mu, theta + r);
                const double nm = std::hypot(mu, theta - r);
                if (!(np > 0.0) || !(nm > 0.0))
                    continue;
                const dcp c0 = gamma / np;
                const dcp c2((theta + r) / np, 0.0);
                const dcp c1 = gamma / nm;
                const dcp c3((theta - r) / nm, 0.0);
                for (size_t i = 0; i < K; ++i) {
                    const dcp gp = M[i][p];
                    const dcp gq = M[i][q];
                    M[i][p] = gp * c0 + gq * c2;
                    M[i][q] = gp * c1 + gq * c3;
                }
            }
        }
        // `worst` is the largest relative off-diagonal of the sweep start; near
        // convergence it decays quadratically, so a small cap on `worst` is a
        // safe stop.  mu is NOT recomputed for the just-rotated pair, but the
        // rotation drives it to ~eps, so the next sweep confirms convergence.
        converged = (worst < 1e-14);
    }

    double smax = 0.0, smin = std::numeric_limits<double>::infinity();
    for (size_t j = 0; j < K; ++j) {
        double nrm = 0.0;
        for (size_t i = 0; i < K; ++i)
            nrm += std::norm(M[i][j]);
        nrm = std::sqrt(nrm);
        if (!std::isfinite(nrm))
            return false;
        smax = std::max(smax, nrm);
        smin = std::min(smin, nrm);
    }
    if (!(smin > 0.0) || !std::isfinite(smax))
        return false;
    if (cond_out)
        *cond_out = smax / smin;
    return true;
}

// Householder QR of A (column-major, M x K) with a single RHS transform in
// place (b <- Q^H b).  v is a length-M workspace.  The upper triangle of A holds
// the factor R on success.  Does NOT report a condition number: the old
// max|R_kk|/min|R_kk| diagonal ratio is not cond(A) (see jacobi_cond2).
//
// Raw interleaved-double arithmetic (not std::complex) on the four shoulders:
// each Householder apply is two straight AXPYs over M elements, which the
// compiler can SLP-vectorize.  The old std::complex< double > form was the
// scalar bottleneck of the gain fit (~6x slower than the Gram it replaced).
bool qr_factor_rhs(dcp* A, size_t M, size_t K, dcp* b, dcp* v)
{
    double* const Ad = reinterpret_cast<double*>(A);
    double* const bd = reinterpret_cast<double*>(b);
    double* const vd = reinterpret_cast<double*>(v);
    for (size_t k = 0; k < K; ++k) {
        const size_t rk = k * M;
        double nrm = 0.0;
        for (size_t i = k; i < M; ++i) {
            const double xr = Ad[2 * (i + rk)];
            const double xi = Ad[2 * (i + rk) + 1];
            nrm += xr * xr + xi * xi;
        }
        nrm = std::sqrt(nrm);
        if (!(nrm > 0.0) || !std::isfinite(nrm))
            return false;
        const double x0r = Ad[2 * (k + rk)];
        const double x0i = Ad[2 * (k + rk) + 1];
        const double ax = std::hypot(x0r, x0i);
        double phr = 1.0, phi = 0.0;
        if (ax > 1e-300) {
            phr = x0r / ax;
            phi = x0i / ax;
        }
        const double alr = -phr * nrm;
        const double ali = -phi * nrm;
        double vn2 = 0.0;
        for (size_t i = k; i < M; ++i) {
            double vr = Ad[2 * (i + rk)];
            double vi = Ad[2 * (i + rk) + 1];
            if (i == k) {
                vr -= alr;
                vi -= ali;
            }
            vd[2 * i] = vr;
            vd[2 * i + 1] = vi;
            vn2 += vr * vr + vi * vi;
        }
        if (!(vn2 > 0.0) || !std::isfinite(vn2))
            return false;
        const double beta = 2.0 / vn2;
        for (size_t j = k; j < K; ++j) {
            const size_t cj = j * M;
            double dr = 0.0, di = 0.0;
            for (size_t i = k; i < M; ++i) {
                const double vr = vd[2 * i];
                const double vi = vd[2 * i + 1];
                const double ar = Ad[2 * (i + cj)];
                const double ai = Ad[2 * (i + cj) + 1];
                dr += vr * ar + vi * ai;
                di += vr * ai - vi * ar;
            }
            const double cr = beta * dr;
            const double ci = beta * di;
            for (size_t i = k; i < M; ++i) {
                const double vr = vd[2 * i];
                const double vi = vd[2 * i + 1];
                Ad[2 * (i + cj)] -= cr * vr - ci * vi;
                Ad[2 * (i + cj) + 1] -= cr * vi + ci * vr;
            }
        }
        double dbr = 0.0, dbi = 0.0;
        for (size_t i = k; i < M; ++i) {
            const double vr = vd[2 * i];
            const double vi = vd[2 * i + 1];
            const double br = bd[2 * i];
            const double bi = bd[2 * i + 1];
            dbr += vr * br + vi * bi;
            dbi += vr * bi - vi * br;
        }
        const double cbr = beta * dbr;
        const double cbi = beta * dbi;
        for (size_t i = k; i < M; ++i) {
            const double vr = vd[2 * i];
            const double vi = vd[2 * i + 1];
            bd[2 * i] -= cbr * vr - cbi * vi;
            bd[2 * i + 1] -= cbr * vi + cbi * vr;
        }
        if (!std::isfinite(Ad[2 * (k + rk)]) ||
            !std::isfinite(Ad[2 * (k + rk) + 1]))
            return false;
    }
    return true;
}

// Back-substitute R h = b (R upper triangle of column-major A).
bool qr_solve(const dcp* A, size_t M, size_t K, const dcp* b, dcp* h)
{
    for (size_t ii = K; ii-- > 0;) {
        dcp s = b[ii];
        for (size_t j = ii + 1; j < K; ++j)
            s -= A[j * M + ii] * h[j];
        const dcp piv = A[ii * M + ii];
        if (!(std::abs(piv) > 0.0) || !std::isfinite(std::abs(piv)))
            return false;
        h[ii] = s / piv;
    }
    return true;
}

void reset_result(SicResult& r, cplx* outbuf, cplx* ihatbuf)
{
    r.state = SicState::Bypassed;
    r.detected = false;
    r.committed = false;
    r.partial = false;
    r.ambiguous = false;
    r.bypass_reason = nullptr;
    r.uses_truth = false;
    r.detected_start = -1;
    r.support_lo = -1;
    r.support_hi = -1;
    r.cfo_hz = 0.0;
    r.n_paths = 0;
    r.n_path_candidates = 0;
    r.n_taus = 0;
    for (size_t k = 0; k < kSicMaxPaths; ++k)
        r.taus[k] = 0.0;
    r.gain_condition = 0.0;
    r.residual_corr = 0.0;
    r.sub_overlap = -1;
    r.coarse_candidates = 0;
    r.fine_candidates = 0;
    r.correlations = 0;
    r.roi_samples = 0;
    r.reject_reason = nullptr;
    r.timings = SicTimings{};
    r.out = outbuf;
    r.i_hat = ihatbuf;
}

inline bool finite_cplx(const cplx& v)
{
    return std::isfinite(v.real()) && std::isfinite(v.imag());
}

} // namespace

// ---------------------------------------------------------------------------
// detail::qr_cond_solve  (see header)
// ---------------------------------------------------------------------------
namespace detail {

bool qr_cond_solve(dcp* A, size_t M, size_t K, dcp* y, dcp* h, dcp* work,
                   double* cond)
{
    if (!A || !y || !h || !work || M == 0 || K == 0 || K > kSicMaxPaths)
        return false;

    // Householder QR of A with the RHS transform b <- Q^H y (both in place).
    if (!qr_factor_rhs(A, M, K, y, work))
        return false;

    // Extract the K x K factor R from the upper triangle of the column-major A
    // and report cond2(A) = cond2(R) from a self-contained SVD.  This is the
    // honest condition number -- NOT the |R_kk| diagonal ratio.
    dcp R[kSicMaxPaths * kSicMaxPaths];
    for (size_t j = 0; j < K; ++j)
        for (size_t i = 0; i < K; ++i)
            R[j * K + i] = (i <= j) ? A[j * M + i] : dcp(0.0, 0.0);
    double c2 = 0.0;
    if (!jacobi_cond2(R, K, &c2))
        return false;

    if (!qr_solve(A, M, K, y, h))
        return false;
    if (cond)
        *cond = c2;
    return true;
}

} // namespace detail

// ---------------------------------------------------------------------------
// prepare
// ---------------------------------------------------------------------------
bool prepare_sic_core(const SicConfig& cfg,
                      const cplx* sync,
                      size_t sync_len,
                      const cplx* preamble,
                      size_t preamble_len,
                      SicScratch& scratch,
                      std::string* err)
{
    scratch = SicScratch{};
    auto fail = [&](const char* msg) {
        if (err)
            *err = msg;
        scratch = SicScratch{};
        return false;
    };

    if (err)
        err->clear();

    const uint8_t mv = static_cast<uint8_t>(cfg.mode);
    if (mv > 2)
        return fail("invalid mode (not Off/Estimate/Oracle)");
    if (!sync || sync_len == 0 || !preamble || preamble_len == 0)
        return fail("null/empty template");
    if (cfg.samples_per_symbol == 0)
        return fail("samples_per_symbol == 0");
    if (sync_len != cfg.samples_per_symbol)
        return fail("sync_len != samples_per_symbol");
    if (preamble_len < cfg.samples_per_symbol)
        return fail("preamble_len < samples_per_symbol");
    if (cfg.sync_reps == 0)
        return fail("sync_reps == 0");
    if (cfg.coarse_stride == 0)
        return fail("coarse_stride == 0");
    if (cfg.k_paths == 0 || cfg.k_paths > kSicMaxPaths)
        return fail("k_paths out of [1, kSicMaxPaths]");
    if (cfg.fit_reps == 0 || cfg.fit_reps > cfg.sync_reps)
        return fail("fit_reps out of range");
    if (cfg.validation_reps == 0)
        return fail("validation_reps == 0");
    if (cfg.fit_reps > std::numeric_limits<size_t>::max() - cfg.validation_reps)
        return fail("fit_reps + validation_reps overflow");
    if (cfg.fit_reps + cfg.validation_reps > kMaxReps)
        return fail("fit_reps + validation_reps exceeds scratch ceiling");
    if (!std::isfinite(cfg.detect_threshold) || cfg.detect_threshold < 0.0 ||
        cfg.detect_threshold > 1.0)
        return fail("detect_threshold out of [0,1]");
    if (!std::isfinite(cfg.validate_threshold) || cfg.validate_threshold < 0.0 ||
        cfg.validate_threshold > 1.0)
        return fail("validate_threshold out of [0,1]");
    if (!(cfg.sample_rate > 0.0) || !std::isfinite(cfg.sample_rate))
        return fail("sample_rate must be finite and > 0");

    scratch.sync_norm.assign(sync, sync + sync_len);
    double e = 0.0;
    for (size_t i = 0; i < sync_len; ++i) {
        if (!finite_cplx(scratch.sync_norm[i]))
            return fail("sync contains non-finite samples");
        e += std::norm(scratch.sync_norm[i]);
    }
    if (!(e > 0.0) || !std::isfinite(e))
        return fail("sync energy == 0");
    const float inv = static_cast<float>(1.0 / std::sqrt(e));
    for (size_t i = 0; i < sync_len; ++i)
        scratch.sync_norm[i] *= inv;

    scratch.preamble.assign(preamble, preamble + preamble_len);
    for (size_t i = 0; i < preamble_len; ++i) {
        if (!finite_cplx(scratch.preamble[i]))
            return fail("preamble contains non-finite samples");
    }

    // ----- polyphase-upsampled preamble (the one fractional-delay operator) --
    // kPreUpPhases rows of `preamble_len`: row g = template advanced by
    // g/kPreUpPhases of a sample.  Row 0 is the exact integer copy; every other
    // row is the windowed-sinc kernel of that sub-sample phase.  Built once
    // here, never touched in sic_one().
    scratch.pre_up.assign(kPreUpPhases * preamble_len, cplx(0.f, 0.f));
    std::copy(preamble, preamble + preamble_len, scratch.pre_up.begin());
    if (kPreUpPhases > 1) {
        double taps[kInterpTaps];
        std::vector<double> tmp(2 * preamble_len, 0.0);
        for (size_t g = 1; g < kPreUpPhases; ++g) {
            build_phase_taps(taps, g);
            std::fill(tmp.begin(), tmp.end(), 0.0);
            for (size_t p = 0; p < kInterpTaps; ++p) {
                const double tap = taps[p];
                if (tap == 0.0)
                    continue;
                const int64_t shift = static_cast<int64_t>(p) - kInterpHalf;
                // tmp[n] += tap * preamble[n - shift]
                axpy_row(reinterpret_cast<dcp*>(tmp.data()), preamble_len,
                         preamble, preamble_len, -shift, dcp(tap, 0.0));
            }
            cplx* row = scratch.pre_up.data() + g * preamble_len;
            for (size_t i = 0; i < preamble_len; ++i) {
                if (!std::isfinite(tmp[2 * i]) || !std::isfinite(tmp[2 * i + 1]))
                    return fail("polyphase preamble contains non-finite samples");
                row[i] = cplx(static_cast<float>(tmp[2 * i]),
                              static_cast<float>(tmp[2 * i + 1]));
            }
        }
    }

    // Template autocorrelation mainlobe width (first lag where |r| < 0.5 peak).
    scratch.autocorr_half = 1;
    {
        const size_t L = sync_len;
        const size_t mmax = std::min<size_t>(L - 1, 4096);
        for (size_t m = 1; m <= mmax; ++m) {
            dcp acc(0.0, 0.0);
            for (size_t k = 0; k + m < L; ++k)
                acc += dcp(scratch.sync_norm[k + m]) * dcp(scratch.sync_norm[k]);
            const double r = std::abs(acc);
            if (std::isfinite(r) && r < 0.5) {
                scratch.autocorr_half = std::max<size_t>(1, m);
                break;
            }
        }
    }
    scratch.min_spacing = (cfg.path_min_spacing > 0)
        ? cfg.path_min_spacing
        : std::max<size_t>(1, scratch.autocorr_half);

    scratch.sync_len = sync_len;
    scratch.preamble_len = preamble_len;
    scratch.sps = cfg.samples_per_symbol;
    scratch.k_paths = cfg.k_paths;
    scratch.fit_reps = cfg.fit_reps;
    scratch.validation_reps = cfg.validation_reps;
    scratch.post_tail = cfg.post_tail;
    scratch.fit_len = cfg.fit_reps * cfg.samples_per_symbol;
    scratch.work_len = std::max(cfg.fit_reps, cfg.validation_reps) *
                       cfg.samples_per_symbol;
    scratch.cap_region = preamble_len + 2 * cfg.post_tail + 32;

    const size_t M = scratch.work_len;
    const size_t K = cfg.k_paths;
    scratch.A.assign(M * K, dcp(0.0, 0.0));
    scratch.hh_v.assign(M, dcp(0.0, 0.0));
    scratch.hh_b.assign(M, dcp(0.0, 0.0));
    scratch.col.assign(M, dcp(0.0, 0.0));
    scratch.col_f.assign(M, cplx(0.f, 0.f));
    scratch.q_f.assign(M, cplx(0.f, 0.f));
    scratch.resid.assign(M, dcp(0.0, 0.0));
    scratch.rot.assign(std::max(M, scratch.cap_region), dcp(0.0, 0.0));
    scratch.rot_f.assign(scratch.cap_region, cplx(0.f, 0.f));
    scratch.rot_base.assign(kRotBlock, cplx(0.f, 0.f));
    scratch.acc.assign(scratch.cap_region, dcp(0.0, 0.0));
    scratch.ihat.assign(scratch.cap_region, cplx(0.f, 0.f));
    const size_t cap_delay = cfg.post_tail + 16;
    scratch.cir.assign(cap_delay, cplx(0.f, 0.f));
    scratch.cir_mag.assign(cap_delay, 0.f);
    scratch.cand_delay.assign(cap_delay, 0);
    scratch.cand_mag.assign(cap_delay, 0.0);

    // Sliding self-energy prefix capacity: cover every lag the detection search
    // can query for THIS (prepare-time) ROI and repetition geometry, i.e. the
    // coarse/fine search over [roi_lo, roi_hi) plus the `rtot` repetition peaks
    // that extend (rtot-1) symbols past the detected origin.  A runtime ROI that
    // exceeds it simply falls back to the per-lag VOLK self-dot, so the fast
    // path is always safe.  roi_hi <= roi_lo (0 => full window, unknown here)
    // disables the prefix and the core keeps the exact legacy kernel.
    {
        const int64_t rtot =
            static_cast<int64_t>(cfg.fit_reps + cfg.validation_reps);
        const int64_t roi_hi_cfg = (cfg.roi_hi > cfg.roi_lo) ? cfg.roi_hi : 0;
        if (roi_hi_cfg > 0 && rtot > 0) {
            int64_t lag_max = std::max<int64_t>(roi_hi_cfg - 1, cfg.roi_lo);
            lag_max +=
                (rtot - 1) * static_cast<int64_t>(cfg.samples_per_symbol);
            const int64_t need = lag_max + static_cast<int64_t>(sync_len) + 1;
            if (need > 0)
                scratch.energy_prefix.assign(static_cast<size_t>(need), 0.0);
        }
    }

    scratch.orig_sync = sync;
    scratch.orig_preamble = preamble;
    scratch.peak_bytes =
        scratch.sync_norm.size() * sizeof(cplx) +
        scratch.preamble.size() * sizeof(cplx) +
        scratch.pre_up.size() * sizeof(cplx) + scratch.cir.size() * sizeof(cplx) +
        scratch.cir_mag.size() * sizeof(float) + scratch.A.size() * sizeof(dcp) +
        scratch.hh_v.size() * sizeof(dcp) + scratch.hh_b.size() * sizeof(dcp) +
        scratch.col.size() * sizeof(dcp) + scratch.resid.size() * sizeof(dcp) +
        scratch.col_f.size() * sizeof(cplx) + scratch.q_f.size() * sizeof(cplx) +
        scratch.rot.size() * sizeof(dcp) + scratch.acc.size() * sizeof(dcp) +
        scratch.ihat.size() * sizeof(cplx) +
        scratch.rot_f.size() * sizeof(cplx) +
        scratch.rot_base.size() * sizeof(cplx) +
        scratch.cand_delay.size() * sizeof(int64_t) +
        scratch.cand_mag.size() * sizeof(double) +
        scratch.energy_prefix.size() * sizeof(double);
    scratch.prepared = true;
    return true;
}

// ---------------------------------------------------------------------------
// sic_one
// ---------------------------------------------------------------------------
bool sic_one(const cplx* y,
             size_t n,
             const SicTemplates& templates,
             const SicConfig& cfg,
             SicScratch& scratch,
             SicResult& out,
             const SicTruth* truth)
{
    const auto t_total0 = std::chrono::steady_clock::now();
    cplx* const outbuf = out.out;
    cplx* const ihatbuf = out.i_hat;
    reset_result(out, outbuf, ihatbuf);
    auto finish = [&]() { out.timings.core_total_us = us_since(t_total0); };

    if (!y || n == 0) {
        out.state = SicState::Failed;
        out.reject_reason = "invalid_input";
        finish();
        return false;
    }
    if (!outbuf) {
        out.state = SicState::Failed;
        out.reject_reason = "no_output_buffer";
        finish();
        return false;
    }

    std::copy(y, y + n, outbuf);
    if (ihatbuf)
        std::fill(ihatbuf, ihatbuf + n, cplx(0.f, 0.f));

    if (!scratch.prepared) {
        out.state = SicState::Failed;
        out.reject_reason = "not_prepared";
        finish();
        return false;
    }
    if (templates.sync != scratch.orig_sync ||
        templates.preamble != scratch.orig_preamble ||
        templates.sync_len != scratch.sync_len ||
        templates.preamble_len != scratch.preamble_len ||
        cfg.samples_per_symbol != scratch.sps || cfg.k_paths != scratch.k_paths ||
        cfg.fit_reps != scratch.fit_reps ||
        cfg.validation_reps != scratch.validation_reps ||
        cfg.post_tail != scratch.post_tail) {
        out.state = SicState::Failed;
        out.reject_reason = "template_or_geometry_mismatch";
        finish();
        return false;
    }
    const uint8_t mv = static_cast<uint8_t>(cfg.mode);
    if (mv > 2) {
        out.state = SicState::Failed;
        out.reject_reason = "invalid_mode";
        finish();
        return false;
    }
    if (cfg.mode == SicMode::Off) {
        out.state = SicState::Bypassed;
        out.bypass_reason = "mode_off";
        finish();
        return true;
    }
    if (cfg.mode == SicMode::Estimate) {
        if (!(cfg.detect_threshold > 0.0)) {
            out.state = SicState::Failed;
            out.reject_reason = "uncalibrated_detect_threshold";
            finish();
            return false;
        }
        if (!(cfg.validate_threshold > 0.0)) {
            out.state = SicState::Failed;
            out.reject_reason = "uncalibrated_validate_threshold";
            finish();
            return false;
        }
    }

    const size_t Ls = scratch.sync_len;
    const size_t sps = scratch.sps;
    const size_t plen = scratch.preamble_len;
    const cplx* const syncn = scratch.sync_norm.data();
    const cplx* const up = scratch.pre_up.data();
    const size_t K = cfg.k_paths;
    const size_t Mfit = scratch.fit_len;
    const double fs = cfg.sample_rate;
    const double detect_thr = cfg.detect_threshold;

    // ---- Oracle: diagnostic upper bound from explicit truth -----------------
    if (cfg.mode == SicMode::Oracle) {
        if (!truth || !truth->valid) {
            out.state = SicState::Failed;
            out.reject_reason = "oracle_truth_missing";
            finish();
            return true;
        }
        out.uses_truth = true;
        const int64_t start = static_cast<int64_t>(std::llround(truth->start_sample));
        const size_t ns = std::min(truth->n_paths, kSicMaxPaths);
        if (ns == 0) {
            out.state = SicState::Failed;
            out.reject_reason = "oracle_no_paths";
            finish();
            return true;
        }
        out.detected = true;
        out.detected_start = start;
        out.cfo_hz = truth->cfo_hz;
        out.n_paths = ns;
        out.n_taus = ns;
        for (size_t k = 0; k < ns; ++k)
            out.taus[k] = truth->start_sample + truth->tau[k];
        const double w = 2.0 * kPi * truth->cfo_hz / fs;
        const int64_t ih_lo = std::max<int64_t>(0, start - kInterpHalf);
        const int64_t ih_hi = std::min<int64_t>(
            static_cast<int64_t>(n), start + static_cast<int64_t>(plen) + 8);
        const size_t ih_len = (ih_hi > ih_lo) ? size_t(ih_hi - ih_lo) : 0;
        if (ih_len > scratch.cap_region) {
            out.state = SicState::Failed;
            out.reject_reason = "capacity_overflow";
            finish();
            return true;
        }
        {
            const auto t0 = std::chrono::steady_clock::now();
            std::fill(scratch.acc.begin(), scratch.acc.begin() + ih_len,
                      dcp(0.0, 0.0));
            for (size_t k = 0; k < ns; ++k)
                accum_shifted(scratch.acc.data(), ih_len, up, plen, ih_lo,
                              truth->start_sample + truth->tau[k],
                              dcp(truth->gain[k]));
            fill_rot(scratch.rot.data(), ih_len, ih_lo, w);
            for (size_t i = 0; i < ih_len; ++i) {
                const dcp v = scratch.acc[i] * scratch.rot[i];
                scratch.ihat[i] =
                    cplx(static_cast<float>(v.real()), static_cast<float>(v.imag()));
            }
            out.timings.reconstruct_us = us_since(t0);
        }
        const int64_t sup_lo = std::max<int64_t>(0, start);
        const int64_t sup_hi =
            std::min<int64_t>(static_cast<int64_t>(n),
                              start + static_cast<int64_t>(cfg.sync_reps * sps));
        out.support_lo = sup_lo;
        out.support_hi = sup_hi;
        for (size_t i = 0; i < ih_len; ++i) {
            const size_t abs_idx = static_cast<size_t>(ih_lo + static_cast<int64_t>(i));
            const cplx ih = scratch.ihat[i];
            outbuf[abs_idx] = outbuf[abs_idx] - ih;
            if (ihatbuf)
                ihatbuf[abs_idx] = ih;
        }
        out.state = SicState::Committed;
        out.committed = true;
        finish();
        return true;
    }

    // ======================= Estimate =======================================
    int64_t roi_lo = cfg.roi_lo;
    int64_t roi_hi = cfg.roi_hi;
    if (roi_hi <= roi_lo) {
        roi_lo = 0;
        roi_hi = static_cast<int64_t>(n);
    }
    roi_hi = std::min<int64_t>(roi_hi, static_cast<int64_t>(n) -
                                           static_cast<int64_t>(Ls) + 1);
    roi_lo = std::max<int64_t>(roi_lo, 0);
    out.roi_samples = (roi_hi > roi_lo) ? static_cast<size_t>(roi_hi - roi_lo) : 0;
    const int64_t stride = std::max<int64_t>(1, static_cast<int64_t>(cfg.coarse_stride));

    // |<sync, seg>| / (||seg|| * ||sync||)  (sync_norm has unit energy)
    //
    // detect_coarse evaluates this at ~(ROI/stride) positions, each a 1016-tap
    // complex conjugate dot.  The TEMPLATE correlation stays a single VOLK
    // kernel (unaligned dispatcher: `seg = y + d` has no alignment guarantee).
    // The segment self-energy is NOT a second per-lag dot any more: it is the
    // difference of two entries of the double-precision |y|^2 prefix
    // `energy_prefix`, built once per window over the queried range below.  That
    // removes ~half the per-lag work; the normalization is mathematically the
    // same sum (double-accumulated instead of the legacy float self-dot), so the
    // detection decisions are unchanged.
    double* const energy =
        scratch.energy_prefix.empty() ? nullptr : scratch.energy_prefix.data();
    size_t energy_fill = 0;
    auto norm_at = [&](int64_t d, cplx* corr_out) -> double {
        if (d < 0 || d + static_cast<int64_t>(Ls) > static_cast<int64_t>(n)) {
            if (corr_out)
                *corr_out = cplx(0.f, 0.f);
            return 0.0;
        }
        const cplx* const seg = y + d;
        const unsigned np = static_cast<unsigned>(Ls);
        cplx dot(0.f, 0.f);
        volk_32fc_x2_conjugate_dot_prod_32fc_u(&dot, seg, syncn, np);
        const size_t ehi = static_cast<size_t>(d) + Ls;
        double e = 0.0;
        if (energy && ehi <= energy_fill) {
            e = std::sqrt(energy[ehi] - energy[static_cast<size_t>(d)]);
        } else {
            // Out-of-range query (runtime ROI/reps beyond the prepared prefix):
            // exact legacy path, one VOLK self conjugate dot, real part = energy.
            cplx self(0.f, 0.f);
            volk_32fc_x2_conjugate_dot_prod_32fc_u(&self, seg, seg, np);
            e = std::sqrt(static_cast<double>(self.real()));
        }
        if (corr_out)
            *corr_out = dot;
        return (e > 0.0) ? static_cast<double>(std::abs(dot)) / e : 0.0;
    };

    // ---- 1. detect_coarse --------------------------------------------------
    int64_t coarse_pos = -1;
    double coarse_nrm = 0.0;
    {
        const auto t0 = std::chrono::steady_clock::now();
        // One double-precision |y|^2 prefix pass over the range the search can
        // reach; every lag energy is then a single difference.  The chosen
        // (double) summation order is fixed, so results are deterministic.
        if (energy) {
            energy_fill = std::min(scratch.energy_prefix.size() - 1, n);
            double acc = 0.0;
            energy[0] = 0.0;
            for (size_t i = 0; i < energy_fill; ++i) {
                const cplx v = y[i];
                acc += static_cast<double>(v.real()) * static_cast<double>(v.real()) +
                       static_cast<double>(v.imag()) * static_cast<double>(v.imag());
                energy[i + 1] = acc;
            }
        }
        for (int64_t d = roi_lo; d < roi_hi; d += stride) {
            const double nrm = norm_at(d, nullptr);
            ++out.coarse_candidates;
            ++out.correlations;
            if (nrm > coarse_nrm) {
                coarse_nrm = nrm;
                coarse_pos = d;
            }
        }
        out.timings.detect_coarse_us = us_since(t0);
    }

    // ---- 1b. detect_fine ---------------------------------------------------
    int64_t detected_start = coarse_pos;
    double peak_norm = coarse_nrm;
    {
        const auto t0 = std::chrono::steady_clock::now();
        if (coarse_pos >= 0) {
            const int64_t lo = std::max(roi_lo, coarse_pos - stride);
            const int64_t hi = std::min(roi_hi, coarse_pos + stride + 1);
            for (int64_t d = lo; d < hi; ++d) {
                const double nrm = norm_at(d, nullptr);
                ++out.fine_candidates;
                ++out.correlations;
                if (nrm > peak_norm) {
                    peak_norm = nrm;
                    detected_start = d;
                }
            }
        }
        out.timings.detect_fine_us = us_since(t0);
    }

    // ---- 2. repetition confirmation (period-ambiguity resolution) ----------
    {
        const auto t0 = std::chrono::steady_clock::now();
        if (detected_start >= 0) {
            int64_t origin = detected_start;
            const int64_t floor_lo = std::max<int64_t>(roi_lo, 0);
            while (origin - static_cast<int64_t>(sps) >= floor_lo) {
                const int64_t d = origin - static_cast<int64_t>(sps);
                const double nrm = (d + static_cast<int64_t>(Ls) <=
                                    static_cast<int64_t>(n))
                    ? norm_at(d, nullptr)
                    : 0.0;
                ++out.correlations;
                if (nrm >= detect_thr)
                    origin = d;
                else
                    break;
            }
            const int64_t lo = std::max(roi_lo, origin - stride);
            const int64_t hi = std::min(roi_hi, origin + stride + 1);
            for (int64_t d = lo; d < hi; ++d) {
                const double nrm = norm_at(d, nullptr);
                ++out.fine_candidates;
                ++out.correlations;
                if (nrm > peak_norm) {
                    origin = d;
                    peak_norm = nrm;
                }
            }
            detected_start = origin;
        }
        out.timings.detect_fine_us += us_since(t0);
    }
    if (detected_start < 0 || peak_norm < detect_thr) {
        out.state = SicState::Bypassed;
        out.bypass_reason = "below_detect_threshold";
        finish();
        return true;
    }
    out.detected = true;
    out.detected_start = detected_start;

    // ---- repetition confirmation -------------------------------------------
    const size_t rtot = cfg.fit_reps + cfg.validation_reps;
    cplx rep_corr[kMaxReps];
    double rep_norms[kMaxReps];
    size_t confirmed = 0;
    {
        for (size_t k = 0; k < rtot; ++k) {
            const int64_t d = detected_start + static_cast<int64_t>(k * sps);
            cplx c(0.f, 0.f);
            const double nrm = (d + static_cast<int64_t>(Ls) <= static_cast<int64_t>(n))
                ? norm_at(d, &c)
                : 0.0;
            rep_corr[k] = c;
            rep_norms[k] = nrm;
            ++out.correlations;
            if (nrm >= detect_thr)
                ++confirmed;
        }
    }
    if (confirmed < 2) {
        out.state = SicState::Ambiguous;
        out.ambiguous = true;
        out.bypass_reason = "repetition_not_confirmed";
        out.reject_reason = "repetition_not_confirmed";
        finish();
        return true;
    }
    if (confirmed < rtot)
        out.partial = true;

    // ---- 3. cfo (steady-state repeated symbols only) -----------------------
    double cfo = 0.0;
    {
        const auto t0 = std::chrono::steady_clock::now();
        if (cfg.enable_cfo) {
            dcp z(0.0, 0.0);
            size_t cnt = 0;
            const size_t k_lo = (rtot >= 3) ? 1 : 0;
            for (size_t k = k_lo; k + 1 < rtot; ++k) {
                if (rep_norms[k] >= detect_thr && rep_norms[k + 1] >= detect_thr) {
                    dcp t = dcp(rep_corr[k + 1]) * std::conj(dcp(rep_corr[k]));
                    const double a = std::abs(t);
                    if (a > 0.0) {
                        z += t / a;
                        ++cnt;
                    }
                }
            }
            if (cnt > 0) {
                const double dphi = std::arg(z);
                cfo = dphi / (2.0 * kPi * static_cast<double>(sps) / fs);
            }
        }
        out.timings.cfo_us = us_since(t0);
    }
    out.cfo_hz = cfo;
    const double w = 2.0 * kPi * cfo / fs;

    // ---- 4. cir ------------------------------------------------------------
    const int64_t dmin = -kCirPre;
    const int64_t dmax = static_cast<int64_t>(cfg.post_tail) + kCirPre;
    const size_t nd = static_cast<size_t>(dmax - dmin);
    if (nd > scratch.cir.size()) {
        out.state = SicState::Failed;
        out.reject_reason = "capacity_overflow";
        finish();
        return true;
    }
    {
        const auto t0 = std::chrono::steady_clock::now();
        for (size_t idx = 0; idx < nd; ++idx) {
            const int64_t pos = detected_start + dmin + static_cast<int64_t>(idx);
            cplx c(0.f, 0.f);
            if (pos >= 0 && pos + static_cast<int64_t>(Ls) <= static_cast<int64_t>(n)) {
                dcp acc(0.0, 0.0);
                for (size_t k = 0; k < Ls; ++k)
                    acc += dcp(std::conj(syncn[k])) * dcp(y[pos + static_cast<int64_t>(k)]);
                c = cplx(static_cast<float>(acc.real()), static_cast<float>(acc.imag()));
            }
            scratch.cir[idx] = c;
            scratch.cir_mag[idx] = static_cast<float>(std::abs(c));
        }
        out.timings.cir_us = us_since(t0);
    }

    // ---- path_select -------------------------------------------------------
    double tau_abs[kSicMaxPaths];
    size_t nsel = 0;
    {
        const auto t0 = std::chrono::steady_clock::now();
        size_t nc = 0;
        for (size_t i = 1; i + 1 < nd; ++i) {
            const double m = scratch.cir_mag[i];
            if (m > scratch.cir_mag[i - 1] && m >= scratch.cir_mag[i + 1]) {
                scratch.cand_delay[nc] = static_cast<int64_t>(i);
                scratch.cand_mag[nc] = m;
                ++nc;
            }
        }
        // selection sort by magnitude descending
        for (size_t a = 0; a < nc; ++a) {
            size_t bi = a;
            for (size_t b = a + 1; b < nc; ++b)
                if (scratch.cand_mag[b] > scratch.cand_mag[bi])
                    bi = b;
            if (bi != a) {
                std::swap(scratch.cand_mag[a], scratch.cand_mag[bi]);
                std::swap(scratch.cand_delay[a], scratch.cand_delay[bi]);
            }
        }
        out.n_path_candidates = nc;
        double mmax = 0.0;
        for (size_t i = 0; i < nd; ++i)
            mmax = std::max(mmax, static_cast<double>(scratch.cir_mag[i]));
        const double floor = kPathPeakFloor * mmax;
        const size_t spacing = scratch.min_spacing;
        int64_t sel_idx[kSicMaxPaths];
        for (size_t c = 0; c < nc && nsel < K; ++c) {
            if (scratch.cand_mag[c] < floor)
                continue;
            bool ok = true;
            for (size_t s = 0; s < nsel; ++s) {
                const int64_t diff = scratch.cand_delay[c] - sel_idx[s];
                const int64_t ad = diff < 0 ? -diff : diff;
                if (ad < static_cast<int64_t>(spacing)) {
                    ok = false;
                    break;
                }
            }
            if (!ok)
                continue;
            sel_idx[nsel] = scratch.cand_delay[c];
            tau_abs[nsel] =
                static_cast<double>(detected_start + dmin + scratch.cand_delay[c]);
            ++nsel;
        }
        out.timings.path_select_us = us_since(t0);
    }
    if (nsel == 0) {
        out.state = SicState::Failed;
        out.bypass_reason = "no_paths";
        out.reject_reason = "no_paths";
        finish();
        return true;
    }
    out.n_paths = nsel;
    out.support_lo = std::max<int64_t>(0, detected_start);
    out.support_hi = std::min<int64_t>(
        static_cast<int64_t>(n),
        detected_start + static_cast<int64_t>(confirmed * sps + cfg.post_tail));

    // ---- 4b. subtraction / commit ROI overlap gate --------------------------
    // The SEARCH ROI (cfg.roi_lo/roi_hi) is deliberately wide so the natural
    // preamble/multipath tail is never clipped by the candidate-start range
    // (R02).  The SUBTRACTION / COMMIT ROI (cfg.sub_roi_lo/sub_roi_hi) is a
    // separate, sensing-owned region: the SIC may only alter samples that
    // overlap the region the sensing receiver actually processes (its preamble /
    // CIR).  A template-consistent comm detected ENTIRELY OUTSIDE that region is
    // real but NOT actionable -- subtracting it would change the output while
    // removing nothing from the sensing signal, which the R01/R10 negative
    // contract counts as a false commit.  Fail-closed: BYPASS (out == y).
    // When no sub_roi is configured the legacy subtraction mask (clipped to the
    // search ROI) is kept so existing callers are byte-for-byte unchanged.
    int64_t sub_lo = 0;
    int64_t sub_hi = 0;
    if (cfg.sub_roi_hi > cfg.sub_roi_lo) {
        const int64_t lo = std::max<int64_t>(out.support_lo, cfg.sub_roi_lo);
        const int64_t hi = std::min<int64_t>(out.support_hi, cfg.sub_roi_hi);
        const int64_t overlap = (hi > lo) ? (hi - lo) : 0;
        out.sub_overlap = overlap;
        const int64_t need = std::max<int64_t>(
            1, static_cast<int64_t>(cfg.sub_min_overlap));
        if (overlap < need) {
            out.state = SicState::Bypassed;
            out.bypass_reason = "outside_sensing_roi";
            out.reject_reason = "outside_sensing_roi";
            finish();
            return true;
        }
        sub_lo = lo;
        sub_hi = hi;
    } else {
        out.sub_overlap = -1;
        sub_lo = std::max<int64_t>(0, std::max<int64_t>(detected_start, roi_lo));
        sub_hi = std::min<int64_t>(static_cast<int64_t>(n),
                                   std::min<int64_t>(out.support_hi, roi_hi));
    }

    const int64_t fit_lo = detected_start;
    const int64_t fit_hi = detected_start + static_cast<int64_t>(Mfit);

    // ---- 5. fractional_refine ---------------------------------------------
    dcp h[kSicMaxPaths];
    for (size_t k = 0; k < nsel; ++k)
        h[k] = dcp(0.0, 0.0);
    double cond = 0.0;
    if (cfg.enable_fractional && fit_hi <= static_cast<int64_t>(n) && Mfit >= nsel) {
        const auto t0 = std::chrono::steady_clock::now();
        fill_rot(scratch.rot.data(), Mfit, fit_lo, w);
        for (size_t k = 0; k < nsel; ++k)
            build_column(scratch.A.data() + k * Mfit, Mfit, up, plen,
                         static_cast<int64_t>(fit_lo), tau_abs[k],
                         scratch.rot.data());
        for (size_t m = 0; m < Mfit; ++m)
            scratch.hh_b[m] = dcp(y[fit_lo + static_cast<int64_t>(m)]);
        if (qr_factor_rhs(scratch.A.data(), Mfit, nsel, scratch.hh_b.data(),
                          scratch.hh_v.data()) &&
            qr_solve(scratch.A.data(), Mfit, nsel, scratch.hh_b.data(), h)) {
            // The projection score |<col, r>|^2 / ||col||^2 with
            // col = u * rot is invariant if both the column and the residual
            // are un-rotated: <u*rot, r> = <u, conj(rot)*r> and ||u*rot|| =
            // ||u||.  So the candidate loop builds the (float32, un-rotated)
            // shifted template `u` and projects it onto the float residual
            // q = conj(rot)*r with two VOLK 32fc kernels -- no double column
            // rebuild and no per-candidate CFO multiply.  The residual itself
            // is still accumulated in double (unchanged path).
            cplx* const colf = scratch.col_f.data();
            cplx* const qf = scratch.q_f.data();
            const unsigned np = static_cast<unsigned>(Mfit);
            for (size_t k = 0; k < nsel; ++k) {
                // residual from all other paths (coordinate descent)
                for (size_t m = 0; m < Mfit; ++m)
                    scratch.resid[m] = dcp(y[fit_lo + static_cast<int64_t>(m)]);
                for (size_t j = 0; j < nsel; ++j) {
                    if (j == k)
                        continue;
                    build_column(scratch.col.data(), Mfit, up, plen,
                                 static_cast<int64_t>(fit_lo), tau_abs[j],
                                 scratch.rot.data());
                    for (size_t m = 0; m < Mfit; ++m)
                        scratch.resid[m] -= h[j] * scratch.col[m];
                }
                for (size_t m = 0; m < Mfit; ++m) {
                    const dcp ro = scratch.rot[m];
                    const dcp rv = scratch.resid[m];
                    qf[m] = cplx(
                        static_cast<float>(ro.real() * rv.real() +
                                           ro.imag() * rv.imag()),
                        static_cast<float>(ro.real() * rv.imag() -
                                           ro.imag() * rv.real()));
                }
                double best_tau = tau_abs[k];
                double best_score = -1.0;
                for (int oi = 0; oi <= 20; ++oi) {
                    const double delta = -0.5 + 0.05 * oi;
                    build_shifted_f(colf, Mfit, up, plen,
                                    static_cast<int64_t>(fit_lo),
                                    tau_abs[k] + delta);
                    cplx num(0.f, 0.f), self(0.f, 0.f);
                    volk_32fc_x2_conjugate_dot_prod_32fc_u(&num, colf, qf, np);
                    volk_32fc_x2_conjugate_dot_prod_32fc_u(&self, colf, colf, np);
                    const double den = static_cast<double>(self.real());
                    const double score =
                        (den > 0.0) ? std::norm(dcp(num)) / den : 0.0;
                    if (score > best_score) {
                        best_score = score;
                        best_tau = tau_abs[k] + delta;
                    }
                }
                tau_abs[k] = best_tau;
            }
        }
        out.timings.fractional_refine_us = us_since(t0);
    }

    // ---- 6a. matrix_build --------------------------------------------------
    {
        const auto t0 = std::chrono::steady_clock::now();
        fill_rot(scratch.rot.data(), Mfit, fit_lo, w);
        for (size_t k = 0; k < nsel; ++k)
            build_column(scratch.A.data() + k * Mfit, Mfit, up, plen,
                         static_cast<int64_t>(fit_lo), tau_abs[k],
                         scratch.rot.data());
        for (size_t m = 0; m < Mfit; ++m)
            scratch.hh_b[m] = dcp(y[fit_lo + static_cast<int64_t>(m)]);
        out.timings.matrix_build_us = us_since(t0);
    }

    // ---- 6b. gain_fit (Householder QR of A; cond(A) from the SVD of R) ------
    bool basis_finite = true;
    {
        const auto t0 = std::chrono::steady_clock::now();
        for (size_t k = 0; k < nsel * Mfit; ++k) {
            if (!std::isfinite(scratch.A[k].real()) ||
                !std::isfinite(scratch.A[k].imag())) {
                basis_finite = false;
                break;
            }
        }
        // Householder QR solves the M x K least squares directly -- no normal
        // equations, so no cond(A)^2 amplification -- and `gain_condition` is
        // cond2(A) computed from the singular values of the factor R (see
        // detail::qr_cond_solve).  The old max|R_kk|/min|R_kk| pivot ratio was
        // NOT cond(A): it underestimated it (e.g. 666669.93 vs 1333333.33 for
        // A = [[1,1],[0,1.5e-6]]) and let ill-conditioned fits through the
        // ceiling.  Any failure here is fail-closed.
        const bool solve_ok = detail::qr_cond_solve(
            scratch.A.data(), Mfit, nsel, scratch.hh_b.data(), h,
            scratch.hh_v.data(), &cond);
        out.gain_condition = solve_ok ? cond : 0.0;
        out.timings.gain_fit_us = us_since(t0);
        if (!solve_ok || !basis_finite) {
            out.state = SicState::Failed;
            out.reject_reason = "gain_fit_failed";
            finish();
            return true;
        }
    }

    // Expose the final (fractionally refined) absolute fit delays for direct
    // cross-checking against sic_reference.py's result["taus"].
    out.n_taus = nsel;
    for (size_t k = 0; k < nsel; ++k)
        out.taus[k] = tau_abs[k];

    // ---- 7. reconstruct over the union of delayed-preamble support ---------
    // Paths are magnitude-sorted, so tau_abs[0] is NOT necessarily the smallest
    // delay: take the min/max over ALL selected paths.
    int64_t union_lo = static_cast<int64_t>(std::floor(tau_abs[0]));
    int64_t union_hi = static_cast<int64_t>(std::ceil(tau_abs[0])) +
                       static_cast<int64_t>(plen);
    for (size_t k = 1; k < nsel; ++k) {
        union_lo = std::min<int64_t>(union_lo,
                                     static_cast<int64_t>(std::floor(tau_abs[k])));
        union_hi = std::max<int64_t>(
            union_hi, static_cast<int64_t>(std::ceil(tau_abs[k])) +
                          static_cast<int64_t>(plen));
    }
    // sub_lo/sub_hi were resolved by the subtraction/commit ROI gate in step 4b:
    // support ∩ sub_roi when a sensing sub_roi is configured, else the legacy
    // support ∩ search-ROI mask.
    int64_t ih_lo = std::max<int64_t>(0, std::min<int64_t>(union_lo, sub_lo));
    int64_t ih_hi = std::min<int64_t>(
        static_cast<int64_t>(n), std::max<int64_t>(union_hi, sub_hi));
    if (ih_hi < ih_lo)
        ih_hi = ih_lo;
    const size_t ih_len = static_cast<size_t>(ih_hi - ih_lo);
    if (ih_len > scratch.cap_region) {
        out.state = SicState::Failed;
        out.reject_reason = "capacity_overflow";
        finish();
        return true;
    }
    {
        const auto t0 = std::chrono::steady_clock::now();
        std::fill(scratch.acc.begin(), scratch.acc.begin() + ih_len, dcp(0.0, 0.0));
        for (size_t k = 0; k < nsel; ++k)
            accum_shifted(scratch.acc.data(), ih_len, up, plen, ih_lo, tau_abs[k],
                          h[k]);
        // CFO rotate + complex combine, VOLK 32fc (same mask semantics: the
        // rotation/combine is applied to the full reconstruction window, and the
        // caller only subtracts inside the subtraction mask).
        reconstruct_combine(scratch.ihat.data(), scratch.acc.data(), ih_len,
                            ih_lo, w, scratch.rot_f.data(),
                            scratch.rot_base.data());
        out.timings.reconstruct_us = us_since(t0);
    }
    if (ihatbuf) {
        for (size_t i = 0; i < ih_len; ++i)
            ihatbuf[static_cast<size_t>(ih_lo + static_cast<int64_t>(i))] =
                scratch.ihat[i];
    }

    // ---- 8. validate_commit (disjoint holdout reps) ------------------------
    bool commit = false;
    {
        const auto t0 = std::chrono::steady_clock::now();
        const int64_t hstart = detected_start + static_cast<int64_t>(Mfit);
        const int64_t hend = hstart + static_cast<int64_t>(cfg.validation_reps * sps);
        // cond is cond2(A) (SVD of the QR factor R), the honest condition
        // number, so this gate is exactly `numpy.linalg.cond(A) < 1e6`.
        const bool cond_ok = cond < kMaxGainCond;
        if (hend <= static_cast<int64_t>(n) && cond_ok) {
            const size_t Mv = static_cast<size_t>(hend - hstart);
            double e_in = 0.0;
            for (size_t m = 0; m < Mv; ++m) {
                const int64_t abs_idx = hstart + static_cast<int64_t>(m);
                const cplx ih = (abs_idx >= ih_lo &&
                                 abs_idx < ih_hi)
                    ? scratch.ihat[static_cast<size_t>(abs_idx - ih_lo)]
                    : cplx(0.f, 0.f);
                scratch.resid[m] = dcp(y[abs_idx]) - dcp(ih);
                e_in += std::norm(scratch.resid[m]);
            }
            // normalized correlation with the delay-0 template over the holdout
            fill_rot(scratch.rot.data(), Mv, hstart, w);
            build_column(scratch.col.data(), Mv, up, plen, hstart,
                         static_cast<double>(detected_start), scratch.rot.data());
            dcp num(0.0, 0.0);
            double ec = 0.0;
            for (size_t m = 0; m < Mv; ++m) {
                num += std::conj(scratch.col[m]) * scratch.resid[m];
                ec += std::norm(scratch.col[m]);
            }
            const double den = std::sqrt(e_in) * std::sqrt(ec);
            const double rc = (den > 0.0) ? std::abs(num) / den : 1.0;
            out.residual_corr = rc;
            commit = rc < cfg.validate_threshold;
            if (!commit)
                out.reject_reason = "holdout_residual_high";
        } else {
            out.residual_corr = std::numeric_limits<double>::quiet_NaN();
            out.reject_reason = "validation_window_unavailable";
        }
        out.timings.validate_commit_us = us_since(t0);
    }

    if (!commit) {
        out.state = SicState::Bypassed;
        out.bypass_reason = "not_committed";
        if (!out.reject_reason)
            out.reject_reason = "not_committed";
        finish();
        return true;
    }

    const int64_t mlo = std::max<int64_t>(0, sub_lo);
    const int64_t mhi = std::min<int64_t>(static_cast<int64_t>(n), sub_hi);
    for (int64_t abs_idx = mlo; abs_idx < mhi; ++abs_idx) {
        const cplx ih = (abs_idx >= ih_lo && abs_idx < ih_hi)
            ? scratch.ihat[static_cast<size_t>(abs_idx - ih_lo)]
            : cplx(0.f, 0.f);
        outbuf[static_cast<size_t>(abs_idx)] =
            outbuf[static_cast<size_t>(abs_idx)] - ih;
    }
    out.state = SicState::Committed;
    out.committed = true;
    finish();
    return true;
}

} // namespace sic
} // namespace uwb
} // namespace gr
