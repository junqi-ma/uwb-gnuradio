/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Shared SYNC-CIR kernel (contract §1 of
 * docs/performance/接口契约_统一SYNC_CIR核与公共前导前端_2026-11-12.md).
 *
 * One header-only implementation of "coherently average the aligned SYNC
 * windows, then correlate the average against the sampled ternary preamble
 * code" used by BOTH the communication receiver (uwb_demod_core.h
 * `stage_cir_softchips`) and the sensing radar CIR path.  The kernel does not
 * depend on the full demodulator, the HRP modulator or any SIC header, so it
 * cannot create an include cycle.
 *
 * ---------------------------------------------------------------------------
 * Math contract (must be followed literally; the comm path is bit-identical)
 * ---------------------------------------------------------------------------
 * With wlen = code_len + pre + post - 1 and tap_count = pre + post:
 *
 *   1. avg[0 .. wlen) = 0
 *   2. for k = skip_initial .. skip_initial + count - 1,
 *        count = min(max_repetitions or remaining, total - skip_initial):
 *          rs = cir_axis_origin + llround((double)k * P)
 *          lo = rs - pre,  hi = lo + wlen
 *          lo < 0 || hi > n  ->  clipped, skipped, NOT counted in the divisor
 *          else avg += rx[lo .. lo + wlen), valid++
 *   3. valid == 0                       -> NoValidRepetitions
 *   4. avg *= 1.f / (float)valid
 *   5. correlate_cir_sparse(avg, tmpl, tap_count, raw)
 *        (the sparse correlation already divides by the float code_energy;
 *         do not scale again)
 *   6. nrm = sqrtf(sum_{i<tap_count} |raw[i]|^2)   (float, ascending order)
 *      nrm <= 0 or non-finite          -> NonFinite
 *   7. norm[i] = raw[i] / (nrm + 1e-12f)           (raw survives)
 *   8. peak_tap = argmax |raw[i]| (strictly greater, first winner),
 *      peak_abs = |raw[peak_tap]|
 *
 * Steps 4-7 are deliberately the exact float sequence of the legacy comm
 * `stage_cir_softchips`, so `norm` is bit-identical to its `values` for the
 * same input.  Do not "improve" them (no double accumulation, no reordering,
 * no extra scaling, no divide-before-correlate).
 *
 * ---------------------------------------------------------------------------
 * code_len
 * ---------------------------------------------------------------------------
 * `code_len` is the CORRELATION WINDOW length in samples, i.e.
 * 2 * kQm35ChipsPerSymbol (1016), NOT the ternary code length (127).  The
 * ternary length is derived as code_len / (kQm35SamplesPerChip *
 * kQm35SpreadingFactor) = 127 and is what build_sparse_cir_template consumes.
 * prepare_sync_code rejects any other code_len so a caller that passes the
 * ternary length fails closed instead of over-reading the code array.
 *
 * ---------------------------------------------------------------------------
 * Failure semantics (fail-closed)
 * ---------------------------------------------------------------------------
 * Every failure path resets `out` (tap_count = 0, peak_tap = 0, peak_abs = 0,
 * raw_l2_norm = 0, valid_repetitions = 0, first/last_window_sample = -1) and
 * zeroes the raw/norm tap buffers, so no stale or seed-only CIR can be
 * mistaken for a result.
 *
 *   InvalidInput          null rx, n == 0, unprepared code, pre + post == 0,
 *                         P <= 0 or non-finite, origin < 0,
 *                         total_repetitions == 0, skip_initial >= total.
 *                         prepare_sync_code / required_input_range additionally
 *                         reject code_len == 0 and any code_len != 1016.
 *   CapacityInsufficient  wlen / tap_count exceed the capacity reserved by
 *                         prepare_sync_code (must not happen after prepare)
 *   NoValidRepetitions    every window was clipped by the buffer edge
 *   NonFinite             the input (or the resulting correlation) is not
 *                         finite; detected at step 6
 *
 * ---------------------------------------------------------------------------
 * Allocation
 * ---------------------------------------------------------------------------
 * prepare_sync_code is the only allocating entry point.  estimate_sync_cir and
 * required_input_range never allocate or grow a vector: the capacity checks
 * run before any write, and correlate_cir_sparse's resize(tap_count) stays
 * inside the capacity reserved at prepare time.
 *
 * Threading: SyncCirScratch / PreparedSyncCode are per worker.  Nothing here
 * is shared between workers.  The kernel does not estimate interference, does
 * not read ground truth, never rotates the input and never mutates it: `rx`
 * must already be compensated with the final CFO.
 */

#pragma once

// Include guard marker.  uwb_demod_core.h used to declare a field-identical
// fallback `sync_cir::SyncCirRequest` and probe this macro; that fallback has
// been removed, so the type below is the only definition in the tree.  The
// marker is kept so a translation unit that pre-defines it still compiles.
#ifndef GR_UWB_SYNC_CIR_CORE_H
#define GR_UWB_SYNC_CIR_CORE_H
#endif

#include <gnuradio/uwb/uwb_phy_profile.h> // kQm35* constants, BuildSampledCode
#include <gnuradio/uwb/uwb_rx_cfo_opt.h>  // SparseCirTemplate, sparse correlate
#include <gnuradio/uwb/uwb_rx_sync_opt.h> // vector accumulate/scale

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

namespace gr {
namespace uwb {
namespace demod {
namespace sync_cir {

enum class SyncCirStatus : uint8_t {
    Ok = 0,
    InvalidInput = 1,         // null/zero length/bad pre-post/not prepared
    CapacityInsufficient = 2, // scratch capacity too small (not after prepare)
    NoValidRepetitions = 3,   // every window clipped by the buffer edge
    NonFinite = 4             // input or output contains a non-finite value
};

// Correlation window length in samples (2 * chips per symbol).  See the
// header comment: this is NOT the ternary code length.
inline constexpr size_t kSyncCirCodeLength = 2 * kQm35ChipsPerSymbol;

// Canonical request description (contract §1.1).  This is the only definition
// of the type in the tree: uwb_demod_core.h includes this header instead of
// declaring a fallback.  The macro guard is retained for source compatibility
// with a translation unit that pre-defines it.
#ifndef GR_UWB_SYNC_CIR_REQUEST_DEFINED
#define GR_UWB_SYNC_CIR_REQUEST_DEFINED 1
struct SyncCirRequest {
    int64_t cir_axis_origin = -1;    // absolute sample of tap-0 reference
    double samples_per_symbol = 0.0; // repetition period P, may be fractional
    size_t total_repetitions = 0;    // total SYNC symbols in the preamble
    size_t skip_initial = 0;         // skip the first N repetitions
    size_t max_repetitions = 0;      // 0 = consume all remaining
    size_t pre = 0;
    size_t post = 0;
};
#endif

struct PreparedSyncCode {
    bool ready = false;
    size_t code_index = 0;         // 9..12, identity record only
    uint64_t fingerprint = 0;      // FNV-1a of the raw ternary code
    rxopt::SparseCirTemplate tmpl; // entries + code_energy (existing impl)
};

struct SyncCirScratch {
    std::vector<std::complex<float>> avg;  // size >= code_len + max_taps - 1
    std::vector<std::complex<float>> raw;  // size >= max_taps
    std::vector<std::complex<float>> norm; // size >= max_taps
};

struct SyncCirResult {
    SyncCirStatus status = SyncCirStatus::InvalidInput;
    size_t tap_count = 0;
    size_t valid_repetitions = 0;
    size_t first_repetition = 0;
    size_t peak_tap = 0;
    float peak_abs = 0.f;
    float raw_l2_norm = 0.f;
    float code_energy = 0.f;
    int64_t first_window_sample = -1; // absolute lo of the first valid window
    int64_t last_window_sample = -1;  // absolute lo of the last valid window
};

namespace detail {

// Same constant as the legacy comm normalisation.
inline constexpr float kNormEpsilon = 1e-12f;

inline bool add_overflow(size_t a, size_t b)
{
    return b > std::numeric_limits<size_t>::max() - a;
}

inline bool to_i64(size_t v, int64_t& out)
{
    if (v > static_cast<size_t>(std::numeric_limits<int64_t>::max()))
        return false;
    out = static_cast<int64_t>(v);
    return true;
}

inline bool add_i64(int64_t a, int64_t b, int64_t& out)
{
    if ((b > 0 && a > std::numeric_limits<int64_t>::max() - b) ||
        (b < 0 && a < std::numeric_limits<int64_t>::min() - b))
        return false;
    out = a + b;
    return true;
}

inline bool sub_i64(int64_t a, int64_t b, int64_t& out)
{
    if ((b < 0 && a > std::numeric_limits<int64_t>::max() + b) ||
        (b > 0 && a < std::numeric_limits<int64_t>::min() + b))
        return false;
    out = a - b;
    return true;
}

// llround(k * P) with an explicit range guard: 2^62 keeps the rounding well
// inside int64 and any larger offset is clipped by the buffer edge anyway.
inline bool repetition_offset(size_t k, double period, int64_t& out)
{
    constexpr double kOffsetLimit = 4611686018427387904.0; // 2^62
    const double offset = static_cast<double>(k) * period;
    if (!(offset >= -kOffsetLimit && offset <= kOffsetLimit))
        return false;
    out = static_cast<int64_t>(std::llround(offset));
    return true;
}

// Number of repetitions to visit: min(max_repetitions or remaining, remaining).
// Requires skip_initial < total_repetitions (validated by valid_geometry).
inline size_t repetition_count(const SyncCirRequest& req)
{
    const size_t remaining = req.total_repetitions - req.skip_initial;
    if (req.max_repetitions == 0)
        return remaining;
    return std::min(req.max_repetitions, remaining);
}

// Shared request geometry: tap_count / wlen, with every size arithmetic checked.
inline bool valid_geometry(const SyncCirRequest& req,
                           size_t code_len,
                           size_t& tap_count,
                           size_t& wlen)
{
    tap_count = 0;
    wlen = 0;
    if (code_len == 0 || req.total_repetitions == 0)
        return false;
    if (req.skip_initial >= req.total_repetitions)
        return false;
    if (req.cir_axis_origin < 0)
        return false;
    if (!std::isfinite(req.samples_per_symbol) || !(req.samples_per_symbol > 0.0))
        return false;
    if (add_overflow(req.pre, req.post))
        return false;
    tap_count = req.pre + req.post;
    if (tap_count == 0)
        return false;
    if (add_overflow(code_len, tap_count - 1))
        return false;
    wlen = code_len + tap_count - 1;
    return true;
}

// Visits every window that survives the clip rule, in ascending k order.
// Both estimate_sync_cir and required_input_range use this so the accepted set
// can never diverge.  Returns false only when the geometry cannot be expressed
// in int64 (treated as InvalidInput by the callers).
template <typename Fn>
inline bool visit_valid_windows(const SyncCirRequest& req,
                                size_t wlen,
                                size_t n,
                                Fn&& fn)
{
    int64_t n64 = 0;
    int64_t pre64 = 0;
    int64_t wlen64 = 0;
    if (!to_i64(n, n64) || !to_i64(req.pre, pre64) || !to_i64(wlen, wlen64))
        return false;

    const size_t count = repetition_count(req);
    for (size_t k = req.skip_initial; k < req.skip_initial + count; ++k) {
        int64_t offset = 0;
        if (!repetition_offset(k, req.samples_per_symbol, offset))
            continue; // absurd offset: clipped by definition
        int64_t rs = 0;
        int64_t lo = 0;
        int64_t hi = 0;
        if (!add_i64(req.cir_axis_origin, offset, rs) ||
            !sub_i64(rs, pre64, lo) || !add_i64(lo, wlen64, hi))
            continue;
        if (lo < 0 || hi > n64)
            continue;
        fn(k, lo);
    }
    return true;
}

inline void zero_taps(SyncCirScratch& scratch)
{
    std::fill(scratch.raw.begin(), scratch.raw.end(),
              std::complex<float>(0.f, 0.f));
    std::fill(scratch.norm.begin(), scratch.norm.end(),
              std::complex<float>(0.f, 0.f));
}

inline bool code_ready(const PreparedSyncCode& prepared)
{
    return prepared.ready && !prepared.tmpl.entries.empty() &&
           std::isfinite(prepared.tmpl.code_energy) &&
           prepared.tmpl.code_energy > 0.0f;
}

} // namespace detail

// May allocate.  Call at init / whenever the TX preamble code changes.
//
// code_len is the correlation window length (kSyncCirCodeLength == 1016); the
// ternary code length consumed by build_sparse_cir_template is derived from it.
// Sizes avg to code_len + max_pre + max_post - 1 and raw/norm to
// max_pre + max_post; on any failure prepared.ready stays false and the
// scratch vectors are left empty.
inline bool prepare_sync_code(const int8_t* code,
                              size_t code_len,
                              size_t code_index,
                              size_t max_pre,
                              size_t max_post,
                              PreparedSyncCode& prepared,
                              SyncCirScratch& scratch)
{
    prepared = PreparedSyncCode{};
    scratch.avg.clear();
    scratch.raw.clear();
    scratch.norm.clear();

    if (!code || code_len == 0 || code_len != kSyncCirCodeLength)
        return false;
    if (detail::add_overflow(max_pre, max_post))
        return false;
    const size_t max_taps = max_pre + max_post;
    if (max_taps == 0)
        return false;
    if (detail::add_overflow(code_len, max_taps - 1))
        return false;
    const size_t max_wlen = code_len + max_taps - 1;

    // ternary length = window length / (samples per chip * spreading factor)
    constexpr size_t kWindowPerTernary =
        kQm35SamplesPerChip * kQm35SpreadingFactor;
    if (kWindowPerTernary == 0 || code_len % kWindowPerTernary != 0)
        return false;
    const size_t ternary_len = code_len / kWindowPerTernary;
    if (ternary_len == 0 || ternary_len > kQm35CodeLength)
        return false;

    rxopt::build_sparse_cir_template(code, ternary_len, prepared.tmpl);
    if (prepared.tmpl.entries.empty() || !(prepared.tmpl.code_energy > 0.0f) ||
        !std::isfinite(prepared.tmpl.code_energy))
        return false;

    prepared.code_index = code_index;
    prepared.fingerprint = rxopt::cir_template_key(code, ternary_len);
    prepared.ready = true;

    scratch.avg.assign(max_wlen, std::complex<float>(0.f, 0.f));
    scratch.raw.assign(max_taps, std::complex<float>(0.f, 0.f));
    scratch.norm.assign(max_taps, std::complex<float>(0.f, 0.f));
    return true;
}

// Pure computation, no allocation.  Returns the union envelope [lo, hi) of all
// windows that estimate_sync_cir would accept, so a caller can plan the final
// CFO derotation coverage.  The clip rule is identical to estimate_sync_cir;
// the returned range is the un-clipped envelope of the accepted windows.
// Returns false (lo = hi = -1) when no window is accepted or the request is
// invalid.
inline bool required_input_range(const SyncCirRequest& req,
                                 size_t code_len,
                                 size_t n,
                                 int64_t& lo,
                                 int64_t& hi)
{
    lo = -1;
    hi = -1;

    size_t tap_count = 0;
    size_t wlen = 0;
    if (n == 0 || !detail::valid_geometry(req, code_len, tap_count, wlen))
        return false;

    int64_t env_lo = -1;
    int64_t env_hi = -1;
    const bool visited = detail::visit_valid_windows(
        req, wlen, n, [&](size_t /*k*/, int64_t window_lo) {
            if (env_lo < 0 || window_lo < env_lo)
                env_lo = window_lo;
            // window_lo + wlen <= n <= INT64_MAX, checked by the visitor.
            const int64_t window_hi = window_lo + static_cast<int64_t>(wlen);
            if (window_hi > env_hi)
                env_hi = window_hi;
        });
    if (!visited || env_lo < 0 || env_hi <= env_lo)
        return false;

    lo = env_lo;
    hi = env_hi;
    return true;
}

// HOT PATH: no allocation, no vector growth.  `rx` must already be derotated
// with the FINAL CFO (phase reference is the caller's contract).  Taps are
// written to scratch.raw / scratch.norm in [0, tap_count).
inline bool estimate_sync_cir(const std::complex<float>* rx,
                              size_t n,
                              const SyncCirRequest& req,
                              const PreparedSyncCode& prepared,
                              SyncCirScratch& scratch,
                              SyncCirResult& out)
{
    out = SyncCirResult{};

    size_t tap_count = 0;
    size_t wlen = 0;
    const bool geometry_ok =
        detail::valid_geometry(req, kSyncCirCodeLength, tap_count, wlen);
    if (!rx || n == 0 || !geometry_ok || !detail::code_ready(prepared)) {
        out.status = SyncCirStatus::InvalidInput;
        detail::zero_taps(scratch);
        return false;
    }

    // Capacity was reserved by prepare_sync_code; the size checks keep a
    // hand-built scratch from writing past the constructed elements.
    if (wlen > scratch.avg.capacity() || tap_count > scratch.raw.capacity() ||
        tap_count > scratch.norm.capacity() || scratch.avg.size() < wlen ||
        scratch.norm.size() < tap_count) {
        out.status = SyncCirStatus::CapacityInsufficient;
        detail::zero_taps(scratch);
        return false;
    }

    // Step 1: clear the averaged window.
    std::fill(scratch.avg.begin(),
              scratch.avg.begin() + static_cast<std::ptrdiff_t>(wlen),
              std::complex<float>(0.f, 0.f));

    // Step 2: coherently accumulate every window that survives the clip rule.
    size_t valid = 0;
    int64_t first_window = -1;
    int64_t last_window = -1;
    const bool visited = detail::visit_valid_windows(
        req, wlen, n, [&](size_t /*k*/, int64_t window_lo) {
            rxopt::vector_accumulate_complex(
                scratch.avg.data(),
                rx + static_cast<size_t>(window_lo),
                wlen);
            if (first_window < 0)
                first_window = window_lo;
            last_window = window_lo;
            ++valid;
        });

    // Step 3: no usable repetition -> fail closed, do not divide by zero.
    if (!visited || valid == 0) {
        out.status = visited ? SyncCirStatus::NoValidRepetitions
                             : SyncCirStatus::InvalidInput;
        detail::zero_taps(scratch);
        return false;
    }

    // Step 4: average (same float form as the legacy comm stage).
    const float inv = 1.0f / static_cast<float>(valid);
    rxopt::vector_scale_complex(scratch.avg.data(), inv, wlen);

    // Step 5: sparse ternary correlation, already divided by code_energy.
    rxopt::correlate_cir_sparse(
        scratch.avg.data(), prepared.tmpl, tap_count, scratch.raw);

    // Step 6: float L2 norm in ascending tap order.
    float nrm = 0.0f;
    for (size_t i = 0; i < tap_count; ++i)
        nrm += std::norm(scratch.raw[i]);
    nrm = std::sqrt(nrm);
    if (!std::isfinite(nrm) || !(nrm > 0.0f)) {
        out.status = SyncCirStatus::NonFinite;
        detail::zero_taps(scratch);
        return false;
    }

    // Steps 7-8: normalise into `norm` (raw survives) and pick the peak from
    // the raw taps (strictly greater, first winner).
    const float denom = nrm + detail::kNormEpsilon;
    size_t peak_tap = 0;
    float peak_abs = -1.0f;
    for (size_t i = 0; i < tap_count; ++i) {
        const std::complex<float> raw = scratch.raw[i];
        scratch.norm[i] = raw / denom;
        const float mag = std::abs(raw);
        if (mag > peak_abs) {
            peak_abs = mag;
            peak_tap = i;
        }
    }

    out.status = SyncCirStatus::Ok;
    out.tap_count = tap_count;
    out.valid_repetitions = valid;
    out.first_repetition = req.skip_initial;
    out.peak_tap = peak_tap;
    out.peak_abs = peak_abs;
    out.raw_l2_norm = nrm;
    out.code_energy = prepared.tmpl.code_energy;
    out.first_window_sample = first_window;
    out.last_window_sample = last_window;
    return true;
}

} // namespace sync_cir
} // namespace demod
} // namespace uwb
} // namespace gr
