// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <openuwb/phy/hrp/rx/rx_sync_opt.h>

namespace openuwb::demod::rxopt {

struct SparseCirEntry {
    size_t pos = 0;      // sample index in [0, 1016), always even: 2 * j
    int8_t sign = 0;     // +1 or -1
};

// Compact, L1-resident offset tables.  Offsets are FLOAT offsets (twice the
// complex sample index), exactly what the wide kernels index with.  A ternary
// HRP code has at most 127 non-zero chips, so 128 entries are enough for every
// profile; the build clamps instead of overflowing.
inline constexpr size_t kMaxSparseCirOffsets = 128;

struct SparseCirTemplate {
    uint64_t key = 0;
    std::vector<SparseCirEntry> entries;
    // Kept for compatibility with callers that still read the vectors.
    std::vector<size_t> pos_offsets;
    std::vector<size_t> neg_offsets;
    // Hot-path tables: one 64-byte cache line each, no vector indirection.
    alignas(64) uint16_t pos_off[kMaxSparseCirOffsets] = {};
    alignas(64) uint16_t neg_off[kMaxSparseCirOffsets] = {};
    uint16_t pos_count = 0;
    uint16_t neg_count = 0;
    float code_energy = 0.0f;

    void reset()
    {
        key = 0;
        entries.clear();
        pos_offsets.clear();
        neg_offsets.clear();
        pos_count = 0;
        neg_count = 0;
        code_energy = 0.0f;
    }
};

inline uint64_t cir_template_key(const int8_t* code, size_t code_len)
{
    uint64_t h = 14695981039346656037ULL;
    for (size_t i = 0; i < code_len; ++i) {
        h ^= static_cast<uint8_t>(code[i]);
        h *= 1099511628211ULL;
    }
    return h;
}

inline void build_sparse_cir_template(const int8_t* code,
                                      size_t code_len,
                                      SparseCirTemplate& tmpl)
{
    tmpl.reset();
    tmpl.key = cir_template_key(code, code_len);
    const std::vector<int8_t> spread = BuildSampledCode(code, code_len);
    tmpl.entries.reserve(code_len);
    for (size_t j = 0; j < spread.size(); ++j) {
        if (spread[j] != 0) {
            const size_t pos = 2 * j;
            tmpl.entries.push_back({ pos, spread[j] });
            if (spread[j] > 0) {
                tmpl.pos_offsets.push_back(2 * pos);
                if (tmpl.pos_count < kMaxSparseCirOffsets)
                    tmpl.pos_off[tmpl.pos_count++] =
                        static_cast<uint16_t>(2 * pos);
            } else {
                tmpl.neg_offsets.push_back(2 * pos);
                if (tmpl.neg_count < kMaxSparseCirOffsets)
                    tmpl.neg_off[tmpl.neg_count++] =
                        static_cast<uint16_t>(2 * pos);
            }
        }
    }
    tmpl.code_energy = static_cast<float>(tmpl.entries.size());
}

// Single-lag dot.  The template is fixed for the whole scan, so the sign
// sequence is the SAME on every call and the branch predictor learns it; a
// branchless `sign * x` instead adds two float multiplies per tap and measured
// ~20% slower in the live wide-window coarse scan (18.7k calls x 127 taps).
inline std::complex<float> sparse_ternary_dot_single(
    const std::complex<float>* rx,
    const SparseCirTemplate& tmpl)
{
    std::complex<float> acc(0.0f, 0.0f);
    for (const auto& e : tmpl.entries) {
        const std::complex<float>& s = rx[e.pos];
        if (e.sign > 0)
            acc += s;
        else
            acc -= s;
    }
    return acc;
}

inline void sparse_ternary_dot_8lag(
    const std::complex<float>* rx,
    const SparseCirTemplate& tmpl,
    std::complex<float> out[8])
{
#if defined(__AVX2__) && defined(__FMA__)
    const float* rx_f = reinterpret_cast<const float*>(rx);
    __m256 p0 = _mm256_setzero_ps();
    __m256 p1 = _mm256_setzero_ps();
    for (size_t i = 0; i < tmpl.pos_count; ++i) {
        const size_t off = tmpl.pos_off[i];
        p0 = _mm256_add_ps(p0, _mm256_loadu_ps(rx_f + off));
        p1 = _mm256_add_ps(p1, _mm256_loadu_ps(rx_f + off + 8));
    }
    __m256 n0 = _mm256_setzero_ps();
    __m256 n1 = _mm256_setzero_ps();
    for (size_t i = 0; i < tmpl.neg_count; ++i) {
        const size_t off = tmpl.neg_off[i];
        n0 = _mm256_add_ps(n0, _mm256_loadu_ps(rx_f + off));
        n1 = _mm256_add_ps(n1, _mm256_loadu_ps(rx_f + off + 8));
    }
    _mm256_storeu_ps(reinterpret_cast<float*>(out), _mm256_sub_ps(p0, n0));
    _mm256_storeu_ps(reinterpret_cast<float*>(out + 4), _mm256_sub_ps(p1, n1));
#else
    for (int m = 0; m < 8; ++m)
        out[m] = std::complex<float>(0.0f, 0.0f);
    for (const auto& e : tmpl.entries) {
        const size_t p = e.pos;
        if (e.sign > 0) {
            for (int m = 0; m < 8; ++m)
                out[m] += rx[p + m];
        } else {
            for (int m = 0; m < 8; ++m)
                out[m] -= rx[p + m];
        }
    }
#endif
}

#if UWB_RXOPT_HAVE_AVX2
__attribute__((target("avx2,fma")))
inline void sparse_ternary_dot_4lag_avx2(const std::complex<float>* rx,
                                         const SparseCirTemplate& tmpl,
                                         std::complex<float> out[4])
{
    const float* rx_f = reinterpret_cast<const float*>(rx);
    __m256 p = _mm256_setzero_ps();
    for (size_t i = 0; i < tmpl.pos_count; ++i)
        p = _mm256_add_ps(p, _mm256_loadu_ps(rx_f + tmpl.pos_off[i]));
    __m256 n = _mm256_setzero_ps();
    for (size_t i = 0; i < tmpl.neg_count; ++i)
        n = _mm256_add_ps(n, _mm256_loadu_ps(rx_f + tmpl.neg_off[i]));
    _mm256_storeu_ps(reinterpret_cast<float*>(out), _mm256_sub_ps(p, n));
}

__attribute__((target("avx2,fma")))
inline void sparse_ternary_dot_16lag_avx2(const std::complex<float>* rx,
                                          const SparseCirTemplate& tmpl,
                                          std::complex<float> out[16])
{
    const float* rx_f = reinterpret_cast<const float*>(rx);
    __m256 p0 = _mm256_setzero_ps();
    __m256 p1 = _mm256_setzero_ps();
    __m256 p2 = _mm256_setzero_ps();
    __m256 p3 = _mm256_setzero_ps();
    for (size_t i = 0; i < tmpl.pos_count; ++i) {
        const size_t off = tmpl.pos_off[i];
        p0 = _mm256_add_ps(p0, _mm256_loadu_ps(rx_f + off));
        p1 = _mm256_add_ps(p1, _mm256_loadu_ps(rx_f + off + 8));
        p2 = _mm256_add_ps(p2, _mm256_loadu_ps(rx_f + off + 16));
        p3 = _mm256_add_ps(p3, _mm256_loadu_ps(rx_f + off + 24));
    }
    __m256 n0 = _mm256_setzero_ps();
    __m256 n1 = _mm256_setzero_ps();
    __m256 n2 = _mm256_setzero_ps();
    __m256 n3 = _mm256_setzero_ps();
    for (size_t i = 0; i < tmpl.neg_count; ++i) {
        const size_t off = tmpl.neg_off[i];
        n0 = _mm256_add_ps(n0, _mm256_loadu_ps(rx_f + off));
        n1 = _mm256_add_ps(n1, _mm256_loadu_ps(rx_f + off + 8));
        n2 = _mm256_add_ps(n2, _mm256_loadu_ps(rx_f + off + 16));
        n3 = _mm256_add_ps(n3, _mm256_loadu_ps(rx_f + off + 24));
    }
    float* o = reinterpret_cast<float*>(out);
    _mm256_storeu_ps(o + 0, _mm256_sub_ps(p0, n0));
    _mm256_storeu_ps(o + 8, _mm256_sub_ps(p1, n1));
    _mm256_storeu_ps(o + 16, _mm256_sub_ps(p2, n2));
    _mm256_storeu_ps(o + 24, _mm256_sub_ps(p3, n3));
}
#endif

inline void sparse_ternary_dot_4lag(const std::complex<float>* rx,
                                    const SparseCirTemplate& tmpl,
                                    std::complex<float> out[4])
{
#if UWB_RXOPT_HAVE_AVX2
    if (has_avx2_fma()) {
        sparse_ternary_dot_4lag_avx2(rx, tmpl, out);
        return;
    }
#endif
    for (int m = 0; m < 4; ++m)
        out[m] = sparse_ternary_dot_single(rx + m, tmpl);
}

inline void sparse_ternary_dot_16lag(const std::complex<float>* rx,
                                     const SparseCirTemplate& tmpl,
                                     std::complex<float> out[16])
{
#if UWB_SIMD_HAVE_AVX512
    if (use_avx512(openuwb::core::simd::KernelGroup::Sync)) {
        openuwb::core::simd::avx512::sparse_ternary_dot_16lag(
            rx, tmpl.pos_off, tmpl.pos_count, tmpl.neg_off, tmpl.neg_count,
            out);
        return;
    }
#endif
#if UWB_RXOPT_HAVE_AVX2
    if (has_avx2_fma()) {
        sparse_ternary_dot_16lag_avx2(rx, tmpl, out);
        return;
    }
#endif
    sparse_ternary_dot_8lag(rx, tmpl, out);
    sparse_ternary_dot_8lag(rx + 8, tmpl, out + 8);
}

inline void sparse_ternary_dot_32lag(const std::complex<float>* rx,
                                     const SparseCirTemplate& tmpl,
                                     std::complex<float> out[32])
{
#if UWB_SIMD_HAVE_AVX512
    if (use_avx512(openuwb::core::simd::KernelGroup::Sync)) {
        openuwb::core::simd::avx512::sparse_ternary_dot_32lag(
            rx, tmpl.pos_off, tmpl.pos_count, tmpl.neg_off, tmpl.neg_count,
            out);
        return;
    }
#endif
    sparse_ternary_dot_16lag(rx, tmpl, out);
    sparse_ternary_dot_16lag(rx + 16, tmpl, out + 16);
}

inline void ensure_sparse_cir_template(const int8_t* code,
                                       size_t code_len,
                                       SparseCirTemplate& tmpl)
{
    if (tmpl.key != cir_template_key(code, code_len))
        build_sparse_cir_template(code, code_len, tmpl);
}

inline void build_half_rate_sparse_template(const SparseCirTemplate& full,
                                            SparseCirTemplate& half)
{
    half.reset();
    half.key = full.key ^ 0x9e3779b97f4a7c15ULL;
    half.code_energy = full.code_energy;
    half.entries.reserve(full.entries.size());
    for (const auto& e : full.entries)
        half.entries.push_back({ e.pos / 2, e.sign });
    half.pos_offsets.reserve(full.pos_offsets.size());
    for (const size_t off : full.pos_offsets)
        half.pos_offsets.push_back(off / 2);
    half.neg_offsets.reserve(full.neg_offsets.size());
    for (const size_t off : full.neg_offsets)
        half.neg_offsets.push_back(off / 2);
    half.pos_count = full.pos_count;
    for (size_t i = 0; i < full.pos_count; ++i)
        half.pos_off[i] = static_cast<uint16_t>(full.pos_off[i] / 2);
    half.neg_count = full.neg_count;
    for (size_t i = 0; i < full.neg_count; ++i)
        half.neg_off[i] = static_cast<uint16_t>(full.neg_off[i] / 2);
}

// Rebuild only when the full-template key changed.  stride=2 acquisition runs
// once per candidate, so this removes a hot-path rebuild/allocation.
inline void ensure_half_rate_sparse_template(const SparseCirTemplate& full,
                                             SparseCirTemplate& half)
{
    if (half.key != (full.key ^ 0x9e3779b97f4a7c15ULL) ||
        half.pos_count != full.pos_count || half.neg_count != full.neg_count)
        build_half_rate_sparse_template(full, half);
}

inline void pack_even_complex(const std::complex<float>* src,
                              size_t even_count,
                              std::complex<float>* dst)
{
#if UWB_SIMD_HAVE_AVX512
    if (use_avx512(openuwb::core::simd::KernelGroup::Sync)) {
        openuwb::core::simd::avx512::pack_even_complex(src, even_count, dst);
        return;
    }
#endif
    for (size_t i = 0; i < even_count; ++i)
        dst[i] = src[2 * i];
}

// ---------------------------------------------------------------------------
// MF-4: (r * code) * p factorization of the dense pulse-shaped SYNC template.
//
// The dense template is, by construction, the ternary Ipatov code convolved
// with the TX pulse p (see add_sync_symbol / kPulse48):
//     template[k] = sum_c code[c] * p[k - 8*c]
// so correlation with the dense template factors exactly:
//     r * template = (r * code) * p        [associativity]
// The first part is the existing 64-entry sparse add/sub kernel; the second is
// a short real FIR. This recovers the pulse-matched gain of the 1016-tap
// template (the sparse path only samples one point per pulse, losing ~2.7 dB)
// while keeping the sparse kernel's throughput.
//
// Only the first `nt` taps of p can be taken from the template without
// contamination: pulses are 8 samples apart, so template[base + 8] already
// contains the next chip. `nt` is clamped to 8 and to whatever the last code
// offset allows, which keeps every c[m] read inside the caller's L-sample
// contract. The omitted tail carries ~0.3% of the template energy
// (kPulse48: 99.7% in the first 8 taps), i.e. <0.02 dB.
// ---------------------------------------------------------------------------
struct ShapedSparseTemplate {
    static constexpr size_t kMaxPulse = 8;
    SparseCirTemplate code;
    float pulse[kMaxPulse] = {};
    uint8_t nt = 0;
    float energy = 0.0f; // sum |template|^2 (the dense normalizer)
    uint64_t key = 0;
    bool valid = false;

    void reset()
    {
        code.reset();
        nt = 0;
        energy = 0.0f;
        key = 0;
        valid = false;
    }
};

// FNV-1a over the template words, matching the real-template key style.
inline uint64_t shaped_template_key(const std::complex<float>* tmpl, size_t len)
{
    uint64_t h = 14695981039346656037ULL;
    for (size_t k = 0; k < len; ++k) {
        const float parts[2] = { tmpl[k].real(), tmpl[k].imag() };
        uint32_t bits[2];
        std::memcpy(bits, parts, sizeof(bits));
        h = (h ^ static_cast<uint64_t>(bits[0])) * 1099511628211ULL;
        h = (h ^ static_cast<uint64_t>(bits[1])) * 1099511628211ULL;
    }
    return h;
}

// Build (or reject) the factorized form. The gate is a reconstruction check:
// a template that is not code*p (e.g. a different pulse or a non-real
// waveform) leaves `valid == false`, and the caller keeps its old path.
inline bool build_shaped_sparse_template(const std::complex<float>* tmpl,
                                         size_t len,
                                         const SparseCirTemplate& code,
                                         ShapedSparseTemplate& out)
{
    out.reset();
    if (!tmpl || len < 32 || code.entries.empty() ||
        code.entries.size() > kMaxSparseCirOffsets)
        return false;
    const SparseCirEntry& first = code.entries.front();
    const size_t base = first.pos;
    const size_t last = code.entries.back().pos;
    if (first.sign == 0 || base >= len || last >= len)
        return false;
    const size_t nt = std::min(ShapedSparseTemplate::kMaxPulse, len - last);
    if (nt < 3)
        return false;

    double energy = 0.0;
    double imag_energy = 0.0;
    for (size_t k = 0; k < len; ++k) {
        energy += static_cast<double>(std::norm(tmpl[k]));
        imag_energy += static_cast<double>(tmpl[k].imag()) *
                       static_cast<double>(tmpl[k].imag());
    }
    if (energy <= 0.0 || imag_energy > 1e-12 * energy)
        return false;

    // First pulse is uncontaminated up to the next chip (8 samples).
    const float sign0 = static_cast<float>(first.sign);
    for (size_t k = 0; k < nt; ++k)
        out.pulse[k] = tmpl[base + k].real() / sign0;

    // Reconstruction gate. The residual is the omitted pulse tail, which is
    // ~0.3% of the energy for kPulse48; anything above 5% is not code*p.
    double resid = 0.0;
    for (const auto& e : code.entries) {
        for (size_t k = 0; k < nt; ++k) {
            const size_t idx = e.pos + k;
            if (idx >= len)
                break;
            const double rec =
                static_cast<double>(e.sign) * out.pulse[k];
            const double d = rec - tmpl[idx].real();
            resid += d * d;
        }
    }
    if (!(resid <= 0.05 * energy))
        return false;

    out.code = code;
    out.nt = static_cast<uint8_t>(nt);
    out.energy = static_cast<float>(energy);
    out.key = shaped_template_key(tmpl, len) ^ code.key;
    out.valid = true;
    return true;
}

// Cache: rebuild only when the template contents or the code change.
inline const ShapedSparseTemplate* ensure_shaped_sparse_template(
    const std::complex<float>* tmpl, size_t len,
    const SparseCirTemplate& code, ShapedSparseTemplate& out)
{
    const uint64_t key = shaped_template_key(tmpl, len) ^ code.key;
    if (out.valid && out.key == key)
        return &out;
    if (!build_shaped_sparse_template(tmpl, len, code, out))
        return nullptr;
    return &out;
}

// c[0..7] then the short real FIR. The 8-lag kernel needs rx[0 .. last+7],
// which is exactly inside the L-sample contract (L - last >= nt).
inline std::complex<float> shaped_ternary_dot_single(
    const std::complex<float>* rx,
    const ShapedSparseTemplate& tmpl)
{
    std::complex<float> c[8];
    sparse_ternary_dot_8lag(rx, tmpl.code, c);
    std::complex<float> acc(0.0f, 0.0f);
    for (size_t k = 0; k < tmpl.nt; ++k)
        acc += tmpl.pulse[k] * c[k];
    return acc;
}

// Contiguous lags: one 32-lag sparse batch plus (nt-1) scalar tail lags, then
// the FIR. Calling the kernel once per output lag would recompute the same
// sparse sums 8 times; sharing them across the block is where the speedup is.
template <typename Visit>
inline void shaped_visit_lags(const std::complex<float>* rx, size_t lag_count,
                              const ShapedSparseTemplate& tmpl, Visit& visit)
{
    constexpr size_t kBatch = 32;
    const size_t nt = tmpl.nt;
    std::complex<float> c[kBatch + ShapedSparseTemplate::kMaxPulse];
    size_t lag = 0;
    for (; lag + kBatch <= lag_count; lag += kBatch) {
        sparse_ternary_dot_32lag(rx + lag, tmpl.code, c);
        for (size_t k = 1; k < nt; ++k)
            c[kBatch - 1 + k] = sparse_ternary_dot_single(
                rx + lag + kBatch - 1 + k, tmpl.code);
        for (size_t j = 0; j < kBatch; ++j) {
            std::complex<float> acc(0.0f, 0.0f);
            for (size_t k = 0; k < nt; ++k)
                acc += tmpl.pulse[k] * c[j + k];
            visit(lag + j, acc, true);
        }
    }
    for (; lag < lag_count; ++lag)
        visit(lag, shaped_ternary_dot_single(rx + lag, tmpl), false);
}

// A non-owning view over a single SYNC symbol template. Sparse batches and
// their scalar tails deliberately retain their respective accumulation orders.
class SyncMatchedFilter {
public:
    SyncMatchedFilter(const std::complex<float>* dense,
                      size_t length,
                      const float* real_taps_dup,
                      const SparseCirTemplate* sparse,
                      bool volk_complex = true,
                      const ShapedSparseTemplate* shaped = nullptr)
        : dense_(dense),
          length_(length),
          real_(real_taps_dup),
          sparse_(sparse && sparse->code_energy > 0.0f ? sparse : nullptr),
          volk_complex_(volk_complex),
          shaped_(shaped && shaped->valid ? shaped : nullptr)
    {
    }

    std::complex<float> dot(const std::complex<float>* rx) const
    {
        if (shaped_)
            return shaped_ternary_dot_single(rx, *shaped_);
        if (sparse_)
            return sparse_ternary_dot_single(rx, *sparse_);
        if (real_)
            return real_template_dot_single(rx, real_, length_);
        if (volk_complex_)
            return conjugate_dot(rx, dense_, length_);
        return openuwb::core::conjugate_dot_scalar(rx, dense_, length_);
    }

    // Visit consecutive lags in increasing order. The caller guarantees
    // length + lag_count - 1 valid samples. The visitor receives a relative
    // lag, its complex correlation and whether it came from a batched path.
    template <typename Visit>
    void for_each_lag(const std::complex<float>* rx, size_t lag_count, Visit visit) const
    {
        if (shaped_) {
            shaped_visit_lags(rx, lag_count, *shaped_, visit);
            return;
        }
        if (sparse_) {
            size_t lag = 0;
#if UWB_SIMD_HAVE_AVX512
            if (use_avx512(openuwb::core::simd::KernelGroup::Sync))
                lag = visit_batches<32>(
                    rx, lag_count, lag, visit,
                    [&](const auto* in, auto* out) {
                        openuwb::core::simd::avx512::sparse_ternary_dot_32lag(
                            in, sparse_->pos_off, sparse_->pos_count,
                            sparse_->neg_off, sparse_->neg_count, out);
                    });
#endif
#if UWB_RXOPT_HAVE_AVX2
            lag = visit_batches<16>(
                rx, lag_count, lag, visit,
                [&](const auto* in, auto* out) {
                    sparse_ternary_dot_16lag(in, *sparse_, out);
                });
            lag = visit_batches<8>(
                rx, lag_count, lag, visit,
                [&](const auto* in, auto* out) {
                    sparse_ternary_dot_8lag(in, *sparse_, out);
                });
            lag = visit_batches<4>(
                rx, lag_count, lag, visit,
                [&](const auto* in, auto* out) {
                    sparse_ternary_dot_4lag(in, *sparse_, out);
                });
#endif
            for (; lag < lag_count; ++lag)
                visit(lag, sparse_ternary_dot_single(rx + lag, *sparse_), false);
        } else if (real_) {
            size_t lag = visit_batches<8>(
                rx, lag_count, 0, visit,
                [&](const auto* in, auto* out) {
                    real_template_dot_8lag(in, real_, length_, out);
                });
            lag = visit_batches<4>(
                rx, lag_count, lag, visit,
                [&](const auto* in, auto* out) {
                    real_template_dot_4lag(in, real_, length_, out);
                });
            for (; lag < lag_count; ++lag)
                visit(lag, real_template_dot_single(rx + lag, real_, length_),
                      false);
        } else {
            for (size_t lag = 0; lag < lag_count; ++lag)
                visit(lag, dot(rx + lag), false);
        }
    }

    size_t length() const { return length_; }

private:
    template <size_t BatchWidth, typename Visit, typename Batch>
    static size_t visit_batches(const std::complex<float>* rx, size_t lag_count,
                                size_t lag, Visit& visit, Batch batch)
    {
        if (lag > lag_count)
            return lag;
        for (; lag + BatchWidth <= lag_count; lag += BatchWidth) {
            std::complex<float> acc[BatchWidth];
            batch(rx + lag, acc);
            for (size_t m = 0; m < BatchWidth; ++m)
                visit(lag + m, acc[m], true);
        }
        return lag;
    }

    const std::complex<float>* dense_;
    size_t length_;
    const float* real_;
    const SparseCirTemplate* sparse_;
    bool volk_complex_;
    const ShapedSparseTemplate* shaped_;
};

struct MatchedFilterPeak {
    size_t lag = 0;
    float metric = -1.0f;
    std::complex<float> corr{};
};

// Initial power comes from the caller so each search keeps its reduction.
// Optional metrics use relative lag indices, like the returned peak.
//
// A division-free cross-multiply variant was measured on this host and was
// ~18% SLOWER than the simple per-lag division: divss has ~4-cycle throughput
// and the extra near-tie branches cost more than they save.  Keep the original
// float division; it is also the bit-exact reference for timing_metric.
inline MatchedFilterPeak scan_normalized_sync(const SyncMatchedFilter& filter,
                                              const std::complex<float>* rx,
                                              size_t lag_count,
                                              float power,
                                              float template_energy,
                                              float* metrics = nullptr)
{
    MatchedFilterPeak peak;
    filter.for_each_lag(rx, lag_count, [&](size_t lag, std::complex<float> corr, bool) {
        const float metric =
            openuwb::core::normalized_correlation_power(corr, power, template_energy);
        if (metrics)
            metrics[lag] = metric;
        if (metric > peak.metric)
            peak = { lag, metric, corr };
        if (lag + 1 < lag_count)
            power += std::norm(rx[lag + filter.length()]) - std::norm(rx[lag]);
    });
    return peak;
}

inline MatchedFilterPeak scan_raw_sync(const SyncMatchedFilter& filter,
                                       const std::complex<float>* rx,
                                       size_t lag_count,
                                       float* powers)
{
    MatchedFilterPeak peak;
    filter.for_each_lag(rx, lag_count, [&](size_t lag, std::complex<float> corr, bool) {
        const float power = std::norm(corr);
        powers[lag] = power;
        if (power > peak.metric)
            peak = { lag, power, corr };
    });
    return peak;
}

// Compare peak/background power without division. An empty or all-zero window
// cannot produce a valid peak.
inline bool raw_par_passes(double peak, double background_sum,
                           size_t background_count, double threshold)
{
    return background_count != 0 && peak > 0.0f && std::isfinite(peak) &&
           peak * static_cast<double>(background_count) >=
               threshold * background_sum;
}

inline bool raw_lag_par_passes(const float* powers,
                               size_t count,
                               const MatchedFilterPeak& peak,
                               size_t guard,
                               size_t min_background,
                               double threshold,
                               bool three_samples = false)
{
    const size_t radius = three_samples ? 1 : 0;
    if (count <= 2 * radius || peak.lag < radius || peak.lag >= count - radius)
        return false;
    const auto cluster_power = [&](size_t lag) {
        double power = powers[lag];
        if (three_samples)
            power += static_cast<double>(powers[lag - 1]) + powers[lag + 1];
        return power;
    };
    double sum = 0.0;
    size_t samples = 0;
    for (size_t lag = radius; lag < count - radius; ++lag) {
        // No part of a background window may overlap the peak guard.
        if (lag + guard + radius < peak.lag || lag > peak.lag + guard + radius) {
            sum += cluster_power(lag);
            ++samples;
        }
    }
    return samples >= min_background &&
           raw_par_passes(three_samples ? cluster_power(peak.lag) : peak.metric,
                          sum, samples, threshold);
}

inline void correlate_cir_sparse_raw(const std::complex<float>* corr,
                                     const SparseCirTemplate& tmpl,
                                     size_t tap_count,
                                     std::complex<float>* values)
{
    if (tap_count == 0)
        return;
    const SyncMatchedFilter filter(nullptr, 0, nullptr, &tmpl);
    filter.for_each_lag(corr, tap_count, [&](size_t lag, std::complex<float> acc, bool) {
        values[lag] = acc;
    });
}

inline void correlate_cir_sparse(const std::complex<float>* corr,
                                 const SparseCirTemplate& tmpl,
                                 size_t tap_count,
                                 std::complex<float>* values)
{
    if (tap_count == 0 || tmpl.code_energy <= 0.0f)
        return;
    correlate_cir_sparse_raw(corr, tmpl, tap_count, values);
    for (size_t i = 0; i < tap_count; ++i)
        values[i] /= tmpl.code_energy;
}

inline void correlate_cir_sparse(const std::complex<float>* corr,
                                 const SparseCirTemplate& tmpl,
                                 size_t tap_count,
                                 std::vector<std::complex<float>>& values)
{
    values.resize(tap_count);
    if (tmpl.code_energy <= 0.0f || tap_count == 0)
        return;
    correlate_cir_sparse(corr, tmpl, tap_count, values.data());
}

} // namespace openuwb::demod::rxopt
