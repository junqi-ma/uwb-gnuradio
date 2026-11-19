// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <gnuradio/uwb/uwb_rx_sync_opt.h>

namespace gr::uwb::demod::rxopt {

struct SparseCirEntry {
    size_t pos = 0;      // sample index in [0, 1016), always even: 2 * j
    int8_t sign = 0;     // +1 or -1
};

struct SparseCirTemplate {
    uint64_t key = 0;
    std::vector<SparseCirEntry> entries;
    std::vector<size_t> pos_offsets;
    std::vector<size_t> neg_offsets;
    float code_energy = 0.0f;

    void reset()
    {
        key = 0;
        entries.clear();
        pos_offsets.clear();
        neg_offsets.clear();
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
            if (spread[j] > 0)
                tmpl.pos_offsets.push_back(2 * pos);
            else
                tmpl.neg_offsets.push_back(2 * pos);
        }
    }
    tmpl.code_energy = static_cast<float>(tmpl.entries.size());
}

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
    for (const size_t off : tmpl.pos_offsets) {
        p0 = _mm256_add_ps(p0, _mm256_loadu_ps(rx_f + off));
        p1 = _mm256_add_ps(p1, _mm256_loadu_ps(rx_f + off + 8));
    }
    __m256 n0 = _mm256_setzero_ps();
    __m256 n1 = _mm256_setzero_ps();
    for (const size_t off : tmpl.neg_offsets) {
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

inline void sparse_ternary_dot_16lag(
    const std::complex<float>* rx,
    const SparseCirTemplate& tmpl,
    std::complex<float> out[16])
{
#if UWB_SIMD_HAVE_AVX512 && defined(__AVX512F__)
    const float* rx_f = reinterpret_cast<const float*>(rx);
    __m512 p0 = _mm512_setzero_ps();
    __m512 p1 = _mm512_setzero_ps();
    for (const size_t off : tmpl.pos_offsets) {
        p0 = _mm512_add_ps(p0, _mm512_loadu_ps(rx_f + off));
        p1 = _mm512_add_ps(p1, _mm512_loadu_ps(rx_f + off + 16));
    }
    __m512 n0 = _mm512_setzero_ps();
    __m512 n1 = _mm512_setzero_ps();
    for (const size_t off : tmpl.neg_offsets) {
        n0 = _mm512_add_ps(n0, _mm512_loadu_ps(rx_f + off));
        n1 = _mm512_add_ps(n1, _mm512_loadu_ps(rx_f + off + 16));
    }
    _mm512_storeu_ps(reinterpret_cast<float*>(out), _mm512_sub_ps(p0, n0));
    _mm512_storeu_ps(reinterpret_cast<float*>(out + 8), _mm512_sub_ps(p1, n1));
#else
    sparse_ternary_dot_8lag(rx, tmpl, out);
    sparse_ternary_dot_8lag(rx + 8, tmpl, out + 8);
#endif
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
}

inline void pack_even_complex(const std::complex<float>* src,
                              size_t even_count,
                              std::complex<float>* dst)
{
    size_t i = 0;
#if UWB_SIMD_HAVE_AVX512 && defined(__AVX512F__)
    const __m512i idx = _mm512_set_epi64(14, 12, 10, 8, 6, 4, 2, 0);
    for (; i + 8 <= even_count; i += 8) {
        const __m512i v0 =
            _mm512_loadu_si512(reinterpret_cast<const void*>(src + 2 * i));
        const __m512i v1 =
            _mm512_loadu_si512(reinterpret_cast<const void*>(src + 2 * i + 8));
        const __m512i ev = _mm512_permutex2var_epi64(v0, idx, v1);
        _mm512_storeu_si512(reinterpret_cast<void*>(dst + i), ev);
    }
#endif
    for (; i < even_count; ++i)
        dst[i] = src[2 * i];
}

// A non-owning view over a single SYNC symbol template. Sparse batches and
// their scalar tails deliberately retain their respective accumulation orders.
class SyncMatchedFilter {
public:
    SyncMatchedFilter(const std::complex<float>* dense,
                      size_t length,
                      const float* real_taps_dup,
                      const SparseCirTemplate* sparse,
                      bool volk_complex = true)
        : dense_(dense),
          length_(length),
          real_(real_taps_dup),
          sparse_(sparse && sparse->code_energy > 0.0f ? sparse : nullptr),
          volk_complex_(volk_complex)
    {
    }

    std::complex<float> dot(const std::complex<float>* rx) const
    {
        if (sparse_)
            return sparse_ternary_dot_single(rx, *sparse_);
        if (real_)
            return real_template_dot_single(rx, real_, length_);
        if (volk_complex_)
            return conjugate_dot(rx, dense_, length_);
        return gr::uwb::core::conjugate_dot_scalar(rx, dense_, length_);
    }

    // Visit consecutive lags in increasing order. The caller guarantees
    // length + lag_count - 1 valid samples. The visitor receives a relative
    // lag, its complex correlation and whether it came from an 8-lag batch.
    template <typename Visit>
    void for_each_lag(const std::complex<float>* rx, size_t lag_count, Visit visit) const
    {
        if (sparse_) {
#if UWB_SPARSE_MF_AVX512_16LAG && UWB_SIMD_HAVE_AVX512 && defined(__AVX512F__)
            visit_batched<16>(
                rx, lag_count, visit,
                [&](const auto* in) { return sparse_ternary_dot_single(in, *sparse_); },
                [&](const auto* in, auto* out) { sparse_ternary_dot_16lag(in, *sparse_, out); });
#else
            visit_batched<8>(
                rx, lag_count, visit,
                [&](const auto* in) { return sparse_ternary_dot_single(in, *sparse_); },
                [&](const auto* in, auto* out) { sparse_ternary_dot_8lag(in, *sparse_, out); });
#endif
        } else if (real_) {
            visit_batched<8>(
                rx, lag_count, visit,
                [&](const auto* in) { return real_template_dot_single(in, real_, length_); },
                [&](const auto* in, auto* out) { real_template_dot_8lag(in, real_, length_, out); });
        } else {
            for (size_t lag = 0; lag < lag_count; ++lag)
                visit(lag, dot(rx + lag), false);
        }
    }

    size_t length() const { return length_; }

private:
    template <size_t BatchWidth, typename Visit, typename Single, typename Batch>
    static void visit_batched(const std::complex<float>* rx, size_t lag_count,
                              Visit& visit, Single single, Batch batch)
    {
        size_t lag = 0;
        for (; lag_count - lag >= BatchWidth; lag += BatchWidth) {
            std::complex<float> acc[BatchWidth];
            batch(rx + lag, acc);
            for (size_t m = 0; m < BatchWidth; ++m)
                visit(lag + m, acc[m], true);
        }
        for (; lag < lag_count; ++lag)
            visit(lag, single(rx + lag), false);
    }

    const std::complex<float>* dense_;
    size_t length_;
    const float* real_;
    const SparseCirTemplate* sparse_;
    bool volk_complex_;
};

struct MatchedFilterPeak {
    size_t lag = 0;
    float metric = -1.0f;
    std::complex<float> corr{};
};

// Initial power comes from the caller so each search keeps its reduction.
// Optional metrics use relative lag indices, like the returned peak.
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
            gr::uwb::core::normalized_correlation_power(corr, power, template_energy);
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

inline void correlate_cir_sparse(const std::complex<float>* corr,
                                 const SparseCirTemplate& tmpl,
                                 size_t tap_count,
                                 std::vector<std::complex<float>>& values)
{
    values.resize(tap_count);
    if (tmpl.code_energy <= 0.0f || tap_count == 0)
        return;
    const SyncMatchedFilter filter(nullptr, 0, nullptr, &tmpl);
    filter.for_each_lag(corr, tap_count, [&](size_t lag, std::complex<float> acc, bool) {
        values[lag] = acc / tmpl.code_energy;
    });
}

} // namespace gr::uwb::demod::rxopt
