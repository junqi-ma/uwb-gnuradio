// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <algorithm>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <vector>
// The acquisition block kernel is the only SIMD code in this app.  The build
// selects the ISA through UWB_SIMD (see gr-uwb/cmake/Modules/UwbSimd.cmake);
// never add -march=native here.  Without AVX2+FMA the same kernel compiles to
// the portable scalar path, so -DUWB_SIMD=SCALAR stays buildable.
#if defined(__AVX2__) && defined(__FMA__)
#include <immintrin.h>
#define UWB_LIVE_ACQ_HAVE_AVX2 1
#else
#define UWB_LIVE_ACQ_HAVE_AVX2 0
#endif

namespace gr::uwb::live {
// Streaming SC16 acquisition. Caller owns the ring and advances probe only
// through available samples. Ring size must be a power of two; scan receives
// step valid pairs. reset() invalidates all accumulated history after a gap.
// ---------------------------------------------------------------------------
// Streaming delayed-autocorrelation acquisition.
//
//   metric[at] = |sum_{k<span} x[at+k] x*[at+k+lag]|^2
//                / (sum_k |x[at+k]|^2 * sum_k |x[at+k+lag]|^2)
//
// The window is evaluated over EVERY sample of the span: `step` is only the
// granularity of the ring table, not a decimation.  Each `step`-sample block is
// reduced by one AVX2 integer kernel into a complex correlation plus the two
// energies, so the table holds span/step = 3000 block sums (96 KB, L2 resident)
// and the sliding window is maintained by `sum += v - old` in double, which
// keeps the large-window subtraction error bounded.
//
// Seed selection tracks the TOP of the correlation triangle rather than its
// leading edge.  With a 192k window the metric top is flat over exactly one
// SYNC symbol (lag = 750 samples), so the leading-edge crossing plus a fixed
// span/2 shift drifts by more than 100k samples between +10 dB and -12 dB.
// The argmax `peak_at` sits in [start-lag, start]; committing at
// `peak_at + lag/2` centres the seed on the symbol.
struct Acquisition {
    static constexpr int64_t lag = 750;
    static constexpr int64_t gap_max = 30000, holdoff_len = 589824;
    // Peak-fall commit.  `fall_frac` is the fraction of the tracked peak that
    // marks "past the top"; `fall_after` is the flat-top escape hatch.  16384
    // is NOT usable: at low SNR the rising ramp has small plateaus that look
    // like a fall and the resulting holdoff masks the true top.
    static constexpr double fall_frac = 0.6;
    static constexpr int64_t fall_after = 65536;
    int64_t step = 64, span = 192000, min_persist = 12000;
    double low_thr = 0.00015, peak_thr = 0.00030;
    struct Pair { double re=0, im=0, e1=0, e2=0; };
    std::vector<Pair> pairs;
    Pair sum{};
    size_t index=0, count=0;
    int64_t probe=0, first=-1, last=-1, holdoff=0, peak_at=-1;
    double peak=0;

    Acquisition() { configure(span, step, min_persist, low_thr, peak_thr); }
    void configure(int64_t new_span, int64_t new_step, int64_t new_persist,
                   double new_low, double new_peak) {
        span = new_span;
        step = new_step;
        min_persist = new_persist;
        low_thr = new_low;
        peak_thr = new_peak;
        const size_t n_pairs = size_t(span / step);
        pairs.assign(n_pairs, Pair{});
        reset(0);
    }
    void reset(int64_t start) {
        std::fill(pairs.begin(), pairs.end(), Pair{});
        sum = Pair{};
        index = 0; count = 0;
        probe = start; first = -1; last = -1; holdoff = 0;
        peak = 0.0; peak_at = -1;
    }

    // Reduce `n` consecutive complex<int16_t> pairs from two contiguous runs:
    //   re += ar*br + ai*bi,  im += ai*br - ar*bi,
    //   e1 += ar*ar + ai*ai,  e2 += br*br + bi*bi.
    // Caller guarantees both runs are non-wrapping.
    //
    // Why integer and not the float form in the plan: measured on the 0.5 s
    // AWGN replay this kernel is what decides whether the frontend fits in the
    // 125 ms budget.  The float version (vpmovsxwd + vcvtdq2ps + 4 FMA per 4
    // samples) needs 92 ms because the int16->float widening convert runs on
    // the single shuffle port (the loads+convert alone cost 50 ms).  vpmaddwd
    // already sums the I/Q pair of every complex sample, so re/e1/e2/im each
    // cost one multiply-add per 4 samples and the inner loop has no convert at
    // all: 66 ms, a 28% cut that the acceptance target needs.
    //
    // Overflow: vpmaddwd sums two int32 products, so a full-scale int16 input
    // would already reach 2^31.  The inputs are therefore shifted right by 3
    // (13 significant bits, |x| <= 4096) and the int32 accumulators are
    // flushed into float every 64 samples, which bounds them at 2.7e8 for any
    // --acq-step.  The metric is a ratio of |sum|^2 over e1*e2, so the uniform
    // scale cancels exactly; the only cost is an 8 LSB quantisation step, i.e.
    // -35.7 dB relative to a 542 LSB packet, which at -12 dB in-band SNR is
    // 55 dB below the noise it is added to.  The >>3 also makes the I/Q sign
    // flip exact (|x| <= 4096), so the int16 == -32768 negation overflow that
    // the float form was chosen to avoid cannot occur.
    static Pair block_sum(const std::complex<int16_t>* a,
                          const std::complex<int16_t>* b, size_t n) {
        double re = 0.0, im = 0.0, e1 = 0.0, e2 = 0.0;
        size_t i = 0;
#if UWB_LIVE_ACQ_HAVE_AVX2
        const __m256i swap = _mm256_setr_epi8(
            2,3,0,1, 6,7,4,5, 10,11,8,9, 14,15,12,13,
            2,3,0,1, 6,7,4,5, 10,11,8,9, 14,15,12,13);
        const __m256i sgn = _mm256_setr_epi16(1,-1,1,-1,1,-1,1,-1,
                                              1,-1,1,-1,1,-1,1,-1);
        // Round-to-nearest before the >>3: a plain arithmetic shift truncates
        // towards -inf and leaves a -0.5 LSB DC in the shifted domain, which
        // adds coherently over 192k samples and lifts the measured noise floor
        // by 10.5%.  The saturating add keeps +32767 at +32767 (no wrap into a
        // sign flip) and still rounds -32768 exactly.
        const __m256i four = _mm256_set1_epi16(4);
        __m256 fre = _mm256_setzero_ps(), fim = _mm256_setzero_ps();
        __m256 fe1 = _mm256_setzero_ps(), fe2 = _mm256_setzero_ps();
        // One 8-sample group: 8 complex samples = 16 int16 = one 256-bit load.
        auto group = [&](size_t j, __m256i& gre, __m256i& gim,
                         __m256i& ge1, __m256i& ge2) {
            const __m256i ia = _mm256_srai_epi16(_mm256_adds_epi16(
                _mm256_loadu_si256(reinterpret_cast<const __m256i*>(a + j)), four), 3);
            const __m256i ib = _mm256_srai_epi16(_mm256_adds_epi16(
                _mm256_loadu_si256(reinterpret_cast<const __m256i*>(b + j)), four), 3);
            gre = _mm256_add_epi32(gre, _mm256_madd_epi16(ia, ib));
            ge1 = _mm256_add_epi32(ge1, _mm256_madd_epi16(ia, ia));
            ge2 = _mm256_add_epi32(ge2, _mm256_madd_epi16(ib, ib));
            // swap I/Q of a ([ai,ar]) and negate b's Q ([br,-bi]):
            // ai*br - ar*bi is exactly the conjugate product's imaginary part.
            gim = _mm256_add_epi32(gim, _mm256_madd_epi16(
                _mm256_shuffle_epi8(ia, swap), _mm256_sign_epi16(ib, sgn)));
        };
        for (; i + 64 <= n; i += 64) {
            __m256i gre = _mm256_setzero_si256(), gim = _mm256_setzero_si256();
            __m256i ge1 = _mm256_setzero_si256(), ge2 = _mm256_setzero_si256();
            for (size_t k = 0; k < 64; k += 8) group(i + k, gre, gim, ge1, ge2);
            fre = _mm256_add_ps(fre, _mm256_cvtepi32_ps(gre));
            fim = _mm256_add_ps(fim, _mm256_cvtepi32_ps(gim));
            fe1 = _mm256_add_ps(fe1, _mm256_cvtepi32_ps(ge1));
            fe2 = _mm256_add_ps(fe2, _mm256_cvtepi32_ps(ge2));
        }
        if (i + 8 <= n) {
            // Tail: at most 7 groups, safe in one int32 accumulator.
            __m256i gre = _mm256_setzero_si256(), gim = _mm256_setzero_si256();
            __m256i ge1 = _mm256_setzero_si256(), ge2 = _mm256_setzero_si256();
            for (; i + 8 <= n; i += 8) group(i, gre, gim, ge1, ge2);
            fre = _mm256_add_ps(fre, _mm256_cvtepi32_ps(gre));
            fim = _mm256_add_ps(fim, _mm256_cvtepi32_ps(gim));
            fe1 = _mm256_add_ps(fe1, _mm256_cvtepi32_ps(ge1));
            fe2 = _mm256_add_ps(fe2, _mm256_cvtepi32_ps(ge2));
        }
        // One 4-way reduction for all four sums (three vhaddps).
        const __m256 t0 = _mm256_hadd_ps(fre, fe1);
        const __m256 t1 = _mm256_hadd_ps(fim, fe2);
        const __m256 t2 = _mm256_hadd_ps(t0, t1);
        const __m128 r = _mm_add_ps(_mm256_castps256_ps128(t2),
                                    _mm256_extractf128_ps(t2, 1));
        re = double(_mm_cvtss_f32(r));
        e1 = double(_mm_cvtss_f32(_mm_shuffle_ps(r, r, 0x55)));
        im = double(_mm_cvtss_f32(_mm_shuffle_ps(r, r, 0xAA)));
        e2 = double(_mm_cvtss_f32(_mm_shuffle_ps(r, r, 0xFF)));
#endif
        for (; i < n; ++i) {
            // Same >>3 the SIMD path applies, so a block that mixes both (any
            // --acq-step not a multiple of 8) stays on one scale.
            const double ar = double((int(a[i].real()) + 4) >> 3);
            const double ai = double((int(a[i].imag()) + 4) >> 3);
            const double br = double((int(b[i].real()) + 4) >> 3);
            const double bi = double((int(b[i].imag()) + 4) >> 3);
            re += ar*br + ai*bi;
            im += ai*br - ar*bi;
            e1 += ar*ar + ai*ai;
            e2 += br*br + bi*bi;
        }
        return Pair{re, im, e1, e2};
    }

    int64_t scan(const std::vector<std::complex<int16_t>>& ring, double* out_metric = nullptr) {
        const size_t mask = ring.size() - 1;
        const size_t p = size_t(probe) & mask;
        const size_t q = size_t(probe + lag) & mask;
        const size_t n = size_t(step);
        Pair v{};
        if (p + n <= ring.size() && q + n <= ring.size()) {
            v = block_sum(ring.data() + p, ring.data() + q, n);
        } else {
            // Ring wrap: two blocks per 2M-sample ring, so the masked scalar
            // loop is cheaper than splitting into contiguous runs.  It applies
            // the same >>3 as the SIMD kernel so both paths share one scale.
            for (size_t i = 0; i < n; ++i) {
                const auto a = ring[(p + i) & mask];
                const auto b = ring[(q + i) & mask];
                const double ar = double((int(a.real()) + 4) >> 3);
                const double ai = double((int(a.imag()) + 4) >> 3);
                const double br = double((int(b.real()) + 4) >> 3);
                const double bi = double((int(b.imag()) + 4) >> 3);
                v.re += ar*br + ai*bi;
                v.im += ai*br - ar*bi;
                v.e1 += ar*ar + ai*ai;
                v.e2 += br*br + bi*bi;
            }
        }
        const size_t n_pairs = pairs.size();
        auto& old = pairs[index];
        sum.re += v.re-old.re; sum.im += v.im-old.im;
        sum.e1 += v.e1-old.e1; sum.e2 += v.e2-old.e2;
        old=v; index=(index+1==n_pairs)?0:(index+1); count=std::min(n_pairs,count+1);
        const int64_t at = probe-(span-step); probe += step;
        if (count < n_pairs || at < holdoff) return -1;
        // Keep the ratio out of the noise path: a 5.76M-block 0.5 s replay
        // would otherwise pay a serial double divide per block (~19 ms).  The
        // gate, the peak comparison and the fall test are all cross-multiplied
        // against `den`, so the division only runs while a cluster is open.
        const double num = sum.re*sum.re + sum.im*sum.im;
        const double den = std::max(1.0, sum.e1*sum.e2);
        // Commit on the way down, before the cluster state is touched: this
        // also covers a noiseless burst that drops straight to zero (the QA
        // cw_burst fixture goes 1.0 -> 0.0 in one step) and would otherwise
        // never satisfy the `metric >= low_thr` branch.
        if (first >= 0 && at-first >= min_persist && peak > peak_thr &&
            (num <= fall_frac*peak*den || at-peak_at >= fall_after)) {
            // span > 6000: the flat top is one SYNC symbol wide, so centre the
            // seed in it.  The legacy small-window offset is kept for span<=6000.
            const int64_t seed = std::max<int64_t>(
                0, peak_at + ((span > 6000) ? lag/2 : 0));
            const double reported = peak;
            holdoff = seed + holdoff_len;
            first = -1; peak = 0; peak_at = -1;
            if (out_metric) *out_metric = reported;
            return seed;
        }
        if (num >= low_thr*den) {
            if (first < 0 || at-last > gap_max) { first=at; peak=0; peak_at=at; }
            last=at;
            const double metric = num/den;
            if (metric > peak) { peak=metric; peak_at=at; }
        } else if (first >= 0 && at-last > gap_max) { first=-1; peak=0; peak_at=-1; }
        return -1;
    }
};
} // namespace gr::uwb::live
