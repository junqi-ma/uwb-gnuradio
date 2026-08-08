/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Standalone UWB detection core.
 *
 * This header is intentionally free of any GNU Radio framework dependency —
 * it holds the pure signal-processing math for the first two detection
 * stages:
 *
 *   1. energy detection   -- sliding-window average of |x|^2
 *   2. preamble detection -- normalized matched-filter correlation score
 *
 * The GNU Radio blocks in this module are thin adapters that call these
 * functions.  The hot correlation loops use VOLK (the standalone SIMD
 * abstraction library that GNU Radio builds on) for vectorization; VOLK is a
 * separate low-level library, not the GNU Radio runtime.
 *
 * All constants were taken from the MATLAB reference implementation in
 * UWB_demodulation/ (buildUwbReference.m, constants.m) and verified against
 * testdata/uwb_code9_preamble64_payload128_standard_sfd.cfile.
 */

#pragma once

#include <volk/volk.h>

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <utility>
#include <vector>

namespace gr {
namespace uwb {
namespace core {

// ---------------------------------------------------------------------------
// IEEE 802.15.4a HRP PHY constants (code index 9, mean PRF 62.4 MHz, fs
// 998.4 MHz, SamplesPerPulse = 2).  The 1016-sample SYNC symbol period was
// confirmed by autocorrelation of the reference .cfile.
// ---------------------------------------------------------------------------
inline constexpr double kUwbSampleRateHz = 998400000.0;
inline constexpr size_t kUwbSamplesPerSymbol = 1016;
inline constexpr size_t kUwbSyncSymbols = 64;
inline constexpr size_t kUwbSfdSymbols = 8;

// Small epsilon used to keep the normalized metric stable when both signal and
// template energy are ~0 (silence / leading zeros).
inline constexpr float kUwbEpsilon = 1e-12f;

/**
 * L2-normalize a template waveform in place.
 *
 * Matches buildUwbReference.m: preambleWaveform / (norm(preambleWaveform)+eps).
 */
inline void uwb_l2_normalize(std::vector<std::complex<float>>& t)
{
    if (t.empty())
        return;
    float e = 0.0f;
    for (const auto& v : t)
        e += std::norm(v);
    float n = std::sqrt(e);
    if (n > 0.0f) {
        for (auto& v : t)
            v /= n;
    }
}

/**
 * Sliding-window average energy.
 *
 *   out[j] = (1/window) * sum_{m=0}^{window-1} |in[j+m]|^2,  j in [0, n)
 *
 * `in` must contain n + window - 1 valid samples (the last window-1 of which
 * are the history/pre-trigger samples provided by the caller); `out` receives
 * n samples.  Window sums are maintained by a running accumulator (O(1) per
 * sample once the first window is formed).
 */
inline void uwb_window_energy(const std::complex<float>* in,
                              size_t n,
                              size_t window,
                              float* out)
{
    if (n == 0 || window == 0)
        return;
    float acc = 0.0f;
    for (size_t m = 0; m < window; ++m)
        acc += std::norm(in[m]);
    for (size_t j = 0; j < n; ++j) {
        out[j] = acc / static_cast<float>(window);
        if (j + 1 < n) { // slide the window forward (guarded: in[n+window-1] is last valid)
            acc -= std::norm(in[j]);
            acc += std::norm(in[j + window]);
        }
    }
}

/**
 * Sliding-window power (un-normalized) over a length-L window:
 *
 *   pow[j] = sum_{m=0}^{L-1} |in[j+m]|^2,  j in [0, n)
 *
 * This is the denominator energy used by the normalized correlation metric.
 * Same history contract as uwb_window_energy.
 */
inline void uwb_window_power(const std::complex<float>* in,
                             size_t n,
                             size_t L,
                             float* pow)
{
    if (n == 0 || L == 0)
        return;
    float acc = 0.0f;
    for (size_t m = 0; m < L; ++m)
        acc += std::norm(in[m]);
    for (size_t j = 0; j < n; ++j) {
        pow[j] = acc;
        if (j + 1 < n) {
            acc -= std::norm(in[j]);
            acc += std::norm(in[j + L]);
        }
    }
}

/**
 * Normalized correlation score (Reference Detector metric, squared form):
 *
 *   metric[j] = |corr[j]|^2 / (winpow[j] * template_energy + eps)
 *
 * where corr is the raw matched-filter output at alignment j and winpow is the
 * windowed signal power over the same window.  This is the MATLAB scan metric
 * `energy = score.^2` with score = |<r,t>| / (||r|| ||t||), clamped to [0,1].
 */
inline void uwb_normalized_score(const std::complex<float>* corr,
                                 const float* winpow,
                                 size_t n,
                                 float template_energy,
                                 float* metric)
{
    for (size_t j = 0; j < n; ++j) {
        float d = winpow[j] * template_energy + kUwbEpsilon;
        float m = std::norm(corr[j]) / d;
        metric[j] = (m < 0.0f) ? 0.0f : ((m > 1.0f) ? 1.0f : m);
    }
}

/**
 * Energy of a template waveform (sum of |x|^2). Used as the normalization
 * factor in uwb_normalized_score when the template is not unit-norm.
 */
inline float uwb_template_energy(const std::vector<std::complex<float>>& t)
{
    float e = 0.0f;
    for (const auto& v : t)
        e += std::norm(v);
    return e;
}

/**
 * Coarse / energy-gate stage of the fast detector.
 *
 * From a per-sample energy series, returns contiguous candidate ranges where
 * the energy exceeds `threshold`, each expanded by `margin` samples on both
 * sides (and clamped to [0, n)).  Adjacent/overlapping ranges are merged.
 *
 * The expensive full-rate preamble correlation is only evaluated inside the
 * returned ranges; everywhere else the metric is treated as 0.  This is what
 * turns the O(N*L) reference detector into a gate + candidate search: for a
 * typical capture the signal occupies only a few percent of the stream, so
 * ~95% of the 1016-tap correlations are skipped.
 */
inline void uwb_candidate_ranges(const float* energy,
                                 size_t n,
                                 float threshold,
                                 size_t margin,
                                 std::vector<std::pair<size_t, size_t>>& ranges)
{
    ranges.clear();
    size_t i = 0;
    while (i < n) {
        if (energy[i] > threshold) {
            size_t start = i;
            while (i < n && energy[i] > threshold)
                ++i;
            size_t end = i;
            const size_t a = (start > margin) ? start - margin : 0;
            const size_t b = std::min(end + margin, n);
            if (ranges.empty() || ranges.back().second < a)
                ranges.emplace_back(a, b);
            else if (b > ranges.back().second)
                ranges.back().second = b; // merge adjacent/overlapping
        } else {
            ++i;
        }
    }
}

/**
 * Decimated (strided) energy gate — the "100× downsampling" coarse stage.
 *
 * Computes |x|^2 only at every D-th sample, smooths it over `win` decimated
 * points, and returns candidate [start,end) ranges in ORIGINAL sample
 * coordinates (each expanded by `margin`, adjacent runs merged).  The gate
 * decision granularity is D samples; the response lag is win*D samples, which
 * is small compared to the 1016-sample SYNC symbol.
 *
 * Cost: O(n / D), i.e. ~D× cheaper than the full-rate energy pass.
 */
inline void uwb_energy_gate_strided(const std::complex<float>* in,
                                    size_t n,
                                    size_t D,
                                    size_t win,
                                    float threshold,
                                    size_t margin,
                                    std::vector<std::pair<size_t, size_t>>& ranges)
{
    ranges.clear();
    if (n == 0 || D == 0)
        return;
    const size_t nd = n / D;
    if (nd == 0)
        return;

    std::vector<float> ring(win, 0.0f); // tiny; one allocation per call
    size_t pos = 0;
    size_t cnt = 0;
    float sum = 0.0f;
    size_t start = 0;
    bool in_run = false;

    auto flush = [&](size_t last) {
        const size_t a = (start * D > margin) ? start * D - margin : 0;
        const size_t b = std::min(n, (last + 1) * D + margin);
        if (b > a)
            ranges.emplace_back(a, b);
    };

    for (size_t k = 0; k < nd; ++k) {
        const float p = std::norm(in[k * D]);
        sum -= ring[pos];
        ring[pos] = p;
        sum += p;
        pos = (pos + 1) % win;
        if (cnt < win)
            ++cnt;
        const bool on = sum / static_cast<float>(cnt) > threshold;
        if (on && !in_run) {
            in_run = true;
            start = k;
        } else if (!on && in_run) {
            in_run = false;
            flush(k > 0 ? k - 1 : 0);
        }
    }
    if (in_run)
        flush(nd - 1);

    std::vector<std::pair<size_t, size_t>> merged;
    for (auto& r : ranges) {
        if (!merged.empty() && r.first <= merged.back().second)
            merged.back().second = std::max(merged.back().second, r.second);
        else
            merged.push_back(r);
    }
    ranges.swap(merged);
}

/**
 * Decimated coarse preamble scan (MATLAB detectRepeatedPreamble legacy
 * coarsePreamblePeak): correlate the D-times decimated signal against the
 * D-times decimated template over [scan_start, range_end), normalize by
 * trailing window energy, accumulate over R repetitions spaced one (decimated)
 * symbol apart, and return the local-max peaks as ORIGINAL sample positions
 * (symbol-start convention).  `scan_start` may be up to ~L samples before the
 * candidate range so symbols straddling a chunk boundary are still seen.
 *
 * Cost: O((range_len/D) * (L/D)) for the correlation, plus O(range_len/D) for
 * the energy/accumulation — ~D² cheaper than full-rate correlation.
 *
 * All output/scratch vectors are caller-owned; no allocation inside.
 */
inline void uwb_coarse_peaks(const std::complex<float>* in,
                             size_t scan_start,
                             size_t range_end,
                             const std::complex<float>* tmpl_ds,
                             size_t Ld,
                             size_t D,
                             size_t R,
                             size_t sym_ds,
                             float peak_rel,   // relative peak threshold (x max)
                             float exist_frac, // absolute exist floor (x R)
                             size_t stride,    // correlation stride (R=1 only)
                             std::vector<std::complex<float>>& sig_ds,
                             std::vector<float>& pow_ds,
                             std::vector<float>& score_ds,
                             std::vector<float>& metric_ds,
                             std::vector<size_t>& peaks,
                             float* max_metric)
{
    peaks.clear();
    *max_metric = 0.0f;
    if (D == 0 || R == 0 || Ld == 0)
        return;
    const size_t scan_len = range_end - scan_start;
    const size_t nd = scan_len / D;
    if (nd < Ld)
        return;

    sig_ds.resize(nd);
    pow_ds.resize(nd);
    for (size_t j = 0; j < nd; ++j)
        sig_ds[j] = in[scan_start + j * D];

    float etd = 0.0f;
    for (size_t k = 0; k < Ld; ++k)
        etd += std::norm(tmpl_ds[k]);
    if (etd <= 0.0f)
        return;

    // trailing window power on the decimated grid
    float psum = 0.0f;
    for (size_t k = 0; k < Ld; ++k)
        psum += std::norm(sig_ds[k]);
    pow_ds[0] = psum;
    for (size_t j = 1; j + Ld <= nd; ++j) {
        psum += std::norm(sig_ds[j + Ld - 1]) - std::norm(sig_ds[j - 1]);
        pow_ds[j] = psum;
    }

    const size_t nscore = nd - Ld + 1;
    score_ds.resize(nscore);

    // The repetition accumulation needs R whole SYNC symbols; in a streaming
    // chunk there may be fewer.  The accumulation also shortens the covered
    // region by (R-1)*sym_ds, so only use it when the chunk holds at least R
    // symbols; otherwise fall back to R=1 which covers the whole chunk and
    // still yields one peak per symbol alignment.
    const size_t avail = (sym_ds > 0) ? nscore / sym_ds : 0;
    const size_t R_eff = (avail >= R && R > 0) ? R : 1;

    if (R_eff > 1) {
        // Full correlation + repetition accumulation.
        for (size_t j = 0; j < nscore; ++j) {
            std::complex<float> acc(0.0f, 0.0f);
            // VOLK vectorized conjugate dot product: sum sig_ds[j+k]*conj(tmpl_ds[k]).
            volk_32fc_x2_conjugate_dot_prod_32fc(
                &acc, sig_ds.data() + j, tmpl_ds,
                static_cast<unsigned int>(Ld));
            score_ds[j] =
                std::abs(acc) / (std::sqrt(pow_ds[j] * etd + kUwbEpsilon));
        }
    } else {
        // R=1: stride the correlation.  The SYNC peaks are ~sym_ds apart, so a
        // stride of a few decimated positions costs ~stride× less while the
        // fine stage still pinpoints the peak (the block widens its fine ROI
        // by stride*D samples).  Only strided positions are evaluated.
        std::fill(score_ds.begin(), score_ds.end(), 0.0f);
        const size_t S = (stride > 0) ? stride : 1;
        for (size_t j = 0; j < nscore; j += S) {
            std::complex<float> acc(0.0f, 0.0f);
            // VOLK vectorized conjugate dot product: sum sig_ds[j+k]*conj(tmpl_ds[k]).
            volk_32fc_x2_conjugate_dot_prod_32fc(
                &acc, sig_ds.data() + j, tmpl_ds,
                static_cast<unsigned int>(Ld));
            score_ds[j] =
                std::abs(acc) / (std::sqrt(pow_ds[j] * etd + kUwbEpsilon));
        }
    }

    const size_t nmet =
        (nscore > (R_eff - 1) * sym_ds) ? nscore - (R_eff - 1) * sym_ds : 0;
    if (nmet == 0)
        return;
    metric_ds.assign(nmet, 0.0f);
    for (size_t r = 0; r < R_eff; ++r)
        for (size_t j = 0; j < nmet; ++j)
            metric_ds[j] += score_ds[j + r * sym_ds];
    for (size_t j = 0; j < nmet; ++j)
        if (metric_ds[j] > *max_metric)
            *max_metric = metric_ds[j];

    // existence check: the accumulated peak must be a meaningful fraction of
    // the number of accumulated repetitions.
    if (*max_metric < exist_frac * static_cast<float>(R_eff))
        return;

    const float thr = std::max(peak_rel * (*max_metric), 0.0f);
    const size_t step = (R_eff > 1) ? 1 : ((stride > 0) ? stride : 1);
    for (size_t j = step; j + step < nmet; j += step) {
        if (metric_ds[j] > metric_ds[j - step] &&
            metric_ds[j] >= metric_ds[j + step] && metric_ds[j] > thr &&
            (peaks.empty() || j - peaks.back() >= sym_ds / (2 * step))) {
            peaks.push_back(j);
        }
    }
    // convert decimated peak index -> original sample position (symbol start)
    for (auto& j : peaks)
        j = scan_start + j * D;
}

} // namespace core

// ---------------------------------------------------------------------------
// Packet extraction core (SEARCH → CAPTURE → HOLD_OFF). Kept GR-independent so
// it can be unit-tested and benchmarked without linking against GNU Radio.
// ---------------------------------------------------------------------------

// RingBuffer: circular buffer holding the most recent `capacity` samples.
// Used for pre-trigger / history so a packet detected across an input chunk
// can recover the samples that arrived before the trigger.
// ---------------------------------------------------------------------------

struct RingBuffer {
    // Physical capacity is rounded up to a power of two so the write index
    // wraps with a bit-mask (hot path: one push per input sample).  The
    // *logical* size is the caller-requested capacity and is what to_vector()
    // returns — the pre-trigger window is exactly `logical_size` samples.
    size_t capacity;
    size_t mask;
    size_t logical_size;
    size_t write_pos = 0;
    std::vector<std::complex<float>> buffer;

    explicit RingBuffer(size_t cap)
        : capacity(round_up_pow2(cap)),
          mask(capacity - 1),
          logical_size(cap),
          buffer(capacity, std::complex<float>(0.0f, 0.0f))
    {
    }

    static size_t round_up_pow2(size_t v)
    {
        size_t p = 1;
        while (p < v)
            p <<= 1;
        return p;
    }

    void push(const std::complex<float>* src, size_t n)
    {
        if (n > capacity)
            n = capacity;
        for (size_t i = 0; i < n; ++i) {
            buffer[write_pos] = src[i];
            write_pos = (write_pos + 1) & mask;
        }
    }

    inline void push_one(const std::complex<float>& v)
    {
        buffer[write_pos] = v;
        write_pos = (write_pos + 1) & mask;
    }

    std::complex<float> get(size_t offset) const // offset=0 = most recent sample
    {
        size_t idx = (write_pos + capacity - offset - 1) & mask;
        return buffer[idx];
    }

    // The most recent `logical_size` samples, oldest .. newest.
    std::vector<std::complex<float>> to_vector() const
    {
        std::vector<std::complex<float>> out;
        out.reserve(logical_size);
        const size_t start = (write_pos + capacity - logical_size) & mask;
        for (size_t i = 0; i < logical_size; ++i)
            out.push_back(buffer[(start + i) & mask]);
        return out;
    }

    void clear()
    {
        write_pos = 0;
        std::fill(buffer.begin(), buffer.end(), std::complex<float>(0.0f, 0.0f));
    }
};

// ---------------------------------------------------------------------------
// Sliding-window energy: running mean of |x|^2 over the last `window` samples.
// One push per sample, O(1). This is the cheap full-rate energy gate.
// ---------------------------------------------------------------------------

class SlidingEnergy {
public:
    explicit SlidingEnergy(size_t window)
        : window_(window > 0 ? window : 1), buf_(window_, 0.0f)
    {
    }

    float push(float power)
    {
        sum_ -= buf_[pos_];
        buf_[pos_] = power;
        sum_ += power;
        pos_ = (pos_ + 1) % window_;
        if (count_ < window_)
            ++count_;
        return sum_ / static_cast<float>(count_);
    }

    void reset()
    {
        std::fill(buf_.begin(), buf_.end(), 0.0f);
        pos_ = 0;
        count_ = 0;
        sum_ = 0.0f;
    }

private:
    size_t window_;
    std::vector<float> buf_;
    size_t pos_ = 0;
    size_t count_ = 0;
    float sum_ = 0.0f;
};

// ---------------------------------------------------------------------------
// UwbDetectorStateMachine: SEARCH → IN_REGION cross-chunk region buffering.
// GNU Radio independent; the caller drives process() per input chunk and owns
// the absolute sample counter (uint64_t).
//
//   SEARCH    : a DECIMATED energy gate (every D-th sample, state carried
//               across chunks) watches for a crossing above the threshold.
//   IN_REGION : every raw sample of the candidate region is appended to a
//               buffer (plus `pre_trigger` samples before the crossing, kept
//               in a ring).  When the gate stays below the threshold for
//               `holdoff_decimated` consecutive decimated points, the region
//               ends and region_ready() turns true.
//
// Buffering the WHOLE region lets the block run the coarse-to-fine preamble
// scan on all symbols at once — no chunk-boundary artifacts.
// ---------------------------------------------------------------------------

class UwbDetectorStateMachine {
public:
    struct Region {
        uint64_t start_abs = 0;      // absolute index of samples[0] (incl. pre-trigger)
        size_t candidate_offset = 0; // offset into samples where the gate first crossed
        std::vector<std::complex<float>> samples;
    };

    UwbDetectorStateMachine(size_t pre_trigger = 2032,
                            float energy_threshold = 1e-3f,
                            size_t gate_decimation = 100,
                            size_t gate_window = 32,
                            size_t holdoff_decimated = 8)
        : pre_trigger_(pre_trigger),
          energy_threshold_(energy_threshold),
          gate_decimation_(gate_decimation > 0 ? gate_decimation : 1),
          gate_window_(gate_window > 0 ? gate_window : 1),
          holdoff_decimated_(holdoff_decimated > 0 ? holdoff_decimated : 1),
          neigh_len_(16),
          gate_(gate_window_),
          pre_ring_(pre_trigger_)
    {
        region_.samples.reserve(pre_trigger_ + 300000);
    }

    void process(const std::complex<float>* in, size_t n, uint64_t abs_sample)
    {
        for (size_t i = 0; i < n; ++i) {
            const uint64_t sample = abs_sample + i;
            pre_ring_.push_one(in[i]);

            if (state_ == IN_REGION)
                region_.samples.push_back(in[i]);

            // Decimated energy gate: accumulate |x|^2 over the first
            // `neigh_len_` samples of each D-sample block (a 16-sample
            // neighborhood reliably catches the sparse UWB pulses at any
            // phase; a single strided point does not — see the phase-sweep
            // measurement).  The gate decision updates once per block.
            // (A bulk VOLK |x|^2 pass over the whole chunk measured SLOWER:
            // the extra memory traffic outweighs the 16%-sparse scalar norm.)
            if (d_block_phase_ < neigh_len_)
                d_block_acc_ += std::norm(in[i]);
            if (++d_block_phase_ >= gate_decimation_) {
                d_block_phase_ = 0;
                const float e = gate_.push(d_block_acc_);
                d_block_acc_ = 0.0f;

                if (state_ == SEARCH) {
                    if (e >= energy_threshold_) {
                        state_ = IN_REGION;
                        // The pre-trigger ring already ends at the current
                        // sample, so samples[0..candidate_offset-1] are the
                        // pre-trigger and samples[candidate_offset] is the
                        // first candidate sample.
                        region_ = Region{};
                        region_.samples = pre_ring_.to_vector();
                        region_.candidate_offset =
                            region_.samples.size() - 1;
                        region_.start_abs =
                            (sample >= region_.candidate_offset)
                                ? sample - region_.candidate_offset
                                : 0;
                        low_count_ = 0;
                    }
                } else { // IN_REGION
                    if (e >= energy_threshold_) {
                        low_count_ = 0;
                    } else if (++low_count_ >= holdoff_decimated_) {
                        // A region ended; queue it so the block can consume it
                        // while a new region may start in the same chunk.
                        d_regions_.push_back(std::move(region_));
                        region_ = Region{};
                        state_ = SEARCH;
                        low_count_ = 0;
                    }
                }
            }
        }
    }

    bool region_ready() const { return !d_regions_.empty(); }

    Region take_region()
    {
        Region r = std::move(d_regions_.front());
        d_regions_.pop_front();
        return r;
    }

    /**
     * Force-close an in-progress region.  Called when the input stream ends so
     * a packet whose tail reaches the end of the stream is still emitted.
     */
    void flush_region()
    {
        if (state_ == IN_REGION) {
            d_regions_.push_back(std::move(region_));
            region_ = Region{};
            state_ = SEARCH;
        }
    }

    size_t pre_trigger() const { return pre_trigger_; }
    void set_pre_trigger(size_t v)
    {
        pre_trigger_ = v;
        pre_ring_ = RingBuffer(pre_trigger_);
        reset();
    }

    void reset()
    {
        state_ = SEARCH;
        d_regions_.clear();
        low_count_ = 0;
        gate_.reset();
        pre_ring_.clear();
        region_.samples.clear();
    }

private:
    enum State { SEARCH, IN_REGION };
    size_t pre_trigger_;
    float energy_threshold_;
    size_t gate_decimation_;
    size_t gate_window_;
    size_t holdoff_decimated_;
    size_t neigh_len_;   // |x|^2 samples summed per decimated block (16)
    SlidingEnergy gate_;
    RingBuffer pre_ring_;
    State state_ = SEARCH;
    Region region_;
    std::deque<Region> d_regions_; // completed regions awaiting the block
    size_t d_block_phase_ = 0;
    float d_block_acc_ = 0.0f;
    size_t low_count_ = 0;
};

} // namespace uwb
} // namespace gr
