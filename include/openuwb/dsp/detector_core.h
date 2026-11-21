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
 *
 * This header declares the public detection API and keeps the types every
 * consumer needs (constants, Sc16DotProduct, RingBufferT, SlidingEnergy,
 * DetectorEnergyGate, UwbDetectorStateMachineT) plus the short inline power
 * helpers.  The window/energy/correlation bodies and the coarse scans live in
 * lib/dsp/uwb_detector_core.cc (target openuwb_dsp), so consumers link
 * openuwb::dsp instead of re-compiling them per translation unit.
 */

#pragma once

#include <openuwb/support/defaults.h>
#include <openuwb/dsp/matched_filter.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <mutex>
#include <utility>
#include <vector>

#if defined(__AVX2__)
#include <immintrin.h>
#endif

namespace openuwb {
namespace core {

// ---------------------------------------------------------------------------
// IEEE 802.15.4a HRP PHY constants (code index 9, mean PRF 62.4 MHz, fs
// 998.4 MHz, SamplesPerPulse = 2).  The 1016-sample SYNC symbol period was
// confirmed by autocorrelation of the reference .cfile.
// ---------------------------------------------------------------------------
// Alias of defaults::kSampleRateHz (single production host-rate source).
inline constexpr double kUwbSampleRateHz = defaults::kSampleRateHz;
inline constexpr size_t kUwbSamplesPerSymbol = 1016;
inline constexpr size_t kUwbSyncSymbols = 64;
inline constexpr size_t kUwbSfdSymbols = 8;

/**
 * L2-normalize a template waveform in place.
 *
 * Matches buildUwbReference.m: preambleWaveform / (norm(preambleWaveform)+eps).
 */
inline void uwb_l2_normalize(std::vector<std::complex<float>>& t)
{
    if (t.empty())
        return;
    const float n = std::sqrt(sample_power_sum(t.data(), t.size()));
    if (n > 0.0f) {
        for (auto& v : t)
            v /= n;
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
void uwb_window_power(const std::complex<float>* in,
                      size_t n,
                      size_t L,
                      float* pow);

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
void uwb_window_energy(const std::complex<float>* in,
                       size_t n,
                       size_t window,
                       float* out);

/**
 * Normalized correlation score (Reference Detector metric, squared form):
 *
 *   metric[j] = |corr[j]|^2 / (winpow[j] * template_energy + eps)
 *
 * where corr is the raw matched-filter output at alignment j and winpow is the
 * windowed signal power over the same window.  This is the MATLAB scan metric
 * `energy = score.^2` with score = |<r,t>| / (||r|| ||t||), clamped to [0,1].
 */
void uwb_normalized_score(const std::complex<float>* corr,
                          const float* winpow,
                          size_t n,
                          float template_energy,
                          float* metric);

/**
 * Full-rate normalized correlation over inclusive SYNC-start candidates.
 * Unlike fir_filter convolution coordinates, j is directly the first input
 * sample aligned with tmpl[0], matching MATLAB's <rx(j:j+L-1), tmpl> scan.
 * All scratch arrays are caller-owned and must hold j1-j0+1 elements.
 */
bool uwb_full_rate_peak(const std::complex<float>* in,
                        size_t n,
                        const std::complex<float>* tmpl,
                        size_t L,
                        float template_energy,
                        size_t j0,
                        size_t j1,
                        std::complex<float>* corr,
                        float* winpow,
                        float* metric,
                        size_t* best_start,
                        float* best_metric);

/**
 * Energy of a template waveform (sum of |x|^2). Used as the normalization
 * factor in uwb_normalized_score when the template is not unit-norm.
 */
inline float uwb_template_energy(const std::vector<std::complex<float>>& t)
{
    return sample_power_sum(t.data(), t.size());
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
void uwb_candidate_ranges(const float* energy,
                          size_t n,
                          float threshold,
                          size_t margin,
                          std::vector<std::pair<size_t, size_t>>& ranges);

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
void uwb_energy_gate_strided(const std::complex<float>* in,
                             size_t n,
                             size_t D,
                             size_t win,
                             float threshold,
                             size_t margin,
                             std::vector<std::pair<size_t, size_t>>& ranges);

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
void uwb_coarse_peaks(const std::complex<float>* in,
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
                      float* max_metric);

struct Sc16DotProduct {
    int64_t real = 0;
    int64_t imag = 0;
};

Sc16DotProduct uwb_sc16_conjugate_dot_product(
    const std::complex<int16_t>* input,
    const std::complex<int16_t>* taps,
    const std::complex<int16_t>* imag_taps,
    size_t count);

/**
 * Q15/SC16 equivalent of uwb_coarse_peaks().  The signal and normalized
 * template remain integer through decimation, window power and correlation;
 * only the normalized score is converted to floating point for the existing
 * peak/existence thresholds.  Scratch storage is caller-owned.
 */
void uwb_coarse_peaks_sc16(
    const std::complex<int16_t>* in,
    size_t scan_start,
    size_t range_end,
    const std::complex<int16_t>* tmpl_ds,
    const std::complex<int16_t>* tmpl_imag_ds,
    size_t Ld,
    size_t D,
    size_t R,
    size_t sym_ds,
    float peak_rel,
    float exist_frac,
    size_t stride,
    std::vector<std::complex<int16_t>>& sig_ds,
    std::vector<uint64_t>& pow_ds,
    std::vector<float>& score_ds,
    std::vector<float>& metric_ds,
    std::vector<size_t>& peaks,
    float* max_metric);

} // namespace core

// ---------------------------------------------------------------------------
// Packet extraction core (SEARCH → CAPTURE → HOLD_OFF). Kept GR-independent so
// it can be unit-tested and benchmarked without linking against GNU Radio.
// ---------------------------------------------------------------------------

// RingBuffer: circular buffer holding the most recent `capacity` samples.
// Used for pre-trigger / history so a packet detected across an input chunk
// can recover the samples that arrived before the trigger.
// ---------------------------------------------------------------------------

template <typename Sample>
struct RingBufferT {
    // Physical capacity is rounded up to a power of two so the write index
    // wraps with a bit-mask (hot path: one push per input sample).  The
    // *logical* size is the caller-requested capacity and is what to_vector()
    // returns — the pre-trigger window is exactly `logical_size` samples.
    size_t capacity;
    size_t mask;
    size_t logical_size;
    size_t write_pos = 0;
    std::vector<Sample> buffer;

    explicit RingBufferT(size_t cap)
        : capacity(round_up_pow2(cap)),
          mask(capacity - 1),
          logical_size(cap),
          buffer(capacity, Sample{})
    {
    }

    static size_t round_up_pow2(size_t v)
    {
        size_t p = 1;
        while (p < v)
            p <<= 1;
        return p;
    }

    void push(const Sample* src, size_t n)
    {
        if (n == 0)
            return;
        if (n >= capacity) {
            src += n - capacity;
            n = capacity;
        }

        const size_t first = std::min(n, capacity - write_pos);
        std::memcpy(buffer.data() + write_pos, src, first * sizeof(*src));
        const size_t second = n - first;
        if (second > 0)
            std::memcpy(buffer.data(), src + first, second * sizeof(*src));
        write_pos = (write_pos + n) & mask;
    }

    inline void push_one(const Sample& v)
    {
        buffer[write_pos] = v;
        write_pos = (write_pos + 1) & mask;
    }

    Sample get(size_t offset) const // offset=0 = most recent sample
    {
        size_t idx = (write_pos + capacity - offset - 1) & mask;
        return buffer[idx];
    }

    // Copy the most recent `logical_size` samples, oldest .. newest, into a
    // caller-owned preallocated destination.
    void copy_to(Sample* out) const
    {
        if (logical_size == 0)
            return;
        const size_t start = (write_pos + capacity - logical_size) & mask;
        const size_t first = std::min(logical_size, capacity - start);
        std::memcpy(out, buffer.data() + start, first * sizeof(*out));
        const size_t second = logical_size - first;
        if (second > 0)
            std::memcpy(out + first, buffer.data(), second * sizeof(*out));
    }

    std::vector<Sample> to_vector() const
    {
        std::vector<Sample> out(logical_size);
        copy_to(out.data());
        return out;
    }

    void clear()
    {
        write_pos = 0;
        std::fill(buffer.begin(), buffer.end(), Sample{});
    }
};

using RingBuffer = RingBufferT<std::complex<float>>;

inline float detector_sample_power(const std::complex<float>& v)
{
    return std::norm(v);
}

inline float detector_sample_power(const std::complex<int16_t>& v)
{
    constexpr float kInvFullScale2 = 1.0f / (32768.0f * 32768.0f);
    const int32_t re = v.real();
    const int32_t im = v.imag();
    const uint64_t raw = static_cast<uint64_t>(static_cast<int64_t>(re) * re) +
                         static_cast<uint64_t>(static_cast<int64_t>(im) * im);
    return static_cast<float>(raw) * kInvFullScale2;
}

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

// Gate-domain policy.  FC32 retains the MATLAB-compatible floating-point
// energy calculation.  SC16 stays entirely in the ADC integer domain: I^2+Q^2
// is accumulated in uint64_t and compared with thresholds scaled once during
// construction.  This avoids a per-sample int->float conversion and
// normalization while preserving the same normalized threshold semantics.
template <typename Sample>
class DetectorEnergyGate {
public:
    DetectorEnergyGate(size_t window, float threshold)
        : gate_(window), threshold_(threshold)
    {
    }

    void accumulate(const Sample& sample)
    {
        block_acc_ += detector_sample_power(sample);
    }

    bool finish_block()
    {
        const float energy = gate_.push(block_acc_);
        block_acc_ = 0.0f;
        return energy >= threshold_;
    }

    void reset()
    {
        gate_.reset();
        block_acc_ = 0.0f;
    }

private:
    SlidingEnergy gate_;
    float threshold_;
    float block_acc_ = 0.0f;
};

template <>
class DetectorEnergyGate<std::complex<int16_t>> {
public:
    DetectorEnergyGate(size_t window, float threshold)
        : window_(window > 0 ? window : 1),
          ring_(window_, 0),
          thresholds_(window_)
    {
        constexpr double full_scale_squared = 32768.0 * 32768.0;
        for (size_t count = 1; count <= window_; ++count) {
            const double scaled = static_cast<double>(threshold) *
                                  full_scale_squared * static_cast<double>(count);
            if (scaled <= 0.0) {
                thresholds_[count - 1] = 0;
            } else if (scaled >= static_cast<double>(
                                     std::numeric_limits<uint64_t>::max())) {
                thresholds_[count - 1] = std::numeric_limits<uint64_t>::max();
            } else {
                thresholds_[count - 1] =
                    static_cast<uint64_t>(std::ceil(scaled));
            }
        }
    }

    void accumulate(const std::complex<int16_t>& sample)
    {
        const int64_t re = sample.real();
        const int64_t im = sample.imag();
        block_acc_ += static_cast<uint64_t>(re * re) +
                      static_cast<uint64_t>(im * im);
    }

    bool finish_block()
    {
        window_sum_ -= ring_[pos_];
        ring_[pos_] = block_acc_;
        window_sum_ += block_acc_;
        block_acc_ = 0;
        pos_ = (pos_ + 1) % window_;
        if (count_ < window_)
            ++count_;
        return window_sum_ >= thresholds_[count_ - 1];
    }

    void reset()
    {
        std::fill(ring_.begin(), ring_.end(), uint64_t{ 0 });
        pos_ = 0;
        count_ = 0;
        window_sum_ = 0;
        block_acc_ = 0;
    }

private:
    size_t window_;
    std::vector<uint64_t> ring_;
    std::vector<uint64_t> thresholds_;
    size_t pos_ = 0;
    size_t count_ = 0;
    uint64_t window_sum_ = 0;
    uint64_t block_acc_ = 0;
};

// ---------------------------------------------------------------------------
// UwbDetectorStateMachine: SEARCH → IN_REGION → CAPTURE cross-chunk buffering.
// GNU Radio independent; the caller drives process() per input chunk and owns
// the absolute sample counter (uint64_t).
//
//   SEARCH    : a DECIMATED energy gate (every D-th sample, state carried
//               across chunks) watches for a crossing above the threshold.
//   IN_REGION : every raw sample of the candidate region is appended to a
//               buffer (plus `pre_trigger` samples before the crossing, kept
//               in a ring).  When the gate stays below the threshold for
//               `holdoff_decimated` consecutive decimated points, the energy
//               Region ends.
//   CAPTURE    : if configured, bulk-copy silence/tail samples until the
//               trigger-relative capture horizon is complete.  This guarantees
//               that a later preamble confirmation can emit a fixed-size PDU.
//
// Buffering the WHOLE region lets the block run the coarse-to-fine preamble
// scan on all symbols at once — no chunk-boundary artifacts.
// ---------------------------------------------------------------------------

template <typename Sample>
class UwbDetectorStateMachineT {
public:
    struct Region {
        uint64_t start_abs = 0;      // absolute index of samples[0] (incl. pre-trigger)
        size_t candidate_offset = 0; // offset into samples where the gate first crossed
        std::vector<Sample> samples;
    };

    using RegionHandle = size_t;
    static constexpr size_t kRegionPoolSize = 8;
    static constexpr RegionHandle kInvalidRegion = std::numeric_limits<size_t>::max();

    UwbDetectorStateMachineT(size_t pre_trigger = defaults::kDetectorPreTrigger,
                            float energy_threshold =
                                defaults::kDetectorEnergyThreshold,
                            size_t gate_decimation = 100,
                            size_t gate_window = 32,
                            size_t holdoff_decimated = 8,
                            size_t post_trigger_capture = 0)
        : pre_trigger_(pre_trigger),
          gate_decimation_(gate_decimation > 0 ? gate_decimation : 1),
          gate_window_(gate_window > 0 ? gate_window : 1),
          holdoff_decimated_(holdoff_decimated > 0 ? holdoff_decimated : 1),
          post_trigger_capture_(post_trigger_capture),
          neigh_len_(16),
          gate_(gate_window_, energy_threshold),
          pre_ring_(pre_trigger_),
          max_region_samples_(std::max<size_t>(
              pre_trigger_ + std::max<size_t>(post_trigger_capture_, 300000),
              524288))
    {
        for (size_t i = 0; i < kRegionPoolSize; ++i) {
            d_region_pool_[i].samples.reserve(max_region_samples_);
            d_free_regions_[i] = kRegionPoolSize - 1 - i;
        }
        d_free_count_ = kRegionPoolSize;
    }

    void process(const Sample* in, size_t n, uint64_t abs_sample)
    {
        size_t consumed = 0;
        while (consumed < n) {
            if (state_ == SEARCH)
                consumed += process_search(in + consumed, n - consumed,
                                           abs_sample + consumed);
            else if (state_ == IN_REGION)
                consumed += process_region(in + consumed, n - consumed,
                                           abs_sample + consumed);
            else
                consumed += process_capture(in + consumed, n - consumed,
                                            abs_sample + consumed);
        }
    }

    bool region_ready() const { return d_ready_count_ != 0; }

    RegionHandle take_region()
    {
        const RegionHandle handle = d_ready_regions_[d_ready_head_];
        d_ready_head_ = (d_ready_head_ + 1) % kRegionPoolSize;
        --d_ready_count_;
        return handle;
    }

    const Region& region(RegionHandle handle) const { return d_region_pool_[handle]; }

    void release_region(RegionHandle handle)
    {
        if (handle == kInvalidRegion)
            return;
        d_region_pool_[handle].samples.clear();
        std::lock_guard<std::mutex> lock(d_pool_mutex_);
        d_free_regions_[d_free_count_++] = handle;
    }

    uint64_t dropped_regions() const { return d_dropped_regions_; }

    /**
     * Force-close an in-progress region.  Called when the input stream ends so
     * a packet whose tail reaches the end of the stream is still emitted.
     */
    void flush_region()
    {
        if (state_ == IN_REGION) {
            finish_region();
            state_ = SEARCH;
            low_count_ = 0;
        }
    }

    size_t pre_trigger() const { return pre_trigger_; }
    void set_pre_trigger(size_t v)
    {
        pre_trigger_ = v;
        pre_ring_ = RingBufferT<Sample>(pre_trigger_);
        max_region_samples_ = std::max<size_t>(
            pre_trigger_ + std::max<size_t>(post_trigger_capture_, 300000),
            524288);
        for (auto& region : d_region_pool_) {
            if (region.samples.capacity() < max_region_samples_)
                region.samples.reserve(max_region_samples_);
        }
        reset();
    }

    void reset()
    {
        state_ = SEARCH;
        low_count_ = 0;
        gate_.reset();
        pre_ring_.clear();
        d_block_phase_ = 0;
        d_ready_head_ = 0;
        d_ready_tail_ = 0;
        d_ready_count_ = 0;
        d_active_region_ = kInvalidRegion;
        capture_end_abs_ = 0;
        d_dropped_regions_ = 0;
        std::lock_guard<std::mutex> lock(d_pool_mutex_);
        d_free_count_ = 0;
        for (size_t i = 0; i < kRegionPoolSize; ++i) {
            d_region_pool_[i].samples.clear();
            d_free_regions_[d_free_count_++] = kRegionPoolSize - 1 - i;
        }
    }

private:
    // SEARCH is block-oriented: one iteration handles up to a complete
    // D-sample gate block.  Ring maintenance is a bulk memcpy and only the
    // first `neigh_len_` samples of each block enter the energy sum.
    size_t process_search(const Sample* in,
                          size_t n,
                          uint64_t abs_sample)
    {
        size_t consumed = 0;
        while (consumed < n && state_ == SEARCH) {
            const size_t block_left = gate_decimation_ - d_block_phase_;
            const size_t take = std::min(block_left, n - consumed);
            const size_t energy_count =
                (d_block_phase_ < neigh_len_)
                    ? std::min(take, neigh_len_ - d_block_phase_)
                    : 0;
            for (size_t j = 0; j < energy_count; ++j)
                gate_.accumulate(in[consumed + j]);

            pre_ring_.push(in + consumed, take);
            d_block_phase_ += take;
            consumed += take;
            if (d_block_phase_ == gate_decimation_) {
                d_block_phase_ = 0;
                update_gate(abs_sample + consumed - 1);
            }
        }
        return consumed;
    }

    // IN_REGION retains raw IQ in block-sized inserts until the gate remains
    // low for the configured holdoff.
    size_t process_region(const Sample* in,
                          size_t n,
                          uint64_t abs_sample)
    {
        size_t consumed = 0;
        while (consumed < n && state_ == IN_REGION) {
            const size_t block_left = gate_decimation_ - d_block_phase_;
            const size_t take = std::min(block_left, n - consumed);
            const size_t energy_count =
                (d_block_phase_ < neigh_len_)
                    ? std::min(take, neigh_len_ - d_block_phase_)
                    : 0;
            for (size_t j = 0; j < energy_count; ++j)
                gate_.accumulate(in[consumed + j]);

            pre_ring_.push(in + consumed, take);
            if (d_active_region_ != kInvalidRegion) {
                auto& samples = d_region_pool_[d_active_region_].samples;
                if (samples.size() + take <= max_region_samples_) {
                    samples.insert(samples.end(), in + consumed, in + consumed + take);
                } else {
                    const RegionHandle overflow = d_active_region_;
                    d_active_region_ = kInvalidRegion;
                    release_region(overflow);
                    ++d_dropped_regions_;
                }
            }
            d_block_phase_ += take;
            consumed += take;
            if (d_block_phase_ == gate_decimation_) {
                d_block_phase_ = 0;
                update_gate(abs_sample + consumed - 1);
            }
        }
        return consumed;
    }

    // Once the energy gate closes, no more detection math is needed.  Copy in
    // bulk until the conservative trigger-relative horizon is present.  Since
    // packet_start is at or before the delayed energy trigger, this contains
    // every sample through packet_start + post_trigger_capture_.
    size_t process_capture(const Sample* in,
                           size_t n,
                           uint64_t abs_sample)
    {
        if (abs_sample >= capture_end_abs_) {
            finish_region();
            state_ = SEARCH;
            return 0;
        }
        const uint64_t remaining64 = capture_end_abs_ - abs_sample;
        const size_t take = static_cast<size_t>(
            std::min<uint64_t>(remaining64, static_cast<uint64_t>(n)));
        if (d_active_region_ != kInvalidRegion) {
            auto& samples = d_region_pool_[d_active_region_].samples;
            if (samples.size() + take <= max_region_samples_) {
                samples.insert(samples.end(), in, in + take);
            } else {
                const RegionHandle overflow = d_active_region_;
                d_active_region_ = kInvalidRegion;
                release_region(overflow);
                ++d_dropped_regions_;
            }
        }
        if (abs_sample + take >= capture_end_abs_) {
            finish_region();
            state_ = SEARCH;
        }
        return take;
    }

    void update_gate(uint64_t sample)
    {
        const bool above_threshold = gate_.finish_block();
        if (state_ == SEARCH) {
            if (!above_threshold)
                return;

            state_ = IN_REGION;
            d_active_region_ = acquire_region();
            if (d_active_region_ != kInvalidRegion) {
                Region& region = d_region_pool_[d_active_region_];
                region.samples.resize(pre_ring_.logical_size);
                pre_ring_.copy_to(region.samples.data());
                region.candidate_offset = region.samples.size() - 1;
                region.start_abs = (sample >= region.candidate_offset)
                                       ? sample - region.candidate_offset
                                       : 0;
                capture_end_abs_ = sample + 1 + post_trigger_capture_;
            } else {
                ++d_dropped_regions_;
            }
            low_count_ = 0;
            return;
        }

        if (above_threshold) {
            low_count_ = 0;
        } else if (++low_count_ >= holdoff_decimated_) {
            if (post_trigger_capture_ > 0 && sample + 1 < capture_end_abs_) {
                state_ = CAPTURE;
            } else {
                finish_region();
                state_ = SEARCH;
            }
            low_count_ = 0;
        }
    }

    RegionHandle acquire_region()
    {
        std::lock_guard<std::mutex> lock(d_pool_mutex_);
        if (d_free_count_ == 0)
            return kInvalidRegion;
        return d_free_regions_[--d_free_count_];
    }

    void finish_region()
    {
        if (d_active_region_ == kInvalidRegion)
            return;
        if (d_ready_count_ < kRegionPoolSize) {
            d_ready_regions_[d_ready_tail_] = d_active_region_;
            d_ready_tail_ = (d_ready_tail_ + 1) % kRegionPoolSize;
            ++d_ready_count_;
        } else {
            release_region(d_active_region_);
            ++d_dropped_regions_;
        }
        d_active_region_ = kInvalidRegion;
    }

    enum State { SEARCH, IN_REGION, CAPTURE };
    size_t pre_trigger_;
    size_t gate_decimation_;
    size_t gate_window_;
    size_t holdoff_decimated_;
    size_t post_trigger_capture_;
    size_t neigh_len_;   // |x|^2 samples summed per decimated block (16)
    DetectorEnergyGate<Sample> gate_;
    RingBufferT<Sample> pre_ring_;
    size_t max_region_samples_;
    State state_ = SEARCH;
    std::array<Region, kRegionPoolSize> d_region_pool_;
    std::array<RegionHandle, kRegionPoolSize> d_free_regions_{};
    std::array<RegionHandle, kRegionPoolSize> d_ready_regions_{};
    mutable std::mutex d_pool_mutex_;
    size_t d_free_count_ = 0;
    size_t d_ready_head_ = 0;
    size_t d_ready_tail_ = 0;
    size_t d_ready_count_ = 0;
    RegionHandle d_active_region_ = kInvalidRegion;
    uint64_t d_dropped_regions_ = 0;
    size_t d_block_phase_ = 0;
    size_t low_count_ = 0;
    uint64_t capture_end_abs_ = 0;
};

using UwbDetectorStateMachine = UwbDetectorStateMachineT<std::complex<float>>;
using UwbDetectorStateMachineSc16 =
    UwbDetectorStateMachineT<std::complex<int16_t>>;

} // namespace openuwb
