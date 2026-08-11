/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Pure fixed 65/48 rational resampler core (no GNU Radio / PMT deps).
 *
 * Contract: docs/performance/规格_固定65_48重采样core契约.md
 * Reference: scipy/MATLAB upfirdn (zero-padded full convolution)
 *
 *   Lout = ceil(((N-1)*L + T) / M)
 *   y[m] = sum_k h[(m*M mod L) + L*k] * x[floor(m*M/L) - k]
 *   x[j] = 0 outside [0, N)
 *
 * Streaming is chunk-invariant.  No allocation in process()/flush() hot path
 * after the first capacity reserve.
 *
 * Kernels (kernel_name()):
 *   scalar_macroblock       — reference FIR + 65-out macroblock schedule
 *   volk_macroblock         — VOLK FIR + macroblock
 *   avx2_fma_macroblock     — AVX2/FMA FIR + macroblock (default if built)
 *   *_legacy                — per-output loop (Phase-2 baseline for A/B)
 *
 * Multi-worker: process(..., nworkers) serial schedule + parallel FIR dots;
 * output equals nworkers=1 within float tolerance (prefer bitwise).
 */

#pragma once

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <complex>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <functional>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <volk/volk.h>

#if defined(__AVX2__) && defined(__FMA__)
#include <immintrin.h>
#define UWB_RESAMP_HAVE_AVX2 1
#else
#define UWB_RESAMP_HAVE_AVX2 0
#endif

namespace gr {
namespace uwb {
namespace core {

class RationalResampler65_48Core
{
public:
    static constexpr uint32_t kInterp = 65;
    static constexpr uint32_t kDecim = 48;

    using gr_complex = std::complex<float>;

    struct ProcessResult {
        size_t produced = 0;
        size_t consumed = 0;
    };

    struct ProfileStats {
        uint64_t calls = 0;
        uint64_t outputs = 0;
        uint64_t macroblocks = 0;
        double ns_assemble = 0.0;
        double ns_schedule = 0.0;
        double ns_fir = 0.0;
        double ns_state = 0.0;
        double ns_total = 0.0;
        void reset() { *this = ProfileStats{}; }
    };

    RationalResampler65_48Core(const float* taps, size_t T)
        : T_(T),
          H_((T + kInterp - 1) / kInterp),
          input_items_(0),
          output_items_(0),
          resets_(0),
          ctr_(0),
          have_current_(false),
          current_(0.0f, 0.0f),
          flush_mode_(false),
          nworkers_(1),
          profile_enabled_(false)
    {
        if (taps == nullptr || T == 0) {
            throw std::invalid_argument(
                "RationalResampler65_48Core: taps must be non-empty");
        }
        if (H_ == 0) {
            throw std::invalid_argument(
                "RationalResampler65_48Core: invalid arm length");
        }

        arms_.assign(static_cast<size_t>(kInterp) * H_, 0.0f);
        arms_rev_.assign(static_cast<size_t>(kInterp) * H_, 0.0f);
        for (size_t i = 0; i < T; ++i) {
            const size_t arm = i % kInterp;
            const size_t k = i / kInterp;
            arms_[arm * H_ + k] = taps[i];
        }
        for (size_t a = 0; a < kInterp; ++a) {
            for (size_t k = 0; k < H_; ++k) {
                arms_rev_[a * H_ + (H_ - 1 - k)] = arms_[a * H_ + k];
            }
        }

        hist_.assign(H_ > 0 ? H_ - 1 : 0, gr_complex(0.0f, 0.0f));

        work_.reserve(H_ + (1u << 20));
        work_.resize(H_);

        schedule_arm_.resize(static_cast<size_t>(kInterp) * kInterp);
        schedule_adv_.resize(static_cast<size_t>(kInterp) * kInterp);
        for (uint32_t p = 0; p < kInterp; ++p) {
            uint32_t c = p;
            for (uint32_t o = 0; o < kInterp; ++o) {
                const size_t idx =
                    static_cast<size_t>(p) * kInterp + o;
                schedule_arm_[idx] = static_cast<uint8_t>(c);
                c += kDecim;
                uint8_t adv = 0;
                while (c >= kInterp) {
                    c -= kInterp;
                    ++adv;
                }
                schedule_adv_[idx] = adv;
            }
        }

        const size_t kMaxOutScratch = 1u << 20;
        sched_arm_buf_.resize(kMaxOutScratch);
        sched_win_buf_.resize(kMaxOutScratch);

        select_default_kernel();
    }

    explicit RationalResampler65_48Core(const std::vector<float>& taps)
        : RationalResampler65_48Core(taps.data(), taps.size())
    {
    }

    ~RationalResampler65_48Core() { stop_pool(); }

    RationalResampler65_48Core(const RationalResampler65_48Core&) = delete;
    RationalResampler65_48Core&
    operator=(const RationalResampler65_48Core&) = delete;

    static size_t expected_output_length(uint64_t N, size_t T)
    {
        const int64_t num =
            (static_cast<int64_t>(N) - 1) * static_cast<int64_t>(kInterp) +
            static_cast<int64_t>(T);
        if (num <= 0)
            return 0;
        return static_cast<size_t>((num + static_cast<int64_t>(kDecim) - 1) /
                                   static_cast<int64_t>(kDecim));
    }

    size_t expected_output_length(uint64_t N) const
    {
        return expected_output_length(N, T_);
    }

    void force_scalar_kernel()
    {
        fir_kind_ = FirKind::Scalar;
        loop_kind_ = LoopKind::Macroblock;
        kernel_name_ = "scalar_macroblock";
    }

    void enable_volk_kernel()
    {
        fir_kind_ = FirKind::Volk;
        loop_kind_ = LoopKind::Macroblock;
        kernel_name_ = "volk_macroblock";
    }

    /** Force kernel for A/B: scalar|scalar_legacy|volk|volk_legacy|avx2|default */
    void set_kernel(const std::string& name)
    {
        if (name == "scalar" || name == "scalar_macroblock") {
            force_scalar_kernel();
        } else if (name == "scalar_legacy") {
            fir_kind_ = FirKind::Scalar;
            loop_kind_ = LoopKind::Legacy;
            kernel_name_ = "scalar_legacy";
        } else if (name == "volk" || name == "volk_macroblock") {
            enable_volk_kernel();
        } else if (name == "volk_legacy") {
            fir_kind_ = FirKind::Volk;
            loop_kind_ = LoopKind::Legacy;
            kernel_name_ = "volk_legacy";
        } else if (name == "avx2" || name == "avx2_fma_macroblock" ||
                   name == "default") {
#if UWB_RESAMP_HAVE_AVX2
            fir_kind_ = FirKind::Avx2;
            loop_kind_ = LoopKind::Macroblock;
            kernel_name_ = "avx2_fma_macroblock";
#else
            enable_volk_kernel();
#endif
        } else {
            throw std::invalid_argument(
                "RationalResampler65_48Core::set_kernel: unknown " + name);
        }
    }

    void set_num_workers(int n)
    {
        n = (n < 1) ? 1 : n;
        if (n == nworkers_)
            return;
        stop_pool();
        nworkers_ = n;
        if (nworkers_ > 1)
            start_pool(nworkers_);
    }

    int num_workers() const { return nworkers_; }

    void enable_profiling(bool on)
    {
        profile_enabled_ = on;
        if (on)
            profile_.reset();
    }

    bool profiling_enabled() const { return profile_enabled_; }
    const ProfileStats& profile_stats() const { return profile_; }
    void reset_profile() { profile_.reset(); }

    ProcessResult process(const gr_complex* in,
                          size_t ninput,
                          gr_complex* out,
                          size_t max_out,
                          int nworkers_override = 0)
    {
        const auto t_all0 =
            profile_enabled_ ? Clock::now() : Clock::time_point{};

        if (flush_mode_) {
            throw std::logic_error(
                "RationalResampler65_48Core: process() after flush without "
                "reset()");
        }

        const size_t hist_n = hist_.size(); // H-1
        const bool started_held = have_current_;
        const size_t held = started_held ? 1u : 0u;
        const size_t post_n = held + ninput;
        if (post_n == 0 || max_out == 0) {
            return ProcessResult{};
        }

        // Snapshot input count before any local accounting.
        const uint64_t in_count_entry = input_items_;

        // ---- (a) work buffer assembly ----
        const auto t_a0 =
            profile_enabled_ ? Clock::now() : Clock::time_point{};
        const size_t need = hist_n + post_n;
        if (work_.capacity() < need)
            work_.reserve(need);
        if (work_.size() < need)
            work_.resize(need);

        if (hist_n > 0) {
            std::memcpy(work_.data(), hist_.data(),
                        hist_n * sizeof(gr_complex));
        }
        size_t wpos = hist_n;
        if (started_held)
            work_[wpos++] = current_;
        if (ninput > 0) {
            std::memcpy(work_.data() + wpos, in,
                        ninput * sizeof(gr_complex));
            wpos += ninput;
        }
        const size_t total_w = wpos;
        if (profile_enabled_)
            profile_.ns_assemble += ns_since(t_a0);

        size_t post_i = 0;
        have_current_ = true;
        current_ = work_[hist_n + post_i];

        const int nw =
            (nworkers_override > 0) ? nworkers_override : nworkers_;

        ProcessResult r;
        if (loop_kind_ == LoopKind::Legacy) {
            r = run_legacy(hist_n, post_n, post_i, started_held, ninput,
                           total_w, out, max_out);
        } else if (nw > 1 && max_out >= 64 && post_n >= 64) {
            r = run_mt(hist_n, post_n, post_i, started_held, ninput, total_w,
                       out, max_out, nw);
        } else {
            r = run_macroblock(hist_n, post_n, post_i, started_held, ninput,
                               total_w, out, max_out);
        }

        // Absolute stream samples taken as current = entry + consumed.
        input_items_ = in_count_entry + r.consumed;

        if (profile_enabled_) {
            profile_.calls += 1;
            profile_.outputs += r.produced;
            profile_.ns_total += ns_since(t_all0);
        }
        return r;
    }

    size_t process(const gr_complex* in, size_t ninput, gr_complex* out)
    {
        return process(in, ninput, out, SIZE_MAX / 4).produced;
    }

    size_t flush(gr_complex* out, size_t max_out)
    {
        flush_mode_ = true;
        const uint64_t N = input_items_;
        const size_t Lout = expected_output_length(N, T_);
        size_t produced = 0;

        const size_t hist_n = hist_.size();
        if (work_.size() < hist_n + 1)
            work_.resize(hist_n + 1);
        if (hist_n > 0)
            std::memcpy(work_.data(), hist_.data(),
                        hist_n * sizeof(gr_complex));

        while (produced < max_out && output_items_ < Lout) {
            if (!have_current_) {
                current_ = gr_complex(0.0f, 0.0f);
                have_current_ = true;
            }
            work_[hist_n] = current_;
            const gr_complex* win =
                (H_ > 1) ? work_.data() + (hist_n + 1 - H_) : &current_;
            out[produced++] = filter_window(ctr_, win);
            ++output_items_;
            ctr_ += kDecim;
            while (ctr_ >= kInterp) {
                ctr_ -= kInterp;
                if (hist_n > 0) {
                    std::memmove(hist_.data(),
                                 hist_.data() + 1,
                                 (hist_n - 1) * sizeof(gr_complex));
                    hist_[hist_n - 1] = current_;
                    std::memcpy(work_.data(), hist_.data(),
                                hist_n * sizeof(gr_complex));
                }
                current_ = gr_complex(0.0f, 0.0f);
                have_current_ = true;
            }
        }
        return produced;
    }

    bool flush_complete() const
    {
        return flush_mode_ &&
               output_items_ >= expected_output_length(input_items_, T_);
    }

    void reset()
    {
        std::fill(hist_.begin(), hist_.end(), gr_complex(0.0f, 0.0f));
        ctr_ = 0;
        have_current_ = false;
        current_ = gr_complex(0.0f, 0.0f);
        input_items_ = 0;
        output_items_ = 0;
        flush_mode_ = false;
        ++resets_;
    }

    uint64_t input_items() const { return input_items_; }
    uint64_t output_items() const { return output_items_; }
    uint32_t phase() const { return ctr_; }
    uint64_t resets() const { return resets_; }

    size_t tap_count() const { return T_; }
    size_t arm_length() const { return H_; }
    size_t history_extra() const { return H_ > 0 ? H_ - 1 : 0; }
    const char* kernel_name() const { return kernel_name_; }

    float arm_tap(size_t arm, size_t k) const
    {
        return arms_.at(arm * H_ + k);
    }

    int64_t map_input_offset_to_output(int64_t p) const
    {
        const double d = 0.5 * static_cast<double>(T_ > 0 ? T_ - 1 : 0);
        const double m =
            (static_cast<double>(p) * static_cast<double>(kInterp) + d) /
            static_cast<double>(kDecim);
        const int64_t r = static_cast<int64_t>(std::llround(m));
        return r < 0 ? 0 : r;
    }

private:
    using Clock = std::chrono::steady_clock;

    enum class FirKind { Scalar, Volk, Avx2 };
    enum class LoopKind { Legacy, Macroblock };

    size_t T_;
    size_t H_;
    std::vector<float> arms_;
    std::vector<float> arms_rev_;
    std::vector<gr_complex> hist_;
    std::vector<gr_complex> work_;
    std::vector<uint8_t> schedule_arm_;
    std::vector<uint8_t> schedule_adv_;
    std::vector<uint8_t> sched_arm_buf_;
    std::vector<uint32_t> sched_win_buf_;

    uint64_t input_items_;
    uint64_t output_items_;
    uint64_t resets_;
    uint32_t ctr_;
    bool have_current_;
    gr_complex current_;
    bool flush_mode_;
    FirKind fir_kind_ = FirKind::Volk;
    LoopKind loop_kind_ = LoopKind::Macroblock;
    const char* kernel_name_ = "volk_macroblock";
    int nworkers_;
    bool profile_enabled_;
    ProfileStats profile_;

    // Persistent worker pool (avoids per-call thread spawn).
    // Epoch protocol: main bumps epoch and sets job; each worker runs once
    // per epoch (tracked by pool_seen_[t]).
    std::vector<std::thread> pool_;
    std::vector<uint64_t> pool_seen_;
    std::mutex pool_mu_;
    std::condition_variable pool_cv_;
    std::condition_variable pool_done_cv_;
    std::function<void(size_t, size_t)> pool_job_;
    std::atomic<uint64_t> pool_epoch_{ 0 };
    std::atomic<int> pool_done_{ 0 };
    std::atomic<bool> pool_stop_{ false };
    int pool_size_ = 0;
    size_t pool_job_n_ = 0;

    static double ns_since(Clock::time_point t0)
    {
        return std::chrono::duration<double, std::nano>(Clock::now() - t0)
            .count();
    }

    void start_pool(int n)
    {
        stop_pool();
        pool_stop_.store(false, std::memory_order_relaxed);
        pool_epoch_.store(0, std::memory_order_relaxed);
        pool_size_ = n;
        pool_seen_.assign(static_cast<size_t>(n), 0);
        pool_.reserve(static_cast<size_t>(n));
        for (int t = 0; t < n; ++t) {
            pool_.emplace_back([this, t, n]() {
                uint64_t local_seen = 0;
                for (;;) {
                    std::function<void(size_t, size_t)> job;
                    size_t job_n = 0;
                    {
                        std::unique_lock<std::mutex> lk(pool_mu_);
                        pool_cv_.wait(lk, [this, &local_seen] {
                            return pool_stop_.load(std::memory_order_relaxed) ||
                                   pool_epoch_.load(std::memory_order_acquire) >
                                       local_seen;
                        });
                        if (pool_stop_.load(std::memory_order_relaxed) &&
                            pool_epoch_.load(std::memory_order_acquire) <=
                                local_seen)
                            return;
                        local_seen =
                            pool_epoch_.load(std::memory_order_acquire);
                        job = pool_job_;
                        job_n = pool_job_n_;
                    }
                    if (job) {
                        const size_t chunk =
                            (job_n + static_cast<size_t>(n) - 1) /
                            static_cast<size_t>(n);
                        const size_t lo = static_cast<size_t>(t) * chunk;
                        const size_t hi = std::min(job_n, lo + chunk);
                        if (lo < hi)
                            job(lo, hi);
                    }
                    if (pool_done_.fetch_add(1, std::memory_order_acq_rel) +
                            1 >=
                        n) {
                        pool_done_cv_.notify_one();
                    }
                }
            });
        }
    }

    void stop_pool()
    {
        {
            std::lock_guard<std::mutex> lk(pool_mu_);
            pool_stop_.store(true, std::memory_order_relaxed);
        }
        pool_cv_.notify_all();
        for (auto& th : pool_) {
            if (th.joinable())
                th.join();
        }
        pool_.clear();
        pool_seen_.clear();
        pool_size_ = 0;
        pool_stop_.store(false, std::memory_order_relaxed);
        pool_epoch_.store(0, std::memory_order_relaxed);
    }

    void pool_parallel_for(size_t n,
                           const std::function<void(size_t, size_t)>& job)
    {
        if (n == 0)
            return;
        if (pool_size_ <= 1 || pool_.empty()) {
            job(0, n);
            return;
        }
        {
            std::lock_guard<std::mutex> lk(pool_mu_);
            pool_job_ = job;
            pool_job_n_ = n;
            pool_done_.store(0, std::memory_order_relaxed);
            pool_epoch_.fetch_add(1, std::memory_order_acq_rel);
        }
        pool_cv_.notify_all();
        {
            std::unique_lock<std::mutex> lk(pool_mu_);
            pool_done_cv_.wait(lk, [this] {
                return pool_done_.load(std::memory_order_acquire) >=
                       pool_size_;
            });
            pool_job_ = nullptr;
        }
    }

    void select_default_kernel()
    {
        // Profile-guided default (i7-12700, Phase-3):
        //   H<=48 (realtime taps): AVX2/FMA macroblock wins (~1.4× VOLK).
        //   H>=64 (quality taps): VOLK macroblock wins (AVX2 horizontal/
        //   unpack path loses to VOLK's tuned kernels on longer arms).
#if UWB_RESAMP_HAVE_AVX2
        if (H_ <= 48) {
            fir_kind_ = FirKind::Avx2;
            loop_kind_ = LoopKind::Macroblock;
            kernel_name_ = "avx2_fma_macroblock";
            return;
        }
#endif
        fir_kind_ = FirKind::Volk;
        loop_kind_ = LoopKind::Macroblock;
        kernel_name_ = "volk_macroblock";
    }

    void update_hist_from_work(size_t total_w)
    {
        const size_t hist_n = hist_.size();
        if (hist_n == 0)
            return;
        if (total_w >= hist_n) {
            std::memcpy(hist_.data(),
                        work_.data() + (total_w - hist_n),
                        hist_n * sizeof(gr_complex));
        } else {
            std::fill(hist_.begin(), hist_.end(), gr_complex(0.0f, 0.0f));
            std::memcpy(hist_.data() + (hist_n - total_w),
                        work_.data(),
                        total_w * sizeof(gr_complex));
        }
    }

    static gr_complex fir_scalar(const float* trev,
                                 const gr_complex* win,
                                 size_t H)
    {
        float r0 = 0, r1 = 0, r2 = 0, r3 = 0;
        float i0 = 0, i1 = 0, i2 = 0, i3 = 0;
        size_t q = 0;
        for (; q + 4 <= H; q += 4) {
            r0 += trev[q + 0] * win[q + 0].real();
            i0 += trev[q + 0] * win[q + 0].imag();
            r1 += trev[q + 1] * win[q + 1].real();
            i1 += trev[q + 1] * win[q + 1].imag();
            r2 += trev[q + 2] * win[q + 2].real();
            i2 += trev[q + 2] * win[q + 2].imag();
            r3 += trev[q + 3] * win[q + 3].real();
            i3 += trev[q + 3] * win[q + 3].imag();
        }
        float acc_re = (r0 + r1) + (r2 + r3);
        float acc_im = (i0 + i1) + (i2 + i3);
        for (; q < H; ++q) {
            acc_re += trev[q] * win[q].real();
            acc_im += trev[q] * win[q].imag();
        }
        return gr_complex(acc_re, acc_im);
    }

    static gr_complex fir_volk(const float* trev,
                               const gr_complex* win,
                               size_t H)
    {
        gr_complex acc(0.0f, 0.0f);
        volk_32fc_32f_dot_prod_32fc(
            &acc, win, trev, static_cast<unsigned>(H));
        return acc;
    }

#if UWB_RESAMP_HAVE_AVX2
    static gr_complex fir_avx2(const float* trev,
                               const gr_complex* win,
                               size_t H)
    {
        // Two independent accumulators break the serial FMA chain.
        // Each step processes 4 complex samples (8 floats) with expanded
        // real taps [t0,t0,t1,t1,t2,t2,t3,t3].
        __m256 acc0 = _mm256_setzero_ps();
        __m256 acc1 = _mm256_setzero_ps();
        size_t q = 0;
        const float* xp = reinterpret_cast<const float*>(win);
        for (; q + 8 <= H; q += 8) {
            const __m256 x0 = _mm256_loadu_ps(xp + 2 * q);
            const __m256 x1 = _mm256_loadu_ps(xp + 2 * q + 8);
            const __m128 th0 = _mm_loadu_ps(trev + q);
            const __m128 th1 = _mm_loadu_ps(trev + q + 4);
            const __m256 h0 = _mm256_set_m128(_mm_unpackhi_ps(th0, th0),
                                              _mm_unpacklo_ps(th0, th0));
            const __m256 h1 = _mm256_set_m128(_mm_unpackhi_ps(th1, th1),
                                              _mm_unpacklo_ps(th1, th1));
            acc0 = _mm256_fmadd_ps(x0, h0, acc0);
            acc1 = _mm256_fmadd_ps(x1, h1, acc1);
        }
        for (; q + 4 <= H; q += 4) {
            const __m256 x = _mm256_loadu_ps(xp + 2 * q);
            const __m128 th = _mm_loadu_ps(trev + q);
            const __m256 h = _mm256_set_m128(_mm_unpackhi_ps(th, th),
                                             _mm_unpacklo_ps(th, th));
            acc0 = _mm256_fmadd_ps(x, h, acc0);
        }
        const __m256 acc = _mm256_add_ps(acc0, acc1);
        // Horizontal: even lanes = re, odd = im
        const __m128 lo = _mm256_castps256_ps128(acc);
        const __m128 hi = _mm256_extractf128_ps(acc, 1);
        const __m128 s = _mm_add_ps(lo, hi); // r0,i0,r1,i1
        const __m128 sh = _mm_movehdup_ps(s); // i0,i0,i1,i1
        const __m128 re2 = _mm_add_ps(s, _mm_movehl_ps(s, s)); // r0+r1, ...
        const __m128 im2 = _mm_add_ps(sh, _mm_movehl_ps(sh, sh));
        float re = _mm_cvtss_f32(re2);
        float im = _mm_cvtss_f32(im2);
        for (; q < H; ++q) {
            re += trev[q] * win[q].real();
            im += trev[q] * win[q].imag();
        }
        return gr_complex(re, im);
    }
#endif

    gr_complex filter_window(uint32_t arm_idx,
                             const gr_complex* win_oldest) const
    {
        const float* trev =
            arms_rev_.data() + static_cast<size_t>(arm_idx) * H_;
        switch (fir_kind_) {
        case FirKind::Scalar:
            return fir_scalar(trev, win_oldest, H_);
        case FirKind::Volk:
            return fir_volk(trev, win_oldest, H_);
        case FirKind::Avx2:
#if UWB_RESAMP_HAVE_AVX2
            return fir_avx2(trev, win_oldest, H_);
#else
            return fir_volk(trev, win_oldest, H_);
#endif
        }
        return fir_scalar(trev, win_oldest, H_);
    }

    /** Compute consumed from posts_taken and started_held. */
    static size_t consumed_from_posts(bool started_held,
                                      size_t posts_taken,
                                      size_t ninput,
                                      bool exhausted)
    {
        if (exhausted)
            return ninput;
        if (started_held)
            return (posts_taken > 1) ? (posts_taken - 1) : 0;
        return posts_taken;
    }

    ProcessResult run_legacy(size_t hist_n,
                             size_t post_n,
                             size_t post_i,
                             bool started_held,
                             size_t ninput,
                             size_t total_w,
                             gr_complex* out,
                             size_t max_out)
    {
        const auto t_f0 =
            profile_enabled_ ? Clock::now() : Clock::time_point{};
        size_t produced = 0;
        while (produced < max_out) {
            const size_t win_start = hist_n + post_i + 1 - H_;
            out[produced++] =
                filter_window(ctr_, work_.data() + win_start);
            ++output_items_;
            ctr_ += kDecim;
            while (ctr_ >= kInterp) {
                ctr_ -= kInterp;
                ++post_i;
                if (post_i >= post_n) {
                    if (profile_enabled_)
                        profile_.ns_fir += ns_since(t_f0);
                    const auto t_st =
                        profile_enabled_ ? Clock::now() : Clock::time_point{};
                    update_hist_from_work(total_w);
                    have_current_ = false;
                    if (profile_enabled_)
                        profile_.ns_state += ns_since(t_st);
                    ProcessResult r;
                    r.produced = produced;
                    r.consumed = ninput;
                    return r;
                }
                current_ = work_[hist_n + post_i];
                have_current_ = true;
            }
        }
        if (profile_enabled_)
            profile_.ns_fir += ns_since(t_f0);

        const auto t_st =
            profile_enabled_ ? Clock::now() : Clock::time_point{};
        if (hist_n > 0) {
            const size_t cur = hist_n + post_i;
            std::memcpy(hist_.data(),
                        work_.data() + (cur - hist_n),
                        hist_n * sizeof(gr_complex));
        }
        have_current_ = true;
        current_ = work_[hist_n + post_i];
        if (profile_enabled_)
            profile_.ns_state += ns_since(t_st);

        ProcessResult r;
        r.produced = produced;
        r.consumed = consumed_from_posts(started_held, post_i + 1, ninput,
                                         /*exhausted=*/false);
        return r;
    }

    ProcessResult run_macroblock(size_t hist_n,
                                 size_t post_n,
                                 size_t post_i,
                                 bool started_held,
                                 size_t ninput,
                                 size_t total_w,
                                 gr_complex* out,
                                 size_t max_out)
    {
        size_t produced = 0;

        // Full macroblocks: need 48 advances of headroom after current and
        // 65 free output slots.  After a macroblock, phase returns to start
        // and post_i advances by exactly 48.
        while (produced + kInterp <= max_out &&
               post_i + static_cast<size_t>(kDecim) < post_n) {
            const auto t_s0 =
                profile_enabled_ ? Clock::now() : Clock::time_point{};
            const size_t sched_base =
                static_cast<size_t>(ctr_) * static_cast<size_t>(kInterp);
            const uint8_t* arms = schedule_arm_.data() + sched_base;
            const uint8_t* advs = schedule_adv_.data() + sched_base;
            size_t cur_post = post_i;
            if (profile_enabled_)
                profile_.ns_schedule += ns_since(t_s0);

            const auto t_f0 =
                profile_enabled_ ? Clock::now() : Clock::time_point{};
            gr_complex* dest = out + produced;
            for (uint32_t o = 0; o < kInterp; ++o) {
                const size_t win_start = hist_n + cur_post + 1 - H_;
                dest[o] = filter_window(arms[o], work_.data() + win_start);
                cur_post += advs[o];
            }
            if (profile_enabled_) {
                profile_.ns_fir += ns_since(t_f0);
                profile_.macroblocks += 1;
            }

            produced += kInterp;
            output_items_ += kInterp;
            post_i += kDecim;
            // ctr_ unchanged after full L outputs (L*M ≡ 0 mod L)
            current_ = work_[hist_n + post_i];
            have_current_ = true;
        }

        // Remainder: per-output.
        const auto t_f1 =
            profile_enabled_ ? Clock::now() : Clock::time_point{};
        while (produced < max_out) {
            const size_t win_start = hist_n + post_i + 1 - H_;
            out[produced++] =
                filter_window(ctr_, work_.data() + win_start);
            ++output_items_;
            ctr_ += kDecim;
            while (ctr_ >= kInterp) {
                ctr_ -= kInterp;
                ++post_i;
                if (post_i >= post_n) {
                    if (profile_enabled_)
                        profile_.ns_fir += ns_since(t_f1);
                    const auto t_st =
                        profile_enabled_ ? Clock::now() : Clock::time_point{};
                    update_hist_from_work(total_w);
                    have_current_ = false;
                    if (profile_enabled_)
                        profile_.ns_state += ns_since(t_st);
                    ProcessResult r;
                    r.produced = produced;
                    r.consumed = ninput;
                    return r;
                }
                current_ = work_[hist_n + post_i];
                have_current_ = true;
            }
        }
        if (profile_enabled_)
            profile_.ns_fir += ns_since(t_f1);

        const auto t_st =
            profile_enabled_ ? Clock::now() : Clock::time_point{};
        if (hist_n > 0) {
            const size_t cur = hist_n + post_i;
            std::memcpy(hist_.data(),
                        work_.data() + (cur - hist_n),
                        hist_n * sizeof(gr_complex));
        }
        have_current_ = true;
        current_ = work_[hist_n + post_i];
        if (profile_enabled_)
            profile_.ns_state += ns_since(t_st);

        ProcessResult r;
        r.produced = produced;
        r.consumed = consumed_from_posts(started_held, post_i + 1, ninput,
                                         /*exhausted=*/false);
        return r;
    }

    void fir_parallel(gr_complex* out,
                      size_t n_out,
                      int nworkers)
    {
        const int nw = std::max(1, nworkers);
        if (nw == 1 || n_out < 1024) {
            for (size_t i = 0; i < n_out; ++i) {
                out[i] = filter_window(sched_arm_buf_[i],
                                       work_.data() + sched_win_buf_[i]);
            }
            return;
        }
        // Ensure pool matches requested worker count.
        if (pool_size_ != nw)
            start_pool(nw);
        pool_parallel_for(n_out, [this, out](size_t lo, size_t hi) {
            for (size_t i = lo; i < hi; ++i) {
                out[i] = filter_window(sched_arm_buf_[i],
                                       work_.data() + sched_win_buf_[i]);
            }
        });
    }

    ProcessResult run_mt(size_t hist_n,
                         size_t post_n,
                         size_t post_i,
                         bool started_held,
                         size_t ninput,
                         size_t total_w,
                         gr_complex* out,
                         size_t max_out,
                         int nworkers)
    {
        // Process in sub-batches bounded by schedule scratch capacity so large
        // grants never truncate mid-call.
        size_t produced_total = 0;
        bool exhausted = false;
        const size_t scratch = sched_arm_buf_.size();

        while (produced_total < max_out && !exhausted) {
            const auto t_s0 =
                profile_enabled_ ? Clock::now() : Clock::time_point{};

            size_t n_out = 0;
            const size_t batch_cap =
                std::min(scratch, max_out - produced_total);
            while (n_out < batch_cap) {
                sched_arm_buf_[n_out] = static_cast<uint8_t>(ctr_);
                sched_win_buf_[n_out] =
                    static_cast<uint32_t>(hist_n + post_i + 1 - H_);
                ++n_out;
                ctr_ += kDecim;
                while (ctr_ >= kInterp) {
                    ctr_ -= kInterp;
                    ++post_i;
                    if (post_i >= post_n) {
                        exhausted = true;
                        break;
                    }
                }
                if (exhausted)
                    break;
            }
            if (profile_enabled_)
                profile_.ns_schedule += ns_since(t_s0);

            if (n_out == 0)
                break;

            const auto t_f0 =
                profile_enabled_ ? Clock::now() : Clock::time_point{};
            fir_parallel(out + produced_total, n_out, nworkers);
            if (profile_enabled_)
                profile_.ns_fir += ns_since(t_f0);

            produced_total += n_out;
            output_items_ += n_out;
        }

        const auto t_st =
            profile_enabled_ ? Clock::now() : Clock::time_point{};
        size_t consumed;
        if (exhausted) {
            update_hist_from_work(total_w);
            have_current_ = false;
            consumed = ninput;
        } else {
            if (hist_n > 0) {
                const size_t cur = hist_n + post_i;
                std::memcpy(hist_.data(),
                            work_.data() + (cur - hist_n),
                            hist_n * sizeof(gr_complex));
            }
            have_current_ = true;
            current_ = work_[hist_n + post_i];
            consumed = consumed_from_posts(started_held, post_i + 1, ninput,
                                           /*exhausted=*/false);
        }
        if (profile_enabled_)
            profile_.ns_state += ns_since(t_st);

        ProcessResult r;
        r.produced = produced_total;
        r.consumed = consumed;
        return r;
    }
};

} // namespace core
} // namespace uwb
} // namespace gr
