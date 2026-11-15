/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Pure rational resampler core (no GNU Radio / PMT deps).
 * Instantiated as 65/48 (737.28→998.4) and 65/32 (491.52→998.4).
 *
 * Contract: docs/performance/规格_固定65_48重采样core契约.md
 *           docs/phase1/开发方案_CG400_491p52雷达接收与65_32重采样.md
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
 * Round-2 kernel candidates (2026-11-11, SC16 macroblock path only, EXPLICIT
 * switches, both default OFF, both bit-identical to the default path):
 *
 *   set_fixed_phase_schedule(true)  — C1.  Replaces the serial
 *       "cur_post += advs[o]" chain of the macroblock loop with a precomputed
 *       per-starting-phase descriptor table (per 4-output group: the 4
 *       polyphase arm tap offsets, the 4 window offsets RELATIVE to the
 *       macroblock base window, and the win[j] == win[j-1] reuse mask), and
 *       executes it with 4-way FIR kernels that are compile-time specialised
 *       for H_pad == 24 and H_pad == 48 (the two shipped profiles) with a
 *       generic runtime-bound fallback for any other H_pad.
 *
 * The C2 "convert_cache" candidate was measured and NOT retained
 * (2026-11-11 round 2: its sign flips with profile/window size, -7.6% ..
 * +7.6% in core screening), so it was REMOVED in the 2026-11-15 native
 * cleanup; only the C1 fixed_phase schedule remains as an explicit switch.
 *
 * The remaining candidate is not reachable when fir_kind_ != Avx2, nor in
 * the FC32 process() path, nor in the multi-worker run_mt_sc16() schedule:
 * those keep the pre-existing code.  It never allocates in process()/flush().
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
#include <cstdlib>
#include <cstring>
#include <functional>
#include <iostream>
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

template <uint32_t Interp, uint32_t Decim>
class RationalResamplerLmCore
{
public:
    static constexpr uint32_t kInterp = Interp;
    static constexpr uint32_t kDecim = Decim;
    static_assert(Interp >= 1 && Decim >= 1, "interp/decim must be >= 1");
    static_assert(Interp <= 255, "interp must fit schedule_arm_ uint8_t");

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

    RationalResamplerLmCore(const float* taps, size_t T)
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
                "RationalResamplerLmCore: taps must be non-empty");
        }
        if (H_ == 0) {
            throw std::invalid_argument(
                "RationalResamplerLmCore: invalid arm length");
        }

        H_pad_ = (H_ + 7) & ~static_cast<size_t>(7);
        arms_.assign(static_cast<size_t>(kInterp) * H_, 0.0f);
        arms_rev_.assign(static_cast<size_t>(kInterp) * H_, 0.0f);
        arms_rev_pad_.assign(static_cast<size_t>(kInterp) * H_pad_, 0.0f);
        arms_rev_dup_.assign(static_cast<size_t>(kInterp) * (2 * H_pad_), 0.0f);
        for (size_t i = 0; i < T; ++i) {
            const size_t arm = i % kInterp;
            const size_t k = i / kInterp;
            arms_[arm * H_ + k] = taps[i];
        }
        for (size_t a = 0; a < kInterp; ++a) {
            for (size_t k = 0; k < H_; ++k) {
                const float val = arms_[a * H_ + k];
                arms_rev_[a * H_ + (H_ - 1 - k)] = val;
                arms_rev_pad_[a * H_pad_ + (H_ - 1 - k)] = val;
            }
            for (size_t k = 0; k < H_pad_; ++k) {
                const float val = (k < H_) ? arms_rev_[a * H_ + k] : 0.0f;
                arms_rev_dup_[a * (2 * H_pad_) + 2 * k + 0] = val;
                arms_rev_dup_[a * (2 * H_pad_) + 2 * k + 1] = val;
            }
        }

        arms_rev_dup_scaled_.assign(static_cast<size_t>(kInterp) * (2 * H_pad_), 0.0f);
        last_scale_ = 0.0f;

        hist_.assign(H_ > 0 ? H_ - 1 : 0, gr_complex(0.0f, 0.0f));
        hist_sc16_.assign(H_ > 0 ? H_ - 1 : 0, std::complex<int16_t>(0, 0));
        current_sc16_ = std::complex<int16_t>(0, 0);

        work_.reserve(H_ + (1u << 20));
        work_.resize(H_);
        work_sc16_.reserve(H_ + (1u << 20));
        work_sc16_.resize(H_);

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

        // 2026-11-11 round 2 (A1): the two multi-worker schedule tables are
        // NOT sized here.  They are read only by fir_parallel()/
        // fir_parallel_sc16(), reachable only from run_mt()/run_mt_sc16(),
        // i.e. only when the effective worker count is > 1.  Burst workers
        // force set_num_workers(1), so the old unconditional
        //     sched_arm_buf_.resize(1 << 20);   // 1 MiB
        //     sched_win_buf_.resize(1 << 20);   // 4 MiB
        // made every single-core instance pay 5 MiB of zero-initialisation it
        // never used.  ensure_schedule_scratch() now sizes them on the first
        // true MT call and reuses that capacity afterwards.

        // Round-2 kernel candidates: pick the compile-time H_pad
        // specialisation ONCE.  H_pad_ is fixed for the lifetime of the
        // object, so switching a candidate on later never re-dispatches; the
        // two shipped profiles (realtime_minorder H_pad = 24,
        // quality_minorder H_pad = 48) get fully unrolled tap loops.
        hp_class_ = (H_pad_ == 24) ? 24 : (H_pad_ == 48) ? 48 : 0;

        select_default_kernel();
    }

    explicit RationalResamplerLmCore(const std::vector<float>& taps)
        : RationalResamplerLmCore(taps.data(), taps.size())
    {
    }

    ~RationalResamplerLmCore()
    {
        stop_pool();
    }

    RationalResamplerLmCore(const RationalResamplerLmCore&) = delete;
    RationalResamplerLmCore&
    operator=(const RationalResamplerLmCore&) = delete;

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
                "RationalResamplerLmCore::set_kernel: unknown " + name);
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

    // =====================================================================
    // Round-2 kernel candidate C1 (explicit switch, default OFF; C2 removed).
    //
    // Both are experiments on HOW the 65 outputs of a full macroblock are
    // computed, never on WHAT is computed: the per-output arithmetic stays
    // "two 8-lane accumulators, one FMA per 8-float tap vector, acc_lo +
    // acc_hi, then the same hadd/hadd chain" over the same pre-scaled taps
    // (arms_rev_dup_scaled_), in the same order.  That is why every variant
    // must be BIT-identical to the default path, and the QA gate
    // (lib/qa_uwb_resampler_burst_opt.cc) enforces it sample by sample.
    // =====================================================================

    /**
     * C1 descriptor: one entry per 4 consecutive outputs of a full
     * macroblock, for ONE starting phase p == ctr_ in [0, kInterp).
     *
     *   win[j]  start of output j's FIR window RELATIVE to the macroblock
     *           base window  base = work_sc16_ + (hist_n + post_i + 1 - H_),
     *           in complex samples.  Always 0 <= win[0] <= win[1] <= win[2]
     *           <= win[3] <= kDecim: the group advances the input cursor by
     *           advs[o..o+3] <= kDecim in total.
     *   tap[j]  offset of output j's pre-scaled tap vector inside
     *           arms_rev_dup_scaled_, in FLOAT elements
     *           (== polyphase arm index * 2 * H_pad_).
     *   reuse   precomputed win[j] == win[j-1] relation, so the kernel never
     *           compares pointers at run time:
     *             bit 1 (value 2) : win[1] == win[0]
     *             bit 2 (value 4) : win[2] == win[1]
     *             bit 3 (value 8) : win[3] == win[2]
     *           bit 0 is unused and always 0.
     *
     * The offsets are NOT an accumulate chain: each descriptor is an absolute
     * offset from the macroblock base, so the hot loop has no serial
     * "cur_post += advs[o]" dependency.
     */
    struct FixedPhaseGroup {
        uint16_t win[4];
        uint16_t tap[4];
        uint8_t reuse;
        uint8_t pad[3];
    };

    /** Leftover outputs of a macroblock (kInterp % 4: 1 for 65, 0 for 48). */
    struct FixedPhaseTail {
        uint16_t win;
        uint16_t tap;
    };

    static constexpr uint32_t kMacroblockGroups = kInterp / 4;
    static constexpr uint32_t kMacroblockTail = kInterp % 4;
    static constexpr uint32_t kTailStride =
        (kMacroblockTail != 0) ? kMacroblockTail : 1;

    /** 4-output groups / leftover outputs in one full macroblock. */
    static constexpr uint32_t macroblock_groups() { return kMacroblockGroups; }
    static constexpr uint32_t macroblock_tail() { return kMacroblockTail; }

    void set_fixed_phase_schedule(bool on)
    {
        if (on)
            build_fixed_phase_schedule();
        fixed_phase_ = on;
    }

    bool fixed_phase_schedule() const { return fixed_phase_; }

    /** C1 table accessors (QA verifies every phase against a local model). */
    bool fixed_phase_table_built() const { return !fp_groups_.empty(); }

    const FixedPhaseGroup& fixed_phase_group(uint32_t phase, uint32_t g) const
    {
        return fp_groups_.at(static_cast<size_t>(phase) * kMacroblockGroups + g);
    }

    const FixedPhaseTail& fixed_phase_tail(uint32_t phase, uint32_t t) const
    {
        return fp_tail_.at(static_cast<size_t>(phase) * kTailStride + t);
    }

    /**
     * Which SC16 macroblock kernel process_sc16() will actually use.  The
     * candidates are inert unless the AVX2/FMA default kernel is selected, so
     * a build without AVX2 reports "default" even with a switch on.
     */
    const char* sc16_kernel_variant() const
    {
#if UWB_RESAMP_HAVE_AVX2
        if (fir_kind_ == FirKind::Avx2 && fixed_phase_)
            return "c1_fixed_phase";
#endif
        return "default";
    }

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
                "RationalResamplerLmCore: process() after flush without "
                "reset()");
        }

        is_sc16_mode_ = false;

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
        const size_t need = hist_n + post_n + H_pad_ + 16;
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
        std::fill(work_.data() + wpos, work_.data() + wpos + H_pad_ + 16, gr_complex(0.0f, 0.0f));
        const size_t total_w = wpos;
        if (profile_enabled_)
            profile_.ns_assemble += ns_since(t_a0);

        size_t post_i = 0;
        // M2-A G0 §3.4 chunk-invariance fix.  Compiled ONLY for Decim > Interp
        // (the new TX work->native directions), so the existing RX directions
        // (65/48, 65/32) are byte-for-byte unchanged and pay no extra work.
        //
        // If the previous process() call exhausted in the middle of a phase
        // wrap, the logical input position floor(output_items_*M/L) is ahead
        // of the consumed count.  Resume the local cursor at that offset so
        // subsequent chunks continue the same upfirdn stream, and restore the
        // exact polyphase arm for the next output.
        if constexpr (kDecim > kInterp) {
            if (!started_held) {
                const int64_t logical_post =
                    (static_cast<int64_t>(output_items_) *
                     static_cast<int64_t>(kDecim)) /
                    static_cast<int64_t>(kInterp);
                const int64_t start_post =
                    logical_post - static_cast<int64_t>(in_count_entry);
                if (start_post > 0)
                    post_i = static_cast<size_t>(start_post);
                ctr_ = static_cast<uint32_t>(
                    (static_cast<uint64_t>(output_items_) *
                     static_cast<uint64_t>(kDecim)) %
                    static_cast<uint64_t>(kInterp));
            }
        }
        have_current_ = true;
        current_ = (post_i < post_n) ? work_[hist_n + post_i]
                                     : gr_complex(0.0f, 0.0f);

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

    ProcessResult process_sc16(const std::complex<int16_t>* in_sc16,
                               size_t ninput,
                               float scale,
                               gr_complex* out,
                               size_t max_out,
                               int nworkers_override = 0)
    {
        const auto t_all0 =
            profile_enabled_ ? Clock::now() : Clock::time_point{};

        if (flush_mode_) {
            throw std::logic_error(
                "RationalResamplerLmCore: process_sc16() after flush without "
                "reset()");
        }

        is_sc16_mode_ = true;
        ensure_scaled_taps(scale);

        const size_t hist_n = hist_sc16_.size(); // H-1
        const bool started_held = have_current_;
        const size_t held = started_held ? 1u : 0u;
        const size_t post_n = held + ninput;
        if (post_n == 0 || max_out == 0) {
            return ProcessResult{};
        }

        const uint64_t in_count_entry = input_items_;

        // ---- (a) work buffer assembly ----
        const auto t_a0 =
            profile_enabled_ ? Clock::now() : Clock::time_point{};
        const size_t need = hist_n + post_n + H_pad_ + 16;
        if (work_sc16_.capacity() < need)
            work_sc16_.reserve(need);
        if (work_sc16_.size() < need)
            work_sc16_.resize(need);

        if (hist_n > 0) {
            std::memcpy(work_sc16_.data(), hist_sc16_.data(),
                        hist_n * sizeof(std::complex<int16_t>));
        }
        size_t wpos = hist_n;
        if (started_held)
            work_sc16_[wpos++] = current_sc16_;

        if (ninput > 0 && in_sc16 != nullptr) {
            std::memcpy(work_sc16_.data() + wpos, in_sc16,
                        ninput * sizeof(std::complex<int16_t>));
            wpos += ninput;
        }

        std::fill(work_sc16_.data() + wpos, work_sc16_.data() + wpos + H_pad_ + 16,
                  std::complex<int16_t>(0, 0));
        const size_t total_w = wpos;
        if (profile_enabled_)
            profile_.ns_assemble += ns_since(t_a0);

        size_t post_i = 0;
        if constexpr (kDecim > kInterp) {
            if (!started_held) {
                const int64_t logical_post =
                    (static_cast<int64_t>(output_items_) *
                     static_cast<int64_t>(kDecim)) /
                    static_cast<int64_t>(kInterp);
                const int64_t start_post =
                    logical_post - static_cast<int64_t>(in_count_entry);
                if (start_post > 0)
                    post_i = static_cast<size_t>(start_post);
                ctr_ = static_cast<uint32_t>(
                    (static_cast<uint64_t>(output_items_) *
                     static_cast<uint64_t>(kDecim)) %
                    static_cast<uint64_t>(kInterp));
            }
        }
        have_current_ = true;
        current_sc16_ = (post_i < post_n) ? work_sc16_[hist_n + post_i]
                                           : std::complex<int16_t>(0, 0);

        const int nw =
            (nworkers_override > 0) ? nworkers_override : nworkers_;

        ProcessResult r;
        if (loop_kind_ == LoopKind::Legacy) {
            r = run_legacy_sc16(hist_n, post_n, post_i, started_held, ninput,
                                total_w, out, max_out);
        } else if (nw > 1 && max_out >= 64 && post_n >= 64) {
            r = run_mt_sc16(hist_n, post_n, post_i, started_held, ninput,
                            total_w, out, max_out, nw);
        } else {
            r = run_macroblock_sc16(hist_n, post_n, post_i, started_held, ninput,
                                    total_w, out, max_out);
        }

        input_items_ = in_count_entry + r.consumed;

        if (profile_enabled_) {
            profile_.calls += 1;
            profile_.outputs += r.produced;
            profile_.ns_total += ns_since(t_all0);
        }
        return r;
    }

    size_t process_sc16(const std::complex<int16_t>* in_sc16,
                        size_t ninput,
                        float scale,
                        gr_complex* out)
    {
        return process_sc16(in_sc16, ninput, scale, out, SIZE_MAX / 4).produced;
    }

    size_t flush(gr_complex* out, size_t max_out)
    {
        [[maybe_unused]] const bool first_flush = !flush_mode_;
        flush_mode_ = true;
        const uint64_t N = input_items_;
        const size_t Lout = expected_output_length(N, T_);
        size_t produced = 0;

        if (is_sc16_mode_) {
            return flush_sc16_internal(out, max_out, first_flush, N, Lout);
        }

        const size_t hist_n = hist_.size();

        // M2-A G0 §3.4 flush fix.  Compiled ONLY for Decim > Interp, so the
        // existing RX directions are byte-for-byte unchanged.
        //
        // process() can return from its exhaustion branch in the MIDDLE of the
        // phase-wrap loop, so ctr_ may still be >= Interp and the logical input
        // pointer floor(output_items_*M/L) may exceed the consumed count N.
        // Restore the exact upfirdn phase and shift the history by that
        // overshoot (shifted-in samples are implicit zero padding) so flush
        // continues the same stream.
        if constexpr (kDecim > kInterp) {
        if (first_flush && hist_n > 0) {
            ctr_ = static_cast<uint32_t>(
                (static_cast<uint64_t>(output_items_) *
                 static_cast<uint64_t>(kDecim)) %
                static_cast<uint64_t>(kInterp));
            const int64_t logical_post =
                (static_cast<int64_t>(output_items_) *
                 static_cast<int64_t>(kDecim)) /
                static_cast<int64_t>(kInterp);
            const int64_t overshoot =
                logical_post - static_cast<int64_t>(N);
            if (overshoot > 0) {
                const size_t sh = static_cast<size_t>(
                    std::min<int64_t>(overshoot,
                                      static_cast<int64_t>(hist_n)));
                std::memmove(hist_.data(), hist_.data() + sh,
                             (hist_n - sh) * sizeof(gr_complex));
                std::fill(hist_.data() + (hist_n - sh),
                          hist_.data() + hist_n, gr_complex(0.0f, 0.0f));
            }
        }
        } // if constexpr (kDecim > kInterp)

        const size_t rem_out = (output_items_ < Lout) ? (Lout - output_items_) : 0;
        const size_t zeros_needed = (rem_out * kDecim) / kInterp + H_ + 32;

        const size_t held = have_current_ ? 1u : 0u;
        const size_t post_n = held + zeros_needed;
        const size_t need = hist_n + post_n + H_pad_ + 16;
        if (work_.capacity() < need)
            work_.reserve(need);
        if (work_.size() < need)
            work_.resize(need);

        if (hist_n > 0) {
            std::memcpy(work_.data(), hist_.data(),
                        hist_n * sizeof(gr_complex));
        }
        size_t wpos = hist_n;
        if (have_current_) {
            work_[wpos++] = current_;
        }
        std::fill(work_.data() + wpos, work_.data() + need, gr_complex(0.0f, 0.0f));

        size_t post_i = 0;
        while (produced < max_out && output_items_ < Lout) {
            const size_t win_start = hist_n + post_i + 1 - H_;
            out[produced++] = filter_window(ctr_, work_.data() + win_start);
            ++output_items_;
            ctr_ += kDecim;
            while (ctr_ >= kInterp) {
                ctr_ -= kInterp;
                ++post_i;
            }
        }
        if (hist_n > 0) {
            const size_t cur = hist_n + post_i;
            std::memcpy(hist_.data(), work_.data() + (cur - hist_n),
                        hist_n * sizeof(gr_complex));
        }
        have_current_ = false;
        current_ = gr_complex(0.0f, 0.0f);
        return produced;
    }

    size_t flush_sc16(gr_complex* out, size_t max_out)
    {
        is_sc16_mode_ = true;
        return flush(out, max_out);
    }

    bool flush_complete() const
    {
        return flush_mode_ &&
               output_items_ >= expected_output_length(input_items_, T_);
    }

    void reset()
    {
        std::fill(hist_.begin(), hist_.end(), gr_complex(0.0f, 0.0f));
        std::fill(hist_sc16_.begin(), hist_sc16_.end(), std::complex<int16_t>(0, 0));
        ctr_ = 0;
        have_current_ = false;
        current_ = gr_complex(0.0f, 0.0f);
        current_sc16_ = std::complex<int16_t>(0, 0);
        is_sc16_mode_ = false;
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
    size_t arm_length_pad() const { return H_pad_; }
    size_t history_extra() const { return H_ > 0 ? H_ - 1 : 0; }
    const char* kernel_name() const { return kernel_name_; }

    // --- lazy multi-worker schedule-table diagnostics (2026-11-11 round 2) ---
    //
    // Names are frozen by
    // docs/performance/接口冻结_UHD_SC16第二轮优化_2026-11-11.md and used by the
    // verification agent, so do not rename them.
    //
    //   schedule_scratch_entries()   entries in ONE table (they are always
    //                                sized together); 0 means "never sized",
    //                                which is what a single-core instance
    //                                must report.
    //   schedule_scratch_bytes()     bytes held by BOTH tables
    //                                (1 B uint8_t arm + 4 B uint32_t window
    //                                 per entry).
    //   schedule_scratch_allocated() entries != 0.
    size_t schedule_scratch_entries() const { return sched_arm_buf_.size(); }

    size_t schedule_scratch_bytes() const
    {
        return sched_arm_buf_.size() * sizeof(uint8_t) +
               sched_win_buf_.size() * sizeof(uint32_t);
    }

    bool schedule_scratch_allocated() const { return !sched_arm_buf_.empty(); }

    const std::vector<float>& arms_rev_pad() const { return arms_rev_pad_; }
    const std::vector<float>& arms_rev_dup() const { return arms_rev_dup_; }
    const std::vector<float>& arms_rev_dup_scaled() const { return arms_rev_dup_scaled_; }
    float last_scale() const { return last_scale_; }
    const std::vector<std::complex<int16_t>>& hist_sc16() const { return hist_sc16_; }
    std::complex<int16_t> current_sc16() const { return current_sc16_; }
    bool is_sc16_mode() const { return is_sc16_mode_; }

    void ensure_scaled_taps(float scale)
    {
        if (scale == last_scale_) {
            return;
        }
        const size_t sz = arms_rev_dup_.size();
        if (arms_rev_dup_scaled_.size() != sz) {
            arms_rev_dup_scaled_.resize(sz);
        }
        if (scale == 1.0f) {
            std::memcpy(arms_rev_dup_scaled_.data(), arms_rev_dup_.data(),
                        sz * sizeof(float));
        } else {
#if UWB_RESAMP_HAVE_AVX2
            const __m256 v_scale = _mm256_set1_ps(scale);
            size_t i = 0;
            for (; i + 8 <= sz; i += 8) {
                __m256 val = _mm256_loadu_ps(arms_rev_dup_.data() + i);
                _mm256_storeu_ps(arms_rev_dup_scaled_.data() + i,
                                 _mm256_mul_ps(val, v_scale));
            }
            for (; i < sz; ++i) {
                arms_rev_dup_scaled_[i] = arms_rev_dup_[i] * scale;
            }
#else
            for (size_t i = 0; i < sz; ++i) {
                arms_rev_dup_scaled_[i] = arms_rev_dup_[i] * scale;
            }
#endif
        }
        last_scale_ = scale;
    }

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

    // Lazy MT schedule-scratch geometry (2026-11-11 round 2, A1).
    //
    // Fixed (not derived from max_out) so that the capacity — and therefore
    // schedule_scratch_entries() — is deterministic and stable for the whole
    // lifetime of the instance: the first MT call sizes it, every later call
    // reuses exactly the same buffers.  64 Ki entries bounds one MT sub-batch
    // to 64 Ki outputs; run_mt()/run_mt_sc16() simply loop again, so this only
    // changes the granularity of pool dispatches, never the arithmetic.
    static constexpr size_t kScheduleScratchEntries = 1u << 16;

    /**
     * Size the two MT schedule tables on first use.
     *
     * Grow-only and idempotent: once sched_arm_buf_ holds
     * kScheduleScratchEntries entries this is a single size() comparison, so
     * repeated MT calls never reallocate or re-zero the tables.  Called at the
     * top of run_mt()/run_mt_sc16() BEFORE they derive their batch bound from
     * sched_arm_buf_.size(), which is why the bound can never be 0 (a zero
     * bound would spin the batch loop with zero progress).
     */
    void ensure_schedule_scratch()
    {
        if (sched_arm_buf_.size() >= kScheduleScratchEntries) {
            return;
        }
        // The two tables are indexed together: entry i of sched_arm_buf_ pairs
        // with entry i of sched_win_buf_.  Size both, always.
        sched_arm_buf_.resize(kScheduleScratchEntries);
        sched_win_buf_.resize(kScheduleScratchEntries);
    }

    size_t T_;
    size_t H_;
    size_t H_pad_;
    std::vector<float> arms_;
    std::vector<float> arms_rev_;
    std::vector<float> arms_rev_pad_;
    std::vector<float> arms_rev_dup_;
    std::vector<float> arms_rev_dup_scaled_;
    float last_scale_ = 0.0f;
    std::vector<gr_complex> hist_;
    std::vector<gr_complex> work_;
    std::vector<std::complex<int16_t>> hist_sc16_;
    std::vector<std::complex<int16_t>> work_sc16_;
    std::complex<int16_t> current_sc16_{0, 0};
    bool is_sc16_mode_ = false;
    std::vector<uint8_t> schedule_arm_;
    std::vector<uint8_t> schedule_adv_;
    std::vector<uint8_t> sched_arm_buf_;
    std::vector<uint32_t> sched_win_buf_;

    // --- round-2 kernel candidate C1 (default OFF; C2 removed) ---
    //
    // hp_class_ is the compile-time H_pad specialisation, chosen ONCE in the
    // constructor (H_pad_ never changes for an instance): 24 / 48 for the two
    // shipped profiles, 0 => generic runtime-bound fallback.
    uint8_t hp_class_ = 0;
    bool fixed_phase_ = false;
    // C1: kInterp * kMacroblockGroups descriptors + kInterp tails.
    std::vector<FixedPhaseGroup> fp_groups_;
    std::vector<FixedPhaseTail> fp_tail_;
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
#if UWB_RESAMP_HAVE_AVX2
        fir_kind_ = FirKind::Avx2;
        loop_kind_ = LoopKind::Macroblock;
        kernel_name_ = "avx2_fma_macroblock";
        return;
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

    void update_hist_from_work_sc16(size_t total_w)
    {
        const size_t hist_n = hist_sc16_.size();
        if (hist_n == 0)
            return;
        if (total_w >= hist_n) {
            std::memcpy(hist_sc16_.data(),
                        work_sc16_.data() + (total_w - hist_n),
                        hist_n * sizeof(std::complex<int16_t>));
        } else {
            std::fill(hist_sc16_.begin(), hist_sc16_.end(),
                      std::complex<int16_t>(0, 0));
            std::memcpy(hist_sc16_.data() + (hist_n - total_w),
                        work_sc16_.data(),
                        total_w * sizeof(std::complex<int16_t>));
        }
    }

    // ---------------------------------------------------------------------
    // C1: fixed-phase descriptor table
    // ---------------------------------------------------------------------

    /**
     * Precompute the per-starting-phase 4-output descriptors.
     *
     * Idempotent and grow-once: set_fixed_phase_schedule(true) may be called
     * any number of times; only the first call builds.  The table is a pure
     * function of (kInterp, kDecim, H_pad_), all fixed at construction, so it
     * survives reset() and any number of process/flush cycles.
     *
     * For starting phase p the upfirdn recurrence is
     *     arm[o] = (p + o*kDecim) mod kInterp
     *     win[o] = floor((p + o*kDecim) / kInterp)      (0 at o = 0)
     * i.e. exactly the same values the default schedule tables
     * (schedule_arm_ / schedule_adv_) walk one add at a time; here they are
     * materialised as absolute offsets so the hot loop has no serial
     * dependency.  After all kInterp outputs the cursor has advanced by
     * exactly kDecim and the phase is p again.
     */
    void build_fixed_phase_schedule()
    {
        if (!fp_groups_.empty())
            return;
        // uint16 descriptor fields: the largest tap offset is
        // (kInterp - 1) * 2 * H_pad and the largest window offset is kDecim.
        // No shipped profile comes close (H_pad <= 64, Decim <= 65); refuse
        // loudly instead of silently truncating.
        if (static_cast<size_t>(kInterp - 1) * 2 * H_pad_ > 65535u ||
            kDecim > 65535u) {
            throw std::logic_error(
                "RationalResamplerLmCore: C1 descriptor does not fit uint16");
        }
        const size_t h_stride = 2 * H_pad_; // float elements per tap vector
        fp_groups_.assign(static_cast<size_t>(kInterp) * kMacroblockGroups,
                          FixedPhaseGroup{});
        fp_tail_.assign(static_cast<size_t>(kInterp) * kTailStride,
                        FixedPhaseTail{});

        for (uint32_t p = 0; p < kInterp; ++p) {
            uint32_t arm = p;
            uint32_t win = 0; // absolute offset from the macroblock base
            FixedPhaseGroup* groups =
                fp_groups_.data() + static_cast<size_t>(p) * kMacroblockGroups;
            for (uint32_t g = 0; g < kMacroblockGroups; ++g) {
                FixedPhaseGroup& d = groups[g];
                for (uint32_t j = 0; j < 4; ++j) {
                    d.win[j] = static_cast<uint16_t>(win);
                    d.tap[j] = static_cast<uint16_t>(
                        static_cast<size_t>(arm) * h_stride);
                    arm += kDecim;
                    while (arm >= kInterp) {
                        arm -= kInterp;
                        ++win;
                    }
                }
                d.reuse = 0;
                if (d.win[1] == d.win[0])
                    d.reuse |= 2;
                if (d.win[2] == d.win[1])
                    d.reuse |= 4;
                if (d.win[3] == d.win[2])
                    d.reuse |= 8;
                d.pad[0] = d.pad[1] = d.pad[2] = 0;
            }
            FixedPhaseTail* tails =
                fp_tail_.data() + static_cast<size_t>(p) * kTailStride;
            for (uint32_t t = 0; t < kMacroblockTail; ++t) {
                tails[t].win = static_cast<uint16_t>(win);
                tails[t].tap = static_cast<uint16_t>(
                    static_cast<size_t>(arm) * h_stride);
                arm += kDecim;
                while (arm >= kInterp) {
                    arm -= kInterp;
                    ++win;
                }
            }
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

    static inline gr_complex fir_scalar_sc16(const float* trev_dup_scaled,
                                             const std::complex<int16_t>* win,
                                             size_t H) noexcept
    {
        float r0 = 0, r1 = 0, r2 = 0, r3 = 0;
        float i0 = 0, i1 = 0, i2 = 0, i3 = 0;
        size_t q = 0;
        for (; q + 4 <= H; q += 4) {
            const float h0 = trev_dup_scaled[2 * (q + 0)];
            const float h1 = trev_dup_scaled[2 * (q + 1)];
            const float h2 = trev_dup_scaled[2 * (q + 2)];
            const float h3 = trev_dup_scaled[2 * (q + 3)];
            r0 += h0 * static_cast<float>(win[q + 0].real());
            i0 += h0 * static_cast<float>(win[q + 0].imag());
            r1 += h1 * static_cast<float>(win[q + 1].real());
            i1 += h1 * static_cast<float>(win[q + 1].imag());
            r2 += h2 * static_cast<float>(win[q + 2].real());
            i2 += h2 * static_cast<float>(win[q + 2].imag());
            r3 += h3 * static_cast<float>(win[q + 3].real());
            i3 += h3 * static_cast<float>(win[q + 3].imag());
        }
        float acc_re = (r0 + r1) + (r2 + r3);
        float acc_im = (i0 + i1) + (i2 + i3);
        for (; q < H; ++q) {
            const float h = trev_dup_scaled[2 * q];
            acc_re += h * static_cast<float>(win[q].real());
            acc_im += h * static_cast<float>(win[q].imag());
        }
        return gr_complex(acc_re, acc_im);
    }

    static gr_complex fir_volk(const float* trev,
                               const gr_complex* win,
                               size_t H)
    {
        return fir_scalar(trev, win, H);
    }

#if UWB_RESAMP_HAVE_AVX2
    static inline __m256 load_sc16_x4_as_ps(const std::complex<int16_t>* p) noexcept
    {
        const __m128i v16 = _mm_loadu_si128(reinterpret_cast<const __m128i*>(p));
        const __m256i v32 = _mm256_cvtepi16_epi32(v16);
        return _mm256_cvtepi32_ps(v32);
    }

    static gr_complex fir_avx2(const float* trev_dup,
                               const gr_complex* win,
                               size_t H_pad)
    {
        __m256 acc0 = _mm256_setzero_ps();
        __m256 acc1 = _mm256_setzero_ps();
        const float* xp = reinterpret_cast<const float*>(win);
        for (size_t q = 0; q < H_pad; q += 8) {
            const __m256 x0 = _mm256_loadu_ps(xp + 2 * q);
            const __m256 x1 = _mm256_loadu_ps(xp + 2 * q + 8);
            const __m256 h0 = _mm256_loadu_ps(trev_dup + 2 * q);
            const __m256 h1 = _mm256_loadu_ps(trev_dup + 2 * q + 8);
            acc0 = _mm256_fmadd_ps(x0, h0, acc0);
            acc1 = _mm256_fmadd_ps(x1, h1, acc1);
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
        return gr_complex(re, im);
    }

    static inline gr_complex fir_avx2_sc16(const float* trev_dup_scaled,
                                           const std::complex<int16_t>* win,
                                           size_t H_pad) noexcept
    {
        __m256 acc0 = _mm256_setzero_ps();
        __m256 acc1 = _mm256_setzero_ps();
        for (size_t q = 0; q < H_pad; q += 8) {
            const __m256 x0 = load_sc16_x4_as_ps(win + q);
            const __m256 x1 = load_sc16_x4_as_ps(win + q + 4);
            const __m256 h0 = _mm256_loadu_ps(trev_dup_scaled + 2 * q);
            const __m256 h1 = _mm256_loadu_ps(trev_dup_scaled + 2 * q + 8);
            acc0 = _mm256_fmadd_ps(x0, h0, acc0);
            acc1 = _mm256_fmadd_ps(x1, h1, acc1);
        }
        const __m256 acc = _mm256_add_ps(acc0, acc1);
        const __m128 lo = _mm256_castps256_ps128(acc);
        const __m128 hi = _mm256_extractf128_ps(acc, 1);
        const __m128 s = _mm_add_ps(lo, hi);
        const __m128 sh = _mm_movehdup_ps(s);
        const __m128 re2 = _mm_add_ps(s, _mm_movehl_ps(s, s));
        const __m128 im2 = _mm_add_ps(sh, _mm_movehl_ps(sh, sh));
        float re = _mm_cvtss_f32(re2);
        float im = _mm_cvtss_f32(im2);
        return gr_complex(re, im);
    }

    static inline void fir_avx2_sc16_4x(const float* h_base,
                                        size_t h_stride,
                                        uint8_t arm0, const std::complex<int16_t>* win0,
                                        uint8_t arm1, const std::complex<int16_t>* win1,
                                        uint8_t arm2, const std::complex<int16_t>* win2,
                                        uint8_t arm3, const std::complex<int16_t>* win3,
                                        size_t H_pad,
                                        gr_complex* dest4) noexcept
    {
        const float* h0_ptr = h_base + static_cast<size_t>(arm0) * h_stride;
        const float* h1_ptr = h_base + static_cast<size_t>(arm1) * h_stride;
        const float* h2_ptr = h_base + static_cast<size_t>(arm2) * h_stride;
        const float* h3_ptr = h_base + static_cast<size_t>(arm3) * h_stride;

        __m256 acc0_lo = _mm256_setzero_ps(), acc0_hi = _mm256_setzero_ps();
        __m256 acc1_lo = _mm256_setzero_ps(), acc1_hi = _mm256_setzero_ps();
        __m256 acc2_lo = _mm256_setzero_ps(), acc2_hi = _mm256_setzero_ps();
        __m256 acc3_lo = _mm256_setzero_ps(), acc3_hi = _mm256_setzero_ps();

        const bool w1_same = (win1 == win0);
        const bool w2_same = (win2 == win1);
        const bool w3_same = (win3 == win2);

        for (size_t q = 0; q < H_pad; q += 8) {
            __m256 x0_lo = load_sc16_x4_as_ps(win0 + q);
            __m256 x0_hi = load_sc16_x4_as_ps(win0 + q + 4);
            acc0_lo = _mm256_fmadd_ps(x0_lo, _mm256_loadu_ps(h0_ptr + 2 * q), acc0_lo);
            acc0_hi = _mm256_fmadd_ps(x0_hi, _mm256_loadu_ps(h0_ptr + 2 * q + 8), acc0_hi);

            __m256 x1_lo = w1_same ? x0_lo : load_sc16_x4_as_ps(win1 + q);
            __m256 x1_hi = w1_same ? x0_hi : load_sc16_x4_as_ps(win1 + q + 4);
            acc1_lo = _mm256_fmadd_ps(x1_lo, _mm256_loadu_ps(h1_ptr + 2 * q), acc1_lo);
            acc1_hi = _mm256_fmadd_ps(x1_hi, _mm256_loadu_ps(h1_ptr + 2 * q + 8), acc1_hi);

            __m256 x2_lo = w2_same ? x1_lo : load_sc16_x4_as_ps(win2 + q);
            __m256 x2_hi = w2_same ? x1_hi : load_sc16_x4_as_ps(win2 + q + 4);
            acc2_lo = _mm256_fmadd_ps(x2_lo, _mm256_loadu_ps(h2_ptr + 2 * q), acc2_lo);
            acc2_hi = _mm256_fmadd_ps(x2_hi, _mm256_loadu_ps(h2_ptr + 2 * q + 8), acc2_hi);

            __m256 x3_lo = w3_same ? x2_lo : load_sc16_x4_as_ps(win3 + q);
            __m256 x3_hi = w3_same ? x2_hi : load_sc16_x4_as_ps(win3 + q + 4);
            acc3_lo = _mm256_fmadd_ps(x3_lo, _mm256_loadu_ps(h3_ptr + 2 * q), acc3_lo);
            acc3_hi = _mm256_fmadd_ps(x3_hi, _mm256_loadu_ps(h3_ptr + 2 * q + 8), acc3_hi);
        }

        __m256 a0 = _mm256_add_ps(acc0_lo, acc0_hi);
        __m256 a1 = _mm256_add_ps(acc1_lo, acc1_hi);
        __m256 a2 = _mm256_add_ps(acc2_lo, acc2_hi);
        __m256 a3 = _mm256_add_ps(acc3_lo, acc3_hi);

        __m128 s0 = _mm_add_ps(_mm256_castps256_ps128(a0), _mm256_extractf128_ps(a0, 1));
        __m128 s1 = _mm_add_ps(_mm256_castps256_ps128(a1), _mm256_extractf128_ps(a1, 1));
        __m128 s2 = _mm_add_ps(_mm256_castps256_ps128(a2), _mm256_extractf128_ps(a2, 1));
        __m128 s3 = _mm_add_ps(_mm256_castps256_ps128(a3), _mm256_extractf128_ps(a3, 1));

        __m128 out01 = _mm_add_ps(
            _mm_castpd_ps(_mm_unpacklo_pd(_mm_castps_pd(s0), _mm_castps_pd(s1))),
            _mm_castpd_ps(_mm_unpackhi_pd(_mm_castps_pd(s0), _mm_castps_pd(s1))));
        __m128 out23 = _mm_add_ps(
            _mm_castpd_ps(_mm_unpacklo_pd(_mm_castps_pd(s2), _mm_castps_pd(s3))),
            _mm_castpd_ps(_mm_unpackhi_pd(_mm_castps_pd(s2), _mm_castps_pd(s3))));

        _mm_storeu_ps(reinterpret_cast<float*>(dest4), out01);
        _mm_storeu_ps(reinterpret_cast<float*>(dest4 + 2), out23);
    }

    static inline void fir_avx2_4x(const float* h_base,
                                   size_t h_stride,
                                   uint8_t arm0, const gr_complex* win0,
                                   uint8_t arm1, const gr_complex* win1,
                                   uint8_t arm2, const gr_complex* win2,
                                   uint8_t arm3, const gr_complex* win3,
                                   size_t H_pad,
                                   gr_complex* dest4) noexcept
    {
        const float* h0_ptr = h_base + static_cast<size_t>(arm0) * h_stride;
        const float* h1_ptr = h_base + static_cast<size_t>(arm1) * h_stride;
        const float* h2_ptr = h_base + static_cast<size_t>(arm2) * h_stride;
        const float* h3_ptr = h_base + static_cast<size_t>(arm3) * h_stride;

        const float* xp0 = reinterpret_cast<const float*>(win0);
        const float* xp1 = reinterpret_cast<const float*>(win1);
        const float* xp2 = reinterpret_cast<const float*>(win2);
        const float* xp3 = reinterpret_cast<const float*>(win3);

        __m256 acc0_lo = _mm256_setzero_ps(), acc0_hi = _mm256_setzero_ps();
        __m256 acc1_lo = _mm256_setzero_ps(), acc1_hi = _mm256_setzero_ps();
        __m256 acc2_lo = _mm256_setzero_ps(), acc2_hi = _mm256_setzero_ps();
        __m256 acc3_lo = _mm256_setzero_ps(), acc3_hi = _mm256_setzero_ps();

        const bool w1_same = (win1 == win0);
        const bool w2_same = (win2 == win1);
        const bool w3_same = (win3 == win2);

        for (size_t q = 0; q < H_pad; q += 8) {
            __m256 x0_lo = _mm256_loadu_ps(xp0 + 2 * q);
            __m256 x0_hi = _mm256_loadu_ps(xp0 + 2 * q + 8);
            acc0_lo = _mm256_fmadd_ps(x0_lo, _mm256_loadu_ps(h0_ptr + 2 * q), acc0_lo);
            acc0_hi = _mm256_fmadd_ps(x0_hi, _mm256_loadu_ps(h0_ptr + 2 * q + 8), acc0_hi);

            __m256 x1_lo = w1_same ? x0_lo : _mm256_loadu_ps(xp1 + 2 * q);
            __m256 x1_hi = w1_same ? x0_hi : _mm256_loadu_ps(xp1 + 2 * q + 8);
            acc1_lo = _mm256_fmadd_ps(x1_lo, _mm256_loadu_ps(h1_ptr + 2 * q), acc1_lo);
            acc1_hi = _mm256_fmadd_ps(x1_hi, _mm256_loadu_ps(h1_ptr + 2 * q + 8), acc1_hi);

            __m256 x2_lo = w2_same ? x1_lo : _mm256_loadu_ps(xp2 + 2 * q);
            __m256 x2_hi = w2_same ? x1_hi : _mm256_loadu_ps(xp2 + 2 * q + 8);
            acc2_lo = _mm256_fmadd_ps(x2_lo, _mm256_loadu_ps(h2_ptr + 2 * q), acc2_lo);
            acc2_hi = _mm256_fmadd_ps(x2_hi, _mm256_loadu_ps(h2_ptr + 2 * q + 8), acc2_hi);

            __m256 x3_lo = w3_same ? x2_lo : _mm256_loadu_ps(xp3 + 2 * q);
            __m256 x3_hi = w3_same ? x2_hi : _mm256_loadu_ps(xp3 + 2 * q + 8);
            acc3_lo = _mm256_fmadd_ps(x3_lo, _mm256_loadu_ps(h3_ptr + 2 * q), acc3_lo);
            acc3_hi = _mm256_fmadd_ps(x3_hi, _mm256_loadu_ps(h3_ptr + 2 * q + 8), acc3_hi);
        }

        __m256 a0 = _mm256_add_ps(acc0_lo, acc0_hi);
        __m256 a1 = _mm256_add_ps(acc1_lo, acc1_hi);
        __m256 a2 = _mm256_add_ps(acc2_lo, acc2_hi);
        __m256 a3 = _mm256_add_ps(acc3_lo, acc3_hi);

        __m128 s0 = _mm_add_ps(_mm256_castps256_ps128(a0), _mm256_extractf128_ps(a0, 1));
        __m128 s1 = _mm_add_ps(_mm256_castps256_ps128(a1), _mm256_extractf128_ps(a1, 1));
        __m128 s2 = _mm_add_ps(_mm256_castps256_ps128(a2), _mm256_extractf128_ps(a2, 1));
        __m128 s3 = _mm_add_ps(_mm256_castps256_ps128(a3), _mm256_extractf128_ps(a3, 1));

        __m128 out01 = _mm_add_ps(
            _mm_castpd_ps(_mm_unpacklo_pd(_mm_castps_pd(s0), _mm_castps_pd(s1))),
            _mm_castpd_ps(_mm_unpackhi_pd(_mm_castps_pd(s0), _mm_castps_pd(s1))));
        __m128 out23 = _mm_add_ps(
            _mm_castpd_ps(_mm_unpacklo_pd(_mm_castps_pd(s2), _mm_castps_pd(s3))),
            _mm_castpd_ps(_mm_unpackhi_pd(_mm_castps_pd(s2), _mm_castps_pd(s3))));

        _mm_storeu_ps(reinterpret_cast<float*>(dest4), out01);
        _mm_storeu_ps(reinterpret_cast<float*>(dest4 + 2), out23);
    }

    /**
     * C1 kernel: four FIR windows over SC16 input, compile-time tap count.
     *
     * Instruction-for-instruction the same as fir_avx2_sc16_4x() above -- two
     * 8-lane accumulators per output, one FMA per 8-float tap vector,
     * acc_lo + acc_hi, then the identical hadd/hadd reduction -- with two
     * differences that are pure code generation and cannot change one bit:
     *
     *   * H_PAD != 0 makes the tap-loop bound a compile-time constant, so the
     *     body is fully unrolled (3 iterations for H_pad = 24, 6 for 48);
     *     H_PAD == 0 keeps the runtime bound for any other profile.
     *   * the four windows arrive as precomputed absolute pointers (relative
     *     offsets from the macroblock base) instead of a serial
     *     cur_post += advs[o] chain, and the reuse mask arrives precomputed.
     *
     * h0..h3 point at one pre-scaled tap vector each
     * (arms_rev_dup_scaled_ + arm*2*H_pad_); in the reuse case the caller
     * passes the same window pointer twice, exactly like the default kernel
     * compares win1 == win0.
     */
    template <size_t H_PAD>
    static inline void fir_avx2_sc16_4x_hp(const float* h0,
                                           const float* h1,
                                           const float* h2,
                                           const float* h3,
                                           const std::complex<int16_t>* win0,
                                           const std::complex<int16_t>* win1,
                                           const std::complex<int16_t>* win2,
                                           const std::complex<int16_t>* win3,
                                           size_t hpad_rt,
                                           uint8_t reuse,
                                           gr_complex* dest4) noexcept
    {
        const size_t HP = (H_PAD != 0) ? H_PAD : hpad_rt;
        const bool w1_same = (reuse & 2u) != 0u;
        const bool w2_same = (reuse & 4u) != 0u;
        const bool w3_same = (reuse & 8u) != 0u;

        __m256 acc0_lo = _mm256_setzero_ps(), acc0_hi = _mm256_setzero_ps();
        __m256 acc1_lo = _mm256_setzero_ps(), acc1_hi = _mm256_setzero_ps();
        __m256 acc2_lo = _mm256_setzero_ps(), acc2_hi = _mm256_setzero_ps();
        __m256 acc3_lo = _mm256_setzero_ps(), acc3_hi = _mm256_setzero_ps();

        for (size_t q = 0; q < HP; q += 8) {
            const __m256 x0_lo = load_sc16_x4_as_ps(win0 + q);
            const __m256 x0_hi = load_sc16_x4_as_ps(win0 + q + 4);
            acc0_lo = _mm256_fmadd_ps(x0_lo, _mm256_loadu_ps(h0 + 2 * q), acc0_lo);
            acc0_hi = _mm256_fmadd_ps(x0_hi, _mm256_loadu_ps(h0 + 2 * q + 8), acc0_hi);

            const __m256 x1_lo = w1_same ? x0_lo : load_sc16_x4_as_ps(win1 + q);
            const __m256 x1_hi = w1_same ? x0_hi : load_sc16_x4_as_ps(win1 + q + 4);
            acc1_lo = _mm256_fmadd_ps(x1_lo, _mm256_loadu_ps(h1 + 2 * q), acc1_lo);
            acc1_hi = _mm256_fmadd_ps(x1_hi, _mm256_loadu_ps(h1 + 2 * q + 8), acc1_hi);

            const __m256 x2_lo = w2_same ? x1_lo : load_sc16_x4_as_ps(win2 + q);
            const __m256 x2_hi = w2_same ? x1_hi : load_sc16_x4_as_ps(win2 + q + 4);
            acc2_lo = _mm256_fmadd_ps(x2_lo, _mm256_loadu_ps(h2 + 2 * q), acc2_lo);
            acc2_hi = _mm256_fmadd_ps(x2_hi, _mm256_loadu_ps(h2 + 2 * q + 8), acc2_hi);

            const __m256 x3_lo = w3_same ? x2_lo : load_sc16_x4_as_ps(win3 + q);
            const __m256 x3_hi = w3_same ? x2_hi : load_sc16_x4_as_ps(win3 + q + 4);
            acc3_lo = _mm256_fmadd_ps(x3_lo, _mm256_loadu_ps(h3 + 2 * q), acc3_lo);
            acc3_hi = _mm256_fmadd_ps(x3_hi, _mm256_loadu_ps(h3 + 2 * q + 8), acc3_hi);
        }

        const __m256 a0 = _mm256_add_ps(acc0_lo, acc0_hi);
        const __m256 a1 = _mm256_add_ps(acc1_lo, acc1_hi);
        const __m256 a2 = _mm256_add_ps(acc2_lo, acc2_hi);
        const __m256 a3 = _mm256_add_ps(acc3_lo, acc3_hi);

        const __m128 s0 = _mm_add_ps(_mm256_castps256_ps128(a0), _mm256_extractf128_ps(a0, 1));
        const __m128 s1 = _mm_add_ps(_mm256_castps256_ps128(a1), _mm256_extractf128_ps(a1, 1));
        const __m128 s2 = _mm_add_ps(_mm256_castps256_ps128(a2), _mm256_extractf128_ps(a2, 1));
        const __m128 s3 = _mm_add_ps(_mm256_castps256_ps128(a3), _mm256_extractf128_ps(a3, 1));

        const __m128 out01 = _mm_add_ps(
            _mm_castpd_ps(_mm_unpacklo_pd(_mm_castps_pd(s0), _mm_castps_pd(s1))),
            _mm_castpd_ps(_mm_unpackhi_pd(_mm_castps_pd(s0), _mm_castps_pd(s1))));
        const __m128 out23 = _mm_add_ps(
            _mm_castpd_ps(_mm_unpacklo_pd(_mm_castps_pd(s2), _mm_castps_pd(s3))),
            _mm_castpd_ps(_mm_unpackhi_pd(_mm_castps_pd(s2), _mm_castps_pd(s3))));

        _mm_storeu_ps(reinterpret_cast<float*>(dest4), out01);
        _mm_storeu_ps(reinterpret_cast<float*>(dest4 + 2), out23);
    }

#endif

    gr_complex filter_window_sc16(uint32_t arm_idx,
                                  const std::complex<int16_t>* win_oldest) const
    {
        switch (fir_kind_) {
        case FirKind::Scalar:
        case FirKind::Volk: {
            const float* trev_dup_scaled =
                arms_rev_dup_scaled_.data() +
                static_cast<size_t>(arm_idx) * (2 * H_pad_);
            return fir_scalar_sc16(trev_dup_scaled, win_oldest, H_);
        }
        case FirKind::Avx2:
#if UWB_RESAMP_HAVE_AVX2
        {
            const float* trev_dup_scaled =
                arms_rev_dup_scaled_.data() +
                static_cast<size_t>(arm_idx) * (2 * H_pad_);
            return fir_avx2_sc16(trev_dup_scaled, win_oldest, H_pad_);
        }
#else
        {
            const float* trev_dup_scaled =
                arms_rev_dup_scaled_.data() +
                static_cast<size_t>(arm_idx) * (2 * H_pad_);
            return fir_scalar_sc16(trev_dup_scaled, win_oldest, H_);
        }
#endif
        }
        const float* trev_dup_scaled =
            arms_rev_dup_scaled_.data() +
            static_cast<size_t>(arm_idx) * (2 * H_pad_);
        return fir_scalar_sc16(trev_dup_scaled, win_oldest, H_);
    }

    gr_complex filter_window(uint32_t arm_idx,
                             const gr_complex* win_oldest) const
    {
        switch (fir_kind_) {
        case FirKind::Scalar: {
            const float* trev =
                arms_rev_.data() + static_cast<size_t>(arm_idx) * H_;
            return fir_scalar(trev, win_oldest, H_);
        }
        case FirKind::Volk: {
            const float* trev =
                arms_rev_pad_.data() + static_cast<size_t>(arm_idx) * H_pad_;
            return fir_volk(trev, win_oldest, H_);
        }
        case FirKind::Avx2:
#if UWB_RESAMP_HAVE_AVX2
        {
            const float* trev_dup =
                arms_rev_dup_.data() + static_cast<size_t>(arm_idx) * (2 * H_pad_);
            return fir_avx2(trev_dup, win_oldest, H_pad_);
        }
#else
        {
            const float* trev =
                arms_rev_pad_.data() + static_cast<size_t>(arm_idx) * H_pad_;
            return fir_volk(trev, win_oldest, H_);
        }
#endif
        }
        const float* trev =
            arms_rev_.data() + static_cast<size_t>(arm_idx) * H_;
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
#if UWB_RESAMP_HAVE_AVX2
            if (fir_kind_ == FirKind::Avx2) {
                const float* h_base = arms_rev_dup_.data();
                const size_t h_stride = 2 * H_pad_;

                uint32_t o = 0;
                for (; o + 4 <= kInterp; o += 4) {
                    const gr_complex* win0 =
                        work_.data() + (hist_n + cur_post + 1 - H_);
                    cur_post += advs[o];
                    const gr_complex* win1 =
                        work_.data() + (hist_n + cur_post + 1 - H_);
                    cur_post += advs[o + 1];
                    const gr_complex* win2 =
                        work_.data() + (hist_n + cur_post + 1 - H_);
                    cur_post += advs[o + 2];
                    const gr_complex* win3 =
                        work_.data() + (hist_n + cur_post + 1 - H_);
                    cur_post += advs[o + 3];

                    fir_avx2_4x(h_base, h_stride,
                                arms[o], win0,
                                arms[o + 1], win1,
                                arms[o + 2], win2,
                                arms[o + 3], win3,
                                H_pad_, dest + o);
                }
                for (; o < kInterp; ++o) {
                    const size_t win_start = hist_n + cur_post + 1 - H_;
                    dest[o] = filter_window(arms[o], work_.data() + win_start);
                    cur_post += advs[o];
                }
            } else
#endif
            {
                for (uint32_t o = 0; o < kInterp; ++o) {
                    const size_t win_start = hist_n + cur_post + 1 - H_;
                    dest[o] = filter_window(arms[o], work_.data() + win_start);
                    cur_post += advs[o];
                }
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
        //
        // Lazy sizing (2026-11-11 round 2, A1): this is the only consumer of
        // the schedule tables, so guarantee capacity here — before the batch
        // bound is read from it — and keep it for every later call.
        ensure_schedule_scratch();
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

    ProcessResult run_legacy_sc16(size_t hist_n,
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
                filter_window_sc16(ctr_, work_sc16_.data() + win_start);
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
                    update_hist_from_work_sc16(total_w);
                    have_current_ = false;
                    if (profile_enabled_)
                        profile_.ns_state += ns_since(t_st);
                    ProcessResult r;
                    r.produced = produced;
                    r.consumed = ninput;
                    return r;
                }
                current_sc16_ = work_sc16_[hist_n + post_i];
                have_current_ = true;
            }
        }
        if (profile_enabled_)
            profile_.ns_fir += ns_since(t_f0);

        const auto t_st =
            profile_enabled_ ? Clock::now() : Clock::time_point{};
        if (hist_n > 0) {
            const size_t cur = hist_n + post_i;
            std::memcpy(hist_sc16_.data(),
                        work_sc16_.data() + (cur - hist_n),
                        hist_n * sizeof(std::complex<int16_t>));
        }
        have_current_ = true;
        current_sc16_ = work_sc16_[hist_n + post_i];
        if (profile_enabled_)
            profile_.ns_state += ns_since(t_st);

        ProcessResult r;
        r.produced = produced;
        r.consumed = consumed_from_posts(started_held, post_i + 1, ninput,
                                         /*exhausted=*/false);
        return r;
    }

    ProcessResult run_macroblock_sc16(size_t hist_n,
                                      size_t post_n,
                                      size_t post_i,
                                      bool started_held,
                                      size_t ninput,
                                      size_t total_w,
                                      gr_complex* out,
                                      size_t max_out)
    {
        // Round-2 kernel candidate C1 (C2 was removed).  Exactly ONE predicted branch per
        // process_sc16() call (never per output) on a member that is false in
        // the default configuration, so the default instruction stream below
        // is the pre-existing one.
#if UWB_RESAMP_HAVE_AVX2
        if (fir_kind_ == FirKind::Avx2 && fixed_phase_) {
            return run_macroblock_sc16_candidate(hist_n, post_n, post_i,
                                                 started_held, ninput, total_w,
                                                 out, max_out);
        }
#endif
        size_t produced = 0;

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

#if UWB_RESAMP_HAVE_AVX2
            if (fir_kind_ == FirKind::Avx2) {
                const float* h_base = arms_rev_dup_scaled_.data();
                const size_t h_stride = 2 * H_pad_;

                uint32_t o = 0;
                for (; o + 4 <= kInterp; o += 4) {
                    const std::complex<int16_t>* win0 =
                        work_sc16_.data() + (hist_n + cur_post + 1 - H_);
                    cur_post += advs[o];
                    const std::complex<int16_t>* win1 =
                        work_sc16_.data() + (hist_n + cur_post + 1 - H_);
                    cur_post += advs[o + 1];
                    const std::complex<int16_t>* win2 =
                        work_sc16_.data() + (hist_n + cur_post + 1 - H_);
                    cur_post += advs[o + 2];
                    const std::complex<int16_t>* win3 =
                        work_sc16_.data() + (hist_n + cur_post + 1 - H_);
                    cur_post += advs[o + 3];

                    fir_avx2_sc16_4x(h_base, h_stride,
                                     arms[o], win0,
                                     arms[o + 1], win1,
                                     arms[o + 2], win2,
                                     arms[o + 3], win3,
                                     H_pad_, dest + o);
                }
                for (; o < kInterp; ++o) {
                    const size_t win_start = hist_n + cur_post + 1 - H_;
                    dest[o] = filter_window_sc16(arms[o], work_sc16_.data() + win_start);
                    cur_post += advs[o];
                }
            } else
#endif
            {
                for (uint32_t o = 0; o < kInterp; ++o) {
                    const size_t win_start = hist_n + cur_post + 1 - H_;
                    dest[o] = filter_window_sc16(arms[o], work_sc16_.data() + win_start);
                    cur_post += advs[o];
                }
            }

            if (profile_enabled_) {
                profile_.ns_fir += ns_since(t_f0);
                profile_.macroblocks += 1;
            }

            produced += kInterp;
            output_items_ += kInterp;
            post_i += kDecim;
            current_sc16_ = work_sc16_[hist_n + post_i];
            have_current_ = true;
        }

        // Remainder: per-output.
        const auto t_f1 =
            profile_enabled_ ? Clock::now() : Clock::time_point{};
        while (produced < max_out) {
            const size_t win_start = hist_n + post_i + 1 - H_;
            out[produced++] =
                filter_window_sc16(ctr_, work_sc16_.data() + win_start);
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
                    update_hist_from_work_sc16(total_w);
                    have_current_ = false;
                    if (profile_enabled_)
                        profile_.ns_state += ns_since(t_st);
                    ProcessResult r;
                    r.produced = produced;
                    r.consumed = ninput;
                    return r;
                }
                current_sc16_ = work_sc16_[hist_n + post_i];
                have_current_ = true;
            }
        }
        if (profile_enabled_)
            profile_.ns_fir += ns_since(t_f1);

        const auto t_st =
            profile_enabled_ ? Clock::now() : Clock::time_point{};
        if (hist_n > 0) {
            const size_t cur = hist_n + post_i;
            std::memcpy(hist_sc16_.data(),
                        work_sc16_.data() + (cur - hist_n),
                        hist_n * sizeof(std::complex<int16_t>));
        }
        have_current_ = true;
        current_sc16_ = work_sc16_[hist_n + post_i];
        if (profile_enabled_)
            profile_.ns_state += ns_since(t_st);

        ProcessResult r;
        r.produced = produced;
        r.consumed = consumed_from_posts(started_held, post_i + 1, ninput,
                                         /*exhausted=*/false);
        return r;
    }

    // =====================================================================
    // Round-2 kernel candidate C1 -- SC16 macroblock path (C2 removed)
    //
    // Compiled only when the AVX2/FMA kernel exists (the candidates are
    // alternative implementations of that kernel; without it
    // run_macroblock_sc16() never dispatches here and the switches are inert).
    // =====================================================================
#if UWB_RESAMP_HAVE_AVX2

    /**
     * Remainder + history tail, shared by every candidate variant.
     *
     * Byte-for-byte the same statements as the tail of run_macroblock_sc16()
     * above (per-output filter_window_sc16 with the production clamps, then
     * the same history/cursor commit).  It is factored out only so the nine
     * template instantiations do not each carry their own copy; the code that
     * runs is identical to the default path, which is what keeps partial
     * macroblocks, tiny max_out grants and the flush tail bit-identical.
     */
    ProcessResult finish_macroblock_sc16(size_t hist_n,
                                         size_t post_n,
                                         size_t post_i,
                                         bool started_held,
                                         size_t ninput,
                                         size_t total_w,
                                         gr_complex* out,
                                         size_t produced,
                                         size_t max_out)
    {
        while (produced < max_out) {
            const size_t win_start = hist_n + post_i + 1 - H_;
            out[produced++] =
                filter_window_sc16(ctr_, work_sc16_.data() + win_start);
            ++output_items_;
            ctr_ += kDecim;
            while (ctr_ >= kInterp) {
                ctr_ -= kInterp;
                ++post_i;
                if (post_i >= post_n) {
                    update_hist_from_work_sc16(total_w);
                    have_current_ = false;
                    ProcessResult r;
                    r.produced = produced;
                    r.consumed = ninput;
                    return r;
                }
                current_sc16_ = work_sc16_[hist_n + post_i];
                have_current_ = true;
            }
        }

        if (hist_n > 0) {
            const size_t cur = hist_n + post_i;
            std::memcpy(hist_sc16_.data(),
                        work_sc16_.data() + (cur - hist_n),
                        hist_n * sizeof(std::complex<int16_t>));
        }
        have_current_ = true;
        current_sc16_ = work_sc16_[hist_n + post_i];

        ProcessResult r;
        r.produced = produced;
        r.consumed = consumed_from_posts(started_held, post_i + 1, ninput,
                                         /*exhausted=*/false);
        return r;
    }

    /**
     * Full-macroblock loop for one candidate variant.
     *
     * FP  = C1 (descriptor table + compile-time H_pad),
     * H_PAD = 24 / 48 (shipped profiles, fully unrolled) or 0 (generic).
     *
     * Post-condition is identical to the default loop: produced is a multiple
     * of kInterp, post_i advanced by kDecim per macroblock, ctr_ unchanged,
     * output_items_ advanced by kInterp per macroblock.
     */
    template <bool FP, size_t H_PAD>
    size_t run_mb_sc16_candidate(size_t hist_n,
                                 size_t post_n,
                                 size_t& post_i,
                                 gr_complex* out,
                                 size_t max_out)
    {
        size_t produced = 0;
        const float* const h_base = arms_rev_dup_scaled_.data();

        while (produced + kInterp <= max_out &&
               post_i + static_cast<size_t>(kDecim) < post_n) {
            const size_t base_idx = hist_n + post_i + 1 - H_;
            gr_complex* const dest = out + produced;

            if constexpr (FP) {
                const FixedPhaseGroup* const groups =
                    fp_groups_.data() +
                    static_cast<size_t>(ctr_) * kMacroblockGroups;
                for (uint32_t g = 0; g < kMacroblockGroups; ++g) {
                    const FixedPhaseGroup& d = groups[g];
                    fir_avx2_sc16_4x_hp<H_PAD>(
                        h_base + d.tap[0], h_base + d.tap[1],
                        h_base + d.tap[2], h_base + d.tap[3],
                        work_sc16_.data() + base_idx +
                            static_cast<size_t>(d.win[0]),
                        work_sc16_.data() + base_idx +
                            static_cast<size_t>(d.win[1]),
                        work_sc16_.data() + base_idx +
                            static_cast<size_t>(d.win[2]),
                        work_sc16_.data() + base_idx +
                            static_cast<size_t>(d.win[3]),
                        H_pad_, d.reuse, dest + 4 * g);
                }
                const FixedPhaseTail* const tails =
                    fp_tail_.data() +
                    static_cast<size_t>(ctr_) * kTailStride;
                for (uint32_t t = 0; t < kMacroblockTail; ++t) {
                    const size_t o = 4 * static_cast<size_t>(kMacroblockGroups) + t;
                    dest[o] = fir_avx2_sc16(
                        h_base + tails[t].tap,
                        work_sc16_.data() + base_idx +
                            static_cast<size_t>(tails[t].win),
                        H_pad_);
                }
            }

            produced += kInterp;
            output_items_ += kInterp;
            post_i += static_cast<size_t>(kDecim);
            current_sc16_ = work_sc16_[hist_n + post_i];
            have_current_ = true;
        }
        return produced;
    }

    /**
     * Candidate dispatcher: one switch on C1 x (H_pad class), all
     * outside the macroblock loop.  The H_pad class was chosen in the
     * constructor, so this is the "selected once at construction" decision.
     */
    ProcessResult run_macroblock_sc16_candidate(size_t hist_n,
                                                size_t post_n,
                                                size_t post_i,
                                                bool started_held,
                                                size_t ninput,
                                                size_t total_w,
                                                gr_complex* out,
                                                size_t max_out)
    {
        const auto t0 = profile_enabled_ ? Clock::now() : Clock::time_point{};
        size_t produced = 0;
        if (fixed_phase_) {
            if (hp_class_ == 24)
                produced = run_mb_sc16_candidate<true, 24>(
                    hist_n, post_n, post_i, out, max_out);
            else if (hp_class_ == 48)
                produced = run_mb_sc16_candidate<true, 48>(
                    hist_n, post_n, post_i, out, max_out);
            else
                produced = run_mb_sc16_candidate<true, 0>(
                    hist_n, post_n, post_i, out, max_out);
        } else {
            if (hp_class_ == 24)
                produced = run_mb_sc16_candidate<false, 24>(
                    hist_n, post_n, post_i, out, max_out);
            else if (hp_class_ == 48)
                produced = run_mb_sc16_candidate<false, 48>(
                    hist_n, post_n, post_i, out, max_out);
            else
                produced = run_mb_sc16_candidate<false, 0>(
                    hist_n, post_n, post_i, out, max_out);
        }
        if (profile_enabled_) {
            // The candidate loop does not split schedule from FIR; the whole
            // macroblock loop is reported as ns_fir so the accounting stays
            // honest (ns_schedule stays 0 on this path).
            profile_.ns_fir += ns_since(t0);
            profile_.macroblocks += produced / kInterp;
        }
        return finish_macroblock_sc16(hist_n, post_n, post_i, started_held,
                                      ninput, total_w, out, produced, max_out);
    }

#endif // UWB_RESAMP_HAVE_AVX2

    void fir_parallel_sc16(gr_complex* out,
                           size_t n_out,
                           int nworkers)
    {
        const int nw = std::max(1, nworkers);
        if (nw == 1 || n_out < 1024) {
            for (size_t i = 0; i < n_out; ++i) {
                out[i] = filter_window_sc16(sched_arm_buf_[i],
                                            work_sc16_.data() + sched_win_buf_[i]);
            }
            return;
        }
        if (pool_size_ != nw)
            start_pool(nw);
        pool_parallel_for(n_out, [this, out](size_t lo, size_t hi) {
            for (size_t i = lo; i < hi; ++i) {
                out[i] = filter_window_sc16(sched_arm_buf_[i],
                                            work_sc16_.data() + sched_win_buf_[i]);
            }
        });
    }

    ProcessResult run_mt_sc16(size_t hist_n,
                              size_t post_n,
                              size_t post_i,
                              bool started_held,
                              size_t ninput,
                              size_t total_w,
                              gr_complex* out,
                              size_t max_out,
                              int nworkers)
    {
        // Lazy sizing (2026-11-11 round 2, A1) — see run_mt().  The batch bound
        // must never be 0, hence the ensure before it is read.
        ensure_schedule_scratch();
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
            fir_parallel_sc16(out + produced_total, n_out, nworkers);
            if (profile_enabled_)
                profile_.ns_fir += ns_since(t_f0);

            produced_total += n_out;
            output_items_ += n_out;
        }

        const auto t_st =
            profile_enabled_ ? Clock::now() : Clock::time_point{};
        size_t consumed;
        if (exhausted) {
            update_hist_from_work_sc16(total_w);
            have_current_ = false;
            consumed = ninput;
        } else {
            if (hist_n > 0) {
                const size_t cur = hist_n + post_i;
                std::memcpy(hist_sc16_.data(),
                            work_sc16_.data() + (cur - hist_n),
                            hist_n * sizeof(std::complex<int16_t>));
            }
            have_current_ = true;
            current_sc16_ = work_sc16_[hist_n + post_i];
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

    size_t flush_sc16_internal(gr_complex* out,
                               size_t max_out,
                               bool first_flush,
                               uint64_t N,
                               size_t Lout)
    {
        size_t produced = 0;
        const size_t hist_n = hist_sc16_.size();

        if constexpr (kDecim > kInterp) {
            if (first_flush && hist_n > 0) {
                ctr_ = static_cast<uint32_t>(
                    (static_cast<uint64_t>(output_items_) *
                     static_cast<uint64_t>(kDecim)) %
                    static_cast<uint64_t>(kInterp));
                const int64_t logical_post =
                    (static_cast<int64_t>(output_items_) *
                     static_cast<int64_t>(kDecim)) /
                    static_cast<int64_t>(kInterp);
                const int64_t overshoot =
                    logical_post - static_cast<int64_t>(N);
                if (overshoot > 0) {
                    const size_t sh = static_cast<size_t>(
                        std::min<int64_t>(overshoot,
                                          static_cast<int64_t>(hist_n)));
                    std::memmove(hist_sc16_.data(), hist_sc16_.data() + sh,
                                 (hist_n - sh) * sizeof(std::complex<int16_t>));
                    std::fill(hist_sc16_.data() + (hist_n - sh),
                              hist_sc16_.data() + hist_n, std::complex<int16_t>(0, 0));
                }
            }
        }

        const size_t rem_out = (output_items_ < Lout) ? (Lout - output_items_) : 0;
        const size_t zeros_needed = (rem_out * kDecim) / kInterp + H_ + 32;

        const size_t held = have_current_ ? 1u : 0u;
        const size_t post_n = held + zeros_needed;
        const size_t need = hist_n + post_n + H_pad_ + 16;
        if (work_sc16_.capacity() < need)
            work_sc16_.reserve(need);
        if (work_sc16_.size() < need)
            work_sc16_.resize(need);

        if (hist_n > 0) {
            std::memcpy(work_sc16_.data(), hist_sc16_.data(),
                        hist_n * sizeof(std::complex<int16_t>));
        }
        size_t wpos = hist_n;
        if (have_current_) {
            work_sc16_[wpos++] = current_sc16_;
        }
        std::fill(work_sc16_.data() + wpos, work_sc16_.data() + need,
                  std::complex<int16_t>(0, 0));

        size_t post_i = 0;
        while (produced < max_out && output_items_ < Lout) {
            const size_t win_start = hist_n + post_i + 1 - H_;
            out[produced++] = filter_window_sc16(ctr_, work_sc16_.data() + win_start);
            ++output_items_;
            ctr_ += kDecim;
            while (ctr_ >= kInterp) {
                ctr_ -= kInterp;
                ++post_i;
            }
        }
        if (hist_n > 0) {
            const size_t cur = hist_n + post_i;
            std::memcpy(hist_sc16_.data(), work_sc16_.data() + (cur - hist_n),
                        hist_n * sizeof(std::complex<int16_t>));
        }
        have_current_ = false;
        current_sc16_ = std::complex<int16_t>(0, 0);
        return produced;
    }
};

using RationalResampler65_48Core = RationalResamplerLmCore<65, 48>;
using RationalResampler65_32Core = RationalResamplerLmCore<65, 32>;

// M2-A G0 §3.3: TX (work -> native) aliases.  Only the two RX directions above
// were instantiated; <48,65> (998.4 -> 737.28 MS/s, UC200) and <32,65>
// (998.4 -> 491.52 MS/s, CG400) are template-legal but had no alias.  The
// template is NOT re-implemented: these are the same causal upfirdn core.
// Their TX taps must have DC sum == Interp (48 / 32) so the effective gain is
// 1 (G0 §3.3); see testdata/twr/m2a/taps/.
using RationalResampler48_65Core = RationalResamplerLmCore<48, 65>;
using RationalResampler32_65Core = RationalResamplerLmCore<32, 65>;

} // namespace core
} // namespace uwb
} // namespace gr
