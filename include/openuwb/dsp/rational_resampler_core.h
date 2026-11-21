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
 * Implementations of RationalResamplerLmCore live in
 * lib/dsp/uwb_rational_resampler_core.cc; this header keeps the
 * class declaration and the public types only.
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
#include <array>
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
#include <memory>
#include <mutex>
#include <new>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <volk/volk.h>

#include <openuwb/support/simd.h>
#include <openuwb/support/simd_config.h>

// The compile-time backend is chosen once by CMake (UWB_SIMD=...).  The
// AVX2/FMA instructions now live in the openuwb_dsp translation units (see
// lib/dsp/uwb_rational_resampler_core.cc); the macros below only name the
// levels to the dispatch code that remains in this header, plus the
// qa_uwb_resampler_burst_opt.cc guard.
#if UWB_SIMD_LEVEL >= 1
#define UWB_RESAMP_HAVE_AVX2 1
#else
#define UWB_RESAMP_HAVE_AVX2 0
#endif

#if UWB_SIMD_LEVEL >= 1
// The declarations of the AVX2 helper members below still name
// `__m256` types; the implementations live in
// lib/dsp/uwb_rational_resampler_core.cc.
#include <immintrin.h>
#endif
#if UWB_SIMD_HAVE_AVX512
#include <openuwb/dsp/rational_resampler_avx512.h>
#define UWB_RESAMP_HAVE_AVX512 1
#else
#define UWB_RESAMP_HAVE_AVX512 0
#endif

#include <openuwb/dsp/resampler_transposed.h>

namespace openuwb {
namespace core {

/**
 * RS-3: 64-byte aligned allocator for the resampler coefficient tables.
 *
 * Only the duplicated/reversed SC16 tap tables need alignment: the AVX-512
 * kernels load them with _mm512_load_ps.  The remaining float vectors keep
 * std::vector's default allocator, so this stays a local, minimal helper.
 */
template <class T, size_t Align>
struct AlignedAllocator {
    using value_type = T;

    AlignedAllocator() noexcept = default;
    template <class U>
    AlignedAllocator(const AlignedAllocator<U, Align>&) noexcept
    {
    }

    T* allocate(size_t n)
    {
        return static_cast<T*>(
            ::operator new(n * sizeof(T), std::align_val_t(Align)));
    }

    void deallocate(T* p, size_t) noexcept
    {
        ::operator delete(p, std::align_val_t(Align));
    }

    template <class U>
    struct rebind {
        using other = AlignedAllocator<U, Align>;
    };
};

template <class T, size_t Align>
bool operator==(const AlignedAllocator<T, Align>&,
                const AlignedAllocator<T, Align>&) noexcept
{
    return true;
}

using AlignedFloatVector = std::vector<float, AlignedAllocator<float, 64>>;

template <uint32_t Interp, uint32_t Decim>
class RationalResamplerLmCore
{

public:
    static constexpr uint32_t kInterp = Interp;
    static constexpr uint32_t kDecim = Decim;
    static_assert(Interp >= 1 && Decim >= 1, "interp/decim must be >= 1");
    static_assert(Interp <= 255, "interp must fit schedule_arm_ uint8_t");

    using gr_complex = std::complex<float>;
    using TransposedKernel =
        simd::transposed::TransposedResampler<Interp, Decim>;

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

    RationalResamplerLmCore(const float* taps, size_t T);

    explicit RationalResamplerLmCore(const std::vector<float>& taps);

    ~RationalResamplerLmCore();

    RationalResamplerLmCore(const RationalResamplerLmCore&) = delete;
    RationalResamplerLmCore&
    operator=(const RationalResamplerLmCore&) = delete;

    static size_t expected_output_length(uint64_t N, size_t T);

    size_t expected_output_length(uint64_t N) const;

    void force_scalar_kernel();

    void enable_volk_kernel();

    /** Force kernel for A/B: scalar|scalar_legacy|volk|volk_legacy|avx2|avx512|default */
    void set_kernel(const std::string& name);

    void set_num_workers(int n);

    int num_workers() const;

    void enable_profiling(bool on);

    bool profiling_enabled() const;

    const ProfileStats& profile_stats() const;

    void reset_profile();

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

    void set_fixed_phase_schedule(bool on);

    bool fixed_phase_schedule() const;

    /** C1 table accessors (QA verifies every phase against a local model). */
    bool fixed_phase_table_built() const;

    const FixedPhaseGroup& fixed_phase_group(uint32_t phase, uint32_t g) const;

    const FixedPhaseTail& fixed_phase_tail(uint32_t phase, uint32_t t) const;

    /**
     * Which SC16 macroblock kernel process_sc16() will actually use.  The
     * candidates are inert unless the AVX2/FMA default kernel is selected, so
     * a build without AVX2 reports "default" even with a switch on.
     */
    const char* sc16_kernel_variant() const;

    bool transposed_enabled() const;

    const char* transposed_kernel_name() const;

    ProcessResult process(const gr_complex* in,
                          size_t ninput,
                          gr_complex* out,
                          size_t max_out,
                          int nworkers_override = 0);

    size_t process(const gr_complex* in, size_t ninput, gr_complex* out);

    ProcessResult process_sc16(const std::complex<int16_t>* in_sc16,
                               size_t ninput,
                               float scale,
                               gr_complex* out,
                               size_t max_out,
                               int nworkers_override = 0);

    size_t process_sc16(const std::complex<int16_t>* in_sc16,
                        size_t ninput,
                        float scale,
                        gr_complex* out);

    /**
     * RS-1: one-shot whole-window SC16 -> CF32 resampling.
     *
     * Equivalent to reset() + process_sc16() + flush-to-completion, but the
     * complete macroblocks in the middle of the window are consumed DIRECTLY
     * from `in` (zero copy): only the leading macroblocks that read the
     * implicit H-1 leading zeros, and the trailing macroblocks/remainder that
     * read past the end, are assembled in small stack buffers.  That removes
     * the whole-window work_sc16_ memcpy (224 KB for a 56k sample window,
     * ~1.8 MB for the 450k production window) from the burst worker's hot
     * path.
     *
     * The per-output arithmetic is the existing kernel: for the middle
     * macroblocks it is the same macroblock schedule / C1 descriptor table and
     * the same tap tables, so the result is BIT-IDENTICAL to the streaming
     * sequence (the QA gate checks memcmp == 0).
     *
     * If `max_out` cannot hold the complete window (or the tap arm is so large
     * that the fixed stack buffers would not fit), the call falls back to the
     * normal reset + process_sc16 + flush sequence and keeps streaming
     * semantics.  resample_sc16_window() always sizes output to the expected
     * length, so production takes the zero-copy path.
     */
    ProcessResult process_window_sc16(const std::complex<int16_t>* in,
                                      size_t ninput,
                                      float scale,
                                      gr_complex* out,
                                      size_t max_out);

    size_t flush(gr_complex* out, size_t max_out);

    size_t flush_sc16(gr_complex* out, size_t max_out);

    bool flush_complete() const;

    void reset();

    uint64_t input_items() const;

    uint64_t output_items() const;

    uint32_t phase() const;

    uint64_t resets() const;

    size_t tap_count() const;

    size_t arm_length() const;

    size_t arm_length_pad() const;

    size_t history_extra() const;

    const char* kernel_name() const;

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
    size_t schedule_scratch_entries() const;

    size_t schedule_scratch_bytes() const;

    bool schedule_scratch_allocated() const;

    const std::vector<float>& arms_rev_pad() const;

    const AlignedFloatVector& arms_rev_dup() const;

    const AlignedFloatVector& arms_rev_dup_scaled() const;

    float last_scale() const;

    const std::vector<std::complex<int16_t>>& hist_sc16() const;

    std::complex<int16_t> current_sc16() const;

    bool is_sc16_mode() const;

    void ensure_scaled_taps(float scale);

    float arm_tap(size_t arm, size_t k) const;

    int64_t map_input_offset_to_output(int64_t p) const;

private:
    using Clock = std::chrono::steady_clock;

    enum class FirKind { Scalar, Volk, Avx2, Avx512 };
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
    void ensure_schedule_scratch();

    size_t T_;
    size_t H_;
    size_t H_pad_;
    // Original polyphase taps (order used by the transposed kernel builder).
    std::vector<float> taps_original_;
    std::vector<float> arms_;
    std::vector<float> arms_rev_;
    std::vector<float> arms_rev_pad_;
    // RS-3: the SC16 tap tables are 64-byte aligned so the AVX-512 kernels can
    // use _mm512_load_ps.  H_pad is always a multiple of 8, hence every arm
    // base (arm * 2 * H_pad floats) keeps the 64-byte alignment.
    AlignedFloatVector arms_rev_dup_;
    AlignedFloatVector arms_rev_dup_scaled_;
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
    // FC32 (process()) uses AVX-512 only when explicitly forced with
    // set_kernel("avx512").  The default AVX-512 selection is for the SC16
    // production path; the FC32 GR path measured ~6-8% slower for H_pad
    // 16/32, so it stays on AVX2 unless the caller asks for the wide kernel.
    bool fc32_use_avx512_ = false;
    const char* kernel_name_ = "volk_macroblock";
    // RS-5: optional one-shot transposed window kernel (explicit switch).
    bool transposed_enabled_ = false;
    std::unique_ptr<TransposedKernel> transposed_;
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

    static double ns_since(Clock::time_point t0);

    void start_pool(int n);

    void stop_pool();

    void pool_parallel_for(size_t n,
                           const std::function<void(size_t, size_t)>& job);

    void enable_transposed(const std::string& kernel);

    void select_default_kernel();

    void update_hist_from_work(size_t total_w);

    void update_hist_from_work_sc16(size_t total_w);

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
    void build_fixed_phase_schedule();

    static gr_complex fir_scalar(const float* trev,
                                 const gr_complex* win,
                                 size_t H);

    static inline gr_complex fir_scalar_sc16(const float* trev_dup_scaled,
                                             const std::complex<int16_t>* win,
                                             size_t H) noexcept;

    static gr_complex fir_volk(const float* trev,
                               const gr_complex* win,
                               size_t H);

#if UWB_RESAMP_HAVE_AVX2

    static inline __m256 load_sc16_x4_as_ps(const std::complex<int16_t>* p) noexcept;

    static gr_complex fir_avx2(const float* trev_dup,
                               const gr_complex* win,
                               size_t H_pad);

    static inline gr_complex fir_avx2_sc16(const float* trev_dup_scaled,
                                           const std::complex<int16_t>* win,
                                           size_t H_pad) noexcept;

    // RS-4 note (2026-11-19): a compile-time reuse-mask specialisation and a
    // branchless pointer-preselect variant of this AVX2 kernel were both
    // measured on the i7-12700 AVX2 build and NOT retained: they regressed the
    // 56k/450k realtime window by 7-23% (extra loads / code bloat) with no
    // stable gain.  The pre-existing branchy form below is kept verbatim.
    // The AVX-512 C1 path does implement the branchless sliding window (see
    // fir_sc16_4x_slide_impl in uwb_rational_resampler_avx512.cc).
    static inline void fir_avx2_sc16_4x(const float* h_base,
                                        size_t h_stride,
                                        uint8_t arm0, const std::complex<int16_t>* win0,
                                        uint8_t arm1, const std::complex<int16_t>* win1,
                                        uint8_t arm2, const std::complex<int16_t>* win2,
                                        uint8_t arm3, const std::complex<int16_t>* win3,
                                        size_t H_pad,
                                        gr_complex* dest4) noexcept;

    static inline void fir_avx2_4x(const float* h_base,
                                   size_t h_stride,
                                   uint8_t arm0, const gr_complex* win0,
                                   uint8_t arm1, const gr_complex* win1,
                                   uint8_t arm2, const gr_complex* win2,
                                   uint8_t arm3, const gr_complex* win3,
                                   size_t H_pad,
                                   gr_complex* dest4) noexcept;

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
                                           gr_complex* dest4) noexcept;

#endif

    gr_complex filter_window_sc16(uint32_t arm_idx,
                                  const std::complex<int16_t>* win_oldest) const;

    gr_complex filter_window(uint32_t arm_idx,
                             const gr_complex* win_oldest) const;

    // ---------------------------------------------------------------------
    // RS-1 zero-copy whole-window helpers
    // ---------------------------------------------------------------------

    // Small stack buffers for the leading/trailing macroblocks.  Guarded by
    // window_scratch_fits(); profiles that would not fit fall back to the
    // streaming path instead of overflowing.
    static constexpr size_t kWindowScratchSamples = 1024;

    bool window_scratch_fits() const;

    void finish_window_sc16(size_t produced, size_t /*ninput*/);

#if UWB_RESAMP_HAVE_AVX2

    template <size_t H_PAD>
    void window_macroblocks_c1_avx2_hp(const std::complex<int16_t>* base_win,
                                       size_t n_mb,
                                       gr_complex* dest);

#endif

#if UWB_RESAMP_HAVE_AVX512

    template <size_t H_PAD>
    void window_macroblocks_c1_avx512_hp(const std::complex<int16_t>* base_win,
                                         size_t n_mb,
                                         gr_complex* dest);

#endif

    void window_macroblocks_c1(const std::complex<int16_t>* base_win,
                               size_t n_mb,
                               gr_complex* dest);

    /**
     * Compute n_mb consecutive full macroblocks, each starting at phase 0.
     * `base_win` is the oldest-first window of macroblock 0 (virtual sample
     * index m*kDecim - (H-1)); no state outside `dest` is touched.
     */
    void window_macroblocks_sc16(const std::complex<int16_t>* base_win,
                                 size_t n_mb,
                                 gr_complex* dest);

    void window_macroblock_sc16(const std::complex<int16_t>* base_win,
                                gr_complex* dest);

    /** Compute consumed from posts_taken and started_held. */
    static size_t consumed_from_posts(bool started_held,
                                      size_t posts_taken,
                                      size_t ninput,
                                      bool exhausted);

    ProcessResult run_legacy(size_t hist_n,
                             size_t post_n,
                             size_t post_i,
                             bool started_held,
                             size_t ninput,
                             size_t total_w,
                             gr_complex* out,
                             size_t max_out);

    ProcessResult run_macroblock(size_t hist_n,
                                 size_t post_n,
                                 size_t post_i,
                                 bool started_held,
                                 size_t ninput,
                                 size_t total_w,
                                 gr_complex* out,
                                 size_t max_out);

    void fir_parallel(gr_complex* out,
                      size_t n_out,
                      int nworkers);

    ProcessResult run_mt(size_t hist_n,
                         size_t post_n,
                         size_t post_i,
                         bool started_held,
                         size_t ninput,
                         size_t total_w,
                         gr_complex* out,
                         size_t max_out,
                         int nworkers);

    ProcessResult run_legacy_sc16(size_t hist_n,
                                  size_t post_n,
                                  size_t post_i,
                                  bool started_held,
                                  size_t ninput,
                                  size_t total_w,
                                  gr_complex* out,
                                  size_t max_out);

    ProcessResult run_macroblock_sc16(size_t hist_n,
                                      size_t post_n,
                                      size_t post_i,
                                      bool started_held,
                                      size_t ninput,
                                      size_t total_w,
                                      gr_complex* out,
                                      size_t max_out);

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
                                         size_t max_out);

    /**
     * C1 4-output kernel selection.  AVX512 is a compile-time template
     * parameter so the hot loop carries no per-group branch; the AVX-512
     * entry points are bit-identical to the AVX2 ones (see the header).
     */
    template <size_t H_PAD, bool AVX512>
    static inline void fir_sc16_4x_hp_any(
        const float* h0, const float* h1,
        const float* h2, const float* h3,
        const std::complex<int16_t>* w0,
        const std::complex<int16_t>* w1,
        const std::complex<int16_t>* w2,
        const std::complex<int16_t>* w3,
        size_t hpad_rt, uint8_t reuse, gr_complex* dest4);

    template <bool AVX512>
    static inline gr_complex fir_sc16_any(
        const float* taps, const std::complex<int16_t>* win, size_t hpad);

    /**
     * Full-macroblock loop for one candidate variant.
     *
     * FP     = C1 (descriptor table + compile-time H_pad),
     * H_PAD  = 16 / 24 / 32 / 48 (shipped profiles, fully unrolled) or 0
     *          (generic),
     * AVX512 = use the AVX-512 kernels (bit-identical to AVX2).
     *
     * Post-condition is identical to the default loop: produced is a multiple
     * of kInterp, post_i advanced by kDecim per macroblock, ctr_ unchanged,
     * output_items_ advanced by kInterp per macroblock.
     */
    template <bool FP, size_t H_PAD, bool AVX512>
    size_t run_mb_sc16_candidate(size_t hist_n,
                                 size_t post_n,
                                 size_t& post_i,
                                 gr_complex* out,
                                 size_t max_out);

    /**
     * Candidate dispatcher: one switch on (ISA) x C1 x (H_pad class), all
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
                                                size_t max_out);

#endif // UWB_RESAMP_HAVE_AVX2

    /**
     * Per-output kernel for the multi-worker schedule.
     *
     * The AVX-512 backend's hot path is the batched C1 macroblock; the MT
     * schedule is an arbitrary per-output list, so calling the out-of-line
     * AVX-512 single-output kernel once per output would pay the call
     * overhead 75k times per window.  Keep this path on the inlined AVX2
     * kernel in an AVX-512 build: it is the pre-existing MT implementation and
     * this change cannot regress it.  The C1 tail (one output per macroblock)
     * still uses the AVX-512 single-output kernel.
     */
    gr_complex filter_window_sc16_mt(uint32_t arm_idx,
                                     const std::complex<int16_t>* win) const;

    void fir_parallel_sc16(gr_complex* out,
                           size_t n_out,
                           int nworkers);

    ProcessResult run_mt_sc16(size_t hist_n,
                              size_t post_n,
                              size_t post_i,
                              bool started_held,
                              size_t ninput,
                              size_t total_w,
                              gr_complex* out,
                              size_t max_out,
                              int nworkers);

    size_t flush_sc16_internal(gr_complex* out,
                               size_t max_out,
                               bool first_flush,
                               uint64_t N,
                               size_t Lout);

};
extern template class RationalResamplerLmCore<65, 48>;
extern template class RationalResamplerLmCore<65, 32>;
extern template class RationalResamplerLmCore<48, 65>;
extern template class RationalResamplerLmCore<32, 65>;

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
} // namespace openuwb
