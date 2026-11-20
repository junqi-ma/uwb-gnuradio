/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Fixed-frequency CW (tone) cancellation core for the native SC16 RX path.
 *
 * The tone frequency is CONFIGURED, never estimated: it is the receive
 * baseband frequency (f_RF - f_LO), not the RF centre frequency.  No FFT, no
 * frequency search, no per-sample NCO and no CF32 intermediate buffer.
 *
 * Mathematics (fixed sample epoch, training length Nt):
 *
 *   c        = (1 / Nt) * sum_{n=0}^{Nt-1} x[epoch + n] * exp(-j w n)
 *   i_hat[t] = c * exp(j w (t - epoch))            for every absolute sample t
 *   out[t]   = saturate_int16(in[t] - round_nearest_even(i_hat[t]))
 *
 * f0/fs = -91/288 for the validated 737.28 MS/s / -232.96 MHz configuration,
 * so exp(j w n) is exactly periodic with P = 288 samples.  The projection is
 * therefore folded first:
 *
 *   S[r] = sum_{n = r (mod P)} x[epoch + n]        (int32 per I/Q component)
 *   c    = (1 / Nt) * sum_{r=0}^{P-1} S[r] * exp(-j w r)
 *
 * which is algebraically identical to the full-window projection for ANY Nt
 * (including a non-multiple of P) because exp(-j w P) == 1.  The period is
 * derived and verified from the configured rate/frequency pair; a pair without
 * a short exact period is rejected instead of being silently approximated.
 *
 * Hot path: one load, 16 saturating int16 subtracts, one store per 8 complex
 * samples, against a 288-entry (1152 B) int16 tone table.  No allocation, no
 * sin/cos, no lock and no atomic count per sample.  The AVX2 kernel is selected
 * at run time and is bit-identical to the scalar kernel.
 */

#pragma once

#include <openuwb/support/api.h>

#include <complex>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace openuwb {
namespace core {

// Default validated configuration (X410 / 737.28 MS/s, 6256.64 - 6489.6 MHz).
inline constexpr double kFixedToneDefaultSampleRate = 737.28e6;
inline constexpr double kFixedToneDefaultFrequencyHz = -232.96e6;
inline constexpr double kFixedToneDefaultPeriodS = 20.0e-3;
inline constexpr double kFixedToneDefaultTrainingS = 0.5e-3;
inline constexpr size_t kFixedToneDefaultCrossfadeSamples = 256;
inline constexpr size_t kFixedToneDefaultRawBlockSamples = 65536;
inline constexpr size_t kFixedToneDefaultRawQueueBlocks = 32;

struct FixedToneCancelConfig {
    // Explicit opt-in.  When false the pipeline keeps its original source path
    // and the core is a bit-exact pass-through.
    bool enable = false;
    // Native (device) sample rate in Hz, NOT the host/work rate.
    double sample_rate = kFixedToneDefaultSampleRate;
    // Receive baseband frequency in Hz (f_RF - f_LO); may be negative.
    double tone_frequency_hz = kFixedToneDefaultFrequencyHz;
    // Model update period and training window, both in seconds.  Both are
    // converted to integer sample counts; wall-clock time is never used to
    // advance the phase.
    double period_s = kFixedToneDefaultPeriodS;
    double training_s = kFixedToneDefaultTrainingS;
    // Linear cross-fade length in samples between the previous and the new
    // tone model.  0 selects a hard switch (mathematical reference mode).
    size_t crossfade_samples = kFixedToneDefaultCrossfadeSamples;
};

struct FixedToneCancelModel {
    bool valid = false;
    uint64_t generation = 0;
    // Absolute sample index of the training window's first sample: the phase
    // origin of the LUT.  Tone index == (t - epoch) mod period.
    int64_t epoch = 0;
    std::complex<double> coefficient{ 0.0, 0.0 };
    double amplitude = 0.0; // |c| in ADC (int16) units
    double phase_radians = 0.0;
    size_t training_samples = 0;
};

struct FixedToneProcessCounts {
    size_t consumed = 0;
    size_t cancelled = 0;
    size_t bypassed = 0;
};

enum class FixedToneTrainingStatus {
    Ok,
    Incomplete,     // fewer than training_length_samples() were submitted
    InvalidModel,   // non-finite coefficient or a tone component that does not
                    // fit int16 (the previous LUT is kept untouched)
};

class UWB_API FixedToneCancelCore {
public:
    FixedToneCancelCore();
    ~FixedToneCancelCore();

    FixedToneCancelCore(const FixedToneCancelCore&) = delete;
    FixedToneCancelCore& operator=(const FixedToneCancelCore&) = delete;

    // Validates the configuration, resolves the exact tone period and builds
    // the one-time unit phasor base.  Returns false (with a reason in *error)
    // for an unsupported rate/frequency pair; nothing is approximated.
    bool prepare(const FixedToneCancelConfig& cfg, std::string* error = nullptr);

    bool prepared() const { return prepared_; }
    // prepared() && config.enable
    bool enabled() const { return prepared_ && cfg_.enable; }
    const FixedToneCancelConfig& config() const { return cfg_; }

    size_t period_samples() const { return period_; }
    // Signed number of tone cycles per period (f0/fs * P), e.g. -91.
    int period_cycles() const { return period_cycles_; }
    double omega_radians() const { return omega_; }
    int64_t period_length_samples() const { return period_length_; }
    size_t training_length_samples() const { return training_length_; }
    size_t crossfade_samples() const { return crossfade_length_; }
    // ceil(training_length / period); capped so the int32 fold cannot overflow.
    size_t max_fold_per_position() const { return max_fold_per_position_; }

    // --- training (CollectTraining) ------------------------------------
    // Starts a fresh fold for the window [epoch, epoch + Nt).  Any previous
    // model stays valid until finish_training() replaces it.
    bool begin_training(int64_t epoch, std::string* error = nullptr);
    // Folds raw (uncancelled) interleaved SC16.  Never accepts more than Nt
    // samples for the current window.
    bool add_training_samples(const int16_t* iq, size_t n, std::string* error = nullptr);
    // Projects the fold, builds the new 288-entry tone table (and the
    // cross-fade table against the previous model) and commits it.
    FixedToneTrainingStatus finish_training(std::string* error = nullptr);
    bool training_active() const { return training_active_; }
    size_t training_samples_collected() const { return training_count_; }
    int64_t training_epoch() const { return training_epoch_; }

    const FixedToneCancelModel& model() const { return model_; }
    bool has_model() const { return model_.valid; }
    // Drops the model (gap / discontinuity / unsupported switch) without
    // touching the prepared base table.  Processing becomes a pass-through.
    void invalidate_model();
    // Clears training, model, tables and the cross-fade state.  Keeps the
    // prepared configuration and the immutable base phasor table.
    void reset();

    // --- processing (Apply) --------------------------------------------
    // start_abs is the absolute sample index of src[0] on the same axis as the
    // training epoch, so phase never depends on the read chunking.
    // Exactly one of src/dst aliasing (in place) or disjoint buffers is
    // supported; a partial overlap would silently corrupt the output.
    FixedToneProcessCounts copy_cancel(const int16_t* src,
                                       int16_t* dst,
                                       size_t n,
                                       int64_t start_abs) const;
    FixedToneProcessCounts cancel_in_place(int16_t* iq, size_t n, int64_t start_abs) const;

    // read-only views (QA / diagnostics / benchmark)
    const int16_t* tone_table() const { return lut_.data(); } // 2 * period
    const int16_t* crossfade_table() const { return crossfade_tone_.data(); }
    size_t crossfade_table_samples() const { return crossfade_active_; }
    int64_t crossfade_epoch() const { return crossfade_epoch_; }

    // Fixed nearest-even quantizer, independent of the process rounding mode.
    static bool quantize_tone(double value, int16_t* out);

    // Independent period resolution helper (also used by QA).
    static bool find_exact_period(double sample_rate,
                                  double tone_hz,
                                  size_t max_period,
                                  size_t* period,
                                  int* cycles,
                                  double* omega,
                                  std::string* error);

    // QA only: 0 = runtime dispatch (default), 1 = scalar, 2 = AVX2.
    void set_kernel_mode_for_testing(int mode) { kernel_mode_ = mode; }
    int kernel_mode() const { return kernel_mode_; }
    bool avx2_available() const { return avx2_available_; }
    // 1 = scalar, 2 = AVX2: the kernel copy_cancel() would use right now.
    int active_kernel() const;

private:
    void clear_state();

    FixedToneCancelConfig cfg_{};
    bool prepared_ = false;
    bool avx2_available_ = false;
    int kernel_mode_ = 0;

    size_t period_ = 0;
    int period_cycles_ = 0;
    double omega_ = 0.0;
    int64_t period_length_ = 0;
    size_t training_length_ = 0;
    size_t crossfade_length_ = 0;
    size_t max_fold_per_position_ = 0;

    // One-time unit phasor base (never regenerated per period).
    std::vector<double> base_cos_;
    std::vector<double> base_sin_;

    // Training fold (int32, exact for <= 65535 samples per position).  Stored
    // interleaved I/Q so a whole period folds as one contiguous accumulate.
    std::vector<int32_t> fold_;
    std::vector<int32_t> fold_scratch_;
    size_t training_count_ = 0;
    size_t fold_pos_ = 0;
    int64_t training_epoch_ = 0;
    bool training_active_ = false;

    FixedToneCancelModel model_{};
    std::vector<int16_t> lut_;             // 2 * period
    std::vector<int16_t> crossfade_tone_;  // 2 * crossfade_length
    size_t crossfade_active_ = 0;          // blend samples valid for this model
    int64_t crossfade_epoch_ = 0;
};

} // namespace core
} // namespace openuwb
