/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Bounded, continuous fixed-frequency CW cancellation stage for the native
 * SC16 RX chain.
 *
 *   UHD/replay continuous recv
 *     -> dedicated raw-reader thread -> bounded raw SC16 block ring
 *     -> this decorator's read_samples() (model state machine + fused
 *        copy-and-cancel into the caller's storage)
 *     -> existing SC16 window assembly -> existing DSP
 *
 * It sits BEFORE the window assembler on purpose: the training window between
 * two scheduled windows is still in the stream there, so the model can be
 * re-estimated every period without depending on packet arrival.  The source
 * keeps recv-ing while the consumer waits for the 0.5 ms training window.
 *
 * Ownership: the raw ring always holds untouched device samples, and the
 * caller's storage is filled by `copy_cancel` (one load / saturating subtract
 * / store).  There is no second "clean" buffer and no in-place rewrite of the
 * raw ring, so the training fold can never see already-cancelled samples and
 * the same raw data cannot be cancelled twice.  Overlapping scheduled windows
 * simply read the same already-clean storage again.
 */

#pragma once

#include <openuwb/support/api.h>
#include <openuwb/dsp/fixed_tone_cancel_core.h>
// Minimal source contract only: including the whole pipeline header here
// would force every tone-cancel consumer to pull in the SIC / radar / demod
// consumer headers.
#include <openuwb/radio/sc16_stream_source.h>

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace gr {
namespace uwb {
namespace pipeline {

// Configuration that was actually applied (the echoed sample rate is the one
// the core resolved, not necessarily the requested default).
struct FixedToneCancelAppliedConfig {
    bool enabled = false;
    double sample_rate = 0.0;
    double tone_frequency_hz = 0.0;
    double frequency_ratio = 0.0;
    size_t period_samples = 0;
    int period_cycles = 0;
    int64_t period_length_samples = 0;
    size_t training_samples = 0;
    size_t crossfade_samples = 0;
    size_t raw_block_samples = 0;
    size_t raw_queue_blocks = 0;
    int active_kernel = 0; // 1 = scalar, 2 = AVX2
};

struct FixedToneCancelStats {
    bool enabled = false;
    std::string prepare_error; // non-empty when the stage could not be prepared

    // Layer 1: hardware/file source reads (NOT the decorator read calls).
    uint64_t source_read_calls = 0;
    uint64_t source_samples = 0;
    uint64_t source_error_reads = 0;

    // Layer 2: raw block ring.
    uint64_t queued_blocks = 0;
    uint64_t queued_samples = 0;
    uint64_t dropped_blocks = 0;
    uint64_t dropped_samples = 0;
    uint64_t consumed_samples = 0; // returned to the window assembler
    uint64_t pending_samples = 0;  // queued - consumed (training staging counts)
    uint64_t max_pending_samples = 0; // backlog watermark (peak pending)
    uint64_t overflow_drops = 0;   // ring-full events

    // Cancellation bookkeeping.
    uint64_t cancelled_samples = 0;
    uint64_t bypassed_samples = 0;
    uint64_t training_windows = 0;
    uint64_t model_updates = 0;
    uint64_t model_rejects = 0;
    uint64_t partial_training_bypasses = 0;
    uint64_t epoch_resets = 0;
    uint64_t gaps_forwarded = 0;
    uint64_t gap_samples_forwarded = 0;

    // Decorator read calls (aggregate view; per-call detail is in the CSV/JSON).
    uint64_t read_calls = 0;
    uint64_t empty_reads = 0;

    uint64_t model_generation = 0;
    double model_amplitude = 0.0;
    double model_phase_radians = 0.0;
    int64_t model_epoch = 0;

    // CPU time in this stage (thread-local clocks, summed).
    double estimate_us = 0.0;   // fold + projection + table build
    double cancel_us = 0.0;     // fused copy-and-cancel
    double reader_us = 0.0;     // wrapped source read (reader thread)
};

class UWB_API FixedToneCancelSc16Source : public ISc16StreamSource {
public:
    FixedToneCancelSc16Source(std::unique_ptr<ISc16StreamSource> inner,
                              const core::FixedToneCancelConfig& cfg,
                              size_t raw_block_samples = core::kFixedToneDefaultRawBlockSamples,
                              size_t raw_queue_blocks = core::kFixedToneDefaultRawQueueBlocks);
    ~FixedToneCancelSc16Source() override;

    // open() only allocates and prepares: the wrapped source is opened but its
    // start_stream() runs later, from the reader thread, so no device can queue
    // samples while the DSP workers are still initialising.
    bool open() override;
    // Starts the raw reader thread and blocks (bounded) for its start_stream()
    // result, so a device start failure is reported synchronously.
    bool start_stream() override;
    void close() override;
    void request_stop() override;

    size_t read_samples(std::complex<int16_t>* dest,
                        size_t max_samples,
                        StreamReadMeta& meta) override;
    double actual_rx_rate() const override;
    bool is_eof() const override;
    bool fatal_error(std::string* msg) const override;

    const FixedToneCancelStats& stats() const;
    const core::FixedToneCancelCore& tone_core() const;
    FixedToneCancelAppliedConfig applied_config() const;

private:
    struct Pending;
    struct Impl;
    std::unique_ptr<Impl> impl_;

    void reader_loop();
    void set_fatal(int code, const std::string& msg);
    // Wait (bounded) for at least one published raw block.
    bool wait_for_data(std::unique_lock<std::mutex>& lk);
    void notify_waiters();
};

} // namespace pipeline
} // namespace uwb
} // namespace gr
