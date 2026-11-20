/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Bounded continuous fixed-frequency CW cancellation stage implementation.
 */

#include <openuwb/radio/fixed_tone_cancel_source.h>

#include <algorithm>
#include <chrono>
#include <cstring>
#include <iostream>
#include <limits>

namespace openuwb {
namespace pipeline {

namespace {

constexpr size_t kMaxConsecutiveEmptySourceReads = 64;
constexpr auto kWaitSlice = std::chrono::milliseconds(100);

uint64_t elapsed_ns(std::chrono::steady_clock::time_point a,
                    std::chrono::steady_clock::time_point b)
{
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(b - a).count());
}

void note_peak(std::atomic<uint64_t>& peak, uint64_t value)
{
    uint64_t cur = peak.load(std::memory_order_relaxed);
    while (value > cur &&
           !peak.compare_exchange_weak(
               cur, value, std::memory_order_relaxed, std::memory_order_relaxed)) {
    }
}

} // namespace

// One pre-allocated raw block.  Storage uses std::complex<int16_t> so the
// wrapped ISc16StreamSource can fill it directly; the core reads it through
// int16_t*, which is well defined because std::complex<T> is guaranteed to be
// laid out exactly like T[2] (interleaved real/imaginary).
struct FixedToneCancelSc16Source::Pending {
    std::vector<std::complex<int16_t>> samples;
    size_t count = 0;
    int64_t start_abs = 0;
    bool has_time_spec = false;
    double time_spec_s = 0.0;
    bool is_gap = false;
    int64_t gap_samples = 0;
    int error_code = 0;
};

struct FixedToneCancelSc16Source::Impl {
    std::unique_ptr<ISc16StreamSource> inner;
    core::FixedToneCancelConfig cfg;
    core::FixedToneCancelCore core;
    size_t block_samples = 0;
    size_t queue_blocks = 0;
    std::vector<std::unique_ptr<Pending>> blocks;
    // Read target used only while the ring is full, so the device/file keeps
    // being drained at full rate instead of stalling recv.
    std::vector<std::complex<int16_t>> discard;

    std::atomic<uint64_t> head{ 0 };
    std::atomic<uint64_t> tail{ 0 };
    std::atomic<bool> stop{ false };
    std::atomic<bool> done{ false };
    std::atomic<bool> fatal{ false };
    std::atomic<int> fatal_code{ 0 };
    std::atomic<bool> start_done{ false };
    std::atomic<bool> start_ok{ false };
    std::atomic<bool> prepared{ false };

    mutable std::mutex error_mutex;
    std::string error_text;

    std::mutex wait_mutex;
    std::condition_variable wait_cv;
    std::thread reader;

    std::atomic<uint64_t> source_read_calls{ 0 };
    std::atomic<uint64_t> source_samples{ 0 };
    std::atomic<uint64_t> source_error_reads{ 0 };
    std::atomic<uint64_t> queued_blocks{ 0 };
    std::atomic<uint64_t> queued_samples{ 0 };
    std::atomic<uint64_t> dropped_blocks{ 0 };
    std::atomic<uint64_t> dropped_samples{ 0 };
    std::atomic<uint64_t> overflow_drops{ 0 };
    std::atomic<uint64_t> max_pending_samples{ 0 };
    std::atomic<uint64_t> consumed_samples{ 0 };
    std::atomic<uint64_t> cancelled_samples{ 0 };
    std::atomic<uint64_t> bypassed_samples{ 0 };
    std::atomic<uint64_t> training_windows{ 0 };
    std::atomic<uint64_t> model_updates{ 0 };
    std::atomic<uint64_t> model_rejects{ 0 };
    std::atomic<uint64_t> partial_training_bypasses{ 0 };
    std::atomic<uint64_t> epoch_resets{ 0 };
    std::atomic<uint64_t> gaps_forwarded{ 0 };
    std::atomic<uint64_t> gap_samples_forwarded{ 0 };
    std::atomic<uint64_t> read_calls{ 0 };
    std::atomic<uint64_t> empty_reads{ 0 };
    std::atomic<uint64_t> estimate_ns{ 0 };
    std::atomic<uint64_t> cancel_ns{ 0 };
    std::atomic<uint64_t> reader_ns{ 0 };

    // Consumer-side state (only the window-assembler thread touches it).
    size_t cur_offset = 0;
    bool have_epoch = false;
    int64_t next_period_start = 0;
    bool training_pending = false;
    bool training_started = false;
    size_t training_folded = 0;
    int64_t training_epoch = 0;
    int64_t bypass_until = -1; // -1: no discontinuity pending
    bool bypass_forever = false;
    int64_t anchored_at = std::numeric_limits<int64_t>::min(); // last epoch anchor

    mutable FixedToneCancelStats view;
    std::string prepare_error;
    double applied_rate = 0.0;
};

// ---------------------------------------------------------------------------
// Construction / lifetime
// ---------------------------------------------------------------------------
FixedToneCancelSc16Source::FixedToneCancelSc16Source(std::unique_ptr<ISc16StreamSource> inner,
                                                     const core::FixedToneCancelConfig& cfg,
                                                     size_t raw_block_samples,
                                                     size_t raw_queue_blocks)
    : impl_(std::make_unique<Impl>())
{
    impl_->inner = std::move(inner);
    impl_->cfg = cfg;
    impl_->block_samples = raw_block_samples > 0 ? raw_block_samples : 1;
    impl_->queue_blocks = raw_queue_blocks > 0 ? raw_queue_blocks : 1;
}

FixedToneCancelSc16Source::~FixedToneCancelSc16Source()
{
    close();
}

bool FixedToneCancelSc16Source::open()
{
    auto& d = *impl_;
    if (!d.inner) {
        return false;
    }
    if (!d.inner->open()) {
        return false;
    }

    std::string err;
    if (!d.core.prepare(d.cfg, &err)) {
        d.prepare_error = err;
        std::cerr << "FixedToneCancelSc16Source: unsupported fixed-tone "
                     "configuration: "
                  << err << std::endl;
        return false;
    }
    d.prepared.store(true, std::memory_order_release);
    d.applied_rate = d.inner->actual_rx_rate();
    if (!(d.applied_rate > 0.0)) {
        d.applied_rate = d.cfg.sample_rate;
    }

    // The training window must fit in the raw ring, otherwise the model could
    // never be completed (the ring is the only place the raw training samples
    // live) and the consumer would wait forever.
    const size_t training = d.core.training_length_samples();
    if (training > d.block_samples * d.queue_blocks) {
        d.prepare_error = "raw ring (" + std::to_string(d.block_samples * d.queue_blocks) +
                          " samples) cannot hold the " + std::to_string(training) +
                          "-sample training window";
        std::cerr << "FixedToneCancelSc16Source: " << d.prepare_error << std::endl;
        d.prepared.store(false, std::memory_order_release);
        return false;
    }

    d.blocks.clear();
    d.blocks.reserve(d.queue_blocks);
    for (size_t i = 0; i < d.queue_blocks; ++i) {
        auto blk = std::make_unique<Pending>();
        blk->samples.resize(d.block_samples);
        d.blocks.push_back(std::move(blk));
    }
    d.discard.resize(d.block_samples);

    d.head.store(0, std::memory_order_relaxed);
    d.tail.store(0, std::memory_order_relaxed);
    d.stop.store(false, std::memory_order_relaxed);
    d.done.store(false, std::memory_order_relaxed);
    d.fatal.store(false, std::memory_order_relaxed);
    d.fatal_code.store(0, std::memory_order_relaxed);
    d.start_done.store(false, std::memory_order_relaxed);
    d.start_ok.store(false, std::memory_order_relaxed);
    d.cur_offset = 0;
    d.have_epoch = false;
    d.next_period_start = 0;
    d.training_pending = false;
    d.training_started = false;
    d.training_folded = 0;
    d.training_epoch = 0;
    d.bypass_until = -1;
    d.bypass_forever = false;
    d.anchored_at = std::numeric_limits<int64_t>::min();
    d.core.reset();
    return true;
}

bool FixedToneCancelSc16Source::start_stream()
{
    auto& d = *impl_;
    if (!d.prepared.load(std::memory_order_acquire) || !d.inner) {
        return false;
    }
    if (d.reader.joinable()) {
        return true; // already started
    }
    d.reader = std::thread(&FixedToneCancelSc16Source::reader_loop, this);

    // Wait (bounded) for the wrapped start_stream() result so a device start
    // failure is reported synchronously, exactly like the unwrapped source.
    std::unique_lock<std::mutex> lk(d.wait_mutex);
    const bool resolved = d.wait_cv.wait_for(lk, std::chrono::seconds(10), [&d] {
        return d.start_done.load(std::memory_order_acquire);
    });
    if (!resolved) {
        return true; // still opening: the reader will report a later failure
    }
    return d.start_ok.load(std::memory_order_acquire);
}

void FixedToneCancelSc16Source::request_stop()
{
    auto& d = *impl_;
    d.stop.store(true, std::memory_order_release);
    notify_waiters();
}

void FixedToneCancelSc16Source::close()
{
    auto& d = *impl_;
    request_stop();
    if (d.reader.joinable()) {
        d.reader.join();
    }
    if (d.inner) {
        d.inner->close();
    }
    d.done.store(true, std::memory_order_release);
    notify_waiters();
    d.prepared.store(false, std::memory_order_release);
}

void FixedToneCancelSc16Source::notify_waiters()
{
    auto& d = *impl_;
    {
        std::lock_guard<std::mutex> lk(d.wait_mutex);
    }
    d.wait_cv.notify_all();
}

void FixedToneCancelSc16Source::set_fatal(int code, const std::string& msg)
{
    auto& d = *impl_;
    {
        std::lock_guard<std::mutex> lk(d.error_mutex);
        if (d.error_text.empty()) {
            d.error_text = msg.empty() ? std::string("wrapped source failed") : msg;
        }
    }
    d.fatal_code.store(code, std::memory_order_relaxed);
    d.fatal.store(true, std::memory_order_release);
    notify_waiters();
}

// ---------------------------------------------------------------------------
// Raw reader thread: drain the wrapped source at full rate into the bounded
// ring.  It never blocks on the consumer: a full ring drops whole blocks and
// records the exact number of discarded samples for the next gap marker.
// ---------------------------------------------------------------------------
void FixedToneCancelSc16Source::reader_loop()
{
    auto& d = *impl_;

    const bool started = d.inner->start_stream();
    if (!started) {
        // Publish the failure BEFORE the start result so start_stream() never
        // returns false while fatal_error() still looks healthy.
        set_fatal(0, "wrapped source start_stream() failed");
        d.start_ok.store(false, std::memory_order_release);
        d.start_done.store(true, std::memory_order_release);
        d.done.store(true, std::memory_order_release);
        notify_waiters();
        return;
    }
    d.start_ok.store(true, std::memory_order_release);
    d.start_done.store(true, std::memory_order_release);
    notify_waiters();

    bool have_expected = false;
    int64_t expected_abs = 0;
    bool next_is_gap = false;
    int64_t next_gap_samples = 0;
    size_t consecutive_empty = 0;

    while (!d.stop.load(std::memory_order_acquire)) {
        const uint64_t h = d.head.load(std::memory_order_relaxed);
        const uint64_t t = d.tail.load(std::memory_order_acquire);
        Pending* blk = nullptr;
        std::complex<int16_t>* target = nullptr;
        if (h - t < d.queue_blocks) {
            blk = d.blocks[static_cast<size_t>(h % d.queue_blocks)].get();
            target = blk->samples.data();
        } else {
            // Full: keep recv-ing into the discard target, drop the block.
            target = d.discard.data();
        }

        StreamReadMeta meta;
        const auto t_read = std::chrono::steady_clock::now();
        const size_t n = d.inner->read_samples(target, d.block_samples, meta);
        d.reader_ns.fetch_add(elapsed_ns(t_read, std::chrono::steady_clock::now()),
                              std::memory_order_relaxed);
        d.source_read_calls.fetch_add(1, std::memory_order_relaxed);
        d.source_samples.fetch_add(n, std::memory_order_relaxed);

        if (meta.is_gap) {
            // The wrapped source already said how much it lost; the absolute
            // index jump is the same event, so take the larger of the two
            // instead of summing them (that would double-count the gap).
            next_is_gap = true;
            const int64_t delta = (have_expected && meta.start_sample_abs != expected_abs)
                                      ? (meta.start_sample_abs - expected_abs)
                                      : 0;
            next_gap_samples += std::max<int64_t>(meta.gap_samples, delta > 0 ? delta : 0);
        } else if (have_expected && meta.start_sample_abs != expected_abs) {
            // The sample axis jumped without an error flag: the next output must
            // carry an explicit gap and the model must not span the jump.
            const int64_t delta = meta.start_sample_abs - expected_abs;
            if (delta > 0) {
                next_is_gap = true;
                next_gap_samples += delta;
            }
        }
        if (n > 0) {
            have_expected = true;
            expected_abs = meta.start_sample_abs + static_cast<int64_t>(n);
            consecutive_empty = 0;
        }

        if (n > 0) {
            if (blk != nullptr) {
                blk->count = n;
                blk->start_abs = meta.start_sample_abs;
                blk->has_time_spec = meta.has_time_spec;
                blk->time_spec_s = meta.time_spec_s;
                blk->is_gap = next_is_gap;
                blk->gap_samples = next_gap_samples;
                blk->error_code = meta.error_code;
                next_is_gap = false;
                next_gap_samples = 0;
                d.head.store(h + 1, std::memory_order_release);
                d.queued_blocks.fetch_add(1, std::memory_order_relaxed);
                const uint64_t queued =
                    d.queued_samples.fetch_add(n, std::memory_order_relaxed) + n;
                const uint64_t consumed =
                    d.consumed_samples.load(std::memory_order_relaxed);
                if (queued > consumed) {
                    note_peak(d.max_pending_samples, queued - consumed);
                }
                notify_waiters();
            } else {
                d.dropped_blocks.fetch_add(1, std::memory_order_relaxed);
                d.dropped_samples.fetch_add(n, std::memory_order_relaxed);
                d.overflow_drops.fetch_add(1, std::memory_order_relaxed);
                next_is_gap = true;
                next_gap_samples += static_cast<int64_t>(n);
            }
            continue;
        }

        if (meta.error_code != 0) {
            d.source_error_reads.fetch_add(1, std::memory_order_relaxed);
        }
        std::string inner_error;
        if (d.inner->fatal_error(&inner_error)) {
            set_fatal(meta.error_code, inner_error);
            break;
        }
        if (meta.is_eof || d.inner->is_eof()) {
            break;
        }
        // No samples and no EOF: a bounded number of retries, then fail loudly
        // instead of spinning this thread forever.
        if (++consecutive_empty >= kMaxConsecutiveEmptySourceReads) {
            set_fatal(meta.error_code,
                      "wrapped source returned no samples and no EOF for " +
                          std::to_string(consecutive_empty) + " consecutive reads");
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    d.done.store(true, std::memory_order_release);
    notify_waiters();
}

// ---------------------------------------------------------------------------
// Consumer side
// ---------------------------------------------------------------------------
double FixedToneCancelSc16Source::actual_rx_rate() const
{
    const auto& d = *impl_;
    if (d.inner) {
        const double rate = d.inner->actual_rx_rate();
        if (rate > 0.0) {
            return rate;
        }
    }
    return d.applied_rate;
}

bool FixedToneCancelSc16Source::is_eof() const
{
    const auto& d = *impl_;
    return d.done.load(std::memory_order_acquire) &&
           !(d.tail.load(std::memory_order_acquire) < d.head.load(std::memory_order_acquire));
}

bool FixedToneCancelSc16Source::fatal_error(std::string* msg) const
{
    const auto& d = *impl_;
    if (!d.fatal.load(std::memory_order_acquire)) {
        if (msg) {
            msg->clear();
        }
        return false;
    }
    if (msg) {
        std::lock_guard<std::mutex> lk(d.error_mutex);
        *msg = d.error_text;
    }
    return true;
}

const core::FixedToneCancelCore& FixedToneCancelSc16Source::tone_core() const
{
    return impl_->core;
}

const FixedToneCancelStats& FixedToneCancelSc16Source::stats() const
{
    auto& d = *impl_;
    auto& v = d.view;
    v.enabled = d.cfg.enable;
    v.prepare_error = d.prepare_error;
    v.source_read_calls = d.source_read_calls.load(std::memory_order_relaxed);
    v.source_samples = d.source_samples.load(std::memory_order_relaxed);
    v.source_error_reads = d.source_error_reads.load(std::memory_order_relaxed);
    v.queued_blocks = d.queued_blocks.load(std::memory_order_relaxed);
    v.queued_samples = d.queued_samples.load(std::memory_order_relaxed);
    v.dropped_blocks = d.dropped_blocks.load(std::memory_order_relaxed);
    v.dropped_samples = d.dropped_samples.load(std::memory_order_relaxed);
    v.overflow_drops = d.overflow_drops.load(std::memory_order_relaxed);
    v.max_pending_samples = d.max_pending_samples.load(std::memory_order_relaxed);
    v.consumed_samples = d.consumed_samples.load(std::memory_order_relaxed);
    v.pending_samples =
        v.queued_samples >= v.consumed_samples ? v.queued_samples - v.consumed_samples : 0;
    v.cancelled_samples = d.cancelled_samples.load(std::memory_order_relaxed);
    v.bypassed_samples = d.bypassed_samples.load(std::memory_order_relaxed);
    v.training_windows = d.training_windows.load(std::memory_order_relaxed);
    v.model_updates = d.model_updates.load(std::memory_order_relaxed);
    v.model_rejects = d.model_rejects.load(std::memory_order_relaxed);
    v.partial_training_bypasses =
        d.partial_training_bypasses.load(std::memory_order_relaxed);
    v.epoch_resets = d.epoch_resets.load(std::memory_order_relaxed);
    v.gaps_forwarded = d.gaps_forwarded.load(std::memory_order_relaxed);
    v.gap_samples_forwarded = d.gap_samples_forwarded.load(std::memory_order_relaxed);
    v.read_calls = d.read_calls.load(std::memory_order_relaxed);
    v.empty_reads = d.empty_reads.load(std::memory_order_relaxed);
    v.model_generation = d.core.model().generation;
    v.model_amplitude = d.core.model().amplitude;
    v.model_phase_radians = d.core.model().phase_radians;
    v.model_epoch = d.core.model().epoch;
    v.estimate_us = 1e-3 * static_cast<double>(d.estimate_ns.load(std::memory_order_relaxed));
    v.cancel_us = 1e-3 * static_cast<double>(d.cancel_ns.load(std::memory_order_relaxed));
    v.reader_us = 1e-3 * static_cast<double>(d.reader_ns.load(std::memory_order_relaxed));
    return v;
}

FixedToneCancelAppliedConfig FixedToneCancelSc16Source::applied_config() const
{
    const auto& d = *impl_;
    FixedToneCancelAppliedConfig out;
    out.enabled = d.cfg.enable;
    out.sample_rate = d.cfg.sample_rate;
    out.tone_frequency_hz = d.cfg.tone_frequency_hz;
    out.frequency_ratio =
        (d.cfg.sample_rate > 0.0) ? (d.cfg.tone_frequency_hz / d.cfg.sample_rate) : 0.0;
    out.period_samples = d.core.period_samples();
    out.period_cycles = d.core.period_cycles();
    out.period_length_samples = d.core.period_length_samples();
    out.training_samples = d.core.training_length_samples();
    out.crossfade_samples = d.core.crossfade_samples();
    out.raw_block_samples = d.block_samples;
    out.raw_queue_blocks = d.queue_blocks;
    out.active_kernel = d.core.active_kernel();
    return out;
}

bool FixedToneCancelSc16Source::wait_for_data(std::unique_lock<std::mutex>& lk)
{
    auto& d = *impl_;
    auto has_data = [&d] {
        return d.tail.load(std::memory_order_acquire) <
               d.head.load(std::memory_order_acquire);
    };
    if (has_data()) {
        return true;
    }
    if (d.fatal.load(std::memory_order_acquire) ||
        d.stop.load(std::memory_order_acquire)) {
        return false;
    }
    if (d.done.load(std::memory_order_acquire)) {
        return false;
    }
    if (!d.wait_cv.wait_for(lk, kWaitSlice, [&d, &has_data] {
            return has_data() || d.done.load(std::memory_order_acquire) ||
                   d.fatal.load(std::memory_order_acquire) ||
                   d.stop.load(std::memory_order_acquire);
        })) {
        // Bounded wait: a source that is genuinely silent must not hang the
        // window assembler, and stop() must always be observable.
        return false;
    }
    return has_data();
}

size_t FixedToneCancelSc16Source::read_samples(std::complex<int16_t>* dest,
                                               size_t max_samples,
                                               StreamReadMeta& meta)
{
    auto& d = *impl_;
    meta = StreamReadMeta{};
    if (!d.prepared.load(std::memory_order_acquire)) {
        meta.is_eof = true;
        return 0;
    }
    d.read_calls.fetch_add(1, std::memory_order_relaxed);
    if (dest == nullptr || max_samples == 0) {
        meta.is_eof = is_eof();
        return 0;
    }

    {
        std::unique_lock<std::mutex> lk(d.wait_mutex);
        if (!wait_for_data(lk)) {
            if (d.fatal.load(std::memory_order_acquire)) {
                meta.error_code = d.fatal_code.load(std::memory_order_relaxed);
            }
            meta.is_eof = is_eof();
            if (!meta.is_eof && !d.fatal.load(std::memory_order_acquire)) {
                d.empty_reads.fetch_add(1, std::memory_order_relaxed);
            }
            return 0;
        }
    }

    const size_t q = d.queue_blocks;
    auto block_at = [&d, q](uint64_t index) -> Pending* {
        return d.blocks[static_cast<size_t>(index % q)].get();
    };
    // One snapshot of the ring bookkeeping that answers BOTH questions the
    // training state machine needs: how far the contiguous raw run reaches from
    // from_abs, and how far data has been published at all.  They must come from
    // the same head/tail observation: taking them from two separate loads makes
    // a still-running stream look like a discontinuity (the reader publishes
    // more in between), which would bypass and re-anchor a perfectly good epoch.
    struct RunSpan {
        int64_t contiguous_end = 0;
        int64_t published_end = std::numeric_limits<int64_t>::min();
    };
    auto run_span = [&d, &block_at](int64_t from_abs) -> RunSpan {
        const uint64_t t = d.tail.load(std::memory_order_acquire);
        const uint64_t h = d.head.load(std::memory_order_acquire);
        RunSpan span;
        if (h != t) {
            const Pending* last = block_at(h - 1);
            span.published_end = last->start_abs + static_cast<int64_t>(last->count);
        }
        bool started = false;
        int64_t expect = from_abs;
        for (uint64_t i = t; i < h; ++i) {
            const Pending* b = block_at(i);
            const size_t off = (i == t) ? d.cur_offset : 0;
            const int64_t bstart = b->start_abs + static_cast<int64_t>(off);
            const int64_t bend = b->start_abs + static_cast<int64_t>(b->count);
            if (bend <= from_abs) {
                continue;
            }
            if (!started) {
                if (bstart > from_abs) {
                    span.contiguous_end = from_abs;
                    return span;
                }
                started = true;
                expect = std::max<int64_t>(from_abs, bstart);
            } else if (b->is_gap || bstart != expect) {
                span.contiguous_end = expect;
                return span;
            }
            expect = bend;
        }
        span.contiguous_end = expect;
        return span;
    };
    auto contiguous_end_from = [&run_span](int64_t from_abs) -> int64_t {
        return run_span(from_abs).contiguous_end;
    };

    uint64_t t0 = d.tail.load(std::memory_order_acquire);
    Pending* cur_block = block_at(t0);
    int64_t cur = cur_block->start_abs + static_cast<int64_t>(d.cur_offset);

    if (!d.have_epoch) {
        d.have_epoch = true;
        d.next_period_start = cur + d.core.period_length_samples();
        d.training_pending = true;
        d.training_started = false;
        d.training_folded = 0;
        d.training_epoch = cur;
        d.anchored_at = cur;
    }

    // A discontinuity in the raw stream (dropped block, UHD overflow, absolute
    // index jump) invalidates the current phase origin: the tone model is only
    // valid on a continuous sample axis.  Drop it and re-establish the epoch at
    // the first sample after the gap, exactly like the start of the run.
    const bool at_gap_boundary =
        (cur == cur_block->start_abs && cur_block->is_gap && cur != d.anchored_at);
    if (at_gap_boundary) {
        d.core.invalidate_model();
        d.next_period_start = cur + d.core.period_length_samples();
        d.training_pending = true;
        d.training_started = false;
        d.training_folded = 0;
        d.training_epoch = cur;
        d.bypass_until = -1;
        d.bypass_forever = false;
        d.anchored_at = cur;
        d.epoch_resets.fetch_add(1, std::memory_order_relaxed);
    } else if (d.bypass_until >= 0 && cur >= d.bypass_until) {
        // Reached the far side of a discontinuity: the model must be rebuilt
        // from a fresh epoch here, never learned across the gap.
        d.bypass_until = -1;
        d.core.invalidate_model();
        d.next_period_start = cur + d.core.period_length_samples();
        d.training_pending = true;
        d.training_started = false;
        d.training_folded = 0;
        d.training_epoch = cur;
        d.anchored_at = cur;
        d.epoch_resets.fetch_add(1, std::memory_order_relaxed);
    } else if (cur == d.next_period_start) {
        d.next_period_start += d.core.period_length_samples();
        d.training_pending = true;
        d.training_started = false;
        d.training_folded = 0;
        d.training_epoch = cur;
    }

    // --- training state machine -------------------------------------------
    // The fold only ever reads published raw blocks.  Nothing is rewritten, so
    // the training window is still available afterwards and is emitted with
    // the model it produced ("estimate first, then replay the window").
    auto fold_range = [&](int64_t from_abs, size_t total) -> bool {
        const uint64_t first = d.tail.load(std::memory_order_acquire);
        const uint64_t h = d.head.load(std::memory_order_acquire);
        int64_t pos_abs = from_abs;
        size_t left = total;
        for (uint64_t idx = first; idx < h && left > 0; ++idx) {
            const Pending* b = block_at(idx);
            const size_t off = (idx == first) ? d.cur_offset : 0;
            const int64_t bstart = b->start_abs + static_cast<int64_t>(off);
            const int64_t bend = b->start_abs + static_cast<int64_t>(b->count);
            if (bend <= pos_abs) {
                continue;
            }
            if (pos_abs < bstart) {
                return false; // contiguity was checked before folding
            }
            const size_t skip = static_cast<size_t>(pos_abs - bstart);
            const size_t avail = b->count - off - skip;
            const size_t take = std::min<size_t>(avail, left);
            std::string err;
            if (!d.core.add_training_samples(
                    reinterpret_cast<const int16_t*>(b->samples.data()) + 2 * (off + skip),
                    take,
                    &err)) {
                return false;
            }
            left -= take;
            pos_abs += static_cast<int64_t>(take);
        }
        return left == 0;
    };

    if (d.training_pending && !d.bypass_forever) {
        const size_t want = d.core.training_length_samples();
        if (!d.training_started) {
            std::string err;
            if (!d.core.begin_training(cur, &err)) {
                // Should not happen for a prepared core; bypass this period.
                d.training_pending = false;
                d.core.invalidate_model();
                d.model_rejects.fetch_add(1, std::memory_order_relaxed);
            } else {
                d.training_started = true;
                d.training_folded = 0;
                d.training_windows.fetch_add(1, std::memory_order_relaxed);
            }
        }
        if (d.training_started) {
            bool aborted = false;
            while (d.training_folded < want) {
                const int64_t from = cur + static_cast<int64_t>(d.training_folded);
                const int64_t run_end = contiguous_end_from(from);
                if (run_end <= from) {
                    if (d.done.load(std::memory_order_acquire)) {
                        // End of stream before the window filled: emit the
                        // remaining tail unmodified.
                        d.partial_training_bypasses.fetch_add(1,
                                                              std::memory_order_relaxed);
                        aborted = true;
                        break;
                    }
                    std::unique_lock<std::mutex> lk(d.wait_mutex);
                    d.wait_cv.wait_for(lk, kWaitSlice, [&d, &from, &contiguous_end_from] {
                        return d.done.load(std::memory_order_acquire) ||
                               d.stop.load(std::memory_order_acquire) ||
                               d.fatal.load(std::memory_order_acquire) ||
                               contiguous_end_from(from) > from;
                    });
                    if (d.stop.load(std::memory_order_acquire) ||
                        d.fatal.load(std::memory_order_acquire)) {
                        d.partial_training_bypasses.fetch_add(1,
                                                              std::memory_order_relaxed);
                        aborted = true;
                        break;
                    }
                    continue;
                }
                const size_t cnt = static_cast<size_t>(std::min<int64_t>(
                    run_end - from, static_cast<int64_t>(want - d.training_folded)));
                const auto t_fold = std::chrono::steady_clock::now();
                const bool folded = fold_range(from, cnt);
                d.estimate_ns.fetch_add(
                    elapsed_ns(t_fold, std::chrono::steady_clock::now()),
                    std::memory_order_relaxed);
                if (!folded) {
                    d.model_rejects.fetch_add(1, std::memory_order_relaxed);
                    aborted = true;
                    break;
                }
                d.training_folded += cnt;
                if (d.training_folded < want) {
                    // Same-snapshot comparison: a run that simply has not
                    // received more data yet ends exactly at published_end and
                    // must be waited for, not mistaken for a gap.
                    const RunSpan next = run_span(from);
                    if (next.contiguous_end < next.published_end) {
                        d.bypass_until = next.contiguous_end;
                        d.model_rejects.fetch_add(1, std::memory_order_relaxed);
                        aborted = true;
                        break;
                    }
                }
            }
            if (!aborted && d.training_folded >= want) {
                const auto t_proj = std::chrono::steady_clock::now();
                std::string err;
                const auto status = d.core.finish_training(&err);
                d.estimate_ns.fetch_add(
                    elapsed_ns(t_proj, std::chrono::steady_clock::now()),
                    std::memory_order_relaxed);
                d.training_pending = false;
                d.training_started = false;
                if (status == core::FixedToneTrainingStatus::Ok) {
                    d.model_updates.fetch_add(1, std::memory_order_relaxed);
                } else {
                    d.core.invalidate_model();
                    d.model_rejects.fetch_add(1, std::memory_order_relaxed);
                }
            } else {
                // No usable model for this period: bypass it (and, unless a
                // re-anchor point is pending, the rest of the stream).
                d.training_pending = false;
                d.training_started = false;
                d.core.invalidate_model();
                if (d.bypass_until < 0 &&
                    (d.done.load(std::memory_order_acquire) ||
                     d.stop.load(std::memory_order_acquire) ||
                     d.fatal.load(std::memory_order_acquire))) {
                    d.bypass_forever = true;
                }
            }
        }
    }

    // --- produce one contiguous region ------------------------------------
    const int64_t run_end = contiguous_end_from(cur);
    if (run_end <= cur) {
        // No contiguous sample available right now (a bounded wait already
        // succeeded, so this only covers a just-detected discontinuity).
        meta.is_eof = is_eof();
        d.empty_reads.fetch_add(1, std::memory_order_relaxed);
        return 0;
    }
    int64_t limit = std::min<int64_t>(run_end, d.next_period_start);
    if (d.bypass_until > cur) {
        limit = std::min(limit, d.bypass_until);
    }
    const size_t count = std::min<size_t>(static_cast<size_t>(limit - cur), max_samples);
    if (count == 0) {
        meta.is_eof = is_eof();
        return 0;
    }

    // Gap marker: only when the region starts exactly at a block boundary that
    // the reader flagged, so the discontinuity lands on the correct sample.
    if (cur == cur_block->start_abs && cur_block->is_gap) {
        meta.is_gap = true;
        meta.gap_samples = cur_block->gap_samples;
        d.gaps_forwarded.fetch_add(1, std::memory_order_relaxed);
        d.gap_samples_forwarded.fetch_add(
            static_cast<uint64_t>(cur_block->gap_samples > 0 ? cur_block->gap_samples : 0),
            std::memory_order_relaxed);
    }
    meta.start_sample_abs = cur;
    if (cur_block->has_time_spec) {
        const double rate = actual_rx_rate();
        if (rate > 0.0) {
            meta.has_time_spec = true;
            meta.time_spec_s =
                cur_block->time_spec_s +
                static_cast<double>(cur - cur_block->start_abs) / rate;
        }
    }

    const auto t_cancel = std::chrono::steady_clock::now();
    size_t produced = 0;
    int64_t pos = cur;
    while (produced < count) {
        const uint64_t idx = d.tail.load(std::memory_order_acquire);
        Pending* b = block_at(idx);
        const size_t off = d.cur_offset;
        const size_t avail = b->count - off;
        const size_t take = std::min(avail, count - produced);
        const auto counts = d.core.copy_cancel(
            reinterpret_cast<const int16_t*>(b->samples.data()) + 2 * off,
            reinterpret_cast<int16_t*>(dest) + 2 * produced,
            take,
            pos);
        d.cancelled_samples.fetch_add(counts.cancelled, std::memory_order_relaxed);
        d.bypassed_samples.fetch_add(counts.bypassed, std::memory_order_relaxed);
        produced += take;
        pos += static_cast<int64_t>(take);
        d.cur_offset += take;
        if (d.cur_offset >= b->count) {
            d.cur_offset = 0;
            d.tail.store(idx + 1, std::memory_order_release);
        }
    }
    d.consumed_samples.fetch_add(produced, std::memory_order_relaxed);
    d.cancel_ns.fetch_add(elapsed_ns(t_cancel, std::chrono::steady_clock::now()),
                          std::memory_order_relaxed);

    meta.is_eof = is_eof();
    return produced;
}

} // namespace pipeline
} // namespace openuwb
