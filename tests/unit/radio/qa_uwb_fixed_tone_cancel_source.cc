/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * QA for the fixed-frequency CW cancellation SOURCE stage and its pipeline
 * integration.
 *
 * The stage owns a raw reader thread and a bounded block ring, so the tests
 * cover the stream contract rather than only the maths:
 *   - training staging, per-period model updates, end-of-stream partial bypass
 *   - gap propagation at the exact output boundary + epoch reset
 *   - ring overflow: whole-block drops, accumulated gap, sample conservation
 *   - fatal errors are never disguised as short reads / EOF
 *   - start/stop, and the pipeline path: disabled == bit-identical, enabled ==
 *     the independently expected cancelled IQ
 */

#include <boost/test/unit_test.hpp>

#include <gnuradio/uwb/uwb_fixed_tone_cancel_source.h>
#include <gnuradio/uwb/uwb_uhd_sc16_pipeline.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <vector>

using namespace gr::uwb::pipeline;
namespace core = gr::uwb::core;

namespace {

constexpr double kRate = 288000.0;             // 288 * 1000
constexpr double kTone = -91000.0;             // exactly -91/288 * fs
constexpr size_t kTonePeriod = 288;
constexpr double kOmega = 2.0 * M_PI * double(-91) / double(kTonePeriod);
constexpr size_t kPeriodLength = 2880;         // 10 ms at 288 kHz
constexpr size_t kTraining = 576;              // 2 ms
constexpr size_t kPipelinePeriod = 20000;      // longer than the capture: one model per run
constexpr size_t kCrossfade = 64;
constexpr double kAmplitude = 6000.0;
constexpr double kPhase = 0.35;

// Deterministic per-sample pseudo-noise, independent of the read chunking.
int16_t noise_at(int64_t i)
{
    const uint32_t u = static_cast<uint32_t>(i) * 1103515245u + 12345u;
    return static_cast<int16_t>(static_cast<int>((u >> 16) & 0x3fu) - 32);
}

std::complex<int16_t> sample_at(int64_t i)
{
    const double re = kAmplitude * std::cos(kOmega * static_cast<double>(i) + kPhase);
    const double im = kAmplitude * std::sin(kOmega * static_cast<double>(i) + kPhase);
    return std::complex<int16_t>(
        static_cast<int16_t>(std::lround(re) + noise_at(i)),
        static_cast<int16_t>(std::lround(im) + noise_at(i + 7919)));
}

struct FakeOptions {
    int64_t total = 0;
    std::vector<size_t> read_sizes;
    int64_t gap_at = -1;       // logical index of the first post-gap sample
    int64_t gap_samples = 0;   // hidden absolute samples
    int64_t fail_at = -1;      // fatal error once pos reaches this index
    bool start_ok = true;
    bool idle = false;         // return 0 (no EOF) until stop is requested
    // Deliver at the configured sample rate instead of as fast as possible.
    // Without pacing a fake source finishes before a slow consumer advances the
    // ring, so every drop would land at the tail and never become an interior
    // gap marker.
    bool paced = false;
};

class FakeToneSource : public ISc16StreamSource
{
public:
    explicit FakeToneSource(FakeOptions opts) : opts_(std::move(opts)) {}

    bool open() override
    {
        pos_ = 0;
        eof_ = false;
        gap_done_ = false;
        fatal_ = false;
        closed_ = false;
        start_time_ = std::chrono::steady_clock::now();
        return true;
    }
    bool start_stream() override
    {
        starts_++;
        return opts_.start_ok;
    }
    void request_stop() override { stop_.store(true); }
    void close() override
    {
        closed_ = true;
        stop_.store(true);
    }
    double actual_rx_rate() const override { return kRate; }
    bool is_eof() const override { return eof_; }
    bool fatal_error(std::string* msg) const override
    {
        if (!fatal_) {
            if (msg) {
                msg->clear();
            }
            return false;
        }
        if (msg) {
            *msg = "fake source fatal error";
        }
        return true;
    }

    int64_t absolute_of(int64_t logical) const
    {
        if (opts_.gap_at >= 0 && logical >= opts_.gap_at) {
            return logical + opts_.gap_samples;
        }
        return logical;
    }

    size_t read_samples(std::complex<int16_t>* dest,
                        size_t max_samples,
                        StreamReadMeta& meta) override
    {
        meta = StreamReadMeta{};
        if (opts_.idle && !stop_.load()) {
            meta.start_sample_abs = absolute_of(pos_);
            return 0;
        }
        if (opts_.fail_at >= 0 && pos_ >= opts_.fail_at) {
            fatal_ = true;
            meta.error_code = 5;
            return 0;
        }
        if (pos_ >= opts_.total) {
            eof_ = true;
            meta.start_sample_abs = absolute_of(pos_);
            meta.is_eof = true;
            return 0;
        }
        size_t n = max_samples;
        if (!opts_.read_sizes.empty()) {
            n = std::min(n, opts_.read_sizes[static_cast<size_t>(calls_++ % opts_.read_sizes.size())]);
        }
        if (opts_.gap_at >= 0 && pos_ < opts_.gap_at) {
            n = std::min<size_t>(n, static_cast<size_t>(opts_.gap_at - pos_));
        }
        n = std::min<size_t>(n, static_cast<size_t>(opts_.total - pos_));
        if (n == 0) {
            meta.start_sample_abs = absolute_of(pos_);
            return 0;
        }
        meta.start_sample_abs = absolute_of(pos_);
        meta.has_time_spec = true;
        meta.time_spec_s = static_cast<double>(meta.start_sample_abs) / kRate;
        if (opts_.gap_at >= 0 && pos_ == opts_.gap_at && !gap_done_) {
            meta.is_gap = true;
            meta.gap_samples = opts_.gap_samples;
            gap_done_ = true;
        }
        for (size_t i = 0; i < n; ++i) {
            dest[i] = sample_at(pos_ + static_cast<int64_t>(i));
        }
        pos_ += static_cast<int64_t>(n);
        if (opts_.paced) {
            const auto target = start_time_ + std::chrono::duration_cast<
                                                   std::chrono::steady_clock::duration>(
                                                   std::chrono::duration<double>(
                                                       static_cast<double>(pos_) / kRate));
            std::this_thread::sleep_until(target);
        }
        if (pos_ >= opts_.total) {
            eof_ = true;
            meta.is_eof = true;
        }
        return n;
    }

    int starts() const { return starts_; }
    bool closed() const { return closed_; }
    bool fatal() const { return fatal_; }

private:
    FakeOptions opts_;
    int64_t pos_ = 0;
    size_t calls_ = 0;
    bool eof_ = false;
    bool gap_done_ = false;
    bool fatal_ = false;
    bool closed_ = false;
    int starts_ = 0;
    std::chrono::steady_clock::time_point start_time_{};
    std::atomic<bool> stop_{ false };
};

core::FixedToneCancelConfig tone_config(size_t training = kTraining,
                                        size_t crossfade = kCrossfade)
{
    core::FixedToneCancelConfig cfg;
    cfg.enable = true;
    cfg.sample_rate = kRate;
    cfg.tone_frequency_hz = kTone;
    cfg.period_s = double(kPeriodLength) / kRate;
    cfg.training_s = double(training) / kRate;
    cfg.crossfade_samples = crossfade;
    return cfg;
}

std::unique_ptr<FixedToneCancelSc16Source> make_stage(std::unique_ptr<ISc16StreamSource> inner,
                                                      size_t block = 256,
                                                      size_t queue = 8,
                                                      size_t training = kTraining,
                                                      size_t crossfade = kCrossfade)
{
    auto stage = std::make_unique<FixedToneCancelSc16Source>(
        std::move(inner), tone_config(training, crossfade), block, queue);
    BOOST_REQUIRE(stage->open());
    return stage;
}

// Drain the stage exactly like the window assembler does.
struct DrainResult {
    std::vector<std::complex<int16_t>> samples;
    std::vector<int64_t> starts;
    std::vector<std::pair<int64_t, int64_t>> gaps; // (start_abs, gap_samples)
    size_t zero_reads = 0;
};

DrainResult drain(FixedToneCancelSc16Source& stage,
                  size_t read_size,
                  bool require_complete = true)
{
    DrainResult out;
    std::vector<std::complex<int16_t>> buf(read_size);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(20);
    while (std::chrono::steady_clock::now() < deadline) {
        StreamReadMeta meta;
        const size_t n = stage.read_samples(buf.data(), buf.size(), meta);
        if (n == 0) {
            if (meta.is_eof || stage.is_eof()) {
                break;
            }
            if (stage.fatal_error(nullptr)) {
                break;
            }
            ++out.zero_reads;
            continue;
        }
        if (meta.is_gap) {
            out.gaps.emplace_back(meta.start_sample_abs, meta.gap_samples);
        }
        for (size_t i = 0; i < n; ++i) {
            // A gap may skip absolute samples: keep the output stream in order
            // and record the absolute start of every returned chunk.
            out.samples.push_back(buf[i]);
        }
        out.starts.push_back(meta.start_sample_abs);
    }
    if (require_complete) {
        BOOST_REQUIRE_MESSAGE(stage.is_eof() || stage.fatal_error(nullptr),
                              "drain did not reach EOF/fatal");
    }
    return out;
}

void check_conservation(const FixedToneCancelStats& s)
{
    BOOST_CHECK_EQUAL(s.source_samples, s.queued_samples + s.dropped_samples);
    BOOST_CHECK_EQUAL(s.queued_samples, s.consumed_samples + s.pending_samples);
}

double coherent_amplitude(const std::vector<std::complex<int16_t>>& x,
                          size_t begin,
                          size_t count,
                          int64_t abs_at_begin)
{
    std::complex<double> acc(0.0, 0.0);
    for (size_t k = 0; k < count; ++k) {
        const double ph = -kOmega * static_cast<double>(abs_at_begin + int64_t(k));
        acc += std::complex<double>(x[begin + k].real(), x[begin + k].imag()) *
               std::complex<double>(std::cos(ph), std::sin(ph));
    }
    return std::abs(acc / static_cast<double>(count));
}

} // namespace

// ---------------------------------------------------------------------------
// Continuous stream: model updates, cancellation and sample conservation
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_stage_cancels_every_period_and_conserves_samples)
{
    FakeOptions opts;
    // Exactly four whole periods, so every training window (including the last)
    // is available in full and the run ends on a period boundary.
    opts.total = 4 * static_cast<int64_t>(kPeriodLength);
    opts.read_sizes = { 97, 256, 31, 512 }; // arbitrary chunking
    // The whole stream fits in the ring, so this case exercises the normal
    // path (overflow has its own case below).
    auto stage = make_stage(std::make_unique<FakeToneSource>(opts), 256, 64);
    BOOST_REQUIRE(stage->start_stream());

    auto out = drain(*stage, 333);
    const auto& s = stage->stats();
    check_conservation(s);
    BOOST_CHECK_EQUAL(s.pending_samples, uint64_t(0));
    BOOST_CHECK_EQUAL(s.dropped_samples, uint64_t(0));
    BOOST_CHECK_EQUAL(s.source_samples, uint64_t(opts.total));
    BOOST_CHECK_EQUAL(s.consumed_samples, uint64_t(opts.total));
    BOOST_CHECK_EQUAL(s.training_windows, uint64_t(4)); // epoch 0 + 3 boundaries
    BOOST_CHECK_EQUAL(s.model_updates, uint64_t(4));
    BOOST_CHECK_EQUAL(s.model_rejects, uint64_t(0));
    BOOST_CHECK_EQUAL(s.partial_training_bypasses, uint64_t(0));
    BOOST_CHECK_EQUAL(s.bypassed_samples, uint64_t(0));
    BOOST_CHECK_EQUAL(s.cancelled_samples, uint64_t(opts.total));
    BOOST_CHECK_EQUAL(s.gaps_forwarded, uint64_t(0));
    BOOST_CHECK_GE(s.model_generation, uint64_t(4));
    BOOST_CHECK_CLOSE(s.model_amplitude, kAmplitude, 5.0);

    // Output must be sample-for-sample the deterministic input, cancelled.
    BOOST_REQUIRE_EQUAL(out.samples.size(), static_cast<size_t>(opts.total));
    // The tone is gone: coherent amplitude in the last period is far below the
    // injected amplitude at the same frequency.
    const size_t tail_begin = 3 * kPeriodLength + 10;
    const double residual =
        coherent_amplitude(out.samples, tail_begin, 1000, int64_t(tail_begin));
    BOOST_CHECK_LT(residual, 0.05 * kAmplitude);
    // ... while the raw input at the same offsets carries the full amplitude.
    std::vector<std::complex<int16_t>> raw(1000);
    for (size_t k = 0; k < raw.size(); ++k) {
        raw[k] = sample_at(static_cast<int64_t>(tail_begin + k));
    }
    BOOST_CHECK_GT(coherent_amplitude(raw, 0, raw.size(), int64_t(tail_begin)),
                   0.9 * kAmplitude);

    // Absolute coordinates must be continuous and strictly increasing.
    for (size_t i = 1; i < out.starts.size(); ++i) {
        BOOST_CHECK_GT(out.starts[i], out.starts[i - 1]);
    }
}

BOOST_AUTO_TEST_CASE(test_stage_end_of_stream_partial_training_bypass)
{
    FakeOptions opts;
    opts.total = static_cast<int64_t>(kTraining) - 100; // shorter than a window
    opts.read_sizes = { 64 };
    auto stage = make_stage(std::make_unique<FakeToneSource>(opts), 256, 8);
    BOOST_REQUIRE(stage->start_stream());
    auto out = drain(*stage, 128);
    const auto& s = stage->stats();
    check_conservation(s);
    BOOST_CHECK_EQUAL(s.partial_training_bypasses, uint64_t(1));
    BOOST_CHECK_EQUAL(s.model_updates, uint64_t(0));
    BOOST_CHECK_EQUAL(s.cancelled_samples, uint64_t(0));
    BOOST_CHECK_EQUAL(s.bypassed_samples, uint64_t(opts.total));
    BOOST_CHECK(!stage->tone_core().has_model());
    // The short tail must come out unmodified, never zero-filled or dropped.
    BOOST_REQUIRE_EQUAL(out.samples.size(), static_cast<size_t>(opts.total));
    for (size_t i = 0; i < out.samples.size(); ++i) {
        BOOST_REQUIRE(out.samples[i] == sample_at(static_cast<int64_t>(i)));
    }
}

BOOST_AUTO_TEST_CASE(test_stage_gap_is_forwarded_and_resets_the_epoch)
{
    FakeOptions opts;
    opts.total = 4 * static_cast<int64_t>(kPeriodLength);
    opts.read_sizes = { 100 };
    // The discontinuity lands mid-period, so it cannot be confused with a
    // period boundary: 4000 logical samples -> 64 hidden samples.
    opts.gap_at = 4000;
    opts.gap_samples = 64;
    auto stage = make_stage(std::make_unique<FakeToneSource>(opts), 256, 64);
    BOOST_REQUIRE(stage->start_stream());
    auto out = drain(*stage, 300);
    const auto& s = stage->stats();
    check_conservation(s);
    BOOST_CHECK_EQUAL(s.dropped_samples, uint64_t(0));
    BOOST_REQUIRE_EQUAL(s.gaps_forwarded, uint64_t(1));
    BOOST_REQUIRE_EQUAL(out.gaps.size(), size_t(1));
    // The marker lands on the first post-gap sample, which is where the source
    // absolute axis jumped by 64 samples.
    BOOST_CHECK_EQUAL(out.gaps[0].first, int64_t(4064));
    BOOST_CHECK_EQUAL(out.gaps[0].second, int64_t(64));
    BOOST_CHECK_GE(s.epoch_resets, uint64_t(1));
    // Training must never span the gap: the model in use was re-anchored at or
    // after the post-gap sample.
    BOOST_CHECK_GE(s.model_epoch, int64_t(4064));
    // The samples after the gap are still cancelled (a new model was built).
    const size_t post_gap_begin = 4064;
    BOOST_REQUIRE_GT(out.samples.size(), post_gap_begin + 1200);
    const double residual = coherent_amplitude(out.samples,
                                               post_gap_begin + 500,
                                               800,
                                               int64_t(post_gap_begin + 500) + 64);
    BOOST_CHECK_LT(residual, 0.15 * kAmplitude);
}

BOOST_AUTO_TEST_CASE(test_stage_ring_overflow_drops_whole_blocks_and_gaps)
{
    FakeOptions opts;
    opts.total = 20000;
    opts.read_sizes = { 256 };
    opts.paced = true; // 69 ms of real-time delivery at 288 kHz
    // Capacity == training window (2 blocks of 256, training 288), so a slow
    // consumer overflows and the ring must drop whole blocks.
    auto stage = make_stage(std::make_unique<FakeToneSource>(opts), 256, 2, 288);
    BOOST_REQUIRE(stage->start_stream());

    std::vector<std::complex<int16_t>> buf(128);
    size_t produced = 0;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    while (std::chrono::steady_clock::now() < deadline) {
        StreamReadMeta meta;
        const size_t n = stage->read_samples(buf.data(), buf.size(), meta);
        if (n > 0) {
            produced += n;
        }
        if (meta.is_eof || stage->is_eof()) {
            break;
        }
        // Deliberately slower than the paced source: the ring must overflow
        // while the stream is still running, so the gap reaches the consumer.
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    const auto& s = stage->stats();
    check_conservation(s);
    BOOST_CHECK_GT(s.dropped_samples, uint64_t(0));
    BOOST_CHECK_GT(s.gaps_forwarded, uint64_t(0));
    BOOST_CHECK_GT(s.overflow_drops, uint64_t(0));
    // Every forwarded gap accounts for the samples dropped before it; drops at
    // the very end of the stream have no following block and stay in
    // dropped_samples only.
    BOOST_CHECK_LE(s.gap_samples_forwarded, s.dropped_samples);
    BOOST_CHECK_GT(s.gap_samples_forwarded, uint64_t(0));
    BOOST_CHECK_EQUAL(s.consumed_samples, uint64_t(produced));
    BOOST_CHECK_EQUAL(s.queued_samples, uint64_t(produced) + s.pending_samples);
    // Dropping must never stall the normal path: everything queued is emitted.
    BOOST_CHECK_EQUAL(s.pending_samples, uint64_t(0));
}

BOOST_AUTO_TEST_CASE(test_stage_fatal_error_is_not_disguised_as_eof)
{
    FakeOptions opts;
    opts.total = 50000;
    opts.read_sizes = { 256 };
    opts.fail_at = 3000;
    auto stage = make_stage(std::make_unique<FakeToneSource>(opts), 256, 8);
    BOOST_REQUIRE(stage->start_stream());
    std::vector<std::complex<int16_t>> buf(128);
    bool saw_fatal = false;
    for (int i = 0; i < 10000 && !saw_fatal; ++i) {
        StreamReadMeta meta;
        const size_t n = stage->read_samples(buf.data(), buf.size(), meta);
        (void)n;
        if (stage->fatal_error(nullptr)) {
            saw_fatal = true;
        } else if (meta.is_eof || stage->is_eof()) {
            break;
        }
    }
    BOOST_REQUIRE(saw_fatal);
    std::string msg;
    BOOST_CHECK(stage->fatal_error(&msg));
    BOOST_CHECK(!msg.empty());
    BOOST_CHECK(!stage->is_eof()); // a failure is never reported as EOF
    const auto& s = stage->stats();
    check_conservation(s);
}

BOOST_AUTO_TEST_CASE(test_stage_start_stream_failure_and_stop)
{
    {
        FakeOptions opts;
        opts.total = 1000;
        opts.start_ok = false;
        auto stage = make_stage(std::make_unique<FakeToneSource>(opts), 256, 8);
        BOOST_CHECK(!stage->start_stream());
        BOOST_CHECK(stage->fatal_error(nullptr));
    }
    {
        // A source that never produces must still let stop() finish the read.
        FakeOptions opts;
        opts.total = 1000;
        opts.idle = true;
        auto stage = make_stage(std::make_unique<FakeToneSource>(opts), 256, 8);
        BOOST_REQUIRE(stage->start_stream());
        std::atomic<bool> returned{ false };
        std::thread reader([&] {
            std::vector<std::complex<int16_t>> buf(64);
            StreamReadMeta meta;
            while (!stage->fatal_error(nullptr)) {
                const size_t n = stage->read_samples(buf.data(), buf.size(), meta);
                if (n == 0 && (meta.is_eof || stage->is_eof())) {
                    break;
                }
                if (stage->fatal_error(nullptr)) {
                    break;
                }
            }
            returned.store(true);
        });
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
        stage->request_stop();
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
        while (!returned.load() && std::chrono::steady_clock::now() < deadline) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        const bool finished = returned.load();
        if (reader.joinable()) {
            reader.join();
        }
        stage->close();
        BOOST_CHECK_MESSAGE(finished, "stop() must unblock a waiting read_samples()");
    }
}

// ---------------------------------------------------------------------------
// Pipeline integration: disabled path identical, enabled path cancels
// ---------------------------------------------------------------------------
namespace {

struct PipelineRun {
    std::string dump_dir;
    std::vector<WindowResult> results;
    FixedToneCancelStats tone;
    PipelineLedger ledger;
    bool tone_stage_present = false;
};

PipelineRun run_pipeline(bool enable_tone, const std::string& tag)
{
    PipelineConfig cfg;
    cfg.source_type = StreamSourceType::Replay;
    cfg.native_rate = kRate;
    cfg.work_rate = kRate;
    cfg.schedule_t0 = 0.0;
    // Window grid and tone period are deliberately different: the tone period
    // is longer than the whole capture, so exactly one model applies and the
    // expected cancellation is unambiguous.
    cfg.period_s = double(kPeriodLength) / kRate;
    cfg.pre_guard_samples = 64;
    cfg.capture_samples = 512;
    cfg.post_guard_samples = 64;
    cfg.pool_capacity = 4;
    cfg.consumer_mode = ConsumerMode::SicOnly;
    cfg.max_windows = 3;
    cfg.resampler_profile = "quality_minorder";
    cfg.iq_dump_dir = (std::filesystem::temp_directory_path() /
                       ("uwb_fixed_tone_qa_" + tag)).string();
    std::filesystem::remove_all(cfg.iq_dump_dir);
    cfg.iq_dump_tag = enable_tone ? "enabled" : "disabled";
    cfg.iq_dump_windows = 3;

    cfg.fixed_tone_cancel_enable = enable_tone;
    cfg.fixed_tone_cancel_frequency_hz = kTone;
    cfg.fixed_tone_cancel_period_s = double(kPipelinePeriod) / kRate;
    cfg.fixed_tone_cancel_training_s = double(kTraining) / kRate;
    cfg.fixed_tone_cancel_crossfade_samples = kCrossfade;
    // Large enough to hold this (unpaced) synthetic stream without drops.
    cfg.fixed_tone_cancel_block_samples = 1024;
    cfg.fixed_tone_cancel_queue_blocks = 16;

    FakeOptions opts;
    opts.total = 3 * static_cast<int64_t>(kPeriodLength) + 640 + 256;
    opts.read_sizes = { 256 };
    auto source = std::make_unique<FakeToneSource>(opts);

    PipelineRun out;
    out.dump_dir = cfg.iq_dump_dir;
    UhdSc16Pipeline pipeline(cfg);
    BOOST_REQUIRE(pipeline.initialize(std::move(source)));
    pipeline.start();
    BOOST_REQUIRE(pipeline.wait_ready(10000));
    pipeline.wait();
    BOOST_REQUIRE_MESSAGE(!pipeline.failed(), pipeline.init_error());
    BOOST_REQUIRE(!pipeline.source_failed());
    out.results = pipeline.results();
    out.ledger = pipeline.ledger();
    out.tone_stage_present = (pipeline.fixed_tone_source() != nullptr);
    if (pipeline.fixed_tone_source() != nullptr) {
        out.tone = pipeline.fixed_tone_source()->stats();
    }
    return out;
}

std::vector<std::complex<float>> read_dump(const std::string& dir,
                                           uint64_t window_id,
                                           const std::string& tag)
{
    const std::string path =
        dir + "/win_" + std::to_string(window_id) + "_" + tag + ".cf32";
    std::ifstream f(path, std::ios::binary | std::ios::ate);
    BOOST_REQUIRE_MESSAGE(f.good(), "missing IQ dump " + path);
    const auto bytes = static_cast<size_t>(f.tellg());
    f.seekg(0);
    std::vector<std::complex<float>> data(bytes / sizeof(std::complex<float>));
    f.read(reinterpret_cast<char*>(data.data()), static_cast<std::streamsize>(bytes));
    return data;
}

} // namespace

BOOST_AUTO_TEST_CASE(test_pipeline_disabled_path_is_bit_identical)
{
    auto a = run_pipeline(false, "off_a");
    auto b = run_pipeline(false, "off_b");
    BOOST_CHECK(!a.tone_stage_present);
    BOOST_CHECK(!b.tone_stage_present);
    BOOST_CHECK(a.ledger.verify_invariants());
    BOOST_CHECK(b.ledger.verify_invariants());
    BOOST_REQUIRE_EQUAL(a.results.size(), b.results.size());
    for (const auto& r : a.results) {
        if (r.state != WindowTerminalState::Completed) {
            continue;
        }
        std::vector<std::complex<float>> da;
        std::vector<std::complex<float>> db;
        std::ifstream fa(a.dump_dir + "/win_" + std::to_string(r.window_id) +
                             "_disabled.cf32",
                         std::ios::binary | std::ios::ate);
        BOOST_REQUIRE(fa.good());
        const auto bytes = static_cast<size_t>(fa.tellg());
        fa.seekg(0);
        da.resize(bytes / sizeof(std::complex<float>));
        fa.read(reinterpret_cast<char*>(da.data()), static_cast<std::streamsize>(bytes));
        std::ifstream fb(b.dump_dir + "/win_" + std::to_string(r.window_id) +
                             "_disabled.cf32",
                         std::ios::binary | std::ios::ate);
        BOOST_REQUIRE(fb.good());
        fb.seekg(0);
        db.resize(da.size());
        fb.read(reinterpret_cast<char*>(db.data()), static_cast<std::streamsize>(bytes));
        BOOST_REQUIRE_EQUAL(da.size(), db.size());
        BOOST_CHECK_MESSAGE(std::memcmp(da.data(), db.data(), da.size() * sizeof(float) * 2) ==
                                0,
                            "disabled fixed-tone path must be bit identical");
    }
}

BOOST_AUTO_TEST_CASE(test_pipeline_enabled_path_cancels_before_capture)
{
    auto run = run_pipeline(true, "on");
    BOOST_REQUIRE(run.tone_stage_present);
    BOOST_CHECK(run.ledger.verify_invariants());
    BOOST_CHECK_EQUAL(run.tone.dropped_samples, uint64_t(0));
    BOOST_CHECK_EQUAL(run.tone.partial_training_bypasses, uint64_t(0));
    BOOST_CHECK_GE(run.tone.model_updates, uint64_t(1));
    check_conservation(run.tone);

    // Independently rebuild the model from the raw samples the source would
    // have produced (epoch 0, no cross-fade on the first model) and compare
    // every captured window sample by sample.
    core::FixedToneCancelCore reference_core;
    std::string err;
    core::FixedToneCancelConfig ref_cfg = tone_config();
    ref_cfg.period_s = double(kPipelinePeriod) / kRate;
    BOOST_REQUIRE_MESSAGE(reference_core.prepare(ref_cfg, &err), err);
    std::vector<int16_t> training(2 * kTraining);
    for (size_t i = 0; i < kTraining; ++i) {
        const auto z = sample_at(static_cast<int64_t>(i));
        training[2 * i] = z.real();
        training[2 * i + 1] = z.imag();
    }
    BOOST_REQUIRE(reference_core.begin_training(0, &err));
    BOOST_REQUIRE(reference_core.add_training_samples(training.data(), kTraining, &err));
    BOOST_REQUIRE(reference_core.finish_training(&err) ==
                  core::FixedToneTrainingStatus::Ok);

    size_t compared = 0;
    size_t windows = 0;
    for (const auto& r : run.results) {
        if (r.state != WindowTerminalState::Completed) {
            continue;
        }
        ++windows;
        const auto dump = read_dump(run.dump_dir, r.window_id, "enabled");
        BOOST_REQUIRE_EQUAL(dump.size(), r.native_sample_count);
        std::vector<int16_t> raw(2 * dump.size());
        for (size_t i = 0; i < dump.size(); ++i) {
            const auto z = sample_at(r.native_start_abs + static_cast<int64_t>(i));
            raw[2 * i] = z.real();
            raw[2 * i + 1] = z.imag();
        }
        std::vector<int16_t> expect(2 * dump.size(), 0);
        const auto counts = reference_core.copy_cancel(
            raw.data(), expect.data(), dump.size(), r.native_start_abs);
        BOOST_REQUIRE_EQUAL(counts.cancelled, dump.size());
        for (size_t i = 0; i < dump.size(); ++i) {
            const float er = static_cast<float>(expect[2 * i]) / 32768.0f;
            const float ei = static_cast<float>(expect[2 * i + 1]) / 32768.0f;
            if (dump[i].real() != er || dump[i].imag() != ei) {
                BOOST_ERROR("window " << r.window_id << " sample " << i
                                      << " differs from the expected cancelled IQ ("
                                      << dump[i].real() << " vs " << er << ")");
                break;
            }
            ++compared;
        }
    }
    BOOST_REQUIRE_GT(windows, size_t(0));
    BOOST_CHECK_EQUAL(compared, windows * 640);

    // The cleaned capture must have lost the configured tone.
    std::vector<std::complex<int16_t>> clean;
    const auto dump = read_dump(run.dump_dir, 2, "enabled");
    for (const auto& z : dump) {
        clean.emplace_back(static_cast<int16_t>(std::lround(z.real() * 32768.0f)),
                           static_cast<int16_t>(std::lround(z.imag() * 32768.0f)));
    }
    // Absolute index of the dump start for window 2:
    const int64_t win2_start = 2 * static_cast<int64_t>(kPeriodLength) - 64;
    const double residual =
        [&] {
            // The dump's first sample sits at win2_start on the absolute axis.
            std::complex<double> acc(0.0, 0.0);
            for (size_t k = 0; k < clean.size(); ++k) {
                const double ph = -kOmega * static_cast<double>(win2_start + int64_t(k));
                acc += std::complex<double>(clean[k].real(), clean[k].imag()) *
                       std::complex<double>(std::cos(ph), std::sin(ph));
            }
            return std::abs(acc / static_cast<double>(clean.size()));
        }();
    BOOST_CHECK_LT(residual, 0.05 * kAmplitude);

    std::filesystem::remove_all(run.dump_dir);
}
