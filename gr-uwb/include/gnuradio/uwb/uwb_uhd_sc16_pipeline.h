/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * UHD SC16 Direct RX and Multi-Core Stream Pipeline.
 * Bypasses GNU Radio scheduler and PMT/PDU messaging overhead.
 */

#pragma once

#include <gnuradio/uwb/api.h>
#include <gnuradio/uwb/uwb_radar_cir_core.h>
#include <gnuradio/uwb/uwb_scheduled_extractor_core.h>
#include <gnuradio/uwb/uwb_sic_unified.h>

#include <atomic>
#include <chrono>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace gr {
namespace uwb {
namespace core {
using gr::uwb::radar::RadarCirConfig;
}
}
}

namespace gr {
namespace uwb {
namespace pipeline {

enum class StreamSourceType {
    Replay,
    UhdDevice
};

enum class ConsumerMode {
    SicOnly,
    SensingCir,
    Both
};

enum class WindowTerminalState {
    Completed,
    ProcessingFailed,
    DroppedAdmission,
    InvalidGap,
    Cancelled
};

enum class SlotState : uint32_t {
    Free = 0,
    Filling = 1,
    Ready = 2,
    Reading = 3
};

enum class Sc16ScaleMode {
    UnitRange, // 1.0f / 32768.0f (default)
    RawInteger // 1.0f
};

struct StreamReadMeta {
    int64_t start_sample_abs = 0;
    bool has_time_spec = false;
    double time_spec_s = 0.0;
    bool is_gap = false;
    int64_t gap_samples = 0;
    bool is_eof = false;
    int error_code = 0;
};

struct WindowTimingStats {
    double slice_us = 0.0;
    double queue_us = 0.0;
    double convert_us = 0.0;
    double resample_us = 0.0;
    double sic_us = 0.0;
    double sensing_us = 0.0;
    double e2e_us = 0.0;
};

struct WindowResult {
    uint64_t window_id = 0;
    WindowTerminalState state = WindowTerminalState::Completed;
    int64_t native_start_abs = 0;
    size_t native_sample_count = 0;
    WindowTimingStats timing;

    // SIC result
    bool sic_executed = false;
    bool sic_committed = false;
    float actual_suppression_db = 0.0f;
    std::string sic_status;

    // Sensing result
    bool radar_executed = false;
    bool radar_sfd_found = false;
    float radar_peak_metric = 0.0f;
    int64_t radar_detected_start = -1;
};

struct UWB_API PipelineLedger {
    uint64_t offered_windows = 0;
    uint64_t admitted_windows = 0;
    uint64_t completed_windows = 0;
    uint64_t processing_failed = 0;
    uint64_t dropped_admission = 0;
    uint64_t invalid_gap_before_admission = 0;
    uint64_t invalid_gap_after_admission = 0;
    uint64_t cancelled_windows = 0;

    uint64_t total_samples_received = 0;
    uint64_t short_reads = 0;
    uint64_t read_timeouts = 0;
    uint64_t read_overflows = 0;

    bool verify_invariants(std::string* err = nullptr) const;
};

struct PipelineConfig {
    StreamSourceType source_type = StreamSourceType::Replay;
    std::string replay_file;
    bool paced_replay = false;

    double native_rate = 998.4e6;
    double work_rate = 998.4e6;
    double schedule_t0 = 0.0;
    double period_s = 0.001; // 1 ms default
    size_t pre_guard_samples = 400;
    size_t capture_samples = 3200;
    size_t post_guard_samples = 400;

    size_t pool_capacity = 8;
    ConsumerMode consumer_mode = ConsumerMode::SicOnly;
    int cpu_core_rx = -1;
    int cpu_core_dsp = -1;
    Sc16ScaleMode scale_mode = Sc16ScaleMode::UnitRange;

    // Resampler configuration
    std::string resampler_profile = "quality_minorder";
    int resampler_workers = 1;

    uint64_t max_windows = 0; // 0 = unlimited
    uint64_t max_samples = 0; // 0 = unlimited

    // UHD specific
    std::string uhd_args;
    std::string uhd_subdev;
    std::string uhd_channel;
    double uhd_center_freq = 3993.6e6;
    double uhd_gain = 30.0;
    std::string uhd_clock_source;
    std::string uhd_time_source;

    // DSP configs
    gr::uwb::sic::SicUnifiedConfig sic_cfg;
    gr::uwb::core::RadarCirConfig radar_cfg;

    size_t native_window_capacity() const {
        return pre_guard_samples + capture_samples + post_guard_samples;
    }
};

struct Sc16WindowSlot {
    alignas(64) std::atomic<SlotState> state{SlotState::Free};
    uint64_t window_id = 0;
    int64_t native_start_abs = 0;
    size_t native_count = 0;
    double native_rate = 0.0;
    double work_rate = 0.0;
    std::chrono::steady_clock::time_point push_time;
    std::vector<std::complex<int16_t>> sc16_samples;
};

// ---------------------------------------------------------------------------
// Conversion kernels
// ---------------------------------------------------------------------------
UWB_API void sc16_to_cf32_scalar(const std::complex<int16_t>* in,
                                 std::complex<float>* out,
                                 size_t n,
                                 float scale = 1.0f / 32768.0f);

UWB_API void sc16_to_cf32_avx2(const std::complex<int16_t>* in,
                               std::complex<float>* out,
                               size_t n,
                               float scale = 1.0f / 32768.0f);

inline void sc16_to_cf32(const std::complex<int16_t>* in,
                         std::complex<float>* out,
                         size_t n,
                         float scale = 1.0f / 32768.0f)
{
#if defined(__AVX2__) && defined(__FMA__)
    sc16_to_cf32_avx2(in, out, n, scale);
#else
    sc16_to_cf32_scalar(in, out, n, scale);
#endif
}

// ---------------------------------------------------------------------------
// Lock-free SPSC Ring Pool
// ---------------------------------------------------------------------------
class UWB_API Sc16WindowRingPool {
public:
    explicit Sc16WindowRingPool(size_t capacity, size_t slot_sample_capacity);
    ~Sc16WindowRingPool();

    size_t capacity() const { return capacity_; }

    // Producer methods
    Sc16WindowSlot* acquire_filling_slot();
    bool publish_ready_slot(Sc16WindowSlot* slot);
    void abort_filling_slot(Sc16WindowSlot* slot);

    // Consumer methods
    Sc16WindowSlot* acquire_ready_slot();
    void release_reading_slot(Sc16WindowSlot* slot);

private:
    const size_t capacity_;
    const size_t slot_sample_capacity_;
    std::vector<std::unique_ptr<Sc16WindowSlot>> slots_;

    alignas(64) std::atomic<size_t> write_idx_{0};
    alignas(64) std::atomic<size_t> read_idx_{0};
};

// ---------------------------------------------------------------------------
// Data Sources
// ---------------------------------------------------------------------------
class UWB_API ISc16StreamSource {
public:
    virtual ~ISc16StreamSource() = default;
    virtual bool open() = 0;
    virtual void close() = 0;
    virtual size_t read_samples(std::complex<int16_t>* dest,
                                size_t max_samples,
                                StreamReadMeta& meta) = 0;
    virtual double actual_rx_rate() const = 0;
    virtual bool is_eof() const = 0;
};

class UWB_API ReplaySc16Source : public ISc16StreamSource {
public:
    ReplaySc16Source(const std::string& filepath,
                     double sample_rate,
                     bool paced = false);
    ~ReplaySc16Source() override;

    bool open() override;
    void close() override;
    size_t read_samples(std::complex<int16_t>* dest,
                        size_t max_samples,
                        StreamReadMeta& meta) override;
    double actual_rx_rate() const override { return sample_rate_; }
    bool is_eof() const override { return eof_; }

private:
    std::string filepath_;
    double sample_rate_;
    bool paced_;
    bool eof_{false};
    int64_t current_sample_abs_{0};
    int fd_{-1};
    std::chrono::steady_clock::time_point start_time_;
};

class UWB_API UhdDeviceSc16Source : public ISc16StreamSource {
public:
    explicit UhdDeviceSc16Source(const PipelineConfig& cfg);
    ~UhdDeviceSc16Source() override;

    bool open() override;
    void close() override;
    size_t read_samples(std::complex<int16_t>* dest,
                        size_t max_samples,
                        StreamReadMeta& meta) override;
    double actual_rx_rate() const override;
    bool is_eof() const override { return eof_; }
    static bool is_supported();

private:
    PipelineConfig cfg_;
    bool eof_{false};
    double actual_rate_{0.0};
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

// ---------------------------------------------------------------------------
// Pipeline Manager
// ---------------------------------------------------------------------------
class UWB_API UhdSc16Pipeline {
public:
    using ResultCallback = std::function<void(const WindowResult&)>;

    explicit UhdSc16Pipeline(const PipelineConfig& cfg);
    ~UhdSc16Pipeline();

    bool initialize(std::unique_ptr<ISc16StreamSource> custom_source = nullptr);
    void start();
    void stop();
    void wait();

    void set_result_callback(ResultCallback cb) { result_cb_ = std::move(cb); }

    const PipelineLedger& ledger() const { return ledger_; }
    const std::vector<WindowResult>& results() const { return results_; }

private:
    void rx_producer_loop();
    void dsp_consumer_loop();

    PipelineConfig cfg_;
    std::unique_ptr<ISc16StreamSource> source_;
    std::unique_ptr<Sc16WindowRingPool> pool_;

    std::atomic<bool> running_{false};
    std::atomic<bool> stop_requested_{false};
    std::atomic<bool> rx_finished_{false};

    std::thread rx_thread_;
    std::thread dsp_thread_;

    PipelineLedger ledger_;
    std::vector<WindowResult> results_;
    ResultCallback result_cb_;
    std::mutex results_mutex_;
};

} // namespace pipeline
} // namespace uwb
} // namespace gr
