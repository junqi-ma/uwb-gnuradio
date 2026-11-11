/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * UHD SC16 Direct RX and Multi-Core Stream Pipeline implementation.
 * Bypasses GNU Radio scheduler and PMT/PDU messaging overhead.
 */

#include <gnuradio/uwb/uwb_uhd_sc16_pipeline.h>
#include <gnuradio/uwb/uwb_rational_resampler_core.h>
#include <gnuradio/uwb/uwb_defaults.h>
#include <volk/volk.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <fstream>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>

#if defined(__linux__)
#include <pthread.h>
#include <sched.h>
#endif

#if defined(__AVX2__) && defined(__FMA__)
#include <immintrin.h>
#endif

#ifndef UWB_TESTDATA_DIR
#define UWB_TESTDATA_DIR "testdata"
#endif

namespace gr {
namespace uwb {
namespace pipeline {

namespace {

std::string find_testdata_file(const std::string& rel)
{
    const char* prefixes[] = {
        "",
        "testdata/",
        "../testdata/",
        "../../testdata/",
        "../../../testdata/",
        UWB_TESTDATA_DIR "/",
    };
    for (const char* p : prefixes) {
        std::string cand = std::string(p) + rel;
        std::ifstream f(cand, std::ios::binary);
        if (f.good()) {
            return cand;
        }
    }
    return "";
}

std::vector<float> load_taps_f32(const std::string& rel)
{
    std::string path = find_testdata_file(rel);
    if (path.empty()) {
        return {};
    }
    std::ifstream f(path, std::ios::binary | std::ios::ate);
    if (!f) return {};
    auto bytes = static_cast<size_t>(f.tellg());
    if (bytes == 0 || bytes % sizeof(float) != 0) return {};
    f.seekg(0);
    std::vector<float> taps(bytes / sizeof(float));
    f.read(reinterpret_cast<char*>(taps.data()), static_cast<std::streamsize>(bytes));
    return taps;
}

std::vector<std::complex<float>> load_cf32_file(const std::string& rel)
{
    std::string path = find_testdata_file(rel);
    if (path.empty()) {
        return {};
    }
    std::ifstream f(path, std::ios::binary | std::ios::ate);
    if (!f) return {};
    auto bytes = static_cast<size_t>(f.tellg());
    if (bytes == 0 || bytes % sizeof(std::complex<float>) != 0) return {};
    f.seekg(0);
    std::vector<std::complex<float>> data(bytes / sizeof(std::complex<float>));
    f.read(reinterpret_cast<char*>(data.data()), static_cast<std::streamsize>(bytes));
    return data;
}

} // namespace

// ---------------------------------------------------------------------------
// Conversion kernels
// ---------------------------------------------------------------------------
void sc16_to_cf32_scalar(const std::complex<int16_t>* in,
                         std::complex<float>* out,
                         size_t n,
                         float scale)
{
    for (size_t i = 0; i < n; ++i) {
        out[i] = std::complex<float>(
            static_cast<float>(in[i].real()) * scale,
            static_cast<float>(in[i].imag()) * scale);
    }
}

void sc16_to_cf32_avx2(const std::complex<int16_t>* in,
                       std::complex<float>* out,
                       size_t n,
                       float scale)
{
#if defined(__AVX2__) && defined(__FMA__)
    size_t i = 0;
    const __m256 v_scale = _mm256_set1_ps(scale);

    for (; i + 8 <= n; i += 8) {
        __m128i in_lo = _mm_loadu_si128(reinterpret_cast<const __m128i*>(in + i));
        __m128i in_hi = _mm_loadu_si128(reinterpret_cast<const __m128i*>(in + i + 4));

        __m256i i32_lo = _mm256_cvtepi16_epi32(in_lo);
        __m256i i32_hi = _mm256_cvtepi16_epi32(in_hi);

        __m256 f32_lo = _mm256_cvtepi32_ps(i32_lo);
        __m256 f32_hi = _mm256_cvtepi32_ps(i32_hi);

        __m256 scaled_lo = _mm256_mul_ps(f32_lo, v_scale);
        __m256 scaled_hi = _mm256_mul_ps(f32_hi, v_scale);

        _mm256_storeu_ps(reinterpret_cast<float*>(out + i), scaled_lo);
        _mm256_storeu_ps(reinterpret_cast<float*>(out + i + 4), scaled_hi);
    }

    for (; i < n; ++i) {
        out[i] = std::complex<float>(
            static_cast<float>(in[i].real()) * scale,
            static_cast<float>(in[i].imag()) * scale);
    }
#else
    sc16_to_cf32_scalar(in, out, n, scale);
#endif
}

// ---------------------------------------------------------------------------
// Lock-free SPSC Ring Pool
// ---------------------------------------------------------------------------
Sc16WindowRingPool::Sc16WindowRingPool(size_t capacity, size_t slot_sample_capacity)
    : capacity_(capacity > 0 ? capacity : 1),
      slot_sample_capacity_(slot_sample_capacity)
{
    slots_.reserve(capacity_);
    for (size_t i = 0; i < capacity_; ++i) {
        auto slot = std::make_unique<Sc16WindowSlot>();
        slot->sc16_samples.resize(slot_sample_capacity_);
        slot->state.store(SlotState::Free, std::memory_order_relaxed);
        slots_.push_back(std::move(slot));
    }
}

Sc16WindowRingPool::~Sc16WindowRingPool() = default;

Sc16WindowSlot* Sc16WindowRingPool::acquire_filling_slot()
{
    size_t w = write_idx_.load(std::memory_order_relaxed);
    size_t idx = w % capacity_;
    Sc16WindowSlot* slot = slots_[idx].get();
    if (slot->state.load(std::memory_order_acquire) == SlotState::Free) {
        slot->state.store(SlotState::Filling, std::memory_order_release);
        return slot;
    }
    return nullptr;
}

bool Sc16WindowRingPool::publish_ready_slot(Sc16WindowSlot* slot)
{
    if (!slot) {
        return false;
    }
    if (slot->state.load(std::memory_order_relaxed) != SlotState::Filling) {
        return false;
    }
    slot->state.store(SlotState::Ready, std::memory_order_release);
    write_idx_.fetch_add(1, std::memory_order_release);
    return true;
}

void Sc16WindowRingPool::abort_filling_slot(Sc16WindowSlot* slot)
{
    if (!slot) {
        return;
    }
    if (slot->state.load(std::memory_order_relaxed) == SlotState::Filling) {
        slot->state.store(SlotState::Free, std::memory_order_release);
    }
}

Sc16WindowSlot* Sc16WindowRingPool::acquire_ready_slot()
{
    size_t r = read_idx_.load(std::memory_order_relaxed);
    size_t w = write_idx_.load(std::memory_order_acquire);
    if (r >= w) {
        return nullptr;
    }
    size_t idx = r % capacity_;
    Sc16WindowSlot* slot = slots_[idx].get();
    if (slot->state.load(std::memory_order_acquire) == SlotState::Ready) {
        slot->state.store(SlotState::Reading, std::memory_order_release);
        return slot;
    }
    return nullptr;
}

void Sc16WindowRingPool::release_reading_slot(Sc16WindowSlot* slot)
{
    if (!slot) {
        return;
    }
    slot->state.store(SlotState::Free, std::memory_order_release);
    read_idx_.fetch_add(1, std::memory_order_release);
}

// ---------------------------------------------------------------------------
// PipelineLedger
// ---------------------------------------------------------------------------
bool PipelineLedger::verify_invariants(std::string* err) const
{
    uint64_t expected_offered = admitted_windows + dropped_admission +
                                invalid_gap_before_admission + cancelled_windows;
    if (offered_windows != expected_offered) {
        if (err) {
            *err = "Ledger invariant 1 violated: offered (" +
                   std::to_string(offered_windows) + ") != admitted (" +
                   std::to_string(admitted_windows) + ") + dropped (" +
                   std::to_string(dropped_admission) + ") + invalid_gap_before (" +
                   std::to_string(invalid_gap_before_admission) + ") + cancelled (" +
                   std::to_string(cancelled_windows) + ") [sum=" +
                   std::to_string(expected_offered) + "]";
        }
        return false;
    }
    uint64_t expected_admitted = completed_windows + processing_failed +
                                 invalid_gap_after_admission;
    if (admitted_windows != expected_admitted) {
        if (err) {
            *err = "Ledger invariant 2 violated: admitted (" +
                   std::to_string(admitted_windows) + ") != completed (" +
                   std::to_string(completed_windows) + ") + failed (" +
                   std::to_string(processing_failed) + ") + invalid_gap_after (" +
                   std::to_string(invalid_gap_after_admission) + ") [sum=" +
                   std::to_string(expected_admitted) + "]";
        }
        return false;
    }
    return true;
}

// ---------------------------------------------------------------------------
// ReplaySc16Source
// ---------------------------------------------------------------------------
ReplaySc16Source::ReplaySc16Source(const std::string& filepath,
                                   double sample_rate,
                                   bool paced)
    : filepath_(filepath),
      sample_rate_(sample_rate > 0.0 ? sample_rate : 998400000.0),
      paced_(paced),
      eof_(false),
      current_sample_abs_(0),
      fd_(-1)
{
}

ReplaySc16Source::~ReplaySc16Source()
{
    close();
}

bool ReplaySc16Source::open()
{
    close();
    fd_ = ::open(filepath_.c_str(), O_RDONLY);
    if (fd_ < 0) {
        return false;
    }
    eof_ = false;
    current_sample_abs_ = 0;
    start_time_ = std::chrono::steady_clock::now();
    return true;
}

void ReplaySc16Source::close()
{
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
    eof_ = true;
}

size_t ReplaySc16Source::read_samples(std::complex<int16_t>* dest,
                                      size_t max_samples,
                                      StreamReadMeta& meta)
{
    meta = StreamReadMeta{};
    if (fd_ < 0 || eof_ || max_samples == 0 || !dest) {
        eof_ = true;
        meta.is_eof = true;
        return 0;
    }

    size_t bytes_to_read = max_samples * sizeof(std::complex<int16_t>);
    char* ptr = reinterpret_cast<char*>(dest);
    size_t total_bytes_read = 0;

    while (total_bytes_read < bytes_to_read) {
        ssize_t n = ::read(fd_, ptr + total_bytes_read, bytes_to_read - total_bytes_read);
        if (n > 0) {
            total_bytes_read += static_cast<size_t>(n);
        } else if (n == 0) {
            eof_ = true;
            break;
        } else {
            if (errno == EINTR) {
                continue;
            }
            meta.error_code = errno;
            break;
        }
    }

    size_t samples_read = total_bytes_read / sizeof(std::complex<int16_t>);
    meta.start_sample_abs = current_sample_abs_;
    meta.has_time_spec = true;
    meta.time_spec_s = static_cast<double>(current_sample_abs_) / sample_rate_;
    meta.is_gap = false;
    meta.gap_samples = 0;
    meta.is_eof = eof_;

    if (paced_ && samples_read > 0 && sample_rate_ > 0.0) {
        int64_t target_sample = current_sample_abs_ + samples_read;
        double elapsed_s = static_cast<double>(target_sample) / sample_rate_;
        auto target_tp = start_time_ + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(elapsed_s));
        auto now = std::chrono::steady_clock::now();
        if (target_tp > now) {
            std::this_thread::sleep_until(target_tp);
        }
    }

    current_sample_abs_ += samples_read;
    return samples_read;
}

// ---------------------------------------------------------------------------
// UhdDeviceSc16Source
// ---------------------------------------------------------------------------
#if defined(UWB_HAVE_UHD) && (UWB_HAVE_UHD)
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/stream.hpp>
#include <uhd/types/metadata.hpp>
#include <uhd/types/stream_cmd.hpp>
#include <uhd/exception.hpp>

struct UhdDeviceSc16Source::Impl {
    uhd::usrp::multi_usrp::sptr usrp;
    uhd::rx_streamer::sptr rx_stream;
    uhd::rx_metadata_t md;
    int64_t current_sample_abs = 0;
};

bool UhdDeviceSc16Source::is_supported()
{
    return true;
}

UhdDeviceSc16Source::UhdDeviceSc16Source(const PipelineConfig& cfg)
    : cfg_(cfg), eof_(false), actual_rate_(0.0)
{
}

UhdDeviceSc16Source::~UhdDeviceSc16Source()
{
    close();
}

bool UhdDeviceSc16Source::open()
{
    close();
    try {
        impl_ = std::make_unique<Impl>();
        impl_->usrp = uhd::usrp::multi_usrp::make(cfg_.uhd_args);
        if (!cfg_.uhd_subdev.empty()) {
            impl_->usrp->set_rx_subdev_spec(cfg_.uhd_subdev);
        }
        if (!cfg_.uhd_clock_source.empty()) {
            impl_->usrp->set_clock_source(cfg_.uhd_clock_source);
        }
        if (!cfg_.uhd_time_source.empty()) {
            impl_->usrp->set_time_source(cfg_.uhd_time_source);
        }
        impl_->usrp->set_rx_rate(cfg_.native_rate);
        actual_rate_ = impl_->usrp->get_rx_rate();
        impl_->usrp->set_rx_freq(cfg_.uhd_center_freq);
        impl_->usrp->set_rx_gain(cfg_.uhd_gain);

        uhd::stream_args_t stream_args("sc16", "sc16");
        if (!cfg_.uhd_channel.empty()) {
            stream_args.channels = {static_cast<size_t>(std::stoul(cfg_.uhd_channel))};
        }
        impl_->rx_stream = impl_->usrp->get_rx_stream(stream_args);

        uhd::stream_cmd_t cmd(uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS);
        cmd.stream_now = true;
        impl_->rx_stream->issue_stream_cmd(cmd);

        eof_ = false;
        impl_->current_sample_abs = 0;
        return true;
    } catch (const std::exception& e) {
        impl_.reset();
        return false;
    }
}

void UhdDeviceSc16Source::close()
{
    if (impl_ && impl_->rx_stream) {
        try {
            uhd::stream_cmd_t cmd(uhd::stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS);
            impl_->rx_stream->issue_stream_cmd(cmd);
        } catch (...) {}
    }
    impl_.reset();
    eof_ = true;
}

size_t UhdDeviceSc16Source::read_samples(std::complex<int16_t>* dest,
                                         size_t max_samples,
                                         StreamReadMeta& meta)
{
    meta = StreamReadMeta{};
    if (!impl_ || !impl_->rx_stream || max_samples == 0 || !dest) {
        eof_ = true;
        meta.is_eof = true;
        return 0;
    }

    double timeout = 0.5;
    size_t num_rx = impl_->rx_stream->recv(dest, max_samples, impl_->md, timeout, false);

    meta.error_code = static_cast<int>(impl_->md.error_code);
    meta.has_time_spec = impl_->md.has_time_spec;
    if (impl_->md.has_time_spec) {
        meta.time_spec_s = impl_->md.time_spec.get_real_secs();
    }
    meta.is_gap = (impl_->md.error_code == uhd::rx_metadata_t::ERROR_CODE_OVERFLOW ||
                   impl_->md.error_code == uhd::rx_metadata_t::ERROR_CODE_LATE_COMMAND);
    meta.gap_samples = 0;
    meta.start_sample_abs = impl_->current_sample_abs;
    meta.is_eof = eof_;

    impl_->current_sample_abs += num_rx;
    return num_rx;
}

double UhdDeviceSc16Source::actual_rx_rate() const
{
    return actual_rate_ > 0.0 ? actual_rate_ : cfg_.native_rate;
}

#else

struct UhdDeviceSc16Source::Impl {};

bool UhdDeviceSc16Source::is_supported()
{
    return false;
}

UhdDeviceSc16Source::UhdDeviceSc16Source(const PipelineConfig& cfg)
    : cfg_(cfg), eof_(true), actual_rate_(0.0)
{
}

UhdDeviceSc16Source::~UhdDeviceSc16Source() = default;

bool UhdDeviceSc16Source::open()
{
    return false;
}

void UhdDeviceSc16Source::close()
{
}

size_t UhdDeviceSc16Source::read_samples(std::complex<int16_t>*,
                                         size_t,
                                         StreamReadMeta& meta)
{
    meta = StreamReadMeta{};
    meta.is_eof = true;
    return 0;
}

double UhdDeviceSc16Source::actual_rx_rate() const
{
    return 0.0;
}

#endif

// ---------------------------------------------------------------------------
// UhdSc16Pipeline
// ---------------------------------------------------------------------------
UhdSc16Pipeline::UhdSc16Pipeline(const PipelineConfig& cfg)
    : cfg_(cfg),
      running_(false),
      stop_requested_(false),
      rx_finished_(false)
{
}

UhdSc16Pipeline::~UhdSc16Pipeline()
{
    stop();
    wait();
}

bool UhdSc16Pipeline::initialize(std::unique_ptr<ISc16StreamSource> custom_source)
{
    // Ensure VOLK alignment mask is initialized when running outside GNU Radio scheduler
    (void)volk_get_alignment();

    if (custom_source) {
        source_ = std::move(custom_source);
    } else {
        if (cfg_.source_type == StreamSourceType::Replay) {
            source_ = std::make_unique<ReplaySc16Source>(
                cfg_.replay_file, cfg_.native_rate, cfg_.paced_replay);
        } else if (cfg_.source_type == StreamSourceType::UhdDevice) {
            source_ = std::make_unique<UhdDeviceSc16Source>(cfg_);
        } else {
            return false;
        }
    }

    size_t slot_cap = cfg_.native_window_capacity();
    pool_ = std::make_unique<Sc16WindowRingPool>(cfg_.pool_capacity, slot_cap);

    if (!source_->open()) {
        return false;
    }

    return true;
}

void UhdSc16Pipeline::start()
{
    if (running_.exchange(true)) {
        return;
    }
    stop_requested_ = false;
    rx_finished_ = false;

    rx_thread_ = std::thread(&UhdSc16Pipeline::rx_producer_loop, this);
#if defined(__linux__) && defined(_GNU_SOURCE)
    if (cfg_.cpu_core_rx >= 0) {
        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        CPU_SET(cfg_.cpu_core_rx, &cpuset);
        pthread_setaffinity_np(rx_thread_.native_handle(), sizeof(cpu_set_t), &cpuset);
    }
#endif

    dsp_thread_ = std::thread(&UhdSc16Pipeline::dsp_consumer_loop, this);
#if defined(__linux__) && defined(_GNU_SOURCE)
    if (cfg_.cpu_core_dsp >= 0) {
        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        CPU_SET(cfg_.cpu_core_dsp, &cpuset);
        pthread_setaffinity_np(dsp_thread_.native_handle(), sizeof(cpu_set_t), &cpuset);
    }
#endif
}

void UhdSc16Pipeline::stop()
{
    stop_requested_ = true;
}

void UhdSc16Pipeline::wait()
{
    if (rx_thread_.joinable()) {
        rx_thread_.join();
    }
    if (dsp_thread_.joinable()) {
        dsp_thread_.join();
    }
    running_ = false;
    if (source_) {
        source_->close();
    }
}

void UhdSc16Pipeline::rx_producer_loop()
{
    const size_t capacity = cfg_.native_window_capacity();
    const double period_samples = cfg_.period_s * cfg_.native_rate;
    const double t0 = cfg_.schedule_t0;
    const uint64_t max_win = cfg_.max_windows;

    std::vector<std::complex<int16_t>> stream_buf;
    int64_t stream_pos_abs = 0;
    int64_t buf_start_abs = 0;
    uint64_t k = 0;

    const size_t kReadChunk = 16384;

    while (!stop_requested_) {
        if (max_win > 0 && k >= max_win) {
            break;
        }

        int64_t pred_start = static_cast<int64_t>(std::llround(t0 + static_cast<double>(k) * period_samples));
        int64_t win_start = pred_start - static_cast<int64_t>(cfg_.pre_guard_samples);
        if (win_start < 0) {
            win_start = 0;
        }
        int64_t win_end = win_start + static_cast<int64_t>(capacity);

        bool eof_hit = false;
        while (stream_pos_abs < win_end && !stop_requested_) {
            size_t needed = static_cast<size_t>(win_end - stream_pos_abs);
            size_t to_read = std::max(kReadChunk, needed);
            size_t curr_size = stream_buf.size();
            stream_buf.resize(curr_size + to_read);

            StreamReadMeta meta;
            size_t n_read = source_->read_samples(stream_buf.data() + curr_size, to_read, meta);
            stream_buf.resize(curr_size + n_read);
            stream_pos_abs += static_cast<int64_t>(n_read);
            ledger_.total_samples_received += n_read;

            if (meta.is_gap) {
                ledger_.read_overflows++;
            }
            if (meta.is_eof) {
                eof_hit = true;
                break;
            }
            if (n_read == 0 && !meta.is_eof) {
                ledger_.read_timeouts++;
                std::this_thread::yield();
            }
        }

        if (stop_requested_) {
            ledger_.offered_windows++;
            ledger_.cancelled_windows++;
            break;
        }

        if (stream_pos_abs < win_end && eof_hit) {
            ledger_.offered_windows++;
            ledger_.cancelled_windows++;
            break;
        }

        ledger_.offered_windows++;

        Sc16WindowSlot* slot = pool_->acquire_filling_slot();
        if (!slot) {
            ledger_.dropped_admission++;
            WindowResult drop_res;
            drop_res.window_id = k;
            drop_res.state = WindowTerminalState::DroppedAdmission;
            drop_res.native_start_abs = win_start;
            drop_res.native_sample_count = capacity;
            {
                std::lock_guard<std::mutex> lock(results_mutex_);
                results_.push_back(drop_res);
            }
            if (result_cb_) {
                result_cb_(drop_res);
            }
        } else {
            ledger_.admitted_windows++;
            slot->window_id = k;
            slot->native_start_abs = win_start;
            slot->native_count = capacity;
            slot->native_rate = cfg_.native_rate;
            slot->work_rate = cfg_.work_rate;
            slot->push_time = std::chrono::steady_clock::now();

            size_t offset = static_cast<size_t>(win_start - buf_start_abs);
            std::memcpy(slot->sc16_samples.data(), stream_buf.data() + offset,
                        capacity * sizeof(std::complex<int16_t>));
            pool_->publish_ready_slot(slot);
        }

        int64_t next_pred = static_cast<int64_t>(std::llround(t0 + static_cast<double>(k + 1) * period_samples));
        int64_t next_win_start = std::max<int64_t>(0, next_pred - static_cast<int64_t>(cfg_.pre_guard_samples));
        if (next_win_start > buf_start_abs) {
            size_t discard = static_cast<size_t>(next_win_start - buf_start_abs);
            if (discard >= 32768 && discard <= stream_buf.size()) {
                stream_buf.erase(stream_buf.begin(), stream_buf.begin() + discard);
                buf_start_abs += static_cast<int64_t>(discard);
            }
        }

        ++k;
    }

    rx_finished_ = true;
}

void UhdSc16Pipeline::dsp_consumer_loop()
{
    const size_t native_capacity = cfg_.native_window_capacity();
    const bool need_resample = (std::abs(cfg_.native_rate - cfg_.work_rate) > 1e3);

    std::unique_ptr<core::RationalResampler65_48Core> resamp_65_48;
    std::unique_ptr<core::RationalResampler65_32Core> resamp_65_32;
    size_t work_capacity = native_capacity;

    if (need_resample) {
        std::string profile = cfg_.resampler_profile.empty() ? "quality_minorder" : cfg_.resampler_profile;
        if (std::abs(cfg_.native_rate - 737.28e6) < 1e5) {
            std::string path = "resampler_65_48/taps_" + profile + ".txt";
            auto taps = load_taps_f32(path);
            if (taps.empty()) {
                taps = load_taps_f32("resampler_65_48/taps_quality_minorder.txt");
            }
            if (taps.empty()) {
                taps = load_taps_f32("resampler_65_48/taps_quality.txt");
            }
            if (taps.empty()) {
                taps.resize(65 * 64, 1.0f / 48.0f);
            }
            resamp_65_48 = std::make_unique<core::RationalResampler65_48Core>(taps);
            resamp_65_48->set_num_workers(cfg_.resampler_workers);
            work_capacity = core::RationalResampler65_48Core::expected_output_length(
                native_capacity, taps.size()) + 256;
        } else if (std::abs(cfg_.native_rate - 491.52e6) < 1e5) {
            std::string path = "resampler_65_32/taps_" + profile + ".txt";
            auto taps = load_taps_f32(path);
            if (taps.empty()) {
                taps = load_taps_f32("resampler_65_32/taps_quality_minorder.txt");
            }
            if (taps.empty()) {
                taps = load_taps_f32("resampler_65_32/taps_quality.txt");
            }
            if (taps.empty()) {
                taps.resize(65 * 64, 1.0f / 32.0f);
            }
            resamp_65_32 = std::make_unique<core::RationalResampler65_32Core>(taps);
            resamp_65_32->set_num_workers(cfg_.resampler_workers);
            work_capacity = core::RationalResampler65_32Core::expected_output_length(
                native_capacity, taps.size()) + 256;
        } else {
            work_capacity = static_cast<size_t>(std::ceil(
                static_cast<double>(native_capacity) * (cfg_.work_rate / cfg_.native_rate))) + 512;
        }
    }

    std::vector<std::complex<float>> cf32_native(native_capacity);
    std::vector<std::complex<float>> cf32_work(work_capacity);
    std::vector<std::complex<float>> sic_out(work_capacity);
    std::vector<std::complex<float>> sic_ihat(work_capacity);

    sic::SicUnifiedScratch sic_scratch;
    sic::SicUnifiedTemplates sic_templates;
    std::vector<std::complex<float>> sic_sync_tmpl;
    std::vector<std::complex<float>> sic_preamble_tmpl;

    if (cfg_.consumer_mode == ConsumerMode::SicOnly || cfg_.consumer_mode == ConsumerMode::Both) {
        sic_sync_tmpl = load_cf32_file("reference_preamble.bin");
        if (sic_sync_tmpl.empty()) {
            sic_sync_tmpl = load_cf32_file("uwb_radar/tx_998p4.cf32");
        }
        if (sic_sync_tmpl.size() >= 1016) {
            sic_sync_tmpl.resize(1016);
        } else {
            sic_sync_tmpl.assign(1016, std::complex<float>(1.0f, 0.0f));
        }

        sic_preamble_tmpl.reserve(64 * 1016);
        for (int rep = 0; rep < 64; ++rep) {
            sic_preamble_tmpl.insert(sic_preamble_tmpl.end(),
                                     sic_sync_tmpl.begin(), sic_sync_tmpl.end());
        }

        sic_templates.sync = sic_sync_tmpl.data();
        sic_templates.sync_len = sic_sync_tmpl.size();
        sic_templates.preamble = sic_preamble_tmpl.data();
        sic_templates.preamble_len = sic_preamble_tmpl.size();

        prepare_sic_unified(cfg_.sic_cfg, sic_templates, sic_scratch);
    }

    radar::RadarCirCoreScratch radar_scratch;
    std::vector<std::complex<float>> radar_sync_tmpl;

    if (cfg_.consumer_mode == ConsumerMode::SensingCir || cfg_.consumer_mode == ConsumerMode::Both) {
        radar_sync_tmpl = load_cf32_file("reference_preamble.bin");
        if (radar_sync_tmpl.empty()) {
            radar_sync_tmpl = load_cf32_file("uwb_radar/tx_998p4.cf32");
        }
        if (radar_sync_tmpl.size() >= 1016) {
            radar_sync_tmpl.resize(1016);
        } else {
            radar_sync_tmpl.assign(1016, std::complex<float>(1.0f, 0.0f));
        }

        prepare_radar_cir_core(cfg_.radar_cfg, radar_sync_tmpl.data(),
                               cfg_.radar_cfg.samples_per_symbol,
                               cfg_.radar_cfg.cir_pre,
                               cfg_.radar_cfg.cir_post,
                               radar_scratch);
    }

    const float scale = (cfg_.scale_mode == Sc16ScaleMode::UnitRange) ? (1.0f / 32768.0f) : 1.0f;

    while (true) {
        Sc16WindowSlot* slot = pool_->acquire_ready_slot();
        if (!slot) {
            if (rx_finished_ || stop_requested_) {
                slot = pool_->acquire_ready_slot();
                if (!slot) {
                    break;
                }
            } else {
                std::this_thread::yield();
                continue;
            }
        }

        auto t_start = std::chrono::steady_clock::now();
        WindowResult res;
        res.window_id = slot->window_id;
        res.native_start_abs = slot->native_start_abs;
        res.native_sample_count = slot->native_count;
        res.timing.queue_us = std::chrono::duration<double, std::micro>(t_start - slot->push_time).count();

        // 1 & 2. Fused SC16 -> Work Resample or Direct SC16 -> CF32 Convert
        const std::complex<float>* dsp_in = nullptr;
        size_t dsp_n = 0;

        if (need_resample) {
            auto t_resamp_start = std::chrono::steady_clock::now();
            if (resamp_65_48) {
                resamp_65_48->reset();
                size_t expected_len = core::RationalResampler65_48Core::expected_output_length(
                    slot->native_count, resamp_65_48->tap_count());
                if (cf32_work.size() < expected_len) {
                    cf32_work.resize(expected_len);
                }
                auto r = resamp_65_48->process_sc16(slot->sc16_samples.data(), slot->native_count,
                                                    scale, cf32_work.data(), cf32_work.size());
                size_t produced = r.produced;
                while (produced < expected_len) {
                    size_t n = resamp_65_48->flush(cf32_work.data() + produced, cf32_work.size() - produced);
                    if (n == 0) break;
                    produced += n;
                }
                dsp_in = cf32_work.data();
                dsp_n = produced;
            } else if (resamp_65_32) {
                resamp_65_32->reset();
                size_t expected_len = core::RationalResampler65_32Core::expected_output_length(
                    slot->native_count, resamp_65_32->tap_count());
                if (cf32_work.size() < expected_len) {
                    cf32_work.resize(expected_len);
                }
                auto r = resamp_65_32->process_sc16(slot->sc16_samples.data(), slot->native_count,
                                                    scale, cf32_work.data(), cf32_work.size());
                size_t produced = r.produced;
                while (produced < expected_len) {
                    size_t n = resamp_65_32->flush(cf32_work.data() + produced, cf32_work.size() - produced);
                    if (n == 0) break;
                    produced += n;
                }
                dsp_in = cf32_work.data();
                dsp_n = produced;
            } else {
                if (cf32_native.size() < slot->native_count) {
                    cf32_native.resize(slot->native_count);
                }
                sc16_to_cf32(slot->sc16_samples.data(), cf32_native.data(), slot->native_count, scale);
                dsp_in = cf32_native.data();
                dsp_n = slot->native_count;
            }
            auto t_resamp_end = std::chrono::steady_clock::now();
            res.timing.convert_us = 0.0;
            res.timing.resample_us = std::chrono::duration<double, std::micro>(t_resamp_end - t_resamp_start).count();
        } else {
            // Direct SC16 -> CF32 without resampling
            auto t_conv_start = std::chrono::steady_clock::now();
            if (cf32_native.size() < slot->native_count) {
                cf32_native.resize(slot->native_count);
            }
            sc16_to_cf32(slot->sc16_samples.data(), cf32_native.data(), slot->native_count, scale);
            auto t_conv_end = std::chrono::steady_clock::now();
            res.timing.convert_us = std::chrono::duration<double, std::micro>(t_conv_end - t_conv_start).count();
            res.timing.resample_us = 0.0;
            dsp_in = cf32_native.data();
            dsp_n = slot->native_count;
        }

        // 3. SIC processing
        if (cfg_.consumer_mode == ConsumerMode::SicOnly || cfg_.consumer_mode == ConsumerMode::Both) {
            auto t_sic_start = std::chrono::steady_clock::now();
            res.sic_executed = true;

            if (sic_out.size() < dsp_n) {
                sic_out.resize(dsp_n);
            }
            if (sic_ihat.size() < dsp_n) {
                sic_ihat.resize(dsp_n);
            }

            sic::SicUnifiedResult sic_res;
            sic_res.out = sic_out.data();
            sic_res.i_hat = sic_ihat.data();

            bool sic_ok = sic::sic_process_unified(
                dsp_in, dsp_n, sic_templates, cfg_.sic_cfg, sic_scratch, sic_res);

            auto t_sic_end = std::chrono::steady_clock::now();
            res.timing.sic_us = std::chrono::duration<double, std::micro>(t_sic_end - t_sic_start).count();

            if (sic_ok) {
                res.sic_committed = sic_res.committed;
                res.actual_suppression_db = static_cast<float>(sic_res.suppression_db);
                res.sic_status = sic_res.reason ? sic_res.reason : "";
                if (sic_res.committed) {
                    dsp_in = sic_out.data();
                }
            } else {
                res.sic_status = "sic_process_failed";
            }
        }

        // 4. Sensing CIR processing
        if (cfg_.consumer_mode == ConsumerMode::SensingCir || cfg_.consumer_mode == ConsumerMode::Both) {
            auto t_sens_start = std::chrono::steady_clock::now();
            res.radar_executed = true;

            double rate_ratio = cfg_.work_rate / cfg_.native_rate;
            int64_t pred_sfd = static_cast<int64_t>(std::llround(static_cast<double>(cfg_.pre_guard_samples) * rate_ratio)) +
                               static_cast<int64_t>(cfg_.radar_cfg.sync_repetitions * cfg_.radar_cfg.samples_per_symbol);

            radar::RadarCirResult cir_res;
            bool cir_ok = radar::radar_cir_one(
                dsp_in, dsp_n, pred_sfd, cfg_.radar_cfg, radar_scratch, cir_res);

            auto t_sens_end = std::chrono::steady_clock::now();
            res.timing.sensing_us = std::chrono::duration<double, std::micro>(t_sens_end - t_sens_start).count();

            if (cir_ok && cir_res.status == radar::RadarCirStatus::Ok) {
                res.radar_sfd_found = (cir_res.sfd_start_sample >= 0);
                res.radar_peak_metric = cir_res.peak_abs;
                res.radar_detected_start = cir_res.preamble_start_sample;
            } else {
                res.radar_sfd_found = false;
                res.radar_peak_metric = 0.0f;
                res.radar_detected_start = -1;
            }
        }

        auto t_end = std::chrono::steady_clock::now();
        res.timing.e2e_us = std::chrono::duration<double, std::micro>(t_end - t_start).count();
        res.state = WindowTerminalState::Completed;
        ledger_.completed_windows++;

        pool_->release_reading_slot(slot);

        {
            std::lock_guard<std::mutex> lock(results_mutex_);
            results_.push_back(res);
        }
        if (result_cb_) {
            result_cb_(res);
        }
    }
}

} // namespace pipeline
} // namespace uwb
} // namespace gr
