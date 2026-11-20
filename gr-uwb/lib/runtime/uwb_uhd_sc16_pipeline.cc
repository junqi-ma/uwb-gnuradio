/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * UHD SC16 Direct RX and Multi-Core Stream Pipeline implementation.
 * Bypasses GNU Radio scheduler and PMT/PDU messaging overhead.
 */

#include <openuwb/runtime/uhd_sc16_pipeline.h>
#include <openuwb/radio/fixed_tone_cancel_source.h>
#include <openuwb/dsp/rational_resampler_core.h>
#include <openuwb/dsp/resample_window.h>
#include <openuwb/support/defaults.h>
#include <openuwb/support/log.h>
#include <volk/volk.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <filesystem>
#include <fstream>
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

#if defined(UWB_HAVE_UHD) && (UWB_HAVE_UHD)
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/stream.hpp>
#include <uhd/types/metadata.hpp>
#include <uhd/types/stream_cmd.hpp>
#include <uhd/exception.hpp>
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

// Load one shaped SYNC waveform (1016 samples) from the shared test data.
// The returned vector is always exactly 1016 samples long; each DSP worker
// calls this to obtain its own private copy (contract C4: no shared state).
// `override_path` (when non-empty) is loaded verbatim and must already hold
// one SYNC symbol on the 998.4 MS/s work grid.
void load_sync_pulse_template(std::vector<std::complex<float>>& tmpl,
                              const std::string& override_path = std::string())
{
    if (!override_path.empty()) {
        tmpl.clear();
        std::ifstream f(override_path, std::ios::binary | std::ios::ate);
        if (f) {
            const auto bytes = static_cast<size_t>(f.tellg());
            if (bytes % sizeof(std::complex<float>) == 0) {
                f.seekg(0);
                tmpl.resize(bytes / sizeof(std::complex<float>));
                f.read(reinterpret_cast<char*>(tmpl.data()),
                       static_cast<std::streamsize>(bytes));
            }
        }
        if (tmpl.size() >= 1016) {
            tmpl.resize(1016);
            return;
        }
        tmpl.assign(1016, std::complex<float>(1.0f, 0.0f));
        return;
    }
    tmpl = load_cf32_file("reference_preamble.bin");
    if (tmpl.empty()) {
        tmpl = load_cf32_file("uwb_radar/tx_998p4.cf32");
    }
    if (tmpl.size() >= 1016) {
        tmpl.resize(1016);
    } else {
        tmpl.assign(1016, std::complex<float>(1.0f, 0.0f));
    }
}

// Resolve the resampler tap file for a profile, falling back to the two
// historical defaults exactly like the single-consumer implementation did.
std::vector<float> load_resampler_taps(const std::string& dir,
                                       const std::string& profile)
{
    auto taps = load_taps_f32(dir + "/taps_" + profile + ".txt");
    if (taps.empty()) {
        taps = load_taps_f32(dir + "/taps_quality_minorder.txt");
    }
    if (taps.empty()) {
        taps = load_taps_f32(dir + "/taps_quality.txt");
    }
    return taps;
}

inline uint64_t elapsed_ns(std::chrono::steady_clock::time_point a,
                           std::chrono::steady_clock::time_point b)
{
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(b - a).count());
}

// Measures how long a mutex acquisition actually blocked and accumulates it as
// nanoseconds.  With wait_ns == nullptr this is a plain std::lock_guard, so the
// disabled path costs one predictable branch.
class ScopedLockWait {
public:
    ScopedLockWait(std::mutex& m, std::atomic<uint64_t>* wait_ns)
        : mutex_(m), wait_ns_(wait_ns)
    {
        if (!wait_ns_) {
            mutex_.lock();
            return;
        }
        const auto t0 = std::chrono::steady_clock::now();
        mutex_.lock();
        wait_ns_->fetch_add(elapsed_ns(t0, std::chrono::steady_clock::now()),
                            std::memory_order_relaxed);
    }

    ~ScopedLockWait() { mutex_.unlock(); }

    ScopedLockWait(const ScopedLockWait&) = delete;
    ScopedLockWait& operator=(const ScopedLockWait&) = delete;

private:
    std::mutex& mutex_;
    std::atomic<uint64_t>* wait_ns_;
};

// Peak tracker.  Compare-exchange (not load/store) because several DSP workers
// can contribute to the same peak counter concurrently.
inline void note_peak(std::atomic<uint64_t>& peak, uint64_t value)
{
    uint64_t cur = peak.load(std::memory_order_relaxed);
    while (value > cur &&
           !peak.compare_exchange_weak(
               cur, value, std::memory_order_relaxed, std::memory_order_relaxed)) {
    }
}

// ---------------------------------------------------------------------------
// Phase-C resampler kernel experiment selector
// ---------------------------------------------------------------------------
// These are switches on HOW the 65 outputs are computed inside the resampler
// core, never on WHAT is computed; every non-"none" variant must remain
// bit-identical to the default kernel.  The selector is validated once in
// initialize() and re-parsed (cheaply) by each worker.
enum class ResamplerExperiment {
    None,
    FixedPhase,
    Invalid
};

// "convert_cache" and "both" (the C2 candidate and its combination with C1)
// were measured, NOT retained and REMOVED in the 2026-11-15 native cleanup;
// they now parse as Invalid so callers fail closed with an explicit message
// instead of being silently accepted.
ResamplerExperiment parse_resampler_experiment(const std::string& name)
{
    if (name.empty() || name == "none") {
        return ResamplerExperiment::None;
    }
    if (name == "fixed_phase") {
        return ResamplerExperiment::FixedPhase;
    }
    return ResamplerExperiment::Invalid;
}

const char* resampler_experiment_name(ResamplerExperiment exp)
{
    switch (exp) {
    case ResamplerExperiment::FixedPhase:
        return "fixed_phase";
    case ResamplerExperiment::None:
    case ResamplerExperiment::Invalid:
        break;
    }
    return "none";
}

// Apply the already-validated experiment to one worker's private core.  With
// the default "none" no setter is called, so the default path stays
// instruction-for-instruction identical to the pre-experiment pipeline.
template <typename ResamplerCore>
void apply_resampler_experiment(ResamplerCore& core, bool fixed_phase)
{
    if (fixed_phase) {
        core.set_fixed_phase_schedule(true);
    }
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

bool ReplaySc16Source::start_stream()
{
    // Paced replay is relative to the instant streaming starts, never to
    // open(): the pipeline opens the source before its worker-ready barrier,
    // so reusing open()'s timestamp would release a multi-millisecond burst at
    // full speed.  Unpaced replay is unaffected.
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
        const size_t channel = cfg_.uhd_channel.empty()
            ? 0 : static_cast<size_t>(std::stoul(cfg_.uhd_channel));
        impl_->usrp->set_rx_rate(cfg_.native_rate, channel);
        actual_rate_ = impl_->usrp->get_rx_rate(channel);
        impl_->usrp->set_rx_freq(cfg_.uhd_center_freq, channel);
        impl_->usrp->set_rx_gain(cfg_.uhd_gain, channel);
        if (!cfg_.uhd_antenna.empty()) {
            impl_->usrp->set_rx_antenna(cfg_.uhd_antenna, channel);
        }
        UWB_LOG_INFO("uhd_pipeline",
                     "UHD RX configured: channel="
                         << channel
                         << " antenna=" << impl_->usrp->get_rx_antenna(channel)
                         << " rate=" << actual_rate_
                         << " frequency=" << impl_->usrp->get_rx_freq(channel)
                         << " gain=" << impl_->usrp->get_rx_gain(channel));

        uhd::stream_args_t stream_args("sc16", "sc16");
        if (!cfg_.uhd_channel.empty()) {
            stream_args.channels = {static_cast<size_t>(std::stoul(cfg_.uhd_channel))};
        }
        impl_->rx_stream = impl_->usrp->get_rx_stream(stream_args);

        // STREAM_MODE_START_CONTINUOUS is deliberately NOT issued here: the
        // pipeline calls start_stream() only after every DSP worker is ready,
        // so the device cannot queue samples behind the barrier.
        eof_ = false;
        impl_->current_sample_abs = 0;
        return true;
    } catch (const std::exception& e) {
        impl_.reset();
        return false;
    }
}

bool UhdDeviceSc16Source::start_stream()
{
    if (!impl_ || !impl_->rx_stream) {
        return false;
    }
    try {
        uhd::stream_cmd_t cmd(uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS);
        cmd.stream_now = true;
        impl_->rx_stream->issue_stream_cmd(cmd);
        return true;
    } catch (const std::exception& e) {
        UWB_LOG_ERR("uhd_pipeline",
                    "UhdDeviceSc16Source: issue_stream_cmd(START_CONTINUOUS) "
                    "failed: "
                        << e.what());
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

bool UhdDeviceSc16Source::start_stream()
{
    // No UHD support compiled in: nothing can be started, but this is a passive
    // no-op rather than an error, matching the previous "open() already
    // failed" behaviour.
    return true;
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

int UhdSc16Pipeline::effective_dsp_workers() const
{
    return cfg_.dsp_workers > 1 ? cfg_.dsp_workers : 1;
}

bool UhdSc16Pipeline::initialize(std::unique_ptr<ISc16StreamSource> custom_source)
{
    // Ensure VOLK alignment mask is initialized when running outside GNU Radio scheduler
    (void)volk_get_alignment();

    const int n_workers = effective_dsp_workers();

    // Fail closed (contract §1.4): an explicit affinity list must describe
    // exactly one core per DSP worker.  Silently degrading (e.g. pinning only
    // the first N workers) would hide a configuration error and destroy the
    // core isolation the caller asked for.
    if (!cfg_.cpu_cores_dsp.empty() &&
        static_cast<int>(cfg_.cpu_cores_dsp.size()) != n_workers) {
        UWB_LOG_ERR("uhd_pipeline",
                    "UhdSc16Pipeline: cpu_cores_dsp has "
                        << cfg_.cpu_cores_dsp.size() << " entries but dsp_workers="
                        << n_workers
                        << "; exactly one affinity entry per DSP worker is required");
        return false;
    }

    // Phase-C resampler kernel experiment selector: fail closed and loud.  An
    // unknown value is reported exactly once here (not once per worker) and
    // normalised to the default, so it can never be silently accepted as some
    // other experiment.
    {
        const ResamplerExperiment exp =
            parse_resampler_experiment(cfg_.resampler_experiment);
        if (exp == ResamplerExperiment::Invalid) {
            UWB_LOG_WARN("uhd_pipeline",
                         "UhdSc16Pipeline: unknown or removed resampler_experiment '"
                             << cfg_.resampler_experiment
                             << "' (accepted: none|fixed_phase; convert_cache/both were "
                                "removed in the 2026-11-15 native cleanup); "
                                "falling back to 'none'");
        }
        cfg_.resampler_experiment = resampler_experiment_name(exp);
    }

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

    // Fixed-frequency CW cancellation: an explicit opt-in decorator that sits
    // between the source and the window assembler.  When disabled nothing is
    // wrapped, so the original data path stays bit-identical.
    fixed_tone_source_ = nullptr;
    if (cfg_.fixed_tone_cancel_enable) {
        core::FixedToneCancelConfig tone_cfg;
        tone_cfg.enable = true;
        tone_cfg.sample_rate = cfg_.native_rate;
        tone_cfg.tone_frequency_hz = cfg_.fixed_tone_cancel_frequency_hz;
        tone_cfg.period_s = cfg_.fixed_tone_cancel_period_s;
        tone_cfg.training_s = cfg_.fixed_tone_cancel_training_s;
        tone_cfg.crossfade_samples = cfg_.fixed_tone_cancel_crossfade_samples;
        auto stage = std::make_unique<FixedToneCancelSc16Source>(
            std::move(source_),
            tone_cfg,
            cfg_.fixed_tone_cancel_block_samples,
            cfg_.fixed_tone_cancel_queue_blocks);
        fixed_tone_source_ = stage.get();
        source_ = std::move(stage);
    }

    // Contract C1: one lock-free SPSC ring per worker.  pools_[w] is produced
    // only by rx_thread_ and consumed only by dsp_threads_[w], so the
    // single-producer / single-consumer guarantee is preserved per pool.
    const size_t slot_cap = cfg_.native_window_capacity();
    pools_.clear();
    pools_.reserve(static_cast<size_t>(n_workers));
    for (int w = 0; w < n_workers; ++w) {
        pools_.push_back(
            std::make_unique<Sc16WindowRingPool>(cfg_.pool_capacity, slot_cap));
    }

    // Contract C3: burst workers always run the flat single-threaded AVX2
    // macroblock kernel (set_num_workers(1) per worker core); the former
    // resampler_workers config entry was always forced to 1 here and has been
    // REMOVED in the 2026-11-15 native cleanup.

    // Restart safety: the reorder buffer starts empty and anchored at window 0.
    {
        std::lock_guard<std::mutex> lk(commit_mutex_);
        next_commit_id_ = 0;
        buffered_result_count_ = 0;
        publish_in_progress_ = false;
        pending_results_.clear();
        results_.clear();
    }

    // A2: a fresh ready epoch.  Nothing is started here -- initialize() only
    // prepares memory and the source; start() spawns the threads.
    {
        std::lock_guard<std::mutex> lk(ready_mutex_);
        worker_ready_.assign(static_cast<size_t>(n_workers), WorkerReadyInfo{});
        // Empty until a worker actually applies its experiment switch, so
        // applied_resampler_experiment() reports "empty before start".
        applied_resampler_experiment_.assign(static_cast<size_t>(n_workers),
                                             std::string());
    }

    // B2: make sure the diagnostic IQ dump directory exists before any worker
    // tries to write into it (single-threaded here, so no creation race).
    if (!cfg_.iq_dump_dir.empty() && cfg_.iq_dump_windows > 0) {
        std::error_code ec;
        std::filesystem::create_directories(cfg_.iq_dump_dir, ec);
        if (ec) {
            UWB_LOG_WARN("uhd_pipeline",
                         "UhdSc16Pipeline: cannot create iq_dump_dir '"
                             << cfg_.iq_dump_dir << "': " << ec.message()
                             << " (IQ dumps will fail)");
        }
    }

    if (!source_->open()) {
        return false;
    }

    // R2: a successful open() starts a fresh source lifecycle -- it has neither
    // started streaming nor failed yet, and any message from an earlier run on
    // this object is stale.
    source_started_.store(false, std::memory_order_relaxed);
    source_failed_.store(false, std::memory_order_relaxed);
    {
        std::lock_guard<std::mutex> lk(source_error_mutex_);
        source_error_.clear();
    }

    return true;
}

void UhdSc16Pipeline::start()
{
    if (running_.exchange(true)) {
        return;
    }

    const int n_workers = effective_dsp_workers();

    // A2: every start() begins a brand-new ready epoch.  Doing this under
    // ready_mutex_ together with stop_requested_ makes "reset the epoch" and
    // "clear the stop" atomic with respect to a concurrent wait_ready()/stop().
    {
        std::lock_guard<std::mutex> lk(ready_mutex_);
        stop_requested_.store(false, std::memory_order_relaxed);
        if (static_cast<int>(worker_ready_.size()) != n_workers) {
            worker_ready_.assign(static_cast<size_t>(n_workers), WorkerReadyInfo{});
        } else {
            for (auto& info : worker_ready_) {
                info = WorkerReadyInfo{};
            }
        }
        // Same epoch reset for the experiment record: a restarted run must not
        // report the previous run's applied value before its workers republish.
        if (static_cast<int>(applied_resampler_experiment_.size()) != n_workers) {
            applied_resampler_experiment_.assign(static_cast<size_t>(n_workers),
                                                 std::string());
        } else {
            for (auto& applied : applied_resampler_experiment_) {
                applied.clear();
            }
        }
    }
    // Reset before any thread is spawned; release/acquire pairing with the
    // producer store and the worker load below.
    rx_finished_.store(false, std::memory_order_relaxed);

    // R2: every start() is a new run epoch for the source too.  Worker readiness
    // is reset above; the source has not started yet and cannot have failed.
    source_started_.store(false, std::memory_order_relaxed);
    source_failed_.store(false, std::memory_order_relaxed);
    {
        std::lock_guard<std::mutex> lk(source_error_mutex_);
        source_error_.clear();
    }

    // Restart safety: drop any thread handles left over from a previous run.
    dsp_threads_.clear();

    rx_thread_ = std::thread(&UhdSc16Pipeline::rx_producer_loop, this);
#if defined(__linux__) && defined(_GNU_SOURCE)
    if (cfg_.cpu_core_rx >= 0) {
        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        CPU_SET(cfg_.cpu_core_rx, &cpuset);
        pthread_setaffinity_np(rx_thread_.native_handle(), sizeof(cpu_set_t), &cpuset);
    }
#endif

    // The DSP workers apply their own affinity so that affinity_requested /
    // affinity_applied can be reported truthfully from the thread that will
    // actually run the DSP work.
    dsp_threads_.reserve(static_cast<size_t>(n_workers));
    for (int w = 0; w < n_workers; ++w) {
        dsp_threads_.emplace_back(&UhdSc16Pipeline::dsp_consumer_loop, this, w);
    }
}

void UhdSc16Pipeline::stop()
{
    {
        // Taking the lock (instead of a bare atomic store) prevents a lost
        // wake-up when a waiter is between its predicate check and its wait.
        std::lock_guard<std::mutex> lk(ready_mutex_);
        stop_requested_.store(true, std::memory_order_relaxed);
    }
    ready_cv_.notify_all();
    {
        std::lock_guard<std::mutex> lk(commit_mutex_);
    }
    commit_cv_.notify_all();
    // Unblock a source that waits on its own condition variable (the
    // fixed-tone stage does) so wait() cannot deadlock behind a read.
    if (source_) {
        source_->request_stop();
    }
}

void UhdSc16Pipeline::wait()
{
    if (rx_thread_.joinable()) {
        rx_thread_.join();
    }
    for (auto& th : dsp_threads_) {
        if (th.joinable()) {
            th.join();
        }
    }
    dsp_threads_.clear();

    // Safety net: an admitted window must always reach a terminal state, even
    // if the consumer that owned it exited before draining its pool.
    recover_orphaned_windows();

    // Contract §1.3: after every thread has stopped, force out any tail holes
    // left by max_windows / stop() in ascending window_id order.
    flush_pending_results();

    running_ = false;
    // Every ready state is terminal now; release any wait_ready() caller.
    ready_cv_.notify_all();
    if (source_) {
        // HEAD semantics: wait() closes the source.  A second start() on the
        // same object therefore sees EOF; call initialize() again (it re-opens
        // the source) to run another pass over the input.
        source_->close();
    }
}

// ---------------------------------------------------------------------------
// Ready barrier / failure propagation (A2)
// ---------------------------------------------------------------------------
bool UhdSc16Pipeline::all_ready_locked() const
{
    if (worker_ready_.empty()) {
        return false;
    }
    for (const auto& info : worker_ready_) {
        if (info.state != WorkerReadyState::Ready) {
            return false;
        }
    }
    return true;
}

bool UhdSc16Pipeline::any_failed_locked() const
{
    for (const auto& info : worker_ready_) {
        if (info.state == WorkerReadyState::Failed) {
            return true;
        }
    }
    return false;
}

// R2: ready_ok() answers exactly one question -- did the DSP workers finish
// their readiness barrier?  It deliberately says nothing about the source:
// "all workers ready" stays true even after a source that started later died.
// Callers that gate on overall health must combine it with failed().
bool UhdSc16Pipeline::ready_ok() const
{
    std::lock_guard<std::mutex> lk(ready_mutex_);
    return all_ready_locked() && !any_failed_locked();
}

// R2: overall failure state.  A worker Failed state and a fatal source error
// are independent facts that both make the run unusable, so failed() reports
// either of them.
bool UhdSc16Pipeline::failed() const
{
    {
        std::lock_guard<std::mutex> lk(ready_mutex_);
        if (any_failed_locked()) {
            return true;
        }
    }
    return source_failed();
}

std::string UhdSc16Pipeline::init_error() const
{
    {
        std::lock_guard<std::mutex> lk(ready_mutex_);
        for (const auto& info : worker_ready_) {
            if (info.state == WorkerReadyState::Failed) {
                return info.error;
            }
        }
    }
    // R2: no worker error -- fall back to a source start/fatal error so
    // init_error() is never silently empty when the run actually failed.
    return source_error();
}

// ---------------------------------------------------------------------------
// R2: source lifecycle (started / failed / error)
//
// start_stream() runs on the RX thread behind the worker-ready barrier, so a
// UHD stream-command failure used to be visible only as a stderr line.  These
// accessors separate "the source never started" from "the source started and
// then failed" and are safe to read after wait() (the RX thread writes the
// message under source_error_mutex_ and publishes source_failed_ with release).
// ---------------------------------------------------------------------------
bool UhdSc16Pipeline::source_started() const
{
    return source_started_.load(std::memory_order_acquire);
}

bool UhdSc16Pipeline::source_failed() const
{
    return source_failed_.load(std::memory_order_acquire);
}

std::string UhdSc16Pipeline::source_error() const
{
    std::lock_guard<std::mutex> lk(source_error_mutex_);
    return source_error_;
}

void UhdSc16Pipeline::set_source_error(const std::string& msg)
{
    {
        std::lock_guard<std::mutex> lk(source_error_mutex_);
        // Keep the first (root cause) message; later symptoms must not hide it.
        if (source_error_.empty()) {
            source_error_ = msg;
        }
    }
    source_failed_.store(true, std::memory_order_release);
}

std::string UhdSc16Pipeline::applied_resampler_experiment(int worker_id) const
{
    if (worker_id < 0) {
        return std::string();
    }
    std::lock_guard<std::mutex> lk(ready_mutex_);
    if (static_cast<size_t>(worker_id) >= applied_resampler_experiment_.size()) {
        return std::string();
    }
    return applied_resampler_experiment_[static_cast<size_t>(worker_id)];
}

bool UhdSc16Pipeline::wait_ready(unsigned timeout_ms)
{
    std::unique_lock<std::mutex> lk(ready_mutex_);
    auto resolved = [this] {
        return stop_requested_.load(std::memory_order_relaxed) ||
               any_failed_locked() || all_ready_locked();
    };

    if (timeout_ms == 0) {
        ready_cv_.wait(lk, resolved);
    } else if (!ready_cv_.wait_for(lk, std::chrono::milliseconds(timeout_ms), resolved)) {
        return false; // timeout
    }
    // Report the current state: true only if every worker is Ready and none
    // failed.  A stop that arrived before the workers finished keeping state
    // Pending therefore returns false; a later call (e.g. after wait()) simply
    // observes the final state.
    return all_ready_locked() && !any_failed_locked();
}

void UhdSc16Pipeline::publish_worker_state(int worker_id,
                                           WorkerReadyState state,
                                           const std::string& err)
{
    if (worker_id < 0) {
        return;
    }
    {
        std::lock_guard<std::mutex> lk(ready_mutex_);
        if (static_cast<size_t>(worker_id) >= worker_ready_.size()) {
            worker_ready_.resize(static_cast<size_t>(worker_id) + 1);
        }
        WorkerReadyInfo& info = worker_ready_[static_cast<size_t>(worker_id)];
        info.state = state;
        info.error = err;
    }
    ready_cv_.notify_all();
}

bool UhdSc16Pipeline::wait_for_workers_ready()
{
    std::unique_lock<std::mutex> lk(ready_mutex_);
    ready_cv_.wait(lk, [this] {
        return stop_requested_.load(std::memory_order_relaxed) ||
               any_failed_locked() || all_ready_locked();
    });
    if (stop_requested_.load(std::memory_order_relaxed) || any_failed_locked()) {
        return false;
    }
    return all_ready_locked();
}

void UhdSc16Pipeline::recover_orphaned_windows()
{
    for (size_t w = 0; w < pools_.size(); ++w) {
        while (Sc16WindowSlot* slot = pools_[w]->acquire_ready_slot()) {
            WindowResult res;
            res.window_id = slot->window_id;
            res.worker_id = static_cast<int>(w);
            res.native_start_abs = slot->native_start_abs;
            res.native_sample_count = slot->native_count;
            res.rx_push_time = slot->push_time;
            res.state = WindowTerminalState::ProcessingFailed;
            res.sic_status = "orphaned_after_worker_exit";
            pools_[w]->release_reading_slot(slot);
            {
                std::lock_guard<std::mutex> lk(ledger_mutex_);
                ledger_.processing_failed++;
            }
            commit_result(static_cast<int>(w), res);
        }
    }
}

// ---------------------------------------------------------------------------
// Diagnostics
// ---------------------------------------------------------------------------
const RxDiagnostics& UhdSc16Pipeline::rx_diagnostics() const
{
    auto& v = rx_diag_view_;
    v.enabled = cfg_.enable_rx_diagnostics;
    v.read_calls = rx_acc_.read_calls.load(std::memory_order_relaxed);
    v.samples_read = rx_acc_.samples_read.load(std::memory_order_relaxed);
    v.bytes_read = rx_acc_.bytes_read.load(std::memory_order_relaxed);
    v.windows_published = rx_acc_.windows_published.load(std::memory_order_relaxed);
    v.windows_dropped = rx_acc_.windows_dropped.load(std::memory_order_relaxed);
    v.slot_copy_bytes = rx_acc_.slot_copy_bytes.load(std::memory_order_relaxed);
    v.stream_buf_resize_count =
        rx_acc_.stream_buf_resize_count.load(std::memory_order_relaxed);
    v.stream_buf_zero_init_bytes =
        rx_acc_.stream_buf_zero_init_bytes.load(std::memory_order_relaxed);
    v.compact_move_bytes = rx_acc_.compact_move_bytes.load(std::memory_order_relaxed);
    v.max_buffered_samples =
        rx_acc_.max_buffered_samples.load(std::memory_order_relaxed);
    v.read_us = 1e-3 * static_cast<double>(rx_acc_.read_ns.load(std::memory_order_relaxed));
    v.assemble_us =
        1e-3 * static_cast<double>(rx_acc_.assemble_ns.load(std::memory_order_relaxed));
    v.slot_copy_us =
        1e-3 * static_cast<double>(rx_acc_.slot_copy_ns.load(std::memory_order_relaxed));
    v.dispatch_us =
        1e-3 * static_cast<double>(rx_acc_.dispatch_ns.load(std::memory_order_relaxed));
    v.ledger_wait_us =
        1e-3 * static_cast<double>(rx_acc_.ledger_wait_ns.load(std::memory_order_relaxed));
    return v;
}

const CommitDiagnostics& UhdSc16Pipeline::commit_diagnostics() const
{
    auto& v = commit_diag_view_;
    v.enabled = cfg_.enable_commit_diagnostics;
    v.commit_calls = commit_acc_.commit_calls.load(std::memory_order_relaxed);
    v.results_emitted = commit_acc_.results_emitted.load(std::memory_order_relaxed);
    v.max_out_of_order_span =
        commit_acc_.max_out_of_order_span.load(std::memory_order_relaxed);
    v.max_pending_results =
        commit_acc_.max_pending_results.load(std::memory_order_relaxed);
    v.reorder_backpressure_waits =
        commit_acc_.reorder_backpressure_waits.load(std::memory_order_relaxed);
    v.reorder_overflow_drops =
        commit_acc_.reorder_overflow_drops.load(std::memory_order_relaxed);
    v.lock_wait_us =
        1e-3 * static_cast<double>(commit_acc_.lock_wait_ns.load(std::memory_order_relaxed));
    v.reorder_us =
        1e-3 * static_cast<double>(commit_acc_.reorder_ns.load(std::memory_order_relaxed));
    v.callback_us =
        1e-3 * static_cast<double>(commit_acc_.callback_ns.load(std::memory_order_relaxed));
    return v;
}

void UhdSc16Pipeline::reset_diagnostics()
{
    auto zero = [](std::atomic<uint64_t>& a) { a.store(0, std::memory_order_relaxed); };

    zero(rx_acc_.read_calls);
    zero(rx_acc_.samples_read);
    zero(rx_acc_.bytes_read);
    zero(rx_acc_.windows_published);
    zero(rx_acc_.windows_dropped);
    zero(rx_acc_.slot_copy_bytes);
    zero(rx_acc_.stream_buf_resize_count);
    zero(rx_acc_.stream_buf_zero_init_bytes);
    zero(rx_acc_.compact_move_bytes);
    zero(rx_acc_.max_buffered_samples);
    zero(rx_acc_.read_ns);
    zero(rx_acc_.assemble_ns);
    zero(rx_acc_.slot_copy_ns);
    zero(rx_acc_.dispatch_ns);
    zero(rx_acc_.ledger_wait_ns);

    zero(commit_acc_.commit_calls);
    zero(commit_acc_.results_emitted);
    zero(commit_acc_.max_out_of_order_span);
    zero(commit_acc_.max_pending_results);
    zero(commit_acc_.reorder_backpressure_waits);
    zero(commit_acc_.reorder_overflow_drops);
    zero(commit_acc_.lock_wait_ns);
    zero(commit_acc_.reorder_ns);
    zero(commit_acc_.callback_ns);

    zero(iq_dump_started_);
    zero(iq_dump_written_);
}

bool UhdSc16Pipeline::write_iq_dump(uint64_t window_id,
                                    const std::complex<float>* data,
                                    size_t n) const
{
    // Diagnostic dumps carry their own, non-parsing-hostile tag: the comparison
    // tool requires win_<id>_<tag>.cf32 with a non-empty tag.
    const std::string tag = cfg_.iq_dump_tag.empty() ? std::string("dump") : cfg_.iq_dump_tag;
    const std::string path = cfg_.iq_dump_dir + "/win_" + std::to_string(window_id) +
                             "_" + tag + ".cf32";

    std::ofstream f(path, std::ios::binary | std::ios::trunc);
    if (!f) {
        UWB_LOG_ERR("uhd_pipeline",
                    "UhdSc16Pipeline: cannot open IQ dump '" << path << "'");
        return false;
    }
    f.write(reinterpret_cast<const char*>(data),
            static_cast<std::streamsize>(n * sizeof(std::complex<float>)));
    if (!f) {
        UWB_LOG_ERR("uhd_pipeline",
                    "UhdSc16Pipeline: failed writing IQ dump '" << path << "' ("
                        << n << " samples)");
        return false;
    }
    f.close();
    if (!f) {
        UWB_LOG_ERR("uhd_pipeline",
                    "UhdSc16Pipeline: failed closing IQ dump '" << path << "'");
        return false;
    }
    return true;
}

void UhdSc16Pipeline::rx_producer_loop()
{
    // A2 ready barrier: do not touch the source -- and in particular do not let
    // a UHD device start streaming -- until every DSP worker published Ready.
    // A failed worker (or a stop() while waiting) aborts before any read, so
    // nothing was admitted and the ledger stays trivially conserved.
    if (!wait_for_workers_ready()) {
        rx_finished_.store(true, std::memory_order_release);
        return;
    }
    if (!source_->start_stream()) {
        UWB_LOG_ERR("uhd_pipeline",
                    "UhdSc16Pipeline: source start_stream() failed; no windows "
                    "will be produced");
        // R2: a start failure is a fatal run failure, not an empty-but-healthy
        // run.  Ready states stay terminal (workers are untouched); the source
        // failure is what failed()/init_error() now report.
        set_source_error("source start_stream() failed");
        rx_finished_.store(true, std::memory_order_release);
        return;
    }
    // R2: publish "streaming" only after the source accepted the start request.
    source_started_.store(true, std::memory_order_release);

    const bool diag = cfg_.enable_rx_diagnostics;
    std::atomic<uint64_t>* ledger_wait_ns = diag ? &rx_acc_.ledger_wait_ns : nullptr;
    const size_t capacity = cfg_.native_window_capacity();
    const double period_samples = cfg_.period_s * cfg_.native_rate;
    const double t0 = cfg_.schedule_t0;
    const uint64_t max_win = cfg_.max_windows;
    const int n_workers = effective_dsp_workers();

    // ---------------------------------------------------------------------
    // Phase B RX buffering: ONE fixed allocation plus an explicit valid
    // cursor.  The historical code grew/shrunk a vector per read (paying a
    // full value-initialisation of the region the source was about to
    // overwrite) and memmoved the residue with erase().
    //
    // State (all indices are into `storage`, all absolute values are native
    // sample indices on the same axis as cfg_.schedule_t0):
    //   storage           : allocated once, exactly storage_cap samples
    //   valid_begin       : index of the first valid (already read) sample
    //   valid_n           : number of valid samples
    //   valid_start_abs   : absolute index of storage[valid_begin]
    //   read_end_abs      : valid_start_abs + valid_n == absolute read end
    // Nothing outside [valid_begin, valid_begin + valid_n) is ever read or
    // copied: the tail past valid_n is uninitialised scratch space that the
    // source overwrites directly.
    //
    // Sizing formula (documented contract):
    //   read_slice  = kReadChunk                      (>= 1)
    //   reserve     = max(read_slice, ceil(capacity/2))
    //   storage_cap = capacity + 2 * reserve
    // Why that is sufficient: a window needs at most `capacity` samples and
    // one read overshoots by at most read_slice, so the live region never
    // exceeds capacity + read_slice.  storage_cap leaves at least `reserve`
    // samples of headroom even in that worst case, so after one compaction a
    // full read slice always fits: one bounded, low-frequency memmove instead
    // of an erase per window.  Adjacent windows (period >= capacity, the
    // measured production geometry) drop the whole live region every window
    // and therefore never compact at all.
    // ---------------------------------------------------------------------
    const size_t kReadChunk = 16384;
    const size_t read_slice = std::max<size_t>(kReadChunk, 1);
    const size_t reserve = std::max(read_slice, (capacity + 1) / 2);
    const size_t storage_cap = capacity + 2 * reserve;

    std::vector<std::complex<int16_t>> storage(storage_cap);
    if (diag) {
        // Truthful accounting: one allocation that value-initialises the whole
        // storage once per run (the old code did this per read).
        rx_acc_.stream_buf_resize_count.fetch_add(1, std::memory_order_relaxed);
        rx_acc_.stream_buf_zero_init_bytes.fetch_add(
            static_cast<uint64_t>(storage_cap) * sizeof(std::complex<int16_t>),
            std::memory_order_relaxed);
    }

    size_t valid_begin = 0;
    size_t valid_n = 0;
    int64_t valid_start_abs = 0;
    int64_t read_end_abs = 0;
    uint64_t k = 0;

    // R1: the ONE release point.  Drop every live sample strictly before
    // min(limit_abs, read_end_abs) and keep the cursor bookkeeping consistent:
    //   valid_start_abs advances to the first sample that is still needed,
    //   valid_n shrinks by exactly the number of dropped samples,
    //   read_end_abs stays valid_start_abs + valid_n (never past what was read),
    //   valid_begin is rebased to 0 for free when nothing is live any more.
    // limit_abs is always a window start, and window starts are monotone, so
    // nothing that a future window needs is ever dropped.  Callers: once per
    // window and once at the top of EVERY read iteration below.
    auto release_before = [&](int64_t limit_abs) {
        if (limit_abs <= valid_start_abs) {
            return;
        }
        const int64_t keep_from = std::min<int64_t>(limit_abs, read_end_abs);
        if (keep_from <= valid_start_abs) {
            return;
        }
        const size_t drop = static_cast<size_t>(keep_from - valid_start_abs);
        valid_begin += drop;
        valid_n -= drop;
        valid_start_abs = keep_from;
        read_end_abs = valid_start_abs + static_cast<int64_t>(valid_n);
        if (valid_n == 0) {
            // Nothing live: restart at index 0 for free (no move, no copy).
            valid_begin = 0;
        }
    };

    // R1: a source that keeps answering "no samples, not EOF" must not spin
    // this thread forever.  Sixteen consecutive zero-progress reads is far
    // above anything a real source does while catching up (each call is
    // blocking) and still bounds the run.
    constexpr int kMaxZeroProgressReads = 16;

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

        // Window starts are monotone, so everything strictly before this
        // window's start can be released.  When the source has already been
        // read past win_start (overlap) the surplus is kept.
        //
        // R1 invariant (and the reason a gap larger than the whole buffer is
        // harmless): the live region is released before EVERY read, so it never
        // exceeds `capacity` (plus the current read slice) no matter how far
        // apart the scheduled windows are.  Reading through a gap therefore
        // recycles the same storage slice after slice; it is dropped again at
        // the top of the next iteration, long before `room` can reach 0.
        release_before(win_start);

        bool eof_hit = false;
        bool source_fatal = false;
        int zero_progress_reads = 0;
        while (read_end_abs < win_end && !stop_requested_) {
            // R1: release again on entry to every read iteration.  Without this
            // the gap samples pile up until `room` hits 0 and `to_read` becomes
            // 0, which ReplaySc16Source reports as EOF (permanent, wrong) and
            // which makes a zero-progress fake source spin forever.
            release_before(win_start);

            const size_t needed = static_cast<size_t>(win_end - read_end_abs);
            size_t to_read = std::max(read_slice, needed);
            size_t room = storage_cap - (valid_begin + valid_n);
            if (room < to_read && valid_begin > 0) {
                // Compact: move ONLY the live tail that future windows still
                // need, and account for exactly those bytes.
                const size_t moved_bytes =
                    valid_n * sizeof(std::complex<int16_t>);
                std::chrono::steady_clock::time_point t_move{};
                if (diag) {
                    t_move = std::chrono::steady_clock::now();
                }
                if (moved_bytes > 0) {
                    std::memmove(storage.data(), storage.data() + valid_begin, moved_bytes);
                }
                if (diag) {
                    rx_acc_.assemble_ns.fetch_add(
                        elapsed_ns(t_move, std::chrono::steady_clock::now()),
                        std::memory_order_relaxed);
                    rx_acc_.compact_move_bytes.fetch_add(
                        static_cast<uint64_t>(moved_bytes), std::memory_order_relaxed);
                }
                valid_begin = 0;
                room = storage_cap - valid_n;
            }
            if (to_read > room) {
                // Bounded slice: the read loop simply runs another iteration.
                to_read = room;
            }
            if (to_read == 0) {
                // R1: never ask the source for zero samples.  ReplaySc16Source
                // treats max_samples == 0 as EOF and latches eof_ forever, so a
                // zero-length "probe" silently truncates a run that still has
                // data.  Reaching this point with the release helper above means
                // a state that must not happen (no room even after compaction);
                // fail loudly instead of pretending the input ended.
                UWB_LOG_ERR("uhd_pipeline",
                            "UhdSc16Pipeline: RX storage cannot fit a read for "
                            "window "
                                << k << " (storage_cap=" << storage_cap
                                << " valid_begin=" << valid_begin
                                << " valid_n=" << valid_n << "); aborting the run");
                set_source_error("RX storage produced a zero-length read request");
                source_fatal = true;
                break;
            }

            std::chrono::steady_clock::time_point t_read{};
            if (diag) {
                t_read = std::chrono::steady_clock::now();
            }
            StreamReadMeta meta;
            // Read straight into the uninitialised tail: no value
            // initialisation of the bytes the source is about to write.
            size_t n_read = source_->read_samples(
                storage.data() + valid_begin + valid_n, to_read, meta);
            if (diag) {
                rx_acc_.read_ns.fetch_add(
                    elapsed_ns(t_read, std::chrono::steady_clock::now()),
                    std::memory_order_relaxed);
                rx_acc_.read_calls.fetch_add(1, std::memory_order_relaxed);
                rx_acc_.samples_read.fetch_add(n_read, std::memory_order_relaxed);
                rx_acc_.bytes_read.fetch_add(
                    static_cast<uint64_t>(n_read) * sizeof(std::complex<int16_t>),
                    std::memory_order_relaxed);
            }
            valid_n += n_read;
            read_end_abs += static_cast<int64_t>(n_read);
            if (diag) {
                note_peak(rx_acc_.max_buffered_samples,
                          static_cast<uint64_t>(valid_n));
            }

            {
                ScopedLockWait lk(ledger_mutex_, ledger_wait_ns);
                ledger_.total_samples_received += n_read;
                if (meta.is_gap) {
                    ledger_.read_overflows++;
                }
                if (n_read == 0 && !meta.is_eof) {
                    ledger_.read_timeouts++;
                }
            }

            // R2: a failed source is fatal, never a normal end of input.  Check
            // it before is_eof so a source that reports both (the fatal error
            // that made it return 0) is not silently downgraded to "drained".
            if (source_failed()) {
                source_fatal = true;
                break;
            }
            // A source may also report a fatal error on its own (the
            // fixed-tone stage wraps another source and must not disguise a
            // reader-thread failure as a short read).
            {
                std::string wrapped_error;
                if (source_->fatal_error(&wrapped_error)) {
                    set_source_error(wrapped_error.empty()
                                         ? std::string("stream source reported a fatal error")
                                         : wrapped_error);
                    source_fatal = true;
                    break;
                }
            }
            if (meta.is_eof) {
                eof_hit = true;
                break;
            }
            if (n_read == 0) {
                // R1: bounded tolerance for non-EOF zero-progress reads.
                ++zero_progress_reads;
                if (zero_progress_reads >= kMaxZeroProgressReads) {
                    UWB_LOG_ERR("uhd_pipeline",
                                "UhdSc16Pipeline: source made no progress after "
                                    << zero_progress_reads
                                    << " consecutive reads for window " << k
                                    << "; aborting the run");
                    set_source_error(
                        "source returned no samples and no EOF for " +
                        std::to_string(zero_progress_reads) + " consecutive reads");
                    source_fatal = true;
                    break;
                }
                std::this_thread::yield();
            } else {
                zero_progress_reads = 0;
            }
        }

        if (source_fatal) {
            // Offered + cancelled: the window was scheduled but never admitted,
            // and the run stops so the caller sees a failed (non-zero) process
            // instead of a short, seemingly-successful one.
            ScopedLockWait lk(ledger_mutex_, ledger_wait_ns);
            ledger_.offered_windows++;
            ledger_.cancelled_windows++;
            break;
        }

        if (stop_requested_) {
            ScopedLockWait lk(ledger_mutex_, ledger_wait_ns);
            ledger_.offered_windows++;
            ledger_.cancelled_windows++;
            break;
        }

        if (read_end_abs < win_end && eof_hit) {
            ScopedLockWait lk(ledger_mutex_, ledger_wait_ns);
            ledger_.offered_windows++;
            ledger_.cancelled_windows++;
            break;
        }

        // Contract: only already-written samples may be copied out.  This is
        // guaranteed by construction (the read loop only exits once
        // read_end_abs >= win_end, and the cursor never advances past
        // win_start); the explicit check turns a future regression into a loud
        // cancelled window instead of a silent copy out of uninitialised
        // storage.
        if (win_start < valid_start_abs ||
            static_cast<size_t>(win_start - valid_start_abs) + capacity > valid_n) {
            UWB_LOG_ERR("uhd_pipeline",
                        "UhdSc16Pipeline: internal RX buffer invariant violated for "
                        "window "
                            << k << " (win_start=" << win_start
                            << " valid_start_abs=" << valid_start_abs
                            << " valid_n=" << valid_n << "); window cancelled");
            ScopedLockWait lk(ledger_mutex_, ledger_wait_ns);
            ledger_.offered_windows++;
            ledger_.cancelled_windows++;
            break;
        }

        {
            ScopedLockWait lk(ledger_mutex_, ledger_wait_ns);
            ledger_.offered_windows++;
        }

        // Bounded reorder window & pre-admission backpressure / overflow drop:
        // keep capacity reserved for the earliest unfinished window
        // (next_commit_id_) when out-of-order span or buffered results reach
        // max_reorder_pending_windows.
        if (cfg_.max_reorder_pending_windows > 0) {
            bool drop_for_reorder_limit = false;
            {
                std::unique_lock<std::mutex> lk(commit_mutex_);
                auto limit_reached = [&]() {
                    const uint64_t span =
                        (k >= next_commit_id_) ? (k - next_commit_id_) : 0;
                    return span >= cfg_.max_reorder_pending_windows ||
                           buffered_result_count_ >= cfg_.max_reorder_pending_windows;
                };
                if (limit_reached()) {
                    if (cfg_.backpressure_on_reorder_limit) {
                        if (diag || cfg_.enable_commit_diagnostics) {
                            commit_acc_.reorder_backpressure_waits.fetch_add(
                                1, std::memory_order_relaxed);
                        }
                        commit_cv_.wait(lk, [&]() {
                            return stop_requested_.load(std::memory_order_relaxed) ||
                                   !limit_reached();
                        });
                    } else {
                        drop_for_reorder_limit = true;
                        if (diag || cfg_.enable_commit_diagnostics) {
                            commit_acc_.reorder_overflow_drops.fetch_add(
                                1, std::memory_order_relaxed);
                        }
                    }
                }
            }
            if (stop_requested_.load(std::memory_order_relaxed)) {
                ScopedLockWait lk(ledger_mutex_, ledger_wait_ns);
                ledger_.cancelled_windows++;
                break;
            }
            if (drop_for_reorder_limit) {
                {
                    ScopedLockWait lk(ledger_mutex_, ledger_wait_ns);
                    ledger_.dropped_admission++;
                }
                if (diag) {
                    rx_acc_.windows_dropped.fetch_add(1, std::memory_order_relaxed);
                }
                WindowResult drop_res;
                drop_res.window_id = k;
                drop_res.state = WindowTerminalState::DroppedAdmission;
                drop_res.native_start_abs = win_start;
                drop_res.native_sample_count = capacity;
                drop_res.rx_push_time = std::chrono::steady_clock::now();
                commit_result(-1, drop_res);
                ++k;
                continue;
            }
        }

        // ------------------------------------------------------------------
        // Dispatch (contract §1.2): prefer pools_[k % N], then probe the
        // remaining pools in order.  Only a fully saturated ring set counts
        // as a dropped admission.  rx_thread_ is the sole producer, so every
        // pool stays a strict 1P/1C SPSC ring and no CAS is needed.
        // ------------------------------------------------------------------
        Sc16WindowSlot* slot = nullptr;
        int pool_idx = -1;
        std::chrono::steady_clock::time_point t_probe{};
        if (diag) {
            t_probe = std::chrono::steady_clock::now();
        }
        for (int probe = 0; probe < n_workers; ++probe) {
            const int cand = static_cast<int>(
                (k + static_cast<uint64_t>(probe)) % static_cast<uint64_t>(n_workers));
            Sc16WindowSlot* candidate =
                pools_[static_cast<size_t>(cand)]->acquire_filling_slot();
            if (candidate) {
                slot = candidate;
                pool_idx = cand;
                break;
            }
        }
        if (diag) {
            rx_acc_.dispatch_ns.fetch_add(
                elapsed_ns(t_probe, std::chrono::steady_clock::now()),
                std::memory_order_relaxed);
        }

        if (!slot) {
            {
                ScopedLockWait lk(ledger_mutex_, ledger_wait_ns);
                ledger_.dropped_admission++;
            }
            if (diag) {
                rx_acc_.windows_dropped.fetch_add(1, std::memory_order_relaxed);
            }
            WindowResult drop_res;
            drop_res.window_id = k;
            drop_res.state = WindowTerminalState::DroppedAdmission;
            drop_res.native_start_abs = win_start;
            drop_res.native_sample_count = capacity;
            // Nothing was queued for this window: only rx_push_time is
            // meaningful, the remaining stage stamps stay default.
            drop_res.rx_push_time = std::chrono::steady_clock::now();
            // commit_result() stamps worker_id = -1 for producer-side drops
            // and keeps the drop in window_id order with the DSP results.
            commit_result(-1, drop_res);
        } else {
            {
                ScopedLockWait lk(ledger_mutex_, ledger_wait_ns);
                ledger_.admitted_windows++;
            }
            slot->window_id = k;
            slot->native_start_abs = win_start;
            slot->native_count = capacity;
            slot->native_rate = cfg_.native_rate;
            slot->work_rate = cfg_.work_rate;
            // Single timestamp shared with WindowResult::rx_push_time so
            // queue_us keeps its historical meaning.
            slot->push_time = std::chrono::steady_clock::now();

            std::chrono::steady_clock::time_point t_copy{};
            if (diag) {
                t_copy = std::chrono::steady_clock::now();
            }
            // The window is fully inside the live region: the read loop above
            // stopped only at read_end_abs >= win_end, and release_data()
            // guarantees valid_start_abs <= win_start.  Copy exactly
            // `capacity` samples; nothing uninitialised is ever touched.
            const size_t src_off =
                valid_begin + static_cast<size_t>(win_start - valid_start_abs);
            std::memcpy(slot->sc16_samples.data(), storage.data() + src_off,
                        capacity * sizeof(std::complex<int16_t>));
            if (diag) {
                rx_acc_.slot_copy_ns.fetch_add(
                    elapsed_ns(t_copy, std::chrono::steady_clock::now()),
                    std::memory_order_relaxed);
                rx_acc_.slot_copy_bytes.fetch_add(
                    static_cast<uint64_t>(capacity) * sizeof(std::complex<int16_t>),
                    std::memory_order_relaxed);
            }

            if (diag) {
                t_copy = std::chrono::steady_clock::now();
            }
            // Publish on the pool the slot actually came from.
            pools_[static_cast<size_t>(pool_idx)]->publish_ready_slot(slot);
            if (diag) {
                rx_acc_.dispatch_ns.fetch_add(
                    elapsed_ns(t_copy, std::chrono::steady_clock::now()),
                    std::memory_order_relaxed);
                rx_acc_.windows_published.fetch_add(1, std::memory_order_relaxed);
            }
        }

        // No post-window compaction here any more: the release step at the top
        // of the next iteration drops exactly the samples that window will
        // never need again, and the storage is compacted only when a read
        // would otherwise not fit (counted in compact_move_bytes).

        ++k;
    }

    // Release: everything this producer admitted has already been published
    // to the ring pools, so any worker that observes this flag is guaranteed
    // to see those slots and can drain them before exiting.
    rx_finished_.store(true, std::memory_order_release);
}

void UhdSc16Pipeline::dsp_consumer_loop(int worker_id)
{
    // ------------------------------------------------------------------
    // A2: affinity first, then every allocation / template load / scratch
    // preparation, then publish Ready.  The RX thread does not read a single
    // sample before this thread is ready.
    // ------------------------------------------------------------------
    // Contract §1.4: explicit list wins, otherwise derive cpu_core_dsp + w,
    // otherwise stay unpinned.
    int affinity_core = -1;
    if (static_cast<size_t>(worker_id) < cfg_.cpu_cores_dsp.size()) {
        affinity_core = cfg_.cpu_cores_dsp[static_cast<size_t>(worker_id)];
    } else if (cfg_.cpu_core_dsp >= 0) {
        affinity_core = cfg_.cpu_core_dsp + worker_id;
    }
    const bool affinity_requested = (affinity_core >= 0);
    bool affinity_applied = false;
    if (affinity_requested) {
#if defined(__linux__) && defined(_GNU_SOURCE)
        if (affinity_core < CPU_SETSIZE) {
            cpu_set_t cpuset;
            CPU_ZERO(&cpuset);
            CPU_SET(affinity_core, &cpuset);
            const int rc = pthread_setaffinity_np(
                pthread_self(), sizeof(cpu_set_t), &cpuset);
            if (rc == 0) {
                affinity_applied = true;
            } else {
                // Never report a silent success: the caller must be able to see
                // that the requested isolation is not in effect.
                UWB_LOG_WARN("uhd_pipeline",
                             "UhdSc16Pipeline: dsp worker " << worker_id
                                 << " affinity to core " << affinity_core
                                 << " failed: " << std::strerror(rc));
            }
        } else {
            UWB_LOG_WARN("uhd_pipeline",
                         "UhdSc16Pipeline: dsp worker " << worker_id
                             << " affinity core " << affinity_core
                             << " is out of range (CPU_SETSIZE=" << CPU_SETSIZE << ")");
        }
#else
        UWB_LOG_WARN("uhd_pipeline",
                     "UhdSc16Pipeline: dsp worker " << worker_id
                         << " affinity to core " << affinity_core
                         << " requested but thread affinity is not supported on this "
                            "platform");
#endif
    }
    {
        std::lock_guard<std::mutex> lk(ready_mutex_);
        if (static_cast<size_t>(worker_id) < worker_ready_.size()) {
            WorkerReadyInfo& info = worker_ready_[static_cast<size_t>(worker_id)];
            info.affinity_requested = affinity_requested;
            info.affinity_applied = affinity_applied;
            info.affinity_core = affinity_core;
        }
    }

    // Contract C1: this thread is the only consumer of its own ring pool and
    // never touches another worker's pool.  Contract C4: every piece of DSP
    // state below is a function-local object owned by this worker, so there is
    // zero shared write and zero false sharing between workers.
    Sc16WindowRingPool* my_pool = pools_[static_cast<size_t>(worker_id)].get();

    const size_t native_capacity = cfg_.native_window_capacity();
    const bool need_resample = (std::abs(cfg_.native_rate - cfg_.work_rate) > 1e3);

    // Phase-C experiment selector: initialize() already validated and
    // normalised cfg_.resampler_experiment.  Resolved once per worker; with the
    // default "none" no setter is called below, so the default path stays
    // instruction-for-instruction identical.
    const ResamplerExperiment resampler_experiment =
        parse_resampler_experiment(cfg_.resampler_experiment);
    const bool exp_fixed_phase =
        (resampler_experiment == ResamplerExperiment::FixedPhase);
    // True once this worker owns a resampler core that received the switch.
    // A worker with no core (native == work, or a rate without a dedicated
    // kernel) reports "none" because nothing was actually applied.
    bool experiment_applied_to_core = false;

    std::unique_ptr<core::RationalResampler65_48Core> resamp_65_48;
    std::unique_ptr<core::RationalResampler65_32Core> resamp_65_32;
    size_t work_capacity = native_capacity;

    std::vector<std::complex<float>> cf32_native;
    std::vector<std::complex<float>> cf32_work;
    std::vector<std::complex<float>> sic_out;
    std::vector<std::complex<float>> sic_ihat;

    sic::SicUnifiedScratch sic_scratch;
    sic::SicUnifiedTemplates sic_templates;
    std::vector<std::complex<float>> sic_sync_tmpl;
    std::vector<std::complex<float>> sic_preamble_tmpl;

    radar::RadarCirCoreScratch radar_scratch;
    std::vector<std::complex<float>> radar_sync_tmpl;

    try {
        // QA-only failure injection point (default empty => no-op, no lock).
        if (cfg_.test_worker_init_hook) {
            cfg_.test_worker_init_hook(worker_id);
        }

        if (need_resample) {
            const std::string profile =
                cfg_.resampler_profile.empty() ? "quality_minorder" : cfg_.resampler_profile;
            if (std::abs(cfg_.native_rate - 737.28e6) < 1e5) {
                auto taps = load_resampler_taps("resampler_65_48", profile);
                if (taps.empty()) {
                    // Historical fallback.  It is a flat boxcar, NOT the designed
                    // 65/48 filter: it doubles the resample cost and destroys the
                    // passband, so every downstream detector fails.  Make the
                    // silent degradation at least visible.
                    UWB_LOG_WARN("uhd_pipeline",
                                 "UhdSc16Pipeline: warning: resampler_65_48 profile '"
                                     << profile
                                     << "' taps not found (cwd-dependent); falling back to a "
                                     << "flat " << (65 * 64)
                                     << "-tap placeholder filter, DSP output "
                                     << "will be wrong");
                    taps.resize(65 * 64, 1.0f / 48.0f);
                }
                resamp_65_48 = std::make_unique<core::RationalResampler65_48Core>(taps);
                // Contract C3: a burst worker must never nest the resampler
                // thread pool.  num_workers == 1 forces the flat
                // run_macroblock_sc16 + fir_avx2_sc16_4x path on this core.
                resamp_65_48->set_num_workers(1);
                // Phase-C experiment switch (default "none" => no call).
                apply_resampler_experiment(*resamp_65_48, exp_fixed_phase);
                if (exp_fixed_phase) {
                    resamp_65_48->set_kernel("transposed");
                }
                experiment_applied_to_core = true;
                work_capacity = core::RationalResampler65_48Core::expected_output_length(
                    native_capacity, taps.size()) + 256;
            } else if (std::abs(cfg_.native_rate - 491.52e6) < 1e5) {
                auto taps = load_resampler_taps("resampler_65_32", profile);
                if (taps.empty()) {
                    taps.resize(65 * 64, 1.0f / 32.0f);
                }
                resamp_65_32 = std::make_unique<core::RationalResampler65_32Core>(taps);
                resamp_65_32->set_num_workers(1);
                // Phase-C experiment switch (default "none" => no call).
                apply_resampler_experiment(*resamp_65_32, exp_fixed_phase);
                if (exp_fixed_phase) {
                    resamp_65_32->set_kernel("transposed");
                }
                experiment_applied_to_core = true;
                work_capacity = core::RationalResampler65_32Core::expected_output_length(
                    native_capacity, taps.size()) + 256;
            } else {
                work_capacity = static_cast<size_t>(std::ceil(
                    static_cast<double>(native_capacity) * (cfg_.work_rate / cfg_.native_rate))) + 512;
            }
        }

        cf32_native.assign(native_capacity, std::complex<float>(0.0f, 0.0f));
        cf32_work.assign(work_capacity, std::complex<float>(0.0f, 0.0f));
        sic_out.assign(work_capacity, std::complex<float>(0.0f, 0.0f));
        sic_ihat.assign(work_capacity, std::complex<float>(0.0f, 0.0f));

        if (cfg_.consumer_mode == ConsumerMode::SicOnly || cfg_.consumer_mode == ConsumerMode::Both) {
            load_sync_pulse_template(sic_sync_tmpl, cfg_.sic_sync_template_file);

            sic_preamble_tmpl.reserve(cfg_.sic_cfg.sync_reps * 1016);
            for (size_t rep = 0; rep < cfg_.sic_cfg.sync_reps; ++rep) {
                sic_preamble_tmpl.insert(sic_preamble_tmpl.end(),
                                         sic_sync_tmpl.begin(), sic_sync_tmpl.end());
            }

            sic_templates.sync = sic_sync_tmpl.data();
            sic_templates.sync_len = sic_sync_tmpl.size();
            sic_templates.preamble = sic_preamble_tmpl.data();
            sic_templates.preamble_len = sic_preamble_tmpl.size();

            prepare_sic_unified(cfg_.sic_cfg, sic_templates, sic_scratch);
        }

        if (cfg_.consumer_mode == ConsumerMode::SensingCir || cfg_.consumer_mode == ConsumerMode::Both) {
            load_sync_pulse_template(radar_sync_tmpl);

            prepare_radar_cir_core(cfg_.radar_cfg, radar_sync_tmpl.data(),
                                   cfg_.radar_cfg.samples_per_symbol,
                                   cfg_.radar_cfg.cir_pre,
                                   cfg_.radar_cfg.cir_post,
                                   radar_scratch);
        }
    } catch (const std::exception& e) {
        publish_worker_state(worker_id, WorkerReadyState::Failed, e.what());
        return;
    } catch (...) {
        publish_worker_state(worker_id, WorkerReadyState::Failed,
                             "unknown exception during worker initialisation");
        return;
    }

    // Everything this worker needs is in place: from here on the RX producer
    // may read the source and hand windows over.
    //
    // Record what this worker ACTUALLY applied before the ready handshake, so
    // applied_resampler_experiment(w) is stable for readers that only
    // synchronise through wait_ready() / wait().
    {
        const std::string applied =
            experiment_applied_to_core
                ? std::string(resampler_experiment_name(resampler_experiment))
                : std::string("none");
        std::lock_guard<std::mutex> lk(ready_mutex_);
        if (static_cast<size_t>(worker_id) >= applied_resampler_experiment_.size()) {
            applied_resampler_experiment_.resize(static_cast<size_t>(worker_id) + 1);
        }
        applied_resampler_experiment_[static_cast<size_t>(worker_id)] = applied;
    }
    publish_worker_state(worker_id, WorkerReadyState::Ready, std::string());

    const float scale = (cfg_.scale_mode == Sc16ScaleMode::UnitRange) ? (1.0f / 32768.0f) : 1.0f;

    // B2: one boolean, evaluated once per worker, so the disabled path costs a
    // single predictable test per window.
    const bool dump_enabled = (!cfg_.iq_dump_dir.empty() && cfg_.iq_dump_windows > 0);

    while (true) {
        Sc16WindowSlot* slot = my_pool->acquire_ready_slot();
        if (!slot) {
            // Exit gate is rx_finished_ ONLY.  stop_requested_ must not stop
            // a worker: the RX producer may already be past its own stop
            // check and still admit+publish one more window into this pool.
            // Leaving it unprocessed would break the ledger invariant
            // admitted == completed + failed + invalid_gap_after.
            if (rx_finished_.load(std::memory_order_acquire)) {
                // Double-check after observing the finish flag: the producer
                // publishes slots before setting rx_finished_, but a slot
                // could have been published between our two reads.
                slot = my_pool->acquire_ready_slot();
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
        res.worker_id = worker_id;
        res.native_start_abs = slot->native_start_abs;
        res.native_sample_count = slot->native_count;
        // Same clock reading that commit_result()/queue_us use: the RX thread
        // stored it in the slot right before publishing.
        res.rx_push_time = slot->push_time;
        res.worker_start_time = t_start;
        res.timing.queue_us = std::chrono::duration<double, std::micro>(t_start - slot->push_time).count();

        // 1 & 2. Fused SC16 -> Work Resample or Direct SC16 -> CF32 Convert
        const std::complex<float>* dsp_in = nullptr;
        size_t dsp_n = 0;

        if (need_resample) {
            auto t_resamp_start = std::chrono::steady_clock::now();
            if (resamp_65_48) {
                dsp_n = core::resample_sc16_window(*resamp_65_48,
                    slot->sc16_samples.data(), slot->native_count, scale, cf32_work);
                dsp_in = cf32_work.data();
            } else if (resamp_65_32) {
                dsp_n = core::resample_sc16_window(*resamp_65_32,
                    slot->sc16_samples.data(), slot->native_count, scale, cf32_work);
                dsp_in = cf32_work.data();
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

        // 2b. Diagnostic IQ dump (default off): the exact CF32 samples that
        // SIC/radar are about to consume, written before any of that work runs.
        // The dump duration is kept out of every service timer below and is
        // subtracted from e2e_us (it is file I/O, not pipeline work).
        double iq_dump_us = 0.0;
        if (dump_enabled &&
            (cfg_.iq_dump_window_limit == 0 ||
             res.window_id < static_cast<uint64_t>(cfg_.iq_dump_window_limit))) {
            if (iq_dump_started_.fetch_add(1, std::memory_order_relaxed) <
                static_cast<uint64_t>(cfg_.iq_dump_windows)) {
                const auto t_dump = std::chrono::steady_clock::now();
                if (write_iq_dump(res.window_id, dsp_in, dsp_n)) {
                    iq_dump_written_.fetch_add(1, std::memory_order_relaxed);
                }
                iq_dump_us = std::chrono::duration<double, std::micro>(
                                 std::chrono::steady_clock::now() - t_dump)
                                 .count();
            }
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
        double e2e_us = std::chrono::duration<double, std::micro>(t_end - t_start).count();
        if (iq_dump_us > 0.0) {
            // The diagnostic dump is not pipeline work: keep it out of e2e_us
            // (sic_us/sensing_us/resample_us/convert_us never included it).
            e2e_us -= iq_dump_us;
            if (e2e_us < 0.0) {
                e2e_us = 0.0;
            }
        }
        res.timing.e2e_us = e2e_us;
        res.worker_end_time = t_end;
        res.state = WindowTerminalState::Completed;

        // Release the slot before committing: the RX producer may refill this
        // ring immediately while this worker is busy ordering the result.
        my_pool->release_reading_slot(slot);

        {
            std::lock_guard<std::mutex> lk(ledger_mutex_);
            ledger_.completed_windows++;
        }

        // Contract §1.3: never hold ledger_mutex_ while committing (no lock
        // nesting); commit_result owns commit_mutex_ and the reorder buffer.
        commit_result(worker_id, res);
    }
}

// ---------------------------------------------------------------------------
// In-order result commit (reorder buffer + outside-lock publication)
// ---------------------------------------------------------------------------
void UhdSc16Pipeline::emit_batch_unlocked(
    std::vector<std::unique_ptr<WindowResult>>& batch,
    bool diag,
    std::atomic<uint64_t>* lock_wait_ns)
{
    if (batch.empty()) {
        return;
    }
    uint64_t callback_ns = 0;
    for (auto& item : batch) {
        if (!item) {
            continue;
        }
        if (diag) {
            commit_acc_.results_emitted.fetch_add(1, std::memory_order_relaxed);
        }
        item->commit_time = std::chrono::steady_clock::now();
        if (result_cb_) {
            // Invoked outside commit_mutex_ by the single active publisher so
            // observers always see strict window_id order without blocking
            // concurrent commit_result() insertions.  *item lives on the heap
            // inside `batch` for the entire callback.
            const auto t_cb = diag ? std::chrono::steady_clock::now()
                                   : std::chrono::steady_clock::time_point{};
            result_cb_(*item);
            const auto t_cb_done = std::chrono::steady_clock::now();
            if (diag) {
                callback_ns += elapsed_ns(t_cb, t_cb_done);
            }
            // Stamped after the callback returned so retained results record
            // the full commit boundary (including callback duration).
            item->commit_time = t_cb_done;
        }
    }
    if (diag && callback_ns > 0) {
        commit_acc_.callback_ns.fetch_add(callback_ns, std::memory_order_relaxed);
    }
    if (cfg_.retain_results) {
        ScopedLockWait lk(commit_mutex_, lock_wait_ns);
        const auto t_ret = diag ? std::chrono::steady_clock::now()
                                : std::chrono::steady_clock::time_point{};
        for (auto& item : batch) {
            if (item) {
                results_.push_back(std::move(*item));
            }
        }
        if (diag) {
            commit_acc_.reorder_ns.fetch_add(
                elapsed_ns(t_ret, std::chrono::steady_clock::now()),
                std::memory_order_relaxed);
        }
    }
}

void UhdSc16Pipeline::commit_result(int worker_id, const WindowResult& res)
{
    const bool diag = cfg_.enable_commit_diagnostics;
    std::atomic<uint64_t>* lock_wait_ns = diag ? &commit_acc_.lock_wait_ns : nullptr;
    if (diag) {
        commit_acc_.commit_calls.fetch_add(1, std::memory_order_relaxed);
    }

    auto stored = std::make_unique<WindowResult>(res);
    stored->worker_id = worker_id;

    {
        ScopedLockWait lk(commit_mutex_, lock_wait_ns);
        const auto t_reorder0 = diag ? std::chrono::steady_clock::now()
                                     : std::chrono::steady_clock::time_point{};
        // pending_results_ is indexed relative to next_commit_id_, so index 0
        // is always the next window_id to emit.
        if (res.window_id < next_commit_id_) {
            return;
        }
        const size_t rel = static_cast<size_t>(res.window_id - next_commit_id_);
        if (diag) {
            note_peak(commit_acc_.max_out_of_order_span,
                      static_cast<uint64_t>(rel));
        }
        if (pending_results_.size() <= rel) {
            pending_results_.resize(rel + 1);
        }
        if (!pending_results_[rel]) {
            ++buffered_result_count_;
        }
        pending_results_[rel] = std::move(stored);
        if (diag) {
            note_peak(commit_acc_.max_pending_results,
                      static_cast<uint64_t>(buffered_result_count_));
            commit_acc_.reorder_ns.fetch_add(
                elapsed_ns(t_reorder0, std::chrono::steady_clock::now()),
                std::memory_order_relaxed);
        }
        if (publish_in_progress_) {
            return;
        }
        publish_in_progress_ = true;
    }

    std::vector<std::unique_ptr<WindowResult>> batch;
    while (true) {
        batch.clear();
        {
            ScopedLockWait lk(commit_mutex_, lock_wait_ns);
            const auto t_reorder0 = diag ? std::chrono::steady_clock::now()
                                         : std::chrono::steady_clock::time_point{};
            size_t flushed = 0;
            while (flushed < pending_results_.size() && pending_results_[flushed]) {
                batch.push_back(std::move(pending_results_[flushed]));
                ++flushed;
            }
            if (flushed > 0) {
                pending_results_.erase(
                    pending_results_.begin(),
                    pending_results_.begin() + static_cast<ptrdiff_t>(flushed));
                next_commit_id_ += flushed;
                buffered_result_count_ =
                    (buffered_result_count_ >= flushed)
                        ? (buffered_result_count_ - flushed)
                        : 0;
            }
            if (diag) {
                commit_acc_.reorder_ns.fetch_add(
                    elapsed_ns(t_reorder0, std::chrono::steady_clock::now()),
                    std::memory_order_relaxed);
            }
            if (batch.empty()) {
                publish_in_progress_ = false;
                commit_cv_.notify_all();
                return;
            }
        }
        commit_cv_.notify_all();
        emit_batch_unlocked(batch, diag, lock_wait_ns);
    }
}

void UhdSc16Pipeline::flush_pending_results()
{
    const bool diag = cfg_.enable_commit_diagnostics;
    std::atomic<uint64_t>* lock_wait_ns = diag ? &commit_acc_.lock_wait_ns : nullptr;
    std::vector<std::unique_ptr<WindowResult>> batch;
    {
        ScopedLockWait lk(commit_mutex_, lock_wait_ns);
        const auto t_reorder0 = diag ? std::chrono::steady_clock::now()
                                     : std::chrono::steady_clock::time_point{};
        // Contract §1.3: after all threads have joined, force out whatever is
        // still buffered (max_windows tail / stop()) in ascending window_id
        // order.  Holes are windows that never produced a result (cancelled
        // before admission); they are skipped without shifting later results.
        size_t advanced = 0;
        for (size_t i = 0; i < pending_results_.size(); ++i) {
            if (!pending_results_[i]) {
                continue;
            }
            batch.push_back(std::move(pending_results_[i]));
            advanced = i + 1;
        }
        if (advanced > 0) {
            pending_results_.erase(
                pending_results_.begin(),
                pending_results_.begin() + static_cast<ptrdiff_t>(advanced));
            next_commit_id_ += advanced;
        }
        pending_results_.clear();
        buffered_result_count_ = 0;
        publish_in_progress_ = false;
        if (diag) {
            commit_acc_.reorder_ns.fetch_add(
                elapsed_ns(t_reorder0, std::chrono::steady_clock::now()),
                std::memory_order_relaxed);
        }
    }
    commit_cv_.notify_all();
    emit_batch_unlocked(batch, diag, lock_wait_ns);
}

} // namespace pipeline
} // namespace uwb
} // namespace gr
