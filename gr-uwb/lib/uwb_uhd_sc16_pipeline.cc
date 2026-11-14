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
#include <gnuradio/uwb/uwb_detector_core.h>
#include <gnuradio/uwb/uwb_pipeline_output_writer.h>
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
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
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

[[maybe_unused]] std::vector<std::complex<float>> load_cf32_file(const std::string& rel)
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

bool mode_runs_sic(ConsumerMode mode)
{
    return mode == ConsumerMode::SicOnly ||
           mode == ConsumerMode::Both ||
           mode == ConsumerMode::SicPlusComm;
}

bool mode_runs_sensing(ConsumerMode mode)
{
    return mode == ConsumerMode::SensingCir ||
           mode == ConsumerMode::Both;
}

bool mode_runs_comm(ConsumerMode mode)
{
    return mode == ConsumerMode::CommOnly ||
           mode == ConsumerMode::SicPlusComm;
}

// Canonical consumer-mode name written to run.json.  `SicPlusSensing` is an
// alias of `Both`, so it must not appear as a second case label.
const char* consumer_mode_name(ConsumerMode mode)
{
    switch (mode) {
    case ConsumerMode::SicOnly:
        return "sic";
    case ConsumerMode::SensingCir:
        return "sensing";
    case ConsumerMode::Both:
        return "both";
    case ConsumerMode::CommOnly:
        return "comm";
    case ConsumerMode::SicPlusComm:
        return "sic_plus_comm";
    }
    return "sic";
}

const char* sic_policy_name(sic::SicPolicy policy)
{
    switch (policy) {
    case sic::SicPolicy::PreambleOnly:
        return "preamble";
    case sic::SicPolicy::FullPacket:
        return "fullpacket";
    case sic::SicPolicy::AutoFallback:
        return "auto";
    }
    return "auto";
}

const char* sic_backend_name(sic::SicEstimatorBackend backend)
{
    switch (backend) {
    case sic::SicEstimatorBackend::DiscreteCir:
        return "discrete";
    case sic::SicEstimatorBackend::PathFractional:
        return "fractional";
    case sic::SicEstimatorBackend::Ipatov:
        return "ipatov";
    }
    return "ipatov";
}

uint64_t fnv1a_64(const uint8_t* data, size_t n)
{
    uint64_t hash = 14695981039346656037ULL;
    for (size_t i = 0; i < n; ++i) {
        hash ^= static_cast<uint64_t>(data[i]);
        hash *= 1099511628211ULL;
    }
    return hash;
}

const char* demod_status_reason(demod::DemodStatus status)
{
    switch (status) {
    case demod::DemodStatus::Success:
        return "ok";
    case demod::DemodStatus::InvalidInput:
        return "invalid_input";
    case demod::DemodStatus::TimingFailed:
        return "timing_failed";
    case demod::DemodStatus::CfoFailed:
        return "cfo_failed";
    case demod::DemodStatus::SfdFailed:
        return "sfd_failed";
    case demod::DemodStatus::PhrFailed:
        return "phr_failed";
    case demod::DemodStatus::PayloadFailed:
        return "payload_failed";
    case demod::DemodStatus::FcsFailed:
        return "fcs_failed";
    case demod::DemodStatus::QueueFull:
        return "queue_full";
    case demod::DemodStatus::InternalError:
        return "internal_error";
    case demod::DemodStatus::CirFailed:
        return "cir_failed";
    }
    return "unknown";
}

// Load one shaped SYNC waveform (1016 samples) from an explicit override file,
// or synthesize an L2-normalized 1-symbol SYNC waveform from `(code_index,
// pulse)` via `uwb_hrp_mod_core.h`.  Each DSP worker calls this independently
// to obtain its own private copy (contract C4: no shared state).
void load_or_synthesize_sync_template(std::vector<std::complex<float>>& tmpl,
                                      size_t code_index,
                                      const mod::PulseSpec& pulse,
                                      const char* sfd_mode,
                                      const std::string& override_path = std::string())
{
    constexpr size_t kSyncSamples = demod::kQm35SamplesPerSymbol; // 1016
    if (!override_path.empty()) {
        tmpl.clear();
        std::ifstream f(override_path, std::ios::binary | std::ios::ate);
        if (!f) {
            throw std::runtime_error(
                "UhdSc16Pipeline: failed to open SYNC template file '" +
                override_path + "'");
        }
        const auto bytes = static_cast<size_t>(f.tellg());
        if (bytes < kSyncSamples * sizeof(std::complex<float>) ||
            bytes % sizeof(std::complex<float>) != 0) {
            throw std::runtime_error(
                "UhdSc16Pipeline: SYNC template file '" + override_path +
                "' is invalid or shorter than 1016 CF32 samples");
        }
        f.seekg(0);
        tmpl.resize(bytes / sizeof(std::complex<float>));
        f.read(reinterpret_cast<char*>(tmpl.data()),
               static_cast<std::streamsize>(bytes));
        tmpl.resize(kSyncSamples);
        core::uwb_l2_normalize(tmpl);
        return;
    }

    mod::HrpModConfig mod_cfg;
    mod_cfg.code_index = code_index;
    mod_cfg.sync_repetitions = 1;
    mod_cfg.sfd_mode = (sfd_mode && sfd_mode[0] != '\0') ? sfd_mode : "4z2";
    mod_cfg.insert_sts = false;
    mod_cfg.pulse = pulse;

    mod::HrpModScratch mod_scratch;
    std::vector<std::complex<float>> prefix_wf(40960);
    size_t produced = 0;
    if (!mod::modulate_prefix_waveform(mod_cfg, mod_scratch, prefix_wf.data(),
                                       prefix_wf.size(), produced) ||
        produced < kSyncSamples) {
        throw std::runtime_error(
            "UhdSc16Pipeline: failed to synthesize 1-symbol SYNC template for code_index=" +
            std::to_string(code_index));
    }

    tmpl.assign(prefix_wf.begin(),
                prefix_wf.begin() + static_cast<std::ptrdiff_t>(kSyncSamples));
    core::uwb_l2_normalize(tmpl);
}

sic::SicUnifiedConfig build_effective_sic_cfg(const PipelineConfig& cfg)
{
    sic::SicUnifiedConfig out = cfg.sic_cfg;
    if (cfg.consumer_mode == ConsumerMode::SicPlusComm &&
        out.policy == sic::SicPolicy::AutoFallback) {
        // Sensing interferer in Mode B is SYNC+SFD only; default to L1 PreambleOnly.
        out.policy = sic::SicPolicy::PreambleOnly;
    }
    if (out.full_window_search) {
        out.roi_lo = 0;
        out.roi_hi = 0;
        out.sub_roi_lo = 0;
        out.sub_roi_hi = 0;
        out.observed_end_sample = 0;
        if (out.coarse_stride <= 1) {
            out.coarse_stride = 8;
        }
    }
    return out;
}

radar::RadarCirConfig build_effective_radar_cfg(const PipelineConfig& cfg)
{
    radar::RadarCirConfig out = cfg.radar_cfg;
    if (!out.use_predicted_timing &&
        (cfg.radar_full_window_search || out.full_window_search)) {
        out.full_window_search = true;
        if (out.timing_coarse_stride <= 1) {
            out.timing_coarse_stride = 8;
        }
    }
    if (out.sync_repetitions < 64 && out.cir_skip_initial > out.sync_repetitions / 4) {
        out.cir_skip_initial = out.sync_repetitions / 4;
    } else if (out.cir_skip_initial >= out.sync_repetitions) {
        out.cir_skip_initial = out.sync_repetitions / 4;
    }
    const size_t avail_reps = out.sync_repetitions - out.cir_skip_initial;
    if (out.cir_repetitions == 0 || out.cir_repetitions > avail_reps) {
        out.cir_repetitions = avail_reps;
    }
    return out;
}

demod::Qm35825Profile build_effective_comm_cfg(const PipelineConfig& cfg,
                                              size_t dsp_n,
                                              int64_t& out_seed)
{
    demod::Qm35825Profile p = cfg.comm_cfg;
    p.pulse_center_sample = mod::pulse_center_tap(cfg.comm_pulse);
    const size_t reps = std::max<size_t>(1, p.preamble_repetitions);
    p.preamble_repetitions = reps;

    if (reps < 64) {
        if (p.cir_skip_initial_repetitions > reps / 4) {
            p.cir_skip_initial_repetitions = reps / 4;
        }
    } else if (p.cir_skip_initial_repetitions >= reps) {
        p.cir_skip_initial_repetitions = std::min<size_t>(10, reps / 4);
    }
    const size_t cir_avail = reps - p.cir_skip_initial_repetitions;
    if (p.cir_repetitions == 0 || p.cir_repetitions > cir_avail) {
        p.cir_repetitions = cir_avail;
    }

    if (p.cfo_min_fit_repetitions == 0 || p.cfo_min_fit_repetitions > reps) {
        p.cfo_min_fit_repetitions = std::max<size_t>(2, std::min<size_t>(reps, 32));
    }
    if (reps < 64) {
        const size_t max_cfo_fit = std::max<size_t>(2, reps / 2);
        if (p.cfo_min_fit_repetitions > max_cfo_fit) {
            p.cfo_min_fit_repetitions = max_cfo_fit;
        }
        if (p.cfo_skip_initial_repetitions + p.cfo_min_fit_repetitions > reps) {
            p.cfo_skip_initial_repetitions =
                reps > p.cfo_min_fit_repetitions ? (reps - p.cfo_min_fit_repetitions) : 0;
        }
    } else if (p.cfo_skip_initial_repetitions >= reps) {
        p.cfo_skip_initial_repetitions =
            reps > p.cfo_min_fit_repetitions ? (reps - p.cfo_min_fit_repetitions) : 0;
    }

    if (p.min_valid_peaks == 0 || p.min_valid_peaks > reps) {
        p.min_valid_peaks = std::max<size_t>(2, std::min<size_t>(reps, 8));
    } else if (reps < 64 && p.min_valid_peaks > std::max<size_t>(2, reps / 2)) {
        p.min_valid_peaks = std::max<size_t>(2, reps / 2);
    }

    if (cfg.comm_full_window_search) {
        out_seed = static_cast<int64_t>(dsp_n / 2);
        p.timing_search_margin = dsp_n;
        p.timing_coarse_stride =
            cfg.comm_timing_coarse_stride > 0 ? cfg.comm_timing_coarse_stride : 8;
    } else {
        const double rate_ratio = cfg.work_rate / cfg.native_rate;
        out_seed = static_cast<int64_t>(
            std::llround(static_cast<double>(cfg.pre_guard_samples) * rate_ratio));
    }
    return p;
}

// Resolve the resampler tap file for a profile, falling back cleanly when a
// rate directory (e.g. resampler_65_32) does not ship equiripple variants.
std::vector<float> load_resampler_taps(const std::string& dir,
                                       const std::string& profile)
{
    auto taps = load_taps_f32(dir + "/taps_" + profile + ".txt");
    if (taps.empty() && profile == "realtime_eq16") {
        taps = load_taps_f32(dir + "/taps_realtime_minorder.txt");
    }
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
    ConvertCache,
    Both,
    Invalid
};

ResamplerExperiment parse_resampler_experiment(const std::string& name)
{
    if (name.empty() || name == "none") {
        return ResamplerExperiment::None;
    }
    if (name == "fixed_phase") {
        return ResamplerExperiment::FixedPhase;
    }
    if (name == "convert_cache") {
        return ResamplerExperiment::ConvertCache;
    }
    if (name == "both") {
        return ResamplerExperiment::Both;
    }
    return ResamplerExperiment::Invalid;
}

const char* resampler_experiment_name(ResamplerExperiment exp)
{
    switch (exp) {
    case ResamplerExperiment::FixedPhase:
        return "fixed_phase";
    case ResamplerExperiment::ConvertCache:
        return "convert_cache";
    case ResamplerExperiment::Both:
        return "both";
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
void apply_resampler_experiment(ResamplerCore& core, bool fixed_phase, bool convert_cache)
{
    if (fixed_phase) {
        core.set_fixed_phase_schedule(true);
    }
    if (convert_cache) {
        core.set_convert_cache(true);
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
        std::cerr << "UhdDeviceSc16Source: issue_stream_cmd(START_CONTINUOUS) "
                     "failed: "
                  << e.what() << std::endl;
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
    // pending_slots_ is empty by construction once wait() returned
    // (flush_pending_results() settles every entry).
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
        std::cerr << "UhdSc16Pipeline: cpu_cores_dsp has "
                  << cfg_.cpu_cores_dsp.size() << " entries but dsp_workers="
                  << n_workers
                  << "; exactly one affinity entry per DSP worker is required"
                  << std::endl;
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
            std::cerr << "UhdSc16Pipeline: unknown resampler_experiment '"
                      << cfg_.resampler_experiment
                      << "' (expected none|fixed_phase|convert_cache|both); "
                         "falling back to 'none'"
                      << std::endl;
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

    // Contract C1: one lock-free SPSC ring per worker.  pools_[w] is produced
    // only by rx_thread_ and consumed only by dsp_threads_[w], so the
    // single-producer / single-consumer guarantee is preserved per pool.
    auto_sched_stats_ = AutoScheduleStats{};
    if (cfg_.schedule_mode == ScheduleMode::AutoAcquire) {
        if (cfg_.auto_schedule.auto_interference_guards) {
            const sic::SicUnifiedConfig eff_sic = build_effective_sic_cfg(cfg_);
            InterferenceGuardSpec spec;
            spec.native_rate = cfg_.native_rate;
            spec.work_rate = cfg_.work_rate;
            spec.int_sync_reps = eff_sic.sync_reps;
            spec.int_sfd_mode = (eff_sic.sfd_mode && eff_sic.sfd_mode[0] != '\0')
                                    ? eff_sic.sfd_mode
                                    : "4z2";
            spec.int_pre_sfd_sync_reps = std::max(
                cfg_.auto_schedule.early_int_pre_sfd_sync_reps, eff_sic.fit_reps);
            spec.early_margin_work = cfg_.auto_schedule.early_int_margin_work;
            spec.late_margin_work = cfg_.auto_schedule.late_int_margin_work;

            switch (cfg_.consumer_mode) {
            case ConsumerMode::SensingCir:
            case ConsumerMode::SicPlusSensing:
                spec.target_is_comm = false;
                spec.target_sync_reps = cfg_.radar_cfg.sync_repetitions;
                spec.target_sfd_mode =
                    (cfg_.radar_cfg.sfd_mode && cfg_.radar_cfg.sfd_mode[0] != '\0')
                        ? cfg_.radar_cfg.sfd_mode
                        : "4z2";
                spec.target_pulse = cfg_.radar_pulse;
                break;
            case ConsumerMode::CommOnly:
            case ConsumerMode::SicPlusComm:
                spec.target_is_comm = true;
                spec.target_sync_reps = cfg_.comm_cfg.preamble_repetitions;
                spec.target_sfd_mode =
                    (cfg_.comm_cfg.sfd_mode && cfg_.comm_cfg.sfd_mode[0] != '\0')
                        ? cfg_.comm_cfg.sfd_mode
                        : "4z2";
                spec.target_max_psdu_bytes = cfg_.auto_schedule.target_max_psdu_bytes;
                spec.target_pulse = cfg_.comm_pulse;
                break;
            case ConsumerMode::SicOnly:
                spec.target_is_comm = false;
                spec.target_sync_reps = eff_sic.sync_reps;
                spec.target_sfd_mode = spec.int_sfd_mode;
                spec.target_pulse = eff_sic.pulse;
                break;
            }

            const AutoWindowGeometry geom = compute_interference_aware_window(spec);
            effective_pre_guard_ = geom.pre_guard_native;
            effective_capture_ = geom.capture_native;
            effective_post_guard_ = geom.post_guard_native;
        } else {
            effective_pre_guard_ = cfg_.pre_guard_samples;
            effective_capture_ = cfg_.capture_samples;
            effective_post_guard_ = cfg_.post_guard_samples;
        }

        const double eff_native_rate =
            (cfg_.native_rate > 0.0) ? cfg_.native_rate : defaults::kSampleRateHz;
        const size_t acq_pre =
            (cfg_.auto_schedule.acq_pre_guard_samples > 0)
                ? cfg_.auto_schedule.acq_pre_guard_samples
                : std::max<size_t>(effective_pre_guard_, 256);
        const size_t acq_post =
            (cfg_.auto_schedule.acq_post_guard_samples > 0)
                ? cfg_.auto_schedule.acq_post_guard_samples
                : effective_post_guard_;
        const size_t acq_cap = effective_capture_ + acq_post;
        const size_t prov_guard_samples = static_cast<size_t>(std::llround(
            std::max(0.0, cfg_.auto_schedule.provisional_guard_us) * 1e-6 *
            eff_native_rate));
        const size_t sched_max = effective_pre_guard_ + effective_capture_ +
                                 effective_post_guard_ + 2 * prov_guard_samples;
        const size_t acq_max = acq_pre + acq_cap;
        effective_slot_cap_ = std::max(sched_max, acq_max);

        auto_sched_stats_.enabled = true;
        auto_sched_stats_.effective_pre_guard = effective_pre_guard_;
        auto_sched_stats_.effective_capture = effective_capture_;
        auto_sched_stats_.effective_post_guard = effective_post_guard_;
    } else {
        effective_pre_guard_ = cfg_.pre_guard_samples;
        effective_capture_ = cfg_.capture_samples;
        effective_post_guard_ = cfg_.post_guard_samples;
        effective_slot_cap_ = cfg_.native_window_capacity();
    }

    const size_t slot_cap = effective_slot_capacity();
    pools_.clear();
    pools_.reserve(static_cast<size_t>(n_workers));
    for (int w = 0; w < n_workers; ++w) {
        pools_.push_back(
            std::make_unique<Sc16WindowRingPool>(cfg_.pool_capacity, slot_cap));
    }

    // Contract C3: burst workers run the flat single-threaded AVX2 macroblock
    // kernel, so the nested resampler pool is not only useless but harmful
    // (futex wake-up tax plus a lost 4-way register reuse).  Say so once.
    if (n_workers > 1 && cfg_.resampler_workers > 1) {
        std::cerr << "UhdSc16Pipeline: dsp_workers=" << n_workers
                  << " disables the nested resampler worker pool "
                     "(resampler_workers="
                  << cfg_.resampler_workers
                  << " ignored); each DSP worker uses the single-threaded "
                     "macroblock kernel"
                  << std::endl;
    }

    // Restart safety: the reorder buffer starts empty and anchored at window 0.
    {
        std::lock_guard<std::mutex> lk(commit_mutex_);
        next_commit_id_ = 0;
        pending_results_.clear();
        // A previous run's slots are all settled by now (wait() drained the
        // writer); drop any leftover bookkeeping so the index space restarts
        // at 0 together with pending_results_.
        pending_slots_.clear();
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
            std::cerr << "UhdSc16Pipeline: cannot create iq_dump_dir '"
                      << cfg_.iq_dump_dir << "': " << ec.message()
                      << " (IQ dumps will fail)\n";
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

    // ------------------------------------------------------------------
    // Per-window result persistence + live console log (default off)
    // ------------------------------------------------------------------
    // Observability only.  Every failure here is fail-open: the pipeline must
    // run exactly as before, so a writer that cannot start is reported once and
    // then dropped (output_writer_ stays null and the hot path costs one null
    // test).  Constructed last, after the source opened, so a failed
    // initialize() never leaves a half-started writer behind.
    if (output_writer_) {
        // Restart on the same object: drain and discard the previous writer.
        output_writer_->stop_and_drain();
        output_writer_.reset();
    }
    if (cfg_.output.enabled()) {
        const radar::RadarCirConfig eff_radar = build_effective_radar_cfg(cfg_);
        // The shared SYNC-CIR kernel always produces exactly cir_pre + cir_post
        // taps (uwb_radar_cir_core.h: `tap_count = cfg.cir_pre + cfg.cir_post`),
        // so that sum is the per-slot tap capacity.
        const size_t max_cir_taps = eff_radar.cir_pre + eff_radar.cir_post;
        // One HRP PSDU (including its 2-byte FCS) never exceeds 127 bytes.
        const size_t max_psdu = 127;

        OutputRunMeta meta;
        meta.consumer_mode = consumer_mode_name(cfg_.consumer_mode);
        meta.native_rate = cfg_.native_rate;
        meta.work_rate = cfg_.work_rate;
        meta.uhd_center_freq = cfg_.uhd_center_freq;
        meta.sic_policy = sic_policy_name(cfg_.sic_cfg.policy);
        meta.sic_backend = sic_backend_name(cfg_.sic_cfg.backend);
        meta.cir_pre = eff_radar.cir_pre;
        meta.cir_post = eff_radar.cir_post;
        // Report the code/repetition geometry of the consumer that actually
        // runs; SicOnly falls back to the SIC (interferer) configuration.
        switch (cfg_.consumer_mode) {
        case ConsumerMode::SensingCir:
        case ConsumerMode::Both:
            meta.code_index = eff_radar.code_index;
            meta.sync_repetitions = eff_radar.sync_repetitions;
            meta.samples_per_symbol = eff_radar.samples_per_symbol;
            break;
        case ConsumerMode::CommOnly:
        case ConsumerMode::SicPlusComm:
            meta.code_index = cfg_.comm_cfg.code_index;
            meta.sync_repetitions = cfg_.comm_cfg.preamble_repetitions;
            meta.samples_per_symbol = static_cast<double>(demod::kQm35SamplesPerSymbol);
            break;
        case ConsumerMode::SicOnly:
            meta.code_index = cfg_.sic_cfg.comm_code;
            meta.sync_repetitions = cfg_.sic_cfg.sync_reps;
            meta.samples_per_symbol = static_cast<double>(cfg_.sic_cfg.samples_per_symbol);
            break;
        }
        meta.max_psdu_bytes = max_psdu;

        auto writer = std::make_unique<PipelineOutputWriter>();
        if (!writer->start(cfg_.output, meta, max_cir_taps, max_psdu)) {
            std::cerr << "UhdSc16Pipeline: output writer failed to start (output_dir='"
                      << cfg_.output.output_dir << "', live_log="
                      << live_log_mode_name(cfg_.output.live_log_mode)
                      << "); continuing with output persistence disabled" << std::endl;
        } else {
            output_writer_ = std::move(writer);
        }
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
    feedback_queue_.reset();

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
    feedback_queue_.wake_all();
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

    // Contract §3.5: no frame may be lost on shutdown.  Every worker has joined
    // and the tail has been submitted above, so this is the one point where the
    // writer can be drained: it flushes and closes every file and joins its own
    // thread before returning.  Idempotent, so a second wait() is harmless.
    // (stop() alone only signals and must not drain: workers may still be
    // submitting, and draining there would drop in-flight frames.)
    if (output_writer_) {
        output_writer_->stop_and_drain();
    }

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
            res.window_kind = slot->window_kind;
            res.acquisition_epoch = slot->acquisition_epoch;
            res.schedule_generation = slot->schedule_generation;
            res.schedule_index = slot->schedule_index;
            res.schedule_state = slot->schedule_state_name;
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
OutputWriterStats UhdSc16Pipeline::output_writer_stats() const
{
    // No writer == output persistence disabled (or its start() failed
    // fail-open), which is reported as a default, disabled stats block.
    return output_writer_ ? output_writer_->stats() : OutputWriterStats{};
}

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
        std::cerr << "UhdSc16Pipeline: cannot open IQ dump '" << path << "'\n";
        return false;
    }
    f.write(reinterpret_cast<const char*>(data),
            static_cast<std::streamsize>(n * sizeof(std::complex<float>)));
    if (!f) {
        std::cerr << "UhdSc16Pipeline: failed writing IQ dump '" << path << "' ("
                  << n << " samples)\n";
        return false;
    }
    f.close();
    if (!f) {
        std::cerr << "UhdSc16Pipeline: failed closing IQ dump '" << path << "'\n";
        return false;
    }
    return true;
}

size_t UhdSc16Pipeline::effective_slot_capacity() const
{
    if (cfg_.schedule_mode == ScheduleMode::AutoAcquire && effective_slot_cap_ > 0) {
        return effective_slot_cap_;
    }
    return cfg_.native_window_capacity();
}

AutoScheduleController::TargetProfile UhdSc16Pipeline::build_target_profile() const
{
    AutoScheduleController::TargetProfile tp;
    const bool same_rate = (std::abs(cfg_.native_rate - cfg_.work_rate) <= 1.0);
    std::string tmpl_file;

    switch (cfg_.consumer_mode) {
    case ConsumerMode::SensingCir:
    case ConsumerMode::SicPlusSensing:
        tp.code_index = static_cast<int>(cfg_.radar_cfg.code_index);
        tp.sync_repetitions = cfg_.radar_cfg.sync_repetitions;
        tp.sfd_mode = (cfg_.radar_cfg.sfd_mode && cfg_.radar_cfg.sfd_mode[0] != '\0')
                          ? cfg_.radar_cfg.sfd_mode
                          : "4z2";
        tp.sfd_symbols = sfd_symbol_count_for_mode(tp.sfd_mode.c_str());
        tp.pulse = cfg_.radar_pulse;
        tp.is_comm = false;
        tmpl_file = cfg_.radar_sync_template_file;
        break;
    case ConsumerMode::CommOnly:
    case ConsumerMode::SicPlusComm:
        tp.code_index = static_cast<int>(cfg_.comm_cfg.code_index);
        tp.sync_repetitions = cfg_.comm_cfg.preamble_repetitions;
        tp.sfd_mode = (cfg_.comm_cfg.sfd_mode && cfg_.comm_cfg.sfd_mode[0] != '\0')
                          ? cfg_.comm_cfg.sfd_mode
                          : "4z2";
        tp.sfd_symbols = sfd_symbol_count_for_mode(tp.sfd_mode.c_str());
        tp.pulse = cfg_.comm_pulse;
        tp.is_comm = true;
        tmpl_file = cfg_.comm_sync_template_file;
        break;
    case ConsumerMode::SicOnly:
        tp.code_index = static_cast<int>(cfg_.sic_cfg.comm_code);
        tp.sync_repetitions = cfg_.sic_cfg.sync_reps;
        tp.sfd_mode = (cfg_.sic_cfg.sfd_mode && cfg_.sic_cfg.sfd_mode[0] != '\0')
                          ? cfg_.sic_cfg.sfd_mode
                          : "4z2";
        tp.sfd_symbols = sfd_symbol_count_for_mode(tp.sfd_mode.c_str());
        tp.pulse = cfg_.sic_cfg.pulse;
        tp.is_comm = false;
        tmpl_file = cfg_.sic_sync_template_file;
        break;
    }

    if (same_rate && !tmpl_file.empty()) {
        load_or_synthesize_sync_template(tp.native_sync_template_override,
                                         static_cast<size_t>(tp.code_index),
                                         tp.pulse,
                                         tp.sfd_mode.c_str(),
                                         tmpl_file);
    }
    return tp;
}

void UhdSc16Pipeline::rx_auto_schedule_loop()
{
    if (!wait_for_workers_ready()) {
        rx_finished_.store(true, std::memory_order_release);
        return;
    }
    if (!source_->start_stream()) {
        std::cerr << "UhdSc16Pipeline: source start_stream() failed; no windows "
                     "will be produced\n";
        set_source_error("source start_stream() failed");
        rx_finished_.store(true, std::memory_order_release);
        return;
    }
    source_started_.store(true, std::memory_order_release);

    AutoScheduleConfig acfg = cfg_.auto_schedule;
    if (acfg.period_s <= 0.0) {
        acfg.period_s = cfg_.period_s;
    }

    AutoScheduleController ctrl;
    try {
        ctrl.configure(acfg, build_target_profile(), cfg_.native_rate, cfg_.work_rate,
                       effective_pre_guard_, effective_capture_, effective_post_guard_);
    } catch (const std::exception& e) {
        set_source_error(std::string("AutoScheduleController configure failed: ") +
                         e.what());
        rx_finished_.store(true, std::memory_order_release);
        return;
    }

    const bool diag = cfg_.enable_rx_diagnostics;
    std::atomic<uint64_t>* ledger_wait_ns = diag ? &rx_acc_.ledger_wait_ns : nullptr;
    const int n_workers = effective_dsp_workers();
    const size_t pool_slot_cap = effective_slot_capacity();
    const size_t slot_cap = std::max(pool_slot_cap, ctrl.max_slot_native_capacity());
    const size_t kReadChunk = 16384;
    const size_t read_slice = std::max<size_t>(kReadChunk, 1);
    const size_t reserve = std::max(read_slice, (slot_cap + 1) / 2);
    const size_t storage_cap = slot_cap + 2 * reserve;

    std::vector<std::complex<int16_t>> storage(storage_cap);
    if (diag) {
        rx_acc_.stream_buf_resize_count.fetch_add(1, std::memory_order_relaxed);
        rx_acc_.stream_buf_zero_init_bytes.fetch_add(
            static_cast<uint64_t>(storage_cap) * sizeof(std::complex<int16_t>),
            std::memory_order_relaxed);
    }

    size_t valid_begin = 0;
    size_t valid_n = 0;
    int64_t valid_start_abs = 0;
    int64_t read_end_abs = 0;
    uint64_t next_window_id = 0;
    uint64_t samples_read_total = 0;
    uint64_t admitted_dynamic_slots = 0;
    uint64_t ingested_feedback_slots = 0;
    int acq_zero_progress_reads = 0;
    constexpr int kMaxZeroProgressReads = 16;
    const bool unpaced_replay =
        (cfg_.source_type == StreamSourceType::Replay && !cfg_.paced_replay);
    const bool pacing_for_replay =
        unpaced_replay && cfg_.auto_schedule.sync_candidate_on_unpaced_replay;

    std::vector<LockFeedback> fb_batch;
    fb_batch.reserve(32);

    auto ingest_fb_batch = [&]() {
        if (!fb_batch.empty()) {
            ingested_feedback_slots += static_cast<uint64_t>(fb_batch.size());
            ctrl.ingest_feedback(fb_batch, read_end_abs);
            fb_batch.clear();
        }
    };

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
            valid_begin = 0;
        }
    };

    auto emit_slot = [&](int64_t win_start_abs,
                         const std::complex<int16_t>* src_samples,
                         size_t sample_count,
                         WindowKind kind,
                         int64_t pred_start_native_rel,
                         size_t pre_guard_used,
                         size_t search_margin_work,
                         uint64_t acq_epoch,
                         uint64_t sched_gen,
                         uint64_t sched_idx,
                         const char* state_name) {
        const uint64_t win_id = next_window_id++;
        {
            ScopedLockWait lk(ledger_mutex_, ledger_wait_ns);
            ledger_.offered_windows++;
        }

        Sc16WindowSlot* slot = nullptr;
        int pool_idx = -1;
        std::chrono::steady_clock::time_point t_probe{};
        if (diag) {
            t_probe = std::chrono::steady_clock::now();
        }
        auto probe_pools = [&]() {
            for (int probe = 0; probe < n_workers; ++probe) {
                const int cand = static_cast<int>(
                    (win_id + static_cast<uint64_t>(probe)) %
                    static_cast<uint64_t>(n_workers));
                Sc16WindowSlot* candidate =
                    pools_[static_cast<size_t>(cand)]->acquire_filling_slot();
                if (candidate) {
                    slot = candidate;
                    pool_idx = cand;
                    break;
                }
            }
        };
        probe_pools();
        while (!slot && pacing_for_replay && !stop_requested_) {
            feedback_queue_.drain(fb_batch);
            ingest_fb_batch();
            std::this_thread::sleep_for(std::chrono::microseconds(200));
            probe_pools();
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
            drop_res.window_id = win_id;
            drop_res.state = WindowTerminalState::DroppedAdmission;
            drop_res.native_start_abs = win_start_abs;
            drop_res.native_sample_count = sample_count;
            drop_res.rx_push_time = std::chrono::steady_clock::now();
            drop_res.window_kind = kind;
            drop_res.acquisition_epoch = acq_epoch;
            drop_res.schedule_generation = sched_gen;
            drop_res.schedule_index = sched_idx;
            drop_res.schedule_state = state_name ? state_name : "fixed";
            commit_result(-1, drop_res);
            return;
        }

        {
            ScopedLockWait lk(ledger_mutex_, ledger_wait_ns);
            ledger_.admitted_windows++;
        }
        ++admitted_dynamic_slots;
        slot->window_id = win_id;
        slot->native_start_abs = win_start_abs;
        slot->native_count = sample_count;
        slot->native_rate = cfg_.native_rate;
        slot->work_rate = cfg_.work_rate;
        slot->push_time = std::chrono::steady_clock::now();
        slot->use_dynamic_slot = true;
        slot->window_kind = kind;
        slot->acquisition_epoch = acq_epoch;
        slot->schedule_generation = sched_gen;
        slot->schedule_index = sched_idx;
        slot->predicted_start_native_rel = pred_start_native_rel;
        slot->pre_guard_used = pre_guard_used;
        slot->search_margin_work = search_margin_work;
        slot->schedule_state_name = state_name ? state_name : "fixed";

        std::chrono::steady_clock::time_point t_copy{};
        if (diag) {
            t_copy = std::chrono::steady_clock::now();
        }
        std::memcpy(slot->sc16_samples.data(), src_samples,
                    sample_count * sizeof(std::complex<int16_t>));
        if (diag) {
            rx_acc_.slot_copy_ns.fetch_add(
                elapsed_ns(t_copy, std::chrono::steady_clock::now()),
                std::memory_order_relaxed);
            rx_acc_.slot_copy_bytes.fetch_add(
                static_cast<uint64_t>(sample_count) * sizeof(std::complex<int16_t>),
                std::memory_order_relaxed);
            t_copy = std::chrono::steady_clock::now();
        }

        pools_[static_cast<size_t>(pool_idx)]->publish_ready_slot(slot);
        if (diag) {
            rx_acc_.dispatch_ns.fetch_add(
                elapsed_ns(t_copy, std::chrono::steady_clock::now()),
                std::memory_order_relaxed);
            rx_acc_.windows_published.fetch_add(1, std::memory_order_relaxed);
        }
    };

    while (!stop_requested_) {
        if (cfg_.max_windows > 0 && next_window_id >= cfg_.max_windows) {
            break;
        }

        feedback_queue_.drain(fb_batch);
        ingest_fb_batch();

        if (ctrl.scheduled_path_active()) {
            AutoScheduleController::ScheduledSlotPlan plan;
            if (!ctrl.next_scheduled_slot(read_end_abs, plan)) {
                continue;
            }
            release_before(plan.win_start_abs);

            const int64_t eff_win_start = std::max(plan.win_start_abs, valid_start_abs);
            if (eff_win_start >= plan.win_end_abs ||
                plan.predicted_start_abs < eff_win_start) {
                ScopedLockWait lk(ledger_mutex_, ledger_wait_ns);
                ledger_.offered_windows++;
                ledger_.cancelled_windows++;
                continue;
            }
            const size_t copy_n = static_cast<size_t>(plan.win_end_abs - eff_win_start);
            if (copy_n > pool_slot_cap) {
                ScopedLockWait lk(ledger_mutex_, ledger_wait_ns);
                ledger_.offered_windows++;
                ledger_.cancelled_windows++;
                continue;
            }

            bool eof_hit = false;
            bool source_fatal = false;
            bool discontinuity_aborted = false;
            int zero_progress_reads = 0;

            while (read_end_abs < plan.win_end_abs && !stop_requested_) {
                release_before(eff_win_start);

                if (cfg_.max_samples > 0 && samples_read_total >= cfg_.max_samples) {
                    eof_hit = true;
                    break;
                }

                const size_t needed = static_cast<size_t>(plan.win_end_abs - read_end_abs);
                size_t to_read = std::max(read_slice, needed);
                size_t room = storage_cap - (valid_begin + valid_n);
                if (room < to_read && valid_begin > 0) {
                    const size_t moved_bytes =
                        valid_n * sizeof(std::complex<int16_t>);
                    std::chrono::steady_clock::time_point t_move{};
                    if (diag) {
                        t_move = std::chrono::steady_clock::now();
                    }
                    if (moved_bytes > 0) {
                        std::memmove(storage.data(), storage.data() + valid_begin,
                                     moved_bytes);
                    }
                    if (diag) {
                        rx_acc_.assemble_ns.fetch_add(
                            elapsed_ns(t_move, std::chrono::steady_clock::now()),
                            std::memory_order_relaxed);
                        rx_acc_.compact_move_bytes.fetch_add(
                            static_cast<uint64_t>(moved_bytes),
                            std::memory_order_relaxed);
                    }
                    valid_begin = 0;
                    room = storage_cap - valid_n;
                }
                if (to_read > room) {
                    to_read = room;
                }
                if (to_read == 0) {
                    set_source_error("RX auto-schedule storage produced a zero-length read request");
                    source_fatal = true;
                    break;
                }

                std::chrono::steady_clock::time_point t_read{};
                if (diag) {
                    t_read = std::chrono::steady_clock::now();
                }
                StreamReadMeta meta;
                const int64_t pre_read_abs = read_end_abs;
                const size_t n_read = source_->read_samples(
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
                samples_read_total += n_read;
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

                const bool disc = ctrl.handle_stream_meta(
                    pre_read_abs, n_read, meta.has_time_spec, meta.time_spec_s,
                    meta.is_gap, meta.gap_samples, read_end_abs);
                if (disc) {
                    valid_begin = 0;
                    valid_n = 0;
                    valid_start_abs = read_end_abs;
                    if (!ctrl.scheduled_path_active()) {
                        discontinuity_aborted = true;
                        break;
                    }
                    discontinuity_aborted = true;
                    break;
                }

                if (source_failed()) {
                    source_fatal = true;
                    break;
                }
                if (meta.is_eof ||
                    (cfg_.max_samples > 0 && samples_read_total >= cfg_.max_samples)) {
                    eof_hit = true;
                    break;
                }
                if (n_read == 0) {
                    ++zero_progress_reads;
                    if (zero_progress_reads >= kMaxZeroProgressReads) {
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

            if (discontinuity_aborted) {
                ScopedLockWait lk(ledger_mutex_, ledger_wait_ns);
                ledger_.offered_windows++;
                ledger_.cancelled_windows++;
                continue;
            }
            if (source_fatal || stop_requested_ ||
                (read_end_abs < plan.win_end_abs && eof_hit)) {
                ScopedLockWait lk(ledger_mutex_, ledger_wait_ns);
                ledger_.offered_windows++;
                ledger_.cancelled_windows++;
                break;
            }
            if (eff_win_start < valid_start_abs ||
                static_cast<size_t>(eff_win_start - valid_start_abs) + copy_n > valid_n) {
                ScopedLockWait lk(ledger_mutex_, ledger_wait_ns);
                ledger_.offered_windows++;
                ledger_.cancelled_windows++;
                break;
            }

            const size_t src_off =
                valid_begin + static_cast<size_t>(eff_win_start - valid_start_abs);
            const int64_t pred_rel = plan.predicted_start_abs - eff_win_start;
            emit_slot(eff_win_start,
                      storage.data() + src_off,
                      copy_n,
                      WindowKind::Scheduled,
                      pred_rel,
                      static_cast<size_t>(pred_rel),
                      plan.search_margin_work,
                      plan.acquisition_epoch,
                      plan.schedule_generation,
                      plan.schedule_index,
                      core::qm35_lock_state_name(ctrl.state()));
            if (pacing_for_replay && ctrl.state() != core::Qm35LockState::Locked) {
                if (feedback_queue_.wait_and_drain(
                        fb_batch, cfg_.auto_schedule.candidate_wait_timeout_ms)) {
                    ingest_fb_batch();
                }
            }
        } else {
            // Acquisition mode: feed live uninspected samples & poll verified candidates.
            const int64_t scan_cur = ctrl.scan_cursor_abs();
            if (read_end_abs > scan_cur && scan_cur >= valid_start_abs) {
                const size_t feed_off =
                    valid_begin + static_cast<size_t>(scan_cur - valid_start_abs);
                const size_t feed_n = static_cast<size_t>(read_end_abs - scan_cur);
                ctrl.feed_acquisition(storage.data() + feed_off, feed_n, scan_cur);
            }
            release_before(ctrl.scan_cursor_abs());

            AutoScheduleController::EmittedAcqSlot acq;
            if (ctrl.poll_verified_candidate(acq)) {
                if (cfg_.max_windows > 0 && next_window_id >= cfg_.max_windows) {
                    ctrl.release_held_regions();
                    break;
                }
                const size_t copy_n = std::min(acq.sample_count, pool_slot_cap);
                const int64_t pred_rel =
                    std::max<int64_t>(0, acq.predicted_start_abs - acq.win_start_abs);
                emit_slot(acq.win_start_abs,
                          acq.samples,
                          copy_n,
                          WindowKind::Acquisition,
                          pred_rel,
                          static_cast<size_t>(pred_rel),
                          cfg_.auto_schedule.acquisition_search_margin_work,
                          acq.acquisition_epoch,
                          acq.schedule_generation,
                          0,
                          core::qm35_lock_state_name(ctrl.state()));
                ctrl.release_held_regions();

                if (pacing_for_replay && ctrl.waiting_candidate_verify()) {
                    const auto cand_deadline =
                        std::chrono::steady_clock::now() +
                        std::chrono::milliseconds(
                            cfg_.auto_schedule.candidate_wait_timeout_ms);
                    while (!stop_requested_ && ctrl.waiting_candidate_verify()) {
                        const auto now = std::chrono::steady_clock::now();
                        if (now >= cand_deadline) {
                            break;
                        }
                        const auto rem_ms = static_cast<uint32_t>(
                            std::chrono::duration_cast<std::chrono::milliseconds>(
                                cand_deadline - now)
                                .count());
                        if (feedback_queue_.wait_and_drain(
                                fb_batch, std::max<uint32_t>(1, rem_ms))) {
                            ingest_fb_batch();
                        }
                    }
                }
                continue;
            }

            if (cfg_.max_samples > 0 && samples_read_total >= cfg_.max_samples) {
                ctrl.flush_acquisition();
                while (!stop_requested_ &&
                       (cfg_.max_windows == 0 || next_window_id < cfg_.max_windows) &&
                       ctrl.poll_verified_candidate(acq)) {
                    const size_t copy_n = std::min(acq.sample_count, pool_slot_cap);
                    const int64_t pred_rel =
                        std::max<int64_t>(0, acq.predicted_start_abs - acq.win_start_abs);
                    emit_slot(acq.win_start_abs,
                              acq.samples,
                              copy_n,
                              WindowKind::Acquisition,
                              pred_rel,
                              static_cast<size_t>(pred_rel),
                              cfg_.auto_schedule.acquisition_search_margin_work,
                              acq.acquisition_epoch,
                              acq.schedule_generation,
                              0,
                              core::qm35_lock_state_name(ctrl.state()));
                    ctrl.release_held_regions();
                }
                break;
            }

            size_t to_read = read_slice;
            size_t room = storage_cap - (valid_begin + valid_n);
            if (room < to_read && valid_begin > 0) {
                const size_t moved_bytes = valid_n * sizeof(std::complex<int16_t>);
                std::chrono::steady_clock::time_point t_move{};
                if (diag) {
                    t_move = std::chrono::steady_clock::now();
                }
                if (moved_bytes > 0) {
                    std::memmove(storage.data(), storage.data() + valid_begin,
                                 moved_bytes);
                }
                if (diag) {
                    rx_acc_.assemble_ns.fetch_add(
                        elapsed_ns(t_move, std::chrono::steady_clock::now()),
                        std::memory_order_relaxed);
                    rx_acc_.compact_move_bytes.fetch_add(
                        static_cast<uint64_t>(moved_bytes),
                        std::memory_order_relaxed);
                }
                valid_begin = 0;
                room = storage_cap - valid_n;
            }
            if (to_read > room) {
                to_read = room;
            }
            if (to_read == 0) {
                set_source_error("RX acquisition storage produced a zero-length read request");
                break;
            }

            std::chrono::steady_clock::time_point t_read{};
            if (diag) {
                t_read = std::chrono::steady_clock::now();
            }
            StreamReadMeta meta;
            const int64_t pre_read_abs = read_end_abs;
            const size_t n_read = source_->read_samples(
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
            samples_read_total += n_read;
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

            const bool disc = ctrl.handle_stream_meta(
                pre_read_abs, n_read, meta.has_time_spec, meta.time_spec_s,
                meta.is_gap, meta.gap_samples, read_end_abs);
            if (disc) {
                valid_begin = 0;
                valid_n = 0;
                valid_start_abs = read_end_abs;
            } else if (n_read > 0) {
                const int64_t cur = ctrl.scan_cursor_abs();
                if (read_end_abs > cur && cur >= valid_start_abs) {
                    const size_t feed_off =
                        valid_begin + static_cast<size_t>(cur - valid_start_abs);
                    const size_t feed_n = static_cast<size_t>(read_end_abs - cur);
                    ctrl.feed_acquisition(storage.data() + feed_off, feed_n, cur);
                }
                release_before(ctrl.scan_cursor_abs());
            }

            if (source_failed()) {
                break;
            }
            if (meta.is_eof ||
                (cfg_.max_samples > 0 && samples_read_total >= cfg_.max_samples)) {
                ctrl.flush_acquisition();
                while (!stop_requested_ &&
                       (cfg_.max_windows == 0 || next_window_id < cfg_.max_windows) &&
                       ctrl.poll_verified_candidate(acq)) {
                    const size_t copy_n = std::min(acq.sample_count, pool_slot_cap);
                    const int64_t pred_rel =
                        std::max<int64_t>(0, acq.predicted_start_abs - acq.win_start_abs);
                    emit_slot(acq.win_start_abs,
                              acq.samples,
                              copy_n,
                              WindowKind::Acquisition,
                              pred_rel,
                              static_cast<size_t>(pred_rel),
                              cfg_.auto_schedule.acquisition_search_margin_work,
                              acq.acquisition_epoch,
                              acq.schedule_generation,
                              0,
                              core::qm35_lock_state_name(ctrl.state()));
                    ctrl.release_held_regions();
                }
                break;
            }
            if (n_read == 0) {
                ++acq_zero_progress_reads;
                if (acq_zero_progress_reads >= kMaxZeroProgressReads) {
                    set_source_error(
                        "source returned no samples and no EOF for " +
                        std::to_string(acq_zero_progress_reads) + " consecutive reads");
                    break;
                }
                std::this_thread::yield();
            } else {
                acq_zero_progress_reads = 0;
            }
        }
    }

    ctrl.release_held_regions();

    if (pacing_for_replay) {
        const auto wait_deadline =
            std::chrono::steady_clock::now() + std::chrono::milliseconds(1000);
        while (!stop_requested_ &&
               ingested_feedback_slots < admitted_dynamic_slots &&
               std::chrono::steady_clock::now() < wait_deadline) {
            if (feedback_queue_.wait_and_drain(fb_batch, 50)) {
                ingest_fb_batch();
            } else {
                feedback_queue_.drain(fb_batch);
                ingest_fb_batch();
            }
        }
    }

    auto_sched_stats_.enabled = true;
    auto_sched_stats_.final_state = core::qm35_lock_state_name(ctrl.state());
    auto_sched_stats_.acq_windows = ctrl.acq_windows();
    auto_sched_stats_.scheduled_windows = ctrl.scheduled_windows();
    auto_sched_stats_.discontinuities = ctrl.discontinuities();
    auto_sched_stats_.stale_feedback = ctrl.stale_feedback();
    auto_sched_stats_.locked_at_window_id = ctrl.locked_at_window_id();
    auto_sched_stats_.t0_exact = ctrl.t0_exact();
    auto_sched_stats_.period_samples = ctrl.period_samples();
    auto_sched_stats_.period_s = ctrl.period_s();
    auto_sched_stats_.effective_pre_guard = effective_pre_guard_;
    auto_sched_stats_.effective_capture = effective_capture_;
    auto_sched_stats_.effective_post_guard = effective_post_guard_;

    rx_finished_.store(true, std::memory_order_release);
}

void UhdSc16Pipeline::rx_producer_loop()
{
    if (cfg_.schedule_mode == ScheduleMode::AutoAcquire) {
        rx_auto_schedule_loop();
        return;
    }

    // A2 ready barrier: do not touch the source -- and in particular do not let
    // a UHD device start streaming -- until every DSP worker published Ready.
    // A failed worker (or a stop() while waiting) aborts before any read, so
    // nothing was admitted and the ledger stays trivially conserved.
    if (!wait_for_workers_ready()) {
        rx_finished_.store(true, std::memory_order_release);
        return;
    }
    if (!source_->start_stream()) {
        std::cerr << "UhdSc16Pipeline: source start_stream() failed; no windows "
                     "will be produced\n";
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
                std::cerr << "UhdSc16Pipeline: RX storage cannot fit a read for "
                             "window "
                          << k << " (storage_cap=" << storage_cap
                          << " valid_begin=" << valid_begin
                          << " valid_n=" << valid_n << "); aborting the run\n";
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
            if (meta.is_eof) {
                eof_hit = true;
                break;
            }
            if (n_read == 0) {
                // R1: bounded tolerance for non-EOF zero-progress reads.
                ++zero_progress_reads;
                if (zero_progress_reads >= kMaxZeroProgressReads) {
                    std::cerr << "UhdSc16Pipeline: source made no progress after "
                              << zero_progress_reads
                              << " consecutive reads for window " << k
                              << "; aborting the run\n";
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
            std::cerr << "UhdSc16Pipeline: internal RX buffer invariant violated for "
                         "window "
                      << k << " (win_start=" << win_start
                      << " valid_start_abs=" << valid_start_abs
                      << " valid_n=" << valid_n << "); window cancelled\n";
            ScopedLockWait lk(ledger_mutex_, ledger_wait_ns);
            ledger_.offered_windows++;
            ledger_.cancelled_windows++;
            break;
        }

        {
            ScopedLockWait lk(ledger_mutex_, ledger_wait_ns);
            ledger_.offered_windows++;
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
                std::cerr << "UhdSc16Pipeline: dsp worker " << worker_id
                          << " affinity to core " << affinity_core
                          << " failed: " << std::strerror(rc) << std::endl;
            }
        } else {
            std::cerr << "UhdSc16Pipeline: dsp worker " << worker_id
                      << " affinity core " << affinity_core
                      << " is out of range (CPU_SETSIZE=" << CPU_SETSIZE << ")"
                      << std::endl;
        }
#else
        std::cerr << "UhdSc16Pipeline: dsp worker " << worker_id
                  << " affinity to core " << affinity_core
                  << " requested but thread affinity is not supported on this "
                     "platform"
                  << std::endl;
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

    const size_t native_capacity = effective_slot_capacity();
    const bool need_resample = (std::abs(cfg_.native_rate - cfg_.work_rate) > 1e3);

    // Phase-C experiment selector: initialize() already validated and
    // normalised cfg_.resampler_experiment.  Resolved once per worker; with the
    // default "none" no setter is called below, so the default path stays
    // instruction-for-instruction identical.
    const ResamplerExperiment resampler_experiment =
        parse_resampler_experiment(cfg_.resampler_experiment);
    const bool exp_fixed_phase = (resampler_experiment == ResamplerExperiment::FixedPhase ||
                                  resampler_experiment == ResamplerExperiment::Both);
    const bool exp_convert_cache =
        (resampler_experiment == ResamplerExperiment::ConvertCache ||
         resampler_experiment == ResamplerExperiment::Both);
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

    sic::SicUnifiedScratch sic_scratch;
    sic::SicUnifiedTemplates sic_templates;
    sic::SicUnifiedConfig effective_sic_cfg = build_effective_sic_cfg(cfg_);
    // This worker owns `sic_out` and reads it only when SIC reports committed.
    // Skipping the per-window O(n) y -> out copy on every bypass exit removes
    // ~122 us of pure waste per clean window (plus the i_hat zero-fill, which
    // is gone entirely because this worker never publishes i_hat).
    effective_sic_cfg.copy_input_on_bypass = false;
    std::vector<std::complex<float>> sic_sync_tmpl;
    std::vector<std::complex<float>> sic_preamble_tmpl;

    radar::RadarCirCoreScratch radar_scratch;
    radar::RadarCirConfig effective_radar_cfg = build_effective_radar_cfg(cfg_);
    std::vector<std::complex<float>> radar_sync_tmpl;

    demod::core::DemodScratch comm_scratch;
    std::vector<std::complex<float>> comm_sync_tmpl;

    try {
        // QA-only failure injection point (default empty => no-op, no lock).
        if (cfg_.test_worker_init_hook) {
            cfg_.test_worker_init_hook(worker_id);
        }

        if (need_resample) {
            const std::string profile =
                cfg_.resampler_profile.empty() ? "realtime_eq16" : cfg_.resampler_profile;
            if (std::abs(cfg_.native_rate - 737.28e6) < 1e5) {
                auto taps = load_resampler_taps("resampler_65_48", profile);
                if (taps.empty()) {
                    // Historical fallback.  It is a flat boxcar, NOT the designed
                    // 65/48 filter: it doubles the resample cost and destroys the
                    // passband, so every downstream detector fails.  Make the
                    // silent degradation at least visible.
                    std::cerr << "UhdSc16Pipeline: warning: resampler_65_48 profile '"
                              << profile << "' taps not found (cwd-dependent); falling back to a "
                              << "flat " << (65 * 64) << "-tap placeholder filter, DSP output "
                              << "will be wrong\n";
                    taps.resize(65 * 64, 1.0f / 48.0f);
                }
                resamp_65_48 = std::make_unique<core::RationalResampler65_48Core>(taps);
                // Contract C3: a burst worker must never nest the resampler
                // thread pool.  num_workers == 1 forces the flat
                // run_macroblock_sc16 + fir_avx2_sc16_4x path on this core.
                resamp_65_48->set_num_workers(1);
                // Phase-C experiment switch (default "none" => no call).
                apply_resampler_experiment(
                    *resamp_65_48, exp_fixed_phase, exp_convert_cache);
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
                apply_resampler_experiment(
                    *resamp_65_32, exp_fixed_phase, exp_convert_cache);
                experiment_applied_to_core = true;
                work_capacity = core::RationalResampler65_32Core::expected_output_length(
                    native_capacity, taps.size()) + 256;
            } else {
                work_capacity = static_cast<size_t>(std::ceil(
                    static_cast<double>(native_capacity) * (cfg_.work_rate / cfg_.native_rate))) + 512;
                // No dedicated kernel: this branch converts through cf32_native,
                // so size it here to keep the per-window path allocation-free.
                cf32_native.assign(native_capacity, std::complex<float>(0.0f, 0.0f));
            }
        }

        // cf32_native is only consumed by the no-resampler branch (and by the
        // generic rate fallback above, which sizes it itself).  When a dedicated
        // 65/48 or 65/32 kernel runs, nothing is ever converted into it, so
        // pre-zeroing 8 * native_capacity bytes at startup is dead work.
        if (!need_resample) {
            cf32_native.assign(native_capacity, std::complex<float>(0.0f, 0.0f));
        }
        cf32_work.assign(work_capacity, std::complex<float>(0.0f, 0.0f));
        // sic_out is the committed-output buffer.  It is sized once here; the
        // per-window guard below only grows it if a window ever exceeds
        // work_capacity, so the steady state has no zero-fill at all.
        sic_out.assign(work_capacity, std::complex<float>(0.0f, 0.0f));

        if (mode_runs_sic(cfg_.consumer_mode)) {
            if (cfg_.schedule_mode == ScheduleMode::AutoAcquire) {
                effective_sic_cfg.max_window_samples =
                    std::max(effective_sic_cfg.max_window_samples, work_capacity);
                effective_sic_cfg.max_packet_samples =
                    std::max(effective_sic_cfg.max_packet_samples, work_capacity);
            }
            load_or_synthesize_sync_template(sic_sync_tmpl,
                                             effective_sic_cfg.comm_code,
                                             effective_sic_cfg.pulse,
                                             effective_sic_cfg.sfd_mode,
                                             cfg_.sic_sync_template_file);

            sic_preamble_tmpl.reserve(effective_sic_cfg.sync_reps * 1016);
            for (size_t rep = 0; rep < effective_sic_cfg.sync_reps; ++rep) {
                sic_preamble_tmpl.insert(sic_preamble_tmpl.end(),
                                         sic_sync_tmpl.begin(), sic_sync_tmpl.end());
            }

            sic_templates.sync = sic_sync_tmpl.data();
            sic_templates.sync_len = sic_sync_tmpl.size();
            sic_templates.preamble = sic_preamble_tmpl.data();
            sic_templates.preamble_len = sic_preamble_tmpl.size();

            prepare_sic_unified(effective_sic_cfg, sic_templates, sic_scratch);
        }

        if (mode_runs_sensing(cfg_.consumer_mode)) {
            if (cfg_.schedule_mode == ScheduleMode::AutoAcquire) {
                effective_radar_cfg.full_window_search = false;
                effective_radar_cfg.use_predicted_timing = false;
                effective_radar_cfg.timing_coarse_stride = 1;
            }
            load_or_synthesize_sync_template(radar_sync_tmpl,
                                             effective_radar_cfg.code_index,
                                             cfg_.radar_pulse,
                                             effective_radar_cfg.sfd_mode,
                                             cfg_.radar_sync_template_file);

            prepare_radar_cir_core(effective_radar_cfg, radar_sync_tmpl.data(),
                                   effective_radar_cfg.samples_per_symbol,
                                   effective_radar_cfg.cir_pre,
                                   effective_radar_cfg.cir_post,
                                   radar_scratch);
            // Reserve the shared preamble front-end scratch for the largest
            // window this worker will see (no allocation in the hot path).
            radar::reserve_radar_cir_core(work_capacity, radar_scratch);
        }

        if (mode_runs_comm(cfg_.consumer_mode)) {
            load_or_synthesize_sync_template(comm_sync_tmpl,
                                             cfg_.comm_cfg.code_index,
                                             cfg_.comm_pulse,
                                             cfg_.comm_cfg.sfd_mode,
                                             cfg_.comm_sync_template_file);
            comm_scratch.reserve(work_capacity);
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

    // Output persistence (default off): same idea.  output_writer_ is non-null
    // only when initialize() created AND successfully started it, so a failed
    // start costs this one test and nothing else.
    const bool output_enabled = (output_writer_ != nullptr);
    // Per-worker staging for the decoded PSDU.  DemodResult is scoped to the
    // comm block, but the output slot is filled at the very end of the window,
    // so the bytes are copied here (only while output persistence is enabled).
    // Declared outside the loop so the buffer keeps its capacity.
    std::vector<uint8_t> out_psdu_buf;

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
        res.window_kind = slot->window_kind;
        res.acquisition_epoch = slot->acquisition_epoch;
        res.schedule_generation = slot->schedule_generation;
        res.schedule_index = slot->schedule_index;
        res.schedule_state =
            slot->schedule_state_name ? slot->schedule_state_name : "fixed";
        // Same clock reading that commit_result()/queue_us use: the RX thread
        // stored it in the slot right before publishing.
        res.rx_push_time = slot->push_time;
        res.worker_start_time = t_start;
        res.timing.queue_us = std::chrono::duration<double, std::micro>(t_start - slot->push_time).count();

        double filter_delay = 0.0;
        int64_t target_seed_work = -1;
        if (slot->use_dynamic_slot) {
            if (need_resample && resamp_65_48) {
                target_seed_work = resamp_65_48->map_input_offset_to_output(
                    slot->predicted_start_native_rel);
                filter_delay = 0.5 * static_cast<double>(
                    resamp_65_48->tap_count() > 0 ? resamp_65_48->tap_count() - 1 : 0);
            } else if (need_resample && resamp_65_32) {
                target_seed_work = resamp_65_32->map_input_offset_to_output(
                    slot->predicted_start_native_rel);
                filter_delay = 0.5 * static_cast<double>(
                    resamp_65_32->tap_count() > 0 ? resamp_65_32->tap_count() - 1 : 0);
            } else if (need_resample) {
                const double native_r =
                    (slot->native_rate > 0.0) ? slot->native_rate : cfg_.native_rate;
                const double work_r =
                    (slot->work_rate > 0.0) ? slot->work_rate : cfg_.work_rate;
                target_seed_work = static_cast<int64_t>(std::llround(
                    static_cast<double>(slot->predicted_start_native_rel) *
                    (work_r / native_r)));
                filter_delay = 0.0;
            } else {
                target_seed_work = slot->predicted_start_native_rel;
                filter_delay = 0.0;
            }
        }

        LockFeedback fb;
        if (slot->use_dynamic_slot) {
            fb.window_id = slot->window_id;
            fb.acquisition_epoch = slot->acquisition_epoch;
            fb.schedule_generation = slot->schedule_generation;
            fb.schedule_index = slot->schedule_index;
            fb.window_kind = slot->window_kind;
            fb.slot_native_start_abs = slot->native_start_abs;
            fb.work_rate = (slot->work_rate > 0.0) ? slot->work_rate : cfg_.work_rate;
            fb.native_rate =
                (slot->native_rate > 0.0) ? slot->native_rate : cfg_.native_rate;
            fb.resample_filter_delay = filter_delay;
        }

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

        // ---- output-persistence mirrors -------------------------------------
        // Everything the writer needs that WindowResult does not publish.
        // RadarCirResult and DemodResult are scoped to their own consumer
        // blocks, while the slot is filled after every consumer has run, so the
        // needed scalars are hoisted here.  They cost nothing but a few
        // registers when output persistence is disabled.
        float out_sfd_metric = 0.0f;
        float out_raw_l2_norm = 0.0f;
        size_t out_tap_count = 0;
        // The kernel's authoritative peak tap (argmax of |raw|, L2) and the
        // RX-detected SFD start.  Both are published verbatim: recomputing
        // peak_tap with a different norm in the writer would pair a tap with a
        // metric measured by another norm.
        size_t out_peak_tap = 0;
        int64_t out_sfd_start = -1;
        bool out_sensing_ok = false;
        int64_t out_pred_sfd = -1;
        bool out_comm_ran = false;
        uint16_t out_fcs_received = 0;
        uint16_t out_fcs_calculated = 0;
        float out_phr_rate_mbps = 0.0f;
        bool out_phr_corrected = false;
        bool out_phr_uncorrectable = false;

        // 3. SIC processing
        if (mode_runs_sic(cfg_.consumer_mode)) {
            auto t_sic_start = std::chrono::steady_clock::now();
            res.sic_executed = true;

            if (sic_out.size() < dsp_n) {
                // Growth-only fallback; the startup assign() already sized this
                // to work_capacity, so the steady state never zero-fills here.
                sic_out.resize(dsp_n);
            }

            sic::SicUnifiedResult sic_res;
            sic_res.out = sic_out.data();
            // No i_hat consumer in this pipeline: leaving it null skips both the
            // reconstruction copy and the outside-support zero fill.
            sic_res.i_hat = nullptr;

            sic::SicUnifiedConfig slot_sic_cfg = effective_sic_cfg;
            if (slot->use_dynamic_slot && target_seed_work >= 0 &&
                (mode_runs_sensing(cfg_.consumer_mode) ||
                 mode_runs_comm(cfg_.consumer_mode))) {
                const uint32_t target_sync_reps =
                    mode_runs_sensing(cfg_.consumer_mode)
                        ? effective_radar_cfg.sync_repetitions
                        : cfg_.comm_cfg.preamble_repetitions;
                const char* target_sfd_str =
                    mode_runs_sensing(cfg_.consumer_mode)
                        ? (effective_radar_cfg.sfd_mode ? effective_radar_cfg.sfd_mode : "4z2")
                        : (cfg_.comm_cfg.sfd_mode ? cfg_.comm_cfg.sfd_mode : "4z2");
                const size_t target_sfd_syms = sfd_symbol_count_for_mode(target_sfd_str);
                const int64_t lo_margin =
                    static_cast<int64_t>(slot->search_margin_work) + 1024;
                const int64_t hi_margin =
                    static_cast<int64_t>(slot->search_margin_work) +
                    3 * static_cast<int64_t>(demod::kQm35SamplesPerSymbol);
                slot_sic_cfg.target_preamble_lo =
                    std::max<int64_t>(0, target_seed_work - lo_margin);
                slot_sic_cfg.target_preamble_hi =
                    target_seed_work +
                    static_cast<int64_t>(target_sync_reps + target_sfd_syms) *
                        demod::kQm35SamplesPerSymbol +
                    hi_margin;
            }

            bool sic_ok = sic::sic_process_unified(
                dsp_in, dsp_n, sic_templates, slot_sic_cfg, sic_scratch, sic_res);

            auto t_sic_end = std::chrono::steady_clock::now();
            res.timing.sic_us = std::chrono::duration<double, std::micro>(t_sic_end - t_sic_start).count();

            if (sic_ok) {
                res.sic_committed = sic_res.committed;
                res.actual_suppression_db = static_cast<float>(sic_res.suppression_db);
                res.sic_status = sic_res.reason ? sic_res.reason : "";
                // copy_input_on_bypass == false: a non-committed result means
                // "read the original input", i.e. keep dsp_in/dsp_n untouched.
                // Only a committed transaction may redirect to sic_out, and that
                // path writes the whole window (ROI plus byte-identical y).
                if (sic_res.committed) {
                    dsp_in = sic_out.data();
                }
            } else {
                res.sic_status = "sic_process_failed";
            }

            if (slot->use_dynamic_slot) {
                fb.timing_ok = sic_ok && sic_res.channel.valid &&
                               (sic_res.channel.q >= 0);
                fb.detected_start_work_rel = fb.timing_ok ? sic_res.channel.q : -1;
                fb.identity_ok = fb.timing_ok && sic_res.committed;
                fb.fcs_pass = sic_res.comm_fcs_pass;
                fb.code_index = static_cast<int>(effective_sic_cfg.comm_code);
                fb.preamble_repetitions = static_cast<int>(effective_sic_cfg.sync_reps);
                fb.sfd_mode = effective_sic_cfg.sfd_mode ? effective_sic_cfg.sfd_mode : "4z2";
            }
        }

        // 4. Sensing CIR processing (Mode A)
        if (mode_runs_sensing(cfg_.consumer_mode)) {
            auto t_sens_start = std::chrono::steady_clock::now();
            res.radar_executed = true;

            radar::RadarCirConfig dyn_radar_cfg = effective_radar_cfg;
            int64_t pred_sfd = -1;
            if (slot->use_dynamic_slot) {
                dyn_radar_cfg.full_window_search = false;
                dyn_radar_cfg.use_predicted_timing = false;
                dyn_radar_cfg.timing_coarse_stride = 1;
                dyn_radar_cfg.sfd_search_margin =
                    static_cast<int64_t>(slot->search_margin_work);
                radar_scratch.demod_profile.timing_search_margin =
                    slot->search_margin_work;
                pred_sfd = target_seed_work +
                           static_cast<int64_t>(dyn_radar_cfg.sync_repetitions *
                                                dyn_radar_cfg.samples_per_symbol);
            } else {
                double rate_ratio = cfg_.work_rate / cfg_.native_rate;
                pred_sfd = static_cast<int64_t>(std::llround(static_cast<double>(cfg_.pre_guard_samples) * rate_ratio)) +
                           static_cast<int64_t>(effective_radar_cfg.sync_repetitions * effective_radar_cfg.samples_per_symbol);
            }
            out_pred_sfd = pred_sfd;

            radar::RadarCirResult cir_res;
            bool cir_ok = radar::radar_cir_one(
                dsp_in, dsp_n, pred_sfd, dyn_radar_cfg, radar_scratch, cir_res);

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
            // Provenance is published for both outcomes: a failed window keeps
            // cfo_epoch == 0 / origins == -1, which is itself the diagnostic
            // (no stale value from the previous window is ever carried over).
            res.radar_cfo_hz = cir_res.final_cfo_hz;
            res.radar_cir_origin = cir_res.cir_axis_origin;
            res.radar_detected_origin = cir_res.detected_sync_origin;
            res.radar_valid_reps = cir_res.valid_repetitions;
            // Output-persistence mirrors: only a successful CIR publishes taps,
            // so a failed window leaves the slot's tap vector empty instead of
            // carrying the previous window's taps.
            out_sensing_ok = (cir_ok && cir_res.status == radar::RadarCirStatus::Ok);
            out_sfd_metric = cir_res.sfd_metric;
            out_raw_l2_norm = cir_res.raw_l2_norm;
            out_tap_count = out_sensing_ok ? cir_res.tap_count : 0;
            out_peak_tap = out_sensing_ok ? cir_res.peak_tap : 0;
            out_sfd_start = cir_res.sfd_start_sample;
            // Front-end total = timing + cfo + sfd.  `coverage_us` is NOT
            // added: the coverage PLANNING pass runs inside the SFD parent
            // (uwb_demod_core.h laps `sfd_us` after the coverage block), so
            // adding it here would count it twice.  The independent final-CFO
            // derotation of the coverage interval is also inside `sfd_us`
            // (accumulated into `sfd_cfo_refit_us`); it is reported by name in
            // benchmark_shared_cir.cc, not folded into this total.
            res.radar_frontend_us = static_cast<double>(
                cir_res.frontend_timing_us + cir_res.frontend_cfo_us +
                cir_res.frontend_sfd_us);
            res.radar_cir_us = static_cast<double>(cir_res.cir_us);
            // Signed remainder of the outer sensing wall clock.  The integer
            // sub-timers are truncated to 1 us each, so a small positive value
            // is expected; it is never clamped, because a negative remainder is
            // the signal that a child was double counted.
            res.radar_other_us = res.timing.sensing_us - res.radar_frontend_us -
                                 res.radar_cir_us;

            if (slot->use_dynamic_slot) {
                fb.timing_ok = res.radar_sfd_found && (res.radar_detected_origin >= 0);
                fb.detected_start_work_rel = res.radar_detected_origin;
                fb.identity_ok = fb.timing_ok;
                fb.fcs_pass = false;
                fb.code_index = static_cast<int>(dyn_radar_cfg.code_index);
                fb.preamble_repetitions =
                    static_cast<int>(dyn_radar_cfg.sync_repetitions);
                fb.sfd_mode = dyn_radar_cfg.sfd_mode ? dyn_radar_cfg.sfd_mode : "4z2";
            }
        }

        // 5. Pure C++ Comm RX processing (Mode B)
        if (mode_runs_comm(cfg_.consumer_mode)) {
            auto t_comm_start = std::chrono::steady_clock::now();
            res.comm_attempted = true;

            int64_t comm_seed = -1;
            demod::Qm35825Profile effective_comm_cfg =
                build_effective_comm_cfg(cfg_, dsp_n, comm_seed);
            if (slot->use_dynamic_slot) {
                effective_comm_cfg.timing_search_margin = slot->search_margin_work;
                effective_comm_cfg.timing_coarse_stride = 1;
                comm_seed = target_seed_work;
            }

            const demod::DemodResult demod_res = demod::core::demodulate_one(
                dsp_in, dsp_n, effective_comm_cfg, res.window_id,
                comm_seed, /*window_start=*/0, comm_sync_tmpl, comm_scratch);

            auto t_comm_end = std::chrono::steady_clock::now();
            res.timing.comm_us =
                std::chrono::duration<double, std::micro>(t_comm_end - t_comm_start).count();
            res.comm_us = res.timing.comm_us;

            res.comm_ok = (demod_res.status == demod::DemodStatus::Success);
            res.comm_status_reason = demod_status_reason(demod_res.status);
            res.comm_sync_origin = demod_res.timing.ok ? demod_res.timing.preamble_start_sample : -1;
            res.comm_sfd_start = demod_res.sfd.ok ? demod_res.sfd.sfd_start_sample : -1;
            res.comm_cfo_hz = demod_res.cfo.ok ? demod_res.cfo.cfo_hz : 0.0;
            res.comm_phr_ok = demod_res.phr.ok;
            res.comm_phr_length = demod_res.phr.ok
                ? static_cast<uint16_t>(std::min<uint32_t>(demod_res.phr.psdu_length, 65535U))
                : 0;
            res.comm_fcs_ok = demod_res.payload.fcs_pass;
            res.comm_payload_bytes = demod_res.payload.bytes.size();
            res.comm_psdu_fnv1a = demod_res.payload.bytes.empty()
                ? 0ULL
                : fnv1a_64(demod_res.payload.bytes.data(),
                           demod_res.payload.bytes.size());

            // Output-persistence mirrors: the payload bytes and the FCS/PHR
            // diagnostics live in the block-scoped DemodResult, so the PSDU is
            // staged here (only while output persistence is enabled) and the
            // scalars are copied out.
            out_comm_ran = true;
            out_fcs_received = demod_res.payload.received_fcs;
            out_fcs_calculated = demod_res.payload.calculated_fcs;
            out_phr_rate_mbps = demod_res.phr.data_rate_mbps;
            out_phr_corrected = demod_res.phr.secded_corrected;
            out_phr_uncorrectable = demod_res.phr.secded_uncorrectable;
            if (output_enabled) {
                out_psdu_buf.assign(demod_res.payload.bytes.begin(),
                                    demod_res.payload.bytes.end());
            }

            if (slot->use_dynamic_slot) {
                fb.timing_ok =
                    (res.comm_sync_origin >= 0) && (res.comm_sfd_start >= 0);
                fb.detected_start_work_rel = res.comm_sync_origin;
                fb.identity_ok =
                    res.comm_ok ||
                    (!cfg_.auto_schedule.require_fcs_pass && fb.timing_ok);
                fb.fcs_pass = res.comm_fcs_ok;
                fb.code_index = static_cast<int>(effective_comm_cfg.code_index);
                fb.preamble_repetitions =
                    static_cast<int>(effective_comm_cfg.preamble_repetitions);
                fb.sfd_mode =
                    effective_comm_cfg.sfd_mode ? effective_comm_cfg.sfd_mode : "4z2";
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

        if (slot->use_dynamic_slot) {
            feedback_queue_.push(fb);
        }

        // ------------------------------------------------------------------
        // Output persistence: the window is complete only now, so this is the
        // first point where every field the writer consumes is final.  One
        // non-blocking slot acquisition per window; on pool exhaustion the
        // writer's ledger has already counted the drop and this window simply
        // is not persisted (never a correctness issue, never a stall).
        // ------------------------------------------------------------------
        OutputPayloadSlot* out_slot = nullptr;
        if (output_enabled) {
            out_slot = output_writer_->try_acquire_slot();
            if (out_slot) {
                OutputWindowRecord rec;
                rec.window_id = res.window_id;
                rec.schedule_index = res.schedule_index;
                rec.worker_id = res.worker_id;
                rec.native_start_abs = res.native_start_abs;
                rec.native_sample_count = res.native_sample_count;
                // Native-grid origin of the buffer this window was computed
                // from.  The comm path is handed the same buffer with
                // `window_start = 0` (demodulate_one), i.e. every sample
                // position it reports is relative to this origin.
                rec.native_window_start = res.native_start_abs;
                rec.predicted_start_sample = out_pred_sfd;

                rec.sic_executed = res.sic_executed;
                rec.sic_committed = res.sic_committed;
                rec.actual_suppression_db = res.actual_suppression_db;
                rec.sic_status = res.sic_status;

                rec.radar_executed = res.radar_executed;
                rec.radar_sfd_found = res.radar_sfd_found;
                rec.radar_peak_metric = res.radar_peak_metric;
                rec.radar_detected_start = res.radar_detected_start;
                rec.radar_cfo_hz = res.radar_cfo_hz;
                rec.radar_cir_origin = res.radar_cir_origin;
                rec.radar_detected_origin = res.radar_detected_origin;
                rec.radar_valid_reps = res.radar_valid_reps;
                rec.sfd_metric = out_sfd_metric;
                rec.raw_l2_norm = out_raw_l2_norm;
                rec.peak_tap = out_peak_tap;
                rec.sfd_start_sample = out_sfd_start;
                rec.cir_pre = effective_radar_cfg.cir_pre;
                rec.cir_post = effective_radar_cfg.cir_post;

                rec.comm_attempted = res.comm_attempted;
                rec.comm_ok = res.comm_ok;
                rec.comm_status_reason = res.comm_status_reason;
                rec.comm_sync_origin = res.comm_sync_origin;
                rec.comm_sfd_start = res.comm_sfd_start;
                rec.comm_cfo_hz = res.comm_cfo_hz;
                rec.comm_phr_ok = res.comm_phr_ok;
                rec.comm_phr_length = res.comm_phr_length;
                rec.comm_fcs_ok = res.comm_fcs_ok;
                rec.comm_payload_bytes = res.comm_payload_bytes;
                rec.comm_psdu_fnv1a = res.comm_psdu_fnv1a;
                rec.fcs_received = out_fcs_received;
                rec.fcs_calculated = out_fcs_calculated;
                rec.phr_rate_mbps = out_phr_rate_mbps;
                rec.phr_corrected = out_phr_corrected;
                rec.phr_uncorrectable = out_phr_uncorrectable;

                rec.resample_us = res.timing.resample_us;
                rec.t_total_us = res.timing.comm_us;
                rec.sensing_us = res.timing.sensing_us;

                out_slot->record = rec;
                // Taps and PSDU are rewritten from scratch every window, so a
                // failed sensing / comm stage leaves them empty rather than
                // carrying the previous window's payload.
                out_slot->cir_taps.clear();
                out_slot->psdu_bytes.clear();
                if (out_sensing_ok && out_tap_count > 0 &&
                    radar_scratch.cir.raw_taps.size() >= out_tap_count) {
                    // True-amplitude taps, exactly the first tap_count entries
                    // of the per-worker scratch the CIR kernel just filled.
                    out_slot->cir_taps.assign(
                        radar_scratch.cir.raw_taps.begin(),
                        radar_scratch.cir.raw_taps.begin() +
                            static_cast<std::ptrdiff_t>(out_tap_count));
                }
                if (out_comm_ran) {
                    out_slot->psdu_bytes.assign(out_psdu_buf.begin(),
                                                out_psdu_buf.end());
                }
            }
        }

        // Release the slot before committing: the RX producer may refill this
        // ring immediately while this worker is busy ordering the result.
        my_pool->release_reading_slot(slot);

        {
            std::lock_guard<std::mutex> lk(ledger_mutex_);
            ledger_.completed_windows++;
        }

        // Contract §1.3: never hold ledger_mutex_ while committing (no lock
        // nesting); commit_result owns commit_mutex_ and the reorder buffer.
        commit_result(worker_id, res, out_slot);
    }
}

// ---------------------------------------------------------------------------
// In-order result commit (reorder buffer)
// ---------------------------------------------------------------------------
void UhdSc16Pipeline::commit_result(int worker_id,
                                    const WindowResult& res,
                                    OutputPayloadSlot* slot)
{
    const bool diag = cfg_.enable_commit_diagnostics;

    std::chrono::steady_clock::time_point t_lock{};
    if (diag) {
        t_lock = std::chrono::steady_clock::now();
        commit_acc_.commit_calls.fetch_add(1, std::memory_order_relaxed);
    }
    std::lock_guard<std::mutex> lk(commit_mutex_);
    std::chrono::steady_clock::time_point t_locked{};
    if (diag) {
        t_locked = std::chrono::steady_clock::now();
        commit_acc_.lock_wait_ns.fetch_add(elapsed_ns(t_lock, t_locked),
                                           std::memory_order_relaxed);
    }

    // pending_results_ is indexed relative to next_commit_id_, so index 0 is
    // always the next window_id to emit and the buffer base never needs to be
    // stored separately.  That keeps the ordering cursor and the buffer base
    // identical, bounds memory to the worker skew, and preserves the
    // invariant "next_commit_id_ == first uncommitted window_id".
    if (res.window_id < next_commit_id_) {
        // Already emitted (cannot happen: each window commits exactly once).
        // The slot (if any) was acquired but will never be persisted, so give it
        // back to the writer's pool -- release_slot() converts the matching
        // `offered` into a `dropped` and never blocks.
        if (slot && output_writer_) {
            output_writer_->release_slot(slot);
        }
        return;
    }
    const size_t rel = static_cast<size_t>(res.window_id - next_commit_id_);
    if (diag) {
        // How far ahead of the ordering cursor this completion landed: the
        // live out-of-order span the reorder buffer has to hold.
        note_peak(commit_acc_.max_out_of_order_span, static_cast<uint64_t>(rel));
    }
    if (pending_results_.size() <= rel) {
        pending_results_.resize(rel + 1);
    }
    auto stored = std::make_unique<WindowResult>(res);
    stored->worker_id = worker_id;
    pending_results_[rel] = std::move(stored);

    // Parallel reorder buffer for the output payload.  It uses the same `rel`
    // index space as pending_results_, so a slot is only handed to the writer
    // when its window is actually emitted -- that is what makes every file and
    // log line strictly ascending by window_id.  Always empty when output
    // persistence is disabled, so the disabled path stays free.
    if (output_writer_) {
        if (pending_slots_.size() <= rel) {
            pending_slots_.resize(rel + 1, nullptr);
        }
        pending_slots_[rel] = slot;
    }

    // Flush every contiguous entry, strictly by ascending window_id.
    size_t flushed = 0;
    uint64_t callback_ns = 0;
    while (flushed < pending_results_.size() && pending_results_[flushed]) {
        WindowResult copy = *pending_results_[flushed];
        pending_results_[flushed].reset();
        results_.push_back(std::move(copy));
        if (diag) {
            commit_acc_.results_emitted.fetch_add(1, std::memory_order_relaxed);
        }
        // Hand the payload over while still inside the in-order emit section.
        // try_submit() takes ownership on success and returns the slot to the
        // pool on failure, so the pointer is never touched again either way.
        if (flushed < pending_slots_.size()) {
            OutputPayloadSlot* emit_slot = pending_slots_[flushed];
            pending_slots_[flushed] = nullptr;
            if (emit_slot) {
                output_writer_->try_submit(emit_slot);
            }
        }
        ++flushed;
        ++next_commit_id_;
        if (result_cb_) {
            // Invoked while commit_mutex_ is held so observers always see
            // strict window_id order.  The callback must not re-enter the
            // pipeline (set_result_callback / stop / wait / results / ledger).
            const auto t_cb = diag ? std::chrono::steady_clock::now()
                                   : std::chrono::steady_clock::time_point{};
            result_cb_(results_.back());
            if (diag) {
                callback_ns += elapsed_ns(t_cb, std::chrono::steady_clock::now());
            }
        }
        // Stamped after the (optional) callback returned, still inside
        // commit_result, so it also covers callback cost for this window.
        results_.back().commit_time = std::chrono::steady_clock::now();
    }

    if (diag) {
        const uint64_t total_ns = elapsed_ns(t_locked, std::chrono::steady_clock::now());
        // Parent/child split: the locked section minus the callback time is
        // the real insertion + in-order flush cost (never double counted).
        commit_acc_.callback_ns.fetch_add(callback_ns, std::memory_order_relaxed);
        commit_acc_.reorder_ns.fetch_add(total_ns > callback_ns ? total_ns - callback_ns : 0,
                                         std::memory_order_relaxed);
    }

    // Drop the consumed prefix: it is all null by construction, so this only
    // rebases the buffer index space and keeps memory proportional to the
    // live out-of-order window rather than to the absolute window count.
    if (flushed > 0) {
        pending_results_.erase(pending_results_.begin(),
                               pending_results_.begin() + static_cast<ptrdiff_t>(flushed));
        // pending_slots_ is only ever resized while the writer is enabled, so it
        // can be shorter than `flushed` (or empty).  Erasing past its end would
        // be undefined behaviour, so clamp instead of assuming the two buffers
        // have the same length.
        const size_t drop_slots =
            std::min(flushed, pending_slots_.size());
        if (drop_slots > 0) {
            pending_slots_.erase(
                pending_slots_.begin(),
                pending_slots_.begin() + static_cast<ptrdiff_t>(drop_slots));
        }
    }
}

void UhdSc16Pipeline::flush_pending_results()
{
    std::lock_guard<std::mutex> lk(commit_mutex_);
    const bool diag = cfg_.enable_commit_diagnostics;

    // Contract §1.3: after all threads have joined, force out whatever is
    // still buffered (max_windows tail / stop()) in ascending window_id order.
    // Holes are windows that never produced a result (cancelled before
    // admission); they are skipped without shifting later results.
    size_t advanced = 0;
    for (size_t i = 0; i < pending_results_.size(); ++i) {
        if (!pending_results_[i]) {
            continue;
        }
        WindowResult copy = *pending_results_[i];
        pending_results_[i].reset();
        results_.push_back(std::move(copy));
        if (diag) {
            commit_acc_.results_emitted.fetch_add(1, std::memory_order_relaxed);
        }
        // Same in-order hand-off as commit_result(): the tail is settled in
        // ascending window_id order, so the writer's ordering contract holds
        // for the drain path too.
        if (i < pending_slots_.size()) {
            OutputPayloadSlot* emit_slot = pending_slots_[i];
            pending_slots_[i] = nullptr;
            if (emit_slot) {
                output_writer_->try_submit(emit_slot);
            }
        }
        if (result_cb_) {
            result_cb_(results_.back());
        }
        results_.back().commit_time = std::chrono::steady_clock::now();
        advanced = i + 1;
    }
    if (advanced > 0) {
        pending_results_.erase(pending_results_.begin(),
                               pending_results_.begin() + static_cast<ptrdiff_t>(advanced));
        const size_t drop_slots =
            std::min(advanced, pending_slots_.size());
        if (drop_slots > 0) {
            pending_slots_.erase(
                pending_slots_.begin(),
                pending_slots_.begin() + static_cast<ptrdiff_t>(drop_slots));
        }
        next_commit_id_ += advanced;
    }
}

} // namespace pipeline
} // namespace uwb
} // namespace gr
