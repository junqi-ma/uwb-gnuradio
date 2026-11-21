// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <openuwb/phy/hrp/rx/demod_pipeline.h>
#include <openuwb/runtime/qm35_acquisition_tracker.h>
#include <openuwb/dsp/rational_resampler_core.h>
#include <openuwb/dsp/resample_window.h>
#include <openuwb/runtime/target_lock_core.h>
#include <openuwb/sic/sic_unified.h>
#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace openuwb::live {
// Identity and coordinates carried from planning through completion. Native
// absolute coordinates belong to the scheduler; predicted_rel_work belongs to
// the resampled window. Slot/generation are provenance, not a completion order.
struct PacketRequest {
    uint64_t id = 0;
    int64_t seed = 0;
    int64_t window_start = 0;
    int64_t predicted_rel_work = 0;
    size_t guard = 0;
    uint64_t slot_index = std::numeric_limits<uint64_t>::max();
    uint64_t generation = 0;
    bool target_slot = false;
    bool feeds_lock = false;
};

// Independent CRC verification intentionally remains separate from codec FCS.
inline bool verify_packet_crc(const std::vector<uint8_t>& b) {
    if (b.size() < 2) return false;
    uint16_t crc = 0;
    for (size_t i = 0; i + 2 < b.size(); ++i) {
        crc ^= b[i];
        for (int k = 0; k < 8; ++k)
            crc = (crc >> 1) ^ ((crc & 1) ? 0x8408 : 0);
    }
    return crc == uint16_t(b[b.size()-2] | uint16_t(b.back()) << 8);
}

inline bool should_retry_full_cir(const demod::DemodResult& decoded,
    const demod::Qm35825Profile& profile, bool auto_cir) {
    return !profile.rx_stop_after_sfd && !profile.rx_stop_after_cir &&
        auto_cir &&
        decoded.timing.ok && decoded.sfd.ok &&
        (decoded.status != demod::DemodStatus::Success ||
         !decoded.payload.fcs_pass || !verify_packet_crc(decoded.payload.bytes));
}

struct PacketProcessingResult {
    demod::DemodResult decoded;
    bool retried = false;
    // SIC stage telemetry.  `sic_end == resample_end` and `sic_executed ==
    // false` when `--sic off`, so callers can keep the pre-SIC cost split.
    bool sic_executed = false;
    bool sic_ok = true;
    bool sic_committed = false;
    sic::SicCancellationLevel sic_level = sic::SicCancellationLevel::L0_Bypass;
    const char* sic_reason = "off";
    double sic_suppression_db = 0.0;
    std::chrono::steady_clock::time_point resample_end, sic_end, demod_end,
        compute_end;
};

// Optional SIC stage.  It is only constructed/allocated when enabled; the
// default-constructed value keeps PacketProcessor's historical path with no
// scratch, no residual buffers and no extra branch beyond one bool.
struct PacketSicOptions {
    bool enabled = false;
    sic::SicUnifiedConfig config;
    sic::SicUnifiedTemplates templates;
};

// One instance per worker. Owns all mutable DSP state; borrows the immutable
// template for its lifetime. No Reporter, queue or TargetLockCore reference.
// The constructor and process() are defined in
// lib/runtime/uwb_live_packet_processor.cc (target openuwb_runtime); the
// per-candidate hot path below is unchanged and stays allocation-free.
class PacketProcessor {
public:
    PacketProcessor(const std::vector<float>& taps,
                    const std::vector<std::complex<float>>& tmpl,
                    size_t max_native_samples,
                    PacketSicOptions sic = PacketSicOptions());

    PacketProcessingResult process(
        const PacketRequest& request, const std::complex<int16_t>* iq,
        size_t count, const demod::Qm35825Profile& profile, bool auto_cir);

private:
    core::RationalResampler65_48Core rs_;
    const std::vector<std::complex<float>>& template_;
    std::vector<std::complex<float>> work_;
    demod::core::DemodScratch scratch_;
    PacketSicOptions sic_;
    sic::SicUnifiedScratch sic_scratch_;
    std::vector<std::complex<float>> sic_out_;
    bool sic_prepared_ = false;
};

// Pure conversion of a completed packet to a lock observation. Publishing it
// remains the worker's responsibility; application remains the RX owner's.
inline core::TargetObservation make_target_observation(
    const PacketRequest& request, const demod::DemodResult& decoded,
    double work_rate, double native_rate, double group_delay)
{
    core::TargetObservation obs;
    obs.packet_id = request.id;
    obs.schedule_generation = request.generation;
    obs.slot_index = request.slot_index;
    obs.window_origin = request.window_start;
    obs.predicted_origin = request.seed;
    obs.timing_ok = decoded.timing.ok;
    obs.sfd_ok = decoded.sfd.ok;
    obs.phr_ok = decoded.phr.ok;
    obs.fcs_pass = decoded.payload.fcs_pass && verify_packet_crc(decoded.payload.bytes);
    obs.code_index = 10;
    obs.preamble_repetitions = 256;
    obs.sfd_mode = "4z2";
    obs.quality = decoded.timing.metric;
    obs.from_target_window = request.target_slot;
    if (decoded.timing.ok) {
        obs.observed_origin = request.window_start + core::map_obs_sample_to_native(
            decoded.timing.preamble_start_sample, work_rate, native_rate, group_delay);
        obs.residual = obs.observed_origin - request.seed;
    }
    return obs;
}

struct PacketJob : PacketRequest {
    std::vector<std::complex<int16_t>> iq;
    size_t count = 0;
    std::chrono::steady_clock::time_point ready{};
    explicit PacketJob(size_t max_window = 0) : iq(max_window) {}
};

// Bounded worker slot pool for PacketRequest submissions.
// Slots are pre-allocated once; submit() copies only from the caller's
// power-of-two ring buffer outside the queue mutex and returns false when all
// slots are busy. Workers drain all admitted slots before stopping.
class PacketSlotPool {
public:
    class WorkerLoop {
    public:
        template <class ProcessFn>
        void run(ProcessFn&& process_fn)
        {
            pool_.mark_ready();
            for (;;) {
                size_t slot = 0;
                if (!pool_.pop_pending(slot))
                    return;
                process_fn(pool_.jobs_[slot]);
                pool_.release_slot(slot);
            }
        }

    private:
        friend class PacketSlotPool;
        explicit WorkerLoop(PacketSlotPool& pool) : pool_(pool) {}
        PacketSlotPool& pool_;
    };

    PacketSlotPool(size_t capacity, size_t max_window)
        : max_window_(max_window)
    {
        jobs_.reserve(capacity);
        for (size_t i = 0; i < capacity; ++i) {
            jobs_.emplace_back(max_window);
            free_.push_back(i);
        }
    }

    ~PacketSlotPool() { stop(); }

    PacketSlotPool(const PacketSlotPool&) = delete;
    PacketSlotPool& operator=(const PacketSlotPool&) = delete;

    template <class WorkerMain>
    void start(size_t num_workers, WorkerMain&& worker_main)
    {
        stop();
        {
            std::lock_guard<std::mutex> l(mutex_);
            stopping_ = false;
            ready_ = 0;
            error_.clear();
        }
        try {
            for (size_t worker = 0; worker < num_workers; ++worker) {
                threads_.emplace_back([this, worker, worker_main]() mutable {
                    try {
                        WorkerLoop loop(*this);
                        worker_main(worker, loop);
                    } catch (const std::exception& e) {
                        std::lock_guard<std::mutex> l(mutex_);
                        if (error_.empty())
                            error_ = e.what();
                        stopping_ = true;
                        cv_.notify_all();
                    }
                });
            }
            std::unique_lock<std::mutex> l(mutex_);
            cv_.wait(l, [&] { return ready_ == num_workers || !error_.empty(); });
            if (!error_.empty())
                throw std::runtime_error(error_);
        } catch (...) {
            stop();
            throw;
        }
    }

    void stop();

    size_t backlog() const
    {
        std::lock_guard<std::mutex> l(mutex_);
        return jobs_.size() - free_.size();
    }

    std::string failure() const
    {
        std::lock_guard<std::mutex> l(mutex_);
        return error_;
    }

    [[nodiscard]] bool submit(const PacketRequest& req, size_t count,
                              const std::vector<std::complex<int16_t>>& ring);

private:
    void mark_ready()
    {
        std::lock_guard<std::mutex> l(mutex_);
        ++ready_;
        cv_.notify_all();
    }

    bool pop_pending(size_t& slot)
    {
        std::unique_lock<std::mutex> l(mutex_);
        cv_.wait(l, [&] { return stopping_ || !pending_.empty(); });
        if (pending_.empty())
            return false;
        slot = pending_.front();
        pending_.pop_front();
        return true;
    }

    void release_slot(size_t slot)
    {
        std::lock_guard<std::mutex> l(mutex_);
        free_.push_back(slot);
    }

    size_t max_window_ = 0;
    mutable std::mutex mutex_;
    std::condition_variable cv_;
    std::vector<PacketJob> jobs_;
    std::deque<size_t> free_, pending_;
    std::vector<std::thread> threads_;
    size_t ready_ = 0;
    bool stopping_ = false;
    std::string error_;
};
} // namespace openuwb::live
