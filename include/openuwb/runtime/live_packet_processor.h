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
class PacketProcessor {
public:
    PacketProcessor(const std::vector<float>& taps,
                    const std::vector<std::complex<float>>& tmpl,
                    size_t max_native_samples,
                    PacketSicOptions sic = PacketSicOptions())
        : rs_(taps), template_(tmpl),
          work_(rs_.expected_output_length(max_native_samples)),
          sic_(std::move(sic)) {
        rs_.set_kernel("transposed");
        scratch_.reserve(work_.size());
        if (sic_.enabled) {
            // Buffer sizing is owned by the processor: the caller only
            // supplies the PHY/policy.  Prepare once here, never on the hot
            // path.
            if (sic_.config.max_window_samples == 0)
                sic_.config.max_window_samples = work_.size();
            if (sic_.config.max_packet_samples == 0)
                sic_.config.max_packet_samples = work_.size();
            // Blind acquisition: center the coarse search on the window and
            // let its margin cover the whole buffer.  The interferer position
            // is not known in a live target window, so a zero/zero ROI must
            // not mean "search nowhere".
            if (sic_.config.roi_hi <= sic_.config.roi_lo) {
                sic_.config.roi_lo = 0;
                sic_.config.roi_hi =
                    static_cast<int64_t>(sic_.config.max_window_samples);
            }
            if (sic_.config.sub_roi_hi <= 0)
                sic_.config.sub_roi_hi =
                    static_cast<int64_t>(sic_.config.max_window_samples);
            std::string err;
            if (!sic::prepare_sic_unified(sic_.config, sic_.templates,
                                          sic_scratch_, &err)) {
                throw std::runtime_error("SIC initialization failed: " + err);
            }
            sic_out_.assign(work_.size(), std::complex<float>(0.f, 0.f));
            sic_prepared_ = true;
        }
    }

    PacketProcessingResult process(const PacketRequest& request,
        const std::complex<int16_t>* iq, size_t count,
        const demod::Qm35825Profile& profile, bool auto_cir) {
        PacketProcessingResult out;
        const size_t made = core::resample_sc16_window(rs_, iq, count,
                                                       1.f/32768, work_);
        out.resample_end = std::chrono::steady_clock::now();
        out.sic_end = out.resample_end;
        const std::complex<float>* demod_in = work_.data();

        // Fail-open is the contract: the unmodified work window is demodulated
        // unless SIC both succeeded AND reached L1/L2 with a committed
        // transaction.  A failed/L0 SIC never replaces the target samples.
        if (sic_prepared_ && made > 0) {
            out.sic_executed = true;
            // ROI bounds are window-relative and the prepared buffers are sized
            // for the largest window.  Clamp the per-call copy to the samples
            // actually produced: the reconstruction transaction rejects a
            // sub-ROI that reaches past `made`, and a centered ROI whose high
            // edge is inside the window keeps the Ipatov acquisition scan
            // bounded to real data.  The copy is a flat struct (no heap).
            sic::SicUnifiedConfig call_cfg = sic_.config;
            call_cfg.roi_lo = std::max<int64_t>(
                0, std::min<int64_t>(call_cfg.roi_lo,
                                     static_cast<int64_t>(made)));
            call_cfg.roi_hi = std::min<int64_t>(
                static_cast<int64_t>(made), call_cfg.roi_hi);
            if (call_cfg.roi_hi <= call_cfg.roi_lo) {
                call_cfg.roi_lo = 0;
                call_cfg.roi_hi = static_cast<int64_t>(made);
            }
            call_cfg.sub_roi_lo = std::max<int64_t>(
                0, std::min<int64_t>(call_cfg.sub_roi_lo,
                                     static_cast<int64_t>(made)));
            call_cfg.sub_roi_hi = std::min<int64_t>(
                static_cast<int64_t>(made), call_cfg.sub_roi_hi);
            if (call_cfg.sub_roi_hi <= call_cfg.sub_roi_lo) {
                call_cfg.sub_roi_lo = 0;
                call_cfg.sub_roi_hi = static_cast<int64_t>(made);
            }
            call_cfg.observed_end_sample = 0;
            sic::SicUnifiedResult sic_result;
            sic_result.out = sic_out_.data();
            // The live path consumes `out` only on committed L1/L2.  Skipping
            // the L0/failure full-window copy and the unused i_hat zero-fill
            // removes ~0.58 ms/candidate of pure memory traffic on the common
            // no-interferer path.
            sic_result.copy_out_on_bypass = false;
            sic_result.i_hat = nullptr;
            out.sic_ok = sic::sic_process_unified(
                work_.data(), made, sic_.templates, call_cfg,
                sic_scratch_, sic_result);
            out.sic_end = std::chrono::steady_clock::now();
            out.sic_committed = out.sic_ok && sic_result.committed;
            out.sic_level = sic_result.level;
            out.sic_reason = sic_result.reason ? sic_result.reason : "";
            out.sic_suppression_db = sic_result.suppression_db;
            if (out.sic_committed &&
                (out.sic_level == sic::SicCancellationLevel::L1_Preamble ||
                 out.sic_level == sic::SicCancellationLevel::L2_FullPacket)) {
                demod_in = sic_out_.data();
            }
        }

        out.decoded = demod::core::demodulate_one(demod_in, made, profile,
            request.id, request.predicted_rel_work, 0, template_, scratch_);
        out.retried = should_retry_full_cir(out.decoded, profile, auto_cir);
        out.demod_end = std::chrono::steady_clock::now();
        if (out.retried) {
            auto full = profile;
            full.cir_soft_chip_mode = demod::CirSoftChipMode::Full;
            out.decoded = demod::core::demodulate_one(demod_in, made, full,
                request.id, request.predicted_rel_work, 0, template_, scratch_);
        }
        out.compute_end = std::chrono::steady_clock::now();
        return out;
    }
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

    void stop()
    {
        {
            std::lock_guard<std::mutex> l(mutex_);
            stopping_ = true;
        }
        cv_.notify_all();
        for (auto& t : threads_) {
            if (t.joinable())
                t.join();
        }
        threads_.clear();
    }

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
                              const std::vector<std::complex<int16_t>>& ring)
    {
        size_t slot = 0;
        {
            std::lock_guard<std::mutex> l(mutex_);
            if (stopping_ || free_.empty())
                return false;
            slot = free_.front();
            free_.pop_front();
        }
        auto& job = jobs_[slot];
        static_cast<PacketRequest&>(job) = req;
        job.count = std::min(count, max_window_);
        job.ready = std::chrono::steady_clock::now();
        if (job.count > 0 && !ring.empty()) {
            const size_t mask = ring.size() - 1;
            const size_t offset = static_cast<size_t>(req.window_start) & mask;
            const size_t first = std::min(job.count, ring.size() - offset);
            std::copy_n(ring.data() + offset, first, job.iq.data());
            if (job.count > first)
                std::copy_n(ring.data(), job.count - first, job.iq.data() + first);
        }
        {
            std::lock_guard<std::mutex> l(mutex_);
            pending_.push_back(slot);
        }
        cv_.notify_one();
        return true;
    }

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
