/* -*- c++ -*- */
/*
 * Copyright 2026
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Fixed-slot SC16 extractor for the radar t0/T path.
 *
 * Block type: gr::sync_block, one stream input with item size 4 bytes
 * (interleaved int16 I/Q), no stream output.  `work()` bulk-skips samples
 * outside a scheduled window and memcpy's complete SC16 pairs into an
 * preallocated pool.  A worker converts a completed pool slot to a PMT
 * s16vector and publishes it on "packet".  Thus no SC16->FC32 expansion is
 * performed on the continuous 737.28 MS/s stream.
 */

#pragma once

#include <gnuradio/sync_block.h>
#include <gnuradio/uwb/api.h>
#include <gnuradio/uwb/uwb_defaults.h>
#include <gnuradio/uwb/uwb_scheduled_extractor_core.h>
#include <pmt/pmt.h>

#include <array>
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <thread>
#include <vector>

namespace gr {
namespace uwb {

class UWB_API UwbScheduledExtractorSc16 : public gr::sync_block
{
public:
    using sptr = std::shared_ptr<UwbScheduledExtractorSc16>;
    using EmitPolicy = core::EmitPolicy;

    static sptr make(double sample_rate,
                     double packet_interval_s,
                     uint64_t first_packet_sample,
                     size_t pre_guard_samples = defaults::kScheduledPreGuard,
                     size_t capture_samples = defaults::kScheduledCapture,
                     size_t post_guard_samples = defaults::kScheduledPostGuard,
                     size_t pool_size = defaults::kScheduledPoolSize,
                     EmitPolicy emit_policy = EmitPolicy::EverySlot);

    ~UwbScheduledExtractorSc16() override;

    uint64_t scheduled_windows() const;
    uint64_t emitted_windows() const;
    uint64_t dropped_windows() const;
    uint64_t completed_windows() const;
    /** Scheduler stream processing, excluding EOS worker-drain wait. */
    uint64_t process_total_us() const;
    /** memcpy portion of process_total_us. */
    uint64_t copy_total_us() const;
    /** Worker PMT build + message_port_pub time. */
    uint64_t publish_total_us() const;

protected:
    UwbScheduledExtractorSc16(double sample_rate,
                              double packet_interval_s,
                              uint64_t first_packet_sample,
                              size_t pre_guard_samples,
                              size_t capture_samples,
                              size_t post_guard_samples,
                              size_t pool_size,
                              EmitPolicy emit_policy);

    int work(int noutput_items,
             gr_vector_const_void_star& input_items,
             gr_vector_void_star& output_items) override;
    bool start() override;
    bool stop() override;

private:
    struct Slot {
        std::vector<int16_t> iq; // I,Q,I,Q... (2 * capacity), preallocated
        core::WindowMeta meta;
        size_t filled = 0; // complex samples
        std::atomic<bool> in_use{ false };
    };

    static constexpr size_t kMaxPoolSize = 16;
    void begin_next_window();
    void finish_active_window();
    void worker_loop();
    void shutdown_worker();
    void publish_slot(size_t slot_index);

    core::ScheduleConfig d_cfg_;
    uint64_t d_current_sample_ = 0;
    uint64_t d_next_schedule_index_ = 0;
    int d_active_slot_ = -1;
    std::array<Slot, kMaxPoolSize> d_slots_;

    std::thread d_worker_;
    std::mutex d_mutex_;
    std::condition_variable d_cv_;
    std::array<size_t, kMaxPoolSize> d_jobs_{};
    size_t d_job_head_ = 0;
    size_t d_job_tail_ = 0;
    size_t d_job_count_ = 0;
    size_t d_jobs_in_flight_ = 0;
    bool d_stop_worker_ = false;

    std::atomic<uint64_t> d_scheduled_{ 0 };
    std::atomic<uint64_t> d_completed_{ 0 };
    std::atomic<uint64_t> d_emitted_{ 0 };
    std::atomic<uint64_t> d_dropped_{ 0 };
    std::atomic<uint64_t> d_process_total_us_{ 0 };
    std::atomic<uint64_t> d_copy_total_us_{ 0 };
    std::atomic<uint64_t> d_publish_total_us_{ 0 };
};

} // namespace uwb
} // namespace gr
