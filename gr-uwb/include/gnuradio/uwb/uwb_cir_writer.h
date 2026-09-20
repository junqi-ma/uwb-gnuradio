/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * UwbCirWriter — per-pulse CIR disk writer (Radar Step 8).
 *
 * Block type: gr::block, zero stream ports, message PDU only (same pattern
 * as UwbPacketWriter).  The "cir" handler only enqueues an immutable PDU
 * into a bounded FIFO; a single writer worker performs all file I/O in
 * order.
 *
 * Files (in `directory`):
 *   <base>.cf32        raw complex CIR taps (complex64, little-endian)
 *   <base>_norm.cf32   L2-normalized taps (only when write_normalized)
 *   <base>.jsonl       one JSON line per pulse.  Per-repetition output is
 *                      represented by compact column arrays in that line.
 *   run.json           static run configuration, written at start()
 *
 * Contract:
 *   - status=="ok" frames append tap_count complex64 taps to both binary
 *     files (normalized file only if enabled) and advance
 *     file_offset_taps.
 *   - failed frames (sfd_failed/timing_failed/cir_failed/invalid_input/
 *     internal_error) have tap_count=0 and leave the file offsets unchanged.
 *     In repetition mode they remain observable in the packet's status and
 *     tap_count columns.  No taps are ever written for them.
 *   - stop() drains the queue before closing, so JSONL and binary files
 *     never disagree about a half-written frame.
 */

#ifndef INCLUDED_GNURADIO_UWB_UWB_CIR_WRITER_H
#define INCLUDED_GNURADIO_UWB_UWB_CIR_WRITER_H

#include <gnuradio/block.h>
#include <gnuradio/uwb/api.h>
#include <pmt/pmt.h>

#include <array>
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <fstream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace gr {
namespace uwb {

class UWB_API UwbCirWriter : public gr::block
{
public:
    using sptr = std::shared_ptr<UwbCirWriter>;

    /**
     * \param directory        output directory (created if missing).
     * \param base_name        file base name (default "cir" → cir.cf32,
     *                         cir.jsonl; cir_norm.cf32 when enabled).
     * \param write_normalized also write <base>_norm.cf32 from the
     *                         "normalized_taps" metadata vector.
     * \param queue_capacity   bounded queue depth (must be > 0).
     */
    static sptr make(const std::string& directory,
                     const std::string& base_name = "cir",
                     bool write_normalized = false,
                     size_t queue_capacity = 64);

    ~UwbCirWriter() override;

    const std::string& directory() const { return d_directory_; }
    const std::string& base_name() const { return d_base_name_; }
    bool write_normalized() const { return d_write_normalized_; }
    size_t queue_capacity() const { return d_queue_.size(); }

    uint64_t frames_received() const;
    uint64_t frames_written() const;   // ok frames with taps
    uint64_t frames_failed() const;    // JSONL-only frames
    uint64_t frames_dropped() const;   // queue full
    uint64_t frames_invalid() const;   // malformed PDU (no JSONL line)
    uint64_t taps_written() const;
    size_t queue_high_watermark() const;

    bool start() override;
    bool stop() override;

protected:
    UwbCirWriter(const std::string& directory,
                 const std::string& base_name,
                 bool write_normalized,
                 size_t queue_capacity);

private:
    void handle_cir(pmt::pmt_t msg);
    void write_frame(pmt::pmt_t msg);
    void append_repetition_record(const pmt::pmt_t& meta,
                                  uint64_t pulse_id,
                                  uint64_t schedule_index,
                                  const std::string& status,
                                  bool ok,
                                  uint64_t tap_count,
                                  uint64_t raw_offset,
                                  uint64_t norm_offset,
                                  uint64_t peak_tap,
                                  double peak_delay_ns);
    void flush_repetition_group();
    void flush_files_if_due();
    void writer_loop();
    void write_run_json();

    struct RepetitionJsonRecord {
        uint64_t index = 0;
        std::string status;
        uint64_t tap_count = 0;
        uint64_t raw_offset = 0;
        uint64_t norm_offset = 0;
        uint64_t peak_tap = 0;
        double peak_metric = 0.0;
        double peak_delay_ns = 0.0;
        double raw_l2_norm = 0.0;
        uint64_t estimator_us = 0;
    };

    std::string d_directory_;
    std::string d_base_name_;
    bool d_write_normalized_ = false;

    std::ofstream d_raw_;    // <base>.cf32
    std::ofstream d_norm_;   // <base>_norm.cf32 (optional)
    std::ofstream d_jsonl_;  // <base>.jsonl
    std::atomic<uint64_t> d_raw_offset_{ 0 };  // complex taps appended
    std::atomic<uint64_t> d_norm_offset_{ 0 };
    std::atomic<uint64_t> d_frames_written_{ 0 };
    std::atomic<uint64_t> d_frames_failed_{ 0 };
    std::atomic<uint64_t> d_taps_{ 0 };
    std::atomic<uint64_t> d_received_{ 0 };
    std::atomic<uint64_t> d_dropped_{ 0 };
    std::atomic<uint64_t> d_invalid_{ 0 };
    std::atomic<size_t> d_high_watermark_{ 0 };

    // Bounded queue of immutable PDU messages (worker drains on stop).
    std::vector<pmt::pmt_t> d_queue_;
    size_t d_queue_head_ = 0;
    size_t d_queue_tail_ = 0;
    size_t d_queue_count_ = 0;
    bool d_stop_ = false;
    std::mutex d_mutex_;
    std::condition_variable d_cv_;
    std::thread d_thread_;
    uint64_t d_since_flush_ = 0;

    // Writer-thread-only packet aggregation state.  The estimator publishes
    // repetitions in pulse/ordinal order; retaining only the first metadata
    // dictionary plus compact changing columns avoids repeating common JSON.
    bool d_repetition_group_active_ = false;
    uint64_t d_repetition_pulse_id_ = 0;
    uint64_t d_repetition_schedule_index_ = 0;
    uint64_t d_repetition_expected_ = 0;
    pmt::pmt_t d_repetition_common_meta_ = pmt::PMT_NIL;
    std::vector<RepetitionJsonRecord> d_repetition_records_;
};

} // namespace uwb
} // namespace gr

#endif /* INCLUDED_GNURADIO_UWB_UWB_CIR_WRITER_H */
