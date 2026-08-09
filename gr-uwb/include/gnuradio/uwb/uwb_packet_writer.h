/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * UwbPacketWriter — writes detected UWB packet PDUs to disk (开发需求参考.md §8).
 *
 * Receives packet PDUs (CF32 payload) on the "packet" message port and appends
 * each packet as **SC16** (interleaved little-endian int16 I/Q) to `capture.iq`
 * plus one JSON metadata line to `capture.jsonl`:
 *
 *   {"packet_id":0,"start_sample":4992001,"trigger_sample":4992001,
 *    "sample_rate":998400000,"sample_count":202032,"file_offset_samples":0,
 *    "detection_metric":1.0,"pre_trigger_samples":2032,
 *    "sample_format":"sc16","iq_scale":2112.34}
 *
 * Quantization: int16 = round(float * iq_scale), clipped to [-32768, 32767],
 * with iq_scale = 32767 / max(|I|,|Q|) per packet (1.0 if silent).
 * Reconstruct: float ≈ int16 / iq_scale.
 *
 * `file_offset_samples` is the sample offset within capture.iq (each sample
 * is 4 bytes = 2 × int16).  MATLAB:
 *   [x, meta] = read_uwb_packet("capture.iq", "capture.jsonl", packetId);
 *
 * Debug mode `one_file_per_packet = true` writes `packet_<id>.iq` instead and
 * records a "file" field in the JSONL line.
 *
 * Block type: gr::block with no stream I/O (message-driven).  The message
 * handler only enqueues an immutable PDU into a bounded FIFO; CF32→SC16
 * conversion and file I/O run on a dedicated worker.  Queue saturation is
 * explicit through packets_dropped()/queue_high_watermark().
 */

#pragma once

#include <gnuradio/block.h>
#include <gnuradio/gr_complex.h>
#include <gnuradio/uwb/api.h>
#include <pmt/pmt.h>

#include <cstdint>
#include <array>
#include <atomic>
#include <condition_variable>
#include <fstream>
#include <mutex>
#include <string>
#include <thread>

namespace gr {
namespace uwb {

class UWB_API UwbPacketWriter : public gr::block
{
public:
    using sptr = std::shared_ptr<UwbPacketWriter>;

    /**
     * Make a new UwbPacketWriter block.
     *
     * \param directory            output directory (created if missing).
     * \param base_name            file base name (default "capture" → capture.iq
     *                             / capture.jsonl).
     * \param one_file_per_packet  write packet_<id>.iq per packet instead of a
     *                             shared capture.iq.
     */
    static sptr make(const std::string& directory,
                     const std::string& base_name = "capture",
                     bool one_file_per_packet = false);

    std::string directory() const;
    std::string base_name() const;
    bool one_file_per_packet() const;
    size_t packets_written() const;
    uint64_t samples_written() const;
    uint64_t packets_received() const;
    uint64_t packets_dropped() const;
    size_t queue_high_watermark() const;

    bool start() override;
    bool stop() override;

protected:
    UwbPacketWriter(const std::string& directory,
                    const std::string& base_name,
                    bool one_file_per_packet);

private:
    void handle_packet(pmt::pmt_t msg);
    void write_packet(pmt::pmt_t msg);
    void writer_loop();

    /** Convert CF32 → interleaved SC16; returns iq_scale (float = sc16/scale). */
    static float convert_to_sc16(const gr_complex* in,
                                 size_t n,
                                 std::vector<int16_t>& out);

    std::string d_directory_;
    std::string d_base_name_;
    bool d_one_file_per_packet_;
    std::ofstream d_iq_;   // shared capture.iq (only when not per-packet)
    std::ofstream d_jsonl_;
    std::atomic<uint64_t> d_sample_offset_{ 0 }; // samples appended to capture.iq
    std::atomic<size_t> d_packets_{ 0 };
    std::atomic<uint64_t> d_received_{ 0 };
    std::atomic<uint64_t> d_dropped_{ 0 };
    std::atomic<size_t> d_high_watermark_{ 0 };
    std::vector<int16_t> d_sc16_; // scratch, reused per packet

    static constexpr size_t kQueueCapacity = 16;
    std::array<pmt::pmt_t, kQueueCapacity> d_queue_{};
    size_t d_queue_head_ = 0;
    size_t d_queue_tail_ = 0;
    size_t d_queue_count_ = 0;
    bool d_stop_ = false;
    std::mutex d_mutex_;
    std::condition_variable d_cv_;
    std::thread d_thread_;
};

} // namespace uwb
} // namespace gr
