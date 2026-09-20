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
 *   <base>.ucr4        concatenated UCR4 records (same wire format as UDP)
 *   <base>.jsonl       one JSON line per pulse.  Per-repetition output is
 *                      represented by compact column arrays in that line.
 *   run.json           static run configuration, written at start()
 *
 * UCR4 (little-endian, 52-byte header + tap_count interleaved SC16 taps):
 *   magic "UCR4" | pulse_id u32 | status u16 | tap_count u16
 *   | repetition_index u16 | repetition_count u16
 *   | sfd_metric f32 | cir_peak_metric f32 | peak_tap i32 | estimator_us u32
 *   | freq_hz f64 | freq_offset_hz f64 | cir_scale f32
 * Reconstruction is FC32 = SC16 * cir_scale (block-floating, same as UDP).
 *
 * Contract:
 *   - status=="ok" frames append one UCR4 record and advance
 *     file_offset_taps by tap_count.
 *   - failed frames (sfd_failed/timing_failed/cir_failed/invalid_input/
 *     internal_error) have tap_count=0 and leave the file offsets unchanged.
 *     In repetition mode they remain observable in the packet's status and
 *     tap_count columns.  No UCR4 record is written for them.
 *   - stop() drains the queue before closing, so JSONL and binary files
 *     never disagree about a half-written frame.
 */

#ifndef INCLUDED_GNURADIO_UWB_UWB_CIR_WRITER_H
#define INCLUDED_GNURADIO_UWB_UWB_CIR_WRITER_H

#include <gnuradio/block.h>
#include <gnuradio/gr_complex.h>
#include <gnuradio/uwb/api.h>
#include <pmt/pmt.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace gr {
namespace uwb {

// On-disk / UDP UCR4 header.  Packed little-endian, 52 bytes.
struct UWB_API Ucr4Header {
    char magic[4];
    uint32_t pulse_id;
    uint16_t status;
    uint16_t tap_count;
    uint16_t repetition_index;
    uint16_t repetition_count;
    float sfd_metric;
    float cir_peak_metric;
    int32_t peak_tap;
    uint32_t estimator_us;
    double freq_hz;
    double freq_offset_hz;
    float cir_scale;
} __attribute__((packed));
static_assert(sizeof(Ucr4Header) == 52, "UCR4 header must be 52 bytes");

inline size_t
ucr4_record_bytes(size_t tap_count)
{
    return sizeof(Ucr4Header) + tap_count * 4;
}

// Block-floating SC16, matching CirUdpSink / UCR4:
//   scale = max(|I|,|Q|) / 32767;  sc16 = round(fc32 / scale) clipped.
inline float
encode_cir_sc16(const gr_complex* in, size_t n, int16_t* interleaved_iq)
{
    float peak = 0.0f;
    for (size_t i = 0; i < n; ++i) {
        peak = std::max(peak, std::fabs(in[i].real()));
        peak = std::max(peak, std::fabs(in[i].imag()));
    }
    if (!std::isfinite(peak) || peak <= 0.0f) {
        std::memset(interleaved_iq, 0, n * 2 * sizeof(int16_t));
        return 0.0f;
    }
    constexpr float kPeak = 32767.0f;
    const float scale = peak / kPeak;
    for (size_t i = 0; i < n; ++i) {
        float re = std::nearbyint(in[i].real() / scale);
        float im = std::nearbyint(in[i].imag() / scale);
        re = std::min(kPeak, std::max(-kPeak, re));
        im = std::min(kPeak, std::max(-kPeak, im));
        interleaved_iq[2 * i] = static_cast<int16_t>(re);
        interleaved_iq[2 * i + 1] = static_cast<int16_t>(im);
    }
    return scale;
}

inline void
decode_cir_sc16(const int16_t* interleaved_iq,
                size_t n,
                float scale,
                gr_complex* out)
{
    for (size_t i = 0; i < n; ++i) {
        out[i] = gr_complex(static_cast<float>(interleaved_iq[2 * i]) * scale,
                            static_cast<float>(interleaved_iq[2 * i + 1]) *
                                scale);
    }
}

class UWB_API UwbCirWriter : public gr::block
{
public:
    using sptr = std::shared_ptr<UwbCirWriter>;

    /**
     * \param directory        output directory (created if missing).
     * \param base_name        file base name (default "cir" → cir.ucr4,
     *                         cir.jsonl).
     * \param write_normalized kept for source compatibility.  Normalized
     *                         taps are still required in the PDU when
     *                         true, but are not written to a second file.
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
    void write_repetition_batch(const pmt::pmt_t& meta,
                                const pmt::pmt_t& data);
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
    void write_ucr4_record(uint32_t pulse_id,
                           uint16_t status,
                           uint16_t tap_count,
                           uint16_t repetition_index,
                           uint16_t repetition_count,
                           float sfd_metric,
                           float peak_metric,
                           int32_t peak_tap,
                           uint32_t estimator_us,
                           double freq_hz,
                           double freq_offset_hz,
                           const gr_complex* taps);

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

    std::ofstream d_raw_;    // <base>.ucr4
    std::ofstream d_jsonl_;  // <base>.jsonl
    std::atomic<uint64_t> d_raw_offset_{ 0 };  // complex taps appended
    std::vector<int16_t> d_sc16_scratch_;      // writer-thread only
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
