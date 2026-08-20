/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * UwbPduWindowCrop — SC16/CF32 PDU crop before 65/48.
 *
 * Block type: gr::block, message-driven, no stream ports.
 * Scheduler: no forecast / general_work; packet handler only.
 *
 * Cuts a scheduled native window around predicted_start_sample:
 *
 *   [pred − pre, pred + capture + post)
 *
 * Acquisition PDUs and missing predicted_start pass through.  Output
 * metadata updates window_start / pre / capture / post / sample_count;
 * predicted_start_sample is unchanged.  Crop happens in the input
 * sample-rate domain (typically SC16 @737.28) so the FIR never sees
 * the 590 µs dump window.
 *
 * Hot path: one preallocated scratch; no steady-state growth once the
 * demod window length is known.  CaptureOnly on the resampler is not
 * a substitute — it still runs FIR on the full input.
 */

#pragma once

#include <gnuradio/block.h>
#include <gnuradio/gr_complex.h>
#include <gnuradio/uwb/api.h>
#include <gnuradio/uwb/uwb_defaults.h>
#include <gnuradio/uwb/uwb_window_crop_core.h>
#include <pmt/pmt.h>

#include <atomic>
#include <cstdint>
#include <vector>

namespace gr {
namespace uwb {

class UWB_API UwbPduWindowCrop : public gr::block
{
public:
    using sptr = std::shared_ptr<UwbPduWindowCrop>;

    static sptr make(size_t pre_samples = defaults::kNativeScheduledPreGuard,
                     size_t capture_samples = defaults::kNativeScheduledCapture,
                     size_t post_samples = defaults::kNativeScheduledPostGuard);

    ~UwbPduWindowCrop() override;

    size_t pre_samples() const { return static_cast<size_t>(d_geom_.pre); }
    size_t capture_samples() const
    {
        return static_cast<size_t>(d_geom_.capture);
    }
    size_t post_samples() const { return static_cast<size_t>(d_geom_.post); }
    void set_geometry(size_t pre, size_t capture, size_t post);

    uint64_t pdus_received() const
    {
        return d_pdus_received_.load(std::memory_order_relaxed);
    }
    uint64_t pdus_emitted() const
    {
        return d_pdus_emitted_.load(std::memory_order_relaxed);
    }
    uint64_t pdus_passthrough() const
    {
        return d_pdus_passthrough_.load(std::memory_order_relaxed);
    }
    uint64_t pdus_cropped() const
    {
        return d_pdus_cropped_.load(std::memory_order_relaxed);
    }
    uint64_t pdus_clamped() const
    {
        return d_pdus_clamped_.load(std::memory_order_relaxed);
    }
    uint64_t pdus_dropped() const
    {
        return d_pdus_dropped_.load(std::memory_order_relaxed);
    }
    uint64_t total_input_samples() const
    {
        return d_total_in_.load(std::memory_order_relaxed);
    }
    uint64_t total_output_samples() const
    {
        return d_total_out_.load(std::memory_order_relaxed);
    }

    void reset_stats();

    // Public for gnuradio::make_block_sptr; use make().
    UwbPduWindowCrop(size_t pre_samples,
                     size_t capture_samples,
                     size_t post_samples);

private:
    void handle_packet(pmt::pmt_t msg);
    void publish_status(const char* event, pmt::pmt_t extra = pmt::PMT_NIL);

    core::WindowCropGeom d_geom_;
    std::vector<int16_t> d_s16_scratch_;
    std::vector<gr_complex> d_c32_scratch_;
    bool d_clamp_logged_ = false;

    std::atomic<uint64_t> d_pdus_received_{ 0 };
    std::atomic<uint64_t> d_pdus_emitted_{ 0 };
    std::atomic<uint64_t> d_pdus_passthrough_{ 0 };
    std::atomic<uint64_t> d_pdus_cropped_{ 0 };
    std::atomic<uint64_t> d_pdus_clamped_{ 0 };
    std::atomic<uint64_t> d_pdus_dropped_{ 0 };
    std::atomic<uint64_t> d_total_in_{ 0 };
    std::atomic<uint64_t> d_total_out_{ 0 };
};

} // namespace uwb
} // namespace gr
