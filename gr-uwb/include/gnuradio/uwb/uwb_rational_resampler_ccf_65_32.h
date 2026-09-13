/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * UwbRationalResamplerCcf65_32 — fixed 65/32 CF32 tagged-stream resampler.
 *
 * Contract rates: 491.52 MS/s (CG400 native) → 998.4 MS/s (work).
 * Block type: gr::tagged_stream_block, 1 stream in, 1 stream out, one
 * "packet_len"-delimited RADAR RX window per work() call.
 *
 * Why tagged stream (not the continuous 65_48 gr::block): each UWB echo
 * window is an independent burst.  Resampling it with a fresh core
 * (process + flush) reproduces exactly the one-shot PDU resampler
 * (UwbPduRationalResamplerCcf65_32), so the CIR delay axis does not depend
 * on the PRI / inter-burst gap.
 *
 * Tag contract (input, native domain; all offsets relative to the window):
 *   packet_len                          (length tag, required)
 *   window_start_sample, pre_guard_samples, capture_samples,
 *   post_guard_samples, sample_count
 *   calibration_delay_native_samples, pulse_id, schedule_index,
 *   sync_repetitions, sfd_mode, code_index, rx_time, ...
 * Output tags (work domain, at the output window start): sample_rate,
 * window_start_sample, pre_guard_samples, capture_samples,
 * post_guard_samples, sample_count, calibration_delay_work_samples and the
 * radar_meta passthrough/index/length keys, mapped with the same law as the
 * PDU resampler (map_input_offset_to_output).
 */

#pragma once

#include <gnuradio/block.h>
#include <gnuradio/gr_complex.h>
#include <gnuradio/tagged_stream_block.h>
#include <gnuradio/uwb/api.h>
#include <gnuradio/uwb/uwb_rational_resampler_core.h>
#include <pmt/pmt.h>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace gr {
namespace uwb {

class UWB_API UwbRationalResamplerCcf65_32 : public gr::tagged_stream_block
{
public:
    using sptr = std::shared_ptr<UwbRationalResamplerCcf65_32>;

    static constexpr uint32_t kInterp = core::RationalResampler65_32Core::kInterp;
    static constexpr uint32_t kDecim = core::RationalResampler65_32Core::kDecim;
    static constexpr double kInputRateHz = 491.52e6;
    static constexpr double kOutputRateHz = 998.4e6;

    static sptr make(const std::string& taps_file_or_profile = "quality_minorder",
                     bool map_radar_tags = true,
                     int num_workers = 1,
                     const std::string& lengthtagname = "packet_len");

    /** QA path: construct directly from an in-memory taps vector. */
    static sptr make_from_taps(const std::vector<float>& taps,
                               bool map_radar_tags = true,
                               int num_workers = 1,
                               const std::string& lengthtagname = "packet_len");

    ~UwbRationalResamplerCcf65_32() override;

    int calculate_output_stream_length(
        const gr_vector_int& ninput_items) override;

    int work(int noutput_items,
             gr_vector_int& ninput_items,
             gr_vector_const_void_star& input_items,
             gr_vector_void_star& output_items) override;

    const std::vector<float>& taps() const { return d_taps_; }
    size_t tap_count() const { return d_taps_.size(); }
    bool map_radar_tags() const { return d_map_tags_; }
    const char* kernel_name() const { return d_core_->kernel_name(); }

    void set_num_workers(int n);
    int num_workers() const;
    void set_kernel(const std::string& name);

    uint64_t windows() const
    {
        return d_stat_windows_.load(std::memory_order_relaxed);
    }
    uint64_t input_items() const
    {
        return d_stat_in_.load(std::memory_order_relaxed);
    }
    uint64_t output_items() const
    {
        return d_stat_out_.load(std::memory_order_relaxed);
    }
    uint64_t tag_errors() const
    {
        return d_stat_tag_err_.load(std::memory_order_relaxed);
    }

    /** Map a native window offset to the work-domain offset. */
    int64_t map_input_offset_to_output(int64_t p) const
    {
        return d_core_->map_input_offset_to_output(p);
    }

    // Public for gnuradio::make_block_sptr.
    UwbRationalResamplerCcf65_32(const std::vector<float>& taps,
                                 bool map_radar_tags,
                                 int num_workers,
                                 const std::string& lengthtagname);

private:
    void emit_mapped_tags(int ninput_items, size_t produced);

    static std::vector<float>
    load_taps_from_profile_or_path(const std::string& taps_file_or_profile);

    std::vector<float> d_taps_;
    std::unique_ptr<core::RationalResampler65_32Core> d_core_;
    bool d_map_tags_;

    std::atomic<uint64_t> d_stat_windows_{ 0 };
    std::atomic<uint64_t> d_stat_in_{ 0 };
    std::atomic<uint64_t> d_stat_out_{ 0 };
    std::atomic<uint64_t> d_stat_tag_err_{ 0 };
};

} // namespace uwb
} // namespace gr
