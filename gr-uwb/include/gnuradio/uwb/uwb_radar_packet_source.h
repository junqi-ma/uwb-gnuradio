/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * UwbRadarPacketSource — load one complete UWB packet and republish it.
 *
 * Block type: gr::block, 0 stream ports, message PDU only.
 * Scheduler: no forecast / general_work.  The emit handler copies pulse_id
 * into a fresh metadata dict and publishes the waveform PMT stored at
 * make(); it does not rebuild IQ or generate a 5 ms PRI.  Host-timer
 * blocks such as gr::blocks::message_strobe are not used for PRI.
 *
 * auto_emit: start() publishes once.  EchoTimer (not this block) owns
 * device-time PRI.
 */

#pragma once

#include <gnuradio/block.h>
#include <gnuradio/gr_complex.h>
#include <gnuradio/uwb/api.h>
#include <gnuradio/uwb/uwb_defaults.h>
#include <pmt/pmt.h>

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace gr {
namespace uwb {

class UWB_API UwbRadarPacketSource : public gr::block
{
public:
    using sptr = std::shared_ptr<UwbRadarPacketSource>;

    static sptr make(const std::string& path,
                     double sample_rate,
                     const std::string& sample_format = "fc32",
                     size_t sync_repetitions = 64,
                     const std::string& sfd_mode = "4z2",
                     size_t code_index = 9,
                     double pri_s = defaults::kQm35PacketIntervalS,
                     bool auto_emit = false,
                     const std::string& descriptor_path = "");

    ~UwbRadarPacketSource() override;

    const std::string& path() const { return d_path_; }
    double sample_rate() const { return d_sample_rate_; }
    const std::string& sample_format() const { return d_sample_format_; }
    size_t sync_repetitions() const { return d_sync_repetitions_; }
    const std::string& sfd_mode() const { return d_sfd_mode_; }
    size_t code_index() const { return d_code_index_; }
    double pri_s() const { return d_pri_s_; }
    bool auto_emit() const { return d_auto_emit_; }
    const std::string& descriptor_path() const { return d_descriptor_path_; }
    size_t num_samples() const { return d_num_samples_; }
    const std::vector<gr_complex>& samples() const { return d_fc32_; }
    const std::vector<int16_t>& sc16_samples() const { return d_sc16_; }

    uint64_t emits_received() const
    {
        return d_emits_received_.load(std::memory_order_relaxed);
    }
    uint64_t pdus_emitted() const
    {
        return d_pdus_emitted_.load(std::memory_order_relaxed);
    }
    uint64_t pdus_dropped() const
    {
        return d_pdus_dropped_.load(std::memory_order_relaxed);
    }

    bool start() override;

    // Public for gnuradio::make_block_sptr; use make().
    UwbRadarPacketSource(const std::string& path,
                         double sample_rate,
                         const std::string& sample_format,
                         size_t sync_repetitions,
                         const std::string& sfd_mode,
                         size_t code_index,
                         double pri_s,
                         bool auto_emit,
                         const std::string& descriptor_path);

private:
    void handle_emit(pmt::pmt_t msg);
    void publish_tx(uint64_t pulse_id);
    void publish_status(const std::string& event, pmt::pmt_t extra = pmt::PMT_NIL);

    std::string d_path_;
    double d_sample_rate_;
    std::string d_sample_format_;
    size_t d_sync_repetitions_;
    std::string d_sfd_mode_;
    size_t d_code_index_;
    double d_pri_s_;
    bool d_auto_emit_;
    std::string d_descriptor_path_;
    size_t d_num_samples_ = 0;
    int64_t d_sync_samples_ = -1;
    int64_t d_sfd_samples_ = -1;

    std::vector<gr_complex> d_fc32_;
    std::vector<int16_t> d_sc16_;
    pmt::pmt_t d_iq_pmt_ = pmt::PMT_NIL;

    uint64_t d_next_pulse_id_ = 0;
    std::atomic<uint64_t> d_emits_received_{ 0 };
    std::atomic<uint64_t> d_pdus_emitted_{ 0 };
    std::atomic<uint64_t> d_pdus_dropped_{ 0 };
};

} // namespace uwb
} // namespace gr
