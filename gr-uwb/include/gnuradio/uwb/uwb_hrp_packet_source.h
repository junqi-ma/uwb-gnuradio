/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * UwbHrpPacketSource — synthesize one IEEE 802.15.4a HRP BPRF packet and
 * publish it as a TX PDU.
 *
 * Block type: gr::block, 0 stream ports, message PDU only.
 * Scheduler: emit handler only.  IQ is built at make() (and rebuilt if emit
 * carries a u8vector PSDU).  Does not generate a host PRI.
 */

#pragma once

#include <gnuradio/block.h>
#include <gnuradio/gr_complex.h>
#include <gnuradio/uwb/api.h>
#include <gnuradio/uwb/uwb_defaults.h>
#include <gnuradio/uwb/uwb_hrp_mod_core.h>
#include <pmt/pmt.h>

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace gr {
namespace uwb {

class UWB_API UwbHrpPacketSource : public gr::block
{
public:
    using sptr = std::shared_ptr<UwbHrpPacketSource>;

    static sptr make(const std::vector<uint8_t>& psdu,
                     size_t sync_repetitions = 64,
                     const std::string& sfd_mode = "ieee",
                     size_t code_index = 9,
                     float peak_amplitude = mod::kDefaultPeakAmplitude,
                     double pri_s = defaults::kQm35PacketIntervalS,
                     bool auto_emit = false,
                     bool insert_sts = false,
                     bool append_fcs = false);

    ~UwbHrpPacketSource() override;

    size_t sync_repetitions() const { return d_cfg_.sync_repetitions; }
    const std::string& sfd_mode() const { return d_sfd_mode_; }
    size_t code_index() const { return d_cfg_.code_index; }
    float peak_amplitude() const { return d_cfg_.peak_amplitude; }
    double pri_s() const { return d_pri_s_; }
    bool auto_emit() const { return d_auto_emit_; }
    bool insert_sts() const { return d_cfg_.insert_sts; }
    bool append_fcs() const { return d_append_fcs_; }
    size_t num_samples() const { return d_num_samples_; }
    const std::vector<gr_complex>& samples() const { return d_fc32_; }
    const std::vector<uint8_t>& psdu() const { return d_psdu_; }

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

    UwbHrpPacketSource(const std::vector<uint8_t>& psdu,
                       size_t sync_repetitions,
                       const std::string& sfd_mode,
                       size_t code_index,
                       float peak_amplitude,
                       double pri_s,
                       bool auto_emit,
                       bool insert_sts,
                       bool append_fcs);

private:
    void handle_emit(pmt::pmt_t msg);
    void publish_tx(uint64_t pulse_id);
    void publish_status(const std::string& event, pmt::pmt_t extra = pmt::PMT_NIL);
    bool rebuild();

    std::vector<uint8_t> d_psdu_;
    std::string d_sfd_mode_;
    mod::HrpModConfig d_cfg_;
    double d_pri_s_;
    bool d_auto_emit_;
    bool d_append_fcs_;
    size_t d_num_samples_ = 0;
    int64_t d_sync_samples_ = -1;
    int64_t d_sfd_samples_ = -1;

    mod::HrpModScratch d_scratch_;
    std::vector<gr_complex> d_fc32_;
    pmt::pmt_t d_iq_pmt_ = pmt::PMT_NIL;

    uint64_t d_next_pulse_id_ = 0;
    std::atomic<uint64_t> d_emits_received_{ 0 };
    std::atomic<uint64_t> d_pdus_emitted_{ 0 };
    std::atomic<uint64_t> d_pdus_dropped_{ 0 };
};

} // namespace uwb
} // namespace gr
