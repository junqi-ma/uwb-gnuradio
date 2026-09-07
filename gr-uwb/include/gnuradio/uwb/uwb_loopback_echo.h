/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * UwbLoopbackEcho — software channel for monostatic-radar QA (no UHD).
 *
 * Block type: gr::block, 0 stream ports, message PDU only.
 * Scheduler: no forecast / general_work.  The tx handler writes a delayed,
 * scaled, optionally noisy RX window into scratch allocated at make()
 * (capacity max_rx_samples).  The handler never grows those buffers.
 *
 * Input grid: both 737.28 MS/s native (production order: native packet ->
 * loopback -> PDU 65/48) and 998.4 MS/s work (direct path) are accepted;
 * delays/gains are sample-index domain on the input grid and the
 * calibration delay is reported as calibration_delay_native_samples.
 *
 * Output metadata follows the EchoTimer / scheduled-capture schema:
 * window_start_sample=0, pre_guard_samples, capture_samples (TX length),
 * post_guard_samples (tail), sample_count=rx_len, source="loopback" and
 * fake tx/rx times.  The capture geometry is what the PDU 65/48 contract
 * requires (pre_guard + capture <= sample_count).
 */

#pragma once

#include <gnuradio/block.h>
#include <gnuradio/gr_complex.h>
#include <gnuradio/uwb/api.h>
#include <gnuradio/uwb/uwb_defaults.h>
#include <pmt/pmt.h>

#include <atomic>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <random>
#include <string>
#include <vector>

namespace gr {
namespace uwb {

class UWB_API UwbLoopbackEcho : public gr::block
{
public:
    using sptr = std::shared_ptr<UwbLoopbackEcho>;

    static sptr make(size_t pre_guard_samples,
                     size_t tail_samples,
                     const std::vector<double>& delay_samples = {},
                     const std::vector<gr_complex>& gains = {},
                     float noise_std = 0.0f,
                     uint32_t rng_seed = 1,
                     size_t max_tx_samples = 262144,
                     size_t max_rx_samples = 524288,
                     size_t num_delay_samps = 0,
                     double t0_s = 0.0,
                     double pri_s = defaults::kQm35PacketIntervalS);

    ~UwbLoopbackEcho() override;

    size_t pre_guard_samples() const { return d_pre_guard_; }
    size_t tail_samples() const { return d_tail_; }
    const std::vector<double>& delay_samples() const { return d_delay_; }
    const std::vector<gr_complex>& gains() const { return d_gains_; }
    float noise_std() const { return d_noise_std_; }
    uint32_t rng_seed() const { return d_rng_seed_; }
    size_t max_tx_samples() const { return d_max_tx_; }
    size_t max_rx_samples() const { return d_max_rx_; }
    size_t num_delay_samps() const { return d_num_delay_samps_; }
    double t0_s() const { return d_t0_s_; }
    double pri_s() const { return d_pri_s_; }

    uint64_t pdus_received() const
    {
        return d_pdus_received_.load(std::memory_order_relaxed);
    }
    uint64_t pdus_emitted() const
    {
        return d_pdus_emitted_.load(std::memory_order_relaxed);
    }
    uint64_t pdus_dropped() const
    {
        return d_pdus_dropped_.load(std::memory_order_relaxed);
    }

    // Public for gnuradio::make_block_sptr; use make().
    UwbLoopbackEcho(size_t pre_guard_samples,
                    size_t tail_samples,
                    const std::vector<double>& delay_samples,
                    const std::vector<gr_complex>& gains,
                    float noise_std,
                    uint32_t rng_seed,
                    size_t max_tx_samples,
                    size_t max_rx_samples,
                    size_t num_delay_samps,
                    double t0_s,
                    double pri_s);

private:
    void handle_tx(pmt::pmt_t msg);
    void publish_status(const std::string& event, pmt::pmt_t extra = pmt::PMT_NIL);
    void drop_status(const std::string& event, pmt::pmt_t extra = pmt::PMT_NIL);
    void apply_integer_path(const gr_complex* tx,
                            size_t tx_len,
                            size_t rx_len,
                            double delay,
                            gr_complex gain);
    void apply_frac_path(size_t rx_len, double delay, gr_complex gain);
    void prepare_pchip_base(const gr_complex* tx, size_t tx_len, size_t rx_len);

    size_t d_pre_guard_;
    size_t d_tail_;
    std::vector<double> d_delay_;
    std::vector<gr_complex> d_gains_;
    float d_noise_std_;
    uint32_t d_rng_seed_;
    size_t d_max_tx_;
    size_t d_max_rx_;
    size_t d_num_delay_samps_;
    double d_t0_s_;
    double d_pri_s_;

    std::vector<gr_complex> d_rx_;
    std::vector<gr_complex> d_tx_scratch_;
    std::vector<int16_t> d_s16_out_;
    std::vector<double> d_y_i_;
    std::vector<double> d_y_q_;
    std::vector<double> d_d_i_;
    std::vector<double> d_d_q_;

    std::mt19937 d_rng_;

    std::atomic<uint64_t> d_pdus_received_{ 0 };
    std::atomic<uint64_t> d_pdus_emitted_{ 0 };
    std::atomic<uint64_t> d_pdus_dropped_{ 0 };
};

} // namespace uwb
} // namespace gr
