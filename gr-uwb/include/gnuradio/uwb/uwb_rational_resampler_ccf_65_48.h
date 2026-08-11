/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Fixed 65/48 CF32 rational resampler block (Level B).
 *
 * Contract rates: 737.28 MS/s → 998.4 MS/s.
 * Block type: gr::block + forecast() + general_work()
 * Tag policy: TPP_DONT + manual propagation (see .cc).
 *
 * Spec: docs/performance/规格_固定65_48重采样core契约.md
 */

#pragma once

#include <gnuradio/block.h>
#include <gnuradio/gr_complex.h>
#include <gnuradio/uwb/api.h>
#include <gnuradio/uwb/uwb_rational_resampler_core.h>
#include <pmt/pmt.h>

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace gr {
namespace uwb {

class UWB_API UwbRationalResamplerCcf65_48 : public gr::block
{
public:
    using sptr = std::shared_ptr<UwbRationalResamplerCcf65_48>;

    static constexpr uint32_t kInterp = core::RationalResampler65_48Core::kInterp;
    static constexpr uint32_t kDecim = core::RationalResampler65_48Core::kDecim;
    static constexpr double kInputRateHz = 737.28e6;
    static constexpr double kOutputRateHz = 998.4e6;

    /**
     * \param taps_file_or_profile  "quality" / "realtime" (resolves under
     *        testdata/resampler_65_48/taps_*.txt) or absolute path to a
     *        float32 binary taps file.
     * \param tag_propagation_enable  If true, manually map supported tags.
     *        If false, drop all tags (TPP_DONT).
     * \param reset_on_discontinuity  If true, core.reset() on overflow /
     *        discontinuity tags and emit a marker on the output.
     * \param num_workers  Ordered multi-worker FIR threads (1 = single-thread).
     *        Default 1; set to hardware_concurrency for multi-core path.
     */
    static sptr make(const std::string& taps_file_or_profile = "quality",
                     bool tag_propagation_enable = true,
                     bool reset_on_discontinuity = true,
                     int num_workers = 1);

    /** QA / benchmark path: construct directly from taps vector. */
    static sptr make_from_taps(const std::vector<float>& taps,
                               bool tag_propagation_enable = true,
                               bool reset_on_discontinuity = true,
                               int num_workers = 1);

    ~UwbRationalResamplerCcf65_48() override;

    void forecast(int noutput_items,
                  gr_vector_int& ninput_items_required) override;

    int general_work(int noutput_items,
                     gr_vector_int& ninput_items,
                     gr_vector_const_void_star& input_items,
                     gr_vector_void_star& output_items) override;

    // --- taps / config ---
    const std::vector<float>& taps() const { return d_taps_; }
    size_t tap_count() const { return d_taps_.size(); }
    size_t arm_length() const { return d_core_->arm_length(); }
    bool tag_propagation_enable() const { return d_tag_prop_; }
    bool reset_on_discontinuity() const { return d_reset_on_disc_; }
    const char* kernel_name() const { return d_core_->kernel_name(); }

    /** Ordered multi-worker FIR count (1 = single-threaded hot path). */
    void set_num_workers(int n);
    int num_workers() const;

    /** Force core kernel for A/B (see RationalResampler65_48Core::set_kernel). */
    void set_kernel(const std::string& name);

    // --- stats (atomic snapshots) ---
    uint64_t input_items() const
    {
        return d_stat_in_.load(std::memory_order_relaxed);
    }
    uint64_t output_items() const
    {
        return d_stat_out_.load(std::memory_order_relaxed);
    }
    uint64_t resets() const
    {
        return d_stat_resets_.load(std::memory_order_relaxed);
    }
    uint64_t discontinuities() const
    {
        return d_stat_disc_.load(std::memory_order_relaxed);
    }
    uint64_t tag_errors() const
    {
        return d_stat_tag_err_.load(std::memory_order_relaxed);
    }

    /** Explicit core reset (also available via discontinuity tags). */
    void reset();

    /** Map input absolute offset → output absolute offset (tag law). */
    int64_t map_input_offset_to_output(int64_t p) const
    {
        return d_core_->map_input_offset_to_output(p);
    }

    // Public for gnuradio::make_block_sptr; use make() / make_from_taps().
    UwbRationalResamplerCcf65_48(const std::vector<float>& taps,
                                 bool tag_propagation_enable,
                                 bool reset_on_discontinuity,
                                 int num_workers = 1);

private:
    std::vector<float> d_taps_;
    std::unique_ptr<core::RationalResampler65_48Core> d_core_;
    bool d_tag_prop_;
    bool d_reset_on_disc_;
    bool d_flushing_ = false;
    bool d_finished_ = false;

    // Absolute sample counters for tag mapping (stream domain).
    uint64_t d_abs_in_ = 0;
    uint64_t d_abs_out_ = 0;
    uint64_t d_disc_epoch_ = 0;

    std::atomic<uint64_t> d_stat_in_{ 0 };
    std::atomic<uint64_t> d_stat_out_{ 0 };
    std::atomic<uint64_t> d_stat_resets_{ 0 };
    std::atomic<uint64_t> d_stat_disc_{ 0 };
    std::atomic<uint64_t> d_stat_tag_err_{ 0 };

    // Interned PMT keys (construction only).
    pmt::pmt_t d_key_rx_time_;
    pmt::pmt_t d_key_rx_rate_;
    pmt::pmt_t d_key_sample_rate_;
    pmt::pmt_t d_key_rx_freq_;
    pmt::pmt_t d_key_overflow_;
    pmt::pmt_t d_key_discontinuity_;
    pmt::pmt_t d_key_resampler_reset_;

    void propagate_tags(int ninput_consumed,
                        int noutput_produced,
                        uint64_t abs_in_before,
                        uint64_t abs_out_before);

    static std::vector<float>
    load_taps_from_profile_or_path(const std::string& taps_file_or_profile);
};

} // namespace uwb
} // namespace gr
