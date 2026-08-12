/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * UwbPduRationalResamplerCcf65_48 — PDU-level fixed 65/48 CF32 resampler.
 *
 * Architecture (docs/performance/分析与下阶段建议_GNURadio软件升采样_65_48.md §3.2):
 *   Capture fixed windows at native 737.28 MS/s with UwbScheduledExtractor,
 *   then upsample only the short window PDU to 998.4 MS/s.  Continuous host
 *   resampling is not real-time; PDU duty-cycle makes the FIR affordable.
 *
 * Block type: gr::block, message-driven, no stream ports.
 * Scheduler: no forecast / general_work; handlers only.  Accepts CF32 PDU or
 * interleaved SC16 PDU input and always emits FC32.  The SC16 conversion is
 * performed only for the scheduled window, never for the continuous stream.
 *
 * Contract: docs/performance/规格_固定65_48重采样core契约.md
 *   Lout = ceil(((N-1)*65 + T)/48)
 *   map(p) = round((p*65 + (T-1)/2)/48)   // group-delay-centered
 * Reuses core::RationalResampler65_48Core (one-shot process + flush per PDU).
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

class UWB_API UwbPduRationalResamplerCcf65_48 : public gr::block
{
public:
    using sptr = std::shared_ptr<UwbPduRationalResamplerCcf65_48>;

    static constexpr uint32_t kInterp = core::RationalResampler65_48Core::kInterp;
    static constexpr uint32_t kDecim = core::RationalResampler65_48Core::kDecim;
    static constexpr double kInputRateHz = 737.28e6;
    static constexpr double kOutputRateHz = 998.4e6;

    /** Emit the full resampled window (default) or only the capture body. */
    enum class EmitPolicy {
        FullWindow = 0,
        CaptureOnly = 1,
    };

    /**
     * \param taps_file_or_profile  "quality" / "realtime" / "quality_minorder"
     *        / "realtime_minorder" (resolves under testdata/resampler_65_48/)
     *        or absolute path to a float32 binary taps file.
     * \param output_sample_rate    meta sample_rate written on emit (default
     *        998.4e6).
     * \param validate_input_rate   If true, drop PDUs whose meta sample_rate
     *        is not ~737.28e6 and publish status "bad_input_rate".
     * \param emit_policy           FullWindow (default) or CaptureOnly.
     */
    static sptr make(const std::string& taps_file_or_profile = "quality_minorder",
                     double output_sample_rate = kOutputRateHz,
                     bool validate_input_rate = true,
                     EmitPolicy emit_policy = EmitPolicy::FullWindow);

    /** QA path: construct from an in-memory taps vector. */
    static sptr make_from_taps(const std::vector<float>& taps,
                               double output_sample_rate = kOutputRateHz,
                               bool validate_input_rate = true,
                               EmitPolicy emit_policy = EmitPolicy::FullWindow);

    ~UwbPduRationalResamplerCcf65_48() override;

    // --- config ---
    const std::vector<float>& taps() const { return d_taps_; }
    size_t tap_count() const { return d_taps_.size(); }
    double output_sample_rate() const { return d_output_rate_; }
    bool validate_input_rate() const { return d_validate_rate_; }
    EmitPolicy emit_policy() const { return d_emit_policy_; }
    void set_emit_policy(EmitPolicy p) { d_emit_policy_ = p; }

    /** Group-delay-centered map: round((p*65 + (T-1)/2)/48). */
    int64_t map_input_offset_to_output(int64_t p) const
    {
        return d_core_->map_input_offset_to_output(p);
    }

    // --- stats (atomic snapshots) ---
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
    uint64_t total_input_samples() const
    {
        return d_total_in_.load(std::memory_order_relaxed);
    }
    uint64_t total_output_samples() const
    {
        return d_total_out_.load(std::memory_order_relaxed);
    }
    uint64_t resets() const
    {
        return d_resets_.load(std::memory_order_relaxed);
    }
    uint64_t short_guard_events() const
    {
        return d_short_guard_.load(std::memory_order_relaxed);
    }
    /** FIR process+flush time only; excludes PMT vector creation/publication. */
    uint64_t resample_total_us() const
    {
        return d_resample_total_us_.load(std::memory_order_relaxed);
    }
    uint64_t resample_max_us() const
    {
        return d_resample_max_us_.load(std::memory_order_relaxed);
    }
    /** Successful PDU handler wall time, including metadata and publication. */
    uint64_t handler_total_us() const
    {
        return d_handler_total_us_.load(std::memory_order_relaxed);
    }
    /** SC16→FC32 conversion immediately before FIR (0 for FC32 input). */
    uint64_t input_convert_total_us() const
    {
        return d_input_convert_total_us_.load(std::memory_order_relaxed);
    }
    /** PMT vector creation plus message_port_pub, nested in handler_total_us. */
    uint64_t publish_total_us() const
    {
        return d_publish_total_us_.load(std::memory_order_relaxed);
    }

    void reset_stats();

    // Public for gnuradio::make_block_sptr; use make() / make_from_taps().
    UwbPduRationalResamplerCcf65_48(const std::vector<float>& taps,
                                    double output_sample_rate,
                                    bool validate_input_rate,
                                    EmitPolicy emit_policy);

private:
    void handle_packet(pmt::pmt_t msg);
    void publish_status(const std::string& event,
                        pmt::pmt_t extra = pmt::PMT_NIL);
    void ensure_scratch(size_t n_out);
    void resample_oneshot(const gr_complex* in, size_t n_in, size_t* n_out);

    static std::vector<float>
    load_taps_from_profile_or_path(const std::string& taps_file_or_profile);

    std::vector<float> d_taps_;
    std::unique_ptr<core::RationalResampler65_48Core> d_core_;
    double d_output_rate_;
    bool d_validate_rate_;
    EmitPolicy d_emit_policy_;

    // Preallocated FIR output scratch (grows only when needed).
    std::vector<gr_complex> d_scratch_;
    // SC16 PDU input is expanded here immediately before FIR.  Fixed window
    // geometry reserves this once; no steady-state handler allocation.
    std::vector<gr_complex> d_input_scratch_;

    // short_guard status is published at most once (then counted silently).
    bool d_short_guard_logged_ = false;

    std::atomic<uint64_t> d_pdus_received_{ 0 };
    std::atomic<uint64_t> d_pdus_emitted_{ 0 };
    std::atomic<uint64_t> d_pdus_dropped_{ 0 };
    std::atomic<uint64_t> d_total_in_{ 0 };
    std::atomic<uint64_t> d_total_out_{ 0 };
    std::atomic<uint64_t> d_resets_{ 0 };
    std::atomic<uint64_t> d_short_guard_{ 0 };
    std::atomic<uint64_t> d_resample_total_us_{ 0 };
    std::atomic<uint64_t> d_resample_max_us_{ 0 };
    std::atomic<uint64_t> d_handler_total_us_{ 0 };
    std::atomic<uint64_t> d_input_convert_total_us_{ 0 };
    std::atomic<uint64_t> d_publish_total_us_{ 0 };
};

} // namespace uwb
} // namespace gr
