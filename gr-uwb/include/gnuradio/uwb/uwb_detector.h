/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * UwbDetector — message/PDU UWB packet detection block.
 *
 * Pipeline (开发需求参考.md §5/§6):
 *
 *   IQ stream → decimated energy gate (D=100, state across chunks)
 *            → candidate region buffered wholesale (cross-chunk)
 *            → decimated coarse preamble scan (D=4) confirms the region is a
 *              real preamble and locates symbol peaks
 *            → full-rate fine correlation in a small ROI around each coarse
 *              peak gives the precise symbol ends
 *            → packet start = first symbol end − (L−1); capture
 *              [start − pre_trigger, start + capture) is emitted as a PDU.
 *
 * Because the whole candidate region is buffered before processing, there are
 * no chunk-boundary artifacts (unlike the per-chunk preamble_detector fast
 * path).  Only confirmed preambles produce a PDU — noise / non-preamble bursts
 * that pass the energy gate are rejected by the coarse scan.
 *
 * Block type: gr::sync_block with zero stream outputs (the tag_debug /
 * null_sink idiom); it consumes every input item and publishes PDUs on the
 * "packet" message port.  PDU payload = captured CF32 IQ; metadata:
 *   packet_id, start_sample, trigger_sample, sample_rate, sample_count,
 *   detection_metric.
 *
 * Scheduler semantics: work() scans and consumes the input stream, then puts
 * completed region handles into a bounded FIFO.  One background worker owns
 * coarse/fine correlation and PDU publication.  Region handles remain valid
 * until that worker releases their preallocated pool slots, preserving order.
 * Finite streams leave a one-sample sentinel so the worker drains before EOS.
 */

#pragma once

#include <gnuradio/filter/fir_filter.h>
#include <gnuradio/gr_complex.h>
#include <gnuradio/io_signature.h>
#include <gnuradio/sync_block.h>
#include <gnuradio/uwb/api.h>
#include <gnuradio/uwb/uwb_detector_core.h>
#include <pmt/pmt.h>

#include <array>
#include <condition_variable>
#include <complex>
#include <cstdint>
#include <mutex>
#include <thread>
#include <vector>

namespace gr {
namespace uwb {

class UWB_API UwbDetector : public gr::sync_block
{
public:
    using sptr = std::shared_ptr<UwbDetector>;
    ~UwbDetector() override;

    /**
     * Make a new UwbDetector block.
     *
     * \param known_preamble      one SYNC symbol waveform (L2-normalized
     *                            internally; used for the coarse + fine scans).
     * \param pre_trigger         samples captured before the packet start.
     * \param capture             samples captured after the packet start.
     * \param energy_threshold    decimated energy-gate threshold.
     * \param energy_gate_decimation  stride for the decimated energy gate
     *                                (default 100).
     * \param coarse_decimation   stride for the decimated coarse preamble scan
     *                            (default 4; 4/8 work, 16 aliases).
     * \param coarse_repetitions  SYNC repetitions summed in the coarse metric
     *                            (default 1).
     * \param coarse_margin       fine-correlation half-width around each coarse
     *                            peak (samples).
     */
    static sptr make(const std::vector<std::complex<float>>& known_preamble,
                     size_t pre_trigger = 2032,
                     size_t capture = 200000,
                     float energy_threshold = 1e-3f,
                     size_t energy_gate_decimation = 100,
                     size_t coarse_decimation = 4,
                     size_t coarse_repetitions = 1,
                     size_t coarse_margin = 16);

    /**
     * Same as make(), but loads the template from a binary file of interleaved
     * complex<float> (I/Q/I/Q) samples.
     */
    static sptr make_from_file(const std::string& template_file,
                               size_t pre_trigger = 2032,
                               size_t capture = 200000,
                               float energy_threshold = 1e-3f,
                               size_t energy_gate_decimation = 100,
                               size_t coarse_decimation = 4,
                               size_t coarse_repetitions = 1,
                               size_t coarse_margin = 16);

    size_t pre_trigger() const;
    void set_pre_trigger(size_t pre_trigger);
    size_t capture() const;
    void set_capture(size_t capture);
    size_t coarse_stride() const;
    uint64_t dropped_regions() const;
    void set_coarse_stride(size_t stride);

    // Low-overhead work-chunk stats (filled by work(); read after top_block::run).
    uint64_t work_calls() const;
    uint64_t work_items_total() const;
    int work_min_noutput_items() const;
    int work_max_noutput_items() const;
    double work_mean_noutput_items() const;
    // Histogram buckets: ≤8k, 8k–32k, 32k–128k, 128k–512k, >512k
    void work_noutput_histogram(uint64_t out[5]) const;
    void reset_work_stats();

protected:
    UwbDetector(const std::vector<std::complex<float>>& known_preamble,
                size_t pre_trigger,
                size_t capture,
                float energy_threshold,
                size_t energy_gate_decimation,
                size_t coarse_decimation,
                size_t coarse_repetitions,
                size_t coarse_margin);

    int work(int noutput_items,
             gr_vector_const_void_star& input_items,
             gr_vector_void_star& output_items) override;

    bool start() override;
    bool stop() override;

private:
    void rebuild_decimated_template();
    void publish_packet(const UwbDetectorStateMachine::Region& region);
    void enqueue_ready_regions();
    void worker_loop();
    void shutdown_worker();
    void wait_for_worker_idle();

    UwbDetectorStateMachine sm_;
    uint64_t d_current_sample_ = 0;
    uint64_t d_packet_id_ = 0;
    size_t d_pre_trigger_;
    size_t d_capture_;

    // Coarse-to-fine state (preamble confirmation + precise start)
    size_t d_template_len_;
    size_t d_coarse_decimation_;
    size_t d_coarse_repetitions_;
    size_t d_coarse_margin_;
    size_t d_coarse_stride_ = 1; // coarse correlation stride (R=1)
    float d_coarse_peak_rel_ = 0.5f;
    float d_coarse_exist_frac_ = 0.5f;
    float d_template_energy_ = 0.0f;
    gr::filter::kernel::fir_filter_ccc d_fir;
    std::vector<std::complex<float>> d_tmpl_ds;
    size_t d_sym_ds_ = 0;
    std::vector<std::complex<float>> d_sig_ds;
    std::vector<float> d_pow_ds;
    std::vector<float> d_score_ds;
    std::vector<float> d_metric_ds;
    std::vector<size_t> d_coarse_peaks;

    // Fine-correlation scratch (ROI length is small: 2*coarse_margin+1)
    std::vector<gr_complex> d_corr;
    std::vector<float> d_winpow;
    std::vector<float> d_fine_metric;

    std::thread d_worker;
    std::mutex d_job_mutex_;
    std::condition_variable d_job_cv_;
    std::array<UwbDetectorStateMachine::RegionHandle,
               UwbDetectorStateMachine::kRegionPoolSize> d_job_queue{};
    size_t d_job_head_ = 0;
    size_t d_job_tail_ = 0;
    size_t d_job_count_ = 0;
    size_t d_jobs_in_flight_ = 0;
    bool d_worker_stop_ = false;
    uint64_t d_dropped_jobs_ = 0;

    // work() instrumentation (single consumer thread; no atomics needed)
    uint64_t d_work_calls_ = 0;
    uint64_t d_work_items_total_ = 0;
    int d_work_min_n_ = 0;
    int d_work_max_n_ = 0;
    uint64_t d_work_hist_[5] = {};
};

} // namespace uwb
} // namespace gr
