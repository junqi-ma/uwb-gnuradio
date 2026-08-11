/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gnuradio/uwb/uwb_detector_sc16.h>
#include <gnuradio/uwb/uwb_detector_core.h>
#include <gnuradio/io_signature.h>

#include <algorithm>
#include <exception>
#include <fstream>
#include <stdexcept>
#include <utility>

namespace gr {
namespace uwb {

UwbDetectorSc16::UwbDetectorSc16(
    const std::vector<std::complex<float>>& known_preamble,
    size_t pre_trigger,
    size_t capture,
    float energy_threshold,
    size_t energy_gate_decimation,
    size_t coarse_decimation,
    size_t coarse_repetitions,
    size_t coarse_margin,
    double sample_rate)
    : gr::sync_block("uwb_detector_sc16",
                     gr::io_signature::make(
                         1, 1, sizeof(std::complex<int16_t>)),
                     gr::io_signature::make(0, 0, 0)),
      // Region history must contain both the requested samples before packet
      // start and the delayed gate response; max() silently truncates long
      // user pre-triggers by the response delay.
      sm_(pre_trigger + 32 * energy_gate_decimation + known_preamble.size(),
          energy_threshold,
          energy_gate_decimation,
          /*gate_window=*/32,
          /*holdoff_decimated=*/8,
          /*post_trigger_capture=*/capture),
      d_pre_trigger_(pre_trigger),
      d_capture_(capture),
      d_sample_rate_(sample_rate),
      d_template_len_(known_preamble.size()),
      d_coarse_decimation_(coarse_decimation > 0 ? coarse_decimation : 4),
      d_coarse_repetitions_(coarse_repetitions > 0 ? coarse_repetitions : 1),
      d_coarse_margin_(coarse_margin),
      d_fir(std::vector<gr_complex>())
{
    // Larger chunks -> fewer work() calls, less per-call flowgraph overhead.
    // Buffer scan (benchmark_detector source-search / detector-sparse) knees
    // near 512k–1M items; 64k was a hard self-cap that kept GR work() in the
    // ~4k default regime when only upstream min_output_buffer was enlarged.
    set_max_noutput_items(1048576);
    message_port_register_out(pmt::mp("packet"));

    // Matched-filter taps: the stateless kernel convolves, so use the reversed
    // conjugated template (see uwb_preamble_detector.cc for the rationale).
    std::vector<std::complex<float>> tmpl = known_preamble;
    core::uwb_l2_normalize(tmpl);
    d_template_norm_ = tmpl;
    d_template_energy_ = core::uwb_template_energy(tmpl);

    std::vector<gr_complex> taps;
    taps.reserve(tmpl.size());
    for (auto it = tmpl.rbegin(); it != tmpl.rend(); ++it)
        taps.push_back(std::conj(*it));
    d_fir.set_taps(taps);

    rebuild_decimated_template();

    // Scratch for the fine ROI: 2*(stride*decimation + margin) + 1 samples,
    // because the strided coarse peak is only within stride*D of the true
    // symbol start.
    const size_t half = d_coarse_stride_ * d_coarse_decimation_ + d_coarse_margin_;
    d_corr.resize(2 * half + 1);
    d_winpow.resize(2 * half + 1);
    d_fine_metric.resize(2 * half + 1);
    d_coarse_peaks.reserve(256);
    const size_t region_prebuffer =
        pre_trigger + 32 * energy_gate_decimation + known_preamble.size();
    const size_t max_coarse_samples =
        region_prebuffer + known_preamble.size() *
                               (core::kUwbSyncSymbols +
                                core::kUwbSfdSymbols + 4);
    const size_t max_coarse_decimated =
        max_coarse_samples / d_coarse_decimation_ + 1;
    d_sig_ds_sc16.reserve(max_coarse_decimated);
    d_pow_ds_sc16.reserve(max_coarse_decimated);
    d_score_ds.reserve(max_coarse_decimated);
    d_metric_ds.reserve(max_coarse_decimated);
    d_fine_input_fc32.reserve(2 * half + known_preamble.size());
}

UwbDetectorSc16::~UwbDetectorSc16() { shutdown_worker(); }

void UwbDetectorSc16::rebuild_decimated_template()
{
    const size_t D = d_coarse_decimation_;
    const size_t Ld = d_template_len_ / D;
    d_sym_ds_ = Ld;
    const auto& taps = d_fir.taps(); // taps[k] = conj(template[L-1-k])
    std::vector<std::complex<float>> tmpl_ds;
    tmpl_ds.reserve(Ld);
    for (size_t j = 0; j < Ld; ++j) {
        const size_t m = j * D;
        tmpl_ds.push_back(std::conj(taps[d_template_len_ - 1 - m]));
    }
    core::uwb_l2_normalize(tmpl_ds);
    d_tmpl_ds_q15.resize(Ld);
    d_tmpl_imag_ds_q15.resize(Ld);
    for (size_t j = 0; j < Ld; ++j) {
        const auto q15 = [](float value) {
            return static_cast<int16_t>(std::max(
                -32767.0f, std::min(32767.0f, std::round(value * 32767.0f))));
        };
        d_tmpl_ds_q15[j] = { q15(tmpl_ds[j].real()), q15(tmpl_ds[j].imag()) };
        d_tmpl_imag_ds_q15[j] = {
            static_cast<int16_t>(-d_tmpl_ds_q15[j].imag()),
            d_tmpl_ds_q15[j].real()
        };
    }
}

std::shared_ptr<UwbDetectorSc16>
UwbDetectorSc16::make(const std::vector<std::complex<float>>& known_preamble,
                  size_t pre_trigger,
                  size_t capture,
                  float energy_threshold,
                  size_t energy_gate_decimation,
                  size_t coarse_decimation,
                  size_t coarse_repetitions,
                  size_t coarse_margin,
                  double sample_rate)
{
    return gnuradio::get_initial_sptr(new UwbDetectorSc16(
        known_preamble, pre_trigger, capture, energy_threshold,
        energy_gate_decimation, coarse_decimation, coarse_repetitions,
        coarse_margin, sample_rate));
}

std::shared_ptr<UwbDetectorSc16>
UwbDetectorSc16::make_from_file(const std::string& template_file,
                            size_t pre_trigger,
                            size_t capture,
                            float energy_threshold,
                            size_t energy_gate_decimation,
                            size_t coarse_decimation,
                            size_t coarse_repetitions,
                            size_t coarse_margin,
                            double sample_rate)
{
    std::ifstream f(template_file, std::ios::binary);
    if (!f) {
        throw std::runtime_error("UwbDetectorSc16: cannot open template file " +
                                 template_file);
    }
    f.seekg(0, std::ios::end);
    std::streamsize bytes = f.tellg();
    f.seekg(0, std::ios::beg);
    if (bytes <= 0 || bytes % sizeof(gr_complex) != 0) {
        throw std::runtime_error("UwbDetectorSc16: template file has invalid size");
    }
    std::vector<std::complex<float>> tmpl(
        static_cast<size_t>(bytes) / sizeof(gr_complex));
    f.read(reinterpret_cast<char*>(tmpl.data()), bytes);
    return make(tmpl, pre_trigger, capture, energy_threshold,
                energy_gate_decimation, coarse_decimation, coarse_repetitions,
                coarse_margin, sample_rate);
}

size_t UwbDetectorSc16::pre_trigger() const { return d_pre_trigger_; }
void UwbDetectorSc16::set_pre_trigger(size_t v) { d_pre_trigger_ = v; }
size_t UwbDetectorSc16::capture() const { return d_capture_; }
void UwbDetectorSc16::set_capture(size_t v) { d_capture_ = v; }
double UwbDetectorSc16::sample_rate() const { return d_sample_rate_; }
void UwbDetectorSc16::set_sample_rate(double v) { d_sample_rate_ = v; }
size_t UwbDetectorSc16::coarse_stride() const { return d_coarse_stride_; }
void UwbDetectorSc16::set_coarse_stride(size_t v)
{
    d_coarse_stride_ = (v > 0) ? v : 1;
}

uint64_t UwbDetectorSc16::dropped_regions() const
{
    return sm_.dropped_regions() + d_dropped_jobs_;
}

uint64_t UwbDetectorSc16::work_calls() const { return d_work_calls_; }
uint64_t UwbDetectorSc16::work_items_total() const { return d_work_items_total_; }
int UwbDetectorSc16::work_min_noutput_items() const { return d_work_min_n_; }
int UwbDetectorSc16::work_max_noutput_items() const { return d_work_max_n_; }
double UwbDetectorSc16::work_mean_noutput_items() const
{
    return d_work_calls_ > 0
               ? static_cast<double>(d_work_items_total_) /
                     static_cast<double>(d_work_calls_)
               : 0.0;
}
void UwbDetectorSc16::work_noutput_histogram(uint64_t out[5]) const
{
    for (int i = 0; i < 5; ++i)
        out[i] = d_work_hist_[i];
}
void UwbDetectorSc16::reset_work_stats()
{
    d_work_calls_ = 0;
    d_work_items_total_ = 0;
    d_work_min_n_ = 0;
    d_work_max_n_ = 0;
    for (int i = 0; i < 5; ++i)
        d_work_hist_[i] = 0;
}

int
UwbDetectorSc16::work(int noutput_items,
                  gr_vector_const_void_star& input_items,
                  gr_vector_void_star& /*output_items*/)
{
    // Record requested chunk size (scheduler grant) for buffer/chunk attribution.
    if (noutput_items > 0) {
        ++d_work_calls_;
        d_work_items_total_ += static_cast<uint64_t>(noutput_items);
        if (d_work_min_n_ == 0 || noutput_items < d_work_min_n_)
            d_work_min_n_ = noutput_items;
        if (noutput_items > d_work_max_n_)
            d_work_max_n_ = noutput_items;
        if (noutput_items <= 8192)
            ++d_work_hist_[0];
        else if (noutput_items <= 32768)
            ++d_work_hist_[1];
        else if (noutput_items <= 131072)
            ++d_work_hist_[2];
        else if (noutput_items <= 524288)
            ++d_work_hist_[3];
        else
            ++d_work_hist_[4];
    }

    const auto* in =
        reinterpret_cast<const std::complex<int16_t>*>(input_items[0]);

    const int consumed = (noutput_items > 1) ? noutput_items - 1 : noutput_items;
    sm_.process(in, consumed, d_current_sample_);
    d_current_sample_ += static_cast<uint64_t>(consumed);

    enqueue_ready_regions();
    if (noutput_items == 1)
        wait_for_worker_idle();

    // sync_block with zero outputs: consuming noutput_items and producing
    // nothing on the (empty) output is the tag_debug / null_sink idiom.
    return consumed;
}

bool
UwbDetectorSc16::start()
{
    std::lock_guard<std::mutex> lock(d_job_mutex_);
    d_job_head_ = 0;
    d_job_tail_ = 0;
    d_job_count_ = 0;
    d_jobs_in_flight_ = 0;
    d_worker_stop_ = false;
    d_worker = std::thread(&UwbDetectorSc16::worker_loop, this);
    return true;
}

void
UwbDetectorSc16::enqueue_ready_regions()
{
    while (sm_.region_ready()) {
        const auto handle = sm_.take_region();
        bool queued = false;
        {
            std::lock_guard<std::mutex> lock(d_job_mutex_);
            if (d_job_count_ < d_job_queue.size()) {
                d_job_queue[d_job_tail_] = handle;
                d_job_tail_ = (d_job_tail_ + 1) % d_job_queue.size();
                ++d_job_count_;
                queued = true;
            }
        }
        if (queued) {
            d_job_cv_.notify_one();
        } else {
            sm_.release_region(handle);
            ++d_dropped_jobs_;
        }
    }
}

void
UwbDetectorSc16::worker_loop()
{
    for (;;) {
        UwbDetectorStateMachineSc16::RegionHandle handle;
        {
            std::unique_lock<std::mutex> lock(d_job_mutex_);
            d_job_cv_.wait(lock, [this] { return d_worker_stop_ || d_job_count_ > 0; });
            if (d_job_count_ == 0 && d_worker_stop_)
                return;
            handle = d_job_queue[d_job_head_];
            d_job_head_ = (d_job_head_ + 1) % d_job_queue.size();
            --d_job_count_;
            ++d_jobs_in_flight_;
        }

        try {
            publish_packet(sm_.region(handle));
        } catch (const std::exception& error) {
            d_logger->error("UwbDetectorSc16 worker failed: {}",
                            std::string(error.what()));
        } catch (...) {
            d_logger->error("UwbDetectorSc16 worker failed with an unknown exception");
        }
        sm_.release_region(handle);
        {
            std::lock_guard<std::mutex> lock(d_job_mutex_);
            --d_jobs_in_flight_;
        }
        d_job_cv_.notify_all();
    }
}

void
UwbDetectorSc16::shutdown_worker()
{
    if (!d_worker.joinable())
        return;
    {
        std::lock_guard<std::mutex> lock(d_job_mutex_);
        d_worker_stop_ = true;
    }
    d_job_cv_.notify_one();
    d_worker.join();
}

void
UwbDetectorSc16::wait_for_worker_idle()
{
    std::unique_lock<std::mutex> lock(d_job_mutex_);
    d_job_cv_.wait(lock, [this] { return d_job_count_ == 0 && d_jobs_in_flight_ == 0; });
}

bool
UwbDetectorSc16::stop()
{
    // Flush a region whose tail reaches the end of the stream.  Note: messages
    // published here may not be delivered to a message sink, so captures that
    // end exactly at a packet tail should be padded with trailing silence (as
    // real captures are).
    sm_.flush_region();
    enqueue_ready_regions();
    shutdown_worker();
    return true;
}

void UwbDetectorSc16::publish_packet(const UwbDetectorStateMachineSc16::Region& region)
{
    const size_t n = region.samples.size();
    if (n == 0)
        return;

    // 1. Decimated coarse scan over a preamble-horizon prefix of the buffered
    //    region (incl. pre-buffer holding the first SYNC symbol).  Regions from
    //    real packets are ~264k samples (preamble + full payload); correlating
    //    the payload tail is wasted.  Horizon covers SYNC+SFD (+margin) past
    //    the gate-crossing offset; short regions are scanned in full.
    //
    //    Cost: O((scan_len/D)*(L/D)).  Cutting scan_len from ~264k to ~80k
    //    avoids scanning the payload tail in the Q15 score loop.
    const size_t preamble_span =
        d_template_len_ *
        (core::kUwbSyncSymbols + core::kUwbSfdSymbols + /*margin_syms=*/4);
    const size_t horizon =
        std::max(preamble_span, region.candidate_offset + preamble_span);
    const size_t coarse_scan_end = std::min(n, horizon);

    float mx = 0.0f;
    core::uwb_coarse_peaks_sc16(
        region.samples.data(),
        0, // scan from region start (incl. pre-buffer)
        coarse_scan_end,
        d_tmpl_ds_q15.data(),
        d_tmpl_imag_ds_q15.data(),
        d_tmpl_ds_q15.size(),
        d_coarse_decimation_,
        d_coarse_repetitions_,
        d_sym_ds_,
        d_coarse_peak_rel_,
        d_coarse_exist_frac_,
        d_coarse_stride_,
        d_sig_ds_sc16,
        d_pow_ds_sc16,
        d_score_ds,
        d_metric_ds,
        d_coarse_peaks,
        &mx);
    if (d_coarse_peaks.empty())
        return; // not a preamble — drop (existence check)

    // 2. Full-rate forward normalized correlation.  Candidate/result
    // coordinates are direct SYNC starts, matching MATLAB.
    const size_t half = d_coarse_stride_ * d_coarse_decimation_ + d_coarse_margin_;
    size_t earliest_start = n;
    float best_metric = 0.0f;
    for (size_t p : d_coarse_peaks) {
        if (p >= n || d_template_len_ > n)
            continue;
        const size_t j0 = (p > half) ? p - half : 0;
        const size_t j1 = std::min(p + half, n - d_template_len_);
        const size_t len = j1 - j0 + 1;
        const size_t convert_count =
            std::min(n - j0, len + d_template_len_ - 1);
        if (convert_count < len + d_template_len_ - 1)
            continue;
        d_fine_input_fc32.resize(convert_count);
        constexpr float kInvFullScale = 1.0f / 32768.0f;
        for (size_t i = 0; i < convert_count; ++i) {
            const auto sample = region.samples[j0 + i];
            d_fine_input_fc32[i] = {
                static_cast<float>(sample.real()) * kInvFullScale,
                static_cast<float>(sample.imag()) * kInvFullScale
            };
        }

        size_t local_start = n;
        float local_metric = 0.0f;
        if (core::uwb_full_rate_peak(
                d_fine_input_fc32.data(), convert_count, d_template_norm_.data(),
                d_template_len_, d_template_energy_, 0, len - 1, d_corr.data(),
                d_winpow.data(), d_fine_metric.data(), &local_start,
                &local_metric) &&
            local_metric >= defaults::kDetectorFineThreshold) {
            earliest_start = j0 + local_start;
            best_metric = local_metric;
            break;
        }
    }
    if (earliest_start >= n || best_metric < defaults::kDetectorFineThreshold)
        return; // no confirmed preamble

    size_t backtracked_symbols = 0;
    float start_metric = best_metric;
    const size_t confirmed_start = earliest_start;
    while (backtracked_symbols < defaults::kDetectorMaxBacktrackSymbols &&
           earliest_start >= d_template_len_) {
        const size_t center = earliest_start - d_template_len_;
        const size_t radius = defaults::kDetectorBacktrackRadius;
        const size_t j0 = (center > radius) ? center - radius : 0;
        const size_t j1 = std::min(center + radius, n - d_template_len_);
        const size_t len = j1 - j0 + 1;
        const size_t convert_count =
            std::min(n - j0, len + d_template_len_ - 1);
        if (convert_count < len + d_template_len_ - 1)
            break;
        d_fine_input_fc32.resize(convert_count);
        constexpr float kInvFullScale = 1.0f / 32768.0f;
        for (size_t i = 0; i < convert_count; ++i) {
            const auto sample = region.samples[j0 + i];
            d_fine_input_fc32[i] = {
                static_cast<float>(sample.real()) * kInvFullScale,
                static_cast<float>(sample.imag()) * kInvFullScale
            };
        }
        size_t local_start = convert_count;
        float local_metric = 0.0f;
        if (!core::uwb_full_rate_peak(
                d_fine_input_fc32.data(), convert_count, d_template_norm_.data(),
                d_template_len_, d_template_energy_, 0, len - 1, d_corr.data(),
                d_winpow.data(), d_fine_metric.data(), &local_start,
                &local_metric) ||
            local_metric < defaults::kDetectorBacktrackThreshold)
            break;
        earliest_start = j0 + local_start;
        start_metric = local_metric;
        ++backtracked_symbols;
    }

    // 3. Fine coordinates are already SYNC-start coordinates.
    const uint64_t packet_start =
        region.start_abs + static_cast<uint64_t>(earliest_start);
    const uint64_t timing_seed =
        region.start_abs + static_cast<uint64_t>(confirmed_start);
    const uint64_t trigger = region.start_abs + region.candidate_offset;

    // 4. Capture [start − pre_trigger, start + capture) — i.e. pre_trigger +
    //    capture samples in total — clamped to the region.
    const uint64_t lo = (packet_start >= d_pre_trigger_)
                            ? packet_start - d_pre_trigger_
                            : 0;
    const size_t lo_off =
        (lo >= region.start_abs) ? static_cast<size_t>(lo - region.start_abs) : 0;
    const size_t cap =
        std::min(d_pre_trigger_ + d_capture_, n - lo_off);

    // 5. Emit the PDU.
    pmt::pmt_t meta = pmt::make_dict();
    meta = pmt::dict_add(meta, pmt::mp("packet_id"),
                         pmt::from_long(static_cast<long>(d_packet_id_++)));
    meta = pmt::dict_add(meta, pmt::mp("start_sample"),
                         pmt::from_uint64(packet_start));
    meta = pmt::dict_add(meta, pmt::mp("predicted_start_sample"),
                         pmt::from_uint64(timing_seed));
    meta = pmt::dict_add(meta, pmt::mp("window_start_sample"),
                         pmt::from_uint64(lo));
    meta = pmt::dict_add(meta, pmt::mp("trigger_sample"),
                         pmt::from_uint64(trigger));
    meta = pmt::dict_add(meta, pmt::mp("sample_rate"),
                         pmt::from_double(d_sample_rate_));
    meta = pmt::dict_add(meta, pmt::mp("sample_count"),
                         pmt::from_long(static_cast<long>(cap)));
    meta = pmt::dict_add(meta, pmt::mp("detection_metric"),
                         pmt::from_double(best_metric));
    meta = pmt::dict_add(meta, pmt::mp("start_metric"),
                         pmt::from_double(start_metric));
    meta = pmt::dict_add(meta, pmt::mp("start_backtracked_symbols"),
                         pmt::from_long(static_cast<long>(backtracked_symbols)));
    meta = pmt::dict_add(meta, pmt::mp("threshold"),
                         pmt::from_double(defaults::kDetectorFineThreshold));
    meta = pmt::dict_add(meta, pmt::mp("pre_trigger_samples"),
                         pmt::from_long(static_cast<long>(d_pre_trigger_)));
    meta = pmt::dict_add(meta, pmt::mp("sample_format"), pmt::mp("sc16"));
    meta = pmt::dict_add(meta, pmt::mp("iq_scale"), pmt::from_double(32768.0));

    static_assert(sizeof(std::complex<int16_t>) == 2 * sizeof(int16_t),
                  "SC16 complex samples must be packed I/Q");
    const int16_t* iq = reinterpret_cast<const int16_t*>(
        region.samples.data() + lo_off);
    pmt::pmt_t data = pmt::init_s16vector(cap * 2, iq);

    message_port_pub(pmt::mp("packet"), pmt::cons(meta, data));
}

} // namespace uwb
} // namespace gr
