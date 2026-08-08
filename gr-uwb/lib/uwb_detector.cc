/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gnuradio/uwb/uwb_detector.h>
#include <gnuradio/uwb/uwb_detector_core.h>
#include <gnuradio/io_signature.h>

#include <algorithm>
#include <fstream>
#include <stdexcept>
#include <utility>

namespace gr {
namespace uwb {

UwbDetector::UwbDetector(
    const std::vector<std::complex<float>>& known_preamble,
    size_t pre_trigger,
    size_t capture,
    float energy_threshold,
    size_t energy_gate_decimation,
    size_t coarse_decimation,
    size_t coarse_repetitions,
    size_t coarse_margin)
    : gr::sync_block("uwb_detector",
                     gr::io_signature::make(1, 1, sizeof(gr_complex)),
                     gr::io_signature::make(0, 0, 0)),
      // The region pre-buffer must span the gate-response delay (gate_window ×
      // gate_decimation samples) plus one SYNC symbol, so the first symbol is
      // always inside the buffered region even though the gate crosses ~3.2k
      // samples into the packet.  The user-facing PDU pre-trigger is smaller.
      sm_(std::max(pre_trigger, 32 * energy_gate_decimation + known_preamble.size()),
          energy_threshold,
          energy_gate_decimation,
          /*gate_window=*/32,
          /*holdoff_decimated=*/8),
      d_pre_trigger_(pre_trigger),
      d_capture_(capture),
      d_template_len_(known_preamble.size()),
      d_coarse_decimation_(coarse_decimation > 0 ? coarse_decimation : 4),
      d_coarse_repetitions_(coarse_repetitions > 0 ? coarse_repetitions : 1),
      d_coarse_margin_(coarse_margin),
      d_fir(std::vector<gr_complex>())
{
    // Larger chunks -> fewer work() calls, less per-call flowgraph overhead.
    set_max_noutput_items(65536);
    message_port_register_out(pmt::mp("packet"));

    // Matched-filter taps: the stateless kernel convolves, so use the reversed
    // conjugated template (see uwb_preamble_detector.cc for the rationale).
    std::vector<std::complex<float>> tmpl = known_preamble;
    core::uwb_l2_normalize(tmpl);
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
}

void UwbDetector::rebuild_decimated_template()
{
    const size_t D = d_coarse_decimation_;
    const size_t Ld = d_template_len_ / D;
    d_sym_ds_ = Ld;
    const auto& taps = d_fir.taps(); // taps[k] = conj(template[L-1-k])
    d_tmpl_ds.clear();
    d_tmpl_ds.reserve(Ld);
    for (size_t j = 0; j < Ld; ++j) {
        const size_t m = j * D;
        d_tmpl_ds.push_back(std::conj(taps[d_template_len_ - 1 - m]));
    }
    core::uwb_l2_normalize(d_tmpl_ds);
}

std::shared_ptr<UwbDetector>
UwbDetector::make(const std::vector<std::complex<float>>& known_preamble,
                  size_t pre_trigger,
                  size_t capture,
                  float energy_threshold,
                  size_t energy_gate_decimation,
                  size_t coarse_decimation,
                  size_t coarse_repetitions,
                  size_t coarse_margin)
{
    return gnuradio::get_initial_sptr(new UwbDetector(
        known_preamble, pre_trigger, capture, energy_threshold,
        energy_gate_decimation, coarse_decimation, coarse_repetitions,
        coarse_margin));
}

std::shared_ptr<UwbDetector>
UwbDetector::make_from_file(const std::string& template_file,
                            size_t pre_trigger,
                            size_t capture,
                            float energy_threshold,
                            size_t energy_gate_decimation,
                            size_t coarse_decimation,
                            size_t coarse_repetitions,
                            size_t coarse_margin)
{
    std::ifstream f(template_file, std::ios::binary);
    if (!f) {
        throw std::runtime_error("UwbDetector: cannot open template file " +
                                 template_file);
    }
    f.seekg(0, std::ios::end);
    std::streamsize bytes = f.tellg();
    f.seekg(0, std::ios::beg);
    if (bytes <= 0 || bytes % sizeof(gr_complex) != 0) {
        throw std::runtime_error("UwbDetector: template file has invalid size");
    }
    std::vector<std::complex<float>> tmpl(
        static_cast<size_t>(bytes) / sizeof(gr_complex));
    f.read(reinterpret_cast<char*>(tmpl.data()), bytes);
    return make(tmpl, pre_trigger, capture, energy_threshold,
                energy_gate_decimation, coarse_decimation, coarse_repetitions,
                coarse_margin);
}

size_t UwbDetector::pre_trigger() const { return d_pre_trigger_; }
void UwbDetector::set_pre_trigger(size_t v) { d_pre_trigger_ = v; }
size_t UwbDetector::capture() const { return d_capture_; }
void UwbDetector::set_capture(size_t v) { d_capture_ = v; }
size_t UwbDetector::coarse_stride() const { return d_coarse_stride_; }
void UwbDetector::set_coarse_stride(size_t v)
{
    d_coarse_stride_ = (v > 0) ? v : 1;
}

int
UwbDetector::work(int noutput_items,
                  gr_vector_const_void_star& input_items,
                  gr_vector_void_star& /*output_items*/)
{
    const auto* in = reinterpret_cast<const gr_complex*>(input_items[0]);

    sm_.process(in, noutput_items, d_current_sample_);
    d_current_sample_ += static_cast<uint64_t>(noutput_items);

    while (sm_.region_ready())
        publish_packet(sm_.take_region());

    // sync_block with zero outputs: consuming noutput_items and producing
    // nothing on the (empty) output is the tag_debug / null_sink idiom.
    return noutput_items;
}

bool
UwbDetector::stop()
{
    // Flush a region whose tail reaches the end of the stream.  Note: messages
    // published here may not be delivered to a message sink, so captures that
    // end exactly at a packet tail should be padded with trailing silence (as
    // real captures are).
    sm_.flush_region();
    while (sm_.region_ready())
        publish_packet(sm_.take_region());
    return true;
}

void UwbDetector::publish_packet(const UwbDetectorStateMachine::Region& region)
{
    const size_t n = region.samples.size();
    if (n == 0)
        return;

    // 1. Decimated coarse scan over the WHOLE buffered region (including the
    //    pre-buffer, which holds the gate-response samples before the crossing
    //    and therefore the first SYNC symbol) confirms a real preamble and
    //    locates the symbol starts.  In silence the correlation is ~0, so no
    //    spurious peaks arise from the pre-buffer.
    float mx = 0.0f;
    core::uwb_coarse_peaks(region.samples.data(),
                           0, // scan the whole region, incl. pre-buffer
                           n,
                           d_tmpl_ds.data(),
                           d_tmpl_ds.size(),
                           d_coarse_decimation_,
                           d_coarse_repetitions_,
                           d_sym_ds_,
                           d_coarse_peak_rel_,
                           d_coarse_exist_frac_,
                           d_coarse_stride_,
                           d_sig_ds,
                           d_pow_ds,
                           d_score_ds,
                           d_metric_ds,
                           d_coarse_peaks,
                           &mx);
    if (d_coarse_peaks.empty())
        return; // not a preamble — drop (existence check)

    // 2. Full-rate fine correlation in a small ROI around each coarse peak.
    //    A coarse peak at region-relative p is the symbol START; the full-rate
    //    metric peaks at the symbol END, p + (L-1).  Track the earliest end
    //    (first SYNC symbol) and the strongest correlation.
    const size_t Lm1 = d_template_len_ - 1;
    // The coarse stride places each peak within stride*D samples of the true
    // symbol start, so the fine search window around (start + L - 1) must be
    // at least that wide plus the margin.
    const size_t half = d_coarse_stride_ * d_coarse_decimation_ + d_coarse_margin_;
    size_t earliest_end = n;
    float best_metric = 0.0f;
    for (size_t p : d_coarse_peaks) {
        if (p + Lm1 >= n)
            continue;
        const size_t center = p + Lm1;
        const size_t j0 = (center > half) ? center - half : 0;
        const size_t j1 = std::min(center + half, n - 1);
        const size_t len = j1 - j0 + 1;

        d_fir.filterN(d_corr.data(), region.samples.data() + j0,
                      static_cast<unsigned long>(len));
        core::uwb_window_power(region.samples.data() + j0, len, d_template_len_,
                               d_winpow.data());
        core::uwb_normalized_score(d_corr.data(), d_winpow.data(), len,
                                   d_template_energy_, d_fine_metric.data());

        size_t local_best = 0;
        for (size_t k = 0; k < len; ++k) {
            if (d_fine_metric[k] > best_metric)
                best_metric = d_fine_metric[k];
            if (d_fine_metric[k] > d_fine_metric[local_best])
                local_best = k;
        }
        // Only accept a coarse peak whose fine argmax is a real correlation
        // (>= 0.5) — weak partial alignments near the region start are
        // ignored.  The earliest such symbol END is the first SYNC symbol end.
        if (d_fine_metric[local_best] >= 0.5f) {
            const size_t end = j0 + local_best;
            if (end < earliest_end)
                earliest_end = end;
        }
    }
    if (earliest_end >= n || best_metric < 0.5f)
        return; // no confirmed preamble

    // 3. Packet start = first SYNC symbol start = earliest end − (L−1).
    const uint64_t packet_start =
        region.start_abs + static_cast<uint64_t>(earliest_end) - Lm1;
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
    meta = pmt::dict_add(meta, pmt::mp("trigger_sample"),
                         pmt::from_uint64(trigger));
    meta = pmt::dict_add(meta, pmt::mp("sample_rate"),
                         pmt::from_double(core::kUwbSampleRateHz));
    meta = pmt::dict_add(meta, pmt::mp("sample_count"),
                         pmt::from_long(static_cast<long>(cap)));
    meta = pmt::dict_add(meta, pmt::mp("detection_metric"),
                         pmt::from_double(best_metric));
    meta = pmt::dict_add(meta, pmt::mp("pre_trigger_samples"),
                         pmt::from_long(static_cast<long>(d_pre_trigger_)));

    // NOTE: pmt::init_c32vector(size, vec) — passing a scalar 0 would bind to
    // the const complex* overload (null pointer) and crash.
    std::vector<gr_complex> iq(region.samples.begin() + lo_off,
                               region.samples.begin() + lo_off + cap);
    pmt::pmt_t data = pmt::init_c32vector(iq.size(), iq);

    message_port_pub(pmt::mp("packet"), pmt::cons(meta, data));
}

} // namespace uwb
} // namespace gr
