/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gnuradio/uwb/uwb_preamble_detector.h>
#include <gnuradio/uwb/uwb_detector_core.h>
#include <gnuradio/io_signature.h>

#include <algorithm>
#include <fstream>
#include <stdexcept>

namespace gr {
namespace uwb {

UwbPreambleDetector::UwbPreambleDetector(
    const std::vector<std::complex<float>>& known_preamble,
    float threshold,
    float energy_threshold,
    size_t energy_window,
    size_t energy_decimation,
    size_t coarse_decimation,
    size_t coarse_repetitions,
    size_t coarse_margin)
    : gr::sync_block("uwb_preamble_detector",
                     gr::io_signature::make(1, 1, sizeof(gr_complex)),
                     gr::io_signature::make2(2,
                                             2,
                                             sizeof(float),
                                             sizeof(unsigned char))),
      d_threshold(threshold),
      d_template_len(known_preamble.size()),
      d_fir(std::vector<gr_complex>()),
      d_energy_threshold(energy_threshold),
      d_energy_window(energy_window > 0 ? energy_window : 8),
      d_energy_decimation(energy_decimation > 0 ? energy_decimation : 1),
      d_coarse_decimation(coarse_decimation > 0 ? coarse_decimation : 4),
      d_coarse_repetitions(coarse_repetitions > 0 ? coarse_repetitions : 1),
      d_coarse_margin(coarse_margin)
{
    // Normalize the template (buildUwbReference.m convention) and build the
    // matched-filter taps.  The stateless kernel implements a FIR convolution
    //   corr[j] = sum_{k} taps[k] * in[j+k],
    // i.e. the output is the inner product of the input window starting at j
    // with the REVERSED tap vector.  To make corr[j] the correlation of the
    // forward template with that same window (peaking at symbol alignment,
    // matching the MATLAB reference fftfilt(flipud(conj(template)), segment)),
    // the taps must be the reversed conjugated template:
    //   taps[k] = conj(template[L-1-k]).
    std::vector<std::complex<float>> tmpl = known_preamble;
    core::uwb_l2_normalize(tmpl);
    d_template_energy = core::uwb_template_energy(tmpl);

    std::vector<gr_complex> taps;
    taps.reserve(tmpl.size());
    for (auto it = tmpl.rbegin(); it != tmpl.rend(); ++it)
        taps.push_back(std::conj(*it));
    d_fir.set_taps(taps);

    // Bound noutput_items so the preallocated scratch vectors are sufficient
    // and no allocation happens inside work().  Large output buffers let the
    // scheduler hand us bigger chunks, which the multi-symbol coarse scan
    // needs (a chunk should hold several 1016-sample SYNC symbols).
    set_max_noutput_items(static_cast<int>(kMaxItems));
    set_max_output_buffer(0, 1 << 20);
    set_max_output_buffer(1, 1 << 20);
    update_history();

    d_corr.resize(kMaxItems);
    d_winpow.resize(kMaxItems);
    d_energy.resize(kMaxItems);
    d_ranges.reserve(64);
    d_coarse_peaks.reserve(256);

    rebuild_decimated_template();
}

void UwbPreambleDetector::update_history()
{
    // The energy gate reads a window of d_energy_window (decimated points when
    // d_energy_decimation > 1), the matched filter one of d_template_len.
    // History must cover the larger of the two (in raw samples).
    const size_t gate_raw = d_energy_window * d_energy_decimation;
    set_history(std::max(d_template_len, gate_raw));
}

void UwbPreambleDetector::rebuild_decimated_template()
{
    // The FIR taps store the reversed conjugated normalized template:
    //   taps[k] = conj(template[L-1-k])   ->   template[m] = conj(taps[L-1-m]).
    // The decimated template is template[0], template[D], template[2D], ...
    const size_t D = d_coarse_decimation;
    const size_t Ld = d_template_len / D;
    d_sym_ds = Ld;
    const auto& taps = d_fir.taps();
    d_tmpl_ds.clear();
    d_tmpl_ds.reserve(Ld);
    for (size_t j = 0; j < Ld; ++j) {
        const size_t m = j * D;
        d_tmpl_ds.push_back(std::conj(taps[d_template_len - 1 - m]));
    }
    core::uwb_l2_normalize(d_tmpl_ds);
}

std::shared_ptr<UwbPreambleDetector>
UwbPreambleDetector::make(const std::vector<std::complex<float>>& known_preamble,
                          float threshold,
                          float energy_threshold,
                          size_t energy_window,
                          size_t energy_decimation,
                          size_t coarse_decimation,
                          size_t coarse_repetitions,
                          size_t coarse_margin)
{
    return gnuradio::get_initial_sptr(new UwbPreambleDetector(
        known_preamble, threshold, energy_threshold, energy_window,
        energy_decimation, coarse_decimation, coarse_repetitions, coarse_margin));
}

std::shared_ptr<UwbPreambleDetector>
UwbPreambleDetector::make_from_file(const std::string& template_file,
                                    float threshold,
                                    float energy_threshold,
                                    size_t energy_window,
                                    size_t energy_decimation,
                                    size_t coarse_decimation,
                                    size_t coarse_repetitions,
                                    size_t coarse_margin)
{
    std::ifstream f(template_file, std::ios::binary);
    if (!f) {
        throw std::runtime_error("UwbPreambleDetector: cannot open template "
                                 "file " +
                                 template_file);
    }
    f.seekg(0, std::ios::end);
    std::streamsize bytes = f.tellg();
    f.seekg(0, std::ios::beg);
    if (bytes <= 0 || bytes % sizeof(gr_complex) != 0) {
        throw std::runtime_error("UwbPreambleDetector: template file has "
                                 "invalid size");
    }
    std::vector<std::complex<float>> tmpl(
        static_cast<size_t>(bytes) / sizeof(gr_complex));
    f.read(reinterpret_cast<char*>(tmpl.data()), bytes);
    return make(tmpl, threshold, energy_threshold, energy_window,
                energy_decimation, coarse_decimation, coarse_repetitions,
                coarse_margin);
}

float UwbPreambleDetector::threshold() const { return d_threshold; }

void UwbPreambleDetector::set_threshold(float threshold)
{
    d_threshold = threshold;
}

size_t UwbPreambleDetector::template_length() const { return d_template_len; }

float UwbPreambleDetector::energy_threshold() const
{
    return d_energy_threshold;
}

void UwbPreambleDetector::set_energy_threshold(float energy_threshold)
{
    d_energy_threshold = energy_threshold;
}

size_t UwbPreambleDetector::energy_window() const { return d_energy_window; }

void UwbPreambleDetector::set_energy_window(size_t window)
{
    d_energy_window = (window > 0) ? window : 8;
    update_history();
}

size_t UwbPreambleDetector::energy_decimation() const
{
    return d_energy_decimation;
}

void UwbPreambleDetector::set_energy_decimation(size_t decimation)
{
    d_energy_decimation = (decimation > 0) ? decimation : 1;
    update_history();
}

size_t UwbPreambleDetector::coarse_decimation() const
{
    return d_coarse_decimation;
}

void UwbPreambleDetector::set_coarse_decimation(size_t decimation)
{
    d_coarse_decimation = (decimation > 0) ? decimation : 4;
    rebuild_decimated_template();
}

size_t UwbPreambleDetector::coarse_repetitions() const
{
    return d_coarse_repetitions;
}

void UwbPreambleDetector::set_coarse_repetitions(size_t reps)
{
    d_coarse_repetitions = (reps > 0) ? reps : 1;
}

int
UwbPreambleDetector::work(int noutput_items,
                          gr_vector_const_void_star& input_items,
                          gr_vector_void_star& output_items)
{
    const auto* in = reinterpret_cast<const gr_complex*>(input_items[0]);
    auto* out_metric = reinterpret_cast<float*>(output_items[0]);
    auto* out_flag = reinterpret_cast<unsigned char*>(output_items[1]);

    if (d_energy_threshold <= 0.0f) {
        // ---- Reference: full-rate correlation on every sample.
        d_fir.filterN(d_corr.data(), in, static_cast<unsigned long>(noutput_items));
        core::uwb_window_power(in, noutput_items, d_template_len, d_winpow.data());
        core::uwb_normalized_score(d_corr.data(), d_winpow.data(), noutput_items,
                                   d_template_energy, out_metric);
    } else {
        std::fill(out_metric, out_metric + noutput_items, 0.0f);

        if (d_energy_decimation <= 1) {
            // ---- Fast (simple): full-rate energy gate, full-rate correlation
            // only inside candidate regions.
            core::uwb_window_energy(in, noutput_items, d_energy_window,
                                    d_energy.data());
            core::uwb_candidate_ranges(d_energy.data(), noutput_items,
                                       d_energy_threshold, d_energy_window,
                                       d_ranges);
            for (const auto& range : d_ranges) {
                const size_t a = range.first;
                const size_t b = range.second;
                if (b <= a)
                    continue;
                d_fir.filterN(d_corr.data() + a, in + a,
                              static_cast<unsigned long>(b - a));
                core::uwb_window_power(in + a, b - a, d_template_len,
                                       d_winpow.data() + a);
                core::uwb_normalized_score(d_corr.data() + a,
                                           d_winpow.data() + a, b - a,
                                           d_template_energy, out_metric + a);
            }
        } else {
            // ---- Fast (coarse-to-fine): decimated energy gate -> decimated
            // coarse preamble scan -> full-rate fine correlation only in a
            // small window around each coarse peak.
            //
            // Coordinate conventions: `in` carries hist = L-1 history samples
            // before output[0], so `in[hist + j]` == output[j].  The energy
            // gate runs on the current chunk (`in + hist`) and returns ranges
            // in OUTPUT coordinates [a, b).  The coarse scan works in `in`
            // coordinates, so a range is [hist+a, hist+b) with a lookback of
            // `hist` samples before it (scan_start = a, in `in` coordinates).
            // A coarse peak at `in` position p means the template starts at
            // `in` position p, and the full-rate correlation peaks at output
            // index p (the trailing window `in[p .. p+L-1]`).
            const size_t hist = d_template_len - 1;
            core::uwb_energy_gate_strided(in + hist,
                                          noutput_items,
                                          d_energy_decimation,
                                          d_energy_window,
                                          d_energy_threshold,
                                          d_energy_decimation,
                                          d_ranges);

            for (const auto& range : d_ranges) {
                const size_t a = range.first;
                const size_t b = range.second;
                if (b <= a)
                    continue;
                const size_t scan_start = a;          // `in` coords, incl. lookback
                const size_t range_hi = hist + b;     // `in` coords

                float max_metric = 0.0f;
                core::uwb_coarse_peaks(in, scan_start, range_hi, d_tmpl_ds.data(),
                                       d_tmpl_ds.size(), d_coarse_decimation,
                                       d_coarse_repetitions, d_sym_ds,
                                       d_coarse_peak_rel, d_coarse_exist_frac,
                                       /*stride=*/1,
                                       d_sig_ds, d_pow_ds, d_score_ds,
                                       d_metric_ds, d_coarse_peaks, &max_metric);
                if (d_coarse_peaks.empty())
                    continue; // no preamble in this region -> metric stays 0

                for (size_t p : d_coarse_peaks) {
                    // p is the output index where the full-rate correlation
                    // peaks (the trailing window in[p..p+L-1] matches the
                    // template).  A symbol's peak lands in exactly one chunk,
                    // so no dedup is needed.  Skip only peaks whose fine
                    // window does not fit this chunk (the next chunk's coarse
                    // scan re-finds them via its history lookback).
                    if (p >= static_cast<size_t>(noutput_items))
                        continue;
                    const size_t j1 = p + d_coarse_margin;
                    if (j1 >= static_cast<size_t>(noutput_items))
                        continue;
                    const size_t j0 = (p > d_coarse_margin) ? p - d_coarse_margin : 0;
                    const size_t len = j1 - j0 + 1;

                    d_fir.filterN(d_corr.data(), in + j0,
                                  static_cast<unsigned long>(len));
                    core::uwb_window_power(in + j0, len, d_template_len,
                                           d_winpow.data());
                    core::uwb_normalized_score(d_corr.data(), d_winpow.data(),
                                               len, d_template_energy,
                                               out_metric + j0);
                }
            }
        }
    }

    const unsigned char one = 1;
    const unsigned char zero = 0;
    for (int j = 0; j < noutput_items; ++j)
        out_flag[j] = (out_metric[j] >= d_threshold) ? one : zero;

    return noutput_items;
}

} // namespace uwb
} // namespace gr
