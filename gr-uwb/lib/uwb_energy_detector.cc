/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gnuradio/uwb/uwb_energy_detector.h>
#include <gnuradio/uwb/uwb_detector_core.h>
#include <gnuradio/io_signature.h>

namespace gr {
namespace uwb {

UwbEnergyDetector::UwbEnergyDetector(float threshold, size_t window)
    : gr::sync_block("uwb_energy_detector",
                     gr::io_signature::make(1, 1, sizeof(gr_complex)),
                     gr::io_signature::make2(2,
                                             2,
                                             sizeof(float),
                                             sizeof(unsigned char))),
      d_threshold(threshold),
      d_window(window > 0 ? window : 1)
{
    set_history(d_window);
}

std::shared_ptr<UwbEnergyDetector>
UwbEnergyDetector::make(float threshold, size_t window)
{
    return gnuradio::get_initial_sptr(
        new UwbEnergyDetector(threshold, window));
}

float UwbEnergyDetector::threshold() const { return d_threshold; }

void UwbEnergyDetector::set_threshold(float threshold)
{
    d_threshold = threshold;
}

size_t UwbEnergyDetector::window() const { return d_window; }

void UwbEnergyDetector::set_window(size_t window)
{
    if (window == 0)
        window = 1;
    d_window = window;
    set_history(d_window);
}

int
UwbEnergyDetector::work(int noutput_items,
                        gr_vector_const_void_star& input_items,
                        gr_vector_void_star& output_items)
{
    const auto* in = reinterpret_cast<const gr_complex*>(input_items[0]);
    auto* out_metric = reinterpret_cast<float*>(output_items[0]);
    auto* out_flag = reinterpret_cast<unsigned char*>(output_items[1]);

    // Because this is a sync_block with set_history(d_window), `in` already
    // points d_window-1 samples back, so out_metric[j] is the trailing average
    // of |x|^2 over the window ending at output sample j.
    core::uwb_window_energy(in, noutput_items, d_window, out_metric);

    const unsigned char one = 1;
    const unsigned char zero = 0;
    for (int j = 0; j < noutput_items; ++j)
        out_flag[j] = (out_metric[j] >= d_threshold) ? one : zero;

    // sync_block: produce == consume, one output item per input item.
    return noutput_items;
}

} // namespace uwb
} // namespace gr
