/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * UWB energy detector block.
 *
 * Computes a sliding-window average of |x|^2 over a configurable window
 * (causal / trailing window, matching the "energy gate" stage of the MATLAB
 * reference pipeline in UWB_demodulation/visualize_qm35_energy_detection.m).
 *
 * Block type: gr::sync_block (1:1 stream ratio, history of `window` samples).
 *
 *   input  0 : gr_complex  IQ samples
 *   output 0 : float        windowed energy metric per sample
 *   output 1 : unsigned char detection flag (1 if metric >= threshold)
 */

#pragma once

#include <gnuradio/sync_block.h>
#include <gnuradio/uwb/api.h>
#include <complex>

namespace gr {
namespace uwb {

class UWB_API UwbEnergyDetector : virtual public gr::sync_block
{
public:
    using sptr = std::shared_ptr<UwbEnergyDetector>;

    /**
     * Make a new UwbEnergyDetector block.
     *
     * \param threshold energy gate threshold (metric is mean |x|^2 over window)
     * \param window    moving-average window length in samples
     */
    static sptr make(float threshold = 0.5f, size_t window = 16);

    /**
     * Returns the current energy-gate threshold.
     */
    float threshold() const;

    /**
     * Set the energy-gate threshold.
     */
    void set_threshold(float threshold);

    /**
     * Returns the moving-average window length (samples).
     */
    size_t window() const;

    /**
     * Set the moving-average window length (samples). The block history is
     * adjusted accordingly so the window spans previous samples.
     */
    void set_window(size_t window);

protected:
    UwbEnergyDetector(float threshold, size_t window);

    int work(int noutput_items,
             gr_vector_const_void_star& input_items,
             gr_vector_void_star& output_items) override;

private:
    float d_threshold;
    size_t d_window;
};

} // namespace uwb
} // namespace gr
