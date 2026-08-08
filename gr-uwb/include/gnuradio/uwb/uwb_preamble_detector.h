/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * UWB preamble detector block.
 *
 * Detects the IEEE 802.15.4a HRP SYNC preamble (code index 9, one symbol =
 * 1016 samples at 998.4 MHz) by normalized matched-filter correlation.  The
 * metric at output sample j is the squared normalized correlation of the
 * template-length window ending at j:
 *
 *   metric[j] = |corr[j]|^2 / ( winpow[j] * template_energy + eps )
 *
 * Three operating modes, selected by constructor parameters:
 *
 *  1. Reference (energy_threshold <= 0)          — full-rate correlation on
 *     every sample (exact, ~7 MS/s).
 *  2. Energy-gated fast (energy_threshold > 0, energy_decimation <= 1) —
 *     full-rate correlation only inside candidate regions found by a
 *     full-rate sliding-window energy gate (~60 MS/s).
 *  3. Coarse-to-fine fast (energy_decimation > 1) — D-times decimated energy
 *     gate (default D=100) finds candidate regions; a D-times decimated
 *     coarse correlation (default D=4, MATLAB legacy coarsePreamblePeak) with
 *     multi-symbol repetition accumulation confirms a preamble exists and
 *     locates its peaks; the full-rate correlation then runs only in a small
 *     window around each coarse peak.  Detection is identical to the
 *     reference, at a small fraction of the correlation cost (~200 MS/s
 *     compute throughput).
 *
 * The template is L2-normalized internally (buildUwbReference.m convention).
 * The matched filter uses the VOLK-accelerated stateless FIR kernel
 * (gr::filter::kernel::fir_filter_ccc) with history = template length, so
 * windows span input chunks.
 */

#pragma once

#include <gnuradio/filter/fir_filter.h>
#include <gnuradio/sync_block.h>
#include <gnuradio/uwb/api.h>
#include <complex>
#include <vector>

namespace gr {
namespace uwb {

class UWB_API UwbPreambleDetector : virtual public gr::sync_block
{
public:
    using sptr = std::shared_ptr<UwbPreambleDetector>;

    /**
     * Make a new UwbPreambleDetector block from an in-memory template.
     *
     * \param known_preamble     one SYNC symbol waveform (gr_complex),
     *                           L2-normalized internally before use.
     * \param threshold          correlation-metric gate threshold (0..1)
     * \param energy_threshold   fast-mode energy gate threshold. > 0 enables
     *                           a fast path; <= 0 runs the exact reference.
     * \param energy_window      energy gate window: full-rate samples when
     *                           energy_decimation <= 1, decimated points
     *                           otherwise (default 8 × 100 = 800 samples).
     * \param energy_decimation  stride for the decimated energy gate
     *                           (1 = full rate; 100 = 100× downsampled).
     * \param coarse_decimation  stride for the decimated coarse preamble
     *                           correlation (4 or 8 recommended; 16+ aliases).
     * \param coarse_repetitions SYNC repetitions summed in the coarse metric.
     * \param coarse_margin      fine-correlation half-width around each coarse
     *                           peak (samples).
     */
    static sptr make(const std::vector<std::complex<float>>& known_preamble,
                     float threshold = 0.7f,
                     float energy_threshold = 0.0f,
                     size_t energy_window = 8,
                     size_t energy_decimation = 1,
                     size_t coarse_decimation = 4,
                     size_t coarse_repetitions = 1,
                     size_t coarse_margin = 16);

    /**
     * Same as make(), but loads the template from a binary file of
     * interleaved complex<float> (I/Q/I/Q) samples.
     */
    static sptr make_from_file(const std::string& template_file,
                               float threshold = 0.7f,
                               float energy_threshold = 0.0f,
                               size_t energy_window = 8,
                               size_t energy_decimation = 1,
                               size_t coarse_decimation = 4,
                               size_t coarse_repetitions = 1,
                               size_t coarse_margin = 16);

    float threshold() const;
    void set_threshold(float threshold);
    size_t template_length() const;

    float energy_threshold() const;
    void set_energy_threshold(float energy_threshold);
    size_t energy_window() const;
    void set_energy_window(size_t window);
    size_t energy_decimation() const;
    void set_energy_decimation(size_t decimation);
    size_t coarse_decimation() const;
    void set_coarse_decimation(size_t decimation);
    size_t coarse_repetitions() const;
    void set_coarse_repetitions(size_t reps);

protected:
    UwbPreambleDetector(const std::vector<std::complex<float>>& known_preamble,
                        float threshold,
                        float energy_threshold,
                        size_t energy_window,
                        size_t energy_decimation,
                        size_t coarse_decimation,
                        size_t coarse_repetitions,
                        size_t coarse_margin);

    int work(int noutput_items,
             gr_vector_const_void_star& input_items,
             gr_vector_void_star& output_items) override;

private:
    void update_history();
    void rebuild_decimated_template();

    // Preallocated scratch (no allocation inside work()). Bounded by
    // set_max_noutput_items() in the constructor.
    static constexpr size_t kMaxItems = 16384;

    float d_threshold;
    size_t d_template_len;
    float d_template_energy;
    float d_energy_threshold;
    size_t d_energy_window;
    size_t d_energy_decimation;
    size_t d_coarse_decimation;
    size_t d_coarse_repetitions;
    size_t d_coarse_margin;
    float d_coarse_peak_rel = 0.5f;
    float d_coarse_exist_frac = 0.5f;

    gr::filter::kernel::fir_filter_ccc d_fir;
    std::vector<gr_complex> d_corr; // matched-filter output, noutput_items
    std::vector<float> d_winpow;    // windowed power (normalization), noutput_items
    std::vector<float> d_energy;    // energy gate series, noutput_items
    std::vector<std::pair<size_t, size_t>> d_ranges; // candidate regions

    // Coarse-to-fine state
    std::vector<std::complex<float>> d_tmpl_ds; // decimated template
    size_t d_sym_ds = 0;                        // decimated SYNC symbol length
    std::vector<std::complex<float>> d_sig_ds;
    std::vector<float> d_pow_ds;
    std::vector<float> d_score_ds;
    std::vector<float> d_metric_ds;
    std::vector<size_t> d_coarse_peaks;
};

} // namespace uwb
} // namespace gr
