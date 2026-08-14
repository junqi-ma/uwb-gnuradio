/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Reusable SC16 preamble verifier (Q15 coarse, full-rate fine, weak-SYNC
 * backtrack).  Extracted from UwbDetectorSc16::publish_packet so the detector
 * QA coordinates stay bit-identical and the auto-scheduled acquisition path
 * can share the same core without nesting another GNU Radio block.
 */

#pragma once

#include <gnuradio/filter/fir_filter.h>
#include <gnuradio/uwb/uwb_defaults.h>
#include <gnuradio/uwb/uwb_detector_core.h>

#include <algorithm>
#include <complex>
#include <cstdint>
#include <vector>

namespace gr {
namespace uwb {
namespace core {

class UwbPreambleVerifierSc16 {
public:
    struct Config {
        size_t coarse_decimation = defaults::kDetectorCoarseDecimation;
        size_t coarse_repetitions = defaults::kDetectorCoarseRepetitions;
        size_t coarse_margin = defaults::kDetectorCoarseMargin;
        size_t coarse_stride = 1;
        float coarse_peak_rel = 0.5f;
        float coarse_exist_frac = 0.5f;
        float fine_threshold = defaults::kDetectorFineThreshold;
        float backtrack_threshold = defaults::kDetectorBacktrackThreshold;
        size_t backtrack_radius = defaults::kDetectorBacktrackRadius;
        size_t max_backtrack = defaults::kDetectorMaxBacktrackSymbols;
    };

    struct Result {
        bool confirmed = false;
        size_t start_offset = 0;      // earliest verified SYNC, region coords
        size_t confirmed_offset = 0;  // first strong SYNC (timing seed)
        float detection_metric = 0.0f;
        float start_metric = 0.0f;
        size_t backtracked_symbols = 0;
    };

    UwbPreambleVerifierSc16() : fir_(std::vector<gr_complex>()) {}

    void configure(const std::vector<std::complex<float>>& known_preamble,
                   const Config& cfg)
    {
        cfg_ = cfg;
        if (cfg_.coarse_decimation == 0)
            cfg_.coarse_decimation = 4;
        if (cfg_.coarse_repetitions == 0)
            cfg_.coarse_repetitions = 1;
        if (cfg_.coarse_stride == 0)
            cfg_.coarse_stride = 1;

        std::vector<std::complex<float>> tmpl = known_preamble;
        uwb_l2_normalize(tmpl);
        template_norm_ = tmpl;
        template_len_ = tmpl.size();
        template_energy_ = uwb_template_energy(tmpl);

        // Same FIR tap construction as the historical UwbDetectorSc16
        // publish_packet path so QA coordinates stay bit-identical.
        std::vector<gr_complex> taps;
        taps.reserve(tmpl.size());
        for (auto it = tmpl.rbegin(); it != tmpl.rend(); ++it)
            taps.push_back(std::conj(*it));
        fir_.set_taps(taps);

        rebuild_decimated_template();
        rebuild_fine_scratch();
        coarse_peaks_.reserve(256);
    }

    void set_coarse_stride(size_t stride)
    {
        cfg_.coarse_stride = stride > 0 ? stride : 1;
        rebuild_fine_scratch();
    }

    size_t coarse_stride() const { return cfg_.coarse_stride; }
    size_t template_length() const { return template_len_; }
    float template_energy() const { return template_energy_; }
    const Config& config() const { return cfg_; }
    const std::vector<std::complex<float>>& template_norm() const
    {
        return template_norm_;
    }

    /**
     * Coarse + fine + backtrack on one buffered energy Region.
     * Coordinates are SYNC-start offsets into `samples` (MATLAB-aligned).
     */
    Result verify(const std::complex<int16_t>* samples,
                  size_t n,
                  size_t candidate_offset)
    {
        Result out;
        if (samples == nullptr || n == 0 || template_len_ == 0)
            return out;

        const size_t preamble_span =
            template_len_ *
            (kUwbSyncSymbols + kUwbSfdSymbols + /*margin_syms=*/4);
        const size_t horizon =
            std::max(preamble_span, candidate_offset + preamble_span);
        const size_t coarse_scan_end = std::min(n, horizon);

        float mx = 0.0f;
        uwb_coarse_peaks_sc16(
            samples,
            0,
            coarse_scan_end,
            tmpl_ds_q15_.data(),
            tmpl_imag_ds_q15_.data(),
            tmpl_ds_q15_.size(),
            cfg_.coarse_decimation,
            cfg_.coarse_repetitions,
            sym_ds_,
            cfg_.coarse_peak_rel,
            cfg_.coarse_exist_frac,
            cfg_.coarse_stride,
            sig_ds_sc16_,
            pow_ds_sc16_,
            score_ds_,
            metric_ds_,
            coarse_peaks_,
            &mx);
        if (coarse_peaks_.empty())
            return out;

        const size_t half =
            cfg_.coarse_stride * cfg_.coarse_decimation + cfg_.coarse_margin;
        size_t earliest_start = n;
        float best_metric = 0.0f;
        for (size_t p : coarse_peaks_) {
            if (p >= n || template_len_ > n)
                continue;
            const size_t j0 = (p > half) ? p - half : 0;
            const size_t j1 = std::min(p + half, n - template_len_);
            const size_t len = j1 - j0 + 1;
            const size_t convert_count =
                std::min(n - j0, len + template_len_ - 1);
            if (convert_count < len + template_len_ - 1)
                continue;
            convert_roi(samples + j0, convert_count);

            size_t local_start = n;
            float local_metric = 0.0f;
            if (uwb_full_rate_peak(
                    fine_input_fc32_.data(), convert_count,
                    template_norm_.data(), template_len_, template_energy_,
                    0, len - 1, corr_.data(), winpow_.data(),
                    fine_metric_.data(), &local_start, &local_metric) &&
                local_metric >= cfg_.fine_threshold) {
                earliest_start = j0 + local_start;
                best_metric = local_metric;
                break;
            }
        }
        if (earliest_start >= n || best_metric < cfg_.fine_threshold)
            return out;

        size_t backtracked_symbols = 0;
        float start_metric = best_metric;
        const size_t confirmed_start = earliest_start;
        while (backtracked_symbols < cfg_.max_backtrack &&
               earliest_start >= template_len_) {
            const size_t center = earliest_start - template_len_;
            const size_t radius = cfg_.backtrack_radius;
            const size_t j0 = (center > radius) ? center - radius : 0;
            const size_t j1 = std::min(center + radius, n - template_len_);
            const size_t len = j1 - j0 + 1;
            const size_t convert_count =
                std::min(n - j0, len + template_len_ - 1);
            if (convert_count < len + template_len_ - 1)
                break;
            convert_roi(samples + j0, convert_count);
            size_t local_start = convert_count;
            float local_metric = 0.0f;
            if (!uwb_full_rate_peak(
                    fine_input_fc32_.data(), convert_count,
                    template_norm_.data(), template_len_, template_energy_,
                    0, len - 1, corr_.data(), winpow_.data(),
                    fine_metric_.data(), &local_start, &local_metric) ||
                local_metric < cfg_.backtrack_threshold)
                break;
            earliest_start = j0 + local_start;
            start_metric = local_metric;
            ++backtracked_symbols;
        }

        out.confirmed = true;
        out.start_offset = earliest_start;
        out.confirmed_offset = confirmed_start;
        out.detection_metric = best_metric;
        out.start_metric = start_metric;
        out.backtracked_symbols = backtracked_symbols;
        return out;
    }

    void reserve_coarse(size_t max_region_samples)
    {
        const size_t max_coarse_decimated =
            max_region_samples / std::max<size_t>(cfg_.coarse_decimation, 1) + 1;
        sig_ds_sc16_.reserve(max_coarse_decimated);
        pow_ds_sc16_.reserve(max_coarse_decimated);
        score_ds_.reserve(max_coarse_decimated);
        metric_ds_.reserve(max_coarse_decimated);
    }

private:
    void rebuild_decimated_template()
    {
        const size_t D = cfg_.coarse_decimation;
        const size_t Ld = template_len_ / D;
        sym_ds_ = Ld;
        const auto& taps = fir_.taps(); // taps[k] = conj(template[L-1-k])
        std::vector<std::complex<float>> tmpl_ds;
        tmpl_ds.reserve(Ld);
        for (size_t j = 0; j < Ld; ++j) {
            const size_t m = j * D;
            tmpl_ds.push_back(std::conj(taps[template_len_ - 1 - m]));
        }
        uwb_l2_normalize(tmpl_ds);
        tmpl_ds_q15_.resize(Ld);
        tmpl_imag_ds_q15_.resize(Ld);
        for (size_t j = 0; j < Ld; ++j) {
            const auto q15 = [](float value) {
                return static_cast<int16_t>(std::max(
                    -32767.0f, std::min(32767.0f, std::round(value * 32767.0f))));
            };
            tmpl_ds_q15_[j] = { q15(tmpl_ds[j].real()), q15(tmpl_ds[j].imag()) };
            tmpl_imag_ds_q15_[j] = {
                static_cast<int16_t>(-tmpl_ds_q15_[j].imag()),
                tmpl_ds_q15_[j].real()
            };
        }
    }

    void rebuild_fine_scratch()
    {
        const size_t half =
            cfg_.coarse_stride * cfg_.coarse_decimation + cfg_.coarse_margin;
        corr_.resize(2 * half + 1);
        winpow_.resize(2 * half + 1);
        fine_metric_.resize(2 * half + 1);
        fine_input_fc32_.reserve(2 * half + template_len_);
    }

    void convert_roi(const std::complex<int16_t>* src, size_t count)
    {
        fine_input_fc32_.resize(count);
        constexpr float kInvFullScale = 1.0f / 32768.0f;
        for (size_t i = 0; i < count; ++i) {
            fine_input_fc32_[i] = {
                static_cast<float>(src[i].real()) * kInvFullScale,
                static_cast<float>(src[i].imag()) * kInvFullScale
            };
        }
    }

    Config cfg_{};
    gr::filter::kernel::fir_filter_ccc fir_{ std::vector<gr_complex>() };
    size_t template_len_ = 0;
    size_t sym_ds_ = 0;
    float template_energy_ = 0.0f;
    std::vector<std::complex<float>> template_norm_;
    std::vector<std::complex<int16_t>> tmpl_ds_q15_;
    std::vector<std::complex<int16_t>> tmpl_imag_ds_q15_;
    std::vector<std::complex<int16_t>> sig_ds_sc16_;
    std::vector<uint64_t> pow_ds_sc16_;
    std::vector<float> score_ds_;
    std::vector<float> metric_ds_;
    std::vector<size_t> coarse_peaks_;
    std::vector<std::complex<float>> corr_;
    std::vector<float> winpow_;
    std::vector<float> fine_metric_;
    std::vector<std::complex<float>> fine_input_fc32_;
};

} // namespace core
} // namespace uwb
} // namespace gr
