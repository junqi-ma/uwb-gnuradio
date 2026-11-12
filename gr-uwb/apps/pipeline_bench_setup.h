/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Shared setup helper for the SC16 pipeline benchmark drivers.
 *
 * The SIC/radar geometry must be identical in every driver, otherwise two runs
 * that claim to be "the same configuration" are not.  This used to live inline
 * in uwb_uhd_sc16_pipeline_runner.cc; it is factored out here so the
 * steady-state driver cannot drift from it.
 */

#pragma once

#include <gnuradio/uwb/uwb_uhd_sc16_pipeline.h>

#include <cmath>
#include <cstdint>

namespace gr {
namespace uwb {
namespace pipeline {
namespace bench {

// Derive the SIC search geometry from the window layout:
//   * the expected preamble origin on the work grid is the pre-guard lead-in,
//   * the ROI is +-sic_roi_margin work samples around it,
//   * sub_roi covers the whole resampled window.
inline void derive_sic_geometry(PipelineConfig& cfg, long long sic_roi_margin)
{
    const size_t native_cap = cfg.native_window_capacity();
    const double rate_ratio = cfg.work_rate / cfg.native_rate;
    const size_t work_cap =
        static_cast<size_t>(std::ceil(static_cast<double>(native_cap) * rate_ratio)) + 512;

    cfg.sic_cfg.sample_rate = cfg.work_rate;
    cfg.sic_cfg.samples_per_symbol = 1016;
    // Preserve the caller's configured preamble length (default remains 64).
    cfg.sic_cfg.max_window_samples = work_cap;
    cfg.sic_cfg.max_packet_samples = work_cap;

    const int64_t q = static_cast<int64_t>(
        std::llround(static_cast<double>(cfg.pre_guard_samples) * rate_ratio));
    int64_t roi_lo = q - sic_roi_margin;
    int64_t roi_hi = q + sic_roi_margin;
    if (roi_lo < 0) {
        roi_lo = 0;
    }
    if (roi_hi > static_cast<int64_t>(work_cap)) {
        roi_hi = static_cast<int64_t>(work_cap);
    }
    cfg.sic_cfg.roi_lo = roi_lo;
    cfg.sic_cfg.roi_hi = roi_hi;

    const int64_t work_floor = static_cast<int64_t>(
        std::floor(static_cast<double>(native_cap) * rate_ratio));
    cfg.sic_cfg.sub_roi_lo = 0;
    cfg.sic_cfg.sub_roi_hi = work_floor;
}

} // namespace bench
} // namespace pipeline
} // namespace uwb
} // namespace gr
