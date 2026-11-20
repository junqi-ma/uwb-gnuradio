/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Native-rate window crop plan (no GNU Radio / PMT).
 *
 * Given a captured PDU covering [window_start, window_start + N) and a
 * predicted_start, select
 *
 *   [pred - pre, pred + capture + post)
 *
 * in the same sample-rate domain.  Acquisition windows and missing
 * predicted_start pass through.  Clamping the start to 0 (or the PDU
 * start) shortens pre and is reported.
 */

#pragma once

#include <cstdint>

namespace gr {
namespace uwb {
namespace core {

struct WindowCropGeom {
    int64_t pre = 7373;
    int64_t capture = 140083;
    int64_t post = 3023;
};

struct WindowCropPlan {
    bool passthrough = false;
    bool clamped_start = false;
    bool clamped_end = false;
    bool empty = false;
    int64_t in_offset = 0;
    int64_t out_count = 0;
    int64_t window_start = 0;
    int64_t pre = 0;
    int64_t capture = 0;
    int64_t post = 0;
    const char* reason = "ok";
};

inline WindowCropPlan plan_window_crop(int64_t window_start,
                                       int64_t sample_count,
                                       int64_t predicted_start,
                                       const WindowCropGeom& geom,
                                       bool acquisition)
{
    WindowCropPlan p;
    if (acquisition) {
        p.passthrough = true;
        p.in_offset = 0;
        p.out_count = sample_count;
        p.window_start = window_start;
        p.reason = "acquisition_passthrough";
        return p;
    }
    if (predicted_start < 0 || sample_count <= 0) {
        p.passthrough = true;
        p.in_offset = 0;
        p.out_count = sample_count < 0 ? 0 : sample_count;
        p.window_start = window_start;
        p.empty = (p.out_count <= 0);
        p.reason = predicted_start < 0 ? "missing_predicted_start" : "empty_input";
        return p;
    }

    const int64_t want_start = predicted_start - geom.pre;
    const int64_t want_end = predicted_start + geom.capture + geom.post;
    const int64_t win_end = window_start + sample_count;

    int64_t crop_start = want_start;
    int64_t crop_end = want_end;
    if (crop_start < 0) {
        crop_start = 0;
        p.clamped_start = true;
    }
    if (crop_start < window_start) {
        crop_start = window_start;
        p.clamped_start = true;
    }
    if (crop_end > win_end) {
        crop_end = win_end;
        p.clamped_end = true;
    }
    if (crop_end < crop_start)
        crop_end = crop_start;

    p.in_offset = crop_start - window_start;
    p.out_count = crop_end - crop_start;
    p.window_start = crop_start;
    if (p.out_count <= 0) {
        p.empty = true;
        p.reason = "empty_crop";
        return p;
    }

    int64_t pre = predicted_start - crop_start;
    if (pre < 0)
        pre = 0;
    if (pre > p.out_count)
        pre = p.out_count;
    int64_t cap = geom.capture;
    if (pre + cap > p.out_count)
        cap = p.out_count - pre;
    if (cap < 0)
        cap = 0;
    int64_t post = p.out_count - pre - cap;
    if (post < 0)
        post = 0;
    p.pre = pre;
    p.capture = cap;
    p.post = post;
    p.reason = (p.clamped_start || p.clamped_end) ? "clamped" : "cropped";
    return p;
}

} // namespace core
} // namespace uwb
} // namespace gr
