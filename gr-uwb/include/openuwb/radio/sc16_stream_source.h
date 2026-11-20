/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Minimal native SC16 stream-source contract for the UHD SC16 RX chain.
 *
 * Split out of uwb_uhd_sc16_pipeline.h so that consumers of the source
 * interface (e.g. the fixed-tone-cancel decorator in
 * uwb_fixed_tone_cancel_source.h) do NOT have to include the whole
 * pipeline header -- and with it the SIC / radar / demod consumer headers.
 * This header is pure C++: no GNU Radio, no PMT, no UHD.
 */

#pragma once

#include <openuwb/support/api.h>

#include <complex>
#include <cstddef>
#include <cstdint>
#include <string>

namespace gr {
namespace uwb {
namespace pipeline {

struct StreamReadMeta {
    int64_t start_sample_abs = 0;
    bool has_time_spec = false;
    double time_spec_s = 0.0;
    bool is_gap = false;
    int64_t gap_samples = 0;
    bool is_eof = false;
    int error_code = 0;
};

class UWB_API ISc16StreamSource {
public:
    virtual ~ISc16StreamSource() = default;
    // Open the device/file.  Must NOT begin acquisition: the pipeline calls
    // start_stream() only after every DSP worker published Ready, so a device
    // cannot queue samples behind the ready barrier.
    virtual bool open() = 0;
    // Begin acquisition.  Called once per run, from the RX thread, after the
    // worker-ready barrier.  Default is a no-op for passive/file sources.
    virtual bool start_stream() { return true; }
    // Ask a source to abandon a blocking read.  Called from stop().  Sources
    // with bounded reads (UHD recv timeout, file reads) need nothing here; a
    // source that waits on its own condition variable overrides it so stop()
    // cannot deadlock behind a read.
    virtual void request_stop() {}
    virtual void close() = 0;
    virtual size_t read_samples(std::complex<int16_t>* dest,
                                size_t max_samples,
                                StreamReadMeta& meta) = 0;
    // A fatal source error must be reported here, never disguised as a normal
    // short read or as EOF.  The RX thread checks it after every read.
    virtual bool fatal_error(std::string* msg) const
    {
        if (msg) {
            msg->clear();
        }
        return false;
    }
    virtual double actual_rx_rate() const = 0;
    virtual bool is_eof() const = 0;
};

} // namespace pipeline
} // namespace uwb
} // namespace gr
