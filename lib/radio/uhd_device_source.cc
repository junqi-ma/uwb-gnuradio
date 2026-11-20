/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * UHD SC16 Direct RX and Multi-Core Stream Pipeline implementation.
 * Bypasses GNU Radio scheduler and PMT/PDU messaging overhead.
 */

#include <openuwb/runtime/uhd_sc16_pipeline.h>
#include <openuwb/radio/fixed_tone_cancel_source.h>
#include <openuwb/dsp/rational_resampler_core.h>
#include <openuwb/dsp/resample_window.h>
#include <openuwb/support/defaults.h>
#include <openuwb/support/log.h>
#include <volk/volk.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <filesystem>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>

#if defined(__linux__)
#include <pthread.h>
#include <sched.h>
#endif

#if defined(__AVX2__) && defined(__FMA__)
#include <immintrin.h>
#endif

#if defined(UWB_HAVE_UHD) && (UWB_HAVE_UHD)
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/stream.hpp>
#include <uhd/types/metadata.hpp>
#include <uhd/types/stream_cmd.hpp>
#include <uhd/exception.hpp>
#endif

#ifndef UWB_TESTDATA_DIR
#define UWB_TESTDATA_DIR "testdata"
#endif

namespace openuwb {
namespace pipeline {

// UhdDeviceSc16Source
// ---------------------------------------------------------------------------
#if defined(UWB_HAVE_UHD) && (UWB_HAVE_UHD)

struct UhdDeviceSc16Source::Impl {
    uhd::usrp::multi_usrp::sptr usrp;
    uhd::rx_streamer::sptr rx_stream;
    uhd::rx_metadata_t md;
    int64_t current_sample_abs = 0;
};

bool UhdDeviceSc16Source::is_supported()
{
    return true;
}

UhdDeviceSc16Source::UhdDeviceSc16Source(const PipelineConfig& cfg)
    : cfg_(cfg), eof_(false), actual_rate_(0.0)
{
}

UhdDeviceSc16Source::~UhdDeviceSc16Source()
{
    close();
}

bool UhdDeviceSc16Source::open()
{
    close();
    try {
        impl_ = std::make_unique<Impl>();
        impl_->usrp = uhd::usrp::multi_usrp::make(cfg_.uhd_args);
        if (!cfg_.uhd_subdev.empty()) {
            impl_->usrp->set_rx_subdev_spec(cfg_.uhd_subdev);
        }
        if (!cfg_.uhd_clock_source.empty()) {
            impl_->usrp->set_clock_source(cfg_.uhd_clock_source);
        }
        if (!cfg_.uhd_time_source.empty()) {
            impl_->usrp->set_time_source(cfg_.uhd_time_source);
        }
        const size_t channel = cfg_.uhd_channel.empty()
            ? 0 : static_cast<size_t>(std::stoul(cfg_.uhd_channel));
        impl_->usrp->set_rx_rate(cfg_.native_rate, channel);
        actual_rate_ = impl_->usrp->get_rx_rate(channel);
        impl_->usrp->set_rx_freq(cfg_.uhd_center_freq, channel);
        impl_->usrp->set_rx_gain(cfg_.uhd_gain, channel);
        if (!cfg_.uhd_antenna.empty()) {
            impl_->usrp->set_rx_antenna(cfg_.uhd_antenna, channel);
        }
        UWB_LOG_INFO("uhd_pipeline",
                     "UHD RX configured: channel="
                         << channel
                         << " antenna=" << impl_->usrp->get_rx_antenna(channel)
                         << " rate=" << actual_rate_
                         << " frequency=" << impl_->usrp->get_rx_freq(channel)
                         << " gain=" << impl_->usrp->get_rx_gain(channel));

        uhd::stream_args_t stream_args("sc16", "sc16");
        if (!cfg_.uhd_channel.empty()) {
            stream_args.channels = {static_cast<size_t>(std::stoul(cfg_.uhd_channel))};
        }
        impl_->rx_stream = impl_->usrp->get_rx_stream(stream_args);

        // STREAM_MODE_START_CONTINUOUS is deliberately NOT issued here: the
        // pipeline calls start_stream() only after every DSP worker is ready,
        // so the device cannot queue samples behind the barrier.
        eof_ = false;
        impl_->current_sample_abs = 0;
        return true;
    } catch (const std::exception& e) {
        impl_.reset();
        return false;
    }
}

bool UhdDeviceSc16Source::start_stream()
{
    if (!impl_ || !impl_->rx_stream) {
        return false;
    }
    try {
        uhd::stream_cmd_t cmd(uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS);
        cmd.stream_now = true;
        impl_->rx_stream->issue_stream_cmd(cmd);
        return true;
    } catch (const std::exception& e) {
        UWB_LOG_ERR("uhd_pipeline",
                    "UhdDeviceSc16Source: issue_stream_cmd(START_CONTINUOUS) "
                    "failed: "
                        << e.what());
        return false;
    }
}

void UhdDeviceSc16Source::close()
{
    if (impl_ && impl_->rx_stream) {
        try {
            uhd::stream_cmd_t cmd(uhd::stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS);
            impl_->rx_stream->issue_stream_cmd(cmd);
        } catch (...) {}
    }
    impl_.reset();
    eof_ = true;
}

size_t UhdDeviceSc16Source::read_samples(std::complex<int16_t>* dest,
                                         size_t max_samples,
                                         StreamReadMeta& meta)
{
    meta = StreamReadMeta{};
    if (!impl_ || !impl_->rx_stream || max_samples == 0 || !dest) {
        eof_ = true;
        meta.is_eof = true;
        return 0;
    }

    double timeout = 0.5;
    size_t num_rx = impl_->rx_stream->recv(dest, max_samples, impl_->md, timeout, false);

    meta.error_code = static_cast<int>(impl_->md.error_code);
    meta.has_time_spec = impl_->md.has_time_spec;
    if (impl_->md.has_time_spec) {
        meta.time_spec_s = impl_->md.time_spec.get_real_secs();
    }
    meta.is_gap = (impl_->md.error_code == uhd::rx_metadata_t::ERROR_CODE_OVERFLOW ||
                   impl_->md.error_code == uhd::rx_metadata_t::ERROR_CODE_LATE_COMMAND);
    meta.gap_samples = 0;
    meta.start_sample_abs = impl_->current_sample_abs;
    meta.is_eof = eof_;

    impl_->current_sample_abs += num_rx;
    return num_rx;
}

double UhdDeviceSc16Source::actual_rx_rate() const
{
    return actual_rate_ > 0.0 ? actual_rate_ : cfg_.native_rate;
}

#else

struct UhdDeviceSc16Source::Impl {};

bool UhdDeviceSc16Source::is_supported()
{
    return false;
}

UhdDeviceSc16Source::UhdDeviceSc16Source(const PipelineConfig& cfg)
    : cfg_(cfg), eof_(true), actual_rate_(0.0)
{
}

UhdDeviceSc16Source::~UhdDeviceSc16Source() = default;

bool UhdDeviceSc16Source::open()
{
    return false;
}

bool UhdDeviceSc16Source::start_stream()
{
    // No UHD support compiled in: nothing can be started, but this is a passive
    // no-op rather than an error, matching the previous "open() already
    // failed" behaviour.
    return true;
}

void UhdDeviceSc16Source::close()
{
}

size_t UhdDeviceSc16Source::read_samples(std::complex<int16_t>*,
                                         size_t,
                                         StreamReadMeta& meta)
{
    meta = StreamReadMeta{};
    meta.is_eof = true;
    return 0;
}

double UhdDeviceSc16Source::actual_rx_rate() const
{
    return 0.0;
}

#endif

} // namespace pipeline
} // namespace openuwb
