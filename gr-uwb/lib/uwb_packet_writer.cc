/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gnuradio/uwb/uwb_packet_writer.h>
#include <gnuradio/io_signature.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>

namespace gr {
namespace uwb {

namespace {
uint64_t dict_u64(const pmt::pmt_t& dict, const char* key, uint64_t fallback)
{
    const pmt::pmt_t value =
        pmt::dict_ref(dict, pmt::mp(key), pmt::from_uint64(fallback));
    if (pmt::is_uint64(value))
        return pmt::to_uint64(value);
    if (pmt::is_integer(value))
        return static_cast<uint64_t>(pmt::to_long(value));
    return fallback;
}
} // namespace

UwbPacketWriter::UwbPacketWriter(const std::string& directory,
                                 const std::string& base_name,
                                 bool one_file_per_packet)
    : gr::block("uwb_packet_writer",
                gr::io_signature::make(0, 0, 0),
                gr::io_signature::make(0, 0, 0)),
      d_directory_(directory),
      d_base_name_(base_name),
      d_one_file_per_packet_(one_file_per_packet)
{
    message_port_register_in(pmt::mp("packet"));
    set_msg_handler(pmt::mp("packet"),
                    [this](pmt::pmt_t msg) { handle_packet(msg); });
}

std::shared_ptr<UwbPacketWriter>
UwbPacketWriter::make(const std::string& directory,
                      const std::string& base_name,
                      bool one_file_per_packet)
{
    return gnuradio::get_initial_sptr(
        new UwbPacketWriter(directory, base_name, one_file_per_packet));
}

std::string UwbPacketWriter::directory() const { return d_directory_; }
std::string UwbPacketWriter::base_name() const { return d_base_name_; }
bool UwbPacketWriter::one_file_per_packet() const
{
    return d_one_file_per_packet_;
}
size_t UwbPacketWriter::packets_written() const { return d_packets_.load(); }
uint64_t UwbPacketWriter::samples_written() const { return d_sample_offset_.load(); }
uint64_t UwbPacketWriter::packets_received() const { return d_received_.load(); }
uint64_t UwbPacketWriter::packets_dropped() const { return d_dropped_.load(); }
size_t UwbPacketWriter::queue_high_watermark() const { return d_high_watermark_.load(); }

float
UwbPacketWriter::convert_to_sc16(const gr_complex* in,
                                 size_t n,
                                 std::vector<int16_t>& out)
{
    float peak = 0.0f;
    for (size_t i = 0; i < n; ++i) {
        peak = std::max(peak, std::abs(in[i].real()));
        peak = std::max(peak, std::abs(in[i].imag()));
    }
    const float scale = (peak > 0.0f) ? (32767.0f / peak) : 1.0f;

    out.resize(n * 2);
    for (size_t i = 0; i < n; ++i) {
        float re = std::round(in[i].real() * scale);
        float im = std::round(in[i].imag() * scale);
        re = std::max(-32768.0f, std::min(32767.0f, re));
        im = std::max(-32768.0f, std::min(32767.0f, im));
        out[2 * i] = static_cast<int16_t>(re);
        out[2 * i + 1] = static_cast<int16_t>(im);
    }
    return scale;
}

bool
UwbPacketWriter::start()
{
    std::error_code ec;
    std::filesystem::create_directories(d_directory_, ec);
    const std::string pre = d_directory_ + "/" + d_base_name_;
    if (!d_one_file_per_packet_)
        d_iq_.open(pre + ".iq", std::ios::binary | std::ios::trunc);
    d_jsonl_.open(pre + ".jsonl", std::ios::trunc);
    d_sample_offset_.store(0);
    d_packets_.store(0);
    d_received_.store(0);
    d_dropped_.store(0);
    d_high_watermark_.store(0);
    {
        std::lock_guard<std::mutex> lock(d_mutex_);
        d_queue_head_ = d_queue_tail_ = d_queue_count_ = 0;
        d_stop_ = false;
    }
    const bool ok = d_jsonl_.is_open() &&
                    (d_one_file_per_packet_ || d_iq_.is_open());
    if (ok)
        d_thread_ = std::thread(&UwbPacketWriter::writer_loop, this);
    return ok;
}

bool
UwbPacketWriter::stop()
{
    {
        std::lock_guard<std::mutex> lock(d_mutex_);
        d_stop_ = true;
    }
    d_cv_.notify_one();
    if (d_thread_.joinable())
        d_thread_.join();
    d_iq_.flush();
    d_jsonl_.flush();
    if (d_iq_.is_open())
        d_iq_.close();
    if (d_jsonl_.is_open())
        d_jsonl_.close();
    return true;
}

void
UwbPacketWriter::handle_packet(pmt::pmt_t msg)
{
    if (!pmt::is_pair(msg))
        return;
    const pmt::pmt_t meta = pmt::car(msg);
    const pmt::pmt_t data = pmt::cdr(msg);
    if (!pmt::is_dict(meta) || !pmt::is_c32vector(data))
        return;

    d_received_.fetch_add(1);
    {
        std::lock_guard<std::mutex> lock(d_mutex_);
        if (d_queue_count_ == kQueueCapacity) {
            d_dropped_.fetch_add(1);
            return;
        }
        d_queue_[d_queue_tail_] = msg;
        d_queue_tail_ = (d_queue_tail_ + 1) % kQueueCapacity;
        ++d_queue_count_;
        size_t old = d_high_watermark_.load();
        while (old < d_queue_count_ &&
               !d_high_watermark_.compare_exchange_weak(old, d_queue_count_)) {}
    }
    d_cv_.notify_one();
}

void UwbPacketWriter::writer_loop()
{
    for (;;) {
        pmt::pmt_t msg;
        {
            std::unique_lock<std::mutex> lock(d_mutex_);
            d_cv_.wait(lock, [this] { return d_stop_ || d_queue_count_ != 0; });
            if (d_queue_count_ == 0 && d_stop_)
                return;
            msg = d_queue_[d_queue_head_];
            d_queue_[d_queue_head_] = pmt::PMT_NIL;
            d_queue_head_ = (d_queue_head_ + 1) % kQueueCapacity;
            --d_queue_count_;
        }
        write_packet(msg);
    }
}

void UwbPacketWriter::write_packet(pmt::pmt_t msg)
{
    pmt::pmt_t meta = pmt::car(msg);
    pmt::pmt_t data = pmt::cdr(msg);

    const uint64_t id = dict_u64(meta, "packet_id", d_packets_.load());
    // Generic detector metadata uses start/trigger; scheduled extractor uses
    // window_start/predicted_start.  Preserve one Writer for both PDU sources.
    const uint64_t scheduled_start = dict_u64(meta, "window_start_sample", 0);
    const uint64_t start = dict_u64(meta, "start_sample", scheduled_start);
    const uint64_t predicted = dict_u64(meta, "predicted_start_sample", start);
    const uint64_t trigger = dict_u64(meta, "trigger_sample", predicted);
    const double rate = pmt::to_double(
        pmt::dict_ref(meta, pmt::mp("sample_rate"), pmt::from_double(0)));
    const double metric = pmt::to_double(
        pmt::dict_ref(meta, pmt::mp("detection_metric"), pmt::from_double(0)));
    const long pre = pmt::to_long(
        pmt::dict_ref(meta, pmt::mp("pre_trigger_samples"), pmt::from_long(0)));

    const std::vector<std::complex<float>>& elems =
        pmt::c32vector_elements(data);
    const size_t n = elems.size();
    const float iq_scale = convert_to_sc16(elems.data(), n, d_sc16_);
    const char* sc16_bytes = reinterpret_cast<const char*>(d_sc16_.data());
    const std::streamsize nbytes =
        static_cast<std::streamsize>(d_sc16_.size() * sizeof(int16_t));

    char line[640];
    if (d_one_file_per_packet_) {
        const std::string fname = "packet_" + std::to_string(id) + ".iq";
        std::ofstream f(d_directory_ + "/" + fname, std::ios::binary);
        if (f)
            f.write(sc16_bytes, nbytes);
        std::snprintf(
            line,
            sizeof(line),
            "{\"packet_id\":%llu,\"file\":\"%s\",\"start_sample\":%llu,"
            "\"trigger_sample\":%llu,\"sample_rate\":%.0f,"
            "\"sample_count\":%zu,\"file_offset_samples\":0,"
            "\"detection_metric\":%.6f,\"pre_trigger_samples\":%ld,"
            "\"sample_format\":\"sc16\",\"iq_scale\":%.9g}\n",
            static_cast<unsigned long long>(id),
            fname.c_str(),
            static_cast<unsigned long long>(start),
            static_cast<unsigned long long>(trigger),
            rate,
            n,
            metric,
            pre,
            static_cast<double>(iq_scale));
    } else {
        d_iq_.write(sc16_bytes, nbytes);
        std::snprintf(
            line,
            sizeof(line),
            "{\"packet_id\":%llu,\"start_sample\":%llu,"
            "\"trigger_sample\":%llu,\"sample_rate\":%.0f,"
            "\"sample_count\":%zu,\"file_offset_samples\":%llu,"
            "\"detection_metric\":%.6f,\"pre_trigger_samples\":%ld,"
            "\"sample_format\":\"sc16\",\"iq_scale\":%.9g}\n",
            static_cast<unsigned long long>(id),
            static_cast<unsigned long long>(start),
            static_cast<unsigned long long>(trigger),
            rate,
            n,
            static_cast<unsigned long long>(d_sample_offset_.load()),
            metric,
            pre,
            static_cast<double>(iq_scale));
        d_sample_offset_.fetch_add(n);
    }

    d_jsonl_ << line;
    d_jsonl_.flush();
    d_packets_.fetch_add(1);
}

} // namespace uwb
} // namespace gr
