/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * UwbCirWriter implementation.  Handler: validate + bounded enqueue only.
 * Worker: ordered file I/O, drains on stop.  No allocation in the handler.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gnuradio/uwb/uwb_cir_writer.h>
#include <gnuradio/io_signature.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <sstream>
#include <stdexcept>

namespace gr {
namespace uwb {

namespace {

constexpr double kSpeedOfLight = 299792458.0;

uint64_t
dict_u64(const pmt::pmt_t& dict, const char* key, uint64_t fallback)
{
    const pmt::pmt_t value =
        pmt::dict_ref(dict, pmt::mp(key), pmt::from_uint64(fallback));
    if (pmt::is_uint64(value))
        return pmt::to_uint64(value);
    if (pmt::is_integer(value))
        return static_cast<uint64_t>(pmt::to_long(value));
    return fallback;
}

int64_t
dict_i64(const pmt::pmt_t& dict, const char* key, int64_t fallback)
{
    const pmt::pmt_t value =
        pmt::dict_ref(dict, pmt::mp(key), pmt::from_long(fallback));
    if (pmt::is_uint64(value))
        return static_cast<int64_t>(pmt::to_uint64(value));
    if (pmt::is_integer(value))
        return pmt::to_long(value);
    return fallback;
}

double
dict_f64(const pmt::pmt_t& dict, const char* key, double fallback)
{
    const pmt::pmt_t value = pmt::dict_ref(dict, pmt::mp(key), pmt::PMT_NIL);
    if (pmt::is_real(value) || pmt::is_integer(value) ||
        pmt::is_uint64(value))
        return pmt::to_double(value);
    return fallback;
}

bool
dict_has(const pmt::pmt_t& dict, const char* key)
{
    return pmt::is_dict(dict) && pmt::dict_has_key(dict, pmt::mp(key));
}

std::string
dict_str(const pmt::pmt_t& dict, const char* key)
{
    const pmt::pmt_t value = pmt::dict_ref(dict, pmt::mp(key), pmt::PMT_NIL);
    if (pmt::is_symbol(value))
        return pmt::symbol_to_string(value);
    return {};
}

// JSON escaping for string values (status / ids are internal, but stay
// defensive about quotes and backslashes).
std::string
json_escape(const std::string& s)
{
    std::string out;
    out.reserve(s.size());
    for (char c : s) {
        if (c == '"' || c == '\\') {
            out.push_back('\\');
            out.push_back(c);
        } else if (static_cast<unsigned char>(c) < 0x20) {
            char buf[8];
            std::snprintf(buf, sizeof(buf), "\\u%04x", c);
            out += buf;
        } else {
            out.push_back(c);
        }
    }
    return out;
}

void
append_f64(std::ostringstream& os, const char* key, double v)
{
    char buf[64];
    if (std::isfinite(v))
        std::snprintf(buf, sizeof(buf), "%.9g", v);
    else
        std::snprintf(buf, sizeof(buf), "null");
    os << ",\"" << key << "\":" << buf;
}

void
append_u64(std::ostringstream& os, const char* key, uint64_t v)
{
    os << ",\"" << key << "\":" << v;
}

void
append_i64(std::ostringstream& os, const char* key, int64_t v)
{
    os << ",\"" << key << "\":" << v;
}

void
append_bool(std::ostringstream& os, const char* key, bool v)
{
    os << ",\"" << key << "\":" << (v ? "true" : "false");
}

void
append_str(std::ostringstream& os, const char* key, const std::string& v)
{
    os << ",\"" << key << "\":\"" << json_escape(v) << "\"";
}

void
append_opt_f64(std::ostringstream& os,
               const pmt::pmt_t& meta,
               const char* key)
{
    if (!dict_has(meta, key))
        return;
    append_f64(os, key, dict_f64(meta, key, 0.0));
}

void
append_opt_i64(std::ostringstream& os,
               const pmt::pmt_t& meta,
               const char* key)
{
    if (!dict_has(meta, key))
        return;
    append_i64(os, key, dict_i64(meta, key, 0));
}

void
append_opt_u64(std::ostringstream& os,
               const pmt::pmt_t& meta,
               const char* key)
{
    if (!dict_has(meta, key))
        return;
    append_u64(os, key, dict_u64(meta, key, 0));
}

void
append_opt_str(std::ostringstream& os,
               const pmt::pmt_t& meta,
               const char* key)
{
    if (!dict_has(meta, key))
        return;
    append_str(os, key, dict_str(meta, key));
}

} // namespace

UwbCirWriter::UwbCirWriter(const std::string& directory,
                           const std::string& base_name,
                           bool write_normalized,
                           size_t queue_capacity)
    : gr::block("uwb_cir_writer",
                gr::io_signature::make(0, 0, 0),
                gr::io_signature::make(0, 0, 0)),
      d_directory_(directory),
      d_base_name_(base_name),
      d_write_normalized_(write_normalized)
{
    if (directory.empty()) {
        throw std::invalid_argument("UwbCirWriter: directory is empty");
    }
    if (base_name.empty()) {
        throw std::invalid_argument("UwbCirWriter: base_name is empty");
    }
    if (queue_capacity == 0) {
        throw std::invalid_argument("UwbCirWriter: queue_capacity must be > 0");
    }
    d_queue_.assign(queue_capacity, pmt::PMT_NIL);

    message_port_register_in(pmt::mp("cir"));
    message_port_register_out(pmt::mp("status"));
    set_msg_handler(pmt::mp("cir"),
                    [this](pmt::pmt_t msg) { handle_cir(msg); });
}

UwbCirWriter::~UwbCirWriter()
{
    {
        std::lock_guard<std::mutex> lock(d_mutex_);
        d_stop_ = true;
    }
    d_cv_.notify_all();
    if (d_thread_.joinable())
        d_thread_.join();
}

std::shared_ptr<UwbCirWriter>
UwbCirWriter::make(const std::string& directory,
                   const std::string& base_name,
                   bool write_normalized,
                   size_t queue_capacity)
{
    return gnuradio::get_initial_sptr(new UwbCirWriter(
        directory, base_name, write_normalized, queue_capacity));
}

uint64_t UwbCirWriter::frames_received() const
{
    return d_received_.load();
}
uint64_t UwbCirWriter::frames_written() const
{
    return d_frames_written_.load();
}
uint64_t UwbCirWriter::frames_failed() const
{
    return d_frames_failed_.load();
}
uint64_t UwbCirWriter::frames_dropped() const
{
    return d_dropped_.load();
}
uint64_t UwbCirWriter::frames_invalid() const
{
    return d_invalid_.load();
}
uint64_t UwbCirWriter::taps_written() const
{
    return d_taps_.load();
}
size_t UwbCirWriter::queue_high_watermark() const
{
    return d_high_watermark_.load();
}

void
UwbCirWriter::write_run_json()
{
    std::ostringstream os;
    os << "{";
    append_str(os, "block", "UwbCirWriter");
    append_str(os, "directory", d_directory_);
    append_str(os, "base_name", d_base_name_);
    append_bool(os, "write_normalized", d_write_normalized_);
    append_str(os, "tap_format", "complex64");
    append_str(os, "byte_order", "little-endian");
    append_u64(os, "bytes_per_tap", 8);
    append_f64(os, "range_m_per_tap", kSpeedOfLight / (2.0 * 998.4e6));
    os << "}\n";
    const std::string path = d_directory_ + "/run.json";
    std::ofstream f(path, std::ios::trunc);
    if (f)
        f << os.str();
}

bool
UwbCirWriter::start()
{
    // Idempotent: the flowgraph may call start() again after a manual
    // start(); join any existing writer thread first (never assign a
    // joinable thread).
    {
        std::lock_guard<std::mutex> lock(d_mutex_);
        d_stop_ = true;
    }
    d_cv_.notify_all();
    if (d_thread_.joinable())
        d_thread_.join();

    std::error_code ec;
    std::filesystem::create_directories(d_directory_, ec);
    if (ec)
        return false;
    // Restart / double start: close previously opened streams first —
    // open() on an already-open fstream is a no-op that sets failbit.
    if (d_raw_.is_open())
        d_raw_.close();
    if (d_norm_.is_open())
        d_norm_.close();
    if (d_jsonl_.is_open())
        d_jsonl_.close();
    d_raw_.clear();
    d_norm_.clear();
    d_jsonl_.clear();
    const std::string pre = d_directory_ + "/" + d_base_name_;
    d_raw_.open(pre + ".cf32", std::ios::binary | std::ios::trunc);
    d_jsonl_.open(pre + ".jsonl", std::ios::trunc);
    if (d_write_normalized_)
        d_norm_.open(pre + "_norm.cf32", std::ios::binary | std::ios::trunc);
    d_raw_offset_.store(0);
    d_norm_offset_.store(0);
    d_frames_written_.store(0);
    d_frames_failed_.store(0);
    d_taps_.store(0);
    d_received_.store(0);
    d_dropped_.store(0);
    d_invalid_.store(0);
    d_high_watermark_.store(0);
    {
        std::lock_guard<std::mutex> lock(d_mutex_);
        d_queue_head_ = d_queue_tail_ = d_queue_count_ = 0;
        d_stop_ = false;
    }
    const bool ok = d_raw_.is_open() && d_jsonl_.is_open() &&
                    (d_norm_.is_open() || !d_write_normalized_);
    if (!ok)
        return false;
    write_run_json();
    d_thread_ = std::thread(&UwbCirWriter::writer_loop, this);
    return true;
}

bool
UwbCirWriter::stop()
{
    {
        std::lock_guard<std::mutex> lock(d_mutex_);
        d_stop_ = true;
    }
    d_cv_.notify_all();
    if (d_thread_.joinable())
        d_thread_.join();
    d_raw_.flush();
    if (d_norm_.is_open())
        d_norm_.flush();
    d_jsonl_.flush();
    d_raw_.close();
    if (d_norm_.is_open())
        d_norm_.close();
    d_jsonl_.close();
    return true;
}

void
UwbCirWriter::handle_cir(pmt::pmt_t msg)
{
    // Validate shape only; I/O happens on the writer thread.
    if (!pmt::is_pair(msg)) {
        d_invalid_.fetch_add(1);
        return;
    }
    const pmt::pmt_t meta = pmt::car(msg);
    const pmt::pmt_t data = pmt::cdr(msg);
    if (!pmt::is_dict(meta) || !pmt::is_c32vector(data)) {
        d_invalid_.fetch_add(1);
        return;
    }

    d_received_.fetch_add(1);
    {
        std::lock_guard<std::mutex> lock(d_mutex_);
        if (d_queue_count_ == d_queue_.size()) {
            d_dropped_.fetch_add(1);
            return;
        }
        d_queue_[d_queue_tail_] = msg;
        d_queue_tail_ = (d_queue_tail_ + 1) % d_queue_.size();
        ++d_queue_count_;
        size_t old = d_high_watermark_.load();
        while (old < d_queue_count_ &&
               !d_high_watermark_.compare_exchange_weak(old, d_queue_count_)) {
        }
    }
    d_cv_.notify_one();
}

void
UwbCirWriter::writer_loop()
{
    for (;;) {
        pmt::pmt_t msg;
        {
            std::unique_lock<std::mutex> lock(d_mutex_);
            d_cv_.wait(lock, [this] {
                return d_stop_ || d_queue_count_ != 0;
            });
            if (d_queue_count_ == 0 && d_stop_)
                return;
            msg = d_queue_[d_queue_head_];
            d_queue_[d_queue_head_] = pmt::PMT_NIL;
            d_queue_head_ = (d_queue_head_ + 1) % d_queue_.size();
            --d_queue_count_;
        }
        write_frame(msg);
    }
}

void
UwbCirWriter::write_frame(pmt::pmt_t msg)
{
    const pmt::pmt_t meta = pmt::car(msg);
    const pmt::pmt_t data = pmt::cdr(msg);

    const uint64_t pulse_id =
        dict_u64(meta, "pulse_id", d_frames_written_.load() +
                                        d_frames_failed_.load());
    const uint64_t schedule_index = dict_u64(meta, "schedule_index", pulse_id);
    const std::string status = dict_str(meta, "status");
    const uint64_t tap_count_in = dict_u64(meta, "tap_count", 0);

    size_t raw_len = 0;
    const gr_complex* raw = pmt::c32vector_elements(data, raw_len);
    size_t norm_len = 0;
    const gr_complex* norm = nullptr;
    pmt::pmt_t norm_v = pmt::PMT_NIL;
    if (d_write_normalized_) {
        norm_v = pmt::dict_ref(meta, pmt::mp("normalized_taps"),
                               pmt::PMT_NIL);
        if (pmt::is_c32vector(norm_v))
            norm = pmt::c32vector_elements(norm_v, norm_len);
    }

    // A frame is only "ok" (taps written, offset advanced) when the tap
    // count matches the payload length *exactly* — and, when enabled, the
    // normalized taps.  Shorter or longer payloads are treated as failed
    // frames (no binary output, no offset advance) so upstream
    // metadata/payload inconsistencies stay observable in cir.jsonl.
    const bool ok = status == "ok" && tap_count_in > 0 &&
                    raw != nullptr && raw_len == tap_count_in &&
                    (!d_write_normalized_ ||
                     (norm != nullptr && norm_len == tap_count_in));
    const uint64_t tap_count = ok ? tap_count_in : 0;

    uint64_t raw_offset = d_raw_offset_.load();
    uint64_t norm_offset = d_norm_offset_.load();
    if (ok) {
        const std::streamsize nbytes = static_cast<std::streamsize>(
            tap_count * sizeof(gr_complex));
        d_raw_.write(reinterpret_cast<const char*>(raw), nbytes);
        if (d_write_normalized_ && norm != nullptr) {
            d_norm_.write(reinterpret_cast<const char*>(norm), nbytes);
        }
        d_raw_.flush();
        if (d_norm_.is_open())
            d_norm_.flush();
        raw_offset = d_raw_offset_.fetch_add(tap_count);
        if (d_write_normalized_)
            norm_offset = d_norm_offset_.fetch_add(tap_count);
        d_taps_.fetch_add(tap_count);
        d_frames_written_.fetch_add(1);
    } else {
        d_frames_failed_.fetch_add(1);
    }

    // JSONL line: one per received frame, ok or failed.
    const double fs = dict_f64(meta, "sample_rate", 998.4e6);
    const double range_per_tap = kSpeedOfLight / (2.0 * fs);
    const int64_t zero_tap = dict_i64(meta, "zero_delay_tap", 0);
    const uint64_t peak_tap = ok ? dict_u64(meta, "peak_tap", 0) : 0;
    double peak_delay_ns = 0.0;
    if (ok && fs > 0.0) {
        const double taps = static_cast<double>(peak_tap) -
                            static_cast<double>(zero_tap);
        peak_delay_ns = taps / fs * 1.0e9;
    }

    std::ostringstream os;
    os << "{\"pulse_id\":" << pulse_id;
    append_u64(os, "schedule_index", schedule_index);
    append_str(os, "status",
               status.empty() ? std::string("unknown") : status);
    append_f64(os, "sample_rate", fs);
    append_u64(os, "tap_count", tap_count);
    append_opt_u64(os, meta, "cir_pre_samples");
    append_opt_u64(os, meta, "cir_post_samples");
    append_u64(os, "file_offset_taps", raw_offset);
    append_i64(os, "zero_delay_tap", zero_tap);
    append_u64(os, "peak_tap", peak_tap);
    append_f64(os, "cir_peak_metric",
               ok ? dict_f64(meta, "cir_peak_metric", 0.0) : 0.0);
    append_f64(os, "peak_delay_from_calibration_ns", peak_delay_ns);
    append_f64(os, "range_m_per_tap", range_per_tap);
    append_f64(os, "raw_l2_norm",
               ok ? dict_f64(meta, "raw_l2_norm", 0.0) : 0.0);
    append_opt_i64(os, meta, "preamble_start_sample");
    append_opt_i64(os, meta, "sfd_start_sample");
    append_opt_i64(os, meta, "cir_origin_sample");
    append_opt_i64(os, meta, "predicted_sfd_start_sample");
    append_opt_i64(os, meta, "tx_time_full");
    append_opt_f64(os, meta, "tx_time_frac");
    append_opt_i64(os, meta, "rx_time_full");
    append_opt_f64(os, meta, "rx_time_frac");
    append_opt_i64(os, meta, "num_delay_samps");
    append_opt_f64(os, meta, "calibration_delay_work_samples");
    append_opt_str(os, meta, "calibration_id");
    if (d_write_normalized_)
        append_u64(os, "file_offset_norm_taps", norm_offset);
    append_bool(os, "sfd_ok", dict_i64(meta, "sfd_start_sample", -1) >= 0);
    append_bool(os, "timing_ok",
                dict_i64(meta, "preamble_start_sample", -1) >= 0);
    append_opt_str(os, meta, "source");
    os << "}\n";

    d_jsonl_ << os.str();
    d_jsonl_.flush();
}

} // namespace uwb
} // namespace gr
