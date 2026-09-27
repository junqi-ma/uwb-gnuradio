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
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <thread>

namespace gr {
namespace uwb {

namespace {

constexpr double kSpeedOfLight = 299792458.0;

// QA-only slow-disk injection.  When UWB_CIR_WRITER_TEST_WRITE_DELAY_US is
// set in the environment at block construction, the writer thread sleeps
// that many microseconds before writing each dequeued message; production
// never sets it, so the steady-state hot path pays one relaxed atomic load.
uint32_t test_write_delay_us_from_env()
{
    const char* s = std::getenv("UWB_CIR_WRITER_TEST_WRITE_DELAY_US");
    if (s == nullptr || *s == '\0')
        return 0;
    char* end = nullptr;
    const long v = std::strtol(s, &end, 10);
    if (end == s || v < 0 || v > 600000000L)
        return 0;
    return static_cast<uint32_t>(v);
}

// QA-only burst-stall injection: UWB_CIR_WRITER_TEST_STALL_MS sleeps this
// long once before the dequeued message selected by
// UWB_CIR_WRITER_TEST_STALL_AT_MSG (default 64), emulating a multi-second
// disk stall somewhere in the middle of a run instead of a per-record
// slowdown.
uint32_t test_stall_ms_from_env()
{
    const char* s = std::getenv("UWB_CIR_WRITER_TEST_STALL_MS");
    if (s == nullptr || *s == '\0')
        return 0;
    char* end = nullptr;
    const long v = std::strtol(s, &end, 10);
    if (end == s || v < 0 || v > 60000000L)
        return 0;
    return static_cast<uint32_t>(v);
}

uint64_t test_stall_at_msg_from_env()
{
    const char* s = std::getenv("UWB_CIR_WRITER_TEST_STALL_AT_MSG");
    if (s == nullptr || *s == '\0')
        return 64;
    char* end = nullptr;
    const long v = std::strtol(s, &end, 10);
    if (end == s || v < 0 || v > 0x7FFFFFFFL)
        return 64;
    return static_cast<uint64_t>(v);
}

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

bool
dict_bool(const pmt::pmt_t& dict, const char* key, bool fallback = false)
{
    const pmt::pmt_t value = pmt::dict_ref(dict, pmt::mp(key), pmt::PMT_NIL);
    if (pmt::eq(value, pmt::PMT_T))
        return true;
    if (pmt::eq(value, pmt::PMT_F))
        return false;
    return fallback;
}

const char*
cir_status_name(uint64_t code)
{
    switch (code) {
    case 0:
        return "ok";
    case 1:
        return "sfd_failed";
    case 2:
        return "timing_failed";
    case 3:
        return "cir_failed";
    case 4:
        return "invalid_input";
    default:
        return "internal_error";
    }
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

void
freq_from_meta(const pmt::pmt_t& meta, double& freq_hz, double& freq_off)
{
    freq_hz = std::numeric_limits<double>::quiet_NaN();
    freq_off = std::numeric_limits<double>::quiet_NaN();
    // Non-jamming sweeps carry the tuned sense frequency under the generic
    // names.  Jam captures retain their historical UCR4 semantics by letting
    // the jammer's actual carrier override those fallback values.
    if (dict_has(meta, "freq_hz"))
        freq_hz = dict_f64(meta, "freq_hz", freq_hz);
    if (dict_has(meta, "freq_offset_hz"))
        freq_off = dict_f64(meta, "freq_offset_hz", freq_off);
    if (dict_has(meta, "jam_freq_actual_hz"))
        freq_hz = dict_f64(meta, "jam_freq_actual_hz", freq_hz);
    if (dict_has(meta, "jam_freq_offset_hz"))
        freq_off = dict_f64(meta, "jam_freq_offset_hz", freq_off);
}

} // namespace

UwbCirWriter::UwbCirWriter(const std::string& directory,
                           const std::string& base_name,
                           bool write_normalized,
                           size_t queue_capacity,
                           size_t aggregate_bytes,
                           uint32_t aggregate_window_ms)
    : gr::block("uwb_cir_writer",
                gr::io_signature::make(0, 0, 0),
                gr::io_signature::make(0, 0, 0)),
      d_directory_(directory),
      d_base_name_(base_name),
      d_write_normalized_(write_normalized),
      d_test_write_delay_us_(test_write_delay_us_from_env()),
      d_aggregate_bytes_(aggregate_bytes),
      d_aggregate_window_ms_(aggregate_window_ms),
      d_test_stall_ms_(test_stall_ms_from_env()),
      d_test_stall_at_msg_(test_stall_at_msg_from_env())
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
    d_repetition_records_.reserve(256);

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
                   size_t queue_capacity,
                   size_t aggregate_bytes,
                   uint32_t aggregate_window_ms)
{
    return gnuradio::get_initial_sptr(new UwbCirWriter(
        directory, base_name, write_normalized, queue_capacity,
        aggregate_bytes, aggregate_window_ms));
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
uint64_t UwbCirWriter::aggregate_flushes() const
{
    return d_aggregate_flushes_.load();
}
uint64_t UwbCirWriter::aggregate_max_bytes() const
{
    return d_aggregate_max_bytes_.load();
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
    append_str(os, "tap_format", "ucr4");
    append_str(os, "byte_order", "little-endian");
    append_u64(os, "header_bytes", sizeof(Ucr4Header));
    append_u64(os, "bytes_per_tap", 4);
    append_str(os, "reconstruction", "fc32 = sc16 * cir_scale");
    append_str(os, "json_layout", "one-line-per-pulse-columnar-repetitions");
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
    if (d_jsonl_.is_open())
        d_jsonl_.close();
    d_raw_.clear();
    d_jsonl_.clear();
    const std::string pre = d_directory_ + "/" + d_base_name_;
    d_raw_.open(pre + ".ucr4", std::ios::binary | std::ios::trunc);
    d_jsonl_.open(pre + ".jsonl", std::ios::trunc);
    d_raw_offset_.store(0);
    d_frames_written_.store(0);
    d_frames_failed_.store(0);
    d_taps_.store(0);
    d_received_.store(0);
    d_dropped_.store(0);
    d_invalid_.store(0);
    d_high_watermark_.store(0);
    d_since_flush_ = 0;
    d_repetition_group_active_ = false;
    d_repetition_common_meta_ = pmt::PMT_NIL;
    d_repetition_records_.clear();
    // Aggregate buffers: preallocate once behind start() (never in the hot
    // path).  Capacity is sticky, so repeated restarts do not reallocate.
    if (d_aggregate_bytes_ > 0) {
        if (d_ucr4_buf_.capacity() < d_aggregate_bytes_)
            d_ucr4_buf_.reserve(d_aggregate_bytes_ + sizeof(Ucr4Header) + 8);
        if (d_jsonl_buf_.capacity() < d_aggregate_bytes_)
            d_jsonl_buf_.reserve(d_aggregate_bytes_ + 4096);
    }
    d_ucr4_buf_.clear();
    d_jsonl_buf_.clear();
    {
        std::lock_guard<std::mutex> lock(d_mutex_);
        d_queue_head_ = d_queue_tail_ = d_queue_count_ = 0;
        d_stop_ = false;
    }
    const bool ok = d_raw_.is_open() && d_jsonl_.is_open();
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
    drain_aggregate_buffers();
    d_raw_.flush();
    d_jsonl_.flush();
    d_raw_.close();
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

    const uint64_t logical_frames =
        dict_bool(meta, "repetition_batch")
            ? std::max<uint64_t>(1, dict_u64(meta, "repetition_count", 1))
            : 1;
    d_received_.fetch_add(logical_frames);
    {
        std::lock_guard<std::mutex> lock(d_mutex_);
        if (d_queue_count_ == d_queue_.size()) {
            d_dropped_.fetch_add(logical_frames);
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
    bool stalled = false;
    uint64_t dequeued = 0;
    if (d_aggregate_bytes_ > 0) {
        const auto now = std::chrono::steady_clock::now();
        d_last_ucr4_sweep_ = now;
        d_last_jsonl_sweep_ = now;
    }
    for (;;) {
        pmt::pmt_t msg;
        bool stop_draining = false;
        {
            std::unique_lock<std::mutex> lock(d_mutex_);
            d_cv_.wait(lock, [this] {
                return d_stop_ || d_queue_count_ != 0;
            });
            if (d_queue_count_ == 0 && d_stop_) {
                flush_repetition_group();
                stop_draining = true;
            }
            else {
                msg = d_queue_[d_queue_head_];
                d_queue_[d_queue_head_] = pmt::PMT_NIL;
                d_queue_head_ = (d_queue_head_ + 1) % d_queue_.size();
                --d_queue_count_;
            }
        }
        if (stop_draining) {
            // Final aggregate egress: same stop() contract as before —
            // the buffered bytes hit the files before the streams are
            // flushed below.
            maybe_sweep_age_window(true);
            return;
        }
        // QA-only whole-writer stall (one shot): emulates a multi-second
        // disk stall between two dequeues.  Outside the lock so the
        // handler keeps enqueueing while the writer is frozen.
        if (!stalled) {
            const uint32_t stall_ms =
                d_test_stall_ms_.load(std::memory_order_relaxed);
            if (stall_ms != 0 && ++dequeued ==
                    d_test_stall_at_msg_.load(std::memory_order_relaxed)) {
                stalled = true;
                std::this_thread::sleep_for(
                    std::chrono::milliseconds(stall_ms));
            }
        }
        // QA-only slow-disk hook: sleep outside the lock so the handler
        // still enqueues while the fake slow write is in progress.
        if (const uint32_t delay_us =
                d_test_write_delay_us_.load(std::memory_order_relaxed)) {
            std::this_thread::sleep_for(std::chrono::microseconds(delay_us));
        }
        // Age-window egress check when aggregation is on; off-path it is
        // one relaxed atomic load, no clock read and no syscall.
        if (d_aggregate_bytes_ > 0)
            maybe_sweep_age_window(false);
        write_frame(msg);
    }
}

void
UwbCirWriter::aggregate_ucr4(const char* bytes, size_t len)
{
    // Legacy per-record path (aggregate_bytes == 0): unchanged bytes,
    // unchanged syscall granularity.
    if (d_aggregate_bytes_ == 0) {
        d_raw_.write(bytes, static_cast<std::streamsize>(len));
        if (++d_since_flush_ >= 256) {
            d_raw_.flush();
            d_jsonl_.flush();
            d_since_flush_ = 0;
        }
        return;
    }
    // If the record does not fit and the buffer is non-empty, emit the
    // accumulated block first, then append: records stay whole, order is
    // preserved, and the buffer never exceeds threshold + one record.
    if (!d_ucr4_buf_.empty() &&
        d_ucr4_buf_.size() + len > d_aggregate_bytes_)
        flush_aggregate_ucr4();
    d_ucr4_buf_.insert(d_ucr4_buf_.end(), bytes, bytes + len);
}

void
UwbCirWriter::aggregate_jsonl(const char* bytes, size_t len)
{
    if (d_aggregate_bytes_ == 0) {
        d_jsonl_ << bytes;
        d_jsonl_.flush();
        return;
    }
    if (!d_jsonl_buf_.empty() &&
        d_jsonl_buf_.size() + len > d_aggregate_bytes_)
        flush_aggregate_jsonl();
    d_jsonl_buf_.insert(d_jsonl_buf_.end(), bytes, bytes + len);
}

void
UwbCirWriter::flush_aggregate_ucr4()
{
    // Writer-thread only: stop() joins the writer thread before calling
    // the drain, so no other party touches d_ucr4_buf_ here.
    if (d_ucr4_buf_.empty())
        return;
    d_raw_.write(d_ucr4_buf_.data(),
                 static_cast<std::streamsize>(d_ucr4_buf_.size()));
    const uint64_t bytes = d_ucr4_buf_.size();
    d_ucr4_buf_.clear();
    d_last_ucr4_sweep_ = std::chrono::steady_clock::now();
    d_aggregate_flushes_.fetch_add(1, std::memory_order_relaxed);
    uint64_t prev = d_aggregate_max_bytes_.load(std::memory_order_relaxed);
    while (prev < bytes &&
           !d_aggregate_max_bytes_.compare_exchange_weak(
               prev, bytes, std::memory_order_relaxed)) {
    }
}

void
UwbCirWriter::flush_aggregate_jsonl()
{
    if (d_jsonl_buf_.empty())
        return;
    d_jsonl_.write(d_jsonl_buf_.data(),
                   static_cast<std::streamsize>(d_jsonl_buf_.size()));
    d_jsonl_buf_.clear();
    d_last_jsonl_sweep_ = std::chrono::steady_clock::now();
}

void
UwbCirWriter::drain_aggregate_buffers()
{
    flush_aggregate_ucr4();
    flush_aggregate_jsonl();
}

void
UwbCirWriter::maybe_sweep_age_window(bool force)
{
    if (d_aggregate_bytes_ == 0)
        return;
    const auto now = std::chrono::steady_clock::now();
    const auto window = std::chrono::milliseconds(d_aggregate_window_ms_);
    if (force || now - d_last_ucr4_sweep_ >= window)
        flush_aggregate_ucr4();
    if (force || now - d_last_jsonl_sweep_ >= window)
        flush_aggregate_jsonl();
}

void
UwbCirWriter::write_ucr4_record(uint32_t pulse_id,
                                uint16_t status,
                                uint16_t tap_count,
                                uint16_t repetition_index,
                                uint16_t repetition_count,
                                float sfd_metric,
                                float peak_metric,
                                int32_t peak_tap,
                                uint32_t estimator_us,
                                double freq_hz,
                                double freq_offset_hz,
                                const gr_complex* taps)
{
    if (d_sc16_scratch_.size() < static_cast<size_t>(tap_count) * 2)
        d_sc16_scratch_.resize(static_cast<size_t>(tap_count) * 2);
    float scale = 0.0f;
    if (taps != nullptr && tap_count > 0)
        scale = encode_cir_sc16(taps, tap_count, d_sc16_scratch_.data());

    Ucr4Header hdr{};
    hdr.magic[0] = 'U';
    hdr.magic[1] = 'C';
    hdr.magic[2] = 'R';
    hdr.magic[3] = '4';
    hdr.pulse_id = pulse_id;
    hdr.status = status;
    hdr.tap_count = tap_count;
    hdr.repetition_index = repetition_index;
    hdr.repetition_count = repetition_count;
    hdr.sfd_metric = sfd_metric;
    hdr.cir_peak_metric = peak_metric;
    hdr.peak_tap = peak_tap;
    hdr.estimator_us = estimator_us;
    hdr.freq_hz = freq_hz;
    hdr.freq_offset_hz = freq_offset_hz;
    hdr.cir_scale = scale;
    aggregate_ucr4(reinterpret_cast<const char*>(&hdr), sizeof(hdr));
    if (tap_count > 0) {
        aggregate_ucr4(reinterpret_cast<const char*>(d_sc16_scratch_.data()),
                       tap_count * 4);
    }
}

void
UwbCirWriter::write_frame(pmt::pmt_t msg)
{
    const pmt::pmt_t meta = pmt::car(msg);
    const pmt::pmt_t data = pmt::cdr(msg);

    if (dict_bool(meta, "repetition_batch")) {
        write_repetition_batch(meta, data);
        return;
    }

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
    uint64_t norm_offset = raw_offset;
    if (ok) {
        double freq_hz = 0.0, freq_off = 0.0;
        freq_from_meta(meta, freq_hz, freq_off);
        const uint16_t rep_index = dict_has(meta, "repetition_index")
            ? static_cast<uint16_t>(
                  dict_u64(meta, "repetition_index", 0) & 0xFFFFu)
            : static_cast<uint16_t>(0xFFFF);
        const uint16_t rep_count = static_cast<uint16_t>(
            dict_u64(meta, "repetition_count", 0) & 0xFFFFu);
        write_ucr4_record(
            static_cast<uint32_t>(pulse_id & 0xFFFFFFFFu),
            0,
            static_cast<uint16_t>(tap_count),
            rep_index,
            rep_count,
            static_cast<float>(dict_f64(meta, "sfd_metric", 0.0)),
            static_cast<float>(dict_f64(meta, "cir_peak_metric", 0.0)),
            static_cast<int32_t>(dict_u64(meta, "peak_tap", 0)),
            static_cast<uint32_t>(dict_u64(meta, "estimator_us", 0)),
            freq_hz,
            freq_off,
            raw);
        raw_offset = d_raw_offset_.fetch_add(tap_count);
        norm_offset = raw_offset;
        d_taps_.fetch_add(tap_count);
        d_frames_written_.fetch_add(1);
    } else {
        d_frames_failed_.fetch_add(1);
    }

    // JSON metadata.  Average/legacy input stays one line per message.
    // Individual repetitions are grouped below into one line per pulse.
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

    const bool is_repetition =
        dict_has(meta, "repetition_index") &&
        dict_u64(meta, "repetition_count", 0) > 0;
    if (is_repetition) {
        append_repetition_record(meta, pulse_id, schedule_index, status, ok,
                                 tap_count, raw_offset, norm_offset, peak_tap,
                                 peak_delay_ns);
        flush_files_if_due();
        return;
    }

    // A non-repetition record closes any incomplete packet group before its
    // own legacy JSON line is emitted.
    flush_repetition_group();

    std::ostringstream os;
    os << "{\"pulse_id\":" << pulse_id;
    append_u64(os, "schedule_index", schedule_index);
    append_opt_str(os, meta, "cir_output");
    append_opt_u64(os, meta, "averaged_repetition_first");
    append_opt_u64(os, meta, "repetition_index");
    append_opt_u64(os, meta, "repetition_ordinal");
    append_opt_u64(os, meta, "repetition_count");
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
    append_opt_str(os, meta, "jam_delay_mode");
    append_opt_i64(os, meta, "jam_delay_native");
    append_opt_i64(os, meta, "jam_overlap_reps");
    append_opt_str(os, meta, "jam_overlap_side");
    append_opt_u64(os, meta, "sense_preamble_reps");
    append_opt_u64(os, meta, "jam_preamble_reps");
    append_opt_u64(os, meta, "jam_overlap_seed");
    append_bool(os, "sfd_ok", dict_i64(meta, "sfd_start_sample", -1) >= 0);
    append_bool(os, "timing_ok",
                dict_i64(meta, "preamble_start_sample", -1) >= 0);
    append_u64(os, "estimator_us", dict_u64(meta, "estimator_us", 0));
    append_opt_str(os, meta, "source");
    os << "}\n";

    const std::string line = os.str();
    aggregate_jsonl(line.data(), line.size());
    flush_files_if_due();
}

void
UwbCirWriter::write_repetition_batch(const pmt::pmt_t& meta,
                                     const pmt::pmt_t& data)
{
    const uint64_t pulse_id = dict_u64(meta, "pulse_id", 0);
    const uint64_t schedule_index =
        dict_u64(meta, "schedule_index", pulse_id);
    const size_t count =
        static_cast<size_t>(dict_u64(meta, "repetition_count", 0));
    const size_t first =
        static_cast<size_t>(dict_u64(meta, "repetition_first", 0));
    const size_t taps = static_cast<size_t>(dict_u64(meta, "tap_count", 0));
    if (count == 0 || taps == 0 ||
        count > std::numeric_limits<size_t>::max() / taps) {
        d_invalid_.fetch_add(1);
        d_frames_failed_.fetch_add(count);
        return;
    }
    const size_t total = count * taps;
    size_t raw_len = 0;
    const gr_complex* raw = pmt::c32vector_elements(data, raw_len);
    const pmt::pmt_t status_v = pmt::dict_ref(
        meta, pmt::mp("repetition_status_code"), pmt::PMT_NIL);
    const pmt::pmt_t peak_v = pmt::dict_ref(
        meta, pmt::mp("repetition_peak_tap"), pmt::PMT_NIL);
    const pmt::pmt_t metric_v = pmt::dict_ref(
        meta, pmt::mp("repetition_peak_metric"), pmt::PMT_NIL);
    const pmt::pmt_t norm_metric_v = pmt::dict_ref(
        meta, pmt::mp("repetition_raw_l2_norm"), pmt::PMT_NIL);
    size_t status_len = 0, peak_len = 0, metric_len = 0, norm_metric_len = 0;
    const uint64_t* statuses = pmt::is_u64vector(status_v)
                                   ? pmt::u64vector_elements(status_v, status_len)
                                   : nullptr;
    const uint64_t* peaks = pmt::is_u64vector(peak_v)
                                ? pmt::u64vector_elements(peak_v, peak_len)
                                : nullptr;
    const float* metrics = pmt::is_f32vector(metric_v)
                               ? pmt::f32vector_elements(metric_v, metric_len)
                               : nullptr;
    const float* norm_metrics =
        pmt::is_f32vector(norm_metric_v)
            ? pmt::f32vector_elements(norm_metric_v, norm_metric_len)
            : nullptr;
    pmt::pmt_t norm_v = pmt::PMT_NIL;
    size_t norm_len = 0;
    const gr_complex* norm = nullptr;
    if (d_write_normalized_) {
        norm_v = pmt::dict_ref(meta, pmt::mp("normalized_taps"),
                               pmt::PMT_NIL);
        if (pmt::is_c32vector(norm_v))
            norm = pmt::c32vector_elements(norm_v, norm_len);
    }
    if (!raw || raw_len != total || !statuses || status_len != count ||
        !peaks || peak_len != count || !metrics || metric_len != count ||
        !norm_metrics || norm_metric_len != count ||
        (d_write_normalized_ && (!norm || norm_len != total))) {
        d_invalid_.fetch_add(1);
        d_frames_failed_.fetch_add(count);
        return;
    }

    if (d_repetition_group_active_)
        flush_repetition_group();
    d_repetition_group_active_ = true;
    d_repetition_pulse_id_ = pulse_id;
    d_repetition_schedule_index_ = schedule_index;
    d_repetition_expected_ = count;
    d_repetition_common_meta_ = meta;
    d_repetition_records_.clear();

    const double fs = dict_f64(meta, "sample_rate", 998.4e6);
    const int64_t zero_tap = dict_i64(meta, "zero_delay_tap", 0);
    const uint64_t estimator_us = dict_u64(meta, "estimator_us", 0);
    double freq_hz = 0.0, freq_off = 0.0;
    freq_from_meta(meta, freq_hz, freq_off);
    const float sfd_m = static_cast<float>(dict_f64(meta, "sfd_metric", 0.0));
    const uint32_t pulse32 = static_cast<uint32_t>(pulse_id & 0xFFFFFFFFu);
    for (size_t i = 0; i < count; ++i) {
        const bool ok = statuses[i] == 0;
        uint64_t raw_offset = d_raw_offset_.load();
        uint64_t norm_offset = raw_offset;
        if (ok) {
            write_ucr4_record(
                pulse32,
                0,
                static_cast<uint16_t>(taps),
                static_cast<uint16_t>((first + i) & 0xFFFFu),
                static_cast<uint16_t>(count & 0xFFFFu),
                sfd_m,
                metrics[i],
                static_cast<int32_t>(peaks[i]),
                static_cast<uint32_t>(estimator_us),
                freq_hz,
                freq_off,
                raw + i * taps);
            raw_offset = d_raw_offset_.fetch_add(taps);
            norm_offset = raw_offset;
            d_taps_.fetch_add(taps);
            d_frames_written_.fetch_add(1);
        } else {
            d_frames_failed_.fetch_add(1);
        }

        RepetitionJsonRecord rec;
        rec.index = first + i;
        rec.status = cir_status_name(statuses[i]);
        rec.tap_count = ok ? taps : 0;
        rec.raw_offset = raw_offset;
        rec.norm_offset = norm_offset;
        rec.peak_tap = ok ? peaks[i] : 0;
        rec.peak_metric = ok ? metrics[i] : 0.0;
        rec.raw_l2_norm = ok ? norm_metrics[i] : 0.0;
        rec.estimator_us = estimator_us;
        if (ok && fs > 0.0) {
            rec.peak_delay_ns =
                (static_cast<double>(rec.peak_tap) -
                 static_cast<double>(zero_tap)) /
                fs * 1.0e9;
        }
        d_repetition_records_.push_back(std::move(rec));
    }
    flush_repetition_group();
    flush_files_if_due();
}

void
UwbCirWriter::append_repetition_record(const pmt::pmt_t& meta,
                                       uint64_t pulse_id,
                                       uint64_t schedule_index,
                                       const std::string& status,
                                       bool ok,
                                       uint64_t tap_count,
                                       uint64_t raw_offset,
                                       uint64_t norm_offset,
                                       uint64_t peak_tap,
                                       double peak_delay_ns)
{
    const uint64_t expected = dict_u64(meta, "repetition_count", 0);
    if (d_repetition_group_active_ &&
        pulse_id != d_repetition_pulse_id_) {
        flush_repetition_group();
    }
    if (!d_repetition_group_active_) {
        d_repetition_group_active_ = true;
        d_repetition_pulse_id_ = pulse_id;
        d_repetition_schedule_index_ = schedule_index;
        d_repetition_expected_ = expected;
        d_repetition_common_meta_ = meta;
        d_repetition_records_.clear();
    }

    RepetitionJsonRecord rec;
    rec.index = dict_u64(meta, "repetition_index", 0);
    rec.status = status.empty() ? std::string("unknown") : status;
    rec.tap_count = tap_count;
    rec.raw_offset = raw_offset;
    rec.norm_offset = norm_offset;
    rec.peak_tap = peak_tap;
    rec.peak_metric = ok ? dict_f64(meta, "cir_peak_metric", 0.0) : 0.0;
    rec.peak_delay_ns = peak_delay_ns;
    rec.raw_l2_norm = ok ? dict_f64(meta, "raw_l2_norm", 0.0) : 0.0;
    rec.estimator_us = dict_u64(meta, "estimator_us", 0);
    d_repetition_records_.push_back(std::move(rec));

    if (d_repetition_expected_ > 0 &&
        d_repetition_records_.size() >= d_repetition_expected_) {
        flush_repetition_group();
    }
}

void
UwbCirWriter::flush_repetition_group()
{
    if (!d_repetition_group_active_)
        return;

    const pmt::pmt_t& meta = d_repetition_common_meta_;
    const double fs = dict_f64(meta, "sample_rate", 998.4e6);
    const bool complete = d_repetition_expected_ > 0 &&
                          d_repetition_records_.size() ==
                              d_repetition_expected_;
    const bool all_ok = !d_repetition_records_.empty() &&
                        std::all_of(d_repetition_records_.begin(),
                                    d_repetition_records_.end(),
                                    [](const RepetitionJsonRecord& r) {
                                        return r.status == "ok";
                                    });

    std::ostringstream os;
    os << "{\"pulse_id\":" << d_repetition_pulse_id_;
    append_u64(os, "schedule_index", d_repetition_schedule_index_);
    append_str(os, "cir_output", "repetitions");
    append_str(os, "status", !complete ? "incomplete"
                                         : (all_ok ? "ok" : "partial_failure"));
    append_f64(os, "sample_rate", fs);
    append_opt_u64(os, meta, "cir_pre_samples");
    append_opt_u64(os, meta, "cir_post_samples");
    append_i64(os, "zero_delay_tap", dict_i64(meta, "zero_delay_tap", 0));
    append_f64(os, "range_m_per_tap", kSpeedOfLight / (2.0 * fs));
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
    append_opt_str(os, meta, "jam_delay_mode");
    append_opt_i64(os, meta, "jam_delay_native");
    append_opt_i64(os, meta, "jam_overlap_reps");
    append_opt_str(os, meta, "jam_overlap_side");
    append_opt_u64(os, meta, "sense_preamble_reps");
    append_opt_u64(os, meta, "jam_preamble_reps");
    append_opt_u64(os, meta, "jam_overlap_seed");
    append_bool(os, "sfd_ok", dict_i64(meta, "sfd_start_sample", -1) >= 0);
    append_bool(os, "timing_ok",
                dict_i64(meta, "preamble_start_sample", -1) >= 0);
    append_opt_str(os, meta, "source");
    append_u64(os, "repetition_count", d_repetition_expected_);
    append_u64(os, "repetition_records", d_repetition_records_.size());
    append_bool(os, "repetition_complete", complete);
    os << ",\"repetitions\":{";

    auto append_u64_column = [&os, this](const char* key, auto getter) {
        os << "\"" << key << "\":[";
        for (size_t i = 0; i < d_repetition_records_.size(); ++i) {
            if (i)
                os << ',';
            os << getter(d_repetition_records_[i]);
        }
        os << ']';
    };
    auto append_f64_column = [&os, this](const char* key, auto getter) {
        os << ",\"" << key << "\":[";
        char buf[64];
        for (size_t i = 0; i < d_repetition_records_.size(); ++i) {
            if (i)
                os << ',';
            const double value = getter(d_repetition_records_[i]);
            if (std::isfinite(value))
                std::snprintf(buf, sizeof(buf), "%.9g", value);
            else
                std::snprintf(buf, sizeof(buf), "null");
            os << buf;
        }
        os << ']';
    };

    append_u64_column("repetition_index",
                      [](const RepetitionJsonRecord& r) { return r.index; });
    os << ",\"status\":[";
    for (size_t i = 0; i < d_repetition_records_.size(); ++i) {
        if (i)
            os << ',';
        os << '\"' << json_escape(d_repetition_records_[i].status) << '\"';
    }
    os << ']';
    os << ',';
    append_u64_column("tap_count",
                      [](const RepetitionJsonRecord& r) { return r.tap_count; });
    os << ',';
    append_u64_column("file_offset_taps", [](const RepetitionJsonRecord& r) {
        return r.raw_offset;
    });
    os << ',';
    append_u64_column("peak_tap",
                      [](const RepetitionJsonRecord& r) { return r.peak_tap; });
    append_f64_column("cir_peak_metric", [](const RepetitionJsonRecord& r) {
        return r.peak_metric;
    });
    append_f64_column("peak_delay_from_calibration_ns",
                      [](const RepetitionJsonRecord& r) {
                          return r.peak_delay_ns;
                      });
    append_f64_column("raw_l2_norm", [](const RepetitionJsonRecord& r) {
        return r.raw_l2_norm;
    });
    os << ',';
    append_u64_column("estimator_us", [](const RepetitionJsonRecord& r) {
        return r.estimator_us;
    });
    os << "}}\n";
    {
        const std::string line = os.str();
        aggregate_jsonl(line.data(), line.size());
    }

    d_repetition_group_active_ = false;
    d_repetition_common_meta_ = pmt::PMT_NIL;
    d_repetition_records_.clear();
}

void
UwbCirWriter::flush_files_if_due()
{
    // Per-repetition output can exceed 20k records/s.  Flushing three files
    // after every record serialises the hot path on syscalls; bounded batch
    // flushing preserves observability while stop() still performs a final
    // drain + flush before close.
    if (++d_since_flush_ >= 256) {
        // With aggregation enabled the byte-threshold + age-window egress
        // bounds the exposure, so the legacy periodic flush is redundant.
        if (d_aggregate_bytes_ == 0) {
            d_raw_.flush();
            d_jsonl_.flush();
        }
        d_since_flush_ = 0;
    }
}

} // namespace uwb
} // namespace gr
