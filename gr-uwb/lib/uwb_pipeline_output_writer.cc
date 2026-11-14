/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Pure C++ asynchronous output writer for UhdSc16Pipeline.
 *
 * Zero GNU Radio / PMT dependency: standard library plus the pure C++ UCR4
 * format header.  On-disk contract is identical to the GNU Radio
 * `UwbCirWriter` block (`<base>.ucr4` + `<base>.jsonl` + `run.json`) and to
 * `uwb_scheduled_dump_io.h` (`demod_results.jsonl`), but the implementation is
 * independent so the pipeline never links `gnuradio-runtime` for I/O.
 *
 * Threading
 * ---------
 *   * `try_acquire_slot()` / `release_slot()` / `try_submit()` are called by
 *     the DSP workers and by `commit_result()`; all three are O(1), never
 *     allocate, and never block on I/O.
 *   * ONE background thread (`writer_loop`) performs every write(), flush()
 *     and printf().  It is the only thread that touches the file streams, the
 *     aggregate buffers and stdout.
 *   * `stop_and_drain()` joins that thread after the queue is empty, then
 *     flushes and closes every file.
 *
 * Ledger (asserted by QA after stop_and_drain())
 * ----------------------------------------------
 *   offered == written + dropped
 *
 *   offered : every try_acquire_slot() call
 *   written : every slot the writer thread finished emitting
 *   dropped : pool exhaustion + queue rejection + release_slot()
 *
 * Back-pressure policy is drop, never block: a full pool or a full queue
 * increments `dropped` and returns immediately.
 *
 * Owner: Track 1 of
 * docs/performance/OpenCode_Pipeline落盘与实时日志_多Agent执行计划_2026-11-14.md
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gnuradio/uwb/uwb_pipeline_output_writer.h>
#include <gnuradio/uwb/uwb_ucr4_format.h>

#include <algorithm>
#include <atomic>
#include <charconv>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace gr {
namespace uwb {
namespace pipeline {

namespace {

constexpr double kSpeedOfLight = 299792458.0;
constexpr size_t kMaxUcr4Taps = 65535; // tap_count is a u16 on the wire

// ---------------------------------------------------------------------------
// Minimal deterministic JSON appenders.
//
// Every number is formatted with a fixed printf conversion so that the
// aggregated and the per-frame write paths produce byte-identical files.
// Non-finite doubles are emitted as `null`, matching
// `gr::uwb::core::json_f64` in uwb_scheduled_dump_io.h.
// ---------------------------------------------------------------------------
void
json_escape_into(std::string& out, const char* s)
{
    if (s == nullptr)
        return;
    for (const char* p = s; *p != '\0'; ++p) {
        const char c = *p;
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
}

void
json_escape_into(std::string& out, const std::string& s)
{
    json_escape_into(out, s.c_str());
}

inline void
j_key(std::string& s, const char* key)
{
    s += ",\"";
    s += key;
    s += "\":";
}

inline void
j_u64(std::string& s, const char* key, uint64_t v)
{
    j_key(s, key);
    s += std::to_string(v);
}

inline void
j_i64(std::string& s, const char* key, int64_t v)
{
    j_key(s, key);
    s += std::to_string(v);
}

inline void
j_bool(std::string& s, const char* key, bool v)
{
    j_key(s, key);
    s += v ? "true" : "false";
}

inline void
j_str(std::string& s, const char* key, const std::string& v)
{
    j_key(s, key);
    s.push_back('"');
    json_escape_into(s, v);
    s.push_back('"');
}

inline void
j_str(std::string& s, const char* key, const char* v)
{
    j_key(s, key);
    s.push_back('"');
    json_escape_into(s, v);
    s.push_back('"');
}

// `prec` significant digits, non-finite -> null.
//
// std::to_chars is used instead of snprintf because it is locale-independent:
// a host with LC_NUMERIC=de_DE would otherwise emit "0,15" and produce
// invalid JSON.  The output is deterministic, which is what the
// aggregate-vs-per-frame byte-identity contract depends on.
inline void
j_f64(std::string& s, const char* key, double v, int prec)
{
    j_key(s, key);
    if (!std::isfinite(v)) {
        s += "null";
        return;
    }
    char buf[64];
    const auto res = std::to_chars(
        buf, buf + sizeof(buf), v, std::chars_format::general, prec);
    if (res.ec != std::errc()) {
        s += "null";
        return;
    }
    s.append(buf, static_cast<size_t>(res.ptr - buf));
}

uint64_t
fnv1a64(const uint8_t* data, size_t n)
{
    uint64_t hash = 14695981039346656037ULL;
    for (size_t i = 0; i < n; ++i) {
        hash ^= static_cast<uint64_t>(data[i]);
        hash *= 1099511628211ULL;
    }
    return hash;
}

void
append_hex(std::string& out, const std::vector<uint8_t>& bytes)
{
    static const char kHex[] = "0123456789abcdef";
    out.reserve(out.size() + bytes.size() * 2);
    for (uint8_t b : bytes) {
        out.push_back(kHex[b >> 4]);
        out.push_back(kHex[b & 0x0Fu]);
    }
}

} // namespace

// Test-only hook storage.  Declared with default visibility because the
// library is built with -fvisibility=hidden and the QA translation unit lives
// in a different shared object.  Production cost is exactly one null check
// per frame (see Impl::writer_loop).
__attribute__((visibility("default"))) std::function<void()>&
test_slow_hook_storage()
{
    static std::function<void()> hook;
    return hook;
}

// ---------------------------------------------------------------------------
// OutputPayloadSlot
// ---------------------------------------------------------------------------
// Exported explicitly: `OutputPayloadSlot` itself is not marked UWB_API (it is
// a plain aggregate in the frozen header), but both the pipeline integration
// and the unit QA call clear() from outside this shared object, and the
// library is built with -fvisibility=hidden.
__attribute__((visibility("default"))) void
OutputPayloadSlot::clear()
{
    record = OutputWindowRecord{};
    cir_taps.clear();  // keeps capacity: that is the whole point
    psdu_bytes.clear();
}

// ---------------------------------------------------------------------------
// Impl
// ---------------------------------------------------------------------------
struct PipelineOutputWriter::Impl {
    // ---- static configuration ----
    OutputPersistenceConfig cfg;
    OutputRunMeta meta;
    size_t max_cir_taps = 0;
    size_t max_psdu_bytes = 0;
    std::string directory;

    // ---- preallocated slot pool (guarded by pool_mutex) ----
    std::vector<std::unique_ptr<OutputPayloadSlot>> pool;
    std::vector<OutputPayloadSlot*> free_list; // used as a stack
    std::mutex pool_mutex;

    // ---- bounded hand-off queue ----
    std::vector<OutputPayloadSlot*> queue;
    size_t q_head = 0;
    size_t q_tail = 0;
    size_t q_count = 0;
    std::mutex q_mutex;
    std::condition_variable cv;
    bool stop_flag = false;

    std::atomic<bool> enabled_flag{ false }; // configured successfully
    std::atomic<bool> running{ false };      // accepting submissions
    std::thread thread;
    std::mutex lifecycle_mutex;

    // ---- files / aggregate buffers (writer thread only) ----
    std::ofstream ucr4_out;
    std::ofstream cir_out;
    std::ofstream comm_out;
    std::vector<char> ucr4_buf;
    std::string jsonl_buf;
    std::string comm_buf;
    std::vector<int16_t> sc16_scratch;
    std::chrono::steady_clock::time_point last_flush{};
    uint64_t file_offset_taps = 0; // cumulative taps appended to <base>.ucr4

    // ---- test-only slow-writer hook (one null check per frame) ----
    std::function<void()> test_slow_hook;

    // ---- live log state (writer thread only) ----
    bool have_prev_sfd = false;
    bool prev_sfd_found = false;
    std::chrono::steady_clock::time_point run_start{};
    std::chrono::steady_clock::time_point last_summary{};
    uint64_t sum_windows = 0;
    uint64_t sum_sens_ok = 0;
    uint64_t sum_sens_total = 0;
    uint64_t sum_comm_att = 0;
    uint64_t sum_comm_ok = 0;
    uint64_t sum_comm_fcs = 0;
    uint64_t sum_sic_exec = 0;
    uint64_t sum_sic_commit = 0;
    double sum_supp_db = 0.0;
    int64_t sum_last_peak_tap = -1;
    std::string sum_last_psdu_hex;

    // ---- stats ----
    std::atomic<uint64_t> offered{ 0 };
    std::atomic<uint64_t> written{ 0 };
    std::atomic<uint64_t> dropped{ 0 };
    std::atomic<uint64_t> cir_written{ 0 };
    std::atomic<uint64_t> cir_failed{ 0 };
    std::atomic<uint64_t> comm_written{ 0 };
    std::atomic<uint64_t> comm_skipped{ 0 };
    std::atomic<uint64_t> aggregate_flushes{ 0 };
    std::atomic<uint64_t> bytes_written{ 0 };
    std::atomic<uint64_t> queue_high_watermark{ 0 };
    std::atomic<uint64_t> live_log_lines{ 0 };
    std::atomic<uint64_t> slots_allocated{ 0 };

    // ------------------------------------------------------------------
    // Derived constants
    // ------------------------------------------------------------------
    double range_m_per_tap() const
    {
        return meta.work_rate > 0.0 ? kSpeedOfLight / (2.0 * meta.work_rate)
                                    : 0.0;
    }

    // ------------------------------------------------------------------
    // Pool
    // ------------------------------------------------------------------
    void return_to_pool(OutputPayloadSlot* slot)
    {
        std::lock_guard<std::mutex> lock(pool_mutex);
        free_list.push_back(slot);
    }

    // ------------------------------------------------------------------
    // run.json
    // ------------------------------------------------------------------
    void write_run_json()
    {
        std::string s;
        s.reserve(512);
        s += "{\"tap_format\":\"ucr4\"";
        j_str(s, "byte_order", "little-endian");
        j_u64(s, "header_bytes", sizeof(Ucr4Header));
        j_u64(s, "bytes_per_tap", 4);
        j_str(s, "reconstruction", "fc32 = sc16 * cir_scale");
        j_f64(s, "range_m_per_tap", range_m_per_tap(), 9);
        j_str(s, "consumer_mode", meta.consumer_mode);
        j_f64(s, "native_rate", meta.native_rate, 17);
        j_f64(s, "work_rate", meta.work_rate, 17);
        j_f64(s, "uhd_center_freq", meta.uhd_center_freq, 17);
        j_str(s, "sic_policy", meta.sic_policy);
        j_str(s, "sic_backend", meta.sic_backend);
        j_u64(s, "cir_pre", meta.cir_pre);
        j_u64(s, "cir_post", meta.cir_post);
        j_u64(s, "code_index", meta.code_index);
        j_u64(s, "sync_repetitions", meta.sync_repetitions);
        j_f64(s, "samples_per_symbol", meta.samples_per_symbol, 17);
        j_u64(s, "max_psdu_bytes", meta.max_psdu_bytes);
        s += "}\n";

        std::ofstream f(directory + "/run.json", std::ios::trunc);
        if (f)
            f << s;
    }

    // ------------------------------------------------------------------
    // Flush / close
    // ------------------------------------------------------------------
    void write_buffer(std::ofstream& out, const char* data, size_t n)
    {
        if (n == 0 || !out.is_open())
            return;
        out.write(data, static_cast<std::streamsize>(n));
        bytes_written.fetch_add(n, std::memory_order_relaxed);
    }

    void flush_all()
    {
        if (!ucr4_buf.empty()) {
            write_buffer(ucr4_out, ucr4_buf.data(), ucr4_buf.size());
            ucr4_buf.clear();
        }
        if (!jsonl_buf.empty()) {
            write_buffer(cir_out, jsonl_buf.data(), jsonl_buf.size());
            jsonl_buf.clear();
        }
        if (!comm_buf.empty()) {
            write_buffer(comm_out, comm_buf.data(), comm_buf.size());
            comm_buf.clear();
        }
        if (ucr4_out.is_open())
            ucr4_out.flush();
        if (cir_out.is_open())
            cir_out.flush();
        if (comm_out.is_open())
            comm_out.flush();
        aggregate_flushes.fetch_add(1, std::memory_order_relaxed);
        last_flush = std::chrono::steady_clock::now();
    }

    void close_files()
    {
        if (ucr4_out.is_open()) {
            ucr4_out.flush();
            ucr4_out.close();
        }
        if (cir_out.is_open()) {
            cir_out.flush();
            cir_out.close();
        }
        if (comm_out.is_open()) {
            comm_out.flush();
            comm_out.close();
        }
    }

    // Flush when either the byte threshold or the time window is reached.
    // aggregate_bytes == 0 writes every record immediately; the resulting
    // bytes are identical because the append order does not change.
    void maybe_flush()
    {
        if (directory.empty())
            return;
        const size_t agg = cfg.writer_aggregate_bytes;
        if (agg == 0) {
            flush_all();
            return;
        }
        if (ucr4_buf.size() >= agg || jsonl_buf.size() >= agg ||
            comm_buf.size() >= agg) {
            flush_all();
            return;
        }
        const size_t window_ms = cfg.writer_aggregate_window_ms;
        if (window_ms > 0 &&
            std::chrono::steady_clock::now() - last_flush >=
                std::chrono::milliseconds(window_ms)) {
            flush_all();
        }
    }

    // ------------------------------------------------------------------
    // CIR summarisation
    // ------------------------------------------------------------------
    struct CirEmit {
        uint16_t code = 4; // invalid_input
        bool ok = false;
        size_t tap_count = 0;
        size_t peak_tap = 0;
        float peak_metric = 0.0f;
        double peak_delay_ns = 0.0;
        uint64_t offset_before = 0;
    };

    CirEmit summarize_cir(const OutputPayloadSlot& slot) const
    {
        const OutputWindowRecord& r = slot.record;
        CirEmit e;
        const size_t n = slot.cir_taps.size();

        // Fail-closed domain checks, in the order that names the first thing
        // that actually went wrong.
        if (!r.radar_executed)
            e.code = 4; // invalid_input
        else if (!r.radar_sfd_found)
            e.code = 1; // sfd_failed
        else if (r.radar_detected_origin < 0)
            e.code = 2; // timing_failed
        else if (n == 0 || n > kMaxUcr4Taps)
            e.code = 3; // cir_failed (u16 tap_count cannot describe it)
        else
            e.code = 0; // ok

        e.ok = (e.code == 0);
        e.tap_count = e.ok ? n : 0;
        e.offset_before = file_offset_taps;

        // `peak_tap` is the CIR kernel's own argmax of |raw tap| (L2).  Publish
        // it verbatim: recomputing it here with a different norm (e.g.
        // max(|re|,|im|)) could select a different tap while `cir_peak_metric`
        // stays the L2 magnitude, i.e. an inconsistent (tap, metric) pair.
        // The range check is defensive only: a record whose peak index is out
        // of the tap range is not something the kernel can produce.
        e.peak_tap = (r.peak_tap < n) ? r.peak_tap : 0;
        e.peak_metric = r.radar_peak_metric;
        if (!std::isfinite(e.peak_metric) || e.peak_metric <= 0.0f) {
            // Fallback only for a malformed record: use the L2 magnitude at the
            // published tap so the pair stays consistent.
            e.peak_metric =
                (e.peak_tap < n) ? std::abs(slot.cir_taps[e.peak_tap]) : 0.0f;
        }

        if (e.ok && meta.work_rate > 0.0) {
            e.peak_delay_ns =
                (static_cast<double>(e.peak_tap) -
                 static_cast<double>(r.cir_pre)) /
                meta.work_rate * 1.0e9;
        }
        return e;
    }

    // ------------------------------------------------------------------
    // <cir_base>.ucr4 + <cir_base>.jsonl
    // ------------------------------------------------------------------
    void emit_cir(const OutputPayloadSlot& slot, const CirEmit& e)
    {
        const OutputWindowRecord& r = slot.record;

        if (e.ok) {
            const size_t tap_count = e.tap_count;
            sc16_scratch.resize(tap_count * 2);
            const float scale = encode_cir_sc16(
                slot.cir_taps.data(), tap_count, sc16_scratch.data());

            Ucr4Header hdr{};
            std::memcpy(hdr.magic, "UCR4", 4);
            hdr.pulse_id = static_cast<uint32_t>(r.window_id & 0xFFFFFFFFu);
            hdr.status = 0;
            hdr.tap_count = static_cast<uint16_t>(tap_count);
            hdr.repetition_index = 0xFFFF; // aggregated-average convention
            hdr.repetition_count = static_cast<uint16_t>(
                std::min<size_t>(r.radar_valid_reps, 0xFFFFu));
            hdr.sfd_metric = r.sfd_metric;
            hdr.cir_peak_metric = e.peak_metric;
            hdr.peak_tap = static_cast<int32_t>(e.peak_tap);
            hdr.estimator_us = static_cast<uint32_t>(
                r.sensing_us > 0.0 ? static_cast<uint64_t>(r.sensing_us) : 0);
            hdr.freq_hz = meta.uhd_center_freq;
            hdr.freq_offset_hz = r.radar_cfo_hz;
            hdr.cir_scale = scale;

            const size_t base = ucr4_buf.size();
            ucr4_buf.resize(base + ucr4_record_bytes(tap_count));
            char* dst = ucr4_buf.data() + base;
            std::memcpy(dst, &hdr, sizeof(Ucr4Header));
            std::memcpy(dst + sizeof(Ucr4Header),
                        sc16_scratch.data(),
                        tap_count * sizeof(int16_t) * 2);

            file_offset_taps += tap_count;
            cir_written.fetch_add(1, std::memory_order_relaxed);
        } else {
            cir_failed.fetch_add(1, std::memory_order_relaxed);
        }

        std::string& line = jsonl_buf;
        line += "{\"pulse_id\":";
        line += std::to_string(r.window_id);
        j_u64(line, "schedule_index", r.schedule_index);
        j_str(line, "status", cir_status_name(e.code));
        j_f64(line, "sample_rate", meta.work_rate, 9);
        j_u64(line, "tap_count", e.tap_count);
        j_u64(line, "cir_pre_samples", r.cir_pre);
        j_u64(line, "cir_post_samples", r.cir_post);
        j_u64(line, "file_offset_taps", e.offset_before);
        j_i64(line, "zero_delay_tap", static_cast<int64_t>(r.cir_pre));
        j_u64(line, "peak_tap", e.ok ? e.peak_tap : 0);
        j_f64(line, "cir_peak_metric", e.ok ? e.peak_metric : 0.0, 9);
        j_f64(line, "peak_delay_from_calibration_ns", e.peak_delay_ns, 9);
        j_f64(line, "range_m_per_tap", range_m_per_tap(), 9);
        j_f64(line, "raw_l2_norm", e.ok ? r.raw_l2_norm : 0.0, 9);
        j_i64(line, "preamble_start_sample", r.radar_detected_start);
        // The RX-detected SFD start, which is a different quantity from the
        // detected SYNC origin (`radar_detected_origin`).  -1 when the SFD
        // stage did not produce one.
        j_i64(line, "sfd_start_sample", r.sfd_start_sample);
        j_i64(line, "cir_origin_sample", r.radar_cir_origin);
        j_bool(line, "sfd_ok", r.radar_sfd_found);
        j_bool(line, "timing_ok", r.radar_detected_origin >= 0);
        j_u64(line,
              "estimator_us",
              r.sensing_us > 0.0 ? static_cast<uint64_t>(r.sensing_us) : 0);
        j_f64(line, "cfo_hz", r.radar_cfo_hz, 9);
        j_u64(line, "valid_repetitions", r.radar_valid_reps);
        j_i64(line, "native_start_abs", r.native_start_abs);
        j_bool(line, "sic_executed", r.sic_executed);
        j_bool(line, "sic_committed", r.sic_committed);
        j_f64(line, "actual_suppression_db", r.actual_suppression_db, 9);
        line += "}\n";
    }

    // ------------------------------------------------------------------
    // <comm_base>.jsonl
    // ------------------------------------------------------------------
    void emit_comm(const OutputPayloadSlot& slot)
    {
        const OutputWindowRecord& r = slot.record;
        const bool ok_frame = r.comm_ok && r.comm_fcs_ok;
        if (cfg.comm_write_only_ok && !ok_frame) {
            comm_skipped.fetch_add(1, std::memory_order_relaxed);
            return;
        }

        const char* reason = r.comm_status_reason;
        const std::string status =
            (reason != nullptr && reason[0] != '\0')
                ? std::string(reason)
                : (r.comm_ok ? std::string("ok") : std::string("unknown"));

        const size_t psdu_n = slot.psdu_bytes.size();
        uint64_t psdu_fnv = r.comm_psdu_fnv1a;
        std::string psdu_hex;
        if (psdu_n > 0) {
            psdu_fnv = fnv1a64(slot.psdu_bytes.data(), psdu_n);
            append_hex(psdu_hex, slot.psdu_bytes);
        }

        std::string& line = comm_buf;
        line += "{\"packet_id\":";
        line += std::to_string(r.window_id);
        j_u64(line, "schedule_index", r.schedule_index);
        j_str(line, "status", status);
        j_bool(line, "fcs_pass", r.comm_fcs_ok);
        j_i64(line, "detected_start_sample", r.comm_sync_origin);
        j_i64(line, "predicted_start_sample", r.predicted_start_sample);
        j_i64(line, "native_window_start", r.native_window_start);
        j_f64(line, "resample_us", r.resample_us, 17);
        j_f64(line, "t_total_us", r.t_total_us, 17);
        j_i64(line, "sfd_start_sample", r.comm_sfd_start);
        j_f64(line, "cfo_hz", r.comm_cfo_hz, 17);
        j_bool(line, "phr_ok", r.comm_phr_ok);
        j_u64(line, "phr_psdu_length", r.comm_phr_length);
        j_f64(line, "phr_rate_mbps", r.phr_rate_mbps, 9);
        j_bool(line, "phr_corrected", r.phr_corrected);
        j_bool(line, "phr_uncorrectable", r.phr_uncorrectable);
        j_u64(line,
              "payload_nbytes",
              psdu_n > 0 ? psdu_n : r.comm_payload_bytes);
        j_u64(line, "fcs_received", r.fcs_received);
        j_u64(line, "fcs_calculated", r.fcs_calculated);
        j_u64(line, "psdu_fnv1a", psdu_fnv);
        j_str(line, "psdu_hex", psdu_hex);
        j_bool(line, "sic_executed", r.sic_executed);
        j_bool(line, "sic_committed", r.sic_committed);
        j_f64(line, "sic_suppression_db", r.actual_suppression_db, 9);
        j_str(line, "sic_status", r.sic_status);
        line += "}\n";

        comm_written.fetch_add(1, std::memory_order_relaxed);
    }

    // ------------------------------------------------------------------
    // Live console logger (this thread only)
    // ------------------------------------------------------------------
    void print_line(const char* buf, int n)
    {
        if (n <= 0)
            return;
        std::fwrite(buf, 1, static_cast<size_t>(n), stdout);
        std::fputc('\n', stdout);
        std::fflush(stdout);
        live_log_lines.fetch_add(1, std::memory_order_relaxed);
    }

    void log_events(const OutputWindowRecord& r, const CirEmit& e, bool has_cir)
    {
        if (r.comm_attempted &&
            (r.comm_sync_origin >= 0 || r.comm_ok || r.comm_fcs_ok)) {
            char buf[768];
            int n = std::snprintf(
                buf,
                sizeof(buf),
                "[WIN %05llu | sch %llu] COMM %s  fcs=%s len=%zuB rate=%.2fM "
                "cfo=%+.1fkHz sic=%s/%.1fdB",
                static_cast<unsigned long long>(r.window_id),
                static_cast<unsigned long long>(r.schedule_index),
                r.comm_ok ? "ok" : "fail",
                r.comm_fcs_ok ? "PASS" : "FAIL",
                r.comm_payload_bytes,
                static_cast<double>(r.phr_rate_mbps),
                r.comm_cfo_hz / 1000.0,
                r.sic_committed ? "on" : "off",
                static_cast<double>(r.actual_suppression_db));
            if (n > 0 && cfg.live_log_psdu) {
                const std::string& hex = slot_psdu_hex_source;
                const int used = std::min<int>(n, static_cast<int>(sizeof(buf)) - 1);
                const int room = static_cast<int>(sizeof(buf)) - used - 1;
                if (room > 8 && !hex.empty()) {
                    const int take = std::min<int>(room - 6,
                                                   static_cast<int>(hex.size()));
                    n += std::snprintf(buf + used,
                                       sizeof(buf) - used,
                                       " psdu=%.*s",
                                       take,
                                       hex.c_str());
                }
            }
            if (n > 0)
                print_line(buf, std::min<int>(n, static_cast<int>(sizeof(buf)) - 1));
        }

        if (has_cir) {
            const bool changed =
                !have_prev_sfd || prev_sfd_found != r.radar_sfd_found;
            have_prev_sfd = true;
            prev_sfd_found = r.radar_sfd_found;
            if (changed) {
                char buf[512];
                const int n = std::snprintf(
                    buf,
                    sizeof(buf),
                    "[WIN %05llu] SENSING %s peak_tap=%zu metric=%.3f "
                    "cfo=%+.1fkHz",
                    static_cast<unsigned long long>(r.window_id),
                    cir_status_name(e.code),
                    e.peak_tap,
                    static_cast<double>(e.peak_metric),
                    r.radar_cfo_hz / 1000.0);
                print_line(buf, n);
            }
        }
    }

    void log_all(const OutputWindowRecord& r, const CirEmit& e, bool has_cir)
    {
        char buf[1024];
        int n = std::snprintf(
            buf,
            sizeof(buf),
            "[WIN %05llu] sic=%s/%.1fdB sens=%s peak=%zu metric=%.3f "
            "cfo=%+.1fkHz comm=%s fcs=%s len=%zuB",
            static_cast<unsigned long long>(r.window_id),
            r.sic_committed ? "on" : "off",
            static_cast<double>(r.actual_suppression_db),
            has_cir ? cir_status_name(e.code) : "off",
            has_cir ? e.peak_tap : 0,
            has_cir ? static_cast<double>(e.peak_metric) : 0.0,
            r.radar_cfo_hz / 1000.0,
            r.comm_attempted ? (r.comm_ok ? "ok" : "fail") : "off",
            r.comm_fcs_ok ? "PASS" : "FAIL",
            r.comm_payload_bytes);
        if (n > 0 && cfg.live_log_psdu && !slot_psdu_hex_source.empty()) {
            const int used = std::min<int>(n, static_cast<int>(sizeof(buf)) - 1);
            const int room = static_cast<int>(sizeof(buf)) - used - 1;
            if (room > 8) {
                const int take = std::min<int>(
                    room - 6, static_cast<int>(slot_psdu_hex_source.size()));
                n += std::snprintf(
                    buf + used,
                    sizeof(buf) - used,
                    " psdu=%.*s",
                    take,
                    slot_psdu_hex_source.c_str());
            }
        }
        print_line(buf, std::min<int>(n, static_cast<int>(sizeof(buf)) - 1));
    }

    void emit_summary()
    {
        const auto now = std::chrono::steady_clock::now();
        const double secs =
            std::chrono::duration<double>(now - run_start).count();
        const double rate = secs > 0.0 ? static_cast<double>(sum_windows) / secs
                                       : 0.0;
        // Queue depth is owned by q_mutex; take a short snapshot rather than
        // reading the counters racily.
        size_t q_depth = 0;
        size_t q_cap = 0;
        {
            std::lock_guard<std::mutex> lock(q_mutex);
            q_depth = q_count;
            q_cap = queue.size();
        }
        char buf[768];
        const int n = std::snprintf(
            buf,
            sizeof(buf),
            "[SUMMARY] win=%llu rate=%.1f/s sens=%llu/%llu comm=%llu/%llu "
            "fcs=%llu/%llu sic=%llu/%llu avg=%.1fdB peak=%lld wr=%llu/%llu "
            "q=%zu/%zu psdu=%s",
            static_cast<unsigned long long>(sum_windows),
            rate,
            static_cast<unsigned long long>(sum_sens_ok),
            static_cast<unsigned long long>(sum_sens_total),
            static_cast<unsigned long long>(sum_comm_ok),
            static_cast<unsigned long long>(sum_comm_att),
            static_cast<unsigned long long>(sum_comm_fcs),
            static_cast<unsigned long long>(sum_comm_att),
            static_cast<unsigned long long>(sum_sic_commit),
            static_cast<unsigned long long>(sum_sic_exec),
            sum_sic_commit > 0 ? sum_supp_db / static_cast<double>(sum_sic_commit)
                               : 0.0,
            static_cast<long long>(sum_last_peak_tap),
            static_cast<unsigned long long>(
                written.load(std::memory_order_relaxed)),
            static_cast<unsigned long long>(
                dropped.load(std::memory_order_relaxed)),
            q_depth,
            q_cap,
            sum_last_psdu_hex.empty() ? "-" : sum_last_psdu_hex.c_str());
        print_line(buf, n);

        sum_windows = 0;
        sum_sens_ok = 0;
        sum_sens_total = 0;
        sum_comm_att = 0;
        sum_comm_ok = 0;
        sum_comm_fcs = 0;
        sum_sic_exec = 0;
        sum_sic_commit = 0;
        sum_supp_db = 0.0;
    }

    // The PSDU hex of the frame currently being logged.  Only the writer
    // thread touches this, so a plain member is enough.
    std::string slot_psdu_hex_source;

    // ------------------------------------------------------------------
    // Per-frame emission
    // ------------------------------------------------------------------
    void write_slot(OutputPayloadSlot& slot)
    {
        const OutputWindowRecord& r = slot.record;
        const bool has_cir = r.radar_executed;
        CirEmit cir;
        if (has_cir)
            cir = summarize_cir(slot);

        slot_psdu_hex_source.clear();
        if (!slot.psdu_bytes.empty())
            append_hex(slot_psdu_hex_source, slot.psdu_bytes);

        if (has_cir && cfg.write_cir && !directory.empty())
            emit_cir(slot, cir);
        if (r.comm_attempted && cfg.write_comm && !directory.empty())
            emit_comm(slot);

        switch (cfg.live_log_mode) {
        case LiveLogMode::Off:
            break;
        case LiveLogMode::Events:
            log_events(r, cir, has_cir);
            break;
        case LiveLogMode::All:
            log_all(r, cir, has_cir);
            break;
        case LiveLogMode::Summary:
            break; // emitted on the timer so idle periods still tick
        }

        // Summary accumulators (cheap, and only read by the summary printer).
        ++sum_windows;
        if (has_cir) {
            ++sum_sens_total;
            if (cir.ok)
                ++sum_sens_ok;
            sum_last_peak_tap = static_cast<int64_t>(cir.peak_tap);
        }
        if (r.comm_attempted) {
            ++sum_comm_att;
            if (r.comm_ok)
                ++sum_comm_ok;
            if (r.comm_fcs_ok)
                ++sum_comm_fcs;
            if (!slot_psdu_hex_source.empty())
                sum_last_psdu_hex = slot_psdu_hex_source;
        }
        if (r.sic_executed) {
            ++sum_sic_exec;
            if (r.sic_committed) {
                ++sum_sic_commit;
                sum_supp_db += static_cast<double>(r.actual_suppression_db);
            }
        }

        if (!directory.empty())
            maybe_flush();
    }

    // ------------------------------------------------------------------
    // Writer thread
    // ------------------------------------------------------------------
    void writer_loop()
    {
        const size_t interval_ms =
            cfg.live_log_interval_ms > 0 ? cfg.live_log_interval_ms : 500;
        const auto interval = std::chrono::milliseconds(interval_ms);
        const bool summary_mode = cfg.live_log_mode == LiveLogMode::Summary;

        for (;;) {
            OutputPayloadSlot* slot = nullptr;
            {
                std::unique_lock<std::mutex> lock(q_mutex);
                if (summary_mode) {
                    // Timed wait so an idle queue still produces a dashboard
                    // line every `live_log_interval_ms`.
                    cv.wait_for(lock, interval, [this] {
                        return stop_flag || q_count != 0;
                    });
                } else {
                    cv.wait(lock, [this] { return stop_flag || q_count != 0; });
                }
                if (q_count != 0) {
                    slot = queue[q_head];
                    q_head = (q_head + 1) % queue.size();
                    --q_count;
                } else if (stop_flag) {
                    break;
                }
            }
            if (slot != nullptr) {
                if (test_slow_hook)
                    test_slow_hook();
                write_slot(*slot);
                written.fetch_add(1, std::memory_order_relaxed);
                return_to_pool(slot);
            }
            if (summary_mode &&
                std::chrono::steady_clock::now() - last_summary >= interval) {
                emit_summary();
                last_summary = std::chrono::steady_clock::now();
            }
        }
        flush_all();
        if (summary_mode)
            emit_summary();
    }

    // ------------------------------------------------------------------
    // Lifecycle
    // ------------------------------------------------------------------
    void reset_stats()
    {
        offered.store(0);
        written.store(0);
        dropped.store(0);
        cir_written.store(0);
        cir_failed.store(0);
        comm_written.store(0);
        comm_skipped.store(0);
        aggregate_flushes.store(0);
        bytes_written.store(0);
        queue_high_watermark.store(0);
        live_log_lines.store(0);
        slots_allocated.store(0);
    }

    // Assumes lifecycle_mutex is held by the caller.
    void stop_locked()
    {
        {
            std::lock_guard<std::mutex> lock(q_mutex);
            stop_flag = true;
            running.store(false, std::memory_order_release);
        }
        cv.notify_all();
        if (thread.joinable())
            thread.join();

        // Defensive: the loop already drained, but a submission that raced
        // with the shutdown must still be accounted for.
        for (;;) {
            OutputPayloadSlot* slot = nullptr;
            {
                std::lock_guard<std::mutex> lock(q_mutex);
                if (q_count == 0)
                    break;
                slot = queue[q_head];
                q_head = (q_head + 1) % queue.size();
                --q_count;
            }
            write_slot(*slot);
            written.fetch_add(1, std::memory_order_relaxed);
            return_to_pool(slot);
        }

        flush_all();
        close_files();
        file_offset_taps = 0;
    }
};

// ---------------------------------------------------------------------------
// PipelineOutputWriter
// ---------------------------------------------------------------------------
PipelineOutputWriter::PipelineOutputWriter() : impl_(new Impl) {}

PipelineOutputWriter::~PipelineOutputWriter()
{
    stop_and_drain();
}

bool PipelineOutputWriter::start(const OutputPersistenceConfig& cfg,
                                 const OutputRunMeta& meta,
                                 size_t max_cir_taps,
                                 size_t max_psdu_bytes)
{
    Impl& d = *impl_;
    std::lock_guard<std::mutex> lock(d.lifecycle_mutex);

    // Idempotent restart: tear down any previous run first.
    d.stop_locked();
    d.enabled_flag.store(false, std::memory_order_release);

    d.cfg = cfg;
    d.meta = meta;
    d.max_cir_taps = max_cir_taps;
    d.max_psdu_bytes = max_psdu_bytes;
    d.directory.clear();
    d.reset_stats();

    if (!cfg.enabled())
        return false;
    if (cfg.writer_queue_capacity == 0)
        return false;

    if (!cfg.output_dir.empty()) {
        std::error_code ec;
        std::filesystem::create_directories(cfg.output_dir, ec);
        if (ec)
            return false;
        d.directory = cfg.output_dir;
    }

    // ---- preallocated slot pool ----
    const size_t pool_size =
        std::max(cfg.writer_queue_capacity + 32, static_cast<size_t>(64));
    d.free_list.clear(); // drop the dangling pointers before the pool dies
    d.pool.clear();
    d.pool.reserve(pool_size);
    d.free_list.reserve(pool_size);
    for (size_t i = 0; i < pool_size; ++i) {
        std::unique_ptr<OutputPayloadSlot> slot(new OutputPayloadSlot());
        slot->cir_taps.reserve(max_cir_taps);
        slot->psdu_bytes.reserve(max_psdu_bytes);
        d.free_list.push_back(slot.get());
        d.pool.push_back(std::move(slot));
    }
    d.slots_allocated.store(pool_size);

    // ---- bounded queue ----
    d.queue.assign(cfg.writer_queue_capacity, nullptr);
    d.q_head = 0;
    d.q_tail = 0;
    d.q_count = 0;

    // ---- aggregate buffers ----
    const size_t agg = cfg.writer_aggregate_bytes;
    d.ucr4_buf.clear();
    d.ucr4_buf.reserve(
        std::max(agg, ucr4_record_bytes(std::max<size_t>(max_cir_taps, 1))));
    d.jsonl_buf.clear();
    d.jsonl_buf.reserve(std::max(agg, static_cast<size_t>(4096)));
    d.comm_buf.clear();
    d.comm_buf.reserve(std::max(agg, static_cast<size_t>(4096)));
    d.sc16_scratch.assign(std::max<size_t>(max_cir_taps, 1) * 2, 0);
    d.file_offset_taps = 0;

    // ---- files ----
    if (!d.directory.empty()) {
        if (cfg.write_cir) {
            const std::string pre = d.directory + "/" + cfg.cir_base_name;
            d.ucr4_out.open(pre + ".ucr4",
                            std::ios::binary | std::ios::trunc);
            d.cir_out.open(pre + ".jsonl", std::ios::trunc);
            if (!d.ucr4_out.is_open() || !d.cir_out.is_open()) {
                d.close_files();
                return false;
            }
        }
        if (cfg.write_comm) {
            d.comm_out.open(d.directory + "/" + cfg.comm_base_name + ".jsonl",
                            std::ios::trunc);
            if (!d.comm_out.is_open()) {
                d.close_files();
                return false;
            }
        }
        d.write_run_json();
    }

    // ---- test-only hook (copied once; one null check per frame) ----
    d.test_slow_hook = test_slow_hook_storage();

    // ---- live log / thread ----
    d.have_prev_sfd = false;
    d.prev_sfd_found = false;
    d.sum_windows = 0;
    d.sum_sens_ok = 0;
    d.sum_sens_total = 0;
    d.sum_comm_att = 0;
    d.sum_comm_ok = 0;
    d.sum_comm_fcs = 0;
    d.sum_sic_exec = 0;
    d.sum_sic_commit = 0;
    d.sum_supp_db = 0.0;
    d.sum_last_peak_tap = -1;
    d.sum_last_psdu_hex.clear();
    d.slot_psdu_hex_source.clear();
    d.run_start = std::chrono::steady_clock::now();
    d.last_flush = d.run_start;
    d.last_summary = d.run_start;
    {
        std::lock_guard<std::mutex> qlock(d.q_mutex);
        d.stop_flag = false;
    }
    d.running.store(true, std::memory_order_release);
    d.enabled_flag.store(true, std::memory_order_release);
    d.thread = std::thread(&Impl::writer_loop, &d);
    return true;
}

void PipelineOutputWriter::stop_and_drain()
{
    Impl& d = *impl_;
    std::lock_guard<std::mutex> lock(d.lifecycle_mutex);
    d.stop_locked();
}

bool PipelineOutputWriter::enabled() const
{
    return impl_->enabled_flag.load(std::memory_order_acquire);
}

OutputPayloadSlot* PipelineOutputWriter::try_acquire_slot()
{
    Impl& d = *impl_;
    d.offered.fetch_add(1, std::memory_order_relaxed);
    if (!d.running.load(std::memory_order_acquire)) {
        d.dropped.fetch_add(1, std::memory_order_relaxed);
        return nullptr;
    }
    std::lock_guard<std::mutex> lock(d.pool_mutex);
    if (d.free_list.empty()) {
        d.dropped.fetch_add(1, std::memory_order_relaxed);
        return nullptr;
    }
    OutputPayloadSlot* slot = d.free_list.back();
    d.free_list.pop_back();
    slot->clear();
    return slot;
}

void PipelineOutputWriter::release_slot(OutputPayloadSlot* slot)
{
    if (slot == nullptr)
        return;
    Impl& d = *impl_;
    d.return_to_pool(slot);
    // The matching `offered` was already counted at acquisition; releasing
    // converts it into a drop so the ledger stays balanced.
    d.dropped.fetch_add(1, std::memory_order_relaxed);
}

bool PipelineOutputWriter::try_submit(OutputPayloadSlot* slot)
{
    if (slot == nullptr)
        return false;
    Impl& d = *impl_;

    bool accepted = false;
    {
        std::lock_guard<std::mutex> lock(d.q_mutex);
        if (d.running.load(std::memory_order_acquire) &&
            d.q_count < d.queue.size()) {
            d.queue[d.q_tail] = slot;
            d.q_tail = (d.q_tail + 1) % d.queue.size();
            ++d.q_count;
            accepted = true;
            uint64_t old =
                d.queue_high_watermark.load(std::memory_order_relaxed);
            while (old < d.q_count &&
                   !d.queue_high_watermark.compare_exchange_weak(
                       old, d.q_count, std::memory_order_relaxed)) {
            }
        }
    }
    if (!accepted) {
        d.return_to_pool(slot);
        d.dropped.fetch_add(1, std::memory_order_relaxed);
        return false;
    }
    d.cv.notify_one();
    return true;
}

OutputWriterStats PipelineOutputWriter::stats() const
{
    const Impl& d = *impl_;
    OutputWriterStats s;
    s.enabled = d.enabled_flag.load(std::memory_order_acquire);
    s.offered = d.offered.load(std::memory_order_relaxed);
    s.written = d.written.load(std::memory_order_relaxed);
    s.dropped = d.dropped.load(std::memory_order_relaxed);
    s.cir_written = d.cir_written.load(std::memory_order_relaxed);
    s.cir_failed = d.cir_failed.load(std::memory_order_relaxed);
    s.comm_written = d.comm_written.load(std::memory_order_relaxed);
    s.comm_skipped = d.comm_skipped.load(std::memory_order_relaxed);
    s.aggregate_flushes = d.aggregate_flushes.load(std::memory_order_relaxed);
    s.bytes_written = d.bytes_written.load(std::memory_order_relaxed);
    s.queue_high_watermark =
        d.queue_high_watermark.load(std::memory_order_relaxed);
    s.live_log_lines = d.live_log_lines.load(std::memory_order_relaxed);
    s.slots_allocated = d.slots_allocated.load(std::memory_order_relaxed);
    return s;
}

const std::string& PipelineOutputWriter::directory() const
{
    return impl_->directory;
}

} // namespace pipeline
} // namespace uwb
} // namespace gr
