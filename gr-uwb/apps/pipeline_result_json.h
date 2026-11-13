/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Shared reporting helpers for the UHD SC16 pipeline benchmark drivers.
 *
 * Owner: main agent (2026-11-11 round 2).  Both
 * `uwb_uhd_sc16_pipeline_runner` (historical cold-start protocol) and
 * `benchmark_sc16_pipeline_steady` (steady-state protocol) emit the SAME
 * JSON/CSV schema so that tools/performance/compare_pipeline_windows.py can
 * diff two runs without knowing which driver produced them.
 *
 * JSON schema (version 1):
 * {
 *   "schema": "uwb-pipeline-results",
 *   "schema_version": 1,
 *   "meta": { "<key>": <json scalar>, ... },
 *   "ledger": { ... },              // see append_ledger_json()
 *   "windows": [ { ... }, ... ]     // see append_window_json()
 * }
 *
 * The 11 functional fields compared by the verification tool are exactly the
 * WindowResult fields a schedule change must not alter.  Timing fields and
 * worker_id are present but are never required to match.
 *
 * R05 (2026-11-13) appended the sensing provenance/timing fields to both the
 * JSON window object and the CSV row -- `radar_cfo_hz`, `radar_cir_origin`,
 * `radar_detected_origin`, `radar_valid_reps`, `radar_frontend_us`,
 * `radar_cir_us` and the signed remainder `radar_other_us`.  They are APPENDED
 * after the existing fields, so every pre-R05 consumer keeps working and the
 * schema_version stays 1 (an additive change).
 */

#pragma once

#include <gnuradio/uwb/uwb_uhd_sc16_pipeline.h>

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <ostream>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace gr {
namespace uwb {
namespace pipeline {
namespace bench {

inline const char* terminal_state_name(WindowTerminalState s)
{
    switch (s) {
    case WindowTerminalState::Completed:        return "Completed";
    case WindowTerminalState::ProcessingFailed: return "ProcessingFailed";
    case WindowTerminalState::DroppedAdmission: return "DroppedAdmission";
    case WindowTerminalState::InvalidGap:       return "InvalidGap";
    case WindowTerminalState::Cancelled:        return "Cancelled";
    }
    return "Unknown";
}

inline std::string json_escape(const std::string& in)
{
    std::string out;
    out.reserve(in.size() + 8);
    for (char c : in) {
        switch (c) {
        case '"':  out += "\\\""; break;
        case '\\': out += "\\\\"; break;
        case '\n': out += "\\n"; break;
        case '\r': out += "\\r"; break;
        case '\t': out += "\\t"; break;
        default:
            if (static_cast<unsigned char>(c) < 0x20) {
                char buf[8];
                std::snprintf(buf, sizeof(buf), "\\u%04x",
                              static_cast<unsigned>(static_cast<unsigned char>(c)));
                out += buf;
            } else {
                out += c;
            }
        }
    }
    return out;
}

// JSON number formatting: never emit nan/inf (invalid JSON).  Non-finite
// values are emitted as null so the consumer can report them as unavailable
// instead of silently treating them as zero.
inline std::string json_number(double v)
{
    if (!std::isfinite(v)) {
        return "null";
    }
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%.9g", v);
    return buf;
}

inline std::string json_number(int64_t v)
{
    return std::to_string(v);
}

inline std::string json_number(uint64_t v)
{
    return std::to_string(v);
}

// LP64 makes int64_t == long and uint64_t == unsigned long, but a caller
// holding `long long`/`int` would otherwise be ambiguous between the double and
// integer overloads.  Keep these exact-match overloads.
inline std::string json_number(long long v)
{
    return std::to_string(v);
}

inline std::string json_number(int v)
{
    return std::to_string(v);
}

inline void append_ledger_json(std::ostream& os, const PipelineLedger& l)
{
    os << "  \"ledger\": {"
       << "\"offered_windows\": " << l.offered_windows
       << ", \"admitted_windows\": " << l.admitted_windows
       << ", \"completed_windows\": " << l.completed_windows
       << ", \"processing_failed\": " << l.processing_failed
       << ", \"dropped_admission\": " << l.dropped_admission
       << ", \"invalid_gap_before_admission\": " << l.invalid_gap_before_admission
       << ", \"invalid_gap_after_admission\": " << l.invalid_gap_after_admission
       << ", \"cancelled_windows\": " << l.cancelled_windows
       << ", \"total_samples_received\": " << l.total_samples_received
       << ", \"short_reads\": " << l.short_reads
       << ", \"read_timeouts\": " << l.read_timeouts
       << ", \"read_overflows\": " << l.read_overflows
       << "},\n";
}

inline void append_window_json(std::ostream& os, const WindowResult& r)
{
    os << "  {\"window_id\": " << r.window_id
       << ", \"worker_id\": " << r.worker_id
       << ", \"state\": \"" << terminal_state_name(r.state) << "\""
       << ", \"native_start_abs\": " << r.native_start_abs
       << ", \"native_sample_count\": " << r.native_sample_count
       << ", \"sic_executed\": " << (r.sic_executed ? "true" : "false")
       << ", \"sic_committed\": " << (r.sic_committed ? "true" : "false")
       << ", \"actual_suppression_db\": " << json_number(static_cast<double>(r.actual_suppression_db))
       << ", \"sic_status\": \"" << json_escape(r.sic_status) << "\""
       << ", \"radar_executed\": " << (r.radar_executed ? "true" : "false")
       << ", \"radar_sfd_found\": " << (r.radar_sfd_found ? "true" : "false")
       << ", \"radar_peak_metric\": " << json_number(static_cast<double>(r.radar_peak_metric))
       << ", \"radar_detected_start\": " << r.radar_detected_start
       << ", \"radar_cfo_hz\": " << json_number(r.radar_cfo_hz)
       << ", \"radar_cir_origin\": " << r.radar_cir_origin
       << ", \"radar_detected_origin\": " << r.radar_detected_origin
       << ", \"radar_valid_reps\": " << r.radar_valid_reps
       << ", \"radar_frontend_us\": " << json_number(r.radar_frontend_us)
       << ", \"radar_cir_us\": " << json_number(r.radar_cir_us)
       << ", \"radar_other_us\": " << json_number(r.radar_other_us)
       << ", \"comm_attempted\": " << (r.comm_attempted ? "true" : "false")
       << ", \"comm_ok\": " << (r.comm_ok ? "true" : "false")
       << ", \"comm_status_reason\": \"" << json_escape(r.comm_status_reason ? r.comm_status_reason : "") << "\""
       << ", \"comm_sync_origin\": " << r.comm_sync_origin
       << ", \"comm_sfd_start\": " << r.comm_sfd_start
       << ", \"comm_cfo_hz\": " << json_number(r.comm_cfo_hz)
       << ", \"comm_phr_ok\": " << (r.comm_phr_ok ? "true" : "false")
       << ", \"comm_phr_length\": " << r.comm_phr_length
       << ", \"comm_fcs_ok\": " << (r.comm_fcs_ok ? "true" : "false")
       << ", \"comm_payload_bytes\": " << r.comm_payload_bytes
       << ", \"comm_psdu_fnv1a\": " << r.comm_psdu_fnv1a
       << ", \"comm_us\": " << json_number(r.comm_us)
       << ", \"timing\": {"
       << "\"slice_us\": " << json_number(r.timing.slice_us)
       << ", \"queue_us\": " << json_number(r.timing.queue_us)
       << ", \"convert_us\": " << json_number(r.timing.convert_us)
       << ", \"resample_us\": " << json_number(r.timing.resample_us)
       << ", \"sic_us\": " << json_number(r.timing.sic_us)
       << ", \"sensing_us\": " << json_number(r.timing.sensing_us)
       << ", \"comm_us\": " << json_number(r.timing.comm_us)
       << ", \"e2e_us\": " << json_number(r.timing.e2e_us)
       << "}}";
}

// meta is a list of (key, already-JSON-encoded value) pairs so each driver can
// publish numbers, booleans and strings without a variant type.
using MetaList = std::vector<std::pair<std::string, std::string>>;

inline bool write_results_json(const std::string& path,
                               const MetaList& meta,
                               const PipelineLedger& ledger,
                               const std::vector<WindowResult>& windows)
{
    if (path.empty()) {
        return true;
    }
    std::ofstream f(path, std::ios::trunc);
    if (!f) {
        return false;
    }
    f << "{\n"
      << "  \"schema\": \"uwb-pipeline-results\",\n"
      << "  \"schema_version\": 1,\n"
      << "  \"meta\": {";
    for (size_t i = 0; i < meta.size(); ++i) {
        if (i != 0) {
            f << ", ";
        }
        f << "\"" << json_escape(meta[i].first) << "\": " << meta[i].second;
    }
    f << "},\n";
    append_ledger_json(f, ledger);
    f << "  \"windows\": [\n";
    for (size_t i = 0; i < windows.size(); ++i) {
        append_window_json(f, windows[i]);
        if (i + 1 != windows.size()) {
            f << ",";
        }
        f << "\n";
    }
    f << "  ]\n}\n";
    return static_cast<bool>(f);
}

// One row per window (including drops), so the CSV is also a drop audit.
inline bool write_results_csv(const std::string& path,
                              const std::vector<WindowResult>& windows)
{
    if (path.empty()) {
        return true;
    }
    std::ofstream f(path, std::ios::trunc);
    if (!f) {
        return false;
    }
    f << "window_id,worker_id,state,native_start_abs,native_sample_count,"
         "queue_us,convert_us,resample_us,sic_us,sensing_us,comm_us,e2e_us,"
         "sic_executed,sic_committed,actual_suppression_db,"
         "radar_executed,radar_sfd_found,radar_peak_metric,radar_detected_start,"
         "radar_cfo_hz,radar_cir_origin,radar_detected_origin,radar_valid_reps,"
         "radar_frontend_us,radar_cir_us,radar_other_us,"
         "comm_attempted,comm_ok,comm_status_reason,comm_sync_origin,comm_sfd_start,"
         "comm_cfo_hz,comm_phr_ok,comm_phr_length,comm_fcs_ok,comm_payload_bytes,"
         "comm_psdu_fnv1a\n";
    f << std::setprecision(6) << std::fixed;
    for (const auto& r : windows) {
        f << r.window_id << ',' << r.worker_id << ','
          << terminal_state_name(r.state) << ','
          << r.native_start_abs << ',' << r.native_sample_count << ','
          << r.timing.queue_us << ',' << r.timing.convert_us << ','
          << r.timing.resample_us << ',' << r.timing.sic_us << ','
          << r.timing.sensing_us << ',' << r.timing.comm_us << ',' << r.timing.e2e_us << ','
          << (r.sic_executed ? 1 : 0) << ',' << (r.sic_committed ? 1 : 0) << ','
          << r.actual_suppression_db << ','
          << (r.radar_executed ? 1 : 0) << ',' << (r.radar_sfd_found ? 1 : 0) << ','
          << r.radar_peak_metric << ',' << r.radar_detected_start << ','
          << r.radar_cfo_hz << ',' << r.radar_cir_origin << ','
          << r.radar_detected_origin << ',' << r.radar_valid_reps << ','
          << r.radar_frontend_us << ',' << r.radar_cir_us << ','
          << r.radar_other_us << ','
          << (r.comm_attempted ? 1 : 0) << ',' << (r.comm_ok ? 1 : 0) << ','
          << (r.comm_status_reason ? r.comm_status_reason : "") << ','
          << r.comm_sync_origin << ',' << r.comm_sfd_start << ','
          << r.comm_cfo_hz << ',' << (r.comm_phr_ok ? 1 : 0) << ','
          << r.comm_phr_length << ',' << (r.comm_fcs_ok ? 1 : 0) << ','
          << r.comm_payload_bytes << ',' << r.comm_psdu_fnv1a << '\n';
    }
    return static_cast<bool>(f);
}

inline std::string meta_string(const std::string& v)
{
    return "\"" + json_escape(v) + "\"";
}

inline std::string meta_bool(bool v)
{
    return v ? "true" : "false";
}

} // namespace bench
} // namespace pipeline
} // namespace uwb
} // namespace gr
