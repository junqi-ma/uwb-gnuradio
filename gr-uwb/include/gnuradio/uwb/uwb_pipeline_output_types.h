/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Configuration / statistics types for the pure C++ pipeline output writer.
 *
 * This header is deliberately free of any GNU Radio, PMT or pipeline
 * dependency so that `uwb_uhd_sc16_pipeline.h` can include it without creating
 * an include cycle, and so that the writer can be unit-tested on its own.
 *
 * Owner: frozen interface (Phase 0 of
 * docs/performance/OpenCode_Pipeline落盘与实时日志_多Agent执行计划_2026-11-14.md).
 * Do not change a field name or its meaning without coordinating every track.
 */

#ifndef INCLUDED_GNURADIO_UWB_UWB_PIPELINE_OUTPUT_TYPES_H
#define INCLUDED_GNURADIO_UWB_UWB_PIPELINE_OUTPUT_TYPES_H

#include <cstddef>
#include <cstdint>
#include <string>

namespace gr {
namespace uwb {
namespace pipeline {

// ---------------------------------------------------------------------------
// Live console logging
// ---------------------------------------------------------------------------
// Off     : no output at all (zero cost: the writer is not even created when
//           the output directory is also empty).
// Events  : one line per state change or successful decode; quiet windows
//           print nothing.
// All     : one line per window (SIC / sensing / comm summary).
// Summary : one rolling dashboard line every `live_log_interval_ms`.
enum class LiveLogMode { Off = 0, Events = 1, All = 2, Summary = 3 };

inline const char* live_log_mode_name(LiveLogMode m)
{
    switch (m) {
    case LiveLogMode::Off:
        return "off";
    case LiveLogMode::Events:
        return "events";
    case LiveLogMode::All:
        return "all";
    case LiveLogMode::Summary:
        return "summary";
    }
    return "off";
}

// ---------------------------------------------------------------------------
// OutputPersistenceConfig
// ---------------------------------------------------------------------------
// Everything here is consumed once, in UhdSc16Pipeline::initialize().  The DSP
// worker hot path only ever reads the already-constructed writer pointer.
struct OutputPersistenceConfig {
    // Empty output_dir AND live_log_mode == Off => the writer is never
    // created: the hot path costs one boolean test and nothing else.
    std::string output_dir;

    // File base names inside output_dir.
    //   <cir_base_name>.ucr4 / <cir_base_name>.jsonl / run.json
    //   <comm_base_name>.jsonl
    std::string cir_base_name = "cir";
    std::string comm_base_name = "demod_results";

    bool write_cir = true;
    bool write_comm = true;
    // Only persist a demod_results.jsonl line when the frame decoded and its
    // FCS passed.  Failed attempts are still counted in the stats.
    bool comm_write_only_ok = false;

    // Bounded hand-off queue between commit_result() and the writer thread.
    // Must be > 0.  When it is full the frame is dropped (never blocks).
    size_t writer_queue_capacity = 64;

    // Aggregate buffering: flush when either threshold is reached.  Setting
    // writer_aggregate_bytes == 0 falls back to one write() per frame, which
    // must produce a byte-identical file.
    size_t writer_aggregate_bytes = 1u << 20; // 1 MiB
    size_t writer_aggregate_window_ms = 50;

    LiveLogMode live_log_mode = LiveLogMode::Off;
    size_t live_log_interval_ms = 500;
    // Include the decoded PSDU as lowercase hex in the live log.
    bool live_log_psdu = true;

    // True when the writer must be constructed at all.
    bool enabled() const
    {
        return !output_dir.empty() || live_log_mode != LiveLogMode::Off;
    }
};

// ---------------------------------------------------------------------------
// OutputRunMeta — static run configuration written once to run.json
// ---------------------------------------------------------------------------
struct OutputRunMeta {
    std::string consumer_mode;
    double native_rate = 0.0;
    double work_rate = 0.0;
    double uhd_center_freq = 0.0;
    std::string sic_policy;
    std::string sic_backend;
    size_t cir_pre = 0;
    size_t cir_post = 0;
    size_t code_index = 0;
    size_t sync_repetitions = 0;
    double samples_per_symbol = 0.0;
    size_t max_psdu_bytes = 127;
};

// ---------------------------------------------------------------------------
// OutputWriterStats
// ---------------------------------------------------------------------------
// Ledger invariant, asserted by QA after stop_and_drain():
//
//   offered == written + dropped
//
// `offered` counts every window the pipeline tried to hand to the writer
// (i.e. every try_acquire_slot() call).  `written` counts frames the writer
// thread finished (ucr4 record and/or jsonl line).  `dropped` counts both
// pool-exhaustion and queue-full rejections.
struct OutputWriterStats {
    bool enabled = false;
    uint64_t offered = 0;
    uint64_t written = 0;
    uint64_t dropped = 0;

    uint64_t cir_written = 0;  // ucr4 records appended
    uint64_t cir_failed = 0;   // jsonl-only frames (tap_count == 0)
    uint64_t comm_written = 0; // demod_results.jsonl lines
    uint64_t comm_skipped = 0; // comm_write_only_ok filtered them out

    uint64_t aggregate_flushes = 0;
    uint64_t bytes_written = 0;
    uint64_t queue_high_watermark = 0;
    uint64_t live_log_lines = 0;
    uint64_t slots_allocated = 0;
};

} // namespace pipeline
} // namespace uwb
} // namespace gr

#endif /* INCLUDED_GNURADIO_UWB_UWB_PIPELINE_OUTPUT_TYPES_H */
