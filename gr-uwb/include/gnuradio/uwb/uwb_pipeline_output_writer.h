/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Pure C++ asynchronous output writer for UhdSc16Pipeline.
 *
 * Zero GNU Radio / PMT dependency: only the standard library plus the pure
 * C++ core headers.  It reuses the on-disk contract of the GNU Radio
 * `UwbCirWriter` block (`.ucr4` + `.jsonl` + `run.json`) and of
 * `uwb_scheduled_dump_io.h` (`demod_results.jsonl`), but it is a separate
 * implementation so the pipeline never links `gnuradio-runtime` for I/O.
 *
 * Threading contract
 * ------------------
 *   * DSP workers call try_acquire_slot() / try_submit() and never block.
 *   * commit_result() calls try_submit() while holding its own commit mutex,
 *     so try_submit() is O(1) and never allocates.
 *   * ONE background thread performs every write(), flush() and printf().
 *   * stop_and_drain() joins that thread after draining the queue.
 *
 * Back-pressure policy: drop, never block.  A full pool or a full queue
 * increments `dropped` and returns immediately; the UHD RX thread and the DSP
 * workers are never slowed down by disk I/O.
 *
 * Ordering contract: the pipeline only submits from the in-order emit section
 * of commit_result(), so every file and every log line is strictly ascending
 * by window_id.
 *
 * Owner: frozen interface (Phase 0 of
 * docs/performance/OpenCode_Pipeline落盘与实时日志_多Agent执行计划_2026-11-14.md).
 */

#ifndef INCLUDED_GNURADIO_UWB_UWB_PIPELINE_OUTPUT_WRITER_H
#define INCLUDED_GNURADIO_UWB_UWB_PIPELINE_OUTPUT_WRITER_H

#include <gnuradio/uwb/api.h>
#include <gnuradio/uwb/uwb_pipeline_output_types.h>

#include <complex>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace gr {
namespace uwb {
namespace pipeline {

// ---------------------------------------------------------------------------
// OutputWindowRecord — the compact mirror of WindowResult the writer consumes
// ---------------------------------------------------------------------------
// Deliberately NOT `WindowResult`: the writer must not depend on the pipeline
// header (that would be an include cycle), and the pipeline must never embed a
// variable-length payload in `results_`.  Every field here is a scalar or a
// short string; the variable-length data lives in OutputPayloadSlot.
struct OutputWindowRecord {
    uint64_t window_id = 0;
    uint64_t schedule_index = 0;
    int worker_id = 0;
    int64_t native_start_abs = 0;
    size_t native_sample_count = 0;
    int64_t native_window_start = -1;
    int64_t predicted_start_sample = -1;

    // ---- SIC ----
    bool sic_executed = false;
    bool sic_committed = false;
    float actual_suppression_db = 0.0f;
    std::string sic_status; // reused capacity; never reallocated per window

    // ---- sensing ----
    bool radar_executed = false;
    bool radar_sfd_found = false;
    float radar_peak_metric = 0.0f;
    int64_t radar_detected_start = -1;
    double radar_cfo_hz = 0.0;
    int64_t radar_cir_origin = -1;
    int64_t radar_detected_origin = -1;
    size_t radar_valid_reps = 0;
    // Extras that RadarCirResult carries but WindowResult does not publish.
    float sfd_metric = 0.0f;
    float raw_l2_norm = 0.0f;
    size_t cir_pre = 0;
    size_t cir_post = 0;

    // ---- comm ----
    bool comm_attempted = false;
    bool comm_ok = false;
    // Static string literal from demod_status_reason(); never owned.
    const char* comm_status_reason = "";
    int64_t comm_sync_origin = -1;
    int64_t comm_sfd_start = -1;
    double comm_cfo_hz = 0.0;
    bool comm_phr_ok = false;
    uint16_t comm_phr_length = 0;
    bool comm_fcs_ok = false;
    size_t comm_payload_bytes = 0;
    uint64_t comm_psdu_fnv1a = 0;
    // Extras that DemodResult carries but WindowResult does not publish.
    uint16_t fcs_received = 0;
    uint16_t fcs_calculated = 0;
    float phr_rate_mbps = 0.0f;
    bool phr_corrected = false;
    bool phr_uncorrectable = false;

    // ---- timing (already measured by the worker; never re-read here) ----
    double resample_us = 0.0;
    double t_total_us = 0.0;
    double sensing_us = 0.0;
};

// ---------------------------------------------------------------------------
// OutputPayloadSlot — one preallocated hand-off buffer
// ---------------------------------------------------------------------------
// Allocated once at start(); after that, filling a slot never allocates as long
// as the payload fits the preallocated capacity.  `clear()` resets the sizes
// but keeps the capacity.
struct OutputPayloadSlot {
    OutputWindowRecord record;

    // Sensing taps, true amplitude (not normalized).  Capacity is
    // cir_pre + cir_post; size is the actual tap count (0 on failure).
    std::vector<std::complex<float>> cir_taps;

    // Decoded PSDU bytes (including FCS).  Capacity is max_psdu_bytes.
    std::vector<uint8_t> psdu_bytes;

    void clear();
};

// ---------------------------------------------------------------------------
// PipelineOutputWriter
// ---------------------------------------------------------------------------
class UWB_API PipelineOutputWriter
{
public:
    PipelineOutputWriter();
    ~PipelineOutputWriter();

    PipelineOutputWriter(const PipelineOutputWriter&) = delete;
    PipelineOutputWriter& operator=(const PipelineOutputWriter&) = delete;

    /**
     * Create the output directory, open every enabled file, write run.json and
     * start the background thread.
     *
     * \param cfg   output configuration (already validated by the caller).
     * \param meta  static run parameters written to run.json.
     * \param max_cir_taps  per-slot tap capacity (cir_pre + cir_post).
     * \param max_psdu_bytes per-slot PSDU capacity.
     * \return false when the directory/files cannot be created, or when
     *         cfg.enabled() is false.  A false return leaves the object
     *         disabled: try_acquire_slot() returns nullptr and stats() reports
     *         enabled == false.
     */
    bool start(const OutputPersistenceConfig& cfg,
               const OutputRunMeta& meta,
               size_t max_cir_taps,
               size_t max_psdu_bytes);

    /**
     * Stop accepting submissions, drain the queue, flush and close every file,
     * then join the background thread.  Idempotent.  After this returns,
     * `stats().offered == stats().written + stats().dropped`.
     */
    void stop_and_drain();

    bool enabled() const;

    /**
     * Non-blocking slot acquisition.  Increments `offered`; on pool exhaustion
     * increments `dropped` and returns nullptr.  The caller must not block or
     * retry.
     */
    OutputPayloadSlot* try_acquire_slot();

    /**
     * Return a slot that was acquired but will not be submitted (e.g. the
     * window failed before any payload was copied).  Does not touch the
     * ledger: the matching `offered` was already counted at acquisition and
     * this call converts it into a `dropped`.
     */
    void release_slot(OutputPayloadSlot* slot);

    /**
     * Hand a filled slot to the writer thread.  Takes ownership on success.
     * On failure (queue full, writer not running) the slot is returned to the
     * free pool by this call, `dropped` is incremented, and the caller must
     * not touch the pointer again.
     */
    bool try_submit(OutputPayloadSlot* slot);

    OutputWriterStats stats() const;

    // Directory actually used (empty when disabled).  For diagnostics only.
    const std::string& directory() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace pipeline
} // namespace uwb
} // namespace gr

#endif /* INCLUDED_GNURADIO_UWB_UWB_PIPELINE_OUTPUT_WRITER_H */
