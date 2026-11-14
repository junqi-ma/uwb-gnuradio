/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * PHASE 0 STUB — replaced by Track 1.
 *
 * This file exists only so that the frozen interface compiles and links while
 * the three tracks of
 * docs/performance/OpenCode_Pipeline落盘与实时日志_多Agent执行计划_2026-11-14.md
 * work in parallel.  It deliberately implements the DISABLED behaviour:
 *
 *   start()             -> always false (never creates a file)
 *   enabled()           -> false
 *   try_acquire_slot()  -> nullptr
 *   try_submit()        -> false, slot returned to the (empty) pool
 *   stats().enabled     -> false
 *
 * Consequence for Track 2: the "writer failed to start" path is exactly what
 * this stub exercises, so Track 2 must make the pipeline keep running with
 * output disabled.  The real writer is Track 1's deliverable.
 */

#include <gnuradio/uwb/uwb_pipeline_output_writer.h>

namespace gr {
namespace uwb {
namespace pipeline {

void OutputPayloadSlot::clear()
{
    record = OutputWindowRecord{};
    cir_taps.clear();
    psdu_bytes.clear();
}

struct PipelineOutputWriter::Impl {
    std::string directory;
};

PipelineOutputWriter::PipelineOutputWriter() : impl_(new Impl) {}

PipelineOutputWriter::~PipelineOutputWriter() = default;

bool PipelineOutputWriter::start(const OutputPersistenceConfig&,
                                 const OutputRunMeta&,
                                 size_t,
                                 size_t)
{
    return false;
}

void PipelineOutputWriter::stop_and_drain() {}

bool PipelineOutputWriter::enabled() const { return false; }

OutputPayloadSlot* PipelineOutputWriter::try_acquire_slot() { return nullptr; }

void PipelineOutputWriter::release_slot(OutputPayloadSlot*) {}

bool PipelineOutputWriter::try_submit(OutputPayloadSlot*) { return false; }

OutputWriterStats PipelineOutputWriter::stats() const
{
    OutputWriterStats s;
    s.enabled = false;
    return s;
}

const std::string& PipelineOutputWriter::directory() const
{
    return impl_->directory;
}

} // namespace pipeline
} // namespace uwb
} // namespace gr
