/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * PHASE 0 STUB — replaced by Track 1.
 *
 * Registers the test target so `ctest` sees it from the first commit; the real
 * cases (UCR4 round-trip, aggregate vs per-frame byte identity, failure-frame
 * offsets, demod_results.jsonl parseability, slow-writer ledger, disabled
 * behaviour) are Track 1's deliverable.
 */

#define BOOST_TEST_MODULE qa_uwb_pipeline_output_writer
#include <boost/test/unit_test.hpp>

#include <gnuradio/uwb/uwb_pipeline_output_writer.h>

BOOST_AUTO_TEST_CASE(test_disabled_writer_is_inert)
{
    gr::uwb::pipeline::PipelineOutputWriter w;
    gr::uwb::pipeline::OutputPersistenceConfig cfg;
    gr::uwb::pipeline::OutputRunMeta meta;

    BOOST_CHECK(!cfg.enabled());
    BOOST_CHECK(!w.start(cfg, meta, 116, 127));
    BOOST_CHECK(!w.enabled());
    BOOST_CHECK(w.try_acquire_slot() == nullptr);
    BOOST_CHECK(!w.try_submit(nullptr));
    BOOST_CHECK(!w.stats().enabled);
    w.stop_and_drain();
}
