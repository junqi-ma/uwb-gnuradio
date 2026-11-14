#!/bin/sh
# PHASE 0 STUB — replaced by Track 3.
#
# End-to-end QA for `--output-dir` / `--live-log` on uwb_uhd_sc16_pipeline_runner.
# The real script must:
#   1. run `--consumer sensing --output-dir <tmp>` on the 737.28 MS/s fixture
#      and assert cir.ucr4 / cir.jsonl / run.json exist and parse;
#   2. run `--consumer comm --output-dir <tmp>` and assert demod_results.jsonl
#      exists and every line parses;
#   3. assert `--live-log summary` emits at least one line and does not corrupt
#      the `--json-out` document;
#   4. assert that omitting --output-dir creates no files at all.
#
# Exit 77 == "skipped" (CMake SKIP_RETURN_CODE), so a stub can never be mistaken
# for a pass.  Exit 0 == all cases passed.
set -eu

echo "SKIP: qa_uwb_pipeline_output_runner.sh is the Phase 0 stub" >&2
exit 77
