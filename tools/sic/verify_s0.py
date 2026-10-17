#!/usr/bin/env python3
# Copyright 2026
#
# SPDX-License-Identifier: GPL-3.0-or-later
"""INDEPENDENT verifier for the S0 preamble-collision / preamble-only-SIC artifacts.

    python3 tools/sic/verify_s0.py --results gr-uwb/build-sic/sic-results
    python3 tools/sic/verify_s0.py --self-test

WHAT IT CHECKS, AND HOW IT IS INDEPENDENT
-----------------------------------------
It reads ONLY the DECLARED REQUIRED artifacts written under ``--results`` (and
its per-run subdirectories), and re-derives every value in Python.  It never
imports a repo C++ module, binding or helper; it never trusts a summary number
it can recompute from ``windows.jsonl`` or ``stage_times.csv``.

The declared-required artifact set (the four frozen tables of
``docs/sic/S0_artifacts.md``) is:

  ``run_manifest.json``, ``windows.jsonl``, ``stage_times.csv``,
  ``summary.json``                                          # one set per run

``truth.jsonl`` (generator sidecar) and a per-window reconstruction ``*.npz``
are read *only when present* to independently recompute the reconstruction
metrics; their absence is not a failure (the sidecar is not a required
artifact).

Checks implemented (each failure names the artifact + field):

1.  ``run_manifest.json`` present/well-formed; re-hash every referenced
    config / template / fixture file and compare to the manifest hash (mismatch
    = FAIL); ``commit`` / ``dirty`` / ``build_type`` / loaded-library fields
    must be non-empty; work rate is the frozen 998.4 MS/s grid, native rate is
    a known CG400 rate, seed lists exist and are disjoint.
2.  ``windows.jsonl``: every line parses; ``run_id``/``window_id`` present and
    unique; the window-id set matches ``summary.json``
    ``windows_expected``/``windows_completed``; ``sample_count`` is consistent
    with the declared window bounds; ``overlap`` is well-formed ``[lo,hi)`` and
    ``overlap.samples == hi-lo``.
3.  Three-way pairing: every window carries exactly the declared consumers
    (``clean``/``raw``/``sic``, ``sic`` only when ``sic_mode != off``); a
    window lacking a declared consumer FAILS.  For the ``baseline`` preset with
    SIC off the ``raw`` consumer result must equal ``clean``.
4.  ``stage_times.csv`` long form: for every ``(run_id, window_id, consumer)``
    the SFD children (``sfd_bootstrap``/``sfd_tail_retrack``/``sfd_cfo_refit``/
    ``sfd_final``/``sfd_anchor_other``) sum to ``<= sfd`` and the CIR children
    (``cir_estimate``/``soft_fir``/``postprocess``) sum to ``<= cir``; the
    reported ``total`` is not a naive parent+child double count; ``not_run``
    rows have an empty ``us`` and timed rows do not; a declared failing stage
    is timed; every consumer has an authoritative ``total`` (and ``core_total``
    for SIC).
5.  ``summary.json``: recompute the completion counts from ``windows.jsonl``
    and cross-check; recompute per-stage timing statistics from the CSV and
    cross-check the summary; every frozen key must exist (a missing key is a
    FAILURE, never silently OK); every unexecuted declared item must appear in
    ``not_run[]``.
6.  Demod result integrity: a window claiming ``fcs_pass``/``psdu_ok`` must
    carry the fields that justify it; for the canonical code-9 sensing PSDU a
    present decoded PSDU must equal the canonical hex
    ``47261DF66F4C1BEF45C8F77CE77BD7D8C4D180FB1221``.
7.  Metrics sanity: ``suppression_db`` must be the ``10*log10(||i||^2/
    ||i-i_hat||^2)``-style number, consistent with ``nmse``
    (``nmse = ||i-i_hat||^2/||i||^2``); a ``||y||^2/||y_clean||^2`` substitute
    is rejected where the fields make it detectable; NMSE is in ``[0, inf)``;
    when a truth sidecar / per-window ``i``+``i_hat`` is present the metrics are
    recomputed over the declared evaluation window and compared.
8.  ``uses_truth`` discipline: any window with ``sic_state.uses_truth == true``
    must be flagged diagnostic-only and must NOT be counted in the effect /
    real-time gates (recomputed over non-truth windows) and must not be used
    when the gates claim a pass.
9.  R02/R06 (below): effective runtime config + hash, one case/config/
    template/input length across functional/core/GR/native, no silent config
    rewrite, the new named quantities, SEARCH ROI separate from SUBTRACTION
    ROI, and estimation / online-gate / evaluation region separation with the
    gate recomputed on the online region.

R01 (ACTUAL-OUTPUT cancellation; main effect gate)
-------------------------------------------------
The full reconstruction NMSE (``recon.nmse``/``recon.suppression_db``) is a
summary of how accurate ``i_hat`` is; it is NOT how much interference the
produced signal actually removed.  The frozen measurement contract
(``docs/sic/S0_整改_测量合同.md``) therefore adds an actual-output metric that
this verifier recomputes from the archived arrays, never from a declared mask:

  For each archived ``recon/window_*.npz`` containing ``y``, ``y_clean`` and
  ``i`` (same length as the evaluated window), over the generator-side truth
  overlap ``[overlap_lo, overlap_hi)`` (NEVER a mask chosen to look best)::

      delta_actual          = y - y_clean      # y = raw input, y_clean = SIC output
      residual_interference = i - delta_actual
      actual_nmse           = ||residual||^2 / ||i||^2
      actual_suppression_db = -10 log10(actual_nmse)

  It is compared to the declared ``recon.actual_nmse`` /
  ``recon.actual_suppression_db``.  The applied subtraction mask
  ``recon.sub_lo``/``recon.sub_hi``/``recon.coverage`` is checked against the
  real ``y != y_clean`` support: a declared mask that is smaller or larger than
  the support the output actually changed over is a FAIL.  When the run declares
  the new actual fields but ships no arrays, that is a FAIL, not a skip.

  The main effect gate must be the ``actual_*`` aggregate.  A run that still
  computes its gate from the reconstruction ``suppression_db`` (or claims
  ``gates.eff_pass`` without an actual aggregate) FAILS.  An uncommitted/bypassed
  window must declare ``actual_suppression_db == 0`` (or N/A when ``||i||^2==0``)
  and must never be counted as cancelled.

R04 (raw target protection)
---------------------------
Target protection is recomputed from the archived raw (non-L2-normalised) CIR
and a fixed target ROI: the complex raw CIR target amplitude/phase error and the
local peak shift are re-derived from ``cir_raw_clean``/``cir_raw_sic`` and
compared to the declared ``target_amp_err_db``/``target_phase_err_deg``/
``target_peak_shift``.  A run whose effect gate only carries the normalized-CIR
target is a FAIL.  The target ROI must be identical across the run (fixed on the
calibration set), never re-searched on the SIC result.

Frozen field names (canonical location first, aliases accepted for robustness):
  recon.actual_nmse, recon.actual_suppression_db, recon.sub_lo, recon.sub_hi,
  recon.coverage, recon.uses_truth;
  summary.effect.median_actual_suppression_db, frac_actual_ge_3db,
  n_interference_windows, n_na_windows;
  per-window target block (top-level ``target`` or ``recon``):
  target_lo, target_hi, target_amp_err_db, target_phase_err_deg,
  target_peak_shift, cir_raw_target_amp_db, weak_target_detected,
  false_peak_count;
  arrays in recon/window_*.npz: y (raw), y_clean (SIC output), i, i_hat,
  cir_raw_clean, cir_raw_sic.

R02 (effective config / one case across implementations)
--------------------------------------------------------
A run must declare the effective runtime config and its hash, and declare, for
functional / core / GR / native, the SAME case / config / template / input
length.  Any difference must be explicitly recorded; an unflagged difference is
a FAIL.  A window whose resolved config (``fit_reps`` / ``validation_reps`` /
``sic_k`` / ``sync_reps``) differs from the effective nominal must record the
override -- a silently widened ``validation_reps`` is a FAIL.  An unsatisfiable
declared config (``fit_reps+validation_reps > sync_reps``, ``validation_reps <
8``, ``fit_reps <= 0``) is a FAIL, never silently rewritten.  New named
quantities per window (fit length, estimation range, online-validation length,
independent-eval length, signal-evidenced start/stop, subtraction mask) must be
present and internally consistent, and the candidate SEARCH ROI must be
declared separately from the applied SUBTRACTION ROI (a subtraction whose hi
equals the search hi is the clipped-tail bug).

Frozen manifest fields (aliases accepted): effective_config / config /
resolved_config + effective_config_hash / config_hash; implementation_configs
(functional/core/gr/native).  Per window: ``regions`` (or recon/config)
estimation_lo/hi, fit_len, online_validation_lo/hi, online_validation_len,
eval_lo/hi, eval_len, signal_start/stop, gate_lo/hi, gate_keys, search_roi,
subtraction_roi / sub_lo|sub_hi.

R06 (estimation / online-gate / evaluation separation)
------------------------------------------------------
The estimation region and the online-validation region must each be DISJOINT
from the final evaluation region.  The commit-gate region must be the online
region (never the evaluation region) and its keys must not name evaluation-only
metrics; where the run declares ``recon.online_nmse`` the verifier recomputes it
from the archived arrays over the online region only.  This is the data
dependency the rectification requires: changing the evaluation region content
must not move a frozen fit/gate parameter.

The long-form CSV schema is fixed here as the authoritative contract:
``run_id,window_id,consumer,tier,stage,us,detail`` where ``tier`` is one of
``cxx``/``adapter``/``outer`` and an empty ``us`` pairs with a ``not_run``
``detail``.  When a stage name exists in BOTH the demod and the SIC stage set
(``cfo``, ``cir``) inside one ``(window, consumer)`` group, the ``detail``
column must carry a ``sic``/``demod`` scope tag so the timers are not
ambiguous.

R10 (provenance binding, false-vs-null, superseded, MATLAB, perf binding)
-----------------------------------------------------------------------
R10 turns the anti-shortcut requirements into fail-closed checks:

* **Provenance binding.**  Every manifest (the app ``run_manifest.json`` and a
  benchmark ``sic_perf_v2`` summary) must bind the ACTUAL run: a real commit
  (a placeholder/omitted identifier fails), a dirty-patch identifier (a sha256
  of the worktree status/diff; a dirty tree with an empty-tree hash fails), the
  loaded-library file hash, the benchmark binary file hash, the template hashes
  bound to the effective config, an input/fixture hash, and the resolved
  effective config + its sha256.  The library/binary hashes are re-derived from
  disk when the referenced file is resolvable here.  A run that only carries a
  stale/omitted identifier, or omits the benchmark/library hash, FAILS.
  The recorded benchmark-binary hash is required everywhere, but its re-hash
  MATCH is scoped: it is strict for a run that DECLARES itself benchmark/perf
  output (a ``sic_perf*`` schema, ``benchmark_exe_used=true``, or a
  ``mode``/``implementation``/``row_kind`` of ``benchmark``/``perf``/``core``/
  ``gr``/``native``), and merely informational for an APP run
  (``uwb_sim_preamble_sic.py`` functional/effect/negative/native/paced), whose
  execution path never runs the benchmark binary -- the current value is
  recorded and a differing hash is a skip/note, not a failure.  The
  loaded-library hash stays strict and re-hashed for EVERY run.
  ``provenance{...}`` is canonical; the same fields are accepted top-level.
* **false vs null.**  ``gates.* == false`` is a TESTED-AND-FAILED result and
  must carry explicit tested evidence (a ``gate_evidence`` mapping, a
  ``<gate>_evidence`` field, or a ``gate`` object with ``tested``/``evidence``).
  ``gates.* == null`` is untested and must be named in ``not_run[]``.  A gate
  that is both ``false`` and named untested is a contradiction.  A static
  ``not_run`` template must not name a required artifact that actually exists.
* **Superseded.**  An old run dir may be explicitly marked ``superseded`` (a
  per-run ``superseded`` block/bool, or a top-level results index mapping run
  labels to reasons) so a full-tree scan can tell "superseded" from
  "must-pass".  A superseded marker must carry a reason; a superseded run may
  skip the provenance hard-failures but MUST NOT claim production-profile
  evidence.  A run that claims production evidence while using synthetic input,
  pre-R07 (non finite-SYNC) templates, or no real downstream consumer FAILS.
* **MATLAB status.**  Each run must carry an explicit 已实跑/未实跑 flag with a
  command and a log path.  An executed claim must have the log on disk; a
  MATLAB-parity claim without a real MATLAB run FAILS (Python-only parity is not
  MATLAB acceptance).
* **Perf/effect binding.**  Every perf row (``*perf_table.csv`` rows and
  ``sic_perf_v2`` summary `stats`) must carry a config hash and template hash
  and bind to an effect-verified case; a production row that is not
  effect-verified FAILS, an unverified row must be explicitly labelled, and all
  production rows must share one config/template identity.

Percentiles are recomputed with numpy's default (``linear``) interpolation;
the generator must use the same convention for the summary to cross-check.

S1 (strict 8-SYNC estimation, frozen model, full-preamble cancellation)
-----------------------------------------------------------------------
The S1 order adds checks 1-7 in ``check_s1``.  They fail closed and are kept
separate from the S0 checks:

1. **Strict-prefix evidence.**  Every committed window carries a ``sic_model``
   block with ``observed_end_sample``, ``training_lo/hi``,
   ``estimation_ready_sample`` and an ``estimation_ranges`` mapping
   (``training``/``cfo``/``cir``); every declared estimation input range must
   end before ``observed_end_sample``.  A strict 8-SYNC claim requires
   ``fit_reps == 8``, ``observed_end_sample == training_hi`` and an explicitly
   declared (zero) guard.
2. **Frozen model.**  A stable ``model_id`` + ``fingerprint`` and every frozen
   parameter (origin, cfo, tau[], gain[], n_paths, support, sub, phase
   reference) are required, and the ``apply`` step must reference the same
   identity and mirror the same parameters with ``refit_count == 0`` and no
   parameter delta.  Mutating one frozen parameter after estimation is caught
   by the estimation/apply comparison.
3. **Support semantics.**  ``support_source`` must be the configured-nominal
   prediction and ``support == [origin-pre_tail, origin+sync_reps*sps+
   post_tail)``; a support returned as ``confirmed*sps`` (the old 8-SYNC clip)
   fails, and ``support_coverage`` is recomputed against the truth overlap.
4. **Regions.**  Training [0,8) SYNC, independent evaluation [8,64) SYNC and
   the last-4-SYNC tail must be explicit and disjoint from training; the gate
   must not use eval data.  The independent-eval and tail regions are DEFINED
   as the nominal repetition ranges intersected with the applied subtraction
   (``sub_lo``/``sub_hi``) and the truth overlap: a run that cannot cancel the
   whole nominal region must declare that clip (``cancellation_clipped`` /
   ``eval_covered_frac`` / ``tail_covered_frac``) with a covered fraction that
   matches the recomputed intersection, and a region that extends outside
   ``sub``/overlap while claiming the full nominal coverage is a FAIL.  A
   run with ZERO committed windows (all bypassed) has no frozen model to audit,
   so the S1 model-evidence family (1-4) is SKIPped, never FAILed; its bypass
   bit-identity / false-commit / output-change gates live in the negative
   aggregate.
5. **Actual effect.**  The suppression is recomputed from ``y``/``y_clean``/
   ``i`` as ``i - (y - y_clean)`` over the truth overlap, the independent
   evaluation region and the tail region; the three per-window metrics and the
   three summary medians/counts are required (a reconstruction-NMSE-only
   report fails) and the evaluation/tail regions must actually change.  When a
   run ships only a subset of the per-window ``recon/window_*.npz`` arrays it
   must declare an ``archive_policy`` (``per_cell`` / ``archived_ids`` /
   ``total_windows`` plus the stated non-archived limitation); the verifier
   then independently recomputes only the ARCHIVED windows (>= 4 per
   cell/case) and skips the rest.  With no policy the full-array requirement is
   kept.
6. **Seeds.**  ``seed_lists.calibration``/``acceptance`` must be recorded and
   disjoint; ``calibration_windows`` records the generated
   ``(seed, seed_offset, noise_root)`` tuples, and no acceptance window may
   reuse a calibration seed/tuple or another acceptance window's tuple.
7. **Provenance/report.**  ``effective_config`` declares the S1 model mode,
   ``fit_reps == 8``, ``evaluation_reps >= 8`` and ``online_validation_reps ==
   0``; effect/core/GR/CIR/full-demod/native rows share that config hash, model
   mode, template and input length; ``index.json`` references the actual latest
   run and its declared window count matches the artifact.

Canonical S1 field names are listed in the ``S1`` constants block; aliases are
accepted for robustness.  A SIC-on run that declares no model mode is a FAIL,
and an explicit ``legacy_cover``-style declaration is a SKIP that may not claim
an S1 pass or report S1 region metrics.

Aggregate roots (``--results`` on a sweep or negative-set directory)
-------------------------------------------------------------------
A results root that contains ``cells/`` (main sweep) or ``cases/`` (negative
set) is an AGGREGATE root: every child run directory is verified with the same
single-run checks, and the aggregate tables are cross-checked against the child
artifacts (a table/child mismatch FAILS):

* ``main_matrix.json`` must carry one row per ``cells/<cell_id>``; each row's
  ``cell_id``/``eff_pass``/``median_suppression_db`` must match that cell's
  ``summary.json``; exactly one ``effective_config_hash`` must be shared by all
  cells; and every cell must declare the same template / shared-input hashes.
* ``negative_table.json`` per-case ``windows``/``detected``/``committed``/
  ``false_commit``/``output_change``/``bypass_bit_identical`` must be
  recomputed from the case's ``windows.jsonl``/``summary.json``; every
  actionable case (``expected_negative``) must carry >= 1000 windows; and the
  overall counters, rates and Wilson CI must be recomputed from the per-case
  counts.

An aggregate root prints one line per child (PASS/FAIL + the first failing
check) instead of per-window spam; ``--verbose`` restores the full detail.
``index.json`` is a results-root artifact, so an aggregate CHILD does not have
to resolve one: the index is only consulted when it sits in the aggregate root
itself.

Exit status 0 iff every check passes; non-zero with a clear diff otherwise.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
import re
import shutil
import sys
import tempfile

try:  # numpy 1.21.5 is present but stay usable without it where possible
    import numpy as np
except Exception:  # pragma: no cover - defensive
    np = None

# ---------------------------------------------------------------------------
# Frozen constants (README.md / S0_artifacts.md / config_schema.json).  They are
# repeated here on purpose: this file is a second, independent statement of the
# contract, not a reader of the generator.
# ---------------------------------------------------------------------------

CANONICAL_PSDU_HEX = "47261DF66F4C1BEF45C8F77CE77BD7D8C4D180FB1221"
CANONICAL_PSDU_BYTES = bytes.fromhex(CANONICAL_PSDU_HEX)
WORK_HZ = 998400000.0
NATIVE_HZ = (491520000.0, 737280000.0)
SYNC_SAMPLES_PER_SYMBOL = 1016

PRESETS = {"baseline", "main", "negative", "stress"}
SIC_MODES = {"off", "estimate", "oracle"}
INPUT_GRIDS = {"work", "cg400"}
CODES = {9, 10, 11, 12}
SYNC_REPS = {16, 32, 64, 128}
SFD_MODES = {"4z2", "ieee"}
SOFTCHIP_MODES = {"bypass", "full", "rake4"}
SIC_KS = {1, 2, 4, 8, 16}
CONSUMERS = ("clean", "raw", "sic")

REQUIRED_ARTIFACTS = (
    "run_manifest.json",
    "windows.jsonl",
    "stage_times.csv",
    "summary.json",
)

MANIFEST_KEYS = (
    "commit", "dirty", "build_type", "compiler", "flags", "lib_paths", "cpu",
    "os", "gnuradio_version", "volk_version", "config_path", "config_hash",
    "fixture_hashes", "template_hashes", "template_meta", "seed_lists",
    "sample_rate_work", "sample_rate_native", "input_grid", "preset",
    "sic_mode",
)

WINDOW_KEYS = (
    "schema_version", "run_id", "window_id", "epoch_id", "sample_rate",
    "sample_count", "window_start_sample", "predicted_sensing_start",
    "sensing_profile_id", "sensing_code", "sensing_sync_reps",
    "sensing_sfd_mode", "template_id", "overlap", "isr_db", "snr_db",
    "comm_code", "sic_k", "fit_reps", "validation_reps", "roi",
    "demod_softchip_mode", "clean", "raw", "sic_state", "admission", "recon",
)

SUMMARY_KEYS = (
    "run_id", "preset", "input_grid", "sic_mode", "windows_expected",
    "windows_completed", "clean", "raw", "sic", "effect", "demod", "negative",
    "runtime", "gates", "not_run",
)

# Consumer keys that exist only when SIC is on; when SIC is off they are emitted
# as JSON null plus a matching not_run[] reason (rev 2 summary completeness).
DEMOD_KEYS = ("clean_ok", "raw_ok", "sic_ok", "raw_ok_sic_fail",
              "raw_fail_sic_ok", "failing_stages")

# stage_times.csv (rev 2 adds an explicit `scope` column)
DEMOD_STAGES = {
    "timing", "timing_coarse", "timing_fine_track", "cfo", "sfd",
    "sfd_bootstrap", "sfd_tail_retrack", "sfd_cfo_refit", "sfd_final",
    "sfd_anchor_other", "cir", "cir_estimate", "soft_fir", "postprocess",
    "ns_sfd", "phr", "payload", "total",
}
SFD_CHILDREN = (
    "sfd_bootstrap", "sfd_tail_retrack", "sfd_cfo_refit", "sfd_final",
    "sfd_anchor_other",
)
CIR_CHILDREN = ("cir_estimate", "soft_fir", "postprocess")
# `timing_coarse`/`timing_fine_track` are the sub-timers of `timing` (the
# producer tags them `demod_timing_child`); they are excluded from the
# top-level demod sum so `total` is not inflated.
TIMING_CHILDREN = ("timing_coarse", "timing_fine_track")
# S1 §8 splits the SIC cost into acquisition / 8-SYNC estimation / model
# preparation / backtrack of the first 8 SYNC / forward apply.  These names are
# accepted alongside the S0 names so a strict-8-SYNC run can report the split
# without being rejected as an unknown timer.  `core_total` remains the
# authoritative SIC total; the children still sum to <= it.
S1_SIC_STAGES = {
    "acquisition", "acquire", "param_estimate", "parameter_estimate",
    "estimate8", "estimate_8sync", "model_prepare", "model_prep",
    "backtrack", "backtrack_cleanup", "retro_clean", "apply", "apply_model",
    "apply_total", "num_ready",
}
SIC_STAGES = {
    "detect_coarse", "detect_fine", "cfo", "cir", "path_select",
    "fractional_refine", "matrix_build", "gain_fit", "reconstruct",
    "validate_commit", "core_total",
} | S1_SIC_STAGES
# Top-level demod stages are the ones the demod `total` must cover; the
# timing/SFD/CIR children and `total` itself are excluded so parents are never
# double-counted.
DEMOD_TOP_LEVEL = (DEMOD_STAGES - set(TIMING_CHILDREN) - set(SFD_CHILDREN)
                   - set(CIR_CHILDREN) - {"total"})
ADAPTER_STAGES = {
    "handler", "queue_wait", "output_wrap_publish", "consumer_service",
    "resample", "completion_latency", "thread_cpu_time",
}
OUTER_STAGES = {"generator", "disk", "plotting", "paced_wait"}
KNOWN_TIERS = {"cxx", "adapter", "outer"}
SCOPES = ("demod", "radar", "sic", "outer", "adapter")
KNOWN_STAGES = DEMOD_STAGES | SIC_STAGES | ADAPTER_STAGES | OUTER_STAGES
# Legal stage names per scope.  Radar carries `cir` plus its own (not fully
# enumerated) subfields, so any known timer name is accepted there.
SCOPE_STAGES = {
    "demod": DEMOD_STAGES,
    "sic": SIC_STAGES,
    "adapter": ADAPTER_STAGES,
    "outer": OUTER_STAGES | {"total"},
    "radar": KNOWN_STAGES,
}
SUCCESS_STATUSES = {"success", "ok", "pass"}
# Keys that are inapplicable (and therefore null) exactly when SIC is off.
SIC_CONDITIONAL_KEYS = {"sic", "sic_ok", "raw_ok_sic_fail", "raw_fail_sic_ok"}

TOL = 1e-9          # absolute tolerance on microseconds / hashes style checks
REL_TOL = 1e-6      # relative tolerance
SUPPRESSION_TOL_DB = 0.05
AMP_DB_TOL = 1e-6

# --- R01: actual-output cancellation (frozen measurement contract) ---------
# The main effect gate is the aggregate of the ACTUAL-output suppression
# recomputed from the archived arrays, not the reconstruction suppression_db.
ACTUAL_SUPPRESSION_MEDIAN_KEYS = (
    "median_actual_suppression_db", "actual_median_suppression_db",
    "median_suppression_db_actual", "actual_suppression_db_median")
ACTUAL_FRAC_KEYS = (
    "frac_actual_ge_3db", "actual_frac_ge_3db", "frac_ge_3db_actual",
    "frac_actual_suppression_ge_3db")
ACTUAL_MEDIAN_DB = 10.0
ACTUAL_FRAC = 0.95
ACTUAL_FLOOR_DB = 3.0
# A declared mask endpoint may trail the true changed support by a couple of
# samples (a subtraction that happens to land on an exact zero changes nothing);
# anything beyond this is a real smaller/larger-mask failure.
MASK_TOL_SAMPLES = 4
# A raw-CIR target metric may be re-declared to ~1e-3 dB without being wrong.
TARGET_AMP_TOL_DB = 0.05
TARGET_PHASE_TOL_DEG = 0.05
TARGET_GATE_AMP_DB = 1.0
TARGET_GATE_SHIFT = 1
NPZ_CIR_CLEAN_KEYS = ("cir_raw_clean", "cir_clean_raw", "cir_raw_ref",
                      "clean_cir_raw", "cir_clean")
NPZ_CIR_SIC_KEYS = ("cir_raw_sic", "cir_sic_raw", "cir_raw_out",
                    "sic_cir_raw", "cir_sic")


HEX64 = re.compile(r"^[0-9a-f]{64}$")

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))


def find_repo_root(start):
    """Walk up from `start` to the directory holding AGENTS.md or .git."""
    current = os.path.abspath(start)
    while True:
        if os.path.isfile(os.path.join(current, "AGENTS.md")) \
                or os.path.isdir(os.path.join(current, ".git")):
            return current
        parent = os.path.dirname(current)
        if parent == current:
            return None
        current = parent


REPO_ROOT = find_repo_root(REPO_ROOT) or REPO_ROOT

# ---------------------------------------------------------------------------
# Reporting
# ---------------------------------------------------------------------------

results = []
_QUIET = False
# When set to a dict, ``ok()`` accumulates per-item pass details instead of
# recording one line per window (used to keep large-run output readable; every
# failure is still recorded individually).
_OK_AGGREGATE = None
# True while a child run of an aggregate root (``cells/`` / ``cases/``) is being
# verified.  The results-root ``index.json`` is then owned by the aggregate
# root, not by the child, so ``_s1_index_path`` must not fall back to a
# sibling/global index.
_AGGREGATE_CHILD = False


def record(item, verdict, detail):
    results.append((item, verdict, detail))
    if not _QUIET:
        print("[%s] %s\n         %s" % (verdict, item, detail))


def _split_detail(rest):
    """Accept either (detail,) or (subject, detail) call shapes."""
    if len(rest) == 1:
        return rest[0], None
    if len(rest) == 2:
        return rest[1], rest[0]
    raise TypeError("expected (item, detail) or (item, subject, detail)")


def fail(item, *rest):
    detail, subject = _split_detail(rest)
    if subject is not None:
        item = "%s: %s" % (item, subject)
    record(item, "FAIL", detail)


def ok(item, *rest):
    detail, subject = _split_detail(rest)
    if subject is not None:
        item = "%s: %s" % (item, subject)
    if _OK_AGGREGATE is not None:
        _OK_AGGREGATE.setdefault(item, []).append(detail)
        return
    record(item, "PASS", detail)


def skip(item, *rest):
    detail, subject = _split_detail(rest)
    if subject is not None:
        item = "%s: %s" % (item, subject)
    record(item, "SKIP", detail)


# ---------------------------------------------------------------------------
# Small helpers
# ---------------------------------------------------------------------------


def is_int(value):
    return isinstance(value, int) and not isinstance(value, bool)


def is_num(value):
    return (isinstance(value, (int, float)) and not isinstance(value, bool)
            and math.isfinite(float(value)))


def is_nonempty_str(value):
    return isinstance(value, str) and value.strip() != ""


def is_hex64(value):
    return isinstance(value, str) and HEX64.match(value.lower()) is not None


def norm_text(text):
    return re.sub(r"[^a-z0-9]+", " ", str(text).lower()).strip()


def close(a, b, abs_tol=TOL, rel_tol=REL_TOL):
    if a is None or b is None:
        return False
    return abs(float(a) - float(b)) <= max(abs_tol, rel_tol * abs(float(b)))


def load_json(path):
    try:
        with open(path, "r", encoding="utf-8") as handle:
            return json.load(handle), None
    except (ValueError, OSError) as exc:
        return None, str(exc)


def load_jsonl(path):
    rows = []
    try:
        with open(path, "r", encoding="utf-8") as handle:
            for lineno, line in enumerate(handle, 1):
                if line.strip() == "":
                    continue
                try:
                    rows.append((lineno, json.loads(line)))
                except ValueError as exc:
                    return None, "line %d: %s" % (lineno, exc)
        return rows, None
    except OSError as exc:
        return None, str(exc)


def sha256_file(path):
    digest = hashlib.sha256()
    with open(path, "rb") as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def normalize_hash(value):
    if not isinstance(value, str):
        return None
    text = value.strip().lower()
    if text.startswith("sha256:"):
        text = text.split(":", 1)[1]
    return text


def resolve_referenced(rel, roots):
    if not isinstance(rel, str) or rel.strip() == "":
        return None
    if os.path.isabs(rel):
        return rel if os.path.isfile(rel) else None
    for root in roots:
        candidate = os.path.join(root, rel)
        if os.path.isfile(candidate):
            return candidate
    # Allow paths relative to the repository root even when invoked elsewhere.
    candidate = os.path.join(REPO_ROOT, rel)
    if os.path.isfile(candidate):
        return candidate
    return None


def resolve_hash_target(raw_key, roots):
    """Resolve a fixture_hashes key: `repo:` -> repo root, else results dir."""
    if isinstance(raw_key, str) and raw_key.startswith("repo:"):
        rel = raw_key[len("repo:"):]
        candidate = os.path.join(REPO_ROOT, rel)
        return candidate if os.path.isfile(candidate) else None
    return resolve_referenced(raw_key, roots)


def window_key(value):
    return str(value)


def parse_us(text):
    """Return float us, or None for an empty (not_run) cell; raise ValueError."""
    text = ("" if text is None else str(text)).strip()
    if text == "":
        return None
    value = float(text)
    if not math.isfinite(value) or value < 0.0:
        raise ValueError("us=%r is not a finite non-negative number" % (text,))
    return value


def percentile(values, q):
    if not values:
        return None
    if np is not None:
        return float(np.percentile(np.asarray(values, dtype=float), q))
    ordered = sorted(values)
    rank = (len(ordered) - 1) * (q / 100.0)
    lo = int(math.floor(rank))
    hi = int(math.ceil(rank))
    if lo == hi:
        return float(ordered[lo])
    frac = rank - lo
    return float(ordered[lo] * (1 - frac) + ordered[hi] * frac)


def median(values):
    return percentile(values, 50)


def label_for(run_dir, results_dir):
    rel = os.path.relpath(run_dir, results_dir)
    return "." if rel == "." else rel


# ---------------------------------------------------------------------------
# 1. run_manifest.json
# ---------------------------------------------------------------------------


def check_manifest(run_dir, results_dir, prefix):
    item = "%s/run_manifest.json" % prefix
    path = os.path.join(run_dir, "run_manifest.json")
    manifest, err = load_json(path)
    if manifest is None:
        fail("%s: present and well-formed JSON" % item, err or "missing")
        return None
    if not isinstance(manifest, dict):
        fail("%s: is a JSON object" % item, "got %r" % type(manifest).__name__)
        return None
    ok("%s: present and well-formed JSON" % item, "object with %d keys" % len(manifest))

    for key in MANIFEST_KEYS:
        if key not in manifest:
            fail("%s: <key>" % item, "missing required key %r" % key)
    # Non-empty provenance fields (S0 §10 / task check 1).
    for key in ("commit", "build_type", "compiler", "cpu", "os",
                "gnuradio_version", "volk_version"):
        if is_nonempty_str(manifest.get(key)):
            ok("%s: %s is a non-empty string" % (item, key),
               manifest[key][:100])
        else:
            fail("%s: %s is a non-empty string" % (item, key),
                 "got %r" % (manifest.get(key),))
    if isinstance(manifest.get("dirty"), bool):
        ok("%s: dirty is a boolean" % item, str(manifest.get("dirty")))
    else:
        fail("%s: dirty is a boolean" % item, "got %r" % (manifest.get("dirty"),))
    flags = manifest.get("flags")
    if isinstance(flags, list) and all(isinstance(f, str) for f in flags):
        ok("%s: flags is a list of strings" % item, "%d flags" % len(flags))
    else:
        fail("%s: flags is a list of strings" % item, "got %r" % (flags,))
    libs = manifest.get("lib_paths")
    if isinstance(libs, dict) and is_nonempty_str(libs.get("gnuradio_uwb")) \
            and isinstance(libs.get("loaded"), list) and libs["loaded"] \
            and all(is_nonempty_str(x) for x in libs["loaded"]):
        ok("%s: lib_paths carries the loaded libraries" % item,
           "%d entries; gnuradio_uwb=%s" % (len(libs["loaded"]),
                                            libs["gnuradio_uwb"][:60]))
    else:
        fail("%s: lib_paths{gnuradio_uwb, loaded[]} is non-empty" % item,
             "got %r" % (libs,))
    seeds = manifest.get("seed_lists")
    if isinstance(seeds, dict):
        cal, acc = seeds.get("calibration"), seeds.get("acceptance")
        good = (isinstance(cal, list) and cal
                and all(is_int(v) for v in cal)
                and isinstance(acc, list) and acc
                and all(is_int(v) for v in acc))
        if good and not (set(cal) & set(acc)):
            ok("%s: seed_lists calibration/acceptance are disjoint" % item,
               "%d + %d seeds" % (len(cal), len(acc)))
        elif good:
            fail("%s: seed_lists calibration/acceptance are disjoint" % item,
                 "they intersect (%d shared)" % len(set(cal) & set(acc)))
        else:
            fail("%s: seed_lists carries calibration and acceptance int lists" % item,
                 "got %r" % (seeds,))
    else:
        fail("%s: seed_lists is an object" % item, "got %r" % (seeds,))

    if manifest.get("sample_rate_work") == WORK_HZ:
        ok("%s: sample_rate_work is the frozen 998.4 MS/s work grid" % item,
           str(WORK_HZ))
    else:
        fail("%s: sample_rate_work is the frozen 998.4 MS/s work grid" % item,
             "got %r" % (manifest.get("sample_rate_work"),))
    if manifest.get("sample_rate_native") in NATIVE_HZ:
        ok("%s: sample_rate_native is a known CG400 native rate" % item,
           str(manifest.get("sample_rate_native")))
    else:
        fail("%s: sample_rate_native is a known CG400 native rate" % item,
             "got %r" % (manifest.get("sample_rate_native"),))
    if manifest.get("input_grid") in INPUT_GRIDS:
        ok("%s: input_grid is work|cg400" % item, str(manifest.get("input_grid")))
    else:
        fail("%s: input_grid is work|cg400" % item,
             "got %r" % (manifest.get("input_grid"),))
    if manifest.get("preset") in PRESETS:
        ok("%s: preset is a frozen preset" % item, str(manifest.get("preset")))
    else:
        fail("%s: preset is a frozen preset" % item,
             "got %r" % (manifest.get("preset"),))
    if manifest.get("sic_mode") in SIC_MODES:
        ok("%s: sic_mode is off|estimate|oracle" % item,
           str(manifest.get("sic_mode")))
    else:
        fail("%s: sic_mode is off|estimate|oracle" % item,
             "got %r" % (manifest.get("sic_mode"),))

    check_manifest_hashes(manifest, run_dir, results_dir, item)
    return manifest


def check_manifest_hashes(manifest, run_dir, results_dir, item):
    roots = [run_dir, results_dir, REPO_ROOT, os.getcwd()]
    config_path = manifest.get("config_path")
    config_hash = manifest.get("config_hash")
    if not is_nonempty_str(config_path):
        fail("%s: config_path is a non-empty string" % item,
             "got %r" % (config_path,))
    if not is_hex64(config_hash):
        fail("%s: config_hash is a 64-char sha256 hex" % item,
             "got %r" % (config_hash,))
    else:
        ok("%s: config_hash is a 64-char sha256 hex" % item,
           normalize_hash(config_hash)[:16] + "...")
    if is_nonempty_str(config_path):
        resolved = resolve_referenced(config_path, roots)
        if resolved is None:
            fail("%s: config_path[%s] exists" % (item, config_path),
                 "not found under %s" % ", ".join(roots[:2]))
        else:
            try:
                with open(resolved, "r", encoding="utf-8") as handle:
                    json.load(handle)
                ok("%s: config_path points to valid JSON" % item, config_path)
            except (OSError, ValueError) as exc:
                fail("%s: config_path points to valid JSON" % item, str(exc))
            # Rev 2 lists config_path/config_hash but does not freeze the
            # config_hash derivation (the producer hashes the CLI-resolved
            # config object, not the file bytes), so it cannot be recomputed
            # from the artifact alone; equality is reported only when it holds.
            if sha256_file(resolved) == normalize_hash(config_hash):
                ok("%s: config_hash equals the config file sha256" % item,
                   config_path)
            else:
                ok("%s: config_hash is the derived resolved-config hash"
                   % item,
                   "not the raw file sha256; rev 2 does not freeze its "
                   "canonical form, file content is still validated")

    fixture_hashes = manifest.get("fixture_hashes")
    if not isinstance(fixture_hashes, dict):
        fail("%s: fixture_hashes is a path->sha256 object" % item,
             "got %r" % (fixture_hashes,))
        fixture_hashes = {}
    checked = 0
    for raw_key, declared in fixture_hashes.items():
        want = normalize_hash(declared)
        if want is None or not is_hex64(want):
            fail("%s: fixture_hashes[%s] is a sha256 hex" % (item, raw_key),
                 "got %r" % (declared,))
            continue
        resolved = resolve_hash_target(raw_key, roots)
        if resolved is None:
            fail("%s: fixture_hashes[%s] referenced file exists" % (item, raw_key),
                 "not found (repo: resolves against %s)" % REPO_ROOT)
            continue
        got = sha256_file(resolved)
        if got == want:
            checked += 1
        else:
            fail("%s: fixture_hashes[%s] re-hashed equals the manifest"
                 % (item, raw_key), "on disk=%s manifest=%s" % (got, want))
    if fixture_hashes and checked == len(fixture_hashes):
        ok("%s: every fixture re-hashes to the manifest" % item,
           "%d file(s) byte-exact" % checked)

    template_hashes = manifest.get("template_hashes")
    if not isinstance(template_hashes, dict):
        fail("%s: template_hashes is a template_id->sha256 object" % item,
             "got %r" % (template_hashes,))
        template_hashes = {}
    else:
        for template_id, value in template_hashes.items():
            if is_int(value):
                fail("%s: template_hashes[%s] is a sha256 hex, not a length int"
                     % (item, template_id),
                     "got %r (lengths belong in template_meta)" % (value,))
            elif not is_hex64(value):
                fail("%s: template_hashes[%s] is a 64-char sha256 hex"
                     % (item, template_id), "got %r" % (value,))
        if template_hashes and all(is_hex64(v) for v in template_hashes.values()):
            ok("%s: template_hashes values are sha256 hex only" % item,
               "%d template(s)" % len(template_hashes))

    template_meta = manifest.get("template_meta")
    if not isinstance(template_meta, dict):
        fail("%s: template_meta is a template_id->object map" % item,
             "got %r" % (template_meta,))
        return
    for template_id, meta in template_meta.items():
        if not isinstance(meta, dict):
            fail("%s: template_meta[%s] is an object" % (item, template_id),
                 "got %r" % (meta,))
            continue
        if not is_int(meta.get("length")) or meta["length"] < 0:
            fail("%s: template_meta[%s].length is a non-negative integer"
                 % (item, template_id), "got %r" % (meta.get("length"),))
        if not is_nonempty_str(meta.get("source")):
            fail("%s: template_meta[%s].source is a non-empty string"
                 % (item, template_id), "got %r" % (meta.get("source"),))
        if not is_hex64(meta.get("sha256")):
            fail("%s: template_meta[%s].sha256 is a sha256 hex"
                 % (item, template_id), "got %r" % (meta.get("sha256"),))
        elif template_id in template_hashes \
                and normalize_hash(meta["sha256"]) \
                != normalize_hash(template_hashes[template_id]):
            fail("%s: template_meta[%s].sha256 equals template_hashes[%s]"
                 % (item, template_id, template_id),
                 "meta=%s hashes=%s" % (meta["sha256"],
                                        template_hashes[template_id]))
        # Lenient length check: length is the waveform length in CF32 samples or
        # in bytes; accept either against the source file size.
        source = resolve_referenced(meta.get("source"), roots) \
            if is_nonempty_str(meta.get("source")) else None
        if source is not None and is_int(meta.get("length")):
            size = os.path.getsize(source)
            if meta["length"] not in (size, size // 8):
                fail("%s: template_meta[%s].length matches the source size"
                     % (item, template_id),
                     "length=%r source bytes=%d" % (meta["length"], size))
    missing = [tid for tid in template_hashes if tid not in template_meta]
    if missing:
        fail("%s: every template_hashes id has template_meta" % item,
             "missing %s" % missing)
    elif template_meta and all(is_hex64(v) for v in template_hashes.values()):
        ok("%s: template_meta sha256 agrees with template_hashes" % item,
           "%d template(s)" % len(template_meta))


# ---------------------------------------------------------------------------
# 2 & 3. windows.jsonl and three-way pairing
# ---------------------------------------------------------------------------


def check_windows(run_dir, manifest, summary, prefix):
    item = "%s/windows.jsonl" % prefix
    path = os.path.join(run_dir, "windows.jsonl")
    if not os.path.isfile(path):
        fail("%s: present" % item, "missing required artifact")
        return None, None
    rows, err = load_jsonl(path)
    if rows is None:
        fail("%s: every line parses as JSON" % item, err)
        return None, None
    if not rows:
        fail("%s: contains at least one window" % item, "empty file")
        return None, None
    ok("%s: every line parses as JSON" % item, "%d windows" % len(rows))

    sic_mode = (manifest or {}).get("sic_mode")
    if sic_mode is None and isinstance(summary, dict):
        sic_mode = summary.get("sic_mode")
    preset = (manifest or {}).get("preset")
    if preset is None and isinstance(summary, dict):
        preset = summary.get("preset")
    grid = (manifest or {}).get("input_grid")
    if grid is None and isinstance(summary, dict):
        grid = summary.get("input_grid")

    windows = []
    seen = {}
    by_id = {}
    run_ids = set()
    dup = False
    for lineno, window in rows:
        where = "%s line %d" % (item, lineno)
        if not isinstance(window, dict):
            fail("%s: is a JSON object" % where, "got %r" % type(window).__name__)
            continue
        for key in WINDOW_KEYS:
            if key not in window:
                fail("%s: <field>" % where, "missing required field %r" % key)
        if not is_nonempty_str(window.get("run_id")):
            fail("%s: run_id is a non-empty string" % where,
                 "got %r" % (window.get("run_id"),))
        else:
            run_ids.add(window["run_id"])
        if "window_id" not in window:
            fail("%s: window_id is present" % where, "missing")
            continue
        key = window_key(window["window_id"])
        if key in seen:
            dup = True
            fail("%s: window_id is unique" % where,
                 "duplicate window_id=%r (first at line %d)"
                 % (window["window_id"], seen[key]))
        else:
            seen[key] = lineno
            by_id[key] = window
            windows.append(window)

    if len(run_ids) > 1:
        fail("%s: a single run_id across all windows" % item,
             "got %s" % sorted(run_ids))
    elif run_ids:
        run_id = next(iter(run_ids))
        for source, value in (("run_manifest.json", (manifest or {}).get("run_id")),
                              ("summary.json", (summary or {}).get("run_id"))):
            if value is not None and value != run_id:
                fail("%s: run_id matches %s" % (item, source),
                     "windows=%r %s=%r" % (run_id, source, value))
    if not dup:
        ok("%s: window_id is unique" % item, "%d distinct" % len(seen))

    # Per-window field semantics.
    declared_bounds_ok = True
    for window in windows:
        wid = window.get("window_id")
        where = "%s window_id=%r" % (item, wid)
        check_window_body(window, where, sic_mode, preset, grid)

    # window-id set vs summary counts.
    if isinstance(summary, dict):
        expected = summary.get("windows_expected")
        completed = summary.get("windows_completed")
        n = len(seen)
        if is_int(completed) and completed == n:
            ok("%s: window count matches summary.windows_completed" % item,
               "%d" % n)
        else:
            fail("%s: window count matches summary.windows_completed" % item,
                 "on disk=%d summary=%r" % (n, completed))
        if is_int(expected):
            if expected >= n:
                ok("%s: summary.windows_expected >= the window count" % item,
                   "%d >= %d" % (expected, n))
            else:
                fail("%s: summary.windows_expected >= the window count" % item,
                     "expected=%d on disk=%d" % (expected, n))
        else:
            fail("%s: summary.windows_expected is an integer" % item,
                 "got %r" % (expected,))
    return windows, by_id


def check_window_body(window, where, sic_mode, preset, grid):
    if not is_int(window.get("schema_version")):
        fail("%s: schema_version is an integer" % where,
             "got %r" % (window.get("schema_version"),))
    sample_count = window.get("sample_count")
    if is_int(sample_count) and sample_count > 0:
        ok("%s: sample_count is a positive integer" % where, str(sample_count))
    else:
        fail("%s: sample_count is a positive integer" % where,
             "got %r" % (sample_count,))
        sample_count = None
    start = window.get("window_start_sample")
    if is_int(start) and start >= 0:
        ok("%s: window_start_sample is a non-negative integer" % where, str(start))
    else:
        fail("%s: window_start_sample is a non-negative integer" % where,
             "got %r" % (start,))
    rate = window.get("sample_rate")
    if is_num(rate) and float(rate) in (WORK_HZ,) + NATIVE_HZ:
        ok("%s: sample_rate is a known grid rate" % where, str(rate))
    else:
        fail("%s: sample_rate is a known grid rate" % where, "got %r" % (rate,))
    pred = window.get("predicted_sensing_start")
    if is_num(pred) and float(pred) >= 0:
        ok("%s: predicted_sensing_start is a non-negative number" % where,
           str(pred))
    else:
        fail("%s: predicted_sensing_start is a non-negative number" % where,
             "got %r" % (pred,))
    for key in ("sensing_profile_id", "template_id"):
        if is_nonempty_str(window.get(key)):
            ok("%s: %s is a non-empty string" % (where, key), window[key][:80])
        else:
            fail("%s: %s is a non-empty string" % (where, key),
                 "got %r" % (window.get(key),))
    sensing_code = window.get("sensing_code")
    comm_code = window.get("comm_code")
    if sensing_code in CODES:
        ok("%s: sensing_code is in 9..12" % where, str(sensing_code))
    else:
        fail("%s: sensing_code is in 9..12" % where, "got %r" % (sensing_code,))
    if comm_code in CODES:
        ok("%s: comm_code is in 9..12" % where, str(comm_code))
    else:
        fail("%s: comm_code is in 9..12" % where, "got %r" % (comm_code,))
    if sensing_code in CODES and sensing_code == comm_code:
        if window.get("diagnostic_preset") is True:
            ok("%s: sensing_code == comm_code is an explicit diagnostic preset"
               % where, "diagnostic_preset=true")
        else:
            fail("%s: sensing_code != comm_code outside a diagnostic preset" % where,
                 "both are %r with no diagnostic_preset" % (sensing_code,))
    if window.get("sensing_sync_reps") in SYNC_REPS:
        ok("%s: sensing_sync_reps is 16/32/64/128" % where,
           str(window.get("sensing_sync_reps")))
    else:
        fail("%s: sensing_sync_reps is 16/32/64/128" % where,
             "got %r" % (window.get("sensing_sync_reps"),))
    if window.get("sensing_sfd_mode") in SFD_MODES:
        ok("%s: sensing_sfd_mode is 4z2|ieee" % where,
           str(window.get("sensing_sfd_mode")))
    else:
        fail("%s: sensing_sfd_mode is 4z2|ieee" % where,
             "got %r" % (window.get("sensing_sfd_mode"),))
    if window.get("demod_softchip_mode") in SOFTCHIP_MODES:
        ok("%s: demod_softchip_mode is bypass|full|rake4" % where,
           str(window.get("demod_softchip_mode")))
    else:
        fail("%s: demod_softchip_mode is bypass|full|rake4" % where,
             "got %r" % (window.get("demod_softchip_mode"),))
    sic_k = window.get("sic_k")
    off = sic_mode == "off"
    if off:
        if is_int(sic_k) and sic_k == 0:
            ok("%s: sic_k == 0 when SIC is off" % where, "0")
        else:
            fail("%s: sic_k == 0 when SIC is off" % where, "got %r" % (sic_k,))
    elif sic_k in SIC_KS:
        ok("%s: sic_k is 1/2/4/8/16" % where, str(sic_k))
    else:
        fail("%s: sic_k is 1/2/4/8/16" % where, "got %r" % (sic_k,))
    for key in ("fit_reps", "validation_reps"):
        value = window.get(key)
        # S1 default: online_validation_reps == 0 (evaluation is offline and
        # separate).  Zero is legal only when the window declares the S1 model
        # mode and a separate independent-evaluation span >= 8 SYNC.
        s1_zero = (key == "validation_reps" and is_int(value) and value == 0
                   and isinstance(window.get("sic_model"), dict)
                   and is_int(window.get("evaluation_reps"))
                   and int(window["evaluation_reps"]) >= 8)
        if off:
            if value is None:
                ok("%s: %s is null when SIC is off" % (where, key), "null")
            else:
                fail("%s: %s is null when SIC is off" % (where, key),
                     "got %r (rev 2 forbids a fake positive value)" % (value,))
        elif is_int(value) and value > 0:
            ok("%s: %s is a positive integer" % (where, key), str(value))
        elif s1_zero:
            ok("%s: validation_reps == 0 is the S1 online-validation default"
               % where, "evaluation_reps=%r" % (window.get("evaluation_reps"),))
        else:
            fail("%s: %s is a positive integer" % (where, key),
                 "got %r" % (value,))
    if not off and is_int(window.get("validation_reps")) \
            and window["validation_reps"] < 8 \
            and not (window["validation_reps"] == 0
                     and isinstance(window.get("sic_model"), dict)
                     and is_int(window.get("evaluation_reps"))
                     and int(window["evaluation_reps"]) >= 8):
        fail("%s: validation_reps >= 8 when SIC is on" % where,
             str(window["validation_reps"]))
    roi = window.get("roi")
    if off:
        if roi is None:
            ok("%s: roi is null when SIC is off" % where, "null")
        else:
            fail("%s: roi is null when SIC is off" % where, "got %r" % (roi,))
    elif isinstance(roi, (list, tuple)) and len(roi) == 2 \
            and all(is_int(v) for v in roi) and 0 <= roi[0] < roi[1]:
        ok("%s: roi is a non-empty [lo,hi) pair" % where, str(list(roi)))
    else:
        fail("%s: roi is a non-empty [lo,hi) pair" % where, "got %r" % (roi,))

    # isr_db may be null (no overlap) but the key must exist.
    if "isr_db" not in window:
        fail("%s: isr_db present (nullable)" % where, "missing")
    elif window["isr_db"] is None or is_num(window["isr_db"]):
        ok("%s: isr_db is a number or null" % where, str(window["isr_db"]))
    else:
        fail("%s: isr_db is a number or null" % where,
             "got %r" % (window["isr_db"],))
    if is_num(window.get("snr_db")):
        ok("%s: snr_db is a number" % where, str(window.get("snr_db")))
    else:
        fail("%s: snr_db is a number" % where, "got %r" % (window.get("snr_db"),))

    # overlap [lo,hi)
    overlap = window.get("overlap")
    if not isinstance(overlap, dict):
        fail("%s: overlap is an object" % where, "got %r" % (overlap,))
    else:
        lo, hi, samples = overlap.get("lo"), overlap.get("hi"), overlap.get("samples")
        if is_int(lo) and is_int(hi) and 0 <= lo <= hi:
            ok("%s: overlap is a well-formed [lo,hi)" % where, "[%d,%d)" % (lo, hi))
            if samples == hi - lo:
                ok("%s: overlap.samples == hi-lo" % where, str(samples))
            else:
                fail("%s: overlap.samples == hi-lo" % where,
                     "samples=%r hi-lo=%d" % (samples, hi - lo))
            if sample_count is not None:
                in_relative = hi <= sample_count
                start_ok = is_int(start)
                in_absolute = (start_ok
                               and start <= lo and hi <= start + sample_count)
                if in_relative or in_absolute:
                    ok("%s: overlap lies within the declared window bounds" % where,
                       "[%d,%d) of sample_count=%d" % (lo, hi, sample_count))
                else:
                    fail("%s: overlap lies within the declared window bounds" % where,
                         "[%d,%d) outside [0,%d) or [%d,%d)"
                         % (lo, hi, sample_count, start if is_int(start) else -1,
                            (start + sample_count) if is_int(start) else -1))
        else:
            fail("%s: overlap is a well-formed [lo,hi)" % where,
                 "got lo=%r hi=%r" % (lo, hi))

    # consumers / three-way pairing
    if sic_mode == "off":
        if "sic" in window:
            fail("%s: 'sic' is absent when sic_mode is off" % where,
                 "sic block present")
        else:
            ok("%s: 'sic' is absent when sic_mode is off" % where, "absent")
    else:
        if "sic" not in window:
            fail("%s: 'sic' consumer is present when sic_mode != off" % where,
                 "window lacks the declared consumer 'sic'")
        elif not isinstance(window["sic"], dict):
            fail("%s: 'sic' consumer is an object" % where,
                 "got %r" % type(window["sic"]).__name__)
        else:
            ok("%s: 'sic' consumer is present" % where, "declared")
    for consumer in ("clean", "raw"):
        block = window.get(consumer)
        if not isinstance(block, dict):
            fail("%s: '%s' consumer is present" % (where, consumer),
                 "got %r" % (block,))
        elif not is_nonempty_str(block.get("status")):
            fail("%s: '%s'.status is a non-empty string" % (where, consumer),
                 "got %r" % (block.get("status"),))
        else:
            ok("%s: '%s' consumer terminal status" % (where, consumer),
               block["status"])

    # sic_state
    state = window.get("sic_state")
    if not isinstance(state, dict):
        fail("%s: sic_state is an object" % where, "got %r" % (state,))
    else:
        for key in ("detected", "committed", "partial", "ambiguous", "uses_truth"):
            if not isinstance(state.get(key), bool):
                fail("%s: sic_state.%s is a boolean" % (where, key),
                     "got %r" % (state.get(key),))
        if "bypass_reason" not in state:
            fail("%s: sic_state.bypass_reason is present" % where, "missing")
        elif state["bypass_reason"] is None or isinstance(state["bypass_reason"], str):
            ok("%s: sic_state.bypass_reason is a string or null" % where,
               repr(state.get("bypass_reason"))[:60])
        else:
            fail("%s: sic_state.bypass_reason is a string or null" % where,
                 "got %r" % (state.get("bypass_reason"),))

    # admission
    admission = window.get("admission")
    if not isinstance(admission, dict):
        fail("%s: admission is an object" % where, "got %r" % (admission,))
    else:
        for key in ("published_ns", "admitted_ns", "completion_ns"):
            if not is_int(admission.get(key)):
                fail("%s: admission.%s is an integer" % (where, key),
                     "got %r" % (admission.get(key),))
        if not isinstance(admission.get("missed_admission"), bool):
            fail("%s: admission.missed_admission is a boolean" % where,
                 "got %r" % (admission.get("missed_admission"),))
        vals = [admission.get(k) for k in
                ("published_ns", "admitted_ns", "completion_ns")]
        if all(is_int(v) for v in vals):
            if vals[0] <= vals[1] <= vals[2]:
                ok("%s: admission timestamps are monotonic" % where, "ok")
            else:
                fail("%s: admission timestamps are monotonic" % where,
                     "published=%d admitted=%d completion=%d" % tuple(vals))
            if admission.get("missed_admission") is True and vals[1] < vals[0]:
                ok("%s: missed admission is flagged with a late admit" % where, "ok")

    # recon (nullable iff SIC is off)
    recon = window.get("recon")
    if off:
        if recon is None:
            ok("%s: recon is null when SIC is off" % where, "null")
        else:
            fail("%s: recon is null when SIC is off" % where,
                 "got %r (not a fake value)" % (recon,))
    elif not isinstance(recon, dict):
        fail("%s: recon is an object when SIC is on" % where, "got %r" % (recon,))
    else:
        for key in ("nmse", "suppression_db", "eval_lo", "eval_hi",
                    "holdout_nmse", "holdout_suppression_db"):
            if key not in recon:
                fail("%s: recon.%s is present" % (where, key), "missing")

    # native coordinate mapping when the grid is cg400
    if grid == "cg400":
        coord = window.get("coord")
        if not isinstance(coord, dict):
            fail("%s: coord is present for the cg400 grid" % where,
                 "got %r" % (coord,))
        else:
            missing = [k for k in ("work_to_native", "native_to_work",
                                   "filter_delay", "valid_lo", "valid_hi")
                       if k not in coord]
            if missing:
                fail("%s: coord carries the native/work mapping" % where,
                     "missing %s" % missing)
            else:
                ok("%s: coord carries the native/work mapping" % where,
                   "valid=[%r,%r)" % (coord.get("valid_lo"), coord.get("valid_hi")))
                if not (is_int(coord.get("valid_lo")) and is_int(coord.get("valid_hi"))
                        and coord["valid_lo"] < coord["valid_hi"]):
                    fail("%s: coord.valid range is non-empty" % where,
                         "got [%r,%r)" % (coord.get("valid_lo"), coord.get("valid_hi")))


def check_baseline_pairing(windows, preset, sic_mode, prefix):
    if preset != "baseline" or sic_mode not in ("off", None):
        return
    item = "%s/windows.jsonl baseline raw==clean" % prefix
    verdict_keys = ("status", "fcs_pass", "psdu_ok", "psdu_hex", "cir_peak_tap")
    for window in windows:
        wid = window.get("window_id")
        clean, raw = window.get("clean"), window.get("raw")
        if not (isinstance(clean, dict) and isinstance(raw, dict)):
            continue
        where = "%s window_id=%r" % (item, wid)
        # (a) exact posted-IQ identity: the generator makes y == s + w bitwise.
        missing = [k for k in ("input_sha256", "input_samples")
                   if k not in clean or k not in raw]
        if missing:
            fail("%s: clean/raw carry the posted-IQ identity fields" % where,
                 "missing %s" % missing)
        a, b = clean.get("input_sha256"), raw.get("input_sha256")
        if is_nonempty_str(a) and a == b:
            ok("%s: clean.input_sha256 == raw.input_sha256" % where,
               a[:16] + "...")
        else:
            fail("%s: clean.input_sha256 == raw.input_sha256" % where,
                 "clean=%r raw=%r" % (a, b))
        sa, sb = clean.get("input_samples"), raw.get("input_samples")
        if is_int(sa) and sa == sb:
            ok("%s: clean.input_samples == raw.input_samples" % where, str(sa))
        else:
            fail("%s: clean.input_samples == raw.input_samples" % where,
                 "clean=%r raw=%r" % (sa, sb))
        # (b) exact signal-verdict subset (all timing fields excluded).
        diffs = ["%s clean=%r raw=%r" % (key, clean.get(key), raw.get(key))
                 for key in verdict_keys if clean.get(key) != raw.get(key)]
        if diffs:
            fail("%s: raw == clean on the signal-verdict subset" % where,
                 "; ".join(diffs))
        else:
            ok("%s: raw == clean on the signal-verdict subset" % where,
               "/".join(verdict_keys))


# ---------------------------------------------------------------------------
# 4. stage_times.csv
# ---------------------------------------------------------------------------


def check_stage_times(run_dir, windows_by_id, prefix):
    item = "%s/stage_times.csv" % prefix
    path = os.path.join(run_dir, "stage_times.csv")
    if not os.path.isfile(path):
        fail("%s: present" % item, "missing required artifact")
        return None
    try:
        with open(path, "r", encoding="utf-8", newline="") as handle:
            reader = csv.reader(handle)
            header = next(reader, None)
            raw_rows = [row for row in reader if any(c.strip() for c in row)]
    except (OSError, ValueError) as exc:
        fail("%s: readable CSV" % item, str(exc))
        return None
    want_header = ["run_id", "window_id", "consumer", "scope", "tier",
                   "stage", "us", "detail"]
    if [h.strip() for h in (header or [])] != want_header:
        fail("%s: header is the frozen rev-2 long form" % item,
             "got %r want %r" % (header, want_header))
        return None
    ok("%s: header is the frozen rev-2 long form" % item,
       ",".join(want_header))

    rows = []
    for index, row in enumerate(raw_rows, 2):
        if len(row) != 8:
            fail("%s: line %d has 8 columns" % (item, index),
                 "got %d" % len(row))
            continue
        (run_id, window_id, consumer, scope, tier, stage, us,
         detail) = [c.strip() for c in row]
        rows.append({
            "line": index, "run_id": run_id, "window_id": window_id,
            "consumer": consumer, "scope": scope, "tier": tier,
            "stage": stage, "us_text": us, "detail": detail,
        })

    # structural validation + us/not_run contract
    groups = {}
    for row in rows:
        where = "%s line %d" % (item, row["line"])
        if row["consumer"] not in CONSUMERS:
            fail("%s: consumer is clean|raw|sic" % where, row["consumer"])
        if row["scope"] not in SCOPES:
            fail("%s: scope is demod|radar|sic|outer|adapter" % where,
                 row["scope"])
        elif row["scope"] != "radar" \
                and row["stage"] not in SCOPE_STAGES.get(row["scope"], set()):
            fail("%s: stage is legal for scope %s" % (where, row["scope"]),
                 "%r not in %s" % (row["stage"],
                                   sorted(SCOPE_STAGES.get(row["scope"], set()))))
        # radar has `cir` plus its own (not enumerated) subfields.
        if row["scope"] != "radar" and row["stage"] not in KNOWN_STAGES:
            fail("%s: stage is a known timer name" % where, row["stage"])
        if row["tier"] not in KNOWN_TIERS:
            fail("%s: tier is cxx|adapter|outer" % where, row["tier"])
        if row["window_id"] not in windows_by_id:
            fail("%s: window_id is declared in windows.jsonl" % where,
                 "unknown window_id=%r" % row["window_id"])
        try:
            us = parse_us(row["us_text"])
        except ValueError as exc:
            fail("%s: us is empty or a non-negative number" % where, str(exc))
            us = None
        row["us"] = us
        not_run = norm_text("not_run") in norm_text(row["detail"])
        if us is None and not not_run:
            fail("%s: an empty us is marked not_run" % where,
                 "detail=%r" % row["detail"])
        elif us is not None and not_run:
            fail("%s: a not_run stage has an empty us" % where,
                 "us=%s detail=%r" % (row["us_text"], row["detail"]))
        groups.setdefault((row["run_id"], row["window_id"], row["consumer"]),
                          []).append(row)

    stats = {}
    for (run_id, window_id, consumer), group in groups.items():
        where = "%s (run=%s window=%s consumer=%s)" % (item, run_id,
                                                       window_id, consumer)
        resolved = resolve_group_rows(group, where)
        check_group_timers(resolved, where, window_id, consumer, windows_by_id)
        # The summary cross-check uses the demod scope only: `cfo`/`cir` also
        # exist in the radar and SIC scopes and would otherwise be ambiguous.
        for row in resolved["demod"]:
            if row["us"] is not None:
                stats.setdefault(consumer, {}).setdefault(
                    row["stage"], []).append(row["us"])

    # consumers present in the CSV must be declared in windows.jsonl
    for (run_id, window_id, consumer) in groups:
        window = windows_by_id.get(window_id)
        if window is None:
            continue
        if consumer not in window:
            fail("%s: consumer %s is declared in windows.jsonl window_id=%r"
                 % (item, consumer, window_id), "missing consumer block")
    return stats


def resolve_group_rows(group, where):
    """Split a (run,window,consumer) group by the rev-2 `scope` column.

    `scope` disambiguates the `cfo`/`cir` names that exist in the demod, radar
    and SIC paths; the verifier trusts the column and does not parse `detail`.
    """
    buckets = {"demod": [], "radar": [], "sic": [], "adapter": [], "outer": []}
    seen = {}
    for row in group:
        scope = row["scope"]
        if scope not in buckets:
            continue
        if scope in ("demod", "sic"):
            key = (scope, row["stage"])
            if key in seen:
                fail("%s: stage %r is unique within scope %r"
                     % (where, row["stage"], scope),
                     "duplicate row (line %d and line %d)"
                     % (seen[key], row["line"]))
                continue
            seen[key] = row["line"]
        buckets[scope].append(row)
    allrows = [r for scope in SCOPES for r in buckets.get(scope, [])]
    return {"demod": buckets["demod"], "radar": buckets["radar"],
            "sic": buckets["sic"], "adapter": buckets["adapter"],
            "outer": buckets["outer"], "all": allrows}


def _us_map(rows):
    return {row["stage"]: row for row in rows if row["stage"]}


def check_group_timers(resolved, where, window_id, consumer, windows_by_id):
    demod = _us_map(resolved["demod"])
    sic = _us_map(resolved["sic"])

    # timing/sfd/cir child sums, within scope=demod only
    for parent, children in (("timing", TIMING_CHILDREN),
                             ("sfd", SFD_CHILDREN), ("cir", CIR_CHILDREN)):
        parent_row = demod.get(parent)
        present_children = [c for c in children if c in demod
                            and demod[c]["us"] is not None]
        if not present_children:
            continue
        if parent_row is None or parent_row["us"] is None:
            fail("%s: %s parent is timed when its children are" % (where, parent),
                 "children %s present without a timed %s" % (present_children,
                                                             parent))
            continue
        child_sum = sum(demod[c]["us"] for c in present_children)
        if child_sum <= parent_row["us"] + max(TOL, REL_TOL * parent_row["us"]):
            ok("%s: %s children sum <= %s" % (where, parent, parent),
               "%.6g <= %.6g" % (child_sum, parent_row["us"]))
        else:
            fail("%s: %s children sum <= %s" % (where, parent, parent),
                 "children sum=%.6g exceeds %s=%.6g"
                 % (child_sum, parent, parent_row["us"]))

    # SIC core_total is the SIC authority.
    sic_children = {s: r for s, r in sic.items() if s != "core_total"
                    and r["us"] is not None}
    if sic_children:
        core = sic.get("core_total")
        if core is None or core["us"] is None:
            fail("%s: core_total is timed when SIC stages are" % where,
                 "SIC stages %s present without core_total" % sorted(sic_children))
        else:
            total = sum(r["us"] for r in sic_children.values())
            if total <= core["us"] + max(TOL, REL_TOL * core["us"]):
                ok("%s: SIC stage sum <= core_total" % where,
                   "%.6g <= %.6g" % (total, core["us"]))
            else:
                fail("%s: SIC stage sum <= core_total" % where,
                     "sum=%.6g exceeds core_total=%.6g" % (total, core["us"]))

    # authoritative demod total + naive double-count check.  Only scope=demod
    # top-level stages count: radar `cir` and adapter/outer rows must not
    # pollute the sum.
    total_row = demod.get("total")
    if total_row is None or total_row["us"] is None:
        fail("%s: demod total is timed (authoritative steady_clock)" % where,
             "missing/empty scope=demod total row")
    else:
        base_stages = {s: r["us"] for s, r in demod.items()
                       if r["us"] is not None and s in DEMOD_TOP_LEVEL}
        child_stages = {s: r["us"] for s, r in demod.items()
                        if r["us"] is not None
                        and (s in TIMING_CHILDREN or s in SFD_CHILDREN
                             or s in CIR_CHILDREN)}
        base = sum(base_stages.values())
        naive = base + sum(child_stages.values())
        total = total_row["us"]
        if total >= base - max(TOL, REL_TOL * max(base, 1.0)):
            ok("%s: demod total >= the top-level demod stage sum" % where,
               "total=%.6g base=%.6g (scope=demod only)" % (total, base))
        else:
            fail("%s: demod total >= the top-level demod stage sum" % where,
                 "total=%.6g < base=%.6g (scope=demod only)" % (total, base))
        if child_stages and abs(naive - base) > max(TOL, REL_TOL * base) \
                and abs(total - naive) <= max(TOL, REL_TOL * naive):
            fail("%s: demod total is not a naive parent+child double count"
                 % where,
                 "total=%.6g equals the double-counted sum (base=%.6g)"
                 % (total, base))
        else:
            ok("%s: demod total is not a naive parent+child double count"
               % where,
               "total=%.6g base=%.6g naive=%.6g" % (total, base, naive))

    # failed stages are timed; not_run stages are empty (already enforced).
    window = windows_by_id.get(window_id)
    if window is not None:
        block = window.get(consumer)
        if isinstance(block, dict):
            failing = block.get("failing_stage") or block.get("failed_stage")
            names = failing if isinstance(failing, list) else [failing]
            for name in names:
                if not name:
                    continue
                row = demod.get(name) or sic.get(name)
                if row is None:
                    fail("%s: failing stage %r is timed" % (where, name),
                         "no stage_times row for a declared failure")
                elif row["us"] is None:
                    fail("%s: failing stage %r is timed" % (where, name),
                         "declared failure with an empty us (not_run)")


# ---------------------------------------------------------------------------
# 5. summary.json
# ---------------------------------------------------------------------------


def summary_key_mentioned(summary, key):
    not_run = summary.get("not_run")
    if not isinstance(not_run, list):
        return False
    needle = norm_text(key)
    return any(needle in norm_text(entry) for entry in not_run)


def null_key_ok(summary, key):
    """A null value is acceptable iff it is named in not_run[].

    For SIC-conditional keys, a `sic` not_run reason (e.g. "sic consumer (sic
    off)") also covers the whole SIC-off situation.
    """
    if summary_key_mentioned(summary, key):
        return True
    if key in SIC_CONDITIONAL_KEYS and summary.get("sic_mode") == "off" \
            and summary_key_mentioned(summary, "sic"):
        return True
    return False


def check_summary(run_dir, manifest, prefix):
    item = "%s/summary.json" % prefix
    path = os.path.join(run_dir, "summary.json")
    summary, err = load_json(path)
    if summary is None:
        fail("%s: present and well-formed JSON" % item, err or "missing")
        return None
    if not isinstance(summary, dict):
        fail("%s: is a JSON object" % item, "got %r" % type(summary).__name__)
        return None
    ok("%s: present and well-formed JSON" % item, "object with %d keys"
       % len(summary))

    sic_mode = summary.get("sic_mode")
    for key in SUMMARY_KEYS:
        if key not in summary:
            fail("%s: <key>" % item, "missing required key %r (never silently OK)"
                 % key)
            continue
        if summary[key] is None and not null_key_ok(summary, key):
            fail("%s: %s is null and not listed in not_run[]" % (item, key),
                 "an unexecuted/inapplicable item must appear in not_run[]")

    demod = summary.get("demod")
    if not isinstance(demod, dict):
        fail("%s: demod is an object" % item, "got %r" % (demod,))
    else:
        for key in DEMOD_KEYS:
            if key not in demod:
                fail("%s: demod.%s is present" % (item, key),
                     "missing required key %r" % key)
                continue
            value = demod[key]
            if value is None:
                if not null_key_ok(summary, key):
                    fail("%s: demod.%s is null and not listed in not_run[]"
                         % (item, key),
                         "an inapplicable item must appear in not_run[]")
                elif sic_mode != "off":
                    fail("%s: demod.%s is null while SIC is on"
                         % (item, key), "sic_mode=%r" % (sic_mode,))
            elif key == "failing_stages":
                if not (isinstance(value, list)
                        and all(isinstance(x, str) for x in value)):
                    fail("%s: demod.failing_stages is a list of strings"
                         % item, "got %r" % (value,))
            elif not is_int(value) or value < 0:
                fail("%s: demod.%s is a non-negative integer" % (item, key),
                     "got %r" % (value,))
        ok("%s: demod carries the rev-2 keys" % item,
           "%d keys; failing_stages=%r" % (len(DEMOD_KEYS),
                                            demod.get("failing_stages")))

    if summary.get("run_id") is not None and manifest is not None \
            and manifest.get("run_id") is not None \
            and summary["run_id"] != manifest["run_id"]:
        fail("%s: run_id matches run_manifest.json" % item,
             "summary=%r manifest=%r" % (summary["run_id"], manifest["run_id"]))
    for key in ("preset", "input_grid", "sic_mode"):
        if manifest is not None and manifest.get(key) is not None \
                and summary.get(key) is not None and summary[key] != manifest[key]:
            fail("%s: %s matches run_manifest.json" % (item, key),
                 "summary=%r manifest=%r" % (summary[key], manifest[key]))
    if is_nonempty_str(summary.get("run_id")):
        ok("%s: run_id is a non-empty string" % item, summary["run_id"][:80])
    else:
        fail("%s: run_id is a non-empty string" % item,
             "got %r" % (summary.get("run_id"),))
    if summary.get("preset") in PRESETS:
        ok("%s: preset is a frozen preset" % item, str(summary.get("preset")))
    else:
        fail("%s: preset is a frozen preset" % item,
             "got %r" % (summary.get("preset"),))
    if summary.get("input_grid") in INPUT_GRIDS:
        ok("%s: input_grid is work|cg400" % item, str(summary.get("input_grid")))
    else:
        fail("%s: input_grid is work|cg400" % item,
             "got %r" % (summary.get("input_grid"),))
    if summary.get("sic_mode") in SIC_MODES:
        ok("%s: sic_mode is off|estimate|oracle" % item,
           str(summary.get("sic_mode")))
    else:
        fail("%s: sic_mode is off|estimate|oracle" % item,
             "got %r" % (summary.get("sic_mode"),))

    not_run = summary.get("not_run")
    if isinstance(not_run, list) and all(isinstance(x, str) for x in not_run):
        ok("%s: not_run is a list of strings" % item, "%d entries" % len(not_run))
    else:
        fail("%s: not_run is a list of strings" % item, "got %r" % (not_run,))

    gates = summary.get("gates")
    if not isinstance(gates, dict):
        fail("%s: gates is an object" % item, "got %r" % (gates,))
    else:
        for key in ("eff_pass", "sic_speed_pass", "demod_speed_pass", "native_pass"):
            if key not in gates:
                fail("%s: gates.%s is present" % (item, key), "missing")
            elif gates[key] is None:
                if not summary_key_mentioned(summary, key):
                    fail("%s: gates.%s is null and not in not_run[]" % (item, key),
                         "unexecuted item must appear in not_run[]")
            elif not isinstance(gates[key], bool):
                fail("%s: gates.%s is a boolean or null" % (item, key),
                     "got %r" % (gates[key],))
        if gates.get("native_pass") is True and summary.get("input_grid") != "cg400":
            fail("%s: gates.native_pass implies the cg400 grid" % item,
                 "native_pass=true input_grid=%r" % (summary.get("input_grid"),))

    runtime = summary.get("runtime")
    if isinstance(runtime, dict):
        keys = ("offered", "admitted", "completed", "failed", "bypassed",
                "dropped", "deadline_misses", "inflight_hw", "queue_hw",
                "rss_bytes", "cpu_seconds")
        missing = [k for k in keys if k not in runtime]
        if missing:
            fail("%s: runtime carries the frozen counters" % item,
                 "missing %s" % missing)
        else:
            ok("%s: runtime carries the frozen counters" % item,
               "%d counters" % len(keys))
            for key in keys:
                if not is_num(runtime[key]) or float(runtime[key]) < 0:
                    fail("%s: runtime.%s is a non-negative number" % (item, key),
                         "got %r" % (runtime[key],))
    else:
        fail("%s: runtime is an object" % item, "got %r" % (runtime,))

    return summary


def cross_check_summary(summary, windows, stats, prefix):
    item = "%s/summary.json" % prefix
    if not isinstance(summary, dict) or windows is None:
        return
    n = len(windows)
    completed = summary.get("windows_completed")
    if is_int(completed) and completed == n:
        ok("%s: windows_completed recomputed from windows.jsonl" % item, str(n))
    else:
        fail("%s: windows_completed recomputed from windows.jsonl" % item,
             "summary=%r on disk=%d" % (completed, n))
    expected = summary.get("windows_expected")
    if is_int(expected) and expected != n and not summary_key_mentioned(summary, "windows"):
        fail("%s: windows_expected != completed needs a not_run entry" % item,
             "expected=%r completed=%d" % (expected, n))

    # runtime.completed
    runtime = summary.get("runtime")
    if isinstance(runtime, dict) and runtime.get("completed") is not None:
        if is_num(runtime.get("completed")) and int(runtime["completed"]) == n:
            ok("%s: runtime.completed matches the window count" % item, str(n))
        else:
            fail("%s: runtime.completed matches the window count" % item,
                 "summary=%r on disk=%d" % (runtime.get("completed"), n))
        offered, admitted, done = (runtime.get("offered"), runtime.get("admitted"),
                                   runtime.get("completed"))
        if all(is_num(v) for v in (offered, admitted, done)):
            if admitted <= offered and done <= admitted:
                ok("%s: runtime offered >= admitted >= completed" % item,
                   "%s >= %s >= %s" % (offered, admitted, done))
            else:
                fail("%s: runtime offered >= admitted >= completed" % item,
                     "%s >= %s >= %s" % (offered, admitted, done))

    # per-consumer completion counts computed from windows
    for consumer in CONSUMERS:
        block = summary.get(consumer)
        if not isinstance(block, dict):
            continue
        present = sum(1 for w in windows if isinstance(w.get(consumer), dict))
        if "completed" in block:
            if is_int(block["completed"]) and block["completed"] == present:
                ok("%s: %s.completed recomputed" % (item, consumer), str(present))
            else:
                fail("%s: %s.completed recomputed" % (item, consumer),
                     "summary=%r on disk=%d" % (block["completed"], present))

    # per-stage timing statistics vs the CSV
    if stats:
        for consumer in CONSUMERS:
            block = summary.get(consumer)
            if not isinstance(block, dict):
                continue
            recompute_stage_stats(block, stats.get(consumer, {}), item, consumer)

    # demod aggregate recompute
    demod_block = summary.get("demod")
    if isinstance(demod_block, dict):
        counts = {"clean_ok": 0, "raw_ok": 0, "sic_ok": 0,
                  "raw_ok_sic_fail": 0, "raw_fail_sic_ok": 0}
        for window in windows:
            clean_ok = consumer_ok(window.get("clean"))
            raw_ok = consumer_ok(window.get("raw"))
            sic_block = window.get("sic")
            sic_ok = consumer_ok(sic_block) if isinstance(sic_block, dict) else None
            counts["clean_ok"] += int(clean_ok)
            counts["raw_ok"] += int(raw_ok)
            if sic_ok is not None:
                counts["sic_ok"] += int(sic_ok)
                counts["raw_ok_sic_fail"] += int(raw_ok and not sic_ok)
                counts["raw_fail_sic_ok"] += int((not raw_ok) and sic_ok)
        for key, value in counts.items():
            if key not in demod_block:
                continue
            declared = demod_block[key]
            if declared is None:
                ok("%s: demod.%s is null (inapplicable when SIC is off)"
                   % (item, key), "null")
            elif is_int(declared) and declared == value:
                ok("%s: demod.%s recomputed" % (item, key), str(value))
            else:
                fail("%s: demod.%s recomputed" % (item, key),
                     "summary=%r on disk=%d" % (declared, value))
        # failing_stages = sorted consumers with >= 1 non-success status.
        on_disk = sorted({consumer for consumer in CONSUMERS
                          for window in windows
                          if isinstance(window.get(consumer), dict)
                          and window[consumer].get("status")
                          not in SUCCESS_STATUSES})
        declared = demod_block.get("failing_stages")
        if isinstance(declared, list):
            declared = sorted(declared)
            if declared == on_disk:
                ok("%s: demod.failing_stages recomputed" % item, str(on_disk))
            else:
                fail("%s: demod.failing_stages recomputed" % item,
                     "summary=%s on disk=%s" % (declared, on_disk))
        elif declared is None:
            ok("%s: demod.failing_stages is null (inapplicable)" % item, "null")
        else:
            fail("%s: demod.failing_stages recomputed" % item,
                 "summary=%r on disk=%s" % (declared, on_disk))


def consumer_ok(block):
    if not isinstance(block, dict):
        return False
    if block.get("status") not in ("success", "ok", "pass"):
        return False
    return block.get("fcs_pass") is True or block.get("psdu_ok") is True \
        or block.get("cir_ok") is True


def recompute_stage_stats(block, stage_values, item, consumer):
    for key, value in block.items():
        stage = key[6:] if key.startswith("stage_") else key
        if not isinstance(value, dict):
            continue
        if stage not in stage_values or not stage_values[stage]:
            continue
        values = stage_values[stage]
        computed = {
            "mean": sum(values) / len(values),
            "p50": percentile(values, 50),
            "p95": percentile(values, 95),
            "p99": percentile(values, 99),
            "max": max(values),
        }
        for stat, want in computed.items():
            if stat in value and is_num(value[stat]):
                if close(value[stat], want):
                    ok("%s: %s.%s recomputed from stage_times.csv"
                       % (item, consumer, key), "%.6g" % want)
                else:
                    fail("%s: %s.%s recomputed from stage_times.csv"
                         % (item, consumer, key),
                         "summary=%r recomputed=%.6g" % (value[stat], want))


# ---------------------------------------------------------------------------
# 6. demod result integrity
# ---------------------------------------------------------------------------


def check_demod_integrity(windows, prefix):
    item = "%s/windows.jsonl demod integrity" % prefix
    for window in windows:
        wid = window.get("window_id")
        for consumer in CONSUMERS:
            block = window.get(consumer)
            if not isinstance(block, dict):
                continue
            where = "%s window_id=%r %s" % (item, wid, consumer)
            fcs = block.get("fcs_pass")
            psdu_ok = block.get("psdu_ok")
            claims = fcs is True or psdu_ok is True
            psdu_hex = block.get("psdu_hex", block.get("psdu"))
            if claims:
                missing = [k for k in ("status", "fcs_pass", "psdu_ok")
                           if k not in block]
                if missing:
                    fail("%s: fields used to justify a pass are present" % where,
                         "claims pass but missing %s" % missing)
                    continue
                if psdu_hex is None:
                    fail("%s: a pass carries the decoded PSDU bytes" % where,
                         "fcs_pass=%r psdu_ok=%r but no psdu_hex" % (fcs, psdu_ok))
                if fcs is True and block.get("status") not in ("success", "ok", "pass"):
                    fail("%s: fcs_pass implies a success status" % where,
                         "fcs_pass=true status=%r" % (block.get("status"),))
                if psdu_ok is True and fcs is not True:
                    fail("%s: psdu_ok implies fcs_pass" % where,
                         "psdu_ok=true fcs_pass=%r" % (fcs,))
            if psdu_hex is None:
                continue
            try:
                decoded = bytes.fromhex(str(psdu_hex))
            except ValueError:
                fail("%s: psdu_hex is valid hex" % where, "got %r" % (psdu_hex,))
                continue
            # Canonical code-9 sensing PSDU.
            if window.get("sensing_code") == 9:
                if decoded == CANONICAL_PSDU_BYTES:
                    ok("%s: code-9 PSDU equals the canonical bytes" % where,
                       decoded.hex().upper())
                else:
                    fail("%s: code-9 PSDU equals the canonical bytes" % where,
                         "got %s want %s" % (decoded.hex().upper(),
                                             CANONICAL_PSDU_HEX))
            # The fields used to justify psdu_ok.
            if block.get("psdu_ok") is True:
                for key in ("psdu_bytes", "psdu_len", "phr_psdu_length"):
                    if key in block and is_int(block[key]) and block[key] != len(decoded):
                        fail("%s: %s equals the decoded PSDU length" % (where, key),
                             "declared=%r decoded=%d" % (block[key], len(decoded)))
            if "cir_peak_tap" in block and not is_int(block["cir_peak_tap"]):
                fail("%s: cir_peak_tap is an integer" % where,
                     "got %r" % (block["cir_peak_tap"],))
            if "target_amp" in block:
                val = block["target_amp"]
                good = (is_num(val)
                        or (isinstance(val, (list, tuple)) and len(val) == 2
                            and all(is_num(v) for v in val)))
                if not good:
                    fail("%s: target_amp is a scalar or a [re,im] pair" % where,
                         "got %r" % (val,))


# ---------------------------------------------------------------------------
# 7. metrics sanity
# ---------------------------------------------------------------------------


def check_recon_metrics(windows, run_dir, prefix):
    item = "%s/windows.jsonl recon" % prefix
    npz_cache = {}
    declared_holdout = 0
    recomputed_holdout = 0
    declared_actual = 0
    recomputed_actual = 0
    for window in windows:
        wid = window.get("window_id")
        recon = window.get("recon")
        if not isinstance(recon, dict):
            continue
        where = "%s window_id=%r" % (item, wid)
        check_recon_sanity(recon, where)
        declares = (recon.get("holdout_nmse") is not None
                    or recon.get("holdout_suppression_db") is not None)
        if declares:
            declared_holdout += 1
        declares_actual = has_actual_fields(recon)
        if declares_actual:
            declared_actual += 1
        # Recompute from the per-window arrays when they exist.  The app dumps
        # arrays for a declared subset of windows (--fixtures), so a window
        # without arrays is not itself an error; the run-level checks below
        # require that at least one window can actually be recomputed.
        arrays = load_recon_arrays(run_dir, wid, npz_cache)
        if arrays is None:
            continue
        if declares and "error" not in arrays and "i" in arrays \
                and "i_hat" in arrays:
            recomputed_holdout += 1
        recompute_recon(arrays, recon, where)
        if declares_actual and "error" not in arrays \
                and all(k in arrays for k in ("y", "y_clean", "i")):
            recomputed_actual += 1
        if "error" not in arrays:
            check_actual_cancellation(window, recon, arrays, where)
    if declared_holdout and recomputed_holdout == 0:
        fail("%s: run provides arrays to recompute its declared holdout metrics"
             % item,
             "%d window(s) declare holdout metrics but no recon/window_*.npz "
             "with i/i_hat exists under %s" % (declared_holdout, run_dir))
    elif declared_holdout:
        ok("%s: holdout metrics recomputed where arrays exist" % item,
           "%d/%d declared-holdout window(s) have recon arrays"
           % (recomputed_holdout, declared_holdout))
    # R01: a declared-actual window with no y/y_clean/i arrays is a FAIL, not a
    # skip (the run cannot be independently recomputed).
    if declared_actual and recomputed_actual == 0:
        fail("%s: declared actual-output metrics have y/y_clean/i arrays" % item,
             "%d window(s) declare recon.actual_*/sub_* but no "
             "recon/window_*.npz with y/y_clean/i exists under %s"
             % (declared_actual, run_dir))
    elif declared_actual:
        ok("%s: actual-output cancellation recomputed where arrays exist" % item,
           "%d/%d declared-actual window(s) have y/y_clean/i arrays"
           % (recomputed_actual, declared_actual))


def scalar_or_pair_mag(value):
    if value is None:
        return None
    if is_num(value):
        return abs(float(value))
    if isinstance(value, (list, tuple)) and len(value) == 2 \
            and all(is_num(v) for v in value):
        return math.hypot(float(value[0]), float(value[1]))
    return None


def check_recon_sanity(recon, where):
    nmse = recon.get("nmse")
    sup = recon.get("suppression_db")
    if is_num(nmse) and float(nmse) >= 0:
        ok("%s: recon.nmse is in [0, inf)" % where, str(nmse))
    else:
        fail("%s: recon.nmse is in [0, inf)" % where, "got %r" % (nmse,))
    if is_num(sup):
        ok("%s: recon.suppression_db is a finite number" % where, str(sup))
    else:
        fail("%s: recon.suppression_db is a finite number" % where,
             "got %r" % (sup,))
    lo, hi = recon.get("eval_lo"), recon.get("eval_hi")
    if is_int(lo) and is_int(hi) and 0 <= lo < hi:
        ok("%s: recon eval window is a non-empty [lo,hi)" % where,
           "[%d,%d)" % (lo, hi))
    else:
        fail("%s: recon eval window is a non-empty [lo,hi)" % where,
             "lo=%r hi=%r" % (lo, hi))

    # R01 actual-output fields (present once the run adopts the frozen contract).
    if has_actual_fields(recon):
        missing = [k for k in ACTUAL_RECON_KEYS if k not in recon]
        if missing:
            fail("%s: recon carries the R01 actual-output fields" % where,
                 "missing %s (the new fields are all-or-nothing)" % missing)
        a_nmse = recon.get("actual_nmse")
        a_sup = recon.get("actual_suppression_db")
        if a_nmse is None or (is_num(a_nmse) and float(a_nmse) >= 0):
            ok("%s: recon.actual_nmse is in [0, inf) or null (N/A)" % where,
               str(a_nmse))
        else:
            fail("%s: recon.actual_nmse is in [0, inf) or null (N/A)" % where,
                 "got %r" % (a_nmse,))
        if a_sup is None or is_num(a_sup):
            ok("%s: recon.actual_suppression_db is a number or null (N/A)"
               % where, str(a_sup))
        else:
            fail("%s: recon.actual_suppression_db is a number or null (N/A)"
                 % where, "got %r" % (a_sup,))
        if is_num(a_nmse) and is_num(a_sup) and float(a_nmse) > 0:
            want = -10.0 * math.log10(float(a_nmse))
            if abs(want - float(a_sup)) <= SUPPRESSION_TOL_DB:
                ok("%s: actual_suppression_db is consistent with actual_nmse"
                   % where, "recomputed=%.4f" % want)
            else:
                fail("%s: actual_suppression_db is consistent with actual_nmse"
                     % where, "recomputed=%.4f declared=%.4f" % (want, a_sup))
        sub_lo, sub_hi = recon.get("sub_lo"), recon.get("sub_hi")
        if is_int(sub_lo) and is_int(sub_hi) and 0 <= sub_lo <= sub_hi:
            ok("%s: recon.sub_lo/sub_hi is a well-formed [lo,hi)" % where,
               "[%d,%d)" % (sub_lo, sub_hi))
        else:
            fail("%s: recon.sub_lo/sub_hi is a well-formed [lo,hi)" % where,
                 "sub_lo=%r sub_hi=%r" % (sub_lo, sub_hi))
        cov = recon.get("coverage")
        if is_num(cov) and float(cov) >= 0:
            ok("%s: recon.coverage is a non-negative number" % where, str(cov))
        else:
            fail("%s: recon.coverage is a non-negative number" % where,
                 "got %r" % (cov,))
        if "uses_truth" in recon and not isinstance(recon["uses_truth"], bool):
            fail("%s: recon.uses_truth is a boolean when present" % where,
                 "got %r" % (recon["uses_truth"],))

    # suppression must match -10*log10(nmse) (the i-based definition).
    if is_num(nmse) and is_num(sup) and float(nmse) > 0:
        want = -10.0 * math.log10(float(nmse))
        if abs(want - float(sup)) <= SUPPRESSION_TOL_DB:
            ok("%s: recon.suppression_db is consistent with nmse" % where,
               "recomputed=%.4f declared=%.4f" % (want, sup))
        else:
            fail("%s: recon.suppression_db is consistent with nmse" % where,
                 "recomputed=%.4f declared=%.4f (not the i-based metric?)"
                 % (want, sup))
    elif is_num(nmse) and float(nmse) == 0 and is_num(sup) and float(sup) >= 60:
        ok("%s: recon.suppression_db is consistent with nmse==0" % where, str(sup))

    # Reject a ||y||^2/||y_clean||^2 substitute when the fields expose it.
    i_energy = first_num(recon, ("i_energy", "i_power", "interference_energy",
                                 "interference_power"))
    err_energy = first_num(recon, ("err_energy", "err_power", "residual_energy",
                                   "residual_power", "i_minus_ihat_energy"))
    y_energy = first_num(recon, ("y_energy", "y_power", "received_energy"))
    y_clean_energy = first_num(recon, ("y_clean_energy", "y_clean_power",
                                       "clean_energy", "clean_power"))
    if is_num(sup) and is_num(i_energy) and i_energy > 0 and is_num(err_energy) \
            and err_energy > 0:
        want = 10.0 * math.log10(i_energy / err_energy)
        if abs(want - float(sup)) <= SUPPRESSION_TOL_DB:
            ok("%s: suppression_db is 10*log10(||i||^2/||i-i_hat||^2)" % where,
               "recomputed=%.4f" % want)
        else:
            fail("%s: suppression_db is 10*log10(||i||^2/||i-i_hat||^2)" % where,
                 "recomputed=%.4f declared=%.4f" % (want, sup))
    if is_num(sup) and is_num(y_energy) and is_num(y_clean_energy):
        if y_clean_energy != 0:
            sub = 10.0 * math.log10(y_energy / y_clean_energy)
            i_based_ok = (is_num(i_energy) and is_num(err_energy) and err_energy > 0
                          and abs(10.0 * math.log10(i_energy / err_energy)
                                  - float(sup)) <= SUPPRESSION_TOL_DB)
            if abs(sub - float(sup)) <= SUPPRESSION_TOL_DB and not i_based_ok:
                fail("%s: suppression_db is not ||y||^2/||y_clean||^2" % where,
                     "declared=%.4f matches the forbidden substitute=%.4f"
                     % (sup, sub))
            else:
                ok("%s: suppression_db is not the ||y||^2/||y_clean||^2 substitute"
                   % where, "y-based=%.4f declared=%.4f" % (sub, sup))

    # holdout fields
    h_nmse = recon.get("holdout_nmse")
    h_sup = recon.get("holdout_suppression_db")
    for key, value in (("holdout_nmse", h_nmse),
                       ("holdout_suppression_db", h_sup)):
        if value is not None and not is_num(value):
            fail("%s: recon.%s is a number or null" % (where, key),
                 "got %r" % (value,))
    if is_num(h_nmse) and float(h_nmse) < 0:
        fail("%s: recon.holdout_nmse is in [0, inf)" % where, str(h_nmse))
    if is_num(h_nmse) and float(h_nmse) > 0 and is_num(h_sup):
        want = -10.0 * math.log10(float(h_nmse))
        if abs(want - float(h_sup)) <= SUPPRESSION_TOL_DB:
            ok("%s: holdout suppression is consistent with holdout nmse" % where,
               "recomputed=%.4f" % want)
        else:
            fail("%s: holdout suppression is consistent with holdout nmse" % where,
                 "recomputed=%.4f declared=%.4f" % (want, h_sup))

    # A declared holdout metric must name a non-empty window that is disjoint
    # from the fit/estimation span (which precedes it: fit = [eval_lo, h_lo)).
    if h_nmse is not None or h_sup is not None:
        h_lo, h_hi = recon.get("holdout_lo"), recon.get("holdout_hi")
        if is_int(h_lo) and is_int(h_hi) and 0 <= h_lo < h_hi:
            ok("%s: holdout window is a non-empty [lo,hi)" % where,
               "[%d,%d)" % (h_lo, h_hi))
        else:
            fail("%s: holdout window is a non-empty [lo,hi)" % where,
                 "holdout_lo=%r holdout_hi=%r" % (h_lo, h_hi))
        if is_int(h_lo) and is_int(lo) and h_lo > lo:
            ok("%s: holdout is disjoint from the fit window" % where,
               "fit=[%d,%d) holdout=[%d,%d)" % (lo, h_lo, h_lo, h_hi)
               if is_int(h_hi) else "fit starts at %d" % lo)
        else:
            fail("%s: holdout is disjoint from the fit window" % where,
                 "eval_lo=%r holdout_lo=%r (holdout must start strictly after "
                 "the fit span)" % (lo, h_lo))


def first_num(mapping, keys):
    for key in keys:
        if is_num(mapping.get(key)):
            return float(mapping[key])
    return None


def load_recon_arrays(run_dir, window_id, cache):
    if np is None:
        return None
    wid = str(window_id)
    if wid in cache:
        return cache[wid]
    roots = [os.path.join(run_dir, "recon"), run_dir]
    found = None
    for root in roots:
        if not os.path.isdir(root):
            continue
        for name in sorted(os.listdir(root)):
            if name.endswith(".npz") and ("window_%s" % wid) in name:
                found = os.path.join(root, name)
                break
        if found:
            break
    if found is None:
        cache[wid] = None
        return None
    try:
        data = np.load(found)
    except Exception as exc:  # pragma: no cover - corrupt npz
        cache[wid] = {"error": "cannot read %s: %s" % (found, exc)}
        return cache[wid]
    arrays = {"path": found}
    for target, candidates in (("i", ("i", "i_true", "interference")),
                               ("i_hat", ("i_hat", "i_recon", "ihat",
                                          "i_estimate")),
                               # R01 actual-output metric: y is the raw SIC
                               # input and y_clean is the IQ the SIC actually
                               # output (delta_actual = y - y_clean).
                               ("y", ("y", "y_raw", "raw", "received")),
                               ("y_clean", ("y_clean", "y_sic", "sic_out",
                                            "cleaned", "y_out")),
                               # R04: raw (non-normalized) complex CIRs.
                               ("cir_raw_clean", NPZ_CIR_CLEAN_KEYS),
                               ("cir_raw_sic", NPZ_CIR_SIC_KEYS)):
        for key in candidates:
            if key in data:
                arrays[target] = np.asarray(data[key]).astype(np.complex128).ravel()
                break
    cache[wid] = arrays
    return arrays


def recompute_recon(arrays, recon, where):
    if arrays is None or "error" in arrays:
        fail("%s: per-window recon npz is readable" % where,
             arrays.get("error") if arrays else "unreadable")
        return
    if "i" not in arrays or "i_hat" not in arrays:
        declares_holdout = (recon.get("holdout_nmse") is not None
                            or recon.get("holdout_suppression_db") is not None)
        if declares_holdout:
            fail("%s: holdout recompute has i/i_hat arrays" % where,
                 "npz %s lacks i/i_hat" % os.path.basename(arrays["path"]))
        else:
            skip("%s: recompute over the eval window" % where,
                 "npz %s lacks i/i_hat; sidecar cross-check only"
                 % os.path.basename(arrays["path"]))
        return
    i_full, i_hat_full = arrays["i"], arrays["i_hat"]
    lo, hi = recon.get("eval_lo"), recon.get("eval_hi")
    if is_int(lo) and is_int(hi) and hi <= len(i_full) and hi <= len(i_hat_full):
        i = i_full[lo:hi]
        i_hat = i_hat_full[lo:hi]
    elif len(i_full) == len(i_hat_full):
        i = i_full
        i_hat = i_hat_full
    else:
        fail("%s: recompute over the eval window" % where,
             "npz length %d/%d cannot cover eval [%r,%r)"
             % (len(i_full), len(i_hat_full), lo, hi))
        return
    diff = i_hat - i
    den = float(np.vdot(i, i).real)
    num = float(np.vdot(diff, diff).real)
    if den <= 0:
        skip("%s: recompute over the eval window" % where,
             "||i||^2 == 0; NMSE undefined")
        return
    nmse = num / den
    sup = 10.0 * math.log10(den / num) if num > 0 else float("inf")
    if is_num(recon.get("nmse")) and close(recon["nmse"], nmse):
        ok("%s: recomputed nmse over [L,R) matches" % where,
           "%.6g" % nmse)
    else:
        fail("%s: recomputed nmse over [L,R) matches" % where,
             "recomputed=%.6g declared=%r" % (nmse, recon.get("nmse")))
    if is_num(recon.get("suppression_db")) and is_num(sup) \
            and abs(float(recon["suppression_db"]) - sup) <= SUPPRESSION_TOL_DB:
        ok("%s: recomputed suppression over [L,R) matches" % where, "%.4f dB" % sup)
    else:
        fail("%s: recomputed suppression over [L,R) matches" % where,
             "recomputed=%s declared=%r"
             % ("inf" if not math.isfinite(sup) else "%.4f" % sup,
                recon.get("suppression_db")))

    # Holdout recompute over the declared [holdout_lo, holdout_hi), sliced from
    # the FULL arrays (the eval slice above must not be reused here).
    if recon.get("holdout_nmse") is not None \
            or recon.get("holdout_suppression_db") is not None:
        recompute_holdout(recon, i_full, i_hat_full, where)


def recompute_holdout(recon, i_full, i_hat_full, where):
    h_lo, h_hi = recon.get("holdout_lo"), recon.get("holdout_hi")
    if not (is_int(h_lo) and is_int(h_hi) and 0 <= h_lo < h_hi):
        fail("%s: holdout window is a non-empty [lo,hi)" % where,
             "holdout_lo=%r holdout_hi=%r" % (h_lo, h_hi))
        return
    if h_hi > len(i_full) or h_hi > len(i_hat_full):
        fail("%s: holdout window fits the recon arrays" % where,
             "[%d,%d) > %d/%d samples"
             % (h_lo, h_hi, len(i_full), len(i_hat_full)))
        return
    ref = i_full[h_lo:h_hi]
    est = i_hat_full[h_lo:h_hi]
    den = float(np.vdot(ref, ref).real)
    if den > 0:
        diff = est - ref
        num = float(np.vdot(diff, diff).real)
        want_nmse = num / den
        want_sup = -10.0 * math.log10(want_nmse) if num > 0 else 60.0
    else:
        # Zero reference energy: mirror the app's declared convention.
        num = float(np.vdot(est, est).real)
        want_nmse, want_sup = (0.0, 60.0) if num == 0 else (1.0, 0.0)
    declared_nmse = recon.get("holdout_nmse")
    if is_num(declared_nmse) and close(declared_nmse, want_nmse):
        ok("%s: recomputed holdout_nmse over [L,R) matches" % where,
           "%.6g" % want_nmse)
    else:
        fail("%s: recomputed holdout_nmse over [L,R) matches" % where,
             "recomputed=%.6g declared=%r" % (want_nmse, declared_nmse))
    declared_sup = recon.get("holdout_suppression_db")
    if is_num(declared_sup) and abs(float(declared_sup) - want_sup) \
            <= SUPPRESSION_TOL_DB:
        ok("%s: recomputed holdout_suppression_db over [L,R) matches" % where,
           "%.4f dB" % want_sup)
    else:
        fail("%s: recomputed holdout_suppression_db over [L,R) matches" % where,
             "recomputed=%.4f declared=%r" % (want_sup, declared_sup))


# ---------------------------------------------------------------------------
# R01: actual-output cancellation (recomputed from y / y_clean / i)
# ---------------------------------------------------------------------------

# Containers a per-window R04 target block may live in, in preference order.
TARGET_CONTAINERS = ("target", "raw_target", "target_protection", "recon")
ACTUAL_RECON_KEYS = ("actual_nmse", "actual_suppression_db", "sub_lo",
                     "sub_hi", "coverage")


def has_actual_fields(recon):
    return isinstance(recon, dict) \
        and any(k in recon for k in ACTUAL_RECON_KEYS)


def window_truth_overlap(window):
    """The generator-side truth overlap [lo,hi) from windows.jsonl."""
    overlap = window.get("overlap")
    if not isinstance(overlap, dict):
        return None, None
    lo, hi = overlap.get("lo"), overlap.get("hi")
    if is_int(lo) and is_int(hi) and 0 <= lo < hi:
        return lo, hi
    return None, None


def _declared_actual_suppression(recon):
    for key in ("actual_suppression_db",):
        if is_num(recon.get(key)):
            return float(recon[key])
    return None


def check_actual_cancellation(window, recon, arrays, where):
    """Recompute the ACTUAL-output cancellation from y / y_clean / i arrays.

    ``y`` is the raw SIC input, ``y_clean`` is the IQ the SIC actually output;
    delta_actual = y - y_clean and residual = i - delta_actual over the truth
    overlap.  Never synthesize the output from the declared mask.
    """
    wid = window.get("window_id")
    for key in ("y", "y_clean", "i"):
        if key not in arrays:
            fail("%s: actual-cancellation arrays include %r" % (where, key),
                 "npz %s lacks %r (needed for R01 recompute)"
                 % (os.path.basename(arrays.get("path", "?")), key))
            return False
    y, y_sic, i = arrays["y"], arrays["y_clean"], arrays["i"]
    if not (len(y) == len(y_sic) == len(i)):
        fail("%s: actual-cancellation arrays are the same length" % where,
             "y=%d y_clean=%d i=%d" % (len(y), len(y_sic), len(i)))
        return False
    ov_lo, ov_hi = window_truth_overlap(window)
    if ov_lo is None:
        overlap = window.get("overlap")
        lo = overlap.get("lo") if isinstance(overlap, dict) else None
        hi = overlap.get("hi") if isinstance(overlap, dict) else None
        if is_int(lo) and is_int(hi) and lo == hi == 0:
            # No interferer overlap: the actual metric is N/A by contract.
            declared_sup = _declared_actual_suppression(recon)
            declared_nmse = recon.get("actual_nmse")
            if declared_sup is not None or declared_nmse is not None:
                fail("%s: actual suppression is N/A for an empty overlap"
                     % where,
                     "overlap=[0,0) but declared actual_nmse=%r "
                     "actual_suppression_db=%r" % (declared_nmse, declared_sup))
            else:
                ok("%s: actual suppression is N/A (empty overlap)" % where,
                   "N/A")
            sub_lo, sub_hi = recon.get("sub_lo"), recon.get("sub_hi")
            if is_int(sub_lo) and is_int(sub_hi) and sub_hi > sub_lo:
                fail("%s: sub mask is empty for an empty overlap" % where,
                     "declared=[%d,%d)" % (sub_lo, sub_hi))
            return True
        fail("%s: truth overlap is well-formed for the actual recompute" % where,
             "overlap=%r" % (overlap,))
        return False
    if ov_hi > len(i):
        fail("%s: truth overlap fits the archived arrays" % where,
             "[%d,%d) > %d samples" % (ov_lo, ov_hi, len(i)))
        return False

    i_ov = i[ov_lo:ov_hi]
    delta = y[ov_lo:ov_hi] - y_sic[ov_lo:ov_hi]
    residual = i_ov - delta
    den = float(np.vdot(i_ov, i_ov).real)
    num = float(np.vdot(residual, residual).real)
    committed = bool((window.get("sic_state") or {}).get("committed", False))
    declared_sup = _declared_actual_suppression(recon)
    declared_nmse = recon.get("actual_nmse")

    if den <= 0.0:
        # Zero interference energy: the actual metric is N/A by contract.
        if declared_sup is not None or (declared_nmse is not None):
            fail("%s: actual suppression is N/A for ||i||^2 == 0" % where,
                 "declared actual_nmse=%r actual_suppression_db=%r"
                 % (declared_nmse, declared_sup))
        else:
            ok("%s: actual suppression is N/A (||i||^2 == 0)" % where, "N/A")
        return True

    actual_nmse = num / den
    actual_sup = (-10.0 * math.log10(actual_nmse)) if num > 0 else 60.0
    if is_num(declared_nmse) and close(declared_nmse, actual_nmse, 1e-6, 1e-6):
        ok("%s: recomputed actual_nmse over the truth overlap matches" % where,
           "%.6g" % actual_nmse)
    else:
        fail("%s: recomputed actual_nmse over the truth overlap matches" % where,
             "recomputed=%.6g declared=%r" % (actual_nmse, declared_nmse))
    if declared_sup is not None and abs(declared_sup - actual_sup) \
            <= SUPPRESSION_TOL_DB:
        ok("%s: recomputed actual_suppression_db over the truth overlap matches"
           % where, "%.4f dB" % actual_sup)
    else:
        fail("%s: recomputed actual_suppression_db over the truth overlap matches"
             % where, "recomputed=%.4f declared=%r" % (actual_sup, declared_sup))

    # An uncommitted/bypassed window must not be counted as cancelled.
    if not committed:
        if declared_sup is not None and declared_sup > 1e-9:
            fail("%s: uncommitted window is not counted as cancelled" % where,
                 "committed=false but actual_suppression_db=%.4f "
                 "(recomputed %.4f)" % (declared_sup, actual_sup))
        elif actual_sup > 1e-6:
            fail("%s: uncommitted window did not change the output" % where,
                 "committed=false but the arrays still show %.4f dB" % actual_sup)
        else:
            ok("%s: uncommitted window is not counted as cancelled" % where,
               "committed=false, actual=%.4f dB" % actual_sup)

    check_actual_sub_mask(window, recon, y, y_sic, ov_lo, ov_hi, where)
    return True


def check_actual_sub_mask(window, recon, y, y_sic, ov_lo, ov_hi, where):
    """Verify the declared sub mask against the real y != y_clean support."""
    sub_lo, sub_hi = recon.get("sub_lo"), recon.get("sub_hi")
    if not (is_int(sub_lo) and is_int(sub_hi) and 0 <= sub_lo <= sub_hi):
        fail("%s: recon.sub_lo/sub_hi declare the applied mask" % where,
             "sub_lo=%r sub_hi=%r" % (sub_lo, sub_hi))
        return
    changed = (y[ov_lo:ov_hi] != y_sic[ov_lo:ov_hi])
    idx = np.nonzero(changed)[0]
    d_lo = min(max(int(sub_lo), ov_lo), ov_hi)
    d_hi = min(max(int(sub_hi), ov_lo), ov_hi)
    span = ov_hi - ov_lo

    if idx.size:
        chg_lo, chg_hi = int(idx[0]) + ov_lo, int(idx[-1]) + ov_lo + 1
        outside = idx[(idx + ov_lo < d_lo) | (idx + ov_lo >= d_hi)]
        if outside.size:
            fail("%s: declared sub mask covers every changed output sample"
                 % where,
                 "%d changed sample(s) outside [%d,%d) (first at %d)"
                 % (int(outside.size), d_lo, d_hi, int(outside[0]) + ov_lo))
        elif abs(int(sub_lo) - chg_lo) > MASK_TOL_SAMPLES \
                or abs(int(sub_hi) - chg_hi) > MASK_TOL_SAMPLES:
            fail("%s: declared sub mask matches the real changed support" % where,
                 "declared=[%d,%d) real=[%d,%d) (>%d sample mismatch)"
                 % (sub_lo, sub_hi, chg_lo, chg_hi, MASK_TOL_SAMPLES))
        else:
            ok("%s: declared sub mask matches the real changed support" % where,
               "declared=[%d,%d) real=[%d,%d)" % (sub_lo, sub_hi, chg_lo, chg_hi))
    else:
        # The output never changed over the truth overlap.
        if d_hi > d_lo:
            fail("%s: declared sub mask matches the real changed support" % where,
                 "declared=[%d,%d) but the output did not change y!=y_clean"
                 % (sub_lo, sub_hi))
        else:
            ok("%s: declared sub mask matches the real changed support" % where,
               "declared empty and the output did not change")

    want_cov = (int(sub_hi) - int(sub_lo)) / float(span) if span else 0.0
    declared_cov = recon.get("coverage")
    if is_num(declared_cov) and abs(float(declared_cov) - want_cov) <= 1e-6:
        ok("%s: recon.coverage matches sub_lo/sub_hi over the truth overlap"
           % where, "%.6g" % want_cov)
    else:
        fail("%s: recon.coverage matches sub_lo/sub_hi over the truth overlap"
             % where, "declared=%r recomputed=%.6g" % (declared_cov, want_cov))


def check_actual_effect_gate(summary, windows, prefix):
    """The main effect gate MUST be the actual-output agreement over the output."""
    item = "%s/summary.json effect (R01 actual)" % prefix
    if not isinstance(summary, dict):
        return
    sic_on = any(isinstance(w.get("recon"), dict) for w in windows)
    if not sic_on:
        return
    effect = summary.get("effect")
    if not isinstance(effect, dict):
        fail("%s: summary.effect is present for a SIC run" % item,
             "got %r" % (effect,))
        return
    gates = summary.get("gates")
    eff_pass = isinstance(gates, dict) and gates.get("eff_pass") is True
    actual_key = next((k for k in ACTUAL_SUPPRESSION_MEDIAN_KEYS
                       if is_num(effect.get(k))), None)
    recon_median = effect.get("median_suppression_db")

    if actual_key is None:
        n_interference = effect.get("n_interference_windows")
        if eff_pass:
            fail("%s: gates.eff_pass requires an actual-suppression aggregate"
                 % item,
                 "eff_pass=true but no actual aggregate is declared "
                 "(n_interference_windows=%r)" % (n_interference,))
        elif is_int(n_interference) and n_interference == 0:
            ok("%s: actual gate is N/A (no interference windows)" % item,
               "n_interference_windows=0, n_na_windows=%r"
               % (effect.get("n_na_windows"),))
        else:
            fail("%s: main effect gate is computed on actual_suppression_db"
                 % item,
                 "no actual aggregate present; declared median_suppression_db=%r "
                 "is the reconstruction metric (n_interference_windows=%r)"
                 % (recon_median, n_interference))
        return
    ok("%s: main effect gate key %r is the actual-output aggregate"
       % (item, actual_key), "%.4f dB" % float(effect[actual_key]))

    # Recompute the actual aggregate from the per-window declared actual values
    # over the non-truth windows (the same window set the app must use).
    vals = []
    uncommitted_bad = []
    for window in windows:
        if window_uses_truth(window):
            continue
        recon = window.get("recon")
        if not isinstance(recon, dict):
            continue
        if is_num(recon.get("actual_suppression_db")):
            vals.append(float(recon["actual_suppression_db"]))
        committed = bool((window.get("sic_state") or {}).get("committed", False))
        if not committed and is_num(recon.get("actual_suppression_db")) \
                and float(recon["actual_suppression_db"]) > 1e-9:
            uncommitted_bad.append(window.get("window_id"))
    if uncommitted_bad:
        fail("%s: no uncommitted window is counted as cancelled" % item,
             "window_id(s) %s declare a positive actual suppression without a "
             "commit" % uncommitted_bad[:5])

    if not vals:
        fail("%s: actual aggregate has per-window actual values" % item,
             "no non-truth window declares recon.actual_suppression_db")
        return
    want_med = median(vals)
    want_frac = sum(1.0 for v in vals if v >= ACTUAL_FLOOR_DB) / len(vals)
    declared = float(effect[actual_key])
    if close(declared, want_med, 1e-6, 1e-6):
        ok("%s: %s recomputed from recon.actual_suppression_db" % (item, actual_key),
           "%.4f dB" % want_med)
    else:
        fail("%s: %s recomputed from recon.actual_suppression_db"
             % (item, actual_key), "summary=%r recomputed=%.4f"
             % (declared, want_med))
    # It must not be the reconstruction metric wearing the actual name.
    if is_num(recon_median) and abs(declared - float(recon_median)) <= 1e-6 \
            and abs(want_med - float(recon_median)) > 1e-6:
        fail("%s: actual aggregate is not the reconstruction median" % item,
             "declared=%r equals reconstruction median=%r but the actual "
             "median is %.4f" % (declared, recon_median, want_med))
    frac_key = next((k for k in ACTUAL_FRAC_KEYS if is_num(effect.get(k))), None)
    if frac_key is not None:
        if close(float(effect[frac_key]), want_frac, 1e-6, 1e-6):
            ok("%s: %s recomputed from recon.actual_suppression_db"
               % (item, frac_key), "%.6g" % want_frac)
        else:
            fail("%s: %s recomputed from recon.actual_suppression_db"
                 % (item, frac_key), "summary=%r recomputed=%.6g"
                 % (effect[frac_key], want_frac))

    # Explicit interference / N/A counts (frozen contract).
    for key in ("n_interference_windows", "n_na_windows"):
        if not is_int(effect.get(key)) or effect.get(key) < 0:
            fail("%s: effect.%s counts the actual-gate windows" % (item, key),
                 "got %r" % (effect.get(key),))
    n_gate = effect.get("n_windows")
    if is_int(effect.get("n_interference_windows")) \
            and is_int(effect.get("n_na_windows")) and is_int(n_gate):
        if effect["n_interference_windows"] + effect["n_na_windows"] != n_gate:
            fail("%s: n_interference_windows + n_na_windows == n_windows"
                 % item,
                 "%d + %d != %d" % (effect["n_interference_windows"],
                                    effect["n_na_windows"], n_gate))

    # A false pass: the run claims eff_pass while the actual gate is not met.
    if eff_pass and (want_med < ACTUAL_MEDIAN_DB or want_frac < ACTUAL_FRAC):
        fail("%s: gates.eff_pass is consistent with the actual gate" % item,
             "eff_pass=true but actual median=%.4f dB (<%.1f) or frac>=3dB="
             "%.4f (<%.2f)" % (want_med, ACTUAL_MEDIAN_DB, want_frac, ACTUAL_FRAC))


# ---------------------------------------------------------------------------
# R04: raw (non-L2-normalised) target protection
# ---------------------------------------------------------------------------


def target_lookup(window, name):
    """Find an R04 target field, returning (value, source-label)."""
    for container in TARGET_CONTAINERS:
        block = window.get(container)
        if isinstance(block, dict) and name in block:
            return block[name], "%s.%s" % (container, name)
    if name in window:
        return window[name], name
    return None, None


def target_roi_of(window):
    lo, _ = target_lookup(window, "target_lo")
    hi, _ = target_lookup(window, "target_hi")
    if is_int(lo) and is_int(hi) and 0 <= lo < hi:
        return lo, hi
    roi, _ = target_lookup(window, "target_roi")
    if isinstance(roi, (list, tuple)) and len(roi) == 2 \
            and all(is_int(v) for v in roi) and 0 <= roi[0] < roi[1]:
        return int(roi[0]), int(roi[1])
    return None, None


def raw_target_metrics(window, arrays):
    """Recompute the raw-CIR target amp/phase/peak from archived arrays."""
    cl, cs = arrays.get("cir_raw_clean"), arrays.get("cir_raw_sic")
    if cl is None or cs is None:
        return None, "no cir_raw_clean/cir_raw_sic arrays"
    lo, hi = target_roi_of(window)
    if lo is None:
        return None, "no fixed target_lo/target_hi ROI"
    if hi > len(cl) or hi > len(cs) or len(cl) != len(cs):
        return None, ("target ROI [%d,%d) outside the raw CIR (%d/%d taps)"
                      % (lo, hi, len(cl), len(cs)))
    seg_c = cl[lo:hi]
    seg_s = cs[lo:hi]
    t = lo + int(np.argmax(np.abs(seg_c)))
    amp_c = abs(complex(cl[t]))
    amp_s = abs(complex(cs[t]))
    if amp_c <= 0:
        return None, "clean raw CIR target tap is zero"
    amp_err_db = (20.0 * math.log10(amp_s / amp_c) if amp_s > 0.0 else None)
    dphi = math.atan2(cs[t].imag, cs[t].real) \
        - math.atan2(cl[t].imag, cl[t].real)
    dphi = (dphi + math.pi) % (2.0 * math.pi) - math.pi
    phase_err_deg = math.degrees(dphi)
    peak_c = lo + int(np.argmax(np.abs(seg_c)))
    peak_s = lo + int(np.argmax(np.abs(seg_s)))
    return {
        "target_tap": t,
        "target_amp_err_db": amp_err_db,
        "target_phase_err_deg": phase_err_deg,
        "target_peak_shift": peak_s - peak_c,
        "cir_raw_target_amp_db": 20.0 * math.log10(amp_s) if amp_s > 0 else None,
        "roi": [lo, hi],
    }, None


def check_raw_target_metrics(run_dir, windows, summary, prefix):
    item = "%s/windows.jsonl raw target (R04)" % prefix
    sic_windows = [w for w in windows if isinstance(w.get("recon"), dict)]
    if not sic_windows:
        return
    # The target ROI must be fixed across the run (never re-searched).
    rois = set()
    for window in windows:
        roi = target_roi_of(window)
        if roi[0] is not None:
            rois.add(roi)
    if len(rois) > 1:
        fail("%s: target ROI is fixed across the run" % item,
             "windows declare different ROIs %s" % sorted(rois)[:5])

    declared_windows = [w for w in windows
                        if target_lookup(w, "target_amp_err_db")[0] is not None]
    if not declared_windows:
        # No raw target metrics: this is only a FAIL when the run still claims a
        # target gate (from the normalized-CIR target_amp) or claims eff_pass.
        effect = summary.get("effect") if isinstance(summary, dict) else None
        gates = summary.get("gates") if isinstance(summary, dict) else None
        claims = isinstance(effect, dict) and (
            is_num(effect.get("target_amp_err_db_p95"))
            or is_num(effect.get("target_amp_err_db_median"))
            or is_num(effect.get("target_peak_shift_max")))
        claims = claims or (isinstance(gates, dict)
                            and gates.get("eff_pass") is True)
        if claims:
            norm = [w for w in windows
                    if isinstance(w.get("clean"), dict)
                    and scalar_or_pair_mag(w["clean"].get("target_amp"))
                    is not None]
            fail("%s: raw target metrics are present" % item,
                 "the run claims a target gate but carries no "
                 "target_amp_err_db / target_phase_err_deg / target_peak_shift; "
                 "only the normalized-CIR target_amp (%d window(s)) is present"
                 % len(norm))
        else:
            skip("%s: raw target metrics are present" % item,
                 "no raw target block and no target gate claimed; target "
                 "protection is not measured (must be in not_run[])")
        return
    ok("%s: raw target metrics are present" % item,
       "%d window(s)" % len(declared_windows))

    npz_cache = {}
    recomputed = 0
    amp_errs, shifts = [], []
    for window in windows:
        if window_uses_truth(window):
            continue
        wid = window.get("window_id")
        where = "%s window_id=%r" % (item, wid)
        amp_decl, _ = target_lookup(window, "target_amp_err_db")
        ph_decl, _ = target_lookup(window, "target_phase_err_deg")
        sh_decl, _ = target_lookup(window, "target_peak_shift")
        if is_num(amp_decl):
            amp_errs.append(float(amp_decl))
        if is_int(sh_decl):
            shifts.append(int(sh_decl))
        arrays = load_recon_arrays(run_dir, wid, npz_cache)
        if arrays is None or "error" in arrays:
            continue
        got, why = raw_target_metrics(window, arrays)
        if got is None:
            continue
        recomputed += 1
        got_amp = got["target_amp_err_db"]
        if got_amp is None:
            if amp_decl is None:
                ok("%s: target_amp_err_db is null (SIC target tap is zero)"
                   % where, "declared null")
            else:
                fail("%s: recomputed target_amp_err_db (raw CIR) matches"
                     % where,
                     "recomputed is undefined (SIC target tap is zero) "
                     "declared=%r" % (amp_decl,))
        elif is_num(amp_decl) and abs(float(amp_decl) - got_amp) \
                <= TARGET_AMP_TOL_DB:
            ok("%s: recomputed target_amp_err_db (raw CIR) matches" % where,
               "%.4f dB" % got_amp)
        else:
            fail("%s: recomputed target_amp_err_db (raw CIR) matches" % where,
                 "recomputed=%.4f declared=%r" % (got_amp, amp_decl))
        if is_num(ph_decl) and abs(float(ph_decl)
                                   - got["target_phase_err_deg"]) <= TARGET_PHASE_TOL_DEG:
            ok("%s: recomputed target_phase_err_deg (raw CIR) matches" % where,
               "%.4f deg" % got["target_phase_err_deg"])
        else:
            fail("%s: recomputed target_phase_err_deg (raw CIR) matches" % where,
                 "recomputed=%.4f declared=%r"
                 % (got["target_phase_err_deg"], ph_decl))
        if is_int(sh_decl) and int(sh_decl) == got["target_peak_shift"]:
            ok("%s: recomputed target_peak_shift (raw CIR) matches" % where,
               "%d" % got["target_peak_shift"])
        else:
            fail("%s: recomputed target_peak_shift (raw CIR) matches" % where,
                 "recomputed=%d declared=%r"
                 % (got["target_peak_shift"], sh_decl))

    if declared_windows and recomputed == 0:
        fail("%s: run provides raw CIR arrays to recompute the target" % item,
             "%d window(s) declare raw target metrics but no recon/window_*.npz "
             "with cir_raw_clean/cir_raw_sic is readable under %s"
             % (len(declared_windows), run_dir))

    # Aggregates + false-pass detection.
    effect = summary.get("effect") if isinstance(summary, dict) else None
    gates = summary.get("gates") if isinstance(summary, dict) else None
    if amp_errs:
        want_p95 = percentile(amp_errs, 95)
        declared_p95 = effect.get("target_amp_err_db_p95") \
            if isinstance(effect, dict) else None
        # Normalized-CIR substitute: recompute the legacy p95 and reject it.
        norm_amps = []
        for window in windows:
            if window_uses_truth(window):
                continue
            clean, sic = window.get("clean"), window.get("sic")
            a = scalar_or_pair_mag(clean.get("target_amp")) \
                if isinstance(clean, dict) else None
            b = scalar_or_pair_mag(sic.get("target_amp")) \
                if isinstance(sic, dict) else None
            if a and b and a > 0 and b > 0:
                norm_amps.append(20.0 * math.log10(b / a))
        norm_p95 = percentile(norm_amps, 95) if norm_amps else None
        if is_num(declared_p95):
            if abs(float(declared_p95) - want_p95) <= TARGET_AMP_TOL_DB:
                ok("%s: effect.target_amp_err_db_p95 is the raw-CIR aggregate"
                   % item, "%.4f dB" % want_p95)
            elif is_num(norm_p95) and abs(float(declared_p95) - norm_p95) \
                    <= TARGET_AMP_TOL_DB:
                fail("%s: effect.target_amp_err_db_p95 is the raw-CIR aggregate"
                     % item,
                     "declared=%r matches the normalized-CIR p95=%.4f, not the "
                     "raw p95=%.4f" % (declared_p95, norm_p95, want_p95))
            else:
                fail("%s: effect.target_amp_err_db_p95 is the raw-CIR aggregate"
                     % item, "declared=%r recomputed(raw)=%.4f norm=%.4f"
                     % (declared_p95, want_p95,
                        norm_p95 if is_num(norm_p95) else float("nan")))
        frac_ok = sum(1.0 for a in amp_errs if abs(a) <= TARGET_GATE_AMP_DB) \
            / len(amp_errs)
        shift_ok = (sum(1.0 for s in shifts if abs(s) <= TARGET_GATE_SHIFT)
                    / len(shifts)) if shifts else None
        if isinstance(gates, dict) and gates.get("eff_pass") is True:
            if frac_ok < 0.95:
                fail("%s: gates.eff_pass is consistent with the raw target gate"
                     % item,
                     "eff_pass=true but only %.4f of windows have |amp err| "
                     "<= %.1f dB" % (frac_ok, TARGET_GATE_AMP_DB))
            elif shift_ok is not None and shift_ok < 0.95:
                fail("%s: gates.eff_pass is consistent with the raw target gate"
                     % item,
                     "eff_pass=true but only %.4f of windows have |peak shift| "
                     "<= %d" % (shift_ok, TARGET_GATE_SHIFT))


# ---------------------------------------------------------------------------
# R02: effective runtime config + one case/config/template/input length
# ---------------------------------------------------------------------------
# The rectification order (R02) forbids the effect profile and the performance
# profile from silently running different cases/configs/templates/input lengths,
# and forbids silently widening ``validation_reps`` (or any config) to cover a
# subtraction the math does not actually support.  The verifier therefore
# requires:
#   * a declared EFFECTIVE runtime config and a sha256 hash for it;
#   * a per-implementation declaration (functional / core / GR / native) of the
#     same case / config / template / input length, with any difference
#     explicitly recorded (an unflagged difference is a FAIL);
#   * every window whose resolved config differs from the effective nominal to
#     record the override (a silent ``validation_reps`` bump is a FAIL);
#   * the new named quantities (fit length, estimation range, online-validation
#     length, independent-eval length, signal-evidenced start/stop, subtraction
#     mask) present and internally consistent, with the SEARCH ROI kept separate
#     from the SUBTRACTION ROI;
#   * a declared config that is unsatisfiable (fit+validation > sync_reps,
#     validation_reps < 8, fit_reps <= 0) is a FAIL, never silently rewritten.

EFFECTIVE_CONFIG_KEYS = ("effective_config", "effective_runtime_config",
                         "runtime_config", "resolved_config", "config")
EFFECTIVE_HASH_KEYS = ("effective_config_hash", "effective_config_sha256",
                       "runtime_config_hash", "resolved_config_hash",
                       "config_hash")
IMPL_CONFIG_KEYS = ("implementation_configs", "layer_configs",
                    "declared_configs", "config_layers")
# functional = effect/app path, core = SIC math core, gr = GNU Radio chain,
# native = CG400 native benchmark.  Aliases keep the check usable across names.
LAYER_ALIASES = {
    "functional": ("functional", "app", "python", "effect", "functional_path",
                   "effect_path"),
    "core": ("core", "cxx", "cpp", "sic_core", "core_path", "math_core"),
    "gr": ("gr", "gnuradio", "flowgraph", "pipeline", "gr_path"),
    "native": ("native", "cg400", "benchmark", "native_path",
               "native_benchmark"),
}
LAYER_CASE_KEYS = ("case_id", "case", "nominal_case", "profile_id",
                   "case_name")
LAYER_HASH_KEYS = ("effective_config_hash", "config_hash", "config_sha256",
                   "config_hash_value", "hash")
LAYER_TEMPLATE_KEYS = ("template_id", "template", "template_hash",
                       "template_sha256")
LAYER_LENGTH_KEYS = ("input_length", "input_len", "input_samples",
                     "input_length_samples", "samples", "window_samples")
CONFIG_DIFF_KEYS = ("differences", "differs", "diff", "deviations",
                    "overrides", "difference_note", "recorded_differences",
                    "differs_from_effective")
RUN_DIFF_KEYS = ("config_differences", "declared_differences",
                 "recorded_config_differences", "config_deviation_notes",
                 "implementation_config_differences")
# A resolved-config source string that carries no information: if a window's
# value differs from the effective nominal and its provenance is one of these,
# the change was silent.
NOMINAL_SOURCE_MARKERS = {"preset", "default", "nominal", "config", "declared",
                          "base", "spec", ""}

# R02/R06 region + named-quantity lookups.
REGION_CONTAINERS = ("regions", "region", "recon", "config", "sic_config",
                     "sic_state", "sic")
FIT_LEN_KEYS = ("fit_len", "fit_length", "fit_len_samples", "fit_samples",
                "fit_reps_samples")
EST_LO_KEYS = ("estimation_lo", "estimation_start", "estimate_lo",
               "estimation_region_lo", "fit_lo")
EST_HI_KEYS = ("estimation_hi", "estimation_end", "estimate_hi",
               "estimation_region_hi", "fit_hi")
ONLINE_LEN_KEYS = ("online_validation_len", "online_validation_length",
                   "online_validation_len_samples", "online_gate_len",
                   "validation_len", "online_len")
ONLINE_LO_KEYS = ("online_validation_lo", "online_validation_start",
                  "online_gate_lo", "validation_lo", "online_lo")
ONLINE_HI_KEYS = ("online_validation_hi", "online_validation_end",
                  "online_gate_hi", "validation_hi", "online_hi")
EVAL_LEN_KEYS = ("eval_len", "eval_length", "independent_eval_len",
                 "independent_eval_length", "eval_len_samples")
EVAL_LO_KEYS = ("eval_lo", "independent_eval_lo", "evaluation_lo")
EVAL_HI_KEYS = ("eval_hi", "independent_eval_hi", "evaluation_hi")
SIGNAL_START_KEYS = ("signal_start", "signal_evidenced_start",
                     "evidence_start", "observed_start", "support_start",
                     "support_lo")
SIGNAL_STOP_KEYS = ("signal_stop", "signal_evidenced_stop", "evidence_stop",
                    "observed_stop", "support_stop", "support_hi")
SEARCH_ROI_KEYS = ("search_roi", "comm_search_roi", "candidate_roi",
                   "search_region", "roi")
SUBTRACTION_ROI_KEYS = ("subtraction_roi", "sub_roi", "sub_mask",
                        "applied_subtraction_roi", "subtraction_region")
GATE_LO_KEYS = ("gate_lo", "gate_region_lo", "online_gate_lo", "commit_gate_lo")
GATE_HI_KEYS = ("gate_hi", "gate_region_hi", "online_gate_hi", "commit_gate_hi")
GATE_KEYS_KEYS = ("gate_keys", "gate_metrics", "gate_inputs",
                  "gate_metric_keys", "gate_input_metrics")
ONLINE_NMSE_KEYS = ("online_nmse", "online_validation_nmse", "validation_nmse")
ONLINE_SUPP_KEYS = ("online_suppression_db", "online_validation_suppression_db",
                    "validation_suppression_db")
REPS_SOURCE_KEYS = ("fit_reps_source", "validation_reps_source",
                    "config_source", "reps_source", "source")
# Metric names that describe the FINAL evaluation region; they must never gate
# a commit (the gate must be decided on the online/estimation region only).
EVAL_ONLY_METRIC_NAMES = ("nmse", "recon_nmse", "suppression_db",
                          "recon_suppression_db", "actual_nmse",
                          "actual_suppression_db", "eval_nmse",
                          "eval_suppression_db")


def _first_key(mapping, keys):
    if not isinstance(mapping, dict):
        return None, None
    for key in keys:
        if key in mapping and mapping[key] is not None:
            return mapping[key], key
    return None, None


def _nested_lookup(obj, keys, subkeys=("sic", "sic_config", "runtime",
                                       "generator", "config", "effective")):
    """Find a config value at the top level or under a known sub-object."""
    value, key = _first_key(obj, keys)
    if value is not None:
        return value, key
    if isinstance(obj, dict):
        for sub in subkeys:
            block = obj.get(sub)
            if isinstance(block, dict):
                value, key = _first_key(block, keys)
                if value is not None:
                    return value, "%s.%s" % (sub, key)
    return None, None


def _config_hash(value):
    """Normalize a hash-like value (string or {hash:...}) to a sha256 or None."""
    if isinstance(value, str):
        return normalize_hash(value), "string"
    if isinstance(value, dict):
        for key in ("hash", "sha256", "config_hash", "value", "digest"):
            if is_hex64(value.get(key)):
                return normalize_hash(value[key]), key
    return None, None


def _region_lookup(window, keys):
    """Look up a region/named quantity across the known containers."""
    for container in REGION_CONTAINERS:
        block = window.get(container)
        if isinstance(block, dict):
            for key in keys:
                if block.get(key) is not None:
                    return block[key], "%s.%s" % (container, key)
    for key in keys:
        if window.get(key) is not None:
            return window[key], key
    return None, None


def _int_pair(value):
    if isinstance(value, (list, tuple)) and len(value) == 2 \
            and all(is_int(v) for v in value):
        return int(value[0]), int(value[1])
    return None


def search_roi_of(window):
    """The declared candidate SEARCH ROI [lo,hi) (never the subtraction mask)."""
    value, path = _region_lookup(window, SEARCH_ROI_KEYS)
    pair = _int_pair(value)
    if pair is not None:
        return pair, path
    lo, _ = _region_lookup(window, ("search_roi_lo", "candidate_lo"))
    hi, _ = _region_lookup(window, ("search_roi_hi", "candidate_hi"))
    if is_int(lo) and is_int(hi):
        return (int(lo), int(hi)), "search_roi_lo/hi"
    return None, None


def subtraction_roi_of(window):
    """The applied SUBTRACTION ROI [lo,hi) (recon.sub_lo/sub_hi or a pair)."""
    value, path = _region_lookup(window, SUBTRACTION_ROI_KEYS)
    pair = _int_pair(value)
    if pair is not None:
        return pair, path
    lo, _ = _region_lookup(window, ("sub_lo", "mask_lo"))
    hi, _ = _region_lookup(window, ("sub_hi", "mask_hi"))
    if is_int(lo) and is_int(hi):
        return (int(lo), int(hi)), "recon.sub_lo/sub_hi"
    return None, None


def _region(window, lo_keys, hi_keys, len_keys):
    """Return (lo, hi, length, path).  Length is derived from the range if the
    explicit length field is absent (equivalent quantity)."""
    lo, lo_path = _region_lookup(window, lo_keys)
    hi, hi_path = _region_lookup(window, hi_keys)
    length, len_path = _region_lookup(window, len_keys)
    if not (is_int(lo) and is_int(hi)):
        return None, None, (int(length) if is_int(length) else None), \
            "%s .. %s" % (lo_path, hi_path)
    return int(lo), int(hi), (int(length) if is_int(length) else int(hi) - int(lo)), \
        "%s .. %s" % (lo_path, hi_path)


def _intervals_overlap(a, b):
    return a[0] < b[1] and b[0] < a[1]


def _within(inner, outer):
    return outer[0] <= inner[0] and inner[1] <= outer[1]


def _recorded_config_difference(manifest, found):
    """True when the run explicitly records a cross-implementation difference."""
    for key in RUN_DIFF_KEYS:
        value = manifest.get(key) if isinstance(manifest, dict) else None
        if isinstance(value, (list, dict)) and value:
            return True
        if is_nonempty_str(value):
            return True
    for entry in (found or {}).values():
        if isinstance(entry, dict):
            for key in CONFIG_DIFF_KEYS:
                value = entry.get(key)
                if isinstance(value, (list, dict)) and value:
                    return True
                if is_nonempty_str(value):
                    return True
    return False


def _layer_identity(entry):
    """(case, config_hash, template, input_length) for one implementation."""
    if isinstance(entry, str):
        return (None, normalize_hash(entry), None, None)
    if not isinstance(entry, dict):
        return (None, None, None, None)
    case, _ = _first_key(entry, LAYER_CASE_KEYS)
    config_hash = None
    for key in LAYER_HASH_KEYS:
        if key in entry and entry[key] is not None:
            config_hash, _ = _config_hash(entry[key])
            if config_hash is not None:
                break
    template, _ = _first_key(entry, LAYER_TEMPLATE_KEYS)
    if isinstance(template, dict):
        template, _ = _config_hash(template)
    length, _ = _first_key(entry, LAYER_LENGTH_KEYS)
    return (case, config_hash, template, length)


def check_effective_config(manifest, windows, summary, prefix):
    item = "%s run_manifest.json effective config (R02)" % prefix
    if not isinstance(manifest, dict):
        return
    sic_mode = manifest.get("sic_mode")
    if sic_mode is None and isinstance(summary, dict):
        sic_mode = summary.get("sic_mode")
    if sic_mode == "off":
        return
    if sic_mode is None:
        sic_mode = (manifest.get("preset") or "")
    # A SIC run must declare its effective runtime config; an unsupported preset
    # would have been rejected, so any non-off SIC run is in scope.
    config, cpath = _first_key(manifest, EFFECTIVE_CONFIG_KEYS)
    if not isinstance(config, dict) or not config:
        fail("%s: effective runtime config is present and non-empty" % item,
             "manifest lacks one of %s" % (list(EFFECTIVE_CONFIG_KEYS),))
        config = {}
    else:
        ok("%s: effective runtime config is present and non-empty" % item,
           "%s (%d keys)" % (cpath, len(config) if isinstance(config, dict)
                             else 0))

    hvalue, hkey = _first_key(manifest, EFFECTIVE_HASH_KEYS)
    if hvalue is None:
        hvalue, sub = _config_hash(config)
        hkey = "config.%s" % sub if sub else None
    if is_hex64(hvalue):
        ok("%s: effective config hash is a sha256 hex" % item,
           "%s=%s..." % (hkey, normalize_hash(hvalue)[:16]))
    else:
        fail("%s: effective config hash is a sha256 hex" % item,
             "got %r (from %r)" % (hvalue, hkey))
    eff_hash = normalize_hash(hvalue) if is_hex64(hvalue) else None

    # Unsatisfiable configs must be rejected, never silently rewritten.
    fit, fit_path = _nested_lookup(config, ("fit_reps", "fit_length_reps"))
    val, val_path = _nested_lookup(config, ("validation_reps",
                                            "online_validation_reps"))
    sync, sync_path = _nested_lookup(config, ("sync_reps", "sensing_sync_reps"))
    if is_int(fit):
        if fit > 0:
            ok("%s: effective fit_reps is positive" % item, "%s=%d"
               % (fit_path, fit))
        else:
            fail("%s: effective fit_reps is positive" % item, "%s=%r"
                 % (fit_path, fit))
    if is_int(val):
        if val >= 8:
            ok("%s: effective validation_reps is >= 8" % item, "%s=%d"
               % (val_path, val))
        elif _s1_zero_validation_ok(config):
            ok("%s: effective validation_reps == 0 is the S1 default" % item,
               "%s=0 with evaluation_reps=%r (S1 offline evaluation)"
               % (val_path, _nested_lookup(config, ("evaluation_reps",
                                                    "eval_reps"))[0]))
        else:
            fail("%s: effective validation_reps is >= 8" % item, "%s=%r"
                 % (val_path, val))
    if is_int(fit) and is_int(val) and is_int(sync):
        if fit + val <= sync:
            ok("%s: effective fit_reps+validation_reps <= sync_reps" % item,
               "%d+%d<=%d" % (fit, val, sync))
        else:
            fail("%s: effective fit_reps+validation_reps <= sync_reps" % item,
                 "unsatisfiable %d+%d>%d (%s/%s/%s)"
                 % (fit, val, sync, fit_path, val_path, sync_path))

    layers, lkey = _first_key(manifest, IMPL_CONFIG_KEYS)
    if not isinstance(layers, dict) or not layers:
        fail("%s: functional/core/GR/native configs are declared" % item,
             "manifest lacks one of %s" % (list(IMPL_CONFIG_KEYS),))
        return
    ok("%s: implementation configs are declared" % item, lkey)
    found = {}
    missing = []
    for layer, aliases in LAYER_ALIASES.items():
        entry = None
        for alias in aliases:
            if alias in layers:
                entry = layers[alias]
                break
        if entry is None:
            missing.append(layer)
        else:
            found[layer] = entry
    if missing:
        unresolved = [l for l in missing if not summary_key_mentioned(summary, l)]
        if unresolved:
            fail("%s: functional/core/GR/native are declared" % item,
                 "missing %s with no not_run[] entry" % unresolved)
        else:
            skip("%s: functional/core/GR/native are declared" % item,
                 "missing %s is declared not_run" % missing)

    identities = {layer: _layer_identity(entry)
                  for layer, entry in found.items()}
    recorded = _recorded_config_difference(manifest, found)
    labels = ("case", "config_hash", "template", "input_length")
    for index, label in enumerate(labels):
        values = {layer: ident[index] for layer, ident in identities.items()}
        absent = sorted(layer for layer, value in values.items()
                        if value is None)
        if absent:
            fail("%s: %s is declared for every implementation" % (item, label),
                 "missing for %s" % absent)
            continue
        if len(set(values.values())) <= 1:
            ok("%s: %s agrees across functional/core/GR/native" % (item, label),
               "%r" % (next(iter(values.values())),))
        elif recorded:
            skip("%s: %s differs across implementations (recorded)" % (item, label),
                 str(values))
        else:
            fail("%s: %s agrees across functional/core/GR/native" % (item, label),
                 "unrecorded difference %s" % (values,))
    # The effective hash must be the identity the layers ran under.
    if eff_hash is not None:
        bad = sorted(layer for layer, ident in identities.items()
                     if ident[1] is not None and ident[1] != eff_hash)
        if bad:
            if recorded:
                skip("%s: layers ran under the effective config hash" % item,
                     "differing (recorded): %s" % bad)
            else:
                fail("%s: layers ran under the effective config hash" % item,
                     "%s do not match %s..." % (bad, eff_hash[:16]))
        else:
            ok("%s: layers ran under the effective config hash" % item,
               eff_hash[:16] + "...")


def _window_records_override(window, name, manifest, found):
    """True when a window's non-nominal value is explicitly recorded."""
    source_keys = tuple(["%s_source" % name]) + REPS_SOURCE_KEYS
    for container in ("config", "runtime_config", "sic_config") + REGION_CONTAINERS:
        block = window.get(container)
        if isinstance(block, dict):
            for key in source_keys:
                value = block.get(key)
                if is_nonempty_str(value) \
                        and norm_text(value) not in NOMINAL_SOURCE_MARKERS:
                    return True, "%s.%s" % (container, key)
    for key in source_keys:
        value = window.get(key)
        if is_nonempty_str(value) \
                and norm_text(value) not in NOMINAL_SOURCE_MARKERS:
            return True, key
    if isinstance(window.get("config_differences"), (list, dict)) \
            and window["config_differences"]:
        return True, "config_differences"
    if _recorded_config_difference(manifest or {}, found or {}):
        return True, "manifest.config_differences"
    return False, None


def check_window_config_recorded(manifest, windows, summary, prefix):
    item = "%s/windows.jsonl config recording (R02)" % prefix
    if not isinstance(manifest, dict):
        return
    sic_mode = manifest.get("sic_mode")
    if sic_mode is None and isinstance(summary, dict):
        sic_mode = summary.get("sic_mode")
    if sic_mode == "off":
        return
    config, _ = _first_key(manifest, EFFECTIVE_CONFIG_KEYS)
    config = config if isinstance(config, dict) else {}
    layers, _ = _first_key(manifest, IMPL_CONFIG_KEYS)
    layers = layers if isinstance(layers, dict) else {}
    nominal = {}
    for name, keys in (("fit_reps", ("fit_reps", "fit_length_reps")),
                       ("validation_reps", ("validation_reps",
                                            "online_validation_reps")),
                       ("sic_k", ("sic_k", "k_paths")),
                       ("sync_reps", ("sync_reps", "sensing_sync_reps"))):
        nominal[name], _ = _nested_lookup(config, keys)
    for window in windows:
        wid = window.get("window_id")
        where = "%s window_id=%r" % (item, wid)
        resolved = {}
        for name in ("fit_reps", "validation_reps", "sic_k"):
            value, _ = _first_key(window, (name,))
            if is_int(value):
                resolved[name] = value
        # unsatisfiable per-window config
        fit = resolved.get("fit_reps")
        val = resolved.get("validation_reps")
        sync = window.get("sensing_sync_reps")
        if is_int(fit) and is_int(val) and is_int(sync) and fit + val > sync:
            fail("%s: fit_reps+validation_reps <= sensing_sync_reps" % where,
                 "unsatisfiable %d+%d>%d" % (fit, val, sync))
        for name, value in resolved.items():
            target = nominal.get(name)
            if not is_int(target) or value == target:
                continue
            recorded, source = _window_records_override(window, name, manifest,
                                                        layers)
            if recorded:
                ok("%s: %s=%d differs from effective %d (recorded at %s)"
                   % (where, name, value, target, source),
                   "recorded override")
            else:
                fail("%s: %s change is recorded, not silent" % (where, name),
                     "window %s=%d != effective %s=%d with no recorded "
                     "source/difference" % (name, value, name, target))


# ---------------------------------------------------------------------------
# R06: estimation / online-validation / independent-eval separation
# ---------------------------------------------------------------------------
# The three regions must be explicit and the final evaluation region must be
# disjoint from BOTH the estimation region (all data used to fit CFO / paths /
# fractional delay / gain) and the online-validation region (the commit gate).
# The gate keys must be computed on the online region, never on the final
# evaluation region (changing eval content must not move a frozen gate).


def _check_regions(window, where):
    """Return (est, online, eval) ranges when well-formed; report failures."""
    est_lo, est_hi, est_len, est_path = _region(window, EST_LO_KEYS, EST_HI_KEYS,
                                                FIT_LEN_KEYS)
    on_lo, on_hi, on_len, on_path = _region(window, ONLINE_LO_KEYS,
                                            ONLINE_HI_KEYS, ONLINE_LEN_KEYS)
    ev_lo, ev_hi, ev_len, ev_path = _region(window, EVAL_LO_KEYS, EVAL_HI_KEYS,
                                            EVAL_LEN_KEYS)
    est = online = ev = None

    if est_lo is None:
        fail("%s: estimation region [lo,hi) is declared" % where,
             "could not read %s" % (est_path,))
    elif not (0 <= est_lo < est_hi):
        fail("%s: estimation region is a non-empty [lo,hi)" % where,
             "got [%r,%r) via %s" % (est_lo, est_hi, est_path))
    else:
        est = (est_lo, est_hi)
        ok("%s: estimation region is a non-empty [lo,hi)" % where,
           "[%d,%d) fit_len=%d" % (est_lo, est_hi, est_len))
        explicit, _ = _region_lookup(window, FIT_LEN_KEYS)
        if is_int(explicit) and explicit != est_hi - est_lo:
            fail("%s: declared fit length matches the estimation range" % where,
                 "fit_len=%r range=%d" % (explicit, est_hi - est_lo))

    online_zero = _s1_online_declared_zero(window)
    if online_zero:
        # S1 default: online_validation_reps == 0.  The gate then runs on the
        # training/estimation region only (checked below), never on eval data.
        if is_int(on_lo) and is_int(on_hi) and on_hi < on_lo:
            fail("%s: zero-length online-validation range is well-formed"
                 % where, "lo=%r hi=%r" % (on_lo, on_hi))
        else:
            online = None
            ok("%s: online_validation_reps == 0 (S1 offline evaluation)" % where,
               "online range %s" % ("absent" if on_lo is None
                                    else "[%d,%d)" % (on_lo, on_hi)))
    elif on_lo is None:
        fail("%s: online-validation region [lo,hi) is declared" % where,
             "could not read %s" % (on_path,))
    elif not (0 <= on_lo < on_hi):
        fail("%s: online-validation region is a non-empty [lo,hi)" % where,
             "got [%r,%r) via %s" % (on_lo, on_hi, on_path))
    else:
        online = (on_lo, on_hi)
        ok("%s: online-validation region is a non-empty [lo,hi)" % where,
           "[%d,%d) len=%d" % (on_lo, on_hi, on_len))
        explicit, _ = _region_lookup(window, ONLINE_LEN_KEYS)
        if is_int(explicit) and explicit != on_hi - on_lo:
            fail("%s: declared online-validation length matches the range"
                 % where, "len=%r range=%d" % (explicit, on_hi - on_lo))

    if ev_lo is None:
        fail("%s: independent-evaluation region [lo,hi) is declared" % where,
             "could not read %s" % (ev_path,))
    elif not (0 <= ev_lo < ev_hi):
        fail("%s: independent-evaluation region is a non-empty [lo,hi)" % where,
             "got [%r,%r) via %s" % (ev_lo, ev_hi, ev_path))
    else:
        ev = (ev_lo, ev_hi)
        ok("%s: independent-evaluation region is a non-empty [lo,hi)" % where,
           "[%d,%d) len=%d" % (ev_lo, ev_hi, ev_len))
        explicit, _ = _region_lookup(window, EVAL_LEN_KEYS)
        if is_int(explicit) and explicit != ev_hi - ev_lo:
            fail("%s: declared independent-eval length matches the range"
                 % where, "len=%r range=%d" % (explicit, ev_hi - ev_lo))

    # signal-evidenced support
    sig_lo, sig_path = _region_lookup(window, SIGNAL_START_KEYS)
    sig_hi, _ = _region_lookup(window, SIGNAL_STOP_KEYS)
    signal = None
    if is_int(sig_lo) and is_int(sig_hi) and sig_lo < sig_hi:
        signal = (int(sig_lo), int(sig_hi))
        ok("%s: signal-evidenced start/stop is a non-empty [lo,hi)" % where,
           "[%d,%d) via %s" % (sig_lo, sig_hi, sig_path))
    else:
        fail("%s: signal-evidenced start/stop is a non-empty [lo,hi)" % where,
             "got start=%r stop=%r" % (sig_lo, sig_hi))

    # search ROI kept separate from the subtraction ROI
    search, search_path = search_roi_of(window)
    sub, sub_path = subtraction_roi_of(window)
    if search is None:
        fail("%s: candidate SEARCH ROI is declared" % where,
             "could not read %s" % (search_path,))
        search = None
    if sub is None:
        fail("%s: SUBTRACTION ROI is declared" % where,
             "could not read %s" % (sub_path,))
        sub = None
    if search is not None and sub is not None:
        # R02: the candidate search range must not define (clip) the applied
        # subtraction.  A subtraction whose hi equals the candidate search hi is
        # the ``confirmed*sps + roi_hi`` clipping the rectification forbids;
        # legitimate runs extend the subtraction past the search range to the
        # signal-evidenced tail.
        if sub[1] == search[1]:
            fail("%s: SEARCH ROI is separate from the SUBTRACTION ROI" % where,
                 "subtraction hi %d equals the candidate search hi; both "
                 "search=%s sub=%s (the candidate range must not clip the "
                 "applied subtraction)" % (sub[1], search_path, sub_path))
        elif search_path == sub_path:
            fail("%s: SEARCH ROI is separate from the SUBTRACTION ROI" % where,
                 "search and subtraction are read from the same declaration %s"
                 % (search_path,))
        else:
            ok("%s: SEARCH ROI is separate from the SUBTRACTION ROI" % where,
               "search=[%d,%d) sub=[%d,%d) (%s / %s)"
               % (search[0], search[1], sub[0], sub[1], search_path, sub_path))
    return est, online, ev, signal


def _check_data_dependency(window, where, est, online, ev):
    if est is None or ev is None:
        return
    if _intervals_overlap(est, ev):
        fail("%s: estimation region is disjoint from the evaluation region"
             % where,
             "estimation=[%d,%d) evaluation=[%d,%d)" % (est[0], est[1],
                                                        ev[0], ev[1]))
    else:
        ok("%s: estimation region is disjoint from the evaluation region"
           % where, "%s vs %s" % (est, ev))
    if online is not None:
        if _intervals_overlap(online, ev):
            fail("%s: online-validation region is disjoint from the evaluation "
                 "region" % where,
                 "online=[%d,%d) evaluation=[%d,%d)" % (online[0], online[1],
                                                        ev[0], ev[1]))
        else:
            ok("%s: online-validation region is disjoint from the evaluation "
               "region" % where, "%s vs %s" % (online, ev))

    # The gate region must be the online region, never the eval region.  When
    # online_validation_reps == 0 (S1 default) the gate must be the training /
    # estimation region instead.
    gate_lo, gate_lo_path = _region_lookup(window, GATE_LO_KEYS)
    gate_hi, _ = _region_lookup(window, GATE_HI_KEYS)
    gate = None
    if is_int(gate_lo) and is_int(gate_hi) and gate_lo < gate_hi:
        gate = (int(gate_lo), int(gate_hi))
    if gate is None:
        fail("%s: commit-gate region is declared" % where,
             "could not read %s" % (gate_lo_path,))
    else:
        target = online if online is not None else est
        label = "online-validation" if online is not None \
            else "training/estimation (online_validation_reps==0)"
        if _intervals_overlap(gate, ev):
            fail("%s: commit-gate region is disjoint from the evaluation region"
                 % where, "gate=[%d,%d) evaluation=[%d,%d)"
                 % (gate[0], gate[1], ev[0], ev[1]))
        elif not _within(gate, target):
            fail("%s: commit gate is computed on the %s region" % (where, label),
                 "gate=[%d,%d) target=[%d,%d)"
                 % (gate[0], gate[1], target[0], target[1]))
        else:
            ok("%s: commit gate is computed on the %s region" % (where, label),
               "gate=[%d,%d)" % gate)

    keys, keys_path = _region_lookup(window, GATE_KEYS_KEYS)
    if not (isinstance(keys, (list, tuple)) and keys
            and all(is_nonempty_str(k) for k in keys)):
        fail("%s: commit-gate keys are declared" % where,
             "got %r via %s" % (keys, keys_path))
        return
    normalized = {norm_text(k) for k in keys}
    bad = sorted(name for name in EVAL_ONLY_METRIC_NAMES if name in normalized)
    if bad:
        fail("%s: no evaluation-region metric is used as a gate key" % where,
             "gate_keys=%s includes eval-only %s" % (list(keys), bad))
    else:
        ok("%s: no evaluation-region metric is used as a gate key" % where,
           ",".join(str(k) for k in keys))


def _recompute_online_metric(window, recon, arrays, where):
    """Independently recompute the online-region NMSE from the arrays when the
    run declares one.  This is the concrete data-dependency evidence: the gate
    value must come from the online region, not the evaluation region."""
    declared, _ = _region_lookup(window, ONLINE_NMSE_KEYS)
    if declared is None:
        return
    if arrays is None or "error" in arrays or "i" not in arrays \
            or "i_hat" not in arrays:
        return
    lo, hi, _, _ = _region(window, ONLINE_LO_KEYS, ONLINE_HI_KEYS,
                           ONLINE_LEN_KEYS)
    if not (is_int(lo) and is_int(hi) and lo < hi):
        return
    i, i_hat = arrays["i"], arrays["i_hat"]
    if hi > len(i) or hi > len(i_hat):
        fail("%s: online-region arrays cover the declared online window" % where,
             "[%d,%d) > %d/%d samples" % (lo, hi, len(i), len(i_hat)))
        return
    ref, est = i[lo:hi], i_hat[lo:hi]
    den = float(np.vdot(ref, ref).real)
    if den <= 0:
        return
    diff = est - ref
    want = float(np.vdot(diff, diff).real) / den
    if is_num(declared) and close(float(declared), want, 1e-6, 1e-6):
        ok("%s: recomputed online_nmse over the online region matches" % where,
           "%.6g" % want)
    else:
        fail("%s: recomputed online_nmse over the online region matches" % where,
             "recomputed=%.6g declared=%r" % (want, declared))


def check_regions_r02_r06(run_dir, windows, summary, prefix):
    item = "%s/windows.jsonl regions (R02/R06)" % prefix
    sic_on = any(isinstance(w.get("recon"), dict) for w in windows)
    if not sic_on:
        return
    global _OK_AGGREGATE
    npz_cache = {}
    aggregate = {}
    _OK_AGGREGATE = aggregate
    try:
        for window in windows:
            recon = window.get("recon")
            if not isinstance(recon, dict):
                continue
            wid = window.get("window_id")
            where = "%s window_id=%r" % (item, wid)
            est, online, ev, _signal = _check_regions(window, where)
            _check_data_dependency(window, where, est, online, ev)
            arrays = load_recon_arrays(run_dir, wid, npz_cache)
            if arrays is not None and "error" not in arrays:
                _recompute_online_metric(window, recon, arrays, where)
    finally:
        _OK_AGGREGATE = None
    by_base = {}
    for name, details in aggregate.items():
        base = re.sub(r"\s*window_id=[^:]*", "", name)
        entry = by_base.setdefault(base, [0, None])
        entry[0] += len(details)
        entry[1] = details[0]
    for base in sorted(by_base):
        count, sample = by_base[base]
        record(base, "PASS", "%d window(s); e.g. %s" % (count, sample))


def discover_truth_files(run_dir):
    """All `truth.jsonl` sidecars under a run dir (app writes fixtures/truth.jsonl)."""
    found = []
    for dirpath, _dirnames, filenames in os.walk(run_dir):
        for name in filenames:
            if name == "truth.jsonl":
                found.append(os.path.join(dirpath, name))
    return sorted(set(found))


def realized_isr_db(comm):
    """Recompute the realized ISR from the truth energies and the applied scale.

    ISR = 10*log10(scale^2 * E_comm_prescale / E_clean) over the overlap.
    Returns None when the energies are absent/zero (no overlap or no signal).
    """
    e_clean = comm.get("overlap_energy_clean")
    e_pre = comm.get("overlap_energy_comm_prescale")
    scale = comm.get("scale")
    if is_num(e_clean) and float(e_clean) > 0 and is_num(e_pre) \
            and float(e_pre) > 0 and is_num(scale) and float(scale) > 0:
        return 10.0 * math.log10(float(scale) ** 2 * float(e_pre)
                                 / float(e_clean))
    return None


def check_truth_sidecar(run_dir, windows, manifest, prefix):
    item = "%s truth.jsonl" % prefix
    files = discover_truth_files(run_dir)
    if not files:
        skip("%s: present (generator sidecar)" % item,
             "not present under %s; truth recompute not possible "
             "(not a required artifact)" % run_dir)
        return
    rel = [os.path.relpath(path, run_dir) for path in files]
    ok("%s: discovered" % item,
       "%d sidecar file(s): %s" % (len(files), rel))

    records = []
    for path in files:
        where = "%s [%s]" % (item, os.path.relpath(path, run_dir))
        rows, err = load_jsonl(path)
        if rows is None:
            fail("%s: every line parses as JSON" % where, err)
            continue
        ok("%s: every line parses as JSON" % where, "%d records" % len(rows))
        for lineno, record in rows:
            if not isinstance(record, dict):
                fail("%s line %d: is a JSON object" % (where, lineno),
                     "got %r" % type(record).__name__)
                continue
            records.append((where, lineno, record))
    if not records:
        fail("%s: contains at least one truth record" % item, "0 records")
        return

    by_id = {str(w.get("window_id")): w for w in windows}
    win_run_ids = {w.get("run_id") for w in windows
                   if w.get("run_id") is not None}
    truth_run_ids = {r.get("run_id") for _, _, r in records
                     if r.get("run_id") is not None}
    if len(truth_run_ids) > 1:
        fail("%s: a single run_id across truth records" % item,
             "got %s" % sorted(truth_run_ids))
    if win_run_ids and truth_run_ids and not truth_run_ids <= win_run_ids:
        fail("%s: truth run_id matches windows.jsonl" % item,
             "truth=%s windows=%s" % (sorted(truth_run_ids),
                                      sorted(win_run_ids)))
    else:
        ok("%s: truth run_id matches windows.jsonl" % item,
           sorted(truth_run_ids)[0] if truth_run_ids else "n/a")

    template_hashes = set()
    if isinstance(manifest, dict) \
            and isinstance(manifest.get("template_hashes"), dict):
        template_hashes = {normalize_hash(v)
                           for v in manifest["template_hashes"].values()}

    checked = 0
    seen_ids = set()
    unknown = []
    for where, lineno, record in records:
        wid = str(record.get("window_id"))
        location = "%s line %d window_id=%s" % (where, lineno, wid)
        seen_ids.add(wid)
        if wid not in by_id:
            unknown.append(wid)
            fail("%s: window_id is declared in windows.jsonl" % location,
                 "unknown window_id=%r" % (record.get("window_id"),))
            continue
        checked += 1
        check_truth_record(record, by_id[wid], location, template_hashes)
    if not unknown:
        ok("%s: every truth window_id is declared in windows.jsonl" % item,
           "%d record(s) checked over %d/%d windows"
           % (checked, len(seen_ids), len(by_id)))


def check_truth_record(record, window, where, template_hashes):
    comm = record.get("comm")
    if not isinstance(comm, dict):
        fail("%s: comm is an object" % where, "got %r" % (comm,))
        return
    overlap = window.get("overlap") or {}
    rlo, rhi = comm.get("real_overlap_lo"), comm.get("real_overlap_hi")
    if is_int(rlo) and is_int(rhi):
        if overlap.get("lo") == rlo and overlap.get("hi") == rhi:
            ok("%s: truth overlap equals windows.overlap" % where,
               "[%d,%d)" % (rlo, rhi))
        else:
            fail("%s: truth overlap equals windows.overlap" % where,
                 "truth=[%r,%r) windows=[%r,%r)"
                 % (rlo, rhi, overlap.get("lo"), overlap.get("hi")))
    else:
        fail("%s: truth overlap bounds are integers" % where,
             "got lo=%r hi=%r" % (rlo, rhi))
    overlap_samples = comm.get("overlap_samples")
    if is_int(overlap_samples) and is_int(rlo) and is_int(rhi) \
            and overlap_samples != rhi - rlo:
        fail("%s: truth overlap_samples == hi-lo" % where,
             "%r != %d" % (overlap_samples, rhi - rlo))
    if record.get("sensing_code") == window.get("sensing_code") \
            and record.get("comm_code") == window.get("comm_code"):
        ok("%s: truth codes match windows" % where,
           "sensing=%r comm=%r" % (record.get("sensing_code"),
                                   record.get("comm_code")))
    else:
        fail("%s: truth codes match windows" % where,
             "truth sensing=%r comm=%r windows sensing=%r comm=%r"
             % (record.get("sensing_code"), record.get("comm_code"),
                window.get("sensing_code"), window.get("comm_code")))

    truth_isr, win_isr = comm.get("isr_db"), window.get("isr_db")
    recomputed = realized_isr_db(comm)
    if recomputed is not None:
        for label, declared in (("truth isr_db", truth_isr),
                                ("windows isr_db", win_isr)):
            if declared is None:
                fail("%s: realized ISR matches %s" % (where, label),
                     "declared null; recomputed=%.4f dB" % recomputed)
            elif close(declared, recomputed, 0.05, 1e-6):
                ok("%s: realized ISR matches %s" % (where, label),
                   "%.4f dB" % recomputed)
            else:
                fail("%s: realized ISR matches %s" % (where, label),
                     "recomputed=%.4f declared=%r" % (recomputed, declared))
    elif truth_isr is None and win_isr is None:
        ok("%s: truth isr_db matches windows (both null)" % where, "null")
    elif is_num(truth_isr) and is_num(win_isr) \
            and close(truth_isr, win_isr, 0.05, 1e-6):
        ok("%s: truth isr_db matches windows" % where, str(win_isr))
    else:
        fail("%s: truth isr_db matches windows" % where,
             "truth=%r windows=%r" % (truth_isr, win_isr))

    hashes = record.get("hashes")
    if isinstance(hashes, dict) and is_hex64(hashes.get("template_sha256")):
        if not template_hashes or normalize_hash(hashes["template_sha256"]) \
                in template_hashes:
            ok("%s: truth template hash is declared in the manifest" % where,
               hashes["template_sha256"][:16] + "...")
        else:
            fail("%s: truth template hash is declared in the manifest" % where,
                 "%s not in %s" % (hashes["template_sha256"],
                                   sorted(template_hashes)))
    else:
        fail("%s: truth.hashes.template_sha256 is a sha256" % where,
             "got %r" % (hashes,))


# ---------------------------------------------------------------------------
# 8. uses_truth discipline
# ---------------------------------------------------------------------------


def check_uses_truth(windows, summary, prefix):
    item = "%s uses_truth discipline" % prefix
    truth_windows = [w for w in windows
                     if isinstance(w.get("sic_state"), dict)
                     and w["sic_state"].get("uses_truth") is True]
    for window in truth_windows:
        wid = window.get("window_id")
        where = "%s window_id=%r" % (item, wid)
        flagged = (window.get("diagnostic_only") is True
                   or (isinstance(window.get("sic_state"), dict)
                       and window["sic_state"].get("diagnostic_only") is True)
                   or window.get("oracle") is True)
        if flagged:
            ok("%s: uses_truth is flagged diagnostic-only" % where, "flagged")
        else:
            fail("%s: uses_truth is flagged diagnostic-only" % where,
                 "uses_truth=true but no diagnostic-only flag")

    if not truth_windows:
        ok("%s: no uses_truth window in the effect/gate set" % item,
           "0 truth windows")
        return

    n_truth = len(truth_windows)
    if not isinstance(summary, dict):
        return
    effect = summary.get("effect")
    # Oracle windows must be counted and excluded explicitly.
    oracle_count = None
    for source in (effect, summary):
        if isinstance(source, dict):
            for key in ("oracle_windows", "n_oracle_windows", "uses_truth_windows"):
                if is_int(source.get(key)):
                    oracle_count = source[key]
                    break
            if oracle_count is not None:
                break
    if oracle_count == n_truth:
        ok("%s: summary declares the oracle window count" % item, str(n_truth))
    else:
        fail("%s: summary declares the oracle window count" % item,
             "truth windows=%d summary=%r" % (n_truth, oracle_count))

    excluded = None
    for source in (effect, summary, summary.get("gates")):
        if isinstance(source, dict) and isinstance(
                source.get("oracle_excluded", source.get("excludes_truth")), bool):
            excluded = source.get("oracle_excluded", source.get("excludes_truth"))
            break
    if excluded is True:
        ok("%s: gates declare the truth windows are excluded" % item, "true")
    else:
        fail("%s: gates exclude the uses_truth windows" % item,
             "truth windows=%d but oracle_excluded is %r" % (n_truth, excluded))

    # Recompute the effect aggregates both ways and require the exclusive one.
    if isinstance(effect, dict):
        inclusive = effect_aggregates(windows, include_truth=True)
        exclusive = effect_aggregates(windows, include_truth=False)
        for key in ("median_suppression_db", "frac_ge_3db", "nmse_median",
                    "peak_shift_max", "target_amp_err_db_p95",
                    "target_peak_shift_max", "median_actual_suppression_db",
                    "frac_actual_ge_3db", "actual_median_suppression_db",
                    "median_suppression_db_actual",
                    "actual_suppression_db_median"):
            declared = effect.get(key)
            if declared is None or key not in inclusive or key not in exclusive:
                continue
            inc, exc = inclusive[key], exclusive[key]
            if is_num(declared) and is_num(exc) and close(declared, exc):
                if is_num(inc) and not close(declared, inc) \
                        and not close(exc, inc):
                    ok("%s: effect.%s excludes the truth windows" % (item, key),
                       "declared=%.6g exclusive=%.6g inclusive=%.6g"
                       % (declared, exc, inc))
                else:
                    ok("%s: effect.%s agrees with the exclusive recompute"
                       % (item, key), "%.6g" % exc)
            elif is_num(declared) and is_num(inc) and close(declared, inc):
                fail("%s: effect.%s excludes the truth windows" % (item, key),
                     "declared=%.6g matches the inclusive aggregate" % declared)
            else:
                fail("%s: effect.%s excludes the truth windows" % (item, key),
                     "declared=%r exclusive=%r inclusive=%r"
                     % (declared, exc, inc))
        for gate in ("sic_speed_pass", "demod_speed_pass"):
            gates = summary.get("gates")
            if isinstance(gates, dict) and gates.get(gate) is True and excluded is not True:
                fail("%s: %s excludes the truth windows" % (item, gate),
                     "gate=true but uses_truth windows are not excluded")


def window_uses_truth(window):
    state = window.get("sic_state")
    if isinstance(state, dict) and state.get("uses_truth") is True:
        return True
    recon = window.get("recon")
    return isinstance(recon, dict) and recon.get("uses_truth") is True


# Effect aggregate keys the gate is built from; the coverage of each is checked
# against windows_completed.  The R01 actual-output median/frac are the main
# gate; the reconstruction keys are diagnostics.  The raw-CIR target keys are
# R04 and must be recomputed from the non-normalized CIR.
AGG_KEYS = ("median_suppression_db", "frac_ge_3db", "nmse_median",
            "peak_shift_max", "target_amp_err_db_p95",
            "median_actual_suppression_db", "frac_actual_ge_3db",
            "actual_median_suppression_db", "median_suppression_db_actual",
            "actual_suppression_db_median", "target_peak_shift_max",
            "holdout_nmse_median", "holdout_suppression_db_median")

EXCLUSION_COUNT_KEYS = ("n_excluded", "excluded_windows", "num_excluded",
                        "n_dropped", "excluded_count", "n_skipped",
                        "n_zero_interferer", "n_zero_target_amp")
EXCLUSION_REASON_KEYS = ("excluded_reasons", "exclusion_reasons", "excluded",
                         "skipped_reasons")


def effect_contributors(windows):
    """How many windows actually feed each effect aggregate."""
    counts = {key: 0 for key in AGG_KEYS}
    for window in windows:
        recon = window.get("recon")
        if isinstance(recon, dict):
            if is_num(recon.get("suppression_db")):
                counts["median_suppression_db"] += 1
                counts["frac_ge_3db"] += 1
            if is_num(recon.get("nmse")):
                counts["nmse_median"] += 1
            if is_num(recon.get("actual_suppression_db")):
                for key in ("median_actual_suppression_db", "frac_actual_ge_3db",
                            "actual_median_suppression_db",
                            "median_suppression_db_actual",
                            "actual_suppression_db_median"):
                    counts[key] += 1
            if is_num(recon.get("holdout_suppression_db")):
                counts["holdout_suppression_db_median"] += 1
            if is_num(recon.get("holdout_nmse")):
                counts["holdout_nmse_median"] += 1
        amp_raw, _ = target_lookup(window, "target_amp_err_db")
        if is_num(amp_raw):
            counts["target_amp_err_db_p95"] += 1
        shift_raw, _ = target_lookup(window, "target_peak_shift")
        if is_int(shift_raw):
            counts["target_peak_shift_max"] += 1
        clean, sic = window.get("clean"), window.get("sic")
        if isinstance(clean, dict) and isinstance(sic, dict):
            if is_int(clean.get("cir_peak_tap")) \
                    and is_int(sic.get("cir_peak_tap")):
                counts["peak_shift_max"] += 1
            a = scalar_or_pair_mag(clean.get("target_amp"))
            b = scalar_or_pair_mag(sic.get("target_amp"))
            if is_num(a) and is_num(b) and a > 0 and b > 0 \
                    and not is_num(amp_raw):
                counts["target_amp_err_db_p95"] += 1
    return counts



def explicit_exclusions(effect, summary):
    """Counted + named windows excluded from the effect gate."""
    total = 0
    reasons = []
    for source in (effect, summary):
        if not isinstance(source, dict):
            continue
        for key in EXCLUSION_COUNT_KEYS:
            value = source.get(key)
            if is_int(value):
                total += value
        for key in EXCLUSION_REASON_KEYS:
            value = source.get(key)
            if isinstance(value, list):
                reasons += [str(x) for x in value]
            elif is_nonempty_str(value):
                reasons.append(value)
    return total, reasons


def effect_aggregates(windows, include_truth):
    subset = []
    for window in windows:
        if window_uses_truth(window) and not include_truth:
            continue
        subset.append(window)
    sups, nmses, shifts, amps = [], [], [], []
    a_sups = []
    h_nmses, h_sups = [], []
    raw_amp, raw_shift = [], []
    for window in subset:
        recon = window.get("recon")
        if isinstance(recon, dict):
            if is_num(recon.get("suppression_db")):
                sups.append(float(recon["suppression_db"]))
            if is_num(recon.get("nmse")):
                nmses.append(float(recon["nmse"]))
            if is_num(recon.get("actual_suppression_db")):
                a_sups.append(float(recon["actual_suppression_db"]))
            if is_num(recon.get("holdout_nmse")):
                h_nmses.append(float(recon["holdout_nmse"]))
            if is_num(recon.get("holdout_suppression_db")):
                h_sups.append(float(recon["holdout_suppression_db"]))
        amp_decl, _ = target_lookup(window, "target_amp_err_db")
        if is_num(amp_decl):
            raw_amp.append(float(amp_decl))
        shift_decl, _ = target_lookup(window, "target_peak_shift")
        if is_int(shift_decl):
            raw_shift.append(int(shift_decl))
        clean, sic = window.get("clean"), window.get("sic")
        if isinstance(clean, dict) and isinstance(sic, dict):
            if is_int(clean.get("cir_peak_tap")) and is_int(sic.get("cir_peak_tap")):
                shifts.append(abs(sic["cir_peak_tap"] - clean["cir_peak_tap"]))
            a = scalar_or_pair_mag(clean.get("target_amp"))
            b = scalar_or_pair_mag(sic.get("target_amp"))
            if is_num(a) and is_num(b) and a > 0 and b > 0:
                amps.append(20.0 * math.log10(b / a))
    out = {
        "median_suppression_db": median(sups) if sups else None,
        "frac_ge_3db": (sum(1.0 for s in sups if s >= 3.0) / len(sups))
        if sups else None,
        "nmse_median": median(nmses) if nmses else None,
        "peak_shift_max": max(shifts) if shifts else None,
        "target_amp_err_db_p95": (percentile(raw_amp, 95) if raw_amp
                                  else (percentile(amps, 95) if amps else None)),
        "target_peak_shift_max": (max(abs(s) for s in raw_shift) if raw_shift
                                  else None),
        "median_actual_suppression_db": median(a_sups) if a_sups else None,
        "frac_actual_ge_3db": (sum(1.0 for s in a_sups if s >= 3.0)
                               / len(a_sups)) if a_sups else None,
        "holdout_nmse_median": median(h_nmses) if h_nmses else None,
        "holdout_suppression_db_median":
            median(h_sups) if h_sups else None,
    }
    # The actual-key aliases all resolve to the same recomputed aggregate.
    for key in ("actual_median_suppression_db", "median_suppression_db_actual",
                "actual_suppression_db_median"):
        out[key] = out["median_actual_suppression_db"]
    for key in ("actual_frac_ge_3db", "frac_ge_3db_actual"):
        out[key] = out["frac_actual_ge_3db"]
    return out



def cross_check_effect(summary, windows, prefix):
    item = "%s/summary.json effect" % prefix
    effect = summary.get("effect")
    if not isinstance(effect, dict) or windows is None:
        return
    non_truth = [w for w in windows if not window_uses_truth(w)]
    agg = effect_aggregates(windows, include_truth=False)

    declared_keys = [key for key in AGG_KEYS if is_num(effect.get(key))]
    completed = summary.get("windows_completed")
    oracle = effect.get("oracle_windows")
    oracle = oracle if is_int(oracle) else 0
    excluded, reasons = explicit_exclusions(effect, summary)
    n_gate = effect.get("n_windows")

    if not declared_keys:
        # No gate aggregate is claimed (SIC off / not run): nothing to cover.
        if is_int(n_gate) and n_gate != 0:
            fail("%s: effect.n_windows == 0 when no aggregate is claimed"
                 % item, "n_windows=%r" % (n_gate,))
        else:
            ok("%s: no effect aggregate is claimed" % item,
               "n_windows=%r" % (n_gate,))
        return

    # Declared-vs-recomputed values over the non-truth windows.
    for key in AGG_KEYS:
        if effect.get(key) is None:
            continue
        declared, want = effect[key], agg.get(key)
        if want is None:
            continue
        if is_num(declared) and close(declared, want, 1e-6, 1e-6):
            ok("%s: effect.%s recomputed over non-truth windows" % (item, key),
               "%.6g" % want)
        else:
            fail("%s: effect.%s recomputed over non-truth windows" % (item, key),
                 "summary=%r recomputed=%.6g" % (declared, want))

    # Coverage: the gate must run over the completed windows minus the truth
    # (oracle) windows and minus any EXPLICITLY counted + named exclusions.
    if not is_int(completed):
        fail("%s: windows_completed sizes the effect gate" % item,
             "got %r" % (completed,))
        return
    if excluded > 0 and not reasons:
        fail("%s: excluded effect windows are named, not just counted" % item,
             "excluded=%d reasons=%r" % (excluded, reasons))
    expected = max(0, completed - oracle - excluded)
    if not is_int(n_gate):
        fail("%s: effect.n_windows is present when aggregates are declared"
             % item, "missing")
    elif n_gate != expected:
        fail("%s: effect.n_windows matches the justified gate coverage" % item,
             "n_windows=%r expected=%d (completed=%d oracle=%d excluded=%d)"
             % (n_gate, expected, completed, oracle, excluded))
    else:
        ok("%s: effect.n_windows == the justified gate coverage" % item,
           "n_windows=%d (completed=%d oracle=%d excluded=%d)"
           % (n_gate, completed, oracle, excluded))

    contrib = effect_contributors(non_truth)
    need = max(1, int(math.ceil(0.95 * expected))) if expected else 0
    for key in declared_keys:
        count = contrib.get(key, 0)
        if expected == 0:
            fail("%s: effect.%s is declared over an empty gate set"
                 % (item, key), "0 gate windows")
        elif count >= expected:
            ok("%s: effect.%s covers every gate window" % (item, key),
               "%d/%d" % (count, expected))
        elif count >= need:
            ok("%s: effect.%s covers >=95%% of the gate windows" % (item, key),
               "%d/%d" % (count, expected))
        elif count + excluded >= expected and reasons:
            ok("%s: effect.%s coverage gap is an explicit exclusion"
               % (item, key), "%d/%d contributed, %d excluded (%s)"
               % (count, expected, excluded, "; ".join(reasons)[:80]))
        else:
            fail("%s: effect.%s coverage" % (item, key),
                 "contributes %d of %d gate windows with no named exclusion "
                 "(counted excluded=%d, reasons=%r)"
                 % (count, expected, excluded, reasons))


def check_not_run_declaration(run_dir, summary, prefix):
    item = "%s not_run declaration" % prefix
    if not isinstance(summary, dict):
        fail("%s: summary.json present to declare not_run items" % item,
             "missing summary; cannot verify not_run coverage")
        return
    not_run = summary.get("not_run")
    if not isinstance(not_run, list):
        fail("%s: not_run is a list" % item, "got %r" % (not_run,))
        return
    # Required artifacts are hard failures when missing; optional declared
    # items must be listed here when they were not executed.
    for name in REQUIRED_ARTIFACTS:
        if not os.path.isfile(os.path.join(run_dir, name)):
            if not summary_key_mentioned(summary, name.replace(".json", "")
                                         .replace(".jsonl", "").replace(".csv", "")):
                fail("%s: missing %s is named in not_run[]" % (item, name),
                     "%s absent and no not_run[] entry covers it" % name)
            else:
                ok("%s: missing %s is named in not_run[]" % (item, name),
                   "declared unexecuted")
    declared_items = summary.get("declared_items")
    if isinstance(declared_items, dict):
        executed = {"executed", "done", "ok", "pass", "true", True}
        for name, status in declared_items.items():
            if status in executed or status is True:
                continue
            if summary_key_mentioned(summary, name):
                ok("%s: unexecuted '%s' is in not_run[]" % (item, name),
                   str(status))
            else:
                fail("%s: unexecuted '%s' is in not_run[]" % (item, name),
                     "declared_items[%s]=%r not covered" % (name, status))


# ---------------------------------------------------------------------------
# R10: provenance binding, not_run discipline, superseded runs, MATLAB status,
#      perf/effect-case identity
# ---------------------------------------------------------------------------
# Every measurement must bind the ACTUAL code and inputs that produced it, must
# distinguish a tested-and-failed gate (``false``) from an unexecuted one
# (``null``), must make superseded runs explicit, must record whether an
# independent MATLAB run happened, and must bind performance rows to a case
# whose effect was verified.  All of these checks fail closed.

PROV_CONTAINER_KEYS = ("provenance", "build_provenance", "env", "environment")
PROV_COMMIT_KEYS = ("git_head", "commit", "git_commit", "head", "revision",
                    "commit_sha", "git_revision", "head_sha")
PROV_DIRTY_KEYS = ("git_dirty", "dirty", "worktree_dirty")
PROV_DIRTY_PATCH_KEYS = ("git_status_sha256", "dirty_patch_sha256",
                         "patch_sha256", "worktree_patch_sha256",
                         "git_diff_sha256", "status_sha256", "dirty_patch")
PROV_LIB_HASH_KEYS = ("libgnuradio_uwb_sha256", "lib_sha256", "lib_hash",
                      "library_sha256", "libuwb_sha256",
                      "gnuradio_uwb_sha256", "libgnuradio_uwb_file_sha256")
PROV_LIB_PATH_KEYS = ("libgnuradio_uwb_path", "lib_path", "library_path",
                      "gnuradio_uwb_path")
PROV_BENCH_HASH_KEYS = ("benchmark_exe_sha256", "benchmark_binary_sha256",
                        "benchmark_sha256", "benchmark_exe_hash",
                        "benchmark_binary_sha")
PROV_BENCH_PATH_KEYS = ("benchmark_exe_path", "benchmark_binary_path",
                        "benchmark_path", "executable_path")
PROV_BENCH_SRC_KEYS = ("benchmark_source_sha256", "benchmark_source_hash")
PROV_INPUT_HASH_KEYS = ("input_hashes", "input_windows", "inputs",
                        "input_window_hashes")
PROV_REFERENCE_KEYS = ("reference_run", "reference", "effect_case",
                       "effect_case_id")
PLACEHOLDER_IDS = {
    "", "unknown", "unspecified", "none", "null", "nil", "todo", "tbd", "head",
    "latest", "current", "stale", "na", "n a", "sample", "placeholder", "unset",
    "missing", "omitted", "not set", "notset", "0000000"}
EMPTY_SHA256 = hashlib.sha256(b"").hexdigest()
PRODUCTION_CLAIM_KEYS = ("production_profile", "official_evidence",
                         "is_production", "production_evidence",
                         "counts_as_evidence", "acceptance_evidence")
SYNTHETIC_INPUT_MARKERS = {"synthetic", "fake", "microbenchmark", "micro",
                           "simulated", "simulate", "prbs", "pseudo", "toy",
                           "stub", "random"}
R07_TEMPLATE_MODES = {"finite_sync_code_pulse", "finite_sync_pulse",
                      "finite_sync_code"}
MATLAB_RUN_MARKERS = {"run", "ran", "executed", "execute", "done", "passed",
                      "pass", "ok", "success", "successful", "verified",
                      "available", "yes", "true"}
MATLAB_NOTRUN_MARKERS = {"not_run", "not run", "notrun", "not-run", "untested",
                         "skipped", "skip", "none", "unavailable",
                         "not executed", "notexecuted", "no", "false"}
MATLAB_PARITY_KEYS = ("matlab_verified", "matlab_parity", "matlab_match",
                      "matlab_crosscheck", "matlab_ok", "matlab_accepted")
GATE_KEYS_R10 = ("eff_pass", "sic_speed_pass", "demod_speed_pass", "native_pass")
GATE_EVIDENCE_CONTAINERS = ("gate_evidence", "gates_evidence", "evidence",
                            "run_evidence", "tested", "tested_items",
                            "gate_provenance")
SUPERSEDED_INDEX_FILES = ("index.json", "runs.json", "superseded.json",
                          "sic-index.json", "results_index.json")


def _prov_lookup(manifest, prov, keys):
    """Find a provenance value in the ``provenance`` block or top-level."""
    for key in keys:
        if isinstance(prov, dict) and key in prov and prov[key] is not None:
            return prov[key], "provenance.%s" % key
    for key in keys:
        if isinstance(manifest, dict) and key in manifest \
                and manifest[key] is not None:
            return manifest[key], key
    return None, None


def _looks_like_placeholder_commit(text):
    if not isinstance(text, str):
        return True
    stripped = text.strip().lower()
    if norm_text(stripped) in PLACEHOLDER_IDS:
        return True
    if not re.match(r"^[0-9a-f]{7,40}$", stripped):
        return True
    if len(set(stripped)) == 1:
        return True
    return False


def _is_real_commit(value):
    return is_nonempty_str(value) and not _looks_like_placeholder_commit(value)


def _resolve_prov_path(rel, roots):
    if not is_nonempty_str(rel):
        return None
    if os.path.isabs(rel):
        return rel if os.path.isfile(rel) else None
    return resolve_referenced(rel, roots)


def _prov_lib_hash(manifest, prov):
    """The loaded gnuradio-uwb library file hash (flat field or lib_hashes)."""
    value, path = _prov_lookup(manifest, prov, PROV_LIB_HASH_KEYS)
    if value is not None:
        return value, path
    for container, label in ((prov, "provenance"), (manifest, "")):
        if not isinstance(container, dict):
            continue
        for hkey in ("lib_hashes", "library_hashes", "loaded_lib_hashes"):
            block = container.get(hkey)
            if isinstance(block, dict):
                for name, hval in block.items():
                    if "uwb" in str(name).lower() and is_hex64(hval):
                        return hval, "%s%s[%s]" % (label + "." if label else "",
                                                   hkey, name)
    return None, None


# Marker keys/values that DECLARE a manifest is a benchmark/perf run rather
# than an app run.  The app ``run_manifest.json`` (written by
# ``uwb_sim_preamble_sic.py``) never carries any of these keys -- only the
# ``sic_perf*`` benchmark summaries do -- so an app run can never be mis-scoped
# as a benchmark.  ``benchmark_exe_used=true`` is the explicit override.
BENCH_RUN_MARKER_KEYS = ("mode", "implementation", "row_kind", "run_kind",
                         "kind", "pipeline_kind", "output_kind", "result_kind",
                         "artifact_kind")
BENCH_RUN_MARKER_VALUES = {"benchmark", "perf", "performance", "core", "gr",
                           "native", "microbenchmark", "micro"}
BENCH_EXE_USED_KEYS = ("benchmark_exe_used", "benchmark_binary_used",
                       "uses_benchmark_exe", "uses_benchmark")


def _run_is_benchmark(manifest, prov):
    """True when the manifest DECLARES itself a benchmark/perf run.

    The benchmark executable's file hash is only on the benchmark row's
    execution path; an APP run (``uwb_sim_preamble_sic.py``
    functional/effect/negative/native/paced) never executes it.  Its re-hash is
    therefore a hard requirement only for a run that actually declares it is
    benchmark/perf output, and merely informational for an app run.

    Returns ``(is_benchmark, reason)``.
    """
    if not isinstance(manifest, dict):
        return False, None
    schema = manifest.get("schema")
    if is_nonempty_str(schema) and norm_text(schema).startswith("sic perf"):
        return True, "schema=%s" % schema
    used, used_path = _prov_lookup(manifest, prov, BENCH_EXE_USED_KEYS)
    if used is True:
        return True, "%s=true" % (used_path or "benchmark_exe_used")
    for key in BENCH_RUN_MARKER_KEYS:
        value = manifest.get(key)
        if isinstance(value, str) \
                and norm_text(value) in BENCH_RUN_MARKER_VALUES:
            return True, "%s=%s" % (key, value)
    return False, None


def check_provenance(manifest, run_dir, results_dir, prefix, superseded=False):
    item = "%s provenance binding (R10)" % prefix
    if not isinstance(manifest, dict):
        return
    prov = None
    prov_key = None
    for key in PROV_CONTAINER_KEYS:
        if isinstance(manifest.get(key), dict):
            prov, prov_key = manifest[key], key
            break
    if prov is None:
        prov = {}
    if prov_key is not None:
        ok("%s: provenance block present" % item, prov_key)
    else:
        fail("%s: a provenance block or top-level provenance fields exist"
             % item,
             "no %s object (a stale/omitted provenance must fail)"
             % "/".join(PROV_CONTAINER_KEYS))

    def prov_fail(message, detail):
        if superseded:
            skip(message, "superseded run: %s" % detail)
        else:
            fail(message, detail)

    roots = (run_dir, results_dir)

    # --- commit: a real identifier, not a stale placeholder ---------------
    commit, commit_path = _prov_lookup(manifest, prov, PROV_COMMIT_KEYS)
    if _is_real_commit(commit):
        ok("%s: commit is a real git identifier" % item,
           "%s=%s" % (commit_path, commit))
    else:
        prov_fail("%s: commit is a real git identifier" % item,
                  "%s=%r is a placeholder/omitted identifier (a stale "
                  "identifier must fail)" % (commit_path, commit))
    top_commit = manifest.get("commit")
    if _is_real_commit(commit) and is_nonempty_str(top_commit) \
            and str(commit).strip().lower() != str(top_commit).strip().lower():
        prov_fail("%s: provenance commit equals run_manifest.commit" % item,
                  "provenance=%r manifest=%r (one binding is stale)"
                  % (commit, top_commit))

    # --- dirty tree + dirty-patch identifier ------------------------------
    dirty, dirty_path = _prov_lookup(manifest, prov, PROV_DIRTY_KEYS)
    if not isinstance(dirty, bool):
        prov_fail("%s: dirty is a boolean" % item,
                  "%s=%r" % (dirty_path, dirty))
        dirty = manifest.get("dirty") if isinstance(manifest.get("dirty"), bool) \
            else None
    patch, patch_path = _prov_lookup(manifest, prov, PROV_DIRTY_PATCH_KEYS)
    if is_hex64(patch):
        ok("%s: dirty-patch identifier is a sha256" % item,
           "%s=%s..." % (patch_path, normalize_hash(patch)[:16]))
        if dirty is True and normalize_hash(patch) == EMPTY_SHA256:
            prov_fail("%s: dirty tree carries a non-empty dirty-patch id" % item,
                      "dirty=true but %s is the empty-tree sha256" % patch_path)
    else:
        prov_fail("%s: dirty-patch identifier is a sha256" % item,
                  "%s=%r (R10 requires a dirty-patch/worktree id; a "
                  "stale/omitted identifier must fail)" % (patch_path, patch))

    # --- loaded-library file hash (re-derived when resolvable) ------------
    lib_hash, lib_hash_path = _prov_lib_hash(manifest, prov)
    if is_hex64(lib_hash):
        ok("%s: loaded-library file hash is a sha256" % item,
           "%s=%s..." % (lib_hash_path, normalize_hash(lib_hash)[:16]))
        lib_path, _ = _prov_lookup(manifest, prov, PROV_LIB_PATH_KEYS)
        resolved = _resolve_prov_path(lib_path, roots)
        if resolved is not None:
            got = sha256_file(resolved)
            if got == normalize_hash(lib_hash):
                ok("%s: loaded library re-hashes to the manifest" % item, resolved)
            else:
                prov_fail("%s: loaded library re-hashes to the manifest" % item,
                          "on disk=%s manifest=%s (%s)"
                          % (got, lib_hash, resolved))
        elif is_nonempty_str(lib_path):
            skip("%s: loaded library re-hash" % item,
                 "%s not resolvable from this results tree; recorded hash kept"
                 % (lib_path,))
    else:
        prov_fail("%s: loaded-library file hash is present" % item,
                  "no %s / lib_hashes entry; the recorded library hash is absent"
                  % "/".join(PROV_LIB_HASH_KEYS[:3]))

    # --- benchmark binary file hash (re-derived when resolvable) ----------
    # SCOPE: the benchmark executable is on the BENCHMARK row's execution path
    # only.  An APP run (uwb_sim_preamble_sic.py functional/effect/negative/
    # native/paced) never executes it, so a benchmark binary rebuilt after the
    # app run must not fail the app's provenance.  The recorded hash is still
    # REQUIRED everywhere (it is the provenance record); only the re-hash MATCH
    # is scoped: strict for a declared benchmark/perf run, informational (a
    # recorded skip/note, never a failure) for an app run.  The loaded-library
    # hash below stays strict for every run.
    is_benchmark, bench_reason = _run_is_benchmark(manifest, prov)
    bench_hash, bench_hash_path = _prov_lookup(manifest, prov,
                                               PROV_BENCH_HASH_KEYS)
    if is_hex64(bench_hash):
        ok("%s: benchmark binary file hash is a sha256" % item,
           "%s=%s...%s" % (bench_hash_path, normalize_hash(bench_hash)[:16],
                           "" if is_benchmark
                           else " (app run: informational)"))
        bench_path, _ = _prov_lookup(manifest, prov, PROV_BENCH_PATH_KEYS)
        resolved = _resolve_prov_path(bench_path, roots)
        if resolved is not None:
            got = sha256_file(resolved)
            if got == normalize_hash(bench_hash):
                ok("%s: benchmark binary re-hashes to the manifest" % item,
                   resolved)
            elif is_benchmark:
                prov_fail("%s: benchmark binary re-hashes to the manifest" % item,
                          "on disk=%s manifest=%s (%s declares a benchmark run)"
                          % (got, bench_hash, bench_reason))
            else:
                skip("%s: benchmark binary re-hash" % item,
                     "app run (%s): recorded=%s on disk=%s; the benchmark "
                     "binary is not on this run's execution path, so the "
                     "re-hash is informational and must not fail"
                     % (bench_reason or "no benchmark marker",
                        normalize_hash(bench_hash)[:16], got[:16]))
        elif is_nonempty_str(bench_path):
            skip("%s: benchmark binary re-hash" % item,
                 "%s not resolvable; recorded hash kept" % bench_path)
    else:
        prov_fail("%s: benchmark binary file hash is present" % item,
                  "no %s; the recorded benchmark hash is absent (R10)"
                  % "/".join(PROV_BENCH_HASH_KEYS[:3]))

    # --- template + input hash binding ------------------------------------
    templates = manifest.get("template_hashes")
    if not isinstance(templates, dict) or not templates:
        templates = manifest.get("templates")
    hash_values = set()
    if isinstance(templates, dict):
        hash_values = {normalize_hash(v) for v in templates.values()
                       if is_hex64(v)}
    if hash_values:
        ok("%s: template hashes are bound" % item,
           "%d hash value(s)" % len(hash_values))
    else:
        prov_fail("%s: template hashes are bound" % item,
                  "no sha256 value in template_hashes/templates: %r" % (templates,))
    cfg, _ = _first_key(manifest, EFFECTIVE_CONFIG_KEYS)
    cfg_sha = None
    if isinstance(cfg, dict):
        for key in ("comm_template_sha256", "template_sha256", "template_sha",
                    "comm_preamble_sha256"):
            if is_hex64(cfg.get(key)):
                cfg_sha = normalize_hash(cfg[key])
                break
    if cfg_sha is not None and isinstance(templates, dict):
        declared = set(hash_values)
        meta = manifest.get("template_meta")
        if isinstance(meta, dict):
            for entry in meta.values():
                if isinstance(entry, dict) and is_hex64(entry.get("sha256")):
                    declared.add(normalize_hash(entry["sha256"]))
        if cfg_sha in declared:
            ok("%s: effective config template hash is in template_hashes"
               % item, cfg_sha[:16] + "...")
        else:
            prov_fail("%s: effective config template hash is in template_hashes"
                      % item, "%s not among %d declared"
                      % (cfg_sha, len(declared)))

    input_hashes, input_path = _prov_lookup(manifest, prov, PROV_INPUT_HASH_KEYS)
    input_ok = False
    if isinstance(input_hashes, dict) and input_hashes \
            and any(is_hex64(v) for v in input_hashes.values()):
        input_ok = True
    elif isinstance(input_hashes, list) and input_hashes:
        input_ok = True
    if not input_ok:
        fixtures = manifest.get("fixture_hashes")
        if isinstance(fixtures, dict) and any(
                re.search(r"(window|win_|input|fixture|cf32|preamble)", str(k),
                          re.I) for k in fixtures):
            input_ok = True
    if input_ok:
        ok("%s: input/fixture hashes are bound" % item,
           input_path or "fixture_hashes")
    else:
        prov_fail("%s: input/fixture hashes are bound" % item,
                  "no %s and no window/input fixture hash"
                  % (input_path or "input_hashes"))

    # --- resolved effective parameters + hash -----------------------------
    eff_hash, eff_hash_path = _first_key(manifest, EFFECTIVE_HASH_KEYS)
    if is_hex64(eff_hash):
        ok("%s: resolved config hash is a sha256" % item,
           "%s=%s..." % (eff_hash_path, normalize_hash(eff_hash)[:16]))
    else:
        prov_fail("%s: resolved config hash is a sha256" % item,
                  "%s=%r" % (eff_hash_path, eff_hash))
    if isinstance(cfg, dict) and cfg:
        ok("%s: resolved parameters are recorded" % item, "%d keys" % len(cfg))
    else:
        prov_fail("%s: resolved parameters are recorded" % item,
                  "no effective/resolved config object")


# ---- not_run discipline (false vs null) -----------------------------------


def _gate_tested_evidence(summary, key):
    """Explicit evidence that a gate was exercised (used for ``false`` gates)."""
    gates = summary.get("gates") if isinstance(summary.get("gates"), dict) else {}
    value = gates.get(key)
    if isinstance(value, dict):
        tested = value.get("tested", value.get("executed"))
        evidence = value.get("evidence", value.get("reason", value.get("note")))
        if tested is True:
            return str(evidence) if is_nonempty_str(evidence) else "declared tested"
    for name in ("%s_evidence" % key, "%s_reason" % key, "evidence_%s" % key,
                 "%s_tested" % key, "%s_note" % key):
        for source in (gates, summary):
            if isinstance(source, dict) and is_nonempty_str(source.get(name)):
                return str(source[name])
    for name in GATE_EVIDENCE_CONTAINERS:
        for source in (summary, gates):
            if not isinstance(source, dict):
                continue
            block = source.get(name)
            if isinstance(block, dict):
                for candidate in (key, key.replace("_pass", "")):
                    if candidate in block and block[candidate]:
                        return str(block[candidate])
            elif isinstance(block, list) and any(
                    is_nonempty_str(x) and (norm_text(key) in norm_text(x)
                                            or norm_text(x) in norm_text(key))
                    for x in block):
                return "listed tested: %s" % key
    return None


def check_not_run_discipline(run_dir, summary, prefix):
    item = "%s not_run discipline (R10)" % prefix
    if not isinstance(summary, dict):
        return
    gates = summary.get("gates") if isinstance(summary.get("gates"), dict) else {}
    for key in GATE_KEYS_R10:
        if key not in gates:
            continue
        value = gates[key]
        evidence = _gate_tested_evidence(summary, key)
        named_untested = summary_key_mentioned(summary, key)
        if isinstance(value, bool) and value:
            if named_untested and evidence is None:
                fail("%s: a passing gate %s is not also named not_run"
                     % (item, key),
                     "gates.%s=true but not_run[] names it" % key)
            else:
                ok("%s: gate %s is a tested pass" % (item, key), "true")
        elif value is None:
            if named_untested or evidence is not None:
                ok("%s: null gate %s is named untested in not_run[]"
                   % (item, key), "named" if named_untested else "evidence note")
            else:
                fail("%s: null gate %s is named in not_run[]" % (item, key),
                     "gates.%s=null (untested) with no not_run[] entry "
                     "(null != false: null means not tested)" % key)
        elif value is False:
            if named_untested and evidence is None:
                fail("%s: false gate %s is not also named untested"
                     % (item, key),
                     "gates.%s=false is a TESTED-FAILED result, but not_run[] "
                     "names it (use null for untested)" % key)
            elif evidence is None:
                fail("%s: false gate %s carries tested evidence" % (item, key),
                     "gates.%s=false means tested-and-failed; no gate evidence "
                     "is recorded (false != null/untested)" % key)
            else:
                ok("%s: false gate %s carries tested evidence" % (item, key),
                   str(evidence)[:80])
        else:
            fail("%s: gate %s is a boolean or null" % (item, key),
                 "got %r" % (value,))

    # A static not_run template must not name a present required artifact.
    for fname in REQUIRED_ARTIFACTS:
        if summary_key_mentioned(summary, fname) \
                and os.path.isfile(os.path.join(run_dir, fname)):
            fail("%s: not_run does not name the present artifact %s"
                 % (item, fname),
                 "not_run[] claims %s is not produced but the file exists "
                 "(a static not_run template is not execution evidence)" % fname)


# ---- superseded runs + production claims ----------------------------------


def find_superseded_index(results_dir):
    """A results-level index mapping run labels to superseded reasons."""
    mapping = {}
    for name in SUPERSEDED_INDEX_FILES:
        path = os.path.join(results_dir, name)
        if not os.path.isfile(path):
            continue
        data, _err = load_json(path)
        if not isinstance(data, dict):
            continue
        block = data.get("superseded")
        if isinstance(block, dict):
            for label, reason in block.items():
                mapping[str(label)] = reason if is_nonempty_str(reason) \
                    else "superseded"
        elif isinstance(block, list):
            for entry in block:
                if not isinstance(entry, dict):
                    continue
                labels = entry.get("runs", entry.get("dirs", entry.get("labels")))
                reason = entry.get("reason", "superseded")
                if isinstance(labels, list):
                    for label in labels:
                        mapping[str(label)] = reason
                elif is_nonempty_str(labels):
                    mapping[str(labels)] = reason
        runs = data.get("runs")
        if isinstance(runs, dict):
            for label, entry in runs.items():
                if entry is True:
                    mapping[str(label)] = "superseded"
                elif isinstance(entry, dict) and entry.get("superseded"):
                    mapping[str(label)] = entry.get("reason", "superseded")
    return mapping


def superseded_status(run_dir, results_dir, manifest, summary, index):
    label = label_for(run_dir, results_dir)
    for source_name, block in (("run_manifest.json", manifest),
                               ("summary.json", summary)):
        if not isinstance(block, dict):
            continue
        marker = block.get("superseded")
        if marker is True:
            return {"by": "%s.superseded" % source_name,
                    "reason": block.get("superseded_reason")}
        if isinstance(marker, dict) and marker.get("superseded", True):
            return {"by": "%s.superseded" % source_name,
                    "reason": marker.get("reason")}
        for key in ("run_status", "status", "evidence_status", "run_state"):
            value = block.get(key)
            if is_nonempty_str(value) and norm_text(value) in (
                    "superseded", "stale", "obsolete", "retired"):
                return {"by": "%s.%s" % (source_name, key),
                        "reason": block.get("superseded_reason", value)}
    if label in (index or {}):
        return {"by": "index[%s]" % label, "reason": index[label]}
    return None


def check_superseded(run_dir, results_dir, manifest, summary, index, prefix):
    item = "%s superseded handling (R10)" % prefix
    status = superseded_status(run_dir, results_dir, manifest, summary, index)
    if status is None:
        ok("%s: run is not superseded (must-pass)" % item, "must-pass")
        return None
    reason = status.get("reason")
    if is_nonempty_str(reason):
        ok("%s: superseded run is explicitly marked with a reason" % item,
           "%s: %s" % (status["by"], str(reason)[:80]))
    else:
        fail("%s: a superseded run records a reason" % item,
             "marked at %s with no reason" % status["by"])
    return status


def _production_claim_sources(manifest, summary):
    sources = []
    for name, block in (("run_manifest.json", manifest), ("summary.json", summary)):
        if not isinstance(block, dict):
            continue
        candidates = [("", block)]
        for cont in ("provenance", "evidence", "classification"):
            if isinstance(block.get(cont), dict):
                candidates.append((cont + ".", block[cont]))
        for prefix, src in candidates:
            for key in PRODUCTION_CLAIM_KEYS:
                if src.get(key) is True:
                    sources.append("%s.%s%s" % (name, prefix, key))
            for key in ("profile", "evidence_class", "classification",
                        "evidence_type"):
                value = src.get(key)
                if is_nonempty_str(value) and norm_text(value) in (
                        "production", "production profile", "official",
                        "official evidence", "acceptance", "acceptance evidence"):
                    sources.append("%s.%s=%s" % (name, key, value))
    return sources


def _input_is_synthetic(manifest, prov):
    value, path = _prov_lookup(manifest, prov,
                               ("input_source", "input_mode", "input_kind",
                                "data_source"))
    if is_nonempty_str(value) and norm_text(value) in SYNTHETIC_INPUT_MARKERS:
        return True, "%s=%s" % (path, value)
    for key in ("synthetic_input", "is_synthetic", "synthetic"):
        for src in (prov, manifest):
            if isinstance(src, dict) and src.get(key) is True:
                return True, key
    return False, None


def _r07_templates_ok(manifest, prov):
    found = False
    for src in (prov, manifest):
        if not isinstance(src, dict):
            continue
        templates = src.get("templates")
        if isinstance(templates, dict) and "r07_pass" in templates:
            found = True
            if templates.get("r07_pass") is not True:
                return False, "templates.r07_pass=%r" % templates.get("r07_pass")
        for key in ("comm_template_mode", "template_mode"):
            mode = src.get(key)
            if is_nonempty_str(mode):
                found = True
                if norm_text(mode) not in R07_TEMPLATE_MODES:
                    return False, "%s=%r" % (key, mode)
    meta = manifest.get("template_meta")
    if isinstance(meta, dict):
        for tid, entry in meta.items():
            if isinstance(entry, dict) and is_nonempty_str(entry.get("mode")):
                found = True
                if norm_text(entry["mode"]) not in R07_TEMPLATE_MODES:
                    return False, "template_meta[%s].mode=%r" % (tid, entry["mode"])
    if not found:
        return None, "no finite-SYNC template mode / r07_pass declaration"
    return True, "finite-length SYNC template (R07)"


def _has_real_consumer(manifest, summary):
    for src in (summary, manifest):
        if not isinstance(src, dict):
            continue
        terminal = src.get("terminal")
        if isinstance(terminal, dict):
            for key in ("downstream_ok", "fcs_pass", "psdu_match",
                        "completions"):
                if is_num(terminal.get(key)) and float(terminal[key]) > 0:
                    return True, "terminal.%s>0" % key
        effect = src.get("effect")
        if isinstance(effect, dict) and is_num(effect.get("ok")) \
                and float(effect["ok"]) > 0:
            return True, "effect.ok>0"
        consumer = src.get("consumer")
        if is_nonempty_str(consumer) and norm_text(consumer) not in (
                "none", "no", "off", "null", "sic only"):
            return True, "consumer=%s" % consumer
    config, _ = _first_key(manifest, EFFECTIVE_CONFIG_KEYS)
    if isinstance(config, dict):
        consumer = config.get("consumer")
        if is_nonempty_str(consumer) and norm_text(consumer) not in (
                "none", "no", "off", "null", "sic only"):
            return True, "config.consumer=%s" % consumer
    return False, None


def check_production_claims(manifest, summary, prov, superseded, prefix):
    item = "%s production-profile evidence (R10)" % prefix
    claims = _production_claim_sources(manifest, summary)
    if not claims:
        ok("%s: run does not claim production-profile evidence" % item,
           "no production claim")
        return
    claim_text = ",".join(claims)
    if superseded:
        fail("%s: a superseded run does not claim production evidence" % item,
             "superseded but claims %s" % claim_text)
        return
    ok("%s: run claims production-profile evidence" % item, claim_text)

    synthetic, why = _input_is_synthetic(manifest, prov)
    if synthetic:
        fail("%s: production evidence uses a real (non-synthetic) input" % item,
             "claims %s but %s (synthetic input cannot be production evidence)"
             % (claim_text, why))
    else:
        ok("%s: production evidence uses a real (non-synthetic) input" % item,
           "no synthetic-input marker")

    r07, why = _r07_templates_ok(manifest, prov)
    if r07 is False:
        fail("%s: production evidence uses post-R07 finite-SYNC templates"
             % item, "claims %s but %s" % (claim_text, why))
    elif r07 is None:
        fail("%s: production evidence uses post-R07 finite-SYNC templates"
             % item,
             "claims %s but no comm_template_mode/r07_pass declaration is "
             "present (pre-R07 templates cannot be production evidence)"
             % claim_text)
    else:
        ok("%s: production evidence uses post-R07 finite-SYNC templates"
           % item, why)

    real, why = _has_real_consumer(manifest, summary)
    if real:
        ok("%s: production evidence runs a real downstream consumer" % item, why)
    else:
        fail("%s: production evidence runs a real downstream consumer" % item,
             "claims %s but no downstream/fcs/psdu/consumer evidence"
             % claim_text)


# ---- MATLAB status --------------------------------------------------------


def _matlab_executed(status):
    if not is_nonempty_str(status):
        return None
    raw = str(status)
    if "未实跑" in raw:
        return False
    if "已实跑" in raw:
        return True
    text = norm_text(raw)
    if text in MATLAB_NOTRUN_MARKERS:
        return False
    if text in MATLAB_RUN_MARKERS:
        return True
    return None


def _matlab_info(manifest, summary):
    info = {"status": None, "command": None, "log_path": None, "source": None}
    for label, block in (("run_manifest.json", manifest),
                         ("summary.json", summary)):
        if not isinstance(block, dict):
            continue
        for key in ("matlab", "matlab_status_check", "matlab_check"):
            value = block.get(key)
            if isinstance(value, dict):
                info["status"] = value.get("status", value.get("state"))
                if info["status"] is None and isinstance(value.get("ran"), bool):
                    info["status"] = "run" if value["ran"] else "not_run"
                if info["status"] is None \
                        and isinstance(value.get("executed"), bool):
                    info["status"] = "run" if value["executed"] else "not_run"
                info["command"] = value.get("command", value.get("cmd"))
                info["log_path"] = value.get(
                    "log_path", value.get("log", value.get("logfile")))
                info["source"] = "%s.%s" % (label, key)
                return info
        for skey in ("matlab_status", "matlab_state"):
            if info["status"] is None and block.get(skey) is not None:
                info["status"] = block[skey]
                info["source"] = "%s.%s" % (label, skey)
        for bkey in ("matlab_ran", "matlab_executed"):
            if info["status"] is None and isinstance(block.get(bkey), bool):
                info["status"] = "run" if block[bkey] else "not_run"
                info["source"] = "%s.%s" % (label, bkey)
        for ckey in ("matlab_command", "matlab_cmd"):
            if info["command"] is None and is_nonempty_str(block.get(ckey)):
                info["command"] = block[ckey]
        for lkey in ("matlab_log", "matlab_log_path", "matlab_logfile"):
            if info["log_path"] is None and is_nonempty_str(block.get(lkey)):
                info["log_path"] = block[lkey]
        if info["status"] is not None and info["command"] is not None \
                and info["log_path"] is not None:
            break
    return info


def _matlab_parity_claims(manifest, summary):
    claims = []
    for label, block in (("run_manifest.json", manifest),
                         ("summary.json", summary)):
        if not isinstance(block, dict):
            continue
        candidates = [("", block)]
        for cont in ("effect", "gates", "evidence", "matlab"):
            if isinstance(block.get(cont), dict):
                candidates.append((cont + ".", block[cont]))
        for prefix, src in candidates:
            for key in MATLAB_PARITY_KEYS:
                if src.get(key) is True:
                    claims.append("%s.%s%s" % (label, prefix, key))
            for key in ("parity", "parity_source", "reference"):
                value = src.get(key)
                if is_nonempty_str(value) and "matlab" in norm_text(value).split():
                    claims.append("%s.%s%s=%s" % (label, prefix, key, value))
    return claims


def check_matlab_status(run_dir, manifest, summary, prefix):
    item = "%s MATLAB status (R10)" % prefix
    info = _matlab_info(manifest, summary)
    if info["status"] is None:
        fail("%s: explicit MATLAB 已实跑/未实跑 flag is present" % item,
             "no matlab{status,...} block / matlab_status; MATLAB parity "
             "cannot be assumed (Python-only parity is not MATLAB acceptance)")
        executed = None
    else:
        ok("%s: explicit MATLAB status flag is present" % item,
           "%s=%r" % (info["source"], info["status"]))
        executed = _matlab_executed(info["status"])
    if executed is None and info["status"] is not None:
        fail("%s: MATLAB status is a recognised 已实跑/未实跑 flag" % item,
             "got %r" % (info["status"],))
    if is_nonempty_str(info["command"]):
        ok("%s: MATLAB command is recorded" % item, str(info["command"])[:90])
    else:
        fail("%s: MATLAB command is recorded" % item,
             "no command (needed to reproduce the 已实跑/未实跑 claim)")
    if is_nonempty_str(info["log_path"]):
        ok("%s: MATLAB log path is recorded" % item, str(info["log_path"]))
    else:
        fail("%s: MATLAB log path is recorded" % item, "no log_path")
    if executed is True:
        resolved = _resolve_prov_path(info["log_path"], (run_dir,))
        if resolved is not None:
            ok("%s: an executed MATLAB run has its log on disk" % item, resolved)
        else:
            fail("%s: an executed MATLAB run has its log on disk" % item,
                 "已实跑 but log %r is absent" % (info["log_path"],))
    elif executed is False:
        ok("%s: MATLAB is declared 未实跑 (not run)" % item, "not_run")
    claims = _matlab_parity_claims(manifest, summary)
    if claims:
        if executed is not True:
            fail("%s: MATLAB parity is not claimed without a real run" % item,
                 "claims %s but MATLAB status=%r (Python-only parity is not "
                 "MATLAB acceptance)" % (",".join(claims), info["status"]))
        else:
            ok("%s: MATLAB parity is claimed and a run is recorded" % item,
               ",".join(claims))


# ---- perf rows bind to an effect-verified case ----------------------------


def _csv_bool(text):
    if text is None:
        return None
    value = norm_text(text)
    if value == "":
        return None
    if value in ("true", "1", "yes", "y", "pass", "passed", "ok", "verified"):
        return True
    if value in ("false", "0", "no", "n", "fail", "failed", "unverified"):
        return False
    return None


def _num_or_none(text):
    try:
        return float(str(text).strip())
    except (TypeError, ValueError):
        return None


def _find_perf_tables(results_dir):
    found = []
    for dirpath, _dirnames, filenames in os.walk(results_dir):
        for name in filenames:
            if name.endswith(".csv") and "perf_table" in name:
                found.append(os.path.join(dirpath, name))
    return sorted(set(found))


def check_perf_tables(results_dir):
    item = "results perf table (R10)"
    tables = _find_perf_tables(results_dir)
    if not tables:
        skip("%s: perf rows bind to an effect-verified case" % item,
             "no *perf_table.csv under %s" % results_dir)
        return
    for path in tables:
        where = "%s [%s]" % (item, os.path.relpath(path, results_dir))
        try:
            with open(path, "r", encoding="utf-8", newline="") as handle:
                rows = list(csv.DictReader(handle))
        except OSError as exc:
            fail("%s: readable" % where, str(exc))
            continue
        if not rows:
            fail("%s: has at least one perf row" % where, "empty")
            continue
        ok("%s: parsed" % where, "%d perf row(s)" % len(rows))
        production_hashes = set()
        for index, row in enumerate(rows, 2):
            label = "%s line %d" % (where, index)
            config_hash = normalize_hash(row.get("config_hash", ""))
            if not is_hex64(config_hash):
                fail("%s: config_hash is a sha256" % label,
                     "got %r" % (row.get("config_hash"),))
            template_hash = None
            for key in ("comm_preamble_sha256", "template_sha256",
                        "comm_template_sha256"):
                if is_hex64(row.get(key)):
                    template_hash = normalize_hash(row[key])
                    break
            if template_hash is None:
                fail("%s: template hash is a sha256" % label,
                     "no comm_preamble_sha256/template_sha256 column value")
            verified = _csv_bool(row.get("effect_verified"))
            effect_ok = _num_or_none(row.get("effect_ok"))
            effect_fail = _num_or_none(row.get("effect_fail"))
            exit_class = (row.get("exit_class") or "").strip()
            production = _csv_bool(row.get("production_profile"))
            if production is None:
                production = _csv_bool(row.get("official_evidence"))
            if verified is True:
                if (effect_ok is not None and effect_ok <= 0) \
                        or (effect_fail is not None and effect_fail > 0):
                    fail("%s: effect_verified=true is consistent with "
                         "effect_ok/fail" % label,
                         "effect_ok=%r effect_fail=%r" % (effect_ok, effect_fail))
                else:
                    ok("%s: perf row binds to an effect-verified case" % label,
                       "effect_verified=true")
            elif production is True:
                fail("%s: production perf row binds to an effect-verified case"
                     % label,
                     "production_profile=true but effect_verified=%r exit_class=%r"
                     % (verified, exit_class))
            else:
                tagged = any(tag in exit_class.lower() for tag in (
                    "unverified", "approx", "synthetic", "micro", "failed",
                    "effect_failed", "native"))
                if tagged:
                    ok("%s: non-evidence perf row is explicitly labelled" % label,
                       exit_class or "labelled")
                else:
                    fail("%s: an unverified perf row is explicitly labelled"
                         % label,
                         "effect_verified=%r with no unverified/approx/synthetic "
                         "label (exit_class=%r)" % (verified, exit_class))
            if production is True and verified is True \
                    and config_hash and template_hash:
                production_hashes.add((config_hash, template_hash))
        if production_hashes:
            if len(production_hashes) == 1:
                ok("%s: production perf rows share one case/config/template"
                   % where, str(next(iter(production_hashes))))
            else:
                fail("%s: production perf rows share one case/config/template"
                     % where, "found %d distinct identities: %s"
                     % (len(production_hashes), sorted(production_hashes)))


def _find_benchmark_summaries(results_dir):
    found = []
    for dirpath, _dirnames, filenames in os.walk(results_dir):
        for name in filenames:
            if not name.endswith("_summary.json"):
                continue
            path = os.path.join(dirpath, name)
            data, _err = load_json(path)
            if isinstance(data, dict) and is_nonempty_str(data.get("schema")) \
                    and data["schema"].startswith("sic_perf"):
                found.append(path)
    return sorted(set(found))


def check_benchmark_summaries(results_dir, index):
    item = "results benchmark summary (R10)"
    summaries = _find_benchmark_summaries(results_dir)
    if not summaries:
        skip("%s: benchmark summaries carry bound provenance" % item,
             "no sic_perf*_summary.json under %s" % results_dir)
        return
    for path in summaries:
        where = "%s [%s]" % (item, os.path.relpath(path, results_dir))
        data, err = load_json(path)
        if not isinstance(data, dict):
            fail("%s: well-formed JSON" % where, err or "not an object")
            continue
        ok("%s: well-formed JSON" % where, data.get("schema"))
        prov = data.get("provenance") \
            if isinstance(data.get("provenance"), dict) else {}
        check_provenance(data, os.path.dirname(path), results_dir, where,
                         superseded=False)
        check_production_claims(data, None, prov, False, where)
        windows = data.get("input_windows")
        if isinstance(windows, list) and windows:
            shas = [w.get("sha256") for w in windows if isinstance(w, dict)]
            if shas and all(is_hex64(x) for x in shas):
                ok("%s: input windows are hash-bound" % where,
                   "%d window(s)" % len(shas))
            else:
                fail("%s: input windows are hash-bound" % where,
                     "not every input_windows[].sha256 is a sha256")
        else:
            fail("%s: input windows are hash-bound" % where,
                 "no input_windows[] list")


# ---------------------------------------------------------------------------
# S1: strict 8-SYNC estimation, frozen model, full-preamble cancellation
# ---------------------------------------------------------------------------
# The S1 order (docs/sic/OpenCode修改指示_S1_八符号估计与全前导消除_2026-10-17.md)
# supersedes the S0 fit/validation/support coupling.  These checks fail closed:
#
# * every SIC-on run must declare its model mode; the only accepted formal mode
#   is ``estimate8_frozen``;
# * the model parameters are estimated from the first 8 SYNC only
#   (``observed_end_sample``), frozen, and applied unchanged to the rest of the
#   nominal 64-SYNC preamble;
# * the cancellation support is the configured-nominal prediction
#   ``origin + sync_reps*sps + post_tail``, never ``confirmed*sps``;
# * training [0,8) SYNC, independent evaluation [8,64) SYNC and the last-4-SYNC
#   tail are explicit, disjoint from training, and ALL OF THEM are cancelled;
# * the actual-output suppression is recomputed from archived arrays over the
#   full truth overlap, the independent-evaluation region and the tail region
#   (``i - (y - y_clean)``), and the three reported medians are required;
# * the formal acceptance windows use acceptance seeds whose generated
#   (seed, seed_offset, noise_root) tuples are disjoint from the calibration
#   windows;
# * effect/core/GR/CIR/full-demod/native run one effective config and one model
#   mode/hash set, and ``index.json`` points at the actual latest run.
#
# Canonical per-window field names (aliases accepted):
#   window.sic_model{...}: mode, model_id, fingerprint, origin, cfo_hz,
#     n_paths, tau[], gain[][], samples_per_symbol, nominal_sync_reps,
#     training_lo/hi, observed_end_sample, estimation_ready_sample,
#     estimation_ranges{training|cfo|cir:[lo,hi]}, guard{pre,post},
#     strict_8sync, support_lo/hi, support_source, support_pre_tail,
#     support_post_tail, support_coverage, sub_lo/hi, phase_reference,
#     eval_lo/hi, tail_lo/hi, fit_reps, fit_residual, cond2, status,
#     apply{model_id, fingerprint, params{...}, refit_count, param_deltas}
#   window.generator{seed_set, seed, seed_offset, noise_root}
#   window.recon.eval_actual_suppression_db,
#   window.recon.tail_actual_suppression_db (plus actual_suppression_db)
#   manifest.sic_model_mode, manifest.calibration_windows[],
#   manifest.effective_config{sic_model_mode, evaluation_reps,
#     online_validation_reps, fit_reps},
#   manifest.implementation_configs{cir, full_demod, ...}
#   summary.effect.median_eval_actual_suppression_db,
#     median_tail_actual_suppression_db,
#     n_eval_windows, n_eval_na_windows, n_tail_windows, n_tail_na_windows
#   results-root index.json.latest_run

S1_MODEL_MODE = "estimate8_frozen"
# Comparisons use norm_text (lowercase, non-alphanumerics collapsed).
S1_MODEL_MODE_NORM = "estimate8 frozen"
S1_MODEL_MODES = {"estimate8 frozen"}
S1_LEGACY_MODES = {"legacy cover", "legacy", "s0 cover", "cover validation",
                   "legacy validated", "validated cover", "s0 8 of 52"}
S1_MODEL_CONTAINERS = ("sic_model", "frozen_model", "model", "sic_model_block")
S1_STATE_CONTAINERS = ("sic_model", "frozen_model", "model")
S1_SUPPORT_SOURCES = {"configured nominal prediction",
                      "configured nominal", "configured nominal predict",
                      "nominal prediction"}
S1_STRICT_FIT_REPS = 8
S1_MIN_EVAL_REPS = 8
S1_TAIL_REPS = 4
S1_SUPPORT_COVERAGE = 0.99
S1_CANCEL_FRACTION = 0.99
S1_ACTUAL_TOL_DB = SUPPRESSION_TOL_DB
S1_PARAM_TOL = 1e-9

S1_EVAL_ACTUAL_KEYS = (
    "eval_actual_suppression_db", "actual_suppression_db_eval",
    "eval_actual_sup_db", "independent_eval_actual_suppression_db")
S1_TAIL_ACTUAL_KEYS = (
    "tail_actual_suppression_db", "actual_suppression_db_tail",
    "tail_actual_sup_db", "tail4_actual_suppression_db")
S1_EVAL_MEDIAN_KEYS = (
    "median_eval_actual_suppression_db", "eval_median_actual_suppression_db",
    "median_actual_suppression_db_eval", "eval_median_suppression_db")
S1_TAIL_MEDIAN_KEYS = (
    "median_tail_actual_suppression_db", "tail_median_actual_suppression_db",
    "median_actual_suppression_db_tail", "tail_median_suppression_db")
S1_EVAL_COUNT_KEYS = ("n_eval_windows", "eval_windows", "eval_window_count")
S1_TAIL_COUNT_KEYS = ("n_tail_windows", "tail_windows", "tail_window_count")
S1_EVAL_NA_KEYS = ("n_eval_na_windows", "eval_na_windows",
                   "n_eval_not_applicable")
S1_TAIL_NA_KEYS = ("n_tail_na_windows", "tail_na_windows",
                   "n_tail_not_applicable")

S1_LAYER_ALIASES = {
    "effect": ("functional", "effect", "app", "python", "effect_path"),
    "core": ("core", "cxx", "sic_core", "math_core"),
    "gr": ("gr", "gnuradio", "flowgraph", "pipeline"),
    "cir": ("cir", "sic_cir", "radar_cir", "cir_path"),
    "full_demod": ("full_demod", "demod", "full_demodulation", "demod_path"),
    "native": ("native", "cg400", "native_benchmark"),
}
S1_INDEX_FILES = ("index.json", "results_index.json")
S1_SEED_CONTAINERS = ("generator", "input_gen", "generation", "seed_info")

# --- S1 clipped regions + reconstruction archive policy --------------------
# The independent-eval and tail regions are DEFINED as the nominal repetition
# ranges intersected with the applied subtraction (``sub_lo``/``sub_hi``) and
# the generator truth overlap.  A run whose public processing ROI cannot cancel
# the whole nominal region must declare the clipped region together with its
# covered fraction (``eval_covered_frac``/``tail_covered_frac``,
# ``cancellation_clipped``); a run that ships only a subset of the per-window
# reconstruction arrays must declare an archive policy that names the archived
# windows and states the non-archived limitation.
S1_MIN_ARCHIVED = 4
S1_ARCHIVE_POLICY_KEYS = ("archive_policy", "recon_archive_policy",
                          "archived_recon", "sic_archive_policy")
S1_ARCHIVED_ID_KEYS = ("archived_ids", "archived_window_ids",
                       "recon_archived_ids", "archived")
S1_ARCHIVED_COUNT_KEYS = ("per_cell", "per_cell_archived", "archived_per_cell",
                          "n_archived_per_cell")
S1_ARCHIVED_TOTAL_KEYS = ("total_windows", "n_windows", "windows")
S1_CLIP_FLAG_KEYS = ("cancellation_clipped", "sub_clipped", "region_clipped",
                     "subtraction_clipped")
S1_EVAL_COVERED_KEYS = ("eval_covered_frac", "evaluation_covered_frac",
                        "eval_covered_fraction", "eval_covered")
S1_TAIL_COVERED_KEYS = ("tail_covered_frac", "tail_eval_covered_frac",
                        "tail_covered_fraction", "tail_covered")


def _s1_online_declared_zero(window):
    """Explicit online_validation_reps == 0 (the S1 default)."""
    for value in (window.get("validation_reps"), window.get("online_validation_reps")):
        if is_int(value) and value == 0:
            return True
    for container in ("regions", "region", "config", "sic_config", "sic"):
        block = window.get(container)
        if isinstance(block, dict):
            for key in ("online_validation_reps", "validation_reps",
                        "online_validation_len", "validation_len"):
                value = block.get(key)
                if is_int(value) and value == 0:
                    return True
    return False


def _s1_zero_validation_ok(config):
    """A zero online-validation config is legal under the S1 offline semantics."""
    if not isinstance(config, dict):
        return False
    mode, _ = _nested_lookup(config, ("sic_model_mode", "model_mode",
                                      "method_mode", "mode"))
    eval_reps, _ = _nested_lookup(config, ("evaluation_reps", "eval_reps",
                                           "independent_eval_reps"))
    return is_nonempty_str(mode) and norm_text(mode) in S1_MODEL_MODES \
        and is_int(eval_reps) and int(eval_reps) >= S1_MIN_EVAL_REPS


def s1_model_of(window):
    """The per-window frozen-model block (S1 canonical + aliases)."""
    for name in S1_MODEL_CONTAINERS:
        block = window.get(name)
        if isinstance(block, dict):
            return block, name
    state = window.get("sic_state")
    if isinstance(state, dict):
        for name in S1_STATE_CONTAINERS:
            block = state.get(name)
            if isinstance(block, dict):
                return block, "sic_state.%s" % name
    return None, None


def _s1_declared_mode(obj):
    if not isinstance(obj, dict):
        return None, None
    value, key = _first_key(obj, ("sic_model_mode", "model_mode", "method_mode",
                                  "estimate_mode", "model_method"))
    if is_nonempty_str(value):
        return norm_text(value), key
    for sub in ("s1", "sic", "effective_config", "config", "provenance",
                "method"):
        block = obj.get(sub)
        if isinstance(block, dict):
            value, key = _first_key(block, ("sic_model_mode", "model_mode",
                                            "method_mode", "estimate_mode"))
            if is_nonempty_str(value):
                return norm_text(value), "%s.%s" % (sub, key)
    return None, None


def s1_mode_of(manifest, summary):
    for source, name in ((manifest, "run_manifest.json"),
                         (summary, "summary.json")):
        value, path = _s1_declared_mode(source)
        if value is not None:
            return value, "%s.%s" % (name, path)
    return None, None


def _s1_legacy_declared(manifest, summary):
    for source, name in ((manifest, "run_manifest.json"),
                         (summary, "summary.json")):
        if not isinstance(source, dict):
            continue
        for key in ("legacy_model_mode", "legacy_method", "legacy"):
            if source.get(key) is True:
                return "%s.%s=true" % (name, key)
        value, path = _s1_declared_mode(source)
        if value is not None and value in S1_LEGACY_MODES:
            return "%s.%s=%s" % (name, path, value)
    return None


def _s1_gain_pair(value):
    if isinstance(value, (list, tuple)) and len(value) == 2 \
            and all(is_num(v) for v in value):
        return float(value[0]), float(value[1])
    if isinstance(value, dict):
        re = value.get("real", value.get("re"))
        im = value.get("imag", value.get("im"))
        if is_num(re) and is_num(im):
            return float(re), float(im)
    return None


def _s1_num_equal(a, b, tol=S1_PARAM_TOL):
    if not (is_num(a) and is_num(b)):
        return False
    fa, fb = float(a), float(b)
    return abs(fa - fb) <= tol + tol * max(abs(fa), abs(fb))


def _s1_vec_equal(a, b, pair=False):
    if not (isinstance(a, (list, tuple)) and isinstance(b, (list, tuple))) \
            or len(a) != len(b):
        return False
    for x, y in zip(a, b):
        if pair:
            px, py = _s1_gain_pair(x), _s1_gain_pair(y)
            if px is None or py is None \
                    or not _s1_num_equal(px[0], py[0]) \
                    or not _s1_num_equal(px[1], py[1]):
                return False
        elif not _s1_num_equal(x, y):
            return False
    return True


def _s1_estimation_ranges(model):
    raw, path = _first_key(model, ("estimation_ranges", "estimation_inputs",
                                   "estimation_input_ranges",
                                   "estimation_visible_ranges"))
    ranges = {}
    if isinstance(raw, dict):
        for name, value in raw.items():
            pair = _int_pair(value)
            if pair is not None:
                ranges[norm_text(name)] = pair
            else:
                lo = value.get("lo") if isinstance(value, dict) else None
                hi = value.get("hi") if isinstance(value, dict) else None
                if is_int(lo) and is_int(hi):
                    ranges[norm_text(name)] = (int(lo), int(hi))
    elif isinstance(raw, list):
        for entry in raw:
            if not isinstance(entry, dict):
                continue
            name = entry.get("name") or entry.get("range_name")
            pair = _int_pair([entry.get("lo"), entry.get("hi")])
            if pair is None:
                pair = _int_pair(entry.get("range"))
            if name and pair is not None:
                ranges[norm_text(name)] = pair
    return ranges, path


def _s1_guard(model):
    raw, path = _first_key(model, ("guard", "guard_samples",
                                   "observation_guard"))
    if isinstance(raw, dict):
        pre = raw.get("pre", raw.get("pre_samples", 0))
        post = raw.get("post", raw.get("post_samples", 0))
        if is_int(pre) and is_int(post) and pre >= 0 and post >= 0:
            return (int(pre), int(post)), path
        return None, path
    if is_int(raw) and raw >= 0:
        return (0, int(raw)), path
    return None, path


def _s1_argument(model, keys):
    return _first_key(model, keys)


def _s1_layer_entry(layers, layer):
    if not isinstance(layers, dict):
        return None, None
    for alias in S1_LAYER_ALIASES.get(layer, (layer,)):
        if alias in layers and layers[alias] is not None:
            return layers[alias], alias
    return None, None


def _s1_actual_metric(i, delta, lo, hi):
    """-10log10(||i-(y-y_clean)||^2 / ||i||^2) over [lo,hi).

    Returns (declared_value, n_samples, interference_energy).  A value of
    ``None`` means N/A (zero interference energy).
    """
    ref = i[lo:hi]
    residual = ref - delta[lo:hi]
    den = float(np.vdot(ref, ref).real)
    num = float(np.vdot(residual, residual).real)
    if den <= 0.0:
        return None, hi - lo, den
    if num <= 0.0:
        return 60.0, hi - lo, den
    return -10.0 * math.log10(num / den), hi - lo, den


def _s1_changed_fraction(y, y_clean, lo, hi):
    span = hi - lo
    if span <= 0:
        return 0.0
    changed = (y[lo:hi] != y_clean[lo:hi])
    return float(np.count_nonzero(changed)) / float(span)


def _s1_run_windows(windows):
    return [w for w in windows
            if isinstance(w.get("recon"), dict)
            and isinstance(w.get("sic_state"), dict)
            and w["sic_state"].get("committed") is True
            and not w.get("diagnostic_only")]


def _s1_parse_model(model, mpath, where):
    """Read + validate the frozen-model evidence.  Returns a geometry dict."""
    g = {}
    simple = (
        ("model_id", ("model_id", "id")),
        ("fingerprint", ("fingerprint", "model_fingerprint", "model_sha256")),
        ("origin", ("origin", "origin_integer", "detected_start")),
        ("cfo_hz", ("cfo_hz", "cfo")),
        ("n_paths", ("n_paths", "path_count")),
        ("support_lo", ("support_lo", "support_start")),
        ("support_hi", ("support_hi", "support_stop")),
        ("support_source", ("support_source", "support_origin")),
        ("support_pre_tail", ("support_pre_tail", "pre_tail",
                              "support_lead_samples")),
        ("support_post_tail", ("support_post_tail", "post_tail",
                               "support_tail_samples")),
        ("support_coverage", ("support_coverage", "nominal_support_coverage",
                              "predicted_support_coverage")),
        ("sub_lo", ("sub_lo", "subtraction_lo", "mask_lo")),
        ("sub_hi", ("sub_hi", "subtraction_hi", "mask_hi")),
        ("phase_reference", ("phase_reference", "phase_reference_sample",
                             "n_ref")),
        ("training_lo", ("training_lo", "training_start")),
        ("training_hi", ("training_hi", "training_end")),
        ("observed_end_sample", ("observed_end_sample", "observed_end",
                                 "prefix_end_sample")),
        ("estimation_ready_sample", ("estimation_ready_sample",
                                     "estimation_ready",
                                     "params_ready_sample")),
        ("eval_lo", ("eval_lo", "evaluation_lo", "independent_eval_lo")),
        ("eval_hi", ("eval_hi", "evaluation_hi", "independent_eval_hi")),
        ("tail_lo", ("tail_lo", "tail_eval_lo", "last4_sync_lo")),
        ("tail_hi", ("tail_hi", "tail_eval_hi", "last4_sync_hi")),
        ("fit_reps", ("fit_reps", "training_reps")),
        ("samples_per_symbol", ("samples_per_symbol", "sps")),
        ("nominal_sync_reps", ("nominal_sync_reps", "sync_reps",
                               "sensing_sync_reps")),
        ("sample_rate", ("sample_rate",)),
        ("strict_8sync", ("strict_8sync", "strict_8_sync", "strict")),
        ("fit_residual", ("fit_residual", "residual")),
        ("cond2", ("cond2", "condition_number")),
        ("status", ("status", "model_status", "quality")),
    )
    for name, keys in simple:
        value, key = _first_key(model, keys)
        g[name] = value
        g[name + "_path"] = ("%s.%s" % (mpath, key)) if key else mpath
    tau, tau_key = _first_key(model, ("tau", "taus", "delays",
                                      "delays_samples", "fractional_delay"))
    gain, gain_key = _first_key(model, ("gain", "gains", "complex_gain"))
    g["tau"], g["tau_path"] = tau, tau_key
    g["gain"], g["gain_path"] = gain, gain_key
    g["ranges"], g["ranges_path"] = _s1_estimation_ranges(model)
    g["guard"], g["guard_path"] = _s1_guard(model)
    g["apply"], g["apply_path"] = _s1_argument(
        model, ("apply", "apply_model", "applied", "apply_state"))
    return g


def _s1_check_prefix(window, g, where, mode):
    """S1-1: strict-prefix evidence (ranges must not read past observed_end)."""
    item = "%s S1-1 strict-prefix evidence" % where
    problems = []
    for name in ("training_lo", "training_hi", "observed_end_sample",
                 "estimation_ready_sample", "origin", "fit_reps",
                 "samples_per_symbol", "nominal_sync_reps"):
        if not is_int(g.get(name)):
            problems.append("%s is an integer (%s=%r via %s)"
                            % (name, name, g.get(name), g.get(name + "_path")))
    for name in ("strict_8sync",):
        if g.get(name) is not None and not isinstance(g.get(name), bool):
            problems.append("strict_8sync is a boolean (got %r)"
                            % (g.get(name),))
    if problems:
        for text in problems:
            fail(item, text)
        return False
    origin = int(g["origin"])
    sps = int(g["samples_per_symbol"])
    sync_reps = int(g["nominal_sync_reps"])
    fit_reps = int(g["fit_reps"])
    strict = g["strict_8sync"] is True
    if g["strict_8sync"] is None:
        strict = (mode == S1_MODEL_MODE_NORM)
    guard, guard_path = g["guard"], g["guard_path"]
    if sps != SYNC_SAMPLES_PER_SYMBOL:
        fail(item, "samples_per_symbol is the frozen work grid %d (got %d via %s)"
             % (SYNC_SAMPLES_PER_SYMBOL, sps, g["samples_per_symbol_path"]))
        return False
    if is_num(g.get("sample_rate")) and float(g["sample_rate"]) != WORK_HZ:
        fail(item, "the model coordinates are on the work grid (%r != %r)"
             % (g["sample_rate"], WORK_HZ))
    if fit_reps != S1_STRICT_FIT_REPS:
        fail(item, "fit_reps is %d (the strict 8-SYNC training); got %d via %s"
             % (S1_STRICT_FIT_REPS, fit_reps, g["fit_reps_path"]))
    if window.get("fit_reps") is not None \
            and is_int(window.get("fit_reps")) \
            and int(window["fit_reps"]) != fit_reps:
        fail(item, "window.fit_reps == model.fit_reps",
             "window=%r model=%r" % (window.get("fit_reps"), fit_reps))
    if guard is None:
        fail(item, "the guard length is explicitly declared",
             "no guard/guard_samples (0 is the strict 8-SYNC value); a hidden "
             "guard cannot be audited via %s" % (guard_path,))
    elif strict and (guard[0] or guard[1]):
        fail(item, "strict 8-SYNC declares a zero guard",
             "guard pre=%d post=%d via %s (report a guarded mode instead of "
             "calling it strict)" % (guard[0], guard[1], guard_path))
    if int(g["training_lo"]) != origin:
        fail(item, "training starts at repetition 0 (training_lo == origin)",
             "training_lo=%d origin=%d" % (g["training_lo"], origin))
    if int(g["training_hi"]) - int(g["training_lo"]) \
            != S1_STRICT_FIT_REPS * sps:
        fail(item, "training covers exactly the first 8 SYNC",
             "[%d,%d) span=%d != %d"
             % (g["training_lo"], g["training_hi"],
                int(g["training_hi"]) - int(g["training_lo"]),
                S1_STRICT_FIT_REPS * sps))
    observed_end = int(g["observed_end_sample"])
    training_hi = int(g["training_hi"])
    if strict and observed_end != training_hi:
        fail(item, "strict 8-SYNC observed_end_sample == training_hi",
             "observed_end=%d training_hi=%d (a strict claim cannot read a "
             "guard)" % (observed_end, training_hi))
    if guard is not None and not strict:
        if observed_end < training_hi + guard[0] + guard[1]:
            fail(item, "observed_end_sample covers the declared guard",
                 "observed_end=%d training_hi=%d guard=(%d,%d)"
                 % (observed_end, training_hi, guard[0], guard[1]))
    sample_count = window.get("sample_count")
    if is_int(sample_count) and not (0 < observed_end <= sample_count):
        fail(item, "observed_end_sample lies inside the window",
             "observed_end=%d sample_count=%d" % (observed_end, sample_count))
    ready = int(g["estimation_ready_sample"])
    if ready < observed_end:
        fail(item, "estimation_ready_sample >= observed_end_sample",
             "ready=%d observed_end=%d" % (ready, observed_end))
    if is_int(sample_count) and ready > sample_count:
        fail(item, "estimation_ready_sample lies inside the window",
             "ready=%d sample_count=%d" % (ready, sample_count))

    ranges, ranges_path = g["ranges"], g["ranges_path"]
    if not ranges:
        fail(item, "the estimation input ranges are declared",
             "no estimation_ranges mapping/list via %s" % (ranges_path,))
        return False
    required = {"training": ("training", "fit", "ls"),
                "cfo": ("cfo",),
                "cir": ("cir", "paths", "path_estimate", "cir_fit")}
    for label, aliases in required.items():
        if not any(alias in ranges for alias in aliases):
            fail(item, "the %s estimation input range is declared" % label,
                 "declared=%s via %s" % (sorted(ranges), ranges_path))
    worst_hi = None
    for name, (lo, hi) in sorted(ranges.items()):
        if not (0 <= lo < hi):
            fail(item, "estimation range %r is a non-empty [lo,hi)" % name,
                 "[%d,%d)" % (lo, hi))
            continue
        worst_hi = hi if worst_hi is None else max(worst_hi, hi)
        if hi > observed_end:
            fail(item, "estimation input %r stays before observed_end_sample"
                 % name,
                 "range=[%d,%d) observed_end=%d (reads %d sample(s) past the "
                 "declared visible prefix)" % (lo, hi, observed_end,
                                               hi - observed_end))
    if worst_hi is not None and ready < worst_hi:
        fail(item, "estimation_ready_sample >= every estimation range",
             "ready=%d max_range_hi=%d" % (ready, worst_hi))
    g["strict"] = strict
    return True


def _s1_check_frozen(window, g, where):
    """S1-2: frozen model identity + every frozen parameter + apply used it."""
    item = "%s S1-2 frozen-model evidence" % where
    model, mpath = s1_model_of(window)
    ok_so_far = True
    if not is_nonempty_str(g.get("model_id")):
        fail(item, "model_id is a non-empty string (got %r via %s)"
             % (g.get("model_id"), g["model_id_path"]))
        ok_so_far = False
    if not is_hex64(g.get("fingerprint")):
        fail(item, "fingerprint is a 64-char sha256 (got %r via %s)"
             % (g.get("fingerprint"), g["fingerprint_path"]))
        ok_so_far = False
    for name in ("origin", "n_paths", "support_lo", "support_hi", "sub_lo",
                 "sub_hi", "phase_reference"):
        if not is_int(g.get(name)):
            fail(item, "%s is an integer (got %r via %s)"
                 % (name, g.get(name), g.get(name + "_path")))
            ok_so_far = False
    if not is_num(g.get("cfo_hz")):
        fail(item, "cfo_hz is a finite number (got %r via %s)"
             % (g.get("cfo_hz"), g["cfo_hz_path"]))
        ok_so_far = False
    n_paths = g.get("n_paths")
    if is_int(n_paths) and 1 <= int(n_paths) <= 16:
        tau, gain = g.get("tau"), g.get("gain")
        if not (isinstance(tau, list) and len(tau) == int(n_paths)
                and all(is_num(v) for v in tau)):
            fail(item, "tau[] carries one fractional delay per path",
                 "n_paths=%d tau=%r" % (n_paths, tau))
            ok_so_far = False
        if not (isinstance(gain, list) and len(gain) == int(n_paths)
                and all(_s1_gain_pair(v) is not None for v in gain)):
            fail(item, "gain[] carries one complex gain per path",
                 "n_paths=%d gain=%r" % (n_paths, gain))
            ok_so_far = False
    elif is_int(n_paths):
        fail(item, "n_paths is in [1,16]", "got %d" % n_paths)
        ok_so_far = False
    for name in ("fit_residual", "cond2"):
        value = g.get(name)
        if not is_num(value) or (name == "cond2" and float(value) <= 0.0) \
                or (name == "fit_residual" and float(value) < 0.0):
            fail(item, "%s is a valid admission quantity (got %r via %s)"
                 % (name, value, g.get(name + "_path")))
            ok_so_far = False
    if not is_nonempty_str(g.get("status")):
        fail(item, "status/quality is a non-empty string (got %r)"
             % (g.get("status"),))
        ok_so_far = False

    apply, apath = g["apply"], g["apply_path"]
    if not isinstance(apply, dict):
        fail(item, "the apply step declares the frozen model it used",
             "no apply block via %s (S1 requires estimate/applied evidence)"
             % (apath,))
        return ok_so_far
    if apply.get("model_id") != g.get("model_id"):
        fail(item, "apply.model_id == the estimation model_id",
             "apply=%r model=%r" % (apply.get("model_id"), g.get("model_id")))
        ok_so_far = False
    fp = apply.get("fingerprint", apply.get("model_fingerprint"))
    if fp != g.get("fingerprint"):
        fail(item, "apply.fingerprint == the frozen estimation fingerprint",
             "apply=%r model=%r" % (fp, g.get("fingerprint")))
        ok_so_far = False
    refit, rpath = _first_key(apply, ("refit_count", "refits", "n_refits",
                                      "reestimate_count", "refit"))
    if not is_int(refit) or int(refit) != 0:
        fail(item, "apply performs no re-fit (refit_count == 0)",
             "got %r via %s (a post-estimation re-fit is not a frozen model)"
             % (refit, rpath))
        ok_so_far = False
    deltas, dpath = _first_key(apply, ("param_deltas", "deltas",
                                       "parameter_deltas", "max_param_delta",
                                       "delta"))
    if not (deltas is None or deltas == {} or deltas == [] or (
            is_num(deltas) and abs(float(deltas)) <= S1_PARAM_TOL)):
        fail(item, "apply reports no frozen-parameter delta",
             "param_deltas=%r via %s (a non-zero delta means the applied "
             "model differs from the estimation)" % (deltas, dpath))
        ok_so_far = False
    params = apply.get("params", apply.get("applied_params",
                                          apply.get("frozen_params")))
    if not isinstance(params, dict):
        fail(item, "apply carries the frozen params it used",
             "no apply.params mapping via %s" % (apath,))
        return ok_so_far
    for name in ("origin", "n_paths", "support_lo", "support_hi", "sub_lo",
                 "sub_hi", "phase_reference"):
        if not (is_int(params.get(name)) and is_int(g.get(name))
                and int(params[name]) == int(g[name])):
            fail(item, "apply.params.%s equals the frozen value" % name,
                 "apply=%r model=%r" % (params.get(name), g.get(name)))
            ok_so_far = False
    if not _s1_num_equal(params.get("cfo_hz"), g.get("cfo_hz")):
        fail(item, "apply.params.cfo_hz equals the frozen cfo_hz",
             "apply=%r model=%r" % (params.get("cfo_hz"), g.get("cfo_hz")))
        ok_so_far = False
    if not _s1_vec_equal(params.get("tau"), g.get("tau")):
        fail(item, "apply.params.tau[] equals the frozen tau[]",
             "apply=%r model=%r" % (params.get("tau"), g.get("tau")))
        ok_so_far = False
    if not _s1_vec_equal(params.get("gain"), g.get("gain"), pair=True):
        fail(item, "apply.params.gain[] equals the frozen gain[]",
             "apply=%r model=%r" % (params.get("gain"), g.get("gain")))
        ok_so_far = False
    return ok_so_far


def _s1_check_support(window, g, where):
    """S1-3: support is the configured-nominal prediction, not confirmed*sps."""
    item = "%s S1-3 support semantics" % where
    origin = g.get("origin")
    sps = g.get("samples_per_symbol")
    sync_reps = g.get("nominal_sync_reps")
    lo, hi = g.get("support_lo"), g.get("support_hi")
    pre, post = g.get("support_pre_tail"), g.get("support_post_tail")
    source = g.get("support_source")
    failures = 0
    if not is_nonempty_str(source):
        fail(item, "support_source is declared",
             "got %r via %s" % (source, g["support_source_path"]))
        failures += 1
    elif norm_text(source) not in S1_SUPPORT_SOURCES:
        fail(item, "support_source is the configured-nominal prediction",
             "got %r via %s (a confirmed/measured support is the legacy "
             "cover-mode coupling)" % (source, g["support_source_path"]))
        failures += 1
    if not (is_int(pre) and is_int(post)):
        fail(item, "support_pre_tail/support_post_tail are integers",
             "pre=%r post=%r (%s / %s)"
             % (pre, post, g.get("support_pre_tail_path"),
                g.get("support_post_tail_path")))
        return failures
    if not (is_int(origin) and is_int(sps) and is_int(sync_reps)
            and is_int(lo) and is_int(hi)):
        fail(item, "the support bounds and geometry are integers",
             "origin=%r sps=%r sync_reps=%r support=[%r,%r)"
             % (origin, sps, sync_reps, lo, hi))
        return failures
    want_lo = int(origin) - int(pre)
    want_hi = int(origin) + int(sync_reps) * int(sps) + int(post)
    if int(lo) != want_lo or int(hi) != want_hi:
        fail(item, "support == origin + sync_reps*sps + post_tail",
             "declared=[%d,%d) predicted=[%d,%d) (origin=%d sync_reps=%d "
             "sps=%d pre=%d post=%d); a support tied to a confirmed count is "
             "the old clip" % (lo, hi, want_lo, want_hi, origin, sync_reps,
                               sps, pre, post))
        failures += 1
    old_clip_hi = int(origin) + S1_STRICT_FIT_REPS * int(sps) + int(post)
    if int(hi) <= old_clip_hi:
        fail(item, "support_hi is not clipped to the first 8 SYNC",
             "support_hi=%d <= origin+8*sps+post_tail=%d (the old 8-SYNC clip)"
             % (hi, old_clip_hi))
        failures += 1
    if int(hi) - int(lo) < int(sync_reps) * int(sps):
        fail(item, "the support spans the full nominal preamble",
             "span=%d < sync_reps*sps=%d" % (hi - lo,
                                             int(sync_reps) * int(sps)))
        failures += 1
    sample_count = window.get("sample_count")
    if is_int(sample_count) and not (0 <= lo < hi <= sample_count):
        fail(item, "the support lies inside the declared window",
             "[%d,%d) sample_count=%d" % (lo, hi, sample_count))
        failures += 1
    coverage = g.get("support_coverage")
    if not (is_num(coverage) and 0.0 <= float(coverage) <= 1.0):
        fail(item, "support_coverage is reported in [0,1]",
             "got %r via %s" % (coverage, g.get("support_coverage_path")))
        failures += 1
    else:
        ov_lo, ov_hi = window_truth_overlap(window)
        state = window.get("sic_state") or {}
        partial = bool(state.get("partial")) or bool(state.get("ambiguous"))
        if ov_lo is not None and not partial:
            covered = max(0, min(hi, ov_hi) - max(lo, ov_lo))
            want = covered / float(ov_hi - ov_lo)
            if want < S1_SUPPORT_COVERAGE:
                fail(item, "the predicted support covers >= %.0f%% of the "
                     "truth overlap" % (100.0 * S1_SUPPORT_COVERAGE),
                     "covered=%d/%d (%.4f) support=[%d,%d) overlap=[%d,%d)"
                     % (covered, ov_hi - ov_lo, want, lo, hi, ov_lo, ov_hi))
                failures += 1
            if abs(float(coverage) - want) > 1e-3:
                fail(item, "support_coverage matches the declared support",
                     "declared=%r recomputed=%.4f" % (coverage, want))
                failures += 1
    return failures


def _s1_lookup_any(window, keys):
    """Find a declaration key at the window top level or in a known block."""
    for key in keys:
        if key in window:
            return window[key], key
    for name in ("sic_model", "frozen_model", "model", "regions", "region",
                 "recon", "config", "sic_config", "sic", "s1"):
        block = window.get(name)
        if isinstance(block, dict):
            for key in keys:
                if key in block:
                    return block[key], "%s.%s" % (name, key)
    return None, None


def _s1_check_clipped_region(item, label, nominal, declared, sub, support,
                             overlap, covered, covered_path, failures):
    """Accept a nominal or nominal∩sub∩overlap-clipped eval/tail region.

    The region is DEFINED as the nominal repetition range intersected with the
    applied subtraction (``sub``) and the generator truth overlap.  A run that
    cannot cancel the whole nominal region must declare that clip and its
    covered fraction.  A declared region that extends outside ``sub``/overlap
    while claiming the full nominal coverage is a FAIL.
    """
    n_lo, n_hi = nominal
    d_lo, d_hi = declared
    sub_lo, sub_hi = sub
    sup_lo, sup_hi = support
    ov_lo, ov_hi = overlap
    span = n_hi - n_lo
    eff_lo = max(n_lo, sub_lo, ov_lo)
    eff_hi = min(n_hi, sub_hi, ov_hi)
    if eff_lo >= eff_hi:
        fail(item, "the %s nominal region intersects the applied subtraction "
             "and the truth overlap" % label,
             "nominal=[%d,%d) sub=[%d,%d) overlap=[%d,%d)"
             % (n_lo, n_hi, sub_lo, sub_hi, ov_lo, ov_hi))
        return failures + 1
    if not (sup_lo <= eff_lo and eff_hi <= sup_hi):
        fail(item, "the %s covered region is inside the predicted support"
             % label, "%s covered=[%d,%d) support=[%d,%d)"
             % (label, eff_lo, eff_hi, sup_lo, sup_hi))
        failures += 1
    clipped = (eff_lo, eff_hi) != (n_lo, n_hi)
    frac_want = (eff_hi - eff_lo) / float(span) if span > 0 else 0.0
    # The declared region may be the nominal repetition range (the app keeps
    # ``eval_lo``/``eval_hi`` nominal and declares the clip separately) or the
    # recomputed clip itself.
    if (d_lo, d_hi) != (n_lo, n_hi) and (d_lo, d_hi) != (eff_lo, eff_hi):
        if d_lo < eff_lo or d_hi > eff_hi:
            fail(item, "the %s region does not extend outside the applied "
                 "subtraction / truth overlap while claiming coverage" % label,
                 "%s=[%d,%d) sub=[%d,%d) overlap=[%d,%d)"
                 % (label, d_lo, d_hi, sub_lo, sub_hi, ov_lo, ov_hi))
        else:
            fail(item, "the %s region is the nominal repetition range or its "
                 "nominal∩sub∩overlap clip" % label,
                 "%s=[%d,%d) nominal=[%d,%d) clip=[%d,%d)"
                 % (label, d_lo, d_hi, n_lo, n_hi, eff_lo, eff_hi))
        return failures + 1
    if not clipped:
        # The whole nominal region is inside sub and the truth overlap: a
        # covered fraction, if declared, must be the full 1.0.
        if is_num(covered) and abs(float(covered) - 1.0) > 1e-3:
            fail(item, "the %s covered fraction is 1.0 when nothing is clipped"
                 % label, "declared=%r via %s" % (covered, covered_path))
            failures += 1
        return failures
    # The nominal region is clipped by sub/overlap: the covered fraction must be
    # declared and match the recomputed intersection, and it must not claim the
    # full nominal coverage.
    if not is_num(covered):
        fail(item, "the clipped %s region declares its covered fraction" % label,
             "nominal=[%d,%d) clip=[%d,%d) but no eval/tail_covered_frac is "
             "declared (%s)" % (n_lo, n_hi, eff_lo, eff_hi, covered_path))
        return failures + 1
    if abs(float(covered) - frac_want) > 1e-3:
        fail(item, "the %s covered fraction matches the recomputed "
             "nominal∩sub∩overlap intersection" % label,
             "declared=%r recomputed=%.4f (nominal span=%d clip span=%d)"
             % (covered, frac_want, span, eff_hi - eff_lo))
        failures += 1
    elif float(covered) >= 1.0 - 1e-3:
        fail(item, "the clipped %s region does not claim full coverage" % label,
             "covered=%r but sub/overlap clip the nominal region" % (covered,))
        failures += 1
    return failures


def _s1_check_regions(window, g, where):
    """S1-4: training/eval/tail regions + eval and tail are cancelled."""
    item = "%s S1-4 regions" % where
    origin = g.get("origin")
    sps = g.get("samples_per_symbol")
    sync_reps = g.get("nominal_sync_reps")
    training_lo, training_hi = g.get("training_lo"), g.get("training_hi")
    eval_lo, eval_hi = g.get("eval_lo"), g.get("eval_hi")
    tail_lo, tail_hi = g.get("tail_lo"), g.get("tail_hi")
    sub_lo, sub_hi = g.get("sub_lo"), g.get("sub_hi")
    failures = 0
    for name in ("training_lo", "training_hi", "eval_lo", "eval_hi",
                 "tail_lo", "tail_hi", "sub_lo", "sub_hi", "origin",
                 "samples_per_symbol", "nominal_sync_reps"):
        if not is_int(g.get(name)):
            fail(item, "%s is an integer (got %r via %s)"
                 % (name, g.get(name), g.get(name + "_path")))
            return failures + 1
    # The optional 8-training + N-online-validation contrast shifts the
    # independent evaluation start by the online-validation span.
    online_reps, _ = _first_key(window, ("online_validation_reps",
                                         "validation_reps"))
    if not is_int(online_reps):
        online_reps, _ = _region_lookup(window, ("online_validation_reps",
                                                 "validation_reps"))
    if not is_int(online_reps):
        online_reps = 0
    if online_reps < 0:
        fail(item, "online_validation_reps is a non-negative integer",
             "got %r" % (online_reps,))
        online_reps = 0
    if not (0 <= sub_lo < sub_hi):
        fail(item, "the applied subtraction range is a non-empty [lo,hi)",
             "[%d,%d)" % (sub_lo, sub_hi))
        failures += 1
    # The nominal eval/tail repetition ranges: the independent evaluation is the
    # [fit+online, sync_reps) repetition span of the configured preamble, NOT a
    # span measured from a (possibly clipped) training end.
    if is_int(g.get("fit_reps")):
        want_eval_lo = int(origin) + int(g["fit_reps"]) * int(sps) \
            + int(online_reps) * int(sps)
    else:
        want_eval_lo = int(training_hi) + int(online_reps) * int(sps)
    want_eval_hi = int(origin) + int(sync_reps) * int(sps)
    if int(eval_lo) < want_eval_lo:
        fail(item, "the independent evaluation region starts after "
             "training%s" % ("+online-validation" if online_reps else ""),
             "eval_lo=%d expected>=%d (online_reps=%d)"
             % (eval_lo, want_eval_lo, online_reps))
        failures += 1
    if _intervals_overlap((int(eval_lo), int(eval_hi)),
                          (int(training_lo), int(training_hi))):
        fail(item, "the independent evaluation region is disjoint from training",
             "eval=[%d,%d) training=[%d,%d)"
             % (eval_lo, eval_hi, training_lo, training_hi))
        failures += 1
    declared_eval_reps, _ = _first_key(window, ("evaluation_reps",
                                                "independent_eval_reps"))
    if is_int(declared_eval_reps) \
            and int(declared_eval_reps) != (int(sync_reps) - S1_STRICT_FIT_REPS
                                            - int(online_reps)):
        fail(item, "evaluation_reps partitions the 64-SYNC preamble",
             "evaluation_reps=%d expected %d (8 training + %d online + eval)"
             % (declared_eval_reps,
                int(sync_reps) - S1_STRICT_FIT_REPS - int(online_reps),
                online_reps))
        failures += 1
    if int(eval_hi) - int(eval_lo) < S1_MIN_EVAL_REPS * int(sps):
        fail(item, "the evaluation region spans >= %d SYNC" % S1_MIN_EVAL_REPS,
             "span=%d < %d" % (eval_hi - eval_lo, S1_MIN_EVAL_REPS * sps))
        failures += 1
    want_tail_lo = int(origin) + (int(sync_reps) - S1_TAIL_REPS) * int(sps)
    want_tail_hi = int(origin) + int(sync_reps) * int(sps)
    if _intervals_overlap((int(tail_lo), int(tail_hi)),
                          (int(training_lo), int(training_hi))):
        fail(item, "the tail-eval region is disjoint from training",
             "tail=[%d,%d) training=[%d,%d)"
             % (tail_lo, tail_hi, training_lo, training_hi))
        failures += 1
    if not _within((int(tail_lo), int(tail_hi)), (int(eval_lo), int(eval_hi))):
        fail(item, "the tail-eval region lies inside the independent evaluation",
             "tail=[%d,%d) eval=[%d,%d)" % (tail_lo, tail_hi, eval_lo, eval_hi))
        failures += 1
    ov_lo, ov_hi = window_truth_overlap(window)
    if ov_lo is None:
        fail(item, "the truth overlap is well-formed",
             "overlap=%r" % (window.get("overlap"),))
        return failures + 1
    support_lo, support_hi = g.get("support_lo"), g.get("support_hi")
    support = (int(support_lo), int(support_hi)) \
        if is_int(support_lo) and is_int(support_hi) else None
    if support is not None and not (support[0] <= sub_lo and sub_hi <= support[1]):
        fail(item, "the applied subtraction is inside the predicted support",
             "sub=[%d,%d) support=[%d,%d)"
             % (sub_lo, sub_hi, support[0], support[1]))
        failures += 1
    # The eval/tail regions are the nominal repetition ranges intersected with
    # the applied subtraction and the truth overlap.  A run that cannot cancel
    # the whole nominal region must declare the clip and its covered fraction.
    eval_covered, eval_cpath = _s1_lookup_any(window, S1_EVAL_COVERED_KEYS)
    tail_covered, tail_cpath = _s1_lookup_any(window, S1_TAIL_COVERED_KEYS)
    sub = (int(sub_lo), int(sub_hi))
    support_for_clip = support if support is not None else sub
    failures = _s1_check_clipped_region(
        item, "independent evaluation", (want_eval_lo, want_eval_hi),
        (int(eval_lo), int(eval_hi)), sub, support_for_clip,
        (int(ov_lo), int(ov_hi)), eval_covered, eval_cpath, failures)
    failures = _s1_check_clipped_region(
        item, "tail-eval", (want_tail_lo, want_tail_hi),
        (int(tail_lo), int(tail_hi)), sub, support_for_clip,
        (int(ov_lo), int(ov_hi)), tail_covered, tail_cpath, failures)
    # The gate is decided on training/online data, never on the eval region.
    gate_lo, gate_path = _first_key(g, ("gate_lo", "commit_gate_lo"))
    gate_hi, _ = _first_key(g, ("gate_hi", "commit_gate_hi"))
    if not (is_int(gate_lo) and is_int(gate_hi)):
        gate_lo, gate_path = _region_lookup(window, GATE_LO_KEYS)
        gate_hi, _ = _region_lookup(window, GATE_HI_KEYS)
    if not (is_int(gate_lo) and is_int(gate_hi) and gate_lo < gate_hi):
        fail(item, "the commit gate region is declared",
             "could not read it (via %s)" % (gate_path,))
        failures += 1
    else:
        if _intervals_overlap((int(gate_lo), int(gate_hi)),
                              (int(eval_lo), int(eval_hi))):
            fail(item, "the gate does not use evaluation-region data",
                 "gate=[%d,%d) eval=[%d,%d)"
                 % (gate_lo, gate_hi, eval_lo, eval_hi))
            failures += 1
        if not _within((int(gate_lo), int(gate_hi)),
                       (int(training_lo), int(want_eval_lo))):
            fail(item, "the gate is decided on the training/online region",
                 "gate=[%d,%d) allowed=[%d,%d) (online_reps=%d; never eval)"
                 % (gate_lo, gate_hi, training_lo, want_eval_lo, online_reps))
            failures += 1
    return failures


def _s1_covered_bounds(recon, window, label):
    """The declared covered (nominal∩sub∩overlap) bounds for eval/tail, if any."""
    lo_key = "eval_actual_lo" if label == "eval" else "tail_actual_lo"
    hi_key = "eval_actual_hi" if label == "eval" else "tail_actual_hi"
    cov_key = "eval_covered" if label == "eval" else "tail_covered"
    for src in (recon, window.get("regions"), window.get("sic_model")):
        if not isinstance(src, dict):
            continue
        value = src.get(cov_key)
        if isinstance(value, (list, tuple)) and len(value) == 2 \
                and all(is_int(v) for v in value):
            return (int(value[0]), int(value[1]))
        if is_int(src.get(lo_key)) and is_int(src.get(hi_key)):
            return (int(src[lo_key]), int(src[hi_key]))
    return None


def _s1_check_actual(window, g, recon, arrays, where):
    """S1-5: actual-output suppression over full / eval / tail, from arrays."""
    item = "%s S1-5 actual-output cancellation" % where
    result = {"eval": None, "tail": None, "full": None,
              "eval_na": False, "tail_na": False, "full_na": False}
    if arrays is None:
        fail(item, "archived arrays exist to recompute the three regions",
             "no recon/window_*.npz for window_id=%r" % (window.get("window_id"),))
        return result
    if "error" in arrays:
        fail(item, "archived arrays are readable", arrays.get("error"))
        return result
    missing = [k for k in ("y", "y_clean", "i") if k not in arrays]
    if missing:
        fail(item, "archived arrays carry y, y_clean and i",
             "missing %s in %s" % (missing, os.path.basename(arrays.get("path", "?"))))
        return result
    y, y_clean, i = arrays["y"], arrays["y_clean"], arrays["i"]
    if not (len(y) == len(y_clean) == len(i)):
        fail(item, "archived arrays are the same length",
             "y=%d y_clean=%d i=%d" % (len(y), len(y_clean), len(i)))
        return result
    ov_lo, ov_hi = window_truth_overlap(window)
    if ov_lo is None:
        fail(item, "the truth overlap is well-formed",
             "overlap=%r" % (window.get("overlap"),))
        return result
    if ov_hi > len(i):
        fail(item, "the truth overlap fits the archived arrays",
             "[%d,%d) > %d" % (ov_lo, ov_hi, len(i)))
        return result
    delta = y - y_clean
    sub_lo, sub_hi = g.get("sub_lo"), g.get("sub_hi")

    def _effective(lo, hi):
        # The eval/tail region for the actual metric is the nominal (or
        # declared) repetition range intersected with the applied subtraction
        # and the truth overlap -- the same clip the covered fraction names.
        if is_int(lo) and is_int(hi) and is_int(sub_lo) and is_int(sub_hi):
            return max(int(lo), int(sub_lo), ov_lo), \
                min(int(hi), int(sub_hi), ov_hi)
        return lo, hi

    eval_lo, eval_hi = g.get("eval_lo"), g.get("eval_hi")
    tail_lo, tail_hi = g.get("tail_lo"), g.get("tail_hi")
    if is_int(eval_lo) and is_int(eval_hi):
        eval_lo, eval_hi = _effective(eval_lo, eval_hi)
    if is_int(tail_lo) and is_int(tail_hi):
        tail_lo, tail_hi = _effective(tail_lo, tail_hi)
    regions = (("full", ov_lo, ov_hi), ("eval", eval_lo, eval_hi),
               ("tail", tail_lo, tail_hi))
    declared = {
        "full": (recon.get("actual_suppression_db"), "recon.actual_suppression_db"),
        "eval": (next((recon[k] for k in S1_EVAL_ACTUAL_KEYS if k in recon),
                      None),
                 "recon.%s" % (next((k for k in S1_EVAL_ACTUAL_KEYS
                                     if k in recon), "eval_actual_suppression_db"))),
        "tail": (next((recon[k] for k in S1_TAIL_ACTUAL_KEYS if k in recon),
                      None),
                 "recon.%s" % (next((k for k in S1_TAIL_ACTUAL_KEYS
                                     if k in recon), "tail_actual_suppression_db"))),
    }
    for label, lo, hi in regions:
        if not (is_int(lo) and is_int(hi) and 0 <= lo < hi):
            fail(item, "%s region is declared before recomputing it" % label,
                 "got [%r,%r)" % (lo, hi))
            continue
        if label != "full":
            if not (ov_lo <= lo and hi <= ov_hi):
                fail(item, "the %s region lies inside the truth overlap" % label,
                     "%s=[%d,%d) overlap=[%d,%d)"
                     % (label, lo, hi, ov_lo, ov_hi))
                continue
            covered_pair = _s1_covered_bounds(recon, window, label)
            if covered_pair is not None and covered_pair != (lo, hi):
                fail(item, "the declared %s covered region is the "
                     "nominal∩sub∩overlap clip" % label,
                     "declared covered=[%d,%d) recomputed=[%d,%d)"
                     % (covered_pair[0], covered_pair[1], lo, hi))
            if hi > len(i):
                fail(item, "the %s region fits the archived arrays" % label,
                     "[%d,%d) > %d" % (lo, hi, len(i)))
                continue
        value, _n, den = _s1_actual_metric(i, delta, lo, hi)
        present_key = all(k not in recon for k in (
            S1_EVAL_ACTUAL_KEYS if label == "eval"
            else (S1_TAIL_ACTUAL_KEYS if label == "tail" else ())))
        if label != "full" and present_key:
            fail(item, "%s_actual_suppression_db is reported" % label,
                 "recon lacks %s (a reconstruction nmse is not the actual "
                 "output metric)"
                 % (S1_EVAL_ACTUAL_KEYS if label == "eval"
                    else S1_TAIL_ACTUAL_KEYS,))
        if value is None:
            result[label] = None
            result[label + "_na"] = True
            if declared[label][0] is not None:
                fail(item, "%s actual suppression is N/A for zero "
                     "interference" % label,
                     "%s=%r but ||i||^2 == 0" % (declared[label][1],
                                                 declared[label][0]))
            continue
        result[label] = value
        declared_value = declared[label][0]
        if not is_num(declared_value):
            fail(item, "%s actual_suppression_db is reported" % label,
                 "%s=%r (recomputed %.4f dB)"
                 % (declared[label][1], declared_value, value))
        elif abs(float(declared_value) - value) > S1_ACTUAL_TOL_DB:
            fail(item, "%s actual_suppression_db matches the arrays" % label,
                 "%s=%r recomputed=%.4f dB"
                 % (declared[label][1], declared_value, value))
        if label != "full":
            frac = _s1_changed_fraction(y, y_clean, lo, hi)
            if frac < S1_CANCEL_FRACTION:
                fail(item, "the %s region is actually cancelled" % label,
                     "only %.2f%% of the %s samples changed in y!=y_clean "
                     "(the region is left un-cancelled)"
                     % (100.0 * frac, label))
    return result


def _s1_declared_region_value(recon, keys):
    for key in keys:
        if key in recon:
            value = recon[key]
            return True, (float(value) if is_num(value) else None)
    return False, None


def _s1_check_summary(summary, windows, prefix):
    """S1-5 at the run level: the three medians/counts are required."""
    item = "%s S1-5 actual effect medians" % prefix
    effect = summary.get("effect") if isinstance(summary, dict) else None
    if not isinstance(effect, dict):
        fail(item, "summary.effect carries the actual-output aggregates",
             "summary.effect is absent; reconstruction NMSE alone is not the "
             "S1 effect report")
        return
    full_vals, eval_vals, tail_vals = [], [], []
    full_na = eval_na = tail_na = 0
    absent_full = absent_eval = absent_tail = 0
    for window in windows:
        if window_uses_truth(window):
            continue
        recon = window.get("recon")
        if not isinstance(recon, dict):
            continue
        if "actual_suppression_db" in recon:
            if is_num(recon["actual_suppression_db"]):
                full_vals.append(float(recon["actual_suppression_db"]))
            else:
                full_na += 1
        else:
            absent_full += 1
        present, value = _s1_declared_region_value(recon, S1_EVAL_ACTUAL_KEYS)
        if not present:
            absent_eval += 1
        elif value is None:
            eval_na += 1
        else:
            eval_vals.append(value)
        present_t, value_t = _s1_declared_region_value(recon, S1_TAIL_ACTUAL_KEYS)
        if not present_t:
            absent_tail += 1
        elif value_t is None:
            tail_na += 1
        else:
            tail_vals.append(value_t)
    if absent_full:
        fail(item, "every evaluated window reports actual_suppression_db",
             "%d window(s) lack it" % absent_full)
    if absent_eval:
        fail(item, "every evaluated window reports eval_actual_suppression_db",
             "%d window(s) lack it (reconstruction nmse does not substitute)"
             % absent_eval)
    if absent_tail:
        fail(item, "every evaluated window reports tail_actual_suppression_db",
             "%d window(s) lack it (reconstruction nmse does not substitute)"
             % absent_tail)
    plans = (
        ("full overlap", ("median_actual_suppression_db",),
         ("n_interference_windows",), ("n_na_windows",),
         full_vals, full_na,
         effect.get("median_actual_suppression_db")),
        ("independent evaluation", S1_EVAL_MEDIAN_KEYS, S1_EVAL_COUNT_KEYS,
         S1_EVAL_NA_KEYS, eval_vals, eval_na,
         next((effect[k] for k in S1_EVAL_MEDIAN_KEYS if k in effect), None)),
        ("tail-4-SYNC", S1_TAIL_MEDIAN_KEYS, S1_TAIL_COUNT_KEYS,
         S1_TAIL_NA_KEYS, tail_vals, tail_na,
         next((effect[k] for k in S1_TAIL_MEDIAN_KEYS if k in effect), None)),
    )
    for label, med_keys, count_keys, na_keys, vals, na, declared in plans:
        med_present = any(k in effect for k in med_keys)
        if vals:
            want = median(vals)
            if not med_present:
                fail(item, "%s median is reported" % label,
                     "no effect key in %s (reconstruction-only reporting is "
                     "not the S1 effect gate)" % (med_keys,))
            elif not is_num(declared) or abs(float(declared) - want) > 1e-6:
                fail(item, "%s median matches the per-window actual values"
                     % label, "summary=%r recomputed=%.4f" % (declared, want))
            else:
                ok(item, "%s: median %.4f dB over %d window(s)%s"
                   % (label, want, len(vals),
                      ", %d N/A" % na if na else ""))
        else:
            if med_present and declared is not None:
                fail(item, "%s median is N/A when every window is N/A" % label,
                     "summary=%r" % (declared,))
        count, cnt_path = _first_key(effect, count_keys)
        if not is_int(count) or int(count) != len(vals):
            fail(item, "%s window count matches the per-window metrics" % label,
                 "effect.%s=%r recomputed=%d"
                 % (cnt_path or count_keys[0], count, len(vals)))
        count_na, cnt_na_path = _first_key(effect, na_keys)
        if not is_int(count_na) or int(count_na) != na:
            fail(item, "%s N/A count matches the evaluated windows" % label,
                 "effect.%s=%r recomputed=%d"
                 % (cnt_na_path or na_keys[0], count_na, na))


def _s1_archive_policy(manifest, summary):
    """The declared reconstruction archive policy, or (None, None).

    Accepted at the summary/manifest top level or nested in a known block
    (the app writes ``summary.sic.archive_policy``).
    """
    for src, name in ((summary, "summary.json"), (manifest, "run_manifest.json")):
        if not isinstance(src, dict):
            continue
        for key in S1_ARCHIVE_POLICY_KEYS:
            block = src.get(key)
            if isinstance(block, dict):
                return block, "%s.%s" % (name, key)
        for sub in ("sic", "recon", "s1", "reconstruction", "artifacts",
                    "runtime", "config"):
            block = src.get(sub)
            if not isinstance(block, dict):
                continue
            for key in S1_ARCHIVE_POLICY_KEYS:
                policy = block.get(key)
                if isinstance(policy, dict):
                    return policy, "%s.%s.%s" % (name, sub, key)
    return None, None


def _s1_archived_ids(policy):
    """The archived window-id set declared by the policy."""
    raw, key = _first_key(policy, S1_ARCHIVED_ID_KEYS)
    ids = set()
    if isinstance(raw, list):
        for value in raw:
            if is_int(value):
                ids.add(str(value))
            elif is_nonempty_str(value):
                ids.add(value.strip())
    elif isinstance(raw, dict):
        for entry_key, value in raw.items():
            if value is True or (is_nonempty_str(value)
                                 and value.strip().lower() in
                                 ("archived", "true", "yes", "1")):
                ids.add(str(entry_key))
    return ids, key


def _s1_archive_limitation(policy, summary):
    """A declared limitation string from the policy or the summary."""
    keys = ("limitation", "limitations", "non_archived", "non_archived_note",
            "declared_limitation", "note", "notes", "caveat")
    for src in (policy, summary):
        if not isinstance(src, dict):
            continue
        for key in keys:
            value = src.get(key)
            if is_nonempty_str(value):
                return value
            if isinstance(value, list) \
                    and any(is_nonempty_str(v) for v in value):
                return " ".join(str(v) for v in value if is_nonempty_str(v))
    return None


def _s1_limitation_declared(text):
    """True when `text` states that something is NOT independently recomputable."""
    if not is_nonempty_str(text):
        return False
    norm = norm_text(text)
    tokens = set(norm.split())
    negated = bool(tokens & {"not", "non", "no", "unable", "cannot",
                             "without", "never"})
    return ("recomput" in norm) and negated


def _s1_check_archive_policy(manifest, summary, windows, prefix):
    """Validate a declared reconstruction archive policy.

    Returns ``(policy, archived_ids)``.  ``archived_ids`` is the set of window
    ids that must ship archived ``recon/window_*.npz`` arrays.  When no policy
    is declared the caller keeps requiring arrays for every committed window.
    """
    item = "%s S1 archive policy" % prefix
    policy, path = _s1_archive_policy(manifest, summary)
    if policy is None:
        return None, None
    n_windows = len(windows)
    all_ids = {str(w.get("window_id")) for w in windows}
    ids, ids_path = _s1_archived_ids(policy)
    if not ids:
        fail("%s: archived_ids declares the archived windows" % item,
             "no non-empty archived_ids list/dict via %s" % (path,))
        return None, None
    bad = 0
    unknown = sorted(i for i in ids if i not in all_ids)
    if unknown:
        fail("%s: every archived id names a window in windows.jsonl" % item,
             "unknown ids %s" % unknown[:8])
        bad += 1
    n_archived = len(ids & all_ids)
    per_cell, per_path = _first_key(policy, S1_ARCHIVED_COUNT_KEYS)
    if not is_int(per_cell) or int(per_cell) < S1_MIN_ARCHIVED:
        fail("%s: per_cell declares >= %d archived windows per cell/case"
             % (item, S1_MIN_ARCHIVED),
             "%s=%r" % (per_path or "per_cell", per_cell))
        bad += 1
    elif n_archived < int(per_cell):
        fail("%s: at least per_cell archived windows are present" % item,
             "per_cell=%d archived=%d" % (per_cell, n_archived))
        bad += 1
    if n_archived < S1_MIN_ARCHIVED:
        fail("%s: at least %d archived windows per cell/case"
             % (item, S1_MIN_ARCHIVED), "archived=%d" % n_archived)
        bad += 1
    total, total_path = _first_key(policy, S1_ARCHIVED_TOTAL_KEYS)
    if not is_int(total) or int(total) != n_windows:
        fail("%s: total_windows matches the run window count" % item,
             "%s=%r windows.jsonl=%d"
             % (total_path or "total_windows", total, n_windows))
        bad += 1
    if n_archived < n_windows:
        limitation = _s1_archive_limitation(policy, summary)
        if not _s1_limitation_declared(limitation):
            fail("%s: the non-archived windows are declared not independently "
                 "recomputable" % item,
                 "limitation=%r via %s (a declared limitation is required)"
                 % (limitation, path))
            bad += 1
    if not bad:
        ok("%s: policy archives %d/%d windows and states the non-archived "
           "limitation" % (item, n_archived, n_windows), path)
    return policy, ids


def _s1_generator_of(window):
    for name in S1_SEED_CONTAINERS:
        block = window.get(name)
        if isinstance(block, dict):
            return block, name
    return None, None


def _s1_check_seeds(manifest, summary, windows, prefix):
    """S1-6: acceptance seeds are recorded and disjoint from calibration."""
    item = "%s S1-6 seed independence" % prefix
    if not isinstance(manifest, dict):
        return
    seeds = manifest.get("seed_lists")
    if not isinstance(seeds, dict):
        fail(item, "manifest.seed_lists declares calibration and acceptance",
             "got %r" % (seeds,))
        return
    cal, acc = seeds.get("calibration"), seeds.get("acceptance")
    if not (isinstance(cal, list) and cal and all(is_int(v) for v in cal)):
        fail(item, "the calibration seed set is recorded",
             "seed_lists.calibration=%r" % (cal,))
        cal = []
    if not (isinstance(acc, list) and acc and all(is_int(v) for v in acc)):
        fail(item, "the formal acceptance seed set is recorded",
             "seed_lists.acceptance=%r" % (acc,))
        acc = []
    shared = sorted(set(cal) & set(acc))
    if shared:
        fail(item, "calibration and acceptance seeds are disjoint by name",
             "%d shared seed(s), first %s" % (len(shared), shared[:5]))
    else:
        ok(item, "calibration and acceptance seed sets are disjoint by name",
           "%d + %d" % (len(cal), len(acc)))
    cal_windows, cal_path = _first_key(manifest, (
        "calibration_windows", "generator_calibration_windows",
        "calibration_window_set"))
    cal_tuples = set()
    if not isinstance(cal_windows, list) or not cal_windows:
        fail(item, "the actual calibration windows are recorded",
             "no calibration_windows[] via %s (a disjoint name list alone "
             "cannot prove the windows were not reused)" % (cal_path,))
    else:
        for entry in cal_windows:
            if not isinstance(entry, dict):
                continue
            seed = entry.get("seed")
            offset = entry.get("seed_offset", 0)
            root = entry.get("noise_root", entry.get("noise_seed"))
            if is_int(seed) and is_int(offset) and is_nonempty_str(root):
                cal_tuples.add((int(seed), int(offset), str(root)))
        if len(cal_tuples) != len(cal_windows):
            fail(item, "every calibration window carries seed/seed_offset/"
                 "noise_root", "declared=%d usable=%d"
                 % (len(cal_windows), len(cal_tuples)))
        else:
            ok(item, "calibration windows are recorded", "%d tuple(s)"
               % len(cal_tuples))
    cal_seed_set = set(cal)
    seen = {}
    reused_seed, reused_tuple = [], []
    missing_gen = 0
    for window in windows:
        wid = window.get("window_id")
        gen, gpath = _s1_generator_of(window)
        if not isinstance(gen, dict):
            missing_gen += 1
            continue
        seed_set = gen.get("seed_set", gen.get("set", gen.get("split")))
        seed = gen.get("seed")
        offset = gen.get("seed_offset", 0)
        root = gen.get("noise_root", gen.get("noise_seed"))
        if not (is_nonempty_str(seed_set) and is_int(seed)
                and is_int(offset) and is_nonempty_str(root)):
            fail(item, "window_id=%r declares the generated-window identity"
                 % (wid,),
                 "generator block via %s needs seed_set/seed/seed_offset/"
                 "noise_root (got %r)" % (gpath, gen))
            continue
        if norm_text(seed_set) != "acceptance":
            fail(item, "window_id=%r belongs to the acceptance set" % (wid,),
                 "seed_set=%r (the formal effect matrix must not run on the "
                 "calibration set)" % (seed_set,))
        if int(seed) in cal_seed_set:
            reused_seed.append(wid)
        key = (int(seed), int(offset), str(root))
        if key in cal_tuples:
            reused_tuple.append(wid)
        elif key in seen:
            fail(item, "window_id=%r does not reuse another acceptance window"
                 % (wid,), "generated tuple %r already used by window_id=%r"
                 % (key, seen[key]))
        else:
            seen[key] = wid
    if reused_seed:
        fail(item, "no acceptance window reuses a calibration seed",
             "window_id(s) %s" % reused_seed[:5])
    if reused_tuple:
        fail(item, "no acceptance window reuses a calibration window/noise",
             "window_id(s) %s (seed, seed_offset, noise_root) collide with "
             "calibration_windows" % reused_tuple[:5])
    if missing_gen:
        fail(item, "every window declares its generator seed identity",
             "%d window(s) lack a generator block" % missing_gen)
    if not (reused_seed or reused_tuple or missing_gen):
        ok(item, "acceptance windows are disjoint from the calibration windows",
           "%d unique generated window(s)" % len(seen))


def _s1_layer_mode(entry):
    if not isinstance(entry, dict):
        return None, None
    value, key = _first_key(entry, ("sic_model_mode", "model_mode",
                                    "method_mode", "mode"))
    return (norm_text(value) if is_nonempty_str(value) else None), key


def _s1_index_path(results_dir):
    for name in S1_INDEX_FILES:
        path = os.path.join(results_dir, name)
        if os.path.isfile(path):
            return path, os.path.abspath(results_dir)
    if _AGGREGATE_CHILD:
        # A child of an aggregate root is not the results-root of the global
        # index; only an index inside the aggregate root itself would describe
        # it, and that is reported once for the aggregate.  Do not fall back to
        # a sibling/global index.json.
        return None, None
    parent = os.path.dirname(os.path.abspath(results_dir))
    if parent != os.path.abspath(results_dir):
        for name in S1_INDEX_FILES:
            path = os.path.join(parent, name)
            if os.path.isfile(path):
                return path, parent
    return None, None


def _s1_run_dirs(root):
    found = []
    for dirpath, dirnames, filenames in os.walk(root):
        if "summary.json" in filenames and "run_manifest.json" in filenames:
            found.append(dirpath)
            dirnames[:] = []
    return found


def _s1_newest_run(root):
    runs = _s1_run_dirs(root)
    if not runs:
        return None

    def stamp(path):
        return os.path.getmtime(os.path.join(path, "summary.json"))
    return max(runs, key=stamp)


def _s1_resolve_index_run(value, root):
    if isinstance(value, dict):
        value = value.get("dir") or value.get("path") or value.get("label") \
            or value.get("run")
    if not is_nonempty_str(value):
        return None
    direct = value if os.path.isabs(value) else os.path.join(root, value)
    if os.path.isdir(direct) \
            and os.path.isfile(os.path.join(direct, "summary.json")):
        return os.path.abspath(direct)
    target = os.path.basename(os.path.normpath(value))
    for path in _s1_run_dirs(root):
        if os.path.basename(path) == target:
            return os.path.abspath(path)
    return None


def _s1_check_index(results_dir, run_dir, manifest, summary, prefix):
    item = "%s S1-7 results index" % prefix
    path, root = _s1_index_path(results_dir)
    if path is None:
        if _AGGREGATE_CHILD:
            skip(item, "aggregate child: no per-cell results index is required "
                 "(the aggregate root owns the results-root index.json)")
            return
        fail(item, "results-root index.json is present",
             "no %s next to %s or its parent (the index must reference the "
             "actual latest run)" % ("/".join(S1_INDEX_FILES), results_dir))
        return
    data, err = load_json(path)
    if not isinstance(data, dict):
        fail(item, "results index is a JSON object", err or "not an object")
        return
    latest = data.get("latest_run", data.get("latest"))
    runs_block = data.get("runs")
    if latest is None and isinstance(runs_block, dict):
        latest = runs_block.get("latest")
    resolved = _s1_resolve_index_run(latest, root)
    if resolved is None:
        fail(item, "index.latest_run resolves to an actual run directory",
             "latest_run=%r under %s" % (latest, root))
        return
    newest = _s1_newest_run(root)
    if newest is not None and os.path.abspath(newest) != resolved:
        fail(item, "index.latest_run references the actual latest run",
             "index -> %s but the newest run is %s (stale index)"
             % (os.path.relpath(resolved, root),
                os.path.relpath(newest, root)))
    else:
        ok(item, "index.latest_run references the actual latest run",
           os.path.relpath(resolved, root))
    # Report/artifact count consistency for the referenced run.
    entry = None
    if isinstance(runs_block, dict):
        for key, value in runs_block.items():
            candidate = _s1_resolve_index_run(key, root)
            if candidate is not None and os.path.abspath(candidate) == resolved:
                entry = value
                break
        if entry is None:
            entry = runs_block.get(os.path.basename(resolved))
    elif isinstance(runs_block, list):
        for value in runs_block:
            candidate = _s1_resolve_index_run(
                value.get("dir") or value.get("path") or value.get("label")
                if isinstance(value, dict) else value, root)
            if candidate is not None and os.path.abspath(candidate) == resolved:
                entry = value
                break
    if isinstance(entry, dict):
        target_summary, _err = load_json(os.path.join(resolved, "summary.json"))
        target_manifest, _err = load_json(os.path.join(
            resolved, "run_manifest.json"))
        count, cpath = _first_key(entry, ("windows", "windows_completed",
                                          "window_count"))
        if is_int(count) and isinstance(target_summary, dict):
            declared = target_summary.get("windows_completed")
            on_disk = None
            try:
                rows, _ = load_jsonl(os.path.join(resolved, "windows.jsonl"))
                on_disk = len(rows) if rows is not None else None
            except OSError:
                on_disk = None
            want = on_disk if on_disk is not None else declared
            if is_int(want) and int(count) != int(want):
                fail(item, "index report count matches the referenced run",
                     "index.%s=%d but the run has %d artifact row(s)"
                     % (cpath, count, want))
            else:
                ok(item, "index report count matches the referenced run",
                   "%s=%r" % (cpath, count))
        run_id = entry.get("run_id")
        if is_nonempty_str(run_id) and isinstance(target_manifest, dict) \
                and target_manifest.get("run_id") != run_id:
            fail(item, "index entry run_id matches the referenced manifest",
                 "index=%r manifest=%r"
                 % (run_id, target_manifest.get("run_id")))
    # The run under verification itself must also be identified consistently.
    if os.path.abspath(resolved) == os.path.abspath(run_dir) \
            and isinstance(manifest, dict) and isinstance(entry, dict):
        if is_nonempty_str(entry.get("run_id")) \
                and entry["run_id"] != manifest.get("run_id"):
            fail(item, "the index entry for this run names its run_id",
                 "index=%r manifest=%r"
                 % (entry["run_id"], manifest.get("run_id")))


def check_s1_provenance(manifest, summary, run_dir, results_dir, prefix, mode):
    """S1-7: one effective config/model mode across all measured rows."""
    item = "%s S1-7 provenance/report consistency" % prefix
    if not isinstance(manifest, dict):
        return
    config, cpath = _first_key(manifest, EFFECTIVE_CONFIG_KEYS)
    config = config if isinstance(config, dict) else {}
    eff_hash = None
    hvalue, _ = _first_key(manifest, EFFECTIVE_HASH_KEYS)
    if is_hex64(hvalue):
        eff_hash = normalize_hash(hvalue)
    mode_value, mode_path = _nested_lookup(
        config, ("sic_model_mode", "model_mode", "method_mode"))
    if not (is_nonempty_str(mode_value)
            and norm_text(mode_value) in S1_MODEL_MODES):
        fail(item, "effective_config declares the S1 model mode",
             "%s=%r (expected %r)" % (mode_path or "effective_config",
                                      mode_value, S1_MODEL_MODE))
    fit, _ = _nested_lookup(config, ("fit_reps", "fit_length_reps"))
    if is_int(fit) and int(fit) != S1_STRICT_FIT_REPS:
        fail(item, "effective fit_reps == %d in the S1 mode"
             % S1_STRICT_FIT_REPS, "got %r" % (fit,))
    eval_reps, eval_path = _nested_lookup(config, ("evaluation_reps",
                                                   "eval_reps",
                                                   "independent_eval_reps"))
    if not (is_int(eval_reps) and int(eval_reps) >= S1_MIN_EVAL_REPS):
        fail(item, "effective evaluation_reps is declared and >= %d"
             % S1_MIN_EVAL_REPS, "%s=%r" % (eval_path or "evaluation_reps",
                                            eval_reps))
    online, online_path = _nested_lookup(config, ("online_validation_reps",
                                                  "validation_reps"))
    if online is not None and not (is_int(online) and int(online) == 0):
        fail(item, "effective online_validation_reps == 0 (S1 default)",
             "%s=%r" % (online_path, online))
    template_id, _ = _nested_lookup(config, ("template_id",))
    input_length, _ = _nested_lookup(config, LAYER_LENGTH_KEYS)
    layers, lpath = _first_key(manifest, IMPL_CONFIG_KEYS)
    layers = layers if isinstance(layers, dict) else {}
    not_run = summary.get("not_run") if isinstance(summary, dict) else []
    missing_rows = []
    for layer in ("effect", "core", "gr", "cir", "full_demod", "native"):
        entry, alias = _s1_layer_entry(layers, layer)
        if entry is None:
            if layer == "native" and isinstance(not_run, list) \
                    and summary_key_mentioned(summary, "native"):
                skip(item, "layer %s is explicitly not_run" % layer,
                     "summary.not_run names native")
                continue
            missing_rows.append(layer)
            continue
        if not isinstance(entry, dict):
            fail(item, "layer %s declares its effective config" % layer,
                 "%s=%r" % (alias, entry))
            continue
        horizon = _layer_identity(entry)
        if eff_hash is not None and horizon[1] is not None \
                and horizon[1] != eff_hash:
            fail(item, "layer %s ran under the effective config hash" % layer,
                 "%s=%s != %s" % (alias, horizon[1], eff_hash))
        layer_mode, layer_mode_path = _s1_layer_mode(entry)
        if layer_mode != mode:
            fail(item, "layer %s runs the declared model mode" % layer,
                 "%s.%s=%r expected %r"
                 % (alias, layer_mode_path, layer_mode, mode))
        if isinstance(horizon[2], str) and is_nonempty_str(template_id) \
                and horizon[2] != str(template_id):
            fail(item, "layer %s uses the effective template" % layer,
                 "%s.template=%r expected %r"
                 % (alias, horizon[2], template_id))
        if is_int(horizon[3]) and is_int(input_length) \
                and int(horizon[3]) != int(input_length):
            fail(item, "layer %s uses the effective input length" % layer,
                 "%s.input_length=%r expected %r"
                 % (alias, horizon[3], input_length))
    if missing_rows:
        fail(item, "effect/core/GR/CIR/full-demod rows are declared",
             "missing %s in %s (or named not_run for native only)"
             % (missing_rows, lpath))
    else:
        ok(item, "effect/core/GR/CIR/full-demod/native rows are declared", lpath)
    # S1-02: perf rows for this run's mode must carry the same model mode.
    for path in _find_perf_tables(results_dir):
        try:
            with open(path, "r", encoding="utf-8", newline="") as handle:
                rows = list(csv.DictReader(handle))
        except OSError:
            continue
        for index, row in enumerate(rows, 2):
            production = _csv_bool(row.get("production_profile"))
            if production is None:
                production = _csv_bool(row.get("official_evidence"))
            verified = _csv_bool(row.get("effect_verified"))
            if not (production or verified):
                continue
            row_mode = row.get("model_mode", row.get("sic_model_mode"))
            row_hash = normalize_hash(row.get("config_hash", ""))
            if eff_hash is not None and is_hex64(row_hash) \
                    and row_hash != eff_hash:
                continue    # a different case/run; not this run's row
            if not is_nonempty_str(row_mode) or norm_text(row_mode) != mode:
                fail(item, "perf row %s carries the run's model mode"
                     % os.path.relpath(path, results_dir),
                     "line %d model_mode=%r expected %r" % (index, row_mode,
                                                            mode))
    _s1_check_index(results_dir, run_dir, manifest, summary, prefix)


def check_s1(run_dir, results_dir, manifest, summary, windows, prefix):
    """Run all S1 checks; fail closed but keep the S0 checks untouched."""
    item = "%s S1 strict-8SYNC/frozen/full-support" % prefix
    if not any(isinstance(w.get("recon"), dict) for w in windows):
        return
    mode, mode_path = s1_mode_of(manifest, summary)
    legacy = _s1_legacy_declared(manifest, summary)
    if legacy is not None:
        skip("%s: explicit legacy method" % item,
             "%s -> S1 evidence not required; S1 claims forbidden" % legacy)
        gates = summary.get("gates") if isinstance(summary, dict) else None
        if isinstance(gates, dict) and gates.get("s1_pass") is True:
            fail("%s: a legacy run does not claim an S1 pass" % item,
                 "gates.s1_pass=true with legacy marker %s" % legacy)
        for window in windows:
            recon = window.get("recon")
            if isinstance(recon, dict) and any(k in recon
                                               for k in S1_EVAL_ACTUAL_KEYS):
                fail("%s: a legacy run does not report S1 region metrics"
                     % item, "window_id=%r reports %s"
                     % (window.get("window_id"),
                        [k for k in S1_EVAL_ACTUAL_KEYS if k in recon]))
                break
        return
    if mode is None:
        fail("%s: the run declares its SIC model mode" % item,
             "expected sic_model_mode=%r at the manifest top level, in "
             "effective_config, or in a sic/s1 block; a SIC-on run that "
             "declares nothing cannot be audited" % S1_MODEL_MODE)
    elif mode not in S1_MODEL_MODES:
        fail("%s: the declared model mode is %r" % (item, S1_MODEL_MODE),
             "got %r via %s" % (mode, mode_path))

    committed = _s1_run_windows(windows)
    with_model = []
    missing_model = []
    for window in committed:
        model, mpath = s1_model_of(window)
        if isinstance(model, dict):
            with_model.append((window, model, mpath))
        else:
            missing_model.append(window.get("window_id"))
    if missing_model:
        fail("%s: committed windows declare the frozen SicModel" % item,
             "%d window(s) lack sic_model, first ids %s"
             % (len(missing_model), missing_model[:5]))
    if not with_model:
        if not committed:
            # A zero-commit (all-bypass) run has no frozen SicModel to audit:
            # the S1 model-evidence family is not applicable, so it is SKIPped.
            # The run is still checked for bypass bit-identity, false-commit
            # and output-change by the negative aggregate (and by the S0/R01
            # per-window checks), which is where those gates live.
            skip("%s: at least one window carries S1 model evidence" % item,
                 "zero committed windows (all bypassed): the S1 model-evidence "
                 "family does not apply")
            for label in ("S1-1 strict-prefix evidence",
                          "S1-2 frozen-model evidence",
                          "S1-3 support semantics", "S1-4 regions"):
                skip("%s %s" % (item, label),
                     "zero committed windows: no frozen model to audit")
        else:
            fail("%s: at least one window carries S1 model evidence" % item,
                 "no sic_model block under %d committed window(s)"
                 % len(committed))
            # Make each S1 evidence family explicit instead of silently
            # skipping it.
            fail("%s S1-1 strict-prefix evidence" % item,
                 "no observed_end_sample/training_lo/hi/"
                 "estimation_ready_sample/estimation_ranges/guard is declared "
                 "on any committed window")
            fail("%s S1-2 frozen-model evidence" % item,
                 "no model_id/fingerprint/frozen params/apply block is declared "
                 "on any committed window")
            fail("%s S1-3 support semantics" % item,
                 "no support_lo/hi/support_source/support_coverage is declared "
                 "on any committed window")
            fail("%s S1-4 regions" % item,
                 "no eval_lo/hi, tail_lo/hi or applied sub range is declared on "
                 "any committed window")
        _s1_check_summary(summary, windows, prefix)
        if manifest is not None:
            _s1_check_seeds(manifest, summary, windows, prefix)
            check_s1_provenance(manifest, summary, run_dir, results_dir,
                                prefix, mode)
        return

    # A declared reconstruction archive policy names the windows whose arrays
    # must be independently recomputed; the remaining windows are explicitly
    # not recomputable.  With no policy the full-array requirement is kept.
    archive_policy, archived_ids = _s1_check_archive_policy(
        manifest, summary, windows, prefix)

    counts = {name: 0 for name in ("prefix", "frozen", "support", "regions",
                                   "actual")}
    per_window = {}
    npz_cache = {}
    for window, model, mpath in with_model:
        wid = window.get("window_id")
        where = "%s window_id=%r" % (item, wid)
        g = _s1_parse_model(model, mpath, where)
        if _s1_check_prefix(window, g, where, mode):
            counts["prefix"] += 1
        if _s1_check_frozen(window, g, where):
            counts["frozen"] += 1
        if _s1_check_support(window, g, where) == 0:
            counts["support"] += 1
        if _s1_check_regions(window, g, where) == 0:
            counts["regions"] += 1
        recon = window.get("recon")
        if archive_policy is not None and str(wid) not in archived_ids:
            # Declared non-archived: no arrays are shipped, so the actual output
            # cannot be independently recomputed here (a declared limitation).
            # The declared metrics are still aggregated by _s1_check_summary.
            per_window[wid] = {"eval": None, "tail": None, "full": None,
                               "eval_na": False, "tail_na": False,
                               "full_na": False}
            continue
        arrays = load_recon_arrays(run_dir, wid, npz_cache)
        result = _s1_check_actual(window, g, recon, arrays, where)
        per_window[wid] = result
        if all(result[k] is not None or result[k + "_na"]
               for k in ("eval", "tail", "full")):
            counts["actual"] += 1
    total = len(with_model)
    for name in ("prefix", "frozen", "support", "regions", "actual"):
        if counts[name] == total:
            ok("%s %s: %d/%d window(s)" % (item, name, counts[name], total),
               "all committed windows pass")
    _s1_check_summary(summary, windows, prefix)
    if manifest is not None:
        _s1_check_seeds(manifest, summary, windows, prefix)
        check_s1_provenance(manifest, summary, run_dir, results_dir, prefix,
                            mode)


# ---------------------------------------------------------------------------
# Driver
# ---------------------------------------------------------------------------


def check_run(run_dir, results_dir, index=None):
    prefix = label_for(run_dir, results_dir)
    for name in REQUIRED_ARTIFACTS:
        if not os.path.isfile(os.path.join(run_dir, name)):
            fail("%s/%s: present" % (prefix, name), "missing required artifact")
    manifest = None
    if os.path.isfile(os.path.join(run_dir, "run_manifest.json")):
        manifest = check_manifest(run_dir, results_dir, prefix)
    summary = None
    if os.path.isfile(os.path.join(run_dir, "summary.json")):
        summary = check_summary(run_dir, manifest, prefix)
    windows, by_id = None, None
    if os.path.isfile(os.path.join(run_dir, "windows.jsonl")):
        windows, by_id = check_windows(run_dir, manifest, summary, prefix)
    if windows is not None:
        sic_mode = (manifest or {}).get("sic_mode")
        if sic_mode is None and isinstance(summary, dict):
            sic_mode = summary.get("sic_mode")
        preset = (manifest or {}).get("preset")
        if preset is None and isinstance(summary, dict):
            preset = summary.get("preset")
        check_baseline_pairing(windows, preset, sic_mode, prefix)
        check_demod_integrity(windows, prefix)
        check_recon_metrics(windows, run_dir, prefix)
        check_raw_target_metrics(run_dir, windows, summary, prefix)
        check_uses_truth(windows, summary, prefix)
        check_truth_sidecar(run_dir, windows, manifest, prefix)
        # R02: one effective config / case / template / input length, and no
        # silent config rewrite.  R06: estimation / online-gate / evaluation
        # region separation and gate data-dependency.
        check_effective_config(manifest, windows, summary, prefix)
        check_window_config_recorded(manifest, windows, summary, prefix)
        check_regions_r02_r06(run_dir, windows, summary, prefix)
        # S1: strict 8-SYNC estimation, frozen model, full-support cancellation,
        # independent-eval/tail actual effect, seed independence, provenance.
        check_s1(run_dir, results_dir, manifest, summary, windows, prefix)
    stats = None
    if os.path.isfile(os.path.join(run_dir, "stage_times.csv")):
        stats = check_stage_times(run_dir, by_id or {}, prefix)
    if summary is not None:
        cross_check_summary(summary, windows, stats, prefix)
        cross_check_effect(summary, windows or [], prefix)
        check_actual_effect_gate(summary, windows or [], prefix)
        check_not_run_declaration(run_dir, summary, prefix)

    # R10: provenance binding, false-vs-null not_run discipline, superseded
    # handling, MATLAB status.  A superseded run is explicitly allowed to skip
    # the provenance hard-failures (but not to claim production evidence).
    superseded = None
    if manifest is not None or summary is not None:
        superseded = check_superseded(run_dir, results_dir, manifest, summary,
                                      index or {}, prefix)
    if manifest is not None:
        prov = None
        for key in PROV_CONTAINER_KEYS:
            if isinstance(manifest.get(key), dict):
                prov = manifest[key]
                break
        prov = prov if isinstance(prov, dict) else {}
        check_provenance(manifest, run_dir, results_dir, prefix,
                         superseded is not None)
        check_production_claims(manifest, summary, prov,
                                superseded is not None, prefix)
    if manifest is not None or summary is not None:
        check_matlab_status(run_dir, manifest, summary, prefix)
        check_not_run_discipline(run_dir, summary, prefix)


def find_run_dirs(results_dir):
    names = tuple(REQUIRED_ARTIFACTS)
    found = []
    for dirpath, dirnames, filenames in os.walk(results_dir):
        if any(n in filenames for n in names):
            found.append(dirpath)
            dirnames[:] = []  # a run directory: do not descend into it
    found.sort()
    return found


def verify_results(results_dir, verbose=False):
    del results[:]
    if not os.path.isdir(results_dir):
        fail("results dir exists", "%s is not a directory" % results_dir)
        return [r for r in results if r[1] == "FAIL"]
    kind = aggregate_kind(results_dir)
    if kind is not None:
        return verify_aggregate(results_dir, kind, verbose)
    runs = find_run_dirs(results_dir)
    if not runs:
        fail("results dir declares at least one run",
             "no run_manifest.json / summary.json / windows.jsonl / "
             "stage_times.csv found under %s (or its subdirectories)"
             % results_dir)
        return [r for r in results if r[1] == "FAIL"]
    ok("results dir declares run(s)", "%d run(s)" % len(runs))
    index = find_superseded_index(results_dir)
    for run_dir in runs:
        check_run(run_dir, results_dir, index)
    check_perf_tables(results_dir)
    check_benchmark_summaries(results_dir, index)
    return [r for r in results if r[1] == "FAIL"]


# ---------------------------------------------------------------------------
# Aggregate roots (sweep ``cells/`` / negative-set ``cases/``)
# ---------------------------------------------------------------------------

# ``sweep`` roots hold ``cells/<cell_id>/``; ``negative`` roots ``cases/<case>/``.
AGGREGATE_SUBDIRS = {"sweep": "cells", "negative": "cases"}
NEGATIVE_REQUIRED_FIELDS = ("windows", "detected", "committed", "false_commit",
                            "output_change", "bypass_bit_identical")
# The frozen actionable-negative bound: a no-actionable-interferer case must be
# measured over at least this many windows.  The per-window ``effect.n_*``
# "evaluable" count is NOT the window count; the count of ``windows.jsonl``
# rows is.
ACTIONABLE_WINDOWS_MIN = 1000


def aggregate_kind(results_dir):
    """Return ``"sweep"``/``"negative"`` when the root aggregates child runs."""
    for kind, sub in AGGREGATE_SUBDIRS.items():
        if os.path.isdir(os.path.join(results_dir, sub)):
            return kind
    return None


def aggregate_children(results_dir, kind):
    """Immediate child run dirs of an aggregate root, sorted by name."""
    sub = os.path.join(results_dir, AGGREGATE_SUBDIRS[kind])
    children = []
    for name in sorted(os.listdir(sub)):
        path = os.path.join(sub, name)
        if os.path.isdir(path):
            children.append(path)
    return children


def wilson_ci(k, n, z=1.96):
    """Wilson score interval, recomputed independently of the generator."""
    if n <= 0:
        return (None, None)
    p = k / n
    d = 1.0 + z * z / n
    c = p + z * z / (2.0 * n)
    half = z * math.sqrt(p * (1.0 - p) / n + z * z / (4.0 * n * n))
    return (max(0.0, (c - half) / d), min(1.0, (c + half) / d))


def _strip_child_prefix(item, prefix):
    if prefix and item.startswith(prefix):
        item = item[len(prefix):].lstrip(": ")
    return item


def _normalize_check(item):
    item = re.sub(r"window_id=[^ ]+", "window_id=*", item)
    return re.sub(r"\s+", " ", item).strip()


def _summary_eff_pass(summary):
    gates = summary.get("gates") if isinstance(summary, dict) else None
    if isinstance(gates, dict) and isinstance(gates.get("eff_pass"), bool):
        return gates["eff_pass"]
    return None


def _summary_median_suppression(summary):
    effect = summary.get("effect") if isinstance(summary, dict) else None
    if isinstance(effect, dict):
        return effect.get("median_suppression_db")
    return None


def _shared_manifest_hashes(manifest):
    """The template hashes plus the shared (non-per-window) input hashes."""
    template = manifest.get("template_hashes")
    fixture = manifest.get("fixture_hashes")
    template = template if isinstance(template, dict) else {}
    shared = {}
    if isinstance(fixture, dict):
        for key, value in fixture.items():
            # ``fixtures/`` and ``recon/`` hold per-window dumps and legitimately
            # differ between cells; every other entry is a shared input file.
            if key.startswith(("fixtures/", "recon/")):
                continue
            shared[key] = value
    return template, shared


def _case_counters(child):
    """Recompute the negative-set counters from the case's ``windows.jsonl``."""
    rows, err = load_jsonl(os.path.join(child, "windows.jsonl"))
    if rows is None:
        return None, err or "windows.jsonl unreadable"
    detected = committed = changed = actionable = 0
    for _lineno, window in rows:
        state = window.get("sic_state")
        state = state if isinstance(state, dict) else {}
        recon = window.get("recon")
        recon = recon if isinstance(recon, dict) else {}
        if state.get("detected") is True:
            detected += 1
        if state.get("committed") is True:
            committed += 1
        if bool(recon.get("output_changed")):
            changed += 1
        overlap = state.get("sub_overlap")
        if is_int(overlap) and int(overlap) > 0:
            actionable += 1
    return {"windows": len(rows), "detected": detected, "committed": committed,
            "output_changed": changed, "actionable_overlap": actionable}, None


def _case_bypass(summary):
    """(bypassed, bit-identical) counts from the case summary (None when absent)."""
    if not isinstance(summary, dict):
        return None, None
    sic = summary.get("sic")
    sic = sic if isinstance(sic, dict) else {}
    block = sic.get("bypass_bit_identical")
    if not isinstance(block, dict):
        return None, None
    return block.get("n_bypassed"), block.get("n_bit_identical")


def check_sweep_table(results_dir, children):
    """Cross-check ``main_matrix.json`` against every ``cells/<id>/`` run."""
    item = "sweep aggregate main_matrix.json"
    data, err = load_json(os.path.join(results_dir, "main_matrix.json"))
    if not isinstance(data, dict):
        fail("%s: present and a JSON object" % item, err or "not an object")
        return
    rows = data.get("cells")
    if not isinstance(rows, list):
        fail("%s: carries a cells[] row list" % item, "got %r" % (rows,))
        return
    child_ids = [os.path.basename(child) for child in children]
    if len(rows) != len(children):
        fail("%s: row count matches the cells/ count" % item,
             "%d row(s) vs %d cell dir(s)" % (len(rows), len(children)))
    else:
        ok("%s: row count matches the cells/ count" % item,
           "%d row(s)" % len(rows))
    by_id = {}
    duplicates = []
    for row in rows:
        cid = row.get("cell_id") if isinstance(row, dict) else None
        if not is_nonempty_str(cid):
            fail("%s: every row declares a cell_id" % item, "row=%r" % (row,))
            continue
        if cid in by_id:
            duplicates.append(cid)
        by_id[cid] = row
    if duplicates:
        fail("%s: cell_id values are unique" % item,
             "duplicate(s) %s" % sorted(set(duplicates))[:8])
    missing = [cid for cid in child_ids if cid not in by_id]
    extra = [cid for cid in by_id if cid not in child_ids]
    if missing:
        fail("%s: every cell dir has a matrix row" % item,
             "missing row(s) for %s" % missing[:8])
    if extra:
        fail("%s: every matrix row names a cells/<id> dir" % item,
             "row(s) without a dir: %s" % extra[:8])
    if not (missing or extra or duplicates):
        ok("%s: rows and cell dirs are 1:1" % item, "%d cell(s)" % len(child_ids))

    mismatches = []
    for child in children:
        cid = os.path.basename(child)
        row = by_id.get(cid)
        summary, _err = load_json(os.path.join(child, "summary.json"))
        if row is None or not isinstance(summary, dict):
            continue
        want_eff = _summary_eff_pass(summary)
        if want_eff is not None and row.get("eff_pass") is not want_eff:
            mismatches.append((cid, "eff_pass", row.get("eff_pass"), want_eff))
        want_med = _summary_median_suppression(summary)
        if is_num(want_med):
            got_med = row.get("median_suppression_db")
            if not (is_num(got_med) and close(float(got_med), float(want_med))):
                mismatches.append(
                    (cid, "median_suppression_db", got_med, want_med))
    if mismatches:
        cid, field, got, want = mismatches[0]
        fail("%s: each row's cell_id/eff_pass/median_suppression_db matches its "
             "cells/<id>/summary.json" % item,
             "%d mismatch(es); first %s: row.%s=%r summary=%r"
             % (len(mismatches), cid, field, got, want))
    else:
        ok("%s: each row's cell_id/eff_pass/median_suppression_db matches its "
           "cells/<id>/summary.json" % item, "%d cell(s)" % len(child_ids))

    hash_groups = {}
    for child in children:
        manifest, _err = load_json(os.path.join(child, "run_manifest.json"))
        if not isinstance(manifest, dict):
            continue
        value, _key = _first_key(manifest, EFFECTIVE_HASH_KEYS)
        hash_groups.setdefault(value, []).append(os.path.basename(child))
    if len(hash_groups) != 1:
        fail("%s: exactly one effective_config_hash across all cells" % item,
             "%d distinct hash(es): %s"
             % (len(hash_groups),
                {str(k): len(v) for k, v in hash_groups.items()}))
    else:
        value = next(iter(hash_groups))
        if not is_hex64(value):
            fail("%s: the effective_config_hash is a sha256 hex string" % item,
                 "got %r" % (value,))
        else:
            ok("%s: exactly one effective_config_hash across all cells" % item,
               "%s (%d cell(s))" % (value, len(hash_groups[value])))

    identity_groups = {}
    for child in children:
        manifest, _err = load_json(os.path.join(child, "run_manifest.json"))
        if not isinstance(manifest, dict):
            continue
        template, shared = _shared_manifest_hashes(manifest)
        identity_groups.setdefault(
            json.dumps([template, shared], sort_keys=True), []).append(
                os.path.basename(child))
    if len(identity_groups) != 1:
        examples = [(names[0], key[:200])
                    for key, names in identity_groups.items()]
        fail("%s: every cell shares the same template/shared-input hashes"
             % item,
             "%d distinct template/input hash set(s); e.g. %s"
             % (len(identity_groups), examples[:2]))
    else:
        ok("%s: every cell shares the same template/shared-input hashes" % item,
           "%d cell(s)" % len(children))


def check_negative_table(results_dir, children):
    """Cross-check ``negative_table.json`` against every ``cases/<case>/`` run."""
    item = "negative aggregate negative_table.json"
    data, err = load_json(os.path.join(results_dir, "negative_table.json"))
    if not isinstance(data, dict):
        fail("%s: present and a JSON object" % item, err or "not an object")
        return
    rows = data.get("cases")
    overall = data.get("overall")
    if not isinstance(rows, list):
        fail("%s: carries a cases[] row list" % item, "got %r" % (rows,))
        return
    if not isinstance(overall, dict):
        fail("%s: carries an overall block" % item, "got %r" % (overall,))
        overall = {}
    child_ids = [os.path.basename(child) for child in children]
    by_id = {}
    duplicates = []
    for row in rows:
        cid = row.get("case") if isinstance(row, dict) else None
        if not is_nonempty_str(cid):
            fail("%s: every row declares a case" % item, "row=%r" % (row,))
            continue
        if cid in by_id:
            duplicates.append(cid)
        by_id[cid] = row
    missing = [cid for cid in child_ids if cid not in by_id]
    extra = [cid for cid in by_id if cid not in child_ids]
    if len(rows) != len(children):
        fail("%s: one row per cases/ dir" % item,
             "%d row(s) vs %d case dir(s)" % (len(rows), len(children)))
    if missing:
        fail("%s: every case dir has a table row" % item,
             "missing row(s) for %s" % missing[:8])
    if extra:
        fail("%s: every table row names a cases/<case> dir" % item,
             "row(s) without a dir: %s" % extra[:8])
    if not (missing or extra or duplicates or len(rows) != len(children)):
        ok("%s: rows and case dirs are 1:1" % item, "%d case(s)" % len(children))

    recomputed = {}
    mismatches = []
    too_few = []
    for child in children:
        cid = os.path.basename(child)
        row = by_id.get(cid)
        counters, cerr = _case_counters(child)
        if counters is None:
            fail("%s: %s windows.jsonl is readable" % (item, cid),
                 cerr or "unreadable")
            continue
        if row is None:
            continue
        summary, _err = load_json(os.path.join(child, "summary.json"))
        bypassed, bit_identical = _case_bypass(summary)
        expected_negative = row.get("expected_negative") is True
        derived = {
            "windows": counters["windows"],
            "detected": counters["detected"],
            "committed": counters["committed"],
            "false_commit": (counters["committed"] if expected_negative
                             else None),
            "output_change": (counters["output_changed"] if expected_negative
                              else None),
            "bypass_bit_identical": bit_identical,
        }
        for field in NEGATIVE_REQUIRED_FIELDS:
            if row.get(field) != derived[field]:
                mismatches.append((cid, field, row.get(field), derived[field]))
        if expected_negative and counters["windows"] < ACTIONABLE_WINDOWS_MIN:
            too_few.append((cid, counters["windows"]))
        recomputed[cid] = dict(counters, expected_negative=expected_negative,
                               bypassed=bypassed, bit_identical=bit_identical)
    if mismatches:
        cid, field, got, want = mismatches[0]
        fail("%s: each row's windows/detected/committed/false_commit/"
             "output_change/bypass_bit_identical matches its case artifacts"
             % item,
             "%d mismatch(es); first %s: table.%s=%r recomputed=%r"
             % (len(mismatches), cid, field, got, want))
    else:
        ok("%s: each row's windows/detected/committed/false_commit/"
           "output_change/bypass_bit_identical matches its case artifacts"
           % item, "%d case(s)" % len(child_ids))
    if too_few:
        fail("%s: every actionable (expected_negative) case has >= %d windows"
             % (item, ACTIONABLE_WINDOWS_MIN),
             "%d case(s) below the bound (windows.jsonl row count): %s"
             % (len(too_few), too_few[:8]))
    else:
        actionable = [cid for cid, info in recomputed.items()
                      if info["expected_negative"]]
        ok("%s: every actionable (expected_negative) case has >= %d windows"
           % (item, ACTIONABLE_WINDOWS_MIN),
           "%d actionable case(s)" % len(actionable))

    # Recompute the overall counters/rates/CI from the per-case counts.
    negative_ids = [cid for cid, info in recomputed.items()
                    if info["expected_negative"]]
    neg_n = sum(recomputed[cid]["windows"] for cid in negative_ids)
    fp = sum(recomputed[cid]["detected"] for cid in negative_ids)
    fn = sum(recomputed[cid]["committed"] for cid in negative_ids)
    fo = sum(recomputed[cid]["output_changed"] for cid in negative_ids)
    fa = sum(recomputed[cid]["actionable_overlap"] for cid in negative_ids)

    def rate(count):
        return (count / neg_n) if neg_n else None

    def pair_equal(a, b):
        if not (isinstance(a, list) and isinstance(b, list)
                and len(a) == len(b) == 2):
            return False
        for x, y in zip(a, b):
            if x is None or y is None:
                if not (x is None and y is None):
                    return False
            elif not close(x, y):
                return False
        return True

    overall_expected = {
        "negative_windows": neg_n,
        "false_detect": fp,
        "false_actionable_detect": fa,
        "false_commit": fn,
        "false_output_change": fo,
        "false_detect_rate": rate(fp),
        "false_actionable_detect_rate": rate(fa),
        "false_commit_rate": rate(fn),
        "false_output_change_rate": rate(fo),
    }
    overall_bad = []
    for field, want in overall_expected.items():
        got = overall.get(field)
        if want is None:
            if got is not None:
                overall_bad.append((field, got, want))
        elif not (is_num(got) and close(got, want)):
            overall_bad.append((field, got, want))
    ci_expected = {
        "false_detect_ci95": list(wilson_ci(fp, neg_n)),
        "false_actionable_detect_ci95": list(wilson_ci(fa, neg_n)),
        "false_commit_ci95": list(wilson_ci(fn, neg_n)),
        "false_output_change_ci95": list(wilson_ci(fo, neg_n)),
    }
    for field, want in ci_expected.items():
        if not pair_equal(overall.get(field), want):
            overall_bad.append((field, overall.get(field), want))
    if overall_bad:
        field, got, want = overall_bad[0]
        fail("%s: overall counters/rates/Wilson CI recomputed from the per-case "
             "counts" % item,
             "%d mismatch(es); first %s: declared=%r recomputed=%r"
             % (len(overall_bad), field, got, want))
    else:
        ok("%s: overall counters/rates/Wilson CI recomputed from the per-case "
           "counts" % item,
           "negative_windows=%d false_commit=%d" % (neg_n, fn))

    declared_neg = overall.get("expected_negative_cases")
    if not (isinstance(declared_neg, list)
            and sorted(str(c) for c in declared_neg) == sorted(negative_ids)):
        fail("%s: expected_negative_cases names exactly the actionable rows"
             % item,
             "declared=%r actionable=%r" % (declared_neg, sorted(negative_ids)))
    else:
        ok("%s: expected_negative_cases names exactly the actionable rows"
           % item, "%s" % sorted(negative_ids))

    bypass_n = sum(int(recomputed[cid]["bypassed"] or 0)
                   for cid in recomputed)
    bypass_ok = sum(int(recomputed[cid]["bit_identical"] or 0)
                    for cid in recomputed)
    bypass_bad = []
    if overall.get("bypass_windows") != bypass_n:
        bypass_bad.append(("bypass_windows", overall.get("bypass_windows"),
                           bypass_n))
    if overall.get("bypass_bit_identical") != bypass_ok:
        bypass_bad.append(("bypass_bit_identical",
                           overall.get("bypass_bit_identical"), bypass_ok))
    want_all = bool(bypass_n == 0 or bypass_n == bypass_ok)
    if overall.get("bypass_all_identical") is not want_all:
        bypass_bad.append(("bypass_all_identical",
                           overall.get("bypass_all_identical"), want_all))
    if bypass_bad:
        field, got, want = bypass_bad[0]
        fail("%s: overall bypass counts recomputed from the per-case summaries"
             % item,
             "%d mismatch(es); first %s: declared=%r recomputed=%r"
             % (len(bypass_bad), field, got, want))
    else:
        ok("%s: overall bypass counts recomputed from the per-case summaries"
           % item, "bypassed=%d bit_identical=%d" % (bypass_n, bypass_ok))

    want_gate = bool(neg_n >= ACTIONABLE_WINDOWS_MIN and neg_n > 0
                     and fn / neg_n <= 0.01)
    want_gate_change = bool(neg_n >= ACTIONABLE_WINDOWS_MIN and neg_n > 0
                            and fo / neg_n <= 0.01)
    gate_bad = []
    if overall.get("gate_pass") is not want_gate:
        gate_bad.append(("gate_pass", overall.get("gate_pass"), want_gate))
    if overall.get("gate_pass_false_commit_or_change") is not want_gate_change:
        gate_bad.append(("gate_pass_false_commit_or_change",
                         overall.get("gate_pass_false_commit_or_change"),
                         want_gate_change))
    if gate_bad:
        field, got, want = gate_bad[0]
        fail("%s: the overall gate flags follow the recomputed counts" % item,
             "%d mismatch(es); first %s: declared=%r recomputed=%r"
             % (len(gate_bad), field, got, want))
    else:
        ok("%s: the overall gate flags follow the recomputed counts" % item,
           "gate_pass=%s" % want_gate)


def check_aggregate_index(results_dir, kind, children):
    """Validate the aggregate's own index (``cells/index.json`` etc.) once.

    The single-run ``S1-7`` index check is a results-root artifact; a child of
    an aggregate root must not resolve a sibling/global index (that produced
    one spurious failure per child).  The index the aggregate runner writes
    inside ``cells/``/``cases/`` is validated here, once, against every child.
    """
    item = "%s aggregate index" % kind
    sub = AGGREGATE_SUBDIRS[kind]
    candidates = [os.path.join(results_dir, name) for name in S1_INDEX_FILES]
    candidates += [os.path.join(results_dir, sub, name)
                   for name in S1_INDEX_FILES]
    path = next((p for p in candidates if os.path.isfile(p)), None)
    if path is None:
        skip("%s: present" % item,
             "no index.json under %s or %s/" % (results_dir, sub))
        return
    data, err = load_json(path)
    if not isinstance(data, dict):
        fail("%s: is a JSON object" % item, err or "not an object")
        return
    child_ids = [os.path.basename(child) for child in children]
    latest = data.get("latest_run", data.get("latest"))
    runs_block = data.get("runs")
    if latest is None and isinstance(runs_block, dict):
        latest = runs_block.get("latest")
    if latest not in child_ids:
        fail("%s: latest_run resolves to an actual child run" % item,
             "latest_run=%r under %s" % (latest, sub))
    else:
        ok("%s: latest_run resolves to an actual child run" % item,
           "%s" % latest)
    mismatches = []
    missing = []
    for child in children:
        cid = os.path.basename(child)
        entry = runs_block.get(cid) if isinstance(runs_block, dict) else None
        if not isinstance(entry, dict):
            missing.append(cid)
            continue
        rows, _err = load_jsonl(os.path.join(child, "windows.jsonl"))
        count = len(rows) if rows is not None else None
        declared, _key = _first_key(entry, ("windows", "windows_completed",
                                            "window_count"))
        if count is not None and is_int(declared) and int(declared) != count:
            mismatches.append((cid, "windows", declared, count))
        manifest, _err = load_json(os.path.join(child, "run_manifest.json"))
        if isinstance(manifest, dict) and is_nonempty_str(entry.get("run_id")) \
                and entry["run_id"] != manifest.get("run_id"):
            mismatches.append((cid, "run_id", entry.get("run_id"),
                               manifest.get("run_id")))
    if missing:
        fail("%s: every child has an index entry" % item,
             "missing entry for %s" % missing[:8])
    if mismatches:
        cid, field, got, want = mismatches[0]
        fail("%s: each index entry's count/run_id matches the child artifact"
             % item,
             "%d mismatch(es); first %s: index.%s=%r child=%r"
             % (len(mismatches), cid, field, got, want))
    else:
        ok("%s: each index entry's count/run_id matches the child artifact"
           % item, "%d child(ren)" % len(child_ids))


def print_aggregate_summary(results_dir, kind, per_child):
    """One line per child (first failing check), no per-window spam."""
    print("-" * 72)
    print("aggregate summary: %s (%s, %d child run dir(s))"
          % (results_dir, kind, len(per_child)))
    print("  %-44s %-4s %7s %8s  %s"
          % ("child", "stat", "checks", "fails", "first failing check"))
    groups = {}
    n_pass = 0
    for child, checks, fails in per_child:
        name = os.path.basename(child)
        if fails:
            prefix = label_for(child, results_dir)
            first = _strip_child_prefix(fails[0][0], prefix)
            print("  %-44s %-4s %7d %8d  %s"
                  % (name, "FAIL", checks, len(fails), first))
            groups.setdefault(_normalize_check(first), []).append(name)
        else:
            n_pass += 1
            print("  %-44s %-4s %7d %8d"
                  % (name, "PASS", checks, len(fails)))
    print("  children: %d PASS / %d FAIL; checks %d, failures %d"
          % (n_pass, len(per_child) - n_pass,
             sum(checks for _c, checks, _f in per_child),
             sum(len(fails) for _c, _checks, fails in per_child)))
    if groups:
        print("  distinct first-failure groups:")
        for key, names in sorted(groups.items(), key=lambda kv: -len(kv[1])):
            print("    %3d child(ren): %s" % (len(names), key))
            if len(names) <= 6:
                print("        %s" % ", ".join(names))


def verify_aggregate(results_dir, kind, verbose=False):
    """Verify every child run dir and cross-check the aggregate tables."""
    global _AGGREGATE_CHILD, _QUIET
    children = aggregate_children(results_dir, kind)
    ok("aggregate results root", "%s: %d child run dir(s) under %s/"
       % (kind, len(children), AGGREGATE_SUBDIRS[kind]))
    if not children:
        fail("aggregate results root declares at least one child run",
             "no run dir under %s/" % AGGREGATE_SUBDIRS[kind])
        return [r for r in results if r[1] == "FAIL"]
    index = find_superseded_index(results_dir)
    quiet = _QUIET
    per_child = []
    try:
        for child in children:
            start = len(results)
            _AGGREGATE_CHILD = True
            _QUIET = quiet or not verbose
            check_run(child, results_dir, index)
            _QUIET = quiet
            _AGGREGATE_CHILD = False
            per_child.append((child, len(results) - start,
                              [r for r in results[start:] if r[1] == "FAIL"]))
    finally:
        _AGGREGATE_CHILD = False
        _QUIET = quiet
    if kind == "sweep":
        check_sweep_table(results_dir, children)
    else:
        check_negative_table(results_dir, children)
    check_aggregate_index(results_dir, kind, children)
    print_aggregate_summary(results_dir, kind, per_child)
    check_perf_tables(results_dir)
    check_benchmark_summaries(results_dir, index)
    return [r for r in results if r[1] == "FAIL"]


# ---------------------------------------------------------------------------
# Self-test fixtures
# ---------------------------------------------------------------------------


def build_good_set(root):
    """Create a tiny, fully consistent artifact set that MUST pass."""
    run_dir = os.path.join(root, "main")
    os.makedirs(os.path.join(run_dir, "templates"), exist_ok=True)
    os.makedirs(os.path.join(run_dir, "recon"), exist_ok=True)
    os.makedirs(os.path.join(run_dir, "fixtures"), exist_ok=True)
    os.makedirs(os.path.join(run_dir, "lib"), exist_ok=True)

    # R10 provenance: a real commit, a dirty-patch identifier, and the actual
    # loaded-library / benchmark binary files whose hashes are re-derived.
    fake_commit = "a1b2c3d4e5f6a7b8c9d0e1f2a3b4c5d6e7f8a9b0"
    lib_file = os.path.join(run_dir, "lib", "libgnuradio-uwb.so.1.0.0")
    with open(lib_file, "wb") as handle:
        handle.write(b"fake gnuradio-uwb shared object\n")
    lib_sha = sha256_file(lib_file)
    bench_file = os.path.join(run_dir, "benchmark_preamble_sic")
    with open(bench_file, "wb") as handle:
        handle.write(b"fake benchmark_preamble_sic executable\n")
    bench_sha = sha256_file(bench_file)
    bench_src_sha = hashlib.sha256(b"fake benchmark source\n").hexdigest()

    n_windows = 5
    sample_count = 190912
    # S1 geometry: training is the first 8 SYNC [origin, origin+8*sps); the
    # independent evaluation is SYNC [8,64); the tail-eval is the last 4 SYNC.
    # online_validation_reps == 0 (the S1 default): the gate runs on training.
    fit_reps = 8
    validation_reps = 0
    evaluation_reps = 56
    sps = SYNC_SAMPLES_PER_SYMBOL
    sync_reps = 64
    origin = 1000
    fit_len = fit_reps * sps                                      # 8128
    est_lo, est_hi = origin, origin + fit_len                     # [1000,9128)
    on_lo, on_hi = est_hi, est_hi                                 # empty (0 reps)
    online_len = 0
    eval_lo, eval_hi = est_hi, origin + sync_reps * sps           # [9128,66024)
    tail_lo, tail_hi = origin + (sync_reps - 4) * sps, eval_hi    # [61960,66024)
    hold_lo, hold_hi = 20000, 24000     # inside eval, disjoint from training
    ov_lo, ov_hi = origin, origin + sync_reps * sps               # truth overlap
    support_pre_tail, support_post_tail = 0, 64
    support_lo = origin - support_pre_tail
    support_hi = origin + sync_reps * sps + support_post_tail     # 66088
    sub_lo, sub_hi = support_lo, support_hi
    array_len = 70000
    model_seed = 1000
    cir_len = 256
    target_lo, target_hi = 85, 96
    target_tap = 90

    # config + template files (referenced and hashed by the manifest)
    config_path = os.path.join(run_dir, "config.json")
    with open(config_path, "w", encoding="utf-8") as handle:
        json.dump({"preset": "main", "generator": {"sensing_code": 9,
                                                   "comm_code": 10}}, handle)
    template_path = os.path.join(run_dir, "templates", "comm_code10.f32")
    with open(template_path, "wb") as handle:
        handle.write(bytes(range(256)) * 4)
    template_sha = sha256_file(template_path)
    template_size = os.path.getsize(template_path)

    windows = []
    fixture_hashes = {}
    stage_rows = []
    truth_rows = []

    rng = np.random.default_rng(1234)
    for wid in range(n_windows):
        # Reconstruction NMSE targets (region-dependent: eval and holdout are
        # independent, and holdout is nested inside eval but overrides it).
        recon_sup = 12.0 + 0.5 * wid
        recon_nmse = 10.0 ** (-recon_sup / 10.0)
        hold_sup = recon_sup - 1.0
        hold_nmse = 10.0 ** (-hold_sup / 10.0)
        i = (rng.standard_normal(array_len) + 1j * rng.standard_normal(array_len))
        i_hat = i.copy()
        for region_lo, region_hi, target_nmse in (
                (eval_lo, eval_hi, recon_nmse),
                (hold_lo, hold_hi, hold_nmse)):
            ref = i[region_lo:region_hi]
            err = (rng.standard_normal(region_hi - region_lo)
                   + 1j * rng.standard_normal(region_hi - region_lo))
            den = float(np.vdot(ref, ref).real)
            scale = math.sqrt(target_nmse * den
                              / float(np.vdot(err, err).real))
            i_hat[region_lo:region_hi] = ref + err * scale

        # Distinct actual-output suppression per region so a copy from one
        # region into another is caught by the independent recompute.
        actual_full_sup = 23.0 - 0.4 * wid
        actual_eval_sup = 21.0 - 0.3 * wid
        actual_tail_sup = 20.0 - 0.2 * wid
        base = 0.25 * (rng.standard_normal(array_len)
                       + 1j * rng.standard_normal(array_len))
        y = base + i
        y_clean = y.copy()
        residual = np.zeros(array_len, dtype=np.complex128)

        def _fill_region(target_sup, lo, hi):
            noise = (rng.standard_normal(hi - lo)
                     + 1j * rng.standard_normal(hi - lo))
            den = float(np.vdot(i[lo:hi], i[lo:hi]).real)
            nmse = 10.0 ** (-target_sup / 10.0)
            scale = math.sqrt(nmse * den / float(np.vdot(noise, noise).real))
            residual[lo:hi] = noise * scale

        _fill_region(actual_full_sup, sub_lo, sub_hi)      # full support
        _fill_region(actual_eval_sup, eval_lo, eval_hi)    # independent eval
        _fill_region(actual_tail_sup, tail_lo, tail_hi)    # tail-4-SYNC
        # y_clean = y - i + residual over the applied support (the SIC removed
        # the true interference and left `residual`).
        y_clean[sub_lo:sub_hi] = base[sub_lo:sub_hi] + residual[sub_lo:sub_hi]
        y_c64 = y.astype(np.complex64)
        yc_c64 = y_clean.astype(np.complex64)
        i_c64 = i.astype(np.complex64)
        i_hat_c64 = i_hat.astype(np.complex64)
        npz_path = os.path.join(run_dir, "recon", "window_%d.npz" % wid)

        # Declare every recomputed quantity from the *cast* arrays so it is
        # exactly the quantity the verifier re-derives.
        def _declared_nmse(lo, hi):
            ref = i_c64[lo:hi].astype(np.complex128)
            est = i_hat_c64[lo:hi].astype(np.complex128)
            den = float(np.vdot(ref, ref).real)
            if den <= 0:
                return 0.0
            return float(np.vdot(est - ref, est - ref).real) / den

        def _declared_actual(lo, hi):
            ref = i_c64[lo:hi].astype(np.complex128)
            delta = (y_c64[lo:hi].astype(np.complex128)
                     - yc_c64[lo:hi].astype(np.complex128))
            resid = ref - delta
            den = float(np.vdot(ref, ref).real)
            num = float(np.vdot(resid, resid).real)
            if den <= 0:
                return None, None
            nmse = num / den
            return nmse, (-10.0 * math.log10(nmse) if num > 0 else 60.0)

        nmse = _declared_nmse(eval_lo, eval_hi)
        suppression = -10.0 * math.log10(nmse) if nmse > 0 else 60.0
        hold_nmse = _declared_nmse(hold_lo, hold_hi)
        hold_sup = -10.0 * math.log10(hold_nmse) if hold_nmse > 0 else 60.0
        declared_actual_nmse, declared_actual_sup = _declared_actual(ov_lo, ov_hi)
        _eval_actual_nmse, eval_actual_sup = _declared_actual(eval_lo, eval_hi)
        _tail_actual_nmse, tail_actual_sup = _declared_actual(tail_lo, tail_hi)
        # R04 raw (non-normalized) CIRs with a fixed weak-target ROI.
        cir_clean = 0.001 * (rng.standard_normal(cir_len)
                             + 1j * rng.standard_normal(cir_len))
        cir_clean[40] = 1.0 + 0.0j             # strong main peak
        cir_clean[target_tap] = 0.10 + 0.02j   # weak target inside the ROI
        cir_sic = cir_clean * (10.0 ** (-0.04 / 20.0))   # ~-0.04 dB, shape kept
        cir_clean_c64 = cir_clean.astype(np.complex64)
        cir_sic_c64 = cir_sic.astype(np.complex64)
        _t = target_lo + int(np.argmax(np.abs(cir_clean_c64[target_lo:target_hi])))
        _amp_err = 20.0 * math.log10(abs(complex(cir_sic_c64[_t]))
                                     / abs(complex(cir_clean_c64[_t])))
        _dphi = math.atan2(cir_sic_c64[_t].imag, cir_sic_c64[_t].real) \
            - math.atan2(cir_clean_c64[_t].imag, cir_clean_c64[_t].real)
        _dphi = (_dphi + math.pi) % (2.0 * math.pi) - math.pi
        _phase_err = math.degrees(_dphi)
        _peak_c = target_lo + int(np.argmax(np.abs(cir_clean_c64[target_lo:target_hi])))
        _peak_s = target_lo + int(np.argmax(np.abs(cir_sic_c64[target_lo:target_hi])))
        _peak_shift = _peak_s - _peak_c
        np.savez(npz_path, i=i_c64, i_hat=i_hat_c64,
                 y=y_c64, y_clean=yc_c64,
                 cir_raw_clean=cir_clean_c64, cir_raw_sic=cir_sic_c64)
        fixture_hashes["recon/window_%d.npz" % wid] = sha256_file(npz_path)

        def consumer(tag, status, fcs, psdu_ok, psdu_hex, tap, amp):
            return {
                "status": status,
                "cir_status": "ok" if status in SUCCESS_STATUSES else "failed",
                "fcs_pass": fcs, "psdu_ok": psdu_ok, "psdu_hex": psdu_hex,
                "cir_peak_tap": tap, "target_amp": amp,
                "input_sha256": hashlib.sha256(
                    ("%s-%d" % (tag, wid)).encode("utf-8")).hexdigest(),
                "input_samples": sample_count,
            }

        clean = consumer("clean", "success", True, True, CANONICAL_PSDU_HEX,
                         18, [1.0, 0.0])
        raw = consumer("raw", "fcs_failed", False, False, None, 19, [1.05, 0.0])
        sic = consumer("sic", "success", True, True, CANONICAL_PSDU_HEX, 18,
                       [0.99, 0.0])
        windows.append({
            "schema_version": 1, "run_id": "run-main-1", "window_id": wid,
            "epoch_id": 0, "sample_rate": WORK_HZ,
            "sample_count": sample_count, "window_start_sample": 0,
            "predicted_sensing_start": 1997,
            "sensing_profile_id": "code9-64sync-4z2",
            "sensing_code": 9, "sensing_sync_reps": 64,
            "sensing_sfd_mode": "4z2", "template_id": "comm_code10",
            "overlap": {"lo": ov_lo, "hi": ov_hi, "samples": ov_hi - ov_lo},
            "isr_db": 10.0, "snr_db": 30.0, "comm_code": 10, "sic_k": 4,
            "fit_reps": fit_reps, "validation_reps": validation_reps,
            "evaluation_reps": evaluation_reps,
            "fit_reps_source": "preset", "validation_reps_source": "preset",
            "roi": [0, 1024],
            "demod_softchip_mode": "bypass",
            "clean": clean, "raw": raw, "sic": sic,
            "sic_state": {"detected": True, "committed": True, "partial": False,
                          "ambiguous": False, "bypass_reason": "",
                          "uses_truth": False},
            "admission": {"published_ns": 1000, "admitted_ns": 1100,
                          "completion_ns": 2000, "missed_admission": False},
            "recon": {"nmse": nmse, "suppression_db": suppression,
                      "eval_lo": eval_lo, "eval_hi": eval_hi,
                      "holdout_nmse": hold_nmse,
                      "holdout_suppression_db": hold_sup,
                      "holdout_lo": hold_lo, "holdout_hi": hold_hi,
                      # R01 actual-output cancellation (main gate) + the S1
                      # independent-eval / tail-4-SYNC actual metrics.
                      "actual_nmse": declared_actual_nmse,
                      "actual_suppression_db": declared_actual_sup,
                      "full_overlap_actual_suppression_db": declared_actual_sup,
                      "eval_actual_suppression_db": eval_actual_sup,
                      "tail_actual_suppression_db": tail_actual_sup,
                      "sub_lo": ov_lo, "sub_hi": ov_hi, "coverage": 1.0,
                      "uses_truth": False},
            # R02/R06 region + named-quantity block (S1 geometry).
            "regions": {
                "fit_len": fit_len,
                "estimation_lo": est_lo, "estimation_hi": est_hi,
                "online_validation_len": online_len,
                "online_validation_lo": on_lo, "online_validation_hi": on_hi,
                "evaluation_reps": evaluation_reps,
                "eval_len": eval_hi - eval_lo,
                "eval_lo": eval_lo, "eval_hi": eval_hi,
                "signal_start": ov_lo, "signal_stop": ov_hi,
                "gate_lo": est_lo, "gate_hi": est_hi,
                "gate_keys": ["fit_residual", "cond2"],
                "search_roi": [0, 1024],
                "subtraction_roi": [sub_lo, sub_hi],
                "training_lo": est_lo, "training_hi": est_hi,
                "tail_eval_lo": tail_lo, "tail_eval_hi": tail_hi,
            },
            # S1: the frozen model estimated from the first 8 SYNC.
            "sic_model": {
                "mode": S1_MODEL_MODE,
                "model_id": "model-w%d" % wid,
                "fingerprint": hashlib.sha256(
                    ("model-%d" % wid).encode("utf-8")).hexdigest(),
                "fit_reps": fit_reps,
                "strict_8sync": True,
                "guard": {"pre": 0, "post": 0},
                "origin": origin,
                "cfo_hz": 0.0,
                "n_paths": 1,
                "tau": [0.0],
                "gain": [[1.0, 0.0]],
                "sample_rate": WORK_HZ,
                "samples_per_symbol": sps,
                "nominal_sync_reps": sync_reps,
                "training_lo": est_lo, "training_hi": est_hi,
                "observed_end_sample": est_hi,
                "estimation_ready_sample": est_hi + 272,
                "estimation_ranges": {
                    "training": [est_lo, est_hi],
                    "cfo": [est_lo, est_hi],
                    "cir": [est_lo, est_hi],
                },
                "support_lo": support_lo, "support_hi": support_hi,
                "support_source": "configured_nominal_prediction",
                "support_pre_tail": support_pre_tail,
                "support_post_tail": support_post_tail,
                "support_coverage": 1.0,
                "sub_lo": sub_lo, "sub_hi": sub_hi,
                "phase_reference": est_lo,
                "eval_lo": eval_lo, "eval_hi": eval_hi,
                "tail_lo": tail_lo, "tail_hi": tail_hi,
                "fit_residual": 1.0e-3,
                "cond2": 2.5,
                "status": "ok",
                "apply": {
                    "model_id": "model-w%d" % wid,
                    "fingerprint": hashlib.sha256(
                        ("model-%d" % wid).encode("utf-8")).hexdigest(),
                    "refit_count": 0,
                    "param_deltas": {},
                    "params": {
                        "origin": origin, "cfo_hz": 0.0, "n_paths": 1,
                        "tau": [0.0], "gain": [[1.0, 0.0]],
                        "support_lo": support_lo, "support_hi": support_hi,
                        "sub_lo": sub_lo, "sub_hi": sub_hi,
                        "phase_reference": est_lo,
                    },
                },
            },
            # S1: the formal acceptance window identity (seed provenance).
            "generator": {
                "seed_set": "acceptance",
                "seed": 2000000 + wid,
                "seed_offset": wid,
                "noise_root": "noise-acc-%d" % wid,
            },
            "target": {"target_lo": target_lo, "target_hi": target_hi,
                       "target_amp_err_db": _amp_err,
                       "target_phase_err_deg": _phase_err,
                       "target_peak_shift": _peak_shift,
                       "cir_raw_target_amp_db": 20.0 * math.log10(
                           abs(complex(cir_sic_c64[_t]))),
                       "weak_target_detected": True,
                       "false_peak_count": 0, "ref": "clean"},
        })
        truth_rows.append({
            "run_id": "run-main-1", "window_id": wid, "seed": 1000 + wid,
            "sensing_code": 9, "comm_code": 10,
            "comm": {"start_sample_exact": 1997.0, "cfo_hz": 0.0,
                     "paths": [{"delay_samples": 0.0, "gain": [1.0, 0.0]}],
                     "isr_db": 10.0, "isr_db_target": 10.0, "scale": math.sqrt(10.0),
                     "real_overlap_lo": ov_lo, "real_overlap_hi": ov_hi,
                     "overlap_samples": ov_hi - ov_lo, "length_samples": 65072,
                     "overlap_energy_clean": 1000.0,
                     "overlap_energy_comm_prescale": 1000.0,
                     "frac_model": "pchip"},
            "sensing": {"paths": [{"delay_samples": 0.0, "gain": [1.0, 0.0]},
                                  {"delay_samples": 37.0, "gain": [0.25, 0.0]}]},
            "hashes": {"template_id": "comm_code10",
                       "template_sha256": template_sha},
        })

        # stage_times rows (rev 2: 8 columns with an explicit scope)
        demod = {"timing": 100, "timing_coarse": 40, "timing_fine_track": 60,
                 "cfo": 50, "sfd": 100, "sfd_bootstrap": 20,
                 "sfd_tail_retrack": 20, "sfd_cfo_refit": 20, "sfd_final": 20,
                 "sfd_anchor_other": 10, "cir": 80, "cir_estimate": 30,
                 "soft_fir": 30, "postprocess": 10, "ns_sfd": 20, "phr": 40,
                 "payload": 30, "total": 1000}
        sic_demod = {"detect_coarse": 40, "detect_fine": 60, "cfo": 25,
                     "cir": 70, "path_select": 20, "fractional_refine": 30,
                     "matrix_build": 15, "gain_fit": 35, "reconstruct": 45,
                     "validate_commit": 10,
                     # S1 §8 cost split.
                     "acquisition": 30, "estimate8": 60, "model_prep": 20,
                     "backtrack": 25, "apply": 40,
                     "core_total": 600}
        for consumer_name in ("clean", "raw", "sic"):
            for stage, us in demod.items():
                stage_rows.append(("run-main-1", wid, consumer_name, "demod",
                                   "cxx", stage, us, "demod"))
            if consumer_name == "sic":
                for stage, us in sic_demod.items():
                    stage_rows.append(("run-main-1", wid, consumer_name, "sic",
                                       "cxx", stage, us, "sic"))
        # A huge radar `cir` and adapter/outer rows must NOT pollute the demod
        # total (rev 2 adjudication 1).
        stage_rows.append(("run-main-1", wid, "raw", "radar", "cxx", "cir",
                           5000, "radar_cir_only"))
        stage_rows.append(("run-main-1", wid, "clean", "adapter", "adapter",
                           "handler", 7, "adapter"))
        stage_rows.append(("run-main-1", wid, "clean", "outer", "outer",
                           "paced_wait", 11, "outer"))

    with open(os.path.join(run_dir, "windows.jsonl"), "w", encoding="utf-8") as handle:
        for window in windows:
            handle.write(json.dumps(window) + "\n")
    with open(os.path.join(run_dir, "fixtures", "truth.jsonl"), "w",
              encoding="utf-8") as handle:
        for record in truth_rows:
            handle.write(json.dumps(record) + "\n")
    with open(os.path.join(run_dir, "stage_times.csv"), "w", encoding="utf-8",
              newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["run_id", "window_id", "consumer", "scope", "tier",
                         "stage", "us", "detail"])
        writer.writerows(stage_rows)

    total_values = [1000.0] * n_windows
    def total_stats():
        return {"mean": sum(total_values) / len(total_values),
                "p50": percentile(total_values, 50),
                "p95": percentile(total_values, 95),
                "p99": percentile(total_values, 99),
                "max": max(total_values)}
    effect = effect_aggregates(windows, include_truth=False)
    effect["oracle_windows"] = 0
    effect["oracle_excluded"] = True
    effect["n_windows"] = n_windows
    effect["n_interference_windows"] = n_windows
    effect["n_na_windows"] = 0
    # S1: the independent-eval and tail-4-SYNC actual medians/counts.
    effect["median_eval_actual_suppression_db"] = median(
        [w["recon"]["eval_actual_suppression_db"] for w in windows])
    effect["median_tail_actual_suppression_db"] = median(
        [w["recon"]["tail_actual_suppression_db"] for w in windows])
    effect["n_eval_windows"] = n_windows
    effect["n_eval_na_windows"] = 0
    effect["n_tail_windows"] = n_windows
    effect["n_tail_na_windows"] = 0
    summary = {
        "run_id": "run-main-1", "preset": "main", "input_grid": "work",
        "sic_mode": "estimate", "consumer": "sic",
        "windows_expected": n_windows, "windows_completed": n_windows,
        "clean": {"completed": n_windows, "total": total_stats()},
        "raw": {"completed": n_windows, "total": total_stats()},
        "sic": {"completed": n_windows, "total": total_stats()},
        "effect": effect,
        "demod": {"clean_ok": n_windows, "raw_ok": 0, "sic_ok": n_windows,
                  "raw_ok_sic_fail": 0, "raw_fail_sic_ok": n_windows,
                  "failing_stages": ["raw"]},
        "negative": {"false_detect": 0, "false_commit": 0.0, "n_windows": 0,
                     "ci95": [0.0, 0.0]},
        "runtime": {"offered": n_windows, "admitted": n_windows,
                    "completed": n_windows, "failed": 0, "bypassed": 0,
                    "dropped": 0, "deadline_misses": 0, "inflight_hw": 1,
                    "queue_hw": 1, "rss_bytes": 1048576, "cpu_seconds": 1.5},
        "gates": {"eff_pass": True, "sic_speed_pass": True,
                  "demod_speed_pass": True, "native_pass": None},
        "not_run": [
            "native cg400 roundtrip",
            "gates.native_pass (untested; native front-end not run)",
            "matlab cross-check",
            "oracle diagnostic",
            "negative window set",
        ],
        "declared_items": {
            "native_cg400_roundtrip": "not_run",
            "gates.native_pass": "not_run",
            "matlab_cross_check": "not_run",
            "oracle_diagnostic": "not_run",
        },
    }
    with open(os.path.join(run_dir, "summary.json"), "w", encoding="utf-8") as handle:
        json.dump(summary, handle, indent=1)

    repo_fixture = os.path.join(REPO_ROOT, "AGENTS.md")
    if os.path.isfile(repo_fixture):
        fixture_hashes["repo:AGENTS.md"] = sha256_file(repo_fixture)

    # R02: the effective runtime config actually used, its hash, and the
    # per-implementation declaration of the SAME case/config/template/input
    # length (functional / core / GR / native).
    effective_config = {
        "case_id": "main",
        "sync_reps": 64,
        "fit_reps": fit_reps,
        "validation_reps": validation_reps,
        "online_validation_reps": validation_reps,
        "evaluation_reps": evaluation_reps,
        "sic_model_mode": S1_MODEL_MODE,
        "sic_k": 4,
        "template_id": "comm_code10",
        "comm_template_sha256": template_sha,
        "input_length": sample_count,
        "search_roi": [0, 1024],
    }
    effective_hash = hashlib.sha256(
        json.dumps(effective_config, sort_keys=True).encode("utf-8")).hexdigest()
    layer_identity = {
        "case_id": "main",
        "config_hash": effective_hash,
        "sic_model_mode": S1_MODEL_MODE,
        "template_id": "comm_code10",
        "input_length": sample_count,
        "validation_reps": validation_reps,
    }

    manifest = {
        "run_id": "run-main-1", "commit": fake_commit, "dirty": False,
        "build_type": "Release", "compiler": "GNU 11.4.0",
        "flags": ["-O3", "-march=native"],
        "lib_paths": {"gnuradio_uwb": "/build-sic/lib/libgnuradio-uwb.so",
                      "loaded": ["libgnuradio-uwb.so => /build-sic/lib",
                                 "libgnuradio-runtime.so => /usr/local/lib"]},
        "cpu": "test-cpu", "os": "Linux test", "gnuradio_version": "3.10.9",
        "volk_version": "3.1.0",
        "config_path": "config.json", "config_hash": sha256_file(config_path),
        "effective_config": effective_config,
        "effective_config_hash": effective_hash,
        "implementation_configs": {
            "functional": dict(layer_identity),
            "core": dict(layer_identity),
            "gr": dict(layer_identity),
            "cir": dict(layer_identity),
            "full_demod": dict(layer_identity),
            "native": dict(layer_identity),
        },
        "fixture_hashes": fixture_hashes,
        "template_hashes": {"comm_code10": template_sha},
        "template_meta": {"comm_code10": {"length": template_size,
                                          "source": "templates/comm_code10.f32",
                                          "sha256": template_sha,
                                          "mode": "finite_sync_code_pulse"}},
        "seed_lists": {"calibration": list(range(1000000, 1000004)),
                       "acceptance": list(range(2000000, 2000005))},
        # S1: the generated calibration windows (seed + offset + noise root) so
        # seed reuse can be disproved, not only a disjoint name list.
        "calibration_windows": [
            {"seed": 1000000 + index, "seed_offset": 0,
             "noise_root": "noise-cal-%d" % index} for index in range(4)],
        "sample_rate_work": WORK_HZ, "sample_rate_native": 491520000.0,
        "input_grid": "work", "preset": "main", "sic_mode": "estimate",
        "sic_model_mode": S1_MODEL_MODE,
        # R10: provenance, an explicit 未实跑 MATLAB status, and an explicit
        # "not superseded" flag.
        "superseded": False,
        "provenance": {
            "repo_root": REPO_ROOT,
            "git_head": fake_commit,
            "git_branch": "feature/sic-research",
            "git_dirty": False,
            "git_status_sha256": EMPTY_SHA256,
            "benchmark_exe_path": "benchmark_preamble_sic",
            "benchmark_exe_sha256": bench_sha,
            "benchmark_source_sha256": bench_src_sha,
            "libgnuradio_uwb_path": "lib/libgnuradio-uwb.so.1.0.0",
            "libgnuradio_uwb_sha256": lib_sha,
            "input_source": "work",
            "production_profile": False,
            "sic_model_mode": S1_MODEL_MODE,
        },
        "matlab": {
            "status": "未实跑",
            "command": "matlab -batch \"run('testdata/sic_preamble/matlab/"
                       "verify_comm_template.m')\"",
            "log_path": "matlab/verify_comm_template.log",
            "note": "no MATLAB/Octave run in this environment (R10)",
        },
    }
    with open(os.path.join(run_dir, "run_manifest.json"), "w",
              encoding="utf-8") as handle:
        json.dump(manifest, handle, indent=1)

    # R10 #5: a perf table whose rows bind a config + template hash to an
    # effect-verified case, and a benchmark summary that carries provenance.
    root = os.path.dirname(run_dir)
    with open(os.path.join(root, "real_perf_table.csv"), "w",
              encoding="utf-8", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["pipeline", "mode", "consumer", "config_hash",
                         "comm_preamble_sha256", "input_len", "windows",
                         "effect_ok", "effect_fail", "effect_verified",
                         "production_profile", "official_evidence",
                         "exit_class", "model_mode"])
        writer.writerow(["sic_only", "gr", "sic", effective_hash, template_sha,
                         sample_count, 2000, 4000, 0, "True", "True", "True",
                         "ok", S1_MODEL_MODE])
    bench_dir = os.path.join(root, "real_core")
    os.makedirs(bench_dir, exist_ok=True)
    bench_summary = {
        "schema": "sic_perf_v2", "mode": "gr", "consumer": "sic",
        "pipeline": "sic_only", "input_source": "work",
        "production_profile": True, "official_evidence": True,
        "config_hash": effective_hash, "windows": 2000,
        "templates": {"comm_preamble_code10_sha256": template_sha,
                      "comm_preamble_length": template_size,
                      "comm_template_mode": "finite_sync_code_pulse",
                      "r07_pass": True},
        "input_windows": [{"index": 0, "seed": 1000, "length": sample_count,
                           "sha256": hashlib.sha256(b"win0").hexdigest()}],
        "provenance": {
            "repo_root": REPO_ROOT, "git_head": fake_commit,
            "git_branch": "feature/sic-research", "git_dirty": False,
            "git_status_sha256": EMPTY_SHA256,
            "benchmark_exe_path": os.path.relpath(bench_file, bench_dir),
            "benchmark_exe_sha256": bench_sha,
            "benchmark_source_sha256": bench_src_sha,
            "libgnuradio_uwb_path": os.path.relpath(lib_file, bench_dir),
            "libgnuradio_uwb_sha256": lib_sha,
        },
        "resolved_config": {"preset": "main", "fit_reps": fit_reps,
                            "validation_reps": validation_reps,
                            "online_validation_reps": validation_reps,
                            "evaluation_reps": evaluation_reps,
                            "sic_model_mode": S1_MODEL_MODE},
        "terminal": {"completions": 4000, "downstream_ok": 4000},
        "effect": {"ok": 4000, "fail": 0, "verified": True},
    }
    with open(os.path.join(bench_dir, "gr_sic_summary.json"), "w",
              encoding="utf-8") as handle:
        json.dump(bench_summary, handle, indent=1)

    # S1-7: the results-root index must reference the actual latest run.
    with open(os.path.join(root, "index.json"), "w", encoding="utf-8") as handle:
        json.dump({
            "schema": "sic-s1-results-index/1",
            "latest_run": "main",
            "runs": {"main": {"run_id": "run-main-1", "windows": n_windows,
                              "model_mode": S1_MODEL_MODE,
                              "config_hash": effective_hash}},
        }, handle, indent=1)
    return results_dir_root(run_dir)


def results_dir_root(run_dir):
    return os.path.dirname(run_dir)


def _write_negative_overall(root, rows):
    """Write a ``negative_table.json`` whose overall block is self-consistent."""
    negative = [r for r in rows if r.get("expected_negative") is True]
    neg_n = sum(r["windows"] for r in negative)
    fp = sum(r["detected"] for r in negative)
    fn = sum(r["committed"] for r in negative)
    fo = sum(r["output_changed"] for r in negative)
    fa = sum(r["actionable_overlap"] for r in negative)
    bypass_n = sum(int(r.get("bypassed") or 0) for r in rows)
    bypass_ok = sum(int(r.get("bypass_bit_identical") or 0) for r in rows)
    overall = {
        "negative_windows": neg_n,
        "false_detect": fp, "false_actionable_detect": fa,
        "false_commit": fn, "false_output_change": fo,
        "false_detect_rate": (fp / neg_n) if neg_n else None,
        "false_actionable_detect_rate": (fa / neg_n) if neg_n else None,
        "false_commit_rate": (fn / neg_n) if neg_n else None,
        "false_output_change_rate": (fo / neg_n) if neg_n else None,
        "false_detect_ci95": list(wilson_ci(fp, neg_n)),
        "false_actionable_detect_ci95": list(wilson_ci(fa, neg_n)),
        "false_commit_ci95": list(wilson_ci(fn, neg_n)),
        "false_output_change_ci95": list(wilson_ci(fo, neg_n)),
        "gate_false_commit_max": 0.01,
        "gate_windows_min": ACTIONABLE_WINDOWS_MIN,
        "gate_pass": bool(neg_n >= ACTIONABLE_WINDOWS_MIN and neg_n > 0
                          and fn / neg_n <= 0.01),
        "gate_pass_false_commit_or_change": bool(
            neg_n >= ACTIONABLE_WINDOWS_MIN and neg_n > 0
            and fo / neg_n <= 0.01),
        "bypass_windows": bypass_n, "bypass_bit_identical": bypass_ok,
        "bypass_all_identical": bool(bypass_n == 0 or bypass_n == bypass_ok),
        "expected_negative_cases": sorted(r["case"] for r in negative),
        "ci_method": "Wilson score interval, z=1.96 (95%)",
    }
    with open(os.path.join(root, "negative_table.json"), "w",
              encoding="utf-8") as handle:
        json.dump({"cases": rows, "overall": overall}, handle, indent=1)


def build_good_aggregate(root, kind="sweep", child_names=None):
    """Create an aggregate root (sweep ``cells/`` or negative ``cases/``)."""
    sub = AGGREGATE_SUBDIRS[kind]
    if child_names is None:
        child_names = (["cell_a", "cell_b"] if kind == "sweep"
                       else ["case_a", "case_b"])
    scratch = tempfile.mkdtemp(prefix="sic_agg_", dir=root)
    try:
        build_good_set(scratch)
        run_dir = os.path.join(scratch, "main")
        parent = os.path.join(root, sub)
        for name in child_names:
            shutil.copytree(run_dir, os.path.join(parent, name))
    finally:
        shutil.rmtree(scratch, ignore_errors=True)
    children = [os.path.join(parent, name) for name in child_names]
    if kind == "sweep":
        rows = []
        for child in children:
            summary, _err = load_json(os.path.join(child, "summary.json"))
            rows.append({
                "cell_id": os.path.basename(child),
                "cell_index": len(rows),
                "eff_pass": _summary_eff_pass(summary),
                "median_suppression_db":
                    _summary_median_suppression(summary),
            })
        with open(os.path.join(root, "main_matrix.json"), "w",
                  encoding="utf-8") as handle:
            json.dump({"mode": "sweep", "n_cells": len(rows), "cells": rows},
                      handle, indent=1)
    else:
        rows = []
        for child in children:
            counters, cerr = _case_counters(child)
            if counters is None:
                raise RuntimeError("fixture case counters: %s" % cerr)
            summary, _err = load_json(os.path.join(child, "summary.json"))
            bypassed, bit_identical = _case_bypass(summary)
            rows.append({
                "case": os.path.basename(child),
                "expected_negative": False,
                "windows": counters["windows"],
                "detected": counters["detected"],
                "committed": counters["committed"],
                "output_changed": counters["output_changed"],
                "actionable_overlap": counters["actionable_overlap"],
                "false_commit": None,
                "output_change": None,
                "bypassed": bypassed,
                "bypass_bit_identical": bit_identical,
                "returncode": 0,
            })
        _write_negative_overall(root, rows)
    # The aggregate runner also writes an index of its children next to them.
    index_runs = {}
    for child in children:
        summary, _err = load_json(os.path.join(child, "summary.json"))
        manifest, _err = load_json(os.path.join(child, "run_manifest.json"))
        count = None
        if isinstance(summary, dict):
            count = summary.get("windows_completed")
        if count is None:
            rows, _err = load_jsonl(os.path.join(child, "windows.jsonl"))
            count = len(rows) if rows is not None else None
        index_runs[os.path.basename(child)] = {
            "run_id": manifest.get("run_id") if isinstance(manifest, dict)
            else None,
            "windows": count,
        }
    with open(os.path.join(parent, "index.json"), "w",
              encoding="utf-8") as handle:
        json.dump({"schema": "sic-s1-results-index/1",
                   "latest_run": child_names[-1],
                   "runs": index_runs}, handle, indent=1)
    return parent


def aggregate_mutation_specs():
    """Aggregate-root counterexamples: each mutates a good aggregate root."""
    specs = []

    def _sweep_table(root):
        return os.path.join(root, "main_matrix.json")

    def _negative_table(root):
        return os.path.join(root, "negative_table.json")

    def sweep_row_value_mismatch(root):
        """A sweep row's median_suppression_db disagrees with its cell."""
        path = _sweep_table(root)
        data, _err = load_json(path)
        data["cells"][0]["median_suppression_db"] += 5.0
        with open(path, "w", encoding="utf-8") as handle:
            json.dump(data, handle, indent=1)
    specs.append(("aggregate sweep table/cell mismatch", "sweep",
                  sweep_row_value_mismatch))

    def sweep_row_count_mismatch(root):
        """The matrix carries a row with no cells/<id> dir."""
        path = _sweep_table(root)
        data, _err = load_json(path)
        data["cells"].append(dict(data["cells"][0], cell_id="cell_ghost"))
        with open(path, "w", encoding="utf-8") as handle:
            json.dump(data, handle, indent=1)
    specs.append(("aggregate sweep table row count mismatch", "sweep",
                  sweep_row_count_mismatch))

    def sweep_two_config_hashes(root):
        """Two cells declare different effective_config_hash values."""
        path = os.path.join(root, "cells", "cell_b", "run_manifest.json")
        manifest, _err = load_json(path)
        manifest["effective_config_hash"] = "0" * 64
        with open(path, "w", encoding="utf-8") as handle:
            json.dump(manifest, handle, indent=1)
    specs.append(("aggregate sweep more than one effective_config_hash",
                  "sweep", sweep_two_config_hashes))

    def negative_actionable_too_few(root):
        """An actionable case is measured over fewer than 1000 windows."""
        path = _negative_table(root)
        data, _err = load_json(path)
        row = data["cases"][0]
        row["expected_negative"] = True
        row["false_commit"] = row["committed"]
        row["output_change"] = row["output_changed"]
        _write_negative_overall(root, data["cases"])
    specs.append(("aggregate negative actionable case with < 1000 windows",
                  "negative", negative_actionable_too_few))

    def negative_case_counter_mismatch(root):
        """A per-case committed counter disagrees with windows.jsonl."""
        path = _negative_table(root)
        data, _err = load_json(path)
        data["cases"][0]["committed"] += 1
        with open(path, "w", encoding="utf-8") as handle:
            json.dump(data, handle, indent=1)
    specs.append(("aggregate negative per-case counter mismatch", "negative",
                  negative_case_counter_mismatch))

    def negative_overall_counter_mismatch(root):
        """The overall false_commit disagrees with the per-case counts."""
        path = _negative_table(root)
        data, _err = load_json(path)
        data["overall"]["false_commit"] += 1
        with open(path, "w", encoding="utf-8") as handle:
            json.dump(data, handle, indent=1)
    specs.append(("aggregate negative overall counter mismatch", "negative",
                  negative_overall_counter_mismatch))

    def aggregate_child_failing(root):
        """One child run is internally inconsistent and must be reported."""
        path = os.path.join(root, "cells", "cell_b", "summary.json")
        summary, _err = load_json(path)
        summary["windows_completed"] = summary["windows_completed"] + 1
        with open(path, "w", encoding="utf-8") as handle:
            json.dump(summary, handle, indent=1)
    specs.append(("aggregate child run failing", "sweep",
                  aggregate_child_failing))

    def aggregate_index_stale(root):
        """The aggregate index's latest_run points at a nonexistent child."""
        path = os.path.join(root, "cells", "index.json")
        data, _err = load_json(path)
        data["latest_run"] = "cell_ghost"
        with open(path, "w", encoding="utf-8") as handle:
            json.dump(data, handle, indent=1)
    specs.append(("aggregate index stale latest_run", "sweep",
                  aggregate_index_stale))

    return specs


def self_test_aggregate(work, failures):
    """Self-test the aggregate-root support and its cross-checks."""
    global _QUIET
    print("== self-test: the fabricated aggregate roots must PASS ==")
    for kind in ("sweep", "negative"):
        root = os.path.join(work, "agg_good_%s" % kind)
        os.makedirs(root)
        build_good_aggregate(root, kind)
        _QUIET = True
        bad = verify_results(root)
        _QUIET = False
        if bad:
            print("[FAIL] good %s aggregate PASSES -- got %d failure(s): %s"
                  % (kind, len(bad), [f[0] for f in bad][:5]))
            failures.append("aggregate good %s" % kind)
        else:
            print("[PASS] good %s aggregate PASSES: %d checks, 0 failures"
                  % (kind, len(results)))

    print("== self-test: each corrupted aggregate must FAIL ==")
    for name, kind, mutate in aggregate_mutation_specs():
        agg_root = os.path.join(
            work, "agg_%s" % re.sub(r"[^A-Za-z0-9]+", "_", name))
        os.makedirs(agg_root)
        build_good_aggregate(agg_root, kind)
        try:
            mutate(agg_root)
        except Exception as exc:  # fixture problem, not a verifier result
            print("[FAIL] aggregate mutation '%s' could not be applied: %s"
                  % (name, exc))
            failures.append(name)
            continue
        _QUIET = True
        caught = verify_results(agg_root)
        _QUIET = False
        if caught:
            print("[PASS] aggregate mutation '%s' caught by %d check(s): %s"
                  % (name, len(caught), caught[0][0]))
        else:
            print("[FAIL] aggregate mutation '%s' was NOT caught" % name)
            failures.append(name)


def make_off_variant(root):
    """Turn the built main/estimate set into a valid rev-2 baseline/SIC-off set.

    The raw consumer becomes an exact copy of clean (the generator guarantees
    y == s + w bitwise when off) and the SIC-conditional window/summary fields
    become JSON null with matching not_run[] reasons.
    """
    run_dir = os.path.join(root, "main")

    manifest_path = os.path.join(run_dir, "run_manifest.json")
    with open(manifest_path, "r", encoding="utf-8") as handle:
        manifest = json.load(handle)
    manifest["preset"] = "baseline"
    manifest["sic_mode"] = "off"
    with open(manifest_path, "w", encoding="utf-8") as handle:
        json.dump(manifest, handle, indent=1)

    windows_path = os.path.join(run_dir, "windows.jsonl")
    with open(windows_path, "r", encoding="utf-8") as handle:
        windows = [json.loads(line) for line in handle if line.strip()]
    for window in windows:
        window.pop("sic", None)
        window["sic_k"] = 0
        window["fit_reps"] = None
        window["validation_reps"] = None
        window["roi"] = None
        window["recon"] = None
        window["raw"] = json.loads(json.dumps(window["clean"]))
    n_windows = len(windows)
    with open(windows_path, "w", encoding="utf-8") as handle:
        for window in windows:
            handle.write(json.dumps(window) + "\n")

    csv_path = os.path.join(run_dir, "stage_times.csv")
    with open(csv_path, "r", encoding="utf-8", newline="") as handle:
        rows = list(csv.reader(handle))
    rows = [row for row in rows if len(row) != 8 or row[2] != "sic"]
    with open(csv_path, "w", encoding="utf-8", newline="") as handle:
        csv.writer(handle).writerows(rows)

    summary_path = os.path.join(run_dir, "summary.json")
    with open(summary_path, "r", encoding="utf-8") as handle:
        summary = json.load(handle)
    summary["preset"] = "baseline"
    summary["sic_mode"] = "off"
    summary["sic"] = None
    summary["demod"] = {"clean_ok": n_windows, "raw_ok": n_windows,
                        "sic_ok": None, "raw_ok_sic_fail": None,
                        "raw_fail_sic_ok": None, "failing_stages": []}
    summary["gates"]["sic_speed_pass"] = None
    summary["effect"] = {key: None for key in (
        "median_suppression_db", "frac_ge_3db", "nmse_median", "peak_shift_max",
        "target_amp_err_db_p95", "holdout_nmse", "holdout_suppression_db")}
    summary["effect"]["oracle_windows"] = 0
    summary["effect"]["oracle_excluded"] = True
    summary["effect"]["n_windows"] = 0
    summary["not_run"] = list(summary.get("not_run", [])) + [
        "sic consumer (sic off)",
        "demod.sic_ok (sic off)",
        "demod.raw_ok_sic_fail (sic off)",
        "demod.raw_fail_sic_ok (sic off)",
        "gates.sic_speed_pass (sic off)",
        "effect metrics (sic off)",
    ]
    with open(summary_path, "w", encoding="utf-8") as handle:
        json.dump(summary, handle, indent=1)


def make_zero_commit_variant(root):
    """Turn the built main/estimate set into an all-bypass (zero-commit) run.

    Every window is bypassed, so there is no committed frozen ``SicModel`` to
    audit: the S1 model-evidence family must be SKIPped (not failed).  The
    bypass bit-identity / false-commit / output-change gates are the negative
    aggregate's job, not the S1 model-evidence checks.
    """
    run_dir = os.path.join(root, "main")
    windows_path = os.path.join(run_dir, "windows.jsonl")
    with open(windows_path, "r", encoding="utf-8") as handle:
        windows = [json.loads(line) for line in handle if line.strip()]
    for window in windows:
        window.pop("sic_model", None)
        state = window.setdefault("sic_state", {})
        state["committed"] = False
        state["detected"] = False
        state["bypass_reason"] = "below_detect_threshold"
    with open(windows_path, "w", encoding="utf-8") as handle:
        for window in windows:
            handle.write(json.dumps(window) + "\n")


def make_partial_archive_variant(root, archived=(0, 1, 2, 3), policy=True):
    """Archive only ``archived`` recon windows and declare the archive policy.

    The run then ships arrays for a declared subset (>= S1_MIN_ARCHIVED) and
    states that the non-archived windows are not independently recomputable.
    """
    run_dir = os.path.join(root, "main")
    windows_path = os.path.join(run_dir, "windows.jsonl")
    with open(windows_path, "r", encoding="utf-8") as handle:
        windows = [json.loads(line) for line in handle if line.strip()]
    total = len(windows)
    keep = {int(w) for w in archived}
    recon_dir = os.path.join(run_dir, "recon")
    removed = []
    for wid in range(total):
        if wid in keep:
            continue
        path = os.path.join(recon_dir, "window_%d.npz" % wid)
        if os.path.isfile(path):
            os.remove(path)
            removed.append(wid)

    def edit_manifest(manifest):
        fixture = manifest.get("fixture_hashes")
        if isinstance(fixture, dict):
            for wid in removed:
                fixture.pop("recon/window_%d.npz" % wid, None)

    _manifest_edit(root, edit_manifest)
    if policy:
        def edit_summary(summary):
            # The app writes the policy nested under summary.sic.
            summary.setdefault("sic", {})["archive_policy"] = {
                "per_cell": len(keep),
                "archived_ids": sorted(keep),
                "n_archived": len(keep),
                "total_windows": total,
                "note": ("recon/window_<id>.npz holds y/y_clean/i for the first "
                         "`per_cell` windows; the remaining windows are covered "
                         "by the per-window declared metrics but not archived, "
                         "so they are not independently recomputable"),
            }
        _summary_edit(root, edit_summary)


def make_app_bench_hash_mismatch_variant(root):
    """An APP run whose recorded benchmark-exe hash no longer matches disk.

    The benchmark binary is not on an app run's execution path, so the
    benchmark-exe re-hash is informational: the run must still PASS with a
    recorded skip/note, never a failure.
    """
    def edit(manifest):
        manifest["provenance"]["benchmark_exe_sha256"] = \
            hashlib.sha256(b"rebuilt benchmark binary").hexdigest()
    _manifest_edit(root, edit)


# ---- corrupted variants ----------------------------------------------------


def _windows_insert(root, mutate):
    path = os.path.join(root, "main", "windows.jsonl")
    with open(path, "r", encoding="utf-8") as handle:
        rows = [json.loads(line) for line in handle if line.strip()]
    rows = mutate(rows)
    with open(path, "w", encoding="utf-8") as handle:
        for row in rows:
            handle.write(json.dumps(row) + "\n")


def _summary_edit(root, mutate):
    path = os.path.join(root, "main", "summary.json")
    with open(path, "r", encoding="utf-8") as handle:
        summary = json.load(handle)
    mutate(summary)
    with open(path, "w", encoding="utf-8") as handle:
        json.dump(summary, handle, indent=1)


def _stage_times_edit(root, mutate):
    path = os.path.join(root, "main", "stage_times.csv")
    with open(path, "r", encoding="utf-8", newline="") as handle:
        rows = list(csv.reader(handle))
    rows = mutate(rows)
    with open(path, "w", encoding="utf-8", newline="") as handle:
        csv.writer(handle).writerows(rows)


def _manifest_edit(root, mutate):
    path = os.path.join(root, "main", "run_manifest.json")
    with open(path, "r", encoding="utf-8") as handle:
        manifest = json.load(handle)
    mutate(manifest)
    with open(path, "w", encoding="utf-8") as handle:
        json.dump(manifest, handle, indent=1)


def _index_edit(root, mutate):
    path = os.path.join(root, "index.json")
    with open(path, "r", encoding="utf-8") as handle:
        data = json.load(handle)
    mutate(data)
    with open(path, "w", encoding="utf-8") as handle:
        json.dump(data, handle, indent=1)


def _truth_edit(root, mutate):
    path = os.path.join(root, "main", "fixtures", "truth.jsonl")
    with open(path, "r", encoding="utf-8") as handle:
        rows = [json.loads(line) for line in handle if line.strip()]
    rows = mutate(rows)
    with open(path, "w", encoding="utf-8") as handle:
        for row in rows:
            handle.write(json.dumps(row) + "\n")


def _rewrite_npz(root, wid, transform):
    """Rewrite one recon npz and keep its manifest fixture hash consistent.

    The counterexamples must be caught by the *semantic* R01/R04 checks, not by
    the fixture-hash check, so the manifest hash is updated in lockstep.
    """
    path = os.path.join(root, "main", "recon", "window_%d.npz" % wid)
    with np.load(path) as data:
        payload = {key: np.array(data[key]) for key in data.files}
    payload = transform(payload)
    np.savez(path, **payload)

    def write(manifest):
        manifest.setdefault("fixture_hashes", {})[
            "recon/window_%d.npz" % wid] = sha256_file(path)

    _manifest_edit(root, write)
    return path



def mutation_specs(root):
    specs = []

    def missing_manifest():
        os.remove(os.path.join(root, "main", "run_manifest.json"))
    specs.append(("missing run_manifest.json", missing_manifest))

    def child_sum_exceeds_parent():
        def edit(rows):
            for row in rows:
                if len(row) == 8 and row[5] == "sfd_final" \
                        and row[3] == "demod":
                    row[6] = "90"
            return rows
        _stage_times_edit(root, edit)
    specs.append(("child sum > parent", child_sum_exceeds_parent))

    def csv_missing_scope_column():
        def edit(rows):
            return [[c for index, c in enumerate(row) if index != 3]
                    for row in rows]
        _stage_times_edit(root, edit)
    specs.append(("stage_times drops the scope column",
                  csv_missing_scope_column))

    def missing_pairing():
        def edit(rows):
            del rows[2]["sic"]
            return rows
        _windows_insert(root, edit)
    specs.append(("missing window pairing (no sic consumer)", missing_pairing))

    def absent_not_run_item():
        def edit(summary):
            summary["not_run"] = [e for e in summary["not_run"]
                                  if "native_pass" not in e]
        _summary_edit(root, edit)
    specs.append(("null gate not named in not_run", absent_not_run_item))

    def config_path_missing():
        def edit(manifest):
            manifest["config_path"] = "does/not/exist.json"
        _manifest_edit(root, edit)
    specs.append(("config_path points to a missing file", config_path_missing))

    def fixture_hash_mismatch():
        path = os.path.join(root, "main", "recon", "window_0.npz")
        with open(path, "ab") as handle:
            handle.write(b"\x00")
    specs.append(("fixture hash mismatch", fixture_hash_mismatch))

    def truth_overlap_mismatch():
        def edit(rows):
            rows[0]["comm"]["real_overlap_hi"] = \
                rows[0]["comm"]["real_overlap_hi"] + 1
            return rows
        _truth_edit(root, edit)
    specs.append(("truth sidecar overlap mismatch", truth_overlap_mismatch))

    def truth_isr_mismatch():
        def edit(rows):
            rows[0]["comm"]["isr_db"] = float(rows[0]["comm"]["isr_db"]) + 3.0
            return rows
        _truth_edit(root, edit)
    specs.append(("truth sidecar realized-ISR mismatch", truth_isr_mismatch))

    def effect_n_windows_understated():
        def edit(summary):
            summary["effect"]["n_windows"] = summary["windows_completed"] - 1
        _summary_edit(root, edit)
    specs.append(("effect gate n_windows understates coverage",
                  effect_n_windows_understated))

    def effect_aggregate_drops_window():
        path = os.path.join(root, "main", "windows.jsonl")
        with open(path, "r", encoding="utf-8") as handle:
            windows = [json.loads(line) for line in handle if line.strip()]
        subset = effect_aggregates(windows[1:], include_truth=False)

        def edit(summary):
            for key, value in subset.items():
                if value is not None:
                    summary["effect"][key] = value
        _summary_edit(root, edit)
    specs.append(("effect aggregate silently drops a window",
                  effect_aggregate_drops_window))

    def holdout_value_corrupted():
        def edit(rows):
            recon = rows[0]["recon"]
            recon["holdout_nmse"] = recon["holdout_nmse"] * 0.5
            return rows
        _windows_insert(root, edit)
    specs.append(("holdout declared value corrupted", holdout_value_corrupted))

    def holdout_window_shrunk():
        def edit(rows):
            recon = rows[0]["recon"]
            recon["holdout_lo"] = recon["holdout_hi"] - 10
            return rows
        _windows_insert(root, edit)
    specs.append(("holdout window shrunk (recompute mismatch)",
                  holdout_window_shrunk))

    def holdout_window_overlaps_fit():
        def edit(rows):
            recon = rows[0]["recon"]
            recon["holdout_lo"] = recon["eval_lo"]
            return rows
        _windows_insert(root, edit)
    specs.append(("holdout window overlaps the fit window",
                  holdout_window_overlaps_fit))

    def template_hashes_length_int():
        def edit(manifest):
            manifest["template_hashes"]["comm_code10"] = 1024
        _manifest_edit(root, edit)
    specs.append(("template_hashes carries a length int",
                  template_hashes_length_int))

    def template_meta_sha_mismatch():
        def edit(manifest):
            manifest["template_meta"]["comm_code10"]["sha256"] = "11" * 32
        _manifest_edit(root, edit)
    specs.append(("template_meta sha mismatch", template_meta_sha_mismatch))

    def template_meta_missing():
        def edit(manifest):
            del manifest["template_meta"]["comm_code10"]
        _manifest_edit(root, edit)
    specs.append(("template_hashes id without template_meta",
                  template_meta_missing))

    def duplicate_window_id():
        path = os.path.join(root, "main", "windows.jsonl")
        with open(path, "r", encoding="utf-8") as handle:
            lines = [ln for ln in handle if ln.strip()]
        with open(path, "w", encoding="utf-8") as handle:
            handle.writelines(lines)
            handle.write(lines[0])
    specs.append(("duplicate window_id", duplicate_window_id))

    def overlap_mismatch():
        def edit(rows):
            rows[0]["overlap"]["samples"] = 123
            return rows
        _windows_insert(root, edit)
    specs.append(("overlap samples mismatch", overlap_mismatch))

    def suppression_substitute():
        def edit(rows):
            recon = rows[0]["recon"]
            recon["suppression_db"] = recon["suppression_db"] + 5.0
            recon["y_energy"] = 100.0
            recon["y_clean_energy"] = 1.0
            return rows
        _windows_insert(root, edit)
    specs.append(("suppression not the i-based metric", suppression_substitute))

    def wrong_canonical_psdu():
        def edit(rows):
            rows[0]["clean"]["psdu_hex"] = "00" * 22
            return rows
        _windows_insert(root, edit)
    specs.append(("non-canonical code-9 PSDU", wrong_canonical_psdu))

    def uses_truth_unflagged():
        def edit(rows):
            rows[0]["sic_state"]["uses_truth"] = True
            return rows
        _windows_insert(root, edit)
    specs.append(("uses_truth not flagged diagnostic", uses_truth_unflagged))

    def uses_truth_counted():
        def edit(rows):
            rows[0]["sic_state"]["uses_truth"] = True
            rows[0]["diagnostic_only"] = True
            rows[0]["recon"]["suppression_db"] = 40.0
            rows[0]["recon"]["nmse"] = 10.0 ** (-4.0)
            return rows
        _windows_insert(root, edit)
    specs.append(("uses_truth counted in the effect gate", uses_truth_counted))

    def baseline_raw_neq_clean():
        make_off_variant(root)

        def edit(rows):
            rows[0]["raw"]["status"] = "fcs_failed"
            rows[0]["raw"]["fcs_pass"] = False
            rows[0]["raw"]["psdu_ok"] = False
            rows[0]["raw"]["psdu_hex"] = None
            return rows
        _windows_insert(root, edit)
    specs.append(("baseline raw != clean (verdict)", baseline_raw_neq_clean))

    def baseline_input_hash_mismatch():
        make_off_variant(root)

        def edit(rows):
            rows[0]["raw"]["input_sha256"] = "00" * 32
            return rows
        _windows_insert(root, edit)
    specs.append(("baseline input_sha256 mismatch",
                  baseline_input_hash_mismatch))

    def off_fake_fit_reps():
        make_off_variant(root)

        def edit(rows):
            rows[0]["fit_reps"] = 8
            return rows
        _windows_insert(root, edit)
    specs.append(("SIC-off with a fake positive fit_reps", off_fake_fit_reps))

    def sic_null_not_named():
        make_off_variant(root)

        def edit(summary):
            summary["not_run"] = []
        _summary_edit(root, edit)
    specs.append(("sic null when off not named in not_run", sic_null_not_named))

    def failing_stages_wrong():
        def edit(summary):
            summary["demod"]["failing_stages"] = ["clean"]
        _summary_edit(root, edit)
    specs.append(("demod.failing_stages wrong", failing_stages_wrong))

    def not_run_with_us():
        def edit(rows):
            for row in rows:
                if len(row) == 8 and row[5] == "phr" and row[3] == "demod":
                    row[7] = "not_run"
            return rows
        _stage_times_edit(root, edit)
    specs.append(("not_run stage with a timed us", not_run_with_us))

    # ---- R01 adversarial counterexamples ---------------------------------
    def accurate_ihat_tiny_actual_mask():
        """CE1: perfect reconstruction but a tiny real subtraction support."""
        def transform(data):
            y = data["y"]
            y_clean = np.array(data["y"]).astype(np.complex64)
            y_clean[:] = y                       # output == input by default
            sl = slice(2000, 2020)               # only 20 samples truly changed
            y_clean[sl] = (y[sl].astype(np.complex128)
                           - data["i"][sl].astype(np.complex128))
            out = dict(data)
            out["y_clean"] = y_clean.astype(np.complex64)
            return out
        _rewrite_npz(root, 0, transform)
    specs.append(("accurate i_hat but a tiny actual subtraction mask",
                  accurate_ihat_tiny_actual_mask))

    def uncommitted_counts_as_cancelled():
        """CE2: an accurate i_hat on a window that never committed."""
        def transform(data):
            out = dict(data)
            out["y_clean"] = np.array(data["y"]).astype(np.complex64)
            return out
        _rewrite_npz(root, 0, transform)

        def edit(rows):
            rows[0]["sic_state"]["committed"] = False
            return rows
        _windows_insert(root, edit)
    specs.append(("uncommitted window with an accurate i_hat",
                  uncommitted_counts_as_cancelled))

    # ---- R04 adversarial counterexamples ---------------------------------
    def target_global_attenuation():
        """CE3: output globally attenuated, normalized shape unchanged."""
        def transform(data):
            out = dict(data)
            out["cir_raw_sic"] = (data["cir_raw_sic"] * 0.5).astype(np.complex64)
            return out
        _rewrite_npz(root, 0, transform)
    specs.append(("target: output globally attenuated, shape unchanged",
                  target_global_attenuation))

    def weak_target_lost():
        """CE4: main peak kept but the weak target erased inside the ROI."""
        def transform(data):
            out = dict(data)
            cs = np.array(data["cir_raw_sic"]).astype(np.complex64)
            cs[85:96] = 0.001 * cs[85:96]
            out["cir_raw_sic"] = cs
            return out
        _rewrite_npz(root, 0, transform)
    specs.append(("target: weak target lost while the main peak is kept",
                  weak_target_lost))

    # ---- R01 gate regressions --------------------------------------------
    def gate_on_reconstruction_suppression():
        """A run whose main gate is the reconstruction suppression_db."""
        def edit(summary):
            summary["effect"]["median_suppression_db"] = \
                summary["effect"]["median_actual_suppression_db"] + 2.0
            for key in list(summary["effect"]):
                if key in ACTUAL_SUPPRESSION_MEDIAN_KEYS:
                    summary["effect"][key] = None
        _summary_edit(root, edit)
    specs.append(("main effect gate uses reconstruction suppression_db",
                  gate_on_reconstruction_suppression))

    def actual_metrics_without_arrays():
        """Declared actual metrics but none of the arrays needed to recompute."""
        for wid in range(5):
            os.remove(os.path.join(root, "main", "recon",
                                   "window_%d.npz" % wid))

        def edit(manifest):
            for key in list(manifest.get("fixture_hashes", {})):
                if key.startswith("recon/window_"):
                    del manifest["fixture_hashes"][key]
        _manifest_edit(root, edit)
    specs.append(("declared actual metrics but no y/y_clean/i arrays",
                  actual_metrics_without_arrays))

    def raw_target_only_normalized():
        """A run that carries only the normalized-CIR target as its gate."""
        def edit(rows):
            for row in rows:
                row.pop("target", None)
            return rows
        _windows_insert(root, edit)
    specs.append(("raw target missing, only normalized target present",
                  raw_target_only_normalized))

    # ---- R02 adversarial counterexamples ---------------------------------
    def config_changed_without_record():
        """A layer ran a different input length without flagging it."""
        def edit(manifest):
            manifest["implementation_configs"]["gr"]["input_length"] += 1016
        _manifest_edit(root, edit)
    specs.append(("layer config changed without a recorded difference",
                  config_changed_without_record))

    def validation_reps_silently_bumped():
        """A window widens validation_reps while keeping nominal provenance."""
        def edit(rows):
            rows[0]["validation_reps"] = rows[0]["validation_reps"] + 8
            return rows
        _windows_insert(root, edit)
    specs.append(("validation_reps silently bumped (no record)",
                  validation_reps_silently_bumped))

    def config_unsatisfiable():
        """A declared effective config that cannot be satisfied."""
        def edit(manifest):
            manifest["effective_config"]["fit_reps"] = 60
            manifest["effective_config"]["validation_reps"] = 32
        _manifest_edit(root, edit)
    specs.append(("declared config is unsatisfiable", config_unsatisfiable))

    # ---- R02/R06 adversarial counterexamples -----------------------------
    def search_equals_subtraction_roi():
        """The search ROI is used as the subtraction ROI (tail clipped)."""
        def edit(rows):
            regions = rows[0]["regions"]
            regions["search_roi"] = list(regions["subtraction_roi"])
            return rows
        _windows_insert(root, edit)
    specs.append(("search ROI equals the subtraction ROI",
                  search_equals_subtraction_roi))

    def eval_overlaps_fit_region():
        """The final evaluation region overlaps the fit/estimation region."""
        def edit(rows):
            regions = rows[0]["regions"]
            regions["eval_hi"] = regions["estimation_hi"]
            return rows
        _windows_insert(root, edit)
    specs.append(("eval region overlaps the estimation region",
                  eval_overlaps_fit_region))

    def eval_overlaps_online_region():
        """The final evaluation region overlaps the online-validation region."""
        def edit(rows):
            regions = rows[0]["regions"]
            regions["eval_lo"] = regions["online_validation_lo"] + 1000
            regions["eval_hi"] = regions["online_validation_lo"] + 2000
            return rows
        _windows_insert(root, edit)
    specs.append(("eval region overlaps the online-validation region",
                  eval_overlaps_online_region))

    def gate_on_eval_region():
        """The commit gate is decided on the final evaluation region."""
        def edit(rows):
            regions = rows[0]["regions"]
            regions["gate_lo"] = regions["eval_lo"]
            regions["gate_hi"] = regions["eval_hi"]
            regions["gate_keys"] = ["nmse", "suppression_db"]
            return rows
        _windows_insert(root, edit)
    specs.append(("commit gate computed on the eval region",
                  gate_on_eval_region))

    # ---- R10 provenance / not_run / superseded / MATLAB / perf mutations ---
    def provenance_missing():
        """The whole provenance binding block is omitted."""
        def edit(manifest):
            manifest.pop("provenance", None)
        _manifest_edit(root, edit)
    specs.append(("R10 provenance: binding block omitted", provenance_missing))

    def provenance_bench_hash_missing():
        """The recorded benchmark binary hash is absent."""
        def edit(manifest):
            manifest["provenance"].pop("benchmark_exe_sha256", None)
        _manifest_edit(root, edit)
    specs.append(("R10 provenance: benchmark hash absent",
                  provenance_bench_hash_missing))

    def provenance_lib_hash_missing():
        """The recorded loaded-library hash is absent."""
        def edit(manifest):
            manifest["provenance"].pop("libgnuradio_uwb_sha256", None)
        _manifest_edit(root, edit)
    specs.append(("R10 provenance: library hash absent",
                  provenance_lib_hash_missing))

    def provenance_lib_hash_mismatch():
        """A recorded loaded-library hash that no longer matches disk (APP run).

        The library hash stays strict for EVERY run, so an app run with a
        mismatched library hash must still FAIL (the scoping only relaxed the
        benchmark-exe re-hash)."""
        def edit(manifest):
            manifest["provenance"]["libgnuradio_uwb_sha256"] = \
                hashlib.sha256(b"rebuilt loaded library").hexdigest()
        _manifest_edit(root, edit)
    specs.append(("R10 provenance: library hash mismatched (app run)",
                  provenance_lib_hash_mismatch))

    def benchmark_exe_hash_mismatch():
        """A declared BENCHMARK run whose benchmark-exe re-hash mismatches.

        The benchmark summary declares itself benchmark output, so its
        benchmark-exe re-hash stays strict and the mismatch must FAIL."""
        path = os.path.join(root, "real_core", "gr_sic_summary.json")
        data, _err = load_json(path)
        data["provenance"]["benchmark_exe_sha256"] = \
            hashlib.sha256(b"rebuilt benchmark binary").hexdigest()
        with open(path, "w", encoding="utf-8") as handle:
            json.dump(data, handle, indent=1)
    specs.append(("R10 provenance: benchmark-exe hash mismatched "
                  "(benchmark run)", benchmark_exe_hash_mismatch))

    def benchmark_lib_hash_mismatch():
        """A declared BENCHMARK run whose library re-hash mismatches."""
        path = os.path.join(root, "real_core", "gr_sic_summary.json")
        data, _err = load_json(path)
        data["provenance"]["libgnuradio_uwb_sha256"] = \
            hashlib.sha256(b"rebuilt loaded library").hexdigest()
        with open(path, "w", encoding="utf-8") as handle:
            json.dump(data, handle, indent=1)
    specs.append(("R10 provenance: library hash mismatched (benchmark run)",
                  benchmark_lib_hash_mismatch))

    def provenance_dirty_patch_missing():
        """A dirty tree with no dirty-patch identifier."""
        def edit(manifest):
            manifest["dirty"] = True
            manifest["provenance"]["git_dirty"] = True
            manifest["provenance"].pop("git_status_sha256", None)
        _manifest_edit(root, edit)
    specs.append(("R10 provenance: dirty-patch id absent",
                  provenance_dirty_patch_missing))

    def provenance_stale_commit():
        """A placeholder commit identifier."""
        def edit(manifest):
            manifest["commit"] = "unknown"
            manifest["provenance"]["git_head"] = "unknown"
        _manifest_edit(root, edit)
    specs.append(("R10 provenance: stale placeholder commit",
                  provenance_stale_commit))

    def provenance_input_hash_missing():
        """No input/fixture hash binding and no window fixtures."""
        def edit(manifest):
            manifest.pop("fixture_hashes", None)
        _manifest_edit(root, edit)
    specs.append(("R10 provenance: input hash absent",
                  provenance_input_hash_missing))

    def gate_false_without_evidence():
        """A false gate with no tested evidence."""
        def edit(summary):
            summary["gates"]["native_pass"] = False
            summary["not_run"] = [e for e in summary["not_run"]
                                  if "native_pass" not in e]
        _summary_edit(root, edit)
    specs.append(("R10 not_run: false gate without tested evidence",
                  gate_false_without_evidence))

    def static_not_run_template():
        """not_run claims a required artifact that actually exists."""
        def edit(summary):
            summary["not_run"] = list(summary["not_run"]) + ["windows.jsonl"]
        _summary_edit(root, edit)
    specs.append(("R10 not_run: static template names a present artifact",
                  static_not_run_template))

    def superseded_without_reason():
        """A superseded marker with no reason."""
        def edit(manifest):
            manifest["superseded"] = True
            manifest.pop("superseded_reason", None)
        _manifest_edit(root, edit)
    specs.append(("R10 superseded: marked without a reason",
                  superseded_without_reason))

    def superseded_claims_production():
        """A superseded run that still claims production evidence."""
        def edit(manifest):
            manifest["superseded"] = {"reason": "old synthetic run"}
            manifest["provenance"]["production_profile"] = True
        _manifest_edit(root, edit)
    specs.append(("R10 superseded: claims production evidence",
                  superseded_claims_production))

    def superseded_index_overrides_must_pass():
        """A results index marks the run superseded; it must not claim
        production evidence (proves the index path is consulted)."""
        with open(os.path.join(root, "index.json"), "w",
                  encoding="utf-8") as handle:
            json.dump({"superseded": {"main": "old synthetic run"}}, handle)
        def edit(manifest):
            manifest["provenance"]["production_profile"] = True
        _manifest_edit(root, edit)
    specs.append(("R10 superseded: index marks superseded + production claim",
                  superseded_index_overrides_must_pass))

    def matlab_executed_missing_log():
        """已实跑 but the log is not on disk."""
        def edit(manifest):
            manifest["matlab"] = {
                "status": "已实跑",
                "command": "matlab -batch run_verify",
                "log_path": "matlab/missing.log"}
        _manifest_edit(root, edit)
    specs.append(("R10 MATLAB: 已实跑 without a log",
                  matlab_executed_missing_log))

    def matlab_parity_without_run():
        """A MATLAB-parity claim while MATLAB was not run."""
        def edit(summary):
            summary["effect"]["matlab_verified"] = True
        _summary_edit(root, edit)
    specs.append(("R10 MATLAB: parity claimed without a run",
                  matlab_parity_without_run))

    def perf_row_not_effect_verified():
        """A production perf row that is not effect-verified."""
        path = os.path.join(root, "real_perf_table.csv")
        with open(path, "r", encoding="utf-8", newline="") as handle:
            rows = list(csv.reader(handle))
        for row in rows[1:]:
            row[9] = "False"    # effect_verified
            row[10] = "True"    # production_profile
        with open(path, "w", encoding="utf-8", newline="") as handle:
            csv.writer(handle).writerows(rows)
    specs.append(("R10 perf: production row not effect-verified",
                  perf_row_not_effect_verified))

    def perf_row_loses_config_hash():
        """A perf row with no config hash binding."""
        path = os.path.join(root, "real_perf_table.csv")
        with open(path, "r", encoding="utf-8", newline="") as handle:
            rows = list(csv.reader(handle))
        for row in rows[1:]:
            row[3] = ""
        with open(path, "w", encoding="utf-8", newline="") as handle:
            csv.writer(handle).writerows(rows)
    specs.append(("R10 perf: row loses its config hash",
                  perf_row_loses_config_hash))

    def benchmark_claims_production_synthetic():
        """A production benchmark summary fed by synthetic input."""
        path = os.path.join(root, "real_core", "gr_sic_summary.json")
        data, _err = load_json(path)
        data["input_source"] = "synthetic"
        with open(path, "w", encoding="utf-8") as handle:
            json.dump(data, handle, indent=1)
    specs.append(("R10 production: synthetic input claimed as production",
                  benchmark_claims_production_synthetic))

    # ---- S1 adversarial counterexamples (one per S1 check 1-7) ------------
    def s1_estimation_range_reads_past_prefix():
        """S1-1: an estimation input range extends past observed_end_sample."""
        def edit(rows):
            model = rows[0]["sic_model"]
            model["estimation_ranges"]["cir"] = [
                model["training_lo"],
                model["observed_end_sample"] + SYNC_SAMPLES_PER_SYMBOL]
            return rows
        _windows_insert(root, edit)
    specs.append(("S1-1 estimation range reads past the visible prefix",
                  s1_estimation_range_reads_past_prefix))

    def s1_strict_claim_with_52_reps():
        """S1-1: the model claims 52 training reps (not a strict 8-SYNC)."""
        def edit(rows):
            rows[0]["sic_model"]["fit_reps"] = 52
            return rows
        _windows_insert(root, edit)
    specs.append(("S1-1 strict 8-SYNC claim with fit_reps != 8",
                  s1_strict_claim_with_52_reps))

    def s1_guard_not_declared():
        """S1-1: the guard length is not explicitly declared."""
        def edit(rows):
            rows[0]["sic_model"].pop("guard", None)
            return rows
        _windows_insert(root, edit)
    specs.append(("S1-1 guard length not explicitly declared",
                  s1_guard_not_declared))

    def s1_frozen_param_mutated_at_apply():
        """S1-2: a frozen gain changed after estimation (apply diverges)."""
        def edit(rows):
            rows[0]["sic_model"]["apply"]["params"]["gain"] = [[1.25, 0.0]]
            return rows
        _windows_insert(root, edit)
    specs.append(("S1-2 frozen parameter mutated after estimation",
                  s1_frozen_param_mutated_at_apply))

    def s1_refit_during_apply():
        """S1-2: the apply step re-fits the model."""
        def edit(rows):
            rows[0]["sic_model"]["apply"]["refit_count"] = 1
            return rows
        _windows_insert(root, edit)
    specs.append(("S1-2 apply re-fits the frozen model", s1_refit_during_apply))

    def s1_support_clipped_to_8sync():
        """S1-3: the support is clipped to the first 8 SYNC."""
        def edit(rows):
            model = rows[0]["sic_model"]
            model["support_hi"] = (model["origin"] + 8 * SYNC_SAMPLES_PER_SYMBOL
                                   + model["support_post_tail"])
            model["apply"]["params"]["support_hi"] = model["support_hi"]
            return rows
        _windows_insert(root, edit)
    specs.append(("S1-3 support clipped to the first 8 SYNC",
                  s1_support_clipped_to_8sync))

    def s1_support_from_confirmed_count():
        """S1-3: the support comes from a measured/confirmed count."""
        def edit(rows):
            model = rows[0]["sic_model"]
            model["support_hi"] = (model["origin"]
                                   + 60 * SYNC_SAMPLES_PER_SYMBOL
                                   + model["support_post_tail"])
            model["apply"]["params"]["support_hi"] = model["support_hi"]
            model["support_source"] = "measured_confirmed"
            return rows
        _windows_insert(root, edit)
    specs.append(("S1-3 support from a confirmed count, not the nominal "
                  "prediction", s1_support_from_confirmed_count))

    def s1_eval_left_uncancelled():
        """S1-4: the independent-evaluation region is not in the subtraction."""
        def edit(rows):
            model = rows[0]["sic_model"]
            model["sub_hi"] = model["eval_lo"]
            model["apply"]["params"]["sub_hi"] = model["sub_hi"]
            return rows
        _windows_insert(root, edit)
    specs.append(("S1-4 evaluation region left un-cancelled",
                  s1_eval_left_uncancelled))

    def s1_eval_overlaps_training():
        """S1-4: the evaluation region overlaps the training region."""
        def edit(rows):
            model = rows[0]["sic_model"]
            model["eval_lo"] = model["training_hi"] - 500
            return rows
        _windows_insert(root, edit)
    specs.append(("S1-4 evaluation region overlaps training",
                  s1_eval_overlaps_training))

    def s1_gate_on_eval_region():
        """S1-4: the commit gate is decided on the evaluation region."""
        def edit(rows):
            model = rows[0]["sic_model"]
            rows[0]["regions"]["gate_lo"] = model["eval_lo"]
            rows[0]["regions"]["gate_hi"] = model["eval_hi"]
            rows[0]["regions"]["gate_keys"] = ["nmse"]
            return rows
        _windows_insert(root, edit)
    specs.append(("S1-4 commit gate uses evaluation-region data",
                  s1_gate_on_eval_region))

    def s1_eval_metric_copied_from_full():
        """S1-5: the eval metric is copied from the full-overlap metric."""
        def edit(rows):
            recon = rows[0]["recon"]
            recon["eval_actual_suppression_db"] = \
                recon["actual_suppression_db"]
            return rows
        _windows_insert(root, edit)
    specs.append(("S1-5 evaluation metric copied from the full overlap",
                  s1_eval_metric_copied_from_full))

    def s1_eval_arrays_left_uncancelled():
        """S1-5: the archived output does not change over the eval region."""
        def transform(data):
            model_lo, model_hi = 9128, 66024
            out = dict(data)
            y_clean = np.array(data["y"]).astype(np.complex64)
            y_clean[model_lo:model_hi] = data["y"][model_lo:model_hi]
            out["y_clean"] = y_clean
            return out
        _rewrite_npz(root, 0, transform)
    specs.append(("S1-5 evaluation region not actually cancelled in the arrays",
                  s1_eval_arrays_left_uncancelled))

    def s1_only_reconstruction_medians():
        """S1-5: only reconstruction NMSE is reported (no region medians)."""
        def edit(summary):
            for key in ("median_eval_actual_suppression_db",
                        "median_tail_actual_suppression_db"):
                summary["effect"].pop(key, None)
            return summary
        _summary_edit(root, edit)
    specs.append(("S1-5 only reconstruction NMSE is reported",
                  s1_only_reconstruction_medians))

    def s1_acceptance_reuses_calibration_seed():
        """S1-6: a formal window reuses a calibration seed."""
        def edit(rows):
            rows[0]["generator"]["seed"] = 1000000
            rows[0]["generator"]["seed_set"] = "calibration"
            return rows
        _windows_insert(root, edit)
    specs.append(("S1-6 acceptance window reuses a calibration seed",
                  s1_acceptance_reuses_calibration_seed))

    def s1_reused_acceptance_noise():
        """S1-6: two formal windows share one generated window/noise."""
        def edit(rows):
            rows[0]["generator"] = dict(rows[1]["generator"])
            return rows
        _windows_insert(root, edit)
    specs.append(("S1-6 acceptance window reuses another window/noise",
                  s1_reused_acceptance_noise))

    def s1_calibration_windows_missing():
        """S1-6: the generated calibration windows are not recorded."""
        def edit(manifest):
            manifest.pop("calibration_windows", None)
        _manifest_edit(root, edit)
    specs.append(("S1-6 calibration windows not recorded",
                  s1_calibration_windows_missing))

    def s1_layer_model_mode_differs():
        """S1-7: one layer ran a different model mode."""
        def edit(manifest):
            manifest["implementation_configs"]["gr"]["sic_model_mode"] = \
                "legacy_cover"
        _manifest_edit(root, edit)
    specs.append(("S1-7 layer runs a different model mode",
                  s1_layer_model_mode_differs))

    def s1_cir_row_missing():
        """S1-7: the CIR row is not declared at all."""
        def edit(manifest):
            manifest["implementation_configs"].pop("cir", None)
        _manifest_edit(root, edit)
    specs.append(("S1-7 CIR implementation row missing", s1_cir_row_missing))

    def s1_stale_index():
        """S1-7: index.json points at a run that does not exist."""
        def edit(index):
            index["latest_run"] = "ghost_run"
        _index_edit(root, edit)
    specs.append(("S1-7 stale index.json latest_run", s1_stale_index))

    def s1_index_count_mismatch():
        """S1-7: the index report count disagrees with the artifact."""
        def edit(index):
            index["runs"]["main"]["windows"] = 3
        _index_edit(root, edit)
    specs.append(("S1-7 index report/artifact count mismatch",
                  s1_index_count_mismatch))

    # ---- S1 clipped-region / archive-policy counterexamples ---------------
    def s1_region_outside_sub_full_coverage():
        """S1-4: the nominal eval/tail region extends outside the subtraction.

        The subtraction is clipped (public ROI) but the eval/tail regions still
        declare the full nominal repetition range and no covered fraction.
        """
        def edit(rows):
            model = rows[0]["sic_model"]
            clipped_hi = model["eval_lo"] + 4 * SYNC_SAMPLES_PER_SYMBOL
            model["sub_hi"] = clipped_hi
            model["apply"]["params"]["sub_hi"] = clipped_hi
            rows[0]["regions"]["sub_hi"] = clipped_hi
            return rows
        _windows_insert(root, edit)
    specs.append(("S1-4 nominal eval/tail extends outside the subtraction while "
                  "claiming full coverage", s1_region_outside_sub_full_coverage))

    def s1_archive_policy_missing():
        """S1-archive: a partial recon archive with no declared policy."""
        make_partial_archive_variant(root, policy=False)
    specs.append(("S1 archive policy missing for a partial archive",
                  s1_archive_policy_missing))

    def s1_archive_too_few():
        """S1-archive: the policy archives fewer than 4 windows per case."""
        make_partial_archive_variant(root, archived=(0, 1, 2))
    specs.append(("S1 archive policy declares < 4 archived windows",
                  s1_archive_too_few))

    def s1_archive_metric_mismatch():
        """S1-archive: a declared metric disagrees on an archived window."""
        make_partial_archive_variant(root)

        def edit(rows):
            rows[0]["recon"]["eval_actual_suppression_db"] += 5.0
            return rows
        _windows_insert(root, edit)
    specs.append(("S1 archive policy: archived-window metric disagrees",
                  s1_archive_metric_mismatch))

    def s1_committed_window_lacks_model():
        """S1: a run WITH committed windows still demands frozen-model evidence.

        Only an all-bypass (zero-commit) run may skip the model-evidence family.
        """
        def edit(rows):
            rows[0].pop("sic_model", None)
            return rows
        _windows_insert(root, edit)
    specs.append(("S1 committed window lacks the frozen SicModel",
                  s1_committed_window_lacks_model))

    return specs


def mutation_names():
    """Stable ordered names; used to build one fresh variant per mutation."""
    return [name for name, _ in mutation_specs("__NAME_PROBE__")]


def self_test():
    global _QUIET
    _QUIET = False
    work = tempfile.mkdtemp(prefix="sic_s0_selftest_",
                            dir="/tmp/opencode" if os.path.isdir("/tmp/opencode")
                            else None)
    failures = []
    try:
        print("== self-test: the fabricated consistent set must PASS ==")
        good_root = os.path.join(work, "good")
        os.makedirs(good_root)
        results_dir = build_good_set(good_root)
        _QUIET = True
        bad = verify_results(results_dir)
        _QUIET = False
        if bad:
            print("[FAIL] good set PASSES (0 failures) -- got %d failure(s): %s"
                  % (len(bad), [f[0] for f in bad]))
            return 1
        print("[PASS] good set PASSES: %d checks, 0 failures" % len(results))

        print("== self-test: the fabricated SIC-off set must PASS ==")
        off_root = os.path.join(work, "good_off")
        os.makedirs(off_root)
        build_good_set(off_root)
        make_off_variant(off_root)
        _QUIET = True
        bad_off = verify_results(results_dir_root(
            os.path.join(off_root, "main")))
        _QUIET = False
        if bad_off:
            print("[FAIL] SIC-off set PASSES -- got %d failure(s): %s"
                  % (len(bad_off), [f[0] for f in bad_off]))
            return 1
        print("[PASS] SIC-off set PASSES: %d checks, 0 failures" % len(results))

        print("== self-test: an all-bypass (zero-commit) run must SKIP the S1 "
              "model evidence, not fail it ==")
        zc_root = os.path.join(work, "zero_commit")
        os.makedirs(zc_root)
        build_good_set(zc_root)
        make_zero_commit_variant(zc_root)
        _QUIET = True
        zc_bad = verify_results(results_dir_root(os.path.join(zc_root, "main")))
        _QUIET = False
        zc_model = [f for f in zc_bad
                    if "S1 model evidence" in f[0]
                    or "S1-1 strict-prefix" in f[0]
                    or "S1-2 frozen-model" in f[0]
                    or "S1-3 support semantics" in f[0]
                    or "S1-4 regions" in f[0]]
        if zc_model:
            print("[FAIL] zero-commit run must skip the S1 model-evidence "
                  "family -- got %s" % [f[0] for f in zc_model])
            failures.append("zero-commit skips the S1 model evidence")
        else:
            print("[PASS] zero-commit run skips the S1 model-evidence family "
                  "(other failures: %d)" % len(zc_bad))

        print("== self-test: a declared partial archive policy must PASS ==")
        pa_root = os.path.join(work, "partial_archive")
        os.makedirs(pa_root)
        build_good_set(pa_root)
        make_partial_archive_variant(pa_root)
        _QUIET = True
        pa_bad = verify_results(results_dir_root(os.path.join(pa_root, "main")))
        _QUIET = False
        if pa_bad:
            print("[FAIL] declared partial archive policy PASSES -- got %d "
                  "failure(s): %s" % (len(pa_bad), [f[0] for f in pa_bad]))
            failures.append("partial archive policy passes")
        else:
            print("[PASS] declared partial archive policy PASSES: 0 failures")

        print("== self-test: an APP run with a mismatched benchmark-exe hash must "
              "PASS (informational) ==")
        ab_root = os.path.join(work, "app_bench_mismatch")
        os.makedirs(ab_root)
        build_good_set(ab_root)
        make_app_bench_hash_mismatch_variant(ab_root)
        _QUIET = True
        ab_bad = verify_results(results_dir_root(os.path.join(ab_root, "main")))
        _QUIET = False
        if ab_bad:
            print("[FAIL] app run with a mismatched benchmark-exe hash must PASS "
                  "-- got %d failure(s): %s" % (len(ab_bad), [f[0] for f in ab_bad]))
            failures.append("app bench-exe mismatch passes (informational)")
        else:
            print("[PASS] app run with a mismatched benchmark-exe hash PASSES "
                  "(recorded informational skip, not a failure)")

        print("== self-test: each corrupted variant must FAIL ==")
        variant_dir = os.path.join(work, "variants")
        os.makedirs(variant_dir)
        for name in mutation_names():
            case_root = os.path.join(
                variant_dir, re.sub(r"[^A-Za-z0-9]+", "_", name))
            os.makedirs(case_root)
            build_good_set(case_root)
            spec = dict(mutation_specs(case_root))
            try:
                spec[name]()
            except Exception as exc:  # fixture problem, not a verifier result
                print("[FAIL] mutation '%s' could not be applied: %s"
                      % (name, exc))
                failures.append(name)
                continue
            _QUIET = True
            caught = verify_results(results_dir_root(
                os.path.join(case_root, "main")))
            _QUIET = False
            if caught:
                print("[PASS] mutation '%s' caught by %d check(s): %s"
                      % (name, len(caught), caught[0][0]))
            else:
                print("[FAIL] mutation '%s' was NOT caught" % name)
                failures.append(name)
        self_test_aggregate(work, failures)
        print("=" * 72)
        if failures:
            print("SELF-TEST FAILED: %s" % failures)
            return 1
        print("SELF-TEST PASSED: the good set passes and every mutation is detected")
        return 0
    finally:
        shutil.rmtree(work, ignore_errors=True)


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Independent verifier for the S0 SIC artifacts.")
    parser.add_argument("--results", help="directory holding the declared artifacts")
    parser.add_argument("--verbose", action="store_true",
                        help="print every check (aggregate roots print a "
                             "compact per-child summary by default)")
    parser.add_argument("--self-test", action="store_true",
                        help="fabricate consistent + corrupted sets and assert the verdicts")
    args = parser.parse_args(argv)

    if args.self_test:
        return self_test()
    if not args.results:
        parser.error("either --results DIR or --self-test is required")
    if not os.path.isdir(args.results):
        sys.stderr.write("error: %s is not a directory\n" % args.results)
        return 2
    failed = verify_results(args.results, verbose=args.verbose)
    print("=" * 72)
    print("%d/%d checks pass" % (len(results) - len(failed), len(results)))
    if failed:
        shown = failed[:10]
        print("FAILED: %d check(s); first %d: %s"
              % (len(failed), len(shown),
                 " | ".join(f[0] for f in shown)))
        if len(failed) > len(shown):
            print("        ... and %d more failure(s)"
                  % (len(failed) - len(shown)))
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
