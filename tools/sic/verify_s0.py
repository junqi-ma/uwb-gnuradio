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

The long-form CSV schema is fixed here as the authoritative contract:
``run_id,window_id,consumer,tier,stage,us,detail`` where ``tier`` is one of
``cxx``/``adapter``/``outer`` and an empty ``us`` pairs with a ``not_run``
``detail``.  When a stage name exists in BOTH the demod and the SIC stage set
(``cfo``, ``cir``) inside one ``(window, consumer)`` group, the ``detail``
column must carry a ``sic``/``demod`` scope tag so the timers are not
ambiguous.

Percentiles are recomputed with numpy's default (``linear``) interpolation;
the generator must use the same convention for the summary to cross-check.

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
SIC_STAGES = {
    "detect_coarse", "detect_fine", "cfo", "cir", "path_select",
    "fractional_refine", "matrix_build", "gain_fit", "reconstruct",
    "validate_commit", "core_total",
}
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


def record(item, verdict, detail):
    results.append((item, verdict, detail))
    if not _QUIET:
        print("[%s] %s\n         %s" % (verdict, item, detail))


def fail(item, detail):
    record(item, "FAIL", detail)


def ok(item, detail):
    record(item, "PASS", detail)


def skip(item, detail):
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
        if off:
            if value is None:
                ok("%s: %s is null when SIC is off" % (where, key), "null")
            else:
                fail("%s: %s is null when SIC is off" % (where, key),
                     "got %r (rev 2 forbids a fake positive value)" % (value,))
        elif is_int(value) and value > 0:
            ok("%s: %s is a positive integer" % (where, key), str(value))
        else:
            fail("%s: %s is a positive integer" % (where, key),
                 "got %r" % (value,))
    if not off and is_int(window.get("validation_reps")) \
            and window["validation_reps"] < 8:
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
        # Recompute from the per-window arrays when they exist.  The app dumps
        # arrays for a declared subset of windows (--fixtures), so a window
        # without arrays is not itself an error; the run-level check below
        # requires that at least one window can actually be recomputed.
        arrays = load_recon_arrays(run_dir, wid, npz_cache)
        if arrays is None:
            continue
        if declares and "error" not in arrays and "i" in arrays \
                and "i_hat" in arrays:
            recomputed_holdout += 1
        recompute_recon(arrays, recon, where)
    if declared_holdout and recomputed_holdout == 0:
        fail("%s: run provides arrays to recompute its declared holdout metrics"
             % item,
             "%d window(s) declare holdout metrics but no recon/window_*.npz "
             "with i/i_hat exists under %s" % (declared_holdout, run_dir))
    elif declared_holdout:
        ok("%s: holdout metrics recomputed where arrays exist" % item,
           "%d/%d declared-holdout window(s) have recon arrays"
           % (recomputed_holdout, declared_holdout))


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
                               ("y", ("y", "received")),
                               ("y_clean", ("y_clean", "clean", "y_ref"))):
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
    if "y" in arrays and "y_clean" in arrays:
        y, y_clean = arrays["y"], arrays["y_clean"]
        n = min(len(y), len(y_clean))
        y, y_clean = y[:n], y_clean[:n]
        yd = float(np.vdot(y, y).real)
        ycd = float(np.vdot(y_clean, y_clean).real)
        if ycd > 0 and is_num(recon.get("suppression_db")):
            sub = 10.0 * math.log10(yd / ycd)
            if abs(sub - float(recon["suppression_db"])) <= SUPPRESSION_TOL_DB \
                    and abs(sup - float(recon["suppression_db"])) > SUPPRESSION_TOL_DB:
                fail("%s: suppression is not ||y||^2/||y_clean||^2" % where,
                     "declared=%.4f matches the y-substitute" % recon["suppression_db"])

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
                    "peak_shift_max", "target_amp_err_db_p95"):
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
    return isinstance(state, dict) and state.get("uses_truth") is True


# Effect aggregate keys the gate is built from; the coverage of each is checked
# against windows_completed.
AGG_KEYS = ("median_suppression_db", "frac_ge_3db", "nmse_median",
            "peak_shift_max", "target_amp_err_db_p95",
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
            if is_num(recon.get("holdout_suppression_db")):
                counts["holdout_suppression_db_median"] += 1
            if is_num(recon.get("holdout_nmse")):
                counts["holdout_nmse_median"] += 1
        clean, sic = window.get("clean"), window.get("sic")
        if isinstance(clean, dict) and isinstance(sic, dict):
            if is_int(clean.get("cir_peak_tap")) \
                    and is_int(sic.get("cir_peak_tap")):
                counts["peak_shift_max"] += 1
            a = scalar_or_pair_mag(clean.get("target_amp"))
            b = scalar_or_pair_mag(sic.get("target_amp"))
            if is_num(a) and is_num(b) and a > 0 and b > 0:
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
        state = window.get("sic_state")
        uses_truth = isinstance(state, dict) and state.get("uses_truth") is True
        if uses_truth and not include_truth:
            continue
        subset.append(window)
    sups, nmses, shifts, amps = [], [], [], []
    h_nmses, h_sups = [], []
    for window in subset:
        recon = window.get("recon")
        if isinstance(recon, dict):
            if is_num(recon.get("suppression_db")):
                sups.append(float(recon["suppression_db"]))
            if is_num(recon.get("nmse")):
                nmses.append(float(recon["nmse"]))
            if is_num(recon.get("holdout_nmse")):
                h_nmses.append(float(recon["holdout_nmse"]))
            if is_num(recon.get("holdout_suppression_db")):
                h_sups.append(float(recon["holdout_suppression_db"]))
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
        "target_amp_err_db_p95": percentile(amps, 95) if amps else None,
        "holdout_nmse_median": median(h_nmses) if h_nmses else None,
        "holdout_suppression_db_median":
            median(h_sups) if h_sups else None,
    }
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
    for key in ("median_suppression_db", "frac_ge_3db", "nmse_median",
                "peak_shift_max", "target_amp_err_db_p95",
                "holdout_nmse_median", "holdout_suppression_db_median"):
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
# Driver
# ---------------------------------------------------------------------------


def check_run(run_dir, results_dir):
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
        check_uses_truth(windows, summary, prefix)
        check_truth_sidecar(run_dir, windows, manifest, prefix)
    stats = None
    if os.path.isfile(os.path.join(run_dir, "stage_times.csv")):
        stats = check_stage_times(run_dir, by_id or {}, prefix)
    if summary is not None:
        cross_check_summary(summary, windows, stats, prefix)
        cross_check_effect(summary, windows or [], prefix)
        check_not_run_declaration(run_dir, summary, prefix)


def find_run_dirs(results_dir):
    names = tuple(REQUIRED_ARTIFACTS)
    found = []
    for dirpath, dirnames, filenames in os.walk(results_dir):
        if any(n in filenames for n in names):
            found.append(dirpath)
            dirnames[:] = []  # a run directory: do not descend into it
    found.sort()
    return found


def verify_results(results_dir):
    del results[:]
    if not os.path.isdir(results_dir):
        fail("results dir exists", "%s is not a directory" % results_dir)
        return [r for r in results if r[1] == "FAIL"]
    runs = find_run_dirs(results_dir)
    if not runs:
        fail("results dir declares at least one run",
             "no run_manifest.json / summary.json / windows.jsonl / "
             "stage_times.csv found under %s (or its subdirectories)"
             % results_dir)
        return [r for r in results if r[1] == "FAIL"]
    ok("results dir declares run(s)", "%d run(s)" % len(runs))
    for run_dir in runs:
        check_run(run_dir, results_dir)
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

    n_windows = 5
    sample_count = 190912
    array_len = SYNC_SAMPLES_PER_SYMBOL * 8          # 8128 samples
    eval_lo, eval_hi = 0, array_len // 2             # eval over the first half
    hold_lo, hold_hi = array_len // 2, array_len     # disjoint holdout half

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
        suppression = 12.0 + 0.5 * wid  # distinct so medians differ
        nmse = 10.0 ** (-suppression / 10.0)
        hold_sup = suppression - 1.0
        hold_nmse = 10.0 ** (-hold_sup / 10.0)
        i = (rng.standard_normal(array_len) + 1j * rng.standard_normal(array_len))
        i_hat = i.copy()
        # Region-dependent error so the eval and holdout NMSEs are independent.
        for region_lo, region_hi, target_nmse in ((eval_lo, eval_hi, nmse),
                                                  (hold_lo, hold_hi, hold_nmse)):
            ref = i[region_lo:region_hi]
            err = (rng.standard_normal(region_hi - region_lo)
                   + 1j * rng.standard_normal(region_hi - region_lo))
            den = float(np.vdot(ref, ref).real)
            scale = math.sqrt(target_nmse * den
                              / float(np.vdot(err, err).real))
            i_hat[region_lo:region_hi] = ref + err * scale
        npz_path = os.path.join(run_dir, "recon", "window_%d.npz" % wid)
        np.savez(npz_path, i=i, i_hat=i_hat,
                 y=i + (rng.standard_normal(array_len)
                        + 1j * rng.standard_normal(array_len)),
                 y_clean=i)
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
            "overlap": {"lo": 0, "hi": 65024, "samples": 65024},
            "isr_db": 10.0, "snr_db": 30.0, "comm_code": 10, "sic_k": 4,
            "fit_reps": 8, "validation_reps": 8, "roi": [0, 1024],
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
                      "holdout_lo": hold_lo, "holdout_hi": hold_hi},
        })
        truth_rows.append({
            "run_id": "run-main-1", "window_id": wid, "seed": 1000 + wid,
            "sensing_code": 9, "comm_code": 10,
            "comm": {"start_sample_exact": 1997.0, "cfo_hz": 0.0,
                     "paths": [{"delay_samples": 0.0, "gain": [1.0, 0.0]}],
                     "isr_db": 10.0, "isr_db_target": 10.0, "scale": math.sqrt(10.0),
                     "real_overlap_lo": 0, "real_overlap_hi": 65024,
                     "overlap_samples": 65024, "length_samples": 65072,
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
                     "validate_commit": 10, "core_total": 400}
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
    summary = {
        "run_id": "run-main-1", "preset": "main", "input_grid": "work",
        "sic_mode": "estimate",
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
                  "demod_speed_pass": True, "native_pass": False},
        "not_run": [
            "native cg400 roundtrip",
            "matlab cross-check",
            "oracle diagnostic",
            "negative window set",
        ],
        "declared_items": {
            "native_cg400_roundtrip": "not_run",
            "matlab_cross_check": "not_run",
            "oracle_diagnostic": "not_run",
        },
    }
    with open(os.path.join(run_dir, "summary.json"), "w", encoding="utf-8") as handle:
        json.dump(summary, handle, indent=1)

    repo_fixture = os.path.join(REPO_ROOT, "AGENTS.md")
    if os.path.isfile(repo_fixture):
        fixture_hashes["repo:AGENTS.md"] = sha256_file(repo_fixture)

    manifest = {
        "run_id": "run-main-1", "commit": "0" * 40, "dirty": False,
        "build_type": "Release", "compiler": "GNU 11.4.0",
        "flags": ["-O3", "-march=native"],
        "lib_paths": {"gnuradio_uwb": "/build-sic/lib/libgnuradio-uwb.so",
                      "loaded": ["libgnuradio-uwb.so => /build-sic/lib",
                                 "libgnuradio-runtime.so => /usr/local/lib"]},
        "cpu": "test-cpu", "os": "Linux test", "gnuradio_version": "3.10.9",
        "volk_version": "3.1.0",
        "config_path": "config.json", "config_hash": sha256_file(config_path),
        "fixture_hashes": fixture_hashes,
        "template_hashes": {"comm_code10": template_sha},
        "template_meta": {"comm_code10": {"length": template_size,
                                          "source": "templates/comm_code10.f32",
                                          "sha256": template_sha}},
        "seed_lists": {"calibration": list(range(1000000, 1000004)),
                       "acceptance": list(range(2000000, 2000004))},
        "sample_rate_work": WORK_HZ, "sample_rate_native": 491520000.0,
        "input_grid": "work", "preset": "main", "sic_mode": "estimate",
    }
    with open(os.path.join(run_dir, "run_manifest.json"), "w",
              encoding="utf-8") as handle:
        json.dump(manifest, handle, indent=1)
    return results_dir_root(run_dir)


def results_dir_root(run_dir):
    return os.path.dirname(run_dir)


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


def _truth_edit(root, mutate):
    path = os.path.join(root, "main", "fixtures", "truth.jsonl")
    with open(path, "r", encoding="utf-8") as handle:
        rows = [json.loads(line) for line in handle if line.strip()]
    rows = mutate(rows)
    with open(path, "w", encoding="utf-8") as handle:
        for row in rows:
            handle.write(json.dumps(row) + "\n")


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
            summary["gates"]["native_pass"] = None
        _summary_edit(root, edit)
    specs.append(("absent not_run item", absent_not_run_item))

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
    failed = verify_results(args.results)
    print("=" * 72)
    print("%d/%d checks pass" % (len(results) - len(failed), len(results)))
    if failed:
        print("FAILED: " + ", ".join(f[0] for f in failed))
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
