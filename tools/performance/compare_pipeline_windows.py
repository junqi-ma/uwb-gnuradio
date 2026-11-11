#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Window-by-window comparison of two UHD SC16 pipeline result dumps.

Dependency-free: Python 3 standard library only.

Contract
--------
* ``docs/performance/接口冻结_UHD_SC16第二轮优化_2026-11-11.md`` §7 (hard
  correctness gates, clause 4: the 11 functional ``WindowResult`` fields must be
  equal window by window; ``worker_id`` and timing fields must NOT be required
  to be equal because scheduling legitimately differs).
* ``docs/performance/OpenCode_UHD_SC16重采样与流水线优化任务书_2026-11-11.md``
  §8 ("重采样 IQ 位级检查必须直接进行，11 个 WindowResult 字段相等不能替代
  IQ 检查").

Input schema (version 1, produced by ``gr-uwb/apps/pipeline_result_json.h``)::

    {"schema": "uwb-pipeline-results", "schema_version": 1,
     "meta": {...},
     "ledger": {"offered_windows": ..., "admitted_windows": ..., ...},
     "windows": [{"window_id": 0, "worker_id": 2, "state": "Completed",
                  "native_start_abs": ..., "native_sample_count": ...,
                  "sic_executed": ..., "sic_committed": ...,
                  "actual_suppression_db": ..., "sic_status": "...",
                  "radar_executed": ..., "radar_sfd_found": ...,
                  "radar_peak_metric": ..., "radar_detected_start": ...,
                  "timing": {...}}, ...]}

Extra keys are tolerated.  A compared key that is absent (or ``null``, or of
the wrong JSON type) is reported as ``unavailable`` and counts as a failure:
"cannot verify" is never silently treated as "equal".

Exit codes
----------
0  every requested check passed
1  at least one check failed (mismatch / unavailable field / ledger break)
2  usage or input-file error (a dump could not be parsed at all)

Per-window IQ files
-------------------
``--iq-dir DIR`` points at a directory containing ``win_<id>_<tag>.cf32``
(complex64, little endian, real/imag float32 pairs) for the run.  The option is
repeatable: the first value is used for dump A, the second for dump B; when it
is given once, dump B falls back to the directory that holds ``B.json``.
``--iq-tag TAG`` pins the ``<tag>`` explicitly; otherwise the tag is discovered
from the files and an ambiguous match is an error (never a guess).

IQ equality is checked BYTE-WISE by default.  ``--iq-tol T`` accepts a pair
whose maximum absolute real/imag difference is ``<= T``; such a pair is
reported as "tolerated", never as byte-identical.
"""

import argparse
import json
import math
import os
import re
import struct
import sys
import tempfile

# ---------------------------------------------------------------------------
# Frozen field set
# ---------------------------------------------------------------------------
#: The 11 functional WindowResult fields.  Order matters for reporting.
FUNCTIONAL_FIELDS = (
    ("state", "str"),
    ("native_start_abs", "int"),
    ("native_sample_count", "int"),
    ("sic_executed", "bool"),
    ("sic_committed", "bool"),
    ("actual_suppression_db", "float"),
    ("sic_status", "str"),
    ("radar_executed", "bool"),
    ("radar_sfd_found", "bool"),
    ("radar_peak_metric", "float"),
    ("radar_detected_start", "int"),
)
FIELD_SPECS = dict(FUNCTIONAL_FIELDS)

#: ``worker_id`` is compared only with ``--strict-worker-id``.
WORKER_FIELD = ("worker_id", "int")

#: Timing fields are informational only; they are never required to match.
TIMING_FIELDS = (
    "slice_us",
    "queue_us",
    "convert_us",
    "resample_us",
    "sic_us",
    "sensing_us",
    "e2e_us",
)

TERMINAL_STATES = (
    "Completed",
    "ProcessingFailed",
    "DroppedAdmission",
    "InvalidGap",
    "Cancelled",
)

LEDGER_FIELDS = (
    "offered_windows",
    "admitted_windows",
    "completed_windows",
    "processing_failed",
    "dropped_admission",
    "invalid_gap_before_admission",
    "invalid_gap_after_admission",
    "cancelled_windows",
    "total_samples_received",
    "short_reads",
    "read_timeouts",
    "read_overflows",
)

_MISSING = object()


# ---------------------------------------------------------------------------
# Reporting
# ---------------------------------------------------------------------------
class Report(object):
    """Collects problems while streaming a human-readable report."""

    def __init__(self, quiet=False):
        self.problems = []
        self.warnings = []
        self.quiet = quiet
        self.first_mismatch = None  # (kind, window_id, field, detail)

    def line(self, text=""):
        if not self.quiet:
            print(text)

    def section(self, title):
        self.line("")
        self.line("=" * 78)
        self.line(title)
        self.line("=" * 78)

    def problem(self, text):
        self.problems.append(text)
        if not self.quiet:
            print("  FAIL: " + text)

    def warn(self, text):
        self.warnings.append(text)
        if not self.quiet:
            print("  warn: " + text)

    def note(self, text):
        if not self.quiet:
            print("  note: " + text)

    @property
    def ok(self):
        return not self.problems


# ---------------------------------------------------------------------------
# Loading
# ---------------------------------------------------------------------------
class Dump(object):
    def __init__(self, path):
        self.path = path
        self.raw = None
        self.meta = None
        self.ledger = None
        self.windows = []
        self.by_id = {}
        self.duplicate_ids = []
        self.order = []

    def load(self):
        with open(self.path, "r", encoding="utf-8") as fh:
            obj = json.load(fh)
        return self.parse(obj)

    def parse(self, obj):
        if not isinstance(obj, dict):
            raise ValueError("%s: top-level JSON value is not an object" % self.path)
        windows = obj.get("windows")
        if not isinstance(windows, list):
            raise ValueError("%s: missing or non-list 'windows'" % self.path)
        self.raw = obj
        meta = obj.get("meta")
        self.meta = meta if isinstance(meta, dict) else None
        ledger = obj.get("ledger")
        self.ledger = ledger if isinstance(ledger, dict) else None
        for idx, entry in enumerate(windows):
            if not isinstance(entry, dict):
                raise ValueError("%s: windows[%d] is not an object" % (self.path, idx))
            wid = entry.get("window_id")
            if isinstance(wid, bool) or not isinstance(wid, int):
                raise ValueError(
                    "%s: windows[%d] has no integer 'window_id'" % (self.path, idx)
                )
            if wid in self.by_id:
                self.duplicate_ids.append(wid)
                continue
            self.by_id[wid] = entry
        self.windows = [w for w in windows if isinstance(w, dict)]
        self.order = [w.get("window_id") for w in self.windows]
        return self


def describe_file(path):
    try:
        st = os.stat(path)
    except OSError as exc:
        return "%s (stat failed: %s)" % (path, exc)
    return "%s (%d bytes, mtime %s)" % (
        path,
        st.st_size,
        _fmt_mtime(st.st_mtime),
    )


def _fmt_mtime(ts):
    import datetime

    return datetime.datetime.fromtimestamp(ts).isoformat(timespec="seconds")


# ---------------------------------------------------------------------------
# Value handling
# ---------------------------------------------------------------------------
def coerce_value(value, kind):
    """Return ``(ok, normalized)``; ``ok is False`` means 'unavailable'."""
    if value is _MISSING or value is None:
        return False, None
    if kind == "str":
        return (True, value) if isinstance(value, str) else (False, None)
    if kind == "bool":
        return (True, value) if isinstance(value, bool) else (False, None)
    if kind == "int":
        if isinstance(value, bool):
            return False, None
        if isinstance(value, int):
            return True, value
        return False, None
    if kind == "float":
        if isinstance(value, bool):
            return False, None
        if isinstance(value, (int, float)):
            if isinstance(value, float) and not math.isfinite(value):
                return False, None
            return True, float(value)
        return False, None
    return False, None


def fmt_value(kind, value):
    if value is None:
        return "null"
    if kind == "str":
        return json.dumps(value, ensure_ascii=False)
    if kind == "bool":
        return "true" if value else "false"
    if kind == "int":
        return str(value)
    return "%.9g" % value


# ---------------------------------------------------------------------------
# Comparison helpers
# ---------------------------------------------------------------------------
def compare_id_sets(dump_a, dump_b, rep):
    ids_a = set(dump_a.by_id)
    ids_b = set(dump_b.by_id)
    rep.section("[1] Window inventory")
    rep.line("  A: %s" % describe_file(dump_a.path))
    rep.line("      %d window records, %d unique ids" % (len(dump_a.windows), len(ids_a)))
    rep.line("  B: %s" % describe_file(dump_b.path))
    rep.line("      %d window records, %d unique ids" % (len(dump_b.windows), len(ids_b)))

    for label, dump in (("A", dump_a), ("B", dump_b)):
        if dump.duplicate_ids:
            rep.problem(
                "%s: duplicate window_id(s) %s -- a window must have exactly one "
                "terminal state" % (label, sorted(set(dump.duplicate_ids))[:10])
            )
        ascending = all(
            isinstance(a, int) and isinstance(b, int) and a < b
            for a, b in zip(dump.order, dump.order[1:])
        )
        if not ascending:
            rep.problem(
                "%s: window records are not in strictly ascending window_id order "
                "(the pipeline commits in order; out-of-order output is a bug)" % label
            )
        else:
            rep.note("%s: window_id strictly ascending" % label)

    only_a = sorted(ids_a - ids_b)
    only_b = sorted(ids_b - ids_a)
    if only_a:
        rep.problem("A-only window_id(s): %s" % _short_list(only_a))
    if only_b:
        rep.problem("B-only window_id(s): %s" % _short_list(only_b))
    if ids_a and ids_b:
        rep.line(
            "  id range: A [%d..%d], B [%d..%d]"
            % (min(ids_a), max(ids_a), min(ids_b), max(ids_b))
        )
    if not only_a and not only_b:
        rep.note("window_id sets identical (%d ids)" % len(ids_a))
    return sorted(ids_a & ids_b)


def _short_list(values, limit=12):
    shown = ", ".join(str(v) for v in values[:limit])
    if len(values) > limit:
        shown += ", ... (%d total)" % len(values)
    return shown


def compare_fields(dump_a, dump_b, common_ids, rep, float_tol, strict_worker):
    rep.section("[2] Functional WindowResult fields")
    if float_tol > 0.0:
        rep.warn(
            "--float-tol=%g is active: float fields are accepted within tolerance. "
            "The frozen gate is exact equality; a tolerated difference is NOT "
            "bit-equality." % float_tol
        )
        rep.line("  float tolerance: %g (non-zero)" % float_tol)
    else:
        rep.line("  float tolerance: 0 (exact value equality)")
    rep.line("  worker_id: %s" % ("compared (--strict-worker-id)" if strict_worker else "not compared"))

    specs = list(FUNCTIONAL_FIELDS)
    if strict_worker:
        specs.append(WORKER_FIELD)

    mismatch = {name: 0 for name, _ in specs}
    unavailable = {name: 0 for name, _ in specs}
    max_delta = {name: 0.0 for name, _ in specs}
    first = None

    for wid in common_ids:
        wa = dump_a.by_id[wid]
        wb = dump_b.by_id[wid]
        for name, kind in specs:
            oka, va = coerce_value(wa.get(name, _MISSING), kind)
            okb, vb = coerce_value(wb.get(name, _MISSING), kind)
            if not (oka and okb):
                unavailable[name] += 1
                if first is None:
                    first = (
                        wid,
                        name,
                        "unavailable",
                        "A=%s B=%s"
                        % (
                            "missing" if not oka else fmt_value(kind, va),
                            "missing" if not okb else fmt_value(kind, vb),
                        ),
                    )
                continue
            if kind == "float":
                delta = abs(va - vb)
                if delta > max_delta[name]:
                    max_delta[name] = delta
                equal = delta == 0.0 if float_tol == 0.0 else delta <= float_tol
            else:
                equal = va == vb
            if not equal:
                mismatch[name] += 1
                if first is None:
                    first = (
                        wid,
                        name,
                        "mismatch",
                        "A=%s B=%s" % (fmt_value(kind, va), fmt_value(kind, vb)),
                    )

    header = "  %-24s %10s %12s %14s" % ("field", "mismatch", "unavailable", "max|delta|")
    rep.line(header)
    rep.line("  " + "-" * (len(header) - 2))
    for name, kind in specs:
        delta = "-" if kind != "float" else "%.9g" % max_delta[name]
        rep.line(
            "  %-24s %10d %12d %14s"
            % (name, mismatch[name], unavailable[name], delta)
        )

    total_mismatch = sum(mismatch.values())
    total_unavailable = sum(unavailable.values())
    if first is not None:
        wid, name, what, detail = first
        rep.first_mismatch = (wid, name, what, detail)
        rep.problem(
            "first differing window: window_id=%d field=%s (%s) %s"
            % (wid, name, what, detail)
        )
    if total_mismatch or total_unavailable:
        rep.problem(
            "%d functional-field mismatch(es) and %d unavailable field(s) over "
            "%d common windows" % (total_mismatch, total_unavailable, len(common_ids))
        )
    else:
        rep.note(
            "all %d compared fields equal on %d windows"
            % (len(specs), len(common_ids))
        )
    if not common_ids:
        if len(dump_a.windows) == 0 and len(dump_b.windows) == 0:
            rep.warn(
                "both dumps contain zero windows: there is nothing to compare "
                "(this is not evidence of equality)"
            )
        else:
            rep.problem("no common window ids: nothing was compared")
    return {
        "mismatch": mismatch,
        "unavailable": unavailable,
        "max_delta": max_delta,
        "comparisons": len(common_ids) * len(specs),
    }


def _state_counts(dump):
    counts = {}
    for entry in dump.windows:
        state = entry.get("state")
        key = state if isinstance(state, str) else "<unavailable>"
        counts[key] = counts.get(key, 0) + 1
    return counts


def check_ledger(dump, label, rep):
    rep.line("")
    rep.line("  %s ledger (%s)" % (label, dump.path))
    if dump.ledger is None:
        rep.problem("%s: no 'ledger' object -- conservation cannot be verified" % label)
        return None
    ledger = dump.ledger
    missing = [f for f in LEDGER_FIELDS if isinstance(ledger.get(f), bool) or not isinstance(ledger.get(f), int)]
    if missing:
        rep.warn(
            "%s: ledger field(s) missing/non-integer: %s" % (label, ", ".join(missing))
        )
    rep.line(
        "    offered=%s admitted=%s completed=%s failed=%s dropped=%s "
        "invalid_before=%s invalid_after=%s cancelled=%s"
        % tuple(ledger.get(f, "?") for f in LEDGER_FIELDS[:8])
    )
    rep.line(
        "    samples_received=%s short_reads=%s read_timeouts=%s read_overflows=%s"
        % tuple(ledger.get(f, "?") for f in LEDGER_FIELDS[8:])
    )

    def get(name):
        value = ledger.get(name)
        if isinstance(value, bool) or not isinstance(value, int):
            return None
        return value

    offered = get("offered_windows")
    admitted = get("admitted_windows")
    completed = get("completed_windows")
    failed = get("processing_failed")
    dropped = get("dropped_admission")
    gap_before = get("invalid_gap_before_admission")
    gap_after = get("invalid_gap_after_admission")
    cancelled = get("cancelled_windows")

    if None not in (offered, admitted, dropped, gap_before, cancelled):
        expected = admitted + dropped + gap_before + cancelled
        if offered != expected:
            rep.problem(
                "%s: ledger invariant 1 broken: offered(%d) != admitted(%d) + "
                "dropped(%d) + invalid_gap_before(%d) + cancelled(%d) = %d"
                % (label, offered, admitted, dropped, gap_before, cancelled, expected)
            )
        else:
            rep.note("%s: ledger invariant 1 holds (offered == sum of outlets)" % label)

    if None not in (admitted, completed, failed, gap_after):
        expected = completed + failed + gap_after
        if admitted != expected:
            rep.problem(
                "%s: ledger invariant 2 broken: admitted(%d) != completed(%d) + "
                "failed(%d) + invalid_gap_after(%d) = %d"
                % (label, admitted, completed, failed, gap_after, expected)
            )
        else:
            rep.note("%s: ledger invariant 2 holds (admitted == completed + failed)" % label)

    # Drop accounting: results() carries drops, so the window table and the
    # ledger must agree on every state that the pipeline emits.
    counts = _state_counts(dump)
    rep.line(
        "    results by state: %s"
        % ", ".join("%s=%d" % (k, v) for k, v in sorted(counts.items()))
    )
    if "<unavailable>" in counts:
        rep.problem(
            "%s: %d window record(s) have no usable 'state'" % (label, counts["<unavailable>"])
        )

    pairs = (
        ("Completed", completed, "completed_windows"),
        ("DroppedAdmission", dropped, "dropped_admission"),
        ("ProcessingFailed", failed, "processing_failed"),
        ("InvalidGap", None, "invalid_gap_*"),
    )
    for state, ledger_value, name in pairs:
        observed = counts.get(state, 0)
        if ledger_value is None:
            if name == "invalid_gap_*":
                gap_total = (gap_before or 0) + (gap_after or 0)
                if observed != gap_total:
                    rep.warn(
                        "%s: %d result(s) with state=%s but ledger invalid_gap total=%d "
                        "(cancelled windows are intentionally absent from results())"
                        % (label, observed, state, gap_total)
                    )
            continue
        if observed != ledger_value:
            rep.problem(
                "%s: %d result(s) with state=%s but ledger %s=%d"
                % (label, observed, state, name, ledger_value)
            )

    if None not in (admitted, dropped, gap_before):
        expected_results = admitted + dropped + gap_before
        if len(dump.windows) != expected_results:
            rep.problem(
                "%s: %d window records but admitted+dropped+invalid_gap_before=%d "
                "(every admitted window must emit exactly one result)"
                % (label, len(dump.windows), expected_results)
            )
        else:
            rep.note(
                "%s: result count equals admitted+dropped+invalid_gap_before (%d)"
                % (label, expected_results)
            )
    if cancelled:
        rep.note(
            "%s: cancelled_windows=%d are not emitted in results() by contract "
            "(pre-admission cancellation)" % (label, cancelled)
        )
    return ledger


def compare_ledgers(ledger_a, ledger_b, rep):
    if not isinstance(ledger_a, dict) or not isinstance(ledger_b, dict):
        return
    deltas = []
    for field in LEDGER_FIELDS:
        va = ledger_a.get(field)
        vb = ledger_b.get(field)
        if isinstance(va, int) and isinstance(vb, int) and not isinstance(va, bool) and va != vb:
            deltas.append("%s: A=%d B=%d (delta %+d)" % (field, va, vb, vb - va))
    if deltas:
        rep.note("ledger differences between A and B (explain them, do not hide them):")
        for text in deltas:
            rep.line("      " + text)
    else:
        rep.note("ledger counters identical between A and B")


def compare_timing(dump_a, dump_b, common_ids, rep):
    """Informational only: timing must never gate functional equality."""
    rep.section("[4] Timing context (NOT compared; different scheduling is legal)")
    rows = []
    for field in TIMING_FIELDS:
        vals_a = [_timing_value(dump_a.by_id[w], field) for w in common_ids]
        vals_b = [_timing_value(dump_b.by_id[w], field) for w in common_ids]
        mean_a = _mean([v for v in vals_a if isinstance(v, (int, float)) and not isinstance(v, bool)])
        mean_b = _mean([v for v in vals_b if isinstance(v, (int, float)) and not isinstance(v, bool)])
        if mean_a is None and mean_b is None:
            continue
        rows.append((field, mean_a, mean_b))
    if not rows:
        rep.note("no timing data present in either dump")
        return
    rep.line("  %-14s %14s %14s" % ("field (us)", "A mean", "B mean"))
    for field, mean_a, mean_b in rows:
        rep.line(
            "  %-14s %14s %14s"
            % (
                field,
                "-" if mean_a is None else "%.3f" % mean_a,
                "-" if mean_b is None else "%.3f" % mean_b,
            )
        )
    rep.note("worker_id distribution: A=%s B=%s" % (
        _worker_histogram(dump_a, common_ids),
        _worker_histogram(dump_b, common_ids),
    ))


def _mean(values):
    if not values:
        return None
    return float(sum(values)) / float(len(values))


def _timing_value(entry, field):
    timing = entry.get("timing")
    if not isinstance(timing, dict):
        return None
    return timing.get(field)


def _worker_histogram(dump, ids):
    counts = {}
    for wid in ids:
        w = dump.by_id[wid].get("worker_id")
        key = str(w) if isinstance(w, int) and not isinstance(w, bool) else "?"
        counts[key] = counts.get(key, 0) + 1
    return "{" + ", ".join("%s:%d" % (k, counts[k]) for k in sorted(counts)) + "}"


# ---------------------------------------------------------------------------
# Per-window IQ comparison
# ---------------------------------------------------------------------------
IQ_NAME_RE = re.compile(r"^win_(\d+)_(.+)\.cf32$")


def index_iq_dir(dirpath):
    """Map window_id -> [(path, tag), ...] for one directory."""
    table = {}
    try:
        entries = list(os.scandir(dirpath))
    except OSError as exc:
        return None, "cannot list %s: %s" % (dirpath, exc)
    for entry in entries:
        if not entry.is_file():
            continue
        match = IQ_NAME_RE.match(entry.name)
        if not match:
            continue
        table.setdefault(int(match.group(1)), []).append((entry.path, match.group(2)))
    return table, None


def _read_all(path):
    with open(path, "rb") as fh:
        return fh.read()


def compare_iq_bytes(path_a, path_b, tol):
    """Byte-wise (default) or tolerance-limited comparison of two cf32 files."""
    data_a = _read_all(path_a)
    data_b = _read_all(path_b)
    result = {
        "path_a": path_a,
        "path_b": path_b,
        "bytes_a": len(data_a),
        "bytes_b": len(data_b),
        "bytes_equal": data_a == data_b,
        "size_mismatch": len(data_a) != len(data_b),
        "max_abs": 0.0,
        "first_diff_sample": None,
        "first_diff_values": None,
        "compared_samples": 0,
        "differing_values": 0,
        "within_tol": False,
        "error": None,
    }
    if result["size_mismatch"]:
        return result
    if len(data_a) % 8 != 0:
        result["error"] = "size %d is not a multiple of 8 bytes (complex64)" % len(data_a)
        return result
    n_samples = len(data_a) // 8
    result["compared_samples"] = n_samples
    if result["bytes_equal"]:
        result["within_tol"] = True
        return result

    n_floats = 2 * n_samples
    floats_a = struct.unpack("<%df" % n_floats, data_a)
    floats_b = struct.unpack("<%df" % n_floats, data_b)
    max_abs = 0.0
    differing = 0
    first_idx = None
    for i in range(n_floats):
        fa = floats_a[i]
        fb = floats_b[i]
        if not (math.isfinite(fa) and math.isfinite(fb)):
            delta = float("inf")
        else:
            delta = abs(fa - fb)
        if delta != 0.0:
            differing += 1
            if first_idx is None:
                first_idx = i
            if delta > max_abs:
                max_abs = delta
    result["max_abs"] = max_abs
    result["differing_values"] = differing
    if first_idx is not None:
        result["first_diff_sample"] = first_idx // 2
        result["first_diff_values"] = (floats_a[first_idx], floats_b[first_idx])
    result["within_tol"] = max_abs <= tol
    return result


def compare_iq(dump_a, dump_b, common_ids, rep, args):
    rep.section("[5] Per-window IQ (direct sample check)")
    if not args.iq_dir:
        rep.warn(
            "no --iq-dir given: the direct IQ check was NOT performed. "
            "11-field equality alone must never be presented as proof that the "
            "resampled IQ is identical."
        )
        return None

    dir_a = args.iq_dir[0]
    dir_b = args.iq_dir[1] if len(args.iq_dir) > 1 else os.path.dirname(os.path.abspath(dump_b.path))
    rep.line("  A IQ dir: %s" % dir_a)
    rep.line("  B IQ dir: %s" % dir_b)
    rep.line(
        "  tolerance: %s"
        % ("0 (byte-wise exact)" if args.iq_tol == 0.0 else "%g" % args.iq_tol)
    )

    table_a, err_a = index_iq_dir(dir_a)
    table_b, err_b = index_iq_dir(dir_b)
    if err_a:
        rep.problem("A: " + err_a)
        return None
    if err_b:
        rep.problem("B: " + err_b)
        return None
    if not table_a and not table_b:
        rep.problem(
            "no win_<id>_<tag>.cf32 files found in either IQ directory: the "
            "mandatory IQ check cannot be performed"
        )
        return None

    compared = 0
    skipped = []
    unavailable = 0
    tolerated = 0
    max_abs_overall = 0.0
    first_iq_mismatch = None
    byte_equal_count = 0

    for wid in common_ids:
        state_a = dump_a.by_id[wid].get("state")
        state_b = dump_b.by_id[wid].get("state")
        if state_a != "Completed" or state_b != "Completed":
            skipped.append((wid, state_a, state_b))
            continue
        pa = _resolve_iq_path(table_a, wid, args.iq_tag)
        pb = _resolve_iq_path(table_b, wid, args.iq_tag)
        if pa is None or pb is None:
            unavailable += 1
            rep.problem(
                "window %d: IQ file %s"
                % (
                    wid,
                    "missing for A" if pa is None else "missing for B",
                )
            )
            continue
        if isinstance(pa, list) or isinstance(pb, list):
            unavailable += 1
            rep.problem(
                "window %d: ambiguous win_<id>_<tag>.cf32 match (%s); pass --iq-tag"
                % (wid, "A" if isinstance(pa, list) else "B")
            )
            continue

        result = compare_iq_bytes(pa, pb, args.iq_tol)
        compared += 1
        if result["error"]:
            rep.problem("window %d: %s" % (wid, result["error"]))
            continue
        if result["size_mismatch"]:
            rep.problem(
                "window %d: IQ size mismatch A=%d bytes B=%d bytes"
                % (wid, result["bytes_a"], result["bytes_b"])
            )
            continue
        if result["bytes_equal"]:
            byte_equal_count += 1
            continue
        max_abs_overall = max(max_abs_overall, result["max_abs"])
        if result["within_tol"]:
            tolerated += 1
            rep.warn(
                "window %d: IQ differs byte-wise but max|delta|=%.9g <= tol %g "
                "(NOT byte-identical)"
                % (wid, result["max_abs"], args.iq_tol)
            )
            continue
        if first_iq_mismatch is None:
            first_iq_mismatch = wid
        detail = "max|delta|=%.9g" % result["max_abs"]
        if result["first_diff_sample"] is not None:
            detail += ", first differing sample %d (A=%s B=%s), %d differing float(s) of %d" % (
                result["first_diff_sample"],
                _fmt_float(result["first_diff_values"][0]),
                _fmt_float(result["first_diff_values"][1]),
                result["differing_values"],
                2 * result["compared_samples"],
            )
        rep.problem("window %d: IQ mismatch: %s" % (wid, detail))

    rep.line(
        "  compared=%d byte-identical=%d tolerated=%d unavailable=%d skipped(non-Completed)=%d"
        % (compared, byte_equal_count, tolerated, unavailable, len(skipped))
    )
    if skipped:
        rep.note(
            "skipped window ids (no DSP output expected): %s"
            % _short_list(["%d(%s/%s)" % (w, sa, sb) for w, sa, sb in skipped])
        )
    if compared:
        rep.note("max|delta| over all compared IQ: %.9g" % max_abs_overall)
    if first_iq_mismatch is not None:
        rep.problem("first IQ-mismatching window: %d" % first_iq_mismatch)
    if compared == 0 and unavailable == 0 and common_ids:
        rep.problem("no IQ pair could be compared (no Completed window in common)")
    return {
        "compared": compared,
        "byte_identical": byte_equal_count,
        "tolerated": tolerated,
        "unavailable": unavailable,
        "max_abs": max_abs_overall,
        "first_mismatch_window": first_iq_mismatch,
    }


def _resolve_iq_path(table, wid, tag):
    """Return a path, a list (ambiguous) or None (missing)."""
    entries = table.get(wid, [])
    if tag is not None:
        for path, entry_tag in entries:
            if entry_tag == tag:
                return path
        return None
    if len(entries) == 1:
        return entries[0][0]
    if not entries:
        return None
    return [p for p, _ in entries]


def _fmt_float(value):
    if value is None:
        return "null"
    return "%.9g" % value


# ---------------------------------------------------------------------------
# Top level comparison
# ---------------------------------------------------------------------------
def compare_dumps(dump_a, dump_b, rep, args):
    common = compare_id_sets(dump_a, dump_b, rep)

    compare_fields(dump_a, dump_b, common, rep, args.float_tol, args.strict_worker_id)

    rep.section("[3] Ledger conservation and drop accounting")
    ledger_a = check_ledger(dump_a, "A", rep)
    ledger_b = check_ledger(dump_b, "B", rep)
    compare_ledgers(ledger_a, ledger_b, rep)

    compare_timing(dump_a, dump_b, common, rep)
    compare_iq(dump_a, dump_b, common, rep, args)
    return common


def run_cli(args):
    rep = Report()
    rep.line("compare_pipeline_windows.py")
    rep.line("  contract: 11 functional WindowResult fields must be equal window by window")
    rep.line("            worker_id/timing are scheduling-dependent and are not gated")
    try:
        dump_a = Dump(args.dump_a).load()
        dump_b = Dump(args.dump_b).load()
    except (OSError, ValueError) as exc:
        print("ERROR: %s" % exc, file=sys.stderr)
        return 2

    for label, dump in (("A", dump_a), ("B", dump_b)):
        if dump.meta:
            keys = ", ".join(sorted(str(k) for k in dump.meta.keys()))
            rep.line("  %s meta keys: %s" % (label, keys))

    compare_dumps(dump_a, dump_b, rep, args)

    rep.section("RESULT")
    if rep.problems:
        print("  FAIL (%d problem(s))" % len(rep.problems))
        for text in rep.problems:
            print("    - " + text)
        if rep.first_mismatch:
            print(
                "  first functional mismatch: window_id=%d field=%s (%s) %s"
                % rep.first_mismatch
            )
        return 1
    print("  PASS")
    if rep.warnings:
        print("  warnings (%d):" % len(rep.warnings))
        for text in rep.warnings:
            print("    - " + text)
    return 0


# ---------------------------------------------------------------------------
# Self-test
# ---------------------------------------------------------------------------
def _synthetic_dump(count=6, tag="self-test"):
    windows = []
    for wid in range(count):
        windows.append(
            {
                "window_id": wid,
                "worker_id": wid % 4,
                "state": "Completed",
                "native_start_abs": 1000 * wid,
                "native_sample_count": 800,
                "sic_executed": True,
                "sic_committed": True,
                "actual_suppression_db": 20.0 + wid,
                "sic_status": "committed" if wid % 2 == 0 else "ok",
                "radar_executed": True,
                "radar_sfd_found": True,
                "radar_peak_metric": 1.5 + 0.25 * wid,
                "radar_detected_start": 100 + wid,
                "timing": {
                    "slice_us": 1.0 * wid,
                    "queue_us": 2.0 * wid,
                    "convert_us": 0.0,
                    "resample_us": 3.0 * wid,
                    "sic_us": 4.0 * wid,
                    "sensing_us": 5.0 * wid,
                    "e2e_us": 6.0 * wid,
                },
            }
        )
    return {
        "schema": "uwb-pipeline-results",
        "schema_version": 1,
        "meta": {"tag": tag},
        "ledger": {
            "offered_windows": count,
            "admitted_windows": count,
            "completed_windows": count,
            "processing_failed": 0,
            "dropped_admission": 0,
            "invalid_gap_before_admission": 0,
            "invalid_gap_after_admission": 0,
            "cancelled_windows": 0,
            "total_samples_received": 800 * count,
            "short_reads": 0,
            "read_timeouts": 0,
            "read_overflows": 0,
        },
        "windows": windows,
    }


def _write_json(path, obj):
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(obj, fh, ensure_ascii=False)


def _write_cf32(path, samples):
    with open(path, "wb") as fh:
        for value in samples:
            fh.write(struct.pack("<ff", value[0], value[1]))


def _cmp(dump_a, dump_b, iq_dirs=None, iq_tol=0.0, strict=False, float_tol=0.0):
    args = argparse.Namespace(
        iq_dir=list(iq_dirs or []),
        iq_tag=None,
        iq_tol=iq_tol,
        strict_worker_id=strict,
        float_tol=float_tol,
    )
    rep = Report(quiet=True)
    dump_a = Dump("<memory-A>").parse(dump_a)
    dump_b = Dump("<memory-B>").parse(dump_b)
    compare_dumps(dump_a, dump_b, rep, args)
    return rep


def run_self_test():
    import copy

    failures = []

    def check(name, condition, detail=""):
        if condition:
            print("  [self-test] PASS %s" % name)
        else:
            print("  [self-test] FAIL %s %s" % (name, detail))
            failures.append(name)

    base = _synthetic_dump()

    # 1. Identical dumps pass.
    rep = _cmp(base, copy.deepcopy(base))
    check("identical dumps accepted", rep.ok, "problems=%r" % (rep.problems,))

    # 2. One injected single-field mismatch on window 3 is detected.
    mutated = copy.deepcopy(base)
    mutated["windows"][3]["radar_peak_metric"] += 0.5
    rep = _cmp(base, mutated)
    check("single-field mismatch rejected", not rep.ok)
    check(
        "single-field mismatch localised",
        rep.first_mismatch is not None
        and rep.first_mismatch[0] == 3
        and rep.first_mismatch[1] == "radar_peak_metric",
        "%r" % (rep.first_mismatch,),
    )

    # 3. A missing optional field is 'unavailable', never silently equal.
    mutated = copy.deepcopy(base)
    del mutated["windows"][2]["sic_status"]
    rep = _cmp(base, mutated)
    check("missing field rejected", not rep.ok)
    check(
        "missing field reported as unavailable",
        rep.first_mismatch is not None
        and rep.first_mismatch[0] == 2
        and rep.first_mismatch[1] == "sic_status"
        and rep.first_mismatch[2] == "unavailable",
        "%r" % (rep.first_mismatch,),
    )

    # 4. A broken ledger invariant is detected even when windows match.
    mutated = copy.deepcopy(base)
    mutated["ledger"]["offered_windows"] += 1
    rep = _cmp(base, mutated)
    check("ledger invariant break rejected", not rep.ok,
          "problems=%r" % (rep.problems,))

    # 5. A worker_id difference is accepted by default and rejected in strict mode.
    mutated = copy.deepcopy(base)
    mutated["windows"][1]["worker_id"] = 3
    rep = _cmp(base, mutated)
    check("worker_id ignored by default", rep.ok, "problems=%r" % (rep.problems,))
    rep = _cmp(base, mutated, strict=True)
    check("worker_id gated with --strict-worker-id", not rep.ok)

    # 6. Timing differences are never a failure.
    mutated = copy.deepcopy(base)
    for entry in mutated["windows"]:
        entry["timing"]["resample_us"] = 999.0
    rep = _cmp(base, mutated)
    check("timing differences ignored", rep.ok, "problems=%r" % (rep.problems,))

    # 7. IQ: identical files pass, a single-sample change is detected.
    with tempfile.TemporaryDirectory(prefix="cmp-self-test-") as root:
        dir_a = os.path.join(root, "a")
        dir_b = os.path.join(root, "b")
        os.makedirs(dir_a)
        os.makedirs(dir_b)
        samples = [(0.5 * i, -0.25 * i) for i in range(64)]
        for wid in range(len(base["windows"])):
            _write_cf32(os.path.join(dir_a, "win_%d_runA.cf32" % wid), samples)
            _write_cf32(os.path.join(dir_b, "win_%d_runB.cf32" % wid), samples)

        rep = _cmp(base, copy.deepcopy(base), iq_dirs=[dir_a, dir_b])
        check("IQ identical accepted", rep.ok, "problems=%r" % (rep.problems,))

        # Perturb exactly one sample (index 17) of window 2's B-side file.
        perturbed = list(samples)
        perturbed[17] = (perturbed[17][0] + 1.0e-3, perturbed[17][1])
        _write_cf32(os.path.join(dir_b, "win_2_runB.cf32"), perturbed)
        rep = _cmp(base, copy.deepcopy(base), iq_dirs=[dir_a, dir_b])
        check("single-sample IQ mismatch rejected", not rep.ok)
        iq_problem = [p for p in rep.problems if "IQ mismatch" in p]
        check(
            "single-sample IQ mismatch localised",
            len(iq_problem) == 1 and "window 2" in iq_problem[0] and "sample 17" in iq_problem[0],
            repr(iq_problem),
        )

        # A tiny (1-ulp) difference is still caught at tol=0 ...
        tiny = list(samples)
        bits = struct.unpack("<I", struct.pack("<f", tiny[5][1]))[0]
        tiny[5] = (tiny[5][0], struct.unpack("<f", struct.pack("<I", bits + 1))[0])
        _write_cf32(os.path.join(dir_b, "win_2_runB.cf32"), tiny)
        rep = _cmp(base, copy.deepcopy(base), iq_dirs=[dir_a, dir_b])
        check("1-ulp IQ difference rejected at tol=0", not rep.ok)
        # ... and tolerated when an explicit non-zero tolerance is requested.
        rep = _cmp(base, copy.deepcopy(base), iq_dirs=[dir_a, dir_b], iq_tol=1.0e-6)
        check(
            "1-ulp IQ difference tolerated with --iq-tol",
            rep.ok and rep.warnings,
            "problems=%r warnings=%r" % (rep.problems, rep.warnings),
        )

        # Missing IQ file for a Completed window is a failure, not a skip.
        os.remove(os.path.join(dir_b, "win_4_runB.cf32"))
        rep = _cmp(base, copy.deepcopy(base), iq_dirs=[dir_a, dir_b])
        check("missing IQ file rejected", not rep.ok)

        # A dump pair without --iq-dir warns but does not pretend to have checked.
        rep = _cmp(base, copy.deepcopy(base), iq_dirs=[])
        check(
            "absent --iq-dir surfaces a warning",
            any("IQ check was NOT performed" in w for w in rep.warnings),
            "warnings=%r" % (rep.warnings,),
        )

    # 8. Drop accounting: a consistent drop ledger is accepted, an inconsistent
    #    one is rejected (drops are real terminal states, not missing rows).
    drop = _synthetic_dump(4, "drops")
    drop["ledger"].update(
        {"admitted_windows": 2, "dropped_admission": 2, "completed_windows": 2}
    )
    for wid in (1, 3):
        entry = drop["windows"][wid]
        entry["state"] = "DroppedAdmission"
        entry["worker_id"] = -1
        entry["sic_executed"] = False
        entry["sic_committed"] = False
        entry["actual_suppression_db"] = 0.0
        entry["sic_status"] = ""
        entry["radar_executed"] = False
        entry["radar_sfd_found"] = False
        entry["radar_peak_metric"] = 0.0
        entry["radar_detected_start"] = -1
    rep = _cmp(drop, copy.deepcopy(drop))
    check("consistent drop ledger accepted", rep.ok, "problems=%r" % (rep.problems,))
    bad = copy.deepcopy(drop)
    bad["ledger"].update(
        {"admitted_windows": 4, "dropped_admission": 0, "completed_windows": 4}
    )
    rep = _cmp(drop, bad)
    check("drop accounting mismatch rejected", not rep.ok)

    # 9. A window present in only one dump is a failure (id sets must match).
    mutated = copy.deepcopy(base)
    mutated["windows"] = mutated["windows"][:-1]
    mutated["ledger"]["offered_windows"] -= 1
    mutated["ledger"]["admitted_windows"] -= 1
    mutated["ledger"]["completed_windows"] -= 1
    rep = _cmp(base, mutated)
    check("window-id set mismatch rejected", not rep.ok)

    print("")
    if failures:
        print("SELF-TEST: FAIL (%d check(s)): %s" % (len(failures), ", ".join(failures)))
        return 1
    print("SELF-TEST: PASS (all checks)")
    return 0


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def build_parser():
    parser = argparse.ArgumentParser(
        prog="compare_pipeline_windows.py",
        description=(
            "Compare two UHD SC16 pipeline result dumps window by window "
            "(11 functional WindowResult fields, ledger conservation, "
            "drop accounting and optional per-window IQ)."
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "examples:\n"
            "  compare_pipeline_windows.py A.json B.json\n"
            "  compare_pipeline_windows.py A.json B.json --iq-dir runA --iq-dir runB\n"
            "  compare_pipeline_windows.py A.json B.json --iq-dir runA --iq-dir runB "
            "--iq-tol 1e-9\n"
            "  compare_pipeline_windows.py --self-test\n"
        ),
    )
    parser.add_argument("dump_a", nargs="?", help="baseline/reference JSON dump")
    parser.add_argument("dump_b", nargs="?", help="candidate JSON dump")
    parser.add_argument(
        "--iq-dir",
        action="append",
        default=[],
        metavar="DIR",
        help="directory with win_<id>_<tag>.cf32 files; repeat once per dump "
        "(first = A, second = B). Defaults to the dump's own directory.",
    )
    parser.add_argument(
        "--iq-tag",
        default=None,
        help="explicit <tag> for win_<id>_<tag>.cf32 (otherwise auto-discovered)",
    )
    parser.add_argument(
        "--iq-tol",
        type=float,
        default=0.0,
        metavar="T",
        help="accept IQ pairs whose max abs real/imag delta is <= T "
        "(default 0.0 = byte-wise exact)",
    )
    parser.add_argument(
        "--strict-worker-id",
        action="store_true",
        help="also require worker_id equality (off by default: scheduling differs)",
    )
    parser.add_argument(
        "--float-tol",
        type=float,
        default=0.0,
        metavar="T",
        help="accept float functional fields within T (default 0.0 = exact; "
        "the frozen gate is exact equality)",
    )
    parser.add_argument(
        "--self-test",
        action="store_true",
        help="run the built-in synthetic self-test and exit",
    )
    return parser


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.self_test:
        return run_self_test()
    if not args.dump_a or not args.dump_b:
        parser.error("two dump files are required (or use --self-test)")
    for path in (args.dump_a, args.dump_b):
        if not os.path.isfile(path):
            print("ERROR: no such file: %s" % path, file=sys.stderr)
            return 2
    if args.iq_tol < 0.0:
        print("ERROR: --iq-tol must be >= 0", file=sys.stderr)
        return 2
    if args.float_tol < 0.0:
        print("ERROR: --float-tol must be >= 0", file=sys.stderr)
        return 2
    return run_cli(args)


if __name__ == "__main__":
    sys.exit(main())
