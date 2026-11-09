#!/usr/bin/env python3
"""Paired feasibility test for the fused PHR CMF + PN-despread path.

The driver runs the SAME binary twice.  ``old`` is the accepted fast-RX
baseline with payload fusion already enabled; ``new`` adds
``--rx-fused-phr-cmf`` (and its shorter necessary soft-chip prefix).  It does
not duplicate any DSP.  Formal runs take the exclusive
``/tmp/uwb-sic-benchmark.lock`` and remove ``LD_LIBRARY_PATH``.

Correctness and performance are reported separately.  A missing/empty run, a
source-window mismatch, a duplicate, a decoded PSDU/PHR mismatch, an
unexpected fused fallback, or a PHR fused kernel that never ran fails
correctness.  A slower or non-improving result is reported as a valid
"correct but not faster" conclusion and does not masquerade as PASS; it does
not change the exit code.

The paired timing statistics never include the FIR segmentation diagnostic.
A separate ``--rx-diagnose-fir-segments`` pass (old args) is used only for the
reported ``phr_path_with_diag`` comparison.  The whole prefix shrink is also
reported separately and must not be attributed to the PHR kernel.
"""
from __future__ import annotations

import argparse
import csv
import datetime
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import statistics
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
LOCK = Path("/tmp/uwb-sic-benchmark.lock")

DEFAULT_CELLS = ["payload_base", "payload_four_frac", "payload_cfo_m50k",
                 "payload_cfo_p50k", "payload_len127", "payload_snr8"]
TIMING_CELLS = ["payload_base", "payload_four_frac", "payload_len127"]

# OLD: accepted fast-RX baseline; payload fusion is part of the baseline.
BASE_ARGS = ["--rx-fast-sfd", "--rx-fast-cfo", "--rx-seed-refine-tail",
             "--rx-softchip-on-demand", "--rx-fused-payload-cmf"]
# NEW: add the fused PHR kernel and its shorter necessary soft-chip prefix.
PHR_ARGS = BASE_ARGS + ["--rx-fused-phr-cmf"]
# Diagnostic-only replay; never used by the paired timing runs.
DIAG_ARGS = BASE_ARGS + ["--rx-diagnose-fir-segments"]

FUNCTIONAL_FIELDS = (
    "comm_fcs", "comm_status", "comm_psdu_hex", "comm_phr_length",
    "comm_phr_bits", "comm_phr_secded_corrected",
    "comm_phr_secded_uncorrectable", "sensing_fcs", "sensing_status",
    "sensing_psdu_hex", "committed", "reason",
)

TIMING_METRICS = (
    "comm_demod", "sic_total", "stage_cir", "stage_phr", "stage_payload",
    "phr_path", "payload_path", "prefix_fir", "soft_extend",
    "phr_fused_template", "phr_fused_correlate", "phr_fused_decode",
    "phr_fused_decode_ns", "fused_template", "fused_correlate",
    "fused_decode", "fused_decode_ns", "chips",
    "chips_initial",
)

DIAG_TIMERS = (
    "diag_fir_preamble_us", "diag_fir_sfd_us", "diag_fir_phr_us",
    "diag_fir_payload_us", "diag_fir_extra_us",
)
DIAG_CHIPS = (
    "diag_fir_preamble_chips", "diag_fir_sfd_chips", "diag_fir_phr_chips",
    "diag_fir_payload_chips", "diag_fir_extra_chips",
)

MAX_PROBLEMS = 200
MISSING = object()


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


def utc_now() -> str:
    return datetime.datetime.now(datetime.timezone.utc).isoformat()


def git_output(*args: str) -> str:
    return subprocess.run(["git", "-C", str(ROOT), *args], text=True,
                          stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                          check=True).stdout.strip()


def clean_env() -> dict:
    env = os.environ.copy()
    env.pop("LD_LIBRARY_PATH", None)
    env["OMP_NUM_THREADS"] = "1"
    env["OPENBLAS_NUM_THREADS"] = "1"
    env["MKL_NUM_THREADS"] = "1"
    return env


def write_json(path: Path, value) -> None:
    path.write_text(json.dumps(value, indent=2, sort_keys=True,
                               allow_nan=False) + "\n")


def check_library(binary: Path, build: Path, env: dict) -> dict:
    completed = subprocess.run(["ldd", str(binary)], env=env, text=True,
                               stdout=subprocess.PIPE,
                               stderr=subprocess.STDOUT, check=True)
    libraries = []
    for line in completed.stdout.splitlines():
        if "libgnuradio-uwb" in line and "=>" in line:
            candidate = Path(line.split("=>", 1)[1].strip().split()[0]).resolve()
            if not candidate.is_relative_to(build.resolve()):
                raise RuntimeError(
                    f"runner loaded uwb library outside build: {candidate}")
            libraries.append({"path": str(candidate), "sha256": sha256(candidate)})
    if not libraries:
        raise RuntimeError("ldd found no uwb library")
    return {"ldd": completed.stdout, "libraries": libraries}


def load_rows(run: Path) -> list[dict]:
    path = run / "windows.jsonl"
    if not path.is_file():
        raise RuntimeError(f"missing {path}")
    rows = [json.loads(line) for line in path.read_text().splitlines()
            if line.strip()]
    if not rows:
        raise RuntimeError(f"runner produced no rows: {run}")
    return rows


def run_variant(runner: Path, case: Path, out: Path, env: dict, repeats: int,
                warmup: int, extra: list[str],
                reconstruction: str) -> tuple[dict, list[dict]]:
    if out.exists():
        raise RuntimeError(f"refusing to overwrite {out}")
    out.mkdir(parents=True)
    command = [str(runner), "--case", str(case), "--out", str(out),
               "--mode", "full", "--reconstruction", reconstruction,
               "--repeat", str(repeats), "--warmup", str(warmup)] + extra
    with (out / "runner.log").open("w") as log:
        started = utc_now()
        completed = subprocess.run(command, cwd=ROOT, env=env, text=True,
                                   stdout=log, stderr=subprocess.STDOUT)
    record = {"argv": command, "returncode": completed.returncode,
              "started_utc": started, "finished_utc": utc_now(),
              "repeats": repeats, "warmup": warmup, "extra_args": extra}
    write_json(out / "command.json", record)
    if completed.returncode != 0:
        raise RuntimeError(f"runner failed ({completed.returncode}): {out}")
    rows = load_rows(out)
    if len(rows) != repeats:
        raise RuntimeError(f"{out}: expected {repeats} rows, got {len(rows)}")
    return record, rows


def p95(values: list[float]) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    index = min(len(ordered) - 1,
                max(0, int(round(0.95 * (len(ordered) - 1)))))
    return ordered[index]


def stats(values: list[float]) -> dict:
    return {"mean": statistics.fmean(values) if values else 0.0,
            "p95": p95(values), "n": len(values)}


def path_value(row: dict, *keys: str, default=MISSING):
    value = row
    for key in keys:
        if not isinstance(value, dict) or key not in value:
            return default
        value = value[key]
    return value


def as_int(value):
    if value is MISSING or value is None:
        return None
    try:
        return int(value)
    except (TypeError, ValueError, OverflowError):
        return None


def as_float(value):
    if value is MISSING or value is None:
        return None
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError):
        return None
    return result if math.isfinite(result) else None


def rx_alias(rx: dict, *names: str):
    for name in names:
        if name in rx:
            return rx[name]
    return MISSING


def diag_lookup(row: dict, *names: str):
    rx = row.get("rx_diag")
    if not isinstance(rx, dict):
        return MISSING
    return rx_alias(rx, *names)


def diag_int(row: dict, *names: str, default: int = 0) -> int:
    value = as_int(diag_lookup(row, *names))
    return default if value is None else value


def diag_float(row: dict, *names: str, default: float = 0.0) -> float:
    value = as_float(diag_lookup(row, *names))
    return default if value is None else value


def metric_value(row: dict, metric: str, variant: str) -> float:
    def number(value) -> float:
        parsed = as_float(value)
        return 0.0 if parsed is None else parsed

    if metric == "comm_demod":
        return number(path_value(row, "times_us", "comm_demod"))
    if metric == "sic_total":
        return number(path_value(row, "times_us", "sic_total"))
    if metric == "stage_cir":
        return number(path_value(row, "comm_stage_us", "cir"))
    if metric == "stage_phr":
        return number(path_value(row, "comm_stage_us", "phr"))
    if metric == "stage_payload":
        return number(path_value(row, "comm_stage_us", "payload"))
    if metric == "phr_path":
        if variant == "old":
            # Paired timing has no separate old PHR FIR; stage_phr alone.
            return number(path_value(row, "comm_stage_us", "phr"))
        return (diag_float(row, "phr_fused_template_us",
                           "stage_phr_fused_template_us") +
                diag_float(row, "phr_fused_correlate_us",
                           "stage_phr_fused_correlate_us") +
                diag_float(row, "phr_fused_decode_us",
                           "stage_phr_fused_decode_us"))
    if metric == "payload_path":
        payload = number(path_value(row, "comm_stage_us", "payload"))
        if variant == "old":
            return diag_float(row, "soft_extend_us") + payload
        return payload
    if metric == "prefix_fir":
        return diag_float(row, "soft_fir_us")
    if metric == "soft_extend":
        return diag_float(row, "soft_extend_us")
    if metric == "phr_fused_template":
        return diag_float(row, "phr_fused_template_us",
                          "stage_phr_fused_template_us")
    if metric == "phr_fused_correlate":
        return diag_float(row, "phr_fused_correlate_us",
                          "stage_phr_fused_correlate_us")
    if metric == "phr_fused_decode":
        return diag_float(row, "phr_fused_decode_us",
                          "stage_phr_fused_decode_us")
    if metric == "phr_fused_decode_ns":
        return diag_float(row, "phr_fused_decode_ns")
    if metric == "fused_template":
        return diag_float(row, "fused_template_us",
                          "stage_fused_template_us")
    if metric == "fused_correlate":
        return diag_float(row, "fused_correlate_us",
                          "stage_fused_correlate_us")
    if metric == "fused_decode":
        return diag_float(row, "fused_decode_us", "stage_fused_decode_us")
    if metric == "fused_decode_ns":
        return diag_float(row, "fused_decode_ns")
    if metric == "chips":
        return diag_float(row, "softchip_final")
    if metric == "chips_initial":
        return diag_float(row, "softchip_initial")
    raise KeyError(f"unknown timing metric {metric}")


def summarize(rows: list[dict], variant: str) -> dict:
    accepted = [row for row in rows if not row.get("rejected")]
    values = {metric: [metric_value(row, metric, variant)
                       for row in accepted]
              for metric in TIMING_METRICS}
    summary = {metric: stats(values[metric]) for metric in TIMING_METRICS}
    summary.update({
        "accepted": len(accepted),
        "rejected": len(rows) - len(accepted),
        "phr_fused_attempts": sum(
            diag_int(row, "phr_fused_attempts", "stage_phr_fused_attempts")
            for row in accepted),
        "phr_fused_fallbacks": sum(
            diag_int(row, "phr_fused_fallbacks", "stage_phr_fused_fallbacks")
            for row in accepted),
        "phr_fused_symbols": sum(
            diag_int(row, "phr_fused_symbols", "stage_phr_fused_symbols")
            for row in accepted),
        "fused_attempts": sum(
            diag_int(row, "fused_attempts", "stage_fused_attempts")
            for row in accepted),
        "fused_fallbacks": sum(
            diag_int(row, "fused_fallbacks", "stage_fused_fallbacks")
            for row in accepted),
        "fused_symbols": sum(
            diag_int(row, "fused_symbols", "stage_fused_symbols")
            for row in accepted),
        "softchip_fallbacks": sum(
            diag_int(row, "softchip_fallbacks") for row in accepted),
        "comm_fcs": sum(bool(row.get("comm_fcs")) for row in accepted),
        "sensing_fcs": sum(bool(row.get("sensing_fcs")) for row in accepted),
    })
    return summary


def timer_sum_problems(rx: dict, names: tuple[str, ...], comm_demod: float,
                       label: str) -> list[str]:
    values = []
    missing = []
    for name in names:
        value = as_float(rx_alias(rx, name, "stage_" + name))
        if value is None:
            missing.append(name)
        else:
            values.append(value)
    if missing:
        return [f"{label}: missing/invalid sub-timer(s) {', '.join(missing)}"]
    total = sum(values)
    if total > comm_demod:
        return [f"{label}: sub-timers {total:.3f} us exceed "
                f"comm_demod {comm_demod:.3f} us"]
    return []


def fused_row_problems(row: dict, variant: str, nominal: bool,
                       label: str) -> list[str]:
    problems = []
    rx = row.get("rx_diag")
    if not isinstance(rx, dict):
        return [f"{label}: missing rx_diag object"]

    initial = as_int(rx.get("softchip_initial", MISSING))
    final = as_int(rx.get("softchip_final", MISSING))
    if initial is None:
        problems.append(f"{label}: missing/invalid softchip_initial")
    if final is None:
        problems.append(f"{label}: missing/invalid softchip_final")
    if variant == "new" and initial is not None and final is not None and \
            final != initial:
        problems.append(
            f"{label}: softchip_final {final} != softchip_initial {initial} "
            "(payload extension observed)")

    payload_attempts = as_int(rx_alias(rx, "fused_attempts",
                                       "stage_fused_attempts"))
    payload_fallbacks = as_int(rx_alias(rx, "fused_fallbacks",
                                        "stage_fused_fallbacks"))
    if payload_attempts is None:
        problems.append(f"{label}: missing/invalid fused_attempts")
    elif payload_attempts != 1:
        problems.append(f"{label}: fused_attempts {payload_attempts} != 1")
    if payload_fallbacks is None:
        problems.append(f"{label}: missing/invalid fused_fallbacks")
    elif variant == "new" and payload_fallbacks != 0:
        problems.append(f"{label}: fused_fallbacks {payload_fallbacks} != 0")
    elif variant == "old" and nominal and payload_fallbacks != 0:
        problems.append(f"{label}: baseline fused_fallbacks "
                        f"{payload_fallbacks} != 0 on a nominal cell")

    phr_attempts = as_int(rx_alias(rx, "phr_fused_attempts",
                                   "stage_phr_fused_attempts"))
    phr_fallbacks = as_int(rx_alias(rx, "phr_fused_fallbacks",
                                    "stage_phr_fused_fallbacks"))
    if variant == "old":
        if phr_attempts is None:
            problems.append(f"{label}: missing/invalid phr_fused_attempts")
        elif phr_attempts != 0:
            problems.append(f"{label}: baseline phr_fused_attempts "
                            f"{phr_attempts} != 0")
        old_phr_fallbacks = as_int(rx_alias(rx, "phr_fused_fallbacks",
                                            "stage_phr_fused_fallbacks"))
        if old_phr_fallbacks is not None and old_phr_fallbacks != 0:
            problems.append(f"{label}: baseline phr_fused_fallbacks "
                            f"{old_phr_fallbacks} != 0")
    else:
        if phr_attempts is None:
            problems.append(f"{label}: missing/invalid phr_fused_attempts")
        elif phr_attempts != 1:
            problems.append(f"{label}: phr_fused_attempts "
                            f"{phr_attempts} != 1")
        if phr_fallbacks is None:
            problems.append(f"{label}: missing/invalid phr_fused_fallbacks")
        elif phr_fallbacks != 0 and nominal:
            problems.append(f"{label}: phr_fused_fallbacks "
                            f"{phr_fallbacks} != 0 on a nominal cell")
        symbols = as_int(rx_alias(rx, "phr_fused_symbols",
                                  "stage_phr_fused_symbols"))
        if symbols is None:
            problems.append(f"{label}: missing/invalid phr_fused_symbols")
        elif symbols != 21:
            problems.append(f"{label}: phr_fused_symbols {symbols} != 21")

    comm_demod = as_float(path_value(row, "times_us", "comm_demod"))
    if comm_demod is None:
        problems.append(f"{label}: missing/invalid comm_demod")
    else:
        payload_timers = ("fused_template_us", "fused_correlate_us",
                          "fused_decode_us")
        if variant == "new" or payload_attempts == 1:
            problems.extend(timer_sum_problems(
                rx, payload_timers, comm_demod, f"{label} payload fused"))
        if variant == "new":
            phr_timers = ("phr_fused_template_us", "phr_fused_correlate_us",
                          "phr_fused_decode_us")
            problems.extend(timer_sum_problems(
                rx, phr_timers, comm_demod, f"{label} PHR fused"))

    required = (
        ("sic_total", path_value(row, "times_us", "sic_total")),
        ("stage_cir", path_value(row, "comm_stage_us", "cir")),
        ("stage_phr", path_value(row, "comm_stage_us", "phr")),
        ("stage_payload", path_value(row, "comm_stage_us", "payload")),
        ("soft_fir_us", rx.get("soft_fir_us", MISSING)),
        ("soft_extend_us", rx.get("soft_extend_us", MISSING)),
    )
    for name, value in required:
        if as_float(value) is None:
            problems.append(f"{label}: missing/invalid {name}")
    return problems


def field_presence_problems(row: dict, label: str) -> list[str]:
    problems = []
    missing = [field for field in FUNCTIONAL_FIELDS if field not in row]
    if missing:
        problems.append(f"{label}: missing field(s) {', '.join(missing)}")
    if "comm_phr_bits" in row and not isinstance(row.get("comm_phr_bits"),
                                                list):
        problems.append(f"{label}: comm_phr_bits is not a list")
    if "comm_phr_length" in row and as_int(row.get("comm_phr_length")) is None:
        problems.append(f"{label}: comm_phr_length is not an integer")
    return problems


def index_by_source(rows: list[dict], label: str,
                    expected_sources: set[int],
                    problems: list[str]) -> dict[int, dict]:
    index = {}
    for row in rows:
        if row.get("rejected"):
            continue
        source = as_int(row.get("source_window"))
        if source is None:
            problems.append(f"{label}: missing/invalid source_window")
            continue
        if source in index:
            problems.append(f"{label}: duplicate source_window {source}")
            continue
        index[source] = row
    got = set(index)
    if got != expected_sources:
        problems.append(f"{label}: source_window set {sorted(got)} != "
                        f"expected {sorted(expected_sources)}")
    return index


def softchip_compare(old_index: dict[int, dict], new_index: dict[int, dict],
                     label: str) -> list[str]:
    problems = []
    for source in sorted(set(old_index) & set(new_index)):
        old_rx = old_index[source].get("rx_diag")
        new_rx = new_index[source].get("rx_diag")
        old_value = as_int(old_rx.get("softchip_initial")) \
            if isinstance(old_rx, dict) else None
        new_value = as_int(new_rx.get("softchip_initial")) \
            if isinstance(new_rx, dict) else None
        if old_value is None or new_value is None:
            continue  # already reported by fused_row_problems
        if new_value >= old_value:
            problems.append(
                f"{label} source {source}: new softchip_initial {new_value} "
                f"is not strictly less than old {old_value}")
    return problems


def timing_softchip_compare(old_rows: list[dict], new_rows: list[dict],
                            label: str) -> list[str]:
    def collect(rows: list[dict]) -> dict[int, list[int]]:
        groups: dict[int, list[int]] = {}
        for row in rows:
            source = as_int(row.get("source_window"))
            rx = row.get("rx_diag")
            value = as_int(rx.get("softchip_initial")) \
                if isinstance(rx, dict) else None
            if source is None or value is None:
                continue
            groups.setdefault(source, []).append(value)
        return groups

    old_groups = collect(old_rows)
    new_groups = collect(new_rows)
    problems = []
    for source in sorted(set(old_groups) & set(new_groups)):
        if max(new_groups[source]) >= min(old_groups[source]):
            problems.append(
                f"{label} source {source}: new softchip_initial "
                f"{max(new_groups[source])} is not strictly less than "
                f"old {min(old_groups[source])}")
    return problems


def functional_compare(old_rows: list[dict], new_rows: list[dict], cell: str,
                       windows: int, nominal: bool) -> dict:
    problems = []
    mismatches = []
    expected_sources = set(range(windows))

    for label, rows in (("old", old_rows), ("new", new_rows)):
        rejected = sum(1 for row in rows if row.get("rejected"))
        if rejected:
            problems.append(f"{label}: {rejected} rejected row(s)")

    old_index = index_by_source(old_rows, "old", expected_sources, problems)
    new_index = index_by_source(new_rows, "new", expected_sources, problems)
    if set(old_index) != set(new_index):
        problems.append("old/new source-window set mismatch")

    for label, index in (("old", old_index), ("new", new_index)):
        for source, row in index.items():
            problems.extend(field_presence_problems(
                row, f"{label} source {source}"))

    for source in sorted(set(old_index) & set(new_index)):
        for field in FUNCTIONAL_FIELDS:
            if field not in old_index[source] or field not in new_index[source]:
                continue
            old_value = old_index[source][field]
            new_value = new_index[source][field]
            if old_value != new_value:
                mismatches.append({"source_window": source, "field": field,
                                   "old": old_value, "new": new_value})

    for source, row in old_index.items():
        problems.extend(fused_row_problems(row, "old", nominal,
                                           f"old source {source}"))
    for source, row in new_index.items():
        problems.extend(fused_row_problems(row, "new", nominal,
                                           f"new source {source}"))
    problems.extend(softchip_compare(old_index, new_index, "functional"))

    accepted_new = list(new_index.values())
    phr_attempts = sum(diag_int(row, "phr_fused_attempts",
                                "stage_phr_fused_attempts")
                       for row in accepted_new)
    phr_fallbacks = sum(diag_int(row, "phr_fused_fallbacks",
                                 "stage_phr_fused_fallbacks")
                        for row in accepted_new)
    fused_attempts = sum(diag_int(row, "fused_attempts",
                                  "stage_fused_attempts")
                         for row in accepted_new)
    fused_fallbacks = sum(diag_int(row, "fused_fallbacks",
                                   "stage_fused_fallbacks")
                          for row in accepted_new)
    if phr_attempts == 0:
        problems.append("fused PHR kernel never attempted")
    if fused_attempts == 0:
        problems.append("fused payload kernel never attempted")

    return {
        "cell": cell,
        "passed": not problems and not mismatches,
        "nominal": nominal,
        "problems": problems[:MAX_PROBLEMS],
        "problem_count": len(problems),
        "windows": windows,
        "mismatches": mismatches,
        "mismatch_count": len(mismatches),
        "phr_fused_attempts": phr_attempts,
        "phr_fused_fallbacks": phr_fallbacks,
        "fused_attempts": fused_attempts,
        "fused_fallbacks": fused_fallbacks,
        "old_comm_fcs": sum(bool(row.get("comm_fcs"))
                            for row in old_index.values()),
        "new_comm_fcs": sum(bool(row.get("comm_fcs"))
                            for row in accepted_new),
        "old_sensing_fcs": sum(bool(row.get("sensing_fcs"))
                               for row in old_index.values()),
        "new_sensing_fcs": sum(bool(row.get("sensing_fcs"))
                               for row in accepted_new),
    }


def timing_protocol(runs: list[dict], repeats: int, windows: int,
                    nominal: bool) -> dict:
    problems = []
    old_rows = []
    new_rows = []
    expected_ids = list(range(repeats))
    expected_sources = {index % windows for index in range(repeats)}

    for run in runs:
        variant = run["variant"]
        label = f"{variant} {Path(run['dir']).name}"
        rows = run["rows"]
        accepted = [row for row in rows if not row.get("rejected")]
        rejected = len(rows) - len(accepted)
        if rejected:
            problems.append(f"{label}: {rejected} rejected row(s)")
        if variant == "old":
            old_rows.extend(accepted)
        else:
            new_rows.extend(accepted)

        ids = []
        for row in accepted:
            window_id = as_int(row.get("window_id"))
            source = as_int(row.get("source_window"))
            if window_id is None:
                problems.append(f"{label}: missing/invalid window_id")
            else:
                ids.append(window_id)
            if source is None:
                problems.append(f"{label}: missing/invalid source_window")
            elif window_id is not None and source != window_id % windows:
                problems.append(
                    f"{label}: window_id {window_id} maps to source {source}, "
                    f"expected {window_id % windows}")
        if sorted(ids) != expected_ids:
            problems.append(f"{label}: window_id set mismatch "
                            f"(expected 0..{repeats - 1})")
        if len(set(ids)) != len(ids):
            problems.append(f"{label}: duplicate window_id")
        got_sources = {as_int(row.get("source_window")) for row in accepted}
        got_sources.discard(None)
        if got_sources != expected_sources:
            problems.append(f"{label}: source_window set {sorted(got_sources)} "
                            f"!= expected {sorted(expected_sources)}")

        for row in accepted:
            problems.extend(fused_row_problems(
                row, variant, nominal,
                f"{label} source {row.get('source_window')}"))

    problems.extend(timing_softchip_compare(old_rows, new_rows, "timing"))
    if sum(diag_int(row, "phr_fused_attempts", "stage_phr_fused_attempts")
           for row in new_rows) == 0:
        problems.append("fused PHR kernel never attempted")
    if sum(diag_int(row, "fused_attempts", "stage_fused_attempts")
           for row in new_rows) == 0:
        problems.append("fused payload kernel never attempted")

    missing_fields = set()
    for row in old_rows + new_rows:
        missing_fields.update(field for field in FUNCTIONAL_FIELDS
                              if field not in row)
    if missing_fields:
        problems.append("missing functional field(s): "
                        + ", ".join(sorted(missing_fields)))

    # Timing-only runs must still fail correctness on any decoded-payload
    # mismatch.  Bind reference results by source_window from the first
    # accepted old observation (falling back to new if needed).
    ref_by_source = {}
    for row in old_rows:
        if row.get("rejected"):
            continue
        source = as_int(row.get("source_window"))
        if source is not None and source not in ref_by_source:
            ref_by_source[source] = row
    if not ref_by_source:
        for row in new_rows:
            if row.get("rejected"):
                continue
            source = as_int(row.get("source_window"))
            if source is not None and source not in ref_by_source:
                ref_by_source[source] = row

    decode_mismatches = []
    # Verify every observation across all rounds against the source_window reference.
    for run in runs:
        variant = run.get("variant", "unknown")
        round_idx = run.get("round")
        for row in run.get("rows", []):
            if row.get("rejected"):
                continue
            source = as_int(row.get("source_window"))
            window_id = as_int(row.get("window_id"))
            if source is None or source not in ref_by_source:
                continue
            ref_row = ref_by_source[source]
            for field in FUNCTIONAL_FIELDS:
                if field not in row or field not in ref_row:
                    continue
                if row[field] != ref_row[field]:
                    decode_mismatches.append({
                        "round": round_idx,
                        "variant": variant,
                        "window_id": window_id,
                        "source_window": source,
                        "field": field,
                        "old": ref_row[field],
                        "new": row[field],
                    })
    if decode_mismatches:
        problems.append(f"{len(decode_mismatches)} decoded-field mismatch(es) "
                        "between timing observations and source reference")

    return {
        "passed": not problems,
        "problems": problems[:MAX_PROBLEMS],
        "problem_count": len(problems),
        "decode_mismatch_count": len(decode_mismatches),
        "decode_mismatches": decode_mismatches[:MAX_PROBLEMS],
        "accepted_rows": {"old": len(old_rows), "new": len(new_rows)},
        "expected_repeats": repeats,
        "expected_sources": sorted(expected_sources),
    }


def ratio_of(new_stats: dict, old_stats: dict) -> float:
    return (new_stats["mean"] / old_stats["mean"]
            if old_stats["mean"] > 0 else 0.0)


def p95_ratio_of(new_stats: dict, old_stats: dict) -> float:
    return (new_stats["p95"] / old_stats["p95"]
            if old_stats["p95"] > 0 else 0.0)


def build_timing_entry(cell: str, runs: list[dict], repeats: int,
                       windows: int, nominal: bool, phr_target: float,
                       regression: float) -> dict:
    old_rows = [row for run in runs if run["variant"] == "old"
                for row in run["rows"] if not row.get("rejected")]
    new_rows = [row for run in runs if run["variant"] == "new"
                for row in run["rows"] if not row.get("rejected")]
    old_summary = summarize(old_rows, "old")
    new_summary = summarize(new_rows, "new")
    protocol = timing_protocol(runs, repeats, windows, nominal)

    # Kernel-only: old stage_phr (despread + FEC) vs the new template +
    # correlation + decode.  This deliberately EXCLUDES the old PHR soft-chip
    # FIR, so it cannot be the >=20% "PHR path" target; it is reported to make
    # the kernel substitution honest.  The primary target is
    # phr_path_with_diag below, which adds the separately measured old PHR FIR.
    phr_kernel_ratio = ratio_of(new_summary["phr_path"],
                                old_summary["phr_path"])
    phr_kernel_p95_ratio = p95_ratio_of(new_summary["phr_path"],
                                        old_summary["phr_path"])
    comm_mean_ratio = ratio_of(new_summary["comm_demod"],
                               old_summary["comm_demod"])
    comm_p95_ratio = p95_ratio_of(new_summary["comm_demod"],
                                  old_summary["comm_demod"])
    sic_mean_ratio = ratio_of(new_summary["sic_total"],
                              old_summary["sic_total"])
    sic_p95_ratio = p95_ratio_of(new_summary["sic_total"],
                                 old_summary["sic_total"])
    no_regression = (
        old_summary["comm_demod"]["mean"] > 0 and
        comm_mean_ratio <= regression and
        old_summary["sic_total"]["mean"] > 0 and
        sic_mean_ratio <= regression)

    return {
        "cell": cell,
        "nominal": nominal,
        "old": old_summary,
        "new": new_summary,
        "phr_kernel_only_mean_ratio": phr_kernel_ratio,
        "phr_kernel_only_p95_ratio": phr_kernel_p95_ratio,
        "phr_kernel_only_target_met": (
            old_summary["phr_path"]["mean"] > 0 and
            phr_kernel_ratio <= phr_target),
        # Primary PHR path ratio is filled from the diagnostic pass.  Until
        # then it mirrors the kernel-only value so a missing diagnostic cannot
        # silently look like a pass.
        "phr_path_mean_ratio": phr_kernel_ratio,
        "phr_path_p95_ratio": phr_kernel_p95_ratio,
        "payload_path_mean_ratio": ratio_of(new_summary["payload_path"],
                                            old_summary["payload_path"]),
        "comm_mean_ratio": comm_mean_ratio,
        "comm_p95_ratio": comm_p95_ratio,
        "sic_mean_ratio": sic_mean_ratio,
        "sic_p95_ratio": sic_p95_ratio,
        "phr_path_target_ratio_max": phr_target,
        "regression_mean_ratio_max": regression,
        "phr_path_target_met": False,
        "no_regression": no_regression,
        "paired_performance_target_met": False,
        "timing_protocol": protocol,
        "phr_path_with_diag": {
            "available": False,
            "reason": "diagnostic pass not run",
            "target_ratio_max": None,
            "target_met": False,
        },
        "rounds": [{"round": run["round"], "variant": run["variant"],
                    "dir": run["dir"], "summary": run["summary"]}
                   for run in runs],
    }


def diagnostic_summary(rows: list[dict], cell: str, directory: Path) -> dict:
    problems = []
    accepted = [row for row in rows if not row.get("rejected")]
    rejected = len(rows) - len(accepted)
    if rejected:
        problems.append(f"{rejected} rejected row(s)")

    missing_fields = set()
    invalid_values = set()
    invalid_rows = 0
    for index, row in enumerate(accepted):
        rx = row.get("rx_diag")
        if not isinstance(rx, dict):
            problems.append(f"row {index}: missing rx_diag object")
            invalid_rows += 1
            continue
        if as_int(rx.get("diag_fir_valid")) != 1:
            invalid_rows += 1
        for key in DIAG_TIMERS + DIAG_CHIPS:
            if key not in rx:
                missing_fields.add(key)
            elif as_float(rx.get(key)) is None:
                invalid_values.add(key)
    if invalid_rows:
        problems.append(f"{invalid_rows} row(s) with diag_fir_valid != 1")
    if missing_fields:
        problems.append("missing diagnostic field(s): "
                        + ", ".join(sorted(missing_fields)))
    if invalid_values:
        problems.append("invalid diagnostic value(s): "
                        + ", ".join(sorted(invalid_values)))

    def values(key: str) -> list[float]:
        result = []
        for row in accepted:
            rx = row.get("rx_diag")
            value = as_float(rx.get(key)) if isinstance(rx, dict) else None
            if value is not None:
                result.append(value)
        return result

    timers = {key: stats(values(key)) for key in DIAG_TIMERS}
    chips = {key: stats(values(key)) for key in DIAG_CHIPS}
    return {
        "cell": cell,
        "passed": not problems,
        "problems": problems[:MAX_PROBLEMS],
        "problem_count": len(problems),
        "windows": len(accepted),
        "rows": len(rows),
        "dir": str(directory),
        "stats": timers,
        "chips": chips,
        "diag_fir_valid": len(accepted) - invalid_rows,
        "missing_fields": sorted(missing_fields),
    }


def combine_phr_with_diag(timing_entry: dict, diagnostic: dict,
                          target: float) -> dict:
    old_stage = timing_entry["old"]["stage_phr"]["mean"]
    new_path = timing_entry["new"]["phr_path"]["mean"]
    phr_stats = diagnostic["stats"].get("diag_fir_phr_us",
                                        {"mean": 0.0, "p95": 0.0, "n": 0})
    diag_mean = phr_stats["mean"]
    old_combined = old_stage + diag_mean
    model_ratio = new_path / old_combined if old_combined > 0 else 0.0
    available = (diagnostic["passed"] and diagnostic["windows"] > 0 and
                 phr_stats["n"] == diagnostic["windows"])
    return {
        "cell": timing_entry["cell"],
        "available": available,
        "old_stage_phr_mean_us": old_stage,
        "old_diag_fir_phr_mean_us": diag_mean,
        "old_combined_mean_us": old_combined,
        "new_phr_path_mean_us": new_path,
        "mean_ratio": model_ratio,
        "target_ratio_max": target,
        "target_met": available and old_combined > 0 and \
        model_ratio <= target,
        "definition": (
            "old = diagnostic diag_fir_phr_us mean + paired old stage_phr "
            "mean; new = paired new phr_fused_template+correlate+decode mean"),
        "note": ("diagnostic only; these values are never mixed into the "
                 "paired timing statistics"),
    }


def window_dirs(case: Path) -> list[Path]:
    directory = case / "windows"
    if not directory.is_dir():
        raise RuntimeError(f"missing window directory: {directory}")
    windows = sorted(path for path in directory.iterdir()
                     if path.is_dir() and (path / "raw.cf32").is_file())
    if not windows:
        raise RuntimeError(f"no input windows: {directory}")
    return windows


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--build",
                        default="gr-uwb/build-sic-fullpacket-fused-phr")
    parser.add_argument("--runner",
                        default="apps/benchmark_fullpacket_compare_fast_rx")
    parser.add_argument("--inputs",
                        default="gr-uwb/build-sic-fullpacket/results/inputs16")
    parser.add_argument("--out", required=True)
    parser.add_argument("--mode", choices=["functional", "timing", "both"],
                        default="both")
    parser.add_argument("--cells", nargs="+", default=None)
    parser.add_argument("--timing-cells", nargs="+", default=None)
    parser.add_argument("--observations", type=int, default=128)
    parser.add_argument("--warmup", type=int, default=16)
    parser.add_argument("--rounds", type=int, default=2)
    parser.add_argument("--diagnostic-observations", type=int, default=16)
    parser.add_argument("--diagnostic-warmup", type=int, default=0)
    parser.add_argument("--reconstruction", default="discrete_cir",
                        choices=["full", "discrete_cir", "payload_cached"])
    parser.add_argument("--phr-target-ratio", type=float, default=0.80)
    parser.add_argument("--phr-with-diag-target-ratio", type=float,
                        default=0.80)
    parser.add_argument("--regression-ratio", type=float, default=1.05)
    parser.add_argument("--non-nominal-cells", nargs="*", default=[],
                        help="cells where a fused fallback is tolerated")
    args = parser.parse_args()

    if args.observations <= 0 or args.warmup < 0:
        raise RuntimeError("observations must be > 0 and warmup >= 0")
    if args.rounds < 1:
        raise RuntimeError("rounds must be >= 1")
    if args.diagnostic_observations <= 0 or args.diagnostic_warmup < 0:
        raise RuntimeError(
            "diagnostic observations must be > 0 and diagnostic warmup >= 0")

    build = (ROOT / args.build).resolve()
    runner = (build / args.runner).resolve()
    inputs = (ROOT / args.inputs).resolve()
    out = (ROOT / args.out).resolve()
    if out.exists():
        raise RuntimeError(f"output exists; choose a new run: {out}")
    if not runner.is_file():
        raise RuntimeError(f"missing runner: {runner}")

    cells = args.cells or DEFAULT_CELLS
    functional_cells = cells if args.mode in ("functional", "both") else []
    timing_cells = (args.timing_cells if args.timing_cells is not None
                    else [cell for cell in cells
                          if cell in TIMING_CELLS])
    if args.mode not in ("timing", "both"):
        timing_cells = []
    if args.mode in ("functional", "both") and not functional_cells:
        raise RuntimeError("functional mode requested but no cells")
    if args.mode in ("timing", "both") and not timing_cells:
        raise RuntimeError("timing mode requested but no timing cells")
    non_nominal = set(args.non_nominal_cells)
    unknown = non_nominal - set(functional_cells + timing_cells)
    if unknown:
        raise RuntimeError(f"non-nominal cell not selected: {sorted(unknown)}")

    run_cells = list(dict.fromkeys(functional_cells + timing_cells))
    window_counts = {}
    for cell in run_cells:
        case = inputs / cell
        if not (case / "public.json").is_file():
            raise RuntimeError(f"missing cell inputs: {case}")
        window_counts[cell] = len(window_dirs(case))

    out.mkdir(parents=True)
    env = clean_env()
    library = check_library(runner, build, env)
    flags_file = build / "apps/CMakeFiles" / (runner.name + ".dir") / "flags.make"
    if not flags_file.is_file():
        raise RuntimeError(f"missing compile flags: {flags_file}")
    status = git_output("status", "--porcelain=v1", "--untracked-files=normal")
    try:
        affinity = sorted(os.sched_getaffinity(0))
    except (AttributeError, OSError):
        affinity = []
    provenance = {
        "schema": "fused-phr-compare/1",
        "created_utc": utc_now(),
        "git_head": git_output("rev-parse", "HEAD"),
        "git_dirty": bool(status),
        "git_status": status.splitlines(),
        "build_dir": str(build),
        "runner": {"path": str(runner), "sha256": sha256(runner)},
        "compile_flags": {"path": str(flags_file),
                          "sha256": sha256(flags_file),
                          "content": flags_file.read_text()},
        "libraries": library["libraries"],
        "ldd": library["ldd"],
        "cpu": next((line.split(":", 1)[1].strip()
                     for line in Path("/proc/cpuinfo").read_text().splitlines()
                     if line.startswith("model name")), "unknown"),
        "platform": platform.platform(),
        "affinity": affinity,
        "inputs": {
            "path": str(inputs),
            "cells": {
                cell: {"public_sha256": sha256(inputs / cell / "public.json"),
                       "windows": window_counts[cell]}
                for cell in run_cells
            },
        },
        "base_args": BASE_ARGS,
        "fused_args": PHR_ARGS,
        "diagnostic_args": DIAG_ARGS,
        "reconstruction": args.reconstruction,
        "lock_path": str(LOCK),
        "ld_library_path_removed": True,
        "full_wait_policy": "whole_public_context",
        "sources": {str(path.relative_to(ROOT)): sha256(path)
                    for path in [
                        ROOT / "gr-uwb/apps/benchmark_fullpacket_compare.cc",
                        ROOT / "gr-uwb/apps/fullpacket_rx_pipeline.h",
                        ROOT / "gr-uwb/include/gnuradio/uwb/uwb_demod_core.h",
                        ROOT / "gr-uwb/include/gnuradio/uwb/uwb_demod_result.h",
                        ROOT / "gr-uwb/include/gnuradio/uwb/uwb_phy_profile.h",
                        ROOT / "gr-uwb/include/gnuradio/uwb/uwb_rx_fused_cmf.h",
                        ROOT / "gr-uwb/include/gnuradio/uwb/uwb_rx_fused_phr.h",
                        ROOT / "gr-uwb/lib/qa_uwb_rx_fused_cmf.cc",
                        ROOT / "gr-uwb/lib/qa_uwb_rx_fused_phr.cc",
                        ROOT / "gr-uwb/include/gnuradio/uwb/uwb_rx_softchip_opt.h",
                        Path(__file__).resolve(),
                    ] if path.is_file()},
    }
    matrix = inputs / "matrix.json"
    if matrix.is_file():
        provenance["inputs"]["matrix_sha256"] = sha256(matrix)
    write_json(out / "provenance.json", provenance)

    summary = {
        "schema": "fused-phr-compare/1",
        "created_utc": utc_now(),
        "mode": args.mode,
        "provenance": str(out / "provenance.json"),
        "build": str(build),
        "runner": str(runner),
        "cells": {},
        "functional": [],
        "timing": [],
        "diagnostic": [],
        "phr_path_with_diag": [],
        "definitions": {
            "phr_path": {
                "old": "stage_phr (paired; no separate old PHR FIR)",
                "new": ("phr_fused_template + phr_fused_correlate + "
                        "phr_fused_decode"),
            },
            "payload_path": {
                "old": "soft_extend + stage_payload",
                "new": "stage_payload",
            },
            "prefix_fir": "rx_diag.soft_fir_us",
            "chips": "rx_diag.softchip_final",
            "chips_initial": "rx_diag.softchip_initial",
            "phr_path_with_diag": {
                "old": ("diagnostic diag_fir_phr_us mean + paired old "
                        "stage_phr mean"),
                "new": "paired new phr_path mean",
                "note": ("diagnostic only; never mixed into paired timing "
                         "statistics; do not attribute the whole prefix "
                         "shrink to the PHR kernel"),
            },
        },
        "targets": {
            "phr_path_mean_ratio_max": args.phr_target_ratio,
            "phr_path_with_diag_mean_ratio_max":
                args.phr_with_diag_target_ratio,
            "comm_demod_mean_ratio_max": args.regression_ratio,
            "sic_total_mean_ratio_max": args.regression_ratio,
        },
        "timing_protocol": {
            "warmup": args.warmup,
            "observations": args.observations,
            "rounds": args.rounds,
            "variant_order": ("alternating old/new pairs; 2 rounds = ABBA "
                              "(old,new,new,old) per cell"),
            "mode": "full",
            "reconstruction": args.reconstruction,
            "diagnostic_observations": args.diagnostic_observations,
            "diagnostic_warmup": args.diagnostic_warmup,
            "diagnostic_args": DIAG_ARGS,
            "same_binary": True,
        },
    }

    with LOCK.open("w") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)

        if functional_cells:
            for cell in functional_cells:
                case = inputs / cell
                nominal = cell not in non_nominal
                old_dir = out / "functional" / cell / "old"
                new_dir = out / "functional" / cell / "new"
                old_record, old_rows = run_variant(
                    runner, case, old_dir, env, window_counts[cell], 0,
                    BASE_ARGS, args.reconstruction)
                new_record, new_rows = run_variant(
                    runner, case, new_dir, env, window_counts[cell], 0,
                    PHR_ARGS, args.reconstruction)
                result = functional_compare(old_rows, new_rows, cell,
                                            window_counts[cell], nominal)
                result["runs"] = {
                    "old": {"dir": str(old_dir),
                            "argv": old_record["argv"]},
                    "new": {"dir": str(new_dir),
                            "argv": new_record["argv"]},
                }
                summary["functional"].append(result)
                verdict = "PASS" if result["passed"] else "FAIL"
                print(f"correctness {cell}: {verdict} "
                      f"windows={result['windows']} "
                      f"mismatches={result['mismatch_count']} "
                      f"phr_fused={result['phr_fused_attempts']} "
                      f"phr_fallbacks={result['phr_fused_fallbacks']} "
                      f"payload_fused={result['fused_attempts']} "
                      f"payload_fallbacks={result['fused_fallbacks']}")

        if timing_cells:
            for cell in timing_cells:
                case = inputs / cell
                nominal = cell not in non_nominal
                runs = []
                order = []
                for round_index in range(args.rounds):
                    pair = ([("old", BASE_ARGS), ("new", PHR_ARGS)]
                            if round_index % 2 == 0 else
                            [("new", PHR_ARGS), ("old", BASE_ARGS)])
                    order.extend(pair)
                for index, (variant, extra) in enumerate(order):
                    run_dir = out / "timing" / cell / f"r{index + 1}_{variant}"
                    _, rows = run_variant(
                        runner, case, run_dir, env, args.observations,
                        args.warmup, extra, args.reconstruction)
                    runs.append({"round": index + 1, "variant": variant,
                                 "dir": str(run_dir),
                                 "summary": summarize(rows, variant),
                                 "rows": rows})
                entry = build_timing_entry(
                    cell, runs, args.observations, window_counts[cell],
                    nominal, args.phr_target_ratio, args.regression_ratio)
                summary["timing"].append(entry)
                print(
                    f"timing {cell}: phr_path "
                    f"{entry['old']['phr_path']['mean']:.1f} -> "
                    f"{entry['new']['phr_path']['mean']:.1f} us "
                    f"(mean ratio {entry['phr_path_mean_ratio']:.3f}), "
                    f"comm ratio {entry['comm_mean_ratio']:.3f}, "
                    f"sic ratio {entry['sic_mean_ratio']:.3f}, "
                    f"protocol={'PASS' if entry['timing_protocol']['passed'] else 'FAIL'}")

            timing_by_cell = {entry["cell"]: entry for entry in summary["timing"]}
            for cell in timing_cells:
                diag_dir = out / "diagnostic" / cell
                try:
                    _, rows = run_variant(
                        runner, inputs / cell, diag_dir, env,
                        args.diagnostic_observations,
                        args.diagnostic_warmup, DIAG_ARGS,
                        args.reconstruction)
                except RuntimeError as exc:
                    raise RuntimeError(
                        f"diagnostic pass failed for {cell} "
                        f"(runner rejected --rx-diagnose-fir-segments or "
                        f"failed): {exc}") from exc
                diagnostic = diagnostic_summary(rows, cell, diag_dir)
                summary["diagnostic"].append(diagnostic)
                timing_entry = timing_by_cell.get(cell)
                if timing_entry is not None:
                    combined = combine_phr_with_diag(
                        timing_entry, diagnostic,
                        args.phr_with_diag_target_ratio)
                    timing_entry["phr_path_with_diag"] = combined
                    # Primary target: old PHR FIR + despread/FEC vs new
                    # template + correlate + decode.  The kernel-only ratio
                    # remains available as phr_kernel_only_mean_ratio.
                    timing_entry["phr_path_mean_ratio"] = combined["mean_ratio"]
                    timing_entry["phr_path_target_met"] = combined["target_met"]
                    timing_entry["paired_performance_target_met"] = (
                        combined["target_met"] and
                        timing_entry["no_regression"])
                    summary["phr_path_with_diag"].append(combined)
                status = "PASS" if diagnostic["passed"] else "FAIL"
                phr_us = diagnostic["stats"]["diag_fir_phr_us"]["mean"]
                print(f"diagnostic {cell}: {status} "
                      f"windows={diagnostic['windows']} "
                      f"diag_fir_phr={phr_us:.1f} us")

        for entry in summary["timing"]:
            combined = entry.get("phr_path_with_diag", {})
            entry["performance_target_met"] = (
                entry["paired_performance_target_met"] and
                bool(combined.get("available")) and
                bool(combined.get("target_met")))

    functional_ok = all(item["passed"] for item in summary["functional"]) \
        if summary["functional"] else True
    timing_ok = all(item["timing_protocol"]["passed"]
                    for item in summary["timing"]) \
        if summary["timing"] else True
    diagnostic_ok = all(item["passed"] for item in summary["diagnostic"]) \
        if summary["diagnostic"] else True

    correctness_passed = functional_ok and timing_ok and diagnostic_ok
    if args.mode in ("functional", "both") and not summary["functional"]:
        correctness_passed = False
    if args.mode in ("timing", "both") and not summary["timing"]:
        correctness_passed = False
    if args.mode in ("timing", "both") and not summary["diagnostic"]:
        correctness_passed = False

    if summary["timing"]:
        phr_path_ok = all(item["phr_path_target_met"]
                          for item in summary["timing"])
        regression_ok = all(item["no_regression"]
                            for item in summary["timing"])
        kernel_only_ok = all(item["phr_kernel_only_target_met"]
                             for item in summary["timing"])
        performance_met = phr_path_ok and regression_ok
        performance_status = "MET" if performance_met else "NOT_MET"
    else:
        phr_path_ok = regression_ok = kernel_only_ok = False
        performance_met = False
        performance_status = "NOT_RUN"

    for cell in run_cells:
        functional = next((item for item in summary["functional"]
                           if item["cell"] == cell), None)
        timing = next((item for item in summary["timing"]
                       if item["cell"] == cell), None)
        diagnostic = next((item for item in summary["diagnostic"]
                           if item["cell"] == cell), None)
        summary["cells"][cell] = {
            "functional": None if functional is None
            else functional["passed"],
            "timing": None if timing is None
            else timing["timing_protocol"]["passed"],
            "diagnostic": None if diagnostic is None
            else diagnostic["passed"],
            "nominal": cell not in non_nominal,
        }

    summary["correctness"] = {
        "passed": correctness_passed,
        "functional": ("PASS" if functional_ok else "FAIL")
        if summary["functional"] else "NOT_RUN",
        "timing_protocol": ("PASS" if timing_ok else "FAIL")
        if summary["timing"] else "NOT_RUN",
        "diagnostic": ("PASS" if diagnostic_ok else "FAIL")
        if summary["diagnostic"] else "NOT_RUN",
    }
    summary["performance"] = {
        "target_met": performance_met,
        "status": performance_status,
        "phr_path_target_met": phr_path_ok,
        "phr_kernel_only_target_met": kernel_only_ok,
        "no_regression": regression_ok,
        "cells": {
            item["cell"]: {
                "phr_path_mean_ratio": item["phr_path_mean_ratio"],
                "phr_path_target_met": item["phr_path_target_met"],
                "phr_kernel_only_mean_ratio": item[
                    "phr_kernel_only_mean_ratio"],
                "phr_kernel_only_target_met": item[
                    "phr_kernel_only_target_met"],
                "phr_path_with_diag_mean_ratio": item.get(
                    "phr_path_with_diag", {}).get("mean_ratio"),
                "phr_path_with_diag_target_met": item.get(
                    "phr_path_with_diag", {}).get("target_met", False),
                "comm_mean_ratio": item["comm_mean_ratio"],
                "sic_mean_ratio": item["sic_mean_ratio"],
                "prefix_fir_mean_ratio": ratio_of(item["new"]["prefix_fir"],
                                                  item["old"]["prefix_fir"]),
                "no_regression": item["no_regression"],
            }
            for item in summary["timing"]
        },
        "attribution_note": (
            "phr_path_with_diag adds only the old PHR-segment FIR to the old "
            "stage_phr.  The prefix_fir shrink is reported separately and must "
            "not be attributed to the PHR kernel."),
    }
    summary["correctness_passed"] = correctness_passed
    summary["performance_target_met"] = performance_met
    write_json(out / "summary.json", summary)

    with (out / "timing.csv").open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["cell", "variant", "metric", "mean_us", "p95_us",
                         "n", "mean_ratio_vs_old", "p95_ratio_vs_old"])
        for item in summary["timing"]:
            for metric in TIMING_METRICS:
                old = item["old"][metric]
                new = item["new"][metric]
                mean_ratio = (new["mean"] / old["mean"]) \
                    if old["mean"] else 0.0
                p95_ratio = (new["p95"] / old["p95"]) \
                    if old["p95"] else 0.0
                writer.writerow([item["cell"], "old", metric,
                                 f"{old['mean']:.6f}", f"{old['p95']:.6f}",
                                 old["n"], "1.000000", "1.000000"])
                writer.writerow([item["cell"], "new", metric,
                                 f"{new['mean']:.6f}", f"{new['p95']:.6f}",
                                 new["n"], f"{mean_ratio:.6f}",
                                 f"{p95_ratio:.6f}"])

    with (out / "functional.csv").open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["cell", "passed", "windows", "mismatches",
                         "phr_fused_attempts", "phr_fused_fallbacks",
                         "fused_attempts", "fused_fallbacks",
                         "old_comm_fcs", "new_comm_fcs",
                         "old_sensing_fcs", "new_sensing_fcs"])
        for item in summary["functional"]:
            writer.writerow([item["cell"], item["passed"], item["windows"],
                             item["mismatch_count"],
                             item["phr_fused_attempts"],
                             item["phr_fused_fallbacks"],
                             item["fused_attempts"],
                             item["fused_fallbacks"],
                             item["old_comm_fcs"], item["new_comm_fcs"],
                             item["old_sensing_fcs"],
                             item["new_sensing_fcs"]])

    with (out / "diagnostic.csv").open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["cell", "passed", "windows",
                         "diag_fir_preamble_us", "diag_fir_sfd_us",
                         "diag_fir_phr_us", "diag_fir_payload_us",
                         "diag_fir_extra_us", "old_stage_phr_mean_us",
                         "old_phr_path_with_diag_us", "new_phr_path_us",
                         "mean_ratio", "target_ratio_max", "target_met"])
        combined_by_cell = {item["cell"]: item
                            for item in summary["phr_path_with_diag"]}
        for item in summary["diagnostic"]:
            combined = combined_by_cell.get(item["cell"], {})
            writer.writerow([
                item["cell"], item["passed"], item["windows"],
                f"{item['stats']['diag_fir_preamble_us']['mean']:.6f}",
                f"{item['stats']['diag_fir_sfd_us']['mean']:.6f}",
                f"{item['stats']['diag_fir_phr_us']['mean']:.6f}",
                f"{item['stats']['diag_fir_payload_us']['mean']:.6f}",
                f"{item['stats']['diag_fir_extra_us']['mean']:.6f}",
                f"{combined.get('old_stage_phr_mean_us', 0.0):.6f}",
                f"{combined.get('old_combined_mean_us', 0.0):.6f}",
                f"{combined.get('new_phr_path_mean_us', 0.0):.6f}",
                f"{combined.get('mean_ratio', 0.0):.6f}",
                f"{combined.get('target_ratio_max') or 0.0:.6f}",
                combined.get("target_met", False),
            ])

    functional_status = summary["correctness"]["functional"]
    timing_status = summary["correctness"]["timing_protocol"]
    diagnostic_status = summary["correctness"]["diagnostic"]
    print(f"correctness={'PASS' if correctness_passed else 'FAIL'} "
          f"(functional={functional_status}, "
          f"timing_protocol={timing_status}, "
          f"diagnostic={diagnostic_status})")
    print(f"performance_target={performance_status}")
    if summary["timing"]:
        print(f"targets: phr_path mean ratio <= {args.phr_target_ratio:.2f}; "
              f"phr_path_with_diag mean ratio <= "
              f"{args.phr_with_diag_target_ratio:.2f}; "
              f"comm/sic mean ratio <= {args.regression_ratio:.2f}")
        for item in summary["timing"]:
            combined = item.get("phr_path_with_diag", {})
            old_combined = combined.get("old_combined_mean_us", 0.0)
            new_combined = combined.get("new_phr_path_mean_us", 0.0)
            combined_ratio = combined.get("mean_ratio", 0.0)
            combined_flag = "MET" if combined.get("target_met") else "NOT_MET"
            print(
                f"  {item['cell']}: phr_kernel_only "
                f"{item['old']['phr_path']['mean']:.1f}->"
                f"{item['new']['phr_path']['mean']:.1f} us "
                f"ratio {item['phr_kernel_only_mean_ratio']:.3f} "
                f"({'MET' if item['phr_kernel_only_target_met'] else 'NOT_MET'}"
                f", excludes old PHR FIR); "
                f"phr_path_with_diag {old_combined:.1f}->"
                f"{new_combined:.1f} us ratio {combined_ratio:.3f} "
                f"({combined_flag}); "
                f"comm {item['comm_mean_ratio']:.3f}; "
                f"sic {item['sic_mean_ratio']:.3f}; "
                f"prefix_fir {item['old']['prefix_fir']['mean']:.1f}->"
                f"{item['new']['prefix_fir']['mean']:.1f} us")
        print(summary["performance"]["attribution_note"])
        if not performance_met:
            print("performance target NOT met "
                  "(this is not a correctness failure)")
    else:
        print("timing=NOT_RUN (performance target not evaluated)")
    return 0 if correctness_passed else 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as exc:  # noqa: BLE001 - CLI boundary
        print(f"error: {exc}", file=sys.stderr)
        sys.exit(1)
